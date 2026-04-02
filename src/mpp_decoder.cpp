#include "mpp_decoder.h"

#include <chrono>

#include <sys/stat.h>
#include <sys/sysmacros.h>

#include <mutex>
#include <thread>

#include <rockchip/mpp_log.h>

namespace rockchip {

MppDecoder::MppDecoder() { mpp_set_log_level(MPP_LOG_ERROR); }

MppDecoder::~MppDecoder() { shutdown(); }

bool MppDecoder::initialize(CodecProfile profile, int width, int height, int drm_fd) {
    std::scoped_lock lock(init_mutex_);
    profile_ = profile;
    if (session_initialized_) return true;

    struct stat stat_buf = {};
    if (drm_fd < 0 || fstat(drm_fd, &stat_buf) != 0 || !S_ISCHR(stat_buf.st_mode) || major(stat_buf.st_rdev) != 226 || minor(stat_buf.st_rdev) < 128) {
        return false;
    }

    session_ = MppSession::create();
    if (!session_) {
        return false;
    }

    MppCtx ctx = session_->ctx();
    MppApi* api = session_->api();

    if (mpp_init(ctx, MPP_CTX_DEC, codecProfileToMpp(profile_)) != MPP_OK) {
        session_.reset();
        return false;
    }

    MppFrameFormat output_format = (profile_ == CodecProfile::AV1 && kAv1ExportAsP010)
                                       ? MPP_FMT_YUV420SP_10BIT
                                       : MPP_FMT_YUV420SP;
    api->control(ctx, MPP_DEC_SET_OUTPUT_FORMAT, &output_format);

    MppDecCfg raw_cfg = nullptr;
    if (mpp_dec_cfg_init(&raw_cfg) == MPP_OK && raw_cfg) {
        MppDecCfgHandle cfg{raw_cfg};
        mpp_dec_cfg_set_u32(cfg.get(), "base:split_parse", 1);
        api->control(ctx, MPP_DEC_SET_CFG, cfg.get());
    }

    uint32_t split_parse = 1;
    api->control(ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, &split_parse);

    MppPollType nonblock = MPP_POLL_NON_BLOCK;
    api->control(ctx, MPP_SET_OUTPUT_TIMEOUT, &nonblock);
    session_initialized_ = true;
    return true;
}

bool MppDecoder::isInitialized() const { return session_initialized_; }

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    if (profile_ == CodecProfile::AV1 && kAv1ExportAsP010) {
        stride = alignUp(static_cast<uint32_t>(width), 128);
    }

    DecodedSurface ds;
    ds.va_id = id;
    ds.width = static_cast<uint32_t>(width);
    ds.height = static_cast<uint32_t>(height);
    ds.stride = stride;
    ds.is_10bit = (profile_ == CodecProfile::AV1) && kAv1ExportAsP010;
    auto dmabuf = createFallbackSurfaceFd(ds.width, ds.height, ds.is_10bit);
    if (!dmabuf) {
        return false;
    }
    ds.dmabuf_fd = dmabuf.value().get();

    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto& entry = surfaces_[id];
    entry.surface = ds;
    entry.dmabuf = std::move(dmabuf.value());
    entry.surface.dmabuf_fd = entry.dmabuf.get();
    entry.ready.store(true);
    entry.failed.store(false);
    out = entry.surface;
    return true;
}

bool MppDecoder::updateSurfaceResolution(VASurfaceID id, int width, int height) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;

    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    it->second.surface.width = static_cast<uint32_t>(width);
    it->second.surface.height = static_cast<uint32_t>(height);
    it->second.surface.stride = stride;
    return true;
}

bool MppDecoder::getSurfaceInfo(VASurfaceID id, uint32_t& width, uint32_t& height, uint32_t& stride, int& dmabuf_fd, bool& failed) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    width = it->second.surface.width;
    height = it->second.surface.height;
    stride = it->second.surface.stride;
    dmabuf_fd = it->second.dmabuf.get();
    failed = it->second.failed.load();
    return true;
}

bool MppDecoder::getSurfaceState(VASurfaceID id, bool& ready, bool& failed) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    ready = it->second.ready.load();
    failed = it->second.failed.load();
    return true;
}

bool MppDecoder::enqueueJob(DecodeJob job) {
    if (!session_) return false;

    job.job_id = next_job_id_.fetch_add(1, std::memory_order_relaxed);

    bool expected = false;
    if (running_.compare_exchange_strong(expected, true)) {
        eos_sent_.store(false, std::memory_order_relaxed);
        eos_seen_.store(false, std::memory_order_relaxed);
        last_progress_us_.store(steadyMicrosNow(), std::memory_order_relaxed);
        input_thread_ = std::jthread([this](std::stop_token st){ inputThreadMain(st); });
        output_thread_ = std::jthread([this](std::stop_token st){ outputThreadMain(st); });
    }

    if (job.target_surface != VA_INVALID_ID) {
        size_t surface_count = 0;
        {
            std::lock_guard<std::mutex> lock(surface_mutex_);
            surface_count = surfaces_.size();
        }

        std::unique_lock<std::mutex> lock(pending_mutex_);
        const size_t max_in_flight = std::max<size_t>(1, std::min<size_t>(surface_count == 0 ? kMaxInFlightJobs : surface_count, kMaxInFlightJobs));
        static std::atomic<int> enqueue_wait_log_count{0};
        while (pending_count_ >= max_in_flight && running_.load()) {
            if (enqueue_wait_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "mpp: enqueueJob waiting surface={} pending={} max_in_flight={}",
                          job.target_surface, pending_count_, max_in_flight);
            }
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            lock.lock();
        }
        pending_surfaces_.push_back(job.target_surface);
        pending_surface_pts_[job.job_id] = job.target_surface;
        ++pending_count_;
        pending_cv_.notify_all();
    }

    static std::atomic<int> enqueue_log_count{0};
    if (enqueue_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "mpp: enqueueJob surface={} eos={} pending={} input_running={}",
                  job.target_surface, job.eos ? 1 : 0, pending_count_, running_.load() ? 1 : 0);
    }

    input_queue_.push(std::move(job));
    last_enqueue_us_.store(steadyMicrosNow(), std::memory_order_relaxed);
    return true;
}

void MppDecoder::setSurfaceState(VASurfaceID surface, bool ready, bool failed) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;
    it->second.ready.store(ready, std::memory_order_release);
    it->second.failed.store(failed, std::memory_order_release);
    std::lock_guard<std::mutex> lock2(cv_mutex_);
    cv_.notify_all();
}

std::vector<VASurfaceID> MppDecoder::failPendingSurfaces(VASurfaceID preferred_surface) {
    std::vector<VASurfaceID> failed_surfaces;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (preferred_surface != VA_INVALID_ID) {
            failed_surfaces.push_back(preferred_surface);
            dropPendingSurfaceLocked(preferred_surface);
        }

        for (VASurfaceID surface : pending_surfaces_) {
            if (surface != VA_INVALID_ID &&
                std::find(failed_surfaces.begin(), failed_surfaces.end(), surface) == failed_surfaces.end()) {
                failed_surfaces.push_back(surface);
            }
        }

        pending_surfaces_.clear();
        pending_surface_pts_.clear();
        pending_payloads_.clear();
        pending_count_ = 0;
        pending_cv_.notify_all();
    }

    for (VASurfaceID surface : failed_surfaces) {
        setSurfaceState(surface, false, true);
    }
    return failed_surfaces;
}

bool MppDecoder::dropPendingSurfaceLocked(VASurfaceID surface) {
    bool removed = false;
    auto queue_it = std::find(pending_surfaces_.begin(), pending_surfaces_.end(), surface);
    if (queue_it != pending_surfaces_.end()) {
        pending_surfaces_.erase(queue_it);
        removed = true;
    }

    for (auto it = pending_surface_pts_.begin(); it != pending_surface_pts_.end();) {
        if (it->second == surface) {
            it = pending_surface_pts_.erase(it);
            removed = true;
        } else {
            ++it;
        }
    }

    if (removed && pending_count_ > 0) {
        --pending_count_;
    }

    if (removed && !pending_payloads_.empty()) {
        pending_payloads_.pop_front();
    }

    if (removed) {
        pending_cv_.notify_all();
    }
    return removed;
}

VASurfaceID MppDecoder::dropOldestPendingSurfaceLocked() {
    if (pending_surfaces_.empty()) return VA_INVALID_ID;
    const VASurfaceID surface = pending_surfaces_.front();
    dropPendingSurfaceLocked(surface);
    return surface;
}

bool MppDecoder::waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms) {
    std::atomic<bool>* ready_flag = nullptr;
    std::atomic<bool>* failed_flag = nullptr;
    {
        std::lock_guard<std::mutex> lock(surface_mutex_);
        auto it = surfaces_.find(surface);
        if (it == surfaces_.end()) return false;
        ready_flag = &it->second.ready;
        failed_flag = &it->second.failed;
    }

    if (!ready_flag || !failed_flag) return false;

    static std::atomic<int> wait_start_log_count{0};
    if (wait_start_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "mpp: waitSurfaceReady start surface={} timeout_ms={}", surface, timeout_ms);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(cv_mutex_);
    cv_.wait_until(lock, deadline, [&] {
        return ready_flag->load(std::memory_order_acquire) || failed_flag->load(std::memory_order_acquire);
    });

    const bool failed = failed_flag->load(std::memory_order_acquire);
    const bool ready = ready_flag->load(std::memory_order_acquire);
    static std::atomic<int> wait_end_log_count{0};
    if (wait_end_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "mpp: waitSurfaceReady end surface={} ready={} failed={}", surface, ready ? 1 : 0, failed ? 1 : 0);
    }

    if (failed) return false;
    return ready;
}

void MppDecoder::forceSurfaceReady(VASurfaceID surface) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;
    it->second.failed.store(false);
    it->second.ready.store(true);
    std::lock_guard<std::mutex> lock2(cv_mutex_);
    cv_.notify_all();
}

void MppDecoder::resetSurface(VASurfaceID surface) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;
    it->second.ready.store(false);
    it->second.failed.store(false);
    std::lock_guard<std::mutex> lock2(cv_mutex_);
    cv_.notify_all();
}

void MppDecoder::releaseSurface(VASurfaceID surface) {
    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) return;

    it->second.dmabuf.reset();
    it->second.surface.dmabuf_fd = -1;

    it->second.ready.store(false);
    it->second.failed.store(false);
}

void MppDecoder::destroySurface(VASurfaceID surface) {
    {
        std::lock_guard<std::mutex> lock(surface_mutex_);
        auto it = surfaces_.find(surface);
        if (it != surfaces_.end()) {
            it->second.frame.reset();
            it->second.buffer.reset();
            it->second.dmabuf.reset();
            it->second.surface.dmabuf_fd = -1;

            surfaces_.erase(it);
        }
    }

    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        dropPendingSurfaceLocked(surface);
    }
}

void MppDecoder::shutdown() {
    std::shared_ptr<MppSession> session;
    {
        std::scoped_lock lock(init_mutex_);
        if (!session_) {
            session_initialized_ = false;
            return;
        }
        session = session_;
    }

    running_ = false;

    if (!eos_sent_) {
        DecodeJob eos;
        eos.target_surface = VA_INVALID_ID;
        eos.eos = true;
        input_queue_.push(eos);
        eos_sent_ = true;
    }

    input_queue_.shutdown();
    {
        std::scoped_lock lock(pending_mutex_);
        pending_cv_.notify_all();
    }

    if (input_thread_.joinable()) {
        input_thread_.request_stop();
        util::log(util::stderr_sink, util::LogLevel::Info, "joining input thread");
        input_thread_.join();
        util::log(util::stderr_sink, util::LogLevel::Info, "joined input thread");
    }

    // Give the output thread a short window to drain any in-flight frames and
    // observe the EOS frame before we force it to stop.
    for (int i = 0; i < 500; ++i) {
        if (eos_seen_.load()) {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (pending_count_ == 0) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (output_thread_.joinable()) {
        output_thread_.request_stop();
        util::log(util::stderr_sink, util::LogLevel::Info, "joining output thread");
        output_thread_.join();
        util::log(util::stderr_sink, util::LogLevel::Info, "joined output thread");
    }

    {
        std::scoped_lock lock(surface_mutex_);
        for (auto& kv : surfaces_) {
            kv.second.frame.reset();
            kv.second.buffer.reset();
            kv.second.dmabuf.reset();
            kv.second.surface.dmabuf_fd = -1;
        }
        surfaces_.clear();
    }

    {
        std::scoped_lock lock(pending_mutex_);
        pending_surfaces_.clear();
        pending_surface_pts_.clear();
        pending_payloads_.clear();
        pending_count_ = 0;
        pending_cv_.notify_all();
    }

    if (MppApi* api = session->api(); api && session_initialized_) {
        api->reset(session->ctx());
    }

    {
        std::scoped_lock lock(init_mutex_);
        session_.reset();
        session_initialized_ = false;
    }
}

void MppDecoder::inputThreadMain(std::stop_token st) {
    std::shared_ptr<MppSession> session = session_;
    if (!session) {
        running_ = false;
        eos_seen_ = true;
        return;
    }
    MppCtx ctx = session->ctx();
    MppApi* api = session->api();

    while (!st.stop_requested() && (running_ || !eos_sent_)) {
        auto job_opt = input_queue_.pop();
        if (!job_opt) break;
        DecodeJob job = std::move(*job_opt);

        std::vector<uint8_t> payload;
        const uint8_t* payload_ptr = nullptr;
        size_t payload_size = 0;

        if (job.eos) {
            const bool submitted = submitDecoderEos(ctx, api, running_);
            static std::atomic<int> eos_log_count{0};
            if (eos_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "mpp: inputThread EOS submitted={}", submitted ? 1 : 0);
            }
            eos_sent_ = true;
            continue;
        }

        if (!job.extra_data.empty() && !job.bitstream.empty()) {
            payload.reserve(job.extra_data.size() + job.bitstream.size());
            payload.insert(payload.end(), job.extra_data.begin(), job.extra_data.end());
            payload.insert(payload.end(), job.bitstream.begin(), job.bitstream.end());
            payload_ptr = payload.data();
            payload_size = payload.size();
        } else if (!job.extra_data.empty()) {
            payload = std::move(job.extra_data);
            payload_ptr = payload.data();
            payload_size = payload.size();
        } else {
            payload = std::move(job.bitstream);
            payload_ptr = payload.data();
            payload_size = payload.size();
        }

        if (payload_size == 0) continue;

        MppPacket raw_packet = nullptr;
        if (mpp_packet_init(&raw_packet, const_cast<uint8_t*>(payload_ptr), payload_size) != MPP_OK) {
            continue;
        }
        MppPacketHandle packet{raw_packet};

        if (job.job_id > 0) {
            mpp_packet_set_pts(packet.get(), static_cast<RK_S64>(job.job_id));
            mpp_packet_set_dts(packet.get(), static_cast<RK_S64>(job.job_id));
        }

        bool packet_submitted = false;
        while (running_) {
            int ret = api->decode_put_packet(ctx, packet.get());
            if (ret == MPP_OK) {
                packet_submitted = true;
                break;
            }
            if (ret == MPP_ERR_BUFFER_FULL) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "mpp: decode_put_packet returned {}", ret);
            failPendingSurfaces(job.target_surface);
            running_ = false;
            eos_seen_ = true;
            break;
        }

        if (!packet_submitted) {
            break;
        }

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_payloads_.push_back(std::make_shared<std::vector<uint8_t>>(std::move(payload)));
        }

        static std::atomic<int> input_log_count{0};
        if (input_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: inputThread submitted surface={} bytes={}",
                      job.target_surface, payload_size);
        }
    }
}

void MppDecoder::outputThreadMain(std::stop_token st) {
    std::shared_ptr<MppSession> session = session_;
    if (!session) {
        running_ = false;
        eos_seen_ = true;
        return;
    }
    MppCtx ctx = session->ctx();
    MppApi* api = session->api();

    while (!st.stop_requested() && (running_ || !eos_seen_)) {
        MppFrame frame = nullptr;
        if (running_ && profile_ != CodecProfile::AV1 && !eos_sent_.load(std::memory_order_relaxed)) {
            const uint64_t now_us = steadyMicrosNow();
            const uint64_t last_us = last_enqueue_us_.load(std::memory_order_relaxed);
            const size_t pending_surfaces = pending_count_;
            if (last_us != 0 && pending_surfaces > 0 && now_us > last_us + kDecoderFlushIdleMicros) {
                const bool submitted = submitDecoderEos(ctx, api, running_);
                if (submitted) {
                    static std::atomic<int> idle_eos_log_count{0};
                    if (idle_eos_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                        util::log(util::stderr_sink, util::LogLevel::Info,
                                  "mpp: outputThread submitted idle EOS pending={}", pending_surfaces);
                    }
                    eos_sent_ = true;
                }
            }
        }
        static std::atomic<int> get_log_count{0};
        if (get_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread calling decode_get_frame pending={} running={}",
                      pending_count_, running_.load() ? 1 : 0);
        }
        int ret = api->decode_get_frame(ctx, &frame);

        if (get_log_count.load(std::memory_order_relaxed) <= 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread decode_get_frame ret={} frame={}", ret, static_cast<const void*>(frame));
        }

        if (ret == MPP_ERR_DISPLAY_FULL) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (ret == MPP_ERR_TIMEOUT || (ret == MPP_OK && !frame)) {
            if (profile_ != CodecProfile::AV1 && !eos_sent_.load(std::memory_order_relaxed)) {
                const uint64_t now_us = steadyMicrosNow();
                const uint64_t last_enqueue = last_enqueue_us_.load(std::memory_order_relaxed);
                size_t pending_surfaces = 0;
                {
                    std::lock_guard<std::mutex> lock(pending_mutex_);
                    pending_surfaces = pending_count_;
                }

                if (pending_surfaces > 0 && last_enqueue != 0 && now_us > last_enqueue + kDecoderFlushIdleMicros) {
                    const bool submitted = submitDecoderEos(ctx, api, running_);
                    if (submitted) {
                        static std::atomic<int> timeout_eos_log_count{0};
                        if (timeout_eos_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                            util::log(util::stderr_sink, util::LogLevel::Info,
                                      "mpp: timeout path submitted idle EOS pending={}", pending_surfaces);
                        }
                        eos_sent_ = true;
                        continue;
                    }
                }
            }

            if (eos_sent_.load(std::memory_order_relaxed)) {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                if (pending_count_ == 0 && pending_surfaces_.empty()) {
                    eos_seen_ = true;
                    running_ = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (ret != MPP_OK || !frame) {
            if (ret != MPP_OK) {
                util::log(util::stderr_sink, util::LogLevel::Error,
                          "mpp: decode_get_frame returned {}", ret);
                failPendingSurfaces();
                running_ = false;
                eos_seen_ = true;
                break;
            }
            continue;
        }
        MppFrameHandle frame_handle{frame};

        static std::atomic<int> output_log_count{0};
        if (output_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread frame eos={} discard={} err={} pending={}",
                      mpp_frame_get_eos(frame) ? 1 : 0,
                      mpp_frame_get_discard(frame) ? 1 : 0,
                      mpp_frame_get_errinfo(frame) ? 1 : 0,
                      pending_count_);
        }

        if (mpp_frame_get_info_change(frame)) {
            uint32_t new_w = mpp_frame_get_width(frame);
            uint32_t new_h = mpp_frame_get_height(frame);
            std::lock_guard<std::mutex> lock(surface_mutex_);
            for (auto& item : surfaces_) {
                item.second.surface.width = new_w;
                item.second.surface.height = new_h;
            }
            api->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
            continue;
        }

        bool is_error = mpp_frame_get_errinfo(frame) || mpp_frame_get_discard(frame);
        bool is_eos_frame = mpp_frame_get_eos(frame);

        const MppBuffer frame_buffer = mpp_frame_get_buffer(frame);
        VASurfaceID surface_id = VA_INVALID_ID;

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (surface_id == VA_INVALID_ID) {
                RK_S64 frame_pts = mpp_frame_get_pts(frame);
                if (frame_pts > 0) {
                    auto it = pending_surface_pts_.find(static_cast<uint64_t>(frame_pts));
                    if (it != pending_surface_pts_.end()) {
                        surface_id = it->second;
                    }
                }
            }

            if (surface_id != VA_INVALID_ID) {
                dropPendingSurfaceLocked(surface_id);
            }
        }

        if (surface_id != VA_INVALID_ID) {
            std::lock_guard<std::mutex> lock(surface_mutex_);
            auto it = surfaces_.find(surface_id);
            if (it != surfaces_.end()) {
                const MppFrameFormat frame_fmt = mpp_frame_get_fmt(frame);
                const uint32_t frame_mode = mpp_frame_get_mode(frame);
                MppBuffer mpp_buf = mpp_frame_get_buffer(frame);
                const uint32_t output_width = mpp_frame_get_width(frame);
                const uint32_t output_height = mpp_frame_get_height(frame);
                const uint32_t src_stride_pixels = mpp_frame_get_hor_stride_pixel(frame);
                const uint32_t src_stride_bytes = mpp_frame_get_hor_stride(frame);
                const uint32_t src_ver_stride = mpp_frame_get_ver_stride(frame);
                const uint32_t zero_copy_stride = src_stride_pixels ? src_stride_pixels
                                                                    : (it->second.surface.is_10bit ? src_stride_bytes / 2 : src_stride_bytes);
                const int frame_fd = mpp_buf ? mpp_buffer_get_fd(mpp_buf) : -1;
                bool export_ok = false;

                if (mpp_buf && frame_fd >= 0 && output_width > 0 && output_height > 0 && !is_error) {
                    mpp_buffer_inc_ref(mpp_buf);
                    it->second.buffer.reset(mpp_buf);
                    it->second.dmabuf.reset(frame_fd >= 0 ? dup(frame_fd) : -1);
                    it->second.surface.dmabuf_fd = it->second.dmabuf.get();
                    export_ok = it->second.surface.dmabuf_fd >= 0;

                    static std::atomic<int> zero_copy_log_count{0};
                    if (export_ok && zero_copy_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                        util::log(util::stderr_sink, util::LogLevel::Info,
                                  "mpp: zero-copy frame surface={} fmt={} mode={} out_w={} out_h={} stride_px={} stride_bytes={} ver_stride={} fd={}",
                                  surface_id, static_cast<uint32_t>(frame_fmt), frame_mode,
                                  output_width, output_height, src_stride_pixels, src_stride_bytes, src_ver_stride, frame_fd);
                    }
                }

                if (is_error || !export_ok) {
                    static std::atomic<int> err_log_count{0};
                    if (err_log_count.fetch_add(1, std::memory_order_relaxed) < 4) {
                        util::log(util::stderr_sink, util::LogLevel::Warn,
                                  "mpp: failed to export decoded frame to surface={} width={} height={} stride={} fd={}",
                                  surface_id, output_width, output_height, src_stride_pixels, frame_fd);
                    }
                    it->second.failed.store(true);
                    it->second.ready.store(false);
                    {
                        std::lock_guard<std::mutex> lock(cv_mutex_);
                        cv_.notify_all();
                    }
                } else {
                    it->second.surface.width = output_width;
                    it->second.surface.height = output_height;
                    it->second.surface.stride = zero_copy_stride;
                    it->second.failed.store(false);
                    it->second.ready.store(true);
                    {
                        std::lock_guard<std::mutex> lock(cv_mutex_);
                        cv_.notify_all();
                    }
                }
            }
        }

        last_progress_us_.store(steadyMicrosNow(), std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (!pending_payloads_.empty()) {
                pending_payloads_.pop_front();
            }
        }

        if (is_eos_frame) {
            eos_seen_ = true;
        }
    }
}

} // namespace rockchip
