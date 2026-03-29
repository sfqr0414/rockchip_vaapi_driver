#include "mpp_decoder.h"

#include <chrono>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <sys/stat.h>
#include <mutex>
#include <thread>
#include <vector>
#include <cassert>

#include <rockchip/mpp_log.h>

namespace rockchip {

static MppCodingType codecProfileToMpp(CodecProfile profile) {
    switch (profile) {
        case CodecProfile::H264: return MPP_VIDEO_CodingAVC;
        case CodecProfile::HEVC: return MPP_VIDEO_CodingHEVC;
        case CodecProfile::VP9: return MPP_VIDEO_CodingVP9;
        case CodecProfile::AV1: return MPP_VIDEO_CodingAV1;
        default: return MPP_VIDEO_CodingUnused;
    }
}

static uint32_t alignUp(uint32_t value, uint32_t align) {
    return (value + align - 1) / align * align;
}

static uint64_t steadyMicrosNow() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

static void* mapBufferFd(int fd, size_t size) {
    if (fd < 0 || size == 0) return nullptr;
    return mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
}

static void unmapBufferFd(void* ptr, size_t size) {
    if (ptr && size > 0) munmap(ptr, size);
}

static bool readExact(FILE* fp, uint8_t* data, size_t size) {
    size_t offset = 0;
    while (offset < size) {
        size_t bytes = fread(data + offset, 1, size - offset, fp);
        if (bytes == 0) return false;
        offset += bytes;
    }
    return true;
}

static void unpackPacked10RowLE(const uint8_t* src,
                                size_t src_bytes,
                                uint16_t* dst,
                                size_t sample_count) {
    const size_t block_count = (sample_count + 7) / 8;
    for (size_t block = 0; block < block_count; ++block) {
        const size_t src_index = block * 10;
        const size_t dst_index = block * 8;
        if (src_index + 10 > src_bytes) {
            for (size_t i = dst_index; i < sample_count; ++i) {
                dst[i] = 0;
            }
            return;
        }

        for (size_t sample = 0; sample < 8 && dst_index + sample < sample_count; ++sample) {
            uint16_t value = 0;
            const size_t bit_base = sample * 10;
            for (size_t bit = 0; bit < 10; ++bit) {
                const size_t bit_index = bit_base + bit;
                const size_t byte_index = src_index + (bit_index >> 3);
                const uint8_t bit_offset = static_cast<uint8_t>(bit_index & 7u);
                value |= static_cast<uint16_t>(((src[byte_index] >> bit_offset) & 0x1u) << bit);
            }
            dst[dst_index + sample] = static_cast<uint16_t>(value << 6);
        }
    }
}

static bool writeSurfaceP010(int fd,
                             uint32_t width,
                             uint32_t height,
                             uint32_t dst_stride,
                             uint32_t src_stride_bytes,
                             uint32_t src_ver_stride,
                             const uint8_t* src,
                             size_t src_size) {
    const size_t src_row_bytes = static_cast<size_t>(src_stride_bytes);
    const size_t packed_row_bytes = ((static_cast<size_t>(width) + 3) / 4) * 5;
    if (src_size < src_row_bytes * static_cast<size_t>(src_ver_stride)) return false;

    const size_t dst_row_bytes = static_cast<size_t>(dst_stride) * 2;
    const size_t aligned_h = ((height + 7) / 8) * 8;
    const size_t dst_size = dst_row_bytes * aligned_h * 3 / 2;
    std::vector<uint8_t> dst(dst_size, 0);

    const uint8_t* src_y = src;
    const uint8_t* src_uv = src + src_row_bytes * src_ver_stride;
    uint8_t* dst_y = dst.data();
    uint8_t* dst_uv = dst.data() + dst_row_bytes * aligned_h;

    for (uint32_t row = 0; row < height; ++row) {
        auto* dst_row = reinterpret_cast<uint16_t*>(dst_y + static_cast<size_t>(row) * dst_row_bytes);
        const uint8_t* src_row = src_y + static_cast<size_t>(row) * src_row_bytes;
        unpackPacked10RowLE(src_row, std::min(src_row_bytes, packed_row_bytes), dst_row, width);
    }

    for (uint32_t row = 0; row < height / 2; ++row) {
        auto* dst_row = reinterpret_cast<uint16_t*>(dst_uv + static_cast<size_t>(row) * dst_row_bytes);
        const uint8_t* src_row = src_uv + static_cast<size_t>(row) * src_row_bytes;
        unpackPacked10RowLE(src_row, std::min(src_row_bytes, packed_row_bytes), dst_row, width);
    }

    return pwrite(fd, dst.data(), dst.size(), 0) == static_cast<ssize_t>(dst.size());
}

static bool writeSurfaceNV12(int fd,
                             uint32_t width,
                             uint32_t height,
                             uint32_t dst_stride,
                             uint32_t src_stride_bytes,
                             uint32_t src_ver_stride,
                             const uint8_t* src,
                             size_t src_size) {
    const size_t src_row_bytes = static_cast<size_t>(src_stride_bytes);
    const size_t expected_size = src_row_bytes * static_cast<size_t>(src_ver_stride) * 3 / 2;
    if (src_size < expected_size) return false;

    const size_t dst_row_bytes = static_cast<size_t>(dst_stride);
    const size_t copy_row_bytes = static_cast<size_t>(width);
    const size_t aligned_h = height;
    const size_t dst_size = dst_row_bytes * aligned_h * 3 / 2;
    std::vector<uint8_t> dst(dst_size, 0);

    const uint8_t* src_y = src;
    const uint8_t* src_uv = src + src_row_bytes * src_ver_stride;
    uint8_t* dst_y = dst.data();
    uint8_t* dst_uv = dst.data() + dst_row_bytes * aligned_h;

    for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(dst_y + static_cast<size_t>(row) * dst_row_bytes,
                    src_y + static_cast<size_t>(row) * src_row_bytes,
                    copy_row_bytes);
    }

    for (uint32_t row = 0; row < height / 2; ++row) {
        std::memcpy(dst_uv + static_cast<size_t>(row) * dst_row_bytes,
                    src_uv + static_cast<size_t>(row) * src_row_bytes,
                    copy_row_bytes);
    }

    return pwrite(fd, dst.data(), dst.size(), 0) == static_cast<ssize_t>(dst.size());
}

static int createFallbackSurfaceFd(uint32_t width, uint32_t height, bool is_10bit) {
    uint64_t total_size = 0;
    uint32_t stride = ((width + 63) / 64) * 64;
    if (!is_10bit) {
        total_size = static_cast<uint64_t>(stride) * height * 3 / 2;
    } else {
        uint32_t aligned_h = ((height + 7) / 8) * 8;
        total_size = static_cast<uint64_t>(stride) * aligned_h * 3;
    }

    char path_template[] = "/tmp/rockchip-vaapi-XXXXXX";
    int fd = mkstemp(path_template);
    if (fd < 0) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp: createFallbackSurfaceFd mkstemp failed errno={} ({})", errno, std::strerror(errno));
        return -1;
    }
    unlink(path_template);
    if (ftruncate(fd, static_cast<off_t>(total_size)) != 0) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp: createFallbackSurfaceFd ftruncate failed errno={} ({})", errno, std::strerror(errno));
        close(fd);
        return -1;
    }

    void* ptr = mmap(nullptr, static_cast<size_t>(total_size), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr != MAP_FAILED) {
        memset(ptr, 0, static_cast<size_t>(total_size));
        munmap(ptr, static_cast<size_t>(total_size));
    }

    return fd;
}

MppDecoder::MppDecoder() = default;
MppDecoder::~MppDecoder() { shutdown(); }

bool MppDecoder::initialize(CodecProfile profile, int width, int height) {
    profile_ = profile;
    if (ctx_ && api_) return true;

    mpp_set_log_level(MPP_LOG_ERROR);

    if (mpp_create(&ctx_, &api_) != MPP_OK) return false;
    if (mpp_init(ctx_, MPP_CTX_DEC, codecProfileToMpp(profile_)) != MPP_OK) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }

    MppFrameFormat output_format = (profile_ == CodecProfile::AV1)
        ? MPP_FMT_YUV420SP_10BIT
        : MPP_FMT_YUV420SP;
    api_->control(ctx_, MPP_DEC_SET_OUTPUT_FORMAT, &output_format);

    MppDecCfg cfg = nullptr;
    if (mpp_dec_cfg_init(&cfg) == MPP_OK && cfg) {
        mpp_dec_cfg_set_u32(cfg, "base:split_parse", 1);
        api_->control(ctx_, MPP_DEC_SET_CFG, cfg);
        mpp_dec_cfg_deinit(cfg);
    }

    uint32_t split_parse = 1;
    api_->control(ctx_, MPP_DEC_SET_PARSER_SPLIT_MODE, &split_parse);

    MppPollType nonblock = MPP_POLL_NON_BLOCK;
    api_->control(ctx_, MPP_SET_OUTPUT_TIMEOUT, &nonblock);
    return true;
}

bool MppDecoder::isInitialized() const { return ctx_ != nullptr; }

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    if (profile_ == CodecProfile::AV1) {
        stride = alignUp(static_cast<uint32_t>(width), 128);
    }

    DecodedSurface ds;
    ds.va_id = id;
    ds.width = static_cast<uint32_t>(width);
    ds.height = static_cast<uint32_t>(height);
    ds.stride = stride;
    ds.is_10bit = (profile_ == CodecProfile::AV1);
    ds.dmabuf_fd = createFallbackSurfaceFd(ds.width, ds.height, ds.is_10bit);

    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto& entry = surfaces_[id];
    entry.surface = ds;
    entry.buffer = nullptr;
    entry.ready.store(true);
    entry.failed.store(false);
    out = ds;
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
    dmabuf_fd = it->second.surface.dmabuf_fd;
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
    if (!ctx_ || !api_) return false;

    job.job_id = next_job_id_.fetch_add(1, std::memory_order_relaxed);

    bool expected = false;
    if (running_.compare_exchange_strong(expected, true)) {
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
        const size_t max_in_flight = std::max<size_t>(1, std::min<size_t>(surface_count == 0 ? 1 : surface_count, 4));
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
    while (!ready_flag->load(std::memory_order_acquire) &&
           !failed_flag->load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

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

    if (it->second.surface.dmabuf_fd >= 0) {
        close(it->second.surface.dmabuf_fd);
        it->second.surface.dmabuf_fd = -1;
    }

    it->second.ready.store(false);
    it->second.failed.store(false);
}

void MppDecoder::destroySurface(VASurfaceID surface) {
    {
        std::lock_guard<std::mutex> lock(surface_mutex_);
        auto it = surfaces_.find(surface);
        if (it != surfaces_.end()) {
            if (it->second.buffer) {
                mpp_buffer_put(it->second.buffer);
                it->second.buffer = nullptr;
            }

            if (it->second.surface.dmabuf_fd >= 0) {
                close(it->second.surface.dmabuf_fd);
                it->second.surface.dmabuf_fd = -1;
            }

            surfaces_.erase(it);
        }
    }

    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_surfaces_.erase(
            std::remove(pending_surfaces_.begin(), pending_surfaces_.end(), surface),
            pending_surfaces_.end());
        pending_count_ = pending_surfaces_.size();
        pending_cv_.notify_all();
    }
}

void MppDecoder::shutdown() {
    if (!ctx_ && !api_) return;

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
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_cv_.notify_all();
    }

    if (input_thread_.joinable()) {
        input_thread_.request_stop();
        util::log(util::stderr_sink, util::LogLevel::Info, "joining input thread"); input_thread_.join(); util::log(util::stderr_sink, util::LogLevel::Info, "joined input thread");
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
        util::log(util::stderr_sink, util::LogLevel::Info, "joining output thread"); output_thread_.join(); util::log(util::stderr_sink, util::LogLevel::Info, "joined output thread");
    }

    {
        std::lock_guard<std::mutex> lock(surface_mutex_);
        for (auto& kv : surfaces_) {
            if (kv.second.buffer) {
                mpp_buffer_put(kv.second.buffer);
                kv.second.buffer = nullptr;
            }
            if (kv.second.surface.dmabuf_fd >= 0) {
                close(kv.second.surface.dmabuf_fd);
                kv.second.surface.dmabuf_fd = -1;
            }
        }
        surfaces_.clear();
    }

    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_surfaces_.clear();
        pending_payloads_.clear();
        pending_count_ = 0;
        pending_cv_.notify_all();
    }

    if (ctx_) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
    }

}

void MppDecoder::inputThreadMain(std::stop_token st) {
    while (!st.stop_requested() && (running_ || !eos_sent_)) {
        auto job_opt = input_queue_.pop();
        if (!job_opt) break;
        DecodeJob job = std::move(*job_opt);

        std::vector<uint8_t> payload;
        const uint8_t* payload_ptr = nullptr;
        size_t payload_size = 0;

        if (job.eos) {
            MppPacket pkt = nullptr;
            mpp_packet_init(&pkt, nullptr, 0);
            mpp_packet_set_eos(pkt);
            for (int i = 0; i < 200 && running_; ++i) {
                int ret = api_->decode_put_packet(ctx_, pkt);
                if (ret == MPP_OK) break;
                if (ret == MPP_ERR_BUFFER_FULL) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    continue;
                }
                break;
            }
            static std::atomic<int> eos_log_count{0};
            if (eos_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "mpp: inputThread EOS submitted");
            }
            mpp_packet_deinit(&pkt);
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

        MppPacket pkt = nullptr;
        if (mpp_packet_init(&pkt, const_cast<uint8_t*>(payload_ptr), payload_size) != MPP_OK) {
            continue;
        }

        if (job.job_id > 0) {
            mpp_packet_set_pts(pkt, static_cast<RK_S64>(job.job_id));
            mpp_packet_set_dts(pkt, static_cast<RK_S64>(job.job_id));
        }

        while (running_) {
            int ret = api_->decode_put_packet(ctx_, pkt);
            if (ret == MPP_OK) break;
            if (ret == MPP_ERR_BUFFER_FULL) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "mpp: decode_put_packet returned {}", ret);
            break;
        }

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_payloads_.push_back(std::make_shared<std::vector<uint8_t>>(std::move(payload)));
        }

        mpp_packet_deinit(&pkt);

        static std::atomic<int> input_log_count{0};
        if (input_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: inputThread submitted surface={} bytes={}",
                      job.target_surface, payload_size);
        }
    }
}

void MppDecoder::outputThreadMain(std::stop_token st) {
    while (!st.stop_requested() && (running_ || !eos_seen_)) {
        MppFrame frame = nullptr;
        if (running_ && !eos_sent_.load(std::memory_order_relaxed)) {
            const uint64_t now_us = steadyMicrosNow();
            const uint64_t last_us = last_enqueue_us_.load(std::memory_order_relaxed);
            const size_t pending_surfaces = pending_count_;
            if (last_us != 0 && pending_surfaces == 0 && now_us > last_us + 2'000'000ULL) {
                MppPacket eos_pkt = nullptr;
                mpp_packet_init(&eos_pkt, nullptr, 0);
                mpp_packet_set_eos(eos_pkt);
                for (int i = 0; i < 200 && running_; ++i) {
                    int ret = api_->decode_put_packet(ctx_, eos_pkt);
                    if (ret == MPP_OK) break;
                    if (ret == MPP_ERR_BUFFER_FULL) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        continue;
                    }
                    break;
                }
                mpp_packet_deinit(&eos_pkt);
                eos_sent_ = true;
            }
        }
        static std::atomic<int> get_log_count{0};
        if (get_log_count.fetch_add(1, std::memory_order_relaxed) < 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread calling decode_get_frame pending={} running={}",
                      pending_count_, running_.load() ? 1 : 0);
        }
        int ret = api_->decode_get_frame(ctx_, &frame);

        if (get_log_count.load(std::memory_order_relaxed) <= 5) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "mpp: outputThread decode_get_frame ret={} frame={}", ret, (void*)frame);
        }

        if (ret == MPP_ERR_DISPLAY_FULL) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (ret == MPP_ERR_TIMEOUT || (ret == MPP_OK && !frame)) {
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
            continue;
        }

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
            api_->control(ctx_, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
            mpp_frame_deinit(&frame);
            continue;
        }

        bool is_error = mpp_frame_get_errinfo(frame) || mpp_frame_get_discard(frame);
        bool is_eos_frame = mpp_frame_get_eos(frame);

        VASurfaceID surface_id = VA_INVALID_ID;
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            RK_S64 frame_pts = mpp_frame_get_pts(frame);
            if (frame_pts > 0) {
                auto it = pending_surface_pts_.find(static_cast<uint64_t>(frame_pts));
                if (it != pending_surface_pts_.end()) {
                    surface_id = it->second;
                    pending_surface_pts_.erase(it);
                    auto queue_it = std::find(pending_surfaces_.begin(), pending_surfaces_.end(), surface_id);
                    if (queue_it != pending_surfaces_.end())
                        pending_surfaces_.erase(queue_it);
                }
            }

            if (surface_id == VA_INVALID_ID && !pending_surfaces_.empty()) {
                surface_id = pending_surfaces_.front();
                pending_surfaces_.pop_front();
            }

            if (surface_id != VA_INVALID_ID && pending_count_ > 0) {
                --pending_count_;
                pending_cv_.notify_all();
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
                const uint32_t dst_stride = it->second.surface.stride;
                bool copy_ok = false;

                if (mpp_buf && it->second.surface.dmabuf_fd >= 0 && output_width > 0 && output_height > 0 && !is_error) {
                    const uint8_t* src_ptr = static_cast<const uint8_t*>(mpp_buffer_get_ptr(mpp_buf));
                    size_t src_size = mpp_buffer_get_size(mpp_buf);
                    std::vector<uint8_t> mapped_copy;
                    static std::atomic<int> copy_trace_count{0};
                    if (copy_trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
                        util::log(util::stderr_sink, util::LogLevel::Info,
                                  "mpp: copy frame surface={} fmt={} mode={} out_w={} out_h={} src_stride_px={} src_stride_bytes={} src_ver_stride={} src_size={}",
                                  surface_id, static_cast<uint32_t>(frame_fmt), frame_mode,
                                  output_width, output_height, src_stride_pixels, src_stride_bytes, src_ver_stride, src_size);
                    }
                    if (!src_ptr) {
                        int src_fd = mpp_buffer_get_fd(mpp_buf);
                        if (src_fd >= 0 && src_size > 0) {
                            void* mapped = mapBufferFd(src_fd, src_size);
                            if (mapped) {
                                src_ptr = static_cast<const uint8_t*>(mapped);
                                mapped_copy.assign(src_size, 0);
                                std::memcpy(mapped_copy.data(), src_ptr, src_size);
                                unmapBufferFd(mapped, src_size);
                                src_ptr = mapped_copy.data();
                            }
                        }
                    }

                    if (src_ptr) {
                        if (profile_ == CodecProfile::AV1) {
                            copy_ok = writeSurfaceP010(it->second.surface.dmabuf_fd,
                                                       output_width,
                                                       output_height,
                                                       dst_stride,
                                                       src_stride_bytes,
                                                       src_ver_stride,
                                                       src_ptr,
                                                       src_size);
                        } else {
                            copy_ok = writeSurfaceNV12(it->second.surface.dmabuf_fd,
                                                       output_width,
                                                       output_height,
                                                       dst_stride,
                                                       src_stride_bytes,
                                                       src_ver_stride,
                                                       src_ptr,
                                                       src_size);
                        }
                    }
                }

                if (is_error || !copy_ok) {
                    static std::atomic<int> err_log_count{0};
                    if (err_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
                        util::log(util::stderr_sink, util::LogLevel::Warn,
                                  "mpp: failed to copy decoded frame to surface={} width={} height={} stride={}",
                                  surface_id, output_width, output_height, src_stride_pixels);
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
                    it->second.surface.stride = dst_stride;
                    it->second.failed.store(false);
                    it->second.ready.store(true);
                    {
                        std::lock_guard<std::mutex> lock(cv_mutex_);
                        cv_.notify_all();
                    }
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (!pending_payloads_.empty()) {
                pending_payloads_.pop_front();
            }
        }

        if (is_eos_frame) {
            eos_seen_ = true;
        }

        mpp_frame_deinit(&frame);
    }
}

CodecProfile vaProfileToCodec(VAProfile profile) {
    if (profile == VAProfileH264High) return CodecProfile::H264;
    if (profile == VAProfileHEVCMain) return CodecProfile::HEVC;
    if (profile == VAProfileAV1Profile0) return CodecProfile::AV1;
    return CodecProfile::Unknown;
}

} // namespace rockchip
