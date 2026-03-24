#include "mpp_decoder.h"

#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <vector>
#include <cassert>

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

MppDecoder::MppDecoder() = default;
MppDecoder::~MppDecoder() { shutdown(); }

bool MppDecoder::initialize(CodecProfile profile, int width, int height) {
    if (ctx_ && api_) return true;
    profile_ = profile;

    if (mpp_create(&ctx_, &api_) != MPP_OK) return false;
    if (mpp_init(ctx_, MPP_CTX_DEC, codecProfileToMpp(profile_)) != MPP_OK) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }

    MppDecCfg cfg = nullptr;
    if (mpp_dec_cfg_init(&cfg) == MPP_OK && cfg) {
        mpp_dec_cfg_set_u32(cfg, "base:split_parse", 1);
        api_->control(ctx_, MPP_DEC_SET_CFG, cfg);
        mpp_dec_cfg_deinit(cfg);
    }

    MppPollType nonblock = MPP_POLL_NON_BLOCK;
    api_->control(ctx_, MPP_SET_OUTPUT_TIMEOUT, &nonblock);
    return true;
}

bool MppDecoder::isInitialized() const { return ctx_ != nullptr; }

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    if (profile_ == CodecProfile::AV1) {
        // AV1 in this path is treated as 10-bit surface, but mpp_buffer API is intentionally avoided.
        stride = alignUp(static_cast<uint32_t>(width), 128);
    }

    DecodedSurface ds;
    ds.va_id = id;
    ds.width = static_cast<uint32_t>(width);
    ds.height = static_cast<uint32_t>(height);
    ds.stride = stride;
    ds.is_10bit = (profile_ == CodecProfile::AV1);

    std::lock_guard<std::mutex> lock(surface_mutex_);
    auto& entry = surfaces_[id];
    entry.surface = ds;
    entry.buffer = nullptr;
    entry.ready.store(false);
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

bool MppDecoder::enqueueJob(DecodeJob job) {
    if (!ctx_ || !api_) return false;

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
        const size_t max_in_flight = surface_count == 0 ? 1 : surface_count;
        pending_cv_.wait(lock, [this, max_in_flight]() {
            return pending_count_ < max_in_flight || !running_.load();
        });
        job.token = next_token_++;
        pending_surfaces_[job.token] = job.target_surface;
        pending_order_.push_back(job.token);
        ++pending_count_;
        pending_cv_.notify_all();
    }

    input_queue_.push(std::move(job));
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

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(cv_mutex_);
    while (!ready_flag->load() && !failed_flag->load() && std::chrono::steady_clock::now() < deadline) {
        cv_.wait_until(lock, deadline);
    }

    if (failed_flag->load()) return false;
    return ready_flag->load();
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
        for (auto it = pending_surfaces_.begin(); it != pending_surfaces_.end();) {
            if (it->second == surface) {
                it = pending_surfaces_.erase(it);
            } else {
                ++it;
            }
        }
        pending_order_.erase(
            std::remove_if(pending_order_.begin(), pending_order_.end(), [this, surface](uint64_t token) {
                auto it = pending_surfaces_.find(token);
                return it == pending_surfaces_.end() || it->second == surface;
            }),
            pending_order_.end());
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
        input_thread_.join();
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
        output_thread_.join();
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
        pending_order_.clear();
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
            payload_ptr = job.extra_data.data();
            payload_size = job.extra_data.size();
        } else {
            payload_ptr = job.bitstream.data();
            payload_size = job.bitstream.size();
        }

        if (payload_size == 0) continue;

        MppPacket pkt = nullptr;
        if (mpp_packet_init(&pkt, const_cast<uint8_t*>(payload_ptr), payload_size) != MPP_OK) {
            continue;
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

        mpp_packet_deinit(&pkt);
    }

}

void MppDecoder::outputThreadMain(std::stop_token st) {
    while (!st.stop_requested() && (running_ || !eos_seen_)) {
        MppFrame frame = nullptr;
        int ret = api_->decode_get_frame(ctx_, &frame);

        if (ret == MPP_ERR_DISPLAY_FULL) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (ret == MPP_ERR_TIMEOUT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (ret != MPP_OK || !frame) {
            continue;
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
        uint64_t token = static_cast<uint64_t>(mpp_frame_get_pts(frame));
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            auto pending_it = pending_surfaces_.find(token);
            if (pending_it != pending_surfaces_.end()) {
                surface_id = pending_it->second;
                pending_surfaces_.erase(pending_it);
                pending_order_.erase(
                    std::remove(pending_order_.begin(), pending_order_.end(), token),
                    pending_order_.end());
                if (pending_count_ > 0) --pending_count_;
                pending_cv_.notify_all();
            } else if (!pending_order_.empty()) {
                uint64_t fallback_token = pending_order_.front();
                pending_order_.pop_front();
                auto fallback_it = pending_surfaces_.find(fallback_token);
                if (fallback_it != pending_surfaces_.end()) {
                    surface_id = fallback_it->second;
                    pending_surfaces_.erase(fallback_it);
                    if (pending_count_ > 0) --pending_count_;
                    pending_cv_.notify_all();
                }
            }
        }

        if (surface_id != VA_INVALID_ID) {
            std::lock_guard<std::mutex> lock(surface_mutex_);
            auto it = surfaces_.find(surface_id);
            if (it != surfaces_.end()) {
                it->second.surface.width = mpp_frame_get_width(frame);
                it->second.surface.height = mpp_frame_get_height(frame);
                it->second.surface.stride = mpp_frame_get_hor_stride(frame);

                MppBuffer mpp_buf = mpp_frame_get_buffer(frame);
                if (mpp_buf) {
                    int buffer_fd = mpp_buffer_get_fd(mpp_buf);
                    if (it->second.surface.dmabuf_fd >= 0) {
                        close(it->second.surface.dmabuf_fd);
                    }
                    it->second.surface.dmabuf_fd = (buffer_fd >= 0 ? dup(buffer_fd) : -1);
                }

                it->second.failed.store(is_error);
                it->second.ready.store(!is_error);
            }
        }

        {
            std::lock_guard<std::mutex> lock(cv_mutex_);
            cv_.notify_all();
        }

        if (is_eos_frame) {
            eos_seen_ = true;
            running_ = false;
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
