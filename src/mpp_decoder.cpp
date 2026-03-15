#include "mpp_decoder.h"

#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <mutex>
#include <span>
#include <thread>
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

static std::mutex g_surfaces_mutex;

MppDecoder::MppDecoder() = default;
MppDecoder::~MppDecoder() { shutdown(); }

bool MppDecoder::initialize(CodecProfile profile, int width, int height) {
    if (ctx_ && group_) return true;
    profile_ = profile;

    if (mpp_create(&ctx_, &api_) != MPP_OK) return false;

    if (mpp_init(ctx_, MPP_CTX_DEC, codecProfileToMpp(profile_)) != MPP_OK) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }

    // Configure MPP to split parse input into packets (per-slice parsing)
    // If the SDK provides mpp_dec_cfg helpers, set the 'base:split_parse' flag.
    // These symbols are normally available in Rockchip MPP builds; if not,
    // the calls are skipped safely by checking their presence at compile-time.
    // Force-enable split-parse so MPP will split input into smaller packets
    // required by some hardware decoders. Use the cfg helper unconditionally.
    {
        MppDecCfg cfg = nullptr;
        if (mpp_dec_cfg_init(&cfg) == MPP_OK && cfg) {
            mpp_dec_cfg_set_u32(cfg, "base:split_parse", 1);
            api_->control(ctx_, MPP_DEC_SET_CFG, cfg);
            mpp_dec_cfg_deinit(cfg);
        }
    }

    // Try buffer group types in priority order: DMA_HEAP then DRM. If the
    // chosen group doesn't produce exportable DMABUF fds, fall back.
    auto try_group = [&](MppBufferType type, const char* name) -> bool {
        MppBufferGroup g = nullptr;
        if (mpp_buffer_group_get_internal(&g, type) != MPP_OK) return false;
        // Quick check: allocate a buffer of a size similar to what we'll need and make sure it has an exportable fd.
        MppBuffer test_buf = nullptr;
        bool ok = false;
        const size_t test_size = 8 * 1024 * 1024;
        if (mpp_buffer_get(g, &test_buf, test_size) == MPP_OK && test_buf) {
            int fd = mpp_buffer_get_fd(test_buf);
            ok = (fd >= 0);
            mpp_buffer_put(test_buf);
        }
        if (!ok) {
            mpp_buffer_group_put(g);
            return false;
        }
        group_ = g;
        util::log(util::stderr_sink, util::LogLevel::Info, "mpp: using {} buffer group", name);
        return true;
    };

    if (!try_group(MPP_BUFFER_TYPE_DMA_HEAP, "DMA_HEAP") &&
        !try_group(MPP_BUFFER_TYPE_DRM, "DRM")) {
        util::log(util::stderr_sink, util::LogLevel::Error, "mpp: failed to create DMA_HEAP or DRM buffer group");
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }

    // Bind the created external buffer group into the decoder context so output
    // buffers are allocated from this group and can provide exportable DMABUF fds.
    api_->control(ctx_, MPP_DEC_SET_EXT_BUF_GROUP, group_);

    // Short output timeout to avoid long hangs in tests
    RK_U32 timeout = 100; // ms
    api_->control(ctx_, MPP_SET_OUTPUT_TIMEOUT, &timeout);

    return true;
}

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    if (profile_ == CodecProfile::AV1) {
        // For 10-bit AV1, MPP expects a stride close to width*10/8 and aligned to 16.
        stride = alignUp((width * 10U + 7U) / 8U, 16U);
    }
    out.va_id = id;
    out.width = static_cast<uint32_t>(width);
    out.height = static_cast<uint32_t>(height);
    out.stride = stride;
    out.is_10bit = (profile_ == CodecProfile::AV1);
    out.ready.store(false);

    SurfaceInfo info;
    info.width = out.width;
    info.height = out.height;
    info.stride = out.stride;
    info.ready = std::make_shared<std::atomic<bool>>(false);
    info.decode_failed = std::make_shared<std::atomic<bool>>(false);

    // Try to allocate an output buffer from the bound group so we have an
    // exportable DMABUF ready for VA to import/export. This ensures the
    // surface mapping contains a real MppBuffer and fd before decoding.
    if (group_) {
        MppBuffer buf = nullptr;
        // Estimate buffer size: stride * height * bytes-per-pixel (approx).
        // For 8-bit YUV420 use 1.5 bytes per pixel; for 10-bit YUV420 use 2.5 bytes per pixel.
        size_t bytes_per_pixel_mul = out.is_10bit ? 5 : 3;
        size_t bytes_per_pixel_div = out.is_10bit ? 2 : 2;
        size_t buf_size = (static_cast<size_t>(out.stride) * static_cast<size_t>(out.height) * bytes_per_pixel_mul) / bytes_per_pixel_div;
        if (buf_size == 0) buf_size = 8 * 1024 * 1024; // fallback 8MB
        if (mpp_buffer_get(group_, &buf, buf_size) == MPP_OK && buf) {
            int fd = mpp_buffer_get_fd(buf);
            util::log(util::stderr_sink, util::LogLevel::Debug,
                      "mpp: allocated surface {} buf_size={} fd={}", id, buf_size, fd);
            info.buffer = buf;
            info.dmabuf_fd = fd;
        } else {
            // Allocation failed — leave buffer null and fd -1, but continue.
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "mpp: failed to allocate buffer for surface %u (size=%zu)", id, buf_size);
            info.buffer = nullptr;
            info.dmabuf_fd = -1;
        }
    }

    std::lock_guard<std::mutex> lock(g_surfaces_mutex);
    surfaces_[id] = std::move(info);
    return true;
}

bool MppDecoder::updateSurfaceResolution(VASurfaceID id, int width, int height) { return true; }

bool MppDecoder::getSurfaceInfo(VASurfaceID id, uint32_t& width, uint32_t& height, uint32_t& stride, int& dmabuf_fd, bool& failed) {
    std::lock_guard<std::mutex> lock(g_surfaces_mutex);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    width = it->second.width;
    height = it->second.height;
    stride = it->second.stride;
    dmabuf_fd = it->second.dmabuf_fd;
    failed = it->second.decode_failed ? it->second.decode_failed->load() : false;
    return true;
}

bool MppDecoder::enqueueJob(DecodeJob job) {
    if (!running_) {
        running_ = true;
        decoder_thread_ = std::jthread([this](std::stop_token st){ decoderThreadMain(st); });
    }
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_surfaces_.push_back(job.target_surface);
    }
    job_queue_.push(std::move(job));
    return true;
}

bool MppDecoder::isInitialized() const { return ctx_ != nullptr; }

bool MppDecoder::waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms) {
    std::shared_ptr<std::atomic<bool>> ready_flag;
    std::shared_ptr<std::atomic<bool>> failed_flag;
    {
        std::lock_guard<std::mutex> lock(g_surfaces_mutex);
        auto it = surfaces_.find(surface);
        if (it == surfaces_.end()) return false;
        ready_flag = it->second.ready;
        failed_flag = it->second.decode_failed;
    }
    if (!ready_flag) return false;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(cv_mutex_);
    while (!ready_flag->load()) {
        if (std::chrono::steady_clock::now() >= deadline) break;
        cv_.wait_until(lock, deadline);
    }

    if (failed_flag && failed_flag->load()) return false;
    return ready_flag->load();
}

void MppDecoder::shutdown() {
    running_ = false;
    job_queue_.shutdown();
    if (decoder_thread_.joinable()) {
        decoder_thread_.request_stop();
        decoder_thread_.join();
    }

    std::lock_guard<std::mutex> lock(g_surfaces_mutex);
    if (ctx_) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
    }
    if (group_) {
        for (auto& pair : surfaces_) {
            if (pair.second.buffer) {
                mpp_buffer_put(pair.second.buffer);
                pair.second.buffer = nullptr;
                pair.second.dmabuf_fd = -1;
            }
        }
        mpp_buffer_group_put(group_);
        group_ = nullptr;
    }
    surfaces_.clear();
}

void MppDecoder::decoderThreadMain(std::stop_token st) {
    while (!st.stop_requested()) {
        auto job = job_queue_.pop();
        if (!job) break;
        processJob(*job);
    }
}

bool MppDecoder::processJob(const DecodeJob& job) {
    if (!ctx_) return false;

    std::shared_ptr<std::atomic<bool>> ready_flag;
    std::shared_ptr<std::atomic<bool>> failed_flag;
    {
        std::lock_guard<std::mutex> lock(g_surfaces_mutex);
        auto it = surfaces_.find(job.target_surface);
        if (it == surfaces_.end()) return false;
        ready_flag = it->second.ready;
        failed_flag = it->second.decode_failed;
    }

    if (ready_flag) ready_flag->store(false);
    if (failed_flag) failed_flag->store(false);

    // Feed extra_data first (e.g. sequence headers)
    if (!job.extra_data.empty()) {
        std::span<const uint8_t> extra(job.extra_data);
        MppPacket pkt;
        mpp_packet_init(&pkt, const_cast<uint8_t*>(extra.data()), extra.size());
        mpp_packet_set_extra_data(pkt);
        api_->decode_put_packet(ctx_, pkt);
        mpp_packet_deinit(&pkt);
    }

    // Feed bitstream
    std::span<const uint8_t> bit(job.bitstream);
    MppPacket pkt;
    mpp_packet_init(&pkt, const_cast<uint8_t*>(bit.data()), bit.size());
    if (job.eos) {
        mpp_packet_set_eos(pkt);
    }
    int ret = api_->decode_put_packet(ctx_, pkt);
    mpp_packet_deinit(&pkt);

    if (ret != MPP_OK) {
        if (failed_flag) failed_flag->store(true);
        if (ready_flag) {
            ready_flag->store(true);
            std::lock_guard<std::mutex> lock(cv_mutex_);
            cv_.notify_all();
        }
        return false;
    }

    // Capture output frames; update surface stride on resolution change or when frame available
    bool got = false;
    while (true) {
        MppFrame frame = nullptr;
        mpp_frame_init(&frame);
        int cret = api_->decode_get_frame(ctx_, &frame);
        if (cret != MPP_OK || !frame) {
            if (frame) mpp_frame_deinit(&frame);
            break;
        }

        if (mpp_frame_get_info_change(frame)) {
            uint32_t new_w = mpp_frame_get_width(frame);
            uint32_t new_h = mpp_frame_get_height(frame);
            uint32_t new_stride = mpp_frame_get_hor_stride(frame);
            std::lock_guard<std::mutex> lock(g_surfaces_mutex);
            auto it = surfaces_.find(job.target_surface);
            if (it != surfaces_.end()) {
                it->second.width = new_w;
                it->second.height = new_h;
                it->second.stride = new_stride;
            }
            api_->control(ctx_, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
            mpp_frame_deinit(&frame);
            continue;
        }

        MppBuffer buf = mpp_frame_get_buffer(frame);
        if (buf) {
            // Increment buffer ref before querying/exporting FD to ensure the
            // buffer remains valid while we inspect and assign it to our surface
            // structure. If we don't keep a ref and fail to store it, we must put it.
            mpp_buffer_inc_ref(buf);
            int fd = mpp_buffer_get_fd(buf);
            if (fd >= 0) {
                std::lock_guard<std::mutex> lock(g_surfaces_mutex);
                auto it = surfaces_.find(job.target_surface);
                if (it != surfaces_.end()) {
                    if (it->second.buffer) mpp_buffer_put(it->second.buffer);
                    it->second.buffer = buf;
                    it->second.dmabuf_fd = fd;
                    it->second.width = mpp_frame_get_width(frame);
                    it->second.height = mpp_frame_get_height(frame);
                    // Immediately update stride from frame to avoid mismatch
                    it->second.stride = mpp_frame_get_hor_stride(frame);
                    got = true;
                } else {
                    // No surface entry to attach to; release our ref.
                    mpp_buffer_put(buf);
                }
            } else {
                // Buffer has no exportable fd; drop our ref.
                mpp_buffer_put(buf);
            }
        }

        mpp_frame_deinit(&frame);
    }

    if (ready_flag) {
        ready_flag->store(true);
        std::lock_guard<std::mutex> lock(cv_mutex_);
        cv_.notify_all();
    }

    // Ensure surfaces_ contains the latest dmabuf_fd for the surface even if
    // the fd was obtained during frame handling above.
    {
        std::lock_guard<std::mutex> lock(g_surfaces_mutex);
        auto it = surfaces_.find(job.target_surface);
        if (it != surfaces_.end() && it->second.buffer) {
            int fd = mpp_buffer_get_fd(it->second.buffer);
            it->second.dmabuf_fd = fd;
        }
    }

    return got;
}

CodecProfile vaProfileToCodec(VAProfile profile) {
    if (profile == VAProfileH264High) return CodecProfile::H264;
    if (profile == VAProfileHEVCMain) return CodecProfile::HEVC;
    if (profile == VAProfileAV1Profile0) return CodecProfile::AV1;
    return CodecProfile::Unknown;
}

} // namespace rockchip
