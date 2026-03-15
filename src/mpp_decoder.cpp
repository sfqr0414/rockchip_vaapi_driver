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
#if defined(HAVE_MPP_DEC_CFG)
    {
        MppDecCfg cfg = nullptr;
        if (mpp_dec_cfg_init(&cfg) == MPP_OK && cfg) {
            mpp_dec_cfg_set_u32(cfg, "base:split_parse", 1);
            api_->control(ctx_, MPP_DEC_SET_CFG, cfg);
            mpp_dec_cfg_deinit(cfg);
        }
    }
#endif

    if (mpp_buffer_group_get_internal(&group_, MPP_BUFFER_TYPE_DMA_HEAP) != MPP_OK) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }

    // Short output timeout to avoid long hangs in tests
    RK_U32 timeout = 100; // ms
    api_->control(ctx_, MPP_SET_OUTPUT_TIMEOUT, &timeout);

    return true;
}

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    uint32_t stride = alignUp(static_cast<uint32_t>(width), 64);
    if (profile_ == CodecProfile::AV1) {
        stride = alignUp((width * 10U + 7U) / 8U, 256U) + 256U;
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
    job_queue_.push(std::move(job));
    return true;
}

bool MppDecoder::isInitialized() const { return ctx_ != nullptr; }

bool MppDecoder::waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms) {
    std::shared_ptr<std::atomic<bool>> ready_flag;
    {
        std::lock_guard<std::mutex> lock(g_surfaces_mutex);
        auto it = surfaces_.find(surface);
        if (it == surfaces_.end()) return false;
        ready_flag = it->second.ready;
    }
    if (!ready_flag) return false;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!ready_flag->load()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
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
            if (pair.second.buffer) mpp_buffer_put(pair.second.buffer);
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
    int ret = api_->decode_put_packet(ctx_, pkt);
    mpp_packet_deinit(&pkt);

    if (ret != MPP_OK) {
        if (failed_flag) failed_flag->store(true);
        if (ready_flag) ready_flag->store(true);
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
            int fd = mpp_buffer_get_fd(buf);
            if (fd >= 0) {
                std::lock_guard<std::mutex> lock(g_surfaces_mutex);
                auto it = surfaces_.find(job.target_surface);
                if (it != surfaces_.end()) {
                    mpp_buffer_inc_ref(buf);
                    if (it->second.buffer) mpp_buffer_put(it->second.buffer);
                    it->second.buffer = buf;
                    it->second.dmabuf_fd = fd;
                    it->second.width = mpp_frame_get_width(frame);
                    it->second.height = mpp_frame_get_height(frame);
                    // Immediately update stride from frame to avoid mismatch
                    it->second.stride = mpp_frame_get_hor_stride(frame);
                    got = true;
                }
            }
        }

        mpp_frame_deinit(&frame);
    }

    if (ready_flag) ready_flag->store(true);
    return got;
}

CodecProfile vaProfileToCodec(VAProfile profile) {
    if (profile == VAProfileH264High) return CodecProfile::H264;
    if (profile == VAProfileHEVCMain) return CodecProfile::HEVC;
    if (profile == VAProfileAV1Profile0) return CodecProfile::AV1;
    return CodecProfile::Unknown;
}

} // namespace rockchip
