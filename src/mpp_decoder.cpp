#include "mpp_decoder.h"
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <mutex>

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
    if (mpp_buffer_group_get_internal(&group_, MPP_BUFFER_TYPE_DMA_HEAP) != MPP_OK) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }

    // Set some common controls for robust decoding
    RK_U32 timeout = 100; // 100ms
    api_->control(ctx_, MPP_SET_OUTPUT_TIMEOUT, &timeout);

    return true;
}

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    uint32_t stride = alignUp(width, 64);
    if (profile_ == CodecProfile::AV1) {
        stride = alignUp((width * 10 + 7) / 8, 256) + 256;
    }
    out.va_id = id;
    out.width = width;
    out.height = height;
    out.stride = stride;
    out.is_10bit = (profile_ == CodecProfile::AV1);
    out.ready.store(false);

    SurfaceInfo info;
    info.width = width;
    info.height = height;
    info.stride = stride;
    info.ready = std::make_shared<std::atomic<bool>>(false);
    info.decode_failed = std::make_shared<std::atomic<bool>>(false);
    
    std::lock_guard<std::mutex> lock(g_surfaces_mutex);
    surfaces_[id] = std::move(info);
    return true;
}

bool MppDecoder::updateSurfaceResolution(VASurfaceID id, int width, int height) { return true; }

bool MppDecoder::getSurfaceInfo(VASurfaceID id, uint32_t& width, uint32_t& height, uint32_t& stride, int& fd, bool& failed) {
    std::lock_guard<std::mutex> lock(g_surfaces_mutex);
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    width = it->second.width;
    height = it->second.height;
    stride = it->second.stride;
    fd = it->second.dmabuf_fd;
    failed = it->second.decode_failed ? it->second.decode_failed->load() : false;
    return true;
}

bool MppDecoder::enqueueJob(DecodeJob job) {
    if (!running_) {
        running_ = true;
        decoder_thread_ = std::thread(&MppDecoder::decoderThreadMain, this);
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
    if (decoder_thread_.joinable()) decoder_thread_.join();
    
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

void MppDecoder::decoderThreadMain() {
    while (running_) {
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

    if (!job.extra_data.empty()) {
        MppPacket pkt;
        mpp_packet_init(&pkt, (void*)job.extra_data.data(), job.extra_data.size());
        mpp_packet_set_extra_data(pkt);
        api_->decode_put_packet(ctx_, pkt);
        mpp_packet_deinit(&pkt);
    }

    MppPacket pkt;
    mpp_packet_init(&pkt, (void*)job.bitstream.data(), job.bitstream.size());
    int ret = api_->decode_put_packet(ctx_, pkt);
    mpp_packet_deinit(&pkt);

    if (ret != MPP_OK) {
        if (failed_flag) failed_flag->store(true);
        if (ready_flag) ready_flag->store(true);
        return false;
    }

    // Capture frames
    bool got = false;
    while (true) {
        MppFrame frame = nullptr;
        mpp_frame_init(&frame);
        int ret = api_->decode_get_frame(ctx_, &frame);
        if (ret != MPP_OK || !frame) {
            if (frame) mpp_frame_deinit(&frame);
            break;
        }

        if (mpp_frame_get_info_change(frame)) {
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

}
