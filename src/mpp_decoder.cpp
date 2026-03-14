#include "mpp_decoder.h"

#include <chrono>
#include <fcntl.h>
#include <unistd.h>

namespace rockchip {

static MppCodingType codecProfileToMpp(CodecProfile profile) {
    switch (profile) {
        case CodecProfile::H264:
            return MPP_VIDEO_CodingAVC;
        case CodecProfile::HEVC:
            return MPP_VIDEO_CodingHEVC;
        case CodecProfile::VP9:
            return MPP_VIDEO_CodingVP9;
        case CodecProfile::AV1:
            return MPP_VIDEO_CodingAV1;
        default:
            return MPP_VIDEO_CodingUnused;
    }
}

static uint32_t alignUp(uint32_t value, uint32_t align) {
    return (value + align - 1) / align * align;
}

static uint32_t computeNv12Size(uint32_t width, uint32_t height, uint32_t stride) {
    const uint32_t ySize = stride * height;
    const uint32_t uvStride = stride / 2;
    const uint32_t uvHeight = (height + 1) / 2;
    return ySize + uvStride * uvHeight * 2;
}

MppDecoder::MppDecoder() = default;

MppDecoder::~MppDecoder() {
    shutdown();
}

bool MppDecoder::initialize(CodecProfile profile, int width, int height) {
    profile_ = profile;

    if (profile_ == CodecProfile::Unknown) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "Unsupported codec profile");
        return false;
    }

    if (mpp_create(&ctx_, &api_) != MPP_OK) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp_create failed");
        return false;
    }

    if (mpp_init(ctx_, MPP_CTX_DEC, codecProfileToMpp(profile_)) != MPP_OK) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp_init failed");
        return false;
    }

    // Enable split-parse mode for robust parsing of streaming buffers.
    // Rockchip MPP API does not expose split_parse directly in this version.
    // Some legacy bindings may provide it; ignore if unavailable.
    (void)ctx_;

    if (mpp_buffer_group_get_internal(&group_, MPP_BUFFER_TYPE_DMA_HEAP) != MPP_OK) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp_buffer_group_get_internal failed");
        return false;
    }

    // Optionally set custom allocator; keep default for now.
    return true;
}

bool MppDecoder::allocateSurface(VASurfaceID id, DecodedSurface& out, int width, int height) {
    if (!group_) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "Cannot allocate surface before decoder init");
        return false;
    }

    const uint32_t stride = alignUp(width, 64);
    const size_t allocationSize = computeNv12Size(width, height, stride);

    // Allocate a DMABUF via MPP and export its FD so VA can access it.
    MppBuffer allocBuf = nullptr;
    if (mpp_buffer_get(group_, &allocBuf, allocationSize) != MPP_OK) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp_buffer_get failed (size=%zu)", allocationSize);
        return false;
    }

    int fd = mpp_buffer_get_fd(allocBuf);
    if (fd < 0) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp_buffer_get_fd failed");
        mpp_buffer_put(allocBuf);
        return false;
    }

    // Commit the DMABUF into the group so MPP can use it as an output buffer.
    MppBufferInfo info{};
    info.type = MPP_BUFFER_TYPE_EXT_DMA;
    info.fd = fd;
    info.size = allocationSize;
    if (mpp_buffer_commit(group_, &info) != MPP_OK) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp_buffer_commit failed");
        mpp_buffer_put(allocBuf);
        close(fd);
        return false;
    }

    out.va_id = id;
    out.dmabuf_fd = fd;
    out.width = static_cast<uint32_t>(width);
    out.height = static_cast<uint32_t>(height);
    out.ready.store(false);

    SurfaceInfo surfaceInfo;
    surfaceInfo.dmabuf_fd = fd;
    surfaceInfo.width = static_cast<uint32_t>(width);
    surfaceInfo.height = static_cast<uint32_t>(height);
    surfaceInfo.stride = stride;
    surfaceInfo.ready = std::make_shared<std::atomic<bool>>(false);

    surfaces_[id] = std::move(surfaceInfo);

    // Release the original allocation handle; the fd remains valid for VA.
    mpp_buffer_put(allocBuf);
    return true;
}

bool MppDecoder::enqueueJob(DecodeJob job) {
    if (!running_) {
        running_.store(true, std::memory_order_release);
        decoder_thread_ = std::thread(&MppDecoder::decoderThreadMain, this);
    }
    job_queue_.push(std::move(job));
    return true;
}

bool MppDecoder::waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms) {
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) {
        return false;
    }
    auto& ready_flag = *it->second.ready;

    auto start = std::chrono::steady_clock::now();
    while (!ready_flag.load(std::memory_order_acquire)) {
        if (timeout_ms &&
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start)
                    .count() >= timeout_ms) {
            return false;
        }
        ready_flag.wait(false);
    }
    return true;
}

void MppDecoder::markSurfaceReady(VASurfaceID surface) {
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end())
        return;

    if (it->second.ready) {
        it->second.ready->store(true, std::memory_order_release);
        it->second.ready->notify_all();
    }
}

void MppDecoder::shutdown() {
    if (running_.exchange(false)) {
        job_queue_.shutdown();
        if (decoder_thread_.joinable()) {
            decoder_thread_.join();
        }
    }
    if (ctx_) {
        mpp_destroy(ctx_);
        ctx_ = nullptr;
        api_ = nullptr;
    }
    if (group_) {
        // Release DMABUF FDs for VA surfaces.
        for (auto& [id, info] : surfaces_) {
            if (info.dmabuf_fd >= 0) {
                close(info.dmabuf_fd);
                info.dmabuf_fd = -1;
            }
        }
        surfaces_.clear();

        mpp_buffer_group_put(group_);
        group_ = nullptr;
    }
}

void MppDecoder::decoderThreadMain() {
    while (running_) {
        auto jobOpt = job_queue_.pop();
        if (!jobOpt) {
            break;
        }
        (void)processJob(*jobOpt);
    }
}

bool MppDecoder::processJob(const DecodeJob& job) {
    if (!ctx_) {
        return false;
    }

    auto it = surfaces_.find(job.target_surface);
    if (it == surfaces_.end()) {
        return false;
    }
    SurfaceInfo& info = it->second;

    // Create an MPP packet for the bitstream.
    MppPacket packet = nullptr;
    if (mpp_packet_init(&packet, nullptr, 0) != MPP_OK) {
        return false;
    }
mpp_packet_set_data(packet, const_cast<void*>(static_cast<const void*>(job.bitstream.data())));
    mpp_packet_set_length(packet, job.bitstream.size());

    // Prepare the output frame (decoder should pull from our committed buffer group).
    MppFrame frame = nullptr;
    if (mpp_frame_init(&frame) != MPP_OK) {
        mpp_packet_deinit(&packet);
        return false;
    }

    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
    mpp_frame_set_width(frame, info.width);
    mpp_frame_set_height(frame, info.height);
    mpp_frame_set_hor_stride(frame, info.stride);
    mpp_frame_set_ver_stride(frame, info.height);
    mpp_frame_set_buf_size(frame, computeNv12Size(info.width, info.height, info.stride));

    // Send packet and get decoded frame.
    if (api_->decode(ctx_, packet, &frame) != MPP_OK) {
        mpp_frame_deinit(&frame);
        mpp_packet_deinit(&packet);
        return false;
    }

    // Notify the surface that decoding is complete.
    if (info.ready) {
        info.ready->store(true, std::memory_order_release);
        info.ready->notify_all();
    }

    mpp_frame_deinit(&frame);
    mpp_packet_deinit(&packet);
    return true;
}

CodecProfile vaProfileToCodec(VAProfile profile) {
    switch (profile) {
        case VAProfileH264High:
            return CodecProfile::H264;
        case VAProfileHEVCMain:
            return CodecProfile::HEVC;
        case VAProfileVP9Profile0:
            return CodecProfile::VP9;
        case VAProfileAV1Profile0:
            return CodecProfile::AV1;
        default:
            return CodecProfile::Unknown;
    }
}

} // namespace rockchip
