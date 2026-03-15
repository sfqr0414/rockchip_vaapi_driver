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
    // Make initialize idempotent: if already initialized, ensure required resources exist.
    if (ctx_) {
        if (group_) {
            return true;
        }
        // If the MPP context exists but the buffer group is missing, attempt to create it.
    }

    profile_ = profile;

    if (profile_ == CodecProfile::Unknown) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "Unsupported codec profile");
        return false;
    }

    if (!ctx_) {
        if (mpp_create(&ctx_, &api_) != MPP_OK) {
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "mpp_create failed");
            return false;
        }
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
        void* ctx_ptr = ctx_;
        void* group_ptr = group_;
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "Cannot allocate surface before decoder init (ctx={} group={})",
                  ctx_ptr, group_ptr);
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

    // Import the DMABUF into MPP so it can be used as a decode output buffer.
    MppBufferInfo bufInfo{};
    bufInfo.type = MPP_BUFFER_TYPE_EXT_DMA;
    bufInfo.fd = fd;
    bufInfo.size = allocationSize;

    MppBuffer imported = nullptr;
    if (mpp_buffer_import(&imported, &bufInfo) != MPP_OK) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "mpp_buffer_import failed");
        mpp_buffer_put(allocBuf);
        close(fd);
        return false;
    }

    out.va_id = id;
    out.dmabuf_fd = fd;
    out.width = static_cast<uint32_t>(width);
    out.height = static_cast<uint32_t>(height);
    out.stride = stride;
    out.ready.store(false);

    SurfaceInfo surfaceInfo;
    surfaceInfo.dmabuf_fd = fd;
    surfaceInfo.buffer = imported;
    surfaceInfo.width = static_cast<uint32_t>(width);
    surfaceInfo.height = static_cast<uint32_t>(height);
    surfaceInfo.stride = stride;
    surfaceInfo.ready = std::make_shared<std::atomic<bool>>(false);

    surfaces_[id] = std::move(surfaceInfo);

    // Drop the canonical allocation handle; the imported buffer holds the ref.
    mpp_buffer_put(allocBuf);
    return true;
}

bool MppDecoder::updateSurfaceResolution(VASurfaceID id, int width, int height) {
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;

    if (it->second.width == static_cast<uint32_t>(width) &&
        it->second.height == static_cast<uint32_t>(height)) {
        return true;
    }

    // Free existing resources.
    if (it->second.buffer) {
        mpp_buffer_put(it->second.buffer);
        it->second.buffer = nullptr;
    }
    if (it->second.dmabuf_fd >= 0) {
        close(it->second.dmabuf_fd);
        it->second.dmabuf_fd = -1;
    }

    DecodedSurface dummy;
    if (!allocateSurface(id, dummy, width, height)) {
        return false;
    }

    // Update stored size/stride.
    it->second.width = static_cast<uint32_t>(width);
    it->second.height = static_cast<uint32_t>(height);
    it->second.stride = alignUp(width, 64);
    return true;
}

bool MppDecoder::getSurfaceInfo(VASurfaceID id, uint32_t& width, uint32_t& height, uint32_t& stride, int& dmabuf_fd) {
    auto it = surfaces_.find(id);
    if (it == surfaces_.end()) return false;
    width = it->second.width;
    height = it->second.height;
    stride = it->second.stride;
    dmabuf_fd = it->second.dmabuf_fd;
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

bool MppDecoder::isInitialized() const {
    return ctx_ != nullptr;
}

bool MppDecoder::waitSurfaceReady(VASurfaceID surface, uint32_t timeout_ms) {
    auto it = surfaces_.find(surface);
    if (it == surfaces_.end()) {
        return false;
    }
    auto& ready_flag = *it->second.ready;

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms ? timeout_ms : 5000);
    while (!ready_flag.load(std::memory_order_acquire)) {
        if (timeout_ms && std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        // Release DMABUF FDs and imported MPP buffers.
        for (auto& [id, info] : surfaces_) {
            if (info.buffer) {
                mpp_buffer_put(info.buffer);
                info.buffer = nullptr;
            }
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

    // Feed the bitstream to MPP per-NAL unit for more robust decoding.
    const uint8_t* data = job.bitstream.data();
    size_t dataSize = job.bitstream.size();

    auto findStart = [&](size_t pos) -> size_t {
        for (size_t i = pos; i + 3 < dataSize; ++i) {
            if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
                return i;
            }
            if (i + 4 < dataSize && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) {
                return i;
            }
        }
        return dataSize;
    };

    size_t nalStart = findStart(0);
    if (nalStart == dataSize) {
        // If no start codes were found, fall back to feeding the whole buffer.
        nalStart = 0;
    }

    bool gotFrame = false;

    const size_t maxPacketSize = 64 * 1024;

    auto sendPacketRange = [&](size_t begin, size_t end, bool isLast, bool allowSplit) {
        if (!allowSplit) {
            // Send the full range as one packet (safe for H264/HEVC NAL units).
            MppPacket packet = nullptr;
            size_t size = end - begin;
            if (mpp_packet_init(&packet, (void*)(data + begin), size) != MPP_OK) {
                return false;
            }
            if (isLast) {
                mpp_packet_set_eos(packet);
            }

            int ret = api_->decode_put_packet(ctx_, packet);
            mpp_packet_deinit(&packet);
            if (ret != MPP_OK) {
                util::log(util::stderr_sink, util::LogLevel::Warn,
                          "mpp_decode_put_packet failed: {} (offset={} size={})",
                          ret, begin, size);
                return false;
            }
            return true;
        }

        size_t offset = begin;
        while (offset < end) {
            size_t chunkSize = std::min(maxPacketSize, end - offset);
            bool isLastChunk = isLast && (offset + chunkSize == end);

            MppPacket packet = nullptr;
            if (mpp_packet_init(&packet, (void*)(data + offset), chunkSize) != MPP_OK) {
                return false;
            }
            if (isLastChunk) {
                mpp_packet_set_eos(packet);
            }

            int ret = api_->decode_put_packet(ctx_, packet);
            mpp_packet_deinit(&packet);
            if (ret != MPP_OK) {
                util::log(util::stderr_sink, util::LogLevel::Warn,
                          "mpp_decode_put_packet failed: {} (offset={} size={})",
                          ret, offset, chunkSize);
                return false;
            }

            offset += chunkSize;
        }
        return true;
    };

    auto parseAv1Obu = [&](size_t off, size_t& obu_size) -> bool {
        if (off >= dataSize) return false;
        size_t ptr = off;
        uint8_t header = data[ptr++];
        bool has_size = header & 0x40;
        size_t size_field_len = 0;
        uint64_t payload_size = 0;

        if (has_size) {
            // leb128
            while (true) {
                if (ptr >= dataSize) return false;
                uint8_t b = data[ptr++];
                payload_size |= uint64_t(b & 0x7F) << (7 * size_field_len);
                size_field_len++;
                if (!(b & 0x80)) break;
                if (size_field_len > 8) return false;
            }

            size_t total = 1 + size_field_len + payload_size;
            if (off + total > dataSize) return false;
            obu_size = total;
            return true;
        }

        // If there is no size field, the OBU extends until the next OBU header.
        // We scan forward for a valid OBU header (forbidden bit == 0, and
        // OBU type in [0,31]). This is heuristic but works for typical streams.
        size_t scan = ptr;
        while (scan + 1 < dataSize) {
            uint8_t candidate = data[scan];
            if ((candidate & 0x80) == 0) {
                uint8_t obu_type = (candidate >> 3) & 0x1F;
                if (obu_type <= 31) {
                    // Assume this is a new OBU header.
                    break;
                }
            }
            scan++;
        }
        if (scan <= ptr) {
            // No next header found; assume rest of stream is current OBU.
            obu_size = dataSize - off;
            return true;
        }

        obu_size = scan - off;
        return true;
    };

    auto drainFrames = [&]() -> bool {
        while (true) {
            MppFrame frame = nullptr;
            if (mpp_frame_init(&frame) != MPP_OK) {
                return false;
            }

            if (profile_ == CodecProfile::AV1) {
                // Let MPP choose output format for AV1 (some HW may not support external NV12 buffers).
            } else {
                mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
                mpp_frame_set_width(frame, info.width);
                mpp_frame_set_height(frame, info.height);
                mpp_frame_set_hor_stride(frame, info.stride);
                mpp_frame_set_ver_stride(frame, info.height);
                mpp_frame_set_buf_size(frame, computeNv12Size(info.width, info.height, info.stride));
                if (info.buffer) {
                    mpp_frame_set_buffer(frame, info.buffer);
                }
            }

            int ret = api_->decode_get_frame(ctx_, &frame);
            if (ret == MPP_ERR_DISPLAY_FULL) {
                mpp_frame_deinit(&frame);
                break;
            }

            if (ret != MPP_OK || !frame) {
                if (ret != MPP_OK) {
                    util::log(util::stderr_sink, util::LogLevel::Warn,
                              "mpp_decode_get_frame returned %d", ret);
                }
                if (frame) mpp_frame_deinit(&frame);
                break;
            }

            // Dynamically update width/height/stride if MPP reports a change.
            uint32_t frame_w = mpp_frame_get_width(frame);
            uint32_t frame_h = mpp_frame_get_height(frame);
            uint32_t frame_stride = mpp_frame_get_hor_stride(frame);
            if (frame_w && frame_h && (frame_w != info.width || frame_h != info.height)) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "MPP reported resolution change %ux%u -> %ux%u",
                          info.width, info.height, frame_w, frame_h);
                // Update buffer/surface sizes.
                updateSurfaceResolution(job.target_surface, frame_w, frame_h);
            }

            if (info.buffer) {
                mpp_buffer_sync_end(info.buffer);
            }

            gotFrame = true;
            if (info.ready) {
                info.ready->store(true, std::memory_order_release);
                info.ready->notify_all();
            }

            mpp_frame_deinit(&frame);
            return true;
        }
        return false;
    };

    if (profile_ == CodecProfile::AV1) {
        // MPP may expect AV1 data in a length-prefixed (IVF-style) format.
        size_t pos = 0;
        while (pos < dataSize) {
            size_t obu_size = 0;
            if (!parseAv1Obu(pos, obu_size)) break;

            bool isLast = (pos + obu_size == dataSize);

            // Create a packet: [4-byte LE size][OBU bytes]
            uint32_t len_le = static_cast<uint32_t>(obu_size);
            std::vector<uint8_t> packet_data;
            packet_data.reserve(4 + obu_size);
            packet_data.push_back(len_le & 0xff);
            packet_data.push_back((len_le >> 8) & 0xff);
            packet_data.push_back((len_le >> 16) & 0xff);
            packet_data.push_back((len_le >> 24) & 0xff);
            packet_data.insert(packet_data.end(), data + pos, data + pos + obu_size);

            MppPacket packet = nullptr;
            if (mpp_packet_init(&packet, packet_data.data(), packet_data.size()) != MPP_OK) {
                return false;
            }
            if (isLast) {
                mpp_packet_set_eos(packet);
            }

            int ret = api_->decode_put_packet(ctx_, packet);
            mpp_packet_deinit(&packet);
            if (ret != MPP_OK) {
                util::log(util::stderr_sink, util::LogLevel::Warn,
                          "mpp_decode_put_packet failed: {} (obu_offset={} obu_size={})",
                          ret, pos, obu_size);
                return false;
            }

            if (drainFrames()) return true;
            pos += obu_size;
        }
    } else {
        // H.264/HEVC: feed each NAL unit as a packet.
        while (nalStart < dataSize) {
            size_t nalEnd = findStart(nalStart + 3);
            bool isLastNal = (nalEnd == dataSize);
            if (!sendPacketRange(nalStart, nalEnd, isLastNal, false)) {
                return false;
            }
            if (drainFrames()) return true;
            nalStart = nalEnd;
        }
    }

    return gotFrame;
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
