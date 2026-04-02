#pragma once

#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_meta.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/rk_mpi.h>
#include <va/va.h>

#include <atomic>
#include <bit>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "util/log.h"

namespace rockchip {

class unique_fd {
   public:
    unique_fd() = default;
    explicit unique_fd(int fd) noexcept : fd_(fd) {}
    ~unique_fd() { reset(); }

    unique_fd(const unique_fd&) = delete;
    unique_fd& operator=(const unique_fd&) = delete;

    unique_fd(unique_fd&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

    unique_fd& operator=(unique_fd&& other) noexcept {
        if (this != &other) {
            reset();
            fd_ = std::exchange(other.fd_, -1);
        }
        return *this;
    }

    [[nodiscard]] int get() const noexcept { return fd_; }
    [[nodiscard]] explicit operator bool() const noexcept { return fd_ >= 0; }
    [[nodiscard]] int release() noexcept { return std::exchange(fd_, -1); }

    void reset(int fd = -1) noexcept {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = fd;
    }

   private:
    int fd_ = -1;
};

template <typename T, typename E>
class Expected {
   public:
    Expected(T value) : storage_(std::move(value)) {}
    Expected(E error) : storage_(error) {}

    [[nodiscard]] bool has_value() const noexcept { return std::holds_alternative<T>(storage_); }
    [[nodiscard]] explicit operator bool() const noexcept { return has_value(); }
    [[nodiscard]] T& value() { return std::get<T>(storage_); }
    [[nodiscard]] const T& value() const { return std::get<T>(storage_); }
    [[nodiscard]] E error() const { return std::get<E>(storage_); }

   private:
    std::variant<T, E> storage_;
};

template <typename E>
class Expected<void, E> {
   public:
    Expected() = default;
    Expected(E error) : error_(error) {}

    [[nodiscard]] bool has_value() const noexcept { return !error_.has_value(); }
    [[nodiscard]] explicit operator bool() const noexcept { return has_value(); }
    [[nodiscard]] E error() const { return *error_; }

   private:
    std::optional<E> error_;
};

class MppSession {
   public:
    MppSession(const MppSession&) = delete;
    MppSession& operator=(const MppSession&) = delete;

    ~MppSession() {
        if (ctx_) {
            mpp_destroy(ctx_);
            ctx_ = nullptr;
            api_ = nullptr;
        }
    }

    [[nodiscard]] static std::shared_ptr<MppSession> create() noexcept {
        MppCtx ctx = nullptr;
        MppApi* api = nullptr;
        if (mpp_create(&ctx, &api) != MPP_OK || !ctx || !api) {
            if (ctx) {
                mpp_destroy(ctx);
            }
            return {};
        }

        return std::shared_ptr<MppSession>(new MppSession(ctx, api));
    }

    [[nodiscard]] MppCtx ctx() const noexcept { return ctx_; }
    [[nodiscard]] MppApi* api() const noexcept { return api_; }

   private:
    MppSession(MppCtx ctx, MppApi* api) noexcept : ctx_(ctx), api_(api) {}

    MppCtx ctx_ = nullptr;
    MppApi* api_ = nullptr;
};

class MppBufferHandle {
   public:
    MppBufferHandle() = default;
    explicit MppBufferHandle(MppBuffer handle) noexcept : handle_(handle) {}
    ~MppBufferHandle() { reset(); }

    MppBufferHandle(const MppBufferHandle&) = delete;
    MppBufferHandle& operator=(const MppBufferHandle&) = delete;

    MppBufferHandle(MppBufferHandle&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}

    MppBufferHandle& operator=(MppBufferHandle&& other) noexcept {
        if (this != &other) {
            reset();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    [[nodiscard]] MppBuffer get() const noexcept { return handle_; }
    [[nodiscard]] explicit operator bool() const noexcept { return handle_ != nullptr; }
    [[nodiscard]] MppBuffer release() noexcept { return std::exchange(handle_, nullptr); }

    void reset(MppBuffer handle = nullptr) noexcept {
        if (handle_) {
            mpp_buffer_put(handle_);
        }
        handle_ = handle;
    }

   private:
    MppBuffer handle_ = nullptr;
};

class MppFrameHandle {
   public:
    MppFrameHandle() = default;
    explicit MppFrameHandle(MppFrame handle) noexcept : handle_(handle) {}
    ~MppFrameHandle() { reset(); }

    MppFrameHandle(const MppFrameHandle&) = delete;
    MppFrameHandle& operator=(const MppFrameHandle&) = delete;

    MppFrameHandle(MppFrameHandle&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}

    MppFrameHandle& operator=(MppFrameHandle&& other) noexcept {
        if (this != &other) {
            reset();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    [[nodiscard]] MppFrame get() const noexcept { return handle_; }
    [[nodiscard]] explicit operator bool() const noexcept { return handle_ != nullptr; }
    [[nodiscard]] MppFrame release() noexcept { return std::exchange(handle_, nullptr); }

    void reset(MppFrame handle = nullptr) noexcept {
        if (handle_) {
            mpp_frame_deinit(&handle_);
        }
        handle_ = handle;
    }

   private:
    MppFrame handle_ = nullptr;
};

enum class DecoderError {
    InvalidState,
    AllocationFailed,
    MppFailure,
    IoFailure,
};

template <typename T>
class DecoderResult {
   public:
    DecoderResult(T value) : value_(std::move(value)) {}
    DecoderResult(DecoderError error) : error_(error) {}

    [[nodiscard]] bool has_value() const noexcept { return value_.has_value(); }
    [[nodiscard]] explicit operator bool() const noexcept { return has_value(); }
    [[nodiscard]] T& value() { return *value_; }
    [[nodiscard]] const T& value() const { return *value_; }
    [[nodiscard]] DecoderError error() const noexcept { return error_; }

   private:
    std::optional<T> value_;
    DecoderError error_ = DecoderError::InvalidState;
};

enum class CodecProfile {
    H264,
    HEVC,
    VP9,
    AV1,
    Unknown,
};

class MappedRegion {
   public:
    MappedRegion() = default;
    MappedRegion(void* address, size_t size) noexcept : address_(address), size_(size) {}
    ~MappedRegion() { reset(); }

    MappedRegion(const MappedRegion&) = delete;
    MappedRegion& operator=(const MappedRegion&) = delete;

    MappedRegion(MappedRegion&& other) noexcept
        : address_(std::exchange(other.address_, nullptr)), size_(std::exchange(other.size_, 0)) {}

    MappedRegion& operator=(MappedRegion&& other) noexcept {
        if (this != &other) {
            reset();
            address_ = std::exchange(other.address_, nullptr);
            size_ = std::exchange(other.size_, 0);
        }
        return *this;
    }

    [[nodiscard]] static MappedRegion map(int fd, size_t size, int prot) noexcept {
        if (fd < 0 || size == 0) {
            return {};
        }
        void* address = mmap(nullptr, size, prot, MAP_SHARED, fd, 0);
        return address == MAP_FAILED ? MappedRegion{} : MappedRegion{address, size};
    }

    [[nodiscard]] void* data() const noexcept { return address_; }
    [[nodiscard]] explicit operator bool() const noexcept { return address_ != nullptr; }

    void reset() noexcept {
        if (address_ != nullptr && size_ != 0) {
            munmap(address_, size_);
        }
        address_ = nullptr;
        size_ = 0;
    }

   private:
    void* address_ = nullptr;
    size_t size_ = 0;
};

class MppPacketHandle {
   public:
    MppPacketHandle() = default;
    explicit MppPacketHandle(MppPacket packet) noexcept : packet_(packet) {}
    ~MppPacketHandle() { reset(); }

    MppPacketHandle(const MppPacketHandle&) = delete;
    MppPacketHandle& operator=(const MppPacketHandle&) = delete;

    MppPacketHandle(MppPacketHandle&& other) noexcept : packet_(std::exchange(other.packet_, nullptr)) {}

    MppPacketHandle& operator=(MppPacketHandle&& other) noexcept {
        if (this != &other) {
            reset();
            packet_ = std::exchange(other.packet_, nullptr);
        }
        return *this;
    }

    [[nodiscard]] MppPacket get() const noexcept { return packet_; }
    [[nodiscard]] explicit operator bool() const noexcept { return packet_ != nullptr; }

    void reset(MppPacket packet = nullptr) noexcept {
        if (packet_) {
            mpp_packet_deinit(&packet_);
        }
        packet_ = packet;
    }

   private:
    MppPacket packet_ = nullptr;
};

class MppDecCfgHandle {
   public:
    MppDecCfgHandle() = default;
    explicit MppDecCfgHandle(MppDecCfg cfg) noexcept : cfg_(cfg) {}
    ~MppDecCfgHandle() { reset(); }

    MppDecCfgHandle(const MppDecCfgHandle&) = delete;
    MppDecCfgHandle& operator=(const MppDecCfgHandle&) = delete;

    [[nodiscard]] MppDecCfg get() const noexcept { return cfg_; }
    [[nodiscard]] explicit operator bool() const noexcept { return cfg_ != nullptr; }

    void reset(MppDecCfg cfg = nullptr) noexcept {
        if (cfg_) {
            mpp_dec_cfg_deinit(cfg_);
        }
        cfg_ = cfg;
    }

   private:
    MppDecCfg cfg_ = nullptr;
};

inline constexpr bool kAv1ExportAsP010 = true;
inline constexpr uint64_t kDecoderFlushIdleMicros = 2'000'000ULL;
inline constexpr size_t kMaxInFlightJobs = 16;

[[nodiscard]] inline MppCodingType codecProfileToMpp(CodecProfile profile) {
    switch (profile) {
        case CodecProfile::H264: return MPP_VIDEO_CodingAVC;
        case CodecProfile::HEVC: return MPP_VIDEO_CodingHEVC;
        case CodecProfile::VP9: return MPP_VIDEO_CodingVP9;
        case CodecProfile::AV1: return MPP_VIDEO_CodingAV1;
        default: return MPP_VIDEO_CodingUnused;
    }
}

[[nodiscard]] inline uint32_t alignUp(uint32_t value, uint32_t align) {
    assert(std::has_single_bit(align));
    return (value + align - 1) / align * align;
}

[[nodiscard]] inline uint64_t steadyMicrosNow() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

[[nodiscard]] inline size_t surfaceBufferSize(uint32_t width, uint32_t height, uint32_t stride, bool is_10bit) {
    if (!is_10bit) {
        return static_cast<size_t>(stride) * static_cast<size_t>(height) * 3 / 2;
    }

    return static_cast<size_t>(stride) * static_cast<size_t>(alignUp(height, 8)) * 3;
}

[[nodiscard]] inline bool submitDecoderEos(MppCtx ctx, MppApi* api, const std::atomic<bool>& running) {
    if (!ctx || !api) return false;

    MppPacket raw_packet = nullptr;
    if (mpp_packet_init(&raw_packet, nullptr, 0) != MPP_OK || !raw_packet) {
        return false;
    }
    MppPacketHandle eos_packet{raw_packet};

    mpp_packet_set_eos(eos_packet.get());
    bool submitted = false;
    for (int i = 0; i < 200 && running.load(std::memory_order_relaxed); ++i) {
        int ret = api->decode_put_packet(ctx, eos_packet.get());
        if (ret == MPP_OK) {
            submitted = true;
            break;
        }
        if (ret == MPP_ERR_BUFFER_FULL) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        break;
    }

    return submitted;
}

inline void unpackPacked10RowLE(std::span<const uint8_t> src,
                                uint16_t* dst,
                                size_t sample_count) {
    for (size_t sample = 0; sample < sample_count; ++sample) {
        const size_t src_index = (sample * 10) / 8;
        const uint8_t bit_offset = static_cast<uint8_t>((sample * 2) & 7u);
        if (src_index + 1 >= src.size()) {
            dst[sample] = 0;
            continue;
        }

        const uint16_t packed = static_cast<uint16_t>((src[src_index] >> bit_offset) |
                                                       (src[src_index + 1] << (8 - bit_offset)));
        const uint16_t value8 = static_cast<uint16_t>((packed & 0x03FFu) >> 2);
        dst[sample] = static_cast<uint16_t>(value8 << 8);
    }
}

[[nodiscard]] inline bool writeSurfaceP010(int fd,
                                           uint32_t width,
                                           uint32_t height,
                                           uint32_t dst_stride,
                                           uint32_t src_stride_bytes,
                                           uint32_t src_ver_stride,
                                           std::span<const uint8_t> src) {
    const size_t src_row_bytes = static_cast<size_t>(src_stride_bytes);
    const size_t packed_row_bytes = ((static_cast<size_t>(width) + 3) / 4) * 5;
    if (src.size() < src_row_bytes * static_cast<size_t>(src_ver_stride)) return false;

    const size_t dst_row_bytes = static_cast<size_t>(dst_stride) * 2;
    const size_t aligned_h = ((height + 7) / 8) * 8;
    const size_t dst_size = dst_row_bytes * aligned_h * 3 / 2;
    std::vector<uint8_t> dst(dst_size, 0);

    const auto src_y = src.first(src_row_bytes * src_ver_stride);
    const auto src_uv = src.subspan(src_row_bytes * src_ver_stride);
    uint8_t* dst_y = dst.data();
    uint8_t* dst_uv = dst.data() + dst_row_bytes * aligned_h;

    for (uint32_t row = 0; row < height; ++row) {
        auto* dst_row = reinterpret_cast<uint16_t*>(dst_y + static_cast<size_t>(row) * dst_row_bytes);
        const auto src_row = src_y.subspan(static_cast<size_t>(row) * src_row_bytes,
                                           std::min(src_row_bytes, packed_row_bytes));
        unpackPacked10RowLE(src_row, dst_row, width);
    }

    for (uint32_t row = 0; row < height / 2; ++row) {
        auto* dst_row = reinterpret_cast<uint16_t*>(dst_uv + static_cast<size_t>(row) * dst_row_bytes);
        const auto src_row = src_uv.subspan(static_cast<size_t>(row) * src_row_bytes,
                                            std::min(src_row_bytes, packed_row_bytes));
        unpackPacked10RowLE(src_row, dst_row, width);
    }

    return pwrite(fd, dst.data(), dst.size(), 0) == static_cast<ssize_t>(dst.size());
}

[[nodiscard]] inline bool writeSurfaceNV12(int fd,
                                           uint32_t width,
                                           uint32_t height,
                                           uint32_t dst_stride,
                                           uint32_t src_stride_bytes,
                                           uint32_t src_ver_stride,
                                           std::span<const uint8_t> src) {
    const size_t src_row_bytes = static_cast<size_t>(src_stride_bytes);
    const size_t expected_size = src_row_bytes * static_cast<size_t>(src_ver_stride) * 3 / 2;
    if (src.size() < expected_size) return false;

    const size_t dst_row_bytes = static_cast<size_t>(dst_stride);
    const size_t copy_row_bytes = static_cast<size_t>(width);
    const size_t aligned_h = ((height + 7) / 8) * 8;
    const size_t dst_size = dst_row_bytes * aligned_h * 3 / 2;
    std::vector<uint8_t> dst(dst_size, 0);

    const auto src_y = src.first(src_row_bytes * src_ver_stride);
    const auto src_uv = src.subspan(src_row_bytes * src_ver_stride);
    uint8_t* dst_y = dst.data();
    uint8_t* dst_uv = dst.data() + dst_row_bytes * aligned_h;

    for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(dst_y + static_cast<size_t>(row) * dst_row_bytes,
                    src_y.data() + static_cast<size_t>(row) * src_row_bytes,
                    copy_row_bytes);
    }

    for (uint32_t row = 0; row < height / 2; ++row) {
        std::memcpy(dst_uv + static_cast<size_t>(row) * dst_row_bytes,
                    src_uv.data() + static_cast<size_t>(row) * src_row_bytes,
                    copy_row_bytes);
    }

    return pwrite(fd, dst.data(), dst.size(), 0) == static_cast<ssize_t>(dst.size());
}

[[nodiscard]] inline CodecProfile vaProfileToCodec(VAProfile profile) {
    switch (profile) {
        case VAProfileH264ConstrainedBaseline:
        case VAProfileH264Baseline:
        case VAProfileH264Main:
        case VAProfileH264High: return CodecProfile::H264;
        case VAProfileHEVCMain:
        case VAProfileHEVCMain10: return CodecProfile::HEVC;
        case VAProfileVP9Profile0: return CodecProfile::VP9;
        case VAProfileAV1Profile0: return CodecProfile::AV1;
        default: return CodecProfile::Unknown;
    }
}

}  // namespace rockchip
