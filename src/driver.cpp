#include <drm/drm_fourcc.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>
#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_dec_av1.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <optional>
#include <span>
#include <mutex>
#include <memory>
#include <array>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include <xf86drm.h>

#include "mpp_decoder.h"
#include "util/bitstream.hpp"
#include "util/log.h"

using namespace rockchip;

namespace rockchip_vaapi {

namespace {

constexpr bool kExposePrime2Export = true;

bool envEnabledLocal(const char* name) {
    const char* value = std::getenv(name);
    if (!value) return false;
    return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0 || std::strcmp(value, "yes") == 0;
}

bool envDisabledLocal(const char* name) {
    const char* value = std::getenv(name);
    if (!value) return false;
    return std::strcmp(value, "0") == 0 || std::strcmp(value, "false") == 0 || std::strcmp(value, "no") == 0;
}

bool av1ExportAs10BitEnabled() {
    return !envDisabledLocal("ROCKCHIP_VAAPI_AV1_EXPORT_P010");
}

bool av1CorrectionEnabled() {
    return envEnabledLocal("ROCKCHIP_VAAPI_AV1_CORRECTION");
}

bool isH264Profile(VAProfile profile) {
    switch (profile) {
        case VAProfileH264ConstrainedBaseline:
        case VAProfileH264Baseline:
        case VAProfileH264Main:
        case VAProfileH264High:
            return true;
        default:
            return false;
    }
}

}  // namespace

namespace impl {

struct SurfaceState {
    MppDecoder::DecodedSurface surface;
    unique_fd dmabuf;

    void set_dmabuf(unique_fd fd) {
        dmabuf = std::move(fd);
        surface.dmabuf_fd = dmabuf.get();
    }
};

struct VABuffer {
    VABufferID id = VA_INVALID_ID;
    VABufferType type = static_cast<VABufferType>(0);
    std::vector<uint8_t> data;
    uint32_t num_elements = 0;

    VABuffer() = default;

    VABuffer(VABufferID id_, VABufferType type_, size_t size_, uint32_t num_elements_, const void* src)
        : id(id_), type(type_), data(size_), num_elements(num_elements_) {
        if (src && size_ > 0) {
            std::memcpy(data.data(), src, size_);
        }
    }

    std::span<uint8_t> map() {
        return std::span<uint8_t>(data);
    }

    std::span<const uint8_t> map() const {
        return std::span<const uint8_t>(data);
    }

    size_t size_bytes() const noexcept {
        return data.size();
    }
};

struct DecodeState {
    std::optional<VABufferID> picture_param_buffer_id;
    std::vector<VABufferID> slice_param_buffer_ids;
    std::vector<VABufferID> slice_data_buffer_ids;

    void clear() {
        picture_param_buffer_id.reset();
        slice_param_buffer_ids.clear();
        slice_data_buffer_ids.clear();
    }
};

struct PictureSubmissionState {
    VASurfaceID current_surface = VA_INVALID_ID;
    std::vector<VABufferID> current_buffers;
    DecodeState decode_state;
    std::vector<uint8_t> frame_buffer;
    std::vector<uint8_t> frame_extra_data;

    void reset() {
        decode_state.clear();
        current_buffers.clear();
        frame_buffer.clear();
        frame_extra_data.clear();
        current_surface = VA_INVALID_ID;
    }

    void beginPicture(VASurfaceID render_target) {
        reset();
        current_surface = render_target;
    }
};

struct PendingAv1Frame {
    bool valid = false;
    VASurfaceID surface = VA_INVALID_ID;
    uint8_t provisional_refresh_frame_flags = 0;
    bool sequence_enable_restoration = false;
    VADecPictureParameterBufferAV1 pic{};
    std::vector<uint8_t> tile_payload;

    void reset() {
        *this = {};
    }
};

struct CodecSessionState {
    bool sequence_headers_sent = false;
    unsigned int startup_prime_pictures_remaining = 0;
    std::vector<VASurfaceID> startup_prime_surfaces;
    size_t next_prime_surface_index = 0;
    bool av1_sequence_enable_restoration = false;
    std::array<VASurfaceID, 8> av1_ref_frame_map{};
    bool av1_ref_frame_map_valid = false;
    PendingAv1Frame pending_av1_frame;

    void beginContext(VAProfile profile) {
        sequence_headers_sent = false;
        startup_prime_pictures_remaining = startupPrimePicturesForProfile(profile);
        startup_prime_surfaces.clear();
        next_prime_surface_index = 0;
        av1_sequence_enable_restoration = false;
        av1_ref_frame_map.fill(VA_INVALID_SURFACE);
        av1_ref_frame_map_valid = false;
        pending_av1_frame.reset();
    }

    void endContext() {
        beginContext(VAProfileNone);
        startup_prime_pictures_remaining = 0;
    }

    unsigned int consumeStartupPrimeCopies(VAProfile profile) {
        if (!shouldPrimeStartup(profile) || startup_prime_pictures_remaining == 0) {
            return 0;
        }
        --startup_prime_pictures_remaining;
        return startupPrimeCopiesPerPicture(profile);
    }

    void noteSequenceHeaders(std::span<const uint8_t> extra_data) {
        if (!extra_data.empty()) {
            sequence_headers_sent = true;
        }
    }

    void setStartupPrimeSurfaces(std::span<const VASurfaceID> surfaces) {
        startup_prime_surfaces.assign(surfaces.begin(), surfaces.end());
        next_prime_surface_index = 0;
    }

    VASurfaceID nextStartupPrimeSurface() {
        if (startup_prime_surfaces.empty()) {
            return VA_INVALID_ID;
        }
        const VASurfaceID surface = startup_prime_surfaces[next_prime_surface_index % startup_prime_surfaces.size()];
        ++next_prime_surface_index;
        return surface;
    }

    static unsigned int startupPrimePicturesForProfile(VAProfile profile) {
        if (isH264Profile(profile)) {
            return 6;
        }
        if (profile == VAProfileHEVCMain || profile == VAProfileHEVCMain10) {
            return 4;
        }
        return 0;
    }

    static unsigned int startupPrimeCopiesPerPicture(VAProfile profile) {
        if (isH264Profile(profile)) {
            return 3;
        }
        if (profile == VAProfileHEVCMain || profile == VAProfileHEVCMain10) {
            return 1;
        }
        return 0;
    }

    static unsigned int startupPrimeSurfaceCountForProfile(VAProfile profile) {
        if (isH264Profile(profile)) {
            return 3;
        }
        if (profile == VAProfileHEVCMain || profile == VAProfileHEVCMain10) {
            return 2;
        }
        return 0;
    }

    static bool shouldPrimeStartup(VAProfile profile) {
        return isH264Profile(profile) || profile == VAProfileHEVCMain || profile == VAProfileHEVCMain10;
    }
};

struct DriverState {
    DriverState() : decoder(std::make_unique<MppDecoder>()) {}

    struct ConfigState {
        VAProfile profile = VAProfileNone;
        uint32_t rt_format = VA_RT_FORMAT_YUV420;
    };

    struct ContextState {
        VAConfigID config_id = VA_INVALID_ID;
        VAProfile profile = VAProfileNone;
        uint32_t rt_format = VA_RT_FORMAT_YUV420;
        std::vector<VASurfaceID> prime_surfaces;
        size_t next_prime_surface_index = 0;
    };

    std::unique_ptr<MppDecoder> decoder;
    std::mutex resource_mutex;
    std::unordered_map<VASurfaceID, std::unique_ptr<SurfaceState>> surfaces;
    std::unordered_map<VABufferID, std::unique_ptr<VABuffer>> buffers;
    VASurfaceID next_surface_id = 1;
    VABufferID next_buffer_id = 1;
    PictureSubmissionState picture;
    CodecSessionState session;
    VADriverVTable* vtable = nullptr;
    std::mutex lock;
    VAProfile profile = VAProfileNone;
    std::unordered_map<VAConfigID, ConfigState> configs;
    VAConfigID next_config_id = 1;
    std::unordered_map<VAContextID, ContextState> active_contexts;
    VAContextID next_context_id = 1;
    VASurfaceID next_internal_surface_id = 0x40000000u;
    std::vector<VADisplayAttribute> display_attributes;
    unique_fd resolved_drm_fd;
    int drm_fd = -1;
    dev_t drm_rdev = 0;
    std::string drm_device_path;
    std::string drm_driver_name;

    // Current expected decoding resolution (from vaCreateContext or MPP feedback).
    uint32_t picture_width = 0;
    uint32_t picture_height = 0;

    // Images created via vaCreateImage / vaDeriveImage.
    struct ImageState {
        VAImage image;
        VASurfaceID derived_surface = VA_INVALID_ID;
        unique_fd direct_dmabuf;
        void* direct_map = nullptr;
        size_t direct_map_size = 0;
        std::vector<uint8_t> palette;
    };

    // Local sync status for vaSyncSurface and fence emulation.
    std::mutex sync_mutex;
    std::condition_variable sync_cv;
    std::unordered_map<VASurfaceID, std::atomic<bool>> ready_flags;
    std::unordered_map<VAImageID, ImageState> images;
    VAImageID next_image_id = 1;

    // Subpictures created via vaCreateSubpicture.
    struct SubpictureState {
        VAImageID image_id = VA_INVALID_ID;
        unsigned int chromakey_min = 0;
        unsigned int chromakey_max = 0;
        unsigned int chromakey_mask = 0;
        float global_alpha = 1.0f;
        std::vector<VASurfaceID> target_surfaces;
        short src_x = 0;
        short src_y = 0;
        unsigned short src_width = 0;
        unsigned short src_height = 0;
        short dest_x = 0;
        short dest_y = 0;
        unsigned short dest_width = 0;
        unsigned short dest_height = 0;
        unsigned int flags = 0;
    };
    std::unordered_map<VASubpictureID, SubpictureState> subpictures;
    VASubpictureID next_subpicture_id = 1;
};

class MappedDmabuf {
   public:
    MappedDmabuf() = default;
    MappedDmabuf(void* address, size_t size) noexcept : address_(address), size_(size) {}
    ~MappedDmabuf() { reset(); }

    MappedDmabuf(const MappedDmabuf&) = delete;
    MappedDmabuf& operator=(const MappedDmabuf&) = delete;

    MappedDmabuf(MappedDmabuf&& other) noexcept
        : address_(std::exchange(other.address_, nullptr)), size_(std::exchange(other.size_, 0)) {}

    MappedDmabuf& operator=(MappedDmabuf&& other) noexcept {
        if (this != &other) {
            reset();
            address_ = std::exchange(other.address_, nullptr);
            size_ = std::exchange(other.size_, 0);
        }
        return *this;
    }

    [[nodiscard]] static MappedDmabuf map(int fd, size_t size, int prot) noexcept {
        if (fd < 0 || size == 0) return {};
        void* address = mmap(nullptr, size, prot, MAP_SHARED, fd, 0);
        return address == MAP_FAILED ? MappedDmabuf{} : MappedDmabuf{address, size};
    }

    [[nodiscard]] void* data() const noexcept { return address_; }
    [[nodiscard]] size_t size() const noexcept { return size_; }
    [[nodiscard]] explicit operator bool() const noexcept { return address_ != nullptr; }

    [[nodiscard]] void* release() noexcept {
        void* address = address_;
        address_ = nullptr;
        size_ = 0;
        return address;
    }

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

static constexpr uint32_t alignTo(uint32_t value, uint32_t align) {
    assert(align && (align & (align - 1)) == 0);
    return (value + align - 1) & ~(align - 1);
}

static uint8_t deriveAv1RefreshFrameFlags(const VADecPictureParameterBufferAV1& pic,
                                          const std::array<VASurfaceID, 8>& previous_ref_frame_map,
                                          bool previous_ref_frame_map_valid,
                                          VASurfaceID current_surface) {
    const uint8_t frame_type = static_cast<uint8_t>(pic.pic_info_fields.bits.frame_type & 0x3);
    const bool show_frame = pic.pic_info_fields.bits.show_frame != 0;
    const bool is_key_show_frame = frame_type == 0 && show_frame;
    const bool is_intra_only = frame_type == 2;
    const bool is_switch_frame = frame_type == 3;
    if (is_key_show_frame || is_intra_only) {
        return 0xff;
    }

    uint8_t flags = 0;
    if (previous_ref_frame_map_valid) {
        for (uint8_t slot = 0; slot < 8; ++slot) {
            if (pic.ref_frame_map[slot] != previous_ref_frame_map[slot] &&
                pic.ref_frame_map[slot] == current_surface) {
                flags |= static_cast<uint8_t>(1u << slot);
            }
        }
    }

    if (flags == 0 && is_switch_frame) {
        return 0xff;
    }

    return flags;
}

static uint8_t provisionalAv1RefreshFrameFlags(const VADecPictureParameterBufferAV1& pic) {
    const uint8_t frame_type = static_cast<uint8_t>(pic.pic_info_fields.bits.frame_type & 0x3);
    const bool show_frame = pic.pic_info_fields.bits.show_frame != 0;
    const bool is_key_show_frame = frame_type == 0 && show_frame;
    const bool is_intra_only = frame_type == 2;
    const bool is_switch_frame = frame_type == 3;
    if (is_key_show_frame || is_intra_only || is_switch_frame) {
        return 0xff;
    }
    return 0;
}

static void resetPictureState(DriverState* d) {
    if (!d) return;
    d->picture.reset();
}

struct ScopedPictureStateReset {
    DriverState* d = nullptr;
    ~ScopedPictureStateReset() { resetPictureState(d); }
};

static void destroyBuffer(DriverState* d, VABufferID buf_id) {
    if (!d) return;
    std::lock_guard<std::mutex> lock(d->resource_mutex);
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return;
    d->buffers.erase(it);
}

static bool isImageBackingBuffer(const DriverState* d, VABufferID buf_id) {
    if (!d) return false;
    for (const auto& kv : d->images) {
        if (kv.second.image.buf == buf_id) {
            return true;
        }
    }
    return false;
}

static uint64_t surfaceTotalSizeBytes(const MppDecoder::DecodedSurface& surf) {
    const uint32_t stride = surf.stride ? surf.stride : alignTo(surf.width, 64);
    if (!surf.is_10bit) {
        return static_cast<uint64_t>(stride) * surf.height * 3 / 2;
    }

    const uint32_t aligned_h = alignTo(surf.height, 8);
    return static_cast<uint64_t>(stride) * aligned_h * 3;
}

static bool imageLayoutMatchesSurface(const DriverState::ImageState& image_state,
                                      const MppDecoder::DecodedSurface& surf) {
    const uint32_t bytes_per_row = surf.stride * (surf.is_10bit ? 2u : 1u);
    const uint32_t aligned_h = surf.is_10bit ? alignTo(surf.height, 8) : surf.height;
    const uint32_t y_size = bytes_per_row * aligned_h;
    const uint32_t expected_fourcc = surf.is_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;

    return image_state.image.width == surf.width &&
           image_state.image.height == surf.height &&
           image_state.image.format.fourcc == expected_fourcc &&
           image_state.image.format.depth == 12 &&
           image_state.image.num_planes == 2 &&
           image_state.image.pitches[0] == bytes_per_row &&
           image_state.image.pitches[1] == bytes_per_row &&
           image_state.image.offsets[0] == 0 &&
           image_state.image.offsets[1] == y_size &&
           image_state.image.data_size == surfaceTotalSizeBytes(surf);
}

static void releaseDirectImageMap(DriverState::ImageState& image_state) {
    if (image_state.direct_map != nullptr && image_state.direct_map_size != 0) {
        munmap(image_state.direct_map, image_state.direct_map_size);
    }
    image_state.direct_map = nullptr;
    image_state.direct_map_size = 0;
}

static void releaseDirectImageAlias(DriverState::ImageState& image_state) {
    releaseDirectImageMap(image_state);
    image_state.direct_dmabuf.reset();
    image_state.derived_surface = VA_INVALID_ID;
}

static bool attachImageToSurfaceDmabuf(DriverState::ImageState& image_state,
                                       VASurfaceID surface,
                                       const MppDecoder::DecodedSurface& surf) {
    if (surf.dmabuf_fd < 0 || !imageLayoutMatchesSurface(image_state, surf)) {
        return false;
    }

    unique_fd exported_fd{dup(surf.dmabuf_fd)};
    if (!exported_fd) {
        return false;
    }

    releaseDirectImageAlias(image_state);
    image_state.direct_dmabuf = std::move(exported_fd);
    image_state.derived_surface = surface;
    return true;
}

static void* mapDirectImageBuffer(DriverState::ImageState& image_state) {
    if (image_state.direct_map != nullptr) {
        return image_state.direct_map;
    }
    if (!image_state.direct_dmabuf) {
        return nullptr;
    }

    auto mapping = MappedDmabuf::map(image_state.direct_dmabuf.get(), image_state.image.data_size, PROT_READ | PROT_WRITE);
    if (!mapping) {
        return nullptr;
    }

    image_state.direct_map_size = mapping.size();
    image_state.direct_map = mapping.release();
    return image_state.direct_map;
}

static constexpr uint32_t supportedRtFormats() {
    return VA_RT_FORMAT_YUV420 | VA_RT_FORMAT_YUV420_10BPP;
}

static bool rtFormatRequires10BitSurface(uint32_t rt_format) {
    const bool wants_10bit = (rt_format & VA_RT_FORMAT_YUV420_10BPP) != 0;
    const bool wants_8bit = (rt_format & VA_RT_FORMAT_YUV420) != 0;
    return wants_10bit && !wants_8bit;
}

static constexpr uint32_t kMaxPictureWidth = 8192;
static constexpr uint32_t kMaxPictureHeight = 8192;

struct ResolvedDrmDevice {
    unique_fd owned_fd;
    int fd = -1;
    dev_t rdev = 0;
    std::string path;
    std::string driver_name;
};

static constexpr unsigned int kDrmRenderMajor = 226;

static bool isRenderNodeDevice(const struct stat& stat_buf) {
    return S_ISCHR(stat_buf.st_mode) && major(stat_buf.st_rdev) == kDrmRenderMajor && minor(stat_buf.st_rdev) >= 128;
}

static bool isAcceptedDrmDriverName(std::string_view driver_name) {
    return driver_name.find("rockchip") != std::string_view::npos || driver_name.find("panthor") != std::string_view::npos;
}

static Expected<void, VAStatus> validateResolvedDrmFd(int fd,
                                                      bool require_supported_driver,
                                                      dev_t* out_rdev,
                                                      std::string* out_path,
                                                      std::string* out_driver_name) {
    if (fd < 0) return VA_STATUS_ERROR_INVALID_DISPLAY;

    struct stat stat_buf = {};
    if (fstat(fd, &stat_buf) != 0 || !isRenderNodeDevice(stat_buf)) {
        return VA_STATUS_ERROR_INVALID_DISPLAY;
    }

    drmVersionPtr version = drmGetVersion(fd);
    if (!version) {
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    const std::string driver_name = version->name ? version->name : "";
    drmFreeVersion(version);
    if (require_supported_driver && !isAcceptedDrmDriverName(driver_name)) {
        return VA_STATUS_ERROR_INVALID_DISPLAY;
    }

    char* device_name = drmGetRenderDeviceNameFromFd(fd);
    if (device_name) {
        if (out_path) *out_path = device_name;
        free(device_name);
    } else if (out_path) {
        *out_path = (std::filesystem::path("/proc") / "self" / "fd" / std::to_string(fd)).string();
    }
    if (out_driver_name) *out_driver_name = driver_name;
    if (out_rdev) *out_rdev = stat_buf.st_rdev;
    return {};
}

static std::optional<int> drmFdFromContext(VADriverContextP ctx) {
    if (!ctx) return std::nullopt;
    if ((ctx->display_type & VA_DISPLAY_DRM) != 0) {
        if (ctx->drm_state) {
            auto* drm = static_cast<drm_state*>(ctx->drm_state);
            if (drm->fd >= 0) return drm->fd;
        }

        const intptr_t native_value = reinterpret_cast<intptr_t>(ctx->native_dpy);
        if (native_value >= 0 && native_value <= std::numeric_limits<int>::max()) {
            return static_cast<int>(native_value);
        }
    }
    return std::nullopt;
}

static Expected<ResolvedDrmDevice, VAStatus> resolveDrmDeviceFromLibdrmScan() {
    std::array<drmDevicePtr, 64> devices{};
    const int device_count = drmGetDevices2(0, devices.data(), static_cast<int>(devices.size()));
    if (device_count < 0) {
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    for (int index = 0; index < device_count; ++index) {
        drmDevicePtr device_info = devices[index];
        if (!device_info) {
            continue;
        }
        if ((device_info->available_nodes & (1 << DRM_NODE_RENDER)) == 0) {
            continue;
        }

        const char* render_node = device_info->nodes[DRM_NODE_RENDER];
        if (!render_node) {
            continue;
        }

        unique_fd fd{open(render_node, O_RDWR | O_CLOEXEC)};
        if (!fd) {
            continue;
        }

        ResolvedDrmDevice device;
        device.owned_fd = std::move(fd);
        device.fd = device.owned_fd.get();
        auto status = validateResolvedDrmFd(device.fd, true, &device.rdev, &device.path, &device.driver_name);
        if (!status) {
            continue;
        }

        drmFreeDevices(devices.data(), device_count);
        return device;
    }

    drmFreeDevices(devices.data(), device_count);
    return VA_STATUS_ERROR_INVALID_DISPLAY;
}

static Expected<ResolvedDrmDevice, VAStatus> resolveDrmDevice(VADriverContextP ctx) {
    if (auto inherited_fd = drmFdFromContext(ctx)) {
        ResolvedDrmDevice device;
        device.fd = *inherited_fd;
        auto status = validateResolvedDrmFd(device.fd, true, &device.rdev, &device.path, &device.driver_name);
        if (status) {
            return device;
        }

        auto fallback = resolveDrmDeviceFromLibdrmScan();
        if (fallback) {
            util::log(util::stdout_sink, util::LogLevel::Warn,
                      "Falling back from inherited DRM fd {} to compatible render node {} (driver={})",
                      *inherited_fd, fallback.value().path, fallback.value().driver_name);
            return fallback;
        }

        return status.error();
    }

    return resolveDrmDeviceFromLibdrmScan();
}

static bool profileSupports10BitOutput(VAProfile profile) {
    switch (profile) {
        case VAProfileHEVCMain10:
            return true;
        case VAProfileAV1Profile0:
            return true;
        default:
            return false;
    }
}

static void ensureDisplayAttributesInitialized(DriverState& driver) {
    if (!driver.display_attributes.empty()) return;

    auto add = [&](VADisplayAttribType type, int32_t min_v, int32_t max_v, int32_t default_v, uint32_t flags) {
        VADisplayAttribute attr = {};
        attr.type = type;
        attr.min_value = min_v;
        attr.max_value = max_v;
        attr.value = default_v;
        attr.flags = flags;
        driver.display_attributes.push_back(attr);
    };

    // Basic video color controls.
    add(VADisplayAttribBrightness, 0, 100, 50, VA_DISPLAY_ATTRIB_GETTABLE | VA_DISPLAY_ATTRIB_SETTABLE);
    add(VADisplayAttribContrast, 0, 100, 50, VA_DISPLAY_ATTRIB_GETTABLE | VA_DISPLAY_ATTRIB_SETTABLE);
    add(VADisplayAttribHue, -180, 180, 0, VA_DISPLAY_ATTRIB_GETTABLE | VA_DISPLAY_ATTRIB_SETTABLE);
    add(VADisplayAttribSaturation, 0, 100, 50, VA_DISPLAY_ATTRIB_GETTABLE | VA_DISPLAY_ATTRIB_SETTABLE);

    // For legacy clients; we do not use direct surface semantics but expose it.
    add(VADisplayAttribDirectSurface, 0, 1, 0, VA_DISPLAY_ATTRIB_GETTABLE);
}

static DriverState* toDriver(VADriverContextP ctx) {
    return reinterpret_cast<DriverState*>(ctx->pDriverData);
}

static bool isSupportedProfile(VAProfile profile) {
    switch (profile) {
        case VAProfileH264ConstrainedBaseline:
        case VAProfileH264Baseline:
        case VAProfileH264Main:
        case VAProfileH264High:
        case VAProfileHEVCMain:
        case VAProfileHEVCMain10:
        case VAProfileVP9Profile0:
        case VAProfileAV1Profile0:
            return true;
        default:
            return false;
    }
}

template <auto Member, typename Fn>
static constexpr auto make_entry(Fn* fn) {
    return std::pair{Member, fn};
}

}  // namespace impl

using namespace impl;

/* api: VA driver callbacks (VADriverVTable functions only).
 * impl: internal helpers and state logic.
 */
namespace api {

static VAStatus vaCreateSurfaces1(VADriverContextP ctx,
                                  int width,
                                  int height,
                                  int format,
                                  int num_surfaces,
                                  VASurfaceID* surfaces,
                                  const VASurfaceAttrib* attrib_list,
                                  int num_attribs) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    if (width <= 0 || height <= 0 || num_surfaces <= 0 || !surfaces)
        return VA_STATUS_ERROR_INVALID_PARAMETER;

    const bool format_supported = (format == static_cast<int>(VA_RT_FORMAT_YUV420) ||
                                   format == static_cast<int>(VA_RT_FORMAT_YUV420_10) ||
                                   format == static_cast<int>(supportedRtFormats()));
    if (!format_supported) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "vaCreateSurfaces1: unsupported format={}", format);
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    const bool wants_10bit = (static_cast<uint32_t>(format) & VA_RT_FORMAT_YUV420_10BPP) != 0;
    const uint32_t expected_fourcc = wants_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;

    for (int i = 0; i < num_attribs; i++) {
        const auto& attr = attrib_list[i];
        switch (attr.type) {
            case VASurfaceAttribNone:
                break;
            case VASurfaceAttribPixelFormat:
            case VASurfaceAttribMemoryType:
            case VASurfaceAttribUsageHint:
            case VASurfaceAttribExternalBufferDescriptor:
            case VASurfaceAttribDRMFormatModifiers:
                if (attr.value.type != VAGenericValueTypeInteger) {
                    if (attr.type == VASurfaceAttribPixelFormat || attr.type == VASurfaceAttribMemoryType || attr.type == VASurfaceAttribUsageHint) {
                        return VA_STATUS_ERROR_INVALID_PARAMETER;
                    }
                }
                if (attr.type == VASurfaceAttribPixelFormat) {
                    if (static_cast<uint32_t>(attr.value.value.i) != expected_fourcc) {
                        util::log(util::stderr_sink, util::LogLevel::Error,
                                  "vaCreateSurfaces1: unsupported pixel format {}", attr.value.value.i);
                        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
                    }
                }
                break;
            case VASurfaceAttribMinWidth:
            case VASurfaceAttribMaxWidth:
            case VASurfaceAttribMinHeight:
            case VASurfaceAttribMaxHeight:
                // Read-only capability hints from vaQuerySurfaceAttributes; ignore on create.
                break;
            default:
                util::log(util::stderr_sink, util::LogLevel::Warn,
                          "vaCreateSurfaces1: unsupported attrib type {}", attr.type);
                return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
    }

    std::lock_guard<std::mutex> lock(d->lock);
    if (wants_10bit && !profileSupports10BitOutput(d->profile)) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "vaCreateSurfaces1: profile {} does not support 10-bit surfaces", static_cast<int>(d->profile));
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }
    {
        static std::atomic<int> create_surface_log_count{0};
        if (create_surface_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
            int initialized = d->decoder && d->decoder->isInitialized() ? 1 : 0;
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "vaCreateSurfaces1: decoder isInitialized={}", initialized);
        }
    }

    for (int i = 0; i < num_surfaces; i++) {
        VASurfaceID id = d->next_surface_id++;
        auto state = std::make_unique<SurfaceState>();
        state->surface.va_id = id;
        state->surface.width = static_cast<uint32_t>(width);
        state->surface.height = static_cast<uint32_t>(height);
        state->surface.stride = alignTo(state->surface.width, 64);
        state->surface.is_10bit = wants_10bit;

        if (!d->decoder->allocateSurface(id, state->surface, width, height)) {
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "Failed to allocate MPP surface for VA surface {}", id);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }

        state->surface.dmabuf_fd = -1;

        d->surfaces[id] = std::move(state);
        surfaces[i] = id;
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaCreateSurfaces(VADriverContextP ctx,
                                 int width,
                                 int height,
                                 int format,
                                 int num_surfaces,
                                 VASurfaceID* surfaces) {
    return vaCreateSurfaces1(ctx, width, height, format, num_surfaces, surfaces, nullptr, 0);
}

static VAStatus vaDestroySurfaces(VADriverContextP ctx,
                                  VASurfaceID* surfaces,
                                  int num_surfaces) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    std::lock_guard<std::mutex> lock(d->lock);
    for (int i = 0; i < num_surfaces; i++) {
        d->decoder->destroySurface(surfaces[i]);
        auto it = d->surfaces.find(surfaces[i]);
        if (it != d->surfaces.end()) {
            it->second->set_dmabuf({});
            d->surfaces.erase(it);
        }
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaSyncSurface(VADriverContextP ctx, VASurfaceID surface) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    {
        std::lock_guard<std::mutex> lock(d->lock);
        const auto it = d->surfaces.find(surface);
        if (it == d->surfaces.end()) {
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "vaSyncSurface: surface={} not tracked", surface);
            return VA_STATUS_SUCCESS;
        }
    }

    // Wait for MPP to report either completion or failure. Avoid an extra
    // ready-only wait path so failed frames do not deadlock callers.
    bool ready = d->decoder->waitSurfaceReady(surface);

    uint32_t width = 0, height = 0, stride = 0;
    int fd = -1;
    bool failed = false;
    bool pending = false;
    if (d->decoder->getSurfaceInfo(surface, width, height, stride, fd, failed, pending)) {
        if (!ready && failed) {
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "vaSyncSurface: surface={} completed with decode failure", surface);
            return VA_STATUS_ERROR_DECODING_ERROR;
        }

        if (!ready && !failed && pending) {
            uint64_t last_submitted_job_id = 0;
            uint64_t last_completed_job_id = 0;
            uint64_t last_submit_us = 0;
            uint64_t last_complete_us = 0;
            d->decoder->getSurfaceDebugInfo(surface,
                                            last_submitted_job_id,
                                            last_completed_job_id,
                                            last_submit_us,
                                            last_complete_us);
            const uint64_t now_us = steadyMicrosNow();
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "vaSyncSurface: surface={} waitSurfaceReady timed out submitted_job={} completed_job={} since_submit_ms={} since_complete_ms={}",
                      surface,
                      static_cast<unsigned long long>(last_submitted_job_id),
                      static_cast<unsigned long long>(last_completed_job_id),
                      last_submit_us == 0 ? 0ull : static_cast<unsigned long long>((now_us - last_submit_us) / 1000),
                      last_complete_us == 0 ? 0ull : static_cast<unsigned long long>((now_us - last_complete_us) / 1000));
            return VA_STATUS_ERROR_TIMEDOUT;
        }

        if (!ready && !failed && !pending) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "vaSyncSurface: surface={} has no decoded output yet; treating idle sync as success",
                      surface);
            return VA_STATUS_SUCCESS;
        }

        if (failed || fd < 0) {
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "vaSyncSurface: surface={} failed={} fd={}", surface, (int)failed, fd);
            return failed ? VA_STATUS_ERROR_DECODING_ERROR : VA_STATUS_ERROR_INVALID_SURFACE;
        }

        {
    static std::atomic<int> sync_wait_log_count{0};
    if (sync_wait_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaSyncSurface: waiting surface={}", surface);
    }

            std::lock_guard<std::mutex> lock(d->lock);
            const auto it = d->surfaces.find(surface);
            if (it == d->surfaces.end()) {
                return VA_STATUS_ERROR_INVALID_SURFACE;
            }

            unique_fd driver_fd{dup(fd)};
            if (!driver_fd) {
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }

            it->second->surface.width = width;
            it->second->surface.height = height;
            it->second->surface.stride = stride;
            it->second->set_dmabuf(std::move(driver_fd));

            static std::atomic<int> sync_ready_log_count{0};
            if (sync_ready_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "vaSyncSurface: surface={} ready w={} h={} stride={} fd={}",
                          surface, width, height, stride, it->second->surface.dmabuf_fd);
            }
        }
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaQueryConfigProfiles(VADriverContextP ctx,
                                      VAProfile* profile_list,
                                      int* num_profiles) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    static std::atomic<int> trace_count{0};
    if (trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaQueryConfigProfiles: profile_list={} num_profiles={}",
                  static_cast<void*>(profile_list), num_profiles ? *num_profiles : -1);
    }
    constexpr VAProfile supported[] = {
        VAProfileH264ConstrainedBaseline,
        VAProfileH264Baseline,
        VAProfileH264Main,
        VAProfileH264High,
        VAProfileHEVCMain,
        VAProfileHEVCMain10,
        VAProfileAV1Profile0,
        VAProfileVP9Profile0,
    };
    const int supported_count = static_cast<int>(std::size(supported));

    if (!num_profiles)
        return VA_STATUS_ERROR_INVALID_PARAMETER;

    // When profiles == nullptr, libva expects the driver to set the count
    // and return success.
    if (!profile_list) {
        *num_profiles = supported_count;
        return VA_STATUS_SUCCESS;
    }

    // Some Firefox probes pass a preallocated buffer with the input count set
    // to 0 and expect the driver to populate the full list anyway.
    int requested = *num_profiles;
    if (requested <= 0) {
        requested = supported_count;
    }

    // Always return success and report the full supported count.
    int fill = std::min(requested, supported_count);
    for (int i = 0; i < fill; i++) {
        profile_list[i] = supported[i];
    }
    *num_profiles = supported_count;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaQueryConfigEntrypoints(VADriverContextP ctx,
                                         VAProfile profile,
                                         VAEntrypoint* entrypoint_list,
                                         int* num_entrypoints) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    static std::atomic<int> trace_count{0};
    if (trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaQueryConfigEntrypoints: profile={} entrypoint_list={} num_entrypoints={}",
                  static_cast<int>(profile), static_cast<void*>(entrypoint_list), num_entrypoints ? *num_entrypoints : -1);
    }
    if (!isSupportedProfile(profile)) return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    if (!num_entrypoints) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (!entrypoint_list) {
        *num_entrypoints = 1;
        return VA_STATUS_SUCCESS;
    }
    entrypoint_list[0] = VAEntrypointVLD;
    *num_entrypoints = 1;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaGetConfigAttributes(VADriverContextP ctx,
                                      VAProfile profile,
                                      VAEntrypoint entrypoint,
                                      VAConfigAttrib* attrib_list,
                                      int num_attribs) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    static std::atomic<int> trace_count{0};
    if (trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetConfigAttributes: profile={} entrypoint={} attrib_list={} num_attribs={}",
                  static_cast<int>(profile), static_cast<int>(entrypoint), static_cast<void*>(attrib_list), num_attribs);
    }
    if (!isSupportedProfile(profile)) return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    if (!attrib_list) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (entrypoint != VAEntrypointVLD) return VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;

    for (int i = 0; i < num_attribs; i++) {
        switch (attrib_list[i].type) {
            case VAConfigAttribRTFormat:
                attrib_list[i].value = supportedRtFormats();
                break;
            case VAConfigAttribMaxPictureWidth:
                attrib_list[i].value = kMaxPictureWidth;
                break;
            case VAConfigAttribMaxPictureHeight:
                attrib_list[i].value = kMaxPictureHeight;
                break;
            default:
                attrib_list[i].value = VA_ATTRIB_NOT_SUPPORTED;
                break;
        }
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaCreateConfig(VADriverContextP ctx,
                               VAProfile profile,
                               VAEntrypoint entrypoint,
                               VAConfigAttrib* attrib_list,
                               int num_attribs,
                               VAConfigID* config_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    static std::atomic<int> trace_count{0};
    if (trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaCreateConfig: profile={} entrypoint={} attrib_list={} num_attribs={}",
                  static_cast<int>(profile), static_cast<int>(entrypoint), static_cast<void*>(attrib_list), num_attribs);
    }
    if (!isSupportedProfile(profile)) return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    if (entrypoint != VAEntrypointVLD) return VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;
    if (!config_id) return VA_STATUS_ERROR_INVALID_PARAMETER;

    std::lock_guard<std::mutex> lock(d->lock);

    // We support only a single config, but still retain the requested RT format
    // so surface queries can choose the correct pixel format.
    uint32_t requested_rt_format = VA_RT_FORMAT_YUV420;
    if (attrib_list && num_attribs > 0) {
        for (int i = 0; i < num_attribs; i++) {
            if (attrib_list[i].type == VAConfigAttribRTFormat) {
                requested_rt_format = static_cast<uint32_t>(attrib_list[i].value) & supportedRtFormats();
                if (requested_rt_format == 0) {
                    return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
                }
                attrib_list[i].value = requested_rt_format;
            }
        }
    }

    const VAConfigID new_config_id = d->next_config_id++;
    d->configs.emplace(new_config_id, DriverState::ConfigState{.profile = profile, .rt_format = requested_rt_format});
    d->profile = profile;
    *config_id = new_config_id;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyConfig(VADriverContextP ctx, VAConfigID config_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    std::lock_guard<std::mutex> lock(d->lock);

    auto config_it = d->configs.find(config_id);
    if (config_it == d->configs.end()) {
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }

    d->configs.erase(config_it);
    return VA_STATUS_SUCCESS;
}

static VAStatus vaQueryConfigAttributes(VADriverContextP ctx,
                                        VAConfigID config_id,
                                        VAProfile* profile,
                                        VAEntrypoint* entrypoint,
                                        VAConfigAttrib* attrib_list,
                                        int* num_attribs) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    static std::atomic<int> trace_count{0};
    if (trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaQueryConfigAttributes: config_id={} profile_ptr={} entrypoint_ptr={} attrib_list={} num_attribs_ptr={}",
                  config_id, static_cast<void*>(profile), static_cast<void*>(entrypoint), static_cast<void*>(attrib_list), static_cast<void*>(num_attribs));
    }
    if (!num_attribs) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    std::lock_guard<std::mutex> lock(d->lock);
    auto config_it = d->configs.find(config_id);
    if (config_it == d->configs.end()) return VA_STATUS_ERROR_INVALID_CONFIG;
    const auto& config = config_it->second;

    if (profile) *profile = config.profile;
    if (entrypoint) *entrypoint = VAEntrypointVLD;

    constexpr int kConfigAttribCount = 3;
    if (!attrib_list) {
        *num_attribs = kConfigAttribCount;
        return VA_STATUS_SUCCESS;
    }

    if (*num_attribs < kConfigAttribCount) {
        *num_attribs = kConfigAttribCount;
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }

    attrib_list[0].type = VAConfigAttribRTFormat;
    attrib_list[0].value = config.rt_format;
    attrib_list[1].type = VAConfigAttribMaxPictureWidth;
    attrib_list[1].value = kMaxPictureWidth;
    attrib_list[2].type = VAConfigAttribMaxPictureHeight;
    attrib_list[2].value = kMaxPictureHeight;
    *num_attribs = kConfigAttribCount;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaQuerySurfaceAttributes(VADriverContextP ctx,
                                         VAConfigID config_id,
                                         VASurfaceAttrib* attrib_list,
                                         unsigned int* num_attribs) {
    if (!ctx || !num_attribs) return VA_STATUS_ERROR_INVALID_PARAMETER;
    static std::atomic<int> trace_count{0};
    if (trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaQuerySurfaceAttributes: config_id={} attrib_list={} num_attribs={}",
                  config_id, static_cast<void*>(attrib_list), *num_attribs);
    }
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    std::lock_guard<std::mutex> lock(d->lock);
    uint32_t config_rt_format = VA_RT_FORMAT_YUV420;
    if (config_id != VA_INVALID_ID) {
        auto config_it = d->configs.find(config_id);
        if (config_it == d->configs.end()) return VA_STATUS_ERROR_INVALID_CONFIG;
        config_rt_format = config_it->second.rt_format;
    }

    const bool uses_10bit_output = rtFormatRequires10BitSurface(config_rt_format);
    static uint64_t linear_modifier = DRM_FORMAT_MOD_LINEAR;
    static VADRMFormatModifierList linear_modifier_list = {
        .num_modifiers = 1,
        .modifiers = &linear_modifier,
    };
    constexpr unsigned int kSupportedCount = 3;
    if (!attrib_list) {
        *num_attribs = kSupportedCount;
        return VA_STATUS_SUCCESS;
    }

    if (*num_attribs < kSupportedCount) {
        *num_attribs = kSupportedCount;
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }

    attrib_list[0].type = VASurfaceAttribPixelFormat;
    attrib_list[0].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    attrib_list[0].value.type = VAGenericValueTypeInteger;
    attrib_list[0].value.value.i = uses_10bit_output ? VA_FOURCC_P010 : VA_FOURCC_NV12;

    attrib_list[1].type = VASurfaceAttribMemoryType;
    attrib_list[1].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    attrib_list[1].value.type = VAGenericValueTypeInteger;
    attrib_list[1].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA | VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
    if (kExposePrime2Export) {
        attrib_list[1].value.value.i |= VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
    }

    attrib_list[2].type = VASurfaceAttribDRMFormatModifiers;
    attrib_list[2].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attrib_list[2].value.type = VAGenericValueTypePointer;
    attrib_list[2].value.value.p = &linear_modifier_list;

    *num_attribs = kSupportedCount;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaGetSurfaceAttributes(VADriverContextP ctx,
                                     VAConfigID config_id,
                                     VASurfaceAttrib* attrib_list,
                                     unsigned int num_attribs) {
    unsigned int count = num_attribs;
    return vaQuerySurfaceAttributes(ctx, config_id, attrib_list, &count);
}

static VAStatus vaCreateSurfaces2(VADriverContextP ctx,
                                  unsigned int format,
                                  unsigned int width,
                                  unsigned int height,
                                  VASurfaceID* surfaces,
                                  unsigned int num_surfaces,
                                  VASurfaceAttrib* attrib_list,
                                  unsigned int num_attribs) {
    return vaCreateSurfaces1(ctx,
                             static_cast<int>(width),
                             static_cast<int>(height),
                             static_cast<int>(format),
                             static_cast<int>(num_surfaces),
                             surfaces,
                             attrib_list,
                             static_cast<int>(num_attribs));
}

static VAStatus vaCreateContext(VADriverContextP ctx,
                                VAConfigID config_id,
                                int picture_width,
                                int picture_height,
                                int flag,
                                VASurfaceID* render_targets,
                                int num_render_targets,
                                VAContextID* context) {
    if (!ctx || !context) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: invalid pointer ctx={} context={}",
              static_cast<const void*>(ctx), static_cast<const void*>(context));
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    auto* d = toDriver(ctx);
    if (!d) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: invalid driver state");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
    std::lock_guard<std::mutex> lock(d->lock);
    auto config_it = d->configs.find(config_id);
    if (config_it == d->configs.end()) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: missing config_id={}", config_id);
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    const auto& config = config_it->second;
    if (num_render_targets <= 0 || !render_targets) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: invalid render targets count={} ptr={}",
              num_render_targets, static_cast<const void*>(render_targets));
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    if (flag != 0 && flag != VA_PROGRESSIVE) {
        util::log(util::stderr_sink, util::LogLevel::Warn,
                  "vaCreateContext: unsupported flag={} (expect 0/VA_PROGRESSIVE)", flag);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    for (int i = 0; i < num_render_targets; i++) {
        if (d->surfaces.find(render_targets[i]) == d->surfaces.end()) {
            util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: target surface {} not found", render_targets[i]);
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
    }

    if (!isSupportedProfile(config.profile)) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: unsupported profile={}", config.profile);
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    }

    if (d->active_contexts.empty()) {
        d->picture_width = static_cast<uint32_t>(picture_width);
        d->picture_height = static_cast<uint32_t>(picture_height);
        d->session.beginContext(config.profile);
        resetPictureState(d);
    }

    d->profile = config.profile;

    // Initialize decoder with the requested size; it may adjust later.
    if (!d->decoder || (!d->decoder->isInitialized() &&
                        !d->decoder->initialize(vaProfileToCodec(config.profile), picture_width, picture_height, d->drm_fd))) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    std::vector<VASurfaceID> prime_surfaces;
    const unsigned int prime_surface_count = CodecSessionState::startupPrimeSurfaceCountForProfile(config.profile);
    const bool wants_10bit_surfaces = rtFormatRequires10BitSurface(config.rt_format);
    for (unsigned int index = 0; index < prime_surface_count; ++index) {
        MppDecoder::DecodedSurface scratch_surface;
        scratch_surface.is_10bit = wants_10bit_surfaces;
        const VASurfaceID internal_surface_id = d->next_internal_surface_id++;
        if (!d->decoder->allocateSurface(internal_surface_id, scratch_surface, picture_width, picture_height)) {
            for (VASurfaceID allocated_surface : prime_surfaces) {
                d->decoder->destroySurface(allocated_surface);
            }
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "vaCreateContext: failed to allocate startup-prime surface profile={} index={}",
                      config.profile,
                      index);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        prime_surfaces.push_back(internal_surface_id);
    }
    if (d->active_contexts.empty()) {
        d->session.setStartupPrimeSurfaces(prime_surfaces);
    }

    const VAContextID new_context = d->next_context_id++;
    d->active_contexts.emplace(new_context, DriverState::ContextState{
        .config_id = config_id,
        .profile = config.profile,
        .rt_format = config.rt_format,
        .prime_surfaces = std::move(prime_surfaces),
    });
    static std::atomic<int> context_prime_log_count{0};
    if (context_prime_log_count.fetch_add(1, std::memory_order_relaxed) < 8) {
        const auto& context_state = d->active_contexts.at(new_context);
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaCreateContext: context={} profile={} rt_format=0x{:x} render_targets={} prime_surfaces={} first_prime_surface={}",
                  new_context,
                  config.profile,
                  config.rt_format,
                  num_render_targets,
                  context_state.prime_surfaces.size(),
                  context_state.prime_surfaces.empty() ? VA_INVALID_ID : context_state.prime_surfaces.front());
    }
    *context = new_context;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyContext(VADriverContextP ctx, VAContextID context) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->lock);
    auto context_it = d->active_contexts.find(context);
    if (context_it == d->active_contexts.end()) return VA_STATUS_ERROR_INVALID_CONTEXT;
    for (VASurfaceID surface : context_it->second.prime_surfaces) {
        d->decoder->destroySurface(surface);
    }
    d->active_contexts.erase(context_it);
    if (!d->active_contexts.empty()) return VA_STATUS_SUCCESS;

    // Stop any decoding activity.
    d->decoder->shutdown();

    // Release driver-owned surface handles after MPP has fully stopped.
    for (auto& kv : d->surfaces) {
        kv.second->set_dmabuf({});
    }

    d->surfaces.clear();
    d->ready_flags.clear();
    d->session.endContext();
    d->picture.reset();

    for (auto& kv : d->images) {
        releaseDirectImageAlias(kv.second);
        destroyBuffer(d, kv.second.image.buf);
    }
    d->images.clear();

    // Release any remaining non-image buffers after image-owned backing
    // storage has been dropped.
    d->buffers.clear();

    d->subpictures.clear();

    return VA_STATUS_SUCCESS;
}

static VAStatus vaCreateBuffer(VADriverContextP ctx,
                               VAContextID /*context*/,
                               VABufferType type,
                               unsigned int size,
                               unsigned int num_elements,
                               void* data,
                               VABufferID* buf_id) {
    auto* d = toDriver(ctx);
    if (!d || !buf_id) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->resource_mutex);

    size_t total_size = size * (size_t)num_elements;
    VABufferID id = d->next_buffer_id++;
    try {
        d->buffers[id] = std::make_unique<VABuffer>(id, type, total_size, num_elements, data);
    } catch (const std::bad_alloc&) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    *buf_id = id;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaBufferSetNumElements(VADriverContextP ctx,
                                       VABufferID buf_id,
                                       unsigned int num_elements) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;

    it->second->num_elements = num_elements;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaMapBuffer(VADriverContextP ctx,
                            VABufferID buf_id,
                            void** pbuf) {
    if (!pbuf) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->resource_mutex);
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) {
        for (auto& kv : d->images) {
            auto& image_state = kv.second;
            if (image_state.image.buf != buf_id) {
                continue;
            }

            void* mapped = mapDirectImageBuffer(image_state);
            if (!mapped) {
                return VA_STATUS_ERROR_INVALID_BUFFER;
            }

            *pbuf = mapped;
            return VA_STATUS_SUCCESS;
        }

        return VA_STATUS_ERROR_INVALID_BUFFER;
    }

    std::span<uint8_t> span = it->second->map();
    *pbuf = span.data();
    return VA_STATUS_SUCCESS;
}

static VAStatus vaUnmapBuffer(VADriverContextP ctx,
                              VABufferID buf_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->resource_mutex);
    if (d->buffers.find(buf_id) == d->buffers.end()) {
        for (auto& kv : d->images) {
            auto& image_state = kv.second;
            if (image_state.image.buf != buf_id) {
                continue;
            }

            releaseDirectImageMap(image_state);
            return VA_STATUS_SUCCESS;
        }

        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyBuffer(VADriverContextP ctx,
                                VABufferID buf_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->resource_mutex);
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) {
        return isImageBackingBuffer(d, buf_id) ? VA_STATUS_SUCCESS : VA_STATUS_ERROR_INVALID_BUFFER;
    }

    if (isImageBackingBuffer(d, buf_id)) {
        return VA_STATUS_SUCCESS;
    }

    d->buffers.erase(it);
    return VA_STATUS_SUCCESS;
}

static VAStatus vaBeginPicture(VADriverContextP ctx,
                               VAContextID context,
                               VASurfaceID render_target) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    {
        std::lock_guard<std::mutex> lock(d->lock);
        auto context_it = d->active_contexts.find(context);
        if (context_it == d->active_contexts.end()) return VA_STATUS_ERROR_INVALID_CONTEXT;
        d->profile = context_it->second.profile;
    }

    d->picture.beginPicture(render_target);
    {
        std::lock_guard<std::mutex> lock(d->sync_mutex);
        d->ready_flags[render_target].store(false);
    }
    d->sync_cv.notify_all();
    return VA_STATUS_SUCCESS;
}

static VAStatus vaEndPicture(VADriverContextP ctx,
                             VAContextID context) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    {
        std::lock_guard<std::mutex> lock(d->lock);
        auto context_it = d->active_contexts.find(context);
        if (context_it == d->active_contexts.end()) return VA_STATUS_ERROR_INVALID_CONTEXT;
        d->profile = context_it->second.profile;
    }
    ScopedPictureStateReset cleanup{d};
    auto& picture = d->picture;
    auto& session = d->session;

    // Ensure we have collected data for the current picture.
    if (picture.current_surface == VA_INVALID_ID) return VA_STATUS_ERROR_INVALID_SURFACE;

    if (!picture.decode_state.slice_data_buffer_ids.empty()) {
        std::vector<uint8_t> rebuilt_frame_buffer;
        for (size_t index = 0; index < picture.decode_state.slice_data_buffer_ids.size(); ++index) {
            auto data_it = d->buffers.find(picture.decode_state.slice_data_buffer_ids[index]);
            if (data_it == d->buffers.end()) {
                continue;
            }

            const auto& slice_data_buffer = *data_it->second;
            const auto slice_data = slice_data_buffer.map();
            const VABuffer* slice_param_buffer = nullptr;
            if (index < picture.decode_state.slice_param_buffer_ids.size()) {
                auto param_it = d->buffers.find(picture.decode_state.slice_param_buffer_ids[index]);
                if (param_it != d->buffers.end()) {
                    slice_param_buffer = param_it->second.get();
                }
            }

            std::vector<uint8_t> framed;
            if (slice_param_buffer && slice_param_buffer->num_elements > 0) {
                if (isH264Profile(d->profile) &&
                    slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferH264)) {
                    framed = bitstream::build_h264_annexb_slices(
                        reinterpret_cast<const VASliceParameterBufferH264*>(slice_param_buffer->data.data()),
                        slice_param_buffer->num_elements,
                        slice_data.data(),
                        slice_data.size());
                } else if ((d->profile == VAProfileHEVCMain || d->profile == VAProfileHEVCMain10) &&
                           slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferHEVC)) {
                    framed = bitstream::build_hevc_annexb_slices(
                        reinterpret_cast<const VASliceParameterBufferHEVC*>(slice_param_buffer->data.data()),
                        slice_param_buffer->num_elements,
                        slice_data.data(),
                        slice_data.size());
                } else if (d->profile == VAProfileAV1Profile0 &&
                           slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferAV1)) {
                    const auto* av1_slices = reinterpret_cast<const VASliceParameterBufferAV1*>(slice_param_buffer->data.data());
                    size_t total_size = 0;
                    bool valid = true;
                    for (uint32_t slice_index = 0; slice_index < slice_param_buffer->num_elements; ++slice_index) {
                        const auto offset = static_cast<size_t>(av1_slices[slice_index].slice_data_offset);
                        const auto size = static_cast<size_t>(av1_slices[slice_index].slice_data_size);
                        if (offset > slice_data.size() || size > slice_data.size() - offset) {
                            valid = false;
                            break;
                        }
                        total_size += size;
                    }
                    if (valid) {
                        framed.reserve(total_size);
                        for (uint32_t slice_index = 0; slice_index < slice_param_buffer->num_elements; ++slice_index) {
                            const auto offset = static_cast<size_t>(av1_slices[slice_index].slice_data_offset);
                            const auto size = static_cast<size_t>(av1_slices[slice_index].slice_data_size);
                            framed.insert(framed.end(), slice_data.begin() + offset, slice_data.begin() + offset + size);
                        }
                    }
                }
            }

            if (!framed.empty()) {
                rebuilt_frame_buffer.insert(rebuilt_frame_buffer.end(), framed.begin(), framed.end());
            } else {
                rebuilt_frame_buffer.insert(rebuilt_frame_buffer.end(), slice_data.begin(), slice_data.end());
            }
        }

        if (!rebuilt_frame_buffer.empty()) {
            picture.frame_buffer = std::move(rebuilt_frame_buffer);
        }
    }

    if (picture.frame_buffer.empty()) return VA_STATUS_ERROR_INVALID_BUFFER;

    picture.frame_extra_data.clear();
    if (picture.decode_state.picture_param_buffer_id) {
        auto it = d->buffers.find(*picture.decode_state.picture_param_buffer_id);
        if (it != d->buffers.end()) {
            const auto& picture_param_buffer = *it->second;
            if (d->profile == VAProfileHEVCMain || d->profile == VAProfileHEVCMain10) {
                const auto& hevc = *reinterpret_cast<const VAPictureParameterBufferHEVC*>(picture_param_buffer.data.data());
                static std::atomic<int> hevc_param_log_count{0};
                if (hevc_param_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
                    util::log(util::stderr_sink, util::LogLevel::Info,
                              "hevc params: w={} h={} chroma={} bit_depth_luma={} bit_depth_chroma={} tiles={} pcm={} sao={} rps={} ltrp={} reorder={} long_term={} extra_slice_bits={}",
                              hevc.pic_width_in_luma_samples,
                              hevc.pic_height_in_luma_samples,
                              hevc.pic_fields.bits.chroma_format_idc,
                              hevc.bit_depth_luma_minus8,
                              hevc.bit_depth_chroma_minus8,
                              hevc.pic_fields.bits.tiles_enabled_flag,
                              hevc.pic_fields.bits.pcm_enabled_flag,
                              hevc.slice_parsing_fields.bits.sample_adaptive_offset_enabled_flag,
                              hevc.num_short_term_ref_pic_sets,
                              hevc.slice_parsing_fields.bits.long_term_ref_pics_present_flag,
                              hevc.pic_fields.bits.NoPicReorderingFlag,
                              hevc.num_long_term_ref_pic_sps,
                              hevc.num_extra_slice_header_bits);
                }
            }
            switch (d->profile) {
                case VAProfileH264ConstrainedBaseline:
                case VAProfileH264Baseline:
                case VAProfileH264Main:
                case VAProfileH264High:
                    if (!session.sequence_headers_sent) {
                        picture.frame_extra_data = bitstream::build_h264_headers(
                            *reinterpret_cast<const VAPictureParameterBufferH264*>(picture_param_buffer.data.data()),
                            d->picture_width,
                            d->picture_height);
                    }
                    break;
                case VAProfileHEVCMain:
                case VAProfileHEVCMain10:
                    picture.frame_extra_data = bitstream::build_hevc_headers(
                        *reinterpret_cast<const VAPictureParameterBufferHEVC*>(picture_param_buffer.data.data()));
                    break;
                case VAProfileAV1Profile0: {
                    const auto& av1_pic = *reinterpret_cast<const VADecPictureParameterBufferAV1*>(picture_param_buffer.data.data());
                    if (session.pending_av1_frame.valid) {
                        std::array<VASurfaceID, 8> pending_ref_frame_map{};
                        std::copy(std::begin(session.pending_av1_frame.pic.ref_frame_map),
                                  std::end(session.pending_av1_frame.pic.ref_frame_map),
                                  pending_ref_frame_map.begin());
                        const uint8_t resolved_refresh_frame_flags = deriveAv1RefreshFrameFlags(
                            av1_pic,
                            pending_ref_frame_map,
                            true,
                            session.pending_av1_frame.surface);
                        if (av1CorrectionEnabled() &&
                            resolved_refresh_frame_flags != session.pending_av1_frame.provisional_refresh_frame_flags) {
                            MppDecoder::DecodeJob correction_job;
                            correction_job.target_surface = session.pending_av1_frame.surface;
                            correction_job.bitstream = bitstream::build_av1_frame_obu(
                                session.pending_av1_frame.pic,
                                session.pending_av1_frame.tile_payload,
                                resolved_refresh_frame_flags,
                                session.pending_av1_frame.sequence_enable_restoration);
                            correction_job.eos = false;
                            if (!d->decoder->enqueueJob(std::move(correction_job))) {
                                util::log(util::stderr_sink, util::LogLevel::Error,
                                          "vaEndPicture: enqueue AV1 correction failed for surface={}",
                                          session.pending_av1_frame.surface);
                                return VA_STATUS_ERROR_ALLOCATION_FAILED;
                            }
                        }
                        session.pending_av1_frame.valid = false;
                    }

                    static std::atomic<int> av1_param_log_count{0};
                    if (av1_param_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
                        util::log(util::stderr_sink, util::LogLevel::Info,
                                  "av1 params: show_frame={} error_resilient={} intrabc={} order_hint={} base_qindex={} tile_rows={} interp_filter={} refresh_flags={} restoration={} filter_u={} filter_v={} cdef_bits={} cdef_damping={} y_restoration={} cb_restoration={} ref_deltas=[{},{},{},{},{},{},{}] ref_map=[{},{},{},{},{},{},{},{}]",
                                  av1_pic.pic_info_fields.bits.show_frame,
                                  av1_pic.pic_info_fields.bits.error_resilient_mode,
                                  av1_pic.pic_info_fields.bits.allow_intrabc,
                                  av1_pic.order_hint,
                                  av1_pic.base_qindex,
                                  av1_pic.tile_rows,
                                  av1_pic.interp_filter,
                                  deriveAv1RefreshFrameFlags(av1_pic,
                                                             session.av1_ref_frame_map,
                                                             session.av1_ref_frame_map_valid,
                                                             picture.current_surface),
                                  session.av1_sequence_enable_restoration,
                                  av1_pic.filter_level_u,
                                  av1_pic.filter_level_v,
                                  av1_pic.cdef_bits,
                                  av1_pic.cdef_damping_minus_3,
                                  av1_pic.loop_restoration_fields.bits.yframe_restoration_type,
                                  av1_pic.loop_restoration_fields.bits.cbframe_restoration_type,
                                  av1_pic.ref_deltas[0],
                                  av1_pic.ref_deltas[1],
                                  av1_pic.ref_deltas[2],
                                  av1_pic.ref_deltas[3],
                                  av1_pic.ref_deltas[4],
                                  av1_pic.ref_deltas[5],
                                  av1_pic.ref_deltas[6],
                                  av1_pic.ref_frame_map[0],
                                  av1_pic.ref_frame_map[1],
                                  av1_pic.ref_frame_map[2],
                                  av1_pic.ref_frame_map[3],
                                  av1_pic.ref_frame_map[4],
                                  av1_pic.ref_frame_map[5],
                                  av1_pic.ref_frame_map[6],
                                  av1_pic.ref_frame_map[7]);
                    }

                    session.av1_sequence_enable_restoration = session.av1_sequence_enable_restoration ||
                                                               av1_pic.loop_restoration_fields.bits.yframe_restoration_type != 0 ||
                                                               av1_pic.loop_restoration_fields.bits.cbframe_restoration_type != 0 ||
                                                               av1_pic.loop_restoration_fields.bits.crframe_restoration_type != 0;
                    const auto tile_payload = picture.frame_buffer;
                    const uint8_t refresh_frame_flags = deriveAv1RefreshFrameFlags(
                        av1_pic,
                        session.av1_ref_frame_map,
                        session.av1_ref_frame_map_valid,
                        picture.current_surface);
                    auto sequence_header = bitstream::build_av1_sequence_header(av1_pic);
                    if (!session.sequence_headers_sent) {
                        picture.frame_extra_data = std::move(sequence_header);
                    }
                    auto frame_obu = bitstream::build_av1_frame_obu(
                        av1_pic,
                        tile_payload,
                        refresh_frame_flags,
                        session.av1_sequence_enable_restoration);
                    picture.frame_buffer = std::move(frame_obu);
                    for (uint8_t slot = 0; slot < 8; ++slot) {
                        if ((refresh_frame_flags & (1u << slot)) != 0) {
                            session.av1_ref_frame_map[slot] = picture.current_surface;
                        }
                    }
                    session.av1_ref_frame_map_valid = true;

                    if (refresh_frame_flags == 0) {
                        session.pending_av1_frame.valid = true;
                        session.pending_av1_frame.surface = picture.current_surface;
                        session.pending_av1_frame.provisional_refresh_frame_flags = refresh_frame_flags;
                        session.pending_av1_frame.sequence_enable_restoration = session.av1_sequence_enable_restoration;
                        session.pending_av1_frame.pic = av1_pic;
                        session.pending_av1_frame.tile_payload = tile_payload;
                    }
                    break;
                }
                default:
                    break;
            }
            session.noteSequenceHeaders(picture.frame_extra_data);
        }
    }

    static std::atomic<int> extra_log_count{0};
    if (extra_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaEndPicture extra_data final size={}", picture.frame_extra_data.size());
    }

    static std::atomic<int> header_log_count{0};
    if (header_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        size_t picture_param_size = 0;
        if (picture.decode_state.picture_param_buffer_id) {
            auto it = d->buffers.find(*picture.decode_state.picture_param_buffer_id);
            if (it != d->buffers.end()) {
                picture_param_size = it->second->size_bytes();
            }
        }
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaEndPicture headers: picture_buf={} profile={} extra_size={}",
                  picture_param_size,
                  static_cast<int>(d->profile),
                  picture.frame_extra_data.size());
    }

    // Before submitting, reset ready/failed flags so vaSyncSurface will block
    // until MPP reports the frame is decoded and attached.
    d->decoder->resetSurface(picture.current_surface);

    static std::atomic<int> end_log_count{0};
    if (end_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        char prefix[64] = {0};
        size_t prefix_len = std::min<size_t>(8, picture.frame_buffer.size());
        for (size_t i = 0; i < prefix_len; ++i) {
            std::snprintf(prefix + i * 3, sizeof(prefix) - i * 3, "%02x ", picture.frame_buffer[i]);
        }
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaEndPicture: surface={} frame_buffer={} extra_data={} prefix={}",
                  picture.current_surface, picture.frame_buffer.size(), picture.frame_extra_data.size(), prefix);
    }

    static std::atomic<int> extra_prefix_log_count{0};
    if (extra_prefix_log_count.fetch_add(1, std::memory_order_relaxed) < 2 && !picture.frame_extra_data.empty()) {
        char extra_prefix[128] = {0};
        size_t extra_prefix_len = std::min<size_t>(16, picture.frame_extra_data.size());
        for (size_t i = 0; i < extra_prefix_len; ++i) {
            std::snprintf(extra_prefix + i * 3, sizeof(extra_prefix) - i * 3, "%02x ", picture.frame_extra_data[i]);
        }
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaEndPicture: extra_data_prefix={} slice_param_buffers={} slice_data_buffers={}",
                  extra_prefix,
                  picture.decode_state.slice_param_buffer_ids.size(),
                  picture.decode_state.slice_data_buffer_ids.size());
    }

    MppDecoder::DecodeJob job;
    job.target_surface = picture.current_surface;
    job.bitstream = std::move(picture.frame_buffer);
    job.extra_data = std::move(picture.frame_extra_data);
    // Do NOT set EOS here; the decoder will take packets as a stream and
    // should not be forced to flush per VA picture.
    job.eos = false;

    const unsigned int startup_prime_copies = session.consumeStartupPrimeCopies(d->profile);
    for (unsigned int copy_index = 0; copy_index < startup_prime_copies; ++copy_index) {
        MppDecoder::DecodeJob prime_job;
        prime_job.target_surface = session.nextStartupPrimeSurface();
        if (prime_job.target_surface == VA_INVALID_ID) {
            prime_job.target_surface = VA_INVALID_ID;
            prime_job.discard_output = true;
        }
        static std::atomic<int> prime_target_log_count{0};
        if (prime_target_log_count.fetch_add(1, std::memory_order_relaxed) < 12) {
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "vaEndPicture: priming copy={}/{} current_surface={} prime_surface={} discard_output={}",
                      copy_index + 1,
                      startup_prime_copies,
                      picture.current_surface,
                      prime_job.target_surface,
                      prime_job.discard_output ? 1 : 0);
        }
        prime_job.bitstream = job.bitstream;
        prime_job.extra_data = job.extra_data;
        prime_job.eos = false;
        if (!d->decoder->enqueueJob(std::move(prime_job))) {
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "vaEndPicture: startup prime enqueue failed for surface={} copy={}/{}",
                      picture.current_surface,
                      copy_index + 1,
                      startup_prime_copies);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
    }

    // Submit the complete frame to the decoder thread (which will feed MPP).
    if (!d->decoder->enqueueJob(std::move(job))) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "vaEndPicture: enqueueJob failed for surface={}", picture.current_surface);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaQuerySurfaceStatus(VADriverContextP ctx,
                                     VASurfaceID surface,
                                     VASurfaceStatus* status) {
    if (!status) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    bool ready = false;
    bool failed = false;
    if (!d->decoder->getSurfaceState(surface, ready, failed)) {
        *status = VASurfaceReady;
        return VA_STATUS_SUCCESS;
    }

    static std::atomic<int> status_trace_count{0};
    if (status_trace_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaQuerySurfaceStatus: surface={} ready={} failed={}", surface, (int)ready, (int)failed);
    }

    *status = ready ? VASurfaceReady : VASurfaceRendering;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaQuerySurfaceError(VADriverContextP ctx,
                                    VASurfaceID surface,
                                    VAStatus error_status,
                                    void** error_info) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto it = d->surfaces.find(surface);
    if (it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;

    if (error_info) *error_info = nullptr;
    return error_status;
}

static VAStatus vaExportSurfaceHandle(VADriverContextP ctx,
                                      VASurfaceID surface,
                                      uint32_t mem_type,
                                      uint32_t flags,
                                      void* descriptor) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    VAStatus sync_status = vaSyncSurface(ctx, surface);
    if (sync_status != VA_STATUS_SUCCESS) {
        return sync_status;
    }

    std::lock_guard<std::mutex> lock(d->lock);

    constexpr uint32_t supported_flags = VA_EXPORT_SURFACE_READ_ONLY |
                                         VA_EXPORT_SURFACE_WRITE_ONLY |
                                         VA_EXPORT_SURFACE_SEPARATE_LAYERS |
                                         VA_EXPORT_SURFACE_COMPOSED_LAYERS;
    if ((flags & ~supported_flags) != 0) {
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    auto it = d->surfaces.find(surface);
    if (it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;
    if (!descriptor) return VA_STATUS_ERROR_INVALID_PARAMETER;

    if (mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME &&
        mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2) {
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    }
    if (!kExposePrime2Export && mem_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2) {
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    }

    auto* desc = static_cast<VADRMPRIMESurfaceDescriptor*>(descriptor);
    std::memset(desc, 0, sizeof(*desc));

    const auto& surf = it->second->surface;
    if (surf.dmabuf_fd < 0) {
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    const uint32_t stride = surf.stride ? surf.stride : alignTo(surf.width, 64);
    const uint32_t aligned_h = surf.is_10bit ? alignTo(surf.height, 8) : surf.height;
    const uint32_t bytes_per_row = stride * (surf.is_10bit ? 2u : 1u);
    const uint32_t y_size = bytes_per_row * aligned_h;
    const uint32_t uv_offset = y_size;

    uint64_t total_size;
    if (!surf.is_10bit) {
        total_size = static_cast<uint64_t>(stride) * surf.height * 3 / 2;
    } else {
        uint32_t aligned_h = ((surf.height + 7) / 8) * 8;
        total_size = static_cast<uint64_t>(stride) * aligned_h * 3;
    }

    desc->fourcc = surf.is_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;
    desc->width = surf.width;
    desc->height = surf.height;
    desc->num_objects = 1;
    unique_fd exported_fd{dup(surf.dmabuf_fd)};
    if (!exported_fd) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    desc->objects[0].fd = exported_fd.release();
    desc->objects[0].size = static_cast<uint32_t>(total_size);
    desc->objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;

    const bool separate_layers = kExposePrime2Export &&
                                 mem_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2 &&
                                 (flags & VA_EXPORT_SURFACE_SEPARATE_LAYERS) != 0;
    if (separate_layers) {
        desc->num_layers = 2;

        desc->layers[0].drm_format = surf.is_10bit ? DRM_FORMAT_R16 : DRM_FORMAT_R8;
        desc->layers[0].num_planes = 1;
        desc->layers[0].object_index[0] = 0;
        desc->layers[0].offset[0] = 0;
        desc->layers[0].pitch[0] = bytes_per_row;

        desc->layers[1].drm_format = surf.is_10bit ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88;
        desc->layers[1].num_planes = 1;
        desc->layers[1].object_index[0] = 0;
        desc->layers[1].offset[0] = uv_offset;
        desc->layers[1].pitch[0] = bytes_per_row;
    } else {
        desc->num_layers = 1;
        desc->layers[0].drm_format = surf.is_10bit ? DRM_FORMAT_P010 : DRM_FORMAT_NV12;
        desc->layers[0].num_planes = 2;
        desc->layers[0].object_index[0] = 0;
        desc->layers[0].object_index[1] = 0;
        desc->layers[0].offset[0] = 0;
        desc->layers[0].offset[1] = uv_offset;
        desc->layers[0].pitch[0] = bytes_per_row;
        desc->layers[0].pitch[1] = bytes_per_row;
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaQueryImageFormats(VADriverContextP ctx,
                                    VAImageFormat* format_list,
                                    int* num_formats) {
    if (!num_formats) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (!format_list) {
        *num_formats = 2;
        return VA_STATUS_SUCCESS;
    }
    if (*num_formats < 2) return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;

    format_list[0].fourcc = VA_FOURCC_NV12;
    format_list[0].depth = 12;
    format_list[0].red_mask = 0;
    format_list[0].green_mask = 0;
    format_list[0].blue_mask = 0;
    format_list[0].alpha_mask = 0;

    format_list[1].fourcc = VA_FOURCC_P010;
    format_list[1].depth = 12;
    format_list[1].red_mask = 0;
    format_list[1].green_mask = 0;
    format_list[1].blue_mask = 0;
    format_list[1].alpha_mask = 0;

    *num_formats = 2;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaCreateImage(VADriverContextP ctx,
                              VAImageFormat* format,
                              int width,
                              int height,
                              VAImage* image) {
    if (!ctx || !format || !image) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    if (width <= 0 || height <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if ((format->fourcc != VA_FOURCC_NV12 && format->fourcc != VA_FOURCC_P010) || format->depth != 12) {
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    const bool is_10bit = format->fourcc == VA_FOURCC_P010;
    uint32_t pitch = ((static_cast<uint32_t>(width) + 63) / 64) * 64 * (is_10bit ? 2u : 1u);
    uint32_t y_size = pitch * static_cast<uint32_t>(height);
    uint32_t uv_height = (static_cast<uint32_t>(height) + 1) / 2;
    uint32_t total_size = y_size + pitch * uv_height;

    VABufferID buf_id = 0;
    VAStatus status = vaCreateBuffer(ctx, 0, VAImageBufferType, total_size, 1, nullptr, &buf_id);
    if (status != VA_STATUS_SUCCESS) return status;

    VAImageID image_id = d->next_image_id++;
    image->image_id = image_id;
    image->format = *format;
    image->buf = buf_id;
    image->width = static_cast<uint16_t>(width);
    image->height = static_cast<uint16_t>(height);
    image->data_size = total_size;
    image->num_planes = 2;
    image->pitches[0] = pitch;
    image->pitches[1] = pitch;
    image->pitches[2] = 0;
    image->offsets[0] = 0;
    image->offsets[1] = y_size;
    image->offsets[2] = 0;
    image->num_palette_entries = 0;
    image->entry_bytes = 0;
    image->component_order[0] = 0;
    image->component_order[1] = 0;
    image->component_order[2] = 0;
    image->component_order[3] = 0;
    memset(image->va_reserved, 0, sizeof(image->va_reserved));

    {
        std::lock_guard<std::mutex> lock(d->resource_mutex);
        DriverState::ImageState state;
        state.image = *image;
        state.derived_surface = VA_INVALID_ID;
        d->images[image_id] = std::move(state);
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDeriveImage(VADriverContextP ctx,
                              VASurfaceID surface,
                              VAImage* image) {
    if (!ctx || !image) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    static std::atomic<int> derive_log_count{0};
    if (derive_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaDeriveImage: surface={} image={}", surface, static_cast<void*>(image));
    }

    auto it = d->surfaces.find(surface);
    if (it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;

    VAStatus sync_status = vaSyncSurface(ctx, surface);
    if (sync_status != VA_STATUS_SUCCESS) {
        return sync_status;
    }

    const auto& surf = it->second->surface;
    if (surf.dmabuf_fd < 0) return VA_STATUS_ERROR_INVALID_SURFACE;

    const bool is_10bit = surf.is_10bit;
    VAImageFormat fmt = {};
    fmt.fourcc = is_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;
    fmt.depth = 12;
    fmt.red_mask = 0;
    fmt.green_mask = 0;
    fmt.blue_mask = 0;
    fmt.alpha_mask = 0;

    uint32_t pitch = ((surf.width + 63) / 64) * 64 * (is_10bit ? 2u : 1u);
    uint32_t aligned_h = is_10bit ? alignTo(surf.height, 8) : surf.height;
    uint32_t y_size = pitch * aligned_h;
    uint32_t uv_height = aligned_h / 2;
    uint32_t total_size = y_size + pitch * uv_height;

    VABufferID buf_id = d->next_buffer_id++;

    VAImageID image_id = d->next_image_id++;
    image->image_id = image_id;
    image->format = fmt;
    image->buf = buf_id;
    image->width = static_cast<uint16_t>(surf.width);
    image->height = static_cast<uint16_t>(surf.height);
    image->data_size = total_size;
    image->num_planes = 2;
    image->pitches[0] = pitch;
    image->pitches[1] = pitch;
    image->pitches[2] = 0;
    image->offsets[0] = 0;
    image->offsets[1] = y_size;
    image->offsets[2] = 0;
    image->num_palette_entries = 0;
    image->entry_bytes = 0;
    image->component_order[0] = 0;
    image->component_order[1] = 0;
    image->component_order[2] = 0;
    image->component_order[3] = 0;
    std::memset(image->va_reserved, 0, sizeof(image->va_reserved));

    {
        std::lock_guard<std::mutex> lock(d->resource_mutex);
        DriverState::ImageState state;
        state.image = *image;
        if (!attachImageToSurfaceDmabuf(state, surface, surf)) {
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        d->images[image_id] = std::move(state);
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyImage(VADriverContextP ctx,
                               VAImageID image) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->resource_mutex);
    auto it = d->images.find(image);
    if (it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    releaseDirectImageAlias(it->second);
    d->buffers.erase(it->second.image.buf);
    d->images.erase(it);
    return VA_STATUS_SUCCESS;
}

static VAStatus vaSetImagePalette(VADriverContextP ctx,
                                  VAImageID image,
                                  unsigned char* palette) {
    if (!ctx || !palette) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto it = d->images.find(image);
    if (it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    // Store palette for paletted images. This driver does not support
    // paletted image formats, but we keep the data in case an app sets it.
    it->second.palette.assign(palette, palette + 256 * 4);
    return VA_STATUS_SUCCESS;
}

static VAStatus vaGetImage(VADriverContextP ctx,
                           VASurfaceID surface,
                           int x,
                           int y,
                           unsigned int w,
                           unsigned int h,
                           VAImageID image) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    static std::atomic<int> get_image_log_count{0};
    if (get_image_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: surface={} x={} y={} w={} h={} image={}",
                  surface, x, y, w, h, image);
    }

    auto surf_it = d->surfaces.find(surface);
    if (surf_it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;

    VAStatus sync_status = vaSyncSurface(ctx, surface);
    if (sync_status != VA_STATUS_SUCCESS) {
        return sync_status;
    }

    VAImage image_state = {};
    void* img_buf = nullptr;
    {
        std::lock_guard<std::mutex> lock(d->resource_mutex);
        auto img_it = d->images.find(image);
        if (img_it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;
        image_state = img_it->second.image;
    }

    static std::atomic<int> get_image_state_log_count{0};
    if (get_image_state_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        const auto& surf = surf_it->second->surface;
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage state: surface={} surf_w={} surf_h={} stride={} fd={} img_w={} img_h={} img_pitch={} img_uv_pitch={}",
                  surface, surf.width, surf.height, surf.stride, surf.dmabuf_fd,
                  image_state.width, image_state.height,
                  image_state.pitches[0], image_state.pitches[1]);
    }

    const bool surf_is_10bit = surf_it->second->surface.is_10bit;
    const uint32_t expected_fourcc = surf_is_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;

    // Enforce matching image format for the current surface bit depth.
    if (image_state.format.fourcc != expected_fourcc ||
        image_state.format.depth != 12) {
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    // Ensure the region lies within the surface bounds.
    const auto& surf = surf_it->second->surface;
    if (x < 0 || y < 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    uint32_t x_coord = static_cast<uint32_t>(x);
    uint32_t y_coord = static_cast<uint32_t>(y);
    if (x_coord >= surf.width || y_coord >= surf.height) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (w == 0 || h == 0) return VA_STATUS_ERROR_INVALID_PARAMETER;

    // NV12/P010 require even coordinates for UV plane sampling.
    if ((x_coord & 1) || (y_coord & 1)) return VA_STATUS_ERROR_INVALID_PARAMETER;

    uint32_t max_w = std::min<uint32_t>(w, surf.width - x_coord);
    uint32_t max_h = std::min<uint32_t>(h, surf.height - y_coord);
    max_w &= ~1u;
    max_h &= ~1u;

    const bool full_surface_request = x_coord == 0 && y_coord == 0 && max_w == surf.width && max_h == surf.height;
    if (full_surface_request) {
        std::lock_guard<std::mutex> lock(d->resource_mutex);
        auto img_it = d->images.find(image);
        if (img_it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;
        if (attachImageToSurfaceDmabuf(img_it->second, surface, surf)) {
            d->buffers.erase(img_it->second.image.buf);
            return VA_STATUS_SUCCESS;
        }

        auto buffer_it = d->buffers.find(img_it->second.image.buf);
        if (buffer_it != d->buffers.end()) {
            img_buf = buffer_it->second->map().data();
        } else {
            img_buf = mapDirectImageBuffer(img_it->second);
        }
    } else {
        std::lock_guard<std::mutex> lock(d->resource_mutex);
        auto img_it = d->images.find(image);
        if (img_it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;
        auto buffer_it = d->buffers.find(img_it->second.image.buf);
        if (buffer_it == d->buffers.end()) {
            img_buf = mapDirectImageBuffer(img_it->second);
        } else {
            img_buf = buffer_it->second->map().data();
        }
    }

    if (!img_buf) {
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }

    static std::atomic<int> get_image_mapbuf_log_count{0};
    if (get_image_mapbuf_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapping image buffer={}", image_state.buf);
    }
    static std::atomic<int> get_image_mapbuf_done_log_count{0};
    if (get_image_mapbuf_done_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapped image buffer={}", image_state.buf);
    }

    uint32_t img_pitch = image_state.pitches[0];
    uint32_t img_uv_pitch = image_state.pitches[1];

    // Fallback path for cropped or layout-mismatched reads.
    uint32_t surf_pitch = surf.is_10bit ? surf.stride * 2 : surf.stride;
    uint32_t surf_y_size = surf_pitch * surf.height;
    uint32_t surf_total_size = surf_y_size + surf_pitch * ((surf.height + 1) / 2);

    static std::atomic<int> get_image_mapdmabuf_log_count{0};
    if (get_image_mapdmabuf_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapping surface fd={} size={}", surf.dmabuf_fd, surf_total_size);
    }
    auto surface_mapping = MappedDmabuf::map(surf.dmabuf_fd, surf_total_size, PROT_READ | PROT_WRITE);
    if (!surface_mapping) {
        return VA_STATUS_ERROR_UNKNOWN;
    }
    static std::atomic<int> get_image_mapdmabuf_done_log_count{0};
    if (get_image_mapdmabuf_done_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapped surface fd={} size={}", surf.dmabuf_fd, surf_total_size);
    }

    // Copy Y plane
    uint8_t* dst_y = static_cast<uint8_t*>(img_buf);
    uint8_t* src_y = static_cast<uint8_t*>(surface_mapping.data());
    const uint32_t y_copy_bytes = max_w * (surf.is_10bit ? 2u : 1u);
    for (uint32_t row = 0; row < max_h; row++) {
        uint8_t* dst_row = dst_y + row * img_pitch;
        uint8_t* src_row = src_y + (y_coord + row) * surf_pitch + x_coord;
        memcpy(dst_row, src_row, y_copy_bytes);
    }

    // Copy UV plane
    uint32_t uv_height = (max_h + 1) / 2;
    uint32_t uv_x = x_coord & ~1u;
    uint32_t uv_w = surf.is_10bit ? max_w * 2 : ((max_w + 1) / 2 * 2);
    uint8_t* dst_uv = static_cast<uint8_t*>(img_buf) + image_state.offsets[1];
    uint8_t* src_uv = static_cast<uint8_t*>(surface_mapping.data()) + surf_y_size;
    for (uint32_t row = 0; row < uv_height; row++) {
        uint8_t* dst_row = dst_uv + row * img_uv_pitch;
        uint8_t* src_row = src_uv + ((static_cast<uint32_t>(y) / 2 + row) * surf_pitch) + uv_x;
        memcpy(dst_row, src_row, uv_w);
    }

    static std::atomic<int> get_image_copy_done_log_count{0};
    if (get_image_copy_done_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: copy complete surface={} image={}", surface, image);
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaPutImage(VADriverContextP ctx,
                           VASurfaceID surface,
                           VAImageID image,
                           int srcx,
                           int srcy,
                           unsigned int srcw,
                           unsigned int srch,
                           int destx,
                           int desty,
                           unsigned int destw,
                           unsigned int desth) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto surf_it = d->surfaces.find(surface);
    if (surf_it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;
    VAImage image_state = {};
    void* img_buf = nullptr;
    {
        std::lock_guard<std::mutex> lock(d->resource_mutex);
        auto img_it = d->images.find(image);
        if (img_it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;
        auto buffer_it = d->buffers.find(img_it->second.image.buf);
        if (buffer_it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;
        image_state = img_it->second.image;
        img_buf = buffer_it->second->map().data();
    }

    // Enforce NV12 images only.
    if (image_state.format.fourcc != VA_FOURCC_NV12 ||
        image_state.format.depth != 12) {
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    const auto& surf = surf_it->second->surface;
    if (surf.dmabuf_fd < 0) return VA_STATUS_ERROR_INVALID_SURFACE;

    // Clip regions to source and destination bounds.
    if (srcx < 0 || srcy < 0 || destx < 0 || desty < 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    uint32_t src_x_coord = static_cast<uint32_t>(srcx);
    uint32_t src_y_coord = static_cast<uint32_t>(srcy);
    uint32_t dst_x_coord = static_cast<uint32_t>(destx);
    uint32_t dst_y_coord = static_cast<uint32_t>(desty);

    // NV12 requires even coordinates for UV sampling.
    if ((src_x_coord & 1) || (src_y_coord & 1) || (dst_x_coord & 1) || (dst_y_coord & 1)) {
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    uint32_t src_w = std::min<uint32_t>(srcw, image_state.width - src_x_coord);
    uint32_t src_h = std::min<uint32_t>(srch, image_state.height - src_y_coord);
    uint32_t dst_w = std::min<uint32_t>(destw, surf.width - dst_x_coord);
    uint32_t dst_h = std::min<uint32_t>(desth, surf.height - dst_y_coord);

    // Ensure even dimensions for NV12.
    src_w &= ~1u;
    dst_w &= ~1u;
    src_h &= ~1u;
    dst_h &= ~1u;

    uint32_t copy_w = std::min(src_w, dst_w);
    uint32_t copy_h = std::min(src_h, dst_h);
    if (copy_w == 0 || copy_h == 0) return VA_STATUS_ERROR_INVALID_PARAMETER;

    uint32_t img_pitch = image_state.pitches[0];
    uint32_t img_uv_pitch = image_state.pitches[1];

    uint32_t surf_pitch = surf.stride;
    uint32_t surf_y_size = surf_pitch * surf.height;
    uint32_t surf_total_size = surf_y_size + surf_pitch * ((surf.height + 1) / 2);

    auto surface_mapping = MappedDmabuf::map(surf.dmabuf_fd, surf_total_size, PROT_READ | PROT_WRITE);
    if (!surface_mapping) {
        return VA_STATUS_ERROR_UNKNOWN;
    }

    uint8_t* dst_y = static_cast<uint8_t*>(surface_mapping.data());
    uint8_t* src_y = static_cast<uint8_t*>(img_buf);
    for (uint32_t row = 0; row < copy_h; row++) {
        uint8_t* dst_row = dst_y + (dst_y_coord + row) * surf_pitch + dst_x_coord;
        uint8_t* src_row = src_y + (src_y_coord + row) * img_pitch + src_x_coord;
        memcpy(dst_row, src_row, copy_w);
    }

    uint32_t uv_height = (copy_h + 1) / 2;
    uint32_t uv_src_x = (src_x_coord & ~1u);
    uint32_t uv_dst_x = (dst_x_coord & ~1u);
    uint32_t uv_w = (copy_w + 1) / 2 * 2;
    uint8_t* dst_uv = static_cast<uint8_t*>(surface_mapping.data()) + surf_y_size;
    uint8_t* src_uv = static_cast<uint8_t*>(img_buf) + image_state.offsets[1];
    for (uint32_t row = 0; row < uv_height; row++) {
        uint8_t* dst_row = dst_uv + ((dst_y_coord / 2 + row) * surf_pitch) + uv_dst_x;
        uint8_t* src_row = src_uv + ((src_y_coord / 2 + row) * img_uv_pitch) + uv_src_x;
        memcpy(dst_row, src_row, uv_w);
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaQuerySubpictureFormats(VADriverContextP ctx,
                                         VAImageFormat* format_list,
                                         unsigned int* flags,
                                         unsigned int* num_formats) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (!num_formats) return VA_STATUS_ERROR_INVALID_PARAMETER;

    // Only NV12 is supported in this driver, matching image format behavior.
    if (!format_list) {
        *num_formats = 1;
    } else {
        if (*num_formats < 1) return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        format_list[0].fourcc = VA_FOURCC_NV12;
        format_list[0].depth = 12;
        format_list[0].red_mask = 0;
        format_list[0].green_mask = 0;
        format_list[0].blue_mask = 0;
        format_list[0].alpha_mask = 0;
        *num_formats = 1;
    }

    if (flags) *flags = 0;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaCreateSubpicture(VADriverContextP ctx,
                                   VAImageID image,
                                   VASubpictureID* subpicture) {
    if (!ctx || !subpicture) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    if (d->images.find(image) == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    VASubpictureID id = d->next_subpicture_id++;
    d->subpictures[id] = {image, 0, 0, 0, 1.0f, {}};
    *subpicture = id;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroySubpicture(VADriverContextP ctx,
                                    VASubpictureID subpicture) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    d->subpictures.erase(subpicture);
    return VA_STATUS_SUCCESS;
}

static VAStatus vaSetSubpictureImage(VADriverContextP ctx,
                                     VASubpictureID subpicture,
                                     VAImageID image) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto sp_it = d->subpictures.find(subpicture);
    if (sp_it == d->subpictures.end()) return VA_STATUS_ERROR_INVALID_SUBPICTURE;
    if (d->images.find(image) == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    sp_it->second.image_id = image;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaSetSubpictureChromakey(VADriverContextP ctx,
                                         VASubpictureID subpicture,
                                         unsigned int chromakey_min,
                                         unsigned int chromakey_max,
                                         unsigned int chromakey_mask) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto sp_it = d->subpictures.find(subpicture);
    if (sp_it == d->subpictures.end()) return VA_STATUS_ERROR_INVALID_SUBPICTURE;

    sp_it->second.chromakey_min = chromakey_min;
    sp_it->second.chromakey_max = chromakey_max;
    sp_it->second.chromakey_mask = chromakey_mask;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaSetSubpictureGlobalAlpha(VADriverContextP ctx,
                                           VASubpictureID subpicture,
                                           float global_alpha) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto sp_it = d->subpictures.find(subpicture);
    if (sp_it == d->subpictures.end()) return VA_STATUS_ERROR_INVALID_SUBPICTURE;

    sp_it->second.global_alpha = global_alpha;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaAssociateSubpicture(VADriverContextP ctx,
                                      VASubpictureID subpicture,
                                      VASurfaceID* target_surfaces,
                                      int num_surfaces,
                                      short src_x,
                                      short src_y,
                                      unsigned short src_width,
                                      unsigned short src_height,
                                      short dest_x,
                                      short dest_y,
                                      unsigned short dest_width,
                                      unsigned short dest_height,
                                      unsigned int flags) {
    if (!ctx || !target_surfaces || num_surfaces <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto sp_it = d->subpictures.find(subpicture);
    if (sp_it == d->subpictures.end()) return VA_STATUS_ERROR_INVALID_SUBPICTURE;

    // Track the surfaces this subpicture is associated with.
    sp_it->second.target_surfaces.clear();
    for (int i = 0; i < num_surfaces; i++) {
        sp_it->second.target_surfaces.push_back(target_surfaces[i]);
    }

    // Apply and validate association rectangle values.
    if (src_width == 0 || src_height == 0 || dest_width == 0 || dest_height == 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;

    sp_it->second.src_x = src_x;
    sp_it->second.src_y = src_y;
    sp_it->second.src_width = src_width;
    sp_it->second.src_height = src_height;
    sp_it->second.dest_x = dest_x;
    sp_it->second.dest_y = dest_y;
    sp_it->second.dest_width = dest_width;
    sp_it->second.dest_height = dest_height;
    sp_it->second.flags = flags;

    return VA_STATUS_SUCCESS;
}

static VAStatus vaDeassociateSubpicture(VADriverContextP ctx,
                                        VASubpictureID subpicture,
                                        VASurfaceID* target_surfaces,
                                        int num_surfaces) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto sp_it = d->subpictures.find(subpicture);
    if (sp_it == d->subpictures.end()) return VA_STATUS_ERROR_INVALID_SUBPICTURE;

    // Only clear associations if the caller passes matching surfaces.
    if (target_surfaces && num_surfaces > 0) {
        // If all surfaces match, clear; otherwise leave unchanged.
        bool all_match = true;
        if (static_cast<int>(sp_it->second.target_surfaces.size()) != num_surfaces) {
            all_match = false;
        } else {
            for (int i = 0; i < num_surfaces; i++) {
                if (sp_it->second.target_surfaces[i] != target_surfaces[i]) {
                    all_match = false;
                    break;
                }
            }
        }
        if (all_match) {
            sp_it->second.target_surfaces.clear();
        }
    } else {
        sp_it->second.target_surfaces.clear();
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaQueryDisplayAttributes(VADriverContextP ctx,
                                         VADisplayAttribute* attr_list,
                                         int* num_attributes) {
    if (!num_attributes) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->lock);
    ensureDisplayAttributesInitialized(*d);
    int total = static_cast<int>(d->display_attributes.size());
    if (!attr_list) {
        *num_attributes = total;
        return VA_STATUS_SUCCESS;
    }

    int fill = std::min(*num_attributes, total);
    for (int i = 0; i < fill; i++) {
        attr_list[i] = d->display_attributes[i];
    }
    *num_attributes = total;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaGetDisplayAttributes(VADriverContextP ctx,
                                       VADisplayAttribute* attr_list,
                                       int num_attributes) {
    if (!attr_list || num_attributes <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->lock);
    ensureDisplayAttributesInitialized(*d);
    for (int i = 0; i < num_attributes; i++) {
        auto* req = &attr_list[i];
        auto it = std::find_if(d->display_attributes.begin(), d->display_attributes.end(),
                               [&](const VADisplayAttribute& a) { return a.type == req->type; });
        if (it == d->display_attributes.end()) return VA_STATUS_ERROR_INVALID_PARAMETER;
        if (!(it->flags & VA_DISPLAY_ATTRIB_GETTABLE)) return VA_STATUS_ERROR_INVALID_PARAMETER;

        // Preserve caller-provided range info, but always return the current value.
        req->min_value = it->min_value;
        req->max_value = it->max_value;
        req->value = it->value;
        req->flags = it->flags;
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaSetDisplayAttributes(VADriverContextP ctx,
                                       VADisplayAttribute* attr_list,
                                       int num_attributes) {
    if (!attr_list || num_attributes <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->lock);
    ensureDisplayAttributesInitialized(*d);
    for (int i = 0; i < num_attributes; i++) {
        const auto& req = attr_list[i];
        auto it = std::find_if(d->display_attributes.begin(), d->display_attributes.end(),
                               [&](const VADisplayAttribute& a) { return a.type == req.type; });
        if (it == d->display_attributes.end()) return VA_STATUS_ERROR_INVALID_PARAMETER;
        if (!(it->flags & VA_DISPLAY_ATTRIB_SETTABLE)) return VA_STATUS_ERROR_INVALID_PARAMETER;
        if (req.value < it->min_value || req.value > it->max_value) return VA_STATUS_ERROR_INVALID_PARAMETER;
        it->value = req.value;
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaRenderPicture(VADriverContextP ctx,
                                VAContextID context,
                                VABufferID* buffers,
                                int num_buffers) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (!buffers || num_buffers <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    {
        std::lock_guard<std::mutex> lock(d->lock);
        auto context_it = d->active_contexts.find(context);
        if (context_it == d->active_contexts.end()) return VA_STATUS_ERROR_INVALID_CONTEXT;
        d->profile = context_it->second.profile;
    }
    auto& picture = d->picture;

    const VABuffer* slice_param_buffer = nullptr;
    const VABuffer* slice_data_buffer = nullptr;

    static std::atomic<int> render_log_count{0};
    if (render_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaRenderPicture: surface={} num_buffers={} frame_buffer_before={}",
                  picture.current_surface, num_buffers, picture.frame_buffer.size());
        for (int i = 0; i < num_buffers; ++i) {
            auto it = d->buffers.find(buffers[i]);
            if (it != d->buffers.end()) {
                char prefix[64] = {0};
                size_t prefix_len = std::min<size_t>(8, it->second->size_bytes());
                auto span = it->second->map();
                for (size_t j = 0; j < prefix_len; ++j) {
                    std::snprintf(prefix + j * 3, sizeof(prefix) - j * 3, "%02x ", span[j]);
                }
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "vaRenderPicture: buffer[{}] id={} type={} size={} prefix={}",
                          i, buffers[i], static_cast<int>(it->second->type), it->second->size_bytes(), prefix);
            }
        }
    }

    // Collect and classify state for later out-of-order resequencing / end-picture assembly.
    for (int i = 0; i < num_buffers; i++) {
        auto it = d->buffers.find(buffers[i]);
        if (it == d->buffers.end()) continue;

        picture.current_buffers.push_back(buffers[i]);

        const auto& vb = *it->second;
        switch (vb.type) {
            case VAPictureParameterBufferType:
                picture.decode_state.picture_param_buffer_id = buffers[i];
                break;
            case VASliceParameterBufferType:
                picture.decode_state.slice_param_buffer_ids.push_back(buffers[i]);
                slice_param_buffer = &vb;
                break;
            case VASliceDataBufferType:
                picture.decode_state.slice_data_buffer_ids.push_back(buffers[i]);
                slice_data_buffer = &vb;
                break;
            default:
                break;
        }
    }

    if (slice_data_buffer) {
        const auto slice_data = slice_data_buffer->map();
        std::vector<uint8_t> framed;
        if (slice_param_buffer && slice_param_buffer->num_elements > 0) {
            if (isH264Profile(d->profile) &&
                slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferH264)) {
                framed = bitstream::build_h264_annexb_slices(
                    reinterpret_cast<const VASliceParameterBufferH264*>(slice_param_buffer->data.data()),
                    slice_param_buffer->num_elements,
                    slice_data.data(),
                    slice_data.size());
            } else if ((d->profile == VAProfileHEVCMain || d->profile == VAProfileHEVCMain10) &&
                       slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferHEVC)) {
                framed = bitstream::build_hevc_annexb_slices(
                    reinterpret_cast<const VASliceParameterBufferHEVC*>(slice_param_buffer->data.data()),
                    slice_param_buffer->num_elements,
                    slice_data.data(),
                    slice_data.size());
            } else if (d->profile == VAProfileAV1Profile0 &&
                       slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferAV1)) {
                const auto* av1_slices = reinterpret_cast<const VASliceParameterBufferAV1*>(slice_param_buffer->data.data());
                size_t total_size = 0;
                bool valid = true;
                for (uint32_t index = 0; index < slice_param_buffer->num_elements; ++index) {
                    const auto offset = static_cast<size_t>(av1_slices[index].slice_data_offset);
                    const auto size = static_cast<size_t>(av1_slices[index].slice_data_size);
                    if (offset > slice_data.size() || size > slice_data.size() - offset) {
                        valid = false;
                        break;
                    }
                    total_size += size;
                }
                if (valid) {
                    framed.reserve(total_size);
                    for (uint32_t index = 0; index < slice_param_buffer->num_elements; ++index) {
                        const auto offset = static_cast<size_t>(av1_slices[index].slice_data_offset);
                        const auto size = static_cast<size_t>(av1_slices[index].slice_data_size);
                        framed.insert(framed.end(), slice_data.begin() + offset, slice_data.begin() + offset + size);
                    }
                } else {
                    framed.clear();
                }
            }
        }

        if (!framed.empty()) {
            picture.frame_buffer.insert(picture.frame_buffer.end(), framed.begin(), framed.end());
        } else {
            picture.frame_buffer.insert(picture.frame_buffer.end(), slice_data.begin(), slice_data.end());
        }
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaTerminate(VADriverContextP ctx) {
    auto* d = toDriver(ctx);
    if (d) {
        // Clean up remaining surfaces before decoder teardown so held MPP buffers are released.
        {
            std::scoped_lock lock(d->lock);
            for (auto& kv : d->surfaces) {
                d->decoder->destroySurface(kv.first);
                kv.second->set_dmabuf({});
            }
            d->surfaces.clear();
            for (auto& kv : d->images) {
                releaseDirectImageAlias(kv.second);
                destroyBuffer(d, kv.second.image.buf);
            }
            d->images.clear();

            d->buffers.clear();
            d->subpictures.clear();
            d->ready_flags.clear();
            d->active_contexts.clear();
            d->display_attributes.clear();
        }

        // Stop any decoding activity after surface resources have been dropped.
        d->decoder->shutdown();

        delete d;
        ctx->pDriverData = nullptr;
    }
    if (ctx && ctx->vtable) {
        free(ctx->vtable);
        ctx->vtable = nullptr;
    }
    if (ctx && ctx->str_vendor) {
        free(const_cast<char*>(ctx->str_vendor));
        ctx->str_vendor = nullptr;
    }
    return VA_STATUS_SUCCESS;
}

}  // namespace api

template <typename T>
static constexpr auto make_vtable_entries_template() {
    using namespace api;
    return std::tuple{
        make_entry<&T::vaTerminate>(vaTerminate),
        make_entry<&T::vaQueryConfigProfiles>(vaQueryConfigProfiles),
        make_entry<&T::vaQueryConfigEntrypoints>(vaQueryConfigEntrypoints),
        make_entry<&T::vaGetConfigAttributes>(vaGetConfigAttributes),
        make_entry<&T::vaCreateConfig>(vaCreateConfig),
        make_entry<&T::vaDestroyConfig>(vaDestroyConfig),
        make_entry<&T::vaQueryConfigAttributes>(vaQueryConfigAttributes),
        make_entry<&T::vaCreateSurfaces>(vaCreateSurfaces),
        make_entry<&T::vaDestroySurfaces>(vaDestroySurfaces),
        make_entry<&T::vaCreateContext>(vaCreateContext),
        make_entry<&T::vaDestroyContext>(vaDestroyContext),
        make_entry<&T::vaQuerySurfaceAttributes>(vaQuerySurfaceAttributes),
        make_entry<&T::vaGetSurfaceAttributes>(vaGetSurfaceAttributes),
        make_entry<&T::vaCreateSurfaces2>(vaCreateSurfaces2),
        make_entry<&T::vaCreateBuffer>(vaCreateBuffer),
        make_entry<&T::vaBufferSetNumElements>(vaBufferSetNumElements),
        make_entry<&T::vaMapBuffer>(vaMapBuffer),
        make_entry<&T::vaUnmapBuffer>(vaUnmapBuffer),
        make_entry<&T::vaDestroyBuffer>(vaDestroyBuffer),
        make_entry<&T::vaBeginPicture>(vaBeginPicture),
        make_entry<&T::vaRenderPicture>(vaRenderPicture),
        make_entry<&T::vaEndPicture>(vaEndPicture),
        make_entry<&T::vaSyncSurface>(vaSyncSurface),
        make_entry<&T::vaQuerySurfaceStatus>(vaQuerySurfaceStatus),
        make_entry<&T::vaQuerySurfaceError>(vaQuerySurfaceError),
        make_entry<&T::vaQueryImageFormats>(vaQueryImageFormats),
        make_entry<&T::vaCreateImage>(vaCreateImage),
        make_entry<&T::vaDeriveImage>(vaDeriveImage),
        make_entry<&T::vaDestroyImage>(vaDestroyImage),
        make_entry<&T::vaSetImagePalette>(vaSetImagePalette),
        make_entry<&T::vaGetImage>(vaGetImage),
        make_entry<&T::vaPutImage>(vaPutImage),
        make_entry<&T::vaQuerySubpictureFormats>(vaQuerySubpictureFormats),
        make_entry<&T::vaCreateSubpicture>(vaCreateSubpicture),
        make_entry<&T::vaDestroySubpicture>(vaDestroySubpicture),
        make_entry<&T::vaSetSubpictureImage>(vaSetSubpictureImage),
        make_entry<&T::vaSetSubpictureChromakey>(vaSetSubpictureChromakey),
        make_entry<&T::vaSetSubpictureGlobalAlpha>(vaSetSubpictureGlobalAlpha),
        make_entry<&T::vaAssociateSubpicture>(vaAssociateSubpicture),
        make_entry<&T::vaDeassociateSubpicture>(vaDeassociateSubpicture),
        make_entry<&T::vaQueryDisplayAttributes>(vaQueryDisplayAttributes),
        make_entry<&T::vaGetDisplayAttributes>(vaGetDisplayAttributes),
        make_entry<&T::vaSetDisplayAttributes>(vaSetDisplayAttributes),
        make_entry<&T::vaExportSurfaceHandle>(vaExportSurfaceHandle)};
}

static VAStatus vaDriverInit(VADriverContextP ctx) {
    util::set_log_source_location(true);
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto device_result = resolveDrmDevice(ctx);
    if (!device_result) {
        return device_result.error();
    }

    // Populate the VA-API function table. libva may free this structure, so
    // allocate it dynamically.
    auto* vt = static_cast<VADriverVTable*>(calloc(1, sizeof(VADriverVTable)));
    if (!vt) return VA_STATUS_ERROR_ALLOCATION_FAILED;

    auto register_vtable_fn = [&](auto member_ptr, auto fn) {
        vt->*member_ptr = fn;
    };

    // Static, compile-time known mappings (template-based T).
    static constexpr auto entries = make_vtable_entries_template<VADriverVTable>();

    std::apply(
        [&](auto&&... entry) {
            (register_vtable_fn(entry.first, entry.second), ...);
        },
        entries);

    ctx->vtable = vt;
    auto* driver_state = new (std::nothrow) DriverState();
    if (!driver_state) {
        free(vt);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    driver_state->vtable = vt;
    ctx->pDriverData = driver_state;
    driver_state->resolved_drm_fd = std::move(device_result.value().owned_fd);
    driver_state->drm_fd = device_result.value().fd;
    driver_state->drm_rdev = device_result.value().rdev;
    driver_state->drm_device_path = std::move(device_result.value().path);
    driver_state->drm_driver_name = std::move(device_result.value().driver_name);
    ensureDisplayAttributesInitialized(*driver_state);

    // Pretend to be VA-API 1.2+ to satisfy modern clients.
    ctx->version_major = 1;
    ctx->version_minor = 2;

    // Required by libva initialization checks.
    ctx->max_profiles = 8;
    ctx->max_entrypoints = 4;
    ctx->max_attributes = 16;
    ctx->max_image_formats = 2;
    ctx->max_subpic_formats = 2;
    ctx->max_display_attributes = static_cast<int>(driver_state->display_attributes.size());
    // libva may free this string, so allocate it.
    ctx->str_vendor = strdup("rockchip-mpp");

    util::log(util::stdout_sink, util::LogLevel::Info,
              "Rockchip VA-API driver initialized (libva {}.{}, fd={}, driver={}, device={})",
              ctx->version_major, ctx->version_minor, driver_state->drm_fd,
              driver_state->drm_driver_name, driver_state->drm_device_path);
    return VA_STATUS_SUCCESS;
}

}  // namespace rockchip_vaapi

extern "C" VAStatus vaDriverInit_0_40(VADriverContextP ctx) {
    return rockchip_vaapi::vaDriverInit(ctx);
}

extern "C" VAStatus vaDriverInit_1_20(VADriverContextP ctx) {
    return rockchip_vaapi::vaDriverInit(ctx);
}

// libva expects the symbol to be named __vaDriverInit_<major>_<minor>
extern "C" VAStatus __vaDriverInit_1_0(VADriverContextP ctx) {
    return rockchip_vaapi::vaDriverInit(ctx);
}
