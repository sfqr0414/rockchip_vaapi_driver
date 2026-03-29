#include <drm/drm_fourcc.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_dec_av1.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdarg>
#include <cstddef>
#include <cstring>
#include <span>
#include <sys/stat.h>
#include <mutex>
#include <memory>
#include <unordered_map>
#include <vector>

#include "mpp_decoder.h"
#include "util/bitstream.hpp"
#include "util/log.h"

using namespace rockchip;

namespace rockchip_vaapi {

namespace impl {

struct SurfaceState {
    DecodedSurface surface;
    MppBuffer buffer = nullptr;
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

struct DriverState {
    MppDecoder decoder;
    std::unordered_map<VASurfaceID, std::unique_ptr<SurfaceState>> surfaces;
    std::unordered_map<VABufferID, std::unique_ptr<VABuffer>> buffers;
    DecodeState decode_state;
    VASurfaceID next_surface_id = 1;
    VABufferID next_buffer_id = 1;
    VASurfaceID current_surface = VA_INVALID_ID;
    std::vector<VABufferID> current_buffers;
    VADriverVTable* vtable = nullptr;
    std::mutex lock;
    VAProfile profile = VAProfileNone;

    // Current expected decoding resolution (from vaCreateContext or MPP feedback).
    uint32_t picture_width = 0;
    uint32_t picture_height = 0;
    // Persistent per-picture buffers collected by vaRenderPicture and submitted
    // atomically in vaEndPicture.
    std::vector<uint8_t> frame_buffer;
    std::vector<uint8_t> frame_extra_data;
    bool sequence_headers_sent = false;

    // Images created via vaCreateImage / vaDeriveImage.
    struct ImageState {
        VAImage image;
        VASurfaceID derived_surface = VA_INVALID_ID;
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
    bool config_created = false;
};

static void* mapDmabuf(int fd, size_t size) {
    if (fd < 0 || size == 0) return nullptr;
    return mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
}

static void unmapDmabuf(void* ptr, size_t size) {
    if (ptr && size > 0) munmap(ptr, size);
}

static constexpr uint32_t alignTo(uint32_t value, uint32_t align) {
    assert(align && (align & (align - 1)) == 0);
    return (value + align - 1) & ~(align - 1);
}

static void resetPictureState(DriverState* d) {
    if (!d) return;
    d->frame_buffer.clear();
    d->frame_extra_data.clear();
    d->decode_state.clear();
    d->current_buffers.clear();
    d->current_surface = VA_INVALID_ID;
}

struct ScopedPictureStateReset {
    DriverState* d = nullptr;
    ~ScopedPictureStateReset() { resetPictureState(d); }
};

static int createFallbackSurfaceFd(uint32_t width, uint32_t height, bool is_10bit) {
    uint64_t total_size = 0;
    uint32_t stride = alignTo(width, 64);
    if (!is_10bit) {
        total_size = static_cast<uint64_t>(stride) * height * 3 / 2;
    } else {
        uint32_t aligned_h = ((height + 7) / 8) * 8;
        uint64_t size = static_cast<uint64_t>(stride) * aligned_h;
        size = ((size + 127) / 128) * 128;
        total_size = size * 2;
    }

    char shm_name[64];
    std::snprintf(shm_name, sizeof(shm_name), "/rockchip-vaapi-%d-%u-%u", getpid(), width, height);
    int fd = shm_open(shm_name, O_RDWR | O_CREAT | O_EXCL, 0600);
    if (fd < 0) return -1;
    shm_unlink(shm_name);
    if (ftruncate(fd, static_cast<off_t>(total_size)) != 0) {
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

static void destroyBuffer(DriverState* d, VABufferID buf_id) {
    if (!d) return;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return;
    d->buffers.erase(it);
}

// Display attributes (vaQueryDisplayAttributes / vaGetDisplayAttributes / vaSetDisplayAttributes)
// are global in this driver (no per-display state tracked).
static std::vector<VADisplayAttribute> g_display_attributes;

static void ensureDisplayAttributesInitialized() {
    if (!g_display_attributes.empty()) return;

    auto add = [&](VADisplayAttribType type, int32_t min_v, int32_t max_v, int32_t default_v, uint32_t flags) {
        VADisplayAttribute attr = {};
        attr.type = type;
        attr.min_value = min_v;
        attr.max_value = max_v;
        attr.value = default_v;
        attr.flags = flags;
        g_display_attributes.push_back(attr);
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
        case VAProfileH264High:
        case VAProfileHEVCMain:
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

    const bool av1_10bit = (d->profile == VAProfileAV1Profile0);
    const int expected_format = av1_10bit ? static_cast<int>(VA_RT_FORMAT_YUV420_10) : static_cast<int>(VA_RT_FORMAT_YUV420);
    if (format != expected_format) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "vaCreateSurfaces1: unsupported format={} (expected {})", format, expected_format);
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

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
                    const uint32_t expected_fourcc = av1_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;
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
    {
        static std::atomic<int> create_surface_log_count{0};
        if (create_surface_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
            int initialized = d->decoder.isInitialized() ? 1 : 0;
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "vaCreateSurfaces1: decoder isInitialized={}", initialized);
        }
    }

    if (!d->decoder.isInitialized()) {
        static std::atomic<int> init_log_count{0};
        if (init_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
            int profile_int = static_cast<int>(d->profile);
            util::log(util::stderr_sink, util::LogLevel::Info,
                      "Initializing decoder for profile={} width={} height={}",
                      profile_int, width, height);
        }
        if (!d->decoder.initialize(vaProfileToCodec(d->profile), width, height)) {
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "Failed to initialize decoder for surface allocation");
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
    }

    for (int i = 0; i < num_surfaces; i++) {
        VASurfaceID id = d->next_surface_id++;
        auto state = std::make_unique<SurfaceState>();
        state->surface.va_id = id;
        state->surface.width = static_cast<uint32_t>(width);
        state->surface.height = static_cast<uint32_t>(height);
        state->surface.stride = alignTo(state->surface.width, 64);
        state->surface.is_10bit = (d->profile == VAProfileAV1Profile0);

        if (!d->decoder.allocateSurface(id, state->surface, width, height)) {
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "Failed to allocate MPP surface for VA surface {}", id);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }

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
        d->decoder.destroySurface(surfaces[i]);
        auto it = d->surfaces.find(surfaces[i]);
        if (it != d->surfaces.end()) {
            if (it->second->surface.dmabuf_fd >= 0) {
                close(it->second->surface.dmabuf_fd);
                it->second->surface.dmabuf_fd = -1;
            }
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
    bool ready = d->decoder.waitSurfaceReady(surface);

    uint32_t width = 0, height = 0, stride = 0;
    int fd = -1;
    bool failed = false;
    if (d->decoder.getSurfaceInfo(surface, width, height, stride, fd, failed)) {
        if (!ready && failed) {
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "vaSyncSurface: surface={} completed with decode failure", surface);
            return VA_STATUS_ERROR_DECODING_ERROR;
        }

        if (!ready && !failed) {
            util::log(util::stderr_sink, util::LogLevel::Warn,
                      "vaSyncSurface: surface={} waitSurfaceReady timed out", surface);
            return VA_STATUS_ERROR_TIMEDOUT;
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

            if (it->second->surface.dmabuf_fd >= 0 && it->second->surface.dmabuf_fd != fd) {
                close(it->second->surface.dmabuf_fd);
                it->second->surface.dmabuf_fd = -1;
            }

            int driver_fd = dup(fd);
            if (driver_fd < 0) {
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }

            it->second->surface.width = width;
            it->second->surface.height = height;
            it->second->surface.stride = stride;
            it->second->surface.dmabuf_fd = driver_fd;

            static std::atomic<int> sync_ready_log_count{0};
            if (sync_ready_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
                util::log(util::stderr_sink, util::LogLevel::Info,
                          "vaSyncSurface: surface={} ready w={} h={} stride={} fd={}",
                          surface, width, height, stride, driver_fd);
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
    static const VAProfile supported[] = {
        VAProfileH264High,
        VAProfileHEVCMain,
        VAProfileVP9Profile0,
        VAProfileAV1Profile0,
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

    // Always return success and report the full supported count.
    // Firefox expects the count even if the provided buffer is smaller.
    int fill = std::min(*num_profiles, supported_count);
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
    // We only support decoding
    if (*num_entrypoints < 1) {
        *num_entrypoints = 1;
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
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
        if (attrib_list[i].type == VAConfigAttribRTFormat) {
            attrib_list[i].value = VA_RT_FORMAT_YUV420;
        } else {
            attrib_list[i].value = 0;
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

    // We support only minimal config and ignore per-config attributes here.
    if (attrib_list && num_attribs > 0) {
        for (int i = 0; i < num_attribs; i++) {
            if (attrib_list[i].type == VAConfigAttribRTFormat) {
                attrib_list[i].value = VA_RT_FORMAT_YUV420;
            }
        }
    }

    d->profile = profile;
    d->config_created = true;
    *config_id = 1;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyConfig(VADriverContextP ctx, VAConfigID config_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    // This driver only has a single config
    if (!d->config_created || config_id != 1) {
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }

    d->config_created = false;
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
    if (!d->config_created || config_id != 1) return VA_STATUS_ERROR_INVALID_CONFIG;

    if (profile) *profile = d->profile;
    if (entrypoint) *entrypoint = VAEntrypointVLD;

    // This driver does not expose additional config-level attributes.
    if (attrib_list) {
        for (int i = 0; i < *num_attribs; i++) {
            attrib_list[i].value = 0;
        }
    }

    // Indicate no extra attributes provided.
    *num_attribs = 0;
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
    if (config_id != VA_INVALID_ID && config_id != 1) return VA_STATUS_ERROR_INVALID_CONFIG;

    const bool av1_10bit = (d->profile == VAProfileAV1Profile0);
    constexpr unsigned int kSupportedCount = 2;
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
    attrib_list[0].value.value.i = av1_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;

    attrib_list[1].type = VASurfaceAttribMemoryType;
    attrib_list[1].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    attrib_list[1].value.type = VAGenericValueTypeInteger;
    attrib_list[1].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA;

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
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: invalid pointer ctx={} context={}", (void*)ctx, (void*)context);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    auto* d = toDriver(ctx);
    if (!d) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: invalid driver state");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
    if (!d->config_created || config_id != 1) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: config not ready config_created={} config_id={}", d->config_created, config_id);
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    if (num_render_targets <= 0 || !render_targets) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: invalid render targets count={} ptr={}", num_render_targets, (void*)render_targets);
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

    if (!isSupportedProfile(d->profile)) {
        util::log(util::stderr_sink, util::LogLevel::Warn, "vaCreateContext: unsupported profile={}", d->profile);
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    }

    // Remember the dimensions requested by the client.
    d->picture_width = static_cast<uint32_t>(picture_width);
    d->picture_height = static_cast<uint32_t>(picture_height);

    // Initialize decoder with the requested size; it may adjust later.
    if (!d->decoder.initialize(vaProfileToCodec(d->profile), picture_width, picture_height)) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    // Minimal context ID.
    *context = 1;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyContext(VADriverContextP ctx, VAContextID context) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (context != 1) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    std::lock_guard<std::mutex> lock(d->lock);

    // Stop any decoding activity.
    d->decoder.shutdown();

    // Release driver-owned surface handles after MPP has fully stopped.
    for (auto& kv : d->surfaces) {
        if (kv.second->surface.dmabuf_fd >= 0) {
            close(kv.second->surface.dmabuf_fd);
            kv.second->surface.dmabuf_fd = -1;
        }
    }

    d->surfaces.clear();
    d->ready_flags.clear();
    d->current_surface = VA_INVALID_ID;
    d->current_buffers.clear();

    // Release any buffers and images.
    d->buffers.clear();

    for (auto& kv : d->images) {
        destroyBuffer(d, kv.second.image.buf);
    }
    d->images.clear();

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
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;

    std::span<uint8_t> span = it->second->map();
    *pbuf = span.data();
    return VA_STATUS_SUCCESS;
}

static VAStatus vaUnmapBuffer(VADriverContextP ctx,
                              VABufferID buf_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (d->buffers.find(buf_id) == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyBuffer(VADriverContextP ctx,
                                VABufferID buf_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;
    d->buffers.erase(it);
    return VA_STATUS_SUCCESS;
}

static VAStatus vaBeginPicture(VADriverContextP ctx,
                               VAContextID /*context*/,
                               VASurfaceID render_target) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    resetPictureState(d);
    d->current_surface = render_target;
    {
        std::lock_guard<std::mutex> lock(d->sync_mutex);
        d->ready_flags[render_target].store(false);
    }
    d->sync_cv.notify_all();
    return VA_STATUS_SUCCESS;
}

static VAStatus vaEndPicture(VADriverContextP ctx,
                             VAContextID /*context*/) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    ScopedPictureStateReset cleanup{d};

    // Ensure we have collected data for the current picture.
    if (d->current_surface == VA_INVALID_ID) return VA_STATUS_ERROR_INVALID_SURFACE;
    if (d->frame_buffer.empty()) return VA_STATUS_ERROR_INVALID_BUFFER;

    d->frame_extra_data.clear();
    if (d->decode_state.picture_param_buffer_id) {
        auto it = d->buffers.find(*d->decode_state.picture_param_buffer_id);
        if (it != d->buffers.end()) {
            const auto& picture_param_buffer = *it->second;
            if (d->profile == VAProfileHEVCMain) {
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
                case VAProfileH264High:
                    d->frame_extra_data = bitstream::build_h264_headers(
                        *reinterpret_cast<const VAPictureParameterBufferH264*>(picture_param_buffer.data.data()));
                    break;
                case VAProfileHEVCMain:
                    d->frame_extra_data = bitstream::build_hevc_headers(
                        *reinterpret_cast<const VAPictureParameterBufferHEVC*>(picture_param_buffer.data.data()));
                    break;
                case VAProfileAV1Profile0:
                    {
                        const auto& av1_pic = *reinterpret_cast<const VADecPictureParameterBufferAV1*>(picture_param_buffer.data.data());
                        static std::atomic<int> av1_param_log_count{0};
                        if (av1_param_log_count.fetch_add(1, std::memory_order_relaxed) < 3) {
                            util::log(util::stderr_sink, util::LogLevel::Info,
                                      "av1 params: frame_type={} show_frame={} showable_frame={} error_resilient={} allow_sct={} force_imv={} allow_intrabc={} order_hint={} primary_ref_frame={} base_qindex={} tile_cols={} tile_rows={} interp_filter={} tx_mode={} reference_select={} skip_mode_present={} reduced_tx_set_used={} loop_filter_y0={} loop_filter_y1={} loop_filter_u={} loop_filter_v={} cdef_bits={} cdef_damping={} ref0={} ref1={} ref2={} ref3={} ref4={} ref5={} ref6={}",
                                      av1_pic.pic_info_fields.bits.frame_type,
                                      av1_pic.pic_info_fields.bits.show_frame,
                                      av1_pic.pic_info_fields.bits.showable_frame,
                                      av1_pic.pic_info_fields.bits.error_resilient_mode,
                                      av1_pic.pic_info_fields.bits.allow_screen_content_tools,
                                      av1_pic.pic_info_fields.bits.force_integer_mv,
                                      av1_pic.pic_info_fields.bits.allow_intrabc,
                                      av1_pic.order_hint,
                                      av1_pic.primary_ref_frame,
                                      av1_pic.base_qindex,
                                      av1_pic.tile_cols,
                                      av1_pic.tile_rows,
                                      av1_pic.interp_filter,
                                      av1_pic.mode_control_fields.bits.tx_mode,
                                      av1_pic.mode_control_fields.bits.reference_select,
                                      av1_pic.mode_control_fields.bits.skip_mode_present,
                                      av1_pic.mode_control_fields.bits.reduced_tx_set_used,
                                      av1_pic.filter_level[0],
                                      av1_pic.filter_level[1],
                                      av1_pic.filter_level_u,
                                      av1_pic.filter_level_v,
                                      av1_pic.cdef_bits,
                                      av1_pic.cdef_damping_minus_3,
                                      av1_pic.ref_deltas[0],
                                      av1_pic.ref_deltas[1],
                                      av1_pic.ref_deltas[2],
                                      av1_pic.ref_deltas[3],
                                      av1_pic.ref_deltas[4],
                                      av1_pic.ref_deltas[5],
                                      av1_pic.ref_deltas[6]);
                        }
                        if (av1_param_log_count.load(std::memory_order_relaxed) <= 3) {
                            char prefix[64] = {0};
                            size_t prefix_len = std::min<size_t>(8, d->frame_buffer.size());
                            for (size_t i = 0; i < prefix_len; ++i) {
                                std::snprintf(prefix + i * 3, sizeof(prefix) - i * 3, "%02x ", d->frame_buffer[i]);
                            }
                            util::log(util::stderr_sink, util::LogLevel::Info,
                                      "av1 frame_buffer prefix size={} prefix={}", d->frame_buffer.size(), prefix);
                        }
                        auto sequence_header = bitstream::build_av1_sequence_header(av1_pic);
                        if (!d->sequence_headers_sent) {
                            d->frame_extra_data = std::move(sequence_header);
                        }
                    }
                    break;
                default:
                    break;
            }
            if (!d->frame_extra_data.empty()) {
                d->sequence_headers_sent = true;
            }
        }
    }

    static std::atomic<int> extra_log_count{0};
    if (extra_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaEndPicture extra_data final size={}", d->frame_extra_data.size());
    }

    static std::atomic<int> header_log_count{0};
    if (header_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        size_t picture_param_size = 0;
        if (d->decode_state.picture_param_buffer_id) {
            auto it = d->buffers.find(*d->decode_state.picture_param_buffer_id);
            if (it != d->buffers.end()) {
                picture_param_size = it->second->size_bytes();
            }
        }
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaEndPicture headers: picture_buf={} profile={} extra_size={}",
                  picture_param_size,
                  static_cast<int>(d->profile),
                  d->frame_extra_data.size());
    }

    // Before submitting, reset ready/failed flags so vaSyncSurface will block
    // until MPP reports the frame is decoded and attached.
    d->decoder.resetSurface(d->current_surface);

    static std::atomic<int> end_log_count{0};
    if (end_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        char prefix[64] = {0};
        size_t prefix_len = std::min<size_t>(8, d->frame_buffer.size());
        for (size_t i = 0; i < prefix_len; ++i) {
            std::snprintf(prefix + i * 3, sizeof(prefix) - i * 3, "%02x ", d->frame_buffer[i]);
        }
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaEndPicture: surface={} frame_buffer={} extra_data={} prefix={}",
                  d->current_surface, d->frame_buffer.size(), d->frame_extra_data.size(), prefix);
    }

    DecodeJob job;
    job.target_surface = d->current_surface;
    job.bitstream = std::move(d->frame_buffer);
    job.extra_data = std::move(d->frame_extra_data);
    // Do NOT set EOS here; the decoder will take packets as a stream and
    // should not be forced to flush per VA picture.
    job.eos = false;

    // Submit the complete frame to the decoder thread (which will feed MPP).
    if (!d->decoder.enqueueJob(std::move(job))) {
        util::log(util::stderr_sink, util::LogLevel::Error,
                  "vaEndPicture: enqueueJob failed for surface={}", d->current_surface);
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
    if (!d->decoder.getSurfaceState(surface, ready, failed)) {
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

    // Accept no flags or the defined export flags. In this simple driver,
    // flags are currently ignored but validated to be within supported set.
    if ((flags & ~VA_SURFACE_ATTRIB_MEM_TYPE_VA) != 0) {
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    if (!d) return VA_STATUS_ERROR_INVALID_DISPLAY;
    auto it = d->surfaces.find(surface);
    if (it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;
    if (!descriptor) return VA_STATUS_ERROR_INVALID_PARAMETER;

    if (mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2 &&
        mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME) {
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    }

    auto* desc = static_cast<VADRMPRIMESurfaceDescriptor*>(descriptor);
    memset(desc, 0, sizeof(*desc));

    const auto& surf = it->second->surface;
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
    desc->objects[0].fd = dup(surf.dmabuf_fd);
    if (desc->objects[0].fd < 0) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    desc->objects[0].size = static_cast<uint32_t>(total_size);
    desc->objects[0].drm_format_modifier = 0;

    desc->num_layers = 1;
    desc->layers[0].drm_format = surf.is_10bit ? DRM_FORMAT_P010 : DRM_FORMAT_NV12;
    desc->layers[0].num_planes = 2;
    desc->layers[0].object_index[0] = 0;
    desc->layers[0].object_index[1] = 0;
    desc->layers[0].offset[0] = 0;
    desc->layers[0].offset[1] = uv_offset;
    desc->layers[0].pitch[0] = bytes_per_row;
    desc->layers[0].pitch[1] = bytes_per_row;

    return VA_STATUS_SUCCESS;
}

static bool copySurfaceToImageBuffer(const DecodedSurface& surf,
                                     const VADRMPRIMESurfaceDescriptor& src_desc,
                                     void* dst_ptr,
                                     const VAImage& image) {
    if (!dst_ptr || src_desc.num_objects == 0 || src_desc.objects[0].fd < 0) return false;

    const bool is_10bit = surf.is_10bit;
    const uint32_t bytes_per_sample = is_10bit ? 2u : 1u;
    const uint32_t src_pitch = src_desc.layers[0].pitch[0];
    const uint32_t src_y_offset = src_desc.layers[0].offset[0];
    const uint32_t src_uv_offset = src_desc.layers[0].offset[1];
    const uint32_t dst_pitch = image.pitches[0];
    const uint32_t dst_uv_pitch = image.pitches[1];
    const uint32_t copy_row_bytes = std::min(dst_pitch, surf.width * bytes_per_sample);
    const uint32_t uv_copy_row_bytes = std::min(dst_uv_pitch, surf.width * bytes_per_sample);

    const uint32_t src_plane_size = src_desc.objects[0].size;
    void* src_ptr = mmap(nullptr, src_plane_size, PROT_READ, MAP_SHARED, src_desc.objects[0].fd, 0);
    if (src_ptr == MAP_FAILED) return false;

    uint8_t* dst = static_cast<uint8_t*>(dst_ptr);
    const uint8_t* src = static_cast<const uint8_t*>(src_ptr);
    const uint32_t height = surf.height;
    const uint32_t uv_height = height / 2;

    for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(dst + image.offsets[0] + static_cast<size_t>(row) * dst_pitch,
                    src + src_y_offset + static_cast<size_t>(row) * src_pitch,
                    copy_row_bytes);
    }

    for (uint32_t row = 0; row < uv_height; ++row) {
        std::memcpy(dst + image.offsets[1] + static_cast<size_t>(row) * dst_uv_pitch,
                    src + src_uv_offset + static_cast<size_t>(row) * src_pitch,
                    uv_copy_row_bytes);
    }

    munmap(src_ptr, src_plane_size);
    return true;
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
    uint32_t y_size = pitch * surf.height;
    uint32_t uv_height = (surf.height + 1) / 2;
    uint32_t total_size = y_size + pitch * uv_height;

    VABufferID buf_id = 0;
    VAStatus status = vaCreateBuffer(ctx, 0, VAImageBufferType, total_size, 1, nullptr, &buf_id);
    if (status != VA_STATUS_SUCCESS) return status;

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
    memset(image->va_reserved, 0, sizeof(image->va_reserved));

    VADRMPRIMESurfaceDescriptor src_desc = {};
    if (vaExportSurfaceHandle(ctx, surface,
                              VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                              VA_EXPORT_SURFACE_READ_ONLY, &src_desc) != VA_STATUS_SUCCESS) {
        destroyBuffer(d, buf_id);
        return VA_STATUS_ERROR_UNKNOWN;
    }

    void* image_ptr = nullptr;
    if (vaMapBuffer(ctx, buf_id, &image_ptr) != VA_STATUS_SUCCESS) {
        if (src_desc.objects[0].fd >= 0) close(src_desc.objects[0].fd);
        destroyBuffer(d, buf_id);
        return VA_STATUS_ERROR_UNKNOWN;
    }

    if (!copySurfaceToImageBuffer(surf, src_desc, image_ptr, *image)) {
        vaUnmapBuffer(ctx, buf_id);
        if (src_desc.objects[0].fd >= 0) close(src_desc.objects[0].fd);
        destroyBuffer(d, buf_id);
        return VA_STATUS_ERROR_UNKNOWN;
    }

    vaUnmapBuffer(ctx, buf_id);
    if (src_desc.objects[0].fd >= 0) close(src_desc.objects[0].fd);

    {
        DriverState::ImageState state;
        state.image = *image;
        state.derived_surface = surface;
        d->images[image_id] = std::move(state);
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaDestroyImage(VADriverContextP ctx,
                               VAImageID image) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    auto it = d->images.find(image);
    if (it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    // Free the backing buffer.
    destroyBuffer(d, it->second.image.buf);
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

    auto img_it = d->images.find(image);
    if (img_it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    static std::atomic<int> get_image_state_log_count{0};
    if (get_image_state_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        const auto& surf = surf_it->second->surface;
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage state: surface={} surf_w={} surf_h={} stride={} fd={} img_w={} img_h={} img_pitch={} img_uv_pitch={}",
                  surface, surf.width, surf.height, surf.stride, surf.dmabuf_fd,
                  img_it->second.image.width, img_it->second.image.height,
                  img_it->second.image.pitches[0], img_it->second.image.pitches[1]);
    }

    const bool surf_is_10bit = surf_it->second->surface.is_10bit;
    const uint32_t expected_fourcc = surf_is_10bit ? VA_FOURCC_P010 : VA_FOURCC_NV12;

    // Enforce matching image format for the current surface bit depth.
    if (img_it->second.image.format.fourcc != expected_fourcc ||
        img_it->second.image.format.depth != 12) {
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

    void* img_buf = nullptr;
    static std::atomic<int> get_image_mapbuf_log_count{0};
    if (get_image_mapbuf_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapping image buffer={}", img_it->second.image.buf);
    }
    VAStatus st = vaMapBuffer(ctx, img_it->second.image.buf, &img_buf);
    if (st != VA_STATUS_SUCCESS) return st;
    static std::atomic<int> get_image_mapbuf_done_log_count{0};
    if (get_image_mapbuf_done_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapped image buffer={}", img_it->second.image.buf);
    }

    uint32_t img_pitch = img_it->second.image.pitches[0];
    uint32_t img_uv_pitch = img_it->second.image.pitches[1];

    // Map surface DMABUF and copy the requested region into the image buffer.
    uint32_t surf_pitch = surf.is_10bit ? surf.stride * 2 : surf.stride;
    uint32_t surf_y_size = surf_pitch * surf.height;
    uint32_t surf_total_size = surf_y_size + surf_pitch * ((surf.height + 1) / 2);

    static std::atomic<int> get_image_mapdmabuf_log_count{0};
    if (get_image_mapdmabuf_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapping surface fd={} size={}", surf.dmabuf_fd, surf_total_size);
    }
    void* surf_ptr = mapDmabuf(surf.dmabuf_fd, surf_total_size);
    if (!surf_ptr) return VA_STATUS_ERROR_UNKNOWN;
    static std::atomic<int> get_image_mapdmabuf_done_log_count{0};
    if (get_image_mapdmabuf_done_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaGetImage: mapped surface fd={} size={}", surf.dmabuf_fd, surf_total_size);
    }

    // Copy Y plane
    uint8_t* dst_y = static_cast<uint8_t*>(img_buf);
    uint8_t* src_y = static_cast<uint8_t*>(surf_ptr);
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
    uint8_t* dst_uv = static_cast<uint8_t*>(img_buf) + img_it->second.image.offsets[1];
    uint8_t* src_uv = static_cast<uint8_t*>(surf_ptr) + surf_y_size;
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

    unmapDmabuf(surf_ptr, surf_total_size);
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
    auto img_it = d->images.find(image);
    if (img_it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    // Enforce NV12 images only.
    if (img_it->second.image.format.fourcc != VA_FOURCC_NV12 ||
        img_it->second.image.format.depth != 12) {
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

    uint32_t src_w = std::min<uint32_t>(srcw, img_it->second.image.width - src_x_coord);
    uint32_t src_h = std::min<uint32_t>(srch, img_it->second.image.height - src_y_coord);
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

    void* img_buf = nullptr;
    VAStatus st = vaMapBuffer(ctx, img_it->second.image.buf, &img_buf);
    if (st != VA_STATUS_SUCCESS) return st;

    uint32_t img_pitch = img_it->second.image.pitches[0];
    uint32_t img_uv_pitch = img_it->second.image.pitches[1];

    uint32_t surf_pitch = surf.stride;
    uint32_t surf_y_size = surf_pitch * surf.height;
    uint32_t surf_total_size = surf_y_size + surf_pitch * ((surf.height + 1) / 2);

    void* surf_ptr = mapDmabuf(surf.dmabuf_fd, surf_total_size);
    if (!surf_ptr) return VA_STATUS_ERROR_UNKNOWN;

    uint8_t* dst_y = static_cast<uint8_t*>(surf_ptr);
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
    uint8_t* dst_uv = static_cast<uint8_t*>(surf_ptr) + surf_y_size;
    uint8_t* src_uv = static_cast<uint8_t*>(img_buf) + img_it->second.image.offsets[1];
    for (uint32_t row = 0; row < uv_height; row++) {
        uint8_t* dst_row = dst_uv + ((dst_y_coord / 2 + row) * surf_pitch) + uv_dst_x;
        uint8_t* src_row = src_uv + ((src_y_coord / 2 + row) * img_uv_pitch) + uv_src_x;
        memcpy(dst_row, src_row, uv_w);
    }

    unmapDmabuf(surf_ptr, surf_total_size);
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

static VAStatus vaQueryDisplayAttributes(VADriverContextP /*ctx*/,
                                         VADisplayAttribute* attr_list,
                                         int* num_attributes) {
    if (!num_attributes) return VA_STATUS_ERROR_INVALID_PARAMETER;

    ensureDisplayAttributesInitialized();
    int total = static_cast<int>(g_display_attributes.size());
    if (!attr_list) {
        *num_attributes = total;
        return VA_STATUS_SUCCESS;
    }

    int fill = std::min(*num_attributes, total);
    for (int i = 0; i < fill; i++) {
        attr_list[i] = g_display_attributes[i];
    }
    *num_attributes = total;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaGetDisplayAttributes(VADriverContextP /*ctx*/,
                                       VADisplayAttribute* attr_list,
                                       int num_attributes) {
    if (!attr_list || num_attributes <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;

    ensureDisplayAttributesInitialized();
    for (int i = 0; i < num_attributes; i++) {
        auto* req = &attr_list[i];
        auto it = std::find_if(g_display_attributes.begin(), g_display_attributes.end(),
                               [&](const VADisplayAttribute& a) { return a.type == req->type; });
        if (it == g_display_attributes.end()) return VA_STATUS_ERROR_INVALID_PARAMETER;
        if (!(it->flags & VA_DISPLAY_ATTRIB_GETTABLE)) return VA_STATUS_ERROR_INVALID_PARAMETER;

        // Preserve caller-provided range info, but always return the current value.
        req->min_value = it->min_value;
        req->max_value = it->max_value;
        req->value = it->value;
        req->flags = it->flags;
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaSetDisplayAttributes(VADriverContextP /*ctx*/,
                                       VADisplayAttribute* attr_list,
                                       int num_attributes) {
    if (!attr_list || num_attributes <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;

    ensureDisplayAttributesInitialized();
    for (int i = 0; i < num_attributes; i++) {
        const auto& req = attr_list[i];
        auto it = std::find_if(g_display_attributes.begin(), g_display_attributes.end(),
                               [&](const VADisplayAttribute& a) { return a.type == req.type; });
        if (it == g_display_attributes.end()) return VA_STATUS_ERROR_INVALID_PARAMETER;
        if (!(it->flags & VA_DISPLAY_ATTRIB_SETTABLE)) return VA_STATUS_ERROR_INVALID_PARAMETER;
        if (req.value < it->min_value || req.value > it->max_value) return VA_STATUS_ERROR_INVALID_PARAMETER;
        it->value = req.value;
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus vaRenderPicture(VADriverContextP ctx,
                                VAContextID /*context*/,
                                VABufferID* buffers,
                                int num_buffers) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (!buffers || num_buffers <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;

    const VABuffer* slice_param_buffer = nullptr;
    const VABuffer* slice_data_buffer = nullptr;

    static std::atomic<int> render_log_count{0};
    if (render_log_count.fetch_add(1, std::memory_order_relaxed) < 1) {
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaRenderPicture: surface={} num_buffers={} frame_buffer_before={}",
                  d->current_surface, num_buffers, d->frame_buffer.size());
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

        d->current_buffers.push_back(buffers[i]);

        const auto& vb = *it->second;
        switch (vb.type) {
            case VAPictureParameterBufferType:
                d->decode_state.picture_param_buffer_id = buffers[i];
                break;
            case VASliceParameterBufferType:
                d->decode_state.slice_param_buffer_ids.push_back(buffers[i]);
                slice_param_buffer = &vb;
                break;
            case VASliceDataBufferType:
                d->decode_state.slice_data_buffer_ids.push_back(buffers[i]);
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
            if (d->profile == VAProfileH264High &&
                slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferH264)) {
                framed = bitstream::build_h264_annexb_slices(
                    reinterpret_cast<const VASliceParameterBufferH264*>(slice_param_buffer->data.data()),
                    slice_param_buffer->num_elements,
                    slice_data.data(),
                    slice_data.size());
            } else if (d->profile == VAProfileHEVCMain &&
                       slice_param_buffer->size_bytes() >= slice_param_buffer->num_elements * sizeof(VASliceParameterBufferHEVC)) {
                framed = bitstream::build_hevc_annexb_slices(
                    reinterpret_cast<const VASliceParameterBufferHEVC*>(slice_param_buffer->data.data()),
                    slice_param_buffer->num_elements,
                    slice_data.data(),
                    slice_data.size());
            }
        }

        if (!framed.empty()) {
            d->frame_buffer.insert(d->frame_buffer.end(), framed.begin(), framed.end());
        } else {
            d->frame_buffer.insert(d->frame_buffer.end(), slice_data.begin(), slice_data.end());
        }
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaTerminate(VADriverContextP ctx) {
    auto* d = toDriver(ctx);
    if (d) {
        // Clean up remaining surfaces before decoder teardown so held MPP buffers are released.
        {
            std::lock_guard<std::mutex> lock(d->lock);
            for (auto& kv : d->surfaces) {
                d->decoder.destroySurface(kv.first);
                if (kv.second->surface.dmabuf_fd >= 0) {
                    close(kv.second->surface.dmabuf_fd);
                    kv.second->surface.dmabuf_fd = -1;
                }
                if (kv.second->buffer) {
                    // Ensure any remaining MPP buffers are released.
                    mpp_buffer_put(kv.second->buffer);
                    kv.second->buffer = nullptr;
                }
            }
            d->surfaces.clear();
            for (auto& kv : d->images) {
                destroyBuffer(d, kv.second.image.buf);
            }
            d->images.clear();

            d->buffers.clear();
            d->subpictures.clear();
            d->ready_flags.clear();
        }

        // Stop any decoding activity after surface resources have been dropped.
        d->decoder.shutdown();

        // Destroy the driver state but do not free the memory; libva may manage it.
        d->~DriverState();
        ctx->pDriverData = nullptr;
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
    void* data = malloc(sizeof(DriverState));
    ctx->pDriverData = data;
    if (data) {
        auto* driver_state = new (data) DriverState();
        driver_state->vtable = vt;
    }

    // Pretend to be VA-API 1.2+ to satisfy modern clients.
    ctx->version_major = 1;
    ctx->version_minor = 2;

    // Required by libva initialization checks.
    ctx->max_profiles = 8;
    ctx->max_entrypoints = 4;
    ctx->max_attributes = 16;
    ctx->max_image_formats = 2;
    ctx->max_subpic_formats = 2;
    ctx->max_display_attributes = 4;
    // libva may free this string, so allocate it.
    ctx->str_vendor = strdup("rockchip-mpp");

    util::log(util::stdout_sink, util::LogLevel::Info,
              "Rockchip VA-API driver initialized (libva {}.{})",
              ctx->version_major, ctx->version_minor);
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
