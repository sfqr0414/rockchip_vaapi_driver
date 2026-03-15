#include "mpp_decoder.h"
#include "util/log.h"

#include <cstdarg>
#include <cstddef>
#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>
#include <va/va_dec_av1.h>
#include <drm/drm_fourcc.h>

#include <atomic>
#include <cassert>
#include <cstring>
#include <memory>
#include <unordered_map>
#include <vector>

using namespace rockchip;

struct SpinLock {
    std::atomic_flag flag = ATOMIC_FLAG_INIT;
    void lock() {
        while (flag.test_and_set(std::memory_order_acquire)) {
            // busy-wait
        }
    }
    void unlock() {
        flag.clear(std::memory_order_release);
    }
};

struct ScopedSpinLock {
    SpinLock& lock;
    explicit ScopedSpinLock(SpinLock& l) : lock(l) { lock.lock(); }
    ~ScopedSpinLock() { lock.unlock(); }
};

struct SurfaceState {
    DecodedSurface surface;
    MppBuffer buffer = nullptr;
};

struct BufferInfo {
    void* data = nullptr;
    size_t size = 0;
    VABufferType type = static_cast<VABufferType>(0);
    uint32_t num_elements = 0;
};

struct DriverState {
    MppDecoder decoder;
    std::unordered_map<VASurfaceID, std::unique_ptr<SurfaceState>> surfaces;
    std::unordered_map<VABufferID, BufferInfo> buffers;
    VASurfaceID next_surface_id = 1;
    VABufferID next_buffer_id = 1;
    VASurfaceID current_surface = VA_INVALID_ID;
    std::vector<VABufferID> current_buffers;
    VADriverVTable* vtable = nullptr;
    SpinLock lock;
    VAProfile profile = VAProfileNone;

    // Current expected decoding resolution (from vaCreateContext or MPP feedback).
    uint32_t picture_width = 0;
    uint32_t picture_height = 0;
};

static ptrdiff_t findStartCode(const std::vector<uint8_t>& data, size_t start, size_t& code_len) {
    if (data.size() < 3 || start >= data.size()) return -1;
    for (size_t i = start; i + 3 < data.size(); ++i) {
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
            code_len = 3;
            return static_cast<ptrdiff_t>(i);
        }
        if (i + 4 < data.size() && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) {
            code_len = 4;
            return static_cast<ptrdiff_t>(i);
        }
    }
    return -1;
}

static bool extractAv1SequenceHeader(const std::vector<uint8_t>& bitstream,
                                     size_t& start,
                                     size_t& end) {
    size_t cursor = 0;
    while (cursor < bitstream.size()) {
        size_t code_len = 0;
        ptrdiff_t pos = findStartCode(bitstream, cursor, code_len);
        if (pos < 0) break;
        size_t header_pos = static_cast<size_t>(pos) + code_len;
        if (header_pos >= bitstream.size()) break;
        uint8_t obu_header = bitstream[header_pos];
        uint8_t obu_type = (obu_header >> 3) & 0x1F;
        if (obu_type == 1) {
            size_t next_start = header_pos + 1;
            size_t next_len = 0;
            ptrdiff_t next_pos = findStartCode(bitstream, next_start, next_len);
            start = static_cast<size_t>(pos);
            end = (next_pos < 0) ? bitstream.size() : static_cast<size_t>(next_pos);
            return true;
        }
        cursor = static_cast<size_t>(pos) + 1;
    }
    return false;
}

static std::vector<uint8_t> synthesizeAv1SequenceHeader(const VADecPictureParameterBufferAV1& pic) {
    std::vector<uint8_t> header;
    header.insert(header.end(), {0x00, 0x00, 0x00, 0x01});
    constexpr uint8_t obu_header = 0x0A; // obu_type=1 (Sequence), has_size_field=1
    header.push_back(obu_header);
    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(pic.profile & 0x07));
    payload.push_back(static_cast<uint8_t>((pic.frame_width_minus1 + 1) & 0xFF));
    payload.push_back(static_cast<uint8_t>(((pic.frame_width_minus1 + 1) >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>((pic.frame_height_minus1 + 1) & 0xFF));
    payload.push_back(static_cast<uint8_t>(((pic.frame_height_minus1 + 1) >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>(pic.bit_depth_idx));
    payload.push_back(static_cast<uint8_t>(((pic.frame_width_minus1 + 1) >> 16) & 0xFF));
    payload.push_back(static_cast<uint8_t>(((pic.frame_height_minus1 + 1) >> 16) & 0xFF));
    uint32_t payload_size = static_cast<uint32_t>(payload.size());
    do {
        uint8_t byte = payload_size & 0x7F;
        payload_size >>= 7;
        if (payload_size) byte |= 0x80;
        header.push_back(byte);
    } while (payload_size);
    header.insert(header.end(), payload.begin(), payload.end());
    return header;
}

static DriverState* toDriver(VADriverContextP ctx) {
    return reinterpret_cast<DriverState*>(ctx->pDriverData);
}

static void rockchip_log_info(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[512];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    util::log(util::stdout_sink, util::LogLevel::Info, "{}", buf);
}

static VAStatus rockchip_vaCreateSurfaces(VADriverContextP ctx,
                                         int width,
                                         int height,
                                         int format,
                                         int num_surfaces,
                                         VASurfaceID* surfaces) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    ScopedSpinLock lock(d->lock);
    {
        int initialized = d->decoder.isInitialized() ? 1 : 0;
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "vaCreateSurfaces: decoder isInitialized={}", initialized);
    }

    // Ensure decoder is initialized before allocating surfaces.
    if (!d->decoder.isInitialized()) {
        int profile_int = static_cast<int>(d->profile);
        int w = width;
        int h = height;
        util::log(util::stderr_sink, util::LogLevel::Info,
                  "Initializing decoder for profile={} width={} height={}",
                  profile_int, w, h);
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
        state->surface.stride = ((state->surface.width + 63) / 64) * 64;
        state->surface.is_10bit = (d->profile == VAProfileAV1Profile0);
        state->surface.ready.store(false);

        // Allocate a buffer for the surface using MPP.
        if (!d->decoder.allocateSurface(id, state->surface, width, height)) {
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "Failed to allocate MPP surface for VA surface %u", id);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }

        d->surfaces[id] = std::move(state);
        surfaces[i] = id;
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaDestroySurfaces(VADriverContextP ctx,
                                          VASurfaceID* surfaces,
                                          int num_surfaces) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    ScopedSpinLock lock(d->lock);
    for (int i = 0; i < num_surfaces; i++) {
        auto it = d->surfaces.find(surfaces[i]);
        if (it != d->surfaces.end()) {
            if (it->second->surface.dmabuf_fd >= 0) {
                close(it->second->surface.dmabuf_fd);
            }
            d->surfaces.erase(it);
        }
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaSyncSurface(VADriverContextP ctx, VASurfaceID surface) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    const auto it = d->surfaces.find(surface);
    if (it == d->surfaces.end()) {
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }

    // Wait for decoder to mark the surface ready.
    if (!d->decoder.waitSurfaceReady(surface)) {
        return VA_STATUS_ERROR_TIMEDOUT;
    }

    // Update the driver-side surface dimensions/stride from the decoder (in case MPP changed resolution).
    uint32_t width, height, stride;
    int fd;
    bool failed = false;
    if (d->decoder.getSurfaceInfo(surface, width, height, stride, fd, failed)) {
        if (failed || fd < 0) {
            // If the surface reached ready state but has no valid FD, it's a decoding failure or flush.
            return (failed) ? VA_STATUS_ERROR_DECODING_ERROR : VA_STATUS_ERROR_INVALID_SURFACE;
        }
        it->second->surface.width = width;
        it->second->surface.height = height;
        it->second->surface.stride = stride;
        it->second->surface.dmabuf_fd = fd;
        it->second->surface.ready.store(true);
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaQueryConfigProfiles(VADriverContextP ctx,
                                               VAProfile* profile_list,
                                               int* num_profiles) {
    (void)ctx;
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

static VAStatus rockchip_vaQueryConfigEntrypoints(VADriverContextP ctx,
                                                  VAProfile profile,
                                                  VAEntrypoint* entrypoint_list,
                                                  int* num_entrypoints) {
    (void)ctx;
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

static VAStatus rockchip_vaGetConfigAttributes(VADriverContextP ctx,
                                               VAProfile profile,
                                               VAEntrypoint entrypoint,
                                               VAConfigAttrib* attrib_list,
                                               int num_attribs) {
    (void)ctx;
    (void)entrypoint;

    if (!attrib_list) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (!isSupportedProfile(profile)) return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;

    for (int i = 0; i < num_attribs; i++) {
        if (attrib_list[i].type == VAConfigAttribRTFormat) {
            attrib_list[i].value = VA_RT_FORMAT_YUV420;
        } else {
            attrib_list[i].value = 0;
        }
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaCreateConfig(VADriverContextP ctx,
                                       VAProfile profile,
                                       VAEntrypoint entrypoint,
                                       VAConfigAttrib* attrib_list,
                                       int num_attribs,
                                       VAConfigID* config_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    d->profile = profile;
    (void)entrypoint;
    (void)attrib_list;
    (void)num_attribs;

    // Minimal driver: single config.
    if (config_id) *config_id = 1;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaDestroyConfig(VADriverContextP ctx, VAConfigID config_id) {
    (void)ctx;
    (void)config_id;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaQueryConfigAttributes(VADriverContextP ctx,
                                                 VAConfigID config_id,
                                                 VAProfile* profile,
                                                 VAEntrypoint* entrypoint,
                                                 VAConfigAttrib* attrib_list,
                                                 int* num_attribs) {
    (void)ctx;
    (void)config_id;
    if (profile) *profile = VAProfileH264High;
    if (entrypoint) *entrypoint = VAEntrypointVLD;
    if (attrib_list && num_attribs) {
        for (int i = 0; i < *num_attribs; i++) {
            attrib_list[i].value = 0;
        }
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaCreateContext(VADriverContextP ctx,
                                        VAConfigID config_id,
                                        int picture_width,
                                        int picture_height,
                                        int flag,
                                        VASurfaceID* render_targets,
                                        int num_render_targets,
                                        VAContextID* context) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    (void)config_id;
    (void)flag;
    (void)render_targets;
    (void)num_render_targets;

    if (!isSupportedProfile(d->profile))
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;

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

static VAStatus rockchip_vaDestroyContext(VADriverContextP ctx, VAContextID context) {
    (void)context;
    (void)ctx;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaCreateBuffer(VADriverContextP ctx,
                                       VAContextID /*context*/,
                                       VABufferType type,
                                       unsigned int size,
                                       unsigned int num_elements,
                                       void* data,
                                       VABufferID* buf_id) {
    auto* d = toDriver(ctx);
    if (!d || !buf_id) return VA_STATUS_ERROR_INVALID_CONTEXT;

    size_t total_size = size * (size_t)num_elements;
    void* ptr = nullptr;
    if (data) {
        ptr = malloc(total_size);
        if (!ptr) return VA_STATUS_ERROR_ALLOCATION_FAILED;
        memcpy(ptr, data, total_size);
    } else {
        ptr = malloc(total_size);
        if (!ptr) return VA_STATUS_ERROR_ALLOCATION_FAILED;
        memset(ptr, 0, total_size);
    }

    VABufferID id = d->next_buffer_id++;
    d->buffers[id] = {ptr, total_size, type, num_elements};
    *buf_id = id;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaBufferSetNumElements(VADriverContextP ctx,
                                                VABufferID buf_id,
                                                unsigned int num_elements) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;

    it->second.num_elements = num_elements;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaMapBuffer(VADriverContextP ctx,
                                     VABufferID buf_id,
                                     void** pbuf) {
    if (!pbuf) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;

    *pbuf = it->second.data;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaUnmapBuffer(VADriverContextP ctx,
                                       VABufferID buf_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (d->buffers.find(buf_id) == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaDestroyBuffer(VADriverContextP ctx,
                                         VABufferID buf_id) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;
    free(it->second.data);
    d->buffers.erase(it);
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaBeginPicture(VADriverContextP ctx,
                                       VAContextID /*context*/,
                                       VASurfaceID render_target) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    d->current_surface = render_target;
    d->current_buffers.clear();
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaEndPicture(VADriverContextP ctx,
                                     VAContextID /*context*/) {
    (void)ctx;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaQuerySurfaceStatus(VADriverContextP ctx,
                                              VASurfaceID surface,
                                              VASurfaceStatus* status) {
    if (!status) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto it = d->surfaces.find(surface);
    if (it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;

    *status = it->second->surface.ready.load(std::memory_order_acquire)
                  ? VASurfaceReady
                  : VASurfaceRendering;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaQuerySurfaceError(VADriverContextP ctx,
                                             VASurfaceID surface,
                                             VAStatus error_status,
                                             void** /*error_info*/) {
    (void)ctx;
    (void)surface;
    (void)error_status;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaExportSurfaceHandle(VADriverContextP ctx,
                                               VASurfaceID surface,
                                               uint32_t mem_type,
                                               uint32_t flags,
                                               void* descriptor) {
    (void)flags;
    auto* d = toDriver(ctx);
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
    const uint32_t stride = surf.stride ? surf.stride : ((surf.width + 63) / 64) * 64;
    const uint32_t y_size = stride * surf.height;
    const uint32_t uv_offset = y_size;

    desc->fourcc = VA_FOURCC_NV12;
    desc->width = surf.width;
    desc->height = surf.height;
    desc->num_objects = 1;
    desc->objects[0].fd = surf.dmabuf_fd;
    desc->objects[0].size = uv_offset + (stride / 2) * ((surf.height + 1) / 2) * 2;
    desc->objects[0].drm_format_modifier = 0;

    desc->num_layers = 1;
    desc->layers[0].drm_format = DRM_FORMAT_NV12;
    desc->layers[0].num_planes = 2;
    desc->layers[0].object_index[0] = 0;
    desc->layers[0].object_index[1] = 0;
    desc->layers[0].offset[0] = 0;
    desc->layers[0].offset[1] = uv_offset;
    desc->layers[0].pitch[0] = stride;
    desc->layers[0].pitch[1] = stride;

    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaQueryImageFormats(VADriverContextP ctx,
                                             VAImageFormat* format_list,
                                             int* num_formats) {
    if (!num_formats) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (!format_list) {
        *num_formats = 1;
        return VA_STATUS_SUCCESS;
    }
    if (*num_formats < 1) return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;

    format_list[0].fourcc = VA_FOURCC_NV12;
    format_list[0].depth = 12;
    format_list[0].red_mask = 0;
    format_list[0].green_mask = 0;
    format_list[0].blue_mask = 0;
    format_list[0].alpha_mask = 0;
    *num_formats = 1;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaCreateImage(VADriverContextP ctx,
                                      VAImageFormat* format,
                                      int width,
                                      int height,
                                      VAImage* image) {
    (void)ctx;
    (void)format;
    (void)width;
    (void)height;
    (void)image;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaDeriveImage(VADriverContextP ctx,
                                      VASurfaceID surface,
                                      VAImage* image) {
    (void)ctx;
    (void)surface;
    (void)image;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaDestroyImage(VADriverContextP ctx,
                                       VAImageID image) {
    (void)ctx;
    (void)image;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaSetImagePalette(VADriverContextP ctx,
                                          VAImageID image,
                                          unsigned char* palette) {
    (void)ctx;
    (void)image;
    (void)palette;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaGetImage(VADriverContextP ctx,
                                   VASurfaceID surface,
                                   int x,
                                   int y,
                                   unsigned int w,
                                   unsigned int h,
                                   VAImageID image) {
    (void)ctx;
    (void)surface;
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    (void)image;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaPutImage(VADriverContextP ctx,
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
    (void)ctx;
    (void)surface;
    (void)image;
    (void)srcx;
    (void)srcy;
    (void)srcw;
    (void)srch;
    (void)destx;
    (void)desty;
    (void)destw;
    (void)desth;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaQuerySubpictureFormats(VADriverContextP ctx,
                                                  VAImageFormat* format_list,
                                                  unsigned int* flags,
                                                  unsigned int* num_formats) {
    (void)ctx;
    if (num_formats) *num_formats = 0;
    if (flags) *flags = 0;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaCreateSubpicture(VADriverContextP ctx,
                                            VAImageID image,
                                            VASubpictureID* subpicture) {
    (void)ctx;
    (void)image;
    if (subpicture) *subpicture = VA_INVALID_ID;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaDestroySubpicture(VADriverContextP ctx,
                                             VASubpictureID subpicture) {
    (void)ctx;
    (void)subpicture;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaSetSubpictureImage(VADriverContextP ctx,
                                              VASubpictureID subpicture,
                                              VAImageID image) {
    (void)ctx;
    (void)subpicture;
    (void)image;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaSetSubpictureChromakey(VADriverContextP ctx,
                                                  VASubpictureID subpicture,
                                                  unsigned int chromakey_min,
                                                  unsigned int chromakey_max,
                                                  unsigned int chromakey_mask) {
    (void)ctx;
    (void)subpicture;
    (void)chromakey_min;
    (void)chromakey_max;
    (void)chromakey_mask;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaSetSubpictureGlobalAlpha(VADriverContextP ctx,
                                                     VASubpictureID subpicture,
                                                     float global_alpha) {
    (void)ctx;
    (void)subpicture;
    (void)global_alpha;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaAssociateSubpicture(VADriverContextP ctx,
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
    (void)ctx;
    (void)subpicture;
    (void)target_surfaces;
    (void)num_surfaces;
    (void)src_x;
    (void)src_y;
    (void)src_width;
    (void)src_height;
    (void)dest_x;
    (void)dest_y;
    (void)dest_width;
    (void)dest_height;
    (void)flags;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus rockchip_vaDeassociateSubpicture(VADriverContextP ctx,
                                                 VASubpictureID subpicture,
                                                 VASurfaceID* target_surfaces,
                                                 int num_surfaces) {
    (void)ctx;
    (void)subpicture;
    (void)target_surfaces;
    (void)num_surfaces;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaQueryDisplayAttributes(VADriverContextP ctx,
                                                  VADisplayAttribute* attr_list,
                                                  int* num_attributes) {
    (void)ctx;
    if (num_attributes) *num_attributes = 0;
    (void)attr_list;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaGetDisplayAttributes(VADriverContextP ctx,
                                                VADisplayAttribute* attr_list,
                                                int num_attributes) {
    (void)ctx;
    (void)attr_list;
    (void)num_attributes;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaSetDisplayAttributes(VADriverContextP ctx,
                                                VADisplayAttribute* attr_list,
                                                int num_attributes) {
    (void)ctx;
    (void)attr_list;
    (void)num_attributes;
    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaRenderPicture(VADriverContextP ctx,
                                        VAContextID /*context*/,
                                        VABufferID* buffers,
                                        int num_buffers) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (!buffers || num_buffers <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;

    std::vector<uint8_t> bitstream;
    std::vector<uint8_t> extra_data;
    const uint8_t start_code[] = {0, 0, 0, 1};
    const VADecPictureParameterBufferAV1* av1_picture = nullptr;

    // First pass: find slice parameters and picture parameters
    std::vector<VASliceParameterBufferH264*> h264_slices;
    std::vector<VASliceParameterBufferHEVC*> hevc_slices;
    std::vector<VASliceParameterBufferAV1*> av1_slices;

    for (int i = 0; i < num_buffers; i++) {
        auto it = d->buffers.find(buffers[i]);
        if (it == d->buffers.end()) continue;

        if (it->second.type == VASliceParameterBufferType) {
            uint32_t num = it->second.num_elements;
            if (d->profile == VAProfileH264High) {
                auto* p = static_cast<VASliceParameterBufferH264*>(it->second.data);
                for (uint32_t j = 0; j < num; j++) h264_slices.push_back(&p[j]);
            } else if (d->profile == VAProfileHEVCMain) {
                auto* p = static_cast<VASliceParameterBufferHEVC*>(it->second.data);
                for (uint32_t j = 0; j < num; j++) hevc_slices.push_back(&p[j]);
            } else if (d->profile == VAProfileAV1Profile0) {
                auto* p = static_cast<VASliceParameterBufferAV1*>(it->second.data);
                for (uint32_t j = 0; j < num; j++) av1_slices.push_back(&p[j]);
            }
        }
        if (av1_picture == nullptr && d->profile == VAProfileAV1Profile0 && it->second.type == VAPictureParameterBufferType) {
            if (it->second.size >= sizeof(VADecPictureParameterBufferAV1)) {
                av1_picture = static_cast<const VADecPictureParameterBufferAV1*>(it->second.data);
            }
        }
    }

    // Second pass: extract bitstream data using slices
    for (int i = 0; i < num_buffers; i++) {
        auto it = d->buffers.find(buffers[i]);
        if (it == d->buffers.end()) continue;

        if (it->second.type == VASliceDataBufferType) {
            uint8_t* data = static_cast<uint8_t*>(it->second.data);
            
            if (d->profile == VAProfileH264High && !h264_slices.empty()) {
                for (auto* s : h264_slices) {
                    bitstream.insert(bitstream.end(), start_code, start_code + 4);
                    bitstream.insert(bitstream.end(), data + s->slice_data_offset, data + s->slice_data_offset + s->slice_data_size);
                }
            } else if (d->profile == VAProfileHEVCMain && !hevc_slices.empty()) {
                for (auto* s : hevc_slices) {
                    bitstream.insert(bitstream.end(), start_code, start_code + 4);
                    bitstream.insert(bitstream.end(), data + s->slice_data_offset, data + s->slice_data_offset + s->slice_data_size);
                }
            } else if (d->profile == VAProfileAV1Profile0 && !av1_slices.empty()) {
                for (auto* s : av1_slices) {
                    // AV1 slices are OBUs. They typically don't use 0001 start codes in the same way,
                    // but MPP might expect them if it's in Annex B mode. 
                    // Most libva AV1 slices have the OBU header already.
                    bitstream.insert(bitstream.end(), data + s->slice_data_offset, data + s->slice_data_offset + s->slice_data_size);
                }
            } else {
                // Fallback: raw copy if no slices found
                bitstream.insert(bitstream.end(), data, data + it->second.size);
            }
        }
    }

    if (d->profile == VAProfileAV1Profile0 && extra_data.empty()) {
        size_t seq_start = 0;
        size_t seq_end = 0;
        if (extractAv1SequenceHeader(bitstream, seq_start, seq_end)) {
            extra_data.assign(bitstream.begin() + seq_start, bitstream.begin() + seq_end);
        } else if (av1_picture) {
            extra_data = synthesizeAv1SequenceHeader(*av1_picture);
        }
    }

    if (bitstream.empty()) {
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }

    DecodeJob job;
    job.target_surface = d->current_surface;
    job.bitstream = std::move(bitstream);
    job.extra_data = std::move(extra_data);

    if (!d->decoder.enqueueJob(std::move(job))) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus rockchip_vaDriverInit(VADriverContextP ctx) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;

    // Populate the VA-API function table. libva may free this structure, so
    // allocate it dynamically.
    auto* vt = static_cast<VADriverVTable*>(calloc(1, sizeof(VADriverVTable)));
    if (!vt) return VA_STATUS_ERROR_ALLOCATION_FAILED;

    vt->vaTerminate = [](VADriverContextP ctx) {
        auto* d = toDriver(ctx);
        if (d) {
            d->decoder.shutdown();
            // Destroy the driver state but do not free the memory; libva may manage it.
            d->~DriverState();
            ctx->pDriverData = nullptr;
        }
        return VA_STATUS_SUCCESS;
    };
    vt->vaQueryConfigProfiles = rockchip_vaQueryConfigProfiles;
    vt->vaQueryConfigEntrypoints = rockchip_vaQueryConfigEntrypoints;
    vt->vaGetConfigAttributes = rockchip_vaGetConfigAttributes;
    vt->vaCreateConfig = rockchip_vaCreateConfig;
    vt->vaDestroyConfig = rockchip_vaDestroyConfig;
    vt->vaQueryConfigAttributes = rockchip_vaQueryConfigAttributes;
    vt->vaCreateSurfaces = rockchip_vaCreateSurfaces;
    vt->vaDestroySurfaces = rockchip_vaDestroySurfaces;
    vt->vaCreateContext = rockchip_vaCreateContext;
    vt->vaDestroyContext = rockchip_vaDestroyContext;
    vt->vaCreateBuffer = rockchip_vaCreateBuffer;
    vt->vaBufferSetNumElements = rockchip_vaBufferSetNumElements;
    vt->vaMapBuffer = rockchip_vaMapBuffer;
    vt->vaUnmapBuffer = rockchip_vaUnmapBuffer;
    vt->vaDestroyBuffer = rockchip_vaDestroyBuffer;
    vt->vaBeginPicture = rockchip_vaBeginPicture;
    vt->vaRenderPicture = rockchip_vaRenderPicture;
    vt->vaEndPicture = rockchip_vaEndPicture;
    vt->vaSyncSurface = rockchip_vaSyncSurface;
    vt->vaQuerySurfaceStatus = rockchip_vaQuerySurfaceStatus;
    vt->vaQuerySurfaceError = rockchip_vaQuerySurfaceError;

    vt->vaQueryImageFormats = rockchip_vaQueryImageFormats;
    vt->vaCreateImage = rockchip_vaCreateImage;
    vt->vaDeriveImage = rockchip_vaDeriveImage;
    vt->vaDestroyImage = rockchip_vaDestroyImage;
    vt->vaSetImagePalette = rockchip_vaSetImagePalette;
    vt->vaGetImage = rockchip_vaGetImage;
    vt->vaPutImage = rockchip_vaPutImage;

    vt->vaQuerySubpictureFormats = rockchip_vaQuerySubpictureFormats;
    vt->vaCreateSubpicture = rockchip_vaCreateSubpicture;
    vt->vaDestroySubpicture = rockchip_vaDestroySubpicture;
    vt->vaSetSubpictureImage = rockchip_vaSetSubpictureImage;
    vt->vaSetSubpictureChromakey = rockchip_vaSetSubpictureChromakey;
    vt->vaSetSubpictureGlobalAlpha = rockchip_vaSetSubpictureGlobalAlpha;
    vt->vaAssociateSubpicture = rockchip_vaAssociateSubpicture;
    vt->vaDeassociateSubpicture = rockchip_vaDeassociateSubpicture;

    vt->vaQueryDisplayAttributes = rockchip_vaQueryDisplayAttributes;
    vt->vaGetDisplayAttributes = rockchip_vaGetDisplayAttributes;
    vt->vaSetDisplayAttributes = rockchip_vaSetDisplayAttributes;

    // Required for VA-API DMABUF export (zero copy).
    vt->vaExportSurfaceHandle = rockchip_vaExportSurfaceHandle;

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
    ctx->max_profiles = 4;
    ctx->max_entrypoints = 1;
    ctx->max_attributes = 8;
    ctx->max_image_formats = 1;
    ctx->max_subpic_formats = 1;
    ctx->max_display_attributes = 0;
    // libva may free this string, so allocate it.
    ctx->str_vendor = strdup("rockchip-mpp");

    util::log(util::stdout_sink, util::LogLevel::Info,
              "Rockchip VA-API driver initialized (libva {}.{})",
              ctx->version_major, ctx->version_minor);
    return VA_STATUS_SUCCESS;
}

extern "C" VAStatus vaDriverInit_0_40(VADriverContextP ctx) {
    return rockchip_vaDriverInit(ctx);
}

extern "C" VAStatus vaDriverInit_1_20(VADriverContextP ctx) {
    return rockchip_vaDriverInit(ctx);
}

// libva expects the symbol to be named __vaDriverInit_<major>_<minor>
extern "C" VAStatus __vaDriverInit_1_0(VADriverContextP ctx) {
    return rockchip_vaDriverInit(ctx);
}


