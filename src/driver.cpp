#include "mpp_decoder.h"
#include "util/log.h"

#include <cstdarg>
#include <cstddef>
#include <fcntl.h>
#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>
#include <va/va_dec_av1.h>
#include <drm/drm_fourcc.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstring>
#include <memory>
#include <sys/mman.h>
#include <unordered_map>
#include <vector>

using namespace rockchip;

namespace rockchip_vaapi {

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
    // Persistent per-picture buffers collected by vaRenderPicture and submitted
    // atomically in vaEndPicture.
    std::vector<uint8_t> frame_buffer;
    std::vector<uint8_t> frame_extra_data;
    // For AV1 we may receive picture params separately; keep a copy to
    // synthesize sequence header at EndPicture if needed.
    std::vector<uint8_t> pending_av1_pic_blob;

    // Images created via vaCreateImage / vaDeriveImage.
    struct ImageState {
        VAImage image;
        VASurfaceID derived_surface = VA_INVALID_ID;
        std::vector<uint8_t> palette;
    };
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
    };
    std::unordered_map<VASubpictureID, SubpictureState> subpictures;
    VASubpictureID next_subpicture_id = 1;
};


static void* mapDmabuf(int fd, size_t size) {
    if (fd < 0 || size == 0) return nullptr;
    return mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
}

static void unmapDmabuf(void* ptr, size_t size) {
    if (ptr && size > 0) munmap(ptr, size);
}

static void destroyBuffer(DriverState* d, VABufferID buf_id) {
    if (!d) return;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return;
    free(it->second.data);
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

static void log_info(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char buf[512];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    util::log(util::stdout_sink, util::LogLevel::Info, "{}", buf);
}

static VAStatus vaCreateSurfaces(VADriverContextP ctx,
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

static VAStatus vaDestroySurfaces(VADriverContextP ctx,
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

static VAStatus vaSyncSurface(VADriverContextP ctx, VASurfaceID surface) {
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
            util::log(util::stderr_sink, util::LogLevel::Error,
                      "vaSyncSurface: surface={} failed={} fd={}", surface, (int)failed, fd);
            return (failed) ? VA_STATUS_ERROR_DECODING_ERROR : VA_STATUS_ERROR_INVALID_SURFACE;
        }
        it->second->surface.width = width;
        it->second->surface.height = height;
        it->second->surface.stride = stride;
        it->second->surface.dmabuf_fd = fd;
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaQueryConfigProfiles(VADriverContextP ctx,
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

static VAStatus vaQueryConfigEntrypoints(VADriverContextP ctx,
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

static VAStatus vaGetConfigAttributes(VADriverContextP ctx,
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

static VAStatus vaCreateConfig(VADriverContextP ctx,
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

static VAStatus vaDestroyConfig(VADriverContextP ctx, VAConfigID config_id) {
    (void)ctx;
    (void)config_id;
    return VA_STATUS_SUCCESS;
}

static VAStatus vaQueryConfigAttributes(VADriverContextP ctx,
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

static VAStatus vaCreateContext(VADriverContextP ctx,
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

static VAStatus vaDestroyContext(VADriverContextP ctx, VAContextID context) {
    (void)context;
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    ScopedSpinLock lock(d->lock);

    // Stop any decoding activity.
    d->decoder.shutdown();

    // Release all allocated surfaces and their dmabuf fds.
    for (auto& kv : d->surfaces) {
        if (kv.second->surface.dmabuf_fd >= 0) {
            close(kv.second->surface.dmabuf_fd);
        }
    }
    d->surfaces.clear();
    d->current_surface = VA_INVALID_ID;
    d->current_buffers.clear();

    // Release any buffers and images.
    for (auto& kv : d->buffers) {
        free(kv.second.data);
    }
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

static VAStatus vaBufferSetNumElements(VADriverContextP ctx,
                                                VABufferID buf_id,
                                                unsigned int num_elements) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;
    auto it = d->buffers.find(buf_id);
    if (it == d->buffers.end()) return VA_STATUS_ERROR_INVALID_BUFFER;

    it->second.num_elements = num_elements;
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

    *pbuf = it->second.data;
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
    free(it->second.data);
    d->buffers.erase(it);
    return VA_STATUS_SUCCESS;
}

static VAStatus vaBeginPicture(VADriverContextP ctx,
                                       VAContextID /*context*/,
                                       VASurfaceID render_target) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    // When a surface is reused for a new picture, release any previous decoded
    // buffer so MPP can reclaim the slot and avoid leaving it permanently "in use".
    d->decoder.releaseSurface(render_target);

    d->current_surface = render_target;
    d->current_buffers.clear();
    return VA_STATUS_SUCCESS;
}

static VAStatus vaEndPicture(VADriverContextP ctx,
                                     VAContextID /*context*/) {
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    // Ensure we have collected data for the current picture.
    if (d->current_surface == VA_INVALID_ID) return VA_STATUS_ERROR_INVALID_SURFACE;
    if (d->frame_buffer.empty()) return VA_STATUS_ERROR_INVALID_BUFFER;

    // For AV1, we currently skip sequence extraction/synthesis and rely on
    // external bitstream being complete enough.

    // Before submitting, reset ready/failed flags so vaSyncSurface will block
    // until MPP reports the frame is decoded and attached.
    d->decoder.resetSurface(d->current_surface);

    DecodeJob job;
    job.target_surface = d->current_surface;
    job.bitstream = std::move(d->frame_buffer);
    job.extra_data = std::move(d->frame_extra_data);
    // Do NOT set EOS here; the decoder will take packets as a stream and
    // should not be forced to flush per VA picture.
    job.eos = false;

    // Submit the complete frame to the decoder thread (which will feed MPP).
    if (!d->decoder.enqueueJob(std::move(job))) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    // Clear per-picture state so next picture can be collected.
    d->frame_buffer.clear();
    d->frame_extra_data.clear();
    d->pending_av1_pic_blob.clear();
    d->current_buffers.clear();

    return VA_STATUS_SUCCESS;
}

static VAStatus vaQuerySurfaceStatus(VADriverContextP ctx,
                                              VASurfaceID surface,
                                              VASurfaceStatus* status) {
    if (!status) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    uint32_t width = 0, height = 0, stride = 0;
    int fd = -1;
    bool failed = false;
    if (!d->decoder.getSurfaceInfo(surface, width, height, stride, fd, failed)) {
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }

    *status = (failed || fd < 0) ? VASurfaceRendering : VASurfaceReady;
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

    uint64_t total_size;
    if (!surf.is_10bit) {
        total_size = static_cast<uint64_t>(stride) * surf.height * 3 / 2;
    } else {
        uint32_t aligned_h = ((surf.height + 7) / 8) * 8;
        uint64_t size = static_cast<uint64_t>(stride) * aligned_h;
        size = ((size + 127) / 128) * 128;
        total_size = size * 2;
    }

    desc->fourcc = VA_FOURCC_NV12;
    desc->width = surf.width;
    desc->height = surf.height;
    desc->num_objects = 1;
    desc->objects[0].fd = surf.dmabuf_fd;
    desc->objects[0].size = static_cast<uint32_t>(total_size);
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

static VAStatus vaQueryImageFormats(VADriverContextP ctx,
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

static VAStatus vaCreateImage(VADriverContextP ctx,
                                      VAImageFormat* format,
                                      int width,
                                      int height,
                                      VAImage* image) {
    if (!ctx || !format || !image) return VA_STATUS_ERROR_INVALID_PARAMETER;
    auto* d = toDriver(ctx);
    if (!d) return VA_STATUS_ERROR_INVALID_CONTEXT;

    if (width <= 0 || height <= 0) return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (format->fourcc != VA_FOURCC_NV12 || format->depth != 12) {
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    uint32_t pitch = ((static_cast<uint32_t>(width) + 63) / 64) * 64;
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

    auto it = d->surfaces.find(surface);
    if (it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;

    const auto& surf = it->second->surface;
    if (surf.dmabuf_fd < 0) return VA_STATUS_ERROR_INVALID_SURFACE;

    // Only support NV12 surfaces.
    VAImageFormat fmt = {};
    fmt.fourcc = VA_FOURCC_NV12;
    fmt.depth = 12;
    fmt.red_mask = 0;
    fmt.green_mask = 0;
    fmt.blue_mask = 0;
    fmt.alpha_mask = 0;

    uint32_t pitch = ((surf.width + 63) / 64) * 64;
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

    auto surf_it = d->surfaces.find(surface);
    if (surf_it == d->surfaces.end()) return VA_STATUS_ERROR_INVALID_SURFACE;

    auto img_it = d->images.find(image);
    if (img_it == d->images.end()) return VA_STATUS_ERROR_INVALID_IMAGE;

    // Enforce NV12 images only.
    if (img_it->second.image.format.fourcc != VA_FOURCC_NV12 ||
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

    // NV12 requires even coordinates for UV plane sampling.
    if ((x_coord & 1) || (y_coord & 1)) return VA_STATUS_ERROR_INVALID_PARAMETER;

    uint32_t max_w = std::min<uint32_t>(w, surf.width - x_coord);
    uint32_t max_h = std::min<uint32_t>(h, surf.height - y_coord);
    max_w &= ~1u;
    max_h &= ~1u;

    void* img_buf = nullptr;
    VAStatus st = vaMapBuffer(ctx, img_it->second.image.buf, &img_buf);
    if (st != VA_STATUS_SUCCESS) return st;

    uint32_t img_pitch = img_it->second.image.pitches[0];
    uint32_t img_uv_pitch = img_it->second.image.pitches[1];

    // Map surface DMABUF and copy the requested region into the image buffer.
    uint32_t surf_pitch = surf.stride;
    uint32_t surf_y_size = surf_pitch * surf.height;
    uint32_t surf_total_size = surf_y_size + surf_pitch * ((surf.height + 1) / 2);

    void* surf_ptr = mapDmabuf(surf.dmabuf_fd, surf_total_size);
    if (!surf_ptr) return VA_STATUS_ERROR_UNKNOWN;

    // Copy Y plane
    uint8_t* dst_y = static_cast<uint8_t*>(img_buf);
    uint8_t* src_y = static_cast<uint8_t*>(surf_ptr);
    for (uint32_t row = 0; row < max_h; row++) {
        uint8_t* dst_row = dst_y + row * img_pitch;
        uint8_t* src_row = src_y + (y_coord + row) * surf_pitch + x_coord;
        memcpy(dst_row, src_row, max_w);
    }

    // Copy UV plane
    uint32_t uv_height = (max_h + 1) / 2;
    uint32_t uv_x = x_coord & ~1u;
    uint32_t uv_w = (max_w + 1) / 2 * 2;
    uint8_t* dst_uv = static_cast<uint8_t*>(img_buf) + img_it->second.image.offsets[1];
    uint8_t* src_uv = static_cast<uint8_t*>(surf_ptr) + surf_y_size;
    for (uint32_t row = 0; row < uv_height; row++) {
        uint8_t* dst_row = dst_uv + row * img_uv_pitch;
        uint8_t* src_row = src_uv + ((static_cast<uint32_t>(y) / 2 + row) * surf_pitch) + uv_x;
        memcpy(dst_row, src_row, uv_w);
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
    (void)ctx;
    if (num_formats) *num_formats = 0;
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

    // This driver does not perform actual composition, but stores the last
    // association parameters for debugging / future implementation.
    (void)src_x;
    (void)src_y;
    (void)src_width;
    (void)src_height;
    (void)dest_x;
    (void)dest_y;
    (void)dest_width;
    (void)dest_height;
    (void)flags;
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

    const uint8_t start_code[] = {0, 0, 0, 1};

    // Collect slice data and any picture-parameter blobs into the driver's
    // persistent per-picture buffers. Do NOT submit to the decoder here.
    for (int i = 0; i < num_buffers; i++) {
        auto it = d->buffers.find(buffers[i]);
        if (it == d->buffers.end()) continue;

        if (it->second.type == VASliceDataBufferType) {
            uint8_t* data = static_cast<uint8_t*>(it->second.data);
            // For H264/HEVC append start codes; for AV1 append raw OBUs.
            if (d->profile == VAProfileH264High || d->profile == VAProfileHEVCMain) {
                d->frame_buffer.insert(d->frame_buffer.end(), start_code, start_code + 4);
                d->frame_buffer.insert(d->frame_buffer.end(), data, data + it->second.size);
            } else {
                d->frame_buffer.insert(d->frame_buffer.end(), data, data + it->second.size);
            }
        }

        // Capture AV1 picture parameter buffer so EndPicture can synthesize
        // a sequence header if needed.
        if (d->profile == VAProfileAV1Profile0 && it->second.type == VAPictureParameterBufferType) {
            d->pending_av1_pic_blob.assign(static_cast<uint8_t*>(it->second.data), static_cast<uint8_t*>(it->second.data) + it->second.size);
        }
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus vaDriverInit(VADriverContextP ctx) {
    if (!ctx) return VA_STATUS_ERROR_INVALID_CONTEXT;

    // Populate the VA-API function table. libva may free this structure, so
    // allocate it dynamically.
    auto* vt = static_cast<VADriverVTable*>(calloc(1, sizeof(VADriverVTable)));
    if (!vt) return VA_STATUS_ERROR_ALLOCATION_FAILED;

    vt->vaTerminate = [](VADriverContextP ctx) {
        auto* d = toDriver(ctx);
        if (d) {
            // Stop any decoding activity first.
            d->decoder.shutdown();

            // Clean up remaining surfaces and buffers (libva may not call vaDestroy*).
            {
                ScopedSpinLock lock(d->lock);
                for (auto& kv : d->surfaces) {
                    if (kv.second->surface.dmabuf_fd >= 0) {
                        close(kv.second->surface.dmabuf_fd);
                    }
                    if (kv.second->buffer) {
                        // Ensure any remaining MPP buffers are released.
                        mpp_buffer_put(kv.second->buffer);
                        kv.second->buffer = nullptr;
                    }
                }
                d->surfaces.clear();

                for (auto& kv : d->buffers) {
                    free(kv.second.data);
                }
                d->buffers.clear();

                for (auto& kv : d->images) {
                    destroyBuffer(d, kv.second.image.buf);
                }
                d->images.clear();
                d->subpictures.clear();
            }

            // Destroy the driver state but do not free the memory; libva may manage it.
            d->~DriverState();
            ctx->pDriverData = nullptr;
        }
        return VA_STATUS_SUCCESS;
    };
    vt->vaQueryConfigProfiles = vaQueryConfigProfiles;
    vt->vaQueryConfigEntrypoints = vaQueryConfigEntrypoints;
    vt->vaGetConfigAttributes = vaGetConfigAttributes;
    vt->vaCreateConfig = vaCreateConfig;
    vt->vaDestroyConfig = vaDestroyConfig;
    vt->vaQueryConfigAttributes = vaQueryConfigAttributes;
    vt->vaCreateSurfaces = vaCreateSurfaces;
    vt->vaDestroySurfaces = vaDestroySurfaces;
    vt->vaCreateContext = vaCreateContext;
    vt->vaDestroyContext = vaDestroyContext;
    vt->vaCreateBuffer = vaCreateBuffer;
    vt->vaBufferSetNumElements = vaBufferSetNumElements;
    vt->vaMapBuffer = vaMapBuffer;
    vt->vaUnmapBuffer = vaUnmapBuffer;
    vt->vaDestroyBuffer = vaDestroyBuffer;
    vt->vaBeginPicture = vaBeginPicture;
    vt->vaRenderPicture = vaRenderPicture;
    vt->vaEndPicture = vaEndPicture;
    vt->vaSyncSurface = vaSyncSurface;
    vt->vaQuerySurfaceStatus = vaQuerySurfaceStatus;
    vt->vaQuerySurfaceError = vaQuerySurfaceError;

    vt->vaQueryImageFormats = vaQueryImageFormats;
    vt->vaCreateImage = vaCreateImage;
    vt->vaDeriveImage = vaDeriveImage;
    vt->vaDestroyImage = vaDestroyImage;
    vt->vaSetImagePalette = vaSetImagePalette;
    vt->vaGetImage = vaGetImage;
    vt->vaPutImage = vaPutImage;

    vt->vaQuerySubpictureFormats = vaQuerySubpictureFormats;
    vt->vaCreateSubpicture = vaCreateSubpicture;
    vt->vaDestroySubpicture = vaDestroySubpicture;
    vt->vaSetSubpictureImage = vaSetSubpictureImage;
    vt->vaSetSubpictureChromakey = vaSetSubpictureChromakey;
    vt->vaSetSubpictureGlobalAlpha = vaSetSubpictureGlobalAlpha;
    vt->vaAssociateSubpicture = vaAssociateSubpicture;
    vt->vaDeassociateSubpicture = vaDeassociateSubpicture;

    vt->vaQueryDisplayAttributes = vaQueryDisplayAttributes;
    vt->vaGetDisplayAttributes = vaGetDisplayAttributes;
    vt->vaSetDisplayAttributes = vaSetDisplayAttributes;

    // Required for VA-API DMABUF export (zero copy).
    vt->vaExportSurfaceHandle = vaExportSurfaceHandle;

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

} // namespace rockchip_vaapi

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


