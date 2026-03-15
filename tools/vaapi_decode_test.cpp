#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>

#include <vector>
#include <string>

static bool read_file(const char* path, std::vector<uint8_t>& out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    if (sz <= 0) {
        fclose(f);
        return false;
    }
    fseek(f, 0, SEEK_SET);
    out.resize(sz);
    if (fread(out.data(), 1, sz, f) != (size_t)sz) {
        fclose(f);
        return false;
    }
    fclose(f);
    return true;
}

static bool extract_elementary_stream(const char* mp4_path,
                                     const char* parser_name,
                                     const char* caps,
                                     bool caps_before_parser,
                                     std::vector<uint8_t>& out) {
    // Create a temporary file for the extracted elementary stream.
    char tmp_path[] = "/tmp/vaapi_decode_test_XXXXXX";
    int tmp_fd = mkstemp(tmp_path);
    if (tmp_fd < 0) return false;
    close(tmp_fd);

    // Build a GStreamer pipeline for extracting the elementary stream.
    std::string cmd = "gst-launch-1.0 -q filesrc location=\"" + std::string(mp4_path) +
                      "\" ! qtdemux name=d d.video_0 ! queue";

    if (parser_name && caps) {
        if (caps_before_parser) {
            cmd += " ! ";
            cmd += caps;
            cmd += " ! ";
            cmd += parser_name;
        } else {
            cmd += " ! ";
            cmd += parser_name;
            cmd += " ! ";
            cmd += caps;
        }
    }

    cmd += " ! filesink location=\"" + std::string(tmp_path) + "\"";

    int ret = system(cmd.c_str());
    if (ret != 0) {
        unlink(tmp_path);
        return false;
    }

    bool ok = read_file(tmp_path, out);
    unlink(tmp_path);
    return ok;
}

static bool extract_av1c_from_mp4(const char* mp4_path, std::vector<uint8_t>& out) {
    std::vector<uint8_t> data;
    if (!read_file(mp4_path, data)) return false;
    size_t offset = 0;
    while (offset + 8 <= data.size()) {
        uint32_t size = (uint32_t(data[offset]) << 24) | (uint32_t(data[offset + 1]) << 16) |
                        (uint32_t(data[offset + 2]) << 8) | uint32_t(data[offset + 3]);
        char type[5] = {char(data[offset + 4]), char(data[offset + 5]), char(data[offset + 6]), char(data[offset + 7]), 0};
        if (size < 8) break;
        if (offset + size > data.size()) break;
        if (strcmp(type, "av1C") == 0) {
            // AV1CodecConfigurationRecord: skip the fixed header and return configOBUs.
            // Per ISO/IEC 14496-12, the first 22 bytes are the header fields.
            size_t header_size = 22;
            if (size <= 8 + header_size) return false;
            out.assign(data.begin() + offset + 8 + header_size, data.begin() + offset + size);
            return !out.empty();
        }
        offset += size;
    }
    return false;
}

int main(int argc, char** argv) {
    const char* file = "./videos/Test Jellyfin 1080p AVC 20M.mp4";
    if (argc > 1) file = argv[1];

    std::vector<uint8_t> bitstream;
    VAProfile profile = VAProfileH264High;

    // Use the file name to choose which codec to pull from the container.
    std::string file_str(file);
    const bool is_mp4 = file_str.ends_with(".mp4");

    if (is_mp4) {
        const char* parser = nullptr;
        const char* caps = nullptr;
        bool caps_before_parser = false;
        if (file_str.find("AVC") != std::string::npos) {
            parser = "h264parse";
            caps = "video/x-h264,stream-format=byte-stream";
            profile = VAProfileH264High;
        } else if (file_str.find("HEVC") != std::string::npos) {
            parser = "h265parse";
            caps = "video/x-h265,stream-format=byte-stream";
            profile = VAProfileHEVCMain;
        } else if (file_str.find("AV1") != std::string::npos) {
            // For AV1, use the stream caps first (as in a working gst-launch pipeline) so
            // the parser sees the correct input format and can emit a usable byte-stream.
            parser = "av1parse";
            caps = "video/x-av1";
            caps_before_parser = true;
            profile = VAProfileAV1Profile0;
        } else {
            fprintf(stderr, "Unable to infer codec from filename: %s\n", file);
            return 1;
        }

        if (!extract_elementary_stream(file, parser, caps, caps_before_parser, bitstream)) {
            fprintf(stderr, "Failed to extract elementary stream from: %s\n", file);
            return 1;
        }

        // For AV1, prepend the av1C box data from the MP4 container so the decoder has
        // the required sequence header / initial OBU units.
        if (profile == VAProfileAV1Profile0) {
            std::vector<uint8_t> av1c;
            if (extract_av1c_from_mp4(file, av1c) && !av1c.empty()) {
                std::vector<uint8_t> merged;
                merged.reserve(av1c.size() + bitstream.size());
                merged.insert(merged.end(), av1c.begin(), av1c.end());
                merged.insert(merged.end(), bitstream.begin(), bitstream.end());
                bitstream.swap(merged);
            }
        }
    } else {
        if (!read_file(file, bitstream)) {
            fprintf(stderr, "Failed to read input file: %s\n", file);
            return 1;
        }
    }

    const char* drm_path = "/dev/dri/renderD128";
    int drm_fd = open(drm_path, O_RDWR);
    if (drm_fd < 0) {
        perror("open drm device");
        return 1;
    }

    VADisplay display = vaGetDisplayDRM(drm_fd);
    if (!display) {
        fprintf(stderr, "vaGetDisplayDRM failed\n");
        close(drm_fd);
        return 1;
    }

    int major = 0, minor = 0;
    VAStatus status = vaInitialize(display, &major, &minor);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaInitialize failed: %d\n", status);
        close(drm_fd);
        return 1;
    }
    fprintf(stderr, "vaInitialize %d.%d\n", major, minor);

    int num_profiles = 0;
    status = vaQueryConfigProfiles(display, nullptr, &num_profiles);
    fprintf(stderr, "vaQueryConfigProfiles: %d profiles\n", num_profiles);

    VAEntrypoint entrypoint = VAEntrypointVLD;
    VAConfigAttrib attrib;
    attrib.type = VAConfigAttribRTFormat;
    status = vaGetConfigAttributes(display, profile, entrypoint, &attrib, 1);
    fprintf(stderr, "vaGetConfigAttributes(rtformat) = 0x%x status=%d\n", attrib.value, status);

    VAConfigID config_id;
    status = vaCreateConfig(display, profile, entrypoint, &attrib, 1, &config_id);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaCreateConfig failed %d\n", status);
        return 1;
    }

    const int width = 1920;
    const int height = 1080;
    VASurfaceID surface;
    status = vaCreateSurfaces(display, VA_RT_FORMAT_YUV420, width, height, &surface, 1, nullptr, 0);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaCreateSurfaces failed %d\n", status);
        return 1;
    }

    VAContextID context;
    status = vaCreateContext(display, config_id, width, height, 0, &surface, 1, &context);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaCreateContext failed %d\n", status);
        return 1;
    }

    // Create slice data buffer
    VABufferID slice_buf;
    status = vaCreateBuffer(display, context, VASliceDataBufferType, bitstream.size(), 1, bitstream.data(), &slice_buf);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaCreateBuffer(VASliceDataBufferType) failed %d\n", status);
        return 1;
    }

    status = vaBeginPicture(display, context, surface);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaBeginPicture failed %d\n", status);
        return 1;
    }

    status = vaRenderPicture(display, context, &slice_buf, 1);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaRenderPicture failed %d\n", status);
        return 1;
    }

    status = vaEndPicture(display, context);
    if (status != VA_STATUS_SUCCESS) {
        fprintf(stderr, "vaEndPicture failed %d\n", status);
        return 1;
    }

    status = vaSyncSurface(display, surface);
    fprintf(stderr, "vaSyncSurface returned %d\n", status);

    VADRMPRIMESurfaceDescriptor desc;
    status = vaExportSurfaceHandle(display, surface, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2, 0, &desc);
    fprintf(stderr, "vaExportSurfaceHandle returned %d, fd=%d\n", status, desc.objects[0].fd);

    return 0;
}
