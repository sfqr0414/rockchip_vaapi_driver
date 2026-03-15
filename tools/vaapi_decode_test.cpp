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
                                     const char* output_caps,
                                     std::vector<uint8_t>& out) {
    char tmp_path[] = "/tmp/vaapi_decode_test_XXXXXX";
    int tmp_fd = mkstemp(tmp_path);
    if (tmp_fd < 0) return false;
    close(tmp_fd);

    std::string cmd = "gst-launch-1.0 -q filesrc location=\"" + std::string(mp4_path) +
                      "\" ! qtdemux name=d d.video_0 ! queue ! " + std::string(parser_name);
    
    if (output_caps && strlen(output_caps) > 0) {
        cmd += " ! \"" + std::string(output_caps) + "\"";
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

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <file.mp4> [width] [height] [codec: h264|hevc|av1]\n", argv[0]);
        return 1;
    }
    const char* file = argv[1];
    int width = (argc > 2) ? atoi(argv[2]) : 1920;
    int height = (argc > 3) ? atoi(argv[3]) : 1080;
    std::string codec_hint = (argc > 4) ? argv[4] : "";

    std::vector<uint8_t> bitstream;
    VAProfile profile = VAProfileH264High;

    std::string file_str(file);
    const char* parser = nullptr;
    const char* out_caps = nullptr;

    if (codec_hint == "h264" || (codec_hint == "" && file_str.find("AVC") != std::string::npos)) {
        parser = "h264parse";
        out_caps = "video/x-h264,stream-format=byte-stream";
        profile = VAProfileH264High;
    } else if (codec_hint == "hevc" || (codec_hint == "" && file_str.find("HEVC") != std::string::npos)) {
        parser = "h265parse";
        out_caps = "video/x-h265,stream-format=byte-stream";
        profile = VAProfileHEVCMain;
    } else if (codec_hint == "av1" || (codec_hint == "" && file_str.find("AV1") != std::string::npos)) {
        parser = "av1parse";
        out_caps = "";
        profile = VAProfileAV1Profile0;
    }

    if (!extract_elementary_stream(file, parser, out_caps, bitstream)) {
        return 1;
    }

    const char* drm_path = "/dev/dri/renderD128";
    int drm_fd = open(drm_path, O_RDWR);
    if (drm_fd < 0) return 1;

    VADisplay display = vaGetDisplayDRM(drm_fd);
    if (!display) return 1;

    int major = 0, minor = 0;
    if (vaInitialize(display, &major, &minor) != VA_STATUS_SUCCESS) return 1;

    VAConfigID config_id;
    vaCreateConfig(display, profile, VAEntrypointVLD, nullptr, 0, &config_id);

    VASurfaceID surfaces[2];
    vaCreateSurfaces(display, VA_RT_FORMAT_YUV420, width, height, surfaces, 2, nullptr, 0);

    VAContextID context;
    vaCreateContext(display, config_id, width, height, VA_PROGRESSIVE, surfaces, 2, &context);

    auto findStartCode = [&](size_t start, size_t& code_len) -> ssize_t {
        for (size_t i = start; i + 3 < bitstream.size(); ++i) {
            if (bitstream[i] == 0 && bitstream[i + 1] == 0 && bitstream[i + 2] == 1) {
                code_len = 3;
                return static_cast<ssize_t>(i);
            }
            if (i + 4 < bitstream.size() && bitstream[i] == 0 && bitstream[i + 1] == 0 && bitstream[i + 2] == 0 && bitstream[i + 3] == 1) {
                code_len = 4;
                return static_cast<ssize_t>(i);
            }
        }
        return -1;
    };

    int surface_idx = 0;
    size_t cursor = 0;
    while (cursor < bitstream.size()) {
        size_t start_len = 0;
        ssize_t start_pos = findStartCode(cursor, start_len);
        if (start_pos < 0) break;

        size_t search_start = static_cast<size_t>(start_pos) + start_len;
        size_t next_len = 0;
        ssize_t next_pos = findStartCode(search_start, next_len);
        size_t end_pos = (next_pos < 0) ? bitstream.size() : static_cast<size_t>(next_pos);
        if (end_pos <= static_cast<size_t>(start_pos)) break;

        std::vector<uint8_t> nal(bitstream.begin() + start_pos, bitstream.begin() + end_pos);
        VABufferID buf;
        vaCreateBuffer(display, context, VASliceDataBufferType, nal.size(), 1, nal.data(), &buf);

        vaBeginPicture(display, context, surfaces[surface_idx % 2]);
        vaRenderPicture(display, context, &buf, 1);
        vaEndPicture(display, context);
        VAStatus sync = vaSyncSurface(display, surfaces[surface_idx % 2]);
        if (sync != VA_STATUS_SUCCESS) {
            fprintf(stderr, "vaSyncSurface failed: %d\n", sync);
        }

        VADRMPRIMESurfaceDescriptor desc;
        if (vaExportSurfaceHandle(display, surfaces[surface_idx % 2], VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2, VA_EXPORT_SURFACE_READ_ONLY, &desc) == VA_STATUS_SUCCESS) {
            fprintf(stderr, "Decoded unit: surface=%d fd=%d stride=%d\n", surface_idx, desc.objects[0].fd, desc.layers[0].pitch[0]);
            surface_idx++;
        }
        vaDestroyBuffer(display, buf);

        if (next_pos < 0) break;
        cursor = static_cast<size_t>(next_pos);
    }

    vaTerminate(display);
    close(drm_fd);
    return 0;
}
