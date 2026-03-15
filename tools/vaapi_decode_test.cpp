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
#include <fstream>

static bool read_file(const char* path, std::vector<uint8_t>& out) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) return false;
    std::streamsize sz = f.tellg();
    if (sz <= 0) return false;
    f.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(sz));
    if (!f.read(reinterpret_cast<char*>(out.data()), sz)) return false;
    return true;
}

static bool run_ffmpeg_capture(const std::string& cmd, std::vector<uint8_t>& out) {
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return false;

    out.clear();
    static const size_t kBufSize = 1 << 16;
    std::vector<uint8_t> buf(kBufSize);
    while (true) {
        size_t n = fread(buf.data(), 1, buf.size(), pipe);
        if (n > 0) out.insert(out.end(), buf.data(), buf.data() + n);
        if (n < buf.size()) break;
    }

    int status = pclose(pipe);
    return status == 0;
}

static bool ends_with(const std::string& s, const std::string& suffix) {
    if (s.size() < suffix.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

// Extract an elementary stream from a file. If the input is an MP4/MKV container
// and we have a codec hint, use ffmpeg to demux to raw bitstream.
static bool extract_av1_config_from_mp4(const char* path, std::vector<uint8_t>& out) {
    // Quick scan for the "av1C" box; in MP4 it is preceded by a 32-bit size field.
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return false;

    const std::string needle = "av1C";
    const size_t buf_size = 1 << 16;
    std::vector<char> buf(buf_size);
    uint64_t offset = 0;
    std::vector<char> carry;

    while (f) {
        f.read(buf.data(), buf_size);
        size_t n = f.gcount();
        if (n == 0) break;

        size_t scan_start = 0;
        if (!carry.empty()) {
            // Prepend carry from previous chunk
            buf.insert(buf.begin(), carry.begin(), carry.end());
            n += carry.size();
            carry.clear();
        }

        for (size_t i = 0; i + needle.size() <= n; i++) {
            if (memcmp(buf.data() + i, needle.data(), needle.size()) == 0) {
                // Found av1C. Read 4 bytes prior for size.
                if (i < 4) break; // not enough room for size field
                uint32_t sz = (uint32_t(uint8_t(buf[i - 4])) << 24) | (uint32_t(uint8_t(buf[i - 3])) << 16) |
                              (uint32_t(uint8_t(buf[i - 2])) << 8) | uint32_t(uint8_t(buf[i - 1]));
                if (sz < 8) return false;
                uint64_t payload_size = sz - 8;
                uint64_t box_start = offset + i - 4;
                uint64_t payload_start = box_start + 8;

                // Read payload
                out.resize(static_cast<size_t>(payload_size));
                f.clear();
                f.seekg(payload_start, std::ios::beg);
                if (!f.read(reinterpret_cast<char*>(out.data()), payload_size)) return false;
                return true;
            }
        }

        // Keep last bytes in case needle spans chunk boundary.
        size_t keep = needle.size() - 1;
        if (n >= keep) {
            carry.assign(buf.end() - keep, buf.end());
        }
        offset += n;
    }
    return false;
}

static bool extract_elementary_stream(const char* path,
                                      const std::string& codec_hint,
                                      std::vector<uint8_t>& out,
                                      bool& out_is_ivf) {
    out_is_ivf = false;
    std::string file(path);

    bool is_container = ends_with(file, ".mp4") || ends_with(file, ".mkv") || ends_with(file, ".mov");
    if (!is_container) {
        return read_file(path, out);
    }

    // If it's an AV1 MP4 container, try to extract the sequence header (av1C)
    // and stash it; the IVF output stream may not contain it.
    std::vector<uint8_t> av1_config;
    if (codec_hint == "av1") {
        if (extract_av1_config_from_mp4(path, av1_config)) {
            fprintf(stderr, "av1 config extracted: %zu bytes\n", av1_config.size());
        } else {
            fprintf(stderr, "av1 config not found in mp4\n");
        }
    }

    // Use ffmpeg to demux container into raw bitstream.
    std::string cmd;
    if (codec_hint == "h264") {
        cmd = "ffmpeg -hide_banner -loglevel error -i \"" + file + "\" -c:v copy -bsf:v h264_mp4toannexb -f h264 -";
    } else if (codec_hint == "hevc") {
        cmd = "ffmpeg -hide_banner -loglevel error -i \"" + file + "\" -c:v copy -bsf:v hevc_mp4toannexb -f hevc -";
    } else if (codec_hint == "av1") {
        // IVF is a simple container for raw AV1 frames.
        out_is_ivf = true;
        cmd = "ffmpeg -hide_banner -loglevel error -i \"" + file + "\" -c:v copy -f ivf -";
    } else {
        // Unknown codec hint: fall back to raw file read.
        return read_file(path, out);
    }

    if (!run_ffmpeg_capture(cmd, out))
        return false;

    if (!av1_config.empty()) {
        // Prepend AV1 config OBUs to the stream so the decoder can initialize.
        // Ensure there is a start code prefix for the parser.
        const uint8_t start_code[] = {0, 0, 0, 1};
        std::vector<uint8_t> combined;
        combined.reserve(sizeof(start_code) + av1_config.size() + out.size());
        combined.insert(combined.end(), start_code, start_code + sizeof(start_code));
        combined.insert(combined.end(), av1_config.begin(), av1_config.end());
        combined.insert(combined.end(), out.begin(), out.end());
        out.swap(combined);
    }

    return true;
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
    bool is_ivf = false;

    std::string file_str(file);
    std::string codec = codec_hint;
    if (codec.empty()) {
        if (file_str.find("AVC") != std::string::npos) codec = "h264";
        if (file_str.find("HEVC") != std::string::npos) codec = "hevc";
        if (file_str.find("AV1") != std::string::npos) codec = "av1";
    }

    if (codec == "h264") {
        profile = VAProfileH264High;
    } else if (codec == "hevc") {
        profile = VAProfileHEVCMain;
    } else if (codec == "av1") {
        profile = VAProfileAV1Profile0;
    }

    if (!extract_elementary_stream(file, codec, bitstream, is_ivf)) {
        fprintf(stderr, "Failed to read input stream: %s\n", file);
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

    if (codec == "av1" && is_ivf) {
        // IVF stream: parse each frame and submit it as a single packet.
        const size_t kIVFHeaderSize = 32;
        if (bitstream.size() < kIVFHeaderSize) {
            fprintf(stderr, "IVF stream too small\n");
            return 1;
        }
        size_t pos = kIVFHeaderSize;
        while (pos + 12 <= bitstream.size()) {
            uint32_t frame_size = bitstream[pos] | (bitstream[pos + 1] << 8) | (bitstream[pos + 2] << 16) | (bitstream[pos + 3] << 24);
            pos += 12; // 4 bytes size + 8 bytes pts
            if (pos + frame_size > bitstream.size()) break;

            // Prepend Annex-B start code to allow simple NAL/OBU parsing.
            const uint8_t start_code[] = {0, 0, 0, 1};
            std::vector<uint8_t> nal;
            nal.reserve(sizeof(start_code) + frame_size);
            nal.insert(nal.end(), start_code, start_code + sizeof(start_code));
            nal.insert(nal.end(), bitstream.begin() + pos, bitstream.begin() + pos + frame_size);
            pos += frame_size;

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
        }
    } else {
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
    }

    vaTerminate(display);
    close(drm_fd);
    return 0;
}
