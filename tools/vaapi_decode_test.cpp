extern "C" {
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cerrno>
#include <cstdint>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>
}

#include <util/log.h>

#include <algorithm>
#include <array>
#include <climits>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::string probe_codec(const std::string& infile) {
    std::string cmd = util::format_to("ffprobe -v error -select_streams v:0 -show_entries stream=codec_name -of default=noprint_wrappers=1:nokey=1 \"{}\"", infile);
    util::SubProcess proc(cmd, util::SubProcess::READ);
    std::vector<uint8_t> buffer(128);
    std::string result;
    while (size_t bytes = proc.read(buffer)) {
        result.append(reinterpret_cast<char*>(buffer.data()), bytes);
    }
    if (!result.empty()) {
        result.erase(0, result.find_first_not_of(" \n\r\t"));
        result.erase(result.find_last_not_of(" \n\r\t") + 1);
    }
    return result;
}

static bool probe_dimensions(const std::string& infile, int& width, int& height) {
    std::string cmd = util::format_to("ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of default=noprint_wrappers=1:nokey=1 \"{}\"", infile);
    util::SubProcess proc(cmd, util::SubProcess::READ);
    std::vector<uint8_t> buffer(128);
    std::string result;
    while (size_t bytes = proc.read(buffer)) {
        result.append(reinterpret_cast<char*>(buffer.data()), bytes);
    }
    if (result.empty()) return false;

    // ffprobe prints width and height on separate lines.
    std::istringstream iss(result);
    std::string width_str;
    std::string height_str;
    if (!std::getline(iss, width_str)) return false;
    if (!std::getline(iss, height_str)) return false;
    try {
        width = std::stoi(width_str);
        height = std::stoi(height_str);
    } catch (...) {
        return false;
    }
    return width > 0 && height > 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        util::console("Usage: {} <input.mp4> [max_decoded_frames]\n", argv[0]);
        return 1;
    }

    const char* infile = argv[1];
    int width = 0;
    int height = 0;
    int kMaxDecodedFrames = (argc > 2) ? atoi(argv[2]) : 0; // 0 means no limit (decode full stream)

    if (!probe_dimensions(infile, width, height)) {
        if (!probe_dimensions(infile, width, height)) {
            util::console(std::cerr, "Failed to probe resolution from file, using fallback 1920x1080\n");
            width = 1920;
            height = 1080;
        } else {
            util::console("Probed resolution: {}x{}\n", width, height);
        }
    }

    util::console("Probing codec for file: {}\n", infile);
    std::string codec = probe_codec(infile);
    if (codec.empty()) {
        util::console(std::cerr, "Failed to probe codec from file: {}\n", infile);
        return 1;
    }
    util::console("Detected codec: {}\n", codec);

    // Map ffprobe codec names to our handling categories
    std::string codec_cat;
    if (codec == "av1") {
        codec_cat = "av1";
    } else if (codec == "hevc" || codec == "hevc_videotoolbox" || codec == "h265") {
        codec_cat = "hevc";
    } else if (codec == "h264" || codec == "avc") {
        codec_cat = "h264";
    } else {
        util::console(std::cerr, "Unsupported/untested codec: {}. Attempting h264 fallback.\n", codec);
        codec_cat = "h264";
    }

    VAProfile profile = VAProfileH264High;
    if (codec_cat == "h264")
        profile = VAProfileH264High;
    else if (codec_cat == "hevc")
        profile = VAProfileHEVCMain;
    else if (codec_cat == "av1")
        profile = VAProfileAV1Profile0;

    const char* drm_path = "/dev/dri/renderD128";
    int drm_fd = open(drm_path, O_RDWR);
    if (drm_fd < 0) {
        util::console(std::cerr, "Failed to open DRM device {}: {}\n", drm_path, strerror(errno));
        return 1;
    }

    VADisplay display = vaGetDisplayDRM(drm_fd);
    if (!display) {
        util::console(std::cerr, "vaGetDisplayDRM failed\n");
        close(drm_fd);
        return 1;
    }

    int major = 0, minor = 0;
    if (vaInitialize(display, &major, &minor) != VA_STATUS_SUCCESS) {
        util::console(std::cerr, "vaInitialize failed\n");
        close(drm_fd);
        return 1;
    }

    VAConfigID config_id = VA_INVALID_ID;
    if (vaCreateConfig(display, profile, VAEntrypointVLD, nullptr, 0, &config_id) != VA_STATUS_SUCCESS) {
        util::console(std::cerr, "vaCreateConfig failed\n");
        vaTerminate(display);
        close(drm_fd);
        return 1;
    }

    VASurfaceID surfaces[2];
    if (vaCreateSurfaces(display, VA_RT_FORMAT_YUV420, width, height, surfaces, 2, nullptr, 0) != VA_STATUS_SUCCESS) {
        util::console(std::cerr, "vaCreateSurfaces failed\n");
        vaDestroyConfig(display, config_id);
        vaTerminate(display);
        close(drm_fd);
        return 1;
    }

    VAContextID context = VA_INVALID_ID;
    if (vaCreateContext(display, config_id, width, height, VA_PROGRESSIVE, surfaces, 2, &context) != VA_STATUS_SUCCESS) {
        util::console(std::cerr, "vaCreateContext failed\n");
        vaDestroySurfaces(display, surfaces, 2);
        vaDestroyConfig(display, config_id);
        vaTerminate(display);
        close(drm_fd);
        return 1;
    }

    std::string fmt = (codec_cat == "av1") ? "ivf" : ((codec_cat == "hevc") ? "hevc" : "h264");
    std::string ffmpeg_cmd = util::format_to("ffmpeg -hide_banner -loglevel error -i \"{}\" -vcodec copy -an -f {} pipe:1", infile, fmt);

    util::SubProcess proc(ffmpeg_cmd, util::SubProcess::READ);

    util::console("Running ffmpeg: {}\n", ffmpeg_cmd);

    // FILE* pipe = popen(ffmpeg_cmd.c_str(), "r");
    if (!proc.get_pipe()) {
        util::console(std::cerr, "Failed to run ffmpeg via popen\n");
        vaDestroyContext(display, context);
        vaDestroySurfaces(display, surfaces, 2);
        vaDestroyConfig(display, config_id);
        vaTerminate(display);
        close(drm_fd);
        return 1;
    }

    int surface_idx = 0;
    int decoded_frames = 0;

    // util::SubProcess proc(cmd, util::SubProcess::READ);

    if (codec_cat == "av1") {
        std::vector<uint8_t> ivf_header(32);
        if (proc.read(ivf_header) != 32 || std::memcmp(ivf_header.data(), "DKIF", 4) != 0) {
            util::console(std::cerr, ">> Input: invalid IVF header\n");
            // s->loop_end = true;
            return 0;
        }
    }

    std::vector<uint8_t> payload;
    const size_t CHUNK = 1024 * 1024;

    auto read_next = [&](std::vector<uint8_t>& out) -> bool {
        out.clear();
        if (codec_cat == "av1") {
            std::vector<uint8_t> fh(12);
            if (proc.read(fh) != 12) return false;
            uint32_t frame_size = fh[0] | (fh[1] << 8) | (fh[2] << 16) | (fh[3] << 24);
            if (frame_size == 0) return false;
            out.resize(frame_size);
            return proc.read(out) == frame_size;
        } else {
            out.resize(CHUNK);
            size_t bytes_read = proc.read(out);
            if (bytes_read == 0) return false;
            out.resize(bytes_read);
            return true;
        }
    };

    try {
        while (kMaxDecodedFrames == 0 || decoded_frames < kMaxDecodedFrames) {
            if (!read_next(payload)) break;
            size_t len = payload.size();
            if (len == 0) continue;

            auto& frame = payload;
            auto& frame_size = len;
            // submit entire IVF frame payload as one slice data buffer
            VABufferID buf = VA_INVALID_ID;
            VAStatus st = vaCreateBuffer(display, context, VASliceDataBufferType, frame_size, 1, frame.data(), &buf);
            if (st != VA_STATUS_SUCCESS) {
                util::console(std::cerr, "vaCreateBuffer failed: {}\n", st);
                break;
            }

            st = vaBeginPicture(display, context, surfaces[surface_idx % 2]);
            if (st != VA_STATUS_SUCCESS) {
                util::console(std::cerr, "vaBeginPicture failed: {}\n", st);
                vaDestroyBuffer(display, buf);
                break;
            }

            st = vaRenderPicture(display, context, &buf, 1);
            if (st != VA_STATUS_SUCCESS) {
                util::console(std::cerr, "vaRenderPicture failed: {}\n", st);
            }

            st = vaEndPicture(display, context);
            if (st != VA_STATUS_SUCCESS) {
                util::console(std::cerr, "vaEndPicture failed: {}\n", st);
            }

            auto syncSurfaceWithRetry = [&](VASurfaceID sid)->VAStatus {
                const int kMaxSyncRetry = 50;
                for (int i = 0; i < kMaxSyncRetry; i++) {
                    VAStatus s = vaSyncSurface(display, sid);
                    if (s == VA_STATUS_SUCCESS) return s;
                    if (s == VA_STATUS_ERROR_SURFACE_BUSY || s == VA_STATUS_ERROR_TIMEDOUT || s == VA_STATUS_ERROR_DECODING_ERROR) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        continue;
                    }
                    return s;
                }
                return VA_STATUS_ERROR_TIMEDOUT;
            };

            VAStatus sync = syncSurfaceWithRetry(surfaces[surface_idx % 2]);
            if (sync == VA_STATUS_SUCCESS) {
                VADRMPRIMESurfaceDescriptor desc;
                VAStatus export_status = vaExportSurfaceHandle(display, surfaces[surface_idx % 2],
                                                               VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                                                               VA_EXPORT_SURFACE_READ_ONLY, &desc);
                if (export_status == VA_STATUS_SUCCESS) {
                    for (uint32_t i = 0; i < desc.num_objects; i++) {
                        if (desc.objects[i].fd >= 0) {
                            close(desc.objects[i].fd);
                            desc.objects[i].fd = -1;
                        }
                    }

                    decoded_frames++;
                    surface_idx++;
                    if ((decoded_frames % 100) == 0) {
                        util::console(std::cerr, "Decoded {} frames (surface={} stride={})\n",
                                      decoded_frames, surface_idx - 1, desc.layers[0].pitch[0]);
                    }
                } else {
                    util::console(std::cerr, "vaExportSurfaceHandle failed: {}\n", export_status);
                }
            } else {
                util::console(std::cerr, "vaSyncSurface failed: {} after retries\n", sync);
            }

            vaDestroyBuffer(display, buf);
        }
    } catch (const std::exception& e) {
        util::console(std::cerr, ">> Input Error: {}\n", e.what());
    }


    // cleanup
    vaDestroyContext(display, context);
    vaDestroySurfaces(display, surfaces, 2);
    vaDestroyConfig(display, config_id);
    vaTerminate(display);
    close(drm_fd);

    util::console(std::cerr, "Finished: decoded_frames={}\n", decoded_frames);
    return 0;
}