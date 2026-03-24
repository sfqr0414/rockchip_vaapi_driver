extern "C" {
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
}

#include <util/log.h>

#include <atomic>
#include <filesystem>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include "rockchip/mpp_log.h"
#include "rockchip/rk_mpi.h"
}

/* ---- Shared context between threads ---- */
struct Shared {
    MppCtx ctx = nullptr;
    MppApi* mpi = nullptr;

    std::atomic<bool> loop_end{false};
    std::atomic<int> frame_count{0};
    bool is_av1 = false;
};

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

/* --- 恢复到最清爽的 8-bit 输出逻辑 --- */
static void save_frame_to_jpg(MppFrame frame, int frame_num) {
    uint32_t width = mpp_frame_get_width(frame);
    uint32_t height = mpp_frame_get_height(frame);
    uint32_t hor_stride = mpp_frame_get_hor_stride(frame);
    uint32_t ver_stride = mpp_frame_get_ver_stride(frame);

    MppBuffer buffer = mpp_frame_get_buffer(frame);
    if (!buffer) return;

    void* ptr = mpp_buffer_get_ptr(buffer);
    if (!ptr) return;

    mpp_buffer_sync_begin(buffer);

    uint8_t* base = static_cast<uint8_t*>(ptr);
    uint8_t* y_base = base;
    uint8_t* uv_base = base + hor_stride * ver_stride;

    std::string filename = util::format_to("output/frame_{:04d}.jpg", frame_num);
    std::string cmd = util::format_to(
        "ffmpeg -hide_banner -loglevel error -y -f rawvideo -pixel_format nv12 "
        "-video_size {}x{} -i pipe:0 -vframes 1 \"{}\"",
        width, height, filename);

    try {
        util::SubProcess proc(cmd, util::SubProcess::WRITE);

        for (uint32_t i = 0; i < height; ++i) {
            proc.write(y_base + i * hor_stride, width);
        }
        for (uint32_t i = 0; i < height / 2; ++i) {
            proc.write(uv_base + i * hor_stride, width);
        }
    } catch (const std::exception& e) {
        util::console(std::cerr, ">> Write Error: {}\n", e.what());
    }

    mpp_buffer_sync_end(buffer);
}

void input_thread(Shared* s, std::string file, std::string codec) {
    std::string fmt = (codec == "av1") ? "ivf" : ((codec == "hevc") ? "hevc" : "h264");
    std::string cmd = util::format_to("ffmpeg -hide_banner -loglevel error -i \"{}\" -vcodec copy -an -f {} pipe:1", file, fmt);

    try {
        util::SubProcess proc(cmd, util::SubProcess::READ);

        if (s->is_av1) {
            std::vector<uint8_t> ivf_header(32);
            if (proc.read(ivf_header) != 32 || std::memcmp(ivf_header.data(), "DKIF", 4) != 0) {
                util::console(std::cerr, ">> Input: invalid IVF header\n");
                s->loop_end = true;
                return;
            }
        }

        std::vector<uint8_t> payload;
        const size_t CHUNK = 1024 * 1024;

        auto read_next = [&](std::vector<uint8_t>& out) -> bool {
            out.clear();
            if (s->is_av1) {
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

        while (!s->loop_end) {
            if (!read_next(payload)) break;
            size_t len = payload.size();
            if (len == 0) continue;

            MppPacket pkt = nullptr;
            if (mpp_packet_init(&pkt, payload.data(), len) != MPP_OK) {
                util::console(std::cerr, ">> Input: mpp_packet_init failed\n");
                s->loop_end = true;
                break;
            }

            while (!s->loop_end) {
                MPP_RET ret = s->mpi->decode_put_packet(s->ctx, pkt);
                if (ret == MPP_OK) {
                    break;
                } else if (ret == MPP_ERR_BUFFER_FULL) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    util::console(std::cerr, ">> Input: decode_put_packet error: {}\n", ret);
                    s->loop_end = true;
                    break;
                }
            }

            if (pkt) mpp_packet_deinit(&pkt);
        }

    } catch (const std::exception& e) {
        util::console(std::cerr, ">> Input Error: {}\n", e.what());
        s->loop_end = true;
    }

    MppPacket eos_pkt = nullptr;
    mpp_packet_init(&eos_pkt, nullptr, 0);
    mpp_packet_set_eos(eos_pkt);
    for (int i = 0; i < 200 && !s->loop_end; ++i) {
        if (s->mpi->decode_put_packet(s->ctx, eos_pkt) == MPP_OK) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (eos_pkt) mpp_packet_deinit(&eos_pkt);

    util::console(">> Input: Stream Finished.\n");
}

void output_thread(Shared* s) {
    MppFrame frame = nullptr;
    const int MAX_SAVE_FRAMES = 20;

    while (!s->loop_end) {
        if (s->mpi->decode_get_frame(s->ctx, &frame) != MPP_OK || !frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (mpp_frame_get_info_change(frame)) {
            uint32_t w = mpp_frame_get_width(frame);
            uint32_t h = mpp_frame_get_height(frame);

            util::console(">> Dec: InfoChange {}x{}, fmt={}\n", w, h, mpp_frame_get_fmt(frame));

            s->mpi->control(s->ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
            mpp_frame_deinit(&frame);
            continue;
        }

        if (!mpp_frame_get_errinfo(frame) && !mpp_frame_get_discard(frame)) {
            int cnt = ++s->frame_count;
            if ((cnt % 100) == 0) util::console(">> Decoded {} frames...\n", cnt);

            if (cnt <= MAX_SAVE_FRAMES) {
                save_frame_to_jpg(frame, cnt);
                util::console(">> Saved output/frame_{:04d}.jpg\n", cnt);
            }
        }

        if (mpp_frame_get_eos(frame)) {
            s->loop_end = true;
        }

        mpp_frame_deinit(&frame);
    }

    util::console(">> Output: thread done\n");
}

int main(int argc, char** argv) {
    if (argc < 2) {
        util::console("Usage: {} input_file\n", argv[0]);
        return -1;
    }
    std::string infile = argv[1];

    std::filesystem::create_directories("output");

    std::string codec = probe_codec(infile);
    if (codec.empty()) {
        util::console(">> Failed to probe codec\n");
        return -1;
    }

    Shared s;
    s.is_av1 = (codec == "av1");

    if (mpp_create(&s.ctx, &s.mpi) != MPP_OK) {
        util::console(">> mpp_create failed\n");
        return -1;
    }

    MppCodingType type = (codec == "av1") ? MPP_VIDEO_CodingAV1 : (codec == "hevc") ? MPP_VIDEO_CodingHEVC
                                                                                    : MPP_VIDEO_CodingAVC;

    if (mpp_init(s.ctx, MPP_CTX_DEC, type) != MPP_OK) {
        util::console(">> mpp_init failed\n");
        mpp_destroy(s.ctx);
        return -1;
    }

    mpp_set_log_level(MPP_LOG_ERROR);

    MppFrameFormat out_fmt = MPP_FMT_YUV420SP;
    if (s.mpi->control(s.ctx, MPP_DEC_SET_OUTPUT_FORMAT, (MppParam)&out_fmt) != MPP_OK) {
        util::console(">> Warning: Failed to set output format to 8-bit NV12\n");
    }

    uint32_t split = 1;
    s.mpi->control(s.ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, &split);
    MppPollType out_nonblock = MPP_POLL_NON_BLOCK;
    s.mpi->control(s.ctx, MPP_SET_OUTPUT_TIMEOUT, (MppParam)&out_nonblock);

    std::thread tin(input_thread, &s, infile, codec);
    std::thread tout(output_thread, &s);

    tin.join();
    tout.join();

    if (s.mpi->reset) s.mpi->reset(s.ctx);
    mpp_destroy(s.ctx);
    s.ctx = nullptr;

    util::console(">> Final Result: {} frames decoded.\n", s.frame_count.load());
    return 0;
}