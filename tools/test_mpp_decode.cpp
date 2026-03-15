#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <rockchip/rk_mpi.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>

static bool readFile(const char* path, std::vector<uint8_t>& out) {
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

int main(int argc, char** argv) {
    const char* inPath = "./videos/test.h264";
    const char* codecStr = "h264";
    if (argc > 1) inPath = argv[1];
    if (argc > 2) codecStr = argv[2];

    std::vector<uint8_t> bitstream;
    if (!readFile(inPath, bitstream)) {
        fprintf(stderr, "Failed to read input file: %s\n", inPath);
        return 1;
    }

    MppCodingType codec = MPP_VIDEO_CodingAVC;
    if (strcmp(codecStr, "av1") == 0) {
        codec = MPP_VIDEO_CodingAV1;
    } else if (strcmp(codecStr, "hevc") == 0) {
        codec = MPP_VIDEO_CodingHEVC;
    } else if (strcmp(codecStr, "vp9") == 0) {
        codec = MPP_VIDEO_CodingVP9;
    }

    fprintf(stderr, "Using codec %s\n", codecStr);

    MppCtx ctx;
    MppApi* api = nullptr;
    if (mpp_create(&ctx, &api) != MPP_OK) {
        fprintf(stderr, "mpp_create failed\n");
        return 1;
    }

    if (mpp_init(ctx, MPP_CTX_DEC, codec) != MPP_OK) {
        fprintf(stderr, "mpp_init failed (codec=%s)\n", codecStr);
        mpp_destroy(ctx);
        return 1;
    }

    // Feed the input bitstream to MPP on a per-NAL basis, and drain all output frames.
    const uint8_t* data = bitstream.data();
    size_t dataSize = bitstream.size();

    auto findStart = [&](size_t pos) -> size_t {
        for (size_t i = pos; i + 3 < dataSize; ++i) {
            if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
                return i;
            }
            if (i + 4 < dataSize && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i + 3] == 1) {
                return i;
            }
        }
        return dataSize;
    };

    size_t nalStart = findStart(0);
    if (nalStart == dataSize) {
        nalStart = 0;
    }

    bool gotFrame = false;

    while (nalStart < dataSize) {
        size_t nalEnd = findStart(nalStart + 3);
        if (nalEnd == dataSize) {
            // Last NAL unit; mark EOS.
        }
        size_t nalSize = nalEnd - nalStart;

        MppPacket packet = nullptr;
        if (mpp_packet_init(&packet, (void*)(data + nalStart), nalSize) != MPP_OK) {
            fprintf(stderr, "mpp_packet_init failed\n");
            break;
        }

        if (nalEnd == dataSize) {
            mpp_packet_set_pts(packet, 0);
            mpp_packet_set_eos(packet);
        }

        int ret = api->decode_put_packet(ctx, packet);
        fprintf(stderr, "decode_put_packet returned %d (nalStart=%zu nalSize=%zu)\n", ret, nalStart, nalSize);
        mpp_packet_deinit(&packet);
        if (ret != MPP_OK) {
            break;
        }

        // Drain any available output frames.
        while (true) {
            MppFrame frame = nullptr;
            if (mpp_frame_init(&frame) != MPP_OK) {
                fprintf(stderr, "mpp_frame_init failed\n");
                break;
            }

            ret = api->decode_get_frame(ctx, &frame);
            if (ret == MPP_ERR_DISPLAY_FULL) {
                mpp_frame_deinit(&frame);
                break;
            }

            if (ret != MPP_OK || !frame) {
                fprintf(stderr, "decode_get_frame returned %d\n", ret);
                if (frame) mpp_frame_deinit(&frame);
                break;
            }

            MppBuffer outBuf = mpp_frame_get_buffer(frame);
            int outFd = mpp_buffer_get_fd(outBuf);
            fprintf(stderr, "decoded output dmabuf fd=%d\n", outFd);

            gotFrame = true;
            mpp_frame_deinit(&frame);
        }

        nalStart = nalEnd;
    }

    if (!gotFrame) {
        fprintf(stderr, "No decoded frames produced\n");
    }

    if (!gotFrame) {
        fprintf(stderr, "No decoded frames produced\n");
    }

    mpp_destroy(ctx);

    return 0;
}
