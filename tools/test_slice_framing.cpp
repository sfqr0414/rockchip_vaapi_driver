#include <cstdint>
#include <cstdio>
#include <vector>

#include "util/bitstream.hpp"

namespace {
int failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

VASliceParameterBufferH264 h264_slice(uint32_t offset, uint32_t size) {
    VASliceParameterBufferH264 slice{};
    slice.slice_data_offset = offset;
    slice.slice_data_size = size;
    slice.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;
    return slice;
}

VASliceParameterBufferAV1 av1_slice(uint32_t offset, uint32_t size) {
    VASliceParameterBufferAV1 slice{};
    slice.slice_data_offset = offset;
    slice.slice_data_size = size;
    slice.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;
    return slice;
}
}  // namespace

int main() {
    // Annex-B NAL and AVCC-style length-prefixed NAL are normalized identically.
    const std::vector<std::uint8_t> annexb{0x00, 0x00, 0x01, 0x65, 0x88, 0x84};
    const std::vector<std::uint8_t> avcc{0x00, 0x00, 0x00, 0x03, 0x65, 0x88, 0x84};
    auto h264 = h264_slice(0, annexb.size());
    bool valid = false;
    auto framed = rockchip::bitstream::build_h264_annexb_slices(&h264, 1, annexb.data(), annexb.size(), &valid);
    expect(valid && framed.size() == annexb.size(), "H264 annex-B framing");

    h264 = h264_slice(0, avcc.size());
    framed = rockchip::bitstream::build_h264_annexb_slices(&h264, 1, avcc.data(), avcc.size(), &valid);
    expect(valid && framed.size() == annexb.size(), "H264 AVCC framing");
    expect(rockchip::bitstream::has_start_code(framed.data(), framed.size()), "H264 AVCC start code");

    // Out-of-bounds and overlapping slices must fail closed.
    h264 = h264_slice(4, 10);
    auto bad = rockchip::bitstream::build_h264_annexb_slices(&h264, 1, annexb.data(), annexb.size(), &valid);
    expect(!valid && bad.empty(), "H264 out-of-bounds rejected");

    VASliceParameterBufferH264 fragments[3] = {
        h264_slice(0, 5), h264_slice(5, 1), h264_slice(6, 1)};
    fragments[0].slice_data_flag = VA_SLICE_DATA_FLAG_BEGIN;
    fragments[1].slice_data_flag = VA_SLICE_DATA_FLAG_MIDDLE;
    fragments[2].slice_data_flag = VA_SLICE_DATA_FLAG_END;
    const std::vector<std::uint8_t> fragmented{0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x85};
    auto fragmented_out = rockchip::bitstream::build_h264_annexb_slices(
        fragments, 3, fragmented.data(), fragmented.size(), &valid);
    expect(valid && fragmented_out == fragmented, "H264 fragmented slice");

    VASliceParameterBufferH264 overlap[2] = {h264_slice(0, 4), h264_slice(2, 4)};
    bad = rockchip::bitstream::build_h264_annexb_slices(overlap, 2, annexb.data(), annexb.size(), &valid);
    expect(!valid && bad.empty(), "H264 overlap rejected");

    // AV1 tile data is copied verbatim, in submission order.
    const std::vector<std::uint8_t> tiles{0x11, 0x22, 0x33, 0x44, 0x55};
    VASliceParameterBufferAV1 av1[2] = {av1_slice(3, 2), av1_slice(0, 2)};
    auto av1_framed = rockchip::bitstream::build_av1_tile_slices(av1, 2, tiles.data(), tiles.size(), &valid);
    expect(valid && av1_framed == std::vector<std::uint8_t>({0x44, 0x55, 0x11, 0x22}), "AV1 tile order");

    // IDR detection ignores leading AUD/SPS and finds the first VCL NAL.
    const std::vector<std::uint8_t> aud_sps_idr{
        0x00, 0x00, 0x01, 0x09, 0xf0,
        0x00, 0x00, 0x01, 0x67, 0x64, 0x00, 0x1f,
        0x00, 0x00, 0x01, 0x65, 0x88, 0x84};
    expect(rockchip::bitstream::h264_access_unit_has_idr(aud_sps_idr), "H264 IDR after SPS");

    std::printf("slice framing checks: %s\n", failures == 0 ? "OK" : "FAILED");
    return failures == 0 ? 0 : 1;
}
