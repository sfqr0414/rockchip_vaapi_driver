#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include <va/va.h>
#include <va/va_dec_av1.h>

namespace rockchip::bitstream {

class BitWriter {
public:
    void put_bits(uint32_t value, int count) {
        for (int bit = count - 1; bit >= 0; --bit) {
            put_bit(((value >> bit) & 1U) != 0);
        }
    }

    void write_ue(uint32_t value) {
        uint32_t code_num = value + 1;
        int leading_zero_bits = 0;
        for (uint32_t tmp = code_num; tmp > 1; tmp >>= 1) {
            ++leading_zero_bits;
        }
        for (int i = 0; i < leading_zero_bits; ++i) {
            put_bit(false);
        }
        put_bits(code_num, leading_zero_bits + 1);
    }

    void write_se(int32_t value) {
        uint32_t code_num = (value <= 0)
            ? static_cast<uint32_t>(-value) * 2U
            : static_cast<uint32_t>(value) * 2U - 1U;
        write_ue(code_num);
    }

    void byte_align() {
        if (bit_count_ == 0) return;
        put_bit(true);
        while (bit_count_ != 0) {
            put_bit(false);
        }
    }

    const std::vector<uint8_t>& bytes() const { return bytes_; }

private:
    void put_bit(bool bit) {
        cur_ = static_cast<uint8_t>((cur_ << 1) | (bit ? 1 : 0));
        ++bit_count_;
        if (bit_count_ == 8) {
            bytes_.push_back(cur_);
            cur_ = 0;
            bit_count_ = 0;
        }
    }

    std::vector<uint8_t> bytes_;
    uint8_t cur_ = 0;
    int bit_count_ = 0;
};

inline void append_start_code(std::vector<uint8_t>& out) {
    out.insert(out.end(), {0x00, 0x00, 0x00, 0x01});
}

inline void append_emulation_prevention_bytes(std::vector<uint8_t>& out,
                                              const std::vector<uint8_t>& rbsp) {
    out.reserve(out.size() + rbsp.size() + 2);
    int zero_count = 0;
    for (uint8_t byte : rbsp) {
        if (zero_count >= 2 && byte <= 0x03) {
            out.push_back(0x03);
            zero_count = 0;
        }
        out.push_back(byte);
        zero_count = (byte == 0x00) ? zero_count + 1 : 0;
    }
}

inline void append_leb128(std::vector<uint8_t>& out, size_t value) {
    do {
        uint8_t byte = static_cast<uint8_t>(value & 0x7f);
        value >>= 7;
        if (value != 0) byte |= 0x80;
        out.push_back(byte);
    } while (value != 0);
}

inline std::vector<uint8_t> make_nal(const std::vector<uint8_t>& rbsp, uint8_t nal_unit_type, uint8_t nal_ref_idc = 3) {
    std::vector<uint8_t> out;
    out.reserve(rbsp.size() + 5);
    append_start_code(out);
    out.push_back(static_cast<uint8_t>((nal_ref_idc << 5) | nal_unit_type));
    append_emulation_prevention_bytes(out, rbsp);
    return out;
}

inline std::vector<uint8_t> build_h264_headers(const VAPictureParameterBufferH264& pic) {
    BitWriter sps;
    const uint8_t profile_idc = 100;

    sps.put_bits(profile_idc, 8);
    sps.put_bits(0, 8);
    sps.put_bits(42, 8);
    sps.write_ue(0);

    sps.write_ue(pic.seq_fields.bits.chroma_format_idc);
    if (pic.seq_fields.bits.chroma_format_idc == 3) {
        sps.put_bits(pic.seq_fields.bits.residual_colour_transform_flag, 1);
    }
    sps.write_ue(pic.bit_depth_luma_minus8);
    sps.write_ue(pic.bit_depth_chroma_minus8);
    sps.put_bits(0, 1);
    sps.put_bits(0, 1);
    sps.put_bits(0, 1);

    sps.write_ue(pic.seq_fields.bits.log2_max_frame_num_minus4);
    sps.write_ue(pic.seq_fields.bits.pic_order_cnt_type);
    if (pic.seq_fields.bits.pic_order_cnt_type == 0) {
        sps.write_ue(pic.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4);
    } else if (pic.seq_fields.bits.pic_order_cnt_type == 1) {
        sps.put_bits(pic.seq_fields.bits.delta_pic_order_always_zero_flag, 1);
        sps.write_se(0);
        sps.write_se(0);
        sps.write_ue(0);
    }

    sps.write_ue(pic.num_ref_frames);
    sps.put_bits(pic.seq_fields.bits.gaps_in_frame_num_value_allowed_flag, 1);
    sps.write_ue(pic.picture_width_in_mbs_minus1);
    sps.write_ue(pic.picture_height_in_mbs_minus1);
    sps.put_bits(pic.seq_fields.bits.frame_mbs_only_flag, 1);
    if (!pic.seq_fields.bits.frame_mbs_only_flag) {
        sps.put_bits(pic.seq_fields.bits.mb_adaptive_frame_field_flag, 1);
    }
    sps.put_bits(pic.seq_fields.bits.direct_8x8_inference_flag, 1);
    sps.put_bits(0, 1);
    sps.put_bits(0, 1);
    sps.byte_align();

    BitWriter pps;
    pps.write_ue(0);
    pps.write_ue(0);
    pps.put_bits(pic.pic_fields.bits.entropy_coding_mode_flag, 1);
    pps.put_bits(pic.pic_fields.bits.pic_order_present_flag, 1);
    pps.write_ue(0);
    pps.write_ue(0);
    pps.write_ue(0);
    pps.put_bits(pic.pic_fields.bits.weighted_pred_flag, 1);
    pps.put_bits(pic.pic_fields.bits.weighted_bipred_idc, 2);
    pps.write_se(pic.pic_init_qp_minus26);
    pps.write_se(pic.pic_init_qs_minus26);
    pps.write_se(pic.chroma_qp_index_offset);
    pps.put_bits(pic.pic_fields.bits.deblocking_filter_control_present_flag, 1);
    pps.put_bits(pic.pic_fields.bits.constrained_intra_pred_flag, 1);
    pps.put_bits(pic.pic_fields.bits.redundant_pic_cnt_present_flag, 1);
    pps.put_bits(pic.pic_fields.bits.transform_8x8_mode_flag, 1);
    pps.put_bits(0, 1);
    pps.write_se(pic.second_chroma_qp_index_offset);
    pps.byte_align();

    std::vector<uint8_t> out;
    auto sps_nal = make_nal(sps.bytes(), 7, 3);
    auto pps_nal = make_nal(pps.bytes(), 8, 3);
    out.reserve(sps_nal.size() + pps_nal.size());
    out.insert(out.end(), sps_nal.begin(), sps_nal.end());
    out.insert(out.end(), pps_nal.begin(), pps_nal.end());
    return out;
}

inline std::vector<uint8_t> build_h264_annexb_slices(const VASliceParameterBufferH264* slices,
                                                     uint32_t num_slices,
                                                     const uint8_t* data,
                                                     size_t data_size) {
    std::vector<uint8_t> out;
    if (!slices || !data || !data_size || !num_slices) return out;

    out.reserve(data_size + static_cast<size_t>(num_slices) * 4);
    for (uint32_t i = 0; i < num_slices; ++i) {
        const auto& slice = slices[i];
        if (slice.slice_data_offset > data_size ||
            slice.slice_data_size > data_size - slice.slice_data_offset) {
            out.clear();
            return out;
        }
        append_start_code(out);
        out.insert(out.end(), data + slice.slice_data_offset, data + slice.slice_data_offset + slice.slice_data_size);
    }
    return out;
}

inline std::vector<uint8_t> build_hevc_annexb_slices(const VASliceParameterBufferHEVC* slices,
                                                     uint32_t num_slices,
                                                     const uint8_t* data,
                                                     size_t data_size) {
    std::vector<uint8_t> out;
    if (!slices || !data || !data_size || !num_slices) return out;

    out.reserve(data_size + static_cast<size_t>(num_slices) * 4);
    for (uint32_t i = 0; i < num_slices; ++i) {
        const auto& slice = slices[i];
        if (slice.slice_data_offset > data_size ||
            slice.slice_data_size > data_size - slice.slice_data_offset) {
            out.clear();
            return out;
        }
        append_start_code(out);
        out.insert(out.end(), data + slice.slice_data_offset, data + slice.slice_data_offset + slice.slice_data_size);
    }
    return out;
}

inline std::vector<uint8_t> build_hevc_headers(const VAPictureParameterBufferHEVC& pic) {
    BitWriter vps;
    vps.put_bits(0, 4);
    vps.put_bits(0xffff, 16);
    vps.put_bits(0, 2);
    vps.put_bits(1, 6);
    vps.put_bits(0, 3);
    vps.put_bits(1, 1);
    vps.put_bits(0, 96);
    vps.byte_align();

    BitWriter sps;
    sps.put_bits(pic.pic_fields.bits.chroma_format_idc & 0x3, 2);
    sps.put_bits(pic.pic_fields.bits.separate_colour_plane_flag & 1, 1);
    sps.write_ue(pic.pic_width_in_luma_samples);
    sps.write_ue(pic.pic_height_in_luma_samples);
    sps.write_ue(pic.bit_depth_luma_minus8);
    sps.write_ue(pic.bit_depth_chroma_minus8);
    sps.byte_align();

    BitWriter pps;
    pps.write_ue(pic.init_qp_minus26);
    pps.write_se(pic.pps_cb_qp_offset);
    pps.write_se(pic.pps_cr_qp_offset);
    pps.byte_align();

    std::vector<uint8_t> out;
    auto vps_nal = make_nal(vps.bytes(), 32, 0);
    auto sps_nal = make_nal(sps.bytes(), 33, 0);
    auto pps_nal = make_nal(pps.bytes(), 34, 0);
    out.reserve(vps_nal.size() + sps_nal.size() + pps_nal.size());
    out.insert(out.end(), vps_nal.begin(), vps_nal.end());
    out.insert(out.end(), sps_nal.begin(), sps_nal.end());
    out.insert(out.end(), pps_nal.begin(), pps_nal.end());
    return out;
}

inline std::vector<uint8_t> build_av1_sequence_header(const VADecPictureParameterBufferAV1& pic) {
    BitWriter writer;
    writer.put_bits(0, 1);  // seq_profile low bit placeholder
    writer.put_bits(pic.profile & 0x7, 3);
    writer.put_bits(0, 1);  // still_picture
    writer.put_bits(0, 1);  // reduced_still_picture_header
    writer.write_ue(pic.order_hint_bits_minus_1);
    writer.put_bits(pic.seq_info_fields.fields.use_128x128_superblock, 1);
    writer.put_bits(pic.seq_info_fields.fields.enable_filter_intra, 1);
    writer.put_bits(pic.seq_info_fields.fields.enable_intra_edge_filter, 1);
    writer.put_bits(pic.seq_info_fields.fields.enable_interintra_compound, 1);
    writer.put_bits(pic.seq_info_fields.fields.enable_masked_compound, 1);
    writer.put_bits(pic.seq_info_fields.fields.enable_dual_filter, 1);
    writer.put_bits(pic.seq_info_fields.fields.enable_order_hint, 1);
    writer.put_bits(pic.seq_info_fields.fields.enable_jnt_comp, 1);
    writer.put_bits(pic.seq_info_fields.fields.mono_chrome, 1);
    writer.put_bits(pic.seq_info_fields.fields.subsampling_x, 1);
    writer.put_bits(pic.seq_info_fields.fields.subsampling_y, 1);
    writer.write_ue(pic.frame_width_minus1);
    writer.write_ue(pic.frame_height_minus1);
    writer.write_ue(pic.bit_depth_idx);
    writer.byte_align();

    std::vector<uint8_t> obu;
    obu.push_back(0x12); // sequence header OBU with size field
    append_leb128(obu, writer.bytes().size());
    obu.insert(obu.end(), writer.bytes().begin(), writer.bytes().end());

    std::vector<uint8_t> out;
    append_start_code(out);
    out.insert(out.end(), obu.begin(), obu.end());
    return out;
}

} // namespace rockchip::bitstream
