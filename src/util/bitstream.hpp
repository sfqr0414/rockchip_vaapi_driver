#pragma once

#include <algorithm>
#include <bit>
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

    void zero_align() {
        while (bit_count_ != 0) {
            put_bit(false);
        }
    }

    void rbsp_trailing() {
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

inline bool has_start_code(const uint8_t* data, size_t size) {
    if (size >= 4 && data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00 && data[3] == 0x01)
        return true;
    if (size >= 3 && data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x01)
        return true;
    return false;
}

inline void append_start_code(std::vector<uint8_t>& out) {
    out.insert(out.end(), {0x00, 0x00, 0x00, 0x01});
}

inline void append_start_code_3(std::vector<uint8_t>& out) {
    out.insert(out.end(), {0x00, 0x00, 0x01});
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

inline std::vector<uint8_t> make_hevc_nal(const std::vector<uint8_t>& rbsp, uint8_t nal_unit_type) {
    std::vector<uint8_t> out;
    out.reserve(rbsp.size() + 7);
    append_start_code(out);
    const uint8_t first_byte = static_cast<uint8_t>((nal_unit_type << 1) & 0x7e);
    const uint8_t second_byte = 0x01;
    out.push_back(first_byte);
    out.push_back(second_byte);
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
    sps.rbsp_trailing();

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
    pps.rbsp_trailing();

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
        const uint8_t* slice_ptr = data + slice.slice_data_offset;
        size_t slice_len = slice.slice_data_size;
        if (!has_start_code(slice_ptr, slice_len)) {
            append_start_code_3(out);
        }
        out.insert(out.end(), slice_ptr, slice_ptr + slice_len);
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
        const uint8_t* slice_ptr = data + slice.slice_data_offset;
        size_t slice_len = slice.slice_data_size;
        if (!has_start_code(slice_ptr, slice_len)) {
            append_start_code(out);
        }
        out.insert(out.end(), slice_ptr, slice_ptr + slice_len);
    }
    return out;
}

inline uint8_t derive_hevc_profile_idc(const VAPictureParameterBufferHEVC& pic) {
    if (pic.pic_fields.bits.chroma_format_idc == 3) return 4;
    return (pic.bit_depth_luma_minus8 > 0 || pic.bit_depth_chroma_minus8 > 0) ? 2 : 1;
}

inline uint8_t derive_hevc_level_idc(uint32_t width, uint32_t height) {
    const uint64_t pixels = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    if (pixels <= 1920ull * 1080ull) return 123;
    if (pixels <= 3840ull * 2160ull) return 150;
    if (pixels <= 7680ull * 4320ull) return 180;
    return 183;
}

inline void write_hevc_profile_tier_level(BitWriter& writer,
                                          uint8_t profile_idc,
                                          uint8_t level_idc,
                                          bool high_tier = false) {
    writer.put_bits(0, 2);
    writer.put_bits(high_tier ? 1u : 0u, 1);
    writer.put_bits(profile_idc & 0x1f, 5);
    for (int i = 0; i < 32; ++i) {
        writer.put_bits(i == profile_idc ? 1u : 0u, 1);
    }
    writer.put_bits(1, 1);
    writer.put_bits(0, 1);
    writer.put_bits(0, 1);
    writer.put_bits(1, 1);

    if (profile_idc == 2) {
        writer.put_bits(0, 7);
        writer.put_bits(0, 1);
        writer.put_bits(0, 32);
        writer.put_bits(0, 3);
    } else {
        writer.put_bits(0, 32);
        writer.put_bits(0, 11);
    }

    if (profile_idc == 1 || profile_idc == 2 || profile_idc == 3 || profile_idc == 4 ||
        profile_idc == 5 || profile_idc == 9 || profile_idc == 11) {
        writer.put_bits(0, 1);
    } else {
        writer.put_bits(0, 1);
    }

    writer.put_bits(level_idc, 8);
}

inline std::vector<uint8_t> build_hevc_headers(const VAPictureParameterBufferHEVC& pic) {
    const uint8_t profile_idc = derive_hevc_profile_idc(pic);
    const uint8_t level_idc = derive_hevc_level_idc(pic.pic_width_in_luma_samples,
                                                    pic.pic_height_in_luma_samples);

    auto build_vps = [&](void) {
        BitWriter writer;
        writer.put_bits(0, 4);  // vps_video_parameter_set_id
        writer.put_bits(1, 1);   // vps_base_layer_internal_flag
        writer.put_bits(1, 1);   // vps_base_layer_available_flag
        writer.put_bits(0, 6);   // vps_max_layers_minus1
        writer.put_bits(0, 3);   // vps_max_sub_layers_minus1
        writer.put_bits(1, 1);   // vps_temporal_id_nesting_flag
        writer.put_bits(0xffff, 16);
        write_hevc_profile_tier_level(writer, profile_idc, level_idc);
        writer.put_bits(0, 1);   // vps_sub_layer_ordering_info_present_flag

        const uint8_t max_dec_pic_buffering = pic.sps_max_dec_pic_buffering_minus1;
        const uint8_t max_num_reorder_pics = 0;
        writer.write_ue(max_dec_pic_buffering);
        writer.write_ue(max_num_reorder_pics);
        writer.write_ue(0);      // vps_max_latency_increase_plus1
        writer.put_bits(0, 6);   // vps_max_layer_id
        writer.write_ue(0);      // vps_num_layer_sets_minus1
        writer.put_bits(0, 1);   // vps_timing_info_present_flag
        writer.put_bits(0, 1);   // vps_extension_flag
        writer.byte_align();
        return make_hevc_nal(writer.bytes(), 32);
    };

    auto build_sps = [&](void) {
        BitWriter writer;
        writer.put_bits(0, 4);   // sps_video_parameter_set_id
        writer.put_bits(0, 3);   // sps_max_sub_layers_minus1
        writer.put_bits(1, 1);   // sps_temporal_id_nesting_flag
        write_hevc_profile_tier_level(writer, profile_idc, level_idc);
        writer.write_ue(0);      // sps_seq_parameter_set_id
        writer.write_ue(pic.pic_fields.bits.chroma_format_idc);
        if (pic.pic_fields.bits.chroma_format_idc == 3) {
            writer.put_bits(pic.pic_fields.bits.separate_colour_plane_flag, 1);
        }
        writer.write_ue(pic.pic_width_in_luma_samples);
        writer.write_ue(pic.pic_height_in_luma_samples);
        writer.put_bits(0, 1);   // conformance_window_flag
        writer.write_ue(pic.bit_depth_luma_minus8);
        writer.write_ue(pic.bit_depth_chroma_minus8);
        writer.write_ue(pic.log2_max_pic_order_cnt_lsb_minus4);
        writer.put_bits(0, 1);   // sps_sub_layer_ordering_info_present_flag
        writer.write_ue(pic.sps_max_dec_pic_buffering_minus1);
        writer.write_ue(0);
        writer.write_ue(0);      // sps_max_latency_increase_plus1
        writer.write_ue(pic.log2_min_luma_coding_block_size_minus3);
        writer.write_ue(pic.log2_diff_max_min_luma_coding_block_size);
        writer.write_ue(pic.log2_min_transform_block_size_minus2);
        writer.write_ue(pic.log2_diff_max_min_transform_block_size);
        writer.write_ue(pic.max_transform_hierarchy_depth_inter);
        writer.write_ue(pic.max_transform_hierarchy_depth_intra);
        writer.put_bits(pic.pic_fields.bits.scaling_list_enabled_flag, 1);
        if (pic.pic_fields.bits.scaling_list_enabled_flag) {
            writer.put_bits(0, 1);   // sps_scaling_list_data_present_flag
        }
        writer.put_bits(pic.pic_fields.bits.amp_enabled_flag, 1);
        writer.put_bits(pic.slice_parsing_fields.bits.sample_adaptive_offset_enabled_flag, 1);
        writer.put_bits(pic.pic_fields.bits.pcm_enabled_flag, 1);
        if (pic.pic_fields.bits.pcm_enabled_flag) {
            writer.put_bits(pic.pcm_sample_bit_depth_luma_minus1, 4);
            writer.put_bits(pic.pcm_sample_bit_depth_chroma_minus1, 4);
            writer.write_ue(pic.log2_min_pcm_luma_coding_block_size_minus3);
            writer.write_ue(pic.log2_diff_max_min_pcm_luma_coding_block_size);
            writer.put_bits(pic.pic_fields.bits.pcm_loop_filter_disabled_flag, 1);
        }

        const uint32_t num_short_term_ref_pic_sets = pic.num_short_term_ref_pic_sets;
        writer.write_ue(num_short_term_ref_pic_sets);
        if (num_short_term_ref_pic_sets > 0) {
            for (uint32_t set_idx = 0; set_idx < num_short_term_ref_pic_sets; ++set_idx) {
                const uint32_t num_negative_pics = (set_idx == 0)
                    ? num_short_term_ref_pic_sets
                    : set_idx;
                if (set_idx > 0) {
                    writer.put_bits(0, 1);   // inter_ref_pic_set_prediction_flag
                }
                writer.write_ue(num_negative_pics);
                writer.write_ue(0);      // num_positive_pics
                for (uint32_t i = 0; i < num_negative_pics; ++i) {
                    writer.write_ue(0);  // delta_poc_s0_minus1
                    writer.put_bits(1, 1);   // used_by_curr_pic_s0_flag
                }
            }
        }
        writer.put_bits(pic.slice_parsing_fields.bits.long_term_ref_pics_present_flag, 1);
        if (pic.slice_parsing_fields.bits.long_term_ref_pics_present_flag) {
            writer.write_ue(pic.num_long_term_ref_pic_sps);
        }
        writer.put_bits(pic.slice_parsing_fields.bits.sps_temporal_mvp_enabled_flag, 1);
        writer.put_bits(pic.pic_fields.bits.strong_intra_smoothing_enabled_flag, 1);
        writer.put_bits(0, 1);   // vui_parameters_present_flag
        writer.put_bits(0, 1);   // sps_extension_present_flag
        writer.rbsp_trailing();
        return make_hevc_nal(writer.bytes(), 33);
    };

    auto build_pps = [&](void) {
        BitWriter writer;
        writer.write_ue(0);      // pps_pic_parameter_set_id
        writer.write_ue(0);      // pps_seq_parameter_set_id
        writer.put_bits(pic.slice_parsing_fields.bits.dependent_slice_segments_enabled_flag, 1);
        writer.put_bits(pic.slice_parsing_fields.bits.output_flag_present_flag, 1);
        writer.put_bits(pic.num_extra_slice_header_bits, 3);
        writer.put_bits(pic.pic_fields.bits.sign_data_hiding_enabled_flag, 1);
        writer.put_bits(pic.slice_parsing_fields.bits.cabac_init_present_flag, 1);
        writer.write_ue(pic.num_ref_idx_l0_default_active_minus1);
        writer.write_ue(pic.num_ref_idx_l1_default_active_minus1);
        writer.write_se(pic.init_qp_minus26);
        writer.put_bits(pic.pic_fields.bits.constrained_intra_pred_flag, 1);
        writer.put_bits(pic.pic_fields.bits.transform_skip_enabled_flag, 1);
        writer.put_bits(pic.pic_fields.bits.cu_qp_delta_enabled_flag, 1);
        if (pic.pic_fields.bits.cu_qp_delta_enabled_flag) {
            writer.write_ue(pic.diff_cu_qp_delta_depth);
        }
        writer.write_se(pic.pps_cb_qp_offset);
        writer.write_se(pic.pps_cr_qp_offset);
        writer.put_bits(pic.slice_parsing_fields.bits.pps_slice_chroma_qp_offsets_present_flag, 1);
        writer.put_bits(pic.pic_fields.bits.weighted_pred_flag, 1);
        writer.put_bits(pic.pic_fields.bits.weighted_bipred_flag, 1);
        writer.put_bits(pic.pic_fields.bits.transquant_bypass_enabled_flag, 1);
        writer.put_bits(pic.pic_fields.bits.tiles_enabled_flag, 1);
        writer.put_bits(pic.pic_fields.bits.entropy_coding_sync_enabled_flag, 1);
        if (pic.pic_fields.bits.tiles_enabled_flag) {
            writer.write_ue(pic.num_tile_columns_minus1);
            writer.write_ue(pic.num_tile_rows_minus1);
            const bool uniform_spacing = (pic.num_tile_columns_minus1 == 0 && pic.num_tile_rows_minus1 == 0);
            writer.put_bits(uniform_spacing ? 1u : 0u, 1);
            if (!uniform_spacing) {
                for (uint32_t i = 0; i < pic.num_tile_columns_minus1; ++i) {
                    writer.write_ue(pic.column_width_minus1[i]);
                }
                for (uint32_t i = 0; i < pic.num_tile_rows_minus1; ++i) {
                    writer.write_ue(pic.row_height_minus1[i]);
                }
            }
            writer.put_bits(pic.pic_fields.bits.loop_filter_across_tiles_enabled_flag, 1);
        }
        writer.put_bits(pic.pic_fields.bits.pps_loop_filter_across_slices_enabled_flag, 1);
        writer.put_bits(1, 1);   // deblocking_filter_control_present_flag
        writer.put_bits(pic.slice_parsing_fields.bits.deblocking_filter_override_enabled_flag, 1);
        writer.put_bits(pic.slice_parsing_fields.bits.pps_disable_deblocking_filter_flag, 1);
        if (!pic.slice_parsing_fields.bits.pps_disable_deblocking_filter_flag) {
            writer.write_se(pic.pps_beta_offset_div2);
            writer.write_se(pic.pps_tc_offset_div2);
        }
        writer.put_bits(0, 1);   // pps_scaling_list_data_present_flag
        writer.put_bits(pic.slice_parsing_fields.bits.lists_modification_present_flag, 1);
        writer.write_ue(pic.log2_parallel_merge_level_minus2);
        writer.put_bits(pic.slice_parsing_fields.bits.slice_segment_header_extension_present_flag, 1);
        writer.put_bits(0, 1);   // pps_extension_present_flag
        writer.rbsp_trailing();
        return make_hevc_nal(writer.bytes(), 34);
    };

    std::vector<uint8_t> out;
    const auto vps = build_vps();
    const auto sps = build_sps();
    const auto pps = build_pps();
    out.insert(out.end(), vps.begin(), vps.end());
    out.insert(out.end(), sps.begin(), sps.end());
    out.insert(out.end(), pps.begin(), pps.end());
    return out;
}

inline std::vector<uint8_t> build_av1_sequence_header(const VADecPictureParameterBufferAV1& pic) {
    BitWriter payload;

    const unsigned width_bits_raw = std::bit_width(static_cast<unsigned>(pic.frame_width_minus1));
    const unsigned height_bits_raw = std::bit_width(static_cast<unsigned>(pic.frame_height_minus1));
    const unsigned width_bits = width_bits_raw == 0 ? 1u : width_bits_raw;
    const unsigned height_bits = height_bits_raw == 0 ? 1u : height_bits_raw;
    const uint8_t bit_depth = (pic.bit_depth_idx == 0) ? 8 : (pic.bit_depth_idx == 1 ? 10 : 12);
    const bool mono_chrome = pic.seq_info_fields.fields.mono_chrome != 0;
    const bool subsampling_x = (pic.profile == 0) ? true : (pic.seq_info_fields.fields.subsampling_x != 0);
    const bool subsampling_y = (pic.profile == 0) ? true : (pic.seq_info_fields.fields.subsampling_y != 0);
    const bool has_color_description = true;
    const bool enable_ref_frame_mvs = true;
    const bool enable_restoration = (pic.loop_restoration_fields.bits.yframe_restoration_type != 0) ||
                                    (pic.loop_restoration_fields.bits.cbframe_restoration_type != 0) ||
                                    (pic.loop_restoration_fields.bits.crframe_restoration_type != 0);

    payload.put_bits(pic.profile & 0x7, 3);
    payload.put_bits(0, 1);  // still_picture
    payload.put_bits(0, 1);  // reduced_still_picture_header
    payload.put_bits(0, 1);  // timing_info_present_flag
    payload.put_bits(0, 1);  // initial_display_delay_present_flag
    payload.put_bits(0, 5);  // operating_points_cnt_minus_1
    payload.put_bits(0, 12); // operating_point_idc[0]
    payload.put_bits(9, 5);  // seq_level_idx[0]
    payload.put_bits(0, 1);  // seq_tier[0]
    payload.put_bits(static_cast<uint32_t>(width_bits - 1), 4);
    payload.put_bits(static_cast<uint32_t>(height_bits - 1), 4);
    payload.put_bits(pic.frame_width_minus1, static_cast<int>(width_bits));
    payload.put_bits(pic.frame_height_minus1, static_cast<int>(height_bits));
    payload.put_bits(0, 1);  // frame_id_numbers_present_flag
    payload.put_bits(pic.seq_info_fields.fields.use_128x128_superblock ? 1u : 0u, 1);
    payload.put_bits(pic.seq_info_fields.fields.enable_filter_intra ? 1u : 0u, 1);
    payload.put_bits(pic.seq_info_fields.fields.enable_intra_edge_filter ? 1u : 0u, 1);
    payload.put_bits(pic.seq_info_fields.fields.enable_interintra_compound ? 1u : 0u, 1);
    payload.put_bits(pic.seq_info_fields.fields.enable_masked_compound ? 1u : 0u, 1);
    payload.put_bits(0, 1);
    payload.put_bits(pic.seq_info_fields.fields.enable_dual_filter ? 1u : 0u, 1);
    payload.put_bits(pic.seq_info_fields.fields.enable_order_hint, 1);
    if (pic.seq_info_fields.fields.enable_order_hint) {
        payload.put_bits(pic.seq_info_fields.fields.enable_jnt_comp, 1);
        payload.put_bits(enable_ref_frame_mvs ? 1u : 0u, 1);
    }

    const bool seq_choose_screen_content_tools = true;
    payload.put_bits(seq_choose_screen_content_tools ? 1u : 0u, 1);  // seq_choose_screen_content_tools
    if (!seq_choose_screen_content_tools) {
        payload.put_bits(0, 1);  // seq_force_screen_content_tools
    } else {
        const bool seq_choose_integer_mv = true;
        payload.put_bits(seq_choose_integer_mv ? 1u : 0u, 1);  // seq_choose_integer_mv
        if (!seq_choose_integer_mv) {
            payload.put_bits(pic.pic_info_fields.bits.force_integer_mv ? 1u : 0u, 1);  // seq_force_integer_mv
        }
    }
    if (pic.seq_info_fields.fields.enable_order_hint) {
        payload.put_bits(pic.order_hint_bits_minus_1 & 0x7, 3);
    }

    payload.put_bits(pic.pic_info_fields.bits.use_superres ? 1u : 0u, 1);
    payload.put_bits(pic.seq_info_fields.fields.enable_cdef, 1);
    payload.put_bits(enable_restoration ? 1u : 0u, 1);

    if (bit_depth > 8) {
        payload.put_bits(1, 1);  // high_bitdepth
        if (pic.profile == 2) {
            payload.put_bits(bit_depth == 12 ? 1u : 0u, 1);  // twelve_bit
        }
    } else {
        payload.put_bits(0, 1);  // high_bitdepth
        if (pic.profile == 2) {
            payload.put_bits(0, 1);  // twelve_bit
        }
    }

    if (pic.profile != 1) {
        payload.put_bits(mono_chrome ? 1u : 0u, 1);
    }

    payload.put_bits(has_color_description ? 1u : 0u, 1);
    if (has_color_description) {
        payload.put_bits(1, 8);  // color_primaries = bt709
        payload.put_bits(2, 8);  // transfer_characteristics = unspecified
        payload.put_bits(pic.matrix_coefficients, 8);
    }

    payload.put_bits(pic.seq_info_fields.fields.color_range, 1);
    if (!mono_chrome && subsampling_x && subsampling_y) {
        payload.put_bits(pic.seq_info_fields.fields.chroma_sample_position & 0x3, 2);
    }
    payload.put_bits(0, 1);  // separate_uv_delta_q
    payload.put_bits(pic.seq_info_fields.fields.film_grain_params_present, 1);
    payload.byte_align();

    BitWriter obu_header;
    obu_header.put_bits(0, 1);  // obu_forbidden_bit
    obu_header.put_bits(1, 4);  // OBU_SEQUENCE_HEADER
    obu_header.put_bits(0, 1);  // obu_extension_flag
    obu_header.put_bits(1, 1);  // obu_has_size_field
    obu_header.put_bits(0, 1);  // obu_reserved_1bit

    std::vector<uint8_t> obu;
    const auto& header_bytes = obu_header.bytes();
    obu.insert(obu.end(), header_bytes.begin(), header_bytes.end());
    append_leb128(obu, payload.bytes().size());
    obu.insert(obu.end(), payload.bytes().begin(), payload.bytes().end());
    return obu;
}

inline void write_av1_frame_size(BitWriter& writer, const VADecPictureParameterBufferAV1& pic) {
    writer.put_bits(pic.frame_width_minus1, 11);
    writer.put_bits(pic.frame_height_minus1, 11);
}

inline std::vector<uint8_t> build_av1_frame_obu(const VADecPictureParameterBufferAV1& pic,
                                                std::span<const uint8_t> tile_payload) {
    BitWriter payload;

    auto put_su_bits = [&](int32_t value, int count) {
        const uint32_t mask = (count >= 32) ? 0xffffffffu : ((1u << count) - 1u);
        payload.put_bits(static_cast<uint32_t>(value) & mask, count);
    };

    auto write_delta_q = [&](int32_t delta_q) {
        payload.put_bits(delta_q != 0 ? 1u : 0u, 1);
        if (delta_q != 0) {
            put_su_bits(delta_q, 7);
        }
    };

    const bool frame_is_intra = (pic.pic_info_fields.bits.frame_type == 0 || pic.pic_info_fields.bits.frame_type == 2);
    const bool show_frame = pic.pic_info_fields.bits.show_frame != 0;
    const bool showable_frame = pic.pic_info_fields.bits.showable_frame != 0;
    const bool error_resilient_mode = pic.pic_info_fields.bits.error_resilient_mode != 0;
    const bool allow_screen_content_tools = pic.pic_info_fields.bits.allow_screen_content_tools != 0;
    const bool force_integer_mv = pic.pic_info_fields.bits.force_integer_mv != 0;
    const bool allow_intrabc = pic.pic_info_fields.bits.allow_intrabc != 0;
    const bool allow_high_precision_mv = pic.pic_info_fields.bits.allow_high_precision_mv != 0;
    const bool is_motion_mode_switchable = pic.pic_info_fields.bits.is_motion_mode_switchable != 0;
    const bool use_ref_frame_mvs = pic.pic_info_fields.bits.use_ref_frame_mvs != 0;
    const bool uniform_tile_spacing_flag = pic.pic_info_fields.bits.uniform_tile_spacing_flag != 0;
    const bool allow_warped_motion = pic.pic_info_fields.bits.allow_warped_motion != 0;
    const bool enable_restoration = (pic.loop_restoration_fields.bits.yframe_restoration_type != 0) ||
                                    (pic.loop_restoration_fields.bits.cbframe_restoration_type != 0) ||
                                    (pic.loop_restoration_fields.bits.crframe_restoration_type != 0);
    const uint8_t frame_type = static_cast<uint8_t>(pic.pic_info_fields.bits.frame_type & 0x3);
    const uint8_t order_hint_bits = static_cast<uint8_t>(pic.order_hint_bits_minus_1 + 1);
    const bool is_switch_frame = frame_type == 3;
    const bool is_key_show_frame = frame_type == 0 && show_frame;
    const uint8_t refresh_frame_flags = frame_is_intra ? 0xff : 0x00;

    payload.put_bits(0, 1);  // show_existing_frame
    payload.put_bits(frame_type, 2);
    payload.put_bits(show_frame ? 1u : 0u, 1);
    if (!show_frame) {
        payload.put_bits(showable_frame ? 1u : 0u, 1);
    }

    if (!(is_switch_frame || is_key_show_frame)) {
        payload.put_bits(error_resilient_mode ? 1u : 0u, 1);
    }

    payload.put_bits(pic.pic_info_fields.bits.disable_cdf_update ? 1u : 0u, 1);

    payload.put_bits(allow_screen_content_tools ? 1u : 0u, 1);
    if (allow_screen_content_tools) {
        payload.put_bits(force_integer_mv ? 1u : 0u, 1);
    }

    if (!is_switch_frame) {
        payload.put_bits(0, 1);  // frame_size_override_flag
    }

    if (pic.seq_info_fields.fields.enable_order_hint) {
        payload.put_bits(pic.order_hint, order_hint_bits);
    }

    if (!(frame_is_intra || error_resilient_mode)) {
        payload.put_bits(pic.primary_ref_frame & 0x7, 3);
    }

    if (!(is_switch_frame || is_key_show_frame)) {
        payload.put_bits(refresh_frame_flags, 8);
    }

    if (frame_is_intra) {
        write_av1_frame_size(payload, pic);
        payload.put_bits(0, 1);  // render_and_frame_size_different
        if (allow_screen_content_tools) {
            payload.put_bits(pic.pic_info_fields.bits.allow_intrabc ? 1u : 0u, 1);
        }
    } else {
        if (pic.seq_info_fields.fields.enable_order_hint) {
            payload.put_bits(0, 1);  // frame_refs_short_signaling
        }

        for (uint32_t i = 0; i < 7; ++i) {
            payload.put_bits(pic.ref_frame_idx[i] & 0x7, 3);
        }

        write_av1_frame_size(payload, pic);
        payload.put_bits(0, 1);  // render_and_frame_size_different

        if (!force_integer_mv) {
            payload.put_bits(allow_high_precision_mv ? 1u : 0u, 1);
        }

        const bool is_filter_switchable = (pic.interp_filter == 4);
        payload.put_bits(is_filter_switchable ? 1u : 0u, 1);
        if (!is_filter_switchable) {
            payload.put_bits(pic.interp_filter & 0x3, 2);
        }

        payload.put_bits(is_motion_mode_switchable ? 1u : 0u, 1);
        if (!(error_resilient_mode || !use_ref_frame_mvs)) {
            payload.put_bits(use_ref_frame_mvs ? 1u : 0u, 1);
        }
    }

    if (!pic.pic_info_fields.bits.disable_cdf_update) {
        payload.put_bits(pic.pic_info_fields.bits.disable_frame_end_update_cdf ? 1u : 0u, 1);
    }

    payload.put_bits(uniform_tile_spacing_flag ? 1u : 0u, 1);
    if (uniform_tile_spacing_flag) {
        const uint32_t frame_width = static_cast<uint32_t>(pic.frame_width_minus1) + 1;
        const uint32_t frame_height = static_cast<uint32_t>(pic.frame_height_minus1) + 1;
        const uint32_t mi_cols = 2u * (((frame_width - 1u) + 8u) >> 3);
        const uint32_t mi_rows = 2u * (((frame_height - 1u) + 8u) >> 3);
        const uint32_t sb_shift = pic.seq_info_fields.fields.use_128x128_superblock ? 7u : 6u;
        const uint32_t frame_width_sb = (mi_cols + ((1u << (sb_shift - 2u)) - 1u)) >> (sb_shift - 2u);
        const uint32_t frame_height_sb = (mi_rows + ((1u << (sb_shift - 2u)) - 1u)) >> (sb_shift - 2u);
        const uint32_t col_count = std::max<uint32_t>(1u, pic.tile_cols ? pic.tile_cols : 1u);
        const uint32_t row_count = std::max<uint32_t>(1u, pic.tile_rows ? pic.tile_rows : 1u);
        const uint32_t min_log2_cols = 0;
        const uint32_t max_log2_cols = 0;
        const uint32_t min_log2_rows = 0;
        const uint32_t max_log2_rows = 0;
        (void)frame_width_sb;
        (void)frame_height_sb;
        for (uint32_t i = min_log2_cols; i < max_log2_cols; ++i) {
            payload.put_bits(1, 1);
        }
        if (col_count > 1) {
            payload.put_bits(0, 1);
        }
        for (uint32_t i = min_log2_rows; i < max_log2_rows; ++i) {
            payload.put_bits(1, 1);
        }
        if (row_count > 1) {
            payload.put_bits(0, 1);
        }
    }

    payload.put_bits(pic.base_qindex, 8);
    write_delta_q(pic.y_dc_delta_q);
    payload.put_bits(pic.qmatrix_fields.bits.using_qmatrix ? 1u : 0u, 1);
    if (pic.qmatrix_fields.bits.using_qmatrix) {
        payload.put_bits(pic.qmatrix_fields.bits.qm_y & 0xf, 4);
        payload.put_bits(pic.qmatrix_fields.bits.qm_u & 0xf, 4);
        payload.put_bits(pic.qmatrix_fields.bits.qm_v & 0xf, 4);
    }

    payload.put_bits(pic.seg_info.segment_info_fields.bits.enabled ? 1u : 0u, 1);
    if (pic.seg_info.segment_info_fields.bits.enabled) {
        payload.put_bits(pic.seg_info.segment_info_fields.bits.update_map ? 1u : 0u, 1);
        if (pic.seg_info.segment_info_fields.bits.update_map) {
            payload.put_bits(pic.seg_info.segment_info_fields.bits.temporal_update ? 1u : 0u, 1);
        }
        payload.put_bits(pic.seg_info.segment_info_fields.bits.update_data ? 1u : 0u, 1);
    }

    if (pic.base_qindex) {
        payload.put_bits(pic.mode_control_fields.bits.delta_q_present_flag ? 1u : 0u, 1);
    }
    if (pic.mode_control_fields.bits.delta_q_present_flag) {
        payload.put_bits(pic.mode_control_fields.bits.log2_delta_q_res & 0x3, 2);
        if (!allow_intrabc) {
            payload.put_bits(pic.mode_control_fields.bits.delta_lf_present_flag ? 1u : 0u, 1);
            if (pic.mode_control_fields.bits.delta_lf_present_flag) {
                payload.put_bits(pic.mode_control_fields.bits.log2_delta_lf_res & 0x3, 2);
                payload.put_bits(pic.mode_control_fields.bits.delta_lf_multi ? 1u : 0u, 1);
            }
        }
    }

    if (!allow_intrabc) {
        payload.put_bits(pic.filter_level[0] & 0x3f, 6);
        payload.put_bits(pic.filter_level[1] & 0x3f, 6);
        if (pic.filter_level[0] || pic.filter_level[1]) {
            payload.put_bits(pic.filter_level_u & 0x3f, 6);
            payload.put_bits(pic.filter_level_v & 0x3f, 6);
        }
        payload.put_bits(pic.loop_filter_info_fields.bits.sharpness_level & 0x7, 3);
        payload.put_bits(pic.loop_filter_info_fields.bits.mode_ref_delta_enabled ? 1u : 0u, 1);
        if (pic.loop_filter_info_fields.bits.mode_ref_delta_enabled) {
            payload.put_bits(pic.loop_filter_info_fields.bits.mode_ref_delta_update ? 1u : 0u, 1);
            if (pic.loop_filter_info_fields.bits.mode_ref_delta_update) {
                for (uint32_t i = 0; i < 8; ++i) {
                    const int32_t ref_delta = pic.ref_deltas[i];
                    const bool update_ref_delta = ref_delta != 0;
                    payload.put_bits(update_ref_delta ? 1u : 0u, 1);
                    if (update_ref_delta) {
                        put_su_bits(ref_delta, 7);
                    }
                }
                for (uint32_t i = 0; i < 2; ++i) {
                    const int32_t mode_delta = pic.mode_deltas[i];
                    const bool update_mode_delta = mode_delta != 0;
                    payload.put_bits(update_mode_delta ? 1u : 0u, 1);
                    if (update_mode_delta) {
                        put_su_bits(mode_delta, 7);
                    }
                }
            }
        }
    }

    if (pic.seq_info_fields.fields.enable_cdef && !allow_intrabc) {
        payload.put_bits(pic.cdef_damping_minus_3 & 0x3, 2);
        payload.put_bits(pic.cdef_bits & 0x3, 2);
        const uint32_t cdef_entries = 1u << (pic.cdef_bits & 0x3);
        for (uint32_t i = 0; i < cdef_entries; ++i) {
            payload.put_bits(pic.cdef_y_strengths[i] >> 2, 4);
            payload.put_bits(pic.cdef_y_strengths[i] & 0x3, 2);
            payload.put_bits(pic.cdef_uv_strengths[i] >> 2, 4);
            payload.put_bits(pic.cdef_uv_strengths[i] & 0x3, 2);
        }
    }

    if (enable_restoration && !allow_intrabc) {
        payload.put_bits(pic.loop_restoration_fields.bits.yframe_restoration_type & 0x3, 2);
        payload.put_bits(pic.loop_restoration_fields.bits.cbframe_restoration_type & 0x3, 2);
        payload.put_bits(pic.loop_restoration_fields.bits.crframe_restoration_type & 0x3, 2);
        payload.put_bits(pic.loop_restoration_fields.bits.lr_unit_shift & 0x3, 2);
        payload.put_bits(pic.loop_restoration_fields.bits.lr_uv_shift ? 1u : 0u, 1);
    }

    if (!allow_intrabc) {
        payload.put_bits(pic.mode_control_fields.bits.tx_mode == 2 ? 1u : 0u, 1);
    }

    if (!frame_is_intra) {
        payload.put_bits(pic.mode_control_fields.bits.reference_select ? 1u : 0u, 1);

        const bool skip_mode_allowed = pic.seq_info_fields.fields.enable_order_hint &&
                                       pic.mode_control_fields.bits.reference_select &&
                                       !error_resilient_mode;
        if (skip_mode_allowed) {
            payload.put_bits(pic.mode_control_fields.bits.skip_mode_present ? 1u : 0u, 1);
        }

        if (!error_resilient_mode) {
            payload.put_bits(pic.pic_info_fields.bits.allow_warped_motion ? 1u : 0u, 1);
        }

        for (uint32_t i = 0; i < 7; ++i) {
            payload.put_bits(0, 1);
        }

    }

    payload.put_bits(pic.mode_control_fields.bits.reduced_tx_set_used ? 1u : 0u, 1);

    payload.put_bits(0, 1);  // film_grain_params_present default 0
    payload.zero_align();

    std::vector<uint8_t> obu;
    BitWriter obu_header;
    obu_header.put_bits(0, 1);
    obu_header.put_bits(6, 4);
    obu_header.put_bits(0, 1);
    obu_header.put_bits(1, 1);
    obu_header.put_bits(0, 1);

    const auto& outer_header_bytes = obu_header.bytes();
    obu.insert(obu.end(), outer_header_bytes.begin(), outer_header_bytes.end());
    append_leb128(obu, payload.bytes().size() + tile_payload.size());
    obu.insert(obu.end(), payload.bytes().begin(), payload.bytes().end());
    return obu;
}

} // namespace rockchip::bitstream
