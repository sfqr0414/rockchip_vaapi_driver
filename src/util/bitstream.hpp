#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <ranges>
#include <span>
#include <type_traits>
#include <tuple>
#include <vector>

#include <va/va.h>
#include <va/va_dec_av1.h>

namespace rockchip::bitstream {

namespace detail {

template <std::size_t Width, std::unsigned_integral UInt>
struct FixedBits {
    UInt value;
};

struct SizedBits {
    std::uint64_t value;
    int width;
};

template <std::unsigned_integral UInt>
struct ExpGolomb {
    UInt value;
};

template <std::signed_integral Int>
struct SignedExpGolomb {
    Int value;
};

template <typename... Fields>
struct Sequence {
    std::tuple<Fields...> fields;
};

template <typename... Fields>
struct Conditional {
    bool enabled;
    Sequence<Fields...> body;
};

template <std::size_t Width, typename Int>
constexpr auto bits(Int value) noexcept {
    using Raw = std::remove_cv_t<Int>;
    using Unsigned = std::conditional_t<std::is_signed_v<Raw>, std::make_unsigned_t<Raw>, Raw>;
    return FixedBits<Width, Unsigned>{static_cast<Unsigned>(value)};
}

template <typename Int>
constexpr auto bits(Int value, int width) noexcept {
    using Raw = std::remove_cv_t<Int>;
    using Unsigned = std::conditional_t<std::is_signed_v<Raw>, std::make_unsigned_t<Raw>, Raw>;
    return SizedBits{static_cast<std::uint64_t>(static_cast<Unsigned>(value)), width};
}

template <typename Int>
constexpr auto ue(Int value) noexcept {
    using Raw = std::remove_cv_t<Int>;
    using Unsigned = std::conditional_t<std::is_signed_v<Raw>, std::make_unsigned_t<Raw>, Raw>;
    return ExpGolomb<Unsigned>{static_cast<Unsigned>(value)};
}

template <std::signed_integral Int>
constexpr auto se(Int value) noexcept {
    return SignedExpGolomb<Int>{value};
}

template <typename... Fields>
constexpr auto seq(Fields... fields) noexcept {
    return Sequence<Fields...>{std::tuple{fields...}};
}

template <typename... Fields>
constexpr auto when(bool enabled, Fields... fields) noexcept {
    return Conditional<Fields...>{enabled, seq(fields...)};
}

template <std::unsigned_integral UInt>
constexpr UInt byteswap(UInt value) noexcept {
    if constexpr (sizeof(UInt) == 1) {
        return value;
    } else if constexpr (sizeof(UInt) == 2) {
        return static_cast<UInt>(__builtin_bswap16(static_cast<std::uint16_t>(value)));
    } else if constexpr (sizeof(UInt) == 4) {
        return static_cast<UInt>(__builtin_bswap32(static_cast<std::uint32_t>(value)));
    } else if constexpr (sizeof(UInt) == 8) {
        return static_cast<UInt>(__builtin_bswap64(static_cast<std::uint64_t>(value)));
    } else {
        static_assert(sizeof(UInt) <= 8, "unsupported integer width");
        return value;
    }
}

template <std::unsigned_integral UInt>
constexpr UInt to_network_order(UInt value) noexcept {
    if constexpr (std::endian::native == std::endian::little) {
        return byteswap(value);
    } else {
        return value;
    }
}

template <typename Int>
constexpr auto to_unsigned_bits(Int value) noexcept {
    using Unsigned = std::make_unsigned_t<std::remove_cv_t<Int>>;
    return static_cast<Unsigned>(value);
}

} // namespace detail

class Serializer {
public:
    using Buffer = std::vector<std::byte>;

    void put_bits(std::uint64_t value, int count) {
        static_assert(std::has_single_bit(static_cast<unsigned>(8)), "serializer expects 8-bit bytes");
        if (count <= 0) {
            return;
        }

        const std::uint64_t mask = (count >= 64) ? std::numeric_limits<std::uint64_t>::max()
                                                 : ((std::uint64_t{1} << count) - 1u);
        const std::uint64_t aligned = (value & mask) << (64 - count);
        const std::uint64_t network = detail::to_network_order(aligned);

        const auto octets = std::bit_cast<std::array<std::byte, sizeof(network)>>(network);
        for (int bit_index = 0; bit_index < count; ++bit_index) {
            const int absolute_bit = bit_index;
            const auto octet = std::to_integer<std::uint8_t>(octets[absolute_bit / 8]);
            const int bit_in_octet = 7 - (absolute_bit % 8);
            put_bit(((octet >> bit_in_octet) & 1u) != 0);
        }
    }

    void write_ue(std::uint32_t value) {
        const std::uint32_t code_num = value + 1u;
        const int leading_zero_bits = 31 - static_cast<int>(std::countl_zero(code_num));
        for (int bit = 0; bit < leading_zero_bits; ++bit) {
            put_bit(false);
        }
        put_bits(code_num, leading_zero_bits + 1);
    }

    void write_se(std::int32_t value) {
        const auto magnitude = static_cast<std::uint32_t>(value < 0 ? -static_cast<std::int64_t>(value)
                                                                    : static_cast<std::int64_t>(value));
        const std::uint32_t code_num = (value <= 0) ? (magnitude << 1) : ((magnitude << 1) - 1u);
        write_ue(code_num);
    }

    void put_ue(std::uint32_t value) { write_ue(value); }
    void put_se(std::int32_t value) { write_se(value); }

    void byte_align(std::uint8_t fill_bit = 1) {
        if (bit_count_ == 0) {
            return;
        }
        put_bit(fill_bit != 0);
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

    template <std::size_t Width, std::unsigned_integral UInt>
    void write(const detail::FixedBits<Width, UInt>& field) {
        put_bits(field.value, static_cast<int>(Width));
    }

    void write(const detail::SizedBits& field) {
        put_bits(field.value, field.width);
    }

    template <std::unsigned_integral UInt>
    void write(const detail::ExpGolomb<UInt>& field) {
        write_ue(static_cast<std::uint32_t>(field.value));
    }

    template <std::signed_integral Int>
    void write(const detail::SignedExpGolomb<Int>& field) {
        write_se(static_cast<std::int32_t>(field.value));
    }

    template <typename... Fields>
    void write(const detail::Sequence<Fields...>& sequence) {
        std::apply([this](const auto&... field) { (write(field), ...); }, sequence.fields);
    }

    template <typename... Fields>
    void write(const detail::Conditional<Fields...>& conditional) {
        if (conditional.enabled) {
            write(conditional.body);
        }
    }

    template <typename Syntax>
    requires requires(const Syntax& syntax, Serializer& serializer) { syntax.serialize(serializer); }
    void write(const Syntax& syntax) {
        syntax.serialize(*this);
    }

    const Buffer& bytes() const noexcept { return bytes_; }

    std::vector<std::uint8_t> bytes_u8() const {
        std::vector<std::uint8_t> out;
        out.reserve(bytes_.size());
        std::ranges::transform(bytes_, std::back_inserter(out), [](std::byte byte) {
            return std::to_integer<std::uint8_t>(byte);
        });
        return out;
    }

private:
    void put_bit(bool bit) {
        cur_ = static_cast<std::uint8_t>((cur_ << 1) | (bit ? 1u : 0u));
        ++bit_count_;
        if (bit_count_ == 8) {
            bytes_.push_back(static_cast<std::byte>(cur_));
            cur_ = 0;
            bit_count_ = 0;
        }
    }

    Buffer bytes_;
    std::uint8_t cur_ = 0;
    int bit_count_ = 0;
};

using BitWriter = Serializer;

namespace syntax {

using detail::bits;
using detail::se;
using detail::seq;
using detail::ue;
using detail::when;

struct H264SpsSyntax {
    std::uint32_t profile_idc : 8 = 100; // profile_idc: identifies the H.264 High profile.
    std::uint32_t constraint_flags : 8 = 0; // constraint_set0_flag..reserved_zero_2bits: profile constraints and reserved bits.
    std::uint32_t level_idc : 8 = 42; // level_idc: decoder capability level.
    std::uint32_t seq_parameter_set_id = 0; // seq_parameter_set_id: Exp-Golomb SPS identifier.
    std::uint32_t chroma_format_idc = 1; // chroma_format_idc: chroma sampling format.
    std::uint32_t residual_colour_transform_flag : 1 = 0; // separate_colour_plane_flag in older VA naming.
    std::uint32_t bit_depth_luma_minus8 = 0; // bit_depth_luma_minus8: luma bit depth minus 8.
    std::uint32_t bit_depth_chroma_minus8 = 0; // bit_depth_chroma_minus8: chroma bit depth minus 8.
    std::uint32_t qpprime_y_zero_transform_bypass_flag : 1 = 0; // qpprime_y_zero_transform_bypass_flag: transform bypass enable.
    std::uint32_t seq_scaling_matrix_present_flag : 1 = 0; // seq_scaling_matrix_present_flag: scaling matrix presence.
    std::uint32_t log2_max_frame_num_minus4 = 0; // log2_max_frame_num_minus4: frame_num width minus 4.
    std::uint32_t pic_order_cnt_type = 0; // pic_order_cnt_type: POC coding mode.
    std::uint32_t log2_max_pic_order_cnt_lsb_minus4 = 0; // log2_max_pic_order_cnt_lsb_minus4: POC LSB width minus 4.
    std::uint32_t delta_pic_order_always_zero_flag : 1 = 0; // delta_pic_order_always_zero_flag: POC delta suppression.
    std::uint32_t num_ref_frames = 0; // num_ref_frames: DPB reference frame count.
    std::uint32_t gaps_in_frame_num_value_allowed_flag : 1 = 0; // gaps_in_frame_num_value_allowed_flag: gap tolerance.
    std::uint32_t picture_width_in_mbs_minus1 = 0; // pic_width_in_mbs_minus1: coded width in macroblocks minus 1.
    std::uint32_t picture_height_in_map_units_minus1 = 0; // pic_height_in_map_units_minus1: coded height in map units minus 1.
    std::uint32_t frame_mbs_only_flag : 1 = 1; // frame_mbs_only_flag: frame-only coding.
    std::uint32_t mb_adaptive_frame_field_flag : 1 = 0; // mb_adaptive_frame_field_flag: MBAFF enable.
    std::uint32_t direct_8x8_inference_flag : 1 = 1; // direct_8x8_inference_flag: direct mode inference size.
    std::uint32_t frame_cropping_flag : 1 = 0; // frame_cropping_flag: cropping metadata presence.
    std::uint32_t vui_parameters_present_flag : 1 = 0; // vui_parameters_present_flag: VUI presence.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            bits<8>(profile_idc),
            bits<8>(constraint_flags),
            bits<8>(level_idc),
            ue(seq_parameter_set_id),
            ue(chroma_format_idc)));
        serializer.write(when(chroma_format_idc == 3, bits<1>(residual_colour_transform_flag)));
        serializer.write(seq(
            ue(bit_depth_luma_minus8),
            ue(bit_depth_chroma_minus8),
            bits<1>(qpprime_y_zero_transform_bypass_flag),
            bits<1>(seq_scaling_matrix_present_flag),
            ue(log2_max_frame_num_minus4),
            ue(pic_order_cnt_type)));
        if (pic_order_cnt_type == 0) {
            serializer.write(ue(log2_max_pic_order_cnt_lsb_minus4));
        } else if (pic_order_cnt_type == 1) {
            serializer.write(seq(
                bits<1>(delta_pic_order_always_zero_flag),
                se(0),
                se(0),
                ue(0)));
        }
        serializer.write(seq(
            ue(num_ref_frames),
            bits<1>(gaps_in_frame_num_value_allowed_flag),
            ue(picture_width_in_mbs_minus1),
            ue(picture_height_in_map_units_minus1),
            bits<1>(frame_mbs_only_flag)));
        if (!frame_mbs_only_flag) {
            serializer.write(bits<1>(mb_adaptive_frame_field_flag));
        }
        serializer.write(seq(
            bits<1>(direct_8x8_inference_flag),
            bits<1>(frame_cropping_flag),
            bits<1>(vui_parameters_present_flag)));
    }
};

struct H264PpsSyntax {
    std::uint32_t pic_parameter_set_id = 0; // pic_parameter_set_id: Exp-Golomb PPS identifier.
    std::uint32_t seq_parameter_set_id = 0; // seq_parameter_set_id: references the active SPS.
    std::uint32_t entropy_coding_mode_flag : 1 = 0; // entropy_coding_mode_flag: CABAC enable.
    std::uint32_t pic_order_present_flag : 1 = 0; // bottom_field_pic_order_in_frame_present_flag in spec naming.
    std::uint32_t num_slice_groups_minus1 = 0; // num_slice_groups_minus1: FMO slice group count minus 1.
    std::uint32_t num_ref_idx_l0_default_active_minus1 = 0; // default L0 reference count minus 1.
    std::uint32_t num_ref_idx_l1_default_active_minus1 = 0; // default L1 reference count minus 1.
    std::uint32_t weighted_pred_flag : 1 = 0; // weighted_pred_flag: P-slice weighted prediction enable.
    std::uint32_t weighted_bipred_idc : 2 = 0; // weighted_bipred_idc: B-slice weighted bipred mode.
    std::int32_t pic_init_qp_minus26 = 0; // pic_init_qp_minus26: initial luma QP offset.
    std::int32_t pic_init_qs_minus26 = 0; // pic_init_qs_minus26: initial chroma QP offset for SP/SI.
    std::int32_t chroma_qp_index_offset = 0; // chroma_qp_index_offset: primary chroma QP offset.
    std::uint32_t deblocking_filter_control_present_flag : 1 = 1; // deblocking_filter_control_present_flag: deblock syntax presence.
    std::uint32_t constrained_intra_pred_flag : 1 = 0; // constrained_intra_pred_flag: intra prediction restriction.
    std::uint32_t redundant_pic_cnt_present_flag : 1 = 0; // redundant_pic_cnt_present_flag: redundant picture count presence.
    std::uint32_t transform_8x8_mode_flag : 1 = 0; // transform_8x8_mode_flag: 8x8 transform enable.
    std::uint32_t pic_scaling_matrix_present_flag : 1 = 0; // pic_scaling_matrix_present_flag: PPS scaling matrix presence.
    std::int32_t second_chroma_qp_index_offset = 0; // second_chroma_qp_index_offset: secondary chroma QP offset.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            ue(pic_parameter_set_id),
            ue(seq_parameter_set_id),
            bits<1>(entropy_coding_mode_flag),
            bits<1>(pic_order_present_flag),
            ue(num_slice_groups_minus1),
            ue(num_ref_idx_l0_default_active_minus1),
            ue(num_ref_idx_l1_default_active_minus1),
            bits<1>(weighted_pred_flag),
            bits<2>(weighted_bipred_idc),
            se(pic_init_qp_minus26),
            se(pic_init_qs_minus26),
            se(chroma_qp_index_offset),
            bits<1>(deblocking_filter_control_present_flag),
            bits<1>(constrained_intra_pred_flag),
            bits<1>(redundant_pic_cnt_present_flag),
            bits<1>(transform_8x8_mode_flag),
            bits<1>(pic_scaling_matrix_present_flag),
            se(second_chroma_qp_index_offset)));
    }
};

struct HevcProfileTierLevelSyntax {
    std::uint32_t general_profile_space : 2 = 0; // general_profile_space: profile namespace.
    std::uint32_t general_tier_flag : 1 = 0; // general_tier_flag: Main/High tier selector.
    std::uint32_t general_profile_idc : 5 = 1; // general_profile_idc: HEVC profile identifier.
    std::uint32_t general_profile_compatibility_flags : 32 = 0; // general_profile_compatibility_flags: profile compatibility bitmap.
    std::uint32_t general_progressive_source_flag : 1 = 1; // general_progressive_source_flag: progressive source marker.
    std::uint32_t general_interlaced_source_flag : 1 = 0; // general_interlaced_source_flag: interlaced source marker.
    std::uint32_t general_non_packed_constraint_flag : 1 = 0; // general_non_packed_constraint_flag: no packed constraint.
    std::uint32_t general_frame_only_constraint_flag : 1 = 1; // general_frame_only_constraint_flag: frame-only stream.
    std::uint64_t general_constraint_flags = 0; // general_reserved_zero_43bits and profile-specific constraints.
    std::uint32_t general_inbld_flag : 1 = 0; // general_inbld_flag/general_reserved_zero_bit depending on profile.
    std::uint32_t general_level_idc : 8 = 120; // general_level_idc: decoder capability level.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            bits<2>(general_profile_space),
            bits<1>(general_tier_flag),
            bits<5>(general_profile_idc),
            bits<32>(general_profile_compatibility_flags),
            bits<1>(general_progressive_source_flag),
            bits<1>(general_interlaced_source_flag),
            bits<1>(general_non_packed_constraint_flag),
            bits<1>(general_frame_only_constraint_flag)));
        if (general_profile_idc == 2) {
            serializer.write(seq(
                bits<8>(general_constraint_flags & ((std::uint64_t{1} << 8) - 1u)),
                bits<32>((general_constraint_flags >> 8) & std::numeric_limits<std::uint32_t>::max()),
                bits<3>((general_constraint_flags >> 40) & 0x7u)));
        } else {
            serializer.write(seq(
                bits<32>(general_constraint_flags & std::numeric_limits<std::uint32_t>::max()),
                bits<11>((general_constraint_flags >> 32) & 0x7ffu)));
        }
        serializer.write(seq(bits<1>(general_inbld_flag), bits<8>(general_level_idc)));
    }
};

struct HevcVpsSyntax {
    std::uint32_t vps_video_parameter_set_id : 4 = 0; // vps_video_parameter_set_id: VPS identifier.
    std::uint32_t vps_base_layer_internal_flag : 1 = 1; // vps_base_layer_internal_flag: base layer present internally.
    std::uint32_t vps_base_layer_available_flag : 1 = 1; // vps_base_layer_available_flag: base layer decodable.
    std::uint32_t vps_max_layers_minus1 : 6 = 0; // vps_max_layers_minus1: number of layers minus 1.
    std::uint32_t vps_max_sub_layers_minus1 : 3 = 0; // vps_max_sub_layers_minus1: temporal layers minus 1.
    std::uint32_t vps_temporal_id_nesting_flag : 1 = 1; // vps_temporal_id_nesting_flag: temporal nesting flag.
    std::uint32_t vps_reserved_0xffff_16bits : 16 = 0xffff; // vps_reserved_0xffff_16bits: mandated all-ones field.
    HevcProfileTierLevelSyntax profile_tier_level{}; // profile_tier_level: general PTL syntax.
    std::uint32_t vps_sub_layer_ordering_info_present_flag : 1 = 0; // vps_sub_layer_ordering_info_present_flag: sub-layer ordering reuse.
    std::uint32_t vps_max_dec_pic_buffering_minus1 = 0; // vps_max_dec_pic_buffering_minus1[0].
    std::uint32_t vps_max_num_reorder_pics = 0; // vps_max_num_reorder_pics[0].
    std::uint32_t vps_max_latency_increase_plus1 = 0; // vps_max_latency_increase_plus1[0].
    std::uint32_t vps_max_layer_id : 6 = 0; // vps_max_layer_id: highest layer id.
    std::uint32_t vps_num_layer_sets_minus1 = 0; // vps_num_layer_sets_minus1: layer set count minus 1.
    std::uint32_t vps_timing_info_present_flag : 1 = 0; // vps_timing_info_present_flag: timing info presence.
    std::uint32_t vps_extension_flag : 1 = 0; // vps_extension_flag: VPS extension presence.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            bits<4>(vps_video_parameter_set_id),
            bits<1>(vps_base_layer_internal_flag),
            bits<1>(vps_base_layer_available_flag),
            bits<6>(vps_max_layers_minus1),
            bits<3>(vps_max_sub_layers_minus1),
            bits<1>(vps_temporal_id_nesting_flag),
            bits<16>(vps_reserved_0xffff_16bits)));
        serializer.write(profile_tier_level);
        serializer.write(seq(
            bits<1>(vps_sub_layer_ordering_info_present_flag),
            ue(vps_max_dec_pic_buffering_minus1),
            ue(vps_max_num_reorder_pics),
            ue(vps_max_latency_increase_plus1),
            bits<6>(vps_max_layer_id),
            ue(vps_num_layer_sets_minus1),
            bits<1>(vps_timing_info_present_flag),
            bits<1>(vps_extension_flag)));
    }
};

struct HevcSpsSyntax {
    std::uint32_t sps_video_parameter_set_id : 4 = 0; // sps_video_parameter_set_id: references VPS id.
    std::uint32_t sps_max_sub_layers_minus1 : 3 = 0; // sps_max_sub_layers_minus1: temporal layers minus 1.
    std::uint32_t sps_temporal_id_nesting_flag : 1 = 1; // sps_temporal_id_nesting_flag: temporal nesting flag.
    HevcProfileTierLevelSyntax profile_tier_level{}; // profile_tier_level: SPS PTL syntax.
    std::uint32_t sps_seq_parameter_set_id = 0; // sps_seq_parameter_set_id: SPS identifier.
    std::uint32_t chroma_format_idc = 1; // chroma_format_idc: chroma sampling format.
    std::uint32_t separate_colour_plane_flag : 1 = 0; // separate_colour_plane_flag: 4:4:4 plane separation.
    std::uint32_t pic_width_in_luma_samples = 0; // pic_width_in_luma_samples: coded width.
    std::uint32_t pic_height_in_luma_samples = 0; // pic_height_in_luma_samples: coded height.
    std::uint32_t conformance_window_flag : 1 = 0; // conformance_window_flag: crop rectangle presence.
    std::uint32_t bit_depth_luma_minus8 = 0; // bit_depth_luma_minus8: luma bit depth minus 8.
    std::uint32_t bit_depth_chroma_minus8 = 0; // bit_depth_chroma_minus8: chroma bit depth minus 8.
    std::uint32_t log2_max_pic_order_cnt_lsb_minus4 = 0; // log2_max_pic_order_cnt_lsb_minus4: POC LSB width minus 4.
    std::uint32_t sps_sub_layer_ordering_info_present_flag : 1 = 0; // sps_sub_layer_ordering_info_present_flag: per-layer ordering syntax.
    std::uint32_t sps_max_dec_pic_buffering_minus1 = 0; // sps_max_dec_pic_buffering_minus1[0].
    std::uint32_t sps_max_num_reorder_pics = 0; // sps_max_num_reorder_pics[0].
    std::uint32_t sps_max_latency_increase_plus1 = 0; // sps_max_latency_increase_plus1[0].
    std::uint32_t log2_min_luma_coding_block_size_minus3 = 0; // log2_min_luma_coding_block_size_minus3: minimum coding block size.
    std::uint32_t log2_diff_max_min_luma_coding_block_size = 0; // log2_diff_max_min_luma_coding_block_size: CTB size delta.
    std::uint32_t log2_min_transform_block_size_minus2 = 0; // log2_min_transform_block_size_minus2: min TU size.
    std::uint32_t log2_diff_max_min_transform_block_size = 0; // log2_diff_max_min_transform_block_size: max TU delta.
    std::uint32_t max_transform_hierarchy_depth_inter = 0; // max_transform_hierarchy_depth_inter: inter TU hierarchy depth.
    std::uint32_t max_transform_hierarchy_depth_intra = 0; // max_transform_hierarchy_depth_intra: intra TU hierarchy depth.
    std::uint32_t scaling_list_enabled_flag : 1 = 0; // scaling_list_enabled_flag: scaling list syntax enable.
    std::uint32_t sps_scaling_list_data_present_flag : 1 = 0; // sps_scaling_list_data_present_flag: scaling list payload presence.
    std::uint32_t amp_enabled_flag : 1 = 0; // amp_enabled_flag: asymmetric motion partitions.
    std::uint32_t sample_adaptive_offset_enabled_flag : 1 = 0; // sample_adaptive_offset_enabled_flag: SAO enable.
    std::uint32_t pcm_enabled_flag : 1 = 0; // pcm_enabled_flag: PCM coding enable.
    std::uint32_t pcm_sample_bit_depth_luma_minus1 : 4 = 0; // pcm_sample_bit_depth_luma_minus1: PCM luma depth minus 1.
    std::uint32_t pcm_sample_bit_depth_chroma_minus1 : 4 = 0; // pcm_sample_bit_depth_chroma_minus1: PCM chroma depth minus 1.
    std::uint32_t log2_min_pcm_luma_coding_block_size_minus3 = 0; // log2_min_pcm_luma_coding_block_size_minus3: min PCM block size.
    std::uint32_t log2_diff_max_min_pcm_luma_coding_block_size = 0; // log2_diff_max_min_pcm_luma_coding_block_size: max PCM block delta.
    std::uint32_t pcm_loop_filter_disabled_flag : 1 = 0; // pcm_loop_filter_disabled_flag: PCM loop filter bypass.
    std::uint32_t num_short_term_ref_pic_sets = 0; // num_short_term_ref_pic_sets: STRPS count.
    std::uint32_t long_term_ref_pics_present_flag : 1 = 0; // long_term_ref_pics_present_flag: LTR syntax enable.
    std::uint32_t num_long_term_ref_pic_sps = 0; // num_long_term_ref_pic_sps: SPS LTR count.
    std::uint32_t sps_temporal_mvp_enabled_flag : 1 = 0; // sps_temporal_mvp_enabled_flag: temporal MVP enable.
    std::uint32_t strong_intra_smoothing_enabled_flag : 1 = 0; // strong_intra_smoothing_enabled_flag: intra smoothing enable.
    std::uint32_t vui_parameters_present_flag : 1 = 0; // vui_parameters_present_flag: VUI syntax presence.
    std::uint32_t sps_extension_present_flag : 1 = 0; // sps_extension_present_flag: SPS extension presence.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            bits<4>(sps_video_parameter_set_id),
            bits<3>(sps_max_sub_layers_minus1),
            bits<1>(sps_temporal_id_nesting_flag)));
        serializer.write(profile_tier_level);
        serializer.write(seq(
            ue(sps_seq_parameter_set_id),
            ue(chroma_format_idc)));
        serializer.write(when(chroma_format_idc == 3, bits<1>(separate_colour_plane_flag)));
        serializer.write(seq(
            ue(pic_width_in_luma_samples),
            ue(pic_height_in_luma_samples),
            bits<1>(conformance_window_flag),
            ue(bit_depth_luma_minus8),
            ue(bit_depth_chroma_minus8),
            ue(log2_max_pic_order_cnt_lsb_minus4),
            bits<1>(sps_sub_layer_ordering_info_present_flag),
            ue(sps_max_dec_pic_buffering_minus1),
            ue(sps_max_num_reorder_pics),
            ue(sps_max_latency_increase_plus1),
            ue(log2_min_luma_coding_block_size_minus3),
            ue(log2_diff_max_min_luma_coding_block_size),
            ue(log2_min_transform_block_size_minus2),
            ue(log2_diff_max_min_transform_block_size),
            ue(max_transform_hierarchy_depth_inter),
            ue(max_transform_hierarchy_depth_intra),
            bits<1>(scaling_list_enabled_flag)));
        serializer.write(when(scaling_list_enabled_flag, bits<1>(sps_scaling_list_data_present_flag)));
        serializer.write(seq(
            bits<1>(amp_enabled_flag),
            bits<1>(sample_adaptive_offset_enabled_flag),
            bits<1>(pcm_enabled_flag)));
        if (pcm_enabled_flag) {
            serializer.write(seq(
                bits<4>(pcm_sample_bit_depth_luma_minus1),
                bits<4>(pcm_sample_bit_depth_chroma_minus1),
                ue(log2_min_pcm_luma_coding_block_size_minus3),
                ue(log2_diff_max_min_pcm_luma_coding_block_size),
                bits<1>(pcm_loop_filter_disabled_flag)));
        }
        serializer.write(ue(num_short_term_ref_pic_sets));
        if (num_short_term_ref_pic_sets > 0) {
            for (std::uint32_t set_idx = 0; set_idx < num_short_term_ref_pic_sets; ++set_idx) {
                const std::uint32_t num_negative_pics = (set_idx == 0) ? num_short_term_ref_pic_sets : set_idx;
                if (set_idx > 0) {
                    serializer.write(bits<1>(0));
                }
                serializer.write(seq(ue(num_negative_pics), ue(0)));
                for (std::uint32_t index = 0; index < num_negative_pics; ++index) {
                    serializer.write(seq(ue(0), bits<1>(1)));
                }
            }
        }
        serializer.write(bits<1>(long_term_ref_pics_present_flag));
        if (long_term_ref_pics_present_flag) {
            serializer.write(ue(num_long_term_ref_pic_sps));
        }
        serializer.write(seq(
            bits<1>(sps_temporal_mvp_enabled_flag),
            bits<1>(strong_intra_smoothing_enabled_flag),
            bits<1>(vui_parameters_present_flag),
            bits<1>(sps_extension_present_flag)));
    }
};

struct HevcPpsSyntax {
    std::uint32_t pps_pic_parameter_set_id = 0; // pps_pic_parameter_set_id: PPS identifier.
    std::uint32_t pps_seq_parameter_set_id = 0; // pps_seq_parameter_set_id: referenced SPS id.
    std::uint32_t dependent_slice_segments_enabled_flag : 1 = 0; // dependent_slice_segments_enabled_flag: dependent segment enable.
    std::uint32_t output_flag_present_flag : 1 = 0; // output_flag_present_flag: pic_output_flag syntax presence.
    std::uint32_t num_extra_slice_header_bits : 3 = 0; // num_extra_slice_header_bits: reserved per-slice flags.
    std::uint32_t sign_data_hiding_enabled_flag : 1 = 0; // sign_data_hiding_enabled_flag: sign hiding enable.
    std::uint32_t cabac_init_present_flag : 1 = 0; // cabac_init_present_flag: CABAC init override.
    std::uint32_t num_ref_idx_l0_default_active_minus1 = 0; // num_ref_idx_l0_default_active_minus1: default L0 refs minus 1.
    std::uint32_t num_ref_idx_l1_default_active_minus1 = 0; // num_ref_idx_l1_default_active_minus1: default L1 refs minus 1.
    std::int32_t init_qp_minus26 = 0; // init_qp_minus26: initial luma QP delta.
    std::uint32_t constrained_intra_pred_flag : 1 = 0; // constrained_intra_pred_flag: intra-only reference restriction.
    std::uint32_t transform_skip_enabled_flag : 1 = 0; // transform_skip_enabled_flag: transform skip enable.
    std::uint32_t cu_qp_delta_enabled_flag : 1 = 0; // cu_qp_delta_enabled_flag: CU QP delta enable.
    std::uint32_t diff_cu_qp_delta_depth = 0; // diff_cu_qp_delta_depth: CU depth for QP deltas.
    std::int32_t pps_cb_qp_offset = 0; // pps_cb_qp_offset: Cb QP offset.
    std::int32_t pps_cr_qp_offset = 0; // pps_cr_qp_offset: Cr QP offset.
    std::uint32_t pps_slice_chroma_qp_offsets_present_flag : 1 = 0; // pps_slice_chroma_qp_offsets_present_flag: slice chroma QP offsets.
    std::uint32_t weighted_pred_flag : 1 = 0; // weighted_pred_flag: P-slice weighted prediction.
    std::uint32_t weighted_bipred_flag : 1 = 0; // weighted_bipred_flag: B-slice weighted bipred.
    std::uint32_t transquant_bypass_enabled_flag : 1 = 0; // transquant_bypass_enabled_flag: transquant bypass.
    std::uint32_t tiles_enabled_flag : 1 = 0; // tiles_enabled_flag: tile syntax enable.
    std::uint32_t entropy_coding_sync_enabled_flag : 1 = 0; // entropy_coding_sync_enabled_flag: WPP enable.
    std::uint32_t num_tile_columns_minus1 = 0; // num_tile_columns_minus1: tile columns minus 1.
    std::uint32_t num_tile_rows_minus1 = 0; // num_tile_rows_minus1: tile rows minus 1.
    bool uniform_spacing_flag = true; // uniform_spacing_flag: uniform tile spacing mode.
    std::span<const std::uint16_t> column_width_minus1{}; // column_width_minus1: explicit tile column widths.
    std::span<const std::uint16_t> row_height_minus1{}; // row_height_minus1: explicit tile row heights.
    std::uint32_t loop_filter_across_tiles_enabled_flag : 1 = 0; // loop_filter_across_tiles_enabled_flag: deblock across tiles.
    std::uint32_t pps_loop_filter_across_slices_enabled_flag : 1 = 0; // pps_loop_filter_across_slices_enabled_flag: deblock across slices.
    std::uint32_t deblocking_filter_control_present_flag : 1 = 1; // deblocking_filter_control_present_flag: deblock syntax presence.
    std::uint32_t deblocking_filter_override_enabled_flag : 1 = 0; // deblocking_filter_override_enabled_flag: slice override enable.
    std::uint32_t pps_disable_deblocking_filter_flag : 1 = 0; // pps_disable_deblocking_filter_flag: disable deblock.
    std::int32_t pps_beta_offset_div2 = 0; // pps_beta_offset_div2: beta deblock offset.
    std::int32_t pps_tc_offset_div2 = 0; // pps_tc_offset_div2: tc deblock offset.
    std::uint32_t pps_scaling_list_data_present_flag : 1 = 0; // pps_scaling_list_data_present_flag: PPS scaling lists.
    std::uint32_t lists_modification_present_flag : 1 = 0; // lists_modification_present_flag: reference list modification syntax.
    std::uint32_t log2_parallel_merge_level_minus2 = 0; // log2_parallel_merge_level_minus2: merge level depth.
    std::uint32_t slice_segment_header_extension_present_flag : 1 = 0; // slice_segment_header_extension_present_flag: slice extension syntax.
    std::uint32_t pps_extension_present_flag : 1 = 0; // pps_extension_present_flag: PPS extension presence.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            ue(pps_pic_parameter_set_id),
            ue(pps_seq_parameter_set_id),
            bits<1>(dependent_slice_segments_enabled_flag),
            bits<1>(output_flag_present_flag),
            bits<3>(num_extra_slice_header_bits),
            bits<1>(sign_data_hiding_enabled_flag),
            bits<1>(cabac_init_present_flag),
            ue(num_ref_idx_l0_default_active_minus1),
            ue(num_ref_idx_l1_default_active_minus1),
            se(init_qp_minus26),
            bits<1>(constrained_intra_pred_flag),
            bits<1>(transform_skip_enabled_flag),
            bits<1>(cu_qp_delta_enabled_flag)));
        if (cu_qp_delta_enabled_flag) {
            serializer.write(ue(diff_cu_qp_delta_depth));
        }
        serializer.write(seq(
            se(pps_cb_qp_offset),
            se(pps_cr_qp_offset),
            bits<1>(pps_slice_chroma_qp_offsets_present_flag),
            bits<1>(weighted_pred_flag),
            bits<1>(weighted_bipred_flag),
            bits<1>(transquant_bypass_enabled_flag),
            bits<1>(tiles_enabled_flag),
            bits<1>(entropy_coding_sync_enabled_flag)));
        if (tiles_enabled_flag) {
            serializer.write(seq(
                ue(num_tile_columns_minus1),
                ue(num_tile_rows_minus1),
                bits<1>(uniform_spacing_flag ? 1u : 0u)));
            if (!uniform_spacing_flag) {
                for (std::uint32_t index = 0; index < num_tile_columns_minus1; ++index) {
                    serializer.write(ue(column_width_minus1[index]));
                }
                for (std::uint32_t index = 0; index < num_tile_rows_minus1; ++index) {
                    serializer.write(ue(row_height_minus1[index]));
                }
            }
            serializer.write(bits<1>(loop_filter_across_tiles_enabled_flag));
        }
        serializer.write(seq(
            bits<1>(pps_loop_filter_across_slices_enabled_flag),
            bits<1>(deblocking_filter_control_present_flag),
            bits<1>(deblocking_filter_override_enabled_flag),
            bits<1>(pps_disable_deblocking_filter_flag)));
        if (!pps_disable_deblocking_filter_flag) {
            serializer.write(seq(se(pps_beta_offset_div2), se(pps_tc_offset_div2)));
        }
        serializer.write(seq(
            bits<1>(pps_scaling_list_data_present_flag),
            bits<1>(lists_modification_present_flag),
            ue(log2_parallel_merge_level_minus2),
            bits<1>(slice_segment_header_extension_present_flag),
            bits<1>(pps_extension_present_flag)));
    }
};

struct Av1ObuHeaderSyntax {
    std::uint32_t obu_forbidden_bit : 1 = 0; // obu_forbidden_bit: always zero.
    std::uint32_t obu_type : 4 = 0; // obu_type: OBU payload type.
    std::uint32_t obu_extension_flag : 1 = 0; // obu_extension_flag: extension header presence.
    std::uint32_t obu_has_size_field : 1 = 1; // obu_has_size_field: leb128 size field presence.
    std::uint32_t obu_reserved_1bit : 1 = 0; // obu_reserved_1bit: reserved zero bit.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            bits<1>(obu_forbidden_bit),
            bits<4>(obu_type),
            bits<1>(obu_extension_flag),
            bits<1>(obu_has_size_field),
            bits<1>(obu_reserved_1bit)));
    }
};

struct Av1SequenceHeaderSyntax {
    std::uint32_t seq_profile : 3 = 0; // seq_profile: AV1 profile.
    std::uint32_t still_picture : 1 = 0; // still_picture: still-image sequence marker.
    std::uint32_t reduced_still_picture_header : 1 = 0; // reduced_still_picture_header: reduced header mode.
    std::uint32_t timing_info_present_flag : 1 = 0; // timing_info_present_flag: timing syntax presence.
    std::uint32_t initial_display_delay_present_flag : 1 = 0; // initial_display_delay_present_flag: display delay syntax presence.
    std::uint32_t operating_points_cnt_minus_1 : 5 = 0; // operating_points_cnt_minus_1: operating point count minus 1.
    std::uint32_t operating_point_idc_0 : 12 = 0; // operating_point_idc[0]: spatial/temporal operating point mask.
    std::uint32_t seq_level_idx_0 : 5 = 9; // seq_level_idx[0]: decoder level index.
    std::uint32_t seq_tier_0 : 1 = 0; // seq_tier[0]: main/high tier selector.
    std::uint32_t frame_width_bits_minus_1 : 4 = 0; // frame_width_bits_minus_1: width field bit count minus 1.
    std::uint32_t frame_height_bits_minus_1 : 4 = 0; // frame_height_bits_minus_1: height field bit count minus 1.
    std::uint32_t max_frame_width_minus_1 = 0; // max_frame_width_minus_1: coded width minus 1.
    std::uint32_t max_frame_height_minus_1 = 0; // max_frame_height_minus_1: coded height minus 1.
    std::uint32_t frame_id_numbers_present_flag : 1 = 0; // frame_id_numbers_present_flag: frame id syntax presence.
    std::uint32_t use_128x128_superblock : 1 = 0; // use_128x128_superblock: superblock size selector.
    std::uint32_t enable_filter_intra : 1 = 0; // enable_filter_intra: filter intra prediction enable.
    std::uint32_t enable_intra_edge_filter : 1 = 0; // enable_intra_edge_filter: intra edge filter enable.
    std::uint32_t enable_interintra_compound : 1 = 0; // enable_interintra_compound: inter-intra compound enable.
    std::uint32_t enable_masked_compound : 1 = 0; // enable_masked_compound: masked compound enable.
    std::uint32_t enable_warped_motion : 1 = 1; // enable_warped_motion: warped motion enable.
    std::uint32_t enable_dual_filter : 1 = 0; // enable_dual_filter: dual interpolation filters enable.
    std::uint32_t enable_order_hint : 1 = 0; // enable_order_hint: order hint syntax enable.
    std::uint32_t enable_jnt_comp : 1 = 0; // enable_jnt_comp: joint compound enable.
    std::uint32_t enable_ref_frame_mvs : 1 = 1; // enable_ref_frame_mvs: reference frame MV usage.
    std::uint32_t seq_choose_screen_content_tools : 1 = 1; // seq_choose_screen_content_tools: decoder chooses SCT mode.
    std::uint32_t seq_force_screen_content_tools : 1 = 0; // seq_force_screen_content_tools: forced SCT value if not chosen.
    std::uint32_t seq_choose_integer_mv : 1 = 1; // seq_choose_integer_mv: decoder chooses integer MV mode.
    std::uint32_t seq_force_integer_mv : 1 = 0; // seq_force_integer_mv: forced integer MV if not chosen.
    std::uint32_t order_hint_bits_minus_1 : 3 = 0; // order_hint_bits_minus_1: order hint width minus 1.
    std::uint32_t use_superres : 1 = 0; // enable_superres: super-resolution usage.
    std::uint32_t enable_cdef : 1 = 0; // enable_cdef: CDEF syntax enable.
    std::uint32_t enable_restoration : 1 = 0; // enable_restoration: loop restoration syntax enable.
    std::uint32_t high_bitdepth : 1 = 0; // high_bitdepth: bit depth > 8 marker.
    std::uint32_t twelve_bit : 1 = 0; // twelve_bit: bit depth == 12 for profile 2.
    std::uint32_t mono_chrome : 1 = 0; // mono_chrome: monochrome sequence flag.
    std::uint32_t color_description_present_flag : 1 = 1; // color_description_present_flag: color description syntax presence.
    std::uint32_t color_primaries : 8 = 1; // color_primaries: ITU-T H.273 primary set.
    std::uint32_t transfer_characteristics : 8 = 2; // transfer_characteristics: transfer function identifier.
    std::uint32_t matrix_coefficients : 8 = 2; // matrix_coefficients: matrix coefficient identifier.
    std::uint32_t color_range : 1 = 0; // color_range: full/limited range.
    std::uint32_t chroma_sample_position : 2 = 0; // chroma_sample_position: chroma siting for 4:2:0.
    std::uint32_t separate_uv_delta_q : 1 = 0; // separate_uv_delta_q: separate U/V delta-Q enable.
    std::uint32_t film_grain_params_present : 1 = 0; // film_grain_params_present: film grain syntax presence.
    std::uint32_t width_bits = 1; // width_bits: serializer-only actual width field width.
    std::uint32_t height_bits = 1; // height_bits: serializer-only actual height field width.
    bool subsampling_x = true; // subsampling_x: horizontal chroma subsampling.
    bool subsampling_y = true; // subsampling_y: vertical chroma subsampling.

    void serialize(Serializer& serializer) const {
        serializer.write(seq(
            bits<3>(seq_profile),
            bits<1>(still_picture),
            bits<1>(reduced_still_picture_header),
            bits<1>(timing_info_present_flag),
            bits<1>(initial_display_delay_present_flag),
            bits<5>(operating_points_cnt_minus_1),
            bits<12>(operating_point_idc_0),
            bits<5>(seq_level_idx_0),
            bits<1>(seq_tier_0),
            bits<4>(frame_width_bits_minus_1),
            bits<4>(frame_height_bits_minus_1)));
        serializer.write(seq(
            bits(max_frame_width_minus_1, static_cast<int>(width_bits)),
            bits(max_frame_height_minus_1, static_cast<int>(height_bits)),
            bits<1>(frame_id_numbers_present_flag),
            bits<1>(use_128x128_superblock),
            bits<1>(enable_filter_intra),
            bits<1>(enable_intra_edge_filter),
            bits<1>(enable_interintra_compound),
            bits<1>(enable_masked_compound),
            bits<1>(enable_warped_motion),
            bits<1>(enable_dual_filter),
            bits<1>(enable_order_hint)));
        if (enable_order_hint) {
            serializer.write(seq(bits<1>(enable_jnt_comp), bits<1>(enable_ref_frame_mvs)));
        }
        serializer.write(bits<1>(seq_choose_screen_content_tools));
        if (!seq_choose_screen_content_tools) {
            serializer.write(bits<1>(seq_force_screen_content_tools));
        } else {
            serializer.write(bits<1>(seq_choose_integer_mv));
            if (!seq_choose_integer_mv) {
                serializer.write(bits<1>(seq_force_integer_mv));
            }
        }
        if (enable_order_hint) {
            serializer.write(bits<3>(order_hint_bits_minus_1));
        }
        serializer.write(seq(
            bits<1>(use_superres),
            bits<1>(enable_cdef),
            bits<1>(enable_restoration),
            bits<1>(high_bitdepth)));
        if (seq_profile == 2) {
            serializer.write(bits<1>(twelve_bit));
        }
        if (seq_profile != 1) {
            serializer.write(bits<1>(mono_chrome));
        }
        serializer.write(bits<1>(color_description_present_flag));
        if (color_description_present_flag) {
            serializer.write(seq(
                bits<8>(color_primaries),
                bits<8>(transfer_characteristics),
                bits<8>(matrix_coefficients)));
        }
        serializer.write(bits<1>(color_range));
        if (!mono_chrome && subsampling_x && subsampling_y) {
            serializer.write(bits<2>(chroma_sample_position));
        }
        serializer.write(seq(bits<1>(separate_uv_delta_q), bits<1>(film_grain_params_present)));
    }
};

struct Av1FrameHeaderSyntax {
    std::uint32_t show_existing_frame : 1 = 0; // show_existing_frame: display an existing reference instead of decoding a new frame.
    std::uint32_t frame_type : 2 = 0; // frame_type: key/inter/intra-only/switch frame selector.
    std::uint32_t show_frame : 1 = 1; // show_frame: display the decoded frame immediately.
    std::uint32_t showable_frame : 1 = 0; // showable_frame: frame can be shown later by show_existing_frame.
    std::uint32_t error_resilient_mode : 1 = 0; // error_resilient_mode: independent entropy/reset behavior.
    std::uint32_t disable_cdf_update : 1 = 0; // disable_cdf_update: disables CDF adaptation.
    std::uint32_t allow_screen_content_tools : 1 = 0; // allow_screen_content_tools: screen-content tool selection.
    std::uint32_t force_integer_mv : 1 = 0; // force_integer_mv: integer-only motion vectors.
    std::uint32_t frame_size_override_flag : 1 = 0; // frame_size_override_flag: explicit frame size signaling.
    std::uint32_t order_hint = 0; // order_hint: display/reference order hint.
    std::uint32_t order_hint_bits = 0; // order_hint_bits: serializer-only order hint width.
    std::uint32_t primary_ref_frame : 3 = 0; // primary_ref_frame: entropy context source frame.
    std::uint32_t refresh_frame_flags : 8 = 0; // refresh_frame_flags: reference slots updated by this frame.
    std::uint32_t render_and_frame_size_different : 1 = 0; // render_and_frame_size_different: render size differs from frame size.
    std::uint32_t allow_intrabc : 1 = 0; // allow_intrabc: intrabc availability for intra frames.
    std::uint32_t frame_refs_short_signaling : 1 = 0; // frame_refs_short_signaling: compact reference signaling.
    std::array<std::uint8_t, 7> ref_frame_idx{}; // ref_frame_idx[7]: reference frame slot indices.
    std::uint32_t allow_high_precision_mv : 1 = 0; // allow_high_precision_mv: subpel MV precision.
    std::uint32_t is_filter_switchable : 1 = 1; // is_filter_switchable: per-block interpolation filter select.
    std::uint32_t interpolation_filter : 2 = 0; // interpolation_filter: global interpolation filter when not switchable.
    std::uint32_t is_motion_mode_switchable : 1 = 0; // is_motion_mode_switchable: per-block motion mode switch.
    std::uint32_t use_ref_frame_mvs : 1 = 0; // use_ref_frame_mvs: projected MV usage.
    std::uint32_t disable_frame_end_update_cdf : 1 = 0; // disable_frame_end_update_cdf: suppresses end-of-frame CDF update.
    std::uint32_t uniform_tile_spacing_flag : 1 = 1; // uniform_tile_spacing_flag: uniform tile layout mode.
    std::uint32_t tile_cols_stop_bit : 1 = 0; // tile_cols_log2 stop bit for 1x1 uniform layout.
    std::uint32_t tile_rows_stop_bit : 1 = 0; // tile_rows_log2 stop bit for 1x1 uniform layout.
    std::uint32_t base_qindex : 8 = 0; // base_q_idx: frame base quantizer index.
    std::int32_t y_dc_delta_q = 0; // delta_q_y_dc: luma DC delta quantizer.
    std::int32_t u_dc_delta_q = 0; // delta_q_u_dc: chroma U DC delta quantizer.
    std::int32_t u_ac_delta_q = 0; // delta_q_u_ac: chroma U AC delta quantizer.
    std::uint32_t using_qmatrix : 1 = 0; // using_qmatrix: quantization matrix usage.
    std::uint32_t qm_y : 4 = 0; // qm_y: luma qmatrix index.
    std::uint32_t qm_u : 4 = 0; // qm_u: chroma U qmatrix index.
    std::uint32_t qm_v : 4 = 0; // qm_v: chroma V qmatrix index.
    std::uint32_t segmentation_enabled : 1 = 0; // segmentation_enabled: segmentation syntax enable.
    std::uint32_t segmentation_update_map : 1 = 0; // segmentation_update_map: segment id map update.
    std::uint32_t segmentation_temporal_update : 1 = 0; // segmentation_temporal_update: temporal map update.
    std::uint32_t segmentation_update_data : 1 = 0; // segmentation_update_data: segment feature data update.
    std::uint32_t delta_q_present_flag : 1 = 0; // delta_q_present_flag: block-level delta Q enable.
    std::uint32_t log2_delta_q_res : 2 = 0; // log2_delta_q_res: delta Q resolution.
    std::uint32_t delta_lf_present_flag : 1 = 0; // delta_lf_present_flag: block-level delta LF enable.
    std::uint32_t log2_delta_lf_res : 2 = 0; // log2_delta_lf_res: delta LF resolution.
    std::uint32_t delta_lf_multi : 1 = 0; // delta_lf_multi: per-plane delta LF signaling.
    std::uint32_t loop_filter_level_y_vertical : 6 = 0; // loop_filter_level[0]: vertical luma filter level.
    std::uint32_t loop_filter_level_y_horizontal : 6 = 0; // loop_filter_level[1]: horizontal luma filter level.
    std::uint32_t loop_filter_level_u : 6 = 0; // loop_filter_level[2]: chroma U filter level.
    std::uint32_t loop_filter_level_v : 6 = 0; // loop_filter_level[3]: chroma V filter level.
    std::uint32_t sharpness_level : 3 = 0; // sharpness_level: loop filter sharpness.
    std::uint32_t mode_ref_delta_enabled : 1 = 0; // mode_ref_delta_enabled: reference/mode LF deltas enabled.
    std::uint32_t mode_ref_delta_update : 1 = 0; // mode_ref_delta_update: reference/mode LF deltas updated.
    std::array<std::int32_t, 8> ref_deltas{}; // ref_deltas[8]: reference-frame loop filter deltas.
    std::array<std::int32_t, 2> mode_deltas{}; // mode_deltas[2]: mode-based loop filter deltas.
    std::uint32_t cdef_damping_minus_3 : 2 = 0; // cdef_damping_minus_3: CDEF damping minus 3.
    std::uint32_t cdef_bits : 2 = 0; // cdef_bits: number of CDEF strength entries.
    std::array<std::uint8_t, 8> cdef_y_strengths{}; // cdef_y_strengths[8]: luma CDEF strengths.
    std::array<std::uint8_t, 8> cdef_uv_strengths{}; // cdef_uv_strengths[8]: chroma CDEF strengths.
    std::uint32_t enable_restoration : 1 = 0; // serializer-only sequence gate for restoration syntax.
    std::uint32_t lr_type_y : 2 = 0; // lr_type[0]: luma restoration type.
    std::uint32_t lr_type_u : 2 = 0; // lr_type[1]: chroma U restoration type.
    std::uint32_t lr_type_v : 2 = 0; // lr_type[2]: chroma V restoration type.
    std::uint32_t lr_unit_shift_value = 0; // lr_unit_shift_value: restoration unit shift.
    std::uint32_t lr_unit_shift_min = 0; // serializer-only minimum unary value for lr_unit_shift.
    std::uint32_t lr_unit_shift_max = 0; // serializer-only maximum unary value for lr_unit_shift.
    std::uint32_t lr_uv_shift : 1 = 0; // lr_uv_shift: chroma restoration unit scale.
    std::uint32_t tx_mode_select : 1 = 0; // tx_mode: only TX_MODE_SELECT vs largest.
    std::uint32_t reference_select : 1 = 0; // reference_select: compound reference mode choice.
    std::uint32_t skip_mode_present : 1 = 0; // skip_mode_present: skip mode availability.
    std::uint32_t write_skip_mode_present : 1 = 0; // serializer-only gate for emitting skip_mode_present.
    std::uint32_t allow_warped_motion : 1 = 0; // allow_warped_motion: frame-level warped motion enable.
    std::uint32_t reduced_tx_set_used : 1 = 0; // reduced_tx_set: reduced transform set usage.
    std::array<std::uint32_t, 7> global_motion_is_identity{}; // is_global[1..7]: global motion identity markers.
    std::uint32_t film_grain_params_present : 1 = 0; // film_grain_params_present: film grain syntax presence.
    bool frame_is_intra = false; // serializer-only frame class discriminator.
    bool is_switch_frame = false; // serializer-only switch frame discriminator.
    bool is_key_show_frame = false; // serializer-only key-frame shortcut.
    bool mono_chrome = false; // serializer-only mono_chrome flag.
    bool include_showable_frame = false; // serializer-only gate for showable_frame syntax.
    bool write_error_resilient_mode = false; // serializer-only gate for error_resilient_mode syntax.
    bool write_primary_ref_frame = false; // serializer-only gate for primary_ref_frame syntax.
    bool write_refresh_frame_flags = false; // serializer-only gate for refresh_frame_flags syntax.
    bool write_render_size_diff = true; // serializer-only gate for render_and_frame_size_different syntax.
    bool write_ref_frame_signaling = false; // serializer-only gate for inter reference syntax.
    bool write_allow_high_precision_mv = false; // serializer-only gate for allow_high_precision_mv syntax.
    bool write_use_ref_frame_mvs = false; // serializer-only gate for use_ref_frame_mvs syntax.
    bool write_disable_frame_end_update_cdf = false; // serializer-only gate for disable_frame_end_update_cdf syntax.
    bool write_delta_q_u = false; // serializer-only gate for chroma delta-Q syntax.
    bool write_delta_q_lf = false; // serializer-only gate for delta LF syntax.
    bool write_chroma_loop_filter_levels = false; // serializer-only gate for chroma loop filter levels.
    bool write_cdef = false; // serializer-only gate for CDEF syntax.
    bool write_restoration = false; // serializer-only gate for restoration syntax.
    bool write_lr_unit_shift = false; // serializer-only gate for lr_unit_shift syntax.
    bool write_lr_uv_shift = false; // serializer-only gate for lr_uv_shift syntax.
    bool write_tx_mode = false; // serializer-only gate for tx_mode syntax.
    bool write_inter_mode_bits = false; // serializer-only gate for reference_select/skip_mode/warped motion.

    void serialize(Serializer& serializer) const {
        auto write_delta_q = [&](std::int32_t delta_q) {
            serializer.write(bits(delta_q != 0 ? 1u : 0u, 1));
            if (delta_q != 0) {
                serializer.write(bits(detail::to_unsigned_bits(delta_q) & 0x7fu, 7));
            }
        };

        auto write_increment = [&](std::uint32_t value, std::uint32_t range_min, std::uint32_t range_max) {
            std::uint32_t current = range_min;
            while (current < range_max) {
                const bool increment = value > current;
                serializer.write(bits(increment ? 1u : 0u, 1));
                if (!increment) {
                    break;
                }
                ++current;
            }
        };

        serializer.write(seq(bits<1>(show_existing_frame), bits<2>(frame_type), bits<1>(show_frame)));
        serializer.write(when(include_showable_frame, bits<1>(showable_frame)));
        serializer.write(when(write_error_resilient_mode, bits<1>(error_resilient_mode)));
        serializer.write(seq(bits<1>(disable_cdf_update), bits<1>(allow_screen_content_tools)));
        serializer.write(when(allow_screen_content_tools, bits<1>(force_integer_mv)));
        serializer.write(when(!is_switch_frame, bits<1>(frame_size_override_flag)));
        serializer.write(when(order_hint_bits != 0, bits(order_hint, static_cast<int>(order_hint_bits))));
        serializer.write(when(write_primary_ref_frame, bits<3>(primary_ref_frame)));
        serializer.write(when(write_refresh_frame_flags, bits<8>(refresh_frame_flags)));
        if (frame_is_intra) {
            serializer.write(when(write_render_size_diff, bits<1>(render_and_frame_size_different)));
            serializer.write(when(allow_screen_content_tools, bits<1>(allow_intrabc)));
        } else if (write_ref_frame_signaling) {
            serializer.write(bits<1>(frame_refs_short_signaling));
            for (std::uint8_t index : ref_frame_idx) {
                serializer.write(bits<3>(index & 0x7u));
            }
            serializer.write(bits<1>(render_and_frame_size_different));
            serializer.write(when(write_allow_high_precision_mv, bits<1>(allow_high_precision_mv)));
            serializer.write(bits<1>(is_filter_switchable));
            if (!is_filter_switchable) {
                serializer.write(bits<2>(interpolation_filter));
            }
            serializer.write(bits<1>(is_motion_mode_switchable));
            serializer.write(when(write_use_ref_frame_mvs, bits<1>(use_ref_frame_mvs)));
        }
        serializer.write(when(write_disable_frame_end_update_cdf, bits<1>(disable_frame_end_update_cdf)));
        serializer.write(bits<1>(uniform_tile_spacing_flag));
        if (uniform_tile_spacing_flag) {
            serializer.write(seq(bits<1>(tile_cols_stop_bit), bits<1>(tile_rows_stop_bit)));
        }
        serializer.write(bits<8>(base_qindex));
        write_delta_q(y_dc_delta_q);
        if (write_delta_q_u) {
            write_delta_q(u_dc_delta_q);
            write_delta_q(u_ac_delta_q);
        }
        serializer.write(bits<1>(using_qmatrix));
        if (using_qmatrix) {
            serializer.write(seq(bits<4>(qm_y), bits<4>(qm_u), bits<4>(qm_v)));
        }
        serializer.write(bits<1>(segmentation_enabled));
        if (segmentation_enabled) {
            serializer.write(bits<1>(segmentation_update_map));
            if (segmentation_update_map) {
                serializer.write(bits<1>(segmentation_temporal_update));
            }
            serializer.write(bits<1>(segmentation_update_data));
        }
        if (base_qindex != 0) {
            serializer.write(bits<1>(delta_q_present_flag));
        }
        if (delta_q_present_flag) {
            serializer.write(bits<2>(log2_delta_q_res));
            if (write_delta_q_lf) {
                serializer.write(bits<1>(delta_lf_present_flag));
                if (delta_lf_present_flag) {
                    serializer.write(seq(bits<2>(log2_delta_lf_res), bits<1>(delta_lf_multi)));
                }
            }
        }
        if (!allow_intrabc) {
            serializer.write(seq(bits<6>(loop_filter_level_y_vertical), bits<6>(loop_filter_level_y_horizontal)));
            if (write_chroma_loop_filter_levels) {
                serializer.write(seq(bits<6>(loop_filter_level_u), bits<6>(loop_filter_level_v)));
            }
            serializer.write(seq(bits<3>(sharpness_level), bits<1>(mode_ref_delta_enabled)));
            if (mode_ref_delta_enabled) {
                serializer.write(bits<1>(mode_ref_delta_update));
                if (mode_ref_delta_update) {
                    for (std::int32_t ref_delta : ref_deltas) {
                        const bool update = ref_delta != 0;
                        serializer.write(bits(update ? 1u : 0u, 1));
                        if (update) {
                            serializer.write(bits(detail::to_unsigned_bits(ref_delta) & 0x7fu, 7));
                        }
                    }
                    for (std::int32_t mode_delta : mode_deltas) {
                        const bool update = mode_delta != 0;
                        serializer.write(bits(update ? 1u : 0u, 1));
                        if (update) {
                            serializer.write(bits(detail::to_unsigned_bits(mode_delta) & 0x7fu, 7));
                        }
                    }
                }
            }
        }
        if (write_cdef) {
            serializer.write(seq(bits<2>(cdef_damping_minus_3), bits<2>(cdef_bits)));
            const std::uint32_t cdef_entries = 1u << cdef_bits;
            for (std::uint32_t index = 0; index < cdef_entries; ++index) {
                serializer.write(seq(
                    bits<4>(cdef_y_strengths[index] >> 2),
                    bits<2>(cdef_y_strengths[index] & 0x3u),
                    bits<4>(cdef_uv_strengths[index] >> 2),
                    bits<2>(cdef_uv_strengths[index] & 0x3u)));
            }
        }
        if (write_restoration) {
            serializer.write(seq(bits<2>(lr_type_y), bits<2>(lr_type_u), bits<2>(lr_type_v)));
            if (write_lr_unit_shift) {
                write_increment(lr_unit_shift_value, lr_unit_shift_min, lr_unit_shift_max);
            }
            if (write_lr_uv_shift) {
                serializer.write(bits<1>(lr_uv_shift));
            }
        }
        serializer.write(when(write_tx_mode, bits<1>(tx_mode_select)));
        if (write_inter_mode_bits) {
            serializer.write(bits<1>(reference_select));
            if (write_skip_mode_present) {
                serializer.write(bits<1>(skip_mode_present));
            }
            if (!error_resilient_mode) {
                serializer.write(bits<1>(allow_warped_motion));
            }
        }
        serializer.write(bits<1>(reduced_tx_set_used));
        if (!frame_is_intra) {
            for (std::uint32_t is_identity : global_motion_is_identity) {
                serializer.write(bits<1>(is_identity));
            }
        }
        if (film_grain_params_present) {
            serializer.write(bits<1>(0));
        }
    }
};

} // namespace syntax

inline bool has_start_code(const std::uint8_t* data, size_t size) {
    if (size >= 4 && data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00 && data[3] == 0x01) {
        return true;
    }
    if (size >= 3 && data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x01) {
        return true;
    }
    return false;
}

inline void append_start_code(std::vector<std::uint8_t>& out) {
    out.insert(out.end(), {0x00, 0x00, 0x00, 0x01});
}

inline void append_start_code_3(std::vector<std::uint8_t>& out) {
    out.insert(out.end(), {0x00, 0x00, 0x01});
}

inline void append_emulation_prevention_bytes(std::vector<std::uint8_t>& out,
                                              std::span<const std::byte> rbsp) {
    out.reserve(out.size() + rbsp.size() + 2);
    int zero_count = 0;
    for (const auto byte_value : rbsp | std::views::transform([](std::byte byte) {
             return std::to_integer<std::uint8_t>(byte);
         })) {
        if (zero_count >= 2 && byte_value <= 0x03) {
            out.push_back(0x03);
            zero_count = 0;
        }
        out.push_back(byte_value);
        zero_count = (byte_value == 0x00) ? (zero_count + 1) : 0;
    }
}

inline void append_leb128(std::vector<std::uint8_t>& out, size_t value) {
    do {
        std::uint8_t byte = static_cast<std::uint8_t>(value & 0x7f);
        value >>= 7;
        if (value != 0) {
            byte |= 0x80;
        }
        out.push_back(byte);
    } while (value != 0);
}

inline std::vector<std::uint8_t> make_nal(std::span<const std::byte> rbsp,
                                          std::uint8_t nal_unit_type,
                                          std::uint8_t nal_ref_idc = 3) {
    std::vector<std::uint8_t> out;
    out.reserve(rbsp.size() + 5);
    append_start_code(out);
    out.push_back(static_cast<std::uint8_t>((nal_ref_idc << 5) | nal_unit_type));
    append_emulation_prevention_bytes(out, rbsp);
    return out;
}

inline std::vector<std::uint8_t> make_hevc_nal(std::span<const std::byte> rbsp,
                                               std::uint8_t nal_unit_type) {
    std::vector<std::uint8_t> out;
    out.reserve(rbsp.size() + 7);
    append_start_code(out);
    out.push_back(static_cast<std::uint8_t>((nal_unit_type << 1) & 0x7e));
    out.push_back(0x01);
    append_emulation_prevention_bytes(out, rbsp);
    return out;
}

inline std::vector<std::uint8_t> build_h264_headers(const VAPictureParameterBufferH264& pic) {
    syntax::H264SpsSyntax sps{};
    sps.chroma_format_idc = pic.seq_fields.bits.chroma_format_idc;
    sps.residual_colour_transform_flag = pic.seq_fields.bits.residual_colour_transform_flag;
    sps.bit_depth_luma_minus8 = pic.bit_depth_luma_minus8;
    sps.bit_depth_chroma_minus8 = pic.bit_depth_chroma_minus8;
    sps.log2_max_frame_num_minus4 = pic.seq_fields.bits.log2_max_frame_num_minus4;
    sps.pic_order_cnt_type = pic.seq_fields.bits.pic_order_cnt_type;
    sps.log2_max_pic_order_cnt_lsb_minus4 = pic.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4;
    sps.delta_pic_order_always_zero_flag = pic.seq_fields.bits.delta_pic_order_always_zero_flag;
    sps.num_ref_frames = pic.num_ref_frames;
    sps.gaps_in_frame_num_value_allowed_flag = pic.seq_fields.bits.gaps_in_frame_num_value_allowed_flag;
    sps.picture_width_in_mbs_minus1 = pic.picture_width_in_mbs_minus1;
    sps.picture_height_in_map_units_minus1 = pic.picture_height_in_mbs_minus1;
    sps.frame_mbs_only_flag = pic.seq_fields.bits.frame_mbs_only_flag;
    sps.mb_adaptive_frame_field_flag = pic.seq_fields.bits.mb_adaptive_frame_field_flag;
    sps.direct_8x8_inference_flag = pic.seq_fields.bits.direct_8x8_inference_flag;

    syntax::H264PpsSyntax pps{};
    pps.entropy_coding_mode_flag = pic.pic_fields.bits.entropy_coding_mode_flag;
    pps.pic_order_present_flag = pic.pic_fields.bits.pic_order_present_flag;
    pps.weighted_pred_flag = pic.pic_fields.bits.weighted_pred_flag;
    pps.weighted_bipred_idc = pic.pic_fields.bits.weighted_bipred_idc;
    pps.pic_init_qp_minus26 = pic.pic_init_qp_minus26;
    pps.pic_init_qs_minus26 = pic.pic_init_qs_minus26;
    pps.chroma_qp_index_offset = pic.chroma_qp_index_offset;
    pps.deblocking_filter_control_present_flag = pic.pic_fields.bits.deblocking_filter_control_present_flag;
    pps.constrained_intra_pred_flag = pic.pic_fields.bits.constrained_intra_pred_flag;
    pps.redundant_pic_cnt_present_flag = pic.pic_fields.bits.redundant_pic_cnt_present_flag;
    pps.transform_8x8_mode_flag = pic.pic_fields.bits.transform_8x8_mode_flag;
    pps.second_chroma_qp_index_offset = pic.second_chroma_qp_index_offset;

    Serializer sps_rbsp;
    sps_rbsp.write(sps);
    sps_rbsp.rbsp_trailing();

    Serializer pps_rbsp;
    pps_rbsp.write(pps);
    pps_rbsp.rbsp_trailing();

    auto sps_nal = make_nal(sps_rbsp.bytes(), 7, 3);
    auto pps_nal = make_nal(pps_rbsp.bytes(), 8, 3);

    std::vector<std::uint8_t> out;
    out.reserve(sps_nal.size() + pps_nal.size());
    out.insert(out.end(), sps_nal.begin(), sps_nal.end());
    out.insert(out.end(), pps_nal.begin(), pps_nal.end());
    return out;
}

inline std::vector<std::uint8_t> build_h264_annexb_slices(const VASliceParameterBufferH264* slices,
                                                          std::uint32_t num_slices,
                                                          const std::uint8_t* data,
                                                          size_t data_size) {
    std::vector<std::uint8_t> out;
    if (!slices || !data || data_size == 0 || num_slices == 0) {
        return out;
    }

    out.reserve(data_size + static_cast<size_t>(num_slices) * 4);
    for (std::uint32_t index = 0; index < num_slices; ++index) {
        const auto& slice = slices[index];
        if (slice.slice_data_offset > data_size || slice.slice_data_size > data_size - slice.slice_data_offset) {
            out.clear();
            return out;
        }
        const auto* slice_ptr = data + slice.slice_data_offset;
        const size_t slice_len = slice.slice_data_size;
        if (!has_start_code(slice_ptr, slice_len)) {
            append_start_code_3(out);
        }
        out.insert(out.end(), slice_ptr, slice_ptr + slice_len);
    }
    return out;
}

inline std::vector<std::uint8_t> build_hevc_annexb_slices(const VASliceParameterBufferHEVC* slices,
                                                          std::uint32_t num_slices,
                                                          const std::uint8_t* data,
                                                          size_t data_size) {
    std::vector<std::uint8_t> out;
    if (!slices || !data || data_size == 0 || num_slices == 0) {
        return out;
    }

    out.reserve(data_size + static_cast<size_t>(num_slices) * 4);
    for (std::uint32_t index = 0; index < num_slices; ++index) {
        const auto& slice = slices[index];
        if (slice.slice_data_offset > data_size || slice.slice_data_size > data_size - slice.slice_data_offset) {
            out.clear();
            return out;
        }
        const auto* slice_ptr = data + slice.slice_data_offset;
        const size_t slice_len = slice.slice_data_size;
        if (!has_start_code(slice_ptr, slice_len)) {
            append_start_code(out);
        }
        out.insert(out.end(), slice_ptr, slice_ptr + slice_len);
    }
    return out;
}

inline std::uint8_t derive_hevc_profile_idc(const VAPictureParameterBufferHEVC& pic) {
    if (pic.pic_fields.bits.chroma_format_idc == 3) {
        return 4;
    }
    return (pic.bit_depth_luma_minus8 > 0 || pic.bit_depth_chroma_minus8 > 0) ? 2 : 1;
}

inline std::uint8_t derive_hevc_level_idc(std::uint32_t width, std::uint32_t height) {
    const std::uint64_t pixels = static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
    if (pixels <= 1920ull * 1080ull) {
        return 123;
    }
    if (pixels <= 3840ull * 2160ull) {
        return 150;
    }
    if (pixels <= 7680ull * 4320ull) {
        return 180;
    }
    return 183;
}

inline syntax::HevcProfileTierLevelSyntax make_hevc_profile_tier_level(std::uint8_t profile_idc,
                                                                       std::uint8_t level_idc,
                                                                       bool high_tier = false) {
    syntax::HevcProfileTierLevelSyntax syntax{};
    syntax.general_tier_flag = high_tier ? 1u : 0u;
    syntax.general_profile_idc = profile_idc & 0x1fu;
    syntax.general_profile_compatibility_flags = std::uint32_t{1} << (31 - (profile_idc & 0x1fu));
    syntax.general_level_idc = level_idc;
    return syntax;
}

inline std::vector<std::uint8_t> build_hevc_headers(const VAPictureParameterBufferHEVC& pic) {
    const std::uint8_t profile_idc = derive_hevc_profile_idc(pic);
    const std::uint8_t level_idc = derive_hevc_level_idc(pic.pic_width_in_luma_samples,
                                                         pic.pic_height_in_luma_samples);

    syntax::HevcVpsSyntax vps{};
    vps.profile_tier_level = make_hevc_profile_tier_level(profile_idc, level_idc);
    vps.vps_max_dec_pic_buffering_minus1 = pic.sps_max_dec_pic_buffering_minus1;

    syntax::HevcSpsSyntax sps{};
    sps.profile_tier_level = make_hevc_profile_tier_level(profile_idc, level_idc);
    sps.chroma_format_idc = pic.pic_fields.bits.chroma_format_idc;
    sps.separate_colour_plane_flag = pic.pic_fields.bits.separate_colour_plane_flag;
    sps.pic_width_in_luma_samples = pic.pic_width_in_luma_samples;
    sps.pic_height_in_luma_samples = pic.pic_height_in_luma_samples;
    sps.bit_depth_luma_minus8 = pic.bit_depth_luma_minus8;
    sps.bit_depth_chroma_minus8 = pic.bit_depth_chroma_minus8;
    sps.log2_max_pic_order_cnt_lsb_minus4 = pic.log2_max_pic_order_cnt_lsb_minus4;
    sps.sps_max_dec_pic_buffering_minus1 = pic.sps_max_dec_pic_buffering_minus1;
    sps.log2_min_luma_coding_block_size_minus3 = pic.log2_min_luma_coding_block_size_minus3;
    sps.log2_diff_max_min_luma_coding_block_size = pic.log2_diff_max_min_luma_coding_block_size;
    sps.log2_min_transform_block_size_minus2 = pic.log2_min_transform_block_size_minus2;
    sps.log2_diff_max_min_transform_block_size = pic.log2_diff_max_min_transform_block_size;
    sps.max_transform_hierarchy_depth_inter = pic.max_transform_hierarchy_depth_inter;
    sps.max_transform_hierarchy_depth_intra = pic.max_transform_hierarchy_depth_intra;
    sps.scaling_list_enabled_flag = pic.pic_fields.bits.scaling_list_enabled_flag;
    sps.amp_enabled_flag = pic.pic_fields.bits.amp_enabled_flag;
    sps.sample_adaptive_offset_enabled_flag = pic.slice_parsing_fields.bits.sample_adaptive_offset_enabled_flag;
    sps.pcm_enabled_flag = pic.pic_fields.bits.pcm_enabled_flag;
    sps.pcm_sample_bit_depth_luma_minus1 = pic.pcm_sample_bit_depth_luma_minus1;
    sps.pcm_sample_bit_depth_chroma_minus1 = pic.pcm_sample_bit_depth_chroma_minus1;
    sps.log2_min_pcm_luma_coding_block_size_minus3 = pic.log2_min_pcm_luma_coding_block_size_minus3;
    sps.log2_diff_max_min_pcm_luma_coding_block_size = pic.log2_diff_max_min_pcm_luma_coding_block_size;
    sps.pcm_loop_filter_disabled_flag = pic.pic_fields.bits.pcm_loop_filter_disabled_flag;
    sps.num_short_term_ref_pic_sets = pic.num_short_term_ref_pic_sets;
    sps.long_term_ref_pics_present_flag = pic.slice_parsing_fields.bits.long_term_ref_pics_present_flag;
    sps.num_long_term_ref_pic_sps = pic.num_long_term_ref_pic_sps;
    sps.sps_temporal_mvp_enabled_flag = pic.slice_parsing_fields.bits.sps_temporal_mvp_enabled_flag;
    sps.strong_intra_smoothing_enabled_flag = pic.pic_fields.bits.strong_intra_smoothing_enabled_flag;

    syntax::HevcPpsSyntax pps{};
    pps.dependent_slice_segments_enabled_flag = pic.slice_parsing_fields.bits.dependent_slice_segments_enabled_flag;
    pps.output_flag_present_flag = pic.slice_parsing_fields.bits.output_flag_present_flag;
    pps.num_extra_slice_header_bits = pic.num_extra_slice_header_bits;
    pps.sign_data_hiding_enabled_flag = pic.pic_fields.bits.sign_data_hiding_enabled_flag;
    pps.cabac_init_present_flag = pic.slice_parsing_fields.bits.cabac_init_present_flag;
    pps.num_ref_idx_l0_default_active_minus1 = pic.num_ref_idx_l0_default_active_minus1;
    pps.num_ref_idx_l1_default_active_minus1 = pic.num_ref_idx_l1_default_active_minus1;
    pps.init_qp_minus26 = pic.init_qp_minus26;
    pps.constrained_intra_pred_flag = pic.pic_fields.bits.constrained_intra_pred_flag;
    pps.transform_skip_enabled_flag = pic.pic_fields.bits.transform_skip_enabled_flag;
    pps.cu_qp_delta_enabled_flag = pic.pic_fields.bits.cu_qp_delta_enabled_flag;
    pps.diff_cu_qp_delta_depth = pic.diff_cu_qp_delta_depth;
    pps.pps_cb_qp_offset = pic.pps_cb_qp_offset;
    pps.pps_cr_qp_offset = pic.pps_cr_qp_offset;
    pps.pps_slice_chroma_qp_offsets_present_flag = pic.slice_parsing_fields.bits.pps_slice_chroma_qp_offsets_present_flag;
    pps.weighted_pred_flag = pic.pic_fields.bits.weighted_pred_flag;
    pps.weighted_bipred_flag = pic.pic_fields.bits.weighted_bipred_flag;
    pps.transquant_bypass_enabled_flag = pic.pic_fields.bits.transquant_bypass_enabled_flag;
    pps.tiles_enabled_flag = pic.pic_fields.bits.tiles_enabled_flag;
    pps.entropy_coding_sync_enabled_flag = pic.pic_fields.bits.entropy_coding_sync_enabled_flag;
    pps.num_tile_columns_minus1 = pic.num_tile_columns_minus1;
    pps.num_tile_rows_minus1 = pic.num_tile_rows_minus1;
    pps.uniform_spacing_flag = (pic.num_tile_columns_minus1 == 0 && pic.num_tile_rows_minus1 == 0);
    pps.column_width_minus1 = std::span<const std::uint16_t>(pic.column_width_minus1, pic.num_tile_columns_minus1);
    pps.row_height_minus1 = std::span<const std::uint16_t>(pic.row_height_minus1, pic.num_tile_rows_minus1);
    pps.loop_filter_across_tiles_enabled_flag = pic.pic_fields.bits.loop_filter_across_tiles_enabled_flag;
    pps.pps_loop_filter_across_slices_enabled_flag = pic.pic_fields.bits.pps_loop_filter_across_slices_enabled_flag;
    pps.deblocking_filter_override_enabled_flag = pic.slice_parsing_fields.bits.deblocking_filter_override_enabled_flag;
    pps.pps_disable_deblocking_filter_flag = pic.slice_parsing_fields.bits.pps_disable_deblocking_filter_flag;
    pps.pps_beta_offset_div2 = pic.pps_beta_offset_div2;
    pps.pps_tc_offset_div2 = pic.pps_tc_offset_div2;
    pps.lists_modification_present_flag = pic.slice_parsing_fields.bits.lists_modification_present_flag;
    pps.log2_parallel_merge_level_minus2 = pic.log2_parallel_merge_level_minus2;
    pps.slice_segment_header_extension_present_flag = pic.slice_parsing_fields.bits.slice_segment_header_extension_present_flag;

    Serializer vps_rbsp;
    vps_rbsp.write(vps);
    vps_rbsp.byte_align();

    Serializer sps_rbsp;
    sps_rbsp.write(sps);
    sps_rbsp.rbsp_trailing();

    Serializer pps_rbsp;
    pps_rbsp.write(pps);
    pps_rbsp.rbsp_trailing();

    const auto vps_nal = make_hevc_nal(vps_rbsp.bytes(), 32);
    const auto sps_nal = make_hevc_nal(sps_rbsp.bytes(), 33);
    const auto pps_nal = make_hevc_nal(pps_rbsp.bytes(), 34);

    std::vector<std::uint8_t> out;
    out.reserve(vps_nal.size() + sps_nal.size() + pps_nal.size());
    out.insert(out.end(), vps_nal.begin(), vps_nal.end());
    out.insert(out.end(), sps_nal.begin(), sps_nal.end());
    out.insert(out.end(), pps_nal.begin(), pps_nal.end());
    return out;
}

inline std::vector<std::uint8_t> build_av1_sequence_header(const VADecPictureParameterBufferAV1& pic) {
    const unsigned width_bits_raw = std::bit_width(static_cast<unsigned>(pic.frame_width_minus1));
    const unsigned height_bits_raw = std::bit_width(static_cast<unsigned>(pic.frame_height_minus1));
    const unsigned width_bits = width_bits_raw == 0 ? 1u : width_bits_raw;
    const unsigned height_bits = height_bits_raw == 0 ? 1u : height_bits_raw;
    const std::uint8_t bit_depth = (pic.bit_depth_idx == 0) ? 8 : (pic.bit_depth_idx == 1 ? 10 : 12);

    syntax::Av1SequenceHeaderSyntax syntax{};
    syntax.seq_profile = pic.profile & 0x7u;
    syntax.frame_width_bits_minus_1 = static_cast<std::uint32_t>(width_bits - 1u);
    syntax.frame_height_bits_minus_1 = static_cast<std::uint32_t>(height_bits - 1u);
    syntax.max_frame_width_minus_1 = pic.frame_width_minus1;
    syntax.max_frame_height_minus_1 = pic.frame_height_minus1;
    syntax.use_128x128_superblock = pic.seq_info_fields.fields.use_128x128_superblock;
    syntax.enable_filter_intra = pic.seq_info_fields.fields.enable_filter_intra;
    syntax.enable_intra_edge_filter = pic.seq_info_fields.fields.enable_intra_edge_filter;
    syntax.enable_interintra_compound = pic.seq_info_fields.fields.enable_interintra_compound;
    syntax.enable_masked_compound = pic.seq_info_fields.fields.enable_masked_compound;
    syntax.enable_dual_filter = pic.seq_info_fields.fields.enable_dual_filter;
    syntax.enable_order_hint = pic.seq_info_fields.fields.enable_order_hint;
    syntax.enable_jnt_comp = pic.seq_info_fields.fields.enable_jnt_comp;
    syntax.order_hint_bits_minus_1 = pic.order_hint_bits_minus_1 & 0x7u;
    syntax.use_superres = pic.pic_info_fields.bits.use_superres;
    syntax.enable_cdef = pic.seq_info_fields.fields.enable_cdef;
    syntax.enable_restoration = (pic.loop_restoration_fields.bits.yframe_restoration_type != 0) ||
                                (pic.loop_restoration_fields.bits.cbframe_restoration_type != 0) ||
                                (pic.loop_restoration_fields.bits.crframe_restoration_type != 0);
    syntax.high_bitdepth = bit_depth > 8 ? 1u : 0u;
    syntax.twelve_bit = (pic.profile == 2 && bit_depth == 12) ? 1u : 0u;
    syntax.mono_chrome = pic.seq_info_fields.fields.mono_chrome;
    syntax.matrix_coefficients = pic.matrix_coefficients;
    syntax.color_range = pic.seq_info_fields.fields.color_range;
    syntax.chroma_sample_position = pic.seq_info_fields.fields.chroma_sample_position & 0x3u;
    syntax.film_grain_params_present = pic.seq_info_fields.fields.film_grain_params_present;
    syntax.width_bits = width_bits;
    syntax.height_bits = height_bits;
    syntax.subsampling_x = (pic.profile == 0) ? true : (pic.seq_info_fields.fields.subsampling_x != 0);
    syntax.subsampling_y = (pic.profile == 0) ? true : (pic.seq_info_fields.fields.subsampling_y != 0);

    Serializer payload;
    payload.write(syntax);
    payload.byte_align();

    syntax::Av1ObuHeaderSyntax obu_header{};
    obu_header.obu_type = 1;
    Serializer header;
    header.write(obu_header);

    std::vector<std::uint8_t> obu;
    auto header_bytes = header.bytes_u8();
    obu.insert(obu.end(), header_bytes.begin(), header_bytes.end());
    append_leb128(obu, payload.bytes().size());
    auto payload_bytes = payload.bytes_u8();
    obu.insert(obu.end(), payload_bytes.begin(), payload_bytes.end());
    return obu;
}

inline void write_av1_frame_size(BitWriter& writer, const VADecPictureParameterBufferAV1& pic) {
    writer.write(detail::seq(detail::bits(pic.frame_width_minus1, 11),
                             detail::bits(pic.frame_height_minus1, 11)));
}

inline std::vector<std::uint8_t> build_av1_frame_obu(const VADecPictureParameterBufferAV1& pic,
                                                     std::span<const std::uint8_t> tile_payload,
                                                     std::uint8_t refresh_frame_flags,
                                                     bool sequence_enable_restoration) {
    auto map_restoration_type = [](std::uint8_t value) -> std::uint8_t {
        switch (value & 0x3u) {
            case 0: return 0;
            case 1: return 2;
            case 2: return 3;
            case 3: return 1;
            default: return 0;
        }
    };

    const bool frame_is_intra = (pic.pic_info_fields.bits.frame_type == 0 || pic.pic_info_fields.bits.frame_type == 2);
    const bool show_frame = pic.pic_info_fields.bits.show_frame != 0;
    const bool error_resilient_mode = pic.pic_info_fields.bits.error_resilient_mode != 0;
    const bool force_integer_mv = pic.pic_info_fields.bits.force_integer_mv != 0;
    const bool allow_intrabc = pic.pic_info_fields.bits.allow_intrabc != 0;
    const bool enable_order_hint = pic.seq_info_fields.fields.enable_order_hint != 0;
    const bool is_switch_frame = (pic.pic_info_fields.bits.frame_type & 0x3u) == 3;
    const bool is_key_show_frame = (pic.pic_info_fields.bits.frame_type & 0x3u) == 0 && show_frame;

    std::uint8_t y_restoration_type = pic.loop_restoration_fields.bits.yframe_restoration_type & 0x3u;
    std::uint8_t u_restoration_type = pic.loop_restoration_fields.bits.cbframe_restoration_type & 0x3u;
    std::uint8_t v_restoration_type = pic.loop_restoration_fields.bits.crframe_restoration_type & 0x3u;
    const bool likely_no_restoration = !frame_is_intra &&
                                       pic.filter_level[0] == 0 &&
                                       pic.filter_level[1] == 0 &&
                                       pic.filter_level_u == 0 &&
                                       pic.filter_level_v == 0 &&
                                       pic.cdef_bits == 0;
    if (likely_no_restoration) {
        y_restoration_type = 0;
        u_restoration_type = 0;
        v_restoration_type = 0;
    }

    syntax::Av1FrameHeaderSyntax syntax{};
    syntax.frame_type = pic.pic_info_fields.bits.frame_type & 0x3u;
    syntax.show_frame = show_frame ? 1u : 0u;
    syntax.showable_frame = pic.pic_info_fields.bits.showable_frame;
    syntax.error_resilient_mode = error_resilient_mode ? 1u : 0u;
    syntax.disable_cdf_update = pic.pic_info_fields.bits.disable_cdf_update;
    syntax.allow_screen_content_tools = pic.pic_info_fields.bits.allow_screen_content_tools;
    syntax.force_integer_mv = force_integer_mv ? 1u : 0u;
    syntax.order_hint = pic.order_hint;
    syntax.order_hint_bits = enable_order_hint ? static_cast<std::uint32_t>(pic.order_hint_bits_minus_1 + 1) : 0u;
    syntax.primary_ref_frame = pic.primary_ref_frame & 0x7u;
    syntax.refresh_frame_flags = refresh_frame_flags;
    syntax.allow_intrabc = allow_intrabc ? 1u : 0u;
    syntax.frame_refs_short_signaling = 0;
    for (std::size_t index = 0; index < syntax.ref_frame_idx.size(); ++index) {
        syntax.ref_frame_idx[index] = pic.ref_frame_idx[index] & 0x7u;
    }
    syntax.allow_high_precision_mv = pic.pic_info_fields.bits.allow_high_precision_mv;
    syntax.is_filter_switchable = (pic.interp_filter == 4) ? 1u : 0u;
    syntax.interpolation_filter = pic.interp_filter & 0x3u;
    syntax.is_motion_mode_switchable = pic.pic_info_fields.bits.is_motion_mode_switchable;
    syntax.use_ref_frame_mvs = pic.pic_info_fields.bits.use_ref_frame_mvs;
    syntax.disable_frame_end_update_cdf = pic.pic_info_fields.bits.disable_frame_end_update_cdf;
    syntax.uniform_tile_spacing_flag = pic.pic_info_fields.bits.uniform_tile_spacing_flag;
    syntax.base_qindex = pic.base_qindex;
    syntax.y_dc_delta_q = pic.y_dc_delta_q;
    syntax.u_dc_delta_q = pic.u_dc_delta_q;
    syntax.u_ac_delta_q = pic.u_ac_delta_q;
    syntax.using_qmatrix = pic.qmatrix_fields.bits.using_qmatrix;
    syntax.qm_y = pic.qmatrix_fields.bits.qm_y & 0xfu;
    syntax.qm_u = pic.qmatrix_fields.bits.qm_u & 0xfu;
    syntax.qm_v = pic.qmatrix_fields.bits.qm_v & 0xfu;
    syntax.segmentation_enabled = pic.seg_info.segment_info_fields.bits.enabled;
    syntax.segmentation_update_map = pic.seg_info.segment_info_fields.bits.update_map;
    syntax.segmentation_temporal_update = pic.seg_info.segment_info_fields.bits.temporal_update;
    syntax.segmentation_update_data = pic.seg_info.segment_info_fields.bits.update_data;
    syntax.delta_q_present_flag = pic.mode_control_fields.bits.delta_q_present_flag;
    syntax.log2_delta_q_res = pic.mode_control_fields.bits.log2_delta_q_res & 0x3u;
    syntax.delta_lf_present_flag = pic.mode_control_fields.bits.delta_lf_present_flag;
    syntax.log2_delta_lf_res = pic.mode_control_fields.bits.log2_delta_lf_res & 0x3u;
    syntax.delta_lf_multi = pic.mode_control_fields.bits.delta_lf_multi;
    syntax.loop_filter_level_y_vertical = pic.filter_level[0] & 0x3fu;
    syntax.loop_filter_level_y_horizontal = pic.filter_level[1] & 0x3fu;
    syntax.loop_filter_level_u = pic.filter_level_u & 0x3fu;
    syntax.loop_filter_level_v = pic.filter_level_v & 0x3fu;
    syntax.sharpness_level = pic.loop_filter_info_fields.bits.sharpness_level & 0x7u;
    syntax.mode_ref_delta_enabled = pic.loop_filter_info_fields.bits.mode_ref_delta_enabled;
    syntax.mode_ref_delta_update = pic.loop_filter_info_fields.bits.mode_ref_delta_update;
    for (std::size_t index = 0; index < syntax.ref_deltas.size(); ++index) {
        syntax.ref_deltas[index] = pic.ref_deltas[index];
    }
    for (std::size_t index = 0; index < syntax.mode_deltas.size(); ++index) {
        syntax.mode_deltas[index] = pic.mode_deltas[index];
    }
    syntax.cdef_damping_minus_3 = pic.cdef_damping_minus_3 & 0x3u;
    syntax.cdef_bits = pic.cdef_bits & 0x3u;
    for (std::size_t index = 0; index < syntax.cdef_y_strengths.size(); ++index) {
        syntax.cdef_y_strengths[index] = pic.cdef_y_strengths[index];
        syntax.cdef_uv_strengths[index] = pic.cdef_uv_strengths[index];
    }
    syntax.enable_restoration = sequence_enable_restoration ? 1u : 0u;
    syntax.lr_type_y = map_restoration_type(y_restoration_type);
    syntax.lr_type_u = map_restoration_type(u_restoration_type);
    syntax.lr_type_v = map_restoration_type(v_restoration_type);
    syntax.lr_unit_shift_value = pic.loop_restoration_fields.bits.lr_unit_shift & 0x3u;
    syntax.lr_unit_shift_min = pic.seq_info_fields.fields.use_128x128_superblock ? 1u : 0u;
    syntax.lr_unit_shift_max = 2u;
    syntax.lr_uv_shift = pic.loop_restoration_fields.bits.lr_uv_shift ? 1u : 0u;
    syntax.tx_mode_select = pic.mode_control_fields.bits.tx_mode == 2 ? 1u : 0u;
    syntax.reference_select = pic.mode_control_fields.bits.reference_select;
    syntax.skip_mode_present = pic.mode_control_fields.bits.skip_mode_present;
    syntax.write_skip_mode_present = pic.mode_control_fields.bits.skip_mode_present ? 1u : 0u;
    syntax.allow_warped_motion = pic.pic_info_fields.bits.allow_warped_motion;
    syntax.reduced_tx_set_used = pic.mode_control_fields.bits.reduced_tx_set_used;
    syntax.film_grain_params_present = pic.seq_info_fields.fields.film_grain_params_present;

    syntax.frame_is_intra = frame_is_intra;
    syntax.is_switch_frame = is_switch_frame;
    syntax.is_key_show_frame = is_key_show_frame;
    syntax.mono_chrome = pic.seq_info_fields.fields.mono_chrome != 0;
    syntax.include_showable_frame = !show_frame;
    syntax.write_error_resilient_mode = !(is_switch_frame || is_key_show_frame);
    syntax.write_primary_ref_frame = !(frame_is_intra || error_resilient_mode);
    syntax.write_refresh_frame_flags = !(is_switch_frame || is_key_show_frame);
    syntax.write_ref_frame_signaling = !frame_is_intra;
    syntax.write_allow_high_precision_mv = !force_integer_mv;
    syntax.write_use_ref_frame_mvs = !error_resilient_mode && enable_order_hint;
    syntax.write_disable_frame_end_update_cdf = !pic.pic_info_fields.bits.disable_cdf_update;
    syntax.write_delta_q_u = !pic.seq_info_fields.fields.mono_chrome;
    syntax.write_delta_q_lf = !allow_intrabc;
    syntax.write_chroma_loop_filter_levels = (pic.filter_level[0] || pic.filter_level[1]);
    syntax.write_cdef = pic.seq_info_fields.fields.enable_cdef && !allow_intrabc;
    syntax.write_restoration = sequence_enable_restoration && !allow_intrabc;
    syntax.write_lr_unit_shift = syntax.write_restoration && (y_restoration_type != 0 || u_restoration_type != 0 || v_restoration_type != 0);
    syntax.write_lr_uv_shift = syntax.write_restoration && (u_restoration_type != 0 || v_restoration_type != 0);
    syntax.write_tx_mode = !allow_intrabc;
    syntax.write_inter_mode_bits = !frame_is_intra;
    for (auto& is_identity : syntax.global_motion_is_identity) {
        is_identity = 0;
    }

    Serializer payload;
    payload.write(syntax);
    payload.zero_align();

    syntax::Av1ObuHeaderSyntax obu_header{};
    obu_header.obu_type = 6;
    Serializer header;
    header.write(obu_header);

    std::vector<std::uint8_t> obu;
    auto header_bytes = header.bytes_u8();
    obu.insert(obu.end(), header_bytes.begin(), header_bytes.end());
    append_leb128(obu, payload.bytes().size() + tile_payload.size());
    auto payload_bytes = payload.bytes_u8();
    obu.insert(obu.end(), payload_bytes.begin(), payload_bytes.end());
    obu.insert(obu.end(), tile_payload.begin(), tile_payload.end());
    return obu;
}

} // namespace rockchip::bitstream