/*
 * hal_caps.c -- Per-SoC capability struct initialization
 *
 * Provides a compile-time-constant rss_hal_caps_t for the target SoC.
 * Exactly one PLATFORM_* macro is defined by the build system; the
 * corresponding capability block is compiled and all others are excluded.
 *
 * Values are derived from the SDK difference analysis across all 10
 * supported Ingenic SoCs (T10, T20, T21, T23, T30, T31, T32, T33, T40, T41),
 * plus the SigmaStar Infinity6 families at the end of the chain.
 */

#include "hal_internal.h"
#include "raptor_hal.h"

/* ═══════════════════════════════════════════════════════════════════════
 * T20
 * ═══════════════════════════════════════════════════════════════════════ */
#if defined(PLATFORM_T20)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = false,
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .jpeg_pulse = true,
    .has_encoder_pool = false,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = true,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* T20 has SetRawDRC, not SetDRC_Strength */
    .has_face_ae = false,
    .has_bcsh_hue = false,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = false,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = false,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = false,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = false,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 2,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T21
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T21)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = false, /* declared but marked "Unsupport" */
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .jpeg_pulse = true,
    .has_encoder_pool = false,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = true,
    .has_color2grey = true,
    .has_roi = true,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = true,
    .has_qpg_ai = false,
    .has_mbrc = true,
    .has_enc_denoise = true,
    .has_gdr = false,
    .has_sei_userdata = true,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = true,
    .has_h265_trans = true,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = true,
    .has_resize_mode = false,
    .has_jpeg_ql = true,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = true,
    .has_face_ae = false,
    .has_bcsh_hue = false,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = false,
    .has_ae_comp = false, /* SetAeComp absent on T21 */
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = true,
    .has_wdr = false,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = false,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = true,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = false,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 2,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T23
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T23)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = false, /* declared but marked "Unsupport" */
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .has_encoder_pool = true,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = true, /* SDK 1.3.0 MultiCamera API */
    .max_sensors = 3,
    .has_t23_multicam_api = true,
    .has_defog = true,
    .has_dpc = true,
    .has_drc = true,
    .has_face_ae = false,
    .has_bcsh_hue = true,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = true,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = true,
    .has_wdr = false,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 6, /* vendor dual-sensor sample uses 6 */
    .max_osd_regions = 16,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T30
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T30)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = false,
    .has_bufshare = false,
    .has_set_default_param = false,
    .has_capped_rc = false,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = false,
    .has_stream_buf_size = false,
    .jpeg_pulse = true,
    .has_encoder_pool = false,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* T30 has SetRawDRC, not SetDRC_Strength */
    .has_face_ae = false,
    .has_bcsh_hue = false,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = false,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = false,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = false,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = false,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 2,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T31
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T31)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = true,
    .has_i2d = false,
    .has_bufshare = true,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = false,
    .has_gop_attr = true,
    .has_set_bitrate = true,
    .has_stream_buf_size = true,
    .has_encoder_pool = true,
    .has_smartp_gop = true,
    .has_rc_options = true,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = true,
    .has_poll_module = true,
    .has_resize_mode = true,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = false,
    .max_sensors = 1,
    .has_t23_multicam_api = false,
    .has_defog = true,
    .has_dpc = true,
    .has_drc = true,
    .has_face_ae = false,
    .has_bcsh_hue = true,
    .has_sinter = true,
    .has_temper = true,
    .has_highlight_depress = true,
    .has_backlight_comp = true,
    .has_ae_comp = true,
    .has_max_gain = true,
    .has_switch_bin = false,
    .has_gamma = true,
    .has_gamma_attr = false,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = false,
    .has_osd_mosaic = false,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = false,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = false,
    .has_alc_gain = true,
    .has_agc_mode = true,
    .has_digital_gain = false,
    .has_howling_suppress = false,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = true,
    .uses_impvi = false,
    /* Limits */
    .max_enc_channels = 3,
    .max_osd_regions = 8,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 0,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T32
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T32)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = false, /* SetbufshareChn not present on T32 */
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = true,  /* T32 has ENC_RC_MODE_SMART (mode 3) */
    .has_gop_attr = false, /* T32 sets GOP via SetDefaultParam arg */
    .has_set_bitrate = true,
    .has_stream_buf_size = false,
    .has_encoder_pool = true,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = true,
    .has_srd = true,
    .has_max_pic_size = true,
    .has_super_frame = true,
    .has_color2grey = false,
    .has_roi = true,
    .has_map_roi = true,
    .has_qp_bounds_per_frame = true,
    .has_qpg_mode = true,
    .has_qpg_ai = true,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = true,
    .has_sei_userdata = true,
    .has_h264_vui = true,
    .has_h265_vui = true,
    .has_h264_trans = true,
    .has_h265_trans = true,
    .has_enc_crop = true,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = true,
    /* ISP */
    .has_multi_sensor = true, /* IMPVI_NUM, up to 4 sensors */
    .max_sensors = 3,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* via SetModuleControl only */
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false, /* via SetModuleControl only */
    .has_temper = false, /* via SetModuleControl only */
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 3,
    .max_osd_regions = 16,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T33 (T32-compatible, no DMIC/DVP/WDR)
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T33)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = false,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = true,
    .has_gop_attr = false,
    .has_set_bitrate = true,
    .has_stream_buf_size = false,
    .has_encoder_pool = true,
    .has_smartp_gop = false,
    .has_rc_options = false,
    .has_pskip = true,
    .has_srd = true,
    .has_max_pic_size = true,
    .has_super_frame = true,
    .has_color2grey = false,
    .has_roi = true,
    .has_map_roi = true,
    .has_qp_bounds_per_frame = true,
    .has_qpg_mode = true,
    .has_qpg_ai = true,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = true,
    .has_sei_userdata = true,
    .has_h264_vui = true,
    .has_h265_vui = true,
    .has_h264_trans = true,
    .has_h265_trans = true,
    .has_enc_crop = true,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = true,
    /* ISP */
    .has_multi_sensor = true,
    .max_sensors = 3,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false,
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false,
    .has_temper = false,
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = false,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = false,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = false,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 3,
    .max_osd_regions = 16,
    .max_osd_groups = 2,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T40
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T40)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = true,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = false,
    .has_gop_attr = true,
    .has_set_bitrate = true,
    .has_stream_buf_size = true,
    .has_encoder_pool = true,
    .has_smartp_gop = true,
    .has_rc_options = true,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = false,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = false,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = false,
    .has_resize_mode = false,
    .has_jpeg_ql = false,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = true,
    .max_sensors = 3,
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* via SetModuleControl only */
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false, /* via SetModuleControl only */
    .has_temper = false, /* via SetModuleControl only */
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = true,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = true,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 4,
    .max_osd_regions = 16,
    .max_osd_groups = 4,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * T41
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_T41)
const rss_hal_caps_t g_hal_caps = {
    /* Encoder */
    .has_h265 = true,
    .has_rotation = false,
    .has_i2d = true,
    .has_bufshare = true,
    .has_set_default_param = true,
    .has_capped_rc = true,
    .has_smart_rc = false,
    .has_gop_attr = true,
    .has_set_bitrate = true,
    .has_stream_buf_size = true,
    .has_encoder_pool = true,
    .has_smartp_gop = true,
    .has_rc_options = true,
    .has_pskip = false,
    .has_srd = false,
    .has_max_pic_size = true,
    .has_super_frame = false,
    .has_color2grey = false,
    .has_roi = false,
    .has_map_roi = false,
    .has_qp_bounds_per_frame = true,
    .has_qpg_mode = false,
    .has_qpg_ai = false,
    .has_mbrc = false,
    .has_enc_denoise = false,
    .has_gdr = false,
    .has_sei_userdata = false,
    .has_h264_vui = false,
    .has_h265_vui = false,
    .has_h264_trans = false,
    .has_h265_trans = false,
    .has_enc_crop = false,
    .has_eval_info = false,
    .has_poll_module = true,
    .has_resize_mode = true,
    .has_jpeg_ql = true,
    .has_jpeg_qp = false,
    /* ISP */
    .has_multi_sensor = true, /* IMPVI_NUM defined but SEC/THR unsupported */
    .max_sensors = 3,         /* experimental — vendor docs say SEC/THR not yet functional */
    .has_t23_multicam_api = false,
    .has_defog = false,
    .has_dpc = false,
    .has_drc = false, /* via SetModuleControl only */
    .has_face_ae = true,
    .has_bcsh_hue = true,
    .has_sinter = false, /* via SetModuleControl only */
    .has_temper = false, /* via SetModuleControl only */
    .has_highlight_depress = false,
    .has_backlight_comp = false,
    .has_ae_comp = false,
    .has_max_gain = false,
    .has_switch_bin = true,
    .has_gamma = false,
    .has_gamma_attr = true,
    .has_module_control = true,
    .has_wdr = true,
    /* OSD */
    .has_isp_osd = true,
    .has_osd_mosaic = true,
    .has_osd_group_callback = true,
    .has_osd_region_invert = true,
    .has_extended_osd_types = true,
    /* Audio */
    .has_audio_process_lib = true,
    .has_audio_aec_channel = true,
    .has_alc_gain = false,
    .has_agc_mode = false,
    .has_digital_gain = true,
    .has_howling_suppress = true,
    .has_hpf_cutoff = true,
    /* System */
    .uses_xburst2 = true,
    .uses_new_sdk = true,
    .uses_impvi = true,
    /* Limits */
    .max_enc_channels = 4,
    .max_osd_regions = 16,
    .max_osd_groups = 4,
    .max_isp_osd_regions = 8,
};

/* ═══════════════════════════════════════════════════════════════════════
 * INFINITY6E (SigmaStar SSC30KQ / SSC338Q)
 * INFINITY6B0 (SigmaStar SSC333 / SSC335 / SSC337)
 *
 * One block for both families. src/star/ is written against the MI ABI
 * vendored in src/star/i6_*.h, which spans the Infinity6 series -- divinus
 * drives infinity6, infinity6e and infinity6b0 through that same i6 HAL and
 * only switches implementation at infinity6c. So the two share every
 * capability here except where marked.
 *
 * Two kinds of false appear below, and the distinction matters:
 *
 *   1. Hardware/SDK facts — the MI SDK genuinely has no equivalent.
 *      These stay false permanently and are commented individually.
 *   2. Not-yet-implemented — the capability may well exist, but the
 *      corresponding hal_* op is not wired up yet. Consumers check these
 *      flags before calling optional ops, so declaring false keeps
 *      rvd/rsd from invoking a NULL vtable entry. Each later phase flips
 *      on the flags it implements (ISP -> phase 3, audio -> 4, OSD -> 5).
 *
 * Since the backend currently populates no video/audio ops at all,
 * everything in category 2 is false. Only fields with a positive value
 * or a permanent-false explanation are listed; the rest default to
 * false/0 via designated initialization.
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_INFINITY6E) || defined(PLATFORM_INFINITY6B0)
const rss_hal_caps_t g_hal_caps = {
    /* System info */
    .soc_name = HAL_PLATFORM_NAME,
    /* Real value comes from MI_SYS_GetVersion() at runtime (phase 2);
     * this is only the compile-time fallback string. */
    .sdk_version = "MI",

    /* Encoder — H.264/H.265/MJPEG are all present in hardware
     * (i6_venc_codec: I6_VENC_CODEC_H264/H265/MJPG). */
    .has_h265 = true,
    /* Permanent false: MI rate control offers CBR/VBR/ABR/FIXQP/AVBR
     * (i6_venc_ratemode) with no equivalent of Ingenic's SMART mode. */
    .has_smart_rc = false,

    /* ISP — single sensor for now; MI supports multi-sensor on some parts
     * but raptor's multi-sensor path is Ingenic IMPVI-specific. */
    .has_multi_sensor = false,
    .max_sensors = 1,
    /* Permanent false: T23-specific IMP_ISP_MultiCamera_* API. */
    .has_t23_multicam_api = false,

    /*
     * ISP tuning — phase 3. True only where MI has a control that
     * genuinely matches the raptor knob:
     *
     *   defog          MI_ISP_IQ_SetDefog, a toggle. isp_set_defog is
     *                  implemented; the *strength* variants are not,
     *                  because there is no strength to set.
     *   sinter/temper  MI's spatial (NRLuma) and temporal (NR3D) noise
     *                  reduction, both 0..255 as raptor expects.
     *   ae_comp        MI_ISP_AE_SetEVComp.
     *   max_gain       Both ceilings live in MI's AE exposure-limit
     *                  struct.
     *
     * Left false, with the reasoning spelled out in hal_isp.c's OP
     * COVERAGE comment: dpc and drc (MI's are a toggle and a curve, not
     * strengths), bcsh_hue (a 64-entry HSV LUT), highlight_depress and
     * backlight_comp (WDR curve descriptors), and switch_bin -- a tuning
     * binary *is* loaded during hal_init, but Ingenic's runtime
     * bin-switching op has no MI counterpart.
     *
     * INFINITY6B0 leaves all five false, and this is the one place the two
     * families genuinely diverge. Every scalar knob is reached by poking a
     * field at a hardcoded (payload size, manual offset) pair in an opaque
     * MI IQ struct -- see g_iq in src/star/hal_isp.c. Those numbers were
     * read out of an INFINITY6E libmi_isp.so with objdump, and they are
     * properties of that library build, not of the MI API. Nothing else
     * here carries that risk: the rest of the backend calls typed entry
     * points whose ABI divinus already exercises on infinity6b0, whereas
     * the IQ pokes are raptor's alone and divinus has no equivalent to
     * corroborate.
     *
     * A wrong offset does not fail. Access is read-modify-write, so it
     * lands a plausible value in the wrong field of a struct nobody has
     * fully described, and the image quietly degrades with nothing naming
     * the cause. False here means isp_set_* returns RSS_ERR_NOTSUP and rvd
     * leaves the tuning binary in charge -- which is the correct image
     * either way, just not an adjustable one.
     *
     * Deriving them needs no board, only the library:
     *   arm-linux-gnueabihf-objdump -d \
     *       --disassemble=MI_ISP_IQ_GetBrightness libmi_isp.so
     * prints the payload size into the size slot. If the five offsets
     * match INFINITY6E's, this becomes a shared block again.
     */
#if defined(PLATFORM_INFINITY6E)
    .has_defog = true,
    .has_sinter = true,
    .has_temper = true,
    .has_ae_comp = true,
    .has_max_gain = true,
#endif

    /*
     * Audio — every audio cap stays false, and phase 4 does not change
     * that even though capture itself works.
     *
     * has_audio_process_lib, has_agc_mode, has_hpf_cutoff,
     * has_howling_suppress and has_audio_aec_channel all describe MI's
     * VQE features (noise reduction, AGC, high-pass, echo cancellation).
     * All false, and not out of caution -- turning any of them on is a
     * crash:
     *
     *   1. None of the 20 libraries OpenIPC ships for infinity6e defines
     *      a single Iaa* symbol. The whole algorithm surface --
     *      IaaApc_* (the NS/AGC/EQ chain), IaaAec_*, IaaSsl_*, IaaBf_*,
     *      and IaaSrc_* (the resampler, which is why capture rates are
     *      gated) -- is weak-undefined in libmi_ai.so with no provider.
     *   2. The MI_AI_*Vqe* wrappers themselves *are* defined, so nothing
     *      fails at dlopen or at symbol load. MI_AI_EnableVqe is mostly
     *      argument validation and logging.
     *   3. The real call sites (_MI_AI_G726Init and the capture path)
     *      reach IaaApc_GetBufferSize/Init/Config/Run through the PLT
     *      with **no null guard**. An unresolved weak symbol's GOT slot
     *      is 0, so the blx jumps to address 0 and takes the process
     *      down.
     *
     * MI's own reference agrees the packs are gone: the API "no longer
     * includes the associated algorithm functions" from version 2.19.
     * Doing any of this in software instead would belong in rad, not
     * here, and is explicitly out of scope.
     *
     * has_alc_gain and has_digital_gain describe two separate gain stages;
     * MI has one input gain control, and audio_set_volume owns it.
     * See hal_audio.c's OP COVERAGE comment.
     */

    /* System — all three describe Ingenic internals (xburst2 core, IMP SDK
     * generation, IMPVI multi-sensor calling convention) and are
     * permanently false for any non-Ingenic vendor. */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,

    /* Limits — the Infinity6 family exposes 9 addressable VENC channels
     * (I6_VENC_CHN_NUM in both divinus's i6_venc.h and waybeam's
     * sigmastar_types.h; divinus allocates channel state for all 9 and
     * iterates them, for every family it drives through the i6 HAL).
     * An earlier 3 here came from misreading
     * MI_VENC_MAX_CHN_NUM_PER_DC, which is the per-device-group limit and
     * not the total. Capped at RSS_MAX_ENC_CHANNELS (8) since that is
     * raptor's array bound, so 8 is the most this field can honestly
     * advertise.
     *
     * OSD limits (phase 5): max_osd_regions is STAR_OSD_REGION_MAX, the
     * backend's own tracking bound rather than an MI limit -- MI
     * publishes none and neither reference probes for one, so this
     * advertises what the backend will actually honour. max_osd_groups is
     * 4 because a group here *is* an encoder channel with a bound VPE
     * port, and there are four ports. The has_osd_* flags all stay false:
     * MI has only OSD and COVER region types (no mosaic, no
     * RSS_OSD_PIC_RMEM), no group callback, and while MI's display attr
     * does carry an invert sub-struct, rss_osd_region_t has no field to
     * drive it, so claiming the capability would promise something raptor
     * cannot ask for.
     *
     * max_fs_channels is 4 because a raptor framesource channel is a VPE
     * output port (src/star/hal_framesource.c) and a VPE channel has four
     * of them: divinus's teardown disables ports 0..3 (i6_hal.c:365) and
     * waybeam uses 0 and 1. Measured on an SSC30KQ: all four ports accept
     * MI_VPE_SetPortMode at 640x360 NV12. The port count is a VPE property
     * rather than a per-family one, so INFINITY6B0 inherits it unmeasured. */
    .max_enc_channels = 8,
    .max_fs_channels = 4,
    /* Keep in step with STAR_OSD_REGION_MAX in src/star/star_state.h.
     * Not referenced symbolically because this file is compiled for every
     * platform and must not pull in the MI headers. */
    .max_osd_regions = 16,
    .max_osd_groups = 4,
};

/* ═══════════════════════════════════════════════════════════════════════
 * INFINITY6C (SigmaStar SSC377 / SSC378 / SSC379)
 *
 * Deliberately its own block rather than an arm of the one above. The two
 * share silicon lineage but not an ABI: MI 3.0 gives MI_SYS and MI_RGN a
 * leading SoC id, MI_VENC a leading device, promotes the ISP to a pipeline
 * stage and moves scaling to SCL. src/infinity6c/ is the backend.
 *
 * The capture and encode path is implemented (SNR, VIF, ISP, SCL, VENC);
 * OSD and audio are not, so their capabilities stay false and their
 * limits zero. Consumers check these flags precisely so they do not call
 * into a vtable slot that is NULL.
 *
 * Both channel limits are 4, and both are the *same* four: a raptor
 * framesource channel is an SCL output port, and an encoder channel is fed
 * by one, so a stream costs one port and the scaler's port count caps
 * both. That number is derived rather than carried over from Infinity6E --
 * mi_scl.ko answers it outright, since _MI_SCL_IMPL_GetPassOutputPortNum
 * is a four-byte function returning 4 (its input counterpart returns 1).
 * The ISP's equivalents are 1 in and 3 out, which is why the ISP sits
 * upstream of the fan-out rather than being it.
 *
 * max_enc_channels is therefore 4 and not the 8 or 12 VENC would accept.
 * MI addresses 12 channels across two devices (H.26x from 0, MJPEG from 8),
 * but a channel with nothing bound to it encodes nothing, so advertising
 * more than the scaler can feed would promise streams that fail at bind
 * time. Note also that MI_VENC_MAX_CHN_NUM_PER_DC is 3 on this part as on
 * Infinity6E and is still the per-device-group limit rather than a total;
 * it was misread that way once already, and MI 3.0's explicit device layer
 * makes it look more like a total than it is.
 *
 * has_rotation is true but coarser than the flag suggests: rotation lives
 * on the SCL channel, so it turns every stream at once. hal_fs_set_rotation
 * accepts it for channel 0 and refuses it elsewhere rather than quietly
 * rotating streams the caller did not name.
 *
 * has_color2grey is true because MI_ISP_IQ_SetColorToGray is bound and is a
 * property of the ISP channel rather than of an encoder.
 *
 * The permanent falses are the same three as above and for the same
 * reason -- xburst2, the IMP SDK generation and the IMPVI calling
 * convention are Ingenic internals.
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_INFINITY6C)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    /* Replaced at runtime from MI_SYS_GetVersion; this is the fallback. */
    .sdk_version = "MI",

    /* Encoder */
    .has_h265 = true,
    .has_rotation = true,
    .has_set_bitrate = true,
    .has_gop_attr = true,
    /* AVBR is what the capped and smart modes map onto; see hal_encoder.c. */
    .has_capped_rc = true,
    .has_smart_rc = true,

    /* ISP */
    .has_color2grey = true,
    .max_sensors = 1,

    /* Limits — both counts are the scaler's four output ports. */
    .max_enc_channels = 4,
    .max_fs_channels = 4,

    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
};

#else
#error "No PLATFORM_* defined. Set one of: PLATFORM_T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 INFINITY6E INFINITY6B0 INFINITY6C"
#endif
