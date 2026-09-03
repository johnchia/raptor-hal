/*
 * hisi_v4/v4_isp_tune.h -- the ISP module-attribute structs the IQ tuning
 * loader writes, and the entry points that carry them.
 *
 * Phase 3 scope: these are the structs behind the HI_MPI_ISP_{Get,Set}*
 * pairs that hal_isp.c drives when it applies /etc/sensors/iq/<sensor>.ini.
 * Nothing else -- the WDR/AWB/CCM/CAC attribute families the fleet's other
 * INIs mention stay untranscribed until a section that needs them is
 * actually applied, because every struct here is a byte-exact ABI promise
 * and an unused promise is pure risk.
 *
 * PROVENANCE. Layouts derived from the Hi3516EV200 SDK V1.0.1.0 headers
 * (mpp/include/hi_comm_isp.h) and cross-checked against
 * ref/openhisilicon/include/comm_isp.h (GPL v3) -- every array dimension
 * that determines a layout (ISP_AUTO_ISO_STRENGTH_NUM 16, AE_ZONE 15x17,
 * SHARPEN_LUMA/GAIN_NUM 32, DRC_TM_NODE_NUM 200, GAMMA_NODE_NUM 1025,
 * BAYERNR_LUT_LENGTH 33) agrees between the two. Sizes and offsets in the
 * _Static_asserts below were read from a probe object compiled against the
 * SDK headers with an ARM EABI cross-compiler (the mechanical-offset-table
 * step the plan prescribes); the asserts keep the transcription honest, the
 * probe never enters the build.
 *
 * One struct is deliberately partial: v4_isp_stat_cfg transcribes the key
 * and the AE half (all this backend writes) and carries the WB/AF tail as
 * opaque bytes, sized so the Get/Set round-trip moves the whole 624 bytes.
 * The tail's content passes through untouched, which is the point of the
 * get-modify-set pattern every user of this header follows.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_ISP_TUNE_H
#define HISI_V4_ISP_TUNE_H

#include "v4_common.h"

#define V4_ISP_ISO_NUM 16       /* ISP_AUTO_ISO_STRENGTH_NUM */
#define V4_ISP_AE_ROUTE_NODES 16 /* ISP_AE_ROUTE_EX_MAX_NODES */
#define V4_ISP_AE_ROWS 15       /* AE_ZONE_ROW */
#define V4_ISP_AE_COLS 17       /* AE_ZONE_COLUMN */
#define V4_ISP_SHARPEN_LUMA 32  /* ISP_SHARPEN_LUMA_NUM */
#define V4_ISP_SHARPEN_GAIN 32  /* ISP_SHARPEN_GAIN_NUM */
#define V4_ISP_DRC_TM_NODES 200 /* HI_ISP_DRC_TM_NODE_NUM */
#define V4_ISP_DRC_CC_NODES 33  /* HI_ISP_DRC_CC_NODE_NUM */
#define V4_ISP_GAMMA_NODES 1025 /* GAMMA_NODE_NUM */
#define V4_ISP_BAYERNR_LUT 33   /* HI_ISP_BAYERNR_LUT_LENGTH */
#define V4_ISP_BAYER_CHN 4      /* ISP_BAYER_CHN_NUM */
#define V4_ISP_DEHAZE_LUT 256

/* ISP_OP_TYPE_E: 0 auto, 1 manual. */
#define V4_ISP_OP_AUTO 0
#define V4_ISP_OP_MANUAL 1

/* ================================================================
 * EXPOSURE -- ISP_EXPOSURE_ATTR_S (lib_hiae.so)
 * ================================================================ */

typedef struct {
    unsigned int max, min; /* u32Max before u32Min, as the SDK has it */
} v4_isp_ae_range;

typedef struct {
    int enable;
    unsigned char frequency; /* 50 or 60 */
    int mode;
} v4_isp_antiflicker;

typedef struct {
    int enable;
    unsigned char luma_diff;
} v4_isp_subflicker;

/* ISP_ME_ATTR_S -- manual exposure. Untouched by the loader; present so
 * the auto half lands at the right offset. */
typedef struct {
    int exp_time_op, again_op, dgain_op, ispdgain_op;
    unsigned int exp_time, again, dgain, ispdgain;
} v4_isp_ae_manual;

/* ISP_AE_ATTR_S */
typedef struct {
    v4_isp_ae_range exp_time_range;
    v4_isp_ae_range again_range;
    v4_isp_ae_range dgain_range;
    v4_isp_ae_range ispdgain_range;
    v4_isp_ae_range sysgain_range;
    unsigned int gain_threshold;
    unsigned char speed;
    unsigned short black_speed_bias;
    unsigned char tolerance;
    unsigned char compensation;
    unsigned short ev_bias;
    int strategy_mode; /* ISP_AE_STRATEGY_E */
    unsigned short hist_ratio_slope;
    unsigned char max_hist_offset;
    int ae_mode; /* ISP_AE_MODE_E */
    v4_isp_antiflicker antiflicker;
    v4_isp_subflicker subflicker;
    unsigned short black_delay_frame; /* ISP_AE_DELAY_S, inlined */
    unsigned short white_delay_frame;
    int manual_exp_value_en;
    unsigned int exp_value;
    int fswdr_mode;
    int wdr_quick;
    unsigned short iso_cal_coef;
} v4_isp_ae_auto;

/* ISP_EXPOSURE_ATTR_S */
typedef struct {
    int bypass;
    int op_type;
    unsigned char run_interval;
    int hist_stat_adjust;
    int route_ex_valid;
    v4_isp_ae_manual manual;
    v4_isp_ae_auto auto_attr;
    int prior_frame;
    int gain_sep_cfg;
} v4_isp_exp_attr;

_Static_assert(sizeof(v4_isp_exp_attr) == 168, "ISP_EXPOSURE_ATTR_S is 168 bytes");
_Static_assert(offsetof(v4_isp_exp_attr, route_ex_valid) == 16, "bAERouteExValid at +16");
_Static_assert(offsetof(v4_isp_exp_attr, auto_attr) == 52, "stAuto at +52");
_Static_assert(offsetof(v4_isp_exp_attr, auto_attr.speed) == 96, "u8Speed at +96");
_Static_assert(offsetof(v4_isp_exp_attr, auto_attr.tolerance) == 100, "u8Tolerance at +100");
_Static_assert(offsetof(v4_isp_exp_attr, auto_attr.hist_ratio_slope) == 108,
               "u16HistRatioSlope at +108");
_Static_assert(offsetof(v4_isp_exp_attr, auto_attr.max_hist_offset) == 110,
               "u8MaxHistOffset at +110");
_Static_assert(offsetof(v4_isp_exp_attr, auto_attr.black_delay_frame) == 136,
               "u16BlackDelayFrame at +136");
_Static_assert(offsetof(v4_isp_exp_attr, auto_attr.iso_cal_coef) == 156, "u16ISOCalCoef at +156");
_Static_assert(offsetof(v4_isp_exp_attr, prior_frame) == 160, "enPriorFrame at +160");

/* ================================================================
 * AE ROUTE EX -- ISP_AE_ROUTE_EX_S (lib_hiae.so)
 * ================================================================ */

typedef struct {
    unsigned int int_time;  /* us */
    unsigned int again;     /* 10-bit precision, 1024 = 1x */
    unsigned int dgain;
    unsigned int isp_dgain;
    int iris_fno;
    unsigned int iris_fno_lin;
} v4_isp_ae_route_node;

typedef struct {
    unsigned int total_num;
    v4_isp_ae_route_node node[V4_ISP_AE_ROUTE_NODES];
} v4_isp_ae_route_ex;

_Static_assert(sizeof(v4_isp_ae_route_ex) == 388, "ISP_AE_ROUTE_EX_S is 388 bytes");
_Static_assert(sizeof(v4_isp_ae_route_node) == 24, "route node is 24 bytes");

/* ================================================================
 * STATISTICS -- ISP_STATISTICS_CFG_S (libisp.so), AE half + opaque tail
 * ================================================================ */

typedef struct {
    unsigned long long key; /* ISP_STATISTICS_CTRL_U -- a u64 of enable bits */
    /* stAECfg (ISP_AE_STATISTICS_CFG_S) at +8 */
    int ae_switch;
    int hist_skip_x, hist_skip_y, hist_offset_x, hist_offset_y; /* stHistConfig */
    int four_plane_mode;
    int hist_mode;
    int aver_mode;
    int max_gain_mode;
    int crop_enable; /* stCrop */
    unsigned short crop_x, crop_y, crop_w, crop_h;
    unsigned char weight[V4_ISP_AE_ROWS][V4_ISP_AE_COLS];
    /* stWBCfg at +312, stFocusCfg at +356 -- carried, never written. The
     * tail starts at +311 (the byte the SDK spends on padding after the
     * weight table) and runs to the end, so the struct has no trailing
     * padding of its own and every byte of a Get survives the Set. */
    unsigned char wb_af_tail[624 - 311];
} v4_isp_stat_cfg;

_Static_assert(sizeof(v4_isp_stat_cfg) == 624, "ISP_STATISTICS_CFG_S is 624 bytes");
_Static_assert(offsetof(v4_isp_stat_cfg, ae_switch) == 8, "stAECfg at +8");
_Static_assert(offsetof(v4_isp_stat_cfg, weight) == 56, "au8Weight at +56");
_Static_assert(offsetof(v4_isp_stat_cfg, wb_af_tail) == 311, "tail covers +311..624");

/* ================================================================
 * LDCI -- ISP_LDCI_ATTR_S (libisp.so / lib_hildci.so registered)
 * ================================================================ */

typedef struct {
    unsigned char wgt, sigma, mean;
} v4_isp_ldci_gauss;

typedef struct {
    v4_isp_ldci_gauss pos, neg;
} v4_isp_ldci_he_wgt;

typedef struct {
    int enable;
    unsigned char gauss_lpf_sigma;
    int op_type;
    v4_isp_ldci_he_wgt manual_he;
    unsigned short manual_blc_ctrl;
    v4_isp_ldci_he_wgt auto_he[V4_ISP_ISO_NUM];
    unsigned short auto_blc_ctrl[V4_ISP_ISO_NUM];
    unsigned short tpr_incr_coef;
    unsigned short tpr_decr_coef;
} v4_isp_ldci_attr;

_Static_assert(sizeof(v4_isp_ldci_attr) == 152, "ISP_LDCI_ATTR_S is 152 bytes");
_Static_assert(offsetof(v4_isp_ldci_attr, auto_he) == 20, "stAuto at +20");
_Static_assert(offsetof(v4_isp_ldci_attr, auto_he[1]) == 26, "He weight stride is 6");
_Static_assert(offsetof(v4_isp_ldci_attr, auto_blc_ctrl) == 116, "au16BlcCtrl at +116");
_Static_assert(offsetof(v4_isp_ldci_attr, tpr_incr_coef) == 148, "u16TprIncrCoef at +148");

/* ================================================================
 * DRC -- ISP_DRC_ATTR_S (libisp.so / lib_hidrc.so registered)
 * ================================================================ */

typedef struct {
    unsigned short x, y, slope;
} v4_isp_drc_cubic_point;

typedef struct {
    unsigned char asymmetry, second_pole, stretch, compress;
} v4_isp_drc_asym_curve;

typedef struct {
    int enable;
    int curve_select; /* 0 asymmetry, 2 user */
    unsigned char pd_strength;
    unsigned char local_mixing_bright_max;
    unsigned char local_mixing_bright_min;
    unsigned char local_mixing_bright_thr;
    signed char local_mixing_bright_slo;
    unsigned char local_mixing_dark_max;
    unsigned char local_mixing_dark_min;
    unsigned char local_mixing_dark_thr;
    signed char local_mixing_dark_slo;
    unsigned char detail_bright_str;
    unsigned char detail_dark_str;
    unsigned char detail_bright_step;
    unsigned char detail_dark_step;
    unsigned char bright_gain_lmt;
    unsigned char bright_gain_lmt_step;
    unsigned char dark_gain_lmt_y;
    unsigned char dark_gain_lmt_c;
    unsigned short cc_lut[V4_ISP_DRC_CC_NODES];
    unsigned short tone_mapping[V4_ISP_DRC_TM_NODES];
    unsigned char flt_scale_coarse;
    unsigned char flt_scale_fine;
    unsigned char contrast_control;
    signed char detail_adjust_factor;
    unsigned char spatial_flt_coef;
    unsigned char range_flt_coef;
    unsigned char range_ada_max;
    unsigned char grad_rev_max;
    unsigned char grad_rev_thr;
    unsigned char dp_detect_range_ratio;
    unsigned char dp_detect_thr_slo;
    unsigned short dp_detect_thr_min;
    int op_type;
    unsigned short manual_strength;      /* stManual */
    unsigned short auto_strength;        /* stAuto */
    unsigned short auto_strength_max;
    unsigned short auto_strength_min;
    v4_isp_drc_cubic_point cubic_point[5];
    v4_isp_drc_asym_curve asym;
} v4_isp_drc_attr;

_Static_assert(sizeof(v4_isp_drc_attr) == 556, "ISP_DRC_ATTR_S is 556 bytes");
_Static_assert(offsetof(v4_isp_drc_attr, cc_lut) == 26, "au16ColorCorrectionLut at +26");
_Static_assert(offsetof(v4_isp_drc_attr, tone_mapping) == 92, "au16ToneMappingValue at +92");
_Static_assert(offsetof(v4_isp_drc_attr, flt_scale_coarse) == 492, "u8FltScaleCoarse at +492");
_Static_assert(offsetof(v4_isp_drc_attr, dp_detect_thr_min) == 504, "u16DpDetectThrMin at +504");
_Static_assert(offsetof(v4_isp_drc_attr, op_type) == 508, "enOpType at +508");
_Static_assert(offsetof(v4_isp_drc_attr, auto_strength) == 514, "stAuto.u16Strength at +514");
_Static_assert(offsetof(v4_isp_drc_attr, asym) == 550, "stAsymmetryCurve at +550");

/* ================================================================
 * BAYER NR -- ISP_NR_ATTR_S (libisp.so)
 * ================================================================ */

typedef struct {
    int enable;
    int low_power_enable;
    int nr_lsc_enable;
    unsigned char nr_lsc_ratio;
    unsigned char bnr_lsc_max_gain;
    unsigned short bnr_lsc_cmp_strength;
    unsigned short coring_ratio[V4_ISP_BAYERNR_LUT];
    int op_type;
    /* stAuto */
    unsigned char auto_chroma_str[V4_ISP_BAYER_CHN][V4_ISP_ISO_NUM];
    unsigned char auto_fine_str[V4_ISP_ISO_NUM];
    unsigned short auto_coring_wgt[V4_ISP_ISO_NUM];
    unsigned short auto_coarse_str[V4_ISP_BAYER_CHN][V4_ISP_ISO_NUM];
    /* stManual */
    unsigned char man_chroma_str[V4_ISP_BAYER_CHN];
    unsigned char man_fine_str;
    unsigned short man_coring_wgt;
    unsigned short man_coarse_str[V4_ISP_BAYER_CHN];
    /* stWdr */
    unsigned char wdr_frame_str[4];
    unsigned char wdr_fusion_frame_str[4];
} v4_isp_nr_attr;

_Static_assert(sizeof(v4_isp_nr_attr) == 352, "ISP_NR_ATTR_S is 352 bytes");
_Static_assert(offsetof(v4_isp_nr_attr, op_type) == 84, "enOpType at +84");
_Static_assert(offsetof(v4_isp_nr_attr, auto_fine_str) == 152, "stAuto.au8FineStr at +152");
_Static_assert(offsetof(v4_isp_nr_attr, auto_coring_wgt) == 168, "stAuto.au16CoringWgt at +168");
_Static_assert(offsetof(v4_isp_nr_attr, man_chroma_str) == 328, "stManual at +328");
_Static_assert(offsetof(v4_isp_nr_attr, wdr_frame_str) == 344, "stWdr at +344");

/* ================================================================
 * DEHAZE -- ISP_DEHAZE_ATTR_S (libisp.so / lib_hidehaze.so registered)
 * ================================================================ */

typedef struct {
    int enable;
    int user_lut_enable;
    unsigned char lut[V4_ISP_DEHAZE_LUT];
    int op_type;
    unsigned char manual_strength;
    unsigned char auto_strength;
    unsigned short tmprflt_incr_coef;
    unsigned short tmprflt_decr_coef;
} v4_isp_dehaze_attr;

_Static_assert(sizeof(v4_isp_dehaze_attr) == 276, "ISP_DEHAZE_ATTR_S is 276 bytes");
_Static_assert(offsetof(v4_isp_dehaze_attr, op_type) == 264, "enOpType at +264");
_Static_assert(offsetof(v4_isp_dehaze_attr, auto_strength) == 269, "stAuto at +269");
_Static_assert(offsetof(v4_isp_dehaze_attr, tmprflt_decr_coef) == 272, "decr coef at +272");

/* ================================================================
 * SHARPEN -- ISP_SHARPEN_ATTR_S (libisp.so)
 * ================================================================ */

typedef struct {
    unsigned char luma_wgt[V4_ISP_SHARPEN_LUMA];
    unsigned short texture_str[V4_ISP_SHARPEN_GAIN];
    unsigned short edge_str[V4_ISP_SHARPEN_GAIN];
    unsigned short texture_freq;
    unsigned short edge_freq;
    unsigned char over_shoot;
    unsigned char under_shoot;
    unsigned char shoot_sup_str;
    unsigned char shoot_sup_adj;
    unsigned char detail_ctrl;
    unsigned char detail_ctrl_thr;
    unsigned char edge_filt_str;
    unsigned char edge_filt_max_cap;
    unsigned char r_gain;
    unsigned char g_gain;
    unsigned char b_gain;
    unsigned char skin_gain;
    unsigned short max_sharp_gain;
    unsigned char weak_detail_gain;
} v4_isp_sharpen_manual;

typedef struct {
    unsigned char luma_wgt[V4_ISP_SHARPEN_LUMA][V4_ISP_ISO_NUM];
    unsigned short texture_str[V4_ISP_SHARPEN_GAIN][V4_ISP_ISO_NUM];
    unsigned short edge_str[V4_ISP_SHARPEN_GAIN][V4_ISP_ISO_NUM];
    unsigned short texture_freq[V4_ISP_ISO_NUM];
    unsigned short edge_freq[V4_ISP_ISO_NUM];
    unsigned char over_shoot[V4_ISP_ISO_NUM];
    unsigned char under_shoot[V4_ISP_ISO_NUM];
    unsigned char shoot_sup_str[V4_ISP_ISO_NUM];
    unsigned char shoot_sup_adj[V4_ISP_ISO_NUM];
    unsigned char detail_ctrl[V4_ISP_ISO_NUM];
    unsigned char detail_ctrl_thr[V4_ISP_ISO_NUM];
    unsigned char edge_filt_str[V4_ISP_ISO_NUM];
    unsigned char edge_filt_max_cap[V4_ISP_ISO_NUM];
    unsigned char r_gain[V4_ISP_ISO_NUM];
    unsigned char g_gain[V4_ISP_ISO_NUM];
    unsigned char b_gain[V4_ISP_ISO_NUM];
    unsigned char skin_gain[V4_ISP_ISO_NUM];
    unsigned short max_sharp_gain[V4_ISP_ISO_NUM];
    unsigned char weak_detail_gain[V4_ISP_ISO_NUM];
} v4_isp_sharpen_auto;

typedef struct {
    int enable;
    unsigned char skin_umin;
    unsigned char skin_vmin;
    unsigned char skin_umax;
    unsigned char skin_vmax;
    int op_type;
    v4_isp_sharpen_manual manual;
    v4_isp_sharpen_auto auto_attr;
} v4_isp_sharpen_attr;

_Static_assert(sizeof(v4_isp_sharpen_attr) == 3056, "ISP_SHARPEN_ATTR_S is 3056 bytes");
_Static_assert(offsetof(v4_isp_sharpen_attr, manual) == 12, "stManual at +12");
_Static_assert(offsetof(v4_isp_sharpen_attr, auto_attr) == 192, "stAuto at +192");
_Static_assert(offsetof(v4_isp_sharpen_attr, auto_attr.texture_str) == 704,
               "au16TextureStr at +704");
_Static_assert(offsetof(v4_isp_sharpen_attr, auto_attr.texture_freq) == 2752,
               "au16TextureFreq at +2752");
_Static_assert(offsetof(v4_isp_sharpen_attr, auto_attr.max_sharp_gain) == 3008,
               "au16MaxSharpGain at +3008");

/* ================================================================
 * DYNAMIC DPC -- ISP_DP_DYNAMIC_ATTR_S (libisp.so)
 * ================================================================ */

typedef struct {
    int enable;
    int sup_twinkle_en;
    signed char soft_thr;
    unsigned char soft_slope;
    int op_type;
    unsigned short man_strength;
    unsigned short man_blend_ratio;
    unsigned short auto_strength[V4_ISP_ISO_NUM];
    unsigned short auto_blend_ratio[V4_ISP_ISO_NUM];
} v4_isp_dp_dyn_attr;

_Static_assert(sizeof(v4_isp_dp_dyn_attr) == 84, "ISP_DP_DYNAMIC_ATTR_S is 84 bytes");
_Static_assert(offsetof(v4_isp_dp_dyn_attr, auto_strength) == 20, "stAuto.au16Strength at +20");
_Static_assert(offsetof(v4_isp_dp_dyn_attr, auto_blend_ratio) == 52,
               "stAuto.au16BlendRatio at +52");

/* ================================================================
 * GAMMA -- ISP_GAMMA_ATTR_S (libisp.so)
 * ================================================================ */

#define V4_ISP_GAMMA_CURVE_USER 3 /* ISP_GAMMA_CURVE_USER_DEFINE */

typedef struct {
    int enable;
    unsigned short table[V4_ISP_GAMMA_NODES];
    int curve_type;
} v4_isp_gamma_attr;

_Static_assert(sizeof(v4_isp_gamma_attr) == 2060, "ISP_GAMMA_ATTR_S is 2060 bytes");
_Static_assert(offsetof(v4_isp_gamma_attr, curve_type) == 2056, "enCurveType at +2056");

/* ================================================================
 * ENTRY POINTS
 *
 * All optional, resolved with v4_symbol_opt over the ISP stack's search
 * list (libisp.so first, then lib_hiae.so, then the MPI handles for a
 * Goke build). A missing pair skips its INI section with a warning
 * rather than failing the load: tuning is a picture-quality upgrade,
 * never a pipeline requirement.
 * ================================================================ */

/*
 * ISP_EXP_INFO_S -- what AE is doing right now. Read-only, and read for
 * one field: u32ISO. Transcribed up to the routes, which are two pairs of
 * ISP_AE_ROUTE_S/_EX_S the ladder never looks at and which are carried
 * as bytes so the size the driver copies is right. Probed: 5476 bytes,
 * u32ISO at +4160.
 */
typedef struct {
    unsigned int exp_time;         /* u32ExpTime */
    unsigned int short_exp_time;
    unsigned int median_exp_time;
    unsigned int long_exp_time;
    unsigned int again;            /* u32AGain, 22.10 */
    unsigned int dgain;
    unsigned int again_sf;
    unsigned int dgain_sf;
    unsigned int isp_dgain;
    unsigned int exposure;         /* u32Exposure, 26.6 */
    int exposure_is_max;           /* bExposureIsMAX */
    short hist_error;              /* s16HistError */
    unsigned int ae_hist[1024];    /* au32AE_Hist1024Value */
    unsigned char ave_lum;         /* u8AveLum */
    unsigned int lines_per_500ms;
    unsigned int piris_fno;
    unsigned int fps;              /* u32Fps */
    unsigned int iso;              /* u32ISO */
    unsigned int iso_sf;
    unsigned int iso_calibrate;
    unsigned int ref_exp_ratio;
    unsigned int first_stable_time;
    unsigned char routes[5476 - 4180]; /* stAERoute, stAERouteEx, stAERouteSF, stAERouteSFEx */
} v4_isp_exp_info;

_Static_assert(sizeof(v4_isp_exp_info) == 5476, "ISP_EXP_INFO_S is 5476 bytes");
_Static_assert(offsetof(v4_isp_exp_info, again) == 16, "u32AGain at +16");
_Static_assert(offsetof(v4_isp_exp_info, exposure_is_max) == 40, "bExposureIsMAX at +40");
_Static_assert(offsetof(v4_isp_exp_info, ae_hist) == 48, "au32AE_Hist1024Value at +48");
_Static_assert(offsetof(v4_isp_exp_info, ave_lum) == 4144, "u8AveLum at +4144");
_Static_assert(offsetof(v4_isp_exp_info, fps) == 4156, "u32Fps at +4156");
_Static_assert(offsetof(v4_isp_exp_info, iso) == 4160, "u32ISO at +4160");

/* ISP_CSC_ATTR_S -- the colour-space conversion at the end of the ISP,
 * and the vendor's own image-adjust surface: hue, luma, contrast and
 * saturation are 0..100 with 50 as unity. The tuning file has no section
 * for it; the [image] knobs (hal_knob.c) are its only writer. */
typedef struct {
    int enable;
    int color_gamut; /* COLOR_GAMUT_E */
    unsigned char hue;
    unsigned char luma;
    unsigned char contr;
    unsigned char satu;
    int limited_range_en;
    int ext_csc_en;
    int ct_mode_en;
    short csc_idc[3]; /* CSC_MATRX_S, inlined */
    short csc_odc[3];
    short csc_coef[9];
} v4_isp_csc_attr;

_Static_assert(sizeof(v4_isp_csc_attr) == 56, "ISP_CSC_ATTR_S is 56 bytes");
_Static_assert(offsetof(v4_isp_csc_attr, hue) == 8, "u8Hue at +8");
_Static_assert(offsetof(v4_isp_csc_attr, contr) == 10, "u8Contr at +10");
_Static_assert(offsetof(v4_isp_csc_attr, limited_range_en) == 12, "bLimitedRangeEn at +12");
_Static_assert(offsetof(v4_isp_csc_attr, csc_idc) == 24, "stCscMagtrx at +24");

typedef struct {
    int (*get_exp)(int vi_pipe, v4_isp_exp_attr *a);
    int (*set_exp)(int vi_pipe, const v4_isp_exp_attr *a);
    int (*get_route_ex)(int vi_pipe, v4_isp_ae_route_ex *a);
    int (*set_route_ex)(int vi_pipe, const v4_isp_ae_route_ex *a);
    int (*get_stat)(int vi_pipe, v4_isp_stat_cfg *a);
    int (*set_stat)(int vi_pipe, const v4_isp_stat_cfg *a);
    int (*get_ldci)(int vi_pipe, v4_isp_ldci_attr *a);
    int (*set_ldci)(int vi_pipe, const v4_isp_ldci_attr *a);
    int (*get_drc)(int vi_pipe, v4_isp_drc_attr *a);
    int (*set_drc)(int vi_pipe, const v4_isp_drc_attr *a);
    int (*get_nr)(int vi_pipe, v4_isp_nr_attr *a);
    int (*set_nr)(int vi_pipe, const v4_isp_nr_attr *a);
    int (*get_dehaze)(int vi_pipe, v4_isp_dehaze_attr *a);
    int (*set_dehaze)(int vi_pipe, const v4_isp_dehaze_attr *a);
    int (*get_sharpen)(int vi_pipe, v4_isp_sharpen_attr *a);
    int (*set_sharpen)(int vi_pipe, const v4_isp_sharpen_attr *a);
    int (*get_dpc)(int vi_pipe, v4_isp_dp_dyn_attr *a);
    int (*set_dpc)(int vi_pipe, const v4_isp_dp_dyn_attr *a);
    int (*get_gamma)(int vi_pipe, v4_isp_gamma_attr *a);
    int (*set_gamma)(int vi_pipe, const v4_isp_gamma_attr *a);
    /* The CSC: brightness and contrast (hal_knob.c). */
    int (*get_csc)(int vi_pipe, v4_isp_csc_attr *a);
    int (*set_csc)(int vi_pipe, const v4_isp_csc_attr *a);
    /* HI_MPI_ISP_QueryExposureInfo, exported by lib_hiae.so rather than
     * libmpi; the 3DNR ladder's ISO source (hal_nrx.c). */
    int (*query_exp)(int vi_pipe, v4_isp_exp_info *info);
} v4_isp_tune_impl;

#endif /* HISI_V4_ISP_TUNE_H */
