/*
 * hisi_v5/v5_isp_tune.h -- the ISP module-attribute structs the IQ tuning
 * loader writes, and the entry points that carry them. HiMPP V5.0.
 *
 * Phase 3 scope, and the same discipline gen4's v4_isp_tune.h keeps: this
 * file transcribes exactly the attribute families hal_isp.c, hal_dyn.c,
 * hal_ladder.c and hal_knob.c apply, and nothing else. Every struct here
 * is a byte-exact ABI promise; an unused promise is pure risk. The
 * families deliberately left out are named at the bottom of this comment.
 *
 * WHERE THE SYMBOLS LIVE, which is not where the names suggest. The whole
 * surface is spelt ss_mpi_isp_*, but it is split across three libraries:
 *
 *   libss_mpi_isp.so   the pipeline modules -- stats, ldci, drc, nr,
 *                      dehaze, sharpen, dp, gamma, pregamma, black level,
 *                      demosaic, csc, ca, anti-false-colour, shading.
 *   libss_mpi_ae.so    ss_mpi_isp_set_exposure_attr and the AE routes.
 *                      Declared in ss_mpi_ae.h, named ss_mpi_isp_*, and
 *                      absent from libss_mpi_isp.so -- the same trap
 *                      ss_mpi_isp_query_exposure_info sets in v5_isp.h.
 *   libss_mpi_awb.so   ss_mpi_isp_set_ccm_attr, _saturation_attr,
 *                      _color_tone_attr, _color_sector_attr, _wb_attr,
 *                      _awb_attr_ex.
 *
 * v5_isp_tune_load resolves all three out of the one v5_mpi_libs search
 * list, so the split costs nothing at the call site -- but a symbol looked
 * for in the wrong .so is the first thing to suspect when one module of a
 * tuning file silently does not apply.
 *
 * EVERY ENTRY POINT IS OPTIONAL. Unlike v5_isp.h's bring-up sequence,
 * where a missing symbol means no image, a missing tuning setter means one
 * section of the .ini does not apply and the rest does. The loader reports
 * that per section and carries on, which is what makes a tuning file
 * written for a fuller SDK usable here.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_isp.h,
 * ss_mpi_isp.h, ss_mpi_ae.h and ss_mpi_awb.h at 1.0.2.0 B051 -- the same
 * headers the rest of hisi_v5 is transcribed from, and the version the
 * CV608 bench board's libraries report. Sizes and offsets in the
 * _Static_asserts below were read from a probe generated off those headers
 * and compiled twice: once for the host and once with the cv6xx musl
 * cross-compiler, with a generated assert file proving the two layouts are
 * identical for every struct here. The probe never enters the build.
 *
 * Two structs are deliberately partial, both following gen4's
 * v4_isp_stat_cfg pattern -- transcribe the half the loader writes, carry
 * the rest as opaque bytes sized so the Get/Set round-trip moves the whole
 * struct:
 *
 *   v5_isp_stats_cfg   the AE and WB halves are written (the weight table,
 *                      the AWB tap point and its black level); the focus
 *                      and motion tail passes through untouched.
 *   v5_isp_nr_attr     everything the .ini dialect names is written, head
 *                      and per-ISO ladders alike; only the WDR and dering
 *                      configs are opaque, 118 bytes with no .ini key
 *                      between them and dering not a CV610 module.
 *
 * NOT HERE, ON PURPOSE. 3DNR is not an ISP module on V5: the vendor's
 * scene_auto reference writes it through ss_mpi_vi_set_pipe_3dnr_param on
 * the VI pipe, not through the VPSS group the way gen4's hal_nrx.c does.
 * ot_3dnr_param belongs to the VI header family and is transcribed in
 * v5_vi.h when hal_nrx.c lands. The lens-shading mesh and the per-pipe
 * gain differences ([static_isp_diff]) stay out for the reason the AWB
 * calibration used to: they are measured off one physical module, and a
 * mesh from another board's lens is worse than no mesh. The AWB
 * calibration is here now, and it is here for the mirror-image reason --
 * ot_isp_wb_attr is the other half of the CCM the loader already applies,
 * and half a calibration is worse than either whole one. See the WHITE
 * BALANCE heading below.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_ISP_TUNE_H
#define HISI_V5_ISP_TUNE_H

#include "v5_common.h"

/*
 * The array dimensions that decide every layout below. Names in the
 * comments are the header's, so a reader can check each against
 * ot_common_isp.h without guessing which constant a number came from.
 */
#define V5_ISP_ISO_NUM 16           /* OT_ISP_AUTO_ISO_NUM */
#define V5_ISP_AE_ROUTE_EX_NODES 16 /* OT_ISP_AE_ROUTE_EX_MAX_NODES */
#define V5_ISP_AE_ROWS 15           /* OT_ISP_AE_ZONE_ROW */
#define V5_ISP_AE_COLS 17           /* OT_ISP_AE_ZONE_COLUMN */
#define V5_ISP_BE_AE_ROWS 32        /* OT_ISP_BE_AE_ZONE_ROW */
#define V5_ISP_BE_AE_COLS 32        /* OT_ISP_BE_AE_ZONE_COLUMN */
#define V5_ISP_SHARPEN_LUMA 32      /* OT_ISP_SHARPEN_LUMA_NUM */
#define V5_ISP_SHARPEN_GAIN 32      /* OT_ISP_SHARPEN_GAIN_NUM */
#define V5_ISP_SHARPEN_MOT 16       /* OT_ISP_SHARPEN_MOT_NUM */
#define V5_ISP_SHARPEN_RLYWGT 16    /* OT_ISP_SHARPEN_RLYWGT_NUM */
#define V5_ISP_SHARPEN_STDGAIN 16   /* OT_ISP_SHARPEN_STDGAIN_NUM */
#define V5_ISP_DRC_TM_NODES 200     /* OT_ISP_DRC_TM_NODE_NUM */
#define V5_ISP_DRC_CC_NODES 33      /* OT_ISP_DRC_CC_NODE_NUM */
#define V5_ISP_DRC_LMIX_NODES 33    /* OT_ISP_DRC_LMIX_NODE_NUM */
#define V5_ISP_DRC_BCNR_NODES 16    /* OT_ISP_DRC_BCNR_NODE_NUM */
#define V5_ISP_GAMMA_NODES 1025     /* OT_ISP_GAMMA_NODE_NUM */
#define V5_ISP_PREGAMMA_NODES 257   /* OT_ISP_PREGAMMA_NODE_NUM */
#define V5_ISP_BAYERNR_LUT 33       /* OT_ISP_BAYERNR_LUT_LENGTH */
#define V5_ISP_BAYERNR_LUT1 32      /* OT_ISP_BAYERNR_LUT_LENGTH1 */
#define V5_ISP_BAYER_CHN 4          /* OT_ISP_BAYER_CHN_NUM */
#define V5_ISP_WDR_FRAMES 4         /* OT_ISP_WDR_MAX_FRAME_NUM */
#define V5_ISP_DEHAZE_LUT 256       /* OT_ISP_DEHAZE_LUT_SIZE */
#define V5_ISP_CCM_MATRIX_NUM 7     /* OT_ISP_CCM_MATRIX_NUM */
#define V5_ISP_CCM_MATRIX_SIZE 9    /* OT_ISP_CCM_MATRIX_SIZE */
#define V5_ISP_CA_LUT 128           /* OT_ISP_CA_YRATIO_LUT_LENGTH */
#define V5_ISP_COLOR_SECTORS 6      /* OT_ISP_COLOR_SECTOR_NUM */
#define V5_ISP_AWB_CURVE_PARA_NUM 6 /* OT_ISP_AWB_CURVE_PARA_NUM */
#define V5_ISP_AWB_LS_NUM 4         /* OT_ISP_AWB_LS_NUM */
#define V5_ISP_AWB_MULTI_CT_NUM 8   /* OT_ISP_AWB_MULTI_CT_NUM */
#define V5_ISP_AWB_LUM_HIST_NUM 6   /* OT_ISP_AWB_LUM_HIST_NUM */
#define V5_ISP_AWB_ZONE_NUM 1024    /* OT_ISP_AWB_ZONE_ORIG_ROW * _COLUMN, 32 x 32 */
#define V5_ISP_CSC_DC_NUM 3         /* OT_ISP_CSC_DC_NUM */
#define V5_ISP_CSC_COEF_NUM 9       /* OT_ISP_CSC_COEF_NUM */
#define V5_ISP_CAC_THR_NUM 2        /* OT_ISP_CAC_THR_NUM */
#define V5_ISP_CAC_CURVE_NUM 3      /* OT_ISP_CAC_CURVE_NUM */
#define V5_ISP_CAC_EXP_RATIO_NUM 16 /* OT_ISP_CAC_EXP_RATIO_NUM */

/* ot_op_mode: 0 auto, 1 manual. Spelt out because the .ini dialect writes
 * the word and every module below carries one. */
#define V5_ISP_OP_AUTO 0
#define V5_ISP_OP_MANUAL 1

/* ot_isp_gamma_curve_type: DEFAULT, SRGB, HDR (unsupported), USER_DEFINE. */
#define V5_ISP_GAMMA_CURVE_USER 3

/* ================================================================
 * EXPOSURE -- ot_isp_exposure_attr, libss_mpi_ae.so
 * ================================================================ */

typedef struct {
    unsigned int max, min; /* max before min, as the header has it */
} v5_isp_ae_range;

_Static_assert(sizeof(v5_isp_ae_range) == 8, "ot_isp_ae_range is 8 bytes");

typedef struct {
    int enable;
    unsigned char frequency; /* 50 or 60 */
    int mode;                /* ot_isp_antiflicker_mode */
} v5_isp_antiflicker;

typedef struct {
    int enable;
    unsigned char luma_diff;
} v5_isp_subflicker;

typedef struct {
    unsigned short black_delay_frame;
    unsigned short white_delay_frame;
} v5_isp_ae_delay;

/* ot_isp_me_attr -- manual exposure. Untouched by the loader; present so
 * the auto half lands at the right offset. */
typedef struct {
    int exp_time_op_type, a_gain_op_type, d_gain_op_type, ispd_gain_op_type;
    unsigned int exp_time, a_gain, d_gain, isp_d_gain;
} v5_isp_ae_manual;

_Static_assert(sizeof(v5_isp_ae_manual) == 32, "ot_isp_me_attr is 32 bytes");

/* ot_isp_ae_attr */
typedef struct {
    v5_isp_ae_range exp_time_range;
    v5_isp_ae_range a_gain_range;
    v5_isp_ae_range d_gain_range;
    v5_isp_ae_range ispd_gain_range;
    v5_isp_ae_range sys_gain_range;
    unsigned int gain_threshold;
    unsigned char speed;
    unsigned short black_speed_bias;
    unsigned char tolerance;
    unsigned char compensation;
    unsigned short ev_bias;
    int ae_strategy_mode; /* ot_isp_ae_strategy */
    unsigned short hist_ratio_slope;
    unsigned char max_hist_offset;
    int ae_mode; /* ot_isp_ae_mode */
    v5_isp_antiflicker antiflicker;
    v5_isp_subflicker subflicker;
    v5_isp_ae_delay ae_delay_attr;
    int manual_exp_value;
    unsigned int exp_value;
    int fswdr_mode; /* ot_isp_fswdr_mode */
    int wdr_quick;
    unsigned short iso_cal_coef;
} v5_isp_ae_auto;

_Static_assert(sizeof(v5_isp_ae_auto) == 108, "ot_isp_ae_attr is 108 bytes");
_Static_assert(offsetof(v5_isp_ae_auto, speed) == 44, "ot_isp_ae_attr.speed at +44");
_Static_assert(offsetof(v5_isp_ae_auto, tolerance) == 48, "ot_isp_ae_attr.tolerance at +48");
_Static_assert(offsetof(v5_isp_ae_auto, hist_ratio_slope) == 56,
               "ot_isp_ae_attr.hist_ratio_slope at +56");
_Static_assert(offsetof(v5_isp_ae_auto, ae_mode) == 60, "ot_isp_ae_attr.ae_mode at +60");
_Static_assert(offsetof(v5_isp_ae_auto, ae_delay_attr) == 84,
               "ot_isp_ae_attr.ae_delay_attr at +84");
_Static_assert(offsetof(v5_isp_ae_auto, iso_cal_coef) == 104,
               "ot_isp_ae_attr.iso_cal_coef at +104");

/*
 * ot_isp_exposure_attr. Four bytes longer than gen4's ISP_EXPOSURE_ATTR_S
 * -- advance_ae is new at the tail -- and the auto half moved with it, so
 * the offsets below are the ones worth asserting.
 */
typedef struct {
    int bypass;
    int op_type;
    unsigned char ae_run_interval;
    int hist_stat_adjust;
    int ae_route_ex_valid;
    v5_isp_ae_manual manual_attr;
    v5_isp_ae_auto auto_attr;
    int prior_frame; /* ot_isp_prior_frame */
    int ae_gain_sep_cfg;
    int advance_ae;
} v5_isp_exp_attr;

_Static_assert(sizeof(v5_isp_exp_attr) == 172, "ot_isp_exposure_attr is 172 bytes");
_Static_assert(offsetof(v5_isp_exp_attr, ae_route_ex_valid) == 16,
               "ot_isp_exposure_attr.ae_route_ex_valid at +16");
_Static_assert(offsetof(v5_isp_exp_attr, auto_attr) == 52, "ot_isp_exposure_attr.auto_attr at +52");
_Static_assert(offsetof(v5_isp_exp_attr, prior_frame) == 160,
               "ot_isp_exposure_attr.prior_frame at +160");

/* ================================================================
 * AE ROUTE EX -- ot_isp_ae_route_ex, libss_mpi_ae.so
 * ================================================================ */

typedef struct {
    unsigned int int_time;
    unsigned int a_gain;
    unsigned int d_gain;
    unsigned int isp_d_gain;
    int iris_fno; /* ot_isp_iris_f_no */
    unsigned int iris_fno_lin;
} v5_isp_ae_route_ex_node;

_Static_assert(sizeof(v5_isp_ae_route_ex_node) == 24, "ot_isp_ae_route_ex_node is 24 bytes");

typedef struct {
    unsigned int total_num;
    v5_isp_ae_route_ex_node route_ex_node[V5_ISP_AE_ROUTE_EX_NODES];
} v5_isp_ae_route_ex;

_Static_assert(sizeof(v5_isp_ae_route_ex) == 388, "ot_isp_ae_route_ex is 388 bytes");

/* ================================================================
 * AE STATISTICS -- ot_isp_stats_cfg, libss_mpi_isp.so
 *
 * Partial by design: the loader writes the metering weight table and
 * nothing else, so the AE half is transcribed and the WB/focus/motion tail
 * rides along as bytes. ctrl and update are bitfield words the header
 * spells as a struct of one td_u32 each; the loader must set the AE bit in
 * `update` for a weight write to take, which is why they are named here
 * rather than folded into the opaque tail.
 * ================================================================ */

typedef struct {
    int enable;
    unsigned short x, y, width, height;
} v5_isp_ae_crop;

_Static_assert(sizeof(v5_isp_ae_crop) == 12, "ot_isp_ae_crop is 12 bytes");

typedef struct {
    int hist_skip_x, hist_skip_y, hist_offset_x, hist_offset_y;
} v5_isp_ae_hist_config;

typedef struct {
    int ae_switch;
    v5_isp_ae_hist_config hist_config;
    int four_plane_mode;
    int hist_mode;
    int aver_mode;
    int max_gain_mode;
    v5_isp_ae_crop crop;
    v5_isp_ae_crop fe_crop;
    unsigned char weight[V5_ISP_AE_ROWS][V5_ISP_AE_COLS];
    unsigned char be_weight[V5_ISP_BE_AE_ROWS][V5_ISP_BE_AE_COLS];
} v5_isp_ae_stats_cfg;

_Static_assert(sizeof(v5_isp_ae_stats_cfg) == 1340, "ot_isp_ae_stats_cfg is 1340 bytes");
_Static_assert(offsetof(v5_isp_ae_stats_cfg, weight) == 60, "ot_isp_ae_stats_cfg.weight at +60");
_Static_assert(offsetof(v5_isp_ae_stats_cfg, be_weight) == 315,
               "ot_isp_ae_stats_cfg.be_weight at +315");

/*
 * ot_isp_awb_crop and ot_isp_wb_stats_cfg -- where in the pipeline the AWB
 * takes its statistics, and the window it takes them over. [static_awb]
 * names two of these fields (awb_switch, black_level) and the vendor's own
 * loader writes them here rather than into the WB attribute, which is why
 * the WB half of the stats config is transcribed and the focus and motion
 * halves after it stay opaque.
 */
typedef struct {
    int enable;
    unsigned short x, y, width, height;
} v5_isp_awb_crop;

_Static_assert(sizeof(v5_isp_awb_crop) == 12, "ot_isp_awb_crop is 12 bytes");

typedef struct {
    int awb_switch; /* ot_isp_awb_switch: 0 after DG, 1 after expander, 2 after DRC */
    unsigned short zone_row, zone_col;
    unsigned short white_level, black_level;
    unsigned short cb_max, cb_min;
    unsigned short cr_max, cr_min;
    v5_isp_awb_crop crop;
} v5_isp_wb_stats_cfg;

_Static_assert(sizeof(v5_isp_wb_stats_cfg) == 32, "ot_isp_wb_stats_cfg is 32 bytes");
_Static_assert(offsetof(v5_isp_wb_stats_cfg, black_level) == 10, "wb_stats_cfg.black_level at +10");
_Static_assert(offsetof(v5_isp_wb_stats_cfg, crop) == 20, "wb_stats_cfg.crop at +20");

typedef struct {
    unsigned int ctrl;   /* ot_isp_stats_ctrl, one u32 of bits */
    unsigned int update; /* likewise; bit 0 is AE */
    v5_isp_ae_stats_cfg ae_cfg;
    v5_isp_wb_stats_cfg wb_cfg;
    unsigned char tail[284]; /* focus_cfg 276 + motion_cfg 8 */
} v5_isp_stats_cfg;

_Static_assert(sizeof(v5_isp_stats_cfg) == 1664, "ot_isp_stats_cfg is 1664 bytes");
_Static_assert(offsetof(v5_isp_stats_cfg, ae_cfg) == 8, "ot_isp_stats_cfg.ae_cfg at +8");
_Static_assert(offsetof(v5_isp_stats_cfg, wb_cfg) == 1348, "ot_isp_stats_cfg.wb_cfg at +1348");
_Static_assert(offsetof(v5_isp_stats_cfg, tail) == 1380, "ot_isp_stats_cfg tail at +1380");

/* ================================================================
 * LDCI -- ot_isp_ldci_attr
 * ================================================================ */

typedef struct {
    unsigned char wgt, sigma, mean;
} v5_isp_ldci_gauss_coef;

typedef struct {
    v5_isp_ldci_gauss_coef he_pos_wgt;
    v5_isp_ldci_gauss_coef he_neg_wgt;
} v5_isp_ldci_he_wgt;

_Static_assert(sizeof(v5_isp_ldci_he_wgt) == 6, "ot_isp_ldci_he_wgt_attr is 6 bytes");

typedef struct {
    v5_isp_ldci_he_wgt he_wgt;
    unsigned short blc_ctrl;
} v5_isp_ldci_manual;

typedef struct {
    v5_isp_ldci_he_wgt he_wgt[V5_ISP_ISO_NUM];
    unsigned short blc_ctrl[V5_ISP_ISO_NUM];
} v5_isp_ldci_auto;

typedef struct {
    int enable;
    unsigned char gauss_lpf_sigma;
    int op_type;
    v5_isp_ldci_manual manual_attr;
    v5_isp_ldci_auto auto_attr;
    unsigned short tpr_incr_coef;
    unsigned short tpr_decr_coef;
} v5_isp_ldci_attr;

_Static_assert(sizeof(v5_isp_ldci_attr) == 152, "ot_isp_ldci_attr is 152 bytes");
_Static_assert(offsetof(v5_isp_ldci_attr, auto_attr) == 20, "ot_isp_ldci_attr.auto_attr at +20");
_Static_assert(offsetof(v5_isp_ldci_attr, tpr_incr_coef) == 148,
               "ot_isp_ldci_attr.tpr_incr_coef at +148");

/* ================================================================
 * DRC -- ot_isp_drc_attr
 * ================================================================ */

/* ot_isp_drc_mixing_bright_param and ..._dark_param: identical layouts. */
typedef struct {
    unsigned char max;
    unsigned char min;
    unsigned char threshold;
    signed char slope;
} v5_isp_drc_mixing_param;

_Static_assert(sizeof(v5_isp_drc_mixing_param) == 4, "ot_isp_drc_mixing_*_param is 4 bytes");

typedef struct {
    unsigned short strength;
} v5_isp_drc_manual;

typedef struct {
    unsigned short strength, strength_max, strength_min;
} v5_isp_drc_auto;

typedef struct {
    unsigned char asymmetry, second_pole, stretch, compress;
} v5_isp_drc_asymmetry_curve;

typedef struct {
    unsigned char brightness, contrast, tolerance;
} v5_isp_drc_auto_curve;

typedef struct {
    int enable;
    unsigned char detail_restore_lut[V5_ISP_DRC_BCNR_NODES];
    unsigned char strength;
} v5_isp_drc_bcnr;

typedef struct {
    int enable;
    int curve_select; /* ot_isp_drc_curve_select */
    unsigned char purple_reduction_strength;
    unsigned char bright_gain_limit;
    unsigned char bright_gain_limit_step;
    unsigned char dark_gain_limit_luma;
    unsigned char dark_gain_limit_chroma;
    unsigned char contrast_ctrl;
    unsigned char rim_reduction_strength;
    unsigned char rim_reduction_threshold;
    unsigned short color_correction_lut[V5_ISP_DRC_CC_NODES];
    unsigned short tone_mapping_value[V5_ISP_DRC_TM_NODES];
    unsigned char spatial_filter_coef;
    unsigned char range_filter_coef;
    unsigned char detail_adjust_coef;
    /*
     * Two unions in the vendor header, one per detail-mixing member: the
     * 33-node LUT is the DV500 form, and the four-field parameter is the
     * CV610 one. Same bytes, and the CV610 form is the first four of them
     * -- so the LUT stays (hal_isp.c's [static_drc] writes it, and a file
     * meant for another die is then simply ignored by this driver) and the
     * parameter is what [dynamic_linear_drc] names.
     */
    union {
        v5_isp_drc_mixing_param local_mixing_bright_param;
        unsigned char local_mixing_bright[V5_ISP_DRC_LMIX_NODES];
    };
    union {
        v5_isp_drc_mixing_param local_mixing_dark_param;
        unsigned char local_mixing_dark[V5_ISP_DRC_LMIX_NODES];
    };
    unsigned char high_saturation_color_ctrl;
    unsigned char global_color_ctrl;
    int shoot_reduction_en;
    int op_type;
    v5_isp_drc_manual manual_attr;
    v5_isp_drc_auto auto_attr;
    v5_isp_drc_asymmetry_curve asymmetry_curve;
    v5_isp_drc_auto_curve auto_curve;
    v5_isp_drc_bcnr bcnr_attr;
} v5_isp_drc_attr;

_Static_assert(sizeof(v5_isp_drc_attr) == 604, "ot_isp_drc_attr is 604 bytes");
_Static_assert(offsetof(v5_isp_drc_attr, color_correction_lut) == 16,
               "ot_isp_drc_attr.color_correction_lut at +16");
_Static_assert(offsetof(v5_isp_drc_attr, tone_mapping_value) == 82,
               "ot_isp_drc_attr.tone_mapping_value at +82");
_Static_assert(offsetof(v5_isp_drc_attr, local_mixing_bright) == 485,
               "ot_isp_drc_attr.local_mixing_bright at +485");
_Static_assert(offsetof(v5_isp_drc_attr, local_mixing_bright_param) == 485,
               "ot_isp_drc_attr.local_mixing_bright_param shares +485");
_Static_assert(offsetof(v5_isp_drc_attr, local_mixing_dark_param) == 518,
               "ot_isp_drc_attr.local_mixing_dark_param shares +518");
_Static_assert(offsetof(v5_isp_drc_attr, op_type) == 560, "ot_isp_drc_attr.op_type at +560");
_Static_assert(offsetof(v5_isp_drc_attr, bcnr_attr) == 580, "ot_isp_drc_attr.bcnr_attr at +580");

/* ================================================================
 * BAYER NR -- ot_isp_nr_attr
 *
 * The whole per-ISO ladder set, because the ladders are the module:
 * noisesd_lut is the sensor's measured noise-versus-signal curve, and
 * without it the filter runs the library's generic profile and cleans the
 * fine grain while leaving the coarse component behind. Only the WDR and
 * dering configs stay opaque -- neither has a key in the .ini dialect and
 * dering is not a CV610 module.
 * ================================================================ */

typedef struct {
    unsigned short sfm0_coarse_strength[V5_ISP_BAYER_CHN][V5_ISP_ISO_NUM];
    unsigned char sfm0_detail_prot[V5_ISP_ISO_NUM];
    unsigned short sfm1_strength[V5_ISP_ISO_NUM];
    unsigned char sfm1_adp_strength[V5_ISP_ISO_NUM];
    unsigned char sfm6_strength[V5_ISP_ISO_NUM];
    unsigned char sfm7_strength[V5_ISP_ISO_NUM];
    unsigned char sth[V5_ISP_ISO_NUM];
    unsigned char tss[V5_ISP_ISO_NUM];
    unsigned char fine_strength[V5_ISP_ISO_NUM];
    unsigned short coring_wgt[V5_ISP_ISO_NUM];
    unsigned char coring_mot_ratio[V5_ISP_ISO_NUM];
    unsigned char noisesd_lut[V5_ISP_BAYERNR_LUT1][V5_ISP_ISO_NUM];
} v5_isp_nr_snr_auto;

_Static_assert(sizeof(v5_isp_nr_snr_auto) == 832, "ot_isp_nr_snr_auto_attr is 832 bytes");
_Static_assert(offsetof(v5_isp_nr_snr_auto, sfm0_detail_prot) == 128,
               "ot_isp_nr_snr_auto_attr.sfm0_detail_prot at +128");
_Static_assert(offsetof(v5_isp_nr_snr_auto, tss) == 240, "ot_isp_nr_snr_auto_attr.tss at +240");
_Static_assert(offsetof(v5_isp_nr_snr_auto, fine_strength) == 256,
               "ot_isp_nr_snr_auto_attr.fine_strength at +256");
_Static_assert(offsetof(v5_isp_nr_snr_auto, coring_wgt) == 272,
               "ot_isp_nr_snr_auto_attr.coring_wgt at +272");
_Static_assert(offsetof(v5_isp_nr_snr_auto, noisesd_lut) == 320,
               "ot_isp_nr_snr_auto_attr.noisesd_lut at +320");

typedef struct {
    unsigned short sfm0_coarse_strength[V5_ISP_BAYER_CHN];
    unsigned char sfm0_detail_prot;
    unsigned short sfm1_strength;
    unsigned char sfm1_adp_strength;
    unsigned char sfm6_strength;
    unsigned char sfm7_strength;
    unsigned char sth;
    unsigned char tss;
    unsigned char fine_strength;
    unsigned short coring_wgt;
    unsigned char coring_mot_ratio;
    unsigned char noisesd_lut[V5_ISP_BAYERNR_LUT1];
} v5_isp_nr_snr_manual;

_Static_assert(sizeof(v5_isp_nr_snr_manual) == 54, "ot_isp_nr_snr_manual_attr is 54 bytes");
_Static_assert(offsetof(v5_isp_nr_snr_manual, sfm1_strength) == 10,
               "ot_isp_nr_snr_manual_attr.sfm1_strength at +10");
_Static_assert(offsetof(v5_isp_nr_snr_manual, coring_wgt) == 18,
               "ot_isp_nr_snr_manual_attr.coring_wgt at +18");
_Static_assert(offsetof(v5_isp_nr_snr_manual, noisesd_lut) == 21,
               "ot_isp_nr_snr_manual_attr.noisesd_lut at +21");

/*
 * ot_isp_nr_snr_attr wraps ot_isp_nr_snr_attr_v0 in a version tag and a
 * one-armed union; the tag is flattened here the way the vendor header's
 * single OT_NR_SNR_V0 makes it.
 */
typedef struct {
    int snr_version; /* ot_nr_snr_mode, OT_NR_SNR_V0 only */
    v5_isp_nr_snr_auto snr_auto;
    v5_isp_nr_snr_manual snr_manual;
} v5_isp_nr_snr_cfg;

_Static_assert(sizeof(v5_isp_nr_snr_cfg) == 892, "ot_isp_nr_snr_attr is 892 bytes");
_Static_assert(offsetof(v5_isp_nr_snr_cfg, snr_auto) == 4, "ot_isp_nr_snr_attr.snr_auto at +4");
_Static_assert(offsetof(v5_isp_nr_snr_cfg, snr_manual) == 836,
               "ot_isp_nr_snr_attr.snr_manual at +836");

typedef struct {
    unsigned char md_mode[V5_ISP_ISO_NUM];
    unsigned char md_size_ratio[V5_ISP_ISO_NUM];
    unsigned char md_anti_flicker_strength[V5_ISP_ISO_NUM];
    unsigned char md_static_ratio[V5_ISP_ISO_NUM];
    unsigned char md_motion_ratio[V5_ISP_ISO_NUM];
    unsigned char md_static_fine_strength[V5_ISP_ISO_NUM];
    unsigned char tfs[V5_ISP_ISO_NUM];
    unsigned char user_define_md[V5_ISP_ISO_NUM];
    signed short user_define_slope[V5_ISP_ISO_NUM];
    unsigned short user_define_dark_thresh[V5_ISP_ISO_NUM];
    unsigned char user_define_color_thresh[V5_ISP_ISO_NUM];
    unsigned char sfr_r[V5_ISP_ISO_NUM];
    unsigned char sfr_g[V5_ISP_ISO_NUM];
    unsigned char sfr_b[V5_ISP_ISO_NUM];
} v5_isp_nr_md_auto;

_Static_assert(sizeof(v5_isp_nr_md_auto) == 256, "ot_isp_nr_md_auto_attr is 256 bytes");
_Static_assert(offsetof(v5_isp_nr_md_auto, user_define_md) == 112,
               "ot_isp_nr_md_auto_attr.user_define_md at +112");
_Static_assert(offsetof(v5_isp_nr_md_auto, user_define_slope) == 128,
               "ot_isp_nr_md_auto_attr.user_define_slope at +128");
_Static_assert(offsetof(v5_isp_nr_md_auto, sfr_r) == 208, "ot_isp_nr_md_auto_attr.sfr_r at +208");

typedef struct {
    unsigned char md_mode;
    unsigned char md_size_ratio;
    unsigned char md_anti_flicker_strength;
    unsigned char md_static_ratio;
    unsigned char md_motion_ratio;
    unsigned char md_static_fine_strength;
    unsigned char tfs;
    unsigned char user_define_md;
    signed short user_define_slope;
    unsigned short user_define_dark_thresh;
    unsigned char user_define_color_thresh;
    unsigned char sfr_r;
    unsigned char sfr_g;
    unsigned char sfr_b;
} v5_isp_nr_md_manual;

_Static_assert(sizeof(v5_isp_nr_md_manual) == 16, "ot_isp_nr_md_manual_attr is 16 bytes");
_Static_assert(offsetof(v5_isp_nr_md_manual, user_define_slope) == 8,
               "ot_isp_nr_md_manual_attr.user_define_slope at +8");

typedef struct {
    v5_isp_nr_md_auto md_auto;
    v5_isp_nr_md_manual md_manual;
} v5_isp_nr_md_cfg;

_Static_assert(sizeof(v5_isp_nr_md_cfg) == 272, "ot_isp_nr_md_attr is 272 bytes");

typedef struct {
    int enable;
    int op_type;
    int md_en;
    int lsc_nr_en;
    unsigned char lsc_ratio1;
    unsigned short coring_ratio[V5_ISP_BAYERNR_LUT];
    unsigned short mix_gain[V5_ISP_BAYERNR_LUT1];
    int ref_mode; /* ot_isp_bnr_ref_mode */
    int load_ref_en;
    v5_isp_nr_snr_cfg snr_cfg;
    /*
     * The vendor header unions md_cfg with ot_isp_nr_tnr_attr, the DV500
     * arm; that arm is the larger of the two, so the union is 324 bytes.
     */
    union {
        v5_isp_nr_md_cfg md_cfg;
        unsigned char tnr_cfg[324];
    };
    unsigned char tail[118]; /* wdr_cfg 32 + dering_cfg 86, neither in the .ini */
} v5_isp_nr_attr;

_Static_assert(sizeof(v5_isp_nr_attr) == 1492, "ot_isp_nr_attr is 1492 bytes");
_Static_assert(offsetof(v5_isp_nr_attr, coring_ratio) == 18, "ot_isp_nr_attr.coring_ratio at +18");
_Static_assert(offsetof(v5_isp_nr_attr, mix_gain) == 84, "ot_isp_nr_attr.mix_gain at +84");
_Static_assert(offsetof(v5_isp_nr_attr, snr_cfg) == 156, "ot_isp_nr_attr.snr_cfg at +156");
_Static_assert(offsetof(v5_isp_nr_attr, md_cfg) == 1048, "ot_isp_nr_attr.md_cfg at +1048");
_Static_assert(offsetof(v5_isp_nr_attr, tail) == 1372, "ot_isp_nr_attr wdr_cfg at +1372");

/* ================================================================
 * DEHAZE -- ot_isp_dehaze_attr
 * ================================================================ */

typedef struct {
    int enable;
    int user_lut_en;
    unsigned char dehaze_lut[V5_ISP_DEHAZE_LUT];
    int op_type;
    unsigned char manual_strength;
    unsigned char auto_strength;
    unsigned short tmprflt_incr_coef;
    unsigned short tmprflt_decr_coef;
} v5_isp_dehaze_attr;

_Static_assert(sizeof(v5_isp_dehaze_attr) == 276, "ot_isp_dehaze_attr is 276 bytes");
_Static_assert(offsetof(v5_isp_dehaze_attr, op_type) == 264, "ot_isp_dehaze_attr.op_type at +264");
_Static_assert(offsetof(v5_isp_dehaze_attr, auto_strength) == 269,
               "ot_isp_dehaze_attr.auto_attr at +269");

/* ================================================================
 * SHARPEN -- ot_isp_sharpen_attr
 *
 * The largest module here, 7168 bytes, and the one whose auto half is a
 * plain [knob][ISO] table throughout -- which is exactly the shape the
 * .ini dialect writes, one line of sixteen numbers per knob.
 * ================================================================ */

typedef struct {
    unsigned short shoot_inner_threshold;
    unsigned short shoot_outer_threshold;
    unsigned short shoot_protect_threshold;
} v5_isp_sharpen_manual_shoot;

typedef struct {
    unsigned short edge_rly_fine_threshold;
    unsigned short edge_rly_coarse_threshold;
    unsigned char edge_overshoot;
    unsigned char edge_undershoot;
    unsigned char edge_gain_by_rly[V5_ISP_SHARPEN_RLYWGT];
    unsigned char edge_rly_by_mot[V5_ISP_SHARPEN_STDGAIN];
    unsigned char edge_rly_by_luma[V5_ISP_SHARPEN_STDGAIN];
} v5_isp_sharpen_manual_edge_rly;

typedef struct {
    unsigned char mf_gain_by_mot[V5_ISP_SHARPEN_MOT];
    unsigned char hf_gain_by_mot[V5_ISP_SHARPEN_MOT];
    unsigned char lmf_gain_by_mot[V5_ISP_SHARPEN_MOT];
} v5_isp_sharpen_manual_gain_by_mot;

typedef struct {
    unsigned char luma_wgt[V5_ISP_SHARPEN_LUMA];
    unsigned short texture_strength[V5_ISP_SHARPEN_GAIN];
    unsigned short edge_strength[V5_ISP_SHARPEN_GAIN];
    unsigned short texture_freq;
    unsigned short edge_freq;
    unsigned char over_shoot;
    unsigned char under_shoot;
    unsigned short motion_texture_strength[V5_ISP_SHARPEN_GAIN];
    unsigned short motion_edge_strength[V5_ISP_SHARPEN_GAIN];
    unsigned short motion_texture_freq;
    unsigned short motion_edge_freq;
    unsigned char motion_over_shoot;
    unsigned char motion_under_shoot;
    unsigned char shoot_sup_strength;
    unsigned char shoot_sup_adj;
    unsigned char detail_ctrl;
    unsigned char detail_ctrl_threshold;
    unsigned char edge_filt_strength;
    unsigned char edge_filt_max_cap;
    unsigned char r_gain;
    unsigned char g_gain;
    unsigned char b_gain;
    unsigned char skin_gain;
    unsigned short max_sharp_gain;
    v5_isp_sharpen_manual_shoot shoot_threshold_attr;
    v5_isp_sharpen_manual_edge_rly edge_rly_attr;
    v5_isp_sharpen_manual_gain_by_mot gain_by_mot_attr;
} v5_isp_sharpen_manual;

_Static_assert(sizeof(v5_isp_sharpen_manual) == 420, "ot_isp_sharpen_manual_attr is 420 bytes");
_Static_assert(offsetof(v5_isp_sharpen_manual, motion_texture_strength) == 166,
               "sharpen manual motion_texture_strength at +166");
_Static_assert(offsetof(v5_isp_sharpen_manual, shoot_threshold_attr) == 312,
               "sharpen manual shoot_threshold_attr at +312");

typedef struct {
    unsigned short shoot_inner_threshold[V5_ISP_ISO_NUM];
    unsigned short shoot_outer_threshold[V5_ISP_ISO_NUM];
    unsigned short shoot_protect_threshold[V5_ISP_ISO_NUM];
} v5_isp_sharpen_auto_shoot;

typedef struct {
    unsigned short edge_rly_fine_threshold[V5_ISP_ISO_NUM];
    unsigned short edge_rly_coarse_threshold[V5_ISP_ISO_NUM];
    unsigned char edge_overshoot[V5_ISP_ISO_NUM];
    unsigned char edge_undershoot[V5_ISP_ISO_NUM];
    unsigned char edge_gain_by_rly[V5_ISP_SHARPEN_RLYWGT][V5_ISP_ISO_NUM];
    unsigned char edge_rly_by_mot[V5_ISP_SHARPEN_STDGAIN][V5_ISP_ISO_NUM];
    unsigned char edge_rly_by_luma[V5_ISP_SHARPEN_STDGAIN][V5_ISP_ISO_NUM];
} v5_isp_sharpen_auto_edge_rly;

typedef struct {
    unsigned char mf_gain_by_mot[V5_ISP_SHARPEN_MOT][V5_ISP_ISO_NUM];
    unsigned char hf_gain_by_mot[V5_ISP_SHARPEN_MOT][V5_ISP_ISO_NUM];
    unsigned char lmf_gain_by_mot[V5_ISP_SHARPEN_MOT][V5_ISP_ISO_NUM];
} v5_isp_sharpen_auto_gain_by_mot;

typedef struct {
    unsigned char luma_wgt[V5_ISP_SHARPEN_LUMA][V5_ISP_ISO_NUM];
    unsigned short texture_strength[V5_ISP_SHARPEN_GAIN][V5_ISP_ISO_NUM];
    unsigned short edge_strength[V5_ISP_SHARPEN_GAIN][V5_ISP_ISO_NUM];
    unsigned short texture_freq[V5_ISP_ISO_NUM];
    unsigned short edge_freq[V5_ISP_ISO_NUM];
    unsigned char over_shoot[V5_ISP_ISO_NUM];
    unsigned char under_shoot[V5_ISP_ISO_NUM];
    unsigned short motion_texture_strength[V5_ISP_SHARPEN_GAIN][V5_ISP_ISO_NUM];
    unsigned short motion_edge_strength[V5_ISP_SHARPEN_GAIN][V5_ISP_ISO_NUM];
    unsigned short motion_texture_freq[V5_ISP_ISO_NUM];
    unsigned short motion_edge_freq[V5_ISP_ISO_NUM];
    unsigned char motion_over_shoot[V5_ISP_ISO_NUM];
    unsigned char motion_under_shoot[V5_ISP_ISO_NUM];
    unsigned char shoot_sup_strength[V5_ISP_ISO_NUM];
    unsigned char shoot_sup_adj[V5_ISP_ISO_NUM];
    unsigned char detail_ctrl[V5_ISP_ISO_NUM];
    unsigned char detail_ctrl_threshold[V5_ISP_ISO_NUM];
    unsigned char edge_filt_strength[V5_ISP_ISO_NUM];
    unsigned char edge_filt_max_cap[V5_ISP_ISO_NUM];
    unsigned char r_gain[V5_ISP_ISO_NUM];
    unsigned char g_gain[V5_ISP_ISO_NUM];
    unsigned char b_gain[V5_ISP_ISO_NUM];
    unsigned char skin_gain[V5_ISP_ISO_NUM];
    unsigned short max_sharp_gain[V5_ISP_ISO_NUM];
    v5_isp_sharpen_auto_shoot shoot_threshold_attr;
    v5_isp_sharpen_auto_edge_rly edge_rly_attr;
    v5_isp_sharpen_auto_gain_by_mot gain_by_mot_attr;
} v5_isp_sharpen_auto;

_Static_assert(sizeof(v5_isp_sharpen_auto) == 6720, "ot_isp_sharpen_auto_attr is 6720 bytes");
_Static_assert(offsetof(v5_isp_sharpen_auto, texture_strength) == 512,
               "sharpen auto texture_strength at +512");
_Static_assert(offsetof(v5_isp_sharpen_auto, motion_texture_strength) == 2656,
               "sharpen auto motion_texture_strength at +2656");
_Static_assert(offsetof(v5_isp_sharpen_auto, max_sharp_gain) == 4960,
               "sharpen auto max_sharp_gain at +4960");
_Static_assert(offsetof(v5_isp_sharpen_auto, edge_rly_attr) == 5088,
               "sharpen auto edge_rly_attr at +5088");

typedef struct {
    int enable;
    int motion_en;
    unsigned char motion_threshold0;
    unsigned char motion_threshold1;
    unsigned short motion_gain0;
    unsigned short motion_gain1;
    unsigned char skin_umin, skin_vmin, skin_umax, skin_vmax;
    int op_type;
    int detail_map; /* ot_isp_sharpen_detail_map */
    v5_isp_sharpen_manual manual_attr;
    v5_isp_sharpen_auto auto_attr;
} v5_isp_sharpen_attr;

_Static_assert(sizeof(v5_isp_sharpen_attr) == 7168, "ot_isp_sharpen_attr is 7168 bytes");
_Static_assert(offsetof(v5_isp_sharpen_attr, manual_attr) == 28,
               "ot_isp_sharpen_attr.manual_attr at +28");
_Static_assert(offsetof(v5_isp_sharpen_attr, auto_attr) == 448,
               "ot_isp_sharpen_attr.auto_attr at +448");

/* ================================================================
 * DEFECT PIXEL, dynamic half -- ot_isp_dp_dynamic_attr
 *
 * The static half (ot_isp_dp_static_attr) is a 32 KB pair of calibrated
 * pixel tables, not scene tuning, and is not transcribed.
 * ================================================================ */

typedef struct {
    unsigned char strength;
    unsigned char blend_ratio;
} v5_isp_dp_dynamic_manual;

typedef struct {
    unsigned char strength[V5_ISP_ISO_NUM];
    unsigned char blend_ratio[V5_ISP_ISO_NUM];
} v5_isp_dp_dynamic_auto;

typedef struct {
    int sup_twinkle_en;
    signed char soft_thr;
    unsigned char soft_slope;
    int op_type;
    v5_isp_dp_dynamic_manual manual_attr;
    v5_isp_dp_dynamic_auto auto_attr;
    unsigned char bright_strength;
    unsigned char dark_strength;
} v5_isp_dp_frame_dynamic;

_Static_assert(sizeof(v5_isp_dp_frame_dynamic) == 48, "ot_isp_dp_frame_dynamic_attr is 48 bytes");
_Static_assert(offsetof(v5_isp_dp_frame_dynamic, auto_attr) == 14,
               "dp frame dynamic auto_attr at +14");

typedef struct {
    int enable;
    v5_isp_dp_frame_dynamic frame_dynamic[V5_ISP_WDR_FRAMES];
} v5_isp_dp_dynamic_attr;

_Static_assert(sizeof(v5_isp_dp_dynamic_attr) == 196, "ot_isp_dp_dynamic_attr is 196 bytes");

/* ================================================================
 * WHITE BALANCE -- ot_isp_wb_attr and ot_isp_awb_attr_ex, libss_mpi_awb.so
 *
 * The AWB's own calibration: the white point the sensor sees under a
 * reference illuminant, the Planckian curve fitted through it, and the
 * limits the estimate is allowed to wander inside. It pairs with the CCM
 * -- the AWB decides which matrix to blend and how far -- so a file that
 * carries [static_ccm] and [static_awb] carries one calibration in two
 * halves, and applying one half is worse than applying neither.
 * ================================================================ */

typedef struct {
    int enable;
    int op_type;
    unsigned short high_rg_limit, high_bg_limit;
    unsigned short low_rg_limit, low_bg_limit;
} v5_isp_awb_ct_limit_attr;

_Static_assert(sizeof(v5_isp_awb_ct_limit_attr) == 16, "ot_isp_awb_ct_limit_attr is 16 bytes");

typedef struct {
    int enable;
    unsigned short cr_max[V5_ISP_ISO_NUM];
    unsigned short cr_min[V5_ISP_ISO_NUM];
    unsigned short cb_max[V5_ISP_ISO_NUM];
    unsigned short cb_min[V5_ISP_ISO_NUM];
} v5_isp_awb_cbcr_track_attr;

_Static_assert(sizeof(v5_isp_awb_cbcr_track_attr) == 132,
               "ot_isp_awb_cbcr_track_attr is 132 bytes");
_Static_assert(offsetof(v5_isp_awb_cbcr_track_attr, cr_max) == 4, "cbcr_track.cr_max at +4");

typedef struct {
    int enable;
    int op_type;
    unsigned char hist_thresh[V5_ISP_AWB_LUM_HIST_NUM];
    unsigned short hist_wt[V5_ISP_AWB_LUM_HIST_NUM];
} v5_isp_awb_lum_hist_attr;

_Static_assert(sizeof(v5_isp_awb_lum_hist_attr) == 28, "ot_isp_awb_lum_histgram_attr is 28 bytes");
_Static_assert(offsetof(v5_isp_awb_lum_hist_attr, hist_wt) == 14, "luma_hist.hist_wt at +14");

typedef struct {
    int enable;
    unsigned short ref_color_temp;
    unsigned short static_wb[V5_ISP_BAYER_CHN];
    int curve_para[V5_ISP_AWB_CURVE_PARA_NUM]; /* signed; curve_para[4] must stay 128 */
    int alg_type;                              /* ot_isp_awb_alg_type */
    unsigned char rg_strength, bg_strength;
    unsigned short speed;
    unsigned short zone_sel;
    unsigned short high_color_temp, low_color_temp;
    v5_isp_awb_ct_limit_attr ct_limit;
    int shift_limit_en;
    unsigned char shift_limit;
    int gain_norm_en;
    int natural_cast_en;
    v5_isp_awb_cbcr_track_attr cb_cr_track;
    v5_isp_awb_lum_hist_attr luma_hist;
    int awb_zone_wt_en;
    unsigned char zone_wt[V5_ISP_AWB_ZONE_NUM];
} v5_isp_awb_attr;

_Static_assert(sizeof(v5_isp_awb_attr) == 1276, "ot_isp_awb_attr is 1276 bytes");
_Static_assert(offsetof(v5_isp_awb_attr, static_wb) == 6, "awb_attr.static_wb at +6");
_Static_assert(offsetof(v5_isp_awb_attr, curve_para) == 16, "awb_attr.curve_para at +16");
_Static_assert(offsetof(v5_isp_awb_attr, speed) == 46, "awb_attr.speed at +46");
_Static_assert(offsetof(v5_isp_awb_attr, ct_limit) == 56, "awb_attr.ct_limit at +56");
_Static_assert(offsetof(v5_isp_awb_attr, shift_limit) == 76, "awb_attr.shift_limit at +76");
_Static_assert(offsetof(v5_isp_awb_attr, cb_cr_track) == 88, "awb_attr.cb_cr_track at +88");
_Static_assert(offsetof(v5_isp_awb_attr, luma_hist) == 220, "awb_attr.luma_hist at +220");
_Static_assert(offsetof(v5_isp_awb_attr, zone_wt) == 252, "awb_attr.zone_wt at +252");

typedef struct {
    unsigned short r_gain, gr_gain, gb_gain, b_gain;
} v5_isp_mwb_attr;

_Static_assert(sizeof(v5_isp_mwb_attr) == 8, "ot_isp_mwb_attr is 8 bytes");

typedef struct {
    int bypass;
    unsigned char awb_run_interval;
    int op_type;
    v5_isp_mwb_attr manual_attr;
    v5_isp_awb_attr auto_attr;
    int alg_type; /* ot_isp_awb_alg: 0 grey world, 1 spectral (not on CV610) */
} v5_isp_wb_attr;

_Static_assert(sizeof(v5_isp_wb_attr) == 1300, "ot_isp_wb_attr is 1300 bytes");
_Static_assert(offsetof(v5_isp_wb_attr, op_type) == 8, "wb_attr.op_type at +8");
_Static_assert(offsetof(v5_isp_wb_attr, manual_attr) == 12, "wb_attr.manual_attr at +12");
_Static_assert(offsetof(v5_isp_wb_attr, auto_attr) == 20, "wb_attr.auto_attr at +20");
_Static_assert(offsetof(v5_isp_wb_attr, alg_type) == 1296, "wb_attr.alg_type at +1296");

typedef struct {
    unsigned short white_r_gain, white_b_gain;
    unsigned short exp_quant;
    unsigned char light_status;
    unsigned char radius;
} v5_isp_awb_light_source;

_Static_assert(sizeof(v5_isp_awb_light_source) == 8,
               "ot_isp_awb_extra_light_source_info is 8 bytes");

typedef struct {
    int enable;
    int op_type;
    int scene_status; /* ot_isp_awb_scene_mode_status: 0 indoor, 1 outdoor */
    unsigned int out_thresh;
    unsigned short low_start, low_stop;
    unsigned short high_start, high_stop;
    int green_enhance_en;
    unsigned char out_shift_limit;
} v5_isp_awb_in_out_attr;

_Static_assert(sizeof(v5_isp_awb_in_out_attr) == 32, "ot_isp_awb_in_out_attr is 32 bytes");
_Static_assert(offsetof(v5_isp_awb_in_out_attr, low_start) == 16, "in_or_out.low_start at +16");
_Static_assert(offsetof(v5_isp_awb_in_out_attr, out_shift_limit) == 28,
               "in_or_out.out_shift_limit at +28");

typedef struct {
    unsigned char tolerance;
    unsigned char zone_radius;
    unsigned short curve_l_limit, curve_r_limit;
    int extra_light_en;
    v5_isp_awb_light_source light_info[V5_ISP_AWB_LS_NUM];
    v5_isp_awb_in_out_attr in_or_out;
    int multi_light_source_en;
    int multi_ls_type; /* ot_isp_awb_multi_ls_type: 0 saturation, 1 CCM */
    unsigned short multi_ls_scaler;
    unsigned short multi_ct_bin[V5_ISP_AWB_MULTI_CT_NUM];
    unsigned short multi_ct_wt[V5_ISP_AWB_MULTI_CT_NUM];
    int fine_tun_en;
    unsigned char fine_tun_strength;
} v5_isp_awb_attr_ex;

_Static_assert(sizeof(v5_isp_awb_attr_ex) == 128, "ot_isp_awb_attr_ex is 128 bytes");
_Static_assert(offsetof(v5_isp_awb_attr_ex, light_info) == 12, "awb_attr_ex.light_info at +12");
_Static_assert(offsetof(v5_isp_awb_attr_ex, in_or_out) == 44, "awb_attr_ex.in_or_out at +44");
_Static_assert(offsetof(v5_isp_awb_attr_ex, multi_ct_bin) == 86, "awb_attr_ex.multi_ct_bin at +86");
_Static_assert(offsetof(v5_isp_awb_attr_ex, multi_ct_wt) == 102, "awb_attr_ex.multi_ct_wt at +102");
_Static_assert(offsetof(v5_isp_awb_attr_ex, fine_tun_en) == 120, "awb_attr_ex.fine_tun_en at +120");

/* ================================================================
 * SATURATION and CCM -- libss_mpi_awb.so
 * ================================================================ */

typedef struct {
    int op_type;
    unsigned char manual_saturation;
    unsigned char auto_sat[V5_ISP_ISO_NUM];
} v5_isp_saturation_attr;

_Static_assert(sizeof(v5_isp_saturation_attr) == 24, "ot_isp_saturation_attr is 24 bytes");
_Static_assert(offsetof(v5_isp_saturation_attr, auto_sat) == 5,
               "ot_isp_saturation_attr.auto_attr at +5");

typedef struct {
    unsigned short color_temp;
    unsigned short ccm[V5_ISP_CCM_MATRIX_SIZE];
} v5_isp_ccm_param;

_Static_assert(sizeof(v5_isp_ccm_param) == 20, "ot_isp_color_matrix_param is 20 bytes");

typedef struct {
    int sat_en;
    unsigned short ccm[V5_ISP_CCM_MATRIX_SIZE];
} v5_isp_ccm_manual;

typedef struct {
    int iso_act_en;
    int temp_act_en;
    unsigned short ccm_tab_num;
    v5_isp_ccm_param ccm_tab[V5_ISP_CCM_MATRIX_NUM];
} v5_isp_ccm_auto;

typedef struct {
    int op_type;
    v5_isp_ccm_manual manual_attr;
    v5_isp_ccm_auto auto_attr;
} v5_isp_ccm_attr;

_Static_assert(sizeof(v5_isp_ccm_attr) == 180, "ot_isp_color_matrix_attr is 180 bytes");
_Static_assert(offsetof(v5_isp_ccm_attr, auto_attr) == 28,
               "ot_isp_color_matrix_attr.auto_attr at +28");
_Static_assert(offsetof(v5_isp_ccm_auto, ccm_tab) == 10, "ccm auto ccm_tab at +10");

/*
 * ot_isp_color_tone_attr -- three per-channel gains applied after the CCM.
 * A separate MPI pair from the matrix, but the same [static_ccm] section
 * carries all four, which is why it sits here rather than under a heading
 * of its own.
 */
typedef struct {
    unsigned short red_cast_gain;
    unsigned short green_cast_gain;
    unsigned short blue_cast_gain;
} v5_isp_color_tone_attr;

_Static_assert(sizeof(v5_isp_color_tone_attr) == 6, "ot_isp_color_tone_attr is 6 bytes");

/*
 * ot_isp_color_sector_attr, libss_mpi_awb.so. Seven tables of six hue and
 * six saturation shifts -- one table per CCM matrix, which is the AWB's
 * colour-temperature axis, not the ISO axis the other auto halves run on.
 * [dynamic_color_sector] (hal_ladder.c) writes all seven per ISO.
 */
typedef struct {
    unsigned char hue_shift[V5_ISP_COLOR_SECTORS];
    unsigned char sat_shift[V5_ISP_COLOR_SECTORS];
} v5_isp_color_sector_param;

typedef struct {
    v5_isp_color_sector_param color_tab[V5_ISP_CCM_MATRIX_NUM];
} v5_isp_color_sector_auto;

typedef struct {
    int enable;
    v5_isp_color_sector_param manual_attr;
    v5_isp_color_sector_auto auto_attr;
} v5_isp_color_sector_attr;

_Static_assert(sizeof(v5_isp_color_sector_attr) == 100, "ot_isp_color_sector_attr is 100 bytes");
_Static_assert(offsetof(v5_isp_color_sector_attr, auto_attr) == 16,
               "ot_isp_color_sector_attr.auto_attr at +16");

/* ================================================================
 * GAMMA and PREGAMMA
 * ================================================================ */

typedef struct {
    int enable;
    unsigned short table[V5_ISP_GAMMA_NODES];
    int curve_type; /* ot_isp_gamma_curve_type */
} v5_isp_gamma_attr;

_Static_assert(sizeof(v5_isp_gamma_attr) == 2060, "ot_isp_gamma_attr is 2060 bytes");
_Static_assert(offsetof(v5_isp_gamma_attr, curve_type) == 2056, "gamma curve_type at +2056");

typedef struct {
    int enable;
    unsigned int table[V5_ISP_PREGAMMA_NODES];
} v5_isp_pregamma_attr;

_Static_assert(sizeof(v5_isp_pregamma_attr) == 1032, "ot_isp_pregamma_attr is 1032 bytes");

/* ================================================================
 * BLACK LEVEL -- ot_isp_black_level_attr
 * ================================================================ */

typedef struct {
    unsigned short black_level[V5_ISP_WDR_FRAMES][V5_ISP_BAYER_CHN];
} v5_isp_blc_manual;

typedef struct {
    int pattern; /* ot_isp_black_level_dynamic_pattern */
    v5_rect ob_area;
    unsigned short low_threshold;
    unsigned short high_threshold;
    signed short offset[V5_ISP_ISO_NUM];
    unsigned short tolerance;
    unsigned char filter_strength;
    int separate_en;
    unsigned short calibration_black_level[V5_ISP_ISO_NUM];
    unsigned short filter_thr;
} v5_isp_blc_dynamic;

_Static_assert(sizeof(v5_isp_blc_dynamic) == 100, "ot_isp_black_level_dynamic_attr is 100 bytes");
_Static_assert(offsetof(v5_isp_blc_dynamic, offset) == 24, "blc dynamic offset at +24");
_Static_assert(offsetof(v5_isp_blc_dynamic, calibration_black_level) == 64,
               "blc dynamic calibration_black_level at +64");

typedef struct {
    int user_black_level_en;
    unsigned short user_black_level[V5_ISP_WDR_FRAMES][V5_ISP_BAYER_CHN];
    int black_level_mode;
    v5_isp_blc_manual manual_attr;
    v5_isp_blc_dynamic dynamic_attr;
} v5_isp_blc_attr;

_Static_assert(sizeof(v5_isp_blc_attr) == 172, "ot_isp_black_level_attr is 172 bytes");
_Static_assert(offsetof(v5_isp_blc_attr, dynamic_attr) == 72, "blc dynamic_attr at +72");

/* ================================================================
 * DEMOSAIC, CSC, CA, ANTI-FALSE-COLOUR, SHADING
 * ================================================================ */

typedef struct {
    unsigned char nddm_strength;
    unsigned char nddm_mf_detail_strength;
    unsigned char hf_detail_strength;
    unsigned char detail_smooth_range;
    unsigned char color_noise_f_threshold;
    unsigned char color_noise_f_strength;
    unsigned char color_noise_y_threshold;
    unsigned char color_noise_y_strength;
} v5_isp_demosaic_manual;

typedef struct {
    unsigned char nddm_strength[V5_ISP_ISO_NUM];
    unsigned char nddm_mf_detail_strength[V5_ISP_ISO_NUM];
    unsigned char hf_detail_strength[V5_ISP_ISO_NUM];
    unsigned char detail_smooth_range[V5_ISP_ISO_NUM];
    unsigned char color_noise_f_threshold[V5_ISP_ISO_NUM];
    unsigned char color_noise_f_strength[V5_ISP_ISO_NUM];
    unsigned char color_noise_y_threshold[V5_ISP_ISO_NUM];
    unsigned char color_noise_y_strength[V5_ISP_ISO_NUM];
} v5_isp_demosaic_auto;

typedef struct {
    int enable;
    int op_type;
    unsigned short ai_detail_strength;
    v5_isp_demosaic_manual manual_attr;
    v5_isp_demosaic_auto auto_attr;
} v5_isp_demosaic_attr;

_Static_assert(sizeof(v5_isp_demosaic_attr) == 148, "ot_isp_demosaic_attr is 148 bytes");
_Static_assert(offsetof(v5_isp_demosaic_attr, auto_attr) == 18, "demosaic auto_attr at +18");

typedef struct {
    signed short csc_in_dc[V5_ISP_CSC_DC_NUM];
    signed short csc_out_dc[V5_ISP_CSC_DC_NUM];
    signed short csc_coef[V5_ISP_CSC_COEF_NUM];
} v5_isp_csc_matrix;

typedef struct {
    int enable;
    int color_gamut; /* ot_color_gamut */
    unsigned char hue, luma, contr, satu;
    int limited_range_en;
    int ext_csc_en;
    int ct_mode_en;
    v5_isp_csc_matrix csc_magtrx;
} v5_isp_csc_attr;

_Static_assert(sizeof(v5_isp_csc_attr) == 56, "ot_isp_csc_attr is 56 bytes");
_Static_assert(offsetof(v5_isp_csc_attr, csc_magtrx) == 24, "csc csc_magtrx at +24");

typedef struct {
    unsigned int y_ratio_lut[V5_ISP_CA_LUT];
    signed int iso_ratio[V5_ISP_ISO_NUM];
    unsigned int y_sat_lut[V5_ISP_CA_LUT];
} v5_isp_ca_lut;

typedef struct {
    unsigned char cp_lut_y[V5_ISP_CA_LUT];
    unsigned char cp_lut_u[V5_ISP_CA_LUT];
    unsigned char cp_lut_v[V5_ISP_CA_LUT];
} v5_isp_cp_lut;

typedef struct {
    int enable;
    int ca_cp_en; /* ot_isp_ca_type */
    v5_isp_ca_lut ca;
    v5_isp_cp_lut cp;
} v5_isp_ca_attr;

_Static_assert(sizeof(v5_isp_ca_attr) == 1480, "ot_isp_ca_attr is 1480 bytes");
_Static_assert(offsetof(v5_isp_ca_attr, cp) == 1096, "ca cp at +1096");

typedef struct {
    int enable;
    int op_type;
    unsigned short manual_strength;
    unsigned short auto_strength[V5_ISP_ISO_NUM];
} v5_isp_anti_false_color_attr;

_Static_assert(sizeof(v5_isp_anti_false_color_attr) == 44,
               "ot_isp_anti_false_color_attr is 44 bytes");
_Static_assert(offsetof(v5_isp_anti_false_color_attr, auto_strength) == 10,
               "anti_false_color auto_attr at +10");

/* ================================================================
 * CHROMATIC ABERRATION -- ot_isp_cac_attr
 *
 * Two correctors in one attribute. ACAC works on the edge itself, gated
 * by a pair of per-ISO thresholds; LCAC works on the purple cast around
 * it, by exposure ratio rather than ISO. The .ini dialect gives them one
 * section and one [module_state] flag, bStaticCac, and so does the
 * vendor's own loader -- bStaticLocalCac is read there and never used.
 *
 * Two constraints are cross-key and so are left to the driver's own
 * check, which reports them: purple_upper_limit must be strictly above
 * purple_lower_limit, and edge_threshold_0 strictly below
 * edge_threshold_1 in every ISO column.
 * ================================================================ */

typedef struct {
    unsigned short edge_threshold[V5_ISP_CAC_THR_NUM];
    unsigned short edge_gain;
    unsigned short cac_rb_strength;
    unsigned short purple_alpha;
    unsigned short edge_alpha;
    unsigned short satu_low_threshold;
    unsigned short satu_high_threshold; /* not a CV610 field */
} v5_isp_cac_acac_manual;

_Static_assert(sizeof(v5_isp_cac_acac_manual) == 16, "ot_isp_cac_acac_manual_attr is 16 bytes");

typedef struct {
    unsigned short edge_threshold[V5_ISP_CAC_THR_NUM][V5_ISP_ISO_NUM];
    unsigned short edge_gain[V5_ISP_ISO_NUM];
    unsigned short cac_rb_strength[V5_ISP_ISO_NUM];
    unsigned short purple_alpha[V5_ISP_ISO_NUM];
    unsigned short edge_alpha[V5_ISP_ISO_NUM];
    unsigned short satu_low_threshold[V5_ISP_ISO_NUM];
    unsigned short satu_high_threshold[V5_ISP_ISO_NUM]; /* not a CV610 field */
} v5_isp_cac_acac_auto;

_Static_assert(sizeof(v5_isp_cac_acac_auto) == 256, "ot_isp_cac_acac_auto_attr is 256 bytes");
_Static_assert(offsetof(v5_isp_cac_acac_auto, edge_gain) == 64,
               "ot_isp_cac_acac_auto_attr.edge_gain at +64");
_Static_assert(offsetof(v5_isp_cac_acac_auto, satu_low_threshold) == 192,
               "ot_isp_cac_acac_auto_attr.satu_low_threshold at +192");

typedef struct {
    v5_isp_cac_acac_manual acac_manual;
    v5_isp_cac_acac_auto acac_auto;
} v5_isp_cac_acac;

_Static_assert(sizeof(v5_isp_cac_acac) == 272, "ot_isp_cac_acac_attr is 272 bytes");

typedef struct {
    unsigned char de_purple_cr_strength;
    unsigned char de_purple_cb_strength;
} v5_isp_cac_lcac_manual;

typedef struct {
    unsigned char de_purple_cr_strength[V5_ISP_CAC_EXP_RATIO_NUM];
    unsigned char de_purple_cb_strength[V5_ISP_CAC_EXP_RATIO_NUM];
} v5_isp_cac_lcac_auto;

typedef struct {
    unsigned short purple_detect_range;
    unsigned short var_threshold;
    unsigned short r_detect_threshold[V5_ISP_CAC_CURVE_NUM];
    unsigned short g_detect_threshold[V5_ISP_CAC_CURVE_NUM];
    unsigned short b_detect_threshold[V5_ISP_CAC_CURVE_NUM];
    v5_isp_cac_lcac_manual lcac_manual;
    v5_isp_cac_lcac_auto lcac_auto;
} v5_isp_cac_lcac;

_Static_assert(sizeof(v5_isp_cac_lcac) == 56, "ot_isp_cac_lcac_attr is 56 bytes");
_Static_assert(offsetof(v5_isp_cac_lcac, b_detect_threshold) == 16,
               "ot_isp_cac_lcac_attr.b_detect_threshold at +16");
_Static_assert(offsetof(v5_isp_cac_lcac, lcac_auto) == 24, "ot_isp_cac_lcac_attr.lcac_auto at +24");

typedef struct {
    int enable;
    int op_type;
    unsigned char detect_mode; /* not a CV610 field */
    signed short purple_upper_limit;
    signed short purple_lower_limit;
    v5_isp_cac_acac acac_cfg;
    v5_isp_cac_lcac lcac_cfg;
} v5_isp_cac_attr;

_Static_assert(sizeof(v5_isp_cac_attr) == 344, "ot_isp_cac_attr is 344 bytes");
_Static_assert(offsetof(v5_isp_cac_attr, purple_upper_limit) == 10,
               "ot_isp_cac_attr.purple_upper_limit at +10");
_Static_assert(offsetof(v5_isp_cac_attr, acac_cfg) == 14, "ot_isp_cac_attr.acac_cfg at +14");
_Static_assert(offsetof(v5_isp_cac_attr, lcac_cfg) == 286, "ot_isp_cac_attr.lcac_cfg at +286");

typedef struct {
    int enable;
    unsigned short mesh_strength;
    unsigned short blend_ratio;
} v5_isp_shading_attr;

_Static_assert(sizeof(v5_isp_shading_attr) == 8, "ot_isp_shading_attr is 8 bytes");

/* ================================================================
 * ENTRY POINTS
 * ================================================================ */

typedef struct {
    /* libss_mpi_ae.so, despite the ss_mpi_isp_ spelling. */
    int (*fnGetExposureAttr)(int vi_pipe, v5_isp_exp_attr *attr);
    int (*fnSetExposureAttr)(int vi_pipe, const v5_isp_exp_attr *attr);
    int (*fnGetAeRouteAttrEx)(int vi_pipe, v5_isp_ae_route_ex *attr);
    int (*fnSetAeRouteAttrEx)(int vi_pipe, const v5_isp_ae_route_ex *attr);

    /* libss_mpi_awb.so, likewise. */
    int (*fnGetCcmAttr)(int vi_pipe, v5_isp_ccm_attr *attr);
    int (*fnSetCcmAttr)(int vi_pipe, const v5_isp_ccm_attr *attr);
    int (*fnGetSaturationAttr)(int vi_pipe, v5_isp_saturation_attr *attr);
    int (*fnSetSaturationAttr)(int vi_pipe, const v5_isp_saturation_attr *attr);
    int (*fnGetColorToneAttr)(int vi_pipe, v5_isp_color_tone_attr *attr);
    int (*fnSetColorToneAttr)(int vi_pipe, const v5_isp_color_tone_attr *attr);
    int (*fnGetColorSectorAttr)(int vi_pipe, v5_isp_color_sector_attr *attr);
    int (*fnSetColorSectorAttr)(int vi_pipe, const v5_isp_color_sector_attr *attr);
    int (*fnGetWbAttr)(int vi_pipe, v5_isp_wb_attr *attr);
    int (*fnSetWbAttr)(int vi_pipe, const v5_isp_wb_attr *attr);
    int (*fnGetAwbAttrEx)(int vi_pipe, v5_isp_awb_attr_ex *attr);
    int (*fnSetAwbAttrEx)(int vi_pipe, const v5_isp_awb_attr_ex *attr);

    /* libss_mpi_isp.so. */
    int (*fnGetStatsCfg)(int vi_pipe, v5_isp_stats_cfg *attr);
    int (*fnSetStatsCfg)(int vi_pipe, const v5_isp_stats_cfg *attr);
    int (*fnGetLdciAttr)(int vi_pipe, v5_isp_ldci_attr *attr);
    int (*fnSetLdciAttr)(int vi_pipe, const v5_isp_ldci_attr *attr);
    int (*fnGetDrcAttr)(int vi_pipe, v5_isp_drc_attr *attr);
    int (*fnSetDrcAttr)(int vi_pipe, const v5_isp_drc_attr *attr);
    int (*fnGetNrAttr)(int vi_pipe, v5_isp_nr_attr *attr);
    int (*fnSetNrAttr)(int vi_pipe, const v5_isp_nr_attr *attr);
    int (*fnGetDehazeAttr)(int vi_pipe, v5_isp_dehaze_attr *attr);
    int (*fnSetDehazeAttr)(int vi_pipe, const v5_isp_dehaze_attr *attr);
    int (*fnGetSharpenAttr)(int vi_pipe, v5_isp_sharpen_attr *attr);
    int (*fnSetSharpenAttr)(int vi_pipe, const v5_isp_sharpen_attr *attr);
    int (*fnGetDpDynamicAttr)(int vi_pipe, v5_isp_dp_dynamic_attr *attr);
    int (*fnSetDpDynamicAttr)(int vi_pipe, const v5_isp_dp_dynamic_attr *attr);
    int (*fnGetGammaAttr)(int vi_pipe, v5_isp_gamma_attr *attr);
    int (*fnSetGammaAttr)(int vi_pipe, const v5_isp_gamma_attr *attr);
    int (*fnGetPregammaAttr)(int vi_pipe, v5_isp_pregamma_attr *attr);
    int (*fnSetPregammaAttr)(int vi_pipe, const v5_isp_pregamma_attr *attr);
    int (*fnGetBlackLevelAttr)(int vi_pipe, v5_isp_blc_attr *attr);
    int (*fnSetBlackLevelAttr)(int vi_pipe, const v5_isp_blc_attr *attr);
    int (*fnGetDemosaicAttr)(int vi_pipe, v5_isp_demosaic_attr *attr);
    int (*fnSetDemosaicAttr)(int vi_pipe, const v5_isp_demosaic_attr *attr);
    int (*fnGetCscAttr)(int vi_pipe, v5_isp_csc_attr *attr);
    int (*fnSetCscAttr)(int vi_pipe, const v5_isp_csc_attr *attr);
    int (*fnGetCaAttr)(int vi_pipe, v5_isp_ca_attr *attr);
    int (*fnSetCaAttr)(int vi_pipe, const v5_isp_ca_attr *attr);
    int (*fnGetAntiFalseColorAttr)(int vi_pipe, v5_isp_anti_false_color_attr *attr);
    int (*fnSetAntiFalseColorAttr)(int vi_pipe, const v5_isp_anti_false_color_attr *attr);
    int (*fnGetShadingAttr)(int vi_pipe, v5_isp_shading_attr *attr);
    int (*fnSetShadingAttr)(int vi_pipe, const v5_isp_shading_attr *attr);
    int (*fnGetCacAttr)(int vi_pipe, v5_isp_cac_attr *attr);
    int (*fnSetCacAttr)(int vi_pipe, const v5_isp_cac_attr *attr);
} v5_isp_tune_impl;

/*
 * v5_isp_tune_load -- bind whatever of the tuning surface this image has.
 *
 * Never fails. A module whose pair does not resolve is a module the
 * loader skips with one note; the caller checks the pair it is about to
 * use, not the return value. That is the whole difference in contract
 * between this and v5_isp_load.
 */
static inline void v5_isp_tune_load(v5_isp_tune_impl *lib, const v5_mpi_libs *libs)
{
    memset(lib, 0, sizeof(*lib));

#define V5_TUNE_PAIR(getter, setter, type, name)                                                   \
    do {                                                                                           \
        lib->getter = (int (*)(int, type *))v5_symbol_opt(libs, "ss_mpi_isp_get_" name);           \
        lib->setter = (int (*)(int, const type *))v5_symbol_opt(libs, "ss_mpi_isp_set_" name);     \
    } while (0)

    V5_TUNE_PAIR(fnGetExposureAttr, fnSetExposureAttr, v5_isp_exp_attr, "exposure_attr");
    V5_TUNE_PAIR(fnGetAeRouteAttrEx, fnSetAeRouteAttrEx, v5_isp_ae_route_ex, "ae_route_attr_ex");

    V5_TUNE_PAIR(fnGetCcmAttr, fnSetCcmAttr, v5_isp_ccm_attr, "ccm_attr");
    V5_TUNE_PAIR(fnGetSaturationAttr, fnSetSaturationAttr, v5_isp_saturation_attr,
                 "saturation_attr");
    V5_TUNE_PAIR(fnGetColorToneAttr, fnSetColorToneAttr, v5_isp_color_tone_attr, "color_tone_attr");
    V5_TUNE_PAIR(fnGetColorSectorAttr, fnSetColorSectorAttr, v5_isp_color_sector_attr,
                 "color_sector_attr");
    V5_TUNE_PAIR(fnGetWbAttr, fnSetWbAttr, v5_isp_wb_attr, "wb_attr");
    V5_TUNE_PAIR(fnGetAwbAttrEx, fnSetAwbAttrEx, v5_isp_awb_attr_ex, "awb_attr_ex");
    V5_TUNE_PAIR(fnGetStatsCfg, fnSetStatsCfg, v5_isp_stats_cfg, "stats_cfg");
    V5_TUNE_PAIR(fnGetLdciAttr, fnSetLdciAttr, v5_isp_ldci_attr, "ldci_attr");
    V5_TUNE_PAIR(fnGetDrcAttr, fnSetDrcAttr, v5_isp_drc_attr, "drc_attr");
    V5_TUNE_PAIR(fnGetNrAttr, fnSetNrAttr, v5_isp_nr_attr, "nr_attr");
    V5_TUNE_PAIR(fnGetDehazeAttr, fnSetDehazeAttr, v5_isp_dehaze_attr, "dehaze_attr");
    V5_TUNE_PAIR(fnGetSharpenAttr, fnSetSharpenAttr, v5_isp_sharpen_attr, "sharpen_attr");
    V5_TUNE_PAIR(fnGetDpDynamicAttr, fnSetDpDynamicAttr, v5_isp_dp_dynamic_attr, "dp_dynamic_attr");
    V5_TUNE_PAIR(fnGetGammaAttr, fnSetGammaAttr, v5_isp_gamma_attr, "gamma_attr");
    V5_TUNE_PAIR(fnGetPregammaAttr, fnSetPregammaAttr, v5_isp_pregamma_attr, "pregamma_attr");
    V5_TUNE_PAIR(fnGetBlackLevelAttr, fnSetBlackLevelAttr, v5_isp_blc_attr, "black_level_attr");
    V5_TUNE_PAIR(fnGetDemosaicAttr, fnSetDemosaicAttr, v5_isp_demosaic_attr, "demosaic_attr");
    V5_TUNE_PAIR(fnGetCscAttr, fnSetCscAttr, v5_isp_csc_attr, "csc_attr");
    V5_TUNE_PAIR(fnGetCaAttr, fnSetCaAttr, v5_isp_ca_attr, "ca_attr");
    V5_TUNE_PAIR(fnGetAntiFalseColorAttr, fnSetAntiFalseColorAttr, v5_isp_anti_false_color_attr,
                 "anti_false_color_attr");
    V5_TUNE_PAIR(fnGetShadingAttr, fnSetShadingAttr, v5_isp_shading_attr, "mesh_shading_attr");
    V5_TUNE_PAIR(fnGetCacAttr, fnSetCacAttr, v5_isp_cac_attr, "cac_attr");

#undef V5_TUNE_PAIR
}

static inline void v5_isp_tune_unload(v5_isp_tune_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_ISP_TUNE_H */
