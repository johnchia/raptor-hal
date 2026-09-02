/*
 * hisi_v4/v4_vpss.h -- HI_MPI_VPSS bindings, HiMPP V4.0
 *
 * VPSS is the scaler, and on gen4 it is also where 3DNR lives. Two levels:
 * a *group* takes one input picture and owns the noise-reduction state, and
 * its *channels* each emit a scaled copy. raptor maps one framesource onto
 * one VPSS channel of group HISI_VPSS_GRP.
 *
 * The thing to notice, because it differs from every other family raptor
 * supports: **3DNR is a group attribute, not a channel one.** bNrEn and
 * stNrAttr sit in VPSS_GRP_ATTR_S, so noise reduction is a property of the
 * whole fan-out and cannot be varied per stream. SigmaStar puts it on the
 * VPE channel and Ingenic on the framesource; here there is exactly one
 * setting and every stream shares it.
 *
 * PROVENANCE. Derived from the Hi3516EV200 SDK V1.0.1.0 headers
 * (mpp/include/hi_comm_vpss.h) and cross-checked against
 * ref/openhisilicon/include/comm_vpss.h (GPL v3). Offsets from a probe
 * compiled with arm-openipc-linux-musleabi-gcc.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_VPSS_H
#define HISI_V4_VPSS_H

#include "v4_common.h"
#include "v4_video.h"

/*
 * VPSS_MAX_PHY_CHN_NUM is 3 on gen4 and VPSS_MAX_CHN_NUM is 7 -- three
 * physical channels plus four extension channels. This is the SDK's own
 * statement for this chip, and it is the prediction the create-0..5
 * experiment in risk R6 exists to test: hal_caps.c publishes the *observed*
 * 2, because two is what a live board was seen running, and a header
 * constant is not a measurement. Sizing arrays by the header bound is a
 * different question from promising channels through caps, and this is the
 * bound.
 */
#define V4_VPSS_MAX_PHY_CHN_NUM 3
#define V4_VPSS_MAX_CHN_NUM 7

/* VPSS_CHN_MODE_E. USER means the channel's width and height are what the
 * caller asked for; AUTO makes it follow the group. Every channel raptor
 * creates is a specific stream resolution, so every one is USER. */
typedef enum {
    V4_VPSS_CHN_MODE_USER = 0,
    V4_VPSS_CHN_MODE_AUTO = 1,
} v4_vpss_chn_mode;

/* VPSS_NR_TYPE_E */
typedef enum {
    V4_VPSS_NR_TYPE_VIDEO = 0,
    V4_VPSS_NR_TYPE_SNAP = 1,
} v4_vpss_nr_type;

/* NR_MOTION_MODE_E */
typedef enum {
    V4_NR_MOTION_MODE_NORMAL = 0,
    V4_NR_MOTION_MODE_COMPENSATE = 1,
} v4_nr_motion_mode;

typedef struct {
    v4_vpss_nr_type nr_type;
    v4_compress_mode compress_mode;
    v4_nr_motion_mode motion_mode;
} v4_vpss_nr_attr;

_Static_assert(sizeof(v4_vpss_nr_attr) == 12, "VPSS_NR_ATTR_S is 12 bytes");

typedef struct {
    unsigned int max_w;
    unsigned int max_h;
    v4_pixel_format pixel_format;
    v4_dynamic_range dynamic_range;
    v4_frame_rate frame_rate;
    int nr_en;
    v4_vpss_nr_attr nr_attr;
} v4_vpss_grp_attr;

_Static_assert(sizeof(v4_vpss_grp_attr) == 40, "VPSS_GRP_ATTR_S is 40 bytes");
_Static_assert(offsetof(v4_vpss_grp_attr, frame_rate) == 16, "stFrameRate at +16");
_Static_assert(offsetof(v4_vpss_grp_attr, nr_en) == 24, "bNrEn at +24");
_Static_assert(offsetof(v4_vpss_grp_attr, nr_attr) == 28, "stNrAttr at +28");

typedef struct {
    v4_vpss_chn_mode chn_mode;
    unsigned int width;
    unsigned int height;
    v4_video_format video_format;
    v4_pixel_format pixel_format;
    v4_dynamic_range dynamic_range;
    v4_compress_mode compress_mode;
    v4_frame_rate frame_rate;
    int mirror;
    int flip;
    unsigned int depth;
    v4_aspect_ratio aspect_ratio;
} v4_vpss_chn_attr;

_Static_assert(sizeof(v4_vpss_chn_attr) == 72, "VPSS_CHN_ATTR_S is 72 bytes");
_Static_assert(offsetof(v4_vpss_chn_attr, width) == 4, "u32Width at +4");
_Static_assert(offsetof(v4_vpss_chn_attr, video_format) == 12, "enVideoFormat at +12");
_Static_assert(offsetof(v4_vpss_chn_attr, frame_rate) == 28, "stFrameRate at +28");
_Static_assert(offsetof(v4_vpss_chn_attr, mirror) == 36, "bMirror at +36");
_Static_assert(offsetof(v4_vpss_chn_attr, depth) == 44, "u32Depth at +44");
_Static_assert(offsetof(v4_vpss_chn_attr, aspect_ratio) == 48, "stAspectRatio at +48");

/* VPSS_CROP_COORDINATE_E: whether crop_rect is in units of 1/1000ths of the
 * source or in pixels. raptor's framesource crop is in pixels. */
typedef enum {
    V4_VPSS_CROP_RATIO = 0,
    V4_VPSS_CROP_ABS = 1,
} v4_vpss_crop_coord;

typedef struct {
    int enable;
    unsigned int crop_coordinate;
    v4_rect crop_rect;
} v4_vpss_crop_info;

_Static_assert(sizeof(v4_vpss_crop_info) == 24, "VPSS_CROP_INFO_S is 24 bytes");

/* ================================================================
 * 3DNR X-PARAMS -- VPSS_GRP_NRX_PARAM_S, the V3 shape (Hi3516EV200)
 * ================================================================
 *
 * PROVENANCE. mpp/include/hi_comm_vpss.h from the Hi3516EV200 SDK
 * V1.0.1.0, the tV200_* block ("Only used for Hi3516EV200"). Sizes and
 * the non-bitfield offsets from a probe compiled with
 * arm-openipc-linux-musleabi-gcc and read back with nm -S; the bitfields
 * are transcribed in the vendor's order with the vendor's storage types,
 * which is what fixes their layout under the EABI. The driver's own
 * /proc/umap/vpss reports "Intf NR_X, Version VER_3" on the EV300.
 *
 * VPSS_GRP_NRX_PARAM_S carries a union of the V1/V2/V3 parameter sets;
 * V3 is the largest (940 bytes) and the only one this backend writes, so
 * the union is transcribed as V3 alone and the total is pinned.
 *
 * Field names are the vendor's, lower-cased, because the tuning text
 * refers to them by those names (-nXsf1 is SFS1:SFT1:SBR1, and so on)
 * and a reader with the INI beside them should not need a dictionary.
 */

typedef struct {
    unsigned char ies0, ies1, ies2, ies3;
    unsigned short iedz : 10, ie_en : 1, rb : 5;
} v4_vpss_nrx_iey;

typedef struct {
    unsigned char spn6 : 3, sfr : 5;
    unsigned char sbn6 : 3, pbr6 : 5;
    unsigned short srt0 : 5, srt1 : 5, jmode : 3, deidx : 3;
    unsigned char sfr6[4], sbr6[2], derate;
    unsigned char sfs1, sft1, sbr1;
    unsigned char sfs2, sft2, sbr2;
    unsigned char sfs4, sft4, sbr4;
    unsigned short sth1 : 9, sfn1 : 3, sfn0 : 3, nry_en : 1;
    unsigned short sthd1 : 9, rb0 : 7;
    unsigned short sth2 : 9, sfn2 : 3, kmode : 3, rb1 : 1;
    unsigned short sthd2 : 9, rb2 : 7;
    unsigned short sbsk[32], sdsk[32];
} v4_vpss_nrx_sfy;

typedef struct {
    unsigned short tfs0 : 4, tdz0 : 10, tdx0 : 2;
    unsigned short tfs1 : 4, tdz1 : 10, tdx1 : 2;
    unsigned short sdz0 : 10, str0 : 5, dzmode0 : 1;
    unsigned short sdz1 : 10, str1 : 5, dzmode1 : 1;
    unsigned char tss0 : 4, tsi0 : 4, tfr0[6];
    unsigned char tss1 : 4, tsi1 : 4, tfr1[6];
    unsigned char tfrs : 4, ted : 2, bref : 1, rb : 1;
} v4_vpss_nrx_tfy;

typedef struct {
    unsigned short madz0 : 9, mai00 : 2, mai01 : 2, mai02 : 2, rb0 : 1;
    unsigned short madz1 : 9, mai10 : 2, mai11 : 2, mai12 : 2, rb1 : 1;
    unsigned char mabr0, mabr1;
    unsigned short math0 : 10, mate0 : 4, matw : 2;
    unsigned short mathd0 : 10, rb2 : 6;
    unsigned short math1 : 10, rb3 : 6;
    unsigned short mathd1 : 10, rb4 : 6;
    unsigned char masw : 4, mate1 : 4;
    unsigned char mabw0 : 4, mabw1 : 4;
    unsigned short advmath : 1, advth : 12, rb5 : 3;
} v4_vpss_nrx_mdy;

typedef struct {
    unsigned char sfc, tfc : 6, rb0 : 2;
    unsigned char trc, tpc : 6, rb1 : 2;
} v4_vpss_nrx_nrc;

typedef struct {
    v4_vpss_nrx_iey iey[5];
    v4_vpss_nrx_sfy sfy[5];
    v4_vpss_nrx_mdy mdy[2];
    v4_vpss_nrx_tfy tfy[3];
    v4_vpss_nrx_nrc nrc;
} v4_vpss_nrx_v3; /* VPSS_NRX_V3_S */

_Static_assert(sizeof(v4_vpss_nrx_iey) == 6, "tV200_VPSS_IEy is 6 bytes");
_Static_assert(sizeof(v4_vpss_nrx_sfy) == 156, "tV200_VPSS_SFy is 156 bytes");
_Static_assert(offsetof(v4_vpss_nrx_sfy, sfr6) == 4, "SFR6 at +4");
_Static_assert(offsetof(v4_vpss_nrx_sfy, sbr6) == 8, "SBR6 at +8");
_Static_assert(offsetof(v4_vpss_nrx_sfy, derate) == 10, "DeRate at +10");
_Static_assert(offsetof(v4_vpss_nrx_sfy, sfs1) == 11, "SFS1 at +11");
_Static_assert(offsetof(v4_vpss_nrx_sfy, sfs4) == 17, "SFS4 at +17");
_Static_assert(offsetof(v4_vpss_nrx_sfy, sbsk) == 28, "SBSk at +28");
_Static_assert(offsetof(v4_vpss_nrx_sfy, sdsk) == 92, "SDSk at +92");
_Static_assert(sizeof(v4_vpss_nrx_tfy) == 24, "tV200_VPSS_TFy is 24 bytes");
_Static_assert(offsetof(v4_vpss_nrx_tfy, tfr0) == 9, "TFR0 at +9");
_Static_assert(offsetof(v4_vpss_nrx_tfy, tfr1) == 16, "TFR1 at +16");
_Static_assert(sizeof(v4_vpss_nrx_mdy) == 18, "tV200_VPSS_MDy is 18 bytes");
_Static_assert(offsetof(v4_vpss_nrx_mdy, mabr0) == 4, "MABR0 at +4");
_Static_assert(sizeof(v4_vpss_nrx_nrc) == 4, "tV200_VPSS_NRc is 4 bytes");
_Static_assert(offsetof(v4_vpss_nrx_nrc, trc) == 2, "TRC at +2");
_Static_assert(sizeof(v4_vpss_nrx_v3) == 922, "VPSS_NRX_V3_S is 922 bytes");
_Static_assert(offsetof(v4_vpss_nrx_v3, sfy) == 30, "SFy at +30");
_Static_assert(offsetof(v4_vpss_nrx_v3, mdy) == 810, "MDy at +810");
_Static_assert(offsetof(v4_vpss_nrx_v3, tfy) == 846, "TFy at +846");
_Static_assert(offsetof(v4_vpss_nrx_v3, nrc) == 918, "NRc at +918");

/* OPERATION_MODE_E, hi_comm_video.h. */
#define V4_OPERATION_MODE_AUTO 0
#define V4_OPERATION_MODE_MANUAL 1

/* VPSS_NR_VER_E. */
#define V4_VPSS_NR_V3 3

/* The most blocks one tuning file may carry; the SDK's own loader caps
 * at 16 (HI_SCENE_3DNR_MAX_COUNT) and the shipped files carry 9. */
#define V4_VPSS_NRX_MAX_BLOCKS 16

typedef struct {
    unsigned int param_num;   /* u32ParamNum */
    unsigned int *iso;        /* pau32ISO, param_num entries, ascending */
    v4_vpss_nrx_v3 *params;   /* pastNRXParam, param_num entries */
} v4_vpss_nrx_auto_v3;

typedef struct {
    int opt_mode; /* enOptMode: V4_OPERATION_MODE_* */
    v4_vpss_nrx_v3 manual;    /* stNRXManual.stNRXParam */
    v4_vpss_nrx_auto_v3 auto_; /* stNRXAuto */
} v4_vpss_nrx_param_v3;

typedef struct {
    int nr_ver; /* enNRVer: V4_VPSS_NR_V3 */
    v4_vpss_nrx_param_v3 v3;
} v4_vpss_grp_nrx_param; /* VPSS_GRP_NRX_PARAM_S, V3 member of the union */

_Static_assert(sizeof(v4_vpss_nrx_auto_v3) == 12, "VPSS_NRX_PARAM_AUTO_V3_S is 12 bytes");
_Static_assert(offsetof(v4_vpss_nrx_auto_v3, iso) == 4, "pau32ISO at +4");
_Static_assert(offsetof(v4_vpss_nrx_auto_v3, params) == 8, "pastNRXParam at +8");
_Static_assert(sizeof(v4_vpss_nrx_param_v3) == 940, "VPSS_NRX_PARAM_V3_S is 940 bytes");
_Static_assert(offsetof(v4_vpss_nrx_param_v3, manual) == 4, "stNRXManual at +4");
_Static_assert(offsetof(v4_vpss_nrx_param_v3, auto_) == 928, "stNRXAuto at +928");
_Static_assert(sizeof(v4_vpss_grp_nrx_param) == 944, "VPSS_GRP_NRX_PARAM_S is 944 bytes");
_Static_assert(offsetof(v4_vpss_grp_nrx_param, v3) == 4, "stNRXParam_V3 at +4");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    int (*fnCreateGrp)(int grp, const v4_vpss_grp_attr *attr);
    int (*fnDestroyGrp)(int grp);
    int (*fnStartGrp)(int grp);
    int (*fnStopGrp)(int grp);
    int (*fnSetGrpAttr)(int grp, const v4_vpss_grp_attr *attr);
    int (*fnGetGrpAttr)(int grp, v4_vpss_grp_attr *attr);
    int (*fnSetGrpCrop)(int grp, const v4_vpss_crop_info *crop);

    int (*fnSetChnAttr)(int grp, int chn, const v4_vpss_chn_attr *attr);
    int (*fnGetChnAttr)(int grp, int chn, v4_vpss_chn_attr *attr);
    int (*fnEnableChn)(int grp, int chn);
    int (*fnDisableChn)(int grp, int chn);
    int (*fnSetChnCrop)(int grp, int chn, const v4_vpss_crop_info *crop);

    /* Userspace frame access on a channel. Needs the channel's u32Depth to
     * be non-zero, which is why hal_fs_set_frame_depth is a real op here
     * rather than bookkeeping: with depth 0 the channel queues nothing for
     * userspace and GetChnFrame times out forever. */
    int (*fnGetChnFrame)(int grp, int chn, v4_video_frame_info *frame, int milli_sec);
    int (*fnReleaseChnFrame)(int grp, int chn, const v4_video_frame_info *frame);
    /* 3DNR X-params; optional, hal_nrx.c degrades to "untuned" without them. */
    int (*fnSetGrpNRXParam)(int grp, const v4_vpss_grp_nrx_param *param);
    int (*fnGetGrpNRXParam)(int grp, v4_vpss_grp_nrx_param *param);
} v4_vpss_impl;

static inline int v4_vpss_load(v4_vpss_impl *lib, const v4_mpi_libs *libs)
{
    static const char mod[] = "v4_vpss";

    memset(lib, 0, sizeof(*lib));

#define V4_VPSS_REQ(field, type, name)                                                             \
    do {                                                                                           \
        if (!(lib->field = (type)v4_symbol(mod, libs, "HI_MPI_VPSS_" name, "GK_API_VPSS_" name)))  \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V4_VPSS_REQ(fnCreateGrp, int (*)(int, const v4_vpss_grp_attr *), "CreateGrp");
    V4_VPSS_REQ(fnDestroyGrp, int (*)(int), "DestroyGrp");
    V4_VPSS_REQ(fnStartGrp, int (*)(int), "StartGrp");
    V4_VPSS_REQ(fnStopGrp, int (*)(int), "StopGrp");
    V4_VPSS_REQ(fnSetChnAttr, int (*)(int, int, const v4_vpss_chn_attr *), "SetChnAttr");
    V4_VPSS_REQ(fnEnableChn, int (*)(int, int), "EnableChn");
    V4_VPSS_REQ(fnDisableChn, int (*)(int, int), "DisableChn");

#undef V4_VPSS_REQ

    lib->fnSetGrpAttr = (int (*)(int, const v4_vpss_grp_attr *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_SetGrpAttr", "GK_API_VPSS_SetGrpAttr");
    lib->fnGetGrpAttr = (int (*)(int, v4_vpss_grp_attr *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_GetGrpAttr", "GK_API_VPSS_GetGrpAttr");
    lib->fnSetGrpCrop = (int (*)(int, const v4_vpss_crop_info *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_SetGrpCrop", "GK_API_VPSS_SetGrpCrop");
    lib->fnGetChnAttr = (int (*)(int, int, v4_vpss_chn_attr *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_GetChnAttr", "GK_API_VPSS_GetChnAttr");
    lib->fnSetChnCrop = (int (*)(int, int, const v4_vpss_crop_info *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_SetChnCrop", "GK_API_VPSS_SetChnCrop");
    lib->fnGetChnFrame = (int (*)(int, int, v4_video_frame_info *, int))v4_symbol_opt(
        libs, "HI_MPI_VPSS_GetChnFrame", "GK_API_VPSS_GetChnFrame");
    lib->fnSetGrpNRXParam = (int (*)(int, const v4_vpss_grp_nrx_param *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_SetGrpNRXParam", "GK_API_VPSS_SetGrpNRXParam");
    lib->fnGetGrpNRXParam = (int (*)(int, v4_vpss_grp_nrx_param *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_GetGrpNRXParam", "GK_API_VPSS_GetGrpNRXParam");
    lib->fnReleaseChnFrame = (int (*)(int, int, const v4_video_frame_info *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_ReleaseChnFrame", "GK_API_VPSS_ReleaseChnFrame");

    return RSS_OK;
}

static inline void v4_vpss_unload(v4_vpss_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_VPSS_H */
