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
    lib->fnReleaseChnFrame = (int (*)(int, int, const v4_video_frame_info *))v4_symbol_opt(
        libs, "HI_MPI_VPSS_ReleaseChnFrame", "GK_API_VPSS_ReleaseChnFrame");

    return RSS_OK;
}

static inline void v4_vpss_unload(v4_vpss_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_VPSS_H */
