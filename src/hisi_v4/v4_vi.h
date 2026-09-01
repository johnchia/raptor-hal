/*
 * hisi_v4/v4_vi.h -- HI_MPI_VI bindings, HiMPP V4.0
 *
 * VI is three nested objects on gen4, and the nesting is the thing to hold on
 * to while reading: a *device* takes the pixels off the MIPI receiver, one or
 * more *pipes* hang off a device and each carries an ISP, and *channels* hang
 * off a pipe and emit frames. raptor drives one of each -- HISI_VI_DEV,
 * HISI_VI_PIPE, HISI_VI_CHN in hisi_state.h, all 0 -- but the API takes all
 * three indices everywhere, so the distinction cannot be collapsed away.
 *
 * The pipe index is also the ISP index. There is no separate ISP device on
 * gen4; every HI_MPI_ISP_*, HI_MPI_AE_* and HI_MPI_AWB_* call is keyed on
 * VI_PIPE. See v4_isp.h.
 *
 * PROVENANCE. Layouts derived from the Hi3516EV200 SDK V1.0.1.0 headers
 * (mpp/include/hi_comm_vi.h) and cross-checked against
 * ref/openhisilicon/include/comm_vi.h (GPL v3). Sizes and offsets in the
 * _Static_asserts were read out of a probe compiled against the SDK headers
 * with arm-openipc-linux-musleabi-gcc, never counted by hand.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_VI_H
#define HISI_V4_VI_H

#include "v4_common.h"
#include "v4_video.h"

/* ================================================================
 * ENUMS
 *
 * Only the values this backend can produce are named. Each is spelled with
 * its numeric value because these are ABI, and a gap in a transcribed
 * enumerator list silently renumbers everything after it -- the single most
 * likely way for a hand-written binding to be wrong in a way that still
 * compiles.
 * ================================================================ */

/* VI_INTF_MODE_E. MIPI is 6, not 0: the enum starts at the parallel BT.656
 * and digital-camera modes gen4 silicon no longer has pins for. */
typedef enum {
    V4_VI_MODE_MIPI = 6,
    V4_VI_MODE_LVDS = 10,
} v4_vi_intf_mode;

/* VI_WORK_MODE_E */
typedef enum {
    V4_VI_WORK_1MULTIPLEX = 0,
} v4_vi_work_mode;

/* VI_SCAN_MODE_E */
typedef enum {
    V4_VI_SCAN_INTERLACED = 0,
    V4_VI_SCAN_PROGRESSIVE = 1,
} v4_vi_scan_mode;

/* VI_DATA_TYPE_E. Counter-intuitive on a raw sensor: RGB is the value a
 * Bayer pipeline wants, because "YUV" here means the device is fed
 * already-converted data. Every stock sensor INI on the board sets
 * InputDataType=1. */
typedef enum {
    V4_VI_DATA_TYPE_YUV = 0,
    V4_VI_DATA_TYPE_RGB = 1,
} v4_vi_data_type;

/* DATA_RATE_E */
typedef enum {
    V4_DATA_RATE_X1 = 0,
    V4_DATA_RATE_X2 = 1,
} v4_data_rate;

/* VI_PIPE_BYPASS_MODE_E */
typedef enum {
    V4_VI_PIPE_BYPASS_NONE = 0,
} v4_vi_pipe_bypass_mode;

/* VI_NR_REF_SOURCE_E */
typedef enum {
    V4_VI_NR_REF_FROM_RFR = 0,
    V4_VI_NR_REF_FROM_CHN0 = 1,
} v4_vi_nr_ref_source;

/* ================================================================
 * DEVICE
 * ================================================================ */

typedef struct {
    unsigned int hsync_hfb;
    unsigned int hsync_act;
    unsigned int hsync_hbb;
    unsigned int vsync_vfb;
    unsigned int vsync_vact;
    unsigned int vsync_vbb;
    unsigned int vsync_vbfb;
    unsigned int vsync_vbact;
    unsigned int vsync_vbbb;
} v4_vi_timing_blank;

/*
 * VI_SYNC_CFG_S. Six sync-polarity enums and the blanking table.
 *
 * Every field is dead on a MIPI sensor -- the header says so, "must be
 * configured in BT.601 mode or DC mode" -- and the stock INIs fill it in
 * anyway. raptor carries the struct so the layout is right and reads the
 * values from the sensor INI where they exist, on the principle that a field
 * the vendor's own configuration bothers to set is not one to zero on a
 * guess.
 */
typedef struct {
    unsigned int vsync;
    unsigned int vsync_neg;
    unsigned int hsync;
    unsigned int hsync_neg;
    unsigned int vsync_valid;
    unsigned int vsync_valid_neg;
    v4_vi_timing_blank timing_blank;
} v4_vi_sync_cfg;

_Static_assert(sizeof(v4_vi_sync_cfg) == 60, "VI_SYNC_CFG_S is 60 bytes");

typedef struct {
    v4_size bas_size;
    unsigned int h_rephase_mode;
    unsigned int v_rephase_mode;
} v4_vi_bas_attr;

typedef struct {
    unsigned int wdr_mode;
    unsigned int cache_line;
} v4_vi_wdr_attr;

#define V4_VI_COMPMASK_NUM 2
#define V4_VI_MAX_ADCHN_NUM 4

typedef struct {
    v4_vi_intf_mode intf_mode;
    v4_vi_work_mode work_mode;
    unsigned int component_mask[V4_VI_COMPMASK_NUM];
    v4_vi_scan_mode scan_mode;
    int ad_chn_id[V4_VI_MAX_ADCHN_NUM];
    unsigned int data_seq;
    v4_vi_sync_cfg sync_cfg;
    v4_vi_data_type input_data_type;
    int data_reverse;
    v4_size size;
    v4_vi_bas_attr bas_attr;
    v4_vi_wdr_attr wdr_attr;
    v4_data_rate data_rate;
} v4_vi_dev_attr;

_Static_assert(sizeof(v4_vi_dev_attr) == 144, "VI_DEV_ATTR_S is 144 bytes");
_Static_assert(offsetof(v4_vi_dev_attr, sync_cfg) == 40, "VI_DEV_ATTR_S.stSynCfg at +40");
_Static_assert(offsetof(v4_vi_dev_attr, input_data_type) == 100, "enInputDataType at +100");
_Static_assert(offsetof(v4_vi_dev_attr, data_reverse) == 104, "bDataReverse at +104");
_Static_assert(offsetof(v4_vi_dev_attr, size) == 108, "stSize at +108");
_Static_assert(offsetof(v4_vi_dev_attr, bas_attr) == 116, "stBasAttr at +116");
_Static_assert(offsetof(v4_vi_dev_attr, wdr_attr) == 132, "stWDRAttr at +132");
_Static_assert(offsetof(v4_vi_dev_attr, data_rate) == 140, "enDataRate at +140");

/* VI_DEV_BIND_PIPE_S. VI_MAX_PHY_PIPE_NUM is 2 on gen4, and it is the array
 * bound, so it is ABI here. */
#define V4_VI_MAX_PHY_PIPE_NUM 2

typedef struct {
    unsigned int num;
    int pipe_id[V4_VI_MAX_PHY_PIPE_NUM];
} v4_vi_dev_bind_pipe;

_Static_assert(sizeof(v4_vi_dev_bind_pipe) == 12, "VI_DEV_BIND_PIPE_S is 12 bytes");

/* ================================================================
 * PIPE
 * ================================================================ */

typedef struct {
    v4_pixel_format pix_fmt;
    v4_data_bitwidth bit_width;
    v4_vi_nr_ref_source nr_ref_source;
    v4_compress_mode compress_mode;
} v4_vi_nr_attr;

typedef struct {
    v4_vi_pipe_bypass_mode bypass_mode;
    int yuv_skip;
    int isp_bypass;
    unsigned int max_w;
    unsigned int max_h;
    v4_pixel_format pix_fmt;
    v4_compress_mode compress_mode;
    v4_data_bitwidth bit_width;
    int nr_en;
    v4_vi_nr_attr nr_attr;
    int sharpen_en;
    v4_frame_rate frame_rate;
    int discard_pro_pic;
} v4_vi_pipe_attr;

_Static_assert(sizeof(v4_vi_pipe_attr) == 68, "VI_PIPE_ATTR_S is 68 bytes");
_Static_assert(offsetof(v4_vi_pipe_attr, max_w) == 12, "VI_PIPE_ATTR_S.u32MaxW at +12");
_Static_assert(offsetof(v4_vi_pipe_attr, pix_fmt) == 20, "enPixFmt at +20");
_Static_assert(offsetof(v4_vi_pipe_attr, nr_attr) == 36, "stNrAttr at +36");
_Static_assert(offsetof(v4_vi_pipe_attr, frame_rate) == 56, "stFrameRate at +56");
_Static_assert(offsetof(v4_vi_pipe_attr, discard_pro_pic) == 64, "bDiscardProPic at +64");

/* ================================================================
 * CHANNEL
 * ================================================================ */

typedef struct {
    v4_size size;
    v4_pixel_format pixel_format;
    v4_dynamic_range dynamic_range;
    v4_video_format video_format;
    v4_compress_mode compress_mode;
    int mirror;
    int flip;
    unsigned int depth;
    v4_frame_rate frame_rate;
} v4_vi_chn_attr;

_Static_assert(sizeof(v4_vi_chn_attr) == 44, "VI_CHN_ATTR_S is 44 bytes");
_Static_assert(offsetof(v4_vi_chn_attr, pixel_format) == 8, "enPixelFormat at +8");
_Static_assert(offsetof(v4_vi_chn_attr, mirror) == 24, "bMirror at +24");
_Static_assert(offsetof(v4_vi_chn_attr, depth) == 32, "u32Depth at +32");
_Static_assert(offsetof(v4_vi_chn_attr, frame_rate) == 36, "stFrameRate at +36");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    int (*fnSetDevAttr)(int dev, const v4_vi_dev_attr *attr);
    int (*fnGetDevAttr)(int dev, v4_vi_dev_attr *attr);
    int (*fnEnableDev)(int dev);
    int (*fnDisableDev)(int dev);
    int (*fnSetDevBindPipe)(int dev, const v4_vi_dev_bind_pipe *bind);

    int (*fnCreatePipe)(int pipe, const v4_vi_pipe_attr *attr);
    int (*fnDestroyPipe)(int pipe);
    int (*fnStartPipe)(int pipe);
    int (*fnStopPipe)(int pipe);
    int (*fnSetPipeAttr)(int pipe, const v4_vi_pipe_attr *attr);
    int (*fnGetPipeAttr)(int pipe, v4_vi_pipe_attr *attr);

    int (*fnSetChnAttr)(int pipe, int chn, const v4_vi_chn_attr *attr);
    int (*fnGetChnAttr)(int pipe, int chn, v4_vi_chn_attr *attr);
    int (*fnEnableChn)(int pipe, int chn);
    int (*fnDisableChn)(int pipe, int chn);

    /* Raw frame access off the VI channel. Not on the streaming path --
     * frames reach VENC through a kernel-side bind -- but rvd's snapshot
     * and IVS paths ask for pictures by hand. */
    int (*fnGetChnFrame)(int pipe, int chn, v4_video_frame_info *frame, int milli_sec);
    int (*fnReleaseChnFrame)(int pipe, int chn, const v4_video_frame_info *frame);
} v4_vi_impl;

static inline int v4_vi_load(v4_vi_impl *lib, const v4_mpi_libs *libs)
{
    static const char mod[] = "v4_vi";

    memset(lib, 0, sizeof(*lib));

#define V4_VI_REQ(field, type, name)                                                               \
    do {                                                                                           \
        if (!(lib->field = (type)v4_symbol(mod, libs, "HI_MPI_VI_" name, "GK_API_VI_" name)))      \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V4_VI_REQ(fnSetDevAttr, int (*)(int, const v4_vi_dev_attr *), "SetDevAttr");
    V4_VI_REQ(fnEnableDev, int (*)(int), "EnableDev");
    V4_VI_REQ(fnDisableDev, int (*)(int), "DisableDev");
    V4_VI_REQ(fnSetDevBindPipe, int (*)(int, const v4_vi_dev_bind_pipe *), "SetDevBindPipe");
    V4_VI_REQ(fnCreatePipe, int (*)(int, const v4_vi_pipe_attr *), "CreatePipe");
    V4_VI_REQ(fnDestroyPipe, int (*)(int), "DestroyPipe");
    V4_VI_REQ(fnStartPipe, int (*)(int), "StartPipe");
    V4_VI_REQ(fnStopPipe, int (*)(int), "StopPipe");
    V4_VI_REQ(fnSetChnAttr, int (*)(int, int, const v4_vi_chn_attr *), "SetChnAttr");
    V4_VI_REQ(fnEnableChn, int (*)(int, int), "EnableChn");
    V4_VI_REQ(fnDisableChn, int (*)(int, int), "DisableChn");

#undef V4_VI_REQ

    lib->fnGetDevAttr =
        (int (*)(int, v4_vi_dev_attr *))v4_symbol_opt(libs, "HI_MPI_VI_GetDevAttr",
                                                      "GK_API_VI_GetDevAttr");
    lib->fnSetPipeAttr = (int (*)(int, const v4_vi_pipe_attr *))v4_symbol_opt(
        libs, "HI_MPI_VI_SetPipeAttr", "GK_API_VI_SetPipeAttr");
    lib->fnGetPipeAttr = (int (*)(int, v4_vi_pipe_attr *))v4_symbol_opt(
        libs, "HI_MPI_VI_GetPipeAttr", "GK_API_VI_GetPipeAttr");
    lib->fnGetChnAttr = (int (*)(int, int, v4_vi_chn_attr *))v4_symbol_opt(
        libs, "HI_MPI_VI_GetChnAttr", "GK_API_VI_GetChnAttr");
    lib->fnGetChnFrame = (int (*)(int, int, v4_video_frame_info *, int))v4_symbol_opt(
        libs, "HI_MPI_VI_GetChnFrame", "GK_API_VI_GetChnFrame");
    lib->fnReleaseChnFrame = (int (*)(int, int, const v4_video_frame_info *))v4_symbol_opt(
        libs, "HI_MPI_VI_ReleaseChnFrame", "GK_API_VI_ReleaseChnFrame");

    return RSS_OK;
}

static inline void v4_vi_unload(v4_vi_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_VI_H */
