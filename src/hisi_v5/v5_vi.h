/*
 * hisi_v5/v5_vi.h -- ss_mpi_vi bindings, HiMPP V5.0
 *
 * VI is the sensor's end of the pipeline: a *device* takes the MIPI
 * receiver's output, a *pipe* runs the ISP over it, and a *channel* hands
 * YUV to whatever is bound downstream. Three objects and three attribute
 * structs, where gen4 had the same three and a different shape for each.
 *
 * WHAT CHANGED FROM gen4, and it is more than names:
 *
 *   - The device-to-pipe binding is explicit. gen4 had VI_PIPE == VI_DEV by
 *     construction; V5 has ss_mpi_vi_bind(dev, pipe), which must be called
 *     between enable_dev and create_pipe. Omit it and create_pipe succeeds,
 *     the pipe never receives a frame, and /proc/umap/vi shows an empty
 *     "vi bind attr" table with no error anywhere.
 *   - ot_vi_pipe_attr gained a bypass mode and an isp_bypass flag that gen4's
 *     VI_PIPE_ATTR_S has no counterpart for, and lost the WDR and NR fields
 *     that moved to their own calls.
 *   - Mirror and flip moved *into* the channel (ot_vi_chn_attr.mirror_en /
 *     flip_en) from the sensor object's pfn_mirror_flip. Both still exist;
 *     the channel's is the one a running pipeline can change.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_vi.h at
 * 1.0.2.0 B051. Sizes and offsets from a probe compiled against it with the
 * cv6xx cross-compiler:
 *
 *   ot_vi_timing_blank  36    ot_vi_sync_cfg     60
 *   ot_vi_dev_attr     120    sync_cfg at +40, in_size at +108
 *   ot_vi_pipe_attr     32    size at +8, frame_rate_ctrl at +24
 *   ot_vi_chn_attr      44    mirror_en at +24, depth at +32
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_VI_H
#define HISI_V5_VI_H

#include "v5_common.h"
#include "v5_video.h"

/* ot_defines.h:238-251. Bounds, not choices: the topology raptor drives is
 * in hisi_state.h. OT_VI_MAX_PIPE_NUM is 4 because two of the pipes are
 * virtual (WDR fusion and stitching), which is also why v5_sys.h's
 * ot_vi_vpss_mode is 16 bytes and gen4's was 8. */
#define V5_VI_MAX_DEV_NUM 2
#define V5_VI_MAX_PHYS_PIPE_NUM 2
#define V5_VI_MAX_PHYS_CHN_NUM 1
#define V5_VI_MAX_EXT_CHN_NUM 1

/* ================================================================
 * DEVICE
 * ================================================================ */

/* ot_vi_intf_mode. MIPI is the only one this backend selects; the parallel
 * and BT modes are for sources no OpenIPC hi3516cv6xx board has. */
typedef enum {
    V5_VI_INTF_MODE_BT656 = 0,
    V5_VI_INTF_MODE_BT601 = 1,
    V5_VI_INTF_MODE_DC = 2,
    V5_VI_INTF_MODE_BT1120 = 3,
    V5_VI_INTF_MODE_MIPI = 4,
    V5_VI_INTF_MODE_MIPI_YUV420_NORM = 5,
    V5_VI_INTF_MODE_MIPI_YUV420_LEGACY = 6,
    V5_VI_INTF_MODE_MIPI_YUV422 = 7,
} v5_vi_intf_mode;

/* ot_vi_work_mode. MULTIPLEX_1 is one sensor on the device, which is this
 * backend's whole topology. */
typedef enum {
    V5_VI_WORK_MODE_MULTIPLEX_1 = 0,
    V5_VI_WORK_MODE_MULTIPLEX_2 = 1,
    V5_VI_WORK_MODE_MULTIPLEX_4 = 2,
} v5_vi_work_mode;

typedef enum {
    V5_VI_SCAN_PROGRESSIVE = 0,
    V5_VI_SCAN_INTERLACED = 1,
} v5_vi_scan_mode;

/* ot_vi_data_seq. Meaningful for YUV input only; a raw sensor ignores it,
 * and every sensor this backend drives is raw. */
typedef enum {
    V5_VI_DATA_SEQ_VUVU = 0,
    V5_VI_DATA_SEQ_UVUV = 1,
    V5_VI_DATA_SEQ_UYVY = 2,
    V5_VI_DATA_SEQ_VYUY = 3,
    V5_VI_DATA_SEQ_YUYV = 4,
    V5_VI_DATA_SEQ_YVYU = 5,
} v5_vi_data_seq;

/* ot_vi_data_type. RAW for a Bayer sensor through the ISP; YUV for a
 * source that arrives already converted. */
typedef enum {
    V5_VI_DATA_TYPE_RAW = 0,
    V5_VI_DATA_TYPE_YUV = 1,
} v5_vi_data_type;

/* ot_data_rate. Pixels per clock into the device, matching the MIPI
 * receiver's own data_rate. */
typedef enum {
    V5_DATA_RATE_X1 = 0,
    V5_DATA_RATE_X2 = 1,
} v5_data_rate;

/*
 * ot_vi_timing_blank, nine u32.
 *
 * Zeroed throughout this backend and transcribed whole anyway: it is 36 of
 * ot_vi_sync_cfg's 60 bytes, so a short version would put data_type,
 * in_size and data_rate at the wrong offsets in ot_vi_dev_attr. The fields
 * only mean anything in BT.601 and DC modes, where the host has to describe
 * the timing the source is not sending.
 */
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
} v5_vi_timing_blank;

_Static_assert(sizeof(v5_vi_timing_blank) == 36, "ot_vi_timing_blank is 36 bytes");

/*
 * ot_vi_sync_cfg. Six synchronisation-polarity enums then the blanking.
 * Meaningful in BT.601 and DC mode only, per the header; zeroed for MIPI,
 * which carries its own framing.
 */
typedef struct {
    int vsync;
    int vsync_neg;
    int hsync;
    int hsync_neg;
    int vsync_valid;
    int vsync_valid_neg;
    v5_vi_timing_blank timing_blank;
} v5_vi_sync_cfg;

_Static_assert(sizeof(v5_vi_sync_cfg) == 60, "ot_vi_sync_cfg is 60 bytes");

/* ot_defines.h. Array bounds inside ot_vi_dev_attr, so ABI. */
#define V5_VI_COMPONENT_MASK_NUM 2
#define V5_VI_MAX_AD_CHN_NUM 4

/*
 * ot_vi_dev_attr.
 *
 * ad_chn_id is four *signed* ints and the vendor's comment says "the
 * default value -1 is recommended" -- it is an AD-converter channel map for
 * analogue sources, and -1 in all four is what a MIPI device wants.
 * Leaving it zeroed selects AD channel 0 four times over, which is a
 * different configuration that happens to work on a device with no AD.
 */
typedef struct {
    v5_vi_intf_mode intf_mode;
    v5_vi_work_mode work_mode;
    unsigned int component_mask[V5_VI_COMPONENT_MASK_NUM];
    v5_vi_scan_mode scan_mode;
    int ad_chn_id[V5_VI_MAX_AD_CHN_NUM];
    v5_vi_data_seq data_seq;
    v5_vi_sync_cfg sync_cfg;
    v5_vi_data_type data_type;
    int data_reverse;
    v5_size in_size;
    v5_data_rate data_rate;
} v5_vi_dev_attr;

_Static_assert(sizeof(v5_vi_dev_attr) == 120, "ot_vi_dev_attr is 120 bytes");
_Static_assert(offsetof(v5_vi_dev_attr, sync_cfg) == 40, "ot_vi_dev_attr.sync_cfg at +40");
_Static_assert(offsetof(v5_vi_dev_attr, data_type) == 100, "ot_vi_dev_attr.data_type at +100");
_Static_assert(offsetof(v5_vi_dev_attr, in_size) == 108, "ot_vi_dev_attr.in_size at +108");
_Static_assert(offsetof(v5_vi_dev_attr, data_rate) == 116, "ot_vi_dev_attr.data_rate at +116");

/* ================================================================
 * PIPE
 * ================================================================ */

/*
 * ot_vi_pipe_bypass_mode. New on V5 and it does what its name says: FE
 * bypasses the front end, BE the back end. NONE is the full ISP path and
 * the only one this backend uses -- the bypasses exist for a raw-dump
 * workflow, and choosing one by accident produces a picture with no
 * processing rather than an error.
 */
typedef enum {
    V5_VI_PIPE_BYPASS_NONE = 0,
    V5_VI_PIPE_BYPASS_FE = 1,
    V5_VI_PIPE_BYPASS_BE = 2,
} v5_vi_pipe_bypass_mode;

/*
 * ot_3dnr_attr (ot_common_video.h:1173-1178): the pipe's 3DNR switch.
 * Read back from a CV608 at bring-up: enable 0, nr_type NORM, compress
 * FRAME, motion NORM -- so the pipe is created ready for it and only the
 * enable is missing. Turning it on allocates the reference frames in MMZ
 * (vi(0)_3dnr_ref, _mad, _stt in the tuning guide's list), which is what
 * it answers OT_ERR_NO_MEM about when there is none left.
 */
typedef struct {
    int enable;         /* td_bool */
    int nr_type;        /* ot_nr_type: 0 VIDEO_NORM */
    int compress_mode;  /* ot_compress_mode: 5 FRAME is the only one 3DNR takes */
    int nr_motion_mode; /* ot_nr_motion_mode: 0 NORM */
} v5_3dnr_attr;

_Static_assert(sizeof(v5_3dnr_attr) == 16, "ot_3dnr_attr is 16 bytes");

#define V5_NR_TYPE_VIDEO_NORM 0
#define V5_NR_MOTION_MODE_NORM 0

/*
 * ot_vi_pipe_attr.
 *
 * isp_bypass is the second bypass and it is not the same switch as
 * pipe_bypass_mode: it turns the ISP off while leaving the pipe's data path
 * intact. Both are false/NONE here.
 *
 * pixel_format is a *Bayer* format on this struct -- the pipe's input is
 * raw -- while ot_vi_chn_attr's is YUV. Same enum, opposite ends of the
 * ISP, and the pair is the easiest thing in this header to get backwards.
 */
typedef struct {
    v5_vi_pipe_bypass_mode pipe_bypass_mode;
    int isp_bypass;
    v5_size size;
    v5_pixel_format pixel_format;
    v5_compress_mode compress_mode;
    v5_frame_rate_ctrl frame_rate_ctrl;
} v5_vi_pipe_attr;

_Static_assert(sizeof(v5_vi_pipe_attr) == 32, "ot_vi_pipe_attr is 32 bytes");
_Static_assert(offsetof(v5_vi_pipe_attr, size) == 8, "ot_vi_pipe_attr.size at +8");
_Static_assert(offsetof(v5_vi_pipe_attr, frame_rate_ctrl) == 24,
               "ot_vi_pipe_attr.frame_rate_ctrl at +24");

/* ================================================================
 * CHANNEL
 * ================================================================ */

/*
 * ot_vi_chn_attr.
 *
 * depth is the number of frames the channel holds for a userspace reader,
 * range [0, 8], and **0 is correct for a bound channel**: a depth above
 * zero makes VI queue frames for ss_mpi_vi_get_chn_frame, and a pipeline
 * that binds VI to VPSS and never reads them fills the queue and stalls.
 * gen4 learned this on VPSS; the rule is the same one stage earlier here.
 */
typedef struct {
    v5_size size;
    v5_pixel_format pixel_format;
    v5_dynamic_range dynamic_range;
    v5_video_format video_format;
    v5_compress_mode compress_mode;
    int mirror_en;
    int flip_en;
    unsigned int depth;
    v5_frame_rate_ctrl frame_rate_ctrl;
} v5_vi_chn_attr;

_Static_assert(sizeof(v5_vi_chn_attr) == 44, "ot_vi_chn_attr is 44 bytes");
_Static_assert(offsetof(v5_vi_chn_attr, mirror_en) == 24, "ot_vi_chn_attr.mirror_en at +24");
_Static_assert(offsetof(v5_vi_chn_attr, depth) == 32, "ot_vi_chn_attr.depth at +32");
_Static_assert(offsetof(v5_vi_chn_attr, frame_rate_ctrl) == 36,
               "ot_vi_chn_attr.frame_rate_ctrl at +36");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    /* Device. */
    int (*fnSetDevAttr)(int dev, const v5_vi_dev_attr *attr);
    int (*fnEnableDev)(int dev);
    int (*fnDisableDev)(int dev);

    /*
     * Device -> pipe. Explicit on V5 and required; see the file comment.
     * One edge per call -- ss_mpi_vi.h:41 -- so a WDR fusion configuration
     * binding one device to several pipes calls it once per pipe. This
     * backend binds exactly one.
     */
    int (*fnBind)(int dev, int pipe);
    int (*fnUnbind)(int dev, int pipe);

    /* Pipe. */
    int (*fnCreatePipe)(int pipe, const v5_vi_pipe_attr *attr);
    int (*fnDestroyPipe)(int pipe);
    int (*fnStartPipe)(int pipe);
    int (*fnStopPipe)(int pipe);
    int (*fnSetPipeAttr)(int pipe, const v5_vi_pipe_attr *attr);
    int (*fnGetPipeAttr)(int pipe, v5_vi_pipe_attr *attr);

    /* Channel. */
    int (*fnSetChnAttr)(int pipe, int chn, const v5_vi_chn_attr *attr);
    int (*fnGetChnAttr)(int pipe, int chn, v5_vi_chn_attr *attr);
    int (*fnEnableChn)(int pipe, int chn);
    int (*fnDisableChn)(int pipe, int chn);
    int (*fnSetChnCrop)(int pipe, int chn, const void *crop);

    /*
     * Frame access straight off VI. Optional: the pipeline binds VI to VPSS
     * and reads there, so nothing in Phase 2 calls these. They are bound
     * because a raw-capture path (Phase 7's memory work wants one) reaches
     * for exactly these two and the alternative is resolving them somewhere
     * that is not this file.
     */
    int (*fnGetChnFrame)(int pipe, int chn, v5_video_frame_info *frame, int milli_sec);
    int (*fnReleaseChnFrame)(int pipe, int chn, const v5_video_frame_info *frame);
    int (*fnGetChnFd)(int pipe, int chn);

    /*
     * 3DNR on the VI side. Which of these and v5_vpss.h's pair is live
     * depends on ss_mpi_sys_set_3dnr_pos (v5_sys.h), so Phase 3 resolves
     * both and calls one. Optional here for the same reason.
     */
    int (*fnSetPipe3dnrAttr)(int pipe, const v5_3dnr_attr *attr);
    int (*fnGetPipe3dnrAttr)(int pipe, v5_3dnr_attr *attr);
    int (*fnSetPipe3dnrParam)(int pipe, const void *param);
    int (*fnGetPipe3dnrParam)(int pipe, void *param);
} v5_vi_impl;

/*
 * v5_vi_load -- bind the VI entry points.
 *
 * Required is "the pipeline cannot come up without it": the device, the
 * bind, the pipe and the channel. Everything else costs an op rather than
 * an init.
 */
static inline int v5_vi_load(v5_vi_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_vi";

    memset(lib, 0, sizeof(*lib));

#define V5_VI_REQ(field, type, name)                                                               \
    do {                                                                                           \
        if (!(lib->field = (type)v5_symbol(mod, libs, name)))                                      \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V5_VI_REQ(fnSetDevAttr, int (*)(int, const v5_vi_dev_attr *), "ss_mpi_vi_set_dev_attr");
    V5_VI_REQ(fnEnableDev, int (*)(int), "ss_mpi_vi_enable_dev");
    V5_VI_REQ(fnDisableDev, int (*)(int), "ss_mpi_vi_disable_dev");
    V5_VI_REQ(fnBind, int (*)(int, int), "ss_mpi_vi_bind");
    V5_VI_REQ(fnUnbind, int (*)(int, int), "ss_mpi_vi_unbind");
    V5_VI_REQ(fnCreatePipe, int (*)(int, const v5_vi_pipe_attr *), "ss_mpi_vi_create_pipe");
    V5_VI_REQ(fnDestroyPipe, int (*)(int), "ss_mpi_vi_destroy_pipe");
    V5_VI_REQ(fnStartPipe, int (*)(int), "ss_mpi_vi_start_pipe");
    V5_VI_REQ(fnStopPipe, int (*)(int), "ss_mpi_vi_stop_pipe");
    V5_VI_REQ(fnSetChnAttr, int (*)(int, int, const v5_vi_chn_attr *), "ss_mpi_vi_set_chn_attr");
    V5_VI_REQ(fnEnableChn, int (*)(int, int), "ss_mpi_vi_enable_chn");
    V5_VI_REQ(fnDisableChn, int (*)(int, int), "ss_mpi_vi_disable_chn");

#undef V5_VI_REQ

    lib->fnSetPipeAttr =
        (int (*)(int, const v5_vi_pipe_attr *))v5_symbol_opt(libs, "ss_mpi_vi_set_pipe_attr");
    lib->fnGetPipeAttr =
        (int (*)(int, v5_vi_pipe_attr *))v5_symbol_opt(libs, "ss_mpi_vi_get_pipe_attr");
    lib->fnGetChnAttr =
        (int (*)(int, int, v5_vi_chn_attr *))v5_symbol_opt(libs, "ss_mpi_vi_get_chn_attr");
    lib->fnSetChnCrop =
        (int (*)(int, int, const void *))v5_symbol_opt(libs, "ss_mpi_vi_set_chn_crop");

    lib->fnGetChnFrame = (int (*)(int, int, v5_video_frame_info *, int))v5_symbol_opt(
        libs, "ss_mpi_vi_get_chn_frame");
    lib->fnReleaseChnFrame = (int (*)(int, int, const v5_video_frame_info *))v5_symbol_opt(
        libs, "ss_mpi_vi_release_chn_frame");
    lib->fnGetChnFd = (int (*)(int, int))v5_symbol_opt(libs, "ss_mpi_vi_get_chn_fd");

    lib->fnSetPipe3dnrAttr =
        (int (*)(int, const v5_3dnr_attr *))v5_symbol_opt(libs, "ss_mpi_vi_set_pipe_3dnr_attr");
    lib->fnGetPipe3dnrAttr =
        (int (*)(int, v5_3dnr_attr *))v5_symbol_opt(libs, "ss_mpi_vi_get_pipe_3dnr_attr");
    lib->fnSetPipe3dnrParam =
        (int (*)(int, const void *))v5_symbol_opt(libs, "ss_mpi_vi_set_pipe_3dnr_param");
    lib->fnGetPipe3dnrParam =
        (int (*)(int, void *))v5_symbol_opt(libs, "ss_mpi_vi_get_pipe_3dnr_param");

    return RSS_OK;
}

static inline void v5_vi_unload(v5_vi_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_VI_H */
