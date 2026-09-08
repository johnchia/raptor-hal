/*
 * hisi_v5/v5_vpss.h -- ss_mpi_vpss bindings, HiMPP V5.0
 *
 * VPSS is the scaler: one *group* takes the frame VI produced and fans it
 * out to several *channels*, each with its own size, format and frame rate.
 * It is where raptor's three streams diverge, so it is the module the
 * framesource layer maps onto almost one-for-one.
 *
 * WHAT CHANGED FROM gen4:
 *
 *   - Three physical channels, not gen4's four (OT_VPSS_MAX_PHYS_CHN_NUM,
 *     ot_defines.h:374). HISI_VPSS_CHN_NUM is already 3 for that reason.
 *   - Six groups, not gen4's counted-by-part number (ot_defines.h:369).
 *   - ot_vpss_grp_attr carries max_dei_width/height and an mcf_en that gen4
 *     had nowhere, and its frame rate control sits at the *end* rather than
 *     beside the sizes.
 *   - The channel attribute absorbed the border and aspect-ratio structs
 *     that gen4 set through separate calls. Both are inline, both are large,
 *     and together they are half of the 96 bytes.
 *
 * THE OPEN QUESTION, plan risk R4. Which physical channel is "channel 0"
 * for a part in this family is not answerable from a header: on several
 * gen4 parts channel 0 was reserved and the usable ones started at 1, which
 * is why gen4 has HISI_VPSS_CHN_BASE at all. This file deliberately does
 * *not* define a base. Bench item 5 -- enable 0/1/2 at different sizes and
 * watch SendOk per channel in /proc/umap/vpss -- decides it, and only then
 * does hisi_state.h get HISI_VPSS_CHN_BASE.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_vpss.h at
 * 1.0.2.0 B051; sizes from a probe compiled against it with the cv6xx
 * cross-compiler:
 *
 *   ot_vpss_grp_attr    56   max_width at +16, dynamic_range at +32,
 *                            frame_rate at +48
 *   ot_vpss_chn_attr    96   width at +12, chn_mode at +24,
 *                            frame_rate at +44, border at +52,
 *                            aspect_ratio at +72
 *   ot_vpss_crop_info   24
 *   ot_vpss_grp_param    8
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_VPSS_H
#define HISI_V5_VPSS_H

#include "v5_common.h"
#include "v5_video.h"

/* ot_defines.h:369, :374. Array bounds and range checks, so ABI: how many
 * raptor uses is hisi_state.h's decision. */
#define V5_VPSS_MAX_GRP_NUM 6
#define V5_VPSS_MAX_PHYS_CHN_NUM 3

/* ot_common_vpss.h:29-34. OT_VPSS_DIRECT_CHN is the low-delay path that
 * bypasses the write-back buffer; named because a stray 31 in a channel
 * argument is otherwise unreadable, not because Phase 2 uses it. */
#define V5_VPSS_CHN_DIRECT 31
#define V5_VPSS_CHN_INVALID (-1)

/*
 * ot_vpss_dei_mode (ot_common_vpss.h:37-42).
 *
 * De-interlace, for an analogue input this part does not have. Present
 * because it is a field in grp_attr and a hole there moves everything after
 * it; OFF is the only value this backend ever writes.
 */
typedef enum {
    V5_VPSS_DEI_MODE_OFF = 0,
    V5_VPSS_DEI_MODE_ON = 1,
    V5_VPSS_DEI_MODE_AUTO = 2,
} v5_vpss_dei_mode;

/*
 * ot_vpss_chn_mode (ot_common_vpss.h:64-68).
 *
 * AUTO takes the channel's geometry from the group; USER takes it from the
 * channel attribute. raptor always wants USER -- three streams at three
 * sizes is the whole point of the module -- and writing AUTO by accident
 * gives three channels that all come out at the sensor's size with no error.
 */
typedef enum {
    V5_VPSS_CHN_MODE_AUTO = 0,
    V5_VPSS_CHN_MODE_USER = 1,
} v5_vpss_chn_mode;

/*
 * ot_vpss_grp_attr (ot_common_vpss.h:44-62).
 *
 * max_width/max_height are the *allocation* size -- they size the group's
 * internal buffers -- so they must be at least the sensor's output and are
 * not a crop. The four td_bool at the front are enums, four bytes each,
 * which is what puts max_width at +16 rather than +4.
 */
typedef struct {
    int ie_en;
    int dci_en;
    int buf_share_en;
    int mcf_en;
    unsigned int max_width;
    unsigned int max_height;
    unsigned int max_dei_width;
    unsigned int max_dei_height;
    v5_dynamic_range dynamic_range;
    v5_pixel_format pixel_format;
    v5_vpss_dei_mode dei_mode;
    int buf_share_chn;
    v5_frame_rate_ctrl frame_rate;
} v5_vpss_grp_attr;

_Static_assert(sizeof(v5_vpss_grp_attr) == 56, "ot_vpss_grp_attr is 56 bytes");
_Static_assert(offsetof(v5_vpss_grp_attr, max_width) == 16, "ot_vpss_grp_attr.max_width at +16");
_Static_assert(offsetof(v5_vpss_grp_attr, dynamic_range) == 32,
               "ot_vpss_grp_attr.dynamic_range at +32");
_Static_assert(offsetof(v5_vpss_grp_attr, frame_rate) == 48, "ot_vpss_grp_attr.frame_rate at +48");

/*
 * ot_vpss_chn_attr (ot_common_vpss.h:70-87).
 *
 * mirror_en/flip_en here are the group-side pair of the ones in
 * v5_vi_chn_attr. Both work; VPSS's is per-stream, which is what raptor
 * wants, and VI's would flip all three at once.
 *
 * depth is the number of frames the channel holds for a userspace reader.
 * Zero means "bound consumer only" -- get_chn_frame then returns
 * OT_ERR_VPSS_BUF_EMPTY forever -- and every nonzero value costs that many
 * frames of VB. The encoder path is bound, so this stays 0 except on a
 * channel something reads by hand.
 */
typedef struct {
    int mirror_en;
    int flip_en;
    int border_en;
    unsigned int width;
    unsigned int height;
    unsigned int depth;
    v5_vpss_chn_mode chn_mode;
    v5_video_format video_format;
    v5_dynamic_range dynamic_range;
    v5_pixel_format pixel_format;
    v5_compress_mode compress_mode;
    v5_frame_rate_ctrl frame_rate;
    v5_border border_attr;
    v5_aspect_ratio aspect_ratio;
} v5_vpss_chn_attr;

/*
 * ot_vpss_chn_buf_wrap_attr (ot_common_vpss.h:138-142). Physical channel
 * 0 only, set between set_chn_attr and enable_chn, and while it is on the
 * channel writes a ring of buf_line lines into one block it takes from
 * the common pools instead of whole frames into its own. buf_size is the
 * caller's arithmetic (ot_comm_get_vpss_venc_wrap_buf_size, ported as
 * hisi_vb_wrap_size) and the block it takes has to be at least that.
 */
typedef struct {
    int enable; /* td_bool */
    unsigned int buf_line;
    unsigned int buf_size;
} v5_vpss_chn_buf_wrap_attr;

_Static_assert(sizeof(v5_vpss_chn_buf_wrap_attr) == 12, "ot_vpss_chn_buf_wrap_attr is 12 bytes");

_Static_assert(sizeof(v5_vpss_chn_attr) == 96, "ot_vpss_chn_attr is 96 bytes");
_Static_assert(offsetof(v5_vpss_chn_attr, width) == 12, "ot_vpss_chn_attr.width at +12");
_Static_assert(offsetof(v5_vpss_chn_attr, chn_mode) == 24, "ot_vpss_chn_attr.chn_mode at +24");
_Static_assert(offsetof(v5_vpss_chn_attr, frame_rate) == 44, "ot_vpss_chn_attr.frame_rate at +44");
_Static_assert(offsetof(v5_vpss_chn_attr, border_attr) == 52,
               "ot_vpss_chn_attr.border_attr at +52");
_Static_assert(offsetof(v5_vpss_chn_attr, aspect_ratio) == 72,
               "ot_vpss_chn_attr.aspect_ratio at +72");

/*
 * ot_vpss_crop_info (ot_common_vpss.h:101-106).
 *
 * The same struct for the group crop and the channel crop; the group's is
 * the digital zoom, the channel's is the per-stream one. crop_mode picks
 * whether crop_rect is in pixels (ABS) or in 1/1000ths (RATIO) -- see
 * v5_coord in v5_video.h.
 */
typedef struct {
    int enable;
    v5_coord crop_mode;
    v5_rect crop_rect;
} v5_vpss_crop_info;

_Static_assert(sizeof(v5_vpss_crop_info) == 24, "ot_vpss_crop_info is 24 bytes");
_Static_assert(offsetof(v5_vpss_crop_info, crop_rect) == 8, "ot_vpss_crop_info.crop_rect at +8");

/*
 * ot_vpss_grp_param (ot_common_vpss.h:108-111).
 *
 * The strengths behind grp_attr's ie_en and dci_en. Both are [0, 63] and
 * both are ignored while their enable is off, so this is a Phase 3 knob
 * rather than a bring-up one.
 */
typedef struct {
    unsigned int contrast;
    unsigned int ie_strength;
} v5_vpss_grp_param;

_Static_assert(sizeof(v5_vpss_grp_param) == 8, "ot_vpss_grp_param is 8 bytes");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    /* Group. */
    int (*fnCreateGrp)(int grp, const v5_vpss_grp_attr *attr);
    int (*fnDestroyGrp)(int grp);
    int (*fnStartGrp)(int grp);
    int (*fnStopGrp)(int grp);
    int (*fnSetGrpAttr)(int grp, const v5_vpss_grp_attr *attr);
    int (*fnGetGrpAttr)(int grp, v5_vpss_grp_attr *attr);

    /* Channel. */
    int (*fnSetChnAttr)(int grp, int chn, const v5_vpss_chn_attr *attr);
    int (*fnGetChnAttr)(int grp, int chn, v5_vpss_chn_attr *attr);
    int (*fnEnableChn)(int grp, int chn);
    int (*fnDisableChn)(int grp, int chn);

    /*
     * Crop. Optional: a pipeline with no digital zoom never calls either,
     * and a board whose VPSS refuses the group crop still streams.
     */
    int (*fnSetGrpCrop)(int grp, const v5_vpss_crop_info *crop);
    int (*fnGetGrpCrop)(int grp, v5_vpss_crop_info *crop);
    int (*fnSetChnCrop)(int grp, int chn, const v5_vpss_crop_info *crop);
    int (*fnGetChnCrop)(int grp, int chn, v5_vpss_crop_info *crop);

    /* IE/DCI strengths; see v5_vpss_grp_param. */
    int (*fnSetGrpParam)(int grp, const v5_vpss_grp_param *param);
    int (*fnGetGrpParam)(int grp, v5_vpss_grp_param *param);

    /*
     * Frame access. The encoder is *bound* to the channel, so nothing in
     * the streaming path calls these; the JPEG snapshot path does, and it
     * is the reason depth in the channel attribute is not always 0.
     */
    int (*fnGetChnFrame)(int grp, int chn, v5_video_frame_info *frame, int milli_sec);
    int (*fnReleaseChnFrame)(int grp, int chn, const v5_video_frame_info *frame);

    /*
     * A pool cut to one channel rather than drawn from the common pools.
     * Optional: without the three of them every channel draws common.
     *
     * All three are needed together. set_chn_vb_src selects USER,
     * attach supplies the pool, and a channel switched to USER with no
     * pool attached produces nothing -- so hisi_fs_pool_acquire requires
     * every one of them before it tries.
     */
    int (*fnSetChnVbSrc)(int grp, int chn, v5_vb_src src);
    int (*fnAttachChnVbPool)(int grp, int chn, unsigned int pool);
    int (*fnDetachChnVbPool)(int grp, int chn);

    /*
     * 3DNR on the VPSS side. The pair of v5_vi.h's; ss_mpi_sys_set_3dnr_pos
     * says which is live. Phase 3 resolves both and calls one.
     */
    int (*fnSetGrp3dnrAttr)(int grp, const void *attr);
    int (*fnGetGrp3dnrAttr)(int grp, void *attr);
    int (*fnSetGrp3dnrParam)(int grp, const void *param);
    int (*fnGetGrp3dnrParam)(int grp, void *param);

    /* Rotation, for a mounted-upside-down camera. Phase 3's. */
    int (*fnSetChnRotation)(int grp, int chn, const void *rotation);
    int (*fnGetChnRotation)(int grp, int chn, void *rotation);

    /* The chn0 -> VENC ring. Optional: a driver without it streams from
     * whole frames, which is correct and costs the private pool. */
    int (*fnSetChnBufWrap)(int grp, int chn, const v5_vpss_chn_buf_wrap_attr *attr);
    int (*fnGetChnBufWrap)(int grp, int chn, v5_vpss_chn_buf_wrap_attr *attr);
} v5_vpss_impl;

/*
 * v5_vpss_load -- bind the VPSS entry points.
 *
 * Required is the group lifecycle and the channel enable, which is exactly
 * what a stream needs. Note create_grp and set_grp_attr are both required
 * even though create takes the attribute: raptor changes resolution on a
 * running group, and doing that by destroying and recreating it drops the
 * VI binding with it.
 */
static inline int v5_vpss_load(v5_vpss_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_vpss";

    memset(lib, 0, sizeof(*lib));

#define V5_VPSS_REQ(field, type, name)                                                             \
    do {                                                                                           \
        if (!(lib->field = (type)v5_symbol(mod, libs, name)))                                      \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V5_VPSS_REQ(fnCreateGrp, int (*)(int, const v5_vpss_grp_attr *), "ss_mpi_vpss_create_grp");
    V5_VPSS_REQ(fnDestroyGrp, int (*)(int), "ss_mpi_vpss_destroy_grp");
    V5_VPSS_REQ(fnStartGrp, int (*)(int), "ss_mpi_vpss_start_grp");
    V5_VPSS_REQ(fnStopGrp, int (*)(int), "ss_mpi_vpss_stop_grp");
    V5_VPSS_REQ(fnSetGrpAttr, int (*)(int, const v5_vpss_grp_attr *), "ss_mpi_vpss_set_grp_attr");
    V5_VPSS_REQ(fnSetChnAttr, int (*)(int, int, const v5_vpss_chn_attr *),
                "ss_mpi_vpss_set_chn_attr");
    V5_VPSS_REQ(fnEnableChn, int (*)(int, int), "ss_mpi_vpss_enable_chn");
    V5_VPSS_REQ(fnDisableChn, int (*)(int, int), "ss_mpi_vpss_disable_chn");

#undef V5_VPSS_REQ

    lib->fnGetGrpAttr =
        (int (*)(int, v5_vpss_grp_attr *))v5_symbol_opt(libs, "ss_mpi_vpss_get_grp_attr");
    lib->fnGetChnAttr =
        (int (*)(int, int, v5_vpss_chn_attr *))v5_symbol_opt(libs, "ss_mpi_vpss_get_chn_attr");

    lib->fnSetGrpCrop =
        (int (*)(int, const v5_vpss_crop_info *))v5_symbol_opt(libs, "ss_mpi_vpss_set_grp_crop");
    lib->fnGetGrpCrop =
        (int (*)(int, v5_vpss_crop_info *))v5_symbol_opt(libs, "ss_mpi_vpss_get_grp_crop");
    lib->fnSetChnCrop = (int (*)(int, int, const v5_vpss_crop_info *))v5_symbol_opt(
        libs, "ss_mpi_vpss_set_chn_crop");
    lib->fnGetChnCrop =
        (int (*)(int, int, v5_vpss_crop_info *))v5_symbol_opt(libs, "ss_mpi_vpss_get_chn_crop");

    lib->fnSetGrpParam =
        (int (*)(int, const v5_vpss_grp_param *))v5_symbol_opt(libs, "ss_mpi_vpss_set_grp_param");
    lib->fnGetGrpParam =
        (int (*)(int, v5_vpss_grp_param *))v5_symbol_opt(libs, "ss_mpi_vpss_get_grp_param");

    lib->fnGetChnFrame = (int (*)(int, int, v5_video_frame_info *, int))v5_symbol_opt(
        libs, "ss_mpi_vpss_get_chn_frame");
    lib->fnReleaseChnFrame = (int (*)(int, int, const v5_video_frame_info *))v5_symbol_opt(
        libs, "ss_mpi_vpss_release_chn_frame");

    lib->fnSetChnVbSrc =
        (int (*)(int, int, v5_vb_src))v5_symbol_opt(libs, "ss_mpi_vpss_set_chn_vb_src");
    lib->fnAttachChnVbPool =
        (int (*)(int, int, unsigned int))v5_symbol_opt(libs, "ss_mpi_vpss_attach_chn_vb_pool");
    lib->fnDetachChnVbPool =
        (int (*)(int, int))v5_symbol_opt(libs, "ss_mpi_vpss_detach_chn_vb_pool");

    lib->fnSetGrp3dnrAttr =
        (int (*)(int, const void *))v5_symbol_opt(libs, "ss_mpi_vpss_set_grp_3dnr_attr");
    lib->fnGetGrp3dnrAttr =
        (int (*)(int, void *))v5_symbol_opt(libs, "ss_mpi_vpss_get_grp_3dnr_attr");
    lib->fnSetGrp3dnrParam =
        (int (*)(int, const void *))v5_symbol_opt(libs, "ss_mpi_vpss_set_grp_3dnr_param");
    lib->fnGetGrp3dnrParam =
        (int (*)(int, void *))v5_symbol_opt(libs, "ss_mpi_vpss_get_grp_3dnr_param");

    lib->fnSetChnRotation =
        (int (*)(int, int, const void *))v5_symbol_opt(libs, "ss_mpi_vpss_set_chn_rotation");
    lib->fnGetChnRotation =
        (int (*)(int, int, void *))v5_symbol_opt(libs, "ss_mpi_vpss_get_chn_rotation");
    lib->fnSetChnBufWrap = (int (*)(int, int, const v5_vpss_chn_buf_wrap_attr *))v5_symbol_opt(
        libs, "ss_mpi_vpss_set_chn_buf_wrap");
    lib->fnGetChnBufWrap = (int (*)(int, int, v5_vpss_chn_buf_wrap_attr *))v5_symbol_opt(
        libs, "ss_mpi_vpss_get_chn_buf_wrap");

    return RSS_OK;
}

static inline void v5_vpss_unload(v5_vpss_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_VPSS_H */
