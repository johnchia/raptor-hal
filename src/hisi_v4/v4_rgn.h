/*
 * hisi_v4/v4_rgn.h -- HI_MPI_RGN bindings, HiMPP V4.0
 *
 * RGN is HiMPP's overlay compositor. A region is a global object with a
 * handle of its own; it carries a bitmap, and it is *attached* to a
 * (module, device, channel) triple, which is where it gets its position,
 * its alpha and its z-order. Nothing about a region lives in the datapath:
 * there is no OSD stage to bind, which is why hal_bind collapses rvd's
 * FS -> OSD -> ENC chain to one FS -> VENC bind and why this file exists
 * beside it rather than inside it.
 *
 * WHICH REGION TYPE, AND WHERE IT ATTACHES
 *
 * Only OVERLAY_RGN is transcribed, and only VENC is attached to. Both are
 * decided by the silicon rather than chosen: hi_defines.h publishes a
 * per-module region budget and for gen4 it reads
 *
 *   OVERLAY_MAX_NUM_VENC   8      <- the only non-zero VENC entry
 *   OVERLAY_MAX_NUM_VPSS   0
 *   OVERLAY_MAX_NUM_VI     0
 *   COVER_MAX_NUM_VI       0      (COVER has no VENC entry at all)
 *
 * so an overlay can only be composited by the encoder, and a COVER region
 * cannot be composited by the encoder at any count. Attaching to VENC is
 * also what makes the overlay per-stream, which is the whole point: the
 * timestamp on the main stream and the one on the sub-stream are different
 * regions at different positions, and a VPSS-side overlay could not be.
 *
 * ARGB1555 is the pixel format. The SDK header says so at the field --
 * "now only support ARGB1555 or ARGB4444" -- and the vendor's own sample
 * uses ARGB1555 (sample_comm_region.c, SAMPLE_REGION_CreateOverLay). There
 * is no probe here of the kind src/star/hal_osd.c runs, because unlike MI
 * the accepted set is written down and there are only two members of it.
 *
 * ALPHA IS 0..128, NOT 0..255
 *
 * u32FgAlpha and u32BgAlpha are documented "range:[0,128]" for
 * OVERLAY_CHN_ATTR_S -- and 0..255 for OVERLAYEX_CHN_ATTR_S three structs
 * later, which is exactly the sort of neighbouring difference a
 * transcription loses. raptor's rss_osd_region_t carries 0..255, so
 * hal_osd.c scales; see v4_rgn_alpha().
 *
 * PROVENANCE. Layouts and field order were derived from the Hi3516EV200
 * SDK V1.0.1.0 MPP headers (mpp/include/hi_comm_region.h, hi_comm_video.h
 * for BITMAP_S/POINT_S, hi_defines.h for the budgets) and cross-checked
 * field by field against ref/openhisilicon/include/comm_region.h (GPL v3,
 * OVERLAY_ATTR_S at :60, OVERLAY_CHN_ATTR_S at :69, RGN_ATTR_S at :173,
 * RGN_CHN_ATTR_S at :178), which agrees member for member and enumerator
 * for enumerator. The citations name openhisilicon where they can: it is
 * the licence-compatible one and a reader can open it.
 *
 * Sizes and offsets were read out of a probe compiled against the SDK
 * headers with arm-openipc-linux-musleabi-gcc, not counted by hand:
 *
 *   RGN_ATTR_S            24  enType +0, unAttr +4
 *   OVERLAY_ATTR_S        20  enPixelFmt +0, u32BgColor +4, stSize +8,
 *                             u32CanvasNum +16
 *   RGN_CHN_ATTR_S        68  bShow +0, enType +4, unChnAttr +8
 *   OVERLAY_CHN_ATTR_S    60  stPoint +0, u32FgAlpha +8, u32BgAlpha +12,
 *                             u32Layer +16, stQpInfo +20, stInvertColor +32,
 *                             enAttachDest +52, u16ColorLUT +56
 *   OVERLAY_QP_INFO_S     12
 *   OVERLAY_INVERT_COLOR_S 20 stInvColArea +0, u32LumThresh +8,
 *                             enChgMod +12, bInvColEn +16
 *   BITMAP_S              16  enPixelFormat +0, u32Width +4, u32Height +8,
 *                             pData +12
 *   POINT_S                8
 *
 * The one worth stating is RGN_CHN_ATTR_S's 68: the union's widest arm is
 * OVERLAY_CHN_ATTR_S at 60, not COVER_CHN_ATTR_S at 56, so a reader who
 * assumed the quadrangle arm was the big one would size the struct short
 * by four bytes and hand the driver a truncated colour LUT.
 *
 * divinus's v4_rgn.h was read last, as the plan asks. It agrees on every
 * layout here; where it goes wrong is one level up, in the call -- see the
 * .channel note in hal_osd.c.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_RGN_H
#define HISI_V4_RGN_H

#include "v4_common.h"
#include "v4_video.h"

/*
 * The budgets, from hi_defines.h. RGN_HANDLE_MAX is the global handle
 * space; OVERLAY_MAX_NUM_VENC is how many overlays one *encoder channel*
 * will composite, and it is the limit a caller actually runs into.
 */
#define V4_RGN_HANDLE_MAX 128
#define V4_RGN_OVERLAY_MAX_PER_VENC 8

/* RGN_MIN_WIDTH / RGN_MIN_HEIGHT, and RGN_ALIGN. A region's dimensions are
 * even; the driver rejects an odd one rather than rounding it. */
#define V4_RGN_MIN_DIM 2
#define V4_RGN_ALIGN 2

/* RGN_OVERLAY_MAX_X / _MAX_Y / _MAX_WIDTH / _MAX_HEIGHT. */
#define V4_RGN_OVERLAY_MAX_XY 4094
#define V4_RGN_OVERLAY_MAX_DIM 4096

/* ================================================================
 * GEOMETRY AND BITMAPS
 * ================================================================ */

/*
 * POINT_S (openhisilicon comm_video.h). Signed, unlike SIZE_S's members,
 * and it lives here rather than in v4_video.h because RGN is the only
 * module in this backend that takes one.
 */
typedef struct {
    int x;
    int y;
} v4_point;

_Static_assert(sizeof(v4_point) == 8, "POINT_S is 8 bytes");
_Static_assert(offsetof(v4_point, y) == 4, "POINT_S.s32Y at +4");

/*
 * PIXEL_FORMAT_E, the two members RGN accepts for an OVERLAY region.
 *
 * Deliberately a separate enum from v4_pixel_format in v4_video.h rather
 * than three more members added to it: that one names what the *pipeline*
 * produces (Bayer and YVU420), this one names what the compositor takes,
 * and the two sets do not overlap. The values are the same enumeration --
 * ARGB_1555 is 8, ARGB_4444 is 9, ARGB_8888 is 11, read from the probe --
 * so they are interchangeable at the ABI; keeping them apart is what stops
 * a caller passing YVU420 to a bitmap.
 */
typedef enum {
    V4_RGN_PIXFMT_ARGB1555 = 8,
    V4_RGN_PIXFMT_ARGB4444 = 9,
    V4_RGN_PIXFMT_ARGB8888 = 11,
} v4_rgn_pixfmt;

/*
 * BITMAP_S (openhisilicon comm_video.h). Carries a pointer, so its size
 * and the offset of pData are 32-bit facts and asserted only there; the
 * three offsets before the pointer are portable and asserted everywhere.
 *
 * There is no stride member. The driver reads width * bpp per row, so the
 * buffer handed over must be tightly packed -- which is what hal_osd.c's
 * conversion produces anyway, since it writes the bitmap itself.
 */
typedef struct {
    v4_rgn_pixfmt pixel_format;
    unsigned int width;
    unsigned int height;
    void *data;
} v4_bitmap;

_Static_assert(offsetof(v4_bitmap, width) == 4, "BITMAP_S.u32Width at +4");
_Static_assert(offsetof(v4_bitmap, height) == 8, "BITMAP_S.u32Height at +8");
#if V4_ABI32
_Static_assert(sizeof(v4_bitmap) == 16, "BITMAP_S is 16 bytes on ARM32");
_Static_assert(offsetof(v4_bitmap, data) == 12, "BITMAP_S.pData at +12");
#endif

/* ================================================================
 * REGION ATTRIBUTES -- what a region *is*
 * ================================================================ */

/* RGN_TYPE_E (openhisilicon comm_region.h:25). All five are named because
 * enType is written into two different structs and a reader needs to see
 * that OVERLAY is 0 rather than take it on trust; only OVERLAY is used. */
typedef enum {
    V4_RGN_TYPE_OVERLAY = 0,
    V4_RGN_TYPE_COVER = 1,
    V4_RGN_TYPE_COVEREX = 2,
    V4_RGN_TYPE_OVERLAYEX = 3,
    V4_RGN_TYPE_MOSAIC = 4,
} v4_rgn_type;

/* OVERLAY_ATTR_S (openhisilicon comm_region.h:60). */
typedef struct {
    v4_rgn_pixfmt pixel_format;
    /* Background colour in the region's own pixel format -- i.e. an
     * ARGB1555 word, not a 32-bit one. Zero is fully transparent black,
     * which is what an overlay whose bitmap carries its own alpha wants. */
    unsigned int bg_color;
    v4_size size;
    /*
     * How many canvases the driver allocates for this region.
     *
     * The canvases are the ping-pong buffers behind HI_MPI_RGN_SetBitMap:
     * with one, a bitmap update races the compositor reading the same
     * memory. Two is the smallest number that does not, and each costs
     * width * height * 2 bytes of the RGN heap. The vendor sample asks for
     * five (sample_comm_region.c:276) and divinus for handle + 1, neither
     * with a reason; two is chosen here because rvd updates a region's
     * bitmap on a timer and the extra three buffers would be idle.
     */
    unsigned int canvas_num;
} v4_rgn_overlay_attr;

_Static_assert(sizeof(v4_rgn_overlay_attr) == 20, "OVERLAY_ATTR_S is 20 bytes");
_Static_assert(offsetof(v4_rgn_overlay_attr, bg_color) == 4, "OVERLAY_ATTR_S.u32BgColor at +4");
_Static_assert(offsetof(v4_rgn_overlay_attr, size) == 8, "OVERLAY_ATTR_S.stSize at +8");
_Static_assert(offsetof(v4_rgn_overlay_attr, canvas_num) == 16,
               "OVERLAY_ATTR_S.u32CanvasNum at +16");

/*
 * RGN_ATTR_S (openhisilicon comm_region.h:173).
 *
 * The vendor's second member is a union of OVERLAY_ATTR_S and
 * OVERLAYEX_ATTR_S. They are the same 20 bytes with the same members, so
 * the union is written out here as its only arm rather than as a union
 * with a duplicate -- the size assert is what makes that safe, and it is
 * the same 24 bytes either way.
 */
typedef struct {
    v4_rgn_type type;
    v4_rgn_overlay_attr overlay;
} v4_rgn_attr;

_Static_assert(sizeof(v4_rgn_attr) == 24, "RGN_ATTR_S is 24 bytes");
_Static_assert(offsetof(v4_rgn_attr, overlay) == 4, "RGN_ATTR_S.unAttr at +4");

/* ================================================================
 * CHANNEL ATTRIBUTES -- where a region *appears*
 * ================================================================ */

/*
 * OVERLAY_QP_INFO_S (openhisilicon comm_region.h:40).
 *
 * Lets an overlay force the encoder's QP over the rectangle it covers, so
 * text stays legible in a low-bitrate stream. raptor has no configuration
 * for it and hal_osd.c leaves the whole struct zeroed, which is
 * bQpDisable = false with a relative delta of 0: no change. Transcribed
 * rather than padded because the two booleans around s32Qp are what give
 * the struct its 12 bytes, and a reader checking the +20 offset of
 * stInvertColor needs to see why.
 */
typedef struct {
    int abs_qp;
    int qp;
    int qp_disable;
} v4_rgn_qp_info;

_Static_assert(sizeof(v4_rgn_qp_info) == 12, "OVERLAY_QP_INFO_S is 12 bytes");
_Static_assert(offsetof(v4_rgn_qp_info, qp) == 4, "OVERLAY_QP_INFO_S.s32Qp at +4");
_Static_assert(offsetof(v4_rgn_qp_info, qp_disable) == 8, "OVERLAY_QP_INFO_S.bQpDisable at +8");

/* INVERT_COLOR_MODE_E (openhisilicon comm_region.h:34). */
typedef enum {
    V4_RGN_INVERT_LESS_THAN_LUM = 0,
    V4_RGN_INVERT_MORE_THAN_LUM = 1,
} v4_rgn_invert_mode;

/*
 * OVERLAY_INVERT_COLOR_S (openhisilicon comm_region.h:46).
 *
 * Inverts the overlay's colour where the video behind it is bright, so
 * white text stays visible on a white wall. Left disabled: rss_osd_region_t
 * has no field to drive it, and hal_caps.c publishes has_osd_region_invert
 * false for the same reason -- a capability raptor cannot ask for is not a
 * capability. The SDK constrains stInvColArea to a multiple of 16 no
 * greater than 64, which is recorded here rather than enforced because
 * nothing writes it.
 */
typedef struct {
    v4_size inv_col_area;
    unsigned int lum_thresh;
    v4_rgn_invert_mode chg_mode;
    int inv_col_en;
} v4_rgn_invert_color;

_Static_assert(sizeof(v4_rgn_invert_color) == 20, "OVERLAY_INVERT_COLOR_S is 20 bytes");
_Static_assert(offsetof(v4_rgn_invert_color, lum_thresh) == 8, "u32LumThresh at +8");
_Static_assert(offsetof(v4_rgn_invert_color, chg_mode) == 12, "enChgMod at +12");
_Static_assert(offsetof(v4_rgn_invert_color, inv_col_en) == 16, "bInvColEn at +16");

/* ATTACH_DEST_E (openhisilicon comm_region.h:53). Which JPEG image an
 * overlay lands on when the attached channel is a JPEG encoder: the main
 * picture or one of the two multi-picture thumbnails. MAIN is 0, which is
 * also what a zeroed struct asks for. */
typedef enum {
    V4_RGN_ATTACH_JPEG_MAIN = 0,
    V4_RGN_ATTACH_JPEG_MPF0 = 1,
    V4_RGN_ATTACH_JPEG_MPF1 = 2,
} v4_rgn_attach_dest;

#define V4_RGN_COLOR_LUT_NUM 2

/*
 * OVERLAY_CHN_ATTR_S (openhisilicon comm_region.h:69).
 *
 * NOTE THE ALPHA RANGE: u32FgAlpha and u32BgAlpha are [0,128] here and
 * [0,255] in OVERLAYEX_CHN_ATTR_S. See the file comment.
 *
 * fg/bg is not a blend of two layers: with ARGB1555 the bitmap's single
 * alpha bit *selects* between them, so fg_alpha is the opacity of a pixel
 * whose alpha bit is 1 and bg_alpha the opacity of one whose alpha bit is
 * 0. Text rendered opaque on a transparent field therefore wants
 * fg = full, bg = 0, which is what rvd asks for.
 */
typedef struct {
    v4_point point;
    unsigned int fg_alpha;
    unsigned int bg_alpha;
    /* [0,7] for OVERLAY, unlike OVERLAYEX's [0,15]. */
    unsigned int layer;
    v4_rgn_qp_info qp_info;
    v4_rgn_invert_color invert_color;
    v4_rgn_attach_dest attach_dest;
    unsigned short color_lut[V4_RGN_COLOR_LUT_NUM];
} v4_rgn_overlay_chn_attr;

_Static_assert(sizeof(v4_rgn_overlay_chn_attr) == 60, "OVERLAY_CHN_ATTR_S is 60 bytes");
_Static_assert(offsetof(v4_rgn_overlay_chn_attr, fg_alpha) == 8, "u32FgAlpha at +8");
_Static_assert(offsetof(v4_rgn_overlay_chn_attr, bg_alpha) == 12, "u32BgAlpha at +12");
_Static_assert(offsetof(v4_rgn_overlay_chn_attr, layer) == 16, "u32Layer at +16");
_Static_assert(offsetof(v4_rgn_overlay_chn_attr, qp_info) == 20, "stQpInfo at +20");
_Static_assert(offsetof(v4_rgn_overlay_chn_attr, invert_color) == 32, "stInvertColor at +32");
_Static_assert(offsetof(v4_rgn_overlay_chn_attr, attach_dest) == 52, "enAttachDest at +52");
_Static_assert(offsetof(v4_rgn_overlay_chn_attr, color_lut) == 56, "u16ColorLUT at +56");

/*
 * RGN_CHN_ATTR_S (openhisilicon comm_region.h:178).
 *
 * The vendor's third member is a five-arm union. Only the OVERLAY arm is
 * written out, and that is safe rather than lucky: OVERLAY_CHN_ATTR_S at
 * 60 bytes is the *widest* arm -- COVER_CHN_ATTR_S is 56, COVEREX 52,
 * MOSAIC 24, OVERLAYEX 24 -- so the struct is the same 68 bytes it would
 * be with the union spelled out, which is what the size assert checks.
 * The other four arms are not transcribed because none of them can attach
 * to a VENC channel; see the file comment's budget table.
 */
typedef struct {
    int show;
    v4_rgn_type type;
    v4_rgn_overlay_chn_attr overlay;
} v4_rgn_chn_attr;

_Static_assert(sizeof(v4_rgn_chn_attr) == 68, "RGN_CHN_ATTR_S is 68 bytes");
_Static_assert(offsetof(v4_rgn_chn_attr, type) == 4, "RGN_CHN_ATTR_S.enType at +4");
_Static_assert(offsetof(v4_rgn_chn_attr, overlay) == 8, "RGN_CHN_ATTR_S.unChnAttr at +8");

/*
 * v4_rgn_alpha -- raptor's 0..255 alpha onto OVERLAY's 0..128.
 *
 * Rounds so that 255 maps to exactly 128 and 0 to exactly 0; divinus's
 * `opacity >> 1` gives 127 for full opacity, which is one step short of
 * opaque on every region it draws. Values above 255 are clamped rather
 * than wrapped -- the field is unsigned and the driver rejects anything
 * over 128 for the whole attach, so a bad alpha would cost the region.
 */
static inline unsigned int v4_rgn_alpha(unsigned int a255)
{
    if (a255 > 255u)
        a255 = 255u;

    return (a255 * 128u + 127u) / 255u;
}

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    int (*fnCreate)(unsigned int handle, const v4_rgn_attr *attr);
    int (*fnDestroy)(unsigned int handle);

    int (*fnGetAttr)(unsigned int handle, v4_rgn_attr *attr);
    int (*fnSetAttr)(unsigned int handle, const v4_rgn_attr *attr);

    int (*fnSetBitMap)(unsigned int handle, const v4_bitmap *bmp);

    int (*fnAttachToChn)(unsigned int handle, const v4_mpp_chn *chn,
                         const v4_rgn_chn_attr *chn_attr);
    int (*fnDetachFromChn)(unsigned int handle, const v4_mpp_chn *chn);

    int (*fnSetDisplayAttr)(unsigned int handle, const v4_mpp_chn *chn,
                            const v4_rgn_chn_attr *chn_attr);
    int (*fnGetDisplayAttr)(unsigned int handle, const v4_mpp_chn *chn,
                            v4_rgn_chn_attr *chn_attr);
} v4_rgn_impl;

/*
 * v4_rgn_load -- bind the RGN entry points.
 *
 * Required versus optional is drawn where it is everywhere else in this
 * backend: at "can the subsystem come up at all". Create, Destroy,
 * SetBitMap, AttachToChn, DetachFromChn and SetDisplayAttr are the six
 * calls an overlay's whole life is made of, so a board missing any of them
 * has no OSD and the caller is told so with RSS_ERR_NOTSUP -- which rvd
 * turns into "overlays disabled" and keeps streaming.
 *
 * The two Get calls are optional and cost only a diagnostic. hal_osd.c
 * tracks every attribute it sets (it has to -- see hisi_osd_region_t), so
 * it never needs to read one back; GetAttr is used for one thing only,
 * telling an existing region from a fresh handle when a create is
 * repeated, and there is a fallback for its absence.
 *
 * Unlike RGN_Init/RGN_DeInit on MI there is nothing to initialise: gen4's
 * RGN has no module lifecycle of its own, so the first Create is the
 * subsystem coming up.
 */
static inline int v4_rgn_load(v4_rgn_impl *lib, const v4_mpi_libs *libs)
{
    static const char mod[] = "v4_rgn";

    memset(lib, 0, sizeof(*lib));

#define V4_RGN_REQ(field, type, name)                                                              \
    do {                                                                                           \
        if (!(lib->field = (type)v4_symbol(mod, libs, "HI_MPI_RGN_" name, "GK_API_RGN_" name)))    \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V4_RGN_REQ(fnCreate, int (*)(unsigned int, const v4_rgn_attr *), "Create");
    V4_RGN_REQ(fnDestroy, int (*)(unsigned int), "Destroy");
    V4_RGN_REQ(fnSetBitMap, int (*)(unsigned int, const v4_bitmap *), "SetBitMap");
    V4_RGN_REQ(fnAttachToChn,
               int (*)(unsigned int, const v4_mpp_chn *, const v4_rgn_chn_attr *), "AttachToChn");
    V4_RGN_REQ(fnDetachFromChn, int (*)(unsigned int, const v4_mpp_chn *), "DetachFromChn");
    V4_RGN_REQ(fnSetDisplayAttr,
               int (*)(unsigned int, const v4_mpp_chn *, const v4_rgn_chn_attr *),
               "SetDisplayAttr");

#undef V4_RGN_REQ

    lib->fnGetAttr =
        (int (*)(unsigned int, v4_rgn_attr *))v4_symbol_opt(libs, "HI_MPI_RGN_GetAttr",
                                                            "GK_API_RGN_GetAttr");
    lib->fnSetAttr = (int (*)(unsigned int, const v4_rgn_attr *))v4_symbol_opt(
        libs, "HI_MPI_RGN_SetAttr", "GK_API_RGN_SetAttr");
    lib->fnGetDisplayAttr = (int (*)(unsigned int, const v4_mpp_chn *, v4_rgn_chn_attr *))
        v4_symbol_opt(libs, "HI_MPI_RGN_GetDisplayAttr", "GK_API_RGN_GetDisplayAttr");

    return RSS_OK;
}

static inline void v4_rgn_unload(v4_rgn_impl *lib)
{
    /* No handle of its own; hisi_mpi_close owns those. Clearing the table
     * turns a use-after-deinit into a NULL check rather than a call into a
     * dlclosed mapping, exactly as v4_sys_unload does. */
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_RGN_H */
