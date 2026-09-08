/*
 * hisi_v5/v5_rgn.h -- ss_mpi_rgn bindings, HiMPP V5.0
 *
 * RGN is HiMPP's overlay compositor, and it works the way gen4's does: a
 * region is a global object with a handle of its own, it carries a bitmap,
 * and it is *attached* to a (module, device, channel) triple, which is
 * where it gets its position, its alpha and its z-order. Nothing about a
 * region lives in the datapath -- there is no OSD stage to bind -- which is
 * why hal_common.c's bind collapses rvd's FS -> OSD -> ENC chain to one
 * FS -> VENC bind and why this file sits beside it.
 *
 * WHICH REGION TYPE, AND WHERE IT ATTACHES
 *
 * Only OVERLAY is transcribed and only VENC is attached to, and as on gen4
 * that is the silicon's decision rather than a preference. ot_defines.h
 * publishes the per-module budgets:
 *
 *   OT_RGN_VENC_MAX_OVERLAY_NUM      8   <- the only VENC entry
 *   OT_RGN_VPSS_MAX_OVERLAYEX_NUM    8       (OVERLAYEX, a different struct)
 *   OT_RGN_VPSS_MAX_COVER_NUM        8
 *   OT_RGN_VI_MAX_COVEREX_NUM       16
 *
 * so the encoder composites OVERLAY and nothing else, and COVER has no VENC
 * entry at all -- the same refusal hal_osd.c gives on gen4 for a privacy
 * cover. Attaching to VENC is also what makes an overlay *per-stream*: the
 * timestamp on the main stream and the one on the sub-stream are two
 * regions on two channels at two positions, which a VPSS-side overlay could
 * not be.
 *
 * WHAT CHANGED FROM gen4
 *
 *   - **Alpha is 0..255**, not gen4's 0..128 (OT_RGN_OVERLAY_MAX_ALPHA and
 *     OT_RGN_OVERLAY_VENC_MAX_ALPHA, ot_defines.h:170-171). raptor's
 *     rss_osd_region_t carries 0..255, so there is nothing to scale and no
 *     v5_rgn_alpha() beside this comment -- v4_rgn_alpha's rounding note is
 *     one generation's problem only.
 *   - The overlay attribute grew a **16-entry colour look-up table**
 *     (`clut`), 64 of its 84 bytes. It is the palette for the CLUT2/CLUT4
 *     pixel formats and is ignored for ARGB; raptor zeroes it.
 *   - The per-channel overlay attribute *lost* gen4's invert-colour block
 *     and its 16-bit colour LUT, and is 36 bytes against gen4's 60.
 *   - **The union's widest arm is no longer the one raptor uses.**
 *     ot_rgn_chn_attr is 64 bytes: 4 + 4 + a 56-byte union whose widest arm
 *     is ot_rgn_cover_chn_attr, while ot_rgn_overlay_chn_attr is 36. So the
 *     transcription pads explicitly. Getting this wrong is not a silent
 *     defect on a set -- it is a short buffer handed to a driver that reads
 *     the whole union.
 *   - OT_ERR_EXIST is 0x8, where gen4's EN_ERR_EXIST is 4. hal_osd.c
 *     matches on it to reclaim a handle a killed daemon left live, so the
 *     number matters; see V5_ERR_EXIST in v5_common.h.
 *
 * PIXEL FORMAT
 *
 * ARGB1555, as on gen4. The field's own comment in ot_common_region.h says
 * "now support clut2 and clut4", which reads like a restriction and is not
 * one: the vendor's own sample (sample_comm_region.c) maps ARGB_1555,
 * ARGB_4444, ARGB_8888 and the six CLUT variants through the same loader,
 * and the CLUT formats exist to save memory on a palette bitmap rather than
 * to replace the direct ones. Board-verified in Phase 5 -- an ARGB1555
 * region composites -- and the fallback if a future part refuses is CLUT4
 * plus a palette, not a different overlay design.
 *
 * THE CANVAS API IS NOT USED
 *
 * ss_mpi_rgn_get_canvas_info + ss_mpi_rgn_update_canvas let a caller draw
 * straight into the region's canvas and avoid a copy. raptor does not: rvd
 * renders BGRA8888 into its own buffer and the conversion to ARGB1555 has
 * to happen somewhere regardless, so the copy is the conversion's output
 * rather than an extra pass. ss_mpi_rgn_set_bmp is also what keeps this
 * file's shape identical to gen4's, which is worth more than one memcpy per
 * overlay update. canvas_num is 2 so the driver double-buffers underneath.
 *
 * PROVENANCE. Layouts are the vendor's ot_common_region.h (SDK V1.0.1.0,
 * which is the only CV610 header set on hand) and the sizes and offsets are
 * a probe compiled against openhisilicon's 1.0.2.0 copy with
 * arm-openipc-linux-musleabi-gcc, not counted by hand:
 *
 *   ot_rgn_attr             88  type +0, attr +4
 *   ot_rgn_overlay_attr     84  pixel_format +0, bg_color +4, size +8,
 *                               canvas_num +16, clut +20
 *   ot_rgn_chn_attr         64  is_show +0, type +4, attr +8
 *   ot_rgn_overlay_chn_attr 36  point +0, fg_alpha +8, bg_alpha +12,
 *                               layer +16, qp_info +20, dst +32
 *   ot_rgn_overlay_qp_info  12
 *   ot_rgn_canvas_info      24  phys_addr +0, size +4, stride +12,
 *                               pixel_format +16, virt_addr +20
 *   ot_bmp                  16  pixel_format +0, width +4, height +8,
 *                               data +12
 *   ot_point                 8
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_RGN_H
#define HISI_V5_RGN_H

#include "v5_common.h"
#include "v5_video.h"

/*
 * The budgets, from ot_defines.h. OT_RGN_HANDLE_MAX is the global handle
 * space; OT_RGN_VENC_MAX_OVERLAY_NUM is how many overlays one *encoder
 * channel* composites, and it is the limit a caller runs into.
 */
#define V5_RGN_HANDLE_MAX 128
#define V5_RGN_OVERLAY_MAX_PER_VENC 8

/* OT_RGN_MIN_WIDTH / OT_RGN_MIN_HEIGHT, and OT_RGN_ALIGN. A region's
 * dimensions are even; the driver rejects an odd one rather than rounding
 * it. */
#define V5_RGN_MIN_DIM 2
#define V5_RGN_ALIGN 2

/* OT_RGN_OVERLAY_MAX_X / _MAX_Y / _MAX_WIDTH / _MAX_HEIGHT. */
#define V5_RGN_OVERLAY_MAX_XY 4094
#define V5_RGN_OVERLAY_MAX_DIM 4096

/* OT_RGN_OVERLAY_MAX_ALPHA, which is raptor's own range -- see the header
 * comment. */
#define V5_RGN_OVERLAY_MAX_ALPHA 255

/* OT_RGN_CLUT_NUM. */
#define V5_RGN_CLUT_NUM 16

/* ================================================================
 * GEOMETRY AND BITMAPS
 * ================================================================ */

/*
 * ot_point (ot_common.h). Signed, unlike ot_size's members, and it lives
 * here rather than in v5_common.h because RGN is the only module in this
 * backend that takes one.
 */
typedef struct {
    int x;
    int y;
} v5_point;

_Static_assert(sizeof(v5_point) == 8, "ot_point is 8 bytes");
_Static_assert(offsetof(v5_point, y) == 4, "ot_point.y at +4");

/*
 * ot_bmp (ot_common_video.h). The bitmap handed to ss_mpi_rgn_set_bmp:
 * format, geometry, and a userspace pointer the driver copies from.
 */
typedef struct {
    v5_pixel_format pixel_format;
    unsigned int width;
    unsigned int height;
    void *data;
} v5_bmp;

_Static_assert(sizeof(v5_bmp) == 16, "ot_bmp is 16 bytes");
_Static_assert(offsetof(v5_bmp, width) == 4, "ot_bmp.width at +4");
_Static_assert(offsetof(v5_bmp, height) == 8, "ot_bmp.height at +8");
_Static_assert(offsetof(v5_bmp, data) == 12, "ot_bmp.data at +12");

/* ================================================================
 * REGION ATTRIBUTES
 * ================================================================ */

/* ot_rgn_type. Nine members on V5 against gen4's four; only the first is
 * used, and COVER is named because hal_osd.c refuses it by name. */
typedef enum {
    V5_RGN_TYPE_OVERLAY = 0,
    V5_RGN_TYPE_COVER = 1,
    V5_RGN_TYPE_OVERLAYEX = 2,
    V5_RGN_TYPE_COVEREX = 3,
    V5_RGN_TYPE_LINEEX = 4,
    V5_RGN_TYPE_MOSAIC = 5,
    V5_RGN_TYPE_MOSAICEX = 6,
    V5_RGN_TYPE_CORNER_RECT = 7,
    V5_RGN_TYPE_CORNER_RECTEX = 8,
} v5_rgn_type;

/*
 * ot_rgn_overlay_attr -- what a region *is*: format, size and how many
 * canvases the driver keeps behind it. The clut is the palette for the
 * CLUT pixel formats and is zero for ARGB.
 */
typedef struct {
    v5_pixel_format pixel_format;
    unsigned int bg_color;
    v5_size size;
    unsigned int canvas_num;
    unsigned int clut[V5_RGN_CLUT_NUM];
} v5_rgn_overlay_attr;

_Static_assert(sizeof(v5_rgn_overlay_attr) == 84, "ot_rgn_overlay_attr is 84 bytes");
_Static_assert(offsetof(v5_rgn_overlay_attr, bg_color) == 4, "ot_rgn_overlay_attr.bg_color at +4");
_Static_assert(offsetof(v5_rgn_overlay_attr, size) == 8, "ot_rgn_overlay_attr.size at +8");
_Static_assert(offsetof(v5_rgn_overlay_attr, canvas_num) == 16,
               "ot_rgn_overlay_attr.canvas_num at +16");
_Static_assert(offsetof(v5_rgn_overlay_attr, clut) == 20, "ot_rgn_overlay_attr.clut at +20");

/*
 * ot_rgn_attr. The union has two arms -- overlay and overlayex -- and they
 * are the same 84 bytes, so the overlay arm alone is the whole union and
 * needs no padding.
 */
typedef struct {
    v5_rgn_type type;
    v5_rgn_overlay_attr overlay;
} v5_rgn_attr;

_Static_assert(sizeof(v5_rgn_attr) == 88, "ot_rgn_attr is 88 bytes");
_Static_assert(offsetof(v5_rgn_attr, overlay) == 4, "ot_rgn_attr.attr at +4");

/* ot_rgn_overlay_qp_info. Zeroed by raptor: disabled, relative, delta 0,
 * i.e. the encoder's own QP under the overlay. */
typedef struct {
    int enable;
    int is_abs_qp;
    int qp_val;
} v5_rgn_overlay_qp_info;

_Static_assert(sizeof(v5_rgn_overlay_qp_info) == 12, "ot_rgn_overlay_qp_info is 12 bytes");

/*
 * ot_rgn_overlay_chn_attr -- what a region *looks like on one channel*:
 * position, alpha, layer. `dst` is ot_rgn_attach_dst, which picks the JPEG
 * image an overlay lands on (main / MPF0 / MPF1) and is ignored by every
 * other codec; 0 is OT_RGN_ATTACH_JPEG_MAIN, which is what an MJPEG channel
 * wants.
 */
typedef struct {
    v5_point point;
    unsigned int fg_alpha;
    unsigned int bg_alpha;
    unsigned int layer;
    v5_rgn_overlay_qp_info qp_info;
    int dst;
} v5_rgn_overlay_chn_attr;

_Static_assert(sizeof(v5_rgn_overlay_chn_attr) == 36, "ot_rgn_overlay_chn_attr is 36 bytes");
_Static_assert(offsetof(v5_rgn_overlay_chn_attr, fg_alpha) == 8,
               "ot_rgn_overlay_chn_attr.fg_alpha at +8");
_Static_assert(offsetof(v5_rgn_overlay_chn_attr, bg_alpha) == 12,
               "ot_rgn_overlay_chn_attr.bg_alpha at +12");
_Static_assert(offsetof(v5_rgn_overlay_chn_attr, layer) == 16,
               "ot_rgn_overlay_chn_attr.layer at +16");
_Static_assert(offsetof(v5_rgn_overlay_chn_attr, qp_info) == 20,
               "ot_rgn_overlay_chn_attr.qp_info at +20");
_Static_assert(offsetof(v5_rgn_overlay_chn_attr, dst) == 32, "ot_rgn_overlay_chn_attr.dst at +32");

/*
 * ot_rgn_chn_attr, and the one place in this file where the transcription
 * has to say something the vendor header says implicitly.
 *
 * The union is 56 bytes wide because ot_rgn_cover_chn_attr is 56; the arm
 * raptor uses is 36. Writing the struct as show + type + overlay would make
 * it 48 bytes, and every attach and display-attr call would hand the driver
 * a buffer 16 bytes short of what it copies. The pad is the union's tail.
 */
typedef struct {
    int show;
    v5_rgn_type type;
    v5_rgn_overlay_chn_attr overlay;
    unsigned char union_pad[20];
} v5_rgn_chn_attr;

_Static_assert(sizeof(v5_rgn_chn_attr) == 64, "ot_rgn_chn_attr is 64 bytes");
_Static_assert(offsetof(v5_rgn_chn_attr, type) == 4, "ot_rgn_chn_attr.type at +4");
_Static_assert(offsetof(v5_rgn_chn_attr, overlay) == 8, "ot_rgn_chn_attr.attr at +8");

/*
 * ot_rgn_canvas_info. Transcribed for completeness of the module's surface
 * and because the loader resolves the canvas pair as optional diagnostics;
 * see THE CANVAS API IS NOT USED. td_phys_addr_t is 32-bit on this part,
 * the same as everywhere else in this backend.
 */
typedef struct {
    unsigned int phys_addr;
    v5_size size;
    unsigned int stride;
    v5_pixel_format pixel_format;
    void *virt_addr;
} v5_rgn_canvas_info;

_Static_assert(sizeof(v5_rgn_canvas_info) == 24, "ot_rgn_canvas_info is 24 bytes");
_Static_assert(offsetof(v5_rgn_canvas_info, size) == 4, "ot_rgn_canvas_info.size at +4");
_Static_assert(offsetof(v5_rgn_canvas_info, stride) == 12, "ot_rgn_canvas_info.stride at +12");
_Static_assert(offsetof(v5_rgn_canvas_info, pixel_format) == 16,
               "ot_rgn_canvas_info.pixel_format at +16");
_Static_assert(offsetof(v5_rgn_canvas_info, virt_addr) == 20,
               "ot_rgn_canvas_info.virt_addr at +20");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    int (*fnCreate)(unsigned int handle, const v5_rgn_attr *attr);
    int (*fnDestroy)(unsigned int handle);

    int (*fnGetAttr)(unsigned int handle, v5_rgn_attr *attr);
    int (*fnSetAttr)(unsigned int handle, const v5_rgn_attr *attr);

    int (*fnSetBitMap)(unsigned int handle, const v5_bmp *bmp);

    int (*fnAttachToChn)(unsigned int handle, const v5_mpp_chn *chn,
                         const v5_rgn_chn_attr *chn_attr);
    int (*fnDetachFromChn)(unsigned int handle, const v5_mpp_chn *chn);

    int (*fnSetDisplayAttr)(unsigned int handle, const v5_mpp_chn *chn,
                            const v5_rgn_chn_attr *chn_attr);
    int (*fnGetDisplayAttr)(unsigned int handle, const v5_mpp_chn *chn, v5_rgn_chn_attr *chn_attr);
} v5_rgn_impl;

/*
 * v5_rgn_load -- bind the RGN entry points.
 *
 * Required versus optional is drawn where it is everywhere else in this
 * backend: at "can the subsystem come up at all". Create, Destroy, SetBmp,
 * AttachToChn, DetachFromChn and SetChnDisplayAttr are the six calls an
 * overlay's whole life is made of, so a board missing any of them has no
 * OSD and the caller is told so with RSS_ERR_NOTSUP -- which rvd turns into
 * "overlays disabled" and keeps streaming.
 *
 * The three Get/Set-attr calls are optional and cost only a diagnostic.
 * hal_osd.c tracks every attribute it sets, so it never needs to read one
 * back; get_attr is used for one thing only, telling an existing region
 * from a fresh handle when a create is repeated, and there is a fallback
 * for its absence.
 *
 * As on gen4 there is nothing to initialise: RGN has no module lifecycle of
 * its own, so the first create is the subsystem coming up. Every symbol
 * lives in libss_mpi.so beside SYS, VI, VPSS and VENC.
 */
static inline int v5_rgn_load(v5_rgn_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_rgn";

    memset(lib, 0, sizeof(*lib));

#define V5_RGN_REQ(field, type, name)                                                              \
    do {                                                                                           \
        if (!(lib->field = (type)v5_symbol(mod, libs, name)))                                      \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V5_RGN_REQ(fnCreate, int (*)(unsigned int, const v5_rgn_attr *), "ss_mpi_rgn_create");
    V5_RGN_REQ(fnDestroy, int (*)(unsigned int), "ss_mpi_rgn_destroy");
    V5_RGN_REQ(fnSetBitMap, int (*)(unsigned int, const v5_bmp *), "ss_mpi_rgn_set_bmp");
    V5_RGN_REQ(fnAttachToChn, int (*)(unsigned int, const v5_mpp_chn *, const v5_rgn_chn_attr *),
               "ss_mpi_rgn_attach_to_chn");
    V5_RGN_REQ(fnDetachFromChn, int (*)(unsigned int, const v5_mpp_chn *),
               "ss_mpi_rgn_detach_from_chn");
    V5_RGN_REQ(fnSetDisplayAttr, int (*)(unsigned int, const v5_mpp_chn *, const v5_rgn_chn_attr *),
               "ss_mpi_rgn_set_chn_display_attr");

#undef V5_RGN_REQ

    lib->fnGetAttr =
        (int (*)(unsigned int, v5_rgn_attr *))v5_symbol_opt(libs, "ss_mpi_rgn_get_attr");
    lib->fnSetAttr =
        (int (*)(unsigned int, const v5_rgn_attr *))v5_symbol_opt(libs, "ss_mpi_rgn_set_attr");
    lib->fnGetDisplayAttr =
        (int (*)(unsigned int, const v5_mpp_chn *, v5_rgn_chn_attr *))v5_symbol_opt(
            libs, "ss_mpi_rgn_get_chn_display_attr");

    return RSS_OK;
}

static inline void v5_rgn_unload(v5_rgn_impl *lib)
{
    /* No handle of its own; hisi_mpi_close owns those. Clearing the table
     * turns a use-after-deinit into a NULL check rather than a call into a
     * dlclosed mapping, exactly as v5_sys_unload does. */
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_RGN_H */
