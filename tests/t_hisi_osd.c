/*
 * t_hisi_osd -- the gen4 OSD backend, host-side.
 *
 * Includes the real hal_osd.c and stubs the eight RGN entry points by
 * filling in hisi_state_t's v4_rgn_impl, which works for the same reason
 * every suite here works: nothing in src/hisi_v4 links a vendor .so.
 *
 * What it is actually asking, in the order the cases run:
 *
 *   1. THE DIVINUS BUG. Two regions, two encoder channels, two positions.
 *      divinus builds its attach descriptor without setting .channel
 *      (v4_hal.c:398), so every region lands on VENC 0 and a two-stream
 *      camera shows both overlays on the main stream and none on the sub.
 *      The assertion is on the descriptor: region A's attach names channel
 *      0, region B's names channel 1, both name V4_MOD_VENC, and their
 *      points differ.
 *
 *   2. THE DEFERRAL. rvd registers regions and sets their attributes
 *      before the encoder channel they belong to exists. Nothing may reach
 *      RGN in that window, and everything set during it must survive to
 *      the attach -- so the case sets a *different* position after
 *      registering and asserts the attach carries the later one.
 *
 *   3. THE BITMAP. An odd-sized region, because that is where the
 *      conversion can read past the caller's buffer: RGN wants even
 *      dimensions, rvd renders what it renders, and hisi_osd_convert has
 *      to pad rather than over-read. Geometry, format, the ARGB1555
 *      packing and the transparent padding are all checked -- and ASan
 *      checks the over-read, which is the half an assertion cannot.
 *
 *   4. THE ENCODER RESTART. detach_chn then flush_pending, which is what
 *      hal_enc_destroy_channel and hal_enc_create_channel do around a
 *      channel rvd rebuilds. The region must come back with its position,
 *      layer and alpha intact and without being recreated.
 *
 *   5. The refusals: RSS_OSD_COVER (no VENC budget for COVER_RGN on gen4)
 *      and the ninth region on one channel (OVERLAY_MAX_NUM_VENC is 8).
 *
 *   6. Teardown, under ASan: every region destroyed, every conversion
 *      buffer freed, once each.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "../src/hisi_v4/hal_osd.c"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------- log capture ---------------- */

static char g_log[64 * 1024];
static size_t g_log_len;

static void t_log(int level, const char *file, int line, const char *fmt, ...)
{
    char msg[1024];
    va_list ap;
    int n;

    (void)level;
    (void)file;
    (void)line;
    va_start(ap, fmt);
    n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;

    n = snprintf(g_log + g_log_len, sizeof(g_log) - g_log_len, "%s\n", msg);
    if (n > 0 && (size_t)n < sizeof(g_log) - g_log_len)
        g_log_len += (size_t)n;
}

rss_hal_log_func_t rss_hal_log_fn = t_log;

static int log_count(const char *needle)
{
    const char *p = g_log;
    int n = 0;

    while ((p = strstr(p, needle))) {
        n++;
        p += strlen(needle);
    }
    return n;
}

/* ---------------- RGN stubs ---------------- */

/*
 * Every call is recorded rather than counted, because most of what this
 * suite asserts is about an *argument* -- the attach descriptor's channel,
 * the bitmap's width -- and a counter cannot answer that.
 */
enum { CALL_MAX = 64 };

typedef struct {
    unsigned int handle;
    v4_rgn_attr attr;
} rec_create_t;

typedef struct {
    unsigned int handle;
    v4_mpp_chn chn;
    v4_rgn_chn_attr ca;
} rec_chn_t;

typedef struct {
    unsigned int handle;
    v4_rgn_pixfmt fmt;
    unsigned int width;
    unsigned int height;
    /* A copy of the converted bitmap, so the assertions can look at pixels
     * after hal_osd.c's own buffer has been reused or freed. */
    uint16_t px[8 * 1024];
    size_t px_n;
} rec_bmp_t;

static rec_create_t g_create[CALL_MAX];
static int g_create_n;
static unsigned int g_destroy[CALL_MAX];
static int g_destroy_n;
static rec_chn_t g_attach[CALL_MAX];
static int g_attach_n;
static rec_chn_t g_detach[CALL_MAX];
static int g_detach_n;
static rec_chn_t g_display[CALL_MAX];
static int g_display_n;
static rec_bmp_t g_bmp[CALL_MAX];
static int g_bmp_n;

/* Which handles the driver believes exist, for the GetAttr path. */
static bool g_exists[HISI_OSD_REGION_MAX + 1];
static bool g_get_attr_absent; /* pretend HI_MPI_RGN_GetAttr never resolved */
static int g_create_fail_ret;  /* non-zero -> Create returns this once */

static int stub_create(unsigned int handle, const v4_rgn_attr *attr)
{
    assert(g_create_n < CALL_MAX);
    g_create[g_create_n].handle = handle;
    g_create[g_create_n].attr = *attr;
    g_create_n++;

    if (g_create_fail_ret) {
        int r = g_create_fail_ret;

        g_create_fail_ret = 0;
        return r;
    }
    if (handle <= HISI_OSD_REGION_MAX)
        g_exists[handle] = true;
    return 0;
}

static int stub_destroy(unsigned int handle)
{
    assert(g_destroy_n < CALL_MAX);
    g_destroy[g_destroy_n++] = handle;
    if (handle <= HISI_OSD_REGION_MAX)
        g_exists[handle] = false;
    return 0;
}

static int stub_get_attr(unsigned int handle, v4_rgn_attr *attr)
{
    if (g_get_attr_absent)
        return -1;
    if (handle > HISI_OSD_REGION_MAX || !g_exists[handle])
        return 0xA0038005; /* RGN / ERROR / EN_ERR_UNEXIST */
    memset(attr, 0, sizeof(*attr));
    return 0;
}

static int stub_set_bitmap(unsigned int handle, const v4_bitmap *bmp)
{
    rec_bmp_t *r;
    size_t n;

    assert(g_bmp_n < CALL_MAX);
    r = &g_bmp[g_bmp_n++];
    r->handle = handle;
    r->fmt = bmp->pixel_format;
    r->width = bmp->width;
    r->height = bmp->height;

    n = (size_t)bmp->width * bmp->height;
    if (n > sizeof(r->px) / sizeof(r->px[0]))
        n = sizeof(r->px) / sizeof(r->px[0]);
    /* Reads the whole buffer the backend claims to have written, so ASan
     * fires here if the conversion under-filled it. */
    memcpy(r->px, bmp->data, n * sizeof(uint16_t));
    r->px_n = n;
    return 0;
}

static int stub_attach(unsigned int handle, const v4_mpp_chn *chn, const v4_rgn_chn_attr *ca)
{
    assert(g_attach_n < CALL_MAX);
    g_attach[g_attach_n].handle = handle;
    g_attach[g_attach_n].chn = *chn;
    g_attach[g_attach_n].ca = *ca;
    g_attach_n++;
    return 0;
}

static int stub_detach(unsigned int handle, const v4_mpp_chn *chn)
{
    assert(g_detach_n < CALL_MAX);
    g_detach[g_detach_n].handle = handle;
    g_detach[g_detach_n].chn = *chn;
    memset(&g_detach[g_detach_n].ca, 0, sizeof(g_detach[g_detach_n].ca));
    g_detach_n++;
    return 0;
}

static int stub_display(unsigned int handle, const v4_mpp_chn *chn, const v4_rgn_chn_attr *ca)
{
    assert(g_display_n < CALL_MAX);
    g_display[g_display_n].handle = handle;
    g_display[g_display_n].chn = *chn;
    g_display[g_display_n].ca = *ca;
    g_display_n++;
    return 0;
}

/* ---------------- fixture ---------------- */

static hisi_state_t g_st;
static rss_hal_ctx_t g_ctx;

static void reset_all(void)
{
    int i;

    g_create_n = g_destroy_n = g_attach_n = g_detach_n = g_display_n = g_bmp_n = 0;
    memset(g_exists, 0, sizeof(g_exists));
    g_get_attr_absent = false;
    g_create_fail_ret = 0;
    g_log[0] = '\0';
    g_log_len = 0;

    memset(&g_st, 0, sizeof(g_st));
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.platform = &g_st;

    for (i = 0; i < HISI_VENC_CHN_NUM; i++) {
        g_st.enc[i].bound_fs = -1;
        g_st.enc[i].idle_fs = -1;
        g_st.enc[i].fd = -1;
        g_st.osd_src_fs[i] = -1;
    }

    g_st.rgn_loaded = true;
    g_st.rgn.fnCreate = stub_create;
    g_st.rgn.fnDestroy = stub_destroy;
    g_st.rgn.fnGetAttr = stub_get_attr;
    g_st.rgn.fnSetBitMap = stub_set_bitmap;
    g_st.rgn.fnAttachToChn = stub_attach;
    g_st.rgn.fnDetachFromChn = stub_detach;
    g_st.rgn.fnSetDisplayAttr = stub_display;
}

/* Stands in for hal_enc_create_channel: the channel exists, and the
 * backend's own hook flushes whatever was waiting on it. */
static void enc_create(int chn)
{
    g_st.enc[chn].created = true;
    hisi_osd_flush_pending(&g_st, chn);
}

/* ...and for hal_enc_destroy_channel, whose first act is the detach. */
static void enc_destroy(int chn)
{
    hisi_osd_detach_chn(&g_st, chn);
    g_st.enc[chn].created = false;
}

static rss_osd_region_t mk_attr(int x, int y, int w, int h, int layer)
{
    rss_osd_region_t a;

    memset(&a, 0, sizeof(a));
    a.type = RSS_OSD_PIC;
    a.x = x;
    a.y = y;
    a.width = w;
    a.height = h;
    a.bitmap_fmt = RSS_PIXFMT_BGRA;
    a.global_alpha_en = true;
    a.fg_alpha = 255;
    a.bg_alpha = 0;
    a.layer = layer;
    return a;
}

/* The one attach recorded for a handle, or NULL. */
static const rec_chn_t *attach_for(unsigned int handle)
{
    int i;

    for (i = 0; i < g_attach_n; i++)
        if (g_attach[i].handle == handle)
            return &g_attach[i];
    return NULL;
}

/* ================================================================
 * CASE 1 + 2 -- per-channel attach, and the deferral
 * ================================================================ */

static void case_two_channels_deferred(void)
{
    rss_osd_region_t a, b;
    const rec_chn_t *at;
    int ha = -1, hb = -1;

    reset_all();

    /*
     * rvd's order, verbatim: create_group, create_region, register_region,
     * set_region_attr, show_region -- all of it before the encoder channel
     * exists and before the bind (rvd_pipeline.c calls osd_create_group
     * then rvd_osd_init_stream, and the bind loop after).
     */
    assert(hal_osd_create_group(&g_ctx, 0) == RSS_OK);
    assert(hal_osd_create_group(&g_ctx, 1) == RSS_OK);

    a = mk_attr(10, 10, 100, 40, 1);
    b = mk_attr(200, 300, 120, 40, 1);

    assert(hal_osd_create_region(&g_ctx, &ha, &a) == RSS_OK);
    assert(hal_osd_create_region(&g_ctx, &hb, &b) == RSS_OK);
    assert(ha == 0 && hb == 1);
    assert(g_create_n == 2);

    /* The region attr the driver was given: ARGB1555, the requested size,
     * and a canvas count that is not zero. */
    assert(g_create[0].attr.type == V4_RGN_TYPE_OVERLAY);
    assert(g_create[0].attr.overlay.pixel_format == V4_RGN_PIXFMT_ARGB1555);
    assert(g_create[0].attr.overlay.size.width == 100);
    assert(g_create[0].attr.overlay.size.height == 40);
    assert(g_create[0].attr.overlay.canvas_num >= 2);

    assert(hal_osd_register_region(&g_ctx, ha, 0) == RSS_OK);
    assert(hal_osd_register_region(&g_ctx, hb, 1) == RSS_OK);

    /* THE DEFERRAL: neither encoder channel exists, so nothing has reached
     * RGN's channel side at all. */
    assert(g_attach_n == 0);
    assert(g_display_n == 0);

    /* rvd now sets the attribute -- with a *moved* position, which is what
     * makes this a test of the deferral rather than of the register. Still
     * nothing reaches RGN, and still no failure is reported. */
    a = mk_attr(12, 14, 100, 40, 1);
    assert(hal_osd_set_region_attr(&g_ctx, ha, &a) == RSS_OK);
    assert(hal_osd_show_region(&g_ctx, ha, 0, 0, 2) == RSS_OK);
    assert(g_attach_n == 0);
    assert(g_display_n == 0);
    /* And no region was recreated behind the caller's back: the geometry
     * did not change. */
    assert(g_create_n == 2);
    assert(g_destroy_n == 0);

    /* The encoder channels come up, and hal_bind flushes. Two calls, in
     * the two places the shipping code makes them. */
    enc_create(0);
    enc_create(1);
    assert(g_attach_n == 2);

    /*
     * THE DIVINUS BUG, stated as an assertion. Region A is on VENC channel
     * 0 and region B on VENC channel 1 -- not both on 0, which is what an
     * unset .channel gives.
     */
    at = attach_for((unsigned int)ha);
    assert(at);
    assert(at->chn.module == V4_MOD_VENC);
    assert(at->chn.device == 0);
    assert(at->chn.channel == 0);

    at = attach_for((unsigned int)hb);
    assert(at);
    assert(at->chn.module == V4_MOD_VENC);
    assert(at->chn.device == 0);
    assert(at->chn.channel == 1);

    /* Different positions, and A's is the one set *after* it registered --
     * the deferred set_region_attr survived to the attach. */
    at = attach_for((unsigned int)ha);
    assert(at->ca.overlay.point.x == 12);
    assert(at->ca.overlay.point.y == 14);
    at = attach_for((unsigned int)hb);
    assert(at->ca.overlay.point.x == 200);
    assert(at->ca.overlay.point.y == 300);

    /* Alpha is scaled into OVERLAY's [0,128], not passed through as
     * rvd's [0,255]: 255 -> 128 exactly, and 0 stays 0. */
    at = attach_for((unsigned int)ha);
    assert(at->ca.overlay.fg_alpha == 128);
    assert(at->ca.overlay.bg_alpha == 0);
    assert(at->ca.type == V4_RGN_TYPE_OVERLAY);
    /* rvd created it hidden. */
    assert(at->ca.show == 0);
    /* The layer show_region carried, clamped into OVERLAY's [0,7]. */
    assert(at->ca.overlay.layer == 2);

    /* A second flush changes nothing: attach is idempotent. */
    hisi_osd_flush_pending(&g_st, 0);
    hisi_osd_flush_pending(&g_st, 1);
    assert(g_attach_n == 2);

    /* Now that the regions are attached, show_region reaches RGN. */
    assert(hal_osd_show_region(&g_ctx, ha, 0, 1, 3) == RSS_OK);
    assert(g_display_n == 1);
    assert(g_display[0].handle == (unsigned int)ha);
    assert(g_display[0].chn.channel == 0);
    assert(g_display[0].ca.show == 1);
    assert(g_display[0].ca.overlay.layer == 3);

    /* A layer past OVERLAY's range is clamped rather than handed over: the
     * driver rejects the whole call for an out-of-range layer. */
    assert(hal_osd_show_region(&g_ctx, hb, 1, 1, 99) == RSS_OK);
    assert(g_display_n == 2);
    assert(g_display[1].ca.overlay.layer == 7);

    hal_osd_destroy_region(&g_ctx, ha);
    hal_osd_destroy_region(&g_ctx, hb);
    hisi_osd_release_all(&g_st);
}

/* ================================================================
 * CASE 3 -- the bitmap
 * ================================================================ */

static void case_bitmap(void)
{
    /* Odd in both dimensions, which is the case RGN cannot take and the
     * caller is allowed to ask for. */
    const int w = 101, h = 41;
    rss_osd_region_t a;
    uint8_t *src;
    const rec_bmp_t *b;
    int handle = -1;
    int i;

    reset_all();

    a = mk_attr(21, 33, w, h, 0);
    assert(hal_osd_create_region(&g_ctx, &handle, &a) == RSS_OK);

    /* Rounded up to even for RGN, and the position rounded *down* so the
     * region cannot walk off the picture. */
    assert(g_create[0].attr.overlay.size.width == 102);
    assert(g_create[0].attr.overlay.size.height == 42);
    assert(hal_osd_register_region(&g_ctx, handle, 0) == RSS_OK);
    enc_create(0);
    assert(attach_for((unsigned int)handle)->ca.overlay.point.x == 20);
    assert(attach_for((unsigned int)handle)->ca.overlay.point.y == 32);

    /*
     * Exactly what rvd hands over: width * height * 4 BGRA bytes, no more.
     * Allocated rather than static so ASan has a real redzone to catch the
     * conversion reading past it -- which it would, if the loop used the
     * region's aligned width to index the source.
     */
    src = malloc((size_t)w * h * 4);
    assert(src);
    for (i = 0; i < w * h; i++) {
        src[i * 4 + 0] = 0x18; /* B */
        src[i * 4 + 1] = 0x28; /* G */
        src[i * 4 + 2] = 0xF8; /* R */
        src[i * 4 + 3] = (uint8_t)(i == 0 ? 0 : 0xFF);
    }

    /* A NULL bitmap is rvd's Ingenic sequence and not an error. */
    assert(hal_osd_update_region_data(&g_ctx, handle, NULL) == RSS_OK);
    assert(g_bmp_n == 0);

    assert(hal_osd_update_region_data(&g_ctx, handle, src) == RSS_OK);
    assert(g_bmp_n == 1);

    b = &g_bmp[0];
    assert(b->handle == (unsigned int)handle);
    assert(b->fmt == V4_RGN_PIXFMT_ARGB1555);
    /* The geometry RGN is told is the region's, not the caller's. */
    assert(b->width == 102);
    assert(b->height == 42);
    assert(b->px_n == 102u * 42u);

    /* Pixel 0 had alpha 0 -> the alpha bit clear; pixel 1 had alpha 255 ->
     * set. Colour is the top five bits of each channel: F8 -> 31,
     * 28 -> 5, 18 -> 3. */
    assert(b->px[0] == (uint16_t)((31u << 10) | (5u << 5) | 3u));
    assert(b->px[1] == (uint16_t)(0x8000u | (31u << 10) | (5u << 5) | 3u));

    /*
     * The padding column and row are transparent rather than whatever the
     * allocator left there. Rows 0..40 are the caller's; row 41 is pad, as
     * is column 101 of every row.
     */
    assert(b->px[101] == 0);                    /* row 0, pad column */
    assert(b->px[40 * 102 + 100] != 0);         /* last real pixel */
    assert(b->px[40 * 102 + 101] == 0);         /* last real row, pad column */
    for (i = 0; i < 102; i++)
        assert(b->px[41 * 102 + i] == 0);       /* the whole pad row */

    free(src);

    hisi_osd_release_all(&g_st);
}

/* ================================================================
 * CASE 4 -- the encoder restart
 * ================================================================ */

static void case_encoder_restart(void)
{
    rss_osd_region_t a;
    const rec_chn_t *at;
    int handle = -1;

    reset_all();

    a = mk_attr(40, 50, 64, 32, 3);
    assert(hal_osd_create_region(&g_ctx, &handle, &a) == RSS_OK);
    assert(hal_osd_register_region(&g_ctx, handle, 1) == RSS_OK);
    enc_create(1);
    assert(g_attach_n == 1);
    assert(hal_osd_show_region(&g_ctx, handle, 1, 1, 3) == RSS_OK);

    /* rvd reconfigures the stream: destroy the encoder channel, make it
     * again. The detach is what lets HI_MPI_VENC_DestroyChn succeed. */
    enc_destroy(1);
    assert(g_detach_n == 1);
    assert(g_detach[0].handle == (unsigned int)handle);
    assert(g_detach[0].chn.channel == 1);
    assert(g_detach[0].chn.module == V4_MOD_VENC);

    /* The registration survived the detach, so nothing was destroyed and
     * nothing has to be created again. */
    assert(g_destroy_n == 0);
    assert(g_st.osd[handle].grp == 1);
    assert(!g_st.osd[handle].attached);

    enc_create(1);
    assert(g_attach_n == 2);
    assert(g_create_n == 1); /* the region itself was never recreated */

    at = &g_attach[1];
    assert(at->handle == (unsigned int)handle);
    assert(at->chn.channel == 1);
    /* Position, layer, alpha and visibility all came back. */
    assert(at->ca.overlay.point.x == 40);
    assert(at->ca.overlay.point.y == 50);
    assert(at->ca.overlay.layer == 3);
    assert(at->ca.overlay.fg_alpha == 128);
    assert(at->ca.show == 1);

    hisi_osd_release_all(&g_st);
}

/* ================================================================
 * CASE 5 -- the refusals, and the stale-handle recreate
 * ================================================================ */

static void case_refusals(void)
{
    rss_osd_region_t a;
    int handles[V4_RGN_OVERLAY_MAX_PER_VENC + 1];
    int extra = -1;
    int i;

    reset_all();

    /* COVER_RGN has no VENC budget entry on gen4, so a privacy cover
     * cannot be composited by the encoder. rvd reads the NOTSUP and leaves
     * privacy_handles[s] at -1. */
    a = mk_attr(0, 0, 1920, 1080, 0);
    a.type = RSS_OSD_COVER;
    a.cover_color = 0xFF000000u;
    assert(hal_osd_create_region(&g_ctx, &extra, &a) == RSS_ERR_NOTSUP);
    assert(g_create_n == 0);

    /* Empty geometry is a caller error, not a capability. */
    a = mk_attr(0, 0, 0, 20, 0);
    assert(hal_osd_create_region(&g_ctx, &extra, &a) == RSS_ERR_INVAL);

    /* OVERLAY_MAX_NUM_VENC is 8 per encoder channel. The ninth is refused
     * here rather than at the driver, because on the deferred path the
     * driver's refusal would arrive inside hal_bind where nothing can
     * report it. */
    a = mk_attr(10, 10, 32, 16, 0);
    for (i = 0; i < V4_RGN_OVERLAY_MAX_PER_VENC; i++) {
        handles[i] = -1;
        assert(hal_osd_create_region(&g_ctx, &handles[i], &a) == RSS_OK);
        assert(hal_osd_register_region(&g_ctx, handles[i], 0) == RSS_OK);
    }
    assert(hal_osd_create_region(&g_ctx, &extra, &a) == RSS_OK);
    assert(hal_osd_register_region(&g_ctx, extra, 0) == RSS_ERR_NOMEM);
    /* ...but the same region on the *other* channel is fine. */
    assert(hal_osd_register_region(&g_ctx, extra, 1) == RSS_OK);

    /* An unknown handle is NOENT rather than a crash. */
    assert(hal_osd_show_region(&g_ctx, 99, 0, 1, 0) == RSS_ERR_NOENT);
    assert(hal_osd_destroy_region(&g_ctx, 99) == RSS_ERR_NOENT);

    hisi_osd_release_all(&g_st);

    /*
     * A handle left live by a previous process. RGN state is in the kernel,
     * so a Create can meet a region this process never made; the backend
     * destroys it and creates its own. Checked on both paths: with GetAttr
     * available, and with it absent so the EN_ERR_EXIST retry runs.
     */
    reset_all();
    g_exists[0] = true;
    a = mk_attr(0, 0, 32, 16, 0);
    assert(hal_osd_create_region(&g_ctx, &extra, &a) == RSS_OK);
    assert(g_destroy_n == 1 && g_destroy[0] == 0);
    assert(g_create_n == 1);
    assert(log_count("already exists") == 1);
    hisi_osd_release_all(&g_st);

    reset_all();
    g_get_attr_absent = true;
    g_create_fail_ret = 0xA0038004; /* RGN / ERROR / EN_ERR_EXIST */
    assert(hal_osd_create_region(&g_ctx, &extra, &a) == RSS_OK);
    assert(g_create_n == 2); /* the refused one, then the retry */
    assert(g_destroy_n == 1);
    hisi_osd_release_all(&g_st);
}

/* ================================================================
 * CASE 6 -- resize, group teardown, and a clean release
 * ================================================================ */

static void case_resize_and_teardown(void)
{
    rss_osd_region_t a;
    int h0 = -1, h1 = -1;

    reset_all();

    assert(hal_osd_create_group(&g_ctx, 0) == RSS_OK);
    a = mk_attr(10, 10, 64, 32, 0);
    assert(hal_osd_create_region(&g_ctx, &h0, &a) == RSS_OK);
    assert(hal_osd_create_region(&g_ctx, &h1, &a) == RSS_OK);
    assert(hal_osd_register_region(&g_ctx, h0, 0) == RSS_OK);
    assert(hal_osd_register_region(&g_ctx, h1, 0) == RSS_OK);
    enc_create(0);
    assert(g_attach_n == 2);

    /* A bitmap, so there is a conversion buffer for the resize to free and
     * ASan has something to complain about if it is freed twice. */
    {
        uint8_t *src = calloc((size_t)64 * 32, 4);

        assert(src);
        assert(hal_osd_update_region_data(&g_ctx, h0, src) == RSS_OK);
        free(src);
    }

    /* A size change is destroy + create + re-attach, because RGN fixes a
     * region's canvases at create time. */
    a = mk_attr(10, 10, 96, 48, 0);
    assert(hal_osd_set_region_attr(&g_ctx, h0, &a) == RSS_OK);
    assert(g_destroy_n == 1 && g_destroy[0] == (unsigned int)h0);
    assert(g_create_n == 3);
    assert(g_create[2].attr.overlay.size.width == 96);
    assert(g_detach_n == 1);
    assert(g_attach_n == 3);
    assert(g_attach[2].chn.channel == 0);

    /* The new geometry reaches SetBitMap too. */
    {
        uint8_t *src = calloc((size_t)96 * 48, 4);

        assert(src);
        assert(hal_osd_update_region_data(&g_ctx, h0, src) == RSS_OK);
        free(src);
    }
    assert(g_bmp[g_bmp_n - 1].width == 96 && g_bmp[g_bmp_n - 1].height == 48);

    /* Destroying the group detaches its members and unregisters them, and
     * leaves the regions themselves alive -- rvd tears one stream down and
     * leaves another running. */
    assert(hal_osd_destroy_group(&g_ctx, 0) == RSS_OK);
    assert(g_detach_n == 3);
    assert(g_st.osd[h0].grp == -1);
    assert(g_st.osd[h1].grp == -1);
    assert(g_st.osd[h0].used && g_st.osd[h1].used);

    /* Explicit destroy of one, release_all for the rest: both paths free
     * the conversion buffer exactly once, which is what ASan is here for. */
    assert(hal_osd_destroy_region(&g_ctx, h0) == RSS_OK);
    assert(!g_st.osd[h0].used);
    hisi_osd_release_all(&g_st);
    assert(!g_st.osd[h1].used);

    /* Idempotent: a second release finds nothing and touches nothing. */
    {
        int destroys = g_destroy_n;

        hisi_osd_release_all(&g_st);
        assert(g_destroy_n == destroys);
    }
}

/* ================================================================
 * CASE 7 -- a board whose libmpi has no RGN
 * ================================================================ */

static void case_no_rgn(void)
{
    rss_osd_region_t a = mk_attr(0, 0, 32, 16, 0);
    int handle = -1;

    reset_all();
    g_st.rgn_loaded = false;

    /* Every op answers NOTSUP, which is what rvd reads as "overlays
     * disabled". Nothing reaches the (still-installed) stubs. */
    assert(hal_osd_create_group(&g_ctx, 0) == RSS_ERR_NOTSUP);
    assert(hal_osd_create_region(&g_ctx, &handle, &a) == RSS_ERR_NOTSUP);
    assert(hal_osd_register_region(&g_ctx, 0, 0) == RSS_ERR_NOTSUP);
    assert(hal_osd_update_region_data(&g_ctx, 0, NULL) == RSS_ERR_NOTSUP);
    assert(hal_osd_show_region(&g_ctx, 0, 0, 1, 0) == RSS_ERR_NOTSUP);
    assert(g_create_n == 0 && g_attach_n == 0);

    /* ...except the two rvd calls unconditionally around every stream, and
     * the pool size it asks for because Ingenic needs one. Answering
     * NOTSUP there would read in rvd's log like a broken OSD. */
    assert(hal_osd_set_pool_size(&g_ctx, 448 * 1024) == RSS_OK);
    assert(hal_osd_start(&g_ctx, 0) == RSS_OK);
    assert(hal_osd_stop(&g_ctx, 0) == RSS_OK);

    /* The flush hooks are safe on a backend with no RGN at all. */
    hisi_osd_flush_pending(&g_st, 0);
    hisi_osd_detach_chn(&g_st, 0);
    hisi_osd_release_all(&g_st);
}

int main(void)
{
    case_two_channels_deferred();
    case_bitmap();
    case_encoder_restart();
    case_refusals();
    case_resize_and_teardown();
    case_no_rgn();

    printf("t_hisi_osd: OK\n");
    return 0;
}
