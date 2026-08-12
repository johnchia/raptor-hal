/*
 * infinity6c/hal_osd.c -- OSD/overlay ops for Infinity6C, over MI_RGN (MI 3.0).
 *
 * Counterpart to star/hal_osd.c; that file's header explains the model this one
 * shares, so only the two things MI 3.0 does differently are written out here.
 *
 * WHERE THE OVERLAY ATTACHES
 *
 * The i6e backend attaches a region to the VPE output port that feeds an
 * encoder. This one attaches to the SCL output port (I6C_SYS_MOD_SCL, the
 * scaler port that feeds the encoder), which is what the working i6c reference
 * does -- waybeam attaches RGN to the SCL module, not VENC. It is not a free
 * choice: this backend's main H.26x channel is fed by a HW ring from SCL, and
 * MI_RGN cannot attach to a ring-input VENC channel -- the attach is refused
 * with RGN ILLEGAL_PARAM. (divinus attaches to VENC because its channels are
 * frame-based; raptor's are not.) The port is the one recorded in
 * osd_src_port[] as rvd's FS -> OSD bind names it -- the whole reason that array
 * exists.
 *
 * A LIMITATION FOR THE SUB STREAM
 *
 * A second video stream is a VENC main->sub cascade off the main's one SCL
 * output port (see i6c_bind_scl_to_venc), and the cascade taps the frame the
 * main receives *before* the SCL port's RGN overlay is applied -- board-checked:
 * the sub carries no timestamp while the main does. So OSD lands on the main
 * stream only. The sub has no SCL port to attach to, and MI_RGN refuses a VENC
 * channel fed by a ring (the ILLEGAL_PARAM this backend first hit), so there is
 * no attach point for it here. The sub's regions are registered by rvd but their
 * osd_src_port never resolves to a live port, so they defer harmlessly rather
 * than erroring. Per-stream sub OSD would need the sub off its own SCL port
 * (frame-based, not the ring cascade), which is a bigger topology change.
 *
 * WHY ATTACH IS DEFERRED
 *
 * rvd creates and registers every region before the SCL port that carries a
 * stream is live. register_region records the intent; i6c_osd_flush_pending
 * performs the attach from i6c_bind_scl_to_venc, once that port is enabled and
 * bound.
 *
 * The SoC id leads every MI_RGN call (I6C_SOC_ID); see i6c_rgn.h.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "infinity6c_state.h"

#include <stdlib.h>
#include <string.h>

/*
 * No HAL_MODULE_VIDEO guard, deliberately: the Makefile passes that flag only to
 * hal_common_{video,audio}.o, so a guard here would compile the file away and
 * the ops table would fail to link. Membership in VIDEO_SRCS keeps it out of the
 * audio archive.
 */

static infinity6c_state_t *osd_state(void *ctx)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;

    return hal ? (infinity6c_state_t *)hal->platform : NULL;
}

static unsigned int i6c_osd_bpp(i6c_rgn_pixfmt fmt)
{
    return fmt == I6C_RGN_PIXFMT_ARGB8888 ? 4 : 2;
}

static const char *i6c_osd_fmt_name(i6c_rgn_pixfmt fmt)
{
    switch (fmt) {
    case I6C_RGN_PIXFMT_ARGB8888:
        return "ARGB8888";
    case I6C_RGN_PIXFMT_ARGB4444:
        return "ARGB4444";
    case I6C_RGN_PIXFMT_ARGB1555:
        return "ARGB1555";
    default:
        return "?";
    }
}

static infinity6c_osd_region_t *i6c_osd_region(infinity6c_state_t *st, int handle)
{
    if (!st || handle < 0 || handle >= I6C_OSD_REGION_MAX)
        return NULL;
    if (!st->osd[handle].used)
        return NULL;

    return &st->osd[handle];
}

/*
 * The SCL output port a region attaches to for a given group.
 *
 * Groups are encoder channels; the port is whichever SCL output port rvd's
 * FS -> OSD bind recorded as feeding that encoder. Returns false when that bind
 * has not happened yet (osd_src_port still -1), which is the normal case during
 * rvd's OSD setup and the reason attach is deferred. A cascade sub's recorded
 * port is a shadow framesource with no live SCL port; it too returns false, and
 * the sub inherits the main's overlay through the cascade (see the file header).
 */
static bool i6c_osd_scl_port(infinity6c_state_t *st, int grp, i6c_sys_bind *port)
{
    int src;

    if (!st || grp < 0 || grp >= I6C_MAX_CHN)
        return false;
    src = st->osd_src_port[grp];
    if (src < 0 || src >= I6C_MAX_CHN)
        return false;

    memset(port, 0, sizeof(*port));
    port->module = I6C_SYS_MOD_SCL;
    port->device = I6C_SCL_DEV;
    port->channel = I6C_SCL_CHN;
    port->port = (unsigned int)src;

    return true;
}

/*
 * Whether an SCL output port scales, which is what decides if it can carry an
 * overlay at all.
 *
 * A port whose output matches the ISP frame it is fed does no scaling, and the
 * driver runs it as a pass-through with no output buffer of its own. MI_RGN has
 * nothing to blend into there: the attach still *succeeds*, and then the port
 * stops draining, VIF parks in MI_SYS_InferGraph_EnsureInputPortFifoEmpty and
 * every stream on the device stalls -- not just the one that asked for the
 * overlay. Since the failure is silent and total, the port has to be checked
 * here rather than trusted to refuse.
 *
 * Any real scale factor is enough; a stream 16 pixels short of the sensor in one
 * axis overlays correctly. Callers that want an overlay on a full-sensor stream
 * have to give up the last few pixels to get one.
 */
static bool i6c_osd_port_scales(infinity6c_state_t *st, unsigned int port)
{
    const infinity6c_fs_chn_t *fs;

    if (port >= I6C_MAX_CHN)
        return false;

    fs = &st->fs[port];
    if (!fs->configured)
        return false;

    return fs->width != st->plane.capt.width || fs->height != st->plane.capt.height;
}

/* Fill the per-channel display attr MI wants from a tracked region. */
static void i6c_osd_fill_chn(const infinity6c_osd_region_t *r, i6c_rgn_chn *chn)
{
    memset(chn, 0, sizeof(*chn));
    chn->show = r->show ? 1 : 0;
    chn->point.x = (unsigned int)r->x;
    chn->point.y = (unsigned int)r->y;

    if (r->type == RSS_OSD_COVER) {
        chn->cover.layer = (unsigned int)r->layer;
        chn->cover.size.width = (unsigned int)r->width;
        chn->cover.size.height = (unsigned int)r->height;
        chn->cover.color = r->cover_color;
        return;
    }

    chn->osd.layer = (unsigned int)r->layer;
    /*
     * PIXEL_ALPHA, not CONSTANT_ALPHA: the bitmap's own alpha channel is what
     * blends antialiased glyph edges, and constant alpha replaces it with one
     * value for the whole rectangle -- which, since rvd sends bg_alpha 0 for text
     * regions, would paint every overlay fully transparent. This is the same trap
     * i6e's hal_osd.c documents at length; MI 3.0 just names it an alpha mode
     * rather than a bool. bgFgAlpha is {background, foreground}, applied per pixel
     * according to the bitmap's alpha. A caller wanting a uniform-alpha rectangle
     * wants a COVER region, which is the branch above.
     */
    chn->osd.alpha.alphaMode = I6C_RGN_ALPHA_PIXEL;
    chn->osd.alpha.alphaPara.bgFgAlpha.bgAlpha = r->bg_alpha;
    chn->osd.alpha.alphaPara.bgFgAlpha.fgAlpha = r->fg_alpha;
}

/*
 * Ask the driver which pixel format it will accept, once.
 *
 * Same rationale as i6e: the accepted set is decided in mi_rgn.ko and cannot be
 * known statically, so probe by creating and destroying a 2x2 region,
 * cheapest-conversion-first (ARGB8888 is a straight copy of rvd's BGRA, then
 * ARGB4444, then ARGB1555). RSS_OSD_PIXFMT narrows the list for bring-up.
 */
static int i6c_osd_probe_pixfmt(infinity6c_state_t *st)
{
    static const i6c_rgn_pixfmt tries[] = {I6C_RGN_PIXFMT_ARGB8888, I6C_RGN_PIXFMT_ARGB4444,
                                           I6C_RGN_PIXFMT_ARGB1555};
    const unsigned int probe_handle = I6C_OSD_REGION_MAX;
    const char *want = getenv("RSS_OSD_PIXFMT");
    unsigned int i;

    if (st->rgn_fmt_known)
        return RSS_OK;

    for (i = 0; i < sizeof(tries) / sizeof(tries[0]); i++) {
        i6c_rgn_cnf cnf;
        int ret;

        if (want && want[0] && strcmp(want, i6c_osd_fmt_name(tries[i])) != 0)
            continue;

        memset(&cnf, 0, sizeof(cnf));
        cnf.type = I6C_RGN_TYPE_OSD;
        cnf.pixFmt = tries[i];
        cnf.size.width = 2;
        cnf.size.height = 2;

        ret = st->rgn.fnCreateRegion(I6C_SOC_ID, probe_handle, &cnf);
        if (ret) {
            /* INFO not DBG: a rejected probe makes mi_rgn.ko print its own
             * "Check osd attr error" to the kernel log, and this is the line that
             * explains it. Once per rejected format per boot. */
            HAL_LOG_INFO("osd: %s rejected by MI_RGN_Create: %#x "
                         "(the kernel's \"Check osd attr error\" is this probe)",
                         i6c_osd_fmt_name(tries[i]), (unsigned int)ret);
            continue;
        }

        st->rgn.fnDestroyRegion(I6C_SOC_ID, probe_handle);
        st->rgn_fmt = tries[i];
        st->rgn_fmt_known = true;
        HAL_LOG_INFO("osd: using %s%s%s", i6c_osd_fmt_name(tries[i]),
                     tries[i] == I6C_RGN_PIXFMT_ARGB8888 ? " (no conversion needed)" : "",
                     want && want[0] ? " (RSS_OSD_PIXFMT)" : "");
        return RSS_OK;
    }

    if (want && want[0])
        HAL_LOG_ERR("osd: RSS_OSD_PIXFMT=%s matched no probeable format, or MI_RGN_Create "
                    "rejected it (try ARGB8888, ARGB4444 or ARGB1555)",
                    want);
    else
        HAL_LOG_ERR("osd: MI_RGN_Create rejected every pixel format tried "
                    "(ARGB8888, ARGB4444, ARGB1555)");
    return RSS_ERR_NOTSUP;
}

/* Bring MI_RGN up on first use. */
static int i6c_osd_ensure_init(infinity6c_state_t *st)
{
    int ret;

    if (!st->rgn_loaded) {
        ret = i6c_rgn_load(&st->rgn);
        if (ret) {
            /* i6c_rgn_load logs which symbol or library was missing. */
            return ret;
        }
        st->rgn_loaded = true;
    }

    if (!st->rgn_inited) {
        /* The palette only matters for the I2/I4/I8 formats, which this backend
         * never selects. MI still wants the argument, so pass a zeroed one. */
        i6c_rgn_pal pal;

        memset(&pal, 0, sizeof(pal));
        ret = st->rgn.fnInit(I6C_SOC_ID, &pal);
        if (ret) {
            HAL_LOG_ERR("MI_RGN_Init failed: %#x", (unsigned int)ret);
            return RSS_ERR_IO;
        }
        st->rgn_inited = true;
    }

    return RSS_OK;
}

/* Attach one region to its group's VENC channel, if that channel exists yet. */
static int i6c_osd_try_attach(infinity6c_state_t *st, int handle, infinity6c_osd_region_t *r)
{
    i6c_sys_bind port;
    i6c_rgn_chn chn;
    int ret;

    if (r->attached || r->grp < 0)
        return RSS_OK;
    if (!i6c_osd_scl_port(st, r->grp, &port))
        return RSS_OK; /* Deferred, not failed. */

    if (!i6c_osd_port_scales(st, port.port)) {
        if (!st->osd_noscale_warned) {
            HAL_LOG_WARN("osd: stream on SCL port %u runs at the full sensor frame (%ux%u) and "
                         "does not scale, so it cannot carry an overlay -- ask for any smaller "
                         "size to get one",
                         port.port, st->plane.capt.width, st->plane.capt.height);
            st->osd_noscale_warned = true;
        }
        return RSS_OK;
    }

    i6c_osd_fill_chn(r, &chn);

    ret = st->rgn.fnAttachChannel(I6C_SOC_ID, (unsigned int)handle, &port, &chn);
    if (ret) {
        HAL_LOG_WARN("MI_RGN_AttachToChn(region %d, SCL port %u) failed: %#x", handle, port.port,
                     (unsigned int)ret);
        return RSS_ERR_IO;
    }

    r->attached = true;
    HAL_LOG_DBG("osd: region %d attached to SCL port %u (group %d), layer %d, alpha bg/fg %u/%u",
                handle, port.port, r->grp, r->layer, r->bg_alpha, r->fg_alpha);

    return RSS_OK;
}

static void i6c_osd_detach(infinity6c_state_t *st, int handle, infinity6c_osd_region_t *r)
{
    i6c_sys_bind port;

    if (!r->attached)
        return;

    if (i6c_osd_scl_port(st, r->grp, &port))
        st->rgn.fnDetachChannel(I6C_SOC_ID, (unsigned int)handle, &port);

    r->attached = false;
}

/* Push the tracked display attr to MI for an attached region. */
static int i6c_osd_push_chn(infinity6c_state_t *st, int handle, infinity6c_osd_region_t *r)
{
    i6c_sys_bind port;
    i6c_rgn_chn chn;
    int ret;

    if (!r->attached)
        return RSS_OK;
    if (!i6c_osd_scl_port(st, r->grp, &port))
        return RSS_OK;

    i6c_osd_fill_chn(r, &chn);

    ret = st->rgn.fnSetChannelConfig(I6C_SOC_ID, (unsigned int)handle, &port, &chn);
    if (ret) {
        HAL_LOG_WARN("MI_RGN_SetDisplayAttr(region %d) failed: %#x", handle, (unsigned int)ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* Create the MI region behind a tracked slot. */
static int i6c_osd_create_mi(infinity6c_state_t *st, int handle, infinity6c_osd_region_t *r)
{
    i6c_rgn_cnf cnf;
    int ret;

    memset(&cnf, 0, sizeof(cnf));
    cnf.type = r->type == RSS_OSD_COVER ? I6C_RGN_TYPE_COVER : I6C_RGN_TYPE_OSD;
    cnf.pixFmt = st->rgn_fmt;
    cnf.size.width = (unsigned int)r->width;
    cnf.size.height = (unsigned int)r->height;

    ret = st->rgn.fnCreateRegion(I6C_SOC_ID, (unsigned int)handle, &cnf);
    if (ret) {
        HAL_LOG_ERR("MI_RGN_Create(region %d, %dx%d, %s) failed: %#x", handle, r->width, r->height,
                    i6c_osd_fmt_name(st->rgn_fmt), (unsigned int)ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * Attach every region whose group is this VENC channel. Called from
 * i6c_bind_scl_to_venc with the SCL port enabled and not yet bound. Failures are
 * logged and not propagated: a region that will not attach costs an overlay, and
 * taking the stream down over it would be worse than the missing text.
 */
void i6c_osd_flush_pending(infinity6c_state_t *st, int chn)
{
    int i;

    if (!st || !st->rgn_inited)
        return;

    for (i = 0; i < I6C_OSD_REGION_MAX; i++) {
        if (st->osd[i].used && st->osd[i].grp == chn)
            i6c_osd_try_attach(st, i, &st->osd[i]);
    }
}

/*
 * Release everything OSD-side. Called at the head of i6c_teardown_all, before
 * the VENC channels are destroyed. Detach, destroy, then deinit -- MI_RGN_DeInit
 * with regions still attached leaves the driver holding references.
 */
void i6c_osd_release_all(infinity6c_state_t *st)
{
    int i;

    if (!st || !st->rgn_loaded)
        return;

    for (i = 0; i < I6C_OSD_REGION_MAX; i++) {
        infinity6c_osd_region_t *r = &st->osd[i];

        if (!r->used)
            continue;

        i6c_osd_detach(st, i, r);
        st->rgn.fnDestroyRegion(I6C_SOC_ID, (unsigned int)i);
        free(r->bmp);
        memset(r, 0, sizeof(*r));
    }

    if (st->rgn_inited) {
        st->rgn.fnDeinit(I6C_SOC_ID);
        st->rgn_inited = false;
    }

    i6c_rgn_unload(&st->rgn);
    st->rgn_loaded = false;
    st->rgn_fmt_known = false;
}

/* ---- ops ------------------------------------------------------------- */

/*
 * MI allocates a region's memory at create and exposes no size control, so there
 * is nothing to set. Reported once and accepted rather than refused: rvd asks
 * because Ingenic needs a pool, and NOTSUP would make its osd-restart path log a
 * failure for a step that was never necessary here.
 */
int hal_osd_set_pool_size(void *ctx, uint32_t bytes)
{
    infinity6c_state_t *st = osd_state(ctx);

    (void)bytes;

    if (!st)
        return RSS_ERR_INVAL;

    HAL_LOG_DBG("osd: pool size %u ignored -- MI allocates per region", bytes);

    return RSS_OK;
}

int hal_osd_create_group(void *ctx, int grp)
{
    infinity6c_state_t *st = osd_state(ctx);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (grp < 0 || grp >= I6C_MAX_CHN) {
        HAL_LOG_ERR("osd: group %d out of range (0..%d)", grp, I6C_MAX_CHN - 1);
        return RSS_ERR_INVAL;
    }

    ret = i6c_osd_ensure_init(st);
    if (ret)
        return ret;

    st->osd_grp[grp] = true;

    return RSS_OK;
}

int hal_osd_destroy_group(void *ctx, int grp)
{
    infinity6c_state_t *st = osd_state(ctx);
    int i;

    if (!st)
        return RSS_ERR_INVAL;
    if (grp < 0 || grp >= I6C_MAX_CHN)
        return RSS_ERR_INVAL;

    /* Regions are global MI objects that outlive the group, so destroying a group
     * detaches its members and leaves them created -- what rvd expects when it
     * tears one stream down and leaves another running. */
    for (i = 0; i < I6C_OSD_REGION_MAX; i++) {
        if (st->osd[i].used && st->osd[i].grp == grp)
            i6c_osd_detach(st, i, &st->osd[i]);
    }

    st->osd_grp[grp] = false;

    return RSS_OK;
}

/*
 * No MI equivalent: there is no group object to start or stop, and whether an
 * overlay is composited is the per-region `show` flag osd_show_region owns.
 * Accepted as a no-op rather than refused, because rvd calls both around every
 * stream.
 */
int hal_osd_start(void *ctx, int grp)
{
    (void)grp;

    return osd_state(ctx) ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_stop(void *ctx, int grp)
{
    (void)grp;

    return osd_state(ctx) ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr)
{
    infinity6c_state_t *st = osd_state(ctx);
    infinity6c_osd_region_t *r;
    int slot;
    int ret;

    if (!st || !handle || !attr)
        return RSS_ERR_INVAL;
    if (attr->width <= 0 || attr->height <= 0) {
        HAL_LOG_ERR("osd: region geometry %dx%d is empty", attr->width, attr->height);
        return RSS_ERR_INVAL;
    }

    ret = i6c_osd_ensure_init(st);
    if (ret)
        return ret;

    ret = i6c_osd_probe_pixfmt(st);
    if (ret)
        return ret;

    for (slot = 0; slot < I6C_OSD_REGION_MAX; slot++) {
        if (!st->osd[slot].used)
            break;
    }
    if (slot == I6C_OSD_REGION_MAX) {
        HAL_LOG_ERR("osd: all %d region slots in use", I6C_OSD_REGION_MAX);
        return RSS_ERR_NOMEM;
    }

    r = &st->osd[slot];
    memset(r, 0, sizeof(*r));
    r->type = attr->type;
    r->x = attr->x;
    r->y = attr->y;
    r->width = attr->width;
    r->height = attr->height;
    r->layer = attr->layer;
    r->global_alpha_en = attr->global_alpha_en;
    r->fg_alpha = attr->fg_alpha;
    r->bg_alpha = attr->bg_alpha;
    r->cover_color = attr->cover_color;
    r->grp = -1;
    r->show = false;

    ret = i6c_osd_create_mi(st, slot, r);
    if (ret) {
        memset(r, 0, sizeof(*r));
        return ret;
    }

    r->used = true;
    *handle = slot;

    HAL_LOG_DBG("osd: region %d created, %dx%d at %d,%d layer %d", slot, r->width, r->height, r->x,
                r->y, r->layer);

    return RSS_OK;
}

int hal_osd_destroy_region(void *ctx, int handle)
{
    infinity6c_state_t *st = osd_state(ctx);
    infinity6c_osd_region_t *r = i6c_osd_region(st, handle);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;

    i6c_osd_detach(st, handle, r);

    ret = st->rgn.fnDestroyRegion(I6C_SOC_ID, (unsigned int)handle);
    if (ret)
        HAL_LOG_WARN("MI_RGN_Destroy(region %d) failed: %#x", handle, (unsigned int)ret);

    free(r->bmp);
    memset(r, 0, sizeof(*r));

    return ret ? RSS_ERR_IO : RSS_OK;
}

int hal_osd_register_region(void *ctx, int handle, int grp)
{
    infinity6c_state_t *st = osd_state(ctx);
    infinity6c_osd_region_t *r = i6c_osd_region(st, handle);

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (grp < 0 || grp >= I6C_MAX_CHN)
        return RSS_ERR_INVAL;

    if (r->grp >= 0 && r->grp != grp) {
        /* One region, one channel. Silently leaving it on the old one would be
         * worse than saying so. */
        HAL_LOG_WARN("osd: region %d moving from group %d to %d", handle, r->grp, grp);
        i6c_osd_detach(st, handle, r);
    }

    r->grp = grp;

    /* Succeeds now if the channel already exists (a runtime region), defers to
     * i6c_osd_flush_pending if this is startup. */
    return i6c_osd_try_attach(st, handle, r);
}

int hal_osd_unregister_region(void *ctx, int handle, int grp)
{
    infinity6c_state_t *st = osd_state(ctx);
    infinity6c_osd_region_t *r = i6c_osd_region(st, handle);

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (grp >= 0 && r->grp != grp)
        return RSS_ERR_INVAL;

    i6c_osd_detach(st, handle, r);
    r->grp = -1;

    return RSS_OK;
}

/*
 * Geometry, position, alpha and layer in one call.
 *
 * A size change cannot be applied in place -- MI fixes a region's dimensions at
 * create -- so it becomes destroy + create + re-attach. Everything else is a
 * display-attr push.
 */
int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr)
{
    infinity6c_state_t *st = osd_state(ctx);
    infinity6c_osd_region_t *r = i6c_osd_region(st, handle);
    bool resized;
    int grp;
    int ret;

    if (!st || !attr)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (attr->width <= 0 || attr->height <= 0)
        return RSS_ERR_INVAL;

    resized = attr->width != r->width || attr->height != r->height || attr->type != r->type;

    r->x = attr->x;
    r->y = attr->y;
    r->layer = attr->layer;
    r->global_alpha_en = attr->global_alpha_en;
    r->fg_alpha = attr->fg_alpha;
    r->bg_alpha = attr->bg_alpha;
    r->cover_color = attr->cover_color;

    if (!resized)
        return i6c_osd_push_chn(st, handle, r);

    grp = r->grp;
    i6c_osd_detach(st, handle, r);
    st->rgn.fnDestroyRegion(I6C_SOC_ID, (unsigned int)handle);

    r->type = attr->type;
    r->width = attr->width;
    r->height = attr->height;

    /* The old bitmap is the wrong size now; the next update reallocates. */
    free(r->bmp);
    r->bmp = NULL;
    r->bmp_size = 0;
    r->bmp_logged = false;

    ret = i6c_osd_create_mi(st, handle, r);
    if (ret) {
        free(r->bmp);
        memset(r, 0, sizeof(*r));
        return ret;
    }

    r->grp = grp;

    return i6c_osd_try_attach(st, handle, r);
}

/*
 * BGRA8888 from rvd -> whatever MI accepted.
 *
 * rvd's "BGRA, width*height*4" is byte order in memory, so as a little-endian
 * 32-bit word each pixel is already 0xAARRGGBB -- MI's ARGB8888. That is why the
 * 8888 path is a copy and 8888 is probed first.
 */
static void i6c_osd_convert(const uint8_t *src, void *dst, int pixels, i6c_rgn_pixfmt fmt)
{
    int i;

    if (fmt == I6C_RGN_PIXFMT_ARGB8888) {
        memcpy(dst, src, (size_t)pixels * 4);
        return;
    }

    if (fmt == I6C_RGN_PIXFMT_ARGB4444) {
        uint16_t *out = (uint16_t *)dst;

        for (i = 0; i < pixels; i++) {
            uint8_t b = src[i * 4 + 0];
            uint8_t g = src[i * 4 + 1];
            uint8_t rr = src[i * 4 + 2];
            uint8_t a = src[i * 4 + 3];

            out[i] = (uint16_t)(((a >> 4) << 12) | ((rr >> 4) << 8) | ((g >> 4) << 4) | (b >> 4));
        }
        return;
    }

    /* ARGB1555: alpha collapses to one bit. Anything not fully transparent is
     * drawn, which keeps thin antialiased strokes visible instead of dropping
     * them. */
    {
        uint16_t *out = (uint16_t *)dst;

        for (i = 0; i < pixels; i++) {
            uint8_t b = src[i * 4 + 0];
            uint8_t g = src[i * 4 + 1];
            uint8_t rr = src[i * 4 + 2];
            uint8_t a = src[i * 4 + 3];

            out[i] = (uint16_t)((a ? 0x8000 : 0) | ((rr >> 3) << 10) | ((g >> 3) << 5) | (b >> 3));
        }
    }
}

int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data)
{
    infinity6c_state_t *st = osd_state(ctx);
    infinity6c_osd_region_t *r = i6c_osd_region(st, handle);
    i6c_rgn_bmp bmp;
    size_t need;
    int pixels;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (!data) {
        /* rvd's Ingenic sequence sets the attr with a NULL data pointer before it
         * has anything to draw. Nothing to push. */
        return RSS_OK;
    }
    if (r->type == RSS_OSD_COVER) {
        /* A cover is a solid rectangle; its colour is in the display attr and MI
         * has no bitmap for it. */
        return RSS_ERR_NOTSUP;
    }

    pixels = r->width * r->height;
    need = (size_t)pixels * i6c_osd_bpp(st->rgn_fmt);

    if (r->bmp_size < need) {
        void *nb = realloc(r->bmp, need);

        if (!nb)
            return RSS_ERR_NOMEM;
        r->bmp = nb;
        r->bmp_size = need;
    }

    i6c_osd_convert(data, r->bmp, pixels, st->rgn_fmt);

    memset(&bmp, 0, sizeof(bmp));
    bmp.pixFmt = st->rgn_fmt;
    bmp.size.width = (unsigned int)r->width;
    bmp.size.height = (unsigned int)r->height;
    bmp.data = r->bmp;

    ret = st->rgn.fnSetBitmap(I6C_SOC_ID, (unsigned int)handle, &bmp);
    if (ret) {
        HAL_LOG_WARN("MI_RGN_SetBitMap(region %d) failed: %#x", handle, (unsigned int)ret);
        return RSS_ERR_IO;
    }

    if (!r->bmp_logged) {
        r->bmp_logged = true;
        HAL_LOG_DBG("osd: region %d first bitmap accepted, %dx%d %s", handle, r->width, r->height,
                    i6c_osd_fmt_name(st->rgn_fmt));
    }

    return RSS_OK;
}

/*
 * Show/hide plus z-order. rvd passes the layer here as well as in the region
 * attr, and this is the call it makes per frame when visibility changes, so both
 * are recorded and pushed together.
 */
int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer)
{
    infinity6c_state_t *st = osd_state(ctx);
    infinity6c_osd_region_t *r = i6c_osd_region(st, handle);

    if (!st)
        return RSS_ERR_INVAL;
    if (!r)
        return RSS_ERR_NOENT;
    if (grp >= 0 && r->grp >= 0 && r->grp != grp)
        return RSS_ERR_INVAL;

    r->show = show ? true : false;
    if (layer >= 0)
        r->layer = layer;

    return i6c_osd_push_chn(st, handle, r);
}
