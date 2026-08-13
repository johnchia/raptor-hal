/*
 * infinity6c/hal_osd.c -- OSD/overlay ops for Infinity6C, over MI_RGN (MI 3.0).
 *
 * Counterpart to star/hal_osd.c; that file's header explains the model this one
 * shares, so only the two things MI 3.0 does differently are written out here.
 *
 * WHERE THE OVERLAY ATTACHES
 *
 * Two places, because this backend's two kinds of video channel are fed
 * differently, and each has to be overlaid where its own frame is assembled.
 *
 * A channel fed from the scaler -- the main H.26x stream, and JPEG -- is overlaid
 * on its SCL output port (I6C_SYS_MOD_SCL), the port recorded in osd_src_port[]
 * as rvd's FS -> OSD bind names it, which is the whole reason that array exists.
 * That is what the working i6c reference does: waybeam attaches RGN to the SCL
 * module. It is not a free choice for the main stream, whose SCL port is bound to
 * the encoder by a hardware ring; the overlay has to be applied on the producing
 * side of that ring.
 *
 * A cascaded channel -- a second H.26x stream, which takes no SCL port of its own
 * and is instead ringed off the main encoder (see i6c_bind_scl_to_venc) -- is
 * overlaid on its own VENC channel (I6C_SYS_MOD_VENC, port 0, the encoder's input
 * port). The cascade taps the main's frame upstream of the SCL overlay, so a
 * region on the main's port never reaches the sub; the sub's own encoder input is
 * the first place its frame exists separately. The vendor RGN documentation lists
 * VENC0 as an overlay path with its own GOP hardware, independent of the GOP that
 * SCL and DISP share, so a main-on-SCL and a sub-on-VENC overlay coexist.
 *
 * The earlier reading of this backend, that MI_RGN simply refuses a ring-fed VENC
 * channel, came from an attach rejected with RGN ILLEGAL_PARAM while the region
 * was ARGB8888 -- a format no path on this part accepts (see
 * i6c_osd_probe_pixfmt). It was the format being refused, not the module.
 *
 * COVER regions are SCL-only: the same table gives VENC0 a cover layer count of
 * NA. A cover asked for on a cascaded channel is declined rather than attached.
 *
 * WHY ATTACH IS DEFERRED
 *
 * rvd creates and registers every region before the port or channel that carries
 * a stream is live. register_region records the intent; i6c_osd_flush_pending
 * performs the attach from i6c_bind_scl_to_venc, once there is something to
 * attach to.
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
 * The port a region attaches to for a given group, and whether that port is an
 * encoder input rather than a scaler output.
 *
 * Groups are encoder channels. A cascaded channel is overlaid on its own VENC
 * input port; every other channel is overlaid on the SCL output port rvd's
 * FS -> OSD bind recorded as feeding it. Returns false while neither is available
 * yet, which is the normal case during rvd's OSD setup and the reason attach is
 * deferred: for the scaler case that means osd_src_port still -1, and for a
 * cascade that the channel has not been bound and so does not know it is one.
 */
static bool i6c_osd_target_port(infinity6c_state_t *st, int grp, i6c_sys_bind *port, bool *on_venc)
{
    int src;

    if (!st || grp < 0 || grp >= I6C_MAX_CHN)
        return false;

    memset(port, 0, sizeof(*port));

    if (st->enc[grp].created && st->enc[grp].cascade) {
        port->module = I6C_SYS_MOD_VENC;
        port->device = st->enc[grp].device;
        port->channel = (unsigned int)grp;
        port->port = 0;
        *on_venc = true;
        return true;
    }

    src = st->osd_src_port[grp];
    if (src < 0 || src >= I6C_MAX_CHN)
        return false;

    port->module = I6C_SYS_MOD_SCL;
    port->device = I6C_SCL_DEV;
    port->channel = I6C_SCL_CHN;
    port->port = (unsigned int)src;
    *on_venc = false;

    return true;
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
 * cheapest-conversion-first. RSS_OSD_PIXFMT narrows the list for bring-up.
 *
 * ARGB8888 is not a candidate on this part, even though it is the one format that
 * needs no conversion from rvd's BGRA. The RGN chapter of the vendor
 * documentation gives a per-chip table of formats each overlay path accepts, and
 * for this one (Maruko) every path -- SCL0-3, VENC0, JPE0, DISP0 -- lists
 * ARGB1555, ARGB4444, I2, I4 and I8 as supported and ARGB8888 and RGB565 as not.
 * The probe cannot discover that: MI_RGN_Create *accepts* an ARGB8888 region, and
 * the format only fails later, in the blend. It fails by hanging rather than by
 * complaining -- the attached port stops draining, VIF parks in
 * MI_SYS_InferGraph_EnsureInputPortFifoEmpty, and every stream on the device
 * stalls -- and it does so only at some frame sizes, which makes an unsupported
 * format look like a resolution limit.
 */
static int i6c_osd_probe_pixfmt(infinity6c_state_t *st)
{
    static const i6c_rgn_pixfmt tries[] = {I6C_RGN_PIXFMT_ARGB4444, I6C_RGN_PIXFMT_ARGB1555};
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

/* Attach one region to its group's overlay port, if that port exists yet. */
static int i6c_osd_try_attach(infinity6c_state_t *st, int handle, infinity6c_osd_region_t *r)
{
    i6c_sys_bind port;
    i6c_rgn_chn chn;
    bool on_venc = false;
    int ret;

    if (r->attached || r->grp < 0)
        return RSS_OK;
    if (!i6c_osd_target_port(st, r->grp, &port, &on_venc))
        return RSS_OK; /* Deferred, not failed. */

    /* An encoder input port carries OSD layers but no cover layers, so a cover
     * asked for on a cascaded stream has nowhere to go. Said once per region, at
     * INFO: it is a property of the part, not a fault to be fixed. */
    if (on_venc && r->type == RSS_OSD_COVER) {
        if (!r->cover_declined) {
            HAL_LOG_INFO("osd: region %d is a cover on cascaded stream %d, which has no cover "
                         "layer -- only the scaler-fed streams can carry one",
                         handle, r->grp);
            r->cover_declined = true;
        }
        return RSS_OK;
    }

    i6c_osd_fill_chn(r, &chn);

    ret = st->rgn.fnAttachChannel(I6C_SOC_ID, (unsigned int)handle, &port, &chn);
    if (ret) {
        HAL_LOG_WARN("MI_RGN_AttachToChn(region %d, %s %u) failed: %#x", handle,
                     on_venc ? "VENC chn" : "SCL port", on_venc ? port.channel : port.port,
                     (unsigned int)ret);
        return RSS_ERR_IO;
    }

    r->attached = true;
    HAL_LOG_DBG("osd: region %d attached to %s %u (group %d), layer %d, alpha bg/fg %u/%u", handle,
                on_venc ? "VENC chn" : "SCL port", on_venc ? port.channel : port.port, r->grp,
                r->layer, r->bg_alpha, r->fg_alpha);

    return RSS_OK;
}

static void i6c_osd_detach(infinity6c_state_t *st, int handle, infinity6c_osd_region_t *r)
{
    i6c_sys_bind port;
    bool on_venc = false;

    if (!r->attached)
        return;

    if (i6c_osd_target_port(st, r->grp, &port, &on_venc))
        st->rgn.fnDetachChannel(I6C_SOC_ID, (unsigned int)handle, &port);

    r->attached = false;
}

/* Push the tracked display attr to MI for an attached region. */
static int i6c_osd_push_chn(infinity6c_state_t *st, int handle, infinity6c_osd_region_t *r)
{
    i6c_sys_bind port;
    i6c_rgn_chn chn;
    bool on_venc = false;
    int ret;

    if (!r->attached)
        return RSS_OK;
    if (!i6c_osd_target_port(st, r->grp, &port, &on_venc))
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
 * 32-bit word each pixel is already 0xAARRGGBB -- MI's ARGB8888, which is why
 * that case is a plain copy. No overlay path on this part accepts ARGB8888
 * though (see i6c_osd_probe_pixfmt), so the formats actually reached here are the
 * 16-bit two.
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
