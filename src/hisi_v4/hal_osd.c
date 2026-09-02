/*
 * hal_osd.c -- OSD/overlay ops for HiSilicon gen4, over HI_MPI_RGN.
 *
 * THE POINT OF THIS FILE
 *
 * rvd renders text into BGRA bitmaps and this file hands them to HiMPP as
 * RGN regions attached to the VENC channel carrying each stream. Attaching
 * to VENC rather than to VPSS is not a preference: gen4's per-module region
 * budget (hi_defines.h, quoted in v4_rgn.h) is OVERLAY_MAX_NUM_VPSS 0 and
 * OVERLAY_MAX_NUM_VENC 8, so the encoder is the only stage that composites
 * an overlay at all. It is also the stage that makes the overlay
 * *per-stream*, which is what the FS -> OSD -> ENC collapse in
 * hisi_bind_collapse gets away with: raptor's OSD "group" is an encoder
 * channel here, and the main stream's timestamp and the sub-stream's are
 * two regions on two channels at two positions.
 *
 * THE DIVINUS DEFECT THIS FILE EXISTS TO NOT REPEAT
 *
 * divinus builds its attach descriptor as
 *
 *     v4_sys_bind dest = { .module = V4_SYS_MOD_VENC, .device = _v4_venc_dev };
 *
 * (ref/divinus/src/hal/hisi/v4_hal.c:398) and never sets `.channel`. Every
 * region it creates therefore lands on VENC channel 0, whatever stream it
 * was meant for -- and the failure is invisible on a one-stream camera,
 * which is how it survived. So the channel is carried in the region record
 * (hisi_osd_region_t.grp) and hisi_osd_mpp_chn is the single place the
 * descriptor is built. t_hisi_osd asserts on the descriptor's channel
 * field for exactly this reason.
 *
 * WHY ATTACH IS DEFERRED
 *
 * A region can only attach to a VENC channel that exists: HI_MPI_RGN_
 * AttachToChn on an uncreated channel fails, and there is no "attach later"
 * in the MPI. rvd's order does not guarantee one -- it sets region
 * attributes before the bind exists on every platform, because its call
 * order is built around Ingenic's object lifetimes -- so registering a
 * region records the *intent* and the attach happens at the first moment it
 * can succeed. Three places drive it:
 *
 *   hal_osd_register_region   attach now if the channel is already there
 *                             (the runtime case: a region added to a
 *                             running stream)
 *   hisi_osd_flush_pending    from hal_bind, and from hal_enc_create_channel
 *   hisi_osd_detach_chn       from hal_enc_destroy_channel
 *
 * The last pair is what makes a region survive an encoder restart. HiMPP
 * refuses to destroy a VENC channel that still has regions attached
 * (HI_ERR_RGN_BUSY is documented as exactly that: "destroy a venc chn
 * without unregistering it"), so the detach is mandatory rather than
 * tidy -- but it clears `attached` and leaves `grp` alone, so when rvd
 * recreates the channel the flush re-attaches every region that belonged
 * to it, with its position, alpha and layer intact. This is the same
 * registered-versus-attached split src/star/hal_osd.c uses, moved from
 * "the VPE port is not known yet" to "the VENC channel does not exist yet".
 *
 * PIXEL FORMAT
 *
 * ARGB1555, with no probe. src/star/hal_osd.c probes because MI's accepted
 * set is decided in a kernel module and written down nowhere; HiMPP writes
 * it down at the field ("now only support ARGB1555 or ARGB4444") and the
 * vendor's own sample uses ARGB1555. rvd renders BGRA8888, so the alpha
 * channel collapses to one bit: anything not fully transparent is drawn,
 * which keeps thin antialiased strokes visible instead of dropping them.
 * The opposite threshold is what makes small text vanish.
 *
 * OP COVERAGE
 *
 * Twelve ops, which is what rvd calls -- rvd_osd.c and rvd_pipeline.c
 * between them issue set_pool_size, create_group, create_region,
 * register_region, set_region_attr, show_region, start, update_region_data,
 * unregister_region, destroy_region, stop and destroy_group, and nothing
 * else. Deliberately absent:
 *
 *   osd_show                   rvd never calls it; osd_show_region carries
 *                              the same information plus the layer, and one
 *                              code path is better than two that can
 *                              disagree.
 *   osd_attach_to_group        an Ingenic re-attach primitive rvd does not
 *                              use. register_region covers it, and the
 *                              deferral above covers the case it existed
 *                              for.
 *   osd_get_region_attr        unused by rvd, and misleading to implement:
 *   osd_get_group_region_attr  position, alpha and layer live in the
 *                              per-channel display attr, which does not
 *                              exist until the region is attached -- so the
 *                              getter would answer nothing during exactly
 *                              the window rvd would use it in.
 *   osd_set_region_attr_with_timestamp
 *                              Ingenic's timestamped variant; HiMPP has no
 *                              equivalent and rvd never calls it.
 *
 * And one type is absent rather than one op: RSS_OSD_COVER is refused with
 * RSS_ERR_NOTSUP. COVER_RGN has no VENC entry in the budget table at all,
 * so a privacy cover cannot be composited by the encoder on this silicon.
 * rvd's privacy path already handles the refusal -- rvd_osd.c takes the
 * create's return and leaves privacy_handles[s] at -1 -- so the cost is the
 * full-frame privacy blank, not a broken stream. Drawing it as an overlay
 * instead was considered and rejected: a 1920x1080 ARGB1555 canvas is 4 MB
 * of the RGN heap, allocated on every stream start whether privacy is ever
 * switched on or not.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <stdlib.h>
#include <string.h>

/*
 * No HAL_MODULE_VIDEO guard, deliberately, for the reason src/star/hal_osd.c
 * states: the Makefile passes that flag only to the two hal_common objects,
 * so a guard here would compile the file away and leave hal_common.c's ops
 * table with unresolved references. Membership in VIDEO_SRCS is what keeps
 * this out of the audio archive.
 */

/* EN_ERR_EXIST, the error id HI_MPI_RGN_Create returns for a handle that is
 * already a live region -- kernel state, so it survives the process that
 * made it. See hisi_osd_create_region. */
#define V4_ERR_EXIST 4u

/* ARGB1555 is two bytes per pixel, everywhere in this file. */
#define HISI_OSD_BPP 2u

static hisi_osd_region_t *hisi_osd_slot(hisi_state_t *st, int handle)
{
    if (!st || handle < 0 || handle >= HISI_OSD_REGION_MAX)
        return NULL;
    if (!st->osd[handle].used)
        return NULL;

    return &st->osd[handle];
}

/* True once RGN's entry points resolved. Every op checks it, because an
 * absent RGN is a board without OSD rather than a broken backend. */
static bool hisi_osd_ready(const hisi_state_t *st)
{
    return st && st->rgn_loaded;
}

/*
 * The attach descriptor. THE ONE PLACE .channel IS SET; see the divinus
 * note at the top of the file.
 *
 * device is 0 because gen4 has a single VENC device -- the same 0
 * hisi_bind_vpss_venc writes into its own MPP_CHN_S, and for the same
 * reason.
 */
static void hisi_osd_mpp_chn(int grp, v4_mpp_chn *chn)
{
    memset(chn, 0, sizeof(*chn));
    chn->module = V4_MOD_VENC;
    chn->device = 0;
    chn->channel = grp;
}

/* Even, at least 2, no more than the driver's ceiling. RGN_ALIGN is 2 and
 * the driver refuses an odd dimension rather than rounding it. */
static unsigned int hisi_osd_align_dim(int v)
{
    unsigned int u;

    if (v < V4_RGN_MIN_DIM)
        v = V4_RGN_MIN_DIM;
    u = (unsigned int)v;
    u = (u + 1u) & ~1u;
    if (u > V4_RGN_OVERLAY_MAX_DIM)
        u = V4_RGN_OVERLAY_MAX_DIM;

    return u;
}

/* Even, non-negative, inside RGN_OVERLAY_MAX_X/_Y. Rounded *down*, so a
 * region never moves off the picture it was positioned against. */
static int hisi_osd_align_pos(int v)
{
    if (v < 0)
        v = 0;
    v &= ~1;
    if (v > V4_RGN_OVERLAY_MAX_XY)
        v = V4_RGN_OVERLAY_MAX_XY;

    return v;
}

/* OVERLAY's layer range is [0,7] -- OVERLAYEX's [0,15] is a different
 * struct. rvd hands layer + 1 through show_region and can exceed it. */
static unsigned int hisi_osd_clamp_layer(int layer)
{
    if (layer < 0)
        return 0;
    if (layer > 7)
        return 7;

    return (unsigned int)layer;
}

/* Fill the per-channel display attr from a tracked region. */
static void hisi_osd_fill_chn_attr(const hisi_osd_region_t *r, v4_rgn_chn_attr *ca)
{
    memset(ca, 0, sizeof(*ca));

    ca->show = r->show ? 1 : 0;
    ca->type = V4_RGN_TYPE_OVERLAY;
    ca->overlay.point.x = r->x;
    ca->overlay.point.y = r->y;
    ca->overlay.layer = hisi_osd_clamp_layer(r->layer);

    /*
     * With ARGB1555 the bitmap's one alpha bit selects between these two
     * rather than blending them: fg_alpha applies where the bit is 1,
     * bg_alpha where it is 0. rvd sends 255/0 for text, which becomes
     * 128/0 -- opaque glyphs on a fully transparent field.
     *
     * global_alpha_en is not mapped onto anything. There is no per-region
     * constant-alpha mode in OVERLAY_CHN_ATTR_S to map it onto, and the
     * fg/bg pair already *is* the "modulate the per-pixel alpha" behaviour
     * the flag asks for on Ingenic. Tracked in the record so a getter
     * would not lie, and otherwise unused.
     */
    ca->overlay.fg_alpha = v4_rgn_alpha(r->fg_alpha);
    ca->overlay.bg_alpha = v4_rgn_alpha(r->bg_alpha);

    /*
     * stQpInfo and stInvertColor stay zeroed, and that is the useful
     * setting rather than the lazy one: zeroed QP info is bQpDisable false
     * with a relative delta of 0, i.e. the encoder's own QP, and zeroed
     * invert-colour is bInvColEn false. enAttachDest 0 is ATTACH_JPEG_MAIN,
     * which is what a region on an MJPEG channel wants and is ignored on
     * every other codec.
     */
}

/* Fill the region attr from a tracked region. */
static void hisi_osd_fill_attr(const hisi_osd_region_t *r, v4_rgn_attr *a)
{
    memset(a, 0, sizeof(*a));

    a->type = V4_RGN_TYPE_OVERLAY;
    a->overlay.pixel_format = V4_RGN_PIXFMT_ARGB1555;
    /* Transparent black in ARGB1555: the alpha bit clear. Every pixel the
     * bitmap does not cover is background, and a text overlay wants those
     * invisible. */
    a->overlay.bg_color = 0;
    a->overlay.size.width = r->width;
    a->overlay.size.height = r->height;
    a->overlay.canvas_num = 2;
}

/*
 * Create the RGN object behind a slot, idempotently.
 *
 * "Idempotently" here means across processes, not just across calls. RGN
 * handles are kernel state: a daemon killed mid-stream leaves its regions
 * live, and the next start's Create returns EN_ERR_EXIST for a handle it
 * has never used. hisi_reclaim_pipeline sweeps the VENC and VPSS side of
 * that for the same reason and cannot sweep this one, because RGN is not
 * loaded at the point it runs.
 *
 * So: ask first if there is a Get to ask with, destroy what is there, and
 * create. The fallback when GetAttr never resolved is to try the create and
 * destroy-then-retry on EN_ERR_EXIST, which reaches the same state one call
 * later.
 */
static int hisi_osd_create_rgn(hisi_state_t *st, int handle, hisi_osd_region_t *r)
{
    v4_rgn_attr attr;
    int ret;

    hisi_osd_fill_attr(r, &attr);

    if (st->rgn.fnGetAttr) {
        v4_rgn_attr cur;

        memset(&cur, 0, sizeof(cur));
        if (st->rgn.fnGetAttr((unsigned int)handle, &cur) == 0) {
            HAL_LOG_INFO("osd: region %d already exists (%ux%u); recreating it", handle,
                         cur.overlay.size.width, cur.overlay.size.height);
            st->rgn.fnDestroy((unsigned int)handle);
        }
    }

    ret = st->rgn.fnCreate((unsigned int)handle, &attr);
    if (ret && V4_ERR_ID(ret) == V4_ERR_EXIST) {
        st->rgn.fnDestroy((unsigned int)handle);
        ret = st->rgn.fnCreate((unsigned int)handle, &attr);
    }
    if (ret) {
        HAL_LOG_ERR("HI_MPI_RGN_Create(%d, %ux%u ARGB1555) failed: 0x%x", handle, r->width,
                    r->height, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * Attach one region to its group's VENC channel, if that channel exists.
 *
 * Returns RSS_OK for a deferral, because a deferral is not a failure: it is
 * the normal state between rvd registering a region and rvd creating the
 * encoder channel it belongs to.
 */
static int hisi_osd_try_attach(hisi_state_t *st, int handle, hisi_osd_region_t *r)
{
    v4_rgn_chn_attr ca;
    v4_mpp_chn chn;
    int ret;

    if (r->attached || r->grp < 0)
        return RSS_OK;
    if (r->grp >= HISI_VENC_CHN_NUM || !st->enc[r->grp].created)
        return RSS_OK; /* Deferred, not failed. */

    hisi_osd_mpp_chn(r->grp, &chn);
    hisi_osd_fill_chn_attr(r, &ca);

    ret = st->rgn.fnAttachToChn((unsigned int)handle, &chn, &ca);
    if (ret) {
        HAL_LOG_WARN("HI_MPI_RGN_AttachToChn(region %d, VENC %d) failed: 0x%x", handle, r->grp,
                     ret);
        return RSS_ERR_IO;
    }

    r->attached = true;
    HAL_LOG_DBG("osd: region %d attached to VENC %d at %d,%d layer %u alpha fg/bg %u/%u", handle,
                r->grp, r->x, r->y, hisi_osd_clamp_layer(r->layer), v4_rgn_alpha(r->fg_alpha),
                v4_rgn_alpha(r->bg_alpha));

    return RSS_OK;
}

static void hisi_osd_detach(hisi_state_t *st, int handle, hisi_osd_region_t *r)
{
    v4_mpp_chn chn;

    if (!r->attached)
        return;

    hisi_osd_mpp_chn(r->grp, &chn);
    st->rgn.fnDetachFromChn((unsigned int)handle, &chn);
    r->attached = false;
}

/*
 * Push position, layer, alpha and show to an attached region.
 *
 * HI_MPI_RGN_SetDisplayAttr covers everything about a region except its
 * geometry and its bitmap, so every runtime change rvd makes lands here.
 * A region that is registered but not yet attached silently keeps the new
 * values in its record and gets them at attach time.
 */
static int hisi_osd_push_chn_attr(hisi_state_t *st, int handle, hisi_osd_region_t *r)
{
    v4_rgn_chn_attr ca;
    v4_mpp_chn chn;
    int ret;

    if (!r->attached)
        return RSS_OK;

    hisi_osd_mpp_chn(r->grp, &chn);
    hisi_osd_fill_chn_attr(r, &ca);

    ret = st->rgn.fnSetDisplayAttr((unsigned int)handle, &chn, &ca);
    if (ret) {
        HAL_LOG_WARN("HI_MPI_RGN_SetDisplayAttr(region %d, VENC %d) failed: 0x%x", handle, r->grp,
                     ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* How many regions are already registered to a group. The budget is a
 * per-VENC-channel one (OVERLAY_MAX_NUM_VENC), so it is counted per group
 * rather than globally. */
static int hisi_osd_group_count(const hisi_state_t *st, int grp, int except)
{
    int i, n = 0;

    for (i = 0; i < HISI_OSD_REGION_MAX; i++)
        if (i != except && st->osd[i].used && st->osd[i].grp == grp)
            n++;

    return n;
}

/* ================================================================
 * CROSS-FILE HOOKS
 * ================================================================ */

/*
 * hisi_osd_flush_pending -- attach every region waiting on this VENC channel.
 *
 * Called from hal_bind, where the SigmaStar backend calls its counterpart,
 * and from hal_enc_create_channel, which is the earlier of the two moments
 * an attach can succeed and the one that makes an encoder restart
 * transparent to the overlay. Both are idempotent: an already-attached
 * region is skipped in hisi_osd_try_attach.
 *
 * Failures are logged there and deliberately not propagated. A region that
 * will not attach costs an overlay; failing the bind over it would cost the
 * stream.
 */
void hisi_osd_flush_pending(hisi_state_t *st, int chn)
{
    int i;

    if (!hisi_osd_ready(st) || chn < 0 || chn >= HISI_VENC_CHN_NUM)
        return;

    for (i = 0; i < HISI_OSD_REGION_MAX; i++)
        if (st->osd[i].used && st->osd[i].grp == chn)
            hisi_osd_try_attach(st, i, &st->osd[i]);
}

/*
 * hisi_osd_detach_chn -- detach every region on this VENC channel, keeping
 * the registration.
 *
 * Called from hal_enc_destroy_channel *before* HI_MPI_VENC_DestroyChn.
 * Mandatory rather than tidy: HI_ERR_RGN_BUSY is documented as "destroy a
 * venc chn without unregistering it". `grp` is left set, so the next
 * hisi_osd_flush_pending for this channel puts the regions back.
 */
void hisi_osd_detach_chn(hisi_state_t *st, int chn)
{
    int i;

    if (!hisi_osd_ready(st) || chn < 0 || chn >= HISI_VENC_CHN_NUM)
        return;

    for (i = 0; i < HISI_OSD_REGION_MAX; i++)
        if (st->osd[i].used && st->osd[i].grp == chn)
            hisi_osd_detach(st, i, &st->osd[i]);
}

/*
 * hisi_osd_release_all -- give every region back. Called from
 * hisi_video_teardown, before the encoder channels go.
 *
 * Order matters for the same reason hisi_osd_detach_chn exists: a region
 * still attached to a VENC channel makes that channel's destroy fail.
 */
void hisi_osd_release_all(hisi_state_t *st)
{
    int i;

    if (!hisi_osd_ready(st))
        return;

    for (i = 0; i < HISI_OSD_REGION_MAX; i++) {
        hisi_osd_region_t *r = &st->osd[i];

        if (!r->used)
            continue;

        hisi_osd_detach(st, i, r);
        st->rgn.fnDestroy((unsigned int)i);
        free(r->bmp);
        memset(r, 0, sizeof(*r));
        r->grp = -1;
    }

    memset(st->osd_grp, 0, sizeof(st->osd_grp));
}

/* ================================================================
 * OPS
 * ================================================================ */

/*
 * RGN allocates a region's canvases when the region is created, out of the
 * MMZ, and publishes no size control through the MPI -- the only knob is
 * the rgn.ko module parameter, which is a boot-time property of the running
 * kernel. So there is nothing to set.
 *
 * Reported once and accepted rather than refused: rvd asks for a pool
 * because Ingenic needs one, and answering NOTSUP would make its
 * osd-restart path log a failure for a step that was never necessary here.
 */
int hal_osd_set_pool_size(void *ctx, uint32_t bytes)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;

    if (!st->osd_pool_logged) {
        st->osd_pool_logged = true;
        HAL_LOG_INFO("osd: pool size %u ignored -- RGN allocates per region from the MMZ", bytes);
    }

    return RSS_OK;
}

int hal_osd_create_group(void *ctx, int grp)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    if (grp < 0 || grp >= HISI_VENC_CHN_NUM) {
        HAL_LOG_ERR("osd: group %d out of range (0..%d)", grp, HISI_VENC_CHN_NUM - 1);
        return RSS_ERR_INVAL;
    }

    /* No RGN object: a group is the encoder channel the regions appear on,
     * and the encoder channel is the encoder's to create. This only records
     * that rvd asked for one, so destroy_group has something to undo. */
    st->osd_grp[grp] = true;

    return RSS_OK;
}

int hal_osd_destroy_group(void *ctx, int grp)
{
    hisi_state_t *st = hisi_state(ctx);
    int i;

    if (!st)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    if (grp < 0 || grp >= HISI_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    /* Regions outlive their group -- they are global RGN objects -- so this
     * detaches its members and leaves them created, which is what rvd
     * expects when it tears one stream down and leaves another running.
     * The registration goes with the group, unlike hisi_osd_detach_chn's
     * encoder restart: rvd will not be putting this group back. */
    for (i = 0; i < HISI_OSD_REGION_MAX; i++) {
        if (st->osd[i].used && st->osd[i].grp == grp) {
            hisi_osd_detach(st, i, &st->osd[i]);
            st->osd[i].grp = -1;
        }
    }

    st->osd_grp[grp] = false;

    return RSS_OK;
}

/*
 * No RGN equivalent: there is no group object to schedule, and whether an
 * overlay is composited is the per-region show flag osd_show_region already
 * owns. Accepted as a no-op rather than refused, because rvd calls both
 * unconditionally around every stream and a NOTSUP here reads in its log
 * like a broken OSD.
 */
int hal_osd_start(void *ctx, int grp)
{
    (void)grp;

    return hisi_state(ctx) ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_stop(void *ctx, int grp)
{
    (void)grp;

    return hisi_state(ctx) ? RSS_OK : RSS_ERR_INVAL;
}

int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_osd_region_t *r;
    int slot;
    int ret;

    if (!st || !handle || !attr)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    if (attr->type == RSS_OSD_COVER) {
        /* See the OP COVERAGE block: COVER_RGN has no VENC budget entry, so
         * the encoder cannot composite one. rvd's privacy path reads the
         * refusal and carries on. */
        HAL_LOG_INFO("osd: COVER regions are not composited by VENC on gen4; "
                     "privacy cover unavailable");
        return RSS_ERR_NOTSUP;
    }
    if (attr->width <= 0 || attr->height <= 0) {
        HAL_LOG_ERR("osd: region geometry %dx%d is empty", attr->width, attr->height);
        return RSS_ERR_INVAL;
    }

    for (slot = 0; slot < HISI_OSD_REGION_MAX; slot++)
        if (!st->osd[slot].used)
            break;
    if (slot == HISI_OSD_REGION_MAX) {
        HAL_LOG_ERR("osd: all %d region slots in use", HISI_OSD_REGION_MAX);
        return RSS_ERR_NOMEM;
    }

    r = &st->osd[slot];
    memset(r, 0, sizeof(*r));
    r->type = attr->type;
    r->x = hisi_osd_align_pos(attr->x);
    r->y = hisi_osd_align_pos(attr->y);
    /*
     * Two geometries, and the difference is load-bearing. src_* is what the
     * caller renders -- rvd's bitmap is exactly src_w * src_h * 4 bytes --
     * while width/height are those rounded up to RGN's even alignment.
     * Conflating them reads past the caller's buffer on any odd dimension,
     * which is the kind of bug that survives every test on a font whose
     * glyphs happen to be even.
     */
    r->src_w = (unsigned int)attr->width;
    r->src_h = (unsigned int)attr->height;
    r->width = hisi_osd_align_dim(attr->width);
    r->height = hisi_osd_align_dim(attr->height);
    r->layer = attr->layer;
    r->global_alpha_en = attr->global_alpha_en;
    r->fg_alpha = attr->fg_alpha;
    r->bg_alpha = attr->bg_alpha;
    r->grp = -1;
    /* rvd creates regions hidden and shows them once there is something to
     * draw; osd_show_region is what flips this. */
    r->show = false;

    ret = hisi_osd_create_rgn(st, slot, r);
    if (ret) {
        memset(r, 0, sizeof(*r));
        return ret;
    }

    r->used = true;
    *handle = slot;

    HAL_LOG_DBG("osd: region %d created, %ux%u at %d,%d layer %d", slot, r->width, r->height, r->x,
                r->y, r->layer);

    return RSS_OK;
}

int hal_osd_destroy_region(void *ctx, int handle)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_osd_region_t *r;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    r = hisi_osd_slot(st, handle);
    if (!r)
        return RSS_ERR_NOENT;

    hisi_osd_detach(st, handle, r);

    ret = st->rgn.fnDestroy((unsigned int)handle);
    if (ret)
        HAL_LOG_WARN("HI_MPI_RGN_Destroy(%d) failed: 0x%x", handle, ret);

    free(r->bmp);
    memset(r, 0, sizeof(*r));
    r->grp = -1;

    return ret ? RSS_ERR_IO : RSS_OK;
}

int hal_osd_register_region(void *ctx, int handle, int grp)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_osd_region_t *r;

    if (!st)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    r = hisi_osd_slot(st, handle);
    if (!r)
        return RSS_ERR_NOENT;
    if (grp < 0 || grp >= HISI_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    if (r->grp == grp)
        return hisi_osd_try_attach(st, handle, r);

    if (hisi_osd_group_count(st, grp, handle) >= V4_RGN_OVERLAY_MAX_PER_VENC) {
        /* OVERLAY_MAX_NUM_VENC. Refused here rather than left to the driver
         * because the driver's refusal arrives at AttachToChn, which on the
         * deferred path happens inside hal_bind where nothing can report
         * it. */
        HAL_LOG_ERR("osd: VENC %d already carries %d overlays, the gen4 maximum", grp,
                    V4_RGN_OVERLAY_MAX_PER_VENC);
        return RSS_ERR_NOMEM;
    }

    if (r->grp >= 0) {
        /* One region, one channel. RGN would accept the same handle on two
         * channels, but raptor's model has a region belonging to a group,
         * and silently leaving it on the old one would be worse than saying
         * so. */
        HAL_LOG_WARN("osd: region %d moving from group %d to %d", handle, r->grp, grp);
        hisi_osd_detach(st, handle, r);
    }

    r->grp = grp;

    /* Succeeds now if the encoder channel is already there (a region added
     * to a running stream), defers to hisi_osd_flush_pending if this is
     * startup. */
    return hisi_osd_try_attach(st, handle, r);
}

int hal_osd_unregister_region(void *ctx, int handle, int grp)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_osd_region_t *r;

    if (!st)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    r = hisi_osd_slot(st, handle);
    if (!r)
        return RSS_ERR_NOENT;
    if (grp >= 0 && r->grp != grp)
        return RSS_ERR_INVAL;

    hisi_osd_detach(st, handle, r);
    r->grp = -1;

    return RSS_OK;
}

/*
 * Geometry, position, alpha and layer in one call.
 *
 * A size change cannot be applied in place: RGN fixes a region's canvases
 * at create time and HI_MPI_RGN_SetAttr will not resize them, so it becomes
 * detach + destroy + create + re-attach. Everything else is a display-attr
 * push. divinus reaches the same conclusion from the other direction, by
 * comparing the current region config and recreating on a mismatch.
 */
int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_osd_region_t *r;
    unsigned int w, h;
    bool resized;
    int grp;
    int ret;

    if (!st || !attr)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    r = hisi_osd_slot(st, handle);
    if (!r)
        return RSS_ERR_NOENT;
    if (attr->width <= 0 || attr->height <= 0)
        return RSS_ERR_INVAL;
    if (attr->type == RSS_OSD_COVER)
        return RSS_ERR_NOTSUP;

    w = hisi_osd_align_dim(attr->width);
    h = hisi_osd_align_dim(attr->height);
    resized = w != r->width || h != r->height;

    r->x = hisi_osd_align_pos(attr->x);
    r->y = hisi_osd_align_pos(attr->y);
    r->layer = attr->layer;
    r->global_alpha_en = attr->global_alpha_en;
    r->fg_alpha = attr->fg_alpha;
    r->bg_alpha = attr->bg_alpha;
    r->src_w = (unsigned int)attr->width;
    r->src_h = (unsigned int)attr->height;

    if (!resized)
        return hisi_osd_push_chn_attr(st, handle, r);

    grp = r->grp;
    hisi_osd_detach(st, handle, r);
    st->rgn.fnDestroy((unsigned int)handle);

    r->width = w;
    r->height = h;

    /* The old conversion buffer is the wrong size; the next update
     * reallocates. */
    free(r->bmp);
    r->bmp = NULL;
    r->bmp_size = 0;
    r->bmp_logged = false;

    ret = hisi_osd_create_rgn(st, handle, r);
    if (ret) {
        /* The slot no longer has an RGN region behind it, so stop claiming
         * it does. */
        memset(r, 0, sizeof(*r));
        r->grp = -1;
        return ret;
    }

    r->grp = grp;

    return hisi_osd_try_attach(st, handle, r);
}

/*
 * BGRA8888 from rvd -> ARGB1555.
 *
 * The source is src_w x src_h; the destination is the region's aligned
 * width x height, which is at most one pixel wider and one taller. The
 * overhang is written transparent rather than left uninitialised -- RGN
 * reads the whole canvas, and an uninitialised column shows up as a stripe
 * of noise down the right-hand edge of every odd-width overlay.
 *
 * Alpha collapses to one bit: anything not fully transparent is drawn. The
 * opposite threshold drops antialiased strokes and makes small text vanish.
 */
static void hisi_osd_convert(const uint8_t *src, uint16_t *dst, const hisi_osd_region_t *r)
{
    unsigned int x, y;

    for (y = 0; y < r->height; y++) {
        uint16_t *out = dst + (size_t)y * r->width;

        if (y >= r->src_h) {
            memset(out, 0, (size_t)r->width * HISI_OSD_BPP);
            continue;
        }

        for (x = 0; x < r->src_w && x < r->width; x++) {
            const uint8_t *p = src + ((size_t)y * r->src_w + x) * 4;
            uint8_t b = p[0];
            uint8_t g = p[1];
            uint8_t rr = p[2];
            uint8_t a = p[3];

            out[x] = (uint16_t)((a ? 0x8000u : 0u) | ((unsigned)(rr >> 3) << 10) |
                                ((unsigned)(g >> 3) << 5) | (unsigned)(b >> 3));
        }
        for (; x < r->width; x++)
            out[x] = 0;
    }
}

int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_osd_region_t *r;
    v4_bitmap bmp;
    size_t need;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    r = hisi_osd_slot(st, handle);
    if (!r)
        return RSS_ERR_NOENT;
    if (!data) {
        /* rvd's Ingenic sequence sets the attr with a NULL data pointer
         * before it has anything to draw. Nothing to push. */
        return RSS_OK;
    }

    need = (size_t)r->width * r->height * HISI_OSD_BPP;
    if (r->bmp_size < need) {
        void *nb = realloc(r->bmp, need);

        if (!nb)
            return RSS_ERR_NOMEM;
        r->bmp = nb;
        r->bmp_size = need;
    }

    hisi_osd_convert(data, (uint16_t *)r->bmp, r);

    memset(&bmp, 0, sizeof(bmp));
    bmp.pixel_format = V4_RGN_PIXFMT_ARGB1555;
    bmp.width = r->width;
    bmp.height = r->height;
    bmp.data = r->bmp;

    ret = st->rgn.fnSetBitMap((unsigned int)handle, &bmp);
    if (ret) {
        HAL_LOG_WARN("HI_MPI_RGN_SetBitMap(region %d, %ux%u) failed: 0x%x", handle, r->width,
                     r->height, ret);
        return RSS_ERR_IO;
    }

    if (!r->bmp_logged) {
        r->bmp_logged = true;
        /*
         * Once per region, for the reason the SigmaStar backend states: the
         * two ways an invisible overlay can fail -- attached but never fed,
         * fed but not composited -- are indistinguishable from the log
         * without it.
         */
        HAL_LOG_DBG("osd: region %d first bitmap accepted, %ux%u ARGB1555", handle, r->width,
                    r->height);
    }

    return RSS_OK;
}

/*
 * Show/hide plus z-order.
 *
 * rvd passes the layer here as well as in the region attr, and this is the
 * call it makes when an overlay's visibility changes, so both are recorded
 * and pushed together.
 */
int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_osd_region_t *r;

    if (!st)
        return RSS_ERR_INVAL;
    if (!hisi_osd_ready(st))
        return RSS_ERR_NOTSUP;
    r = hisi_osd_slot(st, handle);
    if (!r)
        return RSS_ERR_NOENT;
    if (grp >= 0 && r->grp >= 0 && r->grp != grp)
        return RSS_ERR_INVAL;

    r->show = show ? true : false;
    if (layer >= 0)
        r->layer = layer;

    return hisi_osd_push_chn_attr(st, handle, r);
}
