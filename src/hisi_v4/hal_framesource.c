/*
 * hisi_v4/hal_framesource.c -- VPSS channels as raptor framesources
 *
 * MAPPING. raptor framesource channel N == VPSS channel N of group
 * HISI_VPSS_GRP. One group, because one sensor feeds one VI pipe and the
 * group is what the pipe binds to; N channels, because a channel is exactly
 * "one scaled copy of the group's picture", which is what a framesource is.
 *
 * The group is created and started by hal_init, not here: it carries the
 * 3DNR state and the VI bind, and both are properties of the sensor rather
 * than of any one stream. So a framesource op only ever configures,
 * enables or disables a *channel*.
 *
 * OP COVERAGE
 *
 * Published: create/set/destroy channel, enable/disable, get/release frame,
 * get/set frame depth, set rotation.
 *
 * Absent, and why:
 *
 *  - fs_set_fifo / fs_get_fifo. Ingenic's FIFO is a per-channel queue depth
 *    with its own semantics; gen4's nearest thing is u32Depth, which
 *    fs_set_frame_depth already is. Two ops onto one field would let a
 *    caller set it twice and get whichever ran last.
 *
 *  - fs_set_delay / fs_set_max_delay. HI_MPI_VPSS_SetGrpDelay exists but is
 *    a *group* control, so a per-channel op would silently affect every
 *    stream. It belongs on an op that names the group, and rvd has none.
 *
 *  - fs_set_pool / fs_get_pool. gen4 pools are configured before
 *    HI_MPI_VB_Init and cannot be reassigned per channel afterwards.
 *
 *  - fs_snap_frame. rvd's snapshot path goes through a JPEG encoder
 *    channel here, as it does on SigmaStar; a raw grab off VPSS would need
 *    its own depth and would compete with the streaming path for the
 *    channel's queue.
 *
 * Each of those is simply not in the vtable, so RSS_HAL_CALL's NULL guard
 * answers RSS_ERR_NOTSUP and rvd treats them as advisory.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

/* ================================================================
 * CHANNEL LOOKUP
 * ================================================================ */

/*
 * Resolve ctx and channel number in one place.
 *
 * The bound is HISI_FS_CHN_NUM -- the physical channels usable as outputs,
 * which is one fewer than the SDK's count; see HISI_VPSS_CHN_BASE -- and
 * not caps.max_fs_channels: caps is what raptor *promises*, and a request
 * past it should be rejected by the caller rather than crash here. This is
 * the memory-safety bound.
 */
#define HISI_FS_ENTER(ctx, chn, st_var, fs_var)                                                    \
    hisi_state_t *st_var = hisi_state(ctx);                                                        \
    hisi_vpss_chn_t *fs_var;                                                                       \
    do {                                                                                           \
        if (!st_var)                                                                               \
            return RSS_ERR_INVAL;                                                                  \
        if ((chn) < 0 || (chn) >= HISI_FS_CHN_NUM) {                                               \
            HAL_LOG_ERR("vpss: channel %d out of range [0,%d)", (chn), HISI_FS_CHN_NUM);           \
            return RSS_ERR_INVAL;                                                                  \
        }                                                                                          \
        fs_var = &st_var->fs[chn];                                                                 \
    } while (0)

/* ================================================================
 * CHANNEL ATTRIBUTES
 * ================================================================ */

/*
 * hisi_fs_fill_attr -- build VPSS_CHN_ATTR_S from a raptor framesource config.
 *
 * Fresh every time, never read-modify-write. Everything the driver reads is
 * set here, so a round trip through GetChnAttr would only reintroduce
 * whatever the getter declines to populate.
 *
 * The choices worth stating:
 *
 *   enChnMode = USER. The channel's size is the stream's size, which is
 *   what the caller asked for. AUTO would make it follow the group and
 *   silently ignore the request.
 *
 *   enPixelFormat = YVU_SEMIPLANAR_420. The only format VENC accepts on
 *   this family, and the only one anything downstream of VPSS wants.
 *
 *   enCompressMode = NONE. SEG compression saves DDR bandwidth on the *VI*
 *   side, where nothing in userspace reads the frame. A VPSS channel's
 *   output may be read through fs_get_frame, and a compressed frame handed
 *   to a caller expecting NV12 is unreadable.
 *
 *   bMirror / bFlip are the backend's orientation, the same for every
 *   channel -- see hisi_state_t.mirror for why they are here and not on
 *   the VI channel or at the sensor.
 *
 *   u32Depth comes from the channel's tracked value, which is 0 for a
 *   streaming channel. See hisi_vpss_chn_t.depth for why zero is right and
 *   why it is not a bug that GetChnFrame then blocks.
 */
static void hisi_fs_fill_attr(const hisi_state_t *st, const hisi_vpss_chn_t *fs,
                              v4_vpss_chn_attr *attr)
{
    memset(attr, 0, sizeof(*attr));

    attr->chn_mode = V4_VPSS_CHN_MODE_USER;
    attr->width = fs->width;
    attr->height = fs->height;
    attr->video_format = V4_VIDEO_FORMAT_LINEAR;
    attr->pixel_format = V4_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    attr->dynamic_range = V4_DYNAMIC_RANGE_SDR8;
    attr->compress_mode = V4_COMPRESS_MODE_NONE;
    attr->frame_rate = fs->frame_rate;
    attr->mirror = st->mirror;
    attr->flip = st->flip;
    attr->depth = fs->depth;
}

/*
 * hisi_fs_apply_orien -- write the current mirror and flip to every channel
 * that exists. The attribute is rebuilt from tracked state rather than read
 * back, as everywhere in this file, and VPSS takes it on an enabled channel
 * without a gap in the bound encoder's frames. Called by the hflip / vflip
 * ops; a channel created later gets the same bits from hisi_fs_fill_attr.
 */
int hisi_fs_apply_orien(hisi_state_t *st)
{
    int applied = 0;

    if (!st->vpss.fnSetChnAttr)
        return RSS_ERR_NOTSUP;
    for (int chn = 0; chn < HISI_FS_CHN_NUM; chn++) {
        hisi_vpss_chn_t *fs = &st->fs[chn];
        v4_vpss_chn_attr attr;
        int ret;

        if (!fs->configured)
            continue;
        hisi_fs_fill_attr(st, fs, &attr);
        ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, hisi_vpss_phy(chn), &attr);
        if (ret) {
            HAL_LOG_WARN("HI_MPI_VPSS_SetChnAttr(%d, %d) mirror %d flip %d failed: 0x%x",
                         HISI_VPSS_GRP, chn, st->mirror, st->flip, ret);
            return RSS_ERR_IO;
        }
        applied++;
    }
    if (applied)
        HAL_LOG_INFO("orientation: mirror %d, flip %d on %d vpss channel%s", st->mirror, st->flip,
                     applied, applied == 1 ? "" : "s");
    else
        HAL_LOG_DBG("orientation: mirror %d, flip %d noted for the channels to come", st->mirror,
                    st->flip);
    return RSS_OK;
}

/*
 * Frame rate. rvd expresses it as a fraction and gen4 wants a pair of
 * integers, source and destination, where the driver drops frames to get
 * from one to the other.
 *
 * The source is the sensor's rate from the mode INI, not the requested one:
 * telling the driver the source is what you want is how you get no dropping
 * at all and a stream running at the sensor's rate regardless of the config.
 * Both -1 means "no control", which is what a request equal to the source
 * should become -- an fps controller configured to pass everything through
 * still costs a comparison per frame.
 */
static void hisi_fs_frame_rate(const hisi_state_t *st, uint32_t num, uint32_t den,
                               v4_frame_rate *out)
{
    int src = st->mode.frame_rate > 0 ? st->mode.frame_rate : 25;
    int dst;

    if (!num || !den) {
        out->src_frame_rate = -1;
        out->dst_frame_rate = -1;
        return;
    }

    dst = (int)(num / den);
    if (dst <= 0)
        dst = 1;
    if (dst >= src) {
        out->src_frame_rate = -1;
        out->dst_frame_rate = -1;
        return;
    }

    out->src_frame_rate = src;
    out->dst_frame_rate = dst;
}

/* ================================================================
 * OPS
 * ================================================================ */

/*
 * hisi_fs_apply_crop -- the window the group reads before it scales.
 *
 * VPSS scales what it is given to each channel's width and height, on each
 * axis independently. A 2592x1944 group feeding a 1920x1080 channel is
 * therefore not a downscale but a downscale plus a 4:3-to-16:9 stretch, and
 * everything in frame comes out a third too wide. Cropping the source to
 * the target aspect first is what makes the scale uniform, and that is what
 * rss_fs_config_t's crop means: "ISP-level crop (before scaling)".
 *
 * It is the *group* crop, not the channel crop, and that is measured rather
 * than assumed. HI_MPI_VPSS_SetChnCrop accepts the same rectangle happily
 * -- /proc/umap/vpss shows CropEn Y, ABS, 2592x1458+0+242 -- and then the
 * channel stops producing entirely, with the proc's Trim columns reading
 * 1920x838 for a 1920x1080 channel. 838 is 1080 - 242: the driver is
 * applying the rectangle in the channel's *output* coordinates, so a crop
 * meant to select part of the sensor instead clips part of the encoded
 * picture and starves the channel. The group crop applies where the name
 * suggests, ahead of the scalers.
 *
 * The consequence is that the window is shared. One group serves every
 * stream, so two channels asking for different crops cannot both be
 * honoured; the first one wins and the second is reported rather than
 * silently dropped. In practice raptor's streams differ in resolution and
 * not in aspect, so they ask for the same window.
 *
 * Absent on a build whose VPSS does not export SetGrpCrop: not an error,
 * because a crop nobody asked for costs nothing, and one that was asked for
 * says so.
 */
static int hisi_fs_apply_crop(hisi_state_t *st, int chn, const rss_fs_config_t *cfg)
{
    v4_vpss_crop_info crop;
    int ret;

    if (!st->vpss.fnSetGrpCrop) {
        if (cfg->crop.enable)
            HAL_LOG_WARN("vpss chn %d: crop requested, HI_MPI_VPSS_SetGrpCrop unavailable", chn);
        return RSS_OK;
    }

    /*
     * The rect is filled in even when the crop is disabled, because the
     * driver range-checks it either way: a zeroed rect with bEnable false
     * comes back 0xa0078003, VPSS / ILLEGAL_PARAM. "No crop" is the whole
     * group frame, so that is what an off crop carries.
     */
    memset(&crop, 0, sizeof(crop));
    crop.crop_coordinate = V4_VPSS_CROP_ABS;
    crop.crop_rect.width = st->mode.dev_rect.width;
    crop.crop_rect.height = st->mode.dev_rect.height;

    if (cfg->crop.enable && cfg->crop.w > 0 && cfg->crop.h > 0) {
        /*
         * Every edge rounded to an even number, and clamped to the frame.
         *
         * The output is YUV420, so a chroma sample spans two luma samples
         * on each axis and a window cannot start or end between them. The
         * driver enforces it without saying so: an odd origin comes back
         * 0xa0078003, VPSS / ILLEGAL_PARAM, which is the same answer it
         * gives for a rect off the edge of the frame or one narrower than
         * VPSS_MIN_IMAGE_WIDTH. Centring a 16:9 window in a 4:3 sensor
         * lands on an odd y about half the time -- 2592x1458 out of
         * 2592x1944 gives y = 243 -- so this is the common case rather
         * than a corner.
         *
         * Rounded down rather than up on the size, so that alignment can
         * never push the window past the edge it was clamped to.
         */
        unsigned int fw = st->mode.dev_rect.width;
        unsigned int fh = st->mode.dev_rect.height;
        unsigned int cx = cfg->crop.x > 0 ? (unsigned int)cfg->crop.x & ~1u : 0u;
        unsigned int cy = cfg->crop.y > 0 ? (unsigned int)cfg->crop.y & ~1u : 0u;
        unsigned int cw = (unsigned int)cfg->crop.w & ~1u;
        unsigned int chh = (unsigned int)cfg->crop.h & ~1u;

        if (cx >= fw || cy >= fh) {
            HAL_LOG_ERR("vpss chn %d: crop origin %ux%u outside the %ux%u frame", chn, cx, cy, fw,
                        fh);
            return RSS_ERR_INVAL;
        }
        if (cw > fw - cx)
            cw = (fw - cx) & ~1u;
        if (chh > fh - cy)
            chh = (fh - cy) & ~1u;

        if (cw && chh) {
            crop.enable = 1;
            crop.crop_rect.x = (int)cx;
            crop.crop_rect.y = (int)cy;
            crop.crop_rect.width = cw;
            crop.crop_rect.height = chh;
        } else {
            /* Asked for, produced nothing: say so rather than silently
             * running uncropped with the aspect error the crop was meant
             * to fix. */
            HAL_LOG_ERR("vpss chn %d: crop %dx%d+%d+%d collapses to zero after alignment; "
                        "running uncropped",
                        chn, cfg->crop.w, cfg->crop.h, cfg->crop.x, cfg->crop.y);
            return RSS_ERR_INVAL;
        }
    }

    /*
     * One window per group, with an owner rather than a latch. The channel
     * whose request set the window may move or clear it -- a resolution
     * change from a 16:9 stream back to 4:3 must not keep the old 16:9
     * window -- while a non-owner is held to the window that exists: the
     * identical rect is the normal two-streams-same-aspect case and
     * succeeds silently, a different rect is refused out loud.
     */
    if (!crop.enable) {
        if (st->vpss_crop_owner < 0)
            return RSS_OK;
        if (st->vpss_crop_owner != chn) {
            HAL_LOG_DBG("vpss chn %d: no crop; keeping chn %d's group window", chn,
                        st->vpss_crop_owner);
            return RSS_OK;
        }
        /* The owner turning its crop off: restore the full frame. The rect
         * already carries it -- see the fill above. */
    } else if (st->vpss_crop_owner >= 0 && st->vpss_crop_owner != chn) {
        if (crop.crop_rect.x == st->vpss_crop_x && crop.crop_rect.y == st->vpss_crop_y &&
            crop.crop_rect.width == st->vpss_crop_w && crop.crop_rect.height == st->vpss_crop_h)
            return RSS_OK;
        HAL_LOG_WARN("vpss chn %d: crop %ux%u+%d+%d refused; chn %d holds the group window "
                     "%ux%u+%d+%d",
                     chn, crop.crop_rect.width, crop.crop_rect.height, crop.crop_rect.x,
                     crop.crop_rect.y, st->vpss_crop_owner, st->vpss_crop_w, st->vpss_crop_h,
                     st->vpss_crop_x, st->vpss_crop_y);
        return RSS_OK;
    } else if (st->vpss_crop_owner == chn && crop.crop_rect.x == st->vpss_crop_x &&
               crop.crop_rect.y == st->vpss_crop_y && crop.crop_rect.width == st->vpss_crop_w &&
               crop.crop_rect.height == st->vpss_crop_h) {
        return RSS_OK;
    }

    ret = st->vpss.fnSetGrpCrop(HISI_VPSS_GRP, &crop);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_SetGrpCrop(%d) %ux%u+%d+%d failed: 0x%x", HISI_VPSS_GRP,
                    crop.crop_rect.width, crop.crop_rect.height, crop.crop_rect.x, crop.crop_rect.y,
                    ret);
        return RSS_ERR_IO;
    }

    if (crop.enable) {
        st->vpss_crop_owner = chn;
        st->vpss_crop_x = crop.crop_rect.x;
        st->vpss_crop_y = crop.crop_rect.y;
        st->vpss_crop_w = crop.crop_rect.width;
        st->vpss_crop_h = crop.crop_rect.height;
        HAL_LOG_INFO("vpss: group crop %ux%u+%d+%d (owner chn %d)", crop.crop_rect.width,
                     crop.crop_rect.height, crop.crop_rect.x, crop.crop_rect.y, chn);
    } else {
        st->vpss_crop_owner = -1;
        HAL_LOG_INFO("vpss: group crop cleared (chn %d)", chn);
    }
    return RSS_OK;
}

/* ================================================================
 * PER-CHANNEL VB POOLS
 * ================================================================ */

/*
 * Blocks in a channel's own pool.
 *
 * Four, which is what the common stream pool this replaces was configured
 * with and what /proc/umap/vb measured it actually using: one sub-stream
 * channel on a four-block pool held MinFree at 1. rvd's nr_vbs raises it
 * where a caller has asked for more, and never lowers it -- rvd's own
 * default is 2, which is its Ingenic-era figure and not a statement about
 * this pipeline.
 */
#define HISI_FS_POOL_BLK_CNT 4u

/*
 * The block size this channel's own pool should have, or 0 to leave it in
 * the common pool.
 *
 * The threshold is pool 0's block. A channel whose frames fill one anyway
 * gains nothing from a pool of its own and would cost a second set of them:
 * on a 5 MP board the full-resolution main stream is exactly that case, and
 * it stays where it has always been. What this catches is the sub-stream,
 * whose 640x480 frame is a sixteenth of a sensor block.
 */
static unsigned long long hisi_fs_pool_want(const hisi_state_t *st, const hisi_vpss_chn_t *fs)
{
    unsigned long long size;

    if (!st->vb_private_pools || !st->vb_blk_size)
        return 0;
    if (!fs->width || !fs->height)
        return 0;

    size = hisi_vb_nv12_size(fs->width, fs->height);
    if (size >= st->vb_blk_size)
        return 0;
    return size;
}

/*
 * hisi_fs_pool_acquire -- cut a pool to this channel's frame and attach it.
 *
 * Nothing here fails the bring-up, deliberately: a channel with no pool of
 * its own draws from the common pool, which is what every channel does on
 * this board today and is correct everywhere. That keeps the optimisation's
 * failure mode the same shape as the one the pool it replaced had -- a
 * stream too big for it quietly fell back to pool 0 -- rather than
 * introducing a new way for a pipeline to refuse to start.
 *
 * On the EV300 this never gets past the attach; see the NOT_SUPPORT branch.
 */
static void hisi_fs_pool_acquire(hisi_state_t *st, int chn, hisi_vpss_chn_t *fs, int nr_vbs)
{
    v4_vb_pool_conf conf;
    unsigned long long size = hisi_fs_pool_want(st, fs);
    unsigned int pool;
    int ret;

    if (!size)
        return;

    memset(&conf, 0, sizeof(conf));
    conf.blk_size = size;
    conf.blk_cnt = nr_vbs > (int)HISI_FS_POOL_BLK_CNT ? (unsigned int)nr_vbs : HISI_FS_POOL_BLK_CNT;
    conf.remap_mode = V4_VB_REMAP_NOCACHE;
    /* acMmzName left empty: the anonymous zone, which is where the common
     * pools live and the only one this board declares in /proc/media-mem. */

    pool = st->sys.fnVbCreatePool(&conf);
    if (pool == V4_VB_INVALID_POOL) {
        HAL_LOG_WARN("vpss chn %d: HI_MPI_VB_CreatePool(%llu x %u) failed; "
                     "the channel will use the common pool",
                     chn, conf.blk_size, conf.blk_cnt);
        return;
    }

    ret = st->vpss.fnAttachVbPool(HISI_VPSS_GRP, hisi_vpss_phy(chn), pool);
    if (ret) {
        st->sys.fnVbDestroyPool(pool);
        /*
         * NOT_SUPPORT is the driver saying it has no such operation, not
         * this call being wrong, so stop asking: every later channel would
         * get the same answer and log the same line. Measured on an EV300
         * running the openhisilicon modules, which is what this is for --
         * HI_MPI_VPSS_AttachVbPool is in libmpi's symbol table and returns
         * 0xa0078008 whether it is called before EnableChn or after.
         */
        if (ret == V4_ERR_VPSS_NOT_SUPPORT) {
            st->vb_private_pools = false;
            HAL_LOG_INFO("vpss: this driver has no AttachVbPool (0x%x); channels take their "
                         "output blocks from the common pool",
                         ret);
        } else {
            HAL_LOG_WARN("HI_MPI_VPSS_AttachVbPool(%d, %d, %u) failed: 0x%x; "
                         "the channel will use the common pool",
                         HISI_VPSS_GRP, hisi_vpss_phy(chn), pool, ret);
        }
        return;
    }

    fs->vb_pool_owned = true;
    fs->vb_pool = pool;
    fs->vb_pool_blk_size = conf.blk_size;
    HAL_LOG_INFO("vpss chn %d: vb pool %u, %llu x %u = %llu KiB", chn, pool, conf.blk_size,
                 conf.blk_cnt, conf.blk_size * conf.blk_cnt / 1024ull);
}

/*
 * hisi_fs_pool_release -- detach and free, in that order.
 *
 * The destroy runs even when the detach reported a failure. A pool the
 * driver still considers attached refuses to be destroyed and says so, which
 * is a message; a pool nothing points at and nobody freed is CMA gone until
 * the modules are reloaded, which is not. The caller disables the channel
 * first -- see hal_fs_destroy_channel and hisi_fs_release_all.
 */
static void hisi_fs_pool_release(hisi_state_t *st, int chn, hisi_vpss_chn_t *fs)
{
    int ret;

    if (!fs->vb_pool_owned)
        return;

    ret = st->vpss.fnDetachVbPool(HISI_VPSS_GRP, hisi_vpss_phy(chn));
    if (ret)
        HAL_LOG_WARN("HI_MPI_VPSS_DetachVbPool(%d, %d) failed: 0x%x", HISI_VPSS_GRP,
                     hisi_vpss_phy(chn), ret);

    ret = st->sys.fnVbDestroyPool(fs->vb_pool);
    if (ret)
        HAL_LOG_WARN("HI_MPI_VB_DestroyPool(%u) failed: 0x%x", fs->vb_pool, ret);

    fs->vb_pool_owned = false;
    fs->vb_pool = 0;
    fs->vb_pool_blk_size = 0;
}

/*
 * hisi_fs_pool_refresh -- keep the pool matched to a geometry that changed.
 *
 * A channel that grew past its own pool's blocks would otherwise stay
 * attached to a pool that can never satisfy it, and stop producing without
 * logging anything: the silent no-frames failure hisi_vb_nv12_size's comment
 * warns about, arrived at from the other direction. Only an actual change in
 * what the channel needs cycles it, so the fps-only reconfigures rvd issues
 * while streaming still do not touch the channel.
 */
static void hisi_fs_pool_refresh(hisi_state_t *st, int chn, hisi_vpss_chn_t *fs, int nr_vbs)
{
    unsigned long long want;
    bool was_enabled;
    int ret;

    if (!st->vb_private_pools)
        return;

    want = hisi_fs_pool_want(st, fs);
    if (want == (fs->vb_pool_owned ? fs->vb_pool_blk_size : 0ull))
        return;

    was_enabled = fs->enabled;
    if (was_enabled) {
        ret = st->vpss.fnDisableChn(HISI_VPSS_GRP, hisi_vpss_phy(chn));
        if (ret) {
            HAL_LOG_WARN("vpss chn %d: DisableChn failed 0x%x; keeping the old vb pool", chn, ret);
            return;
        }
        fs->enabled = false;
    }

    hisi_fs_pool_release(st, chn, fs);
    hisi_fs_pool_acquire(st, chn, fs, nr_vbs);

    if (was_enabled) {
        ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, hisi_vpss_phy(chn));
        if (ret) {
            HAL_LOG_ERR("HI_MPI_VPSS_EnableChn(%d, %d) after a vb pool change failed: 0x%x",
                        HISI_VPSS_GRP, hisi_vpss_phy(chn), ret);
            return;
        }
        fs->enabled = true;
    }
}

int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    v4_vpss_chn_attr attr;
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    if (!cfg)
        return RSS_ERR_INVAL;
    if (!st->vpss.fnSetChnAttr)
        return RSS_ERR_NOTSUP;
    if (!st->vpss_grp_created) {
        HAL_LOG_ERR("vpss chn %d: no group; hal_init did not complete", chn);
        return RSS_ERR_BUSY;
    }
    if (fs->configured) {
        HAL_LOG_ERR("vpss chn %d: already created", chn);
        return RSS_ERR_BUSY;
    }
    if (!cfg->width || !cfg->height) {
        HAL_LOG_ERR("vpss chn %d: zero geometry", chn);
        return RSS_ERR_INVAL;
    }
    /* The channel always emits NV12 -- see hisi_fs_fill_attr -- so a caller
     * asking for anything else (rvd's raw-grab recreates the channel with a
     * RAW request) must hear that the bytes it dumps will be NV12. */
    if (cfg->pixfmt != RSS_PIXFMT_NV12)
        HAL_LOG_WARN("vpss chn %d: pixfmt %d not supported; output is NV12", chn, (int)cfg->pixfmt);

    fs->width = cfg->width;
    fs->height = cfg->height;
    hisi_fs_frame_rate(st, cfg->fps_num, cfg->fps_den, &fs->frame_rate);

    hisi_fs_fill_attr(st, fs, &attr);
    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, hisi_vpss_phy(chn), &attr);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_SetChnAttr(%d, %d) %ux%u failed: 0x%x", HISI_VPSS_GRP, chn,
                    fs->width, fs->height, ret);
        return RSS_ERR_IO;
    }

    fs->configured = true;

    ret = hisi_fs_apply_crop(st, chn, cfg);
    if (ret != RSS_OK) {
        fs->configured = false;
        return ret;
    }

    /*
     * Enable here, as part of create, and not only in hal_fs_enable_channel.
     *
     * VPSS has no CreateChn on gen4 -- the API is SetChnAttr, EnableChn,
     * DisableChn and nothing else -- so EnableChn is what *instantiates* the
     * channel rather than a streaming on/off switch. A channel that has only
     * been given attributes is not yet a thing the rest of MPP will accept:
     * HI_MPI_SYS_Bind from it returns 0xa0028009, SYS / ERROR /
     * EN_ERR_NOT_PERM, and names nothing.
     *
     * raptor's model is create -> bind -> enable, and rvd issues them in
     * that order, so the bind would always land in the gap. Mapping create
     * onto SetChnAttr+EnableChn closes it and leaves hal_fs_enable_channel
     * idempotent, which is what it already was for an enabled channel. The
     * disable op still works and still stops frames; re-enabling resumes
     * them without touching the bind.
     */
    /* Before EnableChn, which is what instantiates the channel: the attach
     * has to be in place by the time it starts asking for blocks. */
    hisi_fs_pool_acquire(st, chn, fs, cfg->nr_vbs);

    ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, hisi_vpss_phy(chn));
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_EnableChn(%d, %d) failed: 0x%x", HISI_VPSS_GRP, chn, ret);
        hisi_fs_pool_release(st, chn, fs);
        fs->configured = false;
        return RSS_ERR_IO;
    }
    fs->enabled = true;

    HAL_LOG_INFO("vpss chn %d: %ux%u @ %d/%d fps, enabled", chn, fs->width, fs->height,
                 fs->frame_rate.dst_frame_rate, fs->frame_rate.src_frame_rate);
    return RSS_OK;
}

/*
 * hal_fs_set_rotation -- turn one channel's output by a fixed angle.
 *
 * HI_MPI_VPSS_SetChnRotation, which the reference allows after the channel
 * attribute is set and on a physical channel in USER mode with uncompressed
 * semi-planar 4:2:0 -- every channel this file creates. The attribute keeps
 * the caller's width and height and the picture leaves as height x width for
 * 90 and 270, so the encoder bound to the channel has to be created with the
 * swapped geometry; rvd does that from the same config key that reaches here.
 *
 * Per channel, unlike the SigmaStar backends, so a rotated main stream and
 * an upright sub stream is a legal request. The VI channel has a rotation of
 * its own that would turn every stream at once; not used, because it would
 * also turn the sensor geometry rvd sizes everything from.
 */
int hal_fs_set_rotation(void *ctx, int chn, int degrees)
{
    int rot;
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    switch (degrees) {
    case 0:
        rot = V4_ROTATION_0;
        break;
    case 90:
        rot = V4_ROTATION_90;
        break;
    case 180:
        rot = V4_ROTATION_180;
        break;
    case 270:
        rot = V4_ROTATION_270;
        break;
    default:
        HAL_LOG_ERR("vpss chn %d: rotation %d is not 0, 90, 180 or 270", chn, degrees);
        return RSS_ERR_INVAL;
    }
    if (!st->vpss.fnSetChnRotation) {
        HAL_LOG_WARN("vpss chn %d: this libmpi has no HI_MPI_VPSS_SetChnRotation", chn);
        return RSS_ERR_NOTSUP;
    }
    if (!fs->configured) {
        HAL_LOG_ERR("vpss chn %d: rotation asked for before the channel exists", chn);
        return RSS_ERR_INVAL;
    }

    ret = st->vpss.fnSetChnRotation(HISI_VPSS_GRP, hisi_vpss_phy(chn), rot);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_SetChnRotation(%d, %d, %d deg) failed: 0x%x", HISI_VPSS_GRP, chn,
                    degrees, ret);
        return RSS_ERR_IO;
    }
    fs->rotation = degrees;
    if (degrees == 90 || degrees == 270)
        HAL_LOG_INFO("vpss chn %d: turned %d degrees; %ux%u in, %ux%u out", chn, degrees, fs->width,
                     fs->height, fs->height, fs->width);
    else
        HAL_LOG_INFO("vpss chn %d: turned %d degrees", chn, degrees);
    return RSS_OK;
}

/*
 * hal_fs_set_channel_attr -- change geometry or rate on a live channel.
 *
 * VPSS accepts SetChnAttr on an enabled channel, so this does not cycle it.
 * That matters for the fps case, which rvd issues while streaming: cycling
 * the channel would break the VENC bind for a frame or two and show up
 * downstream as a stall.
 */
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    v4_vpss_chn_attr attr;
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    if (!cfg)
        return RSS_ERR_INVAL;
    if (!st->vpss.fnSetChnAttr)
        return RSS_ERR_NOTSUP;
    if (!fs->configured)
        return hal_fs_create_channel(ctx, chn, cfg);

    {
        /* Restore on refusal: the tracked geometry is what fills the next
         * reconfigure, so a value the driver rejected must not survive to
         * be silently re-applied by a later successful call. */
        unsigned int old_w = fs->width, old_h = fs->height;
        v4_frame_rate old_fr = fs->frame_rate;

        if (cfg->width && cfg->height) {
            fs->width = cfg->width;
            fs->height = cfg->height;
        }
        hisi_fs_frame_rate(st, cfg->fps_num, cfg->fps_den, &fs->frame_rate);

        hisi_fs_fill_attr(st, fs, &attr);
        ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, hisi_vpss_phy(chn), &attr);
        if (ret) {
            HAL_LOG_ERR("HI_MPI_VPSS_SetChnAttr(%d, %d) %ux%u failed: 0x%x", HISI_VPSS_GRP, chn,
                        fs->width, fs->height, ret);
            fs->width = old_w;
            fs->height = old_h;
            fs->frame_rate = old_fr;
            return RSS_ERR_IO;
        }
    }

    /* The crop belongs to the geometry, so a live geometry change carries
     * it. Without this a channel reconfigured from cropped to uncropped
     * would keep scaling out of the old window. */
    ret = hisi_fs_apply_crop(st, chn, cfg);
    if (ret != RSS_OK)
        return ret;

    /* And so does the pool. rvd stops the stream around set-resolution, its
     * only caller that changes geometry, so the cycle this may do is inside
     * a gap that already exists. */
    hisi_fs_pool_refresh(st, chn, fs, cfg->nr_vbs);
    return RSS_OK;
}

int hal_fs_enable_channel(void *ctx, int chn)
{
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    if (!st->vpss.fnEnableChn)
        return RSS_ERR_NOTSUP;
    if (!fs->configured) {
        HAL_LOG_ERR("vpss chn %d: enable before create", chn);
        return RSS_ERR_INVAL;
    }
    if (fs->enabled)
        return RSS_OK;

    ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, hisi_vpss_phy(chn));
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_EnableChn(%d, %d) failed: 0x%x", HISI_VPSS_GRP, chn, ret);
        return RSS_ERR_IO;
    }

    fs->enabled = true;
    return RSS_OK;
}

int hal_fs_disable_channel(void *ctx, int chn)
{
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    if (!fs->enabled)
        return RSS_OK;
    if (!st->vpss.fnDisableChn)
        return RSS_ERR_NOTSUP;

    /* A frame checked out through fs_get_frame holds a VB block. Disabling
     * the channel under it strands the block, and /proc/umap/vb then shows
     * a pool that never recovers -- so give it back first. */
    if (fs->frame_held && st->vpss.fnReleaseChnFrame) {
        st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, hisi_vpss_phy(chn), &fs->frame);
        fs->frame_held = false;
    }

    ret = st->vpss.fnDisableChn(HISI_VPSS_GRP, hisi_vpss_phy(chn));
    if (ret)
        HAL_LOG_WARN("HI_MPI_VPSS_DisableChn(%d, %d) failed: 0x%x", HISI_VPSS_GRP, chn, ret);

    fs->enabled = false;
    return RSS_OK;
}

/*
 * hal_fs_destroy_channel -- there is no VPSS_DestroyChn.
 *
 * A channel is not an object on gen4; it is a slot that is either enabled or
 * not. So destroy is disable plus dropping raptor's own record, and saying
 * that here is better than publishing no op and having rvd believe the
 * channel is still live.
 */
int hal_fs_destroy_channel(void *ctx, int chn)
{
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    ret = hal_fs_disable_channel(ctx, chn);

    /* After the disable: a pool is detached from a channel that has stopped
     * asking it for blocks. */
    hisi_fs_pool_release(st, chn, fs);

    fs->configured = false;
    fs->width = 0;
    fs->height = 0;
    fs->depth = 0;

    return ret;
}

/* ================================================================
 * FRAME ACCESS
 * ================================================================ */

/*
 * hal_fs_set_frame_depth -- how many frames the channel queues for userspace.
 *
 * Real, not bookkeeping. A streaming channel runs at depth 0: it feeds its
 * bound VENC through the kernel and queues nothing, which is what keeps the
 * pipeline's memory footprint flat. fs_get_frame on such a channel would
 * block until its timeout, and the reason would be invisible -- the channel
 * is enabled, the group is running, frames are reaching the encoder.
 *
 * So raising the depth is how a caller asks for pictures, and this applies
 * it through SetChnAttr rather than tracking it, because the driver reads
 * u32Depth when the attribute is set and at no other time.
 */
int hal_fs_set_frame_depth(void *ctx, int chn, int depth)
{
    v4_vpss_chn_attr attr;
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    if (depth < 0 || depth > 8) {
        HAL_LOG_ERR("vpss chn %d: depth %d outside [0,8]", chn, depth);
        return RSS_ERR_INVAL;
    }
    if (!st->vpss.fnSetChnAttr)
        return RSS_ERR_NOTSUP;
    if ((unsigned int)depth == fs->depth)
        return RSS_OK;

    fs->depth = (unsigned int)depth;
    if (!fs->configured)
        return RSS_OK;

    hisi_fs_fill_attr(st, fs, &attr);
    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, hisi_vpss_phy(chn), &attr);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_SetChnAttr(%d, %d) depth %d failed: 0x%x", HISI_VPSS_GRP, chn,
                    depth, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

int hal_fs_get_frame_depth(void *ctx, int chn, int *depth)
{
    HISI_FS_ENTER(ctx, chn, st, fs);

    if (!depth)
        return RSS_ERR_INVAL;

    *depth = (int)fs->depth;
    return RSS_OK;
}

/*
 * hal_fs_get_frame -- check out one raw frame.
 *
 * The descriptor stays in the channel's state and the caller gets a pointer
 * to it, because HiMPP wants the same descriptor back at
 * ReleaseChnFrame -- there is no handle it can be reduced to.
 *
 * A 100 ms timeout rather than blocking forever: with depth 0 this call
 * would otherwise never return and the caller could not tell a
 * misconfigured channel from a stopped sensor. RSS_ERR_AGAIN on timeout is
 * what rvd's loops treat as "not this time" rather than as a fault.
 */
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info)
{
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    if (!frame_data)
        return RSS_ERR_INVAL;
    if (!st->vpss.fnGetChnFrame)
        return RSS_ERR_NOTSUP;
    if (!fs->enabled)
        return RSS_ERR_INVAL;
    if (fs->frame_held) {
        HAL_LOG_ERR("vpss chn %d: get_frame with a frame still held", chn);
        return RSS_ERR_BUSY;
    }
    if (!fs->depth) {
        HAL_LOG_ERR("vpss chn %d: get_frame at depth 0 -- raise it with fs_set_frame_depth "
                    "first, or the channel queues nothing for userspace",
                    chn);
        return RSS_ERR_INVAL;
    }

    memset(&fs->frame, 0, sizeof(fs->frame));
    ret = st->vpss.fnGetChnFrame(HISI_VPSS_GRP, hisi_vpss_phy(chn), &fs->frame, 100);
    if (ret) {
        /* Not logged: a timeout is the normal outcome of polling a channel
         * that has not produced yet, and logging it makes a working
         * pipeline look broken. */
        return -EAGAIN;
    }

    fs->frame_held = true;
    *frame_data = &fs->frame;

    if (info) {
        memset(info, 0, sizeof(*info));
        info->width = (uint16_t)fs->frame.frame.width;
        info->height = (uint16_t)fs->frame.frame.height;
        info->timestamp = (int64_t)fs->frame.frame.pts;
        /*
         * Plane 0 only. NV12 is two planes and rss_frame_info_t carries one
         * address, which is enough for the callers that exist: the luma
         * plane leads and the chroma plane follows it contiguously in the
         * same VB block, so a consumer that knows the format can find it.
         *
         * The addresses are 64-bit in the descriptor and 32-bit here. gen4
         * is a 32-bit part with at most 128 MB of DDR, so the truncation
         * cannot lose anything -- but it is a truncation, so it is written
         * as one rather than assigned across.
         */
        info->phys_addr = (uint32_t)fs->frame.frame.phy_addr[0];
        info->virt_addr = (void *)(uintptr_t)fs->frame.frame.vir_addr[0];
        info->size = fs->frame.frame.stride[0] * fs->frame.frame.height * 3u / 2u;
    }

    return RSS_OK;
}

int hal_fs_release_frame(void *ctx, int chn, void *frame_data)
{
    int ret;

    HISI_FS_ENTER(ctx, chn, st, fs);

    (void)frame_data;

    if (!fs->frame_held)
        return RSS_OK;
    if (!st->vpss.fnReleaseChnFrame)
        return RSS_ERR_NOTSUP;

    ret = st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, hisi_vpss_phy(chn), &fs->frame);
    if (ret)
        HAL_LOG_WARN("HI_MPI_VPSS_ReleaseChnFrame(%d, %d) failed: 0x%x", HISI_VPSS_GRP, chn, ret);

    fs->frame_held = false;
    memset(&fs->frame, 0, sizeof(fs->frame));
    return RSS_OK;
}

/*
 * hisi_fs_release_all -- give back every held frame and disable every
 * channel, before the group goes away.
 *
 * Called from hisi_teardown. Order matters within a channel -- frame, then
 * disable -- for the reason hal_fs_disable_channel gives; across channels
 * it does not.
 */
void hisi_fs_release_all(hisi_state_t *st)
{
    int i;

    if (!st)
        return;

    for (i = 0; i < HISI_FS_CHN_NUM; i++) {
        hisi_vpss_chn_t *fs = &st->fs[i];

        if (fs->frame_held && st->vpss.fnReleaseChnFrame) {
            st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, hisi_vpss_phy(i), &fs->frame);
            fs->frame_held = false;
        }
        if (fs->enabled && st->vpss.fnDisableChn) {
            int ret = st->vpss.fnDisableChn(HISI_VPSS_GRP, hisi_vpss_phy(i));
            if (ret)
                HAL_LOG_WARN("HI_MPI_VPSS_DisableChn(%d, %d) failed: 0x%x", HISI_VPSS_GRP, i, ret);
        }
        fs->enabled = false;
        /* Before VB_Exit, which is the point: a pool still alive when the
         * module goes down is the CMA-stuck-until-reload failure. */
        hisi_fs_pool_release(st, i, fs);
        fs->configured = false;
    }
}

/* hal_isp_get_sensor_attr lived here through Phase 2; Phase 3 moved it to
 * hal_isp.c with the rest of the ISP surface, as the note that used to sit
 * in its place promised. */
