/*
 * hisi_v5/hal_framesource.c -- VPSS channels, HiMPP V5.0
 *
 * A raptor framesource is one VPSS channel. The group is the pipeline's
 * and belongs to hal_common.c; everything per-stream -- geometry, frame
 * rate, rotation, the userspace frame queue -- is here.
 *
 * WHAT A CHANNEL IS ON V5, and the one place it differs from gen4:
 * physical channel 0 is usable. gen4 reserves it because HI_MPI_SYS_Bind
 * forces a VPSS destination's channel to 0, so the group's input occupies
 * it; V5 numbers the group's input port and the output channels
 * separately, and the vendor's own sample binds VPSS channel 0 to VENC
 * channel 0. See HISI_VPSS_CHN_BASE.
 *
 * THERE IS NO DestroyChn. ss_mpi_vpss_disable_chn is the whole of channel
 * teardown; the channel keeps its attribute and comes back with
 * enable_chn. So hal_fs_destroy_channel disables and forgets, and a later
 * create writes the attribute again rather than assuming what is there.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <stdio.h>

/*
 * The channel index rvd uses, checked and turned into the physical one.
 *
 * Every op starts here, so the range check is written once. Returning the
 * state rather than a bool lets the caller do both in one line.
 */
static hisi_vpss_chn_t *hisi_fs_chn(void *ctx, int chn)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || chn < 0 || chn >= HISI_VPSS_CHN_NUM)
        return NULL;
    return &st->fs[chn];
}

/*
 * hisi_fs_frame_rate -- rvd's fps_num/fps_den into ot_frame_rate_ctrl.
 *
 * The vendor's pair is (source rate, destination rate) in whole frames,
 * with -1 meaning "do not control". So a stream at the sensor's own rate
 * asks for no control at all rather than for src == dst: the latter is
 * accepted and makes VPSS run a divider that always passes, which is a
 * frame-timing decision taken for no reason.
 *
 * A non-integer rate is rounded up, not down. Rounding down turns 29.97
 * into 29 and quietly drops a frame a second on a stream whose whole point
 * was to match a 30 fps source.
 */
static void hisi_fs_frame_rate(const hisi_state_t *st, uint32_t num, uint32_t den,
                               v5_frame_rate_ctrl *out)
{
    unsigned int src = (unsigned int)(st->mode.frame_rate + 0.5f);
    unsigned int dst;

    out->src_frame_rate = -1;
    out->dst_frame_rate = -1;

    if (!num || !den || !src)
        return;

    dst = (num + den - 1u) / den;
    if (!dst || dst >= src)
        return;

    out->src_frame_rate = (int)src;
    out->dst_frame_rate = (int)dst;
}

/*
 * hisi_fs_fill_attr -- the channel attribute from raptor's config.
 *
 * USER mode, always. AUTO takes the channel's geometry from the group,
 * which would give three streams at the sensor's size and no error to say
 * so.
 *
 * YVU_SEMIPLANAR_420 (NV21) matches the group and the VI channel; the
 * neighbouring YUV_SEMIPLANAR_420 (NV12) differs only in chroma order, so
 * choosing wrong costs swapped colours rather than a failure.
 */
static void hisi_fs_fill_attr(const hisi_state_t *st, const hisi_vpss_chn_t *fs,
                              v5_vpss_chn_attr *attr)
{
    memset(attr, 0, sizeof(*attr));

    attr->width = fs->width;
    attr->height = fs->height;
    attr->depth = fs->depth;
    attr->chn_mode = V5_VPSS_CHN_MODE_USER;
    attr->video_format = V5_VIDEO_FORMAT_LINEAR;
    attr->dynamic_range = V5_DYNAMIC_RANGE_SDR8;
    attr->pixel_format = V5_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    attr->compress_mode = V5_COMPRESS_MODE_NONE;
    attr->frame_rate = fs->frame_rate;
    attr->border_en = 0;
    attr->aspect_ratio.mode = V5_ASPECT_RATIO_NONE;

    /*
     * Mirror and flip are the *channel's* on V5, not the sensor's -- every
     * sensor library on this image has a null pfn_mirror_flip. They are
     * per-stream here, which is what raptor wants, and they follow the
     * backend's orientation state rather than the caller's config.
     */
    attr->mirror_en = 0;
    attr->flip_en = 0;

    (void)st;
}

/*
 * hal_fs_create_channel -- configure and enable one VPSS channel.
 *
 * Enable is part of create rather than only of hal_fs_enable_channel, and
 * that is deliberate: rvd's pipeline calls create then bind then start,
 * and never calls enable for a stream it is about to bind. A channel that
 * is configured but not enabled produces nothing, with the group running
 * and /proc/umap/vpss showing the channel at 0x0.
 */
int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    v5_vpss_chn_attr attr;
    int phy;
    int ret;

    if (!st || !fs || !cfg)
        return RSS_ERR_INVAL;
    if (!st->vpss_grp_created) {
        HAL_LOG_ERR("fs%d: VPSS group %d is not up", chn, HISI_VPSS_GRP);
        return RSS_ERR_NOENT;
    }
    if (!cfg->width || !cfg->height) {
        HAL_LOG_ERR("fs%d: %ux%u is not a size", chn, cfg->width, cfg->height);
        return RSS_ERR_INVAL;
    }
    if (cfg->width > st->mode.dev_rect.width || cfg->height > st->mode.dev_rect.height) {
        /*
         * VPSS scales down, not up: the group was created for the sensor's
         * output and a channel cannot ask for more. Rejecting here rather
         * than letting set_chn_attr fail makes the message name both
         * numbers.
         */
        HAL_LOG_ERR("fs%d: %ux%u is larger than the group's %ux%u", chn, cfg->width, cfg->height,
                    st->mode.dev_rect.width, st->mode.dev_rect.height);
        return RSS_ERR_INVAL;
    }

    fs->width = cfg->width;
    fs->height = cfg->height;
    hisi_fs_frame_rate(st, cfg->fps_num, cfg->fps_den, &fs->frame_rate);
    hisi_fs_fill_attr(st, fs, &attr);

    phy = hisi_vpss_phy(chn);
    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, &attr);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_set_chn_attr(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }
    fs->configured = true;

    ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, phy);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_enable_chn(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy, ret);
        return RSS_ERR_IO;
    }
    fs->enabled = true;

    HAL_LOG_INFO("fs%d: VPSS %d/%d %ux%u, rate %d/%d, depth %u", chn, HISI_VPSS_GRP, phy, fs->width,
                 fs->height, fs->frame_rate.src_frame_rate, fs->frame_rate.dst_frame_rate,
                 fs->depth);
    return RSS_OK;
}

/*
 * hal_fs_set_channel_attr -- geometry or rate on a live channel.
 *
 * VPSS takes a new channel attribute while the group is running, which is
 * what makes a resolution change cheap here: no destroy, no rebind, and
 * the encoder on the far side of the bind sees a differently sized frame
 * on its next one.
 */
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    v5_vpss_chn_attr attr;
    int phy;
    int ret;

    if (!st || !fs || !cfg)
        return RSS_ERR_INVAL;
    if (!fs->configured)
        return hal_fs_create_channel(ctx, chn, cfg);
    if (!cfg->width || !cfg->height)
        return RSS_ERR_INVAL;
    if (cfg->width > st->mode.dev_rect.width || cfg->height > st->mode.dev_rect.height)
        return RSS_ERR_INVAL;

    fs->width = cfg->width;
    fs->height = cfg->height;
    hisi_fs_frame_rate(st, cfg->fps_num, cfg->fps_den, &fs->frame_rate);
    hisi_fs_fill_attr(st, fs, &attr);

    phy = hisi_vpss_phy(chn);
    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, &attr);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_set_chn_attr(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_INFO("fs%d: now %ux%u, rate %d/%d", chn, fs->width, fs->height,
                 fs->frame_rate.src_frame_rate, fs->frame_rate.dst_frame_rate);
    return RSS_OK;
}

int hal_fs_enable_channel(void *ctx, int chn)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (!fs->configured)
        return RSS_ERR_NOENT;
    if (fs->enabled)
        return RSS_OK;

    phy = hisi_vpss_phy(chn);
    ret = st->vpss.fnEnableChn(HISI_VPSS_GRP, phy);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_enable_chn(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy, ret);
        return RSS_ERR_IO;
    }
    fs->enabled = true;
    return RSS_OK;
}

/*
 * hal_fs_disable_channel -- stop delivering, keep the attribute.
 *
 * A frame checked out through fs_get_frame is released first. Disabling a
 * channel with one outstanding leaves the block accounted to a channel
 * that no longer runs, and VB then reports it as in use until the group is
 * destroyed.
 */
int hal_fs_disable_channel(void *ctx, int chn)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (!fs->enabled)
        return RSS_OK;

    phy = hisi_vpss_phy(chn);

    if (fs->frame_held) {
        if (st->vpss.fnReleaseChnFrame)
            st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, phy, &fs->frame);
        fs->frame_held = false;
    }

    ret = st->vpss.fnDisableChn(HISI_VPSS_GRP, phy);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_disable_chn(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }
    fs->enabled = false;
    return RSS_OK;
}

/*
 * hal_fs_destroy_channel -- there is no ss_mpi_vpss_destroy_chn.
 *
 * Disable is the whole of it. The bookkeeping is cleared so a later create
 * writes the attribute again rather than trusting what the driver still
 * holds.
 */
int hal_fs_destroy_channel(void *ctx, int chn)
{
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int ret;

    if (!fs)
        return RSS_ERR_INVAL;

    ret = hal_fs_disable_channel(ctx, chn);
    if (ret)
        return ret;

    memset(fs, 0, sizeof(*fs));
    return RSS_OK;
}

/*
 * hal_fs_set_rotation -- turn one channel's output.
 *
 * ss_mpi_vpss_set_chn_rotation takes an ot_rotation_attr the backend does
 * not transcribe (Phase 3 owns rotation properly), so the op reports what
 * it can do rather than pretending: 0 is accepted because it is the state
 * the channel is already in, and anything else is RSS_ERR_NOTSUP with a
 * line saying why.
 */
int hal_fs_set_rotation(void *ctx, int chn, int degrees)
{
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);

    if (!fs)
        return RSS_ERR_INVAL;
    if (degrees == 0) {
        fs->rotation = 0;
        return RSS_OK;
    }

    HAL_LOG_WARN("fs%d: rotation %d needs ot_rotation_attr, which this backend does not yet "
                 "transcribe",
                 chn, degrees);
    return RSS_ERR_NOTSUP;
}

/*
 * hal_fs_set_frame_depth -- how many frames the channel queues for us.
 *
 * Zero is the streaming case: the channel feeds its bound VENC and queues
 * nothing. rvd raises it to take a snapshot by hand, and with depth 0
 * get_chn_frame would block to its timeout on a channel that is working
 * perfectly -- which is why this is a real op and not bookkeeping.
 *
 * Every frame of depth is a frame of VB held by this channel, so the
 * bound is the vendor's 8 and the default stays 0.
 */
#define HISI_FS_MAX_DEPTH 8

int hal_fs_set_frame_depth(void *ctx, int chn, int depth)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    v5_vpss_chn_attr attr;
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (depth < 0 || depth > HISI_FS_MAX_DEPTH)
        return RSS_ERR_INVAL;
    if ((unsigned int)depth == fs->depth)
        return RSS_OK;

    fs->depth = (unsigned int)depth;

    if (!fs->configured)
        return RSS_OK;

    hisi_fs_fill_attr(st, fs, &attr);
    phy = hisi_vpss_phy(chn);
    ret = st->vpss.fnSetChnAttr(HISI_VPSS_GRP, phy, &attr);
    if (ret) {
        HAL_LOG_ERR("fs%d: depth %d rejected: 0x%x", chn, depth, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

int hal_fs_get_frame_depth(void *ctx, int chn, int *depth)
{
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);

    if (!fs || !depth)
        return RSS_ERR_INVAL;
    *depth = (int)fs->depth;
    return RSS_OK;
}

/*
 * hal_fs_get_frame -- check out one YUV frame.
 *
 * MPP wants the same descriptor back, so it lives in the channel's state
 * and the caller is handed a pointer into it. One outstanding frame per
 * channel: rvd's snapshot path takes one, uses it and gives it back, and a
 * second concurrent take would need a second descriptor for no caller that
 * exists.
 */
#define HISI_FS_FRAME_TIMEOUT_MS 2000

int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs || !frame_data)
        return RSS_ERR_INVAL;
    if (!st->vpss.fnGetChnFrame)
        return RSS_ERR_NOTSUP;
    if (!fs->enabled)
        return RSS_ERR_NOENT;
    if (fs->frame_held) {
        HAL_LOG_ERR("fs%d: a frame is already checked out", chn);
        return RSS_ERR_BUSY;
    }
    if (!fs->depth) {
        HAL_LOG_ERR("fs%d: depth is 0, so no frame is ever queued for userspace -- raise it with "
                    "fs_set_frame_depth first",
                    chn);
        return RSS_ERR_INVAL;
    }

    phy = hisi_vpss_phy(chn);
    memset(&fs->frame, 0, sizeof(fs->frame));
    ret = st->vpss.fnGetChnFrame(HISI_VPSS_GRP, phy, &fs->frame, HISI_FS_FRAME_TIMEOUT_MS);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_get_chn_frame(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP, phy,
                    ret);
        return RSS_ERR_IO;
    }
    fs->frame_held = true;

    *frame_data = &fs->frame;
    if (info) {
        memset(info, 0, sizeof(*info));
        info->width = fs->frame.video_frame.width;
        info->height = fs->frame.video_frame.height;
        info->timestamp = (int64_t)fs->frame.video_frame.pts;
    }

    return RSS_OK;
}

int hal_fs_release_frame(void *ctx, int chn, void *frame_data)
{
    hisi_state_t *st = hisi_state(ctx);
    hisi_vpss_chn_t *fs = hisi_fs_chn(ctx, chn);
    int phy;
    int ret;

    if (!st || !fs)
        return RSS_ERR_INVAL;
    if (!fs->frame_held)
        return RSS_OK;
    /* The pointer is the caller's proof it holds this channel's frame; a
     * mismatch is a caller bug and releasing anyway would hide it. */
    if (frame_data && frame_data != &fs->frame)
        return RSS_ERR_INVAL;

    phy = hisi_vpss_phy(chn);
    ret =
        st->vpss.fnReleaseChnFrame ? st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, phy, &fs->frame) : 0;
    fs->frame_held = false;

    if (ret) {
        HAL_LOG_ERR("ss_mpi_vpss_release_chn_frame(grp %d, chn %d) failed: 0x%x", HISI_VPSS_GRP,
                    phy, ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

/*
 * hisi_fs_release_all -- teardown's entry point.
 *
 * Called from the pipeline teardown before the group is stopped, because
 * ss_mpi_vpss_stop_grp with a channel still enabled returns busy and the
 * whole teardown then unwinds in the wrong order.
 */
void hisi_fs_release_all(hisi_state_t *st)
{
    int chn;

    if (!st)
        return;

    for (chn = 0; chn < HISI_VPSS_CHN_NUM; chn++) {
        hisi_vpss_chn_t *fs = &st->fs[chn];
        int phy = hisi_vpss_phy(chn);

        if (fs->frame_held) {
            if (st->vpss.fnReleaseChnFrame)
                st->vpss.fnReleaseChnFrame(HISI_VPSS_GRP, phy, &fs->frame);
            fs->frame_held = false;
        }
        if (fs->enabled && st->vpss.fnDisableChn)
            st->vpss.fnDisableChn(HISI_VPSS_GRP, phy);
        memset(fs, 0, sizeof(*fs));
    }
}
