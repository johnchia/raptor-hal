/*
 * infinity6c/hal_common.c -- Raptor HAL common layer, SigmaStar MI 3.0 backend
 *
 * Counterpart to star/hal_common.c (MI 2.x) and src/hal_common.c (Ingenic
 * IMP). Provides the factory functions, the ops vtable and the logging hook
 * for Infinity6C parts.
 *
 * Why a third backend rather than guards inside star/: the two MI generations
 * do not differ by degree. MI_SYS and MI_RGN take a leading SoC id, MI_VENC a
 * leading device, the ISP is a pipeline stage with its own device, channel and
 * ports rather than a set of tuning calls, and SCL holds the scaling role VPE
 * had. Struct layouts differ even where a signature does not. Sharing files
 * would mean two disjoint implementations behind mutually exclusive guards.
 *
 * Scope: the capture and encode datapath, ISP tuning, OSD, and audio input. The
 * vtable publishes only what is implemented -- RSS_HAL_CALL NULL-guards every
 * entry and returns RSS_ERR_NOTSUP, so an absent subsystem needs no stub and no
 * file. hal_caps.c declares the matching zeroes. What is still absent: raw frame
 * readback, IVS, and the whole audio *output* half.
 *
 * That has one exception, and it shapes this file: an op whose failure rvd
 * treats as fatal cannot be absent, however little the hardware has to do for
 * it. The encoder group ops are published for that reason alone, and hal_bind
 * collapses a stage this generation does not have rather than refusing it.
 *
 * dlsym resolves by name, so an MI 2.x table binds against these libraries
 * without complaint and then calls MI_SYS_Init with a stray value where the SoC
 * id belongs. Nothing reports that, on either side -- which is why the loaders
 * spell out every argument list rather than sharing one with star/.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "infinity6c_state.h"

#include <stdarg.h>

/* ================================================================
 * LOGGING
 *
 * Mirrors star/hal_common.c: log through a function pointer that
 * defaults to stderr, which daemons redirect to syslog at init.
 * ================================================================ */

static const char *hal_level_str[] = {"FTL", "ERR", "WRN", "INF", "DBG"};

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
{
    if (level < 0)
        level = 0;
    if (level > 4)
        level = 4;
    const char *basename = strrchr(file, '/');
    if (basename)
        file = basename + 1;
    fprintf(stderr, "[HAL %s] %s:%d: ", hal_level_str[level], file, line);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

rss_hal_log_func_t rss_hal_log_fn = hal_log_stderr;

void rss_hal_set_log_func(rss_hal_log_func_t func)
{
    rss_hal_log_fn = func ? func : hal_log_stderr;
}

/* ── Per-SoC capability data (src/hal_caps.c, compiled per platform) ── */

extern const rss_hal_caps_t g_hal_caps;

/*
 * The daemons call rss_hal_get_imp_version() and friends with no context
 * argument, so keep a pointer to the live state for them to reach. Set at
 * init, cleared at deinit; one HAL context per process is assumed throughout
 * raptor.
 */
static infinity6c_state_t *g_infinity6c;

/* ── GPIO / IR-cut (src/hal_gpio.c — plain sysfs, no SDK dependency) ── */

#ifdef HAL_MODULE_VIDEO
int hal_gpio_set(void *ctx, int pin, int value);
int hal_gpio_get(void *ctx, int pin, int *value);
int hal_ircut_set(void *ctx, int state);

/* ── Framesource: SNR, VIF, ISP and SCL (infinity6c/hal_framesource.c) ── */

int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_destroy_channel(void *ctx, int chn);
int hal_fs_enable_channel(void *ctx, int chn);
int hal_fs_disable_channel(void *ctx, int chn);
int hal_fs_set_rotation(void *ctx, int chn, int degrees);

/* ── Encoder: VENC (infinity6c/hal_encoder.c) ── */

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg);
int hal_enc_destroy_channel(void *ctx, int chn);
int hal_enc_create_group(void *ctx, int grp);
int hal_enc_destroy_group(void *ctx, int grp);
int hal_enc_register_channel(void *ctx, int grp, int chn);
int hal_enc_unregister_channel(void *ctx, int chn);
int hal_enc_start(void *ctx, int chn);
int hal_enc_stop(void *ctx, int chn);
int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms);
int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_request_idr(void *ctx, int chn);
int hal_enc_get_fd(void *ctx, int chn);
int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den);
int hal_enc_query(void *ctx, int chn, bool *busy);

/* ── ISP tuning (infinity6c/hal_isp.c) ── */

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height);
int hal_isp_set_brightness(void *ctx, int val);
int hal_isp_get_brightness(void *ctx, int *val);
int hal_isp_set_contrast(void *ctx, int val);
int hal_isp_get_contrast(void *ctx, int *val);
int hal_isp_set_saturation(void *ctx, int val);
int hal_isp_get_saturation(void *ctx, int *val);
int hal_isp_set_drc_strength(void *ctx, int val);
int hal_isp_get_drc_strength(void *ctx, int *val);
int hal_isp_set_sharpness(void *ctx, int val);
int hal_isp_get_sharpness(void *ctx, int *val);
int hal_isp_set_defog(void *ctx, int enable);
int hal_isp_set_defog_strength(void *ctx, int val);
int hal_isp_set_defog_strength_adv(void *ctx, const void *defog_attr);
int hal_isp_get_defog_strength(void *ctx, int *val);
int hal_isp_set_ae_comp(void *ctx, int val);
int hal_isp_get_ae_comp(void *ctx, int *val);
int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode);
int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode);
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode);
int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode);
int hal_isp_set_hflip(void *ctx, int enable);
int hal_isp_set_vflip(void *ctx, int enable);
int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip);
int hal_isp_set_temper_strength(void *ctx, int val);
int hal_isp_get_temper_strength(void *ctx, int *val);
int hal_isp_get_knob_caps(void *ctx, const char *name, rss_isp_knob_t *caps);
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);

/* ── OSD: MI_RGN (infinity6c/hal_osd.c) ── */

int hal_osd_set_pool_size(void *ctx, uint32_t bytes);
int hal_osd_create_group(void *ctx, int grp);
int hal_osd_destroy_group(void *ctx, int grp);
int hal_osd_start(void *ctx, int grp);
int hal_osd_stop(void *ctx, int grp);
int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr);
int hal_osd_destroy_region(void *ctx, int handle);
int hal_osd_register_region(void *ctx, int handle, int grp);
int hal_osd_unregister_region(void *ctx, int handle, int grp);
int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr);
int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data);
int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer);
#endif

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

/*
 * i6c_unload_all -- drop every loaded module.
 *
 * Reverse of the load order, and safe on a partly loaded set: each unloader
 * checks its own handle, so this is also the failure path for a load that got
 * halfway.
 */
static void i6c_unload_all(infinity6c_state_t *st)
{
#ifdef HAL_MODULE_VIDEO
    i6c_venc_unload(&st->venc);
    i6c_scl_unload(&st->scl);
    /* Before the handle goes: the tuning table caches symbols out of it. */
    i6c_isp_forget_knobs();
    i6c_isp_unload(&st->isp);
    i6c_vif_unload(&st->vif);
    i6c_snr_unload(&st->snr);
#endif
    i6c_sys_unload(&st->sys);
}

/*
 * hal_init -- load the MI modules and bring MI_SYS up.
 *
 * The sensor configuration is still accepted and ignored. Sensor selection is
 * the vendor driver's, not this HAL's: MI_SNR publishes what the loaded driver
 * supports and bring-up chooses from that list, so there is nothing here for a
 * caller-supplied sensor name to select. The argument stays so the op signature
 * does not move if that ever stops being true.
 *
 * No pipeline is built here. The datapath's geometry comes from the first
 * framesource channel to be created, and building it at init would mean choosing
 * a resolution before anything has asked for one.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;
    int ret;

    (void)cfg;

    if (!hal)
        return RSS_ERR_INVAL;
    if (hal->platform)
        return RSS_ERR_BUSY;

    st = (infinity6c_state_t *)calloc(1, sizeof(*st));
    if (!st)
        return RSS_ERR_NOMEM;

    /*
     * -1 rather than the calloc'd zero: zero is a valid sensor mode, a valid SCL
     * port and a valid encoder channel, so "unset" needs a value of its own.
     */
    st->snr_profile = -1;
    st->scl_video_port = -1;
    /* Not the calloc'd zero either: the flip predicates are gated on 3DNR. */
    st->isp_nr3d_req = 1;
    for (int i = 0; i < I6C_MAX_CHN; i++) {
        st->osd_src_port[i] = -1;
        st->enc[i].src_port = -1;
        st->enc[i].cascade_src = -1;
        st->enc[i].fd = -1;
    }
    for (int i = 0; i < I6C_VENC_DEV_SLOTS; i++)
        st->enc_ring_chn[i] = -1;

    if ((ret = i6c_sys_load(&st->sys)) != RSS_OK) {
        i6c_unload_all(st);
        free(st);
        return ret;
    }

    /*
     * First call with the new argument list, and the reason this stage exists.
     * A wrong arity reaches MI as a stray SoC id rather than as an error, so
     * the interesting outcomes are a non-zero return here or a version string
     * that does not read like one.
     */
    if ((ret = st->sys.init(I6C_SOC_ID)) != 0) {
        HAL_LOG_ERR("infinity6c: MI_SYS_Init(soc %d) failed: %d", I6C_SOC_ID, ret);
        i6c_unload_all(st);
        free(st);
        return RSS_ERR_IO;
    }
    st->sys_inited = true;

    /*
     * The datapath modules, and only in the video archive -- the audio one has
     * no use for the scaler or the encoder and should not be holding their
     * libraries open.
     *
     * Loaded after MI_SYS is up rather than alongside it, because a failure here
     * has MI_SYS_Exit to owe and st->sys_inited is what records that.
     */
#ifdef HAL_MODULE_VIDEO
    if ((ret = i6c_snr_load(&st->snr)) != RSS_OK)
        goto fail_modules;
    if ((ret = i6c_vif_load(&st->vif)) != RSS_OK)
        goto fail_modules;
    if ((ret = i6c_isp_load(&st->isp)) != RSS_OK)
        goto fail_modules;
    if ((ret = i6c_scl_load(&st->scl)) != RSS_OK)
        goto fail_modules;
    if ((ret = i6c_venc_load(&st->venc)) != RSS_OK)
        goto fail_modules;
#endif

    /*
     * Logged at info rather than debug: until a pipeline exists this string is
     * the whole observable result of a bring-up run, and it is what says the
     * loaded libraries are the drop the headers were transcribed from.
     */
    {
        i6c_sys_ver ver;

        memset(&ver, 0, sizeof(ver));
        if (st->sys.get_version(I6C_SOC_ID, &ver) == 0)
            HAL_LOG_INFO("infinity6c: MI %.*s", (int)sizeof(ver.version), (char *)ver.version);
        else
            HAL_LOG_WARN("infinity6c: MI_SYS_GetVersion failed; MI is up but unidentified");
    }

    hal->platform = st;
    g_infinity6c = st;

#ifdef HAL_MODULE_VIDEO
    HAL_LOG_INFO("infinity6c: %s up, SNR/VIF/ISP/SCL/VENC loaded; pipeline builds on first channel",
                 HAL_PLATFORM_NAME);
#else
    /*
     * MI_SYS only: libmi_ai is loaded by audio_init rather than here, since a
     * caller that never asks for audio should not hold it open -- and rad reaches
     * audio_init without calling this op at all.
     */
    HAL_LOG_INFO("infinity6c: %s up, MI_SYS only; MI_AI loads on audio_init", HAL_PLATFORM_NAME);
#endif
    return RSS_OK;

#ifdef HAL_MODULE_VIDEO
fail_modules:
    st->sys.exit(I6C_SOC_ID);
    st->sys_inited = false;
    i6c_unload_all(st);
    free(st);
    return ret;
#endif
}

static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;

    if (!hal)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)hal->platform;
    if (!st)
        return RSS_OK;

    /*
     * Before MI_SYS goes: the encoder channels and the pipeline hold MI objects
     * that outlive a daemon killed by a signal, and MI_SYS_Exit with a bound
     * channel still up leaves the modules loaded with state nothing owns.
     */
#ifdef HAL_MODULE_VIDEO
    i6c_teardown_all(st);
#endif

    if (st->sys_inited) {
        int ret = st->sys.exit(I6C_SOC_ID);

        /*
         * Reported but not propagated. There is nothing a caller can retry,
         * and the libraries are about to be unloaded either way.
         */
        if (ret != 0)
            HAL_LOG_WARN("infinity6c: MI_SYS_Exit(soc %d) failed: %d", I6C_SOC_ID, ret);
        st->sys_inited = false;
    }

    i6c_unload_all(st);

    if (g_infinity6c == st)
        g_infinity6c = NULL;
    hal->platform = NULL;
    free(st);

    return RSS_OK;
}

/*
 * The per-context copy rather than the static one, matching the other two
 * backends: a caller holding a context expects the caps that belong to it, and a
 * NULL context has none to report.
 */
static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    if (!c)
        return NULL;

    return &c->caps;
}

#ifdef HAL_MODULE_VIDEO

/* ================================================================
 * BIND
 *
 * rvd builds its pipeline as a chain of cells: FS [-> IVS] [-> OSD]
 * -> ENC. Here that chain is a single link, SCL output port -> VENC
 * channel. There is no IVS stage and no OSD stage in the data path --
 * MI_RGN composites onto a port in place rather than sitting between
 * two modules.
 *
 * The OSD cell still must not be rejected. rvd inserts it on
 * `[osd] enabled` alone, which defaults on, and it treats a failed
 * bind step as fatal to the stream -- so refusing the stage would cost
 * the video, not the overlay. It is collapsed instead:
 *
 *   FS  -> OSD   remember which SCL port feeds this encoder
 *   OSD -> ENC   perform the real bind, using the remembered port
 *
 * Unlike star/, nothing then attaches an overlay to that port: this
 * backend publishes no osd_* ops, so rvd's region calls fail on their
 * own and the stream carries no overlay. The stage is collapsed to keep
 * the pipeline up, not to suggest the overlay works.
 *
 * IVS cannot appear -- no ivs_* ops are published, so rvd never sets
 * ivs_active -- and is refused by name rather than silently ignored.
 * ================================================================ */

static int i6c_bind_collapse(infinity6c_state_t *st, const rss_cell_t *src, const rss_cell_t *dst,
                             int *port, int *chn, bool *collapsed)
{
    *collapsed = false;

    if (!src || !dst)
        return RSS_ERR_INVAL;

    /* The direct chain, with OSD disabled. */
    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_ENC) {
        *port = src->group;
        *chn = dst->group;
        return RSS_OK;
    }

    /*
     * First half of an OSD chain. Nothing to bind yet -- record the SCL port so
     * the second half can name it, since rvd does not repeat it there.
     */
    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_OSD) {
        if (dst->group < 0 || dst->group >= I6C_MAX_CHN)
            return RSS_ERR_INVAL;

        st->osd_src_port[dst->group] = src->group;
        *collapsed = true;
        return RSS_OK;
    }

    /* Second half: the bind rvd actually asked for. */
    if (src->device == RSS_DEV_OSD && dst->device == RSS_DEV_ENC) {
        if (src->group < 0 || src->group >= I6C_MAX_CHN)
            return RSS_ERR_INVAL;

        *port = st->osd_src_port[src->group];
        *chn = dst->group;

        if (*port < 0) {
            HAL_LOG_ERR("bind: OSD %d -> ENC %d without a preceding FS -> OSD", src->group,
                        dst->group);
            return RSS_ERR_INVAL;
        }
        return RSS_OK;
    }

    HAL_LOG_ERR("bind: FS -> [OSD ->] ENC is the only chain this backend supports (got %d -> %d)",
                (int)src->device, (int)dst->device);

    return RSS_ERR_NOTSUP;
}

static int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;
    bool collapsed;
    int port = -1;
    int chn = -1;
    int ret;

    if (!hal)
        return RSS_ERR_INVAL;
    st = (infinity6c_state_t *)hal->platform;
    if (!st)
        return RSS_ERR_NOTSUP;

    if ((ret = i6c_bind_collapse(st, src, dst, &port, &chn, &collapsed)) != RSS_OK)
        return ret;
    if (collapsed)
        return RSS_OK; /* Recorded; the OSD -> ENC step does the work. */

    if (port < 0 || port >= I6C_MAX_CHN || chn < 0 || chn >= I6C_MAX_CHN)
        return RSS_ERR_INVAL;
    if (!st->enc[chn].created)
        return RSS_ERR_NOENT;

    /*
     * The source end of this bind is an SCL port, which does not exist until the
     * chain does -- and the bind carries st->fps, which nothing has read from the
     * sensor yet. Refused here so the log names the cause, rather than in MI as a
     * code against a channel that was never created.
     */
    if (!st->pipeline_up) {
        HAL_LOG_ERR("bind: SCL port %d -> venc chn %d before the pipeline is up", port, chn);
        return RSS_ERR_INVAL;
    }

    /* Zero: a video stream is consumed at the rate the pipeline produces. */
    return i6c_bind_scl_to_venc(st, port, chn, 0);
}

static int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;
    bool collapsed;
    int port = -1;
    int chn = -1;
    int ret;

    if (!hal)
        return RSS_ERR_INVAL;
    st = (infinity6c_state_t *)hal->platform;
    if (!st)
        return RSS_ERR_NOTSUP;

    if ((ret = i6c_bind_collapse(st, src, dst, &port, &chn, &collapsed)) != RSS_OK)
        return ret;
    if (collapsed)
        return RSS_OK; /* FS -> OSD bound nothing, so it unbinds nothing. */

    if (chn < 0 || chn >= I6C_MAX_CHN)
        return RSS_ERR_INVAL;

    return i6c_unbind_scl_from_venc(st, chn);
}

#endif /* HAL_MODULE_VIDEO */

/* ================================================================
 * SYSTEM INFO
 * ================================================================ */

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;
    i6c_sys_ver ver;

    if (!hal || !buf || len <= 0)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)hal->platform;
    if (!st || !st->sys.get_version)
        return RSS_ERR_NOTSUP;

    memset(&ver, 0, sizeof(ver));
    if (st->sys.get_version(I6C_SOC_ID, &ver))
        return RSS_ERR_IO;

    /* version[] is not guaranteed terminated; bound the copy by both sizes. */
    snprintf(buf, (size_t)len, "%.*s", (int)sizeof(ver.version), (char *)ver.version);
    return RSS_OK;
}

/*
 * The media clock.
 *
 * rvd samples this twice around a wall-clock read and publishes the mapping in
 * the ring, which is what lets a consumer stamp a frame with an absolute capture
 * time -- the SEI timecodes rest on it. Without the op the mapping is simply
 * never published: rvd's publish_utc_mapping returns early and the stream is
 * fine, just not locatable in wall time.
 *
 * Cheap enough to be called on a timer: the wrapper is one ioctl, and rvd
 * re-samples every five seconds and rejects a pair that straddled a preemption.
 */
static int hal_sys_get_timestamp(void *ctx, int64_t *ts)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;
    unsigned long long pts = 0;

    if (!hal || !ts)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)hal->platform;
    if (!st || !st->sys.get_cur_pts)
        return RSS_ERR_NOTSUP;

    if (st->sys.get_cur_pts(I6C_SOC_ID, &pts))
        return RSS_ERR_IO;

    *ts = (int64_t)pts;
    return RSS_OK;
}

/*
 * Move the media clock's origin.
 *
 * MI_SYS_InitPtsBase rather than MI_SYS_SyncPts, as on Infinity6E: the base call
 * sets where the clock counts from, which is what a rebase means, while SyncPts
 * nudges a running clock toward a value and is for keeping two devices together.
 */
static int hal_sys_rebase_timestamp(void *ctx, int64_t base)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;

    if (!hal)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)hal->platform;
    if (!st || !st->sys.init_pts_base)
        return RSS_ERR_NOTSUP;

    if (st->sys.init_pts_base(I6C_SOC_ID, (unsigned long long)base))
        return RSS_ERR_IO;

    return RSS_OK;
}

/* ================================================================
 * OPS VTABLE
 *
 * Only implemented ops are listed. Everything else stays NULL and
 * resolves to RSS_ERR_NOTSUP through RSS_HAL_CALL.
 * ================================================================ */

static const rss_hal_ops_t g_ops = {
    .init = hal_init,
    .deinit = hal_deinit,
    .get_caps = hal_get_caps,

    .sys_get_version = hal_sys_get_version,

    /* The media clock rvd maps to wall time for the SEI timecodes. */
    .sys_get_timestamp = hal_sys_get_timestamp,
    .sys_rebase_timestamp = hal_sys_rebase_timestamp,

#ifdef HAL_MODULE_AUDIO
    /*
     * MI_AI capture. Only in the audio archive, since hal_audio.c is an
     * AUDIO_SRCS member and the video build has no such symbols.
     *
     * Capture only, and a short list -- see hal_audio.c's OP COVERAGE comment for
     * why each absent op is absent. The short version: this generation's
     * libmi_ai.so has no noise reduction, AGC, high-pass filter, echo
     * cancellation, resampler or audio encoder in it at all, and nothing in scope
     * plays audio out.
     *
     * Longer than the i6e backend's list by set_gain and get_gain, because MI 3.0
     * separates the analog interface gain from the DPGA's digital gain into two
     * calls. On MI 2.x both ops would have written the same register, whichever
     * raptor called last.
     */
    .audio_init = hal_audio_init,
    .audio_deinit = hal_audio_deinit,
    .audio_read_frame = hal_audio_read_frame,
    .audio_release_frame = hal_audio_release_frame,
    .audio_set_volume = hal_audio_set_volume,
    .audio_get_volume = hal_audio_get_volume,
    .audio_set_gain = hal_audio_set_gain,
    .audio_get_gain = hal_audio_get_gain,
    .audio_set_mute = hal_audio_set_mute,
#endif

#ifdef HAL_MODULE_VIDEO
    /* Vendor-neutral sysfs, so it works before any MI subsystem does. */
    .gpio_set = hal_gpio_set,
    .gpio_get = hal_gpio_get,
    .ircut_set = hal_ircut_set,

    /*
     * The datapath link. rvd's chain is FS [-> OSD] -> ENC and both shapes
     * arrive here; the OSD stage is collapsed rather than refused.
     */
    .bind = hal_bind,
    .unbind = hal_unbind,

    /* SNR -> VIF -> ISP -> SCL. A channel is an SCL output port. */
    .fs_create_channel = hal_fs_create_channel,
    .fs_set_channel_attr = hal_fs_set_channel_attr,
    .fs_destroy_channel = hal_fs_destroy_channel,
    .fs_enable_channel = hal_fs_enable_channel,
    .fs_disable_channel = hal_fs_disable_channel,
    .fs_set_rotation = hal_fs_set_rotation,

    /*
     * VENC. fs_get_frame and its release are deliberately absent: raw frame
     * readback means a user-facing MI_SYS output port on the scaler, which
     * nothing in raptor asks for on this part, and RSS_HAL_CALL turns the
     * absence into RSS_ERR_NOTSUP rather than a crash.
     */
    .enc_create_channel = hal_enc_create_channel,
    .enc_destroy_channel = hal_enc_destroy_channel,

    /*
     * MI has no encoder groups, but rvd's pipeline setup is IMP-shaped and calls
     * for one before it creates a channel -- so these are published rather than
     * left NOTSUP, which would fail every stream. register_channel is the one
     * that does real work: it is how a JPEG channel is fed at all.
     */
    .enc_create_group = hal_enc_create_group,
    .enc_destroy_group = hal_enc_destroy_group,
    .enc_register_channel = hal_enc_register_channel,
    .enc_unregister_channel = hal_enc_unregister_channel,

    .enc_start = hal_enc_start,
    .enc_stop = hal_enc_stop,
    .enc_poll = hal_enc_poll,
    .enc_get_frame = hal_enc_get_frame,
    .enc_release_frame = hal_enc_release_frame,
    .enc_request_idr = hal_enc_request_idr,
    .enc_get_fd = hal_enc_get_fd,
    .enc_set_rc_mode = hal_enc_set_rc_mode,
    .enc_set_bitrate = hal_enc_set_bitrate,
    .enc_set_gop = hal_enc_set_gop,
    .enc_set_fps = hal_enc_set_fps,
    .enc_query = hal_enc_query,

    .isp_get_exposure = hal_isp_get_exposure,
    .isp_get_sensor_attr = hal_isp_get_sensor_attr,

    /*
     * ISP tuning. Three mechanisms behind one surface -- the tuning API for the
     * image knobs, the ISP channel's parameter block for orientation and 3DNR,
     * and MI_SNR for the sensor rate; hal_isp.c's header says which is which.
     *
     * What is absent is absent on purpose. Manual white balance, DPC, hue,
     * highlight depress and backlight compensation have no counterpart bound
     * yet. RSS_HAL_CALL answers all of them RSS_ERR_NOTSUP, which rvd treats as
     * "this SoC does not have it" rather than as a fault.
     *
     * Saturation is absent for a different reason: it has a counterpart and
     * the counterpart costs too much. Every shipped tuning here varies it
     * across gain, and enOpType has no state between auto and manual, so the
     * knob could only trade that curve for a constant. See the note above
     * hal_isp_set_saturation. Brightness, contrast, sharpness and defog stay --
     * their curves are either flat in every bin or preserved by the vector
     * write -- so this is one row, not the family-wide withdrawal 6E took.
     */
    .isp_set_brightness = hal_isp_set_brightness,
    .isp_get_brightness = hal_isp_get_brightness,
    .isp_get_knob_caps = hal_isp_get_knob_caps,
    .isp_set_contrast = hal_isp_set_contrast,
    .isp_get_contrast = hal_isp_get_contrast,
    /*
     * Sharpness and spatial denoise are per-band tables on this generation
     * rather than levels, so one knob scales the whole run about the tuning's
     * own values -- see i6c_iq_apply_vector. They were NULL until that existed,
     * on the grounds that moving one band and calling it sharpness is worse
     * than not having the op.
     */
    .isp_set_sharpness = hal_isp_set_sharpness,
    .isp_get_sharpness = hal_isp_get_sharpness,
    /*
     * DRC is MI's WDR module. It is published where saturation is not, and the
     * difference is which way the trade falls: WDR's level is one byte the knob
     * can carry the majestic scale onto exactly, and the module is enabled and
     * in auto in all six shipped tunings here, so the knob has somewhere to go.
     * It does spend the per-gain curve when it leaves auto -- WDR's Strength
     * varies across gain in every bin -- which is the same cost saturation was
     * withdrawn for, taken deliberately this time because there is no other way
     * to offer the control at all and majestic has offered it this way for
     * years. Neutral 128 hands the curve back.
     */
    .isp_set_drc_strength = hal_isp_set_drc_strength,
    .isp_get_drc_strength = hal_isp_get_drc_strength,
    /*
     * No isp_{set,get}_sinter_strength: NrLumaAdv's blend weight is already at
     * the tuning's maximum on every sensor here, so the knob had no range to
     * offer. hal_isp.c says it in full. Left absent rather than stubbed, so
     * RSS_HAL_CALL answers RSS_ERR_NOTSUP and get-isp-caps stops listing it.
     */
    .isp_set_defog = hal_isp_set_defog,
    .isp_set_defog_strength = hal_isp_set_defog_strength,
    /* The one rvd's [image] block calls; the plain setter serves the ctrl API. */
    .isp_set_defog_strength_adv = hal_isp_set_defog_strength_adv,
    .isp_get_defog_strength = hal_isp_get_defog_strength,
    .isp_set_ae_comp = hal_isp_set_ae_comp,
    .isp_get_ae_comp = hal_isp_get_ae_comp,
    .isp_set_antiflicker = hal_isp_set_antiflicker,
    .isp_get_antiflicker = hal_isp_get_antiflicker,

    /* Day/night, which is colour or monochrome here: the IR-cut filter and the
     * illuminator are GPIOs ric drives itself. */
    .isp_set_running_mode = hal_isp_set_running_mode,
    .isp_get_running_mode = hal_isp_get_running_mode,

    .isp_set_hflip = hal_isp_set_hflip,
    .isp_set_vflip = hal_isp_set_vflip,
    .isp_get_hvflip = hal_isp_get_hvflip,
    .isp_set_temper_strength = hal_isp_set_temper_strength,
    .isp_get_temper_strength = hal_isp_get_temper_strength,
    .isp_set_sensor_fps = hal_isp_set_sensor_fps,
    .isp_get_sensor_fps = hal_isp_get_sensor_fps,

    /*
     * OSD over MI_RGN. Regions attach to the VENC channel (see hal_osd.c), so a
     * main and its cascaded sub get independent overlays. The get/timestamp/
     * attach-to-group ops rvd never calls are left NULL -> RSS_ERR_NOTSUP.
     */
    .osd_set_pool_size = hal_osd_set_pool_size,
    .osd_create_group = hal_osd_create_group,
    .osd_destroy_group = hal_osd_destroy_group,
    .osd_start = hal_osd_start,
    .osd_stop = hal_osd_stop,
    .osd_create_region = hal_osd_create_region,
    .osd_destroy_region = hal_osd_destroy_region,
    .osd_register_region = hal_osd_register_region,
    .osd_unregister_region = hal_osd_unregister_region,
    .osd_set_region_attr = hal_osd_set_region_attr,
    .osd_update_region_data = hal_osd_update_region_data,
    .osd_show_region = hal_osd_show_region,
#endif
};

/* ================================================================
 * FACTORY FUNCTIONS
 * ================================================================ */

rss_hal_ctx_t *rss_hal_create(void)
{
    rss_hal_ctx_t *ctx;

    ctx = (rss_hal_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx)
        return NULL;

    ctx->ops = &g_ops;
    memcpy(&ctx->caps, &g_hal_caps, sizeof(ctx->caps));

    return ctx;
}

/*
 * rss_hal_destroy -- free a HAL context.
 *
 * Does NOT call deinit() -- the caller must do that first. No scratch or NAL
 * arrays are freed here because no encoder allocates them yet; that belongs
 * with the subsystem that owns them.
 */
void rss_hal_destroy(rss_hal_ctx_t *ctx)
{
    if (!ctx)
        return;

    free(ctx);
}

const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx)
{
    if (!ctx)
        return NULL;

    return ctx->ops;
}

/* ================================================================
 * SYSTEM INFO (no vtable, called directly)
 * ================================================================ */

/*
 * Both names are IMP-specific but the daemons call them unconditionally to
 * print a build banner, with no context argument. MI's equivalent is
 * MI_SYS_GetVersion, so this answers only after hal_init -- before that there
 * is no loaded library to ask. There is no sysutils equivalent at all, so
 * that one is permanently unsupported.
 */
int rss_hal_get_imp_version(char *buf, int size)
{
    i6c_sys_ver ver;

    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    if (!g_infinity6c || !g_infinity6c->sys.get_version)
        return RSS_ERR_NOTSUP;

    memset(&ver, 0, sizeof(ver));
    if (g_infinity6c->sys.get_version(I6C_SOC_ID, &ver))
        return RSS_ERR_IO;

    snprintf(buf, (size_t)size, "%.*s", (int)sizeof(ver.version), (char *)ver.version);
    return RSS_OK;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    return RSS_ERR_NOTSUP;
}

/*
 * rss_hal_get_cpu_info -- SoC identification string.
 *
 * MI has no equivalent of IMP_System_GetCPUInfo(), so read the "Hardware"
 * line out of /proc/cpuinfo. Cached after the first call because the caller
 * treats the result as a borrowed static string.
 */
const char *rss_hal_get_cpu_info(void)
{
    static char cpu[64];
    static bool loaded = false;

    if (loaded)
        return cpu;

    loaded = true;
    snprintf(cpu, sizeof(cpu), "%s", HAL_PLATFORM_NAME);

    FILE *f = fopen("/proc/cpuinfo", "r");
    if (!f)
        return cpu;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "Hardware", 8) != 0)
            continue;
        char *val = strchr(line, ':');
        if (!val)
            break;
        val++;
        while (*val == ' ' || *val == '\t')
            val++;
        char *end = val + strlen(val);
        while (end > val && (end[-1] == '\n' || end[-1] == '\r' || end[-1] == ' '))
            end--;
        *end = '\0';
        if (*val)
            snprintf(cpu, sizeof(cpu), "%s", val);
        break;
    }

    fclose(f);
    return cpu;
}

const char *rss_hal_get_platform_name(void)
{
    return HAL_PLATFORM_NAME;
}

/*
 * rss_hal_check_platform -- verify the binary matches the running SoC.
 *
 * /proc/cpuinfo reports a marketing string that does not contain
 * "INFINITY6C", so the Ingenic prefix comparison would reject every valid
 * board. Checking properly needs a SoC-ID-to-family table; until then this
 * only warns, and never aborts.
 */
void rss_hal_check_platform(const char *name)
{
    (void)name;

    HAL_LOG_DBG("platform check: built for %s, running on %s", HAL_PLATFORM_NAME,
                rss_hal_get_cpu_info());
}
