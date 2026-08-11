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
 * Current state: module loader and system ops only. The vtable publishes what
 * is implemented and nothing else; RSS_HAL_CALL NULL-guards every entry and
 * returns RSS_ERR_NOTSUP, so an absent subsystem needs no stub and no file.
 * hal_caps.c declares the matching zeroes, which is what keeps rvd from
 * asking for a stream this backend cannot yet build.
 *
 * What this stage is for: proving the argument lists. dlsym resolves by name,
 * so an MI 2.x table binds against these libraries without complaint and then
 * calls MI_SYS_Init with a stray value where the SoC id belongs. Nothing
 * reports that. A backend that loads the modules, initialises MI and reads
 * back a version string has exercised the one thing no compiler or linker
 * checks here.
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

    st->snr_profile = -1;

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
    HAL_LOG_INFO("infinity6c: %s up, MI_SYS only -- no audio subsystem yet", HAL_PLATFORM_NAME);
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

#ifdef HAL_MODULE_VIDEO
    /* Vendor-neutral sysfs, so it works before any MI subsystem does. */
    .gpio_set = hal_gpio_set,
    .gpio_get = hal_gpio_get,
    .ircut_set = hal_ircut_set,

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
