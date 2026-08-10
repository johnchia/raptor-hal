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
#endif

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

/*
 * hal_init -- load the MI modules and bring MI_SYS up.
 *
 * The sensor configuration is accepted and ignored: selecting a sensor means
 * MI_SNR, and configuring one means the VIF -> ISP -> SCL chain, none of which
 * exists yet. Taking the argument now keeps the op signature stable so the
 * daemons need no change when it starts being honoured.
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

    if ((ret = i6c_sys_load(&st->sys)) != RSS_OK) {
        i6c_sys_unload(&st->sys);
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
        i6c_sys_unload(&st->sys);
        free(st);
        return RSS_ERR_IO;
    }
    st->sys_inited = true;

    /*
     * Logged at info rather than debug: until a pipeline exists this string is
     * the whole observable result of a bring-up run, and it is what says the
     * loaded libraries are the drop the headers were transcribed from.
     */
    {
        i6c_sys_version ver;

        memset(&ver, 0, sizeof(ver));
        if (st->sys.get_version(I6C_SOC_ID, &ver) == 0)
            HAL_LOG_INFO("infinity6c: MI %.*s", (int)sizeof(ver.version), (char *)ver.version);
        else
            HAL_LOG_WARN("infinity6c: MI_SYS_GetVersion failed; MI is up but unidentified");
    }

    hal->platform = st;
    g_infinity6c = st;

    HAL_LOG_INFO("infinity6c: %s up, MI_SYS only -- no video or audio subsystem yet",
                 HAL_PLATFORM_NAME);
    return RSS_OK;
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

    i6c_sys_unload(&st->sys);

    if (g_infinity6c == st)
        g_infinity6c = NULL;
    hal->platform = NULL;
    free(st);

    return RSS_OK;
}

static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    (void)ctx;

    return &g_hal_caps;
}

/* ================================================================
 * SYSTEM INFO
 * ================================================================ */

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    rss_hal_ctx_t *hal = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;
    i6c_sys_version ver;

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
    i6c_sys_version ver;

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
