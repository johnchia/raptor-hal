/*
 * hisi_v5/hal_common.c -- Raptor HAL common layer, HiSilicon HiMPP V5.0
 *
 * Counterpart to src/hisi_v4/hal_common.c, and a separate translation unit
 * from it for the reason PLAN-hi3516cv610.md sets out at length: V5 renamed
 * every entry point, moved every layout, and split one libmpi.so into four
 * libraries. Sharing a file with gen4 would mean hiding an argument list
 * behind a macro at nearly every call site.
 *
 * Everything here guards on HAL_HISI_GEN5, never on PLATFORM_HI3516CV610.
 * The generation is the unit of compatibility and the *platform name is the
 * ABI's, not the die's*: OpenIPC builds one MPP for the whole hi3516cv6xx
 * family, and this file was brought up on a CV608 whose libraries report
 * "HI3516CV610_MPP_V1.0.2.0 B051 Release". The die reaches raptor
 * separately, as soc_model, and reaches this file as the chip name read
 * from /proc/umap/sys.
 *
 * Current state: Phase 1 skeleton. The vtable publishes only the ops that
 * are implemented; RSS_HAL_CALL() NULL-guards every entry and returns
 * RSS_ERR_NOTSUP for the rest, so unimplemented subsystems need no stubs.
 * A Phase 1 daemon comes up, logs the MPP version and the part, reaches
 * ss_mpi_sys_init, finds no framesource and exits cleanly. The video
 * pipeline lands in Phase 2.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <unistd.h>

/* ================================================================
 * LOGGING
 *
 * Mirrors src/hal_common.c, star/hal_common.c and the gen4 file: log
 * through a function pointer that defaults to stderr, which daemons
 * redirect to syslog at init.
 * ================================================================ */

static const char *hal_level_str[] = {"FTL", "ERR", "WRN", "INF", "DBG"};

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
{
    const char *basename;
    va_list ap;

    if (level < 0)
        level = 0;
    if (level > 4)
        level = 4;

    basename = strrchr(file, '/');
    if (basename)
        file = basename + 1;
    fprintf(stderr, "[HAL %s] %s:%d: ", hal_level_str[level], file, line);
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
 * The live backend state.
 *
 * The daemons call rss_hal_get_imp_version() and friends with no context
 * argument, and the ISP-cycle forwarders below are entered from a vendor
 * library that has no context to pass either. One HAL context per process
 * is already assumed throughout raptor; this is the pointer that makes that
 * assumption usable. Set at the end of hal_init, cleared in hal_deinit.
 */
static hisi_state_t *g_hisi;

/* ── GPIO / IR-cut (src/hal_gpio.c — plain sysfs, no SDK dependency) ── */

#ifdef HAL_MODULE_VIDEO
int hal_gpio_set(void *ctx, int pin, int value);
int hal_gpio_get(void *ctx, int pin, int *value);
int hal_ircut_set(void *ctx, int state);
#endif

/* ================================================================
 * FORWARDERS -- breaking the ISP / algorithm-library cycle
 *
 * This block is why raptor-hal's HiSilicon builds pass
 * -Wl,--export-dynamic unconditionally, and why it lives in *this* file
 * rather than in a quirks.c of its own.
 *
 * raptor-hal ships as libraptor_hal_video.a. A static-archive member is
 * extracted only if something already linked references a symbol in it, and
 * nothing in raptor references isp_alg_register_drc. A standalone quirks.c
 * would therefore compile, archive, and never link -- and --export-dynamic
 * cannot export what was never linked. -Wl,--gc-sections and -flto compound
 * it. hal_common.c is the translation unit that defines rss_hal_create, so
 * it is always extracted; putting the definitions here is what makes them
 * exist at all. __attribute__((used)) then keeps LTO and --gc-sections from
 * discarding them, since nothing in the program refers to them either.
 *
 * THE CYCLE, AND WHY musl MAKES IT THE EXECUTABLE'S PROBLEM.
 *
 * libot_mpi_isp.so and the eight algorithm libraries reference each other:
 *
 *   libot_mpi_isp.so  -> isp_alg_register_ldci        (in libldci.so)
 *                        isp_alg_register_drc         (in libdrc.so)
 *                        isp_alg_register_dehaze      (in libdehaze.so)
 *                        isp_alg_register_bayer_nr    (in libbnr.so)
 *                        isp_alg_register_acs         (in libacs.so)
 *                        isp_ir_auto_run_once         (in libir_auto.so)
 *                        isp_be_stats_estimate        (in libextend_stats.so)
 *                        calc_flicker_type            (in libcalcflicker.so)
 *   libldci.so        -> 18 symbols in libot_mpi_isp.so
 *   libdrc.so         -> 18      "
 *   libdehaze.so      -> 18      "
 *   libbnr.so         -> 32      "
 *   libacs.so         -> 18      "
 *   libextend_stats.so-> 4       "
 *   libcalcflicker.so -> 3       "
 *   libir_auto.so     -> 1       "
 *
 * (Measured with readelf on the board's own 1.0.2.0 B051 libraries. Those
 * eight are *exactly* libot_mpi_isp.so's undefined set once libc,
 * libsecurec and libss_mpi are accounted for -- there is no ninth waiting
 * to be discovered.)
 *
 * musl implements no lazy binding at all: RTLD_LAZY is accepted and
 * ignored, and every dlopen relocates fully. So neither side of the cycle
 * can be opened first, and something outside it has to satisfy one
 * direction. The vendor never meets this, because Makefile.param links
 * libot_mpi_isp.a and libdrc.a statically; a shipping product that dlopens
 * would meet it the same way raptor does.
 *
 * So the executable defines the eight, and the cycle unrolls:
 *
 *   1. dlopen(libot_mpi_isp.so) -- its eight imports bind to these
 *      forwarders, already in the global scope. It loads.
 *   2. dlopen(libldci.so) and the rest -- their imports resolve against
 *      libot_mpi_isp.so, now loaded and RTLD_GLOBAL. They load.
 *   3. hisi_isp_open (Phase 2) dlsyms the real eight out of step 2 and
 *      stores them in the state.
 *   4. ss_mpi_isp_init calls isp_alg_register_drc, reaches this forwarder,
 *      and is passed through to the real one.
 *
 * Step 4 is why the forwarders must stay valid for the process lifetime and
 * cannot become one-shot: libot_mpi_isp.so's GOT entry was bound once, at
 * its own dlopen, and points here for good.
 *
 * Each forwards through g_hisi rather than through a file-static pointer,
 * so the forwarders and the ISP loader cannot disagree about which library
 * is current. Before Phase 2 fills those pointers -- and after hal_deinit
 * clears them -- every one returns V5_FAILURE, and a vendor caller that
 * gets TD_FAILURE from a registrar declines to register, which is the
 * correct outcome when there is no algorithm library to register with.
 *
 * gen4's other three trampoline groups are absent here, and deliberately:
 * the uClibc ABI group (__ctype_b and friends) and the mmap shim both
 * existed because gen4's vendor libraries are uClibc builds. The V5 set is
 * musl-built and DT_NEEDEDs libc.so; the unresolved-symbol closure over all
 * 60 vendor libraries on the board leaves nothing of either shape. The
 * Goke group has no V5 counterpart at all. See v5_common.h.
 * ================================================================ */

#define HISI_FORWARD(field, ...)                                                                   \
    do {                                                                                           \
        hisi_state_t *st = g_hisi;                                                                 \
        if (!st || !st->field) {                                                                   \
            HAL_LOG_WARN("%s called with no ISP loaded", __func__);                                \
            return V5_FAILURE;                                                                     \
        }                                                                                          \
        return st->field(__VA_ARGS__);                                                             \
    } while (0)

__attribute__((used)) int isp_alg_register_ldci(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_ldci, vi_pipe);
}

__attribute__((used)) int isp_alg_register_drc(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_drc, vi_pipe);
}

__attribute__((used)) int isp_alg_register_dehaze(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_dehaze, vi_pipe);
}

__attribute__((used)) int isp_alg_register_bayer_nr(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_bayer_nr, vi_pipe);
}

__attribute__((used)) int isp_alg_register_acs(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_acs, vi_pipe);
}

/*
 * The three that are not registrars.
 *
 * Their second argument is a payload this backend never dereferences, so it
 * stays void * -- giving it a type would mean transcribing three more
 * structs to no end. The arity is what matters and it was read off the
 * libraries: each of the three touches r0 and r1 before its first call, and
 * none of the eight appears in any public header in the 1.0.2.0 set,
 * because the vendor links this boundary statically and never has to name
 * it.
 */
__attribute__((used)) int isp_ir_auto_run_once(int vi_pipe, void *arg)
{
    HISI_FORWARD(fn_ir_auto_run_once, vi_pipe, arg);
}

__attribute__((used)) int isp_be_stats_estimate(int vi_pipe, void *arg)
{
    HISI_FORWARD(fn_be_stats_estimate, vi_pipe, arg);
}

__attribute__((used)) int calc_flicker_type(int vi_pipe, void *arg)
{
    HISI_FORWARD(fn_calc_flicker_type, vi_pipe, arg);
}

/*
 * hisi_check_trampolines -- prove the executable really exports them.
 *
 * Runs before the first vendor dlopen, and not at ISP-open time, because by
 * then it is too late to be informative: musl relocates libot_mpi_isp.so
 * fully at its dlopen, so all eight have already either resolved or not
 * before any ISP code runs. A missing one shows up as
 * "Error relocating /usr/lib/libot_mpi_isp.so: isp_alg_register_drc: symbol
 * not found" -- which names the symbol but not the reason.
 *
 * dlsym(RTLD_DEFAULT, name) searching the global scope and finding *this*
 * object is what the vendor library will do. A mismatch means the link
 * dropped the definition -- no --export-dynamic, a static link, the archive
 * member not extracted -- and turns an unattributable relocation failure
 * into one log line naming the cause.
 *
 * All eight are checked rather than one canary. They are exported by the
 * same mechanism, so in practice they fail together; but the loop costs
 * eight dlsyms once per process, and "which one" is the first question
 * anybody debugging this would ask.
 *
 * Non-fatal by design: a build that gets this wrong should say so loudly
 * and then fail where the failure is, rather than refusing to start and
 * leaving no evidence.
 *
 * Not static: Phase 4's hal_audio.c is a second entry point that reaches
 * the first vendor dlopen without ever running hal_init -- rad calls
 * audio_init directly -- and it will need the same check for the same
 * reason.
 */
void hisi_check_trampolines(void);

void hisi_check_trampolines(void)
{
    static const struct {
        const char *name;
        void *fn;
    } tramps[] = {
        {"isp_alg_register_ldci", (void *)isp_alg_register_ldci},
        {"isp_alg_register_drc", (void *)isp_alg_register_drc},
        {"isp_alg_register_dehaze", (void *)isp_alg_register_dehaze},
        {"isp_alg_register_bayer_nr", (void *)isp_alg_register_bayer_nr},
        {"isp_alg_register_acs", (void *)isp_alg_register_acs},
        {"isp_ir_auto_run_once", (void *)isp_ir_auto_run_once},
        {"isp_be_stats_estimate", (void *)isp_be_stats_estimate},
        {"calc_flicker_type", (void *)calc_flicker_type},
    };
    unsigned i;
    int bad = 0;

    for (i = 0; i < sizeof(tramps) / sizeof(tramps[0]); i++) {
        void *found = dlsym(RTLD_DEFAULT, tramps[i].name);

        if (found == tramps[i].fn)
            continue;

        bad++;
        HAL_LOG_ERR("trampolines: %s resolves to %p, not %p", tramps[i].name, found, tramps[i].fn);
    }

    if (bad)
        HAL_LOG_ERR("trampolines: %d of %u not exported by the executable. libot_mpi_isp.so will "
                    "fail to relocate. Link with -Wl,--export-dynamic and keep hal_common.o in "
                    "the link.",
                    bad, (unsigned)(sizeof(tramps) / sizeof(tramps[0])));
    else
        HAL_LOG_DBG("trampolines: all 8 ISP-cycle symbols exported and resolvable");
}

/* ================================================================
 * CHIP IDENTIFICATION
 *
 * Two sources answering two questions; see hisi_state.h for which is which.
 * ================================================================ */

/*
 * hisi_read_chip_name -- the part, out of /proc/umap/sys, before any dlopen.
 *
 * The open_sys module prints a banner there as soon as it is modprobed:
 *
 *   [SYS] Version: [HI3516CV610_MPP_V1.0.2.0 B051 Release], Build Time[...]
 *
 *   ----------------------------------------0X3516C608--------------------
 *
 * so the part is the run of non-dash characters on the dashed line. Parsed
 * as text because that is what the driver offers: V5 publishes no register
 * accessor, and gen4's /dev/mem read of SCSYSID0 is a gen4 address with no
 * documented V5 counterpart.
 *
 * Bounded everywhere and tolerant of the file being absent -- which is the
 * ordinary case before load_hisilicon has run, not an error. Returns
 * RSS_ERR_NOENT then, and the caller decides what that means.
 */
static int hisi_read_chip_name(char *out, size_t out_size)
{
    char line[256];
    FILE *f;
    int found = 0;

    if (!out || out_size == 0)
        return RSS_ERR_INVAL;

    out[0] = '\0';

    f = fopen(HISI_UMAP_SYS_PATH, "r");
    if (!f)
        return RSS_ERR_NOENT;

    while (!found && fgets(line, sizeof(line), f)) {
        size_t start, end, len;

        /* The banner line is the one that begins with a dash run. The
         * version line above it begins with '[' and the module-param
         * heading below it also begins with dashes -- so the *first* such
         * line is the part and the loop stops there. */
        if (line[0] != '-')
            continue;

        for (start = 0; line[start] == '-'; start++)
            ;
        if (line[start] == '\0' || line[start] == '\n')
            continue; /* a bare rule, no token in it */

        for (end = start; line[end] && line[end] != '-' && line[end] != '\n'; end++)
            ;

        len = end - start;
        if (len == 0 || len >= out_size)
            break;

        memcpy(out, line + start, len);
        out[len] = '\0';
        found = 1;
    }

    fclose(f);
    return found ? RSS_OK : RSS_ERR_NOENT;
}

/*
 * hisi_chip_is_gen5 -- is this token a part this backend's ABI serves?
 *
 * The family prefix rather than an exact list, because the point of the
 * check is to catch a *generation* mismatch -- a gen4 or gen3 part running
 * a gen5 binary, where every symbol would resolve and every argument list
 * would be wrong. A cv6xx die nobody has held yet is a caps question, not
 * an ABI one, and gets a warning from rss_hal_check_platform.
 */
static bool hisi_chip_is_gen5(const char *name)
{
    return name && strncmp(name, "0X3516C6", 8) == 0;
}

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

/*
 * hisi_teardown -- put MPP back the way it was found.
 *
 * Reverse of bring-up, and the order is the vendor's rather than a
 * preference: ss_mpi_vb_exit with any module still holding a block returns
 * OT_ERR_VB_BUSY, so SYS has to go first. Phase 2 inserts the pipeline
 * reclaim above both.
 *
 * Idempotent and flag-driven, so hal_deinit after a failed hal_init tears
 * down exactly what came up.
 */
static void hisi_teardown(hisi_state_t *st)
{
    if (st->sys_inited) {
        if (st->sys.fnExit)
            st->sys.fnExit();
        st->sys_inited = false;
    }

    if (st->vb_inited) {
        if (st->vb.fnExit) {
            int rc = st->vb.fnExit();

            if (rc)
                HAL_LOG_WARN("vb: ss_mpi_vb_exit returned 0x%x (err %u)", (unsigned)rc,
                             V5_ERR_ID(rc));
        }
        st->vb_inited = false;
    }
}

/*
 * hal_init -- open the libraries and bring MPP up as far as SYS.
 *
 * The sequence, and every step of it is load-bearing:
 *
 *   check trampolines   before the first vendor dlopen; see that block
 *   dlopen              libsecurec, sysmem, libss_mpi, sysbind
 *   resolve             SYS and VB
 *   identify            chip id and MPP version, logged
 *   sys_exit + vb_exit  teardown-first
 *   vb_set_cfg          the pool configuration
 *   vb_init
 *   sys_init
 *
 * TEARDOWN-FIRST is inherited from gen4 and is not defensiveness. MPP state
 * lives in the kernel modules, not in the process: a previous run that was
 * killed, or a daemon restarted without the modules being reloaded, leaves
 * SYS initialised and VB holding its pools. ss_mpi_vb_set_cfg on a live VB
 * returns OT_ERR_VB_NOT_PERM, and the whole bring-up then fails for a
 * reason that has nothing to do with this run's configuration. Calling exit
 * on something that was never inited is harmless -- it returns
 * OT_ERR_*_NOT_READY, which is why the return values here are ignored on
 * purpose rather than by omission.
 *
 * PHASE 1 CONFIGURES NO POOLS. max_pool_cnt is 0, because the pool
 * arithmetic needs the sensor geometry and the stream configuration, and
 * neither exists until Phase 2 brings up hisi_sensor.c. VB comes up with no
 * common pools, SYS initialises on top of it, and the acceptance test --
 * reach sys_init, log the version, decline the pipeline -- is met. Phase 2
 * replaces the zero with hisi_vb_bringup's two-pool arithmetic, and Phase 7
 * measures it.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st;
    v5_vb_cfg vb_cfg;
    int ret;

    if (!c || !cfg || cfg->sensor_count < 1 || cfg->sensor_count > RSS_MAX_SENSORS)
        return RSS_ERR_INVAL;

    if (c->initialized) {
        HAL_LOG_ERR("hal_init: already initialized");
        return RSS_ERR_BUSY;
    }

    if (cfg->sensor_count > 1)
        HAL_LOG_WARN("hal_init: %d sensors requested, gen5 backend drives 1", cfg->sensor_count);

    memcpy(&c->multi_cfg, cfg, sizeof(c->multi_cfg));
    c->sensor_count = 1;
    memcpy(&c->sensors[0], &cfg->sensors[0], sizeof(c->sensors[0]));

    st = (hisi_state_t *)calloc(1, sizeof(*st));
    if (!st)
        return RSS_ERR_NOMEM;
    c->platform = st;

    snprintf(st->sensor_name, sizeof(st->sensor_name), "%s", cfg->sensors[0].name);

    /*
     * Before the first vendor dlopen, for the relocation reason in the
     * FORWARDERS block. Nothing below may be reordered above this line.
     */
    hisi_check_trampolines();

    ret = hisi_mpi_open(&st->libs);
    if (ret)
        goto err_free;

    ret = v5_sys_load(&st->sys, &st->libs);
    if (ret)
        goto err_unload;

    ret = v5_vb_load(&st->vb, &st->libs);
    if (ret)
        goto err_unload;

    /*
     * Identity, logged before anything is initialised so that a bring-up
     * failure below still leaves the two facts a bug report needs.
     *
     * The chip name was already read once by rss_hal_check_platform, which
     * runs before hal_init and has no state to cache it in. Reading it
     * again costs one small file and keeps the two paths independent.
     */
    if (hisi_read_chip_name(st->chip_name, sizeof(st->chip_name)) != RSS_OK)
        HAL_LOG_WARN("chip: %s unreadable; is the open_* module set loaded?", HISI_UMAP_SYS_PATH);

    if (st->sys.fnGetChipId) {
        unsigned int id = 0;

        if (!st->sys.fnGetChipId(&id)) {
            st->chip_id = id;
            st->chip_id_valid = true;
        }
    }

    if (st->sys.fnGetVersion) {
        v5_mpp_version ver;

        memset(&ver, 0, sizeof(ver));
        if (!st->sys.fnGetVersion(&ver)) {
            /*
             * The vendor fills this from its own OT_VERSION string and
             * leaves the assignment in: ss_mpi_sys_get_version hands back
             * "OT_VERSION=HI3516CV610_MPP_V1.0.2.0 B051 Release", where
             * gen4's HI_MPI_SYS_GetVersion returned the bare version. The
             * prefix reaches rvd's banner as "LIBIMP Version OT_VERSION=..."
             * if it is not taken off here, so it is taken off here rather
             * than at each of the three call sites that print it.
             *
             * Conditional, not unconditional: a build that ever stops
             * doing this should keep working, and a version string that
             * does not start with the prefix is used whole.
             */
            static const char pfx[] = "OT_VERSION=";
            const char *v = ver.version;

            if (!strncmp(v, pfx, sizeof(pfx) - 1))
                v += sizeof(pfx) - 1;

            snprintf(st->mpp_version, sizeof(st->mpp_version), "%.*s",
                     (int)(sizeof(ver.version) - (size_t)(v - ver.version)), v);
        }
    }

    HAL_LOG_INFO("mpp: %s", st->mpp_version[0] ? st->mpp_version : "version unavailable");
    if (st->chip_id_valid)
        HAL_LOG_INFO("chip: %s (id 0x%08x)", st->chip_name[0] ? st->chip_name : "unknown",
                     st->chip_id);
    else
        HAL_LOG_INFO("chip: %s", st->chip_name[0] ? st->chip_name : "unknown");

    /* Teardown-first; see the function comment. Return values ignored on
     * purpose: on a clean boot both report NOT_READY. */
    (void)st->sys.fnExit();
    (void)st->vb.fnExit();

    /* Phase 1: no common pools. See the function comment. */
    memset(&vb_cfg, 0, sizeof(vb_cfg));
    vb_cfg.max_pool_cnt = 0;

    ret = st->vb.fnSetCfg(&vb_cfg);
    if (ret) {
        HAL_LOG_ERR("vb: ss_mpi_vb_set_cfg failed 0x%x (err %u)", (unsigned)ret, V5_ERR_ID(ret));
        ret = RSS_ERR_IO;
        goto err_unload;
    }

    ret = st->vb.fnInit();
    if (ret) {
        HAL_LOG_ERR("vb: ss_mpi_vb_init failed 0x%x (err %u)", (unsigned)ret, V5_ERR_ID(ret));
        ret = RSS_ERR_IO;
        goto err_unload;
    }
    st->vb_inited = true;

    ret = st->sys.fnInit();
    if (ret) {
        HAL_LOG_ERR("sys: ss_mpi_sys_init failed 0x%x (err %u)", (unsigned)ret, V5_ERR_ID(ret));
        ret = RSS_ERR_IO;
        goto err_teardown;
    }
    st->sys_inited = true;

    HAL_LOG_INFO("hal_init: MPP up to SYS; the pipeline is not implemented yet (Phase 2)");

    g_hisi = st;
    c->initialized = true;
    return RSS_OK;

err_teardown:
    hisi_teardown(st);
err_unload:
    v5_vb_unload(&st->vb);
    v5_sys_unload(&st->sys);
    hisi_mpi_close(&st->libs);
err_free:
    free(st);
    c->platform = NULL;
    return ret;
}

static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st = hisi_state(ctx);

    if (!c)
        return RSS_ERR_INVAL;

    if (!st)
        return RSS_OK;

    hisi_teardown(st);

    /*
     * Cleared after the teardown, not before.
     *
     * The forwarders are entered from the vendor libraries, and from Phase 2
     * the last thing that legitimately does so is the ISP's own unregister
     * path inside the pipeline teardown. Clearing this first would make
     * those decline and leave the ISP holding callbacks into a library
     * about to be dlclosed.
     */
    if (g_hisi == st)
        g_hisi = NULL;

    v5_vb_unload(&st->vb);
    v5_sys_unload(&st->sys);
    hisi_mpi_close(&st->libs);

    free(st);
    c->platform = NULL;
    c->initialized = false;

    return RSS_OK;
}

/*
 * hal_get_caps -- return the per-SoC capability struct.
 *
 * The context's copy, not g_hal_caps directly: rss_hal_create_backend
 * adjusts the backend-surface flags on it.
 */
static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? &c->caps : &g_hal_caps;
}

/* ================================================================
 * SYSTEM UTILITIES
 * ================================================================ */

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->mpp_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->mpp_version);
    return RSS_OK;
}

static int hal_sys_get_cpu_info(void *ctx, char *buf, int len)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->chip_name[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->chip_name);
    return RSS_OK;
}

/*
 * Media clock. rvd_frame_loop.c uses these to publish the
 * media-clock-to-UTC mapping SEI timecodes are derived from; without them
 * the mapping early-returns, frames still flow, and timecodes silently
 * vanish.
 */
static int hal_sys_get_timestamp(void *ctx, int64_t *ts)
{
    hisi_state_t *st = hisi_state(ctx);
    unsigned long long pts = 0;

    if (!ts)
        return RSS_ERR_INVAL;
    if (!st || !st->sys.fnGetCurPts)
        return RSS_ERR_NOTSUP;

    if (st->sys.fnGetCurPts(&pts))
        return RSS_ERR_IO;

    *ts = (int64_t)pts;
    return RSS_OK;
}

static int hal_sys_rebase_timestamp(void *ctx, int64_t base)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !st->sys.fnInitPtsBase)
        return RSS_ERR_NOTSUP;

    if (st->sys.fnInitPtsBase((unsigned long long)base))
        return RSS_ERR_IO;

    return RSS_OK;
}

/* ================================================================
 * OPS VTABLE
 *
 * Only implemented ops are listed. Everything else stays NULL and resolves
 * to RSS_ERR_NOTSUP through RSS_HAL_CALL -- which is what makes a Phase 1
 * build useful rather than merely compilable: rvd starts, prints its
 * banner, finds no framesource and exits cleanly.
 *
 * The video pipeline (fs_*, enc_*, isp_*) lands in Phase 2, ISP tuning in
 * Phase 3, audio in Phase 4, OSD in Phase 5.
 * ================================================================ */

static const rss_hal_ops_t g_ops = {
    /* System lifecycle */
    .init = hal_init,
    .deinit = hal_deinit,
    .get_caps = hal_get_caps,

    /* System utilities */
    .sys_get_version = hal_sys_get_version,
    .sys_get_cpu_info = hal_sys_get_cpu_info,
    .sys_get_timestamp = hal_sys_get_timestamp,
    .sys_rebase_timestamp = hal_sys_rebase_timestamp,

#ifdef HAL_MODULE_VIDEO
    /* GPIO / IR-cut -- vendor-neutral sysfs, works as-is. The one part of
     * the video surface that owes nothing to MPP, which is why it is here
     * in Phase 1 and the rest is not. */
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
    return rss_hal_create_backend("imp");
}

/*
 * rss_hal_create_backend -- pick a pipeline backend by name.
 *
 * This build carries one, and it is the vendor's: HiMPP is how a gen5 part
 * talks to its ISP and encoder at all. "imp" is the name the config's
 * default carries on every platform, so it means "the built-in one" here
 * rather than Ingenic's library. Any other name gets NULL, because a caller
 * that asked for a different pipeline is better told it is missing than
 * handed this one under its name.
 *
 * The backend-surface flags stay false in Phase 1 and are set as each
 * subsystem lands. That is load-bearing rather than cosmetic: a context
 * with has_framesource false gets no framesource created, so rvd starts,
 * reports what it cannot do, and exits cleanly instead of failing partway
 * into a pipeline that does not exist yet.
 */
rss_hal_ctx_t *rss_hal_create_backend(const char *backend)
{
    rss_hal_ctx_t *ctx;

    if (backend && strcmp(backend, "imp") != 0)
        return NULL;

    ctx = (rss_hal_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx)
        return NULL;

    ctx->ops = &g_ops;
    memcpy(&ctx->caps, &g_hal_caps, sizeof(ctx->caps));

    return ctx;
}

/*
 * rss_hal_destroy -- free a HAL context and internal resources.
 *
 * Does NOT call deinit() -- the caller must do that first.
 */
void rss_hal_destroy(rss_hal_ctx_t *ctx)
{
    int i;

    if (!ctx)
        return;

    for (i = 0; i < RSS_MAX_ENC_CHANNELS; i++) {
        free(ctx->scratch_buf[i]);
        ctx->scratch_buf[i] = NULL;
        free(ctx->nal_arrays[i]);
        ctx->nal_arrays[i] = NULL;
    }

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
 * rss_hal_get_imp_version / rss_hal_get_sysutils_version
 *
 * Both names are IMP-specific, but the daemons call them unconditionally to
 * print a build banner and pass no context. HiMPP's equivalent is
 * ss_mpi_sys_get_version, reached through g_hisi, so this answers only
 * after hal_init -- before that there is no loaded library to ask. There is
 * no sysutils equivalent at all, so that one is permanently unsupported.
 */
int rss_hal_get_imp_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    if (!g_hisi || !g_hisi->mpp_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)size, "%s", g_hisi->mpp_version);
    return RSS_OK;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_NOTSUP;

    return RSS_ERR_NOTSUP;
}

/*
 * rss_hal_get_cpu_info -- SoC identification string.
 *
 * Answers from /proc/umap/sys rather than /proc/cpuinfo, which on this
 * kernel reports "Hisilicon (Flattened Device Tree)" and names no part at
 * all. The caller treats the result as a borrowed static string, so the
 * value is cached after the first read; the part is not going to change.
 *
 * Deliberately independent of hal_init: rvd prints the banner before it
 * brings the pipeline up, and a platform check that only works afterwards
 * is a platform check that never runs on the build that needed it. That is
 * also why this reads the file itself instead of waiting for
 * st->chip_name -- though it prefers the state's copy when there is one,
 * so that the two can never disagree in a log.
 */
const char *rss_hal_get_cpu_info(void)
{
    static char cpu[24];
    static bool loaded = false;

    if (loaded)
        return cpu;

    loaded = true;

    if (g_hisi && g_hisi->chip_name[0]) {
        snprintf(cpu, sizeof(cpu), "%s", g_hisi->chip_name);
        return cpu;
    }

    if (hisi_read_chip_name(cpu, sizeof(cpu)) != RSS_OK)
        snprintf(cpu, sizeof(cpu), "%s", HAL_PLATFORM_NAME);

    return cpu;
}

const char *rss_hal_get_platform_name(void)
{
    return HAL_PLATFORM_NAME;
}

/*
 * rss_hal_check_platform -- verify the binary matches the running SoC.
 *
 * The same two-tier judgement gen4 makes, from a different source:
 *
 *   - a different hi3516cv6xx die. One MPP build serves the family -- the
 *     CV608 this backend was brought up on reports the *CV610* MPP version
 *     string -- so the binary is correct and only the caps numbers may be
 *     off. Warn, and keep running.
 *   - anything else. A gen4 or gen3 part has a different MPI ABI behind
 *     differently-named symbols; in practice the dlopen would fail first,
 *     but a part that somehow carried a V5 library set with a V4 kernel is
 *     exactly the mismatch that produces garbage rather than an error.
 *     Refuse.
 *
 * Unreadable is NOT a mismatch, and on V5 that case is ordinary rather than
 * exotic: /proc/umap/sys does not exist until load_hisilicon has modprobed
 * open_sys, and a daemon started before that -- or on a board where the
 * modules failed to load -- would be refused for a reason that has nothing
 * to do with the binary. Warn and continue; the failure will surface at
 * dlopen, where it can be named.
 */
void rss_hal_check_platform(const char *name)
{
    char chip[24];

    (void)name;

    if (hisi_read_chip_name(chip, sizeof(chip)) != RSS_OK) {
        HAL_LOG_WARN("platform check: %s unreadable, assuming %s", HISI_UMAP_SYS_PATH,
                     HAL_PLATFORM_NAME);
        return;
    }

    if (!hisi_chip_is_gen5(chip)) {
        /* Reported the way Ingenic reports its own fatal mismatch --
         * stderr and syslog both -- because rvd may already have detached
         * from the terminal by the time this runs, and a fatal nobody can
         * read is indistinguishable from a silent one. */
        fprintf(stderr,
                "FATAL: built for %s (HiMPP V5.0) but %s reports '%s', "
                "which is not a hi3516cv6xx part\n",
                HAL_PLATFORM_NAME, HISI_UMAP_SYS_PATH, chip);
        openlog(name ? name : "raptor", LOG_PID, LOG_DAEMON);
        syslog(LOG_ERR,
               "FATAL: built for %s (HiMPP V5.0) but %s reports '%s', "
               "which is not a hi3516cv6xx part",
               HAL_PLATFORM_NAME, HISI_UMAP_SYS_PATH, chip);
        closelog();
        _exit(1);
    }

    if (strcmp(chip, HISI_CHIP_HI3516CV608) != 0 && strcmp(chip, HISI_CHIP_HI3516CV610) != 0)
        HAL_LOG_WARN("platform check: %s reports '%s', a hi3516cv6xx die this build has not met. "
                     "One MPP serves the family, so the code is right; the capability numbers in "
                     "hal_caps.c may not be.",
                     HISI_UMAP_SYS_PATH, chip);
    else
        HAL_LOG_DBG("platform check: built for %s, running on %s", HAL_PLATFORM_NAME, chip);
}
