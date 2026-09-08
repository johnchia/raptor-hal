/*
 * hisi_v5/v5_common.h -- HiMPP V5.0 common types and the vendor library set
 *
 * ONE MPP, TWO DIES. OpenIPC builds a single MPP for the whole hi3516cv6xx
 * family, and the banner it prints on a **CV608** board reads
 * "HI3516CV610_MPP_V1.0.2.0 B051 Release" while /proc/umap/sys names the
 * part "0X3516C608". So HI3516CV610 is the ABI's name here, not the die's,
 * exactly as HI3516EV200 is on gen4; the die reaches raptor separately, as
 * soc_model. Every guard in this backend is HAL_HISI_GEN5.
 *
 * PROVENANCE. Layouts, enumerators and constants were transcribed from
 * openhisilicon's GPL-3.0 public header set for this MPP build,
 * kernel/include/hi3516cv6xx/, which is both the licence-compatible source
 * and the version-matched one -- it is the 1.0.2.0 set, and 1.0.2.0 B051 is
 * what the board runs on both sides. Citations below name the file and line
 * there. The 1.0.1.0 headers in the test3 SDK drop are a diff base and
 * nothing more; where the two disagree the 1.0.2.0 set wins.
 *
 * Sizes in the _Static_asserts were read out of a probe compiled against
 * those headers with the cv6xx build's own arm-openipc-linux-musleabi-gcc
 * (raptor-hal tests/abi_probe_hisi5.c, `make abi-probe-hisi5`), not counted
 * by hand.
 *
 * Nearly every struct this backend asserts is free of pointers and
 * bitfields, so its 32-bit ARM layout is also its x86-64 layout and the
 * asserts hold on the host -- the gen5 test suites carry no
 * -D'_Static_assert(c,m)=' override for the same reason the gen4 ones do
 * not. The exceptions carry pointers (ot_venc_stream, ot_venc_pack, the
 * sensor object's vtable) and their asserts go under V5_ABI32 when Phase 2
 * introduces them.
 *
 * WHAT DID NOT PORT FROM gen4, and why it is absent rather than forgotten:
 *
 *   - The uClibc ABI trampolines (__ctype_b, __fgetc_unlocked,
 *     _stdlib_mb_cur_max). gen4's libsecurec.so is a uClibc build that
 *     leaves them undefined for the executable to satisfy. The V5 set is
 *     built against musl and every library here DT_NEEDEDs libc.so
 *     directly; the unresolved-symbol closure over all 60 vendor libraries
 *     on the board leaves nothing of that shape. Measured, not assumed.
 *   - The mmap shim. gen4 needed the daemon to export its own mmap because
 *     libmpi.so was a uClibc build with a 32-bit off_t meeting musl's
 *     64-bit one. These are musl builds; the ABI matches.
 *   - The Goke spelling. There is no Goke rebrand of V5. What replaces it
 *     is a *mechanical* alias: every MPI entry point is exported twice,
 *     once as ss_mpi_* and once as ot_mpi_*, and a handful (the mmz
 *     alloc_only / remap_* family) only as ot_mpi_*. v5_symbol() derives
 *     the second spelling from the first rather than making 200 call sites
 *     carry both.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_COMMON_H
#define HISI_V5_COMMON_H

#include "hal_symbols.h" /* hal_symbol_load, and hal_internal.h + dlfcn.h */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * True when pointers are four bytes, i.e. when this build's layouts are the
 * target's layouts. Guards the asserts that pin a struct containing a
 * pointer; everything not wrapped in it stays checked on the host too.
 */
#if UINTPTR_MAX == 0xFFFFFFFFu
#define V5_ABI32 1
#else
#define V5_ABI32 0
#endif

/*
 * Calling convention. Every V5 library on the board -- libss_mpi.so,
 * libot_mpi_isp.so, the eight algorithm libraries, all nine libsns_*.so --
 * carries Tag_FP_arch: VFPv4 and no Tag_ABI_VFP_args at all. The FPU is
 * used; the calling convention is soft-float, i.e. arm-*-linux-musleabi and
 * never musleabihf. Same tuple as gen4, measured again on V5's own
 * libraries rather than inherited from it.
 *
 * A compile error rather than a note because the failure is silent: a
 * hard-float build links against these and runs, and every MPI call taking
 * a float reads its argument out of the wrong register file. raptor's other
 * ARM platforms are all hf, so the mistake is one wrong tuple away.
 */
#if defined(__arm__) && defined(__ARM_PCS_VFP)
#error                                                                                             \
    "HiSilicon gen5 libraries are soft-float (no Tag_ABI_VFP_args). Build with an arm-*-musleabi toolchain, not musleabihf."
#endif

/* ================================================================
 * BASE ABI TYPES
 * ================================================================ */

/*
 * ot_mod_id (ot_common.h:193-293), the module identifier every bind and
 * every per-module call carries. Only the modules this backend reaches are
 * transcribed; the remaining ~50 are in that enum. Values are explicit
 * because they are ABI.
 *
 * Note the renumbering against gen4: RGN is still 3 and VPSS still 7, but
 * V5 inserts OT_ID_BASE at 4 and OT_ID_MEM takes 0, where gen4's MOD_ID_E
 * started at MOD_ID_CMPI. Transcribing gen4's values would land VENC on the
 * wrong module and bind silently to nothing.
 */
typedef enum {
    V5_MOD_MEM = 0,
    V5_MOD_VB = 1,
    V5_MOD_SYS = 2,
    V5_MOD_RGN = 3,
    V5_MOD_BASE = 4,
    V5_MOD_VPSS = 7,
    V5_MOD_VENC = 8,
    V5_MOD_H264E = 10,
    V5_MOD_JPEGE = 11,
    V5_MOD_H265E = 13,
    V5_MOD_VI = 16,
    V5_MOD_CHNL = 18,
    V5_MOD_RC = 19,
    V5_MOD_AIO = 20,
    V5_MOD_AI = 21,
    V5_MOD_AO = 22,
    V5_MOD_AENC = 23,
    V5_MOD_ADEC = 24,
    V5_MOD_ISP = 28,
    V5_MOD_VGS = 45,
} v5_mod_id;

/*
 * The error word (ot_errno.h:22-31). Unchanged in shape from gen4:
 *
 *   | 1 | APP_ID (7) | MOD_ID (8) | ERR_LEVEL (3) | ERR_ID (13) |
 *
 * Only the error id is worth matching on -- the module is already known at
 * every call site and the level is ERROR for everything a caller sees -- so
 * that is the only field extracted.
 */
#define V5_ERR_ID(ret) ((unsigned int)(ret) & 0x1fffu)

/* ot_errno.h:49-88. Only the codes this backend acts on differently from
 * "it failed"; the rest are logged as numbers. */
#define V5_ERR_EXIST 0x8u
#define V5_ERR_UNEXIST 0x9u
#define V5_ERR_NOT_CFG 0xbu
#define V5_ERR_NOT_SUPPORT 0xcu
#define V5_ERR_NOT_PERM 0xdu
#define V5_ERR_NO_MEM 0x14u
#define V5_ERR_NO_BUF 0x15u
#define V5_ERR_NOT_READY 0x18u
#define V5_ERR_BUSY 0x22u

/* TD_FAILURE (ot_type.h:29). The vendor's own generic failure, which is
 * what a callback returns when it declines. */
#define V5_FAILURE (-1)

/* ot_common_video.h:254-257 and :259-264. Present because every attribute
 * struct Phase 2 transcribes embeds them, and because a size that is two
 * td_u32 and a rect that is two td_s32 then two td_u32 is exactly the kind
 * of thing that is obvious until it is wrong. */
typedef struct {
    unsigned int width;
    unsigned int height;
} v5_size;

_Static_assert(sizeof(v5_size) == 8, "ot_size is 8 bytes");

typedef struct {
    int x;
    int y;
    unsigned int width;
    unsigned int height;
} v5_rect;

_Static_assert(sizeof(v5_rect) == 16, "ot_rect is 16 bytes");

/* ot_common.h:295-299. The (module, device, channel) triple both sides of
 * ss_mpi_sys_bind take. Same 12 bytes and same field order as gen4's
 * MPP_CHN_S; only the module numbering underneath moved. */
typedef struct {
    v5_mod_id mod_id;
    int dev_id;
    int chn_id;
} v5_mpp_chn;

_Static_assert(sizeof(v5_mpp_chn) == 12, "ot_mpp_chn is 12 bytes");
_Static_assert(offsetof(v5_mpp_chn, dev_id) == 4, "ot_mpp_chn.dev_id at +4");
_Static_assert(offsetof(v5_mpp_chn, chn_id) == 8, "ot_mpp_chn.chn_id at +8");

/*
 * ot_mpp_version (ot_common.h:147-149). OT_MAX_VERSION_NAME_LEN is 96
 * (ot_common.h:57) where gen4's VERSION_NAME_MAXLEN was 64, and the string
 * is not guaranteed terminated, so every read of it is bounded by the array
 * rather than by strlen.
 */
#define V5_MAX_VERSION_NAME_LEN 96

typedef struct {
    char version[V5_MAX_VERSION_NAME_LEN];
} v5_mpp_version;

_Static_assert(sizeof(v5_mpp_version) == 96, "ot_mpp_version is 96 bytes");

/* ot_defines.h:49. Wider than gen4's 16, and it is the tail of
 * ot_vb_pool_cfg, so getting it wrong moves nothing but makes every pool
 * name past 15 characters silently truncate. */
#define V5_MAX_MMZ_NAME_LEN 32

/* ================================================================
 * THE VENDOR LIBRARY SET
 *
 * V5 splits what gen4 kept in one libmpi.so. The measured shape, from the
 * import/export map over the 1.0.2.0 B051 set on the board:
 *
 *   libsecurec.so        memcpy_s / memset_s / snprintf_s / strncpy_s.
 *                        Not a DT_NEEDED of anything -- libss_mpi.so's only
 *                        NEEDED is libc.so -- so those four are undefined
 *                        until something puts securec in the global scope.
 *                        That is this file's job, and it is why securec is
 *                        opened first and RTLD_GLOBAL.
 *   libss_mpi_sysmem.so  the MMZ and mmap surface, split out of SYS.
 *   libss_mpi.so         SYS, VB, VI, VPSS, VENC, RGN -- 951 exports.
 *   libss_mpi_sysbind.so ss_mpi_sys_bind / unbind, split out of SYS.
 *
 * and then, for Phase 2, the ISP tier: eight algorithm libraries in a
 * dependency cycle with libot_mpi_isp.so, the 132-symbol libss_mpi_isp.so
 * facade over it, and libss_mpi_ae.so / libss_mpi_awb.so. Those are opened
 * by hisi_isp_open() in hal_common.c, not here, because breaking the cycle
 * needs the executable's forwarders and this header has no business
 * knowing about them.
 *
 * libot_osal.so ships on the image and is NOT opened: nothing in the
 * userspace set imports a single symbol from it. It is the kernel OSAL's
 * userspace mirror, there for the vendor's own tools.
 * ================================================================ */

typedef struct {
    void *securec; /* opened for its side effect; nothing dlsyms out of it */

    void *sysmem;  /* libss_mpi_sysmem.so */
    void *mpi;     /* libss_mpi.so */
    void *sysbind; /* libss_mpi_sysbind.so */

    /*
     * The ISP tier, opened later by hisi_isp_open() in hal_common.c once
     * the executable's forwarders are in place. Held here so v5_symbol()
     * can reach them: they are libraries like any other, and the only
     * thing special about them is when they may be opened.
     *
     * libot_mpi_isp.so is the implementation; libss_mpi_isp.so is a facade
     * over it -- ss_mpi_isp_init is *four bytes* of code, a branch -- and
     * both are opened because the facade is where the ss_mpi_ spellings
     * live. libss_mpi_ae.so and libss_mpi_awb.so hold the 3A registration
     * calls, and ss_mpi_isp_query_exposure_info is in **ae**, not isp.
     */
    void *isp_impl; /* libot_mpi_isp.so */
    void *isp;      /* libss_mpi_isp.so */
    void *ae;       /* libss_mpi_ae.so */
    void *awb;      /* libss_mpi_awb.so */

    /*
     * The audio tier, opened by v5_aud_open_libs (v5_aud.h) from the
     * audio archive only. The three algorithm libraries are opened for
     * their side effect -- libss_mpi_audio.so imports from them by symbol
     * and is opened RTLD_NOW -- and nothing dlsyms out of them.
     */
    void *upvqe; /* libupvqe.so */
    void *dnvqe; /* libdnvqe.so */
    void *voice; /* libvoice_engine.so */
    void *audio; /* libss_mpi_audio.so */

    /*
     * NULL-terminated resolution order for v5_symbol(). Ordered
     * most-specific-first only by accident: the libraries export disjoint
     * symbol sets, so the order is a formality and any of them answering
     * is the right answer. Kept as a list rather than keying each module
     * to a handle because a future OpenIPC build that merges or splits a
     * library again should cost nothing here.
     *
     * Sized for the three MPI libraries, the four ISP ones, the audio one
     * and the terminator, with room to spare. v5_libs_add_search() is
     * what grows it.
     */
    void *search[12];
} v5_mpi_libs;

/*
 * v5_libs_add_search -- append one open handle to the resolution order.
 *
 * Silently ignores a NULL handle, so a caller can pass the result of an
 * optional dlopen straight through. Returns 0 when the list is full, which
 * is a programming error rather than a runtime condition -- the array is
 * sized for every library this backend knows about.
 */
static inline int v5_libs_add_search(v5_mpi_libs *libs, void *handle)
{
    size_t i, n = sizeof(libs->search) / sizeof(libs->search[0]);

    if (!handle)
        return 1;

    for (i = 0; i + 1 < n; i++) {
        if (!libs->search[i]) {
            libs->search[i] = handle;
            libs->search[i + 1] = NULL;
            return 1;
        }
    }

    HAL_LOG_ERR("hisi_mpi: symbol search list full, %p dropped", handle);
    return 0;
}

/*
 * v5_symbol -- resolve one MPI entry point across the loaded libraries.
 *
 * Callers pass the ss_mpi_* spelling. Every entry point is exported twice,
 * ss_mpi_foo and ot_mpi_foo, and a few (ot_mpi_sys_mmz_alloc_only,
 * ot_mpi_sys_mmz_remap_cached, ot_mpi_sys_mmz_remap_nocache,
 * ot_mpi_sys_mmz_unmap, ot_mpi_sys_mmz_free_only) exist under the ot_
 * spelling alone. Deriving the alias here rather than making every call
 * site carry both strings is the difference between one rule and two
 * hundred opportunities to typo the second name.
 *
 * The derivation is deliberately narrow: it fires only for a name that
 * actually starts "ss_", and it edits two characters. Anything else is
 * looked up once, as given.
 *
 * Returns NULL and logs on total failure, so call sites read as
 *   if (!(lib->fnFoo = (cast)v5_symbol(mod, libs, "ss_mpi_foo")))
 *       return RSS_ERR_NOTSUP;
 */
static inline void *v5_symbol_search(const v5_mpi_libs *libs, const char *name)
{
    char alias[128];
    int i;

    /* "ss_mpi_..." -> "ot_mpi_...". strncpy_s is the vendor's idea of this;
     * a bounded copy and an explicit terminator is raptor's. */
    alias[0] = '\0';
    if (name[0] == 's' && name[1] == 's' && name[2] == '_' && strlen(name) < sizeof(alias)) {
        snprintf(alias, sizeof(alias), "%s", name);
        alias[0] = 'o';
        alias[1] = 't';
    }

    for (i = 0; i < (int)(sizeof(libs->search) / sizeof(libs->search[0])); i++) {
        void *fn;

        if (!libs->search[i])
            break;
        if ((fn = dlsym(libs->search[i], name)))
            return fn;
        if (alias[0] && (fn = dlsym(libs->search[i], alias)))
            return fn;
    }

    return NULL;
}

static inline void *v5_symbol(const char *module, const v5_mpi_libs *libs, const char *name)
{
    void *fn = v5_symbol_search(libs, name);

    if (!fn)
        HAL_LOG_ERR("%s: failed to acquire symbol %s", module, name);

    return fn;
}

/*
 * v5_symbol_opt -- the same, without the diagnostic.
 *
 * For entry points a board is allowed not to have. The caller decides what
 * a NULL means; nothing here does, because "absent" is not an error until
 * something asks for it.
 */
static inline void *v5_symbol_opt(const v5_mpi_libs *libs, const char *name)
{
    return v5_symbol_search(libs, name);
}

/*
 * hisi_mpi_open -- load the MPI side of the vendor library set.
 *
 * RTLD_NOW | RTLD_GLOBAL, and both are deliberate.
 *
 * RTLD_GLOBAL because libss_mpi.so's memcpy_s and friends are satisfied
 * from the global scope rather than by a DT_NEEDED, and because the ISP
 * tier Phase 2 opens on top of this reaches back into these libraries the
 * same way.
 *
 * RTLD_NOW rather than gen4's RTLD_LAZY, and this is a correction rather
 * than a preference. gen4 asked for LAZY on the theory that it would defer
 * the function half of the ISP cycle; musl implements no lazy binding at
 * all, accepts the flag and ignores it, so the request was inert and the
 * comment explaining it described a mechanism that never ran. Asking for
 * what actually happens is worth more than asking for what would have been
 * nice on a different libc -- and on a libc that *does* defer, NOW is still
 * what this backend wants: a missing symbol should surface at dlopen, next
 * to the library that lacks it, not at the first call from inside vendor
 * code.
 *
 * A missing libsecurec is a warning, not a failure: it is possible for a
 * build to satisfy those four another way, and failing the whole load for a
 * library that may be unnecessary would hide the real problem behind the
 * wrong one. The three MPI libraries are each fatal, because nothing works
 * without them and a partial set fails later and less legibly.
 */
static inline void hisi_mpi_close(v5_mpi_libs *libs);

static inline int hisi_mpi_open(v5_mpi_libs *libs)
{
    static const int flags = RTLD_NOW | RTLD_GLOBAL;

    memset(libs, 0, sizeof(*libs));

    if (!(libs->securec = dlopen("libsecurec.so", flags)))
        HAL_LOG_WARN("hisi_mpi: libsecurec.so absent (%s); libss_mpi.so's memcpy_s, memset_s, "
                     "snprintf_s and strncpy_s have nowhere to resolve from",
                     dlerror());

    /* Before libss_mpi.so, because that is the measured order and because
     * sysmem is the smaller, more isolated of the two -- a failure here is
     * unambiguous. */
    if (!(libs->sysmem = dlopen("libss_mpi_sysmem.so", flags))) {
        HAL_LOG_ERR("hisi_mpi: libss_mpi_sysmem.so: %s", dlerror());
        goto fail;
    }

    if (!(libs->mpi = dlopen("libss_mpi.so", flags))) {
        HAL_LOG_ERR("hisi_mpi: libss_mpi.so: %s", dlerror());
        goto fail;
    }

    if (!(libs->sysbind = dlopen("libss_mpi_sysbind.so", flags))) {
        HAL_LOG_ERR("hisi_mpi: libss_mpi_sysbind.so: %s", dlerror());
        goto fail;
    }

    v5_libs_add_search(libs, libs->mpi);
    v5_libs_add_search(libs, libs->sysbind);
    v5_libs_add_search(libs, libs->sysmem);

    HAL_LOG_DBG("hisi_mpi: libss_mpi.so + sysbind + sysmem loaded");
    return RSS_OK;

fail:
    /* Whatever opened above is already held; a caller that retries hal_init
     * would otherwise accumulate a reference per attempt. */
    hisi_mpi_close(libs);
    return RSS_ERR_NOENT;
}

static inline void hisi_mpi_close(v5_mpi_libs *libs)
{
    /* Reverse of the open order. dlclose on a library something else still
     * holds only drops this reference, so the order is bookkeeping rather
     * than a lifetime rule -- but it costs nothing to state it. */
    if (libs->audio)
        dlclose(libs->audio);
    if (libs->voice)
        dlclose(libs->voice);
    if (libs->dnvqe)
        dlclose(libs->dnvqe);
    if (libs->upvqe)
        dlclose(libs->upvqe);

    if (libs->awb)
        dlclose(libs->awb);
    if (libs->ae)
        dlclose(libs->ae);
    if (libs->isp)
        dlclose(libs->isp);
    if (libs->isp_impl)
        dlclose(libs->isp_impl);

    if (libs->sysbind)
        dlclose(libs->sysbind);
    if (libs->mpi)
        dlclose(libs->mpi);
    if (libs->sysmem)
        dlclose(libs->sysmem);
    if (libs->securec)
        dlclose(libs->securec);

    memset(libs, 0, sizeof(*libs));
}

#endif /* HISI_V5_COMMON_H */
