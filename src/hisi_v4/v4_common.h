/*
 * hisi_v4/v4_common.h -- HiMPP V4.0 common types and the vendor library set
 *
 * The gen4 HiSilicon parts (Hi3516EV200, Hi3516EV300, Hi3516DV200,
 * Hi3518EV300) share one MPP build: the banner on an EV300 board reads
 * "Hi3516EV200_MPP_V1.0.1.2 B030 Release", so everything here is a
 * generation fact rather than a part fact and every guard in this backend
 * is HAL_HISI_GEN4.
 *
 * PROVENANCE. Layouts and field order were derived from the Hi3516EV200 SDK
 * V1.0.1.0 MPP headers (mpp/include/hi_common.h, hi_comm_sys.h) and
 * cross-checked field by field against ref/openhisilicon/include/common.h
 * (GPL v3), where the same structs appear under Goke-flat names. The two
 * agree on every member and every enumerator used here, so the citations
 * below name openhisilicon: it is the licence-compatible one and a reader
 * can open it. Sizes and offsets in the _Static_asserts were read out of a
 * probe compiled against the SDK headers with arm-openipc-linux-musleabi-gcc,
 * not counted by hand.
 *
 * One consequence worth knowing before writing the host tests: almost every
 * struct asserted in this backend is free of pointers and bitfields, so its
 * 32-bit ARM layout is also its x86-64 layout and the _Static_asserts hold on
 * the host. raptor-hal/tests/Makefile carries -D'_Static_assert(c,m)=' on each
 * suite because star's asserts pin layouts that are false off ARM by design;
 * the gen4 suites will not need it, and should not carry it -- an assert that
 * can run on the host is one that fails in the test rather than in the
 * cross-build.
 *
 * The exceptions are the three structs that do carry pointers -- the sensor
 * driver's vtable, VENC_PACK_S and VENC_STREAM_S. Their exact sizes and
 * offsets are 32-bit facts, so those asserts are wrapped in V4_ABI32 and are
 * simply absent on the host, while the shape checks that *are* portable (a
 * vtable of nine pointers is nine pointers wide anywhere) stay unconditional.
 *
 * Unlike the SigmaStar backends there is no headers submodule: gen4 MPP
 * headers exist under a licence raptor can cite (openhisilicon's), so the
 * declarations and the dlopen half live in the same file rather than being
 * split across a vendored ABI header and a *_load.h. Each v4_<mod>.h is
 * therefore "the ABI and the loader for one MPP module".
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_COMMON_H
#define HISI_V4_COMMON_H

#include "hal_symbols.h" /* hal_symbol_load, and hal_internal.h + dlfcn.h */

#include <stddef.h>
#include <stdint.h>

/*
 * True when pointers are four bytes, i.e. when this build's layouts are the
 * target's layouts.
 *
 * Guards the handful of _Static_asserts that pin a struct containing a
 * pointer. Those sizes are 32-bit ARM facts and are false on an x86-64 host
 * by construction, so asserting them there would break the host build that
 * the whole dlopen-nothing design exists to keep working. Everything not
 * wrapped in this is portable and stays checked everywhere.
 */
#if UINTPTR_MAX == 0xFFFFFFFFu
#define V4_ABI32 1
#else
#define V4_ABI32 0
#endif

/*
 * Calling convention. Every binary on a gen4 board -- libmpi.so, libisp.so,
 * libsecurec.so, all six lib_hi*.so, all 34 libsns_*.so, majestic -- has no
 * Tag_ABI_VFP_args at all, while carrying Tag_FP_arch: VFPv4. The FPU is
 * used; the calling convention is soft-float, i.e. arm-*-linux-musleabi and
 * never musleabihf.
 *
 * This has to be a compile error rather than a note, because the failure is
 * silent: a hard-float build links against these libraries and runs, and
 * every MPI call taking a float or double reads its argument out of the
 * wrong register file. raptor's other ARM platforms are all hf, so the
 * mistake is one wrong tuple away.
 */
#if defined(__arm__) && defined(__ARM_PCS_VFP)
#error "HiSilicon gen4 libraries are soft-float (no Tag_ABI_VFP_args). Build with an arm-*-musleabi toolchain, not musleabihf."
#endif

/* ================================================================
 * BASE ABI TYPES
 * ================================================================ */

/*
 * MOD_ID_E, the module identifier every bind and every per-module call
 * carries. Only the modules this backend reaches are transcribed; the
 * remaining ~50 are in ref/openhisilicon/include/common.h (MOD_ID_*, :139)
 * and the SDK's hi_common.h (HI_ID_*), which agree enumerator for
 * enumerator. Values are explicit because they are ABI, not because the
 * list is sparse.
 */
typedef enum {
    V4_MOD_VB = 1,
    V4_MOD_SYS = 2,
    V4_MOD_RGN = 3,
    V4_MOD_VPSS = 7,
    V4_MOD_VENC = 8,
    V4_MOD_VI = 16,
    V4_MOD_AIO = 20,
    V4_MOD_AI = 21,
    V4_MOD_AO = 22,
    V4_MOD_AENC = 23,
    V4_MOD_ADEC = 24,
    V4_MOD_ISP = 28,
} v4_mod_id;

/*
 * HiMPP packs every error return into one 32-bit word:
 *
 *   | 1 | APP_ID (7) | MOD_ID (8) | ERR_LEVEL (3) | ERR_ID (13) |
 *
 * so 0xa0028009 reads as SYS / ERROR / EN_ERR_NOT_PERM. Only the error id
 * is worth matching on in code -- the module is already known at every call
 * site, and the level is ERROR for everything a caller ever sees -- so that
 * is the only field this extracts.
 */
#define V4_ERR_ID(ret) ((unsigned int)(ret) & 0x1fffu)

#define V4_ERR_NOT_PERM 9u

/* MPP_CHN_S (openhisilicon common.h:208). The (module, device, channel)
 * triple both sides of HI_MPI_SYS_Bind take. */
typedef struct {
    v4_mod_id module;
    int device;
    int channel;
} v4_mpp_chn;

_Static_assert(sizeof(v4_mpp_chn) == 12, "MPP_CHN_S is 12 bytes");
_Static_assert(offsetof(v4_mpp_chn, device) == 4, "MPP_CHN_S.s32DevId at +4");
_Static_assert(offsetof(v4_mpp_chn, channel) == 8, "MPP_CHN_S.s32ChnId at +8");

/* MPP_VERSION_S (openhisilicon common.h:78). VERSION_NAME_MAXLEN is 64 and
 * the string is not guaranteed terminated, so every read of it is bounded
 * by the array rather than by strlen. */
#define V4_VERSION_NAME_MAXLEN 64

typedef struct {
    char version[V4_VERSION_NAME_MAXLEN];
} v4_sys_ver;

_Static_assert(sizeof(v4_sys_ver) == 64, "MPP_VERSION_S is 64 bytes");

/* ================================================================
 * THE VENDOR LIBRARY SET
 * ================================================================ */

/*
 * Which MPI library this board carries. Recorded rather than inferred
 * because the 3A library names in Phase 2 differ on a Goke build, and
 * because the neo case is not a variant of the others -- see below.
 */
typedef enum {
    V4_MPI_NONE = 0,
    V4_MPI_STOCK, /* libmpi.so, the HiSilicon SDK build */
    V4_MPI_GOKE,  /* libgk_api.so + libhi_mpi.so, the Goke rebrand */
} v4_mpi_variant;

/*
 * Handles held for the life of the process.
 *
 * `search` is the ordered list every v4_<mod>_load() resolves through, and
 * it exists because SYS does not have to come from the same library as
 * VI/VPSS/VENC. openhisilicon's libmpi_neo.so is a single translation unit
 * (libraries/mpi_neo/src/mpi_sys.c) exporting exactly
 * HI_MPI_SYS_{Init,Exit,MmzAlloc,MmzAlloc_Cached,MmzFlushCache,MmzFree} --
 * no VI, no VPSS, no VENC. So neo can never serve the pipeline; it can only
 * serve SYS, alongside a stock or Goke library that serves the rest.
 * Searching an ordered list rather than keying each module to a handle
 * keeps that from becoming a policy decision made in advance of evidence:
 * whichever library actually exports the symbol wins.
 *
 * securec, upvqe, dnvqe and VoiceEngine are opened for their side effects
 * only -- nothing dlsyms out of them. They are loaded first and RTLD_GLOBAL
 * because every other vendor library leaves memcpy_s/strcpy_s and the VQE
 * entry points undefined for the loader to satisfy from the global scope.
 */
typedef struct {
    void *securec;
    void *upvqe;
    void *dnvqe;
    void *voice;

    void *mpi; /* libmpi.so, or libhi_mpi.so on a Goke build */
    void *gk;  /* libgk_api.so, Goke only */
    void *neo; /* libmpi_neo.so, if present */

    /* NULL-terminated resolution order for v4_symbol(). */
    void *search[4];

    v4_mpi_variant variant;
} v4_mpi_libs;

/*
 * v4_symbol -- resolve one MPI entry point across the loaded libraries.
 *
 * Two fallbacks, both of which are properties of real boards rather than
 * defensiveness:
 *
 *   - across handles, because SYS may live in libmpi_neo.so while the rest
 *     of the pipeline lives in libmpi.so (see v4_mpi_libs);
 *   - across spellings, because a Goke build exports GK_API_* where a
 *     HiSilicon build exports HI_MPI_*. openhisilicon reconciles the two at
 *     compile time in include/hicompat.h; a dlopen backend has to do it at
 *     load time. Pass gk_name = NULL for a symbol with no Goke spelling.
 *
 * Returns NULL and logs on total failure, so call sites read as
 *   if (!(lib->fnFoo = (cast)v4_symbol(mod, libs, "HI_MPI_Foo", "GK_API_Foo")))
 *       return RSS_ERR_NOTSUP;
 */
static inline void *v4_symbol(const char *module, const v4_mpi_libs *libs, const char *hi_name,
                              const char *gk_name)
{
    int i;

    for (i = 0; i < (int)(sizeof(libs->search) / sizeof(libs->search[0])); i++) {
        void *fn;

        if (!libs->search[i])
            break;
        if ((fn = dlsym(libs->search[i], hi_name)))
            return fn;
        if (gk_name && (fn = dlsym(libs->search[i], gk_name)))
            return fn;
    }

    HAL_LOG_ERR("%s: failed to acquire symbol %s", module, hi_name);
    return NULL;
}

/*
 * v4_symbol_opt -- the same, without the diagnostic.
 *
 * For entry points a board is allowed not to have. The caller decides what
 * a NULL means; nothing here does, because "absent" is not an error until
 * something asks for it.
 */
static inline void *v4_symbol_opt(const v4_mpi_libs *libs, const char *hi_name,
                                  const char *gk_name)
{
    int i;

    for (i = 0; i < (int)(sizeof(libs->search) / sizeof(libs->search[0])); i++) {
        void *fn;

        if (!libs->search[i])
            break;
        if ((fn = dlsym(libs->search[i], hi_name)))
            return fn;
        if (gk_name && (fn = dlsym(libs->search[i], gk_name)))
            return fn;
    }

    return NULL;
}

/*
 * hisi_mpi_open -- load the MPI side of the vendor library set.
 *
 * Order matters and is not a preference:
 *
 *   1. libsecurec.so first. Every other HiSilicon library leaves memcpy_s
 *      and friends undefined, and libsecurec is itself a DT_NEEDED of
 *      libmpi.so and libisp.so -- opening it explicitly and RTLD_GLOBAL is
 *      what makes those references resolve to one copy.
 *   2. The VQE trio. Same reason, for the audio libraries.
 *   3. The MPI library proper: stock, then Goke. libmpi_neo.so is opened
 *      alongside rather than instead of either, because it serves SYS only.
 *
 * RTLD_LAZY throughout, and RTLD_GLOBAL throughout. Both are load-bearing.
 * RTLD_GLOBAL because libisp.so reaches into libmpi.so for
 * MPI_VI_{Get,Set}FPNAttr and MPI_VI_{Get,Set}IspDISAttr -- note the MPI_
 * prefix, not HI_MPI_ -- and those are satisfied from the global scope
 * rather than by a DT_NEEDED. RTLD_LAZY because libisp.so and the six
 * lib_hi*.so algorithm libraries reference each other circularly, and only
 * the function half of that cycle is deferrable; see hisi_isp_open().
 *
 * Missing libsecurec or VQE is a warning, not a failure: some builds
 * satisfy those symbols another way, and failing the whole load for a
 * library that may be unnecessary would hide the real problem behind the
 * wrong one. A missing MPI library is fatal, because nothing works without
 * it.
 */
static inline int hisi_mpi_open(v4_mpi_libs *libs)
{
    static const int flags = RTLD_LAZY | RTLD_GLOBAL;
    int n = 0;

    memset(libs, 0, sizeof(*libs));

    if (!(libs->securec = dlopen("libsecurec.so", flags)))
        HAL_LOG_WARN("hisi_mpi: libsecurec.so absent (%s)", dlerror());

    libs->upvqe = dlopen("libupvqe.so", flags);
    libs->dnvqe = dlopen("libdnvqe.so", flags);
    libs->voice = dlopen("libVoiceEngine.so", flags);
    if (!libs->upvqe || !libs->dnvqe || !libs->voice)
        HAL_LOG_WARN("hisi_mpi: VQE libraries incomplete (up=%p dn=%p ve=%p)", libs->upvqe,
                     libs->dnvqe, libs->voice);

    if ((libs->mpi = dlopen("libmpi.so", flags))) {
        libs->variant = V4_MPI_STOCK;
    } else if ((libs->gk = dlopen("libgk_api.so", flags))) {
        libs->variant = V4_MPI_GOKE;
        /* Goke splits the API surface: libgk_api.so carries the GK_API_*
         * names, libhi_mpi.so the HI_MPI_* ones. Both are searched, so a
         * build that ships only one of them still resolves. */
        libs->mpi = dlopen("libhi_mpi.so", flags);
    } else {
        HAL_LOG_ERR("hisi_mpi: no MPI library found (libmpi.so, libgk_api.so): %s", dlerror());
        return RSS_ERR_NOENT;
    }

    /* Optional, and never the only source: see v4_mpi_libs. */
    libs->neo = dlopen("libmpi_neo.so", flags);

    if (libs->gk)
        libs->search[n++] = libs->gk;
    if (libs->mpi)
        libs->search[n++] = libs->mpi;
    if (libs->neo)
        libs->search[n++] = libs->neo;
    libs->search[n] = NULL;

    HAL_LOG_INFO("hisi_mpi: %s build%s", libs->variant == V4_MPI_GOKE ? "Goke" : "HiSilicon",
                 libs->neo ? ", libmpi_neo.so present" : "");
    return RSS_OK;
}

static inline void hisi_mpi_close(v4_mpi_libs *libs)
{
    /* Reverse of the open order. dlclose on a library something else still
     * DT_NEEDEDs only drops this reference, so the order is bookkeeping
     * rather than a lifetime rule -- but it costs nothing to state it. */
    if (libs->neo)
        dlclose(libs->neo);
    if (libs->mpi)
        dlclose(libs->mpi);
    if (libs->gk)
        dlclose(libs->gk);
    if (libs->voice)
        dlclose(libs->voice);
    if (libs->dnvqe)
        dlclose(libs->dnvqe);
    if (libs->upvqe)
        dlclose(libs->upvqe);
    if (libs->securec)
        dlclose(libs->securec);

    memset(libs, 0, sizeof(*libs));
}

#endif /* HISI_V4_COMMON_H */
