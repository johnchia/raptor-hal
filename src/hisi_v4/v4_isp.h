/*
 * hisi_v4/v4_isp.h -- libisp.so, the 3A libraries, and their loader
 *
 * Phase 2 scope: enough ISP to make the VI pipe produce pictures. The tuning
 * ops are Phase 3. That split is forced rather than chosen -- on HiMPP the
 * pipe delivers nothing until 3A is registered and HI_MPI_ISP_Init has run,
 * so "bring up the pipeline, then add the ISP" is not a possible order here.
 * SigmaStar could do it that way because CUS3A auto-starts with the VPE
 * channel; gen4 cannot.
 *
 * THE LOAD ORDER IS FORCED, AND RTLD_LAZY DOES NOT HELP. libisp.so and the
 * algorithm libraries reference each other:
 *
 *   - libisp.so needs ISP_AlgRegister{Dehaze,Drc,Ldci} from the algorithm
 *     libraries. All R_ARM_JUMP_SLOT.
 *   - the algorithm libraries need g_astIspCtx and g_pastRegCfgCtx from
 *     libisp.so. Both R_ARM_GLOB_DAT.
 *
 * On glibc or uClibc the JUMP_SLOT half would be deferred under RTLD_LAZY
 * and loading libisp.so first would resolve the cycle. **This rootfs is
 * musl, which implements no lazy binding at all** -- RTLD_LAZY is accepted
 * and ignored, and every dlopen relocates fully. Measured on the board:
 * dlopen("libisp.so", RTLD_LAZY) fails with "Error relocating
 * /usr/lib/libisp.so: ISP_AlgRegisterDrc: symbol not found".
 *
 * So the cycle is broken from outside it. The *executable* defines the
 * three registrars as forwarders -- see the ISP CYCLE block in
 * hal_common.c -- which lets libisp.so load first, after which the
 * algorithm libraries find its data symbols in the global scope. This
 * function then resolves the real registrars and stores them in the state
 * for the forwarders to call.
 *
 * RTLD_LAZY is still passed, because it is correct on any loader that
 * honours it and costs nothing on the one that does not.
 *
 * libisp.so additionally reaches into libmpi.so for MPI_VI_{Get,Set}FPNAttr
 * and MPI_VI_{Get,Set}IspDISAttr -- note the MPI_ prefix, not HI_MPI_ --
 * which is why hisi_mpi_open() opens libmpi.so RTLD_GLOBAL.
 *
 * lib_hiacs.so and lib_hicalcflicker.so DO NOT EXIST on a gen4 board.
 * Verified against the SDK tarball and then against a running board, where
 * no binary references an ACS symbol at all and flicker is handled inside
 * libisp.so, which defines ISP_AlgRegisterFlicker itself. divinus hard-fails
 * when lib_hiacs.so is missing, and worse, dlsyms off the NULL handle -- on
 * glibc and uClibc that is RTLD_DEFAULT, an accidental global-scope search
 * rather than an error. Nothing here dlsyms off a handle it did not check.
 *
 * PROVENANCE. ISP_PUB_ATTR_S and ALG_LIB_S derived from the Hi3516EV200 SDK
 * V1.0.1.0 headers (mpp/include/hi_comm_isp.h, hi_comm_3a.h) and
 * cross-checked against ref/openhisilicon/include/comm_isp.h and comm_3a.h
 * (GPL v3, ALG_LIB_S at :406). Offsets from a compiled probe.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_ISP_H
#define HISI_V4_ISP_H

#include "v4_common.h"
#include "v4_video.h"

#define V4_ALG_LIB_NAME_SIZE_MAX 20

/*
 * ALG_LIB_S -- the 3A library's identity, matched by *string* inside the
 * ISP. The name is not cosmetic: HI_MPI_AE_Register looks the algorithm up
 * by it, so a wrong string is a registration failure with no other symptom.
 *
 * Two spellings exist and they belong to two different ISP builds:
 *
 *   "hisi_ae_lib" / "hisi_awb_lib"   HiSilicon's, from hi_ae_comm.h:30 and
 *                                    hi_awb_comm.h:28. What the board's
 *                                    lib_hiae.so contains, and what majestic
 *                                    passes.
 *   "ae_lib" / "awb_lib"             openhisilicon's, from its ae_comm.h:16
 *                                    and awb_comm.h:15, where AE and AWB are
 *                                    compiled *into* libisp.so rather than
 *                                    shipped as separate libraries.
 *
 * That difference is the discriminator, and it is why v4_isp_load picks the
 * names from which libraries it actually found rather than from a build-time
 * constant: a separate lib_hiae.so means the HiSilicon stack, no separate AE
 * library means the open one.
 */
#define V4_AE_LIB_NAME_HISI "hisi_ae_lib"
#define V4_AWB_LIB_NAME_HISI "hisi_awb_lib"
#define V4_AE_LIB_NAME_OPEN "ae_lib"
#define V4_AWB_LIB_NAME_OPEN "awb_lib"

typedef struct {
    int id;
    char lib_name[V4_ALG_LIB_NAME_SIZE_MAX];
} v4_alg_lib;

_Static_assert(sizeof(v4_alg_lib) == 24, "ALG_LIB_S is 24 bytes");

/*
 * ISP_PUB_ATTR_S -- what the ISP is being asked to process.
 *
 * stWndRect is the crop taken out of the sensor's output, stSnsSize is that
 * output. Both come from the sensor mode INI's DevRect_* keys; a sensor
 * whose INI crops (the IMX335's 5 MP mode starts at x=200, y=20) has them
 * differ, and setting stWndRect to the full sensor size on such a mode gives
 * a green band down two edges.
 *
 * f32FrameRate is a float, but it crosses the ABI inside a struct passed by
 * pointer, so the soft-float calling convention does not reach it. Nothing
 * in this backend passes a float by value to a vendor library, which is the
 * only place __ARM_PCS_VFP would matter at runtime.
 */
typedef struct {
    v4_rect wnd_rect;
    v4_size sns_size;
    float frame_rate;
    v4_bayer_format bayer;
    v4_wdr_mode wdr_mode;
    unsigned char sns_mode;
} v4_isp_pub_attr;

_Static_assert(sizeof(v4_isp_pub_attr) == 40, "ISP_PUB_ATTR_S is 40 bytes");
_Static_assert(offsetof(v4_isp_pub_attr, sns_size) == 16, "stSnsSize at +16");
_Static_assert(offsetof(v4_isp_pub_attr, frame_rate) == 24, "f32FrameRate at +24");
_Static_assert(offsetof(v4_isp_pub_attr, bayer) == 28, "enBayer at +28");
_Static_assert(offsetof(v4_isp_pub_attr, wdr_mode) == 32, "enWDRMode at +32");
_Static_assert(offsetof(v4_isp_pub_attr, sns_mode) == 36, "u8SnsMode at +36");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    void *isp;     /* libisp.so */
    void *ae;      /* lib_hiae.so, or NULL when AE lives inside libisp.so */
    void *awb;     /* lib_hiawb.so, likewise */
    void *ir_auto; /* lib_hiir_auto.so */
    void *ldci;    /* lib_hildci.so */
    void *dehaze;  /* lib_hidehaze.so */
    void *drc;     /* lib_hidrc.so */

    /* Resolution order for v4_symbol(): libisp first, then the algorithm
     * libraries. Same shape as v4_mpi_libs.search and for the same reason --
     * which library exports a symbol is a property of the build, not
     * something to encode as policy. */
    void *search[8];

    /* Which naming convention the loaded stack uses; see v4_alg_lib. */
    const char *ae_lib_name;
    const char *awb_lib_name;

    /* Every one of these is keyed on VI_PIPE. There is no ISP device on
     * gen4 -- the ISP is the front half of the VI pipe. */
    int (*fnMemInit)(int vi_pipe);
    int (*fnSetPubAttr)(int vi_pipe, const v4_isp_pub_attr *attr);
    int (*fnGetPubAttr)(int vi_pipe, v4_isp_pub_attr *attr);
    int (*fnInit)(int vi_pipe);
    int (*fnRun)(int vi_pipe); /* blocks for process lifetime; needs a thread */
    int (*fnExit)(int vi_pipe);

    int (*fnAeRegister)(int vi_pipe, v4_alg_lib *ae_lib);
    int (*fnAeUnRegister)(int vi_pipe, v4_alg_lib *ae_lib);
    int (*fnAwbRegister)(int vi_pipe, v4_alg_lib *awb_lib);
    int (*fnAwbUnRegister)(int vi_pipe, v4_alg_lib *awb_lib);
} v4_isp_impl;

/*
 * Where hisi_isp_open puts the real algorithm registrars.
 *
 * A small struct rather than three out-parameters, and separate from
 * v4_isp_impl because these are not entry points this backend calls: they
 * are what the executable's forwarders call, and they live in hisi_state_t
 * alongside the other forwarder targets so that one pointer reaches all of
 * them.
 */
typedef struct {
    int (**drc)(int vi_pipe);
    int (**dehaze)(int vi_pipe);
    int (**ldci)(int vi_pipe);
} hisi_isp_alg_regs;

/*
 * hisi_isp_open -- load the ISP stack in the one order that works.
 *
 * Returns RSS_ERR_NOENT if libisp.so is absent, RSS_ERR_NOTSUP if it is
 * present but missing a required entry point. The algorithm libraries are
 * each optional: a build with AE compiled into libisp.so has no lib_hiae.so
 * to open, and treating that as a failure would reject the openhisilicon
 * stack outright.
 */
static inline int hisi_isp_open(v4_isp_impl *lib, const v4_mpi_libs *libs,
                               hisi_isp_alg_regs *algs)
{
    static const char mod[] = "v4_isp";
    /* See the load-order note at the top of this file: on musl this is the
     * same as RTLD_NOW, which is why the executable has to supply the three
     * registrars before this call. */
    static const int flags = RTLD_LAZY | RTLD_GLOBAL;
    v4_mpi_libs search;
    int n = 0;

    memset(lib, 0, sizeof(*lib));

    if (!(lib->isp = dlopen("libisp.so", flags))) {
        HAL_LOG_ERR("%s: libisp.so absent: %s", mod, dlerror());
        return RSS_ERR_NOENT;
    }

    /* Algorithm libraries second, so their eager g_astIspCtx / g_pastRegCfgCtx
     * relocations find libisp.so already in the global scope. */
    lib->ae = dlopen("lib_hiae.so", flags);
    lib->awb = dlopen("lib_hiawb.so", flags);
    lib->ir_auto = dlopen("lib_hiir_auto.so", flags);
    lib->ldci = dlopen("lib_hildci.so", flags);
    lib->dehaze = dlopen("lib_hidehaze.so", flags);
    lib->drc = dlopen("lib_hidrc.so", flags);

    /*
     * A separate AE library means HiSilicon's stack and its "hisi_" prefixed
     * algorithm names; AE inside libisp.so means openhisilicon's and its
     * bare ones. Decided from what is on the board rather than from a build
     * flag, because one raptor binary is meant to run against either.
     */
    if (lib->ae) {
        lib->ae_lib_name = V4_AE_LIB_NAME_HISI;
        lib->awb_lib_name = V4_AWB_LIB_NAME_HISI;
    } else {
        lib->ae_lib_name = V4_AE_LIB_NAME_OPEN;
        lib->awb_lib_name = V4_AWB_LIB_NAME_OPEN;
    }
    HAL_LOG_INFO("%s: %s ISP stack (%s / %s)", mod, lib->ae ? "HiSilicon" : "open",
                 lib->ae_lib_name, lib->awb_lib_name);

    lib->search[n++] = lib->isp;
    if (lib->ae)
        lib->search[n++] = lib->ae;
    if (lib->awb)
        lib->search[n++] = lib->awb;

    /*
     * The MPI handles come last in the search order, and they are in it
     * rather than out of it because a Goke build puts the GK_API_ISP_*
     * entry points in libgk_api.so alongside everything else, not in a
     * separate ISP library. On a HiSilicon build they are in libisp.so and
     * the MPI handles are never reached; on a Goke one they are the only
     * place to look. Searching both costs one failed dlsym per symbol at
     * load time and nothing afterwards.
     */
    if (libs) {
        unsigned int k;

        for (k = 0; k < sizeof(libs->search) / sizeof(libs->search[0]); k++) {
            if (!libs->search[k])
                break;
            if (n >= (int)(sizeof(lib->search) / sizeof(lib->search[0])) - 1)
                break;
            lib->search[n++] = libs->search[k];
        }
    }
    lib->search[n] = NULL;

    /*
     * v4_symbol takes a v4_mpi_libs to search, so borrow the ISP's handle
     * list into one rather than duplicating the search helper.
     */
    memset(&search, 0, sizeof(search));
    memcpy(search.search, lib->search,
           sizeof(search.search) < sizeof(lib->search) ? sizeof(search.search)
                                                       : sizeof(lib->search));

#define V4_ISP_REQ(field, type, hi, gk)                                                            \
    do {                                                                                           \
        if (!(lib->field = (type)v4_symbol(mod, &search, hi, gk)))                                 \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V4_ISP_REQ(fnMemInit, int (*)(int), "HI_MPI_ISP_MemInit", "GK_API_ISP_MemInit");
    V4_ISP_REQ(fnSetPubAttr, int (*)(int, const v4_isp_pub_attr *), "HI_MPI_ISP_SetPubAttr",
               "GK_API_ISP_SetPubAttr");
    V4_ISP_REQ(fnInit, int (*)(int), "HI_MPI_ISP_Init", "GK_API_ISP_Init");
    V4_ISP_REQ(fnRun, int (*)(int), "HI_MPI_ISP_Run", "GK_API_ISP_Run");
    V4_ISP_REQ(fnExit, int (*)(int), "HI_MPI_ISP_Exit", "GK_API_ISP_Exit");
    V4_ISP_REQ(fnAeRegister, int (*)(int, v4_alg_lib *), "HI_MPI_AE_Register", "GK_API_AE_Register");
    V4_ISP_REQ(fnAwbRegister, int (*)(int, v4_alg_lib *), "HI_MPI_AWB_Register",
               "GK_API_AWB_Register");

#undef V4_ISP_REQ

    lib->fnGetPubAttr = (int (*)(int, v4_isp_pub_attr *))v4_symbol_opt(
        &search, "HI_MPI_ISP_GetPubAttr", "GK_API_ISP_GetPubAttr");
    lib->fnAeUnRegister = (int (*)(int, v4_alg_lib *))v4_symbol_opt(&search, "HI_MPI_AE_UnRegister",
                                                                    "GK_API_AE_UnRegister");
    lib->fnAwbUnRegister = (int (*)(int, v4_alg_lib *))v4_symbol_opt(
        &search, "HI_MPI_AWB_UnRegister", "GK_API_AWB_UnRegister");

    /*
     * The other half of the cycle: the real registrars, now that the
     * libraries defining them are loaded. dlsym on each specific handle
     * rather than through the search list, because the point is to reach
     * the definition in the algorithm library and *not* the forwarder in
     * the executable -- RTLD_DEFAULT would find the forwarder and build an
     * infinite loop out of it.
     *
     * Missing is survivable: a build without lib_hidrc.so has no DRC to
     * register, and the forwarder reports the call rather than crashing.
     */
    if (algs) {
        if (lib->drc)
            *algs->drc = (int (*)(int))dlsym(lib->drc, "ISP_AlgRegisterDrc");
        if (lib->dehaze)
            *algs->dehaze = (int (*)(int))dlsym(lib->dehaze, "ISP_AlgRegisterDehaze");
        if (lib->ldci)
            *algs->ldci = (int (*)(int))dlsym(lib->ldci, "ISP_AlgRegisterLdci");

        HAL_LOG_INFO("%s: algorithm registrars drc=%s dehaze=%s ldci=%s", mod,
                     *algs->drc ? "ok" : "absent", *algs->dehaze ? "ok" : "absent",
                     *algs->ldci ? "ok" : "absent");
    }

    return RSS_OK;
}

static inline void hisi_isp_close(v4_isp_impl *lib)
{
    /* Reverse of open, and every handle checked: divinus dlsyms off a NULL
     * lib_hiacs handle, which is RTLD_DEFAULT rather than an error. */
    if (lib->drc)
        dlclose(lib->drc);
    if (lib->dehaze)
        dlclose(lib->dehaze);
    if (lib->ldci)
        dlclose(lib->ldci);
    if (lib->ir_auto)
        dlclose(lib->ir_auto);
    if (lib->awb)
        dlclose(lib->awb);
    if (lib->ae)
        dlclose(lib->ae);
    if (lib->isp)
        dlclose(lib->isp);

    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_ISP_H */
