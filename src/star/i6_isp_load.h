/*
 * star/i6_isp_load.h -- dlopen loader and function table for MI_ISP
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_isp.h:
 *
 *   1. HAL_ERROR(mod, ...) becomes HAL_LOG_ERR(...) plus an explicit return,
 *      since HAL_ERROR's hidden `return EXIT_FAILURE` is not a convention
 *      raptor uses.
 *   2. EXIT_SUCCESS/EXIT_FAILURE become RSS_OK/RSS_ERR_*: RSS_ERR_NOENT when
 *      a library is absent, RSS_ERR_NOTSUP when a library is present but
 *      lacks a symbol. Callers can then tell "SDK not installed" from "SDK
 *      too old" without parsing logs.
 *   3. The loaders are `static inline`, not `static`. raptor-hal builds with
 *      -Werror, and an unused `static` function in a header is a
 *      -Wunused-function error; `static inline` is exempt.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#ifndef STAR_I6_ISP_LOAD_H
#define STAR_I6_ISP_LOAD_H

#include <i6_isp.h>

#include "i6_symbols.h"

typedef struct {
    void *handle, *handleCus3a, *handleIspAlgo;

    /*
     * No 3A entry points are bound here on purpose: this backend loads the
     * tuning binary and leaves the vendor algorithms alone (see hal_isp.c's
     * load site). Binding them as hard requirements, as an earlier
     * arrangement of this did, fails ISP init on any board whose
     * libmi_isp.so lacks either -- over symbols nothing would have
     * called.
     */
    int (*fnLoadChannelConfig)(int channel, char *path, unsigned int key);
    int (*fnGetParaInitStatus)(int channel, i6_isp_parainit *status);
    int (*fnGetExposureLimit)(int channel, i6_isp_exp *config);
    int (*fnSetExposureLimit)(int channel, i6_isp_exp *config);

    /*
     * Optional -- may be NULL, unlike everything above. Both are only
     * wanted by isp_get_exposure, which is advisory (ric's day/night
     * detection); a library without them should still bring the ISP up
     * and stream, so the loader must not fail on their absence.
     */
    int (*fnGetAeStatus)(int channel, i6_isp_ae_status *status);
    int (*fnGetAeHwAvgStats)(int channel, i6_isp_ae_hw_stats *stats);
} i6_isp_impl;

static inline int i6_isp_load(i6_isp_impl *isp_lib)
{
    /*
     * Same chain, same order, same best-effort policy as i6_vpe_load --
     * see the long comment there for why DT_NEEDED cannot be relied on
     * and why RTLD_GLOBAL is required. VPE has almost certainly opened
     * all three already by the time the ISP comes up; dlopen refcounts,
     * so opening them again is correct rather than merely harmless, as
     * it keeps this module's teardown independent of VPE's.
     */
    isp_lib->handleIspAlgo = dlopen("libispalgo.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!isp_lib->handleIspAlgo)
        HAL_LOG_WARN("i6_isp: libispalgo.so not loaded (%s) -- ISP algorithms may fail",
                     dlerror());

    isp_lib->handleCus3a = dlopen("libcus3a.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!isp_lib->handleCus3a)
        HAL_LOG_WARN("i6_isp: libcus3a.so not loaded (%s) -- ISP algorithms may fail",
                     dlerror());

    if (!(isp_lib->handle = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_isp: failed to load libmi_isp.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(isp_lib->fnLoadChannelConfig =
              (int (*)(int channel, char *path, unsigned int key))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_API_CmdLoadBinFile")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnGetParaInitStatus =
              (int (*)(int channel, i6_isp_parainit *status))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_IQ_GetParaInitStatus")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnGetExposureLimit =
              (int (*)(int channel, i6_isp_exp *config))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_AE_GetExposureLimit")))
        return RSS_ERR_NOTSUP;

    if (!(isp_lib->fnSetExposureLimit =
              (int (*)(int channel, i6_isp_exp *config))hal_symbol_load(
                  "i6_isp", isp_lib->handle, "MI_ISP_AE_SetExposureLimit")))
        return RSS_ERR_NOTSUP;

    /*
     * dlsym directly rather than hal_symbol_load: these two are optional
     * (see the impl struct), and hal_symbol_load logs at error level, so
     * routing an expected NULL through it would report a fault every boot
     * on a library that simply predates the symbol.
     */
    isp_lib->fnGetAeStatus = (int (*)(int channel, i6_isp_ae_status *status))dlsym(
        isp_lib->handle, "MI_ISP_CUS3A_GetAeStatus");
    isp_lib->fnGetAeHwAvgStats = (int (*)(int channel, i6_isp_ae_hw_stats *stats))dlsym(
        isp_lib->handle, "MI_ISP_AE_GetAeHwAvgStats");
    if (!isp_lib->fnGetAeStatus)
        HAL_LOG_WARN("i6_isp: no MI_ISP_CUS3A_GetAeStatus -- "
                     "exposure readback unavailable, ric cannot detect day/night");

    return RSS_OK;
}

static inline void i6_isp_unload(i6_isp_impl *isp_lib)
{
    if (isp_lib->handle)
        dlclose(isp_lib->handle);
    if (isp_lib->handleCus3a)
        dlclose(isp_lib->handleCus3a);
    if (isp_lib->handleIspAlgo)
        dlclose(isp_lib->handleIspAlgo);
    memset(isp_lib, 0, sizeof(*isp_lib));
}

#endif /* STAR_I6_ISP_LOAD_H */
