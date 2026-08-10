/*
 * star/i6_vpe_load.h -- dlopen loader and function table for MI_VPE
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_vpe.h:
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

#ifndef STAR_I6_VPE_LOAD_H
#define STAR_I6_VPE_LOAD_H

#include <i6_vpe.h>

#include "hal_symbols.h"

typedef struct {
    void *handle;

    /*
     * ISP-side libraries VPE needs resolved before it can run. Not used
     * directly -- held only so they stay mapped and can be closed. See
     * i6_vpe_load for why they must be opened here.
     */
    void *handleIspAlgo;
    void *handleCus3a;
    void *handleIsp;

    int (*fnCreateChannel)(int channel, i6_vpe_chn *config);
    int (*fnDestroyChannel)(int channel);
    int (*fnSetChannelConfig)(int channel, i6_vpe_chn *config);
    int (*fnGetChannelParam)(int channel, i6_vpe_para *config);
    int (*fnSetChannelParam)(int channel, i6_vpe_para *config);
    int (*fnStartChannel)(int channel);
    int (*fnStopChannel)(int channel);

    int (*fnDisablePort)(int channel, int port);
    int (*fnEnablePort)(int channel, int port);
    int (*fnSetPortConfig)(int channel, int port, i6_vpe_port *config);

} i6_vpe_impl;

static inline int i6_vpe_load(i6_vpe_impl *vpe_lib)
{
    /*
     * Load the ISP side first, or MI_VPE_CreateChannel kills the process.
     *
     * libmi_vpe.so leaves MI_ISP_EnableUserspace3A and
     * MI_ISP_DisableUserspace3A undefined, and its DT_NEEDED lists only
     * libc -- so the dynamic loader chains nothing, and with RTLD_LAZY the
     * miss surfaces at first call as a fatal "symbol lookup error" rather
     * than as a dlopen failure we could report -- a fatal exit immediately
     * after MI_VPE_CreateChannel is entered.
     *
     * One level deeper, libmi_isp.so itself needs libcus3a.so (7 symbols)
     * and libispalgo.so (14) and likewise names neither in DT_NEEDED, which
     * is why divinus opens all three in that order before ever touching VPE
     * (i6_isp.h:32-36, loaded at i6_hal.c:53, ten lines ahead of its VPE
     * load). RTLD_GLOBAL is what makes them satisfy the next library's
     * undefined symbols; dlopen refcounts, so hal_isp.c's own MI_ISP
     * binding can open libmi_isp.so again without conflict.
     *
     * The algorithm libraries are best-effort: their symbols are reached
     * through libmi_isp, not from here, and a board missing them is a
     * broken ISP rather than a broken VPE. libmi_isp.so is not optional --
     * without it the next VPE call provably aborts.
     */
    vpe_lib->handleIspAlgo = dlopen("libispalgo.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!vpe_lib->handleIspAlgo)
        HAL_LOG_WARN("i6_vpe: libispalgo.so not loaded (%s) -- ISP algorithms may fail",
                     dlerror());

    vpe_lib->handleCus3a = dlopen("libcus3a.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!vpe_lib->handleCus3a)
        HAL_LOG_WARN("i6_vpe: libcus3a.so not loaded (%s) -- ISP algorithms may fail", dlerror());

    if (!(vpe_lib->handleIsp = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_vpe: failed to load libmi_isp.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    /*
     * Confirm the two symbols VPE actually reaches for. They are resolved
     * lazily inside libmi_vpe.so, so without this check a missing one is a
     * process abort mid-call instead of an error return -- and this loader
     * exists to make missing-SDK cases reportable.
     */
    if (!dlsym(vpe_lib->handleIsp, "MI_ISP_EnableUserspace3A")) {
        HAL_LOG_ERR("i6_vpe: libmi_isp.so lacks MI_ISP_EnableUserspace3A, which "
                    "libmi_vpe.so needs -- mismatched MI libraries?");
        return RSS_ERR_NOTSUP;
    }

    if (!(vpe_lib->handle = dlopen("libmi_vpe.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_vpe: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(vpe_lib->fnCreateChannel = (int(*)(int channel, i6_vpe_chn *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_CreateChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnDestroyChannel = (int(*)(int channel))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_DestroyChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnSetChannelConfig = (int(*)(int channel, i6_vpe_chn *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_SetChannelAttr")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnGetChannelParam = (int(*)(int channel, i6_vpe_para *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_GetChannelParam")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnSetChannelParam = (int(*)(int channel, i6_vpe_para *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_SetChannelParam")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnStartChannel = (int(*)(int channel))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_StartChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnStopChannel = (int(*)(int channel))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_StopChannel")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnDisablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_DisablePort")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnEnablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_EnablePort")))
        return RSS_ERR_NOTSUP;

    if (!(vpe_lib->fnSetPortConfig = (int(*)(int channel, int port, i6_vpe_port *config))
        hal_symbol_load("i6_vpe", vpe_lib->handle, "MI_VPE_SetPortMode")))
        return RSS_ERR_NOTSUP;

    /*
     * MI_VPE_GetPortMode and MI_VPE_SetPortCrop are not bound, but the
     * reason has narrowed. It used to be that i6_vpe_port was
     * reconstructed rather than vendor-supplied, so a size mismatch could
     * write past the caller's frame; it matches MI_VPE_PortMode_t exactly,
     * 16 bytes, field for field, so that objection is gone.
     *
     * What stands is that both were tried and neither behaved as its
     * signature suggests: calling the getter left the port it was asked
     * about with no geometry at all, and the crop came back reading a
     * rectangle it was never passed -- which the headers explain, because
     * MI_VPE_SetPortCrop takes an MI_SYS_WindowRect_t (four u16, 8 bytes)
     * and not a port mode. Nothing on a working path needs either, so the
     * experiment is worth repeating only when a caller wants runtime crop.
     */

    return RSS_OK;
}

static inline void i6_vpe_unload(i6_vpe_impl *vpe_lib)
{
    /* Reverse of the load order: VPE first, then what it depended on. */
    if (vpe_lib->handle)
        dlclose(vpe_lib->handle);
    vpe_lib->handle = NULL;
    if (vpe_lib->handleIsp)
        dlclose(vpe_lib->handleIsp);
    vpe_lib->handleIsp = NULL;
    if (vpe_lib->handleCus3a)
        dlclose(vpe_lib->handleCus3a);
    vpe_lib->handleCus3a = NULL;
    if (vpe_lib->handleIspAlgo)
        dlclose(vpe_lib->handleIspAlgo);
    vpe_lib->handleIspAlgo = NULL;
    memset(vpe_lib, 0, sizeof(*vpe_lib));
}

#endif /* STAR_I6_VPE_LOAD_H */
