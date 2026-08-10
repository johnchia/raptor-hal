/*
 * star/i6_vif_load.h -- dlopen loader and function table for MI_VIF
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_vif.h:
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

#ifndef STAR_I6_VIF_LOAD_H
#define STAR_I6_VIF_LOAD_H

#include <i6_vif.h>

#include "hal_symbols.h"

typedef struct {
    void *handle;

    int (*fnDisableDevice)(int device);
    int (*fnEnableDevice)(int device);
    int (*fnSetDeviceConfig)(int device, i6_vif_dev *config);

    int (*fnDisablePort)(int channel, int port);
    int (*fnEnablePort)(int channel, int port);
    int (*fnSetPortConfig)(int channel, int port, i6_vif_port *config);
} i6_vif_impl;

static inline int i6_vif_load(i6_vif_impl *vif_lib)
{
    if (!(vif_lib->handle = dlopen("libmi_vif.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_vif: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(vif_lib->fnDisableDevice = (int(*)(int device))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_DisableDev")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnEnableDevice = (int(*)(int device))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_EnableDev")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnSetDeviceConfig = (int(*)(int device, i6_vif_dev *config))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_SetDevAttr")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnDisablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_DisableChnPort")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnEnablePort = (int(*)(int channel, int port))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_EnableChnPort")))
        return RSS_ERR_NOTSUP;

    if (!(vif_lib->fnSetPortConfig = (int(*)(int channel, int port, i6_vif_port *config))
        hal_symbol_load("i6_vif", vif_lib->handle, "MI_VIF_SetChnPortAttr")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_vif_unload(i6_vif_impl *vif_lib)
{
    if (vif_lib->handle)
        dlclose(vif_lib->handle);
    vif_lib->handle = NULL;
    memset(vif_lib, 0, sizeof(*vif_lib));
}

#endif /* STAR_I6_VIF_LOAD_H */
