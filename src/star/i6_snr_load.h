/*
 * star/i6_snr_load.h -- dlopen loader and function table for MI_SNR
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_snr.h:
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

#ifndef STAR_I6_SNR_LOAD_H
#define STAR_I6_SNR_LOAD_H

#include <i6_snr.h>

#include "i6_symbols.h"

typedef struct {
    void *handle;

    int (*fnDisable)(unsigned int sensor);
    int (*fnEnable)(unsigned int sensor);

    int (*fnGetFramerate)(unsigned int sensor, unsigned int *framerate);
    int (*fnSetFramerate)(unsigned int sensor, unsigned int framerate);
    int (*fnSetOrientation)(unsigned int sensor, unsigned char mirror, unsigned char flip);

    int (*fnGetPadInfo)(unsigned int sensor, i6_snr_pad *info);
    int (*fnGetPlaneInfo)(unsigned int sensor, unsigned int index, i6_snr_plane *info);
    int (*fnSetPlaneMode)(unsigned int sensor, unsigned char active);

    int (*fnCurrentResolution)(unsigned int sensor, unsigned char *index, i6_snr_res *resolution);
    int (*fnGetResolution)(unsigned int sensor, unsigned char index, i6_snr_res *resolution);
    int (*fnGetResolutionCount)(unsigned int sensor, unsigned int *count);
    int (*fnSetResolution)(unsigned int sensor, unsigned char index);

    int (*fnCustomFunction)(unsigned int sensor, unsigned int command, unsigned int size,
        void *data, int drvOrUsr);
} i6_snr_impl;

static inline int i6_snr_load(i6_snr_impl *snr_lib)
{
    if (!(snr_lib->handle = dlopen("libmi_sensor.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_snr: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(snr_lib->fnDisable = (int(*)(unsigned int sensor))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_Disable")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnEnable = (int(*)(unsigned int sensor))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_Enable")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetFramerate = (int(*)(unsigned int sensor, unsigned int *framerate))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetFps")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetFramerate = (int(*)(unsigned int sensor, unsigned int framerate))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetFps")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetOrientation = (int(*)(unsigned int sensor, unsigned char mirror,
        unsigned char flip))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetOrien")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetPadInfo = (int(*)(unsigned int sensor, i6_snr_pad *info))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetPadInfo")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetPlaneInfo = (int(*)(unsigned int sensor, unsigned int index,
        i6_snr_plane *info))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetPlaneInfo")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetPlaneMode = (int(*)(unsigned int sensor, unsigned char active))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetPlaneMode")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnCurrentResolution = (int(*)(unsigned int sensor, unsigned char *index,
        i6_snr_res *resolution))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetCurRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetResolution = (int(*)(unsigned int sensor, unsigned char index,
        i6_snr_res *resolution))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_GetRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnGetResolutionCount = (int(*)(unsigned int sensor, unsigned int *count))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_QueryResCount")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnSetResolution = (int(*)(unsigned int sensor, unsigned char index))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_SetRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr_lib->fnCustomFunction = (int(*)(unsigned int sensor, unsigned int command,
        unsigned int size, void *data, int drvOrUsr))
        hal_symbol_load("i6_snr", snr_lib->handle, "MI_SNR_CustFunction")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_snr_unload(i6_snr_impl *snr_lib)
{
    if (snr_lib->handle)
        dlclose(snr_lib->handle);
    snr_lib->handle = NULL;
    memset(snr_lib, 0, sizeof(*snr_lib));
}

#endif /* STAR_I6_SNR_LOAD_H */
