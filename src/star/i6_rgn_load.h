/*
 * star/i6_rgn_load.h -- dlopen loader and function table for MI_RGN
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_rgn.h:
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

#ifndef STAR_I6_RGN_LOAD_H
#define STAR_I6_RGN_LOAD_H

#include <i6_rgn.h>

#include "hal_symbols.h"

typedef struct {
    void *handle;

    int (*fnInit)(i6_rgn_pal *palette);
    int (*fnDeinit)(void);

    int (*fnCreateRegion)(unsigned int handle, i6_rgn_cnf *config);
    int (*fnDestroyRegion)(unsigned int handle);
    int (*fnGetRegionConfig)(unsigned int handle, i6_rgn_cnf *config);

    int (*fnAttachChannel)(unsigned int handle, i6_sys_bind *dest, i6_rgn_chn *config);
    int (*fnDetachChannel)(unsigned int handle, i6_sys_bind *dest);
    int (*fnGetChannelConfig)(unsigned int handle, i6_sys_bind *dest, i6_rgn_chn *config);
    int (*fnSetChannelConfig)(unsigned int handle, i6_sys_bind *dest, i6_rgn_chn *config);

    int (*fnSetBitmap)(unsigned int handle, i6_rgn_bmp *bitmap);
} i6_rgn_impl;

/*
 * libmi_rgn.so declares no cross-library DT_NEEDED, like every other MI
 * module, but its dependency closure is only libcam_os_wrapper +
 * libmi_sys -- both of which i6_sys_load has already opened RTLD_GLOBAL
 * by the time any OSD op can run. So there is nothing to preload here.
 */
static inline int i6_rgn_load(i6_rgn_impl *rgn_lib)
{
    if (!(rgn_lib->handle = dlopen("libmi_rgn.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_rgn: dlopen(libmi_rgn.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(rgn_lib->fnInit = (int (*)(i6_rgn_pal *))hal_symbol_load("i6_rgn", rgn_lib->handle,
                                                                  "MI_RGN_Init")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnDeinit =
              (int (*)(void))hal_symbol_load("i6_rgn", rgn_lib->handle, "MI_RGN_DeInit")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnCreateRegion = (int (*)(unsigned int, i6_rgn_cnf *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_Create")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnDestroyRegion = (int (*)(unsigned int))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_Destroy")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnGetRegionConfig = (int (*)(unsigned int, i6_rgn_cnf *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_GetAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnAttachChannel =
              (int (*)(unsigned int, i6_sys_bind *, i6_rgn_chn *))hal_symbol_load(
                  "i6_rgn", rgn_lib->handle, "MI_RGN_AttachToChn")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnDetachChannel = (int (*)(unsigned int, i6_sys_bind *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_DetachFromChn")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnGetChannelConfig =
              (int (*)(unsigned int, i6_sys_bind *, i6_rgn_chn *))hal_symbol_load(
                  "i6_rgn", rgn_lib->handle, "MI_RGN_GetDisplayAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnSetChannelConfig =
              (int (*)(unsigned int, i6_sys_bind *, i6_rgn_chn *))hal_symbol_load(
                  "i6_rgn", rgn_lib->handle, "MI_RGN_SetDisplayAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn_lib->fnSetBitmap = (int (*)(unsigned int, i6_rgn_bmp *))hal_symbol_load(
              "i6_rgn", rgn_lib->handle, "MI_RGN_SetBitMap")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_rgn_unload(i6_rgn_impl *rgn_lib)
{
    if (rgn_lib->handle)
        dlclose(rgn_lib->handle);
    memset(rgn_lib, 0, sizeof(*rgn_lib));
}
#endif /* STAR_I6_RGN_LOAD_H */
