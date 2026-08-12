/*
 * infinity6c/i6c_rgn_load.h -- dlopen loader for MI_RGN, MI 3.0
 *
 * Counterpart to star/i6_rgn_load.h. The ABI it binds to is in
 * sigmastar-headers/infinity6c/i6c_rgn.h; only the dlopen/dlsym half lives here,
 * because it reports through raptor's logger and error codes.
 *
 * Every pointer leads with `unsigned short soc` (MI_U16 u16SocId) -- the leading
 * argument MI 2.x has no equivalent of. dlsym resolves by name, so an MI 2.x
 * table would bind against these symbols without complaint and then call with the
 * handle where the SoC id belongs; a separate table per generation is what keeps
 * that from happening silently.
 *
 * libmi_rgn.so declares no DT_NEEDED, like every MI module, but its dependency
 * closure is libcam_os_wrapper + libmi_sys, both already opened RTLD_GLOBAL by
 * i6c_sys_load before any OSD op can run -- so there is nothing to preload.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_RGN_LOAD_H
#define INFINITY6C_I6C_RGN_LOAD_H

#include "hal_symbols.h"

#include "i6c_rgn.h"

typedef struct {
    void *lib;

    int (*fnInit)(unsigned short soc, i6c_rgn_pal *palette);
    int (*fnDeinit)(unsigned short soc);

    int (*fnCreateRegion)(unsigned short soc, unsigned int handle, i6c_rgn_cnf *config);
    int (*fnDestroyRegion)(unsigned short soc, unsigned int handle);
    int (*fnGetRegionConfig)(unsigned short soc, unsigned int handle, i6c_rgn_cnf *config);

    int (*fnAttachChannel)(unsigned short soc, unsigned int handle, i6c_sys_bind *dest,
                           i6c_rgn_chn *config);
    int (*fnDetachChannel)(unsigned short soc, unsigned int handle, i6c_sys_bind *dest);
    int (*fnGetChannelConfig)(unsigned short soc, unsigned int handle, i6c_sys_bind *dest,
                              i6c_rgn_chn *config);
    int (*fnSetChannelConfig)(unsigned short soc, unsigned int handle, i6c_sys_bind *dest,
                              i6c_rgn_chn *config);

    int (*fnSetBitmap)(unsigned short soc, unsigned int handle, i6c_rgn_bmp *bitmap);
} i6c_rgn_api;

static inline int i6c_rgn_load(i6c_rgn_api *rgn)
{
    if (!(rgn->lib = dlopen("libmi_rgn.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_rgn: dlopen(libmi_rgn.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(rgn->fnInit = (int (*)(unsigned short, i6c_rgn_pal *))hal_symbol_load(
              "i6c_rgn", rgn->lib, "MI_RGN_Init")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnDeinit =
              (int (*)(unsigned short))hal_symbol_load("i6c_rgn", rgn->lib, "MI_RGN_DeInit")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnCreateRegion = (int (*)(unsigned short, unsigned int, i6c_rgn_cnf *))
              hal_symbol_load("i6c_rgn", rgn->lib, "MI_RGN_Create")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnDestroyRegion = (int (*)(unsigned short, unsigned int))hal_symbol_load(
              "i6c_rgn", rgn->lib, "MI_RGN_Destroy")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnGetRegionConfig = (int (*)(unsigned short, unsigned int, i6c_rgn_cnf *))
              hal_symbol_load("i6c_rgn", rgn->lib, "MI_RGN_GetAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnAttachChannel =
              (int (*)(unsigned short, unsigned int, i6c_sys_bind *, i6c_rgn_chn *))hal_symbol_load(
                  "i6c_rgn", rgn->lib, "MI_RGN_AttachToChn")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnDetachChannel =
              (int (*)(unsigned short, unsigned int, i6c_sys_bind *))hal_symbol_load(
                  "i6c_rgn", rgn->lib, "MI_RGN_DetachFromChn")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnGetChannelConfig =
              (int (*)(unsigned short, unsigned int, i6c_sys_bind *, i6c_rgn_chn *))hal_symbol_load(
                  "i6c_rgn", rgn->lib, "MI_RGN_GetDisplayAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnSetChannelConfig =
              (int (*)(unsigned short, unsigned int, i6c_sys_bind *, i6c_rgn_chn *))hal_symbol_load(
                  "i6c_rgn", rgn->lib, "MI_RGN_SetDisplayAttr")))
        return RSS_ERR_NOTSUP;

    if (!(rgn->fnSetBitmap = (int (*)(unsigned short, unsigned int, i6c_rgn_bmp *))hal_symbol_load(
              "i6c_rgn", rgn->lib, "MI_RGN_SetBitMap")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_rgn_unload(i6c_rgn_api *rgn)
{
    if (rgn->lib)
        dlclose(rgn->lib);
    memset(rgn, 0, sizeof(*rgn));
}

#endif /* INFINITY6C_I6C_RGN_LOAD_H */
