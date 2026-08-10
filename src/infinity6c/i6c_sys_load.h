/*
 * infinity6c/i6c_sys_load.h -- dlopen loader for MI_SYS, MI 3.0
 *
 * Counterpart to star/i6_sys_load.h, and deliberately not a variation on it.
 * Every MI_SYS entry point on this generation leads with a SoC id, so a shared
 * table would have to hide an argument behind a macro at each of its call
 * sites -- and would compile silently either way.
 *
 * That last part is why this file exists before anything else does. dlsym
 * resolves by name, so a table built for MI 2.x binds against these libraries
 * without complaint and then calls MI_SYS_Init with whatever happened to be in
 * the first argument register. There is no diagnostic, on either side. Getting
 * the argument lists right once, here, is what the rest of the backend rests
 * on.
 *
 * The signatures are written from the vendor SDK's mi_sys.h for the release the
 * target's libraries were built from; the layouts they refer to live in
 * sigmastar-headers/infinity6c. Member names are this backend's own rather than
 * the MI 2.x loader's, because that one follows divinus and this family follows
 * the vendor.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_SYS_LOAD_H
#define INFINITY6C_I6C_SYS_LOAD_H

#include "hal_symbols.h"

#include "i6c_sys.h"

/*
 * Loaded libraries and the entry points taken from them.
 *
 * The SoC id is spelled out in every signature rather than folded away,
 * because it is the one argument a reader coming from the MI 2.x backend will
 * not expect to be there.
 */
typedef struct {
    void *lib;
    void *lib_cam_os;

    int (*init)(unsigned short soc_id);
    int (*exit)(unsigned short soc_id);
    int (*get_version)(unsigned short soc_id, i6c_sys_version *out);
} i6c_sys_api;

static inline int i6c_sys_load(i6c_sys_api *sys)
{
    /*
     * The wrapper first, and its absence is fatal here -- the opposite of the
     * MI 2.x loader, where it is best-effort. This generation's libmi_* leave
     * the CamOs and CamFs primitives undefined and GLOBAL rather than weak
     * (libmi_sys alone wants four CamOsTsem* entries), and nothing else
     * defines them. Loading it RTLD_GLOBAL ahead of libmi_sys is what makes
     * them resolvable at all.
     *
     * Reported here because this is the last point at which the cause is
     * legible: carry on without it and the failure arrives as a jump into an
     * unresolved PLT slot on the first MI call, naming nothing.
     */
    if (!(sys->lib_cam_os = dlopen("libcam_os_wrapper.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_sys: dlopen(libcam_os_wrapper.so) failed: %s", dlerror());
        HAL_LOG_ERR("i6c_sys: MI 3.0 needs it for CamOs*; libmi_* leave those undefined");
        return RSS_ERR_NOENT;
    }

    if (!(sys->lib = dlopen("libmi_sys.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_sys: dlopen(libmi_sys.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(sys->init =
              (int (*)(unsigned short soc_id))hal_symbol_load("i6c_sys", sys->lib, "MI_SYS_Init")))
        return RSS_ERR_NOTSUP;

    if (!(sys->exit =
              (int (*)(unsigned short soc_id))hal_symbol_load("i6c_sys", sys->lib, "MI_SYS_Exit")))
        return RSS_ERR_NOTSUP;

    if (!(sys->get_version = (int (*)(unsigned short soc_id, i6c_sys_version *out))hal_symbol_load(
              "i6c_sys", sys->lib, "MI_SYS_GetVersion")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_sys_unload(i6c_sys_api *sys)
{
    if (sys->lib)
        dlclose(sys->lib);
    if (sys->lib_cam_os)
        dlclose(sys->lib_cam_os);
    memset(sys, 0, sizeof(*sys));
}

#endif /* INFINITY6C_I6C_SYS_LOAD_H */
