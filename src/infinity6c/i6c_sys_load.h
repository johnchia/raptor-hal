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
    int (*get_version)(unsigned short soc_id, i6c_sys_ver *out);

    /*
     * The datapath half. Binding is what makes a stage feed the next one, so
     * these are the calls that turn a set of configured modules into a
     * pipeline.
     *
     * bind_ext is the one used throughout rather than the plain MI_SYS_BindChnPort,
     * because the frame rates and the link type are not optional in practice:
     * VIF -> ISP -> SCL wants REALTIME so no frame is buffered between stages,
     * while SCL -> VENC wants RING for H.26x, and the plain bind chooses neither.
     */
    int (*bind_ext)(unsigned short soc_id, i6c_sys_bind *src, i6c_sys_bind *dest,
                    unsigned int src_fps, unsigned int dest_fps, i6c_sys_link link,
                    unsigned int link_param);
    int (*unbind)(unsigned short soc_id, i6c_sys_bind *src, i6c_sys_bind *dest);

    /*
     * Private pools, which are not an optimisation here. A stage that hands
     * frames onward through a ring needs one configured before it starts, and
     * MI allocates from the shared heap otherwise -- so this is part of bring-up
     * rather than tuning.
     */
    int (*config_pool)(unsigned short soc_id, i6c_sys_pool *config);

    /* How many frames a port may hold for a user and for the next stage. */
    int (*set_output_depth)(unsigned short soc_id, i6c_sys_bind *port, unsigned int user_depth,
                            unsigned int buf_depth);
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

    if (!(sys->get_version = (int (*)(unsigned short soc_id, i6c_sys_ver *out))hal_symbol_load(
              "i6c_sys", sys->lib, "MI_SYS_GetVersion")))
        return RSS_ERR_NOTSUP;

    if (!(sys->bind_ext = (int (*)(unsigned short, i6c_sys_bind *, i6c_sys_bind *, unsigned int,
                                   unsigned int, i6c_sys_link, unsigned int))
              hal_symbol_load("i6c_sys", sys->lib, "MI_SYS_BindChnPort2")))
        return RSS_ERR_NOTSUP;

    if (!(sys->unbind = (int (*)(unsigned short, i6c_sys_bind *, i6c_sys_bind *))hal_symbol_load(
              "i6c_sys", sys->lib, "MI_SYS_UnBindChnPort")))
        return RSS_ERR_NOTSUP;

    if (!(sys->config_pool = (int (*)(unsigned short, i6c_sys_pool *))hal_symbol_load(
              "i6c_sys", sys->lib, "MI_SYS_ConfigPrivateMMAPool")))
        return RSS_ERR_NOTSUP;

    if (!(sys->set_output_depth =
              (int (*)(unsigned short, i6c_sys_bind *, unsigned int, unsigned int))hal_symbol_load(
                  "i6c_sys", sys->lib, "MI_SYS_SetChnOutputPortDepth")))
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
