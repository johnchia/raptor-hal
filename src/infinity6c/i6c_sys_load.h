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

    /*
     * The media clock, which rvd samples alongside the wall clock to publish the
     * mapping consumers stamp absolute capture times from (rvd_frame_loop.c's
     * publish_utc_mapping, and the SEI timecodes that rest on it).
     *
     * Optional, bound by dlsym: a library without them costs the timecodes and
     * nothing else, so it is not worth failing bring-up over.
     *
     * The arities are read off libmi_sys.so rather than taken from the MI 2.x
     * loader, and they differ from it -- which is the whole reason this comment is
     * here. star/i6_sys_load.h records the question ("waybeam calls
     * MI_SYS_GetCurPts with a leading device argument, which is Mercury6") and
     * this family answers it: every one of the three leads with the SoC id, the
     * same as MI_SYS_Init.
     *
     *   MI_SYS_GetCurPts     strh.w r0, [sp]      -> r0 is the SoC id halfword
     *                        strd of r1 sign-extended, payload size 8
     *                        -> r1 is the pointer, to 8 bytes
     *   MI_SYS_InitPtsBase   strh.w r0, [sp, #8]  -> SoC id
     *                        strd r2, r3, [sp]    -> the u64 passed by value,
     *                        in r2:r3 because AAPCS aligns it to an even pair
     *                        past the leading halfword in r0
     *   MI_SYS_SyncPts       identical to InitPtsBase
     *
     * Getting this wrong on the MI 2.x form -- one argument, the pointer in r0 --
     * would pass the SoC id as the output address and write the clock through it.
     */
    int (*get_cur_pts)(unsigned short soc_id, unsigned long long *pts);
    int (*init_pts_base)(unsigned short soc_id, unsigned long long base);
    int (*sync_pts)(unsigned short soc_id, unsigned long long pts);
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

    if (!(sys->bind_ext =
              (int (*)(unsigned short, i6c_sys_bind *, i6c_sys_bind *, unsigned int, unsigned int,
                       i6c_sys_link, unsigned int))hal_symbol_load("i6c_sys", sys->lib,
                                                                   "MI_SYS_BindChnPort2")))
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

    /* The media clock, optional; see the declarations. dlsym rather than
     * hal_symbol_load, which would log a miss as an error, and a library without
     * these is not in error -- it just cannot stamp absolute time. */
    sys->get_cur_pts =
        (int (*)(unsigned short, unsigned long long *))dlsym(sys->lib, "MI_SYS_GetCurPts");
    sys->init_pts_base =
        (int (*)(unsigned short, unsigned long long))dlsym(sys->lib, "MI_SYS_InitPtsBase");
    sys->sync_pts = (int (*)(unsigned short, unsigned long long))dlsym(sys->lib, "MI_SYS_SyncPts");

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
