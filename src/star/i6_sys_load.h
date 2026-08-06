/*
 * star/i6_sys_load.h -- dlopen loader and function table for MI_SYS
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_sys.h:
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

#ifndef STAR_I6_SYS_LOAD_H
#define STAR_I6_SYS_LOAD_H

#include <i6_sys.h>

#include "i6_symbols.h"

typedef struct {
    void *handle, *handleCamOsWrapper;

    int (*fnExit)(void);
    int (*fnGetVersion)(i6_sys_ver *version);
    int (*fnInit)(void);

    int (*fnBind)(i6_sys_bind *source, i6_sys_bind *dest,
        unsigned int srcFps, unsigned int dstFps);
    int (*fnBindExt)(i6_sys_bind *source, i6_sys_bind *dest, unsigned int srcFps,
        unsigned int dstFps, i6_sys_link link, unsigned int linkParam);
    int (*fnSetOutputDepth)(i6_sys_bind *bind, unsigned int usrDepth, unsigned int bufDepth);
    int (*fnUnbind)(i6_sys_bind *source, i6_sys_bind *dest);

    /*
     * Media-clock access. Not from divinus -- it binds neither -- but needed
     * by rss_hal_ops_t's sys_get_timestamp/sys_rebase_timestamp, which
     * rvd_frame_loop.c uses to publish the media-clock-to-UTC mapping for SEI
     * timecodes.
     *
     * The signatures are read off libmi_sys.so rather than guessed, because
     * getting the arity wrong here writes through a bogus pointer instead of
     * failing. Each of these userspace entry points is a thin ioctl wrapper
     * that spills its arguments and then stores {payload size, user address}
     * for the kernel, so the disassembly states both the argument count and
     * the size of the pointee:
     *
     *   MI_SYS_Init         no spills at all               -> 0 args, and
     *                       matches fnInit(void) above, which confirms the
     *                       method reads true
     *   MI_SYS_GetVersion   one spill, size field 128       -> 1 pointer to
     *                       128 bytes == sizeof(i6_sys_ver), and this call is
     *                       known-good on hardware
     *   MI_SYS_GetCurPts    one spill, size field 8         -> 1 pointer to
     *                       8 bytes, i.e. unsigned long long *
     *   MI_SYS_InitPtsBase  strd r0,r1 -> a register pair   -> one u64 passed
     *   MI_SYS_SyncPts      strd r0,r1 -> a register pair       by value
     *
     * Note this differs by SoC family: waybeam calls MI_SYS_GetCurPts with a
     * leading device argument (maruko_framing_stab.c:626), which is Mercury6.
     * On Infinity6E that form would pass 0 as the output pointer.
     */
    int (*fnGetCurrentPts)(unsigned long long *pts);
    int (*fnInitPtsBase)(unsigned long long ptsBase);
    int (*fnSyncPts)(unsigned long long pts);

    /*
     * Frame access on a channel's output port. Every arity here was read off
     * libmi_sys.so and independently matches waybeam's Infinity6E typedefs
     * (star6e_framing_stab.c) -- which matters because the same calls take a
     * leading chip id on i6c/m6 and do not on i6/i3. Both references and the
     * disassembly agree on that split, so these take no chip id:
     *
     *   MI_SYS_GetFd                 2 args, payload 20 = port(16) + fd(4)
     *   MI_SYS_CloseFd               1 arg,  payload 4
     *   MI_SYS_ChnOutputPortGetBuf   3 args, payload 304, copies 272 out
     *   MI_SYS_ChnOutputPortPutBuf   1 arg,  payload 4
     *   MI_SYS_FlushInvCache         2 args, payload 8  = va(4) + size(4)
     *   MI_SYS_Va2Pa                 2 args, payload 16 = va(4) + pad + pa(8)
     *
     * fnGetFd returns a descriptor that select()/poll() marks readable when a
     * frame is queued, which is how both references avoid polling GetBuf.
     */
    int (*fnGetFd)(i6_sys_bind *port, int *fd);
    int (*fnCloseFd)(int fd);
    int (*fnGetOutputBuf)(i6_sys_bind *port, i6_sys_bufinfo *buf, int *handle);
    int (*fnPutOutputBuf)(int handle);
    int (*fnFlushInvCache)(void *virAddr, unsigned int size);
    int (*fnVa2Pa)(void *virAddr, unsigned long long *phyAddr);
} i6_sys_impl;

static inline int i6_sys_load(i6_sys_impl *sys_lib)
{
    sys_lib->handleCamOsWrapper = dlopen("libcam_os_wrapper.so", RTLD_LAZY | RTLD_GLOBAL);

    if (!(sys_lib->handle = dlopen("libmi_sys.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_sys: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(sys_lib->fnExit = (int(*)(void))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_Exit")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnGetVersion = (int(*)(i6_sys_ver *version))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_GetVersion")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnInit = (int(*)(void))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_Init")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnBind = (int(*)(i6_sys_bind *source, i6_sys_bind *dest,
        unsigned int srcFps, unsigned int dstFps))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_BindChnPort")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnBindExt = (int(*)(i6_sys_bind *source, i6_sys_bind *dest, unsigned int srcFps,
        unsigned int dstFps, i6_sys_link link, unsigned int linkParam))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_BindChnPort2")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnSetOutputDepth = (int(*)(i6_sys_bind *bind, unsigned int usrDepth,
        unsigned int bufDepth))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_SetChnOutputPortDepth")))
        return RSS_ERR_NOTSUP;

    if (!(sys_lib->fnUnbind = (int(*)(i6_sys_bind *source, i6_sys_bind *dest))
        hal_symbol_load("i6_sys", sys_lib->handle, "MI_SYS_UnBindChnPort")))
        return RSS_ERR_NOTSUP;

    /*
     * Optional: missing media-clock entry points cost SEI timecodes, not
     * streaming, so resolve them with a bare dlsym and let the callers
     * null-check. Failing the whole load over them would trade a cosmetic
     * loss for a dead pipeline.
     */
    sys_lib->fnGetCurrentPts = (int(*)(unsigned long long *pts))
        dlsym(sys_lib->handle, "MI_SYS_GetCurPts");
    sys_lib->fnInitPtsBase = (int(*)(unsigned long long ptsBase))
        dlsym(sys_lib->handle, "MI_SYS_InitPtsBase");
    sys_lib->fnSyncPts = (int(*)(unsigned long long pts))
        dlsym(sys_lib->handle, "MI_SYS_SyncPts");

    /*
     * Also optional, for the same reason. raptor's video path binds
     * VIF -> VPE -> VENC in hardware and reads encoded streams from VENC, so
     * nothing on the streaming path calls these -- they serve frame-level
     * consumers (the 2b VIF bring-up check, and later ISP/IPU work). Making
     * them fatal would let a firmware missing one symbol take down streaming
     * that never needed it. Callers null-check.
     */
    sys_lib->fnGetFd = (int(*)(i6_sys_bind *port, int *fd))
        dlsym(sys_lib->handle, "MI_SYS_GetFd");
    sys_lib->fnCloseFd = (int(*)(int fd))
        dlsym(sys_lib->handle, "MI_SYS_CloseFd");
    sys_lib->fnGetOutputBuf = (int(*)(i6_sys_bind *port, i6_sys_bufinfo *buf, int *handle))
        dlsym(sys_lib->handle, "MI_SYS_ChnOutputPortGetBuf");
    sys_lib->fnPutOutputBuf = (int(*)(int handle))
        dlsym(sys_lib->handle, "MI_SYS_ChnOutputPortPutBuf");
    sys_lib->fnFlushInvCache = (int(*)(void *virAddr, unsigned int size))
        dlsym(sys_lib->handle, "MI_SYS_FlushInvCache");
    sys_lib->fnVa2Pa = (int(*)(void *virAddr, unsigned long long *phyAddr))
        dlsym(sys_lib->handle, "MI_SYS_Va2Pa");

    return RSS_OK;
}

static inline void i6_sys_unload(i6_sys_impl *sys_lib)
{
    if (sys_lib->handle)
        dlclose(sys_lib->handle);
    sys_lib->handle = NULL;
    if (sys_lib->handleCamOsWrapper)
        dlclose(sys_lib->handleCamOsWrapper);
    sys_lib->handleCamOsWrapper = NULL;
    memset(sys_lib, 0, sizeof(*sys_lib));
}

#endif /* STAR_I6_SYS_LOAD_H */
