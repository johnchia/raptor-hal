/*
 * star/i6_venc_load.h -- dlopen loader and function table for MI_VENC
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_venc.h:
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

#ifndef STAR_I6_VENC_LOAD_H
#define STAR_I6_VENC_LOAD_H

#include <i6_venc.h>

#include "i6_symbols.h"

typedef struct {
    void *handle;

    int (*fnCreateChannel)(int channel, i6_venc_chn *config);
    int (*fnDestroyChannel)(int channel);
    int (*fnGetChannelConfig)(int channel, i6_venc_chn *config);
    int (*fnGetChannelDeviceId)(int channel, unsigned int *device);
    int (*fnResetChannel)(int channel);
    int (*fnSetChannelConfig)(int channel, i6_venc_chn *config);

    int (*fnFreeDescriptor)(int channel);
    int (*fnGetDescriptor)(int channel);

    int (*fnGetJpegParam)(int channel, i6_venc_jpg *param);
    int (*fnSetJpegParam)(int channel, i6_venc_jpg *param);

    int (*fnFreeStream)(int channel, i6_venc_strm *stream);
    int (*fnGetStream)(int channel, i6_venc_strm *stream, unsigned int timeout);

    int (*fnQuery)(int channel, i6_venc_stat *stats);

    int (*fnSetSourceConfig)(int channel, i6_venc_src_conf *config);

    int (*fnRequestIdr)(int channel, char instant);
    int (*fnStartReceiving)(int channel);
    int (*fnStartReceivingEx)(int channel, int *count);
    int (*fnStopReceiving)(int channel);
} i6_venc_impl;

static inline int i6_venc_load(i6_venc_impl *venc_lib)
{
    if (!(venc_lib->handle = dlopen("libmi_venc.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_venc: failed to load library: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(venc_lib->fnCreateChannel = (int(*)(int channel, i6_venc_chn *config))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_CreateChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnDestroyChannel = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_DestroyChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetChannelConfig = (int(*)(int channel, i6_venc_chn *config))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetChnAttr")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetChannelDeviceId = (int(*)(int channel, unsigned int *device))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetChnDevid")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnResetChannel = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_ResetChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnSetChannelConfig = (int(*)(int channel, i6_venc_chn *config))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_SetChnAttr")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnFreeDescriptor = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_CloseFd")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetDescriptor = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetFd")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetJpegParam = (int(*)(int channel, i6_venc_jpg *param))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetJpegParam")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnSetJpegParam = (int(*)(int channel, i6_venc_jpg *param))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_SetJpegParam")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnFreeStream = (int(*)(int channel, i6_venc_strm *stream))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_ReleaseStream")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnGetStream = (int(*)(int channel, i6_venc_strm *stream,
        unsigned int timeout))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_GetStream")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnQuery = (int(*)(int channel, i6_venc_stat *stats))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_Query")))
        return RSS_ERR_NOTSUP;

    /* Optional — see the file header. Absence is not an error. */
    venc_lib->fnSetSourceConfig = (int(*)(int channel, i6_venc_src_conf *config))
        dlsym(venc_lib->handle, "MI_VENC_SetInputSourceConfig");

    if (!(venc_lib->fnRequestIdr = (int(*)(int channel, char instant))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_RequestIdr")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnStartReceiving = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_StartRecvPic")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnStartReceivingEx = (int(*)(int channel, int *count))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_StartRecvPicEx")))
        return RSS_ERR_NOTSUP;

    if (!(venc_lib->fnStopReceiving = (int(*)(int channel))
        hal_symbol_load("i6_venc", venc_lib->handle, "MI_VENC_StopRecvPic")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_venc_unload(i6_venc_impl *venc_lib)
{
    if (venc_lib->handle)
        dlclose(venc_lib->handle);
    venc_lib->handle = NULL;
    memset(venc_lib, 0, sizeof(*venc_lib));
}

#endif /* STAR_I6_VENC_LOAD_H */
