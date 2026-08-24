/*
 * star/i6_aud_load.h -- dlopen loader and function table for MI_AUD
 *
 * The ABI declarations this binds to are in sigmastar-headers; only the
 * dlopen/dlsym half lives here, because it reports through raptor's logger
 * and error codes. Adaptations from divinus's src/hal/star/i6_aud.h:
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

#ifndef STAR_I6_AUD_LOAD_H
#define STAR_I6_AUD_LOAD_H

#include <i6_aud.h>

#include "hal_symbols.h"

typedef struct {
    void *handle;

    int (*fnDisableDevice)(int device);
    int (*fnEnableDevice)(int device);
    int (*fnSetDeviceConfig)(int device, i6_aud_cnf *config);

    int (*fnDisableChannel)(int device, int channel);
    int (*fnEnableChannel)(int device, int channel);

    int (*fnSetMute)(int device, int channel, char active);
    /* Despite the name this takes an *index* into a per-device analog gain
     * table, not a decibel value -- see star_audio_gain_index in
     * hal_audio.c. Both references pass dB-shaped numbers here, which is
     * why the constants are named for the stage and not for the call.
     *
     * It backs audio_set_GAIN, not audio_set_volume: this is the preamp,
     * and the digital trim volume drives lives in fnSetChannelParam. */
    int (*fnSetVolume)(int device, int channel, int level);

    /*
     * The other gain stage -- see i6_aud_chn_para. OPTIONAL, and the only
     * pair here that is: everything above is what capture needs, so a
     * library missing any of it is a library this backend cannot use, while
     * a library missing these two just has no digital trim. Both are NULL
     * then, and hal_audio_set_gain answers RSS_ERR_NOTSUP exactly as it did
     * before the pair was bound at all.
     */
    int (*fnSetChannelParam)(int device, int channel, i6_aud_chn_para *param);
    int (*fnGetChannelParam)(int device, int channel, i6_aud_chn_para *param);

    int (*fnGetFrame)(int device, int channel, i6_aud_frm *frame, i6_aud_efrm *encFrame,
                      int millis);
    int (*fnFreeFrame)(int device, int channel, i6_aud_frm *frame, i6_aud_efrm *encFrame);
} i6_aud_impl;

/*
 * libmi_ai.so needs libcam_os_wrapper and libmi_sys, and declares no
 * DT_NEEDED for either, so both must already be RTLD_GLOBAL in the
 * process. i6_sys_load does that, and hal_audio.c calls it first.
 *
 * RTLD_LAZY, like every other loader here: libmi_ai.so's optional
 * algorithm symbols are weak undefined and PLT-lazy, so binding eagerly
 * buys nothing and only risks turning an unused NULL into a load
 * failure.
 */
static inline int i6_aud_load(i6_aud_impl *aud_lib)
{
    if (!(aud_lib->handle = dlopen("libmi_ai.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6_aud: dlopen(libmi_ai.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(aud_lib->fnDisableDevice =
              (int (*)(int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_Disable")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnEnableDevice =
              (int (*)(int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_Enable")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnSetDeviceConfig = (int (*)(int, i6_aud_cnf *))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_SetPubAttr")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnDisableChannel =
              (int (*)(int, int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_DisableChn")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnEnableChannel =
              (int (*)(int, int))hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_EnableChn")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnSetMute = (int (*)(int, int, char))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_SetMute")))
        return RSS_ERR_NOTSUP;

    /* MI_AI_SetVqeVolume, not MI_AI_SetVolume: both references use the
     * VQE variant and it works with the VQE algorithm libraries absent,
     * because the volume is a codec register write rather than part of
     * the sound-quality pipeline. */
    if (!(aud_lib->fnSetVolume = (int (*)(int, int, int))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_SetVqeVolume")))
        return RSS_ERR_NOTSUP;

    /*
     * Optional, so plain dlsym rather than hal_symbol_load: the helper logs
     * an error on a miss, and a miss here is not one -- the backend loses
     * the digital trim and keeps capturing. Absence is reported once, by
     * hal_audio_init, as information rather than as a fault.
     */
    aud_lib->fnSetChannelParam = (int (*)(int, int, i6_aud_chn_para *))dlsym(
        aud_lib->handle, "MI_AI_SetChnParam");
    aud_lib->fnGetChannelParam = (int (*)(int, int, i6_aud_chn_para *))dlsym(
        aud_lib->handle, "MI_AI_GetChnParam");

    if (!(aud_lib->fnGetFrame = (int (*)(int, int, i6_aud_frm *, i6_aud_efrm *, int))
              hal_symbol_load("i6_aud", aud_lib->handle, "MI_AI_GetFrame")))
        return RSS_ERR_NOTSUP;

    if (!(aud_lib->fnFreeFrame = (int (*)(int, int, i6_aud_frm *, i6_aud_efrm *))hal_symbol_load(
              "i6_aud", aud_lib->handle, "MI_AI_ReleaseFrame")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6_aud_unload(i6_aud_impl *aud_lib)
{
    if (aud_lib->handle)
        dlclose(aud_lib->handle);
    memset(aud_lib, 0, sizeof(*aud_lib));
}

#endif /* STAR_I6_AUD_LOAD_H */
