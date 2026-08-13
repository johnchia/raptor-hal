/*
 * infinity6c/i6c_aud_load.h -- dlopen loader for MI_AI, MI 3.0
 *
 * Counterpart to star/i6_aud_load.h, sharing nothing with it but the pattern:
 * MI 3.0's audio input module has a different call set, a different addressing
 * scheme and a different division of labour. i6c_aud.h explains the five
 * differences that matter; this file is only the dlopen/dlsym half, which lives
 * here rather than in the ABI header because it reports through raptor's logger
 * and error codes.
 *
 * ON THE DEVICE ARGUMENT
 *
 * Every entry point that names a device takes it as `unsigned int` with the SoC
 * id in the high halfword, composed by I6C_DEV_ID(). MI_AI is in the majority
 * here -- only MI_SYS and MI_RGN take the id separately -- and on a single-die
 * part the two spellings are indistinguishable, which is exactly why the
 * composed one is used everywhere.
 *
 * THE FOUR SYMBOLS THAT ARE NOT LOADED, AND ARE NOT MISSING
 *
 * libmi_ai.so exports 22 functions, of which four are `movs r0, #0; bx lr` --
 * four-byte stubs that report success and do nothing:
 *
 *     MI_AI_OpenWithCfgFile   MI_AI_DupChnGroup
 *     MI_AI_SetIfMute         MI_AI_GetIfMute
 *
 * They are deliberately absent from the table below. A stub that returns
 * MI_SUCCESS is worse than an absent symbol, because hal_symbol_load cannot tell
 * it from a working one and the caller has no way to notice: binding
 * MI_AI_SetIfMute would give a mute op that logs success and passes audio.
 * Interface mute is therefore unavailable on this library, and hal_audio.c mutes
 * through MI_AI_SetMute -- the DPGA's mute, which is real.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_AUD_LOAD_H
#define INFINITY6C_I6C_AUD_LOAD_H

#include "hal_symbols.h"

#include "i6c_aud.h"

typedef struct {
    void *lib;

    /*
     * Device lifecycle. `open` is MI_AI_Open, which both configures and starts
     * the device -- MI 2.x's SetPubAttr/Enable pair collapsed into one call, so
     * there is no configured-but-stopped state to hold.
     */
    int (*open)(unsigned int device, const i6c_aud_cnf *config);
    int (*close)(unsigned int device);
    int (*get_attr)(unsigned int device, i6c_aud_cnf *config);

    /*
     * Point the device's multiplexer at physical inputs. Must follow open, and
     * the vendor is explicit that a device "does not support dynamic Attach
     * Interface" -- so this happens once per open and a change of input means
     * closing the device.
     */
    int (*attach_if)(unsigned int device, const i6c_aud_if *ifaces, unsigned char count);

    int (*enable_grp)(unsigned int device, unsigned char group);
    int (*disable_grp)(unsigned int device, unsigned char group);

    /*
     * One period in, and back again. `echo` is the AEC reference and may be NULL
     * when it is not wanted, but not at the same time as `data` -- MI_AI_Read
     * ORs the two pointers and rejects the pair if both are NULL.
     *
     * millis: -1 blocks indefinitely, 0 polls, >0 blocks that many milliseconds.
     */
    int (*read)(unsigned int device, unsigned char group, i6c_aud_frm *data, i6c_aud_frm *echo,
                int millis);
    int (*release)(unsigned int device, unsigned char group, i6c_aud_frm *data, i6c_aud_frm *echo);

    /*
     * DPGA gain -- the digital stage, one entry per physical channel in the
     * group, in whole dB over [-60, 30]. The array is sized by the sound mode,
     * so a mono group takes one element and a stereo group two.
     */
    int (*set_gain)(unsigned int device, unsigned char group, const signed char *gains,
                    unsigned char count);
    int (*get_gain)(unsigned int device, unsigned char group, signed char *gains,
                    unsigned char *count);

    /*
     * DPGA mute, likewise per physical channel. MI_BOOL is a byte, so this is a
     * byte array and not a bitmask.
     */
    int (*set_mute)(unsigned int device, unsigned char group, const unsigned char *mutes,
                    unsigned char count);

    /*
     * Interface gain -- the analog stage, addressed by interface rather than by
     * device, because it belongs to the peripheral and not to the DMA writer.
     * The units are hardware steps and the range is per interface and per chip;
     * see I6C_AUD_IF_GAIN_MAX_* in infinity6c_state.h.
     */
    int (*set_if_gain)(i6c_aud_if iface, signed char left, signed char right);
    int (*get_if_gain)(i6c_aud_if iface, signed char *left, signed char *right);
} i6c_aud_api;

/*
 * libmi_ai.so names only libc as NEEDED, yet leaves CamOs*, MI_SYS_* and
 * _MI_PRINT_GetDebugLevel undefined and GLOBAL -- six CamOs entries (MemAlloc,
 * MemRelease, RwsemInit, RwsemDownWrite, RwsemUpWrite, TsemDown/Up), MI_SYS_GetFd,
 * MI_SYS_CloseFd and the channel-output pair, and the debug-level query every one
 * of its own log lines starts with. Nothing else defines them, so
 * libcam_os_wrapper.so, libmi_sys.so and libmi_common.so must all already be in
 * the process RTLD_GLOBAL. i6c_sys_load does exactly that, and hal_audio_init
 * calls it first for that reason rather than for MI_SYS_Init's sake alone.
 *
 * The debug-level one is the reason this list is worth keeping accurate: it is
 * not called on any path that runs during bring-up, so leaving it unresolved
 * costs nothing until the day the library decides to log, and then costs the
 * whole daemon (see i6c_sys_load.h).
 *
 * RTLD_LAZY as everywhere else in this backend: there is nothing to gain from
 * binding eagerly, and this generation's library has no optional algorithm packs
 * to worry about -- see i6c_aud.h, point 4.
 */
static inline int i6c_aud_load(i6c_aud_api *aud)
{
    if (!(aud->lib = dlopen("libmi_ai.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_aud: dlopen(libmi_ai.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(aud->open = (int (*)(unsigned int, const i6c_aud_cnf *))hal_symbol_load(
              "i6c_aud", aud->lib, "MI_AI_Open")))
        return RSS_ERR_NOTSUP;

    if (!(aud->close =
              (int (*)(unsigned int))hal_symbol_load("i6c_aud", aud->lib, "MI_AI_Close")))
        return RSS_ERR_NOTSUP;

    if (!(aud->get_attr = (int (*)(unsigned int, i6c_aud_cnf *))hal_symbol_load(
              "i6c_aud", aud->lib, "MI_AI_GetAttr")))
        return RSS_ERR_NOTSUP;

    if (!(aud->attach_if = (int (*)(unsigned int, const i6c_aud_if *, unsigned char))
              hal_symbol_load("i6c_aud", aud->lib, "MI_AI_AttachIf")))
        return RSS_ERR_NOTSUP;

    if (!(aud->enable_grp = (int (*)(unsigned int, unsigned char))hal_symbol_load(
              "i6c_aud", aud->lib, "MI_AI_EnableChnGroup")))
        return RSS_ERR_NOTSUP;

    if (!(aud->disable_grp = (int (*)(unsigned int, unsigned char))hal_symbol_load(
              "i6c_aud", aud->lib, "MI_AI_DisableChnGroup")))
        return RSS_ERR_NOTSUP;

    if (!(aud->read = (int (*)(unsigned int, unsigned char, i6c_aud_frm *, i6c_aud_frm *, int))
              hal_symbol_load("i6c_aud", aud->lib, "MI_AI_Read")))
        return RSS_ERR_NOTSUP;

    if (!(aud->release = (int (*)(unsigned int, unsigned char, i6c_aud_frm *, i6c_aud_frm *))
              hal_symbol_load("i6c_aud", aud->lib, "MI_AI_ReleaseData")))
        return RSS_ERR_NOTSUP;

    if (!(aud->set_gain = (int (*)(unsigned int, unsigned char, const signed char *, unsigned char))
              hal_symbol_load("i6c_aud", aud->lib, "MI_AI_SetGain")))
        return RSS_ERR_NOTSUP;

    if (!(aud->get_gain = (int (*)(unsigned int, unsigned char, signed char *, unsigned char *))
              hal_symbol_load("i6c_aud", aud->lib, "MI_AI_GetGain")))
        return RSS_ERR_NOTSUP;

    if (!(aud->set_mute =
              (int (*)(unsigned int, unsigned char, const unsigned char *, unsigned char))
                  hal_symbol_load("i6c_aud", aud->lib, "MI_AI_SetMute")))
        return RSS_ERR_NOTSUP;

    if (!(aud->set_if_gain = (int (*)(i6c_aud_if, signed char, signed char))hal_symbol_load(
              "i6c_aud", aud->lib, "MI_AI_SetIfGain")))
        return RSS_ERR_NOTSUP;

    if (!(aud->get_if_gain = (int (*)(i6c_aud_if, signed char *, signed char *))hal_symbol_load(
              "i6c_aud", aud->lib, "MI_AI_GetIfGain")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_aud_unload(i6c_aud_api *aud)
{
    if (aud->lib)
        dlclose(aud->lib);
    memset(aud, 0, sizeof(*aud));
}

#endif /* INFINITY6C_I6C_AUD_LOAD_H */
