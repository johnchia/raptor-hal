/*
 * star/hal_isp.c -- ISP tuning and 3A for the SigmaStar MI HAL
 *
 * ================================================================
 * THE POINT OF THIS FILE
 *
 * Phases 2c-2e produced a correct video pipeline with a visibly wrong
 * image: colours off, because enabling VPE auto-starts CUS3A and CUS3A
 * loads /etc/firmware/iqfile0.bin -- a generic tuning file that knows
 * nothing about the attached sensor. OpenIPC already ships a tuned
 * binary per sensor (/etc/sensors/<name>.bin), so the single most
 * valuable thing this file does is load the right one. Everything else
 * here is secondary, and some of it is actively better left undone --
 * see THE TUNING BINARY OUTRANKS THE CONFIG below.
 * ================================================================
 *
 * ================================================================
 * TWO SHAPES OF MI_ISP CALL, AND WHY THERE IS A TABLE
 *
 * libmi_isp.so exports ~360 functions. A handful are typed lifecycle
 * calls with real prototypes in i6_isp.h. The other ~340 are
 * MI_ISP_{IQ,AE,AWB,AF}_{Get,Set}<Module>, all of them
 * (int channel, void *payload), each with its own payload struct.
 *
 * Writing 340 structs to poke one field each would be absurd, and it is
 * not necessary: the payloads follow a fixed convention, discovered by
 * OpenIPC waybeam_venc (src/star6e_iq.c) and confirmed here against the
 * board's own library. An auto/manual module looks like
 *
 *     { uint32 bEnable; uint32 enOpType; <auto>[16]; <manual>; }
 *
 * where <auto> is 16 copies of a per-ISO parameter block, so the manual
 * block always begins at 8 + 16 * sizeof(per_iso_param). A manual-only
 * module drops enOpType and the auto array (manual at offset 4), and a
 * toggle-only module is just bEnable.
 *
 * So one field is reachable with (payload size, manual offset, width) and
 * no struct at all. The payload size is verifiable without vendor
 * headers, because each wrapper hardcodes it:
 *
 *     arm-openipc-linux-gnueabihf-objdump -d \
 *         --disassemble=MI_ISP_IQ_GetBrightness libmi_isp.so
 *
 * prints `mov.w r3, #76` into the size slot. That is where the numbers in
 * i6_isp.h come from, and it is the size that governs how much the
 * library copies into the staging buffer.
 *
 * The manual offset does not follow from the payload size alone, and
 * assuming it did is how one row came to be wrong for as long as the
 * table existed. The auto array holds 16 blocks and the manual block is a
 * seventeenth of the same size, so a payload of 1776 implies a block of
 * (1776 - 8) / 17 = 104 and a manual offset of 8 + 16 * 104 = 1672 -- not
 * the 1288 that was there, which implies an 80-byte block and contradicts
 * the 1776 it sits beside. Brightness is the case where the arithmetic is
 * unambiguous (a u32 at 72 in 76 is provably the last four bytes); the
 * larger modules are not, so tests/abi_iq.c now checks every row against
 * the vendor structs instead.
 *
 * Every access is read-modify-write. That is what makes poking one field
 * of a struct we have not fully described safe: whatever else the
 * payload holds -- and NR3D holds 1776 bytes of it -- survives untouched.
 * ================================================================
 *
 * ================================================================
 * THE TUNING BINARY OUTRANKS THE CONFIG
 *
 * rvd applies its whole [image] block unconditionally at startup, using
 * built-in defaults for every key the config omits (rvd_pipeline.c,
 * section 3c). A HAL that dutifully wrote all of them would flip nine
 * ISP modules from auto to manual on every boot -- overwriting, with
 * hardcoded midpoints, the tuned curves it had just loaded from the
 * sensor binary. The image would be worse than before this file existed.
 *
 * So a scalar knob at its neutral value (128) does not mean "write 128".
 * It means "nobody asked for anything", and this file answers it by
 * putting the module back into *auto*, which is where the tuning binary
 * wants it. Only a value that differs from neutral selects manual mode.
 * The mapping is reversible: setting a knob and then returning it to 128
 * restores auto rather than pinning the midpoint.
 *
 * A consequence worth stating plainly: a user who genuinely wants manual
 * brightness at exactly the midpoint cannot express it, and gets auto.
 * That trade is deliberate -- the alternative penalises every default
 * configuration to serve a request nobody has made.
 * ================================================================
 *
 * ================================================================
 * OP COVERAGE -- what is missing, and why
 *
 * Absent ops return RSS_ERR_NOTSUP through RSS_HAL_CALL's NULL guard,
 * and rvd treats all of these as advisory. Left unimplemented on
 * purpose:
 *
 *   isp_set_dpc_strength  MI's DynamicDP manual field is one bit. A
 *                         0..255 strength knob does not map onto it, and
 *                         pretending otherwise reads as support for a
 *                         control that has two positions.
 *   isp_set_defog_strength / _adv
 *                         Same: MI's Defog is a toggle. The plain
 *                         isp_set_defog *is* implemented.
 *   isp_set_drc_strength, isp_set_highlight_depress,
 *   isp_set_backlight_comp
 *                         These live in WDR/HDR modules whose manual
 *                         blocks are multi-field curve descriptors, not
 *                         single strengths. Mapping one scalar onto a
 *                         curve is a tuning decision, not a HAL one.
 *   isp_set_hue           MI's hue control is a 64-entry HSV LUT
 *                         (manual@3096); a scalar rotation would mean
 *                         synthesising the whole table.
 *   isp_set_wb / isp_get_wb
 *                         AWB_SetAttr's 1464-byte payload is not
 *                         described by the offset convention above (it
 *                         is not an auto/manual pair), so it needs its
 *                         own derivation. Deferred rather than guessed.
 *   isp_set_bypass        No MI equivalent; the ISP cannot be bypassed
 *                         while VPE is the only path to the encoder.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * nanosleep needs this, and it has to precede every include.
 * raptor-hal builds -std=c11 rather than gnu11, so glibc defines
 * __STRICT_ANSI__ and hides everything outside ISO C -- including the
 * whole of POSIX. This is the first file in the HAL that has to wait for
 * hardware, so it is the first to need the macro. nanosleep in
 * preference to usleep: usleep was removed from POSIX in 2008.
 */
#define _POSIX_C_SOURCE 200809L

#include "star_state.h"

#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/*
 * No HAL_MODULE_VIDEO guard here, deliberately. That define exists to
 * build hal_common.c twice, once per archive; this file is video-only by
 * construction -- it is in the Makefile's VIDEO_SRCS, so only
 * libraptor_hal_video.a ever links it. Wrapping it in the guard compiles
 * the whole translation unit away, which links as an empty object and
 * fails only later, as undefined vtable entries. hal_encoder.c and
 * hal_framesource.c are in the same position and carry no guard either.
 */

/*
 * raptor's scalar ISP knobs are 0..255 with 128 as neutral. Named
 * because the neutral value carries the auto/manual meaning described
 * above, so it is a protocol constant rather than a magic midpoint.
 */
#define STAR_ISP_NEUTRAL 128

/*
 * enOpType values. The vendor enum is E_MI_ISP_OP_TYPE_{AUTO,MANUAL},
 * auto first -- the same ordering every MI-family SDK uses for this
 * field. Not independently verified on hardware: a swap would show up
 * as a scalar knob having no effect, never as a crash, since the field
 * is a bounded enum inside a struct we read back before writing.
 */
#define STAR_ISP_OP_AUTO 0u
#define STAR_ISP_OP_MANUAL 1u

/* Offset of enOpType within an auto/manual payload -- always after the
 * leading bEnable, per the layout convention. */
#define STAR_ISP_OPTYPE_OFF 4u

/*
 * How long to wait for the IQ parameter store, and how often to look.
 *
 * The ISP channel initialises asynchronously after MI_VPE_CreateChannel
 * returns, so everything here has to wait for it once. 2000 ms is
 * waybeam's bound (star6e_pipeline.c:171); the observed wait on this
 * board is far shorter, but a cold boot with a slow sensor is the case
 * the bound exists for. divinus instead sleeps a flat second before its
 * load (media.c:827), which is the same wait without the evidence.
 */
#define STAR_ISP_READY_TIMEOUT_MS 2000
#define STAR_ISP_READY_POLL_MS 10

/*
 * Budget for the early opportunistic attempt, made the moment a VPE port
 * is enabled. Short on purpose: frames need about one frame period to
 * start, so a ready ISP answers well inside this, and an unready one must
 * not spend the full timeout here only for the attempt at encoder start
 * to spend it again.
 */
#define STAR_ISP_READY_QUICK_MS 400

/* Largest payload the table below touches (NR3D, 1776). Sized generously
 * so a future entry does not silently overflow -- star_iq_call refuses
 * anything that does not fit rather than truncating. */
#define STAR_IQ_PAYLOAD_MAX 2048

static void star_isp_sleep_ms(unsigned int ms)
{
    struct timespec ts;

    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    (void)nanosleep(&ts, NULL);
}

typedef enum {
    IQ_FLAT,   /* value at offset 0, no bEnable and no enOpType */
    IQ_BOOL,   /* bEnable at offset 0 is itself the value */
    IQ_AUTOMAN /* bEnable, enOpType, auto[16], manual at manual_off */
} star_iq_shape_t;

typedef struct {
    const char *name; /* for diagnostics only */
    const char *get_sym;
    const char *set_sym;
    uint16_t payload;    /* wrapper's hardcoded payload size */
    uint16_t manual_off; /* where the value lives (0 for FLAT/BOOL) */
    uint8_t width;       /* 1, 2 or 4 bytes */
    uint8_t shape;       /* star_iq_shape_t */
    uint32_t mi_max;   /* MI's maximum for the field */
    uint32_t mi_unity; /* MI value that means the same as raptor's 128 */
    /*
     * Set for a parameter whose MI neutral is not a constant this port can
     * know: the baseline is whatever the tuning binary left in the field,
     * and mi_unity above is only the fallback for a board running with no
     * tuning file at all.
     */
    bool unity_from_tuning;

    /* Resolved on first use and cached. dlsym per call would work, but
     * these sit on rvd's control path and the symbol never changes. */
    i6_isp_cmd_fn fn_get;
    i6_isp_cmd_fn fn_set;

    /*
     * Value requested before the ISP would accept it, flushed by
     * star_isp_tune_when_ready. rvd applies its whole [image] block
     * during pipeline *construction*, well before any VPE port is
     * enabled, so without this queue every one of those calls fails and
     * the operator's settings are silently lost. Flushing after the
     * tuning binary loads is also the only correct order -- applied
     * before, the load would overwrite them.
     *
     * Recorded whether or not it could be applied straight away, and
     * *not* cleared by the flush: a tuning reload resets each module to
     * whatever the binary says, so the last value asked for is also the
     * value a re-tune has to put back. Without that, the first hot
     * restart silently reverts every knob the operator had set.
     *
     * Lives in the table beside the cached symbols, on the same
     * single-instance assumption, and is cleared by star_isp_teardown.
     */
    int pending;
    bool has_pending;
    bool pending_is_raw; /* set via star_iq_set_raw, not _set_scalar */

    /* Armed by every tuning load, cleared by the fetch that reads the
     * baseline back out. Only meaningful with unity_from_tuning. */
    bool unity_stale;
} star_iq_param_t;

enum {
    IQ_BRIGHTNESS,
    IQ_CONTRAST,
    IQ_SATURATION,
    IQ_SHARPNESS,
    IQ_SINTER,
    IQ_TEMPER,
    IQ_DEFOG,
    IQ_GRAY,
    IQ_EVCOMP,
    IQ_FLICKER,
    IQ_PARAM_COUNT
};

/*
 * Payload sizes and manual-block offsets live in i6_isp.h, where the
 * vendor ABI belongs and where tests/abi_iq.c asserts every one of them
 * against the vendor headers. mi_unity is the MI value corresponding to
 * raptor's neutral 128, which is what keeps a default config from
 * shifting the image:
 *
 *   brightness/contrast  0..100, midpoint 50
 *   saturation           0..127 where 32 is unity gain (1X), *not* the
 *                        midpoint -- waybeam names this explicitly, and
 *                        a linear 0..255 -> 0..127 map would silently
 *                        double saturation at raptor's neutral
 *   sharpness, NR        0..255, midpoint 128
 *   EV compensation      0..200, unity unknown and learned from the tuning
 *                        -- see unity_from_tuning
 */
static star_iq_param_t g_iq[IQ_PARAM_COUNT] = {
    [IQ_BRIGHTNESS] = { "brightness", "MI_ISP_IQ_GetBrightness", "MI_ISP_IQ_SetBrightness",
                        I6_ISP_IQ_BRIGHTNESS_PAYLOAD, I6_ISP_IQ_BRIGHTNESS_MANUAL,
                        4, IQ_AUTOMAN, 100, 50, false, NULL, NULL },
    [IQ_CONTRAST] = { "contrast", "MI_ISP_IQ_GetContrast", "MI_ISP_IQ_SetContrast",
                      I6_ISP_IQ_CONTRAST_PAYLOAD, I6_ISP_IQ_CONTRAST_MANUAL, 4,
                      IQ_AUTOMAN, 100, 50, false, NULL, NULL },
    [IQ_SATURATION] = { "saturation", "MI_ISP_IQ_GetSaturation", "MI_ISP_IQ_SetSaturation",
                        I6_ISP_IQ_SATURATION_PAYLOAD, I6_ISP_IQ_SATURATION_MANUAL,
                        1, IQ_AUTOMAN, 127, 32, false, NULL, NULL },
    [IQ_SHARPNESS] = { "sharpness", "MI_ISP_IQ_GetSharpness", "MI_ISP_IQ_SetSharpness",
                       I6_ISP_IQ_SHARPNESS_PAYLOAD, I6_ISP_IQ_SHARPNESS_MANUAL,
                       1, IQ_AUTOMAN, 255, 128, false, NULL, NULL },
    /* Spatial (per-frame) luma noise reduction is raptor's "sinter". */
    [IQ_SINTER] = { "sinter", "MI_ISP_IQ_GetNRLuma", "MI_ISP_IQ_SetNRLuma",
                    I6_ISP_IQ_NRLUMA_PAYLOAD, I6_ISP_IQ_NRLUMA_MANUAL, 1, IQ_AUTOMAN,
                    255, 128, false, NULL, NULL },
    /* Temporal noise reduction is raptor's "temper" -- MI calls it 3D NR. */
    [IQ_TEMPER] = { "temper", "MI_ISP_IQ_GetNR3D", "MI_ISP_IQ_SetNR3D",
                    I6_ISP_IQ_NR3D_PAYLOAD, I6_ISP_IQ_NR3D_MANUAL, 1, IQ_AUTOMAN,
                    255, 128, false, NULL, NULL },
    [IQ_DEFOG] = { "defog", "MI_ISP_IQ_GetDefog", "MI_ISP_IQ_SetDefog",
                   I6_ISP_IQ_DEFOG_PAYLOAD, 0, 4, IQ_BOOL, 1, 0, false, NULL, NULL },
    [IQ_GRAY] = { "gray", "MI_ISP_IQ_GetColorToGray", "MI_ISP_IQ_SetColorToGray",
                  I6_ISP_IQ_GRAY_PAYLOAD, 0, 4, IQ_BOOL, 1, 0, false, NULL, NULL },
    /*
     * The only knob whose neutral has to be learned. It is IQ_FLAT, so
     * there is no auto mode to hand it back to, and MI's own no-compensation
     * value is undocumented -- 100 is the midpoint of the range, which is
     * not the same thing. Anything written here pins the AE's target luma,
     * so guessing the neutral wrong shifts every default image.
     */
    [IQ_EVCOMP] = { "ae_comp", "MI_ISP_AE_GetEVComp", "MI_ISP_AE_SetEVComp",
                    I6_ISP_AE_EVCOMP_PAYLOAD, 0, 4, IQ_FLAT, 200, 100, true, NULL, NULL },
    [IQ_FLICKER] = { "antiflicker", "MI_ISP_AE_GetFlicker", "MI_ISP_AE_SetFlicker",
                     I6_ISP_AE_FLICKER_PAYLOAD, 0, 4, IQ_FLAT, 3, 0, false, NULL, NULL },
};

/* ================================================================
 * GENERIC IQ FIELD ACCESS
 * ================================================================ */

static uint32_t star_iq_read(const uint8_t *buf, uint16_t off, uint8_t width)
{
    uint32_t v = 0;

    /* memcpy rather than a cast: the payload is a byte buffer and these
     * offsets are not guaranteed to be aligned for the wider types. */
    switch (width) {
    case 1:
        v = buf[off];
        break;
    case 2: {
        uint16_t t;
        memcpy(&t, buf + off, sizeof(t));
        v = t;
        break;
    }
    default: {
        uint32_t t;
        memcpy(&t, buf + off, sizeof(t));
        v = t;
        break;
    }
    }

    return v;
}

static void star_iq_write(uint8_t *buf, uint16_t off, uint8_t width, uint32_t val)
{
    switch (width) {
    case 1:
        buf[off] = (uint8_t)val;
        break;
    case 2: {
        uint16_t t = (uint16_t)val;
        memcpy(buf + off, &t, sizeof(t));
        break;
    }
    default:
        memcpy(buf + off, &val, sizeof(val));
        break;
    }
}

/* Resolve and cache a parameter's getter and setter. */
static int star_iq_resolve(star_state_t *st, star_iq_param_t *p)
{
    if (p->fn_get && p->fn_set)
        return RSS_OK;

    if (!st->isp_loaded || !st->isp.handle)
        return RSS_ERR_NOENT;

    if (p->payload > STAR_IQ_PAYLOAD_MAX) {
        HAL_LOG_ERR("isp: %s payload %u exceeds the %u-byte buffer", p->name, p->payload,
                    STAR_IQ_PAYLOAD_MAX);
        return RSS_ERR_INVAL;
    }

    p->fn_get = (i6_isp_cmd_fn)hal_symbol_load("i6_isp", st->isp.handle, p->get_sym);
    p->fn_set = (i6_isp_cmd_fn)hal_symbol_load("i6_isp", st->isp.handle, p->set_sym);
    if (!p->fn_get || !p->fn_set) {
        p->fn_get = NULL;
        p->fn_set = NULL;
        return RSS_ERR_NOTSUP;
    }

    return RSS_OK;
}

/*
 * Adopt the tuning binary's own value as the neutral raptor's 128 maps to.
 *
 * Called on every fetch and gated on unity_stale, which each tuning load
 * re-arms, so what it reads is the binary's value and not one of ours: the
 * knob queue is flushed after the load, and this runs on the fetch that
 * flush performs.
 *
 * Without it the sub-neutral half of raptor's scale is measured from a
 * guess. On this board the guess was 100 of 0..200 and the tuning sits far
 * below that, so a default config brightened the picture and every value
 * under 128 was spent climbing back down to where the tuning already was.
 */
static void star_iq_learn_unity(star_iq_param_t *p, const uint8_t *buf)
{
    uint32_t base;

    if (!p->unity_from_tuning || !p->unity_stale)
        return;

    base = star_iq_read(buf, p->manual_off, p->width);
    if (base > p->mi_max) {
        /* Out of range means the offset or width is wrong, and a baseline
         * adopted from a misread field would be invisible afterwards. */
        HAL_LOG_WARN("isp: %s reads MI %u, above its %u maximum -- not adopting it as the "
                     "neutral, keeping %u",
                     p->name, base, p->mi_max, p->mi_unity);
        p->unity_stale = false;
        return;
    }

    p->mi_unity = base;
    p->unity_stale = false;
    HAL_LOG_INFO("isp: %s baseline from the tuning is MI %u/%u -- raptor 128 maps here, "
                 "so 0..127 darkens by up to %u and 129..255 brightens by up to %u",
                 p->name, base, p->mi_max, base, p->mi_max - base);
}

/* Re-arm every baseline that is read out of the tuning rather than assumed.
 * Defined below the AE target curve, since it arms that too. */
static void star_isp_arm_tuning_reads(void);

/*
 * Read the module's current payload. Callers modify one field of it and
 * hand it back to star_iq_store, so the ~1700 bytes of tuning we cannot
 * describe are preserved rather than zeroed.
 */
static int star_iq_fetch(star_state_t *st, int idx, uint8_t *buf)
{
    star_iq_param_t *p = &g_iq[idx];
    int ret;

    ret = star_iq_resolve(st, p);
    if (ret != RSS_OK)
        return ret;

    memset(buf, 0, p->payload);
    ret = p->fn_get(STAR_ISP_CHN, buf);
    if (ret) {
        HAL_LOG_WARN("isp: %s (%s) failed: %d", p->name, p->get_sym, ret);
        return RSS_ERR_IO;
    }

    star_iq_learn_unity(p, buf);
    return RSS_OK;
}

static int star_iq_store(star_state_t *st, int idx, uint8_t *buf)
{
    star_iq_param_t *p = &g_iq[idx];
    int ret;

    (void)st;
    ret = p->fn_set(STAR_ISP_CHN, buf);
    if (ret) {
        HAL_LOG_WARN("isp: %s (%s) failed: %d", p->name, p->set_sym, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * Map raptor's 0..255 onto MI's range, piecewise so that neutral lands
 * exactly on MI's own unity value. A single linear map would not: with
 * saturation's unity at 32 of 127, linear scaling puts raptor's neutral
 * at 64 -- twice unity gain -- and every default config would boost
 * colour.
 */
static uint32_t star_iq_scale(int val, uint32_t unity, uint32_t max)
{
    if (val <= 0)
        return 0;
    if (val >= 255)
        return max;
    /* A unity of 0 is legitimate for a learned baseline and needs no guard:
     * the divisors below are constants, and the sub-neutral half correctly
     * collapses onto 0 because there is nowhere below it to go. */
    if (val == STAR_ISP_NEUTRAL || unity >= max)
        return unity;

    if (val < STAR_ISP_NEUTRAL)
        return (uint32_t)(((uint64_t)val * unity) / STAR_ISP_NEUTRAL);

    return unity + (uint32_t)(((uint64_t)(val - STAR_ISP_NEUTRAL) * (max - unity)) /
                              (255 - STAR_ISP_NEUTRAL));
}

/* Inverse of star_iq_scale, for the getters. */
static uint8_t star_iq_unscale(uint32_t mi, uint32_t unity, uint32_t max)
{
    if (max == 0 || mi >= max)
        return 255;
    if (unity >= max)
        return STAR_ISP_NEUTRAL;
    /* Tested before the mi == 0 case, which it subsumes: with a baseline of
     * 0, MI 0 is neutral rather than the bottom of the scale. */
    if (mi == unity)
        return STAR_ISP_NEUTRAL;

    /* unity is necessarily non-zero on this branch, so the divide is safe. */
    if (mi < unity)
        return (uint8_t)(((uint64_t)mi * STAR_ISP_NEUTRAL) / unity);

    return (uint8_t)(STAR_ISP_NEUTRAL +
                     ((uint64_t)(mi - unity) * (255 - STAR_ISP_NEUTRAL)) / (max - unity));
}

/*
 * Apply one of raptor's 0..255 scalars.
 *
 * Neutral restores auto and leaves the manual field alone -- see THE
 * TUNING BINARY OUTRANKS THE CONFIG. bEnable is never touched: if the
 * tuning binary disabled a module, re-enabling it behind the tuner's
 * back is not this layer's call.
 */
static int star_iq_apply_scalar(star_state_t *st, int idx, int val)
{
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    uint32_t mi_val;
    int ret;

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    if (p->shape == IQ_AUTOMAN) {
        if (val == STAR_ISP_NEUTRAL) {
            star_iq_write(buf, STAR_ISP_OPTYPE_OFF, 4, STAR_ISP_OP_AUTO);
            HAL_LOG_DBG("isp: %s left to the tuning file (auto)", p->name);
            return star_iq_store(st, idx, buf);
        }
        star_iq_write(buf, STAR_ISP_OPTYPE_OFF, 4, STAR_ISP_OP_MANUAL);
    }

    mi_val = star_iq_scale(val, p->mi_unity, p->mi_max);
    star_iq_write(buf, p->manual_off, p->width, mi_val);

    ret = star_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %d (MI %u/%u)", p->name, val, mi_val, p->mi_max);

    return ret;
}

/*
 * Queue-or-apply. Splitting this from star_iq_apply_scalar lets the
 * flush drain the queue without re-entering it, and keeps the "is the
 * ISP reachable yet" question in exactly one place per direction.
 */
static int star_iq_set_scalar(void *ctx, int idx, int val)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];

    if (!st)
        return RSS_ERR_INVAL;

    /* Recorded first and unconditionally, so a re-tune can put it back
     * whether or not it reached MI on this attempt. */
    p->pending = val;
    p->has_pending = true;
    p->pending_is_raw = false;

    if (!st->isp_tuned) {
        HAL_LOG_DBG("isp: %s = %d queued until the ISP is up", p->name, val);
        return RSS_OK;
    }

    return star_iq_apply_scalar(st, idx, val);
}

static int star_iq_get_scalar(void *ctx, int idx, uint8_t *out)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !out)
        return RSS_ERR_INVAL;

    /* Report what was asked for while the ISP cannot be read, so a
     * set/get pair is consistent even before the pipeline runs. */
    if (!st->isp_tuned) {
        *out = p->has_pending ? (uint8_t)p->pending : (uint8_t)STAR_ISP_NEUTRAL;
        return RSS_OK;
    }

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    /*
     * A module in auto mode has no single value to report, and its
     * manual field holds whatever was last written there. Neutral is
     * the honest answer, and it round-trips: it is also the value that
     * puts the module back into auto.
     */
    if (p->shape == IQ_AUTOMAN &&
        star_iq_read(buf, STAR_ISP_OPTYPE_OFF, 4) == STAR_ISP_OP_AUTO) {
        *out = STAR_ISP_NEUTRAL;
        return RSS_OK;
    }

    *out = star_iq_unscale(star_iq_read(buf, p->manual_off, p->width), p->mi_unity, p->mi_max);
    return RSS_OK;
}

/* Raw field access for the enum- and bool-valued parameters, which have
 * no 0..255 scale to map through. */
static int star_iq_apply_raw(star_state_t *st, int idx, uint32_t raw)
{
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    star_iq_write(buf, p->manual_off, p->width, raw);
    ret = star_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %u", p->name, raw);

    return ret;
}

static int star_iq_set_raw(void *ctx, int idx, uint32_t raw)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];

    if (!st)
        return RSS_ERR_INVAL;

    p->pending = (int)raw;
    p->has_pending = true;
    p->pending_is_raw = true;

    if (!st->isp_tuned) {
        HAL_LOG_DBG("isp: %s = %u queued until the ISP is up", p->name, raw);
        return RSS_OK;
    }

    return star_iq_apply_raw(st, idx, raw);
}

static int star_iq_get_raw(void *ctx, int idx, uint32_t *raw)
{
    star_state_t *st = star_state(ctx);
    star_iq_param_t *p = &g_iq[idx];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !raw)
        return RSS_ERR_INVAL;

    if (!st->isp_tuned) {
        *raw = p->has_pending ? (uint32_t)p->pending : 0u;
        return RSS_OK;
    }

    ret = star_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    *raw = star_iq_read(buf, p->manual_off, p->width);
    return RSS_OK;
}

/* ================================================================
 * BRING-UP: TUNING BINARY AND 3A
 * ================================================================ */

/*
 * Wait for the IQ parameter store to come up.
 *
 * Distinguishes "the call failed" from "the flag is not set yet", which
 * matters more than it looks: the first board run of this file reported
 * only a flat "not ready after 2000 ms" while every underlying call was
 * in fact returning 6, and collapsing those two cases into one message
 * is what made a plain ordering bug look like a timing problem.
 */
static int star_isp_wait_ready(star_state_t *st, unsigned int timeout_ms, bool verbose)
{
    unsigned int waited = 0;
    int last_ret = 0;

    if (!st->isp.fnGetParaInitStatus)
        return RSS_ERR_NOTSUP;

    while (waited < timeout_ms) {
        i6_isp_parainit status;
        int ret;

        memset(&status, 0, sizeof(status));
        ret = st->isp.fnGetParaInitStatus(STAR_ISP_CHN, &status);
        if (ret == 0 && status.ready) {
            HAL_LOG_DBG("isp: parameter store ready after %u ms", waited);
            return RSS_OK;
        }
        last_ret = ret;

        star_isp_sleep_ms(STAR_ISP_READY_POLL_MS);
        waited += STAR_ISP_READY_POLL_MS;
    }

    if (!verbose) {
        /* An early opportunistic attempt; a later one will retry. */
        HAL_LOG_DBG("isp: parameter store not up yet after %u ms (last return %d)", timeout_ms,
                    last_ret);
    } else if (last_ret) {
        HAL_LOG_WARN("isp: MI_ISP_IQ_GetParaInitStatus keeps returning %d after %u ms -- the ISP "
                     "is not answering on VPE channel %d, so tuning is being skipped",
                     last_ret, timeout_ms, STAR_ISP_CHN);
    } else {
        HAL_LOG_WARN("isp: parameter store still not ready after %u ms; skipping tuning",
                     timeout_ms);
    }

    return RSS_ERR_TIMEOUT;
}

/*
 * Directories that ship one tuning binary per sensor, searched in order.
 *
 * The file is named after the sensor's driver module
 * (sensor_gc4653_mipi.ko -> gc4653.bin), which is the string the backend
 * reads from /proc/modules during bring-up, so on a stock image the right
 * file is found with nothing declared anywhere. Distributions disagree on the
 * directory -- OpenIPC installs into /etc/sensors, thingino keeps them in
 * /usr/share/sensor -- and the same binary is expected to boot on either
 * rootfs, so both are searched rather than one being fixed at build time.
 *
 * Element type is pointer-to-const rather than a const array: the tests
 * retarget these at temporary directories to exercise the search order.
 */
static const char *star_iq_dirs[] = {
    "/etc/sensors",
    "/usr/share/sensor",
};

/*
 * Work out which tuning binary to load: the sensor's own name picks the file
 * out of the directories above.
 *
 * Returns true and fills out[] when a readable file was found.
 */
static bool star_isp_resolve_iq(star_state_t *st, const rss_sensor_config_t *cfg, char *out,
                                size_t len)
{
    const char *name = NULL;
    char lower[64];
    char tried[192] = "";
    size_t i, d, off = 0;

    out[0] = '\0';

    /*
     * The module name resolves the file, so the backend's own reading of it
     * comes first among the two that carry that spelling; an operator's
     * [sensor] name overrides it, since naming it explicitly is a deliberate
     * act. plane.sensName is the same identity spelled MI's way and is the
     * last resort, for a board whose module is named unusually.
     */
    if (cfg && cfg->name[0])
        name = cfg->name;
    else if (st->sensor_name[0])
        name = st->sensor_name;
    else if (st->snr_enabled && st->plane.sensName[0])
        name = st->plane.sensName;

    if (!name) {
        HAL_LOG_WARN("isp: no sensor name; the generic vendor tuning stays loaded");
        return false;
    }

    for (i = 0; i + 1 < sizeof(lower) && name[i]; i++)
        lower[i] = (char)tolower((unsigned char)name[i]);
    lower[i] = '\0';

    for (d = 0; d < sizeof(star_iq_dirs) / sizeof(star_iq_dirs[0]); d++) {
        snprintf(out, len, "%s/%s.bin", star_iq_dirs[d], lower);
        if (access(out, R_OK) == 0) {
            HAL_LOG_DBG("isp: tuning file %s (from sensor name)", out);
            return true;
        }
        if (off < sizeof(tried)) {
            int n = snprintf(tried + off, sizeof(tried) - off, "%s%s", off ? " " : "", out);
            if (n > 0)
                off += (size_t)n;
        }
    }

    HAL_LOG_WARN("isp: no tuning file for %s (tried %s); the generic vendor tuning stays "
                 "loaded and colour will be off",
                 lower, tried);
    out[0] = '\0';
    return false;
}

/*
 * Read the AE's exposure limits, waiting for it to publish them.
 *
 * A cold-booted AE has not processed enough frames to publish its limits
 * and answers all zeros; capping or clamping against that would treat a
 * garbage floor as calibration. waybeam polls for up to 500 ms here
 * (pipeline_common.c:138-152). The bin load ahead of this already waited
 * on the parameter store, so one retry window is enough.
 */
static int star_isp_read_limits(star_state_t *st, i6_isp_exp *limit)
{
    unsigned int waited;
    int ret;

    memset(limit, 0, sizeof(*limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }
    if (limit->maxShutterUs || limit->maxSensorGain)
        return RSS_OK;

    for (waited = 0; waited < 500; waited += 10) {
        star_isp_sleep_ms(10);
        memset(limit, 0, sizeof(*limit));
        if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, limit) == 0 &&
            (limit->maxShutterUs || limit->maxSensorGain))
            return RSS_OK;
    }

    return RSS_ERR_TIMEOUT;
}

/*
 * Record the tuning's own gain ceilings, once, before the config knobs
 * land on top of them.
 *
 * The ordering is the whole point and it is easy to get wrong:
 * star_isp_flush_pending runs before star_isp_cap_exposure, so by the time
 * anything else reads this struct a requested ceiling is already in it --
 * and because every writer here is a read-modify-write, one bad ceiling
 * propagates into every later write. Snapshotting after the flush would
 * record the overwrite and call it calibration.
 */
static void star_isp_snapshot_bin_limits(star_state_t *st)
{
    i6_isp_exp limit;

    if (!st || !st->isp_loaded)
        return;
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return;

    if (star_isp_read_limits(st, &limit) != RSS_OK) {
        HAL_LOG_WARN("isp: AE published no exposure limits; gain ceilings go on unchecked");
        return;
    }

    st->bin_min_sensor_gain = limit.minSensorGain;
    st->bin_max_sensor_gain = limit.maxSensorGain;
    st->bin_min_isp_gain = limit.minIspGain;
    st->bin_max_isp_gain = limit.maxIspGain;
    st->bin_max_shutter_us = limit.maxShutterUs;

    /*
     * INFO because this is the line every night-mode threshold gets
     * calibrated against. total_gain cannot exceed maxSensorGain *
     * maxIspGain / 1024, so this is what says whether a given night_gain
     * is reachable on this board at all -- and a night_gain that is not
     * reachable means auto night mode simply never triggers.
     */
    HAL_LOG_DBG("isp: AE tuning limits (x1024): sensor gain %u..%u, isp gain %u..%u, "
                "shutter %u..%u us -- so total_gain tops out at %llu",
                limit.minSensorGain, limit.maxSensorGain, limit.minIspGain, limit.maxIspGain,
                limit.minShutterUs, limit.maxShutterUs,
                (unsigned long long)limit.maxSensorGain *
                         (limit.maxIspGain ? limit.maxIspGain : 1024u) / 1024u);
}

/*
 * The rate to hand MI_SNR_SetFps: whole frames, unless a fractional rate
 * was programmed, in which case the milli-frames it was programmed with.
 * Re-issuing a fractional rate as a rounded one would quietly retune the
 * sensor a fraction of a frame away from what was asked for.
 */
static unsigned int star_snr_fps_arg(const star_state_t *st, unsigned int fps)
{
    if (st->fps_milli && st->fps_milli % 1000u)
        return st->fps_milli;
    return fps;
}

int star_isp_cap_exposure(star_state_t *st, unsigned int fps)
{
    i6_isp_exp limit;
    unsigned int frame_us, want;
    int ret;

    if (!st || !st->isp_loaded || !fps)
        return RSS_ERR_INVAL;
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return RSS_ERR_NOTSUP;

    /*
     * Read before write, and the read is not optional: the
     * read-modify-write below would otherwise hand the AE an uninitialised
     * struct, writing stack contents into its limits. Nothing in the ARM
     * -Werror build catches that, so the host suite covers it instead.
     */
    ret = star_isp_read_limits(st, &limit);
    if (ret == RSS_ERR_IO)
        return ret;
    if (ret != RSS_OK || limit.maxShutterUs == 0) {
        HAL_LOG_WARN("isp: AE published no exposure limits; shutter left uncapped");
        return RSS_ERR_TIMEOUT;
    }

    frame_us = 1000000u / fps;

    /*
     * The frame period is the ceiling's upper bound, and the tuning's own
     * ceiling is its target. Fitting to both means a rate change moves the
     * ceiling either way: up to what the tuning asked for when the frame
     * period allows it, down to the frame period when it does not. Neither
     * direction ever exceeds the calibration.
     */
    want = st->bin_max_shutter_us ? st->bin_max_shutter_us : limit.maxShutterUs;
    if (want > frame_us)
        want = frame_us;

    if (limit.maxShutterUs == want) {
        HAL_LOG_DBG("isp: AE max shutter already %u us for the %u us frame period", want, frame_us);
        return RSS_OK;
    }

    HAL_LOG_INFO("isp: AE max shutter %u -> %u us for %u fps", limit.maxShutterUs, want, fps);
    limit.maxShutterUs = want;
    if (limit.minShutterUs > want)
        limit.minShutterUs = want;

    ret = st->isp.fnSetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_SetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * Whether the cap above reaches the AE algorithm is unsettled, and the
     * one measurement in hand says it does not: with the limit reading
     * 22..33333 the AE was seen running a 50000 us shutter, which is the
     * tuning binary's own ceiling and exactly the 20 fps the board
     * delivered when asked for 30. That was on a run where the binary had
     * been loaded too early and re-read from disk underneath us, so it is
     * consistent with the AE taking its limits at init and never looking
     * again -- which would make this write bookkeeping for get-isp and for
     * the reload check.
     *
     * Not concluded, because the obvious follow-up cannot be done from
     * outside the process: MI's per-module AE calls dispatch through a
     * handler table that CUS3A registers in the *owning* process, so a
     * second process writing them proves nothing about what the algorithm
     * sees. Settling it means instrumenting this file, in a dark enough
     * scene for the AE to want the whole frame period.
     *
     * The SetFps below is independent of all that, and it is measured
     * working. It does not touch the AE: it makes the sensor driver
     * recompute its timing, which is what recovers a sensor the tuning
     * brought up slow. waybeam calls this the cold-boot fix
     * (star6e_pipeline.c:2094).
     */
    if (st->snr.fnSetFramerate && st->snr.fnSetFramerate(STAR_SNR_INDEX, star_snr_fps_arg(st, fps)))
        HAL_LOG_WARN("isp: MI_SNR_SetFps(%u) after the exposure fit failed", fps);

    return RSS_OK;
}

/*
 * ================================================================
 * WHY THE TUNING LOAD IS NOT DONE HERE
 *
 * The first board run of this file had every single MI_ISP call fail
 * with 6 and the parameter store never come ready. The cause is
 * ordering, and it is structural rather than a matter of waiting longer.
 *
 * MI has no independent ISP device. Disassembling any
 * MI_ISP_{IQ,AE}_Get<Module> shows it delegate to
 * _MI_ISP_GetIspApiData, which dispatches through a per-channel handler
 * table and, where no handler is registered, falls back to
 * MI_VPE_GetIspApiData -- the ISP is answered *by the VPE channel*. And
 * a VPE channel with no enabled output port has nowhere to send frames,
 * so it does not run, so its ISP front end never initialises and every
 * query is refused.
 *
 * At hal_init time that is exactly the state: star_vpe_bringup has
 * created, started and bound the channel, but the output ports belong to
 * the framesource ops and no caller has enabled one yet. Waiting longer
 * cannot help -- nothing was going to happen.
 *
 * Both references get this right by construction rather than by
 * explanation: divinus loads its IQ file at the very end of sdk_init,
 * after the encoding thread is already running (media.c:827), and
 * waybeam loads after its output and video stages are up. So the load
 * moves to star_isp_tune_when_ready, which the framesource and encoder
 * start paths call once frames can actually flow.
 *
 * What stays here is only what is safe before the pipeline runs: binding
 * the library and working out which file to load. Neither touches MI.
 * ================================================================
 */
void star_isp_bringup(star_state_t *st, const rss_sensor_config_t *cfg)
{
    char path[sizeof(st->iq_file)];
    int ret;

    if (!st)
        return;

    /*
     * Everything below is best-effort. An untuned image streams, and a
     * pipeline aborted over a tuning file does not: a wrong-looking image
     * is a defect, not an outage.
     */
    st->pend_max_again = -1;
    st->pend_max_dgain = -1;

    ret = i6_isp_load(&st->isp);
    if (ret != RSS_OK) {
        HAL_LOG_WARN("isp: MI_ISP unavailable (%d); no tuning or 3A control this run", ret);
        return;
    }
    st->isp_loaded = true;

    /* Resolution is pure path arithmetic plus access(), so it is legal
     * now, and cfg is only in scope during init. */
    if (star_isp_resolve_iq(st, cfg, path, sizeof(path)))
        snprintf(st->iq_file, sizeof(st->iq_file), "%s", path);
}

/*
 * Load the tuning binary, once the ISP is actually answering.
 *
 * Called from the framesource enable and encoder start paths rather than
 * from hal_init -- see the comment above star_isp_bringup for why that
 * is not a detail. Idempotent, and deliberately does *not* mark itself
 * done when the ISP is not ready yet, so a later call retries; the
 * quiet/verbose split keeps the first attempt from logging a warning
 * that the second one is about to make untrue.
 */
static void star_isp_flush_pending(star_state_t *st);
static void star_isp_reload_if_reset(star_state_t *st, bool force);

static void star_isp_arm_tuning_reads(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        if (g_iq[i].unity_from_tuning)
            g_iq[i].unity_stale = true;
}

void star_isp_tune_when_ready(star_state_t *st, bool verbose)
{
    int ret;

    if (!st || !st->isp_loaded)
        return;

    /*
     * Already tuned is not the same as still tuned. Something in the
     * bring-up after this function first runs tears the tuning down -- one
     * reload, once, right at startup on the board -- so every later call
     * checks instead of returning. This path is what makes the repair
     * reliable: it is driven by the pipeline being built, whereas the
     * check in hal_isp_get_exposure only happens because ric polls, and
     * with ric disabled nothing would ever look.
     */
    if (st->isp_tuned) {
        star_isp_reload_if_reset(st, true);
        return;
    }

    if (star_isp_wait_ready(st, verbose ? STAR_ISP_READY_TIMEOUT_MS : STAR_ISP_READY_QUICK_MS,
                            verbose) != RSS_OK)
        return;

    /* Ready is a one-way transition, so one attempt from here on. */
    st->isp_tuned = true;

    if (st->iq_file[0]) {
        /*
         * Load the binary and leave 3A alone, which is what divinus does
         * -- it sleeps a second and loads (media.c:827) -- and divinus is
         * the reference with known-good colour on this board.
         *
         * Do not bracket the load by stopping userspace 3A and restarting
         * CUS3A, the way waybeam does (star6e_pipeline.c:270-284).
         * MI_ISP_DisableUserspace3A tears the vendor algorithms down:
         * libmi_isp imports CUS3A_Init and CUS3A_EnableUserspaceAE/AWB/AF
         * from libcus3a, and that registration is what it undoes.
         * MI_ISP_CUS3A_Enable only sets flags and cannot put the
         * algorithms back -- the one entry point that can is
         * MI_ISP_EnableUserspace3A, a different symbol, which waybeam's
         * own 6E notes say not to call on the normal internal-AE path.
         *
         * The symptom is an auto white balance that is enabled with
         * nothing behind it: a magenta cast under artificial light, with
         * MI_ISP_CUS3A_Enable demonstrably having passed AWB = 1.
         */
        ret = st->isp.fnLoadChannelConfig(STAR_ISP_CHN, st->iq_file, STAR_IQ_LOAD_KEY);
        if (ret) {
            HAL_LOG_WARN("isp: loading %s failed: %d; the generic vendor tuning stays in "
                         "effect and colour will be off",
                         st->iq_file, ret);
            st->iq_file[0] = '\0';
        } else {
            HAL_LOG_INFO("isp: loaded tuning file %s", st->iq_file);
        }
    }

    /*
     * Snapshot the tuning's exposure limits before the knobs land on them,
     * so a requested gain ceiling can be judged against the calibration
     * rather than against whatever the previous write left behind.
     */
    star_isp_snapshot_bin_limits(st);

    /* Same reason, for the knobs whose baseline is the tuning's own value:
     * the fields have to be read before the flush writes to them. */
    star_isp_arm_tuning_reads();

    /* Config knobs go on after the tuning file, never before. */
    star_isp_flush_pending(st);

    /* Worth doing whether or not a tuning file loaded: the limits come
     * from whichever tuning is in effect, and neither is obliged to suit
     * the framerate this pipeline asked for. */
    star_isp_cap_exposure(st, st->fps);

    /*
     * Orientation needs no re-apply here. It lives in the VPE channel
     * param, which this code is the only writer of, and the channel is
     * created once in star_vpe_bringup and destroyed once in teardown --
     * so nothing a tuning load does can lose it. The sensor register this
     * replaced did need one, because MI_SNR_SetOrien leaves the write
     * pending on an AE frame notification that a reload can swallow.
     */
}

/*
 * Forget that the tuning was applied, so the next star_isp_tune_when_ready
 * does it again.
 *
 * Called when the last VPE output port goes down. The VPE channel only
 * runs while a port is enabled, and when it comes back CUS3A auto-starts
 * from scratch and loads the generic /etc/firmware/iqfile0.bin -- so the
 * sensor's own binary, the AE shutter cap and the control knobs are all
 * gone, and only a latch that survived the restart made it look otherwise.
 * That is what turned any hot restart (rvd's stream-restart,
 * set-resolution, set-codec, osd-restart) into generic colour for the rest
 * of the process's life, with nothing in the log to say so.
 */
void star_isp_untune(star_state_t *st)
{
    if (!st || !st->isp_tuned)
        return;

    st->isp_tuned = false;
    HAL_LOG_DBG("isp: VPE channel stopped; tuning will be re-applied when it restarts");
}

void star_isp_teardown(star_state_t *st)
{
    size_t i;

    if (!st || !st->isp_loaded)
        return;

    /* The cached function pointers belong to the handle about to be
     * closed, so they have to go with it. */
    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        g_iq[i].fn_get = NULL;
        g_iq[i].fn_set = NULL;
        g_iq[i].has_pending = false;
        g_iq[i].unity_stale = false;
    }

    i6_isp_unload(&st->isp);
    st->isp_loaded = false;
    st->isp_tuned = false;
    st->iq_file[0] = '\0';
}

/* ================================================================
 * OPS
 * ================================================================ */

int hal_isp_set_brightness(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_BRIGHTNESS, val);
}

int hal_isp_set_contrast(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_CONTRAST, val);
}

int hal_isp_set_saturation(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_SATURATION, val);
}

int hal_isp_set_sharpness(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_SHARPNESS, val);
}

int hal_isp_set_sinter_strength(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_SINTER, val);
}

int hal_isp_set_temper_strength(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_TEMPER, val);
}

int hal_isp_set_ae_comp(void *ctx, int val)
{
    return star_iq_set_scalar(ctx, IQ_EVCOMP, val);
}

int hal_isp_set_defog(void *ctx, int enable)
{
    return star_iq_set_raw(ctx, IQ_DEFOG, enable ? 1u : 0u);
}

int hal_isp_get_brightness(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_BRIGHTNESS, val);
}

int hal_isp_get_contrast(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_CONTRAST, val);
}

int hal_isp_get_saturation(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_SATURATION, val);
}

int hal_isp_get_sharpness(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_SHARPNESS, val);
}

int hal_isp_get_sinter_strength(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_SINTER, val);
}

int hal_isp_get_temper_strength(void *ctx, uint8_t *val)
{
    return star_iq_get_scalar(ctx, IQ_TEMPER, val);
}

int hal_isp_get_ae_comp(void *ctx, int *val)
{
    uint8_t v;
    int ret;

    if (!val)
        return RSS_ERR_INVAL;

    ret = star_iq_get_scalar(ctx, IQ_EVCOMP, &v);
    if (ret == RSS_OK)
        *val = v;

    return ret;
}

/*
 * Anti-flicker. The two enums are the same width and both bounded at 3,
 * but MI orders 60 Hz before 50 Hz, so they have to be translated rather
 * than passed through: DISABLE 0, 60HZ 1, 50HZ 2, AUTO 3. raptor has no
 * enumerator for AUTO, so nothing selects it and a read of it reports OFF.
 *
 * Applied unconditionally, unlike the tuning scalars: mains frequency
 * is a property of where the camera is installed, which a tuning file
 * shipped with a sensor cannot know.
 */
#define STAR_FLICKER_DISABLE 0
#define STAR_FLICKER_60HZ    1
#define STAR_FLICKER_50HZ    2

int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode)
{
    uint32_t raw;

    switch (mode) {
    case RSS_ANTIFLICKER_OFF:
        raw = STAR_FLICKER_DISABLE;
        break;
    case RSS_ANTIFLICKER_50HZ:
        raw = STAR_FLICKER_50HZ;
        break;
    case RSS_ANTIFLICKER_60HZ:
        raw = STAR_FLICKER_60HZ;
        break;
    default:
        HAL_LOG_WARN("isp: antiflicker mode %d out of range", (int)mode);
        return RSS_ERR_INVAL;
    }

    return star_iq_set_raw(ctx, IQ_FLICKER, raw);
}

int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode)
{
    uint32_t raw;
    int ret;

    if (!mode)
        return RSS_ERR_INVAL;

    ret = star_iq_get_raw(ctx, IQ_FLICKER, &raw);
    if (ret != RSS_OK)
        return ret;

    switch (raw) {
    case STAR_FLICKER_50HZ:
        *mode = RSS_ANTIFLICKER_50HZ;
        break;
    case STAR_FLICKER_60HZ:
        *mode = RSS_ANTIFLICKER_60HZ;
        break;
    default:
        *mode = RSS_ANTIFLICKER_OFF;
        break;
    }

    return ret;
}

/*
 * Gain ceilings.
 *
 * MI keeps both in the AE exposure-limit struct, so each setter is a
 * read-modify-write of the same 32 bytes star_isp_cap_exposure uses --
 * which is the point: writing the struct wholesale here would undo that
 * shutter cap.
 *
 * The units are MI's own and are not raptor's 0..255: they are x1024
 * fixed point, 1024 being unity. rvd's defaults (max_again 160, max_dgain
 * 80) are Ingenic gain codes, and rvd applies them whether or not the
 * config mentions the keys, so passing them straight through would write
 * sub-unity ceilings on every boot. Values below unity are refused and
 * values above the tuning's calibrated ceiling clamped to it -- see the
 * reasoning inside. Treat these two keys as MI-native on this platform.
 */
static int star_isp_apply_gain_limit(star_state_t *st, bool sensor_gain, int gain)
{
    i6_isp_exp limit;
    int ret;

    /* Guarded like every other vendor pointer in this file. i6_isp_load
     * refuses to report success without these two, so a live pipeline
     * always has them -- but this is reachable from the flush, and calling
     * through a null pointer is a worse answer than NOTSUP. */
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return RSS_ERR_NOTSUP;

    memset(&limit, 0, sizeof(limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    {
        unsigned int want = (unsigned int)gain;
        unsigned int bin_min = sensor_gain ? st->bin_min_sensor_gain : st->bin_min_isp_gain;
        unsigned int bin_max = sensor_gain ? st->bin_max_sensor_gain : st->bin_max_isp_gain;
        const char *which = sensor_gain ? "sensor" : "isp";

        /*
         * MI's ceilings are x1024 fixed point: 1024 is unity, and the
         * vendor's own constant for a 32x cap is 32768 (waybeam's
         * AE_GAIN_MAX_DEFAULT, "32x sensor cap"). raptor's max_again and
         * max_dgain keys are Ingenic gain codes, and rvd applies its
         * Ingenic defaults -- 160 and 80 -- on every platform whether or
         * not the config mentions them. Written through unscaled those are
         * ceilings of 0.16x and 0.08x: below unity, so not gain ceilings
         * at all. maxIspGain = 80 is the damaging one, because it pins the
         * ISP's digital gain at its floor and so removes all the headroom
         * above the sensor's own ceiling -- which is why total_gain on
         * this board stopped dead at 8192 (8x, the tuning's own
         * maxSensorGain) instead of climbing through it as the light went.
         *
         * Refused rather than scaled: no conversion turns an Ingenic gain
         * code into an MI one, so the only honest answer is to leave the
         * tuning's calibrated ceiling alone and say why, once per load.
         */
        if (want < 1024u) {
            HAL_LOG_WARN("isp: ignoring max %s gain %u -- MI wants x1024 units, so that "
                         "reads as %u.%02ux, a ceiling below unity. The tuning's own limit "
                         "stands. Set max_again/max_dgain in x1024 (1024 = 1.0x).",
                         which, want, want / 1024u, (want % 1024u) * 100u / 1024u);
            return RSS_ERR_INVAL;
        }

        /*
         * MI validates against the calibrated range and quietly keeps its
         * own value when a ceiling is out of it, so clamping here is only
         * about the log: an unexplained ceiling that did not take is much
         * harder to spot than one that says it was clamped. waybeam found
         * the same wall -- gainMax 32000 against a bin ceiling of 8192.
         */
        if (bin_max && want > bin_max) {
            HAL_LOG_INFO("isp: max %s gain %u is above the tuning's calibrated ceiling %u; "
                         "clamping, because MI does not honour a higher one",
                         which, want, bin_max);
            want = bin_max;
        }
        if (bin_min && want < bin_min) {
            HAL_LOG_INFO("isp: max %s gain %u is below the tuning's floor %u; raising to it",
                         which, want, bin_min);
            want = bin_min;
        }

        if (sensor_gain)
            limit.maxSensorGain = want;
        else
            limit.maxIspGain = want;
    }

    ret = st->isp.fnSetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_SetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_DBG("isp: max %s gain = %d", sensor_gain ? "sensor" : "isp", gain);
    return RSS_OK;
}

static int star_isp_get_gain_limit(void *ctx, bool sensor_gain, uint32_t *gain)
{
    star_state_t *st = star_state(ctx);
    i6_isp_exp limit;
    int ret;

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (!gain)
        return RSS_ERR_INVAL;

    if (!st->isp_tuned) {
        int pend = sensor_gain ? st->pend_max_again : st->pend_max_dgain;

        *gain = pend >= 0 ? (uint32_t)pend : 0u;
        return RSS_OK;
    }

    memset(&limit, 0, sizeof(limit));
    ret = st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_AE_GetExposureLimit failed: %d", ret);
        return RSS_ERR_IO;
    }

    *gain = sensor_gain ? limit.maxSensorGain : limit.maxIspGain;
    return RSS_OK;
}

static int star_isp_set_gain_limit(void *ctx, bool sensor_gain, int gain)
{
    star_state_t *st = star_state(ctx);

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (gain < 0)
        return RSS_ERR_INVAL;

    /* Recorded like the IQ knobs, and for the same two reasons: the ISP
     * may not be up yet, and a later tuning load will need it back. */
    if (sensor_gain)
        st->pend_max_again = gain;
    else
        st->pend_max_dgain = gain;

    if (!st->isp_tuned) {
        HAL_LOG_DBG("isp: max %s gain = %d queued until the ISP is up",
                    sensor_gain ? "sensor" : "isp", gain);
        return RSS_OK;
    }

    return star_isp_apply_gain_limit(st, sensor_gain, gain);
}

/*
 * Re-apply everything that has been asked for.
 *
 * Called from star_isp_tune_when_ready *after* the tuning binary has
 * loaded, which is the only correct order: applied first, the load would
 * overwrite them.
 *
 * Nothing is consumed here. Every load resets the modules to the
 * binary's own state, so these values have to be re-applied after each
 * one, not drained once -- see the comment on has_pending.
 */

static void star_isp_flush_pending(star_state_t *st)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        star_iq_param_t *p = &g_iq[i];

        if (!p->has_pending)
            continue;

        if (p->pending_is_raw)
            (void)star_iq_apply_raw(st, (int)i, (uint32_t)p->pending);
        else
            (void)star_iq_apply_scalar(st, (int)i, p->pending);
    }

    if (st->pend_max_again >= 0)
        (void)star_isp_apply_gain_limit(st, true, st->pend_max_again);
    if (st->pend_max_dgain >= 0)
        (void)star_isp_apply_gain_limit(st, false, st->pend_max_dgain);

}

int hal_isp_set_max_again(void *ctx, int gain)
{
    return star_isp_set_gain_limit(ctx, true, gain);
}

int hal_isp_set_max_dgain(void *ctx, int gain)
{
    return star_isp_set_gain_limit(ctx, false, gain);
}

int hal_isp_get_max_again(void *ctx, uint32_t *gain)
{
    return star_isp_get_gain_limit(ctx, true, gain);
}

int hal_isp_get_max_dgain(void *ctx, uint32_t *gain)
{
    return star_isp_get_gain_limit(ctx, false, gain);
}

/*
 * Exposure readback -- what the AE converged on, for ric's day/night
 * detection.
 *
 * MI looks at first as though it exposes no current-exposure query, and
 * two of the obvious symbols are indeed the wrong ones: AE_GetManualExpo
 * returns the manual *setting* and AE_GetExposureLimit the bounds. But
 * CUS3A_GetAeStatus returns what the AE actually converged on.
 *
 * Two calls, and the second one is optional. MI_ISP_CUS3A_GetAeStatus
 * gives shutter and both gains, which is a complete ambient-light signal
 * on its own: a scene going dark drives the shutter and then the gain up,
 * and ric's night->day rule compares gain against its own night baseline
 * so it needs no absolute scale. MI_ISP_AE_GetAeHwAvgStats adds scene
 * luma, which ric's day->night rule prefers because IR illumination does
 * not inflate it the way it inflates gain.
 *
 * Deliberately not filled: ev (an Ingenic GetEVAttr concept with no MI
 * equivalent) and wb_rgain/wb_bgain (MI_ISP_AWB_GetAwbHwAvgStats exists,
 * but its 128x90 grid is not the two global gains the field wants).
 * Leaving them zero is what tells ric the photo trigger has nothing to
 * work with -- see the zero convention below.
 *
 * ================================================================
 * ZERO MEANS "NOT AVAILABLE", AND THAT IS A CONTRACT, NOT A HABIT
 *
 * The Ingenic backend already works this way: when GetAeStatistics
 * fails it warns once and leaves ae_luma at 0 so that "day/night falls
 * back to gain-only behavior" (src/hal_isp.c). A backend that cannot
 * answer for a field leaves it zero, and the consumer must read zero as
 * silence rather than as a reading.
 *
 * It matters most for luma, where the two readings sit at opposite ends:
 * a live sensor never reports a mean luma of exactly 0, and ric's
 * day->night test is `ae_luma < night_luma`. So a zero read as data is
 * the darkest possible scene, so a backend with no luma source that
 * reported zero here would pin the camera in night mode forever.
 * ================================================================
 */

/* Both gains are x1024 fixed point (1024 = 1.0x), so multiplying two of
 * them needs the divide to get back to x1024 -- and 64-bit intermediates,
 * since 32x/32x overflows u32 at gain 64x. */
static uint32_t star_ae_total_gain(const i6_isp_ae_status *ae)
{
    uint64_t sensor = ae->sensorGain;
    uint64_t isp = ae->ispGain ? ae->ispGain : 1024u;
    uint64_t total = sensor * isp / 1024u;

    return total > UINT32_MAX ? UINT32_MAX : (uint32_t)total;
}

/*
 * Mean of the AE grid's Y lane, or 0 if the grid cannot be trusted.
 *
 * The layout is MI_ISP_AE_HW_STATISTICS_t: the grid dimensions lead and
 * the cells are r,g,b,y with luma at lane 3 (see i6_isp.h). What still has
 * to happen at runtime is reading the dimensions -- the buffer is sized
 * for the 128x90 maximum and this sensor reports 32x32, so averaging the
 * whole buffer would average 11,000 cells of zeros into the answer.
 *
 * The dimensions are checked against the AE status' own copy before
 * anything is averaged. It is the same witness the layout search used to
 * choose between two candidate placements, kept because a grid that
 * disagrees with itself is a reason to report no luma rather than a
 * plausible-looking number that will move the IR-cut filter.
 */
static uint32_t star_ae_luma(star_state_t *st, const i6_isp_ae_status *ae)
{
    static bool layout_logged;
    static bool layout_warned;
    i6_isp_ae_hw_stats *stats;
    const unsigned char *cell;
    unsigned int blk_x = ae->avgBlkX;
    unsigned int blk_y = ae->avgBlkY;
    unsigned int cells;
    uint64_t sum[I6_ISP_AE_CELL_SZ] = {0};
    uint32_t luma;
    int ret;

    if (!st->isp.fnGetAeHwAvgStats)
        return 0;

    if (blk_x == 0 || blk_x > I6_ISP_AE_BLK_X || blk_y == 0 || blk_y > I6_ISP_AE_BLK_Y) {
        if (!layout_warned) {
            HAL_LOG_WARN("isp: AE grid dimensions %ux%u are outside the %ux%u block -- "
                         "no scene luma, day/night falls back to gain only",
                         blk_x, blk_y, I6_ISP_AE_BLK_X, I6_ISP_AE_BLK_Y);
            layout_warned = true;
        }
        return 0;
    }

    /* 46KB, so off the stack. Same size every call, so the allocator
     * hands back the same chunk; ric polls this once a second. */
    stats = malloc(sizeof(*stats));
    if (!stats)
        return 0;

    memset(stats, 0, sizeof(*stats));
    ret = st->isp.fnGetAeHwAvgStats(STAR_ISP_CHN, stats);
    if (ret) {
        if (!layout_warned) {
            HAL_LOG_WARN("isp: MI_ISP_AE_GetAeHwAvgStats failed: %d -- no scene luma", ret);
            layout_warned = true;
        }
        free(stats);
        return 0;
    }

    if (stats->blkX != blk_x || stats->blkY != blk_y) {
        if (!layout_warned) {
            HAL_LOG_WARN("isp: AE stats report a %ux%u grid where the AE status says %ux%u -- "
                         "no scene luma, day/night falls back to gain only",
                         stats->blkX, stats->blkY, blk_x, blk_y);
            layout_warned = true;
        }
        free(stats);
        return 0;
    }

    cell = stats->cell;
    cells = blk_x * blk_y;
    for (unsigned int i = 0; i < cells; i++)
        for (unsigned int lane = 0; lane < I6_ISP_AE_CELL_SZ; lane++)
            sum[lane] += cell[i * I6_ISP_AE_CELL_SZ + lane];

    luma = (uint32_t)(sum[I6_ISP_AE_CELL_Y] / cells);

    /*
     * The grid and its lane means, once. Guarded against an all-zero
     * sample -- not hypothetical, since an ISP freshly reset to its
     * defaults reports exactly that, and a line of zeros would be read as
     * the layout having been confirmed against a black frame.
     *
     * The lane means are computed in the argument list rather than into an
     * array, because HAL_LOG_DBG compiles away entirely without HAL_DEBUG
     * and an array filled only for it would be an unused variable in every
     * release build.
     */
    if (!layout_logged && (sum[0] || sum[1] || sum[2] || sum[3])) {
        HAL_LOG_DBG("isp: AE grid %ux%u, lane means r=%u g=%u b=%u y=%u (y is the one used)",
                    blk_x, blk_y, (unsigned int)(sum[0] / cells), (unsigned int)(sum[1] / cells),
                    (unsigned int)(sum[2] / cells), (unsigned int)(sum[3] / cells));
        layout_logged = true;
    }

    free(stats);
    return luma;
}

#define STAR_LIMIT_REASSERT_S 2

/*
 * What the AE's ceilings should currently read: whatever the config asked
 * for, clamped the same way star_isp_apply_gain_limit clamps it, and zero
 * for "this config says nothing, so the tuning's own value stands".
 *
 * Shared by the two functions below because they have to agree. They ask
 * opposite questions of the same numbers -- one wants to know whether a
 * configured ceiling has been lost, the other whether the tuning itself
 * has -- and a ceiling one of them considered legitimate while the other
 * read it as a wiped ISP would have them undoing each other every couple
 * of seconds.
 */
static unsigned int star_isp_wanted_gain(const star_state_t *st)
{
    unsigned int want = st->pend_max_again >= 1024 ? (unsigned int)st->pend_max_again : 0u;

    if (want && st->bin_max_sensor_gain && want > st->bin_max_sensor_gain)
        want = st->bin_max_sensor_gain;

    return want;
}

static void star_isp_reassert_limits(star_state_t *st)
{
    i6_isp_exp limit;
    static time_t last;
    static bool reported;
    struct timespec now;
    unsigned int want_gain;

    if (st->pend_max_again < 1024)
        return;
    if (!st->isp.fnGetExposureLimit || !st->isp.fnSetExposureLimit)
        return;

    if (clock_gettime(CLOCK_MONOTONIC, &now))
        return;
    if (last && now.tv_sec - last < STAR_LIMIT_REASSERT_S)
        return;
    last = now.tv_sec;

    memset(&limit, 0, sizeof(limit));
    if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit))
        return;

    want_gain = star_isp_wanted_gain(st);
    if (!want_gain || limit.maxSensorGain == want_gain)
        return;

    if (!reported) {
        reported = true;
        HAL_LOG_INFO("isp: AE narrowed its sensor gain ceiling to ..%u; restoring the "
                     "configured ..%u and holding it",
                     limit.maxSensorGain, want_gain);
    }

    limit.maxSensorGain = want_gain;
    if (limit.minSensorGain > want_gain)
        limit.minSensorGain = want_gain;

    (void)st->isp.fnSetExposureLimit(STAR_ISP_CHN, &limit);
}

/*
 * Reload the tuning binary when the ISP is found back on its defaults.
 *
 * Loading the api bin plainly works: two different bins produce two
 * different AE limit sets (24576/100000 and 131072/50000), read back from
 * the AE immediately after the load. But `1024..8192` and `..14000` are
 * the limits the ISP reports with *no tuning file at all* -- a deliberately
 * failed load reports exactly those numbers -- and that is where this
 * board's AE otherwise sits, permanently.
 *
 * So the bin lands and is then wiped, and a whole family of symptoms
 * follows from that one cause: the AE ignoring limits widened underneath
 * it, an OEM _night bin that forces monochrome under divinus doing nothing
 * here, deleting the bin changing nothing, and white balance being wrong
 * under artificial light.
 *
 * What wipes it is CUS3A's own AE init, which reads the generic
 * `iqfile<n>.bin` from disk and takes the AE's limits from it. CUS3A defers
 * that init to its frame thread, so it lands *after* this file's load --
 * which happens at the first framesource enable, before the encoder threads
 * are even started, let alone a frame delivered.
 *
 * Ordering does fix it, and the evidence is accidental: a -O0 build of this
 * file brings the pipeline up slowly enough that the load falls on the
 * other side of the AE init, and then it survives -- zero reloads, tuning
 * limits still in place. A load issued from a second process minutes into a
 * run is likewise stable for as long as it is watched. So the load is not
 * doomed; it is early.
 *
 * What has been ruled out as a gate is a non-zero shutter from
 * MI_ISP_CUS3A_GetAeStatus: the sensor is already exposing before the init
 * that sets the limits, so it fires too soon and the wipe still follows.
 * The trigger that would work is the first frame actually delivered, which
 * is a signal the pipeline has and this file is not currently told about.
 *
 * Until it is, detect and repair -- which is needed anyway: CUS3A leaves
 * its init flag clear whenever the sensor is not up yet and retries on
 * every later frame interrupt, so a sensor re-init can re-read the iqfile
 * long after bring-up.
 *
 * The limits are a reliable witness because the tuning's own values were
 * snapshotted before anything could overwrite them, and because the only
 * other thing that writes them is this file -- so "neither what the
 * tuning published nor what the config asked for" means the ISP went back
 * to its defaults underneath us.
 *
 * A reload restores the tuning's own state, config knobs and all, which
 * is exactly what star_isp_flush_pending exists to put back; without that
 * call a repair would silently drop a configured ceiling. Bounded,
 * because a reload that does not stick must not turn into a loop -- and
 * if the bound is reached, the log says so, which is a better failure
 * than silence.
 */
#define STAR_IQ_RELOAD_MAX 5

static void star_isp_reload_if_reset(star_state_t *st, bool force)
{
    i6_isp_exp limit;
    static time_t last;
    struct timespec now;
    unsigned int want_gain;

    if (!st->iq_file[0] || !st->bin_max_sensor_gain)
        return;
    if (!st->isp.fnGetExposureLimit || !st->isp.fnLoadChannelConfig)
        return;

    /*
     * The interval only paces the polled caller. The bring-up path forces
     * the check, because its calls are seconds apart at most and the whole
     * point there is to repair the tear-down before frames flow rather
     * than a poll interval later.
     */
    if (clock_gettime(CLOCK_MONOTONIC, &now))
        return;
    if (!force && last && now.tv_sec - last < STAR_LIMIT_REASSERT_S)
        return;
    last = now.tv_sec;

    memset(&limit, 0, sizeof(limit));
    if (st->isp.fnGetExposureLimit(STAR_ISP_CHN, &limit))
        return;

    /*
     * Either reading means the tuning is in effect: the value it published,
     * or the narrower one this config deliberately asked for. Treating a
     * configured ceiling as evidence of a reset would have this reload on
     * every poll; ignoring it turns setting max_again into a way to switch
     * the repair off.
     *
     * The blind spot is a max_again that happens to equal the untuned
     * default, 8192 on this board: a reset then looks like the config
     * being honoured. Left alone rather than special-cased, because the
     * tuning's ceiling is the one worth having here and the config says to
     * leave max_again unset.
     */
    want_gain = star_isp_wanted_gain(st);
    if (limit.maxSensorGain == st->bin_max_sensor_gain ||
        (want_gain && limit.maxSensorGain == want_gain))
        return;

    if (st->iq_reloads >= STAR_IQ_RELOAD_MAX) {
        static bool gave_up;

        if (!gave_up) {
            gave_up = true;
            HAL_LOG_WARN("isp: AE still on sensor gain ..%u after %d reloads of %s (the tuning "
                         "has ..%u); giving up rather than looping -- something is resetting "
                         "the ISP after every load",
                         limit.maxSensorGain, st->iq_reloads, st->iq_file,
                         st->bin_max_sensor_gain);
        }
        return;
    }

    st->iq_reloads++;
    HAL_LOG_INFO("isp: AE limits are back on sensor gain ..%u where the tuning has ..%u -- CUS3A "
                 "re-read its own iqfile after the load; reloading %s (attempt %d)",
                 limit.maxSensorGain, st->bin_max_sensor_gain, st->iq_file, st->iq_reloads);

    if (st->isp.fnLoadChannelConfig(STAR_ISP_CHN, st->iq_file, STAR_IQ_LOAD_KEY)) {
        HAL_LOG_WARN("isp: reloading %s failed", st->iq_file);
        return;
    }

    /* The load replaced the ISP's state with the binary's own, so anything
     * the config asked for has to go back on -- same reason the first load
     * is followed by this call. A learned neutral is part of that state:
     * keeping the one from before the reload would leave the scale centred
     * on a value no longer in the field, for the rest of the run. */
    star_isp_arm_tuning_reads();
    star_isp_flush_pending(st);
}

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    star_state_t *st = star_state(ctx);
    i6_isp_ae_status ae;
    int ret;

    if (!st || !st->isp_loaded)
        return RSS_ERR_NOENT;
    if (!exposure)
        return RSS_ERR_INVAL;
    if (!st->isp.fnGetAeStatus)
        return RSS_ERR_NOTSUP;

    memset(exposure, 0, sizeof(*exposure));

    /*
     * Before the tuning binary lands the ISP channel is not up and the
     * call errors. That is a startup window, not a fault, and ric polls
     * through it once a second -- so say "busy" and log nothing.
     */
    if (!st->isp_tuned)
        return RSS_ERR_BUSY;

    memset(&ae, 0, sizeof(ae));
    ret = st->isp.fnGetAeStatus(STAR_ISP_CHN, &ae);
    if (ret) {
        static bool warned;

        if (!warned) {
            HAL_LOG_WARN("isp: MI_ISP_CUS3A_GetAeStatus failed: %d -- "
                         "no exposure readback, ric will hold its current mode",
                         ret);
            warned = true;
        }
        return RSS_ERR_IO;
    }

    exposure->exposure_time = ae.shutterUs;
    exposure->total_gain = star_ae_total_gain(&ae);
    exposure->ae_luma = star_ae_luma(st, &ae);

    star_isp_reload_if_reset(st, false);
    star_isp_reassert_limits(st);

    return RSS_OK;
}

/*
 * Day/night.
 *
 * On MI this is the colour-to-gray switch and nothing more. That is the
 * ISP half of a day/night transition -- under IR illumination the colour
 * channels carry no useful chroma, so monochrome output is both cleaner
 * and what the scene actually contains. Driving the physical IR-cut
 * filter is ric's GPIO work; this op does not pretend to do it, and a
 * board with no IR-cut wiring still benefits from the switch.
 */
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = star_iq_set_raw(ctx, IQ_GRAY, mode == RSS_ISP_NIGHT ? 1u : 0u);
    if (ret == RSS_OK) {
        st->gray = (mode == RSS_ISP_NIGHT);
        HAL_LOG_INFO("isp: %s mode", st->gray ? "night (monochrome)" : "day (colour)");
    }

    return ret;
}

int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode)
{
    uint32_t raw;
    int ret;

    if (!mode)
        return RSS_ERR_INVAL;

    ret = star_iq_get_raw(ctx, IQ_GRAY, &raw);
    if (ret == RSS_OK)
        *mode = raw ? RSS_ISP_NIGHT : RSS_ISP_DAY;

    return ret;
}

/*
 * Mirror and flip.
 *
 * The VPE channel's own mirror/flip, which is the same stage Ingenic's ISP
 * flip acts at: digital, downstream of the sensor, and settable whenever.
 * The vendor's stated reason for the fields existing is to cover sensors
 * that cannot flip themselves, so using them for every sensor costs
 * nothing and makes one path do the work.
 *
 * Not the sensor's MI_SNR_SetOrien, which was the earlier mechanism and is
 * best-effort by comparison: it only stores the value and sets a dirty
 * flag, leaving pCus_AEStatusNotify(CUS_FRAME_ACTIVE) to write the
 * register, so it lands on the next AE frame notification if 3A is running
 * and sits pending if it is not -- and MI_SNR_GetOrien cannot say which,
 * because the vendor driver answers it from its static default table
 * rather than the live value. A channel param has no pending state and
 * reads back what it is doing.
 *
 * Not the per-port DMA mirror either. That one is applied after the OSD,
 * so it would flip the timestamp along with the picture.
 *
 * Get-then-set, as the vendor requires: the SDK mutates what it is given
 * (3DNR level is clamped to the per-chip maximum internally), so a blind
 * write would put whatever this file last guessed back over it.
 */
static int star_isp_apply_orien(star_state_t *st, int mirror, int flip)
{
    i6e_vpe_para para;
    int ret;

    if (!st->vpe.fnGetChannelParam || !st->vpe.fnSetChannelParam)
        return RSS_ERR_NOTSUP;
    if (!st->vpe_chn_created)
        return RSS_ERR_NOENT;

    memset(&para, 0, sizeof(para));
    ret = st->vpe.fnGetChannelParam(STAR_VPE_CHN, (i6_vpe_para *)&para);
    if (ret) {
        HAL_LOG_WARN("isp: MI_VPE_GetChannelParam failed: %d", ret);
        return RSS_ERR_IO;
    }

    if (para.mirror == (mirror ? 1 : 0) && para.flip == (flip ? 1 : 0))
        return RSS_OK;

    para.mirror = mirror ? 1 : 0;
    para.flip = flip ? 1 : 0;

    ret = st->vpe.fnSetChannelParam(STAR_VPE_CHN, (i6_vpe_para *)&para);
    if (ret) {
        HAL_LOG_WARN("isp: MI_VPE_SetChannelParam(mirror=%d, flip=%d) failed: %d", para.mirror,
                     para.flip, ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_DBG("isp: orientation mirror=%d flip=%d", para.mirror, para.flip);
    return RSS_OK;
}

int hal_isp_set_hflip(void *ctx, int enable)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    star_state_t *st = star_state(ctx);
    int prev;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    prev = c->hflip_state[0];
    c->hflip_state[0] = enable ? 1 : 0;
    ret = star_isp_apply_orien(st, c->hflip_state[0], c->vflip_state[0]);
    if (ret != RSS_OK)
        c->hflip_state[0] = prev;

    return ret;
}

int hal_isp_set_vflip(void *ctx, int enable)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    star_state_t *st = star_state(ctx);
    int prev;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    prev = c->vflip_state[0];
    c->vflip_state[0] = enable ? 1 : 0;
    ret = star_isp_apply_orien(st, c->hflip_state[0], c->vflip_state[0]);
    if (ret != RSS_OK)
        c->vflip_state[0] = prev;

    return ret;
}

int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    star_state_t *st = star_state(ctx);
    i6e_vpe_para para;

    if (!st)
        return RSS_ERR_INVAL;

    /* Hardware first, cache only as the fallback: the point of moving off
     * the sensor register is that this can now be asked rather than
     * remembered. */
    memset(&para, 0, sizeof(para));
    if (st->vpe_chn_created && st->vpe.fnGetChannelParam &&
        !st->vpe.fnGetChannelParam(STAR_VPE_CHN, (i6_vpe_para *)&para)) {
        c->hflip_state[0] = para.mirror ? 1 : 0;
        c->vflip_state[0] = para.flip ? 1 : 0;
    }

    if (hflip)
        *hflip = c->hflip_state[0];
    if (vflip)
        *vflip = c->vflip_state[0];

    return RSS_OK;
}

/*
 * Sensor frame rate.
 *
 * MI_SNR_SetFps works on an enabled, streaming sensor -- measured, both
 * directions, with MI_SNR_GetFps reporting the new rate immediately and
 * the delivered stream following it. So this is a runtime attribute here
 * as it is on Ingenic, not a bring-up-only setting.
 *
 * Two things the vendor notes add. The valid range belongs to the mode
 * selected by MI_SNR_SetRes, not to the sensor, so a request is clamped to
 * the mode raptor picked rather than refused. And fps is accepted either
 * as whole frames or as milli-frames (min*1000 to max*1000), which is what
 * makes a rational like 30000/1001 expressible.
 *
 * What does not need doing: no rebind. MI_SYS_BindChnPort2's source and
 * destination rates are fixed at bind time and cannot be re-set without
 * unbinding, but they are not a ratio -- a port bound to a lower
 * destination rate holds that rate as an absolute target across a sensor
 * change, and a port bound pass-through delivers whatever arrives. The
 * only limit is that nothing can deliver more than the sensor produces.
 */
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den)
{
    star_state_t *st = star_state(ctx);
    unsigned int milli, min_milli, max_milli;
    int ret;

    if (!st || !fps_num || !fps_den)
        return RSS_ERR_INVAL;
    if (!st->snr.fnSetFramerate)
        return RSS_ERR_NOTSUP;

    milli = (unsigned int)(((uint64_t)fps_num * 1000u) / fps_den);
    if (!milli)
        return RSS_ERR_INVAL;

    min_milli = st->res.minFps * 1000u;
    max_milli = st->res.maxFps * 1000u;
    if (max_milli && milli > max_milli) {
        HAL_LOG_WARN("isp: %u.%03u fps is above the mode's %u fps maximum; using %u", milli / 1000,
                     milli % 1000, st->res.maxFps, st->res.maxFps);
        milli = max_milli;
    } else if (min_milli && milli < min_milli) {
        HAL_LOG_WARN("isp: %u.%03u fps is below the mode's %u fps minimum; using %u", milli / 1000,
                     milli % 1000, st->res.minFps, st->res.minFps);
        milli = min_milli;
    }

    /* Whole frames when the request is one, milli-frames when it is not:
     * MI takes both, and the whole number is what every reference passes. */
    ret = st->snr.fnSetFramerate(STAR_SNR_INDEX, milli % 1000 ? milli : milli / 1000);
    if (ret) {
        HAL_LOG_WARN("isp: MI_SNR_SetFps(%u.%03u) failed: %d", milli / 1000, milli % 1000, ret);
        return RSS_ERR_IO;
    }

    st->fps_milli = milli;
    st->fps = (milli + 500) / 1000;
    HAL_LOG_INFO("isp: sensor fps %u.%03u", milli / 1000, milli % 1000);

    /*
     * The shutter ceiling is a function of the frame period, so it moves
     * with the rate -- otherwise a drop to 15 fps would keep a 33 ms
     * ceiling it no longer needs, and a rise to 30 would leave a 66 ms one
     * the sensor cannot honour without slowing straight back down.
     */
    (void)star_isp_cap_exposure(st, st->fps);

    return RSS_OK;
}

int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den)
{
    star_state_t *st = star_state(ctx);
    unsigned int fps = 0;

    if (!st || !fps_num || !fps_den)
        return RSS_ERR_INVAL;

    if (st->snr.fnGetFramerate && !st->snr.fnGetFramerate(STAR_SNR_INDEX, &fps) && fps) {
        /*
         * MI answers in whatever unit it was set in. A value past the
         * mode's maximum is therefore milli-frames rather than an
         * impossible rate -- there is no other way to tell them apart, and
         * the mode's own ceiling is the only boundary either side of which
         * the reading makes sense.
         */
        if (st->res.maxFps && fps > st->res.maxFps) {
            *fps_num = fps;
            *fps_den = 1000;
        } else {
            *fps_num = fps;
            *fps_den = 1;
        }
        return RSS_OK;
    }

    if (!st->fps)
        return RSS_ERR_NOTSUP;

    *fps_num = st->fps;
    *fps_den = 1;
    return RSS_OK;
}
