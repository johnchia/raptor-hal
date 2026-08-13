/*
 * infinity6c/hal_isp.c -- ISP tuning ops, MI 3.0
 *
 * Three unrelated mechanisms live here because raptor presents them as one
 * surface, and knowing which is which is most of understanding this file.
 *
 * The exposure readback ric polls for its day/night decision comes from CUS3A,
 * the running algorithm, through MI_ISP_CUS3A_GetAeStatus / GetAwbStatus -- not
 * the MI_ISP_AE_* query layer, which reports API-set values that stay at their
 * defaults while CUS3A owns 3A.
 *
 * The image knobs go through the tuning API, which is one wrapper shape for
 * every module: a descriptor naming a payload length and an api id, handed to
 * MI_ISP_GENERAL_{Set,Get}IspApiData. So a module here is a name and two numbers
 * in g_iq rather than a typed call, the numbers live in sigmastar-headers where
 * tests/abi_iq_i6c.c asserts them against the vendor headers, and adding a
 * module is a table row.
 *
 * Orientation and the 3DNR level are neither: they are fields of the ISP
 * channel's parameter block, set with MI_ISP_SetChnParam. That block is written
 * whole, so they are read-modify-written rather than assigned.
 *
 * Each is queried per (device, channel), and the device index carries the SoC id
 * in its high halfword, so I6C_DEV_ID composes it as elsewhere.
 *
 * THE TUNING BINARY OUTRANKS THE CONFIG
 *
 * A value written before the first frame does not survive. The IQ binary loads
 * on that frame and CUS3A initialises its AE there too, and both write over the
 * API store -- so an early write succeeds, does not take effect, and reports
 * nothing. rvd applies its whole [image] block during pipeline construction,
 * which is well before any frame, so every one of those calls would be lost.
 *
 * Hence the queue: a knob asked for before st->isp_knobs_live is recorded and
 * applied by i6c_isp_flush_knobs, which i6c_isp_note_frame calls once the load
 * and the 3A arm are done. Entries are not cleared by the flush, because a
 * pipeline rebuilt in the same process reloads the tuning and puts every module
 * back to what the binary says -- so the last value asked for is also what a
 * rebuild has to restore.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "infinity6c_state.h"

/* raptor's scale is 0..255 with 128 meaning "leave it where the tuning put it". */
#define I6C_ISP_NEUTRAL 128

/* Offsets and values inside a { bEnable, enOpType, stAuto[16], stManual } payload. */
#define I6C_ISP_ENABLE_OFF 0u
#define I6C_ISP_OPTYPE_OFF 4u
#define I6C_ISP_OP_AUTO 0u
#define I6C_ISP_OP_MANUAL 1u

/*
 * Big enough for every row in g_iq, checked against each row's payload before
 * the buffer is used. Saturation's 416 is the largest; the two modules with
 * per-band tables are deliberately not in the table (see i6c_isp.h) and would
 * otherwise pull this to 6264.
 */
#define I6C_IQ_PAYLOAD_MAX 512

/*
 * MI's flicker enum, which is not raptor's. MI orders 60Hz before 50Hz and
 * raptor the other way, so this is a mapping and not a cast -- getting it wrong
 * would band the picture on exactly the mains frequency the operator excluded.
 */
#define I6C_FLICKER_DISABLE 0u
#define I6C_FLICKER_60HZ 1u
#define I6C_FLICKER_50HZ 2u

/*
 * How far ae_comp moves the AE's target, each way, in MI's EV units.
 *
 * Carried over from the Infinity6E port, where it was measured: from a baseline
 * of 41 the scene luma rises to 56 at EV 3, 83 at 9, 105 at 14 and 123 at 20 --
 * and then stops, with EV 25 through 200 all giving exactly luma 123. The clip
 * is a property of the AE's target curve rather than of the scene, and both
 * generations run the same CUS3A algorithm over a curve the tuning binary
 * supplies, so the span is expected to carry. It has not been re-measured here.
 *
 * The floor is the negative of it: MI_ISP_AE_EvCompType_t.s32EV is signed, and
 * with the tuning's baseline at 0 an unsigned range leaves raptor's whole lower
 * half inert.
 */
#define I6C_AE_EV_SPAN 20

static infinity6c_state_t *i6c_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (infinity6c_state_t *)c->platform : NULL;
}

/*
 * The 3A state at the first successful readback, logged once. It is how the
 * colour path is diagnosed without a board in hand: the white-balance gains say
 * where AWB has settled, and whether they move when the illuminant changes says
 * whether it is adapting at all. Gains are 1024-per-unit; the log prints the raw
 * values so the unit can be checked against the scene.
 */
static void i6c_isp_log_3a(const i6c_cus_ae_info *ae, const i6c_cus_awb_info *awb)
{
    HAL_LOG_INFO("isp/ae: shutter=%uus sensorgain=%u ispgain=%u avgY=%u", ae->shutterUs,
                 ae->sensorGain, ae->ispGain, ae->preAvgY);

    if (awb)
        HAL_LOG_INFO("isp/awb: gains r=%u g=%u b=%u (1024=1x)", awb->rGain, awb->gGain, awb->bGain);
}

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    infinity6c_state_t *st = i6c_state(ctx);
    static bool diag_logged;
    i6c_cus_awb_info awb;
    i6c_cus_ae_info ae;
    bool have_awb;
    int ret;

    if (!st || !exposure)
        return RSS_ERR_INVAL;
    if (!st->isp.ae_status)
        return RSS_ERR_NOTSUP;

    memset(exposure, 0, sizeof(*exposure));

    /*
     * Until the first frame CUS3A has not run and a status query returns
     * nothing useful. ric polls through that window once a second, so answer
     * "busy" and say nothing.
     */
    if (!st->iq_load_started)
        return RSS_ERR_BUSY;

    memset(&ae, 0, sizeof(ae));
    ret = st->isp.ae_status(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &ae);
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
    /*
     * Combined analogue and ISP gain, kept in the same 1024-per-unit the parts
     * come in: the product carries the unit twice, so one factor is divided back
     * out.
     */
    exposure->total_gain = (uint32_t)(((uint64_t)ae.sensorGain * ae.ispGain) >> 10);
    exposure->ae_luma = ae.preAvgY;

    have_awb =
        st->isp.awb_status && st->isp.awb_status(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &awb) == 0;
    if (have_awb) {
        exposure->wb_rgain = (uint16_t)awb.rGain;
        exposure->wb_bgain = (uint16_t)awb.bGain;
    }

    if (!diag_logged) {
        diag_logged = true;
        i6c_isp_log_3a(&ae, have_awb ? &awb : NULL);
    }

    return RSS_OK;
}

/*
 * hal_isp_get_sensor_attr -- the sensor's resolution, for the daemon's stream
 * sizing. Without it the daemon falls back to a config default and the scaler
 * quietly downsamples a full-resolution sensor to it.
 *
 * Once the pipeline is up the selected mode's geometry is the answer. Before
 * that -- which is when the daemon asks, since it sizes the streams before it
 * creates the channels that build the pipeline -- the sensor's native geometry
 * is read from the driver's mode list directly. That list is filled from the
 * driver's static tables at registration, so it is available with no I2C and
 * without enabling the sensor; the largest mode is the native array. The probe
 * only reads and sets a plane-mode flag the real bringup sets again, so it does
 * not disturb the mode the pipeline later selects. If the driver has registered
 * no modes yet it reports "none", and the daemon keeps its config fallback.
 */
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height)
{
    infinity6c_state_t *st = i6c_state(ctx);
    uint32_t best_w = 0, best_h = 0;
    unsigned int count = 0, i;

    if (!st || !width || !height)
        return RSS_ERR_INVAL;

    if (st->pipeline_up) {
        *width = st->plane.capt.width;
        *height = st->plane.capt.height;
        return RSS_OK;
    }

    if (!st->snr.set_plane_mode || !st->snr.query_res_count || !st->snr.get_res)
        return RSS_ERR_NOTSUP;

    if (st->snr.set_plane_mode(I6C_DEV_ID(I6C_SNR_PAD), 0) != 0 ||
        st->snr.query_res_count(I6C_DEV_ID(I6C_SNR_PAD), &count) != 0 || !count)
        return RSS_ERR_NOENT;

    for (i = 0; i < count; i++) {
        i6c_snr_res res;

        memset(&res, 0, sizeof(res));
        if (st->snr.get_res(I6C_DEV_ID(I6C_SNR_PAD), (unsigned char)i, &res) != 0)
            return RSS_ERR_IO;

        if ((uint32_t)res.crop.width * res.crop.height > best_w * best_h) {
            best_w = res.crop.width;
            best_h = res.crop.height;
        }
    }

    if (!best_w || !best_h)
        return RSS_ERR_NOENT;

    *width = best_w;
    *height = best_h;
    return RSS_OK;
}

/* ================================================================
 * THE TUNING API
 * ================================================================ */

typedef enum {
    IQ_FLAT,   /* the value is the whole payload, at offset 0 */
    IQ_BOOL,   /* bEnable at offset 0 is itself the value */
    IQ_AUTOMAN /* bEnable, enOpType, stAuto[16], stManual at manual_off */
} i6c_iq_shape_t;

typedef struct {
    const char *name; /* for diagnostics only */
    const char *get_sym;
    const char *set_sym;
    uint16_t payload;    /* the wrapper's own declared payload length */
    uint16_t manual_off; /* where the value lives (0 for FLAT and BOOL) */
    uint8_t width;       /* 1, 2 or 4 bytes */
    uint8_t shape;       /* i6c_iq_shape_t */
    int32_t mi_max;      /* MI's maximum for the field */
    int32_t mi_unity;    /* the MI value that means the same as raptor's 128 */
    int32_t mi_floor;    /* MI's minimum: 0 unless the field is signed */
    /*
     * Set for a module whose neutral is not a constant this port can know: the
     * baseline is whatever the tuning binary left in the field, and mi_unity is
     * only the fallback for a board running with no tuning file at all.
     */
    bool unity_from_tuning;

    /* Resolved on first use and cached; the symbol never changes. */
    i6c_isp_cmd_fn fn_get;
    i6c_isp_cmd_fn fn_set;

    /* Asked for before the ISP would keep it; see the file header. */
    int pending;
    bool has_pending;
    bool pending_is_raw;

    /* Armed by each tuning load, cleared by the fetch that reads the baseline
     * back out. Only meaningful with unity_from_tuning. */
    bool unity_stale;
} i6c_iq_param_t;

enum {
    IQ_BRIGHTNESS,
    IQ_CONTRAST,
    IQ_SATURATION,
    IQ_DEFOG,
    IQ_DEFOG_EN,
    IQ_GRAY,
    AE_EVCOMP,
    AE_FLICKER,
    IQ_PARAM_COUNT
};

/*
 * Payload sizes and manual offsets come from sigmastar-headers, where the vendor
 * ABI belongs and where tests/abi_iq_i6c.c asserts every one against the vendor
 * headers. mi_unity is the MI value raptor's neutral 128 maps to, which is what
 * keeps a default config from shifting the image:
 *
 *   brightness/contrast  u32Lev 0..100, midpoint 50
 *   saturation           u8SatAllStr 0..127 where 32 is unity gain (1X), *not*
 *                        the midpoint -- a linear 0..255 -> 0..127 map would
 *                        silently double saturation at raptor's neutral
 *   defog                u8Strength 0..255, and the module's own bEnable is what
 *                        isp_set_defog switches
 *   EV compensation      signed, +/-I6C_AE_EV_SPAN about the tuning's own value
 *
 * Single-instance, like the Infinity6E table: the resolved symbols and the
 * pending queue belong to one HAL context, which is what rvd creates. Cleared by
 * i6c_isp_forget_knobs at teardown.
 */
static i6c_iq_param_t g_iq[IQ_PARAM_COUNT] = {
    [IQ_BRIGHTNESS] = {"brightness", "MI_ISP_IQ_GetBrightness", "MI_ISP_IQ_SetBrightness",
                       I6C_ISP_IQ_BRIGHTNESS_PAYLOAD, I6C_ISP_IQ_BRIGHTNESS_MANUAL, 4, IQ_AUTOMAN,
                       100, 50, 0, false, NULL, NULL, 0, false, false, false},
    [IQ_CONTRAST] = {"contrast", "MI_ISP_IQ_GetContrast", "MI_ISP_IQ_SetContrast",
                     I6C_ISP_IQ_CONTRAST_PAYLOAD, I6C_ISP_IQ_CONTRAST_MANUAL, 4, IQ_AUTOMAN, 100,
                     50, 0, false, NULL, NULL, 0, false, false, false},
    [IQ_SATURATION] = {"saturation", "MI_ISP_IQ_GetSaturation", "MI_ISP_IQ_SetSaturation",
                       I6C_ISP_IQ_SATURATION_PAYLOAD, I6C_ISP_IQ_SATURATION_MANUAL, 1, IQ_AUTOMAN,
                       127, 32, 0, false, NULL, NULL, 0, false, false, false},
    /*
     * Defog is the one module whose Infinity6E counterpart is a bare enable. Here
     * it is a full auto/manual module with a strength byte, so raptor's two ops --
     * a level and a switch -- land on two fields of it.
     *
     * Two rows rather than one, addressing the same module through the same pair of
     * symbols. A row owns one queued value, and one row would mean isp_set_defog
     * and isp_set_defog_strength overwriting each other's request while both are
     * still queued -- which is exactly what rvd does, since its [image] block sets
     * both before the first frame.
     */
    [IQ_DEFOG] = {"defog", "MI_ISP_IQ_GetDefog", "MI_ISP_IQ_SetDefog", I6C_ISP_IQ_DEFOG_PAYLOAD,
                  I6C_ISP_IQ_DEFOG_MANUAL, 1, IQ_AUTOMAN, 255, 128, 0, false, NULL, NULL, 0, false,
                  false, false},
    [IQ_DEFOG_EN] = {"defog enable", "MI_ISP_IQ_GetDefog", "MI_ISP_IQ_SetDefog",
                     I6C_ISP_IQ_DEFOG_PAYLOAD, I6C_ISP_ENABLE_OFF, 4, IQ_BOOL, 1, 0, 0, false, NULL,
                     NULL, 0, false, false, false},
    [IQ_GRAY] = {"gray", "MI_ISP_IQ_GetColorToGray", "MI_ISP_IQ_SetColorToGray",
                 I6C_ISP_IQ_GRAY_PAYLOAD, 0, 4, IQ_BOOL, 1, 0, 0, false, NULL, NULL, 0, false,
                 false, false},
    /*
     * The only row whose neutral has to be learned and the only one whose MI
     * field is signed. It is IQ_FLAT, so there is no auto mode to hand it back
     * to; what it does is shift the AE's target luma, so a neutral guessed wrong
     * shifts every default image.
     */
    [AE_EVCOMP] = {"ae_comp", "MI_ISP_AE_GetEvComp", "MI_ISP_AE_SetEvComp",
                   I6C_ISP_AE_EVCOMP_PAYLOAD, 0, 4, IQ_FLAT, I6C_AE_EV_SPAN, 0, -I6C_AE_EV_SPAN,
                   true, NULL, NULL, 0, false, false, false},
    [AE_FLICKER] = {"antiflicker", "MI_ISP_AE_GetFlicker", "MI_ISP_AE_SetFlicker",
                    I6C_ISP_AE_FLICKER_PAYLOAD, 0, 4, IQ_FLAT, 3, 0, 0, false, NULL, NULL, 0, false,
                    false, false},
};

/* memcpy rather than a cast: the payload is a byte buffer and these offsets
 * carry no alignment guarantee. */
static uint32_t i6c_iq_read(const uint8_t *buf, uint16_t off, uint8_t width)
{
    uint32_t v = 0;

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

static void i6c_iq_write(uint8_t *buf, uint16_t off, uint8_t width, uint32_t val)
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

/* Resolve and cache a module's getter and setter. */
static int i6c_iq_resolve(infinity6c_state_t *st, i6c_iq_param_t *p)
{
    if (p->fn_get && p->fn_set)
        return RSS_OK;

    if (!st->isp.lib)
        return RSS_ERR_NOENT;

    if (p->payload > I6C_IQ_PAYLOAD_MAX) {
        HAL_LOG_ERR("isp: %s payload %u exceeds the %u-byte buffer", p->name, p->payload,
                    I6C_IQ_PAYLOAD_MAX);
        return RSS_ERR_INVAL;
    }

    p->fn_get = (i6c_isp_cmd_fn)hal_symbol_load("i6c_isp", st->isp.lib, p->get_sym);
    p->fn_set = (i6c_isp_cmd_fn)hal_symbol_load("i6c_isp", st->isp.lib, p->set_sym);
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
 * Gated on unity_stale, which each tuning load re-arms and the flush's first
 * fetch clears -- so what it reads is the binary's value and not one of ours.
 * Without it the sub-neutral half of the knob is measured from a guess.
 */
static void i6c_iq_learn_unity(i6c_iq_param_t *p, const uint8_t *buf)
{
    int32_t base;

    if (!p->unity_from_tuning || !p->unity_stale)
        return;

    /* The cast is the whole of the sign extension, which holds because the one
     * signed field here is four bytes wide. */
    base = (int32_t)i6c_iq_read(buf, p->manual_off, p->width);
    if (base > p->mi_max || base < p->mi_floor) {
        /* Out of range means the offset or width is wrong, and a baseline
         * adopted from a misread field would be invisible afterwards. */
        HAL_LOG_WARN("isp: %s reads MI %d, outside its %d..%d range -- not adopting it as the "
                     "neutral, keeping %d",
                     p->name, base, p->mi_floor, p->mi_max, p->mi_unity);
        p->unity_stale = false;
        return;
    }

    p->mi_unity = base;
    p->unity_stale = false;
    HAL_LOG_INFO("isp: %s baseline from the tuning is MI %d in %d..%d", p->name, base, p->mi_floor,
                 p->mi_max);
}

/*
 * Read the module's current payload. Callers modify one field and hand it back to
 * i6c_iq_store, so the tuning we cannot describe is preserved rather than zeroed.
 */
static int i6c_iq_fetch(infinity6c_state_t *st, int idx, uint8_t *buf)
{
    i6c_iq_param_t *p = &g_iq[idx];
    int ret;

    ret = i6c_iq_resolve(st, p);
    if (ret != RSS_OK)
        return ret;

    memset(buf, 0, p->payload);
    ret = p->fn_get(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, buf);
    if (ret) {
        HAL_LOG_WARN("isp: %s (%s) failed: %d", p->name, p->get_sym, ret);
        return RSS_ERR_IO;
    }

    i6c_iq_learn_unity(p, buf);
    return RSS_OK;
}

static int i6c_iq_store(infinity6c_state_t *st, int idx, uint8_t *buf)
{
    i6c_iq_param_t *p = &g_iq[idx];
    int ret;

    (void)st;
    ret = p->fn_set(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, buf);
    if (ret) {
        HAL_LOG_WARN("isp: %s (%s) failed: %d", p->name, p->set_sym, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * Map raptor's 0..255 onto MI's range, piecewise so neutral lands exactly on
 * MI's own unity value. A single linear map would not: with saturation's unity at
 * 32 of 127, linear scaling puts raptor's neutral at 64 -- twice unity gain -- and
 * every default config would boost colour.
 */
static int32_t i6c_iq_scale(int val, int32_t unity, int32_t floor, int32_t max)
{
    if (val <= 0)
        return floor;
    if (val >= 255)
        return max;
    if (val == I6C_ISP_NEUTRAL || unity >= max)
        return unity;

    if (val < I6C_ISP_NEUTRAL)
        return floor + (int32_t)(((int64_t)val * (unity - floor)) / I6C_ISP_NEUTRAL);

    return unity +
           (int32_t)(((int64_t)(val - I6C_ISP_NEUTRAL) * (max - unity)) / (255 - I6C_ISP_NEUTRAL));
}

/* Inverse of i6c_iq_scale, for the getters. */
static uint8_t i6c_iq_unscale(int32_t mi, int32_t unity, int32_t floor, int32_t max)
{
    if (max == floor || mi >= max)
        return 255;
    if (unity >= max)
        return I6C_ISP_NEUTRAL;
    /*
     * Ahead of the floor test, which would otherwise swallow it: when the learned
     * baseline sits on the floor, MI 0 is neutral rather than the bottom.
     */
    if (mi == unity)
        return I6C_ISP_NEUTRAL;
    if (mi <= floor)
        return 0;

    if (mi < unity)
        return (uint8_t)(((int64_t)(mi - floor) * I6C_ISP_NEUTRAL) / (unity - floor));

    return (uint8_t)(I6C_ISP_NEUTRAL +
                     ((int64_t)(mi - unity) * (255 - I6C_ISP_NEUTRAL)) / (max - unity));
}

/*
 * Apply one of raptor's 0..255 scalars.
 *
 * Neutral restores auto and leaves the manual field alone -- see THE TUNING
 * BINARY OUTRANKS THE CONFIG. bEnable is never touched here: if the tuning
 * disabled a module, re-enabling it behind the tuner's back is not this layer's
 * call.
 */
static int i6c_iq_apply_scalar(infinity6c_state_t *st, int idx, int val)
{
    i6c_iq_param_t *p = &g_iq[idx];
    uint8_t buf[I6C_IQ_PAYLOAD_MAX];
    int32_t mi_val;
    int ret;

    ret = i6c_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    if (p->shape == IQ_AUTOMAN) {
        if (val == I6C_ISP_NEUTRAL) {
            i6c_iq_write(buf, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_AUTO);
            HAL_LOG_DBG("isp: %s left to the tuning file (auto)", p->name);
            return i6c_iq_store(st, idx, buf);
        }
        i6c_iq_write(buf, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_MANUAL);
    }

    mi_val = i6c_iq_scale(val, p->mi_unity, p->mi_floor, p->mi_max);
    /* The cast is the two's-complement pattern MI wants for a negative field,
     * and a no-op for every other row. */
    i6c_iq_write(buf, p->manual_off, p->width, (uint32_t)mi_val);

    ret = i6c_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %d (MI %d in %d..%d)", p->name, val, mi_val, p->mi_floor, p->mi_max);

    return ret;
}

/*
 * Write a field MI defines as an enum or a flag, with no scaling. The row says
 * where and how wide, so an enable row addresses bEnable as the four-byte enum it
 * is -- writing one byte of it would leave the other three as fetched, which is
 * right until the day the value is not small.
 */
static int i6c_iq_apply_raw(infinity6c_state_t *st, int idx, uint32_t raw)
{
    i6c_iq_param_t *p = &g_iq[idx];
    uint8_t buf[I6C_IQ_PAYLOAD_MAX];
    int ret;

    ret = i6c_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    i6c_iq_write(buf, p->manual_off, p->width, raw);

    ret = i6c_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %u (raw)", p->name, raw);

    return ret;
}

/*
 * Queue-or-apply, split from the apply so the flush can drain the queue without
 * re-entering it, and so "is the ISP keeping writes yet" is asked in one place.
 *
 * The value is recorded whether or not it could be applied now: a tuning reload
 * resets each module to what the binary says, so the last value asked for is what
 * a re-tune has to put back.
 */
static int i6c_iq_set(void *ctx, int idx, int val, bool raw)
{
    infinity6c_state_t *st = i6c_state(ctx);
    i6c_iq_param_t *p = &g_iq[idx];

    if (!st)
        return RSS_ERR_INVAL;

    p->pending = val;
    p->pending_is_raw = raw;
    p->has_pending = true;

    if (!st->isp_knobs_live) {
        HAL_LOG_DBG("isp: %s = %d held until the tuning has loaded", p->name, val);
        return RSS_OK;
    }

    return raw ? i6c_iq_apply_raw(st, idx, (uint32_t)val) : i6c_iq_apply_scalar(st, idx, val);
}

static int i6c_iq_set_scalar(void *ctx, int idx, int val)
{
    if (val < 0 || val > 255)
        return RSS_ERR_INVAL;

    return i6c_iq_set(ctx, idx, val, false);
}

static int i6c_iq_set_raw(void *ctx, int idx, uint32_t raw)
{
    return i6c_iq_set(ctx, idx, (int)raw, true);
}

/*
 * Read a knob back.
 *
 * The hardware is asked rather than the queue, so what comes back is what the ISP
 * is doing -- except before the first frame, when there is nothing to ask: the
 * module still holds the tuning's value and the queued one is what the operator
 * set, so the queue is the better answer there.
 */
static int i6c_iq_get_scalar(void *ctx, int idx, uint8_t *val)
{
    infinity6c_state_t *st = i6c_state(ctx);
    i6c_iq_param_t *p = &g_iq[idx];
    uint8_t buf[I6C_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !val)
        return RSS_ERR_INVAL;

    if (!st->isp_knobs_live) {
        if (!p->has_pending || p->pending_is_raw)
            return RSS_ERR_BUSY;
        *val = (uint8_t)p->pending;
        return RSS_OK;
    }

    ret = i6c_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    /*
     * An auto module is reporting the tuning's value, not one of raptor's, and
     * neutral is exactly what raptor calls that.
     */
    if (p->shape == IQ_AUTOMAN && i6c_iq_read(buf, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO) {
        *val = I6C_ISP_NEUTRAL;
        return RSS_OK;
    }

    *val = i6c_iq_unscale((int32_t)i6c_iq_read(buf, p->manual_off, p->width), p->mi_unity,
                          p->mi_floor, p->mi_max);
    return RSS_OK;
}

static int i6c_iq_get_raw(void *ctx, int idx, uint32_t *raw)
{
    infinity6c_state_t *st = i6c_state(ctx);
    i6c_iq_param_t *p = &g_iq[idx];
    uint8_t buf[I6C_IQ_PAYLOAD_MAX];
    int ret;

    if (!st || !raw)
        return RSS_ERR_INVAL;

    if (!st->isp_knobs_live) {
        if (!p->has_pending || !p->pending_is_raw)
            return RSS_ERR_BUSY;
        *raw = (uint32_t)p->pending;
        return RSS_OK;
    }

    ret = i6c_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    *raw = i6c_iq_read(buf, p->manual_off, p->width);
    return RSS_OK;
}

void i6c_isp_flush_knobs(infinity6c_state_t *st)
{
    size_t i;

    if (!st)
        return;

    /*
     * Set before the loop rather than after, so a knob changed from another
     * thread while this runs takes the direct path instead of being queued for a
     * flush that has already passed it.
     */
    st->isp_knobs_live = true;

    /* Every baseline read out of the tuning is stale now the tuning has just
     * loaded, and the fetches below are where they are re-read. */
    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].unity_stale = g_iq[i].unity_from_tuning;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        i6c_iq_param_t *p = &g_iq[i];

        if (!p->has_pending)
            continue;

        if (p->pending_is_raw)
            i6c_iq_apply_raw(st, (int)i, (uint32_t)p->pending);
        else
            i6c_iq_apply_scalar(st, (int)i, p->pending);
    }
}

void i6c_isp_forget_knobs(void)
{
    size_t i;

    /*
     * The cached pointers belong to the library handle about to be closed, so
     * they go with it. The pending values stay: they are what the operator asked
     * for, and a pipeline rebuilt in the same process has to put them back.
     */
    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        g_iq[i].fn_get = NULL;
        g_iq[i].fn_set = NULL;
        g_iq[i].unity_stale = false;
    }
}

/* ================================================================
 * IMAGE KNOBS
 * ================================================================ */

int hal_isp_set_brightness(void *ctx, int val)
{
    return i6c_iq_set_scalar(ctx, IQ_BRIGHTNESS, val);
}

int hal_isp_set_contrast(void *ctx, int val)
{
    return i6c_iq_set_scalar(ctx, IQ_CONTRAST, val);
}

int hal_isp_set_saturation(void *ctx, int val)
{
    return i6c_iq_set_scalar(ctx, IQ_SATURATION, val);
}

int hal_isp_get_brightness(void *ctx, uint8_t *val)
{
    return i6c_iq_get_scalar(ctx, IQ_BRIGHTNESS, val);
}

int hal_isp_get_contrast(void *ctx, uint8_t *val)
{
    return i6c_iq_get_scalar(ctx, IQ_CONTRAST, val);
}

int hal_isp_get_saturation(void *ctx, uint8_t *val)
{
    return i6c_iq_get_scalar(ctx, IQ_SATURATION, val);
}

/*
 * Defog, which is two ops onto one module: the switch is the module's bEnable and
 * the level is its manual strength. Setting the level does not enable it, and
 * enabling it does not choose a level -- which is the vendor's split, not one
 * invented here.
 */
int hal_isp_set_defog(void *ctx, int enable)
{
    return i6c_iq_set_raw(ctx, IQ_DEFOG_EN, enable ? 1u : 0u);
}

int hal_isp_set_defog_strength(void *ctx, int val)
{
    return i6c_iq_set_scalar(ctx, IQ_DEFOG, val);
}

/*
 * The same level through the op rvd actually calls from its [image] block. The
 * argument is opaque in the vtable because Ingenic's own entry point takes a void
 * pointer, and what rvd passes through it is the address of a uint8_t -- so that
 * is what this reads, and it is the only caller.
 */
int hal_isp_set_defog_strength_adv(void *ctx, const void *defog_attr)
{
    if (!defog_attr)
        return RSS_ERR_INVAL;

    return i6c_iq_set_scalar(ctx, IQ_DEFOG, *(const uint8_t *)defog_attr);
}

int hal_isp_get_defog_strength(void *ctx, uint8_t *val)
{
    return i6c_iq_get_scalar(ctx, IQ_DEFOG, val);
}

int hal_isp_set_ae_comp(void *ctx, int val)
{
    return i6c_iq_set_scalar(ctx, AE_EVCOMP, val);
}

int hal_isp_get_ae_comp(void *ctx, int *val)
{
    uint8_t v;
    int ret;

    if (!val)
        return RSS_ERR_INVAL;

    ret = i6c_iq_get_scalar(ctx, AE_EVCOMP, &v);
    if (ret == RSS_OK)
        *val = v;

    return ret;
}

int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode)
{
    uint32_t raw;

    switch (mode) {
    case RSS_ANTIFLICKER_OFF:
        raw = I6C_FLICKER_DISABLE;
        break;
    case RSS_ANTIFLICKER_50HZ:
        raw = I6C_FLICKER_50HZ;
        break;
    case RSS_ANTIFLICKER_60HZ:
        raw = I6C_FLICKER_60HZ;
        break;
    default:
        HAL_LOG_WARN("isp: antiflicker mode %d out of range", (int)mode);
        return RSS_ERR_INVAL;
    }

    return i6c_iq_set_raw(ctx, AE_FLICKER, raw);
}

int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode)
{
    uint32_t raw;
    int ret;

    if (!mode)
        return RSS_ERR_INVAL;

    ret = i6c_iq_get_raw(ctx, AE_FLICKER, &raw);
    if (ret != RSS_OK)
        return ret;

    switch (raw) {
    case I6C_FLICKER_50HZ:
        *mode = RSS_ANTIFLICKER_50HZ;
        break;
    case I6C_FLICKER_60HZ:
        *mode = RSS_ANTIFLICKER_60HZ;
        break;
    default:
        /* DISABLE and AUTO both. Auto is a mode raptor cannot express, and
         * reporting it as one of the two frequencies would be a guess. */
        *mode = RSS_ANTIFLICKER_OFF;
        break;
    }

    return RSS_OK;
}

/*
 * Day and night.
 *
 * Colour or monochrome, and nothing else: there is no second IQ profile to switch
 * to on this backend, and the IR-cut filter and the illuminator are GPIOs that ric
 * drives itself. So night mode here is MI_ISP_IQ_SetColorToGray, which is what a
 * mono night mode is -- and which stops the AWB gains from tinting a scene lit
 * entirely by an infrared LED.
 */
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode)
{
    int ret = i6c_iq_set_raw(ctx, IQ_GRAY, mode == RSS_ISP_NIGHT ? 1u : 0u);

    if (ret == RSS_OK)
        HAL_LOG_INFO("isp: %s mode", mode == RSS_ISP_NIGHT ? "night (monochrome)" : "day (colour)");

    return ret;
}

int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode)
{
    uint32_t raw;
    int ret;

    if (!mode)
        return RSS_ERR_INVAL;

    ret = i6c_iq_get_raw(ctx, IQ_GRAY, &raw);
    if (ret == RSS_OK)
        *mode = raw ? RSS_ISP_NIGHT : RSS_ISP_DAY;

    return ret;
}

/* ================================================================
 * THE ISP CHANNEL PARAMETER BLOCK
 * ================================================================ */

/*
 * Orientation and the 3DNR level, which are fields of the ISP channel rather than
 * tuning modules -- so they are not queued: they take effect when written and are
 * unaffected by a tuning load.
 *
 * Read-modify-write, because MI_ISP_SetChnParam sets the block whole and the SDK
 * mutates what it is given (the 3DNR level is clamped to the per-chip maximum
 * internally), so a blind write would put this file's last idea of every other
 * field back over it.
 *
 * Orientation lives here rather than on the sensor deliberately. MI_SNR_SetOrien
 * only stores the value and sets a dirty flag, leaving the AE's frame notification
 * to write the register -- so it lands on the next notification if 3A is running
 * and sits pending if it is not, and MI_SNR_GetOrien cannot say which, because the
 * vendor driver answers from its static mode table rather than the live value.
 * i6c_snr_bringup leaves the sensor unflipped for that reason.
 *
 * The block also carries `rotate`, and it is not used from here: rotation is
 * hal_fs_set_rotation's, on the scaler, where it can change output geometry. Two
 * stages both rotating would compose.
 */
static int i6c_isp_chn_param(infinity6c_state_t *st, i6c_isp_para *para)
{
    int ret;

    if (!st->isp.get_chn_param || !st->isp.set_chn_param)
        return RSS_ERR_NOTSUP;
    if (!st->pipeline_up)
        return RSS_ERR_NOENT;

    memset(para, 0, sizeof(*para));
    ret = st->isp.get_chn_param(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, para);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_GetChnParam failed: %d", ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * Send the block with orientation and 3DNR as the state asks for them.
 *
 * Held rather than refused before there is a channel to write to, which is where
 * rvd applies all of this: it drives its [image] block during pipeline setup, and
 * on this backend the ISP channel is not created until the first framesource
 * channel is. i6c_isp_bringup fills the block from the same three fields, so a
 * request made early lands then instead of being lost.
 */
static int i6c_isp_apply_chn_param(infinity6c_state_t *st)
{
    i6c_isp_para para;
    int ret;

    if (!st->pipeline_up)
        return RSS_OK;

    ret = i6c_isp_chn_param(st, &para);
    if (ret != RSS_OK)
        return ret;

    if (para.mirror == (st->isp_mirror_req ? 1 : 0) && para.flip == (st->isp_flip_req ? 1 : 0) &&
        para.level3DNR == st->isp_nr3d_req)
        return RSS_OK;

    para.mirror = (char)(st->isp_mirror_req ? 1 : 0);
    para.flip = (char)(st->isp_flip_req ? 1 : 0);
    para.level3DNR = st->isp_nr3d_req;

    ret = st->isp.set_chn_param(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &para);
    if (ret) {
        HAL_LOG_WARN("isp: MI_ISP_SetChnParam(mirror=%d flip=%d 3dnr=%d) failed: %d", para.mirror,
                     para.flip, para.level3DNR, ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_DBG("isp: mirror=%d flip=%d 3dnr=%d", para.mirror, para.flip, para.level3DNR);
    return RSS_OK;
}

int hal_isp_set_hflip(void *ctx, int enable)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st = i6c_state(ctx);
    int prev;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    prev = st->isp_mirror_req;
    st->isp_mirror_req = enable ? 1 : 0;
    ret = i6c_isp_apply_chn_param(st);
    if (ret != RSS_OK)
        st->isp_mirror_req = prev;
    else
        c->hflip_state[0] = st->isp_mirror_req;

    return ret;
}

int hal_isp_set_vflip(void *ctx, int enable)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st = i6c_state(ctx);
    int prev;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    prev = st->isp_flip_req;
    st->isp_flip_req = enable ? 1 : 0;
    ret = i6c_isp_apply_chn_param(st);
    if (ret != RSS_OK)
        st->isp_flip_req = prev;
    else
        c->vflip_state[0] = st->isp_flip_req;

    return ret;
}

int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st = i6c_state(ctx);
    i6c_isp_para para;

    if (!st)
        return RSS_ERR_INVAL;

    /*
     * The hardware when there is any, the request otherwise. Asking is worth the
     * round trip rather than answering from the shadow throughout: the driver can
     * refuse an orientation -- its flip and rotate predicates are gated on the
     * 3DNR level -- and a shadow cannot tell that it did.
     */
    if (i6c_isp_chn_param(st, &para) == RSS_OK) {
        st->isp_mirror_req = para.mirror ? 1 : 0;
        st->isp_flip_req = para.flip ? 1 : 0;
    }

    c->hflip_state[0] = st->isp_mirror_req;
    c->vflip_state[0] = st->isp_flip_req;

    if (hflip)
        *hflip = c->hflip_state[0];
    if (vflip)
        *vflip = c->vflip_state[0];

    return RSS_OK;
}

/*
 * Temporal noise reduction, which MI calls 3DNR and raptor calls temper.
 *
 * The ISP channel's level, not MI_ISP_IQ_SetNr3d: that module's manual block is
 * per-band arrays with no place for a single scalar, while this is one field the
 * driver itself bounds at 7 and clamps to the per-chip maximum. raptor's 0..255
 * maps onto 0..7, so the knob has eight distinct positions rather than 256 -- which
 * is what the hardware has.
 *
 * There is no sinter counterpart. Spatial luma denoise is a separate module here
 * with the same per-band shape as sharpness, so isp_set_sinter_strength stays
 * absent rather than being aliased onto this one and quietly moving temper.
 */
#define I6C_ISP_NR3D_MAX 7

int hal_isp_set_temper_strength(void *ctx, int val)
{
    infinity6c_state_t *st = i6c_state(ctx);
    int prev;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (val < 0 || val > 255)
        return RSS_ERR_INVAL;

    prev = st->isp_nr3d_req;
    /* Rounded rather than truncated, so 255 reaches 7 and the midpoint lands
     * mid-range instead of one step low. */
    st->isp_nr3d_req = (val * I6C_ISP_NR3D_MAX + 127) / 255;

    ret = i6c_isp_apply_chn_param(st);
    if (ret != RSS_OK) {
        st->isp_nr3d_req = prev;
        return ret;
    }

    HAL_LOG_DBG("isp: temper = %d (3DNR level %d of %d)", val, st->isp_nr3d_req, I6C_ISP_NR3D_MAX);
    return RSS_OK;
}

int hal_isp_get_temper_strength(void *ctx, uint8_t *val)
{
    infinity6c_state_t *st = i6c_state(ctx);
    i6c_isp_para para;
    int level;

    if (!st || !val)
        return RSS_ERR_INVAL;

    /* The clamped level when the channel can be asked, the request otherwise: the
     * SDK lowers a level the chip cannot do, so what was asked for is not always
     * what is running. */
    if (i6c_isp_chn_param(st, &para) == RSS_OK && para.level3DNR >= 0)
        st->isp_nr3d_req = para.level3DNR;

    level = st->isp_nr3d_req;
    if (level < 0)
        level = 0;
    if (level > I6C_ISP_NR3D_MAX)
        level = I6C_ISP_NR3D_MAX;

    *val = (uint8_t)((level * 255 + I6C_ISP_NR3D_MAX / 2) / I6C_ISP_NR3D_MAX);
    return RSS_OK;
}

/* ================================================================
 * SENSOR RATE
 * ================================================================ */

/*
 * The sensor's own frame rate, which is neither a tuning module nor a channel
 * parameter but a sensor call.
 *
 * Applied live when the sensor is up and recorded for bring-up when it is not:
 * rvd sets the rate during pipeline setup, and on this backend the sensor is not
 * enabled until the first framesource channel is created. Without the record the
 * request would land on nothing and the sensor would run at whatever the mode
 * table chose.
 */
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den)
{
    infinity6c_state_t *st = i6c_state(ctx);
    unsigned int fps;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!fps_num)
        return RSS_ERR_INVAL;

    fps = fps_den ? (fps_num + fps_den / 2) / fps_den : fps_num;
    if (!fps)
        return RSS_ERR_INVAL;

    if (!st->snr.set_fps)
        return RSS_ERR_NOTSUP;

    /* Recorded either way, not only when it cannot be applied: a pipeline rebuilt
     * in this process picks its rate from here, and a rate applied live would
     * otherwise be forgotten by the rebuild. */
    st->snr_fps_req = fps;

    if (!st->pipeline_up) {
        HAL_LOG_DBG("isp: sensor rate %u fps held until the sensor is enabled", fps);
        return RSS_OK;
    }

    ret = st->snr.set_fps(I6C_DEV_ID(I6C_SNR_PAD), fps);
    if (ret) {
        HAL_LOG_WARN("isp: MI_SNR_SetFps(%u) failed: %d", fps, ret);
        return RSS_ERR_IO;
    }

    st->fps = fps;
    HAL_LOG_INFO("isp: sensor rate %u fps", fps);
    return RSS_OK;
}

/*
 * The rate in force. Answered from the sensor when it can be, since the driver
 * may have clamped the request to what the selected mode supports, and from the
 * cache before the sensor exists -- which is where rvd asks, and where a "cannot
 * report the rate" answer would send it to a 25 fps fallback.
 */
int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den)
{
    infinity6c_state_t *st = i6c_state(ctx);
    unsigned int fps = 0;

    if (!st || !fps_num || !fps_den)
        return RSS_ERR_INVAL;

    if (st->pipeline_up && st->snr.get_fps && st->snr.get_fps(I6C_DEV_ID(I6C_SNR_PAD), &fps) == 0 &&
        fps)
        st->fps = fps;
    else
        fps = st->fps ? st->fps : st->snr_fps_req;

    if (!fps)
        return RSS_ERR_BUSY;

    *fps_num = fps;
    *fps_den = 1;
    return RSS_OK;
}
