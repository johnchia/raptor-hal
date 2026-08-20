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
 * whole, so they are read-modify-written rather than assigned -- and it is only
 * writable while the channel is being created. A running channel refuses it
 * (-1610121208, measured), so these three are start-up settings on this part,
 * which is why they are held in the state for i6c_isp_bringup rather than
 * queued like the tuning values.
 *
 * Each is queried per (device, channel), and the device index carries the SoC id
 * in its high halfword, so I6C_DEV_ID composes it as elsewhere.
 *
 * THE TUNING BINARY OUTRANKS THE CONFIG
 *
 * A value written before the first frame does not survive. The IQ binary loads
 * on that frame and writes over the API store, so an early write succeeds, does
 * not take effect, and reports nothing. rvd applies its whole [image] block
 * during pipeline construction, which is well before any frame, so every one of
 * those calls would be lost.
 *
 * Hence the queue: a knob asked for before st->isp_knobs_live is recorded and
 * applied by i6c_isp_flush_knobs, which i6c_isp_note_frame calls once the 3A arm
 * and the load are done. Entries are not cleared by the flush, because a
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
 * the buffer is used. Sharpness is what sets it: its module carries sixteen
 * per-ISO copies of a 368-byte parameter block, so the whole payload is 6264
 * bytes and the library copies all of it in both directions.
 *
 * It is a stack buffer and it stays one. The alternative -- a small buffer for
 * the eight scalar rows and a large one only where it is needed -- means two
 * copies of the apply and the fetch, which is a worse thing to own than one and
 * a half pages of stack in a call that runs a handful of times per pipeline.
 */
#define I6C_IQ_PAYLOAD_MAX 6264

/*
 * The longest strength run a vector row can have, which is sharpness's six.
 * Only the run is held per row, not the payload: what is learned from the
 * tuning is a handful of gains, not the module.
 */
#define I6C_IQ_VECTOR_MAX 6

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
 * The 3A state at the first successful readback of each, logged once each. It is
 * how the colour path is diagnosed without a board in hand: the white-balance
 * gains say where AWB has settled, and whether they move when the illuminant
 * changes says whether it is adapting at all. Gains are 1024-per-unit; the log
 * prints the raw values so the unit can be checked against the scene.
 *
 * Two latches and not one, which is the whole point of the split. They shared a
 * single flag, set on the first AE success -- and AE answers before AWB has a
 * result to give, every time, because the AE loop converges first. So the flag
 * was always spent on a call where AWB had nothing, and the awb line could never
 * be printed at all. Measured on an SSC377QE: no isp/awb in a whole boot, while
 * the very same gains were being reported to ric all along.
 */
static void i6c_isp_log_ae(const i6c_cus_ae_info *ae)
{
    HAL_LOG_INFO("isp/ae: shutter=%uus sensorgain=%u ispgain=%u avgY=%u", ae->shutterUs,
                 ae->sensorGain, ae->ispGain, ae->preAvgY);
}

static void i6c_isp_log_awb(const i6c_cus_awb_info *awb)
{
    HAL_LOG_INFO("isp/awb: gains r=%u g=%u b=%u (1024=1x)", awb->rGain, awb->gGain, awb->bGain);
}

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    infinity6c_state_t *st = i6c_state(ctx);
    static bool ae_logged;
    static bool awb_logged;
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
        exposure->wb_ggain = (uint16_t)awb.gGain;
        exposure->wb_bgain = (uint16_t)awb.bGain;
    }

    if (!ae_logged) {
        ae_logged = true;
        i6c_isp_log_ae(&ae);
    }

    if (have_awb && !awb_logged) {
        awb_logged = true;
        i6c_isp_log_awb(&awb);
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
    IQ_FLAT,    /* the value is the whole payload, at offset 0 */
    IQ_BOOL,    /* bEnable at offset 0 is itself the value */
    IQ_AUTOMAN, /* bEnable, enOpType, stAuto[16], stManual at manual_off */
    IQ_VECTOR   /* as IQ_AUTOMAN, but the value is a run of `count` fields */
} i6c_iq_shape_t;

/* The two shapes that carry enOpType, so that neutral can hand the module back
 * to the tuning rather than pinning it to the tuning's own numbers. */
#define I6C_IQ_HAS_AUTO(p) ((p)->shape == IQ_AUTOMAN || (p)->shape == IQ_VECTOR)

typedef struct {
    const char *name; /* for diagnostics only */
    const char *get_sym;
    const char *set_sym;
    uint16_t payload;    /* the wrapper's own declared payload length */
    uint16_t manual_off; /* where the value lives (0 for FLAT and BOOL) */
    uint8_t width;       /* 1, 2 or 4 bytes */
    uint8_t count;       /* fields in the run: 1 for everything but IQ_VECTOR */
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

    /*
     * A vector row's baseline: the tuning's own value for each field of the
     * run, which is what raptor's 128 means for that field. Held per row rather
     * than re-read on each write, and that is the whole point -- scaling from
     * the field's current value would compound, so a knob nudged from 128 to
     * 160 four times would end up somewhere quite different from one set to 160
     * once, with no way back short of a tuning reload.
     */
    bool base_valid;
    uint16_t base[I6C_IQ_VECTOR_MAX];

    /*
     * Which field of the run the getter reports from; chosen with the baseline
     * rather than fixed at 0. See i6c_iq_pick_report_field.
     */
    uint8_t report;
} i6c_iq_param_t;

enum {
    IQ_BRIGHTNESS,
    IQ_CONTRAST,
    IQ_SATURATION,
    IQ_SHARPNESS,
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
 * The two vector rows have no constant to put here at all, which is why both
 * learn their neutral from the tuning; see i6c_iq_apply_vector. What is in
 * mi_unity for them is the fallback for a board with no tuning file, and it is
 * the midpoint of the field because nothing better is knowable without one.
 *
 * Single-instance, like the Infinity6E table: the resolved symbols and the
 * pending queue belong to one HAL context, which is what rvd creates. Cleared by
 * i6c_isp_forget_knobs at teardown.
 */
static i6c_iq_param_t g_iq[IQ_PARAM_COUNT] = {
    [IQ_BRIGHTNESS] = {"brightness", "MI_ISP_IQ_GetBrightness", "MI_ISP_IQ_SetBrightness",
                       I6C_ISP_IQ_BRIGHTNESS_PAYLOAD, I6C_ISP_IQ_BRIGHTNESS_MANUAL, 4, 1,
                       IQ_AUTOMAN, 100, 50, 0, false, NULL, NULL, 0, false, false, false},
    [IQ_CONTRAST] = {"contrast", "MI_ISP_IQ_GetContrast", "MI_ISP_IQ_SetContrast",
                     I6C_ISP_IQ_CONTRAST_PAYLOAD, I6C_ISP_IQ_CONTRAST_MANUAL, 4, 1, IQ_AUTOMAN, 100,
                     50, 0, false, NULL, NULL, 0, false, false, false},
    [IQ_SATURATION] = {"saturation", "MI_ISP_IQ_GetSaturation", "MI_ISP_IQ_SetSaturation",
                       I6C_ISP_IQ_SATURATION_PAYLOAD, I6C_ISP_IQ_SATURATION_MANUAL, 1, 1,
                       IQ_AUTOMAN, 127, 32, 0, false, NULL, NULL, 0, false, false, false},
    /*
     * Sharpness, and the reason this file grew a vector shape.
     *
     * MI has no sharpness level on this generation. What it has is six gains --
     * three frequency bands for the undirectional sharpener and three for the
     * directional one, each 0..127 -- and the tuning binary sets all six to a
     * shape that is the tuner's judgement about this sensor. So the knob scales
     * the run and leaves the shape alone, which is the only reading of "more
     * sharpness" that does not throw the tuning away.
     *
     * 63 is the no-tuning fallback and is a midpoint, not a measurement.
     */
    [IQ_SHARPNESS] = {"sharpness", "MI_ISP_IQ_GetSharpness", "MI_ISP_IQ_SetSharpness",
                      I6C_ISP_IQ_SHARPNESS_PAYLOAD,
                      I6C_ISP_IQ_SHARPNESS_MANUAL + I6C_ISP_IQ_SHARPNESS_STRENGTH, 1,
                      I6C_ISP_IQ_SHARPNESS_STRENGTH_NUM, IQ_VECTOR,
                      I6C_ISP_IQ_SHARPNESS_STRENGTH_MAX, 63, 0, true, NULL, NULL, 0, false, false,
                      false},
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
                  I6C_ISP_IQ_DEFOG_MANUAL, 1, 1, IQ_AUTOMAN, 255, 128, 0, false, NULL, NULL, 0,
                  false, false, false},
    [IQ_DEFOG_EN] = {"defog enable", "MI_ISP_IQ_GetDefog", "MI_ISP_IQ_SetDefog",
                     I6C_ISP_IQ_DEFOG_PAYLOAD, I6C_ISP_ENABLE_OFF, 4, 1, IQ_BOOL, 1, 0, 0, false,
                     NULL, NULL, 0, false, false, false},
    [IQ_GRAY] = {"gray", "MI_ISP_IQ_GetColorToGray", "MI_ISP_IQ_SetColorToGray",
                 I6C_ISP_IQ_GRAY_PAYLOAD, 0, 4, 1, IQ_BOOL, 1, 0, 0, false, NULL, NULL, 0, false,
                 false, false},
    /*
     * The only row whose neutral has to be learned and the only one whose MI
     * field is signed. It is IQ_FLAT, so there is no auto mode to hand it back
     * to; what it does is shift the AE's target luma, so a neutral guessed wrong
     * shifts every default image.
     */
    [AE_EVCOMP] = {"ae_comp", "MI_ISP_AE_GetEvComp", "MI_ISP_AE_SetEvComp",
                   I6C_ISP_AE_EVCOMP_PAYLOAD, 0, 4, 1, IQ_FLAT, I6C_AE_EV_SPAN, 0, -I6C_AE_EV_SPAN,
                   true, NULL, NULL, 0, false, false, false},
    [AE_FLICKER] = {"antiflicker", "MI_ISP_AE_GetFlicker", "MI_ISP_AE_SetFlicker",
                    I6C_ISP_AE_FLICKER_PAYLOAD, 0, 4, 1, IQ_FLAT, 3, 0, 0, false, NULL, NULL, 0,
                    false, false, false},
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

    /*
     * Both bounds, because a row is wrong in two different ways and only one of
     * them is a buffer overrun. A run longer than base[] would overrun the
     * baseline on the way in; a run that reaches past the payload would have the
     * library copy from past the staging buffer on the way out. Checked here
     * rather than asserted, because the test build defines _Static_assert away.
     */
    if (p->count > I6C_IQ_VECTOR_MAX ||
        (uint32_t)p->manual_off + (uint32_t)p->count * p->width > p->payload) {
        HAL_LOG_ERR("isp: %s run of %u x %u at %u does not fit its %u-byte payload", p->name,
                    p->count, p->width, p->manual_off, p->payload);
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
/*
 * Which field of a run the getter should read back from.
 *
 * Every field was scaled from the same 0..255, so in principle any of them
 * recovers it. In practice a field whose baseline sits on one of MI's bounds
 * cannot: half the knob maps onto the one value there, and unscaling it reports
 * neutral for everything on that side.
 *
 * That is not hypothetical. The IMX335 tuning on this board leaves the first
 * sharpening gain -- low-frequency undirectional -- at 0 of 127, so reporting
 * from field 0 answered 128 for a knob set to 64, and again for one set to 0.
 * The write was right and every field moved; only the reading was blind.
 *
 * So pick the best-conditioned field: the one furthest from both bounds, which
 * is the one with the most resolution to report with in either direction. Ties
 * go to the earlier field, and a run that is entirely on a bound has no good
 * answer to give -- but a tuning that turned the module off has genuinely made
 * "off" and "as the tuning left it" the same picture, so reporting neutral for
 * both is not wrong there.
 */
static void i6c_iq_pick_report_field(i6c_iq_param_t *p)
{
    int32_t best_room = -1;
    unsigned int i;

    p->report = 0;

    for (i = 0; i < p->count; i++) {
        int32_t base = p->base[i];
        int32_t below = base - p->mi_floor;
        int32_t above = p->mi_max - base;
        int32_t room = below < above ? below : above;

        if (room > best_room) {
            best_room = room;
            p->report = (uint8_t)i;
        }
    }
}

static void i6c_iq_learn_unity(i6c_iq_param_t *p, const uint8_t *buf)
{
    int32_t base;
    unsigned int i;

    if (!p->unity_from_tuning || !p->unity_stale)
        return;

    /*
     * A vector row learns the whole run at once, and all or nothing: one field
     * out of range means the offset or the width is wrong for the module, not
     * that one gain is unusual, so adopting the rest would scale five good
     * fields about a misread sixth and look almost right.
     *
     * All-zero is not out of range and is not rejected. A tuning is entitled to
     * turn a module off, and if it has, then raptor's neutral is off too and
     * everything above it scales up from nothing -- which is the correct
     * reading of a knob whose baseline is zero, not a failure to find one.
     */
    if (p->shape == IQ_VECTOR) {
        for (i = 0; i < p->count; i++) {
            base = (int32_t)i6c_iq_read(buf, p->manual_off + i * p->width, p->width);
            if (base > p->mi_max || base < p->mi_floor) {
                HAL_LOG_WARN("isp: %s field %u reads MI %d, outside its %d..%d range -- not "
                             "adopting the tuning's run as the neutral, keeping %d throughout",
                             p->name, i, base, p->mi_floor, p->mi_max, p->mi_unity);
                p->base_valid = false;
                p->report = 0;
                p->unity_stale = false;
                return;
            }
            p->base[i] = (uint16_t)base;
        }

        p->base_valid = true;
        p->unity_stale = false;
        i6c_iq_pick_report_field(p);
        HAL_LOG_INFO("isp: %s baseline from the tuning is MI %u..%u over %u fields in %d..%d, "
                     "reporting from field %u (MI %u)",
                     p->name, p->base[0], p->base[p->count - 1], p->count, p->mi_floor, p->mi_max,
                     p->report, p->base[p->report]);
        return;
    }

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
 * What raptor's 128 means for one field of a row.
 *
 * mi_unity for everything that is not a vector, and for a vector whose run
 * could not be learned -- which is a board with no tuning file, or a header
 * drop that moved the module. The midpoint is a poor neutral, but it is a knob
 * that still moves in both directions, which is better than one pinned to a
 * field that was misread.
 */
static int32_t i6c_iq_baseline(const i6c_iq_param_t *p, unsigned int i)
{
    if (p->shape != IQ_VECTOR || !p->base_valid)
        return p->mi_unity;

    return p->base[i];
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
    if (val == I6C_ISP_NEUTRAL)
        return unity;

    /*
     * A baseline sitting on the ceiling needs no special case: the upper branch
     * below spans max - unity, which is then zero, so every value above neutral
     * lands on the baseline and the knob simply has no headroom. What it must
     * not do is give up on the lower half as well, which a `unity >= max` bail
     * here used to -- and a tuning that asks for full-strength denoise is
     * exactly the case, since MI's own unity for that field is its maximum.
     */
    if (val < I6C_ISP_NEUTRAL)
        return floor + (int32_t)(((int64_t)val * (unity - floor)) / I6C_ISP_NEUTRAL);

    return unity +
           (int32_t)(((int64_t)(val - I6C_ISP_NEUTRAL) * (max - unity)) / (255 - I6C_ISP_NEUTRAL));
}

/* Inverse of i6c_iq_scale, for the getters. */
static uint8_t i6c_iq_unscale(int32_t mi, int32_t unity, int32_t floor, int32_t max)
{
    if (max == floor)
        return 255;
    /*
     * Ahead of both bound tests, which would otherwise swallow it: a learned
     * baseline can sit on either end, and there MI's own extreme is neutral
     * rather than the extreme. Sitting on the floor is the ae_comp case and
     * makes 0 mean neutral; sitting on the ceiling is the full-strength denoise
     * case and makes the maximum mean it.
     *
     * Which does leave the reading ambiguous at whichever end the baseline is
     * on -- MI cannot distinguish "asked for neutral" from "asked for the end"
     * when they are the same number -- and neutral is the better answer, since
     * it is what the knob was left at rather than what it was driven to.
     */
    if (mi == unity)
        return I6C_ISP_NEUTRAL;
    if (mi >= max)
        return 255;
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
        /*
         * Leaving auto costs the tuning file's per-gain curve for this
         * module: MI interpolates stAuto[16] across gain, and stManual is
         * one value for all of it. Real rather than hypothetical -- every
         * one of the six shipped Infinity6C tunings varies saturation and
         * sharpness across gain. Said once per departure rather than per
         * write, so re-applying the same value does not repeat it.
         */
        if (i6c_iq_read(buf, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO)
            HAL_LOG_INFO("isp: %s goes manual, so the tuning's per-gain curve for it stops "
                         "being used; set %s back to %d to restore it",
                         p->name, p->name, I6C_ISP_NEUTRAL);
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
 * Apply one of raptor's 0..255 scalars to a module that has no scalar in it.
 *
 * Sharpness and spatial denoise are per-band tables here: six sharpening gains
 * across three frequencies and two sharpener kinds, two denoise blend weights.
 * There is no field that means "how much" on its own, and picking one band and
 * calling it sharpness is what this file spent a release declining to do.
 *
 * What there is, in each, is a contiguous run of fields that all mean strength,
 * and a tuning binary that has already chosen a *shape* for them -- how much
 * high frequency relative to low, how much directional relative to
 * undirectional. That shape is the tuner's judgement about this sensor and this
 * lens, and it is not something a single knob has any business rewriting. So
 * the knob scales the run, each field about its own baseline, and the shape
 * survives: raptor's 128 is the tuning untouched, 0 is the module off, 255 is
 * every field at MI's ceiling, and everything between moves them together.
 *
 * The shape does flatten as the knob approaches 255, because the fields run out
 * of headroom at different distances from their baselines and the ceiling is
 * shared. That is inherent in "maximum" meaning maximum, and it is the same
 * thing i6c_iq_scale already does to every scalar row at the top of its range.
 *
 * Scaled from the learned baseline and never from what is in the field now --
 * see base_valid, and i6c_iq_flush_knobs for when a baseline goes stale.
 */
static int i6c_iq_apply_vector(infinity6c_state_t *st, int idx, int val)
{
    i6c_iq_param_t *p = &g_iq[idx];
    uint8_t buf[I6C_IQ_PAYLOAD_MAX];
    unsigned int i;
    int ret;

    /* Which is also what learns the baseline, on the first fetch after a load. */
    ret = i6c_iq_fetch(st, idx, buf);
    if (ret != RSS_OK)
        return ret;

    if (val == I6C_ISP_NEUTRAL) {
        i6c_iq_write(buf, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_AUTO);
        HAL_LOG_DBG("isp: %s left to the tuning file (auto)", p->name);
        return i6c_iq_store(st, idx, buf);
    }

    /* As in i6c_iq_apply_scalar: the band shape inside the manual block
     * survives a vector write, but the per-gain curve does not. */
    if (i6c_iq_read(buf, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO)
        HAL_LOG_INFO("isp: %s goes manual, so the tuning's per-gain curve for it stops being "
                     "used; set %s back to %d to restore it",
                     p->name, p->name, I6C_ISP_NEUTRAL);
    i6c_iq_write(buf, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_MANUAL);

    for (i = 0; i < p->count; i++) {
        int32_t mi_val = i6c_iq_scale(val, i6c_iq_baseline(p, i), p->mi_floor, p->mi_max);

        i6c_iq_write(buf, p->manual_off + i * p->width, p->width, (uint32_t)mi_val);
    }

    ret = i6c_iq_store(st, idx, buf);
    if (ret == RSS_OK)
        HAL_LOG_DBG("isp: %s = %d (%u fields, MI %d..%d in %d..%d)", p->name, val, p->count,
                    i6c_iq_scale(val, i6c_iq_baseline(p, 0), p->mi_floor, p->mi_max),
                    i6c_iq_scale(val, i6c_iq_baseline(p, p->count - 1), p->mi_floor, p->mi_max),
                    p->mi_floor, p->mi_max);

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

/* The one place a row's shape chooses its apply, so the flush and the direct
 * path cannot disagree about which one a row gets. */
static int i6c_iq_apply(infinity6c_state_t *st, int idx, int val, bool raw)
{
    if (raw)
        return i6c_iq_apply_raw(st, idx, (uint32_t)val);
    if (g_iq[idx].shape == IQ_VECTOR)
        return i6c_iq_apply_vector(st, idx, val);

    return i6c_iq_apply_scalar(st, idx, val);
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

    return i6c_iq_apply(st, idx, val, raw);
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
    if (I6C_IQ_HAS_AUTO(p) && i6c_iq_read(buf, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO) {
        *val = I6C_ISP_NEUTRAL;
        return RSS_OK;
    }

    /*
     * One field of the run answers for a vector row, and which one was chosen
     * along with the baseline -- see i6c_iq_pick_report_field. report is 0 for
     * every other shape, which is what lets this be one expression for all of
     * them.
     */
    *val = i6c_iq_unscale((int32_t)i6c_iq_read(buf, p->manual_off + p->report * p->width, p->width),
                          i6c_iq_baseline(p, p->report), p->mi_floor, p->mi_max);
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

/*
 * The rate the camera is configured to hold, as distinct from the rate it is
 * managing right now.
 *
 * Deliberately not the rate the sensor reports. MI_SNR_GetFps answers from the
 * frame length currently programmed, and the frame length is exactly what the AE
 * stretches to buy a longer exposure -- so the sensor's answer moves with the
 * light, and only these two fields say what was asked for.
 *
 * Both are needed. st->fps is the rate that was programmed and snr_fps_req the
 * rate that was asked for; rvd sets the rate while it is still building the
 * pipeline, which on this backend runs before the sensor is enabled, so early on
 * only the latter has anything in it.
 *
 * 0 when nothing has asked for a rate yet.
 *
 * THERE IS NO SHUTTER CEILING HERE ANY MORE
 *
 * This used to feed a cap that held maxShutterUs to one frame period, rewritten
 * after every tuning load. It is gone: on this camera the exposure is worth more
 * than the frame rate, because the light it buys is what keeps the AE off the
 * gain, and gain is where the noise comes from. Denoise cannot be turned up here
 * to compensate -- see the sinter note further down -- so the lever that is left
 * is the exposure itself.
 *
 * What it costs, measured on an SSC377QE + IMX335 before the removal: the
 * tuning's own ceiling is 100 ms, and 90 ms of shutter stretches VMAX 4950 ->
 * 11139, taking 25 fps down to about 11. So a dark scene now runs slow and
 * blurred instead of fast and noisy. That is the trade, chosen deliberately, and
 * it is the thing to undo first if the frame rate turns out to matter more.
 *
 * Anyone restoring it should read the git history rather than start over: the
 * cap had to derive its rate from these two fields and not from MI_SNR_GetFps,
 * because a ceiling decided from a rate the AE had already depressed declines to
 * cap and then latches -- measured at 9 fps reported against a 111 ms frame,
 * with nothing to recover it but the scene brightening on its own.
 */
static unsigned int i6c_isp_nominal_fps(const infinity6c_state_t *st)
{
    return st->fps ? st->fps : st->snr_fps_req;
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

    /*
     * Every baseline read out of the tuning is stale now the tuning has just
     * loaded, and the fetches below are where they are re-read.
     *
     * A vector row's base[] is not separately invalidated here, and does not
     * need to be: every read of it goes through i6c_iq_baseline, every caller
     * of that has just been through i6c_iq_fetch, and a fetch with unity_stale
     * set relearns the run before it returns. So the stale values cannot be
     * reached -- one flag, not two that could disagree.
     */
    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].unity_stale = g_iq[i].unity_from_tuning;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        i6c_iq_param_t *p = &g_iq[i];

        if (!p->has_pending)
            continue;

        i6c_iq_apply(st, (int)i, p->pending, p->pending_is_raw);
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

/*
 * NO SATURATION HERE, DELIBERATELY
 *
 * Saturation is an auto/manual module and MI's enOpType has exactly two
 * states, so any value but the neutral 128 replaces the tuner's per-gain curve
 * with one constant for the whole gain range. That is not an adjustment, it is
 * a discard, and here it discards something real: ALLSTR varies across the
 * sixteen gain entries in all six shipped tunings -- 28..40 on gc4653, 28..36
 * on imx335, 30..45 on imx415 -- which is the tuner pulling colour back as
 * gain climbs so that amplified chroma noise does not come with it. Pinning
 * one value spends exactly that.
 *
 * Sharpness stays, and the difference is the mechanism rather than the
 * principle: its op scales a run of band gains about the tuning's own values,
 * so the shape the tuner chose survives inside the manual block. Brightness
 * and defog stay because their curves are flat in every shipped bin, so
 * leaving auto costs nothing measurable. Saturation is the one row on this
 * family where the knob and the tuning cannot both have it.
 *
 * The function and its table row stay defined -- they drive the same
 * read-modify-write path the remaining knobs use, and tests/t_isp_i6c.c
 * exercises the scaling through saturation's own numbers. What is withdrawn is
 * the vtable entry, which is the whole mechanism: rvd reports the key
 * unsettable, rcd marks it unavailable, and a stale non-neutral saturation in
 * an existing config simply stops being applied and the curve comes back.
 *
 * Infinity6E withdrew this one along with brightness, contrast and sharpness,
 * for the same reason applied to a family where all four curves carry
 * something. This is the saturation half of that, not the whole of it.
 */
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
 * Sharpness: a per-band table driven as a run; see i6c_iq_apply_vector for what
 * a single number does to one.
 *
 * NO SINTER HERE, DELIBERATELY
 *
 * Spatial luma denoise -- raptor's "sinter", MI's NrLumaAdv -- was published
 * here as a second vector row and is not any more, because the knob could not
 * be given a range that means anything on this family.
 *
 * The field it reaches is u16Strength[2], the module's blend weight: how much of
 * the filtered luma replaces the original, per level, out of 256 for a full
 * swap. That is the only part of a 208-byte parameter block the row could
 * address, and it is not where the denoising strength lives -- the filter's own
 * radii and thresholds are, and nothing here moves them.
 *
 * What settles it is what the shipped tunings ask for. Read out of the per-sensor
 * bins in /etc/sensors, every one of the six has the module enabled and in AUTO,
 * and imx335 -- the sensor this was chased on -- pins its whole 16-entry
 * gain-indexed table flat at 255 of 256. So the tuner already asked for
 * essentially the maximum blend at every gain, and raptor's knob had exactly two
 * positions that were not worse than doing nothing: 128, which restores AUTO,
 * and 255, which writes 256 and draws level with it. Everything between was a
 * quiet downgrade -- 192 lands on 129, half the blend the tuning wanted -- and
 * everything below 128 was off.
 *
 * A knob whose whole usable range is its own default is not a knob. The module
 * keeps running on the tuning's terms, which is what it was doing at 128
 * anyway, and rvd gets RSS_ERR_NOTSUP for a set. The vendor ABI for it stays in
 * sigmastar-headers with its assertions in tests/abi_iq_i6c.c, since what is
 * missing here is a useful range and not the knowledge of where the module is.
 *
 * Temporal denoise (temper, MI's 3DNR) is untouched by this: it is a channel
 * level with eight real positions, and it works.
 */
int hal_isp_set_sharpness(void *ctx, int val)
{
    return i6c_iq_set_scalar(ctx, IQ_SHARPNESS, val);
}

int hal_isp_get_sharpness(void *ctx, uint8_t *val)
{
    return i6c_iq_get_scalar(ctx, IQ_SHARPNESS, val);
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
 *
 * Which turns out to be the only way any of it lands: a running channel refuses
 * the write. That makes the early path the working one and a live `raptorctl rvd
 * set-hflip` the one that cannot be honoured -- so the failure explains itself
 * rather than returning a bare error, and the request is rolled back rather than
 * held for a later rebuild, since a command that reports failure and then quietly
 * takes effect minutes later is worse than one that simply fails.
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
        /*
         * A running channel refuses this, measured: with the pipeline up every
         * call returns -1610121208, and bring-up accepts the same values. So
         * orientation and the 3DNR level are creation-time settings on this part
         * rather than runtime ones, and the caller is told where they do work
         * instead of being left with a vendor error code to look up. Said once,
         * because rvd re-applies its whole [image] block on demand.
         */
        static bool explained;

        HAL_LOG_WARN("isp: MI_ISP_SetChnParam(mirror=%d flip=%d 3dnr=%d) failed: %d", para.mirror,
                     para.flip, para.level3DNR, ret);
        if (!explained) {
            explained = true;
            HAL_LOG_WARN("isp: this part takes orientation and 3DNR only while the ISP channel is "
                         "being created -- set hflip, vflip and temper in the config and restart "
                         "the stream. The SDK refuses a live change; it is not being dropped here");
        }
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
 * Neutral is level 1, not the midpoint. It has to be the level the pipeline comes
 * up on -- the one hal_init seeds -- because rvd applies temper on every start
 * whether or not the config names the key. Same unity as Infinity6E's, and the
 * same reasoning as saturation's 32-of-127: raptor's neutral means "nobody asked",
 * which can only be whatever the platform was already doing.
 *
 * The old map sent neutral to 4 and got away with it only by failing: rvd's
 * temper arrives after the channel is up, the running channel refuses the
 * write, and the setter rolls back to the seed. So a board measured at level 1
 * while reporting 36, and the mapping and the hardware now agree instead.
 *
 * Two consequences, both measured on an SSC377QE rather than reasoned about.
 * A unity of 1 over a floor of 0 leaves exactly one level below neutral, so
 * every raptor value under 128 asks for 3DNR off -- there is no graded lower
 * half to be had, because the hardware has eight positions and the default is
 * the second of them. And a config carrying a temper written by an older build
 * reads differently now: that build encoded level 1 as 36, so a stored 36 used
 * to mean the default and now means off. Configs that round-tripped through
 * get-isp are the ones to look at.
 *
 * Sinter is emphatically not this. Spatial luma denoise is its own module,
 * reached through the tuning API as a vector row, and aliasing the two would
 * have isp_set_sinter_strength quietly move temper -- which is the sort of
 * substitution that survives a bench check, because both do reduce noise.
 */
#define I6C_ISP_NR3D_MAX 7
#define I6C_ISP_NR3D_UNITY 1

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
    st->isp_nr3d_req = i6c_iq_scale(val, I6C_ISP_NR3D_UNITY, 0, I6C_ISP_NR3D_MAX);

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

    *val = i6c_iq_unscale(level, I6C_ISP_NR3D_UNITY, 0, I6C_ISP_NR3D_MAX);
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

    /*
     * The sensor's own answer first, since the driver may clamp a request to
     * what the selected mode supports and this backend does not check the mode
     * itself. What it must not do is cache that answer: it moves with the AE
     * (see i6c_isp_nominal_fps), while st->fps is what the pipeline binds its
     * rates to, so letting a dark frame overwrite it would carry a stretched
     * rate into the next bind_ext.
     */
    if (!(st->pipeline_up && st->snr.get_fps &&
          st->snr.get_fps(I6C_DEV_ID(I6C_SNR_PAD), &fps) == 0 && fps))
        fps = i6c_isp_nominal_fps(st);

    if (!fps)
        return RSS_ERR_BUSY;

    *fps_num = fps;
    *fps_den = 1;
    return RSS_OK;
}
