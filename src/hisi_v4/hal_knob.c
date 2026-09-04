/*
 * hisi_v4/hal_knob.c -- the [image] knobs, and the exposure readback
 *
 * Five knobs over three ISP attributes, each a get-modify-set on one of
 * them. The tuning loader (hal_isp.c) writes two of the three and never
 * the CSC, which is why the neutral differs in kind between them:
 *
 *   brightness, contrast,  ISP_CSC_ATTR_S u8Luma / u8Contr / u8Satu, each
 *   saturation             0..100 with 50 as unity. The CSC is the last
 *                          stage before YUV and the vendor's own
 *                          image-adjust surface. The tuning file has no
 *                          section for it, so the driver's 50 is what "as
 *                          the tuning left it" means for all three.
 *
 *                          Saturation is the one knob here that is not the
 *                          only writer of its quantity: the ISP's own
 *                          saturation, ISP_SATURATION_ATTR_S in
 *                          lib_hiawb.so, is a per-ISO table set by the
 *                          tuning's [static_saturation] or, where the file
 *                          has none, by the sensor library's calibrated
 *                          per-gain ladder. That runs on the Bayer side
 *                          and stays exactly as it was; this is a flat
 *                          adjustment of the YUV the ISP hands out, which
 *                          is why unity here means "the tuning's answer,
 *                          whatever it currently is" and not a number.
 *   ae_comp                ISP_EXPOSURE_ATTR_S stAuto.u8Compensation,
 *                          0..255: the setpoint the AE converges its target
 *                          luma on, read afresh by the loop every frame and
 *                          moved by nothing else. It is a constant, not a
 *                          curve -- no engine here varies it with ISO, and
 *                          the driver does not vary it either.
 *
 *                          The baseline is learned rather than named, but it
 *                          is not the tuning file's: iq_sect_static_ae maps
 *                          ten [static_ae] keys and Compensation is not among
 *                          them, so nothing in a load writes this field and
 *                          the value at the first read after ISP init is the
 *                          AE library's own -- 56 on every sensor shipped
 *                          here, which is also the caps' fallback for before
 *                          the ISP runs. Learned anyway, because 56 is
 *                          lib_hiae.so's number and not this port's to
 *                          promise, and re-learned at each load for the same
 *                          reason.
 *
 *                          So `auto` restores that constant and stops the
 *                          reapply below re-asserting the knob; it does not
 *                          hand the field to a curve, there being none. What
 *                          it buys over writing the same number is that rcd
 *                          stores the word rather than 56, so a camera whose
 *                          AE library defaults elsewhere follows it instead
 *                          of this one's. caps.has_auto is true on that
 *                          weaker reading -- see the note in
 *                          hal_isp_get_knob_caps.
 *   drc_strength           ISP_DRC_ATTR_S strength, 0..1023, pinned in
 *                          manual mode. The [dynamic_linear_drc] engine
 *                          (hal_dyn.c) writes the same field by ISO, so a pin
 *                          holds the engine's DRC column for as long as it
 *                          stands and `auto` hands it back -- and the caps
 *                          offer `auto` only where that engine has a curve
 *                          to hand it back to.
 *
 * Units are the hardware's own, as [image]'s comment in raptor.conf
 * promises, and isp_get_knob_caps says what they are.
 *
 * The orientation pair, hflip and vflip, is at the end of the file: not an
 * ISP attribute at all but the VPSS channels' mirror and flip bits.
 *
 * WHEN THE WRITE HAPPENS. rvd applies [image] right after hal_init, before
 * the first frame; the tuning loads on the first frame and rewrites the
 * exposure and DRC attributes from the file, which would silently undo
 * ae_comp and drc_strength. So every set is remembered, written at once when
 * the 3A thread runs, and written again by hisi_knob_reapply at the end of
 * each load -- after the static modules and the engines have laid down the
 * baseline the knob adjusts from. Before that load, hisi_knob_before_load
 * lifts a pinned knob back to its baseline so the file lands on what the
 * tuner meant rather than on the knob.
 *
 * THE READBACK is HI_MPI_ISP_QueryExposureInfo, the same query the engines'
 * ISO tick uses: exposure time in microseconds, the sensor's analogue and
 * digital gains and the ISP's digital gain multiplied into one
 * 1024-per-unit figure (the console divides by 1024), and the AE's average
 * luma. That is what the OSD's %total_gain% and %ae_luma%, the console's
 * sidebar and ric's day/night decision were all waiting on.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <stdlib.h>
#include <string.h>

#define KNOB_CSC_UNITY 50
#define KNOB_CSC_MAX 100
#define KNOB_AE_MAX 255
#define KNOB_DRC_MAX 1023

static bool knob_live(const hisi_state_t *st)
{
    return __atomic_load_n(&st->isp_thread_running, __ATOMIC_ACQUIRE) != 0;
}

/* ---------------- CSC: brightness, contrast and saturation ---------------- */

/*
 * Three of the CSC's four adjust fields. They share one attribute, so a
 * write of any is a get-modify-set of all, and they share these helpers
 * rather than repeating it three times. The fourth field is u8Hue, on the
 * same scale and one row of this enum from being a knob as well; it is not
 * one because nothing has asked for it, not because it is hard.
 */
enum { CSC_LUMA, CSC_CONTR, CSC_SATU, CSC_FIELDS };

static const char *const csc_names[CSC_FIELDS] = {"brightness", "contrast", "saturation"};

static unsigned char *csc_field(v4_isp_csc_attr *a, int f)
{
    return f == CSC_LUMA ? &a->luma : f == CSC_CONTR ? &a->contr : &a->satu;
}

static hisi_knob_slot_t *csc_slot(hisi_state_t *st, int f)
{
    return f == CSC_LUMA    ? &st->knob.brightness
           : f == CSC_CONTR ? &st->knob.contrast
                            : &st->knob.saturation;
}

static int knob_csc_get(hisi_state_t *st, v4_isp_csc_attr *a)
{
    int ret;

    hisi_isp_tune_resolve(st);
    if (!st->tune.get_csc || !st->tune.set_csc)
        return RSS_ERR_NOTSUP;
    ret = st->tune.get_csc(HISI_VI_PIPE, a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_GetCSCAttr failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

static int knob_csc_write(hisi_state_t *st)
{
    v4_isp_csc_attr a;
    bool any = false;
    int f, ret;

    for (f = 0; f < CSC_FIELDS; f++)
        any |= csc_slot(st, f)->asked;
    if (!any)
        return RSS_OK;
    ret = knob_csc_get(st, &a);
    if (ret)
        return ret;
    for (f = 0; f < CSC_FIELDS; f++)
        if (csc_slot(st, f)->asked)
            *csc_field(&a, f) = (unsigned char)csc_slot(st, f)->val;
    ret = st->tune.set_csc(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_SetCSCAttr(luma %u, contrast %u, saturation %u) failed: 0x%x",
                    a.luma, a.contr, a.satu, ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

static int knob_csc_set(hisi_state_t *st, int f, int val)
{
    if (val == RSS_ISP_AUTO)
        val = KNOB_CSC_UNITY; /* no auto mode to hand back to: unity is the tuning's */
    if (val < 0 || val > KNOB_CSC_MAX) {
        HAL_LOG_WARN("%s: %d is outside the CSC's 0..%d", csc_names[f], val, KNOB_CSC_MAX);
        return RSS_ERR_INVAL;
    }
    csc_slot(st, f)->asked = true;
    csc_slot(st, f)->val = val;
    if (!knob_live(st)) {
        HAL_LOG_DBG("%s: %d noted for when the ISP runs", csc_names[f], val);
        return RSS_OK;
    }
    return knob_csc_write(st);
}

static int knob_csc_read(hisi_state_t *st, int f, int *val)
{
    v4_isp_csc_attr a;

    if (knob_live(st) && knob_csc_get(st, &a) == RSS_OK) {
        *val = *csc_field(&a, f);
        return RSS_OK;
    }
    if (csc_slot(st, f)->asked) {
        *val = csc_slot(st, f)->val;
        return RSS_OK;
    }
    return RSS_ERR_BUSY;
}

int hal_isp_set_brightness(void *ctx, int val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st ? knob_csc_set(st, CSC_LUMA, val) : RSS_ERR_INVAL;
}

int hal_isp_get_brightness(void *ctx, int *val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st && val ? knob_csc_read(st, CSC_LUMA, val) : RSS_ERR_INVAL;
}

int hal_isp_set_contrast(void *ctx, int val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st ? knob_csc_set(st, CSC_CONTR, val) : RSS_ERR_INVAL;
}

int hal_isp_get_contrast(void *ctx, int *val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st && val ? knob_csc_read(st, CSC_CONTR, val) : RSS_ERR_INVAL;
}

int hal_isp_set_saturation(void *ctx, int val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st ? knob_csc_set(st, CSC_SATU, val) : RSS_ERR_INVAL;
}

int hal_isp_get_saturation(void *ctx, int *val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st && val ? knob_csc_read(st, CSC_SATU, val) : RSS_ERR_INVAL;
}

/* ---------------- AE compensation ---------------- */

static int knob_ae_get(hisi_state_t *st, v4_isp_exp_attr *a)
{
    int ret;

    hisi_isp_tune_resolve(st);
    if (!st->tune.get_exp || !st->tune.set_exp)
        return RSS_ERR_NOTSUP;
    ret = st->tune.get_exp(HISI_VI_PIPE, a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_GetExposureAttr failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    /* The first look at the attribute since the last load is the
     * baseline, whatever writes come after. Not the tuning file's --
     * [static_ae] has no Compensation key and no load writes this field --
     * but the AE library's own, which is 56 on every sensor here and still
     * read rather than assumed. */
    if (!st->knob.ae_base_known) {
        st->knob.ae_base = a->auto_attr.compensation;
        st->knob.ae_base_known = true;
    }
    return RSS_OK;
}

static int knob_ae_write(hisi_state_t *st, int val, const char *why)
{
    v4_isp_exp_attr a;
    int ret;

    ret = knob_ae_get(st, &a);
    if (ret)
        return ret;
    if (a.auto_attr.compensation == val)
        return RSS_OK;
    a.auto_attr.compensation = (unsigned char)val;
    ret = st->tune.set_exp(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_SetExposureAttr(compensation %d) failed: 0x%x", val, ret);
        return RSS_ERR_IO;
    }
    HAL_LOG_INFO("ae_comp: %d%s (the AE's own is %d)", val, why, st->knob.ae_base);
    return RSS_OK;
}

int hal_isp_set_ae_comp(void *ctx, int val)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    if (val == RSS_ISP_AUTO) {
        int ret = RSS_OK;

        st->knob.ae_comp.asked = false;
        if (knob_live(st) && st->knob.ae_base_known)
            ret = knob_ae_write(st, st->knob.ae_base, ", the AE's own, put back");
        return ret;
    }
    if (val < 0 || val > KNOB_AE_MAX) {
        HAL_LOG_WARN("ae_comp: %d is outside the AE's 0..%d", val, KNOB_AE_MAX);
        return RSS_ERR_INVAL;
    }
    st->knob.ae_comp.asked = true;
    st->knob.ae_comp.val = val;
    if (!knob_live(st)) {
        HAL_LOG_DBG("ae_comp: %d noted for when the ISP runs", val);
        return RSS_OK;
    }
    return knob_ae_write(st, val, "");
}

int hal_isp_get_ae_comp(void *ctx, int *val)
{
    hisi_state_t *st = hisi_state(ctx);
    v4_isp_exp_attr a;

    if (!st || !val)
        return RSS_ERR_INVAL;
    if (knob_live(st) && knob_ae_get(st, &a) == RSS_OK) {
        *val = a.auto_attr.compensation;
        return RSS_OK;
    }
    if (st->knob.ae_comp.asked) {
        *val = st->knob.ae_comp.val;
        return RSS_OK;
    }
    return RSS_ERR_BUSY;
}

/* ---------------- DRC strength ---------------- */

static int knob_drc_get(hisi_state_t *st, v4_isp_drc_attr *a)
{
    int ret;

    hisi_isp_tune_resolve(st);
    if (!st->tune.get_drc || !st->tune.set_drc)
        return RSS_ERR_NOTSUP;
    ret = st->tune.get_drc(HISI_VI_PIPE, a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_GetDRCAttr failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    if (!st->knob.drc_base_known) {
        st->knob.drc_base_op = a->op_type;
        st->knob.drc_base = a->op_type == 0 ? a->auto_strength : a->manual_strength;
        st->knob.drc_base_known = true;
    }
    return RSS_OK;
}

static int knob_drc_write(hisi_state_t *st, int val)
{
    v4_isp_drc_attr a;
    int ret;

    ret = knob_drc_get(st, &a);
    if (ret)
        return ret;
    /* The engine's column and the pin write the same field. */
    hisi_dyn_drc_hold(st, true);
    if (a.op_type == 1 && a.manual_strength == val)
        return RSS_OK;
    a.op_type = 1;
    a.manual_strength = (unsigned short)val;
    ret = st->tune.set_drc(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_SetDRCAttr(strength %d) failed: 0x%x", val, ret);
        return RSS_ERR_IO;
    }
    HAL_LOG_INFO("drc_strength: %d pinned (the tuning's is %d, %s)", val, st->knob.drc_base,
                 st->knob.drc_base_op == 0 ? "auto" : "manual");
    return RSS_OK;
}

/* Put the tuning's strength and op type back; the engine, if the file has
 * one, then takes the field over again. */
static int knob_drc_release(hisi_state_t *st)
{
    v4_isp_drc_attr a;
    int ret;

    if (!st->knob.drc_base_known)
        return RSS_OK;
    ret = knob_drc_get(st, &a);
    if (ret)
        return ret;
    a.op_type = st->knob.drc_base_op;
    if (a.op_type == 0)
        a.auto_strength = (unsigned short)st->knob.drc_base;
    else
        a.manual_strength = (unsigned short)st->knob.drc_base;
    ret = st->tune.set_drc(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_SetDRCAttr(strength %d, the tuning's) failed: 0x%x",
                    st->knob.drc_base, ret);
        return RSS_ERR_IO;
    }
    HAL_LOG_INFO("drc_strength: the tuning's %d put back", st->knob.drc_base);
    return RSS_OK;
}

int hal_isp_set_drc_strength(void *ctx, int val)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    if (val == RSS_ISP_AUTO) {
        int ret = RSS_OK;

        st->knob.drc.asked = false;
        if (knob_live(st)) {
            ret = knob_drc_release(st);
            hisi_dyn_drc_hold(st, false);
        }
        return ret;
    }
    if (val < 0 || val > KNOB_DRC_MAX) {
        HAL_LOG_WARN("drc_strength: %d is outside the DRC's 0..%d", val, KNOB_DRC_MAX);
        return RSS_ERR_INVAL;
    }
    st->knob.drc.asked = true;
    st->knob.drc.val = val;
    if (!knob_live(st)) {
        HAL_LOG_DBG("drc_strength: %d noted for when the ISP runs", val);
        return RSS_OK;
    }
    return knob_drc_write(st, val);
}

int hal_isp_get_drc_strength(void *ctx, int *val)
{
    hisi_state_t *st = hisi_state(ctx);
    v4_isp_drc_attr a;

    if (!st || !val)
        return RSS_ERR_INVAL;
    if (knob_live(st) && knob_drc_get(st, &a) == RSS_OK) {
        *val = a.op_type == 0 ? a.auto_strength : a.manual_strength;
        return RSS_OK;
    }
    if (st->knob.drc.asked) {
        *val = st->knob.drc.val;
        return RSS_OK;
    }
    return RSS_ERR_BUSY;
}

/* ---------------- caps ---------------- */

int hal_isp_get_knob_caps(void *ctx, const char *name, rss_isp_knob_t *caps)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !name || !caps)
        return RSS_ERR_INVAL;
    memset(caps, 0, sizeof(*caps));
    caps->enabled = true;

    if (strcmp(name, "brightness") == 0 || strcmp(name, "contrast") == 0 ||
        strcmp(name, "saturation") == 0) {
        v4_isp_csc_attr a;

        caps->min = 0;
        caps->max = KNOB_CSC_MAX;
        caps->neutral = KNOB_CSC_UNITY;
        caps->has_auto = false;
        if (knob_live(st) && knob_csc_get(st, &a) == RSS_OK)
            caps->enabled = a.enable != 0;
        return RSS_OK;
    }
    if (strcmp(name, "ae_comp") == 0) {
        v4_isp_exp_attr a;

        caps->min = 0;
        caps->max = KNOB_AE_MAX;
        /*
         * True on the weaker of the two readings this flag carries. There
         * is no curve behind it: nothing varies u8Compensation, so `auto`
         * writes the same constant `neutral` names and differs only in
         * that rcd then stores the word instead of the number -- which
         * still matters, the number being lib_hiae.so's rather than this
         * port's. Left true so an operator can say "follow the AE's own"
         * and have the config keep meaning that; the alternative reading,
         * where auto promises a knob that moves with the light, is
         * drc_strength's below and this is not it.
         */
        caps->has_auto = true;
        /* The neutral is the AE library's, learned by the first look; before
         * the ISP runs there is nothing to look at and its published default
         * stands in. */
        if (!st->knob.ae_base_known && knob_live(st))
            knob_ae_get(st, &a);
        caps->neutral = st->knob.ae_base_known ? st->knob.ae_base : 56;
        return RSS_OK;
    }
    if (strcmp(name, "drc_strength") == 0) {
        v4_isp_drc_attr a;

        caps->min = 0;
        caps->max = KNOB_DRC_MAX;
        /*
         * Auto is the [dynamic_linear_drc] column for the light the AE is
         * reporting, so it is offered when the tuning has that curve and
         * not otherwise -- a file with one static strength has nothing for
         * auto to hand back to that `neutral` below does not already say,
         * and a control promising a curve there would be promising one
         * that does not exist. Until the file has been read there is
         * nothing to know, and the optimistic answer is the right one: a
         * control drawn and then withdrawn is worse than one that refuses
         * at the edge.
         */
        caps->has_auto =
            !__atomic_load_n(&st->iq_load_started, __ATOMIC_ACQUIRE) || hisi_dyn_drc_curve(st);
        if (knob_live(st) && knob_drc_get(st, &a) == RSS_OK)
            caps->enabled = a.enable != 0;
        caps->neutral = st->knob.drc_base_known ? st->knob.drc_base : 0;
        return RSS_OK;
    }
    return RSS_ERR_NOTSUP;
}

/* ---------------- the loader's brackets ---------------- */

void hisi_knob_before_load(hisi_state_t *st)
{
    if (st->knob.ae_comp.asked && st->knob.ae_base_known)
        knob_ae_write(st, st->knob.ae_base, ", the AE's own, back for the load");
    if (st->knob.drc.asked && st->knob.drc_base_known)
        knob_drc_release(st);
    st->knob.ae_base_known = false;
    st->knob.drc_base_known = false;
}

void hisi_knob_reapply(hisi_state_t *st)
{
    knob_csc_write(st);
    if (st->knob.ae_comp.asked)
        knob_ae_write(st, st->knob.ae_comp.val, ", again over the tuning");
    if (st->knob.drc.asked)
        knob_drc_write(st, st->knob.drc.val);
}

/* ---------------- the readback ---------------- */

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    hisi_state_t *st = hisi_state(ctx);
    v4_isp_exp_info *info;
    unsigned long long gain;
    int ret;

    if (!st || !exposure)
        return RSS_ERR_INVAL;
    /* ric polls through bring-up once a second; until the 3A thread runs
     * there is no exposure to report, and "busy" says exactly that. */
    if (!knob_live(st))
        return RSS_ERR_BUSY;
    hisi_isp_tune_resolve(st);
    if (!st->tune.query_exp)
        return RSS_ERR_NOTSUP;

    /* 5 KB, from the heap rather than a thread's stack. */
    info = calloc(1, sizeof(*info));
    if (!info)
        return RSS_ERR_NOMEM;
    ret = st->tune.query_exp(HISI_VI_PIPE, info);
    if (ret) {
        if (!st->knob.exp_warned) {
            HAL_LOG_WARN("HI_MPI_ISP_QueryExposureInfo failed: 0x%x -- no exposure readback, "
                         "ric will hold its current mode",
                         ret);
            st->knob.exp_warned = true;
        }
        free(info);
        return RSS_ERR_IO;
    }

    memset(exposure, 0, sizeof(*exposure));
    exposure->exposure_time = info->exp_time;
    exposure->valid_mask |= RSS_EXPOSURE_VALID_TIME;
    /* Three 22.10 gains into one 1024-per-unit figure: each product
     * carries the unit twice, so one factor is divided back out. */
    gain = ((unsigned long long)info->again * info->dgain) >> 10;
    gain = (gain * info->isp_dgain) >> 10;
    exposure->total_gain = gain > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)gain;
    exposure->valid_mask |= RSS_EXPOSURE_VALID_TOTAL_GAIN;
    exposure->ae_luma = info->ave_lum;
    exposure->valid_mask |= RSS_EXPOSURE_VALID_AE_LUMA;
    free(info);
    return RSS_OK;
}

/* ---------------- orientation: the VPSS channels' bMirror / bFlip ---------------- */

/*
 * The value is remembered here and written to every framesource channel
 * that exists by hisi_fs_apply_orien; a channel created later takes it
 * from hisi_fs_fill_attr. Before any exists it is only remembered, which is
 * how rvd's [image] hflip / vflip reach the picture from the first frame:
 * rvd hands them over in the sensor config before hal_init as well as
 * through these ops after it. See hisi_state_t.mirror for why VPSS.
 */
int hal_isp_set_hflip(void *ctx, int enable)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    st->mirror = enable ? 1 : 0;
    return hisi_fs_apply_orien(st);
}

int hal_isp_set_vflip(void *ctx, int enable)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    st->flip = enable ? 1 : 0;
    return hisi_fs_apply_orien(st);
}

/* The remembered value, not a read-back: HI_MPI_VPSS_GetChnAttr answers only
 * while the group runs, and the answer would be the same. */
int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !hflip || !vflip)
        return RSS_ERR_INVAL;
    *hflip = st->mirror;
    *vflip = st->flip;
    return RSS_OK;
}
