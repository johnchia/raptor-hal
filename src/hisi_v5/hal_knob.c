/*
 * hisi_v5/hal_knob.c -- the [image] knobs, and the exposure readback
 *
 * Five knobs over three ISP attributes, each a get-modify-set on one of
 * them. The tuning loader (hal_isp.c) writes all three, which is why the
 * knobs bracket a load rather than fighting it:
 *
 *   brightness, contrast,  ot_isp_csc_attr luma / contr / satu, each
 *   saturation             0..100 with 50 as unity. The CSC is the last
 *                          stage before YUV and the vendor's own
 *                          image-adjust surface.
 *
 *                          Unlike gen4, a V5 scene file DOES carry
 *                          [static_csc], so the neutral 50 is "the CSC's
 *                          own unity" and not "what the tuning left" --
 *                          a file that sets luma 60 has moved the picture
 *                          and 50 moves it back, which is the honest
 *                          reading of a control the tuner also uses.
 *
 *                          Saturation is not the only writer of its
 *                          quantity: the ISP's own saturation
 *                          (ot_isp_saturation_attr, in libss_mpi_awb.so)
 *                          is a per-ISO table set by [static_saturation]
 *                          or, where the file has none, by the sensor
 *                          library's calibrated ladder. That runs on the
 *                          Bayer side and stays exactly as it was; this is
 *                          a flat adjustment of the YUV the ISP hands out.
 *   ae_comp                ot_isp_exposure_attr auto_attr.compensation,
 *                          0..255: the setpoint the AE converges its
 *                          target luma on, read afresh by the loop every
 *                          frame. The driver never moves it; the
 *                          [dynamic_ae] ladder (hal_ladder.c) does, by
 *                          exposure band, when the tuning file has one.
 *
 *                          The baseline is learned rather than named.
 *                          iq_sect_static_ae maps the [static_ae] keys and
 *                          compensation is not among them, so the value
 *                          at the first read after a load is the AE
 *                          library's own -- or the ladder's column for
 *                          the light at the time, where there is a
 *                          ladder. Learned rather than promised, and
 *                          re-learned at each load for the same reason.
 *
 *                          `auto` is offered exactly when the file varies
 *                          the field: a pin holds the ladder's AE engine
 *                          for as long as it stands and `auto` hands the
 *                          field back, the drc_strength arrangement
 *                          below. A file with no curve has nothing for
 *                          auto to hand back that `neutral` does not
 *                          already say, so the caps withhold it; the
 *                          sentinel is still accepted -- a config written
 *                          before this says `ae_comp = auto` and has to
 *                          keep loading -- and reset-isp reaches the same
 *                          value by writing caps.neutral.
 *   drc_strength           ot_isp_drc_attr strength, 0..1023, pinned in
 *                          manual mode. The [dynamic_linear_drc] engine
 *                          (hal_dyn.c) writes the same field by ISO, so a
 *                          pin holds that engine for as long as it stands
 *                          and `auto` hands it back -- and the caps offer
 *                          `auto` only where the engine has a curve to
 *                          hand it back to.
 *
 * Units are the hardware's own, as [image]'s comment in raptor.conf
 * promises, and isp_get_knob_caps says what they are.
 *
 * The orientation pair, hflip and vflip, is at the end of the file: not an
 * ISP attribute at all but the VI channel's mirror and flip bits, for the
 * reasons set out there -- the sensor libraries have no pfn_mirror_flip
 * and VPSS refuses a mirror outright.
 *
 * WHEN THE WRITE HAPPENS. rvd applies [image] right after hal_init, before
 * the first frame; the tuning loads on the first frame and rewrites all
 * three attributes from the file, which would silently undo every knob. So
 * every set is remembered, written at once when the 3A thread runs, and
 * written again by hisi_knob_reapply at the end of each load -- after the
 * static modules and the engines have laid down the baseline the knob
 * adjusts from. Before that load, hisi_knob_before_load lifts a pinned
 * knob back to its baseline so the file lands on what the tuner meant
 * rather than on the knob.
 *
 * THE READBACK is ss_mpi_isp_query_exposure_info, the same query the
 * engines' tick uses: exposure time, the sensor's analogue and digital
 * gains and the ISP's digital gain multiplied into one 1024-per-unit
 * figure (the console divides by 1024), and the AE's average luma. That is
 * what the OSD's %total_gain% and %ae_luma%, the console's sidebar and
 * ric's day/night decision read.
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

/*
 * The AE library's own compensation, for the caps before the ISP runs.
 * Every sensor library on this image answers 56, the same number gen4's
 * lib_hiae.so gives -- but it is still read rather than assumed once
 * there is something to read.
 */
#define KNOB_AE_COMP_DEFAULT 56

static bool knob_live(const hisi_state_t *st)
{
    return __atomic_load_n(&st->isp_thread_running, __ATOMIC_ACQUIRE) != 0;
}

/* ---------------- CSC: brightness, contrast and saturation ---------------- */

/*
 * Three of the CSC's four adjust fields. They share one attribute, so a
 * write of any is a get-modify-set of all, and they share these helpers
 * rather than repeating it three times. The fourth field is hue, on the
 * same scale and one row of this enum from being a knob as well; it is not
 * one because nothing has asked for it.
 */
enum { CSC_LUMA, CSC_CONTR, CSC_SATU, CSC_FIELDS };

static const char *const csc_names[CSC_FIELDS] = {"brightness", "contrast", "saturation"};

static unsigned char *csc_field(v5_isp_csc_attr *a, int f)
{
    return f == CSC_LUMA ? &a->luma : f == CSC_CONTR ? &a->contr : &a->satu;
}

static hisi_knob_slot_t *csc_slot(hisi_state_t *st, int f)
{
    return f == CSC_LUMA    ? &st->knob.brightness
           : f == CSC_CONTR ? &st->knob.contrast
                            : &st->knob.saturation;
}

static int knob_csc_get(hisi_state_t *st, v5_isp_csc_attr *a)
{
    int ret;

    hisi_isp_tune_resolve(st);
    if (!st->tune.fnGetCscAttr || !st->tune.fnSetCscAttr)
        return RSS_ERR_NOTSUP;
    ret = st->tune.fnGetCscAttr(HISI_VI_PIPE, a);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_get_csc_attr failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

static int knob_csc_write(hisi_state_t *st)
{
    v5_isp_csc_attr a;
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
    ret = st->tune.fnSetCscAttr(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_set_csc_attr(luma %u, contrast %u, saturation %u) failed: 0x%x",
                    a.luma, a.contr, a.satu, ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

static int knob_csc_set(hisi_state_t *st, int f, int val)
{
    if (val == RSS_ISP_AUTO)
        val = KNOB_CSC_UNITY; /* no auto mode to hand back to: unity is the CSC's */
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
    v5_isp_csc_attr a;

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

static int knob_ae_get(hisi_state_t *st, v5_isp_exp_attr *a)
{
    int ret;

    hisi_isp_tune_resolve(st);
    if (!st->tune.fnGetExposureAttr || !st->tune.fnSetExposureAttr)
        return RSS_ERR_NOTSUP;
    ret = st->tune.fnGetExposureAttr(HISI_VI_PIPE, a);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_get_exposure_attr failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    /* The first look at the attribute since the last load is the baseline,
     * whatever writes come after. Not the tuning file's -- [static_ae] has
     * no compensation key and no load writes this field -- but the AE
     * library's own, read rather than assumed. */
    if (!st->knob.ae_base_known) {
        st->knob.ae_base = a->auto_attr.compensation;
        st->knob.ae_base_known = true;
    }
    return RSS_OK;
}

static int knob_ae_write(hisi_state_t *st, int val, const char *why)
{
    v5_isp_exp_attr a;
    int ret;

    ret = knob_ae_get(st, &a);
    if (ret)
        return ret;
    /* The ladder's band and the pin write the same field. */
    hisi_lad_ae_hold(st, true);
    if (a.auto_attr.compensation == val)
        return RSS_OK;
    a.auto_attr.compensation = (unsigned char)val;
    ret = st->tune.fnSetExposureAttr(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_set_exposure_attr(compensation %d) failed: 0x%x", val, ret);
        return RSS_ERR_IO;
    }
    HAL_LOG_INFO("ae_comp: %d%s (the baseline is %d)", val, why, st->knob.ae_base);
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
        if (knob_live(st)) {
            if (st->knob.ae_base_known)
                ret = knob_ae_write(st, st->knob.ae_base, ", the baseline, put back");
            hisi_lad_ae_hold(st, false);
        }
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
    v5_isp_exp_attr a;

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

static int knob_drc_get(hisi_state_t *st, v5_isp_drc_attr *a)
{
    int ret;

    hisi_isp_tune_resolve(st);
    if (!st->tune.fnGetDrcAttr || !st->tune.fnSetDrcAttr)
        return RSS_ERR_NOTSUP;
    ret = st->tune.fnGetDrcAttr(HISI_VI_PIPE, a);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_get_drc_attr failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    if (!st->knob.drc_base_known) {
        st->knob.drc_base_op = a->op_type;
        st->knob.drc_base =
            a->op_type == V5_ISP_OP_AUTO ? a->auto_attr.strength : a->manual_attr.strength;
        st->knob.drc_base_known = true;
    }
    return RSS_OK;
}

static int knob_drc_write(hisi_state_t *st, int val)
{
    v5_isp_drc_attr a;
    int ret;

    ret = knob_drc_get(st, &a);
    if (ret)
        return ret;
    /* The engine's column and the pin write the same field. */
    hisi_dyn_drc_hold(st, true);
    if (a.op_type == V5_ISP_OP_MANUAL && a.manual_attr.strength == val)
        return RSS_OK;
    a.op_type = V5_ISP_OP_MANUAL;
    a.manual_attr.strength = (unsigned short)val;
    ret = st->tune.fnSetDrcAttr(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_set_drc_attr(strength %d) failed: 0x%x", val, ret);
        return RSS_ERR_IO;
    }
    HAL_LOG_INFO("drc_strength: %d pinned (the tuning's is %d, %s)", val, st->knob.drc_base,
                 st->knob.drc_base_op == V5_ISP_OP_AUTO ? "auto" : "manual");
    return RSS_OK;
}

/* Put the tuning's strength and op type back; the engine, if the file has
 * one, then takes the field over again. */
static int knob_drc_release(hisi_state_t *st)
{
    v5_isp_drc_attr a;
    int ret;

    if (!st->knob.drc_base_known)
        return RSS_OK;
    ret = knob_drc_get(st, &a);
    if (ret)
        return ret;
    a.op_type = st->knob.drc_base_op;
    if (a.op_type == V5_ISP_OP_AUTO)
        a.auto_attr.strength = (unsigned short)st->knob.drc_base;
    else
        a.manual_attr.strength = (unsigned short)st->knob.drc_base;
    ret = st->tune.fnSetDrcAttr(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_isp_set_drc_attr(strength %d, the tuning's) failed: 0x%x",
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
    v5_isp_drc_attr a;

    if (!st || !val)
        return RSS_ERR_INVAL;
    if (knob_live(st) && knob_drc_get(st, &a) == RSS_OK) {
        *val = a.op_type == V5_ISP_OP_AUTO ? a.auto_attr.strength : a.manual_attr.strength;
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
        v5_isp_csc_attr a;

        caps->min = 0;
        caps->max = KNOB_CSC_MAX;
        caps->neutral = KNOB_CSC_UNITY;
        caps->has_auto = false;
        if (knob_live(st) && knob_csc_get(st, &a) == RSS_OK)
            caps->enabled = a.enable != 0;
        return RSS_OK;
    }
    if (strcmp(name, "ae_comp") == 0) {
        v5_isp_exp_attr a;

        caps->min = 0;
        caps->max = KNOB_AE_MAX;
        /*
         * Auto is the [dynamic_ae] band for the exposure AE is reporting,
         * offered when the loaded tuning has that curve and not otherwise:
         * a knob whose only hand-back is a constant is what `neutral` is
         * for, and reset-isp takes that branch. Before the file has been
         * read the optimistic answer is the right one, as for drc_strength
         * below. The sentinel stays legal in the setter regardless -- this
         * flag is advice to a client about which control to draw.
         */
        caps->has_auto =
            !__atomic_load_n(&st->iq_load_started, __ATOMIC_ACQUIRE) || hisi_lad_ae_curve(st);
        /* The neutral is the AE library's, learned by the first look;
         * before the ISP runs there is nothing to look at. */
        if (!st->knob.ae_base_known && knob_live(st))
            knob_ae_get(st, &a);
        caps->neutral = st->knob.ae_base_known ? st->knob.ae_base : KNOB_AE_COMP_DEFAULT;
        return RSS_OK;
    }
    if (strcmp(name, "drc_strength") == 0) {
        v5_isp_drc_attr a;

        caps->min = 0;
        caps->max = KNOB_DRC_MAX;
        /*
         * Auto is the [dynamic_linear_drc] column for the light the AE is
         * reporting, so it is offered when the tuning has that curve and
         * not otherwise -- a file with one static strength has nothing for
         * auto to hand back to that `neutral` does not already say. Until
         * the file has been read there is nothing to know, and the
         * optimistic answer is the right one: a control drawn and then
         * withdrawn is worse than one that refuses at the edge.
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
        knob_ae_write(st, st->knob.ae_base, ", the baseline, back for the load");
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
    v5_isp_exp_info *info;
    unsigned long long gain;
    int ret;

    if (!st || !exposure)
        return RSS_ERR_INVAL;
    /* ric polls through bring-up once a second; until the 3A thread runs
     * there is no exposure to report, and "busy" says exactly that. */
    if (!knob_live(st))
        return RSS_ERR_BUSY;
    if (!st->isp.fnQueryExposureInfo)
        return RSS_ERR_NOTSUP;

    /* 5484 bytes, from the heap rather than a thread's stack. */
    info = calloc(1, sizeof(*info));
    if (!info)
        return RSS_ERR_NOMEM;
    ret = st->isp.fnQueryExposureInfo(HISI_VI_PIPE, info);
    if (ret) {
        if (!st->knob.exp_warned) {
            HAL_LOG_WARN("ss_mpi_isp_query_exposure_info failed: 0x%x -- no exposure readback, "
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
    gain = ((unsigned long long)info->a_gain * info->d_gain) >> 10;
    gain = (gain * info->isp_d_gain) >> 10;
    exposure->total_gain = gain > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)gain;
    exposure->valid_mask |= RSS_EXPOSURE_VALID_TOTAL_GAIN;
    exposure->ae_luma = info->ave_lum;
    exposure->valid_mask |= RSS_EXPOSURE_VALID_AE_LUMA;
    free(info);
    return RSS_OK;
}

/* ---------------- orientation: the VI channel's mirror_en / flip_en ---------------- */

/*
 * WHY VI AND NOT VPSS. Three places on this SoC carry a mirror: the sensor
 * object's pfn_mirror_flip, which every sensor library on this image
 * leaves null; the VPSS channel attribute, which would have given a
 * per-stream answer and which this driver refuses outright -- 0xa007800d,
 * OT_ERR_VPSS_NOT_PERM, with the channel enabled or disabled alike; and
 * the VI channel attribute, which is one channel for the whole pipe. So
 * orientation turns all three streams together, which is what [image]
 * means by hflip and vflip anyway.
 *
 * The value is remembered in the state and written here; bring-up reads
 * the same pair into the channel attribute it builds, which is how an
 * [image] hflip set before hal_init reaches the first frame.
 */
int hisi_vi_apply_orien(hisi_state_t *st)
{
    v5_vi_chn_attr chn;
    int ret;

    if (!st->vi.fnSetChnAttr || !st->vi.fnGetChnAttr)
        return RSS_ERR_NOTSUP;
    if (!st->vi_chn_enabled) {
        HAL_LOG_DBG("orientation: mirror %d, flip %d noted for bring-up", st->mirror, st->flip);
        return RSS_OK;
    }
    /* Get-modify-set: the channel attribute carries the geometry and the
     * pixel format bring-up settled, and rebuilding it here would be a
     * second copy of that to keep in step. */
    ret = st->vi.fnGetChnAttr(HISI_VI_PIPE, HISI_VI_CHN, &chn);
    if (ret) {
        HAL_LOG_WARN("ss_mpi_vi_get_chn_attr(pipe %d, chn %d) failed: 0x%x", HISI_VI_PIPE,
                     HISI_VI_CHN, ret);
        return RSS_ERR_IO;
    }
    if (chn.mirror_en == st->mirror && chn.flip_en == st->flip)
        return RSS_OK;
    chn.mirror_en = st->mirror;
    chn.flip_en = st->flip;
    ret = st->vi.fnSetChnAttr(HISI_VI_PIPE, HISI_VI_CHN, &chn);
    if (ret) {
        HAL_LOG_WARN("ss_mpi_vi_set_chn_attr(mirror %d, flip %d) failed: 0x%x", st->mirror,
                     st->flip, ret);
        return RSS_ERR_IO;
    }
    HAL_LOG_INFO("orientation: mirror %d, flip %d on the VI channel", st->mirror, st->flip);
    return RSS_OK;
}

int hal_isp_set_hflip(void *ctx, int enable)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    st->mirror = enable ? 1 : 0;
    return hisi_vi_apply_orien(st);
}

int hal_isp_set_vflip(void *ctx, int enable)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    st->flip = enable ? 1 : 0;
    return hisi_vi_apply_orien(st);
}

/* The remembered value, not a read-back: the channel attribute answers
 * only while the pipe runs, and the answer would be the same. */
int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !hflip || !vflip)
        return RSS_ERR_INVAL;
    *hflip = st->mirror;
    *vflip = st->flip;
    return RSS_OK;
}
