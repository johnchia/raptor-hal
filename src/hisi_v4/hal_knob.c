/*
 * hisi_v4/hal_knob.c -- the [image] knobs, and the exposure readback
 *
 * Four knobs, each a get-modify-set on one ISP attribute, three of which
 * the tuning loader (hal_isp.c) already reaches and one it does not:
 *
 *   brightness, contrast   ISP_CSC_ATTR_S u8Luma / u8Contr, 0..100 with 50
 *                          as unity. The CSC is the last stage before YUV
 *                          and the vendor's own image-adjust surface. The
 *                          tuning file has no section for it, so the
 *                          driver's 50/50 is what "as the tuning left it"
 *                          means for these two.
 *   ae_comp                ISP_EXPOSURE_ATTR_S stAuto.u8Compensation,
 *                          0..255: the AE's target-luma multiplier. The
 *                          tuning's [static_ae] may set it, so unity is
 *                          whatever the loader left -- learned before the
 *                          knob's first write over it and forgotten at every
 *                          load, which is the SigmaStar backend's lesson
 *                          (docs/sigmastar.md, "the neutral is the
 *                          tuning's, not the midpoint").
 *   drc_strength           ISP_DRC_ATTR_S strength, 0..1023, pinned in
 *                          manual mode. The [dynamic_linear_drc] engine
 *                          (hal_dyn.c) writes the same field by ISO, so a pin
 *                          holds the engine's DRC column for as long as it
 *                          stands and `auto` hands it back.
 *
 * Units are the hardware's own, as [image]'s comment in raptor.conf
 * promises, and isp_get_knob_caps says what they are.
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

/* ---------------- CSC: brightness and contrast ---------------- */

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
    int ret;

    if (!st->knob.brightness.asked && !st->knob.contrast.asked)
        return RSS_OK;
    ret = knob_csc_get(st, &a);
    if (ret)
        return ret;
    if (st->knob.brightness.asked)
        a.luma = (unsigned char)st->knob.brightness.val;
    if (st->knob.contrast.asked)
        a.contr = (unsigned char)st->knob.contrast.val;
    ret = st->tune.set_csc(HISI_VI_PIPE, &a);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_SetCSCAttr(luma %u, contrast %u) failed: 0x%x", a.luma, a.contr,
                    ret);
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

static int knob_csc_set(hisi_state_t *st, bool bright, int val)
{
    const char *name = bright ? "brightness" : "contrast";

    if (val == RSS_ISP_AUTO)
        val = KNOB_CSC_UNITY; /* no auto mode to hand back to: unity is the tuning's */
    if (val < 0 || val > KNOB_CSC_MAX) {
        HAL_LOG_WARN("%s: %d is outside the CSC's 0..%d", name, val, KNOB_CSC_MAX);
        return RSS_ERR_INVAL;
    }
    if (bright) {
        st->knob.brightness.asked = true;
        st->knob.brightness.val = val;
    } else {
        st->knob.contrast.asked = true;
        st->knob.contrast.val = val;
    }
    if (!knob_live(st)) {
        HAL_LOG_DBG("%s: %d noted for when the ISP runs", name, val);
        return RSS_OK;
    }
    return knob_csc_write(st);
}

static int knob_csc_read(hisi_state_t *st, bool bright, int *val)
{
    v4_isp_csc_attr a;

    if (knob_live(st) && knob_csc_get(st, &a) == RSS_OK) {
        *val = bright ? a.luma : a.contr;
        return RSS_OK;
    }
    if (bright ? st->knob.brightness.asked : st->knob.contrast.asked) {
        *val = bright ? st->knob.brightness.val : st->knob.contrast.val;
        return RSS_OK;
    }
    return RSS_ERR_BUSY;
}

int hal_isp_set_brightness(void *ctx, int val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st ? knob_csc_set(st, true, val) : RSS_ERR_INVAL;
}

int hal_isp_get_brightness(void *ctx, int *val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st && val ? knob_csc_read(st, true, val) : RSS_ERR_INVAL;
}

int hal_isp_set_contrast(void *ctx, int val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st ? knob_csc_set(st, false, val) : RSS_ERR_INVAL;
}

int hal_isp_get_contrast(void *ctx, int *val)
{
    hisi_state_t *st = hisi_state(ctx);

    return st && val ? knob_csc_read(st, false, val) : RSS_ERR_INVAL;
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
     * tuning's own value, whatever writes come after. */
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
    HAL_LOG_INFO("ae_comp: %d%s (the tuning's is %d)", val, why, st->knob.ae_base);
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
            ret = knob_ae_write(st, st->knob.ae_base, ", the tuning's own, put back");
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

    if (strcmp(name, "brightness") == 0 || strcmp(name, "contrast") == 0) {
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
        caps->has_auto = true;
        /* The neutral is the tuning's, learned by the first look; before
         * the ISP runs there is nothing to look at and the driver's own
         * default stands in. */
        if (!st->knob.ae_base_known && knob_live(st))
            knob_ae_get(st, &a);
        caps->neutral = st->knob.ae_base_known ? st->knob.ae_base : 56;
        return RSS_OK;
    }
    if (strcmp(name, "drc_strength") == 0) {
        v4_isp_drc_attr a;

        caps->min = 0;
        caps->max = KNOB_DRC_MAX;
        caps->has_auto = true;
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
        knob_ae_write(st, st->knob.ae_base, ", the tuning's own, back for the load");
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
