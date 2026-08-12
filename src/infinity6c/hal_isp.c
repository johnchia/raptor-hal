/*
 * infinity6c/hal_isp.c -- ISP tuning ops, MI 3.0
 *
 * For now just the exposure readback ric polls for its day/night decision,
 * plus a one-shot diagnostic of the 3A state at the first successful read.
 * Unlike MI 2.x, where the AE status came back from a sensor-driver callback
 * taking only a channel, here it lives behind the ISP library's own AE and AWB
 * entry points, queried per (device, channel) -- and the device index carries
 * the SoC id in its high halfword, so I6C_DEV_ID composes it as elsewhere.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "infinity6c_state.h"

static infinity6c_state_t *i6c_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (infinity6c_state_t *)c->platform : NULL;
}

/*
 * The 3A state at the first successful readback, logged once. It is how the
 * colour path is diagnosed without a board in hand: the AWB op type says
 * whether white balance is on auto at all, and the gains and colour temperature
 * say where it has settled. If those do not move when the illuminant changes,
 * auto is running but not adapting; if the op type is manual, it never was.
 */
static void i6c_isp_log_3a(infinity6c_state_t *st, const i6c_isp_ae_info *ae,
                           const i6c_isp_awb_info *awb)
{
    i6c_isp_awb_attr attr;

    HAL_LOG_INFO("isp/ae: %s lv=%u.%u shutter=%uus sensorgain=%u ispgain=%u avgY=%u target=%u",
                 ae->stable ? "stable" : "settling", ae->lvX10 / 10, ae->lvX10 % 10,
                 ae->expoLong.us, ae->expoLong.sensorGain, ae->expoLong.ispGain,
                 ae->histWeightY.avgY, ae->sceneTarget);

    if (awb)
        HAL_LOG_INFO("isp/awb: %s gains r=%u gr=%u gb=%u b=%u colortemp=%uK",
                     awb->stable ? "stable" : "settling", awb->rGain, awb->grGain, awb->gbGain,
                     awb->bGain, awb->colorTemp);

    if (st->isp.awb_get_attr &&
        st->isp.awb_get_attr(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &attr) == 0)
        HAL_LOG_INFO("isp/awb: mode %s, %s", attr.opType == I6C_AWB_OP_MANUAL ? "manual" : "auto",
                     attr.state == I6C_AWB_STATE_PAUSE ? "paused" : "running");
}

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure)
{
    infinity6c_state_t *st = i6c_state(ctx);
    static bool diag_logged;
    i6c_isp_awb_info awb;
    i6c_isp_ae_info ae;
    bool have_awb;
    int ret;

    if (!st || !exposure)
        return RSS_ERR_INVAL;
    if (!st->isp.ae_query)
        return RSS_ERR_NOTSUP;

    memset(exposure, 0, sizeof(*exposure));

    /*
     * Until the tuning bin loads on the first frame the AE has not run and a
     * query returns nothing useful. ric polls through that window once a
     * second, so answer "busy" and say nothing.
     */
    if (!st->iq_load_started)
        return RSS_ERR_BUSY;

    memset(&ae, 0, sizeof(ae));
    ret = st->isp.ae_query(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &ae);
    if (ret) {
        static bool warned;

        if (!warned) {
            HAL_LOG_WARN("isp: MI_ISP_AE_QueryExposureInfo failed: %d -- "
                         "no exposure readback, ric will hold its current mode",
                         ret);
            warned = true;
        }
        return RSS_ERR_IO;
    }

    exposure->exposure_time = ae.expoLong.us;
    /*
     * Combined analogue and ISP gain. MI reports each as a fixed-point
     * multiplier with a 1024 unit, so the product carries the unit twice and
     * one shift takes it back out. The magnitude is worth checking against the
     * board, which is why the diagnostic logs the raw parts.
     */
    exposure->total_gain =
        (uint32_t)(((uint64_t)ae.expoLong.sensorGain * ae.expoLong.ispGain) >> 10);
    exposure->ae_luma = ae.histWeightY.avgY;

    have_awb =
        st->isp.awb_query && st->isp.awb_query(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &awb) == 0;
    if (have_awb) {
        exposure->wb_rgain = awb.rGain;
        exposure->wb_bgain = awb.bGain;
    }

    if (!diag_logged) {
        diag_logged = true;
        i6c_isp_log_3a(st, &ae, have_awb ? &awb : NULL);
    }

    return RSS_OK;
}
