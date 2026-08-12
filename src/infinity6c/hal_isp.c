/*
 * infinity6c/hal_isp.c -- ISP tuning ops, MI 3.0
 *
 * For now just the exposure readback ric polls for its day/night decision,
 * plus a one-shot diagnostic of the 3A state at the first successful read.
 * The live AE and AWB state comes from CUS3A, the running algorithm, through
 * MI_ISP_CUS3A_GetAeStatus / GetAwbStatus -- not the MI_ISP_AE_* / AWB_* query
 * layer, which reports API-set values that stay at their defaults while CUS3A
 * owns 3A. Each is queried per (device, channel), and the device index carries
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
