/*
 * infinity6c/i6c_snr_load.h -- dlopen loader for MI_SNR, MI 3.0
 *
 * The sensor module is where the pipeline's geometry comes from rather than
 * something the HAL decides: the resolution list, the interface the sensor is
 * wired over, and the bayer order and precision the VIF has to be told about all
 * come back from here. That is why bring-up queries it first and configures VIF,
 * ISP and SCL from the answers, instead of carrying a per-sensor table.
 *
 * On the SoC id: MI_SNR does not take one as a separate argument the way MI_SYS
 * does. It rides in the high halfword of the pad index, which is what
 * I6C_DEV_ID() in infinity6c_state.h composes. Passing a bare index works only
 * because a single-die part puts zero in both halves.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_SNR_LOAD_H
#define INFINITY6C_I6C_SNR_LOAD_H

#include "hal_symbols.h"

#include "i6c_snr.h"

typedef struct {
    void *lib;

    int (*enable)(unsigned int pad);
    int (*disable)(unsigned int pad);

    /*
     * Resolution selection is a query-then-choose, not a set: the sensor driver
     * publishes a list and the HAL picks an entry by index. Nothing accepts a
     * width and height directly.
     */
    int (*query_res_count)(unsigned int pad, unsigned int *count);
    int (*get_res)(unsigned int pad, unsigned char index, i6c_snr_res *res);
    int (*set_res)(unsigned int pad, unsigned char index);
    int (*get_cur_res)(unsigned int pad, unsigned char *index, i6c_snr_res *res);

    int (*set_fps)(unsigned int pad, unsigned int fps);
    int (*set_orien)(unsigned int pad, unsigned char mirror, unsigned char flip);

    /*
     * Single-plane mode. The multi-plane path is for HDR sensors that expose one
     * plane per exposure; with it off there is one plane and plane 0 describes
     * the whole frame.
     */
    int (*set_plane_mode)(unsigned int pad, unsigned char multi);

    /* The two descriptors the rest of bring-up reads its geometry out of. */
    int (*get_pad_info)(unsigned int pad, i6c_snr_pad *info);
    int (*get_plane_info)(unsigned int pad, unsigned int plane, i6c_snr_plane *info);
} i6c_snr_api;

static inline int i6c_snr_load(i6c_snr_api *snr)
{
    if (!(snr->lib = dlopen("libmi_sensor.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_snr: dlopen(libmi_sensor.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(snr->enable =
              (int (*)(unsigned int))hal_symbol_load("i6c_snr", snr->lib, "MI_SNR_Enable")))
        return RSS_ERR_NOTSUP;

    if (!(snr->disable =
              (int (*)(unsigned int))hal_symbol_load("i6c_snr", snr->lib, "MI_SNR_Disable")))
        return RSS_ERR_NOTSUP;

    if (!(snr->query_res_count = (int (*)(unsigned int, unsigned int *))hal_symbol_load(
              "i6c_snr", snr->lib, "MI_SNR_QueryResCount")))
        return RSS_ERR_NOTSUP;

    if (!(snr->get_res = (int (*)(unsigned int, unsigned char, i6c_snr_res *))hal_symbol_load(
              "i6c_snr", snr->lib, "MI_SNR_GetRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr->set_res = (int (*)(unsigned int, unsigned char))hal_symbol_load(
              "i6c_snr", snr->lib, "MI_SNR_SetRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr->get_cur_res =
              (int (*)(unsigned int, unsigned char *, i6c_snr_res *))hal_symbol_load(
                  "i6c_snr", snr->lib, "MI_SNR_GetCurRes")))
        return RSS_ERR_NOTSUP;

    if (!(snr->set_fps = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_snr", snr->lib, "MI_SNR_SetFps")))
        return RSS_ERR_NOTSUP;

    if (!(snr->set_orien =
              (int (*)(unsigned int, unsigned char, unsigned char))hal_symbol_load(
                  "i6c_snr", snr->lib, "MI_SNR_SetOrien")))
        return RSS_ERR_NOTSUP;

    if (!(snr->set_plane_mode = (int (*)(unsigned int, unsigned char))hal_symbol_load(
              "i6c_snr", snr->lib, "MI_SNR_SetPlaneMode")))
        return RSS_ERR_NOTSUP;

    if (!(snr->get_pad_info = (int (*)(unsigned int, i6c_snr_pad *))hal_symbol_load(
              "i6c_snr", snr->lib, "MI_SNR_GetPadInfo")))
        return RSS_ERR_NOTSUP;

    if (!(snr->get_plane_info =
              (int (*)(unsigned int, unsigned int, i6c_snr_plane *))hal_symbol_load(
                  "i6c_snr", snr->lib, "MI_SNR_GetPlaneInfo")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_snr_unload(i6c_snr_api *snr)
{
    if (snr->lib)
        dlclose(snr->lib);
    memset(snr, 0, sizeof(*snr));
}

#endif /* INFINITY6C_I6C_SNR_LOAD_H */
