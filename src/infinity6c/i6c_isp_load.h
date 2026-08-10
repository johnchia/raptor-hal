/*
 * infinity6c/i6c_isp_load.h -- dlopen loader for MI_ISP, MI 3.0
 *
 * The stage with no MI 2.x counterpart at all. On that generation the ISP is a
 * set of tuning calls folded into VPE; here it is a pipeline stage in its own
 * right, with a device, a channel and output ports, sitting between VIF and SCL.
 * So this file has nothing in star/ to be diffed against, and the layouts it
 * refers to were derived rather than adapted.
 *
 * Three libraries, and the two dependencies are not optional. libmi_isp.so leaves
 * the 3A and algorithm entry points undefined and expects them resolved out of the
 * loading process, so libispalgo.so and libcus3a.so have to be open with
 * RTLD_GLOBAL before it. Carry on without them and the failure arrives as a jump
 * into an unresolved slot once the channel starts, naming nothing.
 *
 * On the SoC id: MI_ISP does not take one as a separate argument the way MI_SYS
 * does. It rides in the high halfword of the device index -- MI_ISP_EnableOutputPort
 * shifts it out with lsr #16 -- which is what I6C_DEV_ID() composes.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_ISP_LOAD_H
#define INFINITY6C_I6C_ISP_LOAD_H

#include "hal_symbols.h"

#include "i6c_isp.h"

typedef struct {
    void *lib;
    void *lib_algo;
    void *lib_cus3a;

    /*
     * The combo argument is a bitmask of the sensor pads this device serves, not
     * a count. One sensor means bit 0.
     */
    int (*create_dev)(unsigned int device, unsigned int *combo);
    int (*destroy_dev)(unsigned int device);

    int (*create_chn)(unsigned int device, unsigned int channel, i6c_isp_chn *config);
    int (*destroy_chn)(unsigned int device, unsigned int channel);
    int (*set_chn_param)(unsigned int device, unsigned int channel, i6c_isp_para *config);
    int (*start_chn)(unsigned int device, unsigned int channel);
    int (*stop_chn)(unsigned int device, unsigned int channel);

    int (*set_port_param)(unsigned int device, unsigned int channel, unsigned int port,
                          i6c_isp_port *config);
    int (*enable_port)(unsigned int device, unsigned int channel, unsigned int port);
    int (*disable_port)(unsigned int device, unsigned int channel, unsigned int port);

    /*
     * The last two are bound but have NO CALLER yet: this backend publishes no
     * isp_* ops, so there is no tuning surface for them to sit behind. They are
     * declared here because the constraints on using them are what cost the
     * investigation, and rediscovering those is the expensive part.
     *
     * The IQ tuning binary, and the ordering is the whole difficulty. CUS3A's AE
     * initialisation writes over the API-level tuning, so a load issued before the
     * first frame is silently read back over -- the call succeeds, the tuning does
     * not take, and nothing anywhere reports it. Gate the load on the first frame
     * the way star/hal_isp.c does. Until something does, tuning on this part is
     * inert rather than mis-ordered, which is a different symptom with a different
     * first suspect.
     */
    int (*load_bin)(unsigned int device, unsigned int channel, char *path, unsigned int key);

    /* Day/night without touching the sensor, for a mono night mode. Also uncalled. */
    int (*set_color_to_gray)(unsigned int device, unsigned int channel, char *enable);
} i6c_isp_api;

static inline int i6c_isp_load(i6c_isp_api *isp)
{
    if (!(isp->lib_algo = dlopen("libispalgo.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_isp: dlopen(libispalgo.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(isp->lib_cus3a = dlopen("libcus3a.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_isp: dlopen(libcus3a.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(isp->lib = dlopen("libmi_isp.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_isp: dlopen(libmi_isp.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(isp->create_dev = (int (*)(unsigned int, unsigned int *))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_CreateDevice")))
        return RSS_ERR_NOTSUP;

    /*
     * Spelled as the vendor spells it. The symbol is misspelt in the library, so
     * the correct spelling resolves to nothing.
     */
    if (!(isp->destroy_dev =
              (int (*)(unsigned int))hal_symbol_load("i6c_isp", isp->lib, "MI_ISP_DestoryDevice")))
        return RSS_ERR_NOTSUP;

    if (!(isp->create_chn = (int (*)(unsigned int, unsigned int, i6c_isp_chn *))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_CreateChannel")))
        return RSS_ERR_NOTSUP;

    if (!(isp->destroy_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_DestroyChannel")))
        return RSS_ERR_NOTSUP;

    if (!(isp->set_chn_param = (int (*)(unsigned int, unsigned int, i6c_isp_para *))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_SetChnParam")))
        return RSS_ERR_NOTSUP;

    if (!(isp->start_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_StartChannel")))
        return RSS_ERR_NOTSUP;

    if (!(isp->stop_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_StopChannel")))
        return RSS_ERR_NOTSUP;

    if (!(isp->set_port_param = (int (*)(unsigned int, unsigned int, unsigned int, i6c_isp_port *))
              hal_symbol_load("i6c_isp", isp->lib, "MI_ISP_SetOutputPortParam")))
        return RSS_ERR_NOTSUP;

    if (!(isp->enable_port = (int (*)(unsigned int, unsigned int, unsigned int))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_EnableOutputPort")))
        return RSS_ERR_NOTSUP;

    if (!(isp->disable_port = (int (*)(unsigned int, unsigned int, unsigned int))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_DisableOutputPort")))
        return RSS_ERR_NOTSUP;

    if (!(isp->load_bin = (int (*)(unsigned int, unsigned int, char *, unsigned int))
              hal_symbol_load("i6c_isp", isp->lib, "MI_ISP_ApiCmdLoadBinFile")))
        return RSS_ERR_NOTSUP;

    if (!(isp->set_color_to_gray = (int (*)(unsigned int, unsigned int, char *))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_IQ_SetColorToGray")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_isp_unload(i6c_isp_api *isp)
{
    if (isp->lib)
        dlclose(isp->lib);
    if (isp->lib_cus3a)
        dlclose(isp->lib_cus3a);
    if (isp->lib_algo)
        dlclose(isp->lib_algo);
    memset(isp, 0, sizeof(*isp));
}

#endif /* INFINITY6C_I6C_ISP_LOAD_H */
