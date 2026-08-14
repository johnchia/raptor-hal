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
#include "i6c_isp_3a.h"

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
    /*
     * Read the channel parameters back, which is how orientation and the 3DNR
     * level are changed at runtime: the block is set whole, so a blind write
     * would put this file's idea of every other field over the SDK's. The SDK
     * also clamps what it is given (the 3DNR level to the per-chip maximum), so
     * the value read back is the one in force rather than the one asked for.
     */
    int (*get_chn_param)(unsigned int device, unsigned int channel, i6c_isp_para *config);
    int (*start_chn)(unsigned int device, unsigned int channel);
    int (*stop_chn)(unsigned int device, unsigned int channel);

    int (*set_port_param)(unsigned int device, unsigned int channel, unsigned int port,
                          i6c_isp_port *config);
    int (*enable_port)(unsigned int device, unsigned int channel, unsigned int port);
    int (*disable_port)(unsigned int device, unsigned int channel, unsigned int port);

    /*
     * The IQ tuning binary. The ordering is the whole difficulty, and it has two
     * halves. A load issued before the first frame is silently read back over, so
     * it is gated on the first delivered frame (i6c_isp_note_frame, from the
     * encoder's get_frame). And enable_3a below resets the AE's exposure envelope
     * as part of arming the algorithms, so the load has to come after that too --
     * either way the call succeeds, the tuning does not take, and nothing
     * anywhere reports it.
     */
    int (*load_bin)(unsigned int device, unsigned int channel, char *path, unsigned int key);

    /*
     * The tuning API, reached generically. MI_ISP_IQ_* and MI_ISP_AE_* are all
     * one wrapper shape -- a descriptor naming a payload length and an api id,
     * handed to MI_ISP_GENERAL_{Set,Get}IspApiData -- so a module is a name and
     * two numbers rather than a typed entry point, and hal_isp.c resolves the
     * names it needs from its own table through this handle. There is nothing to
     * bind here beyond the library.
     *
     * That is also why MI_ISP_IQ_SetColorToGray has no dedicated pointer: the
     * mono night mode reaches it as a table row like any other module. Its own
     * argument is a four-byte enum, not the byte the name suggests, which is a
     * good reason to have exactly one way in.
     */

    /*
     * CUS3A live 3A status, MI 3.0. Optional: the capture path does not use
     * them, so a library that lacks one leaves the pointer NULL and the matching
     * op says "not supported" rather than failing bringup. Bound with dlsym
     * directly for that reason -- hal_symbol_load logs a miss as an error, which
     * these are not. ae_status drives ric's day/night; awb_status is the
     * readback that says where white balance has settled.
     */
    int (*ae_status)(unsigned int device, unsigned int channel, i6c_cus_ae_info *info);
    int (*awb_status)(unsigned int device, unsigned int channel, i6c_cus_awb_info *info);

    /*
     * CUS3A framework enable, MI 3.0. The vendor 3A does not arm itself. Without
     * MI_ISP_EnableUserspace3A the SDK's 3A worker thread never spawns, so the AE
     * loop never runs -- exposure and gains stay at their power-on defaults -- and
     * no white balance is applied, which reads as a green cast (a Bayer sensor is
     * about twice as sensitive to green, so unity gains leave the image green).
     * cus3a_enable brings the AE and AWB algorithms up in the engine; enable_3a
     * spawns the worker that runs them each frame and runs their initialisation,
     * which is where the AE's exposure envelope goes back to the untuned default
     * -- so load_bin has to follow it. The enable argument is a Cus3AEnable_t --
     * three MI_BOOLs {AE, AWB, AF}. Optional, bound by dlsym: a library without
     * them leaves 3A on whatever the SDK gives by default.
     */
    int (*cus3a_enable)(unsigned int device, unsigned int channel, const unsigned char *enable);
    int (*enable_3a)(unsigned int device, unsigned int channel);

    /*
     * The AE's own envelope: shutter, aperture and the two gain ceilings. What
     * the shutter cap is written through, and the only readable statement of
     * what the tuning binary installed -- the plain long exposure table behind
     * it is a curve, this is its bounds.
     *
     * Optional and dlsym'd for the same reason as the CUS3A pair: a library
     * without them should cost the cap, not the pipeline. The Infinity6E backend
     * reaches the identical call through hal_symbol_load because there it is a
     * hard requirement of its gain-ceiling ops.
     */
    int (*get_expo_limit)(unsigned int device, unsigned int channel, i6c_isp_exp *limit);
    int (*set_expo_limit)(unsigned int device, unsigned int channel, i6c_isp_exp *limit);
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

    if (!(isp->get_chn_param = (int (*)(unsigned int, unsigned int, i6c_isp_para *))hal_symbol_load(
              "i6c_isp", isp->lib, "MI_ISP_GetChnParam")))
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

    isp->ae_status = (int (*)(unsigned int, unsigned int, i6c_cus_ae_info *))dlsym(
        isp->lib, "MI_ISP_CUS3A_GetAeStatus");
    isp->awb_status = (int (*)(unsigned int, unsigned int, i6c_cus_awb_info *))dlsym(
        isp->lib, "MI_ISP_CUS3A_GetAwbStatus");

    isp->cus3a_enable = (int (*)(unsigned int, unsigned int, const unsigned char *))dlsym(
        isp->lib, "MI_ISP_CUS3A_Enable");
    isp->enable_3a =
        (int (*)(unsigned int, unsigned int))dlsym(isp->lib, "MI_ISP_EnableUserspace3A");

    isp->get_expo_limit = (int (*)(unsigned int, unsigned int, i6c_isp_exp *))dlsym(
        isp->lib, "MI_ISP_AE_GetExposureLimit");
    isp->set_expo_limit = (int (*)(unsigned int, unsigned int, i6c_isp_exp *))dlsym(
        isp->lib, "MI_ISP_AE_SetExposureLimit");

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
