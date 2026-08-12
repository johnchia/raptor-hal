/*
 * infinity6c/i6c_scl_load.h -- dlopen loader for MI_SCL, MI 3.0
 *
 * SCL holds the scaling role that VPE has on MI 2.x, and it is the stage the
 * encoder channels hang off: one output port per stream, each scaled to that
 * stream's resolution, all fed from one channel. That is what makes a second
 * stream cheap here -- it is another port on an existing channel rather than
 * another pipeline.
 *
 * On the SoC id: MI_SCL does not take one as a separate argument the way MI_SYS
 * does. It rides in the high halfword of the device index, which is what
 * I6C_DEV_ID() in infinity6c_state.h composes.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_SCL_LOAD_H
#define INFINITY6C_I6C_SCL_LOAD_H

#include "hal_symbols.h"

#include "i6c_scl.h"

typedef struct {
    void *lib;

    /*
     * MI_SCL_DevAttr_t, whose one field is u32NeedUseHWOutPortMask: a bitmask of
     * the hardware scalers this device reserves, assigned to output ports in
     * ascending bit order (lowest set bit to port 0). Not a count, and not the
     * input sources the device accepts.
     */
    int (*create_dev)(unsigned int device, unsigned int *scalers);
    int (*destroy_dev)(unsigned int device);

    int (*create_chn)(unsigned int device, unsigned int channel, unsigned int *reserved);
    int (*destroy_chn)(unsigned int device, unsigned int channel);
    int (*start_chn)(unsigned int device, unsigned int channel);
    int (*stop_chn)(unsigned int device, unsigned int channel);

    /* Rotation, and the only thing the channel parameter carries. */
    int (*set_chn_param)(unsigned int device, unsigned int channel, i6c_scl_chn *config);

    int (*set_port_param)(unsigned int device, unsigned int channel, unsigned int port,
                          i6c_scl_port *config);
    int (*enable_port)(unsigned int device, unsigned int channel, unsigned int port);
    int (*disable_port)(unsigned int device, unsigned int channel, unsigned int port);
} i6c_scl_api;

static inline int i6c_scl_load(i6c_scl_api *scl)
{
    if (!(scl->lib = dlopen("libmi_scl.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_scl: dlopen(libmi_scl.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(scl->create_dev = (int (*)(unsigned int, unsigned int *))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_CreateDevice")))
        return RSS_ERR_NOTSUP;

    if (!(scl->destroy_dev =
              (int (*)(unsigned int))hal_symbol_load("i6c_scl", scl->lib, "MI_SCL_DestroyDevice")))
        return RSS_ERR_NOTSUP;

    if (!(scl->create_chn = (int (*)(unsigned int, unsigned int, unsigned int *))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_CreateChannel")))
        return RSS_ERR_NOTSUP;

    if (!(scl->destroy_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_DestroyChannel")))
        return RSS_ERR_NOTSUP;

    if (!(scl->start_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_StartChannel")))
        return RSS_ERR_NOTSUP;

    if (!(scl->stop_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_StopChannel")))
        return RSS_ERR_NOTSUP;

    if (!(scl->set_chn_param = (int (*)(unsigned int, unsigned int, i6c_scl_chn *))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_SetChnParam")))
        return RSS_ERR_NOTSUP;

    if (!(scl->set_port_param = (int (*)(unsigned int, unsigned int, unsigned int, i6c_scl_port *))
              hal_symbol_load("i6c_scl", scl->lib, "MI_SCL_SetOutputPortParam")))
        return RSS_ERR_NOTSUP;

    if (!(scl->enable_port = (int (*)(unsigned int, unsigned int, unsigned int))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_EnableOutputPort")))
        return RSS_ERR_NOTSUP;

    if (!(scl->disable_port = (int (*)(unsigned int, unsigned int, unsigned int))hal_symbol_load(
              "i6c_scl", scl->lib, "MI_SCL_DisableOutputPort")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_scl_unload(i6c_scl_api *scl)
{
    if (scl->lib)
        dlclose(scl->lib);
    memset(scl, 0, sizeof(*scl));
}

#endif /* INFINITY6C_I6C_SCL_LOAD_H */
