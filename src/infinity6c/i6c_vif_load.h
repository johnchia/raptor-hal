/*
 * infinity6c/i6c_vif_load.h -- dlopen loader for MI_VIF, MI 3.0
 *
 * The first stage of the datapath: it receives what the sensor sends and hands
 * frames to the ISP. Counterpart to star/i6_vif_load.h, and not a variation on
 * it -- this generation adds a group above the device, so a device cannot be
 * configured until the group it belongs to exists, and the port calls are
 * addressed by device and port where MI 2.x addresses them by channel and port.
 *
 * On the SoC id: MI_VIF does not take one as a separate argument the way MI_SYS
 * does. It rides in the high halfword of the device or group index, which is what
 * I6C_DEV_ID() in infinity6c_state.h composes.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_VIF_LOAD_H
#define INFINITY6C_I6C_VIF_LOAD_H

#include "hal_symbols.h"

#include "i6c_vif.h"

typedef struct {
    void *lib;

    /*
     * The group is what MI 2.x has no equivalent of. It carries the interface
     * mode, the clock edge and the HDR type -- everything about how the sensor is
     * wired, as opposed to what it sends -- and it must exist before the device
     * under it will accept an attribute.
     */
    int (*create_group)(unsigned int group, i6c_vif_grp *config);
    int (*destroy_group)(unsigned int group);

    int (*set_dev_attr)(unsigned int device, i6c_vif_dev *config);
    int (*enable_dev)(unsigned int device);
    int (*disable_dev)(unsigned int device);

    int (*set_port_attr)(unsigned int device, unsigned int port, i6c_vif_port *config);
    int (*enable_port)(unsigned int device, unsigned int port);
    int (*disable_port)(unsigned int device, unsigned int port);
} i6c_vif_api;

static inline int i6c_vif_load(i6c_vif_api *vif)
{
    if (!(vif->lib = dlopen("libmi_vif.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_vif: dlopen(libmi_vif.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(vif->create_group = (int (*)(unsigned int, i6c_vif_grp *))hal_symbol_load(
              "i6c_vif", vif->lib, "MI_VIF_CreateDevGroup")))
        return RSS_ERR_NOTSUP;

    if (!(vif->destroy_group = (int (*)(unsigned int))hal_symbol_load(
              "i6c_vif", vif->lib, "MI_VIF_DestroyDevGroup")))
        return RSS_ERR_NOTSUP;

    if (!(vif->set_dev_attr = (int (*)(unsigned int, i6c_vif_dev *))hal_symbol_load(
              "i6c_vif", vif->lib, "MI_VIF_SetDevAttr")))
        return RSS_ERR_NOTSUP;

    if (!(vif->enable_dev =
              (int (*)(unsigned int))hal_symbol_load("i6c_vif", vif->lib, "MI_VIF_EnableDev")))
        return RSS_ERR_NOTSUP;

    if (!(vif->disable_dev =
              (int (*)(unsigned int))hal_symbol_load("i6c_vif", vif->lib, "MI_VIF_DisableDev")))
        return RSS_ERR_NOTSUP;

    if (!(vif->set_port_attr =
              (int (*)(unsigned int, unsigned int, i6c_vif_port *))hal_symbol_load(
                  "i6c_vif", vif->lib, "MI_VIF_SetOutputPortAttr")))
        return RSS_ERR_NOTSUP;

    if (!(vif->enable_port = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_vif", vif->lib, "MI_VIF_EnableOutputPort")))
        return RSS_ERR_NOTSUP;

    if (!(vif->disable_port = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_vif", vif->lib, "MI_VIF_DisableOutputPort")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_vif_unload(i6c_vif_api *vif)
{
    if (vif->lib)
        dlclose(vif->lib);
    memset(vif, 0, sizeof(*vif));
}

#endif /* INFINITY6C_I6C_VIF_LOAD_H */
