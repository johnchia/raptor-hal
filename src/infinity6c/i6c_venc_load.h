/*
 * infinity6c/i6c_venc_load.h -- dlopen loader for MI_VENC, MI 3.0
 *
 * Counterpart to star/i6_venc_load.h, and the reason a shared table would be
 * unsafe rather than merely awkward: every entry point here leads with a device
 * that MI 2.x does not have. dlsym resolves by name, so the MI 2.x table binds
 * against these libraries without complaint and then calls with the channel where
 * the device belongs, shifting every argument by one. Nothing reports it.
 *
 * The device is not a topology index to be set to zero and forgotten, either -- it
 * selects the codec engine. H.26x and MJPEG live on different devices, so which
 * one a channel is created on follows from the codec that channel encodes.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_VENC_LOAD_H
#define INFINITY6C_I6C_VENC_LOAD_H

#include "hal_symbols.h"

#include "i6c_venc.h"

typedef struct {
    void *lib;

    int (*create_dev)(unsigned int device, i6c_venc_init *config);
    int (*destroy_dev)(unsigned int device);

    int (*create_chn)(unsigned int device, unsigned int channel, i6c_venc_chn *config);
    int (*destroy_chn)(unsigned int device, unsigned int channel);
    int (*get_chn_attr)(unsigned int device, unsigned int channel, i6c_venc_chn *config);
    int (*set_chn_attr)(unsigned int device, unsigned int channel, i6c_venc_chn *config);
    int (*reset_chn)(unsigned int device, unsigned int channel);

    /*
     * How the encoder takes frames from SCL. The ring mode is what lets the
     * encoder start on a partial frame rather than waiting for a whole one, and it
     * is only valid on the H.26x device.
     */
    int (*set_src_conf)(unsigned int device, unsigned int channel, i6c_venc_src_conf *config);

    int (*start_recv)(unsigned int device, unsigned int channel);
    /* Bounded variant: encode exactly count frames, used for a single snapshot. */
    int (*start_recv_ex)(unsigned int device, unsigned int channel, int *count);
    int (*stop_recv)(unsigned int device, unsigned int channel);

    /*
     * The stream side. get_fd returns a descriptor that becomes readable when a
     * frame is ready, which is what lets one thread wait on several channels
     * rather than polling each.
     */
    int (*get_fd)(unsigned int device, unsigned int channel);
    int (*close_fd)(unsigned int device, unsigned int channel);
    int (*query)(unsigned int device, unsigned int channel, i6c_venc_stat *stat);
    int (*get_stream)(unsigned int device, unsigned int channel, i6c_venc_strm *stream,
                      unsigned int timeout);
    int (*release_stream)(unsigned int device, unsigned int channel, i6c_venc_strm *stream);

    int (*request_idr)(unsigned int device, unsigned int channel, char instant);

    int (*get_jpeg_param)(unsigned int device, unsigned int channel, i6c_venc_jpg *param);
    int (*set_jpeg_param)(unsigned int device, unsigned int channel, i6c_venc_jpg *param);
} i6c_venc_api;

static inline int i6c_venc_load(i6c_venc_api *venc)
{
    if (!(venc->lib = dlopen("libmi_venc.so", RTLD_LAZY | RTLD_GLOBAL))) {
        HAL_LOG_ERR("i6c_venc: dlopen(libmi_venc.so) failed: %s", dlerror());
        return RSS_ERR_NOENT;
    }

    if (!(venc->create_dev = (int (*)(unsigned int, i6c_venc_init *))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_CreateDev")))
        return RSS_ERR_NOTSUP;

    if (!(venc->destroy_dev = (int (*)(unsigned int))hal_symbol_load("i6c_venc", venc->lib,
                                                                     "MI_VENC_DestroyDev")))
        return RSS_ERR_NOTSUP;

    if (!(venc->create_chn = (int (*)(unsigned int, unsigned int, i6c_venc_chn *))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_CreateChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc->destroy_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_DestroyChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc->get_chn_attr =
              (int (*)(unsigned int, unsigned int, i6c_venc_chn *))hal_symbol_load(
                  "i6c_venc", venc->lib, "MI_VENC_GetChnAttr")))
        return RSS_ERR_NOTSUP;

    if (!(venc->set_chn_attr =
              (int (*)(unsigned int, unsigned int, i6c_venc_chn *))hal_symbol_load(
                  "i6c_venc", venc->lib, "MI_VENC_SetChnAttr")))
        return RSS_ERR_NOTSUP;

    if (!(venc->reset_chn = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_ResetChn")))
        return RSS_ERR_NOTSUP;

    if (!(venc->set_src_conf =
              (int (*)(unsigned int, unsigned int, i6c_venc_src_conf *))hal_symbol_load(
                  "i6c_venc", venc->lib, "MI_VENC_SetInputSourceConfig")))
        return RSS_ERR_NOTSUP;

    if (!(venc->start_recv = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_StartRecvPic")))
        return RSS_ERR_NOTSUP;

    if (!(venc->start_recv_ex = (int (*)(unsigned int, unsigned int, int *))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_StartRecvPicEx")))
        return RSS_ERR_NOTSUP;

    if (!(venc->stop_recv = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_StopRecvPic")))
        return RSS_ERR_NOTSUP;

    if (!(venc->get_fd = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_GetFd")))
        return RSS_ERR_NOTSUP;

    if (!(venc->close_fd = (int (*)(unsigned int, unsigned int))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_CloseFd")))
        return RSS_ERR_NOTSUP;

    if (!(venc->query = (int (*)(unsigned int, unsigned int, i6c_venc_stat *))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_Query")))
        return RSS_ERR_NOTSUP;

    if (!(venc->get_stream =
              (int (*)(unsigned int, unsigned int, i6c_venc_strm *, unsigned int))hal_symbol_load(
                  "i6c_venc", venc->lib, "MI_VENC_GetStream")))
        return RSS_ERR_NOTSUP;

    if (!(venc->release_stream =
              (int (*)(unsigned int, unsigned int, i6c_venc_strm *))hal_symbol_load(
                  "i6c_venc", venc->lib, "MI_VENC_ReleaseStream")))
        return RSS_ERR_NOTSUP;

    if (!(venc->request_idr = (int (*)(unsigned int, unsigned int, char))hal_symbol_load(
              "i6c_venc", venc->lib, "MI_VENC_RequestIdr")))
        return RSS_ERR_NOTSUP;

    if (!(venc->get_jpeg_param =
              (int (*)(unsigned int, unsigned int, i6c_venc_jpg *))hal_symbol_load(
                  "i6c_venc", venc->lib, "MI_VENC_GetJpegParam")))
        return RSS_ERR_NOTSUP;

    if (!(venc->set_jpeg_param =
              (int (*)(unsigned int, unsigned int, i6c_venc_jpg *))hal_symbol_load(
                  "i6c_venc", venc->lib, "MI_VENC_SetJpegParam")))
        return RSS_ERR_NOTSUP;

    return RSS_OK;
}

static inline void i6c_venc_unload(i6c_venc_api *venc)
{
    if (venc->lib)
        dlclose(venc->lib);
    memset(venc, 0, sizeof(*venc));
}

#endif /* INFINITY6C_I6C_VENC_LOAD_H */
