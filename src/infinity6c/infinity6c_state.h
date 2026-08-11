/*
 * infinity6c/infinity6c_state.h -- shared backend state for the SigmaStar MI 3.0 HAL
 *
 * Counterpart to star/star_state.h. Separate for the same reason that header
 * gives: it cannot live in hal_internal.h, because the ABI headers include
 * that one for HAL_LOG_* and RSS_ERR_* and must not be included back.
 *
 * The datapath is
 *
 *   SNR -> VIF -> ISP -> SCL -> VENC
 *
 * where MI 2.x is VIF -> VPE -> VENC with the ISP folded into VPE. VIF gains a
 * group above the device, the ISP becomes a stage with its own device, channel
 * and ports, SCL holds the scaling role, and VENC gains a device above the
 * channel -- one that selects the codec engine rather than being a topology
 * index.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_STATE_H
#define INFINITY6C_STATE_H

#include "hal_internal.h"

#include "i6c_isp_load.h"
#include "i6c_scl_load.h"
#include "i6c_snr_load.h"
#include "i6c_sys_load.h"
#include "i6c_venc_load.h"
#include "i6c_vif_load.h"

/* ================================================================
 * FIXED TOPOLOGY
 *
 * One sensor, so every device and group index is 0. Named rather than
 * written as literals because MI 3.0 threads more of them through each
 * call than MI 2.x does, and at a call site a bare 0 does not say
 * whether it is a SoC, a device, a group, a channel or a port.
 * ================================================================ */

/*
 * The die MI_SYS and MI_RGN calls are addressed to. Zero on a single-die
 * camera, which is what the vendor's own reference passes. It leads this list
 * because it is the index MI 2.x has no equivalent of, and it is named rather
 * than written as a bare 0 at each call site because a literal there reads
 * like a device or a channel, and it is neither.
 */
#define I6C_SOC_ID 0

#define I6C_SNR_PAD 0
#define I6C_VIF_GRP 0
#define I6C_VIF_DEV 0
#define I6C_VIF_PORT 0
#define I6C_ISP_DEV 0
#define I6C_ISP_CHN 0
#define I6C_ISP_PORT 0
#define I6C_SCL_DEV 0
#define I6C_SCL_CHN 0

/*
 * Only MI_SYS and MI_RGN take the SoC id as a distinct leading argument. VIF,
 * SNR, ISP and SCL pack it into the high halfword of the device or pad index
 * instead, and the wrapper shifts it back out -- MI_ISP_EnableOutputPort opens
 * with `lsr #16` on its first argument.
 *
 * On a single-die part both halves are zero, so passing a bare index happens to
 * work and the distinction is invisible. It is composed explicitly anyway: the
 * cost is nothing and the alternative is a second die's worth of debugging for
 * whoever meets one.
 */
#define I6C_DEV_ID(dev) (((unsigned int)I6C_SOC_ID << 16) | (unsigned int)(dev))

/*
 * Streams. Four rather than the twelve VENC channels MI allows, because each
 * one is an SCL output port and the scaler has four -- so four is the real
 * ceiling on simultaneous streams, whatever VENC would accept.
 */
#define I6C_MAX_CHN 4

/*
 * Packs per frame handled without allocating. One frame arrives as several
 * packs -- roughly one per NAL -- and this covers a keyframe with its parameter
 * sets. Beyond it the pack array grows on the heap; nals[] does not, matching
 * rvd's own ceiling.
 */
#define I6C_VENC_MAX_PACKS 8

/*
 * Codec engines a channel can live on: one H.26x, one MJPEG. Not a channel count
 * -- the channel index space is shared between them -- but the number of ring
 * pools, since a pool belongs to a device.
 */
#define I6C_VENC_DEV_SLOTS 2

/*
 * Elements in a vendor array, for bounding a loop by the declaration rather than
 * by a literal that matches it today. The packs-within-a-pack array is the one
 * that matters: its length is the vendor's to change, and the loop over it is
 * driven by a count the vendor also supplies.
 */
#define I6C_ARRAY_LEN(a) (unsigned int)(sizeof(a) / sizeof((a)[0]))

/* ================================================================
 * PER-CHANNEL STATE
 * ================================================================ */

/*
 * A framesource channel is an SCL output port. It carries no MI object of its
 * own -- the port exists as soon as the channel it hangs off is started -- so
 * this is the geometry the port was configured with plus whether it is running.
 */
typedef struct {
    bool configured;
    bool enabled;
    unsigned short width;
    unsigned short height;
    i6c_common_pixfmt pixfmt;
} infinity6c_fs_chn_t;

typedef struct {
    bool created;
    bool receiving;
    bool bound;

    /*
     * Which codec engine this channel lives on. Not a topology index: H.26x and
     * MJPEG are different devices, so this follows from the codec.
     */
    unsigned int device;
    rss_codec_t codec;
    unsigned short width;
    unsigned short height;

    /*
     * The descriptor MI_VENC_GetFd hands back, cached because closing and
     * reopening it per frame would be a syscall pair per frame for nothing.
     * -1 when not held.
     */
    int fd;

    /* One outstanding frame at a time, matching raptor's get/release contract. */
    bool frame_held;
    i6c_venc_strm strm;
    i6c_venc_pack packs[I6C_VENC_MAX_PACKS];
    i6c_venc_pack *heap_packs;
    unsigned int heap_count;

    /*
     * How many packs the array handed to MI_VENC_GetStream can hold. Kept because
     * the count comes back through the same struct it went out in: the array is
     * sized from a preceding MI_VENC_Query, and reading the returned count
     * without bounding it by this trusts a vendor library not to answer with more
     * than it was given room for.
     */
    unsigned int pack_cap;
    rss_nal_unit_t nals[I6C_VENC_MAX_PACKS];

    /*
     * The requested rate settings, kept because MI exposes no per-knob setter:
     * bitrate, GOP and frame rate all live in the rate half of the channel
     * attribute, which is read, modified and written back whole.
     */
    rss_video_config_t cfg;
} infinity6c_venc_chn_t;

/* ================================================================
 * BACKEND STATE
 * ================================================================ */

typedef struct {
    i6c_sys_api sys;
    i6c_snr_api snr;
    i6c_vif_api vif;
    i6c_isp_api isp;
    i6c_scl_api scl;
    i6c_venc_api venc;

    bool sys_inited; /* MI_SYS_Init succeeded, so MI_SYS_Exit is owed */

    /*
     * The pipeline is brought up once and shared by every channel, because it
     * is one sensor feeding one SCL channel whose ports are the streams. So the
     * first framesource channel to be created builds it and the last one to be
     * destroyed tears it down, rather than either happening at init.
     */
    bool pipeline_up;
    unsigned int pipeline_refs;

    /*
     * What the sensor said about itself. Read once at bring-up and kept, since
     * VIF's pixel format, the ISP's yuv2bayer decision and the pool geometry
     * are all derived from it rather than configured.
     */
    i6c_snr_pad pad;
    i6c_snr_plane plane;
    int snr_profile; /* index into the sensor's resolution list; -1 = unset */
    unsigned int fps;

    /*
     * The geometry each VENC device's ring pool was configured for. Kept because
     * the pool is per device and not per channel -- i6c_sys_poolring names a
     * module and a device and nothing finer -- so several channels on one engine
     * share it and the largest of them is what has to fit.
     */
    unsigned short enc_pool_w[I6C_VENC_DEV_SLOTS];
    unsigned short enc_pool_h[I6C_VENC_DEV_SLOTS];

    infinity6c_fs_chn_t fs[I6C_MAX_CHN];
    infinity6c_venc_chn_t enc[I6C_MAX_CHN];
} infinity6c_state_t;

/* ================================================================
 * SHARED HELPERS
 * ================================================================ */

/*
 * Guard boilerplate for a per-channel entry point. Every fs_* and enc_* op
 * begins by proving the context exists, the backend is initialised and the
 * channel index is in range, and there is no useful variation between them.
 */
#define I6C_ENTER(ctx, chn, st_var)                                                                \
    infinity6c_state_t *st_var;                                                                    \
    do {                                                                                           \
        rss_hal_ctx_t *_hal = (rss_hal_ctx_t *)(ctx);                                              \
        if (!_hal)                                                                                 \
            return RSS_ERR_INVAL;                                                                  \
        (st_var) = (infinity6c_state_t *)_hal->platform;                                           \
        if (!(st_var))                                                                             \
            return RSS_ERR_NOTSUP;                                                                 \
        if ((chn) < 0 || (chn) >= I6C_MAX_CHN)                                                     \
            return RSS_ERR_INVAL;                                                                  \
    } while (0)

/* Pipeline bring-up and teardown (hal_framesource.c). */
int i6c_pipeline_create(infinity6c_state_t *st, const rss_fs_config_t *cfg);
void i6c_pipeline_destroy(infinity6c_state_t *st);

/* Bind and unbind one SCL output port to its encoder channel (hal_encoder.c). */
int i6c_bind_scl_to_venc(infinity6c_state_t *st, int chn);
int i6c_unbind_scl_from_venc(infinity6c_state_t *st, int chn);

/* Release every channel's MI object, in dependency order (hal_common.c). */
void i6c_teardown_all(infinity6c_state_t *st);

#endif /* INFINITY6C_STATE_H */
