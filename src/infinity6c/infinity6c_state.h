/*
 * infinity6c/infinity6c_state.h -- shared backend state for the SigmaStar MI 3.0 HAL
 *
 * Counterpart to star/star_state.h. Separate for the same reason that header
 * gives: it cannot live in hal_internal.h, because the ABI headers include
 * that one for HAL_LOG_* and RSS_ERR_* and must not be included back.
 *
 * The topology constants below are the ones MI 3.0 adds to the MI 2.x set,
 * and they are recorded here even where nothing uses them yet, because the
 * datapath they describe is the part of this port that differs most:
 *
 *   VIF -> ISP -> SCL -> VENC
 *
 * where MI 2.x is VIF -> VPE -> VENC with the ISP folded into VPE. VIF gains
 * a group above the device, the ISP becomes a stage with its own device,
 * channel and ports, SCL holds the scaling role, and VENC gains a device
 * above the channel.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_STATE_H
#define INFINITY6C_STATE_H

#include "hal_internal.h"

#include "i6c_sys_load.h"

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

#define I6C_SNR_INDEX 0
#define I6C_VIF_GRP 0
#define I6C_VIF_DEV 0
#define I6C_VIF_PORT 0
#define I6C_ISP_DEV 0
#define I6C_ISP_CHN 0
#define I6C_ISP_PORT 0
#define I6C_SCL_DEV 0
#define I6C_SCL_CHN 0
#define I6C_VENC_DEV 0

/*
 * MI backend state, hung off rss_hal_ctx_t->platform.
 *
 * Only the module handles exist so far. Everything the pipeline will need --
 * sensor descriptors, port geometry, channel bookkeeping -- arrives with the
 * subsystem that owns it, so that an unimplemented stage has no state to be
 * stale.
 */
typedef struct {
    i6c_sys_api sys;
    bool sys_inited; /* MI_SYS_Init succeeded, so MI_SYS_Exit is owed */
} infinity6c_state_t;

#endif /* INFINITY6C_STATE_H */
