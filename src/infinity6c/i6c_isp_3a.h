/*
 * infinity6c/i6c_isp_3a.h -- CUS3A live AE/AWB status layouts, MI 3.0
 *
 * What MI_ISP_CUS3A_GetAeStatus and MI_ISP_CUS3A_GetAwbStatus write back: the
 * running algorithm's own state, not the MI API tuning layer. The distinction
 * matters and cost a board round to learn -- the MI_ISP_AE_* / MI_ISP_AWB_*
 * query and attribute calls report the API-set values, which sit at their
 * defaults while CUS3A owns 3A, so they read back zero exposure and a frozen,
 * out-of-range white balance. The live numbers are here, the same surface the
 * MI 2.x backend reads through its sensor-driver callback.
 *
 * The layouts are the maruko (i6c) CusAEInfo_t and CusAWBInfo_t, packed. Both
 * sizes are confirmed twice over: by the packed field arithmetic below, and by
 * the copy length the library's own wrapper hands to MI_ISP_GENERAL_GetIspApiData
 * (87 for AE, 63 for AWB -- read off the ioctl descriptor, per DERIVED.md). Only
 * the read fields are named; the rest is a reserved span so the struct is the
 * size the wrapper copies into it. Pointer members are carried as 32-bit words:
 * the target is 32-bit ARM, and keeping them integer-sized also makes the size
 * asserts hold when this header is compiled on a 64-bit host for checking.
 *
 * The AE shutter is microseconds despite the vendor comment saying nanoseconds
 * -- the same correction the MI 2.x backend makes, confirmed there on hardware.
 * Gains are 1024-per-unit.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_ISP_3A_H
#define INFINITY6C_I6C_ISP_3A_H

#include <stddef.h>
#include <stdint.h>

/*
 * CusAEInfo_t -- MI_ISP_CUS3A_GetAeStatus. The long-exposure shutter and gains
 * lead the struct; preAvgY is the frame brightness the day/night poll reads as
 * luma. The HDR-short mirror, aperture, fps and channel-sync tail are carried
 * but not read.
 */
typedef struct __attribute__((packed)) {
    uint8_t reserved0[24]; /* size, two histogram ptrs, avg block dims, avg ptr */
    uint32_t shutterUs;    /* @24 */
    uint32_t sensorGain;   /* @28, 1024 = 1x */
    uint32_t ispGain;      /* @32, 1024 = 1x */
    uint8_t reserved1[12]; /* @36: HDR-short shutter and gains */
    uint32_t preAvgY;      /* @48, previous-frame brightness */
    uint8_t reserved2[35]; /* @52: HDR ctl, aperture, fps, weightY, sync3A tail */
} i6c_cus_ae_info;

/*
 * CusAWBInfo_t -- MI_ISP_CUS3A_GetAwbStatus. The three white-balance channel
 * gains the algorithm has settled on. One green gain, not two: a per-channel
 * Gr/Gb split does not exist at this layer, so a green cast is not a Gr/Gb
 * imbalance read from here. No colour-temperature field -- that lives on the
 * result struct, not the status one.
 */
typedef struct __attribute__((packed)) {
    uint8_t reserved0[12]; /* size, avg block dims */
    uint32_t rGain;        /* @12, 1024 = 1x */
    uint32_t gGain;        /* @16 */
    uint32_t bGain;        /* @20 */
    uint8_t reserved1[39]; /* @24: avg ptr, HDR, BV, weightY, sync3A tail */
} i6c_cus_awb_info;

_Static_assert(sizeof(i6c_cus_ae_info) == 87, "CusAEInfo_t, and the wrapper copies 87");
_Static_assert(offsetof(i6c_cus_ae_info, shutterUs) == 24, "after the stats header");
_Static_assert(offsetof(i6c_cus_ae_info, sensorGain) == 28, "one word past the shutter");
_Static_assert(offsetof(i6c_cus_ae_info, ispGain) == 32, "one word past the sensor gain");
_Static_assert(offsetof(i6c_cus_ae_info, preAvgY) == 48, "past the HDR-short mirror");

_Static_assert(sizeof(i6c_cus_awb_info) == 63, "CusAWBInfo_t, and the wrapper copies 63");
_Static_assert(offsetof(i6c_cus_awb_info, rGain) == 12, "after size and the block dims");
_Static_assert(offsetof(i6c_cus_awb_info, gGain) == 16, "one word past the R gain");
_Static_assert(offsetof(i6c_cus_awb_info, bGain) == 20, "one word past the G gain");

#endif /* INFINITY6C_I6C_ISP_3A_H */
