/*
 * infinity6c/i6c_isp_3a.h -- MI_ISP 3A readback/attribute layouts, MI 3.0
 *
 * The AE and AWB query/attribute structs the tuning surface reads and writes.
 * They are not in the pipeline header (sigmastar-headers' i6c_isp.h) because
 * they belong to the 3A libraries, libmi_isp.so's AE/AWB entry points, rather
 * than to the ISP pipeline stage. They live here, in the backend, until the
 * layouts have been checked on hardware; once verified they move to
 * sigmastar-headers alongside the pipeline structs, with a pin bump.
 *
 * Each layout is described by its ABI -- total size and the offset of every
 * field a caller touches -- rather than copied from a vendor header, matching
 * how the rest of the family was derived (see sigmastar-headers' DERIVED.md).
 * A query call writes the whole struct, so the size has to be right even where
 * only a few fields are read: the members that are not modelled are carried as
 * a named reserved span, and a _Static_assert pins both the size and the read
 * offsets so a hand layout cannot drift. Field meanings follow the public MI
 * ISP API; gains are the AWB block's Bayer-channel multipliers and shutter is
 * microseconds, as everywhere else in this backend.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_I6C_ISP_3A_H
#define INFINITY6C_I6C_ISP_3A_H

#include <stddef.h>

/*
 * AE -- MI_ISP_AE_QueryExposureInfo. One exposure value per HDR leg; the long
 * leg is the only one a non-HDR pipeline drives. lvX10 is the scene light
 * value times ten, which is what a day/night decision keys on. The luma
 * histogram in the middle is what fixes the size at 572 and pushes lvX10 out
 * to 560; it is carried but not read.
 */
typedef struct {
    unsigned int fNx10;
    unsigned int sensorGain;
    unsigned int ispGain;
    unsigned int us;
} i6c_isp_expo_value;

typedef struct {
    unsigned int lumY;
    unsigned int avgY; /* scene average luma -- what day/night keys on */
    unsigned int hits[128];
} i6c_isp_hist_wgt_y;

typedef struct {
    int stable;        /* AE converged */
    int reachBoundary; /* exposure pinned at a limit */
    i6c_isp_expo_value expoLong;
    i6c_isp_expo_value expoShort;
    i6c_isp_hist_wgt_y histWeightY;
    unsigned int lvX10;
    int bv;
    unsigned int sceneTarget;
} i6c_isp_ae_info;

/*
 * AWB -- MI_ISP_AWB_QueryInfo. The live white-balance estimate: per-channel
 * gains and the colour temperature the algorithm has settled on. If the
 * illuminant changes and colorTemp and the gains do not move, AWB is not
 * adapting. Bayer-channel order R, Gr, Gb, B.
 */
typedef struct {
    int stable;
    unsigned short rGain;
    unsigned short grGain;
    unsigned short gbGain;
    unsigned short bGain;
    unsigned short colorTemp;
    unsigned char wpInd;
    int multiLSDetected;
    unsigned char firstLSInd;
    unsigned char secondLSInd;
} i6c_isp_awb_info;

/*
 * AWB -- MI_ISP_AWB_GetAttr/SetAttr. Only the mode words at the front are
 * modelled: state (running vs paused) and op type (auto vs manual). The manual
 * gains and the auto-mode calibration tables that follow are a large span this
 * phase does not touch; they are reserved so the struct is the size the driver
 * writes. A caller must clear the struct before a Set so the reserved span is
 * not filled from the stack.
 */
#define I6C_AWB_STATE_NORMAL 0
#define I6C_AWB_STATE_PAUSE 1
#define I6C_AWB_OP_AUTO 0
#define I6C_AWB_OP_MANUAL 1

typedef struct {
    int state;  /* I6C_AWB_STATE_* */
    int opType; /* I6C_AWB_OP_* */
    unsigned char reserved[1456];
} i6c_isp_awb_attr;

_Static_assert(sizeof(i6c_isp_expo_value) == 16, "MI_ISP_AE_ExpoValueType_t");
_Static_assert(sizeof(i6c_isp_hist_wgt_y) == 520, "lumY, avgY, 128-bin histogram");
_Static_assert(sizeof(i6c_isp_ae_info) == 572, "MI_ISP_AE_QueryExposureInfo payload");
_Static_assert(offsetof(i6c_isp_ae_info, expoLong) == 8, "after the two bools");
_Static_assert(offsetof(i6c_isp_ae_info, lvX10) == 560, "past the histogram");
_Static_assert(offsetof(i6c_isp_ae_info, sceneTarget) == 568, "the last word");

_Static_assert(sizeof(i6c_isp_awb_info) == 24, "MI_ISP_AWB_QueryInfo payload");
_Static_assert(offsetof(i6c_isp_awb_info, rGain) == 4, "after the stable word");
_Static_assert(offsetof(i6c_isp_awb_info, colorTemp) == 12, "after the four gains");
_Static_assert(offsetof(i6c_isp_awb_info, multiLSDetected) == 16, "a word, past a byte of pad");

_Static_assert(sizeof(i6c_isp_awb_attr) == 1464, "MI_ISP_AWB_AttrType_t");
_Static_assert(offsetof(i6c_isp_awb_attr, opType) == 4, "after the state word");

#endif /* INFINITY6C_I6C_ISP_3A_H */
