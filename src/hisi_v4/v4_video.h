/*
 * hisi_v4/v4_video.h -- picture and frame types shared across HiMPP modules
 *
 * hi_comm_video.h's half of the ABI: the geometry primitives, the pixel and
 * compression enums, and VIDEO_FRAME_INFO_S. Separated from v4_common.h
 * because v4_vi.h, v4_vpss.h and v4_venc.h all need it and v4_sys.h does
 * not -- the SYS/VB lifecycle never sees a picture.
 *
 * PROVENANCE. Derived from the Hi3516EV200 SDK V1.0.1.0 headers
 * (mpp/include/hi_comm_video.h) and cross-checked against
 * ref/openhisilicon/include/comm_video.h (GPL v3). Offsets from a probe
 * compiled with arm-openipc-linux-musleabi-gcc.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_VIDEO_H
#define HISI_V4_VIDEO_H

#include "v4_common.h"

/* ================================================================
 * GEOMETRY
 * ================================================================ */

typedef struct {
    unsigned int width;
    unsigned int height;
} v4_size;

_Static_assert(sizeof(v4_size) == 8, "SIZE_S is 8 bytes");

typedef struct {
    int x;
    int y;
    unsigned int width;
    unsigned int height;
} v4_rect;

_Static_assert(sizeof(v4_rect) == 16, "RECT_S is 16 bytes");

/*
 * FRAME_RATE_CTRL_S. Both members are signed and -1 means "no control",
 * which is not the same as 0: zero asks the hardware for no frames at all.
 * Every producer in this backend either sets both or sets both to -1.
 */
typedef struct {
    int src_frame_rate;
    int dst_frame_rate;
} v4_frame_rate;

_Static_assert(sizeof(v4_frame_rate) == 8, "FRAME_RATE_CTRL_S is 8 bytes");

typedef struct {
    unsigned int mode;
    unsigned int bg_color;
    v4_rect video_rect;
} v4_aspect_ratio;

_Static_assert(sizeof(v4_aspect_ratio) == 24, "ASPECT_RATIO_S is 24 bytes");

/* ================================================================
 * PICTURE FORMAT ENUMS
 * ================================================================ */

/*
 * PIXEL_FORMAT_E, the members this backend can produce.
 *
 * The values matter more here than anywhere else in the ABI, because the
 * enum has 59 members and the ones raptor wants sit deep in it. Read from
 * the SDK headers by a compiled probe, not counted: RGB_BAYER_8BPP is 17,
 * so the Bayer run is 17..21 by bit depth, and YVU_SEMIPLANAR_420 -- the
 * only format the encoder takes -- is 26.
 */
typedef enum {
    V4_PIXEL_FORMAT_RGB_BAYER_8BPP = 17,
    V4_PIXEL_FORMAT_RGB_BAYER_10BPP = 18,
    V4_PIXEL_FORMAT_RGB_BAYER_12BPP = 19,
    V4_PIXEL_FORMAT_RGB_BAYER_14BPP = 20,
    V4_PIXEL_FORMAT_RGB_BAYER_16BPP = 21,
    V4_PIXEL_FORMAT_YVU_SEMIPLANAR_420 = 26,
} v4_pixel_format;

/* DATA_BITWIDTH_E */
typedef enum {
    V4_DATA_BITWIDTH_8 = 0,
    V4_DATA_BITWIDTH_10 = 1,
    V4_DATA_BITWIDTH_12 = 2,
    V4_DATA_BITWIDTH_14 = 3,
    V4_DATA_BITWIDTH_16 = 4,
} v4_data_bitwidth;

/* DYNAMIC_RANGE_E */
typedef enum {
    V4_DYNAMIC_RANGE_SDR8 = 0,
    V4_DYNAMIC_RANGE_SDR10 = 1,
} v4_dynamic_range;

/* VIDEO_FORMAT_E */
typedef enum {
    V4_VIDEO_FORMAT_LINEAR = 0,
} v4_video_format;

/* COMPRESS_MODE_E. SEG is the 256-byte-segment reference-frame compression
 * the VI pipe and VPSS group use to save DDR bandwidth; NONE is what
 * anything userspace reads must be. */
typedef enum {
    V4_COMPRESS_MODE_NONE = 0,
    V4_COMPRESS_MODE_SEG = 1,
} v4_compress_mode;

/* ISP_BAYER_FORMAT_E. The order the sensor's colour filter array puts the
 * first two pixels of the first two rows in. Comes from the sensor INI's
 * Isp_Bayer key; getting it wrong swaps red and blue. */
typedef enum {
    V4_BAYER_RGGB = 0,
    V4_BAYER_GRBG = 1,
    V4_BAYER_GBRG = 2,
    V4_BAYER_BGGR = 3,
} v4_bayer_format;

/* WDR_MODE_E. gen4 supports more, but every stock sensor INI on the board
 * is WDR_MODE_NONE and raptor has no WDR configuration to drive the rest
 * from. Named rather than assumed so the INI parser can reject a mode this
 * backend would silently mishandle. */
typedef enum {
    V4_WDR_MODE_NONE = 0,
    V4_WDR_MODE_BUILT_IN = 1,
    V4_WDR_MODE_QUDRA = 2,
    V4_WDR_MODE_2To1_LINE = 3,
    V4_WDR_MODE_2To1_FRAME = 4,
} v4_wdr_mode;

/* ================================================================
 * FRAMES
 * ================================================================ */

/*
 * VIDEO_SUPPLEMENT_S -- the per-frame side-channel (JPEG DCF, ISP
 * statistics, motion data), 64 bytes, carried opaquely.
 *
 * Opaque rather than transcribed because nothing in raptor reads it and
 * every member is a pointer or a physical address whose meaning depends on
 * flags set through calls this backend does not make. Transcribing a struct
 * to never look at it buys a chance to get it wrong and nothing else. The
 * size is asserted, which is all the layout below actually depends on.
 */
typedef struct {
    unsigned char opaque[64];
} v4_video_supplement;

_Static_assert(sizeof(v4_video_supplement) == 64, "VIDEO_SUPPLEMENT_S is 64 bytes");

/*
 * VIDEO_FRAME_S.
 *
 * Three-plane arrays throughout even though every format this backend
 * produces uses two, because the array bound is ABI. Note the padding the
 * asserts pin: u32ExtStride ends at +68 and u64HeaderPhyAddr starts at +72,
 * so there is a 4-byte hole that a member-by-member reading of the header
 * does not show.
 */
typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int field;
    v4_pixel_format pixel_format;
    v4_video_format video_format;
    v4_compress_mode compress_mode;
    v4_dynamic_range dynamic_range;
    unsigned int color_gamut;

    unsigned int header_stride[3];
    unsigned int stride[3];
    unsigned int ext_stride[3];

    unsigned long long header_phy_addr[3];
    unsigned long long header_vir_addr[3];
    unsigned long long phy_addr[3];
    unsigned long long vir_addr[3];
    unsigned long long ext_phy_addr[3];
    unsigned long long ext_vir_addr[3];

    short offset_top;
    short offset_bottom;
    short offset_left;
    short offset_right;

    unsigned int max_luminance;
    unsigned int min_luminance;

    unsigned int time_ref;
    unsigned long long pts;

    unsigned long long private_data;
    unsigned int frame_flag;
    v4_video_supplement supplement;
} v4_video_frame;

_Static_assert(sizeof(v4_video_frame) == 328, "VIDEO_FRAME_S is 328 bytes");
_Static_assert(offsetof(v4_video_frame, stride) == 44, "VIDEO_FRAME_S.u32Stride at +44");
_Static_assert(offsetof(v4_video_frame, phy_addr) == 120, "u64PhyAddr at +120");
_Static_assert(offsetof(v4_video_frame, vir_addr) == 144, "u64VirAddr at +144");
_Static_assert(offsetof(v4_video_frame, time_ref) == 232, "u32TimeRef at +232");
_Static_assert(offsetof(v4_video_frame, pts) == 240, "u64PTS at +240");

typedef struct {
    v4_video_frame frame;
    unsigned int pool_id;
    v4_mod_id mod_id;
} v4_video_frame_info;

_Static_assert(sizeof(v4_video_frame_info) == 336, "VIDEO_FRAME_INFO_S is 336 bytes");
_Static_assert(offsetof(v4_video_frame_info, pool_id) == 328, "u32PoolId at +328");

#endif /* HISI_V4_VIDEO_H */
