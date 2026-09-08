/*
 * hisi_v5/v5_video.h -- the video types VI, VPSS, VENC and the ISP share
 *
 * ot_common_video.h's job, reduced to what this backend touches. Every
 * attribute struct in the three pipeline modules embeds most of these, so
 * they cannot live in whichever module happened to need them first.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_video.h at
 * 1.0.2.0 B051:
 *
 *   ot_op_mode           :47-51      ot_aspect_ratio_type :64-69
 *   ot_video_field       :71-78      ot_video_format      :80-87
 *   ot_compress_mode     :89-99      ot_pixel_format      :~101-200
 *   ot_dynamic_range     :~202-210   ot_frame_rate_ctrl   (struct, 8 bytes)
 *   ot_coord             :266-270    ot_border            (struct, 20 bytes)
 *   ot_aspect_ratio      (struct, 24) ot_wdr_mode         (enum)
 *   ot_video_supplement  (struct, 64) ot_video_frame      (struct, 184)
 *   ot_video_frame_info  (struct, 192)
 *
 * Sizes measured with the cv6xx cross-compiler against those headers, as
 * everywhere in this backend.
 *
 * THE ENUM TRANSCRIPTION RULE. ot_pixel_format has ~80 enumerators and this
 * file lists nine. That is deliberate and it is the rule the whole backend
 * follows: an enumerator is transcribed when this backend can produce or
 * consume it, and its *value* is written explicitly so a reader can check
 * one line against the header rather than counting a list. A format raptor
 * cannot handle is better absent than present-and-untested -- the numbers
 * are ABI, so a wrong one is a silently misconfigured pipeline rather than
 * a compile error.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_VIDEO_H
#define HISI_V5_VIDEO_H

#include "v5_common.h"

/* ================================================================
 * PIXEL AND FRAME FORMAT
 * ================================================================ */

/*
 * ot_pixel_format, the nine this backend names.
 *
 * The pipeline runs in OT_PIXEL_FORMAT_YVU_SEMIPLANAR_420 (NV21 ordering:
 * Y plane then interleaved VU), which is what the vendor sample configures
 * on every stage and what VENC consumes. The Bayer formats are what a VI
 * pipe carries between the sensor and the ISP and are selected from the
 * sensor INI's raw bit depth. ARGB1555 is the OSD's (Phase 5) and is here
 * because it is the same enum.
 *
 * Values counted off the header's enumerator list and then read back from a
 * compiled probe rather than counted twice.
 */
typedef enum {
    V5_PIXEL_FORMAT_ARGB_1555 = 8,
    V5_PIXEL_FORMAT_ARGB_8888 = 11,

    V5_PIXEL_FORMAT_RGB_BAYER_8BPP = 23,
    V5_PIXEL_FORMAT_RGB_BAYER_10BPP = 24,
    V5_PIXEL_FORMAT_RGB_BAYER_12BPP = 25,
    V5_PIXEL_FORMAT_RGB_BAYER_14BPP = 26,
    V5_PIXEL_FORMAT_RGB_BAYER_16BPP = 27,

    V5_PIXEL_FORMAT_YVU_PLANAR_420 = 35,
    V5_PIXEL_FORMAT_YVU_SEMIPLANAR_420 = 38,
    V5_PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 41,
} v5_pixel_format;

/*
 * Both semi-planar 420s are named because they are one enum apart in effect
 * and three apart in value, and picking the wrong one costs a picture with
 * the colours swapped rather than an error: YVU has the V byte first in the
 * chroma plane (NV21), YUV has U first (NV12). The vendor sample and every
 * VENC path here run YVU_SEMIPLANAR_420, which is 38.
 */

/* ot_video_format. LINEAR is the only one this backend produces; the tile
 * formats exist for the codec's internal reference frames. */
typedef enum {
    V5_VIDEO_FORMAT_LINEAR = 0,
} v5_video_format;

/*
 * ot_compress_mode.
 *
 * SEG_COMPACT is the one this backend selects, and only on VPSS physical
 * channel 0: it is what the vendor's own samples set there
 * (sample_venc.c's get_default_vpss_chn_attr and
 * sample_comm_vpss_get_default_vpss_cfg both do), and the tuning guide
 * says CV610 supports YUV output compression on CHN0 alone. It costs
 * about 31% of the block -- see hisi_vb_seg_compact_size -- for a format
 * VENC reads directly.
 *
 * NONE stays the default everywhere else. FRAME is what a 3DNR reference
 * frame wants and LINE is what the FMU wrap path uses; neither is driven
 * here, and both are named so a reader diffing against ot_common_video.h
 * sees no gap.
 */
typedef enum {
    V5_COMPRESS_MODE_NONE = 0,
    V5_COMPRESS_MODE_SEG = 1,
    V5_COMPRESS_MODE_SEG_COMPACT = 2,
    V5_COMPRESS_MODE_TILE = 3,
    V5_COMPRESS_MODE_LINE = 4,
    V5_COMPRESS_MODE_FRAME = 5,
} v5_compress_mode;

/*
 * ot_vb_src (ot_common_video.h:48-53).
 *
 * Where a stage takes its output blocks from. COMMON is the default and
 * means the pools ss_mpi_vb_set_cfg configured; USER means a pool the
 * caller made with ss_mpi_vb_create_pool and attached to this channel,
 * which is how a VPSS channel gets a block cut to its own frame instead
 * of a sensor-sized one out of the common pool.
 *
 * Selecting USER is not enough on its own: ss_mpi_vpss_set_chn_vb_src
 * chooses the source and ss_mpi_vpss_attach_chn_vb_pool supplies the
 * pool, and a channel set to USER with nothing attached has no blocks at
 * all. MOD and PRIVATE are named for completeness; nothing here selects
 * them.
 */
typedef enum {
    V5_VB_SRC_COMMON = 0,
    V5_VB_SRC_MOD = 1,
    V5_VB_SRC_PRIVATE = 2,
    V5_VB_SRC_USER = 3,
} v5_vb_src;

/* ot_dynamic_range. SDR8 is the linear-mode pipeline; the WDR modes this
 * backend does not drive would want SDR10 or HDR10. */
typedef enum {
    V5_DYNAMIC_RANGE_SDR8 = 0,
    V5_DYNAMIC_RANGE_SDR10 = 1,
    V5_DYNAMIC_RANGE_HDR10 = 2,
} v5_dynamic_range;

/* ot_video_field. Progressive sensors give FRAME; the interlaced values
 * exist for BT.656 sources this backend has none of. */
typedef enum {
    V5_VIDEO_FIELD_TOP = 1,
    V5_VIDEO_FIELD_BOTTOM = 2,
    V5_VIDEO_FIELD_INTERLACED = 3,
    V5_VIDEO_FIELD_FRAME = 4,
} v5_video_field;

/* ot_op_mode. The auto/manual switch every ISP tuning attribute carries;
 * Phase 3's users are many, Phase 2's is the exposure attribute. */
typedef enum {
    V5_OP_MODE_AUTO = 0,
    V5_OP_MODE_MANUAL = 1,
} v5_op_mode;

/* ot_coord, the crop rectangle's interpretation. ABS is pixels, RATIO is
 * 1/1000ths of the source. This backend uses ABS: raptor's crop config is
 * in pixels and converting twice is how a rounding error becomes a
 * one-pixel green edge. */
typedef enum {
    V5_COORD_ABS = 0,
    V5_COORD_RATIO = 1,
} v5_coord;

/*
 * ot_wdr_mode. NONE is what a linear sensor runs and what every INI this
 * backend ships selects; the rest are transcribed because the ISP public
 * attribute carries the field and a reader comparing against the sample
 * should not have to guess which value 3 is.
 */
typedef enum {
    V5_WDR_MODE_NONE = 0,
    V5_WDR_MODE_BUILT_IN = 1,
    V5_WDR_MODE_QUADRA = 2,
    V5_WDR_MODE_2To1_LINE = 3,
    V5_WDR_MODE_2To1_FRAME = 4,
} v5_wdr_mode;

/* ot_isp_bayer_format. Which corner of the 2x2 the red pixel is in; comes
 * from the sensor INI and goes into the ISP public attribute. */
typedef enum {
    V5_BAYER_RGGB = 0,
    V5_BAYER_GRBG = 1,
    V5_BAYER_GBRG = 2,
    V5_BAYER_BGGR = 3,
} v5_bayer_format;

/* ================================================================
 * GEOMETRY AND RATE
 * ================================================================ */

/*
 * ot_frame_rate_ctrl. Two *signed* ints, and -1 means "do not control" --
 * which is why they are not unsigned however impossible a negative frame
 * rate is. Setting src to the sensor's rate and dst to the stream's is how
 * every stage drops frames on V5; there is no separate decimation call.
 */
typedef struct {
    int src_frame_rate;
    int dst_frame_rate;
} v5_frame_rate_ctrl;

_Static_assert(sizeof(v5_frame_rate_ctrl) == 8, "ot_frame_rate_ctrl is 8 bytes");

/* ot_border. Five u32: four widths and a colour. Zeroed throughout this
 * backend -- rvd draws its own borders in the OSD -- but present because
 * it is embedded in ot_vpss_chn_attr at a fixed offset. */
typedef struct {
    unsigned int top_width;
    unsigned int bottom_width;
    unsigned int left_width;
    unsigned int right_width;
    unsigned int color;
} v5_border;

_Static_assert(sizeof(v5_border) == 20, "ot_border is 20 bytes");

/* ot_aspect_ratio. Mode NONE means "fill the channel", which is what raptor
 * wants: the scaler's aspect is decided by the geometry rvd asked for, and
 * letterboxing inside the channel would be a second, invisible policy. */
typedef enum {
    V5_ASPECT_RATIO_NONE = 0,
    V5_ASPECT_RATIO_AUTO = 1,
    V5_ASPECT_RATIO_MANUAL = 2,
} v5_aspect_ratio_type;

typedef struct {
    v5_aspect_ratio_type mode;
    unsigned int bg_color;
    v5_rect video_rect;
} v5_aspect_ratio;

_Static_assert(sizeof(v5_aspect_ratio) == 24, "ot_aspect_ratio is 24 bytes");

/* ================================================================
 * THE FRAME
 * ================================================================ */

/*
 * OT_MAX_COLOR_COMPONENT is 2 on this part, not 3: the pipeline is
 * semi-planar throughout, so there is a luma plane and one interleaved
 * chroma plane and never a third. It is the array bound in ot_video_frame,
 * so it is ABI. A part with 3 would move every offset past +32.
 */
#define V5_MAX_COLOR_COMPONENT 2

/* OT_MAX_USER_DATA_NUM, the two 64-bit words a frame carries through the
 * pipeline untouched. raptor does not use them; they are here for the
 * layout. */
#define V5_MAX_USER_DATA_NUM 2

/*
 * ot_video_supplement, the side buffers VB attaches when
 * ss_mpi_vb_set_supplement_cfg asked for them (v5_vb.h). Eight physical
 * addresses then eight pointers, so it is 64 bytes on a 32-bit target and
 * 128 on the host -- which is why ot_video_frame's total is asserted only
 * under V5_ABI32.
 *
 * jpeg_dcf is the one Phase 2 would reach for if raptor wanted EXIF on a
 * snapshot, and isp_info is what a raw dump needs. Neither is used yet;
 * the struct is transcribed whole because a partial one would put
 * ot_video_frame's tail in the wrong place.
 */
typedef struct {
    unsigned int misc_info_phys_addr;
    unsigned int jpeg_dcf_phys_addr;
    unsigned int isp_info_phys_addr;
    unsigned int low_delay_phys_addr;
    unsigned int bnr_mot_phys_addr;
    unsigned int motion_data_phys_addr;
    unsigned int frame_dng_phys_addr;
    unsigned int aiisp_phys_addr;

    void *misc_info_virt_addr;
    void *jpeg_dcf_virt_addr;
    void *isp_info_virt_addr;
    void *low_delay_virt_addr;
    void *bnr_mot_virt_addr;
    void *motion_data_virt_addr;
    void *frame_dng_virt_addr;
    void *aiisp_virt_addr;
} v5_video_supplement;

#if V5_ABI32
_Static_assert(sizeof(v5_video_supplement) == 64, "ot_video_supplement is 64 bytes on ARM32");
#endif

/*
 * ot_video_frame. What ss_mpi_vpss_get_chn_frame fills and what
 * ss_mpi_rgn_* and the JPEG path read.
 *
 * The offsets that matter and are easy to get wrong:
 *
 *   header_stride +32   two u32 -- the compressed-header stride, zero when
 *                       compress_mode is NONE, and *not* the pixel stride
 *   stride        +40   two u32 -- the pixel stride per plane
 *   phys_addr     +56   two u32 -- plane physical addresses
 *   virt_addr     +72   two pointers -- NULL unless the pool was remapped
 *   pts           +88   u64, and 8-byte aligned, which is why time_ref at
 *                       +80 is followed by four bytes of padding
 *
 * Under V5_ABI32 the whole thing is 184 bytes and ot_video_frame_info is
 * 192. On the host the pointers double and both grow; the field offsets up
 * to header_virt_addr are the same either way, so those asserts stay
 * unconditional and catch a mis-transcription of the leading half.
 */
typedef struct {
    unsigned int width;
    unsigned int height;
    v5_video_field field;
    v5_pixel_format pixel_format;
    v5_video_format video_format;
    v5_compress_mode compress_mode;
    v5_dynamic_range dynamic_range;
    int color_gamut;

    unsigned int header_stride[V5_MAX_COLOR_COMPONENT];
    unsigned int stride[V5_MAX_COLOR_COMPONENT];

    unsigned int header_phys_addr[V5_MAX_COLOR_COMPONENT];
    unsigned int phys_addr[V5_MAX_COLOR_COMPONENT];
    void *header_virt_addr[V5_MAX_COLOR_COMPONENT];
    void *virt_addr[V5_MAX_COLOR_COMPONENT];

    unsigned int time_ref;
    unsigned long long pts;

    unsigned long long user_data[V5_MAX_USER_DATA_NUM];
    unsigned int frame_flag;
    v5_video_supplement supplement;
} v5_video_frame;

_Static_assert(offsetof(v5_video_frame, header_stride) == 32,
               "ot_video_frame.header_stride at +32");
_Static_assert(offsetof(v5_video_frame, stride) == 40, "ot_video_frame.stride at +40");
_Static_assert(offsetof(v5_video_frame, header_phys_addr) == 48,
               "ot_video_frame.header_phys_addr at +48");
_Static_assert(offsetof(v5_video_frame, phys_addr) == 56, "ot_video_frame.phys_addr at +56");
_Static_assert(offsetof(v5_video_frame, header_virt_addr) == 64,
               "ot_video_frame.header_virt_addr at +64");

#if V5_ABI32
_Static_assert(sizeof(v5_video_frame) == 184, "ot_video_frame is 184 bytes on ARM32");
_Static_assert(offsetof(v5_video_frame, virt_addr) == 72, "ot_video_frame.virt_addr at +72");
_Static_assert(offsetof(v5_video_frame, time_ref) == 80, "ot_video_frame.time_ref at +80");
_Static_assert(offsetof(v5_video_frame, pts) == 88, "ot_video_frame.pts at +88");
_Static_assert(offsetof(v5_video_frame, supplement) == 116, "ot_video_frame.supplement at +116");
#endif

/* ot_video_frame_info. The frame plus which pool it came from and which
 * module owns it -- the two fields a release call needs and the frame
 * itself does not carry. */
typedef struct {
    v5_video_frame video_frame;
    unsigned int pool_id;
    v5_mod_id mod_id;
} v5_video_frame_info;

#if V5_ABI32
_Static_assert(sizeof(v5_video_frame_info) == 192, "ot_video_frame_info is 192 bytes on ARM32");
_Static_assert(offsetof(v5_video_frame_info, pool_id) == 184,
               "ot_video_frame_info.pool_id at +184");
#endif

#endif /* HISI_V5_VIDEO_H */
