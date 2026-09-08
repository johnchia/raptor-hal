/*
 * hisi_v5/v5_venc.h -- ss_mpi_venc bindings, HiMPP V5.0
 *
 * The encoder is a *channel*: created with an attribute that carries the
 * codec, the rate control and the GOP structure in one struct, started, and
 * then read one stream at a time. raptor binds each VPSS channel to one of
 * these and never calls send_frame.
 *
 * WHAT CHANGED FROM gen4, and the first one is the trap:
 *
 *   - ot_venc_pack carries a *pack_info array*. gen4's VENC_PACK_S had one
 *     NAL per pack and a u32DataType telling you which; V5's pack has
 *     data_num and up to eight (OT_VENC_MAX_PACK_INFO_NUM) sub-packets, each
 *     with its own type, offset and length. Whether the driver actually
 *     fills more than one -- whether a pack is one NAL or one frame -- is
 *     plan risk R9, and it decides how hal_encoder walks a stream. This
 *     file transcribes both paths; only the bench answers which is live.
 *   - ot_venc_attr gained max_pic_width/max_pic_height *separate from*
 *     pic_width/pic_height. The max pair sizes the reference buffers at
 *     create time and cannot grow afterwards; the plain pair is the picture
 *     being coded and can. A backend that sets only the plain pair gets a
 *     channel that will not accept a later resolution increase.
 *   - Rate control moved out to its own header (ot_common_rc.h) and grew
 *     CVBR and RANGEQP modes gen4 has no counterpart for. The mode values
 *     are a flat enumeration across all codecs -- H264_CBR is 2 and
 *     H265_CBR is 14 -- so the mode and the payload type must agree or
 *     create_chn returns OT_ERR_VENC_ILLEGAL_PARAM.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_venc.h and
 * ot_common_rc.h at 1.0.2.0 B051; sizes from a probe compiled against them
 * with the cv6xx cross-compiler:
 *
 *   ot_venc_attr         60   buf_size +12, is_by_frame +20, pic_width +24,
 *                             h264_attr +32
 *   ot_venc_rc_attr      60   ot_venc_gop_attr    28
 *   ot_venc_chn_attr    148   rc_attr +60, gop_attr +120
 *   ot_venc_pack        152   addr +4, len +8, pts +16, data_type +40,
 *                             offset +44, data_num +48, pack_info +52
 *   ot_venc_pack_info    12   ot_venc_stream     520 (pack_cnt +4, seq +8)
 *   ot_venc_chn_status  120   cur_packs +12, stream_info +40
 *   ot_venc_stream_buf_info 16   ot_venc_jpeg_param 204
 *   ot_venc_chn_param    48   ot_venc_start_param  4
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_VENC_H
#define HISI_V5_VENC_H

#include "v5_common.h"
#include "v5_video.h"

/* ot_defines.h:61. Sixteen encoder channels; raptor uses
 * RSS_MAX_ENC_CHANNELS of them, which is smaller. */
#define V5_VENC_MAX_CHN_NUM 16

/* ot_common_venc.h:50, :51, :55. Array bounds, so ABI. */
#define V5_VENC_MAX_PACK_INFO_NUM 8
#define V5_VENC_MAX_MPF_NUM 2
#define V5_VENC_JPEG_QT_COEF_NUM 64

/* ot_defines.h:59. One tile on this part, which is what makes
 * ot_venc_stream_buf_info 16 bytes rather than a multiple of it. */
#define V5_VENC_MAX_TILE_NUM 1

/*
 * ot_payload_type (ot_common.h:328, :348, :372, :378).
 *
 * The RTP payload numbers, which is why they are not consecutive. Only the
 * four raptor can encode are named; the enum has ~60 entries and the rest
 * are decoder-side or audio.
 */
typedef enum {
    V5_PT_JPEG = 26,
    V5_PT_H264 = 96,
    V5_PT_H265 = 265,
    V5_PT_MJPEG = 1002,
} v5_payload_type;

/*
 * ot_venc_rc_mode (ot_common_rc.h:20-52).
 *
 * Flat across codecs and one-based. The values raptor can reach are named;
 * the SVAC3 block (22 onward) is a codec this part does not have.
 */
typedef enum {
    V5_VENC_RC_MODE_H264_ABR = 1,
    V5_VENC_RC_MODE_H264_CBR = 2,
    V5_VENC_RC_MODE_H264_VBR = 3,
    V5_VENC_RC_MODE_H264_AVBR = 4,
    V5_VENC_RC_MODE_H264_QVBR = 5,
    V5_VENC_RC_MODE_H264_CVBR = 6,
    V5_VENC_RC_MODE_H264_FIXQP = 7,
    V5_VENC_RC_MODE_H264_QPMAP = 8,
    V5_VENC_RC_MODE_H264_RANGEQP = 9,
    V5_VENC_RC_MODE_MJPEG_CBR = 10,
    V5_VENC_RC_MODE_MJPEG_VBR = 11,
    V5_VENC_RC_MODE_MJPEG_FIXQP = 12,
    V5_VENC_RC_MODE_H265_ABR = 13,
    V5_VENC_RC_MODE_H265_CBR = 14,
    V5_VENC_RC_MODE_H265_VBR = 15,
    V5_VENC_RC_MODE_H265_AVBR = 16,
    V5_VENC_RC_MODE_H265_QVBR = 17,
    V5_VENC_RC_MODE_H265_CVBR = 18,
    V5_VENC_RC_MODE_H265_FIXQP = 19,
    V5_VENC_RC_MODE_H265_QPMAP = 20,
    V5_VENC_RC_MODE_H265_RANGEQP = 21,
} v5_venc_rc_mode;

/*
 * ot_venc_gop_mode (ot_common_venc.h:439-451).
 *
 * NORMAL_P is IPPP and is what a surveillance stream wants. SMART_P is the
 * long-term-reference mode a low-bitrate profile reaches for; it changes
 * which arm of the GOP union is read, so it is not a drop-in swap.
 */
typedef enum {
    V5_VENC_GOP_MODE_NORMAL_P = 0,
    V5_VENC_GOP_MODE_DUAL_P = 1,
    V5_VENC_GOP_MODE_SMART_P = 2,
    V5_VENC_GOP_MODE_ADV_SMART_P = 3,
    V5_VENC_GOP_MODE_BIPRED_B = 4,
    V5_VENC_GOP_MODE_LOW_DELAY_B = 5,
    V5_VENC_GOP_MODE_SMART_CRR = 6,
    V5_VENC_GOP_MODE_SINGLE_SP = 7,
    V5_VENC_GOP_MODE_I = 8,
} v5_venc_gop_mode;

/*
 * ot_venc_h264_nalu_type (ot_common_venc.h:85-94) and
 * ot_venc_h265_nalu_type (:96-108).
 *
 * These are what a pack's data_type reads back as, and how hal_encoder
 * decides a stream is a keyframe. Note the two enumerations disagree:
 * H.264's IDR is 5 and H.265's is 19, and both call a plain I slice 2. The
 * caller must know the payload type before reading the field.
 */
typedef enum {
    V5_VENC_H264_NALU_B_SLICE = 0,
    V5_VENC_H264_NALU_P_SLICE = 1,
    V5_VENC_H264_NALU_I_SLICE = 2,
    V5_VENC_H264_NALU_IDR_SLICE = 5,
    V5_VENC_H264_NALU_SEI = 6,
    V5_VENC_H264_NALU_SPS = 7,
    V5_VENC_H264_NALU_PPS = 8,
} v5_venc_h264_nalu_type;

typedef enum {
    V5_VENC_H265_NALU_B_SLICE = 0,
    V5_VENC_H265_NALU_P_SLICE = 1,
    V5_VENC_H265_NALU_I_SLICE = 2,
    V5_VENC_H265_NALU_IDR_SLICE = 19,
    V5_VENC_H265_NALU_VPS = 32,
    V5_VENC_H265_NALU_SPS = 33,
    V5_VENC_H265_NALU_PPS = 34,
    V5_VENC_H265_NALU_SEI = 39,
} v5_venc_h265_nalu_type;

/* ot_venc_jpege_pack_type (ot_common_venc.h:141-149). A JPEG pack's
 * data_type; PIC is the whole image, which is the only one a snapshot with
 * ECS output disabled produces. */
typedef enum {
    V5_VENC_JPEG_PACK_ECS = 5,
    V5_VENC_JPEG_PACK_APP = 6,
    V5_VENC_JPEG_PACK_VDO = 7,
    V5_VENC_JPEG_PACK_PIC = 8,
    V5_VENC_JPEG_PACK_DCF = 9,
    V5_VENC_JPEG_PACK_DCF_PIC = 10,
} v5_venc_jpeg_pack_type;

/* ot_venc_pic_recv_mode (ot_common_venc.h:366-370). */
typedef enum {
    V5_VENC_PIC_RECV_SINGLE = 0,
    V5_VENC_PIC_RECV_MULTI = 1,
} v5_venc_pic_recv_mode;

/* ================================================================
 * CHANNEL ATTRIBUTE
 * ================================================================ */

/* ot_venc_h264_attr / ot_venc_h265_attr (ot_common_venc.h:376-384). The two
 * are field-for-field identical, so one type serves both. */
typedef struct {
    int rcn_ref_share_buf_en;
    unsigned int frame_buf_ratio;
} v5_venc_h26x_attr;

/* ot_venc_mpf_cfg (ot_common_venc.h:359-362). td_u8 followed by a
 * 4-aligned array, so large_thumbnail_size is at +4 and the struct is 20. */
typedef struct {
    unsigned char large_thumbnail_num;
    v5_size large_thumbnail_size[V5_VENC_MAX_MPF_NUM];
} v5_venc_mpf_cfg;

_Static_assert(sizeof(v5_venc_mpf_cfg) == 20, "ot_venc_mpf_cfg is 20 bytes");
_Static_assert(offsetof(v5_venc_mpf_cfg, large_thumbnail_size) == 4,
               "ot_venc_mpf_cfg.large_thumbnail_size at +4");

/* ot_venc_jpeg_attr (ot_common_venc.h:371-374). The largest arm of
 * ot_venc_attr's union, which is what makes that union 28 bytes. */
typedef struct {
    int dcf_en;
    v5_venc_mpf_cfg mpf_cfg;
    v5_venc_pic_recv_mode recv_mode;
} v5_venc_jpeg_attr;

_Static_assert(sizeof(v5_venc_jpeg_attr) == 28, "ot_venc_jpeg_attr is 28 bytes");

/*
 * ot_venc_attr (ot_common_venc.h:421-437).
 *
 * buf_size is the stream buffer in bytes and must be a multiple of 64; too
 * small and get_stream starts returning OT_ERR_VENC_BUF_FULL under motion
 * rather than at create time.
 *
 * is_by_frame picks whether get_stream hands back one frame or one slice
 * per call. raptor wants TRUE; with FALSE, a pack is a slice and the R9
 * question changes shape.
 */
typedef struct {
    v5_payload_type type;
    unsigned int max_pic_width;
    unsigned int max_pic_height;
    unsigned int buf_size;
    unsigned int profile;
    int is_by_frame;
    unsigned int pic_width;
    unsigned int pic_height;
    union {
        v5_venc_h26x_attr h264_attr;
        v5_venc_h26x_attr h265_attr;
        v5_venc_jpeg_attr jpeg_attr;
    } codec;
} v5_venc_attr;

_Static_assert(sizeof(v5_venc_attr) == 60, "ot_venc_attr is 60 bytes");
_Static_assert(offsetof(v5_venc_attr, buf_size) == 12, "ot_venc_attr.buf_size at +12");
_Static_assert(offsetof(v5_venc_attr, is_by_frame) == 20, "ot_venc_attr.is_by_frame at +20");
_Static_assert(offsetof(v5_venc_attr, pic_width) == 24, "ot_venc_attr.pic_width at +24");
_Static_assert(offsetof(v5_venc_attr, codec) == 32, "ot_venc_attr codec union at +32");

/*
 * The rate-control arms raptor sets (ot_common_rc.h).
 *
 * Only four of the twenty-one are transcribed: CBR, VBR, CVBR and FIXQP,
 * which cover every profile raptor exposes. The union is sized by the
 * largest arm in the *vendor's* struct -- ot_venc_h264_rangeqp, fourteen
 * u32 -- not by the largest transcribed here, so the assert below is what
 * keeps the omission from shrinking the struct.
 *
 * Note src_frame_rate and dst_frame_rate here are plain u32 and mean what
 * they say, unlike v5_frame_rate_ctrl's signed -1-means-uncontrolled pair.
 */
typedef struct {
    unsigned int gop;
    unsigned int stats_time;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int bit_rate;
} v5_venc_cbr;

/* VBR and AVBR are the same shape with max_bit_rate in place of bit_rate. */
typedef struct {
    unsigned int gop;
    unsigned int stats_time;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int max_bit_rate;
} v5_venc_vbr;

/* ot_venc_h264_cvbr (ot_common_rc.h:115-125). The long-term window is what
 * makes CVBR useful on a bandwidth-capped uplink: it lets a scene burst
 * while holding an hour's average down. */
typedef struct {
    unsigned int gop;
    unsigned int stats_time;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int max_bit_rate;
    unsigned int short_term_stats_time;
    unsigned int long_term_stats_time;
    unsigned int long_term_max_bit_rate;
    unsigned int long_term_min_bit_rate;
} v5_venc_cvbr;

_Static_assert(sizeof(v5_venc_cvbr) == 36, "ot_venc_h264_cvbr is 36 bytes");

/* ot_venc_h264_fixqp (ot_common_rc.h:56-63). */
typedef struct {
    unsigned int gop;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int i_qp;
    unsigned int p_qp;
    unsigned int b_qp;
} v5_venc_fixqp;

/* ot_venc_mjpeg_cbr / _vbr: the same five and four u32 as their H.264
 * counterparts without the gop, which JPEG has no use for. */
typedef struct {
    unsigned int stats_time;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int bit_rate;
} v5_venc_mjpeg_cbr;

/*
 * ot_venc_rc_attr (ot_common_rc.h:196-234).
 *
 * The pad is ot_venc_h264_rangeqp, the union's largest arm and one raptor
 * never sets. Writing it as a sized array rather than transcribing fourteen
 * more u32 keeps the file honest about what it has actually checked, and
 * the size assert is what proves the array is right.
 */
typedef struct {
    v5_venc_rc_mode rc_mode;
    union {
        v5_venc_cbr h264_cbr;
        v5_venc_vbr h264_vbr;
        v5_venc_vbr h264_avbr;
        v5_venc_cvbr h264_cvbr;
        v5_venc_fixqp h264_fixqp;
        v5_venc_cbr h265_cbr;
        v5_venc_vbr h265_vbr;
        v5_venc_vbr h265_avbr;
        v5_venc_cvbr h265_cvbr;
        v5_venc_fixqp h265_fixqp;
        v5_venc_mjpeg_cbr mjpeg_cbr;
        unsigned int pad[14];
    } attr;
} v5_venc_rc_attr;

_Static_assert(sizeof(v5_venc_rc_attr) == 60, "ot_venc_rc_attr is 60 bytes");
_Static_assert(offsetof(v5_venc_rc_attr, attr) == 4, "ot_venc_rc_attr union at +4");

/* ot_venc_gop_normal_p (ot_common_venc.h:453-455) and the SMART_P arm
 * (:463-467), which is the only other one raptor might reach. */
typedef struct {
    int ip_qp_delta;
} v5_venc_gop_normal_p;

typedef struct {
    unsigned int bg_interval;
    int bg_qp_delta;
    int vi_qp_delta;
} v5_venc_gop_smart_p;

/* ot_venc_gop_attr (ot_common_venc.h:510-520). The pad is
 * ot_venc_gop_smart_crr, six words and the largest arm. */
typedef struct {
    v5_venc_gop_mode gop_mode;
    union {
        v5_venc_gop_normal_p normal_p;
        v5_venc_gop_smart_p smart_p;
        v5_venc_gop_smart_p adv_smart_p;
        unsigned int pad[6];
    } attr;
} v5_venc_gop_attr;

_Static_assert(sizeof(v5_venc_gop_attr) == 28, "ot_venc_gop_attr is 28 bytes");

/* ot_venc_chn_attr (ot_common_venc.h:516-520). */
typedef struct {
    v5_venc_attr venc_attr;
    v5_venc_rc_attr rc_attr;
    v5_venc_gop_attr gop_attr;
} v5_venc_chn_attr;

_Static_assert(sizeof(v5_venc_chn_attr) == 148, "ot_venc_chn_attr is 148 bytes");
_Static_assert(offsetof(v5_venc_chn_attr, rc_attr) == 60, "ot_venc_chn_attr.rc_attr at +60");
_Static_assert(offsetof(v5_venc_chn_attr, gop_attr) == 120, "ot_venc_chn_attr.gop_attr at +120");

/*
 * ot_venc_start_param (ot_common_venc.h:543-545).
 *
 * recv_pic_num is how many frames the channel will encode before stopping
 * itself; -1 is "until stop_chn". A JPEG snapshot channel sets 1.
 */
typedef struct {
    int recv_pic_num;
} v5_venc_start_param;

_Static_assert(sizeof(v5_venc_start_param) == 4, "ot_venc_start_param is 4 bytes");

/* ================================================================
 * STREAM
 * ================================================================ */

/*
 * ot_venc_data_type (ot_common_venc.h:165-171).
 *
 * A union of five enums, so four bytes; which arm to read depends on the
 * channel's payload type, and there is nothing in the struct that says so.
 */
typedef union {
    v5_venc_h264_nalu_type h264_type;
    v5_venc_jpeg_pack_type jpeg_type;
    v5_venc_h265_nalu_type h265_type;
    int raw;
} v5_venc_data_type;

/* ot_venc_pack_info (ot_common_venc.h:173-177). */
typedef struct {
    v5_venc_data_type pack_type;
    unsigned int pack_offset;
    unsigned int pack_len;
} v5_venc_pack_info;

_Static_assert(sizeof(v5_venc_pack_info) == 12, "ot_venc_pack_info is 12 bytes");

/*
 * ot_venc_pack (ot_common_venc.h:179-189).
 *
 * addr points at the start of the mapped stream buffer *for this pack*, and
 * `offset` is how far into it the payload begins -- the leading bytes are
 * the start code the hardware wrote. So the bytes to hand to a muxer are
 * addr + offset for len - offset, exactly as gen4's pack worked.
 *
 * data_num and pack_info are the R9 question: if data_num comes back 1 for
 * every pack then a pack is one NAL and hal_encoder walks the pack array;
 * if it comes back greater than 1 then a pack is a frame and hal_encoder
 * must walk pack_info inside each pack. Both are transcribed; the bench
 * decides which loop runs.
 */
typedef struct {
    unsigned int phys_addr;
    unsigned char *addr;
    unsigned int len;
    unsigned long long pts;
    int is_frame_end;
    int stuff03_disable;
    unsigned int layer_id;
    int is_lu_begin;
    v5_venc_data_type data_type;
    unsigned int offset;
    unsigned int data_num;
    v5_venc_pack_info pack_info[V5_VENC_MAX_PACK_INFO_NUM];
} v5_venc_pack;

_Static_assert(sizeof(v5_venc_pack) == 152, "ot_venc_pack is 152 bytes");
_Static_assert(offsetof(v5_venc_pack, addr) == 4, "ot_venc_pack.addr at +4");
_Static_assert(offsetof(v5_venc_pack, len) == 8, "ot_venc_pack.len at +8");
_Static_assert(offsetof(v5_venc_pack, pts) == 16, "ot_venc_pack.pts at +16");
_Static_assert(offsetof(v5_venc_pack, data_type) == 40, "ot_venc_pack.data_type at +40");
_Static_assert(offsetof(v5_venc_pack, offset) == 44, "ot_venc_pack.offset at +44");
_Static_assert(offsetof(v5_venc_pack, data_num) == 48, "ot_venc_pack.data_num at +48");
_Static_assert(offsetof(v5_venc_pack, pack_info) == 52, "ot_venc_pack.pack_info at +52");

/*
 * ot_venc_stream (ot_common_venc.h:325-341).
 *
 * pack is caller-allocated: get_stream fills in pack_cnt only if pack
 * already points at an array big enough, which query_status.cur_packs is
 * how you size. Getting that wrong is a heap overwrite, not an error code.
 *
 * The two trailing unions are per-codec statistics -- bitrate breakdowns,
 * QP histograms, PSNR -- that raptor does not read. They are carried as
 * sized arrays rather than transcribed because transcribing 440 bytes of
 * fields nothing uses is 440 bytes of unchecked transcription; the size
 * assert covers the whole struct either way. codec_info is u32-aligned and
 * adv_info is u64-aligned, which is what the element types are for.
 */
typedef struct {
    v5_venc_pack *pack;
    unsigned int pack_cnt;
    unsigned int seq;
    unsigned int codec_info[17];
    unsigned long long adv_info[55];
} v5_venc_stream;

_Static_assert(sizeof(v5_venc_stream) == 520, "ot_venc_stream is 520 bytes");
_Static_assert(offsetof(v5_venc_stream, pack_cnt) == 4, "ot_venc_stream.pack_cnt at +4");
_Static_assert(offsetof(v5_venc_stream, seq) == 8, "ot_venc_stream.seq at +8");
_Static_assert(offsetof(v5_venc_stream, codec_info) == 12,
               "ot_venc_stream per-codec stream info at +12");
_Static_assert(offsetof(v5_venc_stream, adv_info) == 80,
               "ot_venc_stream per-codec advanced info at +80");

/*
 * ot_venc_chn_status (ot_common_venc.h:526-539).
 *
 * cur_packs is the one field that matters: it is how many ot_venc_pack the
 * next get_stream will want, and the only correct way to size the array.
 * The trailing ot_venc_stream_info is 64 bytes of per-frame statistics
 * raptor does not read, carried as a sized array for the reason above.
 */
typedef struct {
    unsigned int left_pics;
    unsigned int left_stream_bytes;
    unsigned int left_stream_frames;
    unsigned int cur_packs;
    unsigned int left_recv_pics;
    unsigned int left_enc_pics;
    int is_jpeg_snap_end;
    unsigned long long release_pic_pts;
    unsigned long long stream_info[8];
    unsigned int discard_pics;
    unsigned int stream_buf_full_cnt;
    long long cvbr_bytes_saving;
} v5_venc_chn_status;

_Static_assert(sizeof(v5_venc_chn_status) == 120, "ot_venc_chn_status is 120 bytes");
_Static_assert(offsetof(v5_venc_chn_status, cur_packs) == 12,
               "ot_venc_chn_status.cur_packs at +12");
_Static_assert(offsetof(v5_venc_chn_status, stream_info) == 40,
               "ot_venc_chn_status.stream_info at +40");

/*
 * ot_venc_stream_buf_info (ot_common_venc.h:715-719).
 *
 * The physical and (once mapped) virtual base of the channel's stream
 * buffer. One tile on this part, so one of each.
 */
typedef struct {
    unsigned int phys_addr[V5_VENC_MAX_TILE_NUM];
    void *user_addr[V5_VENC_MAX_TILE_NUM];
    unsigned long long buf_size[V5_VENC_MAX_TILE_NUM];
} v5_venc_stream_buf_info;

_Static_assert(sizeof(v5_venc_stream_buf_info) == 16, "ot_venc_stream_buf_info is 16 bytes");

/*
 * ot_venc_chn_param (ot_common_venc.h:938-948).
 *
 * The per-channel crop and frame rate, set after create_chn. This is where
 * a stream gets a frame rate lower than the VPSS channel feeding it without
 * a second VPSS channel.
 */
typedef struct {
    int color_to_grey_en;
    unsigned int priority;
    unsigned int max_stream_cnt;
    unsigned int poll_wake_up_frame_cnt;
    struct {
        int enable;
        v5_rect rect;
    } crop_info;
    v5_frame_rate_ctrl frame_rate;
    unsigned int in_depth;
} v5_venc_chn_param;

_Static_assert(sizeof(v5_venc_chn_param) == 48, "ot_venc_chn_param is 48 bytes");
_Static_assert(offsetof(v5_venc_chn_param, crop_info) == 16, "ot_venc_chn_param.crop_info at +16");
_Static_assert(offsetof(v5_venc_chn_param, frame_rate) == 36,
               "ot_venc_chn_param.frame_rate at +36");

/*
 * ot_venc_jpeg_param (ot_common_venc.h:626-633).
 *
 * qfactor is the snapshot quality knob, [1, 99]. The three quantisation
 * tables are readback: set_jpeg_param recomputes them from qfactor, so
 * writing them by hand is only for a caller supplying its own tables.
 */
typedef struct {
    unsigned int qfactor;
    unsigned char y_qt[V5_VENC_JPEG_QT_COEF_NUM];
    unsigned char cb_qt[V5_VENC_JPEG_QT_COEF_NUM];
    unsigned char cr_qt[V5_VENC_JPEG_QT_COEF_NUM];
    unsigned int mcu_per_ecs;
    int ecs_output_en;
} v5_venc_jpeg_param;

_Static_assert(sizeof(v5_venc_jpeg_param) == 204, "ot_venc_jpeg_param is 204 bytes");
_Static_assert(offsetof(v5_venc_jpeg_param, mcu_per_ecs) == 196,
               "ot_venc_jpeg_param.mcu_per_ecs at +196");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    /* Channel lifecycle. */
    int (*fnCreateChn)(int chn, const v5_venc_chn_attr *attr);
    int (*fnDestroyChn)(int chn);
    int (*fnStartChn)(int chn, const v5_venc_start_param *param);
    int (*fnStopChn)(int chn);
    int (*fnResetChn)(int chn);

    /* Attribute, for a resolution or bitrate change on a live channel. */
    int (*fnSetChnAttr)(int chn, const v5_venc_chn_attr *attr);
    int (*fnGetChnAttr)(int chn, v5_venc_chn_attr *attr);

    /* Stream. get_fd is what the reader thread selects on. */
    int (*fnQueryStatus)(int chn, v5_venc_chn_status *status);
    int (*fnGetStream)(int chn, v5_venc_stream *stream, int milli_sec);
    int (*fnReleaseStream)(int chn, const v5_venc_stream *stream);
    int (*fnGetFd)(int chn);

    /*
     * Keyframe on demand. request_idr's `instant` says whether to code the
     * IDR now or at the next GOP boundary; raptor wants TD_TRUE.
     */
    int (*fnRequestIdr)(int chn, int instant);
    int (*fnEnableIdr)(int chn, int enable);

    /* Per-channel crop and frame rate; see v5_venc_chn_param. */
    int (*fnSetChnParam)(int chn, const v5_venc_chn_param *param);
    int (*fnGetChnParam)(int chn, v5_venc_chn_param *param);

    /* JPEG quality. Optional: a build with no snapshot path never calls it. */
    int (*fnSetJpegParam)(int chn, const v5_venc_jpeg_param *param);
    int (*fnGetJpegParam)(int chn, v5_venc_jpeg_param *param);

    /*
     * The stream buffer's base addresses, for a zero-copy reader. Optional
     * and unused in Phase 2 -- pack.addr is already a mapped pointer -- but
     * bound because Phase 7's memory accounting wants the buffer size and
     * this is the only call that reports it.
     */
    int (*fnGetStreamBufInfo)(int chn, v5_venc_stream_buf_info *info);

    /* SEI insertion, for a timestamp or a watermark. Phase 3's. */
    int (*fnInsertUserData)(int chn, unsigned char *data, unsigned int len);
} v5_venc_impl;

/*
 * v5_venc_load -- bind the VENC entry points.
 *
 * Required is create/destroy/start/stop plus the stream read: without any
 * one of them there is no encoder at all. query_status is required too,
 * because it is the only correct way to size the pack array and guessing is
 * the heap overwrite noted above.
 */
static inline int v5_venc_load(v5_venc_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_venc";

    memset(lib, 0, sizeof(*lib));

#define V5_VENC_REQ(field, type, name)                                                             \
    do {                                                                                           \
        if (!(lib->field = (type)v5_symbol(mod, libs, name)))                                      \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V5_VENC_REQ(fnCreateChn, int (*)(int, const v5_venc_chn_attr *), "ss_mpi_venc_create_chn");
    V5_VENC_REQ(fnDestroyChn, int (*)(int), "ss_mpi_venc_destroy_chn");
    V5_VENC_REQ(fnStartChn, int (*)(int, const v5_venc_start_param *), "ss_mpi_venc_start_chn");
    V5_VENC_REQ(fnStopChn, int (*)(int), "ss_mpi_venc_stop_chn");
    V5_VENC_REQ(fnQueryStatus, int (*)(int, v5_venc_chn_status *), "ss_mpi_venc_query_status");
    V5_VENC_REQ(fnGetStream, int (*)(int, v5_venc_stream *, int), "ss_mpi_venc_get_stream");
    V5_VENC_REQ(fnReleaseStream, int (*)(int, const v5_venc_stream *),
                "ss_mpi_venc_release_stream");

#undef V5_VENC_REQ

    lib->fnResetChn = (int (*)(int))v5_symbol_opt(libs, "ss_mpi_venc_reset_chn");
    lib->fnSetChnAttr =
        (int (*)(int, const v5_venc_chn_attr *))v5_symbol_opt(libs, "ss_mpi_venc_set_chn_attr");
    lib->fnGetChnAttr =
        (int (*)(int, v5_venc_chn_attr *))v5_symbol_opt(libs, "ss_mpi_venc_get_chn_attr");

    lib->fnGetFd = (int (*)(int))v5_symbol_opt(libs, "ss_mpi_venc_get_fd");
    lib->fnRequestIdr = (int (*)(int, int))v5_symbol_opt(libs, "ss_mpi_venc_request_idr");
    lib->fnEnableIdr = (int (*)(int, int))v5_symbol_opt(libs, "ss_mpi_venc_enable_idr");

    lib->fnSetChnParam =
        (int (*)(int, const v5_venc_chn_param *))v5_symbol_opt(libs, "ss_mpi_venc_set_chn_param");
    lib->fnGetChnParam =
        (int (*)(int, v5_venc_chn_param *))v5_symbol_opt(libs, "ss_mpi_venc_get_chn_param");

    lib->fnSetJpegParam =
        (int (*)(int, const v5_venc_jpeg_param *))v5_symbol_opt(libs, "ss_mpi_venc_set_jpeg_param");
    lib->fnGetJpegParam =
        (int (*)(int, v5_venc_jpeg_param *))v5_symbol_opt(libs, "ss_mpi_venc_get_jpeg_param");

    lib->fnGetStreamBufInfo = (int (*)(int, v5_venc_stream_buf_info *))v5_symbol_opt(
        libs, "ss_mpi_venc_get_stream_buf_info");

    lib->fnInsertUserData = (int (*)(int, unsigned char *, unsigned int))v5_symbol_opt(
        libs, "ss_mpi_venc_insert_user_data");

    return RSS_OK;
}

static inline void v5_venc_unload(v5_venc_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_VENC_H */
