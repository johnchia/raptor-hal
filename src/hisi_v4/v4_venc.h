/*
 * hisi_v4/v4_venc.h -- HI_MPI_VENC bindings, HiMPP V4.0
 *
 * One flat level: channels, each with a codec, a rate-control mode and a GOP
 * structure. Frames arrive from a VPSS channel through a kernel-side bind, so
 * nothing here pushes pictures; the userspace side only configures, waits on
 * a file descriptor, and collects packets.
 *
 * Three unions decide the layout, and each is discriminated by the enum
 * immediately before it:
 *
 *   VENC_ATTR_S.enType      -> the per-codec attribute block
 *   VENC_RC_ATTR_S.enRcMode -> the rate-control parameters
 *   VENC_GOP_ATTR_S.enGopMode -> the GOP parameters
 *
 * The rate-control union is the one that punishes carelessness: the H.264
 * and H.265 forms of every mode are separate enumerators over identically
 * shaped payloads, so writing H264CBR's parameters under an H265 channel
 * lands the right bytes with the wrong tag and the driver rejects the
 * channel with an error that names neither. v4_venc_rc_mode() below is the
 * single place that mapping is made.
 *
 * PROVENANCE. Derived from the Hi3516EV200 SDK V1.0.1.0 headers
 * (mpp/include/hi_comm_venc.h, hi_comm_rc.h) and cross-checked against
 * ref/openhisilicon/include/comm_venc.h and comm_rc.h (GPL v3). Offsets from
 * a probe compiled with arm-openipc-linux-musleabi-gcc.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_VENC_H
#define HISI_V4_VENC_H

#include "v4_common.h"
#include "v4_video.h"

/* VENC_MAX_CHN_NUM. The header bound, which is what arrays are sized by.
 * The board's driver reports VencMaxChnNum = 3 as a module parameter, and
 * that measured number is what hal_caps.c promises. */
#define V4_VENC_MAX_CHN_NUM 16

/*
 * PAYLOAD_TYPE_E. Four sparse values out of an RTP-payload-derived
 * enumeration -- 96 and 265 are not adjacent and neither is 1002. Read from
 * a compiled probe, because guessing an ordinal here would be silent.
 */
typedef enum {
    V4_PT_H264 = 96,
    V4_PT_JPEG = 26,
    V4_PT_H265 = 265,
    V4_PT_MJPEG = 1002,
} v4_payload_type;

/* VENC_RC_MODE_E */
typedef enum {
    V4_RC_MODE_H264CBR = 1,
    V4_RC_MODE_H264VBR = 2,
    V4_RC_MODE_H264AVBR = 3,
    V4_RC_MODE_H264QVBR = 4,
    V4_RC_MODE_H264CVBR = 5,
    V4_RC_MODE_H264FIXQP = 6,
    V4_RC_MODE_H264QPMAP = 7,

    V4_RC_MODE_MJPEGCBR = 8,
    V4_RC_MODE_MJPEGVBR = 9,
    V4_RC_MODE_MJPEGFIXQP = 10,

    V4_RC_MODE_H265CBR = 11,
    V4_RC_MODE_H265VBR = 12,
    V4_RC_MODE_H265AVBR = 13,
    V4_RC_MODE_H265QVBR = 14,
    V4_RC_MODE_H265CVBR = 15,
    V4_RC_MODE_H265FIXQP = 16,
    V4_RC_MODE_H265QPMAP = 17,
} v4_venc_rc_mode;

/* VENC_GOP_MODE_E */
typedef enum {
    V4_GOP_MODE_NORMALP = 0,
    V4_GOP_MODE_DUALP = 1,
    V4_GOP_MODE_SMARTP = 2,
} v4_venc_gop_mode;

/*
 * H264E_NALU_TYPE_E and H265E_NALU_TYPE_E, which share a union in
 * VENC_PACK_S and do *not* share values. IDR is 5 on H.264 and 19 on H.265;
 * SPS is 7 and 33. A packet classifier that reads the H.264 constants on an
 * H.265 stream marks parameter sets as slices, which is how a decoder ends
 * up seeing a stream with no SPS.
 */
typedef enum {
    V4_H264_NALU_BSLICE = 0,
    V4_H264_NALU_PSLICE = 1,
    V4_H264_NALU_ISLICE = 2,
    V4_H264_NALU_IDRSLICE = 5,
    V4_H264_NALU_SEI = 6,
    V4_H264_NALU_SPS = 7,
    V4_H264_NALU_PPS = 8,
} v4_h264_nalu_type;

typedef enum {
    V4_H265_NALU_BSLICE = 0,
    V4_H265_NALU_PSLICE = 1,
    V4_H265_NALU_ISLICE = 2,
    V4_H265_NALU_IDRSLICE = 19,
    V4_H265_NALU_VPS = 32,
    V4_H265_NALU_SPS = 33,
    V4_H265_NALU_PPS = 34,
    V4_H265_NALU_SEI = 39,
} v4_h265_nalu_type;

/* ================================================================
 * CHANNEL ATTRIBUTES
 * ================================================================ */

typedef struct {
    unsigned char large_thumbnail_num;
    v4_size large_thumbnail_size[2];
} v4_venc_mpf_cfg;

_Static_assert(sizeof(v4_venc_mpf_cfg) == 20, "VENC_MPF_CFG_S is 20 bytes");

typedef struct {
    int support_dcf;
    v4_venc_mpf_cfg mpf_cfg;
    unsigned int receive_mode;
} v4_venc_attr_jpeg;

_Static_assert(sizeof(v4_venc_attr_jpeg) == 28, "VENC_ATTR_JPEG_S is 28 bytes");

/*
 * The per-codec union. H.264, H.265 and MJPEG contribute at most one HI_BOOL
 * each; JPEG's 28 bytes set the size. Declared as the JPEG form plus a named
 * alias for the shared leading bool rather than as a real union, because the
 * only member raptor ever writes is bRcnRefShareBuf and a union of four
 * near-empty structs reads as more variety than there is.
 */
typedef union {
    /* VENC_ATTR_H264_S.bRcnRefShareBuf / VENC_ATTR_H265_S.bRcnRefShareBuf --
     * lets the reconstruction and reference buffers share memory. */
    int rcn_ref_share_buf;
    v4_venc_attr_jpeg jpeg;
} v4_venc_codec_attr;

_Static_assert(sizeof(v4_venc_codec_attr) == 28, "the codec union is JPEG-sized");

typedef struct {
    v4_payload_type type;
    unsigned int max_pic_width;
    unsigned int max_pic_height;
    unsigned int buf_size;
    unsigned int profile;
    int by_frame;
    unsigned int pic_width;
    unsigned int pic_height;
    v4_venc_codec_attr codec;
} v4_venc_attr;

_Static_assert(sizeof(v4_venc_attr) == 60, "VENC_ATTR_S is 60 bytes");
_Static_assert(offsetof(v4_venc_attr, buf_size) == 12, "u32BufSize at +12");
_Static_assert(offsetof(v4_venc_attr, profile) == 16, "u32Profile at +16");
_Static_assert(offsetof(v4_venc_attr, by_frame) == 20, "bByFrame at +20");
_Static_assert(offsetof(v4_venc_attr, pic_width) == 24, "u32PicWidth at +24");
_Static_assert(offsetof(v4_venc_attr, codec) == 32, "the codec union at +32");

/*
 * Rate control.
 *
 * Every H.264 and H.265 mode has the same shape, which is what makes one
 * struct per *shape* the honest transcription rather than seventeen
 * near-identical ones. The three shapes are:
 *
 *   cbr   gop, stat_time, src_frame_rate, dst_frame_rate, bit_rate
 *   vbr   gop, stat_time, src_frame_rate, dst_frame_rate, max_bit_rate
 *   fixqp gop, src_frame_rate, dst_frame_rate, i_qp, p_qp, b_qp
 *
 * cbr and vbr differ only in what the last member means, so they are one
 * struct with the member named for its position. Note fixqp has no
 * stat_time: its members are *not* at the same offsets, which is why it is
 * a separate type instead of a flag.
 *
 * fr32DstFrameRate is HI_FR32, a fixed-point frame rate. For any integer
 * rate it is the integer, and raptor has no fractional-rate source, so it is
 * carried as unsigned int and the encoding is documented rather than
 * implemented.
 */
typedef struct {
    unsigned int gop;
    unsigned int stat_time;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int bit_rate; /* u32BitRate on CBR, u32MaxBitRate on VBR/AVBR */
} v4_venc_rc_cbr;

_Static_assert(sizeof(v4_venc_rc_cbr) == 20, "VENC_H264_CBR_S is 20 bytes");

typedef struct {
    unsigned int gop;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int i_qp;
    unsigned int p_qp;
    unsigned int b_qp;
} v4_venc_rc_fixqp;

_Static_assert(sizeof(v4_venc_rc_fixqp) == 24, "VENC_H264_FIXQP_S is 24 bytes");

typedef struct {
    unsigned int stat_time;
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int bit_rate;
} v4_venc_rc_mjpeg_cbr;

_Static_assert(sizeof(v4_venc_rc_mjpeg_cbr) == 16, "VENC_MJPEG_CBR_S is 16 bytes");

typedef struct {
    unsigned int src_frame_rate;
    unsigned int dst_frame_rate;
    unsigned int qfactor;
} v4_venc_rc_mjpeg_fixqp;

_Static_assert(sizeof(v4_venc_rc_mjpeg_fixqp) == 12, "VENC_MJPEG_FIXQP_S is 12 bytes");

/* The union's size is set by CVBR, the longest mode, at nine words. raptor
 * writes none of the long forms; the padding member exists so the struct is
 * the size the driver copies. */
typedef struct {
    v4_venc_rc_mode mode;
    union {
        v4_venc_rc_cbr cbr;
        v4_venc_rc_fixqp fixqp;
        v4_venc_rc_mjpeg_cbr mjpeg_cbr;
        v4_venc_rc_mjpeg_fixqp mjpeg_fixqp;
        unsigned int raw[9]; /* CVBR is the longest member */
    };
} v4_venc_rc_attr;

_Static_assert(sizeof(v4_venc_rc_attr) == 40, "VENC_RC_ATTR_S is 40 bytes");
_Static_assert(offsetof(v4_venc_rc_attr, cbr) == 4, "the RC union at +4");

typedef struct {
    int ip_qp_delta;
} v4_venc_gop_normalp;

typedef struct {
    unsigned int bg_interval;
    int bg_qp_delta;
    int vi_qp_delta;
} v4_venc_gop_smartp;

typedef struct {
    v4_venc_gop_mode mode;
    union {
        v4_venc_gop_normalp normal_p;
        v4_venc_gop_smartp smart_p;
        unsigned int raw[3];
    };
} v4_venc_gop_attr;

_Static_assert(sizeof(v4_venc_gop_attr) == 16, "VENC_GOP_ATTR_S is 16 bytes");
_Static_assert(offsetof(v4_venc_gop_attr, normal_p) == 4, "the GOP union at +4");

typedef struct {
    v4_venc_attr venc_attr;
    v4_venc_rc_attr rc_attr;
    v4_venc_gop_attr gop_attr;
} v4_venc_chn_attr;

_Static_assert(sizeof(v4_venc_chn_attr) == 116, "VENC_CHN_ATTR_S is 116 bytes");
_Static_assert(offsetof(v4_venc_chn_attr, rc_attr) == 60, "stRcAttr at +60");
_Static_assert(offsetof(v4_venc_chn_attr, gop_attr) == 100, "stGopAttr at +100");

/* ================================================================
 * STREAMS
 * ================================================================ */

typedef struct {
    unsigned int pack_type; /* the VENC_DATA_TYPE_U union, one enum wide */
    unsigned int pack_offset;
    unsigned int pack_length;
} v4_venc_pack_info;

_Static_assert(sizeof(v4_venc_pack_info) == 12, "VENC_PACK_INFO_S is 12 bytes");

/*
 * VENC_PACK_S -- one NAL unit, or one JPEG.
 *
 * u64PhyAddr leads and forces 8-byte alignment on the whole struct, which is
 * why pu8Addr lands at +8 rather than +4. data_type is the NALU-type union;
 * which enumeration to read it as depends on the channel's codec, not on
 * anything in the packet.
 *
 * u32Offset is the header length *inside* the packet, not an offset into a
 * larger buffer: the payload starts at pu8Addr + u32Offset and runs to
 * pu8Addr + u32Len. A reader that ignores it emits the start code twice.
 */
typedef struct {
    unsigned long long phy_addr;
    unsigned char *addr;
    unsigned int len;
    unsigned long long pts;
    int frame_end;
    unsigned int data_type;
    unsigned int offset;
    unsigned int data_num;
    v4_venc_pack_info pack_info[8];
} v4_venc_pack;

/* Pointer-bearing, so the exact layout is a 32-bit fact; see V4_ABI32. */
#if V4_ABI32
_Static_assert(sizeof(v4_venc_pack) == 136, "VENC_PACK_S is 136 bytes");
_Static_assert(offsetof(v4_venc_pack, addr) == 8, "pu8Addr at +8, not +4");
_Static_assert(offsetof(v4_venc_pack, len) == 12, "u32Len at +12");
_Static_assert(offsetof(v4_venc_pack, pts) == 16, "u64PTS at +16");
_Static_assert(offsetof(v4_venc_pack, frame_end) == 24, "bFrameEnd at +24");
_Static_assert(offsetof(v4_venc_pack, data_type) == 28, "DataType at +28");
_Static_assert(offsetof(v4_venc_pack, offset) == 32, "u32Offset at +32");
_Static_assert(offsetof(v4_venc_pack, pack_info) == 40, "stPackInfo at +40");
#endif

/*
 * VENC_STREAM_S. The two trailing unions are per-codec statistics blocks
 * that raptor does not read -- rvd takes bitrate and frame counts from its
 * own accounting, which is the only accounting that survives a channel
 * reconfigure -- so they are carried opaquely at their measured sizes. The
 * asserted total is what matters: the driver writes the whole struct.
 */
typedef struct {
    v4_venc_pack *pack;
    unsigned int pack_count;
    unsigned int seq;
    unsigned char stream_info[60];  /* the per-codec VENC_STREAM_INFO_*_S union */
    unsigned char advance_info[312]; /* the per-codec ADVANCE_INFO union */
} v4_venc_stream;

#if V4_ABI32
_Static_assert(sizeof(v4_venc_stream) == 384, "VENC_STREAM_S is 384 bytes");
_Static_assert(offsetof(v4_venc_stream, pack_count) == 4, "u32PackCount at +4");
_Static_assert(offsetof(v4_venc_stream, seq) == 8, "u32Seq at +8");
_Static_assert(offsetof(v4_venc_stream, stream_info) == 12, "the info union at +12");
_Static_assert(offsetof(v4_venc_stream, advance_info) == 72, "the advance union at +72");
#endif

/*
 * VENC_CHN_STATUS_S. The leading counters are what a poll loop reads; the
 * trailing VENC_STREAM_INFO_S contains an HI_DOUBLE, which is why there is a
 * four-byte hole after bJpegSnapEnd that a member-by-member reading misses.
 */
typedef struct {
    unsigned int left_pics;
    unsigned int left_stream_bytes;
    unsigned int left_stream_frames;
    unsigned int cur_packs;
    unsigned int left_recv_pics;
    unsigned int left_enc_pics;
    int jpeg_snap_end;
    unsigned int _pad;
    unsigned char stream_info[56]; /* VENC_STREAM_INFO_S */
} v4_venc_chn_status;

_Static_assert(sizeof(v4_venc_chn_status) == 88, "VENC_CHN_STATUS_S is 88 bytes");
_Static_assert(offsetof(v4_venc_chn_status, cur_packs) == 12, "u32CurPacks at +12");
_Static_assert(offsetof(v4_venc_chn_status, jpeg_snap_end) == 24, "bJpegSnapEnd at +24");
_Static_assert(offsetof(v4_venc_chn_status, stream_info) == 32, "stVencStrmInfo at +32");

/* VENC_RECV_PIC_PARAM_S. -1 means "until stopped"; 0 is explicitly not
 * supported by the driver, so a caller asking for zero frames must not
 * start the channel at all. */
typedef struct {
    int recv_pic_num;
} v4_venc_recv_pic_param;

_Static_assert(sizeof(v4_venc_recv_pic_param) == 4, "VENC_RECV_PIC_PARAM_S is 4 bytes");

/*
 * v4_venc_rc_mode -- the codec-and-mode to enumerator mapping.
 *
 * Exists as one function because the failure it prevents is silent at the
 * call site: every H.265 mode is its H.264 counterpart plus ten, and the
 * payloads are identical, so a mismatched pair writes plausible bytes under
 * the wrong tag.
 */
static inline v4_venc_rc_mode v4_venc_rc_mode_for(v4_payload_type codec, bool cbr, bool fixqp)
{
    if (codec == V4_PT_MJPEG || codec == V4_PT_JPEG)
        return fixqp ? V4_RC_MODE_MJPEGFIXQP : V4_RC_MODE_MJPEGCBR;
    if (codec == V4_PT_H265)
        return fixqp ? V4_RC_MODE_H265FIXQP : (cbr ? V4_RC_MODE_H265CBR : V4_RC_MODE_H265VBR);
    return fixqp ? V4_RC_MODE_H264FIXQP : (cbr ? V4_RC_MODE_H264CBR : V4_RC_MODE_H264VBR);
}

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    int (*fnCreateChn)(int chn, const v4_venc_chn_attr *attr);
    int (*fnDestroyChn)(int chn);
    int (*fnResetChn)(int chn);
    int (*fnStartRecvFrame)(int chn, const v4_venc_recv_pic_param *param);
    int (*fnStopRecvFrame)(int chn);
    int (*fnQueryStatus)(int chn, v4_venc_chn_status *status);
    int (*fnSetChnAttr)(int chn, const v4_venc_chn_attr *attr);
    int (*fnGetChnAttr)(int chn, v4_venc_chn_attr *attr);
    int (*fnGetStream)(int chn, v4_venc_stream *stream, int milli_sec);
    int (*fnReleaseStream)(int chn, v4_venc_stream *stream);
    int (*fnRequestIDR)(int chn, int instant);
    int (*fnGetFd)(int chn);
    int (*fnCloseFd)(int chn);
} v4_venc_impl;

static inline int v4_venc_load(v4_venc_impl *lib, const v4_mpi_libs *libs)
{
    static const char mod[] = "v4_venc";

    memset(lib, 0, sizeof(*lib));

#define V4_VENC_REQ(field, type, name)                                                             \
    do {                                                                                           \
        if (!(lib->field = (type)v4_symbol(mod, libs, "HI_MPI_VENC_" name, "GK_API_VENC_" name)))  \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V4_VENC_REQ(fnCreateChn, int (*)(int, const v4_venc_chn_attr *), "CreateChn");
    V4_VENC_REQ(fnDestroyChn, int (*)(int), "DestroyChn");
    V4_VENC_REQ(fnStartRecvFrame, int (*)(int, const v4_venc_recv_pic_param *), "StartRecvFrame");
    V4_VENC_REQ(fnStopRecvFrame, int (*)(int), "StopRecvFrame");
    V4_VENC_REQ(fnQueryStatus, int (*)(int, v4_venc_chn_status *), "QueryStatus");
    V4_VENC_REQ(fnGetStream, int (*)(int, v4_venc_stream *, int), "GetStream");
    V4_VENC_REQ(fnReleaseStream, int (*)(int, v4_venc_stream *), "ReleaseStream");
    V4_VENC_REQ(fnGetFd, int (*)(int), "GetFd");

#undef V4_VENC_REQ

    lib->fnResetChn = (int (*)(int))v4_symbol_opt(libs, "HI_MPI_VENC_ResetChn",
                                                  "GK_API_VENC_ResetChn");
    lib->fnSetChnAttr = (int (*)(int, const v4_venc_chn_attr *))v4_symbol_opt(
        libs, "HI_MPI_VENC_SetChnAttr", "GK_API_VENC_SetChnAttr");
    lib->fnGetChnAttr = (int (*)(int, v4_venc_chn_attr *))v4_symbol_opt(
        libs, "HI_MPI_VENC_GetChnAttr", "GK_API_VENC_GetChnAttr");
    lib->fnRequestIDR =
        (int (*)(int, int))v4_symbol_opt(libs, "HI_MPI_VENC_RequestIDR", "GK_API_VENC_RequestIDR");
    lib->fnCloseFd =
        (int (*)(int))v4_symbol_opt(libs, "HI_MPI_VENC_CloseFd", "GK_API_VENC_CloseFd");

    return RSS_OK;
}

static inline void v4_venc_unload(v4_venc_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_VENC_H */
