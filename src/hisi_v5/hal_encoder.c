/*
 * hisi_v5/hal_encoder.c -- VENC channels, HiMPP V5.0
 *
 * MAPPING. raptor encoder channel N == VENC channel N. Frames arrive from
 * a VPSS channel through a kernel-side bind, so nothing here pushes
 * pictures; the userspace side configures the channel, waits on a file
 * descriptor, and collects packets.
 *
 * WHAT V5 CHANGED, against the gen4 file this one is shaped after:
 *
 *  1. **start_chn / stop_chn, not StartRecvFrame / StopRecvFrame.** Same
 *     meaning -- ot_venc_start_param.recv_pic_num is -1 for "until
 *     stopped" -- and the same rule that a channel which is not receiving
 *     is not a legal bind destination.
 *
 *  2. **Rate control is flat and one-based.** gen4 tagged the union with a
 *     per-codec enumerator drawn from three separate enums; V5 has one
 *     enum with the codec baked into the name (H264_CBR is 2, H265_CBR is
 *     14), so choosing the mode is one function and getting it wrong is a
 *     rejected create rather than a silently misread union.
 *
 *  3. **A pack may be more than one NAL.** ot_venc_pack grew data_num and
 *     pack_info[], and nothing in the vendor's own samples reads them --
 *     the sample writer walks packs and writes addr + offset for len -
 *     offset, exactly as gen4 did. That is plan risk R9: if data_num comes
 *     back 1 the two generations agree, and if it comes back greater the
 *     pack has to be split or every consumer downstream sees several NALs
 *     glued into one. hisi_enc_fill_nals handles both and says in the log
 *     which it saw, once, so the bench answers the question rather than
 *     the header.
 *
 * OP COVERAGE
 *
 * Published: create/destroy group (bookkeeping -- see below), create/
 * destroy channel, register/unregister channel, start, stop, poll, get/
 * release frame, request IDR, set rc_mode/bitrate/gop/fps, get/set JPEG
 * quality, get channel attr, get fps, get avg bitrate, query, get fd.
 *
 * Absent, and why:
 *
 *  - enc_set_qp_bounds. On gen4 these are ot_venc_rc_param's, behind a
 *    second get-modify-set call, and writing them was worth an hour of
 *    measurement there. V5's equivalent is not bound in v5_venc.h at all,
 *    so publishing the op would mean transcribing a structure this backend
 *    has not checked against the board. The channel runs at the driver's
 *    bounds, which is what every gen4 channel did before that work.
 *
 *  - enc_get_rmem_info and enc_inject_stream_shm, which together enable
 *    rvd's refmode. Not established as safe here: V5's stream buffer is
 *    the same per-channel ring gen4 has, and the SigmaStar backend
 *    measured the equivalent to be unsafe on MI. Leaving both
 *    unimplemented is what keeps refmode off.
 *
 *  - ROI, GDR, p-skip, super-frame, entropy mode, crop. Each exists on V5
 *    through its own structure; none is asked for by rvd on this SoC.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/select.h>
#include <unistd.h>

/* ================================================================
 * CHANNEL LOOKUP
 * ================================================================ */

#define HISI_ENC_ENTER(ctx, chn, st_var, enc_var)                                                  \
    hisi_state_t *st_var = hisi_state(ctx);                                                        \
    hisi_venc_chn_t *enc_var;                                                                      \
    do {                                                                                           \
        if (!st_var)                                                                               \
            return RSS_ERR_INVAL;                                                                  \
        if ((chn) < 0 || (chn) >= HISI_VENC_CHN_NUM) {                                             \
            HAL_LOG_ERR("venc: channel %d out of range [0,%d)", (chn), HISI_VENC_CHN_NUM);         \
            return RSS_ERR_INVAL;                                                                  \
        }                                                                                          \
        enc_var = &st_var->enc[chn];                                                               \
    } while (0)

/* ================================================================
 * CODEC AND RATE CONTROL
 * ================================================================ */

/*
 * The payload type, which on V5 is also what decides half the rate-control
 * enum. JPEG and MJPEG both land on PT_MJPEG for gen4's reason: an MJPEG
 * channel emits the same JPEG bytes a snapshot wants, and its FIXQP rc
 * carries the qfactor, so quality works through the ordinary reconfigure
 * path where PT_JPEG would need ss_mpi_venc_set_jpeg_param's separate
 * surface and a channel that is started one frame at a time.
 */
static v5_payload_type hisi_enc_payload(rss_codec_t codec)
{
    switch (codec) {
    case RSS_CODEC_H265:
        return V5_PT_H265;
    case RSS_CODEC_JPEG:
    case RSS_CODEC_MJPEG:
        return V5_PT_MJPEG;
    case RSS_CODEC_H264:
    default:
        return V5_PT_H264;
    }
}

/*
 * hisi_enc_rc_mode -- raptor's six rate-control modes onto V5's twenty-one.
 *
 * The enum is flat and one-based, so the codec is part of the choice
 * rather than of the struct: picking H264_CBR on an H.265 channel is
 * rejected at create with OT_ERR_VENC_ILLEGAL_PARAM.
 *
 * The collapsing is gen4's and each case is the same decision:
 *
 *   FIXQP                          -> FIXQP, the only shape that takes QPs
 *   CBR                            -> CBR
 *   VBR, SMART, CAPPED_VBR,        -> VBR, whose bit_rate field means the
 *   CAPPED_QUALITY                    maximum.
 *
 * AVBR, QVBR and CVBR exist on V5 and are closer analogues of the last
 * three, but each needs parameters rvd does not carry -- CVBR alone wants
 * four extra windows and bitrates -- and inventing them would be inventing
 * a quality target nobody supplied. VBR with a ceiling is the honest
 * approximation.
 *
 * MJPEG has three modes of its own and the H.26x enumerators are rejected
 * outright on a JPEG channel, so it is answered first.
 */
static v5_venc_rc_mode hisi_enc_rc_mode(v5_payload_type payload, rss_rc_mode_t mode)
{
    bool fixqp = mode == RSS_RC_FIXQP;
    bool cbr = mode == RSS_RC_CBR;

    if (payload == V5_PT_MJPEG || payload == V5_PT_JPEG)
        return fixqp ? V5_VENC_RC_MODE_MJPEG_FIXQP : V5_VENC_RC_MODE_MJPEG_CBR;

    if (payload == V5_PT_H265) {
        if (fixqp)
            return V5_VENC_RC_MODE_H265_FIXQP;
        return cbr ? V5_VENC_RC_MODE_H265_CBR : V5_VENC_RC_MODE_H265_VBR;
    }

    if (fixqp)
        return V5_VENC_RC_MODE_H264_FIXQP;
    return cbr ? V5_VENC_RC_MODE_H264_CBR : V5_VENC_RC_MODE_H264_VBR;
}

/*
 * bps -> kbps, clamped into the range the driver accepts.
 *
 * raptor carries bitrates in bits per second; every HiMPP rate-control
 * struct carries them in **kbps**, on V5 as on gen4. Handing the driver a
 * bps value fails channel creation outright, naming no field.
 *
 * Clamped rather than rejected: a caller asking for 1 kbps means "as low
 * as you can go", and refusing the whole channel over a knob with a
 * defensible nearest value would be worse. The conversion rounds up so a
 * sub-kilobit request never becomes zero.
 */
#define HISI_RC_MIN_KBPS 2u
#define HISI_RC_MAX_KBPS 614400u

/*
 * Seconds the rate controller averages over.
 *
 * 4, which is majestic's value and the one gen4 arrived at by measurement.
 * With 1 -- the vendor sample's value -- an unbounded H.265 channel at gop
 * 40 / 20 fps sawtoothed 1.64-1.71x in per-frame acutance and emitted
 * every IDR twice; at 4 the same channel is flat at 1.02x with clean
 * single IDRs. The plausible mechanism is that a 1 s window is shorter
 * than the 2 s GOP, so the controller never accounts a whole GOP. Carried
 * over rather than re-measured on V5: the measurement is about the
 * controller's window against the GOP, which V5 did not change, and the
 * bench will say if this family disagrees.
 */
#define HISI_RC_STATS_TIME 4u

/* ot_venc_h264_attr.frame_buf_ratio, as a percentage. The vendor's own
 * default; see hisi_enc_fill_attr. */
#define HISI_VENC_FRAME_BUF_RATIO 75u

static unsigned int hisi_enc_kbps(unsigned int bps)
{
    unsigned int kbps = (bps + 999u) / 1000u;

    if (kbps < HISI_RC_MIN_KBPS)
        kbps = HISI_RC_MIN_KBPS;
    if (kbps > HISI_RC_MAX_KBPS)
        kbps = HISI_RC_MAX_KBPS;
    return kbps;
}

/*
 * The JPEG quality scale, from rvd's init_qp. rvd stores JPEG *quality*,
 * 1..100 with higher better, and the driver's qfactor runs 1..99 the same
 * way up -- so this is a clamp, not a QP inversion.
 */
static unsigned int hisi_enc_qfactor(int init_qp)
{
    unsigned int q = init_qp > 0 ? (unsigned int)init_qp : 80u;

    if (q > 99u)
        q = 99u;
    return q;
}

/*
 * hisi_enc_fill_rc -- the rate-control half of ot_venc_chn_attr.
 *
 * src is the rate frames actually arrive at, dst the rate asked for. The
 * VPSS channel is the pipeline's one frame dropper, so the delivery rate
 * is its dst when it drops and the sensor's rate when it does not; writing
 * the *requested* rate into src makes the encoder's frame-rate control
 * drop nothing and the rate controller budget for a rate the pipeline is
 * not delivering, so a CBR stream overshoots by src/dst. dst is clamped to
 * src because the encoder cannot invent frames.
 */
static void hisi_enc_fill_rc(const hisi_state_t *st, const hisi_venc_chn_t *enc,
                             v5_venc_rc_attr *rc)
{
    unsigned int src_fps =
        st->mode.frame_rate > 0 ? (unsigned int)(st->mode.frame_rate + 0.5f) : 25u;
    unsigned int req, dst_fps;

    if (enc->bound_fs >= 0 && enc->bound_fs < HISI_VPSS_CHN_NUM &&
        st->fs[enc->bound_fs].frame_rate.dst_frame_rate > 0)
        src_fps = (unsigned int)st->fs[enc->bound_fs].frame_rate.dst_frame_rate;
    if (!src_fps)
        src_fps = 25u;

    /* Rounded, not truncated: 30000/1001 is 30, not 29. */
    req =
        enc->fps_num && enc->fps_den ? (enc->fps_num + enc->fps_den / 2u) / enc->fps_den : src_fps;
    if (!req)
        req = 1u;
    dst_fps = req < src_fps ? req : src_fps;

    memset(rc, 0, sizeof(*rc));
    rc->rc_mode = hisi_enc_rc_mode(enc->payload, enc->rc_mode);

    switch (rc->rc_mode) {
    case V5_VENC_RC_MODE_MJPEG_FIXQP:
        /* src == dst on purpose: a snapshot channel is paced by rvd's
         * pulse loop (start, one frame, stop), and a frame-rate
         * controller in front of it would make the first frame after a
         * start wait out the drop pattern. */
        rc->attr.mjpeg_fixqp.src_frame_rate = src_fps;
        rc->attr.mjpeg_fixqp.dst_frame_rate = src_fps;
        rc->attr.mjpeg_fixqp.qfactor = hisi_enc_qfactor(enc->init_qp);
        return;
    case V5_VENC_RC_MODE_MJPEG_CBR:
        rc->attr.mjpeg_cbr.stats_time = 1;
        rc->attr.mjpeg_cbr.src_frame_rate = src_fps;
        rc->attr.mjpeg_cbr.dst_frame_rate = dst_fps;
        rc->attr.mjpeg_cbr.bit_rate = hisi_enc_kbps(enc->bitrate);
        return;
    case V5_VENC_RC_MODE_H264_FIXQP:
    case V5_VENC_RC_MODE_H265_FIXQP:
        rc->attr.h264_fixqp.gop = enc->gop;
        rc->attr.h264_fixqp.src_frame_rate = src_fps;
        rc->attr.h264_fixqp.dst_frame_rate = dst_fps;
        rc->attr.h264_fixqp.i_qp = enc->init_qp > 0 ? (unsigned int)enc->init_qp : 28u;
        rc->attr.h264_fixqp.p_qp = rc->attr.h264_fixqp.i_qp + 2u;
        rc->attr.h264_fixqp.b_qp = rc->attr.h264_fixqp.p_qp;
        return;
    case V5_VENC_RC_MODE_H264_VBR:
    case V5_VENC_RC_MODE_H265_VBR:
        rc->attr.h264_vbr.gop = enc->gop;
        rc->attr.h264_vbr.stats_time = HISI_RC_STATS_TIME;
        rc->attr.h264_vbr.src_frame_rate = src_fps;
        rc->attr.h264_vbr.dst_frame_rate = dst_fps;
        rc->attr.h264_vbr.max_bit_rate = hisi_enc_kbps(enc->bitrate);
        return;
    default:
        rc->attr.h264_cbr.gop = enc->gop;
        rc->attr.h264_cbr.stats_time = HISI_RC_STATS_TIME;
        rc->attr.h264_cbr.src_frame_rate = src_fps;
        rc->attr.h264_cbr.dst_frame_rate = dst_fps;
        rc->attr.h264_cbr.bit_rate = hisi_enc_kbps(enc->bitrate);
        return;
    }
}

/*
 * hisi_enc_fill_attr -- the whole channel attribute, built fresh.
 *
 * Every field comes from the channel's own tracked state, never from rvd's
 * config, so that hisi_enc_reconfigure can rebuild the struct without a
 * config in hand and without any field quietly reverting to a default.
 *
 * is_by_frame is true, which is what makes get_stream return whole frames.
 * With it false a pack is a slice and every consumer downstream would have
 * to reassemble them -- and the R9 question would change shape.
 */
static void hisi_enc_fill_attr(const hisi_state_t *st, const hisi_venc_chn_t *enc,
                               v5_venc_chn_attr *attr)
{
    memset(attr, 0, sizeof(*attr));

    attr->venc_attr.type = enc->payload;
    attr->venc_attr.max_pic_width = enc->width;
    attr->venc_attr.max_pic_height = enc->height;
    attr->venc_attr.pic_width = enc->width;
    attr->venc_attr.pic_height = enc->height;
    attr->venc_attr.buf_size = enc->buf_size;
    attr->venc_attr.profile = enc->profile;
    attr->venc_attr.is_by_frame = 1;

    /*
     * Share the reconstruction and reference buffers: a straight DDR
     * saving on a part whose whole MMZ is 32 MB. It has no meaning for
     * JPEG, whose union arm is a different struct entirely.
     *
     * frame_buf_ratio is new against gen4 and is **not optional**: it
     * scales the encoder's internal frame buffer as a percentage, and 0 is
     * rejected -- ss_mpi_venc_create_chn returns 0xa0088007
     * (VENC / ILLEGAL_PARAM) naming no field, which is the whole of the
     * diagnosis this call offers. 75 is the vendor's own default
     * (SAMPLE_FRAME_BUF_RATIO_DEFAULT, sample_comm.h:121), set for every
     * H.264, H.265 and SVAC3 channel its samples create.
     */
    if (enc->payload == V5_PT_H264 || enc->payload == V5_PT_H265) {
        attr->venc_attr.codec.h264_attr.rcn_ref_share_buf_en = 1;
        attr->venc_attr.codec.h264_attr.frame_buf_ratio = HISI_VENC_FRAME_BUF_RATIO;
    }

    hisi_enc_fill_rc(st, enc, &attr->rc_attr);

    /* NORMAL_P: every frame after the I is a P referencing the one
     * before. SMART_P encodes a long-term background reference and
     * changes which arm of the GOP union is read, so it is not a drop-in
     * swap and rvd's gop_mode is not wired through on this family yet. */
    attr->gop_attr.gop_mode = V5_VENC_GOP_MODE_NORMAL_P;
    attr->gop_attr.attr.normal_p.ip_qp_delta = enc->ip_qp_delta;
}

/* ================================================================
 * GROUPS
 * ================================================================ */

/*
 * HiMPP has no encoder group. A VENC channel is bound directly to a VPSS
 * channel, so there is no object between them for a group to be.
 *
 * The ops are published anyway and do nothing, exactly as the gen4 and
 * SigmaStar backends do, because rvd calls them unconditionally during
 * pipeline construction and a NOTSUP return would be read as a failure.
 */
int hal_enc_create_group(void *ctx, int grp)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    if (grp < 0 || grp >= HISI_VENC_CHN_NUM)
        return RSS_ERR_INVAL;

    return RSS_OK;
}

int hal_enc_destroy_group(void *ctx, int grp)
{
    return hal_enc_create_group(ctx, grp);
}

/* ================================================================
 * CHANNELS
 * ================================================================ */

/*
 * hisi_enc_start_chn -- ss_mpi_venc_start_chn, for good.
 *
 * recv_pic_num = -1 means "until stopped". Zero is rejected by the driver,
 * which is why there is no "start for N frames" path: rvd never asks for
 * one, and the single value that would express it is the one that fails.
 */
static int hisi_enc_start_chn(hisi_state_t *st, int chn, hisi_venc_chn_t *enc)
{
    v5_venc_start_param param;
    int ret;

    if (enc->receiving)
        return RSS_OK;
    if (!st->venc.fnStartChn)
        return RSS_ERR_NOTSUP;

    memset(&param, 0, sizeof(param));
    param.recv_pic_num = -1;

    ret = st->venc.fnStartChn(chn, &param);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_venc_start_chn(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    enc->receiving = true;
    return RSS_OK;
}

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg)
{
    v5_venc_chn_attr attr;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!cfg)
        return RSS_ERR_INVAL;
    if (!st->venc.fnCreateChn)
        return RSS_ERR_NOTSUP;
    if (enc->created) {
        HAL_LOG_ERR("venc chn %d: already created", chn);
        return RSS_ERR_BUSY;
    }
    if (!cfg->width || !cfg->height) {
        HAL_LOG_ERR("venc chn %d: zero geometry", chn);
        return RSS_ERR_INVAL;
    }

    memset(enc, 0, sizeof(*enc));
    enc->codec = cfg->codec;
    enc->payload = hisi_enc_payload(cfg->codec);
    enc->width = cfg->width;
    enc->height = cfg->height;
    enc->rc_mode = cfg->rc_mode;
    enc->bitrate = cfg->bitrate ? cfg->bitrate : cfg->max_bitrate;
    enc->gop = cfg->gop_length ? cfg->gop_length : 50;
    enc->fps_num = cfg->fps_num;
    enc->fps_den = cfg->fps_den ? cfg->fps_den : 1;
    enc->bound_fs = -1;
    enc->idle_fs = -1;
    enc->fd = -1;
    enc->init_qp = cfg->init_qp;
    enc->ip_qp_delta = cfg->ip_delta >= 0 ? cfg->ip_delta : 2;

    /*
     * The encoder's output ring. Undersizing it is the classic HiMPP
     * encoder fault: the channel creates, runs, and drops frames whose
     * packets do not fit, with no error anywhere. A byte per pixel is
     * comfortably above what an H.264 or H.265 IDR needs at any sane
     * bitrate and is what the vendor's samples use. Aligned to 64 because
     * the driver rejects an unaligned stream buffer.
     */
    enc->buf_size = cfg->buf_size ? cfg->buf_size : (unsigned int)enc->width * enc->height;
    /* The JPEG encoder refuses a buffer smaller than the picture at
     * 16-aligned dimensions -- measured on gen4, whose jpege is the same
     * block, and reported by the create as an ILLEGAL_PARAM naming no
     * field. */
    if (enc->payload == V5_PT_MJPEG) {
        unsigned int min = ((enc->width + 15u) & ~15u) * ((enc->height + 15u) & ~15u);

        if (enc->buf_size < min)
            enc->buf_size = min;
    }
    enc->buf_size = (enc->buf_size + 63u) & ~63u;

    /*
     * Profile. H.264 takes 0=baseline, 1=main, 2=high and rvd's config
     * uses the same numbering, so it passes through, defaulting to high.
     * H.265 takes 0=Main and 1=Main10, where rvd's H.264 numbering would
     * ask for Main10 on a config that meant High -- so it is pinned to
     * Main, the only profile this silicon encodes at 8 bits anyway. JPEG
     * has one profile.
     */
    if (enc->payload == V5_PT_H264)
        enc->profile = cfg->profile >= 0 && cfg->profile <= 2 ? (unsigned int)cfg->profile : 2u;
    else
        enc->profile = 0;

    hisi_enc_fill_attr(st, enc, &attr);
    ret = st->venc.fnCreateChn(chn, &attr);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_venc_create_chn(%d) %ux%u codec %d failed: 0x%x", chn, enc->width,
                    enc->height, (int)enc->codec, ret);
        enc->bound_fs = -1;
        enc->idle_fs = -1;
        enc->fd = -1;
        return RSS_ERR_IO;
    }

    enc->created = true;

    /*
     * Start receiving here, as part of create, for the structural reason
     * hal_fs_create_channel enables its VPSS channel: HiMPP's order is
     * create -> start -> bind, and raptor's is create -> bind -> start. A
     * channel that is not receiving is not a legal bind destination.
     *
     * MJPEG channels start nothing here. rvd manages a snapshot channel's
     * whole duty cycle through enc_start/enc_stop and believes a fresh
     * channel is idle -- a channel receiving from create is one rvd will
     * never stop, and with no consumer draining it the encoder's output
     * fills, it stalls holding its input pictures, and a handful of queued
     * frames is this part's whole VB pool. The bind is deferred with the
     * receive; see hisi_bind_vpss_venc.
     */
    if (enc->payload != V5_PT_MJPEG) {
        ret = hisi_enc_start_chn(st, chn, enc);
        if (ret) {
            if (st->venc.fnDestroyChn)
                st->venc.fnDestroyChn(chn);
            enc->created = false;
            return ret;
        }
    }

    HAL_LOG_INFO("venc chn %d: %ux%u codec %d, %u bps (%u kbps to the driver), gop %u, buf %u", chn,
                 enc->width, enc->height, (int)enc->codec, enc->bitrate,
                 hisi_enc_kbps(enc->bitrate), enc->gop, attr.venc_attr.buf_size);
    return RSS_OK;
}

int hal_enc_destroy_channel(void *ctx, int chn)
{
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_OK;

    /* Order: release any held stream, stop receiving, unbind, close the
     * fd, then destroy. Each step undoes exactly one thing create-and-
     * start did, and destroying a channel that is still bound leaves VPSS
     * holding a reference to a channel that no longer exists. */
    if (enc->frame_held)
        hal_enc_release_frame(ctx, chn, NULL);

    if (enc->receiving)
        hal_enc_stop(ctx, chn);

    if (enc->bound_fs >= 0)
        hisi_unbind_vpss_venc(st, enc->bound_fs, chn);

    if (enc->fd >= 0) {
        /* V5 has no close_fd: the descriptor belongs to the channel and
         * goes away with it. Forgetting it is the whole of the cleanup,
         * and closing it by hand would close a descriptor the driver
         * still owns. */
        enc->fd = -1;
    }

    ret = st->venc.fnDestroyChn ? st->venc.fnDestroyChn(chn) : 0;
    if (ret)
        HAL_LOG_WARN("ss_mpi_venc_destroy_chn(%d) failed: 0x%x", chn, ret);

    memset(enc, 0, sizeof(*enc));
    enc->bound_fs = -1;
    enc->idle_fs = -1;
    enc->fd = -1;
    return RSS_OK;
}

/*
 * hal_enc_register_channel -- bind a VPSS channel to this VENC channel.
 *
 * rvd's "group" is the framesource channel number, which is what makes
 * this op the bind: on a family with no encoder group, registering channel
 * C into group G means "G's pictures go to C".
 */
int hal_enc_register_channel(void *ctx, int grp, int chn)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (grp < 0 || grp >= HISI_VPSS_CHN_NUM) {
        HAL_LOG_ERR("venc chn %d: group %d is not a VPSS channel [0,%d)", chn, grp,
                    HISI_VPSS_CHN_NUM);
        return RSS_ERR_INVAL;
    }
    if (!enc->created) {
        HAL_LOG_ERR("venc chn %d: register before create", chn);
        return RSS_ERR_INVAL;
    }
    if (enc->bound_fs == grp || enc->idle_fs == grp)
        return RSS_OK;
    if (enc->bound_fs >= 0) {
        HAL_LOG_ERR("venc chn %d: already bound to framesource %d", chn, enc->bound_fs);
        return RSS_ERR_BUSY;
    }

    return hisi_bind_vpss_venc(st, grp, chn);
}

int hal_enc_unregister_channel(void *ctx, int chn)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (enc->bound_fs < 0) {
        enc->idle_fs = -1;
        return RSS_OK;
    }

    return hisi_unbind_vpss_venc(st, enc->bound_fs, chn);
}

/*
 * hal_enc_start / hal_enc_stop -- receive frames, or stop receiving.
 *
 * JPEG channels are started the same way as any other, which is a
 * deliberate departure from how the vendor samples drive them: rvd owns
 * the snapshot loop, calling enc_start when a consumer appears and
 * enc_stop when the last one leaves, and pacing the frames in between
 * itself. Special-casing JPEG here to "not really start" would make
 * enc_start a lie on exactly the channel whose lifetime rvd manages most
 * carefully.
 */
int hal_enc_start(void *ctx, int chn)
{
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;

    /* Receive first, so the channel is a legal bind destination, then the
     * other half of enc_stop's unbind: remake the deferred edge. */
    ret = hisi_enc_start_chn(st, chn, enc);
    if (ret)
        return ret;

    if (enc->bound_fs < 0 && enc->idle_fs >= 0) {
        int fs = enc->idle_fs;

        enc->idle_fs = -1;
        ret = hisi_bind_vpss_venc(st, fs, chn);
        if (ret) {
            enc->idle_fs = fs;
            if (st->venc.fnStopChn) {
                st->venc.fnStopChn(chn);
                enc->receiving = false;
            }
            return ret;
        }
    }

    return RSS_OK;
}

int hal_enc_stop(void *ctx, int chn)
{
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->receiving)
        return RSS_OK;
    if (!st->venc.fnStopChn)
        return RSS_ERR_NOTSUP;

    ret = st->venc.fnStopChn(chn);
    if (ret)
        HAL_LOG_WARN("ss_mpi_venc_stop_chn(%d) failed: 0x%x", chn, ret);

    enc->receiving = false;

    /*
     * A duty-cycled MJPEG channel must not stay bound while stopped: the
     * VPSS source keeps queueing pictures at a stopped destination and
     * nothing releases them, which on gen4 held the whole of pool 0 and
     * starved VI within a second. Unbind, remember the edge in idle_fs,
     * and let enc_start remake it; reset_chn then flushes whatever was
     * queued before the unbind. H.26x channels stay bound across a stop.
     */
    if (enc->payload == V5_PT_MJPEG && enc->bound_fs >= 0) {
        int fs = enc->bound_fs;

        hisi_unbind_vpss_venc(st, fs, chn);
        enc->idle_fs = fs;
        if (!enc->frame_held && st->venc.fnResetChn) {
            ret = st->venc.fnResetChn(chn);
            if (ret)
                HAL_LOG_WARN("ss_mpi_venc_reset_chn(%d) failed: 0x%x", chn, ret);
        }
    }

    return RSS_OK;
}

/* ================================================================
 * STREAM COLLECTION
 * ================================================================ */

int hal_enc_get_fd(void *ctx, int chn)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;
    if (!st->venc.fnGetFd)
        return RSS_ERR_NOTSUP;

    if (enc->fd < 0) {
        int fd = st->venc.fnGetFd(chn);

        if (fd < 0) {
            HAL_LOG_ERR("ss_mpi_venc_get_fd(%d) failed: %d", chn, fd);
            return RSS_ERR_IO;
        }
        enc->fd = fd;
    }

    return enc->fd;
}

/*
 * hal_enc_poll -- wait for the channel to have a frame.
 *
 * The descriptor from ss_mpi_venc_get_fd becomes readable when a frame is
 * ready. A zero timeout is a legitimate poll, so it is passed through
 * rather than treated as "block forever".
 *
 * Return values are the ones rvd's encoder thread distinguishes: RSS_OK
 * for ready, -EAGAIN for a timeout, RSS_ERR_IO for a broken descriptor.
 * EINTR is a timeout rather than an error -- a signal arriving mid-wait
 * says nothing about the encoder.
 */
int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms)
{
    struct timeval tv;
    fd_set rfds;
    int fd;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;

    fd = hal_enc_get_fd(ctx, chn);
    if (fd < 0)
        return fd;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    tv.tv_sec = (time_t)(timeout_ms / 1000u);
    tv.tv_usec = (suseconds_t)((timeout_ms % 1000u) * 1000u);

    ret = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (ret < 0) {
        if (errno == EINTR)
            return -EAGAIN;
        HAL_LOG_ERR("venc chn %d: select failed: %s", chn, strerror(errno));
        return RSS_ERR_IO;
    }
    if (ret == 0 || !FD_ISSET(fd, &rfds))
        return -EAGAIN;

    return RSS_OK;
}

/*
 * hisi_enc_nal_type -- classify one pack, or one sub-packet of one.
 *
 * The type union is read as H.264's enumeration or H.265's depending on
 * the channel's codec, and the two disagree on every value that matters:
 * IDR is 5 against 19, SPS is 7 against 33. Reading the wrong one marks
 * parameter sets as slices, which downstream looks like a stream that
 * never sends an SPS.
 */
static rss_nal_type_t hisi_enc_nal_type(rss_codec_t codec, int data_type)
{
    if (codec == RSS_CODEC_JPEG || codec == RSS_CODEC_MJPEG)
        return RSS_NAL_JPEG_FRAME;

    if (codec == RSS_CODEC_H265) {
        switch (data_type) {
        case V5_VENC_H265_NALU_VPS:
            return RSS_NAL_H265_VPS;
        case V5_VENC_H265_NALU_SPS:
            return RSS_NAL_H265_SPS;
        case V5_VENC_H265_NALU_PPS:
            return RSS_NAL_H265_PPS;
        case V5_VENC_H265_NALU_SEI:
            return RSS_NAL_H265_SEI;
        case V5_VENC_H265_NALU_IDR_SLICE:
            return RSS_NAL_H265_IDR;
        case V5_VENC_H265_NALU_I_SLICE:
        case V5_VENC_H265_NALU_P_SLICE:
        case V5_VENC_H265_NALU_B_SLICE:
            return RSS_NAL_H265_SLICE;
        default:
            return RSS_NAL_UNKNOWN;
        }
    }

    switch (data_type) {
    case V5_VENC_H264_NALU_SPS:
        return RSS_NAL_H264_SPS;
    case V5_VENC_H264_NALU_PPS:
        return RSS_NAL_H264_PPS;
    case V5_VENC_H264_NALU_SEI:
        return RSS_NAL_H264_SEI;
    case V5_VENC_H264_NALU_IDR_SLICE:
        return RSS_NAL_H264_IDR;
    case V5_VENC_H264_NALU_I_SLICE:
    case V5_VENC_H264_NALU_P_SLICE:
    case V5_VENC_H264_NALU_B_SLICE:
        return RSS_NAL_H264_SLICE;
    default:
        return RSS_NAL_UNKNOWN;
    }
}

/*
 * hisi_enc_pack_splits -- does this pack describe sub-packets, and are its
 * descriptions usable?
 *
 * PLAN RISK R9, decided per pack rather than per family. data_num > 1 says
 * the pack carries several NALs; pack_info[] says where each begins. Both
 * are trusted only if every entry lies inside the pack and no entry is
 * empty -- a driver that fills data_num but leaves the array at zero would
 * otherwise produce a frame of empty NALs, which is worse than one NAL
 * carrying several.
 *
 * Returns the usable sub-packet count, or 0 for "treat the pack as one
 * NAL".
 */
static unsigned int hisi_enc_pack_splits(const v5_venc_pack *pack)
{
    unsigned int n = pack->data_num;
    unsigned int i;

    if (n < 2 || n > V5_VENC_MAX_PACK_INFO_NUM)
        return 0;

    for (i = 0; i < n; i++) {
        const v5_venc_pack_info *info = &pack->pack_info[i];

        if (!info->pack_len)
            return 0;
        if (info->pack_offset > pack->len)
            return 0;
        if (info->pack_len > pack->len - info->pack_offset)
            return 0;
    }

    return n;
}

/*
 * hisi_enc_fill_nals -- packs become rss_nal_unit_t.
 *
 * offset is the length of the header *inside* the packet, not an offset
 * into a larger buffer: the payload runs from addr + offset to addr + len.
 * A reader that ignores it emits the Annex-B start code twice; one that
 * subtracts it from the wrong base emits a truncated NAL. An offset larger
 * than the length would make the subtraction wrap, so it is treated as "no
 * offset" rather than trusted.
 *
 * The one-time log is how the bench answers R9: whichever shape this board
 * actually produces says so in the first frame and never again.
 */
static void hisi_enc_fill_nals(hisi_state_t *st, int chn, hisi_venc_chn_t *enc, rss_frame_t *frame)
{
    static bool split_logged;
    static bool flat_logged;
    unsigned int packs = enc->stream.pack_cnt;
    unsigned int out = 0;
    unsigned int i, j;

    if (packs > HISI_VENC_MAX_PACKS) {
        HAL_LOG_WARN("venc chn %d: %u packs in one frame, reporting %d", chn, packs,
                     HISI_VENC_MAX_PACKS);
        packs = HISI_VENC_MAX_PACKS;
    }

    frame->is_key = false;

    for (i = 0; i < packs; i++) {
        v5_venc_pack *pack = &enc->packs[i];
        unsigned int splits = hisi_enc_pack_splits(pack);
        unsigned int offset = pack->offset;

        if (offset > pack->len)
            offset = 0;

        if (splits) {
            if (!split_logged) {
                split_logged = true;
                HAL_LOG_INFO("venc chn %d: a pack carries %u NALs (ot_venc_pack.data_num); "
                             "splitting through pack_info",
                             chn, splits);
            }
            for (j = 0; j < splits && out < HISI_VENC_MAX_NALS; j++, out++) {
                const v5_venc_pack_info *info = &pack->pack_info[j];
                rss_nal_unit_t *nal = &enc->nals[out];
                rss_nal_type_t type = hisi_enc_nal_type(enc->codec, info->pack_type.raw);

                nal->data = pack->addr ? pack->addr + info->pack_offset : NULL;
                nal->length = info->pack_len;
                nal->type = type;
                /* Only the last sub-packet of the last pack ends a
                 * frame, and the pack's own flag is what says so. */
                nal->frame_end = pack->is_frame_end != 0 && j + 1 == splits;

                if (type == RSS_NAL_H264_IDR || type == RSS_NAL_H265_IDR)
                    frame->is_key = true;
            }
        } else {
            rss_nal_unit_t *nal;
            rss_nal_type_t type;

            if (out >= HISI_VENC_MAX_NALS)
                break;
            if (!flat_logged && !split_logged && enc->codec != RSS_CODEC_JPEG &&
                enc->codec != RSS_CODEC_MJPEG) {
                flat_logged = true;
                HAL_LOG_INFO("venc chn %d: one pack is one NAL (data_num %u); walking the pack "
                             "array",
                             chn, pack->data_num);
            }

            nal = &enc->nals[out++];
            type = hisi_enc_nal_type(enc->codec, pack->data_type.raw);
            nal->data = pack->addr ? pack->addr + offset : NULL;
            nal->length = pack->len - offset;
            nal->type = type;
            nal->frame_end = pack->is_frame_end != 0;

            if (type == RSS_NAL_H264_IDR || type == RSS_NAL_H265_IDR)
                frame->is_key = true;
        }
    }

    if (out == HISI_VENC_MAX_NALS && packs)
        HAL_LOG_WARN("venc chn %d: frame filled the %d-entry NAL array; some may be missing", chn,
                     HISI_VENC_MAX_NALS);

    frame->nals = enc->nals;
    frame->nal_count = out;
    (void)st;
}

/*
 * hal_enc_get_frame -- check out one encoded frame.
 *
 * query_status first, because get_stream copies into a caller-allocated
 * pack array and needs to be told how big it is. cur_packs == 0 is normal:
 * the descriptor signals readiness slightly ahead of the pack being
 * complete. Report it as -EAGAIN, the "no frame this time" every other
 * backend returns and the only value rvd's encoder thread treats as
 * non-fatal.
 *
 * A frame with more packs than the per-channel array holds is drained into
 * a temporary array and dropped. Clamping does not work: with is_by_frame
 * set, get_stream refuses a pack array smaller than the frame rather than
 * partially filling it, so a clamp leaves the frame queued, the descriptor
 * ready, and this op failing identically forever -- a dead stream with a
 * busy poll loop in front of it. Draining loses one frame and keeps the
 * channel.
 */
int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame)
{
    v5_venc_chn_status status;
    unsigned int packs;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!frame)
        return RSS_ERR_INVAL;
    if (!enc->created)
        return RSS_ERR_INVAL;
    if (!st->venc.fnQueryStatus || !st->venc.fnGetStream)
        return RSS_ERR_NOTSUP;
    if (enc->frame_held) {
        HAL_LOG_ERR("venc chn %d: get_frame with a frame still held", chn);
        return RSS_ERR_BUSY;
    }

    memset(&status, 0, sizeof(status));
    ret = st->venc.fnQueryStatus(chn, &status);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_venc_query_status(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }
    if (!status.cur_packs)
        return -EAGAIN;

    packs = status.cur_packs;
    if (packs > HISI_VENC_MAX_PACKS) {
        v5_venc_pack *tmp = (v5_venc_pack *)calloc(packs, sizeof(*tmp));
        v5_venc_stream drain;

        HAL_LOG_ERR("venc chn %d: frame carries %u packs, array holds %d; dropping the frame", chn,
                    packs, HISI_VENC_MAX_PACKS);
        if (!tmp)
            return RSS_ERR_NOMEM;
        memset(&drain, 0, sizeof(drain));
        drain.pack = tmp;
        drain.pack_cnt = packs;
        ret = st->venc.fnGetStream(chn, &drain, 0);
        if (!ret)
            st->venc.fnReleaseStream(chn, &drain);
        free(tmp);
        return ret ? RSS_ERR_IO : -EAGAIN;
    }

    memset(&enc->stream, 0, sizeof(enc->stream));
    memset(enc->packs, 0, sizeof(enc->packs));
    enc->stream.pack = enc->packs;
    enc->stream.pack_cnt = packs;

    /* Zero timeout: the descriptor already said a frame is ready, and this
     * call moves descriptors rather than pixels. */
    ret = st->venc.fnGetStream(chn, &enc->stream, 0);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_venc_get_stream(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    enc->frame_held = true;

    memset(frame, 0, sizeof(*frame));
    frame->codec = enc->codec;
    frame->seq = enc->stream.seq;
    /* HiMPP timestamps packs in microseconds, the unit rss_frame_t wants;
     * the first pack carries the frame's capture time. */
    frame->timestamp = enc->stream.pack_cnt ? (int64_t)enc->packs[0].pts : 0;
    hisi_enc_fill_nals(st, chn, enc, frame);
    frame->_priv = enc;

    return RSS_OK;
}

int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame)
{
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->frame_held)
        return RSS_OK;
    if (!st->venc.fnReleaseStream)
        return RSS_ERR_NOTSUP;

    ret = st->venc.fnReleaseStream(chn, &enc->stream);
    if (ret)
        HAL_LOG_WARN("ss_mpi_venc_release_stream(%d) failed: 0x%x", chn, ret);

    enc->frame_held = false;
    memset(&enc->stream, 0, sizeof(enc->stream));

    if (frame) {
        frame->nals = NULL;
        frame->nal_count = 0;
        frame->_priv = NULL;
    }

    return RSS_OK;
}

int hal_enc_query(void *ctx, int chn, bool *busy)
{
    v5_venc_chn_status status;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!busy)
        return RSS_ERR_INVAL;
    if (!enc->created) {
        /* Not an error: rvd polls channels it has not built yet during
         * pipeline construction, and "no channel" is not busy. */
        *busy = false;
        return RSS_OK;
    }
    if (!st->venc.fnQueryStatus)
        return RSS_ERR_NOTSUP;

    memset(&status, 0, sizeof(status));
    ret = st->venc.fnQueryStatus(chn, &status);
    if (ret)
        return RSS_ERR_IO;

    /* Busy means "has work outstanding": frames waiting to be encoded, or
     * encoded bytes waiting to be collected. */
    *busy = status.left_pics != 0 || status.left_stream_frames != 0;
    return RSS_OK;
}

/* ================================================================
 * RUNTIME RECONFIGURATION
 * ================================================================ */

/*
 * hisi_enc_reconfigure -- write the channel attribute back.
 *
 * The single path every rate knob funnels through, because HiMPP has no
 * per-knob setter: bitrate, GOP and frame rate all live inside
 * ot_venc_chn_attr and the only way to change one is to write the whole
 * struct.
 *
 * Built fresh from the channel's tracked state rather than read back and
 * patched. get_chn_attr would work, but the tracked state is the record of
 * what raptor asked for, and rebuilding from it means two knobs set in
 * sequence both survive -- where a read-modify-write of the driver's copy
 * would silently adopt whatever the driver normalised.
 */
static int hisi_enc_reconfigure(hisi_state_t *st, int chn, hisi_venc_chn_t *enc)
{
    v5_venc_chn_attr attr;
    int ret;

    if (!enc->created)
        return RSS_OK;
    if (!st->venc.fnSetChnAttr)
        return RSS_ERR_NOTSUP;

    hisi_enc_fill_attr(st, enc, &attr);
    ret = st->venc.fnSetChnAttr(chn, &attr);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_venc_set_chn_attr(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * hisi_enc_refresh_rc -- re-derive the rc attribute after the bind exists.
 *
 * hisi_enc_fill_rc computes the source frame rate from the bound VPSS
 * channel, and a channel is created before it is bound -- so the attribute
 * written at create names the sensor's rate. The frame-rate controller
 * derives its drop pattern from src:dst, and a src above the true delivery
 * rate makes it drop frames it should keep.
 */
void hisi_enc_refresh_rc(hisi_state_t *st, int enc_chn)
{
    hisi_venc_chn_t *enc;
    int ret;

    if (!st || enc_chn < 0 || enc_chn >= HISI_VENC_CHN_NUM)
        return;
    enc = &st->enc[enc_chn];
    if (!enc->created)
        return;

    ret = hisi_enc_reconfigure(st, enc_chn, enc);
    if (ret)
        HAL_LOG_WARN("venc chn %d: rc refresh after bind failed (%d)", enc_chn, ret);
}

/*
 * Every setter below restores the tracked value when the driver refuses
 * the write. Tracked state the driver rejected is not merely a wrong
 * answer from enc_get_channel_attr: hisi_enc_reconfigure rebuilds the
 * whole attribute from tracked state, so a phantom value would be silently
 * re-applied by the next setter that succeeds.
 */
int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate)
{
    rss_rc_mode_t old_mode;
    uint32_t old_bitrate;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    old_mode = enc->rc_mode;
    old_bitrate = enc->bitrate;
    enc->rc_mode = mode;
    if (bitrate)
        enc->bitrate = bitrate;

    ret = hisi_enc_reconfigure(st, chn, enc);
    if (ret) {
        enc->rc_mode = old_mode;
        enc->bitrate = old_bitrate;
    }
    return ret;
}

int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate)
{
    uint32_t old;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!bitrate)
        return RSS_ERR_INVAL;

    old = enc->bitrate;
    enc->bitrate = bitrate;
    ret = hisi_enc_reconfigure(st, chn, enc);
    if (ret)
        enc->bitrate = old;
    return ret;
}

int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length)
{
    uint32_t old;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!gop_length)
        return RSS_ERR_INVAL;

    old = enc->gop;
    enc->gop = gop_length;
    ret = hisi_enc_reconfigure(st, chn, enc);
    if (ret)
        enc->gop = old;
    return ret;
}

int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den)
{
    uint32_t old_num, old_den;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;

    old_num = enc->fps_num;
    old_den = enc->fps_den;
    enc->fps_num = fps_num;
    enc->fps_den = fps_den;
    ret = hisi_enc_reconfigure(st, chn, enc);
    if (ret) {
        enc->fps_num = old_num;
        enc->fps_den = old_den;
    }
    return ret;
}

/*
 * enc_set/get_jpeg_qp -- rvd passes JPEG *quality*, 1..100 higher-better,
 * so despite the op's name there is no QP inversion here. Every JPEG-class
 * channel on this backend is PT_MJPEG -- see hisi_enc_payload -- and its
 * FIXQP rc carries the qfactor, so this is tracked state plus the ordinary
 * reconfigure. Publishing the op is what spares rvd its stop/recreate
 * fallback for a one-field change.
 */
int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp)
{
    int old, ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (qp < 1 || qp > 100)
        return RSS_ERR_INVAL;
    if (!enc->created)
        return RSS_ERR_INVAL;
    if (enc->payload != V5_PT_MJPEG)
        return RSS_ERR_NOTSUP;

    old = enc->init_qp;
    enc->init_qp = qp;
    ret = hisi_enc_reconfigure(st, chn, enc);
    if (ret)
        enc->init_qp = old;
    return ret;
}

int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!qp)
        return RSS_ERR_INVAL;
    if (!enc->created || enc->payload != V5_PT_MJPEG)
        return RSS_ERR_NOTSUP;

    *qp = (int)hisi_enc_qfactor(enc->init_qp);
    return RSS_OK;
}

int hal_enc_request_idr(void *ctx, int chn)
{
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;
    if (!st->venc.fnRequestIdr)
        return RSS_ERR_NOTSUP;

    /* instant: insert the IDR at the next frame rather than at the next
     * GOP boundary, which is what every caller of this op means. */
    ret = st->venc.fnRequestIdr(chn, 1);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_venc_request_idr(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* ================================================================
 * READBACK
 * ================================================================ */

int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!cfg)
        return RSS_ERR_INVAL;
    if (!enc->created)
        return RSS_ERR_INVAL;

    memset(cfg, 0, sizeof(*cfg));
    cfg->codec = enc->codec;
    cfg->width = (uint16_t)enc->width;
    cfg->height = (uint16_t)enc->height;
    cfg->rc_mode = enc->rc_mode;
    cfg->bitrate = enc->bitrate;
    cfg->gop_length = enc->gop;
    cfg->fps_num = enc->fps_num;
    cfg->fps_den = enc->fps_den;
    /* The QP bounds are the driver's on this backend; -1 is
     * rss_video_config_t's own spelling of that. */
    cfg->init_qp = (int16_t)enc->init_qp;
    cfg->min_qp = -1;
    cfg->max_qp = -1;

    return RSS_OK;
}

int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;

    *fps_num = enc->fps_num;
    *fps_den = enc->fps_den ? enc->fps_den : 1;
    return RSS_OK;
}

/*
 * hal_enc_get_avg_bitrate -- the configured target, not a measurement.
 *
 * ot_venc_chn_status carries per-frame byte counts, but averaging them
 * here would need a window and a clock this layer does not own, and rvd
 * already measures throughput from the frames it receives. Answering with
 * the configured number is what the other backends do and what the caller
 * uses it for: reporting the channel's setting.
 */
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!bitrate)
        return RSS_ERR_INVAL;

    *bitrate = enc->bitrate;
    return RSS_OK;
}

/*
 * hisi_enc_release_all -- give back every held stream and tear down every
 * channel, before VPSS and VI go away.
 *
 * Called from hisi_video_teardown, before the framesource release: an
 * encoder still bound to a VPSS channel that is about to be disabled is
 * the state that leaves the kernel side holding buffers.
 */
void hisi_enc_release_all(hisi_state_t *st)
{
    int i;

    if (!st)
        return;

    for (i = 0; i < HISI_VENC_CHN_NUM; i++) {
        hisi_venc_chn_t *enc = &st->enc[i];

        if (!enc->created)
            continue;

        if (enc->frame_held && st->venc.fnReleaseStream) {
            st->venc.fnReleaseStream(i, &enc->stream);
            enc->frame_held = false;
        }
        if (enc->receiving && st->venc.fnStopChn) {
            st->venc.fnStopChn(i);
            enc->receiving = false;
        }
        if (enc->bound_fs >= 0)
            hisi_unbind_vpss_venc(st, enc->bound_fs, i);
        if (st->venc.fnDestroyChn) {
            int ret = st->venc.fnDestroyChn(i);

            if (ret)
                HAL_LOG_WARN("ss_mpi_venc_destroy_chn(%d) failed: 0x%x", i, ret);
        }

        memset(enc, 0, sizeof(*enc));
        enc->bound_fs = -1;
        enc->idle_fs = -1;
        enc->fd = -1;
    }
}
