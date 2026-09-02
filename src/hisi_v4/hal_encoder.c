/*
 * hisi_v4/hal_encoder.c -- VENC channels
 *
 * MAPPING. raptor encoder channel N == VENC channel N. Frames arrive from a
 * VPSS channel through a kernel-side bind, so nothing here pushes pictures;
 * the userspace side configures the channel, waits on a file descriptor,
 * and collects packets.
 *
 * Three things about gen4's encoder shape drive the code below.
 *
 *  1. **One pack per NAL unit.** HiMPP fills pstPack[0..u32PackCount-1] with
 *     one NAL each, which is the shape rss_nal_unit_t was designed around --
 *     unlike SigmaStar's MI, where one pack carries a whole frame and has to
 *     be re-split. So the mapping here is direct and there is no
 *     packetInfo-style sub-structure to reconcile.
 *
 *  2. **The pack array is caller-allocated and caller-sized.** GetStream
 *     copies into pstPack, so QueryStatus has to run first to learn
 *     u32CurPacks. Getting that wrong writes past the array.
 *
 *  3. **Rate control is set at channel creation, through two structures.**
 *     There is no per-knob setter: enc_set_bitrate, enc_set_gop and
 *     enc_set_fps are all read-modify-writes of the whole VENC_CHN_ATTR_S,
 *     which is why the channel's rate state is tracked in hisi_venc_chn_t
 *     rather than re-derived from rvd's config each time. The QP bounds are
 *     not in that structure at all -- they live in VENC_RC_PARAM_S behind a
 *     second call, which hisi_enc_apply_rc_param makes.
 *
 * OP COVERAGE
 *
 * Published: create/destroy group (bookkeeping -- see below), create/destroy
 * channel, register/unregister channel, start, stop, poll, get/release
 * frame, request IDR, set rc_mode/bitrate/gop/fps, get channel attr, get
 * fps, get avg bitrate, query, get fd, set QP bounds.
 *
 * Absent, and why:
 *
 *  - enc_get_rmem_info and enc_inject_stream_shm, which together enable
 *    rvd's *refmode* -- publishing an (offset, length) reference into the
 *    encoder's output memory instead of a copy, read by the consumer after
 *    the HAL has released the frame. Safe on Ingenic, whose encoder writes
 *    each frame to a distinct rmem slot. **Not established as safe here**,
 *    and the SigmaStar backend documents measuring the equivalent to be
 *    unsafe on MI. gen4's stream buffer is a per-channel ring sized by
 *    u32BufSize, which is exactly the shape that recycles. Leaving both ops
 *    unimplemented is what keeps refmode off: rvd falls back to copying
 *    publication when ref_base is zero, which a NOTSUP enc_get_rmem_info
 *    produces. Anyone implementing them has to measure the buffer's reuse
 *    first.
 *
 *  - enc_set_qp, the QP delta pair, and the RC-option bitmask. These share
 *    VENC_RC_PARAM_S with the QP bounds, which are now written
 *    (hisi_enc_apply_rc_param), so the structure is transcribed and the
 *    remaining knobs are a matter of mapping rather than ABI work. They are
 *    left out because rvd does not ask for them on this SoC and because
 *    each needs its own argument about what the mapping should be: a single
 *    "the QP" has no field to land in on a bounded controller, and the
 *    deltas interact with the GOP attribute's ip_qp_delta that is already
 *    set.
 *
 *  - ROI, GDR, p-skip, super-frame, entropy mode, colour-to-grey, buffer
 *    pools, crop. Each exists on gen4 through its own SetParam structure;
 *    none is asked for by rvd on this SoC.
 *
 *  - enc_set_bufshare. HiMPP's buffer sharing is declared at channel
 *    creation through bRcnRefShareBuf, not established between two live
 *    channels, so the op's shape does not fit.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

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

static v4_payload_type hisi_enc_payload(rss_codec_t codec)
{
    switch (codec) {
    case RSS_CODEC_H265:
        return V4_PT_H265;
    case RSS_CODEC_JPEG:
    case RSS_CODEC_MJPEG:
        /*
         * Both on PT_MJPEG, which is divinus's choice too -- the one
         * reference that runs snapshots on this silicon never creates a
         * PT_JPEG channel; its snapshot channels are MJPEG, pulsed one
         * frame at a time, which is also rvd's model. An MJPEG channel
         * emits the same JPEG bytes a snapshot wants, and its FIXQP rc
         * carries the qfactor, so quality works through the ordinary
         * reconfigure path where PT_JPEG would need the separate
         * HI_MPI_VENC_SetJpegParam surface.
         */
        return V4_PT_MJPEG;
    case RSS_CODEC_H264:
    default:
        return V4_PT_H264;
    }
}

/*
 * hisi_enc_fill_rc -- the rate-control half of VENC_CHN_ATTR_S.
 *
 * raptor's six rate-control modes map onto gen4's three shapes. The
 * collapsing is deliberate and each case is a decision:
 *
 *   FIXQP          -> the FIXQP shape, which is the only one that takes QPs.
 *   CBR            -> CBR.
 *   VBR            -> VBR, where the bitrate field means the *maximum*.
 *   SMART,         -> VBR. gen4's AVBR and QVBR exist and are the closer
 *   CAPPED_VBR,       analogues, but each needs parameters
 *   CAPPED_QUALITY    (u32ChangePos, u32MinIQp, the quality-level table)
 *                     that rvd does not carry and that would have to be
 *                     invented. VBR with a max bitrate is the honest
 *                     approximation: it respects the ceiling the caller
 *                     asked for and does not pretend to a quality target
 *                     nobody supplied.
 *
 * JPEG and MJPEG take their own two modes, because the H.264/H.265
 * enumerators are rejected outright on a JPEG channel.
 *
 * The u32Gop field lives here rather than in the GOP attribute, which is
 * about *structure* (normal-P, smart-P) rather than length -- a distinction
 * that costs an hour if you look for GOP length in VENC_GOP_ATTR_S first.
 */
/*
 * bps -> kbps, clamped into the range the driver accepts.
 *
 * raptor carries bitrates in bits per second; every HiMPP rate-control
 * struct carries them in **kbps**, with the header stating Range:[2, 614400]
 * on u32BitRate and u32MaxBitRate alike. Handing the driver a bps value
 * fails channel creation outright -- HI_MPI_VENC_CreateChn returns
 * 0xa0088003, VENC / ERROR / EN_ERR_ILLEGAL_PARAM, naming no field.
 *
 * Clamped rather than rejected: a caller asking for 1 kbps means "as low as
 * you can go", and refusing the whole channel over a knob that has a
 * defensible nearest value would be worse than honouring the intent. The
 * conversion rounds up so that a sub-kilobit request never becomes zero.
 */
#define HISI_RC_MIN_KBPS 2u
#define HISI_RC_MAX_KBPS 614400u

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
 * 1..100 with higher better (rvd_pipeline.c fills it from jpeg_quality),
 * and the driver's Qfactor runs 1..99 the same way up -- so this is a
 * clamp, not a QP inversion. An earlier version here treated the value as
 * an H.264 QP and inverted it, which collapsed qualities 60..100 to one
 * number and made 45 come out *worse* than 30.
 */
static unsigned int hisi_enc_qfactor(int init_qp)
{
    unsigned int q = init_qp > 0 ? (unsigned int)init_qp : 80u;

    if (q > 99u)
        q = 99u;
    return q;
}

static void hisi_enc_fill_rc(const hisi_state_t *st, const hisi_venc_chn_t *enc,
                             v4_venc_rc_attr *rc)
{
    /*
     * src is the rate frames actually arrive at, dst the rate asked for.
     * The VPSS channel is the pipeline's one frame dropper, so the delivery
     * rate is its dst when it drops and the sensor's rate when it does not;
     * writing the *requested* rate into src -- what this function did first
     * -- makes the encoder's frame-rate control drop nothing and the rate
     * controller budget bitrate for a rate the pipeline is not delivering,
     * so a CBR stream overshoots by src/dst. dst is clamped to src because
     * the encoder cannot invent frames the pipeline does not carry.
     */
    unsigned int src_fps = st->mode.frame_rate > 0 ? (unsigned int)st->mode.frame_rate : 25u;
    unsigned int req, dst_fps;
    bool fixqp = enc->rc_mode == RSS_RC_FIXQP;
    bool cbr = enc->rc_mode == RSS_RC_CBR;

    if (enc->bound_fs >= 0 && enc->bound_fs < HISI_FS_CHN_NUM &&
        st->fs[enc->bound_fs].frame_rate.dst_frame_rate > 0)
        src_fps = (unsigned int)st->fs[enc->bound_fs].frame_rate.dst_frame_rate;
    if (!src_fps)
        src_fps = 25u;

    /* Rounded, not truncated: 30000/1001 is 30, not 29. */
    req = enc->fps_num && enc->fps_den ? (enc->fps_num + enc->fps_den / 2u) / enc->fps_den
                                       : src_fps;
    if (!req)
        req = 1u;
    dst_fps = req < src_fps ? req : src_fps;

    memset(rc, 0, sizeof(*rc));

    rc->mode = v4_venc_rc_mode_for(enc->payload, cbr, fixqp);

    if (enc->payload == V4_PT_JPEG || enc->payload == V4_PT_MJPEG) {
        if (fixqp) {
            /* src == dst on purpose: a snapshot channel is paced by rvd's
             * pulse loop (start, one frame, stop), and a frame-rate
             * controller in front of it would make the first frame after a
             * start wait out the drop pattern -- a /snap.jpg latency of up
             * to a second at 1 fps. */
            rc->mjpeg_fixqp.src_frame_rate = src_fps;
            rc->mjpeg_fixqp.dst_frame_rate = src_fps;
            rc->mjpeg_fixqp.qfactor = hisi_enc_qfactor(enc->init_qp);
        } else {
            rc->mjpeg_cbr.stat_time = 1;
            rc->mjpeg_cbr.src_frame_rate = src_fps;
            rc->mjpeg_cbr.dst_frame_rate = dst_fps;
            rc->mjpeg_cbr.bit_rate = hisi_enc_kbps(enc->bitrate);
        }
        return;
    }

    if (fixqp) {
        rc->fixqp.gop = enc->gop;
        rc->fixqp.src_frame_rate = src_fps;
        rc->fixqp.dst_frame_rate = dst_fps;
        rc->fixqp.i_qp = enc->init_qp > 0 ? (unsigned int)enc->init_qp : 28u;
        rc->fixqp.p_qp = rc->fixqp.i_qp + 2u;
        rc->fixqp.b_qp = rc->fixqp.p_qp;
        return;
    }

    rc->cbr.gop = enc->gop;
    /* Seconds the controller averages over. 4 is majestic's value, and it
     * is what raptor's sawtooth and doubled IDRs turned out to be. With 1
     * -- the vendor sample's value -- an unbounded channel at gop 40 /
     * 20 fps sawtoothed 1.64-1.71x and emitted every IDR twice; at 4 the
     * same channel with the same bounds is flat at 1.02x with clean single
     * IDRs 40 frames apart. The plausible mechanism is that a 1 s window is
     * shorter than the 2 s GOP, so the controller never accounts a whole
     * GOP; /proc/umap/rc shows IPRatio at 2-4 with 1 and 39-57 with 4. Not
     * proven -- a gop-20 cell at stat_time 1 would be the test. */
    rc->cbr.stat_time = 4;
    rc->cbr.src_frame_rate = src_fps;
    rc->cbr.dst_frame_rate = dst_fps;
    rc->cbr.bit_rate = hisi_enc_kbps(enc->bitrate);
}

/*
 * hisi_enc_fill_attr -- the whole channel attribute, built fresh.
 *
 * Every field comes from the channel's own tracked state, never from rvd's
 * config, so that hisi_enc_reconfigure can rebuild the struct without a
 * config in hand and without any field quietly reverting to a default.
 *
 * bByFrame is true, which is what makes GetStream return whole frames --
 * with it false the encoder emits slices and every consumer downstream
 * would have to reassemble them.
 */
static void hisi_enc_fill_attr(const hisi_state_t *st, const hisi_venc_chn_t *enc,
                               v4_venc_chn_attr *attr)
{
    memset(attr, 0, sizeof(*attr));

    attr->venc_attr.type = enc->payload;
    attr->venc_attr.max_pic_width = enc->width;
    attr->venc_attr.max_pic_height = enc->height;
    attr->venc_attr.pic_width = enc->width;
    attr->venc_attr.pic_height = enc->height;
    attr->venc_attr.buf_size = enc->buf_size;
    attr->venc_attr.by_frame = 1;

    /*
     * Profile. H.264 takes 0=baseline, 1=main, 2=high and rvd's config uses
     * the same numbering, so it passes through. H.265 takes 0=Main and
     * 1=Main10, where rvd's H.264 numbering would ask for Main10 on a
     * config that meant High -- so it is pinned to Main, which is the only
     * profile this silicon encodes at 8 bits anyway. JPEG has one profile.
     */
    attr->venc_attr.profile = enc->profile;

    /* Share the reconstruction and reference buffers. The vendor samples
     * set it for H.264 and H.265 and it is a straight DDR saving on a part
     * with 128 MB; it has no meaning for JPEG, whose union member is a
     * different struct entirely. */
    if (enc->payload == V4_PT_H264 || enc->payload == V4_PT_H265)
        attr->venc_attr.codec.rcn_ref_share_buf = 1;

    hisi_enc_fill_rc(st, enc, &attr->rc_attr);

    /*
     * NORMALP: every frame after the I is a P referencing the one before.
     * SMARTP exists and encodes a long-term background reference, which is
     * what its u32BgInterval names -- worth having, but it changes what a
     * GOP means and rvd's gop_mode is not wired through on this family yet.
     */
    attr->gop_attr.mode = V4_GOP_MODE_NORMALP;
    attr->gop_attr.normal_p.ip_qp_delta = enc->ip_qp_delta;
}

/* ================================================================
 * GROUPS
 * ================================================================ */

/*
 * HiMPP has no encoder group. A VENC channel is bound directly to a VPSS
 * channel, so there is no object between them for a group to be.
 *
 * The ops are published anyway and do nothing, exactly as the SigmaStar
 * backend does, because rvd calls them unconditionally during pipeline
 * construction and a NOTSUP return would be read as a failure. Answering
 * RSS_OK to "create the group" is truthful here: the group's only job is to
 * exist so a channel can be registered into it, and enc_register_channel is
 * where that actually becomes a bind.
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
 * hisi_enc_start_recv -- HI_MPI_VENC_StartRecvFrame, for good.
 *
 * s32RecvPicNum = -1 means "until stopped". Zero is explicitly rejected by
 * the driver, which is why there is no "start for N frames" path: rvd never
 * asks for one, and the single value that would express it is the one value
 * that fails.
 */
static int hisi_enc_start_recv(hisi_state_t *st, int chn, hisi_venc_chn_t *enc)
{
    v4_venc_recv_pic_param param;
    int ret;

    if (enc->receiving)
        return RSS_OK;
    if (!st->venc.fnStartRecvFrame)
        return RSS_ERR_NOTSUP;

    memset(&param, 0, sizeof(param));
    param.recv_pic_num = -1;

    ret = st->venc.fnStartRecvFrame(chn, &param);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VENC_StartRecvFrame(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    enc->receiving = true;
    return RSS_OK;
}

/*
 * hisi_enc_apply_rc_param -- the QP bounds, written through VENC_RC_PARAM_S.
 *
 * Rate control on gen4 is two structures behind two calls. The channel
 * attribute set at create time carries the *target* -- mode, bitrate, GOP,
 * frame rate. The bounds the controller must respect while chasing that
 * target are here, and a backend that never makes this call gets the
 * driver's defaults for every one of them.
 *
 * That is not a cosmetic gap. Measured on an EV300 at 2592x1944, H.265,
 * 5 Mbps VBR, GOP 100 with the bounds unwritten: I-frames came out at
 * 1,008,235 bytes each against a 625 KB whole-second budget, because the
 * driver's QP floor of 24 let the I-frame spend whatever it wanted and the
 * P-frames after it got what was left. Per-frame acutance through the GOP
 * ran 11.17 at the I-frame down to 7.03 at the worst P-frame -- a 1.95x
 * sawtooth, visible as a picture that sharpens and softens once per GOP.
 * Setting the bounds takes that to 1.00-1.01x.
 *
 * It does not, however, explain it. Majestic reconfigured onto the same
 * wide bounds -- 24..51, and u32MaxIprop loosened to the driver's 20 as
 * well -- stays flat at 1.01-1.03x on the same board and scene. The cause
 * was stat_time: hisi_enc_fill_rc wrote 1 where majestic writes 4, and
 * with 4 the unbounded channel is flat too (1.02x, single IDRs). Writing
 * the bounds is still correct -- a configured bound that goes nowhere is
 * a defect on its own -- but it is a mitigation of that, not a fix for
 * it. See docs/hisilicon.md for the measurements.
 *
 * GET-MODIFY-SET, not build-and-write. Three quarters of this structure is
 * the macroblock-level texture thresholds (three 16-entry Mad tables) plus
 * the row QP delta and the first-frame start QP -- all of them tuned
 * defaults that belong to the driver. Zeroing them to write two fields
 * would silently retune macroblock-level rate control. So the structure is
 * read back, the fields raptor owns are patched, and the rest is handed
 * straight back. This is the same discipline the ISP tuning loader uses,
 * and for the same reason.
 *
 * The union is discriminated by the channel's RC mode, so the shape is
 * chosen through v4_venc_rc_mode_for -- the same function that tagged
 * VENC_RC_ATTR_S. Choosing it any other way is how the two halves come to
 * disagree, and the H.264 and H.265 VBR layouts differ in a way that makes
 * disagreement silent rather than fatal (see v4_venc.h).
 *
 * The I-frame bounds get the same pair as the P/B bounds. A config that
 * says "keep this stream between QP 28 and 42" is not asking for an
 * exemption for one frame in forty, and the I-frame running below the floor
 * is precisely the measured defect. The I/P relationship the caller *did*
 * ask for is ip_qp_delta, which still operates, inside the bounds.
 *
 * Modes without QP bounds -- FIXQP, which is all QP, and the two MJPEG
 * modes, whose quality is a Qfactor -- return OK having done nothing.
 */
static int hisi_enc_apply_rc_param(hisi_state_t *st, int chn, hisi_venc_chn_t *enc)
{
    v4_venc_rc_param param;
    unsigned int *p_max, *p_min, *p_max_i, *p_min_i;
    unsigned int *p_min_iprop, *p_max_iprop;
    unsigned int min_qp, max_qp;
    v4_venc_rc_mode mode;
    int ret;

    if (!enc->created)
        return RSS_OK;
    /* Nothing configured and nothing ever written: leave the driver's
     * defaults alone rather than writing our idea of them back over the
     * vendor's. Nothing configured after something was written is a reset,
     * and falls through to put the driver's own values back. */
    if (enc->min_qp < 0 && enc->max_qp < 0 && !enc->rc_written)
        return RSS_OK;

    mode = v4_venc_rc_mode_for(enc->payload, enc->rc_mode == RSS_RC_CBR,
                               enc->rc_mode == RSS_RC_FIXQP);

    /* Modes with no QP bounds to set are answered before the driver is
     * touched, so a JPEG channel carrying a stray min_qp does not produce a
     * GetRcParam error about a structure that does not apply to it. */
    if (mode != V4_RC_MODE_H264CBR && mode != V4_RC_MODE_H265CBR &&
        mode != V4_RC_MODE_H264VBR && mode != V4_RC_MODE_H265VBR)
        return RSS_OK;

    if (!st->venc.fnGetRcParam || !st->venc.fnSetRcParam) {
        HAL_LOG_WARN("venc chn %d: QP bounds unavailable -- libmpi exports no "
                     "HI_MPI_VENC_%sRcParam",
                     chn, st->venc.fnGetRcParam ? "Set" : "Get");
        return RSS_ERR_NOTSUP;
    }

    ret = st->venc.fnGetRcParam(chn, &param);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VENC_GetRcParam(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    switch (mode) {
    case V4_RC_MODE_H264CBR:
    case V4_RC_MODE_H265CBR:
        p_max = &param.cbr.max_qp;
        p_min = &param.cbr.min_qp;
        p_max_i = &param.cbr.max_iqp;
        p_min_i = &param.cbr.min_iqp;
        p_min_iprop = &param.cbr.min_iprop;
        p_max_iprop = &param.cbr.max_iprop;
        break;
    case V4_RC_MODE_H264VBR:
        p_max = &param.h264_vbr.max_qp;
        p_min = &param.h264_vbr.min_qp;
        p_max_i = &param.h264_vbr.max_iqp;
        p_min_i = &param.h264_vbr.min_iqp;
        p_min_iprop = &param.h264_vbr.min_iprop;
        p_max_iprop = &param.h264_vbr.max_iprop;
        break;
    case V4_RC_MODE_H265VBR:
        p_max = &param.h265_vbr.max_qp;
        p_min = &param.h265_vbr.min_qp;
        p_max_i = &param.h265_vbr.max_iqp;
        p_min_i = &param.h265_vbr.min_iqp;
        p_min_iprop = &param.h265_vbr.min_iprop;
        p_max_iprop = &param.h265_vbr.max_iprop;
        break;
    default:
        /* Unreachable -- filtered above. Present because the switch is over
         * an enum and a silent fallthrough here would use p_* uninitialised. */
        return RSS_OK;
    }

    /*
     * The driver's values as found, captured the first time through for
     * this mode -- before the write below, so they are not raptor's own
     * previous write. On this board a fresh rvd reads the vendor's 24..51
     * back after the SoC-global teardown in hal_init, so "as found" and
     * "the vendor's" have agreed every time it was checked; the wording is
     * cautious because nothing here can tell the two apart. A mode change
     * reshapes the union and may change the values, so the capture is
     * keyed on the mode.
     */
    if (!enc->rc_drv_known || enc->rc_drv_mode != mode) {
        enc->rc_drv_min_qp = *p_min;
        enc->rc_drv_max_qp = *p_max;
        enc->rc_drv_min_iqp = *p_min_i;
        enc->rc_drv_max_iqp = *p_max_i;
        enc->rc_drv_mode = mode;
        enc->rc_drv_known = true;
    }

    /*
     * Whichever bound the caller left unset gets the driver's value back,
     * which is why this reads the captured defaults rather than the
     * structure's current contents (possibly raptor's last write) or a
     * number of raptor's own.
     */
    min_qp = enc->min_qp >= 0 ? (unsigned int)enc->min_qp : enc->rc_drv_min_qp;
    max_qp = enc->max_qp >= 0 ? (unsigned int)enc->max_qp : enc->rc_drv_max_qp;

    if (min_qp > V4_VENC_QP_MAX)
        min_qp = V4_VENC_QP_MAX;
    if (max_qp > V4_VENC_QP_MAX)
        max_qp = V4_VENC_QP_MAX;
    /* The driver rejects the whole call if min > max, naming no field.
     * A caller who inverted them meant a range, so widen to the pair
     * rather than refuse. */
    if (min_qp > max_qp) {
        unsigned int t = min_qp;

        min_qp = max_qp;
        max_qp = t;
    }

    HAL_LOG_INFO("venc chn %d: QP bounds %u..%u (driver had %u..%u, I %u..%u), "
                 "iprop %u..%u, scene-change detect %d insert-idr %d",
                 chn, min_qp, max_qp, *p_min, *p_max, *p_min_i, *p_max_i, *p_min_iprop,
                 *p_max_iprop, param.scene_change.detect_scene_change,
                 param.scene_change.adaptive_insert_idr);

    *p_min = min_qp;
    *p_max = max_qp;
    /* The I-frame pair follows the configured pair, and goes back to the
     * driver's own I-frame pair -- not the P pair -- when both are reset. */
    if (enc->min_qp < 0 && enc->max_qp < 0) {
        *p_min_i = enc->rc_drv_min_iqp;
        *p_max_i = enc->rc_drv_max_iqp;
    } else {
        *p_min_i = min_qp;
        *p_max_i = max_qp;
    }

    ret = st->venc.fnSetRcParam(chn, &param);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VENC_SetRcParam(%d) QP %u..%u failed: 0x%x", chn, min_qp, max_qp, ret);
        return RSS_ERR_IO;
    }
    enc->rc_written = true;

    return RSS_OK;
}

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg)
{
    v4_venc_chn_attr attr;
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
    /* Fresh channel: the driver's bounds get captured again on the first
     * apply, and nothing has been written to them yet. */
    enc->rc_drv_known = false;
    enc->rc_written = false;
    enc->min_qp = cfg->min_qp;
    enc->max_qp = cfg->max_qp;

    /*
     * The encoder's output ring. Undersizing it is the classic gen4 encoder
     * fault: the channel creates, runs, and drops frames whose packets do
     * not fit, with no error anywhere. A byte per pixel is comfortably above
     * what an H.264 or H.265 IDR needs at any sane bitrate and is what the
     * SDK samples use. Aligned to 64 because the driver rejects an unaligned
     * stream buffer, and rounding up beats reporting a size error a caller
     * cannot act on.
     */
    enc->buf_size = cfg->buf_size ? cfg->buf_size : (unsigned int)enc->width * enc->height;
    /* The JPEG encoder refuses a buffer smaller than the picture at
     * 16-aligned dimensions -- measured: 640x360 is rejected with
     * "Buffer [230400] not enough! At least 235520" in /dev/logmpp, and
     * 235520 is exactly 640 x 368. (This, not the payload type, was the
     * 0xa0088003 the whole create returned; VENC_CreateChn reports the
     * jpege module's refusal as ILLEGAL_PARAM and names no field.) */
    if (enc->payload == V4_PT_MJPEG) {
        unsigned int min = (((unsigned int)enc->width + 15u) & ~15u) *
                           (((unsigned int)enc->height + 15u) & ~15u);

        if (enc->buf_size < min)
            enc->buf_size = min;
    }
    enc->buf_size = (enc->buf_size + 63u) & ~63u;

    /*
     * Profile. H.264 takes 0=baseline, 1=main, 2=high and rvd's config uses
     * the same numbering, so it passes through, defaulting to high. H.265
     * takes 0=Main and 1=Main10, where rvd's H.264 numbering would ask for
     * Main10 on a config that meant High -- so it is pinned to Main, the
     * only profile this silicon encodes at 8 bits anyway. JPEG has one.
     */
    if (enc->payload == V4_PT_H264)
        enc->profile = cfg->profile >= 0 && cfg->profile <= 2 ? (unsigned int)cfg->profile : 2u;
    else
        enc->profile = 0;

    hisi_enc_fill_attr(st, enc, &attr);
    ret = st->venc.fnCreateChn(chn, &attr);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VENC_CreateChn(%d) %ux%u codec %d failed: 0x%x", chn, enc->width,
                    enc->height, (int)enc->codec, ret);
        return RSS_ERR_IO;
    }

    enc->created = true;

    /*
     * QP bounds, once the channel exists -- VENC_RC_PARAM_S is per-channel
     * state the driver only has somewhere to keep after CreateChn.
     *
     * A failure here is logged and not fatal. The channel is a working
     * channel without its bounds; it just runs at the driver's, which is
     * what every gen4 channel did before this call existed. Refusing the
     * create would trade a slightly worse stream for no stream.
     */
    if (hisi_enc_apply_rc_param(st, chn, enc) != RSS_OK)
        HAL_LOG_WARN("venc chn %d: continuing at the driver's QP bounds", chn);

    /*
     * Start receiving here, as part of create, for the same structural
     * reason hal_fs_create_channel enables its VPSS channel: HiMPP's order
     * is CreateChn -> StartRecvFrame -> Bind, and raptor's is create ->
     * bind -> start. The vendor's own sequence is explicit about it
     * (mpp/sample/vio/sample_vio.c:190 then :197, with SAMPLE_COMM_VENC_Start
     * issuing StartRecvFrame immediately after CreateChn), and a channel
     * that is not receiving is not a legal bind destination:
     * HI_MPI_SYS_Bind returns 0xa0028009, SYS / ERROR / EN_ERR_NOT_PERM,
     * naming neither end.
     *
     * hal_enc_start stays published and stays meaningful -- it is
     * idempotent for a channel already receiving, and rvd's JPEG loop still
     * stops and restarts channels as consumers come and go.
     */
    /*
     * MJPEG channels start nothing here. rvd manages a snapshot channel's
     * whole duty cycle through enc_start/enc_stop and believes a fresh
     * channel is idle -- a channel receiving from create is one rvd will
     * never stop, and with no consumer draining it the encoder's output
     * fills, it stalls holding its input pictures, and four queued 5 MP
     * frames are VB pool 0 in its entirety (measured -- VI starved within
     * a second). The bind is deferred with the receive; see
     * hisi_bind_vpss_venc.
     */
    if (enc->payload != V4_PT_MJPEG) {
        ret = hisi_enc_start_recv(st, chn, enc);
        if (ret) {
            if (st->venc.fnDestroyChn)
                st->venc.fnDestroyChn(chn);
            enc->created = false;
            return ret;
        }
    }

    /*
     * The channel now exists, which is the first moment an OSD region
     * registered against it can be attached -- and, on a restart, the
     * moment its regions come back. hal_bind flushes too; this is the
     * earlier of the two and the one that makes an encoder destroy/create
     * transparent to the overlay. Idempotent, so both firing costs nothing.
     */
    hisi_osd_flush_pending(st, chn);

    HAL_LOG_INFO("venc chn %d: %ux%u codec %d, %u bps (%u kbps to the driver), gop %u, buf %u",
                 chn, enc->width, enc->height, (int)enc->codec, enc->bitrate,
                 hisi_enc_kbps(enc->bitrate), enc->gop, attr.venc_attr.buf_size);
    return RSS_OK;
}

int hal_enc_destroy_channel(void *ctx, int chn)
{
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_OK;

    /*
     * Overlays first, and this one is not optional: HiMPP refuses to
     * destroy a VENC channel that still carries attached regions, and
     * documents the refusal as exactly that case ("resource is busy, eg.
     * destroy a venc chn without unregistering it", HI_ERR_RGN_BUSY). The
     * registration survives -- only the attach is undone -- so the flush in
     * hal_enc_create_channel puts them back if the channel comes back.
     */
    hisi_osd_detach_chn(st, chn);

    /* Order: release any held stream, stop receiving, unbind, close the fd,
     * then destroy. Each step undoes exactly one thing the create-and-start
     * path did, and destroying a channel that is still bound leaves VPSS
     * holding a reference to a channel that no longer exists. */
    if (enc->frame_held)
        hal_enc_release_frame(ctx, chn, NULL);

    if (enc->receiving)
        hal_enc_stop(ctx, chn);

    if (enc->bound_fs >= 0)
        hisi_unbind_vpss_venc(st, enc->bound_fs, chn);

    if (enc->fd >= 0 && st->venc.fnCloseFd) {
        st->venc.fnCloseFd(chn);
        enc->fd = -1;
    }

    ret = st->venc.fnDestroyChn ? st->venc.fnDestroyChn(chn) : 0;
    if (ret)
        HAL_LOG_WARN("HI_MPI_VENC_DestroyChn(%d) failed: 0x%x", chn, ret);

    memset(enc, 0, sizeof(*enc));
    enc->bound_fs = -1;
    enc->idle_fs = -1;
    enc->fd = -1;
    return RSS_OK;
}

/*
 * hal_enc_register_channel -- bind a VPSS channel to this VENC channel.
 *
 * rvd's "group" is the framesource channel number, which is what makes this
 * op the bind: on a family with no encoder group, registering channel C into
 * group G means "G's pictures go to C".
 */
int hal_enc_register_channel(void *ctx, int grp, int chn)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (grp < 0 || grp >= HISI_FS_CHN_NUM) {
        HAL_LOG_ERR("venc chn %d: group %d is not a VPSS channel [0,%d)", chn, grp,
                    HISI_FS_CHN_NUM);
        return RSS_ERR_INVAL;
    }
    if (!enc->created) {
        HAL_LOG_ERR("venc chn %d: register before create", chn);
        return RSS_ERR_INVAL;
    }
    if (enc->bound_fs == grp)
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

    if (enc->bound_fs < 0)
        return RSS_OK;

    return hisi_unbind_vpss_venc(st, enc->bound_fs, chn);
}

/*
 * hal_enc_start / hal_enc_stop -- receive frames, or stop receiving.
 *
 * s32RecvPicNum = -1 means "until stopped". Zero is explicitly rejected by
 * the driver, which is why there is no "start for N frames" path here: rvd
 * never asks for one, and the one value that would express it is the one
 * value that fails.
 *
 * JPEG channels are started the same way, and that is a deliberate
 * departure from how the vendor references drive them. divinus issues
 * StartRecvFrame(1) per snapshot, because it owns the snapshot loop.
 * raptor does not: rvd's frame loop calls enc_start when a snapshot
 * consumer appears and enc_stop when the last one leaves
 * (rvd/rvd_frame_loop.c ~:170 and ~:137), and paces the frames in between
 * itself. Special-casing JPEG here to "not really start" would make
 * enc_start a lie on exactly the channel whose lifetime rvd is managing
 * most carefully, and the first symptom would be a snapshot request that
 * never produces a picture.
 */
int hal_enc_start(void *ctx, int chn)
{
    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;

    /* Receive first, so the channel is a legal bind destination, then the
     * other half of enc_stop's unbind: remake the deferred edge. Divinus's
     * snapshot lifecycle (bind - receive - unbind), expressed through
     * rvd's start/stop pair. */
    {
        int ret = hisi_enc_start_recv(st, chn, enc);

        if (ret)
            return ret;
    }
    if (enc->bound_fs < 0 && enc->idle_fs >= 0) {
        int fs = enc->idle_fs;
        int ret;

        enc->idle_fs = -1;
        ret = hisi_bind_vpss_venc(st, fs, chn);
        if (ret) {
            enc->idle_fs = fs;
            if (st->venc.fnStopRecvFrame) {
                st->venc.fnStopRecvFrame(chn);
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
    if (!st->venc.fnStopRecvFrame)
        return RSS_ERR_NOTSUP;

    ret = st->venc.fnStopRecvFrame(chn);
    if (ret)
        HAL_LOG_WARN("HI_MPI_VENC_StopRecvFrame(%d) failed: 0x%x", chn, ret);

    enc->receiving = false;

    /*
     * A duty-cycled MJPEG channel must not stay bound while stopped.
     * Measured: the VPSS source keeps queueing pictures at a stopped
     * destination and nothing releases them -- four queued 5 MP frames
     * held the whole of VB pool 0 and starved VI within a second of the
     * pipeline starting. Unbind, remember the edge in idle_fs, and let
     * enc_start remake it; ResetChn then flushes whatever was queued
     * before the unbind. This is divinus's snapshot lifecycle
     * (bind - receive one - unbind) mapped onto rvd's start/stop pair.
     * H.26x channels stay bound across a stop, as they always did.
     */
    if (enc->payload == V4_PT_MJPEG && enc->bound_fs >= 0) {
        enc->idle_fs = enc->bound_fs;
        hisi_unbind_vpss_venc(st, enc->bound_fs, chn);
        if (!enc->frame_held && st->venc.fnResetChn) {
            ret = st->venc.fnResetChn(chn);
            if (ret)
                HAL_LOG_WARN("HI_MPI_VENC_ResetChn(%d) failed: 0x%x", chn, ret);
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
            HAL_LOG_ERR("HI_MPI_VENC_GetFd(%d) failed: %d", chn, fd);
            return RSS_ERR_IO;
        }
        enc->fd = fd;
    }

    return enc->fd;
}

/*
 * hal_enc_poll -- wait for the channel to have a frame.
 *
 * The descriptor from HI_MPI_VENC_GetFd becomes readable when a frame is
 * ready. A zero timeout is a legitimate poll, so it is passed through rather
 * than treated as "block forever".
 *
 * Return values are the ones rvd's encoder thread distinguishes: RSS_OK for
 * ready, -EAGAIN for a timeout, RSS_ERR_IO for a broken descriptor. EINTR
 * is a timeout rather than an error -- a signal arriving mid-wait says
 * nothing about the encoder.
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
 * hisi_enc_nal_type -- classify one pack.
 *
 * The NALU-type union is read as H.264's enumeration or H.265's depending on
 * the channel's codec, and the two disagree on every value that matters: IDR
 * is 5 against 19, SPS is 7 against 33. Reading the wrong one marks
 * parameter sets as slices, which downstream looks like a stream that never
 * sends an SPS.
 */
static rss_nal_type_t hisi_enc_nal_type(rss_codec_t codec, unsigned int data_type)
{
    if (codec == RSS_CODEC_JPEG || codec == RSS_CODEC_MJPEG)
        return RSS_NAL_JPEG_FRAME;

    if (codec == RSS_CODEC_H265) {
        switch (data_type) {
        case V4_H265_NALU_VPS:
            return RSS_NAL_H265_VPS;
        case V4_H265_NALU_SPS:
            return RSS_NAL_H265_SPS;
        case V4_H265_NALU_PPS:
            return RSS_NAL_H265_PPS;
        case V4_H265_NALU_SEI:
            return RSS_NAL_H265_SEI;
        case V4_H265_NALU_IDRSLICE:
            return RSS_NAL_H265_IDR;
        case V4_H265_NALU_ISLICE:
        case V4_H265_NALU_PSLICE:
        case V4_H265_NALU_BSLICE:
            return RSS_NAL_H265_SLICE;
        default:
            return RSS_NAL_UNKNOWN;
        }
    }

    switch (data_type) {
    case V4_H264_NALU_SPS:
        return RSS_NAL_H264_SPS;
    case V4_H264_NALU_PPS:
        return RSS_NAL_H264_PPS;
    case V4_H264_NALU_SEI:
        return RSS_NAL_H264_SEI;
    case V4_H264_NALU_IDRSLICE:
        return RSS_NAL_H264_IDR;
    case V4_H264_NALU_ISLICE:
    case V4_H264_NALU_PSLICE:
    case V4_H264_NALU_BSLICE:
        return RSS_NAL_H264_SLICE;
    default:
        return RSS_NAL_UNKNOWN;
    }
}

/*
 * hisi_enc_fill_nals -- one pack becomes one rss_nal_unit_t.
 *
 * u32Offset is the length of the header *inside* the packet, not an offset
 * into a larger buffer: the payload runs from pu8Addr + u32Offset to
 * pu8Addr + u32Len. A reader that ignores it emits the Annex-B start code
 * twice; one that subtracts it from the wrong base emits a truncated NAL.
 *
 * An offset larger than the length would make the subtraction wrap, so it
 * is treated as "no offset" rather than trusted -- the frame is still worth
 * delivering and a wrapped length is not.
 */
static void hisi_enc_fill_nals(hisi_venc_chn_t *enc, rss_frame_t *frame)
{
    unsigned int count = enc->stream.pack_count;
    unsigned int i;

    if (count > HISI_VENC_MAX_PACKS) {
        HAL_LOG_WARN("venc: %u packs in one frame, reporting %d", count, HISI_VENC_MAX_PACKS);
        count = HISI_VENC_MAX_PACKS;
    }

    frame->is_key = false;

    for (i = 0; i < count; i++) {
        v4_venc_pack *pack = &enc->packs[i];
        rss_nal_unit_t *nal = &enc->nals[i];
        rss_nal_type_t type = hisi_enc_nal_type(enc->codec, pack->data_type);
        unsigned int offset = pack->offset;

        if (offset > pack->len)
            offset = 0;

        nal->data = pack->addr ? pack->addr + offset : NULL;
        nal->length = pack->len - offset;
        nal->type = type;
        nal->frame_end = pack->frame_end != 0;

        if (type == RSS_NAL_H264_IDR || type == RSS_NAL_H265_IDR)
            frame->is_key = true;
    }

    frame->nals = enc->nals;
    frame->nal_count = count;
}

/*
 * hal_enc_get_frame -- check out one encoded frame.
 *
 * QueryStatus first, because GetStream copies into a caller-allocated pack
 * array and needs to be told how big it is. u32CurPacks == 0 is normal: the
 * descriptor signals readiness slightly ahead of the pack being complete.
 * Report it as -EAGAIN, the same "no frame this time" the other backends
 * return and the only value rvd's encoder thread treats as non-fatal.
 *
 * A frame with more packs than the per-channel array holds is drained into
 * a temporary array and dropped. Clamping does not work: with bByFrame set,
 * GetStream refuses a pack array smaller than the frame (VENC / NOMEM)
 * rather than partially filling it, so a clamp leaves the frame queued, the
 * descriptor ready, and this op failing identically forever -- a dead
 * stream with a busy poll loop in front of it. Draining loses one frame and
 * keeps the channel.
 */
int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame)
{
    v4_venc_chn_status status;
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
        HAL_LOG_ERR("HI_MPI_VENC_QueryStatus(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }
    if (!status.cur_packs)
        return -EAGAIN;

    packs = status.cur_packs;
    if (packs > HISI_VENC_MAX_PACKS) {
        v4_venc_pack *tmp = (v4_venc_pack *)calloc(packs, sizeof(*tmp));
        v4_venc_stream drain;

        HAL_LOG_ERR("venc chn %d: frame carries %u packs, array holds %d; dropping the frame",
                    chn, packs, HISI_VENC_MAX_PACKS);
        if (!tmp)
            return RSS_ERR_NOMEM;
        memset(&drain, 0, sizeof(drain));
        drain.pack = tmp;
        drain.pack_count = packs;
        ret = st->venc.fnGetStream(chn, &drain, 0);
        if (!ret)
            st->venc.fnReleaseStream(chn, &drain);
        free(tmp);
        return ret ? RSS_ERR_IO : -EAGAIN;
    }

    memset(&enc->stream, 0, sizeof(enc->stream));
    memset(enc->packs, 0, sizeof(enc->packs));
    enc->stream.pack = enc->packs;
    enc->stream.pack_count = packs;

    /* Zero timeout: the descriptor already said a frame is ready, and this
     * call moves descriptors rather than pixels. */
    /* First encoded frame anywhere = the ISP is demonstrably running; the
     * descriptor said so, and that is enough. One atomic test per frame
     * after that. Before GetStream on purpose: the first time through this
     * runs the whole IQ load -- file parse plus a Get/Set round trip per
     * module -- and doing that with a stream buffer checked out held the
     * first frame for the duration. See hal_isp.c. */
    hisi_isp_note_frame(st);

    ret = st->venc.fnGetStream(chn, &enc->stream, 0);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VENC_GetStream(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    enc->frame_held = true;

    memset(frame, 0, sizeof(*frame));
    frame->codec = enc->codec;
    frame->seq = enc->stream.seq;
    /* HiMPP timestamps packs in microseconds, the unit rss_frame_t wants;
     * the first pack carries the frame's capture time. */
    frame->timestamp = enc->stream.pack_count ? (int64_t)enc->packs[0].pts : 0;
    hisi_enc_fill_nals(enc, frame);
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
        HAL_LOG_WARN("HI_MPI_VENC_ReleaseStream(%d) failed: 0x%x", chn, ret);

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
    v4_venc_chn_status status;
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
 * hisi_enc_reconfigure -- read-modify-write the channel attribute.
 *
 * The single path every rate knob funnels through, because HiMPP has no
 * per-knob setter: bitrate, GOP and frame rate all live inside
 * VENC_CHN_ATTR_S and the only way to change one is to write the whole
 * struct back.
 *
 * Built fresh from the channel's tracked state rather than read back and
 * patched. GetChnAttr would work, but the tracked state is the record of
 * what raptor asked for, and rebuilding from it means two knobs set in
 * sequence both survive -- where a read-modify-write of the driver's copy
 * would silently adopt whatever the driver normalised.
 */
static int hisi_enc_reconfigure(hisi_state_t *st, int chn, hisi_venc_chn_t *enc)
{
    v4_venc_chn_attr attr;
    int ret;

    if (!enc->created)
        return RSS_OK;
    if (!st->venc.fnSetChnAttr)
        return RSS_ERR_NOTSUP;

    hisi_enc_fill_attr(st, enc, &attr);
    ret = st->venc.fnSetChnAttr(chn, &attr);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VENC_SetChnAttr(%d) failed: 0x%x", chn, ret);
        return RSS_ERR_IO;
    }

    /*
     * The bounds are re-applied after every attribute write. Whether
     * SetChnAttr resets VENC_RC_PARAM_S is undocumented and differs by mode
     * -- a mode change certainly re-tags the union, which makes the fields
     * this backend owns land somewhere else. Rewriting them is one MPI call
     * on a path that already does one, and it is the same "rebuild from
     * tracked state" rule the attribute itself follows.
     */
    (void)hisi_enc_apply_rc_param(st, chn, enc);

    return RSS_OK;
}

/*
 * Every setter below restores the tracked value when the driver refuses the
 * write. Tracked state the driver rejected is not merely a wrong answer
 * from enc_get_channel_attr: hisi_enc_reconfigure rebuilds the whole
 * attribute from tracked state, so a phantom value would be silently
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

/*
 * hal_enc_set_qp_bounds -- the one setter that is not a channel-attribute
 * read-modify-write, because its fields are not in the channel attribute.
 *
 * -1 in either argument puts the driver's own bound back on that side,
 * matching rss_video_config_t's convention; -1 in both restores the
 * driver's P and I pairs as they were before raptor first wrote them.
 * Anything else outside [0..51] is refused, where create_channel clamps a
 * config value: a config is a wish and gets the nearest legal thing, a
 * runtime call is an instruction and gets told no. The range check runs
 * on the int, before the narrowing store. "The driver's own" means as found at the
 * first apply; see hisi_enc_apply_rc_param.
 */
int hal_enc_set_qp_bounds(void *ctx, int chn, int min_qp, int max_qp)
{
    int16_t old_min, old_max;
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;
    if (min_qp < -1 || min_qp > (int)V4_VENC_QP_MAX || max_qp < -1 ||
        max_qp > (int)V4_VENC_QP_MAX)
        return RSS_ERR_INVAL;

    old_min = enc->min_qp;
    old_max = enc->max_qp;
    enc->min_qp = (int16_t)min_qp;
    enc->max_qp = (int16_t)max_qp;

    ret = hisi_enc_apply_rc_param(st, chn, enc);
    if (ret) {
        enc->min_qp = old_min;
        enc->max_qp = old_max;
    }
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
 * enc_set/get_jpeg_qp -- rvd passes JPEG *quality*, 1..100 higher-better
 * (rvd_ctrl.c's set-jpeg-quality hands the config value straight through),
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
    if (enc->payload != V4_PT_MJPEG)
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
    if (!enc->created || enc->payload != V4_PT_MJPEG)
        return RSS_ERR_NOTSUP;

    *qp = (int)hisi_enc_qfactor(enc->init_qp);
    return RSS_OK;
}

/*
 * hisi_enc_refresh_rc -- re-derive the rc attribute after the bind exists.
 *
 * hisi_enc_fill_rc computes the source frame rate from the bound VPSS
 * channel, and a channel is created before it is bound -- so the attribute
 * written at create names the sensor's rate. The frame-rate controller
 * derives its drop pattern from src:dst, and a src above the true delivery
 * rate makes it drop frames it should keep. Called from
 * hisi_bind_vpss_venc once bound_fs is recorded.
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

int hal_enc_request_idr(void *ctx, int chn)
{
    int ret;

    HISI_ENC_ENTER(ctx, chn, st, enc);

    if (!enc->created)
        return RSS_ERR_INVAL;
    if (!st->venc.fnRequestIDR)
        return RSS_ERR_NOTSUP;

    /* bInstant: insert the IDR at the next frame rather than at the next
     * GOP boundary, which is what every caller of this op means. */
    ret = st->venc.fnRequestIDR(chn, 1);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VENC_RequestIDR(%d) failed: 0x%x", chn, ret);
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
    cfg->min_qp = enc->min_qp;
    cfg->max_qp = enc->max_qp;

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
 * VENC_STREAM_INFO_S carries per-frame byte counts, but averaging them here
 * would need a window and a clock this layer does not own, and rvd already
 * measures throughput from the frames it receives. Answering with the
 * configured number is what the other backends do and what the caller uses
 * it for: reporting the channel's setting.
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
 * Called from hisi_teardown, before the framesource release: an encoder
 * still bound to a VPSS channel that is about to be disabled is the state
 * that leaves the kernel side holding buffers.
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
        if (enc->receiving && st->venc.fnStopRecvFrame) {
            st->venc.fnStopRecvFrame(i);
            enc->receiving = false;
        }
        if (enc->bound_fs >= 0)
            hisi_unbind_vpss_venc(st, enc->bound_fs, i);
        if (enc->fd >= 0 && st->venc.fnCloseFd) {
            st->venc.fnCloseFd(i);
            enc->fd = -1;
        }
        if (st->venc.fnDestroyChn) {
            int ret = st->venc.fnDestroyChn(i);
            if (ret)
                HAL_LOG_WARN("HI_MPI_VENC_DestroyChn(%d) failed: 0x%x", i, ret);
        }

        memset(enc, 0, sizeof(*enc));
        enc->bound_fs = -1;
        enc->fd = -1;
    }
}
