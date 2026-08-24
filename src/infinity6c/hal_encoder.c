/*
 * infinity6c/hal_encoder.c -- MI_VENC for the MI 3.0 backend
 *
 * The last stage: an SCL output port bound to an encoder channel, and the frames
 * that come back off it. Counterpart to star/hal_encoder.c, and the file where
 * the generations' difference is most likely to bite silently -- every entry
 * point here leads with a device MI 2.x does not have, and dlsym resolves by
 * name, so the wrong table binds cleanly and then shifts every argument by one.
 *
 * That device is not a topology index to be set to zero and forgotten. It selects
 * the codec engine: H.26x and MJPEG live on different devices, so which one a
 * channel is created on follows from the codec it encodes, and the channel index
 * is shared between them rather than per-device.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "infinity6c_state.h"

#include <sys/select.h>

/* ================================================================
 * CODEC AND RATE MAPPING
 * ================================================================ */

/*
 * i6c_enc_device -- which engine encodes this codec.
 *
 * The two H.26x codecs share one device and the two JPEG variants share the
 * other. A channel index is not qualified by the device, so channel 0 on the
 * MJPEG device and channel 0 on the H.26x device are the same channel -- which
 * is why the device is remembered per channel rather than recomputed.
 */
static int i6c_enc_device(rss_codec_t codec, unsigned int *device)
{
    switch (codec) {
    case RSS_CODEC_H264:
    case RSS_CODEC_H265:
        *device = I6C_VENC_DEV_H26X_0;
        return RSS_OK;
    case RSS_CODEC_JPEG:
    case RSS_CODEC_MJPEG:
        *device = I6C_VENC_DEV_MJPG_0;
        return RSS_OK;
    default:
        return RSS_ERR_NOTSUP;
    }
}

static int i6c_enc_codec(rss_codec_t codec, i6c_venc_codec *out)
{
    switch (codec) {
    case RSS_CODEC_H264:
        *out = I6C_VENC_CODEC_H264;
        return RSS_OK;
    case RSS_CODEC_H265:
        *out = I6C_VENC_CODEC_H265;
        return RSS_OK;
    case RSS_CODEC_JPEG:
    case RSS_CODEC_MJPEG:
        *out = I6C_VENC_CODEC_MJPG;
        return RSS_OK;
    default:
        return RSS_ERR_NOTSUP;
    }
}

/*
 * i6c_enc_fill_attrib -- the codec half of a channel attribute.
 *
 * max* and the plain dimensions are set to the same thing: the max pair sizes
 * the encoder's internal buffers and the plain pair is the picture, and a
 * channel that will not be resized wants them equal. bufSize is the vendor's own
 * rule of thumb, one byte per pixel, which is generous for H.26x and about right
 * for a low-quality JPEG.
 */
static void i6c_enc_fill_attrib(i6c_venc_attrib *attrib, const rss_video_config_t *cfg,
                                i6c_venc_codec codec)
{
    unsigned int size = cfg->buf_size ? cfg->buf_size : (unsigned int)cfg->width * cfg->height;

    memset(attrib, 0, sizeof(*attrib));
    attrib->codec = codec;

    if (codec == I6C_VENC_CODEC_MJPG) {
        attrib->mjpg.maxWidth = cfg->width;
        attrib->mjpg.maxHeight = cfg->height;
        attrib->mjpg.bufSize = size;
        attrib->mjpg.byFrame = 1;
        attrib->mjpg.width = cfg->width;
        attrib->mjpg.height = cfg->height;
        attrib->mjpg.dcfThumbs = 0;
        attrib->mjpg.markPerRow = 0;
        return;
    }

    attrib->h264.maxWidth = cfg->width;
    attrib->h264.maxHeight = cfg->height;
    attrib->h264.bufSize = size;
    attrib->h264.profile = (unsigned int)(cfg->profile < 0 ? 0 : cfg->profile);
    attrib->h264.byFrame = 1;
    attrib->h264.width = cfg->width;
    attrib->h264.height = cfg->height;
    /* No B frames: they buy compression at the cost of the latency this is for. */
    attrib->h264.bFrameNum = 0;
    attrib->h264.refNum = 1;
}

/*
 * i6c_enc_fill_rate -- the rate-control half.
 *
 * Cleared first, and that is load-bearing rather than tidy. The driver compares
 * this half against its own copy with a plain memcmp when an attribute is
 * written back, so a stale byte in a union arm that is not in use reads as a
 * change and provokes an encoder reconfiguration that was not asked for.
 *
 * The mode enum is per codec rather than shared, so an H.265 channel asking for
 * CBR is a different value from an H.264 one -- which is why this switches on
 * both.
 *
 * Returns true when the mode asked for was not one this part can encode in this
 * codec and a working one went in its place; see the H.265 remap below.
 */
static bool i6c_enc_fill_rate(i6c_venc_rate *rate, const rss_video_config_t *cfg,
                              i6c_venc_codec codec)
{
    unsigned int fps_num = cfg->fps_num ? cfg->fps_num : 25;
    unsigned int fps_den = cfg->fps_den ? cfg->fps_den : 1;
    unsigned int gop = cfg->gop_length ? cfg->gop_length : fps_num / fps_den * 2;
    unsigned int bitrate = cfg->bitrate;
    rss_rc_mode_t mode = cfg->rc_mode;
    bool substituted = false;

    memset(rate, 0, sizeof(*rate));

    if (!gop)
        gop = 50;
    if (!bitrate)
        bitrate = 2000000;

    if (codec == I6C_VENC_CODEC_MJPG) {
        /*
         * JPEG has no inter-frame prediction, so only two of raptor's modes mean
         * anything here: a bitrate target or a fixed quality.
         */
        if (cfg->rc_mode == RSS_RC_FIXQP) {
            rate->mode = I6C_VENC_RATEMODE_UBR_MJPGQP;
            rate->mjpgQp.fpsNum = fps_num;
            rate->mjpgQp.fpsDen = fps_den;
            rate->mjpgQp.quality = cfg->init_qp > 0 ? (unsigned int)cfg->init_qp : 8;
        } else {
            rate->mode = I6C_VENC_RATEMODE_UBR_MJPGCBR;
            rate->mjpgCbr.bitrate = bitrate;
            rate->mjpgCbr.fpsNum = fps_num;
            rate->mjpgCbr.fpsDen = fps_den;
        }
        return false;
    }

    /*
     * The rate-mode enum this SoC's driver speaks is the UBR one, not the one
     * the SDK document prints, and that is why every mode here is spelled
     * I6C_VENC_RATEMODE_UBR_*.
     *
     * MI ships two numberings. The published enum runs H.264 1..5, MJPEG 6..8,
     * H.265 9..12. The UBR enum inserts a UBR mode into each codec's block, so
     * it runs H.264 1..6, MJPEG 7..9, H.265 10..14. Nothing in the header says
     * which a given chip wants; the driver does. _MI_VENC_IMPL_CheckRcMode,
     * inlined into MI_VENC_IMPL_CreateChn at 0xde984 of the mi.ko this image
     * ships, bounds eType 2 at mode-1 <= 5, eType 4 at mode-7 <= 2 and eType 3
     * at mode-10 <= 4. Six, three and five: the UBR shape exactly.
     *
     * Sending the published numbering into that driver is an off-by-one that
     * changes meaning rather than failing cleanly. It is what produced both of
     * the symptoms this file used to remap around -- H.265 CBR (9) fell below
     * eType 3's floor and MI_VENC_CreateChn returned ILLEGAL_PARAM with
     * "eType:3 unsupport rc mode:9", while H.265 AVBR (12) was read as
     * H265UBR, whose union is not the one filled in here, and
     * MI_VENC_StartRecvPic dereferenced NULL inside the driver and took the
     * VENC device's rwsem down with it.
     *
     * Confirmed on an SSC377QE before the change: with [stream0] codec h265 and
     * rc_mode cbr, the old remap sent H265VBR (10) and
     * /proc/mi_modules/mi_venc/mi_venc0 reported RateCtl CBR. The driver was
     * reading one lower than raptor was writing, on a mode raptor believed it
     * had substituted.
     *
     * So the remap is gone: all six of raptor's modes reach this part, on both
     * codecs, at their right numbers. CBR and VBR keep the values 1 and 2 in
     * both numberings, which is why the default path never showed this.
     */
    switch (mode) {
    case RSS_RC_FIXQP:
        rate->mode = codec == I6C_VENC_CODEC_H265 ? I6C_VENC_RATEMODE_UBR_H265QP
                                                  : I6C_VENC_RATEMODE_UBR_H264QP;
        rate->h264Qp.gop = gop;
        rate->h264Qp.fpsNum = fps_num;
        rate->h264Qp.fpsDen = fps_den;
        rate->h264Qp.interQual = cfg->init_qp > 0 ? (unsigned int)cfg->init_qp : 30;
        rate->h264Qp.predQual = cfg->init_qp > 0 ? (unsigned int)cfg->init_qp : 30;
        break;

    case RSS_RC_VBR:
        rate->mode = codec == I6C_VENC_CODEC_H265 ? I6C_VENC_RATEMODE_UBR_H265VBR
                                                  : I6C_VENC_RATEMODE_UBR_H264VBR;
        rate->h264Vbr.gop = gop;
        rate->h264Vbr.statTime = 1;
        rate->h264Vbr.fpsNum = fps_num;
        rate->h264Vbr.fpsDen = fps_den;
        rate->h264Vbr.maxBitrate = cfg->max_bitrate ? cfg->max_bitrate : bitrate;
        /*
         * MI's quality numbers run the same way as QP: smaller is better. So
         * raptor's min_qp is the *best* quality bound and max_qp the worst, and
         * swapping them here would invert the rate controller.
         */
        rate->h264Vbr.maxQual = cfg->min_qp >= 0 ? (unsigned int)cfg->min_qp : 20;
        rate->h264Vbr.minQual = cfg->max_qp >= 0 ? (unsigned int)cfg->max_qp : 48;
        break;

    /*
     * The capped and smart modes map onto AVBR, which is what MI offers that is
     * bounded above but free to spend less. Nothing here distinguishes them.
     */
    case RSS_RC_SMART:
    case RSS_RC_CAPPED_VBR:
    case RSS_RC_CAPPED_QUALITY:
        rate->mode = codec == I6C_VENC_CODEC_H265 ? I6C_VENC_RATEMODE_UBR_H265AVBR
                                                  : I6C_VENC_RATEMODE_UBR_H264AVBR;
        rate->h264Avbr.gop = gop;
        rate->h264Avbr.statTime = 1;
        rate->h264Avbr.fpsNum = fps_num;
        rate->h264Avbr.fpsDen = fps_den;
        rate->h264Avbr.maxBitrate = cfg->max_bitrate ? cfg->max_bitrate : bitrate;
        rate->h264Avbr.maxQual = cfg->min_qp >= 0 ? (unsigned int)cfg->min_qp : 20;
        rate->h264Avbr.minQual = cfg->max_qp >= 0 ? (unsigned int)cfg->max_qp : 48;
        break;

    case RSS_RC_CBR:
    default:
        rate->mode = codec == I6C_VENC_CODEC_H265 ? I6C_VENC_RATEMODE_UBR_H265CBR
                                                  : I6C_VENC_RATEMODE_UBR_H264CBR;
        rate->h264Cbr.gop = gop;
        rate->h264Cbr.statTime = 1;
        rate->h264Cbr.fpsNum = fps_num;
        rate->h264Cbr.fpsDen = fps_den;
        rate->h264Cbr.bitrate = bitrate;
        rate->h264Cbr.avgLvl = 0;
        break;
    }

    return substituted;
}

static const char *i6c_rc_mode_name(rss_rc_mode_t mode)
{
    switch (mode) {
    case RSS_RC_FIXQP:
        return "fixqp";
    case RSS_RC_CBR:
        return "cbr";
    case RSS_RC_VBR:
        return "vbr";
    case RSS_RC_SMART:
        return "smart";
    case RSS_RC_CAPPED_VBR:
        return "capped_vbr";
    case RSS_RC_CAPPED_QUALITY:
        return "capped_quality";
    default:
        return "an unknown mode";
    }
}

/*
 * i6c_enc_note_rc_substitution -- say what went in instead, once per decision.
 *
 * Reached from both the create and the reapply path, since the rate half is
 * rebuilt whole for every bitrate or GOP change and the substitution is made
 * again each time. Repeating it per write would bury the notice under an
 * adjustment loop; saying it never is Majestic's mistake.
 */
static void i6c_enc_note_rc_substitution(infinity6c_venc_chn_t *enc, int chn, rss_rc_mode_t asked,
                                         const i6c_venc_rate *rate)
{
    if (enc->rc_substituted)
        return;

    enc->rc_substituted = true;
    HAL_LOG_WARN("venc chn %d: H.265 does not encode at rc_mode = %s on this part -- the driver "
                 "refuses CBR and faults on AVBR -- so the channel runs VBR capped at %u bps "
                 "instead; set rc_mode = vbr or fixqp to choose for yourself",
                 chn, i6c_rc_mode_name(asked), rate->h264Vbr.maxBitrate);
}

/* ================================================================
 * NAL CLASSIFICATION
 * ================================================================ */

static rss_nal_type_t i6c_enc_nal_type(rss_codec_t codec, i6c_venc_nalu nalu)
{
    if (codec == RSS_CODEC_H264) {
        switch (nalu.h264Nalu) {
        case I6C_VENC_NALU_H264_PSLICE:
            return RSS_NAL_H264_SLICE;
        case I6C_VENC_NALU_H264_ISLICE:
        case I6C_VENC_NALU_H264_IPSLICE:
            return RSS_NAL_H264_IDR;
        case I6C_VENC_NALU_H264_SEI:
            return RSS_NAL_H264_SEI;
        case I6C_VENC_NALU_H264_SPS:
            return RSS_NAL_H264_SPS;
        case I6C_VENC_NALU_H264_PPS:
            return RSS_NAL_H264_PPS;
        default:
            return RSS_NAL_UNKNOWN;
        }
    }

    if (codec == RSS_CODEC_H265) {
        switch (nalu.h265Nalu) {
        case I6C_VENC_NALU_H265_PSLICE:
            return RSS_NAL_H265_SLICE;
        case I6C_VENC_NALU_H265_ISLICE:
            return RSS_NAL_H265_IDR;
        case I6C_VENC_NALU_H265_VPS:
            return RSS_NAL_H265_VPS;
        case I6C_VENC_NALU_H265_SPS:
            return RSS_NAL_H265_SPS;
        case I6C_VENC_NALU_H265_PPS:
            return RSS_NAL_H265_PPS;
        case I6C_VENC_NALU_H265_SEI:
            return RSS_NAL_H265_SEI;
        default:
            return RSS_NAL_UNKNOWN;
        }
    }

    return RSS_NAL_JPEG_FRAME;
}

static bool i6c_enc_nal_is_key(rss_nal_type_t type)
{
    return type == RSS_NAL_H264_IDR || type == RSS_NAL_H265_IDR || type == RSS_NAL_JPEG_FRAME;
}

/*
 * i6c_enc_packs -- a zeroed pack array of at least `count` entries.
 *
 * The common case fits the inline array and allocates nothing. A frame
 * fragmented past that grows a heap array once and keeps it, since the next
 * frame is likely to be fragmented too.
 */
static i6c_venc_pack *i6c_enc_packs(infinity6c_venc_chn_t *enc, unsigned int count)
{
    if (count <= I6C_VENC_MAX_PACKS) {
        memset(enc->packs, 0, sizeof(enc->packs));
        enc->pack_cap = I6C_VENC_MAX_PACKS;
        return enc->packs;
    }

    if (count > enc->heap_count) {
        i6c_venc_pack *grown = (i6c_venc_pack *)realloc(enc->heap_packs, count * sizeof(*grown));

        if (!grown) {
            HAL_LOG_ERR("i6c_venc: cannot size a %u-pack array", count);
            return NULL;
        }
        enc->heap_packs = grown;
        enc->heap_count = count;
    }

    memset(enc->heap_packs, 0, enc->heap_count * sizeof(*enc->heap_packs));
    enc->pack_cap = enc->heap_count;

    return enc->heap_packs;
}

/*
 * i6c_enc_fill_nals -- one MI pack becomes one rss_nal_unit_t.
 *
 * The pack array grows as needed but nals[] does not, so an unusually
 * fragmented frame reports its first I6C_VENC_MAX_PACKS packs. That matches
 * rvd's own ceiling, so nothing downstream loses what it would have kept.
 */
static void i6c_enc_fill_nals(infinity6c_venc_chn_t *enc, rss_frame_t *frame)
{
    unsigned int count = enc->strm.count;
    unsigned int i;

    /*
     * Bounded by the array before it is bounded by nals[]. The count arrives in
     * the same struct the array pointer went out in, so a library answering with
     * more packs than it was given room for would be read past the end of it --
     * cheaper to refuse than to trust.
     */
    if (count > enc->pack_cap) {
        HAL_LOG_ERR("i6c_venc: MI_VENC_GetStream returned %u packs for room for %u", count,
                    enc->pack_cap);
        count = enc->pack_cap;
    }

    if (count > I6C_VENC_MAX_PACKS) {
        HAL_LOG_WARN("i6c_venc: %u packs in one frame, reporting %d", count, I6C_VENC_MAX_PACKS);
        count = I6C_VENC_MAX_PACKS;
    }

    frame->is_key = false;

    for (i = 0; i < count; i++) {
        i6c_venc_pack *pack = &enc->strm.packet[i];
        rss_nal_unit_t *nal = &enc->nals[i];
        rss_nal_type_t type = i6c_enc_nal_type(enc->codec, pack->naluType);
        unsigned int offset = pack->offset;
        unsigned int k;

        /*
         * An offset past the length would make the subtraction wrap into a huge
         * length. Treat it as no offset and keep the frame.
         */
        if (offset > pack->length) {
            HAL_LOG_WARN("i6c_venc: pack offset %u exceeds length %u, ignoring offset", offset,
                         pack->length);
            offset = 0;
        }

        nal->data = (const uint8_t *)pack->data + offset;
        nal->length = pack->length - offset;
        nal->frame_end = pack->endFrame ? true : false;
        nal->type = type;

        /*
         * Refine from the contained units. A pack that begins with SPS still is
         * the keyframe, and reporting it as SPS would have rvd take a parameter
         * set for the primary type of every IDR.
         */
        for (k = 0; k < pack->packNum && k < I6C_ARRAY_LEN(pack->packetInfo); k++) {
            rss_nal_type_t sub = i6c_enc_nal_type(enc->codec, pack->packetInfo[k].packType);

            if (i6c_enc_nal_is_key(sub))
                frame->is_key = true;
            if (i6c_enc_nal_is_key(sub) || sub == RSS_NAL_H264_SLICE || sub == RSS_NAL_H265_SLICE)
                nal->type = sub;
        }

        if (i6c_enc_nal_is_key(type))
            frame->is_key = true;
    }

    frame->nals = enc->nals;
    frame->nal_count = count;
}

/* Which of the two ring pools a codec engine owns. */
static int i6c_enc_pool_slot(unsigned int device)
{
    return device == I6C_VENC_DEV_MJPG_0 ? 1 : 0;
}

/* ================================================================
 * BINDING
 * ================================================================ */

/*
 * i6c_enc_detach_cascade_subs -- unhook every sub that hangs off this channel.
 *
 * A sub is fed by the main's VENC ring, so the main cannot be destroyed while a
 * sub is still bound to it. Left bound, the sub points at a channel that no
 * longer exists: it is created, started and receiving nothing, and no later bind
 * repairs it, because i6c_bind_scl_to_venc returns early on a channel that is
 * already bound. The stale bind is therefore permanent, and only a full pipeline
 * teardown clears it.
 *
 * That is what a live set-resolution or set-codec on the main stream used to do
 * to the sub stream: rvd destroys and recreates the one channel it was asked
 * about, the sub was never mentioned, and ten seconds later rvd's frame loop
 * reported the sub polling ETIMEDOUT until the daemon was restarted.
 *
 * The sub is stopped and unbound rather than destroyed. It is rvd's stream and
 * rvd did not ask for it to go away, so its channel keeps its geometry and its
 * configuration; cascade_pending records that it is now a stream without a
 * source, and i6c_enc_recascade_subs hands it the next main.
 *
 * Stop receiving before unbinding, which is the order i6c_teardown_all's own
 * pre-pass uses and for the same reason: an encoder left receiving across the
 * unbind strands a task that the kernel flush then waits on forever.
 */
static void i6c_enc_detach_cascade_subs(infinity6c_state_t *st, int main_chn)
{
    int i;

    for (i = 0; i < I6C_MAX_CHN; i++) {
        infinity6c_venc_chn_t *sub = &st->enc[i];

        if (!sub->created || !sub->cascade || sub->cascade_src != main_chn)
            continue;

        if (sub->receiving) {
            st->venc.stop_recv(sub->device, (unsigned int)i);
            sub->receiving = false;
        }
        i6c_unbind_scl_from_venc(st, i);
        sub->cascade_pending = true;

        HAL_LOG_INFO("infinity6c: venc chn %d unhooked from main chn %d, which is going away", i,
                     main_chn);
    }
}

/*
 * i6c_enc_recascade_subs -- give a waiting sub the new main.
 *
 * Called from the ring bind, once the new main is bound and receiving -- the
 * state a cascade source has to be in before anything hangs off it. Only the
 * ring holder can be one, so there is no other place this belongs.
 *
 * The bind is replayed with dst_fps zero, the pipeline rate, because that is what
 * hal_bind passes and a cascade sub only ever arrives through hal_bind: JPEG
 * never rings and so never cascades, and a sub's own frame rate is the main's by
 * construction.
 *
 * A failure is logged and does not fail the caller. The caller is a main stream
 * that has just come up correctly, and losing the sub is not a reason to lose it
 * too -- the sub is no worse off than it was before this function existed. It
 * also stays pending, so the next main to come up tries again rather than the
 * sub being written off on one refusal.
 */
static void i6c_enc_recascade_subs(infinity6c_state_t *st, int main_chn)
{
    int i;

    for (i = 0; i < I6C_MAX_CHN; i++) {
        infinity6c_venc_chn_t *sub = &st->enc[i];

        if (!sub->created || !sub->cascade_pending)
            continue;

        if (i6c_bind_scl_to_venc(st, i, i, 0) != RSS_OK) {
            HAL_LOG_ERR("venc chn %d: no cascade off the new main chn %d -- that stream stays "
                        "dark until it is restarted",
                        i, main_chn);
            continue;
        }
        sub->cascade_pending = false;

        HAL_LOG_INFO("infinity6c: venc chn %d re-cascaded off new main chn %d", i, main_chn);
    }
}

/*
 * i6c_bind_scl_to_venc -- feed one encoder channel from an SCL port.
 *
 * The port is an argument rather than the channel index reused. They are equal
 * for a video stream and are not for a JPEG one: rvd pairs a JPEG encoder with
 * another stream's framesource, so its channel number counts past the video
 * streams while its frames come from one of their ports. Deriving the port from
 * the channel there binds a port nothing configured.
 *
 * The link type is the other half of the argument list that matters, and it is
 * the channel's own uses_ring that decides it, not the codec. The one H.26x
 * channel that holds the engine's ring is bound through it so the encoder can
 * start on a partial frame, which is where this generation's low latency comes
 * from. A second H.26x channel and every JPEG channel are bound frame-by-frame:
 * the SCL channel drives one ring consumer at a time, so a second ring bind is
 * refused outright, and JPEG has no ring mode to begin with.
 *
 * dst_fps is what the destination is asked to consume, and zero means "the same
 * rate the pipeline runs at". A JPEG channel passes its own much lower rate so
 * MI drops the difference in hardware, which is the only pacing this backend
 * has -- the caps block sets no jpeg_pulse, so rvd's duty-cycling is off here.
 */
int i6c_bind_scl_to_venc(infinity6c_state_t *st, int port, int chn, unsigned int dst_fps)
{
    infinity6c_venc_chn_t *enc = &st->enc[chn];
    i6c_sys_bind src;
    i6c_sys_bind dst;
    i6c_sys_link link;
    int ret;

    if (port < 0 || port >= I6C_MAX_CHN)
        return RSS_ERR_INVAL;
    if (enc->bound)
        return RSS_OK;

    /*
     * A second H.26x stream is not a second SCL port. Only one SCL port can ring
     * an H.26x channel -- a second SCL port bound to a second channel is refused
     * (a frame-based bind to an H.26x channel is NOT_SUPPORT) -- so the engine
     * cascades further H.26x channels off the one that holds the ring: the VENC
     * hardware reduces the main's frame to the sub channel's smaller size. A sub
     * binds VENC(main) -> VENC(this) as a ring, takes no SCL port, and sets no
     * input-source config of its own (the main's ring feeds the whole cascade).
     * JPEG never rings and is handled by the SCL path below.
     */
    if (enc->device != I6C_VENC_DEV_MJPG_0 && !enc->uses_ring) {
        int main_chn = st->enc_ring_chn[i6c_enc_pool_slot(enc->device)];

        if (main_chn < 0) {
            HAL_LOG_ERR("venc chn %d: no main H.26x channel holds the ring to cascade from", chn);
            return RSS_ERR_INVAL;
        }

        /*
         * The cascade reduces the main's frame to this channel's size, and only
         * reduces: a sub larger than its main is a bind MI accepts and then does
         * not honour. Board-measured on an SSC377QE -- main set to 320x240 under a
         * 640x480 sub binds clean, logs nothing, and the sub polls ETIMEDOUT until
         * the main is grown again, at which point it recovers by itself.
         *
         * So it warns rather than refusing. The state is legal, transient and
         * self-healing, and the caller is rvd doing what it was asked; what was
         * missing was any word of why a stream had gone quiet.
         */
        if (enc->width > st->enc[main_chn].width || enc->height > st->enc[main_chn].height)
            HAL_LOG_WARN("venc chn %d is %ux%u and cascades off chn %d at %ux%u -- the VENC "
                         "cascade only scales down, so this stream will deliver nothing until "
                         "the main is at least as large",
                         chn, enc->width, enc->height, main_chn, st->enc[main_chn].width,
                         st->enc[main_chn].height);

        if (!enc->receiving) {
            if ((ret = st->venc.start_recv(enc->device, (unsigned int)chn)) != 0) {
                HAL_LOG_ERR("MI_VENC_StartRecvPic(chn %d) failed: %d", chn, ret);
                return RSS_ERR_IO;
            }
            enc->receiving = true;
        }

        memset(&src, 0, sizeof(src));
        memset(&dst, 0, sizeof(dst));
        src.module = I6C_SYS_MOD_VENC;
        src.device = enc->device;
        src.channel = (unsigned int)main_chn;
        src.port = 0;
        dst.module = I6C_SYS_MOD_VENC;
        dst.device = enc->device;
        dst.channel = (unsigned int)chn;
        dst.port = 0;

        ret = st->sys.bind_ext(I6C_SOC_ID, &src, &dst, st->fps, dst_fps ? dst_fps : st->fps,
                               I6C_SYS_LINK_RING, 0);
        if (ret) {
            HAL_LOG_ERR("MI_SYS_BindChnPort2(venc chn %d -> venc chn %d) cascade failed: %d",
                        main_chn, chn, ret);
            return RSS_ERR_IO;
        }
        enc->bound = true;
        enc->cascade = true;
        enc->cascade_src = main_chn;

        /* A shallow queue on the encoder's output for the packetiser to drain. */
        i6c_set_output_depth(st, I6C_SYS_MOD_VENC, enc->device, (unsigned int)chn, 0, 1, 3);

        HAL_LOG_DBG("infinity6c: venc chn %d cascaded off main chn %d, ring at %u fps", chn,
                    main_chn, dst_fps ? dst_fps : st->fps);

        /*
         * The channel exists and knows it is a cascade, so its regions can settle:
         * text attaches to this encoder's own input port, and a cover is declined
         * here because the scaler-port cover already reaches this stream through
         * the ring. See i6c_osd_target_port.
         */
        i6c_osd_flush_pending(st, chn);

        return RSS_OK;
    }

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    src.module = I6C_SYS_MOD_SCL;
    src.device = I6C_SCL_DEV;
    src.channel = I6C_SCL_CHN;
    src.port = (unsigned int)port;
    dst.module = I6C_SYS_MOD_VENC;
    dst.device = enc->device;
    dst.channel = (unsigned int)chn;
    dst.port = 0;

    /*
     * A ring leg only moves data from an IFC-compressed source port, so the port
     * is switched to IFC now that this bind is known to be a ring. A frame leg
     * keeps the uncompressed port it was created with.
     */
    if (enc->uses_ring && (ret = i6c_fs_port_ifc(st, port)) != RSS_OK)
        return ret;

    /*
     * Enable the source port before the bind on a ring leg, the order both
     * references use (divinus's channel_bind is EnablePort -> BindExt): the ring's
     * producer has to be live when the ring is connected. rvd enables the
     * framesource port later, in its own enable pass; that call finds it already
     * enabled and no-ops.
     */
    if (enc->uses_ring && (ret = i6c_fs_enable_port(st, port)) != RSS_OK)
        return ret;

    /*
     * OSD regions attach while the SCL port is configured and enabled but not yet
     * bound, and before StartRecvPic spawns the encoder kthread -- the point both
     * i6c references attach at. The kernel RGN driver builds its singlethread
     * workqueue on the task that first calls into it, which is this one, the same
     * thread that brings the pipeline up.
     */
    i6c_osd_flush_pending(st, chn);

    /*
     * The encoder must already be receiving when the bind connects it: the vendor
     * order is CreateChn -> SetInputSourceConfig -> StartRecvPic -> bind, and both
     * i6c references start the channel before binding it. A ring bound to a
     * channel that has not started has no consumer, so it never drains -- the
     * scaler back-pressures, the ISP rewinds, and VIF completes no frame. Started
     * here rather than left to hal_enc_start, which rvd calls only after the whole
     * pipeline is bound; that call then finds it already receiving and no-ops.
     */
    if (!enc->receiving) {
        if ((ret = st->venc.start_recv(enc->device, (unsigned int)chn)) != 0) {
            HAL_LOG_ERR("MI_VENC_StartRecvPic(chn %d) failed: %d", chn, ret);
            return RSS_ERR_IO;
        }
        enc->receiving = true;
    }

    link = enc->uses_ring ? I6C_SYS_LINK_RING : I6C_SYS_LINK_FRAMEBASE;

    ret = st->sys.bind_ext(I6C_SOC_ID, &src, &dst, st->fps, dst_fps ? dst_fps : st->fps, link, 0);
    if (ret) {
        HAL_LOG_ERR("MI_SYS_BindChnPort2(SCL port %d -> VENC dev %u chn %d) failed: %d", port,
                    enc->device, chn, ret);
        return RSS_ERR_IO;
    }
    enc->bound = true;
    enc->src_port = port;

    /* A shallow queue on the encoder's output for the packetiser to drain. */
    i6c_set_output_depth(st, I6C_SYS_MOD_VENC, enc->device, (unsigned int)chn, 0, 1, 3);

    HAL_LOG_DBG("infinity6c: SCL port %d -> venc dev %u chn %d bound, %s at %u fps", port,
                enc->device, chn, link == I6C_SYS_LINK_RING ? "ring" : "frame-base",
                dst_fps ? dst_fps : st->fps);

    /*
     * A sub left waiting when the previous main was destroyed gets this one. Last,
     * because a sub's cascade needs its source bound, receiving and holding the
     * ring, which is exactly what the lines above have just finished making true.
     */
    if (enc->uses_ring)
        i6c_enc_recascade_subs(st, chn);

    return RSS_OK;
}

int i6c_unbind_scl_from_venc(infinity6c_state_t *st, int chn)
{
    infinity6c_venc_chn_t *enc = &st->enc[chn];
    i6c_sys_bind src;
    i6c_sys_bind dst;
    int ret;

    if (!enc->bound)
        return RSS_OK;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    /* A cascade sub is unbound from its main VENC channel, not from an SCL port. */
    if (enc->cascade) {
        src.module = I6C_SYS_MOD_VENC;
        src.device = enc->device;
        src.channel = (unsigned int)enc->cascade_src;
        src.port = 0;
        dst.module = I6C_SYS_MOD_VENC;
        dst.device = enc->device;
        dst.channel = (unsigned int)chn;
        dst.port = 0;

        ret = st->sys.unbind(I6C_SOC_ID, &src, &dst);
        enc->bound = false;
        enc->cascade = false;
        enc->cascade_src = -1;
        if (ret) {
            HAL_LOG_ERR("MI_SYS_UnBindChnPort(venc chn %d cascade) failed: %d", chn, ret);
            return RSS_ERR_IO;
        }
        return RSS_OK;
    }

    if (enc->src_port < 0 || enc->src_port >= I6C_MAX_CHN) {
        HAL_LOG_ERR("i6c_venc chn %d: bound with no port recorded", chn);
        enc->bound = false;
        return RSS_ERR_INVAL;
    }

    src.module = I6C_SYS_MOD_SCL;
    src.device = I6C_SCL_DEV;
    src.channel = I6C_SCL_CHN;
    src.port = (unsigned int)enc->src_port;
    dst.module = I6C_SYS_MOD_VENC;
    dst.device = enc->device;
    dst.channel = (unsigned int)chn;
    dst.port = 0;

    ret = st->sys.unbind(I6C_SOC_ID, &src, &dst);
    enc->bound = false;
    enc->src_port = -1;
    if (ret) {
        HAL_LOG_ERR("MI_SYS_UnBindChnPort(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * i6c_enc_release_own_port -- give back a port this channel brought up itself.
 *
 * The port has to be read before the unbind, which forgets it. A no-op unless
 * the channel owns one, so it is safe to call on every teardown path.
 */
static void i6c_enc_release_own_port(infinity6c_state_t *st, int chn)
{
    infinity6c_venc_chn_t *enc = &st->enc[chn];
    int port = enc->src_port;

    if (!enc->owns_port)
        return;

    i6c_unbind_scl_from_venc(st, chn);
    i6c_fs_release_port(st, port);
    enc->owns_port = false;
}

/* ================================================================
 * GROUPS
 *
 * MI has no encoder groups, on either generation. rvd's model is
 * IMP's -- create a group, register a channel into it, bind the group
 * -- and here the whole triple is one MI_SYS bind, made by hal_bind.
 *
 * So create_group and destroy_group validate and do nothing else.
 * Leaving them out instead is not the harmless honesty it looks like:
 * rvd_stream_init calls enc_create_group first for every non-JPEG
 * stream and returns on anything but RSS_OK, so an absent op fails the
 * stream before the encoder channel is even created -- no video at
 * all, for want of a concept the hardware does not have.
 *
 * register_channel is where the fiction stops being free.
 * ================================================================ */

int hal_enc_create_group(void *ctx, int grp)
{
    I6C_ENTER(ctx, grp, st);

    (void)st;

    return RSS_OK;
}

int hal_enc_destroy_group(void *ctx, int grp)
{
    return hal_enc_create_group(ctx, grp);
}

/*
 * hal_enc_register_channel -- how a JPEG channel gets fed.
 *
 * rvd skips the bind chain entirely for a JPEG stream (`if (!s->is_jpeg)` around
 * the chain in rvd_stream_init) because on IMP registering a second channel into
 * the paired video stream's group is what feeds it -- the group is already bound
 * to the framesource, so both registered channels see frames.
 *
 * MI has no groups, so validating and returning here would leave the JPEG
 * channel connected to nothing: MI_VENC_CreateChn runs, MI_VENC_StartRecvPic
 * runs, no SCL port is ever bound, and the poll times out forever. That failure
 * is quiet by construction -- rvd reads a JPEG poll timeout as the expected
 * "sensor idle" case -- so it looks like /snap saying no snapshot is available
 * while the ring and the H.264 stream are healthy.
 *
 * So the channel gets an SCL output port of its own, and it must be its own:
 * sharing the video stream's port is not a fallback here, it is refused. MI
 * answers a second bind on an already-bound source with MI_ERR_SYS_BUSY
 * (0xA0092012), which is the vendor's "neither end may already be bound" note
 * being enforced rather than merely documented -- unlike on MI 2.x, where the
 * same shape works and star/ relies on it.
 *
 * A port of its own is also what buys the frame pacing, since the bind carries a
 * destination rate and MI drops the difference in hardware -- the caps block
 * sets no jpeg_pulse, so rvd's own duty-cycling is off on this part.
 *
 * Which port it can have is not a free choice: the narrow scalers behind ports 2
 * and 3 stop at 1920 wide, so a full-resolution snapshot fits only on port 0 or
 * 1. See i6c_fs_spare_port, which knows both that and the fact that a cascade
 * sub occupies an index without occupying a port.
 *
 * Every failure warns and returns RSS_OK rather than propagating. rvd treats a
 * register failure as fatal to the stream, and a board that cannot feed its JPEG
 * channel should lose its snapshots, not its video.
 */
int hal_enc_register_channel(void *ctx, int grp, int chn)
{
    infinity6c_venc_chn_t *enc;
    i6c_common_pixfmt pixfmt;
    unsigned int snap_fps;
    int port;
    int ret;

    I6C_ENTER(ctx, chn, st);

    if (grp < 0 || grp >= I6C_MAX_CHN)
        return RSS_ERR_INVAL;

    enc = &st->enc[chn];
    if (!enc->created)
        return RSS_ERR_NOENT;

    /*
     * A video stream registers into its own group -- rvd passes s->chn twice --
     * and hal_bind connects it immediately afterwards. The group is the fiction,
     * the bind is the reality.
     */
    if (grp == chn)
        return RSS_OK;

    /* Already fed; rvd re-registers on a per-stream hot restart. */
    if (enc->bound)
        return RSS_OK;

    /*
     * The paired video channel is not consulted for geometry: this channel has
     * its own, and MI_VENC_CreateChn has already been told it. So a pairing that
     * never got an SCL port is no longer a reason to give up, which on this part
     * is the ordinary case for a sub stream -- it is a VENC cascade off the main
     * and holds no port at all, and that used to cost the sub its snapshots
     * outright.
     */
    snap_fps = enc->cfg.fps_num / (enc->cfg.fps_den ? enc->cfg.fps_den : 1);

    port = i6c_fs_spare_port(st, enc->width);
    if (port < 0) {
        HAL_LOG_WARN("venc chn %d: no SCL output port free that will go %u wide -- no snapshots "
                     "on this stream",
                     chn, enc->width);
        return RSS_OK;
    }

    /*
     * The format the rest of the chain already runs in. Taken from the video
     * port rather than assumed, and only defaulted when there is somehow no
     * video port to ask -- which cannot happen on the path that gets here, since
     * an encoder channel exists.
     */
    pixfmt = st->scl_video_port >= 0 ? st->fs[st->scl_video_port].pixfmt : I6C_PIXFMT_YUV420SP;

    if ((ret = i6c_fs_snapshot_port(st, port, enc->width, enc->height, pixfmt)) != RSS_OK) {
        HAL_LOG_WARN("venc chn %d: SCL port %d would not take %ux%u: %d -- no snapshots on this "
                     "stream",
                     chn, port, enc->width, enc->height, ret);
        return RSS_OK;
    }

    if ((ret = i6c_bind_scl_to_venc(st, port, chn, snap_fps)) != RSS_OK) {
        i6c_fs_release_port(st, port);
        HAL_LOG_WARN("venc chn %d: could not bind SCL port %d: %d -- no snapshots on this stream",
                     chn, port, ret);
        return RSS_OK;
    }

    if ((ret = i6c_fs_enable_port(st, port)) != RSS_OK) {
        i6c_unbind_scl_from_venc(st, chn);
        i6c_fs_release_port(st, port);
        HAL_LOG_WARN("venc chn %d: could not enable SCL port %d: %d -- no snapshots on this stream",
                     chn, port, ret);
        return RSS_OK;
    }

    enc->owns_port = true;
    HAL_LOG_INFO("infinity6c: venc chn %d takes SCL port %d for snapshots, %ux%u at %u fps", chn,
                 port, enc->width, enc->height, snap_fps);

    return RSS_OK;
}

int hal_enc_unregister_channel(void *ctx, int chn)
{
    I6C_ENTER(ctx, chn, st);

    /*
     * Only a snapshot channel has anything to undo. A video channel's bind
     * belongs to rvd's unbind chain rather than to its group membership, and
     * dropping it here would disconnect a stream that is still running.
     */
    i6c_enc_release_own_port(st, chn);

    return RSS_OK;
}

/* ================================================================
 * CHANNEL LIFECYCLE
 * ================================================================ */

/*
 * i6c_enc_dev_up -- bring up the codec engine a channel is about to live on.
 *
 * MI 3.0 puts a device above the VENC channel and refuses to create a channel on
 * a device that has not been created -- MI_VENC_CreateChn returns NOT_CONFIG. MI
 * 2.x had no such object, which is why this has no counterpart in star/. The
 * device is created once by the first channel that needs it and torn down with
 * the rest at deinit; the max dimensions are the vendor's own ceilings per
 * engine, wide enough that no stream this part can source ever resizes it.
 */
static int i6c_enc_dev_up(infinity6c_state_t *st, unsigned int device)
{
    int slot = i6c_enc_pool_slot(device);
    i6c_venc_init init;
    int ret;

    if (st->enc_dev_up[slot])
        return RSS_OK;

    memset(&init, 0, sizeof(init));
    if (device == I6C_VENC_DEV_MJPG_0) {
        init.maxWidth = 8192;
        init.maxHeight = 6480;
    } else {
        init.maxWidth = 4096;
        init.maxHeight = 2176;
    }

    if ((ret = st->venc.create_dev(device, &init)) != 0) {
        HAL_LOG_ERR("MI_VENC_CreateDev(dev %u, %ux%u) failed: %d", device, init.maxWidth,
                    init.maxHeight, ret);
        return RSS_ERR_IO;
    }

    st->enc_dev_up[slot] = true;
    return RSS_OK;
}

/*
 * i6c_enc_pool -- the encoder's own ring pool, one per codec engine.
 *
 * Distinct from the scaler's: a ring bind has a pool at each end, and this one
 * is where the encoder reads from. The whole frame height rather than a quarter,
 * because the encoder is the consumer here and has nothing downstream to hand a
 * partial frame to.
 *
 * Sized for the largest channel on the engine rather than for the calling one.
 * The pool is per *device*: i6c_sys_poolring names a module and a device and
 * nothing finer, so every channel on one engine shares it. Configuring it once
 * per channel would therefore have a sub-stream shrink the pool the main stream
 * is already reading from -- so a request that fits what is there is skipped,
 * and only a larger one reconfigures.
 */
static int i6c_enc_pool(infinity6c_state_t *st, unsigned int device, const rss_video_config_t *cfg)
{
    int slot = i6c_enc_pool_slot(device);
    unsigned short have_w = st->enc_pool_w[slot];
    unsigned short have_h = st->enc_pool_h[slot];
    i6c_sys_pool pool;
    int ret;

    if (have_w && have_w >= cfg->width && have_h >= cfg->height)
        return RSS_OK;

    /*
     * Growing one that is already in use is the case with no evidence behind it:
     * rvd creates its main stream first, so in practice the first channel sizes
     * the pool and every later one fits. Said out loud because if this line ever
     * appears it is the first thing to suspect.
     */
    if (have_w)
        HAL_LOG_WARN("infinity6c: growing venc dev %u ring pool from %ux%u to %ux%u with channels "
                     "already on it",
                     device, have_w, have_h, cfg->width, cfg->height);

    memset(&pool, 0, sizeof(pool));
    pool.type = I6C_SYS_POOL_DEVICE_RING;
    pool.create = 1;
    pool.config.ring.module = I6C_SYS_MOD_VENC;
    pool.config.ring.device = device;
    pool.config.ring.maxWidth = have_w > cfg->width ? have_w : cfg->width;
    pool.config.ring.maxHeight = have_h > cfg->height ? have_h : cfg->height;
    pool.config.ring.ringLine = pool.config.ring.maxHeight;

    if ((ret = st->sys.config_pool(I6C_SOC_ID, &pool)) != 0) {
        HAL_LOG_ERR("MI_SYS_ConfigPrivateMMAPool(VENC dev %u ring, %ux%u) failed: %d", device,
                    pool.config.ring.maxWidth, pool.config.ring.maxHeight, ret);
        return RSS_ERR_IO;
    }

    st->enc_pool_w[slot] = pool.config.ring.maxWidth;
    st->enc_pool_h[slot] = pool.config.ring.maxHeight;

    return RSS_OK;
}

int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg)
{
    infinity6c_venc_chn_t *enc;
    i6c_venc_codec codec;
    i6c_venc_chn attr;
    unsigned int device;
    int slot;
    bool uses_ring;
    int ret;

    I6C_ENTER(ctx, chn, st);

    if (!cfg)
        return RSS_ERR_INVAL;

    enc = &st->enc[chn];
    if (enc->created)
        return RSS_ERR_BUSY;

    if ((ret = i6c_enc_device(cfg->codec, &device)) != RSS_OK) {
        HAL_LOG_ERR("i6c_venc: codec %d is not encoded by this part", (int)cfg->codec);
        return ret;
    }
    if ((ret = i6c_enc_codec(cfg->codec, &codec)) != RSS_OK)
        return ret;

    /* The device before its pool and its channel: both hang off a created device. */
    if ((ret = i6c_enc_dev_up(st, device)) != RSS_OK)
        return ret;

    /*
     * The first H.26x channel on an engine takes the SCL ring; a later one reads
     * frames from DRAM instead, because the SCL channel drives one ring consumer
     * at a time and a second ring bind is refused. JPEG is always frame-based and
     * never takes the ring. Decided before the pool because the two go together:
     * the VENC ring pool is what a ring-fed channel reads from, and a frame-fed one
     * takes its frames from the SCL output port's pool instead. Creating a ring
     * pool on the JPEG engine -- which never takes a ring -- faults the kernel
     * allocator, so the pool is made only for the ring channel.
     */
    slot = i6c_enc_pool_slot(device);
    uses_ring = device != I6C_VENC_DEV_MJPG_0 && st->enc_ring_chn[slot] < 0;

    if (uses_ring && (ret = i6c_enc_pool(st, device, cfg)) != RSS_OK)
        return ret;

    i6c_enc_fill_attrib(&attr.attrib, cfg, codec);
    if (i6c_enc_fill_rate(&attr.rate, cfg, codec))
        i6c_enc_note_rc_substitution(enc, chn, cfg->rc_mode, &attr.rate);

    if ((ret = st->venc.create_chn(device, (unsigned int)chn, &attr)) != 0) {
        HAL_LOG_ERR("MI_VENC_CreateChn(dev %u, chn %d) failed: %d", device, chn, ret);
        return RSS_ERR_IO;
    }

    enc->created = true;
    enc->device = device;
    enc->codec = cfg->codec;
    enc->width = cfg->width;
    enc->height = cfg->height;
    enc->fd = -1;
    enc->src_port = -1;
    enc->uses_ring = uses_ring;
    enc->cfg = *cfg;

    /*
     * Only the ring channel gets the ring-DMA input config; a frame channel stays
     * in MI's default frame input. The engine's ring slot is claimed here, once
     * the channel is known to exist, so a create that fails earlier never holds it.
     * See i6c_bind_scl_to_venc.
     */
    if (uses_ring) {
        i6c_venc_src_conf src = I6C_VENC_SRC_CONF_RING_DMA;
        int mi;

        if ((mi = st->venc.set_src_conf(device, (unsigned int)chn, &src)) != 0) {
            HAL_LOG_ERR("MI_VENC_SetInputSourceConfig(chn %d) failed: %d", chn, mi);
            ret = RSS_ERR_IO;
            goto fail;
        }
        st->enc_ring_chn[slot] = chn;
    }

    /*
     * Not bound here. rvd binds a video stream itself, through the FS -> ENC
     * chain that reaches this backend as hal_bind, and it skips that chain for a
     * JPEG stream -- which hal_enc_register_channel serves instead. Binding at
     * creation would double-bind the first case and bind an unconfigured port in
     * the second, and MI_SYS_BindChnPort2 requires that neither end already be
     * bound.
     */
    HAL_LOG_INFO("infinity6c: venc chn %d created on dev %u, %ux%u", chn, device, cfg->width,
                 cfg->height);
    return RSS_OK;

fail:
    st->venc.destroy_chn(device, (unsigned int)chn);
    enc->created = false;
    return ret;
}

int hal_enc_destroy_channel(void *ctx, int chn)
{
    infinity6c_venc_chn_t *enc;

    I6C_ENTER(ctx, chn, st);

    enc = &st->enc[chn];
    if (!enc->created)
        return RSS_OK;

    /*
     * Subs before anything of this channel's own: they consume its VENC ring, so a
     * sub still bound when this channel goes is a sub bound to nothing. i6c_teardown_all
     * makes the same pre-pass before its own destroy loop; this is the single-channel
     * path, which is the one rvd takes for set-resolution and set-codec.
     */
    i6c_enc_detach_cascade_subs(st, chn);

    if (enc->frame_held)
        st->venc.release_stream(enc->device, (unsigned int)chn, &enc->strm);
    if (enc->receiving)
        st->venc.stop_recv(enc->device, (unsigned int)chn);
    if (enc->fd >= 0)
        st->venc.close_fd(enc->device, (unsigned int)chn);

    /* Gives back a snapshot channel's own port; no-op for a video channel. */
    i6c_enc_release_own_port(st, chn);
    i6c_unbind_scl_from_venc(st, chn);
    st->venc.destroy_chn(enc->device, (unsigned int)chn);

    /* Free the engine's ring if this channel held it, so a later one can take it. */
    {
        int slot = i6c_enc_pool_slot(enc->device);

        if (st->enc_ring_chn[slot] == chn)
            st->enc_ring_chn[slot] = -1;
    }

    free(enc->heap_packs);
    memset(enc, 0, sizeof(*enc));
    enc->fd = -1;
    enc->src_port = -1;

    return RSS_OK;
}

int hal_enc_start(void *ctx, int chn)
{
    infinity6c_venc_chn_t *enc;
    int ret;

    I6C_ENTER(ctx, chn, st);

    enc = &st->enc[chn];
    if (!enc->created)
        return RSS_ERR_INVAL;
    if (enc->receiving)
        return RSS_OK;

    if ((ret = st->venc.start_recv(enc->device, (unsigned int)chn)) != 0) {
        HAL_LOG_ERR("MI_VENC_StartRecvPic(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }
    enc->receiving = true;

    return RSS_OK;
}

int hal_enc_stop(void *ctx, int chn)
{
    infinity6c_venc_chn_t *enc;
    int ret;

    I6C_ENTER(ctx, chn, st);

    enc = &st->enc[chn];
    if (!enc->receiving)
        return RSS_OK;

    ret = st->venc.stop_recv(enc->device, (unsigned int)chn);
    enc->receiving = false;
    if (ret) {
        HAL_LOG_ERR("MI_VENC_StopRecvPic(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* ================================================================
 * STREAM
 * ================================================================ */

/*
 * hal_enc_get_fd -- the descriptor that signals a ready frame.
 *
 * Cached: MI opens one per channel and reopening it per poll would be a syscall
 * pair per frame for nothing. Released with the channel.
 */
int hal_enc_get_fd(void *ctx, int chn)
{
    infinity6c_venc_chn_t *enc;
    int fd;

    I6C_ENTER(ctx, chn, st);

    enc = &st->enc[chn];
    if (!enc->created)
        return RSS_ERR_INVAL;
    if (enc->fd >= 0)
        return enc->fd;

    fd = st->venc.get_fd(enc->device, (unsigned int)chn);
    if (fd < 0) {
        HAL_LOG_ERR("MI_VENC_GetFd(chn %d) failed: %d", chn, fd);
        return RSS_ERR_IO;
    }
    enc->fd = fd;

    return fd;
}

int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms)
{
    struct timeval tv;
    fd_set readfds;
    int fd;
    int ret;

    fd = hal_enc_get_fd(ctx, chn);
    if (fd < 0)
        return fd;

    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    tv.tv_sec = (time_t)(timeout_ms / 1000);
    tv.tv_usec = (suseconds_t)(timeout_ms % 1000) * 1000;

    ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    if (ret < 0) {
        /* A signal is a retry, not a fault -- the daemon takes signals routinely. */
        if (errno == EINTR)
            return -EAGAIN;
        HAL_LOG_ERR("i6c_venc: select on chn %d failed: %s", chn, strerror(errno));
        return RSS_ERR_IO;
    }
    /* Named the same as the other backends report it; rvd only tests for RSS_OK. */
    if (ret == 0)
        return RSS_ERR_TIMEOUT;

    return RSS_OK;
}

int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame)
{
    infinity6c_venc_chn_t *enc;
    i6c_venc_stat stat;
    int ret;

    I6C_ENTER(ctx, chn, st);

    if (!frame)
        return RSS_ERR_INVAL;

    enc = &st->enc[chn];
    if (!enc->created)
        return RSS_ERR_INVAL;
    if (enc->frame_held) {
        HAL_LOG_ERR("i6c_venc chn %d: get_frame with a frame still held", chn);
        return RSS_ERR_BUSY;
    }

    memset(&stat, 0, sizeof(stat));
    if ((ret = st->venc.query(enc->device, (unsigned int)chn, &stat)) != 0) {
        HAL_LOG_ERR("MI_VENC_Query(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }
    if (!stat.curPacks)
        return -EAGAIN;

    memset(&enc->strm, 0, sizeof(enc->strm));
    enc->strm.packet = i6c_enc_packs(enc, stat.curPacks);
    if (!enc->strm.packet)
        return RSS_ERR_NOMEM;
    enc->strm.count = stat.curPacks;

    /*
     * Zero timeout: the descriptor already said a frame is ready, and this call
     * moves descriptors rather than pixels. The vendor's reference passes the
     * pack count in this argument, which MI reads as milliseconds -- harmless,
     * but not what it looks like.
     */
    if ((ret = st->venc.get_stream(enc->device, (unsigned int)chn, &enc->strm, 0)) != 0) {
        HAL_LOG_ERR("MI_VENC_GetStream(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    enc->frame_held = true;

    /* A frame is through: safe now to load the ISP tuning (loads once). */
    i6c_isp_note_frame(st);

    memset(frame, 0, sizeof(*frame));
    frame->codec = enc->codec;
    frame->seq = enc->strm.sequence;
    /* MI timestamps packs in microseconds, which is what rss_frame_t wants. */
    frame->timestamp = enc->strm.count ? (int64_t)enc->strm.packet[0].timestamp : 0;
    i6c_enc_fill_nals(enc, frame);
    frame->_priv = enc;

    return RSS_OK;
}

int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame)
{
    infinity6c_venc_chn_t *enc;
    int ret;

    I6C_ENTER(ctx, chn, st);

    enc = &st->enc[chn];
    if (!enc->frame_held)
        return RSS_OK;

    ret = st->venc.release_stream(enc->device, (unsigned int)chn, &enc->strm);
    enc->frame_held = false;
    memset(&enc->strm, 0, sizeof(enc->strm));
    if (frame) {
        frame->nals = NULL;
        frame->nal_count = 0;
        frame->_priv = NULL;
    }

    if (ret) {
        HAL_LOG_ERR("MI_VENC_ReleaseStream(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

int hal_enc_request_idr(void *ctx, int chn)
{
    infinity6c_venc_chn_t *enc;
    int ret;

    I6C_ENTER(ctx, chn, st);

    enc = &st->enc[chn];
    if (!enc->created)
        return RSS_ERR_INVAL;
    /* JPEG has no inter-frame prediction, so every frame is already a keyframe. */
    if (enc->device == I6C_VENC_DEV_MJPG_0)
        return RSS_OK;

    if ((ret = st->venc.request_idr(enc->device, (unsigned int)chn, 1)) != 0) {
        HAL_LOG_ERR("MI_VENC_RequestIdr(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* ================================================================
 * RUNTIME RECONFIGURATION
 *
 * MI exposes no per-knob setter: bitrate, GOP and frame rate all live
 * in the rate half of the channel attribute, which is read, modified
 * and written back whole.
 * ================================================================ */

/*
 * i6c_enc_reapply_rate -- rebuild the rate half from the cached settings.
 *
 * The whole attribute is fetched rather than only rebuilt, so that the codec
 * half written back is byte-for-byte what the driver already holds. It compares
 * that half and refuses a change to anything but the resolution and profile, so
 * a locally reconstructed copy risks being rejected over a field neither the
 * caller nor this function meant to touch.
 */
static int i6c_enc_reapply_rate(infinity6c_state_t *st, int chn)
{
    infinity6c_venc_chn_t *enc = &st->enc[chn];
    i6c_venc_codec codec;
    i6c_venc_chn attr;
    int ret;

    if (!enc->created)
        return RSS_ERR_INVAL;
    if ((ret = i6c_enc_codec(enc->codec, &codec)) != RSS_OK)
        return ret;

    memset(&attr, 0, sizeof(attr));
    if ((ret = st->venc.get_chn_attr(enc->device, (unsigned int)chn, &attr)) != 0) {
        HAL_LOG_ERR("MI_VENC_GetChnAttr(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    if (i6c_enc_fill_rate(&attr.rate, &enc->cfg, codec))
        i6c_enc_note_rc_substitution(enc, chn, enc->cfg.rc_mode, &attr.rate);

    if ((ret = st->venc.set_chn_attr(enc->device, (unsigned int)chn, &attr)) != 0) {
        HAL_LOG_ERR("MI_VENC_SetChnAttr(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate)
{
    I6C_ENTER(ctx, chn, st);

    st->enc[chn].cfg.rc_mode = mode;
    if (bitrate)
        st->enc[chn].cfg.bitrate = bitrate;

    /* Naming a mode is a fresh question, so it gets a fresh answer if it is refused. */
    st->enc[chn].rc_substituted = false;

    return i6c_enc_reapply_rate(st, chn);
}

int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate)
{
    I6C_ENTER(ctx, chn, st);

    if (!bitrate)
        return RSS_ERR_INVAL;
    st->enc[chn].cfg.bitrate = bitrate;

    return i6c_enc_reapply_rate(st, chn);
}

int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length)
{
    I6C_ENTER(ctx, chn, st);

    if (!gop_length)
        return RSS_ERR_INVAL;
    st->enc[chn].cfg.gop_length = gop_length;

    return i6c_enc_reapply_rate(st, chn);
}

int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den)
{
    I6C_ENTER(ctx, chn, st);

    if (!fps_num || !fps_den)
        return RSS_ERR_INVAL;
    st->enc[chn].cfg.fps_num = fps_num;
    st->enc[chn].cfg.fps_den = fps_den;

    return i6c_enc_reapply_rate(st, chn);
}

int hal_enc_query(void *ctx, int chn, bool *busy)
{
    i6c_venc_stat stat;
    int ret;

    I6C_ENTER(ctx, chn, st);

    if (!busy)
        return RSS_ERR_INVAL;
    if (!st->enc[chn].created)
        return RSS_ERR_INVAL;

    memset(&stat, 0, sizeof(stat));
    if ((ret = st->venc.query(st->enc[chn].device, (unsigned int)chn, &stat)) != 0) {
        HAL_LOG_ERR("MI_VENC_Query(chn %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    /* Frames accepted but not yet encoded is what "busy" means to raptor. */
    *busy = stat.leftEncPics > 0;

    return RSS_OK;
}

/*
 * i6c_teardown_all -- release every channel, in dependency order.
 *
 * Encoders first so that nothing is still bound to an SCL port when the
 * pipeline goes, then the pipeline itself. Called from hal_deinit, where the
 * daemon may be exiting on a signal and cannot be relied on to have closed its
 * channels.
 */
void i6c_teardown_all(infinity6c_state_t *st)
{
    bool drained = false;
    int i;

    /*
     * OSD first of all: a region is attached to a VENC channel, and detaching it
     * after that channel is destroyed would reference an object that no longer
     * exists. i6c_osd_release_all detaches, destroys and deinits RGN.
     */
    i6c_osd_release_all(st);

    /*
     * Stop the producers first. Every SCL output port is disabled and given a
     * moment to settle before any StopRecvPic or unbind below: a source port left
     * producing while its encoder stops strands an SCL -> VENC task that can never
     * complete, and the unbind's kernel flush then waits on it forever in
     * uninterruptible D-state. The wait is bounded, so a leg that never delivered
     * a frame still tears down rather than wedging. This is the order the working
     * i6c reference tears down in.
     */
    for (i = 0; i < I6C_MAX_CHN; i++) {
        if (st->fs[i].enabled) {
            st->scl.disable_port(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, (unsigned int)i);
            st->fs[i].enabled = false;
            drained = true;
        }
    }
    if (drained) {
        struct timeval tv = {.tv_sec = 0, .tv_usec = 50000};
        select(0, NULL, NULL, NULL, &tv);
    }

    /*
     * Unbind the cascade subs before the loop below destroys any channel: a sub
     * consumes the main's VENC ring, so destroying the main while a sub is still
     * bound to it would strand the sub. Their own destroy happens in the loop.
     *
     * Through the same helper the single-channel destroy uses, rather than a
     * second copy of the rule: this invariant held here and not there, and that
     * asymmetry is what cost the sub stream on every live set-resolution. The
     * cascade_pending it leaves behind is meaningless on this path -- nothing is
     * coming back -- and the memset below clears it.
     */
    for (i = 0; i < I6C_MAX_CHN; i++)
        i6c_enc_detach_cascade_subs(st, i);

    for (i = 0; i < I6C_MAX_CHN; i++) {
        infinity6c_venc_chn_t *enc = &st->enc[i];

        if (!enc->created)
            continue;

        if (enc->frame_held)
            st->venc.release_stream(enc->device, (unsigned int)i, &enc->strm);
        if (enc->receiving)
            st->venc.stop_recv(enc->device, (unsigned int)i);
        if (enc->fd >= 0)
            st->venc.close_fd(enc->device, (unsigned int)i);

        i6c_enc_release_own_port(st, i);
        i6c_unbind_scl_from_venc(st, i);
        st->venc.destroy_chn(enc->device, (unsigned int)i);

        free(enc->heap_packs);
        memset(enc, 0, sizeof(*enc));
        enc->fd = -1;
        enc->src_port = -1;
    }

    /*
     * Devices after channels: MI_VENC_DestroyDev needs its channels gone, and the
     * loop above has just removed them. Indices follow i6c_enc_pool_slot -- slot 0
     * is the H.26x engine, slot 1 MJPEG.
     */
    {
        static const unsigned int devs[I6C_VENC_DEV_SLOTS] = {I6C_VENC_DEV_H26X_0,
                                                              I6C_VENC_DEV_MJPG_0};
        for (i = 0; i < I6C_VENC_DEV_SLOTS; i++) {
            st->enc_ring_chn[i] = -1;
            if (!st->enc_dev_up[i])
                continue;
            st->venc.destroy_dev(devs[i]);
            st->enc_dev_up[i] = false;
        }
    }

    /* Forced rather than reference counted: the daemon is going away. */
    st->pipeline_refs = 1;
    i6c_pipeline_destroy(st);
}
