/*
 * star/star_state.h -- shared backend state for the SigmaStar MI HAL
 *
 * Exists because the MI backend spans more than one translation unit:
 * hal_common.c owns the pipeline lifecycle (MI_SYS / MI_SNR / MI_VIF /
 * the VPE channel) and hal_framesource.c owns the VPE output ports, and
 * both need the same loaded-library handles and sensor descriptors.
 *
 * It cannot live in hal_internal.h: that header is included *by* the
 * i6_*.h ABI headers (for HAL_LOG_ERR and RSS_ERR_*), so it must not
 * include them back. rss_hal_ctx_t->platform points at the
 * star_state_t below for exactly this reason -- see the MI BACKEND
 * STATE comment in hal_common.c.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef STAR_STATE_H
#define STAR_STATE_H

#include "hal_internal.h"

#include "i6_aud_load.h"
#include "i6_isp_load.h"
#include "i6_rgn_load.h"
#include "i6_snr_load.h"
#include "i6_sys_load.h"
#include "i6_venc_load.h"
#include "i6_vif_load.h"
#include "i6_vpe_load.h"

/* ================================================================
 * FIXED TOPOLOGY
 *
 * One sensor on VIF device 0 / channel 0 / port 0, feeding VPE
 * device 0 / channel 0. MI's device and channel numbering only
 * becomes interesting with several sensors, and this target has one.
 * Task 2e generalizes sensor *selection*; the topology stays fixed.
 * ================================================================ */

#define STAR_SNR_INDEX 0
#define STAR_VIF_DEV 0
#define STAR_VIF_CHN 0
#define STAR_VIF_PORT 0
#define STAR_VPE_DEV 0
#define STAR_VPE_CHN 0

/*
 * ISP channel. Every MI_ISP call takes a channel, and it is the *VPE*
 * channel index -- MI's ISP is not a separate device but the front half
 * of the VPE channel, which is why enabling VPE auto-starts CUS3A and
 * why the IQ binary is loaded per VPE channel. Both references pass a
 * bare 0 here (divinus's _i6_isp_chn, waybeam's literal); naming it
 * keeps the coupling to STAR_VPE_CHN visible.
 */
#define STAR_ISP_CHN STAR_VPE_CHN

/*
 * The key MI_ISP_API_CmdLoadBinFile wants alongside the path. Not a
 * checksum of anything -- both references pass this same literal
 * (divinus i6_hal.c:215, waybeam star6e_pipeline.c:295), and the
 * wrapper reads the file itself with fopen, so the value is a
 * protocol constant rather than a property of the binary.
 */
#define STAR_IQ_LOAD_KEY 1234u

/*
 * VPE output ports per channel. divinus's teardown disables ports 0..3
 * (i6_hal.c:365) and waybeam only ever uses 0 and 1, so 4 is the
 * documented-by-use bound. How many of the four actually accept
 * MI_VPE_SetPortMode is a property of the silicon; hal_caps.c's
 * max_fs_channels carries the measured answer.
 *
 * Treat this as an upper bound to size arrays with, not a count of ports
 * that will work. divinus's teardown loop is defensive -- it disables
 * ports it never configured -- so it evidences the ceiling and nothing
 * about how many MI_VPE_SetPortMode accepts here. Anything that wants a
 * port beyond the ones rvd configures must handle not getting one; see
 * hal_enc_register_channel.
 */
#define STAR_VPE_PORT_NUM 4

/*
 * OSD regions the backend will track at once.
 *
 * MI publishes no limit on region handles, and neither reference probes
 * for one: divinus numbers handles from its own overlay slots and
 * waybeam uses a single fixed handle. 16 is chosen to match rvd's
 * largest per-platform region budget in hal_caps.c, so a config that
 * works on Ingenic is not silently truncated here. A region costs a
 * handle and its bitmap, nothing per-slot, so the array is cheap.
 */
#define STAR_OSD_REGION_MAX 16

/*
 * Per-region bookkeeping.
 *
 * Everything here is tracked rather than read back from MI, for two
 * reasons. MI_RGN_GetAttr and MI_RGN_GetDisplayAttr exist, but the
 * geometry raptor cares about (x/y/layer/alpha) is spread across the
 * region attr and the *per-channel* display attr, so a read-back needs
 * both plus a live attach; and rvd sets attributes before the region is
 * attached to anything, when there is no display attr to read.
 *
 * `attached` is the distinction that matters: registered means rvd asked
 * for the region to appear on a group, attached means MI has actually
 * been told, which cannot happen until the VPE port exists. See
 * star_osd_flush_pending.
 */
typedef struct {
    bool used;
    rss_osd_type_t type;

    int x;
    int y;
    int width;
    int height;
    int layer;

    bool global_alpha_en;
    unsigned char fg_alpha;
    unsigned char bg_alpha;
    unsigned int cover_color;

    /* Registered group, or -1. */
    int grp;
    bool attached;
    bool show;

    /* Converted bitmap handed to MI_RGN_SetBitMap. Kept per region so a
     * per-frame update does not allocate, and resized only when the
     * region's geometry changes. */
    void *bmp;
    size_t bmp_size;

    /*
     * Set once the first bitmap has been accepted by MI_RGN_SetBitMap, so
     * that fact can be logged exactly once per region. Without it the
     * only two failure modes an invisible overlay can have -- "attached
     * but never fed" and "fed but not composited" -- look identical from
     * the log, which is precisely the hole the const-alpha bug hid in.
     */
    bool bmp_logged;
} star_osd_region_t;

/*
 * Audio input device.
 *
 * MI's AI "device" is not a numbering convenience like it is elsewhere --
 * the MI_AI reference is explicit that it selects the *physical* input
 * ("Amic/Dmic/I2S RX/Line in"), so the index is a board property. 0 is
 * the onboard analog path, which is what both references use and all the
 * vendor examples pass.
 *
 * rad's `[audio] device` defaults to 1 (an Ingenic index), so hal_audio.c
 * configures this device regardless and warns once if it was asked for a
 * different one; raptor-ssc30kq.conf sets the key to 0 to keep the log
 * quiet.
 */
#define STAR_AUD_DEV 0

/*
 * Tracks this backend supports. MI allows up to I6_AUD_CHN_NUM per
 * device, but raptor's audio config only distinguishes mono from stereo
 * (rss_audio_config_t.chn_count is 1 or 2) and rad only ever reads
 * channel 0, so anything beyond 2 would be state nothing can reach.
 */
#define STAR_AUD_CHN_MAX 2

/*
 * How long a blocking MI_AI_GetFrame waits, in ms.
 *
 * 128 is divinus's value (waybeam uses 50). At the 20 ms capture period
 * this backend configures, either is several periods of slack, and the
 * timeout only decides how promptly a stopped device is noticed -- rad
 * treats a timeout as "try again", so a longer wait costs nothing but
 * latency in the failure case.
 */
#define STAR_AUD_GET_TIMEOUT_MS 128

/*
 * WHICH RAPTOR OP DRIVES WHICH STAGE
 *
 * There are two, and they are reached by two different MI calls:
 *
 *   gain    the analog front end, through MI_AI_SetVqeVolume. Despite the
 *           name that call takes an *index* into a per-device analog-gain
 *           table, not decibels.
 *   volume  the digital trim, through MI_AI_SetChnParam's s16RearGain.
 *
 * That pairing is the way round the Infinity6C backend has it -- gain is
 * the preamp, volume is the level control -- and matching it is the point.
 * This backend had them the other way for a long time, because
 * MI_AI_SetVqeVolume was the only level control it knew about and `volume`
 * was the natural op to hang it on. Once MI_AI_SetChnParam turned out to
 * reach a second, genuinely independent stage, keeping the old pairing
 * would have meant the same two config keys meaning opposite things on two
 * SigmaStar backends in the same tree.
 *
 * The swap is free at rad's defaults, which is what makes it safe to make:
 * gain 25 lands on analog step 17, exactly where volume 80 used to put it,
 * and volume 80 is this map's unity, i.e. 0 dB of digital -- which is what
 * the digital stage was doing when nothing wrote to it. A default install
 * captures at an identical level before and after. Only a hand-tuned
 * `[audio] volume` moves.
 */

/*
 * The analog table, indexed by `gain`.
 *
 * The MI_AI reference spells it out ("the corresponding gain (DB) of
 * s32volumedb under each device") and the columns are arithmetically
 * self-consistent, which is what makes them safe to quote: Amic runs 0..21
 * for -6..+57 dB in 3 dB steps, Line in 0..7 for -6..+15 dB, Dmic 0..4 for
 * 0..+24 dB in 6 dB steps.
 *
 * No longer only a quotation, either. MI_AI_SetChnParam validates this same
 * stage in userspace, per device, and the board's libmi_ai.so bounds it at
 * exactly [0,21] for device 0, [0,7] for device 3 and refuses device 2 --
 * the Amic and Line in columns above, confirmed against the code that
 * enforces them. Only the Dmic 0..4 column remains doc-only, because this
 * backend always configures device 0.
 *
 * Named for the stage rather than for the op, as the Infinity6C backend
 * names its I6C_AUD_IF_GAIN_MAX_*: both references pass dB-shaped numbers
 * to MI_AI_SetVqeVolume -- divinus validates its `[audio] gain` to [-60,30]
 * and waybeam maps 0..100 onto -60..+30 -- so most of their range is off
 * the end of the table, and only their defaults happen to be valid indices.
 * That is the mistake the name is there to stop.
 */
#define STAR_AUD_IF_GAIN_MAX_AMIC 21
#define STAR_AUD_IF_GAIN_MAX_DMIC 4

/* raptor's gain scale, matching the Infinity6C backend and rad's default
 * of 25. Mapped linearly onto the analog table above. */
#define STAR_AUD_GAIN_MAX 31

/*
 * The digital stage: MI_AI_SetChnParam's s16RearGain, a trim in whole dB
 * that nothing else on this API can reach. See i6_aud_chn_para for why
 * front and rear are not interchangeable.
 *
 * The dB bounds are the library's own range check, not a quotation, and
 * they are the same span the Infinity6C backend documents for its DPGA
 * (I6C_AUD_DPGA_MIN_DB / _MAX_DB) -- two independent SoC generations
 * agreeing on the digital stage's range.
 *
 * UNITY is rad's default volume, so a default install applies exactly 0 dB
 * here; below it attenuates toward -60, above it boosts toward +30. Same
 * value and same reasoning as I6C_AUD_VOL_UNITY: mapping volume linearly
 * across the full range would put the shipped default at about +12 dB of
 * digital gain on top of the analog stage, i.e. clipping out of the box for
 * a number nobody chose.
 */
#define STAR_AUD_VOL_UNITY 80
#define STAR_AUD_DPGA_MIN_DB (-60)
#define STAR_AUD_DPGA_MAX_DB 30

/*
 * MI_AI_ERR_BUF_EMPTY -- "audio input buffer is empty", from the MI_AI
 * reference's error-code table. The one MI_AI_GetFrame failure that means
 * "nothing captured yet" rather than "something is wrong", so it is the
 * one hal_audio.c reports as a timeout instead of an error.
 *
 * Quoted from the SSD20X documentation, so treat a *different* code
 * arriving every period on this chip as this constant being wrong rather
 * than as a real fault -- hal_audio.c logs the code it saw, which is what
 * makes that distinguishable.
 */
#define STAR_AUD_ERR_BUF_EMPTY 0xA004200Eu

/*
 * MI_SYS output-port queue for each AI channel, set right after
 * MI_AI_EnableChn. Without it MI_AI_GetFrame returns MI_AI_ERR_NOBUF
 * (0xA004200D) on every call while the device looks healthy -- see the
 * long comment at the call site in hal_audio.c for why these particular
 * numbers and what the three reference sources use.
 */
#define STAR_AUD_PORT_USR_DEPTH 1
#define STAR_AUD_PORT_BUF_DEPTH 16

/*
 * What to fall back to if MI refuses BUF_DEPTH above. The references all use
 * small values (vendor 8, divinus 4, waybeam 2) so a deeper queue is the one
 * thing here with no third-party precedent, and a refused depth fails
 * hal_audio_init outright -- i.e. no audio at all, which is far worse than a
 * shallow queue. 4 is the value this board ran with for the whole bring-up,
 * so it is known to be accepted.
 */
#define STAR_AUD_PORT_BUF_DEPTH_FALLBACK 4

/*
 * "Insufficient audio input buffer" -- the port has no user-side queue. One
 * digit from BUF_EMPTY above and nothing like it in meaning, which is why
 * hal_audio.c names codes in its log instead of printing bare hex.
 */
#define STAR_AUD_ERR_NOBUF 0xA004200Du

/*
 * How many times read_frame will re-establish a channel's output port queue in
 * response to NOBUF before it gives up and just reports the error. Small on
 * purpose: this recovers a port that was lost during bring-up, and must not
 * become an unbounded retry loop running at the capture period.
 */
#define STAR_AUD_NOBUF_RECOVER_MAX 3

/*
 * Consecutive good frames before the NOBUF recovery budget is refilled --
 * 5 seconds at a 20ms period. Long enough that "the port is working again"
 * is a real observation rather than one lucky read between failures.
 */
#define STAR_AUD_NOBUF_RECOVER_REARM_FRAMES 250

/*
 * Default output-port buffer-queue depth, and how long a blocking
 * frame fetch waits.
 *
 * 3 is the vendor's own number: the MI_SYS_ChnOutputPortGetBuf sample
 * in SigmaStar's MI_SYS reference (ref/sigmastar-docs, MI SYS API 2.25)
 * runs SetChnOutputPortDepth(&port, 2, 3), then releases with
 * (&port, 0, 3) -- user depth to zero, queue depth left alone. That
 * release pattern is exactly what rvd's fs_set_frame_depth(chn, 0)
 * means, so the two models line up without inventing anything.
 */
#define STAR_VPE_QUEUE_DEPTH 3
#define STAR_FRAME_TIMEOUT_MS 2000

/*
 * Queue depth for a snapshot port -- a VPE port this backend allocates
 * for itself to feed a JPEG encoder channel. See star_fs_clone_port.
 *
 * Shallower than STAR_VPE_QUEUE_DEPTH on purpose. The queue exists to
 * absorb bursts, and there are none here: the port is bound with a
 * destination rate of the JPEG stream's fps (1 by default) against a
 * source running at the sensor's, so MI drops 29 of every 30 frames at
 * the bind and never has more than one frame in flight. Two rather than
 * one because these buffers are full-resolution -- the snapshot matches
 * its paired video stream -- and a 2560x1440 NV12 frame is 5.5 MB, so
 * every slot is worth arguing about on a 64 MB board.
 */
#define STAR_VPE_SNAP_QUEUE_DEPTH 2

/*
 * VENC channels have one input port, always 0 -- divinus keeps it in a
 * variable (_i6_venc_port) only because it shares this file across four
 * SoC families.
 */
#define STAR_VENC_PORT 0

/*
 * How many i6_venc_pack the encoder may return for one frame without
 * heap traffic in the streaming path.
 *
 * The vendor sample sizes this array from MI_VENC_Query's u32CurPacks
 * and mallocs it per frame; divinus keeps a stack array of 8 and only
 * mallocs above that (i6_hal.c:854). H.264/H.265 hand back one pack per
 * frame here -- multiple NAL units ride *inside* a pack, described by
 * its packetInfo -- so this is generous in practice.
 *
 * 16 rather than divinus's 8 because that is rvd's own ceiling: its
 * encoder thread copies at most 16 NALs into the ring and warns when it
 * truncates (rvd_frame_loop.c:256). Matching it means the HAL never
 * becomes the tighter limit.
 *
 * MI_VENC_GetStream is handed a pack array the caller sizes, and
 * whether it respects that size or writes u32CurPacks entries regardless
 * is not documented -- divinus mallocs precisely to avoid finding out.
 * So does star_enc_packs: above this bound the array is heap-allocated
 * to the full count, and only the NAL *reporting* is capped.
 */
#define STAR_VENC_MAX_PACKS 16

/*
 * Per-VPE-port state. One of these is a raptor "framesource channel":
 * raptor fs channel N maps to VPE port N with no indirection, which is
 * also how divinus does it (i6_channel_create(index) configures port
 * `index`, and i6_channel_bind binds that same index to VENC channel
 * `index`). Keeping the identity mapping means 2d's VENC bind needs no
 * lookup table.
 */
typedef struct {
    bool configured; /* MI_VPE_SetPortMode has succeeded */
    bool enabled;    /* MI_VPE_EnablePort has succeeded */

    unsigned short width;
    unsigned short height;
    i6_common_pixfmt pixFmt;

    /*
     * Frame rate is not a VPE port attribute -- MI applies rate control
     * when binding (MI_SYS_BindChnPort2 takes srcFps/dstFps). Kept here
     * so 2d's VPE->VENC bind can use the rate the caller asked for.
     */
    unsigned int fps_num;
    unsigned int fps_den;

    /* Both arguments of MI_SYS_SetChnOutputPortDepth, tracked because MI
     * offers no getter for either. */
    unsigned int user_depth;
    unsigned int queue_depth;

    /* MI_SYS_GetFd wakeup descriptor, opened on the first frame fetch
     * and closed when the port is disabled. -1 when not open. */
    int fd;

    /* At most one outstanding frame per port -- see hal_fs_get_frame. */
    bool frame_held;
    int frame_handle;
    i6_sys_bufinfo frame;
} star_vpe_port_t;

/*
 * Per-VENC-channel state.
 *
 * raptor encoder channel N is MI VENC channel N. rvd binds framesource
 * channel N to encoder channel N by default but does not require it, so
 * `src_port` records which VPE port was actually bound rather than
 * assuming the identity -- unbind has to name the same pair.
 */
typedef struct {
    bool created;   /* MI_VENC_CreateChn has succeeded */
    bool receiving; /* MI_VENC_StartRecvPic has succeeded */
    bool bound;     /* a VPE port is bound to this channel */

    /*
     * True when this backend configured src_port itself rather than
     * rvd configuring it through fs_create_channel -- the snapshot port
     * of a JPEG channel. It is the flag that says who has to release the
     * port again: rvd never learned this port exists, so nothing will
     * call fs_destroy_channel for it. See hal_enc_register_channel.
     */
    bool owns_port;

    /*
     * MI_VENC_GetChnDevid's answer, cached at create time. The bind
     * needs it and it cannot be read back once the channel is
     * destroyed, which is the order teardown runs in.
     */
    unsigned int device;
    int src_port; /* bound VPE port, -1 when unbound */

    rss_codec_t codec;
    unsigned short width;
    unsigned short height;
    unsigned int fps_num;
    unsigned int fps_den;

    /*
     * Destination rate for this channel's bind, 0 meaning "the rate the
     * VPE port was configured for". MI paces a channel in the bind rather
     * than in the encoder, so fps_num above buys only a bitrate budget;
     * this is the rate frames actually arrive at. Recorded separately
     * because a later rebind would otherwise revert to the port's rate.
     */
    unsigned int bind_fps;
    unsigned int gop;
    rss_rc_mode_t rc_mode;
    unsigned int bitrate;     /* bps, as the caller expressed it */
    unsigned int max_bitrate; /* bps */

    /*
     * QP bounds as the caller gave them, -1 meaning "SDK default".
     * Kept because MI has no per-knob setter: changing the bitrate
     * rewrites the whole rate struct, and without these a set_bitrate
     * would quietly reset the QP bounds the channel was created with.
     */
    int16_t init_qp;
    int16_t min_qp;
    int16_t max_qp;

    /* MI_VENC_GetFd descriptor, opened on first poll. -1 when closed. */
    int fd;

    /*
     * One outstanding stream per channel, mirroring the framesource
     * ports. The pack array backs strm.packet, and the NAL array backs
     * the rss_frame_t handed out by enc_get_frame -- both must outlive
     * the call, and neither may be reused until enc_release_frame.
     */
    bool frame_held;
    i6_venc_strm strm;
    i6_venc_pack packs[STAR_VENC_MAX_PACKS];
    rss_nal_unit_t nals[STAR_VENC_MAX_PACKS];

    /* Oversized pack array for the rare frame above STAR_VENC_MAX_PACKS.
     * Allocated on demand, grown never shrunk, freed at destroy. */
    i6_venc_pack *heap_packs;
    unsigned int heap_count;
} star_venc_chn_t;

typedef struct {
    i6_sys_impl sys;
    i6_snr_impl snr;
    i6_vif_impl vif;
    i6_vpe_impl vpe;
    i6_venc_impl venc;
    i6_isp_impl isp;

    /* Sensor descriptors, read back after MI_SNR_Enable (see hal_init) */
    i6_snr_pad pad;
    i6_snr_plane plane;

    /* Selected sensor mode */
    i6_snr_res res;
    unsigned char res_index;

    /* Sensor frame rate, as programmed. Used for the VIF->VPE bind and
     * as the source rate for 2d's VPE->VENC bind.
     *
     * fps_milli carries the same rate to three decimal places, which is
     * the other unit MI_SNR_SetFps accepts. It exists so a fractional rate
     * can be re-issued exactly -- the AE fit re-programs the sensor, and
     * rounding there would move a 29.97 request to 30. */
    unsigned int fps;

    /*
     * MI's own build stamp as YYYYMMDDhhmmss, read once from
     * MI_SYS_GetVersion, or 0 when it could not be read or parsed. This is what
     * picks the VPE struct layouts -- see star_vpe_modern.
     */
    unsigned long long mi_build;
    unsigned int fps_milli;

    /*
     * ISP state.
     *
     * iq_file is the tuning binary actually loaded, empty when none was.
     * Kept because MI offers no way to ask what is loaded, and because a
     * *reload* is not free: the vendor AE reinitialises the sensor
     * shutter register from the binary's own defaults, which on a
     * running pipeline shows up as a framerate change. waybeam skips
     * redundant loads for exactly that reason
     * (star6e_pipeline.c:2073-2077).
     *
     * gray exists because MI has no getter for it:
     * MI_ISP_IQ_GetColorToGray reads the IQ struct rather than a
     * day/night mode. Orientation needs no such shadow -- it lives in the
     * VPE channel param, which reads back.
     */
    char iq_file[128];
    bool isp_loaded;

    /*
     * Set once the ISP has answered and the tuning binary has had its
     * one attempt. Before that the ISP refuses every query -- it is
     * served by the VPE channel, which does not run until an output port
     * is enabled -- so the control ops queue their values instead of
     * failing, and this is the flag that says which of the two applies.
     * See the comment above star_isp_bringup.
     */
    bool isp_tuned;

    /*
     * Set by the first encoded frame to reach the application, and cleared
     * whenever the VPE channel stops. It gates the tuning load: CUS3A's AE
     * init reads its own iqfile on a frame interrupt and would read back
     * over anything loaded before that. See star_isp_note_frame.
     */
    bool isp_frame_seen;

    /*
     * The AE's own gain ceilings, snapshotted from whichever tuning is in
     * effect before any config knob overwrites them. These are the
     * calibrated limits and MI treats them as authoritative: a ceiling
     * above them does not stick. waybeam found the same thing the hard way
     * -- "isp.gainMax above the bin ceiling never stuck (found with
     * gainMax=32000 vs bin 8192)", maruko_cus3a.c -- so they are the range
     * a requested ceiling has to be judged against rather than a hint.
     * Zero for "the AE never published them".
     */
    unsigned int bin_max_sensor_gain;

    /* How many times the tuning binary has been reloaded after finding the
     * ISP back on its defaults. Bounded: a reload that does not stick must
     * not become a loop. */
    int iq_reloads;

    bool gray;

    star_vpe_port_t port[STAR_VPE_PORT_NUM];
    star_venc_chn_t enc[I6_VENC_CHN_NUM];

    /*
     * OSD state -- src/star/hal_osd.c.
     *
     * MI_RGN has no notion of a region *group*. raptor's group is the
     * encoder channel whose picture the region should appear on, so a
     * group here is just a flag plus the set of regions registered to
     * it, and the real MI operation is attaching the region to the VPE
     * output port feeding that encoder.
     *
     * osd_src_port exists because rvd's bind chain is
     * FS -> OSD -> ENC while MI's data path is VPE port -> VENC with no
     * stage in between. hal_bind records the FS port when it sees
     * FS -> OSD and performs the real bind when it sees OSD -> ENC, so
     * the OSD cell collapses instead of being rejected.
     */
    i6_rgn_impl rgn;
    bool rgn_loaded;
    bool rgn_inited;

    /* Which pixel format MI_RGN_Create actually accepted -- see
     * star_osd_probe_pixfmt. rgn_fmt_known distinguishes "not probed
     * yet" from a successfully probed format. */
    i6_rgn_pixfmt rgn_fmt;
    bool rgn_fmt_known;

    bool osd_grp[I6_VENC_CHN_NUM];
    int osd_src_port[I6_VENC_CHN_NUM];
    star_osd_region_t osd[STAR_OSD_REGION_MAX];

    /*
     * Audio capture state -- src/star/hal_audio.c.
     *
     * Lives in the same struct as the video state even though the two
     * never coexist in one process, because star_state_t is what
     * rss_hal_ctx_t->platform points at and both archives compile
     * hal_common.c. The audio archive simply leaves the video half zero.
     *
     * aud_owns_sys records that audio_init brought MI_SYS up itself.
     * rad calls rss_hal_create and then audio_init directly -- it never
     * calls the init op -- so on this backend audio_init has to do the
     * MI_SYS work that hal_init would otherwise have done, and teardown
     * has to know whether it is entitled to undo it.
     *
     * The volume is tracked rather than read back because MI's getter
     * (MI_AI_GetVqeVolume) reports the *VQE* volume, and the VQE
     * algorithm libraries are absent on this platform.
     */
    i6_aud_impl aud;
    bool aud_loaded;
    bool aud_owns_sys;
    bool aud_dev_enabled;
    bool aud_chn_enabled[STAR_AUD_CHN_MAX];
    int aud_dev;
    unsigned int aud_chn_count;
    int aud_rate;
    int aud_volume;
    int aud_gain;

    /*
     * The analog step last written through MI_AI_SetVqeVolume, or -1 before
     * anything has set the gain.
     *
     * Tracked because MI_AI_SetChnParam carries BOTH stages in one struct
     * and applies both: hal_audio_set_volume has to supply a front gain
     * even though it only means to change the rear one, and supplying a
     * remembered value beats reading one back on every call. The -1 case is
     * the startup ordering -- rad sets volume before gain, so the first
     * set_volume runs before there is anything to remember, and asks the
     * driver once rather than writing a zero over whatever it is using.
     */
    int aud_front_idx;
    rss_audio_input_t aud_input;

    /*
     * PCM channels per captured frame -- 1 for mono, 2 for interleaved
     * stereo. Distinct from aud_chn_count, which counts MI *channels* and
     * is 1 either way: MI carries a stereo pair inside one channel's frame
     * rather than across two channels. Kept only so the bring-up log can
     * say what was configured.
     */
    unsigned int aud_pcm_chn;

    /* One outstanding frame per channel. MI hands out a descriptor that
     * must come back to MI_AI_ReleaseFrame unchanged, and
     * rss_audio_frame_t has nowhere to store 200-odd bytes, so the
     * descriptor stays here and _priv points at it. */
    i6_aud_frm aud_frame[STAR_AUD_CHN_MAX];
    bool aud_frame_held[STAR_AUD_CHN_MAX];

    /* Last MI_AI_GetFrame failure already reported, so a persistent fault
     * is named once instead of every capture period. */
    int aud_last_err;
    bool aud_dev_warned;

    /* NOBUF recovery attempts spent on this channel. Bounds the re-apply in
     * hal_audio_read_frame, which is destructive -- it flushes the port queue.
     * Refilled only after a sustained run of good frames (aud_ok_run), because
     * refilling on a single frame let a fault that alternates with successful
     * reads re-apply at the capture period rate. */
    int aud_nobuf_recover[STAR_AUD_CHN_MAX];
    int aud_ok_run[STAR_AUD_CHN_MAX];

    /*
     * The loaded sensor driver's own name, read from /proc/modules during
     * bring-up. The tuning binary is named after the driver module, so this
     * is what resolves it; MI's plane.sensName is the same sensor spelled
     * MI's way and only exists after Enable.
     */
    char sensor_name[64];

    /* Unwind flags -- each set only once its step has succeeded, so
     * teardown undoes exactly what was done and no more. */
    bool sys_inited;
    bool snr_enabled;
    bool vif_dev_enabled;
    bool vif_port_enabled;
    /*
     * What MI_SNR_SetOrien was told at bring-up, which is the only time it is
     * told anything. MI_SNR_GetOrien cannot be used to check: the vendor
     * driver answers it from its static default table rather than the live
     * value, so it reports unmirrored however the image actually looks.
     */
    int snr_mirror;
    int snr_flip;

    bool vpe_chn_created;
    bool vpe_chn_started;
    bool vif_vpe_bound;
} star_state_t;

/*
 * The 3DNR level the VPE channel is created with, and keeps.
 *
 * Not a strength knob and not raptor's temper -- despite the name, the level
 * selects the *bit depth of the 3DNR reference frame*. mhal's
 * Camera3DNR_GetConfigByLevel maps level 1 to Isp3DNRCompressLevel_2 (8-bit)
 * and level 2 to Isp3DNRCompressLevel_0 (12-bit), and maps every other value,
 * 0 and 3..7 alike, to "engine disabled". So of the eight the enum offers,
 * six mean off and the remaining two differ only in precision. Measured on
 * .229 at 2560x1440: level 1 allocates DNR_INFO0 at 0x384000 (2560 B/row),
 * level 2 at 0x5a0000 (4096 B/row).
 *
 * 2 because that is what every clean reference on this silicon runs: majestic
 * on this board and sensor, and the SDK's own cus3a and ldc demos. The extra
 * ~2.1MB buys the temporal filter a finer history to difference against.
 *
 * Fixed rather than configurable because no other value is a sensible thing
 * to offer: below it is off-by-another-name, and above it MI clamps to 3,
 * which is also off. That is why this SoC publishes no temper op at all --
 * temporal denoise strength lives in the tuning binary's NR3D block, and
 * reaching it from here would cost the tuning's own per-gain curve. See the
 * OP COVERAGE note in hal_isp.c.
 *
 * Level 0 is worth naming as the trap it is. It disables the DNR engine, and
 * on this SoC that engine is what mirror and flip run on -- a channel
 * carrying a flip and asking for level 0 stalls the ISP outright, with no
 * frame-done and a CMDQ timeout. Orientation lives on the sensor now, so
 * nothing here can reach that state, and this constant is the other half of
 * why.
 */
#define STAR_VPE_NR3D_LEVEL 2

/*
 * MI_VPE_SetChannelParam's argument, which is not the same struct on both
 * families this backend serves -- see the ABI note in i6_vpe.h. Wrong choice
 * is silent, so it is a typedef rather than a cast at the call site: the
 * struct that is filled and the struct that is passed cannot drift apart.
 */
/*
 * Which MI_VPE_ChannelPara_t and MI_VPE_ChannelAttr_t this library wants.
 *
 * These two structs grew across MI releases, and the size tracks the *release*,
 * not the chip. Measured from the memcpy length in each libmi_vpe.so:
 *
 *   build_time            MI_VPE_ChannelPara_t   MI_VPE_ChannelAttr_t
 *   2019-10 .. 2021-09            28                52 -> 64 -> 108
 *   2022-06 onward               100                     192
 *
 * The 2019 Infinity6E libraries take the same 28 and 52 that the early
 * Infinity6B0 ones do, so "6E is long, 6B0 is short" was never a property of
 * the silicon -- it is only true because OpenIPC happens to ship a 2022 drop
 * for one and a 2020 drop for the other. Keying on the family gets the right
 * answer today and the wrong one the moment either package is updated.
 *
 * MI_SYS_GetVersion reports the *driver's* build, not the library's -- the two
 * differ by seconds on a board, which is how you can tell -- and the driver is
 * what defines the ioctl ABI. So that is the key.
 *
 * Falling back to the family when the stamp cannot be read keeps today's
 * behaviour on a library too old to carry one: the pre-2020 drops have no
 * version string at all.
 */
#define STAR_MI_BUILD_LONG_PARA 20220101000000ull

static inline bool star_vpe_modern(const star_state_t *st)
{
    if (st->mi_build)
        return st->mi_build >= STAR_MI_BUILD_LONG_PARA;
#if defined(PLATFORM_INFINITY6B0)
    return false;
#else
    return true;
#endif
}

/*
 * MI_VPE_ChannelAttr_t splits the same way and for the same reason, which is
 * only visible once something past the common prefix is actually used. The two
 * agree up to tIspInitPara; 6E then carries an 84-byte MI_VPE_LdcInitPara_t
 * where 6B0 goes straight to bEnLdc and u32ChnPortMode. Infinity6B0's
 * libmi_vpe.so memsets 112 and memcpys 108 out of MI_VPE_CreateChannel, which
 * is sizeof(i6_vpe_chn) exactly; the 6E form is 192 and puts u32ChnPortMode at
 * +188, eighty bytes past anything 6B0 reads.
 *
 * Handing 6B0 the 6E form was survivable only because every field past the
 * prefix was zero, so the driver read zeros where it expected bEnLdc and
 * u32ChnPortMode and applied its defaults. It became a bug the moment
 * u32ChnPortMode needed a value: the write landed outside the struct the
 * driver copies, and MI_VPE_CreateChannel succeeded regardless.
 */
#if defined(PLATFORM_INFINITY6B0)
typedef i6_vpe_chn star_vpe_chn;
#else
typedef i6e_vpe_chn star_vpe_chn;
#endif

/*
 * Build MI_VPE_ChannelPara_t from raptor's own record of it.
 *
 * Fresh every time, never read-modify-write. The struct's first member is
 * MI_VPE_PqParam_t -- chroma and luma spatial/temporal NR strengths, six edge
 * gains and a contrast value -- and it is not ours to carry: MI marks it
 * "only dvr use" and this pipeline is REALTIME. No vendor reference ever
 * round-trips this struct; every one of them builds it from zero and fills
 * the fields it means, which is what this does.
 *
 * Two layouts, because the struct grew between MI releases: the 2022 libraries
 * memcpy 100 bytes out of it and find level3DNR at +92, the 2020 and earlier
 * ones copy 28 and find it at +20. Handing a short-form library the long form
 * left the 3DNR engine reading a zero out of bytes that were never meant for
 * it, so it never ran at all -- and silently, because the ioctl still
 * succeeds. star_vpe_modern picks; i6_vpe.h has the disassembly.
 *
 * The round trip it replaces read the whole struct back and wrote it out
 * again, so whatever MI_VPE_GetChannelParam had declined to populate went to
 * the driver as the zeros the caller's memset left -- the same bytes, but
 * arrived at by accident and only while Get kept quiet about those fields.
 * Rebuilding says it deliberately, and drops a read from the write path.
 */
/*
 * mirror and flip stay zero, and the sensor carries orientation instead --
 * star_sensor_bringup says why. They are left to the memset rather than
 * assigned, so that a reader looking for where this backend sets them finds
 * nothing, which is the point.
 *
 * A macro rather than a function because the two layouts are different types
 * with the same field names, and the alternative is writing these four lines
 * twice and letting them drift.
 */
#define STAR_VPE_FILL_PARA(para)                                                                   \
    do {                                                                                           \
        memset((para), 0, sizeof(*(para)));                                                        \
        (para)->hdr = I6_HDR_OFF;                                                                  \
        (para)->level3DNR = STAR_VPE_NR3D_LEVEL;                                                   \
        (para)->lensAdjOn = 0;                                                                     \
    } while (0)

static inline void star_vpe_fill_param_long(const star_state_t *st, i6e_vpe_para *para)
{
    (void)st;
    STAR_VPE_FILL_PARA(para);
}

static inline void star_vpe_fill_param_short(const star_state_t *st, i6_vpe_para *para)
{
    (void)st;
    STAR_VPE_FILL_PARA(para);
}

static inline star_state_t *star_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (star_state_t *)c->platform : NULL;
}

/*
 * star_vpe_pixfmt -- the pixel format the VPE *channel* consumes.
 *
 * Bayer sensors need this derived rather than read from the plane's own
 * pixFmt field; see the long comment on star_vif_pixfmt in
 * hal_common.c for the hardware evidence.
 */
i6_common_pixfmt star_vif_pixfmt(const i6_snr_plane *plane);

/* Framesource ops -- src/star/hal_framesource.c */
int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_destroy_channel(void *ctx, int chn);
int hal_fs_enable_channel(void *ctx, int chn);
int hal_fs_disable_channel(void *ctx, int chn);
int hal_fs_set_fifo(void *ctx, int chn, int depth);
int hal_fs_get_fifo(void *ctx, int chn, int *depth);
int hal_fs_set_frame_depth(void *ctx, int chn, int depth);
int hal_fs_get_frame_depth(void *ctx, int chn, int *depth);
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
int hal_fs_release_frame(void *ctx, int chn, void *frame_data);

/* Called from star_teardown so a port's frame, fd and enable state do
 * not outlive the VPE channel. */
void star_fs_release_all(star_state_t *st);

/*
 * Bring up a second VPE output port carrying the same picture as an
 * existing one, and tear it down again. Used for JPEG snapshot channels,
 * which rvd never configures a framesource for -- see
 * hal_enc_register_channel. Not part of the HAL vtable: nothing outside
 * this backend knows these ports exist.
 */
int star_fs_clone_port(star_state_t *st, int src, int dst);
int star_fs_enable_port(star_state_t *st, int port);
void star_fs_release_port(star_state_t *st, int port);

/* Encoder ops -- src/star/hal_encoder.c */
int hal_enc_create_group(void *ctx, int grp);
int hal_enc_destroy_group(void *ctx, int grp);
int hal_enc_create_channel(void *ctx, int chn, const rss_video_config_t *cfg);
int hal_enc_destroy_channel(void *ctx, int chn);
int hal_enc_register_channel(void *ctx, int grp, int chn);
int hal_enc_unregister_channel(void *ctx, int chn);
int hal_enc_start(void *ctx, int chn);
int hal_enc_stop(void *ctx, int chn);
int hal_enc_poll(void *ctx, int chn, uint32_t timeout_ms);
int hal_enc_get_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_release_frame(void *ctx, int chn, rss_frame_t *frame);
int hal_enc_request_idr(void *ctx, int chn);
int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den);
int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg);
int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
int hal_enc_get_gop_attr(void *ctx, int chn, uint32_t *gop_length);
int hal_enc_set_gop_attr(void *ctx, int chn, uint32_t gop_length);
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate);
int hal_enc_query(void *ctx, int chn, bool *busy);
int hal_enc_get_fd(void *ctx, int chn);

/*
 * The VPE-port half of a bind, shared with hal_common.c's bind/unbind:
 * MI binds VPE port -> VENC channel, so the encoder side owns the
 * device id and the bound-port bookkeeping.
 */
int star_enc_bind_port(star_state_t *st, int port, int chn);
int star_enc_unbind_port(star_state_t *st, int port, int chn);

/* Called from star_teardown, before the VPE channel goes away. */
void star_enc_release_all(star_state_t *st);

/* ================================================================
 * ISP -- src/star/hal_isp.c
 * ================================================================ */

/*
 * Bind libmi_isp and work out which tuning binary to load.
 *
 * Called from hal_init. Touches no MI call at all, because at that point
 * the ISP cannot answer one: it is served by the VPE channel, and the
 * VPE channel does not run until an output port is enabled. The actual
 * load is therefore star_isp_tune_when_ready's job -- the long comment
 * above star_isp_bringup explains what this cost on the first board run.
 *
 * cfg supplies the optional iq_file override and the sensor name used to
 * derive the default path.
 */
void star_isp_bringup(star_state_t *st, const rss_sensor_config_t *cfg);

/*
 * Load the tuning binary and flush any queued control values, once the
 * ISP is answering *and* a frame has been delivered. Idempotent, and a
 * no-op until both hold, so the enable and start paths can both call it
 * and whichever finds the conditions met wins. verbose=false for early
 * opportunistic attempts, true for the one whose failure is worth a
 * warning.
 */
void star_isp_tune_when_ready(star_state_t *st, bool verbose);

/*
 * Tell the ISP a frame reached the application, which is what releases the
 * tuning load. Called per frame from the encoder checkout path; free after
 * the first. See the definition for why nothing earlier is safe.
 */
void star_isp_note_frame(star_state_t *st);

/*
 * Mark the tuning as lost, so the next star_isp_tune_when_ready re-applies
 * it. Called when the last VPE output port is disabled, because stopping
 * the VPE channel is what throws the tuning away -- see the definition.
 */
void star_isp_untune(star_state_t *st);

/* Release the ISP libraries. Called from star_teardown. */
void star_isp_teardown(star_state_t *st);

/*
 * There is no shutter ceiling on this backend. The AE keeps whatever its
 * tuning binary calibrated, and a dim scene is allowed to run the sensor
 * slower than the requested rate to buy the light. See the block comment
 * where star_isp_cap_exposure used to be, in hal_isp.c, for the trade and
 * for what to read before putting it back.
 */

/* ISP ops. Only those MI can honour are defined; see the OP COVERAGE
 * comment in hal_isp.c for what is deliberately absent and why. */
int hal_isp_set_brightness(void *ctx, int val);
int hal_isp_set_contrast(void *ctx, int val);
int hal_isp_set_saturation(void *ctx, int val);
int hal_isp_set_sharpness(void *ctx, int val);
int hal_isp_set_ae_comp(void *ctx, int val);
int hal_isp_set_drc_strength(void *ctx, int val);
int hal_isp_get_drc_strength(void *ctx, int *val);
int hal_isp_set_temper_strength(void *ctx, int val);
int hal_isp_get_temper_strength(void *ctx, int *val);
int hal_isp_set_defog(void *ctx, int enable);
int hal_isp_set_antiflicker(void *ctx, rss_antiflicker_t mode);
int hal_isp_set_running_mode(void *ctx, rss_isp_mode_t mode);
int hal_isp_set_hflip(void *ctx, int enable);
int hal_isp_set_vflip(void *ctx, int enable);
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);

int hal_isp_get_brightness(void *ctx, int *val);
int hal_isp_get_contrast(void *ctx, int *val);
int hal_isp_get_saturation(void *ctx, int *val);
int hal_isp_get_sharpness(void *ctx, int *val);
int hal_isp_get_knob_caps(void *ctx, const char *name, rss_isp_knob_t *caps);
int hal_isp_get_ae_comp(void *ctx, int *val);
int hal_isp_get_antiflicker(void *ctx, rss_antiflicker_t *mode);
int hal_isp_get_running_mode(void *ctx, rss_isp_mode_t *mode);
int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip);
int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);

/*
 * Audio capture ops -- src/star/hal_audio.c.
 *
 * This is the whole of it. Everything else in the audio half of
 * rss_hal_ops_t stays NULL, which RSS_HAL_CALL already turns into
 * RSS_ERR_NOTSUP; hal_audio.c's OP COVERAGE comment says why for each.
 */
int hal_audio_init(void *ctx, const rss_audio_config_t *cfg);
int hal_audio_deinit(void *ctx);
int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block);
int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame);
int hal_audio_set_volume(void *ctx, int dev, int chn, int vol);
int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol);
int hal_audio_set_gain(void *ctx, int dev, int chn, int gain);
int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain);
int hal_audio_set_mute(void *ctx, int dev, int chn, int mute);

/*
 * OSD ops -- src/star/hal_osd.c.
 *
 * Twelve ops, which is what rvd calls; hal_osd.c's OP COVERAGE comment
 * names the five it leaves NULL and why.
 */
int hal_osd_set_pool_size(void *ctx, uint32_t bytes);
int hal_osd_create_group(void *ctx, int grp);
int hal_osd_destroy_group(void *ctx, int grp);
int hal_osd_start(void *ctx, int grp);
int hal_osd_stop(void *ctx, int grp);
int hal_osd_create_region(void *ctx, int *handle, const rss_osd_region_t *attr);
int hal_osd_destroy_region(void *ctx, int handle);
int hal_osd_register_region(void *ctx, int handle, int grp);
int hal_osd_unregister_region(void *ctx, int handle, int grp);
int hal_osd_set_region_attr(void *ctx, int handle, const rss_osd_region_t *attr);
int hal_osd_update_region_data(void *ctx, int handle, const uint8_t *data);
int hal_osd_show_region(void *ctx, int handle, int grp, int show, int layer);

/*
 * Attach any region registered to this encoder channel, called once the
 * VPE port -> VENC bind exists. rvd registers regions before it binds,
 * so this is where startup overlays actually reach MI.
 */
void star_osd_flush_pending(star_state_t *st, int chn);
void star_osd_release_all(star_state_t *st);

#endif /* STAR_STATE_H */
