/*
 * infinity6c/infinity6c_state.h -- shared backend state for the SigmaStar MI 3.0 HAL
 *
 * Counterpart to star/star_state.h. Separate for the same reason that header
 * gives: it cannot live in hal_internal.h, because the ABI headers include
 * that one for HAL_LOG_* and RSS_ERR_* and must not be included back.
 *
 * The datapath is
 *
 *   SNR -> VIF -> ISP -> SCL -> VENC
 *
 * where MI 2.x is VIF -> VPE -> VENC with the ISP folded into VPE. VIF gains a
 * group above the device, the ISP becomes a stage with its own device, channel
 * and ports, SCL holds the scaling role, and VENC gains a device above the
 * channel -- one that selects the codec engine rather than being a topology
 * index.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INFINITY6C_STATE_H
#define INFINITY6C_STATE_H

#include "hal_internal.h"

#include "i6c_isp_load.h"
#include "i6c_rgn_load.h"
#include "i6c_scl_load.h"
#include "i6c_snr_load.h"
#include "i6c_sys_load.h"
#include "i6c_venc_load.h"
#include "i6c_vif_load.h"

/* ================================================================
 * FIXED TOPOLOGY
 *
 * One sensor, so every device and group index is 0. Named rather than
 * written as literals because MI 3.0 threads more of them through each
 * call than MI 2.x does, and at a call site a bare 0 does not say
 * whether it is a SoC, a device, a group, a channel or a port.
 * ================================================================ */

/*
 * The die MI_SYS and MI_RGN calls are addressed to. Zero on a single-die
 * camera, which is what the vendor's own reference passes. It leads this list
 * because it is the index MI 2.x has no equivalent of, and it is named rather
 * than written as a bare 0 at each call site because a literal there reads
 * like a device or a channel, and it is neither.
 */
#define I6C_SOC_ID 0

#define I6C_SNR_PAD 0
#define I6C_VIF_GRP 0
#define I6C_VIF_DEV 0
#define I6C_VIF_PORT 0
#define I6C_ISP_DEV 0
#define I6C_ISP_CHN 0
#define I6C_ISP_PORT 0

/*
 * The key MI_ISP_ApiCmdLoadBinFile validates a per-sensor API bin against. A
 * fixed protocol value, not per-file: the same 1234 loads every sensor's bin
 * across this ecosystem's tuning files (OpenIPC/thingino), as the i6e backend's
 * STAR_IQ_LOAD_KEY does.
 */
#define I6C_ISP_IQ_LOAD_KEY 1234u
#define I6C_SCL_DEV 0
#define I6C_SCL_CHN 0

/*
 * Only MI_SYS and MI_RGN take the SoC id as a distinct leading argument. VIF,
 * SNR, ISP and SCL pack it into the high halfword of the device or pad index
 * instead, and the wrapper shifts it back out -- MI_ISP_EnableOutputPort opens
 * with `lsr #16` on its first argument.
 *
 * On a single-die part both halves are zero, so passing a bare index happens to
 * work and the distinction is invisible. It is composed explicitly anyway: the
 * cost is nothing and the alternative is a second die's worth of debugging for
 * whoever meets one.
 */
#define I6C_DEV_ID(dev) (((unsigned int)I6C_SOC_ID << 16) | (unsigned int)(dev))

/*
 * Streams. Four rather than the twelve VENC channels MI allows, because each
 * one is an SCL output port and the scaler has four -- so four is the real
 * ceiling on simultaneous streams, whatever VENC would accept.
 */
#define I6C_MAX_CHN 4

/*
 * OSD regions across all streams. MI_RGN handles are global -- a region is
 * created once and attached to whichever VENC channel shows it -- so this is a
 * flat pool, not per channel. Sixteen is what the i6e backend carries and more
 * than rvd asks for (a timestamp plus a handful of user elements per stream).
 */
#define I6C_OSD_REGION_MAX 16

/*
 * Packs per frame handled without allocating. One frame arrives as several
 * packs -- roughly one per NAL -- and this covers a keyframe with its parameter
 * sets. Beyond it the pack array grows on the heap; nals[] does not, matching
 * rvd's own ceiling.
 */
#define I6C_VENC_MAX_PACKS 8

/*
 * Codec engines a channel can live on: one H.26x, one MJPEG. Not a channel count
 * -- the channel index space is shared between them -- but the number of ring
 * pools, since a pool belongs to a device.
 */
#define I6C_VENC_DEV_SLOTS 2

/*
 * Elements in a vendor array, for bounding a loop by the declaration rather than
 * by a literal that matches it today. The packs-within-a-pack array is the one
 * that matters: its length is the vendor's to change, and the loop over it is
 * driven by a count the vendor also supplies.
 */
#define I6C_ARRAY_LEN(a) (unsigned int)(sizeof(a) / sizeof((a)[0]))

/* ================================================================
 * PER-CHANNEL STATE
 * ================================================================ */

/*
 * A framesource channel is an SCL output port. It carries no MI object of its
 * own -- the port exists as soon as the channel it hangs off is started -- so
 * this is the geometry the port was configured with plus whether it is running.
 */
typedef struct {
    bool configured;
    bool enabled;

    /*
     * A shadow channel carries no SCL output port. Only one SCL port feeds the
     * encoder ring; a second video stream is a VENC main->sub cascade off that
     * port, not a port of its own (a second SCL port cannot ring an H.26x
     * channel). rvd still creates a framesource for the sub, so its channel is
     * recorded here as a shadow -- configured, never a real port to apply or
     * enable. See i6c_bind_scl_to_venc.
     */
    bool shadow;

    unsigned short width;
    unsigned short height;
    i6c_common_pixfmt pixfmt;
} infinity6c_fs_chn_t;

typedef struct {
    bool created;
    bool receiving;
    bool bound;

    /*
     * Whether this channel holds the SCL output ring. The first H.26x channel on
     * the codec engine takes it (SCL port -> VENC main, HW_RING); a later H.26x
     * channel is a sub that cascades off the main in VENC rather than binding SCL
     * again, and JPEG is frame-based off its own port. See i6c_bind_scl_to_venc.
     */
    bool uses_ring;

    /*
     * A sub H.26x channel fed by a VENC main->sub HW_RING cascade rather than by
     * SCL directly, with cascade_src the main channel it hangs off. The VENC
     * hardware reduces the main's frame to this channel's (smaller) size, so a sub
     * needs no SCL port and no input-source config of its own.
     */
    bool cascade;
    int cascade_src;

    /*
     * Set when this channel brought up an SCL output port of its own rather than
     * being handed one by rvd's bind chain -- a JPEG snapshot channel, which rvd
     * feeds by group membership instead. Only such a channel releases its port
     * again, since a video channel's port belongs to its framesource.
     */
    bool owns_port;

    /*
     * The SCL output port that feeds this channel, -1 when unbound. Not derivable
     * from the channel index: rvd pairs a JPEG channel with the framesource of
     * another stream, so its port and its channel number differ.
     */
    int src_port;

    /*
     * Which codec engine this channel lives on. Not a topology index: H.26x and
     * MJPEG are different devices, so this follows from the codec.
     */
    unsigned int device;
    rss_codec_t codec;
    unsigned short width;
    unsigned short height;

    /*
     * The descriptor MI_VENC_GetFd hands back, cached because closing and
     * reopening it per frame would be a syscall pair per frame for nothing.
     * -1 when not held.
     */
    int fd;

    /* One outstanding frame at a time, matching raptor's get/release contract. */
    bool frame_held;
    i6c_venc_strm strm;
    i6c_venc_pack packs[I6C_VENC_MAX_PACKS];
    i6c_venc_pack *heap_packs;
    unsigned int heap_count;

    /*
     * How many packs the array handed to MI_VENC_GetStream can hold. Kept because
     * the count comes back through the same struct it went out in: the array is
     * sized from a preceding MI_VENC_Query, and reading the returned count
     * without bounding it by this trusts a vendor library not to answer with more
     * than it was given room for.
     */
    unsigned int pack_cap;
    rss_nal_unit_t nals[I6C_VENC_MAX_PACKS];

    /*
     * The requested rate settings, kept because MI exposes no per-knob setter:
     * bitrate, GOP and frame rate all live in the rate half of the channel
     * attribute, which is read, modified and written back whole.
     */
    rss_video_config_t cfg;
} infinity6c_venc_chn_t;

/*
 * One OSD region. MI owns the pixel memory (allocated at create), so this holds
 * only the geometry and display attrs raptor tracks plus the converted bitmap
 * staging buffer. `grp` is the VENC channel the region shows on, -1 when
 * unregistered; `attached` is whether MI_RGN_AttachToChn has run, which is
 * deferred until that channel exists (see i6c_osd_flush_pending). Mirrors
 * star_osd_region_t.
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

    int grp;
    bool attached;
    bool show;

    /* Converted bitmap handed to MI_RGN_SetBitMap, kept per region so a
     * per-frame update reuses it; resized only on a geometry change. */
    void *bmp;
    size_t bmp_size;
    bool bmp_logged;
} infinity6c_osd_region_t;

/* ================================================================
 * BACKEND STATE
 * ================================================================ */

typedef struct {
    i6c_sys_api sys;
    i6c_snr_api snr;
    i6c_vif_api vif;
    i6c_isp_api isp;
    i6c_scl_api scl;
    i6c_venc_api venc;
    i6c_rgn_api rgn;

    bool sys_inited; /* MI_SYS_Init succeeded, so MI_SYS_Exit is owed */

    /*
     * The pipeline is brought up once and shared by every channel, because it
     * is one sensor feeding one SCL channel whose ports are the streams. So the
     * first framesource channel to be created builds it and the last one to be
     * destroyed tears it down, rather than either happening at init.
     */
    bool pipeline_up;
    unsigned int pipeline_refs;

    /*
     * What the sensor said about itself. Read once at bring-up and kept, since
     * VIF's pixel format, the ISP's yuv2bayer decision and the pool geometry
     * are all derived from it rather than configured.
     */
    i6c_snr_pad pad;
    i6c_snr_plane plane;
    int snr_profile; /* index into the sensor's resolution list; -1 = unset */
    unsigned int fps;

    /*
     * Which SCL output port feeds each encoder channel, taken from the FS -> OSD
     * half of rvd's bind chain so that the OSD -> ENC half can name it. rvd does
     * not repeat the framesource there, and it inserts the OSD stage on
     * `[osd] enabled` alone -- which defaults on -- so this is the normal path
     * rather than a special case. -1 when unset. See hal_bind.
     */
    int osd_src_port[I6C_MAX_CHN];

    /*
     * The geometry each VENC device's ring pool was configured for. Kept because
     * the pool is per device and not per channel -- i6c_sys_poolring names a
     * module and a device and nothing finer -- so several channels on one engine
     * share it and the largest of them is what has to fit.
     */
    unsigned short enc_pool_w[I6C_VENC_DEV_SLOTS];
    unsigned short enc_pool_h[I6C_VENC_DEV_SLOTS];

    /*
     * Whether each codec engine's VENC device has been created. MI 3.0 puts a
     * device above the channel, and a channel cannot be created until its device
     * has been -- MI 2.x had no such object, so star/ has no equivalent. Indexed
     * like the ring pools above: the first channel on an engine brings the device
     * up, and teardown takes it down once its channels are gone.
     */
    bool enc_dev_up[I6C_VENC_DEV_SLOTS];

    /*
     * Which encoder channel holds each engine's SCL ring, -1 when none does. The
     * first H.26x channel on an engine claims it (SCL -> VENC main); later H.26x
     * channels cascade off it in VENC. It is also the cascade source: a sub binds
     * VENC(enc_ring_chn) -> VENC(sub). Indexed like the pools above.
     */
    int enc_ring_chn[I6C_VENC_DEV_SLOTS];

    /*
     * The framesource channel that owns the one SCL output port feeding the
     * encoder ring, -1 when none does. The first video framesource claims it; a
     * later one is a cascade shadow with no port of its own. JPEG snapshot ports
     * are separate (the clone/spare-port path) and not counted here.
     */
    int scl_video_port;

    /*
     * ISP tuning. iq_file is the per-sensor IQ bin found on disk when the
     * pipeline came up, empty if none. CUS3A's AE init runs on a frame interrupt
     * and writes over a load issued before it, so the load is deferred to the
     * first delivered frame; iq_load_started latches that one load across the
     * encoder threads that each call get_frame. See i6c_isp_note_frame.
     */
    char iq_file[128];
    char iq_load_started;

    /*
     * OSD over MI_RGN. Brought up on the first region rather than at init, since
     * a stream with [osd] disabled never loads the library. rgn_fmt is the pixel
     * format the driver accepted at first create -- not knowable statically (the
     * accepted set is decided in mi_rgn.ko), so it is probed once. osd_grp[c] is
     * whether rvd created an OSD group on VENC channel c.
     */
    bool rgn_loaded;
    bool rgn_inited;
    bool rgn_fmt_known;
    i6c_rgn_pixfmt rgn_fmt;
    bool osd_grp[I6C_MAX_CHN];
    /* One warning per run for a stream whose port cannot carry an overlay. */
    bool osd_noscale_warned;
    infinity6c_osd_region_t osd[I6C_OSD_REGION_MAX];

    infinity6c_fs_chn_t fs[I6C_MAX_CHN];
    infinity6c_venc_chn_t enc[I6C_MAX_CHN];
} infinity6c_state_t;

/* ================================================================
 * SHARED HELPERS
 * ================================================================ */

/*
 * Guard boilerplate for a per-channel entry point. Every fs_* and enc_* op
 * begins by proving the context exists, the backend is initialised and the
 * channel index is in range, and there is no useful variation between them.
 */
#define I6C_ENTER(ctx, chn, st_var)                                                                \
    infinity6c_state_t *st_var;                                                                    \
    do {                                                                                           \
        rss_hal_ctx_t *_hal = (rss_hal_ctx_t *)(ctx);                                              \
        if (!_hal)                                                                                 \
            return RSS_ERR_INVAL;                                                                  \
        (st_var) = (infinity6c_state_t *)_hal->platform;                                           \
        if (!(st_var))                                                                             \
            return RSS_ERR_NOTSUP;                                                                 \
        if ((chn) < 0 || (chn) >= I6C_MAX_CHN)                                                     \
            return RSS_ERR_INVAL;                                                                  \
    } while (0)

/* Pipeline bring-up and teardown (hal_framesource.c). */
int i6c_pipeline_create(infinity6c_state_t *st, const rss_fs_config_t *cfg);
void i6c_pipeline_destroy(infinity6c_state_t *st);

/*
 * Buffer one stage's output port (hal_framesource.c). Best-effort against the
 * zero-depth default that makes the chain drop frames under jitter.
 */
void i6c_set_output_depth(infinity6c_state_t *st, i6c_sys_mod mod, unsigned int dev,
                          unsigned int chn, unsigned int port, unsigned int user, unsigned int buf);

/*
 * SCL output ports for a consumer that is not a raptor framesource channel --
 * a JPEG snapshot channel, which rvd never creates a framesource for.
 * (hal_framesource.c)
 */
int i6c_fs_spare_port(const infinity6c_state_t *st);
int i6c_fs_clone_port(infinity6c_state_t *st, int src_port, int dst_port);
int i6c_fs_port_ifc(infinity6c_state_t *st, int port);
int i6c_fs_enable_port(infinity6c_state_t *st, int port);
void i6c_fs_release_port(infinity6c_state_t *st, int port);

/*
 * Load the per-sensor IQ tuning on the first delivered frame (hal_framesource.c).
 * Called from every encoder channel's get_frame; loads once, or never if no bin
 * was found on disk. Deferred to the first frame because CUS3A's AE init would
 * otherwise overwrite it.
 */
void i6c_isp_note_frame(infinity6c_state_t *st);

/*
 * Bind and unbind an SCL output port to an encoder channel (hal_encoder.c).
 *
 * The port is passed rather than inferred from the channel: they are equal for a
 * video stream and are not for a JPEG one, and inferring it binds an
 * unconfigured port.
 */
int i6c_bind_scl_to_venc(infinity6c_state_t *st, int port, int chn, unsigned int dst_fps);
int i6c_unbind_scl_from_venc(infinity6c_state_t *st, int chn);

/*
 * Attach every registered region whose group is this VENC channel (hal_osd.c).
 * Called from i6c_bind_scl_to_venc once the channel exists, which is the first
 * moment MI_RGN_AttachToChn can succeed; rvd creates and registers its regions
 * before it binds the chain, so the attach is deferred to here. A no-op when OSD
 * was never brought up.
 */
void i6c_osd_flush_pending(infinity6c_state_t *st, int chn);

/*
 * Detach, destroy and free every region, then MI_RGN_DeInit (hal_osd.c). Called
 * at the head of i6c_teardown_all, before the VENC channels a region is attached
 * to are destroyed.
 */
void i6c_osd_release_all(infinity6c_state_t *st);

/* Release every channel's MI object, in dependency order (hal_common.c). */
void i6c_teardown_all(infinity6c_state_t *st);

#endif /* INFINITY6C_STATE_H */
