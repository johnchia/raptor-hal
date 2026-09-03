/*
 * hisi_v4/hisi_state.h -- shared backend state for the HiSilicon gen4 HAL
 *
 * Exists for the same reason star/star_state.h does: the backend spans more
 * than one translation unit and they need the same library handles and the
 * same unwind flags. It cannot live in hal_internal.h -- v4_common.h
 * includes hal_internal.h itself, for HAL_LOG_* and RSS_ERR_* -- so
 * rss_hal_ctx_t->platform points at hisi_state_t instead.
 *
 * Named hisi_state rather than v4_state: it is raptor's state, not a
 * transcription of anything in the SDK, and a v5 backend will have its own
 * with the same name in a sibling directory. The v4_ prefix is reserved for
 * things whose layout the vendor decides.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_STATE_H
#define HISI_V4_STATE_H

#include "v4_common.h"
#include "v4_isp.h"
#include "v4_aud.h"
#include "v4_isp_tune.h"
#include "v4_rgn.h"
#include "v4_snr.h"
#include "v4_sys.h"
#include "v4_venc.h"
#include "v4_vi.h"
#include "v4_video.h"
#include "v4_vpss.h"

#include <pthread.h>

/* ================================================================
 * FIXED TOPOLOGY
 *
 * One sensor on VI device 0, pipe 0, channel 0, feeding VPSS group 0.
 * gen4's numbering only becomes interesting with several sensors and this
 * target has one; hal_caps.c publishes max_sensors = 1.
 * ================================================================ */

#define HISI_VI_DEV 0
#define HISI_VI_CHN 0
#define HISI_VPSS_GRP 0

/*
 * The VI pipe, and the one place gen4's topology leaks out of the video
 * path: the ISP is keyed on vi_pipe rather than on a device of its own.
 * Every HI_MPI_ISP_*, HI_MPI_AE_* and HI_MPI_AWB_* call takes this as its
 * first argument, which is why hal_isp.c will reach for a topology constant
 * rather than an ISP one. Naming it here keeps that coupling visible
 * instead of leaving a bare 0 at forty call sites.
 */
#define HISI_VI_PIPE 0

/*
 * How many VPSS channels the backend will track, and how many VENC channels.
 *
 * V4_VPSS_MAX_PHY_CHN_NUM (3) is the SDK's bound for this chip and what
 * sizes the array; caps.max_fs_channels publishes the smaller *measured*
 * number until the create-0..5 experiment runs. Sizing by the header and
 * promising by the measurement are different questions, and conflating them
 * is how a caps table ends up asserting a header constant.
 *
 * The encoder array is RSS_MAX_ENC_CHANNELS rather than V4_VENC_MAX_CHN_NUM:
 * raptor's own ceiling is 8 and the driver's measured cap is 3, so sizing by
 * the vendor's 16 would allocate for channels no caller can ask for.
 */
#define HISI_VPSS_CHN_NUM V4_VPSS_MAX_PHY_CHN_NUM
#define HISI_VENC_CHN_NUM RSS_MAX_ENC_CHANNELS

/*
 * The first VPSS physical channel a stream can come out of.
 *
 * Not zero, because the VI -> VPSS bind takes channel 0. HI_MPI_SYS_Bind
 * overwrites the destination channel with 0 whenever the destination module
 * is VPSS -- it is visible in the driver, which does `if (dst.mod == VPSS)
 * dst.chn = 0` before recording the edge -- so the group's input is always
 * channel 0 and channel 0 cannot also be an output.
 *
 * The vendor's samples encode the same rule without stating it:
 * sample_vio.c uses VPSS_CHN0 in the VPSS-online functions, which bind
 * nothing to the group, and VPSS_CHN1 in the VI-bound offline ones. This
 * backend is always in the second case (see hisi_vpss_bringup).
 *
 * Measured before it was believed. With streams on channels 0 and 1, the
 * group processes frames and /proc/umap/vpss reports SendOk climbing on
 * channel 1 and stuck at zero on channel 0, whatever resolution each is
 * given -- swapping 1920x1080 and 640x360 between them moves the working
 * stream, not the broken one, and a single stream on channel 0 alone is
 * just as dead. Moving both to channels 1 and 2 starts both.
 *
 * So raptor's framesource index n is VPSS physical channel n + 1, and two
 * is all there is room for. That is the number caps_hisilicon.inc already
 * publishes as max_fs_channels, arrived at from the other direction.
 */
#define HISI_VPSS_CHN_BASE 1
#define HISI_FS_CHN_NUM (HISI_VPSS_CHN_NUM - HISI_VPSS_CHN_BASE)

/* raptor framesource index -> VPSS physical channel. */
static inline int hisi_vpss_phy(int fs_chn)
{
    return fs_chn + HISI_VPSS_CHN_BASE;
}

/*
 * Packs reported per frame.
 *
 * HiMPP emits one pack per NAL unit -- unlike SigmaStar's MI, which returns
 * one pack per frame containing several NALs -- so a pack maps directly onto
 * an rss_nal_unit_t and this is a NAL count. Sixteen covers
 * VPS+SPS+PPS+SEI+slice several times over; a frame with more reports its
 * first sixteen, which matches rvd's own ceiling so nothing downstream loses
 * anything it would have kept.
 */
#define HISI_VENC_MAX_PACKS 16

/* ================================================================
 * BACKEND STATE
 * ================================================================ */

/* ================================================================
 * SENSOR MODE
 *
 * Everything bring-up needs about the sensor that raptor's own config does
 * not carry, read from the vendor's /etc/sensors/<mode>.ini. See
 * hisi_sensor.c for why the INI rather than a table in the code.
 * ================================================================ */

typedef struct {
    /* As configured, or as detected from the loaded sys_config module's
     * sensors= parameter when the config leaves it out. */
    char name[32];
    char ini_path[192];
    char dll_file[64];
    char obj_name[64];

    /* [mode], and the three formats derived from raw_bitness -- derived once
     * here rather than read as three keys that could disagree. */
    v4_input_mode input_mode;
    int raw_bitness;
    v4_mipi_data_type mipi_data_type;
    v4_pixel_format pixel_format;
    v4_data_bitwidth bit_width;

    /* [mipi] */
    short lane_id[V4_MIPI_LANE_NUM];

    /* [isp_image] */
    int frame_rate;
    v4_bayer_format bayer;
    v4_wdr_mode wdr_mode;

    /* [vi_dev] -- the VI device attribute, verbatim from the vendor file. */
    v4_vi_intf_mode intf_mode;
    v4_vi_work_mode work_mode;
    unsigned int component_mask[V4_VI_COMPMASK_NUM];
    v4_vi_scan_mode scan_mode;
    unsigned int data_seq;
    v4_vi_sync_cfg sync_cfg;
    v4_vi_data_type input_data_type;
    int data_reverse;
    v4_rect dev_rect;
    unsigned int full_lines_std;
} hisi_sensor_mode_t;

int hisi_sensor_mode_load(hisi_sensor_mode_t *m, const char *sensor_name);

/* ================================================================
 * VB GEOMETRY
 * ================================================================ */

/*
 * COMMON_GetPicBufferSize, transcribed for the one case this backend needs:
 * NV12, 8-bit, uncompressed.
 *
 * From the SDK's own inline helper (mpp/include/hi_buffer.h:179 and the
 * COMMON_GetPicBufferConfig above it), reduced to the branch that applies:
 *
 *   align  = DEFAULT_ALIGN (8), which is what every vendor sample passes
 *   stride = ALIGN_UP(width * 8 / 8, align)
 *   height = ALIGN_UP(height, 2)
 *   size   = stride * height * 3 / 2
 *
 * Transcribed rather than approximated because an undersized VB block is
 * one of the two classic gen4 bring-up failures -- SYS_Init succeeds, the
 * pipeline builds, and VI silently delivers nothing.
 *
 * Here rather than in hal_common.c because both users of the formula need
 * it: hal_common sizes the common pools from the sensor, and
 * hal_framesource sizes a channel's own pool from the stream.
 */
#define HISI_VB_ALIGN 8u

static inline unsigned int hisi_vb_nv12_size(unsigned int width, unsigned int height)
{
    unsigned int stride = ((width + HISI_VB_ALIGN - 1u) / HISI_VB_ALIGN) * HISI_VB_ALIGN;
    unsigned int rows = (height + 1u) & ~1u;

    return stride * rows * 3u / 2u;
}

/* ================================================================
 * PER-CHANNEL BOOKKEEPING
 * ================================================================ */

/*
 * One VPSS channel, which is what a raptor framesource maps onto.
 *
 * Geometry is tracked rather than read back because rvd asks for it before
 * the channel exists and after it is destroyed, and because
 * HI_MPI_VPSS_GetChnAttr answers only while the group is running.
 */
typedef struct {
    bool configured;
    bool enabled;

    unsigned int width;
    unsigned int height;
    v4_frame_rate frame_rate;

    /* Degrees the channel's output is turned, as last set through
     * fs_set_rotation. width and height above stay the caller's; what
     * leaves the channel is height x width for 90 and 270. */
    int rotation;

    /*
     * u32Depth, the number of frames the channel queues for *userspace*.
     *
     * Zero is the streaming case and the right default: with depth 0 the
     * channel feeds its bound VENC and queues nothing, which is what every
     * stream wants. It is also why fs_set_frame_depth is a real op here and
     * not bookkeeping -- rvd's snapshot path raises the depth to take a
     * picture by hand, and with depth 0 GetChnFrame would block until its
     * timeout on a channel that is otherwise working perfectly.
     */
    unsigned int depth;

    /* Set while a frame checked out through fs_get_frame is outstanding.
     * HiMPP requires the same descriptor back, so it is stored here and the
     * caller gets a pointer into it. */
    bool frame_held;
    v4_video_frame_info frame;

    /*
     * This channel's own VB pool, when it has one.
     *
     * hal_init cannot size a common pool for streams it is never told about
     * -- rss_multi_sensor_config_t carries sensors, not streams -- so a
     * channel whose frames would waste a sensor-sized block gets a pool cut
     * to its own geometry at create time, when the geometry is finally
     * known. See hisi_fs_pool_acquire.
     *
     * Two fields rather than V4_VB_INVALID_POOL in one, because the state is
     * zeroed at allocation and pool 0 is a real pool: "no pool" has to be
     * the all-zero state or a fresh channel claims someone else's.
     */
    bool vb_pool_owned;
    unsigned int vb_pool;
    unsigned long long vb_pool_blk_size;
} hisi_vpss_chn_t;

/*
 * One VENC channel.
 *
 * `bound_fs` is the VPSS channel feeding it, or -1. It is what unbind needs
 * at teardown and what makes a double-bind detectable; -1 rather than 0
 * because channel 0 is a real channel and "not bound" needs its own value.
 */
typedef struct {
    bool created;
    bool receiving;
    int bound_fs;
    /* Where a duty-cycled MJPEG channel rebinds when it restarts. enc_stop
     * unbinds those channels -- a stopped-but-bound destination queues the
     * source's pictures without ever releasing them, and four queued 5 MP
     * frames are pool 0 in its entirety -- so the edge to remake has to
     * survive the unbind that cleared bound_fs. -1 otherwise. */
    int idle_fs;
    int fd;

    rss_codec_t codec;
    v4_payload_type payload;
    unsigned int width;
    unsigned int height;

    /* The rate-control state, kept because HiMPP has no per-knob setter:
     * enc_set_bitrate and friends are read-modify-writes of the whole
     * channel attribute, and re-deriving the untouched half from rvd's
     * config on every call would lose anything set through another op. */
    rss_rc_mode_t rc_mode;
    unsigned int bitrate;
    unsigned int gop;
    unsigned int fps_num;
    unsigned int fps_den;

    /*
     * The rest of what the channel attribute is built from, captured at
     * create time.
     *
     * Not a cache of rvd's config: a reconfigure has to rebuild the *whole*
     * VENC_CHN_ATTR_S, and any field it cannot recover would silently
     * revert to a default. Storing profile and the QP deltas here is what
     * makes "set the bitrate" change the bitrate and nothing else.
     */
    unsigned int profile;
    unsigned int buf_size;
    int ip_qp_delta;
    int init_qp;

    /*
     * QP bounds, -1 for "the driver's default".
     *
     * These do not live in VENC_CHN_ATTR_S at all -- they are the
     * VENC_RC_PARAM_S half of rate control, written through a second MPI
     * call (hisi_enc_apply_rc_param). Tracked here for the same reason the
     * rate knobs are: the apply is a get-modify-set of a structure whose
     * other fields belong to the driver, so raptor has to remember which
     * fields are its own.
     */
    int16_t min_qp;
    int16_t max_qp;
    /* The driver's own QP bounds, read once before anything of raptor's
     * was written over them, so an unset or reset side can go back to
     * them rather than to whatever the last write left. Per RC mode,
     * because the union shape and the defaults both change with it. */
    bool rc_drv_known;
    v4_venc_rc_mode rc_drv_mode;
    unsigned int rc_drv_min_qp, rc_drv_max_qp;
    unsigned int rc_drv_min_iqp, rc_drv_max_iqp;
    bool rc_written; /* raptor has written the bounds at least once */

    /* One outstanding stream per channel. HiMPP wants the same descriptor
     * back at ReleaseStream, and rss_frame_t has nowhere to keep it. */
    bool frame_held;
    v4_venc_stream stream;
    v4_venc_pack packs[HISI_VENC_MAX_PACKS];
    rss_nal_unit_t nals[HISI_VENC_MAX_PACKS];
} hisi_venc_chn_t;

/*
 * OSD regions the backend will track at once.
 *
 * Not the vendor's RGN_HANDLE_MAX (128): the limit a caller runs into is
 * OVERLAY_MAX_NUM_VENC, which is 8 overlays per *encoder channel*, and the
 * handle space is only how those are numbered. 16 is two channels' worth
 * and matches rvd's largest per-platform region budget in hal_caps.c, so a
 * config that works on Ingenic is not silently truncated here. A region
 * costs a handle and its conversion buffer, nothing per-slot, so the array
 * is cheap. hal_osd.c enforces the per-channel 8 at register time.
 */
#define HISI_OSD_REGION_MAX 16

/*
 * One OSD region.
 *
 * Everything is tracked rather than read back from RGN, for the reason the
 * SigmaStar backend gives and one more of its own: the values raptor asks
 * for are split between the region attr (geometry) and the *per-channel*
 * display attr (position, alpha, layer, show), and the display attr does
 * not exist until the region is attached -- which on this family cannot
 * happen until the VENC channel exists, which is later than rvd sets them.
 *
 * `attached` is the distinction that matters. Registered means rvd asked
 * for the region to appear on a group; attached means HI_MPI_RGN_AttachToChn
 * has actually been called. See hisi_osd_flush_pending.
 *
 * `grp` is the VENC channel, and carrying it here is the fix for divinus's
 * per-stream OSD defect -- see hal_osd.c.
 */
typedef struct {
    bool used;
    rss_osd_type_t type;

    int x;
    int y;
    /* The region as RGN sees it: even, at least 2. */
    unsigned int width;
    unsigned int height;
    /* The bitmap as the caller renders it, which may be a pixel narrower or
     * shorter than the above. Kept separately so the conversion never reads
     * past the caller's buffer; see hisi_osd_convert. */
    unsigned int src_w;
    unsigned int src_h;
    int layer;

    bool global_alpha_en;
    unsigned char fg_alpha;
    unsigned char bg_alpha;

    /* Registered VENC channel, or -1. */
    int grp;
    bool attached;
    bool show;

    /* ARGB1555 conversion buffer handed to HI_MPI_RGN_SetBitMap. Kept per
     * region so a per-frame update does not allocate, and resized only when
     * the geometry changes. */
    void *bmp;
    size_t bmp_size;

    /* Set once the first bitmap has been accepted, so that fact can be
     * logged exactly once per region: without it "attached but never fed"
     * and "fed but not composited" look identical in the log. */
    bool bmp_logged;
} hisi_osd_region_t;

/*
 * One [image] knob's wanted value. `asked` is the whole difference between
 * a knob nobody has set -- where the tuning's own number stands, and must
 * keep standing across a reload -- and one somebody set to the same number.
 */
typedef struct {
    bool asked;
    int val;
} hisi_knob_slot_t;

typedef struct {
    /* Loaded vendor libraries, and the tables resolved out of them.
     * Layout order follows star_state.h: vtables first, then descriptors,
     * then shadow state, then the unwind flags. */
    v4_mpi_libs libs;
    v4_sys_impl sys;
    v4_vi_impl vi;
    v4_vpss_impl vpss;
    v4_venc_impl venc;
    v4_isp_impl isp;
    v4_snr_impl snr;
    /* RGN, and whether its entry points resolved. Unlike the tables above,
     * a failure to resolve is not fatal: it costs the OSD ops and nothing
     * else, so every one of them tests this flag rather than hal_init
     * refusing to come up. */
    v4_rgn_impl rgn;
    bool rgn_loaded;

    /*
     * SoC identity, read once during hal_init.
     *
     * chip_id is the raw SCSYSID word; chip_name is it decoded, e.g.
     * "Hi3516EV300". Both are kept because the decode is a guess for any
     * part nobody has held: an unrecognised board still logs a number
     * somebody can look up.
     */
    unsigned int chip_id;
    char chip_name[16];

    /* MPP version string as reported by HI_MPI_SYS_GetVersion, e.g.
     * "Hi3516EV200_MPP_V1.0.1.2 B030 Release". Note that an EV300 reports
     * the EV200 string: one MPP build serves the whole generation. */
    char mpp_version[V4_VERSION_NAME_MAXLEN];

    /*
     * The sensor as named by the config, or as the mode load resolved it
     * from the sys_config module's sensors= parameter when the config left
     * it out (hisi_sensor_detect). gen4 has no /proc/jz/sensor; identity is
     * whichever libsns_*.so gets dlopened, and that is chosen by this name.
     */
    char sensor_name[32];

    /*
     * Callback targets for the GK_API_* forwarders defined in hal_common.c.
     * They live in the state rather than in file statics so the forwarders
     * and the ISP loader cannot disagree about which library is current;
     * see the TRAMPOLINES block in hal_common.c. Phase 2 fills them when
     * libisp.so and the 3A libraries load.
     */
    /*
     * Arities are transcribed from ref/openhisilicon/include/gk_api_isp.h:25
     * and :28, gk_api_ae.h:20 and :23, gk_api_awb.h:20 and :23 -- which is
     * the whole reason to transcribe rather than guess: ISP's Reg takes
     * three arguments where AE's and AWB's take four, and every Unreg ends
     * in a SENSOR_ID rather than in the pointer that precedes it. Payload
     * pointers stay void *: a forwarder does not read what it passes
     * through, and giving it a type it never dereferences would mean
     * transcribing four more structs for nothing.
     */
    int (*fn_isp_sensor_reg_cb)(int vi_pipe, void *sns_attr, void *reg);
    int (*fn_isp_sensor_unreg_cb)(int vi_pipe, int sensor_id);
    int (*fn_isp_get_mod_param)(void *mod_param);
    int (*fn_ae_sensor_reg_cb)(int vi_pipe, void *ae_lib, void *sns_attr, void *reg);
    int (*fn_ae_sensor_unreg_cb)(int vi_pipe, void *ae_lib, int sensor_id);
    int (*fn_awb_sensor_reg_cb)(int vi_pipe, void *awb_lib, void *sns_attr, void *reg);
    int (*fn_awb_sensor_unreg_cb)(int vi_pipe, void *awb_lib, int sensor_id);

    /*
     * The three algorithm registrars libisp.so reaches for, resolved out of
     * lib_hidrc.so, lib_hidehaze.so and lib_hildci.so once those are loaded.
     *
     * These exist for a different reason from the GK_API_* set above, and
     * the reason is musl. See the ISP CYCLE block in hal_common.c: libisp
     * and the algorithm libraries reference each other, musl performs no
     * lazy binding, and so neither can be dlopen'd first unless the
     * executable stands in for one direction of the cycle.
     *
     * Arity confirmed against the board's own lib_hidrc.so, not just a
     * header: ISP_AlgRegisterDrc touches only r0 before indexing its
     * per-pipe context array.
     */
    int (*fn_alg_register_drc)(int vi_pipe);
    int (*fn_alg_register_dehaze)(int vi_pipe);
    int (*fn_alg_register_ldci)(int vi_pipe);

    /* The sensor mode, read once during hal_init. */
    hisi_sensor_mode_t mode;

    /*
     * The VI/VPSS coupling actually in force, read back after setting it.
     *
     * Kept because it decides whether VI -> VPSS is a software bind at all:
     * in a VPSS-*online* mode the two are connected in hardware and
     * HI_MPI_SYS_Bind must not be called for that edge. See
     * hisi_vpss_bringup.
     */
    v4_vi_vpss_mode_e vi_vpss_mode;

    /*
     * Orientation: bMirror and bFlip of every VPSS channel, the same on
     * each. Seeded from the config before the first channel is created and
     * rewritten by isp_set_hflip / isp_set_vflip, which VPSS takes on an
     * enabled channel (hisi_fs_apply_orien).
     *
     * Two other places were tried first. The sensor's pfnMirrorFlip, the
     * SigmaStar precedent: optional in ISP_SNS_OBJ_S and left out of the
     * OpenIPC sensor libraries -- the IMX335's has no mirror or flip code at
     * all -- so a configured flip silently did nothing. Then the VI
     * channel's own bMirror / bFlip, which take the write and are the
     * natural single point: flip streams, but mirror stalls the channel on
     * the EV300 in the offline VI mode this backend runs -- VI CHN STATUS
     * counts every frame as lost and the encoders see nothing until it is
     * cleared. VPSS's bits do both, live, which is also where divinus puts
     * them.
     */
    int mirror;
    int flip;

    /* Per-channel state. */
    hisi_vpss_chn_t fs[HISI_FS_CHN_NUM];
    hisi_venc_chn_t enc[HISI_VENC_CHN_NUM];

    /*
     * The framesource named by an FS -> OSD bind, remembered until the
     * matching OSD -> ENC bind arrives, or -1.
     *
     * rvd expresses an overlaid stream as two binds and names the
     * framesource only in the first. HiMPP has no OSD stage in the
     * datapath -- RGN regions attach to a VENC channel -- so the pair
     * collapses to one FS -> VENC bind, and the first half has to be
     * remembered rather than acted on. -1 rather than 0 because
     * framesource 0 is a real framesource.
     */
    int osd_src_fs[HISI_VENC_CHN_NUM];

    /*
     * Phase 5 -- OSD. The regions themselves, and which groups rvd has
     * asked for.
     *
     * A group is an encoder channel and has no RGN object of its own, so
     * osd_grp only records that rvd created one -- what it buys is a
     * destroy_group that knows which regions to detach. osd_pool_logged
     * makes the "there is no pool to size" line once per process rather
     * than once per rvd osd-restart.
     */
    hisi_osd_region_t osd[HISI_OSD_REGION_MAX];
    bool osd_grp[HISI_VENC_CHN_NUM];
    bool osd_pool_logged;

    /*
     * The ISP thread.
     *
     * HI_MPI_ISP_Run does not return: it is the ISP's own service loop and
     * runs for the lifetime of the pipeline. So it gets a thread, and
     * teardown stops it by calling HI_MPI_ISP_Exit -- which is what makes
     * Run return -- rather than by cancelling, because a thread cancelled
     * inside the vendor library leaves its locks held.
     */
    pthread_t isp_thread;

    /*
     * Two flags rather than one, and both touched from two threads, so both
     * go through __atomic. `running` is teardown's statement of intent --
     * cleared before HI_MPI_ISP_Exit so the thread can tell an intentional
     * stop from the pipeline dying. `done` is the thread's own report that
     * it has left the vendor library, which is what makes the join in
     * hisi_video_teardown bounded instead of a place rvd can hang forever.
     */
    bool isp_thread_started;
    int isp_thread_running;
    int isp_thread_done;
    /* Set when the join timed out and the thread was detached: it is still
     * executing inside libisp/libsns, so teardown must not dlclose those
     * handles and hal_deinit must leak this state block rather than free
     * memory the thread will still write to. */
    bool isp_thread_leaked;

    /*
     * Phase 3 -- IQ tuning. The file is settled at bring-up
     * (hisi_isp_resolve_iq), the load happens on the first encoded frame
     * (hisi_isp_note_frame); iq_load_started is the atomic latch that
     * makes exactly one encoder thread do it. The entry points live in
     * `tune` and are resolved lazily inside the load, all optional --
     * see hal_isp.c's OP COVERAGE block.
     */
    char iq_file[128];
    char iq_load_started;
    bool tune_resolved; /* hisi_isp_tune_resolve ran; also lets the host
                         * tests pre-install stub entry points */
    v4_isp_tune_impl tune;
    /* [static_3dnr]: the VPSS NRX ladder, parsed and packed by hal_nrx.c.
     * Heap, and kept for the process's life, because the AUTO form hands
     * the driver pointers into it. */
    struct hisi_nrx_set *nrx;
    /* [dynamic_linear_drc], [dynamic_dehaze], [dynamic_gamma]: the per-ISO
     * ISP engines, hal_dyn.c. Its tick is the one clock for every engine
     * that follows AE -- the 3DNR ladder above included. */
    struct hisi_dyn_set *dyn;
    long long iso_tick_ns; /* the tick's next due time, CLOCK_MONOTONIC */
    char iso_busy;         /* one tick at a time, across encoder threads */

    /*
     * The [image] knobs, hal_knob.c. Each is remembered because a tuning
     * load rewrites the attributes two of them live in and the loader
     * re-applies them afterwards; the two baselines are what the tuning
     * left there, learned before the knob's first write over it and
     * forgotten at every load.
     */
    struct {
        hisi_knob_slot_t brightness, contrast, saturation, ae_comp, drc;
        bool ae_base_known;
        int ae_base;
        bool drc_base_known;
        int drc_base;
        int drc_base_op;
        bool exp_warned;
    } knob;

    /*
     * Phase 4 -- audio. One AI device, one channel, one frame in flight;
     * the codec fd is /dev/acodec, held open because volume, gain and
     * mute all go through it at runtime.
     *
     * aud_owns_sys records that audio_init did the SYS attach itself
     * (rad's normal path -- it never calls hal_init). Note what it does
     * NOT drive: an Exit. SYS and VB state are kernel-global on HiMPP
     * and rvd may be streaming in another process, so the audio archive
     * never calls HI_MPI_SYS_Exit -- the process's own exit is the real
     * detach. See the audit note in the plan ("One MPP consumer per
     * system") and hal_audio.c's OP COVERAGE.
     */
    v4_aud_impl aud;
    bool aud_loaded;
    bool aud_owns_sys;
    bool aud_dev_enabled;
    bool aud_chn_enabled;
    int aud_dev;
    int acodec_fd;
    int aud_rate;
    /* No cached volume/gain here on purpose: both analog controls are
     * pure /dev/acodec pass-throughs, so the getters read the codec and
     * a cache would only ever hold what the codec already knows. */
    bool aud_gain_clamped; /* the "gain exceeds the codec max" warning is once-only */
    v4_audio_frame aud_frame;
    v4_aec_frame aud_aec;
    bool aud_frame_held;
    int aud_last_err;

    /*
     * Unwind flags -- one per bring-up step, set only once that step has
     * succeeded, so hisi_teardown undoes exactly what was done and no more.
     *
     * This is the part of the design worth being strict about on HiMPP.
     * Teardown ordering is where a bad gen4 bring-up leaves the board
     * needing a power cycle: VB_Exit with SYS still up, or SYS_Exit with a
     * VI pipe still running, wedges the kernel side rather than returning
     * an error. t_hisi_bind asserts the reverse ordering on the host,
     * including for an init that failed partway through.
     */
    bool vb_configured;
    bool vb_inited;
    bool sys_inited;

    /*
     * Common pool 0's block size, and whether this libmpi can cut private
     * pools at all. Both are settled in hisi_vb_bringup and read by
     * hal_framesource: the first is the threshold above which a channel is
     * better off in the common pool it would fill anyway, the second is what
     * decides whether a common stream pool is configured in the first place.
     * Zero and false on an audio-only build, which configures no pools.
     */
    unsigned long long vb_blk_size;
    bool vb_private_pools;

    bool mipi_configured;
    bool sensor_registered;
    bool ae_registered;
    bool awb_registered;
    bool isp_inited;

    bool vi_dev_enabled;
    bool vi_pipe_created;
    bool vi_pipe_started;
    bool vi_chn_enabled;

    bool vpss_grp_created;
    bool vpss_grp_started;
    bool vi_vpss_bound;

    /*
     * The group crop is one window shared by every channel, so it carries
     * an owner: the framesource channel whose request set it, or -1. The
     * owner may move or clear its window (a resolution change from 16:9
     * back to 4:3 must not keep the old 16:9 window latched); a non-owner
     * asking for the identical rect is the normal two-streams-same-aspect
     * case and succeeds silently; a non-owner asking for a different rect
     * is refused with a warning, because one window cannot serve both.
     */
    int vpss_crop_owner;
    int vpss_crop_x, vpss_crop_y;
    unsigned int vpss_crop_w, vpss_crop_h;
} hisi_state_t;

static inline hisi_state_t *hisi_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (hisi_state_t *)c->platform : NULL;
}

/* ================================================================
 * CHIP IDENTIFICATION
 * ================================================================ */

/*
 * SCSYSID0, the chip identification register, and the gen4 IDs read out of
 * it.
 *
 * 0x12020000 is the system controller block; the ID word sits at +0xEE0.
 * The address is not discoverable from /proc/iomem -- the block is claimed
 * by no driver -- so it is a constant here, cross-checked against the one
 * thing /proc/iomem does say: the UART at 0x12040000, which is what divinus
 * keys its own gen3/gen4 base selection on (src/hal/support.c:233-245).
 *
 * 0x3516E300 is measured on a live EV300. The rest follow the family's
 * encoding rather than being copied from anywhere, so they are labelled as
 * such: a mismatch logs the raw word, which is the useful half either way.
 */
#define HISI_SCSYSID0_ADDR 0x12020EE0u

#define HISI_CHIP_HI3516EV200 0x3516E200u /* inferred from the encoding */
#define HISI_CHIP_HI3516EV300 0x3516E300u /* measured, 192.168.1.196 */
#define HISI_CHIP_HI3516DV200 0x3516D200u /* inferred from the encoding */
#define HISI_CHIP_HI3518EV300 0x3518E300u /* inferred from the encoding */

static inline bool hisi_chip_is_gen4(unsigned int id)
{
    return id == HISI_CHIP_HI3516EV200 || id == HISI_CHIP_HI3516EV300 ||
           id == HISI_CHIP_HI3516DV200 || id == HISI_CHIP_HI3518EV300;
}

/*
 * The part this binary's caps were written for.
 *
 * The only per-PLATFORM conditional in the backend, and the exception that
 * proves the HAL_HISI_GEN4 rule: it does not select code, it names what
 * rss_hal_check_platform should expect to find. Running a gen4 binary on a
 * different gen4 part is correct-but-approximate rather than wrong -- the
 * MPP build is the same one -- so this drives a warning, not a refusal.
 *
 * A new gen4 part adds a line here and a caps block, and nothing else.
 */
#if defined(PLATFORM_HI3516EV200)
#define HISI_CHIP_THIS_PART HISI_CHIP_HI3516EV200
#elif defined(PLATFORM_HI3516EV300)
#define HISI_CHIP_THIS_PART HISI_CHIP_HI3516EV300
#else
#error "No gen4 PLATFORM_* defined"
#endif

/* ================================================================
 * CROSS-FILE OPS
 *
 * Declared here rather than in each caller because hal_common.c's vtable
 * and the teardown path both need them, and a header is the only place two
 * translation units can agree on a signature.
 * ================================================================ */

/* Framesource -- src/hisi_v4/hal_framesource.c */
int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_destroy_channel(void *ctx, int chn);
int hal_fs_enable_channel(void *ctx, int chn);
int hal_fs_disable_channel(void *ctx, int chn);
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
int hal_fs_release_frame(void *ctx, int chn, void *frame_data);
int hal_fs_set_frame_depth(void *ctx, int chn, int depth);
int hal_fs_get_frame_depth(void *ctx, int chn, int *depth);
int hal_fs_set_rotation(void *ctx, int chn, int degrees);
int hisi_fs_apply_orien(hisi_state_t *st);

/* Called from hisi_teardown so a channel's held frame and enable state do
 * not outlive the VPSS group. */
void hisi_fs_release_all(hisi_state_t *st);

/* Encoder -- src/hisi_v4/hal_encoder.c */
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
int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp);
int hal_enc_set_qp_bounds(void *ctx, int chn, int min_qp, int max_qp);
int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp);
void hisi_enc_refresh_rc(hisi_state_t *st, int enc_chn);
int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den);
int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg);
int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate);
int hal_enc_query(void *ctx, int chn, bool *busy);
int hal_enc_get_fd(void *ctx, int chn);

void hisi_enc_release_all(hisi_state_t *st);
int hisi_venc_max_chn(char *src, size_t src_len);

/* OSD == RGN overlays attached to a VENC channel -- src/hisi_v4/hal_osd.c.
 * The OP COVERAGE block at the top of that file argues each absence. */
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
 * The registered-versus-attached deferral. flush_pending attaches every
 * region waiting on a VENC channel and is called from hal_bind and from
 * hal_enc_create_channel; detach_chn is called from hal_enc_destroy_channel
 * *before* the channel is destroyed, because HiMPP refuses to destroy a
 * VENC channel that still carries regions. Between the two, a region
 * survives an encoder restart. release_all runs from hisi_video_teardown,
 * ahead of the encoders.
 */
void hisi_osd_flush_pending(hisi_state_t *st, int chn);
void hisi_osd_detach_chn(hisi_state_t *st, int chn);
void hisi_osd_release_all(hisi_state_t *st);

/* ISP -- the sensor geometry accessor, plus Phase 3's tuning load. All
 * three live in hal_isp.c; resolve runs once from hal_init, note_frame is
 * the per-frame latch the encoder's frame loop pays one atomic test for. */
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height);
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);
void hisi_isp_resolve_iq(hisi_state_t *st);
void hisi_isp_note_frame(hisi_state_t *st);
void hisi_isp_tune_resolve(hisi_state_t *st);

/* hal_knob.c -- the [image] knobs over the tuning's baseline, and the
 * exposure readback. The loader brackets each load with the two hooks. */
int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);
int hal_isp_set_brightness(void *ctx, int val);
int hal_isp_get_brightness(void *ctx, int *val);
int hal_isp_set_contrast(void *ctx, int val);
int hal_isp_get_contrast(void *ctx, int *val);
int hal_isp_set_saturation(void *ctx, int val);
int hal_isp_get_saturation(void *ctx, int *val);
int hal_isp_set_ae_comp(void *ctx, int val);
int hal_isp_get_ae_comp(void *ctx, int *val);
int hal_isp_set_drc_strength(void *ctx, int val);
int hal_isp_get_drc_strength(void *ctx, int *val);
int hal_isp_get_knob_caps(void *ctx, const char *name, rss_isp_knob_t *caps);
void hisi_knob_before_load(hisi_state_t *st);
void hisi_knob_reapply(hisi_state_t *st);
/* Orientation, on the VPSS channels; see hisi_state_t.mirror. */
int hal_isp_set_hflip(void *ctx, int enable);
int hal_isp_set_vflip(void *ctx, int enable);
int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip);

/* hal_nrx.c -- the [static_3dnr] section (VPSS 3DNR X-params). */
bool hisi_nrx_key(hisi_state_t *st, const char *key, const char *val);
int hisi_nrx_apply(hisi_state_t *st, char *note, size_t note_len);
bool hisi_nrx_armed(hisi_state_t *st);
void hisi_nrx_on_iso(hisi_state_t *st, unsigned iso);
void hisi_nrx_free(hisi_state_t *st);

/* hal_dyn.c -- the dynamic ISP sections, and the ISO tick that drives
 * them and the ladder above. The axis helpers are the SDK sample's. */
unsigned hisi_iso_map(unsigned iso);
unsigned hisi_iso_lerp(unsigned mid, unsigned left, unsigned lv, unsigned right, unsigned rv);
bool hisi_iso_query(hisi_state_t *st, unsigned *iso, unsigned long long *exposure);
bool hisi_dyn_key(hisi_state_t *st, const char *sect, const char *key, const char *val);
int hisi_dyn_apply(hisi_state_t *st, int *failed, char *note, size_t note_len);
void hisi_dyn_on_exposure(hisi_state_t *st, unsigned iso, unsigned long long exposure);
void hisi_dyn_tick(hisi_state_t *st);
void hisi_dyn_drc_hold(hisi_state_t *st, bool hold);
bool hisi_dyn_drc_curve(hisi_state_t *st);
void hisi_dyn_free(hisi_state_t *st);

/* The __ctype_b/mmap trampoline verification in hal_common.c. Exposed
 * because there are two entry points that reach the first vendor dlopen:
 * hal_init, and hal_audio.c's audio_init, which rad calls without ever
 * running hal_init. */
void hisi_check_trampolines(void);

/* Audio -- Phase 4, hal_audio.c (the HAL_MODULE_AUDIO archive). */
int hal_audio_init(void *ctx, const rss_audio_config_t *cfg);
int hal_audio_deinit(void *ctx);
int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block);
int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame);
int hal_audio_set_volume(void *ctx, int dev, int chn, int vol);
int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol);
int hal_audio_set_gain(void *ctx, int dev, int chn, int gain);
int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain);
int hal_audio_set_mute(void *ctx, int dev, int chn, int mute);

/* Bind, shared by hal_common.c's ops and by the encoder's register path --
 * both express "this VPSS channel feeds this VENC channel". */
int hisi_bind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn);
int hisi_unbind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn);

#endif /* HISI_V4_STATE_H */
