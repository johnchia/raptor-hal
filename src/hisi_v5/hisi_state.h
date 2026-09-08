/*
 * hisi_v5/hisi_state.h -- shared backend state for the HiSilicon gen5 HAL
 *
 * The sibling of src/hisi_v4/hisi_state.h and it exists for the same
 * reason: the backend spans more than one translation unit and they need
 * the same library handles and the same unwind flags. It cannot live in
 * hal_internal.h -- v5_common.h includes hal_internal.h itself, for
 * HAL_LOG_* and RSS_ERR_* -- so rss_hal_ctx_t->platform points at
 * hisi_state_t instead.
 *
 * Named hisi_state rather than v5_state, and the two generations use the
 * same name deliberately: it is raptor's state, not a transcription of
 * anything in the SDK, and exactly one of the two directories is ever
 * compiled into a given build. The v5_ prefix is reserved for things whose
 * layout the vendor decides.
 *
 * Phase 1 fills the SYS and VB half. VI, VPSS, VENC, ISP, RGN and audio
 * tables join as their phases land, and the "list only what exists" rule
 * applies to this struct as much as to mk/hisilicon.mk: a member for a
 * table nobody resolves is a member somebody will read as a promise.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_STATE_H
#define HISI_V5_STATE_H

#include "v5_common.h"
#include "v5_sys.h"
#include "v5_vb.h"
#include "v5_video.h"
#include "v5_mipi.h"
#include "v5_vi.h"
#include "v5_vpss.h"
#include "v5_venc.h"
#include "v5_isp.h"
#include "v5_isp_tune.h"
#include "v5_nr.h"
#include "v5_snr.h"
#include "v5_aud.h"

#include <pthread.h>

/* ================================================================
 * FIXED TOPOLOGY
 *
 * One sensor on VI device 0, pipe 0, channel 0, feeding VPSS group 0. The
 * numbering only becomes interesting with several sensors and this target
 * has one; hal_caps.c publishes max_sensors accordingly.
 * ================================================================ */

#define HISI_VI_DEV 0
#define HISI_VI_CHN 0
#define HISI_VPSS_GRP 0

/*
 * The VI pipe, and the one place the topology leaks out of the video path:
 * the ISP is keyed on vi_pipe rather than on a device of its own. Every
 * ss_mpi_isp_*, ss_mpi_ae_* and ss_mpi_awb_* call takes this as its first
 * argument. Naming it here keeps that coupling visible instead of leaving
 * a bare 0 at forty call sites.
 *
 * Unchanged from gen4, and unchanged for a reason worth recording: V5 grew
 * two *virtual* pipes (OT_VI_MAX_VIRT_PIPE_NUM 2, ot_defines.h:240) on top
 * of the two physical ones, and they are for WDR fusion and stitching
 * rather than for a second stream. raptor drives physical pipe 0.
 */
#define HISI_VI_PIPE 0

/*
 * How many VPSS channels the backend tracks, and how many VENC channels.
 *
 * OT_VPSS_MAX_PHYS_CHN_NUM is 3 on this part (plus two extension channels
 * this backend does not use), which is what sizes the array. What
 * caps.max_fs_channels *promises* is the measured number, and it stays 0
 * until Phase 2 runs the experiment -- sizing by the header and promising
 * by the measurement are different questions, and conflating them is how a
 * caps table ends up asserting a header constant.
 *
 * The encoder array is RSS_MAX_ENC_CHANNELS rather than the vendor's 16:
 * raptor's own ceiling is 8, and OpenIPC's load_hisilicon loads the venc
 * module with g_venc_max_chn_num=8 in any case, so sizing by 16 would
 * allocate for channels no caller can ask for.
 */
#define HISI_VPSS_CHN_NUM 3
#define HISI_VENC_CHN_NUM RSS_MAX_ENC_CHANNELS

/*
 * The first VPSS physical channel a stream can come out of.
 *
 * ZERO, and the evidence is the vendor's own sample rather than a header.
 *
 * On gen4 this is 1: HI_MPI_SYS_Bind forces the destination channel to 0
 * whenever the destination module is VPSS, so the group's input is always
 * channel 0 and channel 0 cannot also be an output. That was plan risk R4
 * for V5 -- whether ss_mpi_sys_bind has the same quirk.
 *
 * It does not. sample_venc.c:764 assigns `venc_vpss_chn->vpss_chn[i] = i`
 * and binds VPSS channel 0 to VENC channel 0 in the same breath as the VI
 * -> VPSS bind that names destination channel 0. The two uses of "channel
 * 0" are different namespaces on V5: the bind's destination channel is the
 * group's input port, and the output channels are numbered from 0
 * independently.
 *
 * The bench confirms it -- three channels enabled at 0, 1 and 2 all deliver
 * (see /proc/umap/vpss's SendOk per channel with a stream running).
 */
#define HISI_VPSS_CHN_BASE 0

/*
 * The historical note this replaces, kept because the reasoning is what
 * makes the constant readable rather than arbitrary.
 *
 * On gen4 this is 1, because HI_MPI_SYS_Bind forces the destination channel
 * to 0 whenever the destination module is VPSS, so the group's input is
 * always channel 0 and channel 0 cannot also be an output. That was
 * measured on a board (see the gen4 header's account) after being read out
 * of the driver.
 *
 * Whether ss_mpi_sys_bind has the same quirk is plan risk R4 and an
 * explicit bench item: create channels 0, 1 and 2 at different sizes,
 * bind VI pipe/chn to VPSS group 0, and watch SendOk per channel in
 * /proc/umap/vpss. Until that has run, this backend has no framesource, so
 * the constant has nothing to be wrong about -- and defining it now to
 * either value would be inventing the answer.
 *
 */

static inline int hisi_vpss_phy(int fs_chn)
{
    return HISI_VPSS_CHN_BASE + fs_chn;
}

/* ================================================================
 * SENSOR MODE
 *
 * Everything bring-up needs about the sensor that raptor's own config does
 * not carry. Read from an INI, for the reason hisi_sensor.c states at
 * length: a table in the code covers whichever sensors somebody tested and
 * silently excludes the rest.
 *
 * The V5 file layout differs from gen4's in three ways, and hisi_sensor.c
 * has the account. In short: the object symbol is spelled g_sns_<name>_obj;
 * the I2C bus is part of the mode rather than assumed; and the *die* caps
 * the geometry, so a mode file carries per-die overrides.
 * ================================================================ */

typedef struct {
    /* As configured, or as detected. */
    char name[32];
    char ini_path[192];
    char dll_file[64];
    char obj_name[64];

    /* Which [<section>.<die>] overrides were applied, for the log. Empty
     * when the file has no per-die block for this part. */
    char die_suffix[24];

    /* [mode]. The two formats derived from raw_bitness are derived once,
     * here, rather than read as separate keys that could disagree. */
    v5_input_mode input_mode;
    int raw_bitness;
    v5_mipi_data_type mipi_data_type;
    v5_pixel_format pixel_format;

    /*
     * [mipi]. lane_id is board wiring, not a sensor property: the same
     * sensor is 0|1 on one layout and 0|2 on another, and the vendor's own
     * per-sensor configs disagree for exactly that reason (sc4336p is 0|2
     * where gc4023 beside it is 0|1). Getting it wrong gives a MIPI
     * receiver that never completes a line.
     */
    short lane_id[V5_MIPI_LANE_NUM];
    v5_lane_divide_mode lane_divide_mode;
    v5_mipi_data_rate mipi_data_rate;

    /* [isp_image] -- ot_isp_pub_attr's half. frame_rate is a float in the
     * vendor struct and is carried as one so no conversion happens twice. */
    float frame_rate;
    v5_bayer_format bayer;
    v5_wdr_mode wdr_mode;
    unsigned char sns_mode;

    /*
     * [i2c]. New against gen4, where the bus was implicit. On V5 the
     * sensor library takes it through pfn_set_bus_info before registration,
     * and it is passed *by value* in a one-byte union -- see
     * v5_isp_sns_commbus. -1 means "the library's own default", which is
     * what the vendor's dual-sensor configs use for the second sensor.
     */
    int i2c_dev;

    /*
     * [vi_dev] -- the VI device attribute, from the vendor's own file.
     * The sync-timing block is dead on a MIPI sensor and carried anyway,
     * for gen4's reason: skipping it would be a guess about which fields
     * the driver reads.
     */
    v5_vi_intf_mode intf_mode;
    v5_vi_work_mode work_mode;
    unsigned int component_mask[V5_VI_COMPONENT_MASK_NUM];
    v5_vi_scan_mode scan_mode;
    v5_vi_data_seq data_seq;
    v5_vi_sync_cfg sync_cfg;
    v5_vi_data_type data_type;
    int data_reverse;
    v5_data_rate data_rate;

    /*
     * The sensor's output geometry, which is also the MIPI receiver's
     * img_rect and the ISP's wnd_rect. One size, three consumers: the
     * vendor's configs repeat it three times and they are always equal.
     */
    v5_rect dev_rect;
} hisi_sensor_mode_t;

/*
 * hisi_sensor_mode_load -- fill in everything bring-up needs.
 *
 * chip_name is hisi_state_t's, e.g. "0X3516C608": it selects the per-die
 * override sections. Pass NULL or "" to take the file's defaults.
 */
int hisi_sensor_mode_load(hisi_sensor_mode_t *m, const char *sensor_name, const char *chip_name);

/*
 * hisi_sensor_obj_find -- resolve the sensor object out of an open library.
 *
 * Tries the name the mode gives, then "g_sns_<sensor>_obj", then scans the
 * library's dynamic symbol table for any g_sns_*_obj. The scan is not
 * belt-and-braces: libsns_sp2308.so exports g_sns_os02m10_obj, so a loader
 * that only derives the symbol from the file name finds nothing at all.
 *
 * On success writes the symbol it used into m->obj_name.
 */
v5_isp_sns_obj *hisi_sensor_obj_find(hisi_sensor_mode_t *m, void *handle);

/* ================================================================
 * PER-CHANNEL BOOKKEEPING
 * ================================================================ */

/*
 * One VPSS channel, which is what a raptor framesource maps onto.
 *
 * Geometry is tracked rather than read back because rvd asks for it before
 * the channel exists and after it is destroyed, and because
 * ss_mpi_vpss_get_chn_attr answers only while the group is running.
 */
typedef struct {
    bool configured;
    bool enabled;

    unsigned int width;
    unsigned int height;
    v5_frame_rate_ctrl frame_rate;

    /* Degrees the channel's output is turned, as last set through
     * fs_set_rotation. width and height above stay the caller's; what
     * leaves the channel is height x width for 90 and 270. */
    int rotation;

    /*
     * depth, the number of frames the channel queues for *userspace*.
     *
     * Zero is the streaming case and the right default: with depth 0 the
     * channel feeds its bound VENC and queues nothing. It is also why
     * fs_set_frame_depth is a real op and not bookkeeping -- rvd's
     * snapshot path raises the depth to take a picture by hand, and with
     * depth 0 get_chn_frame blocks to its timeout on a channel that is
     * otherwise working perfectly.
     */
    unsigned int depth;

    /* Set while a frame checked out through fs_get_frame is outstanding.
     * MPP requires the same descriptor back, so it is stored here and the
     * caller gets a pointer into it. */
    bool frame_held;
    v5_video_frame_info frame;

    /*
     * What this channel's output is compressed with, decided by
     * hisi_fs_compress and cached because the pool's block size has to
     * agree with it. Changing one without the other is how a channel ends
     * up attached to blocks it can never fill.
     */
    v5_compress_mode compress_mode;

    /*
     * The channel's own VB pool -- created with ss_mpi_vb_create_pool and
     * attached with ss_mpi_vpss_attach_chn_vb_pool, so the channel draws
     * blocks cut to its own frame instead of sensor-sized ones out of the
     * common pool. See hisi_fs_pool_acquire; owned is false on a channel
     * that fell back to common, which is legal and merely wasteful.
     */
    bool vb_pool_owned;
    unsigned int vb_pool;
    unsigned long long vb_pool_blk_size;

    /*
     * Channel 0 streaming through a line ring instead of whole frames
     * -- see hisi_fs_wrap in hal_framesource.c. While wrapped the channel
     * owns no pool: its one block comes from the common pool hal_init cut
     * for it. wrap_size is what was asked for, so a geometry change can
     * tell whether the ring has to be re-sized.
     */
    bool wrapped;
    unsigned int wrap_line;
    unsigned long long wrap_size;
} hisi_vpss_chn_t;

/* One [image] knob: what was asked for, and whether anything asked. */
typedef struct {
    bool asked;
    int val;
} hisi_knob_slot_t;

/*
 * One VENC channel.
 *
 * bound_fs is the VPSS channel feeding it, or -1 -- not 0, because channel
 * 0 is a real channel and "not bound" needs a value of its own.
 */
#define HISI_VENC_MAX_PACKS 8

/*
 * How many NAL units one frame can be reported as.
 *
 * Larger than the pack array on purpose. A V5 pack carries data_num
 * sub-packets described by pack_info[], so a frame that arrives as one
 * pack can still be several NALs -- that is plan risk R9, and the answer
 * is not known until a stream runs. Sizing this at packs x pack_info's
 * bound would be 64 entries for a case that may never occur; 16 covers
 * every H.264/H.265 frame this backend produces (VPS, SPS, PPS, SEI and a
 * slice, however they are grouped) and hal_encoder logs when a frame
 * exceeds it.
 */
#define HISI_VENC_MAX_NALS 16

typedef struct {
    bool created;
    bool receiving;
    int bound_fs;
    /* Where a duty-cycled MJPEG channel rebinds when it restarts.
     * enc_stop unbinds those channels -- a stopped-but-bound destination
     * queues the source's pictures without ever releasing them -- so the
     * edge to remake has to survive the unbind that cleared bound_fs.
     * -1 otherwise. */
    int idle_fs;
    /* ss_mpi_venc_get_fd's descriptor, cached because rvd polls per
     * frame, or -1. */
    int fd;

    rss_codec_t codec;
    v5_payload_type payload;
    unsigned int width;
    unsigned int height;

    /*
     * The rate-control state. Tracked rather than re-derived, because V5
     * has no per-knob setter either: enc_set_bitrate and friends are
     * read-modify-writes of the whole ot_venc_chn_attr, and rebuilding
     * the untouched half from rvd's config would lose anything set
     * through another op.
     */
    rss_rc_mode_t rc_mode;
    unsigned int bitrate;
    unsigned int gop;
    unsigned int fps_num;
    unsigned int fps_den;

    /* The rest of what the channel attribute is built from, captured at
     * create time so a reconfigure can rebuild the whole struct without
     * any field quietly reverting to a default. */
    unsigned int profile;
    unsigned int buf_size;
    int ip_qp_delta;
    int init_qp;

    /*
     * One outstanding stream per channel. MPP wants the same descriptor
     * back at release_stream, and rss_frame_t has nowhere to keep it.
     *
     * The pack array is fixed rather than sized per call from
     * query_status.cur_packs: get_stream copies into it and trusts
     * pack_cnt, so an array that can move under a caller holding NALs into
     * it is worse than one that occasionally reports fewer packs than a
     * frame had. HISI_VENC_MAX_PACKS is generous for the H.264/H.265 case
     * -- SPS, PPS, SEI and one slice -- and the log says so if a frame
     * ever exceeds it.
     */
    bool frame_held;
    v5_venc_stream stream;
    v5_venc_pack packs[HISI_VENC_MAX_PACKS];
    rss_nal_unit_t nals[HISI_VENC_MAX_NALS];
} hisi_venc_chn_t;

/* ================================================================
 * VB GEOMETRY
 *
 * The block size for one frame, in each of the three shapes this
 * pipeline puts in a VB block: uncompressed 4:2:0 semi-planar, the same
 * thing SEG_COMPACT compressed, and raw Bayer.
 *
 * Transcribed rather than approximated because an undersized VB block is
 * the classic bring-up failure on this family: ss_mpi_sys_init succeeds,
 * the pipeline builds, and VI silently delivers nothing. An *oversized*
 * one is the failure this board actually had -- a 32 MB MMZ with four
 * sensor-sized blocks in it, VI dropping 42% of frames on vb_fail -- so
 * the sizes have to be right in both directions, not merely safe.
 *
 * These reproduce /proc/umap/vb exactly at 2304x1296: raw10 3,732,480,
 * NV21 4,478,976, and 1920x1080 3,110,400 uncompressed against about
 * 2.13 MB compressed.
 *
 * align is OT_DEFAULT_ALIGN (8; ot_defines.h:45), which is what
 * ot_common_get_valid_align returns for align == 0. V5 did not change it
 * from gen4's. The formulas are transcribed again rather than shared with
 * hisi_v4 because the two generations are free to diverge and a shared
 * helper would hide it.
 * ================================================================ */

#define HISI_VB_ALIGN 8u

static inline unsigned long long hisi_vb_align_up(unsigned long long v)
{
    return ((v + HISI_VB_ALIGN - 1u) / HISI_VB_ALIGN) * HISI_VB_ALIGN;
}

/*
 * ot_common_get_uncompressed_yuv_buf_cfg (ot_buffer_detail.h:216-267),
 * the 4:2:0 semi-planar arm:
 *
 *   stride       = ALIGN_UP((width * 8 + 7) >> 3, align) = ALIGN_UP(width, 8)
 *   align_height = ALIGN_UP(height, 2)
 *   size         = stride * align_height * 3 / 2
 */
static inline unsigned long long hisi_vb_nv12_size(unsigned int width, unsigned int height)
{
    unsigned int stride = ((width + HISI_VB_ALIGN - 1u) / HISI_VB_ALIGN) * HISI_VB_ALIGN;
    unsigned int rows = (height + 1u) & ~1u;

    return (unsigned long long)stride * rows * 3u / 2u;
}

/*
 * ot_common_get_raw_buf_cfg_with_compress_ratio (ot_buffer_detail.h:98-146),
 * the OT_COMPRESS_MODE_NONE arm:
 *
 *   stride = ALIGN_UP(ALIGN_UP(width * bit_width, 8) / 8, align)
 *   size   = stride * height
 *
 * Note the height is *not* rounded to an even number here, where the YUV
 * one rounds it: raw blocks are a line-stride times a line count.
 *
 * Only the VI pipe allocates one, and only while it is offline. An online
 * pipe hands the ISP its pixels on chip and never puts a raw frame in
 * DDR, which is the single largest saving available on this part.
 */
static inline unsigned long long hisi_vb_raw_size(unsigned int width, unsigned int height,
                                                  unsigned int bit_width)
{
    unsigned int bytes = ((width * bit_width) + 7u) / 8u;
    unsigned int stride = ((bytes + HISI_VB_ALIGN - 1u) / HISI_VB_ALIGN) * HISI_VB_ALIGN;

    return (unsigned long long)stride * height;
}

/*
 * ot_common_get_compact_seg_compress_yuv_buf_cfg (ot_buffer_detail.h:291-331),
 * the 4:2:0 semi-planar arm.
 *
 * head_stride is 32 for every width (ot_common_get_yuv_head_stride returns
 * a constant on this part), the head is counted twice and then given two
 * aligned 64-byte tails, and the two planes are divided by the vendor's
 * own ratios -- OT_SEG_RATIO_8BIT_LUMA 1430 and OT_SEG_RATIO_8BIT_CHROMA
 * 1800, ot_buffer_detail.h:23-24. About 31% off an uncompressed frame.
 *
 * The multiply runs in 64 bits before the divide, as the vendor's does:
 * width * align_height * 1000 passes 2^31 at 1080p.
 */
#define HISI_VB_SEG_HEAD_STRIDE 32u
#define HISI_VB_SEG_RATIO_LUMA 1430u
#define HISI_VB_SEG_RATIO_CHROMA 1800u

/* The compression header: sized by the frame's height, not by how many
 * of its lines a buffer holds, which is why the wrap ring below wants it
 * separately from the payload. */
static inline unsigned long long hisi_vb_seg_compact_head(unsigned int height)
{
    unsigned int rows = (height + 1u) & ~1u;
    unsigned int crows = rows / 2u;
    unsigned long long head;

    head = hisi_vb_align_up((unsigned long long)HISI_VB_SEG_HEAD_STRIDE * (rows + crows)) * 2u;
    head += hisi_vb_align_up(64u) * 2u;
    return head;
}

static inline unsigned long long hisi_vb_seg_compact_main(unsigned int width, unsigned int height)
{
    unsigned int rows = (height + 1u) & ~1u;
    unsigned int crows = rows / 2u;
    unsigned long long y, c;

    y = hisi_vb_align_up((unsigned long long)width * rows * 1000ull / HISI_VB_SEG_RATIO_LUMA);
    c = hisi_vb_align_up((unsigned long long)width * crows * 1000ull / HISI_VB_SEG_RATIO_CHROMA);
    return y + c;
}

static inline unsigned long long hisi_vb_seg_compact_size(unsigned int width, unsigned int height)
{
    return hisi_vb_seg_compact_head(height) + hisi_vb_seg_compact_main(width, height);
}

/*
 * The one a caller wants: a channel's frame, in whatever it is compressed
 * with. A pool whose blocks disagree with the channel's compress_mode is
 * either wasteful or too small, and too small shows up as a channel that
 * never delivers.
 */
static inline unsigned long long hisi_vb_yuv_size(unsigned int width, unsigned int height,
                                                  v5_compress_mode compress)
{
    if (compress == V5_COMPRESS_MODE_SEG_COMPACT)
        return hisi_vb_seg_compact_size(width, height);
    return hisi_vb_nv12_size(width, height);
}

/*
 * ot_comm_get_vpss_venc_wrap_buf_size (ot_buffer.h:47-70): the chn0 ->
 * VENC ring. buf_line lines of payload, plus -- when the channel is
 * compressed -- the header for the *whole* frame, since the header
 * indexes every line whether or not its pixels are still in the ring.
 * A ring of the full height collapses to the frame.
 */
static inline unsigned long long hisi_vb_wrap_size(unsigned int width, unsigned int height,
                                                   unsigned int buf_line, v5_compress_mode compress)
{
    if (!buf_line || buf_line >= height)
        return hisi_vb_yuv_size(width, height, compress);
    if (compress == V5_COMPRESS_MODE_SEG_COMPACT)
        return hisi_vb_seg_compact_head(height) + hisi_vb_seg_compact_main(width, buf_line);
    return hisi_vb_nv12_size(width, buf_line);
}

/* ================================================================
 * BACKEND STATE
 * ================================================================ */

typedef struct {
    /*
     * Loaded vendor libraries, and the tables resolved out of them.
     * Layout order follows the gen4 header and star_state.h: vtables
     * first, then descriptors, then shadow state, then the unwind flags.
     */
    v5_mpi_libs libs;
    v5_sys_impl sys;
    v5_vb_impl vb;
    v5_vi_impl vi;
    v5_vpss_impl vpss;
    v5_venc_impl venc;
    v5_isp_impl isp;

    /*
     * The sensor library, which is not part of the MPI set: one
     * libsns_<name>.so opened by name from the mode file, and the object
     * inside it. The object is a pointer into that mapping, so it is valid
     * exactly as long as the handle.
     */
    void *snr_handle;
    v5_isp_sns_obj *snr_obj;

    /*
     * The eight ISP algorithm libraries -- libldci, libdrc, libdehaze,
     * libbnr, libacs, libir_auto, libextend_stats, libcalcflicker -- one
     * per forwarder. They are the far end of the dlopen cycle: each needs
     * symbols out of libot_mpi_isp.so, and libot_mpi_isp.so needs exactly
     * one symbol out of each. Held so they can be closed again; nothing is
     * resolved out of them except that one symbol apiece.
     */
#define HISI_ISP_ALG_LIB_NUM 8
    void *isp_alg[HISI_ISP_ALG_LIB_NUM];

    /* The algorithm library names the sensor driver registers under, as
     * filled in by pfn_register_callback and handed to ss_mpi_ae_register /
     * ss_mpi_awb_register unchanged. Kept because teardown needs them
     * again for the unregister pair. */
    v5_isp_3a_alg_lib ae_lib;
    v5_isp_3a_alg_lib awb_lib;

    /*
     * SoC identity.
     *
     * Two sources, answering two different questions, and both are kept:
     *
     *   chip_name is read from /proc/umap/sys before anything is dlopened.
     *   The open_sys module prints the part there as a banner -- "0X3516C608"
     *   on this board -- which makes it the only identification available
     *   to rss_hal_check_platform(), since rvd prints its banner before
     *   hal_init and a platform check that only works afterwards is a
     *   platform check that never runs on the build that needed it.
     *
     *   chip_id is ss_mpi_sys_get_chip_id's word, read during hal_init.
     *   It is the vendor's own answer and the one to quote in a bug report,
     *   but it needs the libraries open.
     *
     * gen4's single /dev/mem read of SCSYSID0 does not port: 0x12020EE0 is
     * a gen4 address and V5 publishes the part through MPP instead.
     */
    unsigned int chip_id;
    bool chip_id_valid;
    char chip_name[24];

    /*
     * MPP version string as reported by ss_mpi_sys_get_version, e.g.
     * "HI3516CV610_MPP_V1.0.2.0 B051 Release" -- which is what a CV608
     * reports, because one MPP build serves the family. 96 bytes on V5,
     * against gen4's 64, and not guaranteed terminated by the vendor, so
     * one spare byte here and every read of the vendor's array bounded by
     * its size.
     */
    char mpp_version[V5_MAX_VERSION_NAME_LEN + 1];

    /*
     * The sensor as named by the config.
     *
     * Identity on V5 is decided in the kernel before raptor runs:
     * load_hisilicon does `modprobe open_sys_config sensors=sns0=<name>`,
     * and that is what sets the sensor's clock and pinmux. A raptor config
     * naming a different sensor gets no clock and every I2C write NAKs, with
     * nothing in the log to say why (plan risk R7). On OpenIPC the kernel's
     * name comes from `fw_printenv sensor`; Phase 2 reads it back and warns
     * on a mismatch rather than guessing which of the two is right.
     */
    char sensor_name[32];

    /*
     * Targets for the eight ISP-cycle forwarders defined in hal_common.c.
     *
     * They live in the state rather than in file statics so the forwarders
     * and the ISP loader cannot disagree about which library is current;
     * see the ISP CYCLE block in hal_common.c for why the executable has to
     * stand in for one direction of the cycle at all. Phase 2 fills them
     * when the algorithm libraries load; before that -- and after
     * hal_deinit clears them -- every forwarder returns V5_FAILURE rather
     * than calling through NULL.
     *
     * Arities are one td_s32 vi_pipe each for the five registrars, read off
     * the libraries themselves rather than a header: none of the eight
     * appears in any public header in the 1.0.2.0 set, because the vendor
     * links these statically and never has to name the boundary. The three
     * that are not registrars have their own shapes and are the reason this
     * is a list of eight declarations and not one array: isp_ir_auto_run_once
     * and isp_be_stats_estimate take a pipe and a payload the forwarder
     * never dereferences, and calc_flicker_type is a pure computation the
     * ISP calls per frame.
     */
    int (*fn_alg_register_ldci)(int vi_pipe);
    int (*fn_alg_register_drc)(int vi_pipe);
    int (*fn_alg_register_dehaze)(int vi_pipe);
    int (*fn_alg_register_bayer_nr)(int vi_pipe);
    int (*fn_alg_register_acs)(int vi_pipe);
    int (*fn_ir_auto_run_once)(int vi_pipe, void *arg);
    int (*fn_be_stats_estimate)(int vi_pipe, void *arg);
    int (*fn_calc_flicker_type)(int vi_pipe, void *arg);

    /*
     * Unwind flags. "-1 means none" wherever the value space includes 0;
     * hal_init sets those explicitly after calloc, because framesource 0,
     * encoder channel 0 and file descriptor 0 are all real values and
     * "nothing here yet" needs a value of its own before anything closes or
     * unbinds one.
     *
     * Phase 1 has two states to unwind and no descriptors, so the list is
     * short and grows with the phases.
     */
    bool vb_inited;
    bool sys_inited;

    /*
     * The sensor mode, read once during hal_init.
     *
     * Present in both archives even though only the video one fills it
     * in. The *call* to hisi_sensor_mode_load is what has to be guarded --
     * hisi_sensor.c is in VIDEO_SRCS alone, so an unguarded call from the
     * shared hal_common.c would leave rad's link short a symbol -- but the
     * member costs the audio build nothing. Guarding the member instead
     * breaks every other video source: the Makefile compiles those through
     * the generic rule, with no HAL_MODULE_VIDEO of their own, and only
     * hal_common.c is compiled twice.
     */
    hisi_sensor_mode_t mode;

    /*
     * The VI/VPSS coupling actually in force, read back after setting it.
     *
     * Kept for gen4's reason: in a VPSS-*online* mode the two are wired in
     * hardware and ss_mpi_sys_bind must not be called for that edge.
     */
    v5_vi_vpss_mode vi_vpss_mode;

    /*
     * The ISP's 3A loop. ss_mpi_isp_run does not return while the ISP is
     * up, so it owns a thread, and teardown stops it with ss_mpi_isp_exit
     * rather than by cancelling -- a thread cancelled inside the vendor
     * library leaves its locks held and the next isp_init blocks forever.
     */
    pthread_t isp_thread;
    volatile int isp_thread_running;
    volatile int isp_thread_done;
    bool isp_thread_started;

    /* Pipeline unwind flags, in bring-up order so teardown can read the
     * list backwards. */
    bool mipi_configured;
    bool sensor_registered;
    bool ae_registered;
    bool awb_registered;
    bool isp_inited;
    bool vi_dev_enabled;
    bool vi_bound;
    bool vi_pipe_created;
    bool vi_pipe_started;
    bool vi_chn_enabled;
    /* Per-channel bookkeeping. */
    hisi_vpss_chn_t fs[HISI_VPSS_CHN_NUM];
    hisi_venc_chn_t enc[HISI_VENC_CHN_NUM];

    bool vpss_grp_created;
    bool vpss_grp_started;
    bool vi_vpss_bound;

    /*
     * Whether a VPSS channel can be given a pool of its own. Set in
     * hal_init from the symbols, and cleared for the run the first time
     * the driver answers NOT_SUPPORT -- every later channel would get the
     * same answer and log the same line.
     */
    /*
     * IQ TUNING (Phase 3).
     *
     * `tune` is bound once, lazily, and every pair in it is optional --
     * see v5_isp_tune.h for why the contract differs from `isp` above.
     * `iq_file` is settled at bring-up and the load waits for the first
     * encoded frame, so by the time it runs the ISP is demonstrably
     * running and every Get returns live state. rvd runs an encoder
     * thread per stream, so the latch is atomic and exactly one thread
     * does the work.
     */
    v5_isp_tune_impl tune;
    bool tune_resolved;
    char iq_file[192];
    volatile char iq_load_started;

    /*
     * The dynamic sections' state, and the clock behind them: one AE
     * query a second, shared by every engine, taken by whichever encoder
     * thread gets there first. See hal_dyn.c.
     */
    struct hisi_dyn_set *dyn;
    long long iso_tick_ns; /* the tick's next due time, CLOCK_MONOTONIC */
    char iso_busy;         /* one tick at a time, across encoder threads */

    /*
     * The 3DNR ladder, which is a VI pipe parameter rather than an ISP
     * module and so has entry points of its own. Bound beside `tune`,
     * both of them optional. See hal_nrx.c.
     */
    v5_nr_impl nr;
    struct hisi_nrx_set *nrx;

    /*
     * The other eight per-light sections -- AE, fps, LDCI on exposure;
     * DPC, BLC, colour sector, CA on ISO; bayer-NR's WDR half read and
     * left alone -- on the same tick. See hal_ladder.c.
     */
    struct hisi_lad_set *lad;

    /*
     * Phase 4 -- audio. One AI device, one channel, one frame in flight;
     * the codec fd is /dev/acodec, held open because volume, gain and
     * mute all go through it at runtime. gen4's arrangement, on V5's
     * structs.
     *
     * aud_owns_sys records that audio_init did the SYS attach itself
     * (rad's normal path -- it never calls hal_init). Note what it does
     * NOT drive: an exit. SYS and VB state are kernel-global and rvd may
     * be streaming in another process, so the audio archive never calls
     * ss_mpi_sys_exit -- the process's own exit is the real detach. See
     * hal_audio.c.
     */
    v5_aud_impl aud;
    bool aud_loaded;
    bool aud_owns_sys;
    bool aud_dev_enabled;
    bool aud_chn_enabled;
    bool aud_first_frame; /* log the first frame's numbers, the layout check */
    int aud_dev;
    int acodec_fd;
    int aud_rate;
    /* No cached volume/gain here on purpose: both analog controls are
     * pure /dev/acodec pass-throughs, so the getters read the codec and
     * a cache would only ever hold what the codec already knows. */
    bool aud_gain_clamped; /* the "gain exceeds the codec max" warning is once-only */
    v5_audio_frame aud_frame;
    v5_aec_frame aud_aec;
    bool aud_frame_held;
    int aud_last_err;

    /*
     * The [image] knobs. rvd sets them before the ISP runs and the tuning
     * load rewrites the same attributes on the first frame, so each is
     * remembered and re-applied around a load. The two baselines are
     * learned from the first Get after each load and deliberately
     * forgotten at every load. See hal_knob.c.
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
     * Orientation. The VI *channel's* mirror and flip, not the sensor's --
     * every sensor library on this image has a null pfn_mirror_flip -- and
     * not VPSS's, which refuses a mirror outright (0xa007800d). So it
     * turns all three streams together, which is what [image] means by
     * hflip and vflip anyway.
     */
    int mirror;
    int flip;

    bool vb_private_pools;
    /*
     * The block hal_init set aside in common pool 1 for channel 0's wrap
     * ring, 0 when there is none -- the driver cannot say how many lines
     * the ring needs, or the pool cannot be sized. Channel 0 wraps only
     * when this is nonzero, and the VI pipe's 3DNR is enabled only then
     * too: the reference frames it allocates are about what the ring
     * saves. See hisi_vb_fill_cfg and hisi_vi_enable_3dnr.
     */
    unsigned long long vb_wrap_blk;
    bool vi_3dnr_enabled;
} hisi_state_t;

static inline hisi_state_t *hisi_state(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? (hisi_state_t *)c->platform : NULL;
}

/* ================================================================
 * CHIP IDENTIFICATION
 *
 * /proc/umap/sys, which the open_sys module publishes as soon as it is
 * modprobed and long before any userspace library is opened. Its second
 * banner line is the part, padded out with dashes:
 *
 *   ----------------------------------------0X3516C608--------------...
 *
 * so the token is what sits between the two dash runs. Read as text
 * because that is what the driver offers -- there is no register accessor
 * on V5 the way HI_MPI_SYS had none on gen4, and unlike gen4 there is no
 * documented physical address to fall back to.
 *
 * The names below are what that token reads on parts raptor has met, kept
 * as strings rather than decoded into a number: the driver's own spelling
 * is the thing to compare against, and a part nobody has held still logs
 * the token somebody can look up.
 * ================================================================ */

#define HISI_UMAP_SYS_PATH "/proc/umap/sys"

#define HISI_CHIP_HI3516CV608 "0X3516C608" /* measured, 192.168.1.238 */
#define HISI_CHIP_HI3516CV610 "0X3516C610" /* the family's other die, not yet held */

/* ================================================================
 * CROSS-FILE ENTRY POINTS
 *
 * Declared unconditionally. The audio archive never calls any of them --
 * hal_common.c's calls are inside #ifdef HAL_MODULE_VIDEO and the files
 * that define them are in VIDEO_SRCS -- and a declaration nobody calls
 * costs it nothing.
 * ================================================================ */

/* hal_framesource.c -- the VPSS channels rvd calls framesources. */
int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg);
int hal_fs_destroy_channel(void *ctx, int chn);
int hal_fs_enable_channel(void *ctx, int chn);
int hal_fs_disable_channel(void *ctx, int chn);
int hal_fs_set_rotation(void *ctx, int chn, int degrees);
int hal_fs_set_frame_depth(void *ctx, int chn, int depth);
int hal_fs_get_frame_depth(void *ctx, int chn, int *depth);
int hal_fs_get_frame(void *ctx, int chn, void **frame_data, rss_frame_info_t *info);
int hal_fs_release_frame(void *ctx, int chn, void *frame_data);
void hisi_fs_release_all(hisi_state_t *st);

/* hal_isp.c */
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height);
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den);
int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den);
/* Settle which tuning file applies (bring-up), bind the tuning symbols,
 * and apply on the first encoded frame. See the head of hal_isp.c. */
void hisi_isp_resolve_iq(hisi_state_t *st);
void hisi_isp_tune_resolve(hisi_state_t *st);
void hisi_isp_note_frame(hisi_state_t *st);

/* hal_dyn.c -- the dynamic ISP sections, and the AE tick that drives them.
 * The axis helpers are the scene_auto sample's, and hal_nrx.c will share
 * them. Every entry point is safe on a state that never saw the section. */
unsigned hisi_iso_map(unsigned iso);
unsigned hisi_iso_lerp(unsigned long long mid, unsigned long long left, unsigned long long lv,
                       unsigned long long right, unsigned long long rv);
bool hisi_iso_query(hisi_state_t *st, unsigned *iso, unsigned long long *exposure);
bool hisi_dyn_key(hisi_state_t *st, const char *sect, const char *key, const char *val);
int hisi_dyn_apply(hisi_state_t *st, int *failed, char *note, size_t note_len);
void hisi_dyn_on_exposure(hisi_state_t *st, unsigned iso, unsigned long long exposure);
void hisi_dyn_tick(hisi_state_t *st);
void hisi_dyn_drc_hold(hisi_state_t *st, bool hold);
bool hisi_dyn_drc_curve(hisi_state_t *st);
void hisi_dyn_free(hisi_state_t *st);

/* hal_nrx.c -- the [static_3dnr] ladder on the VI pipe, walked off the
 * same AE tick. Every entry point is safe on a state that never saw the
 * section. */
bool hisi_nrx_key(hisi_state_t *st, const char *key, const char *val);
int hisi_nrx_apply(hisi_state_t *st, int *failed, char *note, size_t note_len);
bool hisi_nrx_armed(hisi_state_t *st);
void hisi_nrx_on_iso(hisi_state_t *st, unsigned iso);
void hisi_nrx_free(hisi_state_t *st);

/* hal_ladder.c */
bool hisi_lad_key(hisi_state_t *st, const char *sect, const char *key, const char *val);
int hisi_lad_apply(hisi_state_t *st, int *failed, char *note, size_t note_len);
bool hisi_lad_armed(hisi_state_t *st);
void hisi_lad_on_exposure(hisi_state_t *st, unsigned iso, unsigned long long exposure);
void hisi_lad_ae_hold(hisi_state_t *st, bool hold);
bool hisi_lad_ae_curve(hisi_state_t *st);
void hisi_lad_fps_base(hisi_state_t *st, float fps);
void hisi_lad_free(hisi_state_t *st);

/* hal_isp.c: the sensor rate, for rvd and the fps ladder alike. */
int hisi_isp_fps_write(hisi_state_t *st, float fps);

/* hal_common.c: the forwarder check every MPP attach runs first, which
 * hal_audio.c's audio_init is the second entry point for. */
void hisi_check_trampolines(void);

/* hal_audio.c -- the audio archive only; hal_common.c's calls are under
 * HAL_MODULE_AUDIO. */
int hal_audio_init(void *ctx, const rss_audio_config_t *cfg);
int hal_audio_deinit(void *ctx);
int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block);
int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame);
int hal_audio_set_volume(void *ctx, int dev, int chn, int vol);
int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol);
int hal_audio_set_gain(void *ctx, int dev, int chn, int gain);
int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain);
int hal_audio_set_mute(void *ctx, int dev, int chn, int mute);

/* hal_knob.c -- the [image] knobs, the exposure readback and orientation. */
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
int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);
void hisi_knob_before_load(hisi_state_t *st);
void hisi_knob_reapply(hisi_state_t *st);
/* Orientation, on the VPSS channels; see hisi_state_t.mirror. */
int hal_isp_set_hflip(void *ctx, int enable);
int hal_isp_set_vflip(void *ctx, int enable);
int hal_isp_get_hvflip(void *ctx, int *hflip, int *vflip);
int hisi_vi_apply_orien(hisi_state_t *st);

/* hal_encoder.c */
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
int hal_enc_get_fd(void *ctx, int chn);
int hal_enc_set_rc_mode(void *ctx, int chn, rss_rc_mode_t mode, uint32_t bitrate);
int hal_enc_set_bitrate(void *ctx, int chn, uint32_t bitrate);
int hal_enc_set_gop(void *ctx, int chn, uint32_t gop_length);
int hal_enc_set_fps(void *ctx, int chn, uint32_t fps_num, uint32_t fps_den);
int hal_enc_set_jpeg_qp(void *ctx, int chn, int qp);
int hal_enc_get_jpeg_qp(void *ctx, int chn, int *qp);
int hal_enc_get_channel_attr(void *ctx, int chn, rss_video_config_t *cfg);
int hal_enc_get_fps(void *ctx, int chn, uint32_t *fps_num, uint32_t *fps_den);
int hal_enc_get_avg_bitrate(void *ctx, int chn, uint32_t *bitrate);
int hal_enc_query(void *ctx, int chn, bool *busy);
void hisi_enc_release_all(hisi_state_t *st);

/* Re-derive the rate-control attribute once the bind names the source
 * channel; called from hisi_bind_vpss_venc. See hisi_enc_fill_rc. */
void hisi_enc_refresh_rc(hisi_state_t *st, int enc_chn);

/* The VPSS -> VENC edge, shared by the encoder's register path and by
 * hal_common.c's generic bind op: both express the same thing. */
int hisi_bind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn);
int hisi_unbind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn);

#endif /* HISI_V5_STATE_H */
