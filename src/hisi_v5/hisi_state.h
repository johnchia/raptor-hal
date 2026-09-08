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
#include "v5_snr.h"

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
 * The first VPSS physical channel a stream can come out of -- AND IT IS NOT
 * KNOWN YET.
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
 * Deliberately left undefined rather than set to a guess. Phase 2's first
 * commit defines it beside the measurement that produced it, and
 * hisi_vpss_phy() arrives with it.
 */

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
 * VB GEOMETRY
 *
 * The block size for one NV12 frame, which is what every pool in this
 * backend holds.
 *
 * Transcribed rather than approximated because an undersized VB block is
 * the classic bring-up failure on this family: ss_mpi_sys_init succeeds,
 * the pipeline builds, and VI silently delivers nothing.
 *
 * ot_common_get_uncompressed_yuv_buf_cfg (ot_buffer_detail.h:216-267),
 * reduced to the one case this backend needs -- 8-bit NV12, uncompressed,
 * automatic alignment:
 *
 *   align        = OT_DEFAULT_ALIGN (8; ot_defines.h:45), which is what
 *                  ot_common_get_valid_align returns for align == 0
 *   stride       = ALIGN_UP((width * 8 + 7) >> 3, align) = ALIGN_UP(width, 8)
 *   align_height = ALIGN_UP(height, 2)
 *   size         = stride * align_height * 3 / 2
 *
 * Same arithmetic as gen4's, including the alignment: V5 did not change
 * OT_DEFAULT_ALIGN. It is transcribed again rather than shared because the
 * two generations are free to diverge and a shared helper would hide it.
 * ================================================================ */

#define HISI_VB_ALIGN 8u

static inline unsigned long long hisi_vb_nv12_size(unsigned int width, unsigned int height)
{
    unsigned int stride = ((width + HISI_VB_ALIGN - 1u) / HISI_VB_ALIGN) * HISI_VB_ALIGN;
    unsigned int rows = (height + 1u) & ~1u;

    return (unsigned long long)stride * rows * 3u / 2u;
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

#ifdef HAL_MODULE_VIDEO
    /* The sensor mode, read once during hal_init. Video-only: the audio
     * archive compiles the same hal_common.c and must not reference
     * hisi_sensor.c, which is in VIDEO_SRCS alone. */
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
    bool vpss_grp_created;
    bool vpss_grp_started;
    bool vi_vpss_bound;
#endif
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

#endif /* HISI_V5_STATE_H */
