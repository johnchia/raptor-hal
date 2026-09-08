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

#define HISI_CHIP_HI3516CV608 "0X3516C608" /* measured, 192.168.1.233 */
#define HISI_CHIP_HI3516CV610 "0X3516C610" /* the family's other die, not yet held */

#endif /* HISI_V5_STATE_H */
