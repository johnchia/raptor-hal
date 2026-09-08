/*
 * hisi_v5/v5_sys.h -- ss_mpi_sys bindings, HiMPP V5.0
 *
 * SYS on V5 is three libraries, not one. libss_mpi.so carries the
 * lifecycle, the identification calls, the media clock and the VI/VPSS
 * coupling; libss_mpi_sysbind.so carries bind/unbind; libss_mpi_sysmem.so
 * carries mmap, the MMZ allocator and the cache maintenance. All three are
 * "SYS" to a caller and all three resolve through the same search list, so
 * they are one op table here -- the split is the vendor's packaging
 * decision and not something raptor's call sites should have to know.
 *
 * VB is next door in v5_vb.h, unlike gen4 where the two shared a file. The
 * lifecycle coupling that motivated that is still exactly true and is
 * stated here because this is the file that owns the sequence:
 *
 *   bring-up   vb_set_cfg -> vb_init -> sys_init
 *   teardown   sys_exit -> vb_exit
 *
 * ss_mpi_sys_init on a system whose VB was never initialised comes back
 * OT_ERR_SYS_NOT_READY, and ss_mpi_vb_exit with any module still holding a
 * block comes back OT_ERR_VB_BUSY. Neither is a diagnostic worth
 * rediscovering.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ at 1.0.2.0 B051:
 *
 *   ot_vi_vpss_mode_type   ot_common_sys.h:117-126
 *   ot_vi_vpss_mode        ot_common_sys.h:128-130    16 bytes
 *   ot_3dnr_pos_type       ot_common_video.h:1465-1469
 *   ot_mpp_version         ot_common.h:147-149        96 bytes  (v5_common.h)
 *   ot_mpp_chn             ot_common.h:295-299        12 bytes  (v5_common.h)
 *   OT_VI_MAX_PIPE_NUM     ot_defines.h:238-241       4
 *
 * The entry-point names were taken from `readelf --dyn-syms` on the board's
 * own libraries rather than from ss_mpi_sys.h, because the header ships in
 * the SDK drop and the board ships the library, and only one of those is
 * the ABI. Every name below is present in the 1.0.2.0 B051 export list.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_SYS_H
#define HISI_V5_SYS_H

#include "v5_common.h"

/* ================================================================
 * VI / VPSS COUPLING
 * ================================================================ */

/*
 * OT_VI_MAX_PIPE_NUM is 4 on this part -- OT_VI_MAX_PHYS_PIPE_NUM 2 plus
 * OT_VI_MAX_VIRT_PIPE_NUM 2 (ot_defines.h:238-241) -- where gen4's was 2
 * with no virtual pipes at all. It is the array bound in ot_vi_vpss_mode,
 * so it is ABI here rather than a topology choice, and it is why the struct
 * is 16 bytes where gen4's VI_VPSS_MODE_S was 8. The topology choice --
 * that raptor drives pipe 0 -- is HISI_VI_PIPE in hisi_state.h.
 */
#define V5_VI_MAX_PIPE_NUM 4
#define V5_VI_MAX_PHYS_PIPE_NUM 2

/*
 * ot_vi_vpss_mode_type (ot_common_sys.h:117-126). Same six enumerators in
 * the same order as gen4's VI_VPSS_MODE_E, which is worth stating only
 * because it makes the *difference* legible: what changed on V5 is not the
 * modes but that setting them became mandatory. gen4 logged the mode it
 * found; V5's ss_mpi_sys_set_vi_vpss_mode has to be called before any pipe
 * exists, and the sample does so unconditionally
 * (sample_comm_vi.c:851 in the 1.0.1.0 drop).
 *
 * Which mode this board wants for 4M@30 with two streams on 32 MiB of MMZ
 * is a measurement, not a header fact (plan risk R5). It is made in Phase 2
 * and revisited in Phase 7 against the FMU wrap buffers.
 */
typedef enum {
    V5_VI_OFFLINE_VPSS_OFFLINE = 0,
    V5_VI_OFFLINE_VPSS_ONLINE = 1,
    V5_VI_ONLINE_VPSS_OFFLINE = 2,
    V5_VI_ONLINE_VPSS_ONLINE = 3,
    V5_VI_PARALLEL_VPSS_OFFLINE = 4,
    V5_VI_PARALLEL_VPSS_PARALLEL = 5,
} v5_vi_vpss_mode_type;

typedef struct {
    v5_vi_vpss_mode_type mode[V5_VI_MAX_PIPE_NUM];
} v5_vi_vpss_mode;

_Static_assert(sizeof(v5_vi_vpss_mode) == 16, "ot_vi_vpss_mode is 16 bytes");

/*
 * ot_3dnr_pos_type (ot_common_video.h:1465-1469). New on V5 and it decides
 * which module owns the noise reduction ladder Phase 3 writes: with
 * OT_3DNR_POS_VI the parameters go through ss_mpi_vi_set_pipe_3dnr_param,
 * with OT_3DNR_POS_VPSS through ss_mpi_vpss_set_grp_3dnr_param. gen4 had
 * only the VPSS half and hal_nrx.c assumes it; the port has to read this
 * back before deciding which call to make.
 */
typedef enum {
    V5_3DNR_POS_VI = 0,
    V5_3DNR_POS_VPSS = 1,
} v5_3dnr_pos_type;

/*
 * ot_vpss_venc_wrap_param (ot_common_sys.h:153-163): what
 * ss_mpi_sys_get_vpss_venc_wrap_buf_line wants to know before it says how
 * many lines the VPSS chn0 -> VENC ring has to hold. full_lines_std is the
 * sensor's VTS, blanking included. Measured on the CV608 (MPP 1.0.2.0
 * B051): with all_online false the answer is 128 -- the attribute's own
 * floor -- for every stream size, frame rate and VTS tried, so the VTS
 * this backend passes is the sensor height, and the call is still made
 * because the number is the driver's to change.
 */
typedef struct {
    int all_online; /* td_bool: VI online and VPSS online */
    unsigned int frame_rate;
    unsigned int full_lines_std;
    v5_size large_stream_size;
    v5_size small_stream_size;
} v5_vpss_venc_wrap_param;

_Static_assert(sizeof(v5_vpss_venc_wrap_param) == 28, "ot_vpss_venc_wrap_param is 28 bytes");
_Static_assert(offsetof(v5_vpss_venc_wrap_param, large_stream_size) == 12,
               "ot_vpss_venc_wrap_param.large_stream_size at +12");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    /* Lifecycle. VB comes first; see the file comment. */
    int (*fnInit)(void);
    int (*fnExit)(void);

    /*
     * Identification.
     *
     * get_version is what rss_hal_get_imp_version answers from, and the
     * string it fills is 96 bytes and not guaranteed terminated. On this
     * board it reads "HI3516CV610_MPP_V1.0.2.0 B051 Release" -- on a CV608
     * die, which is the whole reason HAL_PLATFORM_NAME names the ABI.
     *
     * get_chip_id is the runtime part check. Unlike gen4 there is no
     * /dev/mem address to read: SCSYSID0 is a gen4 address and V5 publishes
     * the part through MPP instead. That makes the answer available only
     * after the libraries are open -- though, measured on the board, not
     * only after ss_mpi_sys_init, which is why hal_init can call it early.
     * The check rvd needs runs *before* any of that, so hal_common.c also
     * reads /proc/umap/sys; see hisi_read_chip_name() there for which
     * source answers which question.
     */
    int (*fnGetVersion)(v5_mpp_version *version);
    int (*fnGetChipId)(unsigned int *chip_id);

    /*
     * Media clock. rvd_frame_loop.c publishes the media-clock-to-UTC
     * mapping SEI timecodes are derived from through these; without them
     * frames still flow and timecodes silently vanish.
     *
     * The V5 spelling is ordinary snake_case throughout -- ss_mpi_sys_get_cur_pts,
     * not gen4's HI_MPI_SYS_GetCurPTS with its shouted PTS. One fewer thing
     * to get wrong, and confirmed against the board's export list anyway.
     */
    int (*fnGetCurPts)(unsigned long long *pts);
    int (*fnInitPtsBase)(unsigned long long pts_base);
    int (*fnSyncPts)(unsigned long long pts);

    /*
     * Binding, from libss_mpi_sysbind.so. Optional in the loader's sense
     * only: nothing in Phase 1 binds anything, and a build that reaches
     * Phase 2 without these fails at the first bind with a name in the log
     * rather than at init with none.
     */
    int (*fnBind)(const v5_mpp_chn *src, const v5_mpp_chn *dst);
    int (*fnUnbind)(const v5_mpp_chn *src, const v5_mpp_chn *dst);

    /* The VI/VPSS coupling and the 3DNR position. Phase 2 and Phase 3
     * users respectively; both must be set before the stage they configure
     * exists. */
    int (*fnSetViVpssMode)(const v5_vi_vpss_mode *mode);
    int (*fnGetViVpssMode)(v5_vi_vpss_mode *mode);
    int (*fnSet3dnrPos)(v5_3dnr_pos_type pos);
    int (*fnGet3dnrPos)(v5_3dnr_pos_type *pos);
    /* The chn0 wrap ring's line count. A pure computation: it answers
     * before ss_mpi_sys_init, which is when the VB pool that holds the
     * ring has to be sized. Optional; without it there is no wrap. */
    int (*fnGetVpssVencWrapBufLine)(const v5_vpss_venc_wrap_param *param, unsigned int *buf_line);

    /*
     * The memory surface, from libss_mpi_sysmem.so.
     *
     * mmap/munmap rather than the process's own: these map MMZ physical
     * addresses, and gen4 needed a shim here only because its libmpi.so was
     * a uClibc build calling libc's mmap with a 32-bit off_t. V5's are musl
     * builds and go through the vendor's own entry point, so there is
     * nothing to bridge -- see the WHAT DID NOT PORT block in v5_common.h.
     *
     * mmz_alloc / mmz_free are the RGN bitmap and snapshot allocator
     * (Phase 5, Phase 2); flush_cache is what a CACHED pool obliges the
     * reader to call. All optional, all bound here because the library they
     * live in is already open and binding nothing out of it would be
     * stranger than binding these.
     */
    void *(*fnMmap)(unsigned int phys_addr, unsigned int size);
    int (*fnMunmap)(void *virt_addr);
    void *(*fnMmapCached)(unsigned int phys_addr, unsigned int size);
    int (*fnFlushCache)(unsigned int phys_addr, void *virt_addr, unsigned int size);
    int (*fnMmzAlloc)(unsigned int *phys_addr, void **virt_addr, const char *mmb, const char *zone,
                      unsigned int len);
    int (*fnMmzFree)(unsigned int phys_addr, void *virt_addr);
} v5_sys_impl;

/*
 * v5_sys_load -- bind the SYS entry points.
 *
 * Required versus optional is drawn at "can the backend come up at all",
 * as in v5_vb_load. Only init and exit are required: an absent
 * fnGetCurPts means sys_get_timestamp answers RSS_ERR_NOTSUP, which is the
 * supported way to say a platform cannot do something, and an absent bind
 * fails in Phase 2 where the caller can name what it wanted.
 *
 * fnGetVersion is deliberately in the optional half even though hal_init
 * logs from it. A board that cannot report its MPP version is a board
 * raptor should still start on, saying so.
 */
static inline int v5_sys_load(v5_sys_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_sys";

    memset(lib, 0, sizeof(*lib));

    if (!(lib->fnInit = (int (*)(void))v5_symbol(mod, libs, "ss_mpi_sys_init")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnExit = (int (*)(void))v5_symbol(mod, libs, "ss_mpi_sys_exit")))
        return RSS_ERR_NOTSUP;

    lib->fnGetVersion = (int (*)(v5_mpp_version *))v5_symbol_opt(libs, "ss_mpi_sys_get_version");
    lib->fnGetChipId = (int (*)(unsigned int *))v5_symbol_opt(libs, "ss_mpi_sys_get_chip_id");

    lib->fnGetCurPts = (int (*)(unsigned long long *))v5_symbol_opt(libs, "ss_mpi_sys_get_cur_pts");
    lib->fnInitPtsBase =
        (int (*)(unsigned long long))v5_symbol_opt(libs, "ss_mpi_sys_init_pts_base");
    lib->fnSyncPts = (int (*)(unsigned long long))v5_symbol_opt(libs, "ss_mpi_sys_sync_pts");

    lib->fnBind =
        (int (*)(const v5_mpp_chn *, const v5_mpp_chn *))v5_symbol_opt(libs, "ss_mpi_sys_bind");
    lib->fnUnbind =
        (int (*)(const v5_mpp_chn *, const v5_mpp_chn *))v5_symbol_opt(libs, "ss_mpi_sys_unbind");

    lib->fnSetViVpssMode =
        (int (*)(const v5_vi_vpss_mode *))v5_symbol_opt(libs, "ss_mpi_sys_set_vi_vpss_mode");
    lib->fnGetViVpssMode =
        (int (*)(v5_vi_vpss_mode *))v5_symbol_opt(libs, "ss_mpi_sys_get_vi_vpss_mode");
    lib->fnSet3dnrPos = (int (*)(v5_3dnr_pos_type))v5_symbol_opt(libs, "ss_mpi_sys_set_3dnr_pos");
    lib->fnGet3dnrPos = (int (*)(v5_3dnr_pos_type *))v5_symbol_opt(libs, "ss_mpi_sys_get_3dnr_pos");
    lib->fnGetVpssVencWrapBufLine =
        (int (*)(const v5_vpss_venc_wrap_param *, unsigned int *))v5_symbol_opt(
            libs, "ss_mpi_sys_get_vpss_venc_wrap_buf_line");

    lib->fnMmap = (void *(*)(unsigned int, unsigned int))v5_symbol_opt(libs, "ss_mpi_sys_mmap");
    lib->fnMunmap = (int (*)(void *))v5_symbol_opt(libs, "ss_mpi_sys_munmap");
    lib->fnMmapCached =
        (void *(*)(unsigned int, unsigned int))v5_symbol_opt(libs, "ss_mpi_sys_mmap_cached");
    lib->fnFlushCache =
        (int (*)(unsigned int, void *, unsigned int))v5_symbol_opt(libs, "ss_mpi_sys_flush_cache");
    lib->fnMmzAlloc = (int (*)(unsigned int *, void **, const char *, const char *,
                               unsigned int))v5_symbol_opt(libs, "ss_mpi_sys_mmz_alloc");
    lib->fnMmzFree = (int (*)(unsigned int, void *))v5_symbol_opt(libs, "ss_mpi_sys_mmz_free");

    return RSS_OK;
}

static inline void v5_sys_unload(v5_sys_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_SYS_H */
