/*
 * hisi_v4/v4_sys.h -- HI_MPI_SYS and HI_MPI_VB bindings, HiMPP V4.0
 *
 * SYS and VB share a file because they share a lifecycle: on gen4 the
 * bring-up order is VB_SetConfig -> VB_Init -> SYS_Init and teardown is its
 * exact reverse, so nothing ever wants one without the other.
 *
 * PROVENANCE. VB_CONFIG_S, VB_POOL_CONFIG_S and VI_VPSS_MODE_S were derived
 * from the Hi3516EV200 SDK V1.0.1.0 headers (mpp/include/hi_comm_vb.h,
 * hi_comm_sys.h) and cross-checked against ref/openhisilicon/include/comm_vb.h
 * (GPL v3, :69 and :76) and comm_sys.h, which agree member for member. Sizes
 * and offsets were read out of a probe compiled against the SDK headers with
 * arm-openipc-linux-musleabi-gcc:
 *
 *   VB_POOL_CONFIG_S  32 bytes, u32BlkCnt +8, enRemapMode +12, acMmzName +16
 *   VB_CONFIG_S      520 bytes, astCommPool +8
 *   VI_VPSS_MODE_S     8 bytes  (VI_MAX_PIPE_NUM == 2)
 *
 * The +8 on astCommPool is the one worth stating: u64BlkSize gives
 * VB_POOL_CONFIG_S 8-byte alignment, so the array does not start at +4 where
 * a member-by-member reading would put it.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_SYS_H
#define HISI_V4_SYS_H

#include "v4_common.h"

/* ================================================================
 * VB -- the video buffer pools
 * ================================================================ */

#define V4_VB_MAX_COMM_POOLS 16
#define V4_MAX_MMZ_NAME_LEN 16

/* VB_REMAP_MODE_E (openhisilicon comm_vb.h). CACHED means the caller is
 * responsible for its own cache maintenance, which is why NOCACHE is what a
 * pool nothing reads from userspace wants. */
typedef enum {
    V4_VB_REMAP_NONE = 0,
    V4_VB_REMAP_NOCACHE = 1,
    V4_VB_REMAP_CACHED = 2,
} v4_vb_remap_mode;

typedef struct {
    unsigned long long blk_size;
    unsigned int blk_cnt;
    v4_vb_remap_mode remap_mode;
    char mmz_name[V4_MAX_MMZ_NAME_LEN];
} v4_vb_pool_conf;

_Static_assert(sizeof(v4_vb_pool_conf) == 32, "VB_POOL_CONFIG_S is 32 bytes");
_Static_assert(offsetof(v4_vb_pool_conf, blk_cnt) == 8, "VB_POOL_CONFIG_S.u32BlkCnt at +8");
_Static_assert(offsetof(v4_vb_pool_conf, remap_mode) == 12, "VB_POOL_CONFIG_S.enRemapMode at +12");
_Static_assert(offsetof(v4_vb_pool_conf, mmz_name) == 16, "VB_POOL_CONFIG_S.acMmzName at +16");

typedef struct {
    unsigned int max_pool_cnt;
    v4_vb_pool_conf pool[V4_VB_MAX_COMM_POOLS];
} v4_vb_conf;

_Static_assert(sizeof(v4_vb_conf) == 520, "VB_CONFIG_S is 520 bytes");
_Static_assert(offsetof(v4_vb_conf, pool) == 8, "VB_CONFIG_S.astCommPool at +8, not +4");

/*
 * VB_INVALID_POOLID. A pool id is an unsigned handle and 0 is a real pool,
 * so "this channel has none" needs a value of its own; this is the SDK's.
 */
#define V4_VB_INVALID_POOL 0xFFFFFFFFu

/* ================================================================
 * SYS -- VI/VPSS coupling
 * ================================================================ */

/*
 * VI_MAX_PIPE_NUM is 2 on gen4 (VI_MAX_PHY_PIPE_NUM 2 + VI_MAX_VIR_PIPE_NUM
 * 0), and it is the array bound in VI_VPSS_MODE_S, so it is ABI here rather
 * than a topology choice. The topology choice -- that raptor drives pipe 0 --
 * is HISI_VI_PIPE in hisi_state.h.
 */
#define V4_VI_MAX_PIPE_NUM 2

/* VI_VPSS_MODE_E (openhisilicon comm_sys.h). A gen4 board ships
 * VI_OFFLINE_VPSS_ONLINE: /proc/umap/vi on a live 5 MP pipeline reports
 * "Pipe0Mode offline" under VI MODE and "online" under VPSS MODE. Phase 2
 * starts from that measured default rather than searching. */
typedef enum {
    V4_VI_OFFLINE_VPSS_OFFLINE = 0,
    V4_VI_OFFLINE_VPSS_ONLINE = 1,
    V4_VI_ONLINE_VPSS_OFFLINE = 2,
    V4_VI_ONLINE_VPSS_ONLINE = 3,
    V4_VI_PARALLEL_VPSS_OFFLINE = 4,
    V4_VI_PARALLEL_VPSS_PARALLEL = 5,
} v4_vi_vpss_mode_e;

typedef struct {
    v4_vi_vpss_mode_e mode[V4_VI_MAX_PIPE_NUM];
} v4_vi_vpss_mode;

_Static_assert(sizeof(v4_vi_vpss_mode) == 8, "VI_VPSS_MODE_S is 8 bytes");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    /* SYS lifecycle. */
    int (*fnInit)(void);
    int (*fnExit)(void);

    /* VB lifecycle. Separate module, same struct: see the file comment. */
    int (*fnVbInit)(void);
    int (*fnVbExit)(void);
    int (*fnVbSetConfig)(const v4_vb_conf *conf);
    int (*fnVbGetConfig)(v4_vb_conf *conf);

    /*
     * Pools created *after* VB_Init, rather than configured before it.
     *
     * The common pools have to be sized in hal_init, which is not told the
     * stream configuration; these are how a stage that does know its own
     * geometry gets a pool cut to it. Optional -- without them every stage
     * draws from the common pools, which is what the pipeline did before
     * hisi_fs_pool_acquire existed.
     *
     * CreatePool returns the pool id, or V4_VB_INVALID_POOL on failure; it
     * does not follow the 0-is-success convention of its neighbours.
     */
    unsigned int (*fnVbCreatePool)(const v4_vb_pool_conf *conf);
    int (*fnVbDestroyPool)(unsigned int pool);

    /* Identification. */
    int (*fnGetVersion)(v4_sys_ver *version);
    int (*fnGetChipId)(unsigned int *chip_id);

    /*
     * Media clock. rvd_frame_loop.c publishes the media-clock-to-UTC
     * mapping SEI timecodes are derived from through these; without them
     * frames still flow and timecodes silently vanish.
     *
     * Note the spelling: HI_MPI_SYS_GetCurPTS and HI_MPI_SYS_InitPTSBase
     * capitalise PTS, unlike every neighbouring name. Confirmed against the
     * board's own libmpi.so dynamic symbol table, not just the header.
     */
    int (*fnGetCurPts)(unsigned long long *pts);
    int (*fnInitPtsBase)(unsigned long long pts_base);
    int (*fnSyncPts)(unsigned long long pts);

    /* Binding, and the VI/VPSS coupling mode. Both are Phase 2 users. */
    int (*fnBind)(const v4_mpp_chn *src, const v4_mpp_chn *dst);
    int (*fnUnbind)(const v4_mpp_chn *src, const v4_mpp_chn *dst);
    int (*fnSetVIVPSSMode)(const v4_vi_vpss_mode *mode);
    int (*fnGetVIVPSSMode)(v4_vi_vpss_mode *mode);
} v4_sys_impl;

/*
 * v4_sys_load -- bind the SYS and VB entry points.
 *
 * Takes the already-open library set rather than dlopen-ing anything of its
 * own. divinus re-dlopens libmpi.so once per module and relies on the
 * loader's refcount to make that harmless; raptor has explicit state, so
 * hisi_mpi_open() opens once and each v4_<mod>_load only resolves symbols.
 * That also means this function cannot return RSS_ERR_NOENT -- a missing
 * library was reported where it was opened.
 *
 * Required versus optional is drawn at "can the backend come up at all".
 * The six lifecycle calls are required. Everything else is optional and
 * costs a published op rather than a failed init: an absent fnGetCurPts
 * means sys_get_timestamp returns RSS_ERR_NOTSUP, which is the supported
 * way to say a platform cannot do something.
 */
static inline int v4_sys_load(v4_sys_impl *lib, const v4_mpi_libs *libs)
{
    static const char mod[] = "v4_sys";

    memset(lib, 0, sizeof(*lib));

    if (!(lib->fnInit = (int (*)(void))v4_symbol(mod, libs, "HI_MPI_SYS_Init", "GK_API_SYS_Init")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnExit = (int (*)(void))v4_symbol(mod, libs, "HI_MPI_SYS_Exit", "GK_API_SYS_Exit")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnVbInit = (int (*)(void))v4_symbol(mod, libs, "HI_MPI_VB_Init", "GK_API_VB_Init")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnVbExit = (int (*)(void))v4_symbol(mod, libs, "HI_MPI_VB_Exit", "GK_API_VB_Exit")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnVbSetConfig = (int (*)(const v4_vb_conf *))v4_symbol(
              mod, libs, "HI_MPI_VB_SetConfig", "GK_API_VB_SetConfig")))
        return RSS_ERR_NOTSUP;

    lib->fnVbGetConfig =
        (int (*)(v4_vb_conf *))v4_symbol_opt(libs, "HI_MPI_VB_GetConfig", "GK_API_VB_GetConfig");

    lib->fnVbCreatePool = (unsigned int (*)(const v4_vb_pool_conf *))v4_symbol_opt(
        libs, "HI_MPI_VB_CreatePool", "GK_API_VB_CreatePool");
    lib->fnVbDestroyPool = (int (*)(unsigned int))v4_symbol_opt(libs, "HI_MPI_VB_DestroyPool",
                                                                "GK_API_VB_DestroyPool");

    lib->fnGetVersion =
        (int (*)(v4_sys_ver *))v4_symbol_opt(libs, "HI_MPI_SYS_GetVersion", "GK_API_SYS_GetVersion");
    lib->fnGetChipId =
        (int (*)(unsigned int *))v4_symbol_opt(libs, "HI_MPI_SYS_GetChipId", "GK_API_SYS_GetChipId");

    lib->fnGetCurPts = (int (*)(unsigned long long *))v4_symbol_opt(libs, "HI_MPI_SYS_GetCurPTS",
                                                                    "GK_API_SYS_GetCurPTS");
    lib->fnInitPtsBase = (int (*)(unsigned long long))v4_symbol_opt(libs, "HI_MPI_SYS_InitPTSBase",
                                                                    "GK_API_SYS_InitPTSBase");
    lib->fnSyncPts =
        (int (*)(unsigned long long))v4_symbol_opt(libs, "HI_MPI_SYS_SyncPTS", "GK_API_SYS_SyncPTS");

    lib->fnBind = (int (*)(const v4_mpp_chn *, const v4_mpp_chn *))v4_symbol_opt(
        libs, "HI_MPI_SYS_Bind", "GK_API_SYS_Bind");
    lib->fnUnbind = (int (*)(const v4_mpp_chn *, const v4_mpp_chn *))v4_symbol_opt(
        libs, "HI_MPI_SYS_UnBind", "GK_API_SYS_UnBind");

    lib->fnSetVIVPSSMode = (int (*)(const v4_vi_vpss_mode *))v4_symbol_opt(
        libs, "HI_MPI_SYS_SetVIVPSSMode", "GK_API_SYS_SetVIVPSSMode");
    lib->fnGetVIVPSSMode = (int (*)(v4_vi_vpss_mode *))v4_symbol_opt(
        libs, "HI_MPI_SYS_GetVIVPSSMode", "GK_API_SYS_GetVIVPSSMode");

    return RSS_OK;
}

static inline void v4_sys_unload(v4_sys_impl *lib)
{
    /* No handle of its own to drop; hisi_mpi_close owns those. Clearing the
     * table still matters: it makes a use-after-deinit a NULL check in
     * RSS_HAL_CALL rather than a call through a stale pointer into a
     * dlclosed mapping. */
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_SYS_H */
