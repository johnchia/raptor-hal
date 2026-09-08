/*
 * hisi_v5/v5_vb.h -- ss_mpi_vb bindings, HiMPP V5.0
 *
 * A file of its own, where gen4 kept VB in v4_sys.h. The reason is not
 * tidiness: on V5 the pool machinery is where the generation actually
 * gained something, and the Phase 7 memory work has to be able to read this
 * without reading SYS. Sixteen common pools instead of gen4's sixteen that
 * nobody used past two, module common pools, extension pools, and the
 * supplement configuration that carries the JPEG and motion-data side
 * buffers. The lifecycle coupling that made gen4 share a file is still real
 * -- set_cfg -> vb_init -> sys_init, teardown exactly reversed -- and is
 * stated in v5_sys.h, which is where the sequence lives.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_vb.h at
 * the 1.0.2.0 B051 build:
 *
 *   ot_vb_remap_mode      :55-61
 *   ot_vb_pool_cfg        :71-76      48 bytes
 *   ot_vb_cfg             :78-81     776 bytes, common_pool at +8
 *   ot_vb_pool_status     :83-87
 *   ot_vb_supplement_cfg  :89-91
 *   ot_vb_pool_info       :93-99
 *   OT_VB_MAX_COMMON_POOLS :22        16
 *   OT_VB_INVALID_POOL_ID  :19        (-1U)
 *   the supplement masks   :105-108
 *
 * and OT_MAX_MMZ_NAME_LEN 32 from ot_defines.h:49.
 *
 * The two numbers worth stating rather than deriving:
 *
 *   - ot_vb_pool_cfg is **48**, not gen4's 32. td_u64 blk_size still gives
 *     the struct 8-byte alignment, but acMmzName grew from 16 characters to
 *     32. A backend that reused gen4's number would write every pool past
 *     the first into the wrong place with no error at all -- the driver
 *     reads a well-formed struct from the wrong offset.
 *   - ot_vb_cfg's common_pool is at **+8**, not +4. Same cause as gen4's:
 *     the pool struct's 8-byte alignment pads max_pool_cnt out. This is the
 *     assert that catches a member-by-member reading of the header.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_VB_H
#define HISI_V5_VB_H

#include "v5_common.h"

/* ot_common_vb.h:22. The array bound in ot_vb_cfg, so ABI rather than a
 * budget: how many raptor actually configures is hisi_vb_bringup's
 * decision in Phase 7. */
#define V5_VB_MAX_COMMON_POOLS 16

/*
 * ot_common_vb.h:19, OT_VB_INVALID_POOL_ID.
 *
 * A pool id is an unsigned handle and 0 is a real pool, so "this channel
 * has none" needs a value of its own. Note it is (-1U) here where gen4 spelt
 * the same value 0xFFFFFFFF; identical bits, and the vendor's own spelling
 * is kept so a reader diffing against the header sees no difference.
 */
#define V5_VB_INVALID_POOL ((unsigned int)-1)

/*
 * ot_vb_remap_mode (ot_common_vb.h:55-61).
 *
 * CACHED means the caller is responsible for its own cache maintenance --
 * ss_mpi_sys_flush_cache, out of libss_mpi_sysmem.so -- which is why NOCACHE
 * is what a pool nothing reads from userspace wants, and why a pool raptor
 * *does* read (the JPEG snapshot path) is the one place CACHED earns its
 * bookkeeping.
 */
typedef enum {
    V5_VB_REMAP_NONE = 0,
    V5_VB_REMAP_NOCACHE = 1,
    V5_VB_REMAP_CACHED = 2,
} v5_vb_remap_mode;

/* ot_vb_pool_cfg (ot_common_vb.h:71-76). */
typedef struct {
    unsigned long long blk_size;
    unsigned int blk_cnt;
    v5_vb_remap_mode remap_mode;
    char mmz_name[V5_MAX_MMZ_NAME_LEN];
} v5_vb_pool_cfg;

_Static_assert(sizeof(v5_vb_pool_cfg) == 48, "ot_vb_pool_cfg is 48 bytes");
_Static_assert(offsetof(v5_vb_pool_cfg, blk_cnt) == 8, "ot_vb_pool_cfg.blk_cnt at +8");
_Static_assert(offsetof(v5_vb_pool_cfg, remap_mode) == 12, "ot_vb_pool_cfg.remap_mode at +12");
_Static_assert(offsetof(v5_vb_pool_cfg, mmz_name) == 16, "ot_vb_pool_cfg.mmz_name at +16");

/* ot_vb_cfg (ot_common_vb.h:78-81). */
typedef struct {
    unsigned int max_pool_cnt;
    v5_vb_pool_cfg common_pool[V5_VB_MAX_COMMON_POOLS];
} v5_vb_cfg;

_Static_assert(sizeof(v5_vb_cfg) == 776, "ot_vb_cfg is 776 bytes");
_Static_assert(offsetof(v5_vb_cfg, common_pool) == 8, "ot_vb_cfg.common_pool at +8, not +4");

/*
 * ot_vb_supplement_cfg (ot_common_vb.h:89-91) and its masks (:105-108).
 *
 * One word of flags, set before vb_init, that tells VB to attach a side
 * buffer to every block. JPEG is the one raptor will want -- it is what
 * makes a snapshot off a YUV block possible without a second pool -- and
 * MOTION_DATA is what an IVS consumer would want and this backend has none
 * of. Named here rather than in Phase 7 because the call that takes them is
 * declared below and a bare 0x1 at the call site is unreadable.
 */
typedef struct {
    unsigned int supplement_cfg;
} v5_vb_supplement_cfg;

_Static_assert(sizeof(v5_vb_supplement_cfg) == 4, "ot_vb_supplement_cfg is 4 bytes");

#define V5_VB_SUPPLEMENT_JPEG_MASK 0x1u
#define V5_VB_SUPPLEMENT_MOTION_DATA_MASK 0x2u
#define V5_VB_SUPPLEMENT_DNG_MASK 0x4u
#define V5_VB_SUPPLEMENT_BNR_MOT_MASK 0x8u

/*
 * ot_vb_pool_info (ot_common_vb.h:93-99).
 *
 * The readback Phase 7 measures against. td_phys_addr_t is a plain
 * unsigned int on this part -- CONFIG_PHYS_ADDR_BIT_WIDTH_64 is not set for
 * hi3516cv6xx (ot_type.h:52-56) -- so the struct is 32 bytes and holds on
 * the host. If a future part in this family turns that config on, this
 * assert is what says so.
 */
typedef struct {
    unsigned int blk_cnt;
    unsigned long long blk_size;
    unsigned long long pool_size;
    unsigned int pool_phy_addr;
    v5_vb_remap_mode remap_mode;
} v5_vb_pool_info;

_Static_assert(sizeof(v5_vb_pool_info) == 32, "ot_vb_pool_info is 32 bytes");
_Static_assert(offsetof(v5_vb_pool_info, blk_size) == 8, "ot_vb_pool_info.blk_size at +8");
_Static_assert(offsetof(v5_vb_pool_info, pool_size) == 16, "ot_vb_pool_info.pool_size at +16");
_Static_assert(offsetof(v5_vb_pool_info, pool_phy_addr) == 24,
               "ot_vb_pool_info.pool_phy_addr at +24");

/* ot_vb_pool_status (ot_common_vb.h:83-87). td_bool is an enum, so four
 * bytes, not one. */
typedef struct {
    int is_common_pool;
    unsigned int blk_cnt;
    unsigned int free_blk_cnt;
} v5_vb_pool_status;

_Static_assert(sizeof(v5_vb_pool_status) == 12, "ot_vb_pool_status is 12 bytes");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    /* Lifecycle. set_cfg has to precede init and init has to precede
     * ss_mpi_sys_init; see v5_sys.h. */
    int (*fnInit)(void);
    int (*fnExit)(void);
    int (*fnSetCfg)(const v5_vb_cfg *cfg);
    int (*fnGetCfg)(v5_vb_cfg *cfg);

    /*
     * Pools created after vb_init rather than configured before it.
     *
     * The common pools are sized in hal_init, which is not told the stream
     * configuration; these are how a stage that does know its own geometry
     * gets a pool cut to it, and they are what gen4's hisi_fs_pool_acquire
     * uses. Optional -- without them every stage draws from the common
     * pools.
     *
     * create_pool returns the pool id, or V5_VB_INVALID_POOL on failure; it
     * does not follow the 0-is-success convention of its neighbours.
     */
    unsigned int (*fnCreatePool)(const v5_vb_pool_cfg *cfg);
    int (*fnDestroyPool)(unsigned int pool);

    /*
     * The supplement configuration. Optional and Phase 7's: nothing in
     * Phase 1 sets it, and a board whose VB has no supplement support is a
     * board without in-band JPEG side buffers rather than a board that
     * cannot stream.
     */
    int (*fnSetSupplementCfg)(const v5_vb_supplement_cfg *cfg);
    int (*fnGetSupplementCfg)(v5_vb_supplement_cfg *cfg);

    /* Measurement. Phase 7 reads these against /proc/umap/vb the way the
     * gen4 memory handoff read /proc/media-mem. */
    int (*fnGetPoolInfo)(unsigned int pool, v5_vb_pool_info *info);
} v5_vb_impl;

/*
 * v5_vb_load -- bind the VB entry points.
 *
 * Takes the already-open library set rather than dlopen-ing anything of its
 * own, for the reason v4_sys_load states: hisi_mpi_open() opens once and
 * each v5_<mod>_load only resolves. So this cannot return RSS_ERR_NOENT --
 * a missing library was reported where it was opened.
 *
 * Required versus optional is drawn at "can the backend come up at all".
 * The four lifecycle calls are required. Everything else is optional and
 * costs a capability rather than a failed init.
 */
static inline int v5_vb_load(v5_vb_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_vb";

    memset(lib, 0, sizeof(*lib));

    if (!(lib->fnInit = (int (*)(void))v5_symbol(mod, libs, "ss_mpi_vb_init")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnExit = (int (*)(void))v5_symbol(mod, libs, "ss_mpi_vb_exit")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnSetCfg = (int (*)(const v5_vb_cfg *))v5_symbol(mod, libs, "ss_mpi_vb_set_cfg")))
        return RSS_ERR_NOTSUP;

    if (!(lib->fnGetCfg = (int (*)(v5_vb_cfg *))v5_symbol(mod, libs, "ss_mpi_vb_get_cfg")))
        return RSS_ERR_NOTSUP;

    lib->fnCreatePool =
        (unsigned int (*)(const v5_vb_pool_cfg *))v5_symbol_opt(libs, "ss_mpi_vb_create_pool");
    lib->fnDestroyPool = (int (*)(unsigned int))v5_symbol_opt(libs, "ss_mpi_vb_destroy_pool");

    lib->fnSetSupplementCfg =
        (int (*)(const v5_vb_supplement_cfg *))v5_symbol_opt(libs, "ss_mpi_vb_set_supplement_cfg");
    lib->fnGetSupplementCfg =
        (int (*)(v5_vb_supplement_cfg *))v5_symbol_opt(libs, "ss_mpi_vb_get_supplement_cfg");

    lib->fnGetPoolInfo =
        (int (*)(unsigned int, v5_vb_pool_info *))v5_symbol_opt(libs, "ss_mpi_vb_get_pool_info");

    return RSS_OK;
}

static inline void v5_vb_unload(v5_vb_impl *lib)
{
    /* No handle of its own to drop; hisi_mpi_close owns those. Clearing the
     * table still matters: it makes a use-after-deinit a NULL check rather
     * than a call through a stale pointer into a dlclosed mapping. */
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_VB_H */
