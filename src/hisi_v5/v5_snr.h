/*
 * hisi_v5/v5_snr.h -- the sensor library ABI, HiMPP V5.0
 *
 * Every sensor ships as its own libsns_<name>.so under /usr/lib/sensors,
 * and the whole interface is one exported struct of function pointers plus
 * the two 3A library descriptors it fills in. raptor dlopens the library,
 * finds the object, tells it which I2C bus the sensor is on, and asks it to
 * register itself with the ISP; everything after that happens inside the
 * vendor's code.
 *
 * FOUR THINGS THE BENCH FOUND, and each of them breaks a naive port:
 *
 *   1. The symbol is `g_sns_<sensor>_obj`, not gen4's `stSns<Name>Obj`.
 *      Most libraries also export a `<sensor>_get_obj` accessor, but
 *      libsns_imx307.so does not, so the object symbol is the one to
 *      resolve. And the accessor's name is not derivable anyway:
 *      libsns_sc450ai.so spells it `sc45ai_get_obj`.
 *
 *   2. **The object comes in two sizes.** gc4023, os04d10, sc431hai,
 *      sc4336p, sc450ai and sc500ai export 48 bytes -- the twelve pointers
 *      below. imx307, os02m10 and sp2308 export **44**: eleven pointers,
 *      missing the last one. The missing member is pfn_set_fast_ae and only
 *      pfn_set_fast_ae -- checked by resolving the relocations inside
 *      libsns_imx307.so's object, where standby/restart/write_reg/read_reg
 *      land at +16/+20/+32/+36, exactly the twelve-pointer offsets. So
 *      every member raptor uses is at the same place in both, and the rule
 *      is simply: never call pfn_set_fast_ae, and never copy the object by
 *      value. Both would read four bytes past the end of a short one.
 *
 *   3. **Half the pointers are NULL.** On sc500ai, os04d10 and gc4023 the
 *      slots at +12, +24 and +28 -- set_bus_ex_info, mirror_flip and
 *      set_blc_clamp -- are all null, and +44 is null on everything except
 *      os04d10. In particular *pfn_mirror_flip does not exist* on this
 *      family, which is why v5_vi.h and v5_vpss.h both note that the
 *      channel's mirror_en/flip_en is the one that works. Every call
 *      through this struct must be NULL-checked.
 *
 *   4. libsns_sp2308.so exports `g_sns_os02m10_obj` and `os02m10_get_obj`.
 *      It is an os02m10 library under another name, so a loader that
 *      derives the symbol from the *file* name finds nothing. Derive it
 *      from the sensor name the configuration gives, and fall back to
 *      scanning for a `g_sns_*_obj` symbol when that misses.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_sns_ctrl.h at
 * 1.0.2.0 B051 for ot_isp_sns_obj and its argument types, ot_common_3a.h:
 * 485-489 for ot_isp_3a_alg_lib, ot_common_isp.h:2344-2350 for
 * ot_isp_sns_commbus. Sizes 48 / 24 / 1 from a probe compiled with the
 * cv6xx cross-compiler, and the 44-byte variant from readelf on the
 * shipped libraries.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_SNR_H
#define HISI_V5_SNR_H

#include "v5_common.h"

/* Where the images put them. Not an ABI constant -- it is a path -- but it
 * is the same on every OpenIPC V5 build and belongs beside the loader. */
#define V5_SNS_LIB_DIR "/usr/lib/sensors"

/* ot_common_3a.h:485. The bound in ot_isp_3a_alg_lib; the names that go in
 * are shorter, but the struct is what it is. */
#define V5_ALG_LIB_NAME_SIZE_MAX 20

/*
 * ot_isp_3a_alg_lib (ot_common_3a.h:486-489).
 *
 * Filled in by the sensor library's pfn_register_callback and then handed
 * straight to ss_mpi_ae_register / ss_mpi_awb_register. raptor never writes
 * either field: the id is which instance of the algorithm, and the name is
 * what the vendor's own libss_mpi_ae.so answers to.
 */
typedef struct {
    int id;
    char lib_name[V5_ALG_LIB_NAME_SIZE_MAX];
} v5_isp_3a_alg_lib;

_Static_assert(sizeof(v5_isp_3a_alg_lib) == 24, "ot_isp_3a_alg_lib is 24 bytes");
_Static_assert(offsetof(v5_isp_3a_alg_lib, lib_name) == 4, "ot_isp_3a_alg_lib.lib_name at +4");

/*
 * ot_isp_sns_commbus (ot_common_isp.h:2344-2350).
 *
 * A one-byte union: an I2C adapter number, or an SSP device and chip select
 * packed four bits each. **Passed by value**, not by pointer -- see
 * pfn_set_bus_info below -- which is why the exact size matters: a struct
 * this small goes in a register, and getting the type wrong changes the
 * calling convention rather than the contents.
 */
typedef union {
    signed char i2c_dev;
    struct {
        signed char bit4_ssp_dev : 4;
        signed char bit4_ssp_cs : 4;
    } ssp_dev;
} v5_isp_sns_commbus;

_Static_assert(sizeof(v5_isp_sns_commbus) == 1, "ot_isp_sns_commbus is 1 byte");

/* ot_isp_sns_mirrorflip_type (ot_sns_ctrl.h). Kept for completeness; see
 * finding 3 above -- pfn_mirror_flip is null on every library that ships
 * with this image, so nothing calls it. */
typedef enum {
    V5_ISP_SNS_NORMAL = 0,
    V5_ISP_SNS_MIRROR = 1,
    V5_ISP_SNS_FLIP = 2,
    V5_ISP_SNS_MIRROR_FLIP = 3,
} v5_isp_sns_mirrorflip_type;

/* ot_isp_sns_blc_clamp / ot_isp_sns_fast_ae_attr (ot_sns_ctrl.h). Both are
 * a single td_bool; named so the pointers below have real types. */
typedef struct {
    int blc_clamp_en;
} v5_isp_sns_blc_clamp;

typedef struct {
    int enable;
} v5_isp_sns_fast_ae_attr;

/* ot_isp_sns_bus_ex (ot_sns_ctrl.h). Serdes addressing, for a sensor behind
 * a deserialiser. Null on everything here. */
typedef struct {
    char bus_addr;
} v5_isp_sns_bus_ex;

/*
 * ot_isp_sns_obj (ot_sns_ctrl.h).
 *
 * The order is the ABI. Offsets are called out because finding 2 above
 * depends on them: every member up to and including pfn_set_init sits at
 * the same place in the 44-byte libraries, and pfn_set_fast_ae does not
 * exist there at all.
 *
 * The two raptor actually calls are pfn_set_bus_info -- which must come
 * first, because the library talks I2C during registration -- and
 * pfn_register_callback, whose two out-parameters then go to
 * ss_mpi_ae_register and ss_mpi_awb_register.
 */
typedef struct {
    /* +0 */ int (*pfn_register_callback)(int vi_pipe, v5_isp_3a_alg_lib *ae_lib,
                                          v5_isp_3a_alg_lib *awb_lib);
    /* +4 */ int (*pfn_un_register_callback)(int vi_pipe, v5_isp_3a_alg_lib *ae_lib,
                                             v5_isp_3a_alg_lib *awb_lib);
    /* +8 */ int (*pfn_set_bus_info)(int vi_pipe, v5_isp_sns_commbus sns_bus_info);
    /* +12 */ int (*pfn_set_bus_ex_info)(int vi_pipe, v5_isp_sns_bus_ex *serdes_info);
    /* +16 */ void (*pfn_standby)(int vi_pipe);
    /* +20 */ void (*pfn_restart)(int vi_pipe);
    /* +24 */ void (*pfn_mirror_flip)(int vi_pipe, v5_isp_sns_mirrorflip_type sns_mirror_flip);
    /* +28 */ void (*pfn_set_blc_clamp)(int vi_pipe, v5_isp_sns_blc_clamp sns_blc_clamp);
    /* +32 */ int (*pfn_write_reg)(int vi_pipe, unsigned int addr, unsigned int data);
    /* +36 */ int (*pfn_read_reg)(int vi_pipe, unsigned int addr);
    /* +40 */ int (*pfn_set_init)(int vi_pipe, void *init_attr);
    /* +44 */ int (*pfn_set_fast_ae)(int vi_pipe, v5_isp_sns_fast_ae_attr *fast_ae_attr);
} v5_isp_sns_obj;

_Static_assert(sizeof(v5_isp_sns_obj) == 48, "ot_isp_sns_obj is 48 bytes (12 pointers)");
_Static_assert(offsetof(v5_isp_sns_obj, pfn_set_bus_info) == 8,
               "ot_isp_sns_obj.pfn_set_bus_info at +8");
_Static_assert(offsetof(v5_isp_sns_obj, pfn_standby) == 16, "ot_isp_sns_obj.pfn_standby at +16");
_Static_assert(offsetof(v5_isp_sns_obj, pfn_write_reg) == 32,
               "ot_isp_sns_obj.pfn_write_reg at +32");
_Static_assert(offsetof(v5_isp_sns_obj, pfn_set_init) == 40, "ot_isp_sns_obj.pfn_set_init at +40");

/*
 * The size the shorter libraries export. Anything at or past this offset
 * may not exist in the mapping, so a call through it is a read past the
 * end of the object.
 */
#define V5_SNS_OBJ_SHORT_SIZE 44

/*
 * ot_isp_sns_attr_info (ot_common_sns.h:37-39). One field, the sensor id,
 * and it is what ss_mpi_isp_sensor_unreg_callback takes on its own.
 */
typedef struct {
    int sns_id;
} v5_isp_sns_attr_info;

_Static_assert(sizeof(v5_isp_sns_attr_info) == 4, "ot_isp_sns_attr_info is 4 bytes");

#endif /* HISI_V5_SNR_H */
