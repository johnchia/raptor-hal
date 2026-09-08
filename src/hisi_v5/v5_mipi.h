/*
 * hisi_v5/v5_mipi.h -- /dev/ot_mipi_rx, HiMPP V5.0
 *
 * The one part of the V5 pipeline that is not an MPI call. MIPI receiver
 * setup is ioctls on a character device, exactly as gen4's /dev/hi_mipi
 * was, and the node name is unchanged by OpenIPC's rename of the modules --
 * measured on the board, /dev/ot_mipi_rx 218,126, which is the sample's
 * own MIPI_DEV_NAME.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_mipi_rx.h at
 * 1.0.2.0 B051, whole. This is a small header and it is transcribed nearly
 * completely, unlike its neighbours: the union in combo_dev_attr_t is sized
 * by the LVDS arm that this backend never fills, so leaving that arm out
 * would make the struct 172 bytes too short and every field the driver
 * reads past the union would be garbage. A partial transcription of a
 * union is not a smaller struct, it is a wrong one.
 *
 * THE IOCTL NUMBERS ARE CHECKED, NOT COPIED. Every _IOW below is written
 * out with its direction, magic, number and type, and then _IOC_SIZE of the
 * result is asserted against the struct this file declares. That is the
 * check the hisi-mpp-abi-audit note prescribes and it is the only one that
 * catches a struct whose size moved between MPP builds: the kernel compares
 * the whole command word, so a 200-byte attr meeting a 204-byte driver
 * fails with -EINVAL and no further explanation.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_MIPI_H
#define HISI_V5_MIPI_H

#include "v5_common.h"

#include <sys/ioctl.h>

/*
 * The device node.
 *
 * Named as a list because plan risk R8 asked whether OpenIPC's rename of
 * the modules reached the nodes, and the answer measured on the board is
 * that it did not. The fallback is kept anyway and the opener logs which
 * name answered: one line at bring-up is cheaper than the next person
 * rediscovering this.
 */
#define V5_MIPI_DEV_NAME "/dev/ot_mipi_rx"
#define V5_MIPI_DEV_NAME_ALT "/dev/mipi_rx"

/* ot_mipi_rx.h:20-27. Array bounds, so ABI. */
#define V5_MIPI_LANE_NUM 4
#define V5_LVDS_LANE_NUM 4
#define V5_WDR_VC_NUM 4
#define V5_SYNC_CODE_NUM 4

/*
 * lane_divide_mode_t (ot_mipi_rx.h:40-44), the argument to SET_HS_MODE.
 *
 * MODE_0 gives one four-lane receiver; MODE_1 splits the pins into two
 * two-lane receivers. A two-lane sensor on a board wired for MODE_0 still
 * works -- it uses lanes 0 and 1 and leaves 2 and 3 disabled through
 * lane_id -- so this backend sets MODE_0 unless a sensor INI says
 * otherwise, which is what the vendor sample does for every single-sensor
 * case.
 */
typedef enum {
    V5_LANE_DIVIDE_MODE_0 = 0,
    V5_LANE_DIVIDE_MODE_1 = 1,
} v5_lane_divide_mode;

/* input_mode_t (ot_mipi_rx.h:46-53). This backend drives MIPI; the LVDS
 * family is transcribed for the union's sake and never selected. */
typedef enum {
    V5_INPUT_MODE_MIPI = 0x0,
    V5_INPUT_MODE_SUBLVDS = 0x1,
    V5_INPUT_MODE_LVDS = 0x2,
    V5_INPUT_MODE_HISPI = 0x3,
} v5_input_mode;

/* mipi_data_rate_t (ot_mipi_rx.h:55-60). Pixels per clock out of the
 * receiver. X1 for everything this backend has met; X2 is for sensors fast
 * enough to need two. */
typedef enum {
    V5_MIPI_DATA_RATE_X1 = 0,
    V5_MIPI_DATA_RATE_X2 = 1,
} v5_mipi_data_rate;

/* data_type_t (ot_mipi_rx.h:69-79). The wire format, which the sensor INI
 * gives as a raw bit depth and hisi_sensor.c maps to one of these. */
typedef enum {
    V5_DATA_TYPE_RAW_8BIT = 0,
    V5_DATA_TYPE_RAW_10BIT = 1,
    V5_DATA_TYPE_RAW_12BIT = 2,
    V5_DATA_TYPE_RAW_14BIT = 3,
    V5_DATA_TYPE_YUV420_8BIT_NORMAL = 4,
    V5_DATA_TYPE_YUV420_8BIT_LEGACY = 5,
    V5_DATA_TYPE_YUV422_8BIT = 6,
    V5_DATA_TYPE_YUV422_PACKED = 7,
} v5_mipi_data_type;

/* mipi_wdr_mode_t (ot_mipi_rx.h:88-94). NONE for a linear sensor. */
typedef enum {
    V5_MIPI_WDR_MODE_NONE = 0x0,
    V5_MIPI_WDR_MODE_VC = 0x1,
    V5_MIPI_WDR_MODE_DT = 0x2,
    V5_MIPI_WDR_MODE_DOL = 0x3,
} v5_mipi_wdr_mode;

/* img_rect_t (ot_mipi_rx.h:62-67). Byte-identical to ot_rect but a
 * different type in the vendor's headers, so it is a different type here
 * too -- an implicit conversion that happens to work is not a reason to
 * pretend two ABIs are one. */
typedef struct {
    int x;
    int y;
    unsigned int width;
    unsigned int height;
} v5_img_rect;

_Static_assert(sizeof(v5_img_rect) == 16, "img_rect_t is 16 bytes");

/*
 * mipi_dev_attr_t (ot_mipi_rx.h:96-104).
 *
 * lane_id is *signed short* and -1 disables a lane, which is the whole
 * reason a four-element array serves a two-lane sensor: {0, 1, -1, -1}.
 * Getting the type wrong turns -1 into 65535 and the receiver waits
 * forever on a lane that is not there, with no error anywhere.
 */
typedef struct {
    v5_mipi_data_type input_data_type;
    v5_mipi_wdr_mode wdr_mode;
    short lane_id[V5_MIPI_LANE_NUM];
    union {
        short data_type[V5_WDR_VC_NUM];
    };
} v5_mipi_dev_attr;

_Static_assert(sizeof(v5_mipi_dev_attr) == 24, "mipi_dev_attr_t is 24 bytes");
_Static_assert(offsetof(v5_mipi_dev_attr, lane_id) == 8, "mipi_dev_attr_t.lane_id at +8");

/* lvds_vsync_attr_t / lvds_fid_attr_t (ot_mipi_rx.h:~120-150). Present only
 * to size lvds_dev_attr_t; never filled. */
typedef struct {
    int sync_type;
    unsigned short hblank1;
    unsigned short hblank2;
} v5_lvds_vsync_attr;

typedef struct {
    int fid_type;
    unsigned char output_fil;
} v5_lvds_fid_attr;

/*
 * lvds_dev_attr_t (ot_mipi_rx.h:~152-170), 172 bytes.
 *
 * THIS BACKEND NEVER FILLS IT. It is transcribed because it is the larger
 * arm of combo_dev_attr_t's union and therefore decides that struct's size.
 * Leaving it out would give a 40-byte combo_dev_attr_t where the driver
 * expects 200, and the _IOC_SIZE assert below is what makes that
 * impossible to ship.
 */
typedef struct {
    v5_mipi_data_type input_data_type;
    int wdr_mode;
    int sync_mode;
    v5_lvds_vsync_attr vsync_attr;
    v5_lvds_fid_attr fid_attr;
    int data_endian;
    int sync_code_endian;
    short lane_id[V5_LVDS_LANE_NUM];
    unsigned short sync_code[V5_LVDS_LANE_NUM][V5_WDR_VC_NUM][V5_SYNC_CODE_NUM];
} v5_lvds_dev_attr;

_Static_assert(sizeof(v5_lvds_dev_attr) == 172, "lvds_dev_attr_t is 172 bytes");

/* combo_dev_attr_t (ot_mipi_rx.h:172-182). */
typedef struct {
    unsigned int devno;
    v5_input_mode input_mode;
    v5_mipi_data_rate data_rate;
    v5_img_rect img_rect;
    union {
        v5_mipi_dev_attr mipi_attr;
        v5_lvds_dev_attr lvds_attr;
    };
} v5_combo_dev_attr;

_Static_assert(sizeof(v5_combo_dev_attr) == 200, "combo_dev_attr_t is 200 bytes");
_Static_assert(offsetof(v5_combo_dev_attr, img_rect) == 12, "combo_dev_attr_t.img_rect at +12");
_Static_assert(offsetof(v5_combo_dev_attr, mipi_attr) == 28, "combo_dev_attr_t.mipi_attr at +28");

/* phy_cmv_t (ot_mipi_rx.h:184-192). Not set by this backend -- the driver's
 * default suits every sensor met so far -- but the command is transcribed
 * because it is between RESET_SENSOR and SET_DEV_ATTR in the numbering and
 * a gap invites somebody to fill it wrongly. */
typedef struct {
    unsigned int devno;
    int cmv_mode;
} v5_phy_cmv;

/* ================================================================
 * IOCTLS
 *
 * ot_mipi_rx.h:196-230. OT_MIPI_IOC_MAGIC is 'm'.
 * ================================================================ */

#define V5_MIPI_IOC_MAGIC 'm'

/*
 * _IOC_SIZE, spelt out.
 *
 * musl's <sys/ioctl.h> defines _IOW but not the accessors that take a
 * command apart again -- those live in <linux/ioctl.h>, which is a kernel
 * UAPI header this backend has no other reason to include. The encoding is
 * the asm-generic one that every Linux architecture raptor targets uses:
 * 14 size bits starting at bit 16, above 8 type bits and 8 number bits. It
 * is written out here so the asserts below are compile-time constants on
 * the host as well as on the target.
 */
#define V5_IOC_SIZE(cmd) (((cmd) >> 16) & 0x3fffu)

#define V5_MIPI_SET_DEV_ATTR _IOW(V5_MIPI_IOC_MAGIC, 0x01, v5_combo_dev_attr)
#define V5_MIPI_SET_PHY_CMVMODE _IOW(V5_MIPI_IOC_MAGIC, 0x04, v5_phy_cmv)
#define V5_MIPI_RESET_SENSOR _IOW(V5_MIPI_IOC_MAGIC, 0x05, unsigned int)
#define V5_MIPI_UNRESET_SENSOR _IOW(V5_MIPI_IOC_MAGIC, 0x06, unsigned int)
#define V5_MIPI_RESET_MIPI _IOW(V5_MIPI_IOC_MAGIC, 0x07, unsigned int)
#define V5_MIPI_UNRESET_MIPI _IOW(V5_MIPI_IOC_MAGIC, 0x08, unsigned int)
#define V5_MIPI_SET_HS_MODE _IOW(V5_MIPI_IOC_MAGIC, 0x0b, v5_lane_divide_mode)
#define V5_MIPI_ENABLE_MIPI_CLOCK _IOW(V5_MIPI_IOC_MAGIC, 0x0c, unsigned int)
#define V5_MIPI_DISABLE_MIPI_CLOCK _IOW(V5_MIPI_IOC_MAGIC, 0x0d, unsigned int)
#define V5_MIPI_ENABLE_SENSOR_CLOCK _IOW(V5_MIPI_IOC_MAGIC, 0x10, unsigned int)
#define V5_MIPI_DISABLE_SENSOR_CLOCK _IOW(V5_MIPI_IOC_MAGIC, 0x11, unsigned int)

/*
 * The size checks.
 *
 * The four ioctls that carry a plain unsigned int are checked once through
 * RESET_MIPI; the two that carry a struct are checked individually, and
 * SET_DEV_ATTR is the one that matters -- 200 bytes of combo_dev_attr_t is
 * the single largest transcription in this backend and the one a new MPP
 * build is most likely to move.
 *
 * These are _Static_asserts rather than a runtime check because the value
 * is a compile-time constant on both sides: the kernel's command word came
 * out of the same macro applied to the vendor's struct, so if this file's
 * struct is the right size the words are equal. hal_common.c logs the
 * command words at bring-up anyway, which is what catches an MPP whose
 * *numbering* moved rather than its sizes.
 */
_Static_assert(V5_IOC_SIZE(V5_MIPI_SET_DEV_ATTR) == 200,
               "_IOC_SIZE(OT_MIPI_SET_DEV_ATTR) must be sizeof(combo_dev_attr_t) == 200");
_Static_assert(V5_IOC_SIZE(V5_MIPI_RESET_MIPI) == 4,
               "_IOC_SIZE(OT_MIPI_RESET_MIPI) must be sizeof(combo_dev_t) == 4");
_Static_assert(V5_IOC_SIZE(V5_MIPI_SET_HS_MODE) == 4,
               "_IOC_SIZE(OT_MIPI_SET_HS_MODE) must be sizeof(lane_divide_mode_t) == 4");
_Static_assert(V5_IOC_SIZE(V5_MIPI_SET_PHY_CMVMODE) == 8,
               "_IOC_SIZE(OT_MIPI_SET_PHY_CMVMODE) must be sizeof(phy_cmv_t) == 8");

/*
 * And the command words themselves, measured on the vendor headers with the
 * cross-compiler (tests/abi_probe_hisi5.c's method, run ad hoc for these).
 * A struct of the right size in the wrong ioctl slot passes every assert
 * above and fails on the board; these four catch it at compile time.
 */
_Static_assert(V5_MIPI_SET_DEV_ATTR == 0x40c86d01, "OT_MIPI_SET_DEV_ATTR is 0x40c86d01");
_Static_assert(V5_MIPI_SET_HS_MODE == 0x40046d0b, "OT_MIPI_SET_HS_MODE is 0x40046d0b");
_Static_assert(V5_MIPI_RESET_MIPI == 0x40046d07, "OT_MIPI_RESET_MIPI is 0x40046d07");
_Static_assert(V5_MIPI_ENABLE_SENSOR_CLOCK == 0x40046d10,
               "OT_MIPI_ENABLE_SENSOR_CLOCK is 0x40046d10");

#endif /* HISI_V5_MIPI_H */
