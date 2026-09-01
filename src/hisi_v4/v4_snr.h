/*
 * hisi_v4/v4_snr.h -- sensor drivers and the MIPI receiver, HiMPP V4.0
 *
 * Two things that are one job. A gen4 sensor driver is a *userspace*
 * libsns_<model>.so exporting an ISP_SNS_OBJ_S vtable; the kernel side
 * (hi_mipi_rx.ko, hi_sensor_i2c.ko) is sensor-agnostic and takes its lane
 * map and RAW bit depth as ioctls on /dev/hi_mipi. So bringing a sensor up
 * means loading a library, configuring a receiver, and making the two agree
 * -- and both halves are driven from the same sensor mode INI.
 *
 * There is no sensor table here and there must not be one. The driver's
 * filename and its object symbol both come from configuration
 * (/etc/sensors/<mode>.ini, keys DllFile and Sensor_type), with a fallback
 * that scans the library's own dynamic symbol table for a stSns*Obj export
 * when the configured name misses. That is what lets a driver scavenged from
 * a firmware dump work without a rebuild, and what makes all 34 shipped
 * drivers reachable rather than the handful someone thought to list.
 *
 * PROVENANCE. ISP_SNS_OBJ_S and ISP_SNS_COMMBUS_U derived from the
 * Hi3516EV200 SDK V1.0.1.0 headers (mpp/include/hi_sns_ctrl.h,
 * hi_comm_isp.h:1792) and cross-checked against
 * ref/openhisilicon/include/sns_ctrl.h and comm_isp.h:1483 (GPL v3). The
 * MIPI types come from mpp/include/hi_mipi.h, which is a *kernel* interface
 * and therefore not in openhisilicon's userspace include set -- its
 * kernel/ tree carries the same definitions. Offsets from a compiled probe.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_SNR_H
#define HISI_V4_SNR_H

#include "v4_common.h"
#include "v4_isp.h"
#include "v4_video.h"

#include <sys/ioctl.h>

/* ================================================================
 * THE SENSOR DRIVER VTABLE
 * ================================================================ */

/*
 * ISP_SNS_COMMBUS_U -- which bus the driver should talk to the sensor on.
 *
 * One byte. It is passed to pfnSetBusInfo **by value**, not by pointer,
 * which is the only place in this backend where an aggregate crosses the ABI
 * in a register. On AAPCS a one-byte union goes in the low bits of r1, so
 * the value is the I2C adapter number and nothing else has to be right.
 */
typedef union {
    signed char i2c_dev;
    struct {
        signed char ssp_dev : 4;
        signed char ssp_cs : 4;
    } ssp_dev;
} v4_sns_commbus;

_Static_assert(sizeof(v4_sns_commbus) == 1, "ISP_SNS_COMMBUS_U is one byte");

/* ISP_SNS_MIRRORFLIP_TYPE_E. Orientation belongs to the sensor on this
 * family, as it does on SigmaStar: it is latched by the driver's own
 * register writes rather than applied downstream. */
typedef enum {
    V4_SNS_NORMAL = 0,
    V4_SNS_MIRROR = 1,
    V4_SNS_FLIP = 2,
    V4_SNS_MIRROR_FLIP = 3,
} v4_sns_mirrorflip;

/*
 * ISP_SNS_OBJ_S. Nine function pointers, 36 bytes, and the order is the ABI
 * -- the object is a symbol inside a prebuilt .so, so a member out of place
 * calls the wrong function with the right arguments.
 *
 * pfnSetInit takes an ISP_INIT_ATTR_S that raptor never builds, so it is
 * typed as a pointer to nothing. Declaring a struct in order to always pass
 * NULL through it would be transcription for its own sake.
 */
typedef struct {
    int (*pfnRegisterCallback)(int vi_pipe, v4_alg_lib *ae_lib, v4_alg_lib *awb_lib);
    int (*pfnUnRegisterCallback)(int vi_pipe, v4_alg_lib *ae_lib, v4_alg_lib *awb_lib);
    int (*pfnSetBusInfo)(int vi_pipe, v4_sns_commbus bus);
    void (*pfnStandby)(int vi_pipe);
    void (*pfnRestart)(int vi_pipe);
    void (*pfnMirrorFlip)(int vi_pipe, v4_sns_mirrorflip mirror_flip);
    int (*pfnWriteReg)(int vi_pipe, int addr, int data);
    int (*pfnReadReg)(int vi_pipe, int addr);
    int (*pfnSetInit)(int vi_pipe, void *init_attr);
} v4_sns_obj;

/* Nine pointers, and that is the portable statement of it: 36 bytes on the
 * 32-bit target, and still nine pointers on a host that builds this for a
 * test. A member added or removed changes the count either way, which is
 * what the assert is for. */
_Static_assert(sizeof(v4_sns_obj) == 9 * sizeof(void *), "ISP_SNS_OBJ_S is nine pointers");
#if V4_ABI32
_Static_assert(sizeof(v4_sns_obj) == 36, "ISP_SNS_OBJ_S is 36 bytes on the target");
#endif

/* ================================================================
 * THE MIPI RECEIVER
 * ================================================================ */

#define V4_MIPI_DEV "/dev/hi_mipi"
#define V4_MIPI_LANE_NUM 4
#define V4_LVDS_LANE_NUM 4
#define V4_WDR_VC_NUM 2
#define V4_SYNC_CODE_NUM 4

/* data_type_t */
typedef enum {
    V4_MIPI_DATA_TYPE_RAW_8BIT = 0,
    V4_MIPI_DATA_TYPE_RAW_10BIT = 1,
    V4_MIPI_DATA_TYPE_RAW_12BIT = 2,
    V4_MIPI_DATA_TYPE_RAW_14BIT = 3,
    V4_MIPI_DATA_TYPE_RAW_16BIT = 4,
} v4_mipi_data_type;

/* input_mode_t */
typedef enum {
    V4_INPUT_MODE_MIPI = 0,
    V4_INPUT_MODE_SUBLVDS = 1,
    V4_INPUT_MODE_LVDS = 2,
    V4_INPUT_MODE_HISPI = 3,
    V4_INPUT_MODE_CMOS = 4,
    V4_INPUT_MODE_BT601 = 5,
    V4_INPUT_MODE_BT656 = 6,
    V4_INPUT_MODE_BT1120 = 7,
    V4_INPUT_MODE_BYPASS = 8,
} v4_input_mode;

/* mipi_data_rate_t -- pixels per clock, not a bitrate. */
typedef enum {
    V4_MIPI_DATA_RATE_X1 = 0,
    V4_MIPI_DATA_RATE_X2 = 1,
} v4_mipi_data_rate;

/* mipi_wdr_mode_t */
typedef enum {
    V4_MIPI_WDR_MODE_NONE = 0,
    V4_MIPI_WDR_MODE_VC = 1,
    V4_MIPI_WDR_MODE_DT = 2,
    V4_MIPI_WDR_MODE_DOL = 3,
} v4_mipi_wdr_mode;

typedef struct {
    int x;
    int y;
    unsigned int width;
    unsigned int height;
} v4_img_rect;

_Static_assert(sizeof(v4_img_rect) == 16, "img_rect_t is 16 bytes");

/*
 * mipi_dev_attr_t. lane_id is signed 16-bit and -1 disables a lane, which is
 * how a two-lane sensor is expressed on four-lane silicon: {0, 1, -1, -1}.
 * The stock INIs write the list in exactly that form.
 */
typedef struct {
    v4_mipi_data_type input_data_type;
    v4_mipi_wdr_mode wdr_mode;
    short lane_id[V4_MIPI_LANE_NUM];
    short data_type[V4_WDR_VC_NUM];
} v4_mipi_dev_attr;

_Static_assert(sizeof(v4_mipi_dev_attr) == 20, "mipi_dev_attr_t is 20 bytes");

/*
 * lvds_dev_attr_t. raptor drives no LVDS sensor and builds no LVDS
 * attribute, but the union in combo_dev_attr_t is sized by this member, so
 * the struct has to exist at the right size or every MIPI ioctl passes a
 * short buffer. Carried opaquely: transcribing 108 bytes of sync codes that
 * nothing writes would add a chance to be wrong and no capability.
 */
typedef struct {
    unsigned char opaque[108];
} v4_lvds_dev_attr;

_Static_assert(sizeof(v4_lvds_dev_attr) == 108, "lvds_dev_attr_t is 108 bytes");

typedef struct {
    unsigned int devno;
    v4_input_mode input_mode;
    v4_mipi_data_rate data_rate;
    v4_img_rect img_rect;
    union {
        v4_mipi_dev_attr mipi_attr;
        v4_lvds_dev_attr lvds_attr;
    };
} v4_combo_dev_attr;

_Static_assert(sizeof(v4_combo_dev_attr) == 136, "combo_dev_attr_t is 136 bytes");
_Static_assert(offsetof(v4_combo_dev_attr, img_rect) == 12, "img_rect at +12");
_Static_assert(offsetof(v4_combo_dev_attr, mipi_attr) == 28, "the attr union at +28");

/*
 * The ioctl numbers, built the way the driver builds them rather than
 * copied. _IOW encodes the payload size, so writing them out by hand would
 * decouple the number from the struct it names -- and the asserts below
 * check the result against the values a probe read out of the SDK header,
 * which is what makes the reconstruction worth more than the constants.
 */
#define V4_MIPI_IOC_MAGIC 'm'
#define V4_MIPI_SET_DEV_ATTR _IOW(V4_MIPI_IOC_MAGIC, 0x01, v4_combo_dev_attr)
#define V4_MIPI_RESET_SENSOR _IOW(V4_MIPI_IOC_MAGIC, 0x05, unsigned int)
#define V4_MIPI_UNRESET_SENSOR _IOW(V4_MIPI_IOC_MAGIC, 0x06, unsigned int)
#define V4_MIPI_RESET_MIPI _IOW(V4_MIPI_IOC_MAGIC, 0x07, unsigned int)
#define V4_MIPI_UNRESET_MIPI _IOW(V4_MIPI_IOC_MAGIC, 0x08, unsigned int)
#define V4_MIPI_SET_HS_MODE _IOW(V4_MIPI_IOC_MAGIC, 0x0b, unsigned int)
#define V4_MIPI_ENABLE_MIPI_CLOCK _IOW(V4_MIPI_IOC_MAGIC, 0x0c, unsigned int)
#define V4_MIPI_DISABLE_MIPI_CLOCK _IOW(V4_MIPI_IOC_MAGIC, 0x0d, unsigned int)
#define V4_MIPI_ENABLE_SENSOR_CLOCK _IOW(V4_MIPI_IOC_MAGIC, 0x10, unsigned int)
#define V4_MIPI_DISABLE_SENSOR_CLOCK _IOW(V4_MIPI_IOC_MAGIC, 0x11, unsigned int)

_Static_assert(V4_MIPI_SET_DEV_ATTR == 0x40886d01u, "SET_DEV_ATTR matches the SDK header");
_Static_assert(V4_MIPI_RESET_SENSOR == 0x40046d05u, "RESET_SENSOR matches the SDK header");
_Static_assert(V4_MIPI_UNRESET_MIPI == 0x40046d08u, "UNRESET_MIPI matches the SDK header");
_Static_assert(V4_MIPI_ENABLE_SENSOR_CLOCK == 0x40046d10u, "ENABLE_SENSOR_CLOCK matches");

/* lane_divide_mode_t. One value on gen4; named so the SET_HS_MODE call
 * reads as a choice rather than a magic zero. */
#define V4_LANE_DIVIDE_MODE_0 0

/* ================================================================
 * THE SENSOR LIBRARY
 * ================================================================ */

/*
 * Where a sensor driver may live.
 *
 * The empty first entry means "as given", so an absolute path or a name the
 * loader can already find is tried first and the rest are a fallback.
 *
 * /usr/lib/sensors is the one that matters and the reason this list exists
 * at all: that is where OpenIPC ships all 34 drivers, and it is on no
 * loader search path -- the board has no ld-musl-arm.path and majestic
 * carries no RPATH, so majestic must be composing the directory itself.
 * A bare dlopen("libsns_imx335.so") fails on a board where the driver is
 * plainly present, which is a confusing way to be told the path is wrong.
 */
#define V4_SNR_DIRS                                                                                    {                                                                                                      "", "/usr/lib/sensors/", "/usr/lib/", "/lib/"                                                  }

typedef struct {
    void *handle;
    v4_sns_obj *obj;
    /* The symbol the object was found under, for the log line. Borrowed
     * from the caller's configuration or from the scan; never freed. */
    char obj_name[64];
    /* The path dlopen actually accepted, which is what the ELF scan reads
     * and what the log names. */
    char path[192];
} v4_snr_impl;

/*
 * v4_snr_scan_obj_name -- find a stSns*Obj export by reading the library.
 *
 * The fallback for a driver whose object symbol is not what the INI says,
 * or whose INI is absent entirely. Reads the ELF's .dynsym directly rather
 * than guessing names, because the naming is only a convention:
 * stSnsImx335Obj, stSnsGc4653Obj, stSnsSc3335Obj all follow it and nothing
 * enforces it.
 *
 * Declared here and defined in hisi_sensor.c -- it needs an ELF reader,
 * which is more than belongs in a header. It searches V4_SNR_DIRS too, so
 * a caller that passes a bare name gets the same resolution dlopen did.
 */
int v4_snr_scan_obj_name(const char *path, char *out, size_t out_len);

/*
 * v4_snr_load -- dlopen one sensor driver and find its object.
 *
 * `name` is the library file name from the INI's DllFile key
 * (e.g. "libsns_imx335.so"); `obj_name` is its Sensor_type key
 * (e.g. "stSnsImx335Obj") or NULL to go straight to the scan.
 *
 * RTLD_GLOBAL, and last of everything: the driver calls back into libisp.so
 * and libmpi.so, and eight of the 34 shipped drivers additionally call the
 * GK_API_* forwarders the executable exports. Loading it before the ISP
 * stack would leave those references unresolvable.
 */
static inline int v4_snr_load(v4_snr_impl *lib, const char *name, const char *obj_name)
{
    static const char mod[] = "v4_snr";
    static const char *dirs[] = V4_SNR_DIRS;
    char found[64];
    size_t d;

    memset(lib, 0, sizeof(*lib));

    if (!name || !name[0]) {
        HAL_LOG_ERR("%s: no sensor library configured", mod);
        return RSS_ERR_INVAL;
    }

    for (d = 0; d < sizeof(dirs) / sizeof(dirs[0]); d++) {
        /* An absolute name is what it is; do not prefix it. */
        if (name[0] == '/' && d > 0)
            break;
        snprintf(lib->path, sizeof(lib->path), "%s%s", dirs[d], name);
        if ((lib->handle = dlopen(lib->path, RTLD_LAZY | RTLD_GLOBAL)))
            break;
    }

    if (!lib->handle) {
        HAL_LOG_ERR("%s: %s not found in /usr/lib/sensors, /usr/lib or /lib: %s", mod, name,
                    dlerror());
        lib->path[0] = '\0';
        return RSS_ERR_NOENT;
    }

    if (obj_name && obj_name[0])
        lib->obj = (v4_sns_obj *)dlsym(lib->handle, obj_name);

    if (lib->obj) {
        snprintf(lib->obj_name, sizeof(lib->obj_name), "%s", obj_name);
    } else if (v4_snr_scan_obj_name(lib->path, found, sizeof(found)) == RSS_OK &&
               (lib->obj = (v4_sns_obj *)dlsym(lib->handle, found))) {
        snprintf(lib->obj_name, sizeof(lib->obj_name), "%s", found);
        HAL_LOG_WARN("%s: %s has no '%s'; using '%s' found by scanning its symbol table", mod,
                     lib->path, obj_name ? obj_name : "(none configured)", found);
    } else {
        HAL_LOG_ERR("%s: %s exports no sensor object (looked for '%s', then scanned)", mod,
                    lib->path, obj_name ? obj_name : "(none configured)");
        dlclose(lib->handle);
        lib->handle = NULL;
        return RSS_ERR_NOTSUP;
    }

    /* pfnRegisterCallback and pfnSetBusInfo are what bring-up calls; a
     * driver missing either is not usable, and finding out here beats
     * finding out through a NULL call three steps later. */
    if (!lib->obj->pfnRegisterCallback || !lib->obj->pfnSetBusInfo) {
        HAL_LOG_ERR("%s: %s: %s has no register/bus entry points", mod, lib->path,
                    lib->obj_name);
        dlclose(lib->handle);
        lib->handle = NULL;
        lib->obj = NULL;
        return RSS_ERR_NOTSUP;
    }

    HAL_LOG_INFO("%s: %s -> %s", mod, lib->path, lib->obj_name);
    return RSS_OK;
}

static inline void v4_snr_unload(v4_snr_impl *lib)
{
    if (lib->handle)
        dlclose(lib->handle);
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_SNR_H */
