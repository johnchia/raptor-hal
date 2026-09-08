/*
 * hisi_v5/v5_isp.h -- ss_mpi_isp bindings, the pipeline half, HiMPP V5.0
 *
 * Everything Phase 2 needs to get an image out of a sensor and nothing
 * more: the public attribute, the bring-up sequence, the 3A registration
 * handshake, and the exposure readback the OSD and the night-mode logic
 * ask for. The tuning surface -- sharpness, NR, DRC, gamma, CSC, the whole
 * .ini-driven side -- is Phase 3's and belongs in a file of its own.
 *
 * THE BRING-UP SEQUENCE, and it is order-sensitive in a way nothing in the
 * header says. From the vendor's own samples:
 *
 *   1. sensor library: pfn_set_bus_info, then pfn_register_callback, which
 *      fills in the two ot_isp_3a_alg_lib descriptors (see v5_snr.h)
 *   2. ss_mpi_ae_register  / ss_mpi_awb_register with those descriptors
 *   3. ss_mpi_isp_mem_init
 *   4. ss_mpi_isp_set_pub_attr
 *   5. ss_mpi_isp_init
 *   6. ss_mpi_isp_run -- **and this call does not return.** It is the ISP's
 *      own thread, so it must be started on a thread of its own, and
 *      ss_mpi_isp_exit is what makes it return.
 *
 * Steps 1 and 2 before 3: mem_init allocates against the sensor's
 * declared geometry, which it learns through the callbacks. Registering
 * after mem_init gets an ISP that comes up and produces a green frame.
 *
 * WHERE THE SYMBOLS LIVE, which is not where the names suggest:
 *
 *   libss_mpi_isp.so   a facade -- ss_mpi_isp_init is four bytes of code,
 *                      a branch into libot_mpi_isp.so. Opened for the
 *                      ss_mpi_ spellings.
 *   libot_mpi_isp.so   the implementation, and the far end of the dlopen
 *                      cycle the executable's eight forwarders break.
 *   libss_mpi_ae.so    ss_mpi_ae_register **and
 *                      ss_mpi_isp_query_exposure_info**, which is in ae
 *                      rather than isp and is the one symbol in this file
 *                      a reader would look for in the wrong library.
 *   libss_mpi_awb.so   ss_mpi_awb_register.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_isp.h and
 * ss_mpi_isp.h / ss_mpi_ae.h / ss_mpi_awb.h at 1.0.2.0 B051. Sizes from a
 * probe compiled against them with the cv6xx cross-compiler:
 *
 *   ot_isp_pub_attr    68   wnd_rect +0, sns_size +16, frame_rate +24
 *                           (float), bayer_format +28, wdr_mode +32,
 *                           sns_mode +36
 *   ot_isp_exp_info  5484   a_gain +16, d_gain +20, exposure +36,
 *                           iso +4160
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_ISP_H
#define HISI_V5_ISP_H

#include "v5_common.h"
#include "v5_video.h"
#include "v5_snr.h"

/* ot_common_isp.h. The bin count of ae_hist1024_value, and the reason
 * ot_isp_exp_info is five kilobytes rather than a hundred bytes. */
#define V5_ISP_HIST_NUM 1024

/*
 * ot_mipi_crop_attr (ot_common_isp.h:~288).
 *
 * A second crop, ahead of wnd_rect, applied in the MIPI receiver rather
 * than in the ISP. Not something raptor sets -- wnd_rect is the crop that
 * matters -- but it is the tail of pub_attr and a hole here moves nothing
 * because it is last.
 */
typedef struct {
    int enable;
    v5_rect mipi_crop_offset;
} v5_mipi_crop_attr;

_Static_assert(sizeof(v5_mipi_crop_attr) == 20, "ot_mipi_crop_attr is 20 bytes");

/*
 * ot_isp_pub_attr (ot_common_isp.h:288-300).
 *
 * wnd_rect is the crop *out of the sensor's output*, in sensor pixels, and
 * sns_size is that output. Setting wnd_rect to anything but {0, 0,
 * sns_size} is a crop; setting sns_size to anything but what the sensor
 * actually emits is a misconfigured ISP that produces a skewed or black
 * frame with no error from set_pub_attr.
 *
 * frame_rate is a **float**, which is why this struct cannot be built out
 * of the u32 the rest of the MPI uses. It is the sensor's frame rate, not
 * a rate limit: the ISP uses it to convert exposure lines to time.
 *
 * sns_mode selects between a sensor library's built-in modes and is the
 * one field whose meaning is entirely the sensor's; 0 is the default mode
 * on every library that ships with this image.
 */
typedef struct {
    v5_rect wnd_rect;
    v5_size sns_size;
    float frame_rate;
    v5_bayer_format bayer_format;
    v5_wdr_mode wdr_mode;
    unsigned char sns_mode;
    int sns_flip_en;
    int sns_mirror_en;
    v5_mipi_crop_attr mipi_crop_attr;
} v5_isp_pub_attr;

_Static_assert(sizeof(v5_isp_pub_attr) == 68, "ot_isp_pub_attr is 68 bytes");
_Static_assert(offsetof(v5_isp_pub_attr, sns_size) == 16, "ot_isp_pub_attr.sns_size at +16");
_Static_assert(offsetof(v5_isp_pub_attr, frame_rate) == 24, "ot_isp_pub_attr.frame_rate at +24");
_Static_assert(offsetof(v5_isp_pub_attr, bayer_format) == 28,
               "ot_isp_pub_attr.bayer_format at +28");
_Static_assert(offsetof(v5_isp_pub_attr, wdr_mode) == 32, "ot_isp_pub_attr.wdr_mode at +32");
_Static_assert(offsetof(v5_isp_pub_attr, sns_mode) == 36, "ot_isp_pub_attr.sns_mode at +36");
_Static_assert(offsetof(v5_isp_pub_attr, mipi_crop_attr) == 48,
               "ot_isp_pub_attr.mipi_crop_attr at +48");

/*
 * ot_isp_exp_info (ot_common_isp.h).
 *
 * The AE readback. Everything raptor wants is in the first forty bytes --
 * exposure time, the two sensor gains, the composite `exposure` -- plus
 * `iso` four kilobytes in, on the far side of a 1024-bin histogram.
 *
 * The four AE route structs at the end are 1296 bytes of curve nothing here
 * reads; carried as a sized array for the reason v5_venc.h states about
 * ot_venc_stream's statistics unions. The histogram is transcribed because
 * a future exposure-metering feature would want it and it costs one line.
 *
 * The gains are 22.10 fixed point: divide by 1024 for times. `exposure` is
 * the AE's own composite, int_time * again * dgain in the same 10-bit
 * precision, and is what a "how dark is it" test should read rather than
 * reconstructing it from the parts.
 */
typedef struct {
    unsigned int exp_time;
    unsigned int short_exp_time;
    unsigned int median_exp_time;
    unsigned int long_exp_time;
    unsigned int a_gain;
    unsigned int d_gain;
    unsigned int a_gain_sf;
    unsigned int d_gain_sf;
    unsigned int isp_d_gain;
    unsigned int exposure;
    int exposure_is_max;
    short hist_error;
    unsigned int ae_hist1024_value[V5_ISP_HIST_NUM];
    unsigned char ave_lum;
    unsigned int lines_per500ms;
    unsigned int piris_fno;
    unsigned int fps;
    unsigned int iso;
    unsigned int isosf;
    unsigned int iso_calibrate;
    unsigned int ref_exp_ratio;
    unsigned short wdr_exp_coef;
    unsigned int first_stable_time;
    unsigned int quick_star_iso;
    /* ae_route, ae_route_ex, ae_route_sf, ae_route_sf_ex: 260 + 388 twice. */
    unsigned int ae_routes[324];
} v5_isp_exp_info;

_Static_assert(sizeof(v5_isp_exp_info) == 5484, "ot_isp_exp_info is 5484 bytes");
_Static_assert(offsetof(v5_isp_exp_info, a_gain) == 16, "ot_isp_exp_info.a_gain at +16");
_Static_assert(offsetof(v5_isp_exp_info, d_gain) == 20, "ot_isp_exp_info.d_gain at +20");
_Static_assert(offsetof(v5_isp_exp_info, exposure) == 36, "ot_isp_exp_info.exposure at +36");
_Static_assert(offsetof(v5_isp_exp_info, ae_hist1024_value) == 48,
               "ot_isp_exp_info.ae_hist1024_value at +48");
_Static_assert(offsetof(v5_isp_exp_info, iso) == 4160, "ot_isp_exp_info.iso at +4160");
_Static_assert(offsetof(v5_isp_exp_info, ae_routes) == 4188,
               "ot_isp_exp_info AE route block at +4188");

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    /*
     * Bring-up, in call order. mem_init before set_pub_attr before init
     * before run; see the file comment.
     *
     * fnRun does not return until fnExit is called from another thread.
     */
    int (*fnMemInit)(int vi_pipe);
    int (*fnSetPubAttr)(int vi_pipe, const v5_isp_pub_attr *attr);
    int (*fnGetPubAttr)(int vi_pipe, v5_isp_pub_attr *attr);
    int (*fnInit)(int vi_pipe);
    int (*fnRun)(int vi_pipe);
    int (*fnExit)(int vi_pipe);

    /*
     * The sensor handshake. The sensor library calls
     * ss_mpi_isp_sensor_reg_callback itself from inside
     * pfn_register_callback, so raptor never calls it directly -- it is
     * bound anyway because its absence is the clearest possible signal
     * that libss_mpi_isp.so is not the library it claims to be.
     *
     * The unregister half raptor *does* call, on teardown, and it takes
     * only the sensor id.
     */
    int (*fnSensorRegCallback)(int vi_pipe, v5_isp_sns_attr_info *sns_attr_info,
                               const void *sns_register);
    int (*fnSensorUnregCallback)(int vi_pipe, int sensor_id);

    /*
     * 3A registration, out of libss_mpi_ae.so and libss_mpi_awb.so. Both
     * take the descriptor the sensor library filled in, unchanged.
     */
    int (*fnAeRegister)(int vi_pipe, const v5_isp_3a_alg_lib *ae_lib);
    int (*fnAeUnRegister)(int vi_pipe, const v5_isp_3a_alg_lib *ae_lib);
    int (*fnAwbRegister)(int vi_pipe, const v5_isp_3a_alg_lib *awb_lib);
    int (*fnAwbUnRegister)(int vi_pipe, const v5_isp_3a_alg_lib *awb_lib);

    /*
     * Exposure readback. In libss_mpi_ae.so, not libss_mpi_isp.so.
     *
     * Optional: it is a five-kilobyte copy per call and a build that never
     * shows exposure never needs it. A NULL here costs the OSD's exposure
     * fields, not the stream.
     */
    int (*fnQueryExposureInfo)(int vi_pipe, v5_isp_exp_info *exp_info);
} v5_isp_impl;

/*
 * v5_isp_load -- bind the ISP entry points.
 *
 * Called after hisi_isp_open() has put libss_mpi_isp.so and the 3A pair in
 * the search list; before that every one of these resolves to NULL and the
 * function returns RSS_ERR_NOTSUP on its first lookup.
 *
 * Everything except the exposure query is required, because a pipeline
 * missing any part of the bring-up sequence produces no image at all and
 * "the ISP is half loaded" is not a state worth carrying forward.
 */
static inline int v5_isp_load(v5_isp_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_isp";

    memset(lib, 0, sizeof(*lib));

#define V5_ISP_REQ(field, type, name)                                                              \
    do {                                                                                           \
        if (!(lib->field = (type)v5_symbol(mod, libs, name)))                                      \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V5_ISP_REQ(fnMemInit, int (*)(int), "ss_mpi_isp_mem_init");
    V5_ISP_REQ(fnSetPubAttr, int (*)(int, const v5_isp_pub_attr *), "ss_mpi_isp_set_pub_attr");
    V5_ISP_REQ(fnInit, int (*)(int), "ss_mpi_isp_init");
    V5_ISP_REQ(fnRun, int (*)(int), "ss_mpi_isp_run");
    V5_ISP_REQ(fnExit, int (*)(int), "ss_mpi_isp_exit");
    V5_ISP_REQ(fnSensorRegCallback, int (*)(int, v5_isp_sns_attr_info *, const void *),
               "ss_mpi_isp_sensor_reg_callback");
    V5_ISP_REQ(fnAeRegister, int (*)(int, const v5_isp_3a_alg_lib *), "ss_mpi_ae_register");
    V5_ISP_REQ(fnAwbRegister, int (*)(int, const v5_isp_3a_alg_lib *), "ss_mpi_awb_register");

#undef V5_ISP_REQ

    lib->fnGetPubAttr =
        (int (*)(int, v5_isp_pub_attr *))v5_symbol_opt(libs, "ss_mpi_isp_get_pub_attr");
    lib->fnSensorUnregCallback =
        (int (*)(int, int))v5_symbol_opt(libs, "ss_mpi_isp_sensor_unreg_callback");
    lib->fnAeUnRegister =
        (int (*)(int, const v5_isp_3a_alg_lib *))v5_symbol_opt(libs, "ss_mpi_ae_unregister");
    lib->fnAwbUnRegister =
        (int (*)(int, const v5_isp_3a_alg_lib *))v5_symbol_opt(libs, "ss_mpi_awb_unregister");

    lib->fnQueryExposureInfo =
        (int (*)(int, v5_isp_exp_info *))v5_symbol_opt(libs, "ss_mpi_isp_query_exposure_info");

    return RSS_OK;
}

static inline void v5_isp_unload(v5_isp_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_ISP_H */
