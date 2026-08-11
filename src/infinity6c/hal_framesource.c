/*
 * infinity6c/hal_framesource.c -- SNR, VIF, ISP and SCL for the MI 3.0 backend
 *
 * Everything upstream of the encoder. Counterpart to star/hal_framesource.c,
 * which covers VIF and VPE; on this generation the same ground is four modules
 * rather than two, because the ISP is a pipeline stage in its own right and the
 * scaling VPE did belongs to SCL.
 *
 *   SNR -> VIF -> ISP -> SCL
 *
 * A raptor framesource channel maps onto an SCL *output port*, not onto a
 * pipeline. That is the shape worth keeping in mind here: one sensor feeds one
 * VIF device, one ISP channel and one SCL channel, and the streams are ports
 * hanging off that last channel. So a second stream is a port and a scale
 * factor, not a second chain -- and the chain itself is shared, brought up by
 * the first channel to need it and torn down after the last.
 *
 * Almost nothing here is a policy decision. The sensor publishes its resolution
 * list, its interface, its bayer order and its precision, and VIF's pixel
 * format, the ISP's demosaic decision and the pool geometry all follow from
 * those. Querying rather than tabulating is what lets one binary serve any
 * sensor the vendor driver knows.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "infinity6c_state.h"

/* ================================================================
 * FORMAT MAPPING
 * ================================================================ */

/*
 * i6c_fs_pixfmt -- raptor's pixel format for an SCL output port.
 *
 * Only the formats an encoder consumes are mapped. Anything else falls back to
 * NV12, which every codec on this part accepts: a framesource port feeds VENC,
 * so an unsupported request is better served by a working stream in the usual
 * format than by a refusal.
 */
static i6c_common_pixfmt i6c_fs_pixfmt(rss_pixfmt_t fmt)
{
    switch (fmt) {
    case RSS_PIXFMT_NV12:
        return I6C_PIXFMT_YUV420SP;
    case RSS_PIXFMT_NV21:
        return I6C_PIXFMT_YUV420SP_NV21;
    case RSS_PIXFMT_YUYV422:
        return I6C_PIXFMT_YUV422_YUYV;
    case RSS_PIXFMT_UYVY422:
        return I6C_PIXFMT_YUV422_UYVY;
    case RSS_PIXFMT_YUV420P:
        return I6C_PIXFMT_YUV420P;
    default:
        HAL_LOG_WARN("infinity6c: pixel format %d has no SCL equivalent, using NV12", (int)fmt);
        return I6C_PIXFMT_YUV420SP;
    }
}

/*
 * i6c_vif_pixfmt -- what VIF should be told the sensor is sending.
 *
 * A raw sensor reports a bayer order and a precision instead of a pixel format,
 * and the bayer formats are laid out as one contiguous block indexed by both --
 * which is why this is arithmetic on the enum rather than a lookup. A sensor
 * that reports a bayer order past the end of the enum is sending YUV already and
 * names its format directly.
 */
static i6c_common_pixfmt i6c_vif_pixfmt(const i6c_snr_plane *plane)
{
    if (plane->bayer >= I6C_BAYER_END)
        return plane->pixFmt;

    return (i6c_common_pixfmt)(I6C_PIXFMT_RGB_BAYER + (int)plane->precision * (int)I6C_BAYER_END +
                               (int)plane->bayer);
}

/* ================================================================
 * SENSOR
 * ================================================================ */

/*
 * i6c_snr_select -- pick and apply a sensor mode.
 *
 * The sensor driver publishes modes and the HAL chooses one by index; nothing
 * takes a width and height. The first mode that can cover the requested
 * geometry and frame rate wins, which relies on the vendor listing modes
 * largest-first -- they do, and a wrong choice here costs resolution rather
 * than correctness.
 */
static int i6c_snr_select(infinity6c_state_t *st, unsigned short width, unsigned short height,
                          unsigned int fps)
{
    unsigned int count = 0;
    unsigned int i;
    int ret;

    /*
     * Single-plane first, and before anything reads the mode list: multi-plane
     * is for HDR sensors exposing one plane per exposure, and it changes what
     * the list means.
     */
    if ((ret = st->snr.set_plane_mode(I6C_DEV_ID(I6C_SNR_PAD), 0)) != 0) {
        HAL_LOG_ERR("MI_SNR_SetPlaneMode failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->snr.query_res_count(I6C_DEV_ID(I6C_SNR_PAD), &count)) != 0) {
        HAL_LOG_ERR("MI_SNR_QueryResCount failed: %d", ret);
        return RSS_ERR_IO;
    }
    if (!count) {
        HAL_LOG_ERR("infinity6c: sensor reports no modes; is the sensor driver loaded?");
        return RSS_ERR_NOENT;
    }

    st->snr_profile = -1;
    for (i = 0; i < count; i++) {
        i6c_snr_res res;

        memset(&res, 0, sizeof(res));
        if ((ret = st->snr.get_res(I6C_DEV_ID(I6C_SNR_PAD), (unsigned char)i, &res)) != 0) {
            HAL_LOG_ERR("MI_SNR_GetRes(%u) failed: %d", i, ret);
            return RSS_ERR_IO;
        }

        HAL_LOG_DBG("infinity6c: sensor mode %u: %ux%u, up to %u fps, \"%.*s\"", i, res.crop.width,
                    res.crop.height, res.maxFps, (int)sizeof(res.desc), res.desc);

        if (width > res.crop.width || height > res.crop.height || fps > res.maxFps)
            continue;

        st->snr_profile = (int)i;
        break;
    }

    if (st->snr_profile < 0) {
        HAL_LOG_ERR("infinity6c: no sensor mode covers %ux%u at %u fps", width, height, fps);
        return RSS_ERR_INVAL;
    }

    if ((ret = st->snr.set_res(I6C_DEV_ID(I6C_SNR_PAD), (unsigned char)st->snr_profile)) != 0) {
        HAL_LOG_ERR("MI_SNR_SetRes(%d) failed: %d", st->snr_profile, ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->snr.set_fps(I6C_DEV_ID(I6C_SNR_PAD), fps)) != 0) {
        HAL_LOG_ERR("MI_SNR_SetFps(%u) failed: %d", fps, ret);
        return RSS_ERR_IO;
    }
    st->fps = fps;

    /* Orientation belongs to the ISP here; the sensor stays unflipped. */
    if ((ret = st->snr.set_orien(I6C_DEV_ID(I6C_SNR_PAD), 0, 0)) != 0) {
        HAL_LOG_ERR("MI_SNR_SetOrien failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->snr.get_pad_info(I6C_DEV_ID(I6C_SNR_PAD), &st->pad)) != 0) {
        HAL_LOG_ERR("MI_SNR_GetPadInfo failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->snr.get_plane_info(I6C_DEV_ID(I6C_SNR_PAD), 0, &st->plane)) != 0) {
        HAL_LOG_ERR("MI_SNR_GetPlaneInfo failed: %d", ret);
        return RSS_ERR_IO;
    }

    HAL_LOG_INFO("infinity6c: sensor \"%.*s\" mode %d, %ux%u at %u fps",
                 (int)sizeof(st->plane.sensName), st->plane.sensName, st->snr_profile,
                 st->plane.capt.width, st->plane.capt.height, fps);

    return RSS_OK;
}

/* ================================================================
 * POOL
 * ================================================================ */

/*
 * i6c_pool_configure -- the private ring pool SCL hands frames out of.
 *
 * Not a tuning step. A stage that passes frames on through a ring needs a pool
 * of its own configured before it starts, and the ring line count is how much of
 * a frame accumulates before the next stage may begin on it -- a quarter of the
 * frame here, which is what the vendor's own reference uses and what keeps the
 * encoder from waiting on a whole frame.
 */
static int i6c_pool_configure(infinity6c_state_t *st)
{
    i6c_sys_pool pool;
    int ret;

    memset(&pool, 0, sizeof(pool));
    pool.type = I6C_SYS_POOL_DEVICE_RING;
    pool.create = 1;
    pool.config.ring.module = I6C_SYS_MOD_SCL;
    pool.config.ring.device = I6C_SCL_DEV;
    pool.config.ring.maxWidth = st->plane.capt.width;
    pool.config.ring.maxHeight = st->plane.capt.height;
    pool.config.ring.ringLine = st->plane.capt.height / 4;

    if ((ret = st->sys.config_pool(I6C_SOC_ID, &pool)) != 0) {
        HAL_LOG_ERR("MI_SYS_ConfigPrivateMMAPool(SCL ring) failed: %d", ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/* ================================================================
 * PIPELINE
 * ================================================================ */

static int i6c_vif_bringup(infinity6c_state_t *st)
{
    i6c_vif_grp group;
    i6c_vif_dev device;
    i6c_vif_port port;
    int ret;

    /*
     * The group carries how the sensor is wired rather than what it sends, and
     * it has to exist before the device under it will take an attribute. The
     * clock edge only means anything on BT656, where the sensor reports it;
     * everything else is double-edged.
     */
    memset(&group, 0, sizeof(group));
    group.intf = st->pad.intf;
    group.work = I6C_VIF_WORK_1MULTIPLEX;
    group.hdr = I6C_HDR_OFF;
    group.edge = st->pad.intf == I6C_INTF_BT656 ? st->pad.intfAttr.bt656.edge : I6C_EDGE_DOUBLE;
    group.interlaceOn = 0;
    group.grpStitch = 1u << I6C_VIF_GRP;

    if ((ret = st->vif.create_group(I6C_DEV_ID(I6C_VIF_GRP), &group)) != 0) {
        HAL_LOG_ERR("MI_VIF_CreateDevGroup failed: %d", ret);
        return RSS_ERR_IO;
    }

    memset(&device, 0, sizeof(device));
    device.pixFmt = i6c_vif_pixfmt(&st->plane);
    device.crop = st->plane.capt;
    device.field = 0;
    device.halfHScan = 0;

    if ((ret = st->vif.set_dev_attr(I6C_DEV_ID(I6C_VIF_DEV), &device)) != 0) {
        HAL_LOG_ERR("MI_VIF_SetDevAttr failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->vif.enable_dev(I6C_DEV_ID(I6C_VIF_DEV))) != 0) {
        HAL_LOG_ERR("MI_VIF_EnableDev failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * VIF neither scales nor converts: the port hands on what the sensor sent,
     * at full rate and uncompressed. Everything after this is the ISP's.
     */
    memset(&port, 0, sizeof(port));
    port.capt = st->plane.capt;
    port.dest.width = st->plane.capt.width;
    port.dest.height = st->plane.capt.height;
    port.pixFmt = i6c_vif_pixfmt(&st->plane);
    port.frate = I6C_VIF_FRATE_FULL;
    port.compress = I6C_COMPR_NONE;

    if ((ret = st->vif.set_port_attr(I6C_DEV_ID(I6C_VIF_DEV), I6C_VIF_PORT, &port)) != 0) {
        HAL_LOG_ERR("MI_VIF_SetOutputPortAttr failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->vif.enable_port(I6C_DEV_ID(I6C_VIF_DEV), I6C_VIF_PORT)) != 0) {
        HAL_LOG_ERR("MI_VIF_EnableOutputPort failed: %d", ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

static int i6c_isp_bringup(infinity6c_state_t *st)
{
    unsigned int combo = 1u << I6C_SNR_PAD;
    i6c_isp_chn channel;
    i6c_isp_para param;
    i6c_isp_port port;
    int ret;

    if ((ret = st->isp.create_dev(I6C_DEV_ID(I6C_ISP_DEV), &combo)) != 0) {
        HAL_LOG_ERR("MI_ISP_CreateDevice failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * sensorId is a mask of the pads this channel draws from, and the version
     * block is an output rather than an input -- so the struct is cleared and
     * only the mask is set.
     */
    memset(&channel, 0, sizeof(channel));
    channel.sensorId = 1u << I6C_SNR_PAD;

    if ((ret = st->isp.create_chn(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &channel)) != 0) {
        HAL_LOG_ERR("MI_ISP_CreateChannel failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * Cleared before filling, deliberately: the driver reads this struct's last
     * member with a word-wide load where the declaration is a byte, so leaving
     * the padding uninitialised would hand it three bytes of stack. The same
     * applies to the output port below. See sigmastar-headers' i6c_isp.h.
     */
    memset(&param, 0, sizeof(param));
    param.hdr = I6C_HDR_OFF;
    param.level3DNR = 1;
    param.mirror = 0;
    param.flip = 0;
    param.rotate = 0;
    /* A sensor already sending YUV needs the demosaic run backwards. */
    param.yuv2BayerOn = st->plane.bayer >= I6C_BAYER_END;

    if ((ret = st->isp.set_chn_param(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, &param)) != 0) {
        HAL_LOG_ERR("MI_ISP_SetChnParam failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->isp.start_chn(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN)) != 0) {
        HAL_LOG_ERR("MI_ISP_StartChannel failed: %d", ret);
        return RSS_ERR_IO;
    }

    /*
     * A zero crop means "the whole frame", which is what this port wants: the
     * ISP hands SCL everything and SCL does the per-stream scaling. YUYV because
     * it is what the scaler takes without a conversion pass.
     */
    memset(&port, 0, sizeof(port));
    port.pixFmt = I6C_PIXFMT_YUV422_YUYV;
    port.compress = I6C_COMPR_NONE;

    if ((ret = st->isp.set_port_param(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, I6C_ISP_PORT, &port)) !=
        0) {
        HAL_LOG_ERR("MI_ISP_SetOutputPortParam failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->isp.enable_port(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, I6C_ISP_PORT)) != 0) {
        HAL_LOG_ERR("MI_ISP_EnableOutputPort failed: %d", ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

static int i6c_scl_bringup(infinity6c_state_t *st)
{
    /*
     * A mask of the input sources this device will accept, not a count. All four
     * are declared because which one a bind names is decided later, and a source
     * the device was not told about is refused at bind time.
     */
    unsigned int binds = 0xf;
    unsigned int reserved = 0;
    i6c_scl_chn channel;
    int ret;

    if ((ret = st->scl.create_dev(I6C_DEV_ID(I6C_SCL_DEV), &binds)) != 0) {
        HAL_LOG_ERR("MI_SCL_CreateDevice failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->scl.create_chn(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, &reserved)) != 0) {
        HAL_LOG_ERR("MI_SCL_CreateChannel failed: %d", ret);
        return RSS_ERR_IO;
    }

    memset(&channel, 0, sizeof(channel));
    channel.rotate = I6C_SCL_ROTATE_NONE;

    if ((ret = st->scl.set_chn_param(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, &channel)) != 0) {
        HAL_LOG_ERR("MI_SCL_SetChnParam failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = st->scl.start_chn(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN)) != 0) {
        HAL_LOG_ERR("MI_SCL_StartChannel failed: %d", ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * i6c_link -- bind one stage's output port to the next stage's input.
 *
 * REALTIME throughout the capture chain, which is the point of it: the stages
 * hand frames straight on rather than through a queue, so nothing accumulates
 * latency between the sensor and the scaler. The encoder's own bind is
 * different and lives in hal_encoder.c.
 */
static int i6c_link(infinity6c_state_t *st, i6c_sys_mod src_mod, unsigned int src_dev,
                    unsigned int src_chn, unsigned int src_port, i6c_sys_mod dst_mod,
                    unsigned int dst_dev, unsigned int dst_chn, unsigned int dst_port)
{
    i6c_sys_bind src;
    i6c_sys_bind dst;
    int ret;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    src.module = src_mod;
    src.device = src_dev;
    src.channel = src_chn;
    src.port = src_port;
    dst.module = dst_mod;
    dst.device = dst_dev;
    dst.channel = dst_chn;
    dst.port = dst_port;

    ret = st->sys.bind_ext(I6C_SOC_ID, &src, &dst, st->fps, st->fps, I6C_SYS_LINK_REALTIME, 0);
    if (ret) {
        HAL_LOG_ERR("MI_SYS_BindChnPort2(mod %d -> mod %d) failed: %d", (int)src_mod, (int)dst_mod,
                    ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * i6c_pipeline_create -- everything from the sensor to the scaler.
 *
 * Reference counted rather than tied to hal_init, because the chain is shared:
 * the geometry it is built with comes from the first channel that asks for it,
 * and rebuilding it per channel would tear down the streams already running.
 * A later channel at a different resolution is served by its own SCL port.
 */
int i6c_pipeline_create(infinity6c_state_t *st, const rss_fs_config_t *cfg)
{
    unsigned int fps;
    int ret;

    if (st->pipeline_up) {
        st->pipeline_refs++;
        return RSS_OK;
    }

    fps = cfg->fps_den ? cfg->fps_num / cfg->fps_den : cfg->fps_num;
    if (!fps)
        fps = 25;

    if ((ret = i6c_snr_select(st, cfg->width, cfg->height, fps)) != RSS_OK)
        return ret;

    /* Before the sensor is enabled: SCL draws from this pool once frames flow. */
    if ((ret = i6c_pool_configure(st)) != RSS_OK)
        return ret;

    if ((ret = st->snr.enable(I6C_DEV_ID(I6C_SNR_PAD))) != 0) {
        HAL_LOG_ERR("MI_SNR_Enable failed: %d", ret);
        return RSS_ERR_IO;
    }

    if ((ret = i6c_vif_bringup(st)) != RSS_OK)
        goto fail_snr;

    if ((ret = i6c_isp_bringup(st)) != RSS_OK)
        goto fail_vif;

    if ((ret = i6c_scl_bringup(st)) != RSS_OK)
        goto fail_isp;

    /*
     * VIF's output port goes in the bind's *channel* field, with the bind's port
     * left at zero -- not a transcription slip. VIF addresses its ports as
     * (device, port) and has no channel layer, so the descriptor's channel slot
     * is what carries the port here. Every other stage in this file uses the two
     * fields as they read. Both are zero on a one-sensor camera, so getting it
     * wrong would only show up on a second port.
     */
    if ((ret = i6c_link(st, I6C_SYS_MOD_VIF, I6C_VIF_DEV, I6C_VIF_PORT, 0, I6C_SYS_MOD_ISP,
                        I6C_ISP_DEV, I6C_ISP_CHN, I6C_ISP_PORT)) != RSS_OK)
        goto fail_scl;

    if ((ret = i6c_link(st, I6C_SYS_MOD_ISP, I6C_ISP_DEV, I6C_ISP_CHN, I6C_ISP_PORT,
                        I6C_SYS_MOD_SCL, I6C_SCL_DEV, I6C_SCL_CHN, 0)) != RSS_OK)
        goto fail_vif_isp_link;

    st->pipeline_up = true;
    st->pipeline_refs = 1;

    HAL_LOG_INFO("infinity6c: pipeline up, VIF -> ISP -> SCL at %ux%u %u fps", st->plane.capt.width,
                 st->plane.capt.height, st->fps);
    /*
     * Said in the log rather than only in a comment, because this is where it is
     * read. With no tuning loaded the picture is whatever CUS3A's defaults
     * produce, which can look badly wrong while being exactly what this code
     * asks for -- so on this backend a bad picture is expected and carries no
     * information, while no picture at all is a real result. Anyone reading a
     * bring-up log needs that distinction before they read the colour.
     */
    HAL_LOG_INFO("infinity6c: no IQ tuning is loaded on this backend -- judge capture and "
                 "encode, not colour or exposure");
    return RSS_OK;

    /*
     * Unwound in reverse, and only as far as this call got. A half-built
     * pipeline left in place would make the next attempt fail on a device that
     * already exists, which reports as a configuration error rather than as the
     * original fault.
     */
fail_vif_isp_link: {
    i6c_sys_bind src;
    i6c_sys_bind dst;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    src.module = I6C_SYS_MOD_VIF;
    src.device = I6C_VIF_DEV;
    src.channel = I6C_VIF_PORT;
    dst.module = I6C_SYS_MOD_ISP;
    dst.device = I6C_ISP_DEV;
    dst.channel = I6C_ISP_CHN;
    dst.port = I6C_ISP_PORT;
    st->sys.unbind(I6C_SOC_ID, &src, &dst);
}
fail_scl:
    st->scl.stop_chn(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN);
    st->scl.destroy_chn(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN);
    st->scl.destroy_dev(I6C_DEV_ID(I6C_SCL_DEV));
fail_isp:
    st->isp.disable_port(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, I6C_ISP_PORT);
    st->isp.stop_chn(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN);
    st->isp.destroy_chn(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN);
    st->isp.destroy_dev(I6C_DEV_ID(I6C_ISP_DEV));
fail_vif:
    st->vif.disable_port(I6C_DEV_ID(I6C_VIF_DEV), I6C_VIF_PORT);
    st->vif.disable_dev(I6C_DEV_ID(I6C_VIF_DEV));
    st->vif.destroy_group(I6C_DEV_ID(I6C_VIF_GRP));
fail_snr:
    st->snr.disable(I6C_DEV_ID(I6C_SNR_PAD));
    return ret;
}

/*
 * i6c_pipeline_destroy -- the reverse, and unconditionally.
 *
 * Return values are logged by the calls themselves and otherwise ignored:
 * there is nothing to retry, and stopping early would leak whatever the
 * remaining steps would have released.
 */
void i6c_pipeline_destroy(infinity6c_state_t *st)
{
    i6c_sys_bind src;
    i6c_sys_bind dst;
    int i;

    if (!st->pipeline_up)
        return;
    if (st->pipeline_refs && --st->pipeline_refs)
        return;

    for (i = 0; i < I6C_MAX_CHN; i++) {
        if (st->fs[i].enabled) {
            st->scl.disable_port(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, (unsigned int)i);
            st->fs[i].enabled = false;
        }
        st->fs[i].configured = false;
    }

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    src.module = I6C_SYS_MOD_ISP;
    src.device = I6C_ISP_DEV;
    src.channel = I6C_ISP_CHN;
    src.port = I6C_ISP_PORT;
    dst.module = I6C_SYS_MOD_SCL;
    dst.device = I6C_SCL_DEV;
    dst.channel = I6C_SCL_CHN;
    dst.port = 0;
    st->sys.unbind(I6C_SOC_ID, &src, &dst);

    st->scl.stop_chn(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN);
    st->scl.destroy_chn(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN);
    st->scl.destroy_dev(I6C_DEV_ID(I6C_SCL_DEV));

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    src.module = I6C_SYS_MOD_VIF;
    src.device = I6C_VIF_DEV;
    src.channel = I6C_VIF_PORT;
    dst.module = I6C_SYS_MOD_ISP;
    dst.device = I6C_ISP_DEV;
    dst.channel = I6C_ISP_CHN;
    dst.port = I6C_ISP_PORT;
    st->sys.unbind(I6C_SOC_ID, &src, &dst);

    st->isp.disable_port(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN, I6C_ISP_PORT);
    st->isp.stop_chn(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN);
    st->isp.destroy_chn(I6C_DEV_ID(I6C_ISP_DEV), I6C_ISP_CHN);
    st->isp.destroy_dev(I6C_DEV_ID(I6C_ISP_DEV));

    st->vif.disable_port(I6C_DEV_ID(I6C_VIF_DEV), I6C_VIF_PORT);
    st->vif.disable_dev(I6C_DEV_ID(I6C_VIF_DEV));
    st->vif.destroy_group(I6C_DEV_ID(I6C_VIF_GRP));

    st->snr.disable(I6C_DEV_ID(I6C_SNR_PAD));

    st->pipeline_up = false;
    st->snr_profile = -1;

    HAL_LOG_INFO("infinity6c: pipeline down");
}

/* ================================================================
 * FRAMESOURCE OPS
 *
 * A channel is an SCL output port. Creating one configures the port;
 * enabling one starts it.
 * ================================================================ */

/*
 * i6c_fs_apply -- push a channel's geometry to its SCL output port.
 *
 * A zero crop means "take the whole input", so the port's only real content is
 * the output size -- the scale factor, in effect. Compression is left off
 * because the encoder reads these frames directly.
 */
static int i6c_fs_apply(infinity6c_state_t *st, int chn, const rss_fs_config_t *cfg)
{
    i6c_scl_port port;
    unsigned short width;
    unsigned short height;
    int ret;

    /* The scaler block is how a sub-stream asks for less than the sensor. */
    if (cfg->scaler.enable && cfg->scaler.out_width > 0 && cfg->scaler.out_height > 0) {
        width = (unsigned short)cfg->scaler.out_width;
        height = (unsigned short)cfg->scaler.out_height;
    } else {
        width = cfg->width;
        height = cfg->height;
    }

    memset(&port, 0, sizeof(port));
    port.output.width = width;
    port.output.height = height;
    port.mirror = 0;
    port.flip = 0;
    port.pixFmt = i6c_fs_pixfmt(cfg->pixfmt);
    port.compress = I6C_COMPR_NONE;

    ret = st->scl.set_port_param(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, (unsigned int)chn, &port);
    if (ret) {
        HAL_LOG_ERR("MI_SCL_SetOutputPortParam(port %d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    st->fs[chn].width = width;
    st->fs[chn].height = height;
    st->fs[chn].pixfmt = port.pixFmt;
    st->fs[chn].configured = true;

    return RSS_OK;
}

/* ================================================================
 * PORTS FOR A CONSUMER THAT IS NOT A FRAMESOURCE CHANNEL
 *
 * rvd creates a framesource channel per video stream and none for a
 * JPEG stream -- it pairs the JPEG encoder with another stream's
 * framesource instead, and feeds it by group membership. MI has no
 * groups, so on this part that pairing has to become a port of its
 * own. These exist for hal_enc_register_channel to build one with;
 * see its comment for why a dedicated port rather than a shared one.
 * ================================================================ */

/*
 * i6c_fs_spare_port -- the lowest SCL output port nothing is using, or -1.
 *
 * Safe to ask at register time because rvd configures every framesource port
 * before creating any encoder channel: pipeline_init runs its fs_create_channel
 * loop to completion first, and that loop skips JPEG streams outright. So an
 * unconfigured port here is genuinely spare rather than merely not configured
 * yet.
 */
int i6c_fs_spare_port(const infinity6c_state_t *st)
{
    int i;

    for (i = 0; i < I6C_MAX_CHN; i++)
        if (!st->fs[i].configured)
            return i;

    return -1;
}

/*
 * i6c_fs_clone_port -- configure dst_port with src_port's geometry.
 *
 * Configure only. Enabling is left to the caller because the order that works
 * for a framesource port is configure, bind, enable -- which is the order rvd
 * drives, fs_create_channel then bind then fs_enable_channel -- and a port
 * enabled before anything is bound to it has nowhere to put a frame.
 */
int i6c_fs_clone_port(infinity6c_state_t *st, int src_port, int dst_port)
{
    i6c_scl_port port;
    int ret;

    if (src_port < 0 || src_port >= I6C_MAX_CHN || dst_port < 0 || dst_port >= I6C_MAX_CHN)
        return RSS_ERR_INVAL;
    if (!st->fs[src_port].configured)
        return RSS_ERR_INVAL;

    memset(&port, 0, sizeof(port));
    port.output.width = st->fs[src_port].width;
    port.output.height = st->fs[src_port].height;
    port.mirror = 0;
    port.flip = 0;
    port.pixFmt = st->fs[src_port].pixfmt;
    port.compress = I6C_COMPR_NONE;

    ret =
        st->scl.set_port_param(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, (unsigned int)dst_port, &port);
    if (ret) {
        HAL_LOG_ERR("MI_SCL_SetOutputPortParam(port %d, cloned from port %d) failed: %d", dst_port,
                    src_port, ret);
        return RSS_ERR_IO;
    }

    st->fs[dst_port].width = st->fs[src_port].width;
    st->fs[dst_port].height = st->fs[src_port].height;
    st->fs[dst_port].pixfmt = st->fs[src_port].pixfmt;
    st->fs[dst_port].configured = true;

    return RSS_OK;
}

/* Shared with hal_fs_enable_channel: the same port, reached two ways. */
int i6c_fs_enable_port(infinity6c_state_t *st, int port)
{
    int ret;

    if (st->fs[port].enabled)
        return RSS_OK;

    ret = st->scl.enable_port(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, (unsigned int)port);
    if (ret) {
        HAL_LOG_ERR("MI_SCL_EnableOutputPort(%d) failed: %d", port, ret);
        return RSS_ERR_IO;
    }
    st->fs[port].enabled = true;

    return RSS_OK;
}

void i6c_fs_release_port(infinity6c_state_t *st, int port)
{
    if (port < 0 || port >= I6C_MAX_CHN)
        return;

    if (st->fs[port].enabled) {
        st->scl.disable_port(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, (unsigned int)port);
        st->fs[port].enabled = false;
    }
    st->fs[port].configured = false;
}

int hal_fs_create_channel(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    int ret;

    I6C_ENTER(ctx, chn, st);

    if (!cfg)
        return RSS_ERR_INVAL;
    if (st->fs[chn].configured)
        return RSS_ERR_BUSY;

    if ((ret = i6c_pipeline_create(st, cfg)) != RSS_OK)
        return ret;

    if ((ret = i6c_fs_apply(st, chn, cfg)) != RSS_OK) {
        i6c_pipeline_destroy(st);
        return ret;
    }

    HAL_LOG_INFO("infinity6c: fs chn %d configured, %ux%u", chn, st->fs[chn].width,
                 st->fs[chn].height);
    return RSS_OK;
}

int hal_fs_set_channel_attr(void *ctx, int chn, const rss_fs_config_t *cfg)
{
    I6C_ENTER(ctx, chn, st);

    if (!cfg)
        return RSS_ERR_INVAL;
    if (!st->fs[chn].configured)
        return RSS_ERR_INVAL;

    return i6c_fs_apply(st, chn, cfg);
}

int hal_fs_enable_channel(void *ctx, int chn)
{
    I6C_ENTER(ctx, chn, st);

    if (!st->fs[chn].configured)
        return RSS_ERR_INVAL;

    return i6c_fs_enable_port(st, chn);
}

int hal_fs_disable_channel(void *ctx, int chn)
{
    int ret;

    I6C_ENTER(ctx, chn, st);

    if (!st->fs[chn].enabled)
        return RSS_OK;

    ret = st->scl.disable_port(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, (unsigned int)chn);
    st->fs[chn].enabled = false;
    if (ret) {
        HAL_LOG_ERR("MI_SCL_DisableOutputPort(%d) failed: %d", chn, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

int hal_fs_destroy_channel(void *ctx, int chn)
{
    I6C_ENTER(ctx, chn, st);

    if (!st->fs[chn].configured)
        return RSS_OK;

    hal_fs_disable_channel(ctx, chn);
    st->fs[chn].configured = false;

    /* Drops the shared chain once the last channel using it is gone. */
    i6c_pipeline_destroy(st);

    return RSS_OK;
}

/*
 * hal_fs_set_rotation -- rotate every stream at once.
 *
 * Rotation lives on the SCL *channel*, not the port, so it cannot be set per
 * stream: the ports all hang off one channel. Accepted for channel 0 and
 * refused elsewhere rather than silently applied to everything, since a caller
 * asking to rotate one stream would otherwise get all of them.
 */
int hal_fs_set_rotation(void *ctx, int chn, int degrees)
{
    i6c_scl_chn channel;
    int ret;

    I6C_ENTER(ctx, chn, st);

    if (chn != 0) {
        HAL_LOG_WARN("infinity6c: rotation is per SCL channel, so it cannot apply to chn %d alone",
                     chn);
        return RSS_ERR_NOTSUP;
    }
    if (!st->pipeline_up)
        return RSS_ERR_INVAL;

    memset(&channel, 0, sizeof(channel));
    switch (degrees) {
    case 0:
        channel.rotate = I6C_SCL_ROTATE_NONE;
        break;
    case 90:
        channel.rotate = I6C_SCL_ROTATE_90;
        break;
    case 180:
        channel.rotate = I6C_SCL_ROTATE_180;
        break;
    case 270:
        channel.rotate = I6C_SCL_ROTATE_270;
        break;
    default:
        return RSS_ERR_INVAL;
    }

    ret = st->scl.set_chn_param(I6C_DEV_ID(I6C_SCL_DEV), I6C_SCL_CHN, &channel);
    if (ret) {
        HAL_LOG_ERR("MI_SCL_SetChnParam(rotate %d) failed: %d", degrees, ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}
