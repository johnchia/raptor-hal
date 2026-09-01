/*
 * caps_hisilicon.inc -- capability tables for the HiSilicon backends.
 *
 * Included from the #else arm of caps_sigmastar.inc's platform chain, which is
 * itself the #else arm of hal_caps.c's. Chaining here rather than adding a
 * second hook to hal_caps.c keeps the shared file at one include; this file
 * therefore inherits the duty caps_sigmastar.inc used to hold, and owns the
 * unknown-platform #error. Fork-local; see FORK.md.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * WHAT IS PUBLISHED HERE, AND WHY SO LITTLE
 *
 * A capability flag is a promise that an op exists to service it. Phase 2
 * brings up the video pipeline -- VI, the ISP, VPSS and VENC -- so the
 * encoder flags whose ops now exist are true and the rest stay false by
 * design rather than by omission. rvd reads a false flag as "this platform
 * cannot", which is exactly true. Phases 3-5 raise each remaining flag in the
 * commit that publishes its op, never ahead of it.
 *
 * The numbers that are non-zero are the ones measured on hardware.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * MEASURED ON A HI3516EV300, 2026-08-31
 *
 * OpenIPC hi3516ev300_lite, IMX335 at 2592x1944, live two-stream pipeline.
 * Raw dumps in board-dumps/hi3516ev300-196-20260831/ of the parent tree; the
 * write-up is PROBE-hi3516ev300-phase-minus-1.md.
 *
 *   max_enc_channels   3   /proc/umap/venc reports VencMaxChnNum = 3 as a
 *                          module parameter. That is a driver-level cap, well
 *                          under RVD_MAX_STREAMS, and it is a real number
 *                          rather than a header constant.
 *
 *   max_fs_channels    2   A floor, not a ceiling, and the one number here
 *                          that is expected to rise. Two VPSS channels were
 *                          observed enabled on a live pipeline; the driver
 *                          reports u32VpssSplitNodeNum = 3, and divinus's
 *                          V4_VPSS_CHN_NUM 6 is a constant in its source
 *                          rather than a measurement. Publishing the observed
 *                          number keeps the promise honest until the
 *                          create-channels-0..5 experiment runs (risk R6).
 *
 *                          Phase 2 added a prediction to test rather than a
 *                          measurement to publish: the SDK's own header for
 *                          this chip says VPSS_MAX_PHY_CHN_NUM is 3 and
 *                          VPSS_MAX_CHN_NUM is 7 (three physical channels
 *                          plus four extension channels), which is what
 *                          V4_VPSS_MAX_PHY_CHN_NUM in v4_vpss.h sizes the
 *                          channel array by. A header constant is still not
 *                          a measurement, so this number does not move until
 *                          somebody creates channels 0..5 on a board and
 *                          records which ones accept SetChnAttr.
 *
 *   max_enc_pixel_rate 0   Unmeasured, which rvd reads as "do not test
 *                          against". The board runs 5M@20 plus a substream,
 *                          but a working configuration is not a rated ceiling.
 *
 * Per-part differences are expected here eventually -- EV200 and EV300 differ
 * in encoder rating even though they share an MPP build -- which is what the
 * part-level #if chain below is for, even though both arms are identical now.
 * ═══════════════════════════════════════════════════════════════════════
 *
 * SPDX-License-Identifier: MIT
 */

#if defined(PLATFORM_HI3516EV200) || defined(PLATFORM_HI3516EV300)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    /*
     * A family name, not a release. The running MPP answers with its own
     * version string -- "Hi3516EV200_MPP_V1.0.1.2 B030 Release" on the EV300
     * board, which is also why the string here cannot be a release number --
     * and hal_sys_get_version passes that through, so anything wanting the
     * exact drop asks the op rather than reading this.
     */
    .sdk_version = "HiMPP",

    /*
     * Encoder.
     *
     * has_h265 was already true on evidence rather than on the datasheet --
     * the board's VENC channel 0 runs H.265 at 2592x1944. has_set_bitrate is
     * true as of Phase 2 because enc_set_bitrate exists and works the only
     * way HiMPP allows: a read-modify-write of the whole channel attribute.
     *
     * The three that stay false are decisions, not gaps:
     *
     *   has_rotation   VPSS has no rotate on gen4. Mirror and flip are
     *                  channel attributes; a 90-degree rotation is not
     *                  available at any stage of this pipeline.
     *   has_gop_attr   The op would be enc_set_chn_gop_attr, which takes an
     *                  opaque vendor struct. gen4's is VENC_GOP_ATTR_S with a
     *                  per-mode union, and publishing it means committing to
     *                  a layout callers pass through unexamined.
     *   has_capped_rc  gen4 has AVBR, QVBR and CVBR, and each needs
     *   has_smart_rc   parameters rvd does not carry -- u32ChangePos, the
     *                  quality-level table, the long-term statistics window.
     *                  enc_set_rc_mode maps them onto VBR with a maximum
     *                  bitrate, which is the honest approximation; claiming
     *                  the capability would promise a quality target nobody
     *                  supplied.
     */
    .has_h265 = true,
    .has_rotation = false,
    .has_set_bitrate = true,
    .has_gop_attr = false,
    .has_capped_rc = false,
    .has_smart_rc = false,

    /*
     * JPEG. Snapshot channels are PT_MJPEG -- PT_JPEG refuses this
     * attribute set outright (0xa0088003 at CreateChn, measured), divinus
     * never creates one either, and an MJPEG channel emits the same JPEG
     * bytes. Quality is the FIXQP rc's qfactor, which is what
     * enc_set_jpeg_qp drives -- despite the op's name, the value is rvd's
     * 1..100 quality, mapped onto the driver's 1..99 Qfactor. jpeg_pulse
     * because a FIXQP MJPEG channel has no rate limiter: without rvd's
     * start-one-frame-stop pacing, a held snapshot consumer would have the
     * channel encoding full-size JPEGs at the pipeline's full rate.
     * max_jpeg_pixels stays 0 -- unmeasured, per the rule above.
     */
    .has_jpeg = true,
    .has_jpeg_qp = true,
    .jpeg_pulse = true,

    /* ISP -- Phase 3. No ops, so no flags. */
    .has_color2grey = false,
    .has_defog = false,
    .has_sinter = false,
    .has_drc = false,
    .has_temper = false,
    .has_ae_comp = false,

    /*
     * One sensor. Gen4 has a second VI pipe, but sensor identity here is
     * whichever libsns_*.so gets dlopened and the ISP is keyed on vi_pipe, so
     * a second sensor is a design question and not a spare register.
     */
    .max_sensors = 1,

    .max_enc_channels = 3,
    .max_fs_channels = 2,
    .max_enc_pixel_rate = 0,

    /* OSD -- Phase 5. RGN exists on this silicon; nothing drives it yet. */
    .max_osd_regions = 0,
    .max_osd_groups = 0,

    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
};

#else
#error                                                                                             \
    "No PLATFORM_* defined. Set one of: PLATFORM_T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 A1 INFINITY6E INFINITY6B0 INFINITY6C HI3516EV200 HI3516EV300"
#endif
