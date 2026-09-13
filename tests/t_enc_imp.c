/*
 * t_enc_imp.c -- what the rate-control setters send the Ingenic encoder.
 *
 * One claim carries most of the suite: switching rate-control mode must build
 * the target mode's attributes from nothing, never from the channel's current
 * ones. The QP-bound legs at the end are the corollary -- because nothing
 * survives a mode change, the bounds a caller configured have to be put back
 * afterwards, and hal_enc_set_qp_bounds is where that lands.
 *
 * IMPEncoderAttrRcMode is a tag plus a union whose arms do not line up -- CBR
 * carries no uMaxBitRate, so every field after the target bitrate sits one
 * uint32_t earlier than VBR's. Read the channel's attributes as CBR, flip the
 * tag to VBR and write them back, and the encoder is handed one field's bytes
 * under another field's name: an I/P delta of -1 arriving as a QP bound. On a
 * T31 the driver refuses that (Codec_Encode_SetRcParam) and can leave the
 * channel unable to produce a frame, which is a great deal worse than an error
 * return -- rvd's own teardown then hangs in the codec ioctl and only SIGKILL
 * recovers it.
 *
 * Testable on a host because the arm is assembled in plain C before the single
 * vendor call that sends it. IMP_Encoder_GetChnAttrRcMode -- the call that read
 * the current arm -- is left in the abort stubs on purpose: if this code ever
 * reaches for the channel's existing attributes again, the suite dies rather
 * than passes.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#define PLATFORM_T31 1
#define HAL_MODULE_VIDEO 1

#include "hal_encoder.c"

#include <stdio.h>
#include <string.h>

/* Quiet: this suite drives no refusal paths, but the encoder logs an INFO per
 * successful switch and the assertions are easier to read without them. */
static void quiet_log(int level, const char *file, int line, const char *fmt, ...)
{
    (void)level;
    (void)file;
    (void)line;
    (void)fmt;
}

rss_hal_log_func_t rss_hal_log_fn = quiet_log;

static int failures;

#define CHECK(cond, fmt, ...)                                                                      \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);                    \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

/*
 * The channel the fakes describe: 1920x1080 at 25 fps, GOP 30, currently CBR.
 * Geometry matters because SetDefaultParam has to be told the channel's own --
 * a switch that invented a size would build a valid arm for the wrong picture.
 */
#define CHN_W 720
#define CHN_H 576
#define CHN_FPS 25
#define CHN_GOP 30
#define CHN_SCENE 7

static IMPEncoderAttrRcMode g_sent;
static int g_sent_calls;
static int g_idr_calls;

/* What SetChnQpBounds was asked for, and whether it was asked at all: a bound
 * the caller left to the channel has to arrive resolved, and one out of range
 * has to not arrive. */
static int g_bounds_min, g_bounds_max;
static int g_bounds_calls;

/* What SetDefaultParam was asked for, so the test can say the request was
 * shaped by the channel rather than by a constant. */
static IMPEncoderRcMode g_defparam_mode;
static uint16_t g_defparam_w, g_defparam_h;
static uint32_t g_defparam_gop;
static int g_defparam_scene;
static int g_defparam_qp;
static uint32_t g_defparam_br;
static int g_defparam_calls;

int IMP_Encoder_GetChnAttr(int encChn, IMPEncoderChnAttr *const attr)
{
    (void)encChn;
    memset(attr, 0, sizeof(*attr));
    attr->encAttr.eProfile = IMP_ENC_PROFILE_AVC_MAIN;
    attr->encAttr.uWidth = CHN_W;
    attr->encAttr.uHeight = CHN_H;
    attr->rcAttr.outFrmRate.frmRateNum = CHN_FPS;
    attr->rcAttr.outFrmRate.frmRateDen = 1;
    attr->gopAttr.uGopLength = CHN_GOP;
    attr->gopAttr.uMaxSameSenceCnt = CHN_SCENE;

    /*
     * The channel is in CBR, and its arm holds exactly the values that made
     * the original bug bite: deltas of -1, which land on VBR's iMinQP and
     * iMaxQP one uint32_t up the struct. Nothing should read this.
     */
    attr->rcAttr.attrRcMode.rcMode = IMP_ENC_RC_MODE_CBR;
    attr->rcAttr.attrRcMode.attrCbr.uTargetBitRate = 3000;
    attr->rcAttr.attrRcMode.attrCbr.iInitialQP = 38;
    attr->rcAttr.attrRcMode.attrCbr.iMinQP = 34;
    attr->rcAttr.attrRcMode.attrCbr.iMaxQP = 51;
    attr->rcAttr.attrRcMode.attrCbr.iIPDelta = -1;
    attr->rcAttr.attrRcMode.attrCbr.iPBDelta = -1;
    attr->rcAttr.attrRcMode.attrCbr.eRcOptions = IMP_ENC_RC_SCN_CHG_RES;
    attr->rcAttr.attrRcMode.attrCbr.uMaxPictureSize = 3000;
    return 0;
}

/*
 * Stands in for the vendor initialiser. The real one fills fields this header
 * gives no meaning to, which is the reason the code seeds from it rather than
 * assembling an arm by hand; here the useful part is the poison. Every byte is
 * set to a value no sane RC parameter takes, so any field the code under test
 * fails to write shows up in the assertions instead of accidentally matching.
 */
int IMP_Encoder_SetDefaultParam(IMPEncoderChnAttr *chnAttr, IMPEncoderProfile profile,
                                IMPEncoderRcMode rcMode, uint16_t uWidth, uint16_t uHeight,
                                uint32_t frmRateNum, uint32_t frmRateDen, uint32_t uGopLength,
                                int uMaxSameSenceCnt, int iInitialQP, uint32_t uTargetBitRate)
{
    (void)profile;
    (void)frmRateNum;
    (void)frmRateDen;

    g_defparam_calls++;
    g_defparam_mode = rcMode;
    g_defparam_w = uWidth;
    g_defparam_h = uHeight;
    g_defparam_gop = uGopLength;
    g_defparam_scene = uMaxSameSenceCnt;
    g_defparam_qp = iInitialQP;
    g_defparam_br = uTargetBitRate;

    memset(chnAttr, 0xA5, sizeof(*chnAttr));
    chnAttr->rcAttr.attrRcMode.rcMode = rcMode;
    return 0;
}

int IMP_Encoder_SetChnAttrRcMode(int encChn, const IMPEncoderAttrRcMode *pstRcModeCfg)
{
    (void)encChn;
    g_sent_calls++;
    g_sent = *pstRcModeCfg;
    return 0;
}

int IMP_Encoder_SetChnQpBounds(int encChn, int iMinQP, int iMaxQP)
{
    (void)encChn;
    g_bounds_calls++;
    g_bounds_min = iMinQP;
    g_bounds_max = iMaxQP;
    return 0;
}

int IMP_Encoder_RequestIDR(int encChn)
{
    (void)encChn;
    g_idr_calls++;
    return 0;
}

static void reset(void)
{
    memset(&g_sent, 0, sizeof(g_sent));
    g_sent_calls = 0;
    g_idr_calls = 0;
    g_defparam_calls = 0;
    g_bounds_calls = 0;
    g_bounds_min = g_bounds_max = 0;
}

/*
 * The bug, stated as the thing that must not happen: a VBR switch off a CBR
 * channel must not land the CBR deltas in VBR's QP bounds.
 *
 * -1 is the value to assert against rather than "something sensible", because
 * it is what the channel actually holds and what the encoder actually refused.
 */
static void test_a_vbr_switch_does_not_inherit_cbrs_bytes(void)
{
    reset();

    CHECK(hal_enc_set_rc_mode(NULL, 1, RSS_RC_VBR, 2000000) == 0, "the switch is accepted");
    CHECK(g_sent_calls == 1, "exactly one vendor call, got %d", g_sent_calls);
    CHECK(g_sent.rcMode == IMP_ENC_RC_MODE_VBR, "the tag says VBR, got %d", (int)g_sent.rcMode);

    CHECK(g_sent.attrVbr.iMinQP != -1, "iMinQP inherited CBR's iIPDelta");
    CHECK(g_sent.attrVbr.iMaxQP != -1, "iMaxQP inherited CBR's iPBDelta");
    CHECK(g_sent.attrVbr.iMinQP == 20, "iMinQP is the mode's own default, got %d",
          g_sent.attrVbr.iMinQP);
    CHECK(g_sent.attrVbr.iMaxQP == 45, "iMaxQP is the mode's own default, got %d",
          g_sent.attrVbr.iMaxQP);
    CHECK(g_sent.attrVbr.iMinQP < g_sent.attrVbr.iMaxQP, "and the bounds are the right way round");

    CHECK(g_sent.attrVbr.uTargetBitRate == 2000, "target is the caller's kbps, got %u",
          g_sent.attrVbr.uTargetBitRate);
    CHECK(g_sent.attrVbr.uMaxBitRate == 2000 * 4 / 3, "ceiling is 4/3 of target, got %u",
          g_sent.attrVbr.uMaxBitRate);
    CHECK(g_sent.attrVbr.uMaxPictureSize == 2000, "picture cap follows the target, got %u",
          g_sent.attrVbr.uMaxPictureSize);

    CHECK(g_idr_calls == 1, "and an IDR is requested so the change lands, got %d", g_idr_calls);
}

/*
 * The arm is seeded by the vendor's own initialiser, and told the channel's
 * geometry rather than a guess. Asserted because the alternative -- filling
 * the arm by hand -- looks equivalent from the outside and is not: the real
 * SetDefaultParam writes fields this header does not name.
 */
static void test_the_arm_is_seeded_for_this_channel(void)
{
    reset();

    CHECK(hal_enc_set_rc_mode(NULL, 0, RSS_RC_CAPPED_VBR, 4000000) == 0, "accepted");
    CHECK(g_defparam_calls == 1, "the initialiser ran once, got %d", g_defparam_calls);
    CHECK(g_defparam_mode == IMP_ENC_RC_MODE_CAPPED_VBR, "for the target mode, got %d",
          (int)g_defparam_mode);
    CHECK(g_defparam_w == CHN_W && g_defparam_h == CHN_H, "with the channel's size, got %ux%u",
          g_defparam_w, g_defparam_h);
    CHECK(g_defparam_gop == CHN_GOP, "and its GOP, got %u", g_defparam_gop);
    CHECK(g_defparam_scene == CHN_SCENE, "and its scene count, got %d", g_defparam_scene);
    CHECK(g_defparam_br == 4000, "and the caller's bitrate in kbps, got %u", g_defparam_br);
}

/*
 * Every mode gets a whole arm. The 0xA5 poison in the fake initialiser is what
 * gives this teeth: a field the switch forgets to write reads back as nonsense
 * rather than as a plausible leftover.
 */
static void test_every_mode_fills_its_own_arm(void)
{
    static const struct {
        rss_rc_mode_t mode;
        IMPEncoderRcMode vendor;
    } modes[] = {
        {RSS_RC_CBR, IMP_ENC_RC_MODE_CBR},
        {RSS_RC_VBR, IMP_ENC_RC_MODE_VBR},
        {RSS_RC_CAPPED_VBR, IMP_ENC_RC_MODE_CAPPED_VBR},
        {RSS_RC_CAPPED_QUALITY, IMP_ENC_RC_MODE_CAPPED_QUALITY},
    };
    size_t i;

    for (i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        int16_t lo, hi;
        uint32_t target;

        reset();
        CHECK(hal_enc_set_rc_mode(NULL, 1, modes[i].mode, 1000000) == 0, "mode %d accepted",
              (int)modes[i].mode);
        CHECK(g_sent.rcMode == modes[i].vendor, "mode %d: tag is %d", (int)modes[i].mode,
              (int)g_sent.rcMode);

        /* Every arm but FIXQP starts with target bitrate, then the bounds at
         * whatever offset that arm puts them -- read through the arm the tag
         * names, which is the discipline the fix is about. */
        switch (modes[i].vendor) {
        case IMP_ENC_RC_MODE_CBR:
            target = g_sent.attrCbr.uTargetBitRate;
            lo = g_sent.attrCbr.iMinQP;
            hi = g_sent.attrCbr.iMaxQP;
            break;
        case IMP_ENC_RC_MODE_VBR:
            target = g_sent.attrVbr.uTargetBitRate;
            lo = g_sent.attrVbr.iMinQP;
            hi = g_sent.attrVbr.iMaxQP;
            break;
        case IMP_ENC_RC_MODE_CAPPED_VBR:
            target = g_sent.attrCappedVbr.uTargetBitRate;
            lo = g_sent.attrCappedVbr.iMinQP;
            hi = g_sent.attrCappedVbr.iMaxQP;
            break;
        default:
            target = g_sent.attrCappedQuality.uTargetBitRate;
            lo = g_sent.attrCappedQuality.iMinQP;
            hi = g_sent.attrCappedQuality.iMaxQP;
            break;
        }

        CHECK(target == 1000, "mode %d: target %u kbps", (int)modes[i].mode, target);
        CHECK(lo >= 0 && lo <= 51, "mode %d: iMinQP %d is a QP", (int)modes[i].mode, lo);
        CHECK(hi >= 0 && hi <= 51, "mode %d: iMaxQP %d is a QP", (int)modes[i].mode, hi);
        CHECK(lo < hi, "mode %d: bounds %d..%d", (int)modes[i].mode, lo, hi);
    }
}

/*
 * A live mode change and a channel creation are the only two ways a channel
 * acquires an RC arm, and they have to leave the encoder in the same place.
 * They did not once: creation's CBR floor was lowered from 34 to 15 and this
 * path kept 34, so a 1 Mbps stream measured at 1.04 Mbps from boot fell to
 * 68 kbps the moment anything set its rate control mode -- the config
 * unchanged, the mode unchanged, only the floor. Pinned against the shared
 * constants rather than against numbers, because a literal here is exactly how
 * the two came apart.
 */
static void test_the_bounds_are_the_ones_creation_uses(void)
{
    reset();
    CHECK(hal_enc_set_rc_mode(NULL, 0, RSS_RC_CBR, 1000000) == 0, "cbr accepted");
    CHECK(g_sent.attrCbr.iMinQP == HAL_ENC_CBR_DEFAULT_MIN_QP, "cbr floor is creation's, got %d",
          g_sent.attrCbr.iMinQP);
    CHECK(g_sent.attrCbr.iMaxQP == HAL_ENC_CBR_DEFAULT_MAX_QP, "cbr ceiling is creation's, got %d",
          g_sent.attrCbr.iMaxQP);

    /* And the floor has to leave room to reach the target: at 34 the encoder
     * delivers whatever that quality costs, whatever the target says. */
    CHECK(g_sent.attrCbr.iMinQP < 34, "cbr floor leaves rate control room, got %d",
          g_sent.attrCbr.iMinQP);

    reset();
    CHECK(hal_enc_set_rc_mode(NULL, 0, RSS_RC_VBR, 1000000) == 0, "vbr accepted");
    CHECK(g_sent.attrVbr.iMinQP == HAL_ENC_VBR_DEFAULT_MIN_QP, "vbr floor is creation's, got %d",
          g_sent.attrVbr.iMinQP);
    CHECK(g_sent.attrVbr.iMaxQP == HAL_ENC_VBR_DEFAULT_MAX_QP, "vbr ceiling is creation's, got %d",
          g_sent.attrVbr.iMaxQP);

    reset();
    CHECK(hal_enc_set_rc_mode(NULL, 0, RSS_RC_CAPPED_VBR, 1000000) == 0, "capped_vbr accepted");
    CHECK(g_sent.attrCappedVbr.iMinQP == HAL_ENC_VBR_DEFAULT_MIN_QP,
          "capped_vbr floor is creation's, got %d", g_sent.attrCappedVbr.iMinQP);

    reset();
    CHECK(hal_enc_set_rc_mode(NULL, 0, RSS_RC_CAPPED_QUALITY, 1000000) == 0, "capped_q accepted");
    CHECK(g_sent.attrCappedQuality.iMinQP == HAL_ENC_VBR_DEFAULT_MIN_QP,
          "capped_quality floor is creation's, got %d", g_sent.attrCappedQuality.iMinQP);
}

/*
 * FIXQP is the arm with nothing but a QP in it, and the one the caller's
 * bitrate means nothing to. Its initial QP has to be a QP -- the poison would
 * be accepted by an encoder that does not range-check, and produce a picture
 * nobody asked for.
 */
static void test_fixqp_carries_a_qp_and_no_bitrate(void)
{
    reset();

    CHECK(hal_enc_set_rc_mode(NULL, 1, RSS_RC_FIXQP, 3000000) == 0, "accepted");
    CHECK(g_sent.rcMode == IMP_ENC_RC_MODE_FIXQP, "tag is FIXQP, got %d", (int)g_sent.rcMode);
    CHECK(g_sent.attrFixQp.iInitialQP >= 0 && g_sent.attrFixQp.iInitialQP <= 51,
          "iInitialQP %d is a QP", g_sent.attrFixQp.iInitialQP);
    CHECK(g_defparam_br == 0, "and the initialiser is told no bitrate, got %u", g_defparam_br);
    CHECK(g_defparam_qp == 35, "with an explicit QP rather than -1, got %d", g_defparam_qp);
}

/*
 * smart has no rate control of its own on this family and is mapped onto
 * capped VBR. Worth pinning: the map is in hal_translate_rc_mode, a caller
 * can ask for it, and an unmapped mode would fall through to CBR while the
 * reply still said "smart".
 */
static void test_smart_maps_onto_capped_vbr(void)
{
    reset();

    CHECK(hal_enc_set_rc_mode(NULL, 1, RSS_RC_SMART, 1000000) == 0, "accepted");
    CHECK(g_sent.rcMode == IMP_ENC_RC_MODE_CAPPED_VBR, "smart is capped VBR here, got %d",
          (int)g_sent.rcMode);
    CHECK(g_sent.attrCappedVbr.uTargetBitRate == 1000, "and carries the bitrate, got %u",
          g_sent.attrCappedVbr.uTargetBitRate);
}

/*
 * A zero bitrate is the caller having nothing to say, not a request for a zero
 * bitrate. It reaches here from a channel whose config never named one.
 */
static void test_a_zero_bitrate_falls_back(void)
{
    reset();

    CHECK(hal_enc_set_rc_mode(NULL, 1, RSS_RC_CBR, 0) == 0, "accepted");
    CHECK(g_sent.attrCbr.uTargetBitRate == 2000, "fell back rather than sending 0, got %u",
          g_sent.attrCbr.uTargetBitRate);
}

/*
 * hal_enc_set_qp_bounds takes the bounds as a pair because the vendor does,
 * but a config often names only one of them -- min_qp with the max left to the
 * mode. -1 is how that arrives, and it has to be resolved here: the vendor
 * call takes an int and would pass -1 straight through to the encoder.
 *
 * The channel the fakes describe is CBR 34..51, so an unset side comes back as
 * one of those.
 */
static void test_an_unset_qp_bound_keeps_the_channels_own(void)
{
    reset();
    CHECK(hal_enc_set_qp_bounds(NULL, 0, 20, -1) == 0, "accepted");
    CHECK(g_bounds_calls == 1, "sent once, got %d", g_bounds_calls);
    CHECK(g_bounds_min == 20, "the named bound is the caller's, got %d", g_bounds_min);
    CHECK(g_bounds_max == 51, "the unset one is the channel's, got %d", g_bounds_max);

    reset();
    CHECK(hal_enc_set_qp_bounds(NULL, 0, -1, 40) == 0, "accepted");
    CHECK(g_bounds_min == 34, "the unset floor is the channel's, got %d", g_bounds_min);
    CHECK(g_bounds_max == 40, "the named ceiling is the caller's, got %d", g_bounds_max);

    reset();
    CHECK(hal_enc_set_qp_bounds(NULL, 0, -1, -1) == 0, "accepted");
    CHECK(g_bounds_min == 34 && g_bounds_max == 51, "neither named leaves both, got %d..%d",
          g_bounds_min, g_bounds_max);
}

/*
 * Out of range is refused rather than clamped, and refused before the vendor
 * hears about it. A QP nobody can encode at is a mistake in the caller;
 * quietly moving it produces a stream that is merely not the one asked for,
 * which is harder to notice than an error return.
 */
static void test_a_qp_outside_the_range_is_refused(void)
{
    static const struct {
        int min_qp, max_qp;
        const char *what;
    } bad[] = {
        {-2, 40, "a floor below -1"},           {52, 40, "a floor above 51"},
        {20, 52, "a ceiling above 51"},         {20, -2, "a ceiling below -1"},
        {40, 20, "bounds the wrong way round"},
    };
    size_t i;

    for (i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        reset();
        CHECK(hal_enc_set_qp_bounds(NULL, 0, bad[i].min_qp, bad[i].max_qp) == RSS_ERR_INVAL,
              "%s is refused", bad[i].what);
        CHECK(g_bounds_calls == 0, "%s never reaches the encoder, got %d calls", bad[i].what,
              g_bounds_calls);
    }
}

int main(void)
{
    test_a_vbr_switch_does_not_inherit_cbrs_bytes();
    test_the_arm_is_seeded_for_this_channel();
    test_every_mode_fills_its_own_arm();
    test_the_bounds_are_the_ones_creation_uses();
    test_fixqp_carries_a_qp_and_no_bitrate();
    test_smart_maps_onto_capped_vbr();
    test_a_zero_bitrate_falls_back();
    test_an_unset_qp_bound_keeps_the_channels_own();
    test_a_qp_outside_the_range_is_refused();

    if (failures) {
        printf("t_enc_imp: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_enc_imp: ok\n");
    return 0;
}
