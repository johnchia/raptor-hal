/*
 * t_enc_imp.c -- what hal_enc_set_rc_mode sends the Ingenic encoder.
 *
 * One claim, and it is the whole suite: switching rate-control mode must build
 * the target mode's attributes from nothing, never from the channel's current
 * ones.
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

int main(void)
{
    test_a_vbr_switch_does_not_inherit_cbrs_bytes();
    test_the_arm_is_seeded_for_this_channel();
    test_every_mode_fills_its_own_arm();
    test_fixqp_carries_a_qp_and_no_bitrate();
    test_smart_maps_onto_capped_vbr();
    test_a_zero_bitrate_falls_back();

    if (failures) {
        printf("t_enc_imp: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_enc_imp: ok\n");
    return 0;
}
