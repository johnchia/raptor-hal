/*
 * Host-side test of the argument checks in the Ingenic src/hal_isp.c.
 *
 * This suite exists because there was not one. t_isp.c covers Infinity6E and
 * t_isp_i6c.c covers Infinity6C, so the fork had tests for its own silicon and
 * none for upstream's -- and that is exactly where the defect it was written
 * for lived. RSS_ISP_AUTO is INT_MIN, every setter here began by clamping, and
 * a clamp turns INT_MIN into a perfectly ordinary 0. `set-saturation auto`
 * answered ok and produced a greyscale image on every T-series part, while the
 * two SigmaStar backends refused it correctly.
 *
 * Includes the real translation unit rather than a copy, so what is tested is
 * what ships. Built for T31, which is the middle generation: scalar SDK args,
 * no IMPVI_NUM. The guard being tested sits above the #ifdef that chooses
 * between generations, so one build covers all of them for this claim -- and
 * the ARM build compiles the file for each of the ten platforms anyway.
 *
 * libimp is not here and is not needed: see imp_stubs.c, which all but one
 * vendor entry point resolves to and which aborts if a test ever reaches one.
 * The exception is SetBrightness, faked below, because half of what is claimed
 * here is that a legal value still arrives.
 */

#define PLATFORM_T31 1
#define HAL_MODULE_VIDEO 1

#include "hal_isp.c"

#include <stdio.h>
#include <string.h>

/*
 * hal_caps.c is not part of this suite -- it is a table of per-SoC constants
 * with no logic to test -- but hal_isp.c declares the symbol, so the link needs
 * one. Zeroed: nothing below reads it.
 */
const rss_hal_caps_t g_hal_caps;

/* The HAL's logging indirection, which libraptor_hal normally provides. Silent:
 * these tests drive refusal paths on purpose and the noise would bury them. */
static void quiet_log(int level, const char *file, int line, const char *fmt, ...)
{
    (void)level;
    (void)file;
    (void)line;
    (void)fmt;
}

rss_hal_log_func_t rss_hal_log_fn = quiet_log;

/*
 * The one vendor entry point this suite fakes for real rather than stubbing out
 * -- see imp_stubs.c, where it is deliberately absent.
 *
 * Everything else aborts on being reached, which is right for a suite whose
 * claims are all about refusing before the SDK. But "refuses the sentinel" is
 * only half the contract; the other half is that a legal value still gets
 * through, and that cannot be shown by a call that never arrives. So this one
 * records what it was handed.
 *
 * The prototype is T31's -- Gen2, a scalar unsigned char -- and has to match
 * the header, which this translation unit does include.
 */
static int g_bright_calls;
static unsigned char g_bright_last;

int IMP_ISP_Tuning_SetBrightness(unsigned char bright)
{
    g_bright_calls++;
    g_bright_last = bright;
    return 0;
}

static int failures;

#define CHECK(cond, fmt, ...)                                                                      \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);                    \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

/*
 * Every knob that can be handed RSS_ISP_AUTO must refuse it.
 *
 * The list is the [image] knobs rvd drives through ISP_IF_ASKED plus their
 * per-sensor variants, which is the whole set a config file or a raptorctl
 * command can reach. Named individually rather than looped through the vtable
 * so that adding a setter and forgetting the guard fails here rather than
 * quietly widening the hole.
 *
 * A test that reaches the SDK aborts in imp_stubs.c, so a passing CHECK here is
 * also evidence the guard returned before the vendor call rather than after it.
 */
static void test_every_knob_refuses_auto(void)
{
    CHECK(hal_isp_set_brightness(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "brightness");
    CHECK(hal_isp_set_contrast(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "contrast");
    CHECK(hal_isp_set_saturation(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "saturation");
    CHECK(hal_isp_set_sharpness(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "sharpness");
    CHECK(hal_isp_set_hue(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "hue");
    CHECK(hal_isp_set_sinter_strength(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "sinter");
    CHECK(hal_isp_set_temper_strength(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "temper");
    CHECK(hal_isp_set_dpc_strength(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "dpc");
    CHECK(hal_isp_set_drc_strength(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "drc");
    CHECK(hal_isp_set_defog_strength(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "defog");
    CHECK(hal_isp_set_highlight_depress(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "highlight");
    CHECK(hal_isp_set_backlight_comp(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "backlight");
    CHECK(hal_isp_set_ae_comp(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL, "ae_comp");

    CHECK(hal_isp_set_brightness_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "brightness_n");
    CHECK(hal_isp_set_contrast_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "contrast_n");
    CHECK(hal_isp_set_saturation_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "saturation_n");
    CHECK(hal_isp_set_sharpness_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "sharpness_n");
    CHECK(hal_isp_set_hue_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "hue_n");
    CHECK(hal_isp_set_sinter_strength_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "sinter_n");
    CHECK(hal_isp_set_temper_strength_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "temper_n");
    CHECK(hal_isp_set_ae_comp_n(NULL, 0, RSS_ISP_AUTO) == RSS_ERR_INVAL, "ae_comp_n");
}

/*
 * ae_comp is the one that could not be caught by a clamp even in principle: it
 * takes a bare int and handed it straight to the SDK, so the sentinel arrived
 * as INT_MIN rather than as 0. Worth its own test because the fix for the
 * others -- refuse before clamping -- reads like it is about the clamp, and a
 * later simplification that folded the guard into hal_clamp_u8 would silently
 * drop this one.
 */
static void test_ae_comp_refuses_auto_though_it_never_clamps(void)
{
    CHECK(hal_isp_set_ae_comp(NULL, RSS_ISP_AUTO) == RSS_ERR_INVAL,
          "the unclamped knob must refuse the sentinel too");
}

/*
 * The refusal must be exactly the sentinel and nothing near it. INT_MIN + 1 is
 * still a nonsense brightness, but it is nonsense the clamp is entitled to
 * absorb -- turning it into a refusal instead would be a second, undocumented
 * contract.
 */
static void test_only_the_sentinel_is_refused(void)
{
    g_bright_calls = 0;

    CHECK(hal_isp_set_brightness(NULL, RSS_ISP_AUTO + 1) != RSS_ERR_INVAL,
          "INT_MIN + 1 is not the sentinel and must not be refused as one");
    CHECK(g_bright_calls == 1, "and must still reach the SDK, %d calls", g_bright_calls);
    CHECK(g_bright_last == 0, "clamped to the floor, got %u", g_bright_last);
}

/*
 * The guard must not have cost the ordinary path. Easy to get wrong in the
 * direction that looks safe -- a refusal that fires one value too wide, or a
 * guard placed after the clamp so it never matches -- and both leave every
 * other test here passing.
 */
static void test_a_legal_value_still_reaches_the_sdk(void)
{
    g_bright_calls = 0;

    CHECK(hal_isp_set_brightness(NULL, 140) == 0, "a legal value must be accepted");
    CHECK(g_bright_calls == 1, "exactly one vendor call, got %d", g_bright_calls);
    CHECK(g_bright_last == 140, "carrying the value unchanged, got %u", g_bright_last);

    CHECK(hal_isp_set_brightness(NULL, 0) == 0, "the floor is legal");
    CHECK(g_bright_last == 0, "and arrives as 0, got %u", g_bright_last);

    CHECK(hal_isp_set_brightness(NULL, 255) == 0, "the ceiling is legal");
    CHECK(g_bright_last == 255, "and arrives as 255, got %u", g_bright_last);
}

/*
 * And the capability answer must agree with the refusal, which is the whole
 * claim: a client reads caps to decide whether to offer the word at all, so
 * has_auto false and a refusing setter have to be the same fact stated twice.
 *
 * The inverse -- a knob reporting has_auto true here -- would be the real bug,
 * because nothing on this family has an auto/manual op_type to hand back to.
 */
static void test_caps_never_claim_an_auto_this_family_does_not_have(void)
{
    static const char *const knobs[] = {"brightness",
                                        "contrast",
                                        "saturation",
                                        "sharpness",
                                        "hue",
                                        "sinter",
                                        "temper",
                                        "dpc_strength",
                                        "drc_strength",
                                        "defog_strength",
                                        "highlight_depress",
                                        "backlight_comp"};
    size_t i;

    for (i = 0; i < sizeof(knobs) / sizeof(knobs[0]); i++) {
        rss_isp_knob_t caps;

        CHECK(hal_isp_get_knob_caps(NULL, knobs[i], &caps) == RSS_OK, "%s has caps", knobs[i]);
        CHECK(!caps.has_auto, "%s must not claim an auto mode", knobs[i]);
        CHECK(caps.enabled, "%s: IMP has no per-module enable to be false", knobs[i]);
        CHECK(caps.min <= caps.neutral && caps.neutral <= caps.max, "%s: neutral %d outside %d..%d",
              knobs[i], caps.neutral, caps.min, caps.max);
    }
}

/*
 * A knob the platform does not describe answers NOTSUP rather than inventing a
 * range. ae_comp is the live example and the reason the distinction matters:
 * IMP_ISP_Tuning_SetAeComp takes a bare int with no documented bound, so the
 * table deliberately has no row for it, and a client is told "no better
 * information" rather than handed a guess.
 */
static void test_an_undescribed_knob_says_so(void)
{
    rss_isp_knob_t caps;

    CHECK(hal_isp_get_knob_caps(NULL, "ae_comp", &caps) == RSS_ERR_NOTSUP,
          "ae_comp has no documented range on this family and must not claim one");
    CHECK(hal_isp_get_knob_caps(NULL, "no_such_knob", &caps) == RSS_ERR_NOTSUP,
          "an unknown name is NOTSUP");
    CHECK(hal_isp_get_knob_caps(NULL, NULL, &caps) == RSS_ERR_INVAL, "NULL name is refused");
    CHECK(hal_isp_get_knob_caps(NULL, "brightness", NULL) == RSS_ERR_INVAL, "NULL caps is refused");
}

/*
 * The two rows whose neutral is not the midpoint, asserted by name. A client
 * that centres its control on the neutral gets these wrong unless told, and
 * "off" is a different neutral from "half".
 */
static void test_the_strengths_whose_neutral_is_off(void)
{
    rss_isp_knob_t caps;

    CHECK(hal_isp_get_knob_caps(NULL, "highlight_depress", &caps) == RSS_OK, "caps");
    CHECK(caps.neutral == 0, "highlight_depress is off at neutral, got %d", caps.neutral);

    CHECK(hal_isp_get_knob_caps(NULL, "backlight_comp", &caps) == RSS_OK, "caps");
    CHECK(caps.neutral == 0, "backlight_comp is off at neutral, got %d", caps.neutral);
    CHECK(caps.max == 10, "backlight_comp is the vendor's 0..10, got 0..%d", caps.max);
}

int main(void)
{
    test_every_knob_refuses_auto();
    test_ae_comp_refuses_auto_though_it_never_clamps();
    test_only_the_sentinel_is_refused();
    test_a_legal_value_still_reaches_the_sdk();
    test_caps_never_claim_an_auto_this_family_does_not_have();
    test_an_undescribed_knob_says_so();
    test_the_strengths_whose_neutral_is_off();

    if (failures) {
        printf("t_isp_imp: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_isp_imp: ok\n");
    return 0;
}
