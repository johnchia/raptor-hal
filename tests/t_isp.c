/*
 * Host-side test of the pure logic in star/hal_isp.c.
 *
 * Includes the real translation unit rather than a copy, so the scaling
 * and field-access code under test is exactly what ships. MI itself is
 * never called: every test here drives the static helpers directly.
 */

#define PLATFORM_INFINITY6E 1
#define HAL_MODULE_VIDEO 1

/* The AE gate's real budgets are 2000/400 ms of polling. Shortened here so
 * the timeout paths are exercised in milliseconds rather than seconds --
 * what is under test is which branch a timeout takes, not how long it is
 * willing to wait. */
#define STAR_ISP_AE_TIMEOUT_MS 30
#define STAR_ISP_AE_QUICK_MS 20

#include "star/hal_isp.c"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

/*
 * The HAL's logging indirection, which libraptor_hal normally provides.
 * Silent here: these tests drive failure paths on purpose and the noise
 * would bury the results.
 *
 * Note the host build also suppresses the i6_*.h _Static_asserts via
 * -D'_Static_assert(c,m)='. They assert 32-bit pointer layouts on
 * structs none of these tests touch, and the real ARM build still
 * checks every one of them.
 */
/*
 * A real (silent) logger, not a NULL pointer: HAL_LOG_* call through this
 * without a NULL guard, because rss_hal_init always installs one. Any test
 * that exercises a warning path segfaults if this is left NULL.
 */
static void quiet_log(int level, const char *file, int line, const char *fmt, ...)
{
    (void)level;
    (void)file;
    (void)line;
    (void)fmt;
}

/*
 * Some behaviour of this backend IS a log line -- the AE lane
 * identification exists only to be read off a board log -- so one test
 * needs to see what was written rather than just that nothing crashed.
 */
static char g_log[8][256];
static unsigned int g_log_lines;

static void capture_log(int level, const char *file, int line, const char *fmt, ...)
{
    va_list ap;

    (void)level;
    (void)file;
    (void)line;
    if (g_log_lines >= 8)
        return;
    va_start(ap, fmt);
    vsnprintf(g_log[g_log_lines], sizeof(g_log[0]), fmt, ap);
    va_end(ap);
    g_log_lines++;
}

static const char *log_containing(const char *needle)
{
    for (unsigned int i = 0; i < g_log_lines; i++)
        if (strstr(g_log[i], needle))
            return g_log[i];
    return NULL;
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
 * The invariant that matters most: every field the table addresses must
 * lie wholly inside the payload MI copies. An offset past the end would
 * be written into our buffer and silently dropped -- or, if the payload
 * size were wrong the other way, MI would read past what we filled.
 */
static void test_table_bounds(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];

        CHECK(p->name && p->get_sym && p->set_sym, "entry %zu has a NULL string", i);
        CHECK(p->width == 1 || p->width == 2 || p->width == 4, "%s: width %u", p->name, p->width);
        CHECK(p->payload <= STAR_IQ_PAYLOAD_MAX, "%s: payload %u exceeds buffer", p->name,
              p->payload);
        CHECK((size_t)p->manual_off + p->width <= p->payload,
              "%s: field at %u+%u overruns the %u-byte payload", p->name, p->manual_off, p->width,
              p->payload);
        CHECK(p->mi_max > 0, "%s: mi_max is zero", p->name);
        CHECK(p->mi_unity <= p->mi_max, "%s: unity %u above max %u", p->name, p->mi_unity,
              p->mi_max);

        /*
         * An auto/manual entry must leave room for bEnable+enOpType, and
         * its two numbers have to agree with each other. The payload is
         * 8 + 16 auto blocks + one manual block of the same size, padded
         * out to the struct's 4-byte alignment -- so a manual offset fixes
         * the block size, and the block size fixes the payload. Both
         * numbers are functions of one, and a row where they disagree is
         * writing its level into stAuto.
         *
         * Worth having even though tests/abi_iq.c checks the same rows
         * against the vendor structs: this needs no SDK, and it is what
         * catches the arithmetic mistake rather than the transcription one.
         * temper's old 1288 implies an 80-byte block, which implies a
         * 1368-byte payload beside the 1776 the row also claimed.
         */
        if (p->shape == IQ_AUTOMAN) {
            unsigned int block = (p->manual_off - 8) / 16;
            unsigned int implied = (8 + 17 * block + 3) & ~3u;

            CHECK(p->manual_off >= 8, "%s: AUTOMAN manual offset %u below the 8-byte header",
                  p->name, p->manual_off);
            CHECK((p->manual_off - 8) % 16 == 0,
                  "%s: manual offset %u is not 8 plus 16 whole auto blocks", p->name,
                  p->manual_off);
            CHECK(p->payload == implied,
                  "%s: a manual offset of %u implies a %u-byte block and a %u-byte payload, "
                  "not %u",
                  p->name, p->manual_off, block, implied, p->payload);
        } else
            CHECK(p->manual_off == 0, "%s: FLAT/BOOL must live at offset 0, not %u", p->name,
                  p->manual_off);
    }
}

/* The documented payload sizes, restated here so a typo in the table is a
 * test failure rather than a silently wrong ioctl. Values from
 * disassembling libmi_isp.so; tests/abi_iq.c checks the same rows against
 * the vendor structs when the SDK is available, which this cannot. */
static void test_table_matches_disassembly(void)
{
    CHECK(g_iq[IQ_BRIGHTNESS].payload == 76 && g_iq[IQ_BRIGHTNESS].manual_off == 72, "brightness");
    CHECK(g_iq[IQ_CONTRAST].payload == 76 && g_iq[IQ_CONTRAST].manual_off == 72, "contrast");
    CHECK(g_iq[IQ_SATURATION].payload == 416 && g_iq[IQ_SATURATION].manual_off == 392,
          "saturation");
    CHECK(g_iq[IQ_SHARPNESS].payload == 1268 && g_iq[IQ_SHARPNESS].manual_off == 1192, "sharpness");
    CHECK(g_iq[IQ_SINTER].payload == 112 && g_iq[IQ_SINTER].manual_off == 104, "sinter");
    CHECK(g_iq[IQ_TEMPER].payload == 1776 && g_iq[IQ_TEMPER].manual_off == 1672, "temper");
    CHECK(g_iq[IQ_DEFOG].payload == 28, "defog");
    CHECK(g_iq[IQ_GRAY].payload == 4, "gray");
    CHECK(g_iq[IQ_EVCOMP].payload == 8, "evcomp");
    CHECK(g_iq[IQ_FLICKER].payload == 4, "flicker");

    /* Brightness is the entry that proves the convention: a u32 at 72 in
     * a 76-byte payload is exactly the last four bytes. */
    CHECK(g_iq[IQ_BRIGHTNESS].manual_off + g_iq[IQ_BRIGHTNESS].width ==
              g_iq[IQ_BRIGHTNESS].payload,
          "brightness manual field should be the payload's last 4 bytes");
}

/* Read-modify-write must not disturb a single byte outside the field. */
static void test_field_access_is_surgical(void)
{
    uint8_t buf[64], ref[64];
    size_t i;
    unsigned widths[] = { 1, 2, 4 };
    size_t w;

    for (w = 0; w < 3; w++) {
        unsigned width = widths[w];
        uint16_t off = 20;

        for (i = 0; i < sizeof(buf); i++)
            buf[i] = ref[i] = (uint8_t)(i * 7 + 1);

        star_iq_write(buf, off, (uint8_t)width, 0);
        for (i = 0; i < sizeof(buf); i++) {
            if (i >= off && i < off + width)
                continue;
            CHECK(buf[i] == ref[i], "width %u: byte %zu changed (%u -> %u)", width, i, ref[i],
                  buf[i]);
        }
    }

    /* Round-trip at every width, including values that must truncate. */
    memset(buf, 0, sizeof(buf));
    star_iq_write(buf, 4, 1, 200);
    CHECK(star_iq_read(buf, 4, 1) == 200, "u8 round-trip");
    star_iq_write(buf, 8, 2, 40000);
    CHECK(star_iq_read(buf, 8, 2) == 40000, "u16 round-trip");
    star_iq_write(buf, 12, 4, 3000000000u);
    CHECK(star_iq_read(buf, 12, 4) == 3000000000u, "u32 round-trip");

    /* Unaligned offsets must work -- manual offsets are not aligned in
     * general (NRLuma's is 104, but sharpness's manual block starts at
     * 1192 and its neighbours are byte fields). */
    memset(buf, 0, sizeof(buf));
    star_iq_write(buf, 3, 4, 0x01020304u);
    CHECK(star_iq_read(buf, 3, 4) == 0x01020304u, "unaligned u32 round-trip");
    star_iq_write(buf, 7, 2, 0xBEEF);
    CHECK(star_iq_read(buf, 7, 2) == 0xBEEF, "unaligned u16 round-trip");
}

/*
 * The scaling trap this test exists for: raptor's neutral must land on
 * MI's unity, not on the middle of MI's range. Saturation is the case
 * where those differ sharply -- unity is 32 of 127, so a linear map
 * would put neutral at 64 and double the colour on every default config.
 */
static void test_scale_neutral_is_unity(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];

        if (p->mi_unity == 0 || p->mi_unity >= p->mi_max)
            continue; /* bool/enum entries have no scale */

        CHECK(star_iq_scale(STAR_ISP_NEUTRAL, p->mi_unity, p->mi_max) == p->mi_unity,
              "%s: neutral 128 -> %u, expected unity %u", p->name,
              star_iq_scale(STAR_ISP_NEUTRAL, p->mi_unity, p->mi_max), p->mi_unity);
    }

    CHECK(star_iq_scale(128, 32, 127) == 32, "saturation neutral must be unity gain (32), not 64");
    CHECK(star_iq_scale(128, 50, 100) == 50, "brightness neutral");
    CHECK(star_iq_scale(128, 100, 200) == 100, "ev comp neutral means no compensation");
}

static void test_scale_endpoints_and_monotonicity(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];
        uint32_t prev;
        int v;

        if (p->mi_unity == 0 || p->mi_unity >= p->mi_max)
            continue;

        CHECK(star_iq_scale(0, p->mi_unity, p->mi_max) == 0, "%s: 0 -> 0", p->name);
        CHECK(star_iq_scale(255, p->mi_unity, p->mi_max) == p->mi_max, "%s: 255 -> max", p->name);

        /* Never decreasing, and never out of range. */
        prev = 0;
        for (v = 0; v <= 255; v++) {
            uint32_t got = star_iq_scale(v, p->mi_unity, p->mi_max);

            CHECK(got >= prev, "%s: not monotonic at %d (%u after %u)", p->name, v, got, prev);
            CHECK(got <= p->mi_max, "%s: %d -> %u exceeds max %u", p->name, v, got, p->mi_max);
            prev = got;
        }
    }

    /* Out-of-range input is clamped rather than wrapped. */
    CHECK(star_iq_scale(-40, 32, 127) == 0, "negative clamps to 0");
    CHECK(star_iq_scale(9999, 32, 127) == 127, "over-range clamps to max");
}

/*
 * Round-tripping matters because rvd can read a value back and write it
 * again. Exact identity is impossible where MI's range is coarser than
 * raptor's (brightness has 101 steps against 256), so the requirement is
 * that a scale/unscale round trip stays close and that the neutral point
 * is exact.
 */
static void test_unscale_round_trip(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];
        int v;

        if (p->mi_unity == 0 || p->mi_unity >= p->mi_max)
            continue;

        CHECK(star_iq_unscale(p->mi_unity, p->mi_unity, p->mi_max) == STAR_ISP_NEUTRAL,
              "%s: unity must read back as neutral", p->name);
        CHECK(star_iq_unscale(0, p->mi_unity, p->mi_max) == 0, "%s: 0 reads back as 0", p->name);
        CHECK(star_iq_unscale(p->mi_max, p->mi_unity, p->mi_max) == 255, "%s: max reads back as 255",
              p->name);

        for (v = 0; v <= 255; v += 5) {
            uint32_t mi = star_iq_scale(v, p->mi_unity, p->mi_max);
            int back = star_iq_unscale(mi, p->mi_unity, p->mi_max);
            int drift = back > v ? back - v : v - back;
            /* One MI step is worth 255/mi_max raptor steps; allow that
             * plus a rounding unit. */
            int tolerance = (int)(255 / p->mi_max) + 2;

            CHECK(drift <= tolerance, "%s: %d -> MI %u -> %d (drift %d > %d)", p->name, v, mi, back,
                  drift, tolerance);
        }
    }
}

/* Degenerate table entries must not divide by zero or misreport. */
static void test_scale_degenerate_inputs(void)
{
    CHECK(star_iq_scale(200, 5, 5) == 5, "unity == max short-circuits");
    CHECK(star_iq_unscale(0, 0, 0) == 255, "max 0 saturates rather than dividing by zero");
    /* In-range mi with a degenerate unity: falls back to neutral. An mi
     * at or above max saturates first, which is why this uses 5 not 99. */
    CHECK(star_iq_unscale(5, 10, 10) == STAR_ISP_NEUTRAL, "unity >= max reads back neutral");
    CHECK(star_iq_unscale(99, 10, 10) == 255, "mi above max saturates");

    /*
     * A unity of 0 is not degenerate -- it is what a learned baseline looks
     * like when the tuning sits at the bottom of MI's range. Neutral has to
     * still mean 0, the half below it collapse onto 0 because there is
     * nowhere lower, and the half above it must still reach max.
     */
    CHECK(star_iq_scale(STAR_ISP_NEUTRAL, 0, 200) == 0, "unity 0: neutral is 0");
    CHECK(star_iq_scale(64, 0, 200) == 0, "unity 0: below neutral collapses onto 0");
    CHECK(star_iq_scale(255, 0, 200) == 200, "unity 0: the top still reaches max");
    CHECK(star_iq_scale(192, 0, 200) > 0, "unity 0: above neutral still brightens");
    CHECK(star_iq_unscale(0, 0, 200) == STAR_ISP_NEUTRAL, "unity 0: MI 0 reads back neutral");
    CHECK(star_iq_unscale(100, 0, 200) > STAR_ISP_NEUTRAL,
          "unity 0: anything above it reads back above neutral");
}

/*
 * ae_comp's neutral is whatever the tuning binary left in the field, read
 * back once per load. Getting this wrong is not visible in a log: it
 * silently shifts what a default config does to the picture.
 */
static uint32_t g_unity_probe_value;
static int unity_probe_get(int chn, void *buf)
{
    (void)chn;
    memcpy(buf, &g_unity_probe_value, sizeof(g_unity_probe_value));
    return 0;
}

static void test_evcomp_neutral_comes_from_the_tuning(void)
{
    star_iq_param_t saved = g_iq[IQ_EVCOMP];
    star_state_t st;
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];

    memset(&st, 0, sizeof(st));
    st.isp_loaded = true;

    /* Both pointers set, so star_iq_resolve returns without a dlopen. */
    g_iq[IQ_EVCOMP].fn_get = unity_probe_get;
    g_iq[IQ_EVCOMP].fn_set = unity_probe_get;

    CHECK(g_iq[IQ_EVCOMP].unity_from_tuning, "ae_comp must be marked as learning its neutral");
    CHECK(!g_iq[IQ_EVCOMP].unity_stale, "and must not learn before a tuning load arms it");

    g_unity_probe_value = 20;
    CHECK(star_iq_fetch(&st, IQ_EVCOMP, buf) == RSS_OK, "unarmed fetch succeeds");
    CHECK(g_iq[IQ_EVCOMP].mi_unity == saved.mi_unity, "an unarmed fetch must not adopt a baseline");

    star_isp_arm_tuning_reads();
    CHECK(g_iq[IQ_EVCOMP].unity_stale, "a tuning load arms the read");
    CHECK(star_iq_fetch(&st, IQ_EVCOMP, buf) == RSS_OK, "armed fetch succeeds");
    CHECK(g_iq[IQ_EVCOMP].mi_unity == 20, "the tuning's value becomes the neutral, got %u",
          g_iq[IQ_EVCOMP].mi_unity);
    CHECK(!g_iq[IQ_EVCOMP].unity_stale, "and is read once, not on every fetch");

    /* The point of the exercise: raptor's neutral is now inert, and the
     * range below it darkens from the tuning's own value rather than from
     * a guess of 100. */
    CHECK(star_iq_scale(STAR_ISP_NEUTRAL, g_iq[IQ_EVCOMP].mi_unity, 200) == 20,
          "neutral writes the tuning's value straight back");
    CHECK(star_iq_scale(64, g_iq[IQ_EVCOMP].mi_unity, 200) == 10, "half of neutral halves it");
    CHECK(star_iq_scale(0, g_iq[IQ_EVCOMP].mi_unity, 200) == 0, "0 reaches the bottom");

    /* A reading outside the field's range means the offset or the width is
     * wrong, and adopting it would hide that for the rest of the run. */
    g_unity_probe_value = 4096;
    star_isp_arm_tuning_reads();
    CHECK(star_iq_fetch(&st, IQ_EVCOMP, buf) == RSS_OK, "an out-of-range fetch still succeeds");
    CHECK(g_iq[IQ_EVCOMP].mi_unity == 20, "an impossible baseline is refused, got %u",
          g_iq[IQ_EVCOMP].mi_unity);
    CHECK(!g_iq[IQ_EVCOMP].unity_stale, "and is not retried every frame");

    g_iq[IQ_EVCOMP] = saved;
}

/*
 * Anti-flicker is the one knob where raptor's enum and MI's differ in
 * order rather than in range: MI is DISABLE, 60HZ, 50HZ, AUTO. Passing the
 * value through therefore configures the other mains frequency, and does
 * it symmetrically -- the getter reads back the same swap, so a
 * round-trip through the control API looks correct while the picture bands
 * under 50 Hz lighting. This test is the round-trip that is not
 * self-consistent by construction: it names MI's numbers on both sides.
 */
static uint32_t g_flicker_raw;
static int flicker_get(int chn, void *buf)
{
    (void)chn;
    memcpy(buf, &g_flicker_raw, sizeof(g_flicker_raw));
    return 0;
}

static int flicker_set(int chn, void *buf)
{
    (void)chn;
    memcpy(&g_flicker_raw, buf, sizeof(g_flicker_raw));
    return 0;
}

static void test_antiflicker_translates_the_mains_frequency(void)
{
    star_iq_param_t saved = g_iq[IQ_FLICKER];
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    rss_antiflicker_t mode;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.isp_loaded = true;
    st.isp_tuned = true;

    g_iq[IQ_FLICKER].fn_get = flicker_get;
    g_iq[IQ_FLICKER].fn_set = flicker_set;

    CHECK(hal_isp_set_antiflicker(c, RSS_ANTIFLICKER_50HZ) == RSS_OK, "50 Hz must apply");
    CHECK(g_flicker_raw == 2, "50 Hz must reach MI as 2, got %u", g_flicker_raw);
    CHECK(hal_isp_set_antiflicker(c, RSS_ANTIFLICKER_60HZ) == RSS_OK, "60 Hz must apply");
    CHECK(g_flicker_raw == 1, "60 Hz must reach MI as 1, got %u", g_flicker_raw);
    CHECK(hal_isp_set_antiflicker(c, RSS_ANTIFLICKER_OFF) == RSS_OK, "off must apply");
    CHECK(g_flicker_raw == 0, "off must reach MI as 0, got %u", g_flicker_raw);

    /* And back, read from MI's numbers rather than from what was set. */
    g_flicker_raw = 2;
    CHECK(hal_isp_get_antiflicker(c, &mode) == RSS_OK, "get must succeed");
    CHECK(mode == RSS_ANTIFLICKER_50HZ, "MI's 2 is 50 Hz, got %d", (int)mode);
    g_flicker_raw = 1;
    CHECK(hal_isp_get_antiflicker(c, &mode) == RSS_OK, "get must succeed");
    CHECK(mode == RSS_ANTIFLICKER_60HZ, "MI's 1 is 60 Hz, got %d", (int)mode);

    /* MI's AUTO has no raptor enumerator, so it reads as off rather than
     * as a value raptor cannot hand back to its own setter. */
    g_flicker_raw = 3;
    CHECK(hal_isp_get_antiflicker(c, &mode) == RSS_OK, "get must succeed on AUTO");
    CHECK(mode == RSS_ANTIFLICKER_OFF, "MI's AUTO reads as off, got %d", (int)mode);

    CHECK(hal_isp_set_antiflicker(c, (rss_antiflicker_t)7) == RSS_ERR_INVAL,
          "an out-of-range mode is refused");

    g_iq[IQ_FLICKER] = saved;
}

static void test_pending_queue(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    uint8_t v8;
    uint32_t v32;
    int vi;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.isp_loaded = true;
    st.isp_tuned = false; /* ISP not answering yet */
    st.pend_max_again = -1;
    st.pend_max_dgain = -1;

    /* A set before the ISP is up must succeed and be remembered, not
     * fail -- rvd applies the whole [image] block at this point. */
    CHECK(hal_isp_set_saturation(c, 200) == RSS_OK, "set_saturation should queue, not fail");
    CHECK(g_iq[IQ_SATURATION].has_pending, "saturation should be queued");
    CHECK(g_iq[IQ_SATURATION].pending == 200, "queued value should be 200");
    CHECK(!g_iq[IQ_SATURATION].pending_is_raw, "saturation queues as a scalar");

    /* And reading it back must agree with what was asked for. */
    v8 = 0;
    CHECK(hal_isp_get_saturation(c, &v8) == RSS_OK, "get_saturation should succeed while queued");
    CHECK(v8 == 200, "queued saturation should read back as 200, got %u", v8);

    /* An untouched knob reads as neutral rather than failing. */
    v8 = 0;
    CHECK(hal_isp_get_brightness(c, &v8) == RSS_OK, "get_brightness should succeed");
    CHECK(v8 == STAR_ISP_NEUTRAL, "untouched brightness should read neutral, got %u", v8);

    /* Raw-valued params take the raw path. MI orders 60 Hz before 50 Hz,
     * so what gets queued is the translated value, not raptor's. */
    CHECK(hal_isp_set_antiflicker(c, RSS_ANTIFLICKER_50HZ) == RSS_OK, "set_antiflicker queues");
    CHECK(g_iq[IQ_FLICKER].has_pending && g_iq[IQ_FLICKER].pending_is_raw,
          "antiflicker queues as raw");
    CHECK(g_iq[IQ_FLICKER].pending == STAR_FLICKER_50HZ, "queued flicker value should be MI's 2");

    CHECK(hal_isp_set_defog(c, 1) == RSS_OK, "set_defog queues");
    CHECK(g_iq[IQ_DEFOG].has_pending && g_iq[IQ_DEFOG].pending == 1, "defog queued as 1");

    /* Gain ceilings live outside the table and queue in star_state. */
    CHECK(hal_isp_set_max_again(c, 160) == RSS_OK, "set_max_again queues");
    CHECK(st.pend_max_again == 160, "max_again queued, got %d", st.pend_max_again);
    CHECK(hal_isp_set_max_dgain(c, 80) == RSS_OK, "set_max_dgain queues");
    CHECK(st.pend_max_dgain == 80, "max_dgain queued, got %d", st.pend_max_dgain);

    v32 = 0;
    CHECK(hal_isp_get_max_again(c, &v32) == RSS_OK, "get_max_again succeeds while queued");
    CHECK(v32 == 160, "queued max_again reads back, got %u", v32);

    vi = 0;
    CHECK(hal_isp_set_ae_comp(c, 140) == RSS_OK, "set_ae_comp queues");
    CHECK(hal_isp_get_ae_comp(c, &vi) == RSS_OK, "get_ae_comp succeeds while queued");
    CHECK(vi == 140, "queued ae_comp reads back, got %d", vi);

    /* Flip goes to the sensor, not the ISP, so it is tracked regardless
     * of ISP readiness -- but with no MI_SNR loaded it must not crash. */
    CHECK(hal_isp_get_hvflip(c, &vi, NULL) == RSS_OK, "get_hvflip succeeds");
    CHECK(vi == 0, "flip defaults off");

    /* A NULL context must be rejected rather than dereferenced. */
    CHECK(hal_isp_set_saturation(NULL, 200) == RSS_ERR_INVAL, "NULL ctx rejected");

    /* Leave the table clean for any later test. */
    memset(&st, 0, sizeof(st));
    for (size_t i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

/*
 * Orientation. The VPE channel param carries both axes in one struct, so
 * setting one has to carry the other over -- get that wrong and enabling
 * vflip silently cancels an hflip that is already in effect. The whole path
 * is function pointers, so it tests without hardware.
 *
 * The mock answers a read with whatever the last write left, because the
 * op is a read-modify-write and the thing worth testing is that it does not
 * flatten the fields it is not there to change.
 */
static unsigned int vpe_para_set_calls;
static i6e_vpe_para g_vpe_para;
static int vpe_para_get_ret, vpe_para_set_ret;

static int fake_get_chn_param(int chn, i6_vpe_para *param)
{
    (void)chn;
    if (vpe_para_get_ret)
        return vpe_para_get_ret;
    memcpy(param, &g_vpe_para, sizeof(g_vpe_para));
    return 0;
}

static int fake_set_chn_param(int chn, i6_vpe_para *param)
{
    (void)chn;
    if (vpe_para_set_ret)
        return vpe_para_set_ret;
    vpe_para_set_calls++;
    memcpy(&g_vpe_para, param, sizeof(g_vpe_para));
    return 0;
}

static void test_orientation_carries_both_axes(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    int hf, vf;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    memset(&g_vpe_para, 0, sizeof(g_vpe_para));
    ctx.platform = &st;
    st.vpe.fnGetChannelParam = fake_get_chn_param;
    st.vpe.fnSetChannelParam = fake_set_chn_param;
    st.vpe_chn_created = true;
    /* What star_vpe_bringup left behind, and what must still be there after. */
    g_vpe_para.level3DNR = 1;
    vpe_para_set_calls = 0;
    vpe_para_get_ret = vpe_para_set_ret = 0;

    CHECK(hal_isp_set_hflip(c, 1) == RSS_OK, "set_hflip succeeds");
    CHECK(vpe_para_set_calls == 1, "set_hflip issues one write, got %u", vpe_para_set_calls);
    CHECK(g_vpe_para.mirror == 1 && g_vpe_para.flip == 0, "hflip alone -> (1,0), got (%d,%d)",
          g_vpe_para.mirror, g_vpe_para.flip);
    CHECK(g_vpe_para.level3DNR == 1, "the rest of the param must survive the write, 3DNR got %d",
          g_vpe_para.level3DNR);

    /* The one that would regress: vflip must not drop the live hflip. */
    CHECK(hal_isp_set_vflip(c, 1) == RSS_OK, "set_vflip succeeds");
    CHECK(g_vpe_para.mirror == 1 && g_vpe_para.flip == 1, "vflip must keep hflip -> (1,1), "
          "got (%d,%d)", g_vpe_para.mirror, g_vpe_para.flip);

    hf = vf = -1;
    CHECK(hal_isp_get_hvflip(c, &hf, &vf) == RSS_OK, "get_hvflip succeeds");
    CHECK(hf == 1 && vf == 1, "get_hvflip reports both set, got (%d,%d)", hf, vf);

    /* Clearing one leaves the other alone. */
    CHECK(hal_isp_set_hflip(c, 0) == RSS_OK, "clearing hflip succeeds");
    CHECK(g_vpe_para.mirror == 0 && g_vpe_para.flip == 1, "clearing hflip keeps vflip -> (0,1), "
          "got (%d,%d)", g_vpe_para.mirror, g_vpe_para.flip);

    /* Asking for what is already set costs no write. The SDK mutates a param
     * it is handed, so a redundant one is not free. */
    vpe_para_set_calls = 0;
    CHECK(hal_isp_set_vflip(c, 1) == RSS_OK, "a redundant set succeeds");
    CHECK(vpe_para_set_calls == 0, "a redundant set must not write, got %u", vpe_para_set_calls);

    /* A failed write must not leave the cache claiming a change that never
     * reached the hardware, or the next set would carry a lie. */
    vpe_para_set_ret = -1;
    CHECK(hal_isp_set_hflip(c, 1) != RSS_OK, "a failing write is reported");
    hf = -1;
    CHECK(hal_isp_get_hvflip(c, &hf, NULL) == RSS_OK, "get_hvflip still succeeds");
    CHECK(hf == 0, "a failed set must not be recorded, got %d", hf);

    /* Before the channel exists there is nothing to write to, and that is a
     * different answer from a missing symbol. */
    vpe_para_set_ret = 0;
    st.vpe_chn_created = false;
    CHECK(hal_isp_set_hflip(c, 1) == RSS_ERR_NOENT, "no channel yet is NOENT");

    st.vpe_chn_created = true;
    st.vpe.fnSetChannelParam = NULL;
    CHECK(hal_isp_set_hflip(c, 1) == RSS_ERR_NOTSUP, "a missing symbol is NOTSUP");
}

/*
 * Sensor framerate. The mode's range is the sensor's, not the config's, so a
 * request outside it is clamped rather than refused -- and the unit MI takes
 * depends on whether the request is a whole number of frames.
 */
static unsigned int g_fps_programmed;
static unsigned int g_fps_reported;
static int g_fps_set_ret, g_fps_get_ret;

static int fake_set_fps(unsigned int sensor, unsigned int fps)
{
    (void)sensor;
    if (g_fps_set_ret)
        return g_fps_set_ret;
    g_fps_programmed = fps;
    g_fps_reported = fps;
    return 0;
}

static int fake_get_fps(unsigned int sensor, unsigned int *fps)
{
    (void)sensor;
    if (g_fps_get_ret)
        return g_fps_get_ret;
    *fps = g_fps_reported;
    return 0;
}

static void test_sensor_fps_clamps_to_the_mode(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    uint32_t num, den;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.snr.fnSetFramerate = fake_set_fps;
    st.snr.fnGetFramerate = fake_get_fps;
    st.res.minFps = 5;
    st.res.maxFps = 30;
    g_fps_programmed = g_fps_reported = 0;
    g_fps_set_ret = g_fps_get_ret = 0;

    CHECK(hal_isp_set_sensor_fps(c, 15, 1) == RSS_OK, "an in-range rate must succeed");
    CHECK(g_fps_programmed == 15, "15 fps must reach MI as 15, got %u", g_fps_programmed);
    CHECK(st.fps == 15, "st.fps must follow, got %u", st.fps);

    /* Above and below the mode's range, clamped rather than refused. */
    CHECK(hal_isp_set_sensor_fps(c, 60, 1) == RSS_OK, "an over-range rate must still succeed");
    CHECK(g_fps_programmed == 30, "60 must clamp to the mode's 30, got %u", g_fps_programmed);
    CHECK(hal_isp_set_sensor_fps(c, 1, 1) == RSS_OK, "an under-range rate must still succeed");
    CHECK(g_fps_programmed == 5, "1 must clamp to the mode's 5, got %u", g_fps_programmed);

    /* A fractional rate goes as milli-frames, which is MI's other unit. */
    CHECK(hal_isp_set_sensor_fps(c, 30000, 1001) == RSS_OK, "29.97 must succeed");
    CHECK(g_fps_programmed == 29970, "29.97 must reach MI as 29970, got %u", g_fps_programmed);
    CHECK(st.fps == 30, "st.fps rounds for the frame period, got %u", st.fps);

    /* The getter reads hardware, and tells the two units apart by the
     * mode's own ceiling. */
    num = den = 0;
    CHECK(hal_isp_get_sensor_fps(c, &num, &den) == RSS_OK, "get must succeed");
    CHECK(num == 29970 && den == 1000, "a milli reading must report /1000, got %u/%u", num, den);
    g_fps_reported = 20;
    CHECK(hal_isp_get_sensor_fps(c, &num, &den) == RSS_OK, "get must succeed");
    CHECK(num == 20 && den == 1, "a whole reading must report /1, got %u/%u", num, den);

    /* No getter falls back to what was programmed rather than failing. */
    st.snr.fnGetFramerate = NULL;
    CHECK(hal_isp_get_sensor_fps(c, &num, &den) == RSS_OK, "get must fall back");
    CHECK(num == 30 && den == 1, "the fallback is the recorded rate, got %u/%u", num, den);

    /* A zero denominator is a caller error, not a division. */
    CHECK(hal_isp_set_sensor_fps(c, 30, 0) == RSS_ERR_INVAL, "a zero denominator is INVAL");
    CHECK(hal_isp_set_sensor_fps(c, 0, 1) == RSS_ERR_INVAL, "a zero rate is INVAL");

    /* A failing MI call is reported, and no symbol at all is NOTSUP. */
    g_fps_set_ret = -1;
    CHECK(hal_isp_set_sensor_fps(c, 15, 1) == RSS_ERR_IO, "a failing SetFps is io");
    st.snr.fnSetFramerate = NULL;
    CHECK(hal_isp_set_sensor_fps(c, 15, 1) == RSS_ERR_NOTSUP, "a missing symbol is NOTSUP");
}

/*
 * A tuning reload must be able to put the knobs back, so the recorded
 * values have to survive the flush that applies them.
 *
 * The failure this guards is rvd's hot restart (stream-restart,
 * set-resolution, set-codec, osd-restart): it stops the last VPE port,
 * which stops the VPE channel, which throws away the tuning binary and
 * every knob with it. A flush that consumed its queue would reload the
 * binary and silently leave the operator's settings at the binary's
 * defaults -- and a latch that outlived the channel would not reload the
 * binary at all.
 */
static void test_recorded_values_survive_a_reload(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    size_t i;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.isp_loaded = true;
    st.isp_tuned = false;
    st.pend_max_again = -1;
    st.pend_max_dgain = -1;
    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;

    CHECK(hal_isp_set_saturation(c, 200) == RSS_OK, "saturation is recorded");
    CHECK(hal_isp_set_max_again(c, 160) == RSS_OK, "max_again is recorded");

    /* No MI handle here, so the applies inside fail; what this test is
     * about is the state of the record afterwards. */
    star_isp_flush_pending(&st);

    CHECK(g_iq[IQ_SATURATION].has_pending, "the flush must not consume the record");
    CHECK(g_iq[IQ_SATURATION].pending == 200, "nor alter it, got %d", g_iq[IQ_SATURATION].pending);
    CHECK(st.pend_max_again == 160, "the gain ceiling survives the flush too, got %d",
          st.pend_max_again);

    /* A value set while the ISP *is* up must be recorded just the same, or
     * the reload after the next restart loses it. */
    st.isp_tuned = true;
    (void)hal_isp_set_sharpness(c, 90); /* the apply fails: no MI handle */
    CHECK(g_iq[IQ_SHARPNESS].has_pending && g_iq[IQ_SHARPNESS].pending == 90,
          "a live set is recorded for the next reload even when the apply fails");

    /* And losing the VPE channel must clear the latch claiming the tuning
     * is in effect, idempotently. */
    star_isp_untune(&st);
    CHECK(!st.isp_tuned, "untune clears the tuned latch");
    star_isp_untune(&st);
    CHECK(!st.isp_tuned, "untune is idempotent");

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

/*
 * The tuning load must not touch 3A at all.
 *
 * Stopping userspace 3A around the load and restarting CUS3A afterwards
 * leaves auto white balance enabled-but-dead: MI_ISP_DisableUserspace3A
 * unregisters the vendor algorithms and MI_ISP_CUS3A_Enable only sets
 * flags. divinus loads the binary and leaves 3A running, and divinus has
 * good colour on this board. So what this pins is that the load calls
 * nothing but the load.
 */
static unsigned int loadcfg_calls;

static int fake_loadcfg(int channel, char *path, unsigned int key)
{
    (void)channel;
    (void)path;
    (void)key;
    loadcfg_calls++;
    return 0;
}

static int fake_parainit_ready(int channel, i6_isp_parainit *status)
{
    (void)channel;
    status->ready = 1;
    return 0;
}

static void tune_once(star_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->isp_loaded = true;
    st->isp_tuned = false;
    st->pend_max_again = -1;
    st->pend_max_dgain = -1;
    st->fps = 30;
    snprintf(st->iq_file, sizeof(st->iq_file), "/etc/sensors/gc4653.bin");
    st->isp.fnGetParaInitStatus = fake_parainit_ready;
    st->isp.fnLoadChannelConfig = fake_loadcfg;

    loadcfg_calls = 0;
    star_isp_tune_when_ready(st, false);
}

static void test_tuning_load_leaves_3a_alone(void)
{
    star_state_t st;
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;

    tune_once(&st);
    CHECK(loadcfg_calls == 1, "the binary is loaded, got %u calls", loadcfg_calls);
    CHECK(st.isp_tuned, "the tuned latch is set");

    /* The hatch is gone, so neither symbol is bound any more. The test
     * that it cannot be reached is that i6_isp_impl has nowhere to put
     * one: this file would not compile if the fields came back unused. */

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

/*
 * The load has to wait for CUS3A's AE, not just for the IQ parameter
 * store.
 *
 * The AE's own init reads the generic iqfile from disk and takes its
 * limits from it, and that init is deferred to the first ISP frame
 * interrupt -- strictly after the readiness flag this used to be gated on.
 * So a load placed on the flag alone is always overwritten, which is the
 * whole reason the reload loop existed.
 *
 * What has to hold: an early attempt that finds no exposure yet must leave
 * the latch clear so a later call retries, and it must not have loaded --
 * a load that is going to be wiped is worse than no load, because it sets
 * the latch. A late attempt that still finds nothing loads anyway, because
 * an unexplained AE is not a reason to ship an untuned image.
 */
static unsigned int g_gate_shutter;
static unsigned int g_gate_calls;
static int g_gate_ret;

static int fake_gate_ae_status(int channel, i6_isp_ae_status *out)
{
    (void)channel;
    g_gate_calls++;
    if (g_gate_ret)
        return g_gate_ret;
    memset(out, 0, sizeof(*out));
    out->shutterUs = g_gate_shutter;
    return 0;
}

static void gate_setup(star_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->isp_loaded = true;
    st->pend_max_again = -1;
    st->pend_max_dgain = -1;
    st->fps = 30;
    snprintf(st->iq_file, sizeof(st->iq_file), "/etc/sensors/gc4653.bin");
    st->isp.fnGetParaInitStatus = fake_parainit_ready;
    st->isp.fnLoadChannelConfig = fake_loadcfg;
    st->isp.fnGetAeStatus = fake_gate_ae_status;

    loadcfg_calls = 0;
    g_gate_calls = 0;
    g_gate_ret = 0;
}

static void test_tuning_waits_for_the_ae(void)
{
    star_state_t st;
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;

    /* The witness itself: zero shutter is "not initialised", any exposure
     * at all is "running". */
    gate_setup(&st);
    g_gate_shutter = 0;
    CHECK(star_isp_wait_ae_running(&st, 30, false) == RSS_ERR_TIMEOUT,
          "a zero shutter must not satisfy the gate");
    g_gate_shutter = 1;
    CHECK(star_isp_wait_ae_running(&st, 30, false) == RSS_OK,
          "any exposure at all satisfies it");

    /* A failing call is not a running AE either -- it is the "sensor have
     * NOT been initialized" case, which is exactly when the load must not
     * go yet. */
    g_gate_ret = -1;
    CHECK(star_isp_wait_ae_running(&st, 30, false) == RSS_ERR_TIMEOUT,
          "a failing GetAeStatus must not satisfy the gate");
    g_gate_ret = 0;

    /* Returns as soon as the answer arrives rather than spending the
     * budget: 30 ms of budget is three polls, and one answer ends it. */
    g_gate_calls = 0;
    g_gate_shutter = 5000;
    CHECK(star_isp_wait_ae_running(&st, 30, false) == RSS_OK, "must succeed");
    CHECK(g_gate_calls == 1, "must stop at the first answer, polled %u times", g_gate_calls);

    /* The early attempt: no exposure yet, so nothing is loaded and nothing
     * is latched. */
    gate_setup(&st);
    g_gate_shutter = 0;
    star_isp_tune_when_ready(&st, false);
    CHECK(loadcfg_calls == 0, "an early attempt must not load before the AE runs, got %u",
          loadcfg_calls);
    CHECK(!st.isp_tuned, "and must leave the latch clear so a later call retries");

    /* Same state, AE now running: the retry loads. */
    g_gate_shutter = 8000;
    star_isp_tune_when_ready(&st, false);
    CHECK(loadcfg_calls == 1, "the retry must load, got %u calls", loadcfg_calls);
    CHECK(st.isp_tuned, "and latch");

    /* The late attempt with the AE still silent loads regardless. */
    gate_setup(&st);
    g_gate_shutter = 0;
    star_isp_tune_when_ready(&st, true);
    CHECK(loadcfg_calls == 1, "a late attempt must load anyway, got %u calls", loadcfg_calls);
    CHECK(st.isp_tuned, "and latch");

    /* No AE-status symbol is not a reason to refuse to tune. */
    gate_setup(&st);
    st.isp.fnGetAeStatus = NULL;
    star_isp_tune_when_ready(&st, false);
    CHECK(loadcfg_calls == 1, "an unbound witness must not block the load, got %u calls",
          loadcfg_calls);

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

/*
 * ── isp_get_exposure ──────────────────────────────────────────────────
 *
 * The AE grid layout is derived, not documented: i6_isp.h gets the cell
 * width from two wrappers' payload sizes and cannot place the eight spare
 * bytes, so hal_isp.c decides at runtime by matching the grid dimensions
 * from the AE status against both ends of the block. These tests cover
 * both placements, the case where neither matches (which must yield no
 * luma rather than a number from the wrong offset), and the fixed-point
 * gain arithmetic.
 */

static i6_isp_ae_status g_ae_status;
static int g_ae_status_ret;
static unsigned g_ae_status_calls;
static i6_isp_ae_hw_stats *g_ae_stats;
static int g_ae_stats_ret;
static unsigned g_ae_stats_calls;

static int fake_get_ae_status(int channel, i6_isp_ae_status *out)
{
    (void)channel;
    g_ae_status_calls++;
    if (g_ae_status_ret)
        return g_ae_status_ret;
    *out = g_ae_status;
    return 0;
}

static int fake_get_ae_hw_stats(int channel, i6_isp_ae_hw_stats *out)
{
    (void)channel;
    g_ae_stats_calls++;
    if (g_ae_stats_ret)
        return g_ae_stats_ret;
    *out = *g_ae_stats;
    return 0;
}

/* Fill `cells` cells from `cell` with a known y ramp, and everything past
 * them with a value the mean must not pick up. */
static uint32_t fill_grid(unsigned char *cell, unsigned int cells)
{
    unsigned int i;
    uint32_t sum = 0;

    memset(g_ae_stats, 0xEE, sizeof(*g_ae_stats));

    for (i = 0; i < cells; i++) {
        unsigned char y = (unsigned char)(10 + i * 7);

        cell[i * I6_ISP_AE_CELL_SZ + 0] = 1; /* r, g, b deliberately unlike y */
        cell[i * I6_ISP_AE_CELL_SZ + 1] = 2;
        cell[i * I6_ISP_AE_CELL_SZ + 2] = 3;
        cell[i * I6_ISP_AE_CELL_SZ + I6_ISP_AE_CELL_Y] = y;
        sum += y;
    }

    return sum / cells;
}

static void exposure_setup(rss_hal_ctx_t *ctx, star_state_t *st)
{
    memset(ctx, 0, sizeof(*ctx));
    memset(st, 0, sizeof(*st));
    ctx->platform = st;
    st->isp_loaded = true;
    st->isp_tuned = true;
    st->isp.fnGetAeStatus = fake_get_ae_status;
    st->isp.fnGetAeHwAvgStats = fake_get_ae_hw_stats;

    memset(&g_ae_status, 0, sizeof(g_ae_status));
    g_ae_status.shutterUs = 8333;
    g_ae_status.sensorGain = 2048; /* 2.0x */
    g_ae_status.ispGain = 1024;    /* 1.0x */
    g_ae_status.avgBlkX = 4;
    g_ae_status.avgBlkY = 2;
    g_ae_status_ret = 0;
    g_ae_stats_ret = 0;
    g_ae_status_calls = g_ae_stats_calls = 0;
}

static void test_exposure_waits_for_the_isp(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;

    exposure_setup(&ctx, &st);

    /* Untuned: the ISP channel does not exist yet, so no call and no
     * fabricated reading -- ric polls through this window every second. */
    st.isp_tuned = false;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_ERR_BUSY, "untuned ISP must report busy");
    CHECK(g_ae_status_calls == 0, "untuned ISP must not be queried, got %u calls",
          g_ae_status_calls);

    /* A library without the symbol is unsupported, not broken. */
    st.isp_tuned = true;
    st.isp.fnGetAeStatus = NULL;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_ERR_NOTSUP,
          "a missing AE status symbol must report notsup");

    st.isp.fnGetAeStatus = fake_get_ae_status;
    CHECK(hal_isp_get_exposure(&ctx, NULL) == RSS_ERR_INVAL, "a NULL exposure must report inval");

    /* Checked after the state, as everywhere else in this file. */
    st.isp_loaded = false;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_ERR_NOENT, "an unloaded ISP must report noent");
}

static void test_exposure_gain_is_x1024_fixed_point(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;

    exposure_setup(&ctx, &st);
    st.isp.fnGetAeHwAvgStats = NULL; /* gain only, luma is separate */

    /* 2.0x sensor by 4.0x ISP is 8.0x, and x1024 in means x1024 out. */
    g_ae_status.sensorGain = 2048;
    g_ae_status.ispGain = 4096;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a healthy read must succeed");
    CHECK(exp.total_gain == 8192, "2x by 4x is 8192 x1024, got %u", exp.total_gain);
    CHECK(exp.exposure_time == 8333, "shutter must pass through, got %u", exp.exposure_time);
    CHECK(exp.ae_luma == 0, "no stats call means no luma, got %u", exp.ae_luma);

    /* An unreported ISP gain is unity, not zero: multiplying by zero
     * would report no gain at all in the dark. */
    g_ae_status.ispGain = 0;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a zero ISP gain must still read");
    CHECK(exp.total_gain == 2048, "a zero ISP gain is unity, got %u", exp.total_gain);

    /* The product is 64-bit; u32 would wrap at 64x by 64x. */
    g_ae_status.sensorGain = 0xFFFFFFFFu;
    g_ae_status.ispGain = 2048;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "an extreme gain must still read");
    CHECK(exp.total_gain == UINT32_MAX, "an overflowing product must clamp, got %u",
          exp.total_gain);

    /* A failed status read is a failed call, not a zeroed reading. */
    g_ae_status_ret = -1;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_ERR_IO, "a failed AE status must report io");
}

static void test_exposure_luma_from_either_layout(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;
    uint32_t want;

    g_ae_stats = malloc(sizeof(*g_ae_stats));
    assert(g_ae_stats);

    /* Cells after the two dimension words. */
    exposure_setup(&ctx, &st);
    want = fill_grid(g_ae_stats->lead.cell, 8);
    g_ae_stats->lead.blkX = 4;
    g_ae_stats->lead.blkY = 2;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a lead-placed grid must read");
    CHECK(exp.ae_luma == want, "lead layout luma is %u, got %u", want, exp.ae_luma);

    /* Cells first, dimensions after. */
    exposure_setup(&ctx, &st);
    want = fill_grid(g_ae_stats->trail.cell, 8);
    g_ae_stats->trail.blkX = 4;
    g_ae_stats->trail.blkY = 2;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a trail-placed grid must read");
    CHECK(exp.ae_luma == want, "trail layout luma is %u, got %u", want, exp.ae_luma);

    free(g_ae_stats);
    g_ae_stats = NULL;
}

/*
 * The AE lane-identification line must not be spent on a frame that
 * cannot answer the question it exists to ask.
 *
 * This has now cost two board runs. The first logged r=0 g=0 b=0 y=0 from
 * an ISP that had just been reset to its defaults; the guard added for
 * that only rejected all-zero, so the second logged
 * r=253 g=252 b=253 y=253 -- a daylight scene clipped against the untuned
 * 300us shutter floor. Neither distinguishes any lane order from any
 * other, and both consumed the one shot for the process.
 *
 * The line is only worth writing on a frame with colour in it, and then
 * it should state a verdict rather than four numbers for someone to
 * weigh up by eye: with a genuine r,g,b,y layout, lane 3 is the BT.601
 * sum of the first three.
 */
static void fill_lanes(unsigned char *cell, unsigned int cells, unsigned char r, unsigned char g,
                       unsigned char b, unsigned char y)
{
    memset(g_ae_stats, 0xEE, sizeof(*g_ae_stats));

    for (unsigned int i = 0; i < cells; i++) {
        cell[i * I6_ISP_AE_CELL_SZ + 0] = r;
        cell[i * I6_ISP_AE_CELL_SZ + 1] = g;
        cell[i * I6_ISP_AE_CELL_SZ + 2] = b;
        cell[i * I6_ISP_AE_CELL_SZ + I6_ISP_AE_CELL_Y] = y;
    }
}

/* The grid the board reports. It has to be this big: the check scores
 * cells and declines to answer on fewer than STAR_AE_LANE_MIN_CELLS. */
#define LANE_PROBE_BLK_X 32
#define LANE_PROBE_BLK_Y 32

static void lane_probe(unsigned char r, unsigned char g, unsigned char b, unsigned char y)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;

    exposure_setup(&ctx, &st);

    /* The AE status is what says how big the grid is; the stats buffer has
     * to agree with it or the layout is treated as unconfirmed. */
    g_ae_status.avgBlkX = LANE_PROBE_BLK_X;
    g_ae_status.avgBlkY = LANE_PROBE_BLK_Y;
    fill_lanes(g_ae_stats->lead.cell, LANE_PROBE_BLK_X * LANE_PROBE_BLK_Y, r, g, b, y);
    g_ae_stats->lead.blkX = LANE_PROBE_BLK_X;
    g_ae_stats->lead.blkY = LANE_PROBE_BLK_Y;

    /* The "still ambiguous" note is once per process, so clear it between
     * probes -- each frame's own verdict is what is under test. */
    star_ae_lanes_ambiguous = false;

    g_log_lines = 0;
    rss_hal_log_fn = capture_log;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "the probe frame must read");
    rss_hal_log_fn = quiet_log;
}

static void test_ae_lane_identification_waits_for_a_frame_that_answers(void)
{
    g_ae_stats = malloc(sizeof(*g_ae_stats));
    assert(g_ae_stats);
    star_ae_lanes_identified = false;

    /*
     * The two frames that actually happened. Neither may confirm anything:
     * every order predicts an all-zero cell perfectly, and a clipped cell
     * is skipped outright, so the clipped frame scores none at all.
     */
    lane_probe(0, 0, 0, 0);
    CHECK(!log_containing("order is confirmed"), "an all-zero frame identifies nothing");
    lane_probe(253, 252, 253, 253);
    CHECK(!log_containing("AE lanes"), "a clipped frame scores no cells and says nothing");

    /* And the one that was reported as "should be good enough": a neutral
     * scene is unclipped and still cannot separate the lanes. r and g equal
     * means the orders that swap them predict identically. */
    lane_probe(46, 46, 44, 46);
    CHECK(!log_containing("order is confirmed"), "a neutral frame identifies nothing");
    CHECK(!star_ae_lanes_identified, "none of those may consume the one shot");

    /*
     * A coloured frame whose lane 3 really is BT.601 luma:
     * (299*180 + 587*90 + 114*40) / 1000 = 111, so r,g,b,y is off by 0 a cell
     * and the nearest rival (b,r,g) by 16.
     *
     * Asserted on the state rather than the log, because confirming the
     * assumed order is the uninteresting outcome and says so at debug level,
     * which compiles out without HAL_DEBUG. The state is what the rest of
     * the file depends on.
     */
    lane_probe(180, 90, 40, 111);
    CHECK(star_ae_lanes_identified, "a coloured frame must settle the order");
    CHECK(!log_containing("wrong bytes"), "and must not call the assumed order wrong");

    /* One shot: the answer does not change, so nothing is scored again. */
    lane_probe(180, 90, 40, 111);
    CHECK(!log_containing("AE lanes"), "the question is asked once per process");

    /*
     * A lane 3 that is not luma is the outcome that matters, so it warns
     * rather than being read as a confirmation. y = 40 is the b lane, so
     * g,b,r,y fits far better.
     */
    star_ae_lanes_identified = false;
    lane_probe(180, 90, 40, 40);
    CHECK(log_containing("another order fits better") != NULL,
          "a non-luma lane 3 must be called out: %s",
          log_containing("AE lanes") ? log_containing("AE lanes") : "(nothing logged)");
    CHECK(star_ae_lanes_identified, "and must still consume the one shot");

    free(g_ae_stats);
    g_ae_stats = NULL;
    star_ae_lanes_identified = false;
}

static void test_exposure_refuses_an_unconfirmed_layout(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;

    g_ae_stats = malloc(sizeof(*g_ae_stats));
    assert(g_ae_stats);

    /* Dimensions at neither end: the layout guess is wrong, so there is
     * no luma to report. Averaging offset 0 regardless is what this test
     * exists to prevent -- it would look like a reading and move the
     * IR-cut filter. */
    exposure_setup(&ctx, &st);
    fill_grid(g_ae_stats->trail.cell, 8);
    g_ae_stats->trail.blkX = 999;
    g_ae_stats->trail.blkY = 999;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "the gain half must still be reported");
    CHECK(exp.ae_luma == 0, "an unconfirmed layout must yield no luma, got %u", exp.ae_luma);
    CHECK(exp.total_gain == 2048, "gain must survive a luma failure, got %u", exp.total_gain);

    /* Dimensions the grid cannot hold are rejected before the stats call:
     * blk_x * blk_y bounds the scan, so an oversized pair would read past
     * the block. */
    exposure_setup(&ctx, &st);
    g_ae_status.avgBlkX = I6_ISP_AE_BLK_X + 1;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "impossible dimensions still read gain");
    CHECK(exp.ae_luma == 0, "impossible dimensions must yield no luma, got %u", exp.ae_luma);
    CHECK(g_ae_stats_calls == 0, "impossible dimensions must skip the stats call, got %u",
          g_ae_stats_calls);

    exposure_setup(&ctx, &st);
    g_ae_status.avgBlkY = 0;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a zero dimension still reads gain");
    CHECK(exp.ae_luma == 0, "a zero dimension must yield no luma, got %u", exp.ae_luma);

    /* A failed stats call loses the luma and keeps the rest. */
    exposure_setup(&ctx, &st);
    g_ae_stats_ret = -1;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a failed stats call must not fail the read");
    CHECK(exp.ae_luma == 0, "a failed stats call must yield no luma, got %u", exp.ae_luma);
    CHECK(exp.exposure_time == 8333, "shutter must survive a luma failure, got %u",
          exp.exposure_time);

    free(g_ae_stats);
    g_ae_stats = NULL;
}

/* ── AE exposure limits ── */

static i6_isp_exp g_limit;
static int g_limit_get_ret;
static int g_limit_set_ret;
static unsigned int g_limit_set_calls;

static int fake_get_exposure_limit(int chn, i6_isp_exp *cfg)
{
    (void)chn;
    if (g_limit_get_ret)
        return g_limit_get_ret;
    *cfg = g_limit;
    return 0;
}

static int fake_set_exposure_limit(int chn, i6_isp_exp *cfg)
{
    (void)chn;
    g_limit_set_calls++;
    if (g_limit_set_ret)
        return g_limit_set_ret;
    g_limit = *cfg; /* MI keeps it, so the next read-modify-write sees it */
    return 0;
}

/*
 * A deliberately tight tuning fixture: an 8x sensor ceiling and no
 * digital-gain headroom, so a clamp is easy to provoke.
 *
 * 8192 is not what gc4653.bin publishes -- that is 131072 -- it is what
 * the SSC30KQ's ISP reports with *no* tuning loaded. waybeam's board
 * happens to have a real bin ceiling of exactly 8192, and that
 * coincidence made a torn-down tuning look like a calibrated limit for a
 * whole night, so the reload test below states its own ceiling instead of
 * reusing this one.
 */
static void limit_setup(rss_hal_ctx_t *ctx, star_state_t *st)
{
    memset(ctx, 0, sizeof(*ctx));
    memset(st, 0, sizeof(*st));
    ctx->platform = st;
    st->isp_loaded = true;
    st->isp_tuned = true;
    st->pend_max_again = -1;
    st->pend_max_dgain = -1;
    st->fps = 30;
    st->isp.fnGetExposureLimit = fake_get_exposure_limit;
    st->isp.fnSetExposureLimit = fake_set_exposure_limit;

    memset(&g_limit, 0, sizeof(g_limit));
    g_limit.minShutterUs = 30;
    g_limit.maxShutterUs = 40000;
    g_limit.minSensorGain = 1024;
    g_limit.minIspGain = 1024;
    g_limit.maxSensorGain = 8192;
    g_limit.maxIspGain = 1024;
    g_limit_get_ret = g_limit_set_ret = 0;
    g_limit_set_calls = 0;
}

static void test_bin_limits_snapshot_records_the_tuning(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    limit_setup(&ctx, &st);
    star_isp_snapshot_bin_limits(&st);

    CHECK(st.bin_min_sensor_gain == 1024, "sensor floor should be 1024, got %u",
          st.bin_min_sensor_gain);
    CHECK(st.bin_max_sensor_gain == 8192, "sensor ceiling should be 8192, got %u",
          st.bin_max_sensor_gain);
    CHECK(st.bin_max_isp_gain == 1024, "isp ceiling should be 1024, got %u", st.bin_max_isp_gain);
    CHECK(g_limit_set_calls == 0, "a snapshot must only read, got %u writes", g_limit_set_calls);

    /* An AE that has not published yet answers all zeros. Recording that
     * would install a calibrated ceiling of zero and clamp every later
     * request to it, so it has to stay unrecorded. */
    limit_setup(&ctx, &st);
    memset(&g_limit, 0, sizeof(g_limit));
    star_isp_snapshot_bin_limits(&st);
    CHECK(st.bin_max_sensor_gain == 0, "an unpublished ceiling must not be recorded, got %u",
          st.bin_max_sensor_gain);
}

static void test_gain_ceiling_refuses_ingenic_units(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    limit_setup(&ctx, &st);
    star_isp_snapshot_bin_limits(&st);
    g_limit_set_calls = 0;

    /*
     * rvd's Ingenic defaults, which it applies on every platform whether
     * or not the config mentions the keys. In MI's x1024 units 160 is
     * 0.16x and 80 is 0.08x -- ceilings below unity, so not ceilings.
     */
    CHECK(star_isp_apply_gain_limit(&st, true, 160) == RSS_ERR_INVAL,
          "an Ingenic max_again must be refused");
    CHECK(star_isp_apply_gain_limit(&st, false, 80) == RSS_ERR_INVAL,
          "an Ingenic max_dgain must be refused");
    CHECK(g_limit_set_calls == 0, "a refused ceiling must not be written, got %u writes",
          g_limit_set_calls);
    CHECK(g_limit.maxSensorGain == 8192, "the tuning's sensor ceiling must stand, got %u",
          g_limit.maxSensorGain);
    CHECK(g_limit.maxIspGain == 1024,
          "the tuning's isp ceiling must stand -- writing 80 here is what pinned digital gain "
          "and capped total_gain at the sensor's 8192, got %u",
          g_limit.maxIspGain);
}

static void test_gain_ceiling_clamps_to_the_tuning(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    limit_setup(&ctx, &st);
    star_isp_snapshot_bin_limits(&st);

    /* waybeam's wall: gainMax 32000 against a bin ceiling of 8192, which
     * MI silently declines. Clamping makes it visible instead. */
    CHECK(star_isp_apply_gain_limit(&st, true, 32000) == RSS_OK, "a high ceiling must apply");
    CHECK(g_limit.maxSensorGain == 8192, "32000 must clamp to 8192, got %u",
          g_limit.maxSensorGain);

    CHECK(star_isp_apply_gain_limit(&st, true, 4096) == RSS_OK, "an in-range ceiling must apply");
    CHECK(g_limit.maxSensorGain == 4096, "4096 must pass through, got %u", g_limit.maxSensorGain);

    /* The gain writes share their struct with the shutter cap, which is
     * why they are read-modify-write. */
    CHECK(g_limit.maxShutterUs == 40000, "the shutter cap must survive a gain write, got %u",
          g_limit.maxShutterUs);

    /* A ceiling under the tuning's own floor is raised to it rather than
     * written as a maximum below the AE's minimum. */
    st.bin_min_sensor_gain = 2048;
    CHECK(star_isp_apply_gain_limit(&st, true, 1024) == RSS_OK, "a low ceiling must apply");
    CHECK(g_limit.maxSensorGain == 2048, "1024 must rise to the 2048 floor, got %u",
          g_limit.maxSensorGain);
}

static void test_shutter_cap_holds_the_frame_period(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    limit_setup(&ctx, &st);

    /* 25 fps is a 40000 us period and the tuning already sits there. */
    CHECK(star_isp_cap_exposure(&st, 25) == RSS_OK, "an in-range shutter must succeed");
    CHECK(g_limit_set_calls == 0, "nothing to cap means no write, got %u", g_limit_set_calls);

    /* At 30 fps the tuning's 40000 us overruns the 33333 us period. */
    CHECK(star_isp_cap_exposure(&st, 30) == RSS_OK, "an overrunning shutter must be capped");
    CHECK(g_limit.maxShutterUs == 33333, "shutter must cap to 33333, got %u",
          g_limit.maxShutterUs);
    CHECK(g_limit.maxSensorGain == 8192, "capping the shutter must leave the gains alone, got %u",
          g_limit.maxSensorGain);

    /* A failed read is an IO error, not a silently uncapped shutter. */
    limit_setup(&ctx, &st);
    g_limit_get_ret = -1;
    CHECK(star_isp_cap_exposure(&st, 25) == RSS_ERR_IO, "a failed limit read must report io");
}

static void test_limits_are_reasserted_when_configured(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    /*
     * Nothing configured: CUS3A narrowing its own window is its business,
     * and fighting an algorithm over values nobody asked for is how you
     * get an AE that oscillates. Must be checked first -- the unconfigured
     * path returns before the re-assert interval is armed.
     */
    limit_setup(&ctx, &st);
    g_limit.maxSensorGain = 4096;
    star_isp_reassert_limits(&st);
    CHECK(g_limit_set_calls == 0, "an unconfigured ceiling must not be re-asserted, got %u writes",
          g_limit_set_calls);
    CHECK(g_limit.maxSensorGain == 4096, "the AE's own window must stand, got %u",
          g_limit.maxSensorGain);

    /*
     * Configured and then narrowed behind our back, which is the board's
     * actual failure: the ceiling written at tuning load, a narrower one
     * live ninety seconds later with the AE pinned on it.
     */
    limit_setup(&ctx, &st);
    st.pend_max_again = 8192;
    st.bin_max_sensor_gain = 8192;
    g_limit.maxSensorGain = 4096;
    g_limit.minSensorGain = 1024;
    star_isp_reassert_limits(&st);
    CHECK(g_limit.maxSensorGain == 8192, "a configured ceiling must be restored, got %u",
          g_limit.maxSensorGain);
    CHECK(g_limit.maxShutterUs == 40000, "restoring the gain must not touch the shutter, got %u",
          g_limit.maxShutterUs);
}

/*
 * The tear-down repair: something puts this ISP back on its defaults after
 * the tuning loads, and the AE's own ceilings are the witness.
 *
 * The board numbers are the fixture here -- a bin that publishes 131072
 * against an untuned default of 8192 -- because the whole mechanism rests
 * on those two being distinguishable.
 */
static void reload_setup(rss_hal_ctx_t *ctx, star_state_t *st)
{
    size_t i;

    limit_setup(ctx, st);
    snprintf(st->iq_file, sizeof(st->iq_file), "/etc/sensors/gc4653.bin");
    st->isp.fnLoadChannelConfig = fake_loadcfg;
    st->bin_min_sensor_gain = 1024;
    st->bin_max_sensor_gain = 131072;
    st->bin_min_isp_gain = 1024;
    st->bin_max_isp_gain = 1024;
    g_limit.maxSensorGain = 131072;
    loadcfg_calls = 0;

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

static void test_the_tuning_is_reloaded_when_the_isp_resets(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    int i;

    /* The tuning's own ceiling is in effect: nothing to repair. */
    reload_setup(&ctx, &st);
    star_isp_reload_if_reset(&st, true);
    CHECK(loadcfg_calls == 0, "an intact tuning must not be reloaded, got %u", loadcfg_calls);

    /* Back on the untuned default, which is the observed failure. */
    reload_setup(&ctx, &st);
    g_limit.maxSensorGain = 8192;
    star_isp_reload_if_reset(&st, true);
    CHECK(loadcfg_calls == 1, "a reset ISP must be reloaded once, got %u", loadcfg_calls);
    CHECK(st.iq_reloads == 1, "the attempt must be counted, got %d", st.iq_reloads);

    /*
     * A configured ceiling is a legitimate reading, not evidence of a
     * reset. The first version of this bailed out entirely whenever one
     * was set, which quietly turned max_again into a switch that disabled
     * the repair -- while the config was recommending max_again be set.
     */
    reload_setup(&ctx, &st);
    st.pend_max_again = 32768;
    g_limit.maxSensorGain = 32768;
    star_isp_reload_if_reset(&st, true);
    CHECK(loadcfg_calls == 0, "a configured ceiling must not read as a reset, got %u",
          loadcfg_calls);

    /* ...and with one configured, a reset is still repaired, and the
     * ceiling the load just wiped goes back on. */
    reload_setup(&ctx, &st);
    st.pend_max_again = 32768;
    g_limit.maxSensorGain = 8192;
    star_isp_reload_if_reset(&st, true);
    CHECK(loadcfg_calls == 1, "a reset must be repaired even with a ceiling set, got %u",
          loadcfg_calls);
    CHECK(g_limit.maxSensorGain == 32768, "the reload must re-apply the configured ceiling, got %u",
          g_limit.maxSensorGain);

    /* No snapshot means no witness, so no reload -- guessing would reload
     * the binary on every poll of an AE that never published limits. */
    reload_setup(&ctx, &st);
    st.bin_max_sensor_gain = 0;
    g_limit.maxSensorGain = 8192;
    star_isp_reload_if_reset(&st, true);
    CHECK(loadcfg_calls == 0, "no snapshot must mean no reload, got %u", loadcfg_calls);

    /*
     * Bounded. Last, because giving up is reported once per process and
     * the latch is deliberately not resettable.
     */
    reload_setup(&ctx, &st);
    g_limit.maxSensorGain = 8192;
    for (i = 0; i < 8; i++) {
        star_isp_reload_if_reset(&st, true);
        g_limit.maxSensorGain = 8192; /* the reset keeps winning */
    }
    CHECK(loadcfg_calls == 5, "reloads must stop at 5, got %u", loadcfg_calls);
    CHECK(st.iq_reloads == 5, "the counter must stop at 5, got %d", st.iq_reloads);
}

static void touch_file(const char *dir, const char *leaf)
{
    char path[256];
    FILE *f;

    snprintf(path, sizeof(path), "%s/%s", dir, leaf);
    f = fopen(path, "w");
    assert(f);
    fclose(f);
}

/*
 * The tuning file is found by searching, and the order is the contract:
 * a rootfs carrying both layouts must resolve to the first directory, and
 * one carrying only the second must still resolve. Retargeting the search
 * at temporary directories is the only way to assert either, since the
 * real paths are absolute and not writable by a test.
 */
static void test_iq_file_search_order(void)
{
    char dir_a[] = "/tmp/raptor-iq-a-XXXXXX";
    char dir_b[] = "/tmp/raptor-iq-b-XXXXXX";
    const char *saved_a = star_iq_dirs[0];
    const char *saved_b = star_iq_dirs[1];
    rss_sensor_config_t cfg;
    star_state_t st;
    char out[128];
    char expect[128];

    /*
     * The directories themselves, before they are retargeted. Asserting
     * the search order alone would not notice these being reordered or
     * renamed, which is the half of the contract the distributions see.
     */
    CHECK(sizeof(star_iq_dirs) / sizeof(star_iq_dirs[0]) == 2, "expected two search directories");
    CHECK(strcmp(saved_a, "/etc/sensors") == 0, "first directory must be /etc/sensors, got %s",
          saved_a);
    CHECK(strcmp(saved_b, "/usr/share/sensor") == 0,
          "second directory must be /usr/share/sensor, got %s", saved_b);

    assert(mkdtemp(dir_a));
    assert(mkdtemp(dir_b));
    star_iq_dirs[0] = dir_a;
    star_iq_dirs[1] = dir_b;

    memset(&st, 0, sizeof(st));
    memset(&cfg, 0, sizeof(cfg));
    /* Upper case on purpose: MI reports "GC4653", the file is gc4653.bin. */
    snprintf(cfg.name, sizeof(cfg.name), "GC4653");

    CHECK(!star_isp_resolve_iq(&st, &cfg, out, sizeof(out)),
          "no tuning file anywhere must not resolve");
    CHECK(out[0] == '\0', "a failed resolve must leave the path empty");

    /* Only the second directory has it: the fallback has to be reached. */
    touch_file(dir_b, "gc4653.bin");
    snprintf(expect, sizeof(expect), "%s/gc4653.bin", dir_b);
    CHECK(star_isp_resolve_iq(&st, &cfg, out, sizeof(out)), "second directory must resolve");
    CHECK(strcmp(out, expect) == 0, "expected %s, got %s", expect, out);

    /* Both have it: the first wins. */
    touch_file(dir_a, "gc4653.bin");
    snprintf(expect, sizeof(expect), "%s/gc4653.bin", dir_a);
    CHECK(star_isp_resolve_iq(&st, &cfg, out, sizeof(out)), "first directory must resolve");
    CHECK(strcmp(out, expect) == 0, "search order broken: expected %s, got %s", expect, out);

    /*
     * With no [sensor] name, the module name the backend read during
     * bring-up is what resolves the file. This is the path a stock image
     * takes, since no board config names the sensor.
     */
    cfg.name[0] = '\0';
    snprintf(st.sensor_name, sizeof(st.sensor_name), "gc4653");
    snprintf(expect, sizeof(expect), "%s/gc4653.bin", dir_a);
    CHECK(star_isp_resolve_iq(&st, &cfg, out, sizeof(out)),
          "the detected module name must resolve the file");
    CHECK(strcmp(out, expect) == 0, "expected %s, got %s", expect, out);

    /* MI's own spelling is the last resort, and is spelled its way. */
    st.sensor_name[0] = '\0';
    st.snr_enabled = true;
    snprintf(st.plane.sensName, sizeof(st.plane.sensName), "GC4653");
    CHECK(star_isp_resolve_iq(&st, &cfg, out, sizeof(out)),
          "MI's sensor name must resolve the file when nothing else names it");
    CHECK(strcmp(out, expect) == 0, "expected %s, got %s", expect, out);
    st.snr_enabled = false;
    st.plane.sensName[0] = '\0';

    snprintf(expect, sizeof(expect), "%s/gc4653.bin", dir_a);
    remove(expect);
    snprintf(expect, sizeof(expect), "%s/gc4653.bin", dir_b);
    remove(expect);
    rmdir(dir_a);
    rmdir(dir_b);
    star_iq_dirs[0] = saved_a;
    star_iq_dirs[1] = saved_b;
}

int main(void)
{
    test_table_bounds();
    test_table_matches_disassembly();
    test_field_access_is_surgical();
    test_scale_neutral_is_unity();
    test_scale_endpoints_and_monotonicity();
    test_unscale_round_trip();
    test_scale_degenerate_inputs();
    test_evcomp_neutral_comes_from_the_tuning();
    test_antiflicker_translates_the_mains_frequency();
    test_pending_queue();
    test_orientation_carries_both_axes();
    test_sensor_fps_clamps_to_the_mode();
    test_recorded_values_survive_a_reload();
    test_tuning_load_leaves_3a_alone();
    test_tuning_waits_for_the_ae();
    test_exposure_waits_for_the_isp();
    test_exposure_gain_is_x1024_fixed_point();
    test_exposure_luma_from_either_layout();
    test_ae_lane_identification_waits_for_a_frame_that_answers();
    test_exposure_refuses_an_unconfirmed_layout();
    test_bin_limits_snapshot_records_the_tuning();
    test_gain_ceiling_refuses_ingenic_units();
    test_gain_ceiling_clamps_to_the_tuning();
    test_shutter_cap_holds_the_frame_period();
    test_limits_are_reasserted_when_configured();
    test_the_tuning_is_reloaded_when_the_isp_resets();
    test_iq_file_search_order();

    if (failures) {
        printf("\n%d check(s) failed\n", failures);
        return 1;
    }

    printf("all hal_isp logic tests passed\n");
    return 0;
}
