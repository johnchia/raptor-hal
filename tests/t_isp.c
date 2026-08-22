/*
 * Host-side test of the pure logic in star/hal_isp.c.
 *
 * Includes the real translation unit rather than a copy, so the scaling
 * and field-access code under test is exactly what ships. MI itself is
 * never called: every test here drives the static helpers directly.
 */

#define PLATFORM_INFINITY6E 1
#define HAL_MODULE_VIDEO 1

#include "star/hal_isp.c"

#include <assert.h>
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
        CHECK(p->mi_unity <= p->mi_max, "%s: unity %d above max %d", p->name, p->mi_unity,
              p->mi_max);
        CHECK(p->mi_floor <= 0, "%s: a floor above zero makes no sense, got %d", p->name,
              p->mi_floor);
        CHECK(p->mi_unity >= p->mi_floor, "%s: unity %d below floor %d", p->name, p->mi_unity,
              p->mi_floor);
        /* star_iq_read_field sign-extends by casting, which is only the
         * whole story at four bytes. A narrower signed field would read
         * back as a large positive and the range check would reject it. */
        CHECK(p->mi_floor == 0 || p->width == 4,
              "%s: a signed field must be 4 bytes wide, not %u", p->name, p->width);

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
            unsigned int block = 0, b;

            /*
             * Derived from the payload rather than from the offset, because
             * the offset is no longer obliged to sit at the start of the
             * manual block: drc writes one Strength byte 43 into WDR's entry.
             * So find the block size the payload implies, then require the
             * offset to land inside the manual entry rather than on its front
             * edge. Still catches what the older form caught -- temper's old
             * 1288 against a 1776 payload gives block 104 and a manual entry
             * of [1672, 1776), which 1288 is nowhere near.
             */
            CHECK(p->manual_off >= 8, "%s: AUTOMAN manual offset %u below the 8-byte header",
                  p->name, p->manual_off);
            for (b = 1; b < p->payload; b++)
                if (((8 + 17 * b + 3) & ~3u) == p->payload) {
                    block = b;
                    break;
                }
            CHECK(block != 0, "%s: payload %u is not 8 + 17 blocks for any block size", p->name,
                  p->payload);
            if (block) {
                CHECK(p->manual_off >= 8 + 16 * block && p->manual_off < 8 + 17 * block,
                      "%s: manual offset %u is outside the manual entry [%u, %u) that a "
                      "%u-byte payload implies -- a level written there lands in stAuto",
                      p->name, p->manual_off, 8 + 16 * block, 8 + 17 * block, p->payload);
                CHECK(p->manual_off + p->width <= p->payload,
                      "%s: a %u-byte field at %u runs past the %u-byte payload", p->name, p->width,
                      p->manual_off, p->payload);
            }
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
    /*
     * WDR, and the only row whose level is not the front of its manual entry.
     * 892 is what the wrapper declares (mov.w r3, #892 at 0x84d4 in
     * libmi_isp.so's MI_ISP_IQ_GetWDR) and what every shipped bin's block
     * measures; 883 is 8 + 16*52 + 43, the Strength byte inside the manual
     * entry. Restated here because a transcription slip in either number is a
     * write into a neighbouring field that no picture would obviously show.
     */
    CHECK(g_iq[IQ_DRC].payload == 892 && g_iq[IQ_DRC].manual_off == 883, "drc");
    CHECK(g_iq[IQ_DRC].mi_unity == 128 && g_iq[IQ_DRC].mi_max == 255 && g_iq[IQ_DRC].mi_floor == 0,
          "drc must map 1:1, which is what makes a majestic overrideWdr value mean the same "
          "thing here");
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
 * The property that replaced the scaler, and the reason it was worth
 * replacing: a value written is the value read back, exactly, for every value
 * a knob accepts.
 *
 * There used to be an abstract 0..255 in front of MI's range and a pair of
 * functions mapping between them, and this file used to test that pair hard --
 * that neutral landed on unity rather than the midpoint, that the halves were
 * monotonic, that degenerate unities did not divide by zero. All of that was
 * careful work in service of a mapping that could not round-trip, because 256
 * inputs do not fit one-to-one onto a range of 101 or 41. Set brightness to
 * 140 and it read back 138; set ae_comp to 140 and it read back 134.
 *
 * The knobs now speak MI's units, so the mapping is gone and so are its
 * tests. What is left to check is that nothing crept back in.
 */
static void test_a_written_value_reads_back_unchanged(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];
        int v;

        if (p->shape != IQ_AUTOMAN && p->shape != IQ_FLAT)
            continue;

        for (v = p->mi_floor; v <= p->mi_max; v++) {
            uint8_t buf[STAR_IQ_PAYLOAD_MAX];
            int back;

            memset(buf, 0, sizeof(buf));
            star_iq_write(buf, p->manual_off, p->width, (uint32_t)v);
            back = (int)star_iq_read_field(p, buf);
            /* Signed rows come back as the two's-complement pattern; the
             * field is what MI reads, so compare on that footing. */
            if (p->mi_floor < 0 && p->width == 4)
                back = (int32_t)back;
            CHECK(back == v, "%s: wrote %d, field holds %d", p->name, v, back);
        }
    }
}

/*
 * Auto has to be sayable and distinguishable, which is the other half of the
 * change. Reporting a module in auto as "128" was a claim about the picture
 * that a caller could not tell apart from a knob deliberately set to 128 --
 * and once 128 stopped being special, it could not be spelled at all.
 */
static void test_auto_is_out_of_band(void)
{
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++) {
        const star_iq_param_t *p = &g_iq[i];

        CHECK(RSS_ISP_AUTO < p->mi_floor,
              "%s: RSS_ISP_AUTO must not collide with a value the field accepts", p->name);
    }
}

/*
 * The caps a client is given and the values the setter takes are the same two
 * numbers, and they have to stay that way: a control drawn from caps that the
 * daemon then rejects is worse than no control at all.
 *
 *   asked for                 set_scalar   caps say
 *   ------------------------  -----------  ---------------------
 *   mi_floor                  accepted     min
 *   mi_max                    accepted     max
 *   mi_floor - 1              refused      below min
 *   mi_max + 1                refused      above max
 *   RSS_ISP_AUTO, has auto    accepted     has_auto true
 *   RSS_ISP_AUTO, no auto     refused      has_auto false
 */
static void test_caps_describe_what_the_setter_accepts(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    void *c = &ctx;
    rss_isp_knob_t caps;
    const star_iq_param_t *p = &g_iq[IQ_DRC];

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.isp_loaded = true;
    st.isp_tuned = false; /* queue rather than call MI */

    CHECK(hal_isp_get_knob_caps(c, "drc_strength", &caps) == RSS_OK, "drc_strength has caps");
    CHECK(caps.min == p->mi_floor && caps.max == p->mi_max, "caps carry the field's own range");
    CHECK(caps.has_auto, "drc is an auto/manual module");

    CHECK(hal_isp_set_drc_strength(c, caps.min) == RSS_OK, "the published minimum is accepted");
    CHECK(hal_isp_set_drc_strength(c, caps.max) == RSS_OK, "the published maximum is accepted");
    CHECK(hal_isp_set_drc_strength(c, caps.min - 1) == RSS_ERR_INVAL, "below the minimum refused");
    CHECK(hal_isp_set_drc_strength(c, caps.max + 1) == RSS_ERR_INVAL, "above the maximum refused");
    CHECK(hal_isp_set_drc_strength(c, RSS_ISP_AUTO) == RSS_OK, "auto accepted where caps allow it");

    /* Temper is not published on this SoC at all. Neither route to temporal
     * denoise is a strength raptor can offer: the VPE channel level picks a
     * reference bit depth rather than an amount, and the tuning's NR3D module
     * costs its own per-gain curve to write. So no caps, not invented ones. */
    CHECK(hal_isp_get_knob_caps(c, "temper", &caps) == RSS_ERR_NOTSUP,
          "temper is not published on this SoC");

    /* A knob this platform does not publish has no caps rather than
     * invented ones -- 6E withdrew these because every shipped tuning
     * varies them across gain. */
    CHECK(hal_isp_get_knob_caps(c, "saturation", &caps) == RSS_ERR_NOTSUP,
          "an unpublished knob reports no caps");
    CHECK(hal_isp_get_knob_caps(c, "nonsense", &caps) == RSS_ERR_NOTSUP, "an unknown name too");

    memset(&st, 0, sizeof(st));
    for (size_t i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
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
    CHECK(!g_iq[IQ_EVCOMP].tuning_stale, "and must not learn before a tuning load arms it");

    g_unity_probe_value = 20;
    CHECK(star_iq_fetch(&st, IQ_EVCOMP, buf) == RSS_OK, "unarmed fetch succeeds");
    CHECK(g_iq[IQ_EVCOMP].mi_unity == saved.mi_unity, "an unarmed fetch must not adopt a baseline");

    star_isp_arm_tuning_reads();
    CHECK(g_iq[IQ_EVCOMP].tuning_stale, "a tuning load arms the read");
    CHECK(star_iq_fetch(&st, IQ_EVCOMP, buf) == RSS_OK, "armed fetch succeeds");
    CHECK(g_iq[IQ_EVCOMP].mi_unity == 20, "the tuning's value becomes the neutral, got %u",
          g_iq[IQ_EVCOMP].mi_unity);
    CHECK(!g_iq[IQ_EVCOMP].tuning_stale, "and is read once, not on every fetch");

    /* The point of the exercise: the learned value is what the caps report
     * as neutral, so a client centres its control where the tuner did. */
    {
        rss_hal_ctx_t caps_ctx;
        rss_isp_knob_t caps;

        memset(&caps_ctx, 0, sizeof(caps_ctx));
        caps_ctx.platform = &st;
        CHECK(hal_isp_get_knob_caps(&caps_ctx, "ae_comp", &caps) == RSS_OK, "ae_comp has caps");
        CHECK(caps.neutral == 20, "the learned baseline is the published neutral, got %d",
              caps.neutral);
        CHECK(caps.min < 0 && caps.min == -caps.max,
              "and the range stays symmetric about it, got %d..%d", caps.min, caps.max);
    }

    /* A reading outside the field's range means the offset or the width is
     * wrong, and adopting it would hide that for the rest of the run. */
    g_unity_probe_value = 4096;
    star_isp_arm_tuning_reads();
    CHECK(star_iq_fetch(&st, IQ_EVCOMP, buf) == RSS_OK, "an out-of-range fetch still succeeds");
    CHECK(g_iq[IQ_EVCOMP].mi_unity == 20, "an impossible baseline is refused, got %d",
          g_iq[IQ_EVCOMP].mi_unity);
    CHECK(!g_iq[IQ_EVCOMP].tuning_stale, "and is not retried every frame");

    /* And a negative baseline is legitimate for this row, where it would be
     * a misread for any other. */
    g_unity_probe_value = (uint32_t)(-7);
    star_isp_arm_tuning_reads();
    CHECK(star_iq_fetch(&st, IQ_EVCOMP, buf) == RSS_OK, "a negative baseline fetch succeeds");
    CHECK(g_iq[IQ_EVCOMP].mi_unity == -7, "a negative baseline is adopted, got %d",
          g_iq[IQ_EVCOMP].mi_unity);

    g_iq[IQ_EVCOMP] = saved;
}

/*
 * ae_comp is the one knob whose MI field is signed, and the whole of its
 * lower half depended on that being expressed.
 *
 * Under the old abstract scale this was subtle. The tunings we ship leave
 * s32EV at 0, so the learned baseline was 0, and with an unsigned range that
 * mapped raptor's whole 0..127 onto MI's 0..0 -- on the board, ae_comp 0 and
 * 64 were indistinguishable from 128. Half the knob did nothing. The fix then
 * was to give the row a negative floor so the mapping had somewhere to go.
 *
 * In MI's own units the failure cannot be expressed: -20 is -20. What is
 * still worth pinning is that the row is signed and symmetric, that the
 * signed field survives the write/read round trip as a negative number, and
 * that the published range says so -- because a client drawing a 0-based
 * slider over EV compensation is the same bug wearing a different hat.
 */
static void test_ae_comp_is_signed_end_to_end(void)
{
    const star_iq_param_t *p = &g_iq[IQ_EVCOMP];
    uint8_t buf[STAR_IQ_PAYLOAD_MAX];
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_isp_knob_t caps;
    int v;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;

    CHECK(p->mi_floor < 0, "ae_comp's floor must be negative, got %d", p->mi_floor);
    CHECK(p->mi_floor == -p->mi_max, "and symmetric about the baseline, got %d..%d", p->mi_floor,
          p->mi_max);
    CHECK(p->unity_from_tuning, "with the baseline still learned from the tuning");

    CHECK(hal_isp_get_knob_caps(&ctx, "ae_comp", &caps) == RSS_OK, "ae_comp publishes caps");
    CHECK(caps.min == p->mi_floor && caps.max == p->mi_max,
          "the published range is the field's, got %d..%d", caps.min, caps.max);
    CHECK(caps.min < 0, "and reaches below zero -- the whole point");
    CHECK(!caps.has_auto, "EV compensation is not an auto/manual module");
    /*
     * And it is not reported as a switched-off module. A flat row has no
     * bEnable to read: offset zero holds the value, and every tuning we ship
     * leaves EV at 0, so reading it as an enable would call the knob disabled
     * on every camera.
     */
    CHECK(caps.enabled, "a flat row has no enable bit to be false");

    /* Every EV step, negative ones included, survives the field intact. */
    for (v = p->mi_floor; v <= p->mi_max; v++) {
        int back;

        memset(buf, 0, sizeof(buf));
        star_iq_write(buf, p->manual_off, p->width, (uint32_t)v);
        back = (int32_t)star_iq_read_field(p, buf);
        CHECK(back == v, "EV %d must survive the field, got %d", v, back);
    }
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
    int v8;
    int vi;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;
    st.isp_loaded = true;
    st.isp_tuned = false; /* ISP not answering yet */

    /* A set before the ISP is up must succeed and be remembered, not
     * fail -- rvd applies the whole [image] block at this point. */
    CHECK(hal_isp_set_saturation(c, 100) == RSS_OK, "set_saturation should queue, not fail");
    CHECK(g_iq[IQ_SATURATION].has_pending, "saturation should be queued");
    CHECK(g_iq[IQ_SATURATION].pending == 100, "queued value should be 100");
    CHECK(!g_iq[IQ_SATURATION].pending_is_raw, "saturation queues as a scalar");

    /* And reading it back must agree with what was asked for. */
    v8 = 0;
    CHECK(hal_isp_get_saturation(c, &v8) == RSS_OK, "get_saturation should succeed while queued");
    CHECK(v8 == 100, "queued saturation should read back as 100, got %d", v8);

    /* An untouched auto/manual knob reads as auto rather than failing, and
     * rather than quoting a number nobody asked for. */
    v8 = 0;
    CHECK(hal_isp_get_brightness(c, &v8) == RSS_OK, "get_brightness should succeed");
    CHECK(v8 == RSS_ISP_AUTO, "untouched brightness should read auto, got %d", v8);

    /* Raw-valued params take the raw path. MI orders 60 Hz before 50 Hz,
     * so what gets queued is the translated value, not raptor's. */
    CHECK(hal_isp_set_antiflicker(c, RSS_ANTIFLICKER_50HZ) == RSS_OK, "set_antiflicker queues");
    CHECK(g_iq[IQ_FLICKER].has_pending && g_iq[IQ_FLICKER].pending_is_raw,
          "antiflicker queues as raw");
    CHECK(g_iq[IQ_FLICKER].pending == STAR_FLICKER_50HZ, "queued flicker value should be MI's 2");

    CHECK(hal_isp_set_defog(c, 1) == RSS_OK, "set_defog queues");
    CHECK(g_iq[IQ_DEFOG].has_pending && g_iq[IQ_DEFOG].pending == 1, "defog queued as 1");

    /* In EV steps now, so a value that used to be legal on the abstract
     * scale is not one here -- and that is the check worth having. */
    vi = 0;
    CHECK(hal_isp_set_ae_comp(c, 6) == RSS_OK, "set_ae_comp queues");
    CHECK(hal_isp_get_ae_comp(c, &vi) == RSS_OK, "get_ae_comp succeeds while queued");
    CHECK(vi == 6, "queued ae_comp reads back, got %d", vi);
    CHECK(hal_isp_set_ae_comp(c, 140) == RSS_ERR_INVAL, "140 is not an EV step this row holds");

    /* Flip goes to the sensor, not the ISP, so it is tracked regardless
     * of ISP readiness -- but with no MI_SNR loaded it must not crash. */
    CHECK(hal_isp_get_hvflip(c, &vi, NULL) == RSS_OK, "get_hvflip succeeds");
    CHECK(vi == 0, "flip defaults off");

    /* A NULL context must be rejected rather than dereferenced. */
    CHECK(hal_isp_set_saturation(NULL, 100) == RSS_ERR_INVAL, "NULL ctx rejected");

    /* Leave the table clean for any later test. */
    memset(&st, 0, sizeof(st));
    for (size_t i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

/*
 * Orientation is a bring-up setting, applied to the sensor, and the VPE
 * channel param must never carry it.
 *
 * That last clause is the regression this file exists to prevent. bMirror and
 * bFlip look like the obvious home for mirror and flip -- they are the right
 * stage, they read back, and a runtime change lands there deterministically,
 * which is why this backend used them for a fortnight. They also feed 3DNR's
 * motion detection, and with either set the filter stops recognising a moving
 * subject as moving and blends it into the background it crosses. No still
 * frame shows it. So the test is on the param the level writes: whatever the
 * sensor was told, those two fields go out zero.
 */
static unsigned int vpe_para_set_calls;
static unsigned int vpe_para_get_calls;
static i6e_vpe_para g_vpe_para;
static int vpe_para_get_ret, vpe_para_set_ret;

static int fake_get_chn_param(int chn, i6_vpe_para *param)
{
    (void)chn;
    vpe_para_get_calls++;
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

static void test_orientation_is_bringup_only(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    i6e_vpe_para para;
    void *c = &ctx;
    int hf, vf;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    memset(&g_vpe_para, 0, sizeof(g_vpe_para));
    ctx.platform = &st;
    st.vpe.fnGetChannelParam = fake_get_chn_param;
    st.vpe.fnSetChannelParam = fake_set_chn_param;
    st.vpe_chn_created = true;
    /* What star_sensor_bringup told MI_SNR_SetOrien: a camera mounted upside
     * down, which is the case that made this backend want a flip at all. */
    st.snr_mirror = 1;
    st.snr_flip = 1;
    vpe_para_set_calls = 0;
    vpe_para_get_calls = 0;
    vpe_para_get_ret = vpe_para_set_ret = 0;

    /* The reported orientation is what bring-up applied. MI_SNR_GetOrien
     * answers from a static table rather than the live register, so this must
     * come from raptor's record and not from the sensor. */
    hf = vf = -1;
    CHECK(hal_isp_get_hvflip(c, &hf, &vf) == RSS_OK, "get_hvflip succeeds");
    CHECK(hf == 1 && vf == 1, "get_hvflip reports what bring-up applied, got (%d,%d)", hf, vf);

    /* rvd drives the whole [image] block while building the pipeline, from the
     * same keys the sensor config already carried, so the ordinary path asks
     * for what is already true. That must succeed and must not write. */
    CHECK(hal_isp_set_hflip(c, 1) == RSS_OK, "setting the value already applied succeeds");
    CHECK(hal_isp_set_vflip(c, 1) == RSS_OK, "setting the value already applied succeeds");
    CHECK(vpe_para_set_calls == 0, "agreeing with bring-up must not write, got %u",
          vpe_para_set_calls);

    /* An actual change cannot be honoured: on a live sensor MI_SNR_SetOrien
     * only sets a dirty flag, and whether it lands depends on AE running.
     * Refusing beats reporting success for a write that may never happen. */
    CHECK(hal_isp_set_hflip(c, 0) == RSS_ERR_NOTSUP, "changing hflip at runtime is refused");
    CHECK(hal_isp_set_vflip(c, 0) == RSS_ERR_NOTSUP, "changing vflip at runtime is refused");
    CHECK(vpe_para_set_calls == 0, "a refused change must not write, got %u", vpe_para_set_calls);

    /* A refusal must not be recorded, or a later read would report an
     * orientation the sensor was never given. */
    hf = vf = -1;
    CHECK(hal_isp_get_hvflip(c, &hf, &vf) == RSS_OK, "get_hvflip still succeeds");
    CHECK(hf == 1 && vf == 1, "a refused change must not be recorded, got (%d,%d)", hf, vf);

    /*
     * The regression guard. With the sensor mirrored and flipped, the channel
     * param the builder produces must still carry neither -- anything else
     * re-creates the ghosting. Checked against the builder directly because
     * there is no runtime writer of this struct left to go through.
     */
    memset(&para, 0xA5, sizeof(para));
    star_vpe_fill_param(&st, &para);
    CHECK(para.mirror == 0 && para.flip == 0,
          "the channel param must never carry orientation, got (%d,%d)", para.mirror, para.flip);
}

/*
 * The VPE channel's 3DNR level is fixed, and temper is not a knob here.
 *
 * The level is not a strength: mhal maps level 1 to an 8-bit 3DNR reference
 * frame and level 2 to a 12-bit one, and maps every other value -- 0 and 3..7
 * alike -- to "engine disabled". Six of the eight positions mean off, so
 * there is no scale to publish, and this backend fixes the level at 2 (what
 * majestic and the SDK's own demos run) and offers no op to move it.
 *
 * Level 0 is the one that has to stay unreachable. The DNR engine it disables
 * is also what carries mirror and flip on this SoC, so a channel holding a
 * flip and asking for level 0 stalls the ISP outright -- reproduced on .229:
 * 0.4 fps, "SclReal mode NO frame done", CMDQ WAIT_TRIG_TIMEOUT. Orientation
 * lives on the sensor now and the level never moves, so neither half of that
 * pair can occur; this pins both.
 */
static void test_vpe_level_is_fixed_and_temper_is_unpublished(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_isp_knob_t caps;
    i6e_vpe_para para;
    void *c = &ctx;

    memset(&ctx, 0, sizeof(ctx));
    memset(&st, 0, sizeof(st));
    ctx.platform = &st;

    /* A mirrored and flipped sensor -- the state that made level 0 fatal. */
    st.snr_mirror = 1;
    st.snr_flip = 1;

    memset(&para, 0xA5, sizeof(para));
    star_vpe_fill_param(&st, &para);
    CHECK(para.level3DNR == STAR_VPE_NR3D_LEVEL, "the level is fixed at %d, got %d",
          STAR_VPE_NR3D_LEVEL, para.level3DNR);
    CHECK(para.level3DNR != 0, "level 0 disables the engine mirror and flip run on");
    CHECK(para.mirror == 0 && para.flip == 0,
          "orientation stays off the channel however the sensor is set, got (%d,%d)", para.mirror,
          para.flip);

    /* No op and no caps. rvd asks by name, so the name has to answer NOTSUP
     * rather than a range nothing can honour. */
    CHECK(hal_isp_get_knob_caps(c, "temper", &caps) == RSS_ERR_NOTSUP,
          "temper publishes no caps on this SoC");
}

/*
 * The channel param is built, not edited.
 *
 * MI_VPE_ChannelPara_t opens with MI_VPE_PqParam_t -- chroma and luma
 * spatial/temporal NR strengths, six edge gains, a contrast value -- and
 * carries MI_VPE_LdcParam_t after it. Neither belongs to this driver, and the
 * old read-modify-write handed both back to MI having only ever zeroed them
 * with the caller's memset. That worked while MI_VPE_GetChannelParam declined
 * to fill them; it was not a decision.
 *
 * Two things are pinned here. The write path does not call Get at all, so a
 * Get that fails, or one that starts returning those fields populated, cannot
 * change what is written. And the bytes this driver does not own leave as
 * zeros deliberately, which is what every vendor reference sends.
 */
static void test_channel_param_is_built_not_edited(void)
{
    star_state_t st;
    i6e_vpe_para para;
    size_t i;
    int all_zero;

    memset(&st, 0, sizeof(st));
    /* A mirrored sensor, so the builder has something it could wrongly echo. */
    st.snr_mirror = 1;
    st.snr_flip = 1;

    /*
     * The buffer arrives holding everything the builder must not preserve:
     * a populated PqParam, an LDC config, LDC switched on. A builder that
     * edited in place -- or a caller that read the channel back first --
     * would carry every one of them into the next Set.
     */
    memset(&para, 0xA5, sizeof(para));
    para.lensAdjOn = 1;

    star_vpe_fill_param(&st, &para);

    CHECK(para.mirror == 0 && para.flip == 0,
          "orientation must not reach the channel however the sensor is set, got (%d,%d)",
          para.mirror, para.flip);
    CHECK(para.level3DNR == STAR_VPE_NR3D_LEVEL, "the level is the fixed one, got %d",
          para.level3DNR);
    CHECK(para.hdr == I6_HDR_OFF, "hdr is stated, got %d", (int)para.hdr);
    CHECK(para.lensAdjOn == 0, "LDC must not be echoed back on, got %d", para.lensAdjOn);

    /* MI_VPE_PqParam_t: ours to zero, never to carry. */
    all_zero = 1;
    for (i = 0; i < sizeof(para.reserved); i++)
        if (para.reserved[i])
            all_zero = 0;
    CHECK(all_zero, "PqParam must go out zeroed, not echoed");

    /* MI_VPE_LdcParam_t, the 72 bytes after it. */
    CHECK(para.lensAdj.bypassOn == 0 && para.lensAdj.configAddr == NULL &&
              para.lensAdj.configSize == 0,
          "LdcParam must go out zeroed, not echoed");
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
    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;

    CHECK(hal_isp_set_saturation(c, 100) == RSS_OK, "saturation is recorded");

    /* No MI handle here, so the applies inside fail; what this test is
     * about is the state of the record afterwards. */
    star_isp_flush_pending(&st);

    CHECK(g_iq[IQ_SATURATION].has_pending, "the flush must not consume the record");
    CHECK(g_iq[IQ_SATURATION].pending == 100, "nor alter it, got %d", g_iq[IQ_SATURATION].pending);

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

static void tune_setup(star_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->isp_loaded = true;
    st->isp_tuned = false;
    st->fps = 30;
    snprintf(st->iq_file, sizeof(st->iq_file), "/etc/sensors/gc4653.bin");
    st->isp.fnGetParaInitStatus = fake_parainit_ready;
    st->isp.fnLoadChannelConfig = fake_loadcfg;

    loadcfg_calls = 0;
}

static void tune_once(star_state_t *st)
{
    tune_setup(st);
    /* The load is gated on a delivered frame; this test is about what the
     * load itself does, so grant it. */
    st->isp_frame_seen = true;
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
 * The load waits for a delivered frame, and nothing sooner.
 *
 * CUS3A's AE init reads its own iqfile from disk and takes the AE's limits
 * from it, and CUS3A defers that init to its frame thread. So a load issued
 * on IQ-API readiness -- which is what this used to do -- is read back over
 * every time, and the gain ceiling and shutter cap go with it, costing a
 * third of the frame rate in a dark scene. A frame reaching the application
 * is strictly after the ISP frame interrupt that runs the init.
 *
 * The failure this pins is the load happening at all before that: an early
 * load is worse than no load, because it sets the latch that stops anything
 * later from trying.
 */
static void test_tuning_waits_for_the_first_frame(void)
{
    star_state_t st;
    size_t i;

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;

    /* Every bring-up call before the first frame must decline, quietly and
     * without latching, however many times it is made. */
    tune_setup(&st);
    star_isp_tune_when_ready(&st, false);
    star_isp_tune_when_ready(&st, true);
    CHECK(loadcfg_calls == 0, "nothing may load before a frame, got %u calls", loadcfg_calls);
    CHECK(!st.isp_tuned, "and nothing may latch");

    /* The frame is the trigger, and it loads exactly once. */
    star_isp_note_frame(&st);
    CHECK(st.isp_frame_seen, "the frame is recorded");
    CHECK(loadcfg_calls == 1, "the first frame loads, got %u calls", loadcfg_calls);
    CHECK(st.isp_tuned, "and latches");

    star_isp_note_frame(&st);
    star_isp_note_frame(&st);
    CHECK(loadcfg_calls == 1, "later frames must not reload, got %u calls", loadcfg_calls);

    /*
     * Losing the VPE channel has to take the frame back as well as the
     * latch. The channel restarting re-registers CUS3A, which re-reads the
     * iqfile on its next frame interrupt -- so a reload issued on the
     * strength of a frame from before the restart would be lost exactly as
     * the first load was.
     */
    star_isp_untune(&st);
    CHECK(!st.isp_frame_seen, "untune must take the frame back, not just the latch");
    star_isp_tune_when_ready(&st, true);
    CHECK(loadcfg_calls == 1, "and the restart must not reload before a new frame, got %u",
          loadcfg_calls);
    star_isp_note_frame(&st);
    CHECK(loadcfg_calls == 2, "the first frame after the restart reloads, got %u", loadcfg_calls);

    for (i = 0; i < IQ_PARAM_COUNT; i++)
        g_iq[i].has_pending = false;
}

/*
 * ── isp_get_exposure ──────────────────────────────────────────────────
 *
 * Scene luma comes from the AE status' own preAvgY, with the grid behind
 * it as the fallback -- so what has to be tested is that the cheap path is
 * the one taken, and that the expensive one is still reachable when the
 * field is unfilled or reads outside the 0..255 mean ric is calibrated
 * for.
 *
 * The grid's layout is the vendor's MI_ISP_AE_HW_STATISTICS_t, so what is
 * left to test there is what still happens at runtime: the buffer is sized
 * for a 128x90 grid and the sensor reports 32x32, so the live dimensions
 * decide how much of it is averaged, and a buffer whose dimensions
 * disagree with the AE status must produce no luma rather than a number
 * from the wrong cells. Plus the fixed-point gain arithmetic.
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

    /* preAvgY is left zero, so every grid test below runs against the
     * fallback path deliberately rather than by accident. */
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

static void test_exposure_luma_comes_from_the_ae_status(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;
    uint32_t want;

    g_ae_stats = malloc(sizeof(*g_ae_stats));
    assert(g_ae_stats);
    want = fill_grid(g_ae_stats->cell, 8);
    g_ae_stats->blkX = 4;
    g_ae_stats->blkY = 2;

    /* The AE's own frame brightness is the reading, and the 46KB grid call
     * is not made at all -- on hardware the two agree exactly, so paying
     * for the second one buys nothing. */
    exposure_setup(&ctx, &st);
    g_ae_status.preAvgY = 42;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a healthy read must succeed");
    CHECK(exp.ae_luma == 42, "luma must come from preAvgY, got %u", exp.ae_luma);
    CHECK(g_ae_stats_calls == 0, "preAvgY must skip the grid call, got %u", g_ae_stats_calls);

    /* The top of the scale is a reading, not an error. */
    exposure_setup(&ctx, &st);
    g_ae_status.preAvgY = 255;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "a full-scale read must succeed");
    CHECK(exp.ae_luma == 255, "255 is a mean, got %u", exp.ae_luma);
    CHECK(g_ae_stats_calls == 0, "255 must skip the grid call, got %u", g_ae_stats_calls);

    /* Unfilled, as a CUS3A older than V1.1 would leave it: the grid still
     * has to be tried, because reporting the "not available" zero here
     * would drop ric to gain-only on a library that can answer. */
    exposure_setup(&ctx, &st);
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "an unfilled field must still read");
    CHECK(exp.ae_luma == want, "an unfilled field must fall back to the grid, got %u", exp.ae_luma);
    CHECK(g_ae_stats_calls == 1, "the fallback must make the grid call, got %u", g_ae_stats_calls);

    /* Out of range is not the 0..255 mean night_luma is calibrated
     * against, whatever else it may be, so it is refused rather than
     * rescaled by a guess -- a guess here moves the IR-cut filter. */
    exposure_setup(&ctx, &st);
    g_ae_status.preAvgY = 1023;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "an out-of-range field must still read");
    CHECK(exp.ae_luma == want, "an out-of-range field must fall back to the grid, got %u",
          exp.ae_luma);
    CHECK(g_ae_stats_calls == 1, "the fallback must make the grid call, got %u", g_ae_stats_calls);

    /* Neither source: zero, which the contract reads as silence. */
    exposure_setup(&ctx, &st);
    st.isp.fnGetAeHwAvgStats = NULL;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "no luma source must not fail the read");
    CHECK(exp.ae_luma == 0, "no luma source must report zero, got %u", exp.ae_luma);
    CHECK(exp.total_gain == 2048, "gain must survive with no luma, got %u", exp.total_gain);

    free(g_ae_stats);
    g_ae_stats = NULL;
}

static void test_exposure_luma_covers_only_the_live_grid(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;
    uint32_t want;

    g_ae_stats = malloc(sizeof(*g_ae_stats));
    assert(g_ae_stats);

    /*
     * fill_grid puts a known ramp in the first 8 cells and a value the
     * mean must not pick up in every cell after them -- so this fails if
     * the average runs past the 4x2 grid the AE status reports into the
     * rest of the 128x90 buffer.
     */
    exposure_setup(&ctx, &st);
    want = fill_grid(g_ae_stats->cell, 8);
    g_ae_stats->blkX = 4;
    g_ae_stats->blkY = 2;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "the grid must read");
    CHECK(exp.ae_luma == want, "luma is %u, got %u", want, exp.ae_luma);

    free(g_ae_stats);
    g_ae_stats = NULL;
}

static void test_exposure_refuses_a_grid_that_disagrees(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_exposure_t exp;

    g_ae_stats = malloc(sizeof(*g_ae_stats));
    assert(g_ae_stats);

    /* The stats buffer's own dimensions contradict the AE status'. Reading
     * the cells anyway is what this test exists to prevent: the number
     * would look like a reading and move the IR-cut filter. */
    exposure_setup(&ctx, &st);
    fill_grid(g_ae_stats->cell, 8);
    g_ae_stats->blkX = 999;
    g_ae_stats->blkY = 999;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "the gain half must still be reported");
    CHECK(exp.ae_luma == 0, "a disagreeing grid must yield no luma, got %u", exp.ae_luma);
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

    CHECK(st.bin_max_sensor_gain == 8192, "sensor ceiling should be 8192, got %u",
          st.bin_max_sensor_gain);
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

/*
 * The AE's shutter ceiling belongs to the tuning, not to this backend.
 *
 * A cap used to live here holding maxShutterUs to one frame period; the
 * block comment where it was records why it went. What has to stay true is
 * that nothing on the bring-up path writes the limits back, because the
 * failure that would follow is invisible: no error anywhere, just a picture
 * darker than the tuning intended at a frame rate nobody asked to keep.
 */
static void test_the_ae_shutter_ceiling_is_left_alone(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    limit_setup(&ctx, &st);
    st.snr.fnSetFramerate = fake_set_fps;
    g_fps_set_ret = 0;
    g_fps_programmed = 0;

    star_isp_snapshot_bin_limits(&st);
    star_isp_kick_sensor_rate(&st, 30);

    CHECK(g_limit_set_calls == 0, "the exposure limits must not be written, got %u writes",
          g_limit_set_calls);
    CHECK(g_limit.maxShutterUs == 40000,
          "the tuning's 40000 us ceiling must survive a 30 fps bring-up, got %u",
          g_limit.maxShutterUs);
    CHECK(g_limit.minShutterUs == 30, "the floor must survive it too, got %u",
          g_limit.minShutterUs);

    /* The sensor rate is still re-issued. That is the cold-boot fix, which
     * shared a function with the cap rather than a purpose. */
    CHECK(g_fps_programmed == 30, "the sensor rate must still be re-issued, got %u",
          g_fps_programmed);

    /* And a fractional rate still goes out in milli-frames rather than
     * rounded, which is the one thing star_snr_fps_arg exists for. */
    st.fps_milli = 12500;
    star_isp_kick_sensor_rate(&st, 13);
    CHECK(g_fps_programmed == 12500, "a fractional rate must go out as milli-frames, got %u",
          g_fps_programmed);
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
    st->bin_max_sensor_gain = 131072;
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
     * Any ceiling that is not the tuning's own now reads as a reset, because
     * with max_again withdrawn nothing else writes these limits. That is
     * what retires the old blind spot: a configured 8192 used to be
     * indistinguishable from the untuned default.
     */
    reload_setup(&ctx, &st);
    g_limit.maxSensorGain = 32768;
    star_isp_reload_if_reset(&st, true);
    CHECK(loadcfg_calls == 1, "a ceiling that is not the tuning's reads as a reset, got %u",
          loadcfg_calls);

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
    test_a_written_value_reads_back_unchanged();
    test_auto_is_out_of_band();
    test_caps_describe_what_the_setter_accepts();
    test_evcomp_neutral_comes_from_the_tuning();
    test_ae_comp_is_signed_end_to_end();
    test_antiflicker_translates_the_mains_frequency();
    test_pending_queue();
    test_orientation_is_bringup_only();
    test_vpe_level_is_fixed_and_temper_is_unpublished();
    test_channel_param_is_built_not_edited();
    test_sensor_fps_clamps_to_the_mode();
    test_recorded_values_survive_a_reload();
    test_tuning_load_leaves_3a_alone();
    test_tuning_waits_for_the_first_frame();
    test_exposure_waits_for_the_isp();
    test_exposure_gain_is_x1024_fixed_point();
    test_exposure_luma_comes_from_the_ae_status();
    test_exposure_luma_covers_only_the_live_grid();
    test_exposure_refuses_a_grid_that_disagrees();
    test_bin_limits_snapshot_records_the_tuning();
    test_the_ae_shutter_ceiling_is_left_alone();
    test_the_tuning_is_reloaded_when_the_isp_resets();
    test_iq_file_search_order();

    if (failures) {
        printf("\n%d check(s) failed\n", failures);
        return 1;
    }

    printf("all hal_isp logic tests passed\n");
    return 0;
}
