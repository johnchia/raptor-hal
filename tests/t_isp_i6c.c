/*
 * Host-side test of the pure logic in infinity6c/hal_isp.c.
 *
 * Same construction as t_isp.c and for the same reason: the real translation
 * unit is included rather than copied, so what is tested is what ships, and MI
 * is stubbed by filling in the function pointers in infinity6c_state_t. Nothing
 * in src/infinity6c links libmi_*.so directly.
 *
 * Separate from t_isp.c because it is a different backend against a different
 * generation's headers -- MI 3.0's calls take a leading device argument, and the
 * two files would need incompatible -I paths in one binary.
 */

#define PLATFORM_INFINITY6C 1
#define HAL_MODULE_VIDEO 1

#include "infinity6c/hal_isp.c"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

/*
 * A real (silent) logger rather than NULL: HAL_LOG_* call through this without a
 * NULL guard, because rss_hal_init always installs one, and every test here
 * drives at least one warning path.
 *
 * It also keeps what was logged. Most of this file asserts on the bytes a module
 * was handed, which is the right level for a knob -- but a diagnostic's whole
 * observable behaviour is the line it prints, so the only way to test one is to
 * read it back.
 */
static char g_log[8192];
static size_t g_log_len;

static void quiet_log(int level, const char *file, int line, const char *fmt, ...)
{
    size_t room = sizeof(g_log) - g_log_len;
    va_list ap;
    int n;

    (void)level;
    (void)file;
    (void)line;

    /* Two bytes: one for the newline, one for the terminator. */
    if (room < 3)
        return;

    va_start(ap, fmt);
    n = vsnprintf(g_log + g_log_len, room - 2, fmt, ap);
    va_end(ap);
    if (n < 0)
        return;

    g_log_len += (size_t)n < room - 2 ? (size_t)n : room - 3;
    g_log[g_log_len++] = '\n';
    g_log[g_log_len] = '\0';
}

/*
 * Terminating as well as rewinding. vsnprintf's own NUL is overwritten by the
 * newline above, so a buffer that is only rewound still reads as whatever the
 * previous test logged -- and an assertion looking for a line it did not expect
 * would find one and fail on a ghost.
 */
static void log_reset(void)
{
    g_log_len = 0;
    g_log[0] = '\0';
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

/* What MI_SNR_GetFps answers, and whether it was asked at all. */
static unsigned int g_fps_reported;
static int g_fps_gets;
static int g_fps_get_ret;

static int fake_get_fps(unsigned int device, unsigned int *fps)
{
    (void)device;
    g_fps_gets++;
    if (g_fps_get_ret)
        return g_fps_get_ret;
    *fps = g_fps_reported;
    return 0;
}

/*
 * The tuning API, stubbed as what it is: a byte store per module that Get hands
 * out whole and Set takes back whole. That shape is the point -- the vector code
 * is read-modify-write over a payload it cannot describe, so a stub that only
 * modelled the fields under test would not catch the bug worth catching, which
 * is a write landing outside them.
 */
#define SHARP_RUN (I6C_ISP_IQ_SHARPNESS_MANUAL + I6C_ISP_IQ_SHARPNESS_STRENGTH)

static uint8_t g_sharp[I6C_ISP_IQ_SHARPNESS_PAYLOAD];
static int g_sharp_sets;
static int g_sharp_get_ret, g_sharp_set_ret;

static int fake_get_sharp(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    if (g_sharp_get_ret)
        return g_sharp_get_ret;
    memcpy(payload, g_sharp, sizeof(g_sharp));
    return 0;
}

static int fake_set_sharp(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    g_sharp_sets++;
    if (g_sharp_set_ret)
        return g_sharp_set_ret;
    memcpy(g_sharp, payload, sizeof(g_sharp));
    return 0;
}

/*
 * WDR, the module raptor's DRC knob drives. Same byte-store shape as sharpness
 * above, and for the same reason: the claim under test is where a write lands
 * inside a payload the code cannot describe.
 */
#define DRC_OFF (I6C_ISP_IQ_WDR_MANUAL + I6C_ISP_IQ_WDR_STRENGTH)

static uint8_t g_wdr[I6C_ISP_IQ_WDR_PAYLOAD];
static int g_wdr_sets;
static int g_wdr_get_ret, g_wdr_set_ret;

static int fake_get_wdr(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    if (g_wdr_get_ret)
        return g_wdr_get_ret;
    memcpy(payload, g_wdr, sizeof(g_wdr));
    return 0;
}

static int fake_set_wdr(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    g_wdr_sets++;
    if (g_wdr_set_ret)
        return g_wdr_set_ret;
    memcpy(g_wdr, payload, sizeof(g_wdr));
    return 0;
}

/*
 * Defog, the one module raptor drives through two rows: a strength and a
 * switch, both landing in the same payload through the same pair of symbols.
 */
static uint8_t g_defog[I6C_ISP_IQ_DEFOG_PAYLOAD];

static int fake_get_defog(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    memcpy(payload, g_defog, sizeof(g_defog));
    return 0;
}

static int fake_set_defog(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    memcpy(g_defog, payload, sizeof(g_defog));
    return 0;
}

/*
 * NR3D, the one module raptor writes into stAuto rather than stManual. Its
 * payload is the biggest of the three at 1912 bytes, sixteen 112-byte entries
 * behind the two-word header.
 */
static uint8_t g_nr3d[I6C_ISP_IQ_NR3D_PAYLOAD];
static int g_nr3d_sets;

static int fake_get_nr3d(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    memcpy(payload, g_nr3d, sizeof(g_nr3d));
    return 0;
}

static int fake_set_nr3d(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    g_nr3d_sets++;
    memcpy(g_nr3d, payload, sizeof(g_nr3d));
    return 0;
}

/* u16MdGain, the field four bytes into a parameter block. Stands in here for
 * everything the knob must not touch: it is the one that actually ramps with
 * gain in a shipped tuning. */
#define NR3D_MDGAIN 4

/* g_iq is file-static and outlives a test, so each one puts its rows back. */
static void reset_row(int idx, i6c_isp_cmd_fn get, i6c_isp_cmd_fn set)
{
    i6c_iq_param_t *p = &g_iq[idx];

    p->fn_get = get;
    p->fn_set = set;
    p->pending = 0;
    p->has_pending = false;
    p->pending_is_raw = false;
    p->base_valid = false;
    p->report = 0;
    /* As i6c_isp_flush_knobs leaves it: the tuning's own state is owed, and the
     * first fetch is what pays it. */
    p->tuning_stale = true;
    p->tuned_on = false;
    p->tuned_on_valid = false;
    p->overridden = false;
    memset(p->base, 0, sizeof(p->base));
}

/* Write what a tuning binary would have left in a module: a run of values, and
 * the auto/manual mode it chose. */
static void seed_run(uint8_t *store, size_t len, unsigned int off, uint8_t width,
                     const uint16_t *vals, unsigned int count, uint32_t optype)
{
    unsigned int i;

    memset(store, 0, len);
    i6c_iq_write(store, I6C_ISP_OPTYPE_OFF, 4, optype);
    for (i = 0; i < count; i++)
        i6c_iq_write(store, off + i * width, width, vals[i]);
}

static uint32_t run_at(const uint8_t *store, unsigned int off, uint8_t width, unsigned int i)
{
    return i6c_iq_read(store, off + i * width, width);
}

/*
 * The same for a gain run, where the stride is the parameter block rather than
 * the field width. The neighbour argument seeds one field of every entry with a
 * value that varies by entry, which is what a real curve looks like and what
 * makes a stray write visible.
 */
static void seed_gain_run(uint8_t *store, unsigned int off, unsigned int stride, uint8_t width,
                          const uint16_t *vals, unsigned int count, unsigned int neighbour_off,
                          uint32_t optype)
{
    unsigned int i;

    memset(store, 0, I6C_ISP_IQ_NR3D_PAYLOAD);
    i6c_iq_write(store, I6C_ISP_ENABLE_OFF, 4, 1);
    i6c_iq_write(store, I6C_ISP_OPTYPE_OFF, 4, optype);
    for (i = 0; i < count; i++) {
        i6c_iq_write(store, off + i * stride, width, vals[i]);
        i6c_iq_write(store, I6C_ISP_IQ_NR3D_AUTO + i * stride + neighbour_off, 2,
                     (uint32_t)(100 + i * 50));
    }
}

static uint32_t gain_at(const uint8_t *store, unsigned int off, uint8_t width, unsigned int i)
{
    return i6c_iq_read(store, off + i * I6C_ISP_IQ_NR3D_ENTRY, width);
}

static uint32_t neighbour_at(const uint8_t *store, unsigned int i)
{
    return i6c_iq_read(store, I6C_ISP_IQ_NR3D_AUTO + i * I6C_ISP_IQ_NR3D_ENTRY + NR3D_MDGAIN, 2);
}

/*
 * The colour transform, which is the one module here that three knobs share.
 *
 * Stubbed as a byte store like the rest, and for a sharper version of the same
 * reason: the composer writes nine coefficients and one offset out of a payload
 * of twenty-eight bytes, so what has to be caught is a write landing outside
 * the fields it names as much as a wrong value inside them.
 */
static uint8_t g_ctrans[I6C_ISP_IQ_COLORTRANS_PAYLOAD];
static uint8_t g_ctex[I6C_ISP_IQ_COLORTRANSEX_PAYLOAD];
static int g_ctrans_sets;

static int fake_get_ctrans(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    memcpy(payload, g_ctrans, sizeof(g_ctrans));
    return 0;
}

static int fake_set_ctrans(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    g_ctrans_sets++;
    memcpy(g_ctrans, payload, sizeof(g_ctrans));
    return 0;
}

static int fake_get_ctex(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    memcpy(payload, g_ctex, sizeof(g_ctex));
    return 0;
}

static int fake_set_ctex(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    memcpy(g_ctex, payload, sizeof(g_ctex));
    return 0;
}

/*
 * The conversion an IMX335 tuning ships: BT.601 at full swing over 256, with
 * the negative coefficients in the field's 10-bit two's complement and all
 * three offsets at zero.
 */
static const int16_t ct_full[I6C_ISP_IQ_COLORTRANS_MAT_NUM] = {77,  150, 29,   -43, -85,
                                                               128, 128, -107, -21};

/*
 * The same conversion at limited swing, which is what the ISP puts out with the
 * module disabled and therefore what the composer works in once a full-range
 * tuning has been narrowed -- see i6c_ct_capture. The luma row comes to 219 of
 * 256 and the chroma rows to 224, with a 16-count pedestal that is 64 in the
 * offset field's quarter-count domain.
 *
 * The seed for every test that is not about the narrowing itself, so that
 * "unchanged" means unchanged rather than "unchanged after a conversion the
 * test would have to repeat".
 */
static const int16_t ct_lim[I6C_ISP_IQ_COLORTRANS_MAT_NUM] = {66,  129, 25,  -38, -75,
                                                              112, 112, -94, -18};
#define CT_LIM_OFST 64

static void seed_ctrans(const int16_t *m, int32_t yofst)
{
    int i;

    memset(g_ctrans, 0, sizeof(g_ctrans));
    for (i = 0; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        i6c_iq_write(g_ctrans, I6C_ISP_IQ_COLORTRANS_MATRIX + 2 * i, 2,
                     i6c_ct_unsigned(m[i], I6C_ISP_IQ_COLORTRANS_MAT_WRAP));
    i6c_iq_write(g_ctrans, I6C_ISP_IQ_COLORTRANS_YOFST, 2,
                 i6c_ct_unsigned(yofst, I6C_ISP_IQ_COLORTRANS_OFST_WRAP));
}

static int32_t ct_coef(int i)
{
    return i6c_ct_signed(i6c_iq_read(g_ctrans, I6C_ISP_IQ_COLORTRANS_MATRIX + 2 * i, 2),
                         I6C_ISP_IQ_COLORTRANS_MAT_WRAP);
}

static int32_t ct_ofst(int i)
{
    return i6c_ct_signed(i6c_iq_read(g_ctrans, I6C_ISP_IQ_COLORTRANS_YOFST + 2 * i, 2),
                         I6C_ISP_IQ_COLORTRANS_OFST_WRAP);
}

/* What the composed transform makes of a neutral grey: R = G = B = 128 puts
 * 128 * sum/256 through the row, and the offset adds a quarter of its own
 * count. */
static int32_t ct_mid_grey(void)
{
    return (128 * (ct_coef(0) + ct_coef(1) + ct_coef(2))) / 256 + ct_ofst(0) / 4;
}

/* The state the composer keeps is file-static and outlives a test, as g_iq is. */
static void reset_ct_rows(void)
{
    int i;

    reset_row(IQ_COLORTRANS, fake_get_ctrans, fake_set_ctrans);
    reset_row(IQ_COLORTRANS_EX, fake_get_ctex, fake_set_ctex);
    seed_ctrans(ct_lim, CT_LIM_OFST);
    memset(g_ctex, 0, sizeof(g_ctex));
    g_ctrans_sets = 0;

    memset(&g_ct, 0, sizeof(g_ct));
    for (i = 0; i < CT_KNOB_COUNT; i++)
        g_ct.val[i] = RSS_ISP_AUTO;
}

static void reset(infinity6c_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->snr.get_fps = fake_get_fps;

    reset_row(IQ_SHARPNESS, fake_get_sharp, fake_set_sharp);
    memset(g_sharp, 0, sizeof(g_sharp));
    g_sharp_sets = 0;
    g_sharp_get_ret = g_sharp_set_ret = 0;

    reset_row(IQ_DRC, fake_get_wdr, fake_set_wdr);
    memset(g_wdr, 0, sizeof(g_wdr));
    g_wdr_sets = 0;
    g_wdr_get_ret = g_wdr_set_ret = 0;

    reset_row(IQ_NR3D, fake_get_nr3d, fake_set_nr3d);
    memset(g_nr3d, 0, sizeof(g_nr3d));
    g_nr3d_sets = 0;

    reset_row(IQ_DEFOG, fake_get_defog, fake_set_defog);
    reset_row(IQ_DEFOG_EN, fake_get_defog, fake_set_defog);
    memset(g_defog, 0, sizeof(g_defog));

    /*
     * The colour transform, wired for every test rather than only the ones
     * about it: the flush composes it unconditionally, so a test of anything
     * else that reaches the flush would otherwise resolve a NULL getter.
     */
    reset_ct_rows();

    /* The tuning binary's own envelope on this board: a 100 ms ceiling, which
     * at 8.08 us a line is VMAX 12376 and about 8 fps. */
    g_fps_reported = 0;
    g_fps_gets = 0;
    g_fps_get_ret = 0;
}

/*
 * The reporting op may read the sensor -- that is the only way to learn a rate
 * the driver clamped -- but it must not cache the answer. st->fps is what the
 * pipeline binds its rates to, so a reading taken while the AE had the frame
 * stretched would be carried into the next bind_ext, and into the next cap.
 */
static void test_reporting_does_not_cache_the_sensors_answer(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    uint32_t num = 0, den = 0;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.fps = 25;
    st.pipeline_up = true;
    g_fps_reported = 9;

    CHECK(hal_isp_get_sensor_fps(c, &num, &den) == RSS_OK, "get must succeed");
    CHECK(num == 9 && den == 1, "the sensor's answer is what is reported, got %u/%u", num, den);
    CHECK(st.fps == 25, "st.fps must not follow the reading, got %u", st.fps);
}

/* With no sensor to ask, reporting falls back to the configured rate rather
 * than failing, and prefers the programmed one over the request. */
static void test_reporting_falls_back_to_the_configured_rate(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    uint32_t num = 0, den = 0;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;

    /* Pipeline down: the sensor cannot be asked, whatever it would say. */
    st.snr_fps_req = 25;
    g_fps_reported = 9;
    CHECK(hal_isp_get_sensor_fps(c, &num, &den) == RSS_OK, "get must fall back");
    CHECK(num == 25 && den == 1, "the request is the fallback, got %u/%u", num, den);
    CHECK(g_fps_gets == 0, "a down pipeline must not be asked, asked %d times", g_fps_gets);

    st.fps = 20;
    CHECK(hal_isp_get_sensor_fps(c, &num, &den) == RSS_OK, "get must fall back");
    CHECK(num == 20 && den == 1, "the programmed rate outranks the request, got %u/%u", num, den);

    /* Nothing configured and nothing to ask is BUSY, not a fabricated rate. */
    st.fps = 0;
    st.snr_fps_req = 0;
    CHECK(hal_isp_get_sensor_fps(c, &num, &den) == RSS_ERR_BUSY, "an unknown rate is BUSY");
}

/* ================================================================
 * VECTOR ROWS: sharpness and spatial denoise
 * ================================================================ */

/* The tuning's own six sharpening gains: three frequency bands for the
 * undirectional sharpener, then three for the directional one. Deliberately not
 * flat, because the whole claim of the vector shape is that the tuner's *shape*
 * survives a knob. */
static const uint16_t g_tuned_sharp[6] = {40, 30, 20, 60, 50, 45};

static void arm(rss_hal_ctx_t *ctx, infinity6c_state_t *st)
{
    reset(st);
    memset(ctx, 0, sizeof(*ctx));
    ctx->platform = st;
    /* Past the first frame: the tuning has loaded and the ISP keeps writes. */
    st->isp_knobs_live = true;
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, g_tuned_sharp, 6, I6C_ISP_OP_AUTO);
}

/*
 * The heart of it. One 0..255 drives six fields at once, each scaled about the
 * value the tuning left in it -- so the ends are the module's own ends and the
 * middle keeps the tuner's balance between the bands.
 */
static void test_a_vector_knob_scales_every_field_about_its_own_baseline(void)
{
    /*
     * The run reports from field 3, whose baseline of 60 is the furthest from
     * both bounds, so 60 is the pivot: ask for 100 and field 3 becomes exactly
     * 100, while the others move about their own baselines by the same
     * fraction of their own headroom -- 40 + (100-60)*(127-40)/(127-60), and
     * so on. Asking for 30 halves the pivot, so every field halves.
     */
    static const uint16_t up[6] = {91, 87, 83, 100, 95, 93};
    static const uint16_t down[6] = {20, 15, 10, 30, 25, 22};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    unsigned int i;

    arm(&ctx, &st);
    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "set must succeed");
    for (i = 0; i < 6; i++)
        CHECK(run_at(g_sharp, SHARP_RUN, 1, i) == up[i], "field %u: want %u, got %u", i, up[i],
              run_at(g_sharp, SHARP_RUN, 1, i));

    arm(&ctx, &st);
    CHECK(hal_isp_set_sharpness(&ctx, 30) == RSS_OK, "set must succeed");
    for (i = 0; i < 6; i++)
        CHECK(run_at(g_sharp, SHARP_RUN, 1, i) == down[i], "field %u: want %u, got %u", i, down[i],
              run_at(g_sharp, SHARP_RUN, 1, i));

    /* The ends are MI's, not the tuning's: 255 is every field at the ceiling
     * and 0 is the module contributing nothing. */
    arm(&ctx, &st);
    CHECK(hal_isp_set_sharpness(&ctx, I6C_ISP_IQ_SHARPNESS_STRENGTH_MAX) == RSS_OK,
          "set must succeed");
    for (i = 0; i < 6; i++)
        CHECK(run_at(g_sharp, SHARP_RUN, 1, i) == I6C_ISP_IQ_SHARPNESS_STRENGTH_MAX,
              "field %u must reach the ceiling, got %u", i, run_at(g_sharp, SHARP_RUN, 1, i));

    arm(&ctx, &st);
    CHECK(hal_isp_set_sharpness(&ctx, 0) == RSS_OK, "set must succeed");
    for (i = 0; i < 6; i++)
        CHECK(run_at(g_sharp, SHARP_RUN, 1, i) == 0, "field %u must reach the floor, got %u", i,
              run_at(g_sharp, SHARP_RUN, 1, i));
}

/*
 * Neutral hands the module back to the tuning rather than pinning it to the
 * tuning's current numbers. The distinction is the per-ISO auto table: pinned to
 * manual, the run stops following the gain, and a night scene keeps a daylight
 * sharpening.
 */
static void test_neutral_returns_a_vector_row_to_auto_untouched(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    unsigned int i;

    arm(&ctx, &st);
    /* Manual, as a previous non-neutral set would have left it. */
    i6c_iq_write(g_sharp, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_MANUAL);

    CHECK(hal_isp_set_sharpness(&ctx, RSS_ISP_AUTO) == RSS_OK, "set must succeed");
    CHECK(i6c_iq_read(g_sharp, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO,
          "neutral must select auto");
    for (i = 0; i < 6; i++)
        CHECK(run_at(g_sharp, SHARP_RUN, 1, i) == g_tuned_sharp[i],
              "neutral must not rewrite field %u: want %u, got %u", i, g_tuned_sharp[i],
              run_at(g_sharp, SHARP_RUN, 1, i));
}

/*
 * The bug the baseline exists to prevent. Scaling from whatever is in the field
 * now compounds, so the same knob applied twice would walk the run upward with
 * no way back short of a tuning reload -- and rvd re-applies its whole [image]
 * block on demand, so "twice" is a thing that happens by itself.
 */
static void test_a_vector_knob_does_not_compound(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    uint8_t twice[I6C_ISP_IQ_SHARPNESS_PAYLOAD];
    unsigned int i;

    arm(&ctx, &st);
    hal_isp_set_sharpness(&ctx, 100);
    hal_isp_set_sharpness(&ctx, 100);
    hal_isp_set_sharpness(&ctx, 80);
    memcpy(twice, g_sharp, sizeof(twice));

    /* The same knob reached in one step from the pristine tuning. */
    arm(&ctx, &st);
    hal_isp_set_sharpness(&ctx, 80);

    for (i = 0; i < 6; i++)
        CHECK(run_at(twice, SHARP_RUN, 1, i) == run_at(g_sharp, SHARP_RUN, 1, i),
              "field %u drifted: 200,200,160 gave %u where a plain 160 gives %u", i,
              run_at(twice, SHARP_RUN, 1, i), run_at(g_sharp, SHARP_RUN, 1, i));
}

/*
 * A tuning load puts the binary's own run back, so the baseline learned from
 * the previous one is not the neutral any more. Scaling about a stale baseline
 * is the same defect as compounding, one tuning apart.
 */
static void test_a_tuning_load_relearns_the_baseline(void)
{
    static const uint16_t retuned[6] = {80, 60, 40, 120, 100, 90};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    unsigned int i;

    arm(&ctx, &st);
    hal_isp_set_sharpness(&ctx, 100);

    /* A second load: the module reads back as the new binary wrote it. */
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, retuned, 6, I6C_ISP_OP_AUTO);
    i6c_isp_flush_knobs(&st);

    CHECK(g_iq[IQ_SHARPNESS].base_valid, "the new run must be adopted");
    for (i = 0; i < 6; i++)
        CHECK(g_iq[IQ_SHARPNESS].base[i] == retuned[i],
              "baseline %u must come from the new tuning: want %u, got %u", i, retuned[i],
              g_iq[IQ_SHARPNESS].base[i]);

    /*
     * And the pending 100 was re-applied against it, not against the old one.
     * The new run's best-conditioned field is 1, whose baseline is 60, so the
     * pivot is 60 again and field 0 lands at 80 + (100-60)*(127-80)/(127-60).
     */
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 80 + 40 * (127 - 80) / 67,
          "the re-applied knob must scale about the new baseline, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));
}

/*
 * One field outside MI's range means the offset or the width is wrong for this
 * module, not that one gain is unusual -- so the whole run is refused rather
 * than adopted piecemeal, which would scale five good fields about a misread
 * sixth and look very nearly right.
 */
static void test_an_out_of_range_run_is_refused_whole(void)
{
    static const uint16_t bad[6] = {40, 30, 200, 60, 50, 45};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, bad, 6, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "the knob still works");
    CHECK(!g_iq[IQ_SHARPNESS].base_valid, "a bad run must not be adopted");
    /* Every field falls back to the row's constant neutral, 63, which is then
     * both the pivot and every field's baseline -- so the run goes flat at
     * whatever was asked for. */
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 100,
          "field 0 must scale about the fallback neutral, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == run_at(g_sharp, SHARP_RUN, 1, 3),
          "with no baseline every field scales the same way");
}

/*
 * An all-zero run is a tuning that turned the module off, not a failed read. It
 * is adopted, and it makes neutral mean off with the whole knob above it.
 */
static void test_a_zero_run_is_a_baseline_not_a_failure(void)
{
    static const uint16_t off[6] = {0, 0, 0, 0, 0, 0};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, off, 6, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "set must succeed");
    CHECK(g_iq[IQ_SHARPNESS].base_valid, "zero is in range and must be adopted");
    /* With every baseline at zero the pivot is zero too, so the whole knob is
     * upward from nothing and the field simply becomes what was asked for. */
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 100, "must scale up from nothing, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, off, 6, I6C_ISP_OP_AUTO);
    CHECK(hal_isp_set_sharpness(&ctx, 30) == RSS_OK, "set must succeed");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 30, "a smaller ask is still just the ask, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));
}

/*
 * A baseline sitting on its module's ceiling. Sharpness's gains bound at 127, so
 * a tuning that asked for all of one is at unity and at maximum at once -- and
 * the knob must still work downwards from there. It used to bail on `unity >=
 * max` and lose the whole lower half.
 */
static void test_a_baseline_on_the_ceiling_still_scales_down(void)
{
    static const uint16_t maxed[6] = {127, 127, 127, 127, 127, 127};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    int val;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, maxed, 6, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sharpness(&ctx, 30) == RSS_OK, "set must succeed");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 30, "the lower half must still move, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));

    /* And upward there is simply no headroom, which is not the same as broken. */
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, maxed, 6, I6C_ISP_OP_AUTO);
    reset_row(IQ_SHARPNESS, fake_get_sharp, fake_set_sharp);
    /* And with the pivot itself on the ceiling the whole range is below it, so
     * every value the field can hold is now askable -- where the abstract
     * scale had no headroom above neutral and lost that half entirely. */
    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "set must succeed");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 100, "and the upper half too, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));

    /*
     * Reading it back used to be the ambiguous half: the field held MI's
     * maximum and so did the baseline, so it was both "at the ceiling" and
     * "where the tuning left it", and the getter had to choose. It reports the
     * field now, and the mode says which of the two it is -- so there is
     * nothing left to disambiguate.
     */
    val = 0;
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 100, "the field reports itself, got %d", val);
}

/* Everything outside the run is the tuning's and stays byte-identical. The
 * fields on either side of it are a coring threshold and a kernel weight, so a
 * write that overshot would soften the picture as sharpness went up. */
static void test_a_vector_write_touches_only_the_run(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    uint8_t before[I6C_ISP_IQ_SHARPNESS_PAYLOAD];

    arm(&ctx, &st);
    /* A recognisable pattern everywhere, then the tuning's run over the top. */
    memset(g_sharp, 0x5a, sizeof(g_sharp));
    i6c_iq_write(g_sharp, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_AUTO);
    for (unsigned int i = 0; i < 6; i++)
        i6c_iq_write(g_sharp, SHARP_RUN + i, 1, g_tuned_sharp[i]);
    memcpy(before, g_sharp, sizeof(before));

    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "set must succeed");

    CHECK(memcmp(before, g_sharp, I6C_ISP_OPTYPE_OFF) == 0, "bEnable must not move");
    CHECK(memcmp(before + I6C_ISP_OPTYPE_OFF + 4, g_sharp + I6C_ISP_OPTYPE_OFF + 4,
                 SHARP_RUN - I6C_ISP_OPTYPE_OFF - 4) == 0,
          "the auto table and everything before the run must not move");
    CHECK(memcmp(before + SHARP_RUN + 6, g_sharp + SHARP_RUN + 6, sizeof(before) - SHARP_RUN - 6) ==
              0,
          "everything after the run must not move");
}

/*
 * Reading back. An auto module is reporting the tuning and neutral is what
 * raptor calls that; a manual one is unscaled from the first field of the run
 * against the same baseline it was scaled by.
 */
static void test_a_vector_row_reads_back(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    int val = 0;

    arm(&ctx, &st);
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == RSS_ISP_AUTO, "an auto module reads as auto, got %d", val);

    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "set must succeed");
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    /*
     * Exactly 100, not near it. The row reports from field 3, whose baseline
     * of 60 is the furthest from both bounds of the six, and that field is the
     * one the value is expressed on -- so what was asked for is what comes
     * back. This assertion used to read 198 for a knob set to 200, because the
     * value crossed a 0..255 scale twice and truncated on each pass.
     */
    CHECK(g_iq[IQ_SHARPNESS].report == 3, "field 3 is the best conditioned, picked %u",
          g_iq[IQ_SHARPNESS].report);
    CHECK(val == 100, "the value comes back exactly, got %d", val);

    /*
     * Back to auto, which is where the mode matters and nothing else does.
     * The module goes to auto and its manual run keeps the 91 the last set left
     * there -- so a reader that reported the run regardless of the mode would
     * answer 91 for a knob that is once again the tuning's. Only the check on
     * enOpType separates them, and only for vector rows does it have to reach
     * past IQ_AUTOMAN to do it.
     */
    CHECK(hal_isp_set_sharpness(&ctx, RSS_ISP_AUTO) == RSS_OK, "set must succeed");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 91, "auto leaves the run where it was, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == RSS_ISP_AUTO, "a module handed back to the tuning reads as auto, got %d", val);

    /* Before the tuning has loaded there is nothing to ask, and the queue is a
     * better answer than the hardware's. */
    arm(&ctx, &st);
    st.isp_knobs_live = false;
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_ERR_BUSY, "nothing set yet is BUSY");
    CHECK(hal_isp_set_sharpness(&ctx, 77) == RSS_OK, "set must be queued");
    CHECK(g_sharp_sets == 0, "a queued knob must not reach MI, reached it %d times", g_sharp_sets);
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must answer from the queue");
    CHECK(val == 77, "the queued value is the answer, got %d", val);
}

/*
 * The readback defect this board actually had.
 *
 * The IMX335 tuning here leaves the first sharpening gain -- low-frequency
 * undirectional -- at 0 of 127. Reporting from field 0 unconditionally meant
 * half the knob unscaled to neutral: set-sharpness 64 read back 128, and so did
 * set-sharpness 0, while every field of the run had in fact moved. So the row
 * reports from the best-conditioned field instead, and this pins that.
 */
static void test_the_reported_field_avoids_a_baseline_on_a_bound(void)
{
    /* The shape measured on the board: field 0 off, field 5 near the ceiling,
     * and the useful ones in between. */
    static const uint16_t tuned[6] = {0, 20, 45, 60, 80, 100};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    int val = 0;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, tuned, 6, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sharpness(&ctx, 30) == RSS_OK, "set must succeed");
    /* min(base, 127 - base) is largest at 60, which is field 3. */
    CHECK(g_iq[IQ_SHARPNESS].report == 3,
          "must report from the field furthest from both bounds, "
          "picked %u",
          g_iq[IQ_SHARPNESS].report);
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 30, "the reported field answers with the value asked for, got %d", val);

    /* Field 0 did move; it is only useless to read from. */
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 0, "field 0 scales to its floor");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 3) == 30, "field 3 is half its baseline, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 3));

    /* And 0 is distinguishable from neutral, which it was not before. */
    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, tuned, 6, I6C_ISP_OP_AUTO);
    CHECK(hal_isp_set_sharpness(&ctx, 0) == RSS_OK, "set must succeed");
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 0, "the floor must read as the floor, got %u", val);
}

/*
 * A refused baseline takes the reported field with it. Without a baseline every
 * field is scaled against the same constant, so there is nothing to prefer one
 * by, and keeping the index chosen for the *previous* tuning would have the
 * getter answer from an arbitrary field of the run for a reason no longer true.
 */
static void test_a_refused_baseline_resets_the_reported_field(void)
{
    static const uint16_t good[6] = {0, 20, 45, 60, 80, 100};
    static const uint16_t bad[6] = {10, 20, 45, 90, 80, 200};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    int val = 0;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, good, 6, I6C_ISP_OP_AUTO);
    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "set must succeed");
    CHECK(g_iq[IQ_SHARPNESS].report == 3, "the good run picks field 3, picked %u",
          g_iq[IQ_SHARPNESS].report);

    /* A reload whose run this port cannot read: one field out of range. */
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, bad, 6, I6C_ISP_OP_MANUAL);
    g_iq[IQ_SHARPNESS].tuning_stale = true;

    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(!g_iq[IQ_SHARPNESS].base_valid, "the bad run must be refused");
    CHECK(g_iq[IQ_SHARPNESS].report == 0, "and the reported field must go back to 0, is %u",
          g_iq[IQ_SHARPNESS].report);
    /* Field 0 holds 10, and reports it. */
    CHECK(val == 10, "must read field 0 as it stands, got %d", val);
}

/*
 * A run entirely on a bound. There is no best-conditioned field to pick, so
 * field 0 answers -- and it answers with the value that was asked for, because
 * the pivot and the baselines coincide there and the remap is the identity.
 *
 * This used to be the awkward case: with an abstract scale, a tuning that had
 * turned the module off made "off" and "as the tuning left it" the same
 * number, and the getter had to pick one. Nothing to pick between now.
 */
static void test_a_run_entirely_on_a_bound_reports_the_field(void)
{
    static const uint16_t off[6] = {0, 0, 0, 0, 0, 0};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    int val = 0;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, off, 6, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sharpness(&ctx, 30) == RSS_OK, "set must succeed");
    CHECK(g_iq[IQ_SHARPNESS].report == 0, "with nothing to choose between, field 0");
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 30, "an off module still reports its field, got %d", val);
}

/* A queued knob is applied by the flush, and against the baseline the load that
 * triggered the flush left behind. */
static void test_a_vector_knob_queued_before_the_load_is_flushed(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    st.isp_knobs_live = false;

    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "set must be queued");
    CHECK(g_sharp_sets == 0, "nothing may reach MI yet");

    i6c_isp_flush_knobs(&st);
    CHECK(g_sharp_sets == 1, "the flush must apply it once, applied %d", g_sharp_sets);
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 91, "want 91, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));
}

/* A row whose run does not fit is refused at resolve, before any symbol is
 * loaded -- the two ways it does not fit being longer than base[] and reaching
 * past the payload. */
static void test_a_run_that_does_not_fit_is_refused(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    i6c_iq_param_t *p = &g_iq[IQ_SHARPNESS];
    uint8_t saved_count = p->count;
    uint16_t saved_off = p->manual_off;

    arm(&ctx, &st);
    /* Unresolved, with a library handle resolve must not get as far as using. */
    p->fn_get = NULL;
    p->fn_set = NULL;
    st.isp.lib = (void *)(uintptr_t)1;

    p->count = I6C_IQ_VECTOR_MAX + 1;
    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_ERR_INVAL, "a run longer than base[] is refused");

    p->count = saved_count;
    p->manual_off = (uint16_t)(p->payload - 2);
    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_ERR_INVAL,
          "a run reaching past the payload is refused");

    p->count = saved_count;
    p->manual_off = saved_off;
    reset_row(IQ_SHARPNESS, fake_get_sharp, fake_set_sharp);
}

/* A module that will not answer changes nothing, and says so rather than
 * writing a run scaled about a buffer that was never filled. */
static void test_a_failing_module_writes_nothing(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    g_sharp_get_ret = -1;
    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_ERR_IO, "a failing read is IO");
    CHECK(g_sharp_sets == 0, "nothing may be written, wrote %d", g_sharp_sets);
    CHECK(!g_iq[IQ_SHARPNESS].base_valid, "and no baseline may be adopted");

    arm(&ctx, &st);
    g_sharp_set_ret = -1;
    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_ERR_IO, "a failing write is IO");
}

/*
 * The remap that replaced the scalar scale, on its own terms.
 *
 * There used to be an abstract 0..255 in front of every row and a pair of
 * functions mapping to and from MI's range, and this test pinned that pair.
 * The mapping could not round-trip -- 256 inputs onto a range of 101 or 41 --
 * so brightness 140 read back 138. Scalar rows now go to the field unchanged
 * and the remap survives only for the vector rows, where one number has to
 * drive six fields and there is no single field for it to be.
 *
 * The pivot is the reported field's baseline, so the identity that matters is
 * that asking for the pivot puts every field back exactly where the tuning had
 * it -- for any baselines at all, not just this test's.
 */
static void test_the_remap_is_the_identity_at_the_pivot(void)
{
    static const int32_t base[6] = {40, 30, 20, 60, 50, 45};
    unsigned int i;

    for (i = 0; i < 6; i++)
        CHECK(i6c_iq_reband(60, 60, base[i], 0, 127) == base[i],
              "field %u must return to its baseline %d, got %d", i, base[i],
              i6c_iq_reband(60, 60, base[i], 0, 127));

    /* The ends are MI's own, whatever the pivot. */
    CHECK(i6c_iq_reband(0, 60, 40, 0, 127) == 0, "the floor is the floor");
    CHECK(i6c_iq_reband(127, 60, 40, 0, 127) == 127, "the ceiling is the ceiling");

    /* Monotonic across the join, so a knob never doubles back on itself. */
    {
        int32_t prev = i6c_iq_reband(0, 60, 40, 0, 127);
        int v;

        for (v = 1; v <= 127; v++) {
            int32_t got = i6c_iq_reband(v, 60, 40, 0, 127);

            CHECK(got >= prev, "remap must be monotonic: %d gave %d after %d", v, got, prev);
            prev = got;
        }
    }

    /*
     * A pivot on either bound. The half with no span is unreachable rather
     * than dangerous -- the clamps and the equality test take every value that
     * would land there -- and the other half interpolates normally, so the
     * knob keeps its whole range instead of collapsing. A tuning that asks for
     * full sharpening gain puts the pivot on the ceiling exactly like this,
     * and the old scale lost everything above neutral when it did.
     */
    CHECK(i6c_iq_reband(10, 0, 40, 0, 127) == 40 + 10 * (127 - 40) / 127,
          "a pivot on the floor still interpolates upward, got %d",
          i6c_iq_reband(10, 0, 40, 0, 127));
    CHECK(i6c_iq_reband(100, 127, 40, 0, 127) == 100 * 40 / 127,
          "a pivot on the ceiling still interpolates downward, got %d",
          i6c_iq_reband(100, 127, 40, 0, 127));
    CHECK(i6c_iq_reband(0, 127, 40, 0, 127) == 0, "and still reaches the floor");
}

/*
 * DRC has to mean on raptor what it meant on majestic, because that is where
 * the numbers in people's configs come from: overrideWdr wrote its value
 * straight into WDR's manual Strength byte, so 100 is 100 and 200 is 200. The
 * value goes to the field unchanged, which is now true of every scalar row
 * rather than an accident of this one's unity -- and this is what pins it down.
 * The one thing that has changed for the better: 128 was the one strength the
 * knob could not ask for, because it was spent on auto. It is reachable now.
 */
static void test_drc_carries_the_majestic_scale_onto_wdr(void)
{
    static const uint16_t tuned[1] = {30}; /* imx335's own auto Strength at gain 0 */
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    seed_run(g_wdr, sizeof(g_wdr), DRC_OFF, 1, tuned, 1, I6C_ISP_OP_AUTO);
    i6c_iq_write(g_wdr, I6C_ISP_ENABLE_OFF, 4, 1);

    CHECK(hal_isp_set_drc_strength(&ctx, 100) == RSS_OK, "a mid-scale drc must take");
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 100, "drc 100 must write Strength 100, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_MANUAL,
          "a level has to leave auto to take effect");

    CHECK(hal_isp_set_drc_strength(&ctx, 200) == RSS_OK, "a strong drc must take");
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 200, "drc 200 must write Strength 200, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));

    /* The ends are the field's own ends rather than anything scaled. */
    CHECK(hal_isp_set_drc_strength(&ctx, 0) == RSS_OK, "drc 0 must take");
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 0, "drc 0 is Strength 0, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));
    CHECK(hal_isp_set_drc_strength(&ctx, 255) == RSS_OK, "drc 255 must take");
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 255, "drc 255 is Strength 255, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));

    /*
     * 128 is now just a strength like any other -- it used to be the one value
     * this knob could not ask for, because it was spent on meaning auto.
     */
    CHECK(hal_isp_set_drc_strength(&ctx, 128) == RSS_OK, "drc 128 must take");
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 128, "drc 128 is Strength 128, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));

    /* Auto hands the module back to the tuning and leaves the byte alone --
     * the curve is what takes over, so whatever sits in manual stops mattering. */
    CHECK(hal_isp_set_drc_strength(&ctx, RSS_ISP_AUTO) == RSS_OK, "auto must take");
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO,
          "auto must put WDR back into auto");
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 128, "auto must not rewrite the level, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));
}

/*
 * majestic forces bEnable on with every write. This must not, and the
 * difference is visible only on a sensor whose tuner switched WDR off --
 * Infinity6E ships two. Turning it on there would be this layer overruling a
 * tuning decision it has no basis to overrule, so the write goes in and the
 * module stays off.
 */
/*
 * Naming a value switches the module on.
 *
 * The alternative, and what this did until the IMX335 board made the cost
 * plain: a knob that accepts the write, reads back exactly what was asked for,
 * and changes nothing on screen. Brightness ships disabled in that sensor's
 * tuning, so `brightness = 60` was inert and nothing said so anywhere an
 * operator would look.
 */
static void test_a_value_switches_on_a_module_the_tuning_disabled(void)
{
    static const uint16_t tuned[1] = {0};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    seed_run(g_wdr, sizeof(g_wdr), DRC_OFF, 1, tuned, 1, I6C_ISP_OP_AUTO);
    i6c_iq_write(g_wdr, I6C_ISP_ENABLE_OFF, 4, 0);

    CHECK(hal_isp_set_drc_strength(&ctx, 200) == RSS_OK, "the set must take");
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4) == 1,
          "the module must be switched on, enable reads %u",
          i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4));
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 200, "and carry the level, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_MANUAL, "and be in manual");
}

/*
 * And auto hands the switch back, so the tuner's own state is recoverable at
 * runtime rather than only by reloading the binary. Without this the first
 * write would be one-way: the module would stay on for the rest of the boot,
 * running the tuning's auto curve for a module the tuner had turned off.
 */
static void test_auto_hands_the_tuners_switch_back(void)
{
    static const uint16_t tuned[1] = {0};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    seed_run(g_wdr, sizeof(g_wdr), DRC_OFF, 1, tuned, 1, I6C_ISP_OP_AUTO);
    i6c_iq_write(g_wdr, I6C_ISP_ENABLE_OFF, 4, 0);

    CHECK(hal_isp_set_drc_strength(&ctx, 200) == RSS_OK, "the set must take");
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4) == 1, "on, as above");

    CHECK(hal_isp_set_drc_strength(&ctx, RSS_ISP_AUTO) == RSS_OK, "auto must take");
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4) == 0,
          "auto must put the tuning's switch back, enable reads %u",
          i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4));
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO, "and the mode with it");
}

/* A module the tuner left on stays on, and auto does not switch it off. The
 * switch that is put back is the tuning's, not a constant. */
static void test_a_module_the_tuning_enabled_is_left_enabled(void)
{
    static const uint16_t tuned[1] = {90};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    seed_run(g_wdr, sizeof(g_wdr), DRC_OFF, 1, tuned, 1, I6C_ISP_OP_AUTO);
    i6c_iq_write(g_wdr, I6C_ISP_ENABLE_OFF, 4, 1);

    CHECK(hal_isp_set_drc_strength(&ctx, 200) == RSS_OK, "the set must take");
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4) == 1, "still on");
    CHECK(hal_isp_set_drc_strength(&ctx, RSS_ISP_AUTO) == RSS_OK, "auto must take");
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4) == 1,
          "auto must not switch off a module the tuning had on, enable reads %u",
          i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4));
}

/* A tuning load re-reads the switch, so a second sensor's opinion is the one
 * that gets handed back -- not the first sensor's, cached. */
static void test_a_tuning_load_relearns_the_switch(void)
{
    static const uint16_t tuned[1] = {0};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    seed_run(g_wdr, sizeof(g_wdr), DRC_OFF, 1, tuned, 1, I6C_ISP_OP_AUTO);
    i6c_iq_write(g_wdr, I6C_ISP_ENABLE_OFF, 4, 0);
    CHECK(hal_isp_set_drc_strength(&ctx, 200) == RSS_OK, "the set must take");

    /* A new tuning, which has this module on. */
    seed_run(g_wdr, sizeof(g_wdr), DRC_OFF, 1, tuned, 1, I6C_ISP_OP_AUTO);
    i6c_iq_write(g_wdr, I6C_ISP_ENABLE_OFF, 4, 1);
    i6c_isp_flush_knobs(&st);

    CHECK(hal_isp_set_drc_strength(&ctx, RSS_ISP_AUTO) == RSS_OK, "auto must take");
    CHECK(i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4) == 1,
          "the new tuning's switch is the one to hand back, enable reads %u",
          i6c_iq_read(g_wdr, I6C_ISP_ENABLE_OFF, 4));
}

/* The vector shape has the same header and gets the same treatment. */
static void test_a_vector_value_switches_its_module_on_too(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    i6c_iq_write(g_sharp, I6C_ISP_ENABLE_OFF, 4, 0);

    CHECK(hal_isp_set_sharpness(&ctx, 100) == RSS_OK, "the set must take");
    CHECK(i6c_iq_read(g_sharp, I6C_ISP_ENABLE_OFF, 4) == 1,
          "a vector row must switch its module on, enable reads %u",
          i6c_iq_read(g_sharp, I6C_ISP_ENABLE_OFF, 4));

    CHECK(hal_isp_set_sharpness(&ctx, RSS_ISP_AUTO) == RSS_OK, "auto must take");
    CHECK(i6c_iq_read(g_sharp, I6C_ISP_ENABLE_OFF, 4) == 0, "and hand the switch back");
}

/*
 * Defog is the exception, because raptor already publishes its switch: two rows
 * address one module, so a strength write that switched it on would countermand
 * an operator who had just turned defog off -- and, worse, do it from the
 * config every startup, since rvd writes both before the first frame.
 */
static void test_defog_strength_leaves_defogs_own_switch_alone(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    i6c_iq_write(g_defog, I6C_ISP_ENABLE_OFF, 4, 0);
    i6c_iq_write(g_defog, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_defog_strength(&ctx, 200) == RSS_OK, "the set must take");
    CHECK(i6c_iq_read(g_defog, I6C_ISP_ENABLE_OFF, 4) == 0,
          "defog's own switch must survive a strength write, enable reads %u",
          i6c_iq_read(g_defog, I6C_ISP_ENABLE_OFF, 4));
    CHECK(i6c_iq_read(g_defog, I6C_ISP_IQ_DEFOG_MANUAL, 1) == 200, "the level still lands, got %u",
          i6c_iq_read(g_defog, I6C_ISP_IQ_DEFOG_MANUAL, 1));

    /* And the switch is still the operator's to throw. */
    CHECK(hal_isp_set_defog(&ctx, true) == RSS_OK, "the switch must take");
    CHECK(i6c_iq_read(g_defog, I6C_ISP_ENABLE_OFF, 4) == 1, "and be what turns the module on");
}

/* A drc write must touch the level and the mode word and nothing else: the
 * payload it read back carries the whole module, including a 33-entry curve
 * either side of the byte. */
static void test_a_drc_write_touches_only_the_level(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    uint8_t before[I6C_ISP_IQ_WDR_PAYLOAD];
    size_t i, differ = 0;

    arm(&ctx, &st);
    for (i = 0; i < sizeof(g_wdr); i++)
        g_wdr[i] = (uint8_t)(i * 7 + 3);
    i6c_iq_write(g_wdr, I6C_ISP_OPTYPE_OFF, 4, I6C_ISP_OP_AUTO);
    i6c_iq_write(g_wdr, I6C_ISP_ENABLE_OFF, 4, 1);
    memcpy(before, g_wdr, sizeof(before));

    CHECK(hal_isp_set_drc_strength(&ctx, 77) == RSS_OK, "the write must take");
    for (i = 0; i < sizeof(g_wdr); i++)
        if (g_wdr[i] != before[i]) {
            differ++;
            /* Two, and named rather than counted: the level, and the low byte
             * of the mode word going 0 -> 1. Counting alone would pass just as
             * happily if the write had landed two bytes further along. */
            CHECK(i == DRC_OFF || i == I6C_ISP_OPTYPE_OFF, "byte %zu moved and should not have", i);
        }
    CHECK(differ == 2, "only the level and the mode byte may move, %zu bytes moved", differ);
    CHECK(run_at(g_wdr, DRC_OFF, 1, 0) == 77, "and the level is the one asked for, got %u",
          run_at(g_wdr, DRC_OFF, 1, 0));
}

/*
 * The AWB diagnostic must be able to print at all.
 *
 * AE and AWB shared one latch, set on the first AE success -- and AE answers
 * before AWB has a result, every time, because its loop converges first. So the
 * latch was always spent on a call where AWB had nothing to say, and
 * "isp/awb: gains ..." could never be printed. Measured on an SSC377QE: not one
 * isp/awb line in a whole boot, while ric was being handed r=1604 b=2341
 * through this very function.
 *
 * The latches are function-static, so this runs the whole sequence in one test:
 * a first call with AWB not yet answering, then one where it does.
 */
static int g_awb_ret;

static int fake_ae_status(unsigned int device, unsigned int channel, i6c_cus_ae_info *info)
{
    (void)device;
    (void)channel;
    memset(info, 0, sizeof(*info));
    info->shutterUs = 55785;
    info->sensorGain = 65536;
    info->ispGain = 2048;
    info->preAvgY = 70;
    return 0;
}

static int fake_awb_status(unsigned int device, unsigned int channel, i6c_cus_awb_info *info)
{
    (void)device;
    (void)channel;
    if (g_awb_ret)
        return g_awb_ret;
    memset(info, 0, sizeof(*info));
    info->rGain = 1604;
    info->gGain = 1024;
    info->bGain = 2341;
    return 0;
}

static void test_the_awb_line_survives_ae_winning_the_race(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    rss_exposure_t exp;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp.ae_status = fake_ae_status;
    st.isp.awb_status = fake_awb_status;
    st.iq_load_started = 1;

    /* AE first, as it always is, with AWB not answering yet. */
    log_reset();
    g_awb_ret = -1;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "the AE readback must succeed");
    CHECK(strstr(g_log, "isp/ae:") != NULL, "the AE line must be logged");
    CHECK(strstr(g_log, "isp/awb:") == NULL, "no AWB line before AWB has an answer");

    /* And now AWB answers. This is the line the single latch used to eat. */
    log_reset();
    g_awb_ret = 0;
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "the readback must succeed");
    CHECK(strstr(g_log, "isp/awb: gains r=1604 g=1024 b=2341") != NULL,
          "the AWB line must be logged once AWB answers, got: %s", g_log);
    CHECK(strstr(g_log, "isp/ae:") == NULL, "the AE line must not repeat");

    /* The gains reach the caller too, green included -- ric and get-isp both
     * read them from here. */
    CHECK(exp.wb_rgain == 1604 && exp.wb_ggain == 1024 && exp.wb_bgain == 2341,
          "the gains must reach the caller, got r=%u g=%u b=%u", exp.wb_rgain, exp.wb_ggain,
          exp.wb_bgain);

    /* Once each, and no more. */
    log_reset();
    CHECK(hal_isp_get_exposure(&ctx, &exp) == RSS_OK, "the readback must succeed");
    CHECK(g_log_len == 0, "neither line repeats, got: %s", g_log);
}

/*
 * Temper is gone from this backend, and this is what says so.
 *
 * The ISP channel's e3DNRLevel is not a strength -- it selects the 3DNR
 * reference frame's bit depth -- so publishing it as one described a control
 * the hardware does not have. The ops are absent rather than stubbed, which
 * means RSS_HAL_CALL answers RSS_ERR_NOTSUP and get-isp-caps stops listing the
 * knob; the assertion here is on the caps side, since a removed op cannot be
 * called to be tested.
 *
 * st->isp_nr3d_req is deliberately still live: the driver gates its flip and
 * rotate predicates on 3DNR being on, so the seed has to reach the channel.
 * That path is hal_framesource's, not a knob's.
 */
static void test_sinter_is_not_published(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    rss_isp_knob_t caps;
    void *c = &ctx;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;

    /*
     * Spatial luma denoise stays unpublished: NrLumaAdv's only field is a blend
     * weight the shipped tunings already run at maximum, so the knob's whole
     * usable range would be its own default. Temper used to be here for a
     * different reason and no longer is -- see below.
     */
    CHECK(hal_isp_get_knob_caps(c, "sinter", &caps) == RSS_ERR_NOTSUP,
          "sinter must not publish caps");

    /*
     * Sharpness joined it. The row still works and the tests below still drive
     * it directly, but the knob is withdrawn: leaving auto costs a manual block
     * whose EdgeGain is 8 against the curve's 40, and no value of the six the
     * run reaches makes that back. Measured 4790 grad in auto against 1794 at
     * sharpness 127 on an SSC377QE.
     */
    CHECK(hal_isp_get_knob_caps(c, "sharpness", &caps) == RSS_ERR_NOTSUP,
          "sharpness must not publish caps");

    /* The knobs that remain still do, so the lookup itself is not just broken. */
    CHECK(hal_isp_get_knob_caps(c, "defog_strength", &caps) == RSS_OK,
          "defog_strength still publishes caps");
}

/* imx335's own TfStrY, flat across the ladder, as every 6C tuning ships it. */
static const uint16_t g_tuned_tf[16] = {63, 63, 63, 63, 63, 63, 63, 63,
                                        63, 63, 63, 63, 63, 63, 63, 63};

static void arm_nr3d(rss_hal_ctx_t *ctx, infinity6c_state_t *st, uint32_t optype)
{
    arm(ctx, st);
    seed_gain_run(g_nr3d, I6C_ISP_IQ_NR3D_AUTO + I6C_ISP_IQ_NR3D_TFSTRY, I6C_ISP_IQ_NR3D_ENTRY, 1,
                  g_tuned_tf, I6C_ISP_IQ_NR3D_AUTO_NUM, NR3D_MDGAIN, optype);
}

/*
 * The property the whole shape exists for.
 *
 * MI interpolates stAuto by gain, so a knob that wrote one entry would be a knob
 * that worked at one exposure. All sixteen carry the value, and enOpType is
 * still auto afterwards -- which is what keeps MI interpolating at all.
 */
static void test_temper_writes_every_gain_entry_and_stays_in_auto(void)
{
    unsigned int off = I6C_ISP_IQ_NR3D_AUTO + I6C_ISP_IQ_NR3D_TFSTRY;
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    unsigned int i;

    arm_nr3d(&ctx, &st, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_temper_strength(&ctx, 90) == RSS_OK, "temper 90 must take");
    for (i = 0; i < I6C_ISP_IQ_NR3D_AUTO_NUM; i++)
        CHECK(gain_at(g_nr3d, off, 1, i) == 90, "entry %u must carry 90, got %u", i,
              gain_at(g_nr3d, off, 1, i));

    CHECK(i6c_iq_read(g_nr3d, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_AUTO,
          "the module must still be in auto -- going manual is what this shape avoids");

    /* Both ends of the published range, since the vendor's advice stops at 64
     * and raptor deliberately publishes past it. */
    CHECK(hal_isp_set_temper_strength(&ctx, 0) == RSS_OK, "temper 0 must take");
    CHECK(gain_at(g_nr3d, off, 1, 7) == 0, "temper 0 is TfStrY 0");
    CHECK(hal_isp_set_temper_strength(&ctx, 127) == RSS_OK, "temper 127 must take");
    CHECK(gain_at(g_nr3d, off, 1, 7) == 127, "temper 127 is TfStrY 127");
    CHECK(hal_isp_set_temper_strength(&ctx, 128) == RSS_ERR_INVAL, "past the field is refused");
    CHECK(hal_isp_set_temper_strength(&ctx, -1) == RSS_ERR_INVAL, "below the field is refused");
}

/*
 * And the thing it is worth paying sixteen writes for: everything else in the
 * entry is the tuner's, and stays the tuner's.
 *
 * MdGain stands in for the rest here because it is the field that actually
 * ramps -- 125 to 900 across the ladder on the shipped imx335 bin. A stride
 * error would land the strength inside one of these instead, and this is what
 * catches it: the failure is not a wrong TfStrY but an intact one next to a
 * corrupted curve.
 */
static void test_temper_leaves_the_rest_of_the_gain_curve_alone(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    unsigned int i;

    arm_nr3d(&ctx, &st, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_temper_strength(&ctx, 20) == RSS_OK, "temper 20 must take");
    for (i = 0; i < I6C_ISP_IQ_NR3D_AUTO_NUM; i++)
        CHECK(neighbour_at(g_nr3d, i) == 100 + i * 50,
              "entry %u's curve must be untouched: want %u, got %u", i, 100 + i * 50,
              neighbour_at(g_nr3d, i));
}

/*
 * Auto is a restore here, not a mode change, and that is the one way this shape
 * is harder than the others. Every other row hands a module back by setting
 * enOpType and leaving stManual stale; this one has overwritten the only copy of
 * the tuning's numbers, so auto has to put them back from the baseline.
 */
static void test_temper_auto_puts_the_tunings_entries_back(void)
{
    unsigned int off = I6C_ISP_IQ_NR3D_AUTO + I6C_ISP_IQ_NR3D_TFSTRY;
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    unsigned int i;
    int val;

    arm_nr3d(&ctx, &st, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_temper_strength(&ctx, 110) == RSS_OK, "temper 110 must take");
    CHECK(hal_isp_get_temper_strength(&ctx, &val) == RSS_OK && val == 110,
          "a written knob reads back its value, got %d", val);

    CHECK(hal_isp_set_temper_strength(&ctx, RSS_ISP_AUTO) == RSS_OK, "auto must take");
    for (i = 0; i < I6C_ISP_IQ_NR3D_AUTO_NUM; i++)
        CHECK(gain_at(g_nr3d, off, 1, i) == g_tuned_tf[i],
              "entry %u must be back to the tuning's %u, got %u", i, g_tuned_tf[i],
              gain_at(g_nr3d, off, 1, i));

    /*
     * And it reads as auto afterwards. enOpType cannot answer that for this
     * shape -- it never left auto -- so the row's own flag is what does, and a
     * caller must not be able to tell the difference.
     */
    CHECK(hal_isp_get_temper_strength(&ctx, &val) == RSS_OK && val == RSS_ISP_AUTO,
          "a restored knob reads back as auto, got %d", val);
}

/*
 * The neutral is the tuning's, because no constant is right for every sensor.
 * 64 is the vendor's 1x and is only the fallback for a board with no tuning
 * file at all.
 */
static void test_temper_caps_come_from_the_tuning(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    rss_isp_knob_t caps;
    void *c = &ctx;

    arm_nr3d(&ctx, &st, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_get_knob_caps(c, "temper", &caps) == RSS_OK, "temper publishes caps now");
    CHECK(caps.min == 0 && caps.max == 127, "the full field, got %d..%d", caps.min, caps.max);
    CHECK(caps.neutral == 63, "the neutral is this tuning's own TfStrY, got %d", caps.neutral);
    CHECK(caps.has_auto, "auto is askable -- it restores the tuning's entries");
    CHECK(caps.enabled, "the seeded module is enabled");
}

/*
 * A tuning that ships NR3D in manual is reading stManual, so the entries this
 * writes are not the ones in use. Said rather than corrected: forcing auto would
 * overrule a tuning decision on the strength of a knob nobody has to set.
 */
static void test_temper_says_so_when_the_tuning_is_in_manual(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm_nr3d(&ctx, &st, I6C_ISP_OP_MANUAL);
    g_log_len = 0;
    g_log[0] = 0;

    CHECK(hal_isp_set_temper_strength(&ctx, 70) == RSS_OK, "the write still goes in");
    CHECK(strstr(g_log, "in manual") != NULL, "it must say the entries are not the ones in use");
    CHECK(i6c_iq_read(g_nr3d, I6C_ISP_OPTYPE_OFF, 4) == I6C_ISP_OP_MANUAL,
          "and must not have moved the module to auto behind the tuner");
}

/*
 * Neutral is the tuning's own rendering, exactly.
 *
 * The load-bearing test of the whole mechanism. Every knob below is expressed
 * as a departure from this, so if neutral is not a byte-for-byte reproduction
 * of what the tuner left then none of the others means what it says -- and the
 * failure would be invisible, because a picture that is slightly wrong
 * everywhere looks like a picture.
 *
 * Both spellings of neutral, because they are different requests: auto says the
 * tuning owns the knob and 50 says the operator has chosen the middle. They
 * write the same bytes on purpose, and only the getter tells them apart.
 */
static void test_neutral_reproduces_the_tunings_own_transform(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    uint8_t before[I6C_ISP_IQ_COLORTRANS_PAYLOAD];
    void *c = &ctx;
    int v;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_ctrans(ct_lim, CT_LIM_OFST);
    memcpy(before, g_ctrans, sizeof(before));

    CHECK(hal_isp_set_brightness(c, 50) == RSS_OK, "the middle is a legal value");
    CHECK(hal_isp_set_contrast(c, 50) == RSS_OK, "so is the middle of contrast");
    CHECK(hal_isp_set_saturation(c, 50) == RSS_OK, "and of saturation");

    CHECK(memcmp(g_ctrans + I6C_ISP_IQ_COLORTRANS_YOFST, before + I6C_ISP_IQ_COLORTRANS_YOFST,
                 sizeof(before) - I6C_ISP_IQ_COLORTRANS_YOFST) == 0,
          "the middle of all three knobs must reproduce the tuning byte for byte");
    CHECK(i6c_iq_read(g_ctrans, I6C_ISP_ENABLE_OFF, 4) == 1,
          "and must switch the module on, since nothing else will");

    CHECK(hal_isp_set_brightness(c, RSS_ISP_AUTO) == RSS_OK, "auto is a legal value");
    CHECK(hal_isp_set_contrast(c, RSS_ISP_AUTO) == RSS_OK, "for all three");
    CHECK(hal_isp_set_saturation(c, RSS_ISP_AUTO) == RSS_OK, "of them");
    CHECK(memcmp(g_ctrans + I6C_ISP_IQ_COLORTRANS_YOFST, before + I6C_ISP_IQ_COLORTRANS_YOFST,
                 sizeof(before) - I6C_ISP_IQ_COLORTRANS_YOFST) == 0,
          "auto writes the same bytes as the middle does");

    CHECK(hal_isp_get_brightness(c, &v) == RSS_OK && v == RSS_ISP_AUTO,
          "and is the one thing that reads back differently, got %d", v);
}

/*
 * Contrast pivots on mid-grey, not on black.
 *
 * Scaling the luma row alone multiplies every luma including the ones near
 * zero, so raising contrast would raise the whole picture's brightness with it
 * and lowering it would sink it. The offset term is what turns that into a
 * pivot, and the identity it has to satisfy is the one asserted here: mid-grey
 * in gives mid-grey out at any contrast.
 */
static void test_contrast_pivots_on_mid_grey(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    int32_t y_of_mid;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;

    /* Where the baseline puts mid-grey, before any knob has moved. */
    CHECK(hal_isp_set_contrast(c, I6C_CT_KNOB_UNITY) == RSS_OK, "the middle is legal");
    y_of_mid = ct_mid_grey();

    /*
     * What must not move is where the baseline put mid-grey, which is not
     * mid-grey itself: this conversion carries a pedestal, so 128 in comes out
     * at 126. Measured from the baseline rather than assumed, because a test
     * that assumed 128 would pass only for a full-range tuning and would have
     * agreed with a composer that tilts every limited-range one.
     */
    CHECK(y_of_mid == 126, "the baseline maps mid-grey to 126, got %d", y_of_mid);

    CHECK(hal_isp_set_contrast(c, I6C_CT_KNOB_MAX) == RSS_OK, "full contrast is legal");
    for (i = 0; i < I6C_CT_ROW_LEN; i++)
        CHECK(ct_coef(i) == 2 * ct_lim[i], "the luma row doubles at full contrast, %d gave %d", i,
              ct_coef(i));
    CHECK(ct_mid_grey() == y_of_mid, "and mid-grey must not move, got %d", ct_mid_grey());

    CHECK(hal_isp_set_contrast(c, 0) == RSS_OK, "so is none");
    CHECK(ct_mid_grey() == y_of_mid, "nor at none of it, got %d", ct_mid_grey());

    CHECK(hal_isp_set_contrast(c, 70) == RSS_OK, "nor anywhere between");
    CHECK(ct_mid_grey() == y_of_mid, "got %d", ct_mid_grey());

    /* Chroma is not contrast's business, and neither are the chroma offsets. */
    for (i = I6C_CT_ROW_U; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) == ct_lim[i], "contrast must not touch chroma, %d gave %d", i, ct_coef(i));
    CHECK(ct_ofst(1) == 0 && ct_ofst(2) == 0, "nor the chroma offsets");
}

/*
 * Saturation scales the chroma rows and nothing else.
 *
 * It needs no pivot term of its own, and that is a property of the hardware
 * rather than an omission: the 128 pedestal is added after the matrix, so a
 * scaled chroma row already pivots on neutral grey. The test that it works is
 * that the offsets stay where the tuning left them while the rows move.
 */
static void test_saturation_scales_only_the_chroma_rows(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_ctrans(ct_lim, CT_LIM_OFST);

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_UNITY / 2) == RSS_OK, "half saturation is legal");

    for (i = 0; i < I6C_CT_ROW_LEN; i++)
        CHECK(ct_coef(i) == ct_lim[i], "saturation must not touch luma, %d gave %d", i, ct_coef(i));
    for (i = I6C_CT_ROW_U; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++) {
        int32_t want = i6c_ct_scale(ct_lim[i], I6C_CT_KNOB_UNITY / 2);

        CHECK(ct_coef(i) == want, "chroma %d should be %d, got %d", i, want, ct_coef(i));
    }
    CHECK(ct_ofst(0) == CT_LIM_OFST, "the tuning's own luma offset survives, got %d", ct_ofst(0));
    CHECK(ct_ofst(1) == 0 && ct_ofst(2) == 0, "and the chroma offsets are not the pedestal");
}

/*
 * Brightness is four counts of the offset per count of eight-bit luma.
 *
 * The one knob with no gain in it, so the only thing that can be wrong is the
 * domain -- and getting that wrong is a factor of four, which looks like a knob
 * that either does nothing or saturates immediately.
 *
 * Asserted against literal counts rather than against the constants the
 * composer uses. Written the other way the test passes for any domain at all,
 * because both sides of it move together -- which is exactly the mistake it
 * exists to catch.
 */
static void test_brightness_moves_the_offset_and_nothing_else(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_ctrans(ct_lim, CT_LIM_OFST);

    /* 64 counts of luma at four counts of the field each, above the tuning's
     * own offset of 64: 64 + 256. */
    CHECK(hal_isp_set_brightness(c, I6C_CT_KNOB_MAX) == RSS_OK, "full brightness is legal");
    CHECK(ct_ofst(0) == 320, "the top of the knob is +64 luma over the tuning's 64, got %d",
          ct_ofst(0));

    CHECK(hal_isp_set_brightness(c, 0) == RSS_OK, "and so is none");
    CHECK(ct_ofst(0) == -192, "the bottom is the same distance the other way, got %d", ct_ofst(0));

    /* And the middle is the tuning's own offset, untouched. */
    CHECK(hal_isp_set_brightness(c, I6C_CT_KNOB_UNITY) == RSS_OK, "as is the middle");
    CHECK(ct_ofst(0) == 64, "which must leave the offset exactly as found, got %d", ct_ofst(0));

    for (i = 0; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) == ct_lim[i], "brightness must not touch the matrix, %d gave %d", i,
              ct_coef(i));
}

/*
 * A knob out of range clamps the gain, never a coefficient.
 *
 * The distinction is the whole reason i6c_ct_gain_cap exists. A row is a
 * direction: clamping its entries one at a time keeps every number inside the
 * field while tilting the row, which on a chroma row is a hue shift and on the
 * luma row a colour cast -- so the picture would go wrong in a way that looks
 * like a miscalibration rather than like a knob at its limit.
 *
 * Seeded with a chroma row a real tuning would not carry, because a tuning that
 * fits comfortably cannot exercise this at all. Limited swing on the luma side,
 * so the narrowing in i6c_ct_capture stays out of it and the only thing moving
 * here is the clamp.
 */
static void test_a_gain_is_clamped_before_a_coefficient_is(void)
{
    const int16_t wide[I6C_ISP_IQ_COLORTRANS_MAT_NUM] = {66,  129, 25,  -400, -200,
                                                         100, 112, -94, -18};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    rss_isp_knob_t caps;
    void *c = &ctx;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_ctrans(wide, CT_LIM_OFST);
    log_reset();

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_MAX) == RSS_OK, "the value is still accepted");
    CHECK(strstr(g_log, "limit saturation") != NULL,
          "and the tuning's own ceiling is said once, when it is learned");

    for (i = I6C_CT_ROW_U; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) >= I6C_CT_MAT_MIN && ct_coef(i) <= I6C_CT_MAT_MAX,
              "every coefficient stays in the field, %d gave %d", i, ct_coef(i));

    /*
     * Proportional, which is the claim: the ratio the tuner chose between the
     * row's entries survives the clamp. Checked against the largest entry so
     * that rounding is the only source of error.
     */
    for (i = I6C_CT_ROW_U; i < I6C_CT_ROW_U + I6C_CT_ROW_LEN; i++) {
        int32_t want = (int32_t)wide[i] * ct_coef(I6C_CT_ROW_U) / wide[I6C_CT_ROW_U];
        int32_t got = ct_coef(i);

        CHECK(got >= want - 1 && got <= want + 1,
              "the row keeps its shape through the clamp, %d wanted about %d and gave %d", i, want,
              got);
    }

    /* The published range does not shrink with the tuning -- a client draws the
     * same control everywhere -- so the ceiling is a log line and not a cap. */
    CHECK(hal_isp_get_knob_caps(c, "saturation", &caps) == RSS_OK, "saturation publishes caps");
    CHECK(caps.max == I6C_CT_KNOB_MAX, "and the whole range, got %d", caps.max);
}

/*
 * A full-range tuning is composed over the range the ISP was actually putting
 * out, not over the matrix the block holds.
 *
 * The two are not the same on this family, and the difference is not cosmetic:
 * switching the module on with the tuning's own full-range matrix moves the
 * stream from limited to full range with nothing telling a decoder the range
 * changed. Measured on an SSC377QE + IMX335 across three interleaved pairs of
 * the same build with and without the enable -- 13% more chroma, 4% less luma,
 * and the limited-range prediction from the enabled picture landing within 0.07
 * of a luma count of the disabled one.
 *
 * So neutral has to reproduce the disabled rendering, which means narrowing the
 * baseline rather than taking the block at its word.
 */
static void test_a_full_range_tuning_is_narrowed_to_what_the_isp_was_doing(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_ctrans(ct_full, 0);
    log_reset();

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_UNITY) == RSS_OK, "the middle is legal");
    CHECK(strstr(g_log, "full range") != NULL, "and the narrowing is said once, when it happens");

    for (i = 0; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) == ct_lim[i], "coefficient %d should narrow to %d, got %d", i, ct_lim[i],
              ct_coef(i));
    CHECK(ct_ofst(0) == CT_LIM_OFST, "and gain the 16-count pedestal, got %d", ct_ofst(0));

    /*
     * A tuning that already stores limited range is left exactly as it is --
     * narrowing twice would be the same defect in the other direction.
     */
    reset(&st);
    st.isp_knobs_live = true;
    seed_ctrans(ct_lim, CT_LIM_OFST);
    log_reset();

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_UNITY) == RSS_OK, "the middle is still legal");
    CHECK(strstr(g_log, "full range") == NULL, "and this one is not narrowed");
    for (i = 0; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) == ct_lim[i], "coefficient %d must be untouched, got %d", i, ct_coef(i));
    CHECK(ct_ofst(0) == CT_LIM_OFST, "and so must the offset, got %d", ct_ofst(0));
}

/*
 * The vendor's own block: narrowed rows with no pedestal beside them.
 *
 * This is what SigmaStar's IMX335 tuning and OpenIPC's both ship, and it is why
 * the range and the pedestal are corrected separately rather than as one step.
 * A rule that only fired on a full-range matrix would leave this one 16 counts
 * of luma below the picture the module was not in -- and a rule that only
 * looked at the offset would narrow it a second time.
 */
static void test_the_vendor_block_gains_the_pedestal_and_nothing_else(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_ctrans(ct_lim, 0);
    log_reset();

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_UNITY) == RSS_OK, "the middle is legal");
    CHECK(strstr(g_log, "full range") == NULL, "rows already limited, so no narrowing");
    CHECK(strstr(g_log, "no luma pedestal") != NULL, "but the missing pedestal is said");

    for (i = 0; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) == ct_lim[i], "coefficient %d must be untouched, got %d", i, ct_coef(i));
    CHECK(ct_ofst(0) == CT_LIM_OFST, "and the pedestal supplied, got %d", ct_ofst(0));

    /*
     * The full-range block and the vendor's must land in the same place, since
     * they are two spellings of one conversion. That is the whole claim.
     */
    reset(&st);
    st.isp_knobs_live = true;
    seed_ctrans(ct_full, 0);
    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_UNITY) == RSS_OK, "and from the other end");
    for (i = 0; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) == ct_lim[i], "coefficient %d should agree, got %d", i, ct_coef(i));
    CHECK(ct_ofst(0) == CT_LIM_OFST, "and so should the offset, got %d", ct_ofst(0));

    /*
     * The fourth corner, and the one that says the two corrections really are
     * independent: full-range rows with a pedestal already beside them. The
     * rows still have to narrow and the pedestal must not be added twice, which
     * a single coupled condition cannot express -- it would decline to narrow
     * on account of an offset that has nothing to do with the swing.
     */
    reset(&st);
    st.isp_knobs_live = true;
    seed_ctrans(ct_full, CT_LIM_OFST);
    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_UNITY) == RSS_OK, "full range with a pedestal");
    for (i = 0; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++)
        CHECK(ct_coef(i) == ct_lim[i], "coefficient %d must still narrow, got %d", i, ct_coef(i));
    CHECK(ct_ofst(0) == CT_LIM_OFST, "and the pedestal must not double, got %d", ct_ofst(0));
}

/*
 * Setting a knob twice is the same as setting it once.
 *
 * The composer scales from the captured baseline and never from what is in the
 * field, which is what makes this true. Scaling from the field would compound,
 * so a slider dragged across its range would end up somewhere a slider dropped
 * on the same value could not reach, with no way back short of a tuning reload
 * -- the same trap i6c_iq_param_t::base[] exists to avoid for a vector row.
 */
static void test_the_transform_does_not_compound(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    uint8_t once[I6C_ISP_IQ_COLORTRANS_PAYLOAD];
    void *c = &ctx;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;

    CHECK(hal_isp_set_saturation(c, 80) == RSS_OK, "set it once");
    memcpy(once, g_ctrans, sizeof(once));

    CHECK(hal_isp_set_saturation(c, 80) == RSS_OK, "and again");
    CHECK(hal_isp_set_saturation(c, 80) == RSS_OK, "and again");
    CHECK(memcmp(once, g_ctrans, sizeof(once)) == 0, "the same value must give the same matrix");

    /* And the other two knobs are re-derived from the baseline each time rather
     * than carried forward from whatever the last write left. */
    CHECK(hal_isp_set_contrast(c, 70) == RSS_OK, "a second knob moves");
    CHECK(hal_isp_set_contrast(c, I6C_CT_KNOB_UNITY) == RSS_OK, "and comes back");
    CHECK(memcmp(once, g_ctrans, sizeof(once)) == 0,
          "and leaves the first knob exactly where it was");
}

/*
 * A tuning load re-reads the transform, and the knobs are composed over the new
 * one.
 *
 * The tuning binary writes over the API store on the first frame, so everything
 * the knobs are expressed against has just been replaced. A baseline kept from
 * before that would scale the old tuner's matrix into the new tuner's module.
 */
static void test_a_tuning_load_relearns_the_transform(void)
{
    const int16_t other[I6C_ISP_IQ_COLORTRANS_MAT_NUM] = {69,  120, 31,  -35, -77,
                                                          112, 112, -88, -24};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_UNITY / 2) == RSS_OK, "a knob is set");

    /* What a reload looks like from here: the module holds the tuner's values
     * again, and the flush is what notices. */
    seed_ctrans(other, CT_LIM_OFST);
    i6c_isp_flush_knobs(&st);

    for (i = I6C_CT_ROW_U; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++) {
        int32_t want = i6c_ct_scale(other[i], I6C_CT_KNOB_UNITY / 2);

        CHECK(ct_coef(i) == want, "chroma %d should scale the new tuning to %d, got %d", i, want,
              ct_coef(i));
    }
    for (i = 0; i < I6C_CT_ROW_LEN; i++)
        CHECK(ct_coef(i) == other[i], "and the new luma row is left alone, %d gave %d", i,
              ct_coef(i));
}

/*
 * A knob asked for before the tuning has loaded is composed by the flush.
 *
 * rvd applies its whole [image] block during pipeline construction, which is
 * well before any frame, so without this every one of those values would be
 * written into a store the tuning binary is about to overwrite.
 */
static void test_a_composed_knob_queued_before_the_load_is_flushed(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;
    int i;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    st.isp_knobs_live = false;

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_MAX) == RSS_OK, "the value is taken");
    CHECK(g_ctrans_sets == 0, "but nothing is written yet");
    CHECK(i6c_iq_read(g_ctrans, I6C_ISP_ENABLE_OFF, 4) == 0, "not even the enable");

    i6c_isp_flush_knobs(&st);

    CHECK(g_ctrans_sets == 1, "the flush writes it once, got %d", g_ctrans_sets);
    CHECK(i6c_iq_read(g_ctrans, I6C_ISP_ENABLE_OFF, 4) == 1, "and switches the module on");
    for (i = I6C_CT_ROW_U; i < I6C_ISP_IQ_COLORTRANS_MAT_NUM; i++) {
        int32_t want = i6c_ct_scale(ct_lim[i], I6C_CT_KNOB_MAX);

        CHECK(ct_coef(i) == want, "with the queued value, %d should be %d and gave %d", i, want,
              ct_coef(i));
    }
}

/*
 * A module the tuning never populated is refused rather than enabled.
 *
 * An empty transform reads back as zeros, and switching it on then would put
 * out a black picture -- the one failure here bad enough to be worth declining
 * outright rather than clamping into something. The caps say so too, so a
 * client can grey the control instead of offering three knobs that do nothing.
 */
static void test_an_empty_transform_is_refused(void)
{
    const int16_t empty[I6C_ISP_IQ_COLORTRANS_MAT_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    rss_isp_knob_t caps;
    void *c = &ctx;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_ctrans(empty, 0);
    log_reset();

    CHECK(hal_isp_set_saturation(c, 80) == RSS_ERR_NOTSUP, "the write is declined");
    CHECK(i6c_iq_read(g_ctrans, I6C_ISP_ENABLE_OFF, 4) == 0,
          "and the module is not switched on over an empty matrix");
    CHECK(strstr(g_log, "no conversion") != NULL, "and it says why");

    CHECK(hal_isp_get_knob_caps(c, "saturation", &caps) == RSS_OK, "the caps still answer");
    CHECK(!caps.enabled, "but report the knob unavailable");
}

/*
 * The preset selector taking the matrix out of the path is reported.
 *
 * ColorTransEX chooses a fixed conversion, so with it enabled every write below
 * is correct, reads back as asked for, and changes nothing on screen. There is
 * nothing in the picture to tell that apart from a knob that works, which is
 * what makes it worth a line in the log rather than silence.
 */
static void test_the_preset_override_is_reported(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    void *c = &ctx;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    i6c_iq_write(g_ctex, I6C_ISP_ENABLE_OFF, 4, 1);
    i6c_iq_write(g_ctex, I6C_ISP_IQ_COLORTRANSEX_TYPE, 1, 2);
    log_reset();

    CHECK(hal_isp_set_saturation(c, 80) == RSS_OK, "the write still goes in");
    CHECK(strstr(g_log, "overrides the custom matrix") != NULL,
          "but the override has to be said, since the picture will not say it");
    CHECK(i6c_iq_read(g_ctex, I6C_ISP_ENABLE_OFF, 4) == 1,
          "and it is reported, not corrected -- the conversion is the tuning's choice");
}

/*
 * Saturation is published on this family now, on the same scale as the two it
 * shares a module with.
 *
 * It was absent while MI's own Saturation module was the only way to reach it,
 * because every shipped tuning varies it across gain and manual mode would have
 * traded that curve for a constant. Composed into the transform it costs that
 * curve nothing, so what changed is the mechanism and not the judgement.
 */
static void test_saturation_is_published_with_the_other_two(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    rss_isp_knob_t caps;
    void *c = &ctx;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;

    CHECK(hal_isp_get_knob_caps(c, "saturation", &caps) == RSS_OK, "saturation has caps");
    CHECK(caps.min == 0 && caps.max == I6C_CT_KNOB_MAX && caps.neutral == I6C_CT_KNOB_UNITY,
          "on the same scale as brightness and contrast, got %d..%d neutral %d", caps.min, caps.max,
          caps.neutral);
    CHECK(caps.has_auto, "and auto means the tuning's own rendering");

    CHECK(hal_isp_set_saturation(c, I6C_CT_KNOB_MAX + 1) == RSS_ERR_INVAL,
          "a value past the published maximum is refused");
    CHECK(hal_isp_set_saturation(c, -1) == RSS_ERR_INVAL, "and so is one below the floor");
    CHECK(hal_isp_set_saturation(NULL, 50) == RSS_ERR_INVAL, "NULL ctx rejected");
    CHECK(hal_isp_get_saturation(NULL, NULL) == RSS_ERR_INVAL, "and on the way back");
}

/*
 * The neutral a client centres its control on, which is not the same question
 * as the range.
 *
 * Two shapes of knob live in this table. Brightness and contrast are two-sided
 * -- MI's u32Lev runs 0..100 and 50 really is the value that changes nothing --
 * so their neutral is the midpoint. DRC and defog are one-sided effect
 * strengths: 0 is no contribution and there is nothing beneath it, so the
 * midpoint is half on rather than off.
 *
 * Both carried 128 until this test existed, which was the centre of the
 * abstract 0..255 scale every knob was published on before ec571b7. The scale
 * went; the number stayed. A console centring a slider on the published neutral
 * therefore opened DRC at half strength, and reported a defog module the tuning
 * had switched off as sitting at 128 -- see rvd_ctrl.c, which substitutes the
 * neutral as the reported value for any knob in auto.
 */
static void test_a_one_sided_strength_is_neutral_at_its_floor(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    rss_isp_knob_t caps;
    void *c = &ctx;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;

    CHECK(hal_isp_get_knob_caps(c, "drc_strength", &caps) == RSS_OK, "drc_strength has caps");
    CHECK(caps.min == 0 && caps.max == 255, "drc spans WDR's own u8Strength, got %d..%d",
          caps.min, caps.max);
    CHECK(caps.neutral == 0, "drc's neutral is off, not the midpoint, got %d", caps.neutral);

    CHECK(hal_isp_get_knob_caps(c, "defog_strength", &caps) == RSS_OK, "defog_strength has caps");
    CHECK(caps.neutral == 0, "defog's neutral is off, not the midpoint, got %d", caps.neutral);

    /* And the two-sided knobs keep theirs, so this is a distinction rather
     * than a rule that every neutral is the floor. */
    CHECK(hal_isp_get_knob_caps(c, "brightness", &caps) == RSS_OK, "brightness has caps");
    CHECK(caps.neutral == 50 && caps.min == 0 && caps.max == 100,
          "brightness stays 0..100 neutral 50, got %d..%d neutral %d", caps.min, caps.max,
          caps.neutral);
    CHECK(hal_isp_get_knob_caps(c, "contrast", &caps) == RSS_OK, "contrast has caps");
    CHECK(caps.neutral == 50, "contrast stays neutral 50, got %d", caps.neutral);
}

int main(void)
{
    test_reporting_does_not_cache_the_sensors_answer();
    test_reporting_falls_back_to_the_configured_rate();

    test_a_vector_knob_scales_every_field_about_its_own_baseline();
    test_neutral_returns_a_vector_row_to_auto_untouched();
    test_a_vector_knob_does_not_compound();
    test_a_tuning_load_relearns_the_baseline();
    test_an_out_of_range_run_is_refused_whole();
    test_a_zero_run_is_a_baseline_not_a_failure();
    test_a_baseline_on_the_ceiling_still_scales_down();
    test_a_vector_write_touches_only_the_run();
    test_a_vector_row_reads_back();
    test_the_reported_field_avoids_a_baseline_on_a_bound();
    test_a_refused_baseline_resets_the_reported_field();
    test_a_run_entirely_on_a_bound_reports_the_field();
    test_a_vector_knob_queued_before_the_load_is_flushed();
    test_a_run_that_does_not_fit_is_refused();
    test_a_failing_module_writes_nothing();
    test_the_remap_is_the_identity_at_the_pivot();
    test_drc_carries_the_majestic_scale_onto_wdr();
    test_a_value_switches_on_a_module_the_tuning_disabled();
    test_auto_hands_the_tuners_switch_back();
    test_a_module_the_tuning_enabled_is_left_enabled();
    test_a_tuning_load_relearns_the_switch();
    test_a_vector_value_switches_its_module_on_too();
    test_defog_strength_leaves_defogs_own_switch_alone();
    test_a_drc_write_touches_only_the_level();
    test_the_awb_line_survives_ae_winning_the_race();
    test_sinter_is_not_published();
    test_temper_writes_every_gain_entry_and_stays_in_auto();
    test_temper_leaves_the_rest_of_the_gain_curve_alone();
    test_temper_auto_puts_the_tunings_entries_back();
    test_temper_caps_come_from_the_tuning();
    test_temper_says_so_when_the_tuning_is_in_manual();
    test_a_one_sided_strength_is_neutral_at_its_floor();

    test_neutral_reproduces_the_tunings_own_transform();
    test_contrast_pivots_on_mid_grey();
    test_saturation_scales_only_the_chroma_rows();
    test_brightness_moves_the_offset_and_nothing_else();
    test_a_gain_is_clamped_before_a_coefficient_is();
    test_a_full_range_tuning_is_narrowed_to_what_the_isp_was_doing();
    test_the_vendor_block_gains_the_pedestal_and_nothing_else();
    test_the_transform_does_not_compound();
    test_a_tuning_load_relearns_the_transform();
    test_a_composed_knob_queued_before_the_load_is_flushed();
    test_an_empty_transform_is_refused();
    test_the_preset_override_is_reported();
    test_saturation_is_published_with_the_other_two();

    if (failures) {
        printf("t_isp_i6c: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_isp_i6c: ok\n");
    return 0;
}
