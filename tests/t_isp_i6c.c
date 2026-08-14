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
#include <stdio.h>

/*
 * A real (silent) logger rather than NULL: HAL_LOG_* call through this without a
 * NULL guard, because rss_hal_init always installs one, and every test here
 * drives at least one warning path.
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

/* The AE envelope the stubs hand back, and what the last Set wrote. */
static i6c_isp_exp g_limit;
static i6c_isp_exp g_limit_set;
static int g_limit_sets;
static int g_get_ret;
static int g_set_ret;

/* What MI_SNR_GetFps answers, and whether it was asked at all. */
static unsigned int g_fps_reported;
static int g_fps_gets;
static int g_fps_get_ret;

static int fake_get_expo_limit(unsigned int device, unsigned int channel, i6c_isp_exp *limit)
{
    (void)device;
    (void)channel;
    if (g_get_ret)
        return g_get_ret;
    *limit = g_limit;
    return 0;
}

static int fake_set_expo_limit(unsigned int device, unsigned int channel, i6c_isp_exp *limit)
{
    (void)device;
    (void)channel;
    g_limit_sets++;
    g_limit_set = *limit;
    if (g_set_ret)
        return g_set_ret;
    /* MI keeps its own value when a bound is outside the calibrated range; the
     * readback is the only thing that says which happened, so the stub has to
     * model the accepting case explicitly. */
    g_limit = *limit;
    return 0;
}

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
#define NR_RUN (I6C_ISP_IQ_NRLUMAADV_MANUAL + I6C_ISP_IQ_NRLUMAADV_STRENGTH)

static uint8_t g_sharp[I6C_ISP_IQ_SHARPNESS_PAYLOAD];
static uint8_t g_nr[I6C_ISP_IQ_NRLUMAADV_PAYLOAD];
static int g_sharp_sets, g_nr_sets;
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

static int fake_get_nr(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    memcpy(payload, g_nr, sizeof(g_nr));
    return 0;
}

static int fake_set_nr(unsigned int device, unsigned int channel, void *payload)
{
    (void)device;
    (void)channel;
    g_nr_sets++;
    memcpy(g_nr, payload, sizeof(g_nr));
    return 0;
}

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
    /* As i6c_isp_flush_knobs leaves it: a baseline is owed, and the first fetch
     * is what pays it. */
    p->unity_stale = true;
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

static void reset(infinity6c_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->isp.get_expo_limit = fake_get_expo_limit;
    st->isp.set_expo_limit = fake_set_expo_limit;
    st->snr.get_fps = fake_get_fps;

    reset_row(IQ_SHARPNESS, fake_get_sharp, fake_set_sharp);
    reset_row(IQ_NRLUMAADV, fake_get_nr, fake_set_nr);
    memset(g_sharp, 0, sizeof(g_sharp));
    memset(g_nr, 0, sizeof(g_nr));
    g_sharp_sets = g_nr_sets = 0;
    g_sharp_get_ret = g_sharp_set_ret = 0;

    /* The tuning binary's own envelope on this board: a 100 ms ceiling, which
     * at 8.08 us a line is VMAX 12376 and about 8 fps. */
    memset(&g_limit, 0, sizeof(g_limit));
    g_limit.minShutterUs = 30;
    g_limit.maxShutterUs = 100000;
    memset(&g_limit_set, 0, sizeof(g_limit_set));
    g_limit_sets = 0;
    g_get_ret = g_set_ret = 0;
    g_fps_reported = 0;
    g_fps_gets = 0;
    g_fps_get_ret = 0;
}

/*
 * The regression this file exists for.
 *
 * The cap must derive the ceiling from the rate the camera is configured to
 * hold, never from the rate the sensor reports, because the sensor's answer
 * comes from the frame length and the frame length is what the AE stretches to
 * buy the very exposure being capped.
 *
 * Observed on an SSC377QE + IMX335, one binary and one config, two restarts:
 * lit scene, the sensor answered 24 and the ceiling came down; dark scene, where
 * the AE had already stretched VMAX towards 100 ms, it answered 9, so 100 ms
 * looked comfortably inside a 111 ms frame and the ceiling stayed up. And it
 * stays up from then on -- the stretch that caused the misread is what the
 * ceiling would have removed.
 */
static void test_the_cap_holds_the_configured_rate_not_the_delivered_one(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 25;
    st.pipeline_up = true;
    st.isp_knobs_live = true;

    /* The AE has stretched the frame out to 9 fps, which is exactly the state
     * the ceiling is meant to prevent. */
    g_fps_reported = 9;

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_sets == 1, "the ceiling must be written once, got %d", g_limit_sets);
    CHECK(g_limit_set.maxShutterUs == 40000, "one frame at 25 fps is 40000 us, got %u",
          g_limit_set.maxShutterUs);
    CHECK(g_fps_gets == 0, "the sensor's rate must not be consulted at all, asked %d times",
          g_fps_gets);
}

/*
 * The trap the cap fell into on its first outing, kept as a test because the
 * obvious simplification -- read st->fps and be done -- reintroduces it. rvd
 * sets the rate while it is still building the pipeline, which on this backend
 * runs before the sensor is enabled, so the request parks in snr_fps_req and
 * st->fps is still 0 when the first frame arrives and the cap runs.
 */
static void test_a_rate_only_requested_still_caps(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 0;
    st.snr_fps_req = 25;
    st.isp_knobs_live = true;

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_sets == 1, "a requested-but-unprogrammed rate must still cap, got %d writes",
          g_limit_sets);
    CHECK(g_limit_set.maxShutterUs == 40000, "one frame at 25 fps is 40000 us, got %u",
          g_limit_set.maxShutterUs);
}

/* The programmed rate outranks the request: it is the one in force. */
static void test_the_programmed_rate_outranks_the_request(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 20;
    st.snr_fps_req = 25;
    st.isp_knobs_live = true;

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_set.maxShutterUs == 50000,
          "one frame at the programmed 20 fps is 50000 us, "
          "got %u",
          g_limit_set.maxShutterUs);
}

/* Nothing to derive a ceiling from is a warning and no write, not a guess. */
static void test_no_rate_leaves_the_tuning_alone(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.isp_knobs_live = true;

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_sets == 0, "an unknown rate must not write a ceiling, got %d writes",
          g_limit_sets);
    CHECK(g_limit.maxShutterUs == 100000, "the tuning's ceiling must survive, got %u",
          g_limit.maxShutterUs);
}

/* A tuning already inside one frame is left exactly as it is. */
static void test_a_ceiling_already_within_a_frame_is_untouched(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 25;
    g_limit.maxShutterUs = 20000;

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_sets == 0, "a ceiling inside one frame must not be rewritten, got %d writes",
          g_limit_sets);
    CHECK(g_limit.maxShutterUs == 20000, "the tuning's ceiling must survive, got %u",
          g_limit.maxShutterUs);
}

/*
 * A ceiling below the tuning's own floor is a broken envelope rather than a cap,
 * and MI would take it. At that point the rate asked for is not one this sensor
 * can hold, which is worth saying rather than silently installing.
 */
static void test_a_frame_shorter_than_the_floor_is_refused(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 25;
    g_limit.minShutterUs = 50000; /* a floor longer than a 40 ms frame */

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_sets == 0, "a ceiling under the floor must be refused, got %d writes",
          g_limit_sets);
    CHECK(g_limit.maxShutterUs == 100000, "the tuning's ceiling must survive, got %u",
          g_limit.maxShutterUs);
}

/* A library without the pair costs the cap, not the pipeline. */
static void test_missing_symbols_are_survivable(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 25;
    st.isp.set_expo_limit = NULL;
    i6c_isp_apply_shutter_cap(&st);
    CHECK(g_limit_sets == 0, "no Set symbol must write nothing, got %d", g_limit_sets);

    reset(&st);
    st.fps = 25;
    st.isp.get_expo_limit = NULL;
    i6c_isp_apply_shutter_cap(&st);
    CHECK(g_limit_sets == 0, "no Get symbol must write nothing, got %d", g_limit_sets);
}

/* A failing Get is not a reason to write a ceiling over an envelope we could
 * not read: every other field in the struct would go with it. */
static void test_a_failing_read_writes_nothing(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 25;
    g_get_ret = -1;

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_sets == 0, "a failed read must not be followed by a write, got %d", g_limit_sets);
}

/*
 * The rest of the envelope has to survive the cap. The gains and the aperture
 * are the tuning's, and this call is a read-modify-write of one field -- a
 * memset-and-set would quietly zero the AE's gain ceilings, which is the same
 * shape of bug as writing the whole channel parameter block blind.
 */
static void test_the_cap_moves_only_the_ceiling(void)
{
    infinity6c_state_t st;

    reset(&st);
    st.fps = 25;
    g_limit.minApertX10 = 14;
    g_limit.maxApertX10 = 22;
    g_limit.minSensorGain = 1024;
    g_limit.minIspGain = 1024;
    g_limit.maxSensorGain = 64512;
    g_limit.maxIspGain = 2867;

    i6c_isp_apply_shutter_cap(&st);

    CHECK(g_limit_set.maxShutterUs == 40000, "the ceiling must move, got %u",
          g_limit_set.maxShutterUs);
    CHECK(g_limit_set.minShutterUs == 30, "the floor must not, got %u", g_limit_set.minShutterUs);
    CHECK(g_limit_set.minApertX10 == 14 && g_limit_set.maxApertX10 == 22,
          "the aperture must not, got %u..%u", g_limit_set.minApertX10, g_limit_set.maxApertX10);
    CHECK(g_limit_set.minSensorGain == 1024 && g_limit_set.maxSensorGain == 64512,
          "the sensor gains must not, got %u..%u", g_limit_set.minSensorGain,
          g_limit_set.maxSensorGain);
    CHECK(g_limit_set.minIspGain == 1024 && g_limit_set.maxIspGain == 2867,
          "the ISP gains must not, got %u..%u", g_limit_set.minIspGain, g_limit_set.maxIspGain);
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

    /* And with that reading cached nowhere, the cap still holds 25. */
    i6c_isp_apply_shutter_cap(&st);
    CHECK(g_limit_set.maxShutterUs == 40000, "the cap must still hold 25 fps, got %u",
          g_limit_set.maxShutterUs);
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
    /* 40 + (200-128)*(127-40)/127 and so on, field by field. */
    static const uint16_t up[6] = {89, 84, 80, 97, 93, 91};
    static const uint16_t down[6] = {20, 15, 10, 30, 25, 22};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    unsigned int i;

    arm(&ctx, &st);
    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_OK, "set must succeed");
    for (i = 0; i < 6; i++)
        CHECK(run_at(g_sharp, SHARP_RUN, 1, i) == up[i], "field %u: want %u, got %u", i, up[i],
              run_at(g_sharp, SHARP_RUN, 1, i));

    arm(&ctx, &st);
    CHECK(hal_isp_set_sharpness(&ctx, 64) == RSS_OK, "set must succeed");
    for (i = 0; i < 6; i++)
        CHECK(run_at(g_sharp, SHARP_RUN, 1, i) == down[i], "field %u: want %u, got %u", i, down[i],
              run_at(g_sharp, SHARP_RUN, 1, i));

    /* The ends are MI's, not the tuning's: 255 is every field at the ceiling
     * and 0 is the module contributing nothing. */
    arm(&ctx, &st);
    CHECK(hal_isp_set_sharpness(&ctx, 255) == RSS_OK, "set must succeed");
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

    CHECK(hal_isp_set_sharpness(&ctx, 128) == RSS_OK, "set must succeed");
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
    hal_isp_set_sharpness(&ctx, 200);
    hal_isp_set_sharpness(&ctx, 200);
    hal_isp_set_sharpness(&ctx, 160);
    memcpy(twice, g_sharp, sizeof(twice));

    /* The same knob reached in one step from the pristine tuning. */
    arm(&ctx, &st);
    hal_isp_set_sharpness(&ctx, 160);

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
    hal_isp_set_sharpness(&ctx, 200);

    /* A second load: the module reads back as the new binary wrote it. */
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, retuned, 6, I6C_ISP_OP_AUTO);
    i6c_isp_flush_knobs(&st);

    CHECK(g_iq[IQ_SHARPNESS].base_valid, "the new run must be adopted");
    for (i = 0; i < 6; i++)
        CHECK(g_iq[IQ_SHARPNESS].base[i] == retuned[i],
              "baseline %u must come from the new tuning: want %u, got %u", i, retuned[i],
              g_iq[IQ_SHARPNESS].base[i]);

    /* And the pending 200 was re-applied against it, not against the old one. */
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 80 + 72 * (127 - 80) / 127,
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

    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_OK, "the knob still works");
    CHECK(!g_iq[IQ_SHARPNESS].base_valid, "a bad run must not be adopted");
    /* Every field falls back to the row's constant neutral, 63. */
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 63 + 72 * (127 - 63) / 127,
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

    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_OK, "set must succeed");
    CHECK(g_iq[IQ_SHARPNESS].base_valid, "zero is in range and must be adopted");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 72, "must scale up from nothing, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, off, 6, I6C_ISP_OP_AUTO);
    CHECK(hal_isp_set_sharpness(&ctx, 64) == RSS_OK, "set must succeed");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 0, "below a zero baseline is still zero, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));
}

/*
 * The denoise row, which is the sixteen-bit one. Its ceiling is 256 and not
 * 255: MI's unity for a blend weight is a full swap, so the bound is one past
 * what a byte holds -- and a row that wrote it a byte at a time would deliver 0
 * at maximum strength.
 */
static void test_the_denoise_run_is_sixteen_bit_and_reaches_256(void)
{
    static const uint16_t tuned[2] = {200, 180};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_run(g_nr, sizeof(g_nr), NR_RUN, 2, tuned, 2, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sinter_strength(&ctx, 255) == RSS_OK, "set must succeed");
    CHECK(run_at(g_nr, NR_RUN, 2, 0) == 256, "field 0 must reach 256, got %u",
          run_at(g_nr, NR_RUN, 2, 0));
    CHECK(run_at(g_nr, NR_RUN, 2, 1) == 256, "field 1 must reach 256, got %u",
          run_at(g_nr, NR_RUN, 2, 1));

    seed_run(g_nr, sizeof(g_nr), NR_RUN, 2, tuned, 2, I6C_ISP_OP_AUTO);
    reset_row(IQ_NRLUMAADV, fake_get_nr, fake_set_nr);
    CHECK(hal_isp_set_sinter_strength(&ctx, 64) == RSS_OK, "set must succeed");
    CHECK(run_at(g_nr, NR_RUN, 2, 0) == 100, "want 100, got %u", run_at(g_nr, NR_RUN, 2, 0));
    CHECK(run_at(g_nr, NR_RUN, 2, 1) == 90, "want 90, got %u", run_at(g_nr, NR_RUN, 2, 1));
}

/*
 * A tuning is entitled to ask for the maximum, and a baseline sitting on the
 * ceiling used to make the whole knob inert -- i6c_iq_scale bailed out rather
 * than divide a zero span, taking the lower half with it. Full-strength denoise
 * is exactly that case, since MI's unity for the field is its maximum.
 */
static void test_a_baseline_on_the_ceiling_still_scales_down(void)
{
    static const uint16_t maxed[2] = {256, 256};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    uint8_t val;

    reset(&st);
    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    st.isp_knobs_live = true;
    seed_run(g_nr, sizeof(g_nr), NR_RUN, 2, maxed, 2, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sinter_strength(&ctx, 64) == RSS_OK, "set must succeed");
    CHECK(run_at(g_nr, NR_RUN, 2, 0) == 128, "the lower half must still move, got %u",
          run_at(g_nr, NR_RUN, 2, 0));

    /* And upward there is simply no headroom, which is not the same as broken. */
    seed_run(g_nr, sizeof(g_nr), NR_RUN, 2, maxed, 2, I6C_ISP_OP_AUTO);
    reset_row(IQ_NRLUMAADV, fake_get_nr, fake_set_nr);
    CHECK(hal_isp_set_sinter_strength(&ctx, 200) == RSS_OK, "set must succeed");
    CHECK(run_at(g_nr, NR_RUN, 2, 0) == 256, "above neutral must stay at the ceiling, got %u",
          run_at(g_nr, NR_RUN, 2, 0));

    /*
     * Reading it back is the other half, and the ambiguous one: the field holds
     * MI's maximum and so does the baseline, so it is both "at the ceiling" and
     * "where the tuning left it". Neutral is the answer, because that is what
     * the knob was left at -- and the ordering inside i6c_iq_unscale is the
     * whole of the difference, since a ceiling test placed first reports 255 for
     * a knob nobody has touched.
     */
    val = 0;
    CHECK(hal_isp_get_sinter_strength(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 128, "a baseline on the ceiling reads as neutral, got %u", val);
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

    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_OK, "set must succeed");

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
    uint8_t val = 0;

    arm(&ctx, &st);
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 128, "an auto module reads as neutral, got %u", val);

    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_OK, "set must succeed");
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    /*
     * 198 and not 200. The row reports from field 3, whose baseline of 60 is
     * the furthest from both bounds of the six; it holds 97, and 128 +
     * 37 * 127 / 67 truncates twice on the way there. One step of raptor's 255
     * is finer than one step of MI's field, so a round trip is within a step or
     * two by construction -- which is worth pinning exactly, because "close
     * enough" is how an off-by-one in the scaling would hide.
     */
    CHECK(g_iq[IQ_SHARPNESS].report == 3, "field 3 is the best conditioned, picked %u",
          g_iq[IQ_SHARPNESS].report);
    CHECK(val == 198, "want the value back within a step, got %u", val);

    /*
     * Back to neutral, which is where the mode matters and nothing else does.
     * The module goes to auto and its manual run keeps the 89 the last set left
     * there -- so a reader that unscaled the run regardless of the mode would
     * answer 199 for a knob that is once again the tuning's. Only the check on
     * enOpType separates them, and only for vector rows does it have to reach
     * past IQ_AUTOMAN to do it.
     */
    CHECK(hal_isp_set_sharpness(&ctx, 128) == RSS_OK, "set must succeed");
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 89, "neutral leaves the run where it was, got %u",
          run_at(g_sharp, SHARP_RUN, 1, 0));
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 128, "a module handed back to the tuning reads as neutral, got %u", val);

    /* Before the tuning has loaded there is nothing to ask, and the queue is a
     * better answer than the hardware's. */
    arm(&ctx, &st);
    st.isp_knobs_live = false;
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_ERR_BUSY, "nothing set yet is BUSY");
    CHECK(hal_isp_set_sharpness(&ctx, 77) == RSS_OK, "set must be queued");
    CHECK(g_sharp_sets == 0, "a queued knob must not reach MI, reached it %d times", g_sharp_sets);
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must answer from the queue");
    CHECK(val == 77, "the queued value is the answer, got %u", val);
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
    uint8_t val = 0;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, tuned, 6, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sharpness(&ctx, 64) == RSS_OK, "set must succeed");
    /* min(base, 127 - base) is largest at 60, which is field 3. */
    CHECK(g_iq[IQ_SHARPNESS].report == 3,
          "must report from the field furthest from both bounds, "
          "picked %u",
          g_iq[IQ_SHARPNESS].report);
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 64, "the lower half must read back, got %u", val);

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
    uint8_t val = 0;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, good, 6, I6C_ISP_OP_AUTO);
    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_OK, "set must succeed");
    CHECK(g_iq[IQ_SHARPNESS].report == 3, "the good run picks field 3, picked %u",
          g_iq[IQ_SHARPNESS].report);

    /* A reload whose run this port cannot read: one field out of range. */
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, bad, 6, I6C_ISP_OP_MANUAL);
    g_iq[IQ_SHARPNESS].unity_stale = true;

    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(!g_iq[IQ_SHARPNESS].base_valid, "the bad run must be refused");
    CHECK(g_iq[IQ_SHARPNESS].report == 0, "and the reported field must go back to 0, is %u",
          g_iq[IQ_SHARPNESS].report);
    /* Field 0 holds 10 against the fallback neutral of 63: 10 * 128 / 63. */
    CHECK(val == 20, "must read field 0 against the fallback neutral, got %u", val);
}

/* A run that is entirely on a bound has no field that can answer, and neutral
 * is the honest reading: a tuning that turned the module off has made "off" and
 * "as the tuning left it" the same picture. */
static void test_a_run_entirely_on_a_bound_reports_neutral(void)
{
    static const uint16_t off[6] = {0, 0, 0, 0, 0, 0};
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;
    uint8_t val = 0;

    arm(&ctx, &st);
    seed_run(g_sharp, sizeof(g_sharp), SHARP_RUN, 1, off, 6, I6C_ISP_OP_AUTO);

    CHECK(hal_isp_set_sharpness(&ctx, 64) == RSS_OK, "set must succeed");
    CHECK(g_iq[IQ_SHARPNESS].report == 0, "with nothing to choose between, field 0");
    CHECK(hal_isp_get_sharpness(&ctx, &val) == RSS_OK, "get must succeed");
    CHECK(val == 128, "an off module reads as neutral, got %u", val);
}

/* A queued knob is applied by the flush, and against the baseline the load that
 * triggered the flush left behind. */
static void test_a_vector_knob_queued_before_the_load_is_flushed(void)
{
    rss_hal_ctx_t ctx;
    infinity6c_state_t st;

    arm(&ctx, &st);
    st.isp_knobs_live = false;

    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_OK, "set must be queued");
    CHECK(g_sharp_sets == 0, "nothing may reach MI yet");

    i6c_isp_flush_knobs(&st);
    CHECK(g_sharp_sets == 1, "the flush must apply it once, applied %d", g_sharp_sets);
    CHECK(run_at(g_sharp, SHARP_RUN, 1, 0) == 89, "want 89, got %u",
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
    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_ERR_INVAL, "a run longer than base[] is refused");

    p->count = saved_count;
    p->manual_off = (uint16_t)(p->payload - 2);
    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_ERR_INVAL,
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
    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_ERR_IO, "a failing read is IO");
    CHECK(g_sharp_sets == 0, "nothing may be written, wrote %d", g_sharp_sets);
    CHECK(!g_iq[IQ_SHARPNESS].base_valid, "and no baseline may be adopted");

    arm(&ctx, &st);
    g_sharp_set_ret = -1;
    CHECK(hal_isp_set_sharpness(&ctx, 200) == RSS_ERR_IO, "a failing write is IO");
}

/* The scalar rows must be unaffected by all of the above: they share the scale,
 * the fetch and the store. */
static void test_the_scalar_rows_still_map_the_way_they_did(void)
{
    /* saturation: 0..127 with unity 32, which is where a linear map would
     * double colour at neutral. */
    CHECK(i6c_iq_scale(128, 32, 0, 127) == 32, "neutral is unity");
    CHECK(i6c_iq_scale(255, 32, 0, 127) == 127, "255 is the ceiling");
    CHECK(i6c_iq_scale(0, 32, 0, 127) == 0, "0 is the floor");
    CHECK(i6c_iq_scale(64, 32, 0, 127) == 16, "half of neutral is half of unity");
    CHECK(i6c_iq_unscale(32, 32, 0, 127) == 128, "unity reads as neutral");
    CHECK(i6c_iq_unscale(127, 32, 0, 127) == 255, "the ceiling reads as 255");
    CHECK(i6c_iq_unscale(0, 32, 0, 127) == 0, "the floor reads as 0");

    /* ae_comp: signed, and its learned baseline sits on the floor, so MI 0 has
     * to read as neutral rather than as the bottom. */
    CHECK(i6c_iq_scale(128, 0, -20, 20) == 0, "neutral is the tuning's own value");
    CHECK(i6c_iq_scale(0, 0, -20, 20) == -20, "0 is the floor");
    CHECK(i6c_iq_unscale(0, 0, -20, 20) == 128, "a floor-sitting baseline reads as neutral");
}

int main(void)
{
    test_the_cap_holds_the_configured_rate_not_the_delivered_one();
    test_a_rate_only_requested_still_caps();
    test_the_programmed_rate_outranks_the_request();
    test_no_rate_leaves_the_tuning_alone();
    test_a_ceiling_already_within_a_frame_is_untouched();
    test_a_frame_shorter_than_the_floor_is_refused();
    test_missing_symbols_are_survivable();
    test_a_failing_read_writes_nothing();
    test_the_cap_moves_only_the_ceiling();
    test_reporting_does_not_cache_the_sensors_answer();
    test_reporting_falls_back_to_the_configured_rate();

    test_a_vector_knob_scales_every_field_about_its_own_baseline();
    test_neutral_returns_a_vector_row_to_auto_untouched();
    test_a_vector_knob_does_not_compound();
    test_a_tuning_load_relearns_the_baseline();
    test_an_out_of_range_run_is_refused_whole();
    test_a_zero_run_is_a_baseline_not_a_failure();
    test_the_denoise_run_is_sixteen_bit_and_reaches_256();
    test_a_baseline_on_the_ceiling_still_scales_down();
    test_a_vector_write_touches_only_the_run();
    test_a_vector_row_reads_back();
    test_the_reported_field_avoids_a_baseline_on_a_bound();
    test_a_refused_baseline_resets_the_reported_field();
    test_a_run_entirely_on_a_bound_reports_neutral();
    test_a_vector_knob_queued_before_the_load_is_flushed();
    test_a_run_that_does_not_fit_is_refused();
    test_a_failing_module_writes_nothing();
    test_the_scalar_rows_still_map_the_way_they_did();

    if (failures) {
        printf("t_isp_i6c: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_isp_i6c: ok\n");
    return 0;
}
