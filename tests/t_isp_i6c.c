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

static void reset(infinity6c_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->isp.get_expo_limit = fake_get_expo_limit;
    st->isp.set_expo_limit = fake_set_expo_limit;
    st->snr.get_fps = fake_get_fps;

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

    if (failures) {
        printf("t_isp_i6c: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_isp_i6c: ok\n");
    return 0;
}
