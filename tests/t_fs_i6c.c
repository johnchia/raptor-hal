/*
 * Host-side test of sensor mode selection in infinity6c/hal_framesource.c.
 *
 * Same construction as t_isp_i6c.c and t_enc_i6c.c: the real translation unit is
 * included rather than copied, and MI is stubbed through the function pointers in
 * infinity6c_state_t. Only the sensor's are filled in; i6c_snr_select touches
 * nothing else.
 *
 * The stub sensor publishes the list the IMX335 driver for this family does, in
 * its order, because that list is the case the named mode exists for: modes 3 and
 * 6 both cover 1920x1080 at 60, a 4:3 window and a 16:9 one, and the search can
 * only ever return the first.
 */

#define PLATFORM_INFINITY6C 1
#define HAL_MODULE_VIDEO 1

#include "infinity6c/hal_framesource.c"

#include <stdarg.h>
#include <stdio.h>

static char g_log[4096];
static size_t g_log_len;

static void quiet_log(int level, const char *file, int line, const char *fmt, ...)
{
    size_t room = sizeof(g_log) - g_log_len;
    va_list ap;
    int n;

    (void)level;
    (void)file;
    (void)line;

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

rss_hal_log_func_t rss_hal_log_fn = quiet_log;

static int failures;

#define CHECK(cond, fmt, ...)                                                                      \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__);                    \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

/* The ISP side runs once frames flow; nothing here gets that far. */
void i6c_isp_flush_knobs(infinity6c_state_t *st)
{
    (void)st;
}

/* ================================================================
 * THE SENSOR
 * ================================================================ */

static const struct {
    unsigned int w, h, max_fps;
} g_modes[] = {
    {2592, 1944, 25}, {2592, 1944, 40},  {2592, 1944, 59}, {2560, 1920, 60},
    {2208, 1248, 90}, {1920, 1080, 120}, {2560, 1440, 60},
};
#define N_MODES (sizeof(g_modes) / sizeof(g_modes[0]))

static int g_set_res = -1;
static int g_set_fps = -1;

static int snr_query_res_count(unsigned int pad, unsigned int *count)
{
    (void)pad;
    *count = N_MODES;
    return 0;
}

static int snr_get_res(unsigned int pad, unsigned char index, i6c_snr_res *res)
{
    (void)pad;
    if (index >= N_MODES)
        return -1;
    res->crop.width = g_modes[index].w;
    res->crop.height = g_modes[index].h;
    res->maxFps = g_modes[index].max_fps;
    res->minFps = 3;
    return 0;
}

static int snr_set_res(unsigned int pad, unsigned char index)
{
    (void)pad;
    g_set_res = index;
    return 0;
}

static int snr_set_fps(unsigned int pad, unsigned int fps)
{
    (void)pad;
    g_set_fps = (int)fps;
    return 0;
}

static int snr_set_orien(unsigned int pad, unsigned char mirror, unsigned char flip)
{
    (void)pad;
    (void)mirror;
    (void)flip;
    return 0;
}

static int snr_set_plane_mode(unsigned int pad, unsigned char multi)
{
    (void)pad;
    (void)multi;
    return 0;
}

static int snr_get_pad_info(unsigned int pad, i6c_snr_pad *info)
{
    (void)pad;
    memset(info, 0, sizeof(*info));
    return 0;
}

static int snr_get_plane_info(unsigned int pad, unsigned int plane, i6c_snr_plane *info)
{
    (void)pad;
    (void)plane;
    memset(info, 0, sizeof(*info));
    return 0;
}

/* A fresh state per case, asking for the mode given; -1 names none. */
static infinity6c_state_t *state(int mode)
{
    static infinity6c_state_t st;

    memset(&st, 0, sizeof(st));
    st.snr_profile = -1;
    st.snr_mode_req = mode;
    st.snr.query_res_count = snr_query_res_count;
    st.snr.get_res = snr_get_res;
    st.snr.set_res = snr_set_res;
    st.snr.set_fps = snr_set_fps;
    st.snr.set_orien = snr_set_orien;
    st.snr.set_plane_mode = snr_set_plane_mode;
    st.snr.get_pad_info = snr_get_pad_info;
    st.snr.get_plane_info = snr_get_plane_info;

    g_set_res = -1;
    g_set_fps = -1;
    g_log_len = 0;
    g_log[0] = '\0';
    return &st;
}

/* ================================================================
 * CASES
 * ================================================================ */

/* The search is unchanged: with nothing named, 1080p60 takes the 4:3 mode. */
static void test_unnamed_takes_the_first_that_covers(void)
{
    infinity6c_state_t *st = state(-1);

    CHECK(i6c_snr_select(st, 1920, 1080, 60) == RSS_OK, "select failed");
    CHECK(g_set_res == 3, "SetRes(%d), want 3", g_set_res);
    CHECK(g_set_fps == 60, "SetFps(%d), want 60", g_set_fps);
    CHECK(!strstr(g_log, "[sensor] mode"), "unnamed selection mentions the key:\n%s", g_log);
}

/* The reason the key exists: the later of two modes that both cover the stream. */
static void test_named_takes_the_later_equal_mode(void)
{
    infinity6c_state_t *st = state(6);

    CHECK(i6c_snr_select(st, 1920, 1080, 60) == RSS_OK, "select failed");
    CHECK(g_set_res == 6, "SetRes(%d), want 6", g_set_res);
    CHECK(g_set_fps == 60, "SetFps(%d), want 60", g_set_fps);
    CHECK(st->snr_mode_max_fps == 60, "mode ceiling %u, want 60", st->snr_mode_max_fps);
    CHECK(g_log_len == 0 || !strstr(g_log, "instead"), "a clean pick warned:\n%s", g_log);
}

/* Zero is a mode, not "unset": -1 is the only value that means choose. */
static void test_mode_zero_is_a_mode(void)
{
    infinity6c_state_t *st = state(0);

    CHECK(i6c_snr_select(st, 1920, 1080, 60) == RSS_OK, "select failed");
    CHECK(g_set_res == 0, "SetRes(%d), want 0", g_set_res);
}

/* A named mode's rate gives way like a searched one's, and says so. */
static void test_named_rate_is_clamped(void)
{
    infinity6c_state_t *st = state(6);

    CHECK(i6c_snr_select(st, 1920, 1080, 90) == RSS_OK, "select failed");
    CHECK(g_set_res == 6, "SetRes(%d), want 6", g_set_res);
    CHECK(g_set_fps == 60, "SetFps(%d), want the mode's 60", g_set_fps);
    CHECK(st->fps == 60, "st->fps %u, want 60", st->fps);
    CHECK(strstr(g_log, "runs at most 60 fps") != NULL, "no clamp warning:\n%s", g_log);
}

/*
 * A named mode smaller than the stream is not applied, and the search still runs:
 * the stream gets a picture from the mode it would have had.
 */
static void test_named_too_small_falls_back_to_the_search(void)
{
    infinity6c_state_t *st = state(4);

    CHECK(i6c_snr_select(st, 2560, 1440, 60) == RSS_OK, "select failed");
    CHECK(g_set_res == 3, "SetRes(%d), want the search's 3", g_set_res);
    CHECK(strstr(g_log, "smaller than the 2560x1440 stream") != NULL, "no warning:\n%s", g_log);
}

/* One past the end is out of range, and the search runs instead. */
static void test_named_out_of_range_falls_back_to_the_search(void)
{
    infinity6c_state_t *st = state((int)N_MODES);

    CHECK(i6c_snr_select(st, 1920, 1080, 60) == RSS_OK, "select failed");
    CHECK(g_set_res == 3, "SetRes(%d), want the search's 3", g_set_res);
    CHECK(strstr(g_log, "has modes 0-6") != NULL, "no warning:\n%s", g_log);
}

int main(void)
{
    test_unnamed_takes_the_first_that_covers();
    test_named_takes_the_later_equal_mode();
    test_mode_zero_is_a_mode();
    test_named_rate_is_clamped();
    test_named_too_small_falls_back_to_the_search();
    test_named_out_of_range_falls_back_to_the_search();

    if (failures) {
        printf("t_fs_i6c: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_fs_i6c: ok\n");
    return 0;
}
