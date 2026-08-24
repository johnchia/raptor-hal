/*
 * Host-side test of the encoder-statistics guards in star/hal_encoder.c.
 *
 * Same construction as t_isp.c: the real translation unit is included rather
 * than copied, so what is tested is what ships, and MI is stubbed by filling
 * in the function pointers in star_state_t.
 *
 * What this suite is about is one precondition. MI_VENC_Query reads the
 * channel's ring pool, and that pool exists only between MI_VENC_StartRecvPic
 * and MI_VENC_DestroyChn -- so on a created-but-idle channel the driver logs
 * "pstChnRes->hRingPoolHandle == NULL" at kernel error level and fails. rvd's
 * JPEG channel is deliberately on-demand and spends most of its life in
 * exactly that state, so anything walking every stream to report status hit
 * it on every pass. The assertions below are all the same question asked from
 * either op: does an idle channel get asked at all.
 */

#define PLATFORM_INFINITY6E 1
#define HAL_MODULE_VIDEO 1

#include "star/hal_encoder.c"

#include <stdarg.h>
#include <stdio.h>

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

/* ================================================================
 * THE REST OF THE BACKEND
 *
 * hal_encoder.c calls into its neighbours for snapshot ports and for the
 * deferred tuning load. Neither is what this suite is about, and neither is
 * reached by the two ops under test, so all of it succeeds quietly.
 * ================================================================ */

int star_fs_clone_port(star_state_t *st, int src, int dst)
{
    (void)st;
    (void)src;
    (void)dst;
    return RSS_OK;
}

void star_fs_release_port(star_state_t *st, int port)
{
    (void)st;
    (void)port;
}

int star_fs_enable_port(star_state_t *st, int port)
{
    (void)st;
    (void)port;
    return RSS_OK;
}

void star_isp_tune_when_ready(star_state_t *st, bool verbose)
{
    (void)st;
    (void)verbose;
}

void star_isp_note_frame(star_state_t *st)
{
    (void)st;
}

/* ================================================================
 * MI
 * ================================================================ */

static unsigned int g_query_calls;
static int g_query_ret;
static i6_venc_stat g_query_stat;

static int fake_query(int chn, i6_venc_stat *stat)
{
    (void)chn;
    g_query_calls++;
    if (g_query_ret)
        return g_query_ret;
    *stat = g_query_stat;
    return 0;
}

static void setup(rss_hal_ctx_t *ctx, star_state_t *st, bool receiving)
{
    memset(ctx, 0, sizeof(*ctx));
    memset(st, 0, sizeof(*st));
    ctx->platform = st;

    st->venc.fnQuery = fake_query;
    st->enc[0].created = true;
    st->enc[0].receiving = receiving;

    g_query_calls = 0;
    g_query_ret = 0;
    memset(&g_query_stat, 0, sizeof(g_query_stat));
}

/*
 * The channel is up and taking frames, so the statistic is real and MI is the
 * only place it can come from. Nothing here may short-circuit.
 */
static void test_a_receiving_channel_is_queried(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    uint32_t bitrate = 0;
    bool busy = false;

    setup(&ctx, &st, true);
    g_query_stat.bitrate = 4096;
    g_query_stat.leftPics = 2;

    CHECK(hal_enc_get_avg_bitrate(&ctx, 0, &bitrate) == RSS_OK, "bitrate read succeeds");
    CHECK(bitrate == 4096, "the driver's own figure is reported, got %u", bitrate);
    CHECK(g_query_calls == 1, "one query, got %u", g_query_calls);

    CHECK(hal_enc_query(&ctx, 0, &busy) == RSS_OK, "query succeeds");
    CHECK(busy, "frames queued means busy");
    CHECK(g_query_calls == 2, "one query each, got %u", g_query_calls);
}

/*
 * The regression. A created channel that was never started, or was stopped
 * again, has no ring pool -- rvd's JPEG channel between snapshots. Asking MI
 * costs two error lines and returns nothing, so the answer has to come from
 * here without the call.
 */
static void test_an_idle_channel_is_never_queried(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    uint32_t bitrate = 12345;
    bool busy = true;

    setup(&ctx, &st, false);
    /* Would be reported if the guard were missing and the call still made. */
    g_query_stat.bitrate = 4096;
    g_query_stat.leftPics = 9;

    CHECK(hal_enc_get_avg_bitrate(&ctx, 0, &bitrate) == RSS_OK,
          "an idle channel's bitrate is not an error");
    CHECK(bitrate == 0, "an idle channel averages zero, got %u", bitrate);

    CHECK(hal_enc_query(&ctx, 0, &busy) == RSS_OK, "an idle channel's query is not an error");
    CHECK(!busy, "a channel that is not receiving is holding nothing");

    CHECK(g_query_calls == 0, "MI must not be called at all, got %u calls", g_query_calls);
}

/*
 * Stopping is what rvd's frame loop does when the last snapshot consumer
 * leaves, so the guard has to follow the flag rather than only the initial
 * state -- a channel that has been started once must go quiet again.
 */
static void test_the_guard_follows_the_flag(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    uint32_t bitrate = 0;

    setup(&ctx, &st, true);
    g_query_stat.bitrate = 4096;

    CHECK(hal_enc_get_avg_bitrate(&ctx, 0, &bitrate) == RSS_OK, "while receiving, it asks");
    CHECK(g_query_calls == 1, "one query so far, got %u", g_query_calls);

    st.enc[0].receiving = false;
    bitrate = 999;
    CHECK(hal_enc_get_avg_bitrate(&ctx, 0, &bitrate) == RSS_OK, "once stopped, it does not");
    CHECK(bitrate == 0, "and reports zero, got %u", bitrate);
    CHECK(g_query_calls == 1, "still one query, got %u", g_query_calls);
}

/*
 * A channel that was never created is a caller error rather than an idle
 * channel, and the two must not collapse into the same answer -- NOENT says
 * "no such channel", where RSS_OK with zero says "nothing to report yet".
 */
static void test_an_uncreated_channel_still_says_noent(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    uint32_t bitrate = 0;
    bool busy = false;

    setup(&ctx, &st, false);
    st.enc[0].created = false;

    CHECK(hal_enc_get_avg_bitrate(&ctx, 0, &bitrate) == RSS_ERR_NOENT, "bitrate says NOENT");
    CHECK(hal_enc_query(&ctx, 0, &busy) == RSS_ERR_NOENT, "query says NOENT");
    CHECK(g_query_calls == 0, "and neither reaches MI, got %u", g_query_calls);
}

/*
 * A genuine failure from a channel that *is* receiving still has to surface.
 * The guard removes a call that could not work; it does not make the op
 * unable to fail.
 */
static void test_a_real_failure_still_reports(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    uint32_t bitrate = 0;
    bool busy = false;

    setup(&ctx, &st, true);
    g_query_ret = -1;

    CHECK(hal_enc_get_avg_bitrate(&ctx, 0, &bitrate) == RSS_ERR_IO, "a failing query is IO");
    CHECK(hal_enc_query(&ctx, 0, &busy) == RSS_ERR_IO, "on both ops");
    CHECK(g_query_calls == 2, "both did call, got %u", g_query_calls);
}

/*
 * The rate-mode numbers that go on the wire.
 *
 * Read from the driver, not from upstream: _MI_VENC_IMPL_CheckRcMode, inlined
 * into MI_VENC_IMPL_CreateChn, range-checks per codec, and the shipped
 * infinity6e and infinity6b0 mi_venc.ko agree exactly --
 *
 *     eType 2 (H.264)   mode-1 <= 4   ->  1..5
 *     eType 3 (H.265)   mode-8 <= 3   ->  8..11
 *
 * Five H.264 slots, MJPEG's two, then four H.265. The version this pins
 * replaces had the H.264 block as CBR/VBR/FIXQP/AVBR/UBR, one slot short,
 * which put FIXQP and AVBR one low. Measured on an SSC333 before the fix:
 * asking for smart gave a channel the driver reported as FixQP, and asking for
 * fixqp landed on slot 3 -- ABR, which this part does not implement, so the
 * channel never came up and the kernel logged an error. H.265 was unaffected
 * either way because its block starts at a fixed 8 and holds four modes.
 */
static void test_rc_modes_use_the_drivers_numbering(void)
{
    static const struct {
        rss_rc_mode_t ask;
        i6_venc_ratemode h264;
        i6_venc_ratemode h265;
        const char *proc_name;
    } expect[] = {
        {RSS_RC_CBR, 1, 8, "CBR"},
        {RSS_RC_VBR, 2, 9, "VBR"},
        {RSS_RC_FIXQP, 4, 10, "FixQP"},
        {RSS_RC_SMART, 5, 11, "AVBR"},
        {RSS_RC_CAPPED_VBR, 5, 11, "AVBR"},
        {RSS_RC_CAPPED_QUALITY, 5, 11, "AVBR"},
    };
    size_t i;

    for (i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
        CHECK(star_enc_ratemode(RSS_CODEC_H264, expect[i].ask) == expect[i].h264,
              "H.264 %s -> %d, got %d", expect[i].proc_name, (int)expect[i].h264,
              (int)star_enc_ratemode(RSS_CODEC_H264, expect[i].ask));
        CHECK(star_enc_ratemode(RSS_CODEC_H265, expect[i].ask) == expect[i].h265,
              "H.265 %s -> %d, got %d", expect[i].proc_name, (int)expect[i].h265,
              (int)star_enc_ratemode(RSS_CODEC_H265, expect[i].ask));
    }

    /* MJPEG sits between the two blocks and takes only these two. */
    CHECK(star_enc_ratemode(RSS_CODEC_MJPEG, RSS_RC_CBR) == 6, "MJPEG CBR is 6");
    CHECK(star_enc_ratemode(RSS_CODEC_MJPEG, RSS_RC_FIXQP) == 7, "MJPEG FixQP is 7");
    CHECK(star_enc_ratemode(RSS_CODEC_MJPEG, RSS_RC_VBR) == I6_VENC_RATEMODE_END,
          "and refuses the rest rather than substituting");

    /* Every value lands inside the range CheckRcMode enforces. */
    for (i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
        CHECK(expect[i].h264 >= 1 && expect[i].h264 <= 5, "H.264 modes are 1..5");
        CHECK(expect[i].h265 >= 8 && expect[i].h265 <= 11, "H.265 modes are 8..11");
    }
}

int main(void)
{
    test_a_receiving_channel_is_queried();
    test_an_idle_channel_is_never_queried();
    test_the_guard_follows_the_flag();
    test_an_uncreated_channel_still_says_noent();
    test_a_real_failure_still_reports();
    test_rc_modes_use_the_drivers_numbering();

    if (failures) {
        printf("t_enc: %d failure(s)\n", failures);
        return 1;
    }
    printf("all hal_enc logic tests passed\n");
    return 0;
}
