/*
 * Host-side test of the cascade bookkeeping in infinity6c/hal_encoder.c.
 *
 * Same construction as t_isp_i6c.c: the real translation unit is included rather
 * than copied, so what is tested is what ships, and MI is stubbed by filling in
 * the function pointers in infinity6c_state_t.
 *
 * What this suite is about is a topology no other backend has. Only one SCL port
 * can ring an H.26x channel on this part, so the second video stream is not a
 * scaler port at all: it is a VENC main->sub cascade, chn 1 fed from chn 0 inside
 * the encoder. That makes the sub's liveness a property of a channel it does not
 * own, and every assertion here is some form of the same question -- what happens
 * to the sub when the main is taken away and given back, which is what rvd does
 * on set-resolution and set-codec.
 *
 * The MI calls are recorded as a trace rather than as counters, because the
 * ordering is half of what is being tested: unbinding a sub after its main is
 * destroyed is not the same as unbinding it before, and on hardware the wrong
 * one wedges in the kernel rather than failing.
 */

#define PLATFORM_INFINITY6C 1
#define HAL_MODULE_VIDEO 1

#include "infinity6c/hal_encoder.c"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

/*
 * Silent, but it keeps what it was given: one of the behaviours below is a
 * warning, and a diagnostic's whole observable behaviour is the line it prints.
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

/* Terminating as well as rewinding, so a stale line cannot be matched. */
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

/* ================================================================
 * THE REST OF THE BACKEND
 *
 * hal_encoder.c calls into its neighbours for ports, overlay and buffering.
 * None of that is what this suite is about, and all of it succeeds quietly --
 * a cascade sub touches none of it anyway, holding no port of its own.
 * ================================================================ */

void i6c_set_output_depth(infinity6c_state_t *st, i6c_sys_mod mod, unsigned int dev,
                          unsigned int chn, unsigned int port, unsigned int min, unsigned int max)
{
    (void)st;
    (void)mod;
    (void)dev;
    (void)chn;
    (void)port;
    (void)min;
    (void)max;
}

int i6c_fs_spare_port(const infinity6c_state_t *st, unsigned int width)
{
    (void)st;
    (void)width;
    return -1;
}

int i6c_fs_snapshot_port(infinity6c_state_t *st, int port, unsigned short width,
                         unsigned short height, i6c_common_pixfmt pixfmt)
{
    (void)st;
    (void)port;
    (void)width;
    (void)height;
    (void)pixfmt;
    return RSS_OK;
}

int i6c_fs_port_ifc(infinity6c_state_t *st, int port)
{
    (void)st;
    (void)port;
    return RSS_OK;
}

int i6c_fs_enable_port(infinity6c_state_t *st, int port)
{
    (void)st;
    (void)port;
    return RSS_OK;
}

void i6c_fs_release_port(infinity6c_state_t *st, int port)
{
    (void)st;
    (void)port;
}

void i6c_osd_flush_pending(infinity6c_state_t *st, int chn)
{
    (void)st;
    (void)chn;
}

void i6c_osd_release_all(infinity6c_state_t *st)
{
    (void)st;
}

void i6c_isp_note_frame(infinity6c_state_t *st)
{
    (void)st;
}

void i6c_pipeline_destroy(infinity6c_state_t *st)
{
    (void)st;
}

/* ================================================================
 * MI
 * ================================================================ */

static char g_trace[4096];
static size_t g_trace_len;

static void trace(const char *fmt, ...)
{
    size_t room = sizeof(g_trace) - g_trace_len;
    va_list ap;
    int n;

    if (room < 3)
        return;

    va_start(ap, fmt);
    n = vsnprintf(g_trace + g_trace_len, room - 2, fmt, ap);
    va_end(ap);
    if (n < 0)
        return;

    g_trace_len += (size_t)n < room - 2 ? (size_t)n : room - 3;
    g_trace[g_trace_len++] = '\n';
    g_trace[g_trace_len] = '\0';
}

static void trace_reset(void)
{
    g_trace_len = 0;
    g_trace[0] = '\0';
}

/* Where a line sits in the trace, or -1. Ordering is asserted by comparing two. */
static int at(const char *needle)
{
    const char *hit = strstr(g_trace, needle);

    return hit ? (int)(hit - g_trace) : -1;
}

static int fake_create_dev(unsigned int device, i6c_venc_init *config)
{
    (void)config;
    trace("create_dev(%u)", device);
    return 0;
}

static int fake_destroy_dev(unsigned int device)
{
    trace("destroy_dev(%u)", device);
    return 0;
}

static int fake_create_chn(unsigned int device, unsigned int channel, i6c_venc_chn *config)
{
    (void)config;
    trace("create_chn(%u,%u)", device, channel);
    return 0;
}

static int fake_destroy_chn(unsigned int device, unsigned int channel)
{
    trace("destroy_chn(%u,%u)", device, channel);
    return 0;
}

static int fake_set_src_conf(unsigned int device, unsigned int channel, i6c_venc_src_conf *config)
{
    (void)config;
    trace("set_src_conf(%u,%u)", device, channel);
    return 0;
}

static int fake_start_recv(unsigned int device, unsigned int channel)
{
    trace("start_recv(%u,%u)", device, channel);
    return 0;
}

static int fake_stop_recv(unsigned int device, unsigned int channel)
{
    trace("stop_recv(%u,%u)", device, channel);
    return 0;
}

static int fake_close_fd(unsigned int device, unsigned int channel)
{
    trace("close_fd(%u,%u)", device, channel);
    return 0;
}

static int fake_config_pool(unsigned short soc_id, i6c_sys_pool *config)
{
    (void)soc_id;
    (void)config;
    return 0;
}

/* What a bind refuses, and how many were asked for. */
static int g_bind_ret;
static int g_refuse_cascade;
static int g_binds;

static const char *mod_name(i6c_sys_mod mod)
{
    return mod == I6C_SYS_MOD_VENC ? "venc" : "scl";
}

static int fake_bind_ext(unsigned short soc_id, i6c_sys_bind *src, i6c_sys_bind *dest,
                         unsigned int src_fps, unsigned int dest_fps, i6c_sys_link link,
                         unsigned int link_param)
{
    (void)soc_id;
    (void)src_fps;
    (void)link_param;

    g_binds++;
    if (g_bind_ret)
        return g_bind_ret;
    /* A cascade is the VENC -> VENC leg; the main's own is SCL -> VENC. */
    if (g_refuse_cascade && src->module == I6C_SYS_MOD_VENC)
        return -1;

    trace("bind(%s%u:%u.%u->%s%u:%u,%s,%ufps)", mod_name(src->module), src->device, src->channel,
          src->port, mod_name(dest->module), dest->device, dest->channel,
          link == I6C_SYS_LINK_RING ? "ring" : "frame", dest_fps);
    return 0;
}

static int fake_unbind(unsigned short soc_id, i6c_sys_bind *src, i6c_sys_bind *dest)
{
    (void)soc_id;

    trace("unbind(%s%u:%u.%u->%s%u:%u)", mod_name(src->module), src->device, src->channel,
          src->port, mod_name(dest->module), dest->device, dest->channel);
    return 0;
}

/* ================================================================
 * THE PIPELINE rvd BUILDS
 * ================================================================ */

static void reset(infinity6c_state_t *st, rss_hal_ctx_t *ctx)
{
    int i;

    memset(st, 0, sizeof(*st));
    memset(ctx, 0, sizeof(*ctx));
    ctx->platform = st;

    st->fps = 25;
    for (i = 0; i < I6C_MAX_CHN; i++) {
        st->enc[i].src_port = -1;
        st->enc[i].cascade_src = -1;
        st->enc[i].fd = -1;
    }
    for (i = 0; i < I6C_VENC_DEV_SLOTS; i++)
        st->enc_ring_chn[i] = -1;

    st->venc.create_dev = fake_create_dev;
    st->venc.destroy_dev = fake_destroy_dev;
    st->venc.create_chn = fake_create_chn;
    st->venc.destroy_chn = fake_destroy_chn;
    st->venc.set_src_conf = fake_set_src_conf;
    st->venc.start_recv = fake_start_recv;
    st->venc.stop_recv = fake_stop_recv;
    st->venc.close_fd = fake_close_fd;
    st->sys.bind_ext = fake_bind_ext;
    st->sys.unbind = fake_unbind;
    st->sys.config_pool = fake_config_pool;

    g_bind_ret = 0;
    g_refuse_cascade = 0;
    g_binds = 0;
    trace_reset();
    log_reset();
}

static rss_video_config_t video_cfg(unsigned short w, unsigned short h)
{
    rss_video_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.codec = RSS_CODEC_H265;
    cfg.width = w;
    cfg.height = h;
    cfg.fps_num = 25;
    cfg.fps_den = 1;
    cfg.bitrate = 2048;
    cfg.gop_length = 50;
    cfg.rc_mode = RSS_RC_CBR;
    return cfg;
}

/*
 * What rvd does for a video stream: create the channel, then bind it. hal_bind
 * passes the framesource channel as the port and zero for the rate, and for the
 * sub the bind lands in the cascade path instead.
 */
static void bring_up_stream(rss_hal_ctx_t *ctx, infinity6c_state_t *st, int chn, unsigned short w,
                            unsigned short h)
{
    rss_video_config_t cfg = video_cfg(w, h);

    assert(hal_enc_create_channel(ctx, chn, &cfg) == RSS_OK);
    assert(i6c_bind_scl_to_venc(st, chn, chn, 0) == RSS_OK);
}

/* Both video streams, the shape every test here starts from. */
static void bring_up_both(rss_hal_ctx_t *ctx, infinity6c_state_t *st)
{
    reset(st, ctx);
    bring_up_stream(ctx, st, 0, 1920, 1080);
    bring_up_stream(ctx, st, 1, 640, 360);
}

/* ================================================================
 * TESTS
 * ================================================================ */

static void test_the_sub_cascades_off_the_main(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;

    bring_up_both(&ctx, &st);

    /* The premise of everything below: chn 1 has no SCL port, it has chn 0. */
    CHECK(st.enc[0].uses_ring, "the first H.26x channel must take the ring");
    CHECK(!st.enc[1].uses_ring, "the second must not take a second ring");
    CHECK(st.enc[1].cascade && st.enc[1].cascade_src == 0,
          "chn 1 must be a cascade off chn 0, got cascade=%d src=%d", st.enc[1].cascade,
          st.enc[1].cascade_src);
    CHECK(at("bind(scl0:0.0->venc0:0,ring,25fps)") >= 0,
          "the main rings off its SCL port, got:\n%s", g_trace);
    CHECK(at("bind(venc0:0.0->venc0:1,ring,25fps)") >= 0, "the sub rings off the main, got:\n%s",
          g_trace);
}

/*
 * The bug: rvd's set-resolution destroys and recreates one channel, and when
 * that channel is the main, the sub is fed by it. Left bound, the sub points at
 * a destroyed channel and polls ETIMEDOUT until the daemon restarts.
 */
static void test_destroying_the_main_unhooks_the_sub_first(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;

    bring_up_both(&ctx, &st);
    trace_reset();

    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");

    CHECK(at("unbind(venc0:0.0->venc0:1)") >= 0, "the sub must be unbound, got:\n%s", g_trace);
    CHECK(at("stop_recv(0,1)") >= 0, "the sub must stop receiving, got:\n%s", g_trace);

    /*
     * Before, not after. An encoder left receiving across its unbind strands an
     * MI task that the kernel flush then waits on forever, and unbinding a sub
     * from a channel MI no longer has is not a wait anyone returns from. This is
     * the order i6c_teardown_all uses on the path that always worked.
     */
    CHECK(at("stop_recv(0,1)") < at("unbind(venc0:0.0->venc0:1)"),
          "the sub stops receiving before it is unbound, got:\n%s", g_trace);
    CHECK(at("unbind(venc0:0.0->venc0:1)") < at("destroy_chn(0,0)"),
          "the sub is unbound before its main is destroyed, got:\n%s", g_trace);

    /* The sub is a stream rvd never asked about. It keeps its channel. */
    CHECK(at("destroy_chn(0,1)") < 0, "the sub's own channel must survive, got:\n%s", g_trace);
    CHECK(st.enc[1].created, "the sub must still be created");
    CHECK(st.enc[1].width == 640 && st.enc[1].height == 360,
          "the sub keeps its own geometry, got %ux%u", st.enc[1].width, st.enc[1].height);
    CHECK(!st.enc[1].bound && !st.enc[1].cascade && !st.enc[1].receiving,
          "the sub must be left unbound and stopped");
    CHECK(st.enc[1].cascade_pending, "the sub must be recorded as waiting for a main");
}

/* And the other half: the new main picks the waiting sub back up. */
static void test_the_new_main_takes_the_waiting_sub(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;

    bring_up_both(&ctx, &st);
    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");
    trace_reset();

    /* set-resolution's second half: the same channel back at a new size. */
    bring_up_stream(&ctx, &st, 0, 1280, 720);

    CHECK(at("bind(venc0:0.0->venc0:1,ring,25fps)") >= 0,
          "the sub must be re-cascaded off the new main, got:\n%s", g_trace);
    CHECK(at("start_recv(0,1)") >= 0, "the sub must be started again, got:\n%s", g_trace);
    CHECK(st.enc[1].bound && st.enc[1].cascade && st.enc[1].cascade_src == 0,
          "the sub must be bound to the new main again");
    CHECK(st.enc[1].receiving, "the sub must be receiving again");
    CHECK(!st.enc[1].cascade_pending, "nothing is waiting once it is bound");

    /*
     * After the main is itself bound and receiving. A cascade source that is not
     * yet fed has no ring to hang anything off.
     */
    CHECK(at("bind(scl0:0.0->venc0:0,ring,25fps)") < at("bind(venc0:0.0->venc0:1,ring,25fps)"),
          "the main is bound before the sub cascades off it, got:\n%s", g_trace);
}

/*
 * The sub's geometry is its own and the main's is the main's. A resize of one
 * must not resize the other -- rvd sets the sub's size separately, and the VENC
 * cascade is what reduces the frame.
 */
static void test_the_resize_does_not_reach_the_sub(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;

    bring_up_both(&ctx, &st);
    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");
    bring_up_stream(&ctx, &st, 0, 1280, 720);

    CHECK(st.enc[0].width == 1280 && st.enc[0].height == 720,
          "the main is at its new size, got %ux%u", st.enc[0].width, st.enc[0].height);
    CHECK(st.enc[1].width == 640 && st.enc[1].height == 360, "the sub is unchanged, got %ux%u",
          st.enc[1].width, st.enc[1].height);
}

/*
 * A sub that is destroyed while waiting is gone, not waiting. Otherwise the next
 * main to come up would bind a channel MI no longer has -- the same class of
 * mistake as the bug itself, in the other direction.
 */
static void test_a_sub_destroyed_while_waiting_is_not_rebound(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;

    bring_up_both(&ctx, &st);
    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");
    CHECK(hal_enc_destroy_channel(&ctx, 1) == RSS_OK, "destroying the sub must succeed");
    CHECK(!st.enc[1].cascade_pending, "a destroyed sub is not waiting for anything");
    trace_reset();

    bring_up_stream(&ctx, &st, 0, 1280, 720);

    CHECK(at("bind(venc0:0.0->venc0:1") < 0,
          "nothing must be cascaded onto a destroyed channel, "
          "got:\n%s",
          g_trace);
}

/*
 * A cascade that will not go back is a lost sub stream and not a lost main. rvd
 * asked for the main; it gets it.
 */
static void test_a_refused_cascade_does_not_take_the_main_down(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;
    rss_video_config_t cfg = video_cfg(1280, 720);

    bring_up_both(&ctx, &st);
    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");
    CHECK(hal_enc_create_channel(&ctx, 0, &cfg) == RSS_OK, "the main must be created");

    g_refuse_cascade = 1;
    CHECK(i6c_bind_scl_to_venc(&st, 0, 0, 0) == RSS_OK,
          "the main's own bind succeeded, so the main's bind must report success");
    CHECK(st.enc[0].bound && st.enc[0].receiving, "the main must be up regardless");
    CHECK(g_binds >= 2, "the cascade must have been attempted, got %d binds", g_binds);
    CHECK(!st.enc[1].bound, "the sub must not be marked bound after a refusal");

    /* Still waiting, so the next main gets to try. */
    CHECK(st.enc[1].cascade_pending, "a refused sub keeps waiting for a main");

    g_refuse_cascade = 0;
    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");
    bring_up_stream(&ctx, &st, 0, 1280, 720);
    CHECK(st.enc[1].bound && st.enc[1].cascade,
          "the next main must pick up the sub the last one could not");
}

/* The full teardown still leads with its own pre-pass, through the same helper. */
static void test_the_full_teardown_still_unhooks_first(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;

    bring_up_both(&ctx, &st);
    trace_reset();

    i6c_teardown_all(&st);

    CHECK(at("unbind(venc0:0.0->venc0:1)") >= 0, "the sub must be unbound, got:\n%s", g_trace);
    CHECK(at("unbind(venc0:0.0->venc0:1)") < at("destroy_chn(0,0)"),
          "the sub is unbound before the main is destroyed, got:\n%s", g_trace);
    CHECK(at("destroy_chn(0,1)") >= 0, "the sub is destroyed too on this path, got:\n%s", g_trace);
    CHECK(at("destroy_chn(0,0)") < at("destroy_dev(0)"),
          "channels go before their device, got:\n%s", g_trace);
}

/*
 * A JPEG channel is not a sub. It is frame-based off a port of its own, so a
 * main going away must not stop or unbind it.
 */
static void test_a_jpeg_channel_is_not_a_sub(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;
    rss_video_config_t cfg = video_cfg(1920, 1080);

    bring_up_both(&ctx, &st);
    cfg.codec = RSS_CODEC_JPEG;
    CHECK(hal_enc_create_channel(&ctx, 2, &cfg) == RSS_OK, "the JPEG channel must be created");
    CHECK(i6c_bind_scl_to_venc(&st, 0, 2, 1) == RSS_OK, "the JPEG channel must bind");
    CHECK(!st.enc[2].cascade, "a JPEG channel never cascades");
    trace_reset();

    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");

    CHECK(at("stop_recv(8,2)") < 0, "the JPEG channel must be left alone, got:\n%s", g_trace);
    CHECK(st.enc[2].bound, "the JPEG channel keeps its own bind");
    CHECK(!st.enc[2].cascade_pending, "the JPEG channel is not waiting for a main");
}

/*
 * The cascade only scales down. A sub larger than its main is a bind MI takes
 * and does not honour -- measured on an SSC377QE, where a main dropped to
 * 320x240 under a 640x480 sub left that sub polling ETIMEDOUT until the main was
 * grown again. Legal, transient, self-healing, and completely silent, which is
 * the part that is fixed here.
 */
static void test_a_sub_larger_than_its_main_says_so(void)
{
    infinity6c_state_t st;
    rss_hal_ctx_t ctx;

    /* The ordinary way round says nothing. */
    bring_up_both(&ctx, &st);
    CHECK(strstr(g_log, "only scales down") == NULL,
          "a sub smaller than its main is unremarkable, got:\n%s", g_log);

    /* And the resize that inverts them warns, naming both sizes. */
    CHECK(hal_enc_destroy_channel(&ctx, 0) == RSS_OK, "destroying the main must succeed");
    log_reset();
    bring_up_stream(&ctx, &st, 0, 320, 240);

    CHECK(strstr(g_log, "venc chn 1 is 640x360 and cascades off chn 0 at 320x240") != NULL,
          "the warning must name the sub, the main and both sizes, got:\n%s", g_log);
    CHECK(strstr(g_log, "only scales down") != NULL, "the warning must say why, got:\n%s", g_log);

    /* A warning, not a refusal: rvd asked for this and the state is recoverable. */
    CHECK(st.enc[1].bound && st.enc[1].cascade, "the sub is still bound");
    CHECK(!st.enc[1].cascade_pending, "and is not left waiting");
}

int main(void)
{
    test_the_sub_cascades_off_the_main();
    test_destroying_the_main_unhooks_the_sub_first();
    test_the_new_main_takes_the_waiting_sub();
    test_the_resize_does_not_reach_the_sub();
    test_a_sub_destroyed_while_waiting_is_not_rebound();
    test_a_refused_cascade_does_not_take_the_main_down();
    test_the_full_teardown_still_unhooks_first();
    test_a_jpeg_channel_is_not_a_sub();
    test_a_sub_larger_than_its_main_says_so();

    if (failures) {
        printf("t_enc_i6c: %d failure(s)\n", failures);
        return 1;
    }
    printf("t_enc_i6c: ok\n");
    return 0;
}
