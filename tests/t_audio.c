/*
 * Host-side logic tests for src/star/hal_audio.c.
 *
 * Includes the real translation unit rather than a copy, so the code under
 * test is the code that ships. Everything MI-side is a function pointer in
 * star_state_t, which is what makes the frame-lifecycle paths testable
 * without hardware -- the stubs below stand in for libmi_ai.so.
 *
 * What this is actually for: the volume mapping and the frame bookkeeping.
 * MI_AI_SetVqeVolume takes an *index* into a per-device gain table, not a
 * decibel value, and both reference streamers pass dB-shaped numbers -- so
 * the mapping is the single most likely thing here to be wrong, and it is
 * pure arithmetic that a host can check exactly.
 *
 * As in t_isp.c the i6_*.h _Static_asserts are suppressed via
 * -D'_Static_assert(c,m)=': they assert 32-bit pointer layouts, and the real
 * ARM build still checks every one.
 */

#define PLATFORM_INFINITY6E 1

#include "star/hal_audio.c"

#include <stdio.h>

/* HAL_LOG_* call through this with no NULL guard (rss_hal_init always
 * installs one), so a test that trips a warning path needs a real
 * function here, not NULL. */
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

/* ---- MI_AI stand-ins ------------------------------------------------- */

static struct {
    int get_calls, free_calls, vol_calls, mute_calls;
    int disable_chn_calls, disable_dev_calls;
    int last_dev, last_chn, last_vol, last_mute;
    int get_ret, free_ret, vol_ret;

    /* MI_AI_SetChnParam -- the digital trim, which is where volume goes now
     * that gain owns the analog table. */
    int chn_calls, chn_ret;
    int last_gain_on, last_front, last_rear;
    unsigned char buf[64];
    unsigned int length;
    int timeout_ms;

    /* Bring-up sequence */
    int cfg_calls, enable_dev_calls, enable_chn_calls, depth_calls;
    int depth_ret;
    unsigned int depth_usr, depth_buf;
    i6_sys_bind depth_port;
    bool depth_after_enable_chn;
    i6_aud_cnf dev_cfg;

    /* Number of leading fnGetFrame calls that report NOBUF before the port
     * behaves. Drives the recovery tests. */
    int nobuf_remaining;
} mi;

static int fake_set_cfg(int dev, i6_aud_cnf *cfg)
{
    (void)dev;
    mi.cfg_calls++;
    mi.dev_cfg = *cfg;
    return 0;
}

static int fake_enable_dev(int dev)
{
    (void)dev;
    mi.enable_dev_calls++;
    return 0;
}

static int fake_enable_chn(int dev, int chn)
{
    (void)dev;
    (void)chn;
    mi.enable_chn_calls++;
    return 0;
}

static int fake_set_depth(i6_sys_bind *port, unsigned int usr, unsigned int buf)
{
    mi.depth_calls++;
    mi.depth_port = *port;
    mi.depth_usr = usr;
    mi.depth_buf = buf;
    /* Recorded rather than asserted here so the check reads in the test:
     * MI describes the queue of a channel that already exists. */
    mi.depth_after_enable_chn = mi.enable_chn_calls >= mi.depth_calls;
    return mi.depth_ret;
}

static int fake_get(int dev, int chn, i6_aud_frm *frm, i6_aud_efrm *enc, int millis)
{
    mi.get_calls++;
    mi.last_dev = dev;
    mi.last_chn = chn;
    mi.timeout_ms = millis;
    CHECK(enc == NULL, "AEC reference frame must be NULL -- the algorithm is absent");
    if (mi.nobuf_remaining > 0) {
        mi.nobuf_remaining--;
        return (int)STAR_AUD_ERR_NOBUF;
    }
    if (mi.get_ret)
        return mi.get_ret;
    frm->addr[0] = mi.length ? mi.buf : NULL;
    frm->length = mi.length;
    frm->timestamp = 1234567ULL;
    frm->sequence = 42;
    return 0;
}

static int fake_free(int dev, int chn, i6_aud_frm *frm, i6_aud_efrm *enc)
{
    mi.free_calls++;
    mi.last_dev = dev;
    mi.last_chn = chn;
    CHECK(frm != NULL, "release must hand MI back a descriptor");
    CHECK(enc == NULL, "AEC reference frame must be NULL on release too");
    return mi.free_ret;
}

static int fake_vol(int dev, int chn, int level)
{
    mi.vol_calls++;
    mi.last_dev = dev;
    mi.last_chn = chn;
    mi.last_vol = level;
    return mi.vol_ret;
}

static int fake_set_chn_para(int dev, int chn, i6_aud_chn_para *para)
{
    mi.chn_calls++;
    mi.last_dev = dev;
    mi.last_chn = chn;
    CHECK(para != NULL, "SetChnParam must be handed a block");
    if (para) {
        mi.last_gain_on = para->gain.gainOn;
        mi.last_front = para->gain.front;
        mi.last_rear = para->gain.rear;
    }
    return mi.chn_ret;
}

static int fake_mute(int dev, int chn, char active)
{
    mi.mute_calls++;
    mi.last_dev = dev;
    mi.last_chn = chn;
    mi.last_mute = active;
    return 0;
}

static int fake_disable_chn(int dev, int chn)
{
    (void)dev;
    (void)chn;
    mi.disable_chn_calls++;
    return 0;
}

static int fake_disable_dev(int dev)
{
    (void)dev;
    mi.disable_dev_calls++;
    return 0;
}

static void setup(rss_hal_ctx_t *ctx, star_state_t *st, int chn_count)
{
    memset(ctx, 0, sizeof(*ctx));
    memset(st, 0, sizeof(*st));
    memset(&mi, 0, sizeof(mi));
    mi.length = 320; /* 160 samples of 16-bit mono = 10 ms at 16 kHz */

    ctx->platform = st;
    st->aud_loaded = true;
    st->aud_dev = STAR_AUD_DEV;
    st->aud_chn_count = (unsigned int)chn_count;
    st->aud_rate = 16000;
    st->aud_input = RSS_AUDIO_INPUT_AMIC;
    st->aud.fnGetFrame = fake_get;
    st->aud.fnFreeFrame = fake_free;
    st->aud.fnSetVolume = fake_vol;
    st->aud.fnSetChannelParam = fake_set_chn_para;
    /* What hal_audio_init leaves when the library has no MI_AI_GetChnParam:
     * "no front-end index established yet", which is not the same as zero. */
    st->aud_front_idx = -1;
    st->aud.fnSetMute = fake_mute;
    st->aud.fnDisableChannel = fake_disable_chn;
    st->aud.fnDisableDevice = fake_disable_dev;
    st->aud.fnSetDeviceConfig = fake_set_cfg;
    st->aud.fnEnableDevice = fake_enable_dev;
    st->aud.fnEnableChannel = fake_enable_chn;
    st->sys.fnSetOutputDepth = fake_set_depth;
}

/* ---- tests ----------------------------------------------------------- */

static void test_rate_gate(void)
{
    /* The four MI supports. */
    CHECK(star_audio_rate_ok(8000), "8000 accepted");
    CHECK(star_audio_rate_ok(16000), "16000 accepted");
    CHECK(star_audio_rate_ok(32000), "32000 accepted");
    CHECK(star_audio_rate_ok(48000), "48000 accepted");

    /* The ones raptor's enum offers that MI does not, and which would
     * otherwise reach a resampler that is a NULL weak symbol here. */
    CHECK(!star_audio_rate_ok(24000), "24000 refused -- MI does not support it");
    CHECK(!star_audio_rate_ok(44100), "44100 refused -- MI does not support it");
    CHECK(!star_audio_rate_ok(0), "0 refused");
    CHECK(!star_audio_rate_ok(-16000), "negative refused");
}

/*
 * The analog front end is gain's, not volume's.
 *
 * Both references drive MI_AI_SetVqeVolume with a dB-shaped number, but the
 * argument is an *index* into a per-device table -- 0..21 on amic, 0..4 on
 * dmic. raptor's gain is 0..31, so this is the map between them, and it is
 * what Infinity6C already does: gain analog, volume digital.
 */
static void test_gain_maps_to_table_index(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    setup(&ctx, &st, 1);

    /* Endpoints land exactly on the ends. A gain of 31 that produced 22 would
     * earn ILLEGAL_PARAM, and a 0 that produced 1 would put the quietest
     * setting out of reach. */
    CHECK(star_audio_gain_index(&st, 0) == 0, "gain 0 -> index 0, got %d",
          star_audio_gain_index(&st, 0));
    CHECK(star_audio_gain_index(&st, STAR_AUD_GAIN_MAX) == STAR_AUD_IF_GAIN_MAX_AMIC,
          "gain %d -> index %d, got %d", STAR_AUD_GAIN_MAX, STAR_AUD_IF_GAIN_MAX_AMIC,
          star_audio_gain_index(&st, STAR_AUD_GAIN_MAX));

    /* The interior points are the ones swept on an SSC333 and read back out of
     * /proc/mi_modules/mi_ai/mi_ai0, so these are measurements rather than
     * restatements of the arithmetic. */
    CHECK(star_audio_gain_index(&st, 8) == 5, "gain 8 -> 5, got %d", star_audio_gain_index(&st, 8));
    CHECK(star_audio_gain_index(&st, 16) == 11, "gain 16 -> 11, got %d",
          star_audio_gain_index(&st, 16));
    CHECK(star_audio_gain_index(&st, 25) == 17, "gain 25 -> 17, got %d",
          star_audio_gain_index(&st, 25));

    /* Out-of-range input is clamped, never passed through. */
    CHECK(star_audio_gain_index(&st, -50) == 0, "negative gain clamps to 0");
    CHECK(star_audio_gain_index(&st, 1000) == STAR_AUD_IF_GAIN_MAX_AMIC,
          "huge gain clamps to the table end");

    /* Monotonic, and never off the end anywhere in between. */
    for (int g = 0; g <= STAR_AUD_GAIN_MAX; g++) {
        int idx = star_audio_gain_index(&st, g);
        int prev = star_audio_gain_index(&st, g ? g - 1 : 0);

        CHECK(idx >= 0 && idx <= STAR_AUD_IF_GAIN_MAX_AMIC, "gain %d -> index %d out of table", g,
              idx);
        CHECK(idx >= prev, "gain %d gave a lower index than %d", g, g - 1);
    }

    /* Dmic's table is much shorter, and picking the wrong ceiling would push a
     * mid-range gain off the end of it. */
    st.aud_input = RSS_AUDIO_INPUT_DMIC;
    CHECK(star_audio_gain_index(&st, STAR_AUD_GAIN_MAX) == STAR_AUD_IF_GAIN_MAX_DMIC,
          "dmic gain %d -> index %d, got %d", STAR_AUD_GAIN_MAX, STAR_AUD_IF_GAIN_MAX_DMIC,
          star_audio_gain_index(&st, STAR_AUD_GAIN_MAX));
    for (int g = 0; g <= STAR_AUD_GAIN_MAX; g++)
        CHECK(star_audio_gain_index(&st, g) <= STAR_AUD_IF_GAIN_MAX_DMIC,
              "dmic gain %d ran off its table", g);
}

static void test_set_gain_passes_index_not_percent(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    int gain = -1;

    setup(&ctx, &st, 1);

    /* The regression this guards: passing 25 straight to MI. 25 is not a valid
     * index in any of the tables. */
    CHECK(hal_audio_set_gain(&ctx, STAR_AUD_DEV, 0, 25) == RSS_OK, "set_gain succeeds");
    CHECK(mi.vol_calls == 1, "one SetVqeVolume call, got %d", mi.vol_calls);
    CHECK(mi.last_vol == 17, "gain 25 should reach MI as index 17, got %d", mi.last_vol);
    CHECK(mi.last_vol <= STAR_AUD_IF_GAIN_MAX_AMIC, "index must be inside the table");

    /* And it is remembered, so a later volume write does not have to guess a
     * front-end index. */
    CHECK(st.aud_front_idx == 17, "the index is held for set_volume, got %d", st.aud_front_idx);

    CHECK(hal_audio_get_gain(&ctx, STAR_AUD_DEV, 0, &gain) == RSS_OK, "get_gain succeeds");
    CHECK(gain == 25, "get_gain returns the requested value, got %d", gain);

    /* A failing MI call must not be recorded as the live gain. */
    mi.vol_ret = -1;
    CHECK(hal_audio_set_gain(&ctx, STAR_AUD_DEV, 0, 10) != RSS_OK, "a failing set is reported");
    CHECK(hal_audio_get_gain(&ctx, STAR_AUD_DEV, 0, &gain) == RSS_OK, "get_gain still works");
    CHECK(gain == 25, "failed set must not update tracked gain, got %d", gain);
}

/*
 * Volume is the digital DPGA trim, in dB, through MI_AI_SetChnParam -- the
 * stage nothing could reach before it was bound. Unity is 80, so a default
 * install writes 0 dB and the picture is unchanged from when volume drove the
 * analog table.
 *
 * The four points below were read back off an SSC333's own
 * /proc/mi_modules/mi_ai/mi_ai0 rather than derived from the helper.
 */
static void test_set_volume_is_the_digital_trim(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    int vol = -1;

    setup(&ctx, &st, 1);

    CHECK(star_audio_volume_db(0) == STAR_AUD_DPGA_MIN_DB, "volume 0 -> %d dB, got %d",
          STAR_AUD_DPGA_MIN_DB, star_audio_volume_db(0));
    CHECK(star_audio_volume_db(40) == -30, "volume 40 -> -30 dB, got %d", star_audio_volume_db(40));
    CHECK(star_audio_volume_db(STAR_AUD_VOL_UNITY) == 0, "unity -> 0 dB, got %d",
          star_audio_volume_db(STAR_AUD_VOL_UNITY));
    CHECK(star_audio_volume_db(100) == STAR_AUD_DPGA_MAX_DB, "volume 100 -> %d dB, got %d",
          STAR_AUD_DPGA_MAX_DB, star_audio_volume_db(100));

    CHECK(hal_audio_set_volume(&ctx, STAR_AUD_DEV, 0, 80) == RSS_OK, "set_volume succeeds");
    CHECK(mi.chn_calls == 1, "one SetChnParam call, got %d", mi.chn_calls);
    CHECK(mi.last_rear == 0, "volume 80 is 0 dB of trim, got %d", mi.last_rear);
    CHECK(mi.last_gain_on == 1, "the gain block has to be switched on to be read");
    CHECK(mi.vol_calls == 0, "volume must not touch the analog table");

    /* With no gain set and no index established, the front end is derived from
     * the gain this backend holds rather than written as a zero -- which would
     * silently mute the preamp on the way past. */
    CHECK(mi.last_front == star_audio_gain_index(&st, st.aud_gain),
          "front comes from the held gain, got %d", mi.last_front);

    /* An index established by set_gain is the one volume carries through. */
    CHECK(hal_audio_set_gain(&ctx, STAR_AUD_DEV, 0, 25) == RSS_OK, "set_gain succeeds");
    CHECK(hal_audio_set_volume(&ctx, STAR_AUD_DEV, 0, 100) == RSS_OK, "set_volume succeeds");
    CHECK(mi.last_front == 17, "volume preserves the analog index, got %d", mi.last_front);
    CHECK(mi.last_rear == STAR_AUD_DPGA_MAX_DB, "volume 100 is +%d dB, got %d",
          STAR_AUD_DPGA_MAX_DB, mi.last_rear);

    /* get_volume answers in raptor's units, not MI's. */
    CHECK(hal_audio_get_volume(&ctx, STAR_AUD_DEV, 0, &vol) == RSS_OK, "get_volume succeeds");
    CHECK(vol == 100, "get_volume returns the requested percent, got %d", vol);

    /* A failing MI call must not be recorded as the live volume. */
    mi.chn_ret = -1;
    CHECK(hal_audio_set_volume(&ctx, STAR_AUD_DEV, 0, 10) != RSS_OK, "a failing set is reported");
    CHECK(hal_audio_get_volume(&ctx, STAR_AUD_DEV, 0, &vol) == RSS_OK, "get_volume still works");
    CHECK(vol == 100, "failed set must not update tracked volume, got %d", vol);
}

/*
 * A library too old to carry MI_AI_SetChnParam loses the digital trim and keeps
 * capturing: the pair is bound with plain dlsym and left NULL on a miss, and
 * this is the answer that reaches rad.
 */
static void test_volume_without_the_digital_stage(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;

    setup(&ctx, &st, 1);
    st.aud.fnSetChannelParam = NULL;

    CHECK(hal_audio_set_volume(&ctx, STAR_AUD_DEV, 0, 80) == RSS_ERR_NOTSUP,
          "no SetChnParam means no volume control");
    CHECK(mi.vol_calls == 0, "and it must not fall back onto the analog table");
}

static void test_frame_lifecycle(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;

    setup(&ctx, &st, 1);

    memset(&frame, 0, sizeof(frame));
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_OK, "read succeeds");
    CHECK(mi.timeout_ms == STAR_AUD_GET_TIMEOUT_MS, "blocking read waits %d ms, got %d",
          STAR_AUD_GET_TIMEOUT_MS, mi.timeout_ms);
    CHECK(frame.data == (const int16_t *)mi.buf, "frame points into MI's buffer, not a copy");
    CHECK(frame.length == 320, "length carried through, got %u", frame.length);
    CHECK(frame.timestamp == 1234567, "MI's timestamp is reported, got %lld",
          (long long)frame.timestamp);
    CHECK(frame.seq == 42, "sequence carried through, got %u", frame.seq);
    CHECK(frame._priv == &st.aud_frame[0], "_priv carries the descriptor MI must see again");
    CHECK(st.aud_frame_held[0], "the frame is marked held");

    /* Reading twice without releasing would leak MI's buffer. */
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_BUSY,
          "a second read without release is refused");
    CHECK(mi.get_calls == 1, "the refused read must not reach MI, got %d calls", mi.get_calls);

    CHECK(hal_audio_release_frame(&ctx, STAR_AUD_DEV, 0, &frame) == RSS_OK, "release succeeds");
    CHECK(mi.free_calls == 1, "one ReleaseFrame, got %d", mi.free_calls);
    CHECK(!st.aud_frame_held[0], "the frame is no longer held");
    CHECK(frame.data == NULL && frame.length == 0 && frame._priv == NULL,
          "the released frame is cleared so a stale pointer cannot be reused");

    /* Releasing again is a caller error, not a second ReleaseFrame. */
    CHECK(hal_audio_release_frame(&ctx, STAR_AUD_DEV, 0, &frame) == RSS_ERR_INVAL,
          "a double release is refused");
    CHECK(mi.free_calls == 1, "the refused release must not reach MI, got %d", mi.free_calls);

    /* And a read after release works again. */
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_OK,
          "read works again after release");
    CHECK(hal_audio_release_frame(&ctx, STAR_AUD_DEV, 0, &frame) == RSS_OK, "and releases");
}

static void test_empty_buffer_is_a_timeout_not_an_error(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;

    setup(&ctx, &st, 1);

    /* "Nothing captured yet" is the one failure rad's loop must be able to
     * ignore quietly, so it has to arrive as RSS_ERR_TIMEOUT. */
    mi.get_ret = (int)STAR_AUD_ERR_BUF_EMPTY;
    memset(&frame, 0, sizeof(frame));
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_TIMEOUT,
          "BUF_EMPTY reads as a timeout");
    CHECK(!st.aud_frame_held[0], "a timeout holds no frame");

    /* Any other code is a real error and must be distinguishable. */
    mi.get_ret = 0xA0042017; /* MI_AI_ERR_NOT_ENABLED */
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_IO,
          "a real failure reads as an error");
    CHECK(st.aud_last_err == (int)0xA0042017, "the code is latched so it warns once");

    /* A success carrying nothing is handed straight back rather than
     * passed up as a NULL buffer. */
    mi.get_ret = 0;
    mi.length = 0;
    mi.free_calls = 0;
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_TIMEOUT,
          "an empty success is a timeout");
    CHECK(mi.free_calls == 1, "the empty frame is given back to MI, got %d", mi.free_calls);
    CHECK(!st.aud_frame_held[0], "and is not left marked held");

    /* A non-blocking read must not sit in MI for the timeout. */
    mi.length = 320;
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, false) == RSS_OK,
          "non-blocking read succeeds");
    CHECK(mi.timeout_ms == 0, "non-blocking read waits 0 ms, got %d", mi.timeout_ms);
}

static void test_device_and_channel_bounds(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;

    setup(&ctx, &st, 1);

    /* rad's default [audio] device is 1, an Ingenic index. The configured
     * device wins, is warned about once, and capture still works -- the
     * alternative is silence for a value nobody chose. */
    CHECK(hal_audio_read_frame(&ctx, 1, 0, &frame, true) == RSS_OK,
          "a mismatched device still captures");
    CHECK(mi.last_dev == STAR_AUD_DEV, "MI is addressed with the configured device, got %d",
          mi.last_dev);
    CHECK(st.aud_dev_warned, "the mismatch is recorded as warned");
    CHECK(hal_audio_release_frame(&ctx, 1, 0, &frame) == RSS_OK, "and releases");

    /* Channel 1 is out of range on a mono device. */
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 1, &frame, true) == RSS_ERR_INVAL,
          "channel beyond chn_count is refused");
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, -1, &frame, true) == RSS_ERR_INVAL,
          "negative channel is refused");
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, STAR_AUD_CHN_MAX, &frame, true) ==
                  RSS_ERR_INVAL,
          "channel at the array bound is refused");

    /* Stereo makes channel 1 legal. */
    setup(&ctx, &st, 2);
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 1, &frame, true) == RSS_OK,
          "channel 1 works on a stereo device");
    CHECK(mi.last_chn == 1, "MI is addressed with channel 1, got %d", mi.last_chn);
    CHECK(st.aud_frame_held[1] && !st.aud_frame_held[0],
          "per-channel bookkeeping does not cross channels");
    CHECK(hal_audio_release_frame(&ctx, STAR_AUD_DEV, 1, &frame) == RSS_OK, "and releases");

    /* Nothing loaded, and NULL arguments. */
    st.aud_loaded = false;
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_NOTSUP,
          "reading with no library loaded is NOTSUP");
    CHECK(hal_audio_read_frame(NULL, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_INVAL,
          "NULL ctx rejected");
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, NULL, true) == RSS_ERR_INVAL,
          "NULL frame rejected");
}

static void test_mute_and_teardown(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;

    setup(&ctx, &st, 1);

    CHECK(hal_audio_set_mute(&ctx, STAR_AUD_DEV, 0, 1) == RSS_OK, "mute succeeds");
    CHECK(mi.last_mute == 1, "mute passes 1, got %d", mi.last_mute);
    CHECK(hal_audio_set_mute(&ctx, STAR_AUD_DEV, 0, 0) == RSS_OK, "unmute succeeds");
    CHECK(mi.last_mute == 0, "unmute passes 0, got %d", mi.last_mute);
    /* Any nonzero means mute, not just 1. */
    CHECK(hal_audio_set_mute(&ctx, STAR_AUD_DEV, 0, 7) == RSS_OK, "mute with 7 succeeds");
    CHECK(mi.last_mute == 1, "a nonzero mute value normalises to 1, got %d", mi.last_mute);

    /* Teardown must return a frame that is still checked out, or MI keeps
     * the buffer for a channel that no longer exists. */
    st.aud_dev_enabled = true;
    st.aud_chn_enabled[0] = true;
    st.aud.fnDisableChannel = fake_disable_chn;
    st.aud.fnDisableDevice = fake_disable_dev;
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_OK, "read succeeds");
    mi.free_calls = 0;
    star_audio_teardown(&st);
    CHECK(mi.free_calls == 1, "teardown releases the held frame, got %d", mi.free_calls);
    CHECK(!st.aud_frame_held[0], "and clears the held flag");
    CHECK(!st.aud_chn_enabled[0] && !st.aud_dev_enabled, "channel and device are marked down");

    /* Teardown again must be a no-op rather than a second round of
     * Disable calls on things already down. */
    mi.free_calls = 0;
    mi.disable_chn_calls = 0;
    mi.disable_dev_calls = 0;
    star_audio_teardown(&st);
    CHECK(mi.free_calls == 0 && mi.disable_chn_calls == 0 && mi.disable_dev_calls == 0,
          "a second teardown touches nothing");
}

/*
 * The bring-up sequence, and specifically the output-port queue.
 *
 * Added after the first board run: everything above tested the frame and
 * volume logic against a hand-built star_state_t, so nothing ever drove
 * hal_audio_init, and the missing MI_SYS_SetChnOutputPortDepth got all the
 * way to hardware. MI_AI_GetFrame then returned NOBUF (0xA004200D) on every
 * call while the device reported itself enabled -- a failure with no
 * symptom other than silence.
 *
 * Note this drives init through its already-loaded branch, which is what
 * keeps it host-testable: the other branch dlopens libmi_ai.so.
 */
static void test_init_sets_the_output_port_queue(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_config_t cfg;
    unsigned int i;

    for (i = 1; i <= 2; i++) {
        setup(&ctx, &st, (int)i);

        memset(&cfg, 0, sizeof(cfg));
        cfg.sample_rate = RSS_AUDIO_RATE_16000;
        cfg.chn_count = (int)i;
        cfg.input_type = RSS_AUDIO_INPUT_AMIC;

        CHECK(hal_audio_init(&ctx, &cfg) == RSS_OK, "init succeeds for %u channel(s)", i);

        /* One queue, always, because there is one MI channel however many PCM
         * channels it carries: eSoundmode says stereo and the interleaved pair
         * arrives in apVirAddr[0] with the full length. An earlier build read
         * u32ChnCnt as the PCM count and enabled two, which the driver reports
         * as EnChnNum and the board showed as 1. */
        CHECK(mi.depth_calls == 1, "%u PCM channel(s) -> %d SetChnOutputPortDepth call(s), want 1",
              i, mi.depth_calls);
        CHECK(mi.depth_usr == STAR_AUD_PORT_USR_DEPTH && mi.depth_buf == STAR_AUD_PORT_BUF_DEPTH,
              "depths passed through as (%d, %d), got (%u, %u)", STAR_AUD_PORT_USR_DEPTH,
              STAR_AUD_PORT_BUF_DEPTH, mi.depth_usr, mi.depth_buf);

        /* The port has to name the AI channel that was just enabled.
         * Addressing the wrong module or channel returns success and
         * leaves the real port empty, which looks identical on the
         * board. */
        CHECK(mi.depth_port.module == I6_SYS_MOD_AI, "port names the AI module");
        CHECK(mi.depth_port.device == (unsigned int)STAR_AUD_DEV, "port names the AI device");
        CHECK(mi.depth_port.channel == 0, "the one AI channel is 0, got %u",
              mi.depth_port.channel);
        CHECK(mi.depth_port.port == 0, "AI has a single output port, 0");

        /* Ordering: MI describes the queue of an enabled channel. */
        CHECK(mi.depth_after_enable_chn, "depth is set after MI_AI_EnableChn, not before");
    }

    /* A failing depth call must not be shrugged off -- capture would be
     * silent forever, so it fails init and unwinds instead. */
    setup(&ctx, &st, 1);
    mi.depth_ret = -1;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sample_rate = RSS_AUDIO_RATE_16000;
    cfg.chn_count = 1;
    CHECK(hal_audio_init(&ctx, &cfg) != RSS_OK, "a failed depth call fails init");
    CHECK(mi.disable_chn_calls == 1 && mi.disable_dev_calls == 1,
          "a failed depth call tears the device back down");
    CHECK(!st.aud_dev_enabled, "device is not left marked enabled after a failed init");
}

/*
 * NOBUF is the error that means "this port has no user-side queue", so the
 * response is to establish it again and retry -- the board reports it when rad
 * starts during boot but not when rad is started by hand against the same
 * config a little later.
 */
static void test_nobuf_reestablishes_the_port_queue(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;
    int ret;

    setup(&ctx, &st, 1);

    /* One NOBUF, then the port works. */
    mi.nobuf_remaining = 1;
    ret = hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true);
    CHECK(ret == RSS_OK, "a recovered read succeeds (got %d)", ret);
    CHECK(mi.depth_calls == 1, "the port depth is re-applied exactly once (got %d)",
          mi.depth_calls);
    CHECK(mi.get_calls == 2, "and the frame is fetched again after it (got %d)", mi.get_calls);
    CHECK(mi.depth_usr == STAR_AUD_PORT_USR_DEPTH && mi.depth_buf == STAR_AUD_PORT_BUF_DEPTH,
          "with the same depths init uses");
    CHECK(mi.depth_port.module == I6_SYS_MOD_AI && mi.depth_port.channel == 0,
          "naming the AI channel that failed");
    hal_audio_release_frame(&ctx, STAR_AUD_DEV, 0, &frame);

    /* The budget is spent per fault, not per process, so a port lost a second
     * time is still recovered -- it is only the *refill* that now requires a
     * sustained run of good frames rather than one. */
    mi.depth_calls = 0;
    mi.get_calls = 0;
    mi.nobuf_remaining = 1;
    ret = hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true);
    CHECK(ret == RSS_OK, "a second, later loss is recovered too (got %d)", ret);
    CHECK(mi.depth_calls == 1, "spending the next attempt in the budget (got %d)",
          mi.depth_calls);
    hal_audio_release_frame(&ctx, STAR_AUD_DEV, 0, &frame);
}

/*
 * Re-applying the port depth FLUSHES whatever the port had queued, so a fault
 * that alternates with successful reads must not be able to re-apply at the
 * capture period rate. Refilling the budget on a single good frame allowed
 * exactly that: it shipped, and on the board it destroyed about five periods
 * of audio at a time, continuously, for as long as rad ran.
 */
static void test_one_good_frame_does_not_refill_the_recovery_budget(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;
    int i;

    setup(&ctx, &st, 1);

    /* Each pass: one NOBUF, a re-apply, then a frame. Under the old rule the
     * delivered frame refilled the budget and this ran forever. */
    for (i = 0; i < STAR_AUD_NOBUF_RECOVER_MAX + 5; i++) {
        mi.nobuf_remaining = 1;
        if (hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_OK)
            hal_audio_release_frame(&ctx, STAR_AUD_DEV, 0, &frame);
    }

    CHECK(mi.depth_calls == STAR_AUD_NOBUF_RECOVER_MAX,
          "the re-apply stops at the budget even when frames keep arriving "
          "(expected %d, got %d)",
          STAR_AUD_NOBUF_RECOVER_MAX, mi.depth_calls);
}

/*
 * MI_AI overloads NOBUF: it is both "this port has no user-side queue" and the
 * answer an unblocked fetch gives when nothing is queued yet. The vendor docs
 * for VENC and VDEC promise BUF_EMPTY for the latter, MI_AI does not do that,
 * and that is what made a backlog-draining build worse than the one it
 * replaced -- every empty poll was read as a lost queue and re-applied the
 * depth, flushing the queue it was polling.
 *
 * So: on a non-blocking read, NOBUF is a timeout and must touch nothing.
 */
static void test_nonblocking_nobuf_is_empty_not_a_lost_queue(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;
    int ret;

    setup(&ctx, &st, 1);

    mi.nobuf_remaining = 1;
    ret = hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, false);
    CHECK(ret == RSS_ERR_TIMEOUT, "an unblocked NOBUF reads as a timeout (got %d)", ret);
    CHECK(mi.depth_calls == 0, "and does not re-apply the port depth (got %d)", mi.depth_calls);
    CHECK(mi.get_calls == 1, "and does not retry the fetch (got %d)", mi.get_calls);

    /* The protection is not lost: a blocking read still recovers the port. */
    mi.nobuf_remaining = 1;
    ret = hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true);
    CHECK(ret == RSS_OK, "a blocking read still recovers a genuinely lost queue (got %d)", ret);
    CHECK(mi.depth_calls == 1, "by re-applying the depth (got %d)", mi.depth_calls);
    hal_audio_release_frame(&ctx, STAR_AUD_DEV, 0, &frame);
}

/*
 * The retry runs at the capture period, so a port that is genuinely dead must
 * stop being re-applied rather than spin forever.
 */
static void test_nobuf_recovery_is_bounded(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;
    int i;

    setup(&ctx, &st, 1);

    /* Never recovers. */
    for (i = 0; i < STAR_AUD_NOBUF_RECOVER_MAX + 4; i++) {
        int ret;
        mi.nobuf_remaining = 2; /* both the first read and the retry fail */
        ret = hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true);
        CHECK(ret == RSS_ERR_IO, "persistent NOBUF is still reported as an error (read %d)", i);
    }
    CHECK(mi.depth_calls == STAR_AUD_NOBUF_RECOVER_MAX,
          "at most %d re-applies for a dead port (got %d)", STAR_AUD_NOBUF_RECOVER_MAX,
          mi.depth_calls);

    /* A failing SetChnOutputPortDepth must not be retried into a frame fetch,
     * and must still spend budget so it cannot loop. */
    setup(&ctx, &st, 1);
    mi.depth_ret = -1;
    mi.nobuf_remaining = 1;
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_IO,
          "a failed re-apply reports the read error");
    CHECK(mi.get_calls == 1, "without a second fetch attempt (got %d)", mi.get_calls);
}

/* BUF_EMPTY is the harmless neighbour of NOBUF, one digit away, and must not
 * trigger any of this. */
static void test_buf_empty_does_not_touch_the_port(void)
{
    rss_hal_ctx_t ctx;
    star_state_t st;
    rss_audio_frame_t frame;

    setup(&ctx, &st, 1);
    mi.get_ret = (int)STAR_AUD_ERR_BUF_EMPTY;
    CHECK(hal_audio_read_frame(&ctx, STAR_AUD_DEV, 0, &frame, true) == RSS_ERR_TIMEOUT,
          "BUF_EMPTY is still just a timeout");
    CHECK(mi.depth_calls == 0, "and never re-applies the port depth (got %d)", mi.depth_calls);
}

int main(void)
{
    test_rate_gate();
    test_gain_maps_to_table_index();
    test_set_gain_passes_index_not_percent();
    test_set_volume_is_the_digital_trim();
    test_volume_without_the_digital_stage();
    test_frame_lifecycle();
    test_empty_buffer_is_a_timeout_not_an_error();
    test_device_and_channel_bounds();
    test_mute_and_teardown();
    test_init_sets_the_output_port_queue();
    test_nobuf_reestablishes_the_port_queue();
    test_nobuf_recovery_is_bounded();
    test_one_good_frame_does_not_refill_the_recovery_budget();
    test_nonblocking_nobuf_is_empty_not_a_lost_queue();
    test_buf_empty_does_not_touch_the_port();

    if (failures) {
        printf("\n%d check(s) failed\n", failures);
        return 1;
    }

    printf("all hal_audio logic tests passed\n");
    return 0;
}
