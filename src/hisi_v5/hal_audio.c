/*
 * hisi_v5/hal_audio.c -- AI audio capture for HiMPP V5
 *
 * THE POINT OF THIS FILE
 *
 * Raw PCM in, nothing else -- gen4's hal_audio.c, on V5's spellings. rad
 * owns encoding (G.711, L16, Opus and AAC in software), so all this has
 * to do is bring up the AI device and the inner codec, hand out frames,
 * and stay out of the way. The vendor's own sample splits the work the
 * same way: MPI for the digital path, /dev/acodec ioctls for everything
 * analog.
 *
 * WHY audio_init DOES THE WORK hal_init NORMALLY DOES
 *
 * rad calls rss_hal_create and then audio_init directly; it never calls
 * the init op. So this file loads the vendor libraries and attaches to
 * MPP itself when nothing has -- trampoline check first, for the same
 * reason hal_init runs it first (hal_common.c, FORWARDERS).
 *
 * ONE MPP CONSUMER PER SYSTEM -- what this file may and may not touch
 *
 * SYS and VB state are kernel-global. audio_init calls ss_mpi_sys_init
 * only -- the per-process attach AI needs -- and never configures VB,
 * never runs a reclaim, and never calls ss_mpi_sys_exit either, because
 * rvd may be streaming in another process and a global exit is not this
 * archive's to issue. The process's own exit is the real detach.
 * aud_owns_sys records the attach for the log, not for an undo. (Measured
 * on the CV608: ss_mpi_sys_init succeeds in a second process with no VB
 * of its own, so the VB-first order hal_init keeps is not a condition of
 * the attach.)
 *
 * The one global thing it does own is the AB pool, ss_mpi_audio_init /
 * ss_mpi_audio_exit (v5_aud.h): the audio buffer module is rad's alone,
 * so init at attach and exit at deinit are the vendor's own pairing.
 *
 * A consequence to know: if rvd (or another MPP owner) has never run
 * since boot, ss_mpi_sys_init here may fail because VB was never
 * configured. rad on this backend expects to run beside rvd; the error
 * message says as much when it happens.
 *
 * MONO ONLY, for gen4's reason: an AI frame comes back as two planes with
 * len counting bytes per channel, rss_audio_frame_t is one pointer and
 * one length, and the inner codec is one mic front end. A stereo request
 * is refused rather than quietly delivering the left channel.
 *
 * OP COVERAGE
 *
 * Implemented: audio_init, audio_deinit, audio_read_frame,
 * audio_release_frame, audio_set_volume, audio_get_volume,
 * audio_set_gain, audio_get_gain, audio_set_mute.
 *
 * The analog controls map onto /dev/acodec:
 *
 *   volume  OT_ACODEC_SET_INPUT_VOLUME, dB [-78..80] (-78 = mute) per the
 *           vendor sample's comment, which also says 20..50 is the band
 *           where only the analog gain moves and the noise is lowest.
 *           rad's scale is [-30..120] with 60 = unity, so the mapping is
 *           dB = vol - 60, clamped -- the same rule as gen4, and rad's
 *           default 80 lands at 20 dB, the bottom of that band.
 *   gain    OT_ACODEC_SET_GAIN_MIC{L,R}, the mic preamp steps, usable
 *           range [0..15] -- the header says 0..0x1f, the driver answers
 *           EPERM from 16 up; see V5_ACODEC_GAIN_MIC_MAX, where the
 *           measurement is written down. rad's default gain is 25, so a
 *           request is clamped rather than passed through, and the clamp
 *           is logged once.
 *   mute    OT_ACODEC_SET_MIC{L,R}_MUTE.
 *
 * Everything else stays NULL in the vtable -> RSS_ERR_NOTSUP, for the
 * reasons gen4's file gives: VQE (record/talk) is a few hundred bytes of
 * chip-specific config struct each, an ABI promise nothing has paid for;
 * AENC is unused because rad encodes in software; audio_get_chn_param
 * takes Ingenic's struct; the ao_* family is playback.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* One 20 ms period at 8 kHz is 160 samples; a bounded blocking read keeps
 * rad's loop able to notice shutdown even if the device goes quiet. */
#define HISI_AUD_GET_TIMEOUT_MS 1000

/* The userspace frame queue (ot_ai_chn_param.usr_frame_depth): sixteen
 * 20 ms periods of slack for a scheduling stall, gen4's number; V5 caps
 * it at 30. Config wins when it says something. */
#define HISI_AUD_DEPTH_DEFAULT 16

/*
 * Which analog input feeds the mic PGA. The vendor's sample selects
 * IN_D (pseudo-differential) for its demo board and names IN1
 * (single-ended) for the socket board; the driver's own reset state on the
 * CV608 is IN0. The bench board has no microphone populated -- at the top
 * preamp step and 50 dB of input volume not one of the three inputs hears
 * the room -- so the default was settled on the noise floor instead.
 * Measured over RTSP on 2026-09-08, six seconds each: IN1 -64 dBFS,
 * IN_D -52, IN0 -50, and IN1 carries 17 dB less below 100 Hz. IN1 is also
 * the input gen4 selects and the vendor's own answer for anything that is
 * not their demo board. Revisit on the first board with a mic on it.
 */
#define V5_ACODEC_MIXER_DEFAULT V5_ACODEC_MIXER_IN1

static bool hisi_audio_rate_ok(int rate)
{
    return v5_acodec_fs(rate) >= 0;
}

/* ================================================================
 * THE INNER CODEC
 * ================================================================ */

static int hisi_acodec_ioctl(hisi_state_t *st, unsigned long req, void *arg, const char *what)
{
    if (st->acodec_fd < 0)
        return RSS_ERR_NOTSUP;
    if (ioctl(st->acodec_fd, req, arg)) {
        HAL_LOG_WARN("acodec: %s failed: %s", what, strerror(errno));
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

static int hisi_acodec_setup(hisi_state_t *st, int rate)
{
    unsigned int fs = (unsigned int)v5_acodec_fs(rate);
    unsigned int mixer = V5_ACODEC_MIXER_DEFAULT;

    st->acodec_fd = open(V5_ACODEC_PATH, O_RDWR);
    if (st->acodec_fd < 0) {
        /* An external-codec board has no /dev/acodec; capture can still
         * work if something else configured the codec, so this degrades
         * the analog controls instead of failing the device. */
        HAL_LOG_WARN("acodec: %s: %s -- volume/gain/mute unavailable", V5_ACODEC_PATH,
                     strerror(errno));
        return RSS_OK;
    }

    if (ioctl(st->acodec_fd, V5_ACODEC_SOFT_RESET))
        HAL_LOG_WARN("acodec: soft reset failed: %s", strerror(errno));
    hisi_acodec_ioctl(st, V5_ACODEC_SET_I2S1_FS, &fs, "set I2S FS");
    hisi_acodec_ioctl(st, V5_ACODEC_SET_MIXER_MIC, &mixer, "select mic input");
    return RSS_OK;
}

/* ================================================================
 * LIFECYCLE
 * ================================================================ */

static void hisi_audio_stop(hisi_state_t *st)
{
    if (st->aud_frame_held) {
        st->aud.fnReleaseFrame(st->aud_dev, 0, &st->aud_frame, &st->aud_aec);
        st->aud_frame_held = false;
    }
    if (st->aud_chn_enabled) {
        int ret = st->aud.fnDisableChn(st->aud_dev, 0);
        if (ret)
            HAL_LOG_WARN("ss_mpi_ai_disable_chn failed: 0x%x", ret);
        st->aud_chn_enabled = false;
    }
    if (st->aud_dev_enabled) {
        int ret = st->aud.fnDisable(st->aud_dev);
        if (ret)
            HAL_LOG_WARN("ss_mpi_ai_disable failed: 0x%x", ret);
        st->aud_dev_enabled = false;
    }
}

int hal_audio_init(void *ctx, const rss_audio_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st;
    v5_aio_attr attr;
    unsigned int samples;
    int ret;

    if (!c || !cfg)
        return RSS_ERR_INVAL;

    if (!hisi_audio_rate_ok((int)cfg->sample_rate)) {
        HAL_LOG_ERR("audio: %d Hz is not an inner-codec rate", (int)cfg->sample_rate);
        return RSS_ERR_INVAL;
    }

    /* Mono only -- read_frame publishes one plane; see the file comment. */
    if (cfg->chn_count > 1) {
        HAL_LOG_ERR("audio: %d PCM channels requested; this backend captures mono only "
                    "(the inner codec is one mic path and read_frame publishes a single "
                    "plane, so a stereo frame's right channel would be dropped)",
                    cfg->chn_count);
        return RSS_ERR_NOTSUP;
    }

    /*
     * State and MPP. Present already when something called hal_init;
     * created here when nothing did, which is rad's normal path.
     */
    st = hisi_state(ctx);
    if (!st) {
        st = (hisi_state_t *)calloc(1, sizeof(*st));
        if (!st)
            return RSS_ERR_NOMEM;
        st->acodec_fd = -1;

        /* Before the first vendor dlopen; see the file comment. */
        hisi_check_trampolines();

        ret = hisi_mpi_open(&st->libs);
        if (ret) {
            free(st);
            return ret;
        }
        ret = v5_sys_load(&st->sys, &st->libs);
        if (ret) {
            hisi_mpi_close(&st->libs);
            free(st);
            return ret;
        }

        ret = st->sys.fnInit();
        if (ret) {
            HAL_LOG_ERR("ss_mpi_sys_init failed: 0x%x (err %u) -- on this backend audio expects "
                        "rvd (the MPP owner) to be up first; see hal_audio.c",
                        (unsigned)ret, V5_ERR_ID(ret));
            v5_sys_unload(&st->sys);
            hisi_mpi_close(&st->libs);
            free(st);
            return RSS_ERR_IO;
        }
        /* The attach is recorded but never undone from this archive --
         * SYS is kernel-global and not ours to exit. File comment. */
        st->aud_owns_sys = true;
        c->platform = st;
    }

    if (!st->aud_loaded) {
        ret = v5_aud_open_libs(&st->libs);
        if (ret)
            return ret;
        ret = v5_aud_load(&st->aud, &st->libs);
        if (ret)
            return ret;
        /*
         * The AB pool -- V5's audio buffer module, see v5_aud.h. The AI
         * ring lives in it, and ss_mpi_ai_enable answers NO_MEM until this
         * has run. Once per attach; ss_mpi_audio_exit at deinit.
         */
        /*
         * Exit before init, which is what the vendor's own
         * sample_comm_audio_init does and what this did not. It matters
         * for the same reason the video path tears SYS and VB down first:
         * a rad that was killed rather than stopped -- the OOM killer's
         * pick on a board whose Linux half is 27 MB -- leaves the AI
         * device enabled and owned by a process that no longer exists,
         * and the next rad's ss_mpi_ai_set_pub_attr answers NOT_PERM
         * (0xa015800d) for as long as the box stays up. From the new
         * process ss_mpi_ai_disable is refused too; only the module-level
         * exit clears it. On a clean boot there is nothing to exit and
         * this fails harmlessly, so the result is ignored.
         */
        st->aud.fnAudioExit();

        ret = st->aud.fnAudioInit();
        if (ret) {
            HAL_LOG_ERR("ss_mpi_audio_init failed: 0x%x (err %u)", (unsigned)ret, V5_ERR_ID(ret));
            v5_aud_unload(&st->aud);
            return RSS_ERR_IO;
        }
        st->aud_loaded = true;
    } else {
        /* A rate change on a running device: stop before reconfiguring;
         * set_pub_attr describes a device that is not running. */
        hisi_audio_stop(st);
        if (st->acodec_fd >= 0) {
            close(st->acodec_fd);
            st->acodec_fd = -1;
        }
    }

    st->aud_dev = 0;
    st->aud_rate = (int)cfg->sample_rate;
    st->aud_first_frame = true;

    /* Samples per frame per channel; one 20 ms period unless the config
     * says otherwise, which is also rad's own slicing. */
    samples = cfg->samples_per_frame > 0 ? (unsigned int)cfg->samples_per_frame
                                         : (unsigned int)cfg->sample_rate / 50;
    if (samples == 0 || samples > (unsigned int)cfg->sample_rate) {
        unsigned int fallback = (unsigned int)cfg->sample_rate / 50;

        HAL_LOG_WARN("audio: %u samples/frame is out of range; using %u", samples, fallback);
        samples = fallback;
    }

    /*
     * I2S master against the inner codec, 16-bit mono. The config carries
     * no bus-role or codec-type fields, so those are defaults rather than
     * policy: what the vendor sample uses and what the inner codec
     * requires. Rate, period and depth come from the config; the channel
     * count does not -- it is fixed at one, checked above.
     *
     * Two of these are not gen4's, and the driver refuses the attribute
     * (ILLEGAL_PARAM) without them: chn_cnt is 2, the inner codec's FS
     * carrying a left and a right slot -- in mono sound mode that is two
     * AI channels, and channel 0 is the one read -- and clk_share is 1,
     * AI on the codec's one clock. Both are the vendor sample's values.
     */
    memset(&attr, 0, sizeof(attr));
    attr.sample_rate = st->aud_rate;
    attr.bit_width = V5_AUD_BIT_WIDTH_16;
    attr.work_mode = V5_AUD_MODE_I2S_MASTER;
    attr.snd_mode = V5_AUD_SOUND_MONO;
    attr.expand_flag = 0;
    attr.frame_num = 30; /* device-side ring, frames; the sample's number */
    attr.point_num_per_frame = samples;
    attr.chn_cnt = 2;
    attr.clk_share = 1;
    attr.i2s_type = V5_AUD_I2S_INNERCODEC;

    ret = st->aud.fnSetPubAttr(st->aud_dev, &attr);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_ai_set_pub_attr(%d) failed: 0x%x (err %u)", st->aud_dev, (unsigned)ret,
                    V5_ERR_ID(ret));
        return RSS_ERR_IO;
    }

    ret = st->aud.fnEnable(st->aud_dev);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_ai_enable(%d) failed: 0x%x (err %u)", st->aud_dev, (unsigned)ret,
                    V5_ERR_ID(ret));
        return RSS_ERR_IO;
    }
    st->aud_dev_enabled = true;

    ret = st->aud.fnEnableChn(st->aud_dev, 0);
    if (ret) {
        HAL_LOG_ERR("ss_mpi_ai_enable_chn(%d, 0) failed: 0x%x (err %u)", st->aud_dev, (unsigned)ret,
                    V5_ERR_ID(ret));
        hisi_audio_stop(st);
        return RSS_ERR_IO;
    }
    st->aud_chn_enabled = true;

    /*
     * The analog codec AFTER the AI device is up -- the reverse of gen4's
     * order, and the vendor sample's on V5 (sample_audio_ai_aenc starts AI
     * and then calls sample_comm_audio_cfg_acodec): the driver answers
     * EPERM to the FS select while the I2S clock AI provides is not
     * running.
     */
    hisi_acodec_setup(st, st->aud_rate);

    /* The userspace queue: deepen it so a scheduling stall costs latency
     * rather than samples. */
    if (st->aud.fnSetChnParam) {
        v5_ai_chn_param param;
        int depth = cfg->frame_depth > 0 ? cfg->frame_depth : HISI_AUD_DEPTH_DEFAULT;

        if (depth < V5_AI_USER_DEPTH_MIN)
            depth = V5_AI_USER_DEPTH_MIN;
        if (depth > V5_AI_USER_DEPTH_MAX)
            depth = V5_AI_USER_DEPTH_MAX;
        param.usr_frame_depth = (unsigned int)depth;
        ret = st->aud.fnSetChnParam(st->aud_dev, 0, &param);
        if (ret)
            HAL_LOG_WARN("ss_mpi_ai_set_chn_param(depth %d) failed: 0x%x -- capture continues "
                         "on the driver default",
                         depth, ret);
    }

    /* Initial analog levels from the config, through the same mapping the
     * runtime ops use. Zero is a real gain and a real volume on this
     * codec's scales, so only values the config actually set are applied. */
    if (cfg->ai_vol)
        hal_audio_set_volume(ctx, st->aud_dev, 0, cfg->ai_vol);
    if (cfg->ai_gain)
        hal_audio_set_gain(ctx, st->aud_dev, 0, cfg->ai_gain);

    HAL_LOG_INFO("audio: AI dev %d up, %d Hz mono, %u samples/frame", st->aud_dev, st->aud_rate,
                 samples);
    return RSS_OK;
}

int hal_audio_deinit(void *ctx)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_OK;

    if (st->aud_loaded) {
        int ret;

        hisi_audio_stop(st);
        ret = st->aud.fnAudioExit();
        if (ret)
            HAL_LOG_WARN("ss_mpi_audio_exit failed: 0x%x", ret);
        v5_aud_unload(&st->aud);
        st->aud_loaded = false;
    }
    if (st->acodec_fd >= 0) {
        close(st->acodec_fd);
        st->acodec_fd = -1;
    }
    /* No ss_mpi_sys_exit, deliberately -- see the file comment. */
    if (st->aud_owns_sys)
        HAL_LOG_DBG("audio: leaving the MPP attach to process exit (kernel-global state)");

    return RSS_OK;
}

/* ================================================================
 * FRAMES
 * ================================================================ */

int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block)
{
    hisi_state_t *st = hisi_state(ctx);
    int ret;

    (void)dev;

    if (!st || !frame)
        return RSS_ERR_INVAL;
    if (!st->aud_loaded || !st->aud_chn_enabled)
        return RSS_ERR_NOTSUP;
    if (chn != 0)
        return RSS_ERR_INVAL;
    if (st->aud_frame_held) {
        HAL_LOG_WARN("audio: read with a frame still held; release it first");
        return RSS_ERR_BUSY;
    }

    memset(&st->aud_frame, 0, sizeof(st->aud_frame));
    memset(&st->aud_aec, 0, sizeof(st->aud_aec));

    ret = st->aud.fnGetFrame(st->aud_dev, 0, &st->aud_frame, &st->aud_aec,
                             block ? HISI_AUD_GET_TIMEOUT_MS : 0);
    if (ret) {
        /* Empty queue is flow control, not a fault -- on either kind of
         * fetch. NO_BUF likewise: the queue exists (set_chn_param made
         * it), it just has nothing yet. */
        if ((unsigned int)ret == V5_ERR_AI_BUF_EMPTY || (unsigned int)ret == V5_ERR_AI_NO_BUF)
            return RSS_ERR_TIMEOUT;

        if (ret != st->aud_last_err) {
            HAL_LOG_WARN("ss_mpi_ai_get_frame failed: 0x%x (err %u)", (unsigned)ret,
                         V5_ERR_ID(ret));
            st->aud_last_err = ret;
        }
        return RSS_ERR_IO;
    }
    st->aud_last_err = 0;

    if (!st->aud_frame.virt_addr[0] || !st->aud_frame.len) {
        /* A success with nothing in it; give it straight back. */
        st->aud.fnReleaseFrame(st->aud_dev, 0, &st->aud_frame, &st->aud_aec);
        return RSS_ERR_TIMEOUT;
    }

    /* The layout check v5_aud.h promises: a 64-bit phys_addr would put
     * the timestamp and the length four bytes further along, and both
     * would read as nonsense here. Once, at INFO, so the log shows it. */
    if (st->aud_first_frame) {
        st->aud_first_frame = false;
        HAL_LOG_INFO("audio: first frame: %u bytes, seq %u, ts %llu us, %s", st->aud_frame.len,
                     st->aud_frame.seq, st->aud_frame.time_stamp,
                     st->aud_frame.snd_mode == V5_AUD_SOUND_MONO ? "mono" : "stereo");
    }

    st->aud_frame_held = true;

    frame->data = (const int16_t *)st->aud_frame.virt_addr[0];
    frame->length = st->aud_frame.len;
    /* HiMPP's timestamp, microseconds. rad replaces it with a synthetic
     * clock of its own, but reporting what the driver said is still the
     * honest thing for anything that looks. */
    frame->timestamp = (int64_t)st->aud_frame.time_stamp;
    frame->seq = st->aud_frame.seq;
    frame->_priv = &st->aud_frame;

    return RSS_OK;
}

int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame)
{
    hisi_state_t *st = hisi_state(ctx);
    int ret;

    (void)dev;
    (void)chn;
    (void)frame;

    if (!st)
        return RSS_ERR_INVAL;
    if (!st->aud_frame_held)
        return RSS_OK;

    ret = st->aud.fnReleaseFrame(st->aud_dev, 0, &st->aud_frame, &st->aud_aec);
    if (ret)
        HAL_LOG_WARN("ss_mpi_ai_release_frame failed: 0x%x", ret);
    st->aud_frame_held = false;

    return RSS_OK;
}

/* ================================================================
 * ANALOG CONTROLS
 * ================================================================ */

int hal_audio_set_volume(void *ctx, int dev, int chn, int vol)
{
    hisi_state_t *st = hisi_state(ctx);
    int db;

    (void)dev;
    (void)chn;

    if (!st)
        return RSS_ERR_INVAL;

    /* rad's [-30..120] with 60 = unity onto the codec's dB [-78..80]. */
    db = vol - 60;
    if (db < V5_ACODEC_INPUT_VOL_MIN)
        db = V5_ACODEC_INPUT_VOL_MIN;
    if (db > V5_ACODEC_INPUT_VOL_MAX)
        db = V5_ACODEC_INPUT_VOL_MAX;

    /* No cached copy: the setter only gets this far when /dev/acodec is
     * open, which is exactly when the getter can read the real value
     * back, so a cache could never answer a question the codec could not.
     */
    return hisi_acodec_ioctl(st, V5_ACODEC_SET_INPUT_VOL, &db, "set input volume");
}

int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol)
{
    hisi_state_t *st = hisi_state(ctx);
    int db = 0;
    int ret;

    (void)dev;
    (void)chn;

    if (!st || !vol)
        return RSS_ERR_INVAL;

    ret = hisi_acodec_ioctl(st, V5_ACODEC_GET_INPUT_VOL, &db, "get input volume");
    if (ret)
        return ret;

    *vol = db + 60;
    return RSS_OK;
}

int hal_audio_set_gain(void *ctx, int dev, int chn, int gain)
{
    hisi_state_t *st = hisi_state(ctx);
    unsigned int g;
    int ret;

    (void)dev;
    (void)chn;

    if (!st)
        return RSS_ERR_INVAL;
    if (gain < 0)
        gain = 0;
    if (gain > V5_ACODEC_GAIN_MIC_MAX) {
        if (!st->aud_gain_clamped) {
            st->aud_gain_clamped = true;
            HAL_LOG_WARN("audio: mic gain %d exceeds the inner codec's max %d -- clamping", gain,
                         V5_ACODEC_GAIN_MIC_MAX);
        }
        gain = V5_ACODEC_GAIN_MIC_MAX;
    }
    g = (unsigned int)gain;

    /* Both mics: the inner codec is one stereo front end and rad's gain
     * is one number. */
    ret = hisi_acodec_ioctl(st, V5_ACODEC_SET_GAIN_MICL, &g, "set mic gain L");
    if (ret)
        return ret;
    hisi_acodec_ioctl(st, V5_ACODEC_SET_GAIN_MICR, &g, "set mic gain R");

    /* Not cached, for the same reason as the volume. */
    return RSS_OK;
}

int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain)
{
    hisi_state_t *st = hisi_state(ctx);
    unsigned int g = 0;
    int ret;

    (void)dev;
    (void)chn;

    if (!st || !gain)
        return RSS_ERR_INVAL;

    ret = hisi_acodec_ioctl(st, V5_ACODEC_GET_GAIN_MICL, &g, "get mic gain");
    if (ret)
        return ret;

    *gain = (int)g;
    return RSS_OK;
}

int hal_audio_set_mute(void *ctx, int dev, int chn, int mute)
{
    hisi_state_t *st = hisi_state(ctx);
    unsigned int m = mute ? 1 : 0;
    int ret;

    (void)dev;
    (void)chn;

    if (!st)
        return RSS_ERR_INVAL;

    ret = hisi_acodec_ioctl(st, V5_ACODEC_SET_MICL_MUTE, &m, "set mic mute L");
    if (ret)
        return ret;
    hisi_acodec_ioctl(st, V5_ACODEC_SET_MICR_MUTE, &m, "set mic mute R");

    return RSS_OK;
}
