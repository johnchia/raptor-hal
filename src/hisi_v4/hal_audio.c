/*
 * hisi_v4/hal_audio.c -- AI audio capture for HiMPP gen4
 *
 * THE POINT OF THIS FILE
 *
 * Raw PCM in, nothing else. rad owns encoding (it ships G.711, L16, Opus
 * and AAC in software), so all this has to do is bring up the AI device
 * and the inner codec, hand out frames, and stay out of the way. The
 * vendor's own sample splits the work the same way: MPI for the digital
 * path, /dev/acodec ioctls for everything analog.
 *
 * WHY audio_init DOES THE WORK hal_init NORMALLY DOES
 *
 * rad calls rss_hal_create and then audio_init directly; it never calls
 * the init op. So this file loads the vendor libraries and attaches to
 * MPP itself when nothing has -- trampoline check first, for the same
 * eager-__ctype_b reason hal_init runs it first: libsecurec.so arrives
 * as a DT_NEEDED of libmpi.so, and without the check a NULL __ctype_b
 * faults inside the first securec routine that classifies a character.
 *
 * ONE MPP CONSUMER PER SYSTEM -- what this file may and may not touch
 *
 * SYS and VB state are kernel-global on HiMPP, and gen4's hal_init runs
 * teardown-first plus VbSetConfig+VbInit. audio_init does none of that:
 * it calls HI_MPI_SYS_Init only (the per-process attach AI needs) and
 * never configures VB, never runs a reclaim, and -- deliberately --
 * never calls HI_MPI_SYS_Exit either, because rvd may be streaming in
 * another process and a global exit is not this archive's to issue. The
 * process's own exit is the real detach. aud_owns_sys records the
 * attach for the log, not for an undo.
 *
 * A consequence to know: if rvd (or another MPP owner) has never run
 * since boot, HI_MPI_SYS_Init here may fail with SYS_NOTREADY because
 * VB was never configured. rad on this backend expects to run beside
 * rvd; the error message says as much when it happens.
 *
 * MONO ONLY, AND WHY THAT IS NOT A SHORTCUT
 *
 * audio_init takes one channel and refuses anything else with
 * RSS_ERR_NOTSUP. HiMPP hands a stereo AI frame back as two planes --
 * AUDIO_FRAME_S.u64VirAddr[0] and [1], with u32Len counting bytes *per
 * channel* -- while rss_audio_frame_t is one pointer and one length, so
 * publishing plane 0 alone would quietly deliver the left channel and
 * call it the stream. The alternative, interleaving the two planes on
 * every read, needs a scratch buffer and a per-frame copy the vendor
 * frame has no room for. It would be paying for a path the hardware does
 * not have: the EV300's inner codec is a single mic front end, and rad
 * asks for mono. If an external stereo codec ever turns up, the honest
 * change is a real interleave here plus the buffer to do it in -- not
 * relaxing the check.
 *
 * OP COVERAGE
 *
 * Implemented: audio_init, audio_deinit, audio_read_frame,
 * audio_release_frame, audio_set_volume, audio_get_volume,
 * audio_set_gain, audio_get_gain, audio_set_mute.
 *
 * The analog controls map onto /dev/acodec:
 *
 *   volume  ACODEC_SET_INPUT_VOL, which the codec takes in dB [-87..86]
 *           (-87 = mute). rad's scale is [-30..120] with 60 = unity, so
 *           the mapping is dB = vol - 60, clamped -- unity in, 0 dB out.
 *   gain    ACODEC_SET_GAIN_MIC{L,R}, the mic preamp steps, usable
 *           range [0..15] -- see V4_ACODEC_GAIN_MIC_MAX, where the
 *           measurement behind that number is written down. The driver
 *           itself would take 16, so clamping to what it accepts is the
 *           wrong rule; 16 all but mutes the mic. rad's default gain is
 *           25, so the request is clamped rather than passed through,
 *           and the clamp is logged once so a config asking for more is
 *           not silently ignored.
 *   mute    ACODEC_SET_MIC{L,R}_MUTE.
 *
 * Everything else stays NULL in the vtable -> RSS_ERR_NOTSUP. Per op:
 *
 *   audio_set_alc_gain / get_alc_gain
 *       ALC is an AGC feature; see the VQE paragraph.
 *
 *   audio_enable_ns/hpf/agc/aec and friends, audio_set_aec_profile_path,
 *   audio_get_frame_and_ref
 *       gen4 does have VQE -- HI_MPI_AI_SetTalkVqeAttr and the algorithm
 *       libraries are on the board and libmpi hard-links against them --
 *       but the configuration structs (AI_TALKVQE_CONFIG_S and its
 *       per-algorithm blocks) are a few hundred bytes of chip-specific
 *       layout each, an ABI promise nothing has paid for yet. The core
 *       capture path does not need them; they can land as their own
 *       change with their own probe numbers when something asks.
 *
 *   audio_register_encoder / unregister_encoder
 *       AENC is unused: rad encodes in software, so the hardware encoder
 *       ABI is not even loaded.
 *
 *   audio_get_chn_param
 *       Takes an opaque void* callers assume is Ingenic's IMPAudioIOAttr;
 *       there is no honest way to fill that from AI_CHN_PARAM_S.
 *
 *   the ao_* family
 *       Playback. Capture-only by design, same as the SigmaStar backends.
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

/* The userspace frame queue (AI_CHN_PARAM_S.u32UsrFrmDepth). The slack
 * between rad's reads on a loaded system: SigmaStar measured ~80 ms of
 * scheduling delay against VENC and the ISP and needed ~320 ms of queue;
 * sixteen 20 ms periods is that same amount here. Config wins when it
 * says something. */
#define HISI_AUD_DEPTH_DEFAULT 16

static bool hisi_audio_rate_ok(int rate)
{
    return v4_acodec_fs(rate) >= 0;
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
    unsigned int fs = (unsigned int)v4_acodec_fs(rate);
    unsigned int mixer = V4_ACODEC_MIXER_IN1;

    st->acodec_fd = open(V4_ACODEC_PATH, O_RDWR);
    if (st->acodec_fd < 0) {
        /* An external-codec board has no /dev/acodec; capture can still
         * work if something else configured the codec, so this degrades
         * the analog controls instead of failing the device. */
        HAL_LOG_WARN("acodec: %s: %s -- volume/gain/mute unavailable", V4_ACODEC_PATH,
                     strerror(errno));
        return RSS_OK;
    }

    if (ioctl(st->acodec_fd, V4_ACODEC_SOFT_RESET))
        HAL_LOG_WARN("acodec: soft reset failed: %s", strerror(errno));
    hisi_acodec_ioctl(st, V4_ACODEC_SET_I2S1_FS, &fs, "set I2S FS");
    hisi_acodec_ioctl(st, V4_ACODEC_SET_MIXER_MIC, &mixer, "select mic input");
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
            HAL_LOG_WARN("HI_MPI_AI_DisableChn failed: 0x%x", ret);
        st->aud_chn_enabled = false;
    }
    if (st->aud_dev_enabled) {
        int ret = st->aud.fnDisable(st->aud_dev);
        if (ret)
            HAL_LOG_WARN("HI_MPI_AI_Disable failed: 0x%x", ret);
        st->aud_dev_enabled = false;
    }
}

int hal_audio_init(void *ctx, const rss_audio_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st;
    v4_aio_attr attr;
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
        ret = v4_sys_load(&st->sys, &st->libs);
        if (ret) {
            hisi_mpi_close(&st->libs);
            free(st);
            return ret;
        }

        ret = st->sys.fnInit();
        if (ret) {
            HAL_LOG_ERR("HI_MPI_SYS_Init failed: 0x%x -- on this backend audio expects "
                        "rvd (the MPP owner) to be up first; see hal_audio.c",
                        ret);
            v4_sys_unload(&st->sys);
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
        ret = v4_aud_load(&st->aud, &st->libs);
        if (ret)
            return ret;
        st->aud_loaded = true;
    } else {
        /* A rate change on a running device: stop before reconfiguring;
         * SetPubAttr describes a device that is not running. */
        hisi_audio_stop(st);
        if (st->acodec_fd >= 0) {
            close(st->acodec_fd);
            st->acodec_fd = -1;
        }
    }

    st->aud_dev = 0;
    st->aud_rate = (int)cfg->sample_rate;

    /* Samples per frame per channel; one 20 ms period unless the config
     * says otherwise, which is also rad's own slicing. */
    samples = cfg->samples_per_frame > 0 ? (unsigned int)cfg->samples_per_frame
                                         : (unsigned int)cfg->sample_rate / 50;
    if (samples == 0 || samples > (unsigned int)cfg->sample_rate) {
        unsigned int fallback = (unsigned int)cfg->sample_rate / 50;

        HAL_LOG_WARN("audio: %u samples/frame is out of range; using %u", samples, fallback);
        samples = fallback;
    }

    /* The analog codec first: the I2S clocks it consumes are set up by
     * the FS select, and the vendor sample orders it the same way. */
    hisi_acodec_setup(st, st->aud_rate);

    /*
     * I2S master against the inner codec, 16-bit mono. The config carries
     * no bus-role or codec-type fields, so those are defaults rather than
     * policy: what the vendor sample uses and what the inner codec
     * requires. Rate, period and depth come from the config; the channel
     * count does not -- it is fixed at one, checked above.
     */
    memset(&attr, 0, sizeof(attr));
    attr.sample_rate = st->aud_rate;
    attr.bit_width = V4_AUD_BIT_WIDTH_16;
    attr.work_mode = V4_AUD_MODE_I2S_MASTER;
    attr.sound_mode = V4_AUD_SOUND_MONO;
    attr.ex_flag = 0;
    attr.frm_num = 30; /* device-side ring, frames; the sample's number */
    attr.pt_num_per_frm = samples;
    attr.chn_cnt = 1;
    attr.clk_sel = 0;
    attr.i2s_type = V4_AUD_I2S_INNERCODEC;

    ret = st->aud.fnSetPubAttr(st->aud_dev, &attr);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_AI_SetPubAttr(%d) failed: 0x%x", st->aud_dev, ret);
        return RSS_ERR_IO;
    }

    ret = st->aud.fnEnable(st->aud_dev);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_AI_Enable(%d) failed: 0x%x", st->aud_dev, ret);
        return RSS_ERR_IO;
    }
    st->aud_dev_enabled = true;

    ret = st->aud.fnEnableChn(st->aud_dev, 0);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_AI_EnableChn(%d, 0) failed: 0x%x", st->aud_dev, ret);
        hisi_audio_stop(st);
        return RSS_ERR_IO;
    }
    st->aud_chn_enabled = true;

    /* The userspace queue. Without it GetFrame still works on gen4 --
     * unlike MI -- but the default depth is shallow; deepen it so a
     * scheduling stall costs latency rather than samples. */
    if (st->aud.fnSetChnParam) {
        v4_ai_chn_param param;
        int depth = cfg->frame_depth > 0 ? cfg->frame_depth : HISI_AUD_DEPTH_DEFAULT;

        if (depth < 2)
            depth = 2;
        if (depth > 50)
            depth = 50;
        param.usr_frm_depth = (unsigned int)depth;
        ret = st->aud.fnSetChnParam(st->aud_dev, 0, &param);
        if (ret)
            HAL_LOG_WARN("HI_MPI_AI_SetChnParam(depth %d) failed: 0x%x -- capture continues "
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
        hisi_audio_stop(st);
        v4_aud_unload(&st->aud);
        st->aud_loaded = false;
    }
    if (st->acodec_fd >= 0) {
        close(st->acodec_fd);
        st->acodec_fd = -1;
    }
    /* No HI_MPI_SYS_Exit, deliberately -- see the file comment. */
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
         * fetch. NOBUF likewise: the queue exists (SetChnParam made it),
         * it just has nothing yet. */
        if ((unsigned int)ret == V4_ERR_AI_BUF_EMPTY || (unsigned int)ret == V4_ERR_AI_NOBUF)
            return RSS_ERR_TIMEOUT;

        if (ret != st->aud_last_err) {
            HAL_LOG_WARN("HI_MPI_AI_GetFrame failed: 0x%x", ret);
            st->aud_last_err = ret;
        }
        return RSS_ERR_IO;
    }
    st->aud_last_err = 0;

    if (!st->aud_frame.vir_addr[0] || !st->aud_frame.len) {
        /* A success with nothing in it; give it straight back. */
        st->aud.fnReleaseFrame(st->aud_dev, 0, &st->aud_frame, &st->aud_aec);
        return RSS_ERR_TIMEOUT;
    }

    st->aud_frame_held = true;

    frame->data = (const int16_t *)st->aud_frame.vir_addr[0];
    frame->length = st->aud_frame.len;
    /* HiMPP's timestamp, microseconds. rad replaces it with a synthetic
     * clock of its own, but reporting what the driver said is still the
     * honest thing for anything that looks. */
    frame->timestamp = (int64_t)st->aud_frame.timestamp;
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
        HAL_LOG_WARN("HI_MPI_AI_ReleaseFrame failed: 0x%x", ret);
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

    /* rad's [-30..120] with 60 = unity onto the codec's dB [-87..86]. */
    db = vol - 60;
    if (db < -87)
        db = -87;
    if (db > 86)
        db = 86;

    /* No cached copy: the setter only gets this far when /dev/acodec is
     * open, which is exactly when the getter can read the real value
     * back, so a cache could never answer a question the codec could not.
     */
    return hisi_acodec_ioctl(st, V4_ACODEC_SET_INPUT_VOL, &db, "set input volume");
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

    ret = hisi_acodec_ioctl(st, V4_ACODEC_GET_INPUT_VOL, &db, "get input volume");
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
    if (gain > V4_ACODEC_GAIN_MIC_MAX) {
        if (!st->aud_gain_clamped) {
            st->aud_gain_clamped = true;
            HAL_LOG_WARN("audio: mic gain %d exceeds the inner codec's max %d -- clamping",
                         gain, V4_ACODEC_GAIN_MIC_MAX);
        }
        gain = V4_ACODEC_GAIN_MIC_MAX;
    }
    g = (unsigned int)gain;

    /* Both mics: the inner codec is one stereo front end and rad's gain
     * is one number. */
    ret = hisi_acodec_ioctl(st, V4_ACODEC_SET_GAIN_MICL, &g, "set mic gain L");
    if (ret)
        return ret;
    hisi_acodec_ioctl(st, V4_ACODEC_SET_GAIN_MICR, &g, "set mic gain R");

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

    ret = hisi_acodec_ioctl(st, V4_ACODEC_GET_GAIN_MICL, &g, "get mic gain");
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

    ret = hisi_acodec_ioctl(st, V4_ACODEC_SET_MICL_MUTE, &m, "set mic mute L");
    if (ret)
        return ret;
    hisi_acodec_ioctl(st, V4_ACODEC_SET_MICR_MUTE, &m, "set mic mute R");

    return RSS_OK;
}
