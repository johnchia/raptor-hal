/*
 * star/hal_audio.c -- MI_AI audio capture for SigmaStar Infinity6E
 *
 * THE POINT OF THIS FILE
 *
 * Raw PCM in, nothing else. rad owns encoding (it ships G.711, L16, Opus
 * and AAC in software), so all this has to do is bring up the AI device,
 * hand out frames, and stay out of the way. That is also what both
 * references do -- neither ever calls MI's own audio encoder, and
 * waybeam's capture thread is documented "no encode".
 *
 * WHY audio_init DOES THE WORK hal_init NORMALLY DOES
 *
 * rad calls rss_hal_create and then audio_init directly; it never calls
 * the init op, because on Ingenic the audio device needs no system-wide
 * bring-up. On MI it does -- libmi_ai.so has undefined MI_SYS_* symbols
 * and the vendor's own AI example opens with MI_SYS_Init() -- so
 * audio_init loads libmi_sys, initialises it, and records that it was the
 * one who did (aud_owns_sys) so deinit can undo exactly that much. When
 * something *has* called hal_init first, the existing state is reused and
 * MI_SYS is left alone.
 *
 * A consequence worth knowing: rvd and rad are separate processes, so
 * both call MI_SYS_Init in their own address space. That is what MI's
 * multi-process design intends, but this port has only ever been run
 * single-process, so it is the phase's main untested assumption.
 *
 * OP COVERAGE
 *
 * Implemented: audio_init, audio_deinit, audio_read_frame,
 * audio_release_frame, audio_set_volume, audio_get_volume,
 * audio_set_gain, audio_get_gain, audio_set_mute.
 *
 * Everything else is left NULL in the vtable, which RSS_HAL_CALL turns
 * into RSS_ERR_NOTSUP. Per op, why:
 *
 *   audio_set_alc_gain / get_alc_gain
 *       Automatic level control is an AGC feature, and AGC is one of the
 *       absent VQE algorithms below.
 *
 *       (audio_set_gain / get_gain used to be listed here, on the grounds
 *       that MI had exactly one input-gain control and audio_set_volume
 *       already owned it. That was wrong. MI_AI_SetChnParam reaches a
 *       second, independent stage -- see i6_aud_chn_para -- so the two ops
 *       no longer fight over one register: gain drives the analog front end
 *       through MI_AI_SetVqeVolume, volume drives the digital trim through
 *       MI_AI_SetChnParam. rad calls both at startup, in that order, and
 *       now both take effect.
 *
 *       Note which op got which stage: gain is the preamp and volume is the
 *       level control, matching the Infinity6C backend. This file used to
 *       have volume on the analog stage, because that was the only stage it
 *       could reach; see star_state.h for why the swap costs a default
 *       install nothing.)
 *
 *   audio_enable_ns / disable_ns, audio_enable_hpf / disable_hpf,
 *   audio_enable_agc / disable_agc, audio_set_agc_mode,
 *   audio_set_hpf_co_freq, audio_enable_aec / disable_aec,
 *   audio_enable_aec_ref_frame / disable_aec_ref_frame,
 *   audio_set_aec_profile_path, audio_get_frame_and_ref
 *       These are MI's VQE features (noise reduction, high-pass filter,
 *       automatic gain control, echo cancellation). The entry points are
 *       present in libmi_ai.so but the algorithm packs behind them are
 *       **weak undefined** -- IaaXxx_*, MI_AED_* and friends resolve to
 *       NULL rather than failing the link, which is exactly how SigmaStar
 *       intended the library to work without the optional packs. Calling
 *       one would jump to address 0. MI's own documentation confirms the
 *       direction: "In 2.19 and later versions of the API, MI_AI no
 *       longer includes the associated algorithm functions."
 *
 *   audio_register_encoder / unregister_encoder
 *       For SoCs with a hardware audio encoder raptor can drive. MI's
 *       (MI_AI_SetAencAttr / EnableAenc) is unused here and not even
 *       loaded -- see i6_aud.h.
 *
 *   audio_get_chn_param
 *       MI_AI_GetChnParam exists, but the op passes an opaque void* whose
 *       layout callers assume is Ingenic's IMPAudioIOAttr. There is no
 *       honest way to fill that from MI's MI_AI_ChnParam_t.
 *
 *   the ao_* family
 *       Playback. Nothing in scope plays audio out, libmi_ao is never
 *       loaded, and this backend is capture-only by design.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "star_state.h"

#include <stdlib.h>
#include <string.h>

/*
 * No HAL_MODULE_AUDIO guard here, deliberately -- this file is only ever
 * compiled into the audio archive (Makefile's AUDIO_SRCS), so a guard
 * would be redundant at best. It is also the trap hal_isp.c documents in
 * reverse: -DHAL_MODULE_AUDIO reaches only src/hal_common_audio.o, so a
 * guard would compile the whole translation unit away and the failure
 * would surface as undefined vtable symbols at link time.
 */

/* MI's supported capture rates. Anything else has to go through the
 * resampler, whose implementation (IaaSrc_*) is one of the absent
 * algorithm packs, so an unsupported rate is refused rather than
 * resampled into a NULL call. */
static bool star_audio_rate_ok(int rate)
{
    return rate == 8000 || rate == 16000 || rate == 32000 || rate == 48000;
}

/*
 * MI_AI error codes, from the reference's error-code table.
 *
 * Here because a bare %#x cost a board cycle: 0xA004200D differs from the
 * expected-and-harmless 0xA004200E by one digit, and reads as noise until
 * someone looks it up. Anything absent from the table is reported as its
 * hex alone, which is still the truth.
 */
static const char *star_audio_err_name(int err)
{
    switch ((unsigned int)err) {
    case 0xA0042001u: return "INVALID_DEVID";
    case 0xA0042002u: return "INVALID_CHNID";
    case 0xA0042003u: return "ILLEGAL_PARAM";
    case 0xA0042006u: return "NULL_PTR";
    case 0xA0042007u: return "NOT_CONFIG";
    case 0xA0042008u: return "NOT_SUPPORT";
    case 0xA0042009u: return "NOT_PERM";
    case 0xA004200Cu: return "NOMEM";
    /* Missing output-port queue -- see STAR_AUD_PORT_USR_DEPTH. */
    case STAR_AUD_ERR_NOBUF: return "NOBUF";
    case STAR_AUD_ERR_BUF_EMPTY: return "BUF_EMPTY";
    case 0xA004200Fu: return "BUF_FULL";
    case 0xA0042010u: return "SYS_NOTREADY";
    case 0xA0042012u: return "BUSY";
    case 0xA0042017u: return "NOT_ENABLED";
    case 0xA0042100u: return "VQE_ERR";
    case 0xA0042101u: return "AENC_ERR";
    default: return "unknown";
    }
}

/*
 * raptor gain (0..31) -> analog-gain table index.
 *
 * Linear across the device's whole table, so 0 is the quietest setting the
 * hardware offers and 31 the loudest. The ceilings are per input type (see
 * STAR_AUD_IF_GAIN_MAX_*), and the result is clamped: an index off the end
 * of the table earns MI_AI_ERR_ILLEGAL_PARAM, not a louder signal.
 *
 * Same shape and same rounding as the Infinity6C backend's
 * i6c_audio_gain_step, because it is the same quantity -- raptor's gain
 * scale onto a 0..21 analog step -- and the two backends should not
 * disagree about where a given number lands.
 *
 * Worth knowing when reading the log: the table spans about 60 dB, so the
 * mapping is steep. rad's default of 25 lands on step 17, high on purpose
 * because an electret mic needs the gain; if capture clips, lowering
 * `[audio] gain` is the fix rather than anything here.
 */
static int star_audio_gain_index(const star_state_t *st, int gain)
{
    int max = st->aud_input == RSS_AUDIO_INPUT_DMIC ? STAR_AUD_IF_GAIN_MAX_DMIC
                                                    : STAR_AUD_IF_GAIN_MAX_AMIC;
    int idx;

    if (gain < 0)
        gain = 0;
    else if (gain > STAR_AUD_GAIN_MAX)
        gain = STAR_AUD_GAIN_MAX;

    idx = (gain * max + STAR_AUD_GAIN_MAX / 2) / STAR_AUD_GAIN_MAX;
    if (idx > max)
        idx = max;

    return idx;
}

/*
 * raptor volume (0..100) -> MI_AI_SetChnParam's s16RearGain, in whole dB.
 *
 * Piecewise around unity rather than linear across the range, and unity is
 * rad's default of 80 on purpose: a camera whose config never mentions
 * `[audio] volume` gets 0 dB here, which is what the digital stage was
 * already doing when nothing wrote to it. Wiring this op to the digital
 * stage therefore cannot change the level of a default install.
 *
 * Below unity attenuates toward STAR_AUD_DPGA_MIN_DB, above it boosts
 * toward STAR_AUD_DPGA_MAX_DB. Both are the library's own bounds, so the
 * whole output range is accepted; nothing here can produce the
 * ILLEGAL_PARAM the analog path has to guard against.
 *
 * Note the asymmetry in resolution: 80 steps cover 60 dB of cut and 20
 * cover 30 dB of boost. That falls out of putting unity at the default
 * rather than at the midpoint, and it is the right way round -- boosting a
 * digital signal amplifies the noise floor with it, so the coarser half is
 * the one that hurts to use. Identical to i6c_audio_volume_db.
 */
static int star_audio_volume_db(int vol)
{
    if (vol < 0)
        vol = 0;
    else if (vol > 100)
        vol = 100;

    if (vol < STAR_AUD_VOL_UNITY)
        return STAR_AUD_DPGA_MIN_DB +
               (vol * -STAR_AUD_DPGA_MIN_DB + STAR_AUD_VOL_UNITY / 2) / STAR_AUD_VOL_UNITY;

    return ((vol - STAR_AUD_VOL_UNITY) * STAR_AUD_DPGA_MAX_DB + (100 - STAR_AUD_VOL_UNITY) / 2) /
           (100 - STAR_AUD_VOL_UNITY);
}

/*
 * The device rad asked for versus the one that was configured.
 *
 * audio_init takes no device argument, so it configures STAR_AUD_DEV and
 * the per-device ops can only ever be handed something else by config
 * accident -- rad's `[audio] device` defaults to 1, an Ingenic index.
 * Refusing would mean silence out of the box for a value nobody chose, so
 * the configured device wins and the mismatch is named once.
 */
static int star_audio_dev(star_state_t *st, int dev)
{
    if (dev != st->aud_dev && !st->aud_dev_warned) {
        HAL_LOG_WARN("audio: asked for device %d but device %d is the one configured; using %d. "
                     "MI's AI device selects the physical input, so set [audio] device = %d",
                     dev, st->aud_dev, st->aud_dev, st->aud_dev);
        st->aud_dev_warned = true;
    }

    return st->aud_dev;
}

static bool star_audio_chn_ok(const star_state_t *st, int chn)
{
    return chn >= 0 && chn < STAR_AUD_CHN_MAX && (unsigned int)chn < st->aud_chn_count;
}

/*
 * Bring the AI device down, undoing only what came up.
 *
 * Order is the reverse of bring-up and MI requires it: the docs state
 * that every enabled channel must be disabled before the device is.
 */
static void star_audio_teardown(star_state_t *st)
{
    int ret;
    int i;

    for (i = 0; i < STAR_AUD_CHN_MAX; i++) {
        if (!st->aud_chn_enabled[i])
            continue;

        /* Give back a frame still checked out, or MI keeps the buffer. */
        if (st->aud_frame_held[i]) {
            st->aud.fnFreeFrame(st->aud_dev, i, &st->aud_frame[i], NULL);
            st->aud_frame_held[i] = false;
        }

        ret = st->aud.fnDisableChannel(st->aud_dev, i);
        if (ret)
            HAL_LOG_WARN("MI_AI_DisableChn(%d, %d) failed: %d", st->aud_dev, i, ret);
        st->aud_chn_enabled[i] = false;
    }

    if (st->aud_dev_enabled) {
        ret = st->aud.fnDisableDevice(st->aud_dev);
        if (ret)
            HAL_LOG_WARN("MI_AI_Disable(%d) failed: %d", st->aud_dev, ret);
        st->aud_dev_enabled = false;
    }
}

/*
 * hal_audio_init -- configure and start audio capture.
 *
 * Sequence is SetPubAttr -> Enable -> EnableChn, which is what the vendor
 * documentation, divinus and waybeam all use, and the docs are explicit
 * that the device attributes must be set before Enable and the device
 * enabled before any channel.
 *
 * Re-entrant on purpose: rad calls this again to change sample rate at
 * runtime, so an already-running device is torn down first rather than
 * refused.
 */

/*
 * Give one channel's output port somewhere to put frames.
 *
 * This is not tuning and it is not optional. An MI channel's output port
 * starts with no user-side queue at all, so MI_AI_GetFrame has nothing to hand
 * back and fails with MI_AI_ERR_NOBUF (0xA004200D, "insufficient audio input
 * buffer") on every call while the device reports itself perfectly enabled --
 * -- the exact fault a board shows without it. All three sources do it: the vendor MI_AI reference's own capture example (1, 8), divinus
 * (2, 4) and waybeam (1, 2). It is also why the doc calling u32FrmNum
 * "Reserved, unused" matters here -- the device-side ring is not what feeds
 * userspace, this queue is.
 *
 * user_depth 1 because this backend structurally holds at most one frame per
 * channel: read_frame refuses a second read until the first is released.
 * Asking for more would only add the latency waybeam measured at 2.
 *
 * buf_depth is the slack between rad's reads, and 4 (~80 ms) was not enough.
 * On the board, MI reported "Buffer(s) is lost due to slow fetching" and rad
 * independently reported "audio clock resync +151ms" -- the two halves of one
 * event, where rad did not get back to GetFrame inside 80 ms and MI overwrote
 * captured periods rather than queue them. rad's loop is a plain SCHED_OTHER
 * thread competing with VENC and the ISP, so delays of that size are a
 * property of the system, not a bug to be eliminated; the queue is what
 * decides whether they cost latency or samples.
 *
 * 16 (~320 ms) covers the observed stall and still sits inside the 400 ms
 * device ring, so MI never has to choose between the two. Deepening costs
 * nothing in steady state -- GetFrame returns the oldest queued period, and
 * the queue only holds more than one when rad is behind -- and rad's clock
 * slew is already written for the resulting drain bursts (it tolerates being
 * up to 1 s ahead of the wall clock for exactly this reason).
 *
 * Factored out of init because read_frame re-applies it: see the NOBUF
 * recovery there.
 */
static int star_audio_set_port_depth(star_state_t *st, int chn)
{
    i6_sys_bind port;
    int ret;

    memset(&port, 0, sizeof(port));
    port.module = I6_SYS_MOD_AI;
    port.device = (unsigned int)st->aud_dev;
    port.channel = (unsigned int)chn;
    port.port = 0;

    ret = st->sys.fnSetOutputDepth(&port, STAR_AUD_PORT_USR_DEPTH, STAR_AUD_PORT_BUF_DEPTH);
    if (!ret)
        return 0;

    /*
     * A depth MI will not accept must not cost us audio entirely: init treats
     * a failure here as fatal, and every reference source uses a shallower
     * queue than this, so retry at the depth this board is known to accept.
     */
    HAL_LOG_WARN("audio: chn %d rejected output port depth (%d, %d): %d -- retrying at (%d, %d)",
                 chn, STAR_AUD_PORT_USR_DEPTH, STAR_AUD_PORT_BUF_DEPTH, ret,
                 STAR_AUD_PORT_USR_DEPTH, STAR_AUD_PORT_BUF_DEPTH_FALLBACK);

    return st->sys.fnSetOutputDepth(&port, STAR_AUD_PORT_USR_DEPTH,
                                    STAR_AUD_PORT_BUF_DEPTH_FALLBACK);
}

int hal_audio_init(void *ctx, const rss_audio_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    star_state_t *st;
    i6_aud_cnf dev_cfg;
    unsigned int chn_count;
    unsigned int samples;
    int ret;
    unsigned int i;

    if (!c || !cfg)
        return RSS_ERR_INVAL;

    if (!star_audio_rate_ok((int)cfg->sample_rate)) {
        HAL_LOG_ERR("audio: %d Hz is not an MI capture rate -- only 8000, 16000, 32000 and "
                    "48000 are supported, and MI's resampler is not present on this platform",
                    (int)cfg->sample_rate);
        return RSS_ERR_INVAL;
    }

    /* PCM channels in a captured frame, not MI channels -- see the
     * dev_cfg.chnNum assignment below for why those are not the same
     * number. rss_audio_config_t documents this field as 1=mono, 2=stereo,
     * so two is the ceiling rather than STAR_AUD_CHN_MAX. */
    chn_count = cfg->chn_count > 0 ? (unsigned int)cfg->chn_count : 1;
    if (chn_count > 2) {
        HAL_LOG_WARN("audio: %u PCM channels requested, capping at 2 (stereo)", chn_count);
        chn_count = 2;
    }

    /*
     * State and MI_SYS. Present already when something called hal_init;
     * created here when nothing did, which is rad's normal path.
     */
    st = star_state(ctx);
    if (!st) {
        st = (star_state_t *)calloc(1, sizeof(*st));
        if (!st)
            return RSS_ERR_NOMEM;

        ret = i6_sys_load(&st->sys);
        if (ret) {
            free(st);
            return ret;
        }

        ret = st->sys.fnInit();
        if (ret) {
            HAL_LOG_ERR("MI_SYS_Init failed: %d", ret);
            i6_sys_unload(&st->sys);
            free(st);
            return RSS_ERR_IO;
        }

        st->sys_inited = true;
        st->aud_owns_sys = true;
        c->platform = st;
    }

    if (!st->aud_loaded) {
        ret = i6_aud_load(&st->aud);
        if (ret) {
            /* i6_aud_load logs which symbol or library was missing. */
            return ret;
        }
        st->aud_loaded = true;
    } else {
        /* A rate change on a running device: stop before reconfiguring,
         * since MI_AI_SetPubAttr describes a device that is not running. */
        star_audio_teardown(st);
    }

    st->aud_dev = STAR_AUD_DEV;
    /*
     * One MI channel, always. MI puts a stereo pair inside a single
     * channel's frame -- interleaved in addr[0], with length covering both
     * -- so the PCM width is carried by the sound mode, not by the channel
     * count. See dev_cfg.chnNum below.
     */
    st->aud_chn_count = 1;
    st->aud_pcm_chn = chn_count;
    st->aud_rate = (int)cfg->sample_rate;
    st->aud_input = cfg->input_type;

    /* Samples per frame per channel. The config wins when it says
     * something; otherwise one 20 ms period, which is both waybeam's
     * choice and what the vendor example uses (160 at 8 kHz). rad slices
     * its reads into 20 ms chunks anyway, so matching that keeps one
     * GetFrame to one encoded packet. */
    samples = cfg->samples_per_frame > 0 ? (unsigned int)cfg->samples_per_frame
                                         : (unsigned int)cfg->sample_rate / 50;

    /*
     * MI's own bound on this field, which it enforces rather than clamps:
     * libmi_ai.so refuses packNumPerFrm of 0 or greater than the sample
     * rate with ILLEGAL_PARAM, i.e. a config typo would cost all audio
     * instead of one odd period length. The rate above is screened this
     * carefully already; this is the same courtesy for the field beside it.
     */
    if (samples == 0 || samples > (unsigned int)cfg->sample_rate) {
        unsigned int fallback = (unsigned int)cfg->sample_rate / 50;

        HAL_LOG_WARN("audio: %u samples/frame is outside MI's range (1..%d); using %u",
                     samples, (int)cfg->sample_rate, fallback);
        samples = fallback;
    }

    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.rate = (int)cfg->sample_rate;
    dev_cfg.bit24On = 0;
    /*
     * I2S_SLAVE, and this is a hardware finding rather than a documented
     * one. The sources genuinely disagree: divinus uses slave, waybeam
     * master, and the SSC30KQ SDK's own audio sample uses
     * E_MI_AUDIO_MODE_I2S_MASTER. libmi_ai.so accepts either -- it range
     * checks this field at [0,3] and nothing more -- so neither is refused
     * and a wrong choice shows up as silence, not as an error.
     *
     * Slave stays because it is what this board is known to capture with,
     * which outranks a sample written for a demo carrier. (An earlier
     * version of this comment claimed the vendor documentation backed
     * slave; that referred to the SSD20X reference, and the SDK for this
     * SoC does not agree. The value is unchanged, the justification is.)
     * If capture ever comes back silent with the device happily enabled,
     * I6_AUD_INTF_I2S_MASTER is the one-line thing to try.
     */
    dev_cfg.intf = I6_AUD_INTF_I2S_SLAVE;
    dev_cfg.sound = chn_count >= 2 ? I6_AUD_SND_STEREO : I6_AUD_SND_MONO;
    /*
     * Not a ring depth, whatever the name suggests. The vendor's own sample
     * marks u32FrmNum "useless", and libmi_ai.so bears that out: it is the
     * one attribute field the library never reads, validates or acts on --
     * it is copied to the driver and nothing more. What actually decides
     * how much slack a slow reader gets is the MI_SYS output port queue set
     * below, so `[audio] frame_depth` is not the knob for that and the
     * bring-up log deliberately no longer advertises it as one.
     *
     * Still forwarded rather than dropped: it costs nothing, and if a
     * future firmware starts honouring it, waybeam's 20 (~400 ms at 20 ms
     * periods) is the value with reasoning behind it.
     */
    dev_cfg.frmNum = cfg->frame_depth > 0 ? (unsigned int)cfg->frame_depth : 20;
    dev_cfg.packNumPerFrm = samples;
    dev_cfg.codecChnNum = 0;
    /*
     * One MI channel carrying chn_count PCM channels -- NOT chn_count MI
     * channels. u32ChnCnt is how many independent capture channels the
     * device offers (the library uses it to bound the valid channel index),
     * while eSoundmode above is the frame format of each. Setting both from
     * the same number asked for two channels of stereo, i.e. four PCM
     * channels, when the caller meant one stereo stream.
     */
    dev_cfg.chnNum = st->aud_chn_count;
    /* MCLK off: the vendor's own audio sample leaves it disabled and lets
     * the interface derive its clock from the rate. */
    dev_cfg.i2s.clock = I6_AUD_CLK_OFF;
    /*
     * syncRxClkOn stays zero, and that is a decision rather than an
     * oversight -- the vendor sample sets bSyncClock = TRUE, so this is a
     * deliberate divergence from the one configuration it exercises.
     *
     * "I2s Tx BCLK, Rx BCLK use the same clock source" only means something
     * when there is a Tx side, and the sample is full duplex: it brings up
     * MI_AO alongside MI_AI. Nothing in this process opens MI_AO at all, so
     * pointing the receive clock at a transmit clock that was never started
     * has no upside here and one plausible downside -- silence from a
     * device that still reports itself enabled, which is among the harder
     * faults to read. Left alone until there is a board to try it on.
     */

    ret = st->aud.fnSetDeviceConfig(st->aud_dev, &dev_cfg);
    if (ret) {
        HAL_LOG_ERR("MI_AI_SetPubAttr(%d) failed: %d (%d Hz, %u PCM ch in %u MI ch, "
                    "%u samples/frame)",
                    st->aud_dev, ret, dev_cfg.rate, st->aud_pcm_chn, dev_cfg.chnNum, samples);
        return RSS_ERR_IO;
    }

    ret = st->aud.fnEnableDevice(st->aud_dev);
    if (ret) {
        HAL_LOG_ERR("MI_AI_Enable(%d) failed: %d", st->aud_dev, ret);
        return RSS_ERR_IO;
    }
    st->aud_dev_enabled = true;

    for (i = 0; i < st->aud_chn_count; i++) {
        ret = st->aud.fnEnableChannel(st->aud_dev, (int)i);
        if (ret) {
            HAL_LOG_ERR("MI_AI_EnableChn(%d, %u) failed: %d", st->aud_dev, i, ret);
            star_audio_teardown(st);
            return RSS_ERR_IO;
        }
        st->aud_chn_enabled[i] = true;

        ret = star_audio_set_port_depth(st, (int)i);
        if (ret) {
            HAL_LOG_ERR("MI_SYS_SetChnOutputPortDepth(AI %d, %u) failed: %d -- GetFrame would "
                        "return NOBUF forever",
                        st->aud_dev, i, ret);
            star_audio_teardown(st);
            return RSS_ERR_IO;
        }
    }

    HAL_LOG_INFO("audio: AI device %d up, %d Hz %s, %u samples/frame", st->aud_dev, dev_cfg.rate,
                 st->aud_pcm_chn >= 2 ? "stereo" : "mono", samples);

    /*
     * Seed the remembered analog step from whatever the driver came up with,
     * so the first hal_audio_set_volume has a real front gain to preserve
     * rather than a guess. rad sets the volume before the gain, so without
     * this that first call is the one with nothing to go on.
     */
    st->aud_front_idx = -1;
    if (st->aud.fnGetChannelParam) {
        i6_aud_chn_para para;

        memset(&para, 0, sizeof(para));
        if (!st->aud.fnGetChannelParam(st->aud_dev, 0, &para))
            st->aud_front_idx = para.gain.front;
    }

    /* Said once, at INFO, because it changes which ops work rather than
     * being a fault: a library without this keeps capturing at full quality
     * and only loses the digital trim. */
    if (!st->aud.fnSetChannelParam)
        HAL_LOG_INFO("audio: libmi_ai.so has no MI_AI_SetChnParam; volume will report "
                     "unsupported and gain is the only level control");

    /* Volume and gain are not set here: rad sets both right after init, and
     * picking a default would mean overriding whatever the codec came up
     * with for no reason. */
    return RSS_OK;
}

int hal_audio_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    star_state_t *st = star_state(ctx);

    if (!st)
        return RSS_OK;

    if (st->aud_loaded) {
        star_audio_teardown(st);
        i6_aud_unload(&st->aud);
        st->aud_loaded = false;
    }

    /*
     * Only unwind MI_SYS if audio_init was what brought it up. When
     * hal_init did, the video half of the pipeline is still using it and
     * this op has no business tearing it down -- hal_deinit will.
     */
    if (st->aud_owns_sys) {
        if (st->sys_inited) {
            st->sys.fnExit();
            st->sys_inited = false;
        }
        i6_sys_unload(&st->sys);
        st->aud_owns_sys = false;
        if (c)
            c->platform = NULL;
        free(st);
    }

    return RSS_OK;
}

/*
 * hal_audio_read_frame -- one captured period.
 *
 * The buffer belongs to MI until release_frame hands it back, so nothing
 * is copied: frame->data points into MI's DMA buffer and _priv carries
 * the descriptor MI needs to see again.
 *
 * A timeout is not an error. MI reports "no data yet" as a distinct
 * condition (MI_AI_ERR_BUF_EMPTY) and rad's loop treats RSS_ERR_TIMEOUT
 * as "try again", so a device that is merely idle stays quiet in the log.
 * Any *other* failure is named once per distinct code -- a real fault
 * (device not enabled, say) otherwise repeats at the capture period and
 * buries everything else.
 */
int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block)
{
    star_state_t *st = star_state(ctx);
    i6_aud_frm *slot;
    int ret;

    if (!st || !frame)
        return RSS_ERR_INVAL;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    dev = star_audio_dev(st, dev);
    if (!star_audio_chn_ok(st, chn))
        return RSS_ERR_INVAL;
    if (st->aud_frame_held[chn]) {
        HAL_LOG_WARN("audio: chn %d already holds a frame; release it before reading again", chn);
        return RSS_ERR_BUSY;
    }

    slot = &st->aud_frame[chn];
    memset(slot, 0, sizeof(*slot));

    ret = st->aud.fnGetFrame(dev, chn, slot, NULL, block ? STAR_AUD_GET_TIMEOUT_MS : 0);

    /*
     * On a NON-blocking fetch, NOBUF means only "nothing queued right now".
     *
     * MI_AI overloads the code: 0xA004200D is both "this port has no user-side
     * queue" and the empty-queue answer when s32MilliSec is 0. The vendor docs
     * for VENC and VDEC say an unblocked fetch reports BUF_EMPTY, and that is
     * simply not what MI_AI does. Treating it as a lost queue re-applies the
     * output port depth -- which FLUSHES the queue -- on every empty poll, and
     * MI then logs "Buffer(s) is lost" about five periods at a time,
     * continuously.
     *
     * Nothing is given up by returning early here. If the port really has lost
     * its queue, the caller's next *blocking* fetch reports NOBUF too, and the
     * recovery below runs then.
     */
    if ((unsigned int)ret == STAR_AUD_ERR_NOBUF && !block)
        return RSS_ERR_TIMEOUT;

    /*
     * On a blocking fetch, NOBUF means what it says -- this port has no
     * user-side queue -- so the one useful response is to establish it again
     * and retry, rather than report a read error for the life of the process.
     *
     * It is worth doing even though init already set the depth successfully,
     * because the board shows NOBUF when rad starts during boot and not when
     * rad is started by hand a little later against the very same
     * configuration. Something is dropping the queue between init and the
     * first read; until that is identified, re-establishing it is both the
     * correct remedy for the error MI is actually reporting and cheap.
     *
     * Re-applying the depth is NOT free: it discards whatever the port had
     * queued. That is an acceptable price once at bring-up and ruinous at the
     * period rate, which is why the budget below has to be genuinely bounded.
     */
    if ((unsigned int)ret == STAR_AUD_ERR_NOBUF &&
        st->aud_nobuf_recover[chn] < STAR_AUD_NOBUF_RECOVER_MAX) {
        int depth_ret;

        st->aud_nobuf_recover[chn]++;
        depth_ret = star_audio_set_port_depth(st, chn);
        HAL_LOG_WARN("audio: chn %d returned NOBUF; re-applying output port depth (attempt %d/%d, "
                     "SetChnOutputPortDepth -> %d)",
                     chn, st->aud_nobuf_recover[chn], STAR_AUD_NOBUF_RECOVER_MAX, depth_ret);

        if (!depth_ret) {
            memset(slot, 0, sizeof(*slot));
            ret = st->aud.fnGetFrame(dev, chn, slot, NULL, block ? STAR_AUD_GET_TIMEOUT_MS : 0);
            if (!ret)
                HAL_LOG_INFO("audio: chn %d recovered after re-applying the output port depth",
                             chn);
        }
    }

    if (ret) {
        if ((unsigned int)ret == STAR_AUD_ERR_BUF_EMPTY)
            return RSS_ERR_TIMEOUT;

        if (ret != st->aud_last_err) {
            HAL_LOG_WARN("MI_AI_GetFrame(%d, %d) failed: %#x (%s)", dev, chn, (unsigned int)ret,
                         star_audio_err_name(ret));
            st->aud_last_err = ret;
        }
        return RSS_ERR_IO;
    }
    st->aud_last_err = 0;
    /*
     * Refill the recovery budget only after a sustained run of good frames,
     * not on the first one. Resetting per frame made the bound meaningless:
     * any fault that alternates with successful reads gets three destructive
     * re-applies *per period* instead of three in total, which is exactly how
     * an empty-queue poll turned into continuous audio loss. A run means the
     * port is genuinely healthy again, so a later loss deserves a fresh budget.
     */
    if (st->aud_nobuf_recover[chn] &&
        ++st->aud_ok_run[chn] >= STAR_AUD_NOBUF_RECOVER_REARM_FRAMES) {
        st->aud_nobuf_recover[chn] = 0;
        st->aud_ok_run[chn] = 0;
    }

    if (!slot->addr[0] || !slot->length) {
        /* A success with nothing in it. Give it straight back rather than
         * passing a NULL buffer up. */
        st->aud.fnFreeFrame(dev, chn, slot, NULL);
        return RSS_ERR_TIMEOUT;
    }

    st->aud_frame_held[chn] = true;

    frame->data = (const int16_t *)slot->addr[0];
    frame->length = slot->length;
    /* MI's own timestamp, in microseconds. rad replaces it with a
     * synthetic clock for A/V sync reasons of its own, but reporting
     * what MI said is still the honest thing for anything that looks. */
    frame->timestamp = (int64_t)slot->timestamp;
    frame->seq = slot->sequence;
    frame->_priv = slot;

    return RSS_OK;
}

int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st || !frame)
        return RSS_ERR_INVAL;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    dev = star_audio_dev(st, dev);
    if (!star_audio_chn_ok(st, chn))
        return RSS_ERR_INVAL;
    if (!st->aud_frame_held[chn])
        return RSS_ERR_INVAL;

    ret = st->aud.fnFreeFrame(dev, chn, &st->aud_frame[chn], NULL);
    st->aud_frame_held[chn] = false;
    frame->data = NULL;
    frame->length = 0;
    frame->_priv = NULL;

    if (ret) {
        HAL_LOG_WARN("MI_AI_ReleaseFrame(%d, %d) failed: %#x", dev, chn, (unsigned int)ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * hal_audio_set_volume -- the digital trim, i.e. the stage after the ADC.
 *
 * The counterpart to audio_set_gain rather than a duplicate of it: gain
 * moves the analog front end, this moves s16RearGain, and i6_aud_chn_para
 * has the evidence that they are separate stages.
 *
 * The awkward part is that MI_AI_ChnParam_t carries BOTH gains and applies
 * both whenever gainOn is set, so this op has to supply a front gain even
 * though it does not mean to change one. Writing a zero there would drive
 * the analog stage to its minimum and silently undo audio_set_gain. The
 * value therefore comes from what this backend last wrote (aud_front_idx),
 * seeded at init from what the driver was already using.
 */
int hal_audio_set_volume(void *ctx, int dev, int chn, int vol)
{
    star_state_t *st = star_state(ctx);
    i6_aud_chn_para para;
    int front;
    int db;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;
    if (!st->aud.fnSetChannelParam)
        return RSS_ERR_NOTSUP;

    dev = star_audio_dev(st, dev);
    if (!star_audio_chn_ok(st, chn))
        return RSS_ERR_INVAL;

    db = star_audio_volume_db(vol);

    front = st->aud_front_idx;
    if (front < 0) {
        /* Neither init nor set_gain has established one. Derive it from the
         * gain this backend holds rather than writing a zero; rad sets the
         * gain immediately after the volume, so this is corrected within
         * milliseconds of being wrong. */
        front = star_audio_gain_index(st, st->aud_gain);
    }

    memset(&para, 0, sizeof(para));
    para.gain.gainOn = 1;
    para.gain.front = (short)front;
    para.gain.rear = (short)db;

    ret = st->aud.fnSetChannelParam(dev, chn, &para);
    if (ret) {
        HAL_LOG_WARN("MI_AI_SetChnParam(%d, %d, front %d, rear %d dB) failed: %#x (%s)", dev, chn,
                     front, db, (unsigned int)ret, star_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    st->aud_volume = vol < 0 ? 0 : (vol > 100 ? 100 : vol);
    HAL_LOG_DBG("audio: volume %d -> digital %d dB (analog front end left at %d)", st->aud_volume,
                db, front);

    return RSS_OK;
}

int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol)
{
    star_state_t *st = star_state(ctx);

    if (!st || !vol)
        return RSS_ERR_INVAL;

    (void)star_audio_dev(st, dev);
    (void)chn;

    /* Tracked, not queried: MI_AI_GetVqeVolume reports the VQE volume and
     * the VQE algorithms are absent here, so what it answers is not what
     * SetVqeVolume wrote. */
    *vol = st->aud_volume;

    return RSS_OK;
}

/*
 * hal_audio_set_gain -- the analog front end, i.e. the preamp.
 *
 * Through MI_AI_SetVqeVolume, which despite its name indexes the per-device
 * analog-gain table rather than taking decibels. This is the call the
 * backend has always used for level; only the op it hangs off has changed,
 * so the proven path stays proven.
 *
 * Left on MI_AI_SetVqeVolume rather than moved to MI_AI_SetChnParam's front
 * field on purpose, even though the two reach the same stage and using one
 * call for both gains would be tidier. If SetChnParam's front path turned
 * out not to work, the failure would be no analog gain control at all,
 * which is far worse than the failure mode of the split -- and this call is
 * known good on both boards in the fleet.
 */
int hal_audio_set_gain(void *ctx, int dev, int chn, int gain)
{
    star_state_t *st = star_state(ctx);
    int idx;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    dev = star_audio_dev(st, dev);
    if (!star_audio_chn_ok(st, chn))
        return RSS_ERR_INVAL;

    idx = star_audio_gain_index(st, gain);

    ret = st->aud.fnSetVolume(dev, chn, idx);
    if (ret) {
        HAL_LOG_WARN("MI_AI_SetVqeVolume(%d, %d, %d) failed: %#x (%s)", dev, chn, idx,
                     (unsigned int)ret, star_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    st->aud_gain = gain < 0 ? 0 : (gain > STAR_AUD_GAIN_MAX ? STAR_AUD_GAIN_MAX : gain);
    /* Remembered so hal_audio_set_volume can preserve it -- MI_AI_ChnParam_t
     * carries both stages and would otherwise zero this one. */
    st->aud_front_idx = idx;
    HAL_LOG_DBG("audio: gain %d -> analog step %d (%s)", st->aud_gain, idx,
                st->aud_input == RSS_AUDIO_INPUT_DMIC ? "dmic" : "amic");

    return RSS_OK;
}

int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain)
{
    star_state_t *st = star_state(ctx);

    if (!st || !gain)
        return RSS_ERR_INVAL;

    (void)star_audio_dev(st, dev);
    (void)chn;

    /* Tracked rather than read back, for the same reason the Infinity6C
     * backend tracks its own: MI answers in dB, and converting that to
     * raptor's 0..31 would round through a lossy map, so a caller reading
     * back what it just wrote would not always see it. */
    *gain = st->aud_gain;

    return RSS_OK;
}

int hal_audio_set_mute(void *ctx, int dev, int chn, int mute)
{
    star_state_t *st = star_state(ctx);
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    dev = star_audio_dev(st, dev);
    if (!star_audio_chn_ok(st, chn))
        return RSS_ERR_INVAL;

    ret = st->aud.fnSetMute(dev, chn, mute ? 1 : 0);
    if (ret) {
        HAL_LOG_WARN("MI_AI_SetMute(%d, %d, %d) failed: %#x", dev, chn, mute ? 1 : 0,
                     (unsigned int)ret);
        return RSS_ERR_IO;
    }

    return RSS_OK;
}
