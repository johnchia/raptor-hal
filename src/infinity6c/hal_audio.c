/*
 * infinity6c/hal_audio.c -- MI_AI audio capture for SigmaStar Infinity6C (MI 3.0)
 *
 * THE POINT OF THIS FILE
 *
 * Raw PCM in, nothing else. rad owns encoding -- it ships G.711, L16, Opus and
 * AAC in software -- so all this has to do is bring the AI device up, hand out
 * periods, and stay out of the way. That division is not a preference here but
 * the only option: this generation's libmi_ai.so contains no encoder, no
 * resampler and no sound-quality algorithms at all (see i6c_aud.h, point 4).
 *
 * WHY THIS IS NOT star/hal_audio.c WITH THE NAMES CHANGED
 *
 * MI 3.0 reorganised audio input rather than revising it, and i6c_aud.h lists
 * the five differences. Three of them shape this file:
 *
 *   - The input is attached, not indexed. MI_AI_Open takes the data format and
 *     MI_AI_AttachIf points the device's multiplexer at ADC or DMIC pins, so
 *     rss_audio_config_t.input_type is honoured by choosing an interface instead
 *     of by choosing a device number.
 *
 *   - A channel is a channel group. Every per-channel entry point takes a group
 *     index, and one attached interface is two physical channels, so the sound
 *     mode decides how the two are divided. See i6c_audio_open_dev.
 *
 *   - There are two gain stages, reached by two different calls. That is what
 *     lets audio_set_volume and audio_set_gain both be implemented honestly
 *     here: on MI 2.x they would have fought over one register, which is the
 *     collision star/hal_audio.c documents and works around by leaving set_gain
 *     unimplemented.
 *
 * WHY audio_init DOES THE WORK hal_init NORMALLY DOES
 *
 * rad calls rss_hal_create and then audio_init directly; it never calls the init
 * op. On Ingenic that is fine because the audio device needs no system-wide
 * bring-up. On MI it is not: libmi_ai.so declares only libc as NEEDED yet leaves
 * CamOs* and MI_SYS_* undefined and GLOBAL, so both libcam_os_wrapper.so and
 * libmi_sys.so have to be in the process RTLD_GLOBAL before a single AI symbol
 * can bind. So audio_init loads them through i6c_sys_load, initialises MI_SYS,
 * and records that it was the one who did (aud_owns_sys) so deinit can undo
 * exactly that much. When something has called hal_init first, the existing
 * state is reused and MI_SYS is left alone.
 *
 * A consequence worth knowing: rvd and rad are separate processes, so both call
 * MI_SYS_Init in their own address space. That is what MI's multi-process design
 * intends, and the vendor doc is specific about where the limit is -- "the
 * operation of the AI channel group does not support multiple processes. If a
 * process is enabled, it can only be used and disabled here" -- i.e. one process
 * per channel group, which rad satisfies by being the only audio process.
 *
 * OP COVERAGE
 *
 * Implemented: audio_init, audio_deinit, audio_read_frame, audio_release_frame,
 * audio_set_volume, audio_get_volume, audio_set_gain, audio_get_gain,
 * audio_set_mute.
 *
 * Everything else is left NULL in the vtable, which RSS_HAL_CALL turns into
 * RSS_ERR_NOTSUP. Per op, why:
 *
 *   audio_enable_ns / disable_ns, audio_enable_hpf / disable_hpf,
 *   audio_enable_agc / disable_agc, audio_set_agc_mode, audio_set_hpf_co_freq,
 *   audio_enable_aec / disable_aec, audio_enable_aec_ref_frame /
 *   disable_aec_ref_frame, audio_set_aec_profile_path, audio_get_frame_and_ref
 *       Not merely unavailable -- absent. libmi_ai.so exports 22 symbols and not
 *       one matches vqe, aenc, aed, iaa or src, so unlike MI 2.x there is no
 *       entry point to call and no weak-undefined algorithm pack behind it. The
 *       vendor's own note explains the direction: "In 2.19 and later versions of
 *       the API, MI_AI no longer includes the associated algorithm functions."
 *
 *       audio_enable_aec_ref_frame is the one that could be revisited. MI_AI does
 *       still carry the AEC *reference* path -- attach E_MI_AI_IF_ECHO_A and
 *       MI_AI_Read's second output fills with the far-end signal -- so raptor
 *       could hand a caller aligned near and far data. It is left out because
 *       nothing in scope plays audio, so there is no far end to reference, and
 *       the echo interface must be attached at open to exist at all.
 *
 *   audio_set_alc_gain / get_alc_gain
 *       Automatic level control, which is an AGC feature; see above.
 *
 *   audio_register_encoder / unregister_encoder
 *       For SoCs with a hardware audio encoder raptor can drive. There is no
 *       MI_AENC on this generation.
 *
 *   audio_get_chn_param
 *       The op passes an opaque void* whose layout callers assume is Ingenic's
 *       IMPAudioIOAttr. MI 3.0 has no per-group parameter struct to fill it from
 *       -- MI_AI_GetAttr answers per *device* -- so there is no honest mapping.
 *
 *   the ao_* family
 *       Playback. libmi_ao is never loaded and this backend is capture-only by
 *       design.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "infinity6c_state.h"

#include <stdlib.h>
#include <string.h>

/*
 * No HAL_MODULE_AUDIO guard, deliberately -- this file is only ever compiled
 * into the audio archive (the Makefile's AUDIO_SRCS), so a guard would be
 * redundant at best. It is also a trap in reverse: -DHAL_MODULE_AUDIO reaches
 * only src/hal_common_audio.o, so a guard here would compile the whole
 * translation unit away and the failure would arrive as undefined vtable symbols
 * at link time.
 */

/*
 * Capture rates.
 *
 * The vendor rate enum is shared between input and output and annotates five of
 * its members "AO only" -- 11025, 12000, 22050, 24000 and 192000 play but do not
 * record. Of what is left, the AI procfs node documents 8k/16k/32k/48k, which is
 * also every rate the vendor's examples use. 96000 is in the enum without an
 * AO-only note but outside the documented capture set, so it is not offered.
 *
 * An unsupported rate is refused rather than resampled: MI 3.0 has no SRC entry
 * point in libmi_ai.so, so there is nothing to resample with.
 */
static bool i6c_audio_rate_ok(int rate)
{
    return rate == 8000 || rate == 16000 || rate == 32000 || rate == 48000;
}

/*
 * MI_AI error codes, from the vendor doc's error table.
 *
 * Named rather than printed as bare hex because two of them differ by one digit
 * and nothing else: 0xA004200D (NOBUF, "this port has no queue") and 0xA004200E
 * (BUF_EMPTY, "nothing captured yet") mean entirely different things, and on the
 * i6e backend reading one as the other cost a board cycle. Anything absent from
 * the table is reported as its hex alone, which is still the truth.
 */
static const char *i6c_audio_err_name(int err)
{
    switch ((unsigned int)err) {
    case 0xA0042001u: return "INVALID_DEVID";
    case 0xA0042002u: return "INVALID_CHNGRPID";
    case 0xA0042003u: return "ILLEGAL_PARAM";
    case 0xA0042006u: return "NULL_PTR";
    case 0xA0042007u: return "NOT_CONFIG";
    case 0xA0042008u: return "NOT_SUPPORT";
    case 0xA0042009u: return "NOT_PERM";
    case 0xA004200Cu: return "NOMEM";
    case I6C_AUD_ERR_NOBUF: return "NOBUF";
    case I6C_AUD_ERR_BUF_EMPTY: return "BUF_EMPTY";
    case 0xA004200Fu: return "BUF_FULL";
    case 0xA0042010u: return "SYS_NOTREADY";
    case 0xA0042012u: return "BUSY";
    case 0xA0042017u: return "NOT_ENABLED";
    default: return "unknown";
    }
}

/*
 * Which interface carries the microphone.
 *
 * raptor distinguishes only analog from digital, and each maps to the first
 * interface of its kind: ADC A+B for analog (the two-channel Amic/Line-in path)
 * and DMIC channels 0 and 1 for digital. Both are pairs, which is what makes one
 * interface enough for either a mono or a stereo configuration.
 *
 * A board with its mic on ADC C+D or on the second DMIC pair would need this to
 * become configurable. It is not guessed at from the device tree, because the
 * sound node describes pad muxing rather than which interface an application
 * should attach, and attaching the wrong one records silence rather than failing.
 */
static i6c_aud_if i6c_audio_input_if(rss_audio_input_t input)
{
    return input == RSS_AUDIO_INPUT_DMIC ? I6C_AUD_IF_DMIC_A_01 : I6C_AUD_IF_ADC_AB;
}

static int i6c_audio_if_gain_max(rss_audio_input_t input)
{
    return input == RSS_AUDIO_INPUT_DMIC ? I6C_AUD_IF_GAIN_MAX_DMIC : I6C_AUD_IF_GAIN_MAX_ADC;
}

/*
 * raptor volume (0..100) -> DPGA gain in dB.
 *
 * Piecewise around I6C_AUD_VOL_UNITY so that rad's shipped default applies no
 * digital gain at all, with the halves scaling to the documented [-60, +30]:
 *
 *      0 -> -60 dB      80 -> 0 dB (unity)      100 -> +30 dB
 *
 * A single linear map over the whole range would be tidier to read and wrong to
 * use -- it would put a default install at about +12 dB of digital boost on top
 * of whatever the analog stage is doing, which is clipping nobody asked for. The
 * analog stage is where level belongs (see hal_audio_set_gain); this knob is for
 * trimming around it.
 */
static int i6c_audio_volume_db(int vol)
{
    if (vol < 0)
        vol = 0;
    else if (vol > 100)
        vol = 100;

    if (vol < I6C_AUD_VOL_UNITY)
        return I6C_AUD_DPGA_MIN_DB +
               (vol * -I6C_AUD_DPGA_MIN_DB + I6C_AUD_VOL_UNITY / 2) / I6C_AUD_VOL_UNITY;

    return ((vol - I6C_AUD_VOL_UNITY) * I6C_AUD_DPGA_MAX_DB + (100 - I6C_AUD_VOL_UNITY) / 2) /
           (100 - I6C_AUD_VOL_UNITY);
}

/*
 * raptor gain (0..31) -> interface gain, in hardware steps.
 *
 * Linear across the interface's whole range, so 0 is the quietest the front end
 * offers and 31 the loudest. rad's default of 25 lands on step 17 of the ADC's
 * 21, i.e. about +45 dB -- close to the 18 the vendor's own capture example
 * passes, and the right neighbourhood for an electret mic.
 *
 * Clamped, because an index off the end of the table earns
 * MI_AI_ERR_ILLEGAL_PARAM and no gain change rather than a louder signal.
 */
static int i6c_audio_gain_step(const infinity6c_state_t *st, int gain)
{
    if (gain < 0)
        gain = 0;
    else if (gain > 31)
        gain = 31;

    return (gain * st->aud_if_gain_max + 15) / 31;
}

/*
 * The device rad asked for versus the one that exists.
 *
 * audio_init takes no device argument, so it configures I6C_AUD_DEV and the
 * per-device ops can only ever be handed something else by config accident --
 * rad's `[audio] device` defaults to 1, an Ingenic index. Refusing would mean
 * silence out of the box for a value nobody chose, so the configured device wins
 * and the mismatch is named once.
 *
 * On this part the point is sharper than on MI 2.x, where the index at least
 * selected the physical input: Maruko has one WDMA, so device 0 is the only
 * audio input device there is and no other value could ever be right.
 */
static void i6c_audio_check_dev(infinity6c_state_t *st, int dev)
{
    if (dev != I6C_AUD_DEV && !st->aud_dev_warned) {
        HAL_LOG_WARN("audio: asked for device %d, but this chip has one audio input device (%d) "
                     "and that is the one configured; set [audio] device = %d",
                     dev, I6C_AUD_DEV, I6C_AUD_DEV);
        st->aud_dev_warned = true;
    }
}

static bool i6c_audio_grp_ok(const infinity6c_state_t *st, int grp)
{
    return grp >= 0 && grp < I6C_AUD_GRP_MAX && (unsigned int)grp < st->aud_grp_count;
}

/*
 * Give one channel group's output port somewhere to put periods.
 *
 * This is not tuning and it is not optional -- more so than on MI 2.x, because
 * this generation has no device-side ring behind it. An MI channel's output port
 * starts with no user-side queue at all, so MI_AI_Read has nothing to hand back
 * and fails with MI_AI_ERR_NOBUF on every call while the device reports itself
 * perfectly open. The vendor's own capture example sets the depth in the same
 * breath as enabling the group, for this reason.
 *
 * See I6C_AUD_PORT_*_DEPTH for the numbers and the fallback.
 */
static int i6c_audio_set_port_depth(infinity6c_state_t *st, unsigned int grp)
{
    i6c_sys_bind port;
    int ret;

    /*
     * A local copy of what hal_framesource.c's i6c_set_output_depth does, rather
     * than a call to it: that file is a VIDEO_SRCS member and is not linked into
     * the audio archive. It also returns void, being best-effort for a video
     * stage, where here the result decides whether capture can work at all.
     */
    memset(&port, 0, sizeof(port));
    port.module = I6C_SYS_MOD_AI;
    port.device = I6C_AUD_DEV;
    port.channel = grp;
    port.port = 0;

    ret = st->sys.set_output_depth(I6C_SOC_ID, &port, I6C_AUD_PORT_USR_DEPTH,
                                   I6C_AUD_PORT_BUF_DEPTH);
    if (!ret)
        return 0;

    HAL_LOG_WARN("audio: group %u rejected output port depth (%d, %d): %#x (%s) -- retrying at "
                 "the vendor example's (%d, %d)",
                 grp, I6C_AUD_PORT_USR_DEPTH, I6C_AUD_PORT_BUF_DEPTH, (unsigned int)ret,
                 i6c_audio_err_name(ret), I6C_AUD_PORT_USR_DEPTH_FALLBACK,
                 I6C_AUD_PORT_BUF_DEPTH_FALLBACK);

    return st->sys.set_output_depth(I6C_SOC_ID, &port, I6C_AUD_PORT_USR_DEPTH_FALLBACK,
                                    I6C_AUD_PORT_BUF_DEPTH_FALLBACK);
}

/*
 * Bring the AI device down, undoing only what came up.
 *
 * Reverse of bring-up, which MI requires: a group must be disabled before the
 * device is closed, and a period still checked out must go back first or MI keeps
 * the buffer.
 */
static void i6c_audio_teardown(infinity6c_state_t *st)
{
    unsigned int i;
    int ret;

    for (i = 0; i < I6C_AUD_GRP_MAX; i++) {
        if (!st->aud_grp_enabled[i])
            continue;

        if (st->aud_frame_held[i]) {
            st->aud.release(st->aud_dev, (unsigned char)i, &st->aud_frame[i], NULL);
            st->aud_frame_held[i] = false;
        }

        ret = st->aud.disable_grp(st->aud_dev, (unsigned char)i);
        if (ret)
            HAL_LOG_WARN("MI_AI_DisableChnGroup(%u, %u) failed: %#x (%s)", st->aud_dev, i,
                         (unsigned int)ret, i6c_audio_err_name(ret));
        st->aud_grp_enabled[i] = false;
    }

    if (st->aud_dev_open) {
        ret = st->aud.close(st->aud_dev);
        if (ret)
            HAL_LOG_WARN("MI_AI_Close(%u) failed: %#x (%s)", st->aud_dev, (unsigned int)ret,
                         i6c_audio_err_name(ret));
        st->aud_dev_open = false;
    }

    st->aud_grp_count = 0;
}

/*
 * i6c_audio_open_dev -- MI_AI_Open, MI_AI_AttachIf, the output-port queue, then
 * one enabled group.
 *
 * That is the vendor capture example's order. Two steps of it are required and
 * documented as such: attach only after open, enable only after attach. Where the
 * queue goes is not -- see the note at the call.
 *
 * ON ENABLING EXACTLY ONE GROUP
 *
 * A mono configuration on a two-channel interface has two groups available -- one
 * per physical channel -- and only the first is enabled, because raptor consumes
 * one stream and rad reads group 0. An enabled group nobody reads still runs its
 * DMA and fills its output-port queue, which buys nothing and costs both memory
 * and MI complaining about lost buffers. A caller that wants the second
 * microphone asks for stereo, which puts both physical channels in group 0 as
 * interleaved samples.
 */
static int i6c_audio_open_dev(infinity6c_state_t *st, const rss_audio_config_t *cfg,
                              unsigned int chn_count, unsigned int samples)
{
    i6c_aud_cnf dev_cfg;
    i6c_aud_if ifaces[1];
    int ret;

    /*
     * Cleared before filling, and not as a habit: MI_BOOL is a byte, so
     * `interleaved` is followed by three bytes of padding that MI_AI_Open reads
     * and marshals as part of a whole word. See i6c_aud.h.
     */
    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.format = I6C_AUD_FMT_PCM_S16_LE;
    dev_cfg.sound = chn_count >= 2 ? I6C_AUD_SND_STEREO : I6C_AUD_SND_MONO;
    dev_cfg.rate = (i6c_aud_rate)cfg->sample_rate;
    dev_cfg.periodSize = samples;
    /*
     * Interleaved, so a group arrives as one buffer at addr[0] rather than as one
     * buffer per physical channel. raptor's rss_audio_frame_t carries a single
     * pointer and rad's codecs all expect interleaved PCM, so the alternative
     * would mean interleaving in software for no reason. For a mono group the
     * distinction does not arise.
     */
    dev_cfg.interleaved = 1;

    ret = st->aud.open(st->aud_dev, &dev_cfg);
    if (ret) {
        HAL_LOG_ERR("MI_AI_Open(%u) failed: %#x (%s) -- %d Hz, %s, %u samples/period",
                    st->aud_dev, (unsigned int)ret, i6c_audio_err_name(ret), (int)cfg->sample_rate,
                    chn_count >= 2 ? "stereo" : "mono", samples);
        return RSS_ERR_IO;
    }
    st->aud_dev_open = true;

    /*
     * One interface, which is two physical channels -- enough for either sound
     * mode. Attaching more would raise the group count without giving raptor
     * anything to do with the extra groups, and the vendor is explicit that a
     * device does not support attaching again later, so this is the one chance to
     * get it right.
     */
    ifaces[0] = st->aud_if;
    ret = st->aud.attach_if(st->aud_dev, ifaces, 1);
    if (ret) {
        HAL_LOG_ERR("MI_AI_AttachIf(%u, if %d) failed: %#x (%s)", st->aud_dev, (int)st->aud_if,
                    (unsigned int)ret, i6c_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    /*
     * The queue before the group is enabled, which is the order the vendor
     * capture example uses (its step 41 against its step 51).
     *
     * Both orders were tried on the board and **both work**: enabling first and
     * then setting the depth also gives 50 periods a second with no NOBUF and no
     * discards. So this is not load bearing on this chip, and it is written this
     * way only because it is the documented sequence -- if a later reader finds a
     * reason to move it, there is no measurement here standing in the way.
     *
     * Recorded because the opposite was briefly believed. A NOBUF seen in early
     * runs looked like proof that the depth had to come first; it turned out to
     * arrive only at shutdown, from a different cause entirely (see
     * hal_audio_read_frame), and steady-state capture had been fine all along.
     * The i6e backend's NOBUF-at-boot is therefore still unexplained rather than
     * explained by this.
     */
    ret = i6c_audio_set_port_depth(st, 0);
    if (ret) {
        HAL_LOG_ERR("MI_SYS_SetChnOutputPortDepth(AI %d, group 0) failed: %#x (%s) -- MI_AI_Read "
                    "would return NOBUF forever",
                    I6C_AUD_DEV, (unsigned int)ret, i6c_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    ret = st->aud.enable_grp(st->aud_dev, 0);
    if (ret) {
        HAL_LOG_ERR("MI_AI_EnableChnGroup(%u, 0) failed: %#x (%s)", st->aud_dev, (unsigned int)ret,
                    i6c_audio_err_name(ret));
        return RSS_ERR_IO;
    }
    st->aud_grp_enabled[0] = true;
    st->aud_grp_count = 1;

    return RSS_OK;
}

/*
 * hal_audio_init -- configure and start audio capture.
 *
 * Re-entrant on purpose: rad calls this again to change sample rate at runtime,
 * so an already-running device is torn down first rather than refused. MI 3.0
 * makes that mandatory rather than tidy -- the interface cannot be re-attached
 * on an open device, so a reconfiguration is a close and a fresh open.
 */
int hal_audio_init(void *ctx, const rss_audio_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;
    unsigned int chn_count;
    unsigned int samples;
    int ret;

    if (!c || !cfg)
        return RSS_ERR_INVAL;

    if (!i6c_audio_rate_ok((int)cfg->sample_rate)) {
        HAL_LOG_ERR("audio: %d Hz is not an MI capture rate -- only 8000, 16000, 32000 and 48000 "
                    "are supported, and this generation has no resampler",
                    (int)cfg->sample_rate);
        return RSS_ERR_INVAL;
    }

    chn_count = cfg->chn_count > 0 ? (unsigned int)cfg->chn_count : 1;
    if (chn_count > 2) {
        /*
         * Not a hardware limit -- a group can hold up to eight physical channels
         * -- but a limit of what one attached interface provides and of what
         * raptor can express, since rss_audio_config_t.chn_count only means mono
         * or stereo.
         */
        HAL_LOG_WARN("audio: %u channels requested; one interface carries two, capping at stereo",
                     chn_count);
        chn_count = 2;
    }

    /*
     * State and MI_SYS. Present already when something called hal_init; created
     * here when nothing did, which is rad's normal path.
     */
    st = (infinity6c_state_t *)c->platform;
    if (!st) {
        st = (infinity6c_state_t *)calloc(1, sizeof(*st));
        if (!st)
            return RSS_ERR_NOMEM;

        ret = i6c_sys_load(&st->sys);
        if (ret) {
            /* i6c_sys_load names the library or symbol that was missing. */
            free(st);
            return ret;
        }

        ret = st->sys.init(I6C_SOC_ID);
        if (ret) {
            HAL_LOG_ERR("audio: MI_SYS_Init(soc %d) failed: %d", I6C_SOC_ID, ret);
            i6c_sys_unload(&st->sys);
            free(st);
            return RSS_ERR_IO;
        }

        st->sys_inited = true;
        st->aud_owns_sys = true;
        c->platform = st;
    }

    if (!st->aud_loaded) {
        ret = i6c_aud_load(&st->aud);
        if (ret)
            return ret;
        st->aud_loaded = true;
    } else {
        /* A rate change on a running device. */
        i6c_audio_teardown(st);
    }

    st->aud_dev = I6C_DEV_ID(I6C_AUD_DEV);
    st->aud_rate = (int)cfg->sample_rate;
    st->aud_input = cfg->input_type;
    st->aud_if = i6c_audio_input_if(cfg->input_type);
    st->aud_if_gain_max = i6c_audio_if_gain_max(cfg->input_type);
    st->aud_chn_per_grp = chn_count >= 2 ? 2 : 1;

    /*
     * Samples per period, per physical channel. The config wins when it says
     * something; otherwise one 20 ms period, which is what the vendor example
     * uses (160 at 8 kHz) and what rad slices its reads into anyway, so matching
     * it keeps one read to one encoded packet.
     */
    samples = cfg->samples_per_frame > 0 ? (unsigned int)cfg->samples_per_frame
                                         : (unsigned int)cfg->sample_rate / 50;

    ret = i6c_audio_open_dev(st, cfg, chn_count, samples);
    if (ret) {
        i6c_audio_teardown(st);
        return ret;
    }

    HAL_LOG_INFO("audio: AI device %d up on %s, %d Hz %s, %u samples/period, %u group",
                 I6C_AUD_DEV, st->aud_if == I6C_AUD_IF_DMIC_A_01 ? "DMIC 0/1" : "ADC A/B",
                 st->aud_rate, chn_count >= 2 ? "stereo" : "mono", samples, st->aud_grp_count);

    /*
     * Gain is not set here: rad applies volume and gain immediately after init,
     * and choosing a default would mean overriding whatever the codec came up
     * with for no reason.
     */
    return RSS_OK;
}

int hal_audio_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    infinity6c_state_t *st;

    if (!c)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)c->platform;
    if (!st)
        return RSS_OK;

    if (st->aud_loaded) {
        i6c_audio_teardown(st);
        i6c_aud_unload(&st->aud);
        st->aud_loaded = false;
    }

    /*
     * Only unwind MI_SYS if audio_init was what brought it up. When hal_init did,
     * the video half of the pipeline is still using it and this op has no
     * business tearing it down -- hal_deinit will.
     */
    if (st->aud_owns_sys) {
        if (st->sys_inited) {
            st->sys.exit(I6C_SOC_ID);
            st->sys_inited = false;
        }
        i6c_sys_unload(&st->sys);
        st->aud_owns_sys = false;
        c->platform = NULL;
        free(st);
    }

    return RSS_OK;
}

/*
 * hal_audio_read_frame -- one captured period.
 *
 * The buffer belongs to MI until release_frame hands it back, so nothing is
 * copied: frame->data points into MI's DMA buffer and _priv carries the
 * descriptor MI needs to see again.
 *
 * A timeout is not an error. MI reports "no data yet" as a distinct condition and
 * rad's loop treats RSS_ERR_TIMEOUT as "try again", so a device that is merely
 * idle stays quiet in the log. Any other failure is named once per distinct code
 * -- a real fault otherwise repeats at the capture period and buries everything
 * else.
 */
int hal_audio_read_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame, bool block)
{
    infinity6c_state_t *st;
    i6c_aud_frm *slot;
    int ret;

    if (!ctx || !frame)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)((rss_hal_ctx_t *)ctx)->platform;
    if (!st)
        return RSS_ERR_NOTSUP;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    i6c_audio_check_dev(st, dev);
    if (!i6c_audio_grp_ok(st, chn))
        return RSS_ERR_INVAL;
    if (st->aud_frame_held[chn]) {
        HAL_LOG_WARN("audio: group %d already holds a period; release it before reading again",
                     chn);
        return RSS_ERR_BUSY;
    }

    slot = &st->aud_frame[chn];
    memset(slot, 0, sizeof(*slot));

    /*
     * No echo reference: NULL is explicitly allowed for it, and attaching
     * E_MI_AI_IF_ECHO_A would be a precondition anyway.
     */
    ret = st->aud.read(st->aud_dev, (unsigned char)chn, slot, NULL,
                       block ? I6C_AUD_READ_TIMEOUT_MS : 0);

    /*
     * BUF_EMPTY always means "nothing captured yet". NOBUF means that too on a
     * non-blocking read: MI_AI overloads the code, answering an empty queue with
     * it when the timeout is 0, which the i6e backend established the hard way --
     * reading it as a lost queue there and re-establishing the port depth in
     * response flushed the queue on every idle poll and turned an ordinary empty
     * read into continuous audio loss.
     *
     * On a blocking read NOBUF is reported as the fault it documents -- with one
     * benign occurrence to expect. A blocking MI_AI_Read that is sitting in its
     * ioctl when a signal arrives returns NOBUF rather than EINTR, so **every
     * clean shutdown logs exactly one of these**, immediately before rad's
     * "shutting down". It is not a queue fault and nothing is lost. It is left
     * visible rather than suppressed because the return code alone cannot
     * distinguish an interrupted read from a real one, and inventing a
     * teardown-in-progress flag to hide one line per stop would cost more clarity
     * than it buys. A NOBUF in *steady state* is a real fault; one at exit is
     * this.
     *
     * No automatic recovery is attempted, unlike the i6e backend, which
     * re-establishes the port depth on NOBUF and retries. Re-applying a depth is
     * destructive -- it discards whatever the port had queued -- and on this
     * generation that port is the only queue there is, so retrying blindly would
     * trade a diagnosable error for silent audio loss. If this board ever shows
     * NOBUF while genuinely running, that recovery is the thing to reach for, and
     * the fact that it would have fired once per shutdown is the reason it is not
     * here already.
     */
    if ((unsigned int)ret == I6C_AUD_ERR_BUF_EMPTY ||
        ((unsigned int)ret == I6C_AUD_ERR_NOBUF && !block))
        return RSS_ERR_TIMEOUT;

    if (ret) {
        if (ret != st->aud_last_err) {
            HAL_LOG_WARN("MI_AI_Read(%u, %d) failed: %#x (%s)", st->aud_dev, chn,
                         (unsigned int)ret, i6c_audio_err_name(ret));
            st->aud_last_err = ret;
        }
        return RSS_ERR_IO;
    }
    st->aud_last_err = 0;

    if (!slot->addr[0] || !slot->length[0]) {
        /* A success with nothing in it. Give it straight back rather than passing
         * a NULL buffer up. */
        st->aud.release(st->aud_dev, (unsigned char)chn, slot, NULL);
        return RSS_ERR_TIMEOUT;
    }

    st->aud_frame_held[chn] = true;

    frame->data = (const int16_t *)slot->addr[0];
    /* u32Byte is bytes, which is what rss_audio_frame_t.length means too. */
    frame->length = slot->length[0];
    /* MI's own timestamp, in microseconds. rad replaces it with a synthetic clock
     * for A/V sync reasons of its own, but reporting what MI said is still the
     * honest thing for anything that looks. */
    frame->timestamp = (int64_t)slot->timestamp;
    /* MI counts periods in 64 bits and raptor's field is 32. At a 20 ms period
     * the low half takes 2.7 years to wrap, and nothing uses it for more than
     * spotting a discontinuity. */
    frame->seq = (uint32_t)slot->sequence;
    frame->_priv = slot;

    return RSS_OK;
}

int hal_audio_release_frame(void *ctx, int dev, int chn, rss_audio_frame_t *frame)
{
    infinity6c_state_t *st;
    int ret;

    if (!ctx || !frame)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)((rss_hal_ctx_t *)ctx)->platform;
    if (!st)
        return RSS_ERR_NOTSUP;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    i6c_audio_check_dev(st, dev);
    if (!i6c_audio_grp_ok(st, chn))
        return RSS_ERR_INVAL;
    if (!st->aud_frame_held[chn])
        return RSS_ERR_INVAL;

    ret = st->aud.release(st->aud_dev, (unsigned char)chn, &st->aud_frame[chn], NULL);
    st->aud_frame_held[chn] = false;
    frame->data = NULL;
    frame->length = 0;
    frame->_priv = NULL;

    if (ret) {
        HAL_LOG_WARN("MI_AI_ReleaseData(%u, %d) failed: %#x (%s)", st->aud_dev, chn,
                     (unsigned int)ret, i6c_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    return RSS_OK;
}

/*
 * hal_audio_set_volume -- the DPGA, i.e. digital gain.
 *
 * Per physical channel in the group, so the array is one element for a mono group
 * and two for a stereo one, both set to the same value: raptor has one volume and
 * splitting it per channel would be a balance control nothing asks for.
 */
int hal_audio_set_volume(void *ctx, int dev, int chn, int vol)
{
    infinity6c_state_t *st;
    signed char gains[2];
    int db;
    int ret;
    unsigned int i;

    if (!ctx)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)((rss_hal_ctx_t *)ctx)->platform;
    if (!st)
        return RSS_ERR_NOTSUP;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    i6c_audio_check_dev(st, dev);
    if (!i6c_audio_grp_ok(st, chn))
        return RSS_ERR_INVAL;

    db = i6c_audio_volume_db(vol);
    for (i = 0; i < st->aud_chn_per_grp && i < I6C_ARRAY_LEN(gains); i++)
        gains[i] = (signed char)db;

    ret = st->aud.set_gain(st->aud_dev, (unsigned char)chn, gains,
                           (unsigned char)st->aud_chn_per_grp);
    if (ret) {
        HAL_LOG_WARN("MI_AI_SetGain(%u, %d, %d dB) failed: %#x (%s)", st->aud_dev, chn, db,
                     (unsigned int)ret, i6c_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    st->aud_volume = vol < 0 ? 0 : (vol > 100 ? 100 : vol);
    HAL_LOG_DBG("audio: volume %d -> DPGA %d dB on %u channel(s)", st->aud_volume, db,
                st->aud_chn_per_grp);

    return RSS_OK;
}

/*
 * hal_audio_set_gain -- the interface, i.e. the analog front end.
 *
 * Both sides of the pair get the same step. For ADC A+B the two sides are two
 * separate microphone inputs rather than a stereo pair's halves, so setting only
 * the left would leave a stereo configuration lopsided.
 */
int hal_audio_set_gain(void *ctx, int dev, int chn, int gain)
{
    infinity6c_state_t *st;
    signed char left = 0;
    signed char right = 0;
    int step;
    int ret;

    if (!ctx)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)((rss_hal_ctx_t *)ctx)->platform;
    if (!st)
        return RSS_ERR_NOTSUP;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    i6c_audio_check_dev(st, dev);
    if (!i6c_audio_grp_ok(st, chn))
        return RSS_ERR_INVAL;

    step = i6c_audio_gain_step(st, gain);

    ret = st->aud.set_if_gain(st->aud_if, (signed char)step, (signed char)step);
    if (ret) {
        HAL_LOG_WARN("MI_AI_SetIfGain(if %d, %d) failed: %#x (%s)", (int)st->aud_if, step,
                     (unsigned int)ret, i6c_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    st->aud_gain = gain < 0 ? 0 : (gain > 31 ? 31 : gain);

    /*
     * Read back what the front end actually took. Cheap, and the one place a
     * mis-scaled step or a per-chip ceiling that is not what the doc says would
     * show up as a number rather than as a quiet difference in level -- which
     * matters most for the DMIC ceiling, where the sources disagree (see
     * I6C_AUD_IF_GAIN_MAX_DMIC).
     */
    if (!st->aud.get_if_gain(st->aud_if, &left, &right))
        HAL_LOG_DBG("audio: gain %d -> interface step %d of %d (hardware reports %d, %d)",
                    st->aud_gain, step, st->aud_if_gain_max, (int)left, (int)right);
    else
        HAL_LOG_DBG("audio: gain %d -> interface step %d of %d", st->aud_gain, step,
                    st->aud_if_gain_max);

    return RSS_OK;
}

/*
 * The getters answer in raptor's units, from what was set.
 *
 * MI_AI_GetGain and MI_AI_GetIfGain are both real on this library, unlike MI
 * 2.x's volume getter, and the setters log what they report. They are not the
 * source here because they answer in dB and in hardware steps: converting back
 * would round through two lossy maps, so a caller reading a setting straight
 * back would not always see the value it wrote.
 */
int hal_audio_get_volume(void *ctx, int dev, int chn, int *vol)
{
    infinity6c_state_t *st;

    if (!ctx || !vol)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)((rss_hal_ctx_t *)ctx)->platform;
    if (!st)
        return RSS_ERR_NOTSUP;

    i6c_audio_check_dev(st, dev);
    (void)chn;

    *vol = st->aud_volume;

    return RSS_OK;
}

int hal_audio_get_gain(void *ctx, int dev, int chn, int *gain)
{
    infinity6c_state_t *st;

    if (!ctx || !gain)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)((rss_hal_ctx_t *)ctx)->platform;
    if (!st)
        return RSS_ERR_NOTSUP;

    i6c_audio_check_dev(st, dev);
    (void)chn;

    *gain = st->aud_gain;

    return RSS_OK;
}

/*
 * hal_audio_set_mute -- the DPGA's mute, per physical channel.
 *
 * Not the interface's. MI_AI_SetIfMute exists in the library as a four-byte stub
 * that returns MI_SUCCESS and does nothing, so wiring mute to it would give an op
 * that reports success and passes audio -- see i6c_aud_load.h for why those stubs
 * are deliberately not bound.
 */
int hal_audio_set_mute(void *ctx, int dev, int chn, int mute)
{
    infinity6c_state_t *st;
    unsigned char mutes[2];
    int ret;
    unsigned int i;

    if (!ctx)
        return RSS_ERR_INVAL;

    st = (infinity6c_state_t *)((rss_hal_ctx_t *)ctx)->platform;
    if (!st)
        return RSS_ERR_NOTSUP;
    if (!st->aud_loaded)
        return RSS_ERR_NOTSUP;

    i6c_audio_check_dev(st, dev);
    if (!i6c_audio_grp_ok(st, chn))
        return RSS_ERR_INVAL;

    for (i = 0; i < st->aud_chn_per_grp && i < I6C_ARRAY_LEN(mutes); i++)
        mutes[i] = mute ? 1 : 0;

    ret = st->aud.set_mute(st->aud_dev, (unsigned char)chn, mutes,
                           (unsigned char)st->aud_chn_per_grp);
    if (ret) {
        HAL_LOG_WARN("MI_AI_SetMute(%u, %d, %d) failed: %#x (%s)", st->aud_dev, chn, mute ? 1 : 0,
                     (unsigned int)ret, i6c_audio_err_name(ret));
        return RSS_ERR_IO;
    }

    return RSS_OK;
}
