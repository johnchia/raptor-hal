/*
 * caps_sigmastar.inc -- capability tables for the SigmaStar backends.
 *
 * Included from the #else arm of hal_caps.c's platform chain so that the
 * shared file carries one hook instead of these tables. Kept as a self-
 * contained #if chain: a #elif whose #if lives in another file is not
 * valid C.  Fork-local; see FORK.md.
 *
 * SPDX-License-Identifier: MIT
 */

/* ═══════════════════════════════════════════════════════════════════════
 * ENCODE RATE BY PART
 *
 * The vendor rates each part's MAIN stream -- "3M@20", "5M@30" -- and leaves
 * separate headroom for a substream that the figure does not describe. So
 * this is a main-stream ceiling, and rvd compares main streams against it
 * one at a time rather than summing the pipeline.
 *
 * Keyed by part and not by family, because the family does not determine it:
 * ssc333, ssc335 and ssc337 are all INFINITY6B0 at 3M@20, 3M@30 and 5M@20.
 * The suffixed variants (D/Q/DE/QE/G) differ in package and in how much DRAM
 * they carry, never in the encoder, so they share their base part's rate.
 *
 * The part is not recoverable on the board -- the chip ID is family-wide,
 * ipctool reports SSC33X for every 6B0, and /sys/devices/soc0/machine says
 * only "INFINITY6B0 SSC009A-S01A QFN88" -- so SOC_MODEL arrives from the
 * build. A build that names no part leaves this 0, which rvd reads as
 * unmeasured and does not test against.
 *
 * DRAM is deliberately NOT recorded here even though it varies across these
 * parts (64/128/256MB). It is readable at runtime from /proc/meminfo, and a
 * fact that can be read must not be frozen into the binary.
 * ═══════════════════════════════════════════════════════════════════════ */

/* Infinity6B0 "Ispahan" (SSC33X) */
#if defined(SOC_MODEL_SSC333) || defined(SOC_MODEL_SSC333DE)
#define RSS_ENC_PIXEL_RATE 60000000u /* 3M @ 20fps */
#elif defined(SOC_MODEL_SSC335) || defined(SOC_MODEL_SSC335DE)
#define RSS_ENC_PIXEL_RATE 90000000u /* 3M @ 30fps */
#elif defined(SOC_MODEL_SSC337) || defined(SOC_MODEL_SSC337DE)
#define RSS_ENC_PIXEL_RATE 100000000u /* 5M @ 20fps */

/* Infinity6E "Pudding" (SSC33X) */
#elif defined(SOC_MODEL_SSC336D) || defined(SOC_MODEL_SSC336Q)
#define RSS_ENC_PIXEL_RATE 90000000u /* 3M @ 30fps */
#elif defined(SOC_MODEL_SSC30KD) || defined(SOC_MODEL_SSC30KQ)
#define RSS_ENC_PIXEL_RATE 150000000u /* 5M @ 30fps */
#elif defined(SOC_MODEL_SSC338D) || defined(SOC_MODEL_SSC338Q) || defined(SOC_MODEL_SSC338G)
#define RSS_ENC_PIXEL_RATE 160000000u /* 8M @ 20fps */
#elif defined(SOC_MODEL_SSC339G)
#define RSS_ENC_PIXEL_RATE 240000000u /* 8M @ 30fps */

/* Infinity6C "Maruko" (SSC37X) */
#elif defined(SOC_MODEL_SSC377) || defined(SOC_MODEL_SSC377D) ||                                    \
	defined(SOC_MODEL_SSC377DE) || defined(SOC_MODEL_SSC377QE)
#define RSS_ENC_PIXEL_RATE 150000000u /* 5M @ 30fps */
#elif defined(SOC_MODEL_SSC378DE) || defined(SOC_MODEL_SSC378QE)
#define RSS_ENC_PIXEL_RATE 200000000u /* 8M @ 25fps */

#else
#define RSS_ENC_PIXEL_RATE 0u /* part not named at build time */
#endif

/* ═══════════════════════════════════════════════════════════════════════
 * INFINITY6E (SigmaStar SSC30KQ / SSC338Q)
 * INFINITY6B0 (SigmaStar SSC333 / SSC335 / SSC337)
 *
 * One block for both families. src/star/ is written against the MI ABI
 * vendored in src/star/i6_*.h, which spans the Infinity6 series -- divinus
 * drives infinity6, infinity6e and infinity6b0 through that same i6 HAL and
 * only switches implementation at infinity6c. So the two share every
 * capability here except where marked.
 *
 * Two kinds of false appear below, and the distinction matters:
 *
 *   1. Hardware/SDK facts — the MI SDK genuinely has no equivalent.
 *      These stay false permanently and are commented individually.
 *   2. Not-yet-implemented — the capability may well exist, but the
 *      corresponding hal_* op is not wired up yet. Consumers check these
 *      flags before calling optional ops, so declaring false keeps
 *      rvd/rsd from invoking a NULL vtable entry. Each later phase flips
 *      on the flags it implements (ISP -> phase 3, audio -> 4, OSD -> 5).
 *
 * Since the backend currently populates no video/audio ops at all,
 * everything in category 2 is false. Only fields with a positive value
 * or a permanent-false explanation are listed; the rest default to
 * false/0 via designated initialization.
 * ═══════════════════════════════════════════════════════════════════════ */
#if defined(PLATFORM_INFINITY6E) || defined(PLATFORM_INFINITY6B0)
const rss_hal_caps_t g_hal_caps = {
    /* System info */
    .soc_name = HAL_PLATFORM_NAME,
    /* Real value comes from MI_SYS_GetVersion() at runtime (phase 2);
     * this is only the compile-time fallback string. */
    .sdk_version = "MI",

    /* Encoder — H.264/H.265/MJPEG are all present in hardware
     * (i6_venc_codec: I6_VENC_CODEC_H264/H265/MJPG). */
    .has_h265 = true,
    /* Permanent false: MI rate control offers CBR/VBR/ABR/FIXQP/AVBR
     * (i6_venc_ratemode) with no equivalent of Ingenic's SMART mode. */
    .has_smart_rc = false,

    /* ISP — single sensor for now; MI supports multi-sensor on some parts
     * but raptor's multi-sensor path is Ingenic IMPVI-specific. */
    .has_multi_sensor = false,
    .max_sensors = 1,
    /* Permanent false: T23-specific IMP_ISP_MultiCamera_* API. */
    .has_t23_multicam_api = false,

/*
 * ISP tuning — phase 3. True only where MI has a control that
 * genuinely matches the raptor knob:
 *
 *   defog          MI_ISP_IQ_SetDefog, a toggle. isp_set_defog is
 *                  implemented; the *strength* variants are not,
 *                  because there is no strength to set.
 *   sinter/temper  MI's spatial (NRLuma) and temporal (NR3D) noise
 *                  reduction, both 0..255 as raptor expects.
 *   ae_comp        MI_ISP_AE_SetEVComp.
 *   max_gain       Both ceilings live in MI's AE exposure-limit
 *                  struct.
 *
 * Left false, with the reasoning spelled out in hal_isp.c's OP
 * COVERAGE comment: dpc and drc (MI's are a toggle and a curve, not
 * strengths), bcsh_hue (a 64-entry HSV LUT), highlight_depress and
 * backlight_comp (WDR curve descriptors), and switch_bin -- a tuning
 * binary *is* loaded during hal_init, but Ingenic's runtime
 * bin-switching op has no MI counterpart.
 *
 * INFINITY6B0 leaves all five false, and this is the one place the two
 * families genuinely diverge. Every scalar knob is reached by poking a
 * field at a hardcoded (payload size, manual offset) pair in an opaque
 * MI IQ struct -- see g_iq in src/star/hal_isp.c. Those numbers were
 * read out of an INFINITY6E libmi_isp.so with objdump, and they are
 * properties of that library build, not of the MI API. Nothing else
 * here carries that risk: the rest of the backend calls typed entry
 * points whose ABI divinus already exercises on infinity6b0, whereas
 * the IQ pokes are raptor's alone and divinus has no equivalent to
 * corroborate.
 *
 * A wrong offset does not fail. Access is read-modify-write, so it
 * lands a plausible value in the wrong field of a struct nobody has
 * fully described, and the image quietly degrades with nothing naming
 * the cause. False here means isp_set_* returns RSS_ERR_NOTSUP and rvd
 * leaves the tuning binary in charge -- which is the correct image
 * either way, just not an adjustable one.
 *
 * Deriving them needs no board, only the library:
 *   arm-linux-gnueabihf-objdump -d \
 *       --disassemble=MI_ISP_IQ_GetBrightness libmi_isp.so
 * prints the payload size into the size slot. If the five offsets
 * match INFINITY6E's, this becomes a shared block again.
 */
#if defined(PLATFORM_INFINITY6E)
    .has_defog = true,
    /*
     * has_sinter and has_max_gain are false because the ops are no longer
     * published, which is the same answer Infinity6C reached and for
     * overlapping reasons -- see the OP COVERAGE comment in
     * src/star/hal_isp.c. Spatial denoise had no usable range once the
     * shipped tunings were read; the gain ceilings are in MI's x1024 units
     * and raptor's keys carry Ingenic gain codes, which is not a unit
     * mismatch a HAL can convert away.
     *
     * has_temper is false here, and no longer for a reason that holds across the
     * family -- Infinity6C publishes it, see that block below.
     *
     * The VPE channel's e3DNRLevel is not a strength at all and never will be:
     * it picks the 3DNR reference frame's bit depth, six of its eight values
     * mean "engine off", and that engine is what mirror and flip run on here, so
     * its off position can stall the ISP outright. The level is fixed at
     * STAR_VPE_NR3D_LEVEL and nothing moves it at runtime.
     *
     * The tuning's NR3D module does have a strength, u8TfStrY, and the objection
     * to it was that reaching it meant setting enOpType to manual and discarding
     * the sixteen-entry gain curve. That objection is answered on 6C by writing
     * the field into the sixteen entries instead, which leaves the module in
     * auto -- and the same thing should work here. It has not been done: this
     * family's NR3D is a different layout (1776 bytes against 1912, a 104-byte
     * parameter block against 112) and none of it has been verified against a 6E
     * tuning yet. So this stays false until it is measured, not because it
     * cannot be done.
     */
    .has_sinter = false,
    .has_drc = true,
    .has_temper = false,
    .has_ae_comp = true,
    .has_max_gain = false,
#endif

    /*
     * Audio — every audio cap stays false, and phase 4 does not change
     * that even though capture itself works.
     *
     * has_audio_process_lib, has_agc_mode, has_hpf_cutoff,
     * has_howling_suppress and has_audio_aec_channel all describe MI's
     * VQE features (noise reduction, AGC, high-pass, echo cancellation).
     * All false, and not out of caution -- turning any of them on is a
     * crash:
     *
     *   1. None of the 20 libraries OpenIPC ships for infinity6e defines
     *      a single Iaa* symbol. The whole algorithm surface --
     *      IaaApc_* (the NS/AGC/EQ chain), IaaAec_*, IaaSsl_*, IaaBf_*,
     *      and IaaSrc_* (the resampler, which is why capture rates are
     *      gated) -- is weak-undefined in libmi_ai.so with no provider.
     *   2. The MI_AI_*Vqe* wrappers themselves *are* defined, so nothing
     *      fails at dlopen or at symbol load. MI_AI_EnableVqe is mostly
     *      argument validation and logging.
     *   3. The real call sites (_MI_AI_G726Init and the capture path)
     *      reach IaaApc_GetBufferSize/Init/Config/Run through the PLT
     *      with **no null guard**. An unresolved weak symbol's GOT slot
     *      is 0, so the blx jumps to address 0 and takes the process
     *      down.
     *
     * MI's own reference agrees the packs are gone: the API "no longer
     * includes the associated algorithm functions" from version 2.19.
     * Doing any of this in software instead would belong in rad, not
     * here, and is explicitly out of scope.
     *
     * has_alc_gain and has_digital_gain describe two separate gain stages;
     * MI has one input gain control, and audio_set_volume owns it.
     * See hal_audio.c's OP COVERAGE comment.
     */

    /* System — all three describe Ingenic internals (xburst2 core, IMP SDK
     * generation, IMPVI multi-sensor calling convention) and are
     * permanently false for any non-Ingenic vendor. */
    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,

    /* Limits — the Infinity6 family exposes 9 addressable VENC channels
     * (I6_VENC_CHN_NUM in both divinus's i6_venc.h and waybeam's
     * sigmastar_types.h; divinus allocates channel state for all 9 and
     * iterates them, for every family it drives through the i6 HAL).
     * An earlier 3 here came from misreading
     * MI_VENC_MAX_CHN_NUM_PER_DC, which is the per-device-group limit and
     * not the total. Capped at RSS_MAX_ENC_CHANNELS (8) since that is
     * raptor's array bound, so 8 is the most this field can honestly
     * advertise.
     *
     * OSD limits (phase 5): max_osd_regions is STAR_OSD_REGION_MAX, the
     * backend's own tracking bound rather than an MI limit -- MI
     * publishes none and neither reference probes for one, so this
     * advertises what the backend will actually honour. max_osd_groups is
     * 4 because a group here *is* an encoder channel with a bound VPE
     * port, and there are four ports. The has_osd_* flags all stay false:
     * MI has only OSD and COVER region types (no mosaic, no
     * RSS_OSD_PIC_RMEM), no group callback, and while MI's display attr
     * does carry an invert sub-struct, rss_osd_region_t has no field to
     * drive it, so claiming the capability would promise something raptor
     * cannot ask for.
     *
     * max_fs_channels is 4 because a raptor framesource channel is a VPE
     * output port (src/star/hal_framesource.c) and a VPE channel has four
     * of them: divinus's teardown disables ports 0..3 (i6_hal.c:365) and
     * waybeam uses 0 and 1. Measured on an SSC30KQ: all four ports accept
     * MI_VPE_SetPortMode at 640x360 NV12. The port count is a VPE property
     * rather than a per-family one, so INFINITY6B0 inherits it unmeasured. */
    .max_enc_channels = 8,
    .max_fs_channels = 4,
    /*
     * Rated main-stream ceiling for this part; see the table at the top of
     * this file. rvd only warns against it, because the one part measured
     * beats its own rating: an ssc333 at 2304x1296 H.264 30fps delivers 901
     * frames in 30s -- 30.0fps, no drops -- which is 89.6M pixels/second
     * against a rated 60M. The number earns its place by naming the envelope
     * a misbehaving board has left, not by taking frame rate off one that
     * works.
     */
    .max_enc_pixel_rate = RSS_ENC_PIXEL_RATE,
#if defined(SOC_MODEL_SSC333)
    /*
     * The JPEG ceiling, unlike the rate, is a real failure: past it the
     * channel is created, its ring is allocated, and it then never produces
     * a frame. rvd logs "enc_poll failing (chn N, last=-110)" and /snap.jpg
     * answers 503 after its timeout.
     *
     * It matches no documented limit. The i6b0 JPE HAL caps at 2688 wide and
     * 2688*2688 pixels, and MI refuses a channel past max_width *
     * max_height = 2688*1944 = 5225472; this starts failing below three
     * quarters of that. Not memory either -- it persists with 9MB of the
     * 32MB MMA heap free and a 9.79MB contiguous hole -- nor throughput
     * (20fps fails exactly as 30fps does), nor width or height alone, since
     * 2304x1080 and 1920x1296 both work at equal pixel counts.
     *
     * The likeliest reading is that a 3M part will not carry a 3M JPEG
     * channel while the H.264 channel is also at 3M, which would make this
     * ssc337's problem at 5M rather than at 3M. Untested, so ssc335 and
     * ssc337 stay 0 rather than inherit a number measured elsewhere.
     *
     * Swept on an SSC333 with an SC3336: 2304x1248 (2875392) is the largest
     * that works, 2304x1264 (2912256) and up do not. It is marginal rather
     * than hard at the boundary -- one restart in six did produce a frame at
     * 2304x1296 -- so this is the last size that worked every time.
     */
    .max_jpeg_pixels = 2875392u,
#endif
    /* Keep in step with STAR_OSD_REGION_MAX in src/star/star_state.h.
     * Not referenced symbolically because this file is compiled for every
     * platform and must not pull in the MI headers. */
    .max_osd_regions = 16,
    .max_osd_groups = 4,
};

/* ═══════════════════════════════════════════════════════════════════════
 * INFINITY6C (SigmaStar SSC377 / SSC378 / SSC379)
 *
 * Deliberately its own block rather than an arm of the one above. The two
 * share silicon lineage but not an ABI: MI 3.0 gives MI_SYS and MI_RGN a
 * leading SoC id, MI_VENC a leading device, promotes the ISP to a pipeline
 * stage and moves scaling to SCL. src/infinity6c/ is the backend.
 *
 * Capture, encode, OSD, audio input and the ISP tuning surface are all
 * implemented (SNR, VIF, ISP, SCL, VENC, RGN, AI) and all board-verified.
 * Consumers check these flags precisely so they do not call into a vtable
 * slot that is NULL.
 *
 * One thing the flags cannot express, and it is the surprise of the
 * family: orientation and the 3DNR level are settable only while the ISP
 * channel is being created. A running channel refuses the write, so those
 * three are config settings rather than runtime ones. See hal_isp.c.
 *
 * A false flag is not the only thing keeping an unimplemented op from
 * being called, though, and relying on it that way is the trap this block
 * is worth re-reading for. rvd calls some ops unconditionally and treats
 * RSS_ERR_NOTSUP from them as fatal to the stream -- the encoder group ops
 * and bind among them -- so those must be published whatever the flags
 * say. The flags gate what rvd *chooses* to ask for. They do not gate its
 * pipeline setup.
 *
 * There is no audio capability to declare, which is worth stating so the
 * silence is not read as an omission: rss_hal_caps_t carries no audio
 * fields at all. What the audio backend does and does not implement is
 * recorded at the vtable in src/infinity6c/hal_common.c and argued op by
 * op in hal_audio.c's OP COVERAGE comment.
 *
 * Both channel limits are 4, and both are the *same* four: a raptor
 * framesource channel is an SCL output port, and an encoder channel is fed
 * by one, so a stream costs one port and the scaler's port count caps
 * both. That number is derived rather than carried over from Infinity6E --
 * mi_scl.ko answers it outright, since _MI_SCL_IMPL_GetPassOutputPortNum
 * is a four-byte function returning 4 (its input counterpart returns 1).
 * The ISP's equivalents are 1 in and 3 out, which is why the ISP sits
 * upstream of the fan-out rather than being it.
 *
 * max_enc_channels is therefore 4 and not the 8 or 12 VENC would accept.
 * MI addresses 12 channels across two devices (H.26x from 0, MJPEG from 8),
 * but a channel with nothing bound to it encodes nothing, so advertising
 * more than the scaler can feed would promise streams that fail at bind
 * time. Note also that MI_VENC_MAX_CHN_NUM_PER_DC is 3 on this part as on
 * Infinity6E and is still the per-device-group limit rather than a total;
 * it was misread that way once already, and MI 3.0's explicit device layer
 * makes it look more like a total than it is.
 *
 * has_rotation is true but coarser than the flag suggests: rotation lives
 * on the SCL channel, so it turns every stream at once. hal_fs_set_rotation
 * accepts it for channel 0 and refuses it elsewhere rather than quietly
 * rotating streams the caller did not name.
 *
 * has_color2grey stays false even though monochrome works, which looks wrong
 * until you check what the flag gates: rvd maps it to enc_set_color2grey, an
 * *encoder* op, and this backend has no such thing. Grey here is
 * MI_ISP_IQ_SetColorToGray, one setting on the single ISP channel every stream
 * draws from, reached through isp_set_running_mode. A true flag would advertise
 * per-channel greying that nothing can service, and an encoder op that greyed
 * both streams at once would be worse than an absent one.
 *
 * The ISP flags that are true are the ones whose ops hal_isp.c publishes:
 * defog, drc and ae_comp. Nothing in raptor reads them today, so they are
 * advertising rather than gating; they are set because a wrong advertisement is
 * what sent the last reader looking in the wrong place.
 *
 * Sharpness has neither, and the missing flag is not why. rss_hal_caps_t has
 * never carried one -- rvd asks the vtable directly for that knob -- and the op
 * itself is NULL here now; see the note above hal_isp_set_sharpness.
 *
 * The permanent falses are the same three as above and for the same
 * reason -- xburst2, the IMP SDK generation and the IMPVI calling
 * convention are Ingenic internals.
 * ═══════════════════════════════════════════════════════════════════════ */
#elif defined(PLATFORM_INFINITY6C)
const rss_hal_caps_t g_hal_caps = {
    .soc_name = HAL_PLATFORM_NAME,
    /*
     * A family name, not a release. MI_SYS_GetVersion answers with the
     * running library's own string and hal_sys_get_version passes that
     * straight through, so anything wanting the exact drop asks the op
     * rather than reading this.
     */
    .sdk_version = "MI",

    /* Encoder */
    .has_h265 = true,
    .has_rotation = true,
    .has_set_bitrate = true,
    /*
     * has_gop_attr stays false: enc_set_gop_attr and enc_get_gop_attr are
     * not published, and a flag whose ops are absent promises what nothing
     * can service. Plain enc_set_gop is implemented and is not gated by it.
     */
    .has_gop_attr = false,
    /* AVBR is what the capped and smart modes map onto; see hal_encoder.c. */
    .has_capped_rc = true,
    .has_smart_rc = true,

    /* ISP — only what hal_isp.c publishes an op for; see above for the rest. */
    .has_color2grey = false,
    .has_defog = true,
    /*
     * has_sinter is false because the op is not published: the only field
     * NrLumaAdv exposes is a blend weight the shipped tunings already run at
     * maximum, so the knob's whole usable range was its own default. See the
     * note in src/infinity6c/hal_isp.c. Temporal denoise is unaffected.
     */
    .has_sinter = false,
    .has_drc = true,
    /*
     * has_temper is true here and false on Infinity6E, and the difference is the
     * shape rather than the silicon. Both families' channel-level route is still
     * useless -- e3DNRLevel picks the 3DNR reference frame's bit depth, not a
     * strength, and the level stays fixed at the seed hal_common sets because
     * the flip predicates need it non-zero. What this SoC's backend gained is a
     * way to reach the tuning's own NR3D strength, u8TfStrY, without paying for
     * it: IQ_GAINRUN writes the field into all sixteen gain entries and leaves
     * enOpType in auto, so the module's curve is still the one MI interpolates.
     * The same thing is possible on 6E and has not been done there yet.
     */
    .has_temper = true,
    .has_ae_comp = true,
    .max_sensors = 1,

    /* Limits — both counts are the scaler's four output ports. */
    .max_enc_channels = 4,
    .max_fs_channels = 4,
    /*
     * Rated main-stream ceiling for this part; see the table at the top of
     * this file. Warn-only, and max_jpeg_pixels stays 0 because no 6C part
     * has been swept for a JPEG ceiling.
     */
    .max_enc_pixel_rate = RSS_ENC_PIXEL_RATE,

    /*
     * OSD limits. A group here is a VENC channel that regions attach to, so
     * there are as many as there are encoder channels; regions are bounded by
     * the backend's own tracking array rather than by anything MI publishes.
     * Keep in step with I6C_OSD_REGION_MAX and I6C_MAX_CHN in
     * src/infinity6c/infinity6c_state.h -- not referenced symbolically because
     * this file is compiled for every platform and must not pull in the MI
     * headers.
     *
     * The has_osd_* flags stay false for the reason the Infinity6E block gives:
     * MI has only OSD and COVER region types, no mosaic and no group callback,
     * and rss_osd_region_t has no field to drive the invert attribute with.
     */
    .max_osd_regions = 16,
    .max_osd_groups = 4,

    .uses_xburst2 = false,
    .uses_new_sdk = false,
    .uses_impvi = false,
};

#else
#error                                                                                             \
    "No PLATFORM_* defined. Set one of: PLATFORM_T10 T20 T21 T23 T30 T31 T32 T33 T40 T41 INFINITY6E INFINITY6B0 INFINITY6C"
#endif
