/*
 * hisi_v5/v5_aud.h -- AI (audio input) and the inner audio codec, HiMPP V5
 *
 * The same two ABIs gen4's v4_aud.h meets, and the same split:
 *
 *   - ss_mpi_ai_*, the capture pipeline, in libss_mpi_audio.so -- a
 *     library of its own on V5 rather than a corner of libmpi.so, and one
 *     with imports rather than DT_NEEDED entries: audio_upvqe_*,
 *     audio_dnvqe_* and voice_* resolve only if libupvqe.so, libdnvqe.so
 *     and libvoice_engine.so are already mapped RTLD_GLOBAL when it is
 *     opened. v5_aud_open_libs does that in that order. (mpi_sys_bind_*
 *     and ot_mpi_sys_* are satisfied by the sysbind and sysmem libraries
 *     hisi_mpi_open already holds.)
 *
 *   - /dev/ab, the audio buffer module, new on V5: the AI device's ring
 *     is a block of an AB pool rather than a driver-private buffer, and
 *     the pool exists only after ss_mpi_audio_init -- without it
 *     ss_mpi_ai_enable answers NO_MEM with 32 MB of MMZ free, and
 *     /proc/umap/ab shows an empty "ab pub config". The vendor's
 *     sample_comm_audio_init calls exit-then-init; hal_audio.c calls init
 *     once per attach and exit at deinit.
 *
 *   - /dev/acodec, the inner analog codec: sample rate select, mic
 *     mixer, input volume, mic gain and mute, through ioctls a platform
 *     driver owns. The vendor's sample_inner_codec_cfg_audio configures
 *     it through these and never touches it from MPI again.
 *
 * PROVENANCE. openhisilicon kernel/include/hi3516cv6xx/ot_common_aio.h,
 * ss_mpi_audio.h and ot_acodec.h at 1.0.2.0 B051, the header set the rest
 * of hisi_v5 is transcribed from. ot_aio_attr (40) and ot_audio_frame
 * (48) are the Phase -1 survey's numbers; the frame's 48 depends on
 * td_phys_addr_t being 32 bits (CONFIG_PHYS_ADDR_BIT_WIDTH_64 unset), which
 * the first live frame confirms by its timestamp and length landing where
 * this layout says -- hal_audio.c logs both on the first frame for that
 * reason. The ioctl words are ot_acodec.h's IOC_NR_* enum, whose positions
 * differ from gen4's acodec.h: the input/output volume pair sits at 3..6,
 * the boost pair at 7..8, the mic gains at 9..12.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_AUD_H
#define HISI_V5_AUD_H

#include "v5_common.h"

#include <sys/ioctl.h>

/* ================================================================
 * AIO ATTRIBUTES -- ot_aio_attr
 * ================================================================ */

/* ot_audio_bit_width */
#define V5_AUD_BIT_WIDTH_16 1

/* ot_aio_mode */
#define V5_AUD_MODE_I2S_MASTER 0

/* ot_audio_snd_mode */
#define V5_AUD_SOUND_MONO 0
#define V5_AUD_SOUND_STEREO 1

/* ot_aio_i2s_type */
#define V5_AUD_I2S_INNERCODEC 0

typedef struct {
    int sample_rate; /* ot_audio_sample_rate -- the enum values ARE the Hz */
    int bit_width;
    int work_mode;
    int snd_mode;
    unsigned int expand_flag;
    unsigned int frame_num;           /* device-side buffer, frames [2..300] */
    unsigned int point_num_per_frame; /* samples per frame */
    unsigned int chn_cnt;             /* channels on the FS: 1/2/4/8 */
    unsigned int clk_share;
    int i2s_type;
} v5_aio_attr;

_Static_assert(sizeof(v5_aio_attr) == 40, "ot_aio_attr is 40 bytes");
_Static_assert(offsetof(v5_aio_attr, frame_num) == 20, "ot_aio_attr.frame_num at +20");
_Static_assert(offsetof(v5_aio_attr, i2s_type) == 36, "ot_aio_attr.i2s_type at +36");

/* ================================================================
 * FRAMES -- ot_audio_frame / ot_aec_frame / ot_ai_chn_param
 * ================================================================ */

typedef struct {
    int bit_width;
    int snd_mode;
    unsigned char *virt_addr[2];
    unsigned int phys_addr[2];     /* td_phys_addr_t, 32-bit on this kernel config */
    unsigned long long time_stamp; /* microseconds */
    unsigned int seq;
    unsigned int len; /* bytes per channel */
    unsigned int pool_id[2];
} v5_audio_frame;

_Static_assert(sizeof(v5_audio_frame) == 48, "ot_audio_frame is 48 bytes");
_Static_assert(offsetof(v5_audio_frame, virt_addr) == 8, "ot_audio_frame.virt_addr at +8");
_Static_assert(offsetof(v5_audio_frame, phys_addr) == 16, "ot_audio_frame.phys_addr at +16");
_Static_assert(offsetof(v5_audio_frame, time_stamp) == 24, "ot_audio_frame.time_stamp at +24");
_Static_assert(offsetof(v5_audio_frame, len) == 36, "ot_audio_frame.len at +36");

typedef struct {
    v5_audio_frame ref_frame;
    int valid;
    int sys_bind;
} v5_aec_frame;

_Static_assert(sizeof(v5_aec_frame) == 56, "ot_aec_frame is 56 bytes");

typedef struct {
    unsigned int usr_frame_depth;
} v5_ai_chn_param;

_Static_assert(sizeof(v5_ai_chn_param) == 4, "ot_ai_chn_param is 4 bytes");

/* AI error codes this backend tells apart, from OT_DEFINE_ERR (0xA0000000 |
 * OT_ID_AI 21 << 16 | OT_ERR_LEVEL_ERROR 4 << 13 | err): an empty queue and
 * a missing queue are flow control, everything else is a fault. The err
 * ids are V5's, not gen4's -- BUF_EMPTY is 0x16 here, NO_BUF 0x15. */
#define V5_ERR_AI_BUF_EMPTY 0xA0158016u
#define V5_ERR_AI_NO_BUF 0xA0158015u
#define V5_AI_USER_DEPTH_MAX 30 /* OT_MAX_AI_USER_FRAME_DEPTH */
#define V5_AI_USER_DEPTH_MIN 2  /* OT_MIN_AI_USER_FRAME_DEPTH */

/* ================================================================
 * THE INNER CODEC -- /dev/acodec
 * ================================================================ */

#define V5_ACODEC_PATH "/dev/acodec"

/* ot_acodec_fs -- the I2S FS divider select, NOT a rate in Hz. Same table
 * as gen4's. */
static inline int v5_acodec_fs(int rate)
{
    switch (rate) {
    case 8000:
        return 0x1;
    case 11025:
        return 0x2;
    case 12000:
        return 0x3;
    case 16000:
        return 0x4;
    case 22050:
        return 0x5;
    case 24000:
        return 0x6;
    case 32000:
        return 0x7;
    case 44100:
        return 0x8;
    case 48000:
        return 0x9;
    case 64000:
        return 0xa;
    case 96000:
        return 0xb;
    }
    return -1;
}

/* ot_acodec.h's acodec_ioc enum, in order. Only the words this backend
 * issues are named; the positions of the rest fix these numbers. */
#define V5_ACODEC_IOC 'A'
#define V5_ACODEC_SOFT_RESET _IO(V5_ACODEC_IOC, 0x0)
#define V5_ACODEC_SET_I2S1_FS _IOWR(V5_ACODEC_IOC, 0x1, unsigned int)
#define V5_ACODEC_SET_MIXER_MIC _IOWR(V5_ACODEC_IOC, 0x2, unsigned int)
#define V5_ACODEC_SET_INPUT_VOL _IOWR(V5_ACODEC_IOC, 0x3, unsigned int) /* dB, [-78..80] */
/* 0x4 set output vol */
#define V5_ACODEC_GET_INPUT_VOL _IOWR(V5_ACODEC_IOC, 0x5, unsigned int)
/* 0x6 get output vol */
#define V5_ACODEC_ENABLE_BOOSTL _IOWR(V5_ACODEC_IOC, 0x7, unsigned int)
#define V5_ACODEC_ENABLE_BOOSTR _IOWR(V5_ACODEC_IOC, 0x8, unsigned int)
#define V5_ACODEC_SET_GAIN_MICL _IOWR(V5_ACODEC_IOC, 0x9, unsigned int) /* 0..0x1f */
#define V5_ACODEC_SET_GAIN_MICR _IOWR(V5_ACODEC_IOC, 0xa, unsigned int)
#define V5_ACODEC_GET_GAIN_MICL _IOWR(V5_ACODEC_IOC, 0xb, unsigned int)
/* 0xc get gain R; 0xd..0x14 the DAC/ADC volume pairs */
#define V5_ACODEC_SET_MICL_MUTE _IOWR(V5_ACODEC_IOC, 0x15, unsigned int)
#define V5_ACODEC_SET_MICR_MUTE _IOWR(V5_ACODEC_IOC, 0x16, unsigned int)

/*
 * ot_acodec_mixer: which analog input feeds the mic PGA. The vendor's
 * sample says "demo board is pseudo-differential (IN_D), socket board is
 * single-ended (IN1)" and selects IN_D. The bench measurement that picks
 * one is written down at V5_ACODEC_MIXER_DEFAULT in hal_audio.c.
 */
#define V5_ACODEC_MIXER_IN0 0x0
#define V5_ACODEC_MIXER_IN1 0x1
#define V5_ACODEC_MIXER_IN_D 0x2

/* The input volume's range in dB, from the vendor sample's own comment on
 * OT_ACODEC_SET_INPUT_VOLUME: -78 is mute, 80 the top, 20..50 the band
 * where only the analog gain moves. Wider than gen4's [-87..86]. */
#define V5_ACODEC_INPUT_VOL_MIN (-78)
#define V5_ACODEC_INPUT_VOL_MAX 80

/*
 * The mic preamp steps. The header states five bits, 0..0x1f; the driver
 * does not agree. Measured on the CV608 bench board 2026-09-08 through
 * rad's own set-gain: SET_GAIN_MICL takes 0..15 and answers EPERM from 16
 * up, and /proc/umap/acodec reads each step back doubled (12 -> gain_left
 * 24), so the field is four bits of 2 dB and the header's 0x1f is the
 * shadow variable's width, not the codec's. gen4 landed on the same 15
 * from the other side -- there the driver took 16 and the preamp all but
 * muted (v4_aud.h). rad's default gain is 25, so requests are clamped to
 * the last step the driver takes rather than passed through, and the
 * clamp is logged once so a config asking for more is not silently
 * ignored.
 */
#define V5_ACODEC_GAIN_MIC_MAX 15

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    int (*fnAudioInit)(void);
    int (*fnAudioExit)(void);
    int (*fnSetPubAttr)(int dev, const v5_aio_attr *attr);
    int (*fnGetPubAttr)(int dev, v5_aio_attr *attr);
    int (*fnEnable)(int dev);
    int (*fnDisable)(int dev);
    int (*fnEnableChn)(int dev, int chn);
    int (*fnDisableChn)(int dev, int chn);
    int (*fnGetFrame)(int dev, int chn, v5_audio_frame *frm, v5_aec_frame *aec, int timeout_ms);
    int (*fnReleaseFrame)(int dev, int chn, const v5_audio_frame *frm, const v5_aec_frame *aec);
    int (*fnSetChnParam)(int dev, int chn, const v5_ai_chn_param *param);
    int (*fnGetChnParam)(int dev, int chn, v5_ai_chn_param *param);
    int (*fnGetFd)(int dev, int chn);
} v5_aud_impl;

/*
 * v5_aud_open_libs -- the audio tier, on top of what hisi_mpi_open holds.
 *
 * The VQE trio first and RTLD_GLOBAL, because libss_mpi_audio.so imports
 * from them by symbol and RTLD_NOW resolves those at open. A missing
 * algorithm library is a warning, not a failure: the capture path itself
 * needs none of them, but the loader will refuse the MPI library if any
 * symbol it imports is unresolved, and that refusal is the error reported.
 */
static inline int v5_aud_open_libs(v5_mpi_libs *libs)
{
    static const int flags = RTLD_NOW | RTLD_GLOBAL;

    if (libs->audio)
        return RSS_OK;

    libs->upvqe = dlopen("libupvqe.so", flags);
    libs->dnvqe = dlopen("libdnvqe.so", flags);
    libs->voice = dlopen("libvoice_engine.so", flags);
    if (!libs->upvqe || !libs->dnvqe || !libs->voice)
        HAL_LOG_WARN("hisi_mpi: VQE libraries incomplete (up=%p dn=%p voice=%p); "
                     "libss_mpi_audio.so imports from all three",
                     libs->upvqe, libs->dnvqe, libs->voice);

    if (!(libs->audio = dlopen("libss_mpi_audio.so", flags))) {
        HAL_LOG_ERR("hisi_mpi: libss_mpi_audio.so: %s", dlerror());
        return RSS_ERR_NOENT;
    }
    v5_libs_add_search(libs, libs->audio);
    HAL_LOG_DBG("hisi_mpi: libss_mpi_audio.so loaded");
    return RSS_OK;
}

/*
 * v5_aud_load -- bind the AI entry points.
 *
 * Same contract as the other loaders: the capture path is required, the
 * conveniences are optional. RSS_ERR_NOTSUP names the missing symbol via
 * v5_symbol's own diagnostic.
 */
static inline int v5_aud_load(v5_aud_impl *lib, const v5_mpi_libs *libs)
{
    static const char mod[] = "v5_aud";

    memset(lib, 0, sizeof(*lib));

#define V5_AUD_REQ(field, type, name)                                                              \
    do {                                                                                           \
        if (!(lib->field = (type)v5_symbol(mod, libs, name)))                                      \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V5_AUD_REQ(fnAudioInit, int (*)(void), "ss_mpi_audio_init");
    V5_AUD_REQ(fnAudioExit, int (*)(void), "ss_mpi_audio_exit");
    V5_AUD_REQ(fnSetPubAttr, int (*)(int, const v5_aio_attr *), "ss_mpi_ai_set_pub_attr");
    V5_AUD_REQ(fnEnable, int (*)(int), "ss_mpi_ai_enable");
    V5_AUD_REQ(fnDisable, int (*)(int), "ss_mpi_ai_disable");
    V5_AUD_REQ(fnEnableChn, int (*)(int, int), "ss_mpi_ai_enable_chn");
    V5_AUD_REQ(fnDisableChn, int (*)(int, int), "ss_mpi_ai_disable_chn");
    V5_AUD_REQ(fnGetFrame, int (*)(int, int, v5_audio_frame *, v5_aec_frame *, int),
               "ss_mpi_ai_get_frame");
    V5_AUD_REQ(fnReleaseFrame, int (*)(int, int, const v5_audio_frame *, const v5_aec_frame *),
               "ss_mpi_ai_release_frame");

#undef V5_AUD_REQ

    lib->fnGetPubAttr = (int (*)(int, v5_aio_attr *))v5_symbol_opt(libs, "ss_mpi_ai_get_pub_attr");
    lib->fnSetChnParam =
        (int (*)(int, int, const v5_ai_chn_param *))v5_symbol_opt(libs, "ss_mpi_ai_set_chn_param");
    lib->fnGetChnParam =
        (int (*)(int, int, v5_ai_chn_param *))v5_symbol_opt(libs, "ss_mpi_ai_get_chn_param");
    lib->fnGetFd = (int (*)(int, int))v5_symbol_opt(libs, "ss_mpi_ai_get_fd");

    return RSS_OK;
}

static inline void v5_aud_unload(v5_aud_impl *lib)
{
    /* No handle of its own to drop; hisi_mpi_close owns those. */
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_AUD_H */
