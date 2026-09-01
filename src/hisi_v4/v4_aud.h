/*
 * hisi_v4/v4_aud.h -- AI (audio input) and the inner audio codec, gen4
 *
 * Two ABIs meet here and they are not the same kind of thing:
 *
 *   - HI_MPI_AI_*, the capture pipeline, exported by libmpi.so (with hard
 *     DT_NEEDED references into libupvqe/libdnvqe/libVoiceEngine, which is
 *     why hisi_mpi_open loads those first and RTLD_GLOBAL -- unlike
 *     SigmaStar's weak-undefined arrangement, a missing VQE library here
 *     fails the dlopen rather than leaving NULL entry points to fall into).
 *
 *   - /dev/acodec, the inner analog codec, which is not MPI at all: an
 *     ioctl surface owned by a platform driver. Sample rate, mic gain,
 *     input volume and mute all live there. The vendor's own sample
 *     configures the codec through these ioctls and then never touches it
 *     from MPI again, and this backend does the same.
 *
 * PROVENANCE. Structs transcribed from the Hi3516EV200 SDK V1.0.1.0
 * headers (mpp/include/hi_comm_aio.h, acodec.h) and cross-checked against
 * ref/openhisilicon/include/comm_aio.h (GPL v3), which agrees field for
 * field. Sizes and offsets verified with the same compiled-probe method as
 * v4_isp_tune.h; note AUDIO_FRAME_S's u64VirAddr, which despite the name
 * is an array of two *pointers* -- 4 bytes each here -- while u64PhyAddr
 * really is two 64-bit words, so the struct is 56 bytes on ARM32 and the
 * probe confirms it.
 *
 * The ioctl command words are rebuilt from acodec.h's IOC_NR_* enum with
 * the standard _IOWR encoding; the request numbers are part of the
 * driver's ABI exactly the way struct offsets are.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V4_AUD_H
#define HISI_V4_AUD_H

#include "v4_common.h"

#include <sys/ioctl.h>

/* ================================================================
 * AIO ATTRIBUTES -- AIO_ATTR_S
 * ================================================================ */

/* AUDIO_BIT_WIDTH_E */
#define V4_AUD_BIT_WIDTH_8 0
#define V4_AUD_BIT_WIDTH_16 1
#define V4_AUD_BIT_WIDTH_24 2

/* AIO_MODE_E */
#define V4_AUD_MODE_I2S_MASTER 0
#define V4_AUD_MODE_I2S_SLAVE 1

/* AUDIO_SOUND_MODE_E */
#define V4_AUD_SOUND_MONO 0
#define V4_AUD_SOUND_STEREO 1

/* AIO_I2STYPE_E */
#define V4_AUD_I2S_INNERCODEC 0

typedef struct {
    int sample_rate; /* AUDIO_SAMPLE_RATE_E -- the enum values ARE the Hz */
    int bit_width;
    int work_mode;
    int sound_mode;
    unsigned int ex_flag;
    unsigned int frm_num;         /* device-side buffer, frames [2..300] */
    unsigned int pt_num_per_frm;  /* samples per frame */
    unsigned int chn_cnt;         /* channels on the FS: 1/2/4/8 */
    unsigned int clk_sel;
    int i2s_type;
} v4_aio_attr;

_Static_assert(sizeof(v4_aio_attr) == 40, "AIO_ATTR_S is 40 bytes");
_Static_assert(offsetof(v4_aio_attr, frm_num) == 20, "u32FrmNum at +20");
_Static_assert(offsetof(v4_aio_attr, i2s_type) == 36, "enI2sType at +36");

/* ================================================================
 * FRAMES -- AUDIO_FRAME_S / AEC_FRAME_S / AI_CHN_PARAM_S
 * ================================================================ */

typedef struct {
    int bit_width;
    int sound_mode;
    unsigned char *vir_addr[2]; /* the SDK spells this u64VirAddr; it is
                                 * two pointers, not two u64s */
    unsigned long long phy_addr[2];
    unsigned long long timestamp; /* microseconds */
    unsigned int seq;
    unsigned int len; /* bytes per channel */
    unsigned int pool_id[2];
} v4_audio_frame;

_Static_assert(sizeof(v4_audio_frame) == 56, "AUDIO_FRAME_S is 56 bytes");
_Static_assert(offsetof(v4_audio_frame, vir_addr) == 8, "u64VirAddr at +8");
_Static_assert(offsetof(v4_audio_frame, phy_addr) == 16, "u64PhyAddr at +16");
_Static_assert(offsetof(v4_audio_frame, timestamp) == 32, "u64TimeStamp at +32");
_Static_assert(offsetof(v4_audio_frame, len) == 44, "u32Len at +44");

typedef struct {
    v4_audio_frame ref_frame;
    int valid;
    int sys_bind;
} v4_aec_frame;

_Static_assert(sizeof(v4_aec_frame) == 64, "AEC_FRAME_S is 64 bytes");

typedef struct {
    unsigned int usr_frm_depth;
} v4_ai_chn_param;

_Static_assert(sizeof(v4_ai_chn_param) == 4, "AI_CHN_PARAM_S is 4 bytes");

/* AI error codes this backend tells apart, precomputed from HI_DEF_ERR
 * (0xA0000000 | mod 21 << 16 | level 4 << 13 | errid): an empty queue and
 * a missing queue are flow control, everything else is a fault. */
#define V4_ERR_AI_BUF_EMPTY 0xA015800Eu
#define V4_ERR_AI_NOBUF 0xA015800Du

/* ================================================================
 * THE INNER CODEC -- /dev/acodec
 * ================================================================ */

#define V4_ACODEC_PATH "/dev/acodec"

/* ACODEC_FS_E -- the I2S FS divider select, NOT a rate in Hz. */
static inline int v4_acodec_fs(int rate)
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

/*
 * The ioctl words, rebuilt from acodec.h's IOC_NR_* enum. Only the ones
 * this backend issues are named; the enum positions of the rest still
 * matter (they fix these numbers) and are recorded in the comments.
 */
#define V4_ACODEC_IOC 'A'
#define V4_ACODEC_SOFT_RESET _IO(V4_ACODEC_IOC, 0x0)
#define V4_ACODEC_SET_INPUT_VOL _IOWR(V4_ACODEC_IOC, 0x1, int)  /* dB, [-87..86] */
#define V4_ACODEC_GET_INPUT_VOL _IOWR(V4_ACODEC_IOC, 0x3, int)
#define V4_ACODEC_SET_I2S1_FS _IOWR(V4_ACODEC_IOC, 0x5, unsigned int)
#define V4_ACODEC_SET_MIXER_MIC _IOWR(V4_ACODEC_IOC, 0x6, unsigned int)
/* 0x7..0x9 are the clock selects */
#define V4_ACODEC_SET_GAIN_MICL _IOWR(V4_ACODEC_IOC, 0xa, unsigned int)
#define V4_ACODEC_SET_GAIN_MICR _IOWR(V4_ACODEC_IOC, 0xb, unsigned int)
/* 0xc..0xf are the DAC/ADC volume pairs */
#define V4_ACODEC_SET_MICL_MUTE _IOWR(V4_ACODEC_IOC, 0x10, unsigned int)
#define V4_ACODEC_SET_MICR_MUTE _IOWR(V4_ACODEC_IOC, 0x11, unsigned int)
/* 0x12..0x15 DAC mute / boost */
#define V4_ACODEC_GET_GAIN_MICL _IOWR(V4_ACODEC_IOC, 0x16, unsigned int)

/* ACODEC_MIXER_E: IN0 is unsupported on EV200/EV300/18EV300; IN1 is the
 * mic path the vendor sample selects. */
#define V4_ACODEC_MIXER_IN1 0x1

/* ================================================================
 * LOADER
 * ================================================================ */

typedef struct {
    int (*fnSetPubAttr)(int dev, const v4_aio_attr *attr);
    int (*fnGetPubAttr)(int dev, v4_aio_attr *attr);
    int (*fnEnable)(int dev);
    int (*fnDisable)(int dev);
    int (*fnEnableChn)(int dev, int chn);
    int (*fnDisableChn)(int dev, int chn);
    int (*fnGetFrame)(int dev, int chn, v4_audio_frame *frm, v4_aec_frame *aec, int timeout_ms);
    int (*fnReleaseFrame)(int dev, int chn, const v4_audio_frame *frm, const v4_aec_frame *aec);
    int (*fnSetChnParam)(int dev, int chn, const v4_ai_chn_param *param);
    int (*fnGetChnParam)(int dev, int chn, v4_ai_chn_param *param);
    int (*fnGetFd)(int dev, int chn);
} v4_aud_impl;

/*
 * v4_aud_load -- bind the AI entry points.
 *
 * Same contract as the other loaders: the capture path is required, the
 * conveniences are optional. RSS_ERR_NOTSUP names the missing symbol via
 * v4_symbol's own diagnostic.
 */
static inline int v4_aud_load(v4_aud_impl *lib, const v4_mpi_libs *libs)
{
    static const char mod[] = "v4_aud";

    memset(lib, 0, sizeof(*lib));

#define V4_AUD_REQ(field, type, hi, gk)                                                            \
    do {                                                                                           \
        if (!(lib->field = (type)v4_symbol(mod, libs, hi, gk)))                                    \
            return RSS_ERR_NOTSUP;                                                                 \
    } while (0)

    V4_AUD_REQ(fnSetPubAttr, int (*)(int, const v4_aio_attr *), "HI_MPI_AI_SetPubAttr",
               "GK_API_AI_SetPubAttr");
    V4_AUD_REQ(fnEnable, int (*)(int), "HI_MPI_AI_Enable", "GK_API_AI_Enable");
    V4_AUD_REQ(fnDisable, int (*)(int), "HI_MPI_AI_Disable", "GK_API_AI_Disable");
    V4_AUD_REQ(fnEnableChn, int (*)(int, int), "HI_MPI_AI_EnableChn", "GK_API_AI_EnableChn");
    V4_AUD_REQ(fnDisableChn, int (*)(int, int), "HI_MPI_AI_DisableChn", "GK_API_AI_DisableChn");
    V4_AUD_REQ(fnGetFrame, int (*)(int, int, v4_audio_frame *, v4_aec_frame *, int),
               "HI_MPI_AI_GetFrame", "GK_API_AI_GetFrame");
    V4_AUD_REQ(fnReleaseFrame,
               int (*)(int, int, const v4_audio_frame *, const v4_aec_frame *),
               "HI_MPI_AI_ReleaseFrame", "GK_API_AI_ReleaseFrame");

#undef V4_AUD_REQ

    lib->fnGetPubAttr = (int (*)(int, v4_aio_attr *))v4_symbol_opt(libs, "HI_MPI_AI_GetPubAttr",
                                                                   "GK_API_AI_GetPubAttr");
    lib->fnSetChnParam = (int (*)(int, int, const v4_ai_chn_param *))v4_symbol_opt(
        libs, "HI_MPI_AI_SetChnParam", "GK_API_AI_SetChnParam");
    lib->fnGetChnParam = (int (*)(int, int, v4_ai_chn_param *))v4_symbol_opt(
        libs, "HI_MPI_AI_GetChnParam", "GK_API_AI_GetChnParam");
    lib->fnGetFd = (int (*)(int, int))v4_symbol_opt(libs, "HI_MPI_AI_GetFd", "GK_API_AI_GetFd");

    return RSS_OK;
}

static inline void v4_aud_unload(v4_aud_impl *lib)
{
    /* No handle of its own to drop; hisi_mpi_close owns those. */
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V4_AUD_H */
