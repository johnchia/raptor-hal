/*
 * abi_probe_hisi5.c -- the size oracle for the HiMPP V5 transcriptions.
 *
 * src/hisi_v5/v5_*.h declares the V5 ABI itself, the way src/hisi_v4/v4_*.h
 * does, and pins every layout with _Static_assert. This file is where those
 * numbers come from: it is compiled with the ARM cross-compiler against the
 * vendor headers -- openhisilicon's GPL copy of the 1.0.2.0 set, which is the
 * version OpenIPC's hi3516cv6xx libraries were built from -- and leaves a
 * table of sizeof() in its own section for abi_probe_hisi5.py to read back.
 * Nothing is linked and nothing runs on a board.
 *
 * It is a *probe*, not a check: it says what the headers claim, and the
 * headers can disagree with the libraries. abi-check-hisi5 (Phase 1) is the
 * check, and it compiles the transcriptions rather than the vendor headers.
 *
 * The sizes[] order is the contract with abi_probe_hisi5.py's name list; the
 * magic and the count are what catch the two of them drifting apart.
 *
 * SPDX-License-Identifier: MIT
 */
#include "ot_common_vi.h"
#include "ot_common_vpss.h"
#include "ot_common_venc.h"
#include "ot_common_vb.h"
#include "ot_common_sys.h"
#include "ot_common_isp.h"
#include "ot_common_3a.h"
#include "ot_sns_ctrl.h"
#include "ot_common_region.h"
#include "ot_common_aio.h"
#include "ot_mipi_rx.h"
#define S(t) sizeof(t)
const unsigned int sizes[] __attribute__((section(".rodata.sizes"))) = {
    0xC0DE0001,
    S(ot_vi_dev_attr),
    S(ot_vi_pipe_attr),
    S(ot_vi_chn_attr),
    S(ot_vi_vpss_mode),
    S(ot_vpss_grp_attr),
    S(ot_vpss_chn_attr),
    S(ot_venc_chn_attr),
    S(ot_venc_stream),
    S(ot_venc_pack),
    S(ot_venc_chn_status),
    S(ot_venc_rc_param),
    S(ot_vb_cfg),
    S(ot_vb_pool_cfg),
    S(ot_mpp_chn),
    S(ot_isp_pub_attr),
    S(ot_isp_exp_info),
    S(ot_isp_3a_alg_lib),
    S(ot_isp_sns_obj),
    S(ot_isp_sns_commbus),
    S(ot_rgn_attr),
    S(ot_rgn_chn_attr),
    S(ot_rgn_canvas_info),
    S(ot_aio_attr),
    S(ot_audio_frame),
    S(combo_dev_attr_t),
    S(ot_mpp_version),
    /* Appended by Phase 1, which transcribed them: the two geometry structs
     * every attribute embeds, and the VB readback trio v5_vb.h declares.
     * Appended rather than inserted -- the order is the contract with
     * abi_probe_hisi5.py's name list, so a new struct goes at the end. */
    S(ot_size),
    S(ot_rect),
    S(ot_vb_pool_info),
    S(ot_vb_pool_status),
    S(ot_vb_supplement_cfg),
    /* Appended by Phase 2, in the order v5_video.h, v5_mipi.h, v5_vi.h,
     * v5_vpss.h, v5_venc.h, v5_isp.h transcribe them. Same rule as above:
     * the order is the contract, so new entries go at the end. */
    S(ot_video_frame),
    S(ot_video_frame_info),
    S(ot_video_supplement),
    S(ot_frame_rate_ctrl),
    S(ot_border),
    S(ot_aspect_ratio),
    S(mipi_dev_attr_t),
    S(img_rect_t),
    S(ot_vi_sync_cfg),
    S(ot_vi_timing_blank),
    S(ot_vpss_crop_info),
    S(ot_vpss_grp_param),
    S(ot_venc_attr),
    S(ot_venc_rc_attr),
    S(ot_venc_gop_attr),
    S(ot_venc_pack_info),
    S(ot_venc_start_param),
    S(ot_venc_chn_param),
    S(ot_venc_jpeg_attr),
    S(ot_venc_jpeg_param),
    S(ot_venc_mpf_cfg),
    S(ot_venc_stream_buf_info),
    S(ot_mipi_crop_attr),
    S(ot_isp_sns_attr_info),
    S(ot_crop_info),
};
