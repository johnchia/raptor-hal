#!/usr/bin/env python3
"""Print the sizeof() table a cross-compiled abi_probe_hisi5.c left in .rodata.sizes.

    arm-openipc-linux-musleabi-gcc -std=c11 -I<openhisilicon>/kernel/include/hi3516cv6xx -c -o abi_probe_hisi5.o abi_probe_hisi5.c
    arm-openipc-linux-musleabi-objcopy -O binary --only-section=.rodata.sizes abi_probe_hisi5.o abi_probe_hisi5.bin
    python3 abi_probe_hisi5.py abi_probe_hisi5.bin

The name list must match the order of the sizes[] initialiser in abi_probe_hisi5.c.
"""
import struct, sys

NAMES = ("MAGIC ot_vi_dev_attr ot_vi_pipe_attr ot_vi_chn_attr ot_vi_vpss_mode "
         "ot_vpss_grp_attr ot_vpss_chn_attr ot_venc_chn_attr ot_venc_stream ot_venc_pack "
         "ot_venc_chn_status ot_venc_rc_param ot_vb_cfg ot_vb_pool_cfg ot_mpp_chn "
         "ot_isp_pub_attr ot_isp_exp_info ot_isp_3a_alg_lib ot_isp_sns_obj ot_isp_sns_commbus "
         "ot_rgn_attr ot_rgn_chn_attr ot_rgn_canvas_info ot_aio_attr ot_audio_frame "
         "combo_dev_attr_t ot_mpp_version "
         "ot_size ot_rect ot_vb_pool_info ot_vb_pool_status ot_vb_supplement_cfg "
         "ot_video_frame ot_video_frame_info ot_video_supplement ot_frame_rate_ctrl "
         "ot_border ot_aspect_ratio mipi_dev_attr_t img_rect_t ot_vi_sync_cfg "
         "ot_vi_timing_blank ot_vpss_crop_info ot_vpss_grp_param ot_venc_attr "
         "ot_venc_rc_attr ot_venc_mjpeg_fixqp ot_venc_gop_attr ot_venc_pack_info ot_venc_start_param "
         "ot_venc_chn_param ot_venc_jpeg_attr ot_venc_jpeg_param ot_venc_mpf_cfg "
         "ot_venc_stream_buf_info ot_mipi_crop_attr ot_isp_sns_attr_info "
         "ot_crop_info").split()

b = open(sys.argv[1], "rb").read()
v = struct.unpack("<%dI" % (len(b) // 4), b)
if v[0] != 0xC0DE0001:
    sys.exit("bad magic %#x: wrong section or wrong endianness" % v[0])
if len(v) != len(NAMES):
    sys.exit("%d values for %d names: abi_probe_hisi5.c and this list disagree" % (len(v), len(NAMES)))
for n, x in zip(NAMES[1:], v[1:]):
    print("%-22s %6d" % (n, x))
