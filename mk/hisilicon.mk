# mk/hisilicon.mk -- HiSilicon (HiMPP) build settings.
#
# Fork-local, -included from the top of the Makefile so the shared file carries
# a hook instead of a vendor block; see FORK.md. Nothing here runs for an
# Ingenic or SigmaStar PLATFORM: VENDOR is left untouched and every shared-file
# branch keyed on it behaves exactly as if this file were absent.
#
# Usage:
#   make PLATFORM=HI3516EV300 CROSS_COMPILE=arm-openipc-linux-musleabi-
#
# NOTE THE TUPLE: musleabi, not musleabihf.
#
# Gen4 userland is soft-float. Tag_ABI_VFP_args is absent from every vendor
# library on a stock OpenIPC board -- libmpi, libisp, libsecurec, all six
# lib_hi*.so algorithm libraries, all 34 libsns_*.so -- and from majestic
# itself, while Tag_FP_arch reads VFPv4. The FPU is used; the calling
# convention is not. A hard-float build links, loads and runs, and then hands
# garbage to every float argument crossing into MPI. src/hisi_v4/v4_common.h
# carries a #error on __ARM_PCS_VFP so the mistake cannot survive a compile.
# Measured 2026-08-31; see PROBE-hi3516ev300-phase-minus-1.md in the parent
# tree.

# Split by MPP generation rather than by part. V5 (hi3516cv610) is a different
# ABI from V4 and will want its own backend directory when it lands; the union
# is what VENDOR keys on.
#
# EV200 and EV300 share one entry not by assumption but by measurement: an
# EV300 board reports "Hi3516EV200_MPP_V1.0.1.2 B030" on every /proc/umap node.
# One MPP tree, one library set, one ABI. That is why the whole backend guards
# on HAL_HISI_GEN4 and never on a part macro, and why a third gen4 part costs
# a caps block and nothing else.
HISI_GEN4_PLATFORMS := HI3516EV200 HI3516EV300
# HI3516CV610 joins the gen5 list when it lands.
HISI_GEN5_PLATFORMS :=
HISILICON_PLATFORMS := $(strip $(HISI_GEN4_PLATFORMS) $(HISI_GEN5_PLATFORMS))

ifneq ($(filter $(PLATFORM),$(HISILICON_PLATFORMS)),)
VENDOR := hisilicon

# There are no vendor headers to include and no vendor libraries to link. The
# backend declares the HiMPP ABI itself in src/hisi_v4/v4_*.h and reaches the
# libraries through dlopen, the same way the SigmaStar backends reach MI --
# so SDK_INCLUDE stays empty and the shared Makefile drops the -I entirely.
#
# Consequences worth stating, because they are the reason for the shape: the
# build needs no SDK present, and the binary binds to whichever HiMPP the
# device carries, which is coupled to the running kernel.
SDK_INCLUDE :=

# Two backends, because V4 and V5 are different ABIs. Siblings rather than
# src/hisi/v4/, because the shared Makefile's clean rule sweeps src/*/*.o with
# a one-level glob that a nested directory would escape.
ifneq ($(filter $(PLATFORM),$(HISI_GEN5_PLATFORMS)),)
BACKEND_DIR := src/hisi_v5
else
BACKEND_DIR := src/hisi_v4
endif

HAL_COMMON_SRC := $(BACKEND_DIR)/hal_common.c

# Source lists grow one phase at a time, and they list only files that exist.
# An unwritten hal_framesource.c named here would fail the whole build with a
# missing-rule error rather than producing an archive whose ops are simply
# absent -- and "absent op" is the supported way to say a subsystem is not
# ready: RSS_HAL_CALL NULL-guards every vtable entry and returns
# RSS_ERR_NOTSUP. So a Phase 1 build links, runs, and declines the pipeline,
# which is exactly what its acceptance test asks for.
#
# Phase 2 added hal_framesource.c and hal_encoder.c; Phase 3 hal_isp.c (the
# IQ tuning load); Phase 4 adds hal_audio.c; Phase 5 hal_osd.c. hal_gpio is
# vendor-neutral and reused here as the SigmaStar backends reuse it.
VIDEO_SRCS := $(BACKEND_DIR)/hisi_sensor.c \
              $(BACKEND_DIR)/hal_framesource.c \
              $(BACKEND_DIR)/hal_encoder.c \
              $(BACKEND_DIR)/hal_isp.c \
              $(BACKEND_DIR)/hal_nrx.c \
              $(BACKEND_DIR)/hal_dyn.c \
              $(BACKEND_DIR)/hal_osd.c \
              src/hal_gpio.c

# Phase 4: AI capture plus the inner codec. hal_common.c is compiled into
# both archives; hal_audio.c only into this one.
AUDIO_SRCS := $(BACKEND_DIR)/hal_audio.c

endif # hisilicon platform
