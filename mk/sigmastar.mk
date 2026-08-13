# mk/sigmastar.mk -- SigmaStar (Infinity6 / MI SDK) build settings.
#
# Fork-local, -included from the top of the Makefile so the shared file carries
# a hook instead of a vendor block; see FORK.md. Nothing here runs for an
# Ingenic PLATFORM: VENDOR stays empty and every shared-file branch keyed on it
# takes the Ingenic path, which is also what happens if this file is absent.
#
# Usage:
#   make PLATFORM=INFINITY6E CROSS_COMPILE=arm-linux-gnueabihf-
#   make PLATFORM=INFINITY6C CROSS_COMPILE=arm-openipc-linux-musleabihf-
#
# Optional:
#   SIGMASTAR_HEADERS - path to the sigmastar-headers submodule
#                       (default: sigmastar-headers)

SIGMASTAR_PLATFORMS := INFINITY6E INFINITY6B0 INFINITY6C

ifneq ($(filter $(PLATFORM),$(SIGMASTAR_PLATFORMS)),)
VENDOR := sigmastar

SIGMASTAR_HEADERS ?= sigmastar-headers

# SigmaStar keys on the chip family rather than an SDK version, because these
# declarations are reconstructions rather than vendor drops and the submodule
# commit is what pins them. There are no MI libraries to link either: the
# loaders in $(BACKEND_DIR)/i6*_load.h reach MI through dlopen.
#
# INFINITY6C names its own family because MI 3.0 shares no struct layout with
# MI 2.x -- even the calls whose signatures are unchanged take different
# payloads. That family is written from the vendor SDK rather than reconstructed
# from a third-party HAL, which is why it does not reuse infinity6e's.
HEADER_FAMILY_INFINITY6E  := infinity6e
HEADER_FAMILY_INFINITY6B0 := infinity6e
HEADER_FAMILY_INFINITY6C  := infinity6c

SDK_INCLUDE := $(SIGMASTAR_HEADERS)/$(HEADER_FAMILY_$(PLATFORM))

# Two backends, because there are two incompatible MI generations. src/star/
# is MI 2.x (Infinity6E, Infinity6B0); src/infinity6c/ is MI 3.0 (Infinity6C),
# where MI_SYS and MI_RGN take a leading SoC id, MI_VENC takes a leading
# device, the ISP is a pipeline stage in its own right and VPE's scaling role
# belongs to SCL. Sharing a translation unit would mean wrapping nearly every
# call site in a macro to hide an argument list, which buys nothing.
#
# Subsystems not yet ported simply omit their ops from the vtable --
# RSS_HAL_CALL NULL-guards every entry and returns RSS_ERR_NOTSUP -- so there
# is no need for stub translation units per unimplemented subsystem.
ifeq ($(PLATFORM),INFINITY6C)
BACKEND_DIR := src/infinity6c
else
BACKEND_DIR := src/star
endif

HAL_COMMON_SRC := $(BACKEND_DIR)/hal_common.c

ifeq ($(PLATFORM),INFINITY6C)
# The capture and encode path, in datapath order, then OSD and the ISP tuning
# ops. hal_gpio is vendor-neutral.
VIDEO_SRCS := $(BACKEND_DIR)/hal_framesource.c \
              $(BACKEND_DIR)/hal_encoder.c \
              $(BACKEND_DIR)/hal_isp.c \
              $(BACKEND_DIR)/hal_osd.c \
              src/hal_gpio.c

# MI 3.0 audio input. Capture only, and no src/hal_dmic.c counterpart: the
# digital microphone is an MI_AI interface here rather than a separate module,
# so it is a value passed to MI_AI_AttachIf and not a library of its own.
AUDIO_SRCS := $(BACKEND_DIR)/hal_audio.c
else
VIDEO_SRCS := $(BACKEND_DIR)/hal_encoder.c \
              $(BACKEND_DIR)/hal_framesource.c \
              $(BACKEND_DIR)/hal_isp.c \
              $(BACKEND_DIR)/hal_osd.c \
              src/hal_gpio.c

AUDIO_SRCS := $(BACKEND_DIR)/hal_audio.c
endif

endif # sigmastar platform
