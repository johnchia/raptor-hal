/*
 * hal_internal_sigmastar.h -- SigmaStar platform identity and vendor selection.
 *
 * Included from the #else arm of hal_internal.h's platform-name chain, which
 * is why it also owns the unknown-platform #error: reaching here means no
 * Ingenic PLATFORM_* matched, so either this is a SigmaStar part or it is
 * nothing this tree knows.
 *
 * Fork-local. Kept out of hal_internal.h so the shared file carries a hook
 * rather than a vendor block; see FORK.md.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * VENDOR SELECTION
 *
 * hal_internal.h derives HAL_OLD_SDK / HAL_NEW_SDK / HAL_IMPVI_SDK to
 * distinguish IMP *generations*; those are meaningless outside Ingenic.
 * HAL_INGENIC_SDK / HAL_SIGMASTAR_SDK sit one level above them and select
 * which vendor SDK is in play at all. This header defines the SigmaStar half;
 * hal_internal.h defines the Ingenic half by elimination.
 *
 * SigmaStar has two backends, one per MI generation, and which one a platform
 * needs is not a matter of degree:
 *
 *   src/star/        MI 2.x -- Infinity6E, Infinity6B0. Written against the
 *                    ABI vendored in sigmastar-headers/infinity6e. One backend
 *                    covers both, so either needs only its own caps block.
 *   src/infinity6c/  MI 3.0 -- Infinity6C. MI_SYS and MI_RGN take a leading
 *                    SoC id, MI_VENC a leading device, the ISP is a pipeline
 *                    stage rather than tuning calls alone, and SCL holds the
 *                    scaling role VPE had. Struct layouts differ even where a
 *                    signature does not.
 *
 * A new family therefore needs a caps block, and a backend only if its MI
 * generation has no home here yet.
 *
 * ═══════════════════════════════════════════════════════════════════════
 * VENDOR SDK INCLUDES -- there are none, by design
 *
 * SigmaStar's MI SDK is split per module (libmi_sys, libmi_venc, ...) and
 * SigmaStar publishes no redistributable headers for it, so the backend does
 * not include vendor headers or link -lmi_* at all: the per-module *_load.h
 * files carry the ABI declarations together with dlopen-based loaders, and the
 * backend includes those directly.
 *
 * Two consequences worth stating: the build needs no MI libraries present, and
 * the binary binds to whatever MI stack the device itself carries -- which
 * matters because that stack is coupled to the running kernel.
 * ═══════════════════════════════════════════════════════════════════════
 *
 * SPDX-License-Identifier: MIT
 */

#if defined(PLATFORM_INFINITY6E)
#define HAL_PLATFORM_NAME "INFINITY6E"
#define HAL_SIGMASTAR_SDK
#elif defined(PLATFORM_INFINITY6B0)
#define HAL_PLATFORM_NAME "INFINITY6B0"
#define HAL_SIGMASTAR_SDK
#elif defined(PLATFORM_INFINITY6C)
#define HAL_PLATFORM_NAME "INFINITY6C"
#define HAL_SIGMASTAR_SDK
#else
/* HiSilicon platforms, and the unknown-platform #error; chained here rather
 * than from hal_internal.h so a third vendor costs the shared file nothing. */
#include "hal_internal_hisilicon.h"
#endif
