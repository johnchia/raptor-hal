/*
 * abi_hisi5.c -- compile the gen5 (HiMPP V5.0) ABI transcriptions with
 * their _Static_asserts live.
 *
 * The v5_*.h headers under src/hisi_v5/ are hand-transcribed vendor
 * structures, and each pins the sizes and offsets it relies on with
 * _Static_assert. Most of those hold on an x86-64 host -- V5's structs are
 * almost all free of pointers, so the host suites do not need them defined
 * away the way the SigmaStar ones do -- but "almost all" is not a check.
 * This translation unit exists to be cross-compiled on purpose, with
 * nothing defined away and every assert in the tree evaluated at the
 * target's ABI: `make abi-check-hisi5 CROSS_COMPILE=arm-...-`.
 *
 * The relationship to abi_probe_hisi5.c beside it is worth stating, since
 * the two look similar and answer different questions. The *probe* compiles
 * openhisilicon's vendor headers and prints sizeof(); it says what the
 * vendor's declaration is. The *check* compiles raptor's transcription of
 * it; it says whether raptor agrees. A number that moves in the probe means
 * the headers changed. A failure here means the transcription is wrong.
 *
 * No vendor SDK is involved, and that is by design: the headers are
 * self-contained, so a passing compile says the transcription is internally
 * consistent with the numbers the probe measured, not that it matches a
 * header nobody has.
 *
 * SPDX-License-Identifier: MIT
 */
#include "v5_common.h"
#include "v5_sys.h"
#include "v5_vb.h"

int abi_hisi5_compiled(void)
{
    return 1;
}
