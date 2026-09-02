/*
 * abi_hisi.c -- compile the gen4 (HiMPP V4.0) ABI transcriptions with their
 * _Static_asserts live.
 *
 * The v4_*.h headers under src/hisi_v4/ are hand-transcribed vendor
 * structures, and each pins the offsets it relies on with _Static_assert.
 * The host test suites define _Static_assert away because the layouts are
 * 32-bit ARM EABI and do not hold on x86-64 (nor under -m32, which packs
 * 64-bit members to 4 bytes where the EABI aligns them to 8). So the
 * asserts only ever ran when someone happened to cross-compile the
 * backend. This translation unit exists to be cross-compiled on purpose,
 * with nothing defined away: `make abi-check-hisi CROSS_COMPILE=arm-...-`.
 *
 * Unlike the SigmaStar checks beside it, no vendor SDK is involved. The
 * headers are self-contained by design, so a passing compile says the
 * transcription is internally consistent with the offsets that were probed
 * on the board, not that it matches a header nobody has.
 */
#include "v4_common.h"
#include "v4_sys.h"
#include "v4_video.h"
#include "v4_vi.h"
#include "v4_vpss.h"
#include "v4_isp.h"
#include "v4_isp_tune.h"
#include "v4_rgn.h"
#include "v4_snr.h"
#include "v4_venc.h"
#include "v4_aud.h"

int abi_hisi_compiled(void)
{
    return 1;
}
