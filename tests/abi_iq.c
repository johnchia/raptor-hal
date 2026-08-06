/*
 * abi_iq.c -- assert the IQ table's payload sizes and manual offsets
 * against SigmaStar's own headers.
 *
 * hal_isp.c drives ~340 MI_ISP_{IQ,AE,AWB}_{Get,Set}<Module> entry points
 * from a descriptor table of (payload size, manual offset, width) rather
 * than 340 structs. That is the right trade, but it means the numbers are
 * unchecked by the compiler -- and one of them was wrong for as long as
 * the table existed: temper's manual offset was 384 bytes early, which
 * put every 3DNR level inside the per-ISO auto array instead of the
 * manual block.
 *
 * So: one translation unit, compiled only when the vendor SDK is present,
 * that derives the same numbers from the vendor structs and fails the
 * build if a row disagrees. Nothing links; _Static_assert does the work.
 *
 *   make -C tests abi-check SIGMASTAR_SDK=/path/to/project/release/include
 *
 * PAYLOAD is deliberately checked as >= rather than ==. It is the size
 * the *blob's* wrapper declares and copies, which for DEFOG is 28 against
 * a 4-byte vendor struct; the buffer has to satisfy the blob. What the
 * vendor headers can settle is that nothing the table addresses falls
 * outside the buffer, and where stManual begins.
 *
 * The header set is chosen by -I: `pudding` is Infinity6E. Pointing this
 * at another generation's directory is a legitimate thing to do -- it is
 * how you find out that a struct moved between SoCs -- so a failure here
 * is a question ("which family?") before it is a defect.
 */

#include <stddef.h>

#include <mi_common_datatype.h>
#include <mi_sys_datatype.h>
#include <mi_isp_datatype.h>
#include <mi_isp_iq_datatype.h>
#include <mi_isp_3a_datatype.h>

#include "i6_common.h"
#include "i6_isp.h"

#define IQ_FITS(row, type)                                                  \
    _Static_assert(I6_ISP_##row##_PAYLOAD >= sizeof(type),                  \
                   #row ": payload is smaller than " #type                  \
                        " -- the library writes past the staging buffer");

#define IQ_MANUAL_AT(row, type)                                             \
    IQ_FITS(row, type)                                                      \
    _Static_assert(I6_ISP_##row##_MANUAL == offsetof(type, stManual),        \
                   #row ": manual offset is not offsetof(" #type            \
                        ", stManual) -- writes land in stAuto");

I6_ISP_IQ_AUTOMAN_ROWS(IQ_MANUAL_AT)
I6_ISP_IQ_FLAT_ROWS(IQ_FITS)

/*
 * The flat rows all write their one field at offset 0, which is only
 * meaningful if that field really is first.
 */
_Static_assert(offsetof(MI_ISP_IQ_DEFOG_TYPE_t, bEnable) == 0, "defog enable at 0");
_Static_assert(offsetof(MI_ISP_IQ_COLORTOGRAY_TYPE_t, bEnable) == 0, "gray enable at 0");
_Static_assert(offsetof(MI_ISP_AE_EV_COMP_TYPE_t, s32EV) == 0, "EV comp s32EV at 0");

/*
 * s32EV is signed, and hal_isp.c's mapping depends on that: EV
 * compensation is centred, not a positive-only boost.
 */
_Static_assert((__typeof__(((MI_ISP_AE_EV_COMP_TYPE_t *)0)->s32EV))-1 < 0,
               "s32EV is signed");

/*
 * Anti-flicker: the two enums are the same width and the same length, and
 * MI orders 60 Hz before 50 Hz. The translation in hal_isp.c exists for
 * this and nothing else, so pin the vendor's order here.
 */
_Static_assert((int)SS_AE_FLICKER_TYPE_DISABLE == 0, "flicker DISABLE");
_Static_assert((int)SS_AE_FLICKER_TYPE_60HZ == 1, "flicker 60HZ precedes 50HZ");
_Static_assert((int)SS_AE_FLICKER_TYPE_50HZ == 2, "flicker 50HZ follows 60HZ");

/*
 * The AE structs day/night reads. i6_isp.h asserts its own sizes; these
 * are the vendor-side halves of those, which it cannot see.
 */
_Static_assert(sizeof(MI_ISP_AE_HW_STATISTICS_t) == sizeof(i6_isp_ae_hw_stats),
               "AE HW stats size");
_Static_assert(offsetof(MI_ISP_AE_HW_STATISTICS_t, nAvg) == 8,
               "AE HW stats cells follow nBlkX/nBlkY");
_Static_assert(offsetof(MI_ISP_AE_AVGS, uAvgY) == I6_ISP_AE_CELL_Y,
               "AE cell luma lane");
_Static_assert(sizeof(MI_ISP_AE_AVGS) == I6_ISP_AE_CELL_SZ, "AE cell size");
_Static_assert(sizeof(CusAEInfo_t) == 65, "CusAEInfo_t is the 65 bytes the wrapper copies");
_Static_assert(sizeof(i6_isp_ae_status) >= sizeof(CusAEInfo_t),
               "AE status must hold all of CusAEInfo_t");
_Static_assert(offsetof(CusAEInfo_t, Shutter) == offsetof(i6_isp_ae_status, shutterUs),
               "AE status shutter offset");
_Static_assert(offsetof(CusAEInfo_t, AvgBlkX) == offsetof(i6_isp_ae_status, avgBlkX),
               "AE status grid dimension offset");
_Static_assert(offsetof(CusAEInfo_t, PreAvgY) == offsetof(i6_isp_ae_status, preAvgY),
               "AE status PreAvgY offset");
_Static_assert(sizeof(MI_ISP_AE_EXPO_LIMIT_TYPE_t) == sizeof(i6_isp_exp),
               "AE exposure limit size");
_Static_assert(offsetof(MI_ISP_AE_EXPO_LIMIT_TYPE_t, u32MaxShutterUS)
                       == offsetof(i6_isp_exp, maxShutterUs),
               "AE exposure limit max shutter offset");
_Static_assert(offsetof(MI_ISP_AE_EXPO_LIMIT_TYPE_t, u32MaxSensorGain)
                       == offsetof(i6_isp_exp, maxSensorGain),
               "AE exposure limit gain minima precede the maxima");
