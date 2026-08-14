/*
 * abi_iq_i6c.c -- assert the Infinity6C IQ table's payload sizes and manual
 * offsets against SigmaStar's own headers.
 *
 * The Infinity6E counterpart is abi_iq.c and the reasoning there applies
 * unchanged: hal_isp.c drives the MI_ISP_{IQ,AE} entry points from a table of
 * (payload size, manual offset, width) rather than from a struct apiece, which
 * is the right trade but leaves the numbers unchecked by the compiler.
 *
 * Two things make this file worth having separately rather than as another -I
 * at abi_iq.c. MI 3.0's tuning calls take a leading device argument, so the
 * function-pointer shape differs; and the maruko header set moves two of the
 * modules, which is exactly the drift a shared translation unit would hide.
 *
 *   make -C tests abi-check-i6c SIGMASTAR_SDK=/path/to/project/release/include
 *
 * PAYLOAD is checked as >= rather than ==, as in abi_iq.c: it is the length
 * the *blob's* wrapper declares and copies, and the buffer has to satisfy the
 * blob. What the vendor headers settle is that nothing the table addresses
 * falls outside the buffer, and where stManual begins.
 *
 * The header set is chosen by -I: `maruko` is Infinity6C. Pointing this at
 * another generation's directory is a legitimate thing to do -- it is how you
 * find out that a struct moved between SoCs -- so a failure here is a question
 * ("which family?") before it is a defect.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stddef.h>

#include <mi_common_datatype.h>
#include <mi_sys_datatype.h>
#include <isp/mi_isp_datatype.h>
#include <isp/mi_isp_ae_datatype.h>
#include <isp/maruko/mi_isp_iq_datatype.h>

#include "i6c_common.h"
#include "i6c_isp.h"

#define IQ_FITS(row, type)                                                                         \
    _Static_assert(I6C_ISP_##row##_PAYLOAD >= sizeof(type),                                        \
                   #row ": payload is smaller than " #type                                         \
                        " -- the library writes past the staging buffer");

#define IQ_MANUAL_AT(row, type)                                                                    \
    IQ_FITS(row, type)                                                                             \
    _Static_assert(I6C_ISP_##row##_MANUAL == offsetof(type, stManual),                             \
                   #row ": manual offset is not offsetof(" #type                                   \
                        ", stManual) -- writes land in stAuto");

/*
 * A vector row: the same payload shape, but the value is a run of same-width
 * fields inside stManual rather than one level. Three things have to hold, and
 * a payload size checks none of them -- where the run starts, how wide each
 * field is, and that the whole run is still inside the parameter block.
 *
 * The last is the one worth spelling out. hal_isp.c writes STRENTGH_NUM fields
 * from STRENGTH without consulting the vendor type again, so a run that ran off
 * the end of the parameter block would spill into whatever follows stManual --
 * which for these two is the end of the payload, so the library would copy from
 * past the caller's buffer.
 */
#define IQ_VECTOR_AT(row, type, param, field, elem)                                                \
    IQ_MANUAL_AT(row, type)                                                                        \
    _Static_assert(I6C_ISP_##row##_STRENGTH == offsetof(param, field),                             \
                   #row ": the strength run does not start at offsetof(" #param ", " #field ")");  \
    _Static_assert(sizeof(elem) == sizeof(((param *)0)->field[0]),                                 \
                   #row ": the table's field width is not " #field "'s");                          \
    _Static_assert(I6C_ISP_##row##_STRENGTH + I6C_ISP_##row##_STRENGTH_NUM * sizeof(elem) <=       \
                       sizeof(param),                                                              \
                   #row ": the strength run reaches past the end of " #param);

I6C_ISP_IQ_AUTOMAN_ROWS(IQ_MANUAL_AT)
I6C_ISP_IQ_FLAT_ROWS(IQ_FITS)
I6C_ISP_IQ_VECTOR_ROWS(IQ_VECTOR_AT)

/*
 * Sharpness's run is six fields where the vendor declares two arrays of three,
 * so the table is right only for as long as u8SharpnessD sits immediately after
 * u8SharpnessUD. Nothing above catches that: IQ_VECTOR_AT checks the run starts
 * at u8SharpnessUD and stays inside the block, and a run of six would satisfy
 * both while spending its second half on u8PreCorUD -- which is a coring
 * threshold, so the picture would soften as the sharpness knob went up.
 */
_Static_assert(offsetof(MI_ISP_IQ_SharpnessParam_t, u8SharpnessD) ==
                   offsetof(MI_ISP_IQ_SharpnessParam_t, u8SharpnessUD) + SHARPNESS_FREQ_NUM,
               "u8SharpnessD no longer follows u8SharpnessUD -- the run is not six bytes");
_Static_assert(I6C_ISP_IQ_SHARPNESS_STRENGTH_NUM == 2 * SHARPNESS_FREQ_NUM,
               "the run is the undirectional and directional gains, three bands each");

/* The same for the denoise run, which is one array and only needs its length. */
_Static_assert(I6C_ISP_IQ_NRLUMAADV_STRENGTH_NUM == NRLUMA_ADV_LEVEL_NUM,
               "the run is u16Strength, one entry per level");

/*
 * The auto array's length is the one number a payload size cannot catch on its
 * own: get it wrong and every manual offset moves by the same amount, so the
 * table stays self-consistent while pointing 16 entries into the wrong place.
 * It is asserted directly for that reason.
 */
_Static_assert(MI_ISP_AUTO_NUM == 16, "the manual offsets are all 8 + 16 * sizeof(param)");

/*
 * Saturation's parameter block, which is where the arithmetic could hide a
 * mistake: it is the only row whose per-entry size is neither 1 nor 4 bytes,
 * and 24 is what puts stManual at 392 rather than anywhere else.
 */
_Static_assert(sizeof(MI_ISP_IQ_SaturationParam_t) == 24,
               "SAT_LUT_X_NUM 5 and SAT_LUT_Y_NUM 6, plus the strength and coring bytes");

/*
 * The one module the table still omits. If a future header drop shrinks it to
 * Infinity6E's size, that is a fact worth learning from a build failure rather
 * than from a picture: it would mean the families had converged and a level
 * could be driven the same way on both.
 *
 * Sharpness was the other one and no longer is -- it is a vector row above. It
 * is still not Infinity6E's 1268-byte layout and the check that says so is now
 * IQ_FITS's, against 6264.
 */
_Static_assert(sizeof(MI_ISP_IQ_Nr3dType_t) != 1776,
               "3DNR matches Infinity6E's layout -- it may now belong in the table");

/*
 * The ISP channel parameters, which are where Infinity6C's flip, rotation and
 * 3DNR level live instead of in the tuning API. Asserted here rather than in
 * i6c_isp.h because the bound is the vendor's, not something the blob states.
 */
_Static_assert(sizeof(i6c_isp_para) <= sizeof(MI_ISP_ChnParam_t),
               "the ISP channel parameter block is larger than the struct we send it");

/*
 * The AE envelope, field by field rather than by size alone. Size is the weakest
 * possible check on this one: eight same-width words in the wrong order are the
 * same 32 bytes, and the vendor's order is not the one it looks like -- the two
 * gain minima sit together ahead of the two maxima, so the natural min/max
 * pairing would put the shutter ceiling this backend writes into u32MaxFNx10,
 * and holding the frame rate would silently do nothing at all.
 */
#define AE_FIELD_AT(ours, theirs)                                                                  \
    _Static_assert(offsetof(i6c_isp_exp, ours) == offsetof(MI_ISP_AE_ExpoLimitType_t, theirs),     \
                   "i6c_isp_exp." #ours " is not at MI_ISP_AE_ExpoLimitType_t." #theirs);

_Static_assert(sizeof(i6c_isp_exp) == sizeof(MI_ISP_AE_ExpoLimitType_t),
               "the AE exposure limit struct is not the vendor's size");
AE_FIELD_AT(minShutterUs, u32MinShutterUS)
AE_FIELD_AT(maxShutterUs, u32MaxShutterUS)
AE_FIELD_AT(minApertX10, u32MinFNx10)
AE_FIELD_AT(maxApertX10, u32MaxFNx10)
AE_FIELD_AT(minSensorGain, u32MinSensorGain)
AE_FIELD_AT(minIspGain, u32MinISPGain)
AE_FIELD_AT(maxSensorGain, u32MaxSensorGain)
AE_FIELD_AT(maxIspGain, u32MaxISPGain)
