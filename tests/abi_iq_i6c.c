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

I6C_ISP_IQ_AUTOMAN_ROWS(IQ_MANUAL_AT)
I6C_ISP_IQ_FLAT_ROWS(IQ_FITS)

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
 * The two modules the table deliberately omits. If a future header drop shrinks
 * either to Infinity6E's size, that is a fact worth learning from a build
 * failure rather than from a picture: it would mean the families had converged
 * and a level could be driven the same way on both.
 */
_Static_assert(sizeof(MI_ISP_IQ_SharpnessType_t) != 1268,
               "sharpness matches Infinity6E's layout -- it may now belong in the table");
_Static_assert(sizeof(MI_ISP_IQ_Nr3dType_t) != 1776,
               "3DNR matches Infinity6E's layout -- it may now belong in the table");

/*
 * The ISP channel parameters, which are where Infinity6C's flip, rotation and
 * 3DNR level live instead of in the tuning API. Asserted here rather than in
 * i6c_isp.h because the bound is the vendor's, not something the blob states.
 */
_Static_assert(sizeof(i6c_isp_para) <= sizeof(MI_ISP_ChnParam_t),
               "the ISP channel parameter block is larger than the struct we send it");
