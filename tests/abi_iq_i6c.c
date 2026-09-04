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

/*
 * A gain row: the value is one field written into every stAuto entry, so the
 * three numbers that have to hold are where the entry array starts, how long an
 * entry is, and where the field sits inside one.
 *
 * The stride is the one worth spelling out, because getting it wrong fails in a
 * way no picture makes obvious. A vector row that miscounts writes into a
 * neighbouring field of the same block; a gain row that miscounts writes into a
 * different field of a *different gain's* entry, so the module misbehaves at one
 * exposure and is correct at the next. AUTO_NUM is asserted against the vendor's
 * own MI_ISP_AUTO_NUM rather than against 16, so a header drop that changed the
 * number of entries is a build failure and not a partly-written run.
 */
#define IQ_GAINRUN_AT(row, type, param, field, elem, fieldoff)                                     \
    IQ_FITS(row, type)                                                                             \
    _Static_assert(I6C_ISP_##row##_AUTO == offsetof(type, stAuto),                                 \
                   #row ": the entry array does not start at offsetof(" #type ", stAuto)");        \
    _Static_assert(I6C_ISP_##row##_ENTRY == sizeof(param),                                         \
                   #row ": the stride is not sizeof(" #param ") -- a write lands in another "      \
                        "gain's entry");                                                           \
    _Static_assert((fieldoff) == offsetof(param, field),                                           \
                   #row ": the field is not at offsetof(" #param ", " #field ")");                 \
    _Static_assert(sizeof(elem) == sizeof(((param *)0)->field),                                    \
                   #row ": the table's field width is not " #field "'s");                          \
    _Static_assert(I6C_ISP_##row##_AUTO_NUM == MI_ISP_AUTO_NUM,                                    \
                   #row ": the run is not one element per auto entry");                            \
    _Static_assert(I6C_ISP_##row##_AUTO + MI_ISP_AUTO_NUM * sizeof(param) ==                       \
                       offsetof(type, stManual),                                                   \
                   #row ": the entry array does not end where stManual begins");

I6C_ISP_IQ_AUTOMAN_ROWS(IQ_MANUAL_AT)
I6C_ISP_IQ_MANUALONLY_ROWS(IQ_MANUAL_AT)
I6C_ISP_IQ_FLAT_ROWS(IQ_FITS)
I6C_ISP_IQ_VECTOR_ROWS(IQ_VECTOR_AT)
I6C_ISP_IQ_GAINRUN_ROWS(IQ_GAINRUN_AT)

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
 * NR3D was the one module the table omitted, and is now a gain row above. The
 * assertion that used to stand here said only that its layout was not
 * Infinity6E's 1776 bytes; IQ_GAINRUN_AT says considerably more than that, so
 * the weaker check is gone rather than kept alongside.
 *
 * The two families have still not converged: 1912 here against 1776 there, from
 * a 112-byte parameter block against a 104-byte one. That is why has_temper is
 * true on this SoC and false on 6E -- see src/caps_sigmastar.inc.
 */

/*
 * Colortrans, field by field, because it is the one module this port composes a
 * payload for rather than writing a single level into: three offsets and nine
 * matrix entries all have to be where the table says, and a payload size checks
 * none of it. The offsets are addressed individually rather than as an array,
 * so each is asserted rather than only the first.
 */
#define CT_PARAM MI_ISP_IQ_ColorTransParam_t
#define CT_AT(ours, theirs)                                                                        \
    _Static_assert(I6C_ISP_IQ_COLORTRANS_##ours ==                                                 \
                       I6C_ISP_IQ_COLORTRANS_MANUAL + offsetof(CT_PARAM, theirs),                  \
                   "COLORTRANS: " #ours " is not at stManual + offsetof("                          \
                   "MI_ISP_IQ_ColorTransParam_t, " #theirs ")");

CT_AT(YOFST, u16Y_OFST)
CT_AT(UOFST, u16U_OFST)
CT_AT(VOFST, u16V_OFST)
CT_AT(MATRIX, u16Matrix)
_Static_assert(I6C_ISP_IQ_COLORTRANS_MAT_NUM == COLORTRANS_MATRIX_NUM,
               "COLORTRANS: the matrix is not the vendor's number of entries");
_Static_assert(sizeof(((CT_PARAM *)0)->u16Matrix[0]) == 2 &&
                   sizeof(((CT_PARAM *)0)->u16Y_OFST) == 2,
               "COLORTRANS: the composer reads and writes both fields two bytes wide");

/*
 * R2Y is recorded and not driven, so only the two numbers a later use would
 * start from are checked: where its matrix begins and where the Y-pedestal flag
 * sits behind it.
 */
_Static_assert(I6C_ISP_IQ_R2Y_MATRIX ==
                   I6C_ISP_IQ_R2Y_MANUAL + offsetof(MI_ISP_IQ_R2YParam_t, u16Matrix),
               "R2Y: the matrix is not at offsetof(MI_ISP_IQ_R2YParam_t, u16Matrix)");
_Static_assert(I6C_ISP_IQ_R2Y_ADDY16 ==
                   I6C_ISP_IQ_R2Y_MANUAL + offsetof(MI_ISP_IQ_R2YParam_t, u8AddY16),
               "R2Y: the pedestal flag is not at offsetof(MI_ISP_IQ_R2YParam_t, u8AddY16)");

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
