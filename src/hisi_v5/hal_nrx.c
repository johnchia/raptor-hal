/*
 * hisi_v5/hal_nrx.c -- the [static_3dnr] ladder, on the VI pipe.
 *
 * WHAT THE SECTION IS. One block of text per ISO rung -- ten of them in
 * the OS04D10 file, 100 to 51200 -- each naming about 250 fields of the
 * 3DNR parameter set in a formatted ASCII table. The vendor reads it back
 * with a single sscanf against a 957-conversion format string
 * (g_3dnr_fmt) and a 957-pointer argument list (SCENE_3DNR_ARG_LIST) in
 * scene_loadparam.c. That is not a format this reader can hand to scanf:
 * the file's own whitespace and separators drift from the format string,
 * and a single mismatched column would silently shift every field after
 * it. So the text is read as a token stream instead -- see nrx_block.
 *
 * WHERE IT GOES. Not an ISP module. On V5 3DNR is a VI *pipe* parameter,
 * written through ss_mpi_vi_set_pipe_3dnr_param, which is why this is a
 * file of its own rather than another section in hal_isp.c. v5_nr.h has
 * the structure and the provenance of every offset in it.
 *
 * MANUAL, NOT AUTO. ot_nr_norm_param_v2 has an AUTO form -- a count, an
 * ISO array and a parameter array, with the driver picking per frame --
 * which on the face of it is the better place for a ladder. The vendor
 * never uses it: scene_set_3dnr sets op_mode to MANUAL on both the get
 * and the set, and does the ladder in userspace. Two userspace pointers
 * handed to a pipe parameter and expected to stay live is not a contract
 * worth guessing at, so this follows the vendor exactly.
 *
 * THE BLEND IS THE VENDOR'S. ot_scene_set_dynamic_3dnr picks the rung
 * with scene_get_level_ltoh_u32, takes rung 0 whole below the first
 * threshold, and otherwise interpolates *every* field between the rung
 * below and the rung at -- scene_set_nrx_attr_interpulate, applied
 * uniformly across nry/iey/sfy/sfy_lut/tfy/mdy/nrc0/nrc1 with no field
 * treated specially. That uniformity is what makes it cheap here: the
 * rungs are kept as the flat 957 numbers the file gave, the blend is one
 * loop over them, and only the last step -- laying the blended numbers
 * back into the structure -- needs to know where each field lives.
 *
 * GET-MODIFY-SET, like every other section. The file names 957 of the
 * 1298 bytes; the rest have to survive. Each write fetches the pipe's
 * current parameter set, lays the blended rung over the fields the file
 * named, and writes it back.
 *
 * WHAT DRIVES IT. hal_dyn.c's AE tick, once a second, shared. A write
 * costs two ioctls, so the trigger is hisi_iso_map -- the same six-steps-
 * per-stop "has the light actually moved" test the DRC engine uses --
 * rather than every tick. Between two rungs a stop apart that is six
 * blend steps, which is finer than the ladder itself.
 *
 * THE ENABLE MASK does not gate this. [module_state] carries two flags,
 * bStatic3DNR and bDyanamic3DNR, and every vendor file -- the SDK's six
 * reference sensors and the OS04D10 dump alike -- sets the first to 0 and
 * the second to 1. The vendor's own loader reads the section regardless
 * (scene_load_static_3dnr takes module_state and never looks at it) and
 * gates only the runtime on bDyanamic3DNR. A flag that is 0 in every file
 * in existence gates nothing, so this follows hal_dyn.c's rule for the
 * dynamic sections: a section present is a section meant.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"
#include "v5_nr.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

/* HI_SCENE_3DNR_MAX_COUNT. */
#define NRX_MAX 16
/* 937 %3d plus 20 %4d in g_3dnr_fmt. */
#define NRX_FIELDS 957
/* The widest row is -nc1sfs2_sat. */
#define NRX_ROW_MAX 20

/* ================================================================
 * THE ROW TABLE
 * ================================================================
 *
 * One entry per tag in the vendor's format string, in its order. `count`
 * is how many numbers the row carries, which is what the tokenizer needs
 * and what fixes each row's slice of the flat 957.
 *
 * A row whose fields are one contiguous array of one type needs only its
 * offset -- that is 45 of the 75, and 765 of the 957 numbers. The other
 * 30 rows scatter their fields across the tree (three filter stages and
 * the sharpener, interleaved) and are written by nrx_put_scalar below,
 * which is transcribed from SCENE_3DNR_ARG_LIST field for field.
 */

enum {
    NRX_SCALAR = 0,
    NRX_U8,
    NRX_U16,
};

/* The scalar rows, in table order; nrx_put_scalar switches on these. */
enum {
    R_EN = 0,
    R_SF1,
    R_SF2,
    R_SF3,
    R_SF4,
    R_SFK4,
    R_SF5,
    R_SF6,
    R_SF7,
    R_SF8,
    R_SHT,
    R_SFN,
    R_STH,
    R_2SFN,
    R_2STH,
    R_REF,
    R_TFS_MODE,
    R_TSS,
    R_TFS,
    R_TFR0,
    R_TFR1,
    R_MATH,
    R_MATE,
    R_MABW,
    R_PRETFS,
    R_PREMATH,
    R_PREMATHD,
    R_PREMABW,
    R_PRETDZ,
    R_GAMMA_EN,
    R_SCALARS, /* every row from here on is a plain array */
};

struct nrx_row {
    const char *tag;
    unsigned short count;
    unsigned char kind;
    unsigned short off; /* array rows only; offsetof into v5_nr_v2 */
};

#define NRX_OFF(expr) (unsigned short)offsetof(v5_nr_v2, expr)

static const struct nrx_row nrx_rows[] = {
    /* --- the scattered rows, in R_* order --- */
    {"-en", 4, NRX_SCALAR, 0},
    {"-nXsf1", 8, NRX_SCALAR, 0},
    {"-nXsf2", 11, NRX_SCALAR, 0},
    {"-nXsf3", 6, NRX_SCALAR, 0},
    {"-nXsf4", 11, NRX_SCALAR, 0},
    {"-nXsfk4", 4, NRX_SCALAR, 0},
    {"-nXsf5", 1, NRX_SCALAR, 0},
    {"-nXsf6", 7, NRX_SCALAR, 0},
    {"-nXsf7", 6, NRX_SCALAR, 0},
    {"-nXsf8", 6, NRX_SCALAR, 0},
    {"-nXsht", 4, NRX_SCALAR, 0},
    {"-nXsfn", 16, NRX_SCALAR, 0},
    {"-nXsth", 12, NRX_SCALAR, 0},
    {"-nX2sfn", 16, NRX_SCALAR, 0},
    {"-nX2sth", 12, NRX_SCALAR, 0},
    {"-ref", 1, NRX_SCALAR, 0},
    {"-tfs_mode", 1, NRX_SCALAR, 0},
    {"-nXtss", 7, NRX_SCALAR, 0},
    {"-nXtfs", 7, NRX_SCALAR, 0},
    {"-nXtfr0", 14, NRX_SCALAR, 0},
    {"-nXtfr1", 13, NRX_SCALAR, 0},
    {"-mXmath", 4, NRX_SCALAR, 0},
    {"-mXmate", 3, NRX_SCALAR, 0},
    {"-mXmabw", 3, NRX_SCALAR, 0},
    {"-pretfs", 2, NRX_SCALAR, 0},
    {"-premath", 2, NRX_SCALAR, 0},
    {"-premathd", 2, NRX_SCALAR, 0},
    {"-premabw", 2, NRX_SCALAR, 0},
    {"-pretdz", 5, NRX_SCALAR, 0},
    {"-gamma_en", 2, NRX_SCALAR, 0},

    /* --- the contiguous rows --- */
    {"-n2sf_var_f", 17, NRX_U8, NRX_OFF(luty[0].sf_var_f)},
    {"-n2sf_var_b", 17, NRX_U8, NRX_OFF(luty[0].sf_var_b)},
    {"-n2sf_bri_f", 17, NRX_U8, NRX_OFF(luty[0].sf_bri_f)},
    {"-n2sf_bri_b", 17, NRX_U8, NRX_OFF(luty[0].sf_bri_b)},
    {"-n2sf_dir_f", 17, NRX_U8, NRX_OFF(luty[0].sf_dir_f)},
    {"-n2sf_dir_b", 17, NRX_U8, NRX_OFF(luty[0].sf_dir_b)},
    {"-n2sf_cor", 17, NRX_U8, NRX_OFF(luty[0].sf_cor)},
    {"-n2sf_mot", 17, NRX_U8, NRX_OFF(luty[0].sf_mot)},
    {"-n2sf5_var_f", 17, NRX_U8, NRX_OFF(luty[0].sf5_var_f)},
    {"-n2sf5_var_b", 17, NRX_U8, NRX_OFF(luty[0].sf5_var_b)},
    {"-n2sf5_bri_f", 17, NRX_U8, NRX_OFF(luty[0].sf5_bri_f)},
    {"-n2sf5_bri_b", 17, NRX_U8, NRX_OFF(luty[0].sf5_bri_b)},
    {"-n2sf5_dir_f", 17, NRX_U8, NRX_OFF(luty[0].sf5_dir_f)},
    {"-n2sf5_dir_b", 17, NRX_U8, NRX_OFF(luty[0].sf5_dir_b)},
    {"-n2sf5_sad_f", 17, NRX_U8, NRX_OFF(luty[0].sf5_sad_f)},
    {"-n2sf5_cor", 17, NRX_U8, NRX_OFF(luty[0].sf5_cor)},
    {"-n3sf_var_f", 17, NRX_U8, NRX_OFF(luty[1].sf_var_f)},
    {"-n3sf_var_b", 17, NRX_U8, NRX_OFF(luty[1].sf_var_b)},
    {"-n3sf_bri_f", 17, NRX_U8, NRX_OFF(luty[1].sf_bri_f)},
    {"-n3sf_bri_b", 17, NRX_U8, NRX_OFF(luty[1].sf_bri_b)},
    {"-n3sf_dir_f", 17, NRX_U8, NRX_OFF(luty[1].sf_dir_f)},
    {"-n3sf_dir_b", 17, NRX_U8, NRX_OFF(luty[1].sf_dir_b)},
    {"-n3sf_cor", 17, NRX_U8, NRX_OFF(luty[1].sf_cor)},
    {"-n3sf_mot", 17, NRX_U8, NRX_OFF(luty[1].sf_mot)},
    {"-n2var_by_bri", 17, NRX_U8, NRX_OFF(luty[0].var_by_bri)},
    {"-n2sf_bld", 17, NRX_U8, NRX_OFF(luty[0].sf_bld)},
    {"-n3var_by_bri", 17, NRX_U8, NRX_OFF(luty[1].var_by_bri)},
    {"-n3sf_bld", 17, NRX_U8, NRX_OFF(luty[1].sf_bld)},
    {"-n4shp_crtl_mean", 17, NRX_U16, NRX_OFF(iey.shp_ctl_mean)},
    {"-n4shp_crtl_dir", 17, NRX_U16, NRX_OFF(iey.shp_ctl_dir)},
    {"-n4sf_bld", 17, NRX_U16, NRX_OFF(iey.sf_bld)},
    {"-nc0tfs_mot", 17, NRX_U8, NRX_OFF(nrc0.tfs_mot)},
    {"-nc1sfs_mot", 17, NRX_U8, NRX_OFF(nrc1.sfs1_mot)},
    {"-gamma_lut_0", 16, NRX_U16, NRX_OFF(pp.gamma_lut[0])},
    {"-gamma_lut_1", 17, NRX_U16, NRX_OFF(pp.gamma_lut[16])},
    {"-ca_lut_0", 16, NRX_U8, NRX_OFF(pp.ca_lut[0])},
    {"-ca_lut_1", 17, NRX_U8, NRX_OFF(pp.ca_lut[16])},
    {"-n1tfs0_mot", 17, NRX_U8, NRX_OFF(tfy[0].tfs0_mot)},
    {"-n2tfs0_mot", 17, NRX_U8, NRX_OFF(tfy[1].tfs0_mot)},
    {"-n1tfs1_mot", 17, NRX_U8, NRX_OFF(tfy[0].tfs1_mot)},
    {"-n2tfs1_mot", 17, NRX_U8, NRX_OFF(tfy[1].tfs1_mot)},
    {"-nc1sfs2_mot", 16, NRX_U8, NRX_OFF(nrc1.sfs2_mot)},
    {"-nc1sfs2_sat", 20, NRX_U16, NRX_OFF(nrc1.sfs2_sat)},
    {"-n0tfs_mot", 17, NRX_U8, NRX_OFF(mdy0.tfs_mot)},
    {"-n2sf5_mot", 17, NRX_U8, NRX_OFF(luty[0].sf5_mot)},
};

#define NRX_ROWS ((int)(sizeof(nrx_rows) / sizeof(nrx_rows[0])))

/* ================================================================
 * LAYING A BLENDED ROW BACK INTO ot_nr_v2
 * ================================================================ */

static unsigned char nrx_c8(unsigned v)
{
    return (unsigned char)(v > 255 ? 255 : v);
}

static unsigned char nrx_c4(unsigned v)
{
    return (unsigned char)(v > 15 ? 15 : v);
}

/*
 * The 30 rows whose fields are scattered. Transcribed from
 * SCENE_3DNR_ARG_LIST; each case is that row's argument sublist in order,
 * and the ps[]/pi/pt/pm/pm0/pc0/pc1 names there are sfy[]/iey/tfy[]/
 * mdy[]/mdy0/nrc0/nrc1 here.
 */
static void nrx_put_scalar(v5_nr_v2 *p, int row, const unsigned short *f)
{
    int i;

    switch (row) {
    case R_EN:
        p->nry1_en = f[0] ? 1 : 0;
        p->nry2_en = f[1] ? 1 : 0;
        p->nry3_en = f[2] ? 1 : 0;
        p->nry4_en = f[3] ? 1 : 0;
        break;
    case R_SF1:
        for (i = 0; i < 3; i++) {
            p->sfy[i].sfs1 = nrx_c8(f[2 * i]);
            p->sfy[i].sbr1 = nrx_c8(f[2 * i + 1]);
        }
        p->iey.coarse_g1 = f[6];
        p->iey.fine_g1 = f[7];
        break;
    case R_SF2:
        for (i = 0; i < 3; i++) {
            p->sfy[i].sfs2 = nrx_c8(f[3 * i]);
            p->sfy[i].sft2 = nrx_c8(f[3 * i + 1]);
            p->sfy[i].sbr2 = nrx_c8(f[3 * i + 2]);
        }
        p->iey.coarse_g2 = f[9];
        p->iey.fine_g2 = f[10];
        break;
    case R_SF3:
        p->sfy[1].strf3 = nrx_c8(f[0]);
        p->sfy[1].strb3 = nrx_c8(f[1]);
        p->sfy[2].strf3 = nrx_c8(f[2]);
        p->sfy[2].strb3 = nrx_c8(f[3]);
        p->iey.coarse_g3 = f[4];
        p->iey.fine_g3 = f[5];
        break;
    case R_SF4:
        for (i = 0; i < 3; i++) {
            p->sfy[i].sfs4 = nrx_c8(f[3 * i]);
            p->sfy[i].sft4 = nrx_c8(f[3 * i + 1]);
            p->sfy[i].sbr4 = nrx_c8(f[3 * i + 2]);
        }
        /* f[9] and f[10] are coarse_g4 and fine_g4, which this driver's
         * pshrp does not have; see v5_nr.h. */
        break;
    case R_SFK4:
        p->sfy[1].strf4 = nrx_c8(f[0]);
        p->sfy[1].strb4 = nrx_c8(f[1]);
        p->sfy[2].strf4 = nrx_c8(f[2]);
        p->sfy[2].strb4 = nrx_c8(f[3]);
        break;
    case R_SF5:
        p->sfy[1].sfs5 = nrx_c8(f[0]);
        break;
    case R_SF6:
        p->sfy[0].sfn6_0 = nrx_c8(f[0]);
        p->sfy[0].sfn6_1 = nrx_c8(f[1]);
        p->sfy[0].bld6 = f[2];
        p->sfy[1].sfn6_0 = nrx_c8(f[3]);
        p->sfy[1].sfn6_1 = nrx_c8(f[4]);
        p->sfy[2].sfn6_0 = nrx_c8(f[5]);
        p->sfy[2].sfn6_1 = nrx_c8(f[6]);
        break;
    case R_SF7:
        p->sfy[1].sfn7_0 = nrx_c8(f[0]);
        p->sfy[1].sfn7_1 = nrx_c8(f[1]);
        p->sfy[2].sfn7_0 = nrx_c8(f[2]);
        p->sfy[2].sfn7_1 = nrx_c8(f[3]);
        p->iey.sfn7_0 = f[4];
        p->iey.sfn7_1 = f[5];
        break;
    case R_SF8:
        p->sfy[1].sfn8_0 = nrx_c8(f[0]);
        p->sfy[1].sfn8_1 = nrx_c8(f[1]);
        p->sfy[2].sfn8_0 = nrx_c8(f[2]);
        p->sfy[2].sfn8_1 = nrx_c8(f[3]);
        p->iey.sfn8_0 = f[4];
        p->iey.sfn8_1 = f[5];
        break;
    case R_SHT:
        p->iey.o_sht_f = nrx_c8(f[0]);
        p->iey.u_sht_f = nrx_c8(f[1]);
        p->iey.o_sht_b = nrx_c8(f[2]);
        p->iey.u_sht_b = nrx_c8(f[3]);
        break;
    case R_SFN:
        for (i = 0; i < 3; i++) {
            p->sfy[i].sfn0_0 = f[4 * i];
            p->sfy[i].sfn1_0 = f[4 * i + 1];
            p->sfy[i].sfn2_0 = f[4 * i + 2];
            p->sfy[i].sfn3_0 = f[4 * i + 3];
        }
        p->iey.sfn0_0 = nrx_c4(f[12]);
        p->iey.sfn1_0 = nrx_c4(f[13]);
        p->iey.sfn2_0 = nrx_c4(f[14]);
        p->iey.sfn3_0 = nrx_c4(f[15]);
        break;
    case R_STH:
        for (i = 0; i < 3; i++) {
            p->sfy[i].sth1_0 = f[3 * i];
            p->sfy[i].sth2_0 = f[3 * i + 1];
            p->sfy[i].sth3_0 = f[3 * i + 2];
        }
        p->iey.sth1_0 = f[9];
        p->iey.sth2_0 = f[10];
        p->iey.sth3_0 = f[11];
        break;
    case R_2SFN:
        for (i = 0; i < 3; i++) {
            p->sfy[i].sfn0_1 = f[4 * i];
            p->sfy[i].sfn1_1 = f[4 * i + 1];
            p->sfy[i].sfn2_1 = f[4 * i + 2];
            p->sfy[i].sfn3_1 = f[4 * i + 3];
        }
        p->iey.sfn0_1 = nrx_c4(f[12]);
        p->iey.sfn1_1 = nrx_c4(f[13]);
        p->iey.sfn2_1 = nrx_c4(f[14]);
        p->iey.sfn3_1 = nrx_c4(f[15]);
        break;
    case R_2STH:
        for (i = 0; i < 3; i++) {
            p->sfy[i].sth1_1 = f[3 * i];
            p->sfy[i].sth2_1 = f[3 * i + 1];
            p->sfy[i].sth3_1 = f[3 * i + 2];
        }
        p->iey.sth1_1 = f[9];
        p->iey.sth2_1 = f[10];
        p->iey.sth3_1 = f[11];
        break;
    case R_REF:
        p->tfy[1].ref_en = f[0] ? 1 : 0;
        break;
    case R_TFS_MODE:
        p->tfy[0].tfs_mode = nrx_c8(f[0]);
        break;
    case R_TSS:
        p->tfy[0].tss0 = nrx_c8(f[0]);
        p->tfy[0].tss1 = nrx_c8(f[1]);
        p->tfy[0].tss2 = nrx_c8(f[2]);
        p->tfy[1].tss0 = nrx_c8(f[3]);
        p->tfy[1].tss1 = nrx_c8(f[4]);
        p->tfy[1].tss2 = nrx_c8(f[5]);
        p->nrc0_mode = f[6] ? 1 : 0;
        break;
    case R_TFS:
        p->tfy[0].tfs0 = nrx_c4(f[0]);
        p->tfy[0].tfs1 = nrx_c4(f[1]);
        p->tfy[0].tfs2 = nrx_c4(f[2]);
        p->tfy[1].tfs0 = nrx_c4(f[3]);
        p->tfy[1].tfs1 = nrx_c4(f[4]);
        p->tfy[1].tfs2 = nrx_c4(f[5]);
        p->nrc0.tfs = nrx_c8(f[6]);
        break;
    case R_TFR0:
        for (i = 0; i < 3; i++) {
            p->tfy[0].tfr0[i] = nrx_c8(f[i]);
            p->tfy[1].tfr0[i] = nrx_c8(f[3 + i]);
            p->tfy[0].tfr0[3 + i] = nrx_c8(f[7 + i]);
            p->tfy[1].tfr0[3 + i] = nrx_c8(f[10 + i]);
        }
        p->nrc0.sfc = nrx_c8(f[6]);
        p->nrc0.tfc = nrx_c8(f[13]);
        break;
    case R_TFR1:
        for (i = 0; i < 3; i++) {
            p->tfy[0].tfr1[i] = nrx_c8(f[i]);
            p->tfy[1].tfr1[i] = nrx_c8(f[3 + i]);
            p->tfy[0].tfr1[3 + i] = nrx_c8(f[7 + i]);
            p->tfy[1].tfr1[3 + i] = nrx_c8(f[10 + i]);
        }
        p->nrc0.trc = nrx_c8(f[6]);
        break;
    case R_MATH:
        p->mdy[0].math0 = f[0];
        p->mdy[0].math1 = f[1];
        p->mdy[1].math0 = f[2];
        p->mdy[1].math1 = f[3];
        break;
    case R_MATE:
        /* mdy[0].mate0 is not in the vendor's argument list; it keeps
         * whatever the pipe already had. */
        p->mdy[0].mate1 = f[0];
        p->mdy[1].mate0 = f[1];
        p->mdy[1].mate1 = f[2];
        break;
    case R_MABW:
        p->mdy[0].mabw1 = nrx_c4(f[0]);
        p->mdy[1].mabw0 = nrx_c4(f[1]);
        p->mdy[1].mabw1 = nrx_c4(f[2]);
        break;
    case R_PRETFS:
        p->mdy0.tfs = f[0];
        p->nrc_en = f[1] ? 1 : 0;
        break;
    case R_PREMATH:
        p->mdy0.math = nrx_c8(f[0]);
        p->nrc1.pre_sfs = nrx_c8(f[1]);
        break;
    case R_PREMATHD:
        p->mdy0.mathd = nrx_c8(f[0]);
        p->nrc1.sfs1 = nrx_c8(f[1]);
        break;
    case R_PREMABW:
        p->mdy0.mabw = nrx_c8(f[0]);
        p->nrc1.sfs2_mode = nrx_c8(f[1]);
        break;
    case R_PRETDZ:
        p->mdy0.tdz = nrx_c8(f[0]);
        p->nrc1.sfs2_coarse_f = nrx_c8(f[1]);
        p->nrc1.sfs2_coarse = nrx_c8(f[2]);
        p->nrc1.sfs2_fine_f = nrx_c8(f[3]);
        p->nrc1.sfs2_fine_b = nrx_c8(f[4]);
        break;
    case R_GAMMA_EN:
        p->pp.gamma_en = nrx_c8(f[0]);
        p->pp.ca_en = nrx_c8(f[1]);
        break;
    default:
        break;
    }
}

/* ================================================================
 * THE SET
 * ================================================================ */

struct hisi_nrx_set {
    bool seen;
    int cnt;   /* threed_nr_count, 0 = not given */
    int iso_n; /* thresholds given */
    unsigned iso[NRX_MAX];
    /* One rung per block, each the flat 957 the file gave, in row-table
     * order. 30 KB for the full sixteen, allocated on the first block. */
    unsigned short (*v)[NRX_FIELDS];
    unsigned char full[NRX_MAX]; /* every row of this rung parsed */
    int blocks;                  /* highest index seen, plus one */
    unsigned short first[NRX_ROWS + 1];

    int n; /* validated rungs; 0 = section off */
    int failures;
    char engine;
    const char *err;
    unsigned last_map;
    unsigned last_iso;
    int last_lvl;
    v5_3dnr_param *work;
    bool warned;
};

static struct hisi_nrx_set *nrx_set(hisi_state_t *st)
{
    if (!st->nrx) {
        st->nrx = calloc(1, sizeof(*st->nrx));
        if (st->nrx) {
            int r, at = 0;

            st->nrx->last_lvl = -1;
            for (r = 0; r < NRX_ROWS; r++) {
                st->nrx->first[r] = (unsigned short)at;
                at += nrx_rows[r].count;
            }
            st->nrx->first[NRX_ROWS] = (unsigned short)at;
            /* The one invariant a compiler cannot check here: the row
             * table's counts have to add up to the format string's
             * conversions, because that sum is what a rung is sized by.
             * An edit that breaks it would overrun the rung, so the
             * section goes rather than the memory. */
            if (at != NRX_FIELDS) {
                HAL_LOG_ERR("isp tuning: [static_3dnr] row table sums to %d, not %d; section "
                            "disabled",
                            at, NRX_FIELDS);
                free(st->nrx);
                st->nrx = NULL;
            }
        }
    }
    return st->nrx;
}

void hisi_nrx_free(hisi_state_t *st)
{
    if (st->nrx) {
        free(st->nrx->v);
        free(st->nrx->work);
    }
    free(st->nrx);
    st->nrx = NULL;
}

bool hisi_nrx_armed(hisi_state_t *st)
{
    return st->nrx && __atomic_load_n(&st->nrx->engine, __ATOMIC_ACQUIRE);
}

/* ================================================================
 * THE TOKEN STREAM
 * ================================================================ */

static bool nrx_wordchar(char c)
{
    return isalnum((unsigned char)c) || c == '_' || c == '-';
}

static int nrx_find(const char *w)
{
    int r;

    for (r = 0; r < NRX_ROWS; r++)
        if (!strcmp(nrx_rows[r].tag, w))
            return r;
    return -1;
}

/*
 * nrx_block -- one 3DnrParam_N into one rung's flat 957.
 *
 * The rules, all of them the file's own shape rather than anything
 * invented here:
 *
 *   - Words are runs of [A-Za-z0-9_-]; every other character separates.
 *     That absorbs the '|', ':' and '*' the table draws its columns with,
 *     and the "**********nr_c1" banner lines, without a grammar for each.
 *   - A word beginning '-' followed by a letter is a tag. One that names
 *     a row opens it; one that does not is an annotation the vendor prints
 *     inside a row -- -nC0mode, -sfc, -tfc, -trc, -pre_sfs, -sfs2_c and
 *     the rest -- whose number is the next field of the row already open.
 *     So an unknown tag is skipped and the row keeps filling, which is
 *     exactly what SCENE_3DNR_ARG_LIST says those columns are.
 *   - Numbers fill the open row until its count is reached, then the row
 *     closes and further numbers are ignored until the next tag.
 *
 * A short row -- a known tag arriving before the open one is full -- is
 * malformed text, and the block is dropped whole rather than half-applied.
 */
static bool nrx_block(struct hisi_nrx_set *x, int idx, const char *s, const char *key)
{
    unsigned short *dst = x->v[idx];
    unsigned char seen[NRX_ROWS];
    char w[64];
    int row = -1, want = 0, got = 0, base = 0, r;

    memset(seen, 0, sizeof(seen));
    while (*s) {
        size_t n = 0;

        if (!nrx_wordchar(*s)) {
            s++;
            continue;
        }
        while (nrx_wordchar(*s) && n < sizeof(w) - 1)
            w[n++] = *s++;
        while (nrx_wordchar(*s))
            s++;
        w[n] = '\0';

        if (w[0] == '-' && isalpha((unsigned char)w[1])) {
            r = nrx_find(w);
            /* An annotation -- -nC0mode inside -nXtss, -sfc inside -nXtfr0
             * and the rest. Not a row: the number after it is the next
             * field of the row still open, which is what the argument
             * list says those columns are. So it is stepped over without
             * closing anything. */
            if (r < 0)
                continue;
            if (row >= 0) {
                HAL_LOG_WARN("isp tuning: [static_3dnr] %s: %s has %d of %d values; block "
                             "ignored",
                             key, nrx_rows[row].tag, got, want);
                return false;
            }
            if (seen[r]) {
                HAL_LOG_WARN("isp tuning: [static_3dnr] %s: %s given twice; block ignored", key, w);
                return false;
            }
            seen[r] = 1;
            row = r;
            want = nrx_rows[r].count;
            base = x->first[r];
            got = 0;
            continue;
        }

        if (!isdigit((unsigned char)w[0]) && !(w[0] == '-' && isdigit((unsigned char)w[1])))
            continue; /* a banner word like nr_c0 */
        if (row < 0)
            continue; /* before the first tag, or after a row closed */
        {
            long v = strtol(w, NULL, 10);

            dst[base + got] = (unsigned short)(v < 0 ? 0 : v > 65535 ? 65535 : v);
        }
        if (++got == want)
            row = -1;
    }

    if (row >= 0) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] %s: %s has %d of %d values; block ignored", key,
                     nrx_rows[row].tag, got, want);
        return false;
    }
    for (r = 0; r < NRX_ROWS; r++)
        if (!seen[r]) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] %s: %s missing; block ignored", key,
                         nrx_rows[r].tag);
            return false;
        }
    return true;
}

/* ================================================================
 * THE KEYS
 * ================================================================ */

static int nrx_nums(const char *s, long *out, int max)
{
    int n = 0;

    while (*s && n < max) {
        while (*s && !isdigit((unsigned char)*s))
            s++;
        if (!*s)
            break;
        out[n++] = strtol(s, (char **)&s, 10);
    }
    return n;
}

/*
 * hisi_nrx_key -- one key of [static_3dnr]. False for a key with no
 * mapping, which the caller logs at debug and drops.
 */
bool hisi_nrx_key(hisi_state_t *st, const char *key, const char *val)
{
    struct hisi_nrx_set *x = nrx_set(st);
    long v[NRX_MAX];
    int i, n, idx;

    if (!x)
        return false;
    x->seen = true;

    if (!strcmp(key, "threed_nr_count")) {
        long c = 0;

        nrx_nums(val, &c, 1);
        x->cnt = (int)c;
        return true;
    }
    if (!strcmp(key, "threed_nr_iso")) {
        n = nrx_nums(val, v, NRX_MAX);
        for (i = 0; i < n; i++)
            x->iso[i] = (unsigned)(v[i] < 0 ? 0 : v[i]);
        x->iso_n = n;
        return true;
    }
    if (strncmp(key, "3DnrParam_", 10))
        return false;
    if (!isdigit((unsigned char)key[10]))
        return false;
    idx = (int)strtol(key + 10, NULL, 10);
    if (idx < 0 || idx >= NRX_MAX) {
        if (!x->warned) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] %s: past the %d the driver takes; ignored", key,
                         NRX_MAX);
            x->warned = true;
        }
        return true;
    }
    if (!x->v) {
        x->v = calloc(NRX_MAX, sizeof(*x->v));
        if (!x->v) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] out of memory; section ignored");
            return true;
        }
    }
    if (nrx_block(x, idx, val, key))
        x->full[idx] = 1;
    if (idx + 1 > x->blocks)
        x->blocks = idx + 1;
    return true;
}

/* ================================================================
 * THE WRITE
 * ================================================================ */

/* scene_get_level_ltoh_u32 over the rung thresholds. */
static int nrx_level(unsigned iso, int n, const unsigned *thr)
{
    int l;

    for (l = 0; l < n; l++)
        if (iso <= thr[l])
            return l;
    return n - 1;
}

/*
 * nrx_lay -- the blended rung, over a parameter set fetched from the
 * pipe. Below the first threshold the vendor takes rung 0 whole; above
 * it every field is interpolated between the rung below and the rung at,
 * with hisi_iso_lerp clamping at both ends so the top rung past its own
 * threshold is taken whole as well.
 */
static void nrx_lay(struct hisi_nrx_set *x, v5_nr_v2 *p, int lvl, unsigned iso)
{
    unsigned short f[NRX_ROW_MAX];
    int r, i;

    for (r = 0; r < NRX_ROWS; r++) {
        const struct nrx_row *row = &nrx_rows[r];
        const unsigned short *a = x->v[lvl > 0 ? lvl - 1 : 0] + x->first[r];

        const unsigned short *b = x->v[lvl] + x->first[r];

        for (i = 0; i < row->count; i++)
            f[i] = lvl > 0 ? (unsigned short)hisi_iso_lerp(iso, x->iso[lvl - 1], a[i], x->iso[lvl],
                                                           b[i])
                           : b[i];

        if (row->kind == NRX_SCALAR) {
            nrx_put_scalar(p, r, f);
        } else if (row->kind == NRX_U8) {
            unsigned char *d = (unsigned char *)p + row->off;

            for (i = 0; i < row->count; i++)
                d[i] = nrx_c8(f[i]);
        } else {
            unsigned short *d = (unsigned short *)(void *)((char *)p + row->off);

            for (i = 0; i < row->count; i++)
                d[i] = f[i];
        }
    }
}

/*
 * nrx_write -- get, lay, set. nr_version and op_mode are filled in before
 * the get as well as the set: the vendor does, and on this driver they
 * select which arm of the union the get is asked for.
 */
static int nrx_write(hisi_state_t *st, struct hisi_nrx_set *x, unsigned iso)
{
    v5_3dnr_param *w = x->work;
    int lvl, ret;

    if (!st->nr.fnGetPipe3dnrParam || !st->nr.fnSetPipe3dnrParam) {
        x->err = "vi_{get,set}_pipe_3dnr_param";
        return -1;
    }
    if (!w) {
        w = calloc(1, sizeof(*w));
        if (!w) {
            x->err = "vi_get_pipe_3dnr_param";
            return -1;
        }
        x->work = w;
    }

    memset(w, 0, sizeof(*w));
    w->nr_version = V5_NR_V2;
    w->nr_norm_param_v2.op_mode = V5_OP_MODE_MANUAL;
    ret = st->nr.fnGetPipe3dnrParam(HISI_VI_PIPE, w);
    if (ret) {
        x->err = "vi_get_pipe_3dnr_param";
        return ret;
    }

    lvl = nrx_level(iso, x->n, x->iso);
    nrx_lay(x, &w->nr_norm_param_v2.nr_manual, lvl, iso);

    w->nr_version = V5_NR_V2;
    w->nr_norm_param_v2.op_mode = V5_OP_MODE_MANUAL;
    ret = st->nr.fnSetPipe3dnrParam(HISI_VI_PIPE, w);
    if (ret) {
        x->err = "vi_set_pipe_3dnr_param";
        return ret;
    }
    x->last_lvl = lvl;
    return 0;
}

/*
 * hisi_nrx_on_iso -- from hal_dyn.c's AE tick. Two ioctls a write, so the
 * trigger is the same "has the light moved" test the DRC engine uses:
 * hisi_iso_map, six steps per stop, which between two rungs a stop apart
 * is six blend steps.
 */
void hisi_nrx_on_iso(hisi_state_t *st, unsigned iso)
{
    struct hisi_nrx_set *x = st->nrx;
    unsigned map;
    int ret;

    if (!x || !__atomic_load_n(&x->engine, __ATOMIC_ACQUIRE) || !iso)
        return;
    map = hisi_iso_map(iso);
    if (map == x->last_map)
        return;
    x->last_map = map;

    ret = nrx_write(st, x, iso);
    if (ret) {
        if (++x->failures >= 3) {
            HAL_LOG_WARN("3dnr: ss_mpi_%s failed three times running (last 0x%x); leaving the "
                         "last parameters in place and stopping",
                         x->err, ret);
            __atomic_store_n(&x->engine, 0, __ATOMIC_RELEASE);
        }
        return;
    }
    HAL_LOG_DBG("3dnr: ISO %u -> %u, rung %d", x->last_iso, iso, x->last_lvl);
    x->last_iso = iso;
}

/* ================================================================
 * APPLY
 * ================================================================ */

/*
 * What the file has to give before any of it is written: a count, that
 * many thresholds, that many complete blocks, and thresholds that
 * ascend -- nrx_level walks them in order and a descending pair would
 * make a rung unreachable. Returns the usable rung count.
 */
static int nrx_check(struct hisi_nrx_set *x)
{
    int i;

    if (!x->seen)
        return 0;
    if (!x->v || x->cnt <= 0) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] no threed_nr_count or no blocks; section "
                     "ignored");
        return 0;
    }
    if (x->cnt > NRX_MAX) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] threed_nr_count %d over the %d the driver "
                     "takes; section ignored",
                     x->cnt, NRX_MAX);
        return 0;
    }
    if (x->iso_n < x->cnt) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] threed_nr_iso has %d of %d thresholds; section "
                     "ignored",
                     x->iso_n, x->cnt);
        return 0;
    }
    if (x->blocks < x->cnt) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] %d of %d blocks present; section ignored",
                     x->blocks, x->cnt);
        return 0;
    }
    for (i = 0; i < x->cnt; i++)
        if (!x->full[i]) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] 3DnrParam_%d did not parse; section ignored",
                         i);
            return 0;
        }
    for (i = 1; i < x->cnt; i++)
        if (x->iso[i] <= x->iso[i - 1]) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] threed_nr_iso does not ascend at %u -> %u; "
                         "section ignored",
                         x->iso[i - 1], x->iso[i]);
            return 0;
        }
    return x->cnt;
}

/*
 * hisi_nrx_apply -- validate, write the rung for the ISO AE reports now
 * (the first rung when there is no AE to ask), and arm the tick. Returns
 * 1 when the ladder was written, and *failed counts a section that was
 * present and could not be, with `note` naming it for the load summary.
 */
int hisi_nrx_apply(hisi_state_t *st, int *failed, char *note, size_t note_len)
{
    struct hisi_nrx_set *x = st->nrx;
    unsigned iso = 0;
    bool have_ae;
    int ret;

    *failed = 0;
    note[0] = '\0';
    if (!x)
        return 0;

    x->n = nrx_check(x);
    if (!x->n) {
        if (x->seen) {
            *failed = 1;
            snprintf(note, note_len, "static_3dnr(nothing usable)");
        }
        return 0;
    }

    have_ae = hisi_iso_query(st, &iso, NULL);
    if (!have_ae)
        iso = x->iso[0];

    ret = nrx_write(st, x, iso);
    if (ret) {
        if (!st->nr.fnGetPipe3dnrParam || !st->nr.fnSetPipe3dnrParam)
            snprintf(note, note_len, "static_3dnr(no ss_mpi_vi_get/set_pipe_3dnr_param)");
        else
            snprintf(note, note_len, "static_3dnr(ss_mpi_%s 0x%x)", x->err ? x->err : "?", ret);
        HAL_LOG_WARN("isp tuning: [static_3dnr] not applied: %s; 3DNR keeps the driver's own "
                     "parameters",
                     note);
        (*failed)++;
        x->n = 0;
        return 0;
    }

    HAL_LOG_INFO("isp tuning: [static_3dnr] %d rungs, ISO %u..%u; %s ISO %u, rung %d%s", x->n,
                 x->iso[0], x->iso[x->n - 1], have_ae ? "AE at" : "no ISO query, first rung at",
                 iso, x->last_lvl, have_ae ? "; tracking ISO" : "");
    if (have_ae) {
        x->last_map = hisi_iso_map(iso);
        x->last_iso = iso;
        if (x->n > 1)
            __atomic_store_n(&x->engine, 1, __ATOMIC_RELEASE);
    }
    return 1;
}
