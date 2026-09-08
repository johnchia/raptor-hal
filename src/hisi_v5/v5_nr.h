/*
 * v5_nr.h -- HiMPP V5 3DNR: ot_3dnr_param and the ot_nr_v2 tree under it.
 *
 * WHERE IT LIVES. On gen4 3DNR was a VPSS group parameter. On V5 it moved
 * to the VI pipe -- ss_mpi_vi_set_pipe_3dnr_param -- and the vendor's
 * scene_auto sample writes it there, which is what settled the question
 * this port asked in Phase 0. ss_mpi_vpss_set_grp_3dnr_param still exists
 * and takes the same structure; nothing here uses it.
 *
 * VERSIONS. ot_3dnr_param is a version word and a union of two parameter
 * sets. V1 is the gen4-shaped one and is not transcribed: this die reports
 * V2, the scene files carry V2's field set, and a union member nothing
 * writes is a union member nothing can get wrong. The V2 arm is
 * transcribed whole, because the text in a scene file's [static_3dnr]
 * names about 250 of its fields and the rest have to survive underneath
 * them -- so every write is a get-modify-set of the whole 1298 bytes.
 *
 * AUTO AND MANUAL. ot_nr_norm_param_v2 carries both. MANUAL is one
 * parameter set the driver uses as given. AUTO is a count, an ISO array
 * and a parameter array, and the driver picks and interpolates between
 * them itself, once per frame, off the ISO it reads from the ISP -- which
 * is strictly better than doing it from a once-a-second tick in userspace.
 * hal_nrx.c tries AUTO first and falls back to MANUAL.
 *
 * THE TWO POINTERS IN THE AUTO FORM are declared
 * `td_u32 ATTRIBUTE *iso`, where ATTRIBUTE is __attribute__((aligned(8))).
 * That is HiSilicon's 32/64 compatibility trick: the pointer is four bytes
 * on this ARM and eight on the host that measured these offsets, but the
 * alignment forces the same slots either way -- param_num at 0, iso at 8,
 * nr_param at 16, the struct 24 bytes. The static assertions below hold on
 * both, which is the point.
 *
 * PROVENANCE, AND ONE MEASURED CORRECTION. Transcribed from the
 * Hi3516CV610 SDK V1.0.1.0 ot_common_video.h, the only header on hand that
 * defines these; the board runs MPP V1.0.2.0 B051, which has no public
 * header. The two differ in exactly one place, found on the board: the
 * 1.0.1.0 ot_nr_v2_pshrp opens with eight sharpen gains (coarse/fine g1
 * through g4, 16 bytes) and the driver's is 4 bytes shorter. Read back
 * from a running pipe, the parameter set has six 128s where the gains go,
 * then four bytes of 100 100 128 128 (the shoot controls), the two
 * 17-node 63-filled shp_ctl arrays at 50 and 84 rather than 54 and 88,
 * the nrc1.sfs2_sat run of 1023s at 478 rather than 482, the pp gamma
 * curve at 1166 rather than 1170; and the driver's own range checks --
 * strf3 <= 31, bld6 <= 16, sth1_0 <= 511, sfn0_0 <= 8, mdy math0 <= 999,
 * mabw in [5, 9], tfy tss0 <= 31, tfs_mode <= 1, nrc0 tfc <= 63, nrc1
 * pre_sfs <= 16 -- each fire at the offset the 142-byte pshrp predicts
 * and not at the 146-byte one. So pshrp here is the driver's: three gain
 * pairs. Which pair the vendor dropped cannot be told from the board (the
 * gains are neither validated nor distinguishable at their defaults), so
 * the names below keep the header's order and the fourth pair is simply
 * absent; the [static_3dnr] row that carries it (-nXsf4's last two
 * values) has nowhere to go and is not written. Everything else --
 * every other structure's size and internal order -- matched.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef HISI_V5_NR_H
#define HISI_V5_NR_H

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "v5_common.h"

/* ================================================================
 * ot_nr_v2's parts
 * ================================================================
 *
 * Bitfields where the vendor has bitfields. They are all inside a single
 * storage unit of their declared type and in declaration order, which is
 * what the ARM EABI and the host ABI both do, so the assertions below are
 * a real check rather than a formality.
 */

/*
 * ot_nr_v2_pshrp -- the No.4 filter and the shoot control. 142 bytes on
 * this driver, not the header's 146: three gain pairs, see the file
 * comment.
 */
typedef struct {
    unsigned short coarse_g1;
    unsigned short fine_g1;
    unsigned short coarse_g2;
    unsigned short fine_g2;
    unsigned short coarse_g3;
    unsigned short fine_g3;
    unsigned short o_sht_b : 8;
    unsigned short u_sht_b : 8;
    unsigned short o_sht_f : 8;
    unsigned short u_sht_f : 8;
    unsigned short sf_bld[17];
    unsigned short shp_ctl_mean[17];
    unsigned short shp_ctl_dir[17];
    unsigned short sfn7_0;
    unsigned short sfn7_1;
    unsigned short sfn8_0;
    unsigned short sfn8_1;
    unsigned short sth1_0;
    unsigned short sth2_0;
    unsigned short sth3_0;
    unsigned short sth1_1;
    unsigned short sth2_1;
    unsigned short sth3_1;
    unsigned short sfn0_0 : 4;
    unsigned short sfn1_0 : 4;
    unsigned short sfn2_0 : 4;
    unsigned short sfn3_0 : 4;
    unsigned short sfn0_1 : 4;
    unsigned short sfn1_1 : 4;
    unsigned short sfn2_1 : 4;
    unsigned short sfn3_1 : 4;
} v5_nr_pshrp;

_Static_assert(sizeof(v5_nr_pshrp) == 142, "ot_nr_v2_pshrp is 142 bytes on 1.0.2.0 B051");
_Static_assert(offsetof(v5_nr_pshrp, sf_bld) == 16, "ot_nr_v2_pshrp.sf_bld at +16");
_Static_assert(offsetof(v5_nr_pshrp, shp_ctl_mean) == 50, "ot_nr_v2_pshrp.shp_ctl_mean at +50");
_Static_assert(offsetof(v5_nr_pshrp, shp_ctl_dir) == 84, "ot_nr_v2_pshrp.shp_ctl_dir at +84");
_Static_assert(offsetof(v5_nr_pshrp, sfn7_0) == 118, "ot_nr_v2_pshrp.sfn7_0 at +118");
_Static_assert(offsetof(v5_nr_pshrp, sth1_0) == 126, "ot_nr_v2_pshrp.sth1_0 at +126");

/* ot_nr_v2_sfy -- one of the three spatial-filter stages. */
typedef struct {
    unsigned char sfs1;
    unsigned char sbr1;
    unsigned char sfs2;
    unsigned char sft2;
    unsigned char sbr2;
    unsigned char strf3;
    unsigned char strb3;
    unsigned char sfs4;
    unsigned char sft4;
    unsigned char sbr4;
    unsigned char strf4;
    unsigned char strb4;
    unsigned char sfs5;
    unsigned short bld6;
    unsigned short sth1_0;
    unsigned short sth2_0;
    unsigned short sth3_0;
    unsigned short sth1_1;
    unsigned short sth2_1;
    unsigned short sth3_1;
    unsigned short sfn0_0;
    unsigned short sfn1_0;
    unsigned short sfn2_0;
    unsigned short sfn3_0;
    unsigned short sfn0_1;
    unsigned short sfn1_1;
    unsigned short sfn2_1;
    unsigned short sfn3_1;
    unsigned char sfn6_0;
    unsigned char sfn6_1;
    unsigned char sfn7_0;
    unsigned char sfn7_1;
    unsigned char sfn8_0;
    unsigned char sfn8_1;
} v5_nr_sfy;

_Static_assert(sizeof(v5_nr_sfy) == 50, "ot_nr_v2_sfy is 50 bytes");
_Static_assert(offsetof(v5_nr_sfy, bld6) == 14, "ot_nr_v2_sfy.bld6 at +14");
_Static_assert(offsetof(v5_nr_sfy, sfn0_0) == 28, "ot_nr_v2_sfy.sfn0_0 at +28");
_Static_assert(offsetof(v5_nr_sfy, sfn6_0) == 44, "ot_nr_v2_sfy.sfn6_0 at +44");

/* ot_nr_v2_tfy -- one of the two temporal-filter stages. */
typedef struct {
    unsigned char tfs0 : 4;
    unsigned char tfs1 : 4;
    unsigned char tfs2 : 4;
    unsigned char ref_en : 1;
    unsigned char _rb_ : 3;
    unsigned char tss0;
    unsigned char tss1;
    unsigned char tss2;
    unsigned char tfr0[6];
    unsigned char tfr1[6];
    unsigned char tfs0_mot[17];
    unsigned char tfs1_mot[17];
    unsigned char tfs_mode;
} v5_nr_tfy;

_Static_assert(sizeof(v5_nr_tfy) == 52, "ot_nr_v2_tfy is 52 bytes");
_Static_assert(offsetof(v5_nr_tfy, tss0) == 2, "ot_nr_v2_tfy.tss0 at +2");
_Static_assert(offsetof(v5_nr_tfy, tfr0) == 5, "ot_nr_v2_tfy.tfr0 at +5");
_Static_assert(offsetof(v5_nr_tfy, tfs0_mot) == 17, "ot_nr_v2_tfy.tfs0_mot at +17");
_Static_assert(offsetof(v5_nr_tfy, tfs_mode) == 51, "ot_nr_v2_tfy.tfs_mode at +51");

/* ot_nr_v2_mdy -- motion detect, one per temporal stage. */
typedef struct {
    unsigned short math0;
    unsigned short mate0;
    unsigned short math1;
    unsigned short mate1;
    unsigned char mabw0 : 4;
    unsigned char mabw1 : 4;
} v5_nr_mdy;

_Static_assert(sizeof(v5_nr_mdy) == 10, "ot_nr_v2_mdy is 10 bytes");
_Static_assert(offsetof(v5_nr_mdy, mate1) == 6, "ot_nr_v2_mdy.mate1 at +6");

/* ot_nr_v2_mdy0 -- the pre-stage's motion detect. */
typedef struct {
    unsigned short tfs;
    unsigned short math : 8;
    unsigned short mathd : 8;
    unsigned char mabw;
    unsigned char tdz;
    unsigned char tfs_mot[17];
} v5_nr_mdy0;

_Static_assert(sizeof(v5_nr_mdy0) == 24, "ot_nr_v2_mdy0 is 24 bytes");
_Static_assert(offsetof(v5_nr_mdy0, mabw) == 4, "ot_nr_v2_mdy0.mabw at +4");
_Static_assert(offsetof(v5_nr_mdy0, tfs_mot) == 6, "ot_nr_v2_mdy0.tfs_mot at +6");

/* ot_nr_v2_sfy_lut -- the nineteen 17-node strength curves, twice. */
typedef struct {
    unsigned char sf_var_f[17];
    unsigned char sf_var_b[17];
    unsigned char sf_dir_f[17];
    unsigned char sf_dir_b[17];
    unsigned char sf_bri_f[17];
    unsigned char sf_bri_b[17];
    unsigned char sf5_var_f[17];
    unsigned char sf5_var_b[17];
    unsigned char sf5_bri_f[17];
    unsigned char sf5_bri_b[17];
    unsigned char sf5_dir_f[17];
    unsigned char sf5_dir_b[17];
    unsigned char sf5_sad_f[17];
    unsigned char sf_cor[17];
    unsigned char sf5_cor[17];
    unsigned char sf_mot[17];
    unsigned char sf5_mot[17];
    unsigned char var_by_bri[17];
    unsigned char sf_bld[17];
} v5_nr_sfy_lut;

_Static_assert(sizeof(v5_nr_sfy_lut) == 323, "ot_nr_v2_sfy_lut is 323 bytes");
_Static_assert(offsetof(v5_nr_sfy_lut, sf_bld) == 306, "ot_nr_v2_sfy_lut.sf_bld at +306");

/* ot_nr_v2_pp -- the post-processing gamma and colour-attenuation LUTs. */
typedef struct {
    unsigned short gamma_lut[33];
    unsigned char gamma_en;
    unsigned char ca_lut[33];
    unsigned char ca_en;
} v5_nr_pp;

_Static_assert(sizeof(v5_nr_pp) == 102, "ot_nr_v2_pp is 102 bytes");
_Static_assert(offsetof(v5_nr_pp, gamma_en) == 66, "ot_nr_v2_pp.gamma_en at +66");
_Static_assert(offsetof(v5_nr_pp, ca_lut) == 67, "ot_nr_v2_pp.ca_lut at +67");
_Static_assert(offsetof(v5_nr_pp, ca_en) == 100, "ot_nr_v2_pp.ca_en at +100");

/* ot_nr_v2_nrc0, ot_nr_v2_nrc1 -- the two chroma stages. */
typedef struct {
    unsigned char trc;
    unsigned char sfc;
    unsigned char tfc;
    unsigned char tfs;
    unsigned char tfs_mot[17];
} v5_nr_nrc0;

_Static_assert(sizeof(v5_nr_nrc0) == 21, "ot_nr_v2_nrc0 is 21 bytes");
_Static_assert(offsetof(v5_nr_nrc0, tfs_mot) == 4, "ot_nr_v2_nrc0.tfs_mot at +4");

typedef struct {
    unsigned char pre_sfs;
    unsigned char sfs1;
    unsigned char sfs1_mot[17];
    unsigned char sfs2_coarse;
    unsigned char sfs2_coarse_f;
    unsigned char sfs2_fine_f;
    unsigned char sfs2_fine_b;
    unsigned char sfs2_mot[16];
    unsigned short sfs2_sat[20];
    unsigned char sfs2_mode;
} v5_nr_nrc1;

_Static_assert(sizeof(v5_nr_nrc1) == 82, "ot_nr_v2_nrc1 is 82 bytes");
_Static_assert(offsetof(v5_nr_nrc1, sfs1_mot) == 2, "ot_nr_v2_nrc1.sfs1_mot at +2");
_Static_assert(offsetof(v5_nr_nrc1, sfs2_coarse) == 19, "ot_nr_v2_nrc1.sfs2_coarse at +19");
_Static_assert(offsetof(v5_nr_nrc1, sfs2_mot) == 23, "ot_nr_v2_nrc1.sfs2_mot at +23");
_Static_assert(offsetof(v5_nr_nrc1, sfs2_sat) == 40, "ot_nr_v2_nrc1.sfs2_sat at +40");
_Static_assert(offsetof(v5_nr_nrc1, sfs2_mode) == 80, "ot_nr_v2_nrc1.sfs2_mode at +80");

/* ot_nr_v2 -- one whole 3DNR parameter set. */
typedef struct {
    v5_nr_pshrp iey;
    v5_nr_sfy sfy[3];
    v5_nr_mdy mdy[2];
    v5_nr_tfy tfy[2];
    v5_nr_nrc0 nrc0;
    v5_nr_nrc1 nrc1;
    v5_nr_sfy_lut luty[2];
    v5_nr_pp pp;
    v5_nr_mdy0 mdy0;
    struct {
        unsigned char nry1_en : 1;
        unsigned char nry2_en : 1;
        unsigned char nry3_en : 1;
        unsigned char nry4_en : 1;
        unsigned char nrc0_mode : 1;
        unsigned char nrc_en : 1;
    };
    /* The driver reads back zeros here. The vendor's scene sample names
     * one more field in this structure (b_delay_mode) that the 1.0.1.0
     * header does not have; whatever the tail holds, it is left as the
     * get returned it. Sized so the whole matches the header's 1298. */
    unsigned char _tail_[5];
} v5_nr_v2;

_Static_assert(sizeof(v5_nr_v2) == 1298, "ot_nr_v2 is 1298 bytes");
_Static_assert(offsetof(v5_nr_v2, sfy) == 142, "ot_nr_v2.sfy at +142");
_Static_assert(offsetof(v5_nr_v2, mdy) == 292, "ot_nr_v2.mdy at +292");
_Static_assert(offsetof(v5_nr_v2, tfy) == 312, "ot_nr_v2.tfy at +312");
_Static_assert(offsetof(v5_nr_v2, nrc0) == 416, "ot_nr_v2.nrc0 at +416");
_Static_assert(offsetof(v5_nr_v2, nrc1) == 438, "ot_nr_v2.nrc1 at +438");
_Static_assert(offsetof(v5_nr_v2, luty) == 520, "ot_nr_v2.luty at +520");
_Static_assert(offsetof(v5_nr_v2, pp) == 1166, "ot_nr_v2.pp at +1166");
_Static_assert(offsetof(v5_nr_v2, mdy0) == 1268, "ot_nr_v2.mdy0 at +1268");
_Static_assert(offsetof(v5_nr_v2, _tail_) == 1293, "ot_nr_v2's enables at +1292");

/* ================================================================
 * ot_3dnr_param
 * ================================================================ */

/* ot_nr_version. Note it starts at 1, not 0 -- OT_NR_BUTT is 3. Passing 0
 * here selects nothing and the driver answers ILLEGAL_PARAM. */
#define V5_NR_V1 1
#define V5_NR_V2 2

/*
 * ot_nr_param_auto_v2. The alignment attribute, not the pointer width,
 * decides the layout -- see the file comment.
 */
typedef struct {
    unsigned int param_num;
    unsigned int *iso __attribute__((aligned(8)));
    v5_nr_v2 *nr_param __attribute__((aligned(8)));
} v5_nr_auto_v2;

_Static_assert(sizeof(v5_nr_auto_v2) == 24, "ot_nr_param_auto_v2 is 24 bytes");
_Static_assert(offsetof(v5_nr_auto_v2, iso) == 8, "ot_nr_param_auto_v2.iso at +8");
_Static_assert(offsetof(v5_nr_auto_v2, nr_param) == 16, "ot_nr_param_auto_v2.nr_param at +16");

typedef struct {
    int op_mode; /* ot_op_mode: 0 auto, 1 manual */
    v5_nr_v2 nr_manual;
    v5_nr_auto_v2 nr_auto;
} v5_nr_norm_v2;

_Static_assert(sizeof(v5_nr_norm_v2) == 1328, "ot_nr_norm_param_v2 is 1328 bytes");
_Static_assert(offsetof(v5_nr_norm_v2, nr_manual) == 4, "ot_nr_norm_param_v2.nr_manual at +4");
_Static_assert(offsetof(v5_nr_norm_v2, nr_auto) == 1304, "ot_nr_norm_param_v2.nr_auto at +1304");

/*
 * ot_3dnr_param. The V1 arm of the union is not transcribed; it is the
 * larger of the two by no margin that matters and the whole structure is
 * sized by the assertion below, so a V1 the backend never writes cannot
 * make this the wrong size.
 */
typedef struct {
    int nr_version; /* ot_nr_version */
    union {
        v5_nr_norm_v2 nr_norm_param_v2;
        unsigned char _v1_[1328];
    };
} v5_3dnr_param;

_Static_assert(sizeof(v5_3dnr_param) == 1336, "ot_3dnr_param is 1336 bytes");
_Static_assert(offsetof(v5_3dnr_param, nr_norm_param_v2) == 8, "ot_3dnr_param's union at +8");

/* ================================================================
 * ENTRY POINTS
 *
 * Both optional: a driver without them costs the 3DNR ladder and nothing
 * else, the same contract v5_isp_tune.h states.
 * ================================================================ */

typedef struct {
    int (*fnGetPipe3dnrParam)(int vi_pipe, v5_3dnr_param *param);
    int (*fnSetPipe3dnrParam)(int vi_pipe, const v5_3dnr_param *param);
} v5_nr_impl;

static inline void v5_nr_load(v5_nr_impl *lib, const v5_mpi_libs *libs)
{
    memset(lib, 0, sizeof(*lib));
    lib->fnGetPipe3dnrParam =
        (int (*)(int, v5_3dnr_param *))v5_symbol_opt(libs, "ss_mpi_vi_get_pipe_3dnr_param");
    lib->fnSetPipe3dnrParam =
        (int (*)(int, const v5_3dnr_param *))v5_symbol_opt(libs, "ss_mpi_vi_set_pipe_3dnr_param");
}

static inline void v5_nr_unload(v5_nr_impl *lib)
{
    memset(lib, 0, sizeof(*lib));
}

#endif /* HISI_V5_NR_H */
