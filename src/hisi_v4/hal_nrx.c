/*
 * hal_nrx.c -- the [static_3dnr] section: VPSS 3DNR X-params, per ISO.
 *
 * WHAT THIS IS. The shipped /etc/sensors/iq/<sensor>.ini carries, under
 * [static_3dnr], a ladder of 3DNR parameter blocks -- one per ISO step,
 * nine of them for the IMX335, from ISO 100 to 12800 -- in the HiSilicon
 * "X-param" text format: about 45 lines per block, each a tagged row of
 * numbers in columns split by '|'. It is the same text PQTools prints and
 * the SDK's scene_auto sample reads, and it configures VPSS_NRX_V3_S
 * through HI_MPI_VPSS_SetGrpNRXParam -- the VPSS group, not an ISP module,
 * which is why it does not fit hal_isp.c's Get/Set-per-module loop and
 * lives here instead. The loop hands the section's keys over (hisi_nrx_key)
 * and calls hisi_nrx_apply once the file is read.
 *
 * WHY IT MATTERS. VPSS 3DNR is on from bring-up (bNrEn in the group
 * attribute), but on the driver's default strength at every ISO. Majestic
 * steps these blocks up with gain; without them raptor leaves more temporal
 * noise in a dark scene for the encoder to spend its budget on.
 *
 * THE FORMAT, AND WHY THIS IS A TOKENIZER AND NOT AN sscanf. The SDK
 * sample parses a block with one 45-line sscanf format whose literal
 * spacing is the whole grammar. The shipped files carry two embedded tags
 * the sample's format does not know (-mode, -presfc -- fields the driver
 * has and the SDK header does not name in its sscanf), and an sscanf stops
 * dead at the first mismatch, silently leaving everything after it zero.
 * So this reads the block as a token stream instead: a tag opens a row,
 * numbers fill the row's slots in order, '|' and the "(n)" annotations are
 * separators, and an *embedded* tag (-kmode, -mXmath, -sfc ...) takes its
 * fixed count of numbers and hands the row back. Four tags (-mXmath,
 * -mXmathd, -mXmate, -mXmabw) appear twice per block with different
 * meanings -- embedded first, as a row later -- so a spec is keyed on the
 * tag and its occurrence. Which slot each number lands in is the sample's
 * mapping (scene_loadparam.c, SCENE_LoadStatic3DNR), copied field for
 * field. A block is applied only when every non-ignored spec is completely
 * filled; a short or misspelt block is skipped with the tag named.
 *
 * GET-MODIFY-SET, per block. The text carries about 250 of the structure's
 * fields; the rest -- the 64-entry SBSk/SDSk brightness tables, NRyEn,
 * IEEn, MADZ, DZMode -- are the driver's own. Every block therefore starts
 * as a copy of what GetGrpNRXParam returns and has the text laid over it.
 *
 * AUTO IF THE DRIVER TAKES IT, MANUAL IF IT WILL NOT. The V3 parameter has
 * an AUTO form -- N blocks plus N ISO thresholds, and the driver selects
 * between them itself, once per frame, off the ISO it reads from the ISP.
 * That is strictly better than doing it here: no once-a-second tick, no
 * dependency on HI_MPI_ISP_QueryExposureInfo, and the selection happens on
 * the frame rather than up to a second late. So AUTO is tried first.
 *
 * It used to fail with 0xa0078003 (ILLEGAL_PARAM), which read as "this
 * driver does not do AUTO". It was not: stNRXAuto sits at +936 in the
 * driver's VPSS_NRX_PARAM_V3_S and raptor was writing it at +928, so the
 * driver read the pastNRXParam pointer as u32ParamNum and rejected the
 * count. v4_vpss.h now carries the driver's layout, with the disassembly
 * it came from. The fall-back stays, because a driver that really does
 * refuse the form is a thing this backend should survive, and because the
 * ladder can fail the driver's own AUTO-only rules (ISO in 100..3276800,
 * strictly ascending, at most 16 rungs).
 *
 * The fall-back is the SDK's scene_auto sample: read the ISO from AE
 * (HI_MPI_ISP_QueryExposureInfo), interpolate between the two rungs either
 * side of it, and write MANUAL. It runs from hal_dyn.c's once-a-second ISO
 * tick off the encoder's frame hook (hisi_nrx_on_iso), and only when the
 * ISO has moved a step (the sample's MapISO, about six per stop --
 * hisi_iso_map, which lives there with the query and the blend the dynamic
 * ISP sections share). Without the query symbol the ISO 100 rung goes in
 * once and stays.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "hisi_state.h"

/* ================================================================
 * THE PARSED SHAPE
 * ================================================================
 *
 * Plain unsigned ints, one per field the text can name, so the tokenizer
 * can store through a byte offset without knowing about bitfields. Packing
 * into the ABI structure -- with each field clamped to its width -- is the
 * separate, last step.
 */

typedef struct {
    unsigned ies0, ies1, ies2, ies3, iedz;
} nrx_x_iey;

typedef struct {
    unsigned spn6, sfr, sbn6, pbr6;
    unsigned srt0, srt1, jmode, deidx;
    unsigned sfr6[4], sbr6[2], derate;
    unsigned sfs1, sft1, sbr1;
    unsigned sfs2, sft2, sbr2;
    unsigned sfs4, sft4, sbr4;
    unsigned sth1, sfn1, sfn0, sthd1;
    unsigned sth2, sfn2, kmode, sthd2;
} nrx_x_sfy;

typedef struct {
    unsigned tfs0, tdz0, tdx0;
    unsigned tfs1, tdz1, tdx1;
    unsigned sdz0, str0, sdz1, str1;
    unsigned tss0, tsi0, tfr0[6];
    unsigned tss1, tsi1, tfr1[6];
    unsigned tfrs, ted, bref;
} nrx_x_tfy;

typedef struct {
    unsigned mai00, mai01, mai02, mai10, mai11, mai12;
    unsigned mabr0, mabr1;
    unsigned math0, mate0, matw, mathd0, math1, mathd1, mate1, mabw0, mabw1;
    unsigned advmath, advth;
} nrx_x_mdy;

typedef struct {
    unsigned sfc, tfc, trc, tpc, mode, presfc;
} nrx_x_nrc;

typedef struct {
    nrx_x_iey iey[5];
    nrx_x_sfy sfy[5];
    nrx_x_mdy mdy[2];
    nrx_x_tfy tfy[3];
    nrx_x_nrc nrc;
} nrx_x;

/* ================================================================
 * THE SPEC TABLE -- the sample's mapping, one row per tag occurrence
 * ================================================================ */

#define NRX_SLOTS_MAX 20

typedef struct {
    const char *tag;     /* without the leading '-' */
    unsigned char occ;   /* 1 = first appearance in a block, 2 = second */
    bool embedded;       /* takes `n` numbers then returns to the row */
    bool ignored;        /* majestic-only tags with no V3 field */
    unsigned char n;     /* numbers this occurrence owns */
    unsigned short slot[NRX_SLOTS_MAX]; /* byte offsets into nrx_x */
} nrx_spec;

#define O(m) ((unsigned short)offsetof(nrx_x, m))

/* Four columns (ps[0..3]) of X:Y:Z. */
#define COLS4_3(X, Y, Z)                                                                           \
    O(sfy[0].X), O(sfy[0].Y), O(sfy[0].Z), O(sfy[1].X), O(sfy[1].Y), O(sfy[1].Z), O(sfy[2].X),     \
        O(sfy[2].Y), O(sfy[2].Z), O(sfy[3].X), O(sfy[3].Y), O(sfy[3].Z)
/* Five columns (ps[0..4]) of X:Y:Z. */
#define COLS5_3(X, Y, Z)                                                                           \
    COLS4_3(X, Y, Z), O(sfy[4].X), O(sfy[4].Y), O(sfy[4].Z)
/* Five columns of K:X:Y:Z. */
#define COLS5_4(K, X, Y, Z)                                                                        \
    O(sfy[0].K), O(sfy[0].X), O(sfy[0].Y), O(sfy[0].Z), O(sfy[1].K), O(sfy[1].X), O(sfy[1].Y),     \
        O(sfy[1].Z), O(sfy[2].K), O(sfy[2].X), O(sfy[2].Y), O(sfy[2].Z), O(sfy[3].K),              \
        O(sfy[3].X), O(sfy[3].Y), O(sfy[3].Z), O(sfy[4].K), O(sfy[4].X), O(sfy[4].Y), O(sfy[4].Z)
/* Five columns of X:Y. */
#define COLS5_2(X, Y)                                                                              \
    O(sfy[0].X), O(sfy[0].Y), O(sfy[1].X), O(sfy[1].Y), O(sfy[2].X), O(sfy[2].Y), O(sfy[3].X),     \
        O(sfy[3].Y), O(sfy[4].X), O(sfy[4].Y)
/* Five columns of X. */
#define COLS5_1(X) O(sfy[0].X), O(sfy[1].X), O(sfy[2].X), O(sfy[3].X), O(sfy[4].X)
/* The temporal rows: pt[0].X0 | pt[1].X0:pt[1].X1 | pt[2].X0. */
#define TF4(X) O(tfy[0].X##0), O(tfy[1].X##0), O(tfy[1].X##1), O(tfy[2].X##0)

static const nrx_spec nrx_specs[] = {
    {"nXsf1", 1, false, false, 12, {COLS4_3(sfs1, sft1, sbr1)}},
    {"nXsf2", 1, false, false, 12, {COLS4_3(sfs2, sft2, sbr2)}},
    {"nXsf4", 1, false, false, 12, {COLS4_3(sfs4, sft4, sbr4)}},
    {"SelRt", 1, false, false, 2, {O(sfy[0].srt0), O(sfy[0].srt1)}},
    {"kmode", 1, true, false, 2, {O(sfy[2].kmode), O(sfy[3].kmode)}},
    {"DeRt", 1, false, false, 2, {O(sfy[0].derate), O(sfy[0].deidx)}},
    {"sfs5", 1, false, false, 3, {O(sfy[4].sfs1), O(sfy[4].sfs2), O(sfy[4].sfs4)}},
    {"nXsf5", 1, false, false, 20,
     {O(iey[0].ies0), O(iey[0].ies1), O(iey[0].ies2), O(iey[0].ies3), O(iey[1].ies0),
      O(iey[1].ies1), O(iey[1].ies2), O(iey[1].ies3), O(iey[2].ies0), O(iey[2].ies1),
      O(iey[2].ies2), O(iey[2].ies3), O(iey[3].ies0), O(iey[3].ies1), O(iey[3].ies2),
      O(iey[3].ies3), O(iey[4].ies0), O(iey[4].ies1), O(iey[4].ies2), O(iey[4].ies3)}},
    {"dzsf5", 1, false, false, 5,
     {O(iey[0].iedz), O(iey[1].iedz), O(iey[2].iedz), O(iey[3].iedz), O(iey[4].iedz)}},
    {"nXsf6", 1, false, false, 20, {COLS5_4(spn6, sbn6, pbr6, jmode)}},
    {"nXsfr6", 1, false, false, 20, {COLS5_4(sfr6[0], sfr6[1], sfr6[2], sfr6[3])}},
    {"nXsbr6", 1, false, false, 10, {COLS5_2(sbr6[0], sbr6[1])}},
    {"nXsfn", 1, false, false, 15, {COLS5_3(sfn0, sfn1, sfn2)}},
    {"nXsth", 1, false, false, 10, {COLS5_2(sth1, sth2)}},
    {"nXsthd", 1, false, false, 10, {COLS5_2(sthd1, sthd2)}},
    {"sfr", 1, false, false, 5, {COLS5_1(sfr)}},
    {"ref", 1, false, false, 2, {O(tfy[0].bref), O(tfy[1].bref)}},
    {"tedge", 1, false, false, 2, {O(tfy[1].ted), O(tfy[2].ted)}},
    {"mXmath", 1, true, false, 1, {O(mdy[1].math1)}},
    {"mXmathd", 1, true, false, 1, {O(mdy[1].mathd1)}},
    {"nXstr", 1, false, false, 4, {TF4(str)}},
    {"mXmate", 1, true, false, 1, {O(mdy[1].mate1)}},
    {"nXsdz", 1, false, false, 4, {TF4(sdz)}},
    {"mXmabw", 1, true, false, 1, {O(mdy[1].mabw1)}},
    {"nXtss", 1, false, false, 4, {TF4(tss)}},
    {"nXtsi", 1, false, false, 4, {TF4(tsi)}},
    {"nXtfs", 1, false, false, 4, {TF4(tfs)}},
    {"nXtdz", 1, false, false, 4, {TF4(tdz)}},
    {"nXtdx", 1, false, false, 4, {TF4(tdx)}},
    {"mode", 1, true, false, 1, {O(nrc.mode)}},
    {"nXtfrs", 1, false, false, 1, {O(tfy[0].tfrs)}},
    {"presfc", 1, true, false, 1, {O(nrc.presfc)}},
    /* -nXtfr0 spans two physical lines: [0..2] of each column, then -sfc
     * embedded, then [3..5] of each column, then -tfc. */
    {"nXtfr0", 1, false, false, 18,
     {O(tfy[0].tfr0[0]), O(tfy[0].tfr0[1]), O(tfy[0].tfr0[2]), O(tfy[1].tfr0[0]),
      O(tfy[1].tfr0[1]), O(tfy[1].tfr0[2]), O(tfy[2].tfr0[0]), O(tfy[2].tfr0[1]),
      O(tfy[2].tfr0[2]), O(tfy[0].tfr0[3]), O(tfy[0].tfr0[4]), O(tfy[0].tfr0[5]),
      O(tfy[1].tfr0[3]), O(tfy[1].tfr0[4]), O(tfy[1].tfr0[5]), O(tfy[2].tfr0[3]),
      O(tfy[2].tfr0[4]), O(tfy[2].tfr0[5])}},
    {"sfc", 1, true, false, 1, {O(nrc.sfc)}},
    {"tfc", 1, true, false, 1, {O(nrc.tfc)}},
    {"nXtfr1", 1, false, false, 6,
     {O(tfy[1].tfr1[0]), O(tfy[1].tfr1[1]), O(tfy[1].tfr1[2]), O(tfy[1].tfr1[3]),
      O(tfy[1].tfr1[4]), O(tfy[1].tfr1[5])}},
    {"tpc", 1, true, false, 1, {O(nrc.tpc)}},
    {"trc", 1, true, false, 1, {O(nrc.trc)}},
    {"mXid0", 1, false, false, 6,
     {O(mdy[0].mai00), O(mdy[0].mai01), O(mdy[0].mai02), O(mdy[1].mai00), O(mdy[1].mai01),
      O(mdy[1].mai02)}},
    {"mXid1", 1, false, false, 3, {O(mdy[0].mai10), O(mdy[0].mai11), O(mdy[0].mai12)}},
    {"mXmabr", 1, false, false, 3, {O(mdy[0].mabr0), O(mdy[0].mabr1), O(mdy[1].mabr0)}},
    {"AdvMath", 1, false, false, 1, {O(mdy[0].advmath)}},
    {"AdvTh", 1, false, false, 1, {O(mdy[0].advth)}},
    {"mXmath", 2, false, false, 3, {O(mdy[0].math0), O(mdy[0].math1), O(mdy[1].math0)}},
    {"mXmathd", 2, false, false, 3, {O(mdy[0].mathd0), O(mdy[0].mathd1), O(mdy[1].mathd0)}},
    {"mXmate", 2, false, false, 3, {O(mdy[0].mate0), O(mdy[0].mate1), O(mdy[1].mate0)}},
    {"mXmabw", 2, false, false, 3, {O(mdy[0].mabw0), O(mdy[0].mabw1), O(mdy[1].mabw0)}},
    {"mXmatw", 1, false, false, 2, {O(mdy[0].matw), O(mdy[1].matw)}},
};

#define NRX_NSPECS (sizeof(nrx_specs) / sizeof(nrx_specs[0]))

/* ================================================================
 * THE SET -- what the section accumulates, and what the driver is handed
 * ================================================================ */

struct hisi_nrx_set {
    int count;                                     /* 3DNRCount */
    int iso_n;                                     /* IsoThresh entries seen */
    unsigned int iso[V4_VPSS_NRX_MAX_BLOCKS];      /* ascending */
    unsigned char have[V4_VPSS_NRX_MAX_BLOCKS];    /* block parsed completely */
    nrx_x x[V4_VPSS_NRX_MAX_BLOCKS];

    /* The engine, live after hisi_nrx_apply. `n` is the validated rung
     * count; `base` is the driver's structure with everything the text
     * does not name; `engine` is set last, with release semantics, and is
     * what lets a tick on another thread trust the rest. */
    int n;
    v4_vpss_nrx_v3 base;
    unsigned int last_map;     /* MapISO of the rung set last */
    unsigned int last_iso;
    int last_lvl;              /* rung index the ISO fell on or below; -1 = none */
    int set_failures;          /* consecutive; three stops the engine */
    char engine;
    nrx_x cur;                 /* interpolation scratch */
    v4_vpss_nrx_v3 cur_blk;
};

static struct hisi_nrx_set *nrx_set(hisi_state_t *st)
{
    if (!st->nrx)
        st->nrx = calloc(1, sizeof(*st->nrx));
    return st->nrx;
}

void hisi_nrx_free(hisi_state_t *st)
{
    free(st->nrx);
    st->nrx = NULL;
}

/* ================================================================
 * THE TOKENIZER
 * ================================================================ */

static bool nrx_tag_eq(const char *tag, size_t len, const char *name)
{
    size_t i;

    for (i = 0; i < len; i++)
        if (!name[i] || tolower((unsigned char)tag[i]) != tolower((unsigned char)name[i]))
            return false;
    return name[len] == '\0';
}

/*
 * nrx_parse_block -- one 3DnrParam_N value, as hal_isp.c's reader
 * assembled it: the physical lines joined with single spaces, comments
 * already stripped. Returns the number of specs left incomplete, naming
 * the first in `missing`.
 */
static int nrx_parse_block(const char *s, nrx_x *x, const char **missing)
{
    unsigned char filled[NRX_NSPECS];
    unsigned char seen[NRX_NSPECS]; /* occurrences of each tag so far, by spec index */
    const nrx_spec *row = NULL, *emb = NULL;
    unsigned row_i = 0, emb_i = 0;
    unsigned i;
    int stray = 0;

    memset(x, 0, sizeof(*x));
    memset(filled, 0, sizeof(filled));
    memset(seen, 0, sizeof(seen));

    while (*s) {
        if (isspace((unsigned char)*s) || *s == '|' || *s == ':' || *s == ',') {
            s++;
        } else if (*s == '(') {
            /* "(0)", "(1)" ... row annotations from PQTools, not values. */
            const char *close = strchr(s, ')');
            s = close ? close + 1 : s + strlen(s);
        } else if (*s == '*' || *s == ';') {
            /* "***NRc***" banner, or the ";;;;" rule that ends a block. */
            while (*s == '*' || *s == ';')
                s++;
            while (*s && !isspace((unsigned char)*s) && *s != '|')
                s++;
        } else if (*s == '-' && isalpha((unsigned char)s[1])) {
            const char *t = s + 1;
            size_t len = 0;
            unsigned occ = 0, k;
            const nrx_spec *hit = NULL;

            while (isalnum((unsigned char)t[len]))
                len++;
            /* The occurrence is counted per tag name across all its specs. */
            for (k = 0; k < NRX_NSPECS; k++)
                if (nrx_tag_eq(t, len, nrx_specs[k].tag))
                    occ += seen[k];
            occ++;
            for (k = 0; k < NRX_NSPECS; k++)
                if (nrx_tag_eq(t, len, nrx_specs[k].tag) && nrx_specs[k].occ == occ) {
                    hit = &nrx_specs[k];
                    seen[k] = 1;
                    break;
                }
            if (!hit) {
                /* Unknown tag, or a third occurrence: its numbers must not
                 * fall into the open row. */
                row = NULL;
                emb = NULL;
            } else if (hit->embedded) {
                emb = hit;
                emb_i = 0;
            } else {
                row = hit;
                row_i = 0;
                emb = NULL;
            }
            s = t + len;
        } else if (isdigit((unsigned char)*s) || (*s == '-' && isdigit((unsigned char)s[1]))) {
            char *end;
            long v = strtol(s, &end, 10);
            const nrx_spec *dst = NULL;
            unsigned *idx = NULL;

            s = end;
            if (v < 0)
                v = 0;
            if (emb && emb_i < emb->n) {
                dst = emb;
                idx = &emb_i;
            } else if (row && row_i < row->n) {
                dst = row;
                idx = &row_i;
            }
            if (!dst) {
                stray++;
                continue;
            }
            if (!dst->ignored)
                *(unsigned *)((char *)x + dst->slot[*idx]) = (unsigned)v;
            (*idx)++;
            if (*idx == dst->n) {
                filled[dst - nrx_specs] = 1;
                if (dst == emb)
                    emb = NULL;
            }
        } else {
            s++; /* anything else is punctuation this dialect does not use */
        }
    }

    *missing = NULL;
    {
        int incomplete = 0;

        for (i = 0; i < NRX_NSPECS; i++) {
            if (nrx_specs[i].ignored || filled[i])
                continue;
            incomplete++;
            if (!*missing)
                *missing = nrx_specs[i].tag;
        }
        (void)stray;
        return incomplete;
    }
}

/* ================================================================
 * PACKING -- parsed ints into the ABI bitfields, clamped to width
 * ================================================================ */

static unsigned nrx_cl(unsigned v, unsigned max)
{
    return v > max ? max : v;
}

/* Lays the text's fields over `d`, which already holds the driver's
 * values for everything the text does not name. */
static void nrx_pack(v4_vpss_nrx_v3 *d, const nrx_x *x)
{
    int i, j;

    for (i = 0; i < 5; i++) {
        v4_vpss_nrx_iey *di = &d->iey[i];
        const nrx_x_iey *xi = &x->iey[i];
        v4_vpss_nrx_sfy *ds = &d->sfy[i];
        const nrx_x_sfy *xs = &x->sfy[i];

        di->ies0 = nrx_cl(xi->ies0, 255);
        di->ies1 = nrx_cl(xi->ies1, 255);
        di->ies2 = nrx_cl(xi->ies2, 255);
        di->ies3 = nrx_cl(xi->ies3, 255);
        di->iedz = nrx_cl(xi->iedz, 999);

        ds->spn6 = nrx_cl(xs->spn6, 5);
        ds->sbn6 = nrx_cl(xs->sbn6, 5);
        ds->pbr6 = nrx_cl(xs->pbr6, 16);
        ds->jmode = nrx_cl(xs->jmode, 4);
        ds->sfr = nrx_cl(xs->sfr, 31);
        for (j = 0; j < 4; j++)
            ds->sfr6[j] = nrx_cl(xs->sfr6[j], 31);
        ds->sbr6[0] = nrx_cl(xs->sbr6[0], 15);
        ds->sbr6[1] = nrx_cl(xs->sbr6[1], 15);
        ds->sfn0 = nrx_cl(xs->sfn0, 6);
        ds->sfn1 = nrx_cl(xs->sfn1, 6);
        ds->sfn2 = nrx_cl(xs->sfn2, 6);
        ds->sth1 = nrx_cl(xs->sth1, 511);
        ds->sth2 = nrx_cl(xs->sth2, 511);
        ds->sthd1 = nrx_cl(xs->sthd1, 511);
        ds->sthd2 = nrx_cl(xs->sthd2, 511);
        /* The 1:2:4 rows cover four columns; column 4 gets only its SFS
         * triple (the -sfs5 row). Writing the zeros the parser left for
         * column 4's SFT/SBR would retune it, so those stay the driver's. */
        if (i < 4) {
            ds->sfs1 = nrx_cl(xs->sfs1, 255);
            ds->sft1 = nrx_cl(xs->sft1, 255);
            ds->sbr1 = nrx_cl(xs->sbr1, 255);
            ds->sfs2 = nrx_cl(xs->sfs2, 255);
            ds->sft2 = nrx_cl(xs->sft2, 255);
            ds->sbr2 = nrx_cl(xs->sbr2, 255);
            ds->sfs4 = nrx_cl(xs->sfs4, 255);
            ds->sft4 = nrx_cl(xs->sft4, 255);
            ds->sbr4 = nrx_cl(xs->sbr4, 255);
        } else {
            ds->sfs1 = nrx_cl(xs->sfs1, 255);
            ds->sfs2 = nrx_cl(xs->sfs2, 255);
            ds->sfs4 = nrx_cl(xs->sfs4, 255);
        }
    }
    /* SelRt, DeRt and kmode name specific columns. */
    d->sfy[0].srt0 = nrx_cl(x->sfy[0].srt0, 16);
    d->sfy[0].srt1 = nrx_cl(x->sfy[0].srt1, 16);
    d->sfy[0].derate = nrx_cl(x->sfy[0].derate, 255);
    d->sfy[0].deidx = nrx_cl(x->sfy[0].deidx, 6);
    d->sfy[2].kmode = nrx_cl(x->sfy[2].kmode, 3);
    d->sfy[3].kmode = nrx_cl(x->sfy[3].kmode, 3);

    for (i = 0; i < 3; i++) {
        v4_vpss_nrx_tfy *dt = &d->tfy[i];
        const nrx_x_tfy *xt = &x->tfy[i];

        dt->str0 = nrx_cl(xt->str0, 31);
        dt->sdz0 = nrx_cl(xt->sdz0, 999);
        dt->tss0 = nrx_cl(xt->tss0, 15);
        dt->tsi0 = nrx_cl(xt->tsi0, 1);
        dt->tfs0 = nrx_cl(xt->tfs0, 15);
        dt->tdz0 = nrx_cl(xt->tdz0, 999);
        dt->tdx0 = nrx_cl(xt->tdx0, 3);
        for (j = 0; j < 6; j++)
            dt->tfr0[j] = nrx_cl(xt->tfr0[j], 31);
    }
    /* The second temporal path exists on column 1 only; ref on 0 and 1,
     * tedge on 1 and 2, tfrs on 0 -- the columns the text has them in. */
    d->tfy[1].str1 = nrx_cl(x->tfy[1].str1, 31);
    d->tfy[1].sdz1 = nrx_cl(x->tfy[1].sdz1, 999);
    d->tfy[1].tss1 = nrx_cl(x->tfy[1].tss1, 15);
    d->tfy[1].tsi1 = nrx_cl(x->tfy[1].tsi1, 1);
    d->tfy[1].tfs1 = nrx_cl(x->tfy[1].tfs1, 15);
    d->tfy[1].tdz1 = nrx_cl(x->tfy[1].tdz1, 999);
    d->tfy[1].tdx1 = nrx_cl(x->tfy[1].tdx1, 3);
    for (j = 0; j < 6; j++)
        d->tfy[1].tfr1[j] = nrx_cl(x->tfy[1].tfr1[j], 31);
    d->tfy[0].bref = nrx_cl(x->tfy[0].bref, 1);
    d->tfy[1].bref = nrx_cl(x->tfy[1].bref, 1);
    d->tfy[1].ted = nrx_cl(x->tfy[1].ted, 3);
    d->tfy[2].ted = nrx_cl(x->tfy[2].ted, 3);
    d->tfy[0].tfrs = nrx_cl(x->tfy[0].tfrs, 15);

    for (i = 0; i < 2; i++) {
        v4_vpss_nrx_mdy *dm = &d->mdy[i];
        const nrx_x_mdy *xm = &x->mdy[i];

        dm->mai00 = nrx_cl(xm->mai00, 3);
        dm->mai01 = nrx_cl(xm->mai01, 3);
        dm->mai02 = nrx_cl(xm->mai02, 3);
        dm->mabr0 = nrx_cl(xm->mabr0, 255);
        dm->math0 = nrx_cl(xm->math0, 999);
        dm->math1 = nrx_cl(xm->math1, 999);
        dm->mathd0 = nrx_cl(xm->mathd0, 999);
        dm->mathd1 = nrx_cl(xm->mathd1, 999);
        dm->mate0 = nrx_cl(xm->mate0, 8);
        dm->mate1 = nrx_cl(xm->mate1, 8);
        dm->mabw0 = nrx_cl(xm->mabw0, 9);
        dm->mabw1 = nrx_cl(xm->mabw1, 9);
        dm->matw = nrx_cl(xm->matw, 3);
    }
    d->mdy[0].mai10 = nrx_cl(x->mdy[0].mai10, 3);
    d->mdy[0].mai11 = nrx_cl(x->mdy[0].mai11, 3);
    d->mdy[0].mai12 = nrx_cl(x->mdy[0].mai12, 3);
    d->mdy[0].mabr1 = nrx_cl(x->mdy[0].mabr1, 255);
    d->mdy[0].advmath = nrx_cl(x->mdy[0].advmath, 1);
    d->mdy[0].advth = nrx_cl(x->mdy[0].advth, 999);

    d->nrc.sfc = nrx_cl(x->nrc.sfc, 255);
    d->nrc.tfc = nrx_cl(x->nrc.tfc, 32);
    d->nrc.tpc = nrx_cl(x->nrc.tpc, 32);
    d->nrc.trc = nrx_cl(x->nrc.trc, 255);
    d->nrc.mode = x->nrc.mode ? 1 : 0;
    d->nrc.presfc = nrx_cl(x->nrc.presfc, 32);
}

/* ================================================================
 * THE SECTION'S KEYS
 * ================================================================ */

static int nrx_nums(const char *s, unsigned int *out, int max)
{
    int n = 0;

    while (*s && n < max) {
        while (*s && !isdigit((unsigned char)*s))
            s++;
        if (!*s)
            break;
        out[n++] = (unsigned int)strtoul(s, (char **)&s, 10);
    }
    return n;
}

/*
 * hisi_nrx_key -- one key of [static_3dnr]. Returns false for a key it
 * does not know, so the caller can log it the way the other sections do.
 */
bool hisi_nrx_key(hisi_state_t *st, const char *key, const char *val)
{
    struct hisi_nrx_set *set = nrx_set(st);

    if (!set)
        return true; /* out of memory: swallow, apply will say "none" */

    if (!strcasecmp(key, "3DNRCount")) {
        unsigned int n = 0;

        nrx_nums(val, &n, 1);
        if (n < 1 || n > V4_VPSS_NRX_MAX_BLOCKS) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] 3DNRCount %u out of 1..%d; section ignored", n,
                         V4_VPSS_NRX_MAX_BLOCKS);
            set->count = 0;
        } else {
            set->count = (int)n;
        }
        return true;
    }
    if (!strcasecmp(key, "IsoThresh")) {
        set->iso_n = nrx_nums(val, set->iso, V4_VPSS_NRX_MAX_BLOCKS);
        return true;
    }
    if (strncasecmp(key, "3DnrParam_", 10) == 0 && isdigit((unsigned char)key[10])) {
        long idx = strtol(key + 10, NULL, 10);
        const char *missing;
        int left;

        if (idx < 0 || idx >= V4_VPSS_NRX_MAX_BLOCKS) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] %s: index out of range; ignored", key);
            return true;
        }
        left = nrx_parse_block(val, &set->x[idx], &missing);
        if (left) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] %s: %d rows short, first missing -%s; "
                         "block skipped",
                         key, left, missing);
            set->have[idx] = 0;
        } else {
            set->have[idx] = 1;
        }
        return true;
    }
    return false;
}

/* ================================================================
 * SELECTION -- on hal_dyn.c's ISO axis (hisi_iso_map, hisi_iso_lerp)
 * ================================================================ */

/*
 * nrx_select -- the parsed shape for this ISO: a rung, or the blend of the
 * two either side. nrx_x is all unsigned ints by construction, so the
 * blend runs over it as an array.
 */
static int nrx_select(struct hisi_nrx_set *set, unsigned iso, nrx_x *out)
{
    int lvl, n = set->n;
    unsigned mid, left, right, i;
    const unsigned *lo, *hi;
    unsigned *o = (unsigned *)out;

    for (lvl = 0; lvl < n; lvl++)
        if (iso <= set->iso[lvl])
            break;
    if (lvl == 0 || n == 1) {
        *out = set->x[0];
        return 0;
    }
    if (lvl == n) {
        *out = set->x[n - 1];
        return n - 1;
    }
    mid = hisi_iso_map(iso);
    left = hisi_iso_map(set->iso[lvl - 1]);
    right = hisi_iso_map(set->iso[lvl]);
    lo = (const unsigned *)&set->x[lvl - 1];
    hi = (const unsigned *)&set->x[lvl];
    for (i = 0; i < sizeof(nrx_x) / sizeof(unsigned); i++)
        o[i] = hisi_iso_lerp(mid, left, lo[i], right, hi[i]);
    return lvl;
}

/* Blend, pack over the base, write. */
static int nrx_write(hisi_state_t *st, struct hisi_nrx_set *set, unsigned iso)
{
    v4_vpss_grp_nrx_param param;
    int ret, lvl;

    lvl = nrx_select(set, iso, &set->cur);
    set->cur_blk = set->base;
    nrx_pack(&set->cur_blk, &set->cur);

    memset(&param, 0, sizeof(param));
    param.nr_ver = V4_VPSS_NR_V3;
    param.v3.opt_mode = V4_OPERATION_MODE_MANUAL;
    param.v3.manual = set->cur_blk;
    ret = st->vpss.fnSetGrpNRXParam(HISI_VPSS_GRP, &param);
    if (ret == 0) {
        /* Every MapISO step is a write; crossing a rung is worth a line.
         * Nine rungs make at most eight of these per dusk. */
        if (lvl != set->last_lvl && set->engine)
            HAL_LOG_INFO("3dnr: ISO %u -> %u, now %s rung %d (ISO %u); sfc %u tfs %u",
                         set->last_iso, iso, lvl > 0 && iso < set->iso[lvl] ? "below" : "on", lvl,
                         set->iso[lvl], set->cur_blk.nrc.sfc, set->cur_blk.tfy[1].tfs0);
        set->last_lvl = lvl;
        set->last_iso = iso;
        set->last_map = hisi_iso_map(iso);
        set->set_failures = 0;
    }
    return ret;
}

/* ================================================================
 * AUTO -- the whole ladder, and the driver does the selecting
 * ================================================================ */

/* VPSS_DRV_CopyNRXAutoParamFromUser's own bound on each threshold. Out of
 * range it answers ILLEGAL_PARAM, which is the same code a dozen other
 * things answer, so the check is worth making here where it can be named. */
#define NRX_AUTO_ISO_MIN 100u
#define NRX_AUTO_ISO_MAX 3276800u

/*
 * nrx_auto_try -- pack every rung and hand the driver the set.
 *
 * The driver copies the ISO array and the blocks into the group before it
 * returns, so neither has to outlive the call. Returns true when it took
 * them; *err is the driver's code when it answered one, and 0 when the
 * ladder never went in.
 */
static bool nrx_auto_try(hisi_state_t *st, struct hisi_nrx_set *set, int *err)
{
    v4_vpss_grp_nrx_param param;
    v4_vpss_nrx_v3 *blocks;
    int i, ret;

    *err = 0;
    for (i = 0; i < set->n; i++)
        if (set->iso[i] < NRX_AUTO_ISO_MIN || set->iso[i] > NRX_AUTO_ISO_MAX) {
            HAL_LOG_INFO("isp tuning: [static_3dnr] IsoThresh entry %d is %u, outside the "
                         "driver's AUTO range %u..%u",
                         i, set->iso[i], NRX_AUTO_ISO_MIN, NRX_AUTO_ISO_MAX);
            return false;
        }

    blocks = calloc((size_t)set->n, sizeof(*blocks));
    if (!blocks)
        return false;
    for (i = 0; i < set->n; i++) {
        blocks[i] = set->base;
        nrx_pack(&blocks[i], &set->x[i]);
    }

    memset(&param, 0, sizeof(param));
    param.nr_ver = V4_VPSS_NR_V3;
    param.v3.opt_mode = V4_OPERATION_MODE_AUTO;
    /* Unread in this mode, but a group's parameter is one object and
     * leaving half of it zeroed is not a description of anything. */
    param.v3.manual = set->base;
    param.v3.auto_.param_num = (unsigned int)set->n;
    param.v3.auto_.iso = set->iso;
    param.v3.auto_.params = blocks;

    ret = st->vpss.fnSetGrpNRXParam(HISI_VPSS_GRP, &param);
    free(blocks);
    if (ret) {
        *err = ret;
        return false;
    }
    return true;
}

/* ================================================================
 * APPLY, AND THE TICK
 * ================================================================ */

/*
 * hisi_nrx_apply -- validate the ladder, capture the driver's base, write
 * the rung for the ISO AE reports now, and arm the tick. Returns 1 when
 * something was written, 0 when the section was absent or empty, -1 when
 * it was present and the write failed (the caller's "failed" count).
 */
int hisi_nrx_apply(hisi_state_t *st, char *note, size_t note_len)
{
    struct hisi_nrx_set *set = st->nrx;
    v4_vpss_grp_nrx_param base;
    unsigned iso;
    int i, n, ret;

    if (!set || set->count == 0)
        return 0;

    n = set->count;
    for (i = 0; i < n; i++) {
        if (!set->have[i]) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] 3DNRCount is %d but 3DnrParam_%d is missing or "
                         "malformed; using the %d before it",
                         n, i, i);
            n = i;
            break;
        }
    }
    if (n == 0) {
        snprintf(note, note_len, "static_3dnr(no usable block)");
        return -1;
    }
    if (set->iso_n < n) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] IsoThresh has %d entries for %d blocks; using %d",
                     set->iso_n, n, set->iso_n);
        n = set->iso_n;
        if (n == 0) {
            snprintf(note, note_len, "static_3dnr(no IsoThresh)");
            return -1;
        }
    }
    for (i = 1; i < n; i++) {
        if (set->iso[i] <= set->iso[i - 1]) {
            HAL_LOG_WARN("isp tuning: [static_3dnr] IsoThresh not ascending at entry %d (%u after "
                         "%u); truncating to %d blocks",
                         i, set->iso[i], set->iso[i - 1], i);
            n = i;
            break;
        }
    }
    set->n = n;

    if (!st->vpss.fnGetGrpNRXParam || !st->vpss.fnSetGrpNRXParam) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] libmpi exports no HI_MPI_VPSS_%sGrpNRXParam; "
                     "3DNR keeps the driver's defaults",
                     st->vpss.fnGetGrpNRXParam ? "Set" : "Get");
        snprintf(note, note_len, "static_3dnr(no NRX symbols)");
        return -1;
    }

    /* The driver's current block, as the base every rung is laid over. */
    memset(&base, 0, sizeof(base));
    base.nr_ver = V4_VPSS_NR_V3;
    base.v3.opt_mode = V4_OPERATION_MODE_MANUAL;
    ret = st->vpss.fnGetGrpNRXParam(HISI_VPSS_GRP, &base);
    if (ret) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] HI_MPI_VPSS_GetGrpNRXParam(%d) failed: 0x%x; "
                     "3DNR keeps the driver's defaults",
                     HISI_VPSS_GRP, ret);
        snprintf(note, note_len, "static_3dnr(Get failed)");
        return -1;
    }
    set->base = base.v3.manual;
    set->last_lvl = -1;

    /* AUTO first. Nothing is armed on this path: the driver has the whole
     * ladder and picks per frame, so there is no tick to run. */
    if (nrx_auto_try(st, set, &ret)) {
        HAL_LOG_INFO("isp tuning: [static_3dnr] %d rungs, ISO %u..%u; the driver has the ladder "
                     "and selects per frame (AUTO)",
                     n, set->iso[0], set->iso[n - 1]);
        return 1;
    }
    if (ret)
        HAL_LOG_INFO("isp tuning: [static_3dnr] HI_MPI_VPSS_SetGrpNRXParam refused the ladder "
                     "(AUTO): 0x%x; raptor selects the rung instead",
                     (unsigned)ret);
    else
        HAL_LOG_INFO("isp tuning: [static_3dnr] the ladder cannot go in as AUTO; raptor selects "
                     "the rung instead");

    if (!hisi_iso_query(st, &iso, NULL))
        iso = 0;
    ret = nrx_write(st, set, iso ? iso : set->iso[0]);
    if (ret) {
        HAL_LOG_WARN("isp tuning: [static_3dnr] HI_MPI_VPSS_SetGrpNRXParam(%d) failed: 0x%x; "
                     "3DNR keeps the driver's defaults",
                     HISI_VPSS_GRP, ret);
        snprintf(note, note_len, "static_3dnr(Set failed)");
        return -1;
    }

    if (iso) {
        HAL_LOG_INFO("isp tuning: [static_3dnr] %d rungs, ISO %u..%u; AE at ISO %u, rung written "
                     "(sfc %u, tfs %u); tracking ISO once a second",
                     n, set->iso[0], set->iso[n - 1], iso, set->cur_blk.nrc.sfc,
                     set->cur_blk.tfy[1].tfs0);
        __atomic_store_n(&set->engine, 1, __ATOMIC_RELEASE);
    } else {
        HAL_LOG_INFO("isp tuning: [static_3dnr] %d rungs, ISO %u..%u; no ISO query "
                     "(HI_MPI_ISP_QueryExposureInfo unresolved), ISO %u rung written and left",
                     n, set->iso[0], set->iso[n - 1], set->iso[0]);
    }
    return 1;
}

/* Whether the tick has anything to feed here. */
bool hisi_nrx_armed(hisi_state_t *st)
{
    return st->nrx && __atomic_load_n(&st->nrx->engine, __ATOMIC_ACQUIRE);
}

/*
 * hisi_nrx_on_iso -- the ladder, given AE's ISO: from hal_dyn.c's tick
 * once a second, or the host test directly. A write when the ISO has
 * moved a MapISO step; three failed writes running stop the engine.
 */
void hisi_nrx_on_iso(hisi_state_t *st, unsigned iso)
{
    struct hisi_nrx_set *set = st->nrx;
    int ret;

    if (!set || !__atomic_load_n(&set->engine, __ATOMIC_ACQUIRE))
        return;
    if (!iso || hisi_iso_map(iso) == set->last_map)
        return;
    ret = nrx_write(st, set, iso);
    if (ret == 0) {
        HAL_LOG_DBG("3dnr: ISO %u -> rung for %u written (sfc %u, tfs %u)", set->last_iso, iso,
                    set->cur_blk.nrc.sfc, set->cur_blk.tfy[1].tfs0);
        return;
    }
    if (++set->set_failures >= 3) {
        HAL_LOG_WARN("3dnr: HI_MPI_VPSS_SetGrpNRXParam failed three times running (last 0x%x); "
                     "leaving the ISO %u rung in place and stopping",
                     ret, set->last_iso);
        __atomic_store_n(&set->engine, 0, __ATOMIC_RELEASE);
    }
}
