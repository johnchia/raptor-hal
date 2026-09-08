/*
 * hal_ladder.c -- the rest of the per-light sections: eight ladders the
 * vendor's scene file carries beside the three hal_dyn.c walks.
 *
 * WHAT THIS IS. Each of these is a short table over an axis, read once at
 * load and written by get-modify-set whenever AE moves far enough:
 *
 *   on ISO
 *   [dynamic_dpc]           five defect-pixel fields, frame 0
 *   [dynamic_blc]           the four manual black levels, and the mode
 *   [dynamic_color_sector]  seven CCM tables of six hue and six sat shifts
 *   [dynamic_ca]            the CA module's two 128-entry luma LUTs, one
 *                           whole pair per column
 *   on exposure (ISO x integration time / 100, hisi_iso_query's number)
 *   [dynamic_ae]            AE target compensation and max_hist_offset
 *   [dynamic_ldci]          LDCI on or off, per band
 *   [dynamic_fps]           the sensor rate and the AE's longest shutter
 *
 *   [dynamic_nr]            read, and never written on a linear pipe --
 *                           see THE ONE THAT DOES NOTHING below.
 *
 * The vendor's scene_auto sample is the reference for every rule here,
 * as it is for hal_dyn.c: ot_scene_set_dynamic_* in ot_scene_setparam.c,
 * with the band chosen by scene_get_level_ltoh (first threshold the value
 * is at or under, clamped to the last), the bottom and top bands taken
 * whole and the bands between blended by scene_interpulate. Two of the
 * sample's own irregularities are kept because the tables were tuned
 * against them: [dynamic_ldci] never blends (an enable cannot), and
 * [dynamic_dpc]'s sup_twinkle_en takes the LEFT column between bands
 * rather than the right, which is what the sample does.
 *
 * WHICH OF THEM APPLY. Unlike hal_dyn.c's three, these are gated by
 * [module_state] -- bDynamicAE, bDynamicFps, bDynamicLdci, bDynamicDpc,
 * bDynamicBLC, bDynamicColorSector, bDynamicNr, and bDynamicCA or
 * bDynamicLinearCA for the CA ladder -- through the same mask hal_isp.c
 * keeps for the static sections. Those flags carry real intent here: the
 * vendor's own OS04D10 profile ships a full [dynamic_nr] under
 * bDynamicNr=0 and chooses the linear CA path over the WDR one by flag,
 * and a file with no [module_state] at all gets everything, as before.
 *
 * WHAT dynamic_fps IS ALLOWED TO DO. The vendor's version drops the
 * sensor to 5 fps at night, lengthens the AE's shutter to match, and
 * rewrites encoder channel 0's rate and GOP. Here the sensor rate goes
 * through the same path hal_isp_set_sensor_fps uses, so hal_framesource's
 * pacing baseline follows it -- but the ladder may only LOWER the rate
 * below what rvd asked for (the operator's number is a ceiling, exactly
 * as the sample's own fps_max override treats it), and the encoders are
 * not touched: their rate and GOP are rvd's config, and fewer frames
 * arriving than the rate control expects is a conservative bitrate, not a
 * broken stream. venc_gop is read and reported, not applied. When the
 * ceiling holds the rate below the band's, the shutter limit is stretched
 * to the actual frame period, which is what the band's own number is at
 * the band's own rate.
 *
 * THE ONE THAT DOES NOTHING. [dynamic_nr] is the WDR half of the bayer
 * NR: its ISO rows are ot_isp_nr_wdr_attr's snr_sfm0_wdr_strength and
 * snr_sfm0_fusion_strength, the strengths of the frame-merge filter, and
 * its fine_strength_l/h pair is written by the sample only when the pipe
 * is in a WDR mode. The linear per-ISO NR ladder is [static_nr]'s own
 * auto table, which the ISP interpolates itself. So the section is parsed
 * so its keys are known, and noted once, and the linear pipe gets no
 * write -- the same treatment hal_dyn.c gives the WDR rows of
 * [dynamic_dehaze]. Transcribing the NR attribute's tail for a write that
 * would never fire here is the kind of unverified ABI this backend has
 * already been bitten by (v5_nr.h).
 *
 * THE ae_comp KNOB. [dynamic_ae] varies the very field the ae_comp knob
 * pins, so the two share the arrangement hal_dyn.c and the drc_strength
 * knob have: a pinned knob holds this ladder's AE engine for as long as
 * it stands, `auto` hands the field back, and the knob's caps offer
 * `auto` only when the loaded file has a curve here. See hal_knob.c.
 *
 * WHEN A WRITE HAPPENS. The ISO engines write when hisi_iso_map's step
 * changes, as hal_dyn.c's DRC engine does. The exposure engines with a
 * blend (AE, fps) compute their candidate values every tick -- no MPI
 * call in that -- and write only when a value differs from the last one
 * written, which gives the vendor's glide across a band without a write
 * per second for nothing; LDCI writes on a band change. Three failed
 * writes in a row stop an engine and leave the others running.
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
#include "v5_isp_tune.h"

#define LAD_COLS 16 /* HI_SCENE_ISO_STRENGTH_NUM, and the largest exposure count */

/* ================================================================
 * THE ENGINES AND THEIR ROWS
 * ================================================================ */

enum { LAD_AE, LAD_FPS, LAD_LDCI, LAD_DPC, LAD_BLC, LAD_CS, LAD_CA, LAD_NR, LAD_N };

/*
 * One flat row space across the scalar engines, so parsing, validation
 * and the "has this row changed" test are written once. CA's LUT columns
 * and the singletons (advance_ae, black_level_mode) live beside it.
 */
enum {
    R_AE_COMP,
    R_AE_HIST,
    R_FPS,
    R_FPS_GOP,
    R_FPS_AEMAX,
    R_LDCI_EN,
    R_DPC_TWINKLE,
    R_DPC_SOFT_THR,
    R_DPC_SOFT_SLOPE,
    R_DPC_BRIGHT,
    R_DPC_DARK,
    R_BLC_R,
    R_BLC_GR,
    R_BLC_GB,
    R_BLC_B,
    R_CS_0, /* color_tab<j>_{hue,sat}_shift_<i>: R_CS_0 + (j * 2 + kind) * 6 + i */
    R_NF = R_CS_0 + V5_ISP_CCM_MATRIX_NUM * 2 * V5_ISP_COLOR_SECTORS
};

#define ROW_LEFT 1 /* between bands, take the left column rather than blend */
#define ROW_BAND 2 /* never blend: the band's column, whatever the level */

static const struct {
    unsigned char eng;
    const char *key;
    long lo, hi; /* the field's range in its attribute */
    unsigned char flags;
} lad_rows[R_CS_0] = {
    [R_AE_COMP] = {LAD_AE, "auto_compensation", 0, 0xFF, 0},
    [R_AE_HIST] = {LAD_AE, "auto_max_hist_offset", 0, 0xFF, 0},
    [R_FPS] = {LAD_FPS, "fps", 0, 240, 0},
    [R_FPS_GOP] = {LAD_FPS, "venc_gop", 0, 65535, 0},
    [R_FPS_AEMAX] = {LAD_FPS, "ae_max_time", 0, 0x7FFFFFFF, 0},
    [R_LDCI_EN] = {LAD_LDCI, "enable", 0, 1, ROW_BAND},
    [R_DPC_TWINKLE] = {LAD_DPC, "sup_twinkle_en", 0, 1, ROW_LEFT},
    [R_DPC_SOFT_THR] = {LAD_DPC, "soft_thr", 0, 0x7F, 0},
    [R_DPC_SOFT_SLOPE] = {LAD_DPC, "soft_slope", 0, 0xFF, 0},
    [R_DPC_BRIGHT] = {LAD_DPC, "bright_strength", 0, 0xFF, 0},
    [R_DPC_DARK] = {LAD_DPC, "dark_strength", 0, 0xFF, 0},
    [R_BLC_R] = {LAD_BLC, "blc_r", 0, 0xFFFF, 0},
    [R_BLC_GR] = {LAD_BLC, "blc_gr", 0, 0xFFFF, 0},
    [R_BLC_GB] = {LAD_BLC, "blc_gb", 0, 0xFFFF, 0},
    [R_BLC_B] = {LAD_BLC, "blc_b", 0, 0xFFFF, 0},
};

#define CS_LO 0
#define CS_HI 0x28 /* both shifts: Range [0x0, 0x28] */

/* The section, its count key and its threshold row; `iso` says which axis. */
static const struct {
    const char *sect;
    const char *cnt_key;
    const char *thr_key;
    bool iso;
} lad_meta[LAD_N] = {
    [LAD_AE] = {"dynamic_ae", "ae_exposure_cnt", "exp_ltoh_thresh", false},
    [LAD_FPS] = {"dynamic_fps", "fps_exposure_cnt", "exp_ltoh_thresh", false},
    [LAD_LDCI] = {"dynamic_ldci", "enable_cnt", "enable_exp_thresh_ltoh", false},
    [LAD_DPC] = {"dynamic_dpc", "iso_count", "iso_level", true},
    [LAD_BLC] = {"dynamic_blc", "blc_count", "iso_thresh", true},
    [LAD_CS] = {"dynamic_color_sector", "iso_count", "iso_level", true},
    [LAD_CA] = {"dynamic_ca", "iso_count", "iso_level", true},
    [LAD_NR] = {"dynamic_nr", "coring_ratio_count", "coring_ratio_iso", true},
};

/*
 * Keys the vendor's loader reads and this backend has no use for: the
 * high-to-low threshold rows (the sample selects with the ltoh row
 * everywhere it has both), the WDR halves of AE, CA and NR, and LDCI's
 * he_pos_wgt trio, which the sample loads and never writes. Recognised so
 * a file that carries them is not reported as unmapped, key by key.
 */
static const struct {
    unsigned char eng;
    const char *key;
} lad_noted[] = {
    {LAD_AE, "exp_htol_thresh"},
    {LAD_AE, "wdr_ratio_threshold"},
    {LAD_AE, "h_advance_ae"},
    {LAD_FPS, "exp_htol_thresh"},
    {LAD_LDCI, "exp_thresh_cnt"},
    {LAD_LDCI, "exp_thresh_ltoh"},
    {LAD_LDCI, "manual_ldci_he_pos_wgt"},
    {LAD_CA, "ratio_count"},
    {LAD_CA, "ratio_level"},
    {LAD_CA, "blend_weight"},
    {LAD_NR, "ratio_count"},
    {LAD_NR, "ratio_level"},
    {LAD_NR, "wdr_ratio_threshold"},
    {LAD_NR, "fine_strength_l"},
    {LAD_NR, "fine_strength_h"},
};

struct lad_eng {
    bool seen;
    int cnt; /* the count key; 0 = not given */
    int thr_n;
    unsigned long long thr[LAD_COLS];
    int n; /* validated columns; 0 = section off */
    int last_lvl;
    int failures;
    char engine;
};

struct hisi_lad_set {
    struct lad_eng e[LAD_N];
    int v[R_NF][LAD_COLS];
    unsigned char vn[R_NF]; /* columns given per row; 0 = row absent */
    int cur[R_NF];          /* what the last write laid down, per row */

    int ae_adv;   /* l_advance_ae; -1 = not given */
    bool ae_held; /* the ae_comp knob is pinned */
    int blc_mode; /* black_level_mode; -1 = not given */

    /* [dynamic_ca]: a whole LUT pair per column, allocated on first sight. */
    unsigned int (*ca_ratio)[V5_ISP_CA_LUT];
    unsigned int (*ca_sat)[V5_ISP_CA_LUT];
    unsigned char ca_have[2][LAD_COLS];

    /* [dynamic_fps]: rvd's rate is the ceiling; what the ladder last set. */
    float fps_base;
    unsigned fps_cur;
    unsigned ae_time_cur;
    bool fps_clamped_noted;

    const char *err;   /* the vendor call the last failed write died in */
    unsigned last_map; /* hisi_iso_map of the ISO the ISO engines last wrote for */
    unsigned last_iso;
    unsigned long long last_exp;
    char engine; /* any armed; release/acquire */
};

static struct hisi_lad_set *lad_set(hisi_state_t *st)
{
    int i;

    if (!st->lad) {
        st->lad = calloc(1, sizeof(*st->lad));
        if (st->lad) {
            for (i = 0; i < LAD_N; i++)
                st->lad->e[i].last_lvl = -1;
            st->lad->ae_adv = -1;
            st->lad->blc_mode = -1;
        }
    }
    return st->lad;
}

void hisi_lad_free(hisi_state_t *st)
{
    if (st->lad) {
        free(st->lad->ca_ratio);
        free(st->lad->ca_sat);
    }
    free(st->lad);
    st->lad = NULL;
}

/* ================================================================
 * THE KEYS
 * ================================================================ */

/* scene_get_level_ltoh, as hal_dyn.c and hal_nrx.c each have it. */
static int lad_level(unsigned long long v, int n, const unsigned long long *thr)
{
    int l;

    if (n <= 0)
        return 0;
    for (l = 0; l < n; l++)
        if (v <= thr[l])
            return l;
    return n - 1;
}

static int lad_nums(const char *s, long *out, int max)
{
    int n = 0;

    while (*s && n < max) {
        while (*s && !isdigit((unsigned char)*s) && !(*s == '-' && isdigit((unsigned char)s[1])))
            s++;
        if (!*s)
            break;
        out[n++] = strtol(s, (char **)&s, 10);
    }
    return n;
}

static long lad_clamp(long v, long lo, long hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static int lad_engine_of(const char *sect)
{
    int i;

    for (i = 0; i < LAD_N; i++)
        if (!strcasecmp(sect, lad_meta[i].sect))
            return i;
    return -1;
}

/* color_tab<j>_hue_shift_<i> / color_tab<j>_sat_shift_<i> -> row, or -1. */
static int lad_cs_row(const char *key)
{
    long j, i;
    char *end;
    int kind;

    if (strncasecmp(key, "color_tab", 9) != 0 || !isdigit((unsigned char)key[9]))
        return -1;
    j = strtol(key + 9, &end, 10);
    if (j < 0 || j >= V5_ISP_CCM_MATRIX_NUM)
        return -1;
    if (!strncasecmp(end, "_hue_shift_", 11))
        kind = 0;
    else if (!strncasecmp(end, "_sat_shift_", 11))
        kind = 1;
    else
        return -1;
    end += 11;
    if (!isdigit((unsigned char)*end))
        return -1;
    i = strtol(end, &end, 10);
    if (*end || i < 0 || i >= V5_ISP_COLOR_SECTORS)
        return -1;
    return R_CS_0 + ((int)j * 2 + kind) * V5_ISP_COLOR_SECTORS + (int)i;
}

static bool lad_ca_alloc(struct hisi_lad_set *d)
{
    if (d->ca_ratio && d->ca_sat)
        return true;
    if (!d->ca_ratio)
        d->ca_ratio = calloc(LAD_COLS, sizeof(*d->ca_ratio));
    if (!d->ca_sat)
        d->ca_sat = calloc(LAD_COLS, sizeof(*d->ca_sat));
    if (d->ca_ratio && d->ca_sat)
        return true;
    HAL_LOG_WARN("isp tuning: [dynamic_ca] out of memory; section ignored");
    return false;
}

/* ca_y_ratio_lut_iso_<n> / ca_y_sat_lut_iso_<n>: 128 entries, whole or none. */
static bool lad_key_ca_lut(struct hisi_lad_set *d, const char *key, const char *val)
{
    const char *num;
    unsigned int(*lut)[V5_ISP_CA_LUT];
    long idx;
    char *end;
    int kind, n = 0;

    if (!strncasecmp(key, "ca_y_ratio_lut_iso_", 19)) {
        kind = 0;
        num = key + 19;
    } else if (!strncasecmp(key, "ca_y_sat_lut_iso_", 17)) {
        kind = 1;
        num = key + 17;
    } else {
        return false;
    }
    if (!isdigit((unsigned char)*num))
        return false;
    idx = strtol(num, &end, 10);
    if (*end)
        return false;
    if (idx < 0 || idx >= LAD_COLS) {
        HAL_LOG_WARN("isp tuning: [dynamic_ca] %s: column out of 0..%d; ignored", key,
                     LAD_COLS - 1);
        return true;
    }
    if (!lad_ca_alloc(d))
        return true;
    lut = kind ? d->ca_sat : d->ca_ratio;
    while (*val && n < V5_ISP_CA_LUT) {
        while (*val && !isdigit((unsigned char)*val))
            val++;
        if (!*val)
            break;
        lut[idx][n++] = (unsigned int)lad_clamp(strtol(val, (char **)&val, 10), 0, 0xFFFF);
    }
    if (n != V5_ISP_CA_LUT) {
        HAL_LOG_WARN("isp tuning: [dynamic_ca] %s has %d of %d entries -- column skipped", key, n,
                     V5_ISP_CA_LUT);
        d->ca_have[kind][idx] = 0;
    } else {
        d->ca_have[kind][idx] = 1;
    }
    return true;
}

/*
 * hisi_lad_key -- one key of one of the eight sections. Returns false for
 * a key it does not know, so the caller logs it the way hal_isp.c does.
 */
bool hisi_lad_key(hisi_state_t *st, const char *sect, const char *key, const char *val)
{
    struct hisi_lad_set *d = lad_set(st);
    long tmp[LAD_COLS];
    struct lad_eng *e;
    int eng, row = -1, i, n;
    long lo, hi;

    if (!d)
        return true; /* out of memory: swallow; apply will find nothing */
    eng = lad_engine_of(sect);
    if (eng < 0)
        return false;
    e = &d->e[eng];
    e->seen = true;

    if (!strcasecmp(key, lad_meta[eng].cnt_key)) {
        e->cnt = lad_nums(val, tmp, 1) == 1 ? (int)lad_clamp(tmp[0], 0, LAD_COLS) : 0;
        return true;
    }
    if (!strcasecmp(key, lad_meta[eng].thr_key)) {
        n = lad_nums(val, tmp, LAD_COLS);
        for (i = 0; i < n; i++)
            e->thr[i] = (unsigned long long)lad_clamp(tmp[i], 0, 0x7FFFFFFFL);
        e->thr_n = n;
        return true;
    }

    /* The singletons. */
    if (eng == LAD_AE && !strcasecmp(key, "l_advance_ae")) {
        d->ae_adv = lad_nums(val, tmp, 1) == 1 && tmp[0] ? 1 : 0;
        return true;
    }
    if (eng == LAD_BLC && !strcasecmp(key, "black_level_mode")) {
        d->blc_mode = lad_nums(val, tmp, 1) == 1 ? (int)lad_clamp(tmp[0], 0, 2) : -1;
        return true;
    }

    /* Whole-LUT columns; the ratio-axis LUTs are the WDR path's. */
    if (eng == LAD_CA &&
        (lad_key_ca_lut(d, key, val) || !strncasecmp(key, "ca_y_ratio_lut_ratio_", 21)))
        return true;

    /* The WDR-only section: every row is known, none is kept. */
    if (eng == LAD_NR)
        return !strncasecmp(key, "snr_sfm0_", 9) || !strcasecmp(key, "fine_strength_l") ||
               !strcasecmp(key, "fine_strength_h") || !strcasecmp(key, "ratio_count") ||
               !strcasecmp(key, "ratio_level") || !strcasecmp(key, "wdr_ratio_threshold");

    /* The scalar rows. */
    if (eng == LAD_CS) {
        row = lad_cs_row(key);
        lo = CS_LO;
        hi = CS_HI;
    } else {
        for (i = 0; i < R_CS_0; i++)
            if (lad_rows[i].eng == eng && !strcasecmp(key, lad_rows[i].key)) {
                row = i;
                break;
            }
        if (row >= 0) {
            lo = lad_rows[row].lo;
            hi = lad_rows[row].hi;
        }
    }
    if (row >= 0) {
        n = lad_nums(val, tmp, LAD_COLS);
        for (i = 0; i < n; i++)
            d->v[row][i] = (int)lad_clamp(tmp[i], lo, hi);
        d->vn[row] = (unsigned char)n;
        return true;
    }

    for (i = 0; i < (int)(sizeof(lad_noted) / sizeof(lad_noted[0])); i++)
        if (lad_noted[i].eng == eng && !strcasecmp(key, lad_noted[i].key))
            return true;
    return false;
}

/* ================================================================
 * VALIDATION
 * ================================================================ */

/*
 * The columns an engine has: its threshold row, cut to its count where
 * the count is shorter, cut where the thresholds stop ascending, and cut
 * to the shortest row present. 0 means the section is off. The same
 * shape as hal_dyn.c's dyn_check_drc, once for all of them.
 */
static int lad_check(struct hisi_lad_set *d, int eng)
{
    struct lad_eng *e = &d->e[eng];
    const char *s = lad_meta[eng].sect;
    int n = e->thr_n, i, rows = 0;

    if (!e->seen)
        return 0;
    if (eng == LAD_NR) {
        HAL_LOG_INFO("isp tuning: [dynamic_nr] is the WDR half of bayer NR (the frame-merge "
                     "sfm0 strengths); a linear pipe has nothing to write");
        return 0;
    }
    if (n == 0) {
        HAL_LOG_WARN("isp tuning: [%s] no %s; section ignored", s, lad_meta[eng].thr_key);
        return 0;
    }
    if (e->cnt > 0 && e->cnt < n)
        n = e->cnt;
    for (i = 1; i < n; i++) {
        if (e->thr[i] <= e->thr[i - 1]) {
            HAL_LOG_WARN("isp tuning: [%s] %s not ascending at entry %d (%llu after %llu); "
                         "truncating to %d columns",
                         s, lad_meta[eng].thr_key, i, e->thr[i], e->thr[i - 1], i);
            n = i;
            break;
        }
    }

    if (eng == LAD_CA) {
        int k;

        if (!d->ca_ratio)
            return 0;
        for (k = 0; k < n; k++)
            if (!d->ca_have[0][k] || !d->ca_have[1][k])
                break;
        if (k == 0) {
            HAL_LOG_WARN("isp tuning: [dynamic_ca] no whole LUT pair for column 0; section "
                         "ignored");
            return 0;
        }
        if (k < n) {
            HAL_LOG_WARN("isp tuning: [dynamic_ca] LUT pair for column %d missing or short; "
                         "using %d columns",
                         k, k);
            n = k;
        }
        return n;
    }

    for (i = 0; i < R_NF; i++) {
        bool mine = i >= R_CS_0 ? eng == LAD_CS : lad_rows[i].eng == eng;

        if (!mine || !d->vn[i])
            continue;
        rows++;
        if (d->vn[i] < n) {
            HAL_LOG_WARN("isp tuning: [%s] row %d has %d of %d columns; using %d", s, i, d->vn[i],
                         n, d->vn[i]);
            n = d->vn[i];
        }
    }
    if (!rows && !(eng == LAD_BLC && d->blc_mode >= 0)) {
        HAL_LOG_WARN("isp tuning: [%s] no rows; section ignored", s);
        return 0;
    }
    if (eng == LAD_FPS && !d->vn[R_FPS] && !d->vn[R_FPS_AEMAX]) {
        HAL_LOG_WARN("isp tuning: [dynamic_fps] neither fps nor ae_max_time; section ignored");
        return 0;
    }
    return n;
}

/* ================================================================
 * THE WRITES
 * ================================================================ */

/* One row's value at `v` on the axis, by the vendor's rule and the row's
 * own exception to it. */
static int lad_col(const struct hisi_lad_set *d, int eng, int row, unsigned long long v, int lvl)
{
    const struct lad_eng *e = &d->e[eng];
    unsigned flags = row < R_CS_0 ? lad_rows[row].flags : 0;

    if (lvl == 0 || lvl == e->n - 1 || (flags & ROW_BAND))
        return d->v[row][lvl];
    if (flags & ROW_LEFT)
        return d->v[row][lvl - 1];
    return (int)hisi_iso_lerp(v, e->thr[lvl - 1], (unsigned long long)d->v[row][lvl - 1],
                              e->thr[lvl], (unsigned long long)d->v[row][lvl]);
}

/* Lay every row of an engine for `v` into cur[]; true if any moved. */
static bool lad_lay(struct hisi_lad_set *d, int eng, unsigned long long v, int lvl)
{
    bool moved = false;
    int i;

    for (i = 0; i < R_NF; i++) {
        bool mine = i >= R_CS_0 ? eng == LAD_CS : lad_rows[i].eng == eng;
        int c;

        if (!mine || !d->vn[i])
            continue;
        c = lad_col(d, eng, i, v, lvl);
        if (c != d->cur[i])
            moved = true;
        d->cur[i] = c;
    }
    return moved;
}

static void lad_landed(struct hisi_lad_set *d, int eng, int lvl)
{
    d->e[eng].last_lvl = lvl;
    d->e[eng].failures = 0;
}

static int lad_write_ae(hisi_state_t *st, struct hisi_lad_set *d, unsigned long long exposure)
{
    v5_isp_exp_attr a;
    int lvl, ret;

    d->err = "get/set_exposure_attr";
    if (!st->tune.fnGetExposureAttr || !st->tune.fnSetExposureAttr)
        return -1;
    d->err = "get_exposure_attr";
    ret = st->tune.fnGetExposureAttr(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    lvl = lad_level(exposure, d->e[LAD_AE].n, d->e[LAD_AE].thr);
    lad_lay(d, LAD_AE, exposure, lvl);
    if (d->vn[R_AE_COMP])
        a.auto_attr.compensation = (unsigned char)d->cur[R_AE_COMP];
    if (d->vn[R_AE_HIST])
        a.auto_attr.max_hist_offset = (unsigned char)d->cur[R_AE_HIST];
    /* The sample picks l_ or h_advance_ae on the WDR exposure ratio it
     * reads back; on a linear pipe the ratio is a constant 0x40 and never
     * crosses the threshold, so the l_ row is the row. */
    if (d->ae_adv >= 0)
        a.advance_ae = d->ae_adv;

    d->err = "set_exposure_attr";
    ret = st->tune.fnSetExposureAttr(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->e[LAD_AE].last_lvl && d->e[LAD_AE].engine)
            HAL_LOG_INFO("ae: exposure %llu -> %llu, band %d (<= %llu); compensation %d, "
                         "max_hist_offset %d",
                         d->last_exp, exposure, lvl, d->e[LAD_AE].thr[lvl], d->cur[R_AE_COMP],
                         d->cur[R_AE_HIST]);
        lad_landed(d, LAD_AE, lvl);
    }
    return ret;
}

static int lad_write_ldci(hisi_state_t *st, struct hisi_lad_set *d, unsigned long long exposure)
{
    v5_isp_ldci_attr a;
    int lvl, ret;

    d->err = "get/set_ldci_attr";
    if (!st->tune.fnGetLdciAttr || !st->tune.fnSetLdciAttr)
        return -1;
    d->err = "get_ldci_attr";
    ret = st->tune.fnGetLdciAttr(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    lvl = lad_level(exposure, d->e[LAD_LDCI].n, d->e[LAD_LDCI].thr);
    lad_lay(d, LAD_LDCI, exposure, lvl);
    a.enable = d->cur[R_LDCI_EN];

    d->err = "set_ldci_attr";
    ret = st->tune.fnSetLdciAttr(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->e[LAD_LDCI].last_lvl && d->e[LAD_LDCI].engine)
            HAL_LOG_INFO("ldci: exposure %llu -> %llu, band %d (<= %llu); %s", d->last_exp,
                         exposure, lvl, d->e[LAD_LDCI].thr[lvl], a.enable ? "on" : "off");
        lad_landed(d, LAD_LDCI, lvl);
    }
    return ret;
}

/*
 * The fps ladder's two halves. The rate goes through hisi_isp_fps_write,
 * the same path rvd's own set takes, capped at rvd's number; the shutter
 * limit is a get-modify-set of the exposure attribute. Either half alone
 * is fine: a file may carry one row and not the other.
 */
static int lad_write_fps(hisi_state_t *st, struct hisi_lad_set *d, unsigned long long exposure)
{
    struct lad_eng *e = &d->e[LAD_FPS];
    unsigned fps = 0, ae_time = 0;
    int lvl, ret = 0;

    lvl = lad_level(exposure, e->n, e->thr);
    lad_lay(d, LAD_FPS, exposure, lvl);

    if (d->vn[R_FPS] && d->cur[R_FPS] > 0) {
        unsigned base = (unsigned)(d->fps_base + 0.5f);

        fps = (unsigned)d->cur[R_FPS];
        if (base && fps > base) {
            if (!d->fps_clamped_noted && e->engine) {
                d->fps_clamped_noted = true;
                HAL_LOG_INFO("fps: the ladder's %u fps is above the %u rvd asked for; the "
                             "operator's rate is the ceiling",
                             fps, base);
            }
            fps = base;
        }
    }
    if (d->vn[R_FPS_AEMAX] && d->cur[R_FPS_AEMAX] > 0) {
        ae_time = (unsigned)d->cur[R_FPS_AEMAX];
        /* Held below the band's rate: the band's shutter limit is one
         * frame at the band's rate, so one frame at the actual rate is the
         * same limit. */
        if (fps && d->vn[R_FPS] && fps < (unsigned)d->cur[R_FPS] && ae_time < 1000000u / fps)
            ae_time = 1000000u / fps;
    }

    if (fps && fps != d->fps_cur) {
        d->err = "set_pub_attr";
        ret = hisi_isp_fps_write(st, (float)fps);
        if (ret)
            return ret;
        d->fps_cur = fps;
    }
    if (ae_time && ae_time != d->ae_time_cur) {
        v5_isp_exp_attr a;

        d->err = "get/set_exposure_attr";
        if (!st->tune.fnGetExposureAttr || !st->tune.fnSetExposureAttr)
            return -1;
        d->err = "get_exposure_attr";
        ret = st->tune.fnGetExposureAttr(HISI_VI_PIPE, &a);
        if (ret)
            return ret;
        a.auto_attr.exp_time_range.max = ae_time;
        d->err = "set_exposure_attr";
        ret = st->tune.fnSetExposureAttr(HISI_VI_PIPE, &a);
        if (ret)
            return ret;
        d->ae_time_cur = ae_time;
    }
    if (lvl != e->last_lvl && e->engine)
        HAL_LOG_INFO("fps: exposure %llu -> %llu, band %d (<= %llu); %u fps, shutter up to %u us"
                     "%s",
                     d->last_exp, exposure, lvl, e->thr[lvl], d->fps_cur, d->ae_time_cur,
                     d->vn[R_FPS_GOP] ? " (venc_gop read, not applied)" : "");
    lad_landed(d, LAD_FPS, lvl);
    return 0;
}

static int lad_write_dpc(hisi_state_t *st, struct hisi_lad_set *d, unsigned iso)
{
    v5_isp_dp_dynamic_attr a;
    v5_isp_dp_frame_dynamic *f = &a.frame_dynamic[0];
    int lvl, ret;

    d->err = "get/set_dp_dynamic_attr";
    if (!st->tune.fnGetDpDynamicAttr || !st->tune.fnSetDpDynamicAttr)
        return -1;
    d->err = "get_dp_dynamic_attr";
    ret = st->tune.fnGetDpDynamicAttr(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    lvl = lad_level(iso, d->e[LAD_DPC].n, d->e[LAD_DPC].thr);
    lad_lay(d, LAD_DPC, iso, lvl);
    if (d->vn[R_DPC_TWINKLE])
        f->sup_twinkle_en = d->cur[R_DPC_TWINKLE];
    if (d->vn[R_DPC_SOFT_THR])
        f->soft_thr = (signed char)d->cur[R_DPC_SOFT_THR];
    if (d->vn[R_DPC_SOFT_SLOPE])
        f->soft_slope = (unsigned char)d->cur[R_DPC_SOFT_SLOPE];
    if (d->vn[R_DPC_BRIGHT])
        f->bright_strength = (unsigned char)d->cur[R_DPC_BRIGHT];
    if (d->vn[R_DPC_DARK])
        f->dark_strength = (unsigned char)d->cur[R_DPC_DARK];

    d->err = "set_dp_dynamic_attr";
    ret = st->tune.fnSetDpDynamicAttr(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->e[LAD_DPC].last_lvl && d->e[LAD_DPC].engine)
            HAL_LOG_INFO("dpc: ISO %u -> %u, column %d (ISO %llu); bright %d, dark %d", d->last_iso,
                         iso, lvl, d->e[LAD_DPC].thr[lvl], d->cur[R_DPC_BRIGHT],
                         d->cur[R_DPC_DARK]);
        lad_landed(d, LAD_DPC, lvl);
    }
    return ret;
}

static int lad_write_blc(hisi_state_t *st, struct hisi_lad_set *d, unsigned iso)
{
    v5_isp_blc_attr a;
    int lvl, ret, i;

    d->err = "get/set_black_level_attr";
    if (!st->tune.fnGetBlackLevelAttr || !st->tune.fnSetBlackLevelAttr)
        return -1;
    d->err = "get_black_level_attr";
    ret = st->tune.fnGetBlackLevelAttr(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    lvl = lad_level(iso, d->e[LAD_BLC].n, d->e[LAD_BLC].thr);
    lad_lay(d, LAD_BLC, iso, lvl);
    if (d->blc_mode >= 0)
        a.black_level_mode = d->blc_mode;
    /* Every WDR frame slot gets the same four, as the sample writes them. */
    for (i = 0; i < V5_ISP_WDR_FRAMES; i++) {
        if (d->vn[R_BLC_R])
            a.manual_attr.black_level[i][0] = (unsigned short)d->cur[R_BLC_R];
        if (d->vn[R_BLC_GR])
            a.manual_attr.black_level[i][1] = (unsigned short)d->cur[R_BLC_GR];
        if (d->vn[R_BLC_GB])
            a.manual_attr.black_level[i][2] = (unsigned short)d->cur[R_BLC_GB];
        if (d->vn[R_BLC_B])
            a.manual_attr.black_level[i][3] = (unsigned short)d->cur[R_BLC_B];
    }

    d->err = "set_black_level_attr";
    ret = st->tune.fnSetBlackLevelAttr(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->e[LAD_BLC].last_lvl && d->e[LAD_BLC].engine)
            HAL_LOG_INFO("blc: ISO %u -> %u, column %d (ISO %llu); R %d Gr %d Gb %d B %d",
                         d->last_iso, iso, lvl, d->e[LAD_BLC].thr[lvl], d->cur[R_BLC_R],
                         d->cur[R_BLC_GR], d->cur[R_BLC_GB], d->cur[R_BLC_B]);
        lad_landed(d, LAD_BLC, lvl);
    }
    return ret;
}

static int lad_write_cs(hisi_state_t *st, struct hisi_lad_set *d, unsigned iso)
{
    v5_isp_color_sector_attr a;
    int lvl, ret, j, i;

    d->err = "get/set_color_sector_attr";
    if (!st->tune.fnGetColorSectorAttr || !st->tune.fnSetColorSectorAttr)
        return -1;
    d->err = "get_color_sector_attr";
    ret = st->tune.fnGetColorSectorAttr(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    lvl = lad_level(iso, d->e[LAD_CS].n, d->e[LAD_CS].thr);
    lad_lay(d, LAD_CS, iso, lvl);
    for (j = 0; j < V5_ISP_CCM_MATRIX_NUM; j++) {
        for (i = 0; i < V5_ISP_COLOR_SECTORS; i++) {
            int hue = R_CS_0 + (j * 2) * V5_ISP_COLOR_SECTORS + i;
            int sat = R_CS_0 + (j * 2 + 1) * V5_ISP_COLOR_SECTORS + i;

            if (d->vn[hue])
                a.auto_attr.color_tab[j].hue_shift[i] = (unsigned char)d->cur[hue];
            if (d->vn[sat])
                a.auto_attr.color_tab[j].sat_shift[i] = (unsigned char)d->cur[sat];
        }
    }

    d->err = "set_color_sector_attr";
    ret = st->tune.fnSetColorSectorAttr(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->e[LAD_CS].last_lvl && d->e[LAD_CS].engine)
            HAL_LOG_INFO("color_sector: ISO %u -> %u, column %d (ISO %llu); tab0 hue %d sat %d",
                         d->last_iso, iso, lvl, d->e[LAD_CS].thr[lvl], d->cur[R_CS_0],
                         d->cur[R_CS_0 + V5_ISP_COLOR_SECTORS]);
        lad_landed(d, LAD_CS, lvl);
    }
    return ret;
}

static int lad_write_ca(hisi_state_t *st, struct hisi_lad_set *d, unsigned iso)
{
    struct lad_eng *e = &d->e[LAD_CA];
    v5_isp_ca_attr *a;
    int lvl, ret, k;

    d->err = "get/set_ca_attr";
    if (!st->tune.fnGetCaAttr || !st->tune.fnSetCaAttr)
        return -1;
    a = calloc(1, sizeof(*a)); /* 1.5 KB; not an encoder thread's stack */
    if (!a)
        return -1;
    d->err = "get_ca_attr";
    ret = st->tune.fnGetCaAttr(HISI_VI_PIPE, a);
    if (ret == 0) {
        lvl = lad_level(iso, e->n, e->thr);
        if (lvl == 0 || lvl == e->n - 1) {
            memcpy(a->ca.y_ratio_lut, d->ca_ratio[lvl], sizeof(a->ca.y_ratio_lut));
            memcpy(a->ca.y_sat_lut, d->ca_sat[lvl], sizeof(a->ca.y_sat_lut));
        } else {
            for (k = 0; k < V5_ISP_CA_LUT; k++) {
                a->ca.y_ratio_lut[k] = hisi_iso_lerp(iso, e->thr[lvl - 1], d->ca_ratio[lvl - 1][k],
                                                     e->thr[lvl], d->ca_ratio[lvl][k]);
                a->ca.y_sat_lut[k] = hisi_iso_lerp(iso, e->thr[lvl - 1], d->ca_sat[lvl - 1][k],
                                                   e->thr[lvl], d->ca_sat[lvl][k]);
            }
        }
        d->err = "set_ca_attr";
        ret = st->tune.fnSetCaAttr(HISI_VI_PIPE, a);
        if (ret == 0) {
            if (lvl != e->last_lvl && e->engine)
                HAL_LOG_INFO("ca: ISO %u -> %u, column %d (ISO %llu); y_ratio[0] %u", d->last_iso,
                             iso, lvl, e->thr[lvl], a->ca.y_ratio_lut[0]);
            lad_landed(d, LAD_CA, lvl);
        }
    }
    free(a);
    return ret;
}

/* Which write an engine is; `v` is ISO or exposure by the engine's axis. */
static int lad_write(hisi_state_t *st, struct hisi_lad_set *d, int eng, unsigned long long v)
{
    switch (eng) {
    case LAD_AE:
        return lad_write_ae(st, d, v);
    case LAD_LDCI:
        return lad_write_ldci(st, d, v);
    case LAD_FPS:
        return lad_write_fps(st, d, v);
    case LAD_DPC:
        return lad_write_dpc(st, d, (unsigned)v);
    case LAD_BLC:
        return lad_write_blc(st, d, (unsigned)v);
    case LAD_CS:
        return lad_write_cs(st, d, (unsigned)v);
    case LAD_CA:
        return lad_write_ca(st, d, (unsigned)v);
    default:
        return -1;
    }
}

static const char *lad_what(int eng)
{
    static const char *const names[LAD_N] = {"ae",  "fps",          "ldci", "dpc",
                                             "blc", "color_sector", "ca",   "nr"};
    return names[eng];
}

static void lad_stop(struct hisi_lad_set *d, int eng, int ret)
{
    HAL_LOG_WARN("%s: ss_mpi_isp_%s failed three times running (last 0x%x); leaving the last "
                 "value in place and stopping",
                 lad_what(eng), d->err, ret);
    __atomic_store_n(&d->e[eng].engine, 0, __ATOMIC_RELEASE);
}

/* Would a write of this engine for `v` change anything? The blending
 * exposure engines' reason to write; no MPI call in it. */
static bool lad_moves(struct hisi_lad_set *d, int eng, unsigned long long v)
{
    int lvl = lad_level(v, d->e[eng].n, d->e[eng].thr);
    int saved[R_NF];
    bool moved;

    memcpy(saved, d->cur, sizeof(saved));
    moved = lad_lay(d, eng, v, lvl);
    memcpy(d->cur, saved, sizeof(saved));
    return moved || lvl != d->e[eng].last_lvl;
}

/*
 * hisi_lad_on_exposure -- the engines, given AE's numbers. What the tick
 * calls once it has them, right after hal_dyn.c's own.
 */
void hisi_lad_on_exposure(hisi_state_t *st, unsigned iso, unsigned long long exposure)
{
    struct hisi_lad_set *d = st->lad;
    unsigned map;
    bool any = false;
    int eng, ret;

    if (!d || !__atomic_load_n(&d->engine, __ATOMIC_ACQUIRE) || !iso)
        return;

    map = hisi_iso_map(iso);
    for (eng = 0; eng < LAD_N; eng++) {
        struct lad_eng *e = &d->e[eng];
        bool go;

        if (!e->engine)
            continue;
        if (eng == LAD_AE && d->ae_held)
            continue;
        if (lad_meta[eng].iso)
            go = map != d->last_map;
        else if (eng == LAD_LDCI)
            go = lad_level(exposure, e->n, e->thr) != e->last_lvl;
        else
            go = lad_moves(d, eng, exposure);
        if (!go)
            continue;
        ret = lad_write(st, d, eng, lad_meta[eng].iso ? iso : exposure);
        if (ret && ++e->failures >= 3)
            lad_stop(d, eng, ret);
    }
    if (map != d->last_map) {
        d->last_map = map;
        d->last_iso = iso;
    }
    d->last_exp = exposure;

    for (eng = 0; eng < LAD_N; eng++)
        any |= d->e[eng].engine != 0;
    if (!any)
        __atomic_store_n(&d->engine, 0, __ATOMIC_RELEASE);
}

bool hisi_lad_armed(hisi_state_t *st)
{
    return st->lad && __atomic_load_n(&st->lad->engine, __ATOMIC_ACQUIRE);
}

/* ================================================================
 * THE KNOB, AND rvd's RATE
 * ================================================================ */

/*
 * hisi_lad_ae_hold -- the ae_comp knob's grip on the AE ladder, the
 * arrangement hisi_dyn_drc_hold has with drc_strength. Release writes the
 * ladder's column for the exposure last seen, right away, so `auto` is
 * visible at once.
 */
void hisi_lad_ae_hold(hisi_state_t *st, bool hold)
{
    struct hisi_lad_set *d = st->lad;

    if (!d)
        return;
    if (hold) {
        if (d->e[LAD_AE].engine && !d->ae_held)
            HAL_LOG_INFO("ae: [dynamic_ae] held while ae_comp is pinned");
        d->ae_held = true;
        return;
    }
    d->ae_held = false;
    if (!d->e[LAD_AE].engine)
        return;
    d->e[LAD_AE].last_lvl = -1;
    if (d->last_exp && lad_write_ae(st, d, d->last_exp) == 0)
        HAL_LOG_INFO("ae: [dynamic_ae] released; compensation %d for exposure %llu",
                     d->cur[R_AE_COMP], d->last_exp);
    else
        HAL_LOG_INFO("ae: [dynamic_ae] released; the next AE step rewrites it");
}

/* Does the tuning vary AE compensation with the light? The ae_comp knob's
 * caps offer `auto` exactly when it does. */
bool hisi_lad_ae_curve(hisi_state_t *st)
{
    return st && st->lad && st->lad->e[LAD_AE].n >= 2 && st->lad->vn[R_AE_COMP];
}

/*
 * hisi_lad_fps_base -- rvd's own rate, from hal_isp_set_sensor_fps. The
 * ladder's ceiling moves with it, and the ladder answers on the next tick
 * (which re-lays the band's rate against the new ceiling) rather than
 * here, under whatever lock the caller holds.
 */
void hisi_lad_fps_base(hisi_state_t *st, float fps)
{
    struct hisi_lad_set *d = st->lad;

    if (!d)
        return;
    d->fps_base = fps;
    d->fps_clamped_noted = false;
    d->fps_cur = 0;              /* rvd just wrote the rate; the ladder's copy is stale */
    d->e[LAD_FPS].last_lvl = -1; /* and the next tick re-lays the band against it */
}

/* ================================================================
 * APPLY
 * ================================================================ */

static void lad_note(char *note, size_t len, const char *what)
{
    size_t have = strlen(note);

    if (have + strlen(what) + 2 >= len)
        return;
    snprintf(note + have, len - have, "%s%s", have ? " " : "", what);
}

static void lad_apply_fail(struct hisi_lad_set *d, int eng, int ret, char *note, size_t note_len)
{
    const char *sect = lad_meta[eng].sect;
    char what[64];

    if (ret == -1) {
        HAL_LOG_WARN("isp tuning: [%s] ss_mpi_isp_%s unresolved -- the module keeps the static "
                     "values",
                     sect, d->err);
        snprintf(what, sizeof(what), "%s(unresolved)", sect);
    } else {
        HAL_LOG_WARN("isp tuning: [%s] ss_mpi_isp_%s failed: 0x%x -- the module keeps the "
                     "static values",
                     sect, d->err, ret);
        snprintf(what, sizeof(what), "%s(%s failed)", sect,
                 strstr(d->err, "get") == d->err ? "Get" : "Set");
    }
    lad_note(note, note_len, what);
}

/*
 * hisi_lad_apply -- validate the sections, write each for the ISO and
 * exposure AE reports now (the first column without an AE to ask), and
 * arm the tick. The same contract as hisi_dyn_apply: returns how many
 * sections were written; *failed counts those present that could not be,
 * with `note` naming them for the load summary.
 */
int hisi_lad_apply(hisi_state_t *st, int *failed, char *note, size_t note_len)
{
    struct hisi_lad_set *d = st->lad;
    unsigned iso = 0;
    unsigned long long exposure = 0;
    bool have_ae, seen = false, usable = false, armed = false;
    int applied = 0, eng, ret, i;

    *failed = 0;
    note[0] = '\0';
    if (!d)
        return 0;

    for (eng = 0; eng < LAD_N; eng++) {
        d->e[eng].n = lad_check(d, eng);
        usable |= d->e[eng].n > 0;
        /* [dynamic_nr] alone is not a failure: idle by design on this pipe. */
        seen |= d->e[eng].seen && eng != LAD_NR;
    }
    if (!usable) {
        if (seen) {
            *failed = 1;
            lad_note(note, note_len, "dynamic_*(nothing usable)");
        }
        return 0;
    }

    have_ae = hisi_iso_query(st, &iso, &exposure);
    if (!have_ae) {
        iso = 0;
        exposure = 0;
    }
    for (i = 0; i < R_NF; i++)
        d->cur[i] = -1; /* so the first lay counts as a move */
    d->fps_base = st->mode.frame_rate;
    d->fps_cur = 0;
    d->ae_time_cur = 0;

    for (eng = 0; eng < LAD_N; eng++) {
        struct lad_eng *e = &d->e[eng];
        unsigned long long at;

        if (!e->n)
            continue;
        at = lad_meta[eng].iso ? (iso ? iso : e->thr[0]) : (have_ae ? exposure : e->thr[0]);
        e->last_lvl = -1;
        e->failures = 0;
        e->engine = 0;
        if (eng == LAD_AE && d->ae_held) {
            /* Pinned across a reload: arm without writing; release writes. */
            HAL_LOG_INFO("isp tuning: [dynamic_ae] %d bands; held while ae_comp is pinned", e->n);
            e->engine = have_ae && e->n > 1;
            armed |= e->engine;
            applied++;
            continue;
        }
        ret = lad_write(st, d, eng, at);
        if (ret) {
            lad_apply_fail(d, eng, ret, note, note_len);
            (*failed)++;
            e->n = 0;
            continue;
        }
        HAL_LOG_INFO("isp tuning: [%s] %d columns on %s, %llu..%llu; %s %llu%s", lad_meta[eng].sect,
                     e->n, lad_meta[eng].iso ? "ISO" : "exposure", e->thr[0], e->thr[e->n - 1],
                     have_ae ? "AE at" : "no AE query, first column at", at,
                     have_ae && e->n > 1 ? "; tracking" : "");
        applied++;
        e->engine = have_ae && e->n > 1;
        armed |= e->engine;
    }

    if (iso) {
        d->last_map = hisi_iso_map(iso);
        d->last_iso = iso;
        d->last_exp = exposure;
    }
    if (armed)
        __atomic_store_n(&d->engine, 1, __ATOMIC_RELEASE);
    return applied;
}
