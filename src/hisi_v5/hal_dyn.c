/*
 * hal_dyn.c -- the dynamic ISP sections, driven off AE once a second.
 *
 * WHAT THIS IS. A V5 scene file carries, beside the static sections
 * hal_isp.c writes once, three sections that are tables over an axis
 * rather than values:
 *
 *   [dynamic_linear_drc]  forty-three DRC rows, one column per iso_level
 *   [dynamic_dehaze]      one dehaze strength per exp_thresh_ltoh
 *   [dynamic_gamma]       total_num whole gamma curves, one per exposure band
 *
 * The vendor's scene_auto sample walks them from a thread of its own;
 * this walks them from the ISO tick below. Without them the picture keeps
 * whatever column the static load left, which is the daylight picture at
 * every ISO -- and gen4's night comparison (docs/hisilicon.md) is what
 * that costs: DRC lifting shadows a night scene does not have, and the
 * encoder paying for the noise the lift brings up.
 *
 * THE AXES ARE NOT THE ONES GEN4 USED. On V5 only DRC is on ISO. Dehaze
 * moved to exposure -- `exp_thresh_ltoh`, where gen4's dialect said
 * `IsoThresh` -- and gamma stays on exposure. Exposure is the sample's
 * scene_calculate_exp: ISO x integration time / 100, linear mode taking
 * `exp_time` (the WDR arms take the long/median/short field, and this
 * backend is linear-only until a WDR sensor turns up).
 *
 * BAND SELECTION AND BLEND are the sample's, copied rather than invented:
 * scene_get_level_ltoh picks the first band the value is at or under and
 * clamps to the last; a level of 0 or of count-1 takes that column whole,
 * and only the bands between blend (scene_interpulate, the rounding
 * linear one). scene_interpulate_signed rounds the other way, and the two
 * slope rows are the only signed ones.
 *
 * ONE TICK FOR EVERYTHING. AE's ISO is one ss_mpi_isp_query_exposure_info
 * call -- 5484 bytes, most of it a histogram, for two fields -- so there
 * is one clock here (hisi_dyn_tick, off the encoder's frame hook, once a
 * second, one thread at a time). It is the clock the 3DNR ladder will hang
 * off as well, once hal_nrx.c lands. Each engine keeps its own "moved a
 * step" test and its own failure count: three failed writes in a row stop
 * that engine and leave the others running.
 *
 * GAMMA FADES RATHER THAN CUTS. The sample writes its `interval` steps
 * back to back with a 30 ms sleep between them, which would block an
 * encoder thread for a third of a second. Here a level change starts a
 * fade and the tick shortens to 100 ms until it lands, so a ten-step fade
 * takes a second and no thread waits. The step function is the sample's
 * scene_time_filter.
 *
 * GET-MODIFY-SET, per write. Every write starts from a fresh Get, so the
 * static sections' values -- the tone-mapping curve, the dehaze LUT --
 * and anything else that touched the module survive underneath the fields
 * the column names.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* clock_gettime and CLOCK_MONOTONIC, under -std=c11. */
#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "hisi_state.h"
#include "v5_isp_tune.h"

#define DYN_COLS 16      /* HI_SCENE_DRC_ISO_MAX_COUNT, and the dehaze one */
#define DYN_GAMMA_MAX 10 /* HI_SCENE_GAMMA_EXPOSURE_MAX_COUNT */

/* ================================================================
 * THE AXES -- shared with hal_nrx.c
 * ================================================================
 *
 * hisi_iso_map is the step index the engines use as their "has the light
 * actually moved" test: about six steps per stop with ISO 100 at 0. It is
 * not the vendor's -- the vendor recomputes every frame and lets the
 * writes fall where they may -- but the same one gen4 uses, and it keeps
 * a wobbling AE from writing the ISP every second for nothing.
 *
 * hisi_iso_lerp is scene_interpulate; hisi_iso_lerp_signed is
 * scene_interpulate_signed, which rounds toward zero where the unsigned
 * one rounds to nearest. Both are copied for the rounding, not the shape.
 */

unsigned hisi_iso_map(unsigned iso)
{
    unsigned i, j;

    if (iso < 100)
        iso = 100; /* the driver's own floor for the iso field */
    i = (iso >= 200);
    i += (iso >= (200u << 1)) + (iso >= (400u << 1)) + (iso >= (400u << 2)) + (iso >= (400u << 3)) +
         (iso >= (400u << 4));
    i += (iso >= (400u << 5)) + (iso >= (400u << 6)) + (iso >= (400u << 7)) + (iso >= (400u << 8)) +
         (iso >= (400u << 9));
    i += (iso >= (400u << 10)) + (iso >= (400u << 11)) + (iso >= (400u << 12)) +
         (iso >= (400u << 13)) + (iso >= (400u << 14));
    j = (iso > (112u << i)) + (iso > (125u << i)) + (iso > (141u << i)) + (iso > (158u << i)) +
        (iso > (178u << i));
    return i * 6 + j + (iso >= 80) + (iso >= 90) + (iso >= 100) - 3;
}

unsigned hisi_iso_lerp(unsigned long long mid, unsigned long long left, unsigned long long lv,
                       unsigned long long right, unsigned long long rv)
{
    unsigned long long k;

    if (mid <= left)
        return (unsigned)lv;
    if (mid >= right)
        return (unsigned)rv;
    k = right - left;
    return (unsigned)(((right - mid) * lv + (mid - left) * rv + (k >> 1)) / k);
}

/* scene_interpulate_signed: the same blend, minus half a step. */
static int dyn_lerp_signed(long long mid, long long left, long long lv, long long right,
                           long long rv)
{
    long long k;

    if (mid <= left)
        return (int)lv;
    if (mid >= right)
        return (int)rv;
    k = right - left;
    return (int)(((right - mid) * lv + (mid - left) * rv - (k / 2)) / k);
}

/*
 * hisi_iso_query -- what AE is at now. False when there is no way to ask
 * or the ask failed; `exposure` is the sample's ISO x time / 100, for the
 * exposure-axis sections, and may be NULL.
 */
bool hisi_iso_query(hisi_state_t *st, unsigned *iso, unsigned long long *exposure)
{
    v5_isp_exp_info *info;
    bool ok = false;

    /* In v5_isp.h and st->isp, not the tuning impl: the symbol lives in
     * libss_mpi_ae.so and was already bound at bring-up. */
    if (!st->isp.fnQueryExposureInfo)
        return false;
    /* 5484 bytes, three quarters of it a histogram; not a stack frame for
     * an encoder thread. */
    info = calloc(1, sizeof(*info));
    if (!info)
        return false;
    if (st->isp.fnQueryExposureInfo(HISI_VI_PIPE, info) == 0 && info->iso) {
        *iso = info->iso;
        if (exposure)
            *exposure = (unsigned long long)info->iso * info->exp_time / 100;
        ok = true;
    }
    free(info);
    return ok;
}

/*
 * scene_get_level_ltoh over `n` thresholds: the first band the value is
 * at or under, clamped to the last band. Note this is not gen4's, which
 * returned `n` past the last threshold; the vendor clamps, and the
 * column rule below depends on the clamp.
 */
static int dyn_level(unsigned long long v, int n, const unsigned long long *thr)
{
    int l;

    if (n <= 0)
        return 0;
    for (l = 0; l < n; l++)
        if (v <= thr[l])
            return l;
    return n - 1;
}

/* Numbers out of a value: anything that is not a digit (or a minus before
 * one) separates. Returns how many landed. */
static int dyn_nums(const char *s, long *out, int max)
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

static long dyn_clamp(long v, long lo, long hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* ================================================================
 * THE SET
 * ================================================================ */

/*
 * [dynamic_linear_drc]'s rows, in the order scene_set_isp_drc_attr copies
 * them, with each field's range from the header. DRC_LUT_0..15 are the
 * bcnr detail-restore curve, whose sixteen nodes the dialect writes as
 * sixteen separate per-ISO rows.
 */
enum {
    DRC_SPATIAL_FILTER_COEF,
    DRC_RANGE_FILTER_COEF,
    DRC_DETAIL_ADJUST_COEF,
    DRC_RIM_REDUCTION_STRENGTH,
    DRC_RIM_REDUCTION_THRESHOLD,
    DRC_BRIGHT_GAIN_LIMIT,
    DRC_BRIGHT_GAIN_LIMIT_STEP,
    DRC_DARK_GAIN_LIMIT_LUMA,
    DRC_DARK_GAIN_LIMIT_CHROMA,
    DRC_HIGH_SAT_COLOR_CTRL,
    DRC_GLOBAL_COLOR_CTRL,
    DRC_LM_BRIGHT_MAX,
    DRC_LM_BRIGHT_MIN,
    DRC_LM_BRIGHT_THRESHOLD,
    DRC_LM_BRIGHT_SLOPE,
    DRC_LM_DARK_MAX,
    DRC_LM_DARK_MIN,
    DRC_LM_DARK_THRESHOLD,
    DRC_LM_DARK_SLOPE,
    DRC_CURVE_BRIGHTNESS,
    DRC_CURVE_CONTRAST,
    DRC_CURVE_TOLERANCE,
    DRC_BCNR_STRENGTH,
    DRC_ASYMMETRY,
    DRC_SECOND_POLE,
    DRC_COMPRESS,
    DRC_STRETCH,
    DRC_STRENGTH,
    DRC_LUT_0,
    DRC_NF = DRC_LUT_0 + V5_ISP_DRC_BCNR_NODES
};

static const struct {
    const char *key;
    long lo, hi; /* the field's range in ot_isp_drc_attr */
} drc_rows[DRC_NF] = {
    [DRC_SPATIAL_FILTER_COEF] = {"spatial_filter_coef", 0, 0x5},
    [DRC_RANGE_FILTER_COEF] = {"range_filter_coef", 0, 0xA},
    [DRC_DETAIL_ADJUST_COEF] = {"detail_adjust_coef", 0, 0xF},
    [DRC_RIM_REDUCTION_STRENGTH] = {"rim_reduction_strength", 0, 0x40},
    [DRC_RIM_REDUCTION_THRESHOLD] = {"rim_reduction_threshold", 0, 0x80},
    [DRC_BRIGHT_GAIN_LIMIT] = {"bright_gain_limit", 0, 0xF},
    [DRC_BRIGHT_GAIN_LIMIT_STEP] = {"bright_gain_limit_step", 0, 0xF},
    [DRC_DARK_GAIN_LIMIT_LUMA] = {"dark_gain_limit_luma", 0, 0x85},
    [DRC_DARK_GAIN_LIMIT_CHROMA] = {"dark_gain_limit_chroma", 0, 0x85},
    [DRC_HIGH_SAT_COLOR_CTRL] = {"high_saturation_color_ctrl", 0, 0xF},
    [DRC_GLOBAL_COLOR_CTRL] = {"global_color_ctrl", 0, 0xF},
    [DRC_LM_BRIGHT_MAX] = {"local_mixing_bright_max", 0, 0x80},
    [DRC_LM_BRIGHT_MIN] = {"local_mixing_bright_min", 0, 0x80},
    [DRC_LM_BRIGHT_THRESHOLD] = {"local_mixing_bright_threshold", 0, 0xFF},
    [DRC_LM_BRIGHT_SLOPE] = {"local_mixing_bright_slope", -7, 7},
    [DRC_LM_DARK_MAX] = {"local_mixing_dark_max", 0, 0x80},
    [DRC_LM_DARK_MIN] = {"local_mixing_dark_min", 0, 0x80},
    [DRC_LM_DARK_THRESHOLD] = {"local_mixing_dark_threshold", 0, 0xFF},
    [DRC_LM_DARK_SLOPE] = {"local_mixing_dark_slope", -7, 7},
    [DRC_CURVE_BRIGHTNESS] = {"curve_brightness", 0, 0xF},
    [DRC_CURVE_CONTRAST] = {"curve_contrast", 0, 0xF},
    [DRC_CURVE_TOLERANCE] = {"curve_tolerance", 0, 0xF},
    [DRC_BCNR_STRENGTH] = {"drc_bcnr_str", 0, 0x8},
    [DRC_ASYMMETRY] = {"asymmetry", 1, 30},
    [DRC_SECOND_POLE] = {"second_pole", 150, 210},
    [DRC_COMPRESS] = {"compress", 100, 200},
    [DRC_STRETCH] = {"stretch", 30, 60},
    [DRC_STRENGTH] = {"strength", 0, 0x3FF},
    /* detail_restore_lut_0 .. _15, filled below by name at parse time. */
};

/* Whether a row blends with the signed rounding. Only the two slopes. */
static bool drc_row_signed(int row)
{
    return row == DRC_LM_BRIGHT_SLOPE || row == DRC_LM_DARK_SLOPE;
}

struct hisi_dyn_set {
    struct {
        bool seen;
        int enable; /* -1 = not given */
        int cnt;    /* iso_count, 0 = not given */
        int iso_n;
        unsigned iso[DYN_COLS];
        long v[DRC_NF][DYN_COLS];
        int vn[DRC_NF]; /* columns given per row; 0 = row absent */
        int n;          /* validated columns; 0 = section off */
        int last_lvl;
        int failures;
        char engine;
        long cur[DRC_NF]; /* what was last written, for the log */
    } drc;

    struct {
        bool seen;
        int cnt; /* exp_thresh_cnt, 0 = not given */
        int thr_n, str_n;
        unsigned long long thr[DYN_COLS];
        long str[DYN_COLS];
        bool wdr_row_noted;
        int n;
        int last_lvl;
        int failures;
        char engine;
        unsigned cur;
    } dehaze;

    struct {
        bool seen;
        int total;    /* total_num, 0 = not given */
        int interval; /* fade steps, 0 = not given -> 1 */
        int thr_n;
        unsigned long long thr[DYN_GAMMA_MAX]; /* exp_thresh_htol */
        unsigned short (*table)[V5_ISP_GAMMA_NODES];
        unsigned char have[DYN_GAMMA_MAX];
        int n;
        int level;
        int failures;
        char engine;
        /* a fade: from `from` to table[level], step fade_i of interval */
        int fade_i;
        unsigned short *from;
        unsigned short *cur;
    } gamma;

    const char *err;   /* the vendor call the last failed write died in */
    unsigned last_map; /* hisi_iso_map of the ISO the ISO engine last wrote for */
    unsigned last_iso;
    unsigned long long last_exp;
    char engine; /* any of the three armed; release/acquire */
};

/*
 * The gamma tables are 20 KB for ten curves at 1025 u16 nodes, plus two
 * working curves. Allocated on first sight of a [dynamic_gamma] key
 * rather than with the rest, because a file without that section --
 * every file this backend has been pointed at so far bar one -- should
 * not pay for it.
 */
static bool dyn_gamma_alloc(struct hisi_dyn_set *d)
{
    if (d->gamma.table)
        return true;
    d->gamma.table = calloc(DYN_GAMMA_MAX, sizeof(*d->gamma.table));
    d->gamma.from = calloc(V5_ISP_GAMMA_NODES, sizeof(unsigned short));
    d->gamma.cur = calloc(V5_ISP_GAMMA_NODES, sizeof(unsigned short));
    if (d->gamma.table && d->gamma.from && d->gamma.cur)
        return true;
    HAL_LOG_WARN("isp tuning: [dynamic_gamma] out of memory; section ignored");
    return false;
}

static struct hisi_dyn_set *dyn_set(hisi_state_t *st)
{
    if (!st->dyn) {
        st->dyn = calloc(1, sizeof(*st->dyn));
        if (st->dyn) {
            st->dyn->drc.enable = -1;
            st->dyn->drc.last_lvl = -1;
            st->dyn->dehaze.last_lvl = -1;
            st->dyn->gamma.level = -1;
        }
    }
    return st->dyn;
}

static int dyn_write_drc(hisi_state_t *st, struct hisi_dyn_set *d, unsigned iso);

/*
 * hisi_dyn_drc_hold -- the drc_strength knob's grip on the DRC column.
 *
 * A pinned strength and a per-ISO column cannot both own the strength
 * field, so the knob holds the engine's DRC while it is pinned. Release
 * puts the engine back for the ISO it last wrote for, right away rather
 * than at the next step, so `auto` is visible at once. The other two
 * engines are not involved. A load re-arms the engine and the knob's
 * re-apply re-holds it, in that order, so the hold survives a reload.
 */
void hisi_dyn_drc_hold(hisi_state_t *st, bool hold)
{
    struct hisi_dyn_set *d = st->dyn;

    if (!d)
        return;
    if (hold) {
        if (d->drc.engine) {
            d->drc.engine = 0;
            HAL_LOG_INFO("drc: [dynamic_linear_drc] held while drc_strength is pinned");
        }
        return;
    }
    if (d->drc.engine || d->drc.n < 2 || d->drc.failures >= 3)
        return;
    d->drc.engine = 1;
    d->drc.last_lvl = -1;
    __atomic_store_n(&d->engine, 1, __ATOMIC_RELEASE);
    if (d->last_iso && dyn_write_drc(st, d, d->last_iso) == 0)
        HAL_LOG_INFO("drc: [dynamic_linear_drc] released; strength %ld for ISO %u",
                     d->drc.cur[DRC_STRENGTH], d->last_iso);
    else
        HAL_LOG_INFO("drc: [dynamic_linear_drc] released; next ISO step rewrites it");
}

void hisi_dyn_free(hisi_state_t *st)
{
    if (st->dyn) {
        free(st->dyn->gamma.table);
        free(st->dyn->gamma.from);
        free(st->dyn->gamma.cur);
    }
    free(st->dyn);
    st->dyn = NULL;
}

/* ================================================================
 * THE KEYS
 * ================================================================ */

/*
 * hisi_dyn_drc_curve -- does the tuning vary DRC with the light?
 *
 * The drc_strength knob's `auto` is this engine's column for whatever ISO
 * AE is reporting, and the control that offers it says exactly that:
 * follow the tuning's own curve. A file whose [dynamic_linear_drc] is
 * absent, off or one column wide has no curve, so there is nothing for
 * auto to mean and hal_knob.c publishes none.
 *
 * The knob's hold clears `engine` rather than the table, so this answers
 * the same whether or not the knob is currently pinned -- which it has
 * to, since pinned is exactly when the operator wants the way back.
 */
bool hisi_dyn_drc_curve(hisi_state_t *st)
{
    return st && st->dyn && st->dyn->drc.n >= 2;
}

/* The sixteen bcnr LUT rows, named detail_restore_lut_<n>. */
static int drc_lut_row(const char *key)
{
    long idx;
    char *end;

    if (strncasecmp(key, "detail_restore_lut_", 19) != 0)
        return -1;
    if (!isdigit((unsigned char)key[19]))
        return -1;
    idx = strtol(key + 19, &end, 10);
    if (*end || idx < 0 || idx >= V5_ISP_DRC_BCNR_NODES)
        return -1;
    return DRC_LUT_0 + (int)idx;
}

static bool dyn_key_drc(struct hisi_dyn_set *d, const char *key, const char *val)
{
    long tmp[DYN_COLS];
    int i, row;

    d->drc.seen = true;
    if (!strcasecmp(key, "enable")) {
        d->drc.enable = dyn_nums(val, tmp, 1) == 1 && tmp[0] ? 1 : 0;
        return true;
    }
    if (!strcasecmp(key, "iso_count")) {
        d->drc.cnt = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 0, DYN_COLS) : 0;
        return true;
    }
    if (!strcasecmp(key, "iso_level")) {
        int n = dyn_nums(val, tmp, DYN_COLS);

        for (i = 0; i < n; i++)
            d->drc.iso[i] = (unsigned)dyn_clamp(tmp[i], 0, 0x7FFFFFFF);
        d->drc.iso_n = n;
        return true;
    }

    row = drc_lut_row(key);
    if (row < 0) {
        for (i = 0; i < DRC_LUT_0; i++)
            if (drc_rows[i].key && !strcasecmp(key, drc_rows[i].key)) {
                row = i;
                break;
            }
    }
    if (row < 0)
        return false;

    {
        long lo = row >= DRC_LUT_0 ? 0 : drc_rows[row].lo;
        long hi = row >= DRC_LUT_0 ? 0xFF : drc_rows[row].hi;
        int n = dyn_nums(val, d->drc.v[row], DYN_COLS);
        int c;

        for (c = 0; c < n; c++)
            d->drc.v[row][c] = dyn_clamp(d->drc.v[row][c], lo, hi);
        d->drc.vn[row] = n;
    }
    return true;
}

static bool dyn_key_dehaze(struct hisi_dyn_set *d, const char *key, const char *val)
{
    long tmp[DYN_COLS];
    int i;

    d->dehaze.seen = true;
    if (!strcasecmp(key, "exp_thresh_cnt")) {
        d->dehaze.cnt = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 0, DYN_COLS) : 0;
        return true;
    }
    if (!strcasecmp(key, "exp_thresh_ltoh")) {
        int n = dyn_nums(val, tmp, DYN_COLS);

        for (i = 0; i < n; i++)
            d->dehaze.thr[i] = (unsigned long long)dyn_clamp(tmp[i], 0, 0x7FFFFFFFL);
        d->dehaze.thr_n = n;
        return true;
    }
    if (!strcasecmp(key, "manual_strength") || !strcasecmp(key, "auto_strength")) {
        int n = dyn_nums(val, d->dehaze.str, DYN_COLS);

        for (i = 0; i < n; i++)
            d->dehaze.str[i] = dyn_clamp(d->dehaze.str[i], 0, 255);
        d->dehaze.str_n = n;
        return true;
    }
    /*
     * The WDR half of the section. The vendor picks between
     * manual_strength and manual_strengther on the actual exposure ratio
     * read back from ss_mpi_isp_query_inner_state_info; in linear mode
     * the ratio is fixed at 0x40 and the threshold is never crossed, so
     * the row is read and reported rather than used.
     */
    if (!strcasecmp(key, "manual_strengther") || !strcasecmp(key, "wdr_ratio_threshold")) {
        if (!d->dehaze.wdr_row_noted) {
            d->dehaze.wdr_row_noted = true;
            HAL_LOG_DBG("isp tuning: [dynamic_dehaze] %s is the WDR row; linear mode uses "
                        "manual_strength",
                        key);
        }
        return true;
    }
    return false;
}

static bool dyn_key_gamma(struct hisi_dyn_set *d, const char *key, const char *val)
{
    long tmp[DYN_GAMMA_MAX];
    int i;

    d->gamma.seen = true;
    if (!strcasecmp(key, "total_num")) {
        d->gamma.total = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 0, DYN_GAMMA_MAX) : 0;
        return true;
    }
    if (!strcasecmp(key, "interval")) {
        d->gamma.interval = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 1, 100) : 0;
        return true;
    }
    /*
     * The bands. The vendor's own setter reads exp_thresh_htol with the
     * low-to-high selector, so that is the row this follows; ltoh is
     * accepted only as the fall-back for a file that carries just one of
     * the two, and never overwrites htol.
     */
    if (!strcasecmp(key, "exp_thresh_htol") || !strcasecmp(key, "exp_thresh_ltoh")) {
        bool htol = !strcasecmp(key, "exp_thresh_htol");
        int n;

        if (!htol && d->gamma.thr_n)
            return true;
        n = dyn_nums(val, tmp, DYN_GAMMA_MAX);
        for (i = 0; i < n; i++)
            d->gamma.thr[i] = (unsigned long long)dyn_clamp(tmp[i], 0, 0x7FFFFFFFL);
        d->gamma.thr_n = n;
        return true;
    }
    if (strncasecmp(key, "table_", 6) == 0 && isdigit((unsigned char)key[6])) {
        long idx = strtol(key + 6, NULL, 10);
        const char *s = val;
        int n = 0;

        if (idx < 0 || idx >= DYN_GAMMA_MAX) {
            HAL_LOG_WARN("isp tuning: [dynamic_gamma] %s: index out of 0..%d; ignored", key,
                         DYN_GAMMA_MAX - 1);
            return true;
        }
        if (!dyn_gamma_alloc(d))
            return true;
        while (*s && n < V5_ISP_GAMMA_NODES) {
            while (*s && !isdigit((unsigned char)*s))
                s++;
            if (!*s)
                break;
            d->gamma.table[idx][n++] =
                (unsigned short)dyn_clamp(strtol(s, (char **)&s, 10), 0, 4095);
        }
        /* Whole curve or none: applying flips the curve type to USER, under
         * which nodes past n -- whatever the last Get returned for another
         * curve -- would become live. */
        if (n != V5_ISP_GAMMA_NODES) {
            HAL_LOG_WARN("isp tuning: gamma %s has %d of %d nodes -- table skipped; a partial "
                         "user curve would run on a stale tail",
                         key, n, V5_ISP_GAMMA_NODES);
            d->gamma.have[idx] = 0;
        } else {
            d->gamma.have[idx] = 1;
        }
        return true;
    }
    return false;
}

/*
 * hisi_dyn_key -- one key of one of the three sections. Returns false for
 * a key it does not know, so the caller logs it the way hal_isp.c does.
 */
bool hisi_dyn_key(hisi_state_t *st, const char *sect, const char *key, const char *val)
{
    struct hisi_dyn_set *d = dyn_set(st);

    if (!d)
        return true; /* out of memory: swallow; apply will find nothing */
    if (!strcasecmp(sect, "dynamic_linear_drc"))
        return dyn_key_drc(d, key, val);
    if (!strcasecmp(sect, "dynamic_dehaze"))
        return dyn_key_dehaze(d, key, val);
    if (!strcasecmp(sect, "dynamic_gamma"))
        return dyn_key_gamma(d, key, val);
    return false;
}

/* ================================================================
 * THE WRITES
 * ================================================================ */

/*
 * One row's value at `v` on the axis. The vendor's rule exactly: the
 * bottom band and the top band take their column whole, and only the
 * bands between blend.
 */
static long dyn_col(unsigned long long v, int lvl, int n, const unsigned long long *thr,
                    const long *col, bool is_signed)
{
    if (lvl == 0 || lvl == n - 1)
        return col[lvl];
    if (is_signed)
        return dyn_lerp_signed((long long)v, (long long)thr[lvl - 1], col[lvl - 1],
                               (long long)thr[lvl], col[lvl]);
    return (long)hisi_iso_lerp(v, thr[lvl - 1], (unsigned long long)col[lvl - 1], thr[lvl],
                               (unsigned long long)col[lvl]);
}

static int dyn_write_drc(hisi_state_t *st, struct hisi_dyn_set *d, unsigned iso)
{
    v5_isp_drc_attr a;
    unsigned long long thr[DYN_COLS];
    long c[DRC_NF];
    int lvl, i, ret;

    d->err = "get/set_drc_attr";
    if (!st->tune.fnGetDrcAttr || !st->tune.fnSetDrcAttr)
        return -1;
    d->err = "get_drc_attr";
    ret = st->tune.fnGetDrcAttr(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    for (i = 0; i < d->drc.n; i++)
        thr[i] = d->drc.iso[i];
    lvl = dyn_level(iso, d->drc.n, thr);

    for (i = 0; i < DRC_NF; i++)
        c[i] = d->drc.vn[i] ? dyn_col(iso, lvl, d->drc.n, thr, d->drc.v[i], drc_row_signed(i)) : 0;

    if (d->drc.enable >= 0)
        a.enable = d->drc.enable;
#define PUT(row, field, T)                                                                         \
    do {                                                                                           \
        if (d->drc.vn[row])                                                                        \
            a.field = (T)c[row];                                                                   \
    } while (0)
    PUT(DRC_SPATIAL_FILTER_COEF, spatial_filter_coef, unsigned char);
    PUT(DRC_RANGE_FILTER_COEF, range_filter_coef, unsigned char);
    PUT(DRC_DETAIL_ADJUST_COEF, detail_adjust_coef, unsigned char);
    PUT(DRC_RIM_REDUCTION_STRENGTH, rim_reduction_strength, unsigned char);
    PUT(DRC_RIM_REDUCTION_THRESHOLD, rim_reduction_threshold, unsigned char);
    PUT(DRC_BRIGHT_GAIN_LIMIT, bright_gain_limit, unsigned char);
    PUT(DRC_BRIGHT_GAIN_LIMIT_STEP, bright_gain_limit_step, unsigned char);
    PUT(DRC_DARK_GAIN_LIMIT_LUMA, dark_gain_limit_luma, unsigned char);
    PUT(DRC_DARK_GAIN_LIMIT_CHROMA, dark_gain_limit_chroma, unsigned char);
    PUT(DRC_HIGH_SAT_COLOR_CTRL, high_saturation_color_ctrl, unsigned char);
    PUT(DRC_GLOBAL_COLOR_CTRL, global_color_ctrl, unsigned char);
    PUT(DRC_LM_BRIGHT_MAX, local_mixing_bright_param.max, unsigned char);
    PUT(DRC_LM_BRIGHT_MIN, local_mixing_bright_param.min, unsigned char);
    PUT(DRC_LM_BRIGHT_THRESHOLD, local_mixing_bright_param.threshold, unsigned char);
    PUT(DRC_LM_BRIGHT_SLOPE, local_mixing_bright_param.slope, signed char);
    PUT(DRC_LM_DARK_MAX, local_mixing_dark_param.max, unsigned char);
    PUT(DRC_LM_DARK_MIN, local_mixing_dark_param.min, unsigned char);
    PUT(DRC_LM_DARK_THRESHOLD, local_mixing_dark_param.threshold, unsigned char);
    PUT(DRC_LM_DARK_SLOPE, local_mixing_dark_param.slope, signed char);
    PUT(DRC_CURVE_BRIGHTNESS, auto_curve.brightness, unsigned char);
    PUT(DRC_CURVE_CONTRAST, auto_curve.contrast, unsigned char);
    PUT(DRC_CURVE_TOLERANCE, auto_curve.tolerance, unsigned char);
    PUT(DRC_BCNR_STRENGTH, bcnr_attr.strength, unsigned char);
    PUT(DRC_ASYMMETRY, asymmetry_curve.asymmetry, unsigned char);
    PUT(DRC_SECOND_POLE, asymmetry_curve.second_pole, unsigned char);
    PUT(DRC_COMPRESS, asymmetry_curve.compress, unsigned char);
    PUT(DRC_STRETCH, asymmetry_curve.stretch, unsigned char);
#undef PUT
    for (i = 0; i < V5_ISP_DRC_BCNR_NODES; i++)
        if (d->drc.vn[DRC_LUT_0 + i])
            a.bcnr_attr.detail_restore_lut[i] = (unsigned char)c[DRC_LUT_0 + i];

    /* The strength the driver reads is the one for its op type, as
     * scene_set_isp_drc_attr routes it. */
    if (d->drc.vn[DRC_STRENGTH]) {
        if (a.op_type == 0)
            a.auto_attr.strength = (unsigned short)c[DRC_STRENGTH];
        else
            a.manual_attr.strength = (unsigned short)c[DRC_STRENGTH];
    }

    d->err = "set_drc_attr";
    ret = st->tune.fnSetDrcAttr(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->drc.last_lvl && d->drc.engine)
            HAL_LOG_INFO("drc: ISO %u -> %u, now %s column %d (ISO %u); strength %ld", d->last_iso,
                         iso, lvl > 0 && iso < d->drc.iso[lvl] ? "below" : "on", lvl,
                         d->drc.iso[lvl], c[DRC_STRENGTH]);
        memcpy(d->drc.cur, c, sizeof(c));
        d->drc.last_lvl = lvl;
        d->drc.failures = 0;
    }
    return ret;
}

static int dyn_write_dehaze(hisi_state_t *st, struct hisi_dyn_set *d, unsigned long long exposure)
{
    v5_isp_dehaze_attr a;
    unsigned s;
    int lvl, ret;

    d->err = "get/set_dehaze_attr";
    if (!st->tune.fnGetDehazeAttr || !st->tune.fnSetDehazeAttr)
        return -1;
    d->err = "get_dehaze_attr";
    ret = st->tune.fnGetDehazeAttr(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    lvl = dyn_level(exposure, d->dehaze.n, d->dehaze.thr);
    s = (unsigned)dyn_col(exposure, lvl, d->dehaze.n, d->dehaze.thr, d->dehaze.str, false);

    /* Routed by op type, as ot_scene_set_dynamic_dehaze routes it. */
    if (a.op_type == 0)
        a.auto_strength = (unsigned char)s;
    else
        a.manual_strength = (unsigned char)s;

    d->err = "set_dehaze_attr";
    ret = st->tune.fnSetDehazeAttr(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->dehaze.last_lvl && d->dehaze.engine)
            HAL_LOG_INFO("dehaze: exposure %llu -> %llu, now column %d (<= %llu); strength %u",
                         d->last_exp, exposure, lvl, d->dehaze.thr[lvl], s);
        d->dehaze.cur = s;
        d->dehaze.last_lvl = lvl;
        d->dehaze.failures = 0;
    }
    return ret;
}

/* scene_time_filter: step `i` of `cnt` from a to b, landing on b. */
static unsigned short dyn_fade(unsigned short a, unsigned short b, int cnt, int i)
{
    unsigned long long t;

    if (cnt < 1)
        cnt = 1;
    if (a > b) {
        t = (((unsigned long long)(a - b) << 8) * (unsigned)(i + 1) / (unsigned)cnt) >> 8;
        return (unsigned short)(a > (unsigned)t + 1 ? a - (unsigned)t - 1 : 0);
    }
    if (a < b) {
        t = (((unsigned long long)(b - a) << 8) * (unsigned)(i + 1) / (unsigned)cnt) >> 8;
        return (unsigned short)(a + (unsigned)t + 1 > b ? b : a + (unsigned)t + 1);
    }
    return a;
}

/* Write `cur` as the user curve. */
static int dyn_write_gamma(hisi_state_t *st, struct hisi_dyn_set *d)
{
    v5_isp_gamma_attr *a;
    int ret;

    d->err = "get/set_gamma_attr";
    if (!st->tune.fnGetGammaAttr || !st->tune.fnSetGammaAttr)
        return -1;
    a = calloc(1, sizeof(*a)); /* 2 KB; encoder-thread stack again */
    if (!a)
        return -1;
    d->err = "get_gamma_attr";
    ret = st->tune.fnGetGammaAttr(HISI_VI_PIPE, a);
    if (ret == 0) {
        memcpy(a->table, d->gamma.cur, sizeof(a->table));
        a->enable = 1;
        a->curve_type = V5_ISP_GAMMA_CURVE_USER;
        d->err = "set_gamma_attr";
        ret = st->tune.fnSetGammaAttr(HISI_VI_PIPE, a);
    }
    free(a);
    if (ret == 0)
        d->gamma.failures = 0;
    return ret;
}

/* One fade step toward table[level]; the last step lands exactly on it. */
static int dyn_gamma_step(hisi_state_t *st, struct hisi_dyn_set *d)
{
    int n = d->gamma.interval > 0 ? d->gamma.interval : 1;
    int i = d->gamma.fade_i, k;

    if (i >= n - 1) {
        memcpy(d->gamma.cur, d->gamma.table[d->gamma.level],
               V5_ISP_GAMMA_NODES * sizeof(unsigned short));
        d->gamma.fade_i = 0;
    } else {
        for (k = 0; k < V5_ISP_GAMMA_NODES; k++)
            d->gamma.cur[k] = dyn_fade(d->gamma.from[k], d->gamma.table[d->gamma.level][k], n, i);
        d->gamma.fade_i = i + 1;
    }
    return dyn_write_gamma(st, d);
}

static void dyn_stop(struct hisi_dyn_set *d, const char *what, char *engine, int ret)
{
    HAL_LOG_WARN("%s: ss_mpi_isp_%s failed three times running (last 0x%x); leaving the last "
                 "value in place and stopping",
                 what, d->err, ret);
    __atomic_store_n(engine, 0, __ATOMIC_RELEASE);
}

/*
 * hisi_dyn_on_exposure -- the engines, given AE's numbers. What the tick
 * calls once it has them; the host test calls it directly.
 */
void hisi_dyn_on_exposure(hisi_state_t *st, unsigned iso, unsigned long long exposure)
{
    struct hisi_dyn_set *d = st->dyn;
    unsigned map;
    int ret;

    if (!d || !__atomic_load_n(&d->engine, __ATOMIC_ACQUIRE) || !iso)
        return;

    map = hisi_iso_map(iso);
    if (map != d->last_map && d->drc.engine) {
        ret = dyn_write_drc(st, d, iso);
        if (ret) {
            if (++d->drc.failures >= 3)
                dyn_stop(d, "drc", &d->drc.engine, ret);
        } else {
            HAL_LOG_DBG("drc: ISO %u -> %u, strength %ld", d->last_iso, iso,
                        d->drc.cur[DRC_STRENGTH]);
        }
    }
    if (map != d->last_map) {
        d->last_map = map;
        d->last_iso = iso;
    }

    /* The two exposure-axis engines. Exposure moves with every AE step,
     * so the band -- not the number -- is the reason to write. */
    if (d->dehaze.engine) {
        int lvl = dyn_level(exposure, d->dehaze.n, d->dehaze.thr);

        if (lvl != d->dehaze.last_lvl) {
            ret = dyn_write_dehaze(st, d, exposure);
            if (ret && ++d->dehaze.failures >= 3)
                dyn_stop(d, "dehaze", &d->dehaze.engine, ret);
        }
    }

    if (d->gamma.engine) {
        int lvl = dyn_level(exposure, d->gamma.n, d->gamma.thr);

        if (lvl != d->gamma.level) {
            /* Fade from wherever the curve is now, mid-fade included. */
            HAL_LOG_INFO("gamma: exposure %llu -> %llu, table %d -> %d, fading over %d steps",
                         d->last_exp, exposure, d->gamma.level, lvl,
                         d->gamma.interval > 0 ? d->gamma.interval : 1);
            memcpy(d->gamma.from, d->gamma.cur, V5_ISP_GAMMA_NODES * sizeof(unsigned short));
            d->gamma.level = lvl;
            d->gamma.fade_i = 0;
            ret = dyn_gamma_step(st, d);
        } else if (d->gamma.fade_i > 0) {
            ret = dyn_gamma_step(st, d);
        } else {
            ret = 0;
        }
        if (ret && ++d->gamma.failures >= 3) {
            d->gamma.fade_i = 0;
            dyn_stop(d, "gamma", &d->gamma.engine, ret);
        }
    }
    d->last_exp = exposure;

    if (!d->drc.engine && !d->dehaze.engine && !d->gamma.engine)
        __atomic_store_n(&d->engine, 0, __ATOMIC_RELEASE);
}

/* ================================================================
 * APPLY, AND THE TICK
 * ================================================================ */

static void dyn_note(char *note, size_t len, const char *what)
{
    size_t have = strlen(note);

    if (have + strlen(what) + 2 >= len)
        return;
    snprintf(note + have, len - have, "%s%s", have ? " " : "", what);
}

/* The load-time write failed: say which call, or that it was never there,
 * and note the section for the summary line. */
static void dyn_apply_fail(struct hisi_dyn_set *d, const char *sect, int ret, const char *keeps,
                           char *note, size_t note_len)
{
    char what[64];

    if (ret == -1) {
        HAL_LOG_WARN("isp tuning: [%s] ss_mpi_isp_%s unresolved -- %s", sect, d->err, keeps);
        snprintf(what, sizeof(what), "%s(unresolved)", sect);
    } else {
        HAL_LOG_WARN("isp tuning: [%s] ss_mpi_isp_%s failed: 0x%x -- %s", sect, d->err, ret, keeps);
        snprintf(what, sizeof(what), "%s(%s failed)", sect,
                 strstr(d->err, "get") == d->err ? "Get" : "Set");
    }
    dyn_note(note, note_len, what);
}

/* Validate [dynamic_linear_drc]; 0 columns means off. */
static int dyn_check_drc(struct hisi_dyn_set *d)
{
    int n = d->drc.iso_n, i, rows = 0;

    if (!d->drc.seen)
        return 0;
    if (n == 0) {
        HAL_LOG_WARN("isp tuning: [dynamic_linear_drc] no iso_level; section ignored");
        return 0;
    }
    if (d->drc.cnt > 0 && d->drc.cnt < n)
        n = d->drc.cnt;
    for (i = 1; i < n; i++) {
        if (d->drc.iso[i] <= d->drc.iso[i - 1]) {
            HAL_LOG_WARN("isp tuning: [dynamic_linear_drc] iso_level not ascending at entry %d (%u "
                         "after %u); truncating to %d columns",
                         i, d->drc.iso[i], d->drc.iso[i - 1], i);
            n = i;
            break;
        }
    }
    for (i = 0; i < DRC_NF; i++) {
        if (!d->drc.vn[i])
            continue;
        rows++;
        if (d->drc.vn[i] < n) {
            HAL_LOG_WARN("isp tuning: [dynamic_linear_drc] row %d has %d of %d columns; using %d",
                         i, d->drc.vn[i], n, d->drc.vn[i]);
            n = d->drc.vn[i];
        }
    }
    if (!rows && d->drc.enable < 0) {
        HAL_LOG_WARN("isp tuning: [dynamic_linear_drc] no rows; section ignored");
        return 0;
    }
    return n;
}

static int dyn_check_dehaze(struct hisi_dyn_set *d)
{
    int n, i;

    if (!d->dehaze.seen)
        return 0;
    n = d->dehaze.thr_n < d->dehaze.str_n ? d->dehaze.thr_n : d->dehaze.str_n;
    if (n == 0) {
        HAL_LOG_WARN("isp tuning: [dynamic_dehaze] exp_thresh_ltoh or manual_strength missing; "
                     "section ignored");
        return 0;
    }
    if (d->dehaze.thr_n != d->dehaze.str_n)
        HAL_LOG_WARN("isp tuning: [dynamic_dehaze] exp_thresh_ltoh has %d entries, "
                     "manual_strength %d; using %d",
                     d->dehaze.thr_n, d->dehaze.str_n, n);
    if (d->dehaze.cnt > 0 && d->dehaze.cnt != n)
        HAL_LOG_DBG("isp tuning: [dynamic_dehaze] exp_thresh_cnt %d, %d pairs present; using the "
                    "pairs",
                    d->dehaze.cnt, n);
    for (i = 1; i < n; i++) {
        if (d->dehaze.thr[i] <= d->dehaze.thr[i - 1]) {
            HAL_LOG_WARN("isp tuning: [dynamic_dehaze] exp_thresh_ltoh not ascending at entry %d; "
                         "truncating to %d columns",
                         i, i);
            n = i;
            break;
        }
    }
    return n;
}

static int dyn_check_gamma(struct hisi_dyn_set *d)
{
    int n, i;

    if (!d->gamma.seen || !d->gamma.table)
        return 0;
    n = d->gamma.total;
    if (n == 0) {
        /* No total_num: as many leading tables as are present. */
        while (n < DYN_GAMMA_MAX && d->gamma.have[n])
            n++;
    }
    for (i = 0; i < n; i++) {
        if (!d->gamma.have[i]) {
            if (i == 0)
                return 0; /* already warned at the key */
            HAL_LOG_WARN("isp tuning: [dynamic_gamma] total_num is %d but table_%d is missing or "
                         "short; using the %d before it",
                         n, i, i);
            n = i;
            break;
        }
    }
    /*
     * The vendor selects with count = total_num over the same threshold
     * row, so a file with N tables carries N thresholds and the last one
     * is never reached -- unlike gen4's dialect, where N tables took N-1.
     * Short rows lose tables rather than bands.
     */
    if (n > 1 && d->gamma.thr_n < n) {
        HAL_LOG_WARN("isp tuning: [dynamic_gamma] %d tables need %d exp_thresh_htol entries, %d "
                     "given; using %d tables",
                     n, n, d->gamma.thr_n, d->gamma.thr_n);
        n = d->gamma.thr_n;
    }
    for (i = 1; i < n; i++) {
        if (d->gamma.thr[i] <= d->gamma.thr[i - 1]) {
            HAL_LOG_WARN("isp tuning: [dynamic_gamma] exp_thresh_htol not ascending at entry %d; "
                         "using %d tables",
                         i, i);
            n = i;
            break;
        }
    }
    return n;
}

/*
 * hisi_dyn_apply -- validate the three sections, write each for the ISO
 * and exposure AE reports now (the first column without an AE to ask),
 * and arm the tick. Returns how many sections were written; *failed
 * counts those present that could not be, with `note` naming them for
 * the load summary.
 */
int hisi_dyn_apply(hisi_state_t *st, int *failed, char *note, size_t note_len)
{
    struct hisi_dyn_set *d = st->dyn;
    unsigned iso = 0;
    unsigned long long exposure = 0;
    bool have_ae;
    int applied = 0, ret;

    *failed = 0;
    note[0] = '\0';
    if (!d)
        return 0;

    d->drc.n = dyn_check_drc(d);
    d->dehaze.n = dyn_check_dehaze(d);
    d->gamma.n = dyn_check_gamma(d);
    if (!d->drc.n && !d->dehaze.n && !d->gamma.n) {
        if (d->drc.seen || d->dehaze.seen || d->gamma.seen) {
            *failed = 1;
            dyn_note(note, note_len, "dynamic_*(nothing usable)");
        }
        return 0;
    }

    have_ae = hisi_iso_query(st, &iso, &exposure);
    if (!have_ae) {
        iso = 0;
        exposure = 0;
    }

    if (d->drc.n) {
        ret = dyn_write_drc(st, d, iso ? iso : d->drc.iso[0]);
        if (ret) {
            dyn_apply_fail(d, "dynamic_linear_drc", ret, "DRC keeps the static values", note,
                           note_len);
            (*failed)++;
            d->drc.n = 0;
        } else {
            HAL_LOG_INFO("isp tuning: [dynamic_linear_drc] %d columns, ISO %u..%u; %s ISO %u, "
                         "strength %ld%s",
                         d->drc.n, d->drc.iso[0], d->drc.iso[d->drc.n - 1],
                         have_ae ? "AE at" : "no ISO query, first column at",
                         iso ? iso : d->drc.iso[0], d->drc.cur[DRC_STRENGTH],
                         have_ae ? "; tracking ISO" : "");
            applied++;
            d->drc.engine = have_ae && d->drc.n > 1;
        }
    }

    if (d->dehaze.n) {
        ret = dyn_write_dehaze(st, d, have_ae ? exposure : d->dehaze.thr[0]);
        if (ret) {
            dyn_apply_fail(d, "dynamic_dehaze", ret, "dehaze keeps the static values", note,
                           note_len);
            (*failed)++;
            d->dehaze.n = 0;
        } else {
            HAL_LOG_INFO("isp tuning: [dynamic_dehaze] %d columns on exposure, %llu..%llu; %s "
                         "%llu, strength %u%s",
                         d->dehaze.n, d->dehaze.thr[0], d->dehaze.thr[d->dehaze.n - 1],
                         have_ae ? "AE at" : "no AE query, first column at",
                         have_ae ? exposure : d->dehaze.thr[0], d->dehaze.cur,
                         have_ae ? "; tracking exposure" : "");
            applied++;
            d->dehaze.engine = have_ae && d->dehaze.n > 1;
        }
    }

    if (d->gamma.n) {
        int lvl = have_ae ? dyn_level(exposure, d->gamma.n, d->gamma.thr) : 0;

        /* Straight in, no fade: there is no curve of ours to fade from. */
        memcpy(d->gamma.cur, d->gamma.table[lvl], V5_ISP_GAMMA_NODES * sizeof(unsigned short));
        d->gamma.level = lvl;
        d->gamma.fade_i = 0;
        ret = dyn_write_gamma(st, d);
        if (ret) {
            dyn_apply_fail(d, "dynamic_gamma", ret, "gamma keeps the driver's curve", note,
                           note_len);
            (*failed)++;
            d->gamma.n = 0;
        } else {
            if (d->gamma.n > 1)
                HAL_LOG_INFO("isp tuning: [dynamic_gamma] %d tables; %s exposure %llu, table %d "
                             "written%s",
                             d->gamma.n, have_ae ? "AE at" : "no AE query, first at", exposure, lvl,
                             have_ae ? "; tracking exposure" : "");
            else
                HAL_LOG_INFO("isp tuning: [dynamic_gamma] one table, written");
            applied++;
            d->gamma.engine = have_ae && d->gamma.n > 1;
        }
    }

    if (iso) {
        d->last_map = hisi_iso_map(iso);
        d->last_iso = iso;
        d->last_exp = exposure;
    }
    if (d->drc.engine || d->dehaze.engine || d->gamma.engine)
        __atomic_store_n(&d->engine, 1, __ATOMIC_RELEASE);
    return applied;
}

static long long dyn_now_ns(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/*
 * hisi_dyn_tick -- from the encoder's frame hook, every frame, every
 * thread. Cheap until the period has passed; then one thread asks AE once
 * and hands the numbers to every armed engine, the 3DNR ladder included.
 */
void hisi_dyn_tick(hisi_state_t *st)
{
    struct hisi_dyn_set *d = st->dyn;
    bool dyn_on = d && __atomic_load_n(&d->engine, __ATOMIC_ACQUIRE);
    unsigned iso = 0;
    unsigned long long exposure = 0;
    long long now;

    if (!dyn_on)
        return;
    now = dyn_now_ns();
    if (now < st->iso_tick_ns)
        return;
    if (__atomic_test_and_set(&st->iso_busy, __ATOMIC_ACQ_REL))
        return;

    if (hisi_iso_query(st, &iso, &exposure))
        hisi_dyn_on_exposure(st, iso, exposure);
    /* A gamma fade in flight steps every 100 ms; otherwise once a second. */
    st->iso_tick_ns = now + ((d && d->gamma.fade_i > 0) ? 100000000LL : 1000000000LL);
    __atomic_clear(&st->iso_busy, __ATOMIC_RELEASE);
}
