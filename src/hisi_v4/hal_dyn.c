/*
 * hal_dyn.c -- the dynamic ISP sections, driven off AE's ISO once a second.
 *
 * WHAT THIS IS. The shipped /etc/sensors/iq/<sensor>.ini carries three
 * sections that are tables over an axis rather than values:
 *
 *   [dynamic_linear_drc]  seventeen DRC fields, one column per IsoLevel
 *   [dynamic_dehaze]      one dehaze strength per IsoThresh
 *   [dynamic_gamma]       TotalNum whole gamma curves, one per exposure band
 *
 * Majestic walks them at runtime; hal_isp.c used to apply their first
 * column once and leave it, which is the daylight picture at every ISO.
 * The night comparison of 2026-09-01 (docs/hisilicon.md) is what that
 * costs: with the 3DNR ladder tracking ISO, raptor still spent twice
 * majestic's bits at ISO 12000, at the encoder's QP ceiling, and the ISP
 * dump named the difference -- DRC strength 512 against majestic's 300.
 * The daylight DRC lifts shadows the night scene does not have, and the
 * encoder pays for the noise that lift brings up.
 *
 * WORSE THAN THE WRONG COLUMN. The 512 was not even the daylight column
 * (420). [static_drc] sets DRCOpType = 1 -- manual strength -- and the
 * static load wrote every Strength into the *auto* field, which the driver
 * does not read in manual mode. So the strength on the wire was the
 * driver's own default, at every ISO, and the whole Strength row was dead
 * letter. The SDK's scene_auto sample routes it by enOpType
 * (HI_SCENE_SetDynamicLinearDRC), and so does this.
 *
 * THE AXES. DRC and dehaze are on ISO, blended linearly in ISO between
 * the two columns either side (the sample's SCENE_Interpulate); past the
 * ends the end column holds. Gamma is on exposure -- ISO x integration
 * time / 100, the sample's SCENE_CalculateExp -- with the level picked by
 * gammaExpThreshHtoL the way the sample's SetDynamicVideoGamma picks it
 * (the LtoH row is read and reported, not used: the sample never uses it
 * for video, and with this file's numbers -- LtoH 3200 against HtoL
 * 400000 -- a rise/fall hysteresis on the pair would oscillate). A level
 * change fades the curve over Interval steps rather than cutting, as the
 * sample does with its SCENE_TimeFilter.
 *
 * ONE TICK FOR EVERYTHING. AE's ISO is one HI_MPI_ISP_QueryExposureInfo
 * call -- 5 KB of histogram for two fields -- so there is one clock here
 * (hisi_dyn_tick, off the encoder's frame hook, once a second, one thread
 * at a time) and it feeds the 3DNR ladder in hal_nrx.c as well as the
 * three sections above. Each engine keeps its own "moved a step" test and
 * its own failure count: three failed writes in a row stop that engine
 * and leave the others running. The tick shortens to 100 ms while a gamma
 * fade is in flight, so a ten-step fade takes a second.
 *
 * GET-MODIFY-SET, per write. Every write starts from a fresh Get, so the
 * static sections' values -- the tone-mapping curve, the dehaze LUT --
 * and anything else that touched the module survive underneath the
 * fields the column names.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* clock_gettime and CLOCK_MONOTONIC, under -std=c11. */
#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "hisi_state.h"
#include "v4_isp_tune.h"

#define DYN_COLS 16     /* IsoLevel / IsoThresh entries at most */
#define DYN_GAMMA_MAX 8 /* gamma curves at most; the file ships 3 */

/* ================================================================
 * THE ISO AXIS -- shared with hal_nrx.c
 * ================================================================
 *
 * hisi_iso_map is the sample's MapISO: an ISO to a step index, about six
 * steps per stop with ISO 100 at 0. The engines treat "moved a step" as
 * the reason to write, so a wobbling AE does not write every second.
 * hisi_iso_lerp is the sample's SCENE_Interpulate, the rounding linear
 * blend everything here interpolates with.
 */

unsigned hisi_iso_map(unsigned iso)
{
    unsigned i, j;

    if (iso < 100)
        iso = 100; /* the driver's own floor for u32ISO */
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

unsigned hisi_iso_lerp(unsigned mid, unsigned left, unsigned lv, unsigned right, unsigned rv)
{
    unsigned k;

    if (mid <= left)
        return lv;
    if (mid >= right)
        return rv;
    k = right - left;
    return ((right - mid) * lv + (mid - left) * rv + (k >> 1)) / k;
}

/*
 * hisi_iso_query -- what AE is at now. False when there is no way to ask
 * or the ask failed; `exposure` is the sample's ISO x time / 100, for the
 * gamma bands, and may be NULL.
 */
bool hisi_iso_query(hisi_state_t *st, unsigned *iso, unsigned long long *exposure)
{
    v4_isp_exp_info *info;
    bool ok = false;

    if (!st->tune.query_exp)
        return false;
    /* 5 KB, most of it a histogram; not a stack frame for an encoder thread. */
    info = calloc(1, sizeof(*info));
    if (!info)
        return false;
    if (st->tune.query_exp(HISI_VI_PIPE, info) == 0 && info->iso) {
        *iso = info->iso;
        if (exposure)
            *exposure = (unsigned long long)info->iso * info->exp_time / 100;
        ok = true;
    }
    free(info);
    return ok;
}

/* The sample's SCENE_GetLevelLtoH over `n` thresholds: the first band the
 * value is at or under, band `n` past the last threshold. */
static int dyn_level(unsigned long long v, int n, const unsigned long long *thr)
{
    int l;

    for (l = 0; l < n; l++)
        if (v <= thr[l])
            return l;
    return n;
}

/* Numbers out of a value: anything that is not a digit (or a minus before
 * one) separates. Returns how many landed. */
static int dyn_nums(const char *s, long *out, int max)
{
    int n = 0;

    while (*s && n < max) {
        while (*s && !isdigit((unsigned char)*s) &&
               !(*s == '-' && isdigit((unsigned char)s[1])))
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

/* [dynamic_linear_drc]'s rows, in the order the sample copies them. */
enum {
    DRC_LM_BRIGHT_MAX,
    DRC_LM_BRIGHT_MIN,
    DRC_LM_DARK_MAX,
    DRC_LM_DARK_MIN,
    DRC_BRIGHT_GAIN_LMT,
    DRC_BRIGHT_GAIN_LMT_STEP,
    DRC_DARK_GAIN_LMT_Y,
    DRC_DARK_GAIN_LMT_C,
    DRC_FLT_SCALE_COARSE,
    DRC_FLT_SCALE_FINE,
    DRC_CONTRAST_CONTROL,
    DRC_DETAIL_ADJUST_FACTOR,
    DRC_ASYMMETRY,
    DRC_SECOND_POLE,
    DRC_COMPRESS,
    DRC_STRETCH,
    DRC_STRENGTH,
    DRC_NF
};

static const struct {
    const char *key;
    long lo, hi; /* the field's range in ISP_DRC_ATTR_S */
} drc_rows[DRC_NF] = {
    {"LocalMixingBrightMax", 0, 0x80}, {"LocalMixingBrightMin", 0, 0x40},
    {"LocalMixingDarkMax", 0, 0x80},   {"LocalMixingDarkMin", 0, 0x40},
    {"BrightGainLmt", 0, 0xF},         {"BrightGainLmtStep", 0, 0xF},
    {"DarkGainLmtY", 0, 0x85},         {"DarkGainLmtC", 0, 0x85},
    {"FltScaleCoarse", 0, 0xF},        {"FltScaleFine", 0, 0xF},
    {"ContrastControl", 0, 0xF},       {"DetailAdjustFactor", -15, 15},
    {"Asymmetry", 0x1, 0x1E},          {"SecondPole", 0x96, 0xD2},
    {"Compress", 0x64, 0xC8},          {"Stretch", 0x1E, 0x3C},
    {"Strength", 0, 0x3FF},
};

struct hisi_dyn_set {
    struct {
        bool seen;
        int enable;    /* -1 = not given */
        int cnt;       /* IsoCnt, 0 = not given */
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
        int cnt; /* ExpThreshCnt, 0 = not given */
        int iso_n, str_n;
        unsigned iso[DYN_COLS];
        long str[DYN_COLS];
        int n;
        int last_lvl;
        int failures;
        char engine;
        unsigned cur;
    } dehaze;

    struct {
        bool seen;
        int total;    /* TotalNum, 0 = not given */
        int interval; /* fade steps, 0 = not given -> 1 */
        int thr_n, ltoh_n;
        unsigned long long thr[DYN_GAMMA_MAX]; /* gammaExpThreshHtoL */
        unsigned short table[DYN_GAMMA_MAX][V4_ISP_GAMMA_NODES];
        unsigned char have[DYN_GAMMA_MAX];
        int n;
        int level;
        int failures;
        char engine;
        /* a fade: from `from` to table[level], step fade_i of interval */
        int fade_i;
        unsigned short from[V4_ISP_GAMMA_NODES];
        unsigned short cur[V4_ISP_GAMMA_NODES];
    } gamma;

    const char *err;   /* the vendor call the last failed write died in */
    unsigned last_map; /* hisi_iso_map of the ISO the ISO engines last wrote for */
    unsigned last_iso;
    unsigned long long last_exp;
    char engine; /* any of the three armed; release/acquire */
};

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

void hisi_dyn_free(hisi_state_t *st)
{
    free(st->dyn);
    st->dyn = NULL;
}

/* ================================================================
 * THE KEYS
 * ================================================================ */

static bool dyn_key_drc(struct hisi_dyn_set *d, const char *key, const char *val)
{
    long tmp[DYN_COLS];
    int i;

    d->drc.seen = true;
    if (!strcasecmp(key, "Enable")) {
        d->drc.enable = dyn_nums(val, tmp, 1) == 1 && tmp[0] ? 1 : 0;
        return true;
    }
    if (!strcasecmp(key, "IsoCnt")) {
        d->drc.cnt = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 0, DYN_COLS) : 0;
        return true;
    }
    if (!strcasecmp(key, "IsoLevel")) {
        int n = dyn_nums(val, tmp, DYN_COLS);

        for (i = 0; i < n; i++)
            d->drc.iso[i] = (unsigned)dyn_clamp(tmp[i], 0, 0x7FFFFFFF);
        d->drc.iso_n = n;
        return true;
    }
    for (i = 0; i < DRC_NF; i++) {
        if (!strcasecmp(key, drc_rows[i].key)) {
            int n = dyn_nums(val, d->drc.v[i], DYN_COLS);
            int c;

            for (c = 0; c < n; c++)
                d->drc.v[i][c] = dyn_clamp(d->drc.v[i][c], drc_rows[i].lo, drc_rows[i].hi);
            d->drc.vn[i] = n;
            return true;
        }
    }
    return false;
}

static bool dyn_key_dehaze(struct hisi_dyn_set *d, const char *key, const char *val)
{
    long tmp[DYN_COLS];
    int i;

    d->dehaze.seen = true;
    if (!strcasecmp(key, "ExpThreshCnt")) {
        d->dehaze.cnt = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 0, DYN_COLS) : 0;
        return true;
    }
    if (!strcasecmp(key, "IsoThresh")) {
        int n = dyn_nums(val, tmp, DYN_COLS);

        for (i = 0; i < n; i++)
            d->dehaze.iso[i] = (unsigned)dyn_clamp(tmp[i], 0, 0x7FFFFFFF);
        d->dehaze.iso_n = n;
        return true;
    }
    /* Majestic's name and the SDK sample's for the same row. */
    if (!strcasecmp(key, "AutoDehazeStr") || !strcasecmp(key, "ManualDehazeStr")) {
        int n = dyn_nums(val, d->dehaze.str, DYN_COLS);

        for (i = 0; i < n; i++)
            d->dehaze.str[i] = dyn_clamp(d->dehaze.str[i], 0, 255);
        d->dehaze.str_n = n;
        return true;
    }
    return false;
}

static bool dyn_key_gamma(struct hisi_dyn_set *d, const char *key, const char *val)
{
    long tmp[DYN_GAMMA_MAX];
    int i;

    d->gamma.seen = true;
    if (!strcasecmp(key, "TotalNum")) {
        d->gamma.total = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 0, DYN_GAMMA_MAX) : 0;
        return true;
    }
    if (!strcasecmp(key, "Interval")) {
        d->gamma.interval = dyn_nums(val, tmp, 1) == 1 ? (int)dyn_clamp(tmp[0], 1, 100) : 0;
        return true;
    }
    if (!strcasecmp(key, "gammaExpThreshHtoL") || !strcasecmp(key, "ExpThreshHtoL")) {
        int n = dyn_nums(val, tmp, DYN_GAMMA_MAX);

        for (i = 0; i < n; i++)
            d->gamma.thr[i] = (unsigned long long)dyn_clamp(tmp[i], 0, 0x7FFFFFFFL);
        d->gamma.thr_n = n;
        return true;
    }
    if (!strcasecmp(key, "gammaExpThreshLtoH") || !strcasecmp(key, "ExpThreshLtoH")) {
        d->gamma.ltoh_n = dyn_nums(val, tmp, DYN_GAMMA_MAX);
        return true; /* read for the record; see the header */
    }
    if (strncasecmp(key, "Table_", 6) == 0 && isdigit((unsigned char)key[6])) {
        long idx = strtol(key + 6, NULL, 10);
        const char *s = val;
        int n = 0;

        if (idx < 0 || idx >= DYN_GAMMA_MAX) {
            HAL_LOG_WARN("isp tuning: [dynamic_gamma] %s: index out of 0..%d; ignored", key,
                         DYN_GAMMA_MAX - 1);
            return true;
        }
        while (*s && n < V4_ISP_GAMMA_NODES) {
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
        if (n != V4_ISP_GAMMA_NODES) {
            HAL_LOG_WARN("isp tuning: gamma %s has %d of %d nodes -- module skipped; a partial "
                         "user curve would run on a stale tail",
                         key, n, V4_ISP_GAMMA_NODES);
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

/* A column blend on the ISO axis, the end columns holding past the ends.
 * `lvl` is the first column whose ISO is at or above this one. The
 * sample's linear-DRC setter takes the top column as soon as the ISO is
 * in the top band; its 3DNR setter blends there like anywhere else, and
 * that is what this does -- the same rule as hal_nrx.c's nrx_select. */
static long dyn_col(unsigned iso, int lvl, int n, const unsigned *thr, const long *v, long lo)
{
    if (lvl == 0 || iso >= thr[n - 1])
        return v[lvl];
    /* Offset into unsigned for the one signed row (DetailAdjustFactor). */
    return (long)hisi_iso_lerp(iso, thr[lvl - 1], (unsigned)(v[lvl - 1] - lo), thr[lvl],
                               (unsigned)(v[lvl] - lo)) +
           lo;
}

static int dyn_write_drc(hisi_state_t *st, struct hisi_dyn_set *d, unsigned iso)
{
    v4_isp_drc_attr a;
    long c[DRC_NF];
    int lvl, i, ret;

    d->err = "Get/SetDRCAttr";
    if (!st->tune.get_drc || !st->tune.set_drc)
        return -1;
    d->err = "GetDRCAttr";
    ret = st->tune.get_drc(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    for (lvl = 0; lvl < d->drc.n; lvl++)
        if (iso <= d->drc.iso[lvl])
            break;
    if (lvl == d->drc.n)
        lvl = d->drc.n - 1;

    for (i = 0; i < DRC_NF; i++)
        c[i] = d->drc.vn[i] ? dyn_col(iso, lvl, d->drc.n, d->drc.iso, d->drc.v[i], drc_rows[i].lo)
                            : -1;

    if (d->drc.enable >= 0)
        a.enable = d->drc.enable;
#define PUT(row, field, T)                                                                         \
    do {                                                                                           \
        if (d->drc.vn[row])                                                                        \
            a.field = (T)c[row];                                                                   \
    } while (0)
    PUT(DRC_LM_BRIGHT_MAX, local_mixing_bright_max, unsigned char);
    PUT(DRC_LM_BRIGHT_MIN, local_mixing_bright_min, unsigned char);
    PUT(DRC_LM_DARK_MAX, local_mixing_dark_max, unsigned char);
    PUT(DRC_LM_DARK_MIN, local_mixing_dark_min, unsigned char);
    PUT(DRC_BRIGHT_GAIN_LMT, bright_gain_lmt, unsigned char);
    PUT(DRC_BRIGHT_GAIN_LMT_STEP, bright_gain_lmt_step, unsigned char);
    PUT(DRC_DARK_GAIN_LMT_Y, dark_gain_lmt_y, unsigned char);
    PUT(DRC_DARK_GAIN_LMT_C, dark_gain_lmt_c, unsigned char);
    PUT(DRC_FLT_SCALE_COARSE, flt_scale_coarse, unsigned char);
    PUT(DRC_FLT_SCALE_FINE, flt_scale_fine, unsigned char);
    PUT(DRC_CONTRAST_CONTROL, contrast_control, unsigned char);
    PUT(DRC_DETAIL_ADJUST_FACTOR, detail_adjust_factor, signed char);
    PUT(DRC_ASYMMETRY, asym.asymmetry, unsigned char);
    PUT(DRC_SECOND_POLE, asym.second_pole, unsigned char);
    PUT(DRC_COMPRESS, asym.compress, unsigned char);
    PUT(DRC_STRETCH, asym.stretch, unsigned char);
#undef PUT
    /* The strength the driver reads is the one for its op type -- manual
     * (the shipped file's DRCOpType = 1) or auto. See the header. */
    if (d->drc.vn[DRC_STRENGTH]) {
        if (a.op_type == 0)
            a.auto_strength = (unsigned short)c[DRC_STRENGTH];
        else
            a.manual_strength = (unsigned short)c[DRC_STRENGTH];
    }

    d->err = "SetDRCAttr";
    ret = st->tune.set_drc(HISI_VI_PIPE, &a);
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

static int dyn_write_dehaze(hisi_state_t *st, struct hisi_dyn_set *d, unsigned iso)
{
    v4_isp_dehaze_attr a;
    unsigned s;
    int lvl, ret;

    d->err = "Get/SetDehazeAttr";
    if (!st->tune.get_dehaze || !st->tune.set_dehaze)
        return -1;
    d->err = "GetDehazeAttr";
    ret = st->tune.get_dehaze(HISI_VI_PIPE, &a);
    if (ret)
        return ret;

    for (lvl = 0; lvl < d->dehaze.n; lvl++)
        if (iso <= d->dehaze.iso[lvl])
            break;
    if (lvl == d->dehaze.n)
        lvl = d->dehaze.n - 1;
    s = (unsigned)dyn_col(iso, lvl, d->dehaze.n, d->dehaze.iso, d->dehaze.str, 0);

    /* Routed by op type, as the sample's SetDynamicDehaze routes it. */
    if (a.op_type == 0)
        a.auto_strength = (unsigned char)s;
    else
        a.manual_strength = (unsigned char)s;

    d->err = "SetDehazeAttr";
    ret = st->tune.set_dehaze(HISI_VI_PIPE, &a);
    if (ret == 0) {
        if (lvl != d->dehaze.last_lvl && d->dehaze.engine)
            HAL_LOG_INFO("dehaze: ISO %u -> %u, now %s column %d (ISO %u); strength %u",
                         d->last_iso, iso, lvl > 0 && iso < d->dehaze.iso[lvl] ? "below" : "on",
                         lvl, d->dehaze.iso[lvl], s);
        d->dehaze.cur = s;
        d->dehaze.last_lvl = lvl;
        d->dehaze.failures = 0;
    }
    return ret;
}

/* The sample's SCENE_TimeFilter: step `i` of `cnt` from a to b. */
static unsigned short dyn_fade(unsigned short a, unsigned short b, int cnt, int i)
{
    unsigned long long t;

    if (a > b) {
        t = ((unsigned long long)(a - b) << 8) * (unsigned)(i + 1) / (unsigned)cnt >> 8;
        return (unsigned short)(a - t);
    }
    t = ((unsigned long long)(b - a) << 8) * (unsigned)(i + 1) / (unsigned)cnt >> 8;
    return (unsigned short)(a + t);
}

/* Write `cur` as the user curve. */
static int dyn_write_gamma(hisi_state_t *st, struct hisi_dyn_set *d)
{
    v4_isp_gamma_attr *a;
    int ret;

    d->err = "Get/SetGammaAttr";
    if (!st->tune.get_gamma || !st->tune.set_gamma)
        return -1;
    a = calloc(1, sizeof(*a)); /* 2 KB; encoder-thread stack again */
    if (!a)
        return -1;
    d->err = "GetGammaAttr";
    ret = st->tune.get_gamma(HISI_VI_PIPE, a);
    if (ret == 0) {
        memcpy(a->table, d->gamma.cur, sizeof(a->table));
        a->enable = 1;
        a->curve_type = V4_ISP_GAMMA_CURVE_USER;
        d->err = "SetGammaAttr";
        ret = st->tune.set_gamma(HISI_VI_PIPE, a);
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
        memcpy(d->gamma.cur, d->gamma.table[d->gamma.level], sizeof(d->gamma.cur));
        d->gamma.fade_i = 0;
    } else {
        for (k = 0; k < V4_ISP_GAMMA_NODES; k++)
            d->gamma.cur[k] = dyn_fade(d->gamma.from[k], d->gamma.table[d->gamma.level][k], n, i);
        d->gamma.fade_i = i + 1;
    }
    return dyn_write_gamma(st, d);
}

static void dyn_stop(struct hisi_dyn_set *d, const char *what, char *engine, int ret)
{
    HAL_LOG_WARN("%s: HI_MPI_ISP_%s failed three times running (last 0x%x); leaving the last "
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
    if (map != d->last_map) {
        if (d->drc.engine) {
            ret = dyn_write_drc(st, d, iso);
            if (ret) {
                if (++d->drc.failures >= 3)
                    dyn_stop(d, "drc", &d->drc.engine, ret);
            } else {
                HAL_LOG_DBG("drc: ISO %u -> %u, strength %ld", d->last_iso, iso,
                            d->drc.cur[DRC_STRENGTH]);
            }
        }
        if (d->dehaze.engine) {
            ret = dyn_write_dehaze(st, d, iso);
            if (ret) {
                if (++d->dehaze.failures >= 3)
                    dyn_stop(d, "dehaze", &d->dehaze.engine, ret);
            } else {
                HAL_LOG_DBG("dehaze: ISO %u -> %u, strength %u", d->last_iso, iso, d->dehaze.cur);
            }
        }
        d->last_map = map;
        d->last_iso = iso;
    }

    if (d->gamma.engine) {
        int lvl = dyn_level(exposure, d->gamma.n - 1, d->gamma.thr);

        if (lvl != d->gamma.level) {
            /* Fade from wherever the curve is now, mid-fade included. */
            HAL_LOG_INFO("gamma: exposure %llu -> %llu, table %d -> %d, fading over %d steps",
                         d->last_exp, exposure, d->gamma.level, lvl,
                         d->gamma.interval > 0 ? d->gamma.interval : 1);
            memcpy(d->gamma.from, d->gamma.cur, sizeof(d->gamma.from));
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
        HAL_LOG_WARN("isp tuning: [%s] %s unresolved -- %s", sect, d->err, keeps);
        snprintf(what, sizeof(what), "%s(unresolved)", sect);
    } else {
        HAL_LOG_WARN("isp tuning: [%s] %s failed: 0x%x -- %s", sect, d->err, ret, keeps);
        snprintf(what, sizeof(what), "%s(%s failed)", sect, d->err[0] == 'G' ? "Get" : "Set");
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
        HAL_LOG_WARN("isp tuning: [dynamic_linear_drc] no IsoLevel; section ignored");
        return 0;
    }
    if (d->drc.cnt > 0 && d->drc.cnt < n)
        n = d->drc.cnt;
    for (i = 1; i < n; i++) {
        if (d->drc.iso[i] <= d->drc.iso[i - 1]) {
            HAL_LOG_WARN("isp tuning: [dynamic_linear_drc] IsoLevel not ascending at entry %d (%u "
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
            HAL_LOG_WARN("isp tuning: [dynamic_linear_drc] %s has %d of %d columns; using %d",
                         drc_rows[i].key, d->drc.vn[i], n, d->drc.vn[i]);
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
    n = d->dehaze.iso_n < d->dehaze.str_n ? d->dehaze.iso_n : d->dehaze.str_n;
    if (n == 0) {
        HAL_LOG_WARN("isp tuning: [dynamic_dehaze] IsoThresh or AutoDehazeStr missing; section "
                     "ignored");
        return 0;
    }
    if (d->dehaze.iso_n != d->dehaze.str_n)
        HAL_LOG_WARN("isp tuning: [dynamic_dehaze] IsoThresh has %d entries, AutoDehazeStr %d; "
                     "using %d",
                     d->dehaze.iso_n, d->dehaze.str_n, n);
    /* The shipped file says ExpThreshCnt = 8 over nine pairs. The pairs
     * are what the tuner wrote; the count is noted, not obeyed. */
    if (d->dehaze.cnt > 0 && d->dehaze.cnt != n)
        HAL_LOG_DBG("isp tuning: [dynamic_dehaze] ExpThreshCnt %d, %d pairs present; using the "
                    "pairs",
                    d->dehaze.cnt, n);
    for (i = 1; i < n; i++) {
        if (d->dehaze.iso[i] <= d->dehaze.iso[i - 1]) {
            HAL_LOG_WARN("isp tuning: [dynamic_dehaze] IsoThresh not ascending at entry %d (%u "
                         "after %u); truncating to %d columns",
                         i, d->dehaze.iso[i], d->dehaze.iso[i - 1], i);
            n = i;
            break;
        }
    }
    return n;
}

static int dyn_check_gamma(struct hisi_dyn_set *d)
{
    int n, i;

    if (!d->gamma.seen)
        return 0;
    n = d->gamma.total;
    if (n == 0) {
        /* No TotalNum: as many leading tables as are present. */
        while (n < DYN_GAMMA_MAX && d->gamma.have[n])
            n++;
    }
    for (i = 0; i < n; i++) {
        if (!d->gamma.have[i]) {
            if (i == 0)
                return 0; /* already warned at the key */
            HAL_LOG_WARN("isp tuning: [dynamic_gamma] TotalNum is %d but Table_%d is missing or "
                         "short; using the %d before it",
                         n, i, i);
            n = i;
            break;
        }
    }
    if (n > 1 && d->gamma.thr_n < n - 1) {
        HAL_LOG_WARN("isp tuning: [dynamic_gamma] %d tables need %d gammaExpThreshHtoL entries, "
                     "%d given; using %d tables",
                     n, n - 1, d->gamma.thr_n, d->gamma.thr_n + 1);
        n = d->gamma.thr_n + 1;
    }
    for (i = 1; i < n - 1; i++) {
        if (d->gamma.thr[i] <= d->gamma.thr[i - 1]) {
            HAL_LOG_WARN("isp tuning: [dynamic_gamma] gammaExpThreshHtoL not ascending at entry "
                         "%d; using %d tables",
                         i, i + 1);
            n = i + 1;
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
        iso = 0; /* the first column, below */
        exposure = 0;
    }

    if (d->drc.n) {
        ret = dyn_write_drc(st, d, iso ? iso : d->drc.iso[0]);
        if (ret) {
            dyn_apply_fail(d, "dynamic_linear_drc", ret, "DRC keeps the static values", note, note_len);
            (*failed)++;
            d->drc.n = 0;
        } else {
            HAL_LOG_INFO("isp tuning: [dynamic_linear_drc] %d columns, ISO %u..%u; %s ISO %u, "
                         "strength %ld%s",
                         d->drc.n, d->drc.iso[0], d->drc.iso[d->drc.n - 1],
                         have_ae ? "AE at" : "no ISO query, first column at", iso ? iso : d->drc.iso[0],
                         d->drc.cur[DRC_STRENGTH], have_ae ? "; tracking ISO" : "");
            applied++;
            d->drc.engine = have_ae && d->drc.n > 1;
        }
    }

    if (d->dehaze.n) {
        ret = dyn_write_dehaze(st, d, iso ? iso : d->dehaze.iso[0]);
        if (ret) {
            dyn_apply_fail(d, "dynamic_dehaze", ret, "dehaze keeps the static values", note, note_len);
            (*failed)++;
            d->dehaze.n = 0;
        } else {
            HAL_LOG_INFO("isp tuning: [dynamic_dehaze] %d columns, ISO %u..%u; %s ISO %u, "
                         "strength %u%s",
                         d->dehaze.n, d->dehaze.iso[0], d->dehaze.iso[d->dehaze.n - 1],
                         have_ae ? "AE at" : "no ISO query, first column at",
                         iso ? iso : d->dehaze.iso[0], d->dehaze.cur, have_ae ? "; tracking ISO" : "");
            applied++;
            d->dehaze.engine = have_ae && d->dehaze.n > 1;
        }
    }

    if (d->gamma.n) {
        int lvl = have_ae ? dyn_level(exposure, d->gamma.n - 1, d->gamma.thr) : 0;

        /* Straight in, no fade: there is no curve of ours to fade from. */
        memcpy(d->gamma.cur, d->gamma.table[lvl], sizeof(d->gamma.cur));
        d->gamma.level = lvl;
        d->gamma.fade_i = 0;
        ret = dyn_write_gamma(st, d);
        if (ret) {
            dyn_apply_fail(d, "dynamic_gamma", ret, "gamma keeps the driver's curve", note, note_len);
            (*failed)++;
            d->gamma.n = 0;
        } else {
            if (d->gamma.n > 1)
                HAL_LOG_INFO("isp tuning: [dynamic_gamma] %d tables; %s exposure %llu, table %d "
                             "written%s",
                             d->gamma.n, have_ae ? "AE at" : "no ISO query, first at", exposure,
                             lvl, have_ae ? "; tracking exposure" : "");
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
    bool nrx_on = hisi_nrx_armed(st);
    unsigned iso = 0;
    unsigned long long exposure = 0;
    long long now;

    if (!dyn_on && !nrx_on)
        return;
    now = dyn_now_ns();
    if (now < st->iso_tick_ns)
        return;
    if (__atomic_test_and_set(&st->iso_busy, __ATOMIC_ACQ_REL))
        return;

    if (hisi_iso_query(st, &iso, &exposure)) {
        if (nrx_on)
            hisi_nrx_on_iso(st, iso);
        if (dyn_on)
            hisi_dyn_on_exposure(st, iso, exposure);
    }
    /* A gamma fade in flight steps every 100 ms; otherwise once a second. */
    st->iso_tick_ns = now + ((d && d->gamma.fade_i > 0) ? 100000000LL : 1000000000LL);
    __atomic_clear(&st->iso_busy, __ATOMIC_RELEASE);
}
