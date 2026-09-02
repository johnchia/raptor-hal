/*
 * hisi_v4/hal_isp.c -- ISP tuning load for HiMPP gen4 (Hi3516EV200/EV300)
 *
 * OP COVERAGE
 *
 * Implemented: isp_get_sensor_attr (moved here from hal_framesource.c, as
 * the Phase 2 notes promised), plus the internal tuning loader below --
 * which is Phase 3's entire scope. Every knob op that SigmaStar's table
 * mirrors (brightness, contrast, saturation, sharpness, sinter/temper
 * strength, ae_comp, defog, antiflicker, hflip/vflip, the getters) is
 * deliberately absent and stays NULL in the vtable: none of them matter
 * until there is a baseline image to adjust from, and RSS_HAL_CALL turns
 * their absence into RSS_ERR_NOTSUP, which is the truth.
 *
 * WHAT THE TUNING LOADER DOES
 *
 * With majestic replaced, nothing on the board applies IQ: the ISP runs on
 * whatever defaults libsns_*.so and the algorithm libraries carry, and the
 * shipped /etc/sensors/iq/<sensor>.ini sits unread. This file reads it --
 * OpenIPC's text tuning dialect, section by section -- and hands each
 * section to the matching HI_MPI_ISP_Set* through a get-modify-set: fetch
 * the module's whole attribute struct, patch exactly the fields the INI
 * names, write it back. Fields the INI does not mention keep the running
 * defaults, which is what makes a partial file safe to apply.
 *
 * Sections applied: static_ae, static_aerouteex, static_aeweight,
 * static_ldci, static_drc, static_nr, static_dehaze, static_sharpen,
 * static_dpc, and -- statically, at their lowest-ISO column --
 * dynamic_dehaze, dynamic_linear_drc and dynamic_gamma (Table_0). The
 * dynamic sections describe per-ISO interpolation majestic performs at
 * runtime; applying the daylight column gives the daylight baseline and
 * defers the interpolation engine, which is recorded in the plan rather
 * than hidden here.
 *
 * Sections skipped, and why:
 *   (static_3dnr is not skipped any more: it is the VPSS NRX X-param
 *   text, not an ISP module, and hal_nrx.c owns it -- the parser, the
 *   per-ISO rung selection and the HI_MPI_VPSS_SetGrpNRXParam write. The
 *   dispatch below hands its keys over and the apply loop calls it last.)
 *   ir_*           the night-mode mirror of the whole set. Day/night
 *                  switching is not this backend's to drive yet.
 *   all_param, dynamic_ae/fps/nr/...
 *                  majestic's runtime engines (ISO thresholds, fps
 *                  ladders). No static application exists.
 *
 * WHEN IT RUNS
 *
 * The file is resolved at bring-up (hisi_isp_resolve_iq) and applied on
 * the first encoded frame (hisi_isp_note_frame), following the
 * infinity6c shape: by the first frame the ISP is demonstrably running
 * and registered, every Get returns live state, and the one-time cost
 * lands on one encoder thread instead of in the init path. The latch is
 * atomic because rvd runs an encoder thread per stream.
 *
 * Selection: $RSS_ISP_TUNING if set (explicit wins outright -- an
 * unreadable explicit path is a warning and an untuned picture, not a
 * silent fallback), else /etc/sensors/iq/<sensor-stem>.ini, else untuned
 * with one WARN naming the path that was tried.
 *
 * THE PARSER IS LOCAL, ON PURPOSE
 *
 * hisi_sensor.c's INI reader holds 128 entries of 80 bytes each, which is
 * right for the sensor mode files and hopeless for this dialect: the IQ
 * file carries backslash-continued tables of several KB per value (the
 * 1025-node gamma LUT; the shipped imx335.ini's longest assembled value
 * is 6034 bytes, see HISI_IQ_VAL_MAX). The reader here streams entry by
 * entry instead of building a table, and the value scanner treats
 * anything that is not a number as a separator, which quietly handles the
 * dialect's quoting ("64, 72"), bare lists, trailing commas and mixed
 * whitespace alike.
 *
 * WHOLE TABLES ARE ALL-OR-NOTHING
 *
 * Get-modify-set makes a partial *file* safe -- a field the INI never
 * names keeps the running default. It does not make a partial *table*
 * safe: a curve or LUT is one field, and filling its first n entries from
 * the INI leaves the tail on whatever the Get returned, which is a
 * different curve. Worse for gamma, where applying also flips
 * enGammaCurveType to USER_DEFINE, so nodes past n are read back under a
 * curve type they were never sampled for. So the three whole-table values
 * -- gamma Table_0 (1025), DRCToneMappingValue (200) and DehazeLut (256)
 * -- are applied only at their exact node count, and a short or truncated
 * one is dropped with a WARN naming the count.
 *
 * The per-ISO arrays (the ISP_AUTO_ISO_STRENGTH_NUM=16 columns in NR,
 * LDCI, sharpen and DPC) are deliberately *not* under that rule: those are
 * sixteen independent per-gain scalars rather than one curve, so a short
 * list leaves the higher-ISO columns on the running default, which is the
 * ordinary partial-file contract. Same for the AE route, whose used length
 * is TotalNum rather than the array bound.
 *
 * A malformed section header -- a '[' with no ']' -- is not treated as
 * more of the previous section, which would silently file its keys under
 * the wrong module. It clears the current section instead, so its keys
 * apply to nothing until the next well-formed header.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"
#include "v4_isp_tune.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 * SENSOR GEOMETRY
 * ================================================================ */

/*
 * hal_isp_get_sensor_attr -- the sensor's output size.
 *
 * Answers from the mode INI rather than from HI_MPI_ISP_GetPubAttr: rvd
 * asks before the pipeline is up, and the INI is where the number came
 * from in the first place.
 */
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st)
        return RSS_ERR_INVAL;
    if (!width || !height)
        return RSS_ERR_INVAL;
    if (!st->mode.dev_rect.width || !st->mode.dev_rect.height)
        return RSS_ERR_NOTSUP;

    *width = st->mode.dev_rect.width;
    *height = st->mode.dev_rect.height;
    return RSS_OK;
}

/* ================================================================
 * IQ FILE RESOLUTION
 * ================================================================ */

/*
 * hisi_isp_resolve_iq -- settle which tuning file applies, or none.
 *
 * Called once from hal_init after the video pipeline is up (the sensor
 * name is settled by then). Only the path is decided here; the load waits
 * for the first frame. Missing tuning is a degraded picture, not a
 * failure, so this never fails the pipeline.
 */
void hisi_isp_resolve_iq(hisi_state_t *st)
{
    const char *env = getenv("RSS_ISP_TUNING");
    char stem[32];
    unsigned int j;
    FILE *f;

    st->iq_file[0] = '\0';

    if (env && *env) {
        if ((f = fopen(env, "r"))) {
            fclose(f);
            snprintf(st->iq_file, sizeof(st->iq_file), "%s", env);
            HAL_LOG_INFO("isp: tuning is %s (RSS_ISP_TUNING; loads on the first frame)",
                         st->iq_file);
        } else {
            HAL_LOG_WARN("isp: RSS_ISP_TUNING=%s is unreadable -- running untuned "
                         "(explicit wins; no fallback to the sensor default)",
                         env);
        }
        return;
    }

    /* Lowercase up to the first separator: "imx335_i2c_4M" -> "imx335". */
    for (j = 0; j + 1 < sizeof(stem) && st->sensor_name[j]; j++) {
        char c = st->sensor_name[j];
        if (c == '_' || c == ' ' || c == '-')
            break;
        stem[j] = (char)tolower((unsigned char)c);
    }
    stem[j] = '\0';
    if (!stem[0])
        return;

    snprintf(st->iq_file, sizeof(st->iq_file), "/etc/sensors/iq/%s.ini", stem);
    if ((f = fopen(st->iq_file, "r"))) {
        fclose(f);
        HAL_LOG_INFO("isp: tuning is %s (loads on the first frame)", st->iq_file);
        return;
    }

    HAL_LOG_WARN("isp: no IQ tuning at %s -- the picture stays on the "
                 "sensor driver's defaults (untuned)",
                 st->iq_file);
    st->iq_file[0] = '\0';
}

/* ================================================================
 * THE IQ INI READER
 * ================================================================ */

#define HISI_IQ_LINE_MAX 512

/*
 * The assembled value, continuations and all. The longest one in the
 * board's own /etc/sensors/iq/imx335.ini is 6034 bytes (Table_1 in
 * [dynamic_gamma]; the longest *physical* line in that file is 196). The
 * arithmetic bound for the widest thing this dialect can carry is a
 * 1025-node 12-bit table written "4095, " -- 6150 bytes, plus one joining
 * space per continuation -- so 8 KB was inside a factor of 1.3 of a real
 * file, and any vendor who pads to a wider column would run into it. That
 * matters more since truncation stopped being cosmetic: a truncated table
 * now fails the exact-node-count rule above and drops its module. 16 KB is
 * one short-lived malloc per load and puts the margin out of reach.
 */
#define HISI_IQ_VAL_MAX 16384

typedef struct {
    FILE *f;
    const char *path;
    char sect[40];
    char key[64];
    char *val; /* HISI_IQ_VAL_MAX, heap */
    bool long_line_warned;
    bool truncated_warned;
    bool bad_sect_warned;
} hisi_iq_reader;

static int iq_ci_eq(const char *a, const char *b)
{
    for (; *a && *b; a++, b++) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
            return 0;
    }
    return *a == '\0' && *b == '\0';
}

/* Case-insensitive "key is prefix_<digits>"; returns the index or -1. */
static int iq_row_index(const char *key, const char *prefix)
{
    while (*prefix) {
        if (tolower((unsigned char)*key) != tolower((unsigned char)*prefix))
            return -1;
        key++;
        prefix++;
    }
    if (*key != '_')
        return -1;
    key++;
    if (!isdigit((unsigned char)*key))
        return -1;
    return atoi(key);
}

static char *iq_trim(char *s)
{
    char *end;

    while (*s == ' ' || *s == '\t')
        s++;
    end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
        end--;
    *end = '\0';
    return s;
}

/* One physical line: read, guard against over-length, strip comment, trim.
 * Returns NULL at EOF. */
static char *iq_line(hisi_iq_reader *r, char *buf)
{
    char *comment;

    if (!fgets(buf, HISI_IQ_LINE_MAX, r->f))
        return NULL;

    /* Same policy as hisi_ini_read: a longer-than-buffer line arrives in
     * pieces and a later piece would be misparsed, so take the first piece
     * and consume the rest. The measured maximum across the shipped IQ
     * files is 229 bytes, so this guard should never fire in practice. */
    if (!strchr(buf, '\n') && !feof(r->f)) {
        int c;

        if (!r->long_line_warned) {
            HAL_LOG_WARN("isp tuning: %s: line over %d bytes truncated", r->path,
                         HISI_IQ_LINE_MAX - 1);
            r->long_line_warned = true;
        }
        while ((c = fgetc(r->f)) != EOF && c != '\n')
            ;
    }

    if ((comment = strpbrk(buf, ";#")))
        *comment = '\0';

    return iq_trim(buf);
}

static void iq_val_append(hisi_iq_reader *r, const char *piece)
{
    size_t have = strlen(r->val);
    size_t want = strlen(piece);

    if (have + want + 2 > HISI_IQ_VAL_MAX) {
        if (!r->truncated_warned) {
            HAL_LOG_WARN("isp tuning: %s: value of %s over %d bytes truncated", r->path, r->key,
                         HISI_IQ_VAL_MAX - 1);
            r->truncated_warned = true;
        }
        return;
    }
    /* A space between pieces keeps "...4095\<nl>4096..." two numbers. */
    if (have)
        r->val[have++] = ' ';
    memcpy(r->val + have, piece, want + 1);
}

/*
 * iq_next -- the next key = value entry, continuations assembled.
 *
 * Section headers are consumed transparently (r->sect tracks the current
 * one). A trailing backslash continues the value onto the next physical
 * line, which is how the shipped files carry their multi-KB tables.
 */
static bool iq_next(hisi_iq_reader *r)
{
    char buf[HISI_IQ_LINE_MAX];
    char *p;

    while ((p = iq_line(r, buf))) {
        char *eq;
        bool cont;

        if (!*p)
            continue;

        if (*p == '[') {
            char *close = strchr(p, ']');

            if (close) {
                *close = '\0';
                snprintf(r->sect, sizeof(r->sect), "%s", iq_trim(p + 1));
                continue;
            }
            /* No ']'. Keeping the previous section would file this one's
             * keys under the wrong module, which is worse than losing
             * them; drop to no section until a well-formed header. */
            if (!r->bad_sect_warned) {
                HAL_LOG_WARN("isp tuning: %s: unterminated section header \"%s\" -- "
                             "its keys are ignored until the next [section]",
                             r->path, p);
                r->bad_sect_warned = true;
            }
            r->sect[0] = '\0';
            continue;
        }

        if (!(eq = strchr(p, '=')))
            continue; /* stray fragment; nothing to anchor it to */
        *eq = '\0';

        snprintf(r->key, sizeof(r->key), "%s", iq_trim(p));
        r->val[0] = '\0';

        p = iq_trim(eq + 1);
        for (;;) {
            size_t len = strlen(p);

            cont = len > 0 && p[len - 1] == '\\';
            if (cont)
                p[len - 1] = '\0';
            iq_val_append(r, iq_trim(p));

            if (!cont)
                break;
            if (!(p = iq_line(r, buf)))
                break;
        }
        return true;
    }
    return false;
}

/* ================================================================
 * NUMBER SCANNING AND CLAMPED STORES
 * ================================================================ */

/*
 * Anything that is not part of a number separates numbers. That single
 * rule absorbs the dialect's quoting, commas, colons and stray tabs
 * without a grammar for each.
 */
static int iq_nums(const char *s, long *out, int max)
{
    int n = 0;

    while (*s && n < max) {
        while (*s && !isdigit((unsigned char)*s) && *s != '-')
            s++;
        if (!*s)
            break;
        if (*s == '-' && !isdigit((unsigned char)s[1])) {
            s++;
            continue;
        }
        out[n++] = strtol(s, (char **)&s, 10);
    }
    return n;
}

static long iq_num(const char *s, long def)
{
    long v;

    return iq_nums(s, &v, 1) == 1 ? v : def;
}

static long iq_clamp(long v, long lo, long hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void iq_fill_u8(unsigned char *dst, int dn, const long *src, int sn)
{
    int i;

    for (i = 0; i < dn && i < sn; i++)
        dst[i] = (unsigned char)iq_clamp(src[i], 0, 255);
}

static void iq_fill_u16(unsigned short *dst, int dn, const long *src, int sn)
{
    int i;

    for (i = 0; i < dn && i < sn; i++)
        dst[i] = (unsigned short)iq_clamp(src[i], 0, 65535);
}

static void iq_fill_u32(unsigned int *dst_stride24, int dn, const long *src, int sn, size_t stride)
{
    int i;

    for (i = 0; i < dn && i < sn; i++)
        *(unsigned int *)((char *)dst_stride24 + i * stride) =
            (unsigned int)iq_clamp(src[i], 0, 0x7FFFFFFF);
}

/* ================================================================
 * THE LOAD
 * ================================================================ */

enum {
    IQ_EXP = 1 << 0,
    IQ_ROUTE = 1 << 1,
    IQ_STAT = 1 << 2,
    IQ_LDCI = 1 << 3,
    IQ_DRC = 1 << 4,
    IQ_NR = 1 << 5,
    IQ_DEHAZE = 1 << 6,
    IQ_SHARPEN = 1 << 7,
    IQ_DPC = 1 << 8,
    IQ_GAMMA = 1 << 9,
};

typedef struct {
    v4_isp_exp_attr exp;
    v4_isp_ae_route_ex route;
    v4_isp_stat_cfg stat;
    v4_isp_ldci_attr ldci;
    v4_isp_drc_attr drc;
    v4_isp_nr_attr nr;
    v4_isp_dehaze_attr dehaze;
    v4_isp_sharpen_attr sharpen;
    v4_isp_dp_dyn_attr dpc;
    v4_isp_gamma_attr gamma;

    unsigned int have;    /* fetched via Get */
    unsigned int unavail; /* Get failed or symbol missing; do not retry */
    unsigned int dirty;   /* INI touched it; Set on completion */
    bool route_seen;      /* static_aerouteex present -> bAERouteExValid */

    long nums[V4_ISP_GAMMA_NODES];
    char skipped[192]; /* section names for the summary line */
} hisi_iq_load;

static const char *iq_mod_name(unsigned int bit)
{
    switch (bit) {
    case IQ_EXP:
        return "ExposureAttr";
    case IQ_ROUTE:
        return "AERouteAttrEx";
    case IQ_STAT:
        return "StatisticsConfig";
    case IQ_LDCI:
        return "LDCIAttr";
    case IQ_DRC:
        return "DRCAttr";
    case IQ_NR:
        return "NRAttr";
    case IQ_DEHAZE:
        return "DehazeAttr";
    case IQ_SHARPEN:
        return "IspSharpenAttr";
    case IQ_DPC:
        return "DPDynamicAttr";
    case IQ_GAMMA:
        return "GammaAttr";
    }
    return "?";
}

static void *iq_struct_of(hisi_iq_load *ld, unsigned int bit)
{
    switch (bit) {
    case IQ_EXP:
        return &ld->exp;
    case IQ_ROUTE:
        return &ld->route;
    case IQ_STAT:
        return &ld->stat;
    case IQ_LDCI:
        return &ld->ldci;
    case IQ_DRC:
        return &ld->drc;
    case IQ_NR:
        return &ld->nr;
    case IQ_DEHAZE:
        return &ld->dehaze;
    case IQ_SHARPEN:
        return &ld->sharpen;
    case IQ_DPC:
        return &ld->dpc;
    case IQ_GAMMA:
        return &ld->gamma;
    }
    return NULL;
}

/* The Get half of get-modify-set, once per module per load. */
static bool iq_fetch(hisi_state_t *st, hisi_iq_load *ld, unsigned int bit)
{
    int (*get)(int, void *) = NULL;
    int ret;

    if (ld->have & bit)
        return true;
    if (ld->unavail & bit)
        return false;

    switch (bit) {
    case IQ_EXP:
        get = (int (*)(int, void *))st->tune.get_exp;
        break;
    case IQ_ROUTE:
        get = (int (*)(int, void *))st->tune.get_route_ex;
        break;
    case IQ_STAT:
        get = (int (*)(int, void *))st->tune.get_stat;
        break;
    case IQ_LDCI:
        get = (int (*)(int, void *))st->tune.get_ldci;
        break;
    case IQ_DRC:
        get = (int (*)(int, void *))st->tune.get_drc;
        break;
    case IQ_NR:
        get = (int (*)(int, void *))st->tune.get_nr;
        break;
    case IQ_DEHAZE:
        get = (int (*)(int, void *))st->tune.get_dehaze;
        break;
    case IQ_SHARPEN:
        get = (int (*)(int, void *))st->tune.get_sharpen;
        break;
    case IQ_DPC:
        get = (int (*)(int, void *))st->tune.get_dpc;
        break;
    case IQ_GAMMA:
        get = (int (*)(int, void *))st->tune.get_gamma;
        break;
    }

    if (!get) {
        HAL_LOG_WARN("isp tuning: Get%s unresolved -- its sections are skipped",
                     iq_mod_name(bit));
        ld->unavail |= bit;
        return false;
    }

    ret = get(HISI_VI_PIPE, iq_struct_of(ld, bit));
    if (ret) {
        HAL_LOG_WARN("isp tuning: Get%s failed: 0x%x -- its sections are skipped",
                     iq_mod_name(bit), ret);
        ld->unavail |= bit;
        return false;
    }

    ld->have |= bit;
    return true;
}

/* ---------------- per-section handlers ---------------- */

static void iq_sect_static_ae(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v4_isp_exp_attr *e = &ld->exp;

    if (!iq_fetch(st, ld, IQ_EXP))
        return;

    if (iq_ci_eq(key, "MaxHistOffset"))
        e->auto_attr.max_hist_offset = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "HistRatioSlope"))
        e->auto_attr.hist_ratio_slope = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "AutoSpeed"))
        e->auto_attr.speed = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "AutoTolerance"))
        e->auto_attr.tolerance = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "AutoBlackDelayFrame"))
        e->auto_attr.black_delay_frame = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "AutoWhiteDelayFrame"))
        e->auto_attr.white_delay_frame = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "AERouteExValid"))
        e->route_ex_valid = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "AERunInterval"))
        e->run_interval = (unsigned char)iq_clamp(iq_num(val, 1), 1, 255);
    else if (iq_ci_eq(key, "AutoSysGainMax"))
        e->auto_attr.sysgain_range.max = (unsigned int)iq_clamp(iq_num(val, 0), 1024, 0x7FFFFFFF);
    else {
        HAL_LOG_DBG("isp tuning: [static_ae] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_EXP;
}

static void iq_sect_route_ex(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v4_isp_ae_route_ex *r = &ld->route;
    int n;

    if (!iq_fetch(st, ld, IQ_ROUTE))
        return;

    n = iq_nums(val, ld->nums, V4_ISP_AE_ROUTE_NODES);

    if (iq_ci_eq(key, "TotalNum"))
        r->total_num = (unsigned int)iq_clamp(iq_num(val, 0), 1, V4_ISP_AE_ROUTE_NODES);
    else if (iq_ci_eq(key, "RouteEXIntTime"))
        iq_fill_u32(&r->node[0].int_time, V4_ISP_AE_ROUTE_NODES, ld->nums, n,
                    sizeof(r->node[0]));
    else if (iq_ci_eq(key, "RouteEXAGain"))
        iq_fill_u32(&r->node[0].again, V4_ISP_AE_ROUTE_NODES, ld->nums, n, sizeof(r->node[0]));
    else if (iq_ci_eq(key, "RouteEXDGain"))
        iq_fill_u32(&r->node[0].dgain, V4_ISP_AE_ROUTE_NODES, ld->nums, n, sizeof(r->node[0]));
    else if (iq_ci_eq(key, "RouteEXISPDGain"))
        iq_fill_u32(&r->node[0].isp_dgain, V4_ISP_AE_ROUTE_NODES, ld->nums, n,
                    sizeof(r->node[0]));
    else {
        HAL_LOG_DBG("isp tuning: [static_aerouteex] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_ROUTE;
    ld->route_seen = true;
}

static void iq_sect_aeweight(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    int row = iq_row_index(key, "ExpWeight");
    int n, i;

    if (row < 0 || row >= V4_ISP_AE_ROWS) {
        HAL_LOG_DBG("isp tuning: [static_aeweight] %s: no mapping", key);
        return;
    }
    if (!iq_fetch(st, ld, IQ_STAT))
        return;

    n = iq_nums(val, ld->nums, V4_ISP_AE_COLS);
    for (i = 0; i < n; i++)
        ld->stat.weight[row][i] = (unsigned char)iq_clamp(ld->nums[i], 0, 15);
    ld->dirty |= IQ_STAT;
}

static void iq_sect_ldci(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v4_isp_ldci_attr *l = &ld->ldci;
    int n, i;

    if (!iq_fetch(st, ld, IQ_LDCI))
        return;

    n = iq_nums(val, ld->nums, V4_ISP_ISO_NUM);

    if (iq_ci_eq(key, "Enable"))
        l->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "LDCIOpType"))
        l->op_type = (int)iq_clamp(iq_num(val, 0), 0, 1);
    else if (iq_ci_eq(key, "LDCIGaussLPFSigma"))
        l->gauss_lpf_sigma = (unsigned char)iq_clamp(iq_num(val, 1), 1, 255);
    else if (iq_ci_eq(key, "AutoHePosWgt")) {
        for (i = 0; i < n; i++)
            l->auto_he[i].pos.wgt = (unsigned char)iq_clamp(ld->nums[i], 0, 255);
    } else if (iq_ci_eq(key, "AutoHePosSigma")) {
        for (i = 0; i < n; i++)
            l->auto_he[i].pos.sigma = (unsigned char)iq_clamp(ld->nums[i], 1, 255);
    } else if (iq_ci_eq(key, "AutoHePosMean")) {
        for (i = 0; i < n; i++)
            l->auto_he[i].pos.mean = (unsigned char)iq_clamp(ld->nums[i], 0, 255);
    } else if (iq_ci_eq(key, "AutoHeNegWgt")) {
        for (i = 0; i < n; i++)
            l->auto_he[i].neg.wgt = (unsigned char)iq_clamp(ld->nums[i], 0, 255);
    } else if (iq_ci_eq(key, "AutoHeNegSigma")) {
        for (i = 0; i < n; i++)
            l->auto_he[i].neg.sigma = (unsigned char)iq_clamp(ld->nums[i], 1, 255);
    } else if (iq_ci_eq(key, "AutoHeNegMean")) {
        for (i = 0; i < n; i++)
            l->auto_he[i].neg.mean = (unsigned char)iq_clamp(ld->nums[i], 0, 255);
    } else if (iq_ci_eq(key, "AutoBlcCtrl")) {
        for (i = 0; i < n; i++)
            l->auto_blc_ctrl[i] = (unsigned short)iq_clamp(ld->nums[i], 0, 0x1FF);
    } else {
        HAL_LOG_DBG("isp tuning: [static_ldci] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_LDCI;
}

static void iq_sect_static_drc(hisi_state_t *st, hisi_iq_load *ld, const char *key,
                               const char *val)
{
    v4_isp_drc_attr *d = &ld->drc;

    if (!iq_fetch(st, ld, IQ_DRC))
        return;

    if (iq_ci_eq(key, "Enable"))
        d->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "CurveSelect"))
        d->curve_select = (int)iq_clamp(iq_num(val, 0), 0, 2);
    else if (iq_ci_eq(key, "DRCOpType"))
        d->op_type = (int)iq_clamp(iq_num(val, 0), 0, 1);
    else if (iq_ci_eq(key, "DRCAutoStr"))
        d->auto_strength = (unsigned short)iq_clamp(iq_num(val, 0), 0, 0x3FF);
    else if (iq_ci_eq(key, "DRCAutoStrMin"))
        d->auto_strength_min = (unsigned short)iq_clamp(iq_num(val, 0), 0, 0x3FF);
    else if (iq_ci_eq(key, "DRCAutoStrMax"))
        d->auto_strength_max = (unsigned short)iq_clamp(iq_num(val, 0), 0, 0x3FF);
    else if (iq_ci_eq(key, "DRCToneMappingValue")) {
        int n = iq_nums(val, ld->nums, V4_ISP_DRC_TM_NODES);

        /* Whole table or none -- see the header. */
        if (n != V4_ISP_DRC_TM_NODES) {
            HAL_LOG_WARN("isp tuning: DRCToneMappingValue has %d of %d nodes -- not "
                         "applied; a partial curve would splice onto the running one",
                         n, V4_ISP_DRC_TM_NODES);
            return;
        }
        iq_fill_u16(d->tone_mapping, V4_ISP_DRC_TM_NODES, ld->nums, n);
    } else {
        HAL_LOG_DBG("isp tuning: [static_drc] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_DRC;
}

/* dynamic_linear_drc, applied at its lowest-ISO column (see the header). */
static void iq_sect_dynamic_drc(hisi_state_t *st, hisi_iq_load *ld, const char *key,
                                const char *val)
{
    v4_isp_drc_attr *d = &ld->drc;
    long v0 = iq_num(val, -1);

    if (!iq_fetch(st, ld, IQ_DRC))
        return;
    if (v0 < 0 && !iq_ci_eq(key, "DetailAdjustFactor")) {
        HAL_LOG_DBG("isp tuning: [dynamic_linear_drc] %s: empty", key);
        return;
    }

    if (iq_ci_eq(key, "Enable"))
        d->enable = v0 ? 1 : 0;
    else if (iq_ci_eq(key, "LocalMixingBrightMax"))
        d->local_mixing_bright_max = (unsigned char)iq_clamp(v0, 0, 0x80);
    else if (iq_ci_eq(key, "LocalMixingBrightMin"))
        d->local_mixing_bright_min = (unsigned char)iq_clamp(v0, 0, 0x40);
    else if (iq_ci_eq(key, "LocalMixingDarkMax"))
        d->local_mixing_dark_max = (unsigned char)iq_clamp(v0, 0, 0x80);
    else if (iq_ci_eq(key, "LocalMixingDarkMin"))
        d->local_mixing_dark_min = (unsigned char)iq_clamp(v0, 0, 0x40);
    else if (iq_ci_eq(key, "BrightGainLmt"))
        d->bright_gain_lmt = (unsigned char)iq_clamp(v0, 0, 0xF);
    else if (iq_ci_eq(key, "BrightGainLmtStep"))
        d->bright_gain_lmt_step = (unsigned char)iq_clamp(v0, 0, 0xF);
    else if (iq_ci_eq(key, "DarkGainLmtY"))
        d->dark_gain_lmt_y = (unsigned char)iq_clamp(v0, 0, 0x85);
    else if (iq_ci_eq(key, "DarkGainLmtC"))
        d->dark_gain_lmt_c = (unsigned char)iq_clamp(v0, 0, 0x85);
    else if (iq_ci_eq(key, "FltScaleCoarse"))
        d->flt_scale_coarse = (unsigned char)iq_clamp(v0, 0, 0xF);
    else if (iq_ci_eq(key, "FltScaleFine"))
        d->flt_scale_fine = (unsigned char)iq_clamp(v0, 0, 0xF);
    else if (iq_ci_eq(key, "ContrastControl"))
        d->contrast_control = (unsigned char)iq_clamp(v0, 0, 0xF);
    else if (iq_ci_eq(key, "DetailAdjustFactor"))
        d->detail_adjust_factor = (signed char)iq_clamp(iq_num(val, 0), -15, 15);
    else if (iq_ci_eq(key, "Asymmetry"))
        d->asym.asymmetry = (unsigned char)iq_clamp(v0, 0x1, 0x1E);
    else if (iq_ci_eq(key, "SecondPole"))
        d->asym.second_pole = (unsigned char)iq_clamp(v0, 0x96, 0xD2);
    else if (iq_ci_eq(key, "Stretch"))
        d->asym.stretch = (unsigned char)iq_clamp(v0, 0x1E, 0x3C);
    else if (iq_ci_eq(key, "Compress"))
        d->asym.compress = (unsigned char)iq_clamp(v0, 0x64, 0xC8);
    else if (iq_ci_eq(key, "Strength"))
        d->auto_strength = (unsigned short)iq_clamp(v0, 0, 0x3FF);
    else {
        HAL_LOG_DBG("isp tuning: [dynamic_linear_drc] %s: per-ISO engine only; skipped", key);
        return;
    }
    ld->dirty |= IQ_DRC;
}

static void iq_sect_nr(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v4_isp_nr_attr *nr = &ld->nr;
    int n;

    if (!iq_fetch(st, ld, IQ_NR))
        return;

    n = iq_nums(val, ld->nums, V4_ISP_ISO_NUM);

    if (iq_ci_eq(key, "Enable"))
        nr->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "FineStr")) {
        int i;
        for (i = 0; i < n; i++)
            nr->auto_fine_str[i] = (unsigned char)iq_clamp(ld->nums[i], 0, 0x80);
    } else if (iq_ci_eq(key, "CoringWgt")) {
        int i;
        for (i = 0; i < n; i++)
            nr->auto_coring_wgt[i] = (unsigned short)iq_clamp(ld->nums[i], 0, 0xC80);
    } else {
        HAL_LOG_DBG("isp tuning: [static_nr] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_NR;
}

static void iq_sect_dehaze(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v4_isp_dehaze_attr *dh = &ld->dehaze;

    if (!iq_fetch(st, ld, IQ_DEHAZE))
        return;

    if (iq_ci_eq(key, "Enable"))
        dh->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "DehazeUserLutEnable"))
        dh->user_lut_enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "DehazeOpType"))
        dh->op_type = (int)iq_clamp(iq_num(val, 0), 0, 1);
    else if (iq_ci_eq(key, "DehazeLut")) {
        int n = iq_nums(val, ld->nums, V4_ISP_DEHAZE_LUT);

        /* Whole table or none -- see the header. */
        if (n != V4_ISP_DEHAZE_LUT) {
            HAL_LOG_WARN("isp tuning: DehazeLut has %d of %d entries -- not applied; a "
                         "partial LUT would splice onto the running one",
                         n, V4_ISP_DEHAZE_LUT);
            return;
        }
        iq_fill_u8(dh->lut, V4_ISP_DEHAZE_LUT, ld->nums, n);
    } else if (iq_ci_eq(key, "AutoDehazeStr")) {
        /* dynamic_dehaze: the lowest-ISO column, statically. */
        dh->auto_strength = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    } else {
        HAL_LOG_DBG("isp tuning: [dehaze] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_DEHAZE;
}

static void iq_sect_sharpen(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v4_isp_sharpen_auto *a = &ld->sharpen.auto_attr;
    int row, n, i;

    if (!iq_fetch(st, ld, IQ_SHARPEN))
        return;

    n = iq_nums(val, ld->nums, V4_ISP_ISO_NUM);

    if ((row = iq_row_index(key, "AutoLumaWgt")) >= 0 && row < V4_ISP_SHARPEN_LUMA) {
        for (i = 0; i < n; i++)
            a->luma_wgt[row][i] = (unsigned char)iq_clamp(ld->nums[i], 0, 127);
    } else if ((row = iq_row_index(key, "AutoTextureStr")) >= 0 && row < V4_ISP_SHARPEN_GAIN) {
        for (i = 0; i < n; i++)
            a->texture_str[row][i] = (unsigned short)iq_clamp(ld->nums[i], 0, 4095);
    } else if ((row = iq_row_index(key, "AutoEdgeStr")) >= 0 && row < V4_ISP_SHARPEN_GAIN) {
        for (i = 0; i < n; i++)
            a->edge_str[row][i] = (unsigned short)iq_clamp(ld->nums[i], 0, 4095);
    } else if (iq_ci_eq(key, "Enable"))
        ld->sharpen.enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "AutoTextureFreq"))
        iq_fill_u16(a->texture_freq, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoEdgeFreq"))
        iq_fill_u16(a->edge_freq, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoOverShoot"))
        iq_fill_u8(a->over_shoot, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoUnderShoot"))
        iq_fill_u8(a->under_shoot, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoShootSupStr"))
        iq_fill_u8(a->shoot_sup_str, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoShootSupAdj"))
        iq_fill_u8(a->shoot_sup_adj, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoDetailCtrl"))
        iq_fill_u8(a->detail_ctrl, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoDetailCtrlThr"))
        iq_fill_u8(a->detail_ctrl_thr, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoEdgeFiltStr"))
        iq_fill_u8(a->edge_filt_str, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoEdgeFiltMaxCap"))
        iq_fill_u8(a->edge_filt_max_cap, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoRGain"))
        iq_fill_u8(a->r_gain, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoGGain"))
        iq_fill_u8(a->g_gain, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoBGain"))
        iq_fill_u8(a->b_gain, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoSkinGain"))
        iq_fill_u8(a->skin_gain, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoMaxSharpGain"))
        iq_fill_u16(a->max_sharp_gain, V4_ISP_ISO_NUM, ld->nums, n);
    else if (iq_ci_eq(key, "AutoWeakDetailGain"))
        iq_fill_u8(a->weak_detail_gain, V4_ISP_ISO_NUM, ld->nums, n);
    else {
        HAL_LOG_DBG("isp tuning: [static_sharpen] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_SHARPEN;
}

static void iq_sect_dpc(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v4_isp_dp_dyn_attr *dp = &ld->dpc;
    int n, i;

    if (!iq_fetch(st, ld, IQ_DPC))
        return;

    n = iq_nums(val, ld->nums, V4_ISP_ISO_NUM);

    if (iq_ci_eq(key, "DpcEnable") || iq_ci_eq(key, "Enable"))
        dp->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "DpcStrength") || iq_ci_eq(key, "Strength")) {
        for (i = 0; i < n; i++)
            dp->auto_strength[i] = (unsigned short)iq_clamp(ld->nums[i], 0, 255);
    } else if (iq_ci_eq(key, "DpcBlendRatio") || iq_ci_eq(key, "BlendRatio")) {
        for (i = 0; i < n; i++)
            dp->auto_blend_ratio[i] = (unsigned short)iq_clamp(ld->nums[i], 0, 128);
    } else {
        HAL_LOG_DBG("isp tuning: [static_dpc] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_DPC;
}

static void iq_sect_gamma(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    if (!iq_ci_eq(key, "Table_0")) {
        HAL_LOG_DBG("isp tuning: [dynamic_gamma] %s: per-exposure engine only; skipped", key);
        return;
    }
    if (!iq_fetch(st, ld, IQ_GAMMA))
        return;

    {
        int n = iq_nums(val, ld->nums, V4_ISP_GAMMA_NODES);
        int i;

        /* Whole table or none. Applying would also flip the curve type to
         * USER_DEFINE, under which nodes past n -- still holding whatever
         * GetGammaAttr returned for a different curve -- become live. */
        if (n != V4_ISP_GAMMA_NODES) {
            HAL_LOG_WARN("isp tuning: gamma Table_0 has %d of %d nodes -- module skipped; "
                         "a partial user curve would run on a stale tail",
                         n, V4_ISP_GAMMA_NODES);
            return;
        }
        for (i = 0; i < n; i++)
            ld->gamma.table[i] = (unsigned short)iq_clamp(ld->nums[i], 0, 4095);
    }
    ld->gamma.enable = 1;
    ld->gamma.curve_type = V4_ISP_GAMMA_CURVE_USER;
    ld->dirty |= IQ_GAMMA;
}

/* Sections with no static application, remembered once for the summary. */
static void iq_note_skip(hisi_iq_load *ld, const char *sect)
{
    size_t have = strlen(ld->skipped);

    /* Already noted? A substring match is enough at this scale. */
    if (strstr(ld->skipped, sect))
        return;
    if (have + strlen(sect) + 2 >= sizeof(ld->skipped))
        return;
    snprintf(ld->skipped + have, sizeof(ld->skipped) - have, "%s%s", have ? " " : "", sect);
}

static void iq_dispatch(hisi_state_t *st, hisi_iq_load *ld, hisi_iq_reader *r)
{
    const char *s = r->sect;

    /* No section: either keys before the first header or the fallout of a
     * malformed one. Either way they belong to no module. */
    if (!s[0])
        return;

    /* The ir_* mirror is the night-mode set; day is what this load is. */
    if (tolower((unsigned char)s[0]) == 'i' && tolower((unsigned char)s[1]) == 'r' &&
        s[2] == '_') {
        iq_note_skip(ld, "ir_*");
        return;
    }

    if (iq_ci_eq(s, "static_ae"))
        iq_sect_static_ae(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_aerouteex"))
        iq_sect_route_ex(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_aeweight"))
        iq_sect_aeweight(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_ldci"))
        iq_sect_ldci(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_drc"))
        iq_sect_static_drc(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "dynamic_linear_drc"))
        iq_sect_dynamic_drc(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_nr"))
        iq_sect_nr(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_dehaze") || iq_ci_eq(s, "dynamic_dehaze"))
        iq_sect_dehaze(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_sharpen"))
        iq_sect_sharpen(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_dpc"))
        iq_sect_dpc(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "dynamic_gamma"))
        iq_sect_gamma(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_3dnr")) {
        if (!hisi_nrx_key(st, r->key, r->val))
            HAL_LOG_DBG("isp tuning: [static_3dnr] %s: no mapping", r->key);
    }
    else
        iq_note_skip(ld, s);
}

/* ---------------- resolve, load, apply ---------------- */

static void hisi_isp_tune_resolve(hisi_state_t *st)
{
    v4_mpi_libs search;

    if (st->tune_resolved)
        return;
    st->tune_resolved = true;

    memset(&search, 0, sizeof(search));
    memcpy(search.search, st->isp.search,
           sizeof(search.search) < sizeof(st->isp.search) ? sizeof(search.search)
                                                          : sizeof(st->isp.search));

#define V4_TUNE(field, type, hi, gk) st->tune.field = (type)v4_symbol_opt(&search, hi, gk)

    V4_TUNE(get_exp, int (*)(int, v4_isp_exp_attr *), "HI_MPI_ISP_GetExposureAttr",
            "GK_API_ISP_GetExposureAttr");
    V4_TUNE(set_exp, int (*)(int, const v4_isp_exp_attr *), "HI_MPI_ISP_SetExposureAttr",
            "GK_API_ISP_SetExposureAttr");
    V4_TUNE(get_route_ex, int (*)(int, v4_isp_ae_route_ex *), "HI_MPI_ISP_GetAERouteAttrEx",
            "GK_API_ISP_GetAERouteAttrEx");
    V4_TUNE(set_route_ex, int (*)(int, const v4_isp_ae_route_ex *), "HI_MPI_ISP_SetAERouteAttrEx",
            "GK_API_ISP_SetAERouteAttrEx");
    V4_TUNE(get_stat, int (*)(int, v4_isp_stat_cfg *), "HI_MPI_ISP_GetStatisticsConfig",
            "GK_API_ISP_GetStatisticsConfig");
    V4_TUNE(set_stat, int (*)(int, const v4_isp_stat_cfg *), "HI_MPI_ISP_SetStatisticsConfig",
            "GK_API_ISP_SetStatisticsConfig");
    V4_TUNE(get_ldci, int (*)(int, v4_isp_ldci_attr *), "HI_MPI_ISP_GetLDCIAttr",
            "GK_API_ISP_GetLDCIAttr");
    V4_TUNE(set_ldci, int (*)(int, const v4_isp_ldci_attr *), "HI_MPI_ISP_SetLDCIAttr",
            "GK_API_ISP_SetLDCIAttr");
    V4_TUNE(get_drc, int (*)(int, v4_isp_drc_attr *), "HI_MPI_ISP_GetDRCAttr",
            "GK_API_ISP_GetDRCAttr");
    V4_TUNE(set_drc, int (*)(int, const v4_isp_drc_attr *), "HI_MPI_ISP_SetDRCAttr",
            "GK_API_ISP_SetDRCAttr");
    V4_TUNE(get_nr, int (*)(int, v4_isp_nr_attr *), "HI_MPI_ISP_GetNRAttr",
            "GK_API_ISP_GetNRAttr");
    V4_TUNE(set_nr, int (*)(int, const v4_isp_nr_attr *), "HI_MPI_ISP_SetNRAttr",
            "GK_API_ISP_SetNRAttr");
    V4_TUNE(get_dehaze, int (*)(int, v4_isp_dehaze_attr *), "HI_MPI_ISP_GetDehazeAttr",
            "GK_API_ISP_GetDehazeAttr");
    V4_TUNE(set_dehaze, int (*)(int, const v4_isp_dehaze_attr *), "HI_MPI_ISP_SetDehazeAttr",
            "GK_API_ISP_SetDehazeAttr");
    V4_TUNE(get_sharpen, int (*)(int, v4_isp_sharpen_attr *), "HI_MPI_ISP_GetIspSharpenAttr",
            "GK_API_ISP_GetIspSharpenAttr");
    V4_TUNE(set_sharpen, int (*)(int, const v4_isp_sharpen_attr *), "HI_MPI_ISP_SetIspSharpenAttr",
            "GK_API_ISP_SetIspSharpenAttr");
    V4_TUNE(get_dpc, int (*)(int, v4_isp_dp_dyn_attr *), "HI_MPI_ISP_GetDPDynamicAttr",
            "GK_API_ISP_GetDPDynamicAttr");
    V4_TUNE(set_dpc, int (*)(int, const v4_isp_dp_dyn_attr *), "HI_MPI_ISP_SetDPDynamicAttr",
            "GK_API_ISP_SetDPDynamicAttr");
    V4_TUNE(get_gamma, int (*)(int, v4_isp_gamma_attr *), "HI_MPI_ISP_GetGammaAttr",
            "GK_API_ISP_GetGammaAttr");
    V4_TUNE(set_gamma, int (*)(int, const v4_isp_gamma_attr *), "HI_MPI_ISP_SetGammaAttr",
            "GK_API_ISP_SetGammaAttr");
    V4_TUNE(query_exp, int (*)(int, v4_isp_exp_info *), "HI_MPI_ISP_QueryExposureInfo",
            "GK_API_ISP_QueryExposureInfo");

#undef V4_TUNE
}

static void hisi_isp_apply_tuning(hisi_state_t *st)
{
    static const unsigned int apply_order[] = {IQ_EXP,    IQ_ROUTE,   IQ_STAT, IQ_NR,
                                               IQ_SHARPEN, IQ_LDCI,   IQ_DRC,  IQ_DEHAZE,
                                               IQ_DPC,    IQ_GAMMA};
    hisi_iq_reader r;
    hisi_iq_load *ld;
    int applied = 0, failed = 0;
    unsigned int i;

    hisi_isp_tune_resolve(st);

    memset(&r, 0, sizeof(r));
    r.path = st->iq_file;
    if (!(r.f = fopen(st->iq_file, "r"))) {
        HAL_LOG_WARN("isp tuning: %s vanished between resolve and load", st->iq_file);
        return;
    }
    if (!(ld = calloc(1, sizeof(*ld))) || !(r.val = malloc(HISI_IQ_VAL_MAX))) {
        HAL_LOG_WARN("isp tuning: out of memory; running untuned");
        free(ld);
        fclose(r.f);
        return;
    }

    while (iq_next(&r))
        iq_dispatch(st, ld, &r);
    fclose(r.f);

    /* A route section without an explicit AERouteExValid still needs the
     * flag, or the route the file spent five lines on is dead letter. */
    if (ld->route_seen && (ld->have & IQ_EXP)) {
        ld->exp.route_ex_valid = 1;
        ld->dirty |= IQ_EXP;
    }

    for (i = 0; i < sizeof(apply_order) / sizeof(apply_order[0]); i++) {
        unsigned int bit = apply_order[i];
        int (*set)(int, const void *) = NULL;
        int ret;

        if (!(ld->dirty & bit) || !(ld->have & bit))
            continue;

        switch (bit) {
        case IQ_EXP:
            set = (int (*)(int, const void *))st->tune.set_exp;
            break;
        case IQ_ROUTE:
            set = (int (*)(int, const void *))st->tune.set_route_ex;
            break;
        case IQ_STAT:
            set = (int (*)(int, const void *))st->tune.set_stat;
            break;
        case IQ_LDCI:
            set = (int (*)(int, const void *))st->tune.set_ldci;
            break;
        case IQ_DRC:
            set = (int (*)(int, const void *))st->tune.set_drc;
            break;
        case IQ_NR:
            set = (int (*)(int, const void *))st->tune.set_nr;
            break;
        case IQ_DEHAZE:
            set = (int (*)(int, const void *))st->tune.set_dehaze;
            break;
        case IQ_SHARPEN:
            set = (int (*)(int, const void *))st->tune.set_sharpen;
            break;
        case IQ_DPC:
            set = (int (*)(int, const void *))st->tune.set_dpc;
            break;
        case IQ_GAMMA:
            set = (int (*)(int, const void *))st->tune.set_gamma;
            break;
        }

        if (!set) {
            HAL_LOG_WARN("isp tuning: Set%s unresolved -- module keeps its defaults",
                         iq_mod_name(bit));
            failed++;
            continue;
        }
        ret = set(HISI_VI_PIPE, iq_struct_of(ld, bit));
        if (ret) {
            HAL_LOG_WARN("isp tuning: Set%s failed: 0x%x -- module keeps its defaults",
                         iq_mod_name(bit), ret);
            failed++;
        } else {
            HAL_LOG_DBG("isp tuning: Set%s applied", iq_mod_name(bit));
            applied++;
        }
    }

    /* The VPSS half: the 3DNR ladder, written through its own call. */
    {
        char note[64];
        int nrx = hisi_nrx_apply(st, note, sizeof(note));

        if (nrx > 0)
            applied++;
        else if (nrx < 0) {
            failed++;
            iq_note_skip(ld, note);
        }
    }

    if (failed)
        HAL_LOG_WARN("isp tuning: %s: %d modules applied, %d failed%s%s", st->iq_file, applied,
                     failed, ld->skipped[0] ? "; skipped: " : "", ld->skipped);
    else
        HAL_LOG_INFO("isp tuning: %s: %d modules applied%s%s", st->iq_file, applied,
                     ld->skipped[0] ? "; skipped: " : "", ld->skipped);

    free(r.val);
    free(ld);
}

/*
 * hisi_isp_note_frame -- one-shot hook off the encoder's frame loop.
 *
 * rvd runs an encoder thread per stream and every one reaches here, so
 * the latch is atomic and exactly one thread does the work. A restart
 * rebuilds the state from zero, which clears the latch, so the tuning
 * is reloaded on the next run.
 */
void hisi_isp_note_frame(hisi_state_t *st)
{
    if (__atomic_test_and_set(&st->iq_load_started, __ATOMIC_ACQ_REL)) {
        /* Every frame after the load: the 3DNR ladder's ISO tick, which
         * rate-limits itself and does nothing until the load has finished
         * building the ladder. */
        hisi_nrx_tick(st);
        return;
    }

    if (st->iq_file[0])
        hisi_isp_apply_tuning(st);
}
