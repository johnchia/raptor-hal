/*
 * t_hisi_iq -- the gen4 IQ tuning loader, host-side.
 *
 * Includes the real hal_isp.c and stubs the ten Get/Set pairs, so what is
 * tested is the shipping parser and field routing: continuation lines,
 * the quoting dialect, comment stripping, per-row keys, the ir_* skip,
 * the daylight-column application of the dynamic sections, and the
 * implied bAERouteExValid. The INI is synthesized here rather than
 * vendored -- the shipped files are OEM tuning data -- with every quirk
 * of the dialect represented: a 1025-node continued table, quoted lists,
 * trailing commas, comments after values.
 *
 * Three loads run, each against freshly zeroed stub state:
 *
 *   1. a well-formed file, which is the field-routing test above;
 *   2. a deliberately malformed one -- unterminated section headers, a
 *      line past HISI_IQ_LINE_MAX and one exactly at it, an assembled
 *      value past HISI_IQ_VAL_MAX, values that overflow strtol, short
 *      whole-tables, and a value left hanging on a backslash at EOF --
 *      where the question is what the loader refuses to apply;
 *   3. one with the vendor calls failing: a Get that errors, a Set that
 *      errors, and a symbol that never resolved;
 *   4. files carrying [module_state], the vendor's own enable mask, where
 *      the question is which sections the loader declines to apply and
 *      whether it still reads the mask when the file puts it last.
 *
 * Loads 2 and 3 assert on the log as well as on the structs, because
 * "skipped" is most of what they are supposed to do, and a skip is
 * otherwise indistinguishable from a parse that never happened.
 *
 * The tuning structs contain no pointers, so their ARM32 layouts hold on
 * the host too; the state header's *other* vendor structs do not, which
 * is why the Makefile defines _Static_assert away here like every suite.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "../src/hisi_v4/hal_isp.c"
#include "../src/hisi_v4/hal_nrx.c"
#include "../src/hisi_v4/hal_dyn.c"
#include "../src/hisi_v4/hal_knob.c"

#include <assert.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static v4_isp_exp_attr g_exp;
static v4_isp_ae_route_ex g_route;
static v4_isp_stat_cfg g_stat;
static v4_isp_ldci_attr g_ldci;
static v4_isp_drc_attr g_drc;
static v4_isp_nr_attr g_nr;
static v4_isp_dehaze_attr g_dehaze;
static v4_isp_sharpen_attr g_sharpen;
static v4_isp_dp_dyn_attr g_dpc;
static v4_isp_gamma_attr g_gamma;
static v4_isp_csc_attr g_csc;
static v4_isp_sat_attr g_sat;

enum {
    M_EXP,
    M_ROUTE,
    M_STAT,
    M_LDCI,
    M_DRC,
    M_NR,
    M_DEHAZE,
    M_SHARPEN,
    M_DPC,
    M_GAMMA,
    M_CSC,
    M_SAT,
    M_N
};

/* Failure injection for load 3. HI_DEF_ERR-shaped values so the log lines
 * read like the real thing; nothing reads them back. */
static bool g_get_fail[M_N];
static bool g_set_fail[M_N];
static int g_sets;      /* Sets that stored something */
static int g_set_calls; /* Sets that were reached at all */

#define STUB(name, T, g, idx)                                                                      \
    static int get_##name(int p, T *a)                                                             \
    {                                                                                              \
        (void)p;                                                                                   \
        if (g_get_fail[idx])                                                                       \
            return 0xA0018003;                                                                     \
        *a = g;                                                                                    \
        return 0;                                                                                  \
    }                                                                                              \
    static int set_##name(int p, const T *a)                                                       \
    {                                                                                              \
        (void)p;                                                                                   \
        g_set_calls++;                                                                             \
        if (g_set_fail[idx])                                                                       \
            return 0xA0018004;                                                                     \
        g = *a;                                                                                    \
        g_sets++;                                                                                  \
        return 0;                                                                                  \
    }

STUB(exp, v4_isp_exp_attr, g_exp, M_EXP)
STUB(route, v4_isp_ae_route_ex, g_route, M_ROUTE)
STUB(stat, v4_isp_stat_cfg, g_stat, M_STAT)
STUB(ldci, v4_isp_ldci_attr, g_ldci, M_LDCI)
STUB(drc, v4_isp_drc_attr, g_drc, M_DRC)
STUB(nr, v4_isp_nr_attr, g_nr, M_NR)
STUB(dehaze, v4_isp_dehaze_attr, g_dehaze, M_DEHAZE)
STUB(sharpen, v4_isp_sharpen_attr, g_sharpen, M_SHARPEN)
STUB(dpc, v4_isp_dp_dyn_attr, g_dpc, M_DPC)
STUB(gamma, v4_isp_gamma_attr, g_gamma, M_GAMMA)
STUB(csc, v4_isp_csc_attr, g_csc, M_CSC)
STUB(sat, v4_isp_sat_attr, g_sat, M_SAT)

/* hal_framesource.c is not in this suite; the orientation ops hand the
 * remembered bits to it, and this records what they handed over. */
static int g_orien_calls, g_orien_mirror, g_orien_flip;

int hisi_fs_apply_orien(hisi_state_t *st)
{
    g_orien_calls++;
    g_orien_mirror = st->mirror;
    g_orien_flip = st->flip;
    return RSS_OK;
}

/* ---------------- log capture ---------------- */

static char g_log[256 * 1024];
static size_t g_log_len;

static void t_log(int level, const char *file, int line, const char *fmt, ...)
{
    char msg[2048];
    va_list ap;
    int n;

    (void)level;
    (void)file;
    (void)line;
    va_start(ap, fmt);
    n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;

    fprintf(stderr, "%s\n", msg);
    n = snprintf(g_log + g_log_len, sizeof(g_log) - g_log_len, "%s\n", msg);
    if (n > 0 && (size_t)n < sizeof(g_log) - g_log_len)
        g_log_len += (size_t)n;
}
rss_hal_log_func_t rss_hal_log_fn = t_log;

static int log_count(const char *needle)
{
    const char *p = g_log;
    int n = 0;

    while ((p = strstr(p, needle))) {
        n++;
        p += strlen(needle);
    }
    return n;
}

/* ---------------- the VPSS NRX pair, hal_nrx.c's two vendor calls ---------------- */

static int g_nrx_get_calls, g_nrx_set_calls;
static bool g_nrx_get_fail, g_nrx_set_fail;
static bool g_nrx_auto_refuse; /* the driver takes MANUAL but not the ladder */
static unsigned g_iso;      /* what the stubbed AE query reports */
static unsigned g_exp_time; /* ...and its integration time, us */
static int g_nrx_mode;
static unsigned g_nrx_num;
static unsigned g_nrx_iso[V4_VPSS_NRX_MAX_BLOCKS];
static v4_vpss_nrx_v3 g_nrx_blk[V4_VPSS_NRX_MAX_BLOCKS];

static int get_nrx(int grp, v4_vpss_grp_nrx_param *p)
{
    int i;

    (void)grp;
    g_nrx_get_calls++;
    if (g_nrx_get_fail)
        return 0xA0078003;
    assert(p->nr_ver == V4_VPSS_NR_V3);
    /* Fields the text never names, which the pack must carry through. */
    for (i = 0; i < 5; i++) {
        p->v3.manual.sfy[i].sbsk[0] = 1234;
        p->v3.manual.sfy[i].sdsk[31] = 4321;
        p->v3.manual.sfy[i].nry_en = 1;
        p->v3.manual.iey[i].ie_en = 1;
    }
    p->v3.manual.mdy[0].madz0 = 300;
    p->v3.manual.tfy[0].dzmode0 = 1;
    /* And one the text does name, to prove it is overwritten. */
    p->v3.manual.nrc.sfc = 7;
    return 0;
}

static int set_nrx(int grp, const v4_vpss_grp_nrx_param *p)
{
    unsigned i;

    (void)grp;
    g_nrx_set_calls++;
    assert(p->nr_ver == V4_VPSS_NR_V3);
    if (g_nrx_set_fail)
        return 0xA0078006;
    /* What this driver answered to an AUTO write before the NRc offsets
     * were corrected, and what any driver too old for the ladder answers:
     * VPSS/ILLEGAL_PARAM. */
    if (g_nrx_auto_refuse && p->v3.opt_mode == V4_OPERATION_MODE_AUTO)
        return 0xA0078003;
    g_nrx_mode = p->v3.opt_mode;
    if (p->v3.opt_mode == V4_OPERATION_MODE_AUTO) {
        g_nrx_num = p->v3.auto_.param_num;
        assert(g_nrx_num <= V4_VPSS_NRX_MAX_BLOCKS);
        for (i = 0; i < g_nrx_num; i++) {
            g_nrx_iso[i] = p->v3.auto_.iso[i];
            g_nrx_blk[i] = p->v3.auto_.params[i];
        }
    } else {
        g_nrx_num = 1;
        g_nrx_blk[0] = p->v3.manual;
    }
    return 0;
}

static unsigned g_again, g_dgain, g_isp_dgain;
static unsigned char g_ave_lum;

static int query_exp(int pipe, v4_isp_exp_info *info)
{
    (void)pipe;
    memset(info, 0, sizeof(*info));
    info->iso = g_iso;
    info->exp_time = g_exp_time;
    info->again = g_again;
    info->dgain = g_dgain;
    info->isp_dgain = g_isp_dgain;
    info->ave_lum = g_ave_lum;
    return 0;
}

/* Block 0 of the shipped imx335.ini, verbatim: the ISO 100 rung. */
static const char NRX_ROWS_A[] =
    "-nXsf1      18:  0:128 |     20:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-nXsf2      20:  0:128 |     30:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-nXsf4      18:  0:128 |     25:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-SelRt          16: 16 |                | -kmode       1 |                   1           \\\n"
    "-DeRt            0:  4 |                |                |                               \\\n"
    "-sfs5                  |                |                |          60: 60: 60           \\\n"
    "-nXsf5  64: 64: 64: 64 | 64: 64: 64: 64 | 64: 64: 64: 64 |110: 90: 64: 64| 96: 72: 64: 64\\\n"
    "-dzsf5               0 |              0 |              0 |              0|              0\\\n"
    "-nXsf6   0:  0:  0:  0 |  4:  2:  0:  4 |  4:  2:  0:  4 |  4:  5:  0:  4|  1:  5:  0:  4\\\n"
    "-nXsfr6  0:  0:  0:  0 |  0:  0:  8:  0 |  0:  0:  8:  0 | 10: 10:  0:  0| 20: 20:  0:  0\\\n"
    "-nXsbr6         15: 15 |         12: 12 |         12: 12 |         12: 15|         12: 15\\\n"
    "                       |                |                |               |               \\\n"
    "-nXsfn       1:  2:  4 |      6:  6:  4 |      6:  6:  4 |      6:  6:  4|      6:  6:  4\\\n"
    "-nXsth          20: 40 |         30: 30 |         32: 36 |         36: 40|         36: 40\\\n"
    "-nXsthd         15: 20 |         20: 20 |         24: 28 |         28: 30|         24: 30\\\n"
    "-sfr    (0)     31     |         31     |         31     |         31    |         31    \\\n"
    "                       |                |                |                               \\\n"
    "-ref             0     |          1     |                |                               \\\n"
    "-tedge                 |          0     |          0     | -mXmath       90              \\\n"
    "                       |                |                | -mXmathd      60              \\\n"
    "-nXstr  (1)     31     |         31: 31 |         31     | -mXmate        2              \\\n"
    "-nXsdz           0     |          0:  0 |          0     | -mXmabw        5              \\\n"
    "                       |                |                |                               \\\n"
    "-nXtss           0     |          0:  0 |          0     |                               \\\n"
    "-nXtsi           1     |          1:  1 |          1     |                               \\\n"
    "-nXtfs           0     |          7: 11 |         10     |                               \\\n"
    "-nXtdz  (3)      0     |          0:  0 |          0     |**************NRc**************\\\n"
    "-nXtdx           2     |          2:  2 |          2     | -mode          0              \\\n"
    "-nXtfrs         15     |                |                | -presfc        0              \\\n"
    "-nXtfr0 (2) 16:  8: 16 |      8:  4:  0 |     16:  8: 16 | -sfc          60              \\\n"
    "             8:  0:  0 |      0:  0:  0 |      8:  0:  0 | -tfc          10              \\\n"
    "-nXtfr1 (2)            |     16:  8: 16 |                | -tpc          10              \\\n"
    "                       |      8:  0:  0 |                | -trc          12              \\\n"
    "                       |                |                |                               \\\n"
    "-mXid0                 |      1:  1:  2 |      1:  1:  2 |                               \\\n"
    "-mXid1                 |      2:  2:  2 |                |                               \\\n"
    "-mXmabr                |          0:  0 |          0     |                               \\\n"
    "-AdvMath               |          1     |                |                               \\\n"
    "-AdvTh                 |          0     |                |                               \\\n"
    "-mXmath                |         40:150 |        150     |                               \\\n"
    "-mXmathd               |         20:120 |        100     |                               \\\n"
    "-mXmate                |          2:  2 |          2     |                               \\\n"
    "-mXmabw                |          4:  9 |          5     |                               \\\n"
    "-mXmatw                |              3 |          3     |                               \\\n"
    ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\n";

/* The same rung with -sfc 60 -> 61 and -nXtfs "7: 11" -> "9: 13". */
static const char NRX_ROWS_B[] =
    "-nXsf1      18:  0:128 |     20:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-nXsf2      20:  0:128 |     30:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-nXsf4      18:  0:128 |     25:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-SelRt          16: 16 |                | -kmode       1 |                   1           \\\n"
    "-DeRt            0:  4 |                |                |                               \\\n"
    "-sfs5                  |                |                |          60: 60: 60           \\\n"
    "-nXsf5  64: 64: 64: 64 | 64: 64: 64: 64 | 64: 64: 64: 64 |110: 90: 64: 64| 96: 72: 64: 64\\\n"
    "-dzsf5               0 |              0 |              0 |              0|              0\\\n"
    "-nXsf6   0:  0:  0:  0 |  4:  2:  0:  4 |  4:  2:  0:  4 |  4:  5:  0:  4|  1:  5:  0:  4\\\n"
    "-nXsfr6  0:  0:  0:  0 |  0:  0:  8:  0 |  0:  0:  8:  0 | 10: 10:  0:  0| 20: 20:  0:  0\\\n"
    "-nXsbr6         15: 15 |         12: 12 |         12: 12 |         12: 15|         12: 15\\\n"
    "                       |                |                |               |               \\\n"
    "-nXsfn       1:  2:  4 |      6:  6:  4 |      6:  6:  4 |      6:  6:  4|      6:  6:  4\\\n"
    "-nXsth          20: 40 |         30: 30 |         32: 36 |         36: 40|         36: 40\\\n"
    "-nXsthd         15: 20 |         20: 20 |         24: 28 |         28: 30|         24: 30\\\n"
    "-sfr    (0)     31     |         31     |         31     |         31    |         31    \\\n"
    "                       |                |                |                               \\\n"
    "-ref             0     |          1     |                |                               \\\n"
    "-tedge                 |          0     |          0     | -mXmath       90              \\\n"
    "                       |                |                | -mXmathd      60              \\\n"
    "-nXstr  (1)     31     |         31: 31 |         31     | -mXmate        2              \\\n"
    "-nXsdz           0     |          0:  0 |          0     | -mXmabw        5              \\\n"
    "                       |                |                |                               \\\n"
    "-nXtss           0     |          0:  0 |          0     |                               \\\n"
    "-nXtsi           1     |          1:  1 |          1     |                               \\\n"
    "-nXtfs           0     |          9: 13 |         10     |                               \\\n"
    "-nXtdz  (3)      0     |          0:  0 |          0     |**************NRc**************\\\n"
    "-nXtdx           2     |          2:  2 |          2     | -mode          0              \\\n"
    "-nXtfrs         15     |                |                | -presfc        0              \\\n"
    "-nXtfr0 (2) 16:  8: 16 |      8:  4:  0 |     16:  8: 16 | -sfc          61              \\\n"
    "             8:  0:  0 |      0:  0:  0 |      8:  0:  0 | -tfc          10              \\\n"
    "-nXtfr1 (2)            |     16:  8: 16 |                | -tpc          10              \\\n"
    "                       |      8:  0:  0 |                | -trc          12              \\\n"
    "                       |                |                |                               \\\n"
    "-mXid0                 |      1:  1:  2 |      1:  1:  2 |                               \\\n"
    "-mXid1                 |      2:  2:  2 |                |                               \\\n"
    "-mXmabr                |          0:  0 |          0     |                               \\\n"
    "-AdvMath               |          1     |                |                               \\\n"
    "-AdvTh                 |          0     |                |                               \\\n"
    "-mXmath                |         40:150 |        150     |                               \\\n"
    "-mXmathd               |         20:120 |        100     |                               \\\n"
    "-mXmate                |          2:  2 |          2     |                               \\\n"
    "-mXmabw                |          4:  9 |          5     |                               \\\n"
    "-mXmatw                |              3 |          3     |                               \\\n"
    ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\n";

/* Block A without its last row (-mXmatw): incomplete, must be skipped. */
static const char NRX_ROWS_SHORT[] =
    "-nXsf1      18:  0:128 |     20:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-nXsf2      20:  0:128 |     30:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-nXsf4      18:  0:128 |     25:  0:128 |     20:  0:128 |          30:  0:128           \\\n"
    "-SelRt          16: 16 |                | -kmode       1 |                   1           \\\n"
    "-DeRt            0:  4 |                |                |                               \\\n"
    "-sfs5                  |                |                |          60: 60: 60           \\\n"
    "-nXsf5  64: 64: 64: 64 | 64: 64: 64: 64 | 64: 64: 64: 64 |110: 90: 64: 64| 96: 72: 64: 64\\\n"
    "-dzsf5               0 |              0 |              0 |              0|              0\\\n"
    "-nXsf6   0:  0:  0:  0 |  4:  2:  0:  4 |  4:  2:  0:  4 |  4:  5:  0:  4|  1:  5:  0:  4\\\n"
    "-nXsfr6  0:  0:  0:  0 |  0:  0:  8:  0 |  0:  0:  8:  0 | 10: 10:  0:  0| 20: 20:  0:  0\\\n"
    "-nXsbr6         15: 15 |         12: 12 |         12: 12 |         12: 15|         12: 15\\\n"
    "                       |                |                |               |               \\\n"
    "-nXsfn       1:  2:  4 |      6:  6:  4 |      6:  6:  4 |      6:  6:  4|      6:  6:  4\\\n"
    "-nXsth          20: 40 |         30: 30 |         32: 36 |         36: 40|         36: 40\\\n"
    "-nXsthd         15: 20 |         20: 20 |         24: 28 |         28: 30|         24: 30\\\n"
    "-sfr    (0)     31     |         31     |         31     |         31    |         31    \\\n"
    "                       |                |                |                               \\\n"
    "-ref             0     |          1     |                |                               \\\n"
    "-tedge                 |          0     |          0     | -mXmath       90              \\\n"
    "                       |                |                | -mXmathd      60              \\\n"
    "-nXstr  (1)     31     |         31: 31 |         31     | -mXmate        2              \\\n"
    "-nXsdz           0     |          0:  0 |          0     | -mXmabw        5              \\\n"
    "                       |                |                |                               \\\n"
    "-nXtss           0     |          0:  0 |          0     |                               \\\n"
    "-nXtsi           1     |          1:  1 |          1     |                               \\\n"
    "-nXtfs           0     |          7: 11 |         10     |                               \\\n"
    "-nXtdz  (3)      0     |          0:  0 |          0     |**************NRc**************\\\n"
    "-nXtdx           2     |          2:  2 |          2     | -mode          0              \\\n"
    "-nXtfrs         15     |                |                | -presfc        0              \\\n"
    "-nXtfr0 (2) 16:  8: 16 |      8:  4:  0 |     16:  8: 16 | -sfc          60              \\\n"
    "             8:  0:  0 |      0:  0:  0 |      8:  0:  0 | -tfc          10              \\\n"
    "-nXtfr1 (2)            |     16:  8: 16 |                | -tpc          10              \\\n"
    "                       |      8:  0:  0 |                | -trc          12              \\\n"
    "                       |                |                |                               \\\n"
    "-mXid0                 |      1:  1:  2 |      1:  1:  2 |                               \\\n"
    "-mXid1                 |      2:  2:  2 |                |                               \\\n"
    "-mXmabr                |          0:  0 |          0     |                               \\\n"
    "-AdvMath               |          1     |                |                               \\\n"
    "-AdvTh                 |          0     |                |                               \\\n"
    "-mXmath                |         40:150 |        150     |                               \\\n"
    "-mXmathd               |         20:120 |        100     |                               \\\n"
    "-mXmate                |          2:  2 |          2     |                               \\\n"
    "-mXmabw                |          4:  9 |          5     |                               \\\n"
    ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\n";

/* ---------------- fixture ---------------- */

static void reset_all(hisi_state_t *st)
{
    memset(&g_exp, 0, sizeof(g_exp));
    memset(&g_route, 0, sizeof(g_route));
    memset(&g_stat, 0, sizeof(g_stat));
    memset(&g_ldci, 0, sizeof(g_ldci));
    memset(&g_drc, 0, sizeof(g_drc));
    memset(&g_nr, 0, sizeof(g_nr));
    memset(&g_dehaze, 0, sizeof(g_dehaze));
    memset(&g_sharpen, 0, sizeof(g_sharpen));
    memset(&g_dpc, 0, sizeof(g_dpc));
    memset(&g_gamma, 0, sizeof(g_gamma));
    memset(&g_csc, 0, sizeof(g_csc));
    memset(&g_sat, 0, sizeof(g_sat));
    g_again = g_dgain = g_isp_dgain = 1024;
    g_ave_lum = 0;
    memset(g_get_fail, 0, sizeof(g_get_fail));
    memset(g_set_fail, 0, sizeof(g_set_fail));
    g_sets = 0;
    g_set_calls = 0;
    g_log[0] = '\0';
    g_log_len = 0;

    g_nrx_get_calls = 0;
    g_nrx_set_calls = 0;
    g_nrx_get_fail = false;
    g_nrx_set_fail = false;
    g_nrx_auto_refuse = false;
    g_iso = 100;
    g_exp_time = 1000; /* ISO 100 at 1 ms: exposure 1000, the first gamma band */
    g_nrx_mode = -1;
    g_nrx_num = 0;
    memset(g_nrx_iso, 0, sizeof(g_nrx_iso));
    memset(g_nrx_blk, 0, sizeof(g_nrx_blk));

    hisi_nrx_free(st); /* the previous load's ladder, before the memset loses it */
    hisi_dyn_free(st);
    memset(st, 0, sizeof(*st));
    st->tune_resolved = true; /* keep the stubs; nothing to dlsym here */
    st->vpss.fnGetGrpNRXParam = get_nrx;
    st->vpss.fnSetGrpNRXParam = set_nrx;
    st->tune.query_exp = query_exp;
    st->tune.get_exp = get_exp;
    st->tune.set_exp = set_exp;
    st->tune.get_csc = get_csc;
    st->tune.set_csc = set_csc;
    st->tune.get_route_ex = get_route;
    st->tune.set_route_ex = set_route;
    st->tune.get_stat = get_stat;
    st->tune.set_stat = set_stat;
    st->tune.get_ldci = get_ldci;
    st->tune.set_ldci = set_ldci;
    st->tune.get_drc = get_drc;
    st->tune.set_drc = set_drc;
    st->tune.get_nr = get_nr;
    st->tune.set_nr = set_nr;
    st->tune.get_dehaze = get_dehaze;
    st->tune.set_dehaze = set_dehaze;
    st->tune.get_sharpen = get_sharpen;
    st->tune.set_sharpen = set_sharpen;
    st->tune.get_dpc = get_dpc;
    st->tune.set_dpc = set_dpc;
    st->tune.get_gamma = get_gamma;
    st->tune.set_gamma = set_gamma;
    st->tune.get_sat = get_sat;
    st->tune.set_sat = set_sat;
}

static FILE *open_tmp(char *path)
{
    FILE *f;
    int fd;

    memcpy(path, "/tmp/t_hisi_iq_XXXXXX", 22);
    fd = mkstemp(path);
    assert(fd >= 0);
    f = fdopen(fd, "w");
    assert(f);
    return f;
}

/* ================================================================
 * LOAD 1 -- the well-formed file
 * ================================================================ */

static void write_ini_good(char *path)
{
    FILE *f = open_tmp(path);
    int i;

    fputs("[all_param]\n"
          "UpFrameIso = 400\n"
          "[static_ae]\n"
          "MaxHistOffset = \"32\" ; comment after a quoted value\n"
          "AutoSpeed = 64\n"
          "AutoTolerance = 2\n"
          "AutoBlackDelayFrame = 8\n"
          "[static_aerouteex]\n"
          "TotalNum = \"3\"\n"
          "RouteEXIntTime = \"  32, 20000, 40000\"\n"
          "RouteEXAGain = \"1024, 2048, 15872,\"\n" /* trailing comma */
          "[static_aeweight]\n"
          "ExpWeight_0 = 1,2,3,4,5,6,7,8,9,1,1,1,1,1,1,1,7,\n"
          "expweight_14 = 9,9\n" /* lowercase key */
          "[static_ldci]\t\n"    /* trailing tab on the header */
          "Enable = \"1\"\n"
          "LDCIGaussLPFSigma = \"28\"\n"
          "AutoHePosWgt = \"20, 21, 30\"\n"
          "[static_drc]\n"
          "Enable = \"1\"\n"
          "DRCOpType = \"1\"\n"
          "DRCAutoStr = \"512\"\n"
          "DRCToneMappingValue = \\\n",
          f);
    /* The whole 200-node curve, continued every 16 nodes. A partial one is
     * refused now, which load 2 checks. */
    for (i = 0; i < V4_ISP_DRC_TM_NODES; i++)
        fprintf(f, "%d%s", i * 20,
                i == V4_ISP_DRC_TM_NODES - 1 ? "\n" : (i % 16 == 15 ? ", \\\n" : ", "));
    fputs("[static_nr]\n"
          "Enable = \"1\"\n"
          "FineStr = \"64, 72\"\n"
          "[static_dehaze]  \n"
          "Enable = \"1\"\n"
          "DehazeUserLutEnable = \"1\"\n"
          "DehazeLut = \\\n",
          f);
    for (i = 0; i < V4_ISP_DEHAZE_LUT; i++)
        fprintf(f, "%d%s", i, i == V4_ISP_DEHAZE_LUT - 1 ? "\n" : (i % 20 == 19 ? ",\\\n" : ","));
    fputs("[dynamic_dehaze]\n"
          "ExpThreshCnt = \"2\"\n" /* the shipped file's off-by-one: noted, not obeyed */
          "IsoThresh = \"100, 200, 400\"\n"
          "AutoDehazeStr = \"58,65,90\"\n"
          "[dynamic_linear_drc]\n"
          "IsoCnt = \"3\"\n"
          "IsoLevel = \"100, 200, 400\"\n"
          "Strength = \"420, 380, 100\"\n"
          "Asymmetry = \"4, 4, 6\"\n"
          "DetailAdjustFactor = \"8, 8, -4\"\n"
          "[static_sharpen]\n"
          "Enable = \"1\"\n"
          "AutoLumaWgt_2 = \"31, 30\"\n"
          "AutoOverShoot = \"90, 80\"\n"
          "[static_dpc]\n"
          "DpcEnable = \"1\"\n"
          "DpcStrength = \"50, 100\"\n"
          "[static_saturation]\n"
          "AutoSat = \"128, 122, 120\"\n" /* a short table: the rest stay the driver's */
          "[static_3dnr]\n"
          "3DnrParam_0 = \\\n"
          "-nXsf1 18: 0:128 | 20: 0:128 \\\n"
          " -ref 0\n"
          "[ir_static_ae]\n"
          "AutoSpeed = 250\n" /* must NOT override the day value */
          "[dynamic_gamma]\n"
          "TotalNum = \"2\"\n"
          "Interval = \"4\"\n"
          "gammaExpThreshLtoH = \"3200, 6400\"\n" /* read, not used */
          "gammaExpThreshHtoL = \"400000, 800000\"\n"
          "Table_0 = \\\n",
          f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i % 4096,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fputs("Table_1 = \\\n", f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i / 2,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fputs("[static_3dnr]\n"
          "3DNRCount            = \"2\"\n"
          "IsoThresh            = \"100, 400\"\n"
          "\n"
          ";ISO 100\n"
          "3DnrParam_0 = \\\n",
          f);
    fputs(NRX_ROWS_A, f);
    fputs("\n;ISO 400\n3DnrParam_1 = \\\n", f);
    fputs(NRX_ROWS_B, f);
    fclose(f);
}

static void load_good(void)
{
    static hisi_state_t st;
    char path[32];

    reset_all(&st);
    write_ini_good(path);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);

    hisi_isp_note_frame(&st);

    /* [static_ae] -- and the ir_ mirror must not have overwritten it */
    assert(g_exp.auto_attr.max_hist_offset == 32);
    assert(g_exp.auto_attr.speed == 64);
    assert(g_exp.auto_attr.tolerance == 2);
    assert(g_exp.auto_attr.black_delay_frame == 8);
    /* implied by the route section's presence */
    assert(g_exp.route_ex_valid == 1);

    /* [static_aerouteex] */
    assert(g_route.total_num == 3);
    assert(g_route.node[0].int_time == 32);
    assert(g_route.node[2].int_time == 40000);
    assert(g_route.node[2].again == 15872);

    /* [static_aeweight]: row keys, case-insensitive */
    assert(g_stat.weight[0][0] == 1);
    assert(g_stat.weight[0][8] == 9);
    assert(g_stat.weight[0][16] == 7);
    assert(g_stat.weight[14][0] == 9);

    /* [static_ldci], trailing tab on the section header */
    assert(g_ldci.enable == 1);
    assert(g_ldci.gauss_lpf_sigma == 28);
    assert(g_ldci.auto_he[1].pos.wgt == 21);

    /* [static_drc] + [dynamic_linear_drc] at AE's ISO 100; the whole
     * 200-node tone-mapping curve arrives through its continuations */
    assert(g_drc.enable == 1);
    assert(g_drc.op_type == 1);
    assert(g_drc.tone_mapping[0] == 0);
    assert(g_drc.tone_mapping[7] == 140);
    assert(g_drc.tone_mapping[V4_ISP_DRC_TM_NODES - 1] == (V4_ISP_DRC_TM_NODES - 1) * 20);
    /* DRCOpType = 1 is manual, so the column's Strength lands in the
     * manual field the driver reads and the static DRCAutoStr stays where
     * it was put -- the dead-letter bug the engine fixed */
    assert(g_drc.auto_strength == 512);
    assert(g_drc.manual_strength == 420);
    assert(g_drc.asym.asymmetry == 4);
    assert(g_drc.detail_adjust_factor == 8);
    assert(log_count("[dynamic_linear_drc] 3 columns, ISO 100..400; AE at ISO 100, strength 420; "
                     "tracking ISO") == 1);

    /* [static_nr] */
    assert(g_nr.enable == 1);
    assert(g_nr.auto_fine_str[1] == 72);

    /* [static_dehaze] + [dynamic_dehaze] at ISO 100, whole LUT */
    assert(g_dehaze.enable == 1);
    assert(g_dehaze.user_lut_enable == 1);
    assert(g_dehaze.op_type == 0);
    assert(g_dehaze.auto_strength == 58);
    assert(log_count("[dynamic_dehaze] 3 columns, ISO 100..400; AE at ISO 100, strength 58; "
                     "tracking ISO") == 1);
    assert(g_dehaze.lut[0] == 0);
    assert(g_dehaze.lut[V4_ISP_DEHAZE_LUT - 1] == V4_ISP_DEHAZE_LUT - 1);

    /* [static_sharpen] */
    assert(g_sharpen.enable == 1);
    assert(g_sharpen.auto_attr.luma_wgt[2][1] == 30);
    assert(g_sharpen.auto_attr.over_shoot[0] == 90);

    /* [static_dpc] */
    assert(g_dpc.enable == 1);
    assert(g_dpc.auto_strength[1] == 100);

    /* [static_saturation]: the per-ISO table, op type left as fetched */
    assert(g_sat.auto_sat[0] == 128 && g_sat.auto_sat[1] == 122 && g_sat.auto_sat[2] == 120);
    assert(g_sat.auto_sat[3] == 0 && g_sat.op_type == 0);

    /* [dynamic_gamma]: exposure 1000 is the first band, so Table_0, all
     * 1025 nodes through the continuations, straight in */
    assert(g_gamma.enable == 1);
    assert(g_gamma.curve_type == V4_ISP_GAMMA_CURVE_USER);
    assert(g_gamma.table[0] == 0);
    assert(g_gamma.table[1024] == 1024 % 4096);
    assert(g_gamma.table[512] == 512);
    assert(log_count("[dynamic_gamma] 2 tables; AE at exposure 1000, table 0 written; tracking "
                     "exposure") == 1);

    /* one Set per touched module -- ten static, three dynamic -- and
     * only touched modules; with the 3DNR ladder that is fourteen. A
     * well-formed file trips none of the refusals load 2 is about */
    assert(g_sets == 13);
    assert(log_count("14 modules applied") == 1);
    assert(log_count("truncated") == 0);
    assert(log_count("not applied") == 0);
    assert(log_count("module skipped") == 0);

    /* [static_3dnr]: two rungs, and the driver takes the whole ladder, so
     * the write is AUTO and block 0 is still the ISO 100 rung */
    {
        const v4_vpss_nrx_v3 *b = &g_nrx_blk[0];

        assert(g_nrx_get_calls == 1);
        assert(g_nrx_set_calls == 1);
        assert(g_nrx_mode == V4_OPERATION_MODE_AUTO);
        assert(g_nrx_num == 2 && g_nrx_iso[0] == 100 && g_nrx_iso[1] == 400);
        assert(log_count("2 rungs, ISO 100..400; the driver has the ladder "
                         "and selects per frame (AUTO)") == 1);

        /* the sample's mapping, row by row, on the ISO 100 block */
        assert(b->sfy[0].sfs1 == 18 && b->sfy[0].sft1 == 0 && b->sfy[0].sbr1 == 128);
        assert(b->sfy[1].sfs2 == 30 && b->sfy[3].sfs4 == 30);
        assert(b->sfy[0].srt0 == 16 && b->sfy[0].srt1 == 16);  /* -SelRt */
        assert(b->sfy[2].kmode == 1 && b->sfy[3].kmode == 1);  /* -kmode, embedded */
        assert(b->sfy[0].derate == 0 && b->sfy[0].deidx == 4); /* -DeRt */
        assert(b->sfy[4].sfs1 == 60 && b->sfy[4].sfs2 == 60 && b->sfy[4].sfs4 == 60);  /* -sfs5 */
        assert(b->iey[3].ies0 == 110 && b->iey[3].ies1 == 90 && b->iey[4].ies0 == 96); /* -nXsf5 */
        assert(b->iey[2].iedz == 0);                                                   /* -dzsf5 */
        assert(b->sfy[1].spn6 == 4 && b->sfy[1].sbn6 == 2 && b->sfy[1].pbr6 == 0 &&
               b->sfy[1].jmode == 4);                                              /* -nXsf6 */
        assert(b->sfy[1].sfr6[2] == 8 && b->sfy[3].sfr6[0] == 10);                 /* -nXsfr6 */
        assert(b->sfy[0].sbr6[0] == 15 && b->sfy[3].sbr6[1] == 15);                /* -nXsbr6 */
        assert(b->sfy[0].sfn0 == 1 && b->sfy[0].sfn1 == 2 && b->sfy[0].sfn2 == 4); /* -nXsfn */
        assert(b->sfy[0].sth1 == 20 && b->sfy[0].sth2 == 40 && b->sfy[4].sthd2 == 30);
        assert(b->sfy[0].sfr == 31 && b->sfy[4].sfr == 31); /* -sfr (0) */
        assert(b->tfy[0].bref == 0 && b->tfy[1].bref == 1); /* -ref */
        assert(b->tfy[1].ted == 0 && b->tfy[2].ted == 0);   /* -tedge */
        assert(b->mdy[1].math1 == 90);                      /* -mXmath, embedded */
        assert(b->mdy[1].mathd1 == 60);                     /* -mXmathd, embedded */
        assert(b->tfy[0].str0 == 31 && b->tfy[1].str0 == 31 && b->tfy[1].str1 == 31 &&
               b->tfy[2].str0 == 31);                       /* -nXstr (1) */
        assert(b->mdy[1].mate1 == 2);                       /* -mXmate, embedded */
        assert(b->mdy[1].mabw1 == 5);                       /* -mXmabw, embedded */
        assert(b->tfy[0].tsi0 == 1 && b->tfy[1].tsi1 == 1); /* -nXtsi */
        assert(b->tfy[0].tfs0 == 0 && b->tfy[1].tfs0 == 7 && b->tfy[1].tfs1 == 11 &&
               b->tfy[2].tfs0 == 10);                       /* -nXtfs */
        assert(b->tfy[0].tdx0 == 2 && b->tfy[1].tdx1 == 2); /* -nXtdx; -mode ignored */
        assert(b->tfy[0].tfrs == 15);                       /* -nXtfrs; -presfc ignored */
        assert(b->tfy[0].tfr0[0] == 16 && b->tfy[0].tfr0[1] == 8 && b->tfy[0].tfr0[2] == 16 &&
               b->tfy[0].tfr0[3] == 8 && b->tfy[0].tfr0[4] == 0 && b->tfy[0].tfr0[5] == 0);
        assert(b->tfy[1].tfr0[0] == 8 && b->tfy[1].tfr0[1] == 4 && b->tfy[1].tfr0[3] == 0);
        assert(b->tfy[2].tfr0[0] == 16 && b->tfy[2].tfr0[3] == 8); /* -nXtfr0, two lines */
        assert(b->nrc.sfc == 60 && b->nrc.tfc == 10);              /* embedded in tfr0 */
        assert(b->tfy[1].tfr1[0] == 16 && b->tfy[1].tfr1[1] == 8 && b->tfy[1].tfr1[2] == 16 &&
               b->tfy[1].tfr1[3] == 8 && b->tfy[1].tfr1[4] == 0); /* -nXtfr1 */
        assert(b->nrc.tpc == 10 && b->nrc.trc == 12);             /* embedded in tfr1 */
        assert(b->mdy[0].mai00 == 1 && b->mdy[0].mai01 == 1 && b->mdy[0].mai02 == 2 &&
               b->mdy[1].mai00 == 1 && b->mdy[1].mai02 == 2);                         /* -mXid0 */
        assert(b->mdy[0].mai10 == 2 && b->mdy[0].mai11 == 2 && b->mdy[0].mai12 == 2); /* -mXid1 */
        assert(b->mdy[0].mabr0 == 0 && b->mdy[0].mabr1 == 0 && b->mdy[1].mabr0 == 0);
        assert(b->mdy[0].advmath == 1 && b->mdy[0].advth == 0);
        assert(b->mdy[0].math0 == 40 && b->mdy[0].math1 == 150 && b->mdy[1].math0 == 150);
        assert(b->mdy[0].mathd0 == 20 && b->mdy[0].mathd1 == 120 && b->mdy[1].mathd0 == 100);
        assert(b->mdy[0].mate0 == 2 && b->mdy[0].mate1 == 2 && b->mdy[1].mate0 == 2);
        assert(b->mdy[0].mabw0 == 4 && b->mdy[0].mabw1 == 9 && b->mdy[1].mabw0 == 5);
        assert(b->mdy[0].matw == 3 && b->mdy[1].matw == 3);

        /* get-modify-set: what the text never names is the driver's */
        assert(b->sfy[0].sbsk[0] == 1234 && b->sfy[4].sdsk[31] == 4321);
        assert(b->sfy[2].nry_en == 1 && b->iey[1].ie_en == 1);
        assert(b->mdy[0].madz0 == 300 && b->tfy[0].dzmode0 == 1);
        /* column 4's SFT/SBR are not in the text either */
        assert(b->sfy[4].sft1 == 0 && b->sfy[4].sbr1 == 0);

    }

    /* the latch is one-shot: a second frame reloads nothing, and the tick
     * behind it is not due */
    {
        int sets = g_sets;

        hisi_isp_note_frame(&st);
        assert(g_sets == sets);
    }

    unlink(path);
}

/*
 * The MANUAL fall-back, on a driver that will not take the ladder. That
 * was every board until the NRc offsets were corrected -- an AUTO write
 * came back 0xa0078003 -- and it stays the path for any driver too old
 * for the whole ladder, so the per-ISO engine behind it is live code and
 * is tested here. Load 1's file, verbatim; only the vendor's answer to
 * the AUTO write differs.
 */
static void load_nrx_manual(void)
{
    static hisi_state_t st;
    const v4_vpss_nrx_v3 *b = &g_nrx_blk[0];
    char path[32];

    reset_all(&st);
    g_nrx_auto_refuse = true;
    write_ini_good(path);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);

    hisi_isp_note_frame(&st);

    /* the refused ladder, then the rung AE asks for */
    assert(g_nrx_set_calls == 2);
    assert(g_nrx_mode == V4_OPERATION_MODE_MANUAL);
    assert(log_count("refused the ladder (AUTO): 0xa0078003; raptor selects the rung "
                     "instead") == 1);
    assert(log_count("2 rungs, ISO 100..400; AE at ISO 100, rung written") == 1);
    assert(b->nrc.sfc == 60);

    /* the tick: ISO 200 sits halfway between the rungs in stops, so the
     * blend is the sample's rounding midpoint of 60/61 and 7/9, 11/13 */
    g_iso = 200;
    hisi_nrx_on_iso(&st, g_iso);
    assert(g_nrx_set_calls == 3);
    assert(b->nrc.sfc == 61);
    assert(b->tfy[1].tfs0 == 8 && b->tfy[1].tfs1 == 12);
    assert(b->sfy[0].sfs1 == 18 && b->sfy[0].sbsk[0] == 1234); /* unchanged, and kept */

    /* the second rung, where the fixture changed it and only there */
    g_iso = 400;
    hisi_nrx_on_iso(&st, g_iso);
    assert(g_nrx_set_calls == 4);
    assert(b->nrc.sfc == 61);
    assert(b->tfy[1].tfs0 == 9 && b->tfy[1].tfs1 == 13);
    assert(b->sfy[0].sfs1 == 18 && b->mdy[0].math1 == 150);

    /* past the top rung is the top rung; the same step is no write */
    g_iso = 50000;
    hisi_nrx_on_iso(&st, g_iso);
    assert(g_nrx_set_calls == 5 && b->nrc.sfc == 61 && b->tfy[1].tfs0 == 9);
    hisi_nrx_on_iso(&st, g_iso);
    assert(g_nrx_set_calls == 5);
    g_iso = 50500; /* same MapISO step */
    hisi_nrx_on_iso(&st, g_iso);
    assert(g_nrx_set_calls == 5);

    /* and back down to the first */
    g_iso = 100;
    hisi_nrx_on_iso(&st, g_iso);
    assert(g_nrx_set_calls == 6 && b->nrc.sfc == 60 && b->tfy[1].tfs0 == 7);

    /* the clocked entry point is armed and rate-limited: two calls in
     * the same second are one query at most, and a stopped engine is
     * silent. The same tick feeds the ISP engines: ISO 400 is their
     * top column too. */
    g_iso = 400;
    hisi_dyn_tick(&st);
    hisi_dyn_tick(&st);
    assert(g_nrx_set_calls == 7);
    assert(g_drc.manual_strength == 100 && g_drc.detail_adjust_factor == -4);
    assert(g_dehaze.auto_strength == 90);
    assert(log_count("drc: ISO 100 -> 400, now on column 2 (ISO 400); strength 100") == 1);
    assert(log_count("dehaze: ISO 100 -> 400, now on column 2 (ISO 400); strength 90") == 1);
    g_nrx_set_fail = true;
    g_iso = 100;
    hisi_nrx_on_iso(&st, g_iso);
    hisi_nrx_on_iso(&st, g_iso);
    hisi_nrx_on_iso(&st, g_iso);
    assert(log_count("failed three times running") == 1);
    assert(st.nrx->engine == 0);
    g_nrx_set_fail = false;
    st.iso_tick_ns = 0;
    hisi_dyn_tick(&st);
    assert(g_nrx_set_calls == 10); /* the three failed writes, none after */
    /* ...while the ISP engines, untroubled, followed AE back down */
    assert(g_drc.manual_strength == 420 && g_dehaze.auto_strength == 58);

    unlink(path);
}

/* ================================================================
 * LOAD 2 -- malformed input
 * ================================================================ */

/* `lead`, spaces, then `tail`, on a line of exactly `total` bytes. */
static void put_padded(FILE *f, const char *lead, const char *tail, int total)
{
    int pad = total - (int)strlen(lead) - (int)strlen(tail);
    int i;

    assert(pad > 0);
    fputs(lead, f);
    for (i = 0; i < pad; i++)
        fputc(' ', f);
    fputs(tail, f);
    fputc('\n', f);
}

static void write_ini_bad(char *path)
{
    FILE *f = open_tmp(path);
    int i;

    /* Two headers with no ']'. Neither may keep the previous section
     * alive, and the warning is once-only. */
    fputs("[static_ae\n"
          "MaxHistOffset = 32\n"
          "AutoSpeed = 64\n"
          "[static_drc\n"
          "Enable = 1\n",
          f);

    /* strtol saturation in both directions, then a value that is fine */
    fprintf(f,
            "[static_nr]\n"
            "Enable = \"1\"\n"
            "FineStr = 99999999999999999999999, -99999999999999999999999, 72\n"
            "CoringWgt = %ld, %ld, 8\n",
            LONG_MAX, LONG_MIN);

    /* A physical line past HISI_IQ_LINE_MAX: the tail ("4242") is dropped
     * and the remainder consumed, so the *next* key still parses. Then one
     * exactly at the limit, which must survive whole. */
    fputs("[static_dpc]\n", f);
    put_padded(f, "DpcStrength = 50, 100,", "4242", HISI_IQ_LINE_MAX + 200);
    put_padded(f, "DpcBlendRatio = 5,", "6", HISI_IQ_LINE_MAX - 1);

    /* Short whole-tables: both must be refused rather than spliced onto
     * whatever the Get returned. The Enable beside each still applies. */
    fputs("[static_drc]\n"
          "Enable = \"1\"\n"
          "DRCToneMappingValue = \"1, 2, 3\"\n"
          "[static_dehaze]\n"
          "Enable = \"1\"\n"
          "DehazeLut = \"1, 2, 3\"\n",
          f);

    /* An assembled value past HISI_IQ_VAL_MAX. Every node is in the file,
     * but the value is cut mid-table, so the node count comes up short and
     * the module is skipped -- which is why the cap wants enough headroom
     * that a real file never reaches it. */
    fputs("[dynamic_gamma]\nTable_0 = \\\n", f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "4095,                    %s",
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 15 == 14 ? "\\\n" : ""));

    /* Last of all: a value still asking for a continuation at EOF, with no
     * newline after it either. */
    fputs("[static_ldci]\n"
          "Enable = \"1\"\n"
          "LDCIGaussLPFSigma = 28,\\",
          f);
    fclose(f);
}

static void load_bad(void)
{
    static hisi_state_t st;
    char path[32];

    reset_all(&st);
    write_ini_bad(path);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);

    hisi_isp_apply_tuning(&st);

    /* Unterminated headers: warned once, and their keys went nowhere --
     * neither to the module named nor to the previous section. */
    assert(log_count("unterminated section header") == 1);
    assert(g_exp.auto_attr.max_hist_offset == 0);
    assert(g_exp.auto_attr.speed == 0);
    assert(g_stat.weight[0][0] == 0);

    /* strtol saturation is clamped like any other out-of-range value */
    assert(g_nr.enable == 1);
    assert(g_nr.auto_fine_str[0] == 0x80); /* LONG_MAX -> the field's max */
    assert(g_nr.auto_fine_str[1] == 0);    /* LONG_MIN -> the field's min */
    assert(g_nr.auto_fine_str[2] == 72);
    assert(g_nr.auto_coring_wgt[0] == 0xC80);
    assert(g_nr.auto_coring_wgt[1] == 0);
    assert(g_nr.auto_coring_wgt[2] == 8);

    /* The over-length line lost its tail and nothing else */
    assert(log_count("line over") == 1);
    assert(g_dpc.auto_strength[0] == 50);
    assert(g_dpc.auto_strength[1] == 100);
    assert(g_dpc.auto_strength[2] == 0); /* the 4242 past the cap never landed */
    /* ...and the next key, which lives past the discarded remainder,
     * parsed normally at exactly HISI_IQ_LINE_MAX - 1 bytes */
    assert(g_dpc.auto_blend_ratio[0] == 5);
    assert(g_dpc.auto_blend_ratio[1] == 6);

    /* Short whole-tables refused, their sections otherwise applied */
    assert(log_count("DRCToneMappingValue has 3 of 200 nodes") == 1);
    assert(log_count("DehazeLut has 3 of 256 entries") == 1);
    assert(g_drc.enable == 1);
    assert(g_drc.tone_mapping[0] == 0);
    assert(g_dehaze.enable == 1);
    assert(g_dehaze.lut[0] == 0);

    /* Value past HISI_IQ_VAL_MAX -> short gamma table -> module skipped,
     * so nothing switched the curve type to USER over a stale tail */
    assert(log_count("value of Table_0 over") == 1);
    assert(log_count("module skipped") == 1);
    assert(g_gamma.enable == 0);
    assert(g_gamma.curve_type == 0);
    assert(g_gamma.table[0] == 0);

    /* The dangling backslash at EOF ends the value instead of running off */
    assert(g_ldci.enable == 1);
    assert(g_ldci.gauss_lpf_sigma == 28);

    /* nr, dpc, drc, dehaze, ldci -- and nothing else */
    assert(g_sets == 5);

    unlink(path);
}

/* ================================================================
 * LOAD 3 -- the vendor calls failing
 * ================================================================ */

static void load_vendor_failures(void)
{
    static hisi_state_t st;
    char path[32];

    reset_all(&st);
    write_ini_good(path);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);

    g_get_fail[M_GAMMA] = true; /* Get errors -> the section is skipped */
    g_set_fail[M_EXP] = true;   /* Set errors -> the module keeps defaults */
    st.tune.get_ldci = NULL;    /* symbol never resolved, on the Get side */
    st.tune.set_dpc = NULL;     /* ...and on the Set side */

    hisi_isp_apply_tuning(&st);

    assert(log_count("GetGammaAttr failed") == 1);
    assert(log_count("GetLDCIAttr unresolved") == 1);
    assert(log_count("SetExposureAttr failed") == 1);
    assert(log_count("SetDPDynamicAttr unresolved") == 1);

    /* A failed Get takes its whole module out: no Set is even attempted,
     * and the struct the ISP would have been handed stays untouched. */
    assert(g_gamma.enable == 0);
    assert(g_gamma.curve_type == 0);
    assert(g_ldci.enable == 0);
    assert(g_ldci.gauss_lpf_sigma == 0);
    /* A failed Set is reached but stores nothing */
    assert(g_exp.auto_attr.speed == 0);
    assert(g_dpc.enable == 0);

    /* Nine static modules were fetched and dirtied; dpc has no Set to
     * call, so eight are attempted, of which exp fails. Then the engines:
     * DRC and dehaze write, gamma's Get fails before its Set. With the
     * 3DNR ladder that is ten applied, and exp, dpc, gamma failed. */
    assert(g_set_calls == 10);
    assert(g_sets == 9);
    assert(log_count("10 modules applied, 3 failed") == 1);
    assert(log_count("skipped: all_param ir_* dynamic_gamma(Get failed)") == 1);

    /* The undisturbed ones still went through */
    assert(g_nr.enable == 1);
    assert(g_sharpen.enable == 1);
    assert(g_drc.enable == 1);

    unlink(path);
}

/* ================================================================
 * LOAD 4 -- [static_3dnr] edges: a short block, a bad ladder, AUTO refused
 * ================================================================ */

static void write_ini_nrx(char *path, bool short_second, const char *iso, bool no_count)
{
    FILE *f = open_tmp(path);

    fputs("[static_3dnr]\n", f);
    if (!no_count)
        fputs("3DNRCount = \"2\"\n", f);
    fprintf(f, "IsoThresh = \"%s\"\n", iso);
    fputs("3DnrParam_0 = \\\n", f);
    fputs(NRX_ROWS_A, f);
    fputs("3DnrParam_1 = \\\n", f);
    fputs(short_second ? NRX_ROWS_SHORT : NRX_ROWS_B, f);
    fclose(f);
}

static void load_nrx_edges(void)
{
    static hisi_state_t st;
    char path[32];

    /* A second rung one row short: skipped by name, and the ladder the
     * driver is handed shortens to the one rung that parsed. */
    reset_all(&st);
    g_iso = 400;
    write_ini_nrx(path, true, "100, 400", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(log_count("3DnrParam_1: 1 rows short, first missing -mXmatw; block skipped") == 1);
    assert(log_count("3DNRCount is 2 but 3DnrParam_1 is missing or malformed; using the 1") == 1);
    assert(g_nrx_mode == V4_OPERATION_MODE_AUTO && g_nrx_set_calls == 1);
    assert(g_nrx_num == 1 && g_nrx_iso[0] == 100);
    assert(g_nrx_blk[0].nrc.sfc == 60);
    assert(log_count("1 rungs, ISO 100..100; the driver has the ladder") == 1);
    unlink(path);

    /* A ladder that is not ascending is cut at the fault. On the MANUAL
     * fall-back, with no way to ask AE, the first rung goes in once and
     * the engine stays off. */
    reset_all(&st);
    g_nrx_auto_refuse = true;
    st.tune.query_exp = NULL;
    write_ini_nrx(path, false, "400, 100", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(log_count("IsoThresh not ascending at entry 1 (100 after 400); truncating to 1") == 1);
    assert(g_nrx_set_calls == 2); /* the refused ladder, then the rung */
    assert(g_nrx_mode == V4_OPERATION_MODE_MANUAL);
    assert(g_nrx_blk[0].nrc.sfc == 60 && g_nrx_blk[0].sfy[0].sbsk[0] == 1234);
    assert(log_count("no ISO query") == 1);
    assert(st.nrx->engine == 0);
    st.iso_tick_ns = 0;
    hisi_dyn_tick(&st);
    assert(g_nrx_set_calls == 2);
    unlink(path);

    /* The write itself failing counts as a failed module. */
    reset_all(&st);
    g_nrx_set_fail = true;
    write_ini_nrx(path, false, "100, 400", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(log_count("skipped: static_3dnr(Set failed)") == 1);
    unlink(path);

    /* No 3DNRCount at all: nothing is written, and the summary does not
     * count a failure either -- the section was never complete. */
    reset_all(&st);
    write_ini_nrx(path, false, "100, 400", true);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_nrx_set_calls == 0 && g_nrx_get_calls == 0);
    unlink(path);

    /* The Get half failing keeps the driver's defaults and counts as a
     * failed module. */
    reset_all(&st);
    g_nrx_get_fail = true;
    write_ini_nrx(path, false, "100, 400", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_nrx_set_calls == 0);
    assert(log_count("GetGrpNRXParam(0) failed") == 1);
    assert(log_count("skipped: static_3dnr(Get failed)") == 1);
    unlink(path);

    hisi_nrx_free(&st);
    hisi_dyn_free(&st);
}

/* ================================================================
 * LOAD 5 -- the dynamic ISP sections: blends, the gamma fade, the stops
 * ================================================================ */

static void write_ini_dyn(char *path, const char *iso_level, const char *strength)
{
    FILE *f = open_tmp(path);
    int i;

    fprintf(f,
            "[static_drc]\n"
            "DRCOpType = \"0\"\n" /* auto: Strength goes to the auto field */
            "[dynamic_linear_drc]\n"
            "Enable = \"1\"\n"
            "IsoLevel = \"%s\"\n"
            "Strength = \"%s\"\n"
            "DetailAdjustFactor = \"8, 8, -4\"\n"
            "[dynamic_dehaze]\n"
            "IsoThresh = \"100, 200, 400\"\n"
            "AutoDehazeStr = \"58, 65, 90\"\n"
            "[dynamic_gamma]\n"
            "TotalNum = \"3\"\n"
            "Interval = \"4\"\n"
            "gammaExpThreshHtoL = \"400000, 800000, 1600000\"\n",
            iso_level, strength);
    fputs("Table_0 = \\\n", f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i % 4096,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fputs("Table_1 = \\\n", f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i / 2,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fputs("Table_2 = \\\n", f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i / 4,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fclose(f);
}

static void load_dyn(void)
{
    static hisi_state_t st;
    char path[32];
    int sets;

    /* The blends: the sample's rounding linear interpolation in ISO, the
     * end columns holding past the ends, and no write for an ISO on the
     * same MapISO step. */
    reset_all(&st);
    write_ini_dyn(path, "100, 200, 400", "420, 380, 100");
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_drc.enable == 1 && g_drc.op_type == 0);
    assert(g_drc.auto_strength == 420 && g_drc.manual_strength == 0);
    assert(g_dehaze.auto_strength == 58);
    assert(g_gamma.table[1024] == 1024 && g_gamma.curve_type == V4_ISP_GAMMA_CURVE_USER);
    assert(g_sets == 4 && st.dyn->engine == 1); /* static_drc, and the three engines */
    assert(log_count("4 modules applied") == 1);

    hisi_dyn_on_exposure(&st, 150, 1500); /* halfway in ISO between the first two */
    assert(g_drc.auto_strength == 400 && g_drc.detail_adjust_factor == 8);
    assert(g_dehaze.auto_strength == 62);
    assert(g_sets == 6);
    hisi_dyn_on_exposure(&st, 300, 3000); /* halfway between the last two; signed row */
    assert(g_drc.auto_strength == 240 && g_drc.detail_adjust_factor == 2);
    assert(g_dehaze.auto_strength == 78);
    assert(log_count("drc: ISO 150 -> 300, now below column 2 (ISO 400); strength 240") == 1);
    sets = g_sets;
    hisi_dyn_on_exposure(&st, 305, 3050); /* same step: nothing */
    assert(g_sets == sets);
    hisi_dyn_on_exposure(&st, 50000, 500); /* past the top: the top column */
    assert(g_drc.auto_strength == 100 && g_drc.detail_adjust_factor == -4);
    assert(g_dehaze.auto_strength == 90);

    /* The gamma fade: exposure into the second band starts a four-step
     * fade from the curve on the wire toward Table_1, one step per call;
     * the last step lands exactly. A retarget mid-fade fades from where
     * the curve got to. */
    assert(g_gamma.table[1024] == 1024);
    hisi_dyn_on_exposure(&st, 50000, 500000);
    assert(log_count("gamma: exposure 500 -> 500000, table 0 -> 1, fading over 4 steps") == 1);
    assert(g_gamma.table[1024] == 896 && g_gamma.table[0] == 0);
    assert(st.dyn->gamma.fade_i == 1);
    hisi_dyn_on_exposure(&st, 50000, 500000);
    assert(g_gamma.table[1024] == 768);
    hisi_dyn_on_exposure(&st, 50000, 500000);
    assert(g_gamma.table[1024] == 640);
    hisi_dyn_on_exposure(&st, 50000, 500000);
    assert(g_gamma.table[1024] == 512 && st.dyn->gamma.fade_i == 0);
    sets = g_sets;
    hisi_dyn_on_exposure(&st, 50000, 500000); /* settled: no write */
    assert(g_sets == sets);
    hisi_dyn_on_exposure(&st, 50000, 2000000); /* the third band */
    assert(g_gamma.table[1024] == 448);
    hisi_dyn_on_exposure(&st, 50000, 1000); /* back to the first, mid-fade */
    assert(log_count("table 2 -> 0, fading over 4 steps") == 1);
    assert(g_gamma.table[1024] == 592); /* 448 + (1024 - 448) / 4 */
    /* the tick runs the fade at 100 ms rather than a second */
    st.iso_tick_ns = 0;
    g_iso = 50000;
    g_exp_time = 2; /* exposure 1000 */
    hisi_dyn_tick(&st);
    assert(g_gamma.table[1024] == 736);
    assert(st.iso_tick_ns > 0);
    {
        struct timespec ts;
        long long now;

        clock_gettime(CLOCK_MONOTONIC, &ts);
        now = (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
        assert(st.iso_tick_ns - now < 200000000LL);
    }
    hisi_dyn_on_exposure(&st, 50000, 1000);
    hisi_dyn_on_exposure(&st, 50000, 1000);
    assert(g_gamma.table[1024] == 1024 && st.dyn->gamma.fade_i == 0);

    /* Three failed writes running stop that engine and only that one. */
    g_set_fail[M_DRC] = true;
    hisi_dyn_on_exposure(&st, 100, 1000);
    hisi_dyn_on_exposure(&st, 200, 2000);
    hisi_dyn_on_exposure(&st, 400, 4000);
    assert(log_count("drc: HI_MPI_ISP_SetDRCAttr failed three times running") == 1);
    assert(st.dyn->drc.engine == 0 && st.dyn->dehaze.engine == 1 && st.dyn->engine == 1);
    assert(g_dehaze.auto_strength == 90);
    g_set_fail[M_DRC] = false;
    sets = g_sets;
    hisi_dyn_on_exposure(&st, 100, 1000);
    assert(g_dehaze.auto_strength == 58 && g_drc.auto_strength == 100); /* DRC stayed */
    g_get_fail[M_GAMMA] = true;
    hisi_dyn_on_exposure(&st, 100, 500000);
    hisi_dyn_on_exposure(&st, 100, 500000);
    hisi_dyn_on_exposure(&st, 100, 500000);
    assert(log_count("gamma: HI_MPI_ISP_GetGammaAttr failed three times running") == 1);
    assert(st.dyn->gamma.engine == 0 && st.dyn->engine == 1);
    g_get_fail[M_GAMMA] = false;
    g_set_fail[M_DEHAZE] = true;
    hisi_dyn_on_exposure(&st, 200, 1000);
    hisi_dyn_on_exposure(&st, 400, 1000);
    hisi_dyn_on_exposure(&st, 800, 1000);
    assert(st.dyn->dehaze.engine == 0 && st.dyn->engine == 0);
    st.iso_tick_ns = 0;
    sets = g_sets;
    hisi_dyn_tick(&st); /* every engine stopped: the tick does not even ask */
    assert(g_sets == sets);
    unlink(path);

    /* No way to ask AE: the first column and the first table go in once,
     * nothing is armed. */
    reset_all(&st);
    st.tune.query_exp = NULL;
    write_ini_dyn(path, "100, 200, 400", "420, 380, 100");
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_drc.auto_strength == 420 && g_dehaze.auto_strength == 58);
    assert(g_gamma.table[1024] == 1024);
    assert(log_count("no ISO query, first column at ISO 100, strength 420") == 1);
    assert(log_count("no ISO query, first at exposure 0, table 0 written") == 1);
    assert(st.dyn->engine == 0);
    unlink(path);

    /* A ladder that is not ascending is cut at the fault; a row shorter
     * than the ladder shortens it. */
    reset_all(&st);
    write_ini_dyn(path, "100, 400, 200", "420, 380");
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(log_count("IsoLevel not ascending at entry 2 (200 after 400); truncating to 2") == 1);
    assert(log_count("Strength has 2 of 2 columns") == 0);
    assert(st.dyn->drc.n == 2);
    hisi_dyn_on_exposure(&st, 50000, 1000);
    assert(g_drc.auto_strength == 380 && g_drc.detail_adjust_factor == 8);
    unlink(path);

    reset_all(&st);
    write_ini_dyn(path, "100, 200, 400", "420");
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(log_count("Strength has 1 of 3 columns; using 1") == 1);
    assert(st.dyn->drc.n == 1 && st.dyn->drc.engine == 0);
    assert(st.dyn->dehaze.engine == 1); /* the other engines are unaffected */
    unlink(path);

    /* A load-time write failing counts as a failed module by name, and
     * the engines that did write still run. */
    reset_all(&st);
    g_set_fail[M_DEHAZE] = true;
    write_ini_dyn(path, "100, 200, 400", "420, 380, 100");
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(log_count("[dynamic_dehaze] SetDehazeAttr failed") == 1);
    assert(log_count("3 modules applied, 1 failed; skipped: dynamic_dehaze(Set failed)") == 1);
    assert(st.dyn->dehaze.engine == 0 && st.dyn->drc.engine == 1);
    unlink(path);

    /* No dynamic section at all: nothing allocated, nothing counted. */
    reset_all(&st);
    write_ini_nrx(path, false, "100, 400", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(st.dyn == NULL);
    assert(g_sets == 0);
    unlink(path);

    hisi_nrx_free(&st);
    hisi_dyn_free(&st);
}

/* ---------------- [module_state], the file's own enable mask ---------------- */

/*
 * The same body every time; only the mask and where it sits change. Each
 * section carries one field that is unmistakable once written, so an
 * assertion on the struct says whether the section ran at all -- which is
 * the whole question here.
 */
static void write_ini_state(char *path, const char *mask, bool at_end)
{
    FILE *f = open_tmp(path);
    int i;

    if (!at_end)
        fputs(mask, f);
    fputs("[static_ae]\n"
          "AutoSpeed = \"64\"\n"
          "[static_aerouteex]\n"
          "TotalNum = \"3\"\n"
          "RouteEXIntTime = \"32, 20000, 40000\"\n"
          "[static_aeweight]\n"
          "ExpWeight_0 = \"1, 2, 3\"\n"
          "[static_ldci]\n"
          "Enable = \"1\"\n"
          "[static_drc]\n"
          "Enable = \"1\"\n"
          "DRCAutoStr = \"512\"\n"
          "[static_nr]\n"
          "Enable = \"1\"\n"
          "FineStr = \"64, 72\"\n"
          "[static_dehaze]\n"
          "Enable = \"1\"\n"
          "[static_sharpen]\n"
          "Enable = \"1\"\n"
          "[static_dpc]\n"
          "DpcEnable = \"1\"\n"
          "[static_saturation]\n"
          "AutoSat = \"128, 122, 120\"\n"
          "[dynamic_gamma]\n"
          "TotalNum = \"2\"\n"
          "Interval = \"4\"\n"
          "gammaExpThreshHtoL = \"400000, 800000\"\n"
          "Table_0 = \\\n",
          f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i % 4096,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fputs("Table_1 = \\\n", f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i / 2,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fputs("[static_3dnr]\n"
          "3DNRCount = \"1\"\n"
          "IsoThresh = \"100\"\n"
          "3DnrParam_0 = \\\n",
          f);
    fputs(NRX_ROWS_A, f);
    fputs("\n", f);
    if (at_end)
        fputs(mask, f);
    fclose(f);
}

/* AE, its weight table, LDCI, NR, saturation, gamma and the 3DNR ladder
 * on; DRC, dehaze, sharpen and DPC off, with their sections present and
 * full. */
static const char MASK_MIXED[] = "[module_state]\n"
                                 "bStaticAE          = \"1\"\n"
                                 "bAeWeightTab       = \"1\"\n"
                                 "bStaticLdci        = \"1\"\n"
                                 "bStaticDRC         = \"0\"\n"
                                 "bStaticNr          = \"1\"\n"
                                 "bStaticDehaze      = \"0\"\n"
                                 "bStaticSharpen     = \"0\"\n"
                                 "bStaticDPC         = \"0\"\n"
                                 "bStaticSaturation  = \"1\"\n"
                                 "bDynamicGamma      = \"1\"\n"
                                 "bDyanamic3DNR      = \"1\"\n";

static void check_mixed(void)
{
    /* On. */
    assert(g_exp.auto_attr.speed == 64);
    assert(g_route.total_num == 3 && g_route.node[2].int_time == 40000);
    assert(g_exp.route_ex_valid == 1);
    assert(g_stat.weight[0][0] == 1 && g_stat.weight[0][2] == 3);
    assert(g_ldci.enable == 1);
    assert(g_nr.enable == 1 && g_nr.auto_fine_str[0] == 64);
    assert(g_sat.auto_sat[0] == 128 && g_sat.auto_sat[2] == 120);
    assert(g_gamma.table[1024] == 1024);
    assert(g_nrx_set_calls == 1 && g_nrx_mode == V4_OPERATION_MODE_AUTO);

    /* Off, and every one of them had a section in the file. */
    assert(g_drc.enable == 0 && g_drc.auto_strength == 0);
    assert(g_dehaze.enable == 0);
    assert(g_sharpen.enable == 0);
    assert(g_dpc.enable == 0);

    assert(log_count("[module_state] off: static_drc static_dehaze static_sharpen static_dpc") ==
           1);
    assert(log_count("8 modules applied") == 1);
}

static void load_module_state(void)
{
    static hisi_state_t st;
    char path[32];

    /* The mask where the vendor writes it. */
    reset_all(&st);
    write_ini_state(path, MASK_MIXED, false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    check_mixed();
    unlink(path);
    hisi_nrx_free(&st);
    hisi_dyn_free(&st);

    /* And the same mask after everything it governs, which only holds
     * because the mask is read in a pass of its own. */
    reset_all(&st);
    write_ini_state(path, MASK_MIXED, true);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    check_mixed();
    unlink(path);
    hisi_nrx_free(&st);
    hisi_dyn_free(&st);

    /* A section that names one flag has named its whole set: everything
     * it does not list is off, the vendor's zeroed-struct rule. */
    reset_all(&st);
    write_ini_state(path, "[module_state]\nbStaticNr = \"1\"\n", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_nr.enable == 1);
    assert(g_exp.auto_attr.speed == 0 && g_route.total_num == 0);
    assert(g_stat.weight[0][0] == 0 && g_ldci.enable == 0 && g_sat.auto_sat[0] == 0);
    assert(g_gamma.table[1024] == 0);
    assert(g_nrx_set_calls == 0);
    assert(log_count("1 modules applied") == 1);
    assert(log_count("[module_state] off: static_ae static_aerouteex static_aeweight "
                     "static_ldci static_drc static_dehaze static_sharpen static_dpc "
                     "static_saturation dynamic_gamma static_3dnr") == 1);
    unlink(path);
    hisi_nrx_free(&st);
    hisi_dyn_free(&st);

    /* bStaticAE alone carries the route with it -- the vendor writes both
     * in one function -- but not the weight table, which is nested behind
     * its own flag inside that same function. */
    reset_all(&st);
    write_ini_state(path, "[module_state]\nbStaticAE = \"1\"\n", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_exp.auto_attr.speed == 64 && g_route.total_num == 3);
    assert(g_stat.weight[0][0] == 0);
    assert(log_count("2 modules applied") == 1);
    unlink(path);
    hisi_nrx_free(&st);
    hisi_dyn_free(&st);

    /* And the nesting the other way: the table's own flag is not enough. */
    reset_all(&st);
    write_ini_state(path, "[module_state]\nbAeWeightTab = \"1\"\n", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_exp.auto_attr.speed == 0 && g_route.total_num == 0);
    assert(g_stat.weight[0][0] == 0);
    assert(log_count("0 modules applied") == 1);
    unlink(path);
    hisi_nrx_free(&st);
    hisi_dyn_free(&st);

    /* The ladder answers to the dynamic flag, spelled either way: the
     * vendor's bDyanamic3DNR above, and the correction below. bStatic3DNR
     * governs nothing, which is why a file may carry it as "0" and still
     * have a ladder. */
    reset_all(&st);
    write_ini_state(path, "[module_state]\nbStatic3DNR = \"0\"\nbDynamic3DNR = \"1\"\n", false);
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    hisi_isp_note_frame(&st);
    assert(g_nrx_set_calls == 1 && g_nrx_mode == V4_OPERATION_MODE_AUTO);
    assert(log_count("1 modules applied") == 1);
    unlink(path);
    hisi_nrx_free(&st);
    hisi_dyn_free(&st);
}

/* ---- the sensor rate: [sensor] fps over the mode INI's Isp_FrameRate ---- */

static v4_isp_pub_attr g_pub;
static int g_pub_gets, g_pub_sets, g_pub_set_fail;

static int get_pub(int pipe, v4_isp_pub_attr *p)
{
    (void)pipe;
    g_pub_gets++;
    *p = g_pub;
    return 0;
}

static int set_pub(int pipe, const v4_isp_pub_attr *p)
{
    (void)pipe;
    g_pub_sets++;
    if (g_pub_set_fail)
        return 0xa01c0004;
    g_pub = *p;
    return 0;
}

static void sensor_fps(void)
{
    static hisi_state_t st;
    rss_hal_ctx_t g_ctx;
    uint32_t num = 0, den = 0;

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.platform = &st;

    /* Before the 3A thread runs, the set is a note in the mode: nothing to
     * write yet, hisi_isp_bringup will build the attribute from it. */
    reset_all(&st);
    st.mode.frame_rate = 30;
    assert(hal_isp_set_sensor_fps(&g_ctx, 20, 1) == RSS_OK);
    assert(st.mode.frame_rate == 20 && g_pub_sets == 0);
    assert(log_count("sensor: 20 fps for bring-up (the mode INI said 30)") == 1);
    assert(hal_isp_get_sensor_fps(&g_ctx, &num, &den) == RSS_OK && num == 20 && den == 1);
    assert(hal_isp_set_sensor_fps(&g_ctx, 0, 1) == RSS_ERR_INVAL);
    assert(hal_isp_set_sensor_fps(&g_ctx, 1, 4) == RSS_ERR_INVAL);
    assert(st.mode.frame_rate == 20);

    /* Running: get-modify-set of the public attribute, the exact fraction
     * in the attribute and the rounded whole in the mode. */
    st.isp.fnGetPubAttr = get_pub;
    st.isp.fnSetPubAttr = set_pub;
    g_pub.frame_rate = 20.0f;
    g_pub.bayer = 3;
    st.isp_thread_running = 1;
    assert(hal_isp_set_sensor_fps(&g_ctx, 25, 1) == RSS_OK);
    assert(g_pub_gets == 1 && g_pub_sets == 1);
    assert(g_pub.frame_rate == 25.0f && g_pub.bayer == 3 && st.mode.frame_rate == 25);
    assert(log_count("sensor: 25/1 fps (the mode INI said 20)") == 1);
    assert(hal_isp_get_sensor_fps(&g_ctx, &num, &den) == RSS_OK && num == 25000 && den == 1000);
    assert(hal_isp_set_sensor_fps(&g_ctx, 25, 1) == RSS_OK && g_pub_sets == 1); /* same: no write */
    assert(hal_isp_set_sensor_fps(&g_ctx, 59, 2) == RSS_OK);
    assert(g_pub.frame_rate == 29.5f && st.mode.frame_rate == 30);

    /* A refused write leaves the mode as it was. */
    g_pub_set_fail = 1;
    assert(hal_isp_set_sensor_fps(&g_ctx, 15, 1) == RSS_ERR_IO);
    assert(g_pub.frame_rate == 29.5f && st.mode.frame_rate == 30);
    assert(log_count("HI_MPI_ISP_SetPubAttr(15/1 fps) failed: 0xa01c0004") == 1);
    g_pub_set_fail = 0;

    /* Running without the optional GetPubAttr symbol: unsettable, and the
     * get falls back to the mode. */
    st.isp.fnGetPubAttr = NULL;
    assert(hal_isp_set_sensor_fps(&g_ctx, 15, 1) == RSS_ERR_NOTSUP);
    assert(hal_isp_get_sensor_fps(&g_ctx, &num, &den) == RSS_OK && num == 30 && den == 1);
    st.isp_thread_running = 0;
    st.isp.fnSetPubAttr = NULL;
}

/* ---- the [image] knobs over the tuning, and the exposure readback ---- */

static void knobs(void)
{
    static hisi_state_t st;
    rss_hal_ctx_t ctx;
    rss_isp_knob_t k;
    rss_exposure_t e;
    char path[32];
    int v = -1;

    memset(&ctx, 0, sizeof(ctx));
    ctx.platform = &st;
    reset_all(&st);
    g_csc.enable = 1;
    g_csc.luma = 50;
    g_csc.contr = 50;
    g_exp.auto_attr.compensation = 56;
    g_drc.enable = 1;
    g_drc.op_type = 1;
    g_drc.manual_strength = 512;
    write_ini_dyn(path, "100, 200, 400", "420, 380, 100");
    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);

    /* Before the ISP runs: noted, nothing written, the readback busy. */
    assert(hal_isp_get_exposure(&ctx, &e) == RSS_ERR_BUSY);
    assert(hal_isp_set_brightness(&ctx, 70) == RSS_OK && g_csc.luma == 50);
    assert(hal_isp_set_brightness(&ctx, 101) == RSS_ERR_INVAL);
    assert(hal_isp_set_ae_comp(&ctx, 80) == RSS_OK && g_exp.auto_attr.compensation == 56);
    assert(hal_isp_set_ae_comp(&ctx, 256) == RSS_ERR_INVAL);
    assert(hal_isp_set_drc_strength(&ctx, 300) == RSS_OK && g_drc.manual_strength == 512);
    assert(hal_isp_get_brightness(&ctx, &v) == RSS_OK && v == 70);
    assert(hal_isp_get_contrast(&ctx, &v) == RSS_ERR_BUSY);
    assert(hal_isp_get_knob_caps(&ctx, "ae_comp", &k) == RSS_OK && k.neutral == 56);

    /* The load: the file over the baseline, then the knobs over the file.
     * The fixture's [static_drc] is auto with the engine's column at ISO
     * 100 (420); the pin goes on over it and holds the engine. */
    st.isp_thread_running = 1;
    hisi_isp_note_frame(&st);
    assert(g_csc.luma == 70 && g_csc.contr == 50);
    assert(g_exp.auto_attr.compensation == 80);
    assert(st.knob.ae_base_known && st.knob.ae_base != 80);
    assert(g_drc.op_type == 1 && g_drc.manual_strength == 300);
    assert(st.knob.drc_base == 420 && st.knob.drc_base_op == 0);
    assert(st.dyn && st.dyn->drc.engine == 0 && st.dyn->dehaze.engine == 1);
    assert(log_count("drc: [dynamic_linear_drc] held while drc_strength is pinned") == 1);
    assert(log_count("drc_strength: 300 pinned (the tuning's is 420, auto)") == 1);

    /* Live: written at once, read back live. */
    assert(hal_isp_set_contrast(&ctx, 30) == RSS_OK && g_csc.contr == 30 && g_csc.luma == 70);
    assert(hal_isp_get_contrast(&ctx, &v) == RSS_OK && v == 30);
    assert(hal_isp_set_ae_comp(&ctx, 100) == RSS_OK && g_exp.auto_attr.compensation == 100);
    assert(hal_isp_get_ae_comp(&ctx, &v) == RSS_OK && v == 100);
    assert(hal_isp_get_drc_strength(&ctx, &v) == RSS_OK && v == 300);
    /* An ISO step moves dehaze and leaves the pinned DRC alone. */
    hisi_dyn_on_exposure(&st, 300, 3000);
    assert(g_drc.op_type == 1 && g_drc.manual_strength == 300 && g_dehaze.auto_strength == 78);

    /* Caps: the hardware's units, the tuning's neutrals. */
    assert(hal_isp_get_knob_caps(&ctx, "ae_comp", &k) == RSS_OK);
    assert(k.min == 0 && k.max == 255 && k.neutral == st.knob.ae_base && k.has_auto);
    assert(hal_isp_get_knob_caps(&ctx, "drc_strength", &k) == RSS_OK);
    assert(k.max == 1023 && k.neutral == 420 && k.has_auto && k.enabled);
    assert(hal_isp_get_knob_caps(&ctx, "brightness", &k) == RSS_OK);
    assert(k.max == 100 && k.neutral == 50 && !k.has_auto && k.enabled);
    assert(hal_isp_get_knob_caps(&ctx, "saturation", &k) == RSS_ERR_NOTSUP);

    /* auto: the tuning's value back; the DRC engine released and writing
     * its column for the ISO it last saw, at once. */
    assert(hal_isp_set_ae_comp(&ctx, RSS_ISP_AUTO) == RSS_OK);
    assert(g_exp.auto_attr.compensation == st.knob.ae_base);
    assert(hal_isp_set_drc_strength(&ctx, RSS_ISP_AUTO) == RSS_OK);
    assert(st.dyn->drc.engine == 1 && g_drc.op_type == 0 && g_drc.auto_strength == 240);
    assert(log_count("drc: [dynamic_linear_drc] released; strength 240 for ISO 300") == 1);
    assert(hal_isp_set_brightness(&ctx, RSS_ISP_AUTO) == RSS_OK && g_csc.luma == 50);

    /* A reload lifts a pin before the file and puts it back after. */
    assert(hal_isp_set_drc_strength(&ctx, 200) == RSS_OK && g_drc.op_type == 1);
    st.iq_load_started = 0;
    hisi_isp_note_frame(&st);
    assert(g_drc.op_type == 1 && g_drc.manual_strength == 200 && st.dyn->drc.engine == 0);
    assert(log_count("drc_strength: the tuning's 420 put back") == 2);
    assert(log_count("drc_strength: 200 pinned (the tuning's is 420, auto)") == 2);

    /* The readback: three 22.10 gains into one 1024-per-unit figure. */
    g_again = 2048;
    g_dgain = 1024;
    g_isp_dgain = 1536;
    g_ave_lum = 77;
    g_exp_time = 8333;
    assert(hal_isp_get_exposure(&ctx, &e) == RSS_OK);
    assert(e.total_gain == 3072 && e.exposure_time == 8333 && e.ae_luma == 77);
    assert(e.valid_mask ==
           (RSS_EXPOSURE_VALID_TOTAL_GAIN | RSS_EXPOSURE_VALID_TIME | RSS_EXPOSURE_VALID_AE_LUMA));

    /* A refused write is an error, and the wanted value stays for the
     * next re-apply. */
    g_set_fail[M_CSC] = true;
    assert(hal_isp_set_contrast(&ctx, 40) == RSS_ERR_IO && g_csc.contr == 30);
    g_set_fail[M_CSC] = false;
    assert(st.knob.contrast.val == 40);
    st.isp_thread_running = 0;
}

/* Orientation: the ops remember the bits and hand them to the framesource
 * layer every time, and the getter answers from memory. */
static void orien(void)
{
    static hisi_state_t st;
    rss_hal_ctx_t g_ctx;
    int hf = -1, vf = -1;

    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.platform = &st;
    reset_all(&st);
    g_orien_calls = 0;

    assert(hal_isp_get_hvflip(&g_ctx, &hf, &vf) == RSS_OK);
    assert(hf == 0 && vf == 0);

    assert(hal_isp_set_hflip(&g_ctx, 1) == RSS_OK);
    assert(g_orien_calls == 1 && g_orien_mirror == 1 && g_orien_flip == 0);
    /* Any non-zero is on. */
    assert(hal_isp_set_vflip(&g_ctx, 5) == RSS_OK);
    assert(g_orien_calls == 2 && g_orien_mirror == 1 && g_orien_flip == 1);
    assert(hal_isp_get_hvflip(&g_ctx, &hf, &vf) == RSS_OK);
    assert(hf == 1 && vf == 1);

    assert(hal_isp_set_hflip(&g_ctx, 0) == RSS_OK);
    assert(g_orien_calls == 3 && g_orien_mirror == 0 && g_orien_flip == 1);

    assert(hal_isp_get_hvflip(&g_ctx, NULL, &vf) == RSS_ERR_INVAL);
    assert(hal_isp_set_hflip(NULL, 1) == RSS_ERR_INVAL);
    assert(g_orien_calls == 3);
}

int main(void)
{
    load_good();
    load_nrx_manual();
    load_bad();
    load_vendor_failures();
    load_nrx_edges();
    load_dyn();
    load_module_state();
    sensor_fps();
    knobs();
    orien();

    printf("t_hisi_iq: OK\n");
    return 0;
}
