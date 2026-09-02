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
 *      errors, and a symbol that never resolved.
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

#include <assert.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

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

enum { M_EXP, M_ROUTE, M_STAT, M_LDCI, M_DRC, M_NR, M_DEHAZE, M_SHARPEN, M_DPC, M_GAMMA, M_N };

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
    memset(g_get_fail, 0, sizeof(g_get_fail));
    memset(g_set_fail, 0, sizeof(g_set_fail));
    g_sets = 0;
    g_set_calls = 0;
    g_log[0] = '\0';
    g_log_len = 0;

    memset(st, 0, sizeof(*st));
    st->tune_resolved = true; /* keep the stubs; nothing to dlsym here */
    st->tune.get_exp = get_exp;
    st->tune.set_exp = set_exp;
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
          "AutoDehazeStr = \"58,65,90\"\n" /* first column applies */
          "[dynamic_linear_drc]\n"
          "Strength = \"420, 380, 100\"\n"
          "Asymmetry = \"4, 4, 6\"\n"
          "[static_sharpen]\n"
          "Enable = \"1\"\n"
          "AutoLumaWgt_2 = \"31, 30\"\n"
          "AutoOverShoot = \"90, 80\"\n"
          "[static_dpc]\n"
          "DpcEnable = \"1\"\n"
          "DpcStrength = \"50, 100\"\n"
          "[static_3dnr]\n"
          "3DnrParam_0 = \\\n"
          "-nXsf1 18: 0:128 | 20: 0:128 \\\n"
          " -ref 0\n"
          "[ir_static_ae]\n"
          "AutoSpeed = 250\n" /* must NOT override the day value */
          "[dynamic_gamma]\n"
          "TotalNum = \"3\"\n"
          "Table_0 = \\\n",
          f);
    for (i = 0; i < V4_ISP_GAMMA_NODES; i++)
        fprintf(f, "%d%s", i % 4096,
                i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
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

    /* [static_drc] + [dynamic_linear_drc] daylight column; the whole
     * 200-node tone-mapping curve arrives through its continuations */
    assert(g_drc.enable == 1);
    assert(g_drc.op_type == 1);
    assert(g_drc.tone_mapping[0] == 0);
    assert(g_drc.tone_mapping[7] == 140);
    assert(g_drc.tone_mapping[V4_ISP_DRC_TM_NODES - 1] == (V4_ISP_DRC_TM_NODES - 1) * 20);
    assert(g_drc.auto_strength == 420); /* dynamic wins over the static 512 */
    assert(g_drc.asym.asymmetry == 4);

    /* [static_nr] */
    assert(g_nr.enable == 1);
    assert(g_nr.auto_fine_str[1] == 72);

    /* [static_dehaze] + [dynamic_dehaze] daylight column, whole LUT */
    assert(g_dehaze.enable == 1);
    assert(g_dehaze.user_lut_enable == 1);
    assert(g_dehaze.auto_strength == 58);
    assert(g_dehaze.lut[0] == 0);
    assert(g_dehaze.lut[V4_ISP_DEHAZE_LUT - 1] == V4_ISP_DEHAZE_LUT - 1);

    /* [static_sharpen] */
    assert(g_sharpen.enable == 1);
    assert(g_sharpen.auto_attr.luma_wgt[2][1] == 30);
    assert(g_sharpen.auto_attr.over_shoot[0] == 90);

    /* [static_dpc] */
    assert(g_dpc.enable == 1);
    assert(g_dpc.auto_strength[1] == 100);

    /* [dynamic_gamma] Table_0, all 1025 nodes through the continuations */
    assert(g_gamma.enable == 1);
    assert(g_gamma.curve_type == V4_ISP_GAMMA_CURVE_USER);
    assert(g_gamma.table[0] == 0);
    assert(g_gamma.table[1024] == 1024 % 4096);
    assert(g_gamma.table[512] == 512);

    /* one Set per touched module, and only touched modules */
    assert(g_sets == 10);
    /* a well-formed file trips none of the refusals load 2 is about */
    assert(log_count("truncated") == 0);
    assert(log_count("not applied") == 0);
    assert(log_count("module skipped") == 0);

    /* the latch is one-shot: a second frame reloads nothing */
    hisi_isp_note_frame(&st);
    assert(g_sets == 10);

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

    /* Eight modules were fetched and dirtied; dpc has no Set to call, so
     * seven are attempted, of which exp fails. */
    assert(g_set_calls == 7);
    assert(g_sets == 6);
    assert(log_count("6 modules applied, 2 failed") == 1);

    /* The undisturbed ones still went through */
    assert(g_nr.enable == 1);
    assert(g_sharpen.enable == 1);
    assert(g_drc.enable == 1);

    unlink(path);
}

int main(void)
{
    load_good();
    load_bad();
    load_vendor_failures();

    printf("t_hisi_iq: OK\n");
    return 0;
}
