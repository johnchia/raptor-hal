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
 * The tuning structs contain no pointers, so their ARM32 layouts hold on
 * the host too; the state header's *other* vendor structs do not, which
 * is why the Makefile defines _Static_assert away here like every suite.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "../src/hisi_v4/hal_isp.c"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

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
static int g_sets;

#define STUB(name, T, g)                                                                           \
    static int get_##name(int p, T *a)                                                             \
    {                                                                                              \
        (void)p;                                                                                   \
        *a = g;                                                                                    \
        return 0;                                                                                  \
    }                                                                                              \
    static int set_##name(int p, const T *a)                                                       \
    {                                                                                              \
        (void)p;                                                                                   \
        g = *a;                                                                                    \
        g_sets++;                                                                                  \
        return 0;                                                                                  \
    }

STUB(exp, v4_isp_exp_attr, g_exp)
STUB(route, v4_isp_ae_route_ex, g_route)
STUB(stat, v4_isp_stat_cfg, g_stat)
STUB(ldci, v4_isp_ldci_attr, g_ldci)
STUB(drc, v4_isp_drc_attr, g_drc)
STUB(nr, v4_isp_nr_attr, g_nr)
STUB(dehaze, v4_isp_dehaze_attr, g_dehaze)
STUB(sharpen, v4_isp_sharpen_attr, g_sharpen)
STUB(dpc, v4_isp_dp_dyn_attr, g_dpc)
STUB(gamma, v4_isp_gamma_attr, g_gamma)

static void t_log(int level, const char *file, int line, const char *fmt, ...)
{
    va_list ap;

    (void)level;
    (void)file;
    (void)line;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}
rss_hal_log_func_t rss_hal_log_fn = t_log;

static void write_ini(const char *path)
{
    FILE *f = fopen(path, "w");
    int i;

    assert(f);
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
          "DRCToneMappingValue = \\\n"
          "4095,5995,8191, \\\n" /* continuation, twice */
          "11583\n"
          "[static_nr]\n"
          "Enable = \"1\"\n"
          "FineStr = \"64, 72\"\n"
          "[static_dehaze]  \n"
          "Enable = \"1\"\n"
          "DehazeUserLutEnable = \"1\"\n"
          "[dynamic_dehaze]\n"
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
        fprintf(f, "%d%s", i % 4096, i == V4_ISP_GAMMA_NODES - 1 ? "\n" : (i % 30 == 29 ? ",\\\n" : ","));
    fclose(f);
}

int main(void)
{
    static hisi_state_t st;
    char path[] = "/tmp/t_hisi_iq_XXXXXX";
    int fd = mkstemp(path);

    assert(fd >= 0);
    close(fd);
    write_ini(path);

    snprintf(st.iq_file, sizeof(st.iq_file), "%s", path);
    st.tune_resolved = true; /* keep the stubs; nothing to dlsym here */
    st.tune.get_exp = get_exp;
    st.tune.set_exp = set_exp;
    st.tune.get_route_ex = get_route;
    st.tune.set_route_ex = set_route;
    st.tune.get_stat = get_stat;
    st.tune.set_stat = set_stat;
    st.tune.get_ldci = get_ldci;
    st.tune.set_ldci = set_ldci;
    st.tune.get_drc = get_drc;
    st.tune.set_drc = set_drc;
    st.tune.get_nr = get_nr;
    st.tune.set_nr = set_nr;
    st.tune.get_dehaze = get_dehaze;
    st.tune.set_dehaze = set_dehaze;
    st.tune.get_sharpen = get_sharpen;
    st.tune.set_sharpen = set_sharpen;
    st.tune.get_dpc = get_dpc;
    st.tune.set_dpc = set_dpc;
    st.tune.get_gamma = get_gamma;
    st.tune.set_gamma = set_gamma;

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

    /* [static_drc] + [dynamic_linear_drc] daylight column */
    assert(g_drc.enable == 1);
    assert(g_drc.op_type == 1);
    assert(g_drc.tone_mapping[0] == 4095);
    assert(g_drc.tone_mapping[3] == 11583);
    assert(g_drc.auto_strength == 420); /* dynamic wins over the static 512 */
    assert(g_drc.asym.asymmetry == 4);

    /* [static_nr] */
    assert(g_nr.enable == 1);
    assert(g_nr.auto_fine_str[1] == 72);

    /* [static_dehaze] + [dynamic_dehaze] daylight column */
    assert(g_dehaze.enable == 1);
    assert(g_dehaze.user_lut_enable == 1);
    assert(g_dehaze.auto_strength == 58);

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

    unlink(path);
    printf("t_hisi_iq: OK\n");
    return 0;
}
