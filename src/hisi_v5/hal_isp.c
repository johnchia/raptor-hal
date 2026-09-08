/*
 * hisi_v5/hal_isp.c -- ISP ops and the IQ tuning load for HiMPP V5.0
 *
 * OP COVERAGE
 *
 * Implemented: isp_get_sensor_attr, isp_set_sensor_fps, isp_get_sensor_fps,
 * plus the tuning loader below, which is Phase 3's scope. The rest of the
 * ISP vtable -- brightness, contrast, saturation, sharpness, defog,
 * antiflicker, the hflip/vflip pair -- stays NULL, and RSS_HAL_CALL turns
 * the absence into RSS_ERR_NOTSUP, which is the truth until hal_knob.c
 * lands.
 *
 * WHAT THE TUNING LOADER DOES
 *
 * Nothing on a V5 OpenIPC image applies IQ. There is no majestic here and
 * no /etc/sensors/iq: the ISP runs on whatever defaults libsns_<sensor>.so
 * and the algorithm libraries carry, which is a picture that works and a
 * picture nobody tuned. This file reads a tuning file -- the vendor's own
 * scene_auto dialect, section by section -- and hands each section to the
 * matching ss_mpi_isp_set_* through a get-modify-set: fetch the module's
 * whole attribute struct, patch exactly the fields the file names, write it
 * back. Fields the file does not mention keep the running defaults, which
 * is what makes a partial file safe to apply.
 *
 * THE DIALECT IS THE VENDOR'S, NOT MAJESTIC'S
 *
 * gen4's loader reads OpenIPC's /etc/sensors/iq/<sensor>.ini, which is
 * majestic's transcription of this same format. V5 has no majestic, so
 * the file read here is the upstream one: the `config_product_scene_*.ini`
 * that the SDK's scene_auto sample ships per sensor, and that the vendor's
 * own firmware carries under /etc/sensor/scene. Same lexical rules --
 * `key = "comma list"`, backslash continuation, ';' comments, an opening
 * [module_state] -- so gen4's reader ports unchanged. What differs is the
 * key set, which is larger and spelt in the vendor's snake_case rather
 * than majestic's CamelCase.
 *
 * Sections applied here, all of them values:
 *   static_ae, static_aerouteex, static_aeweight, static_ccm,
 *   static_saturation, static_ldci, static_drc, static_dm, static_nr,
 *   static_dehaze, static_sharpen, static_dpc, static_ca, static_pregamma,
 *   static_blc, static_csc, static_shading
 *
 * Sections deliberately not applied, and why:
 *   dynamic_*      tables over an axis (ISO, exposure, WDR ratio) that a
 *                  runtime engine walks. hal_dyn.c and hal_nrx.c are the
 *                  next two files in this phase; until they land these are
 *                  named in the summary line as skipped rather than
 *                  silently dropped, so a file that carries them is not
 *                  mistaken for one that applied them.
 *   static_3dnr    the 3DNR ladder. Not an ISP module on V5: the vendor's
 *                  reference writes it through ss_mpi_vi_set_pipe_3dnr_param
 *                  on the VI pipe, not the VPSS group gen4 uses. hal_nrx.c.
 *   static_awb,    sensor calibration -- white-balance curves, per-pipe
 *   static_awbex,  gain differences, lens shading meshes. These belong to
 *   static_isp_diff the module vendor's calibration of *that* lens and
 *                  sensor, and raptor has no business applying one file's
 *                  calibration to another board.
 *   static_venc,   encoder and scene-mode settings that raptor's own
 *   static_wdrexposure, config already owns, or that only apply in WDR.
 *   static_FsWdr
 *   static_cac,    modules whose attribute structs are not transcribed in
 *   static_crosstalk, v5_isp_tune.h. Naming them here is the promise that
 *   static_color_sector they were seen and skipped, not missed.
 *
 * THE ENABLE MASK
 *
 * A file opens with [module_state], the author's own list of which of its
 * sections are meant to be applied, and the vendor's loader opens every
 * setter with the matching flag. So a section sitting under a "0" is
 * documentation, not tuning, and reading it as tuning is how a file
 * written to leave a module alone ends up rewriting it. Every section
 * above is gated on its flag; a file carrying no [module_state] means all
 * of them. Two details in the map are the vendor's, not ours, and are
 * called out at iq_state_bits.
 *
 * WHEN IT RUNS
 *
 * Resolved at bring-up (hisi_isp_resolve_iq), applied on the first encoded
 * frame (hisi_isp_note_frame) -- gen4's shape and infinity6c's before it.
 * By the first frame the ISP is demonstrably running and registered, every
 * Get returns live state, and the one-time cost lands on one encoder
 * thread instead of in the init path.
 *
 * Selection: $RSS_ISP_TUNING if set (explicit wins outright -- an
 * unreadable explicit path is a warning and an untuned picture, not a
 * silent fallback), else /usr/share/raptor/iq/<sensor-stem>.ini, else the
 * image's /etc/sensors/iq/<sensor-stem>.ini, else untuned with one WARN.
 *
 * WHOLE TABLES ARE ALL-OR-NOTHING
 *
 * Get-modify-set makes a partial *file* safe. It does not make a partial
 * *table* safe: a curve or LUT is one field, and filling its first n
 * entries leaves the tail on whatever the Get returned, which is a
 * different curve. So the whole-table values -- DehazeLut (256),
 * DRC color_correction_lut (33), the pregamma table (257) and the CA LUTs
 * (128) -- are applied only at their exact node count, and a short or
 * truncated one is dropped with a WARN naming the count. The per-ISO
 * arrays (the 16 columns in NR, LDCI, sharpen, demosaic and DPC) are
 * deliberately not under that rule: those are sixteen independent per-gain
 * scalars rather than one curve, so a short list leaves the higher-ISO
 * columns on the running default, which is the ordinary partial-file
 * contract. Same for the AE route, whose used length is total_num.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"
#include "v5_isp_tune.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 * SENSOR GEOMETRY AND RATE
 * ================================================================ */

/*
 * hal_isp_get_sensor_attr -- the sensor's output size.
 *
 * Answers from the mode file rather than from ss_mpi_isp_get_pub_attr:
 * rvd asks before the pipeline is up, and the file is where the number
 * came from in the first place.
 */
int hal_isp_get_sensor_attr(void *ctx, uint32_t *width, uint32_t *height)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !width || !height)
        return RSS_ERR_INVAL;
    if (!st->mode.dev_rect.width || !st->mode.dev_rect.height)
        return RSS_ERR_NOTSUP;

    *width = st->mode.dev_rect.width;
    *height = st->mode.dev_rect.height;
    return RSS_OK;
}

/*
 * hal_isp_set_sensor_fps / hal_isp_get_sensor_fps -- the sensor's rate.
 *
 * The rate lives in the ISP's public attribute: the sensor library's fps
 * callback takes frame_rate from there and reprograms VMAX, and
 * ss_mpi_isp_set_pub_attr accepts a rate change on a running pipe (the
 * geometry it refuses to change; the rate it does not). So the set is a
 * get-modify-set of the public attribute once the 3A thread runs, and a
 * note in the mode before that -- hisi_isp_bringup builds the attribute
 * from the mode.
 *
 * The mode's copy is updated either way, because hal_framesource paces
 * VPSS channels against it, and a channel asked for 20 fps against a
 * sensor now at 20 should get no pacing rather than the file's 30-to-20
 * drop. Unlike gen4's, V5's mode carries the rate as the float the
 * attribute wants, so nothing is rounded on the way through.
 */
int hal_isp_set_sensor_fps(void *ctx, uint32_t fps_num, uint32_t fps_den)
{
    hisi_state_t *st = hisi_state(ctx);
    float fps;

    if (!st || !fps_num || !fps_den)
        return RSS_ERR_INVAL;
    fps = (float)fps_num / (float)fps_den;
    if (fps < 1.0f)
        return RSS_ERR_INVAL;

    if (__atomic_load_n(&st->isp_thread_running, __ATOMIC_ACQUIRE)) {
        v5_isp_pub_attr pub;
        int ret;

        if (!st->isp.fnGetPubAttr || !st->isp.fnSetPubAttr)
            return RSS_ERR_NOTSUP;
        ret = st->isp.fnGetPubAttr(HISI_VI_PIPE, &pub);
        if (ret) {
            HAL_LOG_ERR("ss_mpi_isp_get_pub_attr failed: 0x%x", ret);
            return RSS_ERR_IO;
        }
        if (pub.frame_rate != fps) {
            pub.frame_rate = fps;
            ret = st->isp.fnSetPubAttr(HISI_VI_PIPE, &pub);
            if (ret) {
                HAL_LOG_ERR("ss_mpi_isp_set_pub_attr(%u/%u fps) failed: 0x%x", fps_num, fps_den,
                            ret);
                return RSS_ERR_IO;
            }
            HAL_LOG_INFO("sensor: %u/%u fps (the mode file said %.2f)", fps_num, fps_den,
                         (double)st->mode.frame_rate);
        }
    } else if (fps != st->mode.frame_rate) {
        HAL_LOG_INFO("sensor: %.2f fps for bring-up (the mode file said %.2f)", (double)fps,
                     (double)st->mode.frame_rate);
    }
    st->mode.frame_rate = fps;
    return RSS_OK;
}

int hal_isp_get_sensor_fps(void *ctx, uint32_t *fps_num, uint32_t *fps_den)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !fps_num || !fps_den)
        return RSS_ERR_INVAL;

    if (__atomic_load_n(&st->isp_thread_running, __ATOMIC_ACQUIRE) && st->isp.fnGetPubAttr) {
        v5_isp_pub_attr pub;

        if (st->isp.fnGetPubAttr(HISI_VI_PIPE, &pub) == 0 && pub.frame_rate > 0.0f) {
            *fps_num = (uint32_t)(pub.frame_rate * 1000.0f + 0.5f);
            *fps_den = 1000;
            return RSS_OK;
        }
    }
    if (st->mode.frame_rate <= 0.0f)
        return RSS_ERR_NOTSUP;
    *fps_num = (uint32_t)(st->mode.frame_rate * 1000.0f + 0.5f);
    *fps_den = 1000;
    return RSS_OK;
}

/* ================================================================
 * IQ FILE RESOLUTION
 * ================================================================ */

/*
 * hisi_isp_resolve_iq -- settle which tuning file applies, or none.
 *
 * Called once from hal_init after the video pipeline is up, so the sensor
 * name is settled. Only the path is decided here; the load waits for the
 * first frame. Missing tuning is a degraded picture, not a failure, so
 * this never fails the pipeline.
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

    /* Lowercase up to the first separator: "os04d10_2l_10bit" -> "os04d10". */
    for (j = 0; j + 1 < sizeof(stem) && st->sensor_name[j]; j++) {
        char c = st->sensor_name[j];
        if (c == '_' || c == ' ' || c == '-')
            break;
        stem[j] = (char)tolower((unsigned char)c);
    }
    stem[j] = '\0';
    if (!stem[0])
        return;

    /*
     * An override first, then the image's own. raptor ships no IQ files:
     * a tuning belongs to the sensor package that knows the part and the
     * lens in front of it. /usr/share/raptor/iq is the place to put one
     * file without editing a vendor package -- empty on a stock image.
     * /etc/sensors/iq is the path OpenIPC's HiSilicon packages already
     * use on gen4, kept here so one convention covers both generations.
     */
    {
        static const char *const dirs[] = {"/usr/share/raptor/iq", "/etc/sensors/iq"};
        unsigned int d;

        for (d = 0; d < sizeof(dirs) / sizeof(dirs[0]); d++) {
            snprintf(st->iq_file, sizeof(st->iq_file), "%s/%s.ini", dirs[d], stem);
            if ((f = fopen(st->iq_file, "r"))) {
                fclose(f);
                HAL_LOG_INFO("isp: tuning is %s (loads on the first frame)", st->iq_file);
                return;
            }
        }
    }

    HAL_LOG_WARN("isp: no IQ tuning for %s in /usr/share/raptor/iq or /etc/sensors/iq -- "
                 "the picture stays on the sensor library's defaults (untuned)",
                 stem);
    st->iq_file[0] = '\0';
}

/* ================================================================
 * THE TUNING FILE READER
 * ================================================================ */

/*
 * One physical line. Far larger than gen4's 512, and measured rather than
 * guessed: this dialect puts a whole 1025-node gamma curve on ONE line --
 * `[dynamic_gamma] table_0` is 5075 bytes in the sc4336p file, and the
 * widest line across every scene_auto param file on hand (five CV610
 * sensors plus the DV500 set) is 5095. The vendor's own files use the
 * backslash continuation for some tables and not for others, so a reader
 * that only handles continuations is not enough.
 *
 * A truncated line is not cosmetic: for a whole table it trips the
 * exact-node-count rule and drops the module's table, and for a per-ISO
 * column it would quietly apply the first few entries. 512 truncated the
 * first vendor file this loader was pointed at.
 *
 * Heap, alongside the assembled value, because it is read on an encoder
 * thread whose stack rvd sized for encoding.
 */
#define HISI_IQ_LINE_MAX 8192

/*
 * The assembled value, continuations and all. The widest thing this
 * dialect carries is a 1025-node 12-bit gamma table written "4095, " --
 * 6150 bytes, plus one joining space per continuation. The vendor's own
 * files stay under 7 KB; 16 KB is one short-lived malloc per load and
 * puts the margin out of reach. That matters because truncation is not
 * cosmetic: a truncated table fails the exact-node-count rule and drops
 * its module rather than applying half a curve.
 */
#define HISI_IQ_VAL_MAX 16384

typedef struct {
    FILE *f;
    const char *path;
    char sect[40];
    char key[64];
    char *line; /* HISI_IQ_LINE_MAX, heap */
    char *val;  /* HISI_IQ_VAL_MAX, heap */
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
static char *iq_line(hisi_iq_reader *r)
{
    char *buf = r->line;
    char *comment;

    if (!fgets(buf, HISI_IQ_LINE_MAX, r->f))
        return NULL;

    /* A longer-than-buffer line arrives in pieces and a later piece would
     * be misparsed, so take the first piece and consume the rest. */
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
 * line, which is how these files carry their multi-KB tables.
 */
static bool iq_next(hisi_iq_reader *r)
{
    char *p;

    while ((p = iq_line(r))) {
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
            if (!(p = iq_line(r)))
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

/*
 * The strided stores. A per-ISO column that lives inside an array of
 * structs -- LDCI's gaussian coefficients, the CCM's per-temperature
 * matrices -- is one value per element at a fixed stride, and a stride of
 * the element size collapses to the flat case, so there is one filler per
 * width rather than two.
 */
static void iq_fill_u8(unsigned char *dst, int dn, const long *src, int sn, size_t stride)
{
    int i;

    for (i = 0; i < dn && i < sn; i++)
        *(unsigned char *)((char *)dst + (size_t)i * stride) =
            (unsigned char)iq_clamp(src[i], 0, 255);
}

static void iq_fill_u16(unsigned short *dst, int dn, const long *src, int sn, size_t stride)
{
    int i;

    for (i = 0; i < dn && i < sn; i++)
        *(unsigned short *)((char *)dst + (size_t)i * stride) =
            (unsigned short)iq_clamp(src[i], 0, 65535);
}

static void iq_fill_u32(unsigned int *dst, int dn, const long *src, int sn, size_t stride)
{
    int i;

    for (i = 0; i < dn && i < sn; i++)
        *(unsigned int *)((char *)dst + (size_t)i * stride) =
            (unsigned int)iq_clamp(src[i], 0, 0x7FFFFFFF);
}

static void iq_fill_s32(signed int *dst, int dn, const long *src, int sn)
{
    int i;

    for (i = 0; i < dn && i < sn; i++)
        dst[i] = (signed int)iq_clamp(src[i], -0x7FFFFFFF, 0x7FFFFFFF);
}

/* ================================================================
 * THE LOAD
 * ================================================================ */

#define IQ_NOTE_MAX 64
#define IQ_NOTE_LEN 32

enum {
    IQ_EXP = 1u << 0,
    IQ_ROUTE = 1u << 1,
    IQ_STAT = 1u << 2,
    IQ_CCM = 1u << 3,
    IQ_SAT = 1u << 4,
    IQ_TONE = 1u << 17,
    IQ_LDCI = 1u << 5,
    IQ_DRC = 1u << 6,
    IQ_DM = 1u << 7,
    IQ_NR = 1u << 8,
    IQ_DEHAZE = 1u << 9,
    IQ_SHARPEN = 1u << 10,
    IQ_DPC = 1u << 11,
    IQ_CA = 1u << 12,
    IQ_PREGAMMA = 1u << 13,
    IQ_BLC = 1u << 14,
    IQ_CSC = 1u << 15,
    IQ_SHADING = 1u << 16,
};

/*
 * One struct per module, fetched on first touch and written once at the
 * end. Heap-allocated as a unit: sharpen alone is 7 KB and this would be
 * a 16 KB stack frame on a thread rvd sized for encoding.
 */
typedef struct {
    v5_isp_exp_attr exp;
    v5_isp_ae_route_ex route;
    v5_isp_stats_cfg stat;
    v5_isp_ccm_attr ccm;
    v5_isp_saturation_attr sat;
    v5_isp_color_tone_attr tone;
    v5_isp_ldci_attr ldci;
    v5_isp_drc_attr drc;
    v5_isp_demosaic_attr dm;
    v5_isp_nr_attr nr;
    v5_isp_dehaze_attr dehaze;
    v5_isp_sharpen_attr sharpen;
    v5_isp_dp_dynamic_attr dpc;
    v5_isp_ca_attr ca;
    v5_isp_pregamma_attr pregamma;
    v5_isp_blc_attr blc;
    v5_isp_csc_attr csc;
    v5_isp_shading_attr shading;

    unsigned int have;    /* fetched via Get */
    unsigned int unavail; /* Get failed or symbol missing; do not retry */
    unsigned int dirty;   /* the file touched it; Set on completion */
    bool route_seen;      /* static_aerouteex present -> ae_route_ex_valid */
    unsigned int state;   /* [module_state]; MS_ALL when the file carries none */

    long nums[V5_ISP_DEHAZE_LUT]; /* the largest table parsed here */

    /*
     * The two lists the summary line ends with, kept as names rather than
     * as one assembled string. A V5 file carries forty-odd sections and
     * most of them are ones this loader does not apply, so the summary
     * runs long -- and a name arrives once per *key*, not once per
     * section, so the de-duplication has to be by name. Sixty-four slots
     * is past every file on hand; the counter is what happens if a file
     * ever goes past that.
     */
    char skipped[IQ_NOTE_MAX][IQ_NOTE_LEN];  /* sections with no handler here */
    char disabled[IQ_NOTE_MAX][IQ_NOTE_LEN]; /* sections [module_state] turned off */
    int skipped_n, skipped_more;
    int disabled_n, disabled_more;
} hisi_iq_load;

static const struct {
    unsigned int bit;
    const char *name;
    size_t offset;
} iq_modules[] = {
    {IQ_EXP, "exposure_attr", offsetof(hisi_iq_load, exp)},
    {IQ_ROUTE, "ae_route_attr_ex", offsetof(hisi_iq_load, route)},
    {IQ_STAT, "stats_cfg", offsetof(hisi_iq_load, stat)},
    {IQ_CCM, "ccm_attr", offsetof(hisi_iq_load, ccm)},
    {IQ_SAT, "saturation_attr", offsetof(hisi_iq_load, sat)},
    {IQ_TONE, "color_tone_attr", offsetof(hisi_iq_load, tone)},
    {IQ_LDCI, "ldci_attr", offsetof(hisi_iq_load, ldci)},
    {IQ_DRC, "drc_attr", offsetof(hisi_iq_load, drc)},
    {IQ_DM, "demosaic_attr", offsetof(hisi_iq_load, dm)},
    {IQ_NR, "nr_attr", offsetof(hisi_iq_load, nr)},
    {IQ_DEHAZE, "dehaze_attr", offsetof(hisi_iq_load, dehaze)},
    {IQ_SHARPEN, "sharpen_attr", offsetof(hisi_iq_load, sharpen)},
    {IQ_DPC, "dp_dynamic_attr", offsetof(hisi_iq_load, dpc)},
    {IQ_CA, "ca_attr", offsetof(hisi_iq_load, ca)},
    {IQ_PREGAMMA, "pregamma_attr", offsetof(hisi_iq_load, pregamma)},
    {IQ_BLC, "black_level_attr", offsetof(hisi_iq_load, blc)},
    {IQ_CSC, "csc_attr", offsetof(hisi_iq_load, csc)},
    {IQ_SHADING, "mesh_shading_attr", offsetof(hisi_iq_load, shading)},
};

static const char *iq_mod_name(unsigned int bit)
{
    size_t i;

    for (i = 0; i < sizeof(iq_modules) / sizeof(iq_modules[0]); i++) {
        if (iq_modules[i].bit == bit)
            return iq_modules[i].name;
    }
    return "?";
}

static void *iq_struct_of(hisi_iq_load *ld, unsigned int bit)
{
    size_t i;

    for (i = 0; i < sizeof(iq_modules) / sizeof(iq_modules[0]); i++) {
        if (iq_modules[i].bit == bit)
            return (char *)ld + iq_modules[i].offset;
    }
    return NULL;
}

/*
 * The get/set pairs, by bit. Cast through void * on purpose: every pair
 * has a different attribute type and the loader only ever hands each one
 * its own struct, which iq_struct_of guarantees by construction.
 */
static int (*iq_getter(hisi_state_t *st, unsigned int bit))(int, void *)
{
    const v5_isp_tune_impl *t = &st->tune;

    switch (bit) {
    case IQ_EXP:
        return (int (*)(int, void *))t->fnGetExposureAttr;
    case IQ_ROUTE:
        return (int (*)(int, void *))t->fnGetAeRouteAttrEx;
    case IQ_STAT:
        return (int (*)(int, void *))t->fnGetStatsCfg;
    case IQ_CCM:
        return (int (*)(int, void *))t->fnGetCcmAttr;
    case IQ_SAT:
        return (int (*)(int, void *))t->fnGetSaturationAttr;
    case IQ_TONE:
        return (int (*)(int, void *))t->fnGetColorToneAttr;
    case IQ_LDCI:
        return (int (*)(int, void *))t->fnGetLdciAttr;
    case IQ_DRC:
        return (int (*)(int, void *))t->fnGetDrcAttr;
    case IQ_DM:
        return (int (*)(int, void *))t->fnGetDemosaicAttr;
    case IQ_NR:
        return (int (*)(int, void *))t->fnGetNrAttr;
    case IQ_DEHAZE:
        return (int (*)(int, void *))t->fnGetDehazeAttr;
    case IQ_SHARPEN:
        return (int (*)(int, void *))t->fnGetSharpenAttr;
    case IQ_DPC:
        return (int (*)(int, void *))t->fnGetDpDynamicAttr;
    case IQ_CA:
        return (int (*)(int, void *))t->fnGetCaAttr;
    case IQ_PREGAMMA:
        return (int (*)(int, void *))t->fnGetPregammaAttr;
    case IQ_BLC:
        return (int (*)(int, void *))t->fnGetBlackLevelAttr;
    case IQ_CSC:
        return (int (*)(int, void *))t->fnGetCscAttr;
    case IQ_SHADING:
        return (int (*)(int, void *))t->fnGetShadingAttr;
    }
    return NULL;
}

static int (*iq_setter(hisi_state_t *st, unsigned int bit))(int, const void *)
{
    const v5_isp_tune_impl *t = &st->tune;

    switch (bit) {
    case IQ_EXP:
        return (int (*)(int, const void *))t->fnSetExposureAttr;
    case IQ_ROUTE:
        return (int (*)(int, const void *))t->fnSetAeRouteAttrEx;
    case IQ_STAT:
        return (int (*)(int, const void *))t->fnSetStatsCfg;
    case IQ_CCM:
        return (int (*)(int, const void *))t->fnSetCcmAttr;
    case IQ_SAT:
        return (int (*)(int, const void *))t->fnSetSaturationAttr;
    case IQ_TONE:
        return (int (*)(int, const void *))t->fnSetColorToneAttr;
    case IQ_LDCI:
        return (int (*)(int, const void *))t->fnSetLdciAttr;
    case IQ_DRC:
        return (int (*)(int, const void *))t->fnSetDrcAttr;
    case IQ_DM:
        return (int (*)(int, const void *))t->fnSetDemosaicAttr;
    case IQ_NR:
        return (int (*)(int, const void *))t->fnSetNrAttr;
    case IQ_DEHAZE:
        return (int (*)(int, const void *))t->fnSetDehazeAttr;
    case IQ_SHARPEN:
        return (int (*)(int, const void *))t->fnSetSharpenAttr;
    case IQ_DPC:
        return (int (*)(int, const void *))t->fnSetDpDynamicAttr;
    case IQ_CA:
        return (int (*)(int, const void *))t->fnSetCaAttr;
    case IQ_PREGAMMA:
        return (int (*)(int, const void *))t->fnSetPregammaAttr;
    case IQ_BLC:
        return (int (*)(int, const void *))t->fnSetBlackLevelAttr;
    case IQ_CSC:
        return (int (*)(int, const void *))t->fnSetCscAttr;
    case IQ_SHADING:
        return (int (*)(int, const void *))t->fnSetShadingAttr;
    }
    return NULL;
}

/* The Get half of get-modify-set, once per module per load. */
static bool iq_fetch(hisi_state_t *st, hisi_iq_load *ld, unsigned int bit)
{
    int (*get)(int, void *);
    int ret;

    if (ld->have & bit)
        return true;
    if (ld->unavail & bit)
        return false;

    if (!(get = iq_getter(st, bit))) {
        HAL_LOG_WARN("isp tuning: ss_mpi_isp_get_%s unresolved -- its sections are skipped",
                     iq_mod_name(bit));
        ld->unavail |= bit;
        return false;
    }

    ret = get(HISI_VI_PIPE, iq_struct_of(ld, bit));
    if (ret) {
        HAL_LOG_WARN("isp tuning: ss_mpi_isp_get_%s failed: 0x%x -- its sections are skipped",
                     iq_mod_name(bit), ret);
        ld->unavail |= bit;
        return false;
    }

    ld->have |= bit;
    return true;
}

/*
 * A whole table, at its exact node count or not at all. Returns the count
 * on success and 0 when the value is short, having said so once per key.
 */
static int iq_table(hisi_iq_load *ld, const char *sect, const char *key, const char *val, int want)
{
    int n = iq_nums(val, ld->nums, want);

    if (n != want) {
        HAL_LOG_WARN("isp tuning: [%s] %s: %d of %d nodes -- the table is left alone "
                     "(a partial curve is a different curve)",
                     sect, key, n, want);
        return 0;
    }
    return n;
}

/* ---------------- per-section handlers ---------------- */

static void iq_sect_static_ae(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_exp_attr *e = &ld->exp;

    if (!iq_fetch(st, ld, IQ_EXP))
        return;

    if (iq_ci_eq(key, "ae_run_interval"))
        e->ae_run_interval = (unsigned char)iq_clamp(iq_num(val, 1), 1, 255);
    else if (iq_ci_eq(key, "ae_route_ex_valid"))
        e->ae_route_ex_valid = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "auto_sys_gain_max"))
        e->auto_attr.sys_gain_range.max = (unsigned int)iq_clamp(iq_num(val, 0), 1024, 0x7FFFFFFF);
    else if (iq_ci_eq(key, "auto_exp_time_max"))
        e->auto_attr.exp_time_range.max = (unsigned int)iq_clamp(iq_num(val, 0), 1, 0x7FFFFFFF);
    else if (iq_ci_eq(key, "auto_speed"))
        e->auto_attr.speed = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "auto_tolerance"))
        e->auto_attr.tolerance = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "auto_black_delay_frame"))
        e->auto_attr.ae_delay_attr.black_delay_frame =
            (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "auto_white_delay_frame"))
        e->auto_attr.ae_delay_attr.white_delay_frame =
            (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "hist_ratio_slope"))
        e->auto_attr.hist_ratio_slope = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "max_hist_offset"))
        e->auto_attr.max_hist_offset = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else {
        HAL_LOG_DBG("isp tuning: [static_ae] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_EXP;
}

/*
 * The route is four parallel columns -- int_time, again, dgain, isp_dgain
 * -- indexed by node, and total_num says how many of the sixteen are
 * live. A column is filled to whatever length it carries, because the
 * columns are independent per-node values rather than one curve.
 */
static void iq_sect_route_ex(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_ae_route_ex *r = &ld->route;
    unsigned int *first = NULL;
    int n;

    if (!iq_fetch(st, ld, IQ_ROUTE))
        return;

    ld->route_seen = true;

    if (iq_ci_eq(key, "total_num")) {
        r->total_num = (unsigned int)iq_clamp(iq_num(val, 0), 0, (long)V5_ISP_AE_ROUTE_EX_NODES);
        ld->dirty |= IQ_ROUTE;
        return;
    }

    if (iq_ci_eq(key, "int_time"))
        first = &r->route_ex_node[0].int_time;
    else if (iq_ci_eq(key, "again"))
        first = &r->route_ex_node[0].a_gain;
    else if (iq_ci_eq(key, "dgain"))
        first = &r->route_ex_node[0].d_gain;
    else if (iq_ci_eq(key, "isp_dgain"))
        first = &r->route_ex_node[0].isp_d_gain;
    else {
        HAL_LOG_DBG("isp tuning: [static_aerouteex] %s: no mapping", key);
        return;
    }

    n = iq_nums(val, ld->nums, V5_ISP_AE_ROUTE_EX_NODES);
    iq_fill_u32(first, V5_ISP_AE_ROUTE_EX_NODES, ld->nums, n, sizeof(r->route_ex_node[0]));
    ld->dirty |= IQ_ROUTE;
}

/*
 * The metering weight table, one key per row. The FE table is the one the
 * dialect writes; the BE table it does not name is left as fetched. The
 * `ctrl` and `update` keys are left exactly as the Get returned them --
 * the vendor's reference does the same plain get-modify-set, and those
 * two words are the ISR's own access keys rather than a commit flag.
 */
static void iq_sect_aeweight(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    int row = iq_row_index(key, "ae_weight");
    int n;

    if (row < 0) {
        HAL_LOG_DBG("isp tuning: [static_aeweight] %s: no mapping", key);
        return;
    }
    if (row >= V5_ISP_AE_ROWS) {
        HAL_LOG_WARN("isp tuning: [static_aeweight] %s: row past %d -- ignored", key,
                     V5_ISP_AE_ROWS - 1);
        return;
    }
    if (!iq_fetch(st, ld, IQ_STAT))
        return;

    n = iq_nums(val, ld->nums, V5_ISP_AE_COLS);
    iq_fill_u8(ld->stat.ae_cfg.weight[row], V5_ISP_AE_COLS, ld->nums, n, 1);
    ld->dirty |= IQ_STAT;
}

static void iq_sect_ccm(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_ccm_attr *c = &ld->ccm;
    int row, n;

    if (!iq_fetch(st, ld, IQ_CCM))
        return;

    if ((row = iq_row_index(key, "auto_ccm")) >= 0) {
        if (row >= V5_ISP_CCM_MATRIX_NUM) {
            HAL_LOG_WARN("isp tuning: [static_ccm] %s: past matrix %d -- ignored", key,
                         V5_ISP_CCM_MATRIX_NUM - 1);
            return;
        }
        if (!(n = iq_table(ld, "static_ccm", key, val, V5_ISP_CCM_MATRIX_SIZE)))
            return;
        iq_fill_u16(c->auto_attr.ccm_tab[row].ccm, V5_ISP_CCM_MATRIX_SIZE, ld->nums, n,
                    sizeof(unsigned short));
    } else if (iq_ci_eq(key, "ccm_op_type"))
        c->op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "auto_iso_act_en"))
        c->auto_attr.iso_act_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "auto_temp_act_en"))
        c->auto_attr.temp_act_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "total_num"))
        c->auto_attr.ccm_tab_num =
            (unsigned short)iq_clamp(iq_num(val, 0), 0, V5_ISP_CCM_MATRIX_NUM);
    else if (iq_ci_eq(key, "manual_ccm")) {
        if (!(n = iq_table(ld, "static_ccm", key, val, V5_ISP_CCM_MATRIX_SIZE)))
            return;
        iq_fill_u16(c->manual_attr.ccm, V5_ISP_CCM_MATRIX_SIZE, ld->nums, n,
                    sizeof(unsigned short));
    } else if (iq_ci_eq(key, "auto_color_temp")) {
        n = iq_nums(val, ld->nums, V5_ISP_CCM_MATRIX_NUM);
        iq_fill_u16(&c->auto_attr.ccm_tab[0].color_temp, V5_ISP_CCM_MATRIX_NUM, ld->nums, n,
                    sizeof(c->auto_attr.ccm_tab[0]));
    } else if (iq_ci_eq(key, "red_cast_gain") || iq_ci_eq(key, "green_cast_gain") ||
               iq_ci_eq(key, "blue_cast_gain")) {
        /*
         * The three per-channel gains after the matrix. A different MPI
         * pair -- ss_mpi_isp_set_color_tone_attr -- carried in this same
         * section, and the vendor's reference writes both from it.
         */
        if (!iq_fetch(st, ld, IQ_TONE))
            return;
        if (iq_ci_eq(key, "red_cast_gain"))
            ld->tone.red_cast_gain = (unsigned short)iq_clamp(iq_num(val, 0x100), 0, 65535);
        else if (iq_ci_eq(key, "green_cast_gain"))
            ld->tone.green_cast_gain = (unsigned short)iq_clamp(iq_num(val, 0x100), 0, 65535);
        else
            ld->tone.blue_cast_gain = (unsigned short)iq_clamp(iq_num(val, 0x100), 0, 65535);
        ld->dirty |= IQ_TONE;
        return;
    } else {
        HAL_LOG_DBG("isp tuning: [static_ccm] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_CCM;
}

static void iq_sect_saturation(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    int n;

    if (!iq_fetch(st, ld, IQ_SAT))
        return;

    if (iq_ci_eq(key, "op_type"))
        ld->sat.op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "manual_sat"))
        ld->sat.manual_saturation = (unsigned char)iq_clamp(iq_num(val, 128), 0, 255);
    else if (iq_ci_eq(key, "auto_sat")) {
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        iq_fill_u8(ld->sat.auto_sat, V5_ISP_ISO_NUM, ld->nums, n, 1);
    } else {
        HAL_LOG_DBG("isp tuning: [static_saturation] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_SAT;
}

/*
 * LDCI's six gaussian coefficients are per-ISO columns inside an array of
 * six-byte structs, so each is a strided fill at the stride of the
 * element -- the reason iq_fill_u8 takes one at all.
 */
static void iq_sect_ldci(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_ldci_attr *l = &ld->ldci;
    const size_t wgt_stride = sizeof(l->auto_attr.he_wgt[0]);
    unsigned char *col = NULL;
    int n;

    if (!iq_fetch(st, ld, IQ_LDCI))
        return;

    if (iq_ci_eq(key, "enable"))
        l->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "ldci_op_type"))
        l->op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "gauss_lpf_sigma"))
        l->gauss_lpf_sigma = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "tpr_incr_coef"))
        l->tpr_incr_coef = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "tpr_decr_coef"))
        l->tpr_decr_coef = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "auto_blc_ctrl")) {
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        iq_fill_u16(l->auto_attr.blc_ctrl, V5_ISP_ISO_NUM, ld->nums, n, sizeof(unsigned short));
    } else {
        if (iq_ci_eq(key, "auto_he_pos_wgt"))
            col = &l->auto_attr.he_wgt[0].he_pos_wgt.wgt;
        else if (iq_ci_eq(key, "auto_he_pos_sigma"))
            col = &l->auto_attr.he_wgt[0].he_pos_wgt.sigma;
        else if (iq_ci_eq(key, "auto_he_pos_mean"))
            col = &l->auto_attr.he_wgt[0].he_pos_wgt.mean;
        else if (iq_ci_eq(key, "auto_he_neg_wgt"))
            col = &l->auto_attr.he_wgt[0].he_neg_wgt.wgt;
        else if (iq_ci_eq(key, "auto_he_neg_sigma"))
            col = &l->auto_attr.he_wgt[0].he_neg_wgt.sigma;
        else if (iq_ci_eq(key, "auto_he_neg_mean"))
            col = &l->auto_attr.he_wgt[0].he_neg_wgt.mean;
        else {
            HAL_LOG_DBG("isp tuning: [static_ldci] %s: no mapping", key);
            return;
        }
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        iq_fill_u8(col, V5_ISP_ISO_NUM, ld->nums, n, wgt_stride);
    }
    ld->dirty |= IQ_LDCI;
}

static void iq_sect_static_drc(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_drc_attr *d = &ld->drc;
    int n;

    if (!iq_fetch(st, ld, IQ_DRC))
        return;

    if (iq_ci_eq(key, "enable"))
        d->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "curve_select"))
        d->curve_select = (int)iq_clamp(iq_num(val, 0), 0, 1);
    else if (iq_ci_eq(key, "op_type"))
        d->op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "contrast_ctrl"))
        d->contrast_ctrl = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "global_color_ctrl"))
        d->global_color_ctrl = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "high_saturation_color_ctrl"))
        d->high_saturation_color_ctrl = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "purple_reduction_strength"))
        d->purple_reduction_strength = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "drc_bcnr_en"))
        d->bcnr_attr.enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "color_correction_lut")) {
        if (!(n = iq_table(ld, "static_drc", key, val, V5_ISP_DRC_CC_NODES)))
            return;
        iq_fill_u16(d->color_correction_lut, V5_ISP_DRC_CC_NODES, ld->nums, n,
                    sizeof(unsigned short));
    } else if (iq_ci_eq(key, "tone_mapping_value")) {
        if (!(n = iq_table(ld, "static_drc", key, val, V5_ISP_DRC_TM_NODES)))
            return;
        iq_fill_u16(d->tone_mapping_value, V5_ISP_DRC_TM_NODES, ld->nums, n,
                    sizeof(unsigned short));
    } else {
        HAL_LOG_DBG("isp tuning: [static_drc] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_DRC;
}

static void iq_sect_dm(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_demosaic_auto *a = &ld->dm.auto_attr;
    unsigned char *col = NULL;
    int n;

    if (!iq_fetch(st, ld, IQ_DM))
        return;

    if (iq_ci_eq(key, "enable"))
        ld->dm.enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "op_type"))
        ld->dm.op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else {
        if (iq_ci_eq(key, "nddm_strength"))
            col = a->nddm_strength;
        else if (iq_ci_eq(key, "nddm_mf_detail_strength"))
            col = a->nddm_mf_detail_strength;
        else if (iq_ci_eq(key, "hf_detail_strength"))
            col = a->hf_detail_strength;
        else if (iq_ci_eq(key, "detail_smooth_range"))
            col = a->detail_smooth_range;
        else if (iq_ci_eq(key, "color_noise_f_threshold"))
            col = a->color_noise_f_threshold;
        else if (iq_ci_eq(key, "color_noise_f_strength"))
            col = a->color_noise_f_strength;
        else if (iq_ci_eq(key, "color_noise_y_threshold"))
            col = a->color_noise_y_threshold;
        else if (iq_ci_eq(key, "color_noise_y_strength"))
            col = a->color_noise_y_strength;
        else {
            HAL_LOG_DBG("isp tuning: [static_dm] %s: no mapping", key);
            return;
        }
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        iq_fill_u8(col, V5_ISP_ISO_NUM, ld->nums, n, 1);
    }
    ld->dirty |= IQ_DM;
}

/*
 * Bayer NR, the head of the attribute. The spatial and motion-detect
 * configs behind it are the opaque tail v5_isp_tune.h documents, so the
 * dialect's sfm0_*, md_* and noisesd_* keys are named as unmapped rather
 * than written into bytes nobody has transcribed.
 */
static void iq_sect_nr(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_nr_attr *nr = &ld->nr;
    int n;

    if (!iq_fetch(st, ld, IQ_NR))
        return;

    if (iq_ci_eq(key, "enable"))
        nr->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "op_type"))
        nr->op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "md_enable"))
        nr->md_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "lsc_nr_enable"))
        nr->lsc_nr_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "lsc_ratio1"))
        nr->lsc_ratio1 = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "bnr_ref_mode"))
        nr->ref_mode = (int)iq_clamp(iq_num(val, 0), 0, 3);
    else if (iq_ci_eq(key, "load_ref_en"))
        nr->load_ref_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "coring_ratio")) {
        n = iq_nums(val, ld->nums, V5_ISP_BAYERNR_LUT);
        iq_fill_u16(nr->coring_ratio, V5_ISP_BAYERNR_LUT, ld->nums, n, sizeof(unsigned short));
    } else if (iq_ci_eq(key, "coring_wgt")) {
        n = iq_nums(val, ld->nums, V5_ISP_BAYERNR_LUT1);
        iq_fill_u16(nr->mix_gain, V5_ISP_BAYERNR_LUT1, ld->nums, n, sizeof(unsigned short));
    } else {
        HAL_LOG_DBG("isp tuning: [static_nr] %s: not in the transcribed head", key);
        return;
    }
    ld->dirty |= IQ_NR;
}

static void iq_sect_dehaze(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_dehaze_attr *d = &ld->dehaze;
    int n;

    if (!iq_fetch(st, ld, IQ_DEHAZE))
        return;

    if (iq_ci_eq(key, "enable"))
        d->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "user_lut_enable"))
        d->user_lut_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "dehaze_op_type"))
        d->op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "manual_strength"))
        d->manual_strength = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "auto_strength"))
        d->auto_strength = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "tmprflt_incr_coef"))
        d->tmprflt_incr_coef = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "tmprflt_decr_coef"))
        d->tmprflt_decr_coef = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "dehaze_lut")) {
        if (!(n = iq_table(ld, "static_dehaze", key, val, V5_ISP_DEHAZE_LUT)))
            return;
        iq_fill_u8(d->dehaze_lut, V5_ISP_DEHAZE_LUT, ld->nums, n, 1);
    } else {
        HAL_LOG_DBG("isp tuning: [static_dehaze] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_DEHAZE;
}

/*
 * Sharpen. Every knob in the auto half is a sixteen-wide per-ISO row, and
 * the six two-dimensional ones are written a row at a time as
 * `<knob>_<row>`, which is exactly [knob][ISO] in the struct -- so a row
 * key is a flat sixteen-value fill at the row's address.
 */
static void iq_sect_sharpen(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_sharpen_attr *s = &ld->sharpen;
    v5_isp_sharpen_auto *a = &s->auto_attr;
    unsigned char *u8col = NULL;
    unsigned short *u16col = NULL;
    int row, n;

    if (!iq_fetch(st, ld, IQ_SHARPEN))
        return;

    /* The two-dimensional knobs, one row per key. */
    if ((row = iq_row_index(key, "luma_wgt")) >= 0) {
        if (row >= V5_ISP_SHARPEN_LUMA)
            goto past_row;
        u8col = a->luma_wgt[row];
    } else if ((row = iq_row_index(key, "texture_strength")) >= 0) {
        if (row >= V5_ISP_SHARPEN_GAIN)
            goto past_row;
        u16col = a->texture_strength[row];
    } else if ((row = iq_row_index(key, "edge_strength")) >= 0) {
        if (row >= V5_ISP_SHARPEN_GAIN)
            goto past_row;
        u16col = a->edge_strength[row];
    } else if ((row = iq_row_index(key, "motion_texture_strength")) >= 0) {
        if (row >= V5_ISP_SHARPEN_GAIN)
            goto past_row;
        u16col = a->motion_texture_strength[row];
    } else if ((row = iq_row_index(key, "motion_edge_strength")) >= 0) {
        if (row >= V5_ISP_SHARPEN_GAIN)
            goto past_row;
        u16col = a->motion_edge_strength[row];
    } else if ((row = iq_row_index(key, "edge_gain_by_rly")) >= 0) {
        if (row >= V5_ISP_SHARPEN_RLYWGT)
            goto past_row;
        u8col = a->edge_rly_attr.edge_gain_by_rly[row];
    } else if ((row = iq_row_index(key, "edge_rly_by_mot")) >= 0) {
        if (row >= V5_ISP_SHARPEN_STDGAIN)
            goto past_row;
        u8col = a->edge_rly_attr.edge_rly_by_mot[row];
    } else if ((row = iq_row_index(key, "edge_rly_by_luma")) >= 0) {
        if (row >= V5_ISP_SHARPEN_STDGAIN)
            goto past_row;
        u8col = a->edge_rly_attr.edge_rly_by_luma[row];
    } else if ((row = iq_row_index(key, "mf_gain_by_mot")) >= 0) {
        if (row >= V5_ISP_SHARPEN_MOT)
            goto past_row;
        u8col = a->gain_by_mot_attr.mf_gain_by_mot[row];
    } else if ((row = iq_row_index(key, "hf_gain_by_mot")) >= 0) {
        if (row >= V5_ISP_SHARPEN_MOT)
            goto past_row;
        u8col = a->gain_by_mot_attr.hf_gain_by_mot[row];
    } else if ((row = iq_row_index(key, "lmf_gain_by_mot")) >= 0) {
        if (row >= V5_ISP_SHARPEN_MOT)
            goto past_row;
        u8col = a->gain_by_mot_attr.lmf_gain_by_mot[row];
    }
    /* The scalars, and the one-dimensional per-ISO knobs. */
    else if (iq_ci_eq(key, "enable"))
        s->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "op_type"))
        s->op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "motion_enable"))
        s->motion_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "skin_umin"))
        s->skin_umin = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "skin_vmin"))
        s->skin_vmin = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "skin_umax"))
        s->skin_umax = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "skin_vmax"))
        s->skin_vmax = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "texture_freq"))
        u16col = a->texture_freq;
    else if (iq_ci_eq(key, "edge_freq"))
        u16col = a->edge_freq;
    else if (iq_ci_eq(key, "motion_texture_freq"))
        u16col = a->motion_texture_freq;
    else if (iq_ci_eq(key, "motion_edge_freq"))
        u16col = a->motion_edge_freq;
    else if (iq_ci_eq(key, "max_sharp_gain"))
        u16col = a->max_sharp_gain;
    else if (iq_ci_eq(key, "shoot_inner_threshold"))
        u16col = a->shoot_threshold_attr.shoot_inner_threshold;
    else if (iq_ci_eq(key, "shoot_outer_threshold"))
        u16col = a->shoot_threshold_attr.shoot_outer_threshold;
    else if (iq_ci_eq(key, "shoot_protect_threshold"))
        u16col = a->shoot_threshold_attr.shoot_protect_threshold;
    else if (iq_ci_eq(key, "edge_rly_fine_threshold"))
        u16col = a->edge_rly_attr.edge_rly_fine_threshold;
    else if (iq_ci_eq(key, "edge_rly_coarse_threshold"))
        u16col = a->edge_rly_attr.edge_rly_coarse_threshold;
    else if (iq_ci_eq(key, "over_shoot"))
        u8col = a->over_shoot;
    else if (iq_ci_eq(key, "under_shoot"))
        u8col = a->under_shoot;
    else if (iq_ci_eq(key, "motion_over_shoot"))
        u8col = a->motion_over_shoot;
    else if (iq_ci_eq(key, "motion_under_shoot"))
        u8col = a->motion_under_shoot;
    else if (iq_ci_eq(key, "shoot_sup_strength"))
        u8col = a->shoot_sup_strength;
    else if (iq_ci_eq(key, "shoot_sup_adj"))
        u8col = a->shoot_sup_adj;
    else if (iq_ci_eq(key, "detail_ctrl"))
        u8col = a->detail_ctrl;
    else if (iq_ci_eq(key, "detail_ctrl_threshold"))
        u8col = a->detail_ctrl_threshold;
    else if (iq_ci_eq(key, "edge_filt_strength"))
        u8col = a->edge_filt_strength;
    else if (iq_ci_eq(key, "edge_filt_max_cap"))
        u8col = a->edge_filt_max_cap;
    else if (iq_ci_eq(key, "edge_overshoot"))
        u8col = a->edge_rly_attr.edge_overshoot;
    else if (iq_ci_eq(key, "edge_undershoot"))
        u8col = a->edge_rly_attr.edge_undershoot;
    else if (iq_ci_eq(key, "r_gain"))
        u8col = a->r_gain;
    else if (iq_ci_eq(key, "g_gain"))
        u8col = a->g_gain;
    else if (iq_ci_eq(key, "b_gain"))
        u8col = a->b_gain;
    else if (iq_ci_eq(key, "skin_gain"))
        u8col = a->skin_gain;
    else {
        HAL_LOG_DBG("isp tuning: [static_sharpen] %s: no mapping", key);
        return;
    }

    if (u8col || u16col) {
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        if (u8col)
            iq_fill_u8(u8col, V5_ISP_ISO_NUM, ld->nums, n, 1);
        else
            iq_fill_u16(u16col, V5_ISP_ISO_NUM, ld->nums, n, sizeof(unsigned short));
    }
    ld->dirty |= IQ_SHARPEN;
    return;

past_row:
    HAL_LOG_WARN("isp tuning: [static_sharpen] %s: row past the array -- ignored", key);
}

/*
 * Defect pixel. The dialect names one set of columns; the attribute
 * carries one per WDR frame, and a linear-mode file means frame 0. Frames
 * 1..3 keep what the Get returned, which is the right answer for a mode
 * that never uses them.
 */
static void iq_sect_dpc(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_dp_frame_dynamic *f = &ld->dpc.frame_dynamic[0];
    int n;

    if (!iq_fetch(st, ld, IQ_DPC))
        return;

    if (iq_ci_eq(key, "enable"))
        ld->dpc.enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "op_type"))
        f->op_type = iq_num(val, 0) ? V5_ISP_OP_MANUAL : V5_ISP_OP_AUTO;
    else if (iq_ci_eq(key, "sup_twinkle_enable"))
        f->sup_twinkle_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "soft_thr"))
        f->soft_thr = (signed char)iq_clamp(iq_num(val, 0), -128, 127);
    else if (iq_ci_eq(key, "soft_slope"))
        f->soft_slope = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "bright_strength"))
        f->bright_strength = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "dark_strength"))
        f->dark_strength = (unsigned char)iq_clamp(iq_num(val, 0), 0, 255);
    else if (iq_ci_eq(key, "strength")) {
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        iq_fill_u8(f->auto_attr.strength, V5_ISP_ISO_NUM, ld->nums, n, 1);
    } else if (iq_ci_eq(key, "blend_ratio")) {
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        iq_fill_u8(f->auto_attr.blend_ratio, V5_ISP_ISO_NUM, ld->nums, n, 1);
    } else {
        HAL_LOG_DBG("isp tuning: [static_dpc] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_DPC;
}

static void iq_sect_ca(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_ca_attr *c = &ld->ca;
    int n;

    if (!iq_fetch(st, ld, IQ_CA))
        return;

    if (iq_ci_eq(key, "enable"))
        c->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "iso_ratio")) {
        n = iq_nums(val, ld->nums, V5_ISP_ISO_NUM);
        iq_fill_s32(c->ca.iso_ratio, V5_ISP_ISO_NUM, ld->nums, n);
    } else if (iq_ci_eq(key, "y_ratio_lut")) {
        if (!(n = iq_table(ld, "static_ca", key, val, V5_ISP_CA_LUT)))
            return;
        iq_fill_u32(c->ca.y_ratio_lut, V5_ISP_CA_LUT, ld->nums, n, sizeof(unsigned int));
    } else if (iq_ci_eq(key, "y_sat_lut")) {
        if (!(n = iq_table(ld, "static_ca", key, val, V5_ISP_CA_LUT)))
            return;
        iq_fill_u32(c->ca.y_sat_lut, V5_ISP_CA_LUT, ld->nums, n, sizeof(unsigned int));
    } else {
        HAL_LOG_DBG("isp tuning: [static_ca] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_CA;
}

static void iq_sect_pregamma(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    int n;

    if (!iq_fetch(st, ld, IQ_PREGAMMA))
        return;

    if (iq_ci_eq(key, "enable"))
        ld->pregamma.enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "table")) {
        /* 257 nodes of 20 bits. Long enough that it arrives as a dozen
         * continuations, which is exactly the case the exact-count rule
         * exists for. */
        if (!(n = iq_table(ld, "static_pregamma", key, val, V5_ISP_PREGAMMA_NODES)))
            return;
        iq_fill_u32(ld->pregamma.table, V5_ISP_PREGAMMA_NODES, ld->nums, n, sizeof(unsigned int));
    } else {
        HAL_LOG_DBG("isp tuning: [static_pregamma] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_PREGAMMA;
}

/*
 * Black level. `user_offset` is the four Bayer channels of frame 0, and
 * setting it means the user level rather than the calibrated one, so the
 * enable that selects it travels with the values.
 */
static void iq_sect_blc(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    int n;

    if (!iq_fetch(st, ld, IQ_BLC))
        return;

    if (iq_ci_eq(key, "enable"))
        ld->blc.user_black_level_en = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "user_offset")) {
        if (!(n = iq_table(ld, "static_blc", key, val, V5_ISP_BAYER_CHN)))
            return;
        iq_fill_u16(ld->blc.user_black_level[0], V5_ISP_BAYER_CHN, ld->nums, n,
                    sizeof(unsigned short));
    } else {
        HAL_LOG_DBG("isp tuning: [static_blc] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_BLC;
}

static void iq_sect_csc(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    v5_isp_csc_attr *c = &ld->csc;

    if (!iq_fetch(st, ld, IQ_CSC))
        return;

    if (iq_ci_eq(key, "enable"))
        c->enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "hue"))
        c->hue = (unsigned char)iq_clamp(iq_num(val, 50), 0, 100);
    else if (iq_ci_eq(key, "luma"))
        c->luma = (unsigned char)iq_clamp(iq_num(val, 50), 0, 100);
    else if (iq_ci_eq(key, "contrast"))
        c->contr = (unsigned char)iq_clamp(iq_num(val, 50), 0, 100);
    else if (iq_ci_eq(key, "saturation"))
        c->satu = (unsigned char)iq_clamp(iq_num(val, 50), 0, 100);
    else if (iq_ci_eq(key, "color_gamut"))
        c->color_gamut = (int)iq_clamp(iq_num(val, 0), 0, 3);
    else if (iq_ci_eq(key, "limited_range_en"))
        c->limited_range_en = iq_num(val, 0) ? 1 : 0;
    else {
        HAL_LOG_DBG("isp tuning: [static_csc] %s: no mapping", key);
        return;
    }
    ld->dirty |= IQ_CSC;
}

static void iq_sect_shading(hisi_state_t *st, hisi_iq_load *ld, const char *key, const char *val)
{
    if (!iq_fetch(st, ld, IQ_SHADING))
        return;

    if (iq_ci_eq(key, "enable"))
        ld->shading.enable = iq_num(val, 0) ? 1 : 0;
    else if (iq_ci_eq(key, "mesh_strength"))
        ld->shading.mesh_strength = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else if (iq_ci_eq(key, "blend_ratio"))
        ld->shading.blend_ratio = (unsigned short)iq_clamp(iq_num(val, 0), 0, 65535);
    else {
        HAL_LOG_DBG("isp tuning: [static_shading] %s: no mapping", key);
        return;
    }
    /*
     * The mesh itself is calibration -- ss_mpi_isp_set_mesh_shading_gain_lut_attr,
     * a per-lens table this file does not carry -- so what applies here is
     * the strength over whatever mesh the sensor library loaded.
     */
    ld->dirty |= IQ_SHADING;
}

/* ---------------- the summary line ---------------- */

/*
 * Record a section name once. The caller hands the same name in for every
 * key the section carries, so the search is by exact name rather than the
 * substring test gen4 gets away with on a shorter list.
 */
static void iq_note_into(char list[][IQ_NOTE_LEN], int *n, int *more, const char *sect)
{
    int i;

    for (i = 0; i < *n; i++) {
        if (iq_ci_eq(list[i], sect))
            return;
    }
    if (*n >= IQ_NOTE_MAX) {
        (*more)++;
        return;
    }
    snprintf(list[*n], IQ_NOTE_LEN, "%s", sect);
    (*n)++;
}

static void iq_note_skip(hisi_iq_load *ld, const char *sect)
{
    iq_note_into(ld->skipped, &ld->skipped_n, &ld->skipped_more, sect);
}

/* The list, joined for the log line. Returns buf so it can be an argument. */
static const char *iq_note_join(char list[][IQ_NOTE_LEN], int n, int more, char *buf, size_t cap)
{
    size_t have = 0;
    int i;

    buf[0] = '\0';
    for (i = 0; i < n && have + 1 < cap; i++)
        have += (size_t)snprintf(buf + have, cap - have, "%s%s", have ? " " : "", list[i]);
    if (more && have + 1 < cap)
        snprintf(buf + have, cap - have, " +%d more", more);
    return buf;
}

/* ---------------- [module_state] ---------------- */

/*
 * The one part of this dialect that is not a value. [module_state] is the
 * author's list of which of the file's own sections are meant to be
 * applied, and the vendor's loader opens every ot_scene_set_* with the
 * matching flag and returns without touching the ISP when it is clear. So
 * a section sitting under a "0" is documentation, not tuning.
 *
 * Two details in the map below are the vendor's, not ours:
 *
 *   - [static_aerouteex] has no flag of its own. The vendor's
 *     ot_scene_set_static_ae writes the route and the exposure attribute
 *     together, under bStaticAE.
 *   - the AE weight table is nested inside that same function, which
 *     writes the table only if bAeWeightTab, so [static_aeweight] needs
 *     both flags and not just its own.
 *
 * Inside a [module_state] that is present, a flag that is not listed is
 * off: the vendor's parser writes only the flags it finds into a struct
 * that started zeroed. A file that names the section has named its set.
 */
enum {
    MS_STATIC_AE = 1u << 0,
    MS_AE_WEIGHT = 1u << 1,
    MS_STATIC_CCM = 1u << 2,
    MS_STATIC_SAT = 1u << 3,
    MS_STATIC_LDCI = 1u << 4,
    MS_STATIC_DRC = 1u << 5,
    MS_STATIC_DM = 1u << 6,
    MS_STATIC_NR = 1u << 7,
    MS_STATIC_DEHAZE = 1u << 8,
    MS_STATIC_SHARPEN = 1u << 9,
    MS_STATIC_DPC = 1u << 10,
    MS_STATIC_CA = 1u << 11,
    MS_STATIC_PREGAMMA = 1u << 12,
    MS_STATIC_BLC = 1u << 13,
    MS_STATIC_CSC = 1u << 14,
    MS_STATIC_SHADING = 1u << 15,
    MS_ALL = (1u << 16) - 1,
};

static const struct {
    const char *key;
    unsigned int bit;
} iq_state_keys[] = {
    {"bStaticAE", MS_STATIC_AE},
    {"bAeWeightTab", MS_AE_WEIGHT},
    {"bStaticCCM", MS_STATIC_CCM},
    {"bStaticSaturation", MS_STATIC_SAT},
    {"bStaticLdci", MS_STATIC_LDCI},
    {"bStaticDRC", MS_STATIC_DRC},
    {"bStaticDemosaic", MS_STATIC_DM},
    {"bStaticNr", MS_STATIC_NR},
    {"bStaticDehaze", MS_STATIC_DEHAZE},
    {"bStaticSharpen", MS_STATIC_SHARPEN},
    {"bStaticDPC", MS_STATIC_DPC},
    {"bStaticCa", MS_STATIC_CA},
    {"bStaticPreGamma", MS_STATIC_PREGAMMA},
    {"bStaticBlc", MS_STATIC_BLC},
    {"bStaticCSC", MS_STATIC_CSC},
    {"bStaticShading", MS_STATIC_SHADING},
};

/* Which flags a section needs; 0 for one the mask does not cover. */
static unsigned int iq_state_bits(const char *s)
{
    if (iq_ci_eq(s, "static_ae") || iq_ci_eq(s, "static_aerouteex"))
        return MS_STATIC_AE;
    if (iq_ci_eq(s, "static_aeweight"))
        return MS_STATIC_AE | MS_AE_WEIGHT;
    if (iq_ci_eq(s, "static_ccm"))
        return MS_STATIC_CCM;
    if (iq_ci_eq(s, "static_saturation"))
        return MS_STATIC_SAT;
    if (iq_ci_eq(s, "static_ldci"))
        return MS_STATIC_LDCI;
    if (iq_ci_eq(s, "static_drc"))
        return MS_STATIC_DRC;
    if (iq_ci_eq(s, "static_dm"))
        return MS_STATIC_DM;
    if (iq_ci_eq(s, "static_nr"))
        return MS_STATIC_NR;
    if (iq_ci_eq(s, "static_dehaze"))
        return MS_STATIC_DEHAZE;
    if (iq_ci_eq(s, "static_sharpen"))
        return MS_STATIC_SHARPEN;
    if (iq_ci_eq(s, "static_dpc"))
        return MS_STATIC_DPC;
    if (iq_ci_eq(s, "static_ca"))
        return MS_STATIC_CA;
    if (iq_ci_eq(s, "static_pregamma"))
        return MS_STATIC_PREGAMMA;
    if (iq_ci_eq(s, "static_blc"))
        return MS_STATIC_BLC;
    if (iq_ci_eq(s, "static_csc"))
        return MS_STATIC_CSC;
    if (iq_ci_eq(s, "static_shading"))
        return MS_STATIC_SHADING;
    return 0;
}

/*
 * A pass of its own, before the one that applies. Every file the vendor
 * writes puts [module_state] first, so reading it as it goes by would
 * work today -- but a mask that only held when the section came first
 * would be a trap for the file that puts it last, and the whole file is
 * one short read. Returns MS_ALL for a file with no such section.
 */
static unsigned int iq_scan_state(hisi_iq_reader *r)
{
    unsigned int mask = 0;
    bool seen = false;
    size_t i;

    while (iq_next(r)) {
        if (!iq_ci_eq(r->sect, "module_state"))
            continue;
        seen = true;
        for (i = 0; i < sizeof(iq_state_keys) / sizeof(iq_state_keys[0]); i++) {
            if (!iq_ci_eq(r->key, iq_state_keys[i].key))
                continue;
            if (iq_num(r->val, 0))
                mask |= iq_state_keys[i].bit;
            break;
        }
    }

    rewind(r->f);
    r->sect[0] = '\0';
    r->key[0] = '\0';
    /* The *_warned flags stay set: the second pass reads the same file and
     * a warning it already earned should not be printed twice. */
    return seen ? mask : MS_ALL;
}

static void iq_dispatch(hisi_state_t *st, hisi_iq_load *ld, hisi_iq_reader *r)
{
    const char *s = r->sect;
    unsigned int bits;

    /* No section: either keys before the first header or the fallout of a
     * malformed one. Either way they belong to no module. */
    if (!s[0])
        return;

    /* Read in full by iq_scan_state, before this pass began. */
    if (iq_ci_eq(s, "module_state"))
        return;

    /* What the file says it meant. A section the mask does not cover
     * needs no flag and keeps going. */
    bits = iq_state_bits(s);
    if (bits && (ld->state & bits) != bits) {
        iq_note_into(ld->disabled, &ld->disabled_n, &ld->disabled_more, s);
        return;
    }

    if (iq_ci_eq(s, "static_ae"))
        iq_sect_static_ae(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_aerouteex"))
        iq_sect_route_ex(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_aeweight"))
        iq_sect_aeweight(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_ccm"))
        iq_sect_ccm(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_saturation"))
        iq_sect_saturation(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_ldci"))
        iq_sect_ldci(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_drc"))
        iq_sect_static_drc(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_dm"))
        iq_sect_dm(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_nr"))
        iq_sect_nr(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_dehaze"))
        iq_sect_dehaze(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_sharpen"))
        iq_sect_sharpen(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_dpc"))
        iq_sect_dpc(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_ca"))
        iq_sect_ca(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_pregamma"))
        iq_sect_pregamma(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_blc"))
        iq_sect_blc(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_csc"))
        iq_sect_csc(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "static_shading"))
        iq_sect_shading(st, ld, r->key, r->val);
    else if (iq_ci_eq(s, "dynamic_linear_drc") || iq_ci_eq(s, "dynamic_dehaze") ||
             iq_ci_eq(s, "dynamic_gamma")) {
        /* Tables over an axis rather than values; hal_dyn.c keeps them and
         * walks them off the AE tick. No [module_state] flag gates these
         * three -- the vendor's own bDynamic* bits are the sample's thread
         * switches, and a section present here is a section meant. */
        if (!hisi_dyn_key(st, s, r->key, r->val))
            HAL_LOG_DBG("isp tuning: [%s] %s: no mapping", s, r->key);
    } else
        iq_note_skip(ld, s);
}

/* ---------------- apply ---------------- */

void hisi_isp_tune_resolve(hisi_state_t *st)
{
    if (st->tune_resolved)
        return;
    st->tune_resolved = true;
    v5_isp_tune_load(&st->tune, &st->libs);
}

/*
 * The apply order matters in one place: the exposure attribute carries
 * ae_route_ex_valid, and the route must be in before the flag that says
 * to use it. Everything after is independent.
 */
static void hisi_isp_apply_tuning(hisi_state_t *st)
{
    static const unsigned int apply_order[] = {
        IQ_ROUTE, IQ_EXP,  IQ_STAT, IQ_BLC, IQ_PREGAMMA, IQ_NR,   IQ_DM,  IQ_DPC,    IQ_SHADING,
        IQ_CCM,   IQ_TONE, IQ_SAT,  IQ_CA,  IQ_SHARPEN,  IQ_LDCI, IQ_DRC, IQ_DEHAZE, IQ_CSC,
    };
    hisi_iq_reader r;
    hisi_iq_load *ld;
    int applied = 0, failed = 0;
    unsigned int i;

    hisi_isp_tune_resolve(st);
    /* A pinned knob is lifted back to its baseline first, so the file
     * lands on what the tuner meant rather than on the knob. */
    hisi_knob_before_load(st);

    memset(&r, 0, sizeof(r));
    r.path = st->iq_file;
    if (!(r.f = fopen(st->iq_file, "r"))) {
        HAL_LOG_WARN("isp tuning: %s vanished between resolve and load", st->iq_file);
        return;
    }
    if (!(ld = calloc(1, sizeof(*ld))) || !(r.val = malloc(HISI_IQ_VAL_MAX)) ||
        !(r.line = malloc(HISI_IQ_LINE_MAX))) {
        HAL_LOG_WARN("isp tuning: out of memory; running untuned");
        free(r.line);
        free(r.val);
        free(ld);
        fclose(r.f);
        return;
    }

    ld->state = iq_scan_state(&r);

    while (iq_next(&r))
        iq_dispatch(st, ld, &r);
    fclose(r.f);

    /* A route section without an explicit ae_route_ex_valid still needs
     * the flag, or the route the file spent five lines on is dead letter. */
    if (ld->route_seen && (ld->have & IQ_EXP)) {
        ld->exp.ae_route_ex_valid = 1;
        ld->dirty |= IQ_EXP;
    }

    for (i = 0; i < sizeof(apply_order) / sizeof(apply_order[0]); i++) {
        unsigned int bit = apply_order[i];
        int (*set)(int, const void *);
        int ret;

        if (!(ld->dirty & bit) || !(ld->have & bit))
            continue;

        if (!(set = iq_setter(st, bit))) {
            HAL_LOG_WARN("isp tuning: ss_mpi_isp_set_%s unresolved -- module keeps its defaults",
                         iq_mod_name(bit));
            failed++;
            continue;
        }
        ret = set(HISI_VI_PIPE, iq_struct_of(ld, bit));
        if (ret) {
            HAL_LOG_WARN("isp tuning: ss_mpi_isp_set_%s failed: 0x%x -- module keeps its defaults",
                         iq_mod_name(bit), ret);
            failed++;
        } else {
            HAL_LOG_DBG("isp tuning: ss_mpi_isp_set_%s applied", iq_mod_name(bit));
            applied++;
        }
    }

    /* The engines, over the static values just written. */
    {
        char note[128];
        int dyn_failed = 0;

        applied += hisi_dyn_apply(st, &dyn_failed, note, sizeof(note));
        if (dyn_failed) {
            failed += dyn_failed;
            iq_note_skip(ld, note);
        }
    }

    {
        char skipped[IQ_NOTE_MAX * IQ_NOTE_LEN];
        char disabled[IQ_NOTE_MAX * IQ_NOTE_LEN];

        iq_note_join(ld->skipped, ld->skipped_n, ld->skipped_more, skipped, sizeof(skipped));
        iq_note_join(ld->disabled, ld->disabled_n, ld->disabled_more, disabled, sizeof(disabled));

        if (failed)
            HAL_LOG_WARN("isp tuning: %s: %d modules applied, %d failed%s%s%s%s", st->iq_file,
                         applied, failed, skipped[0] ? "; skipped: " : "", skipped,
                         disabled[0] ? "; [module_state] off: " : "", disabled);
        else
            HAL_LOG_INFO("isp tuning: %s: %d modules applied%s%s%s%s", st->iq_file, applied,
                         skipped[0] ? "; skipped: " : "", skipped,
                         disabled[0] ? "; [module_state] off: " : "", disabled);
    }

    free(r.line);
    free(r.val);
    free(ld);

    /* The [image] knobs, over the baseline the file just laid down. */
    hisi_knob_reapply(st);
}

/*
 * hisi_isp_note_frame -- one-shot hook off the encoder's frame loop.
 *
 * rvd runs an encoder thread per stream and every one reaches here, so
 * the latch is atomic and exactly one thread does the work. A restart
 * rebuilds the state from zero, which clears the latch, so the tuning is
 * reloaded on the next run.
 */
void hisi_isp_note_frame(hisi_state_t *st)
{
    if (__atomic_test_and_set(&st->iq_load_started, __ATOMIC_ACQ_REL)) {
        /* Every frame after the load: the AE tick behind the dynamic
         * sections, which rate-limits itself and does nothing until the
         * load has armed an engine. */
        hisi_dyn_tick(st);
        return;
    }

    if (st->iq_file[0])
        hisi_isp_apply_tuning(st);
}
