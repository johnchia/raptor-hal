/*
 * hisi_v5/hisi_sensor.c -- sensor mode discovery for the HiSilicon gen5 HAL
 *
 * Two jobs, both in service of the same rule as gen4's: no sensor table in
 * the code.
 *
 *   1. Read a sensor mode INI and turn it into everything bring-up needs
 *      that raptor's own config does not carry -- the library and its
 *      object symbol, the MIPI lane map, the RAW bit depth, the Bayer
 *      order, the sensor's output size and frame rate, the I2C bus, and the
 *      whole VI device attribute.
 *
 *   2. Find the object symbol inside a loaded library, for when the INI
 *      does not say or says the wrong thing.
 *
 * WHERE THE FILES COME FROM, and this is the difference from gen4. A gen4
 * OpenIPC image ships /etc/sensors from the vendor's osdrv package, so
 * hisi_v4/hisi_sensor.c reads a file that is already on the board. **The
 * cv6xx image ships none**: opensdk builds the sensor libraries from source
 * and has no config directory at all. So raptor carries its own set, under
 * /etc/sensors like the vendor's, and the search order below still looks in
 * the vendor's place first so a board that does have them wins.
 *
 * The values in raptor's files are the vendor's own, read out of the
 * Hi3516CV610 PQTools configs -- configs/<sensor>/<sensor>_<mode>.ini,
 * which are in the V5 dialect and name ot_isp_pub_attr, combo_dev_attr_t
 * and ot_vi_dev_attr fields directly. Read, not copied: the layout here is
 * gen4's INI dialect so one reader serves both generations.
 *
 * THREE THINGS THE V5 LAYOUT ADDS.
 *
 *   - **The die caps the geometry.** One MPP serves CV610, CV610_10B and
 *     CV608, and clk_cfg.c clocks the CV608's ISP at 148.5 MHz against the
 *     CV610's 198. Every vendor config therefore ships in three variants,
 *     and the _608 one is 2304x1296 where the others are 4M or 5M. Rather
 *     than three files per sensor, a mode file here carries per-die
 *     override sections: a key in [vi_dev.hi3516cv608] beats the same key
 *     in [vi_dev]. hisi_state_t::chip_name selects the suffix.
 *
 *   - **The I2C bus is explicit.** gen4 assumed it; V5's sensor library
 *     takes it through pfn_set_bus_info before registration.
 *
 *   - **The object symbol is g_sns_<name>_obj**, not gen4's stSns<Name>Obj,
 *     and it cannot be derived from the file name: libsns_sp2308.so exports
 *     g_sns_os02m10_obj. See hisi_sensor_obj_find.
 *
 * The parser is deliberately small and local rather than raptor-common's:
 * raptor-hal links nothing outside itself, and that property is what makes
 * the backend host-testable. It is a near-copy of gen4's, which is the
 * right amount of duplication for two files that must not drift together
 * -- the dialects are the same today and the vendor is under no obligation
 * to keep them so.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <ctype.h>
#include <dirent.h>
#include <elf.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/* ================================================================
 * A SMALL INI READER
 *
 * Enough for the vendor's dialect and no more: [sections], key = value,
 * ';' comments to end of line, values that may be decimal, hex,
 * TRUE/FALSE, a symbolic enumerator name, or a '|'-separated list.
 * ================================================================ */

#define HISI_INI_MAX_ENTRIES 160
#define HISI_INI_MAX_LINE 256

typedef struct {
    char section[40];
    char key[40];
    char value[96];
} hisi_ini_entry;

typedef struct {
    hisi_ini_entry entry[HISI_INI_MAX_ENTRIES];
    int count;
    /* The [<section>.<suffix>] override suffix in force, e.g.
     * "hi3516cv608". Empty means the file's defaults. */
    char die[24];
} hisi_ini;

/*
 * Case-insensitive compare, written out rather than reached for.
 *
 * The library version lives in <strings.h> behind a POSIX feature macro,
 * and raptor-hal compiles -std=c11 -- widening the feature set of a
 * translation unit for one three-line function invites the next one in for
 * free.
 */
static int hisi_ci_eq(const char *a, const char *b)
{
    for (; *a && *b; a++, b++) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
            return 0;
    }
    return *a == '\0' && *b == '\0';
}

static char *hisi_ini_trim(char *s)
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

static int hisi_ini_read(hisi_ini *ini, const char *path)
{
    char line[HISI_INI_MAX_LINE];
    char section[40] = "";
    FILE *f;

    memset(ini, 0, sizeof(*ini));

    if (!(f = fopen(path, "r")))
        return RSS_ERR_NOENT;

    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        char *eq;
        char *comment;

        /* A line longer than the buffer arrives in pieces, and a later
         * piece would parse as its own key. Drop the tail. */
        if (!strchr(line, '\n') && !feof(f)) {
            int c;

            while ((c = fgetc(f)) != EOF && c != '\n')
                ;
        }

        if ((comment = strchr(p, ';')))
            *comment = '\0';
        if ((comment = strchr(p, '#')))
            *comment = '\0';

        p = hisi_ini_trim(p);
        if (!*p)
            continue;

        if (*p == '[') {
            char *close = strchr(p, ']');

            if (!close)
                continue;
            *close = '\0';
            snprintf(section, sizeof(section), "%s", hisi_ini_trim(p + 1));
            continue;
        }

        if (!(eq = strchr(p, '=')))
            continue;
        *eq = '\0';

        if (ini->count >= HISI_INI_MAX_ENTRIES) {
            HAL_LOG_WARN("sensor ini: %s has more than %d entries; the rest are ignored", path,
                         HISI_INI_MAX_ENTRIES);
            break;
        }

        snprintf(ini->entry[ini->count].section, sizeof(ini->entry[0].section), "%s", section);
        snprintf(ini->entry[ini->count].key, sizeof(ini->entry[0].key), "%s", hisi_ini_trim(p));
        snprintf(ini->entry[ini->count].value, sizeof(ini->entry[0].value), "%s",
                 hisi_ini_trim(eq + 1));
        ini->count++;
    }

    fclose(f);
    return RSS_OK;
}

/* One exact section/key lookup. */
static const char *hisi_ini_find(const hisi_ini *ini, const char *section, const char *key)
{
    int i;

    for (i = 0; i < ini->count; i++) {
        if (hisi_ci_eq(ini->entry[i].section, section) && hisi_ci_eq(ini->entry[i].key, key))
            return ini->entry[i].value;
    }
    return NULL;
}

/*
 * hisi_ini_str -- the die-aware lookup every accessor goes through.
 *
 * [vi_dev.hi3516cv608] beats [vi_dev] for the same key. Only the keys the
 * die actually changes need repeating, which is what keeps a mode file
 * readable: the CV608 block of a typical sensor is two lines.
 */
static const char *hisi_ini_str(const hisi_ini *ini, const char *section, const char *key,
                                const char *fallback)
{
    const char *v;

    if (ini->die[0]) {
        char scoped[64];

        snprintf(scoped, sizeof(scoped), "%s.%s", section, ini->die);
        if ((v = hisi_ini_find(ini, scoped, key)))
            return v;
    }

    if ((v = hisi_ini_find(ini, section, key)))
        return v;

    return fallback;
}

/*
 * A value that may be decimal, 0x-hex, or TRUE/FALSE.
 *
 * strtol with base 0 covers the first two; the booleans are spelt out
 * because the vendor's files use them for td_bool fields and strtol would
 * quietly return 0 for both.
 */
static int hisi_ini_int(const hisi_ini *ini, const char *section, const char *key, int fallback)
{
    const char *v = hisi_ini_str(ini, section, key, NULL);
    char *end;
    long n;

    if (!v || !*v)
        return fallback;
    if (hisi_ci_eq(v, "true"))
        return 1;
    if (hisi_ci_eq(v, "false"))
        return 0;

    n = strtol(v, &end, 0);
    if (end == v)
        return fallback;
    return (int)n;
}

static float hisi_ini_float(const hisi_ini *ini, const char *section, const char *key,
                            float fallback)
{
    const char *v = hisi_ini_str(ini, section, key, NULL);
    char *end;
    float f;

    if (!v || !*v)
        return fallback;
    f = strtof(v, &end);
    if (end == v)
        return fallback;
    return f;
}

/*
 * A value that may be a symbolic enumerator name or the number itself.
 *
 * The vendor's own files mix the two in the same file -- "Isp_Bayer =
 * BAYER_RGGB" beside "input_mode = 0" -- and raptor's files follow suit,
 * spelling out whatever a bare number would make unreadable.
 */
typedef struct {
    const char *name;
    int value;
} hisi_enum;

static int hisi_ini_enum_val(const hisi_ini *ini, const char *section, const char *key,
                             const hisi_enum *table, int fallback)
{
    const char *v = hisi_ini_str(ini, section, key, NULL);
    const hisi_enum *e;

    if (!v || !*v)
        return fallback;

    for (e = table; e->name; e++) {
        if (hisi_ci_eq(v, e->name))
            return e->value;
    }

    return hisi_ini_int(ini, section, key, fallback);
}

static const hisi_enum hisi_enum_bayer[] = {
    {"BAYER_RGGB", V5_BAYER_RGGB}, {"BAYER_GRBG", V5_BAYER_GRBG}, {"BAYER_GBRG", V5_BAYER_GBRG},
    {"BAYER_BGGR", V5_BAYER_BGGR}, {"RGGB", V5_BAYER_RGGB},       {"GRBG", V5_BAYER_GRBG},
    {"GBRG", V5_BAYER_GBRG},       {"BGGR", V5_BAYER_BGGR},       {NULL, 0},
};

static const hisi_enum hisi_enum_wdr[] = {
    {"WDR_MODE_NONE", V5_WDR_MODE_NONE},
    {"OT_WDR_MODE_NONE", V5_WDR_MODE_NONE},
    {"WDR_MODE_BUILT_IN", V5_WDR_MODE_BUILT_IN},
    {"WDR_MODE_2To1_LINE", V5_WDR_MODE_2To1_LINE},
    {"WDR_MODE_2To1_FRAME", V5_WDR_MODE_2To1_FRAME},
    {NULL, 0},
};

static const hisi_enum hisi_enum_input_mode[] = {
    {"INPUT_MODE_MIPI", V5_INPUT_MODE_MIPI},
    {"INPUT_MODE_SUBLVDS", V5_INPUT_MODE_SUBLVDS},
    {"INPUT_MODE_LVDS", V5_INPUT_MODE_LVDS},
    {"INPUT_MODE_HISPI", V5_INPUT_MODE_HISPI},
    {NULL, 0},
};

static const hisi_enum hisi_enum_vi_mode[] = {
    {"VI_MODE_BT656", V5_VI_INTF_MODE_BT656},
    {"VI_MODE_BT601", V5_VI_INTF_MODE_BT601},
    {"VI_MODE_DIGITAL_CAMERA", V5_VI_INTF_MODE_DC},
    {"VI_MODE_BT1120", V5_VI_INTF_MODE_BT1120},
    {"VI_MODE_MIPI", V5_VI_INTF_MODE_MIPI},
    {"OT_VI_INTF_MODE_MIPI", V5_VI_INTF_MODE_MIPI},
    {NULL, 0},
};

static const hisi_enum hisi_enum_work_mode[] = {
    {"VI_WORK_MODE_1Multiplex", V5_VI_WORK_MODE_MULTIPLEX_1},
    {"VI_WORK_MODE_2Multiplex", V5_VI_WORK_MODE_MULTIPLEX_2},
    {"VI_WORK_MODE_4Multiplex", V5_VI_WORK_MODE_MULTIPLEX_4},
    {NULL, 0},
};

static const hisi_enum hisi_enum_scan_mode[] = {
    {"VI_SCAN_PROGRESSIVE", V5_VI_SCAN_PROGRESSIVE},
    {"VI_SCAN_INTERLACED", V5_VI_SCAN_INTERLACED},
    {NULL, 0},
};

static const hisi_enum hisi_enum_data_seq[] = {
    {"VI_DATA_SEQ_VUVU", V5_VI_DATA_SEQ_VUVU},
    {"VI_DATA_SEQ_UVUV", V5_VI_DATA_SEQ_UVUV},
    {"VI_DATA_SEQ_UYVY", V5_VI_DATA_SEQ_UYVY},
    {"VI_DATA_SEQ_VYUY", V5_VI_DATA_SEQ_VYUY},
    {"VI_DATA_SEQ_YUYV", V5_VI_DATA_SEQ_YUYV},
    {"VI_DATA_SEQ_YVYU", V5_VI_DATA_SEQ_YVYU},
    {NULL, 0},
};

/*
 * hisi_parse_lane_map -- "0|1|-1|-1|" into a short array.
 *
 * Anything the string does not supply stays -1, which is the disable value
 * rather than lane 0: a short list must not enable lanes it did not name.
 */
static void hisi_parse_lane_map(const char *s, short *out, int count)
{
    int i;

    for (i = 0; i < count; i++)
        out[i] = -1;

    if (!s)
        return;

    for (i = 0; i < count && *s; i++) {
        char *end;
        long n = strtol(s, &end, 0);

        if (end == s)
            break;
        out[i] = (short)n;
        s = end;
        while (*s == ' ' || *s == '\t')
            s++;
        if (*s == '|')
            s++;
    }
}

/* ================================================================
 * DERIVED FORMATS
 * ================================================================ */

/*
 * raw_bitness -> the two enumerations that have to agree with it.
 *
 * One key rather than three, because three keys can disagree and a MIPI
 * receiver told 10-bit feeding a VI told 12-bit produces a picture that is
 * merely wrong rather than absent.
 */
static v5_mipi_data_type hisi_mipi_data_type(int bits)
{
    switch (bits) {
    case 8:
        return V5_DATA_TYPE_RAW_8BIT;
    case 10:
        return V5_DATA_TYPE_RAW_10BIT;
    case 14:
        return V5_DATA_TYPE_RAW_14BIT;
    case 12:
        return V5_DATA_TYPE_RAW_12BIT;
    default:
        /* 16-bit has no entry in the 1.0.2.0 data_type_t -- see v5_mipi.h.
         * Falling through to 12 rather than passing 4 matters: 4 is
         * YUV420_8BIT_NORMAL on this drop, and the receiver would be
         * configured for a format the sensor is not sending. */
        HAL_LOG_WARN("sensor: RAW%d is not a MIPI data type this MPP has; using RAW12", bits);
        return V5_DATA_TYPE_RAW_12BIT;
    }
}

static v5_pixel_format hisi_bayer_pixel_format(int bits)
{
    switch (bits) {
    case 8:
        return V5_PIXEL_FORMAT_RGB_BAYER_8BPP;
    case 10:
        return V5_PIXEL_FORMAT_RGB_BAYER_10BPP;
    case 14:
        return V5_PIXEL_FORMAT_RGB_BAYER_14BPP;
    case 16:
        return V5_PIXEL_FORMAT_RGB_BAYER_16BPP;
    case 12:
    default:
        return V5_PIXEL_FORMAT_RGB_BAYER_12BPP;
    }
}

/* ================================================================
 * FINDING THE FILE
 * ================================================================ */

/*
 * Where mode files live, most specific first.
 *
 * /etc/sensors is the vendor's own directory on a gen4 image and the one
 * raptor installs into on a cv6xx image, so a board that has been given the
 * vendor's files wins over raptor's fallback copy in /usr/share.
 */
static const char *const hisi_sensor_dirs[] = {
    "/etc/sensors",
    "/usr/share/sensors",
};

/*
 * hisi_sensor_ini_find -- the best-matching file for a sensor name.
 *
 * Ranked rather than exact because a board may carry the vendor's naming
 * ("os04d10_i2c_4M.ini") beside raptor's ("os04d10.ini"): rank 0 is a file
 * whose name is the sensor or begins with it, rank 1 merely contains it.
 * Ties break on the lexically first path so the choice is stable, and every
 * additional match is logged -- an ambiguous set is a configuration
 * problem the operator can fix and raptor cannot.
 */
static int hisi_sensor_ini_find(const char *sensor_name, char *out, size_t out_len)
{
    char lower[64];
    char best[192] = "";
    int best_rank = 2;
    int matches = 0;
    size_t d, i, n = strlen(sensor_name);

    if (n >= sizeof(lower))
        return RSS_ERR_INVAL;
    for (i = 0; i < n; i++)
        lower[i] = (char)tolower((unsigned char)sensor_name[i]);
    lower[n] = '\0';

    for (d = 0; d < sizeof(hisi_sensor_dirs) / sizeof(hisi_sensor_dirs[0]); d++) {
        struct dirent *de;
        DIR *dir = opendir(hisi_sensor_dirs[d]);

        if (!dir)
            continue;

        while ((de = readdir(dir))) {
            /* Two bounded copies of the entry name, one verbatim for the
             * path and one lowercased for the match. Copying first is what
             * keeps the path's own bound provable: d_name is declared 256
             * wide and the compiler has no other way to know this loop has
             * already rejected anything longer. */
            char entry[128];
            char name[128];
            char path[192];
            size_t len = strlen(de->d_name);
            size_t j;
            int rank;

            if (len < 5 || len >= sizeof(entry))
                continue;
            if (!hisi_ci_eq(de->d_name + len - 4, ".ini"))
                continue;

            for (j = 0; j < len; j++) {
                entry[j] = de->d_name[j];
                name[j] = (char)tolower((unsigned char)de->d_name[j]);
            }
            entry[len] = '\0';
            name[len] = '\0';
            if (!strstr(name, lower))
                continue;

            snprintf(path, sizeof(path), "%s/%s", hisi_sensor_dirs[d], entry);
            matches++;

            rank = (strncmp(name, lower, n) == 0 && (name[n] == '_' || name[n] == '.')) ? 0 : 1;
            if (rank < best_rank || (rank == best_rank && (!best[0] || strcmp(path, best) < 0))) {
                best_rank = rank;
                snprintf(best, sizeof(best), "%s", path);
            } else {
                HAL_LOG_INFO("sensor ini: %s also matches \"%s\"", path, sensor_name);
            }
        }
        closedir(dir);
    }

    if (!best[0])
        return RSS_ERR_NOENT;
    if (matches > 1)
        HAL_LOG_WARN("sensor ini: %d files match \"%s\"; using %s. Remove the others to choose.",
                     matches, sensor_name, best);

    snprintf(out, out_len, "%s", best);
    return RSS_OK;
}

/* ================================================================
 * FINDING THE SENSOR'S NAME
 * ================================================================ */

/*
 * hisi_sensor_detect -- the sensor the kernel side was loaded for.
 *
 * V5 has no /proc entry naming the sensor, and the two places gen4 looked
 * both fail here:
 *
 *   - /sys/module/open_sys_config/parameters/sensors reads back **"sns0"**.
 *     The parameter is passed as "sns0=os04d10,sns1=..." and the kernel's
 *     own copy is truncated at the '=' -- checked with od on the board --
 *     so the name is simply not there. It is still parsed, in case a build
 *     passes a bare name, and a value of "sns0"/"sns1" is rejected.
 *
 *   - There is no /proc/jz/sensor equivalent at all.
 *
 * What does carry the name on an OpenIPC image is the **hostname**, which
 * its build writes as "openipc-<soc>-<sensor>-<id>"
 * ("openipc-hi3516cv608-os04d10-9205"). That is a convention rather than an
 * interface, so it is the last resort and it says so in the log: the right
 * answer is a sensor name in raptor's config.
 *
 * Only the first name is used: the parameter is a list on a dual-sensor
 * board, and this backend drives one.
 */
static int hisi_sensor_from_module(char *out, size_t out_len, char *src, size_t src_len)
{
    static const char *suffix = "sys_config";
    DIR *dir = opendir("/sys/module");
    struct dirent *de;
    int ret = RSS_ERR_NOENT;

    if (!dir)
        return RSS_ERR_NOENT;

    while (ret != RSS_OK && (de = readdir(dir))) {
        char module[128];
        char path[192];
        char line[128];
        size_t n = strlen(de->d_name);
        size_t len;
        FILE *f;
        char *v, *e;

        /* Bounded copy first: d_name is declared 256 wide, so the compiler
         * cannot otherwise prove the path below fits. */
        if (n < strlen(suffix) || n >= sizeof(module))
            continue;
        if (strcmp(de->d_name + n - strlen(suffix), suffix) != 0)
            continue;
        memcpy(module, de->d_name, n);
        module[n] = '\0';
        snprintf(path, sizeof(path), "/sys/module/%s/parameters/sensors", module);
        if (!(f = fopen(path, "r")))
            continue;
        if (!fgets(line, sizeof(line), f))
            line[0] = '\0';
        fclose(f);

        for (v = line; *v == ' ' || *v == '\t'; v++)
            ;
        /* "sns0=os04d10" if the kernel kept the whole thing. */
        if ((e = strchr(v, '=')))
            v = e + 1;
        for (e = v; *e && *e != ',' && *e != ' ' && *e != '\t' && *e != '\r' && *e != '\n'; e++)
            ;
        len = (size_t)(e - v);
        if (!len || len >= out_len)
            continue;
        if ((len == 7 && strncmp(v, "unknown", 7) == 0) || (len == 4 && strncmp(v, "sns", 3) == 0))
            continue;
        memcpy(out, v, len);
        out[len] = '\0';
        snprintf(src, src_len, "%s", path);
        ret = RSS_OK;
    }
    closedir(dir);
    return ret;
}

static int hisi_sensor_from_hostname(char *out, size_t out_len, char *src, size_t src_len)
{
    static const char *path = "/proc/sys/kernel/hostname";
    char line[128];
    char *field[8];
    int n = 0;
    char *p;
    FILE *f;

    if (!(f = fopen(path, "r")))
        return RSS_ERR_NOENT;
    if (!fgets(line, sizeof(line), f))
        line[0] = '\0';
    fclose(f);

    /* "openipc-hi3516cv608-os04d10-9205": four fields, sensor third. */
    for (p = hisi_ini_trim(line); n < 8;) {
        char *dash = strchr(p, '-');

        field[n++] = p;
        if (!dash)
            break;
        *dash = '\0';
        p = dash + 1;
    }

    if (n != 4 || !hisi_ci_eq(field[0], "openipc") || !field[2][0] || strlen(field[2]) >= out_len)
        return RSS_ERR_NOENT;

    snprintf(out, out_len, "%s", field[2]);
    snprintf(src, src_len, "%s", path);
    return RSS_OK;
}

/* ================================================================
 * FINDING THE OBJECT
 * ================================================================ */

/*
 * hisi_sensor_obj_scan -- any g_sns_*_obj in a library's dynamic symbols.
 *
 * dlsym answers a name; nothing in libdl enumerates. So the library is
 * reopened as a file and its .dynsym walked -- the same trick gen4 uses,
 * and needed for the same reason plus a sharper one: libsns_sp2308.so
 * exports g_sns_os02m10_obj, so the file name is not a source of the symbol
 * at all.
 *
 * Writes the name it found into `out` and leaves the caller to dlsym it;
 * returning the ELF's own address would be wrong, since it is a link-time
 * address and the mapping is somewhere else.
 */
static int hisi_sensor_obj_scan(const char *path, char *out, size_t out_len)
{
    Elf32_Ehdr eh;
    Elf32_Shdr *sh = NULL;
    char *strtab = NULL;
    Elf32_Sym *syms = NULL;
    size_t nsyms = 0, strsz = 0;
    int ret = RSS_ERR_NOENT;
    unsigned int i;
    FILE *f;

    if (!(f = fopen(path, "rb")))
        return RSS_ERR_NOENT;

    if (fread(&eh, sizeof(eh), 1, f) != 1)
        goto out;
    if (memcmp(eh.e_ident, ELFMAG, SELFMAG) != 0 || eh.e_ident[EI_CLASS] != ELFCLASS32)
        goto out;
    if (!eh.e_shnum || eh.e_shentsize != sizeof(Elf32_Shdr))
        goto out;

    if (!(sh = calloc(eh.e_shnum, sizeof(*sh))))
        goto out;
    if (fseek(f, (long)eh.e_shoff, SEEK_SET) != 0)
        goto out;
    if (fread(sh, sizeof(*sh), eh.e_shnum, f) != eh.e_shnum)
        goto out;

    for (i = 0; i < eh.e_shnum; i++) {
        if (sh[i].sh_type != SHT_DYNSYM || !sh[i].sh_entsize)
            continue;
        if (sh[i].sh_link >= eh.e_shnum)
            continue;

        nsyms = sh[i].sh_size / sh[i].sh_entsize;
        strsz = sh[sh[i].sh_link].sh_size;
        if (!nsyms || !strsz || nsyms > 65536u || strsz > (1u << 20))
            goto out;

        if (!(syms = calloc(nsyms, sizeof(*syms))) || !(strtab = calloc(strsz, 1)))
            goto out;
        if (fseek(f, (long)sh[i].sh_offset, SEEK_SET) != 0)
            goto out;
        if (fread(syms, sizeof(*syms), nsyms, f) != nsyms)
            goto out;
        if (fseek(f, (long)sh[sh[i].sh_link].sh_offset, SEEK_SET) != 0)
            goto out;
        if (fread(strtab, 1, strsz, f) != strsz)
            goto out;
        break;
    }

    if (!syms || !strtab)
        goto out;

    /* Force a terminator so a truncated table cannot run a name off the
     * end of the buffer. */
    strtab[strsz - 1] = '\0';

    for (i = 0; i < nsyms; i++) {
        const char *name;
        size_t len;

        if (ELF32_ST_TYPE(syms[i].st_info) != STT_OBJECT)
            continue;
        if (syms[i].st_name >= strsz)
            continue;
        name = strtab + syms[i].st_name;
        len = strlen(name);
        if (len < 11 || len >= out_len)
            continue;
        if (strncmp(name, "g_sns_", 6) != 0 || strcmp(name + len - 4, "_obj") != 0)
            continue;

        snprintf(out, out_len, "%s", name);
        ret = RSS_OK;
        break;
    }

out:
    free(strtab);
    free(syms);
    free(sh);
    fclose(f);
    return ret;
}

v5_isp_sns_obj *hisi_sensor_obj_find(hisi_sensor_mode_t *m, void *handle)
{
    char candidate[64];
    void *sym;

    if (m->obj_name[0] && (sym = dlsym(handle, m->obj_name)))
        return (v5_isp_sns_obj *)sym;

    /* The vendor's own convention, and right for eight of the nine
     * libraries on this image. */
    snprintf(candidate, sizeof(candidate), "g_sns_%s_obj", m->name);
    if ((sym = dlsym(handle, candidate))) {
        snprintf(m->obj_name, sizeof(m->obj_name), "%s", candidate);
        return (v5_isp_sns_obj *)sym;
    }

    /* The ninth. libsns_sp2308.so exports g_sns_os02m10_obj. */
    {
        char path[256];

        if (m->dll_file[0] == '/')
            snprintf(path, sizeof(path), "%s", m->dll_file);
        else
            snprintf(path, sizeof(path), "%s/%s", V5_SNS_LIB_DIR, m->dll_file);

        if (hisi_sensor_obj_scan(path, candidate, sizeof(candidate)) == RSS_OK &&
            (sym = dlsym(handle, candidate))) {
            HAL_LOG_WARN("sensor: %s exports \"%s\", not \"g_sns_%s_obj\"; using it", path,
                         candidate, m->name);
            snprintf(m->obj_name, sizeof(m->obj_name), "%s", candidate);
            return (v5_isp_sns_obj *)sym;
        }
    }

    HAL_LOG_ERR("sensor: %s has no g_sns_*_obj symbol", m->dll_file);
    return NULL;
}

/* ================================================================
 * THE LOADER
 * ================================================================ */

/*
 * hisi_die_suffix -- "0X3516C608" -> "hi3516cv608".
 *
 * The chip name as /proc/umap/sys prints it is not a section name anybody
 * would type, so it is normalised here into the spelling the vendor's own
 * trees use for the die. An unrecognised shape yields no suffix, which
 * means the file's defaults apply -- the right failure, since a mode file
 * without die overrides is the common case.
 */
static void hisi_die_suffix(const char *chip_name, char *out, size_t out_len)
{
    out[0] = '\0';

    if (!chip_name || !chip_name[0])
        return;
    /*
     * "0X3516C608" -- ten characters: "0X", four digits, a family letter,
     * three more digits. The die spelling puts a 'v' in front of the last
     * three and keeps the family letter, so "0X3516C608" is "hi3516cv608"
     * and not "hi3516v608".
     */
    if (strlen(chip_name) != 10 || (chip_name[0] != '0' && chip_name[0] != 'O') ||
        (chip_name[1] != 'X' && chip_name[1] != 'x'))
        return;

    snprintf(out, out_len, "hi%.4s%cv%s", chip_name + 2, tolower((unsigned char)chip_name[6]),
             chip_name + 7);
}

int hisi_sensor_mode_load(hisi_sensor_mode_t *m, const char *sensor_name, const char *chip_name)
{
    hisi_ini ini;
    const char *lane;
    int ret;

    memset(m, 0, sizeof(*m));

    if (sensor_name && sensor_name[0]) {
        snprintf(m->name, sizeof(m->name), "%s", sensor_name);
    } else {
        char src[192];

        if (hisi_sensor_from_module(m->name, sizeof(m->name), src, sizeof(src)) == RSS_OK) {
            HAL_LOG_INFO("sensor: \"%s\", from %s (no [sensor] name in the config)", m->name, src);
        } else if (hisi_sensor_from_hostname(m->name, sizeof(m->name), src, sizeof(src)) ==
                   RSS_OK) {
            HAL_LOG_WARN("sensor: \"%s\", guessed from %s -- OpenIPC's hostname convention is not "
                         "an interface. Put the name in raptor's config.",
                         m->name, src);
        } else {
            HAL_LOG_ERR("sensor: no [sensor] name in the config, and none to be found. On V5 the "
                        "kernel's sensors= parameter reads back truncated to \"sns0\", so the "
                        "config is the only reliable source.");
            return RSS_ERR_INVAL;
        }
    }
    sensor_name = m->name;

    ret = hisi_sensor_ini_find(sensor_name, m->ini_path, sizeof(m->ini_path));
    if (ret != RSS_OK) {
        HAL_LOG_ERR("sensor: no mode INI for \"%s\" in /etc/sensors or /usr/share/sensors",
                    sensor_name);
        return ret;
    }

    ret = hisi_ini_read(&ini, m->ini_path);
    if (ret != RSS_OK) {
        HAL_LOG_ERR("sensor: %s: unreadable", m->ini_path);
        return ret;
    }

    hisi_die_suffix(chip_name, ini.die, sizeof(ini.die));
    snprintf(m->die_suffix, sizeof(m->die_suffix), "%s", ini.die);

    /* [sensor] -- which library, and which symbol inside it. */
    snprintf(m->dll_file, sizeof(m->dll_file), "%s", hisi_ini_str(&ini, "sensor", "DllFile", ""));
    snprintf(m->obj_name, sizeof(m->obj_name), "%s",
             hisi_ini_str(&ini, "sensor", "Sensor_type", ""));
    m->wdr_mode =
        (v5_wdr_mode)hisi_ini_enum_val(&ini, "sensor", "Mode", hisi_enum_wdr, V5_WDR_MODE_NONE);

    if (!m->dll_file[0]) {
        /* The vendor's own convention, and right for every library that
         * ships. A fallback rather than the rule because the INI is
         * allowed to disagree -- see sp2308. */
        snprintf(m->dll_file, sizeof(m->dll_file), "libsns_%s.so", sensor_name);
        HAL_LOG_WARN("sensor: %s has no DllFile; assuming %s", m->ini_path, m->dll_file);
    }

    /* [mode] */
    m->input_mode = (v5_input_mode)hisi_ini_enum_val(&ini, "mode", "input_mode",
                                                     hisi_enum_input_mode, V5_INPUT_MODE_MIPI);
    m->raw_bitness = hisi_ini_int(&ini, "mode", "raw_bitness", 12);
    m->mipi_data_type = hisi_mipi_data_type(m->raw_bitness);
    m->pixel_format = hisi_bayer_pixel_format(m->raw_bitness);

    /* [mipi] */
    lane = hisi_ini_str(&ini, "mipi", "lane_id", NULL);
    hisi_parse_lane_map(lane, m->lane_id, V5_MIPI_LANE_NUM);
    m->lane_divide_mode =
        (v5_lane_divide_mode)hisi_ini_int(&ini, "mipi", "lane_divide_mode", V5_LANE_DIVIDE_MODE_0);
    m->mipi_data_rate =
        (v5_mipi_data_rate)hisi_ini_int(&ini, "mipi", "data_rate", V5_MIPI_DATA_RATE_X1);

    /* [i2c] */
    m->i2c_dev = hisi_ini_int(&ini, "i2c", "i2c_dev", 0);

    /* [isp_image] */
    m->frame_rate = hisi_ini_float(&ini, "isp_image", "Isp_FrameRate", 30.0f);
    m->bayer = (v5_bayer_format)hisi_ini_enum_val(&ini, "isp_image", "Isp_Bayer", hisi_enum_bayer,
                                                  V5_BAYER_RGGB);
    m->sns_mode = (unsigned char)hisi_ini_int(&ini, "isp_image", "Isp_SnsMode", 0);

    /* [vi_dev] -- the device attribute, from the vendor's own file. */
    m->intf_mode = (v5_vi_intf_mode)hisi_ini_enum_val(&ini, "vi_dev", "Input_mod",
                                                      hisi_enum_vi_mode, V5_VI_INTF_MODE_MIPI);
    m->work_mode = (v5_vi_work_mode)hisi_ini_enum_val(
        &ini, "vi_dev", "Work_mod", hisi_enum_work_mode, V5_VI_WORK_MODE_MULTIPLEX_1);
    m->component_mask[0] = (unsigned int)hisi_ini_int(&ini, "vi_dev", "Mask_0", 0xFFF00000);
    m->component_mask[1] = (unsigned int)hisi_ini_int(&ini, "vi_dev", "Mask_1", 0);
    m->scan_mode = (v5_vi_scan_mode)hisi_ini_enum_val(&ini, "vi_dev", "Scan_mode",
                                                      hisi_enum_scan_mode, V5_VI_SCAN_PROGRESSIVE);
    m->data_seq = (v5_vi_data_seq)hisi_ini_enum_val(&ini, "vi_dev", "Data_seq", hisi_enum_data_seq,
                                                    V5_VI_DATA_SEQ_YVYU);
    m->data_type =
        (v5_vi_data_type)hisi_ini_int(&ini, "vi_dev", "InputDataType", V5_VI_DATA_TYPE_RAW);
    m->data_reverse = hisi_ini_int(&ini, "vi_dev", "DataRev", 0);
    m->data_rate = (v5_data_rate)hisi_ini_int(&ini, "vi_dev", "DataRate", V5_DATA_RATE_X1);

    m->sync_cfg.vsync = hisi_ini_int(&ini, "vi_dev", "Vsync", 0);
    m->sync_cfg.vsync_neg = hisi_ini_int(&ini, "vi_dev", "VsyncNeg", 0);
    m->sync_cfg.hsync = hisi_ini_int(&ini, "vi_dev", "Hsync", 0);
    m->sync_cfg.hsync_neg = hisi_ini_int(&ini, "vi_dev", "HsyncNeg", 0);
    m->sync_cfg.vsync_valid = hisi_ini_int(&ini, "vi_dev", "VsyncValid", 1);
    m->sync_cfg.vsync_valid_neg = hisi_ini_int(&ini, "vi_dev", "VsyncValidNeg", 0);
    m->sync_cfg.timing_blank.hsync_hfb =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_HsyncHfb", 0);
    m->sync_cfg.timing_blank.hsync_act =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_HsyncAct", 0);
    m->sync_cfg.timing_blank.hsync_hbb =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_HsyncHbb", 0);
    m->sync_cfg.timing_blank.vsync_vfb =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_VsyncVfb", 0);
    m->sync_cfg.timing_blank.vsync_vact =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_VsyncVact", 0);
    m->sync_cfg.timing_blank.vsync_vbb =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_VsyncVbb", 0);
    m->sync_cfg.timing_blank.vsync_vbfb =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_VsyncVbfb", 0);
    m->sync_cfg.timing_blank.vsync_vbact =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_VsyncVbact", 0);
    m->sync_cfg.timing_blank.vsync_vbbb =
        (unsigned int)hisi_ini_int(&ini, "vi_dev", "Timingblank_VsyncVbbb", 0);

    /*
     * The sensor's output size, and the one field the die overrides in
     * practice: a CV608 runs every sensor here at 2304x1296 where a CV610
     * runs 4M or 5M, because its ISP is clocked at 148.5 MHz rather than
     * 198 (clk_cfg.c). That is why hisi_ini_str consults [vi_dev.<die>]
     * first.
     *
     * DevRect_x/DevRect_y are read and reported and then not applied, for
     * the reason gen4's file gives at length: used as a crop they ask for
     * DevRect_x + DevRect_w columns of a stream only DevRect_w wide, and
     * the pipe never completes a line. The vendor's own V5 configs set
     * wnd_rect.x/y to 0 on every sensor, which is the same conclusion.
     */
    m->dev_rect.x = hisi_ini_int(&ini, "vi_dev", "DevRect_x", 0);
    m->dev_rect.y = hisi_ini_int(&ini, "vi_dev", "DevRect_y", 0);
    m->dev_rect.width = (unsigned int)hisi_ini_int(&ini, "vi_dev", "DevRect_w", 0);
    m->dev_rect.height = (unsigned int)hisi_ini_int(&ini, "vi_dev", "DevRect_h", 0);

    if (!m->dev_rect.width || !m->dev_rect.height) {
        HAL_LOG_ERR("sensor: %s gives no DevRect_w/DevRect_h%s -- there is no other source for "
                    "the sensor's output size",
                    m->ini_path, ini.die[0] ? " for this die" : "");
        return RSS_ERR_INVAL;
    }

    if (m->wdr_mode != V5_WDR_MODE_NONE) {
        HAL_LOG_WARN("sensor: %s asks for WDR mode %d; this backend drives linear only and will "
                     "configure the pipeline as if WDR were off",
                     m->ini_path, (int)m->wdr_mode);
        m->wdr_mode = V5_WDR_MODE_NONE;
    }

    HAL_LOG_INFO("sensor: %s%s%s -> %s (%s), %ux%u, RAW%d, %.2f fps, bayer %d, i2c %d, "
                 "lanes %d|%d|%d|%d",
                 m->ini_path, ini.die[0] ? " +" : "", ini.die[0] ? ini.die : "", m->dll_file,
                 m->obj_name[0] ? m->obj_name : "object by scan", m->dev_rect.width,
                 m->dev_rect.height, m->raw_bitness, (double)m->frame_rate, (int)m->bayer,
                 m->i2c_dev, m->lane_id[0], m->lane_id[1], m->lane_id[2], m->lane_id[3]);
    return RSS_OK;
}
