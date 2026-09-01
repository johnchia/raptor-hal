/*
 * hisi_v4/hisi_sensor.c -- sensor mode discovery for the HiSilicon gen4 HAL
 *
 * Two jobs, both in service of the same rule: no sensor table in the code.
 *
 *   1. Read a sensor mode INI. Every gen4 board carries a set of them in
 *      /etc/sensors,
 *      one per sensor and mode, and between them they hold everything
 *      bring-up needs that raptor's own config does not: the driver's file
 *      name and object symbol, the MIPI lane map, the RAW bit depth, the
 *      Bayer order, the sensor's frame rate, and the whole VI device
 *      attribute. This is the same data majestic reads, so a board that
 *      works under the stock firmware has the values already.
 *
 *   2. Find a driver's object symbol by reading its ELF, for when the INI
 *      is absent or disagrees with the library.
 *
 * WHY THE INI RATHER THAN A TABLE. 34 libsns_*.so ship on a stock image and
 * each has its own lane map and bit depth. A table in this file would cover
 * whichever subset someone tested and silently exclude the rest, and it
 * would go stale every time OpenIPC adds a sensor. Reading the file that is
 * already on the board makes every shipped sensor reachable and makes the
 * failure mode legible: a sensor with no INI is reported by name.
 *
 * The parser is deliberately small and local rather than raptor-common's:
 * raptor-hal links nothing outside itself, and that property is what makes
 * the backend host-testable.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <ctype.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 * A SMALL INI READER
 *
 * Enough for the vendor's dialect and no more: [sections], key = value,
 * ';' comments to end of line, values that may be decimal, hex, TRUE/FALSE,
 * a symbolic enumerator name, or a '|'-separated list.
 * ================================================================ */

#define HISI_INI_MAX_ENTRIES 128
#define HISI_INI_MAX_LINE 256

typedef struct {
    char section[32];
    char key[40];
    char value[80];
} hisi_ini_entry;

typedef struct {
    hisi_ini_entry entry[HISI_INI_MAX_ENTRIES];
    int count;
} hisi_ini;

/*
 * Case-insensitive compare, written out rather than reached for.
 *
 * The library version lives in <strings.h> behind a POSIX feature macro,
 * and raptor-hal compiles -std=c11 -- widening the feature set of a
 * translation unit for one three-line function invites the next one in for
 * free. The shipped INIs need it: they spell keys inconsistently
 * ("Work_mod" beside "input_mode") and enumerator names in mixed case.
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
    char section[32] = "";
    FILE *f;

    memset(ini, 0, sizeof(*ini));

    if (!(f = fopen(path, "r")))
        return RSS_ERR_NOENT;

    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        char *eq;
        char *comment;

        /* Comments run to end of line and may follow a value. Both ';' and
         * '#' appear in the shipped files. */
        if ((comment = strpbrk(p, ";#")))
            *comment = '\0';

        p = hisi_ini_trim(p);
        if (!*p)
            continue;

        if (*p == '[') {
            char *close = strchr(p, ']');
            if (close) {
                *close = '\0';
                snprintf(section, sizeof(section), "%s", hisi_ini_trim(p + 1));
            }
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

/* Case-insensitive key lookup: the shipped files are inconsistent about it
 * ("Work_mod" beside "input_mode"), and a lookup that misses returns the
 * caller's default rather than failing, which is the behaviour every use
 * below wants. */
static const char *hisi_ini_str(const hisi_ini *ini, const char *section, const char *key,
                                const char *def)
{
    int i;

    for (i = 0; i < ini->count; i++) {
        if (!hisi_ci_eq(ini->entry[i].section, section))
            continue;
        if (!hisi_ci_eq(ini->entry[i].key, key))
            continue;
        return ini->entry[i].value;
    }
    return def;
}

/*
 * Integer values, in the four spellings the vendor files use: decimal, hex
 * with an 0x prefix, TRUE/FALSE, and a symbolic enumerator name resolved
 * through a caller-supplied table.
 */
typedef struct {
    const char *name;
    int value;
} hisi_ini_enum;

static int hisi_ini_int(const hisi_ini *ini, const char *section, const char *key, int def)
{
    const char *v = hisi_ini_str(ini, section, key, NULL);
    char *end;
    long n;

    if (!v || !*v)
        return def;
    if (hisi_ci_eq(v, "true"))
        return 1;
    if (hisi_ci_eq(v, "false"))
        return 0;

    n = strtol(v, &end, 0);
    if (end == v)
        return def;
    return (int)n;
}

static int hisi_ini_enum_val(const hisi_ini *ini, const char *section, const char *key,
                             const hisi_ini_enum *table, int def)
{
    const char *v = hisi_ini_str(ini, section, key, NULL);
    int i;

    if (!v || !*v)
        return def;

    for (i = 0; table[i].name; i++) {
        if (hisi_ci_eq(v, table[i].name))
            return table[i].value;
    }

    /* A bare number in a field the vendor usually spells symbolically is
     * legal and appears in some files -- InputDataType=1 next to
     * Input_mod=VI_MODE_MIPI in the same section. */
    if (v[0] == '-' || isdigit((unsigned char)v[0]))
        return (int)strtol(v, NULL, 0);

    HAL_LOG_WARN("sensor ini: [%s] %s = \"%s\" is not a value this backend knows; using %d",
                 section, key, v, def);
    return def;
}

/* ================================================================
 * ELF SYMBOL SCAN
 * ================================================================ */

/*
 * v4_snr_scan_obj_name -- find a stSns*Obj export in a sensor driver.
 *
 * The fallback when the INI's Sensor_type key is absent or wrong. Reads the
 * library's own .dynsym rather than guessing at the naming convention,
 * because the convention is only a convention: stSnsImx335Obj,
 * stSnsGc4653Obj and stSnsSc3335Obj all follow it and nothing enforces it.
 *
 * A hand-rolled ELF32 walk rather than libelf: raptor-hal links nothing, and
 * what is needed here is three structure reads. Only defined symbols count
 * -- an undefined stSns*Obj would be a driver referencing another driver's
 * object, which is not what the caller is looking for.
 *
 * `path` is a library name as passed to dlopen, so it may be a bare
 * "libsns_imx335.so". Resolving it means searching the same directories the
 * loader would; the two that matter on a gen4 board are named below.
 */
int v4_snr_scan_obj_name(const char *path, char *out, size_t out_len)
{
    static const char *dirs[] = V4_SNR_DIRS;
    unsigned char ehdr[52];
    unsigned char *shdrs = NULL;
    char *strtab = NULL;
    unsigned char *symtab = NULL;
    unsigned int shoff, shentsize, shnum;
    unsigned int symoff = 0, symsize = 0, symentsize = 0, strlink = 0;
    unsigned int stroff = 0, strsize = 0;
    unsigned int i;
    int ret = RSS_ERR_NOENT;
    FILE *f = NULL;
    size_t d;

    if (!path || !out || out_len == 0)
        return RSS_ERR_INVAL;
    out[0] = '\0';

    for (d = 0; d < sizeof(dirs) / sizeof(dirs[0]); d++) {
        char full[192];

        if (path[0] == '/' && d > 0)
            break;
        snprintf(full, sizeof(full), "%s%s", dirs[d], path);
        if ((f = fopen(full, "rb")))
            break;
    }
    if (!f)
        return RSS_ERR_NOENT;

    if (fread(ehdr, 1, sizeof(ehdr), f) != sizeof(ehdr))
        goto out;
    if (memcmp(ehdr, "\177ELF", 4) != 0 || ehdr[4] != 1 /* ELFCLASS32 */ ||
        ehdr[5] != 1 /* ELFDATA2LSB */)
        goto out;

#define RD32(p) ((unsigned int)(p)[0] | ((unsigned int)(p)[1] << 8) | ((unsigned int)(p)[2] << 16) | \
                 ((unsigned int)(p)[3] << 24))
#define RD16(p) ((unsigned int)(p)[0] | ((unsigned int)(p)[1] << 8))

    shoff = RD32(ehdr + 32);
    shentsize = RD16(ehdr + 46);
    shnum = RD16(ehdr + 48);

    /* A driver with no section headers is stripped past usefulness, and a
     * shentsize other than 40 is not an ELF32 this code understands. Bail
     * rather than compute an offset from a number that means something
     * else. */
    if (!shoff || shentsize != 40 || shnum == 0 || shnum > 512)
        goto out;

    if (!(shdrs = (unsigned char *)malloc((size_t)shnum * shentsize)))
        goto out;
    if (fseek(f, (long)shoff, SEEK_SET) != 0 ||
        fread(shdrs, 1, (size_t)shnum * shentsize, f) != (size_t)shnum * shentsize)
        goto out;

    for (i = 0; i < shnum; i++) {
        const unsigned char *sh = shdrs + (size_t)i * shentsize;

        if (RD32(sh + 4) == 11 /* SHT_DYNSYM */) {
            symoff = RD32(sh + 16);
            symsize = RD32(sh + 20);
            strlink = RD32(sh + 24);
            symentsize = RD32(sh + 36);
            break;
        }
    }
    if (!symsize || symentsize != 16 || strlink >= shnum)
        goto out;

    {
        const unsigned char *sh = shdrs + (size_t)strlink * shentsize;
        stroff = RD32(sh + 16);
        strsize = RD32(sh + 20);
    }
    if (!strsize || strsize > (16u << 20) || symsize > (16u << 20))
        goto out;

    if (!(strtab = (char *)malloc(strsize)) || !(symtab = (unsigned char *)malloc(symsize)))
        goto out;
    if (fseek(f, (long)stroff, SEEK_SET) != 0 || fread(strtab, 1, strsize, f) != strsize)
        goto out;
    if (fseek(f, (long)symoff, SEEK_SET) != 0 || fread(symtab, 1, symsize, f) != symsize)
        goto out;
    strtab[strsize - 1] = '\0';

    for (i = 0; i < symsize / symentsize; i++) {
        const unsigned char *sym = symtab + (size_t)i * symentsize;
        unsigned int nameoff = RD32(sym);
        unsigned int shndx = RD16(sym + 14);
        const char *name;
        size_t len;

        if (shndx == 0 /* SHN_UNDEF */ || nameoff >= strsize)
            continue;

        name = strtab + nameoff;
        len = strlen(name);
        if (len < 8 || len >= out_len)
            continue;
        if (strncmp(name, "stSns", 5) != 0 || strcmp(name + len - 3, "Obj") != 0)
            continue;

        snprintf(out, out_len, "%s", name);
        ret = RSS_OK;
        break;
    }

#undef RD32
#undef RD16

out:
    free(symtab);
    free(strtab);
    free(shdrs);
    if (f)
        fclose(f);
    return ret;
}

/* ================================================================
 * SENSOR MODE
 * ================================================================ */

static const hisi_ini_enum hisi_enum_input_mode[] = {
    {"INPUT_MODE_MIPI", V4_INPUT_MODE_MIPI},
    {"INPUT_MODE_SUBLVDS", V4_INPUT_MODE_SUBLVDS},
    {"INPUT_MODE_LVDS", V4_INPUT_MODE_LVDS},
    {"INPUT_MODE_HISPI", V4_INPUT_MODE_HISPI},
    {"INPUT_MODE_CMOS", V4_INPUT_MODE_CMOS},
    {NULL, 0},
};

static const hisi_ini_enum hisi_enum_bayer[] = {
    {"BAYER_RGGB", V4_BAYER_RGGB},
    {"BAYER_GRBG", V4_BAYER_GRBG},
    {"BAYER_GBRG", V4_BAYER_GBRG},
    {"BAYER_BGGR", V4_BAYER_BGGR},
    {NULL, 0},
};

static const hisi_ini_enum hisi_enum_wdr[] = {
    {"WDR_MODE_NONE", V4_WDR_MODE_NONE},
    {"WDR_MODE_BUILT_IN", V4_WDR_MODE_BUILT_IN},
    {"WDR_MODE_QUDRA", V4_WDR_MODE_QUDRA},
    {"WDR_MODE_2To1_LINE", V4_WDR_MODE_2To1_LINE},
    {"WDR_MODE_2To1_FRAME", V4_WDR_MODE_2To1_FRAME},
    {NULL, 0},
};

static const hisi_ini_enum hisi_enum_vi_mode[] = {
    {"VI_MODE_MIPI", V4_VI_MODE_MIPI},
    {"VI_MODE_LVDS", V4_VI_MODE_LVDS},
    {NULL, 0},
};

static const hisi_ini_enum hisi_enum_work_mode[] = {
    {"VI_WORK_MODE_1Multiplex", V4_VI_WORK_1MULTIPLEX},
    {NULL, 0},
};

static const hisi_ini_enum hisi_enum_scan_mode[] = {
    {"VI_SCAN_INTERLACED", V4_VI_SCAN_INTERLACED},
    {"VI_SCAN_PROGRESSIVE", V4_VI_SCAN_PROGRESSIVE},
    {NULL, 0},
};

/*
 * The MIPI RAW bit depth appears twice in the vendor's own data: as
 * raw_bitness in the INI and as the pixel format the VI pipe carries. Both
 * derive from one number, so derive them here rather than reading two keys
 * that can disagree.
 */
static v4_mipi_data_type hisi_mipi_data_type(int raw_bitness)
{
    switch (raw_bitness) {
    case 8:
        return V4_MIPI_DATA_TYPE_RAW_8BIT;
    case 10:
        return V4_MIPI_DATA_TYPE_RAW_10BIT;
    case 14:
        return V4_MIPI_DATA_TYPE_RAW_14BIT;
    case 16:
        return V4_MIPI_DATA_TYPE_RAW_16BIT;
    case 12:
    default:
        return V4_MIPI_DATA_TYPE_RAW_12BIT;
    }
}

static v4_pixel_format hisi_bayer_pixel_format(int raw_bitness)
{
    switch (raw_bitness) {
    case 8:
        return V4_PIXEL_FORMAT_RGB_BAYER_8BPP;
    case 10:
        return V4_PIXEL_FORMAT_RGB_BAYER_10BPP;
    case 14:
        return V4_PIXEL_FORMAT_RGB_BAYER_14BPP;
    case 16:
        return V4_PIXEL_FORMAT_RGB_BAYER_16BPP;
    case 12:
    default:
        return V4_PIXEL_FORMAT_RGB_BAYER_12BPP;
    }
}

static v4_data_bitwidth hisi_bit_width(int raw_bitness)
{
    switch (raw_bitness) {
    case 8:
        return V4_DATA_BITWIDTH_8;
    case 10:
        return V4_DATA_BITWIDTH_10;
    case 14:
        return V4_DATA_BITWIDTH_14;
    case 16:
        return V4_DATA_BITWIDTH_16;
    case 12:
    default:
        return V4_DATA_BITWIDTH_12;
    }
}

/*
 * "0|1|2|3|-1|-1|-1|-1|" -- the lane map, four entries of which gen4 uses.
 * -1 disables a lane, which is how a two-lane sensor is written on four-lane
 * silicon. A list shorter than four leaves the remainder disabled, which is
 * the safe direction: an accidentally-enabled lane reads noise onto a pipe.
 */
static void hisi_parse_lane_map(const char *s, short *lane, int n)
{
    int i;

    for (i = 0; i < n; i++)
        lane[i] = -1;

    if (!s)
        return;

    for (i = 0; i < n && *s; i++) {
        char *end;
        long v = strtol(s, &end, 10);

        if (end == s)
            break;
        lane[i] = (short)v;
        s = end;
        while (*s == '|' || *s == ' ' || *s == '\t')
            s++;
    }
}

/*
 * hisi_sensor_ini_find -- locate the mode INI for a configured sensor name.
 *
 * The file names are not derivable: the IMX335's modes ship as both
 * "imx335_i2c_4M.ini" and "5M_imx335.ini", and the GC4653's as
 * "gc4653_i2c_4M.ini". So this scans the directory for any .ini whose name
 * contains the sensor name, rather than composing a path.
 *
 * When several match, the first in sorted order wins and every candidate is
 * logged. Deterministic beats clever here: the alternative is a heuristic
 * about which mode is "best", which is a decision the person who put two
 * modes on the board already made and can express by removing one.
 *
 * /etc/sensors/iq/ holds the IQ tuning INIs, a different thing with names
 * that would also match; only the top level of each directory is read.
 */
static int hisi_sensor_ini_find(const char *sensor_name, char *out, size_t out_len)
{
    static const char *dirs[] = {"/etc/sensors", "/usr/share/sensors"};
    char lower[64];
    char best[192] = "";
    int matches = 0;
    size_t d, i;

    for (i = 0; i + 1 < sizeof(lower) && sensor_name[i]; i++)
        lower[i] = (char)tolower((unsigned char)sensor_name[i]);
    lower[i] = '\0';
    if (!lower[0])
        return RSS_ERR_INVAL;

    for (d = 0; d < sizeof(dirs) / sizeof(dirs[0]); d++) {
        struct dirent *de;
        DIR *dir = opendir(dirs[d]);

        if (!dir)
            continue;

        while ((de = readdir(dir))) {
            /* Two bounded copies of the entry name, one verbatim for the
             * path and one lowercased for the match. Copying first rather
             * than formatting straight out of de->d_name is what keeps the
             * path's own bound provable: d_name is declared 256 wide and
             * the compiler has no other way to know this loop has already
             * rejected anything longer than `entry`. */
            char entry[128];
            char name[128];
            char path[192];
            size_t len = strlen(de->d_name);
            size_t j;

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

            snprintf(path, sizeof(path), "%s/%s", dirs[d], entry);
            matches++;
            if (!best[0] || strcmp(path, best) < 0)
                snprintf(best, sizeof(best), "%s", path);
            else
                HAL_LOG_INFO("sensor ini: %s also matches \"%s\"", path, sensor_name);
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

/*
 * hisi_sensor_mode_load -- fill in everything bring-up needs about a sensor.
 *
 * Returns RSS_ERR_NOENT when no INI matches, and says so by name: that is
 * the actionable failure, because the fix is to put the vendor's own file
 * on the board rather than to change raptor.
 *
 * Values the INI does not carry get defaults that suit a modern MIPI Bayer
 * sensor -- progressive scan, one-lane-per-pin data rate, RGB input type --
 * rather than zeros, because a zero here is a legal enumerator meaning
 * something else, and a mode file that omits a key is asking for the usual
 * answer rather than for no answer.
 */
int hisi_sensor_mode_load(hisi_sensor_mode_t *m, const char *sensor_name)
{
    hisi_ini ini;
    const char *lane;
    int ret;

    memset(m, 0, sizeof(*m));

    if (!sensor_name || !sensor_name[0]) {
        HAL_LOG_ERR("sensor: [sensor] name is required on gen4 -- there is no /proc/jz/sensor "
                    "equivalent and identity is whichever libsns_*.so is loaded");
        return RSS_ERR_INVAL;
    }

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

    /* [sensor] -- which library, and which symbol inside it. */
    snprintf(m->dll_file, sizeof(m->dll_file), "%s",
             hisi_ini_str(&ini, "sensor", "DllFile", ""));
    snprintf(m->obj_name, sizeof(m->obj_name), "%s",
             hisi_ini_str(&ini, "sensor", "Sensor_type", ""));
    m->wdr_mode = (v4_wdr_mode)hisi_ini_enum_val(&ini, "sensor", "Mode", hisi_enum_wdr,
                                                 V4_WDR_MODE_NONE);

    if (!m->dll_file[0]) {
        /* Composing "libsns_<name>.so" is the vendor's own convention and
         * holds for all 34 shipped drivers; it is a fallback rather than
         * the rule because the INI is allowed to disagree. */
        snprintf(m->dll_file, sizeof(m->dll_file), "libsns_%s.so", sensor_name);
        HAL_LOG_WARN("sensor: %s has no DllFile; assuming %s", m->ini_path, m->dll_file);
    }

    /* [mode] */
    m->input_mode = (v4_input_mode)hisi_ini_enum_val(&ini, "mode", "input_mode",
                                                     hisi_enum_input_mode, V4_INPUT_MODE_MIPI);
    m->raw_bitness = hisi_ini_int(&ini, "mode", "raw_bitness", 12);
    m->mipi_data_type = hisi_mipi_data_type(m->raw_bitness);
    m->pixel_format = hisi_bayer_pixel_format(m->raw_bitness);
    m->bit_width = hisi_bit_width(m->raw_bitness);

    /* [mipi] */
    lane = hisi_ini_str(&ini, "mipi", "lane_id", NULL);
    hisi_parse_lane_map(lane, m->lane_id, V4_MIPI_LANE_NUM);

    /* [isp_image] */
    m->frame_rate = hisi_ini_int(&ini, "isp_image", "Isp_FrameRate", 25);
    m->bayer = (v4_bayer_format)hisi_ini_enum_val(&ini, "isp_image", "Isp_Bayer", hisi_enum_bayer,
                                                  V4_BAYER_RGGB);

    /* [vi_dev] -- the device attribute, verbatim from the vendor's own file.
     * The sync-timing block is dead on a MIPI sensor and filled in anyway;
     * carrying it costs nothing and skipping it would be a guess about
     * which fields the driver reads. */
    m->intf_mode = (v4_vi_intf_mode)hisi_ini_enum_val(&ini, "vi_dev", "Input_mod",
                                                      hisi_enum_vi_mode, V4_VI_MODE_MIPI);
    m->work_mode = (v4_vi_work_mode)hisi_ini_enum_val(&ini, "vi_dev", "Work_mod",
                                                      hisi_enum_work_mode, V4_VI_WORK_1MULTIPLEX);
    m->component_mask[0] = (unsigned int)hisi_ini_int(&ini, "vi_dev", "Mask_0", 0xFFF00000);
    m->component_mask[1] = (unsigned int)hisi_ini_int(&ini, "vi_dev", "Mask_1", 0);
    m->scan_mode = (v4_vi_scan_mode)hisi_ini_enum_val(&ini, "vi_dev", "Scan_mode",
                                                      hisi_enum_scan_mode, V4_VI_SCAN_PROGRESSIVE);
    m->data_seq = (unsigned int)hisi_ini_int(&ini, "vi_dev", "Data_seq", 0);
    m->input_data_type = (v4_vi_data_type)hisi_ini_int(&ini, "vi_dev", "InputDataType",
                                                       V4_VI_DATA_TYPE_RGB);
    m->data_reverse = hisi_ini_int(&ini, "vi_dev", "DataRev", 0);

    m->sync_cfg.vsync = (unsigned int)hisi_ini_int(&ini, "vi_dev", "Vsync", 0);
    m->sync_cfg.vsync_neg = (unsigned int)hisi_ini_int(&ini, "vi_dev", "VsyncNeg", 0);
    m->sync_cfg.hsync = (unsigned int)hisi_ini_int(&ini, "vi_dev", "Hsync", 0);
    m->sync_cfg.hsync_neg = (unsigned int)hisi_ini_int(&ini, "vi_dev", "HsyncNeg", 0);
    m->sync_cfg.vsync_valid = (unsigned int)hisi_ini_int(&ini, "vi_dev", "VsyncValid", 0);
    m->sync_cfg.vsync_valid_neg = (unsigned int)hisi_ini_int(&ini, "vi_dev", "VsyncValidNeg", 0);
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
     * DevRect is the crop the ISP is asked to process, and its origin is not
     * always zero -- the IMX335's 5 MP mode starts at (200, 20). Setting the
     * ISP's window to the full sensor output on such a mode gives a green
     * band down two edges, so the offset is carried rather than assumed away.
     */
    /*
     * DevRect_x and DevRect_y are read and reported, and then nothing
     * applies them.
     *
     * They read like the origin of the sensor's active window, and the
     * IMX335's 5 MP entry says (200, 20). Used as a crop -- in the MIPI
     * device's img_rect or the ISP's stWndRect -- they ask for
     * DevRect_x + DevRect_w columns of a stream that is only DevRect_w
     * wide, and the pipe then never completes a line: /proc/umap/vi shows
     * IntCnt frozen a few counts above zero with FrameRate 0, and
     * /proc/umap/mipi_rx reports a detected width short of the configured
     * one by roughly the offset.
     *
     * divinus parses both fields into its ISP capture rect and then
     * assigns 0 to each before HI_MPI_ISP_SetPubAttr
     * (src/hal/hisi/v4_hal.c:322), which is the same conclusion reached
     * from the other direction. They are kept here because they are what
     * the file says and the log prints the file.
     */
    m->dev_rect.x = hisi_ini_int(&ini, "vi_dev", "DevRect_x", 0);
    m->dev_rect.y = hisi_ini_int(&ini, "vi_dev", "DevRect_y", 0);
    m->dev_rect.width = (unsigned int)hisi_ini_int(&ini, "vi_dev", "DevRect_w", 0);
    m->dev_rect.height = (unsigned int)hisi_ini_int(&ini, "vi_dev", "DevRect_h", 0);
    m->full_lines_std = (unsigned int)hisi_ini_int(&ini, "vi_dev", "FullLinesStd", 0);

    if (!m->dev_rect.width || !m->dev_rect.height) {
        HAL_LOG_ERR("sensor: %s gives no DevRect_w/DevRect_h -- there is no other source for the "
                    "sensor's output size",
                    m->ini_path);
        return RSS_ERR_INVAL;
    }

    if (m->wdr_mode != V4_WDR_MODE_NONE) {
        HAL_LOG_WARN("sensor: %s asks for WDR mode %d; this backend drives linear only and will "
                     "configure the pipeline as if WDR were off",
                     m->ini_path, (int)m->wdr_mode);
        m->wdr_mode = V4_WDR_MODE_NONE;
    }

    HAL_LOG_INFO("sensor: %s -> %s (%s), %ux%u+%d+%d, RAW%d, %d fps, bayer %d, lanes %d|%d|%d|%d",
                 m->ini_path, m->dll_file, m->obj_name[0] ? m->obj_name : "object by scan",
                 m->dev_rect.width, m->dev_rect.height, m->dev_rect.x, m->dev_rect.y,
                 m->raw_bitness, m->frame_rate, (int)m->bayer, m->lane_id[0], m->lane_id[1],
                 m->lane_id[2], m->lane_id[3]);
    return RSS_OK;
}
