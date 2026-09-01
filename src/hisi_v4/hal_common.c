/*
 * hisi_v4/hal_common.c -- Raptor HAL common layer, HiSilicon HiMPP V4.0
 *
 * Counterpart to src/hal_common.c (Ingenic IMP) and src/star/hal_common.c
 * (SigmaStar MI). Provides the factory functions, the ops vtable, the
 * logging hook, the system lifecycle, and the symbols the vendor libraries
 * expect the *executable* to define.
 *
 * A separate translation unit rather than #ifdefs elsewhere, for the reason
 * star/hal_common.c gives: the HAL_OLD_SDK / HAL_NEW_SDK conditionals in
 * src/hal_common.c distinguish generations of one vendor's SDK, where the
 * call sequences match and only layouts differ. HiMPP is a third SDK with a
 * third pipeline model -- VI -> VPSS -> VENC, with the ISP keyed on the VI
 * pipe rather than existing as a stage -- so sharing a file would produce
 * two disjoint implementations behind mutually exclusive guards.
 *
 * Everything here guards on HAL_HISI_GEN4, never on PLATFORM_HI3516EV200.
 * The generation is the unit of compatibility: an EV300 board reports the
 * MPP version string "Hi3516EV200_MPP_V1.0.1.2", i.e. one MPP build serves
 * EV200, EV300, DV200 and 3518EV300. Adding one of those costs a word in
 * HISI_GEN4_PLATFORMS and a caps block, and no code. HAL_HISI_GEN5 exists
 * to be got wrong, which is why this rule has teeth.
 *
 * Current state: Phase 1 skeleton. The vtable publishes only the ops that
 * are implemented; RSS_HAL_CALL() NULL-guards every entry and returns
 * RSS_ERR_NOTSUP for the rest, so unimplemented subsystems need no stubs.
 * The video pipeline lands in Phase 2.
 *
 * Copyright (C) 2026 Thingino Project
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "hisi_state.h"

#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/select.h>
#include <syslog.h>
#include <unistd.h>

/* ================================================================
 * LOGGING
 *
 * Mirrors src/hal_common.c and star/hal_common.c: log through a function
 * pointer that defaults to stderr, which daemons redirect to syslog at init.
 * ================================================================ */

static const char *hal_level_str[] = {"FTL", "ERR", "WRN", "INF", "DBG"};

static void hal_log_stderr(int level, const char *file, int line, const char *fmt, ...)
{
    if (level < 0)
        level = 0;
    if (level > 4)
        level = 4;
    const char *basename = strrchr(file, '/');
    if (basename)
        file = basename + 1;
    fprintf(stderr, "[HAL %s] %s:%d: ", hal_level_str[level], file, line);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

rss_hal_log_func_t rss_hal_log_fn = hal_log_stderr;

void rss_hal_set_log_func(rss_hal_log_func_t func)
{
    rss_hal_log_fn = func ? func : hal_log_stderr;
}

/* ── Per-SoC capability data (src/hal_caps.c, compiled per platform) ── */

extern const rss_hal_caps_t g_hal_caps;

/*
 * The live backend state.
 *
 * The daemons call rss_hal_get_imp_version() and friends with no context
 * argument, and the GK_API_* forwarders below are called by a vendor
 * library that has no context to pass either. One HAL context per process
 * is already assumed throughout raptor; this is the pointer that makes that
 * assumption usable. Set at the end of hal_init, cleared at the start of
 * hal_deinit.
 */
static hisi_state_t *g_hisi;

/* ── GPIO / IR-cut (src/hal_gpio.c — plain sysfs, no SDK dependency) ── */

#ifdef HAL_MODULE_VIDEO
int hal_gpio_set(void *ctx, int pin, int value);
int hal_gpio_get(void *ctx, int pin, int *value);
int hal_ircut_set(void *ctx, int state);
#endif

/* ================================================================
 * TRAMPOLINES -- symbols the vendor libraries expect from the executable
 *
 * This block is why raptor-hal's HiSilicon builds pass
 * -Wl,--export-dynamic unconditionally, and why it lives in *this* file
 * rather than in a quirks.c of its own.
 *
 * raptor-hal ships as libraptor_hal_video.a. A static-archive member is
 * extracted only if something already linked references a symbol in it, and
 * nothing in raptor references __ctype_b or GK_API_ISP_SensorRegCallBack.
 * A standalone quirks.c would therefore compile, archive, and never link --
 * and --export-dynamic cannot export what was never linked. -Wl,--gc-sections
 * and -flto compound it. hal_common.c is the translation unit that defines
 * rss_hal_create, so it is always extracted; putting the definitions here is
 * what makes them exist at all. __attribute__((used)) then keeps LTO and
 * --gc-sections from discarding them, since nothing in the program refers to
 * them either.
 *
 * The symbol list is measured, not inherited. Taking the closure over all 48
 * vendor libraries on a gen4 board -- every UND symbol, minus everything any
 * of them defines, minus libc.so, libgcc_s.so.1 and libatomic.so.1 -- leaves
 * ten strong undefined symbols and five weak ones that need nothing. The ten
 * fall into two unrelated groups, neither of which is the isp_malloc /
 * isp_alg_register_* set divinus defines: no gen4 library on this board
 * references any of those.
 * ================================================================ */

/* ── Group 1: the uClibc ABI, needed by libsecurec.so ──
 *
 * Every vendor .so declares NEEDED libc.so.0 while the rootfs is musl.
 * OpenIPC bridges that with an ld-uClibc.so.0 -> libc.so symlink plus these
 * three definitions in the executable; majestic exports exactly them (its
 * whole dynamic symbol table is 20 entries).
 *
 * __ctype_b is the one that constrains the design. It is a data symbol, so
 * its relocation is R_ARM_GLOB_DAT and binds *eagerly* when libsecurec.so
 * is mapped -- and libsecurec.so is a DT_NEEDED of both libmpi.so and
 * libisp.so. No dlopen ordering can fix it after the fact: the symbol has
 * to be in global scope before the first vendor dlopen, which means defined
 * in the executable and exported. hisi_check_trampolines() below verifies
 * that it actually is, before anything can fail obscurely for want of it.
 */

/*
 * uClibc's __ctype_b is a pointer to the table, biased by 128 so that
 * indices -128..255 are all valid, with the same bit assignments glibc
 * uses. musl publishes a glibc-compatible table through __ctype_b_loc(),
 * so the correct value is one dereference away.
 *
 * majestic leaves this NULL -- the object is 4 bytes of .bss that nothing
 * in its disassembly ever writes. That satisfies the relocation and
 * segfaults the moment any securec routine actually classifies a character.
 * Pointing it at a real table costs one line at init and is the difference
 * between "works" and "works until something calls vsnprintf_s".
 *
 * Declared here rather than by defining _GNU_SOURCE for the whole
 * translation unit: musl guards __ctype_b_loc behind the POSIX/GNU feature
 * macros and raptor-hal compiles -std=c11, and widening the feature set of
 * a file for one symbol invites the next one in for free.
 */
extern const unsigned short **__ctype_b_loc(void);

__attribute__((used)) const unsigned short *__ctype_b;

/* uClibc's getc_unlocked spelling. A tail call; majestic's is literally one
 * branch to fgetc. musl's fgetc takes the lock, which costs nothing here --
 * securec calls this on a FILE * no other thread has. */
__attribute__((used)) int __fgetc_unlocked(FILE *f)
{
    return fgetc(f);
}

/*
 * uClibc's MB_CUR_MAX accessor.
 *
 * majestic's returns a constant 0, which is wrong for any locale -- a
 * multibyte character is at least one byte -- and works only because
 * nothing it runs consults it. musl answers the same question properly
 * through MB_CUR_MAX, so forward to that rather than reproducing the bug.
 */
__attribute__((used)) size_t _stdlib_mb_cur_max(void)
{
    return MB_CUR_MAX;
}

/* ── Group 2: the Goke sensor tier ──
 *
 * Eight of the 34 libsns_*.so on a gen4 board are T5-tier drivers built
 * against Goke's spelling of the API: they call GK_API_* where the
 * HiSilicon libraries define HI_MPI_*. openhisilicon reconciles the two at
 * compile time (include/hicompat.h); a binary driver needs them at link
 * time, from the executable, exactly as with group 1. All six are
 * R_ARM_JUMP_SLOT, so they bind lazily -- a board whose sensor is not one
 * of the eight never calls them.
 *
 * majestic defines none of them, so those eight drivers cannot be loading
 * under it at all. Six forwarding one-liners is a place raptor is
 * straightforwardly better than the reference consumer.
 *
 * Each forwards through g_hisi rather than through a file-static function
 * pointer, so the forwarders and the ISP loader cannot disagree about which
 * library is current. Before Phase 2 fills those pointers -- and after
 * hal_deinit clears them -- every one of these returns failure rather than
 * calling through NULL. HI_FAILURE is -1 in HiMPP, and a sensor driver that
 * gets it declines to register, which is the correct outcome when there is
 * no ISP to register with.
 */

#define HISI_TRAMPOLINE_FAIL (-1) /* HI_FAILURE */

#define HISI_FORWARD(field, ...)                                                                   \
    do {                                                                                           \
        hisi_state_t *st = g_hisi;                                                                 \
        if (!st || !st->field) {                                                                   \
            HAL_LOG_WARN("%s called with no ISP loaded", __func__);                                \
            return HISI_TRAMPOLINE_FAIL;                                                           \
        }                                                                                          \
        return st->field(__VA_ARGS__);                                                             \
    } while (0)

__attribute__((used)) int GK_API_ISP_SensorRegCallBack(int vi_pipe, void *sns_attr, void *reg)
{
    HISI_FORWARD(fn_isp_sensor_reg_cb, vi_pipe, sns_attr, reg);
}

__attribute__((used)) int GK_API_ISP_SensorUnRegCallBack(int vi_pipe, int sensor_id)
{
    HISI_FORWARD(fn_isp_sensor_unreg_cb, vi_pipe, sensor_id);
}

__attribute__((used)) int GK_API_ISP_GetModParam(void *mod_param)
{
    HISI_FORWARD(fn_isp_get_mod_param, mod_param);
}

__attribute__((used)) int GK_API_AE_SensorRegCallBack(int vi_pipe, void *ae_lib, void *sns_attr,
                                                      void *reg)
{
    HISI_FORWARD(fn_ae_sensor_reg_cb, vi_pipe, ae_lib, sns_attr, reg);
}

__attribute__((used)) int GK_API_AE_SensorUnRegCallBack(int vi_pipe, void *ae_lib, int sensor_id)
{
    HISI_FORWARD(fn_ae_sensor_unreg_cb, vi_pipe, ae_lib, sensor_id);
}

__attribute__((used)) int GK_API_AWB_SensorRegCallBack(int vi_pipe, void *awb_lib, void *sns_attr,
                                                       void *reg)
{
    HISI_FORWARD(fn_awb_sensor_reg_cb, vi_pipe, awb_lib, sns_attr, reg);
}

__attribute__((used)) int GK_API_AWB_SensorUnRegCallBack(int vi_pipe, void *awb_lib, int sensor_id)
{
    HISI_FORWARD(fn_awb_sensor_unreg_cb, vi_pipe, awb_lib, sensor_id);
}

/* ── Group 3: breaking the libisp <-> algorithm-library cycle ──
 *
 * THE ISP CYCLE, AND WHY musl MAKES IT THE EXECUTABLE'S PROBLEM.
 *
 * libisp.so and the algorithm libraries reference each other:
 *
 *   libisp.so    -> ISP_AlgRegister{Drc,Dehaze,Ldci}   (in lib_hi{drc,dehaze,ldci}.so)
 *   lib_hidrc.so -> g_astIspCtx, g_pastRegCfgCtx, ISP_MALLOC, IO_READ*, ...  (in libisp.so)
 *
 * The Phase -1 probe read those relocation types correctly -- the first
 * group is R_ARM_JUMP_SLOT and the second R_ARM_GLOB_DAT -- and drew the
 * wrong conclusion from them: that loading libisp.so first under RTLD_LAZY
 * would defer the function half and let the cycle resolve. That is true on
 * glibc and on uClibc. **It is false on musl, which implements no lazy
 * binding at all**: RTLD_LAZY is accepted and ignored, every dlopen
 * relocates fully, and so the JUMP_SLOT relocations bind as eagerly as the
 * data ones. Measured on the board -- dlopen("libisp.so", RTLD_LAZY) fails
 * with "Error relocating /usr/lib/libisp.so: ISP_AlgRegisterDrc: symbol not
 * found", which is exactly the deferral that was supposed to happen not
 * happening.
 *
 * Neither library can go first, so something outside the cycle has to
 * satisfy one direction of it. majestic does that by DT_NEEDED-linking all
 * fourteen libraries, which makes the loader map the whole graph before
 * relocating any of it -- not available to a dlopen-only backend.
 *
 * So the executable defines the three registrars, and the cycle unrolls:
 *
 *   1. dlopen(libisp.so) -- its three registrars bind to these forwarders,
 *      which are already in the global scope. It loads.
 *   2. dlopen(lib_hidrc.so) and friends -- their g_astIspCtx and the rest
 *      resolve against libisp.so, now loaded and RTLD_GLOBAL. They load.
 *   3. hisi_isp_open dlsyms the real registrars out of step 2 and stores
 *      them in the state.
 *   4. HI_MPI_ISP_Init calls ISP_AlgRegisterDrc, reaches this forwarder,
 *      and is passed through to the real one.
 *
 * Step 4 is why the forwarders must stay valid for the process lifetime and
 * cannot become a one-shot: libisp.so's GOT entry was bound once, at its
 * own dlopen, and points here for good. Being called before step 3 would
 * mean libisp registering algorithms before its own libraries are loaded,
 * which does not happen -- registration is inside ISP_Init -- but is
 * reported rather than assumed away.
 *
 * Only these three symbols need it: the whole of libisp.so's 39 undefined
 * symbols resolve from libmpi.so, libsecurec.so and libc apart from them.
 */

__attribute__((used)) int ISP_AlgRegisterDrc(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_drc, vi_pipe);
}

__attribute__((used)) int ISP_AlgRegisterDehaze(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_dehaze, vi_pipe);
}

__attribute__((used)) int ISP_AlgRegisterLdci(int vi_pipe)
{
    HISI_FORWARD(fn_alg_register_ldci, vi_pipe);
}

/* ── Group 4: mmap, where the ABI differs but the symbol does not ──
 *
 * THE ONE THE CLOSURE ANALYSIS COULD NOT SEE.
 *
 * Phase -1 computed the undefined-symbol closure over all 48 vendor
 * libraries and found ten symbols nothing provided. `mmap` was not among
 * them, because musl provides `mmap` and the reference resolves. What the
 * closure cannot see is that it resolves to the *wrong function*:
 *
 *   uClibc/arm  void *mmap(void *, size_t, int, int, int, off_t)  off_t = 32-bit
 *   musl/arm    void *mmap(void *, size_t, int, int, int, off_t)  off_t = 64-bit
 *
 * Same name, same arity, different sixth argument. On ARM EABI a 64-bit
 * stack argument is 8-byte aligned, so a musl callee reads the offset at
 * sp+8 while a uClibc caller wrote it at sp+4. libmpi.so is a uClibc build
 * and imports `mmap`; every mapping it makes on musl therefore gets an
 * offset assembled out of whatever follows the caller's frame.
 *
 * Measured, because the symptom is three layers away from the cause:
 * HI_MPI_ISP_MemInit succeeds, HI_MPI_ISP_SetPubAttr succeeds, and
 * HI_MPI_ISP_Init fails with 0xa01c8042 while libisp prints "Isp[0] WDR
 * mode doesn't config!". Underneath, the ISP's virtual registers are never
 * mapped:
 *
 *   ioctl(/dev/isp_dev, VREG_DRV_GETADDR)  -> phy_addr 0x420d0000   (correct)
 *   HI_MPI_SYS_Mmap(phy=0x420d0000, size=0x10000) -> NULL
 *     mmap(len=65536, fd=/dev/mmz_userdev, off=0xb6f7500000000000) -> EINVAL
 *
 * 0xb6f7500000000000 is a library address that happened to be on the stack.
 * SetPubAttr writes the WDR flag through that mapping and ISP_Init reads it
 * back; with no mapping the write goes nowhere and the read returns zero.
 *
 * majestic solves this and has all along -- it exports its own `mmap`
 * alongside __ctype_b and the other two, which is why its dynamic symbol
 * table has four entries where the closure predicted three. Its
 * implementation is six instructions: take a 32-bit offset, shift it right
 * by twelve, and call SYS_mmap2 directly. This is the same function.
 *
 * THE HAZARD, AND WHY IT NEEDS A DISPATCHER. Defining `mmap` in the
 * executable replaces it for *everything* in the process, and the daemon's
 * own callers are musl. The failure is not the narrow one it first looks
 * like -- "a non-zero offset would be misread" -- it is total: a musl caller
 * passing offset 0 does not write a zero at sp+4, it writes nothing there
 * at all, because its 64-bit off_t is 8-byte aligned and sp+4 is padding.
 * A uClibc-ABI definition therefore reads stack garbage as the offset for
 * every raptor mmap in the daemon.
 *
 * Measured that way too. With the plain uClibc definition the ISP came up
 * and the pipeline bound, and then rss_ring's mmap of a 2 MiB shm object
 * got a garbage page offset: the kernel maps a shared file mapping without
 * validating the offset against the file size and only faults on access, so
 * the daemon died with SIGBUS on the first write to the ring header.
 *
 * Neither convention can serve both, and nothing in the call itself
 * distinguishes them -- the two ABIs disagree about which stack word holds
 * the offset, and each leaves the other word undefined. What does
 * distinguish them is *who is calling*. Every uClibc caller is a vendor
 * library this backend dlopen'd; every musl caller is the daemon itself or
 * one of raptor's own shared libraries. So the shim takes the return
 * address, asks the loader which object it lies in, and reads the offset
 * from the word that object's compiler would have written it to.
 *
 * hisi_reg_access below does not go through any of this -- it calls
 * hisi_mmap_pages directly, because it knows its own offset.
 */

/* syscall() is POSIX and behind a feature macro under -std=c11, and this
 * file will not widen its feature set for one prototype; declaring it is
 * both smaller and more honest about what is being reached for. */
extern long syscall(long number, ...);

/*
 * Dl_info and dladdr, declared rather than included for the same reason:
 * dladdr is a GNU extension that <dlfcn.h> hides behind _GNU_SOURCE. The
 * layout is identical on glibc, musl and the BSDs and has been since the
 * interface appeared.
 */
typedef struct {
    const char *dli_fname;
    void *dli_fbase;
    const char *dli_sname;
    void *dli_saddr;
} hisi_dl_info;

extern int dladdr(const void *addr, hisi_dl_info *info);

/*
 * SYS_mmap2 takes the offset in 4096-byte pages, which is what makes a
 * 32-bit offset argument enough to address a 44-bit file. Bypassing libc
 * is what lets one function serve both conventions: the syscall has only
 * ever had the page-offset form, so there is no second ABI down here.
 *
 * The offset is a *page* count and 32 bits wide, and that is a correctness
 * requirement rather than a convenience. GCC compiles the shim below into a
 * sibling call, which reuses the caller's outgoing-argument area for this
 * function's arguments -- and a uClibc caller of mmap allocated exactly
 * eight bytes of it, for fd and a 32-bit offset. A 64-bit offset parameter
 * here would be written at +8 and +12, eight bytes past the end, into the
 * caller's own frame.
 *
 * That was measured too: with an unsigned long long here, HI_MPI_SYS_Mmap
 * returned the right address and libmpi died a few calls later with its
 * saved registers overwritten. Keeping the argument list no wider than the
 * narrowest caller's is what makes the tail call safe.
 */
#if defined(__arm__)
static void *hisi_mmap_pages(void *addr, size_t len, int prot, int flags, int fd,
                             unsigned long pgoff)
{
    return (void *)syscall(SYS_mmap2, addr, len, prot, flags, fd, pgoff);
}
#else
/*
 * Off 32-bit ARM there is no SYS_mmap2 and no vendor library to interpose
 * for, so this is libc's mmap and the shim below does not exist at all. The
 * host build has to keep compiling: it is the property the dlopen-nothing
 * design exists to protect, and it is where the portable _Static_asserts in
 * this backend actually run.
 */
static void *hisi_mmap_pages(void *addr, size_t len, int prot, int flags, int fd,
                             unsigned long pgoff)
{
    return mmap(addr, len, prot, flags, fd, (off_t)pgoff * 4096);
}
#endif

#if defined(__arm__)

/*
 * Which object holds this code -- the daemon, since raptor-hal is a static
 * archive. Resolved once; a loaded object's name does not change.
 */
static const char *hisi_own_object(void)
{
    static const char *name;
    static bool asked;
    hisi_dl_info info;

    if (!asked) {
        asked = true;
        if (dladdr((const void *)(uintptr_t)&hisi_mmap_pages, &info))
            name = info.dli_fname;
    }
    return name;
}

/*
 * Whether a return address belongs to a uClibc-built vendor library.
 *
 * Stated as "not one of ours" rather than as a list of vendor names,
 * because the vendor set is open -- libmpi, libisp, six algorithm
 * libraries, thirty-four sensor drivers, and whatever a Goke build
 * substitutes -- while raptor's own set is two rules long and this tree
 * controls both of them.
 *
 * An address the loader cannot place counts as ours. That is the safer
 * default of the two: guessing musl for a vendor library reproduces the ISP
 * failure above, which is loud, names itself in the log, and stops
 * bring-up; guessing uClibc for raptor's own code is the SIGBUS, which
 * happens later, somewhere else, with no way back to here.
 */
static bool hisi_caller_is_vendor(const void *ra)
{
    const char *own = hisi_own_object();
    hisi_dl_info info;
    const char *base;

    if (!dladdr(ra, &info) || !info.dli_fname)
        return false;
    if (own && strcmp(info.dli_fname, own) == 0)
        return false;

    base = strrchr(info.dli_fname, '/');
    base = base ? base + 1 : info.dli_fname;
    return strncmp(base, "librss_", 7) != 0;
}

/*
 * Declared with an explicit assembler name so that it exports as `mmap`
 * without colliding with the musl prototype <sys/mman.h> has already
 * introduced.
 *
 * The trailing three arguments are the stack words the two conventions
 * disagree about, taken individually rather than as an offset: AAPCS puts
 * the fifth argument (fd) at sp+0, so uClibc's 32-bit offset lands at sp+4
 * and musl's 64-bit one at sp+8 and sp+12. Naming all three as separate
 * words is what lets the shim read either without knowing in advance which
 * it will need. Only the ones the caller actually wrote are ever read.
 */
void *hisi_mmap_shim(void *addr, size_t len, int prot, int flags, int fd, unsigned long w4,
                     unsigned long w8, unsigned long w12) __asm__("mmap");

__attribute__((used)) void *hisi_mmap_shim(void *addr, size_t len, int prot, int flags, int fd,
                                           unsigned long w4, unsigned long w8, unsigned long w12)
{
    void *ra = __builtin_return_address(0);
    unsigned long long off;

    if (hisi_caller_is_vendor(ra))
        off = w4;
    else
        off = (unsigned long long)w8 | ((unsigned long long)w12 << 32);

    return hisi_mmap_pages(addr, len, prot, flags, fd, (unsigned long)(off >> 12));
}

#endif /* __arm__ */

/*
 * hisi_check_trampolines -- prove the executable really exports them.
 *
 * Runs before the first vendor dlopen, and not at ISP-open time, because by
 * then it would be too late to be informative: libsecurec.so arrives as a
 * DT_NEEDED of libmpi.so, so __ctype_b's eager binding has already either
 * succeeded or failed before any ISP code runs.
 *
 * dlsym(RTLD_DEFAULT, "__ctype_b") searching the global scope and finding
 * *this* object is what the vendor libraries will do. A mismatch means the
 * link dropped the definition -- no --export-dynamic, a static link, the
 * archive member not extracted -- and turns what would otherwise be an
 * unattributable SIGSEGV inside libisp.so into one log line naming the
 * cause.
 *
 * Non-fatal by design: a build that gets this wrong should say so loudly
 * and then fail where the failure is, rather than refusing to start and
 * leaving no evidence.
 */
static void hisi_check_trampolines(void)
{
    void *found;

    __ctype_b = *__ctype_b_loc();

    found = dlsym(RTLD_DEFAULT, "__ctype_b");
    if (found == (void *)&__ctype_b) {
        HAL_LOG_DBG("trampolines: __ctype_b exported and resolvable");
        return;
    }

    HAL_LOG_ERR("trampolines: __ctype_b resolves to %p, not %p -- the executable is not "
                "exporting it. Vendor libraries will fault inside libsecurec.so. "
                "Link with -Wl,--export-dynamic and keep hal_common.o in the link.",
                found, (void *)&__ctype_b);
}

/* ================================================================
 * REGISTER ACCESS
 *
 * HiMPP publishes no register accessor -- IMP_System_ReadReg32 has no
 * counterpart -- so this is /dev/mem, which is also how the chip ID is
 * read. Opened and closed per access rather than kept mapped: reads are
 * rare (identification at init, and whatever a caller asks for), and a
 * long-lived /dev/mem mapping is a liability worth more than the syscalls
 * it saves.
 * ================================================================ */

static int hisi_reg_access(uint32_t addr, uint32_t *val, bool write)
{
    long page = sysconf(_SC_PAGESIZE);
    off_t base;
    void *map;
    int fd;

    if (page <= 0)
        page = 4096;
    base = (off_t)addr & ~((off_t)page - 1);

    fd = open("/dev/mem", (write ? O_RDWR : O_RDONLY) | O_SYNC);
    if (fd < 0) {
        HAL_LOG_ERR("/dev/mem: %s", strerror(errno));
        return RSS_ERR_IO;
    }

    /* hisi_mmap_pages, not mmap: this file *defines* mmap, and a physical
     * base is exactly the non-zero offset the two conventions disagree
     * about. Going straight to the syscall skips the dispatch rather than
     * relying on it to classify this file's own code. See above. */
    map = hisi_mmap_pages(NULL, (size_t)page, write ? (PROT_READ | PROT_WRITE) : PROT_READ,
                          MAP_SHARED, fd, (unsigned long)((unsigned long long)base >> 12));
    close(fd);
    if (map == MAP_FAILED) {
        HAL_LOG_ERR("/dev/mem mmap at 0x%08x: %s", addr, strerror(errno));
        return RSS_ERR_IO;
    }

    {
        volatile uint32_t *p = (volatile uint32_t *)((char *)map + (addr - (uint32_t)base));

        if (write)
            *p = *val;
        else
            *val = *p;
    }

    munmap(map, (size_t)page);
    return RSS_OK;
}

/*
 * hisi_read_chip_id -- SCSYSID0, and the family name it encodes.
 *
 * The encoding is positional rather than a lookup table: the word reads as
 * the marketing name with the 'V' elided, so 0x3516E300 is Hi3516EV300 and
 * 0x3518E300 is Hi3518EV300. A leading nibble of 7 means a Goke rebrand and
 * takes the "GK" prefix instead of "Hi" -- divinus derives the name the same
 * way (src/hal/support.c:274-284), which is worth knowing because it means
 * two independent readings of the silicon agree.
 *
 * The raw word is kept alongside the name. A part nobody here has held gets
 * a plausible-looking name out of this function, and the number is the half
 * that can be checked.
 */
static void hisi_read_chip_id(hisi_state_t *st)
{
    uint32_t id = 0;

    if (hisi_reg_access(HISI_SCSYSID0_ADDR, &id, false) != RSS_OK) {
        snprintf(st->chip_name, sizeof(st->chip_name), "%s", HAL_PLATFORM_NAME);
        HAL_LOG_WARN("chip id unreadable; assuming %s", st->chip_name);
        return;
    }

    st->chip_id = id;
    /* "Hi" + the top five hex digits + the elided 'V' + the last three:
     * 0x3516E300 -> Hi3516EV300. Eleven characters, whatever the part. */
    snprintf(st->chip_name, sizeof(st->chip_name), "%s%05XV%03X",
             (id >> 28) == 0x7 ? "GK" : "Hi", id >> 12, id & 0xfff);

    HAL_LOG_INFO("chip: %s (SCSYSID0 0x%08x)", st->chip_name, id);
}

#ifdef HAL_MODULE_VIDEO
static int hal_sys_read_reg32(void *ctx, uint32_t addr, uint32_t *val)
{
    (void)ctx;
    if (!val)
        return RSS_ERR_INVAL;
    return hisi_reg_access(addr, val, false);
}

static int hal_sys_write_reg32(void *ctx, uint32_t addr, uint32_t val)
{
    (void)ctx;
    return hisi_reg_access(addr, &val, true);
}
#endif

#ifdef HAL_MODULE_VIDEO
/* ================================================================
 * MIPI RECEIVER
 *
 * The kernel side of VI, configured entirely through ioctls on
 * /dev/hi_mipi. Sequence taken from the vendor's own
 * SAMPLE_COMM_VI_StartMIPI (mpp/sample/common/sample_comm_vi.c:2014):
 *
 *   SET_HS_MODE -> ENABLE_MIPI_CLOCK -> RESET_MIPI
 *               -> ENABLE_SENSOR_CLOCK -> RESET_SENSOR
 *               -> SET_DEV_ATTR
 *               -> UNRESET_MIPI -> UNRESET_SENSOR
 *
 * The two resets bracket SET_DEV_ATTR rather than preceding it, and that
 * is the part worth not rearranging: the receiver's lane configuration is
 * latched out of reset, so an attribute written while the block is running
 * is accepted and ignored. A pipeline built that way produces no frames and
 * reports no error.
 * ================================================================ */

static int hisi_mipi_ioctl(int fd, unsigned long req, void *arg, const char *what)
{
    if (ioctl(fd, req, arg) < 0) {
        HAL_LOG_ERR("mipi: %s failed: %s", what, strerror(errno));
        return RSS_ERR_IO;
    }
    return RSS_OK;
}

static int hisi_mipi_configure(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v4_combo_dev_attr attr;
    unsigned int devno = HISI_VI_DEV;
    unsigned int hs_mode = V4_LANE_DIVIDE_MODE_0;
    unsigned int sns_src = 0;
    int fd;
    int ret;

    fd = open(V4_MIPI_DEV, O_RDWR);
    if (fd < 0) {
        HAL_LOG_ERR("mipi: %s: %s", V4_MIPI_DEV, strerror(errno));
        return RSS_ERR_NOENT;
    }

    memset(&attr, 0, sizeof(attr));
    attr.devno = devno;
    attr.input_mode = m->input_mode;
    /*
     * MIPI_DATA_RATE_X1 -- one pixel per clock. X2 exists for sensors fast
     * enough to need two, and no gen4 stock INI selects it; the key that
     * would say so does not appear in any shipped file, so there is nothing
     * to read it from and X1 is what the vendor samples use for every MIPI
     * sensor on this part.
     */
    attr.data_rate = V4_MIPI_DATA_RATE_X1;
    /* (0,0), not the ini's DevRect_x/y -- see hisi_sensor_mode_load. This
     * window is a crop out of what the sensor actually sends, and the
     * sensor sends exactly DevRect_w by DevRect_h. */
    attr.img_rect.x = 0;
    attr.img_rect.y = 0;
    attr.img_rect.width = m->dev_rect.width;
    attr.img_rect.height = m->dev_rect.height;

    attr.mipi_attr.input_data_type = m->mipi_data_type;
    attr.mipi_attr.wdr_mode = V4_MIPI_WDR_MODE_NONE;
    memcpy(attr.mipi_attr.lane_id, m->lane_id, sizeof(attr.mipi_attr.lane_id));

    ret = hisi_mipi_ioctl(fd, V4_MIPI_SET_HS_MODE, &hs_mode, "SET_HS_MODE");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V4_MIPI_ENABLE_MIPI_CLOCK, &devno, "ENABLE_MIPI_CLOCK");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V4_MIPI_RESET_MIPI, &devno, "RESET_MIPI");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V4_MIPI_ENABLE_SENSOR_CLOCK, &sns_src, "ENABLE_SENSOR_CLOCK");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V4_MIPI_RESET_SENSOR, &sns_src, "RESET_SENSOR");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V4_MIPI_SET_DEV_ATTR, &attr, "SET_DEV_ATTR");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V4_MIPI_UNRESET_MIPI, &devno, "UNRESET_MIPI");
    if (ret)
        goto out;
    ret = hisi_mipi_ioctl(fd, V4_MIPI_UNRESET_SENSOR, &sns_src, "UNRESET_SENSOR");
    if (ret)
        goto out;

    st->mipi_configured = true;
    HAL_LOG_INFO("mipi: dev %u, RAW%d, %ux%u+%d+%d, lanes %d|%d|%d|%d", devno, m->raw_bitness,
                 m->dev_rect.width, m->dev_rect.height, m->dev_rect.x, m->dev_rect.y,
                 m->lane_id[0], m->lane_id[1], m->lane_id[2], m->lane_id[3]);

out:
    close(fd);
    return ret;
}

/*
 * Reverse of the above, per SAMPLE_COMM_VI_StopMIPI: reset the sensor, stop
 * its clock, reset the receiver, stop its clock. Failures are logged and
 * never propagated -- this runs during teardown, where stopping halfway is
 * worse than any individual failure.
 */
static void hisi_mipi_shutdown(hisi_state_t *st)
{
    unsigned int devno = HISI_VI_DEV;
    unsigned int sns_src = 0;
    int fd;

    if (!st->mipi_configured)
        return;

    fd = open(V4_MIPI_DEV, O_RDWR);
    if (fd < 0) {
        HAL_LOG_WARN("mipi: %s on shutdown: %s", V4_MIPI_DEV, strerror(errno));
        st->mipi_configured = false;
        return;
    }

    hisi_mipi_ioctl(fd, V4_MIPI_RESET_SENSOR, &sns_src, "RESET_SENSOR");
    hisi_mipi_ioctl(fd, V4_MIPI_DISABLE_SENSOR_CLOCK, &sns_src, "DISABLE_SENSOR_CLOCK");
    hisi_mipi_ioctl(fd, V4_MIPI_RESET_MIPI, &devno, "RESET_MIPI");
    hisi_mipi_ioctl(fd, V4_MIPI_DISABLE_MIPI_CLOCK, &devno, "DISABLE_MIPI_CLOCK");

    close(fd);
    st->mipi_configured = false;
}

/* ================================================================
 * VI -- DEVICE, PIPE, CHANNEL
 * ================================================================ */

/*
 * hisi_vi_vpss_mode -- read-modify-write the VI/VPSS coupling.
 *
 * VI_OFFLINE_VPSS_OFFLINE: the VI pipe writes raw to DDR, the VI channel
 * writes YUV frames to a VB block, and VPSS reads them over an explicit
 * bind.
 *
 * VPSS has to be offline because that is the mode the pipeline built above
 * describes -- a VI channel with attributes, enabled, and bound onward.
 * sample_comm_vi.c:2873 skips HI_MPI_VI_EnableChn in exactly the
 * VPSS-online modes, which is the same statement from the other side: with
 * VPSS online the group is fed in hardware and the channel is bypassed.
 *
 * VI is offline because this part will not have it otherwise at this
 * resolution. VI_ONLINE_VPSS_OFFLINE is the vendor's own default for a
 * single-sensor pipeline (sample_comm_vi.c:3771) and would save the raw
 * round trip through DDR, which is the largest single consumer of VB; on a
 * 2592x1944 IMX335 the driver rejects it and HI_MPI_SYS_SetVIVPSSMode
 * fails. Asking for offline is therefore not a preference but the only
 * setting that is accepted, and asking for it explicitly matters: the mode
 * has to be *set*, because the board ships in VI_OFFLINE_VPSS_ONLINE.
 *
 * Which one is set is not cosmetic and the failure is silent. Left on the
 * board's default, everything up to and including the ISP runs -- the pipe
 * captures, AE converges -- and /proc/umap/vpss shows RecvPic0 at zero with
 * every channel still 0x0, because nothing is delivering to a group whose
 * only declared input is a bind the hardware path does not use.
 *
 * The call has to run before HI_MPI_SYS_Init, which is why it is in
 * hal_init and not with the rest of the VI bring-up: the coupling is fixed
 * when the system starts, and setting it afterwards is accepted and
 * ignored. The vendor sequences it the same way -- SAMPLE_COMM_VI_SetParam
 * comes before SAMPLE_COMM_SYS_Init in every sample.
 *
 * Read-modify-write rather than build-and-set, following the vendor: the
 * array has an entry per pipe and writing a fresh one would reset the mode
 * of a pipe this backend does not drive.
 *
 * Failure is a warning, not an error. The driver has a default and refusing
 * to start a pipeline because the mode could not be confirmed would trade a
 * working camera for a tidier log.
 */
static void hisi_vi_vpss_mode(hisi_state_t *st)
{
    v4_vi_vpss_mode mode;

    /* What the board ships with, and so what is in force if the mode can be
     * neither read nor written. */
    st->vi_vpss_mode = V4_VI_OFFLINE_VPSS_ONLINE;

    if (!st->sys.fnSetVIVPSSMode || !st->sys.fnGetVIVPSSMode)
        return;

    memset(&mode, 0, sizeof(mode));
    if (st->sys.fnGetVIVPSSMode(&mode)) {
        HAL_LOG_WARN("HI_MPI_SYS_GetVIVPSSMode failed; leaving the coupling at its default");
        return;
    }

    if (mode.mode[HISI_VI_PIPE] != V4_VI_OFFLINE_VPSS_OFFLINE) {
        mode.mode[HISI_VI_PIPE] = V4_VI_OFFLINE_VPSS_OFFLINE;
        if (st->sys.fnSetVIVPSSMode(&mode)) {
            HAL_LOG_WARN("HI_MPI_SYS_SetVIVPSSMode(pipe %d = VI_OFFLINE_VPSS_OFFLINE) failed",
                         HISI_VI_PIPE);
            return;
        }
    }

    st->vi_vpss_mode = mode.mode[HISI_VI_PIPE];
    HAL_LOG_INFO("vi/vpss coupling: mode %d", (int)st->vi_vpss_mode);
}


static int hisi_vi_bringup(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v4_vi_dev_attr dev;
    v4_vi_dev_bind_pipe bind;
    v4_vi_pipe_attr pipe;
    v4_vi_chn_attr chn;
    int ret;

    /* ── Device: the raw side, configured from the sensor mode INI ── */
    memset(&dev, 0, sizeof(dev));
    dev.intf_mode = m->intf_mode;
    dev.work_mode = m->work_mode;
    dev.component_mask[0] = m->component_mask[0];
    dev.component_mask[1] = m->component_mask[1];
    dev.scan_mode = m->scan_mode;
    /* AD channel IDs are -1 ("unused") on any part with no analog decoder,
     * and zero would name channel 0. calloc left zeros, so this is not
     * optional. */
    dev.ad_chn_id[0] = -1;
    dev.ad_chn_id[1] = -1;
    dev.ad_chn_id[2] = -1;
    dev.ad_chn_id[3] = -1;
    dev.data_seq = m->data_seq;
    dev.sync_cfg = m->sync_cfg;
    dev.input_data_type = m->input_data_type;
    dev.data_reverse = m->data_reverse;
    dev.size.width = m->dev_rect.width;
    dev.size.height = m->dev_rect.height;
    dev.wdr_attr.wdr_mode = V4_WDR_MODE_NONE;
    dev.data_rate = V4_DATA_RATE_X1;

    ret = st->vi.fnSetDevAttr(HISI_VI_DEV, &dev);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VI_SetDevAttr(%d) failed: 0x%x", HISI_VI_DEV, ret);
        return RSS_ERR_IO;
    }

    ret = st->vi.fnEnableDev(HISI_VI_DEV);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VI_EnableDev(%d) failed: 0x%x", HISI_VI_DEV, ret);
        return RSS_ERR_IO;
    }
    st->vi_dev_enabled = true;

    /* ── Device to pipe ── */
    memset(&bind, 0, sizeof(bind));
    bind.num = 1;
    bind.pipe_id[0] = HISI_VI_PIPE;
    /* The second entry is -1, not 0: zero names pipe 0, which is the one
     * already bound, and a device bound twice to the same pipe is rejected. */
    bind.pipe_id[1] = -1;

    ret = st->vi.fnSetDevBindPipe(HISI_VI_DEV, &bind);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VI_SetDevBindPipe(%d) failed: 0x%x", HISI_VI_DEV, ret);
        return RSS_ERR_IO;
    }

    /* ── Pipe: the ISP's own object ── */
    memset(&pipe, 0, sizeof(pipe));
    pipe.bypass_mode = V4_VI_PIPE_BYPASS_NONE;
    pipe.max_w = m->dev_rect.width;
    pipe.max_h = m->dev_rect.height;
    pipe.pix_fmt = m->pixel_format;
    /*
     * COMPRESS_MODE_NONE, and this is measured rather than reasoned.
     *
     * SEG compression on the pipe's own frames looks free -- nothing in
     * userspace reads a VI frame, the datapath is VI -> VPSS in the kernel,
     * and it is a straight DDR bandwidth saving. The driver disagrees:
     * HI_MPI_VI_CreatePipe returns 0xa0108003 (VI / ERROR /
     * EN_ERR_ILLEGAL_PARAM) for it on an EV300. Every VI_PIPE_ATTR_S in the
     * vendor's own table uses NONE, including the 2592x1944 RAW12 entry
     * this pipeline is the same shape as (sample_comm_vi.c,
     * PIPE_ATTR_2592x1944_RAW12_420_3DNR_RFR).
     */
    pipe.compress_mode = V4_COMPRESS_MODE_NONE;
    pipe.bit_width = m->bit_width;
    /*
     * 3DNR belongs to the VPSS group on gen4, not to the VI pipe, so
     * bNrEn is false here and set there. Setting it in both places
     * allocates two reference-frame buffers and runs the filter twice.
     *
     * stNrAttr is filled in *regardless*, which is the part a reader would
     * not guess and the vendor's table makes plain: every entry populates
     * it even with bNrEn false. Left to the memset it would describe a
     * reference frame in PIXEL_FORMAT_RGB_444 -- enumerator 0 -- and the
     * driver validates the field whether or not it is going to use it.
     */
    pipe.nr_en = 0;
    pipe.nr_attr.pix_fmt = V4_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    pipe.nr_attr.bit_width = V4_DATA_BITWIDTH_8;
    pipe.nr_attr.nr_ref_source = V4_VI_NR_REF_FROM_RFR;
    pipe.nr_attr.compress_mode = V4_COMPRESS_MODE_NONE;
    pipe.sharpen_en = 0;
    pipe.frame_rate.src_frame_rate = -1;
    pipe.frame_rate.dst_frame_rate = -1;
    pipe.discard_pro_pic = 0;

    ret = st->vi.fnCreatePipe(HISI_VI_PIPE, &pipe);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VI_CreatePipe(%d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->vi_pipe_created = true;

    ret = st->vi.fnStartPipe(HISI_VI_PIPE);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VI_StartPipe(%d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->vi_pipe_started = true;

    /* ── Channel: the pipe's YUV output, which is what VPSS binds to ── */
    memset(&chn, 0, sizeof(chn));
    chn.size.width = m->dev_rect.width;
    chn.size.height = m->dev_rect.height;
    chn.pixel_format = V4_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    chn.dynamic_range = V4_DYNAMIC_RANGE_SDR8;
    chn.video_format = V4_VIDEO_FORMAT_LINEAR;
    chn.compress_mode = V4_COMPRESS_MODE_NONE;
    /* Orientation lives at the sensor; see hisi_sensor_bringup. Left to the
     * memset so a reader looking for where this backend sets mirror and
     * flip on the VI channel finds nothing, which is the point. */
    chn.depth = 0;
    chn.frame_rate.src_frame_rate = -1;
    chn.frame_rate.dst_frame_rate = -1;

    ret = st->vi.fnSetChnAttr(HISI_VI_PIPE, HISI_VI_CHN, &chn);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VI_SetChnAttr(%d, %d) failed: 0x%x", HISI_VI_PIPE, HISI_VI_CHN, ret);
        return RSS_ERR_IO;
    }

    ret = st->vi.fnEnableChn(HISI_VI_PIPE, HISI_VI_CHN);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VI_EnableChn(%d, %d) failed: 0x%x", HISI_VI_PIPE, HISI_VI_CHN, ret);
        return RSS_ERR_IO;
    }
    st->vi_chn_enabled = true;

    HAL_LOG_INFO("vi: dev %d -> pipe %d -> chn %d, %ux%u RAW%d", HISI_VI_DEV, HISI_VI_PIPE,
                 HISI_VI_CHN, m->dev_rect.width, m->dev_rect.height, m->raw_bitness);
    return RSS_OK;
}

/* ================================================================
 * SENSOR
 * ================================================================ */

/*
 * hisi_sensor_bringup -- load the driver and wire it into the ISP.
 *
 * The order is the vendor's, from SAMPLE_COMM_VI_StartIsp
 * (mpp/sample/common/sample_comm_vi.c:3285) and
 * SAMPLE_COMM_ISP_Sensor_Regiter_callback (sample_comm_isp.c:440):
 *
 *   pfnRegisterCallback -> pfnSetBusInfo -> AE_Register -> AWB_Register
 *
 * **This differs from divinus**, which calls pfnSetBusInfo first. Both work
 * and the reason they do is worth stating rather than leaving as a
 * coincidence: pfnRegisterCallback touches no I2C. Reading the shipped
 * driver source confirms it -- openhisilicon's
 * libraries/sensor/hi3516ev200/sony_imx335/imx335_cmos.c:1965 only fills in
 * three function tables and hands them to GK_API_{ISP,AE,AWB}_SensorRegCallBack,
 * while IMX335_set_bus_info at :1924 just stores the adapter number for
 * later. The bus is not used until the ISP thread runs. The SDK's order is
 * followed because the SDK is the authority when the two disagree; the
 * divergence is noted here rather than silently resolved.
 *
 * That same file is also the direct evidence for the GK_API_* trampolines in
 * this translation unit: those three calls are exactly the symbols the
 * executable has to export.
 */
static int hisi_sensor_bringup(hisi_state_t *st, const rss_sensor_config_t *cfg)
{
    v4_alg_lib ae_lib;
    v4_alg_lib awb_lib;
    v4_sns_commbus bus;
    int ret;

    ret = v4_snr_load(&st->snr, st->mode.dll_file, st->mode.obj_name);
    if (ret)
        return ret;

    /*
     * The algorithm library names the sensor driver will hand on to
     * AE_SensorRegCallBack and AWB_SensorRegCallBack. They must match what
     * the loaded ISP stack registers under -- hisi_isp_open picks the
     * spelling from which libraries are present, because HiSilicon's and
     * openhisilicon's differ.
     *
     * s32Id is the VI pipe, which is also the ISP index. The vendor writes
     * IspDev here and IspDev is the pipe.
     */
    memset(&ae_lib, 0, sizeof(ae_lib));
    memset(&awb_lib, 0, sizeof(awb_lib));
    ae_lib.id = HISI_VI_PIPE;
    awb_lib.id = HISI_VI_PIPE;
    snprintf(ae_lib.lib_name, sizeof(ae_lib.lib_name), "%s", st->isp.ae_lib_name);
    snprintf(awb_lib.lib_name, sizeof(awb_lib.lib_name), "%s", st->isp.awb_lib_name);

    ret = st->snr.obj->pfnRegisterCallback(HISI_VI_PIPE, &ae_lib, &awb_lib);
    if (ret) {
        HAL_LOG_ERR("sensor: pfnRegisterCallback(pipe %d) failed: 0x%x. If this is one of the "
                    "Goke-tier drivers, check that the GK_API_* forwarders are exported "
                    "(nm -D on the daemon).",
                    HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->sensor_registered = true;

    /*
     * The I2C adapter, passed by value in one byte. rvd's config carries
     * it; a board that does not set it gets adapter 0, which is where every
     * gen4 reference design puts the sensor.
     */
    memset(&bus, 0, sizeof(bus));
    bus.i2c_dev = (signed char)(cfg && cfg->i2c_adapter >= 0 ? cfg->i2c_adapter : 0);
    ret = st->snr.obj->pfnSetBusInfo(HISI_VI_PIPE, bus);
    if (ret) {
        HAL_LOG_ERR("sensor: pfnSetBusInfo(pipe %d, i2c %d) failed: 0x%x", HISI_VI_PIPE,
                    (int)bus.i2c_dev, ret);
        return RSS_ERR_IO;
    }

    ret = st->isp.fnAeRegister(HISI_VI_PIPE, &ae_lib);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_AE_Register(pipe %d, \"%s\") failed: 0x%x -- the ISP registers "
                    "algorithms by name, so a mismatch here is a naming mismatch",
                    HISI_VI_PIPE, ae_lib.lib_name, ret);
        return RSS_ERR_IO;
    }
    st->ae_registered = true;

    ret = st->isp.fnAwbRegister(HISI_VI_PIPE, &awb_lib);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_AWB_Register(pipe %d, \"%s\") failed: 0x%x", HISI_VI_PIPE,
                    awb_lib.lib_name, ret);
        return RSS_ERR_IO;
    }
    st->awb_registered = true;

    /*
     * Orientation, applied at the sensor.
     *
     * Same decision as the SigmaStar backend and for the same reason: the
     * driver latches mirror and flip during its own register writes, before
     * any caller can reach an ISP op, and a camera is mounted one way round
     * and stays there. Doing it downstream instead would put it on the VPSS
     * channel, where it is per-stream and would have to be repeated.
     *
     * pfnMirrorFlip is optional in the vtable -- a few drivers omit it --
     * so a configured orientation on a driver that cannot do it is reported
     * rather than silently dropped.
     */
    st->mirror = cfg && cfg->hflip ? 1 : 0;
    st->flip = cfg && cfg->vflip ? 1 : 0;
    if (st->mirror || st->flip) {
        v4_sns_mirrorflip mf = st->mirror ? (st->flip ? V4_SNS_MIRROR_FLIP : V4_SNS_MIRROR)
                                          : V4_SNS_FLIP;

        if (st->snr.obj->pfnMirrorFlip)
            st->snr.obj->pfnMirrorFlip(HISI_VI_PIPE, mf);
        else
            HAL_LOG_WARN("sensor: %s has no pfnMirrorFlip; hflip=%d vflip=%d not applied",
                         st->snr.obj_name, st->mirror, st->flip);
    }

    return RSS_OK;
}

/* ================================================================
 * ISP
 * ================================================================ */

/*
 * The ISP service loop.
 *
 * HI_MPI_ISP_Run does not return while the ISP is up: it is the 3A loop and
 * runs for the lifetime of the pipeline. So it gets a thread of its own, and
 * teardown stops it by calling HI_MPI_ISP_Exit -- which is what makes Run
 * return -- rather than by cancelling it. A thread cancelled inside the
 * vendor library leaves its locks held, and the next HI_MPI_ISP_Init in the
 * same process then blocks forever.
 */
static void *hisi_isp_thread(void *arg)
{
    hisi_state_t *st = (hisi_state_t *)arg;
    int ret;

    ret = st->isp.fnRun(HISI_VI_PIPE);

    /* Returning at all means the ISP stopped. During teardown that is
     * expected and the flag is already clear; at any other time it is the
     * pipeline dying and worth a line. */
    if (__atomic_load_n(&st->isp_thread_running, __ATOMIC_ACQUIRE))
        HAL_LOG_ERR("HI_MPI_ISP_Run(pipe %d) returned 0x%x -- the ISP has stopped", HISI_VI_PIPE,
                    ret);

    /* Published last: teardown waits on this to decide whether joining is
     * safe. Release ordering so everything above it is visible first. */
    __atomic_store_n(&st->isp_thread_done, 1, __ATOMIC_RELEASE);
    return NULL;
}

/*
 * hisi_isp_thread_stop -- join the 3A thread, but not forever.
 *
 * HI_MPI_ISP_Exit is what makes HI_MPI_ISP_Run return, so the thread should
 * be gone within a frame or two. A plain pthread_join would be correct
 * whenever that holds -- and would hang rvd's shutdown permanently on any
 * board or library build where it does not. That is a failure mode raptor
 * has already paid for once on another vendor (see
 * FIXED-i6c-rvd-unkillable-wedge.md in the parent tree), and the cost of
 * not repeating it is this loop.
 *
 * Cancelling is not the alternative. A thread cancelled inside the vendor
 * library leaves its locks held, and the next HI_MPI_ISP_Init in the same
 * process then blocks forever -- trading a hang at shutdown for a hang at
 * the next start. Detaching leaks a thread that is already stuck in a
 * library the process is about to stop using, and lets everything else
 * finish.
 */
#define HISI_ISP_JOIN_TIMEOUT_MS 2000
#define HISI_ISP_JOIN_POLL_MS 10

static void hisi_isp_thread_stop(hisi_state_t *st)
{
    int waited;

    for (waited = 0; waited < HISI_ISP_JOIN_TIMEOUT_MS; waited += HISI_ISP_JOIN_POLL_MS) {
        struct timeval tv;

        if (__atomic_load_n(&st->isp_thread_done, __ATOMIC_ACQUIRE)) {
            pthread_join(st->isp_thread, NULL);
            return;
        }

        /* select with no descriptors is the sleep this file can reach:
         * nanosleep and usleep are both behind POSIX feature macros under
         * -std=c11, and widening the feature set of a translation unit for
         * a ten-millisecond pause is not a trade worth making. */
        tv.tv_sec = 0;
        tv.tv_usec = HISI_ISP_JOIN_POLL_MS * 1000;
        select(0, NULL, NULL, NULL, &tv);
    }

    HAL_LOG_ERR("isp: HI_MPI_ISP_Run did not return %d ms after HI_MPI_ISP_Exit; detaching the "
                "3A thread rather than blocking shutdown",
                HISI_ISP_JOIN_TIMEOUT_MS);
    pthread_detach(st->isp_thread);
}

static int hisi_isp_bringup(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v4_isp_pub_attr pub;
    int ret;

    /*
     * Teardown-first, for the ISP as for SYS and VB.
     *
     * The ISP's virtual registers are not process memory: HI_MPI_ISP_MemInit
     * asks the kernel to allocate them out of MMZ (VREG_DRV_INIT), and they
     * are freed only by HI_MPI_ISP_Exit. A previous consumer that was killed
     * rather than closed -- which on a camera is the normal case, not the
     * exceptional one -- leaves them allocated, and the next MemInit then
     * gets a vreg mapping that does not describe the memory it is handed.
     *
     * The symptom is worth writing down because it names nothing useful:
     * MemInit and SetPubAttr both *succeed*, and HI_MPI_ISP_Init then fails
     * with 0xa01c8042 and the library prints "Isp[0] WDR mode doesn't
     * config!". SetPubAttr sets that flag through the vreg and ISP_Init
     * reads it back through the vreg; with the mapping wrong the write goes
     * nowhere and the read returns zero. Underneath, mmap on
     * /dev/mmz_userdev is failing with EINVAL for an offset of
     * 0x1000000100000 -- which is ISP_VREG_SIZE in the high word and
     * ISP_VREG_BASE in the low, i.e. not a physical address at all.
     *
     * Ignoring the result is deliberate and matches the SYS and VB calls
     * above it: on a clean boot there is nothing to exit and the call fails,
     * which is not a condition worth reporting.
     */
    if (st->isp.fnExit)
        st->isp.fnExit(HISI_VI_PIPE);

    ret = st->isp.fnMemInit(HISI_VI_PIPE);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_MemInit(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }

    /*
     * The window the ISP processes: all of it.
     *
     * (0,0), not the ini's DevRect_x/y, for the reason in
     * hisi_sensor_mode_load -- the offset describes a sensor array this
     * mode does not emit. stSnsSize is the same width and height, because
     * DevRect is what the sensor is configured to send rather than a crop
     * out of something larger.
     */
    memset(&pub, 0, sizeof(pub));
    pub.wnd_rect.x = 0;
    pub.wnd_rect.y = 0;
    pub.wnd_rect.width = m->dev_rect.width;
    pub.wnd_rect.height = m->dev_rect.height;
    pub.sns_size.width = m->dev_rect.width;
    pub.sns_size.height = m->dev_rect.height;
    pub.frame_rate = (float)m->frame_rate;
    pub.bayer = m->bayer;
    pub.wdr_mode = V4_WDR_MODE_NONE;
    pub.sns_mode = 0;

    ret = st->isp.fnSetPubAttr(HISI_VI_PIPE, &pub);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_SetPubAttr(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }

    ret = st->isp.fnInit(HISI_VI_PIPE);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_ISP_Init(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        return RSS_ERR_IO;
    }
    st->isp_inited = true;

    /* The flag goes up before the thread starts, so the thread's own exit
     * path can never observe it unset while the ISP is genuinely running. */
    __atomic_store_n(&st->isp_thread_running, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&st->isp_thread_done, 0, __ATOMIC_RELEASE);
    ret = pthread_create(&st->isp_thread, NULL, hisi_isp_thread, st);
    if (ret) {
        __atomic_store_n(&st->isp_thread_running, 0, __ATOMIC_RELEASE);
        HAL_LOG_ERR("isp: cannot start the 3A thread: %s", strerror(ret));
        return RSS_ERR_IO;
    }
    st->isp_thread_started = true;

    HAL_LOG_INFO("isp: pipe %d running, %ux%u+%d+%d @ %d fps, bayer %d", HISI_VI_PIPE,
                 m->dev_rect.width, m->dev_rect.height, m->dev_rect.x, m->dev_rect.y,
                 m->frame_rate, (int)m->bayer);
    return RSS_OK;
}

/* ================================================================
 * VPSS
 * ================================================================ */

/*
 * hisi_vpss_bringup -- the group, which is where 3DNR lives.
 *
 * One group per sensor. Its channels are created by the framesource ops as
 * rvd asks for streams; the group itself is a property of the pipeline and
 * belongs here.
 *
 * bNrEn is true and the attribute is VIDEO/NORMAL. That is the whole of
 * raptor's noise-reduction control on this family in Phase 2 -- it is a
 * *group* attribute, so it cannot be varied per stream, and the strength
 * curves live in the ISP tuning that Phase 3 reaches. Turning it off would
 * be the only other option and would cost image quality at every gain.
 *
 * enCompressMode NONE on the reference frame: SEG compression of the 3DNR
 * reference is available and saves DDR, but it also quantises the history
 * the temporal filter differences against. The vendor samples leave it off
 * for video NR and there is no measurement on this silicon to justify
 * otherwise.
 */
static int hisi_vpss_bringup(hisi_state_t *st)
{
    const hisi_sensor_mode_t *m = &st->mode;
    v4_vpss_grp_attr grp;
    int ret;

    memset(&grp, 0, sizeof(grp));
    grp.max_w = m->dev_rect.width;
    grp.max_h = m->dev_rect.height;
    grp.pixel_format = V4_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    grp.dynamic_range = V4_DYNAMIC_RANGE_SDR8;
    grp.frame_rate.src_frame_rate = -1;
    grp.frame_rate.dst_frame_rate = -1;
    grp.nr_en = 1;
    grp.nr_attr.nr_type = V4_VPSS_NR_TYPE_VIDEO;
    grp.nr_attr.compress_mode = V4_COMPRESS_MODE_NONE;
    grp.nr_attr.motion_mode = V4_NR_MOTION_MODE_NORMAL;

    ret = st->vpss.fnCreateGrp(HISI_VPSS_GRP, &grp);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_CreateGrp(%d) %ux%u failed: 0x%x", HISI_VPSS_GRP, grp.max_w,
                    grp.max_h, ret);
        return RSS_ERR_IO;
    }
    st->vpss_grp_created = true;

    /*
     * ── Start, then bind VI to VPSS ──
     *
     * Both unconditional, and both were briefly not.
     *
     * The vendor's own samples make the bind look mode-dependent:
     * sample_vio.c calls SAMPLE_COMM_VI_Bind_VPSS in every VPSS-*offline*
     * configuration and in none of the VPSS-online ones, and
     * sample_comm_vi.c gates HI_MPI_VI_EnableChn the same way. Following
     * that on this board produces a pipeline that builds perfectly and
     * carries no frames: /proc/umap/vi shows the pipe capturing (IntCnt
     * climbing, the ISP running, AE converging on real exposures) while
     * /proc/umap/vpss shows RecvPic0 stuck at zero and every channel's
     * output resolution still 0x0.
     *
     * divinus drives the same silicon with the same kernel default of
     * VI_OFFLINE_VPSS_ONLINE -- it never calls HI_MPI_SYS_SetVIVPSSMode at
     * all -- and binds VI(dev 0, chn 0) to VPSS(grp 0, chn 0) with no mode
     * test anywhere (src/hal/hisi/v4_hal.c:347). It also enables the VI
     * channel unconditionally. That is the configuration that moves frames
     * here, so it is the one this follows; the samples describe a mode
     * matrix this part does not appear to implement.
     *
     * StartGrp belongs here rather than deferred to the first channel for
     * the same reason: divinus starts the group immediately after creating
     * it and enables channels later, which is exactly raptor's shape, and
     * the group has to be running before the bind is made.
     */
    ret = st->vpss.fnStartGrp(HISI_VPSS_GRP);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VPSS_StartGrp(%d) failed: 0x%x", HISI_VPSS_GRP, ret);
        return RSS_ERR_IO;
    }
    st->vpss_grp_started = true;

    if (st->sys.fnBind) {
        v4_mpp_chn src, dst;

        memset(&src, 0, sizeof(src));
        src.module = V4_MOD_VI;
        src.device = HISI_VI_DEV;
        src.channel = HISI_VI_CHN;

        memset(&dst, 0, sizeof(dst));
        dst.module = V4_MOD_VPSS;
        dst.device = HISI_VPSS_GRP;
        dst.channel = 0;

        ret = st->sys.fnBind(&src, &dst);
        if (ret) {
            HAL_LOG_ERR("HI_MPI_SYS_Bind VI(%d,%d) -> VPSS(%d,0) failed: 0x%x", HISI_VI_DEV,
                        HISI_VI_CHN, HISI_VPSS_GRP, ret);
            return RSS_ERR_IO;
        }
        st->vi_vpss_bound = true;
    }

    HAL_LOG_INFO("vpss: group %d, %ux%u, 3DNR on", HISI_VPSS_GRP, grp.max_w, grp.max_h);
    return RSS_OK;
}

/* ================================================================
 * BIND
 * ================================================================ */

/*
 * VPSS channel to VENC channel.
 *
 * The one bind raptor makes at runtime. VI -> VPSS is made once during
 * bring-up and never changes; VPSS -> VENC changes whenever rvd
 * reconfigures a stream, which is why it is a function both hal_bind and
 * enc_register_channel reach rather than code in either.
 *
 * The device field is the VPSS *group* and the channel field is the VPSS
 * channel -- a group is a device to HiMPP's binding model. On the encoder
 * side the device is 0 and the channel is the VENC channel: VENC has no
 * device dimension on gen4.
 */
int hisi_bind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn)
{
    v4_mpp_chn src, dst;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (fs_chn < 0 || fs_chn >= HISI_FS_CHN_NUM || enc_chn < 0 || enc_chn >= HISI_VENC_CHN_NUM)
        return RSS_ERR_INVAL;
    if (!st->sys.fnBind)
        return RSS_ERR_NOTSUP;

    memset(&src, 0, sizeof(src));
    src.module = V4_MOD_VPSS;
    src.device = HISI_VPSS_GRP;
    src.channel = hisi_vpss_phy(fs_chn);

    memset(&dst, 0, sizeof(dst));
    dst.module = V4_MOD_VENC;
    dst.device = 0;
    dst.channel = enc_chn;

    ret = st->sys.fnBind(&src, &dst);

    /*
     * A bind that survived the last process.
     *
     * The kernel holds the bind table, so a VPSS -> VENC edge outlives the
     * rvd that made it: SIGKILL, a crash, or a `timeout` during bring-up all
     * leave VENC(enc_chn) still bound. The next start then gets NOT_PERM
     * from SYS_Bind and stays wedged until reboot, because nothing in
     * userspace remembers a bind it did not make. That is the same class of
     * leak the rest of this backend answers by tearing down first, and the
     * bind is the one piece of MPP state that was not doing it.
     *
     * HI_MPI_SYS_UnBind matches on the source as well as the destination --
     * the kernel compares all three fields of the recorded source against
     * the one passed in and refuses on any mismatch -- so clearing the
     * destination means naming the source that holds it, which is exactly
     * what is not known here. Sweeping the group's channels covers it: the
     * stale edge can only have come from this group, whether from an earlier
     * rvd or from the majestic that shipped on the board.
     *
     * Only on NOT_PERM, and it says so when it happens. A leaked bind is
     * worth a line in the log even though it is recoverable, because it is
     * the visible end of an unclean shutdown.
     */
    if (ret && V4_ERR_ID(ret) == V4_ERR_NOT_PERM && st->sys.fnUnbind) {
        int chn;

        HAL_LOG_WARN("VENC(%d) still bound from a previous process; clearing", enc_chn);

        for (chn = 0; chn < HISI_VPSS_CHN_NUM; chn++) {
            v4_mpp_chn stale = src;

            stale.channel = chn;
            if (st->sys.fnUnbind(&stale, &dst) == 0)
                break;
        }

        ret = st->sys.fnBind(&src, &dst);
    }

    if (ret) {
        HAL_LOG_ERR("HI_MPI_SYS_Bind VPSS(%d,%d) -> VENC(%d) failed: 0x%x", HISI_VPSS_GRP,
                    hisi_vpss_phy(fs_chn),
                    enc_chn, ret);
        return RSS_ERR_IO;
    }

    st->enc[enc_chn].bound_fs = fs_chn;
    HAL_LOG_DBG("bind: VPSS(%d,%d) -> VENC(%d)", HISI_VPSS_GRP, hisi_vpss_phy(fs_chn), enc_chn);
    return RSS_OK;
}

int hisi_unbind_vpss_venc(hisi_state_t *st, int fs_chn, int enc_chn)
{
    v4_mpp_chn src, dst;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;
    if (fs_chn < 0 || fs_chn >= HISI_FS_CHN_NUM || enc_chn < 0 || enc_chn >= HISI_VENC_CHN_NUM)
        return RSS_ERR_INVAL;
    if (!st->sys.fnUnbind)
        return RSS_ERR_NOTSUP;

    memset(&src, 0, sizeof(src));
    src.module = V4_MOD_VPSS;
    src.device = HISI_VPSS_GRP;
    src.channel = hisi_vpss_phy(fs_chn);

    memset(&dst, 0, sizeof(dst));
    dst.module = V4_MOD_VENC;
    dst.device = 0;
    dst.channel = enc_chn;

    ret = st->sys.fnUnbind(&src, &dst);
    if (ret)
        HAL_LOG_WARN("HI_MPI_SYS_UnBind VPSS(%d,%d) -> VENC(%d) failed: 0x%x", HISI_VPSS_GRP,
                     hisi_vpss_phy(fs_chn), enc_chn, ret);

    st->enc[enc_chn].bound_fs = -1;
    return RSS_OK;
}

/*
 * hisi_bind_collapse -- turn rvd's cell pair into (framesource, encoder).
 *
 * rvd expresses an overlaid stream as FS -> OSD -> ENC, two binds where
 * only the first names the framesource. HiMPP has no OSD stage in the
 * datapath: RGN regions attach to a VENC channel, so the pair collapses to
 * one FS -> VENC bind and the first half is recorded rather than acted on.
 * Same shape as the SigmaStar backend, and for the same reason.
 */
static int hisi_bind_collapse(hisi_state_t *st, const rss_cell_t *src, const rss_cell_t *dst,
                              int *fs_chn, int *enc_chn, bool *collapsed)
{
    *collapsed = false;

    if (!src || !dst)
        return RSS_ERR_INVAL;

    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_ENC) {
        *fs_chn = src->group;
        *enc_chn = dst->group;
        return RSS_OK;
    }

    if (src->device == RSS_DEV_FS && dst->device == RSS_DEV_OSD) {
        if (dst->group < 0 || dst->group >= HISI_VENC_CHN_NUM)
            return RSS_ERR_INVAL;
        st->osd_src_fs[dst->group] = src->group;
        *collapsed = true;
        return RSS_OK;
    }

    if (src->device == RSS_DEV_OSD && dst->device == RSS_DEV_ENC) {
        if (src->group < 0 || src->group >= HISI_VENC_CHN_NUM)
            return RSS_ERR_INVAL;
        *fs_chn = st->osd_src_fs[src->group];
        *enc_chn = dst->group;
        if (*fs_chn < 0) {
            HAL_LOG_ERR("bind: OSD %d -> ENC %d without a preceding FS -> OSD", src->group,
                        dst->group);
            return RSS_ERR_INVAL;
        }
        return RSS_OK;
    }

    HAL_LOG_ERR("bind: FS -> [OSD ->] ENC is the only chain this backend supports (got %d -> %d)",
                src->device, dst->device);
    return RSS_ERR_NOTSUP;
}

static int hal_bind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    hisi_state_t *st = hisi_state(ctx);
    bool collapsed;
    int fs_chn = -1;
    int enc_chn = -1;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = hisi_bind_collapse(st, src, dst, &fs_chn, &enc_chn, &collapsed);
    if (ret)
        return ret;
    if (collapsed)
        return RSS_OK; /* Recorded; the OSD -> ENC step does the work. */

    return hisi_bind_vpss_venc(st, fs_chn, enc_chn);
}

static int hal_unbind(void *ctx, const rss_cell_t *src, const rss_cell_t *dst)
{
    hisi_state_t *st = hisi_state(ctx);
    bool collapsed;
    int fs_chn = -1;
    int enc_chn = -1;
    int ret;

    if (!st)
        return RSS_ERR_INVAL;

    ret = hisi_bind_collapse(st, src, dst, &fs_chn, &enc_chn, &collapsed);
    if (ret)
        return ret;
    if (collapsed) {
        st->osd_src_fs[dst->group] = -1;
        return RSS_OK;
    }

    return hisi_unbind_vpss_venc(st, fs_chn, enc_chn);
}

/* ================================================================
 * VIDEO PIPELINE BRING-UP AND TEARDOWN
 * ================================================================ */

/*
 * hisi_video_bringup -- everything between SYS_Init and a running pipeline.
 *
 * Order, and each step's reason for being where it is:
 *
 *   1. the ISP libraries, because the sensor driver's own registration
 *      calls into them and they must be in the global scope first;
 *   2. VI/VPSS coupling, before any VI object exists to read it;
 *   3. MIPI, because the receiver latches its configuration out of reset;
 *   4. VI device, pipe and channel;
 *   5. the sensor driver and 3A registration, which needs the pipe;
 *   6. ISP init and its thread, which needs the sensor registered;
 *   7. VPSS group and the VI -> VPSS bind.
 *
 * The sensor mode INI is read earlier still, in hal_init, because VB's pool
 * size comes from it and VB precedes SYS_Init.
 *
 * Channels -- VPSS and VENC both -- are not created here. They are created
 * by the framesource and encoder ops as rvd asks for streams, which is what
 * lets a stream be reconfigured without rebuilding the pipeline.
 */
static int hisi_video_bringup(hisi_state_t *st, const rss_sensor_config_t *cfg)
{
    int ret;

    {
        hisi_isp_alg_regs algs = {
            .drc = &st->fn_alg_register_drc,
            .dehaze = &st->fn_alg_register_dehaze,
            .ldci = &st->fn_alg_register_ldci,
        };

        ret = hisi_isp_open(&st->isp, &st->libs, &algs);
        if (ret)
            return ret;
    }


    ret = hisi_mipi_configure(st);
    if (ret)
        return ret;

    ret = hisi_vi_bringup(st);
    if (ret)
        return ret;

    ret = hisi_sensor_bringup(st, cfg);
    if (ret)
        return ret;

    ret = hisi_isp_bringup(st);
    if (ret)
        return ret;

    return hisi_vpss_bringup(st);
}

/*
 * hisi_video_teardown -- the exact reverse, driven by the unwind flags.
 *
 * Teardown ordering is where a bad HiMPP bring-up leaves a board needing a
 * power cycle, so this is the part to be strict about. Every step is
 * guarded by the flag its bring-up set, failures are logged and never
 * propagated, and the sequence is the reverse of hisi_video_bringup with no
 * exceptions.
 *
 * The ISP thread is the one step that is not a mirror image. It is stopped
 * by calling HI_MPI_ISP_Exit -- which makes HI_MPI_ISP_Run return -- and
 * then joined. Cancelling it instead would leave the vendor library's locks
 * held and deadlock the next bring-up in the same process.
 */
/*
 * hisi_reclaim_pipeline -- tear down whatever the *last* process left running.
 *
 * hal_init already exited SYS and VB before configuring its own, on the
 * grounds that MPP state lives in the kernel and outlives the process that
 * made it. That was right and did not go far enough: HI_MPI_VB_Exit fails
 * while anything still holds a block, so a daemon that was killed mid-
 * pipeline leaves VI enabled, the VPSS group started and VENC channels
 * created, and the VB pool survives with them. /proc/umap/vb shows it
 * plainly -- the pool is still listed, four blocks free of four, Owner -1,
 * with no process anywhere that owns it.
 *
 * The next start then configures VB against a live one, gets a failure it
 * has no way to interpret, and carries on into an ISP bring-up that faults
 * inside the vendor library. Nothing in the log names the pool.
 *
 * So the reclaim is the pipeline in the same reverse order hisi_video_teardown
 * uses, with none of its flags: those record what *this* process built, and
 * this runs before this process has built anything. Every call here is
 * expected to fail on a clean boot and none is checked, which is the same
 * contract the SYS_Exit and VB_Exit above it have always had.
 *
 * The unbind sweep is wider than the binds raptor makes because the previous
 * owner need not have been raptor -- majestic ships on these boards and binds
 * the same modules -- and HI_MPI_SYS_UnBind matches on the source, so
 * clearing an edge means naming both of its ends.
 *
 * The ISP is not here. It is torn down at the top of hisi_isp_bringup, which
 * is where the library that owns it is first loaded; by then VI is already
 * down, which is the order that matters.
 */
static void hisi_reclaim_pipeline(hisi_state_t *st)
{
    int chn, src;

    for (chn = 0; chn < HISI_VENC_CHN_NUM; chn++) {
        v4_mpp_chn dst;

        if (st->venc.fnStopRecvFrame)
            st->venc.fnStopRecvFrame(chn);

        memset(&dst, 0, sizeof(dst));
        dst.module = V4_MOD_VENC;
        dst.device = 0;
        dst.channel = chn;

        for (src = 0; src < HISI_VPSS_CHN_NUM && st->sys.fnUnbind; src++) {
            v4_mpp_chn from;

            memset(&from, 0, sizeof(from));
            from.module = V4_MOD_VPSS;
            from.device = HISI_VPSS_GRP;
            from.channel = src;

            st->sys.fnUnbind(&from, &dst);
        }

        if (st->venc.fnDestroyChn)
            st->venc.fnDestroyChn(chn);
    }

    for (chn = 0; chn < HISI_VPSS_CHN_NUM; chn++)
        if (st->vpss.fnDisableChn)
            st->vpss.fnDisableChn(HISI_VPSS_GRP, hisi_vpss_phy(chn));

    if (st->vpss.fnStopGrp)
        st->vpss.fnStopGrp(HISI_VPSS_GRP);
    if (st->vpss.fnDestroyGrp)
        st->vpss.fnDestroyGrp(HISI_VPSS_GRP);

    if (st->vi.fnDisableChn)
        st->vi.fnDisableChn(HISI_VI_PIPE, HISI_VI_CHN);
    if (st->vi.fnStopPipe)
        st->vi.fnStopPipe(HISI_VI_PIPE);
    if (st->vi.fnDestroyPipe)
        st->vi.fnDestroyPipe(HISI_VI_PIPE);
    if (st->vi.fnDisableDev)
        st->vi.fnDisableDev(HISI_VI_DEV);
}

static void hisi_video_teardown(hisi_state_t *st)
{
    int ret;

    /* Encoders first: they are downstream of the VPSS channels, and a VENC
     * channel left bound to a channel that is about to be disabled is what
     * strands buffers in the kernel. */
    hisi_enc_release_all(st);
    hisi_fs_release_all(st);

    /* Only ever set in an offline mode; nothing to undo otherwise. */
    if (st->vi_vpss_bound && st->sys.fnUnbind) {
        v4_mpp_chn src, dst;

        memset(&src, 0, sizeof(src));
        src.module = V4_MOD_VI;
        src.device = HISI_VI_PIPE;
        src.channel = HISI_VI_CHN;

        memset(&dst, 0, sizeof(dst));
        dst.module = V4_MOD_VPSS;
        dst.device = HISI_VPSS_GRP;
        dst.channel = 0;

        ret = st->sys.fnUnbind(&src, &dst);
        if (ret)
            HAL_LOG_WARN("HI_MPI_SYS_UnBind VI -> VPSS failed: 0x%x", ret);
        st->vi_vpss_bound = false;
    }

    if (st->vpss_grp_started && st->vpss.fnStopGrp) {
        ret = st->vpss.fnStopGrp(HISI_VPSS_GRP);
        if (ret)
            HAL_LOG_WARN("HI_MPI_VPSS_StopGrp(%d) failed: 0x%x", HISI_VPSS_GRP, ret);
        st->vpss_grp_started = false;
    }

    if (st->vpss_grp_created && st->vpss.fnDestroyGrp) {
        ret = st->vpss.fnDestroyGrp(HISI_VPSS_GRP);
        if (ret)
            HAL_LOG_WARN("HI_MPI_VPSS_DestroyGrp(%d) failed: 0x%x", HISI_VPSS_GRP, ret);
        st->vpss_grp_created = false;
    }

    if (st->isp_inited && st->isp.fnExit) {
        /* Clear the flag first so the thread's own exit path reads this as
         * an intentional stop rather than as the pipeline dying. */
        __atomic_store_n(&st->isp_thread_running, 0, __ATOMIC_RELEASE);

        ret = st->isp.fnExit(HISI_VI_PIPE);
        if (ret)
            HAL_LOG_WARN("HI_MPI_ISP_Exit(pipe %d) failed: 0x%x", HISI_VI_PIPE, ret);
        st->isp_inited = false;

        if (st->isp_thread_started)
            hisi_isp_thread_stop(st);
        st->isp_thread_started = false;
    }

    if (st->awb_registered && st->isp.fnAwbUnRegister) {
        v4_alg_lib lib;

        memset(&lib, 0, sizeof(lib));
        lib.id = HISI_VI_PIPE;
        snprintf(lib.lib_name, sizeof(lib.lib_name), "%s", st->isp.awb_lib_name);
        st->isp.fnAwbUnRegister(HISI_VI_PIPE, &lib);
    }
    st->awb_registered = false;

    if (st->ae_registered && st->isp.fnAeUnRegister) {
        v4_alg_lib lib;

        memset(&lib, 0, sizeof(lib));
        lib.id = HISI_VI_PIPE;
        snprintf(lib.lib_name, sizeof(lib.lib_name), "%s", st->isp.ae_lib_name);
        st->isp.fnAeUnRegister(HISI_VI_PIPE, &lib);
    }
    st->ae_registered = false;

    if (st->sensor_registered && st->snr.obj && st->snr.obj->pfnUnRegisterCallback) {
        v4_alg_lib ae_lib, awb_lib;

        memset(&ae_lib, 0, sizeof(ae_lib));
        memset(&awb_lib, 0, sizeof(awb_lib));
        ae_lib.id = HISI_VI_PIPE;
        awb_lib.id = HISI_VI_PIPE;
        snprintf(ae_lib.lib_name, sizeof(ae_lib.lib_name), "%s", st->isp.ae_lib_name);
        snprintf(awb_lib.lib_name, sizeof(awb_lib.lib_name), "%s", st->isp.awb_lib_name);
        st->snr.obj->pfnUnRegisterCallback(HISI_VI_PIPE, &ae_lib, &awb_lib);
    }
    st->sensor_registered = false;

    if (st->vi_chn_enabled && st->vi.fnDisableChn) {
        ret = st->vi.fnDisableChn(HISI_VI_PIPE, HISI_VI_CHN);
        if (ret)
            HAL_LOG_WARN("HI_MPI_VI_DisableChn failed: 0x%x", ret);
        st->vi_chn_enabled = false;
    }

    if (st->vi_pipe_started && st->vi.fnStopPipe) {
        ret = st->vi.fnStopPipe(HISI_VI_PIPE);
        if (ret)
            HAL_LOG_WARN("HI_MPI_VI_StopPipe failed: 0x%x", ret);
        st->vi_pipe_started = false;
    }

    if (st->vi_pipe_created && st->vi.fnDestroyPipe) {
        ret = st->vi.fnDestroyPipe(HISI_VI_PIPE);
        if (ret)
            HAL_LOG_WARN("HI_MPI_VI_DestroyPipe failed: 0x%x", ret);
        st->vi_pipe_created = false;
    }

    if (st->vi_dev_enabled && st->vi.fnDisableDev) {
        ret = st->vi.fnDisableDev(HISI_VI_DEV);
        if (ret)
            HAL_LOG_WARN("HI_MPI_VI_DisableDev failed: 0x%x", ret);
        st->vi_dev_enabled = false;
    }

    hisi_mipi_shutdown(st);

    v4_snr_unload(&st->snr);
    hisi_isp_close(&st->isp);
}

#endif /* HAL_MODULE_VIDEO */

/* ================================================================
 * SYSTEM LIFECYCLE
 * ================================================================ */

/*
 * COMMON_GetPicBufferSize, transcribed for the one case this backend needs:
 * NV12, 8-bit, uncompressed.
 *
 * From the SDK's own inline helper (mpp/include/hi_buffer.h:179 and the
 * COMMON_GetPicBufferConfig above it), reduced to the branch that applies:
 *
 *   align  = DEFAULT_ALIGN (8), which is what every vendor sample passes
 *   stride = ALIGN_UP(width * 8 / 8, align)
 *   height = ALIGN_UP(height, 2)
 *   size   = stride * height * 3 / 2
 *
 * Transcribed rather than approximated because an undersized VB block is
 * one of the two classic gen4 bring-up failures -- SYS_Init succeeds, the
 * pipeline builds, and VI silently delivers nothing.
 */
#define HISI_VB_ALIGN 8u

#ifdef HAL_MODULE_VIDEO
static unsigned int hisi_vb_nv12_size(unsigned int width, unsigned int height)
{
    unsigned int stride = ((width + HISI_VB_ALIGN - 1u) / HISI_VB_ALIGN) * HISI_VB_ALIGN;
    unsigned int rows = (height + 1u) & ~1u;

    return stride * rows * 3u / 2u;
}
#endif /* HAL_MODULE_VIDEO */

/*
 * hisi_vb_bringup -- configure and start the video buffer pools.
 *
 * Two common pools, and the split is the vendor's.
 *
 * Pool 0 holds sensor-sized frames: the raw the VI pipe writes, the NV12
 * the VI channel writes, and the VPSS group's 3DNR reference. On this part
 * the raw and the picture come to the same number of bytes --
 * VI_GetRawBufferSize(2592, 1944, RAW12) and COMMON_GetPicBufferSize(2592,
 * 1944, NV12) are both 7558272 -- so one pool serves both, where
 * sample_vio.c needs two.
 *
 * Pool 1 holds what the VPSS channels put out, which is smaller than a
 * sensor frame in every configuration that scales at all. VB hands out a
 * block from the smallest pool whose blocks fit, so a 1920x1080 channel
 * takes 3 MiB from pool 1 rather than 7.2 MiB from pool 0, and a channel
 * larger than pool 1's blocks quietly falls back to pool 0. That makes the
 * second pool an optimisation with a safe failure mode rather than a bound
 * that has to be right.
 *
 * Both halves were forced by the board. With a single pool of four
 * sensor-sized blocks the pipeline deadlocks: /proc/umap/vb shows Free 0
 * and MinFree 0 with all four blocks owned by VI, while /proc/umap/vpss
 * counts pictures arriving and none leaving. Nothing logs an error -- VI's
 * VbFail stays zero, because VI is not the module being refused. Six is
 * enough for VI and the group; the channel outputs need their own.
 *
 * The counts cannot simply keep rising, and the reason is in
 * /proc/cmdline: this board runs mmz_allocator=cma, so a pool is one
 * contiguous CMA allocation. Six sensor blocks (45 MiB) succeed and seven
 * (53 MiB) fail with HI_MPI_VB_Init returning 0xa001800c, VB / NOMEM, on a
 * 96 MiB zone reporting 95 MiB free. Splitting the memory across two
 * smaller pools is therefore not only tighter, it is likelier to be
 * satisfiable at all.
 */
#define HISI_VB_BLK_CNT 6u

/*
 * Pool 1's geometry: the sensor's, capped at 1920x1080.
 *
 * hal_init is not told the stream configuration -- rss_multi_sensor_config_t
 * carries sensors, not streams -- so the size cannot be derived from what
 * rvd is about to ask for. The cap is a statement about the common case
 * rather than a limit: a stream above it falls back to pool 0 and still
 * works, and a sensor below it makes the two pools the same size, which
 * costs nothing.
 */
#define HISI_VB_STREAM_MAX_W 1920u
#define HISI_VB_STREAM_MAX_H 1080u
#define HISI_VB_STREAM_BLK_CNT 4u

static int hisi_vb_bringup(hisi_state_t *st)
{
    v4_vb_conf conf;
    int ret;

    memset(&conf, 0, sizeof(conf));

#ifdef HAL_MODULE_VIDEO
    if (st->mode.dev_rect.width && st->mode.dev_rect.height) {
        unsigned int sw = st->mode.dev_rect.width;
        unsigned int sh = st->mode.dev_rect.height;
        unsigned int cw = sw < HISI_VB_STREAM_MAX_W ? sw : HISI_VB_STREAM_MAX_W;
        unsigned int ch = sh < HISI_VB_STREAM_MAX_H ? sh : HISI_VB_STREAM_MAX_H;

        conf.max_pool_cnt = 2;

        conf.pool[0].blk_size = hisi_vb_nv12_size(sw, sh);
        conf.pool[0].blk_cnt = HISI_VB_BLK_CNT;
        /* NOCACHE: nothing in userspace reads these blocks on the streaming
         * path, and a cached mapping would need explicit maintenance at
         * every hand-off between VI, VPSS and VENC. */
        conf.pool[0].remap_mode = V4_VB_REMAP_NOCACHE;

        conf.pool[1].blk_size = hisi_vb_nv12_size(cw, ch);
        conf.pool[1].blk_cnt = HISI_VB_STREAM_BLK_CNT;
        conf.pool[1].remap_mode = V4_VB_REMAP_NOCACHE;
    }
#endif

    /*
     * The audio archive has no sensor and no pipeline, so it configures no
     * pools -- and that is the one case where an empty table is right.
     */
    if (conf.max_pool_cnt)
        HAL_LOG_INFO("vb: %u pools, %llu x %u + %llu x %u = %llu KiB", conf.max_pool_cnt,
                     conf.pool[0].blk_size, conf.pool[0].blk_cnt, conf.pool[1].blk_size,
                     conf.pool[1].blk_cnt,
                     (conf.pool[0].blk_size * conf.pool[0].blk_cnt +
                      conf.pool[1].blk_size * conf.pool[1].blk_cnt) /
                         1024ull);
    else
        HAL_LOG_INFO("vb: no common pools (no sensor geometry -- audio-only build)");

    ret = st->sys.fnVbSetConfig(&conf);
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VB_SetConfig failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    st->vb_configured = true;

    ret = st->sys.fnVbInit();
    if (ret) {
        HAL_LOG_ERR("HI_MPI_VB_Init failed: 0x%x", ret);
        return RSS_ERR_IO;
    }
    st->vb_inited = true;

    return RSS_OK;
}

static int hisi_teardown(hisi_state_t *st);

/*
 * hal_init -- bring MPP up as far as SYS.
 *
 * The Ingenic equivalent runs IMP_ISP_Open -> AddSensor -> EnableSensor ->
 * IMP_System_Init; the MI one runs dlopen -> MI_SYS_Init -> MI_SNR_* ->
 * MI_VIF_*. HiMPP's is
 *
 *   dlopen -> chip id -> SYS_Exit + VB_Exit -> VB_SetConfig -> VB_Init
 *          -> SYS_Init
 *
 * with VI, VPSS and VENC added in Phase 2.
 *
 * The teardown-first step is not defensive tidying, it is the documented
 * gen4 sequence and divinus does the same (v4_hal.c:990-991). MPP state
 * lives in the kernel modules, not in the process: a previous consumer that
 * was killed rather than closed leaves VB pools allocated and SYS
 * initialised, and VB_SetConfig on a live VB returns busy. Exiting first
 * makes bring-up idempotent across a crash, which on a camera is the normal
 * case rather than the exceptional one. Both calls are expected to fail on
 * a clean boot; neither result is checked, which is why they are the only
 * unchecked MPI calls in this file.
 */
static int hal_init(void *ctx, const rss_multi_sensor_config_t *cfg)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st;
    int ret;

    if (!c || !cfg || cfg->sensor_count < 1 || cfg->sensor_count > RSS_MAX_SENSORS)
        return RSS_ERR_INVAL;

    if (c->initialized) {
        HAL_LOG_ERR("hal_init: already initialized");
        return RSS_ERR_BUSY;
    }

    if (cfg->sensor_count > 1)
        HAL_LOG_WARN("hal_init: %d sensors requested, gen4 backend drives 1", cfg->sensor_count);

    memcpy(&c->multi_cfg, cfg, sizeof(c->multi_cfg));
    c->sensor_count = 1;
    memcpy(&c->sensors[0], &cfg->sensors[0], sizeof(c->sensors[0]));

    st = (hisi_state_t *)calloc(1, sizeof(*st));
    if (!st)
        return RSS_ERR_NOMEM;
    c->platform = st;

    snprintf(st->sensor_name, sizeof(st->sensor_name), "%s", cfg->sensors[0].name);

    /*
     * -1, not the 0 calloc left behind. Framesource 0, encoder channel 0 and
     * file descriptor 0 are all real values, so "nothing here yet" needs a
     * value of its own before anything closes or unbinds one.
     */
    {
        int i;

        for (i = 0; i < HISI_VENC_CHN_NUM; i++) {
            st->enc[i].bound_fs = -1;
            st->enc[i].fd = -1;
            st->osd_src_fs[i] = -1;
        }
    }

    /*
     * Before the first vendor dlopen, for the eager-binding reason in the
     * TRAMPOLINES block. Nothing below may be reordered above this line.
     */
    hisi_check_trampolines();

    ret = hisi_mpi_open(&st->libs);
    if (ret)
        goto err_free;

    ret = v4_sys_load(&st->sys, &st->libs);
    if (ret)
        goto err_unload;

#ifdef HAL_MODULE_VIDEO
    /* VI, VPSS and VENC are video-module dependencies: the audio archive
     * has no pipeline ops, so it has no reason to resolve them. */
    ret = v4_vi_load(&st->vi, &st->libs);
    if (ret)
        goto err_unload;
    ret = v4_vpss_load(&st->vpss, &st->libs);
    if (ret)
        goto err_unload;
    ret = v4_venc_load(&st->venc, &st->libs);
    if (ret)
        goto err_unload;
#endif

    hisi_read_chip_id(st);

    if (st->sys.fnGetVersion) {
        v4_sys_ver ver;

        memset(&ver, 0, sizeof(ver));
        if (!st->sys.fnGetVersion(&ver))
            snprintf(st->mpp_version, sizeof(st->mpp_version), "%.*s",
                     (int)sizeof(ver.version), ver.version);
    }
    HAL_LOG_INFO("mpp: %s", st->mpp_version[0] ? st->mpp_version : "version unavailable");

    /* Teardown-first; see the function comment. The pipeline has to go
     * before SYS and VB, because a module still holding a block is what
     * makes HI_MPI_VB_Exit fail. */
#ifdef HAL_MODULE_VIDEO
    hisi_reclaim_pipeline(st);
#endif
    st->sys.fnExit();
    st->sys.fnVbExit();

#ifdef HAL_MODULE_VIDEO
    /*
     * The sensor mode, before VB and not with the rest of the video
     * bring-up, because VB's pool size comes out of it and VB has to be
     * configured before HI_MPI_SYS_Init. Failing here costs nothing: no
     * MPP state has been created yet.
     */
    ret = hisi_sensor_mode_load(&st->mode, cfg->sensors[0].name);
    if (ret)
        goto err_unload;
#endif

#ifdef HAL_MODULE_VIDEO
    /* Before VB and before SYS_Init; see hisi_vi_vpss_mode. */
    hisi_vi_vpss_mode(st);
#endif

    ret = hisi_vb_bringup(st);
    if (ret)
        goto err_teardown;

    ret = st->sys.fnInit();
    if (ret) {
        HAL_LOG_ERR("HI_MPI_SYS_Init failed: 0x%x", ret);
        ret = RSS_ERR_IO;
        goto err_teardown;
    }
    st->sys_inited = true;

    /*
     * g_hisi before the video bring-up, not after: the sensor driver's
     * pfnRegisterCallback runs inside it and calls straight back into the
     * GK_API_* forwarders, which reach the ISP through this pointer. Set it
     * afterwards and eight of the 34 shipped drivers fail to register with
     * a message about no ISP being loaded.
     */
    g_hisi = st;

#ifdef HAL_MODULE_VIDEO
    ret = hisi_video_bringup(st, &c->sensors[0]);
    if (ret)
        goto err_teardown;
#endif

    c->initialized = true;
    return RSS_OK;

err_teardown:
    hisi_teardown(st);
    /* After the teardown, not before: hisi_video_teardown calls the sensor
     * driver's pfnUnRegisterCallback, which reaches the GK_API_* forwarders
     * through this pointer. Clearing it first would make every unregister
     * decline, leaving the ISP's tables pointing into a library that is
     * about to be dlclosed. */
    if (g_hisi == st)
        g_hisi = NULL;
err_unload:
#ifdef HAL_MODULE_VIDEO
    v4_venc_unload(&st->venc);
    v4_vpss_unload(&st->vpss);
    v4_vi_unload(&st->vi);
#endif
    v4_sys_unload(&st->sys);
    hisi_mpi_close(&st->libs);
err_free:
    free(st);
    c->platform = NULL;
    return ret;
}

/*
 * hisi_teardown -- undo whatever bring-up actually completed.
 *
 * Driven by the unwind flags rather than by assuming a fully-built
 * pipeline, so a failure partway through hal_init does not exit blocks that
 * were never entered. Return codes are logged and never propagated:
 * teardown has no recovery, and a first failure must not skip the remaining
 * steps -- which on HiMPP is the difference between a clean restart and a
 * board that needs its power cycled.
 *
 * Exact reverse of hal_init: SYS before VB. VB_Exit with SYS still up
 * pulls the pools out from under the modules holding blocks in them.
 */
static int hisi_teardown(hisi_state_t *st)
{
    int ret;

    if (!st)
        return RSS_OK;

#ifdef HAL_MODULE_VIDEO
    /* The whole video pipeline, in the exact reverse of its bring-up,
     * before SYS and VB go. See hisi_video_teardown. */
    hisi_video_teardown(st);
#endif

    if (st->sys_inited) {
        ret = st->sys.fnExit();
        if (ret)
            HAL_LOG_WARN("HI_MPI_SYS_Exit failed: 0x%x", ret);
        st->sys_inited = false;
    }

    if (st->vb_inited) {
        ret = st->sys.fnVbExit();
        if (ret)
            HAL_LOG_WARN("HI_MPI_VB_Exit failed: 0x%x", ret);
        st->vb_inited = false;
    }

    /* VB_SetConfig has no undo of its own -- VB_Exit is what discards the
     * table -- so the flag exists to record that a configuration was
     * applied, not to drive a call. Cleared for the same reason the others
     * are: a re-init must not believe it. */
    st->vb_configured = false;

    return RSS_OK;
}

static int hal_deinit(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;
    hisi_state_t *st = hisi_state(ctx);

    if (!c)
        return RSS_ERR_INVAL;

    if (!st)
        return RSS_OK;

    hisi_teardown(st);

    /*
     * Cleared after the teardown, not before.
     *
     * The forwarders are entered from the vendor libraries, and the last
     * thing that legitimately does so is the sensor driver's
     * pfnUnRegisterCallback, which hisi_video_teardown calls. Clearing this
     * first would make that unregister decline and leave the ISP holding
     * callbacks into a library about to be dlclosed -- which is fine until
     * something re-inits in the same process.
     *
     * By the time control gets here the ISP thread has been joined and the
     * sensor library unloaded, so there is nothing left to call in.
     */
    if (g_hisi == st)
        g_hisi = NULL;

#ifdef HAL_MODULE_VIDEO
    v4_venc_unload(&st->venc);
    v4_vpss_unload(&st->vpss);
    v4_vi_unload(&st->vi);
#endif
    v4_sys_unload(&st->sys);
    hisi_mpi_close(&st->libs);

    free(st);
    c->platform = NULL;
    c->initialized = false;

    return RSS_OK;
}

/*
 * hal_get_caps -- return the per-SoC capability struct.
 *
 * The context's copy, not g_hal_caps directly: rss_hal_create_backend
 * adjusts the backend-surface flags on it.
 */
static const rss_hal_caps_t *hal_get_caps(void *ctx)
{
    rss_hal_ctx_t *c = (rss_hal_ctx_t *)ctx;

    return c ? &c->caps : &g_hal_caps;
}

/* ================================================================
 * SYSTEM UTILITIES
 * ================================================================ */

static int hal_sys_get_version(void *ctx, char *buf, int len)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->mpp_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->mpp_version);
    return RSS_OK;
}

static int hal_sys_get_cpu_info(void *ctx, char *buf, int len)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!buf || len <= 0)
        return RSS_ERR_INVAL;
    if (!st || !st->chip_name[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)len, "%s", st->chip_name);
    return RSS_OK;
}

/*
 * Media clock. rvd_frame_loop.c uses these to publish the
 * media-clock-to-UTC mapping SEI timecodes are derived from; without them
 * the mapping early-returns, frames still flow, and timecodes silently
 * vanish.
 */
static int hal_sys_get_timestamp(void *ctx, int64_t *ts)
{
    hisi_state_t *st = hisi_state(ctx);
    unsigned long long pts = 0;

    if (!ts)
        return RSS_ERR_INVAL;
    if (!st || !st->sys.fnGetCurPts)
        return RSS_ERR_NOTSUP;

    if (st->sys.fnGetCurPts(&pts))
        return RSS_ERR_IO;

    *ts = (int64_t)pts;
    return RSS_OK;
}

static int hal_sys_rebase_timestamp(void *ctx, int64_t base)
{
    hisi_state_t *st = hisi_state(ctx);

    if (!st || !st->sys.fnInitPtsBase)
        return RSS_ERR_NOTSUP;

    if (st->sys.fnInitPtsBase((unsigned long long)base))
        return RSS_ERR_IO;

    return RSS_OK;
}

/* ================================================================
 * OPS VTABLE
 *
 * Only implemented ops are listed. Everything else stays NULL and resolves
 * to RSS_ERR_NOTSUP through RSS_HAL_CALL -- which is what makes a Phase 1
 * build useful rather than merely compilable: rvd starts, prints its
 * banner, finds no framesource and exits cleanly.
 *
 * The video pipeline (fs_*, enc_*, isp_*, osd_*) lands in Phase 2, audio in
 * Phase 4, OSD in Phase 5.
 * ================================================================ */

static const rss_hal_ops_t g_ops = {
    /* System lifecycle */
    .init = hal_init,
    .deinit = hal_deinit,
    .get_caps = hal_get_caps,

    /* System utilities */
    .sys_get_version = hal_sys_get_version,
    .sys_get_cpu_info = hal_sys_get_cpu_info,
    .sys_get_timestamp = hal_sys_get_timestamp,
    .sys_rebase_timestamp = hal_sys_rebase_timestamp,

#ifdef HAL_MODULE_VIDEO
    /* /dev/mem, not MPP: HiMPP publishes no register accessor. Video-only
     * because the audio archive has no use for one and no reason to carry
     * the mmap code. */
    .sys_read_reg32 = hal_sys_read_reg32,
    .sys_write_reg32 = hal_sys_write_reg32,

    /* Pipeline topology. FS -> [OSD ->] ENC, collapsed in
     * hisi_bind_collapse because HiMPP has no OSD stage in the datapath. */
    .bind = hal_bind,
    .unbind = hal_unbind,

    /* Framesource == VPSS channel (src/hisi_v4/hal_framesource.c). The
     * OP COVERAGE block at the top of that file argues each absence. */
    .fs_create_channel = hal_fs_create_channel,
    .fs_set_channel_attr = hal_fs_set_channel_attr,
    .fs_destroy_channel = hal_fs_destroy_channel,
    .fs_enable_channel = hal_fs_enable_channel,
    .fs_disable_channel = hal_fs_disable_channel,
    .fs_get_frame = hal_fs_get_frame,
    .fs_release_frame = hal_fs_release_frame,
    .fs_set_frame_depth = hal_fs_set_frame_depth,
    .fs_get_frame_depth = hal_fs_get_frame_depth,

    /* Encoder == VENC channel (src/hisi_v4/hal_encoder.c). The group ops
     * are bookkeeping: HiMPP has no encoder group, and rvd calls them
     * unconditionally. */
    .enc_create_group = hal_enc_create_group,
    .enc_destroy_group = hal_enc_destroy_group,
    .enc_create_channel = hal_enc_create_channel,
    .enc_destroy_channel = hal_enc_destroy_channel,
    .enc_register_channel = hal_enc_register_channel,
    .enc_unregister_channel = hal_enc_unregister_channel,
    .enc_start = hal_enc_start,
    .enc_stop = hal_enc_stop,
    .enc_poll = hal_enc_poll,
    .enc_get_frame = hal_enc_get_frame,
    .enc_release_frame = hal_enc_release_frame,
    .enc_request_idr = hal_enc_request_idr,
    .enc_set_rc_mode = hal_enc_set_rc_mode,
    .enc_set_bitrate = hal_enc_set_bitrate,
    .enc_set_gop = hal_enc_set_gop,
    .enc_set_fps = hal_enc_set_fps,
    .enc_get_channel_attr = hal_enc_get_channel_attr,
    .enc_get_fps = hal_enc_get_fps,
    .enc_get_avg_bitrate = hal_enc_get_avg_bitrate,
    .enc_query = hal_enc_query,
    .enc_get_fd = hal_enc_get_fd,

    /*
     * ISP. Phase 2 publishes the sensor-geometry accessor and nothing else:
     * the tuning ops are Phase 3, and a knob published before its mapping
     * is argued is a knob that quietly costs the tuning's own per-gain
     * curve. Every one of them answers RSS_ERR_NOTSUP until then, which rvd
     * reports as unsettable and rcd hides.
     */
    .isp_get_sensor_attr = hal_isp_get_sensor_attr,

    /* GPIO / IR-cut — vendor-neutral sysfs, works as-is */
    .gpio_set = hal_gpio_set,
    .gpio_get = hal_gpio_get,
    .ircut_set = hal_ircut_set,
#endif
};

/* ================================================================
 * FACTORY FUNCTIONS
 * ================================================================ */

rss_hal_ctx_t *rss_hal_create(void)
{
    return rss_hal_create_backend("imp");
}

/*
 * rss_hal_create_backend -- pick a pipeline backend by name.
 *
 * This build carries one, and it is the vendor's: HiMPP is how a gen4 part
 * talks to its ISP and encoder at all. "imp" is the name the config's
 * default carries on every platform, so it means "the built-in one" here
 * rather than Ingenic's library. Any other name -- v4l2, composed on the
 * Ingenic side out of parts with no counterpart here -- gets NULL, because
 * a caller that asked for a different pipeline is better told it is missing
 * than handed this one under its name.
 *
 * The backend-surface flags stay false in Phase 1 and are set as each
 * subsystem lands. That is load-bearing rather than cosmetic: a context
 * with has_framesource false gets no framesource created, so rvd starts,
 * reports what it cannot do, and exits cleanly instead of failing partway
 * into a pipeline that does not exist yet.
 */
rss_hal_ctx_t *rss_hal_create_backend(const char *backend)
{
    rss_hal_ctx_t *ctx;

    if (backend && strcmp(backend, "imp") != 0)
        return NULL;

    ctx = (rss_hal_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx)
        return NULL;

    ctx->ops = &g_ops;
    memcpy(&ctx->caps, &g_hal_caps, sizeof(ctx->caps));

#ifdef HAL_MODULE_VIDEO
    /*
     * Backend-surface flags, set here rather than in the per-SoC caps table
     * because they describe the *pipeline* rather than the part: this one
     * has a framesource graph and a JPEG encoder, and no IVS at all.
     *
     * has_osd stays false until Phase 5. It is load-bearing rather than
     * cosmetic: a context that leaves it false gets no regions created, so
     * rvd reports overlays as unavailable instead of configuring an OSD
     * stage whose ops all answer NOTSUP.
     */
    ctx->caps.has_framesource = true;
    ctx->caps.has_jpeg = true;
#endif

    return ctx;
}

/*
 * rss_hal_destroy -- free a HAL context and internal resources.
 *
 * Does NOT call deinit() -- the caller must do that first.
 */
void rss_hal_destroy(rss_hal_ctx_t *ctx)
{
    int i;

    if (!ctx)
        return;

    for (i = 0; i < RSS_MAX_ENC_CHANNELS; i++) {
        free(ctx->scratch_buf[i]);
        ctx->scratch_buf[i] = NULL;
        free(ctx->nal_arrays[i]);
        ctx->nal_arrays[i] = NULL;
    }

    free(ctx);
}

const rss_hal_ops_t *rss_hal_get_ops(rss_hal_ctx_t *ctx)
{
    if (!ctx)
        return NULL;

    return ctx->ops;
}

/* ================================================================
 * SYSTEM INFO (no vtable, called directly)
 * ================================================================ */

/*
 * rss_hal_get_imp_version / rss_hal_get_sysutils_version
 *
 * Both names are IMP-specific, but the daemons call them unconditionally to
 * print a build banner and pass no context. HiMPP's equivalent is
 * HI_MPI_SYS_GetVersion, reached through g_hisi, so this answers only after
 * hal_init -- before that there is no loaded library to ask. There is no
 * sysutils equivalent at all, so that one is permanently unsupported.
 */
int rss_hal_get_imp_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    if (!g_hisi || !g_hisi->mpp_version[0])
        return RSS_ERR_NOTSUP;

    snprintf(buf, (size_t)size, "%s", g_hisi->mpp_version);
    return RSS_OK;
}

int rss_hal_get_sysutils_version(char *buf, int size)
{
    if (!buf || size <= 0)
        return RSS_ERR_INVAL;

    return RSS_ERR_NOTSUP;
}

/*
 * rss_hal_get_cpu_info -- SoC identification string.
 *
 * Answers from SCSYSID0 rather than from /proc/cpuinfo, which on this
 * kernel reports "Hisilicon (Flattened Device Tree)" and names no part at
 * all. The caller treats the result as a borrowed static string, so the
 * value is cached after the first read; the register is not going to
 * change.
 *
 * Deliberately independent of hal_init: rvd prints the banner before it
 * brings the pipeline up, and a platform check that only works afterwards
 * is a platform check that never runs on the build that needed it.
 */
const char *rss_hal_get_cpu_info(void)
{
    static char cpu[16];
    static bool loaded = false;
    uint32_t id = 0;

    if (loaded)
        return cpu;

    loaded = true;

    if (g_hisi && g_hisi->chip_name[0]) {
        snprintf(cpu, sizeof(cpu), "%s", g_hisi->chip_name);
        return cpu;
    }

    if (hisi_reg_access(HISI_SCSYSID0_ADDR, &id, false) == RSS_OK)
        snprintf(cpu, sizeof(cpu), "%s%05XV%03X", (id >> 28) == 0x7 ? "GK" : "Hi", id >> 12,
                 id & 0xfff);
    else
        snprintf(cpu, sizeof(cpu), "%s", HAL_PLATFORM_NAME);

    return cpu;
}

const char *rss_hal_get_platform_name(void)
{
    return HAL_PLATFORM_NAME;
}

/*
 * rss_hal_check_platform -- verify the binary matches the running SoC.
 *
 * gen4 is the one family in raptor where this can be done properly, and it
 * is worth doing: SCSYSID0 reports the part exactly, so neither of the
 * compromises the other backends make is necessary here. SigmaStar warns
 * only, because /proc/cpuinfo gives it a marketing string that no prefix
 * comparison can match, and checking properly there needs a SoC-ID-to-family
 * table nobody has built. Ingenic _exit(1)s on any mismatch, which is right
 * for it -- IMP reports "T31" and the parts genuinely are incompatible.
 *
 * The distinction gen4 can draw, and neither of those can, is between two
 * kinds of mismatch:
 *
 *   - a different gen4 part. One MPP build serves EV200, EV300, DV200 and
 *     3518EV300 -- the EV300 board this backend was brought up on reports
 *     the *EV200* MPP version string -- so the binary is correct and only
 *     the caps numbers may be off. Warn, and keep running.
 *   - anything else. A gen3 or gen5 part has a different MPI ABI behind
 *     identically-named symbols, so the calls would link, run, and pass
 *     garbage. That is worth refusing.
 *
 * Unreadable is not a mismatch: a kernel without /dev/mem is a
 * configuration this cannot see through, and refusing to start over a
 * failed diagnostic would be worse than the thing being diagnosed.
 */
void rss_hal_check_platform(const char *name)
{
    uint32_t id = 0;

    (void)name;

    if (hisi_reg_access(HISI_SCSYSID0_ADDR, &id, false) != RSS_OK) {
        HAL_LOG_WARN("platform check: SCSYSID0 unreadable, assuming %s", HAL_PLATFORM_NAME);
        return;
    }

    if (!hisi_chip_is_gen4(id)) {
        /* Reported the way Ingenic reports its own fatal mismatch -- stderr
         * and syslog both -- because rvd may already have detached from the
         * terminal by the time this runs, and a fatal nobody can read is
         * indistinguishable from a silent one. */
        fprintf(stderr, "FATAL: built for %s (HiMPP V4.0) but SCSYSID0 reads 0x%08x, "
                        "which is not a V4.0 part\n",
                HAL_PLATFORM_NAME, id);
        openlog(name ? name : "raptor", LOG_PID, LOG_DAEMON);
        syslog(LOG_ERR, "FATAL: built for %s (HiMPP V4.0) but SCSYSID0 reads 0x%08x, "
                        "which is not a V4.0 part",
               HAL_PLATFORM_NAME, id);
        closelog();
        _exit(1);
    }

    if (id != HISI_CHIP_THIS_PART)
        HAL_LOG_WARN("platform check: built for %s but running on %s. Same MPP build, so the "
                     "code is right; the capability numbers in hal_caps.c may not be.",
                     HAL_PLATFORM_NAME, rss_hal_get_cpu_info());
    else
        HAL_LOG_DBG("platform check: built for %s, running on %s", HAL_PLATFORM_NAME,
                    rss_hal_get_cpu_info());
}
