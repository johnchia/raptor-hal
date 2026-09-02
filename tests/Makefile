# Host-side logic tests for the SigmaStar (Infinity6E) backend.
#
# Each test #includes the real translation unit it exercises, so the code
# under test is the code that ships, and stubs MI by filling in the
# function pointers in star_state_t. That is the whole reason these can
# run on a host at all: nothing in src/star links libmi_*.so directly,
# every call goes through a dlopen'd pointer.
#
# WHY -D'_Static_assert(c,m)='
#
# The i6_*.h headers assert the vendor ABI's 32-bit struct layouts
# (sizeof(pointer) == 4 and the offsets that follow from it). On an x86-64
# host every one of those is false, and they are *supposed* to be -- they
# exist to catch an ABI drift in the ARM build, which still checks all of
# them with -Werror. Suppressing them here is what lets the logic be
# tested at all. -m32 would be the alternative and is not available: this
# host has no 32-bit libc headers.
#
#   make -C tests test      build and run everything
#   make -C tests test-osd   just one suite
#
# Each suite is its own binary with its own main(), so a crash in one does
# not hide the others.

CC ?= gcc
SAN ?= address

# SIGMASTAR_HEADERS mirrors the top-level Makefile's variable, so a checkout
# with the submodule elsewhere can point both at the same place. The MI
# declarations are host-compiled here with the size assertions defined away:
# they describe an ARM ABI, and the suites test logic rather than layout.
SIGMASTAR_HEADERS ?= ../sigmastar-headers

CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) \
	 -D'_Static_assert(c,m)=' -DPLATFORM_INFINITY6E=1 \
	 -I../src -I../include -I../../raptor-common/include \
	 -I$(SIGMASTAR_HEADERS)/infinity6e
LDFLAGS = -fsanitize=$(SAN) -lpthread

# gpio is the odd one out: src/hal_gpio.c is platform-independent sysfs
# rather than a star/ file. It is here because phase 6 (IR-cut day/night)
# rests on it and its behaviour is which file gets written in what order,
# which a host can check once the sysfs root is redirected.
SUITES = osd isp audio gpio enc
BINS = $(addprefix t_,$(SUITES)) t_isp_i6c t_enc_i6c t_isp_imp t_enc_imp t_hisi_iq t_hisi_osd

test: $(addprefix test-,$(SUITES)) test-isp_i6c test-enc_i6c test-isp_imp test-enc_imp \
      test-hisi_iq test-hisi_osd

$(addprefix test-,$(SUITES)): test-%: t_%
	@./$<

test-isp_i6c: t_isp_i6c
	@./$<

test-enc_i6c: t_enc_i6c
	@./$<

# The suites #include the sources they test, so those are prerequisites --
# without this a change to src/star leaves make thinking the binary is up
# to date, and the suite reports on code that is no longer there. That is
# not a hypothetical: it produced a pass on a version of hal_isp.c that had
# already been edited, and then a failure that looked like the edit's fault.
# Wildcarded deliberately -- an over-broad prerequisite costs a rebuild of
# four small files, a missing one costs a wrong answer.
DEPS = $(wildcard ../src/star/*.c ../src/star/*.h ../src/*.c ../src/*.h \
		  ../include/*.h ../../raptor-common/include/*.h)

t_%: t_%.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# Infinity6C, which needs its own everything: a different PLATFORM_ define
# selects a different state struct, and the MI 3.0 headers are a separate
# include directory whose types collide with Infinity6E's. Hence a rule of
# its own rather than another name in SUITES -- the two cannot share CFLAGS,
# and putting both in one binary is not possible at all.
I6C_CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) \
	     -D'_Static_assert(c,m)=' -DPLATFORM_INFINITY6C=1 \
	     -I../src -I../include -I../../raptor-common/include \
	     -I$(SIGMASTAR_HEADERS)/infinity6c

I6C_DEPS = $(wildcard ../src/infinity6c/*.c ../src/infinity6c/*.h ../src/*.c ../src/*.h \
		      ../include/*.h ../../raptor-common/include/*.h)

t_isp_i6c t_enc_i6c: t_%: t_%.c $(I6C_DEPS)
	$(CC) $(I6C_CFLAGS) -o $@ $< $(LDFLAGS)

# HiSilicon gen4: a fourth PLATFORM_ define and no vendor headers at all --
# the backend declares the HiMPP ABI itself in src/hisi_v4/v4_*.h, so the
# suite needs only the tree. _Static_assert is defined away for the same
# reason as the SigmaStar suites: the audio structs carry pointers whose
# ARM32 offsets are asserted (the tuning structs would hold on the host,
# but the state header pulls in both).
HISI_CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) \
	      -D'_Static_assert(c,m)=' -DPLATFORM_HI3516EV300=1 \
	      -I../src -I../include -I../../raptor-common/include

HISI_DEPS = $(wildcard ../src/hisi_v4/*.c ../src/hisi_v4/*.h ../src/*.c ../src/*.h \
		       ../include/*.h ../../raptor-common/include/*.h)

t_hisi_iq t_hisi_osd: t_%: t_%.c $(HISI_DEPS)
	$(CC) $(HISI_CFLAGS) -o $@ $< $(LDFLAGS) -ldl

test-hisi_iq: t_hisi_iq
	@./$<

# t_hisi_osd includes src/hisi_v4/hal_osd.c and stubs RGN through
# hisi_state_t's v4_rgn_impl. Its point is the attach descriptor -- the
# divinus defect the plan names is a .channel that is never set -- plus the
# registered-before-bound deferral and the ARGB1555 conversion, which is
# the one place a host suite can catch an over-read of the caller's bitmap.
test-hisi_osd: t_hisi_osd
	@./$<

# Ingenic, which needs neither of the above: a third PLATFORM_ define, the
# vendor's own headers out of ingenic-headers, and -- unlike the SigmaStar
# backends -- a link against the SDK, because src/hal_isp.c calls IMP_* by name
# rather than through a dlopen'd pointer. imp_stubs.c is what stands in for it.
#
# Built for T31 only. The file is compiled for ten platforms by the ARM build,
# which is what checks that each generation's #ifdef branch still compiles; what
# is tested here is the logic above those branches, and that is the same code on
# all ten.
#
# INGENIC_HEADERS mirrors the top-level Makefile's variable. The suite is
# skipped rather than failed when the tree is absent, since it is a submodule
# and a SigmaStar-only checkout has no reason to populate it.
INGENIC_HEADERS ?= ../ingenic-headers
IMP_HEADERS = $(INGENIC_HEADERS)/T31/1.1.6/en

IMP_CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -fsanitize=$(SAN) \
	     -DPLATFORM_T31=1 -DHAL_MODULE_VIDEO=1 \
	     -I../src -I../include -I../../raptor-common/include -I$(IMP_HEADERS)

IMP_DEPS = $(wildcard ../src/*.c ../src/*.h ../include/*.h ../../raptor-common/include/*.h)

t_isp_imp: t_isp_imp.c imp_stubs.c $(IMP_DEPS)
	@if [ -d "$(IMP_HEADERS)" ]; then \
		$(CC) $(IMP_CFLAGS) -o $@ t_isp_imp.c imp_stubs.c $(LDFLAGS); \
	else \
		echo "t_isp_imp: skipped, no $(IMP_HEADERS)"; \
	fi

test-isp_imp: t_isp_imp
	@if [ -x ./t_isp_imp ]; then ./t_isp_imp; fi

t_enc_imp: t_enc_imp.c imp_enc_stubs.c $(IMP_DEPS)
	@if [ -d "$(IMP_HEADERS)" ]; then \
		$(CC) $(IMP_CFLAGS) -o $@ t_enc_imp.c imp_enc_stubs.c $(LDFLAGS); \
	else \
		echo "t_enc_imp: skipped, no $(IMP_HEADERS)"; \
	fi

test-enc_imp: t_enc_imp
	@if [ -x ./t_enc_imp ]; then ./t_enc_imp; fi

# ABI conformance against SigmaStar's own headers. Opt-in: the vendor
# headers are not redistributable and are not in the repo, so this is
# skipped rather than failed when SIGMASTAR_SDK is unset.
#
# SIGMASTAR_SDK points at the SDK's release include directory -- the one
# holding mi_common_datatype.h. `pudding` is the Infinity6E ISP header
# set; the other families sit beside it and are worth compiling against
# deliberately when asking whether a struct moved between SoCs.
#
# Cross-compiled, and that is not incidental: this is the one thing here
# that is *about* layout, so it has to be the target's ABI, with the
# _Static_asserts left in place rather than defined away. Compile-only,
# so no sysroot beyond the toolchain's own headers is needed.
SIGMASTAR_SDK ?=
ABI_CC = $(CROSS_COMPILE)gcc
ABI_CFLAGS = -Wall -Wextra -std=c11 -DPLATFORM_INFINITY6E=1 \
	     -I$(SIGMASTAR_HEADERS)/infinity6e \
	     -I$(SIGMASTAR_SDK) -I$(SIGMASTAR_SDK)/isp/pudding

abi-check:
ifeq ($(SIGMASTAR_SDK),)
	@echo "abi-check: skipped, set SIGMASTAR_SDK=/path/to/project/release/include"
else
	$(ABI_CC) $(ABI_CFLAGS) -c -o /dev/null abi_iq.c
	@echo "abi-check: IQ table matches the vendor headers"
endif

# The same check for Infinity6C, against its own SDK and its own ISP header
# set. Separate because MI 3.0's tuning calls take a leading device argument
# and because two of the modules moved -- see abi_iq_i6c.c. SIGMASTAR_SDK
# points at an SSC377-generation release/include here, not Infinity6E's.
ABI_I6C_CFLAGS = -Wall -Wextra -std=c11 -DPLATFORM_INFINITY6C=1 \
		 -I$(SIGMASTAR_HEADERS)/infinity6c \
		 -I$(SIGMASTAR_SDK)

abi-check-i6c:
ifeq ($(SIGMASTAR_SDK),)
	@echo "abi-check-i6c: skipped, set SIGMASTAR_SDK=/path/to/project/release/include"
else
	$(ABI_CC) $(ABI_I6C_CFLAGS) -c -o /dev/null abi_iq_i6c.c
	@echo "abi-check-i6c: IQ table matches the vendor headers"
endif

# The gen4 (HiMPP V4.0) layouts. Different shape of check: the v4_*.h
# headers are self-contained transcriptions with their offsets pinned by
# _Static_assert, so there is no SDK to compile against -- but the asserts
# describe 32-bit ARM EABI and are defined away in HISI_CFLAGS above, which
# means the host suite never runs them. This target compiles them live with
# the ARM cross-compiler; it needs CROSS_COMPILE and nothing else.
abi-check-hisi:
ifeq ($(CROSS_COMPILE),)
	@echo "abi-check-hisi: skipped, set CROSS_COMPILE=arm-openipc-linux-musleabi-"
else
	$(ABI_CC) -Wall -Wextra -std=c11 -DPLATFORM_HI3516EV300=1 \
	    -I../src/hisi_v4 -I../src -I../include -I../../raptor-common/include \
	    -c -o /dev/null abi_hisi.c
	@echo "abi-check-hisi: gen4 ABI transcriptions hold ($$(grep -c _Static_assert ../src/hisi_v4/v4_*.h | awk -F: '{n+=$$2} END {print n}') asserts)"
endif

clean:
	rm -f $(BINS)

.PHONY: test clean abi-check abi-check-i6c abi-check-hisi test-isp_i6c test-enc_i6c \
	test-hisi_iq test-hisi_osd $(addprefix test-,$(SUITES))
