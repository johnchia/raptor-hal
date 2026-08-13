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
SUITES = osd isp audio gpio
BINS = $(addprefix t_,$(SUITES))

test: $(addprefix test-,$(SUITES))

$(addprefix test-,$(SUITES)): test-%: t_%
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

clean:
	rm -f $(BINS)

.PHONY: test clean abi-check abi-check-i6c $(addprefix test-,$(SUITES))
