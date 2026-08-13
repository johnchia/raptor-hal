# Maintaining this fork

This is a long-lived downstream fork of [gtxaspec/raptor-hal][upstream] adding
SigmaStar Infinity6 support. It is not a staging area for upstreaming the
backends: upstream targets Ingenic, and the MI backends are expected to live
here indefinitely.

Which makes exactly one thing worth engineering for: **the cost of taking
upstream's next commit.**

[upstream]: https://github.com/gtxaspec/raptor-hal

## The invariant

Everything vendor-specific goes in a file upstream does not have. Files upstream
*does* have are touched as little as possible, and only through a hook.

That is what makes the fork cheap. Upstream's most-edited files are
`src/hal_encoder.c`, `src/hal_common.c`, `include/raptor_hal.h`, `src/hal_isp.c`
and `src/hal_osd.c` — and this fork modifies **none of them**, because the
backends are parallel implementations in `src/star/` and `src/infinity6c/`
selected by `BACKEND_DIR`, not `#ifdef`s threaded through the shared ones.

Additive files cannot conflict. Shared files always can. So:

| kind | where | conflict cost |
|---|---|---|
| backend implementations | `src/star/`, `src/infinity6c/` | none |
| vendor ABI declarations | `sigmastar-headers/` (submodule) | none |
| capability tables | `src/caps_sigmastar.inc` | none |
| platform identity, vendor select | `src/hal_internal_sigmastar.h` | none |
| build settings | `mk/sigmastar.mk` | none |
| host tests | `tests/` | none |
| **hooks into the above** | 7 shared files | **the whole cost** |

## The hooks, in full

There are only four, and each is a couple of lines:

- `src/hal_caps.c` — the platform chain's `#else` arm includes
  `caps_sigmastar.inc`. That file opens its own `#if` chain and owns the
  unknown-platform `#error`; an `#elif` whose `#if` lives in another file is not
  valid C, which is why it is shaped that way.
- `src/hal_internal.h` — the name chain's `#else` arm includes
  `hal_internal_sigmastar.h`, which sets `HAL_PLATFORM_NAME` and
  `HAL_SIGMASTAR_SDK`; the Ingenic half is then defined by elimination. The
  remaining `#ifdef HAL_INGENIC_SDK` brackets are irreducible — they bracket
  upstream's own IMP-specific sections.
- `Makefile` — `-include mk/sigmastar.mk` near the top, and branches keyed on
  `$(VENDOR)`. Written so that if the fragment is **absent** the build behaves
  exactly like upstream: `VENDOR` is empty, every branch takes the Ingenic path,
  and a SigmaStar `PLATFORM` is rejected as invalid rather than mis-built.
- `src/hal_gpio.c` — a compile-time sysfs-root override so the host test can
  point at a temp directory. The direction fix itself is upstream PR#4; once
  that merges this shrinks to the override alone.

Prose lives with the code it explains, in the additive files. It is deliberately
**not** in the shared files: a comment block in a file upstream edits weekly is
pure conflict risk for zero function.

## The tripwire

    scripts/check-fork-surface.sh          # needs origin/main fetched

Fails if a shared file outside its allowlist is modified at all, or if the total
added lines across shared files exceeds the budget. CI runs it on every push and
weekly (`.github/workflows/fork-surface.yml`).

The allowlist is the load-bearing half. A stray `#ifdef` in `hal_encoder.c` is
caught by the allowlist long before it would trouble the budget — and that
single edit, repeated, is how forks become unmergeable.

Adding to the allowlist is allowed but deliberate: prefer moving the code into an
additive file behind a hook. Raising the budget to accommodate prose is the wrong
move; move the prose instead.

## Syncing with upstream

Merge, never rebase — this branch is published, and rebasing rewrites well over
a hundred commits.

    git fetch origin
    git merge origin/main
    scripts/check-fork-surface.sh
    # then build every family, see below

Upstream moves slowly (single digits of commits per quarter), so weekly CI is
enough to keep drift to one commit at a time. Sync while the drift is small: the
conflicts you get merging one commit are trivial, and the ones you get merging
fifty are a project.

When a change is platform-neutral, send it upstream — every commit that lands
there deletes lines from this fork permanently.

## Building every family before you push a sync

The Ingenic control build needs no cross toolchain (headers come from the
`ingenic-headers` submodule, and CI runs this one):

    make PLATFORM=T31 CROSS_COMPILE= CC=gcc

The SigmaStar families need an ARM cross compiler. Their ABI headers carry
`_Static_assert`s on struct sizes, so a 64-bit host build fails by design —
those assertions are the point, and a host "failure" there is the check working,
not a break:

    make PLATFORM=INFINITY6E  CROSS_COMPILE=arm-linux-gnueabihf-
    make PLATFORM=INFINITY6B0 CROSS_COMPILE=arm-linux-gnueabihf-
    make PLATFORM=INFINITY6C  CROSS_COMPILE=arm-openipc-linux-musleabihf- \
        SYSROOT=/path/to/openipc/output/staging

Host tests, which need no toolchain at all:

    make -C tests test
