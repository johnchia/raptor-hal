#!/bin/sh
# check-fork-surface.sh -- keep this fork's contact with upstream small.
#
# The SigmaStar backends are additive: src/star/, src/infinity6c/, the
# sigmastar-headers submodule and tests/ are files upstream does not have, so
# they can never conflict on a merge. What costs us on every sync is the handful
# of *shared* files we modify -- and the failure mode of a long-lived fork is
# not the initial diff, it is the fiftieth "just one #ifdef" landing in a file
# upstream edits every week.
#
# So this script fails when either of two invariants breaks:
#
#   1. A shared file outside the allowlist is modified at all.
#   2. The total added lines across shared files exceeds BUDGET.
#
# Adding a file to the allowlist is a deliberate act, reviewed like any other
# change. Prefer the alternative: put the code in an additive file and reach it
# from a one-line hook (see FORK.md).
#
# Usage:  scripts/check-fork-surface.sh [upstream-ref]
# Env:    UPSTREAM (default origin/main), BUDGET (default 140)
#
# SPDX-License-Identifier: MIT

set -eu

UPSTREAM=${1:-${UPSTREAM:-origin/main}}
BUDGET=${BUDGET:-140}

# Shared files this fork is allowed to touch, and why. Keep this list short.
#   .gitignore    build artefacts of the host test binaries
#   .gitmodules   the sigmastar-headers submodule
#   Makefile      platform list + the -include of mk/sigmastar.mk
#   README.md     says the fork exists and which SoCs it adds
#   src/hal_caps.c      #else arm includes caps_sigmastar.inc
#   src/hal_gpio.c      sysfs-root override for the host test (fix is upstream PR#4)
#   src/hal_internal.h  platform hook + the HAL_INGENIC_SDK brackets
ALLOWED='.gitignore
.gitmodules
Makefile
README.md
src/hal_caps.c
src/hal_gpio.c
src/hal_internal.h'

if ! git rev-parse --verify --quiet "$UPSTREAM" >/dev/null; then
	echo "check-fork-surface: cannot resolve upstream ref '$UPSTREAM'" >&2
	echo "  add the remote and fetch it, e.g.:" >&2
	echo "  git remote add origin https://github.com/gtxaspec/raptor-hal.git && git fetch origin" >&2
	exit 2
fi

# Measure from the merge base, not from the upstream tip. A two-dot diff against
# the tip reports every upstream commit we have not merged yet as though the fork
# had reverted it -- an upstream file we simply do not have yet shows up as our
# deletion. What belongs to the fork is what it changed since it diverged.
BASE=$(git merge-base HEAD "$UPSTREAM")

added_total=0
removed_total=0
offenders=''
rows=''

# Only files that exist in the base count as shared; anything else is additive.
# Diffing BASE against the working tree (not BASE..HEAD) so uncommitted edits
# are caught too; on a clean tree, as in CI, the two are identical.
while read -r added removed path; do
	[ -n "${path:-}" ] || continue
	# binary diffs report '-' for counts
	[ "$added" = '-' ] && added=0
	[ "$removed" = '-' ] && removed=0
	git cat-file -e "$BASE:$path" 2>/dev/null || continue

	rows="$rows$path|$added|$removed
"
	added_total=$((added_total + added))
	removed_total=$((removed_total + removed))

	if ! printf '%s\n' "$ALLOWED" | grep -qxF "$path"; then
		offenders="$offenders  $path (+$added/-$removed)
"
	fi
done <<EOF
$(git diff --numstat "$BASE")
EOF

printf 'fork surface vs %s (merge base with %s)\n' \
	"$(git rev-parse --short "$BASE")" "$UPSTREAM"
printf '%s' "$rows" | while IFS='|' read -r p a r; do
	[ -n "${p:-}" ] || continue
	printf '  %-24s +%-5s -%s\n' "$p" "$a" "$r"
done
printf '  %-24s +%d/-%d  (budget %d)\n' 'TOTAL' "$added_total" "$removed_total" "$BUDGET"

status=0

if [ -n "$offenders" ]; then
	printf '\nFAIL: shared files modified that are not on the allowlist:\n%s' "$offenders"
	echo "Put the change in an additive file behind a hook, or add the file to"
	echo "ALLOWED in this script with a one-line reason. See FORK.md."
	status=1
fi

if [ "$added_total" -gt "$BUDGET" ]; then
	printf '\nFAIL: %d added lines across shared files exceeds the budget of %d.\n' \
		"$added_total" "$BUDGET"
	echo "Move prose and code into additive files rather than raising the budget;"
	echo "raise it only when the fork genuinely needs more contact. See FORK.md."
	status=1
fi

[ "$status" -eq 0 ] && echo 'ok: fork surface within budget and allowlist'
exit "$status"
