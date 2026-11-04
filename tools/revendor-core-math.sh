#!/usr/bin/env bash
# Regenerate extern/core-math/ from CORE-MATH at a pinned commit.
#
# CORE-MATH (https://core-math.gitlabpages.inria.fr/, MIT) is the bit-exact reference
# for every fixed-point transcendental in this repository: see docs/determinism-math.md.
# It is TEST-ONLY. These files are compiled into ./bin/test and never into libbox3d.
#
# THE VENDORED TREE IS GENERATED, NEVER HAND-EDITED, exactly as extern/fixed is by
# tools/revendor-fixed.sh. A change to a vendored file is a patch under
# extern/core-math/patches/, applied by this script, recorded with its reason in
# extern/core-math/NOTES.md. A reference that someone edited by hand is no longer the
# reference.
#
# To move the pin: edit CORE_MATH_PIN and the FETCHED_SHA256 table below, run this
# script, and commit the result together with the NOTES.md update.
#
# usage:  tools/revendor-core-math.sh [--check]
#   (no args)  rewrite extern/core-math/ from the pin
#   --check    rebuild into a temp dir and diff; exit 1 on any drift. No writes.

set -euo pipefail

CORE_MATH_REPO="https://gitlab.inria.fr/core-math/core-math"
# committed 2026-10-01T17:55:16+02:00 by Paul Zimmermann
CORE_MATH_PIN="cac142bc65dc140d645ba3f79511a9c591d1746e"

# upstream path -> vendored name, and the sha256 of each file as fetched at the pin.
# The files are fetched one by one at the pinned commit (a shallow git fetch of the
# whole repository from gitlab.inria.fr ran for minutes and failed on 2026-10-17),
# and each must match its hash here before anything is written, so the fetch cannot
# silently pick up something other than the pin.
FILES=(
	"src/binary64/atan2/atan2.c atan2.c"
	"src/binary64/atan2/tint.h tint.h"
	"src/binary64/sin/sin.c sin.c"
	"src/binary64/cos/cos.c cos.c"
	"src/binary64/log2/log2.c log2.c"
	"src/binary64/exp2/exp2.c exp2.c"
	"LICENSE LICENSE"
)
FETCHED_SHA256=(
	"atan2.c 404b0db90e536f8248101544d0fcad9141fd76b2a6cc3d189dd80dbfc865bd91"
	"tint.h 41264d54812a8ce2b3f8ca4c8c033094df7cbf2d214da1c7555783328171b921"
	"sin.c a3278038675d4f69567445deac9a0f790a64b614a3621db1ffdadd9421592884"
	"cos.c 04bffbc8b483c886f78f2c9ab3f2a1ca318dda78ab637a2983a4a9d82374e9d9"
	"log2.c b06b3c6ce071ad4298bc584f8087cbf9970b774bdab222ee05a08bbfc9a6f4a2"
	"exp2.c a4e892adf5e30ddde023b74881173e170748431979e999db8e206f221174064d"
	"LICENSE 81830875362f4d477637de58eecd6d458db108cc90c40904fca3694807b86de3"
)

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/extern/core-math"
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

sha256_of() {
	if command -v sha256sum > /dev/null 2>&1; then
		sha256sum "$1" | cut -d' ' -f1
	else
		shasum -a 256 "$1" | cut -d' ' -f1
	fi
}

STAGE="$WORK/staged"
mkdir -p "$STAGE"
for entry in "${FILES[@]}"; do
	src="${entry% *}"
	name="${entry#* }"
	url="$CORE_MATH_REPO/-/raw/$CORE_MATH_PIN/$src"
	if ! curl -sSfL --retry 3 -m 120 -o "$STAGE/$name" "$url"; then
		echo "fetch failed: $url" >&2
		exit 2
	fi
done

for entry in "${FETCHED_SHA256[@]}"; do
	name="${entry% *}"
	want="${entry#* }"
	got="$(sha256_of "$STAGE/$name")"
	if [ "$got" != "$want" ]; then
		echo "sha256 mismatch for $name at pin ${CORE_MATH_PIN:0:12}: got $got, want $want" >&2
		exit 2
	fi
done

# Patches, if any, in name order. Each is a unified diff against the staged files
# (paths relative to extern/core-math), recorded with its reason in NOTES.md.
if [ -d "$DEST/patches" ]; then
	for p in "$DEST"/patches/*.patch; do
		[ -e "$p" ] || continue
		patch -s -p1 -d "$STAGE" < "$p"
	done
fi

# NOTES.md and patches/ are OURS, not upstream's: excluded from the copy and the
# comparison, and preserved across a rewrite.
DIFF_OPTS=(-r -x NOTES.md -x patches)

if [ "$CHECK" = "1" ]; then
	if diff "${DIFF_OPTS[@]}" -q "$STAGE" "$DEST" > /dev/null 2>&1; then
		echo "extern/core-math matches pin ${CORE_MATH_PIN:0:12} -- clean"
		exit 0
	fi
	echo "DRIFT: extern/core-math does not match pin ${CORE_MATH_PIN:0:12}" >&2
	diff "${DIFF_OPTS[@]}" -u "$DEST" "$STAGE" >&2 || true
	echo >&2
	echo "The vendored tree is generated. Re-run tools/revendor-core-math.sh, or add a" >&2
	echo "patch under extern/core-math/patches/ and record its reason in NOTES.md." >&2
	exit 1
fi

KEEP="$WORK/keep"
mkdir -p "$KEEP"
[ -f "$DEST/NOTES.md" ] && cp "$DEST/NOTES.md" "$KEEP/NOTES.md"
[ -d "$DEST/patches" ] && cp -R "$DEST/patches" "$KEEP/patches"
rm -rf "$DEST"
mkdir -p "$DEST"
cp -R "$STAGE"/. "$DEST"/
[ -f "$KEEP/NOTES.md" ] && cp "$KEEP/NOTES.md" "$DEST/NOTES.md"
[ -d "$KEEP/patches" ] && cp -R "$KEEP/patches" "$DEST/patches"
echo "vendored CORE-MATH @ ${CORE_MATH_PIN:0:12} -> extern/core-math"
echo "vendored sha256:"
for entry in "${FILES[@]}"; do
	name="${entry#* }"
	echo "  $name $(sha256_of "$DEST/$name")"
done
echo "remember: update extern/core-math/NOTES.md in the same commit"
