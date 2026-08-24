#!/usr/bin/env bash
# Regenerate extern/fixed/ from mas-bandwidth/fixed at a pinned commit.
#
# THE VENDORED TREE IS GENERATED, NEVER HAND-EDITED. That is the whole contract:
# a hand-maintained copy of another repository is two copies of one truth and it
# rots silently, because nothing ever fails when they diverge. This script is the
# generator; .github/workflows/vendor-drift.yml is the reader that fails when the
# tree on disk stops matching the pin. A generator without that check is just a
# copy with extra steps.
#
# To move the pin: edit FIXED_PIN below, run this script, commit the result and the
# NOTES.md update together. Reviewing the diff IS the upstream review.
#
# usage:  tools/revendor-fixed.sh [--check]
#   (no args)  rewrite extern/fixed/ from the pin
#   --check    rebuild into a temp dir and diff; exit 1 on any drift. No writes.

set -euo pipefail

FIXED_REPO="https://github.com/mas-bandwidth/fixed.git"
# v1.3.0. Pinned as a SHA rather than the tag name on purpose: a tag can be moved,
# a commit cannot, and "vendored at v1.3.0" should mean one specific tree forever.
FIXED_PIN="db2e4809f62a031e994b336cbada2d012f9c99df"
FIXED_VERSION="v1.3.0"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$ROOT/extern/fixed"
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# Fetch exactly the pinned commit. A shallow fetch of one SHA, so this cannot
# silently pick up whatever main happens to be today -- which is the failure mode
# a "vendor the latest" script has and never reports.
git -C "$WORK" init -q
git -C "$WORK" remote add origin "$FIXED_REPO"
if ! git -C "$WORK" fetch -q --depth 1 origin "$FIXED_PIN" 2>/dev/null; then
	echo "fetch of pinned commit failed; retrying with full history" >&2
	git -C "$WORK" fetch -q origin
fi
git -C "$WORK" checkout -q "$FIXED_PIN"

STAGE="$WORK/staged"
mkdir -p "$STAGE/include/fixed" "$STAGE/src"
cp "$WORK"/include/fixed/*.h "$STAGE/include/fixed/"
cp "$WORK"/src/*.c           "$STAGE/src/"
cp "$WORK"/LICENSE "$WORK"/NOTICE "$STAGE/"

# Tests and CI deliberately do NOT come across: they are the upstream repo's job,
# they run there on every push, and duplicating them here would mean two suites
# that can disagree about the same code.

# NOTES.md is OURS -- fixed3d's record of what is vendored and why. It lives inside this
# directory so a reader finds it, but it does not come from upstream, so it is excluded
# from both the copy and the comparison. Found by the check itself: it reported NOTES.md
# as drift, and a plain re-vendor would have rm -rf'd the file documenting the re-vendor.
DIFF_OPTS=(-r -x NOTES.md)

if [ "$CHECK" = "1" ]; then
	if diff "${DIFF_OPTS[@]}" -q "$STAGE" "$DEST" > /dev/null 2>&1; then
		echo "extern/fixed matches pin ${FIXED_PIN:0:12} -- clean"
		exit 0
	fi
	echo "DRIFT: extern/fixed does not match pin ${FIXED_PIN:0:12}" >&2
	diff "${DIFF_OPTS[@]}" -u "$DEST" "$STAGE" >&2 || true
	echo >&2
	echo "The vendored tree is generated. Either re-run tools/revendor-fixed.sh," >&2
	echo "or if you meant to change the code, change it upstream in" >&2
	echo "mas-bandwidth/fixed and move FIXED_PIN." >&2
	exit 1
fi

# Preserve NOTES.md across the rewrite -- it is ours, and destroying the file that
# documents the vendoring during a re-vendor would be a particularly stupid loss.
NOTES_KEEP=""
if [ -f "$DEST/NOTES.md" ]; then
	NOTES_KEEP="$WORK/NOTES.md.keep"
	cp "$DEST/NOTES.md" "$NOTES_KEEP"
fi
rm -rf "$DEST"
mkdir -p "$DEST"
cp -R "$STAGE"/. "$DEST"/
[ -n "$NOTES_KEEP" ] && cp "$NOTES_KEEP" "$DEST/NOTES.md"
echo "vendored mas-bandwidth/fixed @ ${FIXED_PIN:0:12} -> extern/fixed"
echo "remember: update the pin line in extern/fixed/NOTES.md in the same commit"
