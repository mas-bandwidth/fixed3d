#!/usr/bin/env bash
# oracle_check.sh — verify the scalar build reproduces the frozen body-state hashes.
#
#   tools/perf/oracle_check.sh [<narrow-build-dir> [<ludicrous-build-dir>]] [--regenerate]
#
# Runs every scene in the two scalar configs (no BOX3D_AVX512, no BOX3D_NEON) with worker
# counts {1,2,4} and diffs the HASH lines against tools/perf/scalar-oracle.txt. It also
# asserts the determinism claim: a scene's hash must agree across workers 1, 2 and 4 within
# a build. With --regenerate it rewrites the oracle, but only after two independent scalar
# runs produce identical hashes.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ORACLE_FILE="$SCRIPT_DIR/scalar-oracle.txt"
NARROW_DIR="${1:-$SCRIPT_DIR/../../build}"
LUDICROUS_DIR="${2:-$SCRIPT_DIR/../../build-ludicrous}"
REGENERATE=0
for a in "$@"; do
    [ "$a" = "--regenerate" ] && REGENERATE=1
done

TMP="${TMPDIR:-/tmp}/oracle_check.$$"
mkdir -p "$TMP"

NARROW_BIN="$NARROW_DIR/bin/benchmark"
LUDICROUS_BIN="$LUDICROUS_DIR/bin/benchmark"

fail() { echo "oracle_check: $*" >&2; exit 1; }

for bin in "$NARROW_BIN" "$LUDICROUS_BIN"; do
    [ -x "$bin" ] || fail "missing benchmark binary: $bin (build the dir first)"
done

# Run one config and emit normalized lines: "HASH <scene> workers=<n> <config> <hash>".
collect() {
    local bin="$1" config="$2"
    "$bin" -t=4 -r=1 --hash 2>/dev/null \
        | grep '^HASH ' \
        | sed -E 's/^HASH ([^ ]+) workers=([0-9]+) ([0-9a-f]{16})/HASH \1 workers=\2 '"$config"' \3/' \
        | grep -E 'workers=(1|2|4) ' \
        | sort
}

# Assert that, within one config, every scene has the same hash across workers {1,2,4}.
assert_determinism() {
    local data="$1" cfg="$2"
    awk -v cfg="$cfg" '
        $4 == cfg {
            scene = $2;
            if (!(scene in seen)) seen[scene] = $5;
            if (seen[scene] != $5) {
                bad = 1;
                print "  DETERMINISM FAIL " scene " workers=" $3 " hash=" $5 " expected=" seen[scene];
            }
        }
        END { exit bad }
    ' "$data"
}

echo "oracle_check: collecting narrow ($NARROW_BIN)"
collect "$NARROW_BIN" narrow > "$TMP/narrow.tmp"
echo "oracle_check: collecting ludicrous ($LUDICROUS_BIN)"
collect "$LUDICROUS_BIN" ludicrous > "$TMP/ludicrous.tmp"

cat "$TMP/narrow.tmp" "$TMP/ludicrous.tmp" | sort > "$TMP/candidate.tmp"

assert_determinism "$TMP/candidate.tmp" narrow \
    || fail "narrow hashes disagree across worker counts (determinism claim broken)"
assert_determinism "$TMP/candidate.tmp" ludicrous \
    || fail "ludicrous hashes disagree across worker counts (determinism claim broken)"

if [ "$REGENERATE" -eq 1 ]; then
    collect "$NARROW_BIN" narrow > "$TMP/narrow2.tmp"
    collect "$LUDICROUS_BIN" ludicrous > "$TMP/ludicrous2.tmp"
    cat "$TMP/narrow2.tmp" "$TMP/ludicrous2.tmp" | sort > "$TMP/candidate2.tmp"
    cmp -s "$TMP/candidate.tmp" "$TMP/candidate2.tmp" \
        || fail "two scalar runs disagree; refusing to overwrite $ORACLE_FILE"
    cp "$TMP/candidate.tmp" "$ORACLE_FILE"
    echo "oracle_check: regenerated $ORACLE_FILE"
    exit 0
fi

[ -f "$ORACLE_FILE" ] || fail "no oracle file at $ORACLE_FILE (run with --regenerate to create it)"

if diff -u "$ORACLE_FILE" "$TMP/candidate.tmp" > "$TMP/diff.tmp"; then
    echo "oracle_check: PASS — all hashes match $ORACLE_FILE"
    exit 0
else
    echo "oracle_check: FAIL — hash mismatch:" >&2
    cat "$TMP/diff.tmp" >&2
    exit 1
fi
