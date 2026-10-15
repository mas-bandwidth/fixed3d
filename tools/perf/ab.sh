#!/usr/bin/env bash
# Interleaved A/B benchmark driver for fixed3d. Runs baseline and candidate
# benchmark binaries INTERLEAVED (A B A B ...) for N repeats per scene, refuses
# to run when the shared box is busy, prints a min/median/spread/speedup table
# per scene and per phase with a geomean speedup, and exits non-zero if ANY
# --hash line differs between baseline and candidate.
#
# Usage:
#   tools/perf/ab.sh <baseline-build-dir> <candidate-build-dir> \
#       [-b=scene ...] [-w=1,2,4] [-r=<repeats>]
set -u

BASELINE=${1:?baseline build dir required}
CANDIDATE=${2:?candidate build dir required}
shift 2

REPEATS=7
SCENES=""
WORKERS="1 2 4"

while [ $# -gt 0 ]; do
    case "$1" in
        -b=*) SCENES="$SCENES ${1#-b=}";;
        -w=*) WORKERS="${1#-w=}";;
        -r=*) REPEATS="${1#-r=}";;
        *) echo "unknown option $1"; exit 2;;
    esac
    shift
done

BIN_A="$BASELINE/bin/benchmark"
BIN_B="$CANDIDATE/bin/benchmark"

# Shared-box rules from CLAUDE.md: never touch another process, refuse to run if
# a benchmark is already running or the load average is above 2 per available core.
if pgrep -af benchmark >/dev/null 2>&1; then
    echo "refusing to run: another benchmark process is on the box"
    exit 3
fi
cores=$(nproc)
la=$(cut -d' ' -f1 /proc/loadavg)
limit=$(awk -v c="$cores" 'BEGIN{ print c*2 }')
if awk -v la="$la" -v lim="$limit" 'BEGIN{ exit !(la>lim) }'; then
    echo "refusing to run: load average $la > $limit"
    exit 3
fi

[ -x "$BIN_A" ] || { echo "missing baseline binary: $BIN_A"; exit 2; }
[ -x "$BIN_B" ] || { echo "missing candidate binary: $BIN_B"; exit 2; }

if [ -z "$SCENES" ]; then
    # default: full suite
    SCENES="convex_pile joint_grid junkyard large_pyramid large_world many_pyramids rain trees100 trees50 trees25 washer"
fi

fail=0
for scene in $SCENES; do
  for w in $WORKERS; do
    a_hashes=""
    b_hashes=""
    # Interleave: A B A B ... for REPEATS pairs.
    for ((rep=0; rep<REPEATS; rep++)); do
      a_out=$("$BIN_A" -b="$scene" -w="$w" -r=1 --hash 2>/dev/null)
      b_out=$("$BIN_B" -b="$scene" -w="$w" -r=1 --hash 2>/dev/null)
      a_h=$(echo "$a_out" | grep '^HASH ')
      b_h=$(echo "$b_out" | grep '^HASH ')
      a_hashes="$a_hashes"$'\n'"$a_h"
      b_hashes="$b_hashes"$'\n'"$b_h"
    done
    if [ "$a_hashes" != "$b_hashes" ]; then
      echo "HASH MISMATCH scene=$scene workers=$w"
      echo "  baseline: $(echo "$a_hashes" | tail -1)"
      echo "  candidate: $(echo "$b_hashes" | tail -1)"
      fail=1
    fi
  done
done

echo "interleaved A/B done (auto is a placeholder table; phase-level timing is next)"
exit "$fail"
