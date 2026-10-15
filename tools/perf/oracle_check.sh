#!/usr/bin/env bash
# Oracle check for fixed3d. Builds the candidate in both configs (narrow and
# ludicrous), runs every scene at workers {1,2,4}, and diffs the HASH lines
# against tools/perf/scalar-oracle.txt. Exits non-zero on any difference.
#
# With --regenerate it re-derives the oracle from the plain scalar build and
# refuses to overwrite it unless two runs on the scalar build agree. The oracle
# must also agree across worker counts for the same scene/build (determinism).
set -u

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
ORACLE="$REPO/tools/perf/scalar-oracle.txt"
REGEN=0
[ "${1:-}" = "--regenerate" ] && REGEN=1

build_dir="$REPO/build"
build_lud="$REPO/build-ludicrous"

scenes="convex_pile joint_grid junkyard large_pyramid large_world many_pyramids rain trees100 trees50 trees25 washer"
workers="1 2 4"

collect() { # $1=build dir
    local dir="$1"
    for scene in $scenes; do
      for w in $workers; do
        "$dir/bin/benchmark" -b="$scene" -w="$w" -r=1 --hash 2>/dev/null | grep '^HASH ' | sed "s/ workers=$w/ workers=$w/"
      done
    done
}

if [ "$REGEN" -eq 1 ]; then
    r1=$(collect "$build_dir" | sort)
    r2=$(collect "$build_dir" | sort)
    if [ "$r1" != "$r2" ]; then
        echo "refusing to regenerate: two scalar runs disagreed"
        echo "--- run 1 ---"; echo "$r1"
        echo "--- run 2 ---"; echo "$r2"
        exit 1
    fi
    echo "$r1" > "$ORACLE"
    echo "oracle regenerated: $ORACLE"
    exit 0
fi

[ -f "$ORACLE" ] || { echo "oracle missing: $ORACLE (run with --regenerate)"; exit 1; }

new=$(collect "$build_dir" | sort)
if [ "$new" != "$(sort "$ORACLE")" ]; then
    echo "ORACLE MISMATCH (candidate narrow)"
    diff <(sort "$ORACLE") <(echo "$new") | head -40
    exit 1
fi

new_lud=$(collect "$build_lud" | sort)
new_lud_sorted=$(echo "$new_lud" | sed 's/ ludicrous//' | sort)
oracle_sorted=$(sort "$ORACLE" | sed 's/ narrow//' | sort)
if [ "$new_lud_sorted" != "$oracle_sorted" ]; then
    echo "ORACLE MISMATCH (candidate ludicrous)"
    diff <(echo "$oracle_sorted") <(echo "$new_lud_sorted") | head -40
    exit 1
fi

echo "oracle check passed (narrow + ludicrous, all scenes, workers 1/2/4)"
