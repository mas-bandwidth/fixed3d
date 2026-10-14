#!/usr/bin/env bash
# ab.sh — interleaved A/B benchmark of two builds.
#
#   tools/perf/ab.sh <baseline-build-dir> <candidate-build-dir> [-b=scene ...] [-w=1,2,4] [-r=N]
#
# Runs the two benchmark binaries A B A B ... for N repeats (default 7) per scene and
# worker count, prints a min/median/spread/speedup table (whole step and per phase) plus a
# geomean, and exits non-zero if any --hash differs between baseline and candidate.
# Refuses to run if another benchmark is on the box or the load average exceeds 2 per core.
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BASE_DIR="${1:?usage: ab.sh <baseline-build-dir> <candidate-build-dir> [-b=scene] [-w=1,2,4] [-r=N]}"
CAND_DIR="${2:?usage: ab.sh <baseline-build-dir> <candidate-build-dir> [-b=scene] [-w=1,2,4] [-r=N]}"
shift 2

BASE_BIN="$BASE_DIR/bin/benchmark"
CAND_BIN="$CAND_DIR/bin/benchmark"
[ -x "$BASE_BIN" ] || { echo "ab.sh: missing $BASE_BIN" >&2; exit 1; }
[ -x "$CAND_BIN" ] || { echo "ab.sh: missing $CAND_BIN" >&2; exit 1; }

REPEATS=7
WORKERS="4"
SCENES=""

while [ $# -gt 0 ]; do
    case "$1" in
        -b=*) SCENES="$SCENES ${1#-b=}" ;;
        --benchmark=*) SCENES="$SCENES ${1#--benchmark=}" ;;
        -w=*) WORKERS="${1#-w=}" ;;
        --workers=*) WORKERS="${1#--workers=}" ;;
        -r=*) REPEATS="${1#-r=}" ;;
        --repeats=*) REPEATS="${1#--repeats=}" ;;
        *) echo "ab.sh: unknown option $1" >&2; exit 2 ;;
    esac
    shift
done

# ---- guard: never run on a busy box -----------------------------------------------
foreign="$(pgrep -af '[b]in/benchmark' || true)"
if [ -n "$foreign" ]; then
    echo "ab.sh: another benchmark is running; refusing to run (never killed it):" >&2
    echo "$foreign" >&2
    exit 3
fi

cores="$(getconf _NPROCESSORS_ONLN 2>/dev/null || nproc)"
load="$(cut -d' ' -f1 /proc/loadavg 2>/dev/null || echo 0)"
limit="$(echo "$cores * 2" | bc)"
if [ -n "$cores" ] && [ "$cores" -gt 0 ] && command -v bc >/dev/null 2>&1; then
    if [ "$(echo "$load > $limit" | bc)" = "1" ]; then
        echo "ab.sh: load average $load exceeds 2 per core ($limit); refusing to run" >&2
        exit 3
    fi
fi

# ---- scene list -------------------------------------------------------------------
if [ -z "$SCENES" ]; then
    SCENES="$( "$BASE_BIN" -l | awk '/^ *[0-9]+ /{print $2}' | tr '\n' ' ' )"
fi

TMP="$(mktemp -d "${TMPDIR:-/tmp}/ab.XXXXXX")"
FAIL=0
: > "$TMP/geo.log"

for scene in $SCENES; do
    for w in $(echo "$WORKERS" | tr ',' ' '); do
        hbase=""; hcand=""; hash_ok=1
        base_steps=""; cand_steps=""
        base_phases=""; cand_phases=""
        i=1
        while [ $i -le "$REPEATS" ]; do
            # baseline (A)
            out="$TMP/a.out"; json="$TMP/a.json"
            "$BASE_BIN" -b="$scene" -w="$w" -r=1 --hash --phases-json="$json" > "$out" 2>/dev/null
            h="$(grep '^HASH ' "$out" | tail -1 | awk '{print $NF}')"
            [ -z "$hbase" ] && hbase="$h"
            [ "$hbase" != "$h" ] && hash_ok=0
            ms="$(grep -o 'run 0 : [0-9.]*' "$out" | grep -o '[0-9.]*$')"
            base_steps="$base_steps $ms"
            base_phases="$base_phases $json"

            # candidate (B)
            out="$TMP/b.out"; json="$TMP/b.json"
            "$CAND_BIN" -b="$scene" -w="$w" -r=1 --hash --phases-json="$json" > "$out" 2>/dev/null
            h="$(grep '^HASH ' "$out" | tail -1 | awk '{print $NF}')"
            [ -z "$hcand" ] && hcand="$h"
            [ "$hcand" != "$h" ] && hash_ok=0
            ms="$(grep -o 'run 0 : [0-9.]*' "$out" | grep -o '[0-9.]*$')"
            cand_steps="$cand_steps $ms"
            cand_phases="$cand_phases $json"

            i=$((i + 1))
        done

        if [ "$hash_ok" != "1" ] || [ "$hbase" != "$hcand" ]; then
            echo "ab.sh: HASH DIFFERS for $scene workers=$w (baseline=$hbase candidate=$hcand)" >&2
            FAIL=1
        fi

        pyout="$(python3 - "$scene" "$w" "$base_steps" "$cand_steps" "$base_phases" "$cand_phases" <<'PYEOF'
import sys, json, statistics

scene = sys.argv[1]; w = sys.argv[2]
base_steps = [float(x) for x in sys.argv[3].split()]
cand_steps = [float(x) for x in sys.argv[4].split()]
base_jsons = sys.argv[5].split()
cand_jsons = sys.argv[6].split()

def stat(v):
    v = sorted(v)
    n = len(v)
    med = v[n//2] if n % 2 else 0.5*(v[n//2-1]+v[n//2])
    return min(v), med, max(v)

phases = ["pairs","collide","solve","solverSetup","constraints","prepareConstraints",
          "integrateVelocities","warmStart","solveImpulses","integratePositions",
          "relaxImpulses","applyRestitution","storeImpulses","transforms","refit",
          "bullets","sleepIslands","step"]

def load_phase(paths):
    acc = {p: [] for p in phases}
    for p in paths:
        doc = json.load(open(p))[0]
        d = doc["ns_per_step"]
        for ph in phases:
            acc[ph].append(d[ph]["median"])
    return acc

bmin, bmed, bmax = stat(base_steps)
cmin, cmed, cmax = stat(cand_steps)
spread_b = 100.0*(bmax-bmin)/bmed if bmed else 0.0
spread_c = 100.0*(cmax-cmin)/cmed if cmed else 0.0
speed = bmed/cmed if cmed else float('inf')

bp = load_phase(base_jsons)
cp = load_phase(cand_jsons)

print(f"\n== {scene} workers={w} ==")
print(f"  whole step  base: min {bmin:.1f}  med {bmed:.1f}  max {bmax:.1f} ms  (spread {spread_b:.1f}%)")
print(f"              cand: min {cmin:.1f}  med {cmed:.1f}  max {cmax:.1f} ms  (spread {spread_c:.1f}%)")
print(f"              speedup {speed:.3f}x  (base/cand, >1 = candidate faster)")
print(f"  per-phase ns/step (median over repeats), speedup base/cand:")
for ph in phases:
    b = statistics.median(bp[ph]); c = statistics.median(cp[ph])
    sp = b/c if c else 0.0
    print(f"    {ph:20s} base {b:12.1f}  cand {c:12.1f}  {sp:6.3f}x")
print(f"GEO {speed:.6f}")
PYEOF
)"
        echo "$pyout" | grep -v '^GEO '
        echo "$pyout" | grep '^GEO ' | awk '{print $2}' >> "$TMP/geo.log"
    done
done

if [ -s "$TMP/geo.log" ]; then
    geo="$(python3 -c 'import sys,math; v=[float(x) for x in open(sys.argv[1]).read().split() if x]; print(math.exp(sum(math.log(x) for x in v)/len(v)))' "$TMP/geo.log")"
    echo "geomean speedup (base/cand): $geo"
fi

rm -rf "$TMP"
[ "$FAIL" = "1" ] && exit 1
echo "ab.sh: done"
exit 0
