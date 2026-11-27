#!/usr/bin/env bash
# Interleaved A/B benchmark driver for fixed3d.
# Usage:
#   tools/perf/ab.sh <baseline-build-dir> <candidate-build-dir> \
#       [-b=scene ...] [-w=1,2,4] [-r=<repeats>] [--isa-a=<variant>] [--isa-b=<variant>]
# Runs A B A B ... for N repeats (default 7). FIXED3D_ISA passes through to both
# sides; --isa-a / --isa-b force a variant per side, so one binary against
# itself compares variants: tools/perf/ab.sh build build --isa-a=scalar --isa-b=avx512
# Refuses if another benchmark
# process is on the box or the load average is above 2 per core. Never kills
# anything. Exits non-zero if any final HASH line differs.
set -u

BASELINE=${1:?baseline build dir required}
CANDIDATE=${2:?candidate build dir required}
shift 2

REPEATS=7
SCENES=""
WORKERS="1 2 4"
STEPS=""
ISA_A="${FIXED3D_ISA:-}"
ISA_B="${FIXED3D_ISA:-}"

while [ $# -gt 0 ]; do
	case "$1" in
		-b=*) SCENES="$SCENES ${1#-b=}" ;;
		-w=*) WORKERS=$(printf '%s' "${1#-w=}" | tr ',' ' ') ;;
		-r=*) REPEATS="${1#-r=}" ;;
		--steps=*) STEPS="${1#--steps=}" ;;
		--isa-a=*) ISA_A="${1#--isa-a=}" ;;
		--isa-b=*) ISA_B="${1#--isa-b=}" ;;
		*) echo "unknown option $1"; exit 2 ;;
	esac
	shift
done

STEPARG=""
if [ -n "$STEPS" ]; then
	case "$STEPS" in
		''|*[!0-9]*) echo "invalid --steps: $STEPS" >&2; exit 2 ;;
	esac
	STEPARG="--steps=$STEPS"
fi

BIN_A="$BASELINE/bin/benchmark"
BIN_B="$CANDIDATE/bin/benchmark"

busy=$(pgrep -af benchmark 2>/dev/null | grep -v -e 'pgrep' -e 'ab.sh' || true)
if [ -n "$busy" ]; then
	echo "refusing to run: another benchmark process is on the box"
	echo "$busy"
	exit 3
fi

if [ -r /proc/loadavg ]; then
	cores=$(nproc)
	la=$(cut -d' ' -f1 /proc/loadavg)
	limit=$(awk -v c="$cores" 'BEGIN{ print c*2 }')
	if awk -v la="$la" -v lim="$limit" 'BEGIN{ exit !(la+0>lim+0) }'; then
		echo "refusing to run: load average $la > $limit"
		exit 3
	fi
fi

[ -x "$BIN_A" ] || { echo "missing baseline binary: $BIN_A"; exit 2; }
[ -x "$BIN_B" ] || { echo "missing candidate binary: $BIN_B"; exit 2; }

# Run one side with its ISA variant: empty means detection picks (FIXED3D_ISA unset)
run_side() {
	local isa="$1"
	shift
	if [ -n "$isa" ]; then
		FIXED3D_ISA="$isa" "$@"
	else
		env -u FIXED3D_ISA "$@"
	fi
}

# A pre-dispatch binary does not know --isa and would run the whole suite, so
# ask its help text first
isa_name() {
	"$2" -h 2>/dev/null | grep -q -- '--isa' || return 0
	run_side "$1" "$2" --isa 2>/dev/null | sed -n 's/^isa = //p' | head -1
}
isa_a=$(isa_name "$ISA_A" "$BIN_A")
isa_b=$(isa_name "$ISA_B" "$BIN_B")
echo "isa A=${isa_a:-n/a (no --isa: pre-dispatch build)} B=${isa_b:-n/a (no --isa: pre-dispatch build)}"

if [ -z "$SCENES" ]; then
	SCENES="convex_pile joint_grid junkyard large_pyramid large_world many_pyramids rain trees100 trees50 trees25 washer ragdoll_pile bodies_20k"
fi

tmpdir=${TMPDIR:-/tmp}/fixed3d-ab.$$
mkdir -p "$tmpdir"
trap 'rm -rf "$tmpdir"' EXIT

final_hash() {
	printf '%s\n' "$1" | grep -E '^HASH [^ ]+ workers=[0-9]+ [0-9a-fA-F]{16}$' || true
}

fail=0
speedups=""

echo "scene workers phase A_min A_med A_spread B_min B_med B_spread speedup"

for scene in $SCENES; do
	for w in $WORKERS; do
		for ((rep = 0; rep < REPEATS; rep++)); do
			ja="$tmpdir/a.${scene}.${w}.${rep}.json"
			jb="$tmpdir/b.${scene}.${w}.${rep}.json"
			a_out=$(run_side "$ISA_A" "$BIN_A" -b="$scene" -w="$w" -r=1 --hash --phases-json="$ja" $STEPARG 2>/dev/null || true)
			b_out=$(run_side "$ISA_B" "$BIN_B" -b="$scene" -w="$w" -r=1 --hash --phases-json="$jb" $STEPARG 2>/dev/null || true)
			a_h=$(final_hash "$a_out")
			b_h=$(final_hash "$b_out")
			printf '%s\n' "$a_h" > "$tmpdir/a.${scene}.${w}.${rep}.hash"
			printf '%s\n' "$b_h" > "$tmpdir/b.${scene}.${w}.${rep}.hash"
			if [ "$a_h" != "$b_h" ] || [ -z "$a_h" ]; then
				echo "HASH MISMATCH scene=$scene workers=$w"
				echo "  baseline: $a_h"
				echo "  candidate: $b_h"
				fail=1
			fi
		done

		phases=$(sed -n 's/.*"phase":"\([^"]*\)".*/\1/p' "$tmpdir/a.${scene}.${w}.0.json" | sort -u)
		for phase in $phases; do
			a_vals=""
			b_vals=""
			for ((rep = 0; rep < REPEATS; rep++)); do
				av=$(sed -n "s/.*\"phase\":\"$phase\",\"min_ns\":[0-9]*,\"median_ns\":\\([0-9]*\\).*/\\1/p" "$tmpdir/a.${scene}.${w}.${rep}.json")
				bv=$(sed -n "s/.*\"phase\":\"$phase\",\"min_ns\":[0-9]*,\"median_ns\":\\([0-9]*\\).*/\\1/p" "$tmpdir/b.${scene}.${w}.${rep}.json")
				a_vals="$a_vals $av"
				b_vals="$b_vals $bv"
			done
			row=$(awk -v a="$a_vals" -v b="$b_vals" '
			function med(n, v,    i, j, t) {
				for(i=1;i<=n;i++) for(j=i+1;j<=n;j++) if(v[j]+0<v[i]+0){t=v[i]; v[i]=v[j]; v[j]=t}
				if(n%2==1) return v[(n+1)/2]
				return int( (v[n/2] + v[n/2+1]) / 2 )
			}
			BEGIN{
				n=split(a,A," "); m=split(b,B," ");
				if(n<1 || A[1]=="" || m<1 || B[1]=="") { print "na"; exit }
				amin=A[1]+0; amax=A[1]+0
				for(i=1;i<=n;i++){ if(A[i]+0<amin) amin=A[i]+0; if(A[i]+0>amax) amax=A[i]+0 }
				bmin=B[1]+0; bmax=B[1]+0
				for(i=1;i<=m;i++){ if(B[i]+0<bmin) bmin=B[i]+0; if(B[i]+0>bmax) bmax=B[i]+0 }
				amed=med(n,A); bmed=med(m,B)
				sp = (bmed>0) ? amed/bmed : 0
				printf "%d %d %d %d %d %d %.6f\n", amin, amed, amax-amin, bmin, bmed, bmax-bmin, sp
			}')
			echo "$scene $w $phase $row"
			if [ "$phase" = "step" ]; then
				sp=$(echo "$row" | awk '{print $7}')
				speedups="$speedups $sp"
			fi
		done
	done
done

geomean=$(awk -v s="$speedups" 'BEGIN{
	n=split(s,v," "); sum=0; c=0
	for(i=1;i<=n;i++) if(v[i]+0>0){ sum+=log(v[i]+0); c++ }
	if(c==0){ print "na"; exit }
	print exp(sum/c)
}')
echo "geomean_speedup $geomean"
echo "interleaved A/B done"
exit "$fail"
