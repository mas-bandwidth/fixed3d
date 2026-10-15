#!/usr/bin/env bash
# Build the scalar candidate in narrow and ludicrous configs, run every scene at
# workers {1,2,4}, and diff HASH lines against tools/perf/scalar-oracle.txt.
# --regenerate re-derives the oracle and refuses to overwrite it unless two
# scalar runs agree. Worker counts for the same scene and build must agree.
# The step count every run is capped at ORACLE_STEPS (default 120).
set -u

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
ORACLE="$REPO/tools/perf/scalar-oracle.txt"
REGEN=0
SCENES=""
STEPS="${ORACLE_STEPS:-120}"

for arg in "$@"; do
	case "$arg" in
		--regenerate) REGEN=1 ;;
		--scenes=*) SCENES="${arg#--scenes=}" ;;
		--steps=*) STEPS="${arg#--steps=}" ;;
		*) echo "unknown argument: $arg" >&2; exit 2 ;;
	esac
done

case "$STEPS" in
	''|*[!0-9]*) echo "ORACLE_STEPS must be a positive integer, got '$STEPS'" >&2; exit 2 ;;
esac
[ "$STEPS" -ge 1 ] || { echo "ORACLE_STEPS must be a positive integer" >&2; exit 2; }

CMAKE_COMMON="-DCMAKE_BUILD_TYPE=RelWithDebInfo -DBOX3D_SAMPLES=OFF -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBOX3D_BENCHMARKS=ON -DBOX3D_AVX512=OFF -DBOX3D_NEON=OFF"

build_one() {
	local dir="$1"
	shift
	cmake -B "$dir" $CMAKE_COMMON "$@"
	cmake --build "$dir" --parallel 2
}

all_scenes="convex_pile joint_grid junkyard large_pyramid large_world many_pyramids rain trees100 trees50 trees25 washer ragdoll_pile bodies_20k"
if [ -n "$SCENES" ]; then
	scenes=$(printf '%s' "$SCENES" | tr ',' ' ')
else
	scenes="$all_scenes"
fi
workers="1 2 4"

busy=$(pgrep -af benchmark 2>/dev/null | grep -v -e 'pgrep' -e 'oracle_check.sh' || true)
if [ -n "$busy" ]; then
	echo "refusing to run: another benchmark process is on the box"
	echo "$busy"
	exit 3
fi

assert_workers() {
	local lines="$1"
	local scene mode hashes n
	local scenes_u modes_u
	scenes_u=$(printf '%s\n' "$lines" | awk '{print $2}' | sort -u)
	modes_u=$(printf '%s\n' "$lines" | awk '{print $4}' | sort -u)
	for scene in $scenes_u; do
		for mode in $modes_u; do
			hashes=$(printf '%s\n' "$lines" | awk -v s="$scene" -v m="$mode" '$2==s && $4==m {print $5}' | sort -u)
			n=$(printf '%s\n' "$hashes" | grep -c .)
			if [ "$n" -ne 1 ]; then
				echo "worker hashes disagree for $scene $mode"
				printf '%s\n' "$lines" | awk -v s="$scene" -v m="$mode" '$2==s && $4==m'
				return 1
			fi
		done
	done
	return 0
}

collect() {
	local dir="$1"
	local mode="$2"
	local scene w line hex
	for scene in $scenes; do
		for w in $workers; do
			echo "  [$mode] $scene workers=$w" >&2
			line=$("$dir/bin/benchmark" -b="$scene" -w="$w" -r=1 --hash --steps="$STEPS" 2>/dev/null | grep -E '^HASH [^ ]+ workers=[0-9]+ [0-9a-fA-F]{16}$' || true)
			hex=$(printf '%s\n' "$line" | awk '{print $NF}')
			if [ -z "$hex" ]; then
				echo "missing HASH for $scene workers=$w ($mode)" >&2
				return 1
			fi
			printf 'HASH %s workers=%s %s %s\n' "$scene" "$w" "$mode" "$hex"
		done
	done
}

build_one "$REPO/build"
build_one "$REPO/build-ludicrous" -DBOX3D_LUDICROUS_MODE=ON

if [ "$REGEN" -eq 1 ]; then
	r1=$(collect "$REPO/build" narrow)
	r2=$(collect "$REPO/build" narrow)
	if [ "$r1" != "$r2" ]; then
		echo "refusing to regenerate: two narrow scalar runs disagreed"
		exit 1
	fi
	l1=$(collect "$REPO/build-ludicrous" ludicrous)
	l2=$(collect "$REPO/build-ludicrous" ludicrous)
	if [ "$l1" != "$l2" ]; then
		echo "refusing to regenerate: two ludicrous scalar runs disagreed"
		exit 1
	fi
	assert_workers "$r1" || exit 1
	assert_workers "$l1" || exit 1

	# Preserve oracle lines for scenes outside this invocation so a --scenes
	# split can accumulate the full matrix across several calls.
	if [ -n "$SCENES" ] && [ -f "$ORACLE" ]; then
		kept=$(grep '^HASH ' "$ORACLE" | awk -v list="$scenes" 'BEGIN{n=split(list,a," "); for(i=1;i<=n;i++) sel[a[i]]=1} {if(!($2 in sel)) print}')
	else
		kept=""
	fi

	{
		echo "# HASH <scene> workers=<n> <narrow|ludicrous> <16 hex>"
		echo "# scalar build, no BOX3D_AVX512, no BOX3D_NEON. steps=$STEPS."
		echo "# Regenerated only when two runs agree and workers {1,2,4} agree."
		printf '%s\n' "$r1"
		printf '%s\n' "$l1"
		[ -z "$kept" ] || printf '%s\n' "$kept"
	} | sort -k2,2 -k3,3n -k4,4 > "$ORACLE"
	echo "oracle regenerated: $ORACLE"
	exit 0
fi

[ -f "$ORACLE" ] || { echo "oracle missing: $ORACLE (run with --regenerate)"; exit 1; }

new=$(collect "$REPO/build" narrow | sort)
new_lud=$(collect "$REPO/build-ludicrous" ludicrous | sort)
got=$(printf '%s\n%s\n' "$new" "$new_lud" | sort)
want=$(grep '^HASH ' "$ORACLE" | awk -v list="$scenes" 'BEGIN{n=split(list,a," "); for(i=1;i<=n;i++) sel[a[i]]=1} {if($2 in sel) print}' | sort)
assert_workers "$new" || exit 1
assert_workers "$new_lud" || exit 1
if [ "$got" != "$want" ]; then
	echo "ORACLE MISMATCH"
	diff -u <(printf '%s\n' "$want") <(printf '%s\n' "$got") | head -80
	exit 1
fi
echo "oracle check passed (narrow + ludicrous, workers 1/2/4)"
