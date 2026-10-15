#!/usr/bin/env bash
# Build the scalar candidate in narrow and ludicrous configs, run every scene at
# workers {1,2,4}, and diff HASH lines against tools/perf/scalar-oracle.txt.
# --regenerate re-derives the oracle and refuses to overwrite it unless two
# scalar runs agree. Worker counts for the same scene and build must agree.
set -u

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
ORACLE="$REPO/tools/perf/scalar-oracle.txt"
REGEN=0
[ "${1:-}" = "--regenerate" ] && REGEN=1

CMAKE_COMMON="-DCMAKE_BUILD_TYPE=RelWithDebInfo -DBOX3D_SAMPLES=OFF -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBOX3D_BENCHMARKS=ON -DBOX3D_AVX512=OFF -DBOX3D_NEON=OFF"

build_one() {
	local dir="$1"
	shift
	cmake -B "$dir" $CMAKE_COMMON "$@"
	cmake --build "$dir" --parallel 2
}

scenes="convex_pile joint_grid junkyard large_pyramid large_world many_pyramids rain trees100 trees50 trees25 washer ragdoll_pile bodies_20k"
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
			line=$("$dir/bin/benchmark" -b="$scene" -w="$w" -r=1 --hash 2>/dev/null | grep -E '^HASH [^ ]+ workers=[0-9]+ [0-9a-fA-F]{16}$' || true)
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
	{
		echo "# HASH <scene> workers=<n> <narrow|ludicrous> <16 hex>"
		echo "# scalar build, no BOX3D_AVX512, no BOX3D_NEON. Regenerated only when two runs agree."
		printf '%s\n' "$r1"
		printf '%s\n' "$l1"
	} > "$ORACLE"
	echo "oracle regenerated: $ORACLE"
	exit 0
fi

[ -f "$ORACLE" ] || { echo "oracle missing: $ORACLE (run with --regenerate)"; exit 1; }

new=$(collect "$REPO/build" narrow | sort)
new_lud=$(collect "$REPO/build-ludicrous" ludicrous | sort)
got=$(printf '%s\n%s\n' "$new" "$new_lud" | sort)
want=$(grep '^HASH ' "$ORACLE" | sort)
assert_workers "$new" || exit 1
assert_workers "$new_lud" || exit 1
if [ "$got" != "$want" ]; then
	echo "ORACLE MISMATCH"
	diff -u <(printf '%s\n' "$want") <(printf '%s\n' "$got") | head -80
	exit 1
fi
echo "oracle check passed (narrow + ludicrous, all scenes, workers 1/2/4)"
