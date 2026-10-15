#!/usr/bin/env bash
# Fail-closed scalar HASH oracle. Every selected scene needs both modes and all
# three worker counts before a same-directory temporary file replaces the oracle.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
ORACLE="$REPO/tools/perf/scalar-oracle.txt"
REGEN=0
SELF_TEST=0
SCENES=""
SCENES_SET=0
STEPS="${ORACLE_STEPS:-120}"
all_scenes="convex_pile joint_grid junkyard large_pyramid large_world many_pyramids rain trees100 trees50 trees25 washer ragdoll_pile bodies_20k"
workers="1 2 4"

for arg in "$@"; do
	case "$arg" in
		--regenerate) REGEN=1 ;;
		--self-test) SELF_TEST=1 ;;
		--scenes=*) SCENES_SET=1; SCENES="${arg#--scenes=}" ;;
		--steps=*) STEPS="${arg#--steps=}" ;;
		*) echo "unknown argument: $arg" >&2; exit 2 ;;
	esac
done
case "$STEPS" in
	''|*[!0-9]*) echo "ORACLE_STEPS must be a positive integer, got '$STEPS'" >&2; exit 2 ;;
esac
[ "$STEPS" -ge 1 ] && [ "$STEPS" -le 1000000 ] || {
	echo "ORACLE_STEPS must be between 1 and 1000000" >&2; exit 2;
}

# Commas are the only separator accepted for a split invocation. Rejecting
# duplicate and unknown scene names keeps the matrix count unambiguous.
if [ "$SCENES_SET" -eq 1 ]; then
	[ -n "$SCENES" ] || { echo "empty --scenes" >&2; exit 2; }
	[[ "$SCENES" =~ ^[a-z0-9_]+(,[a-z0-9_]+)*$ ]] || {
		echo "invalid --scenes list: $SCENES" >&2; exit 2;
	}
	scenes="${SCENES//,/ }"
else
	scenes="$all_scenes"
fi
selected=" "
for scene in $scenes; do
	case " $all_scenes " in *" $scene "*) ;; *) echo "unknown scene: $scene" >&2; exit 2 ;; esac
	case "$selected" in *" $scene "*) echo "duplicate scene: $scene" >&2; exit 2 ;; esac
	selected="$selected$scene "
done
[ "$selected" != " " ] || { echo "no scenes selected" >&2; exit 2; }

# Validate exact keys, legal values, and worker determinism. For an existing
# split oracle, require complete six-row groups for every scene it contains.
validate_matrix() {
	local lines="$1" expected="$2" require_all="$3"
	printf '%s\n' "$lines" | awk -v expected="$expected" -v require_all="$require_all" '
	BEGIN { n=split(expected,names," "); for(i=1;i<=n;i++) allowed[names[i]]=1; bad=0 }
	/^#/ || NF==0 { next }
	{
		if (NF!=5 || $1!="HASH" || !($2 in allowed) ||
		    $3 !~ /^workers=[124]$/ || ($4!="narrow" && $4!="ludicrous") ||
		    length($5)!=16 || $5 ~ /[^0-9a-f]/) {
			print "invalid oracle row: " $0 > "/dev/stderr"; bad=1; next
		}
		key=$2 SUBSEP $4 SUBSEP $3
		if (++seen[key]!=1) { print "duplicate oracle key: " $0 > "/dev/stderr"; bad=1 }
		group=$2 SUBSEP $4
		if (group in hash && hash[group]!=$5) {
			print "worker hashes disagree: " $2 " " $4 > "/dev/stderr"; bad=1
		}
		hash[group]=$5; present[$2]=1
	}
	END {
		for (i=1;i<=n;i++) {
			s=names[i]
			if (require_all && !(s in present)) { print "missing scene: " s > "/dev/stderr"; bad=1 }
			if (!(s in present)) continue
			for (m=1;m<=2;m++) {
				mode=(m==1 ? "narrow" : "ludicrous")
				for (w=1;w<=3;w++) {
					worker=(w==1 ? 1 : (w==2 ? 2 : 4))
					key=s SUBSEP mode SUBSEP "workers=" worker
					if (seen[key]!=1) { print "missing oracle key: " s " " mode " workers=" worker > "/dev/stderr"; bad=1 }
				}
			}
		}
		exit bad ? 1 : 0
	}'
}

# Check cap and existing matrix before building: a partial regeneration must
# never silently combine rows produced at different step counts.
prior_rows=""
if [ -f "$ORACLE" ]; then
	cap_headers=$(grep -Ec '^# scalar build, no BOX3D_AVX512, no BOX3D_NEON\. steps=[0-9]+\.$' "$ORACLE" || true)
	[ "$cap_headers" -eq 1 ] || { echo "oracle has no unique steps header" >&2; exit 1; }
	prior_steps=$(sed -n 's/^# scalar build, no BOX3D_AVX512, no BOX3D_NEON\. steps=\([0-9][0-9]*\)\.$/\1/p' "$ORACLE")
	if [ "$prior_steps" -ne "$STEPS" ] && { [ "$REGEN" -ne 1 ] || [ "$SCENES_SET" -eq 1 ]; }; then
		echo "oracle steps=$prior_steps differs from requested steps=$STEPS; refusing mixed-cap run" >&2; exit 1;
	fi
	prior_rows=$(grep '^HASH' "$ORACLE" || true)
	validate_matrix "$prior_rows" "$all_scenes" 0 || exit 1
	# Anything other than comments and canonical HASH rows is corruption.
	if grep -Ev '^(#|HASH |$)' "$ORACLE" >/dev/null; then
		echo "unexpected oracle content" >&2; exit 1
	fi
elif [ "$REGEN" -ne 1 ]; then
	echo "oracle missing: $ORACLE (run with --regenerate)" >&2; exit 1
fi

CMAKE_COMMON="-DCMAKE_BUILD_TYPE=RelWithDebInfo -DBOX3D_SAMPLES=OFF -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBOX3D_BENCHMARKS=ON -DBOX3D_AVX512=OFF -DBOX3D_NEON=OFF"
build_one() {
	local dir="$1"
	shift
	cmake -B "$dir" $CMAKE_COMMON "$@" || return 1
	cmake --build "$dir" --parallel 2 || return 1
}

busy=$(pgrep -af benchmark 2>/dev/null | grep -v -e 'pgrep' -e 'oracle_check.sh' || true)
if [ -n "$busy" ]; then
	echo "refusing to run: another benchmark process is on the box" >&2
	echo "$busy" >&2
	exit 3
fi

collect() {
	local dir="$1" mode="$2" scene w output line hex
	for scene in $scenes; do
		for w in $workers; do
			echo "  [$mode] $scene workers=$w" >&2
			output=$("$dir/bin/benchmark" -b="$scene" -w="$w" -r=1 --hash --steps="$STEPS") || {
				echo "benchmark failed for $scene workers=$w ($mode)" >&2; return 1;
			}
			line=$(printf '%s\n' "$output" | grep '^HASH' || true)
			if [[ "$line" == *$'\n'* ]] || [[ ! "$line" =~ ^HASH[[:space:]]([^[:space:]]+)[[:space:]]workers=([0-9]+)[[:space:]]([0-9a-fA-F]{16})$ ]] ||
			   [ "${BASH_REMATCH[1]:-}" != "$scene" ] || [ "${BASH_REMATCH[2]:-}" != "$w" ]; then
				echo "missing, duplicate, or malformed HASH for $scene workers=$w ($mode)" >&2
				return 1
			fi
			hex=$(printf '%s' "${BASH_REMATCH[3]}" | tr 'A-F' 'a-f')
			printf 'HASH %s workers=%s %s %s\n' "$scene" "$w" "$mode" "$hex"
		done
	done
}

# A deterministic negative probe for the failure that once allowed worker-1
# fragments to replace a complete oracle. It needs no compiler or clock.
if [ "$SELF_TEST" -eq 1 ]; then
	probe=$(mktemp -d "$REPO/.oracle-selftest.XXXXXX")
	trap 'rm -rf "$probe"' EXIT
	mkdir -p "$probe/bin"
	cat > "$probe/bin/benchmark" <<'MOCK'
#!/usr/bin/env bash
for arg in "$@"; do
	case "$arg" in -b=*) scene=${arg#-b=} ;; -w=*) worker=${arg#-w=} ;; esac
done
[ "$worker" != 2 ] || exit 9
printf 'HASH %s workers=%s 0123456789abcdef\n' "$scene" "$worker"
MOCK
	chmod +x "$probe/bin/benchmark"
	if partial=$(collect "$probe" narrow); then
		echo "self-test failed: interrupted collection was accepted" >&2; exit 1
	fi
	if validate_matrix "$partial" "$scenes" 1; then
		echo "self-test failed: partial matrix was accepted" >&2; exit 1
	fi
	echo "oracle self-test passed: interrupted collection and partial matrix rejected"
	exit 0
fi

build_one "$REPO/build"
build_one "$REPO/build-ludicrous" -DBOX3D_LUDICROUS_MODE=ON

if [ "$REGEN" -eq 1 ]; then
	r1=$(collect "$REPO/build" narrow) || exit 1
	r2=$(collect "$REPO/build" narrow) || exit 1
	l1=$(collect "$REPO/build-ludicrous" ludicrous) || exit 1
	l2=$(collect "$REPO/build-ludicrous" ludicrous) || exit 1
	[ "$r1" = "$r2" ] || { echo "two narrow scalar runs disagreed" >&2; exit 1; }
	[ "$l1" = "$l2" ] || { echo "two ludicrous scalar runs disagreed" >&2; exit 1; }
	fresh=$(printf '%s\n%s\n' "$r1" "$l1")
	validate_matrix "$fresh" "$scenes" 1 || exit 1
	if [ -n "$SCENES" ]; then
		kept=$(printf '%s\n' "$prior_rows" | awk -v selected="$scenes" 'BEGIN{n=split(selected,a," ");for(i=1;i<=n;i++) drop[a[i]]=1} $1=="HASH" && !($2 in drop)')
	else
		kept=""
	fi
	combined=$(printf '%s\n%s\n' "$fresh" "$kept")
	validate_matrix "$combined" "$all_scenes" 0 || exit 1
	if [ -z "$SCENES" ]; then validate_matrix "$combined" "$all_scenes" 1 || exit 1; fi
	tmp=$(mktemp "$ORACLE.tmp.XXXXXX") || exit 1
	trap 'rm -f "$tmp"' EXIT
	{
		echo "# HASH <scene> workers=<n> <narrow|ludicrous> <16 hex>"
		echo "# scalar build, no BOX3D_AVX512, no BOX3D_NEON. steps=$STEPS."
		echo "# Regenerated only when two runs agree and workers {1,2,4} agree."
		printf '%s\n' "$combined" | grep '^HASH ' | sort -k2,2 -k3,3n -k4,4
	} > "$tmp"
	mv "$tmp" "$ORACLE"
	trap - EXIT
	echo "oracle regenerated: $ORACLE"
	exit 0
fi

validate_matrix "$prior_rows" "$all_scenes" 1 || exit 1
new=$(collect "$REPO/build" narrow) || exit 1
new_lud=$(collect "$REPO/build-ludicrous" ludicrous) || exit 1
got=$(printf '%s\n%s\n' "$new" "$new_lud" | sort)
validate_matrix "$got" "$scenes" 1 || exit 1
want=$(printf '%s\n' "$prior_rows" | awk -v selected="$scenes" 'BEGIN{n=split(selected,a," ");for(i=1;i<=n;i++) keep[a[i]]=1} $1=="HASH" && ($2 in keep)' | sort)
if [ "$got" != "$want" ]; then
	echo "ORACLE MISMATCH" >&2
	diff -u <(printf '%s\n' "$want") <(printf '%s\n' "$got") | head -80 || true
	exit 1
fi
echo "oracle check passed (narrow + ludicrous, workers 1/2/4)"
