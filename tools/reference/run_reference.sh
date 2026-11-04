#!/usr/bin/env bash
# The float box3d reference runner: build both drivers (done by CMake), run a scene
# through each, and compare the two traces against the scene's tolerance header.
#
#   run_reference.sh <rev> <scene...>
#
# <rev> is the vanilla box3d commit the float side was built against (informational
# here; CMake pins it at configure time). The built binaries are located through
# REFERENCE_BIN_DIR, which the `ctest -L reference` wiring sets to the build dir.
# Set it yourself when running by hand.
set -euo pipefail

if [ "$#" -lt 2 ]; then
	echo "usage: run_reference.sh <rev> <scene...>" >&2
	exit 2
fi

rev="$1"
shift

bin_dir="${REFERENCE_BIN_DIR:-$(cd "$(dirname "$0")" && pwd)}"
run_fixed="$bin_dir/run_fixed"
run_float="$bin_dir/run_float"
ref_compare="$bin_dir/ref_compare"

for b in "$run_fixed" "$run_float" "$ref_compare"; do
	if [ ! -x "$b" ]; then
		echo "missing driver: $b (build with -DBOX3D_REFERENCE=ON -DBOX3D_REFERENCE_REV=$rev)" >&2
		exit 2
	fi
done

workdir="$(mktemp -d)"
trap 'rm -rf "$workdir"' EXIT

status=0
for scene in "$@"; do
	name="$(basename "$scene" .scene)"
	fixed_trace="$workdir/$name.fixed.trace"
	float_trace="$workdir/$name.float.trace"

	echo "== reference scene: $name (float rev $rev) =="
	"$run_fixed" "$scene" > "$fixed_trace"
	"$run_float" "$scene" > "$float_trace"
	if ! "$ref_compare" "$scene" "$fixed_trace" "$float_trace"; then
		status=1
	fi
done

exit $status
