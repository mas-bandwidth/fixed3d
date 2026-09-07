# Scalar hull-edge rejection measurements

The default scalar scan rejects ordinary candidate edge pairs using signed
64-bit raw dots. A conservative magnitude gate proves that products, partial
sums and negation cannot overflow. Its bounds come from actual hull vertices
and face normals, avoiding older vendored AABB transforms that can round inward.
Larger inputs keep the 128-bit path; every survivor uses the original
full-precision separation calculation, in the same order.

Measured on 2026-09-07 with Apple M3 Ultra, arm64 Apple Clang, RelWithDebInfo,
`-O2`, default thin LTO, four workers, `BOX3D_NEON=OFF`. The baseline is corrected
Fixed3D `0b27fe3` from #55; the final implementation is `bb6f23c`. Both use fixed
`a0fa624d739790844af34499381c55abaa65180f` (v1.4.0). The separate vendor update is
not included in these measurements.

## Results

Convex Pile takes **11.3% less time**, from 14.27 to 12.66
seconds by the median of three runs per binary. Every optimized run in this
check was faster than every baseline run. Smaller differences in other scenes
remain inconclusive on the shared workstation. These measurements do not
establish an engine-wide percentage improvement.

| Scene | Before median (ms) | After median (ms) | Runtime change |
|---|---:|---:|---:|
| convex_pile | 14274.20 | 12655.60 | -11.34% |
| junkyard | 10289.80 | 10167.60 | -1.19% |
| many_pyramids | 1704.81 | 1715.95 | +0.65% |
| rain | 1385.23 | 1381.57 | -0.26% |
| trees100 | 195.58 | 190.76 | -2.47% |
| large_world | 25.17 | 25.35 | +0.73% |

Each final scene used before/after/after/before/before/after ordering, with one
run per process. End-of-run body, shape, contact, joint and stack counters match.
[The JSON record](2026-09-07-scalar-edge-reject.json) contains every final trial
and executable fingerprints, plus the earlier candidate's full eleven-scene
sweep, grouped recheck, and longer Trees check. Those earlier measurements are
explicitly separated: the final vertex-bound implementation was retested.
Several earlier short runs were noisy or unfavorable, and remain in the record.

## Reproduction

Build separate copies of the baseline and optimized revision with the same
configuration, preserving both executables. Keep other builds/tests idle while
measuring, then alternate binaries with:

```sh
cmake -S . -B build-perf -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=ON -DBOX3D_BENCHMARKS=ON \
  -DBOX3D_NEON=OFF -DBOX3D_COMPILE_WARNING_AS_ERROR=ON
cmake --build build-perf --parallel 8
./build-perf/bin/benchmark -t=4 -w=4 -b=convex_pile -r=1
```

Omit `-b` for all scenes. Use a separate working directory for each process to
preserve CSV outputs. Results on other architectures and compilers may differ.

## Correctness

All 24 local suites pass in Debug, RelWithDebInfo, wide-position RelWithDebInfo,
NEON RelWithDebInfo, and Debug ASan+UBSan. Existing determinism hashes are
unchanged, including worker counts 1–5 and narrow/wide references. The new
analytic test covers separating edges at scales 8192, 16384 and 32768, across the
64-bit admission boundary.

A separate corpus compared 6000 hull pairs at six scales from 0.2 through 32768,
each with a fresh and reused cache. All 12000 serialized query results matched
an archive containing the original convex-manifold object exactly. SHA256:
`62467f0ac17eb4f2277db8bd5d686ef167775a3d4bd3b402904c316a3fb1768d`.

Twenty sample scenes (joint samples, two distance scenes, Box Stack and Falling
Ragdolls) were captured at 120 nominal 60 Hz frames, twice per binary. All 80
captures reproduce exactly within and across the pre-optimization and final
fixed builds. Decoded RGBA hashes and binary fingerprints are in the JSON.
Both binaries are fixed builds; float-image equality is not required.

The existing NEON and AVX-512 paths and build defaults are unchanged. The
cross-platform matrix has not run for the stacked draft: its workflow currently
targets pull requests into `main`.
