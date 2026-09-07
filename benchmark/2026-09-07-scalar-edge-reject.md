# Scalar hull-edge rejection measurements

The default scalar scan can reject ordinary candidate edge pairs using signed
64-bit raw dots. A conservative magnitude gate proves that products, partial
sums and negation cannot overflow. Larger inputs keep the 128-bit path, and
every survivor uses the original full-precision separation calculation.

Measured on 2026-09-07 with Apple M3 Ultra, arm64 Apple Clang, RelWithDebInfo,
`-O2`, default thin LTO, four workers, `BOX3D_NEON=OFF`. The baseline is corrected
Fixed3D `0b27fe3` from #55; the implementation is `d010dd0`. Both use fixed
`a0fa624d739790844af34499381c55abaa65180f` (v1.4.0). The separate vendor update is
not included in these measurements.

## Results

Convex Pile takes about **9% less time**, from 14.67 to 13.36 seconds by the
median of three runs per binary. Every optimized run in that grouped check was
faster than every baseline run. Smaller differences in other scenes are
inconclusive on this shared workstation; these measurements do not establish an
engine-wide percentage improvement.

| Scene | Before median (ms) | After median (ms) | Runtime change | Runs per binary |
|---|---:|---:|---:|---:|
| convex_pile | 14666.30 | 13361.80 | -8.89% | 3 |
| junkyard | 10988.80 | 11064.90 | +0.69% | 3 |
| many_pyramids | 2010.75 | 2052.40 | +2.07% | 3 |
| rain | 1732.98 | 1713.89 | -1.10% | 3 |
| trees100 | 236.26 | 234.17 | -0.88% | 20 |
| large_world | 31.56 | 31.62 | +0.18% | 3 |

All eleven benchmarks were initially run in before/after/after/before order.
Noise was substantial: one baseline Trees run was almost twice another baseline
run. Six selected scenes were then grouped individually in
before/after/after/before/before/after order. Trees still had an anomalous short
result, so it received a final before/after/after/before check with ten repetitions
per process; that longer check is the Trees row above. All raw trials, including
the noisy and unfavorable observations, are in
[the JSON record](2026-09-07-scalar-edge-reject.json). End-of-run body, shape,
contact, joint and stack counters matched throughout the paired sweeps.

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
preserve the benchmark's CSV outputs. The JSON also fingerprints the measured
executables with SHA256. Results on other architectures and compilers may differ.

## Correctness

All 24 local suites pass in Debug, RelWithDebInfo, wide-position RelWithDebInfo,
NEON RelWithDebInfo, and Debug ASan+UBSan. Existing determinism hashes are
unchanged, including worker counts 1–5 and the narrow/wide references. The new
analytic test covers separating edges at scales 8192, 16384 and 32768, across the
64-bit admission boundary.

A separate corpus compared 6000 hull pairs at six scales from 0.2 through 32768,
each with a fresh and reused cache. All 12000 serialized query results matched
an archive containing the original convex-manifold object exactly. SHA256:
`62467f0ac17eb4f2277db8bd5d686ef167775a3d4bd3b402904c316a3fb1768d`.

Twenty sample scenes (the joint samples, two distance scenes, Box Stack and
Falling Ragdolls) were captured at 120 nominal 60 Hz frames, twice per binary.
All 80 captures reproduce exactly within and across the pre-optimization and
optimized fixed builds. Decoded RGBA hashes and binary fingerprints are in the
JSON. This compares fixed against fixed; float-image equality is not required.

The existing NEON and AVX-512 paths and build defaults are unchanged. The
cross-platform build matrix has not run for the stacked draft: its workflow
currently targets pull requests into `main`.
