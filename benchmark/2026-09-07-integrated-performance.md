# Fixed3D integrated performance, 7 September 2026

The new fixed library reduces geometric-mean runtime by **15.0%** in the independent library sweep. The exact-zero joint shortcut reduces Joint Grid from 911.2 to 741.2 ms (**18.7% less runtime**) in three grouped trials. The preceding scalar edge rejection change separately reduced Convex Pile by 11.3% on the old library.

The final suite measures **2.36× float runtime in scalar mode** and **2.25× with NEON**, equivalent to 42.5% and 44.4% of float throughput. Final scalar runtime is about 16.2% lower than the old-library scalar build that already includes the collision optimization. These percentages describe different comparisons and must not be added.

## Revisions and method

Apple M3 Ultra, macOS 26.6.2, Apple Clang 21.0.0, arm64, RelWithDebInfo/O2, thin LTO, four workers; scalar defaults and optional NEON; continuous collision on.

- Old library scalar: `044aee0`, fixed `a0fa624` (v1.4.0), including the numerical repairs and scalar edge optimization.
- New library before joint shortcut: `48f8acf`, fixed `2f1ed91`.
- Final scalar/NEON: `8f52c1898dc0c26c009b9f5c16554587d1448c26`, fixed `2f1ed91`.
- Float Box3D: `47d7f7cc7e091142c08d11dc7d2e493c5d34f536`, default NEON.

Two runs per scene and binary, old/pre-solver/final/neon/float/float/neon/final/pre-solver/old order, four workers, continuous collision enabled, median runtime. Scenes with an initial within-binary spread over 5% received three additional balanced runs of every binary; their final medians include all five trials, with no discarded observations. Separate processes and working directories preserve each run. No local builds, tests or captures ran during timing. The raw JSON preserves every observation, executable SHA256, final counters and any rechecks. This is a shared workstation; small differences and short-scene percentages are inconclusive.

| Scene | Old library scalar (ms) | New library before joint shortcut (ms) | Final scalar (ms) | Final NEON (ms) | Float (ms) |
|---|---:|---:|---:|---:|---:|
| convex_pile | 12672.90 | 10784.80 | 10828.70 | 7020.12 | 3574.49 |
| joint_grid | 901.40 | 906.09 | 719.68 | 718.97 | 267.33 |
| junkyard | 9477.52 | 8365.64 | 8382.29 | 7953.26 | 3530.05 |
| large_pyramid | 1653.44 | 1590.32 | 1550.68 | 1597.03 | 468.24 |
| large_world | 24.90 | 24.20 | 24.29 | 24.70 | 12.44 |
| many_pyramids | 1622.74 | 1557.23 | 1570.25 | 1588.52 | 472.38 |
| rain | 1428.48 | 1420.21 | 1376.59 | 1351.70 | 566.12 |
| trees100 | 193.90 | 150.30 | 151.64 | 146.94 | 84.66 |
| trees50 | 316.25 | 209.64 | 210.41 | 208.26 | 106.72 |
| trees25 | 664.60 | 395.05 | 399.38 | 396.91 | 233.31 |
| washer | 15316.85 | 13776.50 | 13694.05 | 13806.20 | 6770.39 |

End-of-run counters repeat exactly within each binary. The new-library scalar, final scalar and final NEON counters agree for all eleven scenes. The library update changes some contacts and trajectories; its speedup includes both faster arithmetic and changed simulation work. The joint shortcut preserves numerical behavior.

## Reproduce

```sh
cmake -S . -B build-perf -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=ON -DBOX3D_BENCHMARKS=ON \
  -DBOX3D_NEON=OFF -DBOX3D_COMPILE_WARNING_AS_ERROR=ON
cmake --build build-perf --parallel 8
./build-perf/bin/benchmark -t=4 -w=4 -r=1 -b=joint_grid
```

Build each pinned revision separately. Set `BOX3D_NEON=ON` for the ARM SIMD build. Omit `-b` for all scenes. Use separate working directories, alternate binaries, and retain all runs rather than choosing the fastest result.

## Correctness and range

All 24 local suites pass in Debug, optimized scalar, NEON, wide positions and ASan+UBSan. Existing determinism references are unchanged by the joint shortcut, including workers 1–5. New analytic cases cover singular systems, inverse values of only a few quanta, large intermediates requiring 256 bits, and positive/negative determinants. Geometry-based checks verify mass-proportional off-centre impulses on 1-, 10- and 250-unit cubes against quantization-derived bounds.

Only an exactly zero right-hand side skips matrix construction/solving. Accumulated impulses, damping and force caps still update. Every nonzero solve keeps the original 256-bit calculation and 40-bit inverse scale. The comment in `src/inverse.h` records Space Game's asteroid failure and this range requirement. The CI wide-position job now selects the actual `BOX3D_LUDICROUS_MODE` option.

Twenty sample scenes at 120 frames, twice per binary, give 80 exactly matching decoded RGBA captures before/after the joint shortcut. A separate 80-capture run against float also repeats exactly within each binary; cross-engine pixels differ in all 20 scenes, as expected. Gear Lift and Falling Ragdolls were visually inspected: their layouts agree, with differing dynamic poses. These captures complement the analytic tests; image differences are not used as a numerical tolerance or proof of complete physics equivalence.

The nonzero solver body is byte-identical to the preceding implementation after the early zero return; its source hash is recorded in the JSON.

## Profiling and experiments

The new-library Convex Pile profile still concentrates in hull edge and face searches; Rain concentrates in joint solving and matrix products. Sampled solver inputs showed many exactly zero right-hand sides in Joint Grid. Avoiding both its matrix construction and solve produced the retained improvement.

A bounded native-128 solve was tested separately. Joint Grid was effectively the same as the simple zero shortcut; Rain improved only about 2% in the recheck. That arithmetic variant was excluded. A support-search variant using actual-vertex bounds improved Convex Pile/Junkyard only around 1–1.5% and remains an unshipped experiment. Their measurements remain in the JSON. Existing NEON acceleration predates this work; enabling it is not a new implementation change.

The [earlier collision report](2026-09-07-scalar-edge-reject.md) isolates that optimization. [Raw measurements and comparisons](2026-09-07-integrated-performance.json) contain both sweeps and the experiment records.
