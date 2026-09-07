# Contact and joint optimization, 7 September 2026

This pass reduces geometric-mean runtime by **3.3% in scalar mode** and **3.9% with NEON** across eleven benchmark scenes. The resulting float-speed ratios are **44.6% scalar** and **47.0% NEON** (2.24x and 2.13x float runtime). The 50% target remains the next milestone in this comparison.

## Method and full results

Apple M3 Ultra, macOS 26.6.2, Apple Clang 21, arm64, RelWithDebInfo/O2, thin LTO, four workers, four substeps per 60 Hz step, continuous collision enabled. Before: fixed3d `33a9008` (code identical to `8f52c18`). After: `542fa3593373b817feed479cf526389a4c6b8e60`. Both use fixed `2f1ed91`. Float: Box3D `47d7f7c`, default NEON. All executables are preserved with SHA256 fingerprints in the [raw record](2026-09-07-contact-joint-performance.json).

Two runs per scene and binary, before-scalar/after-scalar/before-neon/after-neon/float/float/after-neon/before-neon/after-scalar/before-scalar order, four workers, continuous collision enabled, median runtime. Scenes with an initial within-binary spread over 5% received three additional balanced runs of every binary; their final medians include all five trials, with no discarded observations.

Each observation is a separate process and working directory. No builds, tests or captures from this task ran during timing. Other builds and file-sync processes were observed on the shared workstation. Small differences and short-scene percentages are inconclusive; the float-speed percentage is an estimate under these conditions. Rechecks use temporary working directories outside the synced workspace, and all five binaries run under the same procedure. Geometric means weight each scene equally. Earlier measurements remain in the [integrated library report](2026-09-07-integrated-performance.md); do not add percentages measured against different baselines.

| Scene | Before scalar (ms) | After scalar (ms) | Before NEON (ms) | After NEON (ms) | Float (ms) |
|---|---:|---:|---:|---:|---:|
| convex_pile | 11114.40 | 11088.80 | 7127.92 | 7163.61 | 3769.91 |
| joint_grid | 712.10 | 624.23 | 709.62 | 631.27 | 266.70 |
| junkyard | 9719.38 | 9577.54 | 9308.53 | 8985.01 | 4291.97 |
| large_pyramid | 1682.55 | 1633.42 | 1755.66 | 1637.78 | 571.70 |
| large_world | 26.64 | 25.38 | 27.76 | 25.31 | 13.91 |
| many_pyramids | 1540.37 | 1444.43 | 1553.60 | 1434.87 | 449.60 |
| rain | 1319.88 | 1238.72 | 1277.56 | 1234.97 | 538.50 |
| trees100 | 160.33 | 159.60 | 158.43 | 156.58 | 88.46 |
| trees50 | 197.25 | 198.44 | 195.32 | 195.37 | 101.20 |
| trees25 | 408.53 | 405.90 | 392.68 | 398.57 | 237.14 |
| washer | 14740.20 | 14607.60 | 14577.70 | 14246.40 | 7210.53 |

Physics counters repeat exactly within each binary and match between the four fixed builds in all eleven scenes. The cache increases temporary solver storage, so the stack-byte counter changes deliberately. Float contact counts may differ.

## Retained changes

- Cache exact convex contact separation until the position-integration stage changes geometry. The next solve reuses the preceding relaxation's value. Every prepare invalidates the cache; existing stage barriers publish each generation to workers. Memory cost: 128 bytes per group of four convex constraints.
- Skip relative-orientation calculations for spherical joints when no enabled angular spring or limit uses them. The angular motor uses velocity and remains active independently.
- Replace the two generic skew-matrix products with the same nonzero products, retaining the original rounding, signs and addition order. Multiplication count falls from 54 to 36 per body. Negation remains inside products because round-half-up is not odd at ties.

The inverse scale remains 40 fractional bits and every nonzero joint solve retains its 256-bit calculation. No vendor file changes. A forced-inlining experiment slowed the stacking scenes and is excluded.

## Correctness

The full [CI matrix](https://github.com/mas-bandwidth/fixed3d/actions/runs/34166282099) passed on code commit `542fa35`, including TSan, MSan, wide positions and the static/dynamic sample builds. All 24 local suites pass in Debug, optimized scalar, NEON, wide positions and ASan+UBSan. Existing determinism references remain unchanged, including workers 1-5 and the asteroid-response tests. The new skew-product test matches the generic two-matrix expression across 6,144 cases spanning six scales, tiny and large inverse entries, zero factors and half-quantum signs.

An independent executable linked against each library compares full transform and velocity hashes over 20 scenarios: 36 rotating cubes, sides 1 or 250, workers 1 or 4, substeps 1/2/4/8 or varying per frame, and warm-start toggles. All 4,800 per-frame hashes agree. Output SHA256: `de8a36a2165eb2062329cc432d0357f9709785152d0d5a9fea6d8461bbd82d04`.

The [state-hash harness](../tools/benchmark/contact_state_hash.c) can be linked against either pinned library; compare its complete stdout byte-for-byte.

Twenty sample scenes at 120 frames, twice per binary, give 80 exact decoded RGBA matches before/after. The capture tool's `float` slot contains the preceding fixed binary here. This preserves the preceding pass's float visual comparison, whose dynamic poses differ between engines. Images complement numerical tests and do not prove complete physical equivalence.

## Reproduce

```sh
cmake -S . -B build-perf -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=ON -DBOX3D_BENCHMARKS=ON \
  -DBOX3D_NEON=OFF -DBOX3D_COMPILE_WARNING_AS_ERROR=ON
cmake --build build-perf --parallel 8
./build-perf/bin/benchmark -t=4 -w=4 -r=1 -b=joint_grid
```

On macOS/Linux, build the optional state comparator against each library:

```sh
cc -O2 -std=gnu17 -Iinclude -Iextern/fixed/include \
  tools/benchmark/contact_state_hash.c build-perf/src/libbox3d.a \
  -lm -lpthread -o contact-state-hash
./contact-state-hash > contact-states.txt
```

Build each pinned revision separately. Set `BOX3D_NEON=ON` for ARM narrow-phase SIMD. Omit `-b` for all scenes. Alternate processes and retain every result; the JSON records initial and follow-up trials.
