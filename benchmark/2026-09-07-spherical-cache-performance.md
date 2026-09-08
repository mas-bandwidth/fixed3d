# Spherical joint geometry reuse

This pass reuses the two rotated point-constraint anchors while body poses are
unchanged. It also skips angular mass inversion when neither spring nor motor
uses it, and skips cone/twist axes when both limits are disabled. Enabling a
feature restores the original calculation on the next prepare stage. The point
matrix, 40-bit inverse scale, 256-bit solve, rounding and stored impulses are
unchanged.

## Determinism contract

Identical initial state and ordered simulation inputs must produce identical
physics regardless of worker count or scheduling. The cache follows that rule:

- Prepare invalidates every joint cache on every step, including after sleeping,
  enabling bodies, changing transforms, locking rotation or changing substeps.
- The existing `positionGeneration` changes at position integration. Reuse is an
  exact integer generation comparison, with no timing or worker-dependent choice.
- A joint belongs to one solver block per stage. Graph colors execute in order;
  overflow joints execute serially. The stage completion/publication atomics
  order cache writes, generation changes and body-pose updates before later stages.
- A miss performs the same two rotations with the same inputs and rounding.

At four substeps with warm starting, this reduces anchor-pair calculations from
12 to 5 per joint per step. It adds 48 bytes to each internal `b3JointSim`
(864 to 912 bytes on arm64), because spherical data shares the joint union. Public
API and recording layouts are unchanged. An experimental full point-matrix cache
added 128 bytes and regressed Rain substantially; it was rejected.

## Measurements

Apple M3 Ultra, macOS 26.6.2, Apple Clang 21, RelWithDebInfo (`-O2`, thin LTO),
NEON enabled, four workers and four substeps. Baseline is PR60, main `8048b78`.
Each trial runs a fresh process using `benchmark -t=4 -w=4 -r=1 -b=SCENE`.
The final sweep uses balanced before/after ordering; Joint Grid, Rain and Large
Pyramid have four observations per binary, and other scenes have two. Every
observation is retained, and end-of-run body/shape/contact/joint/stack counters
match exactly for each scene.

| Scene | Before median (ms) | After median (ms) | Runtime reduction |
|---|---:|---:|---:|
| Joint Grid | 659.094 | 574.872 | **12.8%** |
| Rain | 1427.570 | 1407.155 | 1.4%, inconclusive |

Joint Grid is the supported gain: its final within-binary spread is 1.4% before
and 2.8% after. Earlier independent anchor-cache batches also improve this scene.
Rain's final spread is 13–19%, so its small median difference establishes neither
a gain nor a regression. The cache footprint remains a cost worth watching.

The shared workstation was busy with indexing, file synchronization and other
compilers. No other work from this task ran during timing, but external contention
remained uncontrolled. Several scenes with zero joints are especially unstable:
Large Pyramid's median regresses 14.4%, Convex Pile 19.3%, and Trees50 75.9%; the
Trees50 candidate trials themselves vary from 290 to 728 ms. These observations
are preserved, not removed or presented as proof that unrelated scenes improved.
They do not support an overall performance conclusion from this batch.

The last measured full-suite float-throughput figures remain **44.6% scalar /
47.0% NEON**. This targeted gain does not establish a new ratio. After the tree
ordering repair below, an additional four-process recheck gives Joint Grid
655.978 to 567.754 ms (13.4% reduction). Rain gives 1370.270 to 1455.280 ms
(6.2% regression), with candidate observations 1345.54 and 1565.02 ms; its
performance remains inconclusive. Raw trials from all five batches, including
the rejected matrix cache, are in the
[measurement record](2026-09-07-spherical-cache-performance.json).

## Validation

All 24 suites pass locally in Debug, optimized scalar, NEON, wide positions,
ASan+UBSan and ThreadSanitizer. The existing physics goldens and asteroid-response
tests are unchanged. The standalone runs match **144,000 before/after frame
hashes** across the two position widths, with **63,000 cross-worker comparisons
per build**. Optimized scalar, NEON and Debug output also match byte for byte.
Cross-platform validation is tracked by the pull request's CI.

The expanded checks also exposed a pre-existing scheduling bug: an all-sleeping
world could validate its broad-phase tree while the queued rebuild was still
writing it. The empty-solver path now joins that task before validation. A
regression scheduler deliberately defers the rebuild until `finishTask`, reliably
failing the old Debug build and passing the repair. The active solver path and
physics calculations are unaffected; the tree task already had to finish before
the world step returned.

The permanent `SphericalGeometryTest` uses two chains and a dynamic star: 99
bodies, 96 joints, parallel blocks and serial overflow constraints. It checks
each frame across workers 1–8 and also checks goldens captured from the uncached
solver. The latter catches a consistently stale cache even when every worker
count produces the same wrong result. Numeric fields are hashed explicitly,
including every position bit in the wide build, rather than structure padding.

Coverage includes changing 1/2/4/8 substeps, a zero-duration step, warm-start
toggles, spring/motor/limit toggles, body disable/enable and sleep/wake, rotation
locks and an explicit transform change. Cubes have side length 1 or 250 and unit
density, so the larger bodies have mass 15,625,000. The initial default-density
fixture hit an impulse-range assertion in both old and new Debug solvers when
locking its massive chain; the test density was reduced without changing the
library's arithmetic or assertions.

Separate negative controls deliberately omitted integration invalidation and
step invalidation. Both fail the golden check. Temporary instrumentation confirmed
that joint warm-start blocks executed on every worker index 0–7; this instrumentation
is absent from the shipped library.

The [standalone comparator](../tools/benchmark/spherical_state_hash.c) covers
point-only, spring/motor, limit, runtime-change and overflow scenarios, at both
sizes, workers 1–8, fixed substeps 1/2/4/8 and substeps varying by frame. Each build
produces 72,000 frame hashes. It compares transforms, linear/angular velocities,
awake flags and constraint forces/torques. Run it against preserved before/after
libraries and compare stdout byte for byte:

```sh
cmake -S . -B build-perf -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBOX3D_SAMPLES=OFF -DBOX3D_NEON=ON
cmake --build build-perf
cc -O2 -std=c17 -Iinclude -Iextern/fixed/include \
  tools/benchmark/spherical_state_hash.c build-perf/src/libbox3d.a \
  -lpthread -lm -o spherical_state_hash
./spherical_state_hash > state-hashes.txt
./build-perf/bin/test DeterminismTest
```

For wide positions, configure with `-DBOX3D_LUDICROUS_MODE=ON` and pass the same
definition when compiling the comparator. Wide and narrow builds carry separate
reference hashes because their position representations differ.
