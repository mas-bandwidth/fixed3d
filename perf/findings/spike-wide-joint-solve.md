# VERDICT: DROP (bit-exact, no gain: the lane = joint shape without lane math is 0.5 percent slower overall and 0.9 to 1.7 percent slower on joint_grid)

Attempt 5, 2026-10-15, on hetzner2 (AVX-512). This attempt finishes the attempt-2 work (head
abc98815) on the bench the card names. It is the first attempt to measure the wide entry point
wired into the solver and to check its hashes against the scalar build.

## Hypothesis

joint_grid is 2.34x float. The contact solver is already wide under AVX-512, but joints are scalar
everywhere. Solving four spherical joints of one graph color per wide constraint (lane = whole
joint; the coloring keeps the eight bodies disjoint) should give joints the lane parallelism the
wide mesh contact solver already has, with every lane bit-identical to the scalar function.

## The change (experiment.patch, applies cleanly with `git apply` on BASE 6183d14)

`b3WarmStartSphericalJointWide` and `b3SolveSphericalJointWide` (src/spherical_joint.c, declared
in src/joint.h) take four contiguous `b3JointSim`. In the AVX-512 build (`B3_SIMD_AVX512`),
`b3WarmStartJointsTask` and `b3SolveJointsTask` in src/solver.c now send every run of four
consecutive spherical joints in a color block through them. The force and torque threshold checks
still run per joint, after the quad. Overflow joints and the scalar build keep the scalar path.
This tracer bullet keeps the per-lane arithmetic scalar: each wide function is a four-way loop over
the scalar function. So the measurement prices the lane = joint dispatch shape on its own, without
lane math. The unit test is `TestWideSphericalJointEquivalence`, a subtest of `JointTest` in
test/test_joint.c and already registered through `RUN_SUBTEST`. It runs 10,000 seeded quadruples
from a fixed xorshift with no clock. Each seed randomizes body velocities and delta positions, the
accumulated impulses, the spring/motor/cone/twist flags and useBias, and the bodies include one at
asteroid scale (invMass is one raw ulp). The test runs scalar and wide on byte copies and compares
the whole `b3JointSim` and every body state with memcmp. At BASE it is red because it does not
compile: the wide symbols do not exist there. A mutation (wide loop `i < 4` changed to `i < 3`)
turns it red, so the test catches a dropped lane.

## Bench

- CPU: AMD Ryzen 9 7950X3D 16-Core. ISA: avx2 avx512dq avx512f avx512ifma avx512vl. 32 threads.
- Compiler: gcc 13.3.0 (Ubuntu 24.04), cmake 3.28.3.
- Build: RelWithDebInfo, LTO on (the default), -ffp-contract=off, -Werror, BOX3D_VALIDATE=ON,
  BOX3D_BENCHMARKS=ON. AVX-512 dirs add BOX3D_AVX512=ON.
- `taskset -cp $$` printed 0-31 (no pinning).
- Load average: 2.15 before, 1.71 between the two A/B runs, 1.43 after. The box was shared with
  LLM-harness agents, and no other benchmark process was running (`pgrep -af bin/benchmark` was
  empty before the A/B).
- Workers: 1, 2 and 4. The A/B used `tools/perf/ab.sh`, interleaved, `-r=7 --steps=120`.

## Before / after (whole step, ns; median of 7, spread = max-min of the 7)

AVX-512 build: A is base/build-avx (unchanged), B is cand/build-avx (wide entry wired).

| scene        | w | A median   | A spread  | B median   | B spread  | speedup |
|--------------|---|------------|-----------|------------|-----------|---------|
| joint_grid   | 1 | 30,194,153 |   389,406 | 30,422,092 | 2,184,595 | 0.9925  |
| joint_grid   | 2 | 15,322,465 |   291,383 | 15,587,243 |   237,333 | 0.9830  |
| joint_grid   | 4 |  8,088,256 |   270,694 |  8,163,185 |    78,530 | 0.9908  |
| ragdoll_pile | 1 | 12,491,673 |    93,562 | 12,474,513 |   149,435 | 1.0014  |
| ragdoll_pile | 2 |  6,486,102 |   119,586 |  6,447,840 |   162,435 | 1.0059  |
| ragdoll_pile | 4 |  3,522,434 |   359,057 |  3,475,942 |   142,921 | 1.0134  |
| trees25      | 1 |  1,518,725 |    22,989 |  1,522,540 |    41,111 | 0.9975  |
| trees25      | 2 |    934,447 |   168,605 |  1,002,736 |   113,780 | 0.9319  |
| trees25      | 4 |    774,160 |   121,024 |    741,247 |   131,968 | 1.0444  |

Geomean whole-step speedup 0.9952. The ab.sh exit was 0, so the final HASH lines of A and B were
identical.

The joint phases on joint_grid were all below 1. warmStart: 0.985 / 0.974 / 0.974. At w=2 the
42k ns delta is outside both spreads (29k and 22k). relaxImpulses: 0.988 / 0.984 / 0.988.
solveImpulses: 0.991 / 0.984 / 0.991. ragdoll_pile, which has fewer runs of four spherical joints
in a color, was flat to slightly positive and inside its spread. trees25 has no spherical joints,
and its w=2 and w=4 rows are noise: they move in opposite directions by more than 4 percent, and
the spreads are as large as the deltas.

Scalar control: A is base/build, B is cand/build, where the wide path is not compiled. Geomean
1.0012. Every step row is inside its spread: joint_grid 1.0019 / 0.9965 / 1.0011, ragdoll_pile
0.9976 / 0.9973 / 0.9809, trees25 1.0107 / 1.0495 / 0.9769. That is the noise floor on this box
for these runs.

The full per-phase tables (min, median and spread per scene, phase and worker count) for both
A/B runs are in raw.txt.

## Bit-exact evidence

- **Same-commit hash matrix: 78 of 78 rows match.** That is all 13 scenes x workers 1/2/4 x narrow
  and ludicrous at 120 steps. The candidate AVX-512 build, with the wide entry wired, gives the
  same hash as the candidate scalar build in every row, and in the narrow rows it also matches the
  unchanged base AVX-512 build (raw.txt, `ROW` lines). Workers 1, 2 and 4 agree in every row.
- **Scalar `tools/perf/oracle_check.sh` on the candidate:** it passed (narrow + ludicrous, workers
  1/2/4) on junkyard, large_pyramid, large_world, trees25, trees50, trees100, washer,
  many_pyramids, convex_pile and joint_grid. It failed (ORACLE MISMATCH) on bodies_20k,
  ragdoll_pile and rain. **That failure is already red at BASE:** the same check on the unchanged
  base tree gives a byte-identical diff (bodies_20k ed59e6aff81bcd26, ragdoll_pile
  42fa9c6438e3527f, rain f96c9e008d59a375 against the frozen rows), so the frozen
  `scalar-oracle.txt` is stale for those three scenes at 6183d14. The scalar build does not
  compile the wide path at all. The arbiter for the wide path is the same-commit matrix above, and
  it matches everywhere, including ragdoll_pile, which exercises the wide path.
- **`./bin/test`:** green in build (narrow) and build-ludicrous: "All Fixed3D tests passed".
  In build-avx and build-avx-lud, 25 of 26 suites pass when run one by one. The exception is
  `AllocatorTest` (`AllocatorWidenedSignatureTest`, condition `!g_alloc.badAlignment`), which is
  **red at BASE** in the unchanged base AVX-512 build too. test/test_allocator.c is not touched by
  this patch. The runner stops at the first failing suite, which is why the AVX suites were run
  one at a time. `JointTest`, including the 10,000-quadruple equivalence test, is green in all
  four builds.

Process note: for about 30 seconds my first hash-only oracle pass and my second hash-collection
pass overlapped on the box. Hashes do not depend on timing, so the evidence still holds, and both
A/B timing runs ran with nothing else benchmarking.

## What the experiment taught

1. **The dispatch shape alone costs about 1 percent on joint_grid and buys nothing.** Grouping four
   joints per call, with a type check over four `b3JointSim` and the threshold checks moved out of
   the loop, is measurably slower in warmStart (2.5 percent at w=2, beyond the spread). The rest is
   at or inside noise. Any gain has to come from lane arithmetic. Lane = joint batching gives no
   cache or branch-prediction win on its own.
2. **The same-commit matrix is the gate that works on this base. The frozen oracle is not.** Three
   scenes (bodies_20k, ragdoll_pile, rain) moved when the base advanced to 6183d14, and the oracle
   was not regenerated. Every spike on this stream that runs `oracle_check.sh` on those scenes will
   see a MISMATCH that is not its own.
3. **The AVX-512 build is slower than scalar on ragdoll_pile on this box.** Base AVX-512 had a
   12.49 ms median against 11.40 ms for base scalar at w=1. This comes from two separate A/B runs,
   so it is indicative, not interleaved. joint_grid goes the other way (30.19 vs 32.20 ms). That is
   worth its own card.
4. The surface a real wide spherical solve needs is unchanged from attempt 2. It needs full 9-entry
   `b3Matrix3W` for invIA/invIB, `b3SkewSandwichW`, and a wide `b3Solve3AcrossScales`. The 256-bit
   divides in that solve stay scalar per lane, and `b3InvMulW` is already scalar per lane in the
   contact solver. The vectorizable share is the unfused crosses and products, while the 128- and
   256-bit inverse-scale work stays scalar. So the realistic ceiling for this candidate is well
   below the 1.5x phase gain the keep rule asks for. The candidate should be costed before anyone
   builds it.

## What to try next

- Regenerate `tools/perf/scalar-oracle.txt` for bodies_20k, ragdoll_pile and rain at the current
  base (two agreeing scalar runs, per the README), as its own small card. Until then, check bit
  identity on this stream with the same-commit scalar vs SIMD matrix.
- Before writing any wide joint lane math, profile the scalar spherical point solve on joint_grid
  and split it into the 256-bit `b3Solve3AcrossScales` share and the share that can be laned. If
  the solve-3 share dominates, take issue #62 candidate 3 instead: an exact common-case shortcut
  for `b3Solve3AcrossScales`, with proven range checks and the 256-bit fallback. It helps the
  scalar path everywhere and avoids the lane plumbing.
- Look into why the AVX-512 build is slower than scalar on ragdoll_pile (indicative 9 percent).

## What not to repeat

- Do not build the lane = joint dispatch without lane arithmetic and expect a gain. It was measured
  here at 0.995 geomean, with joint_grid 0.983 to 0.993.
- Do not read an `oracle_check.sh` MISMATCH on bodies_20k, ragdoll_pile or rain at 6183d14 as a
  spike's divergence without running the same check on the unchanged base.
- The already-rejected list from issue #62 still stands: the full point-matrix cache, int32 or
  Q16.16 lanes, tangent2 recomputed on the fly, b3BodyState padding, NEON emulation, and
  data-dependent guards in the wide loop.

## Attempt history

- Attempt 1: tools/perf was absent, so nothing could be measured.
- Attempt 2 (AMD Ryzen 7 PRO 8700GE): wrote the wide entry point shape and the first equivalence
  test, and measured an A/A noise floor. The wide entry point was not wired into the solver.
- Attempts 3 and 4 ran on the wrong host (a Mac, then vision with avx2 only). No measurement.
- Attempt 5 (this one): wired the entry point into the solver, extended the test to 10,000 seeded
  quadruples, ran the A/B and the hash gates on hetzner2, and wrote this finding.

## Not done

- No lane arithmetic was written, by design of this tracer bullet. The per-lane math is still
  scalar.
- The frozen oracle was not regenerated (out of this card's PATHS).
- The pre-existing AVX-512 `AllocatorTest` failure was not investigated.
