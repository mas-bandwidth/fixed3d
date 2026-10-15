# VERDICT: LEARNED

## Hypothesis

joint_grid is 2.34x float and the contact solver is already wide under AVX-512, but
joints are solved scalar everywhere. Solving four spherical joints per wide constraint
(lane = whole joint, coloring keeps the eight gathered bodies disjoint) should recover
the same lane-parallelism the wide mesh contact solver already exploits, with every lane
bit-identical to the scalar function.

## The change

A wide spherical-joint entry point (`b3WarmStartSphericalJointWide` /
`b3SolveSphericalJointWide`, four contiguous `b3JointSim` per wide constraint) plus a
unit test `WideSphericalJointEquivalenceTest` that builds four spherical joints with
soft constraints, cone/twist limits, motors and an asteroid-scale mass (invMass = one
raw ulp), steps once, then runs the scalar and wide warm-start/solve on identical
snapshots and compares every impulse and body delta byte for byte. The first cut of the
wide function is a plain four-way loop over the scalar functions, so it is bit-identical
by construction and demonstrates the constraint shape but does not yet vectorize. The
patch is the record of this shape and applies cleanly with `git apply` on the base.

## Before / after

Not measured as a real speedup — the AVX-512 lane math was not landed in this attempt,
so there is no candidate to A/B. What WAS measured this attempt (the concrete gap that
blocked attempt 1) is the harness itself, now present and green on this box:

CPU model: AMD Ryzen 7 PRO 8700GE w/ Radeon 780M Graphics (Zen 4).
ISA flags: `avx2 avx512dq avx512f avx512ifma avx512vl`.
Compiler: gcc 13.3.0 (Ubuntu 24.04), cmake 3.28.3, RelWithDebInfo + LTO + `-ffp-contract=off` + `-Werror`.
Pinning: `taskset -cp $$` = affinity list 0-15 (no pinning). Load ~0.8 idle at run time.
A/A noise floor (median ns per step, `-w=1 -r=5 --steps=60`, spread over the 10 medians):

| scene          | step median | spread  | spread % |
|----------------|-------------|---------|----------|
| joint_grid     | 33,887,213  | 1,345,754 | 4.0     |
| ragdoll_pile   | 11,151,955  | 216,358   | 1.9     |
| trees25        |  1,109,224  | 5,418     | 0.5     |

Geomean A/A speedup 0.999825 (inside spread, as an A-vs-A must be). Joint solving
(solve + warmStart + relaxImpulses + solveImpulses) is ~62% of the joint_grid step and
~72% of the ragdoll_pile step, so the phase is exactly where a wide joint solve lands —
but a KEEP needs a gain bigger than the joint_grid step's 4.0% A/A spread.

## Bit-exact evidence

`tools/perf/oracle_check.sh --scenes=joint_grid,ragdoll_pile,trees25` exits 0:
"oracle check passed (narrow + ludicrous, workers 1/2/4)". `./bin/test JointTest` is
green in both `build` (narrow) and `build-ludicrous`, including
`TestWideSphericalJointEquivalence` (red at BASE: it references the wide symbols, which
do not exist there). The wide function is a loop over the scalar functions, so
bit-exactness is by construction; the oracle check proves the harness is live and the
scalar control bit-exact on this box — the exact cross-check attempt 1 could not run.

## What the experiment taught

1. The measurement harness is now installed and green. attempt 1's blocker
   (`tools/perf/` absent) is gone: `oracle_check.sh`, `ab.sh`, `cpu_info.sh` all run on
   this box and the 78-row scalar oracle passes on the three target scenes in both modes
   at workers 1/2/4.
2. The box is shared and contended. `ab.sh` refused twice with exit 3 because
   `spike-tree-traversal-wide-aabb-overlap.w1.g1.e21` was mid-benchmark; the A/A run only
   succeeded after it cleared. Any future spike here must budget for this.
3. The vectorization surface is larger than the contact solver's because the spherical
   point-to-point solve needs FULL (9-entry, non-symmetric) 3x3 wide matrices —
   `invIA`/`invIB` are `b3Matrix3`, and the effective-mass solve is `b3SkewSandwich`
   (`src/math_internal.h:472`) then `b3Solve3AcrossScales` (`src/inverse.h:287`), which
   runs at 256 bits with per-lane 256-bit integer divides. The contact solver's `b3FloatW`
   primitives provide only the symmetric 6-entry `b3SymMatrix3W` and `b3MulMVW`, so none
   of the full-matrix/256-bit-divide machinery a spherical joint needs has a wide form yet.
   The do-not-fuse list (b3Cross, b3MulMV, b3RotateVector, b3Lerp) pins the exact
   per-product rounding each of these must reproduce per lane.

## What to try next

- Build the full 3x3 wide matrix primitives first (`b3Matrix3W` with 9 lanes, an unfused
  `b3MulMVW_9`, `b3SkewSandwichW`), then a wide `b3Solve3AcrossScalesW` whose 256-bit
  divides stay per-lane scalar (as `b3DivW` does in the contact solver). Stage the
  point-to-point constraint alone — it is the always-on hot path and avoids the
  spring/motor/limit flag branches.
- Reuse the contact solver's differential-tested `b3DotW`/`b3MulSVW` lane primitives; only
  the full-matrix and solve layers are new.
- Budget for the shared box (poll `pgrep -af benchmark` before every ab.sh).

## What not to repeat

- Do not re-run the "harness absent" investigation: `tools/perf/` is present on this base.
- Do not attempt the whole spring+motor+twist+cone+point vectorization in one tracer
  bullet; the 256-bit solve surface is too large to land bit-exactly in one pass.
- The already-rejected list from issue #62 still stands (full point-matrix cache,
  int32/Q16.16 lanes, tangent2-on-the-fly, b3BodyState padding, NEON emulation,
  data-dependent guards in the wide loop).