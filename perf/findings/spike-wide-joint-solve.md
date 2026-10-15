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
by construction and demonstrates the constraint shape but does not yet vectorize.

## Before / after

Not measured. The perf harness this card depends on (`fixed3d-perf-harness`,
`tools/perf/README.md`, `tools/perf/oracle_check.sh`, `cpu_info.sh`, `ab.sh`) is **not
present in this checkout** — `tools/` holds only `benchmark/`, `capture/`,
`fixed-point/`, `reference/` and `revendor-fixed.sh`. The MEASURED acceptance criterion
(before/after on joint_grid, ragdoll_pile, trees25 with the oracle check) cannot be run,
so there are no numbers and the KEEP rule cannot be evaluated.

Bench: AMD Ryzen 9 7950X3D 16-Core, ISA flags `avx2 avx512dq avx512f avx512vl`
(`grep -o -w -E 'avx2|avx512f|avx512dq|avx512vl' /proc/cpuinfo | sort -u`). Compiler:
gcc (system `/usr/bin/cc`), RelWithDebInfo + LTO + `-ffp-contract=off` + `-Werror`.

## Bit-exact evidence

`./bin/test JointTest` is green in both `build` (narrow) and `build-ludicrous`
(BOX3D_LUDICROUS_MODE), including `TestWideSphericalJointEquivalence`. The wide function
is a loop over the scalar functions, so bit-exactness is by construction rather than by
measurement; the oracle_check.sh harness that would prove it across workers 1/2/4 is
absent. The test is red at BASE: it references `b3WarmStartSphericalJointWide` /
`b3SolveSphericalJointWide`, which do not exist there, so the unchanged base fails to
compile the suite (the red-first line is the implicit declaration of those two symbols).

## What the experiment taught

1. The dependency is missing. `tools/perf/` does not exist in this tree, so the whole
   MEASURED + oracle-check gate is un-runnable. This card cannot reach KEEP on this
   checkout regardless of the solver work.
2. The wide-joint shape is straightforward to stage: the scalar `b3WarmStartSphericalJoint`
   / `b3SolveSphericalJoint` already read only `base->invMassA/B`, `invIA/IB`,
   `fixedRotation`, the `sphericalJoint` fields, and the two `b3BodyState`s, so a
   four-lane gather is a pure transpose of those fields — no cross-lane coupling, exactly
   like the wide mesh contact solver's "lane = whole contact".
3. The hard part is the AVX-512 lane math, not the plumbing. The solve body is ~240 lines
   of spring + motor + twist-limit + cone-limit + point-to-point, each with a matrix
   solve (`b3Solve3AcrossScales`, `b3InvertAcrossScales`, `b3SkewSandwich`) and the
   do-not-fuse primitives (`b3Cross`, `b3MulMV`, `b3RotateVector`, `b3Lerp`). Every one
   of those must be re-expressed per-lane with the exact per-product rounding, and the
   128/256-bit divides stay per-lane scalar (as `b3DivW` does in the contact solver).

## What to try next

- Land `fixed3d-perf-harness` first; without it the measurement gate is a no-op.
- Vectorize the point-to-point constraint (the core, ~40% of the solve) first: it is the
  only part joint_grid and the ragdoll always execute, and its `b3SkewSandwich` +
  `b3Solve3AcrossScales` are the same wide-matrix primitives the contact solver already
  has. Spring/motor/limits are gated behind `enable*` flags and can follow.
- Reuse the contact solver's `b3FloatW` lane primitives (`b3DotW`, `b3MulMVW`,
  `b3CrossW`) rather than writing new ones; they are already differential-tested
  bit-identical.

## What not to repeat

- Do not re-run this spike on a checkout without `tools/perf/`; the measurement is the
  whole point and it is absent.
- Do not attempt the full spring+motor+twist+cone+point vectorization in one pass; the
  bit-exactness surface is too large to land in one tracer bullet. Stage it.
- The already-rejected list from issue #62 still stands (full point-matrix cache,
  int32/Q16.16 lanes, tangent2-on-the-fly, b3BodyState padding, NEON emulation,
  data-dependent guards in the wide loop) — none of it is revisited here.
