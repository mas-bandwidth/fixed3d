# VERDICT: LEARNED (bit-identical across compilers and builds; joints green; but the golden table is incomplete and the ragdoll-human scene no longer settles)

This spike ports only upstream 0cfc475's merged axial solve in `b3SolveRevoluteJoint`
(src/revolute_joint.c): the spring, motor and limit solves are merged into one block guarded
by `fixedRotation == false && (enableSpring || enableMotor || enableLimit)` that precomputes the
axis inverse-inertia response once (`gA = iA*axis`, `gB = iB*axis`, `k = dot(axis, gA+gB)`),
keeps a running `axialCdot` updated after each sub-sove by `axialCdot += k*deltaImpulse`, and
applies the accumulated `axialImpulse` to `wA`/`wB` once at the end. The fixed-point spelling
follows CLAUDE.md: `b3FixMul` for products, `b3Dot` raw-128 form for the running cdot, the
existing `b3FixMax`/`b3FixClamp` impulse caps, and `b3InvMulSV` for the final application. The
collinearity and point-to-point solves are untouched (fixed3d's `b3SkewSandwich` +
`b3Solve3AcrossScales` already satisfies upstream's `b3SolvePointConstraint`; `b3Skew` and the
wide contact solver untouched). Joint side and the contact solver are unchanged. This is a pure
reassociation; it is expected to move goldens (upstream moved every golden in both precisions).

## Golden table

The full table could not be completed within the wall. `DeterminismTest` aborts at the first
`ENSURE` failure, so the unmodified suite only surfaces the first subtest (`RagdollHumanTest`,
which runs first). A neutered build (ENSURE_GOLDEN → no-op, measurement-only, not committed) was
built but its run did not finish before the wall; the values below are from the aborting runs and
therefore cover only the one scenario that fails first, in all four builds.

| scenario | field | old (BASE) | new (all 4 builds) | identical across gcc/clang & narrow/ludicrous |
|---|---|---|---|---|
| ragdoll-human (RagdollHumanTest) | sleepStep | 219 | **0** | yes, all 4 |
| ragdoll-human | hash | 0xD945B8B3 narrow / 0x407D12F3 ludicrous | **0x00000000** | yes, all 4 |
| ragdoll | sleepStep/hash | 408 / 0x69572D49 | not measured (subtest behind the abort) | - |
| wave pile | sleepStep/hash | 335 / 0x64609ED9 | not measured | - |
| query spawn | sleepStep/hash | 243 / 0x250D3BFA | not measured | - |
| mesh drop | sleepStep/hash | 284 / 0x784AD80A | not measured | - |

Not measured, and this is the gap the design step must close before adopting: the wave-pile,
query-spawn, mesh-drop and primary ragdoll goldens. The measurement harness to close it is the
CLAUDE.md printed-value procedure (force every pin impossible so the printf fires, neuter
`ENSURE_GOLDEN` so no abort, run all workers 1-5, confirm two agreeing runs).

## Determinism / repeatability evidence

- `JointTest` exits 0 in all four builds (gcc and clang x narrow and ludicrous).
- `PerfStateHashGoldenTest` exits 0 in all four builds.
- The one visible DeterminismTest value (ragdoll-human sleepStep=0, hash=0x00000000) is
  identical in all four builds, and it reproduced on two separate runs of the same build. No
  nondeterminism was observed.

## What the design step must decide

- Whether `RagdollHumanTest` moving to sleepStep=0 — the human no longer settles within 800
  steps, hash 0 — is an acceptable consequence of the reassociation or a rejection. Upstream's
  float commit also moved every golden, but a never-settling fixed ragdoll is a qualitative
  outcome, not a one-quantum golden shift, and it needs an attribution pass (spring vs motor vs
  limit hunk, bisected) before landing.
- Whether the merged axial solve's reordering is worth the golden churn at all, given it is a
  pure reassociation with no arithmetic change: the benefit is fewer matrix-vector multiplies
  (one `iA*axis`/`iB*axis` and one accumulate instead of one per sub-solve) and one impulse
  application instead of up to four.
- Per CLAUDE.md, reassociations are not free, and the full multi-scene golden table (both
  precisions, all workers, two runs each) must be harvested and shown bit-identical before any
  ADOPTABLE verdict.

## What not to repeat

- Do not run `./bin/test DeterminismTest` expecting to see every moved golden: `ENSURE` aborts
  at the first failing subtest, so only the first scenario (RagdollHumanTest) prints. Collect
  the table with the neutered printed-value procedure.
- The fixed `b3SkewSandwich` + `b3Solve3AcrossScales` point-to-point solve is upstream's
  `b3SolvePointConstraint`; do not rewrite it. Only the axial spring/motor/limit merge belongs
  to this card.

## Not done

- Full golden table (wave pile, query spawn, mesh drop, primary ragdoll, both precisions).
- Second full run of every build for cross-run confirmation (only the one visible value was
  run twice).
- Per-hunk attribution of the ragdoll-human sleepStep=0 outcome.
