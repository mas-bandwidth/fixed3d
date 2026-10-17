# VERDICT: ADOPTABLE (bit-identical across gcc and clang and across two runs, JointTest green in all four builds, only the determinism goldens move)

Spike f3dup-0cfc475-spherical-axial-merge-spike, 2026-10-17, on the Linux bench (gcc
15.2.0 / clang 21.1.8, RelWithDebInfo + VALIDATE + Werror). This card ports ONLY upstream
0cfc475's merge of the spherical joint's spring, motor, twist-limit and cone-limit solves
into one block and lands no engine change: the port is recorded as experiment.patch, the
measurement in raw.txt.

## The change (experiment.patch, `git apply --check` clean on BASE)

`b3SolveSphericalJoint` in src/spherical_joint.c merges the four separate rotational solves
(spring, motor, lower/upper twist limit, cone limit) into one block gated on
`fixedRotation == false && (enableSpring || enableMotor || enableTwistLimit || enableConeLimit)`.
One running `cdot = wB - wA` and one accumulated `angularImpulse` are kept across all four.
The inverse-inertia sum `invInertiaSum = iA + iB` is precomputed once; after each sub-solve the
cdot is updated by `cdot += invInertiaSum * impulse` and the impulse is accumulated. The twist
and cone limits read the running `cdot` (lower: `+dot(cdot, axis)`, upper/cone: `-dot(cdot,
axis)`) and update `cdot` and `angularImpulse` with a precomputed `twistResponse =
invInertiaSum * twistJacobian`. At the end the accumulated impulse is applied once:
`wA -= iA * angularImpulse`, `wB += iB * angularImpulse`. Fixed-point spelling follows the house
rules: `b3Fixed` only, `b3FixMul` for products, `b3Dot` (raw-128 fused) for the cdot scalar
projections, `b3InvMulMV`/`b3InvMulSV` for inverse-tensor applications (the Q40 inverse scale),
and the existing `b3FixMax( ..., -oldImpulse )` clamp (equivalent to the old
`max(old+delta, 0); delta = new - old`, bit-identical). The point-to-point solve is untouched:
it keeps `b3SkewSandwich` + `b3Solve3AcrossScales` (upstream's `b3SolvePointConstraint` is
already satisfied here). No other joint file, wide contact solver, or `b3Skew` is touched.

## What moved (DeterminismTest; printed values, ENSURE_GOLDEN neutralized to let every moved
golden print, then reverted)

Every moved value is identical across gcc and clang, across narrow and ludicrous (where
applicable), across worker counts 1-5, and across two independent runs.

| scenario | field | old narrow | new narrow | old ludicrous | new ludicrous |
|---|---|---|---|---|---|
| ragdoll (multithreading/cross-platform) | sleepStep | 408 | **327** | 408 | **327** |
| ragdoll | hash | 0x7D29E209 | **0x5BA7361D** | 0x69572D49 | **0x5BB90A5D** |
| ragdoll-human | sleepStep | 219 | **206** | 219 | **206** |
| ragdoll-human | hash | 0x407D12F3 | **0x9B4E0A07** | 0xD945B8B3 | **0x6BEB1B87** |
| spherical geometry | hash scale 0 | 0xccf165f88a73ebb7 | **0xf63b8ecb25a0abcb** | 0xee343d0d530f22f1 | **0xfd8a13e3b05f3544** |
| spherical geometry | hash scale 1 | 0x2add50ab7852c3ce | **0xc1a50ebf3c370975** | 0x46256d278ff09af2 | **0xd2cfdbaa8c733c66** |
| wave pile | sleepStep / hash | 335 / 0x64609ED9 | unchanged | 335 / 0x22E6F859 | unchanged |
| query spawn | sleepStep / hash | 243 / 0x250D3BFA | unchanged | 243 / 0xC3538B7A | unchanged |
| mesh drop | sleepStep / hash | 284 / 0x784AD80A | unchanged | 284 / 0xE44E6A0E | unchanged |

The perf state hash golden does NOT move: PerfStateHashGoldenTest prints
c4f45e4c6b5534e6 and exits 0 in all four builds.

## Joints and other suites

JointTest exits 0 in all four builds (narrow/ludicrous x gcc/clang) with the patch applied.
DeterminismTest is red only where the goldens above moved (exit 1). PerfStateHashGoldenTest is
green.

## What the design step must decide

1. **The reassociation is real and moves three scenarios, not all.** Only the two ragdoll
   scenarios and the spherical-geometry scenario move; wave pile, query spawn and mesh drop
   are bit-stable. So the merge is observable but narrow in blast radius on this corpus.
2. **The running `cdot` is not bit-witnessed against the actual `wB - wA`.** The velocity
   application uses `b3InvMulMV` (Q40 inverse scale) while the cdot update uses the summed
   `invInertiaSum` in one call; these are algebraically equal but can differ by one raw ULP in
   fixed point. That is faithful to upstream's structure (upstream precomputes
   `invInertiaSum` once) but is the physical approximation the coordinator must bless. A
   future "exact" variant would update `cdot` by the two separate per-body applications instead.
3. **The upper-twist and cone `cdot` signs changed from `dot(wA - wB, axis)` to
   `-dot(cdot, axis)` with `cdot = wB - wA`.** Round-half-up is not odd-symmetric, so this is
   a genuine rounding path change (part of upstream's diff), worth a deliberate accept/reject.

## What not to repeat

- Do not trust the first failing subtest as the whole story: DeterminismTest aborts at the
  first failure (RagdollHumanTest runs first), so run with `ENSURE_GOLDEN` neutralized to
  harvest every moved golden, then revert. This is the documented printed-value procedure.
- Do not edit the goldens in the patch; the spike records them in the finding and raw.txt only.
- Do not reuse `b3MulMV` for the cdot update against the inverse tensor: inverse tensors live at
  Q40 and `b3MulMV` would be a silent factor of 2^24 off. `b3InvMulMV` is the correct primitive.

## Not done

- No engine change is committed (by design of the spike): the port lives as experiment.patch.
- The goldens were not updated in the tree (out of PATHS).
- No perf timing was taken; this is a determinism/behavior spike.
