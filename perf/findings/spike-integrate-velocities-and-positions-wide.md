VERDICT: LEARNED (not measured) — the measurement gate cannot run on this attempt.

## Why not KEEP

This card's KEEP rule requires (1) MEASURED before/after numbers from the
`fixed3d-perf-harness` on an AVX-512 **Linux** bench, (2) BIT-EXACT oracle
hashes from `tools/perf/oracle_check.sh`, and (3) a before/after table from
`ab.sh`. None of those three things is available in this attempt:

- **No Linux bench.** The run landed on the working Mac (`Darwin x86_64`,
  Intel Xeon W-2191B, Skylake-X). The card's TOOLCHAIN is explicit ("a Linux
  bench ... never the working Mac") and STEP 1 repeats it ("Build and test
  only on the Linux bench JOB.md names, never the working Mac"). JOB.md names
  no Linux bench; the checkout *is* this Mac.
- **The harness is absent.** `DEPENDS-ON: fixed3d-perf-harness` has not
  landed. There is no `tools/perf/` directory, no `oracle_check.sh`, no
  `ab.sh`, no `cpu_info.sh`, and no `bodies_20k` scene anywhere in the tree.
  The three ACME deliverables (1)/(2)/(3) all route through that harness.

So a KEEP verdict is unreachable by construction on this attempt, and a real
before/after table cannot be produced. What this spike *can* deliver is the
analysis the card asked for first ("read the integrate functions and write
down their exact operation order and rounding points") plus the pre-existing
profile evidence that strongly predicts the outcome the card itself flagged.

## Predicting the outcome: the phases are sub-5% of the step

The card states the escape hatch explicitly: "If the first measurement shows
the phases are under 5 percent of the step on every scene, the verdict is
LEARNED/DROP and you stop there." Every committed profile agrees that these
phases are small. `benchmark/apple_m3_ultra_fixed/profiles/profile_*.txt`
(large_pyramid) attributes the *entire* `b3World_Step` time to
`b3SolveContacts_Convex`, `b3RotateVectorW`, `b3CrossW`, `b3MulMV3W`,
`b3MakeMatrixFromQuatW`, `b3RelVelocityW` — the contact solver and the wide
prepare math. `b3IntegrateVelocitiesTask` / `b3IntegratePositionsTask` do not
appear in the hot path at all. The bodies are per-body, independent and scalar
everywhere, but their work is a few multiplies, one division-free Pade damping
step, and a 2-iteration `B3_GYROSCOPIC_ITERATIONS` Newton solve per body — a
small constant factor against the O(contacts) wide solve that dominates
`bodies_20k`-class scenes. This is the "under 5 percent" case: wide-ifying the
integrate loops would hide a few percent in the solve-dominated scenes and
cannot reach the KEEP threshold (geomean >= 3%).

## The change that was going to be tried (for the record)

Body integration in the AVX-512 lanes: process four contiguous `b3BodyState` /
`b3BodySim` records per iteration in `b3IntegrateVelocitiesTask` and
`b3IntegratePositionsTask`, behind the existing `B3_SIMD_AVX512` gate, with the
scalar loop kept verbatim in the `#else` (the CLAUDE.md "scalar path stays as
the control" pattern already used across `contact_solver.c`). Each per-lane
rounding point has to reproduce the scalar operation exactly — see below for
where those rounding points are, because this is the hard part and it is what
makes a hypothetical wide port of *these* two functions more delicate than the
wider solver work that already landed.

## Operation order and rounding points (read from src/solver.c)

`b3IntegrateVelocitiesTask` (solver.c:67-178), per body:
1. `linearDamping = b3FixDiv(1, 1 + b3FixMul(h, sim->linearDamping))` — a Pade
   step; the `b3FixMul` rounds half-up, then `b3FixDiv` truncates. Division is
   NOT in the "division-free multiply chain" the card's brief suggests — the
   damping scale is one truncating divide per body per axis.
2. `angularDamping` likewise (one `b3FixDiv`).
3. `gravityScale` is a conditional select (`invMass > 0`), zero for kinematic.
4. `linearVelocityDelta = b3Add(b3InvMulSV(b3FixMul(h, invMass), force),
   b3MulSV(b3FixMul(h, gravityScale), gravity))` — two `b3FixMul` scale
   roundings, one `b3InvMulSV` (invMass hits the inverse scale, per the
   comment), one `b3MulSV`, one `b3Add`.
5. `v = b3MulAdd(linearVelocityDelta, linearDamping, v)` — fused multiply-add,
   ONE rounding.
6. `angularVelocityDelta = b3MulSV(h, b3InvMulMV(invInertiaWorld, torque))` —
   `b3InvMulMV` is one of the deliberately-NOT-fused primitives (rounds per
   product), then one `b3MulSV`.
7. `w = b3MulAdd(angularVelocityDelta, angularDamping, w)` — one rounding.
8. Gyroscopic term (the expensive part): `q = b3MulQuat(deltaRotation, q0)`,
   `omega1 = b3InvRotateVector(q, w)` (**do-not-fuse** b3RotateVector family),
   then `B3_GYROSCOPIC_ITERATIONS` Newton steps. Each iteration builds the
   residual `b` with 9 unfused `b3FixMul` products + `b3FixMul(h, ...)` on the
   three `b3Cross`-like terms (the cross is spelled element-wise, not fused),
   builds the 9-entry Jacobian, and solves `omega2 -= b3Solve3(J, b)` — the
   `b3Solve3` divide is 128-bit exact. So the gyro loop is ~36 `b3FixMul`
   roundings + one 128-bit `b3Solve3` per iteration, per body, in local
   coordinates. `w = b3RotateVector(q, omega2)` (**do-not-fuse**) closes it.

`b3IntegratePositionsTask` (solver.c:180-231), per body:
1. Six motion-lock selects (pure `?:`, no arithmetic).
2. Linear clamp: `b3Dot(v,v)` (fused raw-128, one rounding) compared to
   `maxLinearSpeedSquared`; on cap, `b3FixDiv(maxLinearSpeed, b3Length(v))`
   (divide-last, `b3Length` is exact raw-128) then `b3MulSV` and a flag set.
3. Angular clamp: same, gated on `b3_allowFastRotation == 0`.
4. `deltaPosition = b3MulAdd(deltaPosition, h, v)` — one rounding.
5. `deltaRotation = b3IntegrateRotation(deltaRotation, b3MulSV(h, w))` — the
   quaternion normalize inside uses the exact raw-128 sum the card names.

The wide port would have to reproduce every one of those round-half-up
`b3FixMul`s, the truncating `b3FixDiv`s, and the two do-not-fuse rotations
per body per lane. It is entirely doable in the same style as the wide
contact solver, but none of this has been measured on AVX-512 hardware, and
there is no reason to believe it repays the effort given the profile evidence
above.

## Bit-exact evidence

None could be produced: `tools/perf/oracle_check.sh` and the `bodies_20k` /
`rain` / `many_pyramids` harness scenes do not exist in this tree, and there is
no AVX-512 Linux bench in reach. `./bin/test` was not run here (the working
Mac is off-limits for this card's build; the tree's own CLAUDE.md puts the
canonical narrowing/wide bounds on Zen 4 AVX-512 and M3, both Linux/vm style).

## What to try next

Re-run this spike once `fixed3d-perf-harness` has landed and on the real
AVX-512 Linux bench (the `space` box is AMD EPYC 9124 Zen 4 with full AVX-512).
Then the honest first pass is: measure the `integrateVelocities` +
`integratePositions` phase fractions on `bodies_20k`, `rain`, `many_pyramids`
via the harness JSON. If (as predicted) they are under ~5% of the step on
every scene, stop with LEARNED/DROP and do not wide-ify the loops.

## What not to repeat

- Do not wide-ify `b3IntegrateVelocities`/`b3IntegratePositions` before the
  phase-fraction measurement exists — the profile evidence says the returns
  will be well under the KEEP threshold.
- Do not attempt this on the working Mac; the ISA check, the bit-exact
  oracle, and the A/B table all require the Linux AVX-512 bench and the
  harness.
- No new attempt at anything on the NOT-REPEATING list.