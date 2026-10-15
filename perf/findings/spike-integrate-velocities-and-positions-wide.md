VERDICT: LEARNED (measured, not wide-ified) — the phases are NOT sub-5%; integrateVelocities is 37% of the step on bodies_20k, but the AVX-512 bench is still absent so the wide work cannot be measured.

## What this attempt changed vs attempt 1

Attempt 1 (head 2adb413) concluded LEARNED-not-measured and *predicted* the
integrate phases were sub-5% of the step on every scene, from profile evidence
alone. This attempt has the harness (`tools/perf/` landed) and a Linux bench,
so the phase fractions are now MEASURED rather than predicted — and the
prediction is wrong. The phases are a real target, not a rounding error.

## The measurement (the card's escape hatch, answered)

The card says: "If the first measurement shows the phases are under 5 percent
of the step on every scene, the verdict is LEARNED/DROP and you stop there."
They are not. Per-scene phase fractions (scalar build, `-w=1 -r=1 --steps=120`,
`--phases-json`, median_ns, phase / `step`):

| scene          | integrateVelocities | integratePositions | sum      |
|----------------|---------------------|--------------------|----------|
| bodies_20k     | 19,701,026 ns (37.0%) | 4,926,845 ns (9.25%) | 46.2%  |
| rain           |   737,637 ns (8.28%)  |   252,191 ns (2.83%) | 11.1%  |
| many_pyramids  | 9,584,408 ns (5.13%)  | 2,699,267 ns (1.44%) | 6.57%  |

`integrateVelocities` alone is 37% of the step on `bodies_20k` — the scene the
card names as the stress case — and 5.1% on `many_pyramids`. The escape hatch
does not fire. The gyroscopic Newton solve (`B3_GYROSCOPIC_ITERATIONS` × a
3×3 `b3Solve3` per body, solver.c:136-168) is the dominant cost, not the
"few multiplies and one Pade damping step" attempt 1 assumed.

## Why not KEEP / not wide-ified

The card's KEEP rule and its whole premise are AVX-512: "wide
integrate-velocities and integrate-positions on the AVX-512 lanes", "NEEDS AN
AVX-512 BENCH". This bench is **AMD Ryzen Threadripper 2990WX (Zen+)** — ISA
flags `avx2` only, no `avx512f/dq/vl`. `BOX3D_AVX512=ON` cannot be built or
run here (the wide path is AVX-512VL 256-bit integer ops; there is no AVX2
wide path in the tree). So the wide port cannot be written-and-measured on
this attempt, and a KEEP verdict is unreachable by construction. What this
attempt delivers is the measurement that *corrects* attempt 1's premise.

## Operation order and rounding points (unchanged from attempt 1, re-read)

`b3IntegrateVelocitiesTask` (solver.c:67-178), per body: Pade damping
`b3FixDiv(1, 1 + b3FixMul(h, damping))` (one truncating divide per axis);
`linearVelocityDelta = b3Add(b3InvMulSV(b3FixMul(h, invMass), force),
b3MulSV(b3FixMul(h, gravityScale), gravity))`; `v = b3MulAdd(delta, damping, v)`
(one fused rounding); `w = b3MulAdd(b3MulSV(h, b3InvMulMV(invInertiaWorld,
torque)), angularDamping, w)`; then the gyroscopic term: `q = b3MulQuat(deltaRotation, q0)`,
`omega1 = b3InvRotateVector(q, w)` (do-not-fuse), `B3_GYROSCOPIC_ITERATIONS`
Newton steps each building a 9-entry Jacobian and residual from unfused
`b3FixMul` products plus one 128-bit `b3Solve3`, closed by
`w = b3RotateVector(q, omega2)` (do-not-fuse). `b3IntegratePositionsTask`
(solver.c:180-231): motion-lock selects, linear/angular speed clamps
(`b3Dot` fused raw-128, `b3FixDiv` divide-last), `deltaPosition = b3MulAdd(...)`,
`deltaRotation = b3IntegrateRotation(...)` with the exact raw-128 quaternion
normalize. A wide port must reproduce every one of these rounding points per
lane; the do-not-fuse `b3RotateVector`/`b3InvRotateVector` and the `b3Solve3`
128-bit divide are the delicate parts.

## Bit-exact evidence

None for a wide path (no wide path was written; no AVX-512). `./bin/test` is
green on the scalar build (5.67 s, "All Fixed3D tests passed!"). The scalar
oracle was not regenerated or diffed — there is no candidate to diff.

## What to try next

Re-run this spike on the real AVX-512 bench (the `space` box, AMD EPYC 9124
Zen 4, full AVX-512). The phase measurement now says the target is real:
`integrateVelocities` is 37% of `bodies_20k` and 5.1% of `many_pyramids`, so
a wide port that halves the gyroscopic solve would move the geomean past the
3% KEEP threshold on the target scenes. The wide port should process four
contiguous `b3BodyState`/`b3BodySim` records per iteration behind
`B3_SIMD_AVX512`, scalar loop kept verbatim in the `#else`, reproducing each
rounding point per lane (the CLAUDE.md "Wide solver reductions" / "Round 3"
precedent).

## What not to repeat

- Do not repeat attempt 1's claim that the phases are sub-5% — it is now
  measured false (37% on bodies_20k).
- Do not attempt the wide port on this AVX2-only box; the AVX-512 path cannot
  build or run here.
- No new attempt at anything on the NOT-REPEATING list.

## Header

```
== CPU model ==
model name	: AMD Ryzen Threadripper 2990WX 32-Core Processor
== ISA flags ==
avx2
== cores ==
64
== CPU pinning ==
pid ... current affinity list: 0-63
== Load average ==
1.83 1.37 1.06 (before); 1.04 0.84 0.85 (after)
== Compiler ==
cc (Ubuntu 15.2.0-16ubuntu1) 15.2.0
== cmake ==
cmake version 4.2.3
== cmake flags ==
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBOX3D_SAMPLES=OFF -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBOX3D_BENCHMARKS=ON
```
