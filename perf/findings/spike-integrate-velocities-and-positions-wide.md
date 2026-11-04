VERDICT: KEEP (conditional on one pre-existing gate defect). Measured on an AVX-512 bench: integrateVelocities 1.26-1.28x and integratePositions 1.14-1.15x faster on all three target scenes, step geomean 1.041, no scene slower. The candidate's hashes are identical to unpatched BASE on all 78 rows (AVX-512 and scalar builds). The committed tools/perf/scalar-oracle.txt is stale at BASE 6183d14 for bodies_20k, ragdoll_pile and rain: unpatched scalar BASE misses it in the same way. So "identical to the oracle" cannot hold for any build until that oracle is regenerated.

Attempt 5 (sprint/spike-integrate-velocities-and-positions-wide.w5.g1.e21). It continues from attempt 2's head 0884de2, which measured the phases on an AVX2-only box, and merges the BASE tip sprint/fixed3d-box3d-ports 6183d14 so the patch applies to today's BASE.

## Hypothesis

Attempt 2 measured integrateVelocities at 37% of bodies_20k's step on a scalar build. Almost all of that is the gyroscopic Newton step: two quaternion rotations of the angular velocity, and the Iw / residual / Jacobian build. These are chains of plain b3FixMul with wrapping adds, which run four lanes wide exactly through contact_solver.c's b3MulLanesAVX decomposition. That makes them bit-identical by construction. Making them wide should speed up the phase without moving a bit.

## The change, in one paragraph

In src/solver.c, each kernel's per-body body moves verbatim into b3IntegrateVelocityScalar and b3IntegratePositionScalar. These are the scalar control and the whole path on non-AVX-512 builds. b3IntegrateVelocitiesRange and b3IntegratePositionsRange (declared in solver.h, used by the tasks and by the test) run four consecutive bodies per iteration under B3_SIMD_AVX512, with a scalar tail. In the velocity kernel, the parts that are not a plain multiply chain stay per lane and scalar: the damping divides, b3InvMulSV/b3InvMulMV (inverse scale), b3MulQuat (fused raw-128) and b3Solve3 (128-bit cofactors and divides). The rest runs wide: b3InvRotateVector, the Newton Iw/b/J build, omega2 -= delta, and b3RotateVector, each written exactly as fixed_vec.h writes it. That means the two-cross form with one rounding per product; the do-not-fuse list is respected and nothing is fused. The position kernel applies the lock selects per lane. The speed-clamp compare uses the exact scalar raw-128 b3Dot. If any lane would cap, all four lanes go through the scalar body, which is idempotent on the locks; this is the rare case. Otherwise deltaPosition += h*v and h*w run wide, and b3IntegrateRotation stays scalar per lane. The scalar path's operation order is unchanged. No floats in src/. The wide multiply is a local copy of b3MulLanesAVX (b3IntegrateMulAVX) rather than a shared header edit.

## Bench and conditions

- Bench: space, AMD EPYC 9124 16-Core (Zen 4). ISA: avx2 avx512dq avx512f avx512ifma avx512vl.
- Compiler: cc (Ubuntu 15.2.0-16ubuntu1) 15.2.0, cmake 4.2.3, RelWithDebInfo, -flto=auto (the default LTO build), BOX3D_VALIDATE=ON.
- `taskset -cp $$`: 0-31, not pinned (the space jail rule was respected).
- Load average: 2.25 2.66 2.07 before the A/B and 3.99 2.61 2.16 after, on 32 threads. The box is shared with swarm workers. `pgrep -af benchmark` was empty before every run.
- A = BASE 6183d14 with BOX3D_AVX512=ON. B = BASE + experiment.patch with BOX3D_AVX512=ON. Same flags, interleaved, 7 repeats, -w=1, --steps=120. Only one worker count was measured (w=1); see not-done.

## Before / after (ns per step, median of 7, -w=1; spread = max-min of the medians as ab.sh reports it)

| scene         | phase               | A median    | A spread  | B median    | B spread   | speedup |
|---------------|---------------------|-------------|-----------|-------------|------------|---------|
| bodies_20k    | integrateVelocities | 11,382,378  | 41,060    | 9,013,745   | 47,003     | 1.263   |
| bodies_20k    | integratePositions  | 3,802,895   | 26,548    | 3,325,925   | 21,671     | 1.143   |
| bodies_20k    | step                | 35,512,455  | 367,953   | 32,543,856  | 372,128    | 1.091   |
| rain          | integrateVelocities | 541,422     | 4,411     | 422,555     | 5,433      | 1.281   |
| rain          | integratePositions  | 183,559     | 1,570     | 160,673     | 2,773      | 1.142   |
| rain          | step                | 7,532,068   | 47,258    | 7,414,343   | 28,948     | 1.016   |
| many_pyramids | integrateVelocities | 6,071,793   | 241,267   | 4,809,462   | 573,303    | 1.262   |
| many_pyramids | integratePositions  | 2,061,320   | 63,426    | 1,796,835   | 137,746    | 1.147   |
| many_pyramids | step                | 149,736,560 | 6,184,353 | 146,954,068 | 17,679,808 | 1.019   |

Step geomean speedup (ab.sh): 1.04144. Exit 0, and the final HASH lines of A and B agree on every run.

Phase share of the AVX-512 baseline step: bodies_20k 32.1% + 10.7%; rain 7.2% + 2.4%; many_pyramids 4.1% + 1.4%. The card's "under 5% on every scene" escape hatch does not fire.

Against the KEEP rule: the geomean is 4.1%, at least the 3% the rule asks for, and no scene is slower. The bodies_20k gain (8.4% of the step) is far outside its 1.0% spread. The rain gain (1.6%) is outside its 0.6% spread. The many_pyramids step gain (1.9%) is INSIDE its spread, which is wide on this shared box (4-12%), but its integrate phases are clearly faster. The phase speedups are below the 1.5x arm of the rule, so this KEEP rests on the geomean arm.

## Bit-exact evidence

1. Unit tier: IntegrateWideEquivalenceTest is in test/test_body.c and registered in BodyTest. It builds 100,002 seeded bodies (a count that is not a multiple of four, so the tail runs) with kinematic (invMass 0), all motion locks, allowFastRotation, thin-rod inertia, and speeds that trip both clamps. It asserts that some body was actually capped. It runs two substeps of velocity then position, scalar and wide, and compares every state field byte for byte. Red at unchanged BASE (compile, the card's red line):
   `test_body.c:1233:17: error: implicit declaration of function 'b3IntegrateVelocitiesRange'`
   Green on the candidate AVX-512 build: `./bin/test BodyTest` -> "All Fixed3D tests passed!".
2. Candidate vs unpatched BASE, both BOX3D_AVX512=ON, all 13 harness scenes x workers 1/2/4 x narrow/ludicrous (78 rows, --steps=120): **diff-lines=0**.
3. Candidate scalar build (BOX3D_AVX512=OFF): `tools/perf/oracle_check.sh` exits 1 with ORACLE MISMATCH on bodies_20k, ragdoll_pile and rain (18 rows). The other 10 scenes match. **This mismatch is pre-existing.** Unpatched BASE 6183d14 built scalar gives bodies_20k ed59e6aff81bcd26, ragdoll_pile 42fa9c6438e3527f, rain f96c9e008d59a375 (w=1). These are the candidate's values, not the oracle's (3e7cbde301a4f6c2 for bodies_20k). The oracle was regenerated on an earlier base, and the 44 commits since changed those three scenes. Unpatched BASE AVX-512 differs from the oracle on the same 36 diff lines. Workers 1/2/4 agree in every build.
4. `./bin/test`: green on the candidate scalar build (exit 0). On AVX-512 builds, AllocatorTest / AllocatorWidenedSignatureTest fails on unpatched BASE as well as on the candidate. test/test_allocator.c is unchanged by this work, so it was already red at BASE under BOX3D_AVX512, probably the 32-byte B3_ALIGNMENT. That failure ends the full run before BodyTest, so BodyTest was run as its own suite.

## What to try next

- Regenerate tools/perf/scalar-oracle.txt at the BASE tip (a harness card, not this one). Then re-run the oracle gate here and confirm the 78/78 match that (2) and (3) already imply.
- Fix or skip AllocatorWidenedSignatureTest under BOX3D_AVX512 so the full AVX-512 ./bin/test can be the gate.
- If adopted: measure workers 2 and 4, and repeat the A/B on a quiet box to bring many_pyramids inside its own spread.
- The next cost in integrateVelocities is the per-lane b3Solve3, with three 128-bit divides per body. The "solve3 across scales 256-by-128 arm" landed on BASE; a wide cofactor build feeding per-lane divides is the natural follow-on. b3MulQuat and the lock selects could also go wide using the b3DotAccAVX accumulators, which are exact for these sizes.

## What not to repeat

- Do not claim the phases are sub-5%: they are 32% + 11% of bodies_20k even with the AVX-512 contact solver.
- Do not take an oracle mismatch for a code fault without hashing unpatched BASE first. Here the oracle, not the patch, was behind.
- Nothing on the NOT-REPEATING list was attempted. There is no forced inlining. The guard in the wide position loop is a per-group fallback to the scalar control, outside any solve loop.

## Header

See raw.txt (cpu_info.sh output, ab.sh table, gate logs, all hash rows).
