# Fixed-point benchmark results — Apple M3 Ultra (2026-07-11)

Results for the Q48.16 fixed-point build on an Apple M3 Ultra (arm64 clang,
RelWithDebInfo). The CSVs in this directory are the **current** state:
ab2d210 plus the solver pass (per-constraint delta-rotation matrices and
precomputed contact Jacobian rows). The vanilla float baseline (e9f6f1d)
measured under identical conditions is in `../apple_m3_ultra_float/`.

Command per benchmark (note the equals-form flags — space-separated flags are
silently ignored and the full suite runs):

```
benchmark -b=<name> -w=4 -t=4 -r=2
```

CSV `ms` is the wall time for the whole run (199-999 steps depending on the
benchmark), minimum over 2 repeats, at 4 workers.

## Fixed vs float (ms, 4 workers, min of 2 runs)

| benchmark      | fixed (narrow) | float (e9f6f1d) | fixed / float |
|----------------|----------------|-----------------|---------------|
| convex_pile    | 21040          | 13626           | 1.54x         |
| joint_grid     | 776            | 271             | 2.87x         |
| large_pyramid  | 1588           | 508             | 3.13x         |
| large_world    | 63             | 14              | 4.46x         |
| many_pyramids  | 1666           | 490             | 3.40x         |
| rain           | 1236           | 582             | 2.12x         |
| trees50        | 198            | 116             | 1.70x         |
| washer         | 13545          | 6577            | 2.06x         |

**Geomean: ~2.51x of float** (was ~5.4x before any optimization, ~3.2x after
98b9889 alone, ~2.85x before the solver pass, ~2.7x before round 3, ~2.56x
before narrow storage). Session run-to-run variance is roughly +/-2-5%
(washer up to +/-10%).

The narrow-storage step keeps all constraint geometry (anchors, normals,
tangents, Jacobian rows) as int32 Q16.16 — the range audit shows those values
stay under 225 units, so they round-trip losslessly and the simulation is
bit-identical (determinism goldens unchanged) while the constraint shrinks
~30% and prepare/warm-start bandwidth drops. The same audit ruled OUT full
Q16.16 32-bit lane math: convex_pile impulses reach ~5.8M units, 177x over
the int32 range.

Round 3 fused the scalar quaternion/dot operations (b3Dot, b3LengthSquared,
b3DistanceSquared, b3DotQuat, b3MulQuat, b3InvMulQuat) into single-rounding
128-bit reductions and routed b3NormalizeQuat divides through the b3FixDiv
fast path. b3Cross, b3MulMV, b3RotateVector, and b3Lerp are DELIBERATELY kept
at per-product rounding: fusing each was bisected to either a TestMeshDrop
sleep limit cycle or a convex_pile SAT cache-miss regime (+40%). See the
round-3 notes in CLAUDE.md before touching scalar rounding.

## Optimization timeline (ms, 4 workers)

| benchmark      | before | 98b9889 | session 3 | + SAH fix | solver pass | round 3 | narrow | float |
|----------------|--------|---------|-----------|-----------|-------------|---------|--------|-------|
| convex_pile    | 46779  | 27279   | 21117     | 20708     | 21055       | 20917   | 21040  | 13626 |
| joint_grid     | 1555   | 809     | 814       | 810       | 855         | 783     | 776    | 271   |
| large_pyramid  | 4444   | 2295    | 2078      | 2072      | 1735        | 1638    | 1588   | 508   |
| large_world    | 162    | 92      | 84        | 85        | 72          | 66      | 63     | 14    |
| many_pyramids  | 4263   | 2246    | 2039      | 2031      | 1699        | 1669    | 1666   | 490   |
| rain           | 2637   | 1747    | 1733      | 1356      | 1380        | 1238    | 1236   | 582   |
| trees50        | 419    | 226     | 209       | 203       | 217         | 205     | 198    | 116   |
| washer         | 28767  | 18976   | 17575     | 15305     | 13840       | 14093   | 13545  | 6577  |

"before" numbers are from the morning benchmark run predating both the 98b9889
fixed.h fast paths and the session-3 pass. The sentinel-audit fix to the
dynamic tree SAH (skip degenerate 0|N split planes) was also a real speedup for
rebuild-heavy scenes: rain -22%, washer -13%. The solver pass replaces the
per-iteration anchor quaternion rotations with per-constraint delta-rotation
matrices and precomputes the contact Jacobian rows (cross(r, n), invI*cross(r, n),
invMass*n, invI*n, cross(origin, tangent)) in prepare.

## Profiles (`profiles/`, macOS `sample` output, large_pyramid at 1 worker)

- `profile_fixed_before.txt` — fixed build before either optimization pass.
  `b3QueryEdgeDirections` is ~60% of all samples, `b3FindHullSupportVertex`
  another ~11%: the SAT narrow phase dominated everything. (Caveat: captured
  during a default-suite run, so the workload is convex_pile, not large_pyramid.)
- `profile_float_baseline.txt` — float build, same default-suite workload as
  above. Also edge-query dominated (~65% of busy samples).
- `profile_98b9889.txt` — after 98b9889's fixed.h fast paths (non-saturating
  b3FixMul, b3FixDiv fast path, seeded sqrt). The narrow phase shrinks and the
  wide contact solver becomes the top cost (`b3CrossW` ~27%).
- `profile_now.txt` — current fixed build (solver pass). Still solver-bound
  but flatter: `b3SolveContacts_Convex` ~24%, anchor rotation
  (`b3MulMV3W` + `b3MakeMatrixFromQuatW`) ~15%, prepare ~14%, warm start ~11%,
  `b3RelVelocityW` ~6%. Prepare and warm start are now memory-bound on the
  fatter constraint (precomputed Jacobian rows); the old `b3CrossW`/matvec
  towers are gone.
- `profile_float_large_pyramid.txt` — float build, same workload. Also
  solver-bound (~50% in the same functions with the lane math inlined), but
  spends ~20% in `b3GatherBodies`/`b3ScatterBodies` SIMD state shuffling that
  the scalar int64 lanes of the fixed build do for ~3%. The remaining
  fixed-vs-float gap is the 64x64->128 lane multiplies vs float FMA. NEON is
  not the answer on this hardware (no 64-bit lane multiplies without SVE2);
  the remaining ideas are a Q16.16 32-bit-lane solver experiment (range
  analysis needed) and trimming constraint memory.
