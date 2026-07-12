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

| benchmark      | fixed (solver pass) | float (e9f6f1d) | fixed / float |
|----------------|---------------------|-----------------|---------------|
| convex_pile    | 21055               | 13626           | 1.55x         |
| joint_grid     | 855                 | 271             | 3.16x         |
| large_pyramid  | 1735                | 508             | 3.41x         |
| large_world    | 72                  | 14              | 5.12x         |
| many_pyramids  | 1699                | 490             | 3.47x         |
| rain           | 1380                | 582             | 2.37x         |
| trees50        | 217                 | 116             | 1.87x         |
| washer         | 13840               | 6577            | 2.10x         |

**Geomean: ~2.7x of float** (was ~5.4x before any optimization, ~3.2x after
98b9889 alone, ~2.85x before the solver pass). Session run-to-run variance is
roughly +/-2-5%; the solver pass trades a small prepare/warm-start bandwidth
cost (joint_grid and trees50 gave back ~5-7%) for 10-16% on solver-bound
scenes.

## Optimization timeline (ms, 4 workers)

| benchmark      | before | 98b9889 | session 3 | + SAH fix | solver pass | float |
|----------------|--------|---------|-----------|-----------|-------------|-------|
| convex_pile    | 46779  | 27279   | 21117     | 20708     | 21055       | 13626 |
| joint_grid     | 1555   | 809     | 814       | 810       | 855         | 271   |
| large_pyramid  | 4444   | 2295    | 2078      | 2072      | 1735        | 508   |
| large_world    | 162    | 92      | 84        | 85        | 72          | 14    |
| many_pyramids  | 4263   | 2246    | 2039      | 2031      | 1699        | 490   |
| rain           | 2637   | 1747    | 1733      | 1356      | 1380        | 582   |
| trees50        | 419    | 226     | 209       | 203       | 217         | 116   |
| washer         | 28767  | 18976   | 17575     | 15305     | 13840       | 6577  |

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
