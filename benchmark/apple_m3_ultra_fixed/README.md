# Fixed-point benchmark results — Apple M3 Ultra (2026-07-11)

Results for the Q48.16 fixed-point build on an Apple M3 Ultra (arm64 clang,
RelWithDebInfo). The CSVs in this directory are the **current** state (commit
ab2d210: 98b9889 fast paths + session-3 raw-dot pass + sentinel-audit SAH fix).
The vanilla float baseline (e9f6f1d) measured under identical conditions is in
`../apple_m3_ultra_float/`.

Command per benchmark (note the equals-form flags — space-separated flags are
silently ignored and the full suite runs):

```
benchmark -b=<name> -w=4 -t=4 -r=2
```

CSV `ms` is the wall time for the whole run (199-999 steps depending on the
benchmark), minimum over 2 repeats, at 4 workers.

## Fixed vs float (ms, 4 workers, min of 2 runs, same session)

| benchmark      | fixed (ab2d210) | float (e9f6f1d) | fixed / float |
|----------------|-----------------|-----------------|---------------|
| convex_pile    | 20708           | 13626           | 1.52x         |
| joint_grid     | 810             | 271             | 2.99x         |
| large_pyramid  | 2072            | 508             | 4.08x         |
| large_world    | 85              | 14              | 5.99x         |
| many_pyramids  | 2031            | 490             | 4.15x         |
| rain           | 1356            | 582             | 2.33x         |
| trees50        | 203             | 116             | 1.74x         |
| washer         | 15305           | 6577            | 2.33x         |

**Geomean: ~2.85x of float** (was ~5.4x before any optimization, ~3.2x after
98b9889 alone). The sentinel-audit fix to the dynamic tree SAH (skip degenerate
0|N split planes) was also a real speedup for rebuild-heavy scenes: rain
1733 -> 1356 (-22%), washer 17575 -> 15305 (-13%) versus the pre-audit numbers.

## Optimization timeline (ms, 4 workers)

| benchmark      | before | 98b9889 | session 3 | + SAH fix | float |
|----------------|--------|---------|-----------|-----------|-------|
| convex_pile    | 46779  | 27279   | 21117     | 20708     | 13626 |
| joint_grid     | 1555   | 809     | 814       | 810       | 271   |
| large_pyramid  | 4444   | 2295    | 2078      | 2072      | 508   |
| large_world    | 162    | 92      | 84        | 85        | 14    |
| many_pyramids  | 4263   | 2246    | 2039      | 2031      | 490   |
| rain           | 2637   | 1747    | 1733      | 1356      | 582   |
| trees50        | 419    | 226     | 209       | 203       | 116   |
| washer         | 28767  | 18976   | 17575     | 15305     | 6577  |

"before" numbers are from the morning benchmark run predating both the 98b9889
fixed.h fast paths and the session-3 pass.

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
- `profile_now.txt` — current fixed build. Solver-bound: contact solve inner
  iteration ~75% (`b3SolveContacts_Convex` 27%, `b3RotateVectorW` 17%,
  `b3MulAdd/SubMVW` 17%, warm start 8%); narrow phase ~6%; 128-bit divides ~2%.
- `profile_float_large_pyramid.txt` — float build, same workload as
  `profile_now.txt`. Also solver-bound (~50% in the same functions with the
  lane math inlined), but spends ~20% in `b3GatherBodies`/`b3ScatterBodies`
  SIMD state shuffling that the scalar int64 lanes of the fixed build do for
  ~3%. The remaining fixed-vs-float gap is concentrated in the wide lane math:
  64x64->128 multiplies with rounding versus float FMA. Next levers: NEON
  int64 2-lane vectorization of the wide solver, b3RotateVectorW restructure
  (two crosses -> per-body matrix), remaining `__udivmodti4` calls.
