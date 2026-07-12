# Fixed-point benchmark results — Apple M3 Ultra (2026-07-11)

Results for the Q48.16 fixed-point build on an Apple M3 Ultra (arm64 clang,
RelWithDebInfo). The CSVs in this directory are the **current** state: commit
98b9889 plus the session-3 performance pass (raw 128-bit dot layer, exact SAT
sign tests, single-rounding wide solver reductions).

Command per benchmark (note the equals-form flags — space-separated flags are
silently ignored and the full suite runs):

```
benchmark -b=<name> -w=4 -t=4 -r=2
```

CSV `ms` is the wall time for the whole run (199-999 steps depending on the
benchmark), minimum over 2 repeats, at 4 workers.

## Timeline (ms, 4 workers, min of 2 runs)

| benchmark      | before session 3* | 98b9889 alone | now    | float (e9f6f1d) |
|----------------|-------------------|---------------|--------|-----------------|
| convex_pile    | 46779             | 27279         | 21117  | 13645           |
| joint_grid     | 1555              | 809           | 814    |                 |
| large_pyramid  | 4444              | 2295          | 2078   | 518             |
| large_world    | 162               | 92            | 84     |                 |
| many_pyramids  | 4263              | 2246          | 2039   |                 |
| rain           | 2637              | 1747          | 1733   |                 |
| trees50        | 419               | 226           | 209    |                 |
| washer         | 28767             | 18976         | 17575  |                 |

\* "before" numbers are from the morning benchmark run predating both the
98b9889 fixed.h fast paths and the session-3 pass.

## Profiles (`profiles/`, macOS `sample` output)

- `profile_fixed_before.txt` — fixed build before either optimization pass.
  `b3QueryEdgeDirections` is ~60% of all samples, `b3FindHullSupportVertex`
  another ~11%: the SAT narrow phase dominated everything. (Caveat: captured
  during a default-suite run, so the workload is convex_pile, not large_pyramid.)
- `profile_float_baseline.txt` — float build (e9f6f1d), same workload. Also
  edge-query dominated (~65% of busy samples): the workload is inherently
  SAT-heavy; fixed point was paying the same shape at a higher per-op cost.
- `profile_98b9889.txt` — after 98b9889's fixed.h fast paths (non-saturating
  b3FixMul, b3FixDiv fast path, seeded sqrt), large_pyramid at 1 worker. The
  narrow phase shrinks and the wide contact solver becomes the top cost
  (`b3CrossW` ~27%).
- `profile_now.txt` — current state, large_pyramid at 1 worker. Solver-bound:
  `b3SolveContacts_Convex`, `b3RotateVectorW`, `b3MulSub/AddMVW` lead; narrow
  phase and 128-bit divides are slivers. Next levers: NEON int64 2-lane
  vectorization of the wide solver, `b3RotateVectorW` restructure, remaining
  `__udivmodti4` calls.
