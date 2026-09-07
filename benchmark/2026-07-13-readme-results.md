# Historical README measurements

These July measurements used an older float revision and are retained as history.

## Profile results: fixed point vs. single precision floating point

`benchmark -t=4 -w=4 -r=2` (4 workers, min of 2 runs, continuous collision on),
Apple M3 Ultra, macOS 26.5.1, Apple clang 21, RelWithDebInfo, Ninja.
Measured 2026-07-13 at the current build defaults; all three columns were
re-run in the same session, float included.

- **float** = Box3D at `e961bfb` (single precision, NEON SIMD)
- **fixed** = this tree, scalar int64 lanes
- **fixed+NEON** = this tree with `-DBOX3D_NEON=ON` (narrow phase only)

| Benchmark     | float (ms) | fixed (ms) | fixed+NEON (ms) | fixed/float | NEON/float | NEON speedup |
|---------------|-----------:|-----------:|----------------:|------------:|-----------:|-------------:|
| convex_pile   |   13,725.4 |   21,391.2 |        10,316.8 |       1.6× |  **0.75×** |        2.07× |
| joint_grid    |      276.7 |      789.1 |           785.3 |       2.9× |      2.8× |        1.00× |
| junkyard      |    4,875.1 |    9,859.1 |         8,750.3 |       2.0× |      1.8× |        1.13× |
| large_pyramid |      547.8 |    1,676.9 |         1,625.0 |       3.1× |      3.0× |        1.03× |
| large_world   |       13.4 |       23.5 |            23.9 |       1.8× |      1.8× |        0.98× |
| many_pyramids |      518.0 |    1,681.5 |         1,651.8 |       3.2× |      3.2× |        1.02× |
| rain          |      610.5 |    1,289.0 |         1,286.1 |       2.1× |      2.1× |        1.00× |
| trees25       |      234.5 |      358.7 |           348.4 |       1.5× |      1.5× |        1.03× |
| trees50       |      117.5 |      196.5 |           195.0 |       1.7× |      1.7× |        1.01× |
| trees100      |       84.2 |      154.1 |           149.0 |       1.8× |      1.8× |        1.03× |
| washer        |    6,896.4 |   13,599.9 |        13,606.9 |       2.0× |      2.0× |        1.00× |

Geometric mean: 2.07× slower scalar, 1.90× with NEON. I expect this to worsen to around 2.5× as any
worthwhile optimizations found during this exercise are backported to the real Box3D.

