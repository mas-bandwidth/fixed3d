# Vanilla float benchmark results — Apple M3 Ultra (2026-07-11)

Reference numbers for the **float baseline** (commit e9f6f1d, vanilla Box3D
single-precision with SIMD) on the same Apple M3 Ultra used for the fixed-point
results in `../apple_m3_ultra_fixed/`. Built RelWithDebInfo, arm64 clang, from
a worktree at e9f6f1d.

Same measurement protocol as the fixed results:

```
benchmark -b=<name> -w=4 -t=4 -r=2
```

CSV `ms` is wall time for the whole run, minimum over 2 repeats, at 4 workers.
See `../apple_m3_ultra_fixed/README.md` for the side-by-side comparison
(geomean: fixed is ~2.85x of this float baseline).
