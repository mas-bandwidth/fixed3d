# Box3D, but it's fixed point now

> # ⚠️ DO NOT USE THIS LIBRARY ⚠️
>
> **This is a joke.** The entire purpose of this fork is to make Erin mad.
>
> It is not supported. It is not maintained. It is not endorsed by Erin Catto,
> and if he has seen it, he is mad about it, which was the point.
>
> Use the real [Box3D](https://github.com/erincatto/box3d).
>
> **DO NOT USE THIS LIBRARY.**

## What is this

Box3D with every `float` torn out of the simulation and replaced with **Q48.16
fixed point** in an `int64_t`. All of it: the solver, GJK, the trig, the ray
casts, the mass properties, the recording format. The SIMD is gone. In
exchange, every step is bit-exact on every platform, resolution is a uniform
1/65536 everywhere in a ±1.4×10¹⁴ meter world, and all 22 unit test suites
still pass.

```
45078b4 there i fixed it for you
```

## Profile results: fixed point vs. vanilla single precision

`benchmark -t=4 -w=4 -r=2` (4 workers, min of 2 runs, continuous collision on),
Apple M3 Ultra, macOS 26.5.1, Apple clang 21, RelWithDebInfo, Ninja.

- **float** = vanilla Box3D at `e9f6f1d` (single precision, NEON SIMD)
- **fixed** = this tree (Q48.16 `int64_t`, SIMD removed, scalar 4-wide constraint blocks)

| Benchmark     | float (ms) | fixed (ms) | fixed / float |
|---------------|-----------:|-----------:|--------------:|
| convex_pile   |   13,733.1 |   21,039.6 |         1.5× |
| joint_grid    |      275.4 |      776.1 |         2.8× |
| junkyard      |    4,733.1 |    9,517.8 |         2.0× |
| large_pyramid |      521.9 |    1,588.4 |         3.0× |
| large_world   |       13.2 |       63.2 |         4.8× |
| many_pyramids |      501.7 |    1,666.1 |         3.3× |
| rain          |      586.1 |    1,235.5 |         2.1× |
| trees25       |      227.2 |      390.2 |         1.7× |
| trees50       |      113.6 |      198.0 |         1.7× |
| trees100      |       80.7 |      144.1 |         1.8× |
| washer        |    6,630.4 |   13,545.4 |         2.0× |

**Geometric mean: 2.3× slower** (down from 3.0× at the first commit; the
optimization log with per-pass numbers and sample profiles lives in
[benchmark/apple_m3_ultra_fixed](benchmark/apple_m3_ultra_fixed/README.md)).
Worst case (large_world): 4.8× slower. The unit test suite runs in ~0.9 s vs
0.39 s for the float baseline (~2.2×).

### Where the time goes

Both builds are now solver-bound in the same functions, so the gap is pure
arithmetic: a fixed-point multiply is `mul + smulh + add + shift` against a
single float FMA, and no 64-bit NEON multiply exists to vectorize it away.
The narrow phase — once 60% of a step — is a sliver after exact raw 128-bit
sign tests replaced per-product rounding in the SAT queries. The contact
solver runs on per-step precomputed Jacobian rows stored as 32-bit lanes
(they fit losslessly; the impulses that don't fit stay 64-bit — a pile of
dense hulls was measured carrying 5.8 million units of accumulated impulse,
177× past what 32-bit lanes would hold).

The earlier edition of this README reported **12,858,880 "CCD stall"
warnings** during a benchmark run. That turned out to be the stall *report*
threshold, not the stall: the float code disables it with
`1000 * FLT_MAX = inf`, and the fixed-point translation of that idiom wraps
to roughly −0.015 ms, so every single time-of-impact query "exceeded" it.
The physics was fine. The physics was being slandered.

### What you get for the 2.3×

- Cross-platform, cross-compiler, bit-exact determinism by construction —
  no FP contraction flags, no `-ffloat-store`, no x87 anxiety, no per-platform
  golden files. The determinism test hashes 200 steps of ragdoll piles and
  gets the same answer on 1, 2, 3, and 4 threads, every time.
- Uniform 1.5×10⁻⁵ resolution at the origin and at 100 km from the origin.
  Large-world mode deleted because every world is a large world now.

## Should I use this?

No. **DO NOT USE THIS LIBRARY.** It exists to make one specific person mad.

## License

MIT, same as the real Box3D, which — once more — you should use instead:
<https://github.com/erincatto/box3d>
