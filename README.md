# Fixed3D. Box3D, but it's fixed point to make Erin mad

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
- **fixed** = this repo at `98b9889` (Q48.16 `int64_t`, SIMD removed, scalar
  4-wide constraint blocks)

| Benchmark     | float (ms) | fixed (ms) | fixed / float |
|---------------|-----------:|-----------:|--------------:|
| convex_pile   |   13,733.1 |   27,075.7 |         2.0× |
| joint_grid    |      275.4 |      808.1 |         2.9× |
| junkyard      |    4,733.1 |   11,778.5 |         2.5× |
| large_pyramid |      521.9 |    2,293.7 |         4.4× |
| large_world   |       13.2 |       90.8 |         6.9× |
| many_pyramids |      501.7 |    2,246.8 |         4.5× |
| rain          |      586.1 |    1,956.1 |         3.3× |
| trees25       |      227.2 |      464.4 |         2.0× |
| trees50       |      113.6 |      228.0 |         2.0× |
| trees100      |       80.7 |      170.2 |         2.1× |
| washer        |    6,630.4 |   20,212.2 |         3.0× |

**Geometric mean: 3.0× slower.** Worst case (large_world): 6.9× slower.

### Where the time goes

Both builds spend most of their time in the same place —
`b3QueryEdgeDirections`, the hull–hull edge-edge SAT query inside
`b3CollideHulls` — so the gap is mostly arithmetic, not algorithm:

- `b3FixMul`/`b3FixDiv` go through 128-bit intermediates;
  `__divti3`/`__udivmodti4` (software 128-bit division) show up directly in
  the fixed-point sample profiles.
- Square roots are exact bit-by-bit integer sqrt. Normalization does 128-bit
  component divides. Correctness first; none of it is vectorized.
- During this benchmark run the fixed-point build printed **12,858,880
  "CCD stall" warnings** (the time-of-impact solver making zero progress on
  quantized geometry). Some of the slowdown above is the physics; some of it
  is the physics complaining about itself at millions of lines per second.

### What you get for the 3×

- Cross-platform, cross-compiler, bit-exact determinism by construction —
  no FP contraction flags, no `-ffloat-store`, no x87 anxiety, no per-platform
  golden files.
- Uniform 1.5×10⁻⁵ resolution at the origin and at 100 km from the origin.
  Large-world mode deleted because every world is a large world now.

## Should I use this?

No. **DO NOT USE THIS LIBRARY.** It exists to make one specific person mad.

## License

MIT, same as the real Box3D, which — once more — you should use instead:
<https://github.com/erincatto/box3d>
