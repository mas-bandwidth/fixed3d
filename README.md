# Box3D, but it's fixed point to make Erin mad

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
casts, the mass properties, the recording format. The SIMD is gone (it grew
back on AVX-512 — same bits, just faster; see below). In exchange, resolution
is a uniform 1/65536 everywhere in a ±1.4×10¹⁴ meter world, every step is
still bit-exact on every platform (vanilla Box3D already was — see below),
and all 22 unit test suites still pass.

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
single float FMA, and no 64-bit NEON multiply exists to vectorize it away
(on Zen 4 one exists — see the AVX-512 section).
The narrow phase — once 60% of a step — is a sliver after exact raw 128-bit
sign tests replaced per-product rounding in the SAT queries. The contact
solver runs on per-step precomputed Jacobian rows stored as 32-bit lanes
(they fit losslessly; the impulses that don't fit stay 64-bit — a pile of
dense hulls was measured carrying 5.8 million units of accumulated impulse,
177× past what 32-bit lanes would hold).

### What you get for the 2.3×

To be clear: **vanilla Box3D is already deterministic across platforms.**
Erin did that work in floating point — pinned contraction, no fast-math, the
discipline. Determinism is not a Fixed3D feature; it came with the library we
vandalized. Fixed point merely gets it by construction instead of by
vigilance: no FP contraction flags, no `-ffloat-store`, no x87 anxiety —
integers wrap the same everywhere, so there is nothing to hold carefully.

What you actually get for the 2.3×:

- Uniform 1.5×10⁻⁵ resolution at the origin and at 100 km from the origin,
  in a ±1.4×10¹⁴ meter world. Large-world mode deleted because every world
  is a large world now.
- Erin, mad.

## The SIMD grew back: AVX-512 results

NEON never had a chance: fixed point needs 64×64-bit lane multiplies, ARM
keeps those in SVE2, and Apple does not expose SVE2. So Apple silicon runs
scalar, by decree of Cupertino, and the table above is what that costs. But
Zen 4 ships `vpmullq` — a native, single-µop, 64-bit vector multiply — and a
Q48.16 solver is exactly the workload it was born for.

`-DBOX3D_AVX512=ON` (default OFF) packs the wide contact solver's four Q48.16
lanes into one 256-bit register. One `vpmullq` + one `vpmuludq` + one
`vpmuldq` reproduce the exact 128-bit fixed-point product, and the solver's
fused 128-bit dot reductions ride along as three carry-free 64-bit partial
sums. **Bit-identical to the scalar path for all inputs** — same determinism
goldens, same state hash on every thread count, verified by a 25-million-case
differential harness against the scalar reference and the full test suite
under ASan/UBSan. It is the same simulation. It is just faster.

`benchmark -t=4 -w=4 -r=2` (4 workers, min of 2 runs, continuous collision
on), AMD EPYC 9124 (Zen 4), Ubuntu 24.04, clang 18, RelWithDebInfo.

- **float** = vanilla Box3D at `e961bfb` (single precision, SSE2 SIMD)
- **fixed** = this tree, scalar int64 lanes
- **fixed+AVX** = this tree with `-DBOX3D_AVX512=ON`

| Benchmark     | float (ms) | fixed (ms) | fixed+AVX (ms) | fixed/float | AVX/float | AVX speedup |
|---------------|-----------:|-----------:|---------------:|------------:|----------:|------------:|
| convex_pile   |   63,942.9 |  126,761.0 |      102,083.0 |       2.0× |     1.6× |       1.24× |
| joint_grid    |    5,470.8 |   18,353.0 |       18,277.6 |       3.4× |     3.3× |       1.00× |
| junkyard      |   50,846.1 |  184,930.1 |      117,152.0 |       3.6× |     2.3× |       1.58× |
| large_pyramid |    8,582.3 |   56,672.0 |       27,559.1 |       6.6× |     3.2× |       2.06× |
| large_world   |       26.2 |      526.0 |          202.6 |      20.1× |     7.7× |       2.60× |
| many_pyramids |   11,843.7 |   53,265.0 |       29,166.7 |       4.5× |     2.5× |       1.83× |
| rain          |    6,659.8 |   24,157.0 |       21,649.2 |       3.6× |     3.3× |       1.12× |
| trees100      |      407.5 |      974.0 |          949.6 |       2.4× |     2.3× |       1.03× |
| trees25       |    1,020.4 |    2,382.0 |        2,323.7 |       2.3× |     2.3× |       1.03× |
| trees50       |      465.1 |    1,152.0 |        1,067.2 |       2.5× |     2.3× |       1.08× |
| washer        |   70,407.1 |  313,886.0 |      178,370.0 |       4.5× |     2.5× |       1.76× |

**Geomean: 3.9× of float scalar, 2.8× with AVX-512 — a 1.41× overall speedup,
and the solver-bound scenes halve or better** (large_pyramid 6.6× → 3.2×,
washer 4.5× → 2.5×, junkyard 3.6× → 2.3×; up to 2.6× faster per scene). Both
the solve loop and the contact prepare run wide: `b3SolveContacts_Convex`
alone is 1.96× faster with the anchor-rotation helpers vanishing into it, and
`b3PrepareContacts_Convex` is 1.6× faster on its own (one-point-manifold
scenes like the trees keep a small staging tax, measured −1% on rain and −6%
on the one-second trees100 run). The tiny scenes don't move because their
steps are dominated by joints and the narrow phase, which stay scalar. Note
the scalar gap is wider on Zen 4 than on the M3 to begin with — the floats
get SSE2 on x86 while scalar int64 gets nothing, and 128-bit multiply chains
sting more at 3 GHz — which is exactly why this is the machine where the SIMD
had to grow back.

House rule: taunting Erin is only permitted once fixed point is close to or
beating his single-precision floats. At 2.8× we have not earned it, so there
will be no taunting today. But convex_pile is at 1.6× and the narrow phase
has not even been vectorized yet. Getting warmer. Preparing the nya.

## Should I use this?

No. **DO NOT USE THIS LIBRARY.** It exists to make one specific person mad.

## License

MIT, same as the real Box3D, which — once more — you should use instead:
<https://github.com/erincatto/box3d>
