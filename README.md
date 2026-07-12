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
back on AVX-512 and NEON — same bits, just faster; see below). In exchange,
resolution is a uniform 1/65536 everywhere in a ±1.4×10¹⁴ meter world, every
step is still bit-exact on every platform (vanilla Box3D already was — see
below), and all 22 unit test suites still pass.

```
45078b4 there i fixed it for you
```

## Profile results: fixed point vs. vanilla single precision

`benchmark -t=4 -w=4 -r=2` (4 workers, min of 2 runs, continuous collision on),
Apple M3 Ultra, macOS 26.5.1, Apple clang 21, RelWithDebInfo, Ninja.

- **float** = vanilla Box3D at `e961bfb` (single precision, NEON SIMD)
- **fixed** = this tree, scalar int64 lanes
- **fixed+NEON** = this tree with `-DBOX3D_NEON=ON` (narrow phase only — see below)

| Benchmark     | float (ms) | fixed (ms) | fixed+NEON (ms) | fixed/float | NEON/float | NEON speedup |
|---------------|-----------:|-----------:|----------------:|------------:|-----------:|-------------:|
| convex_pile   |   13,821.5 |   20,558.5 |        10,187.9 |       1.5× |  **0.74×** |        2.02× |
| joint_grid    |      267.9 |      774.8 |           768.4 |       2.9× |      2.9× |        1.01× |
| junkyard      |    4,713.5 |    9,676.6 |         8,596.0 |       2.1× |      1.8× |        1.13× |
| large_pyramid |      506.1 |    1,592.4 |         1,593.3 |       3.1× |      3.1× |        1.00× |
| large_world   |       13.4 |       22.6 |            22.7 |       1.7× |      1.7× |        1.00× |
| many_pyramids |      484.5 |    1,575.0 |         1,573.7 |       3.3× |      3.3× |        1.00× |
| rain          |      571.5 |    1,245.5 |         1,247.3 |       2.2× |      2.2× |        1.00× |
| trees25       |      209.0 |      351.1 |           342.5 |       1.7× |      1.6× |        1.03× |
| trees50       |      111.7 |      192.0 |           188.0 |       1.7× |      1.7× |        1.02× |
| trees100      |       86.5 |      145.7 |           149.9 |       1.7× |      1.7× |        0.97× |
| washer        |    6,365.3 |   13,089.4 |        13,064.5 |       2.1× |      2.1× |        1.00× |

**Geometric mean: 2.09× slower scalar, 1.94× with NEON — and convex_pile, the
most collision-bound scene in the suite, comes in at 0.74× of float: fixed
point beats the floats on Apple silicon too.** The optimization log with
per-pass numbers and sample profiles lives in
[benchmark/apple_m3_ultra_fixed](benchmark/apple_m3_ultra_fixed/README.md).

### Where the time goes

Both builds are solver-bound in the same functions on most scenes, so the gap
is pure arithmetic: a fixed-point multiply is `mul + smulh + add + shift`
against a single float FMA, and no 64-bit NEON multiply exists to vectorize
it away (on Zen 4 one exists — see the AVX-512 section; on Apple silicon the
narrow phase escapes through a 32-bit side door — see the NEON section).
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

NEON was never supposed to have a chance: fixed point needs 64×64-bit lane
multiplies, ARM keeps those in SVE2/SME, and Apple exposes neither on the M3
(`FEAT_SME: 0`; AMX is private). So the wide solver runs scalar on Apple
silicon, by decree of Cupertino. But Zen 4 ships `vpmullq` — a native,
single-µop, 64-bit vector multiply — and a Q48.16 solver is exactly the
workload it was born for. (NEON found a side door anyway — see below.)

`-DBOX3D_AVX512=ON` (default OFF) runs the whole hot path four lanes wide:
the contact solver (solve, warm start, restitution, prepare), the body
gather/scatter transposes, the hull support scans, and the SAT edge query's
Minkowski sign tests. One `vpmullq` + one `vpmuludq` + one `vpmuldq`
reproduce the exact 128-bit fixed-point product, dot reductions ride as
carry-free partial sums, and the narrow-phase scans take an int64 fast path
only when a conservative bound proves the exact dots fit (falling back to the
128-bit scalar scan otherwise). **Bit-identical to the scalar path for all
inputs** — same determinism goldens, same state hash on every thread count,
verified by a 25-million-case differential harness against the scalar
reference and the full test suite under ASan/UBSan. It is the same
simulation. It is just faster.

`benchmark -t=4 -w=4 -r=2` (4 workers, min of 2 runs, continuous collision
on), AMD EPYC 9124 (Zen 4), Ubuntu 24.04, clang 18, RelWithDebInfo.

- **float** = vanilla Box3D at `e961bfb` (single precision, SSE2 SIMD)
- **fixed** = this tree, scalar int64 lanes
- **fixed+AVX** = this tree with `-DBOX3D_AVX512=ON`

| Benchmark     | float (ms) | fixed (ms) | fixed+AVX (ms) | fixed/float | AVX/float | AVX speedup |
|---------------|-----------:|-----------:|---------------:|------------:|----------:|------------:|
| convex_pile   |   63,942.9 |  126,761.0 |       53,091.8 |       2.0× | **0.83×** |       2.39× |
| joint_grid    |    5,470.8 |   18,353.0 |       18,439.9 |       3.4× |     3.4× |       1.00× |
| junkyard      |   50,846.1 |  184,930.1 |      108,090.0 |       3.6× |     2.1× |       1.71× |
| large_pyramid |    8,582.3 |   56,672.0 |       26,282.7 |       6.6× |     3.1× |       2.16× |
| large_world   |       26.2 |       99.7 |           51.7 |       3.8× |     2.0× |       1.93× |
| many_pyramids |   11,843.7 |   53,265.0 |       27,587.2 |       4.5× |     2.3× |       1.93× |
| rain          |    6,659.8 |   24,157.0 |       20,874.7 |       3.6× |     3.1× |       1.16× |
| trees100      |      407.5 |      974.0 |          990.2 |       2.4× |     2.4× |       0.98× |
| trees25       |    1,020.4 |    2,382.0 |        2,359.6 |       2.3× |     2.3× |       1.01× |
| trees50       |      465.1 |    1,152.0 |        1,123.6 |       2.5× |     2.4× |       1.03× |
| washer        |   70,407.1 |  313,886.0 |      166,511.0 |       4.5× |     2.4× |       1.89× |

**Geomean: 3.4× of float scalar, 2.3× with AVX-512 — a 1.48× overall
speedup.** The solver-bound scenes land at 2.1–3.1× (large_pyramid
6.6× → 3.1×, washer 4.5× → 2.4×, junkyard 3.6× → 2.1×), the joint and tree
scenes are joint/mesh-bound and barely move, and convex_pile — the hull pile,
the most collision-bound scene in the suite — comes in **under the float
build**. The heavy lifting, in order: the wide solver, the wide contact
prepare, and finally the SAT edge query, which was 63% of convex_pile until
both of its Gauss-map sign tests went four-pairs-per-iteration over
precomputed edge vectors (survivors take the exact scalar path, so admission
is unchanged bit for bit). LTO adds another 1–3% on top if you want it
(`-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON`); the table is without it.

## The NEON side door

The M3 has no 64-bit vector multiply, but the AVX-512 round exposed a loophole:
the narrow phase runs on values its exactness gates already prove are small.
When every operand also fits in int32 — hull-local edge vectors, unit normals,
bounded directions, checked at runtime — NEON's native `smull`/`smlal`
32×32→64 widening multiplies compute the exact int64 dots four elements at a
time. `-DBOX3D_NEON=ON` (default OFF) runs the SAT edge query's two Minkowski
sign tests and both hull support scans that way, sharing the AVX-512
scaffolding: same SoA prepass (narrowed to int32 lanes), same shared scalar
body for survivors, same first-wins reductions. Anything that fails the gate
falls back to the 128-bit scalar scan, so results stay **bit-identical for
all inputs** — same goldens, verified under ASan/UBSan. The solver keeps its
scalar lanes on ARM: Apple's very wide scalar core wins that emulation trade,
and the table above shows the solver-bound scenes unmoved. The
collision-bound ones are a different story: convex_pile 20,559 → 10,188 ms.

## The taunt section

House rule: taunting Erin is only permitted once fixed point is close to or
beating his single-precision floats.

**convex_pile, AMD Zen 4: fixed point 53,092 ms, float 63,943 ms — 17%
faster. convex_pile, Apple M3 Ultra: fixed point 10,188 ms, float 13,822 ms —
26% faster. The bit-exact integer physics engine beats the floats on both
instruction sets.** nya nya nya.

Full disclosure, so the taunt stays legal: we did not out-multiply the FPU —
we out-vectorized it. Erin's float SIMD stops at the contact solver; his SAT
edge query is scalar on every platform. Ours tests four edge pairs per
iteration with exact integer sign tests, on AVX-512 and on NEON. Your floats
could do this too, Erin. That is the taunt: they don't. The geomeans are
still 2.3× (Zen 4) and 1.9× (M3), so the nya is scoped to where we won. For
now.

Post-script for the archaeologists: large_world used to report fixed point
4–20× slower. That wasn't the engine — the scene's drop placement went
through `B3_FIX(0.1f)` arithmetic that landed every sphere 55 mm off the
floor seams the float build hits exactly, the off-center impacts kicked the
spheres into eternal friction-proof rolling, and nothing ever slept. Forcing
the float build to the same landing spot reproduced the eternal rolling
there too. The engines agreed all along; the benchmark was running two
different worlds. Placement math in shared scenes now uses arithmetic that
is exact in both number systems.

## Should I use this?

No. **DO NOT USE THIS LIBRARY.** It exists to make one specific person mad.

## License

MIT, same as the real Box3D, which — once more — you should use instead:
<https://github.com/erincatto/box3d>
