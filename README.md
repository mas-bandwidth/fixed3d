# Fixed3D: Box3D in Q48.16 fixed point

This fork exists to answer two questions:

1. **What would [Box3D](https://github.com/erincatto/box3d) look like if it
   was fixed point?**
2. **Exactly how much slower would it be?**

The answers: it looks like the code in this repository, and it is about
**2× slower** (geometric mean over the full benchmark suite, measured on
Apple silicon and on Zen 4 — full tables below). What you get in exchange is
one thing: a truly huge world with uniform precision everywhere.

That trade is narrower than it sounds, and **you should probably keep using
vanilla Box3D** — see [Should I use this?](#should-i-use-this) for the honest
comparison.

## What is this

Box3D with every `float` torn out of the simulation and replaced with **Q48.16
fixed point** in an `int64_t`. All of it: the solver, GJK, the trig, the ray
casts, the mass properties, the recording format. The float SIMD is gone (it
grew back on AVX-512 and NEON — same bits, just faster; see below). In
exchange, resolution is a uniform 1/65536 everywhere in a ±1.4×10¹⁴ meter
world, every step is still bit-exact on every platform (vanilla Box3D already
was — see below), and all 22 unit test suites still pass.

## Profile results: fixed point vs. vanilla single precision

`benchmark -t=4 -w=4 -r=2` (4 workers, min of 2 runs, continuous collision on),
Apple M3 Ultra, macOS 26.5.1, Apple clang 21, RelWithDebInfo, Ninja.
Measured 2026-07-13 at the current build defaults; all three columns were
re-run in the same session, float included.

- **float** = vanilla Box3D at `e961bfb` (single precision, NEON SIMD)
- **fixed** = this tree, scalar int64 lanes
- **fixed+NEON** = this tree with `-DBOX3D_NEON=ON` (narrow phase only — see below)

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

**Geometric mean: 2.07× slower scalar, 1.90× with NEON — and convex_pile, the
most collision-bound scene in the suite, comes in at 0.75× of float: fixed
point beats the floats on Apple silicon.** (That one win is likely
temporary — see [below](#where-fixed-point-wins--and-why-thats-not-the-flex-it-looks-like).)
The optimization log with per-pass numbers and sample profiles lives in
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

### What you get for the 2×

To be clear: **vanilla Box3D is already deterministic across platforms.**
Erin did that work in floating point — pinned contraction, no fast-math, the
discipline. Determinism is not a Fixed3D feature; vanilla already has it.
Fixed point merely gets it by construction instead of by vigilance: no FP
contraction flags, no `-ffloat-store`, no x87 anxiety — integers wrap the
same everywhere, so there is nothing to hold carefully.

What you actually get for the 2× is uniform 1.5×10⁻⁵ resolution at the
origin and at 100 km from the origin, in a ±1.4×10¹⁴ meter world. There is
no large-world mode in this tree because every world is a large world now.
Whether that is worth 2× is the question the
[Should I use this?](#should-i-use-this) section answers (short version:
probably not).

## The SIMD grew back: AVX-512 results

NEON was never supposed to have a chance: fixed point needs 64×64-bit lane
multiplies, ARM keeps those in SVE2/SME, and Apple exposes neither on the M3
(`FEAT_SME: 0`; AMX is private). So the wide solver runs scalar on Apple
silicon. But Zen 4 ships `vpmullq` — a native, single-µop, 64-bit vector
multiply — and a Q48.16 solver is exactly the workload it was born for.
(NEON found a side door anyway — see below.)

`-DBOX3D_AVX512=ON` (default OFF) runs the whole hot path four lanes wide:
the convex contact solver (solve, warm start, restitution, prepare), the
mesh contact solver (four whole contacts per constraint, manifolds
serialized in-register to keep the scalar solve order), the body
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

*(Table measured 2026-07-12, before three later changes: link-time
optimization became the build default (+1–3%), the hardware-divide fast
path landed (+1–2.5% on the joint-heavy scenes), and the mesh contact
solver went four-wide (trees100/trees50 measured 21–22% faster with
AVX-512, rain ~2% slower). Current builds run faster than the numbers
below.)*

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
6.6× → 3.1×, washer 4.5× → 2.4×, junkyard 3.6× → 2.1×), the joint scenes
are joint-bound and barely move, the mesh-bound tree scenes moved once the
mesh contact solver went wide (21–22% faster than this table — see the
note above), and convex_pile — the hull pile, the most collision-bound
scene in the suite — comes in **under the float build**. The heavy
lifting, in order: the wide solver, the wide contact
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
collision-bound ones are a different story: convex_pile 21,391 → 10,317 ms.

## Where fixed point wins — and why that's not the flex it looks like

convex_pile — a pile of convex hulls, the most collision-bound scene in the
suite — runs faster in this tree than in the float build on both instruction
sets: 53,092 ms vs 63,943 ms on Zen 4 (17% faster), 10,317 ms vs 13,725 ms
on the M3 Ultra (25% faster).

Full disclosure on the mechanics: the integer build did not out-multiply the
FPU — it out-vectorized it. Upstream's float SIMD stops at the contact
solver; its SAT edge query is scalar on every platform. This tree tests four
edge pairs per iteration with exact integer sign tests, on AVX-512 and on
NEON. Nothing about that requires fixed point — a float build could do the
same thing, more easily (no exactness gates needed). Everything we found
that transfers back to vanilla float Box3D — this included — is written up
in [ERIN.md](ERIN.md). The geomeans are still 2.3× (Zen 4) and 1.9× (M3);
the win is scoped to the collision-bound scenes.

Which means **the win is almost certainly temporary**. The vectorized narrow
phase is a technique, not a fixed-point advantage, and it is documented in
ERIN.md precisely so it can be backported to vanilla Box3D. Once it is,
float gets the same narrow-phase speedup on top of its faster arithmetic,
convex_pile flips back, and fixed point most likely returns to its natural
place: about 2× the cost, everywhere. Treat the row in bold as a snapshot
of an unfair comparison that time will correct.

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

Probably not. Check what you actually need against what vanilla Box3D
already does:

- **Determinism?** Vanilla Box3D is already deterministic across platforms,
  in floating point. Fixed point changes how that is achieved (by
  construction rather than by FP discipline), not whether.
- **A big world?** Vanilla Box3D already handles a 20,000 km cubed world
  with just ±1M of broadphase padding, and its large position support costs
  about 3% over standard float positions — against the ~2× this library
  costs.
- **Uniform resolution over a truly enormous range?** The same 1/65536
  everywhere in a ±1.4×10¹⁴ m world, with zero precision falloff away from
  the origin — this is the one thing this tree does that vanilla Box3D does
  not. If your world genuinely outruns what large positions plus broadphase
  padding cover, this library is the answer to your problem. Be sure that is
  your problem before paying 2× for it.

For everything else, vanilla Box3D almost certainly does what you need — and
it is the supported, maintained one: <https://github.com/erincatto/box3d>

## License

MIT, same as Box3D.
