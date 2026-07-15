# Fixed3D: Box3D in Q48.16 fixed point

This fork exists to answer two questions:

1. **What would [Box3D](https://github.com/erincatto/box3d) look like if it
   was *entirely* fixed point?**
2. **Exactly how much slower would it be?**

The answer: **2× slower** (geometric mean over the full benchmark suite,
measured on Apple silicon and on AMD Zen 4).

What you get in exchange is one thing: a truly huge world with uniform precision everywhere.

That trade is narrower than it sounds, and **you should probably keep using
vanilla Box3D** — see [Should I use this?](#should-i-use-this) for the honest
comparison.

## What is this

Box3D with every `float` torn out of the simulation and replaced with **Q48.16
fixed point** in an `int64_t`. All of it: the solver, GJK, the trig, the ray
casts, the mass properties, the recording format. The float SIMD is gone (it
grew back on AVX-512 and NEON — same bits, just faster). In
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

## Should I use this?

Probably not. Check what you actually need against what vanilla Box3D
already does:

- **Determinism?** Box3D is already deterministic in floating point across platforms.

- **A big world?** Vanilla Box3D already handles a 20,000 km cubed world
  with just ~1M of broadphase padding at 10,000km from origin, and its double
  position support costs just 3% over standard float positions.
  
- **Uniform resolution over a truly enormous range?** The same 1/65536
  everywhere in a ±1.4×10¹⁴ m world, with zero precision falloff away from
  the origin — this is the one thing this tree does that vanilla Box3D does
  not.

If your world genuinely outruns what large positions plus broadphase
padding cover, this library is the answer to your problem. Be sure that is
your problem before paying 2× for it.

For everything else, vanilla Box3D almost certainly does what you need: <https://github.com/erincatto/box3d>

## Maintenance

Fixed3D is maintained by [Glenn Fiedler](https://github.com/gafferongames) and
[Rowan](https://github.com/rowan-claude), Glenn's AI collaborator. New work
landing in vanilla Box3D gets ported across.

Issues specific to Fixed3D are welcome
[here](https://github.com/mas-bandwidth/fixed3d/issues). For issues with Box3D
in general, please use the
[vanilla Box3D repository](https://github.com/erincatto/box3d).

## License

MIT, same as Box3D.
