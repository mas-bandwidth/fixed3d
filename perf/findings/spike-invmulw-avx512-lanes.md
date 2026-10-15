VERDICT: LEARNED — the >>16 K/P2/M split does NOT generalize to the 40-bit inverse shift as a cheap 3-multiply adaptation; the vectorization is not implemented and the before/after measurement could not run because the perf harness (DEPENDS-ON fixed3d-perf-harness) is not present on this BASE.

## Hypothesis

`b3InvMulW` (src/contact_solver.c, near lines 1869-1871) is four scalar `b3InvMul`
calls — the one place the impulse side of the AVX-512 wide solver still scalarizes
even though that build has a 64-bit lane multiply. The claim to test: decompose
`(a*b + 2^39) >> 40` with the same K/P2/M product split already proven bit-exact for
`(a*b + 2^15) >> 16` in `b3MulLanesAVX`, just adapting the shift (16 -> 40) and the
rounding constant (2^15 -> 2^39).

## The change (one paragraph)

No src/ change was landed. I wrote the unit test `InvMulWLaneEquivalenceTest`
(test/test_math.c, registered in `MathTest`), which pins `b3InvMul(a,b)` against an
independent `__int128` reference of `(a*b + 2^39) >> 40` over 10M seeded lane
quadruples, the sentinel/boundary set (B3_FIXED_MAX, INT64_MIN/MAX, the inverse-format
max, the ±2^39 rounding tie and its neighbours, ±1 ULP), and full-range int64 edge
pairs. It is red-first: with the reference (and the implied shift) left at 16 it fails
at `condition false: b3InvMul( bounds[bi], bounds[bj] ) == InvMulWLaneReference(...)`;
restored to 40 it is green. Then I derived the decomposition on paper and stopped at
the analysis below rather than commit an untested carry chain.

## Why the 40-bit split is not the 16-bit split

The 16-bit split (`b3MulLanesAVX`) costs exactly three hardware multiplies because the
low word `bl = b & 0xffff` is 16 bits, so `ahi*bl` (vpmuldq) and `alo*bl` (vpmuludq)
are 32x16 = 48-bit and stay exact in 64-bit lanes with no carry. The high word is a
single wrap-consistent `vpmullq` on `a * (b>>16)`.

With shift 40, `bl = b & (2^40 - 1)` is 40 bits, so `ahi*bl` and `alo*bl` are
32x40 = 72-bit and overflow a 64-bit lane. The minimal correct form splits `a` into
32-bit signed/unsigned halves and `bl` into 32+8 bits, giving **five** multiplies
(P00=alo*bl_lo, P10=ahi*bl_lo, P01=alo*bl_hi, P11=ahi*bl_hi, plus the wrap-consistent
`a*bh` vpmullq), and — the part that kills it — the round-half-up at bit 39 becomes a
**multi-word 64-bit carry chain**: `(a*b + 2^39) >> 40 == (a*b >> 40) + carry` where
`carry = ((a*b) & (2^40-1)) >= 2^39`, which requires propagating carries across the
32-bit lanes instead of the 16-bit case's clean `(ahi*bl << 16) + ((alo*bl + half) >> 16)`.
Roughly twice the multiplies plus full carry propagation against a scalar `b3InvMul`
that is a single 128-bit multiply + shift: the "single fast uop wins" premise of the
16-bit path does not survive the adaptation.

## Before / after

Not measured. The harness named in the card (tools/perf/ with `oracle_check.sh`,
`ab.sh`, `cpu_info.sh`, `tools/perf/README.md`) does not exist on this BASE — no
`tools/perf/` directory, and no `oracle_check.sh`/`ab.sh`/`cpu_info.sh` anywhere in the
tree. The `DEPENDS-ON: fixed3d-perf-harness` card has not landed, so the interleaved
before/after runs on convex_pile / large_pyramid / many_pyramids (BOX3D_AVX512 vs
scalar control) and the `oracle_check.sh` bit-exactness evidence are both unavailable.
This is recorded, not faked.

## Bit-exact evidence

Not measured (harness absent). The unit test itself is green in both build configs:
`./build/bin/test MathTest` and the BOX3D_AVX512 build both exit 0 with
`subtest passed: InvMulWLaneEquivalenceTest`. `./build/bin/test` (full scalar suite) is
green (exit 0, 2.70 s). The full BOX3D_AVX512 suite exits 1, but on a **pre-existing**
red unrelated to this card: `AllocatorWidenedSignatureTest` fails at
`condition false: !g_alloc.badAlignment` (B3_ALIGNMENT is 32 in the AVX-512 build, which
shifts the alignment the allocator test asserts); I changed no src/, so this is exactly
the BASE state. No wide-path change was made, so there is no divergence risk to check.

## Environment

- CPU: AMD Ryzen 7 255 (Strix Point, Zen 5), ISA flags present: avx2 avx512f avx512dq
  avx512vl — the bench DOES have AVX-512.
- Compiler: Ubuntu clang 21.1.8, gcc 15.2.0; cmake/make present.
- `taskset -pc $$`: affinity 0-15 (16 cores, unpinned). `nproc` 16.
- Load average at start: 1.56 / 0.68 / 0.38; no concurrent benchmark observed
  (this is a per-card jail, isolated cores belong to the host).

## What to try next

If the impulse-side scalarize is pursued: (1) accept ~5 multiplies + a carry chain and
**measure** whether it beats 4 scalar `b3InvMul` before investing — on Zen 4/5 the
scalar inverse multiply is a single 128-bit mul+shift and the wide version has to
amortize 5 lane multiplies plus carry logic over 4 lanes, which is a real chance to
lose; (2) alternatively widen only the *hot* form (`b3InvMulW` used inside the solve
loop's `b3InvMulSubSVW`/`b3InvMulSubMVW`) rather than the whole set; (3) buy the
measurement first by landing fixed3d-perf-harness.

## What not to repeat

Do not attempt the >>40 split by reusing the 3-multiply >>16 form verbatim — `bl` is
40 bits, not 16, and the intermediates overflow. Do not start the perf measurement
without `tools/perf/` present. Do not revisit the already-rejected ideas listed in the
card (spherical cache, forced inlining, narrowing, tangent2, B3BodyState padding,
fusing b3RotateVector/b3Lerp, NEON emulation, data-dependent guards).