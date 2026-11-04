VERDICT: LEARNED — the 64-bit arm is bit-exact (proven by the harness oracle: the candidate reproduces the frozen scalar oracle on 12 of 13 scenes and matches the unpatched baseline bit-for-bit on the 13th, plus the 10M-pair equivalence test and green narrow+ludicrous suites), but on this x86-64 box it is not a win: the 128-bit form already compiles to a single imul, and the 64-bit arm's leading-zero gate adds branch overhead on the hot impulse-apply path. The clean before/after could not be measured because a sibling card kept a benchmark on the shared box for the whole run; the arm64 bench where the branch would pay is still absent. Do not adopt on x86; the interesting measurement is arm64.

# Spike: b3InvMul and b3InvFromRatio take a 64-bit arm when the product provably fits

## Hypothesis
Every mesh and convex impulse apply runs `b3InvMul(a, b) = (a*b + 2^39) >> 40` at 128 bits.
A typical invMass of 1 is raw 2^40 against an impulse near 2^16, a product near 2^56 that
fits in 64 bits. On arm64 the 128-bit multiply is software-emulated (several multiplies +
adds), while a proven-to-fit 64-bit multiply is a single `mul`. So a 64-bit arm gated on
"the product provably fits" should be a win on ARM; on x86 the 128-bit form is already a
single `imul` writing rdx:rax, so the branch is expected to be a wash-to-slight-slowdown —
a legitimate LEARNED.

## The change
`b3InvMul` gains a second arm, taken when the leading-zero counts of the two absolute
operands sum to at least 66 (a strictly safe, conservative gate: it guarantees
`|a*b| < 2^62`, so `a*b + 2^39` cannot reach bit 63 and the single 64-bit multiply/add/
arithmetic-shift is UB-free and bit-identical to the 128-bit path). Zero operands return 0
up front. `b3InvFromRatio` drops its `fixInt128Div` for a single hardware divide
`((int64_t)1 << 56) / a` — the numerator 2^56 already fits int64, exactly as its own
comment (and `b3InvReciprocal`) already claim — keeping the identical saturation guards.

## Bit-exact evidence (the harness, this attempt)
The DEPENDS-ON harness (tools/perf/) is present in this checkout and was run to its letter.

- `tools/perf/oracle_check.sh` (candidate = patched source, scalar, workers 1/2/4, narrow
  + ludicrous): the candidate reproduces the frozen scalar oracle for **12 of 13 scenes**.
  The one divergence is `bodies_20k` (`3e7cbde301a4f6c2` -> `adf418f3227d393e`), and it is
  **pre-existing, not caused by the patch**: the unpatched baseline build prints the same
  `adf418f3227d393e` on this machine. The oracle was regenerated on an AMD Ryzen 7 PRO
  8700GE (Zen 4); this box is a Ryzen 9 7950X3D, and the README notes "Cross-ISA
  verification of this exact base remains a separate gate". On `bodies_20k` the candidate
  and baseline agree bit-for-bit, so the patch itself is bit-exact.
- `tools/perf/ab.sh` hash check: no HASH MISMATCH on any of the five target scenes —
  candidate and baseline produce identical state hashes.
- `./bin/test` narrow and ludicrous: "All Fixed3D tests passed!", exit 0.
- `InvMulFits64EquivalenceTest` (MathTest): green — 10M seeded pairs + the full
  gate-boundary/extreme set (0, +/-1, 2^30, 2^40, 2^61, 2^62, 2/3*2^62, INT64_MAX,
  INT64_MIN, INT64_MIN+1, -(2^62), -1-2^40) all agree with the kept 128-bit reference.

## Before / after
x86-64 (AMD Ryzen 9 7950X3D, avx2/avx512f/dq/vl/ifma, gcc 13.3.0, RelWithDebInfo + LTO,
scalar, 1 worker, 120 steps, 7 repeats). **These numbers are CONTAMINATED**: a sibling
card (spike-hull-int32-soa-baked-in-the-blob.w1) kept a benchmark on the shared box for the
whole run (load rose 0.96 -> 3.2-4.8), so the magnitudes are not trustworthy and the
per-scene signs are inconsistent (joint_grid faster, the convex scenes slower) — the
signature of the concurrent load, not a real effect.

| scene | step A_med (ns) | step B_med (ns) | speedup |
|---|---|---|---|
| large_pyramid | 42,390,085 | 56,134,810 | 0.755 |
| many_pyramids | 116,282,453 | 142,540,598 | 0.816 |
| joint_grid | 34,837,561 | 32,216,253 | 1.081 |
| rain | 5,158,138 | 5,400,326 | 0.955 |
| geomean | | | 0.908 |

A clean re-run was not possible within the wall (the sibling benchmark never left the box).
The direction is consistent with the theory: on x86 the 128-bit form is already one `imul`
+ `add` + `shrd`, and the 64-bit arm's zero-check + `lzcnt` gate adds ~8 instructions on
the hot path, so it is a wash-to-slight-slowdown here, not a win.

## What to try next
Run the SAME patch on arm64, where the 128-bit multiply is software-emulated and this
branch is the entire point of the card. Use the harness as specified, on a box that is not
shared with another benchmark. If the target scenes show >= 3% geomean with no scene >2%
slower and oracle_check is clean at workers 1/2/4 in both modes, it becomes a KEEP. Also
resolve the pre-existing `bodies_20k` cross-machine oracle divergence (regenerate the
oracle on this box, or pin the scene's non-determinism) so the oracle gate is clean.

## What not to repeat
- Re-measuring this on x86-64 and expecting a win: the 128-bit form is already one imul,
  and the 64-bit arm's gate adds branch overhead.
- Running a benchmark on the shared box while a sibling card is benchmarking: the numbers
  come out garbage (this attempt's ab.sh table is the proof).
- The already-rejected ideas in the NOT REPEATING list (spherical cache, pointer variants,
  forced inlining, int32/Q16.16 lanes, tangent2 recompute, b3BodyState padding, fusing
  b3RotateVector/b3Lerp, NEON b3FloatW emulation, data-dependent guards in the wide loop).
