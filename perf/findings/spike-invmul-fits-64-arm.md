VERDICT: LEARNED — the 64-bit arm is bit-exact (proven by a 10M-pair equivalence test plus the unchanged determinism goldens in both builds), but on this x86-64 box it is a wash: the 128-bit form already compiles to a single imul, so there is no speed to capture here, and the arm64 bench where the branch would pay is not available in this checkout. The harness the card depends on (fixed3d-perf-harness) is absent, so the specified measured/oracle gate could not be run to its letter. Do not adopt the patch on x86; the interesting measurement is arm64, which needs the harness and a real ARM bench.

# Spike: b3InvMul and b3InvFromRatio take a 64-bit arm when the product provably fits

## Hypothesis
Every mesh and convex impulse apply runs `b3InvMul(a, b) = (a*b + 2^39) >> 40` at 128 bits.
A typical invMass of 1 is raw 2^40 against an impulse near 2^16, a product near 2^56 that
fits in 64 bits. On arm64 the 128-bit multiply is software-emulated (several multiplies +
adds), while a proven-to-fit 64-bit multiply is a single `mul`. So a 64-bit arm gated on
"the product provably fits" should be a win on ARM; on x86 the 128-bit form is already a
single `imul` writing rdx:rax, so the branch is expected to be a wash — a legitimate LEARNED.

## The change
`b3InvMul` gains a second arm, taken when the leading-zero counts of the two absolute
operands sum to at least 66 (a strictly safe, conservative gate: it guarantees
`|a*b| < 2^62`, so `a*b + 2^39` cannot reach bit 63 and the single 64-bit multiply/add/
arithmetic-shift is UB-free and bit-identical to the 128-bit path). Zero operands return 0
up front. `b3InvFromRatio` drops its `fixInt128Div` for a single hardware divide
`((int64_t)1 << 56) / a` — the numerator 2^56 already fits int64, exactly as its own
comment (and `b3InvReciprocal`) already claim — keeping the identical saturation guards.

## Before / after
x86-64 (AMD Ryzen Threadripper 2990WX, avx2 only, gcc 15.2.0, Release+LTO, 1 worker,
joint_grid, 100 steps, 5 repeats, ms; run-0 dropped as warm-up):

| scene | base min/median | patch min/median | note |
|---|---|---|---|
| joint_grid | 6308.58 / ~6330 | 6282.25 / ~6297 | sub-1% wash, inside +/-2% spread |

The remaining target scenes (convex_pile, large_pyramid, many_pyramids, rain) were not
measured: the missing harness ab.sh/cpu_info.sh/oracle_check.sh and the 40-minute wall
together made them out of reach; only the standalone benchmark was available and its
convex_pile/rain scenes exceed the wall at 500/400 steps.

## Bit-exact evidence
`tools/perf/oracle_check.sh` is absent (missing fixed3d-perf-harness). The available
oracle is the repo's determinism goldens, and both are clean on the patch:
- `./bin/test` narrow (RelWithDebInfo + VALIDATE + Werror): all tests passed, exit 0.
- `./bin/test` ludicrous (same flags): all tests passed, exit 0.
- `InvMulFits64EquivalenceTest` (new, in MathTest) compares the 64-bit arm against a kept
  128-bit reference over 10,000,000 seeded pairs and the full gate-boundary/extreme set
  (0, +/-1, 2^30, 2^40, 2^61, 2^62, 2/3 * 2^62, INT64_MAX, INT64_MIN, INT64_MIN+1,
  -(2^62), -1-2^40) — zero mismatches on x86-64 (gcc).
- The determinism goldens (ragdoll, wave pile, query spawn, mesh drop) hold bit-identically
  across worker counts 1-5 in both builds, i.e. state hashes agree with the scalar oracle.

## Not a red-first test
`InvMulFits64EquivalenceTest` is an equivalence test, not a red/green regression: at the
unchanged BASE `b3InvMul` *is* the 128-bit form, so the comparison is trivially green at
base and cannot be "seen red first". That is inherent to a spike whose only deliverable is
"a new fast path must equal the old path"; the equivalence (10M pairs + boundaries) is the
whole point and it is green only because the new arm is genuinely bit-identical.

## What to try next
Run the SAME patch on arm64, where the 128-bit multiply is emulated and this branch is the
entire point of the card. Use the fixed3d-perf-harness (DEPENDS-ON) as specified, with its
ab.sh/cpu_info.sh/oracle_check.sh. If the target scenes show >= 3% geomean with no scene
>2% slower and oracle_check is clean at workers 1/2/4 in both modes, it becomes a KEEP.

## What not to repeat
- Re-measuring this on x86-64 and expecting a win: the 128-bit form is already one imul.
- The already-rejected ideas in the NOT REPEATING list (spherical cache, pointer variants,
  forced inlining, int32/Q16.16 lanes, tangent2 recompute, b3BodyState padding, fusing
  b3RotateVector/b3Lerp, NEON b3FloatW emulation, data-dependent guards in the wide loop).