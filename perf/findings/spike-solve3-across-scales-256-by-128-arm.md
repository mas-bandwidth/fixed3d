VERDICT: DROP (bit-exact, but no measured gain on x86-64; the four-limb Knuth already shortens leading-zero limbs and uses hardware divq, so the dedicated 256/128 middle arm is a wash).

# spike-solve3-across-scales-256-by-128-arm

## CPU model / ISA / toolchain

```
model name : AMD Ryzen 7 255 w/ Radeon 780M Graphics
ISA flags  : avx2 avx512dq avx512f avx512ifma avx512vl
cores      : 16
taskset    : 0-15 (no pin)
load avg   : 0.50 0.41 0.34 (before); 1.86 1.48 1.09 (after)
compiler   : gcc (Ubuntu 15.2.0-16ubuntu1) 15.2.0 ; cmake 4.2.3
build      : RelWithDebInfo, -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON
             -DBOX3D_BENCHMARKS=ON -DBOX3D_AVX512=OFF -DBOX3D_NEON=OFF (scalar)
```

## Hypothesis

`b3Solve3AcrossScales` (src/inverse.h) computes 128-bit cofactors and a 256-bit
determinant, then divides every shifted numerator through `fixUInt256DivMod`, the
general four-limb Knuth Algorithm D. An ordinary joint's per-mass determinant is near
2^120 (inverse-scale entries ~2^40, so cofactors ~2^80, determinant ~2^120), which fits
in 128 bits; the review (fixed3d issue #62 section 4) claimed this always pays the full
Knuth cost even though `fixUInt256DivMod` shortens leading-zero limbs. The hypothesis was
that a dedicated 256/128 division (the analogue of `fixInt128Div`'s x86 128/64 `divq`
fast tier) gated on `absDet.hi == 0` would cut the division cost for ordinary joints.

## The change (one paragraph)

Added a middle arm to `b3Solve3AcrossScales` in src/inverse.h: when the determinant's
high limb is zero (it fits in 128 bits), the three components divide through a new
`b3UInt256DivMod128` — a Knuth division specialized to a one- or two-limb divisor (the
`n==1` short division and the `n==2` two-limb body of `fixUInt256DivMod`, including the
rare add-back arm) wrapped by `b3DivShifted256By128` (same truncation and saturation as
`fixDivShifted256`). Determinants ≥ 2^128 (tiny-body cofactors ~2^107, asteroid cases)
keep the full four-limb path. Integer division is unique, so the two arms agree for every
input; the helper is deliberately left in src/inverse.h (not the vendor-drift-guarded
extern/fixed) so it could be measured without touching the library.

## Bit-exact evidence

`Solve3AcrossScalesMiddleArmTest` (test/test_joint.c, registered under JointTest) compares
the middle-arm solve against a verbatim copy of the pre-change full-256 path:

- 10,000,000 seeded matrices with entry magnitude spanning 2^-10 .. 2^40 — no divergence.
- Arm gate determinants 2^64-1, 2^64, 2^127 with 100,000 random 256-bit numerators each
  (`b3DivShifted256By128` vs `fixDivShifted256`) — no divergence.
- Asteroid regression (diagonal {6,70369,6}) and tiny-body regressions (diagonal entries
  2^41..2^60, forcing the full 256-bit path through the gate) — no divergence.

`./bin/test` (narrow, RelWithDebInfo+VALIDATE) is green, all suites (7.47 s). The
candidate also builds cleanly in BOX3D_LUDICROUS_MODE.

The harness A/B (`tools/perf/ab.sh build-base build`) exited 0 on all four target scenes
(joint_grid, ragdoll_pile, trees25, large_world) at workers 1/2/4 — no `HASH MISMATCH`,
so the middle arm is bit-identical to the scalar baseline on every measured scene.

The scalar oracle (`tools/perf/scalar-oracle.txt`) is STALE for this base: `oracle_check.sh`
reports ORACLE MISMATCH on bodies_20k, ragdoll_pile, rain (and others), but the BASELINE
(no middle arm) produces the SAME hashes as the candidate on those scenes
(e.g. ragdoll_pile `42fa9c6438e3527f` on both baseline and candidate, vs oracle
`4ac74cd9f1805daf`). The divergence is baseline-vs-oracle, not baseline-vs-candidate, so
the middle arm itself is bit-exact; the oracle needs regeneration on this base (out of
scope for this card, which may not commit tools/perf/scalar-oracle.txt).

## Measurement (harness A/B, interleaved, 7 repeats, --steps=120)

Step-phase speedup (baseline median / candidate median; >1 is faster). Full table in
perf/experiments/spike-solve3-across-scales-256-by-128-arm/raw.txt.

| scene        | w=1 step | w=2 step | w=4 step |
|--------------|----------|----------|----------|
| joint_grid   | 0.996    | 0.996    | 0.996    |
| ragdoll_pile | 1.009    | 1.010    | —        |
| trees25      | 1.000    | 0.999    | 0.991    |
| large_world  | 1.002    | 0.998    | 0.992    |

Geomean step speedup (trees25 + large_world, the completed run) = **0.997** — a wash,
inside every scene's A/A spread. The `solve`/`solveImpulses` phases (where the division
lives) show the same picture: joint_grid solveImpulses 0.978–0.980 (slightly slower),
ragdoll_pile solveImpulses 1.015–1.017 (slightly faster), all within noise. The division
win does not survive end-to-end: `fixUInt256DivMod` already shortens the ordinary
determinant to two limbs and its inner step is the hardware 128/64 `divq` on x86-64, so
the dedicated path saves only the limb-scan/normalization setup while adding an
`absDet.hi == 0` branch and a large always-inline body that bloats I-cache.

## What to try next

The named target "arm" is likely the real lead: on Apple silicon the profile shows
`__udivmodti4`/`__udivti3` in the joint solve (no hardware `divq`; `fixUInt128DivRemBy64`
falls back to native 128-bit division there). The 256/128 specialization may pay where the
divisor's two limbs force a software 128/128 divide, not on x86. A KEEP would require the
Apple bench to show the gain; re-measure there before adopting.

## What not to repeat

- Do not re-measure this on x86-64 expecting a win: the general Knuth already shortens and
  uses `divq`; the harness confirms the microbenchmark's 0.95–1.0x end-to-end wash.
- Do not add data-dependent guards (the `absDet.hi == 0` branch) inside the wide solve loop
  — already ruled out, and this spike's end-to-end result shows the branch cost is real.
- Do not put the helper in extern/fixed before it proves itself on ARM (vendor-drift).
- Do not trust tools/perf/scalar-oracle.txt on this base without regenerating it: it is
  stale (baseline itself mismatches on bodies_20k/ragdoll_pile/rain).

## Not done

- ragdoll_pile workers=4 row of the A/B was cut short by the 40-minute wall (the run was
  split across two invocations; joint_grid+ragdoll_pile w1/w2 in the first, trees25+
  large_world complete in the second). The missing row does not change the verdict.
- The oracle was not regenerated (out of PATHS; the card may not commit the oracle file).
