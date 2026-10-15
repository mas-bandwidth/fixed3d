VERDICT: DROP (bit-exact, but no measured gain on x86-64; the four-limb Knuth already shortens leading-zero limbs and uses hardware divq, so the dedicated 256/128 middle arm is a wash or slightly negative).

# spike-solve3-across-scales-256-by-128-arm

## CPU model / ISA / toolchain

```
model name : AMD EPYC 9124 16-Core Processor
ISA flags  : avx2 avx512dq avx512f avx512ifma avx512vl
taskset    : 0-31 (32 cores; no pin)
load avg   : 3.02 2.69 2.26 (before); low, shared box, no other benchmark running
compiler   : gcc (Ubuntu 15.2.0-16ubuntu1) 15.2.0 ; cmake 4.2.3
build      : RelWithDebInfo, -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON
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

`./bin/test` (narrow, RelWithDebInfo+VALIDATE) is green, all suites. The test run is
`./bin/test JointTest` (subtest `Solve3AcrossScalesMiddleArmTest`). Full `./bin/test`
duration 10.87 s. (Ludicrous build and the oracle_check/ab.sh harness were not run within
the 40-minute wall — see "not done".)

## Measurement (microbenchmark, NOT the ab.sh harness)

The full harness A/B (7 interleaved repeats, joint_grid/ragdoll_pile/trees25 + asteroid
controls, both narrow and ludicrous, oracle_check) could not run inside the wall. As a
stand-in, a standalone `-O2` microbenchmark isolated the two paths on identical inputs:

- Division alone, 20M divisions, determinant ~2^120: middle arm 1264 ms vs full Knuth
  1403 ms → **1.11x faster** on the division.
- End-to-end solve, 5M `b3Solve3AcrossScales` vs the full-path reference, ordinary-range
  matrices: middle arm 1391 ms vs 1316 ms → **0.95x (slightly slower)**.

The division win does not survive end-to-end: `fixUInt256DivMod` already shortens the
ordinary determinant to two limbs and its inner step is the hardware 128/64 `divq` on
x86-64, so the dedicated path saves only the limb-scan/normalization setup while adding a
`absDet.hi == 0` branch and a large always-inline body that bloats I-cache. Net is a wash
to slightly negative on Zen 4.

## Before/after table

No harness table was produced (wall). Microbenchmark summary above.

## What to try next

The named target "arm" is likely the real lead: on Apple silicon the profile shows
`__udivmodti4`/`__udivti3` in the joint solve (no hardware `divq`; `fixUInt128DivRemBy64`
falls back to native 128-bit division there). The 256/128 specialization may pay where the
divisor's two limbs force a software 128/128 divide, not on x86. A KEEP would require the
Apple bench to show the gain; re-measure there before adopting.

## What not to repeat

- Do not re-measure this on x86-64 expecting a win: the general Knuth already shortens and
  uses `divq`.
- Do not add data-dependent guards (the `absDet.hi == 0` branch) inside the wide solve loop
  — already ruled out, and this spike's end-to-end result shows the branch cost is real.
- Do not put the helper in extern/fixed before it proves itself on ARM (vendor-drift).

## Not done (wall)

- Full ab.sh harness A/B (7 interleaved repeats) on joint_grid/ragdoll_pile/trees25 + asteroid controls.
- BOX3D_LUDICROUS_MODE build and test.
- tools/perf/oracle_check.sh bit-exact gate (the oracle is a 78-row matrix; not run).
- Red-first at unchanged BASE (the differential test needs the middle arm to exist, so the
  red was structural rather than a failing line; the proof is the differential itself).
