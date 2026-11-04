VERDICT: LEARNED — the retained scalar x86-64 A/B subset shows no keep-rule gain, but the correctness gate is incomplete and the archived 10-million-matrix test has undefined behavior. Do not adopt this patch.

# spike-solve3-across-scales-256-by-128-arm

## CPU model / ISA / toolchain

The retained attempt-3 log reports AMD Ryzen 7 255 w/ Radeon 780M Graphics,
AVX2 and AVX-512 DQ/F/IFMA/VL flags, affinity 0-15 (no pin), and load averages
0.50/0.41/0.34 before and 1.86/1.48/1.09 after. It reports GCC 15.2.0 and
CMake 4.2.3, RelWithDebInfo with validation, warnings as errors, benchmarks on,
AVX-512 and NEON off. The candidate ludicrous build is reported; no ludicrous
test or oracle run is preserved. This correction did not rerun a build or bench.

## Hypothesis and archived change

`b3Solve3AcrossScales` in `src/inverse.h` sends a 256-bit numerator and determinant
through `fixUInt256DivMod`. An ordinary inverse-scale joint can have a determinant
near 2^120, fitting 128 bits. The archived patch adds a specialized one- or
two-limb divisor routine in `src/inverse.h` and selects it when the determinant's
high 128-bit limb is zero; other determinants retain the full-width path. It
also adds a differential subtest to `test/test_joint.c`. The patch is an
experiment record, not an adopted source change. The vendored divisor already
shortens leading-zero limbs, which plausibly limits the benefit on this CPU.

## Correctness evidence and limits

The attempt-3 raw log reports a green narrow `./bin/test` and `JointTest`, plus
no divergence in the archived subtest's gate-value numerators, asteroid case,
and tiny-body cases. Those are sampled observations, not a proof of all-input
equivalence. In particular, the archived `MiddleArmEntry` generator chooses
`shift` in -10..40 (or -10..24) and evaluates `r >> shift`: a negative shift
count is undefined behavior in C. The fourth generated matrix entry already
selects -3. For nonnegative counts, shifting a random 64-bit word this way does
not generate the claimed 2^-10..2^40 Q48.16 magnitude sweep. The cast to
`int64_t` can also be implementation-defined for high-bit values, and negating
`INT64_MIN` would overflow. **The previously reported 10-million-matrix
scale-sweep proof is withdrawn.** The raw log and patch are retained unchanged
as historical evidence; their older proof wording is superseded by this finding.

The retained `oracle_check.sh` result is **EXIT=1, ORACLE MISMATCH**. For three
sampled narrow, workers=1, 120-step keys, direct baseline and candidate hashes
match each other but both differ from the frozen oracle:

| scene | baseline = candidate | frozen oracle |
|---|---|---|
| bodies_20k | ed59e6aff81bcd26 | 3e7cbde301a4f6c2 |
| ragdoll_pile | 42fa9c6438e3527f | 4ac74cd9f1805daf |
| rain | f96c9e008d59a375 | d3e59885dbca43e1 |

This supports stale-oracle diagnosis for those keys only. It does not establish
parity across every harness scene, workers 1/2/4, or narrow and ludicrous modes.
The archived helper also lacks the original `FIX_VALIDATE( representable )`
assertion before saturation, so validation behavior can differ when a quotient
is unrepresentable even if the returned saturated value agrees. Full bit-exact
acceptance has **not** been established.

## Retained measurement (scalar x86-64, interleaved A/B, seven repeats, 120 steps)

Step speedup is baseline median / candidate median; above 1 is faster. These
are the eleven preserved scene-worker rows, not the requested complete matrix.
The raw log contains each row's min, median, and spread; it retains only five
selected `solveImpulses` rows rather than every scene and phase.

| scene | workers=1 | workers=2 | workers=4 |
|---|---:|---:|---:|
| joint_grid | 0.996 | 0.996 | 0.996 |
| ragdoll_pile | 1.009 | 1.010 | not run |
| trees25 | 1.000 | 0.999 | 0.991 |
| large_world | 1.002 | 0.998 | 0.992 |

The first A/B invocation stopped before ragdoll_pile workers=4; only the second
(trees25 and large_world at workers 1/2/4) records EXIT=0. Its **six-row**
geometric mean is 0.997051. It is not the full target-scene geomean. The five
preserved `solveImpulses` speedups are 0.978139/0.980307/0.978747 for joint_grid
workers 1/2/4 and 1.017001/1.015374 for ragdoll_pile workers 1/2. No A/A run
or complete per-phase matrix is recorded, so no comparison to A/A spread or
phase-wide gain is justified. No instruction-cache or branch counters were
preserved; any explanation of the near-wash based on those effects is a
hypothesis. The measured subset supplies no 3% geomean gain or 1.5x phase gain.

## What to try next

If this idea is revisited, first replace the archived generator with a legal
raw-Q48.16 exponent construction and rerun its differential test. Then establish
all-scene, all-worker, narrow and ludicrous oracle parity against an oracle
valid for the chosen base, and collect the complete per-phase A/B matrix. An
ARM measurement might answer whether software wide division changes the cost
balance, but none is present in this record.

## What not to repeat

- Do not cite the archived undefined-behavior sweep as 10-million-case proof or
  call the patch unconditionally bit-exact.
- Do not treat three baseline=candidate hashes as a full oracle gate or silently
  regenerate the frozen oracle to make the result green.
- Do not use the six-row geomean as the full target matrix or claim an A/A noise
  bound without an A/A run.
- Do not adopt or tune this patch from these results; the correctness gate and
  measurement contract are incomplete.
