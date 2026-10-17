# Determinism math: error bounds against CORE-MATH

Fixed3D is integer-only: every math function in libbox3d works on Q48.16 raws and gives
the same bits on every platform. This page says what each fixed-point transcendental and
root function is measured against, how far from it each may be, and where that is tested.
Glenn approved CORE-MATH as the reference on 2026-10-17.

## The reference

**CORE-MATH** (<https://core-math.gitlabpages.inria.fr/>, MIT) is a libm whose binary64
functions are *correctly rounded*: each returns the double nearest the exact mathematical
result. It is pinned at commit `cac142bc65dc140d645ba3f79511a9c591d1746e` (2026-10-01) and
vendored in `extern/core-math/` by `tools/revendor-core-math.sh`; `extern/core-math/NOTES.md`
records the URL, the hashes and any patch.

CORE-MATH is a **test-only** reference. It is built into `./bin/test` as its own object
library (`box3d_core_math`, GNU C, warnings off for those files only,
`-ffp-contract=off`, no LTO) and never into libbox3d: `nm libbox3d.a` shows no `cr_` symbol.
Nothing in the library calls floating point because of it.

- **Every fixed-point transcendental** (atan2, sin, cos, log2, exp2) is measured against
  CORE-MATH's correctly rounded double result at the pin, converted to Q48.16 by
  multiplying by 65536 (exact: a power of two) and rounding to nearest, ties to even
  (`cmRefFromDouble` in `test/core_math_harness.h`, independent of the FPU rounding mode).
- **fixSqrt** is measured against exact integer arithmetic, rounded to nearest
  (`cmRefSqrt`). CORE-MATH has no sqrt, and a double cannot hold a 63-bit raw:
  sqrt(raw / 2^16) in Q48.16 is sqrt(raw * 2^16), an integer square root of up to 79 bits.
- Where the double's ulp is coarser than a Q48.16 ulp (|result| >= 2^37, so
  |result| * 65536 >= 2^53), the double reference cannot resolve a raw unit and the bound
  is **relative**.

Errors are in Q48.16 ulps: the absolute difference of the two raws (`cmUlpDistance`,
computed unsigned so it cannot overflow).

## The stated bounds

| function | bound (Q48.16 ulps) | domain notes | row | owning card |
|---|---|---|---|---|
| fixSqrt | 1 | all raw > 0; raw <= 0 returns 0 | sqrt | cm-fixed3d-policy |
| fixAtan2, b3Atan2 | 2 | all inputs but (0, 0), which returns 0 | atan2 | cm-fixed3d-atan2 |
| fixComputeCosSin, b3ComputeCosSin (both outputs) | 107 for \|x\| < 32 rad; 108 + ceil(\|x\| / 16) for 32 <= \|x\| < 2^21 rad | **no bound at and above 2^21 rad** (known defect, below) | sincos | cm-fixed3d-sincos |
| fixLog2 | 0 | all raw > 0; INT64_MIN for raw <= 0 | log2 | cm-fixed3d-log2 |
| fixExp2 | 1 for x < 8; 5 for 8 <= x < 16; relative 2^-29 for 16 <= x < 47 | exactly INT64_MAX at and above 47 | exp2 | cm-fixed3d-exp2 |

No fixed-point rsqrt, acos, exp or natural log exists, so none has a bound.

**Known defect, sincos at and above 2^21 rad.** fixUnwindAngle reduces the angle by
subtracting a rounded 2*pi, so the reduction error grows with |x| and the angle is lost
entirely by 2^22 rad (error 131072 ulps = 2.0, the whole range of the result). The fix is
owed in mas-bandwidth/fixed.

**Where a fix lands.** fixSqrt, fixAtan2, fixComputeCosSin, fixLog2 and fixExp2 live in
mas-bandwidth/fixed, vendored at `extern/fixed` by `tools/revendor-fixed.sh`; that tree is
generated and checked by `.github/workflows/vendor-drift.yml`, never hand-edited. A fix to a
vendored function lands in mas-bandwidth/fixed and arrives here by moving the pin in
`tools/revendor-fixed.sh`. Only fixed3d's own copies in `src/math_functions.c` (b3Atan2 and
b3ComputeCosSin) are fixed in this repository.

## The harness

`./bin/test CoreMathTest` runs the rows. Each prints

    core-math row=<name> inputs=<n> maxulp=<m> bound=<b> worst=<raw>

(one line per piece of a piecewise bound, named `<name>[<lo>,<hi>)`) and fails when
maxulp > bound. Inputs are deterministic: a fixed-seed xorshift64 (seed
0x9E3779B97F4A7C15), never time.

- **Hard cases**, every row: raw 0, +-1, +-2; +-4096 raw around round(k*pi/2 * 2^16) for
  |k| <= 64; +-2^20 .. +-2^62; INT64_MAX and INT64_MIN + 1; each edge of the row's bound
  pieces and its neighbours.
- **CI size**: the hard cases plus 2^20 dense inputs per row; the whole suite runs in well
  under 2 s.
- **Exhaustive**, `B3_CORE_MATH_EXHAUSTIVE=1 ./bin/test CoreMathTest`: univariate rows run
  every raw in [-2^24, 2^24] plus 2^28 random raws; bivariate rows 2^28 random pairs.
- **The reversed witness**, CoreMathWitnessTest: the sqrt row on a planted function equal to
  fixSqrt except one ulp lower at a single input where fixSqrt is already one below the
  nearest (raw 10, then the first such raw of the dense random set). The row must go red with
  maxulp=2 and name that input; it prints `core-math witness red maxulp=2`.

Each transcendental row is one file under `test/core_math/` (see the README there); until
it lands the suite prints `core-math row=<stem> pending: cm-fixed3d-<stem>`.

## Measured at BASE 53f1352

Measured on hetzner1 (gcc 13.3) before this policy landed, reference as above (the double
result times 65536, rounded by llrint in round-to-nearest-even); errors in Q48.16 ulps.

- **fixSqrt** vs exact integer round-to-nearest of sqrt(raw * 2^16): maxulp=1 (fixSqrt
  floors; half the inputs differ), worst raw 10.
- **fixAtan2 and b3Atan2** vs cr_atan2: maxulp=2 (2^24 pairs in raw +-2^20, 2^22 in raw
  +-2^46).
- **fixComputeCosSin and b3ComputeCosSin** (identical results) vs cr_sin / cr_cos:
  maxulp=107 exhaustive over raw [-2^20, 2^20] (|x| <= 16 rad); 61 within 4096 raw of
  k*pi/2, |k| <= 8; by |x| band: [2^10, 2^11) 160, [2^18, 2^19) 14058, [2^20, 2^21) 54344,
  at and above 2^22 rad 131072 (angle lost: fixUnwindAngle subtracts a rounded 2*pi). The
  stated bound holds per input (excess <= 0 on every third raw to 2048 rad and 2^23 random
  inputs to 2^21 rad).
- **fixLog2** vs cr_log2: maxulp=0 on raw 1..2^24 exhaustive plus 2^22 random positive raws.
- **fixExp2** vs cr_exp2: x in [-16, 8) maxulp=1; [8, 16) 5; [16, 24) 1242; [24, 32)
  317896; [32, 36) 5086344; relative error <= 1.831e-9 (2^-29.03) over [16, 47) exhaustive
  (the Q32 table).

## Platforms

CORE-MATH is GNU C (`__int128` / `_BitInt(128)`, GNU inline asm under `__x86_64__` and
`__aarch64__`, `#pragma STDC FENV_ACCESS`). It is built unpatched on every CI
configuration in `.github/workflows/build.yml` and `extended.yml`. ClangCL on x64 adds
`-msse4.1` for the CORE-MATH files only, so `__builtin_roundeven` inlines instead of calling
the C23 `roundeven` the Windows CRT lacks; the flag is proven bit-equal over the dense sets
(`extern/core-math/NOTES.md`, Build flags). If a compiler ever cannot
build a file, the order of preference is the smallest patch under
`extern/core-math/patches/`, applied by the revendor script and proven equal to unpatched
CORE-MATH over the dense sets; only failing that are the rows compiled out under that
compiler, printing `core-math rows skipped: <compiler>`. That is safe because CoreMathTest is
a test-only accuracy check: the determinism goldens are what pin the integer library's bits
across platforms.
