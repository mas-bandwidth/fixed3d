# CORE-MATH rows

One file per row of CoreMathTest (policy: `docs/determinism-math.md`). A row card adds its
file here and touches nothing else:

| file | function | defines | runs | owning card |
|---|---|---|---|---|
| `atan2.c` | `int CoreMathAtan2Test( void )` | `B3_CORE_MATH_ROW_ATAN2=1` | fixAtan2, b3Atan2 vs cr_atan2 | cm-fixed3d-atan2 |
| `sincos.c` | `int CoreMathSinCosTest( void )` | `B3_CORE_MATH_ROW_SINCOS=1` | fixComputeCosSin, b3ComputeCosSin vs cr_sin / cr_cos | cm-fixed3d-sincos |
| `log2.c` | `int CoreMathLog2Test( void )` | `B3_CORE_MATH_ROW_LOG2=1` | fixLog2 vs cr_log2 | cm-fixed3d-log2 |
| `exp2.c` | `int CoreMathExp2Test( void )` | `B3_CORE_MATH_ROW_EXP2=1` | fixExp2 vs cr_exp2 | cm-fixed3d-exp2 |

`test/CMakeLists.txt` globs `core_math/*.c` into box3d_test and, for each file present,
defines `B3_CORE_MATH_ROW_<STEM>=1` (the upper-cased file stem). `test/test_core_math.c`
runs `RUN_SUBTEST( CoreMath<Name>Test )` under that `#if`, and otherwise prints
`core-math row=<stem> pending: cm-fixed3d-<stem>`. Re-run cmake after adding a file (the
glob is CONFIGURE_DEPENDS, so a build does it).

A row file includes `core_math_harness.h` and copies the sqrt row in `test_core_math.c`:
hard cases from `cmHardCases` with the edges of its bound pieces, 2^20 dense inputs in CI
(`cmRandomCount`, 2^28 under `B3_CORE_MATH_EXHAUSTIVE=1`, plus every raw in
[-2^24, 2^24] for univariate rows), the reference from `cmRefFromDouble( cr_<f>( ... ) )`,
and one `cmRowEnd` line per piece of its bound, named `<name>[<lo>,<hi>)`. Declare the row
function with a prototype first: the test target builds as strict C17 with warnings as
errors.
