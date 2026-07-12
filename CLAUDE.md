# Box3D Fixed-Point Conversion — Session Handoff

This working tree contains the conversion of Box3D from float to fixed-point math
(internal and external API). The conversion itself is committed (45078b4); the
session-3 performance pass below is a working-tree diff on top. Baseline float
code is at e9f6f1d.

## Current status (as of 2026-07-11, session 3 — performance pass)

**ALL 22 test suites pass** (`./build-fixed2/bin/test`, ~1.2 s). Single suite:
`./build-fixed2/bin/test <SuiteName>`.

**TWO TREES WARNING**: `/Users/glenn/box3d` is a separate clone. Commit 98b9889
("optimized to ~3.2x of float") was made there in a parallel session and has been
**fetched and fast-forwarded into this repo** (main is now 98b9889 + the session-3
work). `build-fixed/` and `build-ast/` here were copied from box3d and their
build.ninja files hold ABSOLUTE paths into /Users/glenn/box3d — building them
compiles the WRONG tree. Use `build-fixed2/` (fresh configure).

Session-3 performance work (all on top of 98b9889's fixed.h fast paths):

- **Raw 128-bit dot layer**: `b3DotRaw` / `b3FixFromDotRaw` in math_functions.h —
  exact int128 dot, one round-half-up at the end (divide last), overflow checks
  under `BOX3D_FIXED_SATURATE` like `b3FixMul`.
- **SAT edge queries** (`b3QueryEdgeDirections`, hull-capsule variant, triangle-hull
  `b3QueryTriangleAndHullEdges`, and both cached-axis revalidation blocks): Minkowski
  sign tests are now exact XOR sign tests on raw dots (no fixed products just to
  test a sign), `adc/bdc` are computed lazily after the `cba/dba` test passes, and
  `t = cba/(cba-dba)` divides the raw 128-bit dots directly. Cache-validate and
  full-query admission criteria are kept IDENTICAL — if you touch one, touch both.
- **Support scans** (`b3FindHullSupportVertex/Face`): argmax over raw int128 dots.
- **Wide solver reductions** (contact_solver.c): `b3CrossW`, `b3DotW`, `b3MulMVW`,
  `b3MulSubMVW`, `b3MulAddMVW`, `b3MulMV2W` now use per-lane raw-128 reduction
  helpers (`b3MulMulSubW`, `b3Dot2W/3W`, `b3Add/SubDot3W`) — one rounding per
  reduction instead of one per product.
- Normalize component divides routed through `b3FixDiv` (picks up its 64-bit fast
  path; identical truncating quotient).

Benchmarks (4 workers, this machine, min of 2 runs; "before" = the CSVs from the
morning run, "98b9889" = that commit alone; full A/B vs float rerun at ab2d210 —
CSVs and sample profiles in benchmark/apple_m3_ultra_fixed|_float/):

| benchmark      | before | 98b9889 | now (ab2d210) | float | ratio |
|----------------|--------|---------|---------------|-------|-------|
| convex_pile    | 46779  | 27279   | 20708         | 13626 | 1.52x |
| joint_grid     | 1555   | 809     | 810           | 271   | 2.99x |
| large_pyramid  | 4444   | 2295    | 2072          | 508   | 4.08x |
| large_world    | 162    | 92      | 85            | 14    | 5.99x |
| many_pyramids  | 4263   | 2246    | 2031          | 490   | 4.15x |
| rain           | 2637   | 1747    | 1356          | 582   | 2.33x |
| trees50        | 419    | 226     | 203           | 116   | 1.74x |
| washer         | 28767  | 18976   | 15305         | 6577  | 2.33x |

**Geomean ~2.85x of float** (was 5.4x pre-optimization, 3.2x after 98b9889).
~2.3x over the morning state. The sentinel-audit SAH fix was itself a perf win
for rebuild-heavy scenes (rain −22%, washer −13%). Profile is dominated by the
contact solver inner iteration (b3SolveContacts_Convex 27%, b3RotateVectorW 17%,
b3MulSub/AddMVW 17%) — narrow phase is ~6%. Float on the same workload is also
solver-bound but pays ~20% in SIMD gather/scatter that the scalar int64 lanes
don't; the residual gap is the 64x64->128 lane multiplies vs float FMA. Next
levers, in order: (1) NEON int64 2-lane vectorization of the wide solver (big
project), (2) `b3RotateVectorW` restructure (two crosses → matrix form per
body), (3) the remaining `__udivmodti4` calls (~2%).

**Benchmark CLI gotcha**: flags need the equals form (`-b=large_pyramid -w=4 -t=4
-r=2`). Space-separated flags are silently ignored and the FULL suite runs (looks
like a hang). It writes `<name>.csv` to the CWD.

Fixes landed in session 2 (beyond the session-1 list):

- **ShapeTest**: divide-last test references; `N - B3_FIX(1.0f)` int/fixed mixup that fed
  quickhull a degenerate cloud (hung `b3HullBuilder_ConnectFaces` — quickhull still has no
  iteration guard, see follow-ups); sub-resolution tolerance literals floored; on-ray
  reconstruction tolerance scaled by ray length.
- **b3RayCastSphere** rewritten: quadratic on the *raw* translation at 128 bits with a
  shift-normalization for huge coefficients. Errors now 1e-14..9e-3 over origins 1e1..1e7.
- **b3RayCastCapsule** rewritten: exact `b3Normalize` for the axis (was reciprocal-multiply,
  6 ulp error), the two-branch (Cramer + parallel) cylinder intersection replaced by a single
  128-bit perpendicular-plane quadratic valid for all directions, and a far-origin pre-pass
  that advances the ray to a padded bounding sphere so the solver runs on small local values.
  Far-origin test asserts capsules to 1e6 (vs float's 1e7 carve-out), sub-0.5 m at 1e6.
- **RecordingTest**: `b3RecW_F32`/`b3RecR_F32` still moved only 4 of the 8 bytes of a
  `b3Fixed` (reader returned uninitialized high bytes!) — wire widened to 64 bits; the raw
  byte-offset Step scanner in `b3RecScanFile` updated (dt at payload+4 is 8 bytes,
  subStepCount at +12); `B3_HASH_FLOAT` in `b3HashWorldState` widened to 8 bytes.

## Build workflow

- Dev build: `cmake --build build-fixed2` (Ninja, RelWithDebInfo, samples OFF,
  benchmarks ON, tests ON). **Do NOT use `build-fixed/` or `build-ast/`** — they
  compile /Users/glenn/box3d (absolute paths in build.ninja); delete or reconfigure
  them. Validation compiles only in Debug; a Debug build has NOT been run recently —
  `B3_ASSERT`/`B3_VALIDATE` conditions were all converted, but expect some asserts
  to need ULP slack when first run (pattern: see `b3GetTwistAngle` in
  math_functions.h).
- The AST audit needs a compile_commands.json that points at THIS tree; reconfigure
  build-ast (Debug, `-DBOX3D_DISABLE_SIMD=ON -DBOX3D_VALIDATE=ON`) before trusting
  `tools/fixed-point/ast_audit.py` again.
- Git worktrees were added for tooling and may now dangle (they live in a
  session-specific scratch dir): `git worktree list` / `git worktree prune`.
  One was a pristine copy for the rewriter; one (`floatref`) built the **float
  library at HEAD** for ground-truthing behavior — recreating that is the best
  way to answer "is this fixed-point behavior a bug or did float do it too?"
  (that trick resolved capsule rolling and ray-triangle winding questions).

## The fixed-point design

- **Format: Q48.16 in `int64_t`** (`b3Fixed`, [include/box3d/fixed.h](include/box3d/fixed.h)).
  Resolution 1/65536 ≈ 1.5e-5 uniform everywhere; range ±1.4e14; squared distances
  safe for |v| up to ~1e7 (B3_HUGE stays 1e5). Chosen over Q32.32 for overflow
  headroom (same format as Photon Quantum). `B3_FIXED_FRACTION_BITS` is central
  but not fully parameterized everywhere (Q32.32 trig constants are hardcoded).
- `+ - < ==` on `b3Fixed` are plain int64 ops. `b3FixMul` (128-bit, round-half-up;
  overflow checks are **opt-in** via `BOX3D_FIXED_SATURATE` since 98b9889 — default
  wraps, simulation values are far below range), `b3FixDiv` (truncating, saturating;
  64-bit fast path for numerators under 2^47; div-by-zero returns ±B3_FIXED_MAX with
  numerator sign, 0/0=0 — the "poor man's infinity"). `b3FixSqrt` is an exact
  integer sqrt (double-seeded + integer repair, still exact ⇒ deterministic).
  `B3_FIX(1.5f)` converts literals at compile time (constant-foldable, works in
  static initializers).
- **Integer scaling is idiomatic**: `2 * fixedValue`, `fixedValue / 4`,
  `n * B3_FIXED_EPSILON` are correct native ops. Only fixed×fixed and
  fixed÷fixed need `b3FixMul`/`b3FixDiv`.
- Trig (`b3ComputeCosSin`, `b3Atan2` in src/math_functions.c) and internal
  helpers use **Q32.32 intermediates** (`b3Q32Mul/Div/Sqrt`) — pure integer,
  cross-platform deterministic. cos/sin ≤ 0.0017 abs err; atan2 ≤ 3.5e-5.
- `BOX3D_DOUBLE_PRECISION` (large-world mode) is **removed** (`#error`); fixed
  point has uniform precision so `b3Pos == b3Vec3` always.
- Float SIMD (SSE2/NEON) is **removed**; `core.h` hard-defines `B3_SIMD_NONE`
  and the scalar `b3V32`/`b3FloatW` lanes are fixed-point. `B3_SIMD_WIDTH` is
  still 4 (wide constraint blocks work, four int64 lanes).
- MSVC x64 paths exist in fixed.h (`_mul128`/`_div128`/`__shiftright128`) but
  are **untested** (this machine is arm64 clang). ARM64 MSVC is `#error`'d
  (use clang-cl).
- Floats/doubles remain ONLY at non-simulation boundaries: `src/timer.c`
  (wall-clock), height field file load (`%lf` scan → `b3FixFromDouble`),
  test reference math vs libm, `b3FixToFloat/Double` converters for
  rendering/logging.

## Hard-won fixed-point rules (bug classes actually hit — do not reintroduce)

1. **Determinant underflow**: `FixMul(a,b)` of values < ~0.004 quantizes to 0.
   All matrix inverse/solve helpers (`b3InvertMatrix`, `b3Solve3`, `b3InvertT`,
   `b3Invert2`, `b3Solve2`) now compute determinants/cofactors at 128-bit
   internally (with a >2^62 cofactor fallback path for huge matrices). Symptom
   when violated: zero friction (tangent mass), rotation-locked bodies.
2. **Divide last**: never `FixDiv(1, x)` then multiply — the quantized
   reciprocal costs ~1e-3 relative error that subtractions amplify (hull mass
   Steiner term, /120 inertia scale). Pattern: `b3FixDiv(b3FixMul(a, b), c)`.
3. **Normalize precision**: `b3Normalize`, `b3Length`, `b3GetLengthAndNormalize`,
   `b3NormalizeQuat` compute the squared length as a raw 128-bit sum (exact) and
   divide components at 128-bit. Naive `FixMul` sums made GJK's exit normal
   garbage for the tiny search directions it terminates with (symptom: capsules
   reported distance 0 / "treat as overlap", nothing ever slept).
4. **Sentinel arithmetic wraps**: `-B3_FIXED_MAX - x` wraps to +huge (float
   `-FLT_MAX` saturates). Edge-query "no admissible pair" cases are now guarded
   via `indexB != B3_NULL_INDEX` in `convex_manifold.c` (hull-capsule deep path)
   and `triangle_manifold.c` (triangle-capsule). Multiplication too, and worse
   since b3FixMul stopped saturating: `b3FixMul(1000, B3_FIXED_MAX)` wraps to
   ~-1000 raw — this made the CCD stall threshold (default B3_FIXED_MAX, float
   used `1000 * FLT_MAX = inf`) log on every CCD call until guarded in
   solver.c/shape.c. Audit any new `sentinel ± value` OR `sentinel * value` code.
   All 63 sentinel sites were audited 2026-07-11: one more hit (dynamic_tree.c
   SAH scored degenerate planes with wrapped empty-AABB perimeters — float gave
   them NaN via 0*inf; now skipped like mesh.c already did). Rules of thumb from
   that audit: `b3FixMul(B3_FIXED_MAX, x)` only wraps for |x| >= 1 (joint force
   limits * h with h < 1 are fine); `MAX - positiveTol` cannot wrap; sentinel
   returns (b3CollideHullFace) are safe only behind pointCount short-circuits.
5. **Degenerate simplexes are common** in fixed point (support points quantize
   to identical values). GJK (`src/distance.c`) flushes cached simplexes with
   duplicate vertices and restarts (instead of restoring an empty backup) when
   a simplex solver fails on iteration 0.
6. **Tiny-body inertia floor**: bodies smaller than ~4 cm have true inertia below
   1 ULP. `b3FloorInertia` in src/body.c floors the diagonal at 4 ULP so rotation
   stays finite (symptom: |w| frozen forever, mesh-drop test never sleeps).
7. **Squared-tolerance guards collapse**: `FixMul(d, d) > 0` is false for
   d < √ULP ≈ 0.004. Use `d > 0` directly.
8. **Int-vs-fixed operand mixups**: `b3FixDiv(fixedValue, SOME_INT_MACRO)` is
   catastrophically wrong (found in height-field compression `UINT16_MAX`,
   ragdoll `RAGDOLL_GRID_COUNT` spacing, mesh BVH `B3_BIN_COUNT`). Conversely
   `sampleCount` in mesh_contact.c accumulates `B3_FIX(1.0)` and is already
   fixed — check which convention a variable uses before "fixing" it.
9. **scanf/printf holes**: varargs pointers (`%f` into a `b3Fixed*`) are
   invisible to type-based conversion. Grep for `SCAN`/`scanf` when touching I/O.

## Test conventions after conversion

- `ENSURE_SMALL(x, tol)` compares in fixed; the macro prints via
  `b3FixToDouble`. Tolerances below resolution were floored to
  `8 * B3_FIXED_EPSILON` (≈1.2e-4); tests that legitimately need more slack got
  measured, commented values (e.g. quat-between-vectors uses a 1/|half-vector|
  scaled tolerance; edge-axis scale test uses 48 ULP floor).
- Test *reference* math must use double libm + `b3FixFromDouble`, NOT
  `b3Sin`/`b3Cos` (the deterministic approximations carry ~1e-3 error and were
  never meant as references — see `ExactQuat` in test_manifold.c, the cylinder
  expectations in test_hull.c, and the trig comparisons in test_math.c).
- Determinism goldens (`test/test_determinism.c`): `EXPECTED_SLEEP_STEP 305`,
  `EXPECTED_HASH 0x1C6FD0EA` (updated in session 3 after the single-rounding
  reduction changes; verified bit-identical across 1-4 workers). Any
  solver-affecting change invalidates these: rerun, take the printed values,
  confirm they're identical for all worker counts before updating.
- `ATAN_TOL` in test_math.c is `B3_FIX(0.0001f)` (poly error + output quantization).

## Conversion tooling (copies in `tools/fixed-point/`, originals were in a
session scratchpad that may be gone)

- `fix_rewriter.py` — the one-shot clang `-ast-dump=json` driven float→fixed
  source rewriter (already applied; kept for reference). Key details: byte
  offsets (not chars!), sparse `"file"` fields must be tracked in document
  order, macro args edit at spelling loc / macro names at expansion loc,
  same-span wraps need tagged close-paren dedupe keys.
- `ast_audit.py` — **reusable safety net**: scans the *converted* tree (needs
  `build-ast/compile_commands.json`) for raw `b3Fixed * b3Fixed` /
  `b3Fixed / b3Fixed` binops that compile silently as integer math, and can
  `--fix` them. Caveats: skip int-literal operands (sugar inheritance makes
  `2 * B3_PI` look fixed×fixed), and function-like-macro operands get the close
  paren placed after the macro *name* token (grep for `B3_FIX )(` afterwards).
  Rerun after any large edit or merge from upstream float code.

## Known leftovers / follow-ups

- Run `./build-fixed/bin/benchmark`; `shared/benchmarks.c` is converted but
  never executed. Also check `data/dumps/single_box` — if anything replays
  recorded float-era data, it is format-incompatible (recording major version
  was bumped for this).
- Debug-config test run (asserts + B3_VALIDATE active) has not been done;
  expect a few asserts to need ulp slack.
- Quickhull (`b3HullBuilder_ConnectFaces`) has no iteration guard; a degenerate
  input hangs instead of failing. Consider a bounded walk + failure return.
- Samples: excluded. `include/box3d/math_functions.h` C++ operator overloads
  are converted, but sample .cpp files still contain float literals feeding
  `b3Fixed` params (silent truncation in C++) — needs a dedicated pass.
- Performance: correctness-first choices worth revisiting — bit-by-bit 128-bit
  sqrt (could seed from hardware sqrt + integer fixup), saturation branches in
  `b3FixMul`, 128-bit divides in normalize on hot paths. Benchmark before/after.
- Docs: README still describes float/SIMD/large-world features.
- `verstable.h` (vendored hash table) intentionally keeps internal floats
  (load factors only); excluded from conversion.
- A few sub-resolution guards were mapped `1000*FLT_MIN → 0` comparisons and
  `FLT_EPSILON → B3_FIXED_EPSILON`; `b3IsNormalized` uses 100 ULP,
  `b3IsNormalizedQuat` 100 ULP — tuned to pass, revisit if quat drift shows up.
