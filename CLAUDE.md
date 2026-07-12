# Box3D Fixed-Point Conversion — Session Handoff

Box3D converted from float to Q48.16 fixed point (internal and external API).
Baseline float code is commit e961bfb. EVERYTHING below is committed and pushed;
there is no pending working-tree state.

## Current status (as of 2026-07-12 — all work landed)

- **ALL 22 test suites pass** in Release (`./build-fixed2/bin/test`, ~1.1 s) AND
  in Debug + B3_VALIDATE + ASan/UBSan (exit 0, zero sanitizer reports). Single
  suite: `./build-fixed2/bin/test <SuiteName>`.
- **CI: fully green** — 13 jobs (ubuntu gcc / clang-TSan / clang-MSan, macos
  sanitized, windows-clang-cl, windows-arm64, windows-mingw, emscripten, six
  samples jobs). See the CI section for the rules that keep it green.
- **Performance: ~2.3x of vanilla float** geomean over all 11 benchmarks (was
  5.4x at the first conversion commit). Full optimization log with per-pass
  numbers and profiles: benchmark/apple_m3_ultra_fixed/README.md.
- **Samples build and run** (the float→fixed sample pass is done, including the
  newly re-enabled GyroscopicPrecession sample from e961bfb).
- **Determinism goldens**: sleepStep=287, hash=0x6FA8A4C5, verified bit-identical
  across 1-5 workers. Updated for the e961bfb friction-center weighted-average
  port — any solver-affecting change invalidates these, see the test conventions
  section for how to regenerate.
- History (main): e9f6f1d float baseline → 45078b4 + 98b9889 conversion →
  d29ef7d..a40134f optimization passes → 924cd56 narrow storage → ea684c7..632ff0d
  CI/samples → 973acd1 bug-hunt hardening → 1f1c941 friction center weighted
  average (ports box3d e961bfb).

## AVX-512 wide solver path (branch avx512, 2026-07-12)

- **BOX3D_AVX512** (CMake, default OFF) makes `b3FloatW` a union with one
  `__m256i` and implements every wide lane primitive in contact_solver.c with
  AVX-512VL 256-bit integer ops (needs AVX512F/DQ/VL; clang/gcc `-mavx512*`,
  MSVC `/arch:AVX512`). B3_ALIGNMENT becomes 32 under it (aligned constraint
  streams). Disabled under BOX3D_FIXED_SATURATE (implements wrapping b3FixMul
  only). Scalar path untouched; b3DivW/b3SqrtW stay per-lane scalar.
- **Bit-identical by construction**: `(a*b + half) >> 16` decomposes as
  `a*(b>>16) + ((a*(b&0xffff) + half) >> 16)` — one vpmullq (single-uop on
  Zen 4) + vpmuludq + vpmuldq, wrap-consistent mod 2^64 for ALL inputs
  including sentinels. Dot reductions carry `(K<<16) + (P2<<32) + M`
  (congruent mod 2^80, |P2|<2^51 |M|<2^53 stay exact through nine products
  plus a doubling; K wraps consistently). Round-half-up is not odd-symmetric:
  SubDot3W/RotDiagW accumulate the NEGATED sum then round (do not "simplify"
  to acc - round(S)). AddDot3W = AddW(acc, Dot3W(...)) is exact because
  acc<<16 splits out of the rounded shift.
- **Verified**: differential harness (tools/session scratchpad; 25M random +
  edge-pair multiplies, 8M x 8 reduction forms, full-range int64) zero
  mismatches; full suite + DeterminismTest (same 287/0x6FA8A4C5 goldens) in
  Release AND Debug+VALIDATE+ASan/UBSan with AVX on, on Zen 4.
- **Perf (AMD EPYC 9124, ssh space, 4 workers, min of 2)**: geomean 1.35x
  over scalar fixed across all 11 benchmarks — large_world 2.09x,
  large_pyramid 1.96x, many_pyramids 1.75x, washer 1.69x, junkyard 1.54x;
  joint/tree scenes flat (not wide-solver bound). vs float e961bfb (SSE2) on
  the same box: geomean 3.91x scalar → 2.89x AVX. b3SolveContacts_Convex
  itself 1.96x faster; **b3PrepareContacts_Convex is now the top remaining
  scalar target** (23% of the AVX profile). Benchmark dirs on the box:
  ~/fixed3d (main, scalar), ~/fixed3d-avx (branch, AVX on), ~/box3d-float
  (upstream e961bfb float; e961bfb..e9f6f1d changed nothing in benchmark/,
  so scenes and step counts are identical across all three).
- **Linux/Windows/Emscripten timer bug fixed on the branch** (2fdc189):
  b3GetMilliseconds cast double ms straight to b3Fixed (ms/65536 — only the
  macOS path used b3FixFromDouble). Any Linux benchmark CSV written before
  the fix needs values multiplied by 65536; the M3 CSVs are unaffected.
- **The `space` box**: `ssh space`, AMD EPYC 9124 (Zen 4, 16 cores, full
  AVX-512 incl. DQ/VL/IFMA), Ubuntu 24.04, clang 18, cmake 4.3.2, no ninja
  (make -j16), passwordless sudo. perf needs
  `sudo sysctl kernel.perf_event_paranoid=1` (default 4; restore after).
  GitHub is reachable over https; the fixed3d clone there has upstream main
  fetched (e961bfb resolves).
- Follow-ups: no CI job yet (GitHub runners aren't guaranteed AVX-512 —
  a compile-only job would work); B3_SIMD_WIDTH=8 zmm variant unexplored
  (Zen 4 double-pumps 512-bit, expect small gains at best); prepare/warm
  start gather-scatter and the mesh contact path are still scalar.

## Repository and remotes (IMPORTANT)

- This checkout (`/Users/glenn/fixed3d`) maps to **github.com/mas-bandwidth/fixed3d**
  (`origin`, push target; main tracks it). `upstream` is erincatto/box3d,
  fetch-only, with its push URL set to the invalid `DO-NOT-PUSH-TO-UPSTREAM` as a
  guard. NEVER push to Erin's repo. Glenn sometimes edits the README in the GitHub
  web UI — pull/merge before pushing if the remote is ahead.
- **TWO TREES WARNING**: `/Users/glenn/box3d` is a separate stale clone (its
  history was fetched into this repo long ago). `build-fixed/` and `build-ast/`
  dirs here were copied from it and their build.ninja files hold ABSOLUTE paths
  into /Users/glenn/box3d — building them compiles the WRONG tree. Use
  `build-fixed2/` (fresh configure). If a build behaves impossibly, check
  `grep -m1 /Users/glenn build*/build.ninja` FIRST.

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

Solver pass (after ab2d210, contact_solver.c only):

- **Anchor rotation via matrices** (Erin's todo): `bA.dq/bB.dq` are constant per
  wide constraint, so they become `b3Matrix3W` (rows; entries are fused raw-128
  reductions with the doubling as an exact shift) built once per constraint, and
  each anchor rotate is three `b3Dot3W` rows. `b3RotateVectorW` is gone.
- **Precomputed contact Jacobians**: prepare stores `cross(r, n)` and
  `invI*cross(r, n)` per point, plus `invMass*n`, `invI*n` (twist), and
  `cross(origin, tangent)` per constraint. The solve/warm-start/restitution
  impulse applications collapse to scalar*vector ops, and relative velocities
  project through `b3RelVelocityW` (nine products, one rounding). Trade-off:
  the wide constraint is fatter, so prepare/warm start pay more bandwidth —
  light scenes (joint_grid, trees50) gave back ~5-7%, solver-bound scenes
  gained 10-16%.

Round 3 (scalar math fusion — PARTIAL, read this before touching rounding):

- **Kept fused** (single rounding on raw-128 reductions): `b3Dot`,
  `b3LengthSquared`, `b3DistanceSquared`, `b3DotQuat`, `b3MulQuat`,
  `b3InvMulQuat`, plus `b3NormalizeQuat` component divides routed through
  `b3FixDiv` (bit-identical, picks up the 64-bit fast path). Wins: joint_grid
  −10%, rain −10%, trees50 −6% (quat ops feed joints, b3InvMulWorldTransforms,
  integration).
- **Deliberately NOT fused — do not "clean these up"** (each was bisected):
  - `b3Cross`: the single-rounding product difference carries a tie bias that
    acted as a phantom half-ulp torque; TestMeshDrop never slept (limit cycle
    at 4000 steps).
  - `b3MulMV`: shifting the SAT edge-query frame transform by an ulp put
    convex_pile into a persistent narrow-phase cache-miss regime, +40% wall
    time at identical contact counts.
  - `b3RotateVector`/`b3InvRotateVector` and `b3Lerp`: each independently
    re-rolled the TestMeshDrop sleep equilibrium into a limit cycle (b3Lerp
    via CCD sweep interpolation). Two configs that individually passed
    combined into failures — the equilibrium is a knife edge.
- **Lesson**: global rounding changes to scalar primitives are a lottery
  against equilibrium-sensitive scenes. Prefer call-site fusion in hot
  non-equilibrium code (the wide solver approach) over changing shared
  primitives. If a primitive must change, gate on TestMeshDrop AND a
  convex_pile wall-time check, not just the unit suites.

Benchmarks (4 workers, this machine, min of 2 runs; run-to-run variance ±2-5%,
washer up to ±10%; CSVs and sample profiles in benchmark/apple_m3_ultra_fixed|_float/):

| benchmark      | before | 98b9889 | ab2d210 | solver pass | round 3 | float | ratio |
|----------------|--------|---------|---------|-------------|---------|-------|-------|
| convex_pile    | 46779  | 27279   | 20708   | 21055       | 20917   | 13626 | 1.53x |
| joint_grid     | 1555   | 809     | 810     | 855         | 783     | 271   | 2.89x |
| large_pyramid  | 4444   | 2295    | 2072    | 1735        | 1638    | 508   | 3.22x |
| large_world    | 162    | 92      | 85      | 72          | 66      | 14    | 4.68x |
| many_pyramids  | 4263   | 2246    | 2031    | 1699        | 1669    | 490   | 3.41x |
| rain           | 2637   | 1747    | 1356    | 1380        | 1238    | 582   | 2.13x |
| trees50        | 419    | 226     | 203     | 217         | 205     | 116   | 1.77x |
| washer         | 28767  | 18976   | 15305   | 13840       | 14093   | 6577  | 2.14x |

**Geomean ~2.51x of float** (5.4x pre-optimization → 3.2x after 98b9889 →
2.85x after session 3 → 2.7x after the solver pass → 2.56x after round 3 →
2.51x after narrow constraint storage). Narrow storage keeps all constraint
geometry (anchors, normals, tangents, Jacobian rows) as int32 Q16.16
(`b3Vec3WN`, widened exactly on load — bit-identical simulation, goldens
unchanged) and shrinks the wide constraint ~30%; the numbers in the table
below are the round-3 column of benchmark/apple_m3_ultra_fixed/README.md,
see there for the narrow column.
The sentinel-audit SAH fix was itself a perf win for rebuild-heavy scenes
(rain −22%, washer −13%). Profile (large_pyramid, 1 worker) is flat:
b3SolveContacts_Convex ~24%, anchor rotation ~15%, prepare ~14%, warm start
~11% (memory-bound on the fatter constraint), b3RelVelocityW ~6%. Float on
the same workload is also solver-bound but pays ~20% in SIMD gather/scatter
that the scalar int64 lanes don't; the residual gap is the 64x64->128 lane
multiplies vs float FMA. NEON is NOT a lever on this hardware (no 64-bit lane
multiply without SVE2, which Apple doesn't expose).

**Q16.16 32-bit lanes are RULED OUT by measurement** (range audit 2026-07-12,
`-DBOX3D_RANGE_AUDIT` in contact_solver.c — reusable, prints a max-|value|
table at exit): convex_pile reaches normal impulses of ~946k units and total
normal impulses of ~5.8M (177x over the ±32768 Q16.16 limit), with masses
saturating 32k-65k. Don't revisit without a redesigned impulse representation.
The same audit shows all GEOMETRY/JACOBIAN fields (anchors, cross(r,n),
invI*cross(r,n), invMass*n, invI*n, cross(o,t)) stay under 225 units — they
fit int32 Q16.16 storage losslessly (values that fit round-trip bit-exactly),
which is the basis for the narrow-storage constraint layout (landed: `b3Vec3WN`,
bit-identical, ~30% smaller constraint). Remaining ideas: the `__udivmodti4`
calls (~2%), and not much else — see the branch audit below.

**Branchless is a dead end here too** (audited 2026-07-12): the compiled
b3SolveContacts_Convex has 56 csel against 33 conditional branches — clang
already lowers every lane ternary (Max/Blend/SymClamp/GreaterThan) to csel.
The 33 branches are loop control, the coherent useBias/rollingResistance
gates, and the b3FixDiv/b3FixSqrt internals inside the friction cone
projection. Gather/scatter compile nearly branch-free (warm start: 3 branches
total). The obvious counter-move — skip the friction sqrt+div when no lane
exceeds the cone — measured neutral-to-worse (large_pyramid within noise to
+4%): in never-sleeping benchmark scenes the saturation mask is per-lane
noisy, so the guard branch mispredicts, and the out-of-order core was already
absorbing the sqrt/div latency across the four independent lanes. Reverted;
don't re-add data-dependent guards to the wide solve loop.

**Benchmark CLI gotcha**: flags need the equals form (`-b=large_pyramid -w=4 -t=4
-r=2`). Space-separated flags are silently ignored and the FULL suite runs (looks
like a hang). It writes `<name>.csv` to the CWD.

Fixes landed in session 2 (beyond the session-1 list):

- **ShapeTest**: divide-last test references; `N - B3_FIX(1.0f)` int/fixed mixup that fed
  quickhull a degenerate cloud (hung `b3HullBuilder_ConnectFaces`; the builder is
  guarded now, see the bug-hunt section); sub-resolution tolerance literals floored;
  on-ray reconstruction tolerance scaled by ray length.
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
  benchmarks ON, tests ON). **Do NOT use `build-fixed/` or `build-ast/`** — see
  the two-trees warning above. `build-ast` may also be a dangling symlink into a
  dead session scratchpad; delete and reconfigure fresh when needed.
- Sanitized debug config (matches the macos CI job): configure a separate dir
  with `-DCMAKE_BUILD_TYPE=Debug -DBOX3D_VALIDATE=ON -DBOX3D_SANITIZE=ON
  -DCMAKE_COMPILE_WARNING_AS_ERROR=ON`. Both configs are expected green at all
  times. Debug asserts print to a BUFFERED stdout and the buffer is lost on
  trap — run under lldb (`lldb -b -o run -o bt -o quit -- bin/test Suite`) to
  see which assert fired.
- The AST audit (`tools/fixed-point/ast_audit.py`) derives the repo root from
  its own path and reads `build-ast/compile_commands.json`; reconfigure
  build-ast fresh (Debug, `-DBOX3D_DISABLE_SIMD=ON -DBOX3D_VALIDATE=ON
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`) before trusting it. Rerun it after any
  large edit or merge from upstream float code.
- Float ground truth: `git worktree add <scratch>/floatref e9f6f1d`, then cmake
  it standalone. Running the same scenario in the float build is the best way to
  answer "is this fixed-point behavior a bug or did float do it too?" — it
  settled the edge-query profile, capsule rolling, and ray-triangle winding
  questions. Worktrees live in session scratchpads and dangle when those die:
  `git worktree prune`.
- Samples build: `-DBOX3D_SAMPLES=ON -DBOX3D_UNIT_TESTS=OFF` (Release). If
  configure fails on a stale `.fetchcontent-cache`, delete it (it may be a copy
  from the box3d tree with baked absolute paths).

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
10. **Over-tight B3_VALIDATE canaries survive from upstream too** — not every
    B3_VALIDATE failure is a fixed-point bug. `b3CollideCapsuleAndTriangle`
    (triangle_manifold.c) asserted `faceSeparation <= 0` after clipping the
    capsule segment to the triangle face, but `b3BuildTriangleAndCapsuleFaceContact`
    only bails out (pointCount stays 0) when BOTH clipped points exceed
    `speculativeDistance + radius` — so a legitimate two-point speculative
    contact can land with both points positive but under `speculativeDistance`,
    which the very next comment in the file already documents ("Face contact
    can be empty if it does not realize the axis of minimum penetration").
    Upstream box3d has the identical `B3_VALIDATE( faceSeparation <= 0.0f )`
    verbatim — this was always a latent gap, just never exercised by Erin's
    own float test corpus. Only surfaced here because the e961bfb friction-center
    port changed the DeterminismTest ragdoll trajectory enough to walk into the
    configuration. Fix: bound the assert at `B3_SPECULATIVE_DISTANCE` instead of
    `0`, which is the actual provable invariant given the clip function's own
    early-return logic. Same pattern as the removed TOI conservative-advancement
    canary in the CI section — a debug-only sanity check being wrong is not the
    same as the algorithm being wrong.

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
- Determinism goldens (`test/test_determinism.c`): `EXPECTED_SLEEP_STEP 287`,
  `EXPECTED_HASH 0x6FA8A4C5` (updated for the e961bfb friction-center
  weighted-average port; verified bit-identical across 1-5 workers). Any
  solver-affecting change invalidates these: rerun, take the printed values,
  confirm they're identical for all worker counts
  before updating.
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

## CI (all green as of 2026-07-12)

The full matrix passes: ubuntu gcc/clang(TSan)/clang(MSan), macos (ASan+UBSan),
windows-clang-cl, windows-arm64 (clang-cl), windows-mingw, emscripten, and six
samples jobs. Hard-won CI knowledge:

- **__int128 is required**: pure MSVC cannot build the core; every Windows job
  uses `-T ClangCL`, and clang-cl needs `clang_rt.builtins-<arch>.lib` linked
  for `__divti3` (done in CMakeLists). The double-precision jobs are gone (the
  mode is deleted by design). No Windows ASan (clang-cl rejects /MTd with it).
- **UBSan**: never left-shift a signed value; use `b3FixShiftLeft` /
  `b3Int128ShiftLeft` (unsigned round-trip, same bits).
- **gcc -Wpedantic**: only mention __int128 through the `__extension__`
  typedefs (b3Int128/b3UInt128) in fixed.h.
- **Content hashes sweep raw bytes**: any struct memcpy'd into a hashed blob
  must have deterministic padding. Fixed point changed layouts — b3MeshNode
  grew 8 pad bytes that broke mesh dedup on gcc and tripped MSan until the
  temp node array was cleared. Blob allocations (mesh/hull/height field) are
  memset; keep it that way.
- `{ 0 }` is the universal zero initializer — `{ b3FixFromInt( 0 ) }` loses
  clang's missing-field-initializer exemption (157 sites were converted back).
- `b3IsValidFixed` accepts everything except INT64_MIN: the saturation values
  are legal (FLT_MAX-analog defaults in joint thresholds and spring limits).
- The TOI conservative-advancement failure path is a legitimate fixed-point
  outcome (quantized separations); its debug canary was removed.

## Samples (converted, build and run)

The dedicated float→fixed pass is done (~3800 sites): braced-init narrowing
wrapped with B3_FIX/b3FixFromFloat, 445 silently-truncating float literal
arguments wrapped with B3_FIX, 50 varargs %f holes wrapped with b3FixToDouble,
cast callbacks and their contexts converted to b3Fixed, `samples/fixed_ui.h`
provides SliderFixed/SliderFixed3/InputFixed for editing fixed fields, and the
debug-draw adapter converts at the render boundary. The C++11-narrowing error
plus -Wliteral-conversion and -Wformat warnings are the tools for auditing any
new sample code — keep them clean.

## Bug-hunt pass (2026-07-12, all landed in 973acd1)

- **b3FindFarthestPointFromLine** divided by a squared segment length that
  quantizes to zero for sub-resolution segments; `b3FixDiv(1, 0)` returned the
  saturation sentinel and the following non-saturating multiply wrapped —
  garbage initial hull seeds (rules 7 + 4 combined; the likely origin of the
  historical ConnectFaces hang). Now: exact `b3DotRaw` squared length,
  cross-multiplied threshold compare, coincident endpoints return no-index.
- **Quickhull is hang-proof and overflow-proof**: `failed` flag on the builder,
  bounded ring walks (cap = edge pool size) in ConnectFaces and the merge
  scans, release-safe pool allocators (capacity checks were assert-only — in
  Release, corrupt topology meant out-of-bounds pool writes; they now alias the
  last slot and fail the build), driver-level propagation to a clean NULL.
  Regression test: `CreateHullNearDegenerateTest` (sub-resolution jittered
  clouds — thin needles, near-planar grids, ulp clusters). That test flushed
  out the FromLine bug within minutes of existing.
- **b3UnwindAngle**: quotient stays 64-bit (was truncated through the 32-bit
  `b3FixRoundToInt`) and `n * twoPi` forms at 128 bits (overflows int64 near
  the b3Fixed range limit).
- **b3Log** now has the printf format attribute — mismatches used to be
  invisible to -Wformat (rule 9). All call sites were already correct; now the
  compiler enforces it.
- **ast_audit.py** had a hardcoded /Users/glenn/box3d root, so every earlier
  "clean" audit ran against the WRONG tree's compile database. Root is now
  derived from the script path; the first honest audit of this tree is clean.
- Checked clean: height-field grid indexing (floor-then-trunc matches float),
  the scanf boundary, b3MakeQuatFromMatrix/FromVectors, benchmark scenes under
  Debug+VALIDATE+sanitizers (10-step smoke, NDEBUG gates full step counts).

## Known leftovers / follow-ups

- ~~Quickhull iteration guard~~ DONE (bug-hunt pass): the builder has a
  `failed` flag, bounded ring walks in ConnectFaces/merge loops, release-safe
  pool allocators, and `b3FindFarthestPointFromLine` no longer divides by a
  quantized-to-zero squared length (exact raw-128 compare instead — the old
  reciprocal went through the 1/0 sentinel and wrapped). Regression test:
  CreateHullNearDegenerateTest.
- `verstable.h` (vendored hash table) intentionally keeps internal floats
  (load factors only); excluded from conversion.
- A few sub-resolution guards were mapped `1000*FLT_MIN → 0` comparisons and
  `FLT_EPSILON → B3_FIXED_EPSILON`; `b3IsNormalized` uses 100 ULP,
  `b3IsNormalizedQuat` 100 ULP — tuned to pass, revisit if quat drift shows up.
- `data/dumps/single_box` float-era literals are quantized via B3_FIX for the
  sample; regenerate the dump with the fixed build when the recording flow is
  exercised next.
