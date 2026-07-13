# Box3D Fixed-Point Conversion — Session Handoff

Box3D converted from float to Q48.16 fixed point (internal and external API).
Baseline float code is commit e961bfb. EVERYTHING below is committed and pushed;
there is no pending working-tree state.

## Current status (as of 2026-07-13 — all work landed)

- **ALL 22 test suites pass** in Release (`./build-fixed2/bin/test`, ~1.1 s) AND
  in Debug + B3_VALIDATE + ASan/UBSan (exit 0, zero sanitizer reports). Single
  suite: `./build-fixed2/bin/test <SuiteName>`.
- **CI: fully green** — 14 jobs (ubuntu gcc / clang-TSan / clang-MSan, macos
  sanitized, windows-clang-cl, windows-arm64, windows-mingw, emscripten,
  ubuntu-clang-avx512 compile-only, six samples jobs). The avx512 job runs
  the suite opportunistically when the runner has the ISA. See the CI
  section for the rules that keep the matrix green.
- **LTO is default ON** for Release-family top-level builds since 13b73cc
  (CMAKE_INTERPROCEDURAL_OPTIMIZATION, gated by check_ipo_supported, skipped
  when BOX3D_SANITIZE, off switch BOX3D_LTO=OFF). Measured +1-3% on Zen 4;
  on M3 it is a wash (interleaved A/B 2026-07-13: convex_pile/washer flat,
  large_pyramid +1.7% — kept ON for the single-knob simplicity and the Zen 4
  win). All benchmark baselines must now be built with LTO or A/Bs conflate.
- **Performance vs vanilla float (geomean, all 11 benchmarks)**: M3 Ultra
  2.07x scalar / 1.90x with BOX3D_NEON (table re-measured 2026-07-13 at the
  current defaults, float re-run the same session); Zen 4 3.4x scalar /
  2.3x with BOX3D_AVX512 (README table kept at the clean 2026-07-12
  measurement — see below — with a footnote for LTO/div). **convex_pile
  BEATS float on both**: 0.75x on M3, 0.83x on Zen 4 (the README taunt is
  scoped to this and to the honest mechanics: Erin's float SIMD stops at
  his solver, our narrow phase is vectorized). TWO RULES learned refreshing
  tables on 2026-07-13: (1) absolute times drift a few percent with machine
  state, so a refresh must re-run the float reference in the same session
  or the ratios silently lie; (2) the space box is SHARED — check
  `pgrep -af benchmark` and the load average before benchmarking there. The
  attempted Zen 4 refresh was aborted: a concurrent session (build-wmesh in
  ~/fixed3d-avx, the mesh wide-ification chip) ran its own benchmarks on
  the box mid-suite, load hit 12, and every number came out up to 2x off —
  both sessions' data was garbage. The published Zen 4 table stands.
- **Samples build and run** (the float→fixed sample pass is done, including the
  newly re-enabled GyroscopicPrecession sample from e961bfb).
- **Determinism goldens**: sleepStep=287, hash=0x6FA8A4C5, verified bit-identical
  across 1-5 workers. Updated for the e961bfb friction-center weighted-average
  port — any solver-affecting change invalidates these, see the test conventions
  section for how to regenerate.
- **ERIN.md rule (from Glenn, 2026-07-13)**: ERIN.md at the repo root lists
  everything worth backporting into vanilla float Box3D — latent upstream
  bugs, Erin's in-code todos implemented and measured (wins AND rejections),
  SIMD designs that transfer, test-infrastructure lessons. KEEP IT CURRENT:
  when work lands that produces a float-applicable finding, add it to
  ERIN.md in the same or an adjacent commit. Discipline for entries: verify
  any claim about Erin's tree against upstream e961bfb first (cite his
  file:line), be honest about fixed-vs-float transferability of
  measurements (our multiplies cost ~4x his FMA), and the public-claims
  rule below applies there too.
- **Public-claims rule (from Glenn)**: vanilla Box3D is ALREADY deterministic
  across platforms — Erin achieves it in float with FP discipline. Never pitch
  determinism as a Fixed3D feature (README, commit messages, anywhere public);
  the honest differentiators are uniform resolution in an enormous world
  (large-world mode deleted) and making Erin mad. Fixed point only changes HOW
  determinism is achieved (by construction, no FP flags to police).
- History (main): e9f6f1d float baseline → 45078b4 + 98b9889 conversion →
  d29ef7d..a40134f optimization passes → 924cd56 narrow storage → ea684c7..632ff0d
  CI/samples → 973acd1 bug-hunt hardening → 1f1c941 friction center weighted
  average (ports box3d e961bfb) → a9f4dc8 AVX-512 wide solver + 2fdc189 timer
  fix → 45f5313/f0ffbf5 wide prepare → a501dfc point-slot skip + cd2b1b8
  gather transpose + eaf90ce support scans + d70126d/a9d1ecf SAT edge query
  (convex_pile beats float on Zen 4) → 5aca95a BOX3D_NEON (beats float on M3)
  → 8d32da5 gitignore build*/ → 6855c97/78ce3a0 large_world scene fix →
  cd00cfd compile guard on the unconverted-float SAH tree branch
  (B3_TREE_HEURISTIC != 0 now #errors: those #else branches were
  preprocessed out during the conversion, the AST rewriter never saw them,
  and they still do raw float math on b3Fixed) → 13b73cc avx512 CI job +
  LTO default → 4273c4c b3Int128Div hardware divide fast path (see the
  division section) → cd4b9a5 divq asm made volatile (gcc speculated it,
  see the known-issue bullet) → b218eb6..ede0457 wide mesh contact solver
  + B3_MESH_WIDE gate + README/doc refresh + repo sweep cleanups →
  67fa9e7/011f758 compound material dedup padding fix (b3StageMaterial,
  see the content-hashes rule).
  NOTE: main's history was force-push rewritten ONCE on 2026-07-12 (with
  Glenn's explicit approval) to purge 50MB of accidentally committed build
  dirs; any clone made in the ~30 minutes before that needs a reset.
- **Box clone state (ssh space, as of 2026-07-13 late)**: ~/fixed3d parent
  is checked out ON BRANCH main at 4273c4c with build/ (scalar) and
  build-avx2/ (AVX on) both rebuilt there with LTO, scalar goldens
  verified. ~/fixed3d-avx is a WORKTREE of ~/fixed3d, detached at the
  wide-mesh branch tip, with build-wmesh/ (AVX on, RelWithDebInfo, LTO)
  and build-wmesh-san/ (clang-18 Debug+VALIDATE+ASan/UBSan+AVX; plain
  `cmake` there picks gcc, which fails on a pre-existing
  -Wformat-truncation in scheduler.c under -Werror — use CC=clang-18);
  ~/wmesh-base is one more worktree at 57afe1f with build/ (AVX on, LTO),
  the baseline the wide-mesh A/B was measured against. ~/divcheck is a
  worktree on branch fixpad at 011f758 with test configs build-nolto (gcc
  AVX no-LTO — the CompoundMaterialDedup repro config), build-lto (gcc
  AVX+LTO), build-clang (clang-18 AVX+LTO), build-scalar (gcc scalar+LTO),
  plus build-bench/-base benchmark dirs (no test binary; also samples are
  OFF in the test dirs — a plain top-level configure there dies on a stale
  .fetchcontent-cache nfd, delete it or pass -DBOX3D_SAMPLES=OFF). The box is SHARED
  between sessions: check `pgrep -af benchmark` + load average before
  benchmarking (a standing root `/app/launcher server` process eats ~3.5
  cores, so absolute numbers run hot vs the published tables — trust
  interleaved A/B ratios only). Start any new box session with git fetch
  origin main.

## AVX-512 wide solver path (landed on main 2026-07-12)

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
- **Perf (AMD EPYC 9124, ssh space, 4 workers, min of 2)**: geomean 1.62x
  over scalar fixed across all 11 benchmarks; vs float e961bfb (SSE2) on the
  same box: geomean 3.36x scalar → 2.27x AVX (post large_world scene fix),
  and **convex_pile BEATS float
  at 0.83x** (53,092 ms vs 63,943 — the README taunt is scoped to this; the
  honest story is Erin's float SIMD stops at the solver while our narrow
  phase is vectorized too). Per-scene speedups over scalar fixed:
  large_world 5.4x, convex_pile 2.39x, large_pyramid 2.16x, many_pyramids
  1.93x, washer 1.89x, junkyard 1.71x; joint/tree scenes flat. LTO
  (CMAKE_INTERPROCEDURAL_OPTIMIZATION) adds a further 1-3%, measured but not
  made default. B3_SIMD_WIDTH=8 zmm remains unexplored (Zen 4 double-pumps
  512-bit; expected small). Benchmark dirs on the box: ~/fixed3d (main,
  scalar; build-avx2 = main with AVX on), ~/fixed3d-avx (branch, AVX on;
  bench-final has the published numbers), ~/box3d-float (upstream e961bfb
  float; e961bfb..e9f6f1d changed nothing in benchmark/, so scenes and step
  counts are identical across all three).
- **Round 2 (the narrow phase and the leftovers)**:
  (1) maxPointCount on the wide constraint (Erin's todo): prepare stores the
  widest manifold across the four lanes; warm start/solve/restitution loop
  only that far — slots past it are exact zeros, skipping is bit-identical,
  benefits scalar builds too. (2) b3GatherBodies/b3ScatterBodies: three 4x4
  int64 transposes over the 13 contiguous b3BodyState fields (dq.s scalar —
  a fourth row load would overread the array end; offsets pinned by
  _Static_assert). (3) b3FindHullSupportVertex/Face: four elements per
  iteration behind an int64-exactness gate (3 * max|dir| * bound < 2^63;
  vertex bound from hull->aabb, face bound 4*ONE for validated unit
  normals); exact values mean identical comparisons and ties; first-wins
  preserved via per-lane runs + value-then-smaller-index reduction + 128-bit
  tail. (4) b3QueryEdgeDirections — THE convex_pile lever (was 63% of it):
  inner body moved verbatim to b3TestEdgePair (shared by scalar loop and
  wide survivors so admission criteria cannot diverge); per-call SoA prepass
  of hull A's edge vectors AND adjacent face normals (uint8 edge indices cap
  hulls at 128 pairs — stack arrays); BOTH Minkowski sign tests run four
  pairs per iteration on un-negated dots ((dA^dB)<0 and (cba^dB)<0 replace
  the scalar adc/bdc sign flips); two runtime exactness gates from actual
  uB/vB/eB values and the AABB edge bound, each with a UINT64_MAX/3 guard so
  the 128-bit gate product cannot wrap; survivors (genuine Gauss-map
  intersections only) run the exact scalar body in ascending index order.
  Every step re-verified: goldens 287/0x6FA8A4C5 + full suite in Release AND
  Debug+VALIDATE+ASan/UBSan on Zen 4, plus scalar arm64 both configs.
- **Wide prepare**: b3PrepareContacts_Convex has a full wide variant under
  B3_SIMD_AVX512 (scalar version kept verbatim in the #else). The per-lane
  pass gathers and keeps b3Perp, b3Invert2, b3InvertMatrix, and lever-arm
  sqrts scalar; all other math runs wide with the exact per-site rounding
  (unfused b3Cross/b3MulMV compositions, fused b3DotW). KEY SUBTLETIES:
  (1) invInertiaWorld = (R*I)*R^T is NOT bitwise symmetric, so the wide math
  stages all NINE entries (b3Matrix3FullW) even though the constraint stores
  only the symmetric 6; (2) inactive (lane, point) slots are zero-fed or
  masked (b3MaskKeepW) so stored bytes equal the scalar zero-fill and setup
  memset exactly; (3) the per-point loop is bounded by max(pointCount) over
  the four lanes — without that bound, one-point-manifold scenes (rain,
  trees) paid for four slots. Result: prepare itself 1.6x faster (23% -> ~14%
  of the large_pyramid profile); residual staging tax on sphere scenes
  measured -1% rain / -6% trees100 (min-of-3 interleaved A/B; the trees
  scenes swing +/-10% run to run — always A/B before believing small-scene
  deltas). Remaining scalar targets: b3CollideTask (~14% of the profile),
  gather/scatter transposes (~6%).
- **Wide mesh contact solver (landed 2026-07-13; implements the wide-ification
  design that was sized in this doc — trees100 spent ~39% in
  b3SolveContacts_Mesh alone, ~50% in the whole mesh pipeline, rain ~7%, M3
  `sample` profiles at 1 worker)**: colored mesh contacts
  solve four contacts per b3ContactConstraintMeshWide (lane = whole contact;
  coloring keeps the eight gathered bodies disjoint), manifolds serialize
  in-register to preserve the scalar Gauss-Seidel order, and the ragged
  dimension (manifoldCount) is sized per group of four as the widest lane
  (flat slot array + per-slot start table built in solver setup). EVERY lane
  is bit-identical to b3SolveContacts_Mesh by construction: unfused crosses,
  full nine-entry b3MulMV with per-product rounding (invIA/invIB/rollingMass
  are NOT bitwise symmetric), unfused 2x2 tangent-mass products (scalar
  b3MulMV2/b3Dot2 are unfused, unlike the fused convex b3MulMV2W), the
  FixMul(-mass, x) forms multiply by the NEGATED mass (round-half-up is not
  odd-symmetric), and the anchor rotation is the exact unfused two-cross
  b3RotateVectorW — the fused b3Matrix3W rotation trick was differential
  tested and is NOT bit-identical to b3RotateVector (20M/20M random inputs
  mismatch), and DeterminismTest drops ragdolls onto meshes, so using it
  would break the goldens. Inactive (lane, manifold/point) slots hold exact
  zeros and compute exact zero deltas; the ONE non-self-neutralizing spot is
  rolling resistance (per-contact rolling mass), whose stored impulse is
  blended per lane (manifoldCounts mask AND rr > 0). Goldens 287/0x6FA8A4C5
  verified with the wide path running on BOTH arm64 scalar-emulated lanes
  (commit b218eb6, pre-gate) and Zen 4 AVX-512; overflow keeps the scalar
  functions (b3PrepareContacts_Mesh and friends stay).
  **B3_MESH_WIDE gates the path to AVX-512 builds**: on M3 the emulated
  lanes measured 32-46% SLOWER than the scalar colored path (min-of-3
  interleaved: trees100 143.9 -> 210.3 ms, trees50 184.6 -> 243.6, rain a
  wash) — same no-64-bit-vector-multiply trade as the wide solver, so
  scalar/NEON builds keep the scalar colored path (bit-identical either
  way; solver setup computes both count families and the branch constant
  folds). Zen 4 AVX-512 (min-of-3 interleaved, 4 workers, vs main 57afe1f,
  both LTO ON): trees100 1884.6 -> 1465.1 ms (-22%), trees50 2419.2 ->
  1901.1 (-21%), rain 57.4 s -> 58.4 s (+1.8%, all of it in the solve
  phase). The rain tax is NOT lane raggedness — rain's mesh contacts are
  all 1-manifold (waste ratio 1.00) while trees100 carries 49% wasted lane
  slots and still wins big — it is fixed per-slot/per-point overhead on
  tiny manifolds. Follow-up if it ever matters: a per-color indirection
  sorted by manifoldCount would homogenize groups (contact order within a
  color cannot affect results — bodies are disjoint), or a 1-manifold fast
  path.
- **NEON narrow-phase path (BOX3D_NEON, landed with the M3 work)**: the M3
  has no 64-bit vector multiply (FEAT_SME 0, no SVE2, AMX private), so the
  wide solver stays scalar on ARM (documented: Apple's scalar core wins the
  emulation trade; do not port b3FloatW to NEON without measuring first).
  The narrow phase escapes through int32: when the exactness gates ALSO
  prove every operand fits int32 (b3WideScanAdmissible/b3EdgeWideAdmissible
  have per-ISA clauses), smull/smlal 32x32->64 compute the exact int64 dots
  four elements at a time. Shares the AVX scaffolding: b3EdgeLane typedef
  narrows the edge-query SoA prepass to int32 (stores are unconditional
  truncating casts - only read when the gate passes), survivors go through
  the same b3TestEdgePair, argmax uses int64x2 lane pairs (vcgtq_s64+vbslq)
  with the same value-then-smaller-index reduction. Vertex xyz de-swizzling:
  vuzp1q_s32 64->32 narrowing + vqtbl3q_u8 byte-table gathers (b3_pickX/Y/Z).
  M3 Ultra results (4 workers, min of 2): convex_pile 20,558 -> 10,188 ms =
  2.02x, 0.74x of float e961bfb (fixed BEATS float on Apple silicon);
  junkyard 1.13x; solver-bound scenes flat by design; geomean 2.09x scalar ->
  1.94x NEON of float (post large_world scene fix). M3 bench protocol: macOS timer was always correct, CSVs are true
  ms; float reference = worktree at e961bfb built locally.
- **large_world scene bug (fixed on main)**: the benchmark reported fixed
  point 4-20x slower, but the engine was innocent — the scene's drop inset
  went through B3_FIX(0.1f) products (1000.061 vs float's exact 1000), every
  sphere landed 55mm off the floor-box seams float lands on exactly, the
  off-center impact caught the neighbor box's top edge and kicked spheres
  into eternal rolling (zero rolling resistance + pure rolling defeats
  friction), so nothing slept and the awake set grew monotonically. Float
  forced to the same landing rolls forever too (79/100 awake). RULE: shared
  benchmark scene placement math must be exact in BOTH number systems
  (integer scaling like halfSpan / 5), or the builds silently run different
  workloads. Post-fix large_world: M3 22.6 ms scalar vs float 13.4 (1.7x);
  EPYC 99.7 scalar / 51.7 AVX vs float 26.2. Diagnosis pattern that found
  it: per-phase b3Profile totals (scratchpad driver), then
  b3World_GetAwakeBodyCount over time (fixed grew, float plateaued), then
  per-sphere velocity traces showing v = w x r steady-state rolling.
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
- Follow-ups: B3_SIMD_WIDTH=8 zmm variant unexplored (Zen 4 double-pumps
  512-bit, expect small gains at best); prepare/warm start gather-scatter
  is still scalar; the mesh contact path is wide as of 2026-07-13 (see the
  wide mesh bullet above; scalar remains for overflow and non-AVX builds).

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

**Erin's tangent2-on-the-fly todo is measured and rejected** (2026-07-13,
branch `tangent2-on-the-fly` on origin, NOT merged): dropping the stored
`b3Vec3WN tangent2` (48 bytes off the wide constraint) and recomputing
cross(tangent1, normal) at warm start/solve/store-impulses with prepare's
unfused rounding IS bit-identical by construction (narrow unit vectors
round-trip losslessly; goldens and full suites passed on both ISAs) — but
it does not pay: interleaved min-of-3 A/B geomean +0.7% (slower) on M3
scalar (washer +1.8%, the int64 lanes pay 24 extra 128-bit multiplies per
constraint per pass) and +0.1% (wash, 9/18 pairwise wins each) on Zen 4
AVX-512 (washer −0.5% consistently was the only real signal). The 48-byte
load it saves is ~1.3% of a ~3.7KB constraint that streams anyway. Don't
redo this; the same math says don't chase the rtA2s/rtB2s rows either
(recomputing those needs the friction centers stored back, a wash on
bytes). The companion experiment — Erin's body.h todo_erin, padding
b3BodyState (112 bytes in fixed point) to 128 = two exact cache lines
with B3_ALIGNMENT raised to 64 (branch `bodystate-pad-128` on origin,
NOT merged) — is ALSO measured and rejected: M3 scalar geomean −0.4%
(faster), Zen 4 AVX-512 geomean +0.3% (slower), and the per-scene signs
expose it as an alignment lottery, not a systematic win (on Zen 4,
large_pyramid was −2.6% in all three passes while the nearly identical
many_pyramids workload was +2.9% in all three — reshuffled cache-set
conflicts, both well inside the swing such a change can produce by
accident). That completes the Erin-todo triage of 2026-07-13: everything
else in the codebase is dead API (b3GetShapeArea has no callers), dead
code (the guarded SAH branch), or upstream-shared design musings that
would just create merge pain with the box3d→fixed3d ports.

**b3Int128Div hardware divide fast path (landed 4273c4c, 2026-07-13)**: the
joint solvers spend ~9% of ragdoll scenes in __divti3/__udivmodti4 via the
128-bit cofactor divides in b3InvertMatrix/b3Solve3/b3Invert2/b3Solve2.
b3Int128Div (fixed.h) is exact signed 128/128 division — bit-identical to
`a / b` for EVERY input because integer division is unique — with x86-64
fast tiers: both-fit-64 → one hardware divide; divisor-fits-64 AND
uhi < v (proves the quotient fits, so the instruction cannot fault) → one
`divq` via inline asm. Non-x86 keeps plain `a / b`: Apple's libcall was
measured within 8% of a hand-written Knuth divide (fastdiv microbench:
1.08x arm64, 3.93x Zen 4 with divq; 20M-case differential fuzz + edges,
zero mismatches on both). Routed sites: b3FixDiv slow path, the matrix
inverse/solve helpers, b3Q32Div, the atan2 slope. Zen 4 A/B: joint_grid
−2.4% (3/3 passes), many_pyramids −1.0% (3/3), rain −1.2%. LESSON: the
first version also routed the SAT edge-query t divides and the raycast
quadratics — convex_pile went +2.3% (0/3), the always-inline tier checks
bloat the hottest narrow-phase loop for no benefit — so the narrow-phase
and raycast sites keep the plain `/`. Don't re-route them. MSan/TSan/gcc
are fine with the inline asm (CI green across the matrix). Also fixed in
passing: the huge-matrix path of b3InvertMatrix did raw `<<` on negative
128-bit cofactors (UB, never caught because the path needs cofactors
>= 2^62) — now b3Int128ShiftLeft.
POSTSCRIPT (2026-07-13, found during the wide-mesh AVX verification): the
divq asm MUST be `__asm__ volatile`. gcc assumes a non-volatile asm is
side-effect-free and trap-free, so it may SPECULATE it above the uhi < v
guard and the sign-magnitude negation — gcc 13 with -mavx512* did exactly
that (disassembly showed back-to-back unguarded `div %r9` fed raw signed
bits) and ManifoldTest's TriangleHullEdgeSweepTest died with SIGFPE on a
division whose true quotient was -1. clang never speculated it, scalar
gcc codegen happened not to, LTO irrelevant — which is why the 20M-case
fuzz and CI (compile-only avx512 job, scalar ubuntu jobs) never caught
it. RULE: any inline asm containing an instruction that can fault must be
volatile; and run the AVX suite with BOTH compilers on the box before
trusting an asm change.

**gcc AVX-512 SIGFPE in b3Int128Div — found during the wide-mesh AVX
verification, FIXED on main (cd4b9a5, 2026-07-13)**: gcc AVX-512 Release
builds (LTO or not) trapped in ManifoldTest's TriangleHullEdgeSweepTest.
Root cause (gdb + disassembly): gcc treats a non-volatile asm as
side-effect-free and TRAP-FREE, so it speculated the `divq` above both
the uhi < v quotient-fits guard and the sign-magnitude negation —
back-to-back unguarded `div %r9` fed the raw two's-complement bits of a
negative dividend whose true quotient was -1. Fix: `__asm__ volatile`
(pins the instruction to its branch; values unchanged, so bit-identical
by construction; the comment in fixed.h marks volatile as load-bearing).
Verified at the fix: gcc AVX+LTO full suite + goldens, gcc AVX no-LTO,
gcc scalar + goldens, clang-18 AVX, arm64; perf wash on joint_grid
(identical-config interleaved min-of-3). See also the POSTSCRIPT in the
division section above. Two lessons: (1) the buffered-stdout trap rule
strikes again — the full-suite run APPEARS to die around JointTest but
the lost buffer hides that ManifoldTest is the faulter; bisect suites
individually; (2) run the AVX suite with BOTH compilers on the box
before trusting a division/asm change — CI cannot (the avx512 job is
clang compile-only, the ubuntu gcc jobs are scalar). SEPARATE issue
found during that sweep, FIXED on main (67fa9e7, merged 011f758):
CompoundTest's CompoundMaterialDedup failed deterministically in gcc
AVX **no-LTO** builds — it WAS the raw-bytes-hash bug class, see the
content-hashes rule bullet for the mechanism and fix.

**Benchmark CLI gotcha**: flags need the equals form (`-b=large_pyramid -w=4 -t=4
-r=2`). Space-separated flags are silently ignored and the FULL suite runs (looks
like a hang). It writes `<name>.csv` to the CWD. Full suite = omit `-b`.

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
- Float SIMD (SSE2/NEON) is **removed**; `core.h` selects `B3_SIMD_NONE`
  unless the opt-in BOX3D_AVX512/BOX3D_NEON fixed-point paths are enabled,
  and the scalar `b3V32`/`b3FloatW` lanes are fixed-point. `B3_SIMD_WIDTH`
  is still 4 (wide constraint blocks work, four int64 lanes).
- Pure MSVC is **unsupported by design**: fixed.h `#error`s without
  `__SIZEOF_INT128__` ("use clang, gcc, or clang-cl on Windows"). The old
  `_mul128`/`_div128` MSVC intrinsic fallbacks are gone (audited 2026-07-13;
  the only `_umul128` left is in vendored verstable.h hash mixing). ARM64
  MSVC is likewise `#error`'d (use clang-cl).
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
  memset; keep it that way. Second instance (fixed 67fa9e7, 2026-07-13):
  b3SurfaceMaterial has 4 tail pad bytes and the compound material dedup map
  hashed/memcmp'd whole structs whose key pointers aimed straight into the
  CALLER's defs — gcc-13 AVX-512 no-LTO left different stack garbage in the
  padding and dedup broke (CompoundMaterialDedup). Fix: b3StageMaterial in
  compound.c copies field-by-field into the pre-zeroed materials slot and
  that slot is the map key, which also keeps the compound blob's material
  bytes deterministic for b3RecInternCompound's blob hash. Whole-struct
  assignment copies padding garbage — never struct-assign into memory that
  gets hashed or serialized raw. The same-class shape-storage exposure is
  FIXED too (2026-07-13): b3StageMaterial moved to shape.h and every shape
  material write stages — b3CreateShapeInternal's heap array (was raw
  memcpy of def->materials) and inline material (was struct-assign), plus
  b3Shape_SetSurfaceMaterial / b3Shape_SetMeshMaterial (by-value params
  carry caller padding) — so the bytes world_snapshot.c serializes raw
  (whole b3Shape image + materials array) are deterministic. Regression:
  ShapeMaterialStagingTest in test_shape.c (garbage-fills padding, memcmps
  stored bytes vs a staged reference; verified red pre-fix). height_field.c
  and mesh.c audited clean — their blobs hold only uint8 material indices.
  SEPARATE gap found during this fix (chip spawned): b3Shape_SetMeshMaterial
  has no B3_REC coverage at all, so recorded sessions that call it replay
  divergent.
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
- `data/dumps/single_box/box3d_dump.inl` CANNOT be regenerated and does not
  need to be (resolved 2026-07-13): it is generated C code shipped as data by
  upstream box3d — the world→C generator is Erin's offline tooling and is not
  in either repo. The B3_FIX-wrapped literals ARE the fixed-point port:
  quantization happens deterministically at compile time, the samples CI jobs
  compile the .inl, and the sample ran in the samples pass. The binary
  recording flow (b3CreateRecording / b3SaveRecordingToFile in sample.cpp) is
  a separate system, already fixed (64-bit wire) and covered by RecordingTest.
