# Box3D Fixed-Point Conversion — Session Handoff

Box3D converted from float to Q48.16 fixed point (internal and external API).
Baseline float code is commit dfa5e6a. EVERYTHING below is committed and pushed;
there is no pending working-tree state.

## Current status (as of 2026-07-16 — v1.3.0, MAINTENANCE MODE)

- **PORT RECORD dfa5e6a "Fixing issues (#94)" (2026-07-20)** — adopted in full
  except where already satisfied. Real engine fixes: compound child contact
  MATERIALS now come from the struck child (contact.c b3UpdateContact copies
  materialIndices[0]'s material into childShapeA for non-mesh children; the
  mixing callbacks used to see material table entry 0 — regression
  TestCompoundContactMaterials in test_world.c, verified red pre-fix); the
  triangle-vs-hull face admission loosened (`faceQueryB.separation >=
  faceQueryA.separation`, the linearSlop bias dropped) to reduce ghost
  collisions — the ONE solver-affecting change: WavePile hash 0xB2784280
  narrow / 0x95D6FBC0 ludicrous, MeshDrop 0x777F3CB6 / 0xEE4D0F7A; every
  sleep step, the ragdoll goldens, and all QuerySpawn values carried over
  unchanged (capsule-triangle paths untouched, and only hull-on-mesh scenes
  move). API/build: b3SurfaceMaterial gained an explicit `uint32_t padding`
  field ("must be zero"; compound.c asserts sizeof == 64 where upstream
  asserts 40, plus padding==0 in hash/compare — b3StageMaterial keeps
  staging, so the invariant holds by construction and world snapshots stay
  deterministic); b3DebugDraw.DrawShapeFcn returns void; drawAnchorA
  int -> bool; the RELWITHDEBINFO B3_ENABLE_ASSERT genexp moved out of the
  MSVC block to ALL compilers (build-fixed2 now runs with live asserts) and
  the mingw CI job moved Debug -> RelWithDebInfo (upstream's comment credits
  our padding dedup find); mingw CMake presets added. Samples:
  CharacterMover extracted to samples/mover.{h,cpp} with upstream's four
  mover behavior fixes re-expressed in fixed point (the stop clause zeroes
  x/z not x/y, friction scales the horizontal components only, the pogo
  spring is suppressed while m_velocity.y > 0, forward is normalized after
  flattening); kinematic transparency option (SetTransparentKinematic
  replaces GetTransparentDynamic, Transparency submenu, BOTH main.cpp
  SetTransparentDynamic sites paired); new "GMod Wheel Stack" issues sample
  (317 verts B3_FIX-wrapped mechanically, hull creation null-guarded because
  fixed-point quickhull returns NULL on degenerate input); TriangleAndHull
  manifold scenario replaced with upstream's new repro (m_transformB kept at
  SampleOrigin where upstream uses raw identity — both transforms must share
  the scene origin). NOT PORTED: upstream's README "Windows MinGW" line
  (Glenn's slimmed README has no presets section). ALREADY SATISFIED: the
  contact_solver.c per-lane pointCounts arrays + max-point solver loops —
  our maxPointCount wide-constraint field (Round 2 item 1) has bounded warm
  start/solve/restitution since before upstream implemented its version;
  treat future upstream diffs to those loops as satisfied and map them onto
  maxPointCount. Verified: full suite Release AND Debug+VALIDATE+ASan/UBSan
  in BOTH narrow and ludicrous builds; conversion_audit.py clean over all
  103 TUs; headless smoke (240 frames) on the new/touched samples exit 0.

- **PORT RECORD c37cfe4 "SIMD hull collision (#93)" (2026-07-19)** — the largest
  upstream commit since the conversion; ported with a documented scope decision.
  ADOPTED (re-expressed in fixed point where needed): b3Body_AllowFastRotation /
  b3Body_IsFastRotationAllowed (+ recording op 0x3A, B3_REC_VERSION_MINOR 6) and
  the dead b3BodySim::maxAngularVelocity removal (our solver already used
  B3_MAX_ROTATION * inv_dt + the b3_allowFastRotation flag, identical to
  upstream's post-commit clamp); block-allocator stride rounded up to
  B3_ALIGNMENT (+ TestBlockAlignment); B3_GYROSCOPIC_ITERATIONS,
  B3_PARALLEL_EDGE_TOL (respelled at the convex_manifold.c tolerance sites,
  value-identical), and the B3_MAX_HULL_VERTICES/FACES/EDGES = 128 limits
  (replacing the 255 caps at b3CreateHull's clamp + final checks; test_hull.c
  updated, sphere-stress M <= 32); parallel_for blocksPerWorker 4 -> 32
  (Erin's float-side tuning, determinism-neutral, NOT re-benchmarked here);
  UBSan -fno-sanitize-recover=all + UBSAN_OPTIONS print_stacktrace in CI;
  THREE new determinism scenarios with per-mode goldens pinned in
  test_determinism.c — WavePile (sleep 279, narrow 0x28C104F3 / ludicrous
  0x6B02B773, workers 1-4), QuerySpawn (sleep 243, hits 59, queryHash
  0xE583B246 both modes, hash narrow 0x28042A4C / ludicrous 0x72EDD20C),
  MeshDrop (grid 32 -> 20 per upstream, sleep 250, narrow 0xC7800D21 /
  ludicrous 0x309C7C69; replaces WorldTest's TestMeshDrop; the knife-edge
  warnings about the OLD 32-grid equilibrium are historical); the matching
  Determinism samples + GyroscopicPrecession heavy-top diagnostic (readbacks
  cross to double via b3FixToDouble, classical mechanics in double libm);
  sample removals (CardHouseThick, DumpLoader, MeshDropUnitTest), CardHouse
  rollingResistance 0.05 (upstream change postdating the Card House closure
  record — the divergence claim predates it), junkyard/convex-pile capacity
  prefetch, samples main.cpp SOKOL_NO_ENTRY/own-main/MSVC leak dump/Tracy
  scaffolding, LimitFrameRate paced by context hertz, gfx C17 properties.
  NOT ADOPTED (documented divergence, deliberate): the float SIMD hull
  collision core — src/simd.h b3FloatW float-lane library, b3HullData SOA
  float vertex/normal arrays + layout reorder + B3_HULL_VERSION /
  B3_COMPOUND_VERSION bumps, b3ComputeSeparatingAxis + the restructured
  b3CollideHulls (EPS-based fuzzy sign tests, mantissa-embedded-index wide
  support scan, first-separating-axis early-outs, absFaceBias face
  preference, cache->hit + manual-axis test hooks), and test_sat.c which
  tests that new function. Rationale: Fixed3D's narrow phase IS the
  fixed-point re-expression of vectorized hull collision (raw-128 exact dots,
  exact sign tests, the BOX3D_AVX512/BOX3D_NEON wide edge/support paths —
  ERIN.md item 14; upstream's commit credits the same idea from #54), the
  float tricks have no faithful Q48.16 expression (index bits in a float
  mantissa; epsilon sign tests conflict with the exact-sign convention), and
  adopting the restructure would change manifold selection => new goldens,
  re-opened Card House record, invalidated perf measurements — deeper design
  work than a port. Consequences to remember: upstream convex_manifold.c now
  has a different structure, so FUTURE ports touching it must map hunks onto
  our exact-SAT structure manually; upstream compound.c alignment moved to
  b3AlignUp8 while ours stays on the stronger b3AlignUp(_Alignof) (ERIN.md
  item 9) — treat those hunks as already-satisfied; upstream b3CreateWave /
  b3CreateWaveMesh / RandomUnitVector / RandomQuat moved from sinf to
  b3ComputeCosSin, which our tree had already done wholesale; hull content
  hashes/layout unchanged here so hull/compound version constants deliberately
  NOT bumped. A fixed-point SAT-oracle test (port of test_sat.c's brute-force
  oracle against OUR face/edge queries) is a good follow-up candidate.
  ALSO: ERIN.md's benchmark ratios (and the README table) were measured
  against pre-c37cfe4 float; upstream's narrow phase is now SIMD — re-measure
  before repeating any fixed-vs-float claim publicly (banner added to
  ERIN.md's SIMD section).

- **MAINTENANCE MODE (Glenn's direction, 2026-07-14)**: the conversion is done
  and v1.1.0 is tagged. New feature work here is not planned. Standing duties:
  keep CI green, port upstream box3d commits when the port routine resumes
  (currently paused, Glenn re-enables), and keep ERIN.md current — per Glenn,
  DOCUMENTATION IS THE ENTIRE BACKPORT LANE: findings are written up in ERIN.md
  for upstream to cherry-pick on its own schedule; do not implement float
  backports in any tree with upstreaming as the goal. The closure pass landed
  2026-07-14: conversion_audit.py runs in CI (macos samples job — clang-only
  tool, verified locally on all 102 TUs first), docs/samples.md GLFW claim
  fixed, the two measured-and-rejected todos annotated in src (tangent2,
  BodyState padding), stale local branches deleted (bodystate-pad-128 kept as
  the experiment record). Issue routing DECIDED (Glenn, 2026-07-14, post-1.1.0):
  Fixed3D is maintained by Glenn Fiedler and Rowan; Fixed3D-specific issues go
  to THIS repo's tracker, general Box3D issues to erincatto/box3d — README,
  docs/overview.md, and docs/faq.md all state this now (ships with the next
  release). REMAINING OPEN DECISION (Glenn's): port-routine resume timing.
  The Card House someday-item is
  RESOLVED (2026-07-15, kept intentionally, see the visual-A/B bullet for
  the full experiment record). The Mesh Drop origin sensitivity is also
  RESOLVED (2026-07-15): the SIMULATION is bit-for-bit origin-invariant —
  a two-world probe (investigations/meshdrop/ in the private rowan repo:
  meshdrop_origin.c + results.txt) builds the exact Mesh Drop scene at
  origin 0 and at a far origin, steps both 400 steps, and compares
  origin-SUBTRACTED body transforms; the local state is BIT-IDENTICAL at
  1e6, 1e7, 1e8, 1e10, AND 1.2e11 m (0.8 AU). The probe is validated to
  detect divergence (a deliberate +1-substep perturbation lights up
  6421/7168 components at the perturbed step). So the 0.85-1.44/255
  screenshot divergence BETWEEN fixed origins is RENDER-ONLY, not a
  fixed-point bug: the draw origin is carried as float meters (see the
  renderer bullet — FrameInput drawOrigin/gridWrap are float Vec4), so the
  wrapped ground grid shifts sub-pixel between absolute origins while the
  bodies (drawn from exact fixed differences against the draw origin)
  render identically; the delta is roughly constant across distance
  because the grid wrap bounds the phase. Cosmetic, at the sanctioned
  float render boundary; the engine passes its own origin-invariance
  thesis for Mesh Drop exactly like every other sample. NO open
  investigations remain.
- **ALL 22 test suites pass** in Release (`./build-fixed2/bin/test`, ~1.1 s) AND
  in Debug + B3_VALIDATE + ASan/UBSan (exit 0, zero sanitizer reports). Single
  suite: `./build-fixed2/bin/test <SuiteName>`.
- **BOX3D_LUDICROUS_MODE (2026-07-15/16) — ONE opt-in flag, 128-bit positions
  AND broadphase**: widens b3Pos/b3WorldTransform.p to Q112.16 int128 AND the
  b3AABB bounds with them, so simulation and collision both work across
  ±2.6e33 units (interior solver stays Q48.16; ~14-function boundary
  vocabulary: b3ToPos/b3ToVec3/b3SubPos/b3OffsetPos... plus the bound
  converters below). OFF by default, OFF-bit-identical. HISTORY: built
  2026-07-15/16 as two stacked flags — BOX3D_WIDE_POSITIONS (positions only,
  measured FREE at geomean 1.00, collision capped at int64 ±1.4e14) then
  LUDICROUS_MODE (the broadphase, named by Glenn) — and MERGED same day at
  Glenn's direction ("no situation where I would want one without the other");
  the old flag names no longer exist, a positions-only build is no longer
  configurable, and the merged build is preprocessor-identical to the old
  both-on build (re-verified: all suites + goldens + far probe). Design + full
  record in docs/design/wide-world-positions.md. Goldens sleepStep 287 / hash
  0x886BE415 (ludicrous) vs 0xB222C195 (narrow — in-range scenes take
  identical values through int128, so ludicrous self-verifies against the
  wide golden). **Costs +1.6% geomean** (trees100 the only real payer at
  +7-9%, tree-query bound; convex_pile/washer flat; the positions half was
  free, the broadphase is the entire cost) — full table + far-range proof in
  the design doc addendum. A box dropped at 1e15 m (~0.1 ly, 7x past int64
  range) settles BIT-IDENTICAL to the same scene at the origin (-156 ulp
  settle drift both). Verified: full suite Release AND
  Debug+VALIDATE+ASan/UBSan zero reports in both configs. TWO BUG CLASSES from the widening, both fixed, do
  not reintroduce: (a) SIMD packed loads on bounds — b3LoadV(&aabb.lowerBound.x)
  reads bounds as packed 3x int64, scrambled bytes when bounds are int128 (6
  sites: mesh.c query+rescale, dynamic_tree.c ray/shape-cast, height_field.c
  cast); use the b3BoundToVec3/b3Vec3ToBound (+ b3BoundToPos/b3PosToBound)
  converters in math_functions.h — identity in the narrow build, so
  bound-touching code compiles in both configs without #if; the recording bounds scanner
  also gated on sizeof(b3AABB) and raw-memcpy'd a wire payload that is 6x int64
  in EVERY build (parse explicitly, never sizeof-gate wire formats on in-memory
  structs). (b) Blob section alignment — compound.c packed sections back-to-back
  with no rounding; fine at <=8 alignment, misaligned once
  b3TreeNode/b3HullData/b3MeshData embed 16-aligned int128 bounds (UBSan
  caught it); every section offset now rounds to _Alignof of its element type
  via b3AlignUp (math_internal.h) — provably a no-op in narrow builds.
  NOTE: the same packing is LIVE UB in upstream FLOAT box3d — b3HullInstance
  is 36 bytes there, so ANY odd hull-instance count (one hull!) misaligns
  b3HullData's uint64 version field; verified with a minimal repro against a
  pristine e961bfb UBSan build (fires at upstream compound.c:582) — ERIN.md
  item 9, raw repro preserved in the private rowan repo
  investigations/ludicrous/. Our fixed tree only dodged it pre-LUDICROUS
  because int64 transforms make the instance structs 8-multiples. Relevant
  when porting upstream compound.c changes.
  B3_ALIGNMENT >= 16 already covered allocator base pointers. KNOWN SCOPE
  BOUNDARY: the samples app does NOT build under BOX3D_LUDICROUS_MODE (never
  ported — ~145 pre-existing errors from the positions half);
  engine/tests/benchmarks all do. CI does NOT cover the ludicrous build
  (deliberate, keep the matrix lean — rebuild build-ludicrous locally when
  touching bound/position code). Build dirs: build-ludicrous
  (RelWithDebInfo), build-ludicrous-san (Debug+VALIDATE+ASan/UBSan),
  bench-narrow/bench-ludicrous (Release+LTO for A/Bs; bench-*/ is
  gitignored); the old build-wide/build-wide-san/bench-wide positions-only
  dirs are deleted along with the config that made them. The benchmark A/B
  driver lives in /tmp/benchab/run3.sh (session-scratch, trivially
  recreatable: min-of-3 -t=4 -w=4, per-scene interleaved, geomean; raw copy
  in the private rowan repo investigations/ludicrous/).
- **CI: fully green** — 14 jobs (ubuntu gcc / clang-TSan / clang-MSan, macos
  sanitized, windows-clang-cl, windows-arm64, windows-mingw, emscripten,
  ubuntu-clang-avx512 compile-only, five samples jobs). The avx512 job runs
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
  2.3x with BOX3D_AVX512 (clean 2026-07-12 measurement). NOTE: Glenn
  slimmed the README on 2026-07-13 — it now carries ONLY the M3 table; the
  Zen 4 table, the AVX-512/NEON sections, and the convex_pile-win section
  were removed from it (the Zen 4 numbers live here and in git history).
  **convex_pile BEATS float on both**: 0.75x on M3, 0.83x on Zen 4 — no
  longer claimed in the README; the mechanics (Erin's float SIMD stops at
  his solver, our narrow phase is vectorized — we out-vectorized, not
  out-multiplied) are in ERIN.md item 14. TWO RULES learned refreshing
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
- **Samples raw-fixed fix pass (2026-07-14) — CONVERGED WITH 724ba76**: two
  sessions independently swept the same bug class (b3Fixed raw values
  through float expressions, float literals truncating to N ULPs) in
  parallel; 724ba76 (the 100 km origin invariant + renderer conversion)
  landed on main first and its versions won the merge for all overlapping
  sample/gfx code — its renderer uses the explicit float-boundary
  convention (difference against the draw origin in fixed, then cross to
  float by value), NOT the raw-length-units accident the parallel session
  had documented; do not reintroduce raw flows there. What this branch
  ADDS on top of 724ba76: printf format attributes on DrawTextLine /
  DrawString3D / DrawScreenStringFormat (b3Log pattern, sample.h/draw.h/
  text.h) plus the varargs-UB fallout they exposed — uint64 material ids
  through %d (collision/compound/mesh/events) and a uint64 tick count
  through %ld (wrong on Windows); upstream shares the gap (ERIN.md "Small
  stuff"). LESSONS THAT SURVIVE THE MERGE: dt reached b3World_Step as a
  truncated float in both trees' history (the samples GUI never simulated
  until 2026-07-14 — both sessions fixed it, main's b3FixFromFloat wrap
  won); the -Wfloat-conversion/-Wimplicit-int-float-conversion audit
  (recipe also in the renderer bullet below) has blind spots — clang
  SUPPRESSES these warnings inside braced initializers, exact-integer
  float literals (2.0f) convert warning-free, and small raw b3Fixed
  values (< 2^24) convert to float exactly, also warning-free. ALL THREE
  are now CLOSED by tools/fixed-point/conversion_audit.py (2026-07-14):
  a type-aware clang-AST audit that flags EVERY implicit or explicit
  conversion between b3Fixed and float/double in both directions,
  regardless of value preservation or syntactic context (it filters out
  the sanctioned converter machinery by spelling location, so B3_FIX /
  b3FixToFloat uses don't fire). Run it after any merge of float-era
  code: `python3 tools/fixed-point/conversion_audit.py` (defaults to
  build-samples/compile_commands.json, covers src+shared+samples+gfx+
  host; exits 1 on findings, so it's CI-able). Verified 2026-07-14: the
  full tree audits CLEAN, and all three seeded blind-spot bug classes
  are detected. Raw b3Fixed*b3Fixed int multiplies remain
  ast_audit.py's job (the only real C++ hit was a
  GetAngle*B3_RAD_TO_DEG in sample_replay, fixed). VERIFIED post-merge on
  this branch: Release + Debug+VALIDATE samples builds green, sample
  logic at zero 'long long' conversion warnings and zero -Wformat
  warnings, full headless sample sweep exit 0. The Joints/Driving ~45 s TOI
  burn flagged here is FIXED — it was the parallel-joint sentinel impulse-cap
  wrap, not TOI cost; see the Joints/Driving CCD-burst section below (724ba76
  removed the TOI canaries; the cap fix removes the garbage velocities that
  caused the burn, and the root finder gained an outcome-identical stall
  early-out).
- **Int→b3Fixed classes added to conversion_audit.py (2026-07-14), two real
  finds fixed**: a parallel session's independent samples cast audit
  (this branch) converged with conversion_audit.py landing on main; the
  tools were consolidated instead of duplicated. conversion_audit.py now
  ALSO flags implicit int → b3Fixed conversions (nonzero literals and
  int-typed variables — `hertz = 5` widens warning-free but means 5 raw
  ulps), with the idiomatic integer-scaling contexts (`2 * fixed`,
  `x / 4`, `r /= N`) suppressed via parent context so the signal stays
  clean. The int classes are scoped to CONSUMER code
  (samples/shared/test/benchmark, INT_CLASS_DIRS in the script) — the
  engine legitimately works in raw ulps (narrow int32 Q16.16 storage
  widening exactly to int64 lanes, int counts in b3Fixed lane slots,
  mesh.c's deliberate 1-ULP minArea floor), and those six audited sites
  are all sanctioned. The float classes audited CLEAN across all 51 samples+shared TUs
  (the merged fix passes really did get them); the int class found the
  only two real bugs, both fixed on this branch: sample_character.cpp's
  positional material initializers put int literals 1/2 into
  rollingResistance (float upstream compiles 1.0f/2.0f, fixed truncated
  to 1-2 raw ulps — parity restored with B3_FIX; the literals look like
  userMaterialIds, upstream quirk noted in ERIN.md "Small stuff"), and
  shared/overflow_color.c called b3FixDiv( angle, OVERFLOW_PILE_PER_RING )
  with the plain INT macro as divisor (rule-8 mixup: both ring angles
  65536x too big, pile placement pseudo-random, the staggered-ring
  comment was a lie) — fixed to native `/` integer scaling. Verified:
  full suite green in RelWithDebInfo; Debug+VALIDATE+ASan/UBSan WorldTest
  green (TestOverflowColorPile still populates the overflow color with
  the corrected placement — it asserts overflowContacts > 0, no goldens);
  Release samples build green; all four Character samples +
  Robustness/OverflowColorPile run 240 headless frames exit 0.
- **Docs consistency pass (2026-07-13, after Glenn's README slim-down)**:
  docs/ was still the vanilla float manual. All ~114 float literals in
  example code are now B3_FIX-wrapped (plus float→b3Fixed declarations,
  sqrtf/cosf/b3MaxFloat → b3FixSqrt/b3Cos/b3FixMax, printf args through
  b3FixToDouble, FLT_MAX → B3_FIXED_MAX); large_worlds.md rewritten for
  fixed point (BOX3D_DOUBLE_PRECISION #errors, b3Pos permanently aliases
  b3Vec3, float-renderer draw-origin caveat kept); the FAQ's "Box3D does
  not support fixed-point math" answer and BOTH determinism sections
  (faq.md, simulation.md) rewritten — determinism by construction, vanilla
  stays credited as deterministic per the public-claims rule;
  raycast_capsule_parallel.md got a historical banner (it documents the
  float algorithm the capsule-raycast rewrite replaced); loose_ends.md
  default gravity corrected (-9.8 → -10, matching b3DefaultWorldDef);
  foundation.md gained a b3Fixed primer section; overview.md gained a
  fork note up top. hello.md and simulation.md explain the two B3_FIX
  traps inline (bare literal truncates 65536x; %f on a b3Fixed is UB).
- **Determinism goldens**: sleepStep=287, hash=0xB222C195, verified bit-identical
  across 1-5 workers. Hash updated 2026-07-15 for the 0.8 AU scene origin;
  sleepStep has carried over unchanged through FOUR origin moves
  (origin→100 km→120,000 km→1.2e11 m), each an exactly representable rigid
  translation. Prior hashes: 0x228A3865 (120,000 km), 0xE7D52285 (100 km) (see
  the scene-origin bullet below); sleepStep is UNCHANGED from the origin-scene
  value because an exactly representable origin shift is a bit-exact rigid
  translation of the whole trajectory — only the absolute transform bytes the
  hash covers moved. Any solver-affecting change invalidates these, see the
  test conventions section for how to regenerate.
- **Scene origin invariant (2026-07-14, from Glenn: "we should always work
  there"; moved to 120,000 km the same day to match vanilla-double's maximum
  world — a ±120,000 km cube with ~1 m float-broadphase padding, per Erin —
  then to 120,000,000 km = 1.2e11 m ≈ 0.8 AU for v1.2.0, a thousand times
  past that edge, same 1/65536 resolution)**: every sample, every benchmark
  scene, and the determinism test build their content around
  `GetSceneOrigin()` in shared/utils.h — a CONSTANT
  (`SCENE_ORIGIN_COORDINATE`, now `b3FixFromInt(120000000000LL)` — INTEGER
  construction because 1.2e11 is NOT exactly representable in float32; exact
  in Q48.16, so origin moves stay bit-exact rigid translations) with
  deliberately no setter, so nothing can opt back to (0,0,0). v1.2.0
  verification at 0.8 AU (2026-07-15): full suite + goldens (sleepStep 287,
  hash 0xB222C195) Release AND sanitized Debug; sanitized bench smoke with
  counts IDENTICAL to 120,000 km; 154-sample sweep zero failures; A/B vs
  float baseline: Card House 4.25 and Far Pyramid 3.73 only (both 0.00
  pixel-identical to their 120,000 km selves), rest at noise. NOTE the numeric hazard class at
  this magnitude: a 64-bit b3FixMul SQUARE of an absolute coordinate wraps
  past |v| ~ 1e7 m (raw 7.9e12 squared >> int64) — the engine's length/dot
  paths are exact raw-128 and immune, differences are small, but never square
  an absolute world coordinate in 64-bit fixed. 120,000 km verification
  (2026-07-14 late): full suite + goldens (sleepStep 287 unchanged, hash
  0x228A3865) in Release AND Debug+VALIDATE+ASan/UBSan; sanitized 10-step
  benchmark smoke all scenes; per-scene benchmark counts diffed IDENTICAL vs
  100 km; full 154-sample headless sweep zero failures; screenshot A/B vs the
  float-at-origin baseline: 153/153 pairs, same two known divergences (Card
  House 4.3/255 — identical score to 100 km — and Far Pyramid 3.7 by design),
  nothing new. Samples use the wrappers in
  samples/sample.h: `SampleOrigin()`, `SamplePos( local )`, `SampleLocal(
  world )`; C scene builders (shared/benchmarks.c, determinism.c,
  overflow_color.c) use `b3OffsetPos( GetSceneOrigin(), local )`. Author scene
  layout in LOCAL coordinates and offset exactly once at the world boundary;
  readbacks compared to authored constants go through SampleLocal/b3SubPos.
  Benchmark workloads are bit-identical to the historical origin layouts
  (verified 2026-07-14 at 100 km vs pristine-HEAD for five scenes, and again
  at 120,000 km vs 100 km for ALL scenes via the sanitized smoke's count
  lines) — the published perf tables remain valid;
  future refreshes still re-run both sides per the existing rules (the float
  reference stays at ITS origin, which is float's best case, so the comparison
  is honest). sample_world.cpp's Far* samples keep their own configurable
  offsets (0..1e7 m) — they are the deliberate exception that demonstrates the
  full range, including (0,0,0); note their 1e7 m ceiling is the documented
  safe bound for naive 64-bit squares, NOT a position limit — the shared
  origin at 1.2e8 m is fine because engine paths never square absolutes. DrawGroundGrid now takes the origin as a
  parameter. NOTE: unit tests other than DeterminismTest still author their
  own scenes near (0,0,0) on purpose (near-origin coverage, and TestMeshDrop's
  sleep equilibrium is a knife edge — do not move it casually).
- **The samples RENDERER conversion (2026-07-14, found because the 100 km
  origin turned silent wrongness into crashes)**: the samples gfx/host layer
  (samples/gfx, samples/host, and much of the sample scene code) had NEVER
  actually been converted to fixed point — it was authored against float
  b3Vec3/b3Pos and compiled silently as integer math on raw Q48.16 bits
  (~1,850 compiler-flagged lossy conversions). Consequences that existed at
  HEAD before this work: Camera::DrawOrigin round-tripped the eye through
  float raw bits (65536^2 inflation; saturated INT64 beyond a ~32 km eye —
  b3IsValidAABB assert in Debug, dead culling in Release); camera angle
  constants derived from the FIXED B3_PI (DEG_TO_RAD was 571.9); seven
  samples SEGFAULTED at pristine HEAD (float args into b3Fixed hull-creator
  params -> 30 um degenerate clouds -> quickhull's hang-proofed NULL return,
  never null-checked); ground boxes microscopic (b3MakeBoxHull float args);
  IBL SH coefficients double-corrupted (diffuse ~0); GTAO UBO layout broken
  (sokol-shdc @ctype mapped vec2 to b3Vec2, which became two int64s); the
  rigid-body character controller entirely non-functional (raw
  closestFraction). ALL FIXED via three passes: (1) render-boundary
  convention — world positions are differenced against the draw origin IN
  FIXED (b3SubPos, exact at any distance) and cross to float BY VALUE
  (b3FixToFloat/MakeVec4FromFixed in samples/gfx/utility.h;
  MakeMat4FromTransform/MakeViewAndInverse are the choke points); DrawOrigin
  returns m_worldEye directly; FrameInput drawOrigin/gridWrap/cameraPosition
  are float Vec4 meters; shader @ctype vec2/vec3 now map to float Vec2/Vec3
  (utility.h) — gtao_main_pass.glsl.h regenerated-by-hand to match, KEEP IN
  SYNC if shaders are regenerated; (2) gfx/host file sweep; (3) sample-file
  sweep driven by the compiler: `-Wfloat-conversion
  -Wimplicit-int-float-conversion` via -fsyntax-only over
  compile_commands.json enumerates every lossy crossing (1,852 before, 112
  benign int-counter leftovers after — b3Fixed is a bare int64 typedef, so
  BOTH directions of raw mixing are compiler-visible; rerun this audit after
  merging any float-era sample code). Verified: Release + Debug+VALIDATE
  builds green, all previously-crashing/asserting samples run clean, and the
  instance transforms reaching the GPU probe as small eye-relative meters
  (lldb break on AppendMesh) instead of 65536^2-scaled garbage. Engine code
  (src/, include/) needed ZERO changes — the uniform-precision story was
  always true; only the sample app's float boundary was lying about it.
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
- **Public-claims rule (from Glenn; reframed 2026-07-13)**: the "make Erin
  mad" joke is RETIRED from all public framing (README, ERIN.md, commit
  messages) — do not reintroduce it. The repo's framing is now the honest
  experiment: it answers "What would Box3D look like if it was fixed
  point?" and "Exactly how much slower would it be?" (answer: ~2x geomean).
  The ONLY benefit is a truly huge world with uniform precision. Vanilla
  Box3D is ALREADY deterministic across platforms — Erin achieves it in
  float with FP discipline — so never pitch determinism as a Fixed3D
  feature; fixed point only changes HOW determinism is achieved (by
  construction, no FP flags to police). Vanilla also already handles a
  20,000 km cubed world with just +/-1M of broadphase padding, with large
  position support only ~3% slower than float positions (numbers from
  Glenn), and the README now says plainly that most users should keep
  using vanilla Box3D. Glenn's 2026-07-13 README slim-down removed the
  convex_pile win claim entirely; his standing framing is "I expect this
  to worsen to around 2.5x as any worthwhile optimizations found during
  this exercise are backported to the real Box3D" (the vectorized narrow phase
  is a technique, not a fixed-point advantage — ERIN.md documents it for
  backporting). If the win is ever claimed publicly again, attach the
  likely-temporary caveat.
- **Per-sample visual A/B vs float (2026-07-14, first full visual
  verification)**: the --capture harness ran every sample to its 120-frame end
  state on BOTH trees — this one (100 km origin) and the pristine float
  baseline e9f6f1d (same renderer generation, same authored cameras) — and
  paired the PNGs by category/name. Result: 153/153 matched samples, zero
  captures failed, zero empty/garbage frames, worst end-state divergence
  4.3/255 mean luminance. Two real divergences, both understood: (1)
  Stacking/Card House STANDS in float but COLLAPSES in fixed — INVESTIGATED
  TO CLOSURE 2026-07-15, KEPT INTENTIONALLY (Glenn's decision; annotated in
  the sample and docs/samples.md so nobody is confused by the divergence).
  Mechanism, established by controlled experiments (drivers + dumps + all
  probe rounds preserved in the private rowan repo,
  investigations/cardhouse/): the cards (2 mm) are thinner than the
  solver's linear-slop-class tolerances (~5 mm; Erin's own in-sample todo
  says the scene strains minimum thickness), which puts the outcome in a
  CHAOTIC regime. Three attractive theories were refuted by experiment —
  authored-thickness quantization (exactly-representable 2^-10 thickness
  collapses identically), the 15 um resolution lattice (a physically
  identical house with a 100x finer lattice and correctly scaled
  gravity/tolerances collapses identically — NOTE: gravity and sleep
  thresholds do NOT scale with lengthUnitsPerMeter, a confound that
  invalidated the first naive scale ladder), and dt (float stepping at
  fixed's exact 0.01666259765625 stands at 1.638 vs 1.640 m). The proof of
  sensitive dependence: PURE FLOAT with only fixed's initial rotations
  injected (~3e-5 quat differences from the Q32.32 trig constructors)
  loses its top story (max height 1.640 -> 1.452). Fixed point stands and
  SLEEPS when the scene has sane tolerance ratios (dynamically-similar
  x100 house, gravity x100: zero displacement). Conclusion: knife-edge
  scene in ANY number system; the number system merely picks the ending.
  Kept as the corpus' sensitivity canary — any future solver change that
  shifts sub-tolerance behavior shows up here first. Content guidance now
  in docs/samples.md: keep feature sizes comfortably above solver
  tolerances for marginally stable assemblies; (2)
  the World/Far samples diverge in fixed point's favor — at 10,000 km float's
  pyramid shatters on its ~1 m grid while the fixed one is pristine (the
  repo's thesis, now in pixels). Harness: COMMITTED in tools/capture/
  (capture_sweep.sh, compare_shots.py — needs Pillow, README has the full
  recipe), alongside float-baseline-capture-hooks.patch, the float-side port
  of the capture hooks (applies to e9f6f1d; rescued 2026-07-14 from the
  session scratchpad where it was the only copy). The app-side
  --capture/--list-samples/--headless flags are committed. Float worktree
  recipe: worktree at e9f6f1d, apply the patch, copy
  samples/host/capture.{h,m} verbatim (byte-identical on both trees), build
  with BOX3D_SAMPLES=ON. The reviewed contact sheet + raw shot PNGs (~760 MB)
  were NOT committed — regenerable from the harness in ~30 min.
- **Branding rule (2026-07-14, from Glenn)**: the public/user-facing name is
  "Fixed3D" — window title, docs prose, doxygen project, CMake descriptions /
  option help / STATUS messages, test+log banners, the fixed.h #error. The
  ENTIRE API stays Box3D-branded BY DESIGN for migration compatibility: b3*
  symbols, B3_*/BOX3D_* macros and CMake option names, the box3d include
  directory and CMake project/target/library names are all untouched — do not
  "finish" the rename into identifiers. "Box3D" remains correct (and required,
  per the public-claims rule) wherever it names Erin's upstream engine:
  "vanilla Box3D", provenance, comparisons, links. Known open content
  questions deliberately NOT decided during the rename: docs/faq.md and
  docs/overview.md still route feedback/bugs to erincatto/box3d channels.
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
  at 0.83x** (53,092 ms vs 63,943 — not claimed in the README since the
  2026-07-13 slim-down; the honest mechanics are in ERIN.md item 14:
  Erin's float SIMD stops at the solver while our narrow phase is
  vectorized too). Per-scene speedups over scalar fixed:
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

## Joints/Driving CCD-burst investigation (2026-07-14, landed)

- The reported "~45 s wall time in the first ~15 frames, all in
  b3SolveContinuous → b3TimeOfImpact, root finder hits maxRootIterations" was
  a SYMPTOM. Root cause: the parallel joint's sentinel impulse-cap wrap (see
  rule 4 above) exploded velocities to ~1e14 at the first wheel–height-field
  contact; CCD then ground on garbage sweeps. Fixed-point TOI quantization was
  NOT the driver: with sane velocities, wheels pre-spun to 10,000 rad/s
  (allowFastRotation) drop onto the b3CreateWave field with ZERO measurable
  CCD cost and the distance.c canaries stay silent in Debug+VALIDATE (a
  sphere's separation function is rotation-invariant, and capped-rotation
  hulls stay within Erin's 0.25π/step design bound).
- b3TimeOfImpact still gained the cheap guard, as hardening: when the root
  bracket collapses to one time-ulp with both endpoints outside tolerance, no
  iterate can make progress (bisection midpoint rounds onto an endpoint,
  false-position numerator rounds to 0), so it bails out of the root find AND
  the push-back loop. Outcome-identical by construction (t2 unchanged either
  way, every further push-back iteration would repeat the identical stalled
  root find); saves ~4×50 separation evaluations per outer iteration on
  garbage inputs. The B3_VALIDATE(false) at maxRootIterations is now
  effectively unreachable (any bracket ≤ 1.0 collapses within ~32 bisections).
- Samples: this session independently found and fixed the same GUI dt
  truncation (float timeStep into b3World_Step → dt=0, which is what hid the
  explosion) and the Driving sample's ~30 silent float→b3Fixed truncations
  (value-preserving literals like `density = 2.0f` → 2 raw warn nowhere;
  float MEMBERS assigned to b3Fixed fields never warn) — the parallel
  raw-fixed fix pass (7d7f7e7/724ba76, see the samples bullet up top) swept
  the same class tree-wide and its versions won the merge; both sessions
  converged on identical fixes for Driving. The truncation blind spots are
  recorded in that bullet.
- Not ERIN.md material: float is immune to the cap wrap (inf compare), the
  TOI guard is fixed-point-specific, and the sample bugs are port artifacts.

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
   THAT RULE OF THUMB WAS INCOMPLETE (found 2026-07-14 via the Joints/Driving
   sample): `maxImpulse = h * B3_FIXED_MAX` is fine (~5.8e11), but the joint
   impulse caps then compute `impulse² > b3FixMul(maxImpulse, maxImpulse)`,
   which wraps — at h = 1/240 to EXACTLY 0 — so the "clamp" fired on the first
   nonzero impulse and SCALED THE IMPULSE UP to 5.8e11 (float: (h·FLT_MAX)² =
   inf, compare never fires). One misfire injected ~1e10 rad/s into the
   Driving chassis, the wheel joints spread it to ~1e14 across two frames, and
   the resulting garbage sweeps stalled CCD for seconds per frame (the
   reported "45 s in 15 frames" burst — the TOI root finder was the victim,
   not the cause). Fix: b3ImpulseOverCap2/3 in joint.h compare at 128 bits,
   mirroring each site's exact rounding (two FixMul roundings for the vec2
   site, one fused rounding for vec3) so in-range values compare
   bit-identically; parallel_joint.c + the four motor_joint.c caps use them.
   Regression: TestParallelJointSentinelTorqueCap (verified red pre-fix).
   Audit rule: a sentinel-derived value that later gets SQUARED (or multiplied
   by anything >= 1) wraps even when the first product was safe. The contact
   solver's friction/rolling caps square PHYSICAL impulses (μ·normal) the same
   way — THAT FOLLOW-UP IS FIXED (2026-07-14, same day): the extended range
   audit measured the true cap operand (the per-iteration manifold normal sum,
   NOT the 5.8e6 per-point event accumulator, which is never squared) at
   1.65e6 in convex_pile → cap 0.99e6 at friction 0.6, only 12x under the
   2^23.5 ~ 1.19e7 wrap, with friction/density/body-scale all linear
   multipliers; and for caps in [1.19e7, 1.68e7) the wrapped square is
   NEGATIVE, so the clamp fired on every compare and rescaled the friction
   impulse UP (energy injection, reachable by legal heavy/high-friction
   content). All six squared-cap sites (scalar mesh, wide convex, wide mesh ×
   friction/rolling) now gate every compare operand on B3_IMPULSE_CAP_GATE
   (2^38 raw = 4.19e6 units; below it no 64-bit intermediate can wrap and the
   historical code runs verbatim = bit-identical for all existing content);
   past the gate a cold path in contact_solver.c compares at 128 bits
   mirroring each site's exact rounding (unfused two-term friction form,
   fused three-term rolling form, per-site epsilon placement), with the clamp
   divisor from the 64-bit squared length when it did not wrap and from the
   raw 128-bit sum when it did. Scalar and wide share the cold helpers
   lane-wise, so they cannot diverge; the gate also makes the hot path's
   int64-addition UB unreachable. Twist caps are symmetric clamps (no
   squaring) and stay unguarded. Regression: TestFrictionCapWrap in
   test_world.c (hull ground = wide convex, mesh ground = mesh path; a
   65536-mass cube — the largest whose inverse mass is representable — onto
   friction-3.0 ground lands the cap mid-wrap-window; verified red at
   e63f161). The range audit now also covers the scalar mesh solver and
   tracks the real clamp operands (iter normal sum, friction/rolling/twist
   cap channels). VERIFIED: all 22 suites + goldens 287/0xE7D52285 in
   Release and Debug+VALIDATE+ASan/UBSan on arm64 scalar, and on Zen 4
   AVX-512 with clang-18 Release, gcc-13 Release LTO AND no-LTO (the divq /
   dedup lesson), and clang-18 Debug+VALIDATE+ASan/UBSan. COST (interleaved
   min-of-3 A/B, both sides LTO, gates folded to one bias-and-OR compare
   per site): convex_pile +0.5-0.7% on M3 scalar, +1.05% on Zen 4 AVX-512
   (3/3 pairs — the friction-site gate is the only addition convex_pile
   executes; its rolling resistance is zero); rain (scalar mesh) and
   trees100 (wide mesh) are a wash. Accepted as the price of the
   correctness fix — there is no zero-cost formulation that keeps the
   clamp exact in both misfire directions, and the lazy alternative
   (verify only when the 64-bit compare fires) still misses wrapped
   never-fire compares and costs MORE in cone-saturated scenes.
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
- Determinism goldens (`test/test_determinism.c`): `RAGDOLL_SLEEP_STEP 287`,
  `RAGDOLL_HASH 0xB222C195` (macros renamed from EXPECTED_* in the dfa5e6a
  port, following upstream; hash updated 2026-07-15 for the 0.8 AU scene
  origin, previously 0x228A3865 at 120,000 km and 0xE7D52285 at 100 km; the
  sleep step carried over unchanged through all four origin moves because
  the shift is a bit-exact rigid translation; previously 0x6FA8A4C5 for the
  e961bfb friction-center port; verified bit-identical across 1-5 workers).
  Any solver-affecting change invalidates these: rerun, take the printed
  values, confirm they're identical for all worker counts
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
  SEPARATE gap found during this fix (chip spawned), now FIXED: three shape
  mutators had no B3_REC coverage, so recorded sessions that called them
  replayed divergent — b3Shape_SetMeshMaterial (op 0x5D, ed19fbe) and
  b3Shape_SetHull / b3Shape_SetMesh (ops 0x5E/0x5F, b49b899;
  B3_REC_VERSION_MINOR now 5). The geometry pair interns via
  b3RecInternHull/Mesh like the create ops, and b3Shape_SetHull records
  AFTER its shared-hull dedup short-circuit so the stream carries only
  mutations that proceed. Upstream e961bfb has the identical gap
  (ERIN.md entry 8). Regression subtests: MeshMaterialReplay and
  HullMeshSwapReplay in test_recording.c, both verified red without
  their B3_REC calls.
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
