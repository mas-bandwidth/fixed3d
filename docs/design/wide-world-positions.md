# Design: 128-bit world positions with high-precision local space

Status: **IMPLEMENTED** as the opt-in `BOX3D_WIDE_POSITIONS` build (Rowan,
2026-07-15). Phases 2–4 landed in #2 and the follow-up PR; OFF is bit-identical,
ON passes the full suite in Release and Debug+VALIDATE+ASan/UBSan. This document
is the design of record — a few points below read differently in hindsight, and
the notable one is called out where it mattered. Built on the four-lens
precision-architecture survey (preserved in the private rowan repo,
`investigations/precision-architecture/`).

> **Hindsight note (2026-07-15).** The build confirmed the central thesis
> (the interior needs zero changes) and the Q112.16 boundary decision. Two
> claims below were corrected by the implementation: (1) there are **seven**
> `b3Pos_zero` demote sites, not three; and (2) they do **not** overflow at the
> 0.8 AU scene origin — that coordinate (~7.9e15 raw) fits int64 comfortably, so
> the demote is exact within the ±1.4e14 collision-active radius, which
> `LargeWorldTest` confirms empirically. The demote sites only fail *beyond*
> ±1.4e14, which Option A does not support for collision by design.

## The question

Fixed3D today uses one number format everywhere: Q48.16 in `int64_t`
(`b3Fixed`), resolution 1/65536 ≈ 1.5e-5, range ±1.4e14. That uniform
precision is the whole thesis — a body at 0.8 AU has the same 15-micron
resolution as one at the origin. But it leaves two things on the table:

1. **Range.** ±1.4e14 units is enormous, but it is still a wall. A
   solar-system-scale world in metres (Neptune ≈ 4.5e12 m; a light-year ≈
   9.5e15 m) runs out inside Q48.16.
2. **Local precision.** 15 microns is coarse for the *small* end. It forces
   the resolution-floor workarounds this codebase is full of: the 4-ULP
   inertia floor (rule 6), the raw-128 normalize rewrite (rule 3), the
   determinant-underflow 128-bit paths (rule 1), the `d > 0` squared-tolerance
   fix (rule 7). All of these exist because 1.5e-5 is not fine enough near
   zero.

Upstream Box3D solved the mirror-image problem in float with
`BOX3D_DOUBLE_PRECISION`: widen *only* world positions (to double), keep the
hot interior in float, convert at a thin boundary. The survey confirms that
architecture ports to fixed point almost 1:1 — and buys us **both** things at
once, because a wider position type plus a finer local type is the same
boundary with two different formats on its two sides.

## The proposal in one sentence

Make `b3Pos` a distinct **128-bit** fixed-point world coordinate; keep the
entire simulation interior in `int64` fixed point exactly as it is today;
convert between them at the ~13-function boundary that already exists.

## What the survey settles (so we don't relitigate it)

The single most important finding: **the boundary already exists, as a
~13-function vocabulary in `math_functions.h`**, and outside that file the
`src/` tree touches absolute world coordinates *only* through it —
`b3SubPos`, `b3OffsetPos`, `b3InvMulWorldTransforms`, `b3ToRelativeTransform`,
`b3TransformWorldPoint`, and friends. Upstream introduced this exact vocabulary
for double precision; fixed3d inherited it and currently has each function
collapse to a trivial `int64` op because `b3Pos` aliases `b3Vec3`. Widening
`b3Pos` changes only these inlines and the three body fields behind them.

The equally important negative finding: **the local format cannot be pushed
to Q16.48** (the tempting "32 extra fraction bits" idea). The range audit is
decisive — impulses reach 5.8M units, effective masses 32k–65k, inertia
tensors 3e9, and `INT64_MAX`-as-infinity sentinels are load-bearing
throughout. A Q16.48 local format (±32768 range) breaks masses, impulses,
inertia, the sentinel convention, the SIMD bit-identity proofs, the int32
narrow storage, and the NEON escape hatch — each independently fatal. So the
interior **stays Q48.16**. This design widens the *world* end, not the *local*
end. (Finer local precision, if we ever want it, is a separate and much harder
project — noted in "Rejected / deferred" below.)

## The two-format architecture

```
        WORLD (absolute)                    LOCAL (relative)
        b3Pos = int128 fixed                b3Vec3 = int64 Q48.16
        ─────────────────────               ────────────────────────
        b3BodySim.transform.p               everything the solver sees
        b3BodySim.center, center0           b3BodyState (deltas, velocities)
        dynamic-tree AABBs (option A/B)     all narrow-phase math
        public API positions                all joint math
        query origins, event points         GJK / SAT / TOI / distance
                                            mass properties, impulses, masses
                    │                                    ▲
                    └──────── the boundary ──────────────┘
                     b3SubPos / b3OffsetPos / b3InvMul...
                     int128 subtract → int64 (exact when
                     the operands are within local range)
```

**Format for `b3Pos`: Q112.16 in `int128`** — the same **16 fraction bits**
as `b3Fixed`, with the extra 64 bits going entirely to *integer* range
(112 integer bits, ±2.6e33 units — far past a light-year in metres). This is
the load-bearing decision, and matching the local fraction count is *why the
whole design works*, not an incidental choice:

- **The boundary subtract stays exact and shift-free.** When `b3Pos` and
  `b3Fixed` share 16 fraction bits, an `int128 − int128` position difference
  that fits Q48.16's range **is** the exact Q48.16 value in its low 64 bits —
  no rescale, no rounding, just a range check and a truncation to `int64`.
  Likewise the once-per-step delta-fold is `int128 += (int128)int64`, exact,
  because the fraction points already align.
- **Extra *fraction* bits in the world type would be worse than useless.** The
  interior is Q48.16 and (per the survey) must stay there, so any fraction
  bits beyond 16 are *thrown away at the boundary* the instant a position is
  differenced into local space — while forcing every boundary op to rescale
  (`>>N`) instead of truncate. Finer world resolution buys nothing the
  interior can consume and costs the exactness that makes this elegant.
- **Range, not resolution, is the only thing widening the type can add.** So
  put all 64 new bits into range. Uniform 15-micron resolution — the repo's
  thesis — is *preserved exactly*: a body at any representable distance has
  the identical resolution it has today.

The translation is 128-bit; **rotation stays a Q48.16 quaternion** — rotation
is frame-local and never needs range (the same split Jolt uses for `DMat44`,
and the same split upstream uses: wide translation, narrow rotation).

**The boundary operation is exact.** In float, `b3SubPos` is "subtract then
round to float" — lossy by nature, which is why upstream needs the entire
`b3RoundDownFloat`/`b3RoundUpFloat` directed-rounding apparatus. In fixed
point, `b3SubPos` is an **integer subtract**: `int128 − int128`, then narrow
to `int64`. It is *exact* whenever the result fits Q48.16's ±1.4e14 range —
which it always does for a contact pair, a joint, or a query, because those
are AABB-adjacent or reach-bounded. The only failure mode is a range
violation, so the helper needs a **range assert** (debug) / **saturate**
(release), not rounding care. This deletes upstream's entire directed-rounding
subsystem.

## Why the interior needs zero changes

The survey's subsystem census is the proof, and it rests on a design fixed3d
already inherited from Erin: **the solver runs in delta space.** `b3BodyState`
— the only per-body view the solver, the wide SIMD path, and the gather/scatter
transposes ever touch — carries `deltaPosition`/`deltaRotation`, not absolute
pose. Integration accumulates `h·v` into the delta; the absolute `int128`
position is touched exactly **once per body per step**, at finalize
(`solver.c` ~710), where the delta folds into `center` (an `int128 + int64`
widening add). Sleep logic measures delta magnitude, never absolute position.

Consequently, everything that makes fixed3d fast is out of scope:
- the AVX-512 wide solver and its bit-identity multiply decomposition,
- the NEON narrow-phase int32 path,
- the `b3Vec3WN` int32 Q16.16 narrow constraint storage,
- the gather/scatter transposes pinned by `_Static_assert`,
- GJK / SAT / manifolds / TOI / distance,
- mass properties, joint solving.

None of them see a `b3Pos`. They see `int64` locals, exactly as today. This is
the entire reason the design is viable rather than a rewrite.

## The per-step / per-pair conversion budget (preserve these numbers)

- **1 int128 add per body per step** — the finalize delta-fold.
- **1 int128 subtract per contact manifold per update** — `anchorB = anchorA
  + (posA − posB)`, then COM re-centering keeps anchor magnitudes at body
  scale.
- **1 int128 subtract per joint per step** — `deltaCenter` at prepare; all
  sub-step solving is `int64` thereafter.
- **1 int128 subtract per candidate per CCD sweep** — re-based on the fast
  body's `center0`.
- **1 int128 subtract per shape per query** — the origin-relative
  re-differencing.

All O(bodies/pairs/joints/queries) per step, never O(iterations). The hot
inner loops stay pure `int64`.

## The migration surface (what actually changes)

1. **Three `b3BodySim` fields** widen: `transform.p`, `center`, `center0` —
   three coordinate triples going from `int64` to `int128`, so +8 bytes per
   coordinate × 9 = **+72 bytes/body** (before alignment). `b3BodyState`
   untouched.
2. **~13 boundary inlines** in `math_functions.h` gain real int128↔int64
   bodies. `b3LerpPosition` is the one that currently does `b3FixMul` on
   absolute coordinates — reformulate as `a + t·b3SubPos(b,a)` so the
   multiply happens on the in-range difference (mind the round-half-up
   asymmetry lessons from the scalar-fusion work).
3. **The three "demote at `b3Pos_zero`" sites** (`broad_phase.c` compound
   query, `mesh_contact.c` triangle query, `sensor.c` overlap) must re-base
   on a nearby origin instead of world zero. Today they work because
   `b3Pos==b3Vec3` makes absolute representation exact; with a distinct
   `int128` `b3Pos` at the 0.8 AU scene origin they would overflow the demote.
   **This is the highest-risk change** — it has no float analog to crib from,
   and it is a silent-overflow class, not a compile error. Each re-bases on
   the query AABB centre / body A position / sensor shape position
   respectively.
4. **Dynamic-tree AABBs** — decision, see Open question 2.
5. **Public API + events** — every `b3Pos`/`b3WorldTransform` in `types.h`
   and `box3d.h` (the survey enumerates the complete list: `b3BodyDef.position`,
   `b3ContactHitEvent.point`, `b3RayResult.point`, the debug-draw callbacks,
   the getters/setters). This re-opens the migration list that fixed3d closed
   when it aliased `b3Pos == b3Vec3`.
6. **Serialization / recording / determinism hash** — widen the wire to two
   64-bit words per axis (recording already widened once, for `b3Fixed`), add
   the snapshot precision flag + load-time refusal, and mix the **full** 128
   bits into the determinism hash (else the gate silently validates only low
   bits). New per-mode goldens.
7. **ABI guard** — the `#define b3CreateWorld b3CreateWorldWidePositions`
   link-error trick, PUBLIC compile definition, `b3IsWidePrecision()` runtime
   query, widened `B3_HUGE`. All transfer literally from upstream's pattern.

## Build model: opt-in, diffable, zero-cost when off

Mirror upstream exactly: a CMake option (`BOX3D_WIDE_POSITIONS`) that, when
**off**, makes `b3Pos` a `typedef` for `b3Vec3` and every boundary helper
collapse to today's trivial `int64` ops — i.e. the current engine, bit-for-bit,
goldens unchanged. When **on**, `b3Pos` is the distinct `int128` struct and the
compiler enumerates every migration site for us (making it distinct rather than
a wider typedef is what turns silent puns into compile errors — we *want* that).
This keeps the default build and its performance table untouched, and makes the
whole feature reviewable as a diff.

## Acceptance suite (writes itself from upstream's, but stronger)

Upstream's `test_large_world.c` patterns, ported — but where upstream asserts
relative trajectories match "to 1e-3" (an honest float artifact), the fixed
version asserts **bit-identical**, because exact integer subtraction makes the
tolerance unnecessary (we already proved the 100 km → 0.8 AU origin shifts are
bit-exact rigid translations, sleepStep unchanged):
- same-sleep-step stack at a far base vs at the origin,
- no-tunnel fast bullet caught by a thin wall far from origin,
- fat-AABB containment far from origin,
- query parity (overlap/cast/mover) at a far base,
- hull manifold at a far base equals the origin manifold **exactly**.

Plus a new class upstream can't test: **beyond ±1.4e14** (past old Q48.16
range) to prove the widening actually extends reach, and an explicit
**range-assert** test on `b3SubPos` for a pair separated by more than local
range (the failure the design deliberately makes loud).

## Open questions for review (Glenn)

1. **`b3Pos` fraction bits — decided in the body of this doc: Q112.16**
   (16 frac, matching `b3Fixed`). This is not really open; I'm flagging it
   only because my first draft got it backwards (recommended more fraction
   bits) and the correction is the crux of the whole design, so it's worth
   your explicit sign-off. Sharing the local fraction count makes the boundary
   a truncating range-check rather than an arithmetic rescale, and makes the
   position difference *bit-identical* to the local value — extra world
   fraction bits would be discarded at the boundary anyway while costing that
   exactness. All 64 new bits go to range.
2. **Dynamic-tree AABBs: stay Q48.16-world, or widen to int128?** Option A
   (stay `int64` world): tree traversal is compare/min/max only, works at any
   magnitude, and `b3AABB_Center`'s `lower+upper` sum only overflows past
   ~7e13 — so a Q48.16 tree covers the *current* range for free and only shape
   AABB *generation* changes. But it caps the *broadphase* world at ±1.4e14
   even though positions now go further, so the extra range would be
   body-position-only, not collision-active. Option B (widen tree to int128):
   full range everywhere, but doubles the 48-byte node AABB field and the SAH
   rebuild arrays, and touches the one hot subsystem this design otherwise
   spares. **Recommendation: start with A** (positions widen, broadphase stays
   Q48.16, document the "collision-active radius" as ±1.4e14 which is already
   past any real content), measure, and only do B if a real world needs
   collision past 1.4e14 units. This keeps phase 1 entirely out of the tree.
3. **Is the extra *range* even the goal, or is it insurance?** The honest
   framing: fixed3d at 0.8 AU already exceeds any shipped need. Q48.16's ±1.4e14
   is not currently a constraint anyone has hit. So this feature is (a) future
   insurance for solar-system-plus worlds, and (b) — more interestingly — the
   *architecture* that a future finer-local-precision project would require.
   Worth being clear-eyed that it may be built for elegance and headroom rather
   than a present need. Your call on priority.

## Rejected / deferred (with reasons, so nobody re-runs them)

- **Q16.48 local format** (finer interior precision). Rejected by measurement:
  breaks masses (32k–65k), impulses (5.8M), inertia (3e9), the
  INT64_MAX-infinity sentinel convention (becomes 32768 units — smaller than
  real impulses), the AVX-512 bit-identity proofs, the int32 narrow storage,
  and the NEON int32 gate. The four "resolution-floor" bug classes it would
  fix (rules 1/3/6/7) are already fixed by the raw-128 patterns. Not worth
  reopening without a fundamentally different impulse representation.
- **Mixed Q16.48-local + Q48.16-dynamic-quantities.** The hazard matrix shows
  this makes the solver inner loop mixed-format at every line, with silent
  format-alchemy through `b3FixMul` (both formats are the same bare `int64`
  typedef, so even `conversion_audit.py` is blind). Strictly worse than the
  raw-fixed-through-float class we just finished sweeping. Only viable with
  distinct wrapper types and per-site shift annotations — a large, risky,
  low-payoff project.
- **Widening the local/interior format at all.** The interior is where all the
  performance work lives (2.07× geomean, the convex_pile-beats-float result);
  it stays `int64` Q48.16 untouched. This design deliberately confines all
  change to the world boundary.

## Phasing (if approved)

- **Phase 2:** the `BOX3D_WIDE_POSITIONS`-off scaffold — introduce distinct
  `b3Pos`/`b3WorldTransform` types that `typedef`-collapse to today's, land the
  boundary-vocabulary indirection with zero behavior change, confirm goldens
  bit-identical. (All the migration-site churn, none of the risk.)
- **Phase 3:** the int128 bodies behind the option — three `b3BodySim` fields,
  the boundary inlines, the three re-basing fixes, ABI guard. Tree stays
  Q48.16 (Option A).
- **Phase 4:** serialization/recording/hash widening + per-mode goldens +
  the acceptance suite.
- **Phase 5 (optional):** int128 tree (Option B) if a world needs
  collision-active range past 1.4e14. **LANDED 2026-07-16 as
  `LUDICROUS_MODE` — see the addendum below.**

Each phase is independently landable and the off-path stays bit-identical
throughout.

## Addendum: Phase 5 landed as LUDICROUS_MODE (Rowan, 2026-07-16)

Phase 5 — the int128 broadphase tree, "Option B" above — is built, verified,
and measured. Glenn named it: the CMake option and preprocessor symbol are
literally `LUDICROUS_MODE` (deliberately not `BOX3D_`-prefixed; the name is
the documentation). It widens `b3AABB` bounds to `b3Pos`, so the broadphase
tree is collision-active across the full wide-position range instead of
capped at ±1.4e14 units. Requires `BOX3D_WIDE_POSITIONS`. Off by default,
off bit-identical, and nobody should ever turn it on — it exists to answer
one question: exactly how much slower is a 128-bit broadphase?

**The answer: +1.6% geomean.** M3 Ultra, 4 workers, min-of-3, Release+LTO,
per-scene interleaved against the same-session narrow build:

| scene         | narrow (ms) | ludicrous (ms) | ratio |
|---------------|-------------|----------------|-------|
| large_pyramid |     1712.3  |        1726.5  | 1.008 |
| many_pyramids |     1757.0  |        1717.3  | 0.977 |
| rain          |     1291.0  |        1326.3  | 1.027 |
| trees100      |      154.2  |        167.6   | 1.087 |
| large_world   |       24.3  |         24.9   | 1.023 |
| joint_grid    |      797.5  |        803.7   | 1.008 |
| convex_pile   |    22521.0  |      22531.1   | 1.000 |
| washer        |    14546.4  |      14559.3   | 1.001 |

Geomean 1.016 vs narrow (and 1.017 vs the wide-positions build, which
re-measured at 1.000 of narrow). Only trees100 — the most tree-query-bound
scene — pays a real cost (~+7–9%); solver-bound scenes are flat. int128
compare/min/max is two 64-bit ops with carry on arm64, and the broadphase
is a thin slice of the frame.

**Proof it does what it says**: a box dropped onto a static box at
1e15 m from the origin (~0.1 light-year; 7× past the int64 b3Fixed range,
where the narrow broadphase cannot represent the AABB at all) settles at
local Y = 1.4999 m, sleeps at step 119, with a settle drift of −156 ulp —
**bit-identical** to the same scene built at the origin.

Implementation notes (the two real bug classes, both caught by the
verification gates):

1. **SIMD packed loads on bounds**: six `b3LoadV( &aabb.lowerBound.x )`
   sites (mesh.c query/rescale, dynamic_tree.c ray/shape-cast,
   height_field.c cast) read AABB bounds as packed 3× int64 — scrambled
   bytes once bounds are int128. Caught by the determinism goldens (an
   8-ulp mesh-contact divergence, box-on-box was bit-identical).
   Fixed with `b3BoundToVec3`/`b3Vec3ToBound` converters (identity in
   narrow builds), plus `b3BoundToPos`/`b3PosToBound` for the two exact
   bound↔position crossings. The recording bounds scanner also gated on
   `sizeof( b3AABB )` and raw-memcpy'd the wire payload; the wire format
   is 6× int64 in every build, parsed explicitly now.
2. **Blob section alignment**: compound.c packed heterogeneous sections
   back-to-back with no alignment rounding — fine when everything needs
   ≤8, misaligned stores once `b3TreeNode`/`b3HullData`/`b3MeshData`
   embed 16-aligned int128 bounds. Caught by UBSan. Every section offset
   now rounds to the `_Alignof` of its element type (provably a no-op in
   narrow builds — CompoundTest layout-hash subtests confirm). Allocator
   base pointers were already safe: `B3_ALIGNMENT` ≥ 16 everywhere.

Verified in all three configurations (narrow / wide-positions /
ludicrous): full 22-suite pass in Release AND Debug+VALIDATE+ASan/UBSan
with zero sanitizer output, determinism goldens intact (narrow
0xB222C195, wide 0x886BE415, both sleepStep 287 — ludicrous
self-verifies against the wide golden because in-range scenes take
identical values through int128). The samples app does not build under
`BOX3D_WIDE_POSITIONS` (pre-existing scope boundary from the Option A
work), so it doesn't build under ludicrous mode either; the engine,
tests, and benchmarks all do.
