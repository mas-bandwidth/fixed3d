# Notes for Erin

While converting Box3D to fixed point we spent a lot of time staring at your
code, implementing your todos, and stress-testing paths your test corpus
doesn't reach. This file is the useful residue: everything we found or built
that applies to **float Box3D**, with the fixed-point parts filtered
out.

Line numbers reference your tree at `e961bfb`. Performance numbers were
measured in the fixed-point tree (Apple M3 Ultra and AMD EPYC 9124 Zen 4,
4 workers) — our multiplies cost ~4 ops against your single FMA, so
re-measure anything perf-related before believing it transfers. Items are
ordered by how confident we are that you'll care.

## Latent bugs and hazards in your tree

### 1. Over-tight B3_VALIDATE in b3CollideCapsuleAndTriangle

`triangle_manifold.c:480` asserts `faceSeparation <= 0.0f` after clipping
the capsule segment to the triangle face. But
`b3BuildTriangleAndCapsuleFaceContact` only bails out (pointCount stays 0)
when BOTH clipped points exceed `speculativeDistance + radius` — so a
legitimate two-point speculative contact can arrive with both points
positive but under `speculativeDistance`. Your own comment a few lines
later already documents the possibility ("Face contact can be empty if it
does not realize the axis of minimum penetration"). It never fires in your
corpus by luck of trajectories; a slightly different ragdoll drop walked
straight into it. The provable bound given the clip function's
early-return logic is `faceSeparation <= speculativeDistance`, not `<= 0`.

### 2. Quickhull writes out of bounds in Release on inconsistent topology

`b3HullBuilder_NewEdge` (hull.c:210) checks pool capacity with
`B3_ASSERT( b->edgeCount < b->edgeCapacity )` and then increments
unconditionally. The Euler-formula pool budget only holds for consistent
topology — feed the builder a near-degenerate cloud (thin needles,
near-planar grids, epsilon-scale clusters; float jitter can produce these
too) and corrupt topology can exceed the budget. In Release that is an
out-of-bounds write into whatever follows the pool. The face allocator has
the same shape, and the ring walks in ConnectFaces can spin forever on
corrupt topology (we hit an infinite loop there before hardening).

What we did: a `failed` flag on the builder, release-safe allocators that
alias the last slot and mark the build failed instead of writing OOB,
ring-walk bounds capped by pool size, and NULL propagation up through the
driver. Plus a regression test that generates near-degenerate clouds — a
float-scale port of that test would tell you whether your tree can be made
to trip it.

### 3. GJK restores an empty simplex if the solver fails on iteration 0

`distance.c`: `b3Simplex backup = { 0 }` (line 781); on a solver failure
the code does `B3_ASSERT( backup.count != 0 ); simplex = backup;` (lines
830, 888). If a *cached* simplex is degenerate — duplicate support points,
which vertex-on-vertex configurations can produce in float too — the
solver can fail on the first iteration, before `backup` was ever written.
Debug asserts; Release continues with a zero-count simplex. Our fix:
detect duplicate vertices in the warm-start simplex, flush, and restart
from a fresh single-vertex simplex instead of restoring the backup.

### 4. The dynamic tree's binned SAH scores empty-side planes; mesh.c doesn't

Your mesh BVH build guards split evaluation with
`if ( leftCount > 0 && rightCount > 0 )` (mesh.c:628). The dynamic tree's
binned partition has no such guard — an empty side carries the empty-AABB
sentinel (`FLT_MAX` bounds), the cost comes out NaN via 0×inf, and NaN
comparisons happening to be false is the only thing keeping those planes
from being selected. It works, but it works by IEEE accident. The explicit
skip is one line and makes the two SAH sites consistent.

### 5. The test runner loses the crash location to stdout buffering

When the full suite crashes, stdout's buffer dies with the process, so the
last printed "subtest passed" line points at the wrong suite — we chased a
"JointTest crash" twice that was actually ManifoldTest, suites apart. One
`setvbuf( stdout, NULL, _IOLBF, ... )` in the test main (or a flush per
subtest) makes traps attributable.

### 6. b3Log has no printf format attribute

`core.h:135` declares `void b3Log( const char* format, ... )` bare, so
mismatched varargs are invisible to `-Wformat`. One attribute line makes
the compiler check every call site. All our call sites happened to be
correct when we added it — yours probably are too, until one isn't.

### 7. The large_world benchmark scene is balanced on a knife edge

We found this chasing a phantom 4–20x slowdown: if the spheres land ~55 mm
off the floor-box seams (any placement-math perturbation will do), the
off-center impact catches the neighbor box's top edge and kicks them into
steady-state pure rolling — and with zero rolling resistance, pure rolling
defeats friction, nothing sleeps, and the awake set grows monotonically.
We forced YOUR float build to the same landing spot: 79/100 spheres awake,
rolling forever. So the benchmark silently measures "did the spheres hit
the seams exactly," and any engine change that shifts trajectories slightly
can multiply its cost several-fold without anything being wrong. Placement
math in benchmark scenes wants to be robust to that (we switched shared
scenes to arithmetic that is exact in both number systems; for you, exact
binary-representable spacings would do it).

### 8. Three shape mutators don't record: SetHull, SetMesh, SetMeshMaterial

Your recording manifest covers the shape mutators through `ShapeSetName`
(0x5C), but `b3Shape_SetHull` (shape.c:1589), `b3Shape_SetMesh`
(shape.c:1630), and `b3Shape_SetMeshMaterial` (shape.c:1286) have no
`B3_REC` call and no op — a recorded session that swaps a shape's
geometry or retunes a per-triangle material replays divergent, because
the replayed world keeps the old hull/mesh/material. The material op is
a one-liner next to `ShapeSetSurfaceMaterial`. The geometry pair needs
interning like your create ops: intern the hull/mesh into the registry
at the record site and carry a GEOMID the dispatch resolves the way
`CreateHullShape`/`CreateMeshShape` do. One placement subtlety:
`b3Shape_SetHull`'s shared-hull dedup short-circuit returns without
mutating, so record after it, or the stream (and the registry) carries
geometry for calls that changed nothing. We shipped these as ops
0x5D/0x5E/0x5F with a minor version bump; the negative test — delete
the `B3_REC` call, watch the replay hash gate trip — is cheap and worth
keeping.

### 9. Compound blob packing misaligns b3HullData — live UB, one hull is enough

`b3CreateCompound` packs its blob sections back-to-back with no alignment
rounding (compound.c:472, your own `todo 64 byte alignment` comment sits on
the tree-node section). That's fine only while every section size is a
multiple of every later section's alignment — and two of yours aren't:
`b3HullInstance` is 36 bytes and `b3MeshInstance` is 60 (compound.c:30–43;
a 28-byte transform plus uint32 tails, the mesh instance adds a 12-byte
scale). `b3HullData` and `b3MeshData` both start
with `uint64_t version`, so a compound with an **odd number of hull
instances** puts its first shared hull blob at offset ≡ 4 (mod 8).
Verified on pristine e961bfb: a compound with a **single box hull** —
`def.hullCount = 1`, nothing else — fires

    compound.c:582:11: runtime error: store to misaligned address 0x...72c
    for type 'b3HullData *', which requires 8 byte alignment

under `-fsanitize=undefined`. Odd mesh-instance counts misalign
`b3MeshData` the same way. Your corpus presumably uses counts that happen
to land aligned; ours did too until we widened the AABB in an experiment
and UBSan lit up our copy of the same code. On x86-64/arm64 the misaligned
uint64 loads work silently today, so the practical exposure is UBSan noise,
strict-alignment targets, and any future typed-SIMD access through those
pointers — but it's UB as it stands. The fix is small and free: round each
section offset up to the `_Alignof` of the element type it holds
(`offset = (offset + align - 1) & ~(align - 1)` before each section; the
blob is already memset, so the pad bytes don't perturb your content
hashes). That also gives you the hook for the 64-byte tree-node alignment
your todo asks for — same rounding, bigger constant.

## Your todos, implemented and measured

We did your homework. Some of it was worth doing.

### 9. `todo_erin use the max point count of the four manifolds` — DO THIS

contact_solver.c:2032. Prepare stores the widest manifold across the four
lanes; warm start, solve, and restitution loop only that far. Slots past a
lane's own count hold exact zeros and apply nothing, so skipping them
changes no results. One-point-manifold scenes (spheres, rain) stop paying
for four point slots. Pure win, no downside, benefits scalar tails too.

### 10. `todo speed this up using matrices` (anchor rotation) — worked

contact_solver.c:2042. The delta rotations are constant across a wide
constraint, so build rotation matrices once per constraint and rotate each
anchor as three row dots instead of the quaternion two-cross dance per
point. Combined with precomputing the contact Jacobian rows (cross(r,n),
invI·cross(r,n), invMass·n, invI·n, cross(origin,tangent)), our
solver-bound scenes gained 10–16%; light scenes gave back 5–7% because the
constraint gets fatter and warm start becomes bandwidth-bound. Caveat for
you: precomputed rows paid for us because our multiplies are expensive —
against float FMA the memory-for-multiplies trade may not clear, so
measure the two halves separately. The matrix rotation half should
transfer as-is.

### 11. `todo test computing the tangents on the fly, at least tangent2` — measured, rejected (for us)

contact_solver.c:1360. Dropping the stored tangent2 and recomputing
cross(tangent1, normal) at each use is correct and saves 48 bytes per wide
constraint — and it measured a wash to slightly SLOWER on both of our
platforms: the recompute is 24 extra (expensive) multiplies per constraint
per pass, and 48 bytes is ~1.3% of a constraint that streams anyway. Your
float cross is cheap enough that this could flip for you, but don't expect
much: the bytes just don't matter.

### 12. `todo_erin measure perf padding to 64 bytes` (b3BodyState) — an alignment lottery

body.h:159. Measured on two microarchitectures with interleaved min-of-3
A/Bs: geomean −0.4% on one machine, +0.3% on the other, and the per-scene
signs expose it as noise — two nearly identical pyramid workloads moved
in OPPOSITE directions, consistently, on the same machine. That's
reshuffled cache-set conflicts, not a systematic effect. Not worth the
memory.

### 13. Branchless friction-cone guard — measured, rejected

Skipping the sqrt+div when no lane exceeds the friction cone sounds free.
It measured neutral-to-worse: in never-sleeping benchmark scenes the
saturation mask is per-lane noisy so the guard branch mispredicts, and the
out-of-order core was already hiding the sqrt/div latency across four
independent lanes. Also: the compiler already emits csel/cmov for every
lane ternary — the compiled solver was 56 conditional-selects to 33
branches before we touched anything. There is no branchless win hiding in
there.

## SIMD your floats are leaving on the table

This is where the fixed-point build wins its head-to-head benchmarks, so
here it is in earnest. Your SIMD stops at the contact solver. The narrow phase is scalar on every
platform, and it doesn't have to be. None of this needs fixed point — in
float it's *easier*, because we needed exactness gates and you just need
dot products.

### 14. The SAT edge query, four edge pairs at a time — the big one

`b3QueryEdgeDirections` was 63% of convex_pile for us before this. The
structure: a per-call SoA prepass over hull A's edge vectors and adjacent
face normals (uint8 edge indices — mind your hull budgets; ours cap at 128
edge pairs for stack arrays), then BOTH Minkowski/Gauss-map sign tests run
four edge pairs per iteration on un-negated dots — `(dA×dB)` and
`(cba×dB)` sign forms replace the scalar sign flips. Survivors (genuine
Gauss-map arc intersections, a tiny fraction of pairs) run the exact
scalar inner body, moved verbatim into a shared function so the wide and
scalar paths cannot diverge on admission. It vectorizes with SSE2 and NEON
float ops directly. Whole-benchmark effect for us: convex_pile 21.4 s →
10.3 s on M3 (NEON), 126.8 s → 53.1 s on Zen 4 (AVX-512). This one change
is most of why the integer build beats your float build on the hull pile.

### 15. A wide mesh contact solver — answers your constraint_graph.h todo

Your comment block muses about lumping mesh contacts with convex wide
constraints (re-linking when manifold counts change) or Dirk's
manifold-based coloring (manifold identity and color-overflow concerns).
There's a third option that sidesteps both: **lane = whole contact**.
Graph coloring already guarantees contacts in a color share no bodies, so
four whole mesh contacts gather/scatter safely with zero graph changes;
manifolds keep their identity; and the manifold dimension serializes
in-register, preserving Gauss-Seidel across a contact's manifolds — the
property your "much more stable for the Jenga stack" comment is about.
The ragged dimension (manifoldCount) is sized per group of four as the
widest lane, with a flat slot array and per-slot start table built in
solver setup.

Measured (Zen 4, AVX-512): trees100 −22%, trees50 −21% — and that's
carrying 49% wasted lane slots from ragged manifold counts, so don't fear
the raggedness. The one loss: rain (+1.8%), whose mesh contacts are all
1-manifold — the tax is fixed per-slot overhead on tiny manifolds, not
waste. Two free mitigations if that matters: a 1-manifold fast path, or
sorting each color's contacts by manifoldCount through an indirection
(order within a color cannot affect results — the bodies are disjoint).

### 16. Hull support scans, four elements at a time

`b3FindHullSupportVertex/Face` vectorize with one subtlety worth stealing
even if you don't vectorize: tie order. The scalar scan is first-wins; a
naive SIMD max is whichever-lane-wins. We preserve first-wins with
per-lane runs and a value-then-smaller-index reduction, which keeps the
vectorized scan's result identical to the scalar one — the kind of thing
your cross-platform determinism story needs if these ever go wide.

### 17. Body gather/scatter as 4×4 transposes, and an overread trap

Gathering four bodies' states as three 4×4 transposes over the contiguous
fields beats per-lane member loads. The trap: transposing ALL the fields
wants a fourth row load that reads past the end of the states array for
the last body. Keep the final field scalar (or pad the array). Static
asserts pinning the field offsets saved us twice during refactors.

## Small stuff

- **Restitution early-out:** skip the whole wide constraint when all four
  lanes have zero restitution. It's the common case and it skips the body
  gather too.
- **Prepare/store as a flat parallel-for:** one uniform block range over
  the whole wide-constraint array, with a small per-color span table
  consulted inside the task, parallelizes better than per-color loops.
- **Hashed blobs and padding:** any struct that gets memcpy'd into a
  content hash needs deterministic padding bytes — memset or fully stage
  the memory before hashing. This class bit us twice: layout changes grew
  pad bytes inside a mesh node struct and silently broke mesh
  deduplication, and the compound material dedup hashed structs with
  uninitialized padding — deterministic failure, but only under one
  compiler configuration, which is the nasty part. If your material or
  node structs ever grow padding, the dedup breaks silently. Cheap rule,
  annoying bug.
- **Benchmark harness papercut:** unrecognized arguments are silently
  ignored, so `-b large_pyramid` (space instead of `=`) runs the full
  suite and looks like a hang. Erroring on unknown args would have saved
  us an embarrassing hour.
- **Sample text helpers have no printf format attribute:** your
  `Sample::DrawTextLine`, `DrawString3D`, and `DrawScreenStringFormat`
  (e961bfb samples/sample.h:183 and the gfx headers) are varargs with no
  `__attribute__(( format( printf, ... ) ))`, so -Wformat is blind to
  their call sites. Adding the attribute (the core's b3Log already has
  it) immediately caught real varargs UB in your own sample code that we
  inherited: `uint64_t` material ids printed with `%d`
  (e961bfb samples/sample_events.cpp:221, plus the compound/collision
  material readouts) and a `uint64_t` tick count through `%ld` — broken
  on Windows where long is 32-bit. Three lines of attribute, compiler
  finds the rest.
- **Character sample material "ids" land in rollingResistance:** the
  positional initializers in e961bfb samples/sample_character.cpp:336-338
  (repeated at 396-398, 1336-1338, 1396-1398) read
  `materials[1] = { 0.6f, 1.0f, 1 };` — the third positional field of
  b3SurfaceMaterial is rollingResistance, so the 0/1/2 literals (which
  look like the userMaterialId-by-slot pattern your mesh and collision
  samples set via designated initializers) compile to rolling resistance
  1.0 and 2.0 on the level mesh, while userMaterialId stays 0. Rolling
  resistance only acts on spheres and capsules — exactly the character
  capsule those levels host. If ids were the intent, `.userMaterialId = 1`
  is the one-line fix. We only noticed because in fixed point the bare `1`
  quantized to ~1.5e-5 instead of 1.0 and an AST audit flagged the
  implicit int→fixed conversion; in float it is silent and value-changing
  all the same.
- **A per-sample screenshot sweep is a cheap, shockingly effective
  regression net:** we ran every sample to its 120-frame end state,
  captured one PNG each, and paired ours against your float build's — 153
  pairs, sorted by a 64×36 grayscale mean-difference so the divergent ones
  surface first. An afternoon of harness work; it caught one real physics
  divergence (an equilibrium knife-edge our number format can't hold) and
  certified every other sample visually identical to yours within
  antialiasing noise. App-side cost is three small flags (`--list-samples`,
  `--sample N --frames F`, `--capture out.png`) plus an optional headless
  mode so a sweep doesn't own your desktop for ten minutes. One Metal/sokol
  trap if you build the headless variant: the readback padding frames must
  contain a REAL render pass — an empty `sg_commit` creates no command
  buffer and therefore no in-flight frame rotation, so the cross-queue blit
  races the render and reads undefined memory (solid magenta on Apple
  GPUs). Three LOAD-action passes on the capture target force the rotation
  that guarantees completion.
- **shape.h declares b3GetShapeArea and b3GetShapeProjectedArea twice**
  (shape.h:95-96 and again at 107-108, verbatim) — and nothing in the tree
  calls either outside their definitions. Cosmetic, but it reads like a
  merge leftover, and dead API invites drift.

---

Everything above was verified against the full test suite plus a
determinism golden (state hash + sleep step over a few hundred frames of
ragdolls on meshes, checked across 1–5 worker threads) on arm64 and
x86-64. That harness catches a one-ulp change anywhere in the solver — we
can recommend the setup regardless of what your numbers are made of.
Box3D was already deterministic when we got here, and having now lived
inside the machinery: the bulk of that is your multithreading determinism
work — simulation order pinned to creation order, deterministic event
ordering, identical results at any worker count — which this fork inherits
whole and guards with the same cross-worker golden you'd recognize. The
fixed-point part only changes how the math layer underneath gets its
determinism. The rest was always yours.
