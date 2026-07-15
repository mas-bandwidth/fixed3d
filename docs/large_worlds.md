# Large Worlds {#large-worlds}

In Fixed3D **every world is a large world**. Positions are Q48.16 fixed point:
a uniform resolution of 1/65536 (about 1.5e-5 meters) at the origin, at
10,000 km from the origin, and everywhere else. There is no precision falloff
with distance, so there is no separate large-world mode to enable — and the
double-precision mode from Box3D is removed. Defining
`BOX3D_DOUBLE_PRECISION` is a compile error in this tree.

For context: float Box3D loses position resolution far from the
origin (a float has a step of about one meter at 1e7 meters), and solves this
with an optional double-precision world-position mode that costs only a few
percent. If a big world is all you need, that mode is the pragmatic choice —
see the README's "Should I use this?" section. What fixed point adds is that
resolution is *uniform*: sub-millimeter everywhere, with nothing to configure.

## The world-position API

The Box3D API's world-position vocabulary is kept so code written against
it compiles unchanged, but the types collapse permanently:

- `b3Pos` is an alias for `b3Vec3`.
- `b3WorldTransform` is an alias for `b3Transform`.
- The boundary helpers (`b3ToPos`, `b3ToVec3`, `b3SubPos`, `b3OffsetPos`,
  `b3RoundDownFloat`, `b3RoundUpFloat`) remain as trivial pass-throughs.

Everywhere the Box3D API accepts or returns a world position —
`b3BodyDef.position`, `b3Body_GetPosition` / `b3Body_GetTransform`, the query
origins (`b3World_OverlapShape`, `b3World_CastShape`, `b3World_CastMover`,
`b3World_CastRay`), hit points, and the body move event — the fixed-point
build uses the same `b3Pos` names with full precision at any distance. There
is no re-differencing machinery because there is nothing to re-difference:
a query at 1e7 meters is exactly as precise as one at the origin, including
the broad phase, whose AABBs are stored in the same fixed-point coordinates
and never quantize.

## Operating range

Positions are representable out to ±1.4e14 meters at full resolution. The
practical simulation envelope is smaller: keep world coordinates within
about ±1e7 meters (10,000 km — a 20,000 km cubed world) so that
squared-distance forms remain representable in Q48.16. The engine's internal
reductions run at 128 bits where it matters, but API-level distance and
length math on coordinates much beyond 1e7 meters can exceed the fixed-point
range.

## Debug drawing

`b3World_Draw` hands every callback world coordinates in `b3Pos`. The engine
side is uniformly precise, but a float *renderer* is not: converting an
absolute coordinate at 1e7 meters through `b3FixToFloat` snaps to the float
grid (about one meter there). Keep a draw origin near the camera and
difference in fixed point before converting to float — `b3SubPos( p,
drawOrigin )` is exact — and a distant scene renders crisply. The sample app
sets the draw origin from the camera eye each frame, and the Large World
sample uses it to render a stack at 1e7 with no jitter.

## Everything runs 120 million kilometers out

Every sample scene, every benchmark scene, and the determinism test build
their content around a shared world origin 120,000,000 km — about 0.8
astronomical units, most of the way to the Sun — from (0,0,0) on all three
axes (`GetSceneOrigin()` in `shared/utils.h` — a constant with no setter,
so nothing can quietly opt back to the origin). For scale: Box3D's
double-precision mode supports a ±120,000 km cube, with its
single-precision broad phase costing about a meter of bounding-box padding
at the edge. This origin is a thousand times past that edge, at the same
1/65536 resolution the engine has everywhere. Working out here enforces
the large-world claim continuously: all performance numbers are measured,
and every sample is exercised, three orders of magnitude beyond the
comparable world. An exactly representable origin shift is a bit-exact
rigid translation of the whole simulation — the falling-ragdolls
determinism scene sleeps on the identical step at 0.8 AU as at the origin
(the sleep step has carried unchanged through four origin moves), and the
benchmark workloads (contact counts, solver stack high-water marks) are
bit-identical — so working out there costs nothing and proves the
uniform-precision claim on every run. Sentinel probes hold clean further
still, out to 1.2e14 m (~800 AU, 85% of the Q48.16 range). The
World > Far samples vary the offset independently of the shared origin.

## Determinism

There is a single precision mode, so there is a single set of determinism
expectations: the same simulation produces bit-identical results across
platforms and worker counts. (Box3D is also deterministic — see the
FAQ — fixed point only changes how that is achieved.)
