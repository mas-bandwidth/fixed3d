# The float box3d reference harness

This directory holds the producer every other card of the port stream consumes: a
harness that runs **one scenario through vanilla float box3d (the reference) and
through fixed3d**, then compares the results inside a documented tolerance. The
harness is OFF by default (`BOX3D_REFERENCE=OFF`), so a plain configure builds
exactly as before.

## Layout

- `ref_scene.h` / `ref_scene.c` — the shared, engine-free C parser and the trace
  comparer. Numbers cross the engine boundary as `double`; no box3d header is
  included, so one source builds against both engines.
- `run_fixed.c` — links the in-tree fixed3d. Converts every decimal with
  `b3FixFromDouble` at the one sanctioned boundary.
- `run_float.c` — compiled against vanilla box3d at the commit named by
  `-DBOX3D_REFERENCE_REV=<sha>`, fetched with `ExternalProject` into
  `$BOX3D_REFERENCE_CACHE` (default `~/.cache/fixed3d-reference/<sha>`), built once
  and reused. The pin and the cache key are the same sha.
- `ref_compare.c` — reads a scene header and two traces, prints a per-quantity
  maximum-error table and the first step that breaks the tolerance, exits 0 only
  inside tolerance.
- `run_reference.sh <rev> <scene...>` — runs both drivers and the comparer. The
  built binaries are found through `REFERENCE_BIN_DIR` (set automatically by the
  `ctest -L reference` wiring).
- `scenes/*.scene` — the seed scenes, one directive per line, decimal numbers only.

## Build and run

```
cmake -B build-ref -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBOX3D_SAMPLES=OFF \
      -DBOX3D_REFERENCE=ON -DBOX3D_REFERENCE_REV=47d7f7c
cmake --build build-ref --parallel
cd build-ref && ctest -L reference --output-on-failure
```

The unit tier (no engine, no network) is `./bin/test ReferenceSceneParseTest` and
is always built with the unit suite; it parses an in-memory scene and compares two
hand-written traces inside and outside tolerance.

## Scene format

A scene is plain text, one directive per line, `#` comments, decimal numbers only.
The **first non-comment line** is the tolerance header:

```
expect pos <m> rot <rad> vel <m/s> horizon <steps> counts exact|<n>
```

- `pos`, `rot`, `vel` are the per-quantity tolerances (position in m, rotation in
  rad, linear velocity in m/s). Angular velocity uses the `vel` tolerance.
- `horizon` is how many of the emitted steps are compared; a chaotic scene diverges
  after a horizon in any number system, so the horizon is **measured, not guessed**
  (the Card House record in `CLAUDE.md` is the precedent).
- `counts exact` means the per-step integer counters (contact count, awake body
  count, joint count) must match exactly; `counts <n>` allows a delta of `<n>`.

The directive vocabulary (v1):

```
world gravity <x> <y> <z> dt <num> <den> substeps <n> steps <n>
body <static|dynamic|kinematic> position <x> <y> <z> rotation <x> <y> <z> <w>
     linvel <x> <y> <z> angvel <x> <y> <z> [flags <bullet|fastrotation|sleep|nosleep>...]
shape <body> box <hx> <hy> <hz> density <d> friction <f> restitution <r> [rolling <rr>] [sensor]
shape <body> sphere <r> [center <x> <y> <z>] density <d> friction <f> restitution <r> [rolling <rr>] [sensor]
shape <body> capsule <r> <half> [axis <x> <y> <z>] density <d> friction <f> restitution <r> [rolling <rr>] [sensor]
shape <body> hull points <n> <x> <y> <z> ... density <d> friction <f> restitution <r> [sensor]
joint revolute <a> <b> anchor <x> <y> <z> [axis <x> <y> <z>]
joint spherical <a> <b> anchor <x> <y> <z>
joint distance <a> <b> anchorA <x> <y> <z> anchorB <x> <y> <z> length <l>
joint weld <a> <b> anchor <x> <y> <z> [axis <x> <y> <z>]
query raycast origin <x> <y> <z> translation <x> <y> <z>
query overlap_aabb lower <x> <y> <z> upper <x> <y> <z>
query overlap_shape origin <x> <y> <z> <box hx hy hz | sphere r | capsule r half [axis x y z]>
query shape_cast origin <x> <y> <z> translation <x> <y> <z> <box hx hy hz | sphere r | capsule r half [axis x y z]>
query closest_points <a> <b>
query mover origin <x> <y> <z> <radius> <half> translation <x> <y> <z>
```

Bodies are created in file order (indices start at 0); shapes attach to the body
named by `<body>`; joints connect two body indices; queries run once after the last
step. The driver normalizes each rotation quaternion and capsule axis at the
boundary, so slightly off-unit decimal quaternions are fine.

## Trace format (both drivers, byte-for-byte identical structure)

```
step <n> counters <contactCount> <awakeBodyCount> <jointCount>
body <i> p <x> <y> <z> q <x> <y> <z> <w> v <x> <y> <z> w <x> <y> <z>
...              (one body line per body per step)
query <i> <kind> <payload...>
```

All numbers are printed `%.9g`. Query payloads, per kind:

- `raycast` `<hit> <fraction> <px> <py> <pz> <nx> <ny> <nz>`
- `overlap_aabb` / `overlap_shape` `<count>`
- `shape_cast` `<hit> <fraction> <px> <py> <pz> <nx> <ny> <nz>` (no hit prints `0 1 0 0 0 0 0 0`)
- `closest_points` `<dist> <px> <py> <pz>`
- `mover` `<fraction>`

## Adding a directive

Add a new line type by adding a branch in `ref_scene.c` (`parseBody`, `parseShape`,
`parseJoint`, or `parseQuery`), a field to the matching struct in `ref_scene.h`, and
a matching `printf`/query emission in **both** `run_fixed.c` and `run_float.c`.
Never edit another card's scene; a later card extends the vocabulary this way.

## Generator-form scenes (the `ref-<sha>-*` form)

A landing port card may state its observable as final-step bounds instead of a
per-quantity header. Those scenes use the generator form, accepted by the same
parser (`ref-9f998c8-complex-hull.scene` is the first):

```
scene <name>
hull_generator complex <seed> [radius <r>]
body hull position <x> <y> <z> [density <d>]
body box half <hx> <hy> <hz> position <x> <y> <z> [density <d>] [static|dynamic|kinematic]
steps <n> dt <num>/<den> [substeps <n>]
expect contactCount <op> <n>
expect restingY <op> <y>
```

- `hull_generator complex <seed>` expands, engine-free and in double, to the 32
  hull points of `b3CreateComplexHull` (box3d 9f998c8): the PEEL BasicRandom LCG
  (`state = state*2147001325 + 715136305`, low 16 bits mapped by
  `raw/65535 - 0.5`), rejection-sampled inside the unit sphere, normalized,
  scaled by `radius` (default 0.5). Both engines then build the hull from ONE
  shared point list — the fixed side quantizes it at the `b3FixFromDouble`
  boundary, exactly the divergence the scene documents ("the /65535.0f map is
  quantized to Q48.16"). No engine-side generator call is needed, so the scene
  runs against the same pinned revision as the seeds.
- Omitted material fields default to the corpus values: density 1, friction
  0.5, restitution 0. Omitted gravity defaults to `b3DefaultWorldDef`'s
  (0, -10, 0). Omitted substeps default to 4. `dt` must be the exact fraction.
- `expect` predicate lines (`contactCount`, `restingY`; ops `>= > <= <`) state
  absolute bounds on the FINAL step. A scene uses the predicate form OR the v1
  header, never both.
- Predicate-mode traces carry one extra line per step per hull-shape body:
  `low <i> <y>`, the lowest transformed hull vertex (the resting height). v1
  scenes never emit it.
- The comparison in predicate mode: every predicate must hold on BOTH traces,
  the final contact count must be equal, and the final resting height must
  agree within `B3_REF_RESTING_Y_TOLERANCE` (0.05 m — the tolerance the landing
  card stated for its scene, carried here because the predicate grammar has no
  place for it). The comparer prints the measured deltas; the table below
  records them.

## Tolerance policy

Static and query scenes hold within `4 * B3_FIXED_EPSILON + 1e-4 relative`
(≈ 1.2e-4 m) and compare `counts exact`. Dynamic scenes hold within **the measured
maximum error times two over the stated horizon**. The measured maximum error for
each seed scene (fixed3d vs vanilla box3d `47d7f7c`, this machine) is:

| scene | pos (m) | rot (rad) | linvel (m/s) | angvel (rad/s) | horizon |
|---|---|---|---|---|---|
| box_settle | 2.7e-4 | 4.9e-5 | 7.7e-4 | 1.5e-3 | 120 |
| box_stack | 3.4e-3 | 2.8e-4 | 1.1e-2 | 5.2e-3 | 120 |
| box_pyramid | 5.6e-4 | 4.9e-5 | 6.1e-3 | 4.8e-3 | 120 |
| sphere_restitution | 7.5e-4 | 0 | 1.3e-3 | 0 | 120 |
| capsule_roll | 5.1e-4 | 4.5e-4 | 7.7e-3 | 1.5e-2 | 60 |
| hull_drop | 7.3e-4 | 5.6e-4 | 2.6e-2 | 3.6e-2 | 120 |
| pendulum_chain | 3.6e-3 | 6.1e-4 | 7.5e-2 | 1.2e-2 | 60 |
| queries | 0 | 0 | 0 | 0 | 1 |
| ref-9f998c8-complex-hull | resting height delta 2.1e-5 (tol 0.05, generator form) | | | | 120 |

**Counter divergence is sleep timing, and it is measured.** With the counters
actually parsed (the `step <n> counters <c> <a> <j>` tail — a latent harness bug
fixed with this change: the tail was silently dropped and every counter
comparison passed vacuously), the contact and joint counts hold **exact** in
every scene; only the AWAKE count diverges, by the size of the sleeping island
while one engine's island sleeps a few steps earlier than the other's:
box_settle 1, box_stack 5, box_pyramid 10, hull_drop 1 (sphere_restitution,
capsule_roll, pendulum_chain, queries and the complex-hull scene hold exact).
The four headers therefore state `counts <measured awake delta * 2>`
(2 / 10 / 20 / 2) per the measured-error-times-two policy.

Diverges by design: the rotation/velocity of chaotic dynamic scenes (pendulum,
bouncing hull) diverge past their horizon in *any* number system — the horizon is
the number of steps the two engines provably agree, and it is what the header
records, not a guess. The complex-hull scene's fixed hull differs from the float
hull by construction (the shared generator point list is quantized to Q48.16 at
the `b3FixFromDouble` boundary), which is why its observable is the resting
height and contact count, not the transform trace.

## Compound, mesh, snapshot, and point-set directives

These directives use the same public APIs in both drivers. A baked compound and a
mesh **must attach to a static body**. Both engines reject a baked compound on a
dynamic/kinematic body; the parser reports this instead of silently running a
scene without its geometry. Use ordinary multiple shapes for runtime compounds.

```
shape <body> compound children <n> <child> ...
```

Each child is one of the following, followed by the **required** common tail:

```
box <hx> <hy> <hz>
sphere <radius>
capsule <radius> <halfLength>
hull points <n> <x> <y> <z> ...

position <x> <y> <z> rotation <qx> <qy> <qz> <qw>
density <d> friction <f> restitution <r>
```

Write the entire shape on one line. Positions and rotations are local to the
compound. Quaternions are normalized at the driver boundary; capsules initially
lie along local +Y. A box becomes a public box hull, not a set of independent
runtime shapes. A compound has one genuine baked shape and owns its geometry
until its world has been destroyed. The current bounds are 128 children across
the scene and 128 points per hull. Non-finite numbers, zero quaternions,
nonpositive dimensions, invalid counts and negative material values are refused.

**Public API limitation:** both engines' `b3Compound*Def` children carry surface
materials, but density belongs to the enclosing `b3ShapeDef`. Child densities
must therefore be equal; unequal values fail with an explicit API-gap error.
A future heterogeneous-density extension requires an engine public API addition
(per-child density in compound definitions plus corresponding baked mass data).
This harness does not flatten geometry or implement substitute mass mathematics.

```
shape <body> mesh vertices <n> <x> <y> <z> ... triangles <m> <i> <j> <k> ...
    [materials <count> <friction> <restitution> ... indices <material-index> ...]
```

Again, one physical line. Indices are zero-based, triangles use counterclockwise
winding, and the optional material table has one friction/restitution pair per
material and **one material index per triangle**. The default is one material
(friction 0.5, restitution 0). Bounds: 64 vertices, 128 triangles, 16 materials.
The mesh builder identifies shared edges. Invalid vertex/material indices and
non-static bodies are refused. Degenerate triangles cause the driver to fail
rather than silently reduce the requested triangle count. For example:

```
shape 0 mesh vertices 4 -10 0 -10 -10 0 10 10 0 10 10 0 -10 triangles 2 0 1 2 0 2 3 materials 2 0.5 0 0.7 0.1 indices 0 1
shape 1 compound children 1 box 2 0.5 2 position 0 0 0 rotation 0 0 0 1 density 1 friction 0.5 restitution 0
```

### Snapshot boundaries

```
snapshot save at 30
snapshot restore at 60
```

There is one save/restore pair, with `0 <= save < restore < steps`. Both boundaries
are **before** the numbered step and its events. Each driver saves a snapshot seed
using public `b3CreateRecording`, `b3World_StartRecording`, and
`b3World_StopRecording`; restoration uses `b3CreatePlayer`,
`b3RecPlayer_GetWorldId`, and `b3RecPlayer_GetBodyId`. The restored player remains
alive while its world is stepped directly so borrowed geometry remains valid.
No private headers or serialization structures enter the harness.

Trace step numbers continue monotonically. Simulation time rewinds: output step
60 executes logical step 30, so output steps 60–89 repeat 30–59. Scheduled joints
and mass events follow that logical clock, including repetition after restore.
Queries run against the final restored world. Bodies are never deleted by this
vocabulary, so the public player's seed body enumeration matches creation order.

The checked vanilla pins `954cf87`, `f8c4fe8`, and `79fc559` all expose this real
snapshot-seeded player API: **neither driver uses a replay fallback at these
pins**. The brief's fallback for a float pin without public restore is to rebuild
from step zero and replay all events through the save boundary, without emitting
those replay steps. Such an older pin would need that adapter before it can be
used here; there is no simulated restore substituted for the fixed snapshot.

### Complete contact point sets

```
trace contacts
```

This requires a numeric tolerance header (predicate mode is refused). It adds
these lines after the bodies of **every** step, including steps with zero touching
manifolds:

```
contacts <manifold-count>
contact <shape-a> <shape-b> <point-count> <x> <y> <z> <separation> ...
```

Shape ordinals are `body-creation-index * (B3_REF_MAX_SHAPES + 1) + body-local-shape-index`,
using public body/shape enumeration, never the internals of engine handles. The
smaller ordinal comes first. Contact positions are world-space midpoints of the
two center-relative anchors. Separation retains its sign. Rows are sorted by
shape pair, point count, then lexicographic point coordinates/separation; each
row's points are likewise sorted. Zero-point manifolds are omitted. The current
limits are 256 nonempty manifolds per step and four points per manifold; exceeding
a limit fails explicitly instead of truncating the trace.

The comparer requires both complete sets, matches shape pairs and both manifold
and point counts **exactly**, and compares positions **and separations** within
`pos`. `counts <delta>` does not relax these checks. It finds a bijection between
point sets, and between compatible manifolds, rather than relying on rounded
lexicographic order or greedy nearest-neighbor selection. Near-equal coordinates
can legitimately swap order between float and fixed; a point still cannot be
reused to cover two different points. Missing/truncated sets and non-finite
contact values fail. Existing scenes omit the directive and retain byte-identical
traces.

### Measured new scenes

On Linux, fixed stream base `59eff1c` versus vanilla
`79fc55976a041d63f160da1de3ef8d224249ce7e`, narrow and ludicrous produced the same
maximum errors below. Tolerances were tightened after measurement, never widened
to hide a failing scene.

| Scene | Maximum position/point error | Maximum velocity error | Query error | Horizon |
|---|---:|---:|---:|---:|
| ref-harness-compound-mesh-vocabulary | 0.00011086 | 0.00048745 | 0.000002861 | 12 |
| ref-harness-snapshot-restore | 0.001709133 | 0 | 0 | 90 |
| ref-5d83df8-compound-roundtrip | 0.000976487 | 0 | 0.000042916 | 90 |
| ref-954cf87-manifold-points-exact | 0.000030518 | 0 | 0 | 4 |

The roundtrip scene builds all four convex child kinds, keeps a sphere touching
the static compound, and moves a separate free body to make the restore boundary
observable. A sliding touching sphere was unsuitable for this horizon: fixed and
float contact-recycling cadence diverged at step 36 (point error 0.024405882).
The final scene avoids that unrelated divergence without relaxing tolerances.
The exact-point scene uses zero dt to isolate the four-point face manifold from
the solver. Both contact scenes have nonzero point sets. Within each engine,
steps 60–89 of both snapshot scenes repeat steps 30–59 byte-for-byte. Omitting
the compound or the restore makes the roundtrip comparison fail.

At this single float pin, the unchanged baseline already fails `box_stack`,
`pendulum_chain`, `ref-954cf87-collide-connected-false`,
`ref-a2e4816-speculative-sphere-sphere`, and
`ref-f8c4fe8-shape-cast-encroach-band`. The change preserves all 25 older scenes'
fixed and float traces byte-for-byte in both configurations; it neither changes
those mixed-pin scenes nor relaxes their tolerances.

The historical deferred `ref-5643cd8-compound-six-materials.scene` and
`ref-3d9db79-compound-identical-builds.scene` remain follow-up work. Mesh children
inside compounds and their per-child material query vocabulary are outside this
extension's four convex child types.
