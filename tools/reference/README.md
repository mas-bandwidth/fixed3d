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
