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

Diverges by design: the rotation/velocity of chaotic dynamic scenes (pendulum,
bouncing hull) diverge past their horizon in *any* number system — the horizon is
the number of steps the two engines provably agree, and it is what the header
records, not a guess.
