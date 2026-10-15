VERDICT: DROP

# Spike: cook int32 SoA vertex arrays into the hull blob when the local AABB fits int32

## Hypothesis

The AVX-512 hull support scan (`b3FindSupportVertexWide` in src/hull.c) gathers
x/y/z lanes out of the AoS `b3Vec3` point array (24-byte stride) with
`_mm256_permutex2var_epi64` per group. Baking the vertices as int32 SoA arrays
(`vx[]`, `vy[]`, `vz[]`, padded to 4 lanes) into the hull blob would let the scan
do straight `_mm_loadu_si128` + `_mm256_cvtepi32_epi64` loads, removing the gather.
The 225-unit range audit already says hull geometry fits int32 Q16.16 losslessly,
so truncating in-range int64 to int32 is bit-identical.

## The change (one paragraph)

`b3HullData::padding` is renamed `soaOffset` (same 224-byte narrow layout, no
unnamed padding). `b3CreateHull` computes the final-vertex AABB before allocation;
when every vertex fits int32 raw Q48.16 it appends `3 * ceil(V/4) * 4` bytes of
int32 SoA after the AoS points and records the offset, otherwise `soaOffset` stays 0.
`b3CloneAndTransformHull` recomputes (or clears and zeroes) the inherited SoA after
the point transform. `b3FindHullSupportVertex` takes a new
`b3FindSupportVertexSoaWide` path (straight loads, no gather) when `soaOffset != 0`
and the existing `b3WideScanAdmissible` gate passes; the scalar and AoS-gather paths
are unchanged. `B3_HULL_VERSION` bumped `0x5C0B7E41A9D3F218` -> `0x5C0B7E41A9D3F219`
(`B3_COMPOUND_VERSION` follows by XOR; `B3_REC_VERSION_MAJOR` 9 and `B3_SNAP_VERSION`
3 were NOT bumped in this spike — the adopt card must).

## Before / after (AVX-512, gcc 13.3.0, Release+LTO, 4 workers, 7 repeats, ms)

| scene | before min | before median | after min | after median | delta (min) |
|---|---|---|---|---|---|
| convex_pile | 6953.38 | 7104.25 | 7039.98 | 7254.35 | +1.2% |
| junkyard | 11102.2 | 11247.2 | 11056.2 | 11187.1 | -0.4% |

convex_pile before: 7020.8 7104.25 7135.1 6953.38 6968.32 8038.56 7125.93
convex_pile after:  7172.73 7157.26 7254.35 7275.4 7428.11 7259.98 7039.98
junkyard before:    12147.6 11102.2 11369.3 11247.2 11136.6 11350.2 11174.2
junkyard after:     11056.2 11206.5 11187.1 11248.5 11164.3 11324.9 11304.7

Both scenes are within their own run-to-run spread (~+-2-3%, convex_pile has a
8038.56 outlier in the before set). The SoA bake is a wash: the permutex2var gather
is not the bottleneck in the support scan, so removing it buys nothing measurable.

## Bit-exact evidence

`./bin/test` green in the narrow build (RelWithDebInfo+VALIDATE+Werror) and in the
AVX-512 build (Release+VALIDATE). `DeterminismTest` green in the AVX-512 build —
the determinism goldens (ragdoll/wave-pile/query-spawn/mesh-drop hashes and sleep
steps) are unchanged, so the SoA wide path returns bit-identical support indices to
the scalar oracle. `HullBlobSoaMatchesAosTest` (new, in HullTest) pins: every baked
lane equals the truncation of the AoS value, padding lanes are zero, two builds hash
equal, and a clone-and-transform whose AABB exceeds int32 bakes nothing
(`soaOffset == 0`). The `tools/perf/oracle_check.sh` harness named by the card is
absent at this base (DEPENDS-ON fixed3d-perf-harness), so the oracle_check lines are
not available; the determinism goldens stand in for the state-hash check.

## Versions and sizes

- `B3_HULL_VERSION` 0x5C0B7E41A9D3F218 -> 0x5C0B7E41A9D3F219 (bumped).
- `B3_COMPOUND_VERSION` follows (XOR of B3_HULL_VERSION).
- `B3_REC_VERSION_MAJOR` 9, `B3_SNAP_VERSION` 3: NOT bumped (adopt card must).
- `sizeof(b3HullData)` narrow 224 (unchanged; `padding` -> `soaOffset` rename),
  ludicrous 272 (unchanged). Sum-of-members == sizeof, no unnamed padding.
- Memory cost per hull: `12 * ceil(V/4) * 4` bytes (3 int32 arrays padded to 4
  lanes); 96 bytes for an 8-vertex hull, 48 bytes for a 4-vertex hull.

## What to try next

Nothing on this axis. The gather is already cheap; the win, if any, is in the
query-local staging the sibling card measured (no layout change, no version bump,
no memory cost). If the support scan ever becomes the profile hotspot, revisit with
a per-query SoA stage rather than a serialized layout change.

## What not to repeat

- Baking SoA into the serialized hull blob: it costs a layout change + version bump
  + per-hull memory for no measurable gain.
- The "AABB exceeds int32" gate is nearly unreachable in practice: `b3CreateHull`'s
  quickhull builder hangs on moderate coordinates (measured: a +-100-unit cube
  never returns; the base tree does the same, so it is pre-existing). Large boxes in
  the benchmark corpus use `b3MakeBoxHull`, not `b3CreateHull`, so the gate only
  matters for clone-and-transform, which this spike handles by clearing the SoA.
