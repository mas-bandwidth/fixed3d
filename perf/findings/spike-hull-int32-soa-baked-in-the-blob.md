VERDICT: LEARNED — the SoA bake artifact applies cleanly and its unit test holds, but the measurement was NOT quiet (sibling spikes ran concurrently) and the required scalar-oracle bit-exact gate was NOT run, so attempt 1's "wash / gather is not the bottleneck" conclusion is WITHDRAWN and the acceptance gates are unestablished; no adoption, the patch is retained only as the record of what failed to be proven.

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

## Before / after — INCONCLUSIVE, not a clean measurement

The table below is the data attempt 1 collected, and it MUST NOT be read as a
before/after verdict: the run was not quiet. Load average climbed 0.62,0.52,0.55
to 2.75,3.61,2.60 mid-run because sibling spike cards ran beside this one, which
violates the shared-box / quiet-run rule (tools/perf/README.md "Shared-box rules",
and ab.sh refuses exactly this). A measurement taken beside another bench is
garbage up to 2x off (CLAUDE.md, the 2026-07-13 table refresh). The numbers are
kept for provenance only.

(AVX-512, gcc 13.3.0, Release+LTO, 4 workers, 7 repeats, ms — CONTAMINATED)

| scene | before min | before median | after min | after median | delta (min) |
|---|---|---|---|---|---|
| convex_pile | 6953.38 | 7104.25 | 7039.98 | 7254.35 | +1.2% |
| junkyard | 11102.2 | 11247.2 | 11056.2 | 11187.1 | -0.4% |

convex_pile before: 7020.8 7104.25 7135.1 6953.38 6968.32 8038.56 7125.93
convex_pile after:  7172.73 7157.26 7254.35 7275.4 7428.11 7259.98 7039.98
junkyard before:    12147.6 11102.2 11369.3 11247.2 11136.6 11350.2 11174.2
junkyard after:     11056.2 11206.5 11187.1 11248.5 11164.3 11324.9 11304.7

The superficial read is "within spread" (the 8038.56 outlier in the before set),
but with a concurrent bench the spread itself is arbitrary, so the prior
conclusion — "the permutex2var gather is not the bottleneck; removing it buys
nothing measurable" — is WITHDRAWN. The only honest statement is that the effect
of the SoA bake on `b3FindHullSupportVertex` has NOT been measured under quiet
conditions.

## Bit-exact evidence — limited sampled evidence, NOT the scalar oracle

The card's bit-exact gate (ACCEPTANCE #2) is `tools/perf/oracle_check.sh`: EVERY
harness scene at workers 1/2/4, narrow AND ludicrous, must produce state hashes
identical to the frozen scalar oracle, AND `./bin/test` must be green. That gate
was NOT run. Attempt 1's finding said the harness was "absent at this base"; that
was true of its own (older) base but is now FALSE at the re-staged base `9432dff`,
which carries the fixed3d-perf-harness (`tools/perf/oracle_check.sh`,
`tools/perf/scalar-oracle.txt`, `tools/perf/ab.sh` are all present). The honest
status is "present, but not run": no build was reproduced in this attempt because
this rework is artifact-only ("No builds rerun" in the review directive).

What WAS observed (attempt 1's sample, not re-run here):

- `./bin/test` green in the narrow build (RelWithDebInfo+VALIDATE+Werror) and in
  the AVX-512 build (Release+VALIDATE) — the UNIT suite, which does not exercise
  the hashed state oracle.
- `DeterminismTest` green in the AVX-512 build — the determinism goldens
  (ragdoll/wave-pile/query-spawn/mesh-drop sleep steps and hashes) are unchanged.
  This is LIMITED SAMPLED EVIDENCE: four scenarios, not the full harness scene
  matrix, and it is not the `oracle_check.sh` state-hash comparison. It does NOT
  establish that the SoA wide path returns bit-identical support indices on every
  scene/worker/mode.
- `HullBlobSoaMatchesAosTest` (new, in HullTest) pins: every baked lane equals the
  truncation of the AoS value, padding lanes are zero, two builds hash equal, and a
  clone-and-transform whose AABB exceeds int32 bakes nothing (`soaOffset == 0`).
  This is LAYOUT evidence only — it proves the baked SoA bytes are the lossless
  truncation of the AoS vertices — not a simulation-equivalence proof.

Claiming "the SoA wide path returns bit-identical support indices to the scalar
oracle" on this evidence (attempt 1's finding lines 48-56) is OVERCLAIMED and is
hereby corrected to the above.

### UNRUN gates (explicit)

None of the following was run, and none can be claimed complete:

1. **scalar oracle** — `tools/perf/oracle_check.sh` never ran against the SoA
   build; state-hash parity (every scene x workers {1,2,4} x {narrow,ludicrous})
   is NOT established, not even for one scene.
2. **NEON bench** — this box is x86-64 (AVX-512); no ARM/NEON build or bench was
   run (and cannot be on this machine).
3. **ragdoll_pile** — the third target scene was never benchmarked at all.
4. **per-phase** — the card requires min/median/spread per scene AND per phase;
   only whole-scene (2 scenes) numbers exist, and those contaminated.
5. **workers 1 and 2** — only 4-worker runs exist; the oracle's worker {1,2,4}
   agreement was never checked for this change.
6. **ludicrous** — no before/after or oracle run in BOX3D_LUDICROUS_MODE (only
   `sizeof` was confirmed, 272).
7. **query-local staging comparison** — the card's decisive tie-break ("if the
   two give the same gain, DROP because the cheaper change wins") was never
   measured, so the sibling card's per-query staging is unranked against this
   serialized-layout change.

## Versions and sizes

- `B3_HULL_VERSION` 0x5C0B7E41A9D3F218 -> 0x5C0B7E41A9D3F219 (bumped).
- `B3_COMPOUND_VERSION` follows (XOR of B3_HULL_VERSION).
- `B3_REC_VERSION_MAJOR` 9, `B3_SNAP_VERSION` 3: NOT bumped (adopt card must).
- `sizeof(b3HullData)` narrow 224 (unchanged; `padding` -> `soaOffset` rename),
  ludicrous 272 (unchanged). Sum-of-members == sizeof, no unnamed padding.
- Memory cost per hull: `12 * ceil(V/4) * 4` bytes (3 int32 arrays padded to 4
  lanes); 96 bytes for an 8-vertex hull, 48 bytes for a 4-vertex hull.

## What to try next

- Re-run the full gate on a quiet box before drawing ANY perf conclusion:
  apply the patch, build scalar (oracle) + AVX-512, run
  `tools/perf/oracle_check.sh` (bit-exact), then `tools/perf/ab.sh` interleaved
  on convex_pile, junkyard AND ragdoll_pile at workers 1/2/4, per phase.
- Measure the query-local staging sibling card on the same scenes in the same
  session; the conclusion rides on that comparison, which is still missing.
- If the clean rerun reproduces "within spread / no gain", the verdict becomes
  DROP (the cheaper no-layout-change query-local staging wins). If the SoA bake
  moves a phase >1.5x and the oracle holds, reopen as KEEP.

## What not to repeat

- Baking SoA into the serialized hull blob: it costs a layout change + version bump
  + per-hull memory, and this card has NOT shown a quiet-run benefit to justify it.
- Substituting DeterminismTest for `oracle_check.sh` and calling it bit-exact: the
  determinism goldens are four scenarios, not the full state-hash matrix.
- Benchmarking beside another bench and reporting the contaminated numbers as a
  verdict (ab.sh refuses to run under another benchmark for exactly this reason).
- The "AABB exceeds int32" gate is nearly unreachable in practice: `b3CreateHull`'s
  quickhull builder hangs on moderate coordinates (measured: a +-100-unit cube
  never returns; the base tree does the same, so it is pre-existing). Large boxes in
  the benchmark corpus use `b3MakeBoxHull`, not `b3CreateHull`, so the gate only
  matters for clone-and-transform, which this spike handles by clearing the SoA.
