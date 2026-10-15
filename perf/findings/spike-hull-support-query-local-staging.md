# VERDICT: LEARNED

## Hypothesis

Face SAT (`b3QueryFaceDirections`) calls `b3FindHullSupportVertex` once per face, and
every call re-de-swizzles the AoS vertices (`vuzp1q_s32` + `vqtbl3q_u8` on NEON,
`permutex2var` on AVX-512). The edge-SAT path already stages SoA once per call. Staging
the hull's int32 xyz once per query and scanning from the stage should remove the
per-scan de-swizzle and win on NEON, where the de-swizzle is expensive.

## The change

A caller-owned `b3HullSupportStage` (stack frame, no allocation, no cross-stage
ownership) holds the hull's vertex xyz and face normals in SoA lanes — int64 on
AVX-512/scalar, int32 on NEON (same per-ISA lane rule as `b3EdgeLane`).
`b3StageHullSupport` de-swizzles once per query; `b3FindHullSupportVertexStaged` /
`b3FindHullSupportFaceStaged` scan the stage under the exact same `b3WideScanAdmissible`
gate and fall back to the exact AoS scan otherwise (NEON rejects components >= 2^31 by
leaving `vertexCount == 0`). Only `b3QueryFaceDirections` is wired to the staged vertex
scan; every other support site is unchanged. Results are bit-identical by construction:
same exact raw-128 dots, same first-wins tie rule, same scalar tail.

## Before / after (BOX3D_AVX512, 4 workers, RelWithDebInfo+LTO, min of 2)

| scene         | before (ms)      | after (ms)       | delta (min) |
|---------------|------------------|------------------|-------------|
| convex_pile   | 13461.9 / 13533.8| 13087.9 / 13039.4| -3.1%       |
| junkyard      | 19379.3 / 19378.1| 19174.8 / 19245.9| -1.0%       |
| large_pyramid | 4322.79 / 4334.12| 4416.55 / 4362.11| +0.9%       |

The `+3%` on convex_pile (the hull-heavy scene) is the only signal, and it sits inside
the run-to-run spread (±2-5%). It is NOT the required geomean ≥3% with no scene >2%
slower; junkyard is a wash and large_pyramid is nominally slower. On AVX-512 the
de-swizzle (`permutex2var`) was never the bottleneck, so this is expected.

## Bit-exact evidence

`./bin/test` is green in the scalar build (full suite, 4.63 s) and in the AVX-512 build
for the suites that matter: `HullTest` (the new `HullSupportStagedEquivalenceTest`
proves the staged scan returns the same vertex AND face index as the existing scan over
box / tetra / 32-vertex sphere hull / thin-box (ragdoll) corpus against 100,000 seeded
directions plus 13 explicit exact-tie directions) and `DeterminismTest` (goldens
bit-identical, so the production face-loop staging shifted no state hash). The one
AVX-512 failure, `AllocatorWidenedSignatureTest`, is pre-existing at BASE (B3_ALIGNMENT
32 vs the test's alignment check) and untouched by this card.

## What this taught

1. The staging is bit-exact and safe, and it is trivial on AVX-512 (a few-percent,
   noise-level effect) because `permutex2var` is not the cost centre.
2. The whole point of this spike is the NEON de-swizzle (`vuzp1q_s32` +
   `vqtbl3q_u8`), which is far more expensive per byte than AVX-512's shuffle. That path
   cannot be measured on this Zen 4 Linux box; it needs an Apple-silicon bench.
3. The perf harness (`fixed3d-perf-harness`) is absent at BASE — no `tools/perf/`,
   `oracle_check.sh`, `ab.sh`, `cpu_info.sh` — so the full acceptance (7 interleaved
   repeats, per-phase tables, scalar-oracle hashes) could not run; measurement here is
   hand-rolled (raw.txt).

## What to try next

Measure the SAME patch on an Apple-silicon bench with `BOX3D_NEON=ON`. The NEON vertex
scan (`b3FindSupportVertexWide` in `src/hull.c`) currently pays `vuzp1q_s32` +
`vqtbl3q_u8` per group per face scan; the staged SoA scan (`b3FindSupportVertexLanesSoA`
NEON arm) loads the columns directly. If convex_pile shows ≥3% geomean with no scene
>2% slower, promote to KEEP.

## What not to repeat

Do not re-benchmark the AVX-512 side expecting a win — the de-swizzle is not the
bottleneck there. Do not add a hull-owned cache (the rejected spherical-matrix-cache
shape); the stage stays caller-owned and stack-scoped.
