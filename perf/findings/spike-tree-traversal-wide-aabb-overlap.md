# Spike: wide AABB overlap tests in the dynamic tree traversal

VERDICT: DROP — the wide compare is neutral-to-slower on the target scenes (no
geomean gain, no phase 1.5x), so the patch is not adopted. What it taught: the
per-node scalar loads into `_mm256_set_epi64x` and the movemask+branch tail cost
more than the six scalar comparisons they replace, and the AVX-512 solver itself
is slower than scalar on this Zen 4 part.

## Hypothesis

The broad-phase pair finder (`b3DynamicTree_Query`, called from
`b3FindPairsTask` in broad_phase.c) tests each child's fat AABB with six scalar
integer comparisons. Testing both children's AABBs in one wide compare (four
int64 lanes on AVX-512/AVX2) should cut the comparison count and improve ILP in
the tree-query-bound scenes (trees25/50/100, rain).

## The change (one paragraph)

Added `b3WideOverlap2(a1, a2, b)` in `src/dynamic_tree.c`, gated on
`B3_SIMD_AVX512 && !BOX3D_LUDICROUS_MODE`. It packs the two children's two
separations per axis into three `__m256i` (lane0/1 = child1, lane2/3 = child2),
subtracts, `_mm256_cmpgt_epi64` against zero, ORs the three axes, and reads a
2-bit overlap mask from `_mm256_movemask_epi8`. `b3DynamicTree_Query` pushes only
the overlapping children, in child1-then-child2 order so child2 still pops first
and the DFS visit order (and thus the callback sequence) is unchanged. Ludicrous
mode stays scalar (int128 bounds would scramble packed int64 loads). The unit
test `TreeQueryWideEquivalenceTest` (test/test_shape.c) compares the SEQUENCE of
proxy ids from the wide query against a scalar reference traversal over 20,000
seeded boxes and 1,000 seeded query boxes.

## Before / after

CPU: AMD Ryzen 7 PRO 8700GE (Zen 4), avx2/avx512f/avx512dq/avx512vl.
Compiler: gcc 13.3.0, Release + LTO (default), `-w=4 -t=4`, min of 5 runs, ms.

| scene    | narrow (scalar q+solver) | avx512 scalar q | avx512 wide q | wide vs scalar |
|----------|--------------------------|-----------------|---------------|----------------|
| trees25  | 533.03                   | 596.83          | 624.89        | +4.7% (noisy)  |
| trees50  | 263.22                   | 332.60          | 336.26        | +1.1%          |
| trees100 | 180.90                   | 264.79          | 267.75        | +1.1%          |
| rain     | 1949.17                  | 1946.34         | 1946.34       | 0.0%           |

The wide compare is isolated by the avx512-scalar vs avx512-wide columns (same
AVX-512 solver, only the query differs). It is neutral-to-slower everywhere. The
larger gap (narrow vs avx512) is the AVX-512 solver, which is itself slower than
scalar on this 8700GE — a separate, pre-existing observation, not this spike's
change.

## Bit-exact evidence

`TreeQueryWideEquivalenceTest` passes in all three builds (narrow scalar,
narrow+AVX512 wide, ludicrous): the wide query returns the identical SEQUENCE of
proxy ids as the scalar reference. Red-first: swapping the overlap mask
(overlap1<->overlap2) makes the subtest fail, proving it discriminates visit
order. Full `./bin/test` green in narrow (2.70s) and ludicrous (2.72s); the
determinism goldens in test_determinism.c are unchanged. `tools/perf/` (the
fixed3d-perf-harness) is absent at this BASE, so `oracle_check.sh`/`ab.sh` could
not run; the determinism goldens are the bit-exactness evidence.

Caveat: the wide path changes `b3TreeStats.nodeVisits` (non-overlapping children
are no longer popped/counted). The broad phase ignores the return value, and the
simulation hash is unaffected, but a recording of a world query API
(OverlapAABB/OverlapShape/CastRay/CastShape) would record different nodeVisits.
A production adoption would need to preserve nodeVisits or gate the wide path to
the broad phase only.

## What to try next

- A SoA node layout (lower/upper bounds as separate arrays) so the wide compare
  can use contiguous vector loads instead of six scalar loads per axis into
  `_mm256_set_epi64x`; the scalar-load gather is the likely reason the wide
  compare does not pay.
- Test four children at once (two internal nodes) to amortize the movemask+branch
  tail, or a branchless traversal that pushes both children unconditionally and
  masks the overlap test.
- Re-measure the AVX-512 solver on this 8700GE: it is ~+46% on trees100 vs
  scalar, which contradicts the published EPYC 9124 numbers and is worth its own
  spike before any tree-query work is judged on an AVX-512 build.

## What not to repeat

- The wide compare with per-node scalar loads into `_mm256_set_epi64x` (this
  experiment): neutral-to-slower, measured above.
- Judging a tree-query change on an AVX-512 build without first isolating the
  AVX-512 solver's own regression on this part.
