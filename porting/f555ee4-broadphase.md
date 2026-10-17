# Port plan: box3d f555ee4 "Optimize Broad-Phase (#159)"

Upstream: erincatto/box3d `f555ee42084e0b43cbffa863f40bff8117c08896`, parent `954cf87`.
Base: `sprint/fixed3d-box3d-ports` at `820dbc7`. Branch: `port/f555ee4-broadphase`.
Model: `tla/BroadPhaseTree.tla` (merged as `5bfa76c`, PR #70) checks the moved bit,
the traversal and the sorted pair keys; this port is the C step it precedes.

## What f555ee4 changes

1. **Tree layout.** `b3TreeNode` becomes `{ aabb, flagIndex, height|shapeIndex }`. `flagIndex`
   packs the leaf bit (31), the moved bit (30) and a 30-bit index: the left child of an even
   sibling pair, or the proxy id of a leaf. The root is node 0, node 1 is the empty sentinel,
   siblings sit at `2k, 2k+1`. Parents move to a side array, category bits and user data to a
   `b3TreeProxy` array. Free nodes go back as pairs (`pairFreeList`), proxies through
   `proxyFreeList`. `b3_enlargedNode` and the node flags enum are gone.
2. **Rebuild.** The partial rebuild gathers unmoved subtrees as build leaves, builds into a spare
   array (`swapNodes`) in DFS order and swaps; `dfsOrdered` lets the refit sweep pairs from high
   index to low. The refit (`b3DynamicTree_Refit`) replaces per-proxy enlarge walks for the
   solver path; `b3DynamicTree_MarkProxyMoved` marks ancestors with an atomic fetch-or.
3. **Broad-phase.** The move buffer (`movedProxies` bit sets, `moveArray`, per-proxy query and
   linked pair lists) is removed. Pairs come from a tree-against-tree traversal: moved sibling
   pairs of the dynamic tree against themselves, and dynamic-against-static and
   dynamic-against-kinematic from breadth-first cross seeds. Each candidate is ordered
   `shapeIdA = min, shapeIdB = max`, culled against the pair set in batches of 32, filtered,
   pushed as a 64-bit pair key per worker, then all keys are concatenated, **sorted** and turned
   into contacts in key order. Duplicates are a validation trap and are skipped.
4. **Solver.** Finalize marks a proxy only when its fat AABB grew (`b3BroadPhase_MarkProxyMoved`),
   continuous collision does the same for non-bullets, the user tree task is joined before
   finalize, the kinematic and dynamic trees are refit, and bullets enlarge after the refit.
   `b3_enlargedAABB` and the enlarged body bit sets are gone.
5. **Small pieces.** `compoundShapeCount` on the world (only the static pass meets compounds),
   per-worker `pairKeys`, `b3KeyHash` in `table.h` and `b3ContainsHashedKey`, `B3_GROW`,
   `B3_GROW_ZERO`, `b3GrowAllocZeroed`, `b3AllocZeroed` renamed `b3AllocZero`, relaxed atomic
   load and fetch-or in `platform.h`, `b3DeclareArrayNative( uint64_t )`, snapshot format
   (tree fields, proxies, compound count; move buffer dropped), recording minor note.
6. **Tests.** `test_dynamic_tree.c` (brute force query, move/refit/rebuild, scrambled destroy,
   leaf root, category filter, casts, closest, save/load), `TestCompoundShapeCount`,
   `TestEnlargedProxyDestroyed` rewritten for move marks.

## Which parts touch determinism and goldens

- **The A/B swap.** Before, a contact's shape A was the shape the query found and B the moved
  query proxy; after, A is always the lower shape index. Manifold normals, impulses and solver
  order follow. This is why the three reference scenes fail against every float revision from
  f555ee4 on (collide-connected-false flips the sign of every z angular velocity).
- **Contact creation order** becomes the sorted key order, independent of tree shape and worker
  count. Worker splits only partition the traversal; the sort makes the result order free.
- **Tree structure** (insertion without rotation on move, pair layout, rebuild leaves) changes
  query callback order but not the sorted pair output.
- Expected golden moves: every determinism scene that creates contacts mid run (ragdoll, wave
  pile, ragdoll human, the query hash of query spawn). The goldens are re-harvested in their own
  commit after the code is in, on gcc and clang, Debug and RelWithDebInfo, narrow and ludicrous,
  workers 1 to 5, and that commit says which scene moved and why.

## How fixed point replaces the float operations

- `float` costs become `b3Fixed`; `FLT_MAX` becomes `B3_FIXED_MAX`; `b3MinFloat` becomes
  `b3FixMin`; the area ratio divides with `b3FixDiv`. All SAH arithmetic stays integer, as in
  the rest of the fork; the unconverted SAH heuristic stays fenced by its `#error`.
- **SIMD box helpers.** Upstream adds Neon and SSE2 float kernels (`b3LoadAABBV`, `b3OverlapNode`,
  `b3UnionPairV`, `b3StoreAABBV`, `b3UnionV`). fixed3d takes the scalar fallback only, routed
  through `b3AABB_Overlaps` and `b3AABB_Union` so the 128-bit ludicrous bounds compare portably.
  Integer compare and min/max are exact on every compiler and architecture.
- **Empty sentinel.** Upstream uses `+INF` lower and `-INF` upper. fixed3d uses the extreme
  values of the bound type, inverted: `B3_FIXED_MAX` / `B3_FIXED_MIN` narrow, the 128-bit extremes
  in ludicrous mode (a wide coordinate can pass the 64-bit range). The sentinel is only compared
  and unioned, never subtracted, measured or centred, so no arithmetic can overflow on it.
- **Bound crossings.** Ray and box cast segment boxes, node loads into `b3V32`, closest point
  clamps and the load scale go through `b3BoundToVec3` / `b3Vec3ToBound`, as the fork already
  does for the pre-f555ee4 tree.
- **Size checks.** Upstream validates `upper - lower < B3_HUGE`; fixed3d compares half extents to
  `B3_HUGE / 2`, which stays inside `b3Fixed` when the bounds are 128-bit.
- **Layout.** A node is the AABB plus two 32-bit words: 56 bytes narrow, 112 in ludicrous mode with
  an explicit zero `padding[2]` so no serialized byte is indeterminate. Proxies are 24 bytes.
- **Compound blob.** The fork's blob is pointer free (its own 5d83df8 harvest). The baked tree is
  stored dense (`nodeCount = nodeEnd = 2 * childCount`) with a proxy array at `proxyOffset`, which
  replaces the `root` field (the root is always node 0). The validator checks the paired layout:
  node 1 empty, no other empty node, no moved bit, leaf proxy back pointers, child pairs even and
  above the parent (so a traversal cannot cycle), heights decreasing. `B3_COMPOUND_VERSION`
  changes.
- **Snapshot** version 6 to 7 (efca0c1 took 6) with the upstream tree serialization and `sizeof( b3TreeProxy )` in
  the layout hash. The recording op stream is unchanged, so its counters stay.

## Carried with it

- **H-H (box3d 79fc559).** A full rebuild keeps move marks, so `b3World_RebuildStaticTree` no
  longer wipes pending static pairs. CLAUDE.md requires it to land in the same step as f555ee4.
  It brings `test_broad_phase.c` (`StaticRebuildKeepsMarks`, and the brute force pair set stress
  test, trimmed for the CI budget).

## Skipped or held

- `benchmark/*.csv`, `benchmark/main.c`, `shared/benchmarks.*`, `samples/sample_benchmark.cpp`,
  `samples/CMakeLists.txt`: benchmark data and sample app content, no simulation content.
- The Neon and SSE2 kernels: float lanes; the scalar integer form is the deterministic one.
- `samples/sample_tree.cpp` is ported because it must build against the new tree.

## Commits

1. this plan;
2. infrastructure with no behaviour change (allocation helpers, atomics, hash, arrays, compound
   census) and its test;
3. the tree data structure under the existing move buffer broad-phase, with `test_dynamic_tree.c`;
4. the broad-phase update and the solver marking, with H-H and `test_broad_phase.c`;
5. the golden re-harvest, alone, with its justification;
6. the PORT RECORD in CLAUDE.md.

## Gates

`./bin/test` with gcc (narrow and ludicrous) and clang, Debug and RelWithDebInfo, VALIDATE on;
cross-compiler determinism hashes identical; `ctest -L reference` against float box3d `f555ee4`
and `a2e4816`: box_stack, pendulum_chain and ref-954cf87-collide-connected-false pass at their
tolerances and no other scene regresses.
