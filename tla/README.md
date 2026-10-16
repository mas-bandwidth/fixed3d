# Broad-phase moved bits, traversal and sorted pair keys

This bounded model checks the tree traversal and update ordering for upstream
box3d [`f555ee4`](https://github.com/erincatto/box3d/tree/f555ee42084e0b43cbffa863f40bff8117c08896).
It precedes the fixed3d C port; it does not establish correctness of that port.

## Tree and traversal

Four proxy slots belong to two binary trees: static leaf 1 and an empty sentinel
under root 5, and dynamic leaf 2/subtree 7 under root 6; subtree 7 holds leaves 3/4. Absent proxies have empty boxes. Node 0
is the empty sentinel. `children` and `parent` are explicit relations;
`nodeMoved` records the moved bit of each leaf and ancestor. Rebuild exchanges
child order and reconstructs the parent relation without changing membership.
`TreeConsistent` and `RootMarkConsistent` check those relations and marks.

`BeginUpdate` seeds a node-pair work sequence from each marked dynamic internal node's
sibling subtrees and the dynamic/static root pair. This abstracts the moved
internal-node self seeds and cross-root gather at `broad_phase.c:515`.
`VisitPair` pops one item, prunes on node marks and box overlap, expands
internal pairs into child work items, and appends leaf pairs to a candidate
**sequence**. It follows the cross-subtree traversal at lines 367–461.
The asymmetric trees exercise internal/internal, leaf/internal, internal/leaf
and leaf/leaf traversal, including ancestor propagation through node 7. Other
shapes and greater depths are not covered by these configurations.

`CandidateDuplicateFree` checks raw multiplicity, before deduplication can hide
a bug. `UpdatePairs` deduplicates and sorts candidate keys, corresponding to
the upstream duplicate validation/skip at lines 752–762 (upstream sorts before
skipping adjacent duplicates; this model uses a set then sorts). Pair completeness
compares this traversal result with `ExpectedPairs`, a separate all-live-leaf
oracle. No action calls the oracle. `SortedDuplicateFree` checks the final order;
`EmittedPairsAreOverlaps` and `NoDestroyedProxyInPairs` check soundness/lifetime.

## Phases and fresh move obligations

`collect -> update -> rebuild -> solver -> collect` is explicit. Mutations may
occur during collection. Dynamic moves/enlargements may also occur during solver
finalization, after rebuild joins; those marks carry into the next update.
Creation/destruction occurs only in collection. Every create/move/enlarge removes
that proxy from `considered`, including a move to the same cell. Consequently an
old consideration cannot discharge a fresh obligation. Destruction cancels it.
Weak fairness on each phase/traversal action establishes
`MovedEventuallyConsidered` even after the mutation budget is exhausted.

`NoBoundaryWithPendingMove` checks the transition **into solver finalization**:
marks have been cleared and the update's input moves have been considered.
It deliberately permits solver-produced marks at the end-of-step boundary.
This matches `physics_world.c:1093` updating pairs before `b3Solve` at 1141,
and `solver.c:1884–1894` joining rebuild and validating clear marks before
finalization, which marks proxies at lines 598, 634 and 849.

The static tree clears in `UpdatePairs` (`broad_phase.c:701`);
the queued dynamic-tree `RebuildStale` clears dynamic marks when it joins
(`dynamic_tree.c:1926–1960`). `MovedClearedAfterUpdate` is checked after that
join, not immediately after the pair-emission action. `RebuildPreservesPairs`
checks that rebuild preserves live proxies and the contact-candidate set.

`DropClear` is a deliberate **combined abstraction mutant**: it retains marks
in both the static clear and the dynamic rebuild. It must violate
`MovedClearedAfterUpdate`. Removing `b3DynamicTree_ClearMoved` alone is only
the static-tree analogue: dynamic and kinematic marks clear in rebuild,
queued at `broad_phase.c:707`. The mutant is not a claim that deleting that
one C function leaves every tree marked, nor evidence of an upstream C bug.

## Bounded checks and reproducibility

`MaxSteps` bounds mutation actions, not traversal or phase progress. Main and
ludicrous configurations use 4 mutations; the step configuration uses 3.
This includes four creations before one traversal and repeated moves with
intervening updates. The observed step counter saturates at `MaxSteps` so
phase progress remains enabled and final solver moves can still be consumed.
It is not a bound on the number of physical timesteps.

Run on a serialized Linux bench, never the working Mac:

```sh
/usr/bin/time -f 'wall_seconds=%e maxrss_kb=%M exit=%x' timeout 600 nice -n 10 \
  /home/nova/sdk/bin/java -Xmx4g -XX:+UseParallelGC -jar /opt/tla/tla2tools.jar \
  -workers 4 -seed 1 -config BroadPhaseTree.cfg BroadPhaseTree
```

Repeat for `BroadPhaseTreeLudicrous.cfg`, `BroadPhaseTreeStep.cfg`, and
`BroadPhaseTreeMutant.cfg`. Measured results on Hetzner2, 2026-10-15 UTC:

| Configuration | Exit | Outcome | Generated | Distinct | Depth | Wall seconds |
|---|---:|---|---:|---:|---:|---:|
| BroadPhaseTree.cfg | 0 | no error | 380083 | 192274 | 54 | 11.09 |
| BroadPhaseTreeLudicrous.cfg | 0 | no error | 380083 | 192274 | 54 | 11.45 |
| BroadPhaseTreeStep.cfg | 0 | no error | 48757 | 30228 | 41 | 2.30 |
| BroadPhaseTreeMutant.cfg | 12 | MovedClearedAfterUpdate | 1852 | 454 | 6 | 0.82 |

All four rows use TLC **2.19 (5a47802)**, Java **Temurin 21.0.12.1+1**,
and jar SHA256
`936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88`.
`RUNS.tsv` records this provenance on every row, including UTC run start.
`tlacheck` was absent from the bench PATH; records were extracted directly
from TLC and GNU time output. No cfg exceeded twelve seconds.
The 72/128 counts are identical because `NodeBytes` is unused.

Red first on unchanged `bf95e634` plus the review's property/config:
`Error: Action property NoBoundaryWithPendingMove is violated.` Exit 13;
64 generated, 52 distinct, depth 4, wall 0.61 seconds. The failing trace is
Init -> CreateProxy(2,0) -> StepBoundary, with moved={2}, considered={}.
Four-worker discovery counts and proxy choice can vary.

The uncommitted throwaway traversal mutant drops the second child push from
the `leafA` branch of `VisitPair`. It produced
`Error: Invariant PairCompleteness is violated.` Exit 12; 1,179 generated,
670 distinct, depth 9, wall 0.88 seconds. This demonstrates that the oracle
detects missing traversal output; the broken line is not in this module.

## Explicit limits

- Geometry is two cells and set intersection for node boxes. AABB union,
  perimeter/SAH arithmetic, ray casts and Q48.16 rounding are outside the model.
  Sentinel boxes are empty; this does not reproduce fixed-point overflow or
  prove that the C implementation guards every sentinel arithmetic operand.
- Only two fixed asymmetric binary trees of maximum depth two are explored. Empty slots stay in place;
  insertion/removal allocation, rotations, varying shapes and deeper ancestors
  are not modelled. Rebuild checks a child permutation, not the rebuild algorithm.
- Node indices/links are modelled, but physical sibling alignment, free lists,
  DFS memory layout and byte offsets are not. `NodeBytes` is unused: equal counts
  at 72 and 128 show only that the abstract behaviour ignores node size.
- Threading, atomic fetch-or interleavings, worker scheduling, seed batching,
  and bounded C stack overflow are not explored. Traversal is serial.
- Kinematic trees, filters, same-body/sensor/joint/custom exclusions, compound
  expansion, and contact lifetime beyond destruction are not modelled.
- Larger proxy populations, rebuild quality, serialization, snapshot versions
  and golden hashes are unchecked. These finite checks are not a general proof.
