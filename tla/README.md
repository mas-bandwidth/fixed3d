# BroadPhaseTree

Abstract state machine for the broad phase in upstream box3d
`f555ee42084e0b43cbffa863f40bff8117c08896` (Optimize Broad-Phase, #159).
This is the state a fixed3d port of that commit has to keep. It is not a C
port and not a C test. No floats.

Attempt 2 held because the card's literal rules are not true of that commit.
This model uses the three corrected predicates.

1. Emission. `b3FlushCandidatePairs` (`src/broad_phase.c`) drops a key already
   in `pairSet`, then calls `b3ShouldCreatePair`. Not every overlapping moved
   pair is emitted. The invariant is `NewEligibleEmittedOnce`: every new
   eligible overlapping candidate that survives the existing-contact test and
   the shape/body filter is emitted once, as one sorted duplicate-free list.
2. Moved bits. `b3UpdateBroadPhasePairs` calls `b3DynamicTree_ClearMoved` on
   the static tree only (`broad_phase.c` line 701). Dynamic and kinematic
   marks clear inside `b3DynamicTree_Rebuild` (`dynamic_tree.c` lines 1932
   and 1957, then `ValidateNoMoved`), which `b3EnqueueTreeUpdate` schedules
   before the next consuming step. `MovedBitsClearedAsSpecified` does not
   require those marks to be clear at the end of `UpdatePairs`. Kinematic is
   not a third tree here; it would follow the dynamic rule.
3. Liveness. A proxy moved and then destroyed before `UpdatePairs` is not
   considered. `LiveMovedConsidered` obliges only proxies still live at
   update time. A moved static proxy that overlaps no live dynamic is not
   visited: the search is seeded from the dynamic tree, and an empty dynamic
   root contributes no seeds. That state is reachable in the C code, so it
   is not a violation here.

`ClearMoved` is TRUE in the real configs. `BroadPhaseTreeMutant.cfg` sets it
FALSE and drops both clears. `MovedBitsClearedAsSpecified` does not mention
the constant, so the mutant fails it.

## Instance

Three proxies, two trees (`Static`, `Dynamic`), at most 8 actions
(`MaxActions = 8`, inside the card's 6..8). At most 3 mutations
(`CreateProxy`, `DestroyProxy`, `MoveProxy`, `EnlargeProxy`) between step
boundaries. `UpdatePairs`, `RebuildStale`, and `StepBoundary` count toward
the 8 and do not count toward the burst.

`P1` and `P2` share a body, so `{P1,P2}` fails the filter. `ShapeAllows` is
the shape gauntlet and is true for every pair in this instance. `Sensor` is
false for all three. The keys that can be emitted are `<<P1,P3>>` and
`<<P2,P3>>`, in that rank order. Create always marks (`forcePairCreation`
TRUE). Fat boxes are the three intervals `<<0,0>>`, `<<1,1>>`, and
`<<0,1>>`.

`moveBuffer` is the set of live proxies whose leaf moved bit is set. Upstream
deleted the move array; the bit replaced it (T1 §1). `layout` is `Narrow` or
`Ludicrous` and no action reads it.

Insert pairs a new leaf with the old root. Three proxies on that spine have
height at most 2, so a box or moved-bit edit updates the leaf, its parent,
and its grandparent. Rebuild clears dynamic marks and does not reparent.

## TLC

TLC 2.19 (rev 5a47802), Eclipse Adoptium 21, on `ubuntu@space`, cwd
`~/scratch/bpt-f555ee4`. BFS. Deadlock checking left on (no `-deadlock`
flag). `Done` is a stutter once `taken >= MaxActions`, so a state at the
cap has a successor.

Both layouts, real clears, 2026-10-14 14:58:05–14:58:09, exit 0:

```
java -XX:+UseParallelGC -Xmx6g -Djava.io.tmpdir=$HOME/scratch/bpt-f555ee4/tmp \
  -cp /opt/tla/tla2tools.jar tlc2.TLC -workers 4 \
  -metadir $HOME/scratch/bpt-f555ee4/main \
  -config BroadPhaseTree.cfg BroadPhaseTree.tla
```

346028 states generated, 169338 distinct, depth 9, 0 left on the queue.
No error. Fingerprint collision estimate 1.6E-9 (optimistic), 7.5E-10
(actual). Seed -676402487453198982, fp 70.

Narrow only (`BroadPhaseTreeNarrow.cfg`, workers 2, `-Xmx4g`, metadir
`~/scratch/bpt-f555ee4/narrow`), 2026-10-14 15:00:06–15:00:09, exit 0:
173014 generated, 84669 distinct, depth 9. Seed 2970732491737620781.

Ludicrous only (`BroadPhaseTreeLudicrous.cfg`, workers 2, `-Xmx4g`, metadir
`~/scratch/bpt-f555ee4/ludicrous`), 2026-10-14 15:00:10–15:00:13, exit 0:
173014 generated, 84669 distinct, depth 9. Seed -4750293552910417453.

84669 + 84669 = 169338. The two layouts are the same machine. The label is
not an input to any action.

## Mutant

`BroadPhaseTreeMutant.cfg` is the same instance with `ClearMoved = FALSE`.
TLC 2.19, workers 2, `-Xmx4g`, metadir `~/scratch/bpt-f555ee4/mut`,
2026-10-14 14:57:32, exit 12. Seed 238578412623148494.

`MovedBitsClearedAsSpecified` is violated. 28 states generated, 28 distinct,
depth 3, 22 left on the queue when the search stopped. Shortest trace, layout
`Narrow`:

1. Init. `phase = mutate`, `moveBuffer = {}`.
2. `CreateProxy` `P1` on `Static`, box `<<0,0>>`, leaf moved, `moveBuffer = {P1}`.
3. `UpdatePairs`. `phase = bound` (no dynamic proxy was marked, so no rebuild
   is scheduled). `moveBuffer` is still `{P1}`. Static leaf 1 is still
   `moved = TRUE`. `emitted = <<>>`, `considered = {}`, `pairSet = {}`.

Hand check against `f555ee42084e0b43cbffa863f40bff8117c08896`. Create of a
static proxy with `forcePairCreation` TRUE sets the moved bit. The following
`b3UpdateBroadPhasePairs` sees the static tree moved, finds no pairs (the
dynamic tree is empty, so there are no cross seeds; `considered = {}` matches
that), then calls `b3DynamicTree_ClearMoved` on the static tree
(`broad_phase.c` line 701). That clear starts at the root and unsets the flag
(`dynamic_tree.c` `b3DynamicTree_ClearMoved`, line 2046). The state in step 3
is what that function returns if line 701 is omitted. The C function does not
omit it. This is the mutant, not a state the C code reaches, and not a model
error.

## What an invariant does not claim

The naive temporal sentence "every moved proxy is eventually considered" was
checked by hand and is not a `PROPERTY`. It is false of this C code for three
independent reasons: a moved static proxy that overlaps no live dynamic is
never visited; destruction before `UpdatePairs` discharges the obligation
(`b3ResetProxy` destroys the shape's contacts, then
`b3BroadPhase_DestroyProxy`); a finite action bound would falsify leads-to
even if the first two did not. The safety form is `LiveMovedConsidered`.

## What this model does not cover

- Numeric AABBs, Q48.16 overflow, perimeter, SAH rotate, and binning.
- Byte layout. Upstream's node is 32 bytes. Fixed3d's is 72 narrow and 128
  under ludicrous mode (T1 §§1–3). `Narrow` and `Ludicrous` are labels.
  Sibling pairs at even indices, the parent array, height, `shapeIndex`
  packing, root at slot 0, and the sentinel fixed at slot 1 are not in the
  state. Each tree has one explicit empty node (`kind = "empty"`,
  `box = "none"`). It is the root exactly when the tree is empty.
  `NodeOverlaps` is false for it. Upstream's `b3MakeEmptyNode` uses
  `±INFINITY` (T1, the sentinel section). Q48.16 has no infinity, and a huge
  box wraps. The model does not encode a huge box.
- Kinematic as its own tree. `b3UpdateTreesTask` rebuilds dynamic and
  kinematic. A rebuild whose root is not moved and whose tree is
  `dfsOrdered` returns 0 (`dynamic_tree.c` line 1878). The model schedules
  `RebuildStale` only when a dynamic proxy is in the buffer. After a
  static-only update the C rebuild is that early return, and the end state
  matches. There is no separate no-op rebuild action.
- Compounds and child-index keys. `b3EmitCompoundPairs` is not in the
  emission rule.
- Sensors, joint overrides, and custom filters as populated data.
  `ShapeAllows` is true for every pair here. The filter this instance
  actually rejects with is same-body on `{P1,P2}`.
- `forcePairCreation` FALSE. Every create marks.
- Worker parallelism, the candidate batch, and merge order. The model emits
  the sorted set. Upstream asserts a duplicate key is a bug and skips it
  (`broad_phase.c` lines 754–761). A set cannot hold that duplicate.
- The BFS / rebuild stack overflowing.
- Rebuild reshaping. SAH and DFS reindexing rewrite parent links in C. Here
  the topology stays the insertion spine. `RebuildStale` does not drop a
  proxy and does not change `pairSet`. It does not claim the post-rebuild
  parent array.
- The contact solver, the narrow phase, and bullets.
- Any fat box outside the three intervals. On this set a union-overlap is a
  leaf-overlap, so the leaf-pair rule matches a descent. That is not a proof
  for continuous AABBs.
- More than 3 proxies, more than 8 actions, or more than 3 mutations between
  step boundaries.
- Atomic mark races. Marks are sequential.
- Pair removal for any reason other than fat-box separation at
  `StepBoundary`, or destruction of a proxy. Separation is not done inside
  `UpdatePairs`; a separated key can sit in `pairSet` until the step
  boundary. That matches the world, which drops disjoint contacts in the
  step rather than in `b3UpdateBroadPhasePairs`.
- Bare `b3BroadPhase_DestroyProxy` (`broad_phase.c` lines 65–72), which
  removes the tree leaf and does not touch `pairSet`. `DestroyProxy` here is
  the world path: destroy the contacts (drop the keys), then the proxy
  (`b3ResetProxy` in `src/shape.c`).

## Actions and variables

Variables: `proxies` (alive, tree, fat box), `nodes` (kind, moved bit, parent,
children, proxy, box) and `root` per tree, `moveBuffer`, `pairSet`, `emitted`,
`layout`, the per-step `stepSeq`, plus the bookkeeping `phase`, `taken`,
`burst`, `pre`, `considered`.

Actions: `CreateProxy`, `DestroyProxy`, `MoveProxy`, `EnlargeProxy`,
`UpdatePairs`, `RebuildStale`, `StepBoundary`, and `Done`.
