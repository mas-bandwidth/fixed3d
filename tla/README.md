# Broad-phase tree: TLA+ model of the moved bit and sorted pair keys

Model for upstream box3d commit `f555ee4` "Optimize Broad-Phase" (PR #159),
written before the C port per the plan row "A TLA+ module for the tree's
moved-bit and pair-key invariants comes before the branch" and triage
`porting/triage-2026-10-09/T1.md` sections 1-6 (esp. section 7: "This is
the item that should be modelled in TLA+ before the code is written").

The two later port cards (node relayout; pair traversal) cite
`tla/BroadPhaseTree.tla` from their code.

## Files

| File | Role |
|---|---|
| `BroadPhaseTree.tla` | The model: proxies, fat boxes (as cells), moved bits, pair set, per-step actions |
| `BroadPhaseTree.cfg` | Sound instance: 4 proxies (2 static, 2 dynamic), 2 trees, `MaxSteps = 6`, narrow node (`NodeBytes = 72`) |
| `BroadPhaseTreeMutant.cfg` | Deliberate mutant: `DropClear = TRUE` drops the moved-bit clear in `UpdatePairs` |

## What is modelled

Variables are the state the tree and broad phase own: the live proxy set
and their fat boxes, the moved bits (leaf set plus the root mark the
atomic fetch-or propagates to), the move buffer's abstract effect (the
moved set `UpdatePairs` consumes), the emitted pair list and the live
pair set, and the per-step action sequence (`CreateProxy`,
`DestroyProxy`, `MoveProxy`, `EnlargeProxy`, `UpdatePairs`,
`RebuildStale`, `StepBoundary`).

Invariants (numbering follows the card):

1. `PairCompleteness` — every overlapping fat-box pair of live proxies
   with at least one moved proxy is found by `UpdatePairs`;
   `EmittedPairsAreOverlaps` is the soundness half, and
   `SortedDuplicateFree` gives exactly-once.
2. `NoDestroyedProxyInPairs` — no emitted or live pair involves a
   destroyed proxy.
3. `SortedDuplicateFree` — pair keys are totally ordered (`Key`), and
   the emitted list is strictly increasing, hence sorted and
   duplicate-free (the upstream merge + `QSORT` + duplicate trap).
4. `MovedClearedAfterUpdate` — the moved bit is cleared exactly at the
   end of `UpdatePairs`; `RootMarkConsistent` pins the root mark to
   "some leaf is marked", and every `MoveProxy`/`EnlargeProxy`/
   `CreateProxy` sets the bit.
5. `RebuildPreservesPairs` — a rebuild (stale-tree rebuild when the
   move buffer empties) loses no proxy and changes no pair.
6. `SentinelNeverOverlaps` — the resident empty node (upstream index 1,
   `B3_EMPTY_NODE`, built from `INFINITY`/`-INFINITY`) never reports an
   overlap and never appears in a pair. In Q48.16 there is no infinity,
   so the C port must keep the sentinel out of every
   Perimeter/Union/Overlap operand (triage section 7, CLAUDE.md trap
   4); the model's `NodeOverlap` is that guard stated once.

Liveness: `MovedEventuallyConsidered` — every proxy moved in a step is
considered in that step's `UpdatePairs`, unless it is destroyed first
(then its pairs are removed with it; there is nothing to consider).

Two node layouts, one behaviour: `NodeBytes` is 72 in the narrow build
and 128 in ludicrous mode (the current fixed3d node sizes; triage
section 2 works out the post-relayout sizes). The constant stands for
"some layout": no action reads `NodeBytes`. TLC was run at both
values; the distinct-state counts are identical, which is the
two-config claim at this abstraction level.

## TLC runs (Linux bench b1, never the working Mac)

Toolchain: OpenJDK 21, `tla2tools.jar` (TLC 2.19). Commands, run from
this directory after copying the three files to the bench:

```
java -XX:+UseParallelGC -jar tla2tools.jar -workers 4 -config BroadPhaseTree.cfg BroadPhaseTree
java -XX:+UseParallelGC -jar tla2tools.jar -workers 4 -config BroadPhaseTreeMutant.cfg BroadPhaseTree
# ludicrous equivalence: same cfg with NodeBytes = 128 (bench-local copy, not committed)
java -XX:+UseParallelGC -jar tla2tools.jar -workers 4 -config BroadPhaseTreeLudicrous.cfg BroadPhaseTree
```

Results at head (see the PR body for the exact head SHA fence):

- `BroadPhaseTree.cfg` (narrow, 72): **No error found.**
  2,111,116 states generated, 485,685 distinct states, depth 11.
- Ludicrous copy (128): **No error found**, identical distinct-state
  count (485,685) — the layout constant does not change behaviour.
- `BroadPhaseTreeMutant.cfg`: **fails as designed** —
  `Error: Invariant MovedClearedAfterUpdate is violated` within the
  first ~60 states generated (the exact count varies with the TLC
  seed; the invariant named does not). The trace is: create proxies,
  `UpdatePairs` runs, and the state after it still has `moved # {}`
  and `rootMoved = TRUE` because the clear was dropped.

Red first: at the unchanged base (no `tla/` directory), the first
command fails with `Error: ... ConfigFileException: ... File not
found` for `BroadPhaseTree.cfg`. That failing line is pasted in the
pull request.

## Counterexamples checked by hand against the code

Two TLC findings were checked against the C code before being written
down; both were model/spec corrections, not C bugs:

1. First liveness draft ("every moved proxy is eventually considered")
   produced a trace: create proxies 3 and 4, destroy both, then
   `UpdatePairs` — the destroyed proxies are never considered. That
   behaviour is reachable and correct in the C design (a destroyed
   proxy's pairs are removed on destroy; `b3UpdateBroadPhasePairs`
   never sees it), so the property now exempts proxies destroyed
   before the update. Model error, fixed in the model.
2. The mutant trace (moved bits still set after `UpdatePairs`) was
   verified reachable in the upstream code: deleting the body of
   `b3DynamicTree_ClearMoved` (upstream `src/dynamic_tree.c`) leaves
   `B3_MOVED_NODE` on the root, so `b3HasTreeMoved` stays true and
   the next `b3UpdateBroadPhasePairs` re-gathers and re-collides the
   same moved siblings. The model drops exactly that clear and TLC
   names exactly `MovedClearedAfterUpdate`.

No other counterexamples were found; the sound configuration is clean.

## What the model does NOT cover

- Geometry: fat boxes are cells in {0,1}; overlap is cell equality.
  AABB union/perimeter arithmetic, the SAH rotate choice, ray/box
  casts, and all Q48.16 rounding (including the sentinel-perimeter
  wrap itself — the model states the guard, it does not reproduce
  the arithmetic that makes the guard necessary) are outside it.
- Node layout mechanics: sibling pairing at even indices, the parent
  array, free lists, DFS reordering, and node indices are abstracted
  to one layout flag. Index-level properties (for example
  `nodeEnd >= 2`, root at index 0) are for the C tests, not this model.
- Concurrency: worker interleaving, the atomic fetch-or propagation
  order, parallel seed gathering, and batching in
  `b3FlushCandidatePairs` are abstracted away (OR commutes, and the
  emitted set is what is checked). No thread schedule is explored.
- Filtering: `b3ShouldCreatePair` (same-body, sensor, filter bits,
  joint and custom filters), compound child expansion
  (`b3EmitCompoundPairs`, including the held ludicrous demotion), and
  the pair-set/contact lifecycle beyond "pair is live or destroyed
  with its proxy" are not modelled.
- Scale: 4 proxies, 2 trees, 6 bounded actions. Larger populations,
  the kinematic tree, and rebuild quality (tree height, refit cost)
  are unchecked.
- Serialization: tree/proxy blob layout, snapshot/recording version
  bumps, and golden hashes are outside this model.
