# TLA+ models

Two bounded models live here: `BroadPhaseTree` (below) and `MoverJoints` (at the end).

## Broad-phase moved bits, traversal and sorted pair keys

This bounded model checks the tree traversal and update ordering for upstream
box3d [`f555ee4`](https://github.com/erincatto/box3d/tree/f555ee42084e0b43cbffa863f40bff8117c08896).
It precedes the fixed3d C port; it does not establish correctness of that port.

### Tree and traversal

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

### Phases and fresh move obligations

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

### Bounded checks and reproducibility

`MaxSteps` bounds mutation actions, not traversal or phase progress. Main and
ludicrous configurations use 4 mutations; the step configuration uses 3.
This includes four creations before one traversal and repeated moves with
intervening updates. The observed step counter saturates at `MaxSteps` so
phase progress remains enabled and final solver moves can still be consumed.
It is not a bound on the number of physical timesteps.

Run on a serialized Linux bench, never the working Mac:

```sh
/usr/bin/time -f 'wall_seconds=%e maxrss_kb=%M exit=%x' timeout 600 nice -n 10 \
  java -Xmx4g -XX:+UseParallelGC -jar /opt/tla/tla2tools.jar \
  -workers 4 -seed 1 -config BroadPhaseTree.cfg BroadPhaseTree
```

Repeat for `BroadPhaseTreeLudicrous.cfg`, `BroadPhaseTreeStep.cfg`, and
`BroadPhaseTreeMutant.cfg`. Measured results on bench-linux-b, 2026-10-15 UTC:

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

### Explicit limits

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

## Mover and pogo joints' carried impulse

`MoverJoints.tla` models the state the mover joint and the pogo joint (upstream box3d
[`3362633`](https://github.com/erincatto/box3d/tree/336263320469e5e9febfe18697ab562488f7e97a))
carry from step to step: the mover's accumulated planar impulse and its cap `h * maxForce`, and
the pogo's accumulated impulse, spring velocity, tension and compression caps and whether its
hertz is zero. It was written with the fixed3d port and checks the C spelling in
`src/mover_joint.c` and `src/pogo_joint.c`.

Actions: `BeginStep` (prepare and warm start: impulses zeroed when warm starting is off, a
zero-hertz pogo drops its impulse), `SolveMover` and `SolvePogo` (one solve or relax pass each, any
number per step, any unclamped impulse in the domain, then the clamp), `EndStep` (both joints
solved at least once), the setters between steps, `RebuildPogo` (the dynamic mover's per-frame
destroy and create, seeding any impulse and velocity from the def), `Sleep` / `Wake` (the solver
set copy keeps the state), `Snapshot` / `Restore`.

The mover's clamp is modelled in the code's own integer arithmetic at a reduced fraction width
`F` (layout, not only contract). `ClampForm = "exact"` is `b3ClampToDisc`: exact sum of the raw
squares, exact compare, ceiling square root, each component `|x| * cap / length` truncated toward
zero. `ClampForm = "rounded"` is the `src/joint.h` spelling the motor and parallel joints use
(`b3ImpulseOverCap2` rounding each square half up, `b3Length2`, `fixMulDiv`).

Invariants: `ImpulseWithinBoundAfterSolve` (after each joint's solve its impulse is inside the
bounds in force: the exact disc for the mover, `[-tension, compression]` for the pogo, and zero
impulse and speed for a zero-hertz pogo), `ImpulseWithinBoundAtRest` (between steps, once a step
has run since the last setter or rebuild, the carried state is inside the bounds, through sleep,
wake and snapshot), `RestoreExact` (a restore reproduces the snapshot exactly). The assumption
`ClampDivisorPositive` checks, for every cap and impulse, that the clamp's divisor is positive
and (exact form) at least each component, so the 128/64 divide's quotient fits.

Reversed witnesses, each a separate configuration that TLC must fail:

- `MoverJointsRounded.cfg` — the `joint.h` spelling. Fails `ImpulseWithinBoundAfterSolve` at depth
  3: cap 0, unclamped impulse `<<-1, -1>>` at F = 2, whose squares round to zero, so the compare
  reads it as inside. Checked against Q48.16 by hand and by `TestMoverJointZeroForce`: a raw cap
  of 0 or 137 (0.5 N at h = 1/240) passes impulses to 181 raw.
- `MoverJointsMutant.cfg` (`clampOnGrowth`) — the clamp only fires when the impulse grew. Fails
  `ImpulseWithinBoundAfterSolve` at depth 8: solve to `<<0, -1>>` under cap 1, end the step,
  `SetMoverMaxForce(0)`, solve again with the same unclamped impulse. This is the setter lowering
  the budget below the carried impulse; the C clamp does not look at the old impulse, and
  `TestMoverJointForceBudget` covers the case.
- `MoverJointsSnapshotMutant.cfg` (`restoreDropsImpulse`) — restore zeroes the impulses. Fails
  `RestoreExact` at depth 3 (a def-seeded pogo impulse is lost). `MoverPogoForkTest` covers the C.

Run on bench-mac-a (Apple M-series, 32 cores) with the same jar as the rows above:

```sh
nice -n 10 java -Xmx12g -XX:+UseParallelGC -jar tla2tools.jar -workers 8 -seed 1 -cleanup \
  -config MoverJoints.cfg MoverJoints
```

Results are in `RUNS.tsv`.

### Explicit limits

- Bounds are tiny: F = 2 with raw impulses in -4..4 and caps 0..3 for the main and mutant
  configurations; F = 1, -1..1 and 0..1 for the snapshot pair, whose state space squares with the
  snapshot copy. The arithmetic is the code's at that width; Q48.16 magnitudes are not explored.
- The time step is folded into the caps. `b3FixMul( h, maxForce )` itself, the velocity error that
  produces the unclamped impulse, the effective mass, and `b3SpringDamper` are not modelled; the
  pogo velocity is an abstract value in -1..1.
- Only the joint state is modelled: bodies, islands, the constraint graph and the snapshot byte
  format are not. The C side of sleep, wake and restore is covered by
  `TestPogoJointSleepKeepsImpulse` and `MoverPogoForkTest`.
- One mover and one pogo, solved in interleaved passes; multithreading is not modelled.
