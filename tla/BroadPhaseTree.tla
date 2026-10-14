--------------------------- MODULE BroadPhaseTree ---------------------------
(***************************************************************************)
(* Abstract model of the broad-phase dynamic tree after upstream commit  *)
(* f555ee4 "Optimize Broad-Phase" (box3d PR #159): node relayout with a   *)
(* moved bit (B3_MOVED_NODE, bit 30 of flagIndex), tree-against-tree      *)
(* pair traversal (Ericson 6.3.2 cross-subtree descent), 64-bit pair keys *)
(* merged and sorted before contact creation, an always-resident empty    *)
(* sentinel node at index 1, and stale-tree rebuild when the move buffer  *)
(* empties.                                                               *)
(*                                                                        *)
(* The model is written BEFORE the C port. The two later port cards (node *)
(* relayout, pair traversal) cite this module from their code.            *)
(*                                                                        *)
(* Abstraction choices (see README.md for the full list of what is NOT    *)
(* modelled):                                                             *)
(*  - A fat box is abstracted to a cell in {0,1}; two proxies overlap iff *)
(*    they share a cell. Only the overlap RELATION matters to the pair    *)
(*    invariants, not box geometry.                                       *)
(*  - The sibling-pair node layout, parent array and DFS order are       *)
(*    abstracted to a single layout flag flipped by RebuildStale. The     *)
(*    invariants quantify over proxies and pairs, never node indices,     *)
(*    which is exactly the property the relayout must preserve.           *)
(*  - Ancestor propagation of the moved bit (atomic fetch-or up to the    *)
(*    root) is abstracted to one root flag kept in sync with the set of   *)
(*    moved leaves: rootMoved <=> moved # {}. Atomicity and worker       *)
(*    interleaving are not modelled; OR is commutative, so the abstract   *)
(*    result is order-independent (triage T1.md section 5).               *)
(*  - Node size (72 bytes narrow, 128 bytes ludicrous in fixed3d;         *)
(*    triage T1.md section 2) is the constant NodeBytes. No action reads  *)
(*    it. TLC is run at both values and must produce the same state       *)
(*    count: the two layouts have the same abstract behaviour.            *)
(*  - DropClear is the deliberate mutant switch. With DropClear = TRUE,   *)
(*    UpdatePairs forgets to clear the moved bits (the equivalent of      *)
(*    deleting the body of b3DynamicTree_ClearMoved in the upstream       *)
(*    code). TLC must then violate MovedClearedAfterUpdate.               *)
(***************************************************************************)
EXTENDS Integers, Sequences, FiniteSets, TLC

CONSTANTS
    MaxSteps,   \* bound on non-UpdatePairs actions, keeps TLC finite
    DropClear,  \* BOOLEAN: mutant switch, TRUE drops the moved-bit clear
    NodeBytes   \* 72 (narrow) or 128 (ludicrous); never read by any action

\* Four proxies: 1,2 live in the static tree, 3,4 in the dynamic tree.
Proxies   == 1..4
StaticP   == {1, 2}
DynP      == {3, 4}
Cells     == 0..1
Sentinel  == 0   \* the empty node; it has no cell and no box

\* 64-bit pair keys upstream are (shapeIdA, shapeIdB, childIndex) packed
\* high-to-low; with childIndex = 0 here, Key is a total order on pairs.
Key(a, b) == a * 10 + b

UnorderedPair(a, b) == IF a < b THEN <<a, b>> ELSE <<b, a>>

\* Overlap of tree NODES. The sentinel (index 0, upstream node index 1
\* with flagIndex = B3_EMPTY_NODE) never overlaps anything: upstream
\* builds it from INFINITY / -INFINITY so IEEE arithmetic saturates,
\* but Q48.16 has no infinity (triage T1.md section 7, CLAUDE.md trap 4).
\* The fixed3d port must therefore keep the sentinel out of every
\* Perimeter / Union / Overlap operand; this definition is that guard.
NodeOverlap(pos, a, b) ==
    IF a = Sentinel \/ b = Sentinel THEN FALSE
    ELSE pos[a] = pos[b]

\* Pairs the traversal may emit: dynamic against itself or against the
\* static tree. Static-static pairs are never generated upstream
\* (b3UpdateBroadPhasePairs collides the dynamic tree with itself and
\* gathers dynamic-vs-static / dynamic-vs-kinematic cross seeds only).
AllowedPair(a, b) == a \in DynP \/ b \in DynP

ExpectedPairs(aliveSet, pos, movedSet) ==
    { p \in { UnorderedPair(a, b) : a, b \in aliveSet } :
        /\ p[1] # p[2]
        /\ AllowedPair(p[1], p[2])
        /\ NodeOverlap(pos, p[1], p[2])
        /\ (p[1] \in movedSet \/ p[2] \in movedSet) }

RECURSIVE SortPairs(_)
SortPairs(S) ==
    IF S = {} THEN << >>
    ELSE LET m == CHOOSE p \in S : \A q \in S : Key(p[1], p[2]) <= Key(q[1], q[2])
         IN  <<m>> \o SortPairs(S \ {m})

SeqKeys(s) == [ i \in 1..Len(s) |-> Key(s[i][1], s[i][2]) ]

StrictlySorted(s) ==
    \A i \in 1..(Len(s) - 1) : SeqKeys(s)[i] < SeqKeys(s)[i + 1]

VARIABLES
    alive,          \* set of live proxies
    pos,            \* fat-box cell per proxy: Proxies -> Cells
    moved,          \* proxies whose leaf moved bit is set this step
    rootMoved,      \* moved bit on the root (propagated mark)
    emitted,        \* pair list emitted by the last UpdatePairs (sorted)
    emittedFor,     \* moved set the last UpdatePairs was computed from
    considered,     \* proxies considered by the last UpdatePairs
    pairSet,        \* live contact-candidate pairs (no destroyed proxy)
    layout,         \* 0/1 abstract node layout, flipped by RebuildStale
    justUpdated,    \* TRUE iff the previous action was UpdatePairs
    justRebuilt,    \* TRUE iff the previous action was RebuildStale
    aliveAtRebuild,
    pairsAtRebuild,
    step,           \* step counter (Step boundary increments it)
    ticks           \* action budget for TLC finiteness

vars == << alive, pos, moved, rootMoved, emitted, emittedFor, considered,
           pairSet, layout, justUpdated, justRebuilt, aliveAtRebuild,
           pairsAtRebuild, step, ticks >>

Init ==
    /\ alive = {}
    /\ pos = [ p \in Proxies |-> 0 ]
    /\ moved = {}
    /\ rootMoved = FALSE
    /\ emitted = << >>
    /\ emittedFor = {}
    /\ considered = {}
    /\ pairSet = {}
    /\ layout = 0
    /\ justUpdated = FALSE
    /\ justRebuilt = FALSE
    /\ aliveAtRebuild = {}
    /\ pairsAtRebuild = {}
    /\ step = 0
    /\ ticks = 0

CreateProxy(p, c) ==
    /\ p \in Proxies \ alive
    /\ ticks < MaxSteps
    /\ alive' = alive \cup {p}
    /\ pos' = [pos EXCEPT ![p] = c]
    /\ moved' = moved \cup {p}       \* creation marks the proxy moved
    /\ rootMoved' = TRUE
    /\ justUpdated' = FALSE
    /\ ticks' = ticks + 1
    /\ justRebuilt' = FALSE
    /\ UNCHANGED << emitted, emittedFor, considered, pairSet, layout,
                    aliveAtRebuild, pairsAtRebuild, step >>

DestroyProxy(p) ==
    /\ p \in alive
    /\ ticks < MaxSteps
    /\ alive' = alive \ {p}
    /\ moved' = moved \ {p}
    /\ rootMoved' = (moved' # {})
    /\ pairSet' = { q \in pairSet : q[1] # p /\ q[2] # p }
    /\ emitted' = << >>
    /\ emittedFor' = {}
    /\ justUpdated' = FALSE
    /\ ticks' = ticks + 1
    /\ justRebuilt' = FALSE
    /\ UNCHANGED << pos, considered, layout,
                    aliveAtRebuild, pairsAtRebuild, step >>

MoveProxy(p, c) ==
    /\ p \in alive
    /\ ticks < MaxSteps
    /\ pos' = [pos EXCEPT ![p] = c]
    /\ moved' = moved \cup {p}       \* every Move sets the moved bit
    /\ rootMoved' = TRUE
    /\ justUpdated' = FALSE
    /\ ticks' = ticks + 1
    /\ justRebuilt' = FALSE
    /\ UNCHANGED << alive, emitted, emittedFor, considered, pairSet,
                    layout, aliveAtRebuild, pairsAtRebuild,
                    step >>

EnlargeProxy(p) ==
    /\ p \in alive
    /\ ticks < MaxSteps
    /\ moved' = moved \cup {p}       \* every Enlarge sets the moved bit
    /\ rootMoved' = TRUE
    /\ justUpdated' = FALSE
    /\ ticks' = ticks + 1
    /\ justRebuilt' = FALSE
    /\ UNCHANGED << alive, pos, emitted, emittedFor, considered, pairSet,
                    layout, aliveAtRebuild, pairsAtRebuild,
                    step >>

\* Tree-against-tree traversal + merge + sort of 64-bit pair keys +
\* contact creation, as one atomic step (worker parallelism and the
\* batching in b3FlushCandidatePairs do not change the emitted set:
\* keys are sorted and duplicates trapped upstream, so the sorted list
\* is the observable). The moved bits are cleared exactly here, at the
\* end of UpdatePairs (b3DynamicTree_ClearMoved) -- unless the mutant
\* switch DropClear drops that clear.
UpdatePairs ==
    LET expected == ExpectedPairs(alive, pos, moved) IN
    /\ emitted' = SortPairs(expected)
    /\ emittedFor' = moved
    /\ considered' = moved
    /\ pairSet' = pairSet \cup expected
    /\ IF DropClear
         THEN /\ moved' = moved          \* MUTANT: clear forgotten
              /\ rootMoved' = rootMoved  \* MUTANT: root stays marked
         ELSE /\ moved' = {}
              /\ rootMoved' = FALSE
    /\ justUpdated' = TRUE
    /\ justRebuilt' = FALSE
    /\ UNCHANGED << alive, pos, layout, aliveAtRebuild, pairsAtRebuild,
                    step, ticks >>

\* Rebuild of stale trees when the move buffer empties (upstream
\* b3DynamicTree_Rebuild via b3EnqueueTreeUpdate; the 954cf87 step-2
\* rule named in the card). Rebuild re-lays out nodes (DFS order,
\* sibling pairs) but must never lose a proxy or change the pair set.
RebuildStale ==
    /\ moved = {}                    \* move buffer empty
    /\ ticks < MaxSteps
    /\ layout' = 1 - layout
    /\ aliveAtRebuild' = alive
    /\ pairsAtRebuild' = pairSet
    /\ justRebuilt' = TRUE
    /\ ticks' = ticks + 1
    /\ UNCHANGED << alive, pos, moved, rootMoved, emitted, emittedFor,
                    considered, pairSet, justUpdated, step >>

StepBoundary ==
    /\ ticks < MaxSteps
    /\ step' = step + 1
    /\ justUpdated' = FALSE
    /\ justRebuilt' = FALSE
    /\ ticks' = ticks + 1
    /\ UNCHANGED << alive, pos, moved, rootMoved, emitted, emittedFor,
                    considered, pairSet, layout,
                    aliveAtRebuild, pairsAtRebuild >>

Next ==
    \/ \E p \in Proxies, c \in Cells : CreateProxy(p, c)
    \/ \E p \in Proxies : DestroyProxy(p)
    \/ \E p \in Proxies, c \in Cells : MoveProxy(p, c)
    \/ \E p \in Proxies : EnlargeProxy(p)
    \/ UpdatePairs
    \/ RebuildStale
    \/ StepBoundary

Spec == Init /\ [][Next]_vars /\ WF_vars(UpdatePairs)

-----------------------------------------------------------------------------
(* Invariants: the six rules from the card, plus typing. *)

TypeOK ==
    /\ alive \subseteq Proxies
    /\ pos \in [Proxies -> Cells]
    /\ moved \subseteq alive
    /\ rootMoved \in BOOLEAN
    /\ Len(emitted) <= 6
    /\ \A i \in 1..Len(emitted) : emitted[i] \in (Proxies \X Proxies)
    /\ pairSet \subseteq { UnorderedPair(a, b) : a, b \in Proxies }
    /\ layout \in 0..1
    /\ step \in Nat /\ ticks \in 0..MaxSteps

\* Ancestor propagation, abstracted: the root is marked iff some leaf is.
RootMarkConsistent == rootMoved = (moved # {})

\* (1) Every overlapping fat-box pair of live proxies with at least one
\* moved proxy is found exactly once per UpdatePairs. Exactly-once is
\* SortedDuplicateFree below; completeness is this equality in the state
\* immediately after UpdatePairs (justUpdated), where alive/pos/emittedFor
\* still hold the values UpdatePairs computed from.
PairCompleteness ==
    justUpdated =>
        { emitted[i] : i \in 1..Len(emitted) }
            = ExpectedPairs(alive, pos, emittedFor)

\* (2) No pair involves a destroyed proxy.
NoDestroyedProxyInPairs ==
    /\ \A i \in 1..Len(emitted) : emitted[i][1] \in alive /\ emitted[i][2] \in alive
    /\ \A q \in pairSet : q[1] \in alive /\ q[2] \in alive

\* (3) Pair keys are totally ordered; the emitted list is sorted and
\* duplicate-free (strict increase gives both).
SortedDuplicateFree == StrictlySorted(emitted)

\* (4) The moved bit is cleared exactly at the end of UpdatePairs. The
\* mutant (DropClear = TRUE) violates this invariant.
MovedClearedAfterUpdate == justUpdated => (moved = {} /\ rootMoved = FALSE)

\* (5) A rebuild never loses a proxy or changes the pair set: in the
\* state immediately after RebuildStale, both equal the snapshot the
\* rebuild took of its input.
RebuildPreservesPairs ==
    justRebuilt => (alive = aliveAtRebuild /\ pairSet = pairsAtRebuild)

\* (6) The empty-node sentinel never reports an overlap, and never
\* appears in an emitted pair.
SentinelNeverOverlaps ==
    /\ \A p \in Proxies : ~NodeOverlap(pos, Sentinel, p) /\ ~NodeOverlap(pos, p, Sentinel)
    /\ \A i \in 1..Len(emitted) : emitted[i][1] # Sentinel /\ emitted[i][2] # Sentinel

\* Emitted pairs are real overlaps of live, allowed proxies (soundness,
\* the other half of rule 1).
EmittedPairsAreOverlaps ==
    justUpdated =>
    \A i \in 1..Len(emitted) :
        /\ NodeOverlap(pos, emitted[i][1], emitted[i][2])
        /\ AllowedPair(emitted[i][1], emitted[i][2])

Invariants ==
    /\ TypeOK
    /\ RootMarkConsistent
    /\ PairCompleteness
    /\ NoDestroyedProxyInPairs
    /\ SortedDuplicateFree
    /\ MovedClearedAfterUpdate
    /\ RebuildPreservesPairs
    /\ SentinelNeverOverlaps
    /\ EmittedPairsAreOverlaps

-----------------------------------------------------------------------------
(* Liveness: every proxy moved in a step is considered in that step's
   UpdatePairs -- unless it is destroyed first, in which case there is
   nothing left to consider (its pairs are removed with it). Weak
   fairness on UpdatePairs makes the step boundary reachable;
   considered is the snapshot UpdatePairs took of moved. *)
MovedEventuallyConsidered ==
    \A p \in Proxies :
        []((p \in moved) => <>((p \in considered) \/ (p \notin alive)))

=============================================================================
\* Modification History
\* Last modified Tue Oct 13 2026 by aurora (card work; no friend names)
\* Created Tue Oct 13 2026 for upstream box3d f555ee4 / PR #159.
