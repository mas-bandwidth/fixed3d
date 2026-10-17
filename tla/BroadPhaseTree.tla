--------------------------- MODULE BroadPhaseTree ---------------------------
EXTENDS Naturals, Sequences, FiniteSets, TLC
CONSTANTS MaxSteps, DropClear, NodeBytes

\* Two bounded binary trees: one static leaf, three dynamic leaves. An unoccupied slot
\* is an empty node. NodeBytes deliberately has no semantic use.
Proxies == 1..4
StaticP == {1}
DynP == {2, 3, 4}
Nodes == 0..7
Roots == {5, 6}
Internal == {5, 6, 7}
Cells == 0..1
Sentinel == 0
Children(order) == [n \in Nodes |->
    CASE n = 5 -> IF order = 0 THEN <<1, 0>> ELSE <<0, 1>>
      [] n = 6 -> IF order = 0 THEN <<2, 7>> ELSE <<7, 2>>
      [] n = 7 -> IF order = 0 THEN <<3, 4>> ELSE <<4, 3>>
      [] OTHER -> <<>>]
Parents(ch) == [n \in Nodes |->
    IF n \in Proxies \cup {7} THEN CHOOSE r \in Internal : n \in {ch[r][1], ch[r][2]}
    ELSE Sentinel]
RECURSIVE Descendants(_, _)
Descendants(ch, n) ==
    IF n = Sentinel THEN {} ELSE IF n \in Proxies THEN {n}
    ELSE Descendants(ch, ch[n][1]) \cup Descendants(ch, ch[n][2])
Marks(ch, m) == [n \in Nodes |-> Descendants(ch, n) \cap m # {}]
Boxes(ch, a, ps, n) == {ps[p] : p \in Descendants(ch, n) \cap a}
NodeOverlap(ch, a, ps, n, k) == Boxes(ch, a, ps, n) \cap Boxes(ch, a, ps, k) # {}
Key(a, b) == a * 10 + b
UnorderedPair(a, b) == IF a < b THEN <<a, b>> ELSE <<b, a>>
AllowedPair(a, b) == a \in DynP \/ b \in DynP
\* Independent leaf oracle: never called by seeding, visiting or emission.
ExpectedPairs(a, ps, m) ==
    {p \in {UnorderedPair(x, y) : x, y \in a} :
      p[1] # p[2] /\ AllowedPair(p[1], p[2]) /\ ps[p[1]] = ps[p[2]]
      /\ (p[1] \in m \/ p[2] \in m)}
PairValues(s) == {s[i] : i \in 1..Len(s)}
RECURSIVE SortPairs(_)
SortPairs(s) == IF s = {} THEN <<>> ELSE
    LET p == CHOOSE p \in s : \A q \in s : Key(p[1], p[2]) <= Key(q[1], q[2])
    IN <<p>> \o SortPairs(s \ {p})
StrictlySorted(s) == \A i \in 1..(Len(s)-1) :
    Key(s[i][1], s[i][2]) < Key(s[i+1][1], s[i+1][2])

VARIABLES alive, pos, moved, nodeMoved, children, parent, layout,
    considered, emitted, emittedFor, candidates, work, pairSet,
    phase, justUpdated, justRebuilt, aliveAtRebuild, pairsAtRebuild, step, ticks
vars == <<alive, pos, moved, nodeMoved, children, parent, layout,
    considered, emitted, emittedFor, candidates, work, pairSet,
    phase, justUpdated, justRebuilt, aliveAtRebuild, pairsAtRebuild, step, ticks>>

Init ==
    /\ alive = {} /\ pos = [p \in Proxies |-> 0]
    /\ moved = {} /\ nodeMoved = Marks(Children(0), {})
    /\ children = Children(0) /\ parent = Parents(children) /\ layout = 0
    /\ considered = {} /\ emitted = <<>> /\ emittedFor = {}
    /\ candidates = <<>> /\ work = <<>> /\ pairSet = {}
    /\ phase = "collect" /\ justUpdated = FALSE /\ justRebuilt = FALSE
    /\ aliveAtRebuild = {} /\ pairsAtRebuild = {} /\ step = 0 /\ ticks = 0

\* Solver finalization may mark dynamic proxies AFTER rebuild has joined.
\* Such marks survive StepBoundary and are consumed by the NEXT update.
CanMutate == ticks < MaxSteps /\ phase \in {"collect", "solver"}
MarkProxy(p, c, create) ==
    /\ CanMutate
    /\ IF create THEN phase = "collect" /\ p \notin alive
                  ELSE p \in alive /\ (phase = "collect" \/ p \in DynP)
    /\ alive' = alive \cup {p} /\ pos' = [pos EXCEPT ![p] = c]
    /\ moved' = moved \cup {p} /\ nodeMoved' = Marks(children, moved')
    /\ considered' = considered \ {p} \* fresh obligation, even same-cell move
    /\ ticks' = ticks + 1 /\ justUpdated' = FALSE /\ justRebuilt' = FALSE
    /\ UNCHANGED <<children, parent, layout, emitted, emittedFor, candidates,
                    work, pairSet, phase, aliveAtRebuild, pairsAtRebuild, step>>
CreateProxy(p, c) == MarkProxy(p, c, TRUE)
MoveProxy(p, c) == MarkProxy(p, c, FALSE)
EnlargeProxy(p) == MarkProxy(p, pos[p], FALSE)
DestroyProxy(p) ==
    /\ CanMutate /\ phase = "collect" /\ p \in alive
    /\ alive' = alive \ {p} /\ moved' = moved \ {p}
    /\ nodeMoved' = Marks(children, moved') /\ considered' = considered \ {p}
    /\ pairSet' = {q \in pairSet : q[1] # p /\ q[2] # p}
    /\ emitted' = <<>> /\ emittedFor' = {} /\ candidates' = <<>>
    /\ ticks' = ticks + 1 /\ justUpdated' = FALSE /\ justRebuilt' = FALSE
    /\ UNCHANGED <<pos, children, parent, layout, work, phase,
                    aliveAtRebuild, pairsAtRebuild, step>>

TestPair(a, b) == (nodeMoved[a] \/ nodeMoved[b]) /\
    NodeOverlap(children, alive, pos, a, b)
\* A moved dynamic internal node seeds its disjoint sibling subtrees;
\* cross-tree gathering starts with the dynamic/static root pair (C:515).
Seeds == (IF nodeMoved[6] THEN <<<<children[6][1], children[6][2]>>>> ELSE <<>>)
    \o (IF nodeMoved[7] THEN <<<<children[7][1], children[7][2]>>>> ELSE <<>>)
    \o (IF TestPair(6, 5) THEN <<<<6, 5>>>> ELSE <<>>)
BeginUpdate ==
    /\ phase = "collect" /\ phase' = "update"
    /\ work' = Seeds /\ candidates' = <<>> /\ emittedFor' = moved
    /\ justUpdated' = FALSE /\ justRebuilt' = FALSE
    /\ UNCHANGED <<alive, pos, moved, nodeMoved, children, parent, layout,
                    considered, emitted, pairSet, aliveAtRebuild, pairsAtRebuild, step, ticks>>
\* One explicit work item. Two internal nodes push all four child pairs;
\* leaf/internal visits push two. Raw production stays a SEQUENCE.
VisitPair ==
    /\ phase = "update" /\ work # <<>>
    /\ LET a == Head(work)[1] b == Head(work)[2]
           leafA == children[a] = <<>> leafB == children[b] = <<>>
           push == IF ~TestPair(a, b) \/ (leafA /\ leafB) THEN <<>>
                   ELSE IF leafA THEN <<<<a, children[b][1]>>, <<a, children[b][2]>>>>
                   ELSE IF leafB THEN <<<<children[a][1], b>>, <<children[a][2], b>>>>
                   ELSE <<<<children[a][1], children[b][1]>>, <<children[a][1], children[b][2]>>,
                          <<children[a][2], children[b][1]>>, <<children[a][2], children[b][2]>>>>
       IN /\ work' = push \o Tail(work)
          /\ candidates' = IF TestPair(a, b) /\ leafA /\ leafB
                           THEN Append(candidates, UnorderedPair(a, b)) ELSE candidates
    /\ UNCHANGED <<alive, pos, moved, nodeMoved, children, parent, layout,
                    considered, emitted, emittedFor, pairSet, phase, justUpdated,
                    justRebuilt, aliveAtRebuild, pairsAtRebuild, step, ticks>>
UpdatePairs ==
    /\ phase = "update" /\ work = <<>> /\ phase' = "rebuild"
    /\ emitted' = SortPairs(PairValues(candidates))
    /\ pairSet' = pairSet \cup PairValues(candidates)
    /\ considered' = considered \cup emittedFor
    \* Upstream ClearMoved at broad_phase.c:701 clears STATIC only.
    /\ moved' = IF DropClear THEN moved ELSE moved \cap DynP
    /\ nodeMoved' = Marks(children, moved')
    /\ justUpdated' = TRUE /\ justRebuilt' = FALSE
    /\ UNCHANGED <<alive, pos, children, parent, layout, emittedFor, candidates,
                    work, aliveAtRebuild, pairsAtRebuild, step, ticks>>
RebuildStale ==
    /\ phase = "rebuild" /\ phase' = "solver"
    \* The queued dynamic rebuild joins before solver finalization marks.
    \* Reorder real child links, rebuild parents, preserve leaf membership.
    /\ layout' = 1-layout /\ children' = Children(layout') /\ parent' = Parents(children')
    /\ moved' = IF DropClear THEN moved ELSE {}
    /\ nodeMoved' = Marks(children', moved')
    /\ aliveAtRebuild' = alive /\ pairsAtRebuild' = pairSet /\ justRebuilt' = TRUE
    /\ UNCHANGED <<alive, pos, considered, emitted, emittedFor, candidates,
                    work, pairSet, justUpdated, step, ticks>>
StepBoundary ==
    /\ phase = "solver" /\ phase' = "collect"
    /\ step' = IF step < MaxSteps THEN step+1 ELSE step
    /\ justUpdated' = FALSE /\ justRebuilt' = FALSE
    /\ UNCHANGED <<alive, pos, moved, nodeMoved, children, parent, layout,
                    considered, emitted, emittedFor, candidates, work, pairSet,
                    aliveAtRebuild, pairsAtRebuild, ticks>>
Next ==
    \/ \E p \in Proxies, c \in Cells : CreateProxy(p, c) \/ MoveProxy(p, c)
    \/ \E p \in Proxies : EnlargeProxy(p) \/ DestroyProxy(p)
    \/ BeginUpdate \/ VisitPair \/ UpdatePairs \/ RebuildStale \/ StepBoundary
Spec == Init /\ [][Next]_vars /\ WF_vars(BeginUpdate) /\ WF_vars(VisitPair)
    /\ WF_vars(UpdatePairs) /\ WF_vars(RebuildStale) /\ WF_vars(StepBoundary)

TypeOK ==
    /\ alive \subseteq Proxies /\ pos \in [Proxies -> Cells] /\ moved \subseteq alive
    /\ considered \subseteq alive /\ nodeMoved \in [Nodes -> BOOLEAN]
    /\ children = Children(layout) /\ parent = Parents(children) /\ layout \in 0..1
    /\ emitted \in Seq(Proxies \X Proxies) /\ Len(emitted) <= 6
    /\ candidates \in Seq(Proxies \X Proxies) /\ Len(candidates) <= 6
    /\ work \in Seq(Nodes \X Nodes) /\ Len(work) <= 6
    /\ pairSet \subseteq Proxies \X Proxies
    /\ phase \in {"collect", "update", "rebuild", "solver"}
    /\ ticks \in 0..MaxSteps /\ step \in 0..MaxSteps
RootMarkConsistent == nodeMoved = Marks(children, moved)
TreeConsistent ==
    /\ \A p \in Proxies \cup {7} : p \in {children[parent[p]][1], children[parent[p]][2]}
    /\ Descendants(children, 5) = StaticP /\ Descendants(children, 6) = DynP
PairCompleteness == justUpdated => PairValues(emitted) = ExpectedPairs(alive, pos, emittedFor)
NoDestroyedProxyInPairs == \A q \in PairValues(emitted) \cup pairSet : q[1] \in alive /\ q[2] \in alive
SortedDuplicateFree == StrictlySorted(emitted)
\* Validate duplicates BEFORE deduplication can mask a traversal bug.
CandidateDuplicateFree == Len(candidates) = Cardinality(PairValues(candidates))
MovedClearedAfterUpdate == justRebuilt => moved = {} /\ ~nodeMoved[5] /\ ~nodeMoved[6]
RebuildPreservesPairs == justRebuilt => alive = aliveAtRebuild /\ pairSet = pairsAtRebuild
SentinelNeverOverlaps == \A n \in Nodes : ~NodeOverlap(children, alive, pos, Sentinel, n)
EmittedPairsAreOverlaps == justUpdated => \A q \in PairValues(emitted) :
    pos[q[1]] = pos[q[2]] /\ AllowedPair(q[1], q[2])
\* The correct boundary is rebuild join -> solver finalization, not end of
\* solver -> next step. Later solver moves must NOT be forced backwards.
NoBoundaryWithPendingMove == [][(phase # "solver" /\ phase' = "solver") =>
    moved' = {} /\ emittedFor \subseteq considered']_vars
MovedEventuallyConsidered == \A p \in Proxies :
    []((p \in alive /\ p \notin considered) => <>(p \in considered \/ p \notin alive))
=============================================================================
