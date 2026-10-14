---------------------------- MODULE BroadPhaseTree ----------------------------
\* Abstract broad phase of upstream box3d f555ee42084e0b43cbffa863f40bff8117c08896
\* (Optimize Broad-Phase, #159). The state the fixed3d port has to keep.
\* Not a C test. No floats.
\*
\* Attempt 3 uses the corrected predicates. The card's literal rules are not
\* true of that commit:
\*
\* 1. Emission. b3FlushCandidatePairs drops a key already in pairSet, then
\*    b3ShouldCreatePair. Not every overlapping moved pair is emitted.
\*    NewEligibleEmittedOnce: every new eligible overlapping candidate that
\*    survives the existing-contact test and the shape/body filter is emitted
\*    once, as one sorted duplicate-free list.
\* 2. Moved bits. UpdatePairs clears the static tree only. Dynamic marks
\*    (kinematic would share this rule; it is not a third tree here) clear in
\*    RebuildStale, before the next step boundary. They are not required to
\*    be clear at the end of UpdatePairs.
\* 3. Liveness. A proxy moved and then destroyed before UpdatePairs is not
\*    obliged. LiveMovedConsidered applies to proxies still live at update
\*    time. A moved static proxy that overlaps no live dynamic is not
\*    visited: the dynamic root is the empty sentinel. That state is
\*    reachable in the C code, so it is not a violation here.
\*
\* ClearMoved is TRUE in the real model. The mutant cfg sets it FALSE and
\* drops both clears. MovedBitsClearedAsSpecified does not mention the
\* constant, so the mutant fails it.
\*
\* Layout is Narrow or Ludicrous. No action reads it.
\*
\* Insert pairs a new leaf with the old root (no SAH, no rotate). Three
\* proxies on that spine have height at most 2, so a box or moved-bit edit
\* updates the leaf, its parent, and its grandparent only. Rebuild clears
\* dynamic marks and does not reshape the tree.
\*
\* What this model does not cover is listed in tla/README.md.

EXTENDS Integers, FiniteSets, Sequences

CONSTANTS
  P1, P2, P3,
  Static, Dynamic,
  Narrow, Ludicrous,
  MaxActions,
  ClearMoved,
  Layouts

ASSUME ClearMoved \in BOOLEAN
ASSUME Layouts \subseteq {Narrow, Ludicrous} /\ Layouts # {}
ASSUME MaxActions \in 6..8

\* Mutations between step boundaries. UpdatePairs, RebuildStale, and
\* StepBoundary do not consume it. Three is enough to build both trees
\* and still move or destroy before the consuming step.
MaxBurst == 3

Proxies == {P1, P2, P3}
Trees == {Static, Dynamic}

\* Three fat intervals on a line, closed under bounding union. In this set
\* a union-overlap is a leaf-overlap, so the leaf-pair rule matches descent.
Intervals == {<<0, 0>>, <<1, 1>>, <<0, 1>>}
Points == {<<0, 0>>, <<1, 1>>}
Wide == <<0, 1>>

MaxN == 6
NodeIds == 0..MaxN

EmptyNode == [kind |-> "empty", moved |-> FALSE, parent |-> -1,
              left |-> -1, right |-> -1, proxy |-> "none", box |-> "none"]
FreeNode == [kind |-> "free", moved |-> FALSE, parent |-> -1,
             left |-> -1, right |-> -1, proxy |-> "none", box |-> "none"]

\* P1 and P2 share a body, so {P1,P2} is rejected. ShapeAllows is the shape
\* gauntlet and is true for every pair here. Sensor is false for all three.
\* The keys that can be emitted are <<P1,P3>> and <<P2,P3>>, in that order.
BodyOf(p) == IF p = P3 THEN "V" ELSE "U"
SameBody(p, q) == BodyOf(p) = BodyOf(q)
ShapeAllows(p, q) == TRUE
Sensor(p) == FALSE

PassesFilter(p, q) ==
  /\ ~SameBody(p, q)
  /\ ShapeAllows(p, q)
  /\ ~Sensor(p) /\ ~Sensor(q)

\* Self search is dynamic-dynamic. Cross search is dynamic-static.
\* Static-static is not searched.
Searched(t1, t2) == t1 = Dynamic \/ t2 = Dynamic

Rank(p) == CASE p = P1 -> 1 [] p = P2 -> 2 [] p = P3 -> 3

Min(a, b) == IF a < b THEN a ELSE b
Max(a, b) == IF a > b THEN a ELSE b
Overlaps(a, b) == a[1] <= b[2] /\ b[1] <= a[2]
Union(a, b) == <<Min(a[1], b[1]), Max(a[2], b[2])>>

\* Explicit empty marker. An empty node has no interval. NodeOverlaps is
\* false for it, including against Wide. Not a huge box.
NodeOverlaps(n1, n2) ==
  /\ n1.kind \in {"leaf", "internal"}
  /\ n2.kind \in {"leaf", "internal"}
  /\ Overlaps(n1.box, n2.box)

LeafNode(p, box, mark, par) ==
  [kind |-> "leaf", moved |-> mark, parent |-> par,
   left |-> -1, right |-> -1, proxy |-> p, box |-> box]

InternalNode(lft, rgt, par, mark, box) ==
  [kind |-> "internal", moved |-> mark, parent |-> par,
   left |-> lft, right |-> rgt, proxy |-> "none", box |-> box]

\* One function, read off the input. Not stacked on a previous copy.
ClearTree(ns) ==
  [k \in NodeIds |-> [ns[k] EXCEPT !.moved = FALSE]]

\* First leaf replaces the empty root. Later leaves pair with the old root.
Insert(ns, rt, p, box, mark) ==
  IF rt = 0
  THEN LET i == CHOOSE j \in 1..MaxN : ns[j].kind = "free"
       IN <<[k \in NodeIds |->
               IF k = i THEN LeafNode(p, box, mark, -1) ELSE ns[k]],
            i>>
  ELSE LET i == CHOOSE j \in 1..MaxN : ns[j].kind = "free"
           j == CHOOSE k \in 1..MaxN : k # i /\ ns[k].kind = "free"
           ubox == Union(ns[rt].box, box)
           umov == ns[rt].moved \/ mark
       IN <<[k \in NodeIds |->
               IF k = i THEN LeafNode(p, box, mark, j)
               ELSE IF k = j THEN InternalNode(rt, i, -1, umov, ubox)
               ELSE IF k = rt THEN [ns[rt] EXCEPT !.parent = j]
               ELSE ns[k]],
            j>>

\* b3RemoveLeaf: a root leaf becomes the sentinel; otherwise the sibling
\* takes the parent's place. A grandparent, if any, retakes the union.
Remove(ns, rt, p) ==
  LET i == CHOOSE j \in 1..MaxN : ns[j].kind = "leaf" /\ ns[j].proxy = p
      par == ns[i].parent
  IN IF par = -1
     THEN <<[k \in NodeIds |-> IF k = i THEN FreeNode ELSE ns[k]], 0>>
     ELSE LET sib == IF ns[par].left = i THEN ns[par].right ELSE ns[par].left
              gp == ns[par].parent
          IN IF gp = -1
             THEN <<[k \in NodeIds |->
                       IF k = i \/ k = par THEN FreeNode
                       ELSE IF k = sib THEN [ns[sib] EXCEPT !.parent = -1]
                       ELSE ns[k]],
                    sib>>
             ELSE LET leftP == IF ns[gp].left = par THEN sib ELSE ns[gp].left
                      rightP == IF ns[gp].right = par THEN sib ELSE ns[gp].right
                      boxP == Union(ns[leftP].box, ns[rightP].box)
                      movP == ns[leftP].moved \/ ns[rightP].moved
                  IN <<[k \in NodeIds |->
                          IF k = i \/ k = par THEN FreeNode
                          ELSE IF k = sib THEN [ns[sib] EXCEPT !.parent = gp]
                          ELSE IF k = gp
                               THEN [ns[gp] EXCEPT !.left = leftP,
                                                  !.right = rightP,
                                                  !.box = boxP,
                                                  !.moved = movP]
                          ELSE ns[k]],
                       rt>>

\* In-place grow of a point box to Wide. Parent and grandparent only.
Enlarge(ns, i) ==
  LET par == ns[i].parent
      gp == IF par = -1 THEN -1 ELSE ns[par].parent
      boxAt(n) == IF n = i THEN Wide ELSE ns[n].box
      movAt(n) == IF n = i THEN TRUE ELSE ns[n].moved
      parBox == IF par = -1 THEN "none"
                ELSE Union(boxAt(ns[par].left), boxAt(ns[par].right))
      parMov == IF par = -1 THEN FALSE
                ELSE movAt(ns[par].left) \/ movAt(ns[par].right)
      boxC(c) == IF c = par THEN parBox ELSE boxAt(c)
      movC(c) == IF c = par THEN parMov ELSE movAt(c)
      gpBox == IF gp = -1 THEN "none"
               ELSE Union(boxC(ns[gp].left), boxC(ns[gp].right))
      gpMov == IF gp = -1 THEN FALSE
               ELSE movC(ns[gp].left) \/ movC(ns[gp].right)
  IN [k \in NodeIds |->
       IF k = i THEN [ns[i] EXCEPT !.box = Wide, !.moved = TRUE]
       ELSE IF par # -1 /\ k = par
            THEN [ns[par] EXCEPT !.box = parBox, !.moved = parMov]
       ELSE IF gp # -1 /\ k = gp
            THEN [ns[gp] EXCEPT !.box = gpBox, !.moved = gpMov]
       ELSE ns[k]]

VARIABLES
  layout,
  proxies,
  nodes,
  root,
  moveBuffer,
  pairSet,
  emitted,
  considered,
  pre,
  havePre,
  phase,
  stepSeq,
  taken,
  burst

vars == <<layout, proxies, nodes, root, moveBuffer, pairSet, emitted,
          considered, pre, havePre, phase, stepSeq, taken, burst>>

NullPre == [
  alive |-> [p \in Proxies |-> FALSE],
  treeOf |-> [p \in Proxies |-> Static],
  boxOf |-> [p \in Proxies |-> <<0, 0>>],
  moved |-> {},
  pairSet |-> {}
]

\* Lowest dead id. Any live proxy can still be destroyed.
LowestDead ==
  IF ~proxies[P1].alive THEN P1
  ELSE IF ~proxies[P2].alive THEN P2
  ELSE IF ~proxies[P3].alive THEN P3
  ELSE "none"

BlankTree == [i \in NodeIds |-> IF i = 0 THEN EmptyNode ELSE FreeNode]

Init ==
  /\ layout \in Layouts
  /\ proxies = [p \in Proxies |-> [alive |-> FALSE, tree |-> Static, box |-> <<0, 0>>]]
  /\ nodes = [t \in Trees |-> BlankTree]
  /\ root = [t \in Trees |-> 0]
  /\ moveBuffer = {}
  /\ pairSet = {}
  /\ emitted = <<>>
  /\ considered = {}
  /\ pre = NullPre
  /\ havePre = FALSE
  /\ phase = "mutate"
  /\ stepSeq = <<>>
  /\ taken = 0
  /\ burst = 0

CanAct == taken < MaxActions /\ phase = "mutate" /\ burst < MaxBurst

\* Create always marks. Callers that pass forcePairCreation FALSE are not
\* in this instance.
CreateProxy ==
  /\ CanAct
  /\ LowestDead # "none"
  /\ \E t \in Trees, b \in Points:
       LET p == LowestDead
           ins == Insert(nodes[t], root[t], p, b, TRUE)
       IN /\ nodes' = [nodes EXCEPT ![t] = ins[1]]
          /\ root' = [root EXCEPT ![t] = ins[2]]
          /\ proxies' = [proxies EXCEPT ![p] = [alive |-> TRUE, tree |-> t, box |-> b]]
          /\ moveBuffer' = moveBuffer \cup {p}
          /\ stepSeq' = Append(stepSeq, "CreateProxy")
          /\ taken' = taken + 1
          /\ burst' = burst + 1
          /\ UNCHANGED <<layout, pairSet, emitted, considered, pre, havePre, phase>>

DestroyProxy ==
  /\ CanAct
  /\ \E p \in Proxies:
       /\ proxies[p].alive
       /\ LET t == proxies[p].tree
              gone == Remove(nodes[t], root[t], p)
          IN /\ nodes' = [nodes EXCEPT ![t] = gone[1]]
             /\ root' = [root EXCEPT ![t] = gone[2]]
             /\ proxies' = [proxies EXCEPT ![p].alive = FALSE]
             /\ moveBuffer' = moveBuffer \ {p}
             /\ pairSet' = {k \in pairSet : k[1] # p /\ k[2] # p}
             /\ stepSeq' = Append(stepSeq, "DestroyProxy")
             /\ taken' = taken + 1
             /\ burst' = burst + 1
             /\ UNCHANGED <<layout, emitted, considered, pre, havePre, phase>>

\* Remove and reinsert, marked. The pair set is not edited here.
MoveProxy ==
  /\ CanAct
  /\ \E p \in Proxies, b \in Intervals:
       /\ proxies[p].alive
       /\ b # proxies[p].box
       /\ LET t == proxies[p].tree
              gone == Remove(nodes[t], root[t], p)
              ins == Insert(gone[1], gone[2], p, b, TRUE)
          IN /\ nodes' = [nodes EXCEPT ![t] = ins[1]]
             /\ root' = [root EXCEPT ![t] = ins[2]]
             /\ proxies' = [proxies EXCEPT ![p].box = b]
             /\ moveBuffer' = moveBuffer \cup {p}
             /\ stepSeq' = Append(stepSeq, "MoveProxy")
             /\ taken' = taken + 1
             /\ burst' = burst + 1
             /\ UNCHANGED <<layout, pairSet, emitted, considered, pre, havePre, phase>>

EnlargeProxy ==
  /\ CanAct
  /\ \E p \in Proxies:
       /\ proxies[p].alive
       /\ proxies[p].box \in Points
       /\ LET t == proxies[p].tree
              i == CHOOSE j \in 1..MaxN :
                     nodes[t][j].kind = "leaf" /\ nodes[t][j].proxy = p
          IN /\ nodes' = [nodes EXCEPT ![t] = Enlarge(nodes[t], i)]
             /\ proxies' = [proxies EXCEPT ![p].box = Wide]
             /\ moveBuffer' = moveBuffer \cup {p}
             /\ stepSeq' = Append(stepSeq, "EnlargeProxy")
             /\ taken' = taken + 1
             /\ burst' = burst + 1
             /\ UNCHANGED <<layout, root, pairSet, emitted, considered, pre, havePre, phase>>

Snapshot == [
  alive |-> [p \in Proxies |-> proxies[p].alive],
  treeOf |-> [p \in Proxies |-> proxies[p].tree],
  boxOf |-> [p \in Proxies |-> proxies[p].box],
  moved |-> moveBuffer,
  pairSet |-> pairSet
]

Eligible(snap) ==
  { <<p, q>> \in Proxies \X Proxies :
      /\ Rank(p) < Rank(q)
      /\ snap.alive[p] /\ snap.alive[q]
      /\ Overlaps(snap.boxOf[p], snap.boxOf[q])
      /\ (p \in snap.moved \/ q \in snap.moved)
      /\ Searched(snap.treeOf[p], snap.treeOf[q])
      /\ PassesFilter(p, q)
      /\ <<p, q>> \notin snap.pairSet }

\* Moved dynamic leaves are tested. A moved static leaf is tested only when
\* some live dynamic overlaps it. Destroyed proxies are already out of
\* snap.moved and not alive.
Obliged(snap, p) ==
  /\ snap.alive[p]
  /\ p \in snap.moved
  /\ \/ snap.treeOf[p] = Dynamic
     \/ \E q \in Proxies :
          /\ snap.alive[q]
          /\ snap.treeOf[q] = Dynamic
          /\ Overlaps(snap.boxOf[p], snap.boxOf[q])

BeforeKey(a, b) ==
  \/ Rank(a[1]) < Rank(b[1])
  \/ /\ Rank(a[1]) = Rank(b[1])
     /\ Rank(a[2]) < Rank(b[2])

\* At most three unordered pairs exist among three proxies.
SortedSeq(S) ==
  IF S = {} THEN <<>>
  ELSE LET k1 == CHOOSE x \in S : \A y \in S \ {x} : BeforeKey(x, y)
           r1 == S \ {k1}
       IN IF r1 = {} THEN <<k1>>
          ELSE LET k2 == CHOOSE x \in r1 : \A y \in r1 \ {x} : BeforeKey(x, y)
                   r2 == r1 \ {k2}
               IN IF r2 = {} THEN <<k1, k2>>
                  ELSE LET k3 == CHOOSE x \in r2 :
                                  \A y \in r2 \ {x} : BeforeKey(x, y)
                       IN <<k1, k2, k3>>

SeqToSet(s) == {s[i] : i \in DOMAIN s}

\* Snapshot first, then clear static marks only. A dynamic proxy in the
\* pre-clear buffer schedules the rebuild (phase "rebuild").
UpdatePairs ==
  /\ phase = "mutate"
  /\ taken < MaxActions
  /\ moveBuffer # {}
  /\ LET snap == Snapshot
         seq == SortedSeq(Eligible(snap))
         owed == \E p \in moveBuffer : proxies[p].tree = Dynamic
     IN /\ nodes' = [nodes EXCEPT ![Static] =
                       IF ClearMoved THEN ClearTree(nodes[Static]) ELSE nodes[Static]]
        /\ moveBuffer' = IF ClearMoved
                         THEN {p \in moveBuffer : proxies[p].tree = Dynamic}
                         ELSE moveBuffer
        /\ pairSet' = pairSet \cup SeqToSet(seq)
        /\ emitted' = seq
        /\ considered' = {p \in Proxies : Obliged(snap, p)}
        /\ pre' = snap
        /\ havePre' = TRUE
        /\ phase' = IF owed THEN "rebuild" ELSE "bound"
        /\ stepSeq' = Append(stepSeq, "UpdatePairs")
        /\ taken' = taken + 1
        /\ UNCHANGED <<layout, proxies, root, burst>>

\* Clears dynamic marks when ClearMoved. Does not drop a proxy, does not
\* change pairSet, does not touch the static tree, does not reparent.
RebuildStale ==
  /\ phase = "rebuild"
  /\ taken < MaxActions
  /\ nodes' = [nodes EXCEPT ![Dynamic] =
                 IF ClearMoved THEN ClearTree(nodes[Dynamic]) ELSE nodes[Dynamic]]
  /\ moveBuffer' = IF ClearMoved THEN {} ELSE moveBuffer
  /\ phase' = "bound"
  /\ stepSeq' = Append(stepSeq, "RebuildStale")
  /\ taken' = taken + 1
  /\ UNCHANGED <<layout, proxies, root, pairSet, emitted, considered, pre, havePre, burst>>

\* End of the step. Contacts whose fat boxes no longer overlap are
\* destroyed here, not inside UpdatePairs. That removes the pair key.
StepBoundary ==
  /\ taken < MaxActions
  /\ \/ phase = "bound"
     \/ /\ phase = "mutate"
        /\ moveBuffer = {}
        /\ stepSeq # <<>>
  /\ pairSet' = {k \in pairSet :
                   /\ proxies[k[1]].alive
                   /\ proxies[k[2]].alive
                   /\ Overlaps(proxies[k[1]].box, proxies[k[2]].box)}
  /\ phase' = "mutate"
  /\ stepSeq' = <<>>
  /\ burst' = 0
  /\ taken' = taken + 1
  /\ UNCHANGED <<layout, proxies, nodes, root, moveBuffer, emitted, considered, pre, havePre>>

Done ==
  /\ taken >= MaxActions
  /\ UNCHANGED vars

Next ==
  \/ CreateProxy
  \/ DestroyProxy
  \/ MoveProxy
  \/ EnlargeProxy
  \/ UpdatePairs
  \/ RebuildStale
  \/ StepBoundary
  \/ Done

Spec == Init /\ [][Next]_vars

\* --- invariants ---

NonFree(ns) == {i \in 1..MaxN : ns[i].kind \in {"leaf", "internal"}}

\* Height at most 2: the third parent step is -1. A cycle fails this.
ClimbsToRoot(ns, i) ==
  LET p1 == ns[i].parent
      p2 == IF p1 = -1 THEN -1 ELSE ns[p1].parent
      p3 == IF p2 = -1 THEN -1 ELSE ns[p2].parent
  IN /\ p1 \in NonFree(ns) \cup {-1}
     /\ p3 = -1

TreeOk(t) ==
  LET ns == nodes[t]
      live == {p \in Proxies : proxies[p].alive /\ proxies[p].tree = t}
      leaves == {i \in NonFree(ns) : ns[i].kind = "leaf"}
  IN /\ ns[0] = EmptyNode
     /\ \A i \in 1..MaxN : ns[i].kind \in {"free", "leaf", "internal"}
     /\ \A i \in 1..MaxN : ns[i].kind = "free" => ns[i] = FreeNode
     /\ \/ /\ live = {}
           /\ root[t] = 0
           /\ NonFree(ns) = {}
        \/ /\ live # {}
           /\ root[t] \in NonFree(ns)
           /\ ns[root[t]].parent = -1
           /\ Cardinality(NonFree(ns)) = 2 * Cardinality(live) - 1
           /\ Cardinality(leaves) = Cardinality(live)
           /\ \A i \in NonFree(ns) : ns[i].parent = -1 <=> i = root[t]
           /\ \A i \in NonFree(ns) : ClimbsToRoot(ns, i)
           /\ \A i \in leaves :
                /\ ns[i].proxy \in live
                /\ ns[i].box = proxies[ns[i].proxy].box
                /\ ns[i].box \in Intervals
                /\ ns[i].left = -1 /\ ns[i].right = -1
                /\ (ns[i].proxy \in moveBuffer) <=> ns[i].moved
           /\ {ns[i].proxy : i \in leaves} = live
           /\ \A i \in NonFree(ns) :
                ns[i].kind = "internal" =>
                  /\ ns[i].left \in NonFree(ns) /\ ns[i].right \in NonFree(ns)
                  /\ ns[i].left # ns[i].right
                  /\ ns[ns[i].left].parent = i
                  /\ ns[ns[i].right].parent = i
                  /\ ns[i].proxy = "none"
                  /\ ns[i].box = Union(ns[ns[i].left].box, ns[ns[i].right].box)
                  /\ ns[i].moved =
                       (ns[ns[i].left].moved \/ ns[ns[i].right].moved)

TypeOK ==
  /\ layout \in Layouts
  /\ \A p \in Proxies :
       /\ proxies[p].alive \in BOOLEAN
       /\ proxies[p].tree \in Trees
       /\ proxies[p].box \in Intervals
  /\ moveBuffer \subseteq Proxies
  /\ pairSet \subseteq (Proxies \X Proxies)
  /\ Len(emitted) <= 3
  /\ \A i \in 1..Len(emitted) : emitted[i] \in Proxies \X Proxies
  /\ considered \subseteq Proxies
  /\ phase \in {"mutate", "rebuild", "bound"}
  /\ taken \in 0..MaxActions
  /\ burst \in 0..MaxBurst
  /\ Len(stepSeq) <= MaxActions
  /\ pre.moved \subseteq Proxies
  /\ pre.pairSet \subseteq (Proxies \X Proxies)

TreeAgrees == \A t \in Trees : TreeOk(t)

MoveBufferAgrees ==
  moveBuffer = {p \in Proxies :
                  /\ proxies[p].alive
                  /\ \E i \in NonFree(nodes[proxies[p].tree]) :
                       /\ nodes[proxies[p].tree][i].kind = "leaf"
                       /\ nodes[proxies[p].tree][i].proxy = p
                       /\ nodes[proxies[p].tree][i].moved}

IsSorted(s) ==
  \A i, j \in 1..Len(s) : i < j => BeforeKey(s[i], s[j])

\* Checked on the snapshot UpdatePairs took, before the static clear.
\* Vacuous in the mutate phase.
NewEligibleEmittedOnce ==
  \/ phase = "mutate"
  \/ /\ havePre
     /\ SeqToSet(emitted) = Eligible(pre)
     /\ pairSet = pre.pairSet \cup SeqToSet(emitted)
     /\ IsSorted(emitted)

NoPairOfDestroyedProxy ==
  \A k \in pairSet : proxies[k[1]].alive /\ proxies[k[2]].alive

PairKeysTotallyOrdered ==
  /\ \A k \in pairSet : Rank(k[1]) < Rank(k[2])
  /\ \A i \in 1..Len(emitted) : Rank(emitted[i][1]) < Rank(emitted[i][2])
  /\ IsSorted(emitted)

EmittedSorted ==
  /\ IsSorted(emitted)
  /\ \A i, j \in 1..Len(emitted) : i < j => emitted[i] # emitted[j]

\* Static marks are clear in rebuild and bound. Dynamic marks are clear
\* at bound, after the rebuild, not at the end of UpdatePairs.
MovedBitsClearedAsSpecified ==
  /\ phase \in {"rebuild", "bound"} =>
       \A p \in Proxies :
         ~(proxies[p].alive /\ proxies[p].tree = Static /\ p \in moveBuffer)
  /\ phase = "bound" => moveBuffer = {}

SentinelNeverOverlaps ==
  \A t \in Trees :
    /\ nodes[t][0].kind = "empty"
    /\ nodes[t][0].box = "none"
    /\ nodes[t][0].moved = FALSE
    /\ \A i \in NodeIds : ~NodeOverlaps(nodes[t][0], nodes[t][i])

\* After a step boundary the pair set has dropped fat-box separations.
DisjointPairsDropped ==
  phase = "mutate" /\ stepSeq = <<>> =>
    \A k \in pairSet : Overlaps(proxies[k[1]].box, proxies[k[2]].box)

LiveMovedConsidered ==
  \/ phase = "mutate"
  \/ /\ havePre
     /\ considered = {p \in Proxies : Obliged(pre, p)}

LayoutIsAbstractLabel ==
  /\ layout \in Layouts
  /\ layout \in {Narrow, Ludicrous}

=============================================================================
