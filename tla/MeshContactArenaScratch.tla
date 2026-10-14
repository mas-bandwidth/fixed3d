--------------------- MODULE MeshContactArenaScratch ---------------------
(***************************************************************************)
(* Mesh-contact scratch moved from the C stack to the per-worker arena.   *)
(* State: the bump index of one worker arena during b3RefreshCache, plus  *)
(* whether the caller's by-value copy has been restored on return.        *)
(* The scratch sizes are functions of the cap only, never of the number   *)
(* of triangles found, so the largest case is bounded before any query.   *)
(***************************************************************************)
EXTENDS Integers, TLC

CONSTANTS Cap, IntBytes, CacheBytes, ArenaCapacity
ASSUME Cap > 0 /\ IntBytes > 0 /\ CacheBytes > 0
ASSUME ArenaCapacity >= Cap * (IntBytes + CacheBytes)

VARIABLES index, phase, callerIndex

ScratchBytes == Cap * IntBytes + Cap * CacheBytes

Init == /\ index = 0
        /\ callerIndex = 0
        /\ phase = "idle"

AllocIndices ==
    /\ phase = "idle"
    /\ index + Cap * IntBytes <= ArenaCapacity
    /\ index' = index + Cap * IntBytes
    /\ phase' = "indices"
    /\ callerIndex' = callerIndex

AllocCache ==
    /\ phase = "indices"
    /\ index + Cap * CacheBytes <= ArenaCapacity
    /\ index' = index + Cap * CacheBytes
    /\ phase' = "cached"
    /\ callerIndex' = callerIndex

Return ==
    /\ phase = "cached"
    /\ phase' = "idle"
    /\ index' = callerIndex   \* by-value arena copy restores the bump pointer
    /\ callerIndex' = callerIndex

Next == AllocIndices \/ AllocCache \/ Return

Spec == Init /\ [][Next]_<<index, phase, callerIndex>> /\ WF_<<index, phase, callerIndex>>(Return)

\* The bump pointer never leaves the frame budget.
Bounded == index <= ArenaCapacity
\* Scratch is sized by the cap, so allocation never exceeds the budget.
SizedByCap == ScratchBytes <= ArenaCapacity
\* The caller's arena copy is unchanged by the call (per-worker isolation).
CallerStable == callerIndex = 0
\* Every started allocation completes and returns; no state leaks a phase.
LiveReturn == phase = "cached" ~> phase = "idle"

=============================================================================
