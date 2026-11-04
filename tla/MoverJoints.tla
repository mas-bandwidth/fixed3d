---------------------------- MODULE MoverJoints ----------------------------
\* Accumulated-impulse state of the mover joint and the pogo joint (upstream box3d 3362633,
\* src/mover_joint.c and src/pogo_joint.c). Both joints carry impulse state across steps: it is
\* warm started, clamped on every solve, kept through sleep and wake (solver_set.c memcpy), and
\* copied by the world snapshot. Setters change the clamp bounds between steps, and the pogo is
\* recreated from a def that seeds its impulse and spring velocity.
\*
\* LAYOUT, NOT ONLY CONTRACT: the mover's circular clamp is modelled with the code's own integer
\* arithmetic at a reduced fraction width F (Q.F instead of Q48.16). ClampForm "exact" is
\* b3ClampToDisc in src/mover_joint.c: exact sum of raw squares, exact compare, ceiling square
\* root, each component |x| * cap / length truncated toward zero. ClampForm "rounded" is the
\* src/joint.h spelling the motor and parallel joints use (b3ImpulseOverCap2 rounding each square
\* half up, b3Length2 = fixSqrt of the rounded dot, fixMulDiv truncating); it is kept as a
\* reversed witness, because it does not keep the impulse inside a small cap. The time step is
\* folded into the bounds: a cap here is the raw value of h * maxForce.
EXTENDS Integers, TLC

CONSTANTS
    F,            \* fraction bits of the reduced fixed-point format
    MaxRaw,       \* raw impulse domain is -MaxRaw..MaxRaw
    MaxCap,       \* raw caps are 0..MaxCap
    EnableSnapshot,
    ClampForm,    \* "exact" (the code) or "rounded" (the joint.h spelling, a reversed witness)
    Mutant        \* "none", "clampOnGrowth" or "restoreDropsImpulse"

One == 2^F
Half == 2^(F - 1)
Raw == -MaxRaw..MaxRaw
Caps == 0..MaxCap
Vel == -1..1          \* the pogo spring velocity is abstract: b3SpringDamper is not modelled

FloorDiv(a, b) == IF a >= 0 THEN a \div b ELSE -((-a + b - 1) \div b)
TruncDiv(a, b) == IF a >= 0 THEN a \div b ELSE -((-a) \div b)
Abs(a) == IF a < 0 THEN -a ELSE a
FixMul(a, b) == FloorDiv(a * b + Half, One)
ISqrt(n) == CHOOSE r \in 0..(n + 1) : r * r <= n /\ (r + 1) * (r + 1) > n
CeilSqrt(n) == LET r == ISqrt(n) IN IF r * r < n THEN r + 1 ELSE r
FixSqrt(a) == IF a <= 0 THEN 0 ELSE ISqrt(a * One)
LengthSquared(v) == v[1] * v[1] + v[2] * v[2]

\* ClampForm "rounded": b3ImpulseOverCap2 (src/joint.h), squares non-negative so >> is division
Dot2(v) == FixMul(v[1], v[1]) + FixMul(v[2], v[2])
OverCap2(v, m) ==
    ((v[1] * v[1] + Half) \div One) + ((v[2] * v[2] + Half) \div One) > ((m * m + Half) \div One)
RoundedLength(v) == FixSqrt(Dot2(v))

Fires(v, m) == IF ClampForm = "exact" THEN LengthSquared(v) > m * m ELSE OverCap2(v, m)
Length(v) == IF ClampForm = "exact" THEN CeilSqrt(LengthSquared(v)) ELSE RoundedLength(v)
\* exact: |x| * m / length truncated, sign restored; rounded: fixMulDiv, truncating toward zero
Scale(x, m, len) == TruncDiv(x * m, len)

\* b3SolveMoverJoint's clamp on the unclamped accumulated impulse v, with the previous one old.
\* The mutant only clamps an impulse that grew, so a cap lowered by a setter is never enforced.
MoverClamp(v, old, m) ==
    LET fire == IF Mutant = "clampOnGrowth" THEN Fires(v, m) /\ LengthSquared(v) > LengthSquared(old)
                ELSE Fires(v, m)
        len == Length(v)
    IN IF fire THEN <<Scale(v[1], m, len), Scale(v[2], m, len)>> ELSE v

\* b3SolvePogoJoint's clamp: b3FixClamp( impulse, -h * maxTension, h * maxCompression )
PogoClamp(p, t, c) == IF p < -t THEN -t ELSE IF p > c THEN c ELSE p

VARIABLES
    mImp, mCap,                 \* mover: accumulated 2D impulse, h * maxForce
    pImp, pVel, pTen, pCom, pHz, \* pogo: impulse, spring velocity, tension and compression caps, hertz # 0
    awake,                      \* the island's solver set: awake or sleeping
    phase, solved,              \* "idle" between steps, "step" inside one; a solve ran this step
    dirty,                      \* a setter or a pogo rebuild changed bounds or seeds since the last step
    lastSolve,                  \* which joint the previous action solved, or "none"
    snap, snapDirty, hasSnap,   \* the snapshot copy (joint state and the dirty flag), taken yet
    lastRestore                 \* the previous action restored the snapshot

jointVars == <<mImp, mCap, pImp, pVel, pTen, pCom, pHz, awake>>
snapVars == <<snap, snapDirty, hasSnap>>
vars == <<jointVars, phase, solved, dirty, lastSolve, snapVars, lastRestore>>

Current == [mImp |-> mImp, mCap |-> mCap, pImp |-> pImp, pVel |-> pVel,
            pTen |-> pTen, pCom |-> pCom, pHz |-> pHz, awake |-> awake]

TypeOK ==
    /\ mImp \in Raw \X Raw /\ mCap \in Caps
    /\ pImp \in Raw /\ pVel \in Vel /\ pTen \in Caps /\ pCom \in Caps /\ pHz \in {0, 1}
    /\ awake \in BOOLEAN /\ phase \in {"idle", "step"} /\ solved \in [{"mover", "pogo"} -> BOOLEAN]
    /\ dirty \in BOOLEAN /\ lastSolve \in {"none", "mover", "pogo"} /\ lastRestore \in BOOLEAN

\* The bound is the exact disc. The rounded form's own compare would be weaker than this, and the
\* point of the exact form is that the two agree.
MoverWithin == LengthSquared(mImp) <= mCap * mCap
PogoWithin == -pTen <= pImp /\ pImp <= pCom /\ (pHz = 0 => pImp = 0 /\ pVel = 0)

Init ==
    /\ mImp = <<0, 0>> /\ mCap \in Caps                       \* b3CreateMoverJoint zeroes the impulse
    /\ pImp \in Raw /\ pVel \in Vel /\ pTen \in Caps /\ pCom \in Caps /\ pHz \in {0, 1}
    /\ awake = TRUE /\ phase = "idle" /\ solved = [mover |-> FALSE, pogo |-> FALSE]
    /\ dirty = TRUE                                           \* the pogo def's seeds are unchecked
    /\ lastSolve = "none" /\ lastRestore = FALSE
    /\ snap = [mImp |-> <<0, 0>>, mCap |-> 0, pImp |-> 0, pVel |-> 0, pTen |-> 0, pCom |-> 0, pHz |-> 0,
               awake |-> TRUE]
    /\ snapDirty = TRUE /\ hasSnap = FALSE

Quiet == lastSolve' = "none" /\ lastRestore' = FALSE

\* Prepare (impulses zeroed when warm starting is off) and warm start (a zero-hertz pogo drops its
\* impulse). Only an awake island is stepped.
BeginStep(warm) ==
    /\ phase = "idle" /\ awake
    /\ phase' = "step" /\ solved' = [mover |-> FALSE, pogo |-> FALSE]
    /\ IF warm
       THEN /\ mImp' = mImp /\ pVel' = pVel
            /\ pImp' = IF pHz = 0 THEN 0 ELSE pImp
       ELSE /\ mImp' = <<0, 0>> /\ pImp' = 0 /\ pVel' = 0
    /\ Quiet
    /\ UNCHANGED <<mCap, pTen, pCom, pHz, awake, dirty, snapVars>>

\* One solve or relax pass of one joint. The two joints share no state, so their passes interleave
\* freely. The unclamped accumulated impulses are arbitrary (any velocity error); the clamp is what
\* is checked.
SolveMover ==
    /\ phase = "step"
    /\ \E vx, vy \in Raw : mImp' = MoverClamp(<<vx, vy>>, mImp, mCap)
    /\ solved' = [solved EXCEPT !.mover = TRUE] /\ lastSolve' = "mover" /\ lastRestore' = FALSE
    /\ UNCHANGED <<mCap, pImp, pVel, pTen, pCom, pHz, awake, phase, dirty, snapVars>>

SolvePogo ==
    /\ phase = "step"
    /\ IF pHz = 0
       THEN pImp' = 0 /\ pVel' = 0
       ELSE \E p \in Raw, v \in Vel : pImp' = PogoClamp(p, pTen, pCom) /\ pVel' = v
    /\ solved' = [solved EXCEPT !.pogo = TRUE] /\ lastSolve' = "pogo" /\ lastRestore' = FALSE
    /\ UNCHANGED <<mImp, mCap, pTen, pCom, pHz, awake, phase, dirty, snapVars>>

EndStep ==
    /\ phase = "step" /\ solved.mover /\ solved.pogo
    /\ phase' = "idle" /\ dirty' = FALSE
    /\ Quiet
    /\ UNCHANGED <<jointVars, solved, snapVars>>

\* Setters between steps. None of them touches the carried impulse.
SetMoverMaxForce(m) ==
    /\ phase = "idle" /\ mCap' = m /\ dirty' = TRUE /\ Quiet
    /\ UNCHANGED <<mImp, pImp, pVel, pTen, pCom, pHz, awake, phase, solved, snapVars>>
SetPogoForces(t, c) ==
    /\ phase = "idle" /\ pTen' = t /\ pCom' = c /\ dirty' = TRUE /\ Quiet
    /\ UNCHANGED <<mImp, mCap, pImp, pVel, pHz, awake, phase, solved, snapVars>>
SetPogoHertz(z) ==
    /\ phase = "idle" /\ pHz' = z /\ dirty' = TRUE /\ Quiet
    /\ UNCHANGED <<mImp, mCap, pImp, pVel, pTen, pCom, awake, phase, solved, snapVars>>

\* The dynamic mover destroys its pogo and creates a new one each frame, seeding the impulse and
\* velocity from the old one (or anything the caller chooses).
RebuildPogo ==
    /\ phase = "idle"
    /\ \E i \in Raw, v \in Vel, t, c \in Caps, z \in {0, 1} :
        pImp' = i /\ pVel' = v /\ pTen' = t /\ pCom' = c /\ pHz' = z
    /\ dirty' = TRUE /\ Quiet
    /\ UNCHANGED <<mImp, mCap, awake, phase, solved, snapVars>>

Sleep ==
    /\ phase = "idle" /\ awake /\ ~dirty
    /\ awake' = FALSE /\ Quiet
    /\ UNCHANGED <<mImp, mCap, pImp, pVel, pTen, pCom, pHz, phase, solved, dirty, snapVars>>
Wake ==
    /\ phase = "idle" /\ ~awake
    /\ awake' = TRUE /\ Quiet
    /\ UNCHANGED <<mImp, mCap, pImp, pVel, pTen, pCom, pHz, phase, solved, dirty, snapVars>>

Snapshot ==
    /\ EnableSnapshot /\ phase = "idle"
    /\ snap' = Current /\ snapDirty' = dirty /\ hasSnap' = TRUE
    /\ Quiet
    /\ UNCHANGED <<jointVars, phase, solved, dirty>>

Restore ==
    /\ EnableSnapshot /\ phase = "idle" /\ hasSnap
    /\ mImp' = IF Mutant = "restoreDropsImpulse" THEN <<0, 0>> ELSE snap.mImp
    /\ pImp' = IF Mutant = "restoreDropsImpulse" THEN 0 ELSE snap.pImp
    /\ pVel' = snap.pVel /\ mCap' = snap.mCap /\ pTen' = snap.pTen /\ pCom' = snap.pCom
    /\ pHz' = snap.pHz /\ awake' = snap.awake /\ dirty' = snapDirty
    /\ lastSolve' = "none" /\ lastRestore' = TRUE
    /\ UNCHANGED <<phase, solved, snap, snapDirty, hasSnap>>

Next ==
    \/ \E w \in BOOLEAN : BeginStep(w)
    \/ SolveMover
    \/ SolvePogo
    \/ EndStep
    \/ \E m \in Caps : SetMoverMaxForce(m)
    \/ \E t, c \in Caps : SetPogoForces(t, c)
    \/ \E z \in {0, 1} : SetPogoHertz(z)
    \/ RebuildPogo
    \/ Sleep
    \/ Wake
    \/ Snapshot
    \/ Restore

Spec == Init /\ [][Next]_vars

\* Every solve leaves both impulses inside the bounds in force, by the code's own compare.
ImpulseWithinBoundAfterSolve ==
    /\ lastSolve = "mover" => MoverWithin
    /\ lastSolve = "pogo" => PogoWithin
\* Between steps, once a step has run since the last change, the carried state is inside the bounds,
\* and sleep, wake and snapshot do not move it out.
ImpulseWithinBoundAtRest == (phase = "idle" /\ ~dirty) => MoverWithin /\ PogoWithin
\* A restore reproduces the snapshot exactly.
RestoreExact == lastRestore =>
    /\ mImp = snap.mImp /\ pImp = snap.pImp /\ pVel = snap.pVel /\ mCap = snap.mCap
    /\ pTen = snap.pTen /\ pCom = snap.pCom /\ pHz = snap.pHz /\ awake = snap.awake
\* The divide is safe: whenever the mover clamp fires, the length is positive, and (exact form)
\* at least each component, so every quotient is at most the cap. Constant-level, checked once.
ASSUME ClampDivisorPositive == \A m \in Caps, vx, vy \in Raw :
    Fires(<<vx, vy>>, m) =>
        /\ Length(<<vx, vy>>) > 0
        /\ ClampForm = "exact" => Length(<<vx, vy>>) >= Abs(vx) /\ Length(<<vx, vy>>) >= Abs(vy)
=============================================================================
