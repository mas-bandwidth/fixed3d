VERDICT: ADOPTABLE (bit-identical across gcc and clang, narrow and ludicrous, workers 1-5 and two runs; JointTest green in all four builds; only the falling-ragdoll golden moves)

Spike f3dup-0cfc475-revolute-axial-merge-spike, attempt 3, 2026-10-17, on amd-3 (gcc 15.2.0 /
clang 21.1.8, RelWithDebInfo + VALIDATE + Werror), BASE sprint/fixed3d-box3d-ports at adbe7ec.
This card ports ONLY upstream 0cfc475's merged axial solve in `b3SolveRevoluteJoint` and lands
no engine change: the port is experiment.patch (`git apply --check` clean on BASE), the
measurement is raw.txt.

## The change

In src/revolute_joint.c the spring, motor and limit (lower and upper) solves are merged into one
block gated on `fixedRotation == false && (enableSpring || enableMotor || enableLimit)`. The
axis response is precomputed once: `gA = b3MulMV(iA, axis)`, `gB = b3MulMV(iB, axis)`,
`k = b3Dot(axis, gA + gB)`. One running `axialCdot = b3Dot(wB - wA, axis)` is kept; after the
spring, motor and lower-limit sub-solves it is advanced by `axialCdot += b3InvMul(k, deltaImpulse)`
and the impulse is accumulated in `axialImpulse` (the upper limit subtracts its impulse and,
as upstream, does not advance the cdot because it is last). The accumulated impulse is applied
once at the end: `wA -= b3InvMulSV(axialImpulse, gA)`, `wB += b3InvMulSV(axialImpulse, gB)`.
The motor clamp becomes upstream's `clamp(delta, -max - old, max - old)` and the limits become
`max(delta, -old)`; both are exact in integers, so they equal the old `clamp/max then subtract`
bit for bit. Products use `b3FixMul`, projections the raw-128 `b3Dot`. The collinearity and
point-to-point solves (`b3SkewSandwich` + `b3Solve3AcrossScales`), the other joint files, the
wide contact solver and `b3Skew` are untouched.

## The bug attempt 2 hit, and its fix

Attempt 2 (e9fa4ec) advanced the cdot with `b3FixMul(k, deltaImpulse)`. `k` is an INVERSE
quantity: `iA`/`iB` live at the inverse scale (2^40, B3_INVERSE_FRACTION_BITS), `b3MulMV` keeps
that scale, and the prepare step turns the same `k` into `axialMass` with `b3InvReciprocal`.
`b3FixMul` shifts by 16 instead of 40, so every cdot update was 2^24 too large. That is why
attempt 2 saw ragdoll-human at sleepStep 0 / hash 0 (nothing ever settled) and why it was not a
golden shift. `b3InvMul(k, deltaImpulse)` (inverse times ordinary gives ordinary, round half up
at 128 bits) is the correct primitive; with it the ragdoll-human scene is unchanged.

## What moved (DeterminismTest, printed values with the goldens neutralized, then reverted)

| scenario | field | old narrow | new narrow | old ludicrous | new ludicrous | gcc == clang | run 1 == run 2 |
|---|---|---|---|---|---|---|---|
| ragdoll (MultithreadingTest, CrossPlatformTest) | sleepStep | 408 | **354** | 408 | **354** | yes | yes |
| ragdoll | hash | 0x7D29E209 | **0xD9216BE4** | 0x69572D49 | **0x27D948A4** | yes | yes |
| ragdoll-human | sleepStep / hash | 219 / 0xD945B8B3 | unchanged | 219 / 0x407D12F3 | unchanged | yes | yes |
| wave pile | sleepStep / hash | 335 / 0x64609ED9 | unchanged | 335 / 0x22E6F859 | unchanged | yes | yes |
| query spawn | sleepStep / hash / hits / queryHash | 243 / 0x250D3BFA / 59 / 0xE583B246 | unchanged | 243 / 0xC3538B7A | unchanged | yes | yes |
| mesh drop | sleepStep / hash | 284 / 0x784AD80A | unchanged | 284 / 0xE44E6A0E | unchanged | yes | yes |
| spherical geometry | hash scale 0 / 1 | 0xccf165f88a73ebb7 / 0x2add50ab7852c3ce | unchanged | 0xee343d0d530f22f1 / 0x46256d278ff09af2 | unchanged | yes | yes |

The ragdoll values are identical for workers 1-5 within each build. Query spawn prints only on a
mismatch and printed nothing, so all four of its pins hold. PerfStateHashGoldenTest exits 0 in
all four builds (the state-hash golden does not move). JointTest exits 0 in all four builds.

With the patch and the real goldens, `./bin/test DeterminismTest` exits 1 in all four builds at
exactly one row of this table: `condition false: data.sleepStep == 408` in MultithreadingTest
(RagdollHumanTest passes first, as the table says).

## What the design step must decide

1. Adopt the merge and re-pin RAGDOLL_SLEEP_STEP 408 -> 354 and RAGDOLL_HASH (narrow
   0x7D29E209 -> 0xD9216BE4, ludicrous 0x69572D49 -> 0x27D948A4). The falling ragdolls
   (shared/human.c revolute joints: limit and motor enabled, spring when hertz > 0) are the
   only scenario in this corpus whose goldens move, so the blast radius is that one golden; it
   settles sooner (354 vs 408), not later.
2. The running cdot is an approximation of the true `dot(wB - wA, axis)`: one `b3InvMul` of the
   summed `k` versus the per-body `b3InvMulSV` applications, which can differ by a raw ULP. That
   is upstream's structure (upstream also skips recomputing the cdot); an exact variant would
   advance the cdot from the two per-body applications instead. Bless one.
3. The upper limit now reads `-axialCdot` where it read `dot(wA - wB, axis)`. Negation of a
   b3Dot result is exact, so this one is not a rounding change beyond item 2.
4. Whether to land it together with the spherical merge (f3dup-0cfc475-spherical-axial-merge-spike,
   which moves ragdoll, ragdoll-human and spherical geometry) as one golden re-pin, since both
   move the ragdoll golden and their combined values are not measured here.

## What not to repeat

- Never advance an ordinary velocity by `b3FixMul` of an inverse-scaled quantity: inverse
  inertia and anything built from it by `b3MulMV`/`b3Dot` stays at 2^40 and needs `b3InvMul`
  / `b3InvMulSV` / `b3InvMulMV`. The symptom is "nothing settles" (sleepStep 0, hash 0), not a
  moved golden.
- DeterminismTest aborts at the first failing subtest; harvest with ENSURE_GOLDEN neutralized
  and the printf guards forced, then revert. Do not edit the goldens in the patch.
- Do not commit the engine change on a spike branch; only the three NEW files land.

## Not done

- No engine change and no golden update committed (spike contract; PATHS hold only the three
  NEW files).
- The combined golden values with the spherical merge applied as well were not measured.
- No perf timing, and no per-hunk attribution of the ragdoll move (spring vs motor vs limit):
  the merge is one structure and the move is a single bit-identical value in every build.
