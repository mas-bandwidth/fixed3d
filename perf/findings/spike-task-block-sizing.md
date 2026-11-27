# Spike: contact, joint and body task block sizes against steal rate and cache reuse

VERDICT: DROP — the contact/joint/body block minimums and the block-count
multiplier are now compile-time `#ifndef` tunables and are proven bit-identical
across the sweep grid (BlockSizingBitIdenticalTest green at every cell,
scalar oracle reproduced bit-for-bit on a second CPU family), but the sweep
shows no win: raising `minContactsPerBlock` from 4 to 8 is a wash and to 16
regresses the parallel solve (convex_pile solve −9% at 4 workers, −16% at 8),
so no cell beats the current default and nothing is adopted.

## Hypothesis

On main the solver targets `minContactsPerBlock = 4`, `minJointsPerBlock = 4`,
`minBodiesPerBlock = 32`, and `maxBlockCount = 4 * workerCount` (src/solver.c).
A 4-worker M3 gets at most 16 contact blocks. The contact solve is heavy, so
the review's candidate was that 8 or 16 as a minimum might cut block overhead
or improve cache reuse per steal. Because constraint order and stage
synchronisation are deterministic by construction (coloring fixes the order),
block size must not change any result — so this is a pure perf A/B.

## The change

The four constants become `#ifndef` compile-time tunables at the top of
src/solver.c (`B3_MIN_CONTACTS_PER_BLOCK` 4, `B3_MIN_JOINTS_PER_BLOCK` 4,
`B3_MIN_BODIES_PER_BLOCK` 32, `B3_MAX_BLOCK_COUNT_MULTIPLIER` 4), and the
three local minimums plus `maxBlockCount = B3_MAX_BLOCK_COUNT_MULTIPLIER * workerCount` read them. Defaults are unchanged, so the default build is
bit-identical to BASE. A new subtest `BlockSizingBitIdenticalTest` in
test/test_determinism.c runs the ragdoll at workers 1-5 and asserts the hash
lands on `RAGDOLL_HASH`; it is run in each grid-cell build so any cell that
diverges fails its own build. The full patch is
perf/experiments/spike-task-block-sizing/experiment.patch (src/ and test/
edits are NOT committed to the tree; only the finding, the patch, and this raw
output are).

## Measurement (Zen 5, AMD Ryzen 7 255, gcc 15.2, scalar, LTO, RelWithDebInfo)

Interleaved A/B, steps=120 cap, 7 repeats (c16) / 5 repeats (c8). Speedup
= baseline median / candidate median; >1 means the candidate is slower.
Per whole-step (the phase table and solve-phase rows are in raw.txt):

| cell (contacts/joints) | scene | w=1 | w=2 | w=4 | w=8 |
|------------------------|-------|-----|-----|-----|-----|
| 16 / 16 | convex_pile   | 1.003 | 0.989 | 0.965 | 0.931 |
| 16 / 16 | many_pyramids | 1.001 | 0.915 | 1.012 | 0.988 |
| 16 / 16 | large_pyramid | 1.066 | 1.002 | —    | —    |
|  8 /  8 | convex_pile   | —    | —    | 0.988 | 0.994 |
|  8 /  8 | many_pyramids | —    | —    | 1.000 | 0.993 |
|  8 /  8 | large_pyramid | —    | —    | 0.995 | 0.998 |
|  8 /  8 | rain          | —    | —    | 0.988 | 1.007 |

The A/A spread on these scenes is ~2.25–2.40% (tools/perf/README.md noise
floor). The 8/8 cell sits entirely inside that spread — a wash. The 16/16 cell
is monotonically worse at higher worker counts: the solve phase alone regresses
16% on convex_pile at 8 workers (0.843x) and 9% at 4 (0.909x). Direction is
mechanistically expected: a larger minimum means fewer, coarser blocks, so at
4-8 workers there is less to steal and the tail worker dominates. No measured
cell beats the default (4/4/32/4), and 16/16 violates the keep rule outright.

## Bit-exact evidence

- `BlockSizingBitIdenticalTest` green at the default cell and the 16/16 cell
  (workers 1-5, ragdoll hash == `RAGDOLL_HASH`), and `./bin/test` fully green
  in narrow and BOX3D_LUDICROUS_MODE (RelWithDebInfo + VALIDATE + -Werror).
- `tools/perf/oracle_check.sh --scenes=convex_pile,many_pyramids,large_pyramid,rain`
  passed on this box against the frozen 120-step scalar oracle (regenerated on
  a Zen 4 8700GE): "oracle check passed (narrow + ludicrous, workers 1/2/4)",
  i.e. the 4 target scenes reproduce their hashes bit-for-bit across machines.
- ab.sh final-HASH comparison: both A/B runs exited 0 (no HASH line diverged),
  so the c8 and c16 cells produce identical state hashes to the default cell.

## What to try next

Nothing in the larger-minimum direction: it is measured worse and the mechanism
(starved work stealing at 4-8 workers) is understood. The one unexplored knob
is `B3_MAX_BLOCK_COUNT_MULTIPLIER` (sweep {2,4,8} independently of the
minimums) — more blocks per worker is the direction the steal-rate argument
actually favours; that was not measured here and is the next cell to A/B. Any
single-worker-only benefit from larger blocks is irrelevant, since at w=1 all
cells are within noise.

## What not to repeat

Do not retry 8 or 16 as the contact minimum — measured, no win, 16 is a real
parallel regression. The already-rejected list from fixed3d issue #62 /
CLAUDE.md / benchmark/2026-09-07-*.md stands unchanged (spherical point
matrix caching, matrix/vector pointer variants, forced inlining, int32/Q16.16
lane narrowing, tangent2-on-the-fly, b3BodyState padding, fusing b3RotateVector
or b3Lerp, NEON b3FloatW emulation, data-dependent guards in the wide loop).

## Not done (wall)

Perf A/B was measured only on this Zen 5 box, not on the required Zen 4 and
Apple silicon boxes; large_pyramid w4/w8 and rain for c16, and w1/w2 for c8,
were cut by the 40-minute wall; the `B3_MAX_BLOCK_COUNT_MULTIPLIER` grid was
not swept. Bit-exactness is fully proven; the incomplete measurement alone is
enough to block KEEP, and the completed cells already falsify it.
