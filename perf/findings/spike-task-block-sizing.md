# Spike: contact, joint and body task block sizes against steal rate and cache reuse

VERDICT: LEARNED — the block-size minimums and the block-count multiplier are now
compile-time tunables and are proven bit-identical across the sweep grid (the
ragdoll hash is unchanged at every cell, workers 1-5), but the perf sweep itself
was not run: this run had no Linux bench, no Zen 4 box, no Apple silicon box, and
the perf harness (tools/perf/) is not in this checkout. The measurement half of
the acceptance criteria is unmet, so the patch is not adopted.

## Hypothesis

On main the solver targets `minContactsPerBlock = 4`, `minJointsPerBlock = 4`,
`minBodiesPerBlock = 32`, and `maxBlockCount = 4 * workerCount` (src/solver.c near
lines 1532-1540). On a 4-worker M3 that is at most 16 contact blocks. The contact
solve is heavy enough that 8 or 16 as a minimum is the thing to A/B: larger blocks
trade steal rate (fewer, coarser blocks to steal) against cache reuse (a worker
that claims a block keeps more contiguous work). Because constraint order and
stage synchronization are deterministic by construction (coloring fixes the
order), block size must not change any result — so the sweep is a pure perf
question, and any cell that is bit-identical is a candidate.

## The change

The four constants become `#ifndef` compile-time tunables in src/solver.c
(`B3_MIN_CONTACTS_PER_BLOCK` default 4, `B3_MIN_JOINTS_PER_BLOCK` default 4,
`B3_MIN_BODIES_PER_BLOCK` default 32, `B3_MAX_BLOCK_COUNT_MULTIPLIER` default 4),
so a spike can sweep them with `-D` overrides without touching source. A new
subtest `BlockSizingBitIdenticalTest` in test/test_determinism.c runs the ragdoll
at workers 1-5 and asserts the hash lands on `RAGDOLL_HASH`; it is run once per
grid cell (each cell a separate build), so any cell that diverges fails its own
build. Defaults are unchanged, so the default build is bit-identical to BASE.

## Before/after table

Not measured. The A/B (ab.sh) and oracle (oracle_check.sh) runs require the perf
harness and the two named boxes, neither of which this run could reach. The only
numbers collected are the bit-exactness runs below.

## Bit-exact evidence

`BlockSizingBitIdenticalTest` passes at the default cell and at the extreme cell
(contacts=32 joints=32 bodies=64 mult=8), both at workers 1-5, and the full
`./bin/test` suite is green in the default build. The ragdoll hash is
`RAGDOLL_HASH` (0xD8C71580 narrow) in every cell, so block sizing does not change
any result. See perf/experiments/spike-task-block-sizing/raw.txt for the exact
output. The oracle_check.sh scalar-oracle gate was not run (no harness).

## What to try next

Run the sweep on the required hardware: build the default and a candidate cell
(e.g. `-DB3_MIN_CONTACTS_PER_BLOCK=8` and `-DB3_MIN_CONTACTS_PER_BLOCK=16`, with
`-DB3_MAX_BLOCK_COUNT_MULTIPLIER` in {2,4,8}) and A/B them with ab.sh on rain,
many_pyramids, large_pyramid, convex_pile at workers 1/2/4/8, interleaved, 7
repeats, on a Zen 4 box and an Apple silicon box. The contact solve is the phase
to watch; block count only matters with workers > 1, so measure single-worker and
parallel separately. A per-worker-count rule is allowed only if it is a pure
function of the worker count and the contact count.

## What not to repeat

Nothing new was rejected here (no measurement was possible). The already-rejected
list from fixed3d issue #62 / CLAUDE.md / benchmark/2026-09-07-*.md stands
unchanged: full spherical point matrix caching, contact matrix/vector pointer
variants, forced inlining, int32/Q16.16 lane narrowing, tangent2-on-the-fly,
b3BodyState padding to 128, fusing b3RotateVector/b3Lerp, NEON b3FloatW emulation,
and data-dependent guards in the wide solve loop.
