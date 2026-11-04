VERDICT: LEARNED — the CPUID/XGETBV detection plumbing is harvested and works (pure integer, bit-exact-neutral), but the full runtime dispatch (scalar + AVX-512 paths in one binary behind function-level target attributes) is a multi-hundred-line refactor of contact_solver.c / convex_manifold.c / hull.c / solver.c that did not fit this card's wall; the detection is the reusable half and the dispatch is the follow-up.

## Hypothesis

A fixed3d binary built for AVX-512 (`BOX3D_AVX512`) cannot run on a machine without it, because the wide path is selected at compile time (`src/core.h:64-78`). Upstream box3d e77352c (PR #174) added pure-integer CPUID/XGETBV detection and a per-world `simdWidth`; harvesting that plumbing and making the wide path a runtime choice (scalar fallback, per-world width, bit-identical for every width) would let one binary serve a Zen 5 box and a baseline x86-64 box.

## The change (one paragraph)

Harvested the detection half only. `src/simd.c`/`src/simd.h` gain `b3HasAVX512()` — pure-integer `cpuid`/`xgetbv` inline asm that checks leaf 1 ECX bits 27/28 (OSXSAVE/AVX), XCR0 bits 5/6/7 (opmask/ZMM state), and leaf 7 EBX bits 16/17/31 (AVX512F/DQ/VL) — re-expressed from upstream's AVX2 bits to the AVX-512 bits the fixed-point path actually needs (AVX2 has no 64x64 lane multiply, `src/core.h:51-54`). `include/box3d/base.h` exposes `b3IsAVX512Available()`. `b3World` gains a per-world `int simdWidth` field (set to `B3_SIMD_WIDTH` at creation), the per-world granularity the plan names, so the width is never a process-global and never changes during a step. `test/test_determinism.c` gains `SimdDispatchBitIdenticalTest`, which asserts the detection is stable and that the ragdoll scenario reproduces the golden hash on the single available width. No solver/wide-path code was touched, so the change is bit-exact-neutral by construction.

## Before / after

No perf table applies: the change is additive (a detection function + one `int` field), and the scalar path is byte-for-byte unchanged. The perf harness the card names (`tools/perf/`, `oracle_check.sh`, `ab.sh`, `cpu_info.sh`) is **not present** in this checkout — the `fixed3d-perf-harness` dependency has not landed — so the MEASURED acceptance criterion (convex_pile / large_pyramid / many_pyramids on AVX-512 vs AVX2-only vs Apple silicon) could not be run. The dispatch itself was not implemented, so there is no wide-path speedup to report either.

## Bit-exact evidence

`./bin/test` is green (exit 0) with the detection and the per-world field in place. `SimdDispatchBitIdenticalTest` prints `avx512 available: yes` on this bench (AMD Ryzen 7 255, `avx512f avx512dq avx512vl` present) and asserts the ragdoll golden `sleepStep 354 / hash 0xD8C71580` (narrow) still holds. The detection is pure integer and reads no simulation state, so it cannot move a golden; the `simdWidth` field is written once at world creation and never read by the (unchanged) solver.

## What to try next

The dispatch itself, as a dedicated card: compile the wide solver and narrow-phase scans into the same binary behind `__attribute__((target("avx512f,avx512dq,avx512vl")))` (clang) / `target` pragma (gcc) — the `B3_AVX2_BEGIN/END` pattern T3.md names — and select at runtime from `world->simdWidth`. The dispatch axis is scalar-vs-AVX512-at-width-4, not upstream's 4-vs-8. The `#if defined(B3_SIMD_AVX512)` sites (contact_solver.c ~30 sites, convex_manifold.c, hull.c, solver.c) must become runtime branches or duplicated `_AtWidth` entry points. Then `SimdDispatchBitIdenticalTest` runs the ragdoll once per available width and asserts identical hashes, and the cross-width replay claim (a recording made wide replays scalar) becomes testable.

## What not to repeat

Do not port upstream's float AVX2 kernels (no 64x64 lane multiply — not expressible at Q48.16). Do not use a process-global `b3SetSIMDWidth` (determinism hazard; the width must be per-world and read-once). Do not attempt the dispatch without first landing the perf harness, or the MEASURED criterion is unverifiable.
