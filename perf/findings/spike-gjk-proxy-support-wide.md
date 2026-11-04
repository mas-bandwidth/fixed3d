VERDICT: LEARNED — the first AVX-512 run gained only 0.597% geomean on the target scene/worker rows, below the 3% keep rule; NEON and the required green full-suite/frozen-oracle gates remain unproved. Do not adopt the patch.

# GJK proxy support: four rounded dots at once

## Hypothesis and change

Hull-sphere and hull-capsule GJK calls `b3GetProxySupport`, which scans vertices with a rounded, origin-shifted `b3Dot`. Four exact lane dots might improve Rain, junkyard, and trees50 without changing the selected index. The experimental `src/distance.c` path checks a conservative dot bound, then uses AVX-512 64-bit lane products or NEON gated int32 widening products. Both round each sum once before comparing and preserve the first index on ties. Counts below eight use the existing scalar loop. The test adds 100,000 seeded directions over hull counts 8, 20, 64, and 128, large common translations, a fallback axis, and a discriminating tie where raw dots 32768 and 65536 both round to one quantum. `test_distance.c` is already listed in `test/CMakeLists.txt`; `RUN_SUBTEST( ProxySupportWideEquivalenceTest )` registers the new case in the existing `DistanceTest` suite.

## Method and first measurement

Exact base: `59eff1c450f7c6d308f73d8a88c633f42e160ecb`. The unchanged support scan plus the new test was build A; the patched scan was build B. Both used GCC 13.3.0, RelWithDebInfo, default LTO, validation and warning-as-error ON, benchmarks ON, and `BOX3D_AVX512=ON`. Hetzner2 is an AMD Ryzen 9 7950X3D with AVX-512F/DQ/VL/IFMA. Affinity was CPUs 0–31 (`taskset -cp $$`); load averages before/after were 1.07/1.29/1.67 and 1.81/1.48/1.57. No competing benchmark process was present. Runs used `nice -n 10`, two build workers, a private TMPDIR, 120 steps, workers 1/2/4, and seven interleaved A/B repeats. A direct `-w=4` probe printed `thread count: 4`; the harness passed no `-t=1`. The red-first allowance was the unchanged-base A/A Rain baseline table (step speedup 0.999535), because the equivalence test correctly passes on the scalar base rather than failing there.

Step time in ns; each triplet is **min / median / spread** across seven runs. Speedup is A median / B median. The full per-scene, per-phase A/B table, including every phase's min, median, and spread, is in [`raw.txt`](../experiments/spike-gjk-proxy-support-wide/raw.txt).

| Scene | Workers | Base step min / med / spread | Candidate step min / med / spread | Speedup |
|---|---:|---:|---:|---:|
| rain | 1 | 5,129,643 / 5,253,650 / 369,247 | 5,151,238 / 5,262,553 / 176,530 | 0.9983× |
| rain | 2 | 2,827,552 / 2,851,060 / 177,755 | 2,814,458 / 2,904,617 / 127,158 | 0.9816× |
| rain | 4 | 1,615,967 / 1,718,500 / 126,773 | 1,576,088 / 1,664,147 / 261,536 | 1.0327× |
| junkyard | 1 | 8,963,939 / 9,018,789 / 187,520 | 9,043,502 / 9,112,181 / 226,580 | 0.9898× |
| junkyard | 2 | 4,744,915 / 4,771,527 / 2,033,677 | 4,792,240 / 4,837,460 / 4,366,906 | 0.9864× |
| junkyard | 4 | 2,653,979 / 2,716,113 / 202,414 | 2,656,512 / 2,681,593 / 194,006 | 1.0129× |
| trees50 | 1 | 899,782 / 902,575 / 41,639 | 899,709 / 901,565 / 7,746 | 1.0011× |
| trees50 | 2 | 548,074 / 619,094 / 154,698 | 546,330 / 607,198 / 148,560 | 1.0196× |
| trees50 | 4 | 379,825 / 485,423 / 122,956 | 408,198 / 469,945 / 101,467 | 1.0329× |

The harness geomean across these nine step rows was **1.00597×**. The target `collide` phase ranged from 0.9617× to 1.0520×; the best `solve` row was 1.0308×. No target phase approached the alternate 1.5× rule. The apparent 4-worker gains do not overcome the total geomean threshold, and the first complete run ended the experiment without tuning.

## Exactness and gate limits

The focused `ProxySupportWideEquivalenceTest` passed on the narrow base and the AVX-512 candidate in both narrow and ludicrous configurations. The A/B harness reported no target hash mismatch. A separate direct matrix compared **all 78 hashes** from base and candidate SIMD binaries: 13 scenes × narrow/ludicrous × workers 1/2/4; **78/78 matched**. This direct comparison is needed because `tools/perf/oracle_check.sh` hard-codes both SIMD options OFF.

The frozen scalar oracle is stale at this exact base for 18 rows: Rain (`d3e59885dbca43e1` frozen versus `f96c9e008d59a375` live), ragdoll_pile (`4ac74cd9f1805daf` versus `42fa9c6438e3527f`), and bodies_20k (`3e7cbde301a4f6c2` versus `ed59e6aff81bcd26`), each in both modes and all three worker counts. Base and candidate produce the same live values. `oracle_check.sh --scenes=rain` failed with that same mismatch on both the unchanged base and candidate; no oracle was regenerated.

Unfiltered `./bin/test` stopped on unchanged `test/test_allocator.c` at `AllocatorWidenedSignatureTest` (`condition false: !g_alloc.badAlignment`) in both narrow and ludicrous builds. Base and candidate logs were byte-identical. Running the 24 other registered suites individually passed in both modes, including `DistanceTest` and the hash/determinism suites. The required unfiltered green suite is therefore **not claimed**. No aarch64 Linux bench with the required CMake toolchain was allocated; NEON timing, unit, and oracle gates were **not run**. Cross-ISA acceptance is not claimed.

## Next and avoid

Reconcile the frozen scalar oracle and base allocator test in separate stream work, then measure a new support-specific idea only if profiling shows this scan is material. A future candidate could avoid the per-call admissibility pre-scan with validated proxy-local bounds, but that changes the proxy contract and needs a new card. Do not repeat this pre-scan plus four-lane implementation or tune the first-run result toward the threshold: it did not meet the keep rule.
