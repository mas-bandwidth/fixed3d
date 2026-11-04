VERDICT: LEARNED — exact AVX2 lane multiply, but only 1.1484× median micro speedup on AVX2-only Zen+; below the 1.5× gate.

The hypothesis was that four Q48.16 lanes in a `__m256i` could recover enough throughput on AVX2-only consumer CPUs to justify a second wide contact-solver path. The experiment reconstructed bits 16–79 of each signed 64×64 product with four `vpmuludq` 32×32 products, shifts, adds, and a high-word sign correction, retaining the scalar round-half-up and wrap semantics. The archived patch contains this lane primitive, the standalone micro driver, and an opt-in `Avx2LaneMulEquivalenceTest` registration. It does **not** wire the primitive into the solver.

| Zen+ micro, cache-resident | scalar `b3FixMul` | AVX2 four lanes | scalar / AVX2 |
|---|---:|---:|---:|
| Minimum ns per multiply | 1.1695 | 1.0171 | — |
| Median ns per multiply | 1.2042 | 1.0486 | **1.1484×** |
| Seven-pass spread, ns | 0.1096 | 0.1605 | — |

The differential compared every AVX2 lane with the actual scalar `b3FixMul` over 25,000,000 full-range int64 pairs, including 225 edge combinations: zero mismatches. Seven interleaved scalar/AVX2 timings used GCC 15.2, `-O2 -g -DNDEBUG -flto -mavx2 -ffp-contract=off`, one process on vision's Threadripper 2990WX (AVX2 present, AVX-512F/DQ/VL absent), no explicit CPU pinning (affinity 0–63), and load 0.48/0.52/0.79 before and after. Worker count is one microbenchmark process; no game-world workers were run. The exact commands, pass values, compiler, source hashes, and load are in `perf/experiments/spike-avx2-wide-lanes-feasibility/raw.txt`.

The card's Step 1 says to stop unless the micro multiply is at least 1.5× faster. It did not clear that gate, so I did not build the AVX2 solver, run the three-scene interleaved harness, compare all scene/worker/mode hashes to the scalar oracle, run `./bin/test`, or measure Zen 4. The micro proves arithmetic equivalence for its sampled pairs, **not** full-engine bit-exactness. The keep rule is therefore unmet; no code should be adopted from this patch.

A next experiment would need a materially different way to amortize AVX2's missing 64×64 lane multiply, with a reason to expect a ≥1.5× primitive gain before touching the solver. Do not retune this same four-product decomposition or revisit the rejected int32/Q16.16 narrowing, NEON emulation, or equilibrium-sensitive rounding/fusion changes without new range and performance evidence.
