VERDICT: LEARNED — the M5 Pro exposes SME2 and 512-bit streaming vectors, but an exact Q48.16 lane multiply is slower than scalar on the first measured micro. The card's 1.5× threshold was not met, so no solver path was built or adopted.

# Apple M5 Pro streaming-SVE feasibility

## Hypothesis and change

The Apple M5 Pro might use 64-bit SVE lanes in SME streaming mode to accelerate the four-lane `b3FloatW` multiply that currently stays scalar on ARM. The probe implements the default wrapping `b3FixMul` arithmetic in SVE lanes: multiply the signed high portion of one operand, multiply two low-part terms, add the half-ULP, then shift and combine. This preserves the source's 128-bit product and round-half-up behavior; a plain 64-bit vector `MUL` alone would lose required high bits. The probe compares this with native `__int128` scalar multiplication and scalar multiplication of values loaded and stored through NEON registers. It times both multiply and multiply-add, with streaming entry/exit on every call region, and a variant that creates a fresh ZA scope. It is an exploratory program, not a solver change.

The actual bench was `mini-m5-1` (`Glenns-Mac-Mini.local`), Mac mini with Apple M5 Pro, 18 cores, Darwin arm64. `sysctl` returned `FEAT_SME=1`, `FEAT_SME2=1`, `FEAT_SME2p1=1`, `FEAT_SME_I16I64=1`, and `hw.optional.arm.sme_max_svl_b=64` (eight 64-bit lanes per streaming vector). `FEAT_SME_I16I64` denotes 16-bit source products accumulated into 64-bit ZA elements; it is not a 64×64→128 lane multiply. [Arm's SME instruction description](https://developer.arm.com/community/arm-community-blogs/b/architectures-and-processors-blog/posts/arm-scalable-matrix-extension-introduction-p2) and [streaming-mode overview](https://developer.arm.com/community/arm-community-blogs/b/architectures-and-processors-blog/posts/arm-scalable-matrix-extension-introduction) support those distinctions. Apple clang 21.0.0 accepted `-march=armv9-a+sme2` in a dry run, but applying it globally emitted a non-streaming `cntd` in `main` that trapped SIGILL. The measured build instead uses default target flags and `target("+sme2")` only on a `__arm_locally_streaming` function. [Clang's SME attribute documentation](https://clang.llvm.org/docs/AttributeReference.html) describes its automatic mode transitions.

## First measurement

Apple clang 21, `-O3 -Wall -Wextra -Werror`, `nice -n 10`. Each row is the median ns per lane operation from seven interleaved scalar/NEON/SME/SME+ZA runs. Each run performs 1.6 million lane operations. The 8-lane call is already twice the solver's four-lane width; larger regions deliberately amortize mode switching more generously. `min`, `max`, checksums, and every run are in `raw.txt`.

| Operation | Lanes per call | Scalar | NEON emulation | SME | SME + fresh ZA | SME / scalar time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Multiply | 8 | 0.400 | 0.748 | 3.780 | 3.658 | 9.45× slower |
| Multiply | 32 | 0.354 | 0.307 | 1.857 | 1.860 | 5.25× slower |
| Multiply | 128 | 0.375 | 0.297 | 2.581 | 2.583 | 6.88× slower |
| Multiply | 512 | 0.358 | 0.304 | 3.168 | 3.173 | 8.85× slower |
| Multiply-add | 8 | 0.432 | 0.742 | 3.840 | 3.840 | 8.89× slower |
| Multiply-add | 32 | 0.388 | 0.351 | 1.974 | 1.976 | 5.09× slower |
| Multiply-add | 128 | 0.409 | 0.343 | 2.990 | 2.999 | 7.31× slower |
| Multiply-add | 512 | 0.392 | 0.332 | 3.432 | 3.448 | 8.76× slower |

The fresh-ZA version's disassembly contains `smstart sm`, a `TPIDR2_EL0` lazy-save check, `smstart za`, and `zero {za}`. No caller had live ZA state, so this measures a new ZA scope and possible lazy-save check, not the cost of forcing a full save of an existing tile. Its median is close to the no-ZA version at these sizes; neither changes the threshold decision. Load averages were 1.53/1.52/1.46 immediately before and after the final measurement. `taskset` is unavailable on macOS; the probe did not set CPU affinity and used one process with no workers.

## Exactness and gate boundary

The standalone micro printed `EXACT 512 seeded lanes including signed extremes: SME == scalar`. It compared one multiply and one multiply-add per lane for both SME variants, with explicit `INT64_MIN`, `INT64_MAX`, and half-ULP edge pairs among the 512 lanes. This is limited differential coverage; it is not the required full-range proof or an application state-hash oracle. Checksums matched across all four variants in every timing row.

The card says to stop before the solver experiment unless the micro is at least 1.5× faster including amortized mode cost. It was 5.25× **slower** than scalar even at 32 lanes, so there is no `BOX3D_SME` solver path, no application before/after table, and no `oracle_check.sh` output. `SmeLaneMulEquivalenceTest` and its CMake registration were not added because the experiment stopped at the micro; the patch records the standalone probe. `./bin/test`, narrow/ludicrous state-hash matrices, and the fixed3d A/B harness were not run. The M5 mini has Apple clang but no `cmake` in PATH or `/opt/homebrew/bin`; no installation was attempted. The historical M3 Ultra benchmark numbers are not a same-compiler M5 before/after and are not used as a speedup claim.

The source-level keep rule is not met. The M5's 512-bit streaming register does not overcome the three 64-bit vector multiplies, shifts and combines needed for this exact fixed-point operation, nor the mode boundary. That is a measured observation of this implementation and compiler, not a general limit on SME hardware. Future work should revisit only if a materially different exact instruction sequence or compiler/OS behavior is available; first measure it as an isolated micro with the same round-half-up semantics. Do not repeat NEON `b3FloatW` emulation, precision narrowing, arithmetic fusion, or other rejected solver changes recorded in `CLAUDE.md` without a new reason.
