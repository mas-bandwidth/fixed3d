VERDICT: DROP

# Constraint-color memory locality tracer

The single one-constraint-ahead prefetch experiment improves Many Pyramids by 3.2–3.7% and Large Pyramid by 2.3–2.9% on Zen 4, but the five-scene, three-worker geometric mean is **1.01344×** (1.34% faster), below the 1.03× keep rule. Rain, Washer, and Bodies 20k are within or near their measured spread. No source adoption is justified. This is one candidate and one complete A/B measurement, with no post-measurement tuning.

## Hypothesis and change

Colored convex and mesh contact arrays are already contiguous by graph color. Their hot warm-start and solve loops still gather scattered awake-body states. Prefetching the *next* constraint and its A/B body states while solving the current one could overlap cache latency, especially on pyramid scenes. The patch adds read-only hints one constraint ahead in convex-wide, scalar-mesh, and mesh-wide warm-start/solve loops. It bounds each hint to the current solver block, skips static identity indices, and keeps the existing constraint order and all arithmetic untouched. Constraint arrays are transient for a step; prepared indices and awake-body states are valid through each color stage barrier. There is no persistent cache or new cross-stage owner.

## First profile and counter limit

Before changing source, the base build was profiled over seven 120-step runs at one worker. Phase medians below are user-space timings in microseconds per step. The requested PMU loads/stores and cache misses **could not be counted**: `perf stat -e cache-misses,L1-dcache-load-misses` exited 255 because `perf_event_paranoid=4`. No privilege or host configuration was changed. Consequently this table locates time, not bytes, and the measured-counter acceptance condition is unmet.

| Scene | Warm start | Solve impulses | Relax impulses | Store impulses | Whole step |
|---|---:|---:|---:|---:|---:|
| Rain | 211.234 | 1,503.159 | 1,595.899 | 0.970 | 4,986.499 |
| Large pyramid | 10,964.773 | 9,095.753 | 16,982.316 | 888.818 | 53,556.904 |

## Interleaved before/after measurement

Host `hetzner1`: AMD Ryzen 7 PRO 8700GE (Zen 4), 16 logical cores, `avx2 avx512f avx512dq avx512vl avx512ifma`; GCC 13.3.0; CMake 3.28.3; `RelWithDebInfo`, `BOX3D_SAMPLES=OFF`, `BOX3D_VALIDATE=ON`, `CMAKE_COMPILE_WARNING_AS_ERROR=ON`, `BOX3D_BENCHMARKS=ON`, default LTO (`-flto=auto`) in both builds. No SIMD opt-in was set. `taskset -cp $$`: `0-15`. Load average before A/B: `5.31 5.43 3.37`; after: `2.01 2.31 2.58`; benchmark-process check found only the invoking shell before the run and no other benchmark. The harness interleaved A/B seven times for each scene at workers 1, 2, and 4, 120 steps. Direct A/B final hashes matched in all 105 pairs; `ab.sh` exited 0. A separate `-w=4` invocation printed `thread count: 4`, confirming the requested worker count was not clamped. All raw 18-phase rows and commands are in `raw.txt`.

Values are microseconds per step: minimum / median / spread across seven runs. Speedup is A median divided by B median. These are the four targeted solver phases plus whole step; the raw artifact contains every harness phase.

| Scene | Workers | Phase | Base min / med / spread (µs) | Prefetch min / med / spread (µs) | Speedup |
|---|---:|---|---:|---:|---:|
| Rain | 1 | step | 4,964.952 / 4,970.940 / 30.551 | 4,963.473 / 4,975.070 / 37.892 | 0.9992× |
| Rain | 1 | warmStart | 209.916 / 210.618 / 1.846 | 209.497 / 210.930 / 2.061 | 0.9985× |
| Rain | 1 | solveImpulses | 1,499.460 / 1,502.373 / 9.118 | 1,501.019 / 1,503.322 / 9.259 | 0.9994× |
| Rain | 1 | relaxImpulses | 1,591.605 / 1,593.631 / 5.683 | 1,589.041 / 1,591.587 / 7.815 | 1.0013× |
| Rain | 1 | storeImpulses | 0.809 / 0.824 / 0.077 | 0.805 / 0.860 / 0.133 | 0.9581× |
| Rain | 2 | step | 2,685.484 / 2,705.337 / 49.396 | 2,682.764 / 2,709.256 / 53.514 | 0.9986× |
| Rain | 2 | warmStart | 118.241 / 119.543 / 2.741 | 117.968 / 119.074 / 2.468 | 1.0039× |
| Rain | 2 | solveImpulses | 779.660 / 784.984 / 12.643 | 780.114 / 785.661 / 14.999 | 0.9991× |
| Rain | 2 | relaxImpulses | 835.502 / 841.732 / 11.914 | 834.054 / 839.074 / 13.796 | 1.0032× |
| Rain | 2 | storeImpulses | 0.771 / 0.785 / 0.042 | 0.759 / 0.787 / 0.073 | 0.9975× |
| Rain | 4 | step | 1,549.706 / 1,558.704 / 34.167 | 1,544.060 / 1,568.656 / 37.452 | 0.9937× |
| Rain | 4 | warmStart | 76.941 / 77.346 / 2.120 | 76.808 / 77.823 / 2.612 | 0.9939× |
| Rain | 4 | solveImpulses | 433.779 / 435.732 / 11.489 | 431.617 / 440.520 / 12.558 | 0.9891× |
| Rain | 4 | relaxImpulses | 469.277 / 471.761 / 10.444 | 467.131 / 474.541 / 11.861 | 0.9941× |
| Rain | 4 | storeImpulses | 0.692 / 0.718 / 0.045 | 0.692 / 0.707 / 0.043 | 1.0156× |
| Many pyramids | 1 | step | 126,055.536 / 127,213.894 / 1,439.201 | 122,460.424 / 122,895.484 / 1,526.292 | 1.0351× |
| Many pyramids | 1 | warmStart | 28,348.204 / 28,452.488 / 402.080 | 27,175.591 / 27,330.499 / 743.397 | 1.0411× |
| Many pyramids | 1 | solveImpulses | 21,220.696 / 21,284.030 / 190.364 | 20,193.339 / 20,281.776 / 297.672 | 1.0494× |
| Many pyramids | 1 | relaxImpulses | 38,621.201 / 39,202.924 / 808.794 | 37,163.079 / 37,369.515 / 670.901 | 1.0491× |
| Many pyramids | 1 | storeImpulses | 2,213.495 / 2,226.624 / 32.390 | 2,225.550 / 2,233.358 / 29.038 | 0.9970× |
| Many pyramids | 2 | step | 65,027.362 / 65,251.243 / 502.631 | 62,781.635 / 62,951.339 / 1,008.747 | 1.0365× |
| Many pyramids | 2 | warmStart | 14,817.624 / 14,955.666 / 249.176 | 14,205.867 / 14,299.022 / 175.249 | 1.0459× |
| Many pyramids | 2 | solveImpulses | 10,891.815 / 10,917.931 / 118.428 | 10,371.099 / 10,397.014 / 158.236 | 1.0501× |
| Many pyramids | 2 | relaxImpulses | 19,853.810 / 19,954.939 / 200.843 | 18,871.868 / 19,008.190 / 562.853 | 1.0498× |
| Many pyramids | 2 | storeImpulses | 1,261.829 / 1,268.676 / 13.411 | 1,268.160 / 1,271.347 / 10.311 | 0.9979× |
| Many pyramids | 4 | step | 33,976.546 / 34,097.395 / 270.755 | 32,967.039 / 33,029.730 / 721.877 | 1.0323× |
| Many pyramids | 4 | warmStart | 7,745.150 / 7,781.400 / 97.612 | 7,517.746 / 7,569.512 / 161.583 | 1.0280× |
| Many pyramids | 4 | solveImpulses | 5,659.708 / 5,677.973 / 52.148 | 5,406.098 / 5,429.933 / 111.379 | 1.0457× |
| Many pyramids | 4 | relaxImpulses | 10,076.734 / 10,204.810 / 178.264 | 9,689.699 / 9,701.094 / 277.260 | 1.0519× |
| Many pyramids | 4 | storeImpulses | 780.314 / 782.053 / 5.559 | 778.813 / 785.067 / 14.823 | 0.9962× |
| Large pyramid | 1 | step | 54,630.183 / 55,706.107 / 1,775.633 | 53,265.478 / 54,446.404 / 2,122.855 | 1.0231× |
| Large pyramid | 1 | warmStart | 11,566.134 / 11,930.730 / 697.274 | 11,479.157 / 11,594.764 / 655.135 | 1.0290× |
| Large pyramid | 1 | solveImpulses | 9,331.827 / 9,601.101 / 400.948 | 9,020.646 / 9,238.835 / 457.694 | 1.0392× |
| Large pyramid | 1 | relaxImpulses | 17,254.345 / 17,497.044 / 412.427 | 16,688.944 / 16,921.257 / 666.227 | 1.0340× |
| Large pyramid | 1 | storeImpulses | 885.801 / 891.696 / 23.911 | 880.248 / 894.461 / 27.218 | 0.9969× |
| Large pyramid | 2 | step | 27,851.849 / 28,037.245 / 844.383 | 27,167.711 / 27,339.262 / 708.154 | 1.0255× |
| Large pyramid | 2 | warmStart | 6,057.708 / 6,109.366 / 177.243 | 5,819.060 / 5,905.299 / 236.654 | 1.0346× |
| Large pyramid | 2 | solveImpulses | 4,708.433 / 4,756.435 / 192.194 | 4,533.351 / 4,587.844 / 142.914 | 1.0367× |
| Large pyramid | 2 | relaxImpulses | 8,597.175 / 8,718.149 / 346.411 | 8,371.206 / 8,475.131 / 268.248 | 1.0287× |
| Large pyramid | 2 | storeImpulses | 523.958 / 529.559 / 13.264 | 523.270 / 526.096 / 9.538 | 1.0066× |
| Large pyramid | 4 | step | 14,237.886 / 14,270.244 / 275.217 | 13,751.864 / 13,864.106 / 217.062 | 1.0293× |
| Large pyramid | 4 | warmStart | 2,996.483 / 3,017.470 / 73.269 | 2,869.212 / 2,875.592 / 50.995 | 1.0493× |
| Large pyramid | 4 | solveImpulses | 2,327.416 / 2,340.676 / 51.609 | 2,197.902 / 2,214.037 / 37.169 | 1.0572× |
| Large pyramid | 4 | relaxImpulses | 4,242.516 / 4,262.299 / 85.022 | 4,052.687 / 4,097.134 / 80.071 | 1.0403× |
| Large pyramid | 4 | storeImpulses | 323.974 / 325.344 / 10.079 | 322.311 / 326.360 / 7.777 | 0.9969× |
| Washer | 1 | step | 25,361.398 / 25,537.149 / 1,891.368 | 25,205.923 / 25,282.741 / 365.622 | 1.0101× |
| Washer | 1 | warmStart | 2,324.758 / 2,348.945 / 65.011 | 2,351.131 / 2,364.152 / 75.123 | 0.9936× |
| Washer | 1 | solveImpulses | 1,872.151 / 1,896.666 / 43.594 | 1,777.000 / 1,791.222 / 50.536 | 1.0589× |
| Washer | 1 | relaxImpulses | 3,542.813 / 3,575.580 / 70.628 | 3,432.880 / 3,465.466 / 91.240 | 1.0318× |
| Washer | 1 | storeImpulses | 175.390 / 178.709 / 9.069 | 175.303 / 178.502 / 5.060 | 1.0012× |
| Washer | 2 | step | 13,383.184 / 13,405.060 / 222.646 | 13,295.776 / 13,336.328 / 244.558 | 1.0052× |
| Washer | 2 | warmStart | 1,228.153 / 1,236.762 / 48.117 | 1,227.925 / 1,237.342 / 58.579 | 0.9995× |
| Washer | 2 | solveImpulses | 968.894 / 973.233 / 29.721 | 925.577 / 932.531 / 34.375 | 1.0436× |
| Washer | 2 | relaxImpulses | 1,806.709 / 1,816.478 / 56.986 | 1,761.039 / 1,779.076 / 56.917 | 1.0210× |
| Washer | 2 | storeImpulses | 101.713 / 104.793 / 4.800 | 103.300 / 104.715 / 3.983 | 1.0007× |
| Washer | 4 | step | 7,299.184 / 7,327.426 / 104.892 | 7,231.791 / 7,295.732 / 104.704 | 1.0043× |
| Washer | 4 | warmStart | 655.378 / 660.673 / 24.339 | 645.830 / 651.146 / 19.126 | 1.0146× |
| Washer | 4 | solveImpulses | 507.604 / 508.594 / 18.699 | 481.700 / 487.050 / 12.233 | 1.0442× |
| Washer | 4 | relaxImpulses | 934.158 / 940.686 / 30.448 | 908.530 / 915.293 / 22.450 | 1.0277× |
| Washer | 4 | storeImpulses | 60.231 / 60.651 / 1.390 | 60.066 / 60.661 / 2.144 | 0.9998× |
| Bodies 20k | 1 | step | 26,973.154 / 27,027.853 / 292.664 | 26,901.490 / 26,968.445 / 370.989 | 1.0022× |
| Bodies 20k | 1 | warmStart | 1,543.114 / 1,568.677 / 83.858 | 1,549.117 / 1,555.756 / 168.693 | 1.0083× |
| Bodies 20k | 1 | solveImpulses | 830.115 / 845.318 / 40.936 | 771.301 / 785.796 / 28.928 | 1.0757× |
| Bodies 20k | 1 | relaxImpulses | 2,013.449 / 2,036.392 / 57.508 | 1,976.258 / 2,005.635 / 96.901 | 1.0153× |
| Bodies 20k | 1 | storeImpulses | 85.022 / 85.297 / 4.577 | 85.381 / 86.784 / 3.967 | 0.9829× |
| Bodies 20k | 2 | step | 14,706.402 / 14,914.417 / 298.510 | 14,779.591 / 14,817.228 / 183.172 | 1.0066× |
| Bodies 20k | 2 | warmStart | 812.980 / 842.211 / 45.545 | 812.163 / 814.690 / 42.830 | 1.0338× |
| Bodies 20k | 2 | solveImpulses | 456.534 / 462.604 / 10.421 | 430.111 / 436.671 / 8.333 | 1.0594× |
| Bodies 20k | 2 | relaxImpulses | 1,051.483 / 1,062.680 / 24.960 | 1,033.725 / 1,039.630 / 27.737 | 1.0222× |
| Bodies 20k | 2 | storeImpulses | 51.318 / 51.890 / 1.010 | 51.671 / 52.125 / 2.105 | 0.9955× |
| Bodies 20k | 4 | step | 8,550.386 / 8,623.385 / 114.994 | 8,488.496 / 8,610.975 / 168.723 | 1.0014× |
| Bodies 20k | 4 | warmStart | 438.348 / 445.337 / 14.483 | 430.257 / 437.271 / 8.932 | 1.0184× |
| Bodies 20k | 4 | solveImpulses | 247.012 / 247.704 / 2.125 | 232.565 / 233.648 / 2.474 | 1.0602× |
| Bodies 20k | 4 | relaxImpulses | 556.656 / 560.289 / 6.871 | 545.357 / 546.831 / 3.927 | 1.0246× |
| Bodies 20k | 4 | storeImpulses | 32.717 / 33.157 / 1.071 | 32.804 / 33.043 / 0.859 | 1.0034× |

The whole-step speedups by scene at workers 1/2/4 are: Rain `0.9992 / 0.9986 / 0.9937`; Many Pyramids `1.0351 / 1.0365 / 1.0323`; Large Pyramid `1.0231 / 1.0255 / 1.0293`; Washer `1.0101 / 1.0052 / 1.0043`; Bodies 20k `1.0022 / 1.0066 / 1.0014`. The 15-row geometric mean is **1.01344×**. No scene is more than 2% slower, but the required 3% aggregate gain is absent; targeted phase speedups are also below the alternate 1.5× rule.

## Bit-exact checks and limits

- A baseline-capture run of `ConstraintLayoutBitIdenticalTest` printed the 24 per-color ordered contact-ID fingerprints then failed against intentionally zero placeholders. This is fixture capture, **not** a product regression. After freezing the base table, the unchanged base and candidate tests pass for ragdoll workers 1–5. Candidate `./bin/test` passed in both narrow and `BOX3D_LUDICROUS_MODE` builds.
- `tools/perf/oracle_check.sh` was run in five small groups over all 13 scenes, workers 1/2/4 and both modes. Ten scenes passed the frozen scalar oracle. Rain, ragdoll_pile, and bodies_20k failed because the scalar-oracle file is stale at base `59eff1c`: the unchanged base and candidate produced the **same** scene hash in every mode and worker count. Rain: file `d3e59885dbca43e1`, base/candidate `f96c9e008d59a375`; ragdoll_pile: file `4ac74cd9f1805daf`, base/candidate `42fa9c6438e3527f`; bodies_20k: file `3e7cbde301a4f6c2`, base/candidate `ed59e6aff81bcd26`. Thus the candidate is direct-base bit-identical in these rows, but the literal frozen-oracle gate is red and was not rebaselined in this artifact-only card.
- Apple silicon measurement was not run. The coordinator reported that the available mini M5 lacks CMake, and this card was not authorized to install tools. Cross-ISA timing and oracle acceptance remain unmeasured. This alone prevents a KEEP verdict even if Zen 4 had met the speed threshold.
- The five A/A noise-floor passes suggested by the harness were not run. The complete A/B matrix already misses the 3% keep threshold, so no gain is claimed beyond the seven-run spreads shown above.

## Next and do not repeat

If a new locality card is considered, first repair the three stale scalar-oracle entries on the shared stream with independent base captures and provide an Apple bench with an existing toolchain. Profile phase-attributed memory events on a host that permits PMU access before choosing a different distance or layout. Do not adopt this one-slot prefetch patch from a single-host 1.34% aggregate gain. Do not revisit the previously rejected full spherical cache, pointer variants, forced inlining, narrowed fixed-point lanes, tangent2 recomputation, padded body state, fused primitives, NEON-emulated wide arithmetic, or data-dependent guards without a new reason.
