# fixed3d perf lab

Measure a change and prove it did not change a bit. `ab.sh` measures.
`oracle_check.sh` proves the bits.

## Build

From the repo root, on the Linux bench (never the working Mac). Benchmarks are
OFF by default, so the lab adds `-DBOX3D_BENCHMARKS=ON`. Scalar means no
`BOX3D_AVX512` and no `BOX3D_NEON`.

    cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBOX3D_SAMPLES=OFF -DBOX3D_VALIDATE=ON \
        -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBOX3D_BENCHMARKS=ON
    cmake --build build --parallel 2

Ludicrous is the same with `-B build-ludicrous -DBOX3D_LUDICROUS_MODE=ON`.
`oracle_check.sh` configures and builds both.

The unit suite is `./bin/test` from the build dir. One suite:
`./bin/test PerfStateHashGoldenTest`.

## Header

Paste `tools/perf/cpu_info.sh` at the top of every finding: CPU model, ISA
flags (`avx2`, `avx512f/dq/vl/ifma`, `neon`, `sme/sme2`), `taskset -cp $$`,
load average, compiler, cmake flags.

## Oracle

`tools/perf/scalar-oracle.txt` holds frozen lines

    HASH <scene> workers=<n> <narrow|ludicrous> <16 hex>

for every scene x workers {1,2,4} x {narrow, ludicrous}, from the plain scalar
build. The benchmark prints `HASH <scene> workers=<n> <16 hex>` (no mode); the
script tags the mode. Hashes are FNV-1a/64 over every body's position, rotation,
linear and angular velocity in body-id order. They must not depend on thread
count, allocator, or wall clock. The script asserts workers 1/2/4 agree for the
same scene and build.

Every run is capped at `ORACLE_STEPS` (default 120) steps so the full matrix
fits a short wall; the cap is recorded in the oracle header. Split the gate
with `--scenes=a,b,...` (two or three scenes per invocation) so no single call
runs the whole 78-line matrix:

    tools/perf/oracle_check.sh --regenerate                  # all scenes, or refuse
    tools/perf/oracle_check.sh --regenerate --scenes=large_pyramid,many_pyramids
    tools/perf/oracle_check.sh                               # build both, diff, exit 1 on any bit
    tools/perf/oracle_check.sh --scenes=junkyard,bodies_20k  # check a subset

A regenerate with `--scenes` merges those scenes into the oracle and keeps the
rest, so several split invocations accumulate the full matrix.

The frozen hashes are platform-independent: re-verified on an Intel Mac
(x86_64, AppleClang 17) they match the AMD EPYC 9124 bench bit-for-bit, so the
state hash does not depend on the ISA, the allocator, or the wall clock.

## A/B

Interleaved `A B A B ...` (default 7 repeats). Refuses if `pgrep -af benchmark`
shows another benchmark (it never kills it) or load average is above 2 per
core.

    tools/perf/ab.sh build build-candidate [-b=scene ...] [-w=1,2,4] [-r=7] [--steps=120]

Equals form is mandatory. A space-separated flag is ignored and the full suite
runs. `--steps=` caps the step count passed to both binaries. The table is
min/median/spread/speedup per scene and per phase, plus geomean of the
whole-step speedups (baseline median / candidate median; above 1 is faster).
Exit non-zero if any final `HASH` line differs. A spike's gain must exceed that
scene's A/A spread.

`tools/perf/ab.sh build build` is A against A. Speedups should sit inside the
spread and the script should exit 0.

## Noise floor

A spike's gain must exceed its scene's A/A spread. Five A/A runs
(`ab.sh build build -b=<scene> -r=3 --steps=120`, five times) set the floor;
a gain inside the largest spread is noise. A spike on a scene not in the table
measures that scene's A/A first.

Measured on this bench (AMD EPYC 9124, 32 cores, avx2/avx512f/dq/vl/ifma),
five A/A passes per scene at `-w=1 -r=3 --steps=120` (whole-step ns; median
and spread are over the ten medians, five baseline and five candidate):

| scene          | median ns   | spread ns | spread % |
|----------------|-------------|-----------|----------|
| large_pyramid  | 74,604,912  | 5,399,404 | 7.2      |
| many_pyramids  | 168,512,253 | 4,370,108 | 2.6      |
| junkyard       | 13,803,975  | 1,875,969 | 13.6     |
| bodies_20k     | 37,507,172  | 1,021,348 | 2.7      |

geomean step speedup across the twenty A/A passes was 0.9986 (baseline vs
baseline). A spike's gain must exceed its scene's spread above; the junkyard
spread is dominated by one transient pass (a load spike between the heavy
`many_pyramids` and the light `junkyard`), so a spike on junkyard should
re-measure its A/A floor first. `many_pyramids` at full step count is
~17s/run at `-w=1`, so the `--steps` cap (default 120) is what makes even
one pass fit a short wall.

The oracle gate is proven live: neutering one rounding (fixMul's round-half-up
-> truncate, in a throwaway copy, never committed) flips large_pyramid's hash
`72331624b02f2508` -> `62b04f2aadc8e0b3` and `oracle_check.sh
--scenes=large_pyramid` exits 1 (ORACLE MISMATCH).

## Adding a scene

1. Author it in `shared/benchmarks.c` around `GetSceneOrigin()` with integer
   scaling (`b3FixFromInt`). Inexact placement runs a different workload
   (CLAUDE.md, large_world scene bug).
2. Register it at the end of `benchmarks[]` in `benchmark/main.c` so old
   numeric `-b=` indexes stay put.
3. Add the name to the scene lists in `ab.sh` and `oracle_check.sh`.
4. Regenerate the oracle only when two scalar runs agree.

`ragdoll_pile` is 200 `CreateHuman` ragdolls from the determinism corpus
(friction/hertz/damping copied from `shared/determinism.c`), dropped onto an
integer-scaled ground box. `bodies_20k` is 20,000 mixed spheres, capsules and
boxes on an integer grid. The existing pyramid scenes remain the pyramid case.

## Shared-box rules

The space box is shared. Check `pgrep -af benchmark` and the load average
before a run. One bench at a time. Never kill a process you did not start.
`isolcpus` / affinity is the caller's job; `cpu_info.sh` prints `taskset -cp $$`
so the finding records it.
