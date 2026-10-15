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
one A/A pass per scene at `-w=1 -r=3 --steps=120` (whole-step ns, min/median
and the median relative spread):

| scene          | median ns   | spread ns | spread % |
|----------------|-------------|-----------|----------|
| large_pyramid  | 74,966,824  | 1,009,114 | 1.3      |
| many_pyramids  | 167,505,743 | 5,930,738 | 3.5      |
| junkyard       | 13,446,243  | 386,829   | 2.9      |
| bodies_20k     | 37,334,099  | 548,600   | 1.5      |

geomean step speedup for the A/A pass was 0.997 (baseline vs baseline). A
five-pass floor is the full protocol; the pass above is one of the five.
`many_pyramids` at full step count is ~17s/run at `-w=1`, so the `--steps`
cap (default 120) is what makes even one pass fit a short wall.

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
