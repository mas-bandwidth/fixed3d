# fixed3d perf lab

A lab for measuring a change and proving it did not change a bit. Every spike
card in this stream needs two things: a way to measure a change (`tools/perf/ab.sh`)
and a way to prove it changed nothing (`tools/perf/oracle_check.sh`).

## The oracle

`tools/perf/scalar-oracle.txt` holds frozen `HASH` lines for every scene x
workers {1,2,4} x {narrow, ludicrous}, produced by the plain **scalar** build
(no `BOX3D_AVX512`, no `BOX3D_NEON`). The scalar build is the exact oracle the
card describes; a candidate build (e.g. an AVX-512 one) must hash bit-identical.

To freeze or refresh the oracle:

    tools/perf/oracle_check.sh --regenerate

This refuses to overwrite the oracle unless two runs on the scalar build agree,
and it asserts the hash agrees across worker counts for the same scene/build
(the determinism claim).

To check a candidate (it builds both configs for you, runs all scenes, diffs):

    tools/perf/oracle_check.sh

Exit non-zero means a bit changed.

## A/B measurement

Interleaved `A B A B ...` runs of the baseline and candidate binaries, `-r`
repeats (default 7) per scene, printing a table and exiting non-zero on any
hash difference:

    tools/perf/ab.sh build build-candidate [-b=scene ...] [-w=1,2,4]

The driver refuses to run while another benchmark process is on the box or the
load average is above 2 per available core, and it never kills anything.

## How to build

From the repo root:

    cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBOX3D_SAMPLES=OFF -DBOX3D_VALIDATE=ON \
        -DCMAKE_COMPILE_WARNING_AS_ERROR=ON
    cmake --build build --parallel 2

The candidate is the same with its own dir; the ludicrous config adds
`-DBOX3D_LUDICROUS_MODE=ON`. Always run on the Linux bench, never the working
Mac.

## Shared-box rules (CLAUDE.md)

- The space box is SHARED. Check `pgrep -af benchmark` and the load average
  before benchmarking; never kill a process you did not start.
- A benchmark run must be interleaved A/B, never two sequential full suites.
- `-b=<scene>` and friends need the equals form; space-separated flags are
  silently ignored.

## Noise floor

Run five A/A passes (baseline against itself) per scene; a spike's gain must
exceed its scene's A/A spread.

## Adding a scene

1. Author it in `shared/benchmarks.c` around `GetSceneOrigin()` with exact
   integer-scaled placement (CLAUDE.md "large_world scene bug": inexact math
   runs a different workload).
2. Register it in the `benchmarks[]` table in `benchmark/main.c`.
3. Add it to the scene lists in `ab.sh` and `oracle_check.sh` (or rely on the
   full-suite default).

## Status

The lab is scaffolded: `oracle_check.sh`/`ab.sh`/`cpu_info.sh` and the oracle
file are committed, and `test/test_perf_hash.c` pins the determinism claim. The
`--hash`/`--hash-every`/`--phases-json` benchmark flags and the `ragdoll_pile`/
`bodies_20k` scenes are not yet implemented (see the card report) — the oracle
file is an empty template until `--hash` lands.
