# fixed3d perf lab

How to measure a change and prove it did not change a bit. The scalar build is the
bit-exact oracle: the AVX-512/NEON paths are only ever accepted if they reproduce the
scalar body-state hash bit for bit.

## Build the baseline and candidate

Two scalar configs from the repo root (clang; gcc trips a pre-existing
`-Werror=maybe-uninitialized` in `distance.c`/`recording_replay.c`):

```sh
CC=clang cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBOX3D_SAMPLES=OFF \
    -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBOX3D_BENCHMARKS=ON
CC=clang cmake -B build-ludicrous -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBOX3D_SAMPLES=OFF \
    -DBOX3D_VALIDATE=ON -DCMAKE_COMPILE_WARNING_AS_ERROR=ON -DBOX3D_BENCHMARKS=ON \
    -DBOX3D_LUDICROUS_MODE=ON
cmake --build build --parallel 2
cmake --build build-ludicrous --parallel 2
```

The test suite runs from inside a build dir: `./bin/test` (all) or `./bin/test Suite`.
A candidate build is the same two dirs with `-DBOX3D_AVX512=ON` (or `-DBOX3D_NEON=ON`
on Apple silicon) added — everything else identical.

## Verify the oracle

```sh
tools/perf/oracle_check.sh build build-ludicrous
```

It runs every scene at worker counts {1,2,4} in both configs, diffs the `HASH` lines
against `tools/perf/scalar-oracle.txt`, and asserts the determinism claim: a scene's hash
must agree across worker counts within a build. `--regenerate` rewrites the oracle but
only after two independent scalar runs agree. A fresh oracle must be committed.

## Measure an A/B

```sh
tools/perf/ab.sh build build-candidate -b=convex_pile -w=4 -r=7
```

`ab.sh` runs the two binaries interleaved (A B A B ...) for N repeats per scene, prints
min/median/spread and speedup for the whole step and every phase, a geomean, and exits
non-zero if any `--hash` differs. Speedup > 1 means the candidate is faster (base/cand).

## Reading the numbers

- Only trust **interleaved A/B ratios**, never absolute times: the box is shared and
  absolute numbers drift with machine state. The published lesson from CLAUDE.md: a
  refresh must re-run the reference in the same session or ratios silently lie.
- A spike's gain must exceed that scene's **A/A spread** (baseline against itself) — see
  the noise floor below. Anything inside the spread is noise, not a win.
- The `--phases-json` per-phase ns-per-step values come from `b3World_GetProfile`
  (b3Fixed milliseconds, Q48.16); the benchmark converts to nanoseconds exactly once, at
  output (`b3FixToDouble(msPerStep) * 1e6`).

## Shared-box rules (from CLAUDE.md)

- The box is shared between sessions. **Check `pgrep -af benchmark` and the load average
  before benchmarking.** `ab.sh` refuses to run (exit 3) if another benchmark is on the
  box or the load exceeds 2 per available core — it never kills anything.
- Use the `space` box or isolated cores (`isolcpus`) where possible. One benchmark at a
  time. The launcher process eats ~3.5 cores, so trust only interleaved ratios.
- `perf` needs `sudo sysctl kernel.perf_event_paranoid=1` (default 4; restore after).

## The benchmark app

`benchmark/main.c` runs fixed scenes at a fixed dt and step count. Flags (the `=` form is
mandatory — a space-separated flag is silently ignored and the FULL suite runs):

- `-t=<n>` max threads, `-w=<n>` single worker count, `-r=<n>` repeats, `-b=<scene>`.
- `--hash` prints `HASH <scene> workers=<n> <16 hex>` after the last step: an FNV-1a/64
  over every body's position, rotation, linear and angular velocity in body-id order
  (independently of thread count, allocator and wall clock). `--hash-every=<k>` also
  prints a running hash every k steps.
- `--phases-json=<file>` writes per-scene, per-phase min/median/max ns-per-step.

## Adding a scene

Add a `CreateX`/`GetXCapacity`/`DestroyX` triple to `shared/benchmarks.c` (declared in
`shared/benchmarks.h`), register it in the `benchmarks[]` table in `benchmark/main.c`,
and author every position around `GetSceneOrigin()` with **exact integer-scaled math**.
A scene whose placement math is inexact silently runs a different workload (CLAUDE.md
"large_world scene bug": `B3_FIX(0.1f)` product chains quantize differently from float).
Create bodies through `b3BenchmarkCreateBody` so the hash registry sees them, then
regenerate `scalar-oracle.txt` and commit it.

## Noise floor

Measured on the bench with `tools/perf/ab.sh build build` (baseline against itself), five
A/A runs per scene; a spike's gain must exceed its scene's A/A spread:

| scene | A/A spread (whole-step, %) |
|-------|---------------------------|
|       |                           |

*(Fill this table by running `ab.sh build build -r=7` five times and recording the
per-scene whole-step spread.)*
