# Skip disabled joint work: targeted measurements

The follow-up to PR59 avoids work unused by the active motor/weld configuration.
It does not change the 40-bit inverse scale, the wide inverse or joint solve,
rounding, or the application of stored impulses.

- Motor orientation is computed only for an active angular spring.
- Motor angular mass is inverted only when an angular spring or velocity motor
  consumes it. The condition is reevaluated every prepare stage, so enabling a
  feature restores the original wide inversion immediately.
- Motor anchors are rotated only when a linear spring or velocity motor uses them.
- Weld orientation is computed only when angular bias is needed. Velocity-only
  relaxation and fully locked rotation do not use it.

Warm starting still applies accumulated impulses even after a feature is disabled.
A comment records this requirement; clearing those impulses would change behavior.

## Results

Apple M3 Ultra, macOS 26.6.2, Apple Clang 21, RelWithDebInfo (`-O2`, thin LTO),
NEON narrow phase enabled. The preserved baseline is PR59, main `1e58adf`.
These are **targeted single-worker workloads**, not the full-suite four-worker
float comparison. Each scene has 1,024 independent paired cubes, no contacts,
720 frames, four substeps per frame, no sleeping and zero gravity. Every third
pair has one static body; the others have two dynamic bodies. Bodies start with
small nonzero velocities and mixed quaternion signs. Warm starting is disabled
at frame 70 and enabled at frame 130.

World-step time excludes creation, hashing and force reads. Six fresh processes
per scene run in `before, after, after, before, before, after` order; the table
uses the median of all three observations for each binary. The accompanying
process-CPU measurements support the same reductions. No observation was removed.

| Joint configuration | Before (ms) | After (ms) | Runtime reduction |
|---|---:|---:|---:|
| Angular-velocity motor | 1125.5 | 944.6 | 16.1% |
| Linear-only motor | 1548.2 | 1319.2 | 14.8% |
| Rigid weld | 1940.1 | 1867.1 | 3.8% |
| Rotation-locked weld | 1622.8 | 1497.2 | 7.7% |
| Motor with springs active | 3420.7 | 3402.4 | 0.5% |
| Soft weld | 2005.6 | 2007.9 | -0.1% |

The active spring controls are effectively unchanged. In the four optimized
cases, within-binary wall-time spread was 0.6–2.1%. Earlier four-worker trials
were much noisier (up to 37% spread) and do not establish a parallel throughput
gain. Their initial harness also predates the added quaternion-sign and feature-
toggle coverage; do not use the two batches to infer worker scaling.

The full-suite **44.6% scalar / 47.0% NEON float-throughput comparison remains the
last measured overall result**. This targeted pass does not establish a new ratio.

All raw trials, binary hashes, controls and rejected experiments are in the
[JSON record](2026-09-07-joint-feature-performance.json). The contact matrix-pointer
and relative-velocity-pointer experiments were excluded: apparent early wins did
not persist in the balanced recheck. The source profile still identifies contact
solving and joint solving as the larger future targets.

## Validation and reproduction

All 24 suites pass locally in Debug, optimized scalar, NEON, wide positions
and ASan+UBSan. Existing determinism references and asteroid-response tests are
unchanged. Cross-platform CI is tracked by the pull request.

The [portable comparator](../tools/benchmark/joint_modes.c) compares complete body
transforms, linear/angular velocities and joint forces/torques every frame.
Nine modes, sizes 1 and 250, workers 1 and 4, 32 pairs and 240 frames produce
**8,640 identical before/after frame hashes**, plus **4,320 identical cross-worker
hashes** per build. Modes include disabled features, runtime enable/disable,
rotation-lock changes, opposite quaternion signs and warm-start transitions.
Performance trials also preserve every before/after frame hash.

Build and run the same tool against each preserved library:

```sh
cmake -S . -B build-perf -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBOX3D_SAMPLES=OFF -DBOX3D_NEON=ON
cmake --build build-perf
cc -O2 -std=gnu17 -Iinclude -Iextern/fixed/include \
  tools/benchmark/joint_modes.c build-perf/src/libbox3d.a \
  -lpthread -lm -o joint_modes
./joint_modes 0 1 1024 720 1 > state-hashes.txt
```

Arguments are mode, workers, pair count, frames and cube side length. Modes 0–8
are angular-velocity motor, spring motor, rigid weld, soft weld, locked weld,
linear-only motor, disabled motor, toggled motor and toggled weld. Stdout holds
frame hashes; stderr reports summed wall time in `b3World_Step`. The measured
CPU-clock variant adds `CLOCK_PROCESS_CPUTIME_ID` observations around those same
steps; the portable tool omits this platform-specific instrumentation.
