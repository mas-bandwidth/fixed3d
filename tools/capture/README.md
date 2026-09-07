# Per-sample visual A/B harness

Runs every sample to its end state on two builds (this fixed-point tree and the
pristine float baseline), captures one PNG per sample, and produces a worst-first
HTML contact sheet of the pairs. This is the harness behind the 2026-07-14
153/153-sample visual verification recorded in CLAUDE.md.

macOS only: the app-side capture path (`--capture` / `--headless` /
`--list-samples`, `samples/host/capture.{h,m}`) is Metal/ImageIO, gated behind
`if(APPLE)` in `samples/CMakeLists.txt`.

## Files

- `capture_sweep.sh <samples-binary> <data-dir> <outdir> <frames>` — enumerates
  samples via `--list-samples`, then runs each headless for N frames (default 120)
  with a 300 s timeout guard, writing `<outdir>/<safe-name>.png` + a per-sample log.
- `compare_shots.py FIXED_DIR FLOAT_DIR OUT_HTML` — pairs captures by
  category/name, computes 64x36 grayscale mean-difference metrics plus an
  empty-frame heuristic (stddev < 3.0), and emits a self-contained data-URI
  contact sheet sorted most-different-first. Needs Pillow (`pip install pillow`).
  NOTE: the category/name -> filename mapping is duplicated in BOTH scripts and
  must stay in sync if either changes.
- `float-baseline-capture-hooks.patch` — the float-side port of the capture hooks:
  applies to upstream float baseline e9f6f1d (`samples/main.cpp` +184 lines,
  `samples/CMakeLists.txt` +5). The float tree's main.cpp differs from ours, so
  this patch is the record of the hand-port; re-deriving it is real work.

## Float-baseline recipe

```sh
git worktree add <scratch>/floatref e9f6f1d
cd <scratch>/floatref
git apply <this-dir>/float-baseline-capture-hooks.patch
cp <fixed3d>/samples/host/capture.h  samples/host/
cp <fixed3d>/samples/host/capture_macos.m samples/host/   # byte-identical on both trees
cmake -B build -DBOX3D_SAMPLES=ON -DBOX3D_UNIT_TESTS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Then sweep both binaries and compare:

```sh
tools/capture/capture_sweep.sh <fixed3d>/build-samples/bin/samples <fixed3d>/data shots-fixed 120
tools/capture/capture_sweep.sh <floatref>/build/bin/samples <floatref>/data shots-float 120
python3 tools/capture/compare_shots.py shots-fixed shots-float contact_sheet.html
```

Reading the sheet: chaotic scenes diverge legitimately — look for empty/garbage
frames and structural differences, not pixel equality. Known real divergences as
of 2026-07-14 (see CLAUDE.md): Stacking/Card House stands in float, collapses in
fixed (equilibrium knife edge at the Q48.16 resolution floor); the World/Far
samples diverge in fixed point's favor (float shatters at 10,000 km).

## Repeated captures and exact pixel hashes

`compare_samples.py` runs the same named samples in both applications, prints a
SHA-256 hash of each decoded RGBA image, and writes `report.json` and `report.html`.
It requires Python 3.11+ and Pillow. It runs each sample twice by default and fails
if the same binary produces different pixels, a capture fails, a counterpart is
missing, or image sizes differ. Its output directory must be new so stale images
cannot pass as fresh captures. Binary hashes, sample indexes, frame count, full
image hashes and difference metrics are retained in the JSON report.

For the current float upstream revision `47d7f7c`, use the self-contained
`float-47d7f7c-capture-hooks.patch` instead of the older patch above. It includes
both capture support files and changes only the sample host, not the engine:

```sh
git -C <box3d> worktree add --detach <scratch>/floatref 47d7f7c
git -C <scratch>/floatref apply <fixed3d>/tools/capture/float-47d7f7c-capture-hooks.patch
cmake -S <scratch>/floatref -B <scratch>/float-build \
  -DBOX3D_SAMPLES=ON -DBOX3D_UNIT_TESTS=OFF \
  -DBOX3D_DOUBLE_PRECISION=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build <scratch>/float-build
```

Then compare selected samples (or omit `--match` for the full sample union):

```sh
python3 tools/capture/compare_samples.py \
  --fixed <fixed-build>/bin/samples --fixed-data <fixed3d>/data \
  --float <float-build>/bin/samples --float-data <floatref>/data \
  --output <new-output-directory> --seconds 2 --no-axes \
  --match '^Joints/(Parallel Spring|Prismatic|Revolute|Wheel)$'
```

`--seconds` means nominal simulation time at 60 Hz; `--frames N` selects the
number of physics frames directly. Each frame uses the sample's own substep
setting. The fixed timestep is quantized to Q48.16, so two nominal seconds are
not exactly two seconds of integrated time. Both apps start from fresh processes
with their authored seeds, camera, settings and default worker count; interactive
settings are not loaded by the headless path. Samples that seed from wall time
can fail the repeatability check and need a deterministic seed before comparison.

`--no-axes` passes `--capture-no-axes` to both headless applications, hiding just
the colored absolute world-axis lines. This is useful because current Fixed3D
builds these scenes around 120,000,000,000 units on each axis, while float samples
are generally centered at zero. The repeating ground grid and physics are
unchanged. Without this option, those lines alone can produce different hashes.

Add `--exact` to fail on any difference between the two image hashes. This is an
exact *pixel* check, not a general physics compliance test: an image cannot see
velocities, hidden objects, contacts or future behavior. Float and fixed are not
expected to produce identical pixels in every scene, especially after chaotic
motion. Inspect the full-resolution pairs and retain analytical numerical tests.
The report also gives changed-pixel percentage, RGB mean absolute difference,
and a small-image luminance metric for sorting. No tolerance is silently treated
as a physics pass. GPU/renderer changes can invalidate image references, so keep
the binaries and environment consistent when using these as regression goldens.
