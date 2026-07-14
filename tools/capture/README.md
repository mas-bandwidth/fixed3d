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
