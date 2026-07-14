#!/bin/zsh
# Capture end-state PNGs for every sample of a samples binary.
# usage: capture_sweep.sh <samples-binary> <data-dir> <outdir> <frames>
set -u
BIN=$1
DATA=$2
OUT=$3
FRAMES=${4:-120}
RUN=$OUT/cwd
mkdir -p "$OUT" "$RUN"
ln -sf "$DATA" "$RUN/data"
cd "$RUN"

"$BIN" --list-samples > "$OUT/samples.tsv"
count=$(wc -l < "$OUT/samples.tsv" | tr -d ' ')
echo "sweeping $count samples"

fails=""
while IFS=$'\t' read -r idx cat name; do
  safe=$(echo "${cat}__${name}" | tr ' /' '__' | tr -cd 'A-Za-z0-9_.-')
  png="$OUT/${safe}.png"
  # 300s guard: heavy scenes (million-body world, TOI bursts) are slow but finite.
  ( "$BIN" --headless --sample "$idx" --frames "$FRAMES" --capture "$png" > "$OUT/${safe}.log" 2>&1 ) &
  pid=$!
  for i in $(seq 1 300); do
    kill -0 $pid 2>/dev/null || break
    sleep 1
  done
  if kill -0 $pid 2>/dev/null; then
    kill -9 $pid 2>/dev/null
    echo "TIMEOUT $idx $cat/$name"
    fails="$fails T:$idx"
    continue
  fi
  wait $pid
  code=$?
  if [ $code -ne 0 ] || [ ! -s "$png" ]; then
    echo "FAIL $idx $cat/$name exit=$code"
    fails="$fails F:$idx"
  fi
done < "$OUT/samples.tsv"

echo "DONE failures:[$fails]"
