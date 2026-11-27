#!/usr/bin/env bash
# Print the header every perf finding pastes: CPU model, ISA flags, pinning,
# load average, compiler version, and cmake flags. The shared-box rules in
# CLAUDE.md are the context this header is meant to make reproducible.
set -u

echo "== CPU model =="
grep -m1 'model name' /proc/cpuinfo 2>/dev/null || sysctl -n machdep.cpu.brand_string 2>/dev/null

echo "== ISA flags =="
flags=$(grep -o -w -E 'avx2|avx512f|avx512dq|avx512vl|avx512ifma|neon|sve|sme|sme2' /proc/cpuinfo 2>/dev/null | sort -u)
if [ -n "$flags" ]; then
    echo "$flags"
else
    for k in opt.arm.FEAT_AdvSIMD opt.arm.FEAT_SVE opt.arm.FEAT_SME opt.arm.FEAT_SME2; do
        v=$(sysctl -n hw.$k 2>/dev/null)
        echo "$k=$v"
    done
fi

echo "== cores =="
nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null

echo "== CPU pinning =="
taskset -cp $$ 2>/dev/null || echo "taskset unavailable"

echo "== Load average =="
cat /proc/loadavg 2>/dev/null || uptime

echo "== Compiler =="
cc --version 2>/dev/null | head -1
c++ --version 2>/dev/null | head -1

echo "== cmake =="
cmake --version 2>/dev/null | head -1

echo "== cmake flags =="
echo "configured by the build dir; paste the cmake -B ... line used"
