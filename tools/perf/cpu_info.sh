#!/usr/bin/env bash
# The header every perf finding pastes: CPU model, ISA flags, affinity, load,
# and the toolchain. Idempotent and read-only.
set -u

echo "== cpu =="
if command -v lscpu >/dev/null 2>&1; then
    lscpu | grep -Ei 'model name|^CPU\(s\)|Thread|Core|Socket|Architecture|Vendor' | sed 's/^ *//'
else
    sysctl -n machdep.cpu.brand_string 2>/dev/null || uname -m
fi

echo
echo "== isa flags =="
if [ "$(uname -s)" = "Darwin" ]; then
    sysctl hw.optional.arm.FEAT_ 2>/dev/null | sed 's/^hw.optional.arm.//'
    sysctl -a 2>/dev/null | grep -Ei 'hw.optional.arm.*(neon|sme)' || true
else
    grep -o -w -E 'avx2|avx512f|avx512dq|avx512vl|avx512ifma|neon|sse4_2' /proc/cpuinfo 2>/dev/null | sort -u
fi

echo
echo "== affinity =="
if command -v taskset >/dev/null 2>&1; then
    taskset -cp $$ 2>/dev/null || echo "taskset unavailable"
else
    echo "taskset unavailable (non-Linux)"
fi

echo
echo "== load average =="
uptime

echo
echo "== toolchain =="
cc --version 2>/dev/null | head -1 || gcc --version 2>/dev/null | head -1
clang --version 2>/dev/null | head -1
cmake --version 2>/dev/null | head -1
make --version 2>/dev/null | head -1
