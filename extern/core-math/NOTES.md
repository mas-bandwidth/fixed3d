# Vendored CORE-MATH (test-only reference)

This directory is a **generated copy** of six files of CORE-MATH, the correctly rounded
mathematical library, written by `tools/revendor-core-math.sh` and never hand-edited.

    Pinned at: cac142bc65dc140d645ba3f79511a9c591d1746e
    URL:       https://gitlab.inria.fr/core-math/core-math.git
    Committed: 2026-10-01T17:55:16+02:00 (Paul Zimmermann, "[powf] added exact and midpoint inputs")
    License:   MIT (LICENSE, copied from the repository root)

It is the bit-exact reference of `docs/determinism-math.md`, and it is **test-only**:
`test/CMakeLists.txt` builds these files as the `box3d_core_math` object library and links
it into `./bin/test` only. libbox3d never contains a `cr_` symbol.

## Files

| vendored | upstream path | defines | copyright |
|---|---|---|---|
| `atan2.c` | `src/binary64/atan2/atan2.c` | `cr_atan2` | Copyright (c) 2024-2025 Paul Zimmermann and Alexei Sibidanov |
| `tint.h` | `src/binary64/atan2/tint.h` | (included by atan2.c) | Copyright (c) 2023-2024 Paul Zimmermann and Alexei Sibidanov |
| `sin.c` | `src/binary64/sin/sin.c` | `cr_sin` | Copyright (c) 2022-2026 Paul Zimmermann and Tom Hubrecht and Alexei Sibidanov |
| `cos.c` | `src/binary64/cos/cos.c` | `cr_cos` | Copyright (c) 2022-2025 Paul Zimmermann and Tom Hubrecht |
| `log2.c` | `src/binary64/log2/log2.c` | `cr_log2` | Copyright (c) 2023 Alexei Sibidanov. |
| `exp2.c` | `src/binary64/exp2/exp2.c` | `cr_exp2` | Copyright (c) 2023-2026 Alexei Sibidanov <sibid@uvic.ca>. |
| `LICENSE` | `LICENSE` | | the CORE-MATH MIT license |

## sha256

As fetched at the pin; the script refuses a fetch that does not match. Each also equals the
`content_sha256` the GitLab files API reports for that path at the pin (checked
2026-10-17). With no patches, as vendored equals as fetched.

| file | as fetched | as vendored |
|---|---|---|
| `atan2.c` | `404b0db90e536f8248101544d0fcad9141fd76b2a6cc3d189dd80dbfc865bd91` | same |
| `tint.h` | `41264d54812a8ce2b3f8ca4c8c033094df7cbf2d214da1c7555783328171b921` | same |
| `sin.c` | `a3278038675d4f69567445deac9a0f790a64b614a3621db1ffdadd9421592884` | same |
| `cos.c` | `04bffbc8b483c886f78f2c9ab3f2a1ca318dda78ab637a2983a4a9d82374e9d9` | same |
| `log2.c` | `b06b3c6ce071ad4298bc584f8087cbf9970b774bdab222ee05a08bbfc9a6f4a2` | same |
| `exp2.c` | `a4e892adf5e30ddde023b74881173e170748431979e999db8e206f221174064d` | same |
| `LICENSE` | `81830875362f4d477637de58eecd6d458db108cc90c40904fca3694807b86de3` | same |

## Patches

None. A patch goes in `patches/` (see the README there), is applied by the script, and is
recorded here with its reason and the proof that it equals unpatched CORE-MATH over the
dense sets.

## Build flags (not patches)

The vendored files are unpatched. One compiler flag is added for one configuration:

- **ClangCL on x86 / x64: `-msse4.1` for these files only** (test/CMakeLists.txt). `sin.c`
  calls `__builtin_roundeven`; below SSE4.1 clang lowers it to a call to the C23 function
  `roundeven`, which the Windows CRT does not provide, so `box3d_test` failed to link
  (`lld-link : error : undefined symbol: roundeven`, referenced by `sin.c:917`, run
  39071830800). With SSE4.1 it inlines as `roundsd` with mode 8, which is the same exact
  round-half-to-even. Proof, on amd-3 2026-10-18 (gcc 15.2, clang 21.1), `-O2 -std=gnu17
  -ffp-contract=off -w`, each compiler with and without `-msse4.1` (four builds; in each
  compiler's build `objdump -d` shows 3 calls to `roundeven` without the flag, 0 with it).
  Each hash is FNV-1a 64 over the 8 little-endian bytes of every result, in order:
  - A: cr_sin, cr_cos and cr_exp2 of raw / 65536 for every raw in [-2^24, 2^24], and
    cr_log2 of the positive ones: `4e2fa5984a763ae1`.
  - B: cr_atan2(y, x) on 2^24 pairs from xorshift64 (13, 7, 17; seed 0x9E3779B97F4A7C15),
    each raw = (int64)(next >> 16) - 2^47, divided by 65536: `2d9d5a63bd7c1545`.
  - C: 2^24 further xorshift64 outputs read as doubles, non-finite skipped, through
    cr_sin, cr_cos, cr_exp2, cr_log2 and cr_atan2(d, 1): `c3dd651be62c593c`.

  All three hashes are identical in all four builds.

## How the files are fetched

One by one from `https://gitlab.inria.fr/core-math/core-math/-/raw/<pin>/<path>`, then
checked against the sha256 table in the script. A commit cannot move, and the hashes make
the fetch refuse anything else. A shallow `git fetch --depth 1` of the pinned commit, the
shape `tools/revendor-fixed.sh` uses, ran for over six minutes against gitlab.inria.fr and
failed on 2026-10-17; the per-file fetch takes seconds.

## Regenerating

    tools/revendor-core-math.sh            # rewrite this directory from the pin
    tools/revendor-core-math.sh --check    # diff against the pin, exit 1 on drift

To move the pin: edit `CORE_MATH_PIN` and the `FETCHED_SHA256` table in the script, run it,
re-measure the bounds of `docs/determinism-math.md`, and commit the result with this file.
