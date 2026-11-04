# Vendored `fixed`

This directory is a **generated copy** of [mas-bandwidth/fixed](https://github.com/mas-bandwidth/fixed),
the deterministic fixed-point math library that Fixed3D's own fixed-point core was
extracted into.

    Pinned at: unreleased-8dda4fc  (8dda4fcdae38a7fde0a63bd9ba8edc3f2f75ad1c)

fixed main after [#66](https://github.com/mas-bandwidth/fixed/pull/66): every unary minus
on an unsigned value is spelled `(T)0 - x` (MSVC C4146 at /W4), which is bit-identical, and
the portable `fixInt128Div` path (the Windows one) negates its quotient as `0 - q`, so
`INT128_MIN / 1` no longer overflows a signed negation. Nothing else in the vendored files
moved from the previous pin (4a70c64), whose notes follow; the determinism goldens hold.
moo's `extern/fixed` moves to the same commit, as `tools/moo-fixed3d-compare.sh` requires.

Previous pin, 4a70c64 -- fixed main after [#59](https://github.com/mas-bandwidth/fixed/pull/59)-[#62](https://github.com/mas-bandwidth/fixed/pull/62):
under `FIX_SATURATE` (box3d's `BOX3D_FIXED_SATURATE`) every add, subtract and 128-bit
reduction saturates with the sign of the exact result instead of wrapping -- `fixDot`,
`fixVecAdd`/`fixVecSub`, `fixCross`, the quaternion products, `fixMulAdd` and the rotations,
the lerps, `fixMulMV` and the rest. The default build's code is unchanged from the previous
pin (2f1ed91, fixed #28) apart from new functions and clang-cl taking the GNU always_inline
attribute. This pin is a commit, not a published release tag; consumer determinism checks
cover integration.

## Do not edit anything in this directory

Every file here is written by `tools/revendor-fixed.sh`. A hand edit will be reverted
the next time anyone runs it, and CI fails before that happens
(`.github/workflows/vendor-drift.yml` runs `--check` on every push and pull request).

That check is the point of the arrangement. A vendored dependency is two copies of one
truth, and two copies that nothing compares will diverge without ever failing — the
copy keeps compiling while quietly describing a version that no longer exists. So the
copy is *generated*, and something *reads* it. A generator with no reader is a copy
with extra steps.

To change this code, change it upstream in `mas-bandwidth/fixed`, cut a release, then
move the pin here. Reviewing the re-vendor diff **is** the upstream review.

## Regenerating

    tools/revendor-fixed.sh            # rewrite this directory from the pin
    tools/revendor-fixed.sh --check    # diff against the pin, exit 1 on drift

To move to a newer `fixed`: edit `FIXED_PIN` and `FIXED_VERSION` in
`tools/revendor-fixed.sh`, run it, and commit the result together with the pin line at
the top of this file. The pin is a commit SHA rather than a tag name deliberately — a
tag can be moved, a commit cannot, and "vendored at v1.0.0" should mean one specific
tree forever.

## Why vendored rather than fetched

1. **Fixed3D is a fork of box3d** and takes continuous upstream ports from it. box3d
   has no external dependencies; adding a required one changes the build shape away
   from upstream and makes every future port merge harder. A vendored directory is a
   clean seam in a tree that already merges upstream regularly.
2. **Determinism is the product claim.** A version-resolved dependency is a determinism
   hazard. A vendored tree pins exactly, by construction.
3. **Consumers use Fixed3D through FetchContent.** A transitive FetchContent is real
   friction downstream — offline builds, corporate mirrors, nested resolution.
4. It is already the house pattern: netcode and yojimbo vendor libsodium the same way.

## What is not copied

Upstream's tests and CI stay upstream. They run there on every push, and duplicating
them here would mean two suites that can disagree about the same code. What this repo
checks is that *Fixed3D still produces identical results* with the library wired in —
the determinism goldens in `test/test_determinism.c` — which is a different question
and the one that matters here.

## Licence

`LICENSE` and `NOTICE` are copied verbatim from upstream. The library is MIT; the
Box3D-derived portions remain under Erin Catto's MIT grant in every case.
