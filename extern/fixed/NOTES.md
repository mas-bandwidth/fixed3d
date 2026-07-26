# Vendored `fixed`

This directory is a **generated copy** of [mas-bandwidth/fixed](https://github.com/mas-bandwidth/fixed),
the deterministic fixed-point math library that Fixed3D's own fixed-point core was
extracted into.

    Pinned at: v1.2.0  (ba88f94cb9cc03951533daf7678d8980d4118163)

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
