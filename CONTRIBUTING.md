# Contributing to Fixed3D

Pull requests are welcome. If a change is good and useful, we will merge it.

Fixed3D is a fork of [Box3D](https://github.com/erincatto/box3d) by Erin Catto, and Box3D
does not take pull requests. That is Erin's call for his project, and a reasonable one. It
is not our policy here.

## Where to send what

Anything specific to Fixed3D belongs here: the fixed-point conversion, determinism, the
wide world types, the build, the docs. Open an
[issue](https://github.com/mas-bandwidth/fixed3d/issues) or send a pull request.

Anything wrong with Box3D itself, unrelated to fixed point, is best reported upstream at
[erincatto/box3d](https://github.com/erincatto/box3d/issues). New Box3D work gets ported
across, so a fix there arrives here too.

You do not need permission to open a pull request. For a bug fix or a small improvement,
just send it. For something large, a new subsystem, an API change, a different solver,
open an issue first so we can agree on the approach before you spend a weekend on it.

## Determinism is the whole point

Fixed3D exists so a simulation produces bit-identical results on every platform and
architecture. A change that makes results depend on the compiler, the target
architecture, or the optimization level defeats the reason this fork exists, and it will
not be merged however fast it is.

The full unit suite, including cross-worker and uncached-golden checks, runs on every
pull request with GCC, Clang/ThreadSanitizer, clang-cl and ARM64 NEON, covering Linux,
Windows, macOS and both position widths. The extended workflow adds the slower sanitizer,
platform and sample coverage described below. A known failing check must be investigated;
moving a check off the PR path does not make its findings optional. Performance work is
very welcome, but it has to produce the same bits.

## Fast and extended CI

PR and main-branch checks target **1–2 minutes**, excluding runner queue time. The
`build_test` workflow runs all 24 suites in five configurations: Linux GCC Debug,
Linux Clang Debug with ThreadSanitizer, Linux wide-position Debug, Windows optimized
clang-cl and macOS optimized NEON. `build_test result` fails if a job fails, times out
or unexpectedly skips. The contributor-agreement and vendor-drift checks also remain
on PRs. A three-minute job backstop allows runner variance; it is not a new timing target.

The manually triggered **Extended checks** workflow preserves the complete previous
matrix: MemorySanitizer, macOS ASan/UBSan, wide-position ASan/UBSan, Windows Debug and
ARM64, MinGW, AVX-512, Emscripten compile coverage, every static/dynamic sample build,
and the fixed/float conversion audit. It also reruns the common core configurations.
Its aggregate result fails on failed, cancelled or skipped jobs; long sanitizer runs
retain their existing explicit time limits.

Run extended coverage when relevant to an arithmetic, concurrency, portability,
sample or rendering change, and investigate any failures before calling the change
validated. In GitHub Actions, select **Extended checks → Run workflow** and choose
the branch, or use:

```sh
gh workflow run extended.yml --repo mas-bandwidth/fixed3d --ref <branch>
```

The split was based on measured execution times: the conversion audit job took 7m47s,
wide-position sanitizers 3m29s, macOS sanitizers 3m09s, MinGW 3m01s and Windows sample
builds 2m30s–2m41s. MemorySanitizer is substantially longer. Keep later slow additions
in the extended workflow and measure PR time after changing the fast matrix.

## Opening a pull request

1. Fork the repository and branch from `main`.
2. Keep the pull request to one logical change. If you find yourself touching unrelated
   things, split them into separate pull requests.
3. Say in the description what the change does and why.
4. Build and test locally before you send it:

   ```bash
   ./build.sh
   ./build/bin/test
   ```

   Then make sure CI is green on the pull request.
5. Match the surrounding code. Library sources are C17, samples are C++20, and CI builds
   with warnings as errors. The repository ships a `.clang-format`; use it on code you add
   or edit. Please do not mass-reformat files you are not otherwise changing, because it
   buries the real change.

## Contributor Assignment Agreement

Before your first contribution can be merged you need to sign the
[Contributor Assignment Agreement](https://github.com/mas-bandwidth/.github/blob/main/CAA.md).
A bot posts the link on your first pull request. Signing is one comment, it takes a
minute, and it covers everything you contribute to any Más Bandwidth repository after
that.

Read it before you sign. It is a copyright assignment, a stronger grant than the license a
CLA usually asks for: you assign copyright in your contribution to Más Bandwidth LLC, and
you get back a perpetual license to use your own work for any purpose. We ask for it so
these libraries can be relicensed in future without tracking down every past contributor
for permission. If your contribution includes third-party material, identify it when you
submit; that material stays under its own license.

Fixed3D is MIT, the same as Box3D, and the Box3D-derived material stays MIT.

## Credit

Contributors keep their name in the commit history, and a change worth calling out gets
called out. The [README](README.md) carries the crediting request that goes the other way,
for products that ship Fixed3D.

## Questions

Open an [issue](https://github.com/mas-bandwidth/fixed3d/issues).
