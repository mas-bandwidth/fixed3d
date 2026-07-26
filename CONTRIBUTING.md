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

The test suite checks this and CI enforces it: gcc, clang, clang-cl, MinGW and emscripten,
on Linux, macOS, and Windows x64 and ARM64, under thread, memory, address and
undefined-behavior sanitizers, plus AVX-512 and 128-bit world position variants. If your
change turns any of that red, it is not ready. Performance work is very welcome, but it
has to produce the same bits.

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
