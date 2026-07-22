# fixed

A small, standalone, deterministic fixed-point math library. The core type is a
64-bit `b3Fixed` (Q48.16), with pure-integer arithmetic and integer-only
transcendentals — so results are **bit-identical on every platform and
architecture**, which floating point is not.

## Why

Deterministic lockstep and client/server simulations require every machine to
compute the same numbers from the same inputs. Floating point breaks this:
fused multiply-add, `libm` transcendentals, and `-ffp-contract` differ across
compilers and architectures. Fixed-point removes the whole class of divergence.

This library is extracted from the fixed-point core of
[box3d](https://github.com/erincatto/box3d) by Erin Catto (see `LICENSE`), so it
can be reused independently of the physics engine.

## Guarantee, checked in code

`test/determinism_test.c` runs the core ops over a deterministic input stream and
hashes every result. The hash is identical across arm64 and x86-64 (validated),
and CI extends that to Linux/libstdc++ and Windows/MSVC.

## Status

- [x] Core scalar type + arithmetic (`fixed.h`): mul, div, sqrt, abs, floor,
      ceil, clamp, conversions — extracted and cross-arch determinism-validated.
- [x] Integer transcendentals (`b3ComputeCosSin`, `b3Atan2`, `b3Sin`, `b3Cos`,
      `b3UnwindAngle`) — pure fixed-point, cross-arch determinism-validated, and
      correct vs libm (cos/sin < 0.0017, atan2 < 0.00004 rad).
- [ ] Vector / quaternion / matrix / transform / position types on `b3Fixed`.
- [ ] Fixed-point scalars and fixed-point time.
- [ ] box3d depends on `fixed` for its fixed-point core.

## License

MIT, © Erin Catto. See `LICENSE`.
