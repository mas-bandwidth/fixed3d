// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// The AVX-512 variant (AVX512F/DQ/VL, four int64 lanes in a ymm register).
//
// This translation unit, and only this one, is compiled with
// -mavx512f -mavx512dq -mavx512vl (/arch:AVX512 on MSVC): src/CMakeLists.txt
// sets the flags on this file alone, never globally, so no AVX-512 instruction
// can leak into the baseline code that runs before dispatch has checked the
// host. It re-includes the kernel sections of the three sources that hold wide
// kernels (Highway's foreach_target pattern), so the scalar and the AVX-512
// variants are the same text: B3_SIMD_AVX512 selects the lane primitives, and
// B3_ISA_FN suffixes the entry points with _avx512. Everything outside the
// kernel sections compiles once, in the ordinary translation units.

#if defined( BOX3D_FIXED_SATURATE )

// Saturating builds hold only the scalar variant (src/core.h)
typedef int b3IsaAvx512NotInSaturatingBuilds;

#else

// B3_SIMD_AVX512 must be set before core.h is first included (it selects the
// lane primitives there), so the target test here repeats core.h's
// B3_ISA_HAVE_AVX512 condition instead of including it
#if !( defined( __x86_64__ ) || defined( _M_X64 ) ) || defined( __EMSCRIPTEN__ )
#error "src/isa_avx512.c is compiled only into x86-64 builds that hold the AVX-512 variant"
#endif

#define B3_ISA_VARIANT_TU
#define B3_SIMD_AVX512
#define B3_ISA_FN( name ) name##_avx512

#include "contact_solver.c"
#include "convex_manifold.c"
#include "hull.c"

#endif
