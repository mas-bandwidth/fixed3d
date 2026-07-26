// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
//
// Box3D's names for the scalar core of mas-bandwidth/fixed.
//
// The Q48.16 type and its arithmetic used to be defined here. They now live in the
// `fixed` library, vendored at extern/fixed and pinned in tools/revendor-fixed.sh,
// where they are called fixed_t, fixMul, FIX(x). box3d goes on calling them b3Fixed,
// b3FixMul and B3_FIX, because STAYING DROP-IN COMPATIBLE WITH BOX3D IS A DESIGN GOAL
// of this fork -- the public interface is not ours to evolve to suit a dependency.
//
// GENERATED. Every name below was read out of the previous box3d/fixed.h and mapped
// through the rename that moved the library off the b3 prefix, so the set is exactly
// what box3d defined before -- no more, no less.
//
// Forwarders rather than #defines: a macro would stop b3FixMul being a declared
// identifier, so debuggers would show fixMul and a consumer could no longer name
// anything b3FixMul. All of these were header-inline in box3d and stay header-inline,
// so linkage is unchanged; after inlining the wrapper costs nothing.
#pragma once

#include "fixed/fixed.h"

#if defined( BOX3D_FIXED_SATURATE ) && !defined( FIX_SATURATE )
	#define FIX_SATURATE
#endif

// ---- types ------------------------------------------------------------------------
typedef fixed_t                b3Fixed;
typedef fixInt128              b3Int128;
typedef fixUInt128             b3UInt128;

// ---- macros -----------------------------------------------------------------------
#define B3_FIX                     FIX
#define B3_FIXED_EPSILON           FIX_EPSILON
#define B3_FIXED_FRACTION_BITS     FIX_FRACTION_BITS
#define B3_FIXED_HALF              FIX_HALF
#define B3_FIXED_INLINE            FIX_ALWAYS_INLINE
#define B3_FIXED_MAX               FIX_MAX
#define B3_FIXED_MIN               FIX_MIN
#define B3_FIXED_ONE               FIX_ONE
#define B3_HAS_INT128              FIX_HAS_INT128

// ---- forwarders -------------------------------------------------------------------
B3_FIXED_INLINE fixed_t b3FixAbs( fixed_t a ) { return fixAbs( a ); }
B3_FIXED_INLINE fixed_t b3FixCeil( fixed_t a ) { return fixCeil( a ); }
B3_FIXED_INLINE fixed_t b3FixClamp( fixed_t a, fixed_t lower, fixed_t upper ) { return fixClamp( a, lower, upper ); }
B3_FIXED_INLINE fixed_t b3FixDiv( fixed_t a, fixed_t b ) { return fixDiv( a, b ); }
B3_FIXED_INLINE fixed_t b3FixFloor( fixed_t a ) { return fixFloor( a ); }
B3_FIXED_INLINE int b3FixFloorToInt( fixed_t a ) { return fixFloorToInt( a ); }
B3_FIXED_INLINE fixed_t b3FixFromDouble( double x ) { return fixFromDouble( x ); }
B3_FIXED_INLINE fixed_t b3FixFromFloat( float x ) { return fixFromFloat( x ); }
B3_FIXED_INLINE fixed_t b3FixFromInt( int64_t i ) { return fixFromInt( i ); }
B3_FIXED_INLINE fixed_t b3FixMax( fixed_t a, fixed_t b ) { return fixMax( a, b ); }
B3_FIXED_INLINE fixed_t b3FixMin( fixed_t a, fixed_t b ) { return fixMin( a, b ); }
B3_FIXED_INLINE fixed_t b3FixMul( fixed_t a, fixed_t b ) { return fixMul( a, b ); }
B3_FIXED_INLINE int b3FixRoundToInt( fixed_t a ) { return fixRoundToInt( a ); }
B3_FIXED_INLINE fixed_t b3FixShiftLeft( fixed_t a, int shift ) { return fixShiftLeft( a, shift ); }
B3_FIXED_INLINE fixed_t b3FixSqrt( fixed_t a ) { return fixSqrt( a ); }
B3_FIXED_INLINE double b3FixToDouble( fixed_t a ) { return fixToDouble( a ); }
B3_FIXED_INLINE float b3FixToFloat( fixed_t a ) { return fixToFloat( a ); }
B3_FIXED_INLINE int b3FixTruncToInt( fixed_t a ) { return fixTruncToInt( a ); }
B3_FIXED_INLINE uint64_t b3ISqrt128High( uint64_t hi, uint64_t lo ) { return fixISqrt128High( hi, lo ); }
B3_FIXED_INLINE fixInt128 b3Int128Div( fixInt128 a, fixInt128 b ) { return fixInt128Div( a, b ); }
B3_FIXED_INLINE fixInt128 b3Int128ShiftLeft( fixInt128 a, int shift ) { return fixInt128ShiftLeft( a, shift ); }

