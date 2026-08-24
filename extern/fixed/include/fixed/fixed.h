// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

// The 128-bit seam: fixInt128/fixUInt128 and their operation vocabulary, native
// __int128 where the compiler has it and an emulated pair where it does not (plain
// MSVC). Also carries this library's FIX_ALWAYS_INLINE definition, because the seam is
// the first thing every other header needs. This header stays self-contained.
#include "fixed/fixed_int128.h"

/**
 * @defgroup fixed Fixed-point scalar
 * @brief Deterministic Q48.16 fixed-point scalar type and operations
 *
 * Fixed3D does all math in fixed point. A scalar is a signed 64 bit integer holding
 * a Q48.16 value: 48 integer bits and 16 fraction bits. The resolution is
 * 1/65536 (about 1.5e-5) uniformly across the entire range, so precision does not
 * degrade far from the origin. All operations are integer operations and are exactly
 * reproducible across platforms and compilers.
 *
 * Multiplication and division use 128 bit intermediates and saturate on overflow
 * instead of wrapping. Division by zero saturates with the sign of the numerator
 * (and 0/0 == 0), which plays the role of infinity in the float version.
 * @{
 */

/// A Q48.16 fixed-point number stored in a signed 64 bit integer.
/// Addition, subtraction, negation, and comparison are the plain integer operations.
/// Use fixMul and fixDiv for multiplication and division.
typedef int64_t fixed_t;

/// Fraction bits in a fixed_t
#define FIX_FRACTION_BITS 16

/// One in fixed point
#define FIX_ONE ( (fixed_t)1 << FIX_FRACTION_BITS )

/// One half in fixed point
#define FIX_HALF ( (fixed_t)1 << ( FIX_FRACTION_BITS - 1 ) )

/// The smallest representable positive increment (analog of FLT_EPSILON at 1.0,
/// but fixed point has uniform absolute resolution everywhere)
#define FIX_EPSILON ( (fixed_t)1 )

/// Saturation value, plays the role of FLT_MAX / infinity
#define FIX_MAX INT64_MAX

/// Negative saturation value
#define FIX_MIN ( -INT64_MAX )

/// Convert a numeric literal to fixed point at compile time, e.g. FIX( 1.5f ).
/// Works in static initializers. Rounds to nearest.
#define FIX( x )                                                                                                              \
	( (fixed_t)( (double)( x ) * (double)FIX_ONE + ( (double)( x ) >= 0.0 ? 0.5 : -0.5 ) ) )

/// Left shift that is well defined for negative values (two's complement wrap).
/// C makes shifting a negative value undefined; routing through unsigned keeps
/// the same bits and keeps UBSan quiet.
FIX_ALWAYS_INLINE fixed_t fixShiftLeft( fixed_t a, int shift )
{
	return (fixed_t)( (uint64_t)a << shift );
}

/// 128-bit variant of fixShiftLeft
FIX_ALWAYS_INLINE fixInt128 fixInt128ShiftLeft( fixInt128 a, int shift )
{
	return fixInt128FromUnsigned( fixUInt128Shl( fixInt128ToUnsigned( a ), shift ) );
}

/// @return the minimum of two 128-bit values.
FIX_ALWAYS_INLINE fixInt128 fixInt128Min( fixInt128 a, fixInt128 b )
{
	return fixInt128Lt( a, b ) ? a : b;
}

/// @return the maximum of two 128-bit values.
FIX_ALWAYS_INLINE fixInt128 fixInt128Max( fixInt128 a, fixInt128 b )
{
	return fixInt128Gt( a, b ) ? a : b;
}

/// Exact signed 128-bit division. Bit-identical to the compiler's __divti3 for
/// every input: integer division has a unique truncating result, so any exact
/// algorithm agrees. On x86-64 the case the solver almost always produces --
/// divisor fits in 64 bits and the quotient provably fits in 64 bits -- runs
/// as a single hardware divide instruction instead of the generic 128-bit
/// library loop (measured 3.9x on Zen 4; zero mismatches in a 20M-case
/// differential fuzz on both ISAs). Apple silicon's library call is already
/// within 8% of a hand-written Knuth divide, so non-x86 targets keep the
/// plain division.
/// On the emulated arm (plain MSVC, or any build with FIX_FORCE_EMULATED_INT128) the
/// shift-subtract divide in fixed_int128.h is the whole implementation: the fast paths
/// below are native-type transformations with nothing to accelerate.
FIX_ALWAYS_INLINE fixInt128 fixInt128Div( fixInt128 a, fixInt128 b )
{
#if FIX_INT128_EMULATED
	return fixEmuInt128Div( a, b );
#else
#if defined( __x86_64__ )
	fixUInt128 ua = a < 0 ? -(fixUInt128)a : (fixUInt128)a;
	fixUInt128 ub = b < 0 ? -(fixUInt128)b : (fixUInt128)b;
	if ( ( ub >> 64 ) == 0 )
	{
		uint64_t v = (uint64_t)ub;
		uint64_t uhi = (uint64_t)( ua >> 64 );
		if ( uhi < v )
		{
			uint64_t ulo = (uint64_t)ua;
			uint64_t q;
			if ( uhi == 0 )
			{
				q = ulo / v;
			}
			else
			{
				// Hardware 128/64 divide; uhi < v proves the quotient fits in
				// 64 bits, so the instruction cannot fault ON THIS PATH.
				// volatile is load-bearing: divq traps on quotient overflow,
				// and gcc assumes a non-volatile asm is side-effect-free and
				// safe to speculate, hoisting it ABOVE the uhi < v guard and
				// the sign-magnitude negation (observed with gcc 13 -mavx512*:
				// SIGFPE in ManifoldTest from a divide fed the raw negative
				// bits of a dividend whose true quotient was -1). volatile
				// pins the instruction to this branch.
				uint64_t rem;
				__asm__ volatile( "divq %[v]" : "=a"( q ), "=d"( rem ) : [v] "r"( v ), "a"( ulo ), "d"( uhi ) );
				(void)rem;
			}
			return ( a < 0 ) != ( b < 0 ) ? -(fixInt128)q : (fixInt128)q;
		}
	}
	// Divisor beyond 64 bits, quotient beyond 64 bits, or division by zero:
	// fall through to the generic path (identical behavior in all three).
#endif
#if defined( _WIN32 )
	// ClangCL does not link compiler-rt builtins, so native 128-bit division
	// (__divti3) is unavailable. Restoring shift-subtract division: bit-identical
	// results, and this path is cold -- the fast paths above catch simulation
	// workloads. Every caller guards division by zero; if reached anyway it
	// returns 0 deterministically instead of trapping.
	{
		fixUInt128 un = a < 0 ? -(fixUInt128)a : (fixUInt128)a;
		fixUInt128 ud = b < 0 ? -(fixUInt128)b : (fixUInt128)b;
		if ( ud == 0 )
		{
			return 0;
		}
		fixUInt128 q = 0;
		fixUInt128 r = 0;
		for ( int i = 127; i >= 0; i-- )
		{
			r = ( r << 1 ) | ( ( un >> i ) & 1 );
			if ( r >= ud )
			{
				r -= ud;
				q |= (fixUInt128)1 << i;
			}
		}
		return ( a < 0 ) != ( b < 0 ) ? -(fixInt128)q : (fixInt128)q;
	}
#else
	return a / b;
#endif
#endif
}

/// Multiply two fixed-point numbers with round-to-nearest.
/// By default the product is not checked for overflow: simulation quantities are
/// far below the +/-1.4e14 range and the checks cost real time in the solver.
/// Define FIX_SATURATE to saturate on overflow instead of wrapping. (box3d passes its
/// own BOX3D_FIXED_SATURATE down through its compatibility header.)
FIX_ALWAYS_INLINE fixed_t fixMul( fixed_t a, fixed_t b )
{
	fixInt128 product = fixInt128MulI64( a, b );
	// Round half up, then shift out the fraction bits (arithmetic shift)
	fixInt128 r = fixInt128Shr( fixInt128Add( product, fixInt128FromI64( FIX_HALF ) ), FIX_FRACTION_BITS );
#if defined( FIX_SATURATE )
	if ( fixInt128Gt( r, fixInt128FromI64( INT64_MAX ) ) )
	{
		return FIX_MAX;
	}
	if ( fixInt128Lt( r, fixInt128FromI64( -INT64_MAX ) ) )
	{
		return FIX_MIN;
	}
#endif
	return (fixed_t)fixInt128ToI64( r );
}

/// Divide two fixed-point numbers with truncation toward zero and saturation.
/// Division by zero saturates with the sign of the numerator; 0/0 == 0.
FIX_ALWAYS_INLINE fixed_t fixDiv( fixed_t a, fixed_t b )
{
	if ( b == 0 )
	{
		return a > 0 ? FIX_MAX : ( a < 0 ? FIX_MIN : 0 );
	}

	// Fast path: when the numerator fits in 64 bits (nearly always) a single
	// hardware divide replaces the 128-bit library call. Bit-identical result.
	if ( -( (int64_t)1 << 47 ) < a && a < ( (int64_t)1 << 47 ) )
	{
		return fixShiftLeft( a, FIX_FRACTION_BITS ) / b;
	}

	fixInt128 q = fixInt128Div( fixInt128ShiftLeft( fixInt128FromI64( a ), FIX_FRACTION_BITS ), fixInt128FromI64( b ) );
	if ( fixInt128Gt( q, fixInt128FromI64( INT64_MAX ) ) )
	{
		return FIX_MAX;
	}
	if ( fixInt128Lt( q, fixInt128FromI64( -INT64_MAX ) ) )
	{
		return FIX_MIN;
	}
	return (fixed_t)fixInt128ToI64( q );
}

// The seed for the exact integer square root below. gcc and clang compile
// __builtin_sqrt to the hardware instruction; plain MSVC has no such builtin and uses
// <math.h>'s sqrt. Either spelling is only a STARTING POINT -- the repair loops that
// follow drive the result to the exact integer floor from any nearby seed -- so the
// choice cannot move a bit of output, and the frozen determinism hashes hold that.
// Internal to this header; not part of the overridable macro set in base.h.
#if defined( _MSC_VER ) && !defined( __clang__ )
	#include <math.h>
	#define FIX_SQRT_SEED( x ) sqrt( x )
#else
	#define FIX_SQRT_SEED( x ) __builtin_sqrt( x )
#endif

/// Exact integer square root of an unsigned 128 bit value (helper for fixSqrt).
FIX_ALWAYS_INLINE uint64_t fixISqrt128High( uint64_t hi, uint64_t lo )
{
	if ( hi == 0 )
	{
		// Common case: 64 bit input. Seed with the hardware double sqrt (an exact,
		// correctly rounded IEEE operation) and repair to the exact integer floor.
		// The result is exact on every platform, so determinism is preserved.
		if ( lo == 0 )
		{
			return 0;
		}

		uint64_t r = (uint64_t)FIX_SQRT_SEED( (double)lo );
		if ( r > 0xFFFFFFFFu )
		{
			r = 0xFFFFFFFFu;
		}
		while ( r > 0 && fixUInt128Gt( fixUInt128MulU64( r, r ), fixUInt128FromU64( lo ) ) )
		{
			r -= 1;
		}
		while ( fixUInt128Le( fixUInt128MulU64( r + 1, r + 1 ), fixUInt128FromU64( lo ) ) )
		{
			r += 1;
		}
		return r;
	}

	// Rare 128 bit case: restoring shift-subtract square root
	fixUInt128 n = fixUInt128Make( hi, lo );
	fixUInt128 x = n;
	fixUInt128 c = FIX_UINT128_ZERO;
	fixUInt128 d = fixUInt128Shl( fixUInt128FromU64( 1 ), 126 );
	while ( fixUInt128Gt( d, n ) )
	{
		d = fixUInt128Shr( d, 2 );
	}

	while ( !fixUInt128Eq( d, FIX_UINT128_ZERO ) )
	{
		fixUInt128 cd = fixUInt128Add( c, d );
		if ( fixUInt128Ge( x, cd ) )
		{
			x = fixUInt128Sub( x, cd );
			c = fixUInt128Add( fixUInt128Shr( c, 1 ), d );
		}
		else
		{
			c = fixUInt128Shr( c, 1 );
		}
		d = fixUInt128Shr( d, 2 );
	}

	return fixUInt128Lo( c );
}

/// Fixed-point square root. Exact (round toward zero). Negative inputs return 0.
FIX_ALWAYS_INLINE fixed_t fixSqrt( fixed_t a )
{
	if ( a <= 0 )
	{
		return 0;
	}

	// sqrt( a / 2^16 ) * 2^16 = sqrt( a * 2^16 )
	uint64_t hi = (uint64_t)a >> ( 64 - FIX_FRACTION_BITS );
	uint64_t lo = (uint64_t)a << FIX_FRACTION_BITS;
	return (fixed_t)fixISqrt128High( hi, lo );
}

/// Absolute value
FIX_ALWAYS_INLINE fixed_t fixAbs( fixed_t a )
{
	return a < 0 ? -a : a;
}

/// Minimum of two fixed-point numbers
FIX_ALWAYS_INLINE fixed_t fixMin( fixed_t a, fixed_t b )
{
	return a < b ? a : b;
}

/// Maximum of two fixed-point numbers
FIX_ALWAYS_INLINE fixed_t fixMax( fixed_t a, fixed_t b )
{
	return a > b ? a : b;
}

/// Clamp a fixed-point number between a lower and upper bound
FIX_ALWAYS_INLINE fixed_t fixClamp( fixed_t a, fixed_t lower, fixed_t upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

/// Convert an integer to fixed point
FIX_ALWAYS_INLINE fixed_t fixFromInt( int64_t i )
{
	return fixShiftLeft( (fixed_t)i, FIX_FRACTION_BITS );
}

/// Convert fixed point to an integer, truncating toward zero (C float-to-int semantics)
FIX_ALWAYS_INLINE int fixTruncToInt( fixed_t a )
{
	return (int)( a / FIX_ONE );
}

/// Convert fixed point to an integer, rounding toward negative infinity
FIX_ALWAYS_INLINE int fixFloorToInt( fixed_t a )
{
	return (int)( a >> FIX_FRACTION_BITS );
}

/// Convert fixed point to an integer, rounding to nearest
FIX_ALWAYS_INLINE int fixRoundToInt( fixed_t a )
{
	// The add is unsigned so the extreme-edge wrap (a near FIX_MAX) is
	// defined two's-complement behavior instead of signed-overflow UB. Same
	// bits on every compiler; the frozen hashes pin it.
	return (int)( (fixed_t)( (uint64_t)a + FIX_HALF ) >> FIX_FRACTION_BITS );
}

/// Largest integral fixed-point value not greater than a (like floorf)
FIX_ALWAYS_INLINE fixed_t fixFloor( fixed_t a )
{
	return a & ~( FIX_ONE - 1 );
}

/// Smallest integral fixed-point value not less than a (like ceilf)
FIX_ALWAYS_INLINE fixed_t fixCeil( fixed_t a )
{
	// Unsigned add: defined wrap at the extreme edge (see fixRoundToInt)
	return (fixed_t)( (uint64_t)a + FIX_ONE - 1 ) & ~( FIX_ONE - 1 );
}

// Conversions to and from floating point. These are boundary helpers for graphics,
// logging, and timing. The simulation itself never uses them.

/// Convert fixed point to float (for rendering, logging, and other non-simulation uses)
FIX_ALWAYS_INLINE float fixToFloat( fixed_t a )
{
	return (float)( (double)a * ( 1.0 / (double)FIX_ONE ) );
}

/// Convert fixed point to double. Exact.
FIX_ALWAYS_INLINE double fixToDouble( fixed_t a )
{
	return (double)a * ( 1.0 / (double)FIX_ONE );
}

/// Convert a float to fixed point, rounding to nearest. A boundary helper: results
/// depend on the float input, so keep this out of deterministic simulation code.
FIX_ALWAYS_INLINE fixed_t fixFromFloat( float x )
{
	double d = (double)x * (double)FIX_ONE;
	return (fixed_t)( d >= 0.0 ? d + 0.5 : d - 0.5 );
}

/// Convert a double to fixed point, rounding to nearest. A boundary helper.
FIX_ALWAYS_INLINE fixed_t fixFromDouble( double x )
{
	double d = x * (double)FIX_ONE;
	return (fixed_t)( d >= 0.0 ? d + 0.5 : d - 0.5 );
}

//! @cond

// ---------------------------------------------------------------------------------------------
// Q2.30 packed components (fixed30_t)
//
// A SECOND raw fixed-point domain: 32-bit storage, 2 integer bits (sign included), 30 fraction
// bits. Built for always-normalized quantities (quaternion components never leave [-1, 1], so
// nearly all bits go to fraction). The raw value is wrapped in a struct ON PURPOSE: fixed_t is
// a bare int64 typedef and Q48.16/Q2.30 raws differ by 2^14, so a bare 32-bit typedef would
// let a mixup compile silently -- the same trap as assigning a double into a fixed_t. With the
// struct, arithmetic and cross-domain assignment refuse to compile; go through the converters.
//
// ROUNDING RULE (pinned): dropping bits rounds to nearest via ( raw + ( 1 << ( drop - 1 ) ) ) >> drop,
// the same form as the floor( v * scale + 0.5 ) wire family.
// ---------------------------------------------------------------------------------------------

//! @endcond

/// Number of fractional bits in a Q2.30 packed component.
#define FIX30_FRACTION_BITS 30

/// One, in Q2.30.
#define FIX30_ONE ( (int32_t)1 << FIX30_FRACTION_BITS )

/// The shift between the Q48.16 and Q2.30 raw domains.
#define FIX30_SHIFT ( FIX30_FRACTION_BITS - FIX_FRACTION_BITS )

/// A Q2.30 packed fixed-point component: 32-bit storage, 2 integer bits (sign included),
/// 30 fraction bits. Domain [-2, 2); built for always-normalized quantities. Deliberately a
/// struct so a Q48.16/Q2.30 raw mixup cannot compile silently.
typedef struct fixed30_t
{
	int32_t raw;
} fixed30_t;

/// Pack Q48.16 to Q2.30. Gains 14 fraction bits (exact for in-range values); values outside
/// [-2, 2) saturate to the domain bounds.
FIX_ALWAYS_INLINE fixed30_t fix30FromFix( fixed_t a )
{
	// Saturation is tested BEFORE the shift. Shifting first and testing after -- which is
	// what this looked like in box3d -- is signed overflow for |a| >= 2^49, and the shifted
	// value is discarded in exactly those cases, so the order is free. Same answer for
	// every input, no undefined behavior. The in-range shift routes through fixShiftLeft
	// because a is signed and may be negative.
	if ( a >= ( (fixed_t)2 << FIX_FRACTION_BITS ) )
	{
		fixed30_t saturated = { INT32_MAX };
		return saturated;
	}

	if ( a < -( (fixed_t)2 << FIX_FRACTION_BITS ) )
	{
		fixed30_t saturated = { INT32_MIN };
		return saturated;
	}

	fixed30_t result = { (int32_t)fixShiftLeft( a, FIX30_SHIFT ) };
	return result;
}

/// Unpack Q2.30 to Q48.16. Drops 14 fraction bits, rounding to nearest per the pinned rule.
FIX_ALWAYS_INLINE fixed_t fixFromFix30( fixed30_t a )
{
	return ( (fixed_t)a.raw + ( (fixed_t)1 << ( FIX30_SHIFT - 1 ) ) ) >> FIX30_SHIFT;
}

/// Convert Q2.30 to a double. Exact: every Q2.30 value is representable in a double.
FIX_ALWAYS_INLINE double fix30ToDouble( fixed30_t a )
{
	return (double)a.raw / (double)FIX30_ONE;
}

/// Convert a double to Q2.30, rounding to nearest, saturating to the domain. A boundary
/// helper, and like the other double boundary helpers it takes a finite input.
FIX_ALWAYS_INLINE fixed30_t fix30FromDouble( double x )
{
	double d = x * (double)FIX30_ONE;
	d = ( d >= 0.0 ? d + 0.5 : d - 0.5 );
	if ( d >= (double)INT32_MAX )
	{
		d = (double)INT32_MAX;
	}
	else if ( d <= (double)INT32_MIN )
	{
		d = (double)INT32_MIN;
	}
	fixed30_t result = { (int32_t)d };
	return result;
}

/**@}*/ // fixed
