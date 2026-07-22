// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

// This header is self-contained so that base.h can include it before the rest
// of the API macros are defined.
#ifndef B3_INLINE
#if defined( _MSC_VER )
#define B3_FIXED_INLINE static __forceinline
#elif defined( __GNUC__ ) || defined( __clang__ )
#define B3_FIXED_INLINE static inline __attribute__( ( always_inline ) )
#else
#define B3_FIXED_INLINE static inline
#endif
#else
#define B3_FIXED_INLINE B3_FORCE_INLINE
#endif

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
/// Use b3FixMul and b3FixDiv for multiplication and division.
typedef int64_t b3Fixed;

/// Fraction bits in a b3Fixed
#define B3_FIXED_FRACTION_BITS 16

/// One in fixed point
#define B3_FIXED_ONE ( (b3Fixed)1 << B3_FIXED_FRACTION_BITS )

/// One half in fixed point
#define B3_FIXED_HALF ( (b3Fixed)1 << ( B3_FIXED_FRACTION_BITS - 1 ) )

/// The smallest representable positive increment (analog of FLT_EPSILON at 1.0,
/// but fixed point has uniform absolute resolution everywhere)
#define B3_FIXED_EPSILON ( (b3Fixed)1 )

/// Saturation value, plays the role of FLT_MAX / infinity
#define B3_FIXED_MAX INT64_MAX

/// Negative saturation value
#define B3_FIXED_MIN ( -INT64_MAX )

/// Convert a numeric literal to fixed point at compile time, e.g. B3_FIX( 1.5f ).
/// Works in static initializers. Rounds to nearest.
#define B3_FIX( x )                                                                                                              \
	( (b3Fixed)( (double)( x ) * (double)B3_FIXED_ONE + ( (double)( x ) >= 0.0 ? 0.5 : -0.5 ) ) )

// 128 bit helper layer. The fixed-point core requires the __int128 compiler
// extension (clang, gcc, or clang-cl on Windows; pure MSVC does not have it).
// __extension__ keeps -Wpedantic quiet: __int128 is not ISO C.
#if defined( __SIZEOF_INT128__ )

#define B3_HAS_INT128 1
__extension__ typedef __int128 b3Int128;
__extension__ typedef unsigned __int128 b3UInt128;

#else

#error "Fixed3D fixed point math requires __int128: use clang, gcc, or clang-cl on Windows"

#endif

/// Left shift that is well defined for negative values (two's complement wrap).
/// C makes shifting a negative value undefined; routing through unsigned keeps
/// the same bits and keeps UBSan quiet.
B3_FIXED_INLINE b3Fixed b3FixShiftLeft( b3Fixed a, int shift )
{
	return (b3Fixed)( (uint64_t)a << shift );
}

/// 128-bit variant of b3FixShiftLeft
B3_FIXED_INLINE b3Int128 b3Int128ShiftLeft( b3Int128 a, int shift )
{
	return (b3Int128)( (b3UInt128)a << shift );
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
B3_FIXED_INLINE b3Int128 b3Int128Div( b3Int128 a, b3Int128 b )
{
#if defined( __x86_64__ )
	b3UInt128 ua = a < 0 ? -(b3UInt128)a : (b3UInt128)a;
	b3UInt128 ub = b < 0 ? -(b3UInt128)b : (b3UInt128)b;
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
			return ( a < 0 ) != ( b < 0 ) ? -(b3Int128)q : (b3Int128)q;
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
		b3UInt128 un = a < 0 ? -(b3UInt128)a : (b3UInt128)a;
		b3UInt128 ud = b < 0 ? -(b3UInt128)b : (b3UInt128)b;
		if ( ud == 0 )
		{
			return 0;
		}
		b3UInt128 q = 0;
		b3UInt128 r = 0;
		for ( int i = 127; i >= 0; i-- )
		{
			r = ( r << 1 ) | ( ( un >> i ) & 1 );
			if ( r >= ud )
			{
				r -= ud;
				q |= (b3UInt128)1 << i;
			}
		}
		return ( a < 0 ) != ( b < 0 ) ? -(b3Int128)q : (b3Int128)q;
	}
#else
	return a / b;
#endif
}

/// Multiply two fixed-point numbers with round-to-nearest.
/// By default the product is not checked for overflow: simulation quantities are
/// far below the +/-1.4e14 range and the checks cost real time in the solver.
/// Define BOX3D_FIXED_SATURATE to saturate on overflow instead of wrapping.
B3_FIXED_INLINE b3Fixed b3FixMul( b3Fixed a, b3Fixed b )
{
	b3Int128 product = (b3Int128)a * b;
	// Round half up, then shift out the fraction bits (arithmetic shift)
	b3Int128 r = ( product + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS;
#if defined( BOX3D_FIXED_SATURATE )
	if ( r > (b3Int128)INT64_MAX )
	{
		return B3_FIXED_MAX;
	}
	if ( r < -(b3Int128)INT64_MAX )
	{
		return B3_FIXED_MIN;
	}
#endif
	return (b3Fixed)r;
}

/// Divide two fixed-point numbers with truncation toward zero and saturation.
/// Division by zero saturates with the sign of the numerator; 0/0 == 0.
B3_FIXED_INLINE b3Fixed b3FixDiv( b3Fixed a, b3Fixed b )
{
	if ( b == 0 )
	{
		return a > 0 ? B3_FIXED_MAX : ( a < 0 ? B3_FIXED_MIN : 0 );
	}

	// Fast path: when the numerator fits in 64 bits (nearly always) a single
	// hardware divide replaces the 128-bit library call. Bit-identical result.
	if ( -( (int64_t)1 << 47 ) < a && a < ( (int64_t)1 << 47 ) )
	{
		return b3FixShiftLeft( a, B3_FIXED_FRACTION_BITS ) / b;
	}

	b3Int128 q = b3Int128Div( b3Int128ShiftLeft( a, B3_FIXED_FRACTION_BITS ), b );
	if ( q > (b3Int128)INT64_MAX )
	{
		return B3_FIXED_MAX;
	}
	if ( q < -(b3Int128)INT64_MAX )
	{
		return B3_FIXED_MIN;
	}
	return (b3Fixed)q;
}

/// Exact integer square root of an unsigned 128 bit value (helper for b3FixSqrt).
B3_FIXED_INLINE uint64_t b3ISqrt128High( uint64_t hi, uint64_t lo )
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

		uint64_t r = (uint64_t)__builtin_sqrt( (double)lo );
		if ( r > 0xFFFFFFFFu )
		{
			r = 0xFFFFFFFFu;
		}
		while ( r > 0 && (b3UInt128)r * r > lo )
		{
			r -= 1;
		}
		while ( (b3UInt128)( r + 1 ) * ( r + 1 ) <= lo )
		{
			r += 1;
		}
		return r;
	}

	// Rare 128 bit case: restoring shift-subtract square root
	b3UInt128 n = ( (b3UInt128)hi << 64 ) | lo;
	b3UInt128 x = n;
	b3UInt128 c = 0;
	b3UInt128 d = (b3UInt128)1 << 126;
	while ( d > n )
	{
		d >>= 2;
	}

	while ( d != 0 )
	{
		if ( x >= c + d )
		{
			x -= c + d;
			c = ( c >> 1 ) + d;
		}
		else
		{
			c >>= 1;
		}
		d >>= 2;
	}

	return (uint64_t)c;
}

/// Fixed-point square root. Exact (round toward zero). Negative inputs return 0.
B3_FIXED_INLINE b3Fixed b3FixSqrt( b3Fixed a )
{
	if ( a <= 0 )
	{
		return 0;
	}

	// sqrt( a / 2^16 ) * 2^16 = sqrt( a * 2^16 )
	uint64_t hi = (uint64_t)a >> ( 64 - B3_FIXED_FRACTION_BITS );
	uint64_t lo = (uint64_t)a << B3_FIXED_FRACTION_BITS;
	return (b3Fixed)b3ISqrt128High( hi, lo );
}

/// Absolute value
B3_FIXED_INLINE b3Fixed b3FixAbs( b3Fixed a )
{
	return a < 0 ? -a : a;
}

/// Minimum of two fixed-point numbers
B3_FIXED_INLINE b3Fixed b3FixMin( b3Fixed a, b3Fixed b )
{
	return a < b ? a : b;
}

/// Maximum of two fixed-point numbers
B3_FIXED_INLINE b3Fixed b3FixMax( b3Fixed a, b3Fixed b )
{
	return a > b ? a : b;
}

/// Clamp a fixed-point number between a lower and upper bound
B3_FIXED_INLINE b3Fixed b3FixClamp( b3Fixed a, b3Fixed lower, b3Fixed upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

/// Convert an integer to fixed point
B3_FIXED_INLINE b3Fixed b3FixFromInt( int64_t i )
{
	return b3FixShiftLeft( (b3Fixed)i, B3_FIXED_FRACTION_BITS );
}

/// Convert fixed point to an integer, truncating toward zero (C float-to-int semantics)
B3_FIXED_INLINE int b3FixTruncToInt( b3Fixed a )
{
	return (int)( a / B3_FIXED_ONE );
}

/// Convert fixed point to an integer, rounding toward negative infinity
B3_FIXED_INLINE int b3FixFloorToInt( b3Fixed a )
{
	return (int)( a >> B3_FIXED_FRACTION_BITS );
}

/// Convert fixed point to an integer, rounding to nearest
B3_FIXED_INLINE int b3FixRoundToInt( b3Fixed a )
{
	// The add is unsigned so the extreme-edge wrap (a near B3_FIXED_MAX) is
	// defined two's-complement behavior instead of signed-overflow UB. Same
	// bits on every compiler; the frozen hashes pin it.
	return (int)( (b3Fixed)( (uint64_t)a + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS );
}

/// Largest integral fixed-point value not greater than a (like floorf)
B3_FIXED_INLINE b3Fixed b3FixFloor( b3Fixed a )
{
	return a & ~( B3_FIXED_ONE - 1 );
}

/// Smallest integral fixed-point value not less than a (like ceilf)
B3_FIXED_INLINE b3Fixed b3FixCeil( b3Fixed a )
{
	// Unsigned add: defined wrap at the extreme edge (see b3FixRoundToInt)
	return (b3Fixed)( (uint64_t)a + B3_FIXED_ONE - 1 ) & ~( B3_FIXED_ONE - 1 );
}

// Conversions to and from floating point. These are boundary helpers for graphics,
// logging, and timing. The simulation itself never uses them.

/// Convert fixed point to float (for rendering, logging, and other non-simulation uses)
B3_FIXED_INLINE float b3FixToFloat( b3Fixed a )
{
	return (float)( (double)a * ( 1.0 / (double)B3_FIXED_ONE ) );
}

/// Convert fixed point to double. Exact.
B3_FIXED_INLINE double b3FixToDouble( b3Fixed a )
{
	return (double)a * ( 1.0 / (double)B3_FIXED_ONE );
}

/// Convert a float to fixed point, rounding to nearest. A boundary helper: results
/// depend on the float input, so keep this out of deterministic simulation code.
B3_FIXED_INLINE b3Fixed b3FixFromFloat( float x )
{
	double d = (double)x * (double)B3_FIXED_ONE;
	return (b3Fixed)( d >= 0.0 ? d + 0.5 : d - 0.5 );
}

/// Convert a double to fixed point, rounding to nearest. A boundary helper.
B3_FIXED_INLINE b3Fixed b3FixFromDouble( double x )
{
	double d = x * (double)B3_FIXED_ONE;
	return (b3Fixed)( d >= 0.0 ? d + 0.5 : d - 0.5 );
}

/**@}*/ // fixed
