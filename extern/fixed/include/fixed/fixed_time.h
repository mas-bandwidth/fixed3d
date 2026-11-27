// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
// Fixed-point time: Q32.32 seconds in an int64. 233 picosecond resolution,
// ±68 year range. Time gets more fraction bits than fixed_t because time
// accumulates — thousands of small dt adds per minute — and every add is
// exact in integer arithmetic, so the only error is the one-time rounding of
// dt itself. At Q32.32 that is ~2.3e-10 s per tick.
//
// THE OVERFLOW RULE is fixed_t's, operation by operation. Add, subtract, negate and
// multiply wrap in the default build (two's complement, through unsigned, so defined where
// a plain int64 `+` is not) and saturate under FIX_SATURATE, as fixVecAdd and fixMul do.
// Division and the conversion from fixed_t saturate in every build, as fixDiv does.
// Saturation lands on FIX_TIME_MIN/FIX_TIME_MAX: INT64_MIN is reserved, as it is for fixed_t.
#pragma once

#include "fixed/fixed.h"

/// Fixed-point time in seconds: Q32.32 in a 64-bit integer.
typedef int64_t fixTime;

#define FIX_TIME_FRACTION_BITS 32
#define FIX_TIME_ONE ( (fixTime)1 << FIX_TIME_FRACTION_BITS )
#define FIX_TIME_HALF ( (fixTime)1 << ( FIX_TIME_FRACTION_BITS - 1 ) )

/// Saturation value, FIX_MAX's role for time: just under 2^31 seconds.
#define FIX_TIME_MAX INT64_MAX

/// Negative saturation value. INT64_MIN is one below it and reserved, as for fixed_t.
#define FIX_TIME_MIN ( -INT64_MAX )

/// Convert a numeric literal to fixed-point time at compile time, e.g. FIX_TIME( 1.0 / 60.0 ).
#define FIX_TIME( x ) ( (fixTime)( ( x ) * (double)FIX_TIME_ONE + ( ( x ) >= 0 ? 0.5 : -0.5 ) ) )

/// Convert Q48.16 seconds to Q32.32 time. Exact (left shift) for |seconds| below 2^31 s;
/// past that the shift leaves int64 and the result saturates to FIX_TIME_MAX/FIX_TIME_MIN,
/// in every build, as fixDiv's range clamp does.
FIX_ALWAYS_INLINE fixTime fixTimeFromFixed( fixed_t seconds )
{
	const int shift = FIX_TIME_FRACTION_BITS - FIX_FRACTION_BITS;
	if ( seconds >= ( (fixed_t)1 << ( 63 - shift ) ) )
	{
		return FIX_TIME_MAX;
	}
	if ( seconds <= -( (fixed_t)1 << ( 63 - shift ) ) )
	{
		return FIX_TIME_MIN;
	}
	return (fixTime)( (uint64_t)seconds << shift );
}

/// Convert Q32.32 time to Q48.16 seconds, rounding to nearest.
FIX_ALWAYS_INLINE fixed_t fixTimeToFixed( fixTime t )
{
	// Split before rounding: the quotient fits even when t + half would not.
	const int shift = FIX_TIME_FRACTION_BITS - FIX_FRACTION_BITS;
	return ( t >> shift ) + ( ( t & ( ( (fixTime)1 << shift ) - 1 ) ) >= ( (fixTime)1 << ( shift - 1 ) ) );
}

/// Convert seconds (double) to Q32.32 time, rounding to nearest. Init/tooling
/// only — never in simulation, where cross-platform float behavior is the
/// enemy this library exists to remove.
FIX_ALWAYS_INLINE fixTime fixTimeFromSeconds( double seconds )
{
	double scaled = seconds * (double)FIX_TIME_ONE;
	return (fixTime)( scaled >= 0 ? scaled + 0.5 : scaled - 0.5 );
}

/// Convert Q32.32 time to seconds (double). Debug/display only.
FIX_ALWAYS_INLINE double fixTimeToSeconds( fixTime t )
{
	return (double)t / (double)FIX_TIME_ONE;
}

// Internal: the reserved minimum held at FIX_TIME_MIN. fixAddSaturate and fixSubSaturate
// pass a reserved operand through unchanged when the other is zero (INT64_MIN + 0); a
// saturating time result never lands there, as a saturating wide result never lands on the
// 128-bit minimum.
FIX_ALWAYS_INLINE fixTime fixTimeLiftReserved( fixTime t )
{
	return t == INT64_MIN ? FIX_TIME_MIN : t;
}

/// t + u. Wraps by default (defined, through unsigned); saturates to the time range under
/// FIX_SATURATE. The bounds are fixed_t's own, so the saturating form is fixAddSaturate.
FIX_ALWAYS_INLINE fixTime fixTimeAdd( fixTime t, fixTime u )
{
#if defined( FIX_SATURATE )
	return fixTimeLiftReserved( fixAddSaturate( t, u ) );
#else
	return (fixTime)( (uint64_t)t + (uint64_t)u );
#endif
}

/// t - u. Wraps by default (defined, through unsigned); saturates under FIX_SATURATE.
FIX_ALWAYS_INLINE fixTime fixTimeSub( fixTime t, fixTime u )
{
#if defined( FIX_SATURATE )
	return fixTimeLiftReserved( fixSubSaturate( t, u ) );
#else
	return (fixTime)( (uint64_t)t - (uint64_t)u );
#endif
}

/// -t. The reserved minimum wraps to itself by default and reads as FIX_TIME_MAX under FIX_SATURATE.
FIX_ALWAYS_INLINE fixTime fixTimeNeg( fixTime t )
{
#if defined( FIX_SATURATE )
	if ( t == INT64_MIN )
	{
		return FIX_TIME_MAX;
	}
#endif
	return (fixTime)( (uint64_t)0 - (uint64_t)t );
}

// Internal: a 128-bit quotient or product held at [FIX_TIME_MIN, FIX_TIME_MAX].
FIX_ALWAYS_INLINE fixTime fixTimeFromInt128Saturate( fixInt128 a )
{
	if ( fixInt128Gt( a, fixInt128FromI64( FIX_TIME_MAX ) ) )
	{
		return FIX_TIME_MAX;
	}
	if ( fixInt128Lt( a, fixInt128FromI64( FIX_TIME_MIN ) ) )
	{
		return FIX_TIME_MIN;
	}
	return (fixTime)fixInt128ToI64( a );
}

/// Scale a time by a Q48.16 factor, rounding to nearest. 128-bit interior. By default the
/// result is not checked for overflow and wraps, as fixMul does; under FIX_SATURATE it
/// saturates to FIX_TIME_MIN/FIX_TIME_MAX with the sign of the exact product.
FIX_ALWAYS_INLINE fixTime fixTimeMulFixed( fixTime t, fixed_t s )
{
	fixInt128 product = fixInt128MulI64( t, s );
	fixInt128 rounded = fixInt128Shr( fixInt128Add( product, fixInt128FromI64( FIX_HALF ) ), FIX_FRACTION_BITS );
#if defined( FIX_SATURATE )
	return fixTimeFromInt128Saturate( rounded );
#else
	return (fixTime)fixInt128ToI64( rounded );
#endif
}

/// Divide a time by a Q48.16 factor with truncation toward zero and saturation, in every
/// build: fixDiv with a Q32.32 numerator. Division by zero saturates with the sign of the
/// numerator; 0 / 0 == 0.
FIX_ALWAYS_INLINE fixTime fixTimeDivFixed( fixTime t, fixed_t s )
{
	if ( s == 0 )
	{
		return t > 0 ? FIX_TIME_MAX : ( t < 0 ? FIX_TIME_MIN : 0 );
	}
	fixInt128 q = fixInt128Div( fixInt128ShiftLeft( fixInt128FromI64( t ), FIX_FRACTION_BITS ), fixInt128FromI64( s ) );
	return fixTimeFromInt128Saturate( q );
}

/// The ratio of two times as a Q48.16 factor, truncating toward zero and saturating to
/// FIX_MIN/FIX_MAX, in every build. Division by zero saturates with the sign of the
/// numerator; 0 / 0 == 0. The fraction points cancel, so this is the integer quotient of
/// t << 16 by u, and the bounds are fixed_t's.
FIX_ALWAYS_INLINE fixed_t fixTimeDiv( fixTime t, fixTime u )
{
	if ( u == 0 )
	{
		return t > 0 ? FIX_MAX : ( t < 0 ? FIX_MIN : 0 );
	}
	fixInt128 q = fixInt128Div( fixInt128ShiftLeft( fixInt128FromI64( t ), FIX_FRACTION_BITS ), fixInt128FromI64( u ) );
	return (fixed_t)fixTimeFromInt128Saturate( q );
}
