// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
// Fixed-point time: Q32.32 seconds in an int64. 233 picosecond resolution,
// ±68 year range. Time gets more fraction bits than b3Fixed because time
// accumulates — thousands of small dt adds per minute — and every add is
// exact in integer arithmetic, so the only error is the one-time rounding of
// dt itself. At Q32.32 that is ~2.3e-10 s per tick.
#pragma once

#include "fixed/fixed.h"

/// Fixed-point time in seconds: Q32.32 in a 64-bit integer.
typedef int64_t b3Time;

#define B3_TIME_FRACTION_BITS 32
#define B3_TIME_ONE ( (b3Time)1 << B3_TIME_FRACTION_BITS )
#define B3_TIME_HALF ( (b3Time)1 << ( B3_TIME_FRACTION_BITS - 1 ) )

/// Convert a numeric literal to fixed-point time at compile time, e.g. B3_TIME( 1.0 / 60.0 ).
#define B3_TIME( x ) ( (b3Time)( ( x ) * (double)B3_TIME_ONE + ( ( x ) >= 0 ? 0.5 : -0.5 ) ) )

/// Convert Q48.16 seconds to Q32.32 time. Exact (left shift).
B3_FIXED_INLINE b3Time b3TimeFromFixed( b3Fixed seconds )
{
	return (b3Time)( (uint64_t)seconds << ( B3_TIME_FRACTION_BITS - B3_FIXED_FRACTION_BITS ) );
}

/// Convert Q32.32 time to Q48.16 seconds, rounding to nearest.
B3_FIXED_INLINE b3Fixed b3TimeToFixed( b3Time t )
{
	return ( t + ( (b3Time)1 << ( B3_TIME_FRACTION_BITS - B3_FIXED_FRACTION_BITS - 1 ) ) ) >>
		   ( B3_TIME_FRACTION_BITS - B3_FIXED_FRACTION_BITS );
}

/// Convert seconds (double) to Q32.32 time, rounding to nearest. Init/tooling
/// only — never in simulation, where cross-platform float behavior is the
/// enemy this library exists to remove.
B3_FIXED_INLINE b3Time b3TimeFromSeconds( double seconds )
{
	double scaled = seconds * (double)B3_TIME_ONE;
	return (b3Time)( scaled >= 0 ? scaled + 0.5 : scaled - 0.5 );
}

/// Convert Q32.32 time to seconds (double). Debug/display only.
B3_FIXED_INLINE double b3TimeToSeconds( b3Time t )
{
	return (double)t / (double)B3_TIME_ONE;
}

/// Scale a time by a Q48.16 factor, rounding to nearest. 128-bit interior.
B3_FIXED_INLINE b3Time b3TimeMulFixed( b3Time t, b3Fixed s )
{
#if B3_HAS_INT128
	b3Int128 product = (b3Int128)t * s;
	return (b3Time)( ( product + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS );
#else
#error "b3TimeMulFixed requires 128-bit integer support"
#endif
}
