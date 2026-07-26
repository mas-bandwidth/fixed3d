// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
// Fixed-point time: Q32.32 seconds in an int64. 233 picosecond resolution,
// ±68 year range. Time gets more fraction bits than fixed_t because time
// accumulates — thousands of small dt adds per minute — and every add is
// exact in integer arithmetic, so the only error is the one-time rounding of
// dt itself. At Q32.32 that is ~2.3e-10 s per tick.
#pragma once

#include "fixed/fixed.h"

/// Fixed-point time in seconds: Q32.32 in a 64-bit integer.
typedef int64_t fixTime;

#define FIX_TIME_FRACTION_BITS 32
#define FIX_TIME_ONE ( (fixTime)1 << FIX_TIME_FRACTION_BITS )
#define FIX_TIME_HALF ( (fixTime)1 << ( FIX_TIME_FRACTION_BITS - 1 ) )

/// Convert a numeric literal to fixed-point time at compile time, e.g. FIX_TIME( 1.0 / 60.0 ).
#define FIX_TIME( x ) ( (fixTime)( ( x ) * (double)FIX_TIME_ONE + ( ( x ) >= 0 ? 0.5 : -0.5 ) ) )

/// Convert Q48.16 seconds to Q32.32 time. Exact (left shift).
FIX_ALWAYS_INLINE fixTime fixTimeFromFixed( fixed_t seconds )
{
	return (fixTime)( (uint64_t)seconds << ( FIX_TIME_FRACTION_BITS - FIX_FRACTION_BITS ) );
}

/// Convert Q32.32 time to Q48.16 seconds, rounding to nearest.
FIX_ALWAYS_INLINE fixed_t fixTimeToFixed( fixTime t )
{
	return ( t + ( (fixTime)1 << ( FIX_TIME_FRACTION_BITS - FIX_FRACTION_BITS - 1 ) ) ) >>
		   ( FIX_TIME_FRACTION_BITS - FIX_FRACTION_BITS );
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

/// Scale a time by a Q48.16 factor, rounding to nearest. 128-bit interior.
FIX_ALWAYS_INLINE fixTime fixTimeMulFixed( fixTime t, fixed_t s )
{
#if FIX_HAS_INT128
	fixInt128 product = (fixInt128)t * s;
	return (fixTime)( ( product + FIX_HALF ) >> FIX_FRACTION_BITS );
#else
#error "fixTimeMulFixed requires 128-bit integer support"
#endif
}
