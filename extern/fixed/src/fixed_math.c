// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
#include "fixed/fixed_math.h"

static inline int64_t b3Q32Mul( int64_t a, int64_t b )
{
	return (int64_t)( ( (b3Int128)a * b ) >> 32 );
}

static inline int64_t b3Q32Div( int64_t a, int64_t b )
{
	return (int64_t)b3Int128Div( b3Int128ShiftLeft( a, 32 ), b );
}


static inline int64_t b3Q32Sqrt( int64_t a )
{
	if ( a <= 0 )
	{
		return 0;
	}
	uint64_t hi = (uint64_t)a >> 32;
	uint64_t lo = (uint64_t)a << 32;
	return (int64_t)b3ISqrt128High( hi, lo );
}

// Round a Q32.32 value to Q48.16
static inline b3Fixed b3Q32ToFix( int64_t a )
{
	return ( a + ( (int64_t)1 << 15 ) ) >> 16;
}

#define B3_Q32_PI 13493037705LL	   // round(pi * 2^32)
#define B3_Q32_HALF_PI 6746518852LL // round(pi/2 * 2^32)

b3Fixed b3Atan2( b3Fixed y, b3Fixed x )
{
	// Added check for (0,0) to match atan2f and avoid a zero divide
	if ( x == 0 && y == 0 )
	{
		return 0;
	}

	b3Fixed ax = b3FixAbs( x );
	b3Fixed ay = b3FixAbs( y );
	b3Fixed mx = b3FixMax( ay, ax );
	b3Fixed mn = b3FixMin( ay, ax );

	// a = mn / mx in [0, 1], evaluated in Q32.32. mn >= 0 so the raw shift
	// would be fine, but the helper is the convention for signed shifts.
	int64_t a = (int64_t)b3Int128Div( b3Int128ShiftLeft( (b3Int128)mn, 32 ), mx );

	// Minimax polynomial approximation to atan(a) on [0,1]
	int64_t s = b3Q32Mul( a, a );
	int64_t c = b3Q32Mul( s, a );
	int64_t q = b3Q32Mul( s, s );
	int64_t r = b3Q32Mul( 106688212LL, q ) + 802360794LL;	  // 0.024840285, 0.18681418
	int64_t t = b3Q32Mul( -404147609LL, q ) - 1426490580LL; // -0.094097948, -0.33213072
	r = b3Q32Mul( r, s ) + t;
	r = b3Q32Mul( r, c ) + a;

	// Map to full circle
	if ( ay > ax )
	{
		r = B3_Q32_HALF_PI - r;
	}

	if ( x < 0 )
	{
		r = B3_Q32_PI - r;
	}

	if ( y < 0 )
	{
		r = -r;
	}

	return b3Q32ToFix( r );
}

// Approximate cosine and sine for determinism, evaluated in pure integer math.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
b3CosSin b3ComputeCosSin( b3Fixed radians )
{
	const int64_t pi = B3_Q32_PI;
	const int64_t halfPi = B3_Q32_HALF_PI;
	const int64_t pi2 = 42389628127LL;		 // pi^2 in Q32.32
	const int64_t fivePi2 = 211948140636LL; // 5*pi^2 in Q32.32

	// The unwound angle is in [-pi, pi] to within an ulp; promote to Q32.32
	int64_t x = b3FixShiftLeft( b3UnwindAngle( radians ), 16 );
	x = x < -pi ? -pi : ( x > pi ? pi : x );

	// cosine needs angle in [-pi/2, pi/2]
	int64_t c;
	if ( x < -halfPi )
	{
		int64_t y = x + pi;
		int64_t y2 = b3Q32Mul( y, y );
		c = -b3Q32Div( pi2 - 4 * y2, pi2 + y2 );
	}
	else if ( x > halfPi )
	{
		int64_t y = x - pi;
		int64_t y2 = b3Q32Mul( y, y );
		c = -b3Q32Div( pi2 - 4 * y2, pi2 + y2 );
	}
	else
	{
		int64_t y2 = b3Q32Mul( x, x );
		c = b3Q32Div( pi2 - 4 * y2, pi2 + y2 );
	}

	// sine needs angle in [0, pi]
	int64_t s;
	if ( x < 0 )
	{
		int64_t y = x + pi;
		int64_t u = b3Q32Mul( y, pi - y );
		s = -b3Q32Div( 16 * u, fivePi2 - 4 * u );
	}
	else
	{
		int64_t u = b3Q32Mul( x, pi - x );
		s = b3Q32Div( 16 * u, fivePi2 - 4 * u );
	}

	// Normalize so the pair lies on the unit circle
	int64_t mag = b3Q32Sqrt( b3Q32Mul( s, s ) + b3Q32Mul( c, c ) );
	if ( mag > 0 )
	{
		c = b3Q32Div( c, mag );
		s = b3Q32Div( s, mag );
	}

	b3CosSin cs = { b3Q32ToFix( c ), b3Q32ToFix( s ) };
	return cs;
}
