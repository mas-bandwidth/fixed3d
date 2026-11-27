// SPDX-FileCopyrightText: 2026 Fixed3D core-math harness
// SPDX-License-Identifier: MIT
//
// The CORE-MATH harness: what every row of CoreMathTest is measured with. The policy it
// implements is docs/determinism-math.md; read that first.
//
// CORE-MATH (extern/core-math, pinned by tools/revendor-core-math.sh) is a TEST-ONLY
// reference. It is linked into ./bin/test, never into libbox3d, which stays integer-only.
//
// A row compares one fixed-point function against its reference and reports, in Q48.16
// ulps (raw units), the largest distance seen:
//
//   core-math row=<name> inputs=<n> maxulp=<m> bound=<b> worst=<raw>
//
// one line per piece of a piecewise bound, named <name>[<lo>,<hi>). A row fails (returns 1)
// when maxulp > bound. Inputs are the hard cases (cmHardCases) plus dense inputs: 2^20 per
// row in CI, and with B3_CORE_MATH_EXHAUSTIVE=1 every raw in [-2^24, 2^24] plus 2^28 random
// raws (univariate) or 2^28 random pairs (bivariate). Randomness is a fixed-seed xorshift64,
// never time, so every run sees the same inputs on every platform.
//
// Everything here is static inline so a row file under test/core_math/ includes it and
// touches nothing else. No __int128: the 128-bit steps are done in two 64-bit halves so the
// harness builds under the strict C17 test target on every compiler, MSVC included.

#pragma once

#include "box3d/math_functions.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// The CORE-MATH references, correctly rounded in binary64 (extern/core-math, built by
// test/CMakeLists.txt as the box3d_core_math object library).
double cr_atan2( double y, double x );
double cr_sin( double x );
double cr_cos( double x );
double cr_log2( double x );
double cr_exp2( double x );

// True when B3_CORE_MATH_EXHAUSTIVE=1 is set. Defined in test_core_math.c.
bool CoreMathExhaustive( void );

// ---------------------------------------------------------------------------------------------
// Distances and randomness
// ---------------------------------------------------------------------------------------------

// Q48.16 ulp distance: the absolute raw difference. Two int64 raws can be 2^64 - 1 apart, which
// overflows int64 but not uint64, so the subtraction is done unsigned, larger minus smaller.
static inline uint64_t cmUlpDistance( int64_t a, int64_t b )
{
	return a >= b ? (uint64_t)a - (uint64_t)b : (uint64_t)b - (uint64_t)a;
}

#define CM_SEED UINT64_C( 0x9E3779B97F4A7C15 )

// xorshift64 (13, 7, 17). Seed with CM_SEED; never with time.
static inline uint64_t cmNext( uint64_t* state )
{
	uint64_t x = *state;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	*state = x;
	return x;
}

// A uniform raw in [lo, hi] (inclusive), lo <= hi.
static inline int64_t cmRandomRaw( uint64_t* state, int64_t lo, int64_t hi )
{
	uint64_t span = (uint64_t)hi - (uint64_t)lo;
	uint64_t x = cmNext( state );
	uint64_t offset = span == UINT64_MAX ? x : x % ( span + 1 );
	return (int64_t)( (uint64_t)lo + offset );
}

// ---------------------------------------------------------------------------------------------
// Reference converters (docs/determinism-math.md, "The reference")
// ---------------------------------------------------------------------------------------------

// Round a double to the nearest integer, ties to even, independent of the floating-point
// rounding mode. Saturates at the int64 range; NaN maps to 0 (no row feeds a NaN reference).
static inline int64_t cmRoundNearestEven( double s )
{
	if ( s != s )
	{
		return 0;
	}
	if ( s >= 9223372036854775808.0 )
	{
		return INT64_MAX;
	}
	if ( s < -9223372036854775808.0 )
	{
		return INT64_MIN;
	}
	double f = floor( s );
	double frac = s - f; // exact: s and f are within 1 of each other
	int64_t r = (int64_t)f;
	if ( frac > 0.5 )
	{
		r += 1;
	}
	else if ( frac == 0.5 && ( r & 1 ) != 0 )
	{
		r += 1;
	}
	return r;
}

// The transcendental reference: CORE-MATH's correctly rounded double result, times 65536
// (exact: a power of two), rounded to nearest, ties to even.
static inline int64_t cmRefFromDouble( double d )
{
	return cmRoundNearestEven( d * 65536.0 );
}

// A raw as the double the reference is evaluated at. Exact for |raw| <= 2^53; above that the
// double is the nearest representable value and the row must say how it treats the difference.
static inline double cmRawToDouble( int64_t raw )
{
	return (double)raw / 65536.0;
}

typedef struct cmU128
{
	uint64_t hi, lo;
} cmU128;

static inline cmU128 cmMul64( uint64_t a, uint64_t b )
{
	uint64_t a0 = a & 0xffffffffu, a1 = a >> 32;
	uint64_t b0 = b & 0xffffffffu, b1 = b >> 32;
	uint64_t p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
	uint64_t mid = ( p00 >> 32 ) + ( p01 & 0xffffffffu ) + ( p10 & 0xffffffffu );
	cmU128 r;
	r.lo = ( mid << 32 ) | ( p00 & 0xffffffffu );
	r.hi = p11 + ( p01 >> 32 ) + ( p10 >> 32 ) + ( mid >> 32 );
	return r;
}

// -1, 0, 1 as a <, ==, > b
static inline int cmCmp128( cmU128 a, cmU128 b )
{
	if ( a.hi != b.hi )
	{
		return a.hi < b.hi ? -1 : 1;
	}
	if ( a.lo != b.lo )
	{
		return a.lo < b.lo ? -1 : 1;
	}
	return 0;
}

// The fixSqrt reference: exact integer arithmetic, rounded to nearest. sqrt(raw / 2^16) in
// Q48.16 is sqrt(N) with N = raw * 2^16 < 2^79, which no double holds exactly. r is the exact
// floor sqrt (the double sqrt is only a seed; the two loops make r*r <= N < (r+1)^2 hold by
// construction). Since (r + 1/2)^2 = r^2 + r + 1/4 and N is an integer, N is nearer r + 1
// exactly when N - r^2 > r; no tie is possible. raw <= 0 gives 0, as fixSqrt does.
static inline int64_t cmRefSqrt( int64_t raw )
{
	if ( raw <= 0 )
	{
		return 0;
	}

	cmU128 n;
	n.hi = (uint64_t)raw >> 48;
	n.lo = (uint64_t)raw << 16;

	double seed = sqrt( (double)n.hi * 18446744073709551616.0 + (double)n.lo );
	uint64_t r = (uint64_t)seed;
	while ( r > 0 && cmCmp128( cmMul64( r, r ), n ) > 0 )
	{
		--r;
	}
	while ( cmCmp128( cmMul64( r + 1, r + 1 ), n ) <= 0 )
	{
		++r;
	}

	cmU128 sq = cmMul64( r, r );
	cmU128 rem;
	rem.lo = n.lo - sq.lo;
	rem.hi = n.hi - sq.hi - ( n.lo < sq.lo ? 1u : 0u );
	if ( rem.hi != 0 || rem.lo > r )
	{
		r += 1;
	}
	return (int64_t)r;
}

// ---------------------------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------------------------

typedef struct cmInputs
{
	int64_t* raws;
	int count;
	int capacity;
} cmInputs;

static inline void cmInputsPush( cmInputs* in, int64_t raw )
{
	if ( in->capacity < 0 )
	{
		return;
	}
	if ( in->count == in->capacity )
	{
		int capacity = in->capacity == 0 ? 1024 : 2 * in->capacity;
		int64_t* raws = (int64_t*)realloc( in->raws, (size_t)capacity * sizeof( int64_t ) );
		if ( raws == NULL )
		{
			free( in->raws );
			in->raws = NULL;
			in->count = 0;
			in->capacity = -1;
			return;
		}
		in->raws = raws;
		in->capacity = capacity;
	}
	in->raws[in->count++] = raw;
}

static inline void cmInputsFree( cmInputs* in )
{
	free( in->raws );
	in->raws = NULL;
	in->count = 0;
	in->capacity = 0;
}

#define CM_PI_2 1.57079632679489661923
#define CM_HARD_K 64
#define CM_HARD_SPREAD 4096

// The hard cases every row runs: raw 0, +-1, +-2; +-4096 raw around round(k*pi/2 * 2^16) for
// |k| <= 64; +-2^20..2^62; INT64_MAX and INT64_MIN + 1; and each edge of the row's bound
// pieces with its two neighbours (edges may be NULL when edgeCount is 0). Returns false when
// an allocation failed (the input list is then empty and stays so).
static inline bool cmHardCases( cmInputs* in, const int64_t* edges, int edgeCount )
{
	static const int64_t small[] = { 0, 1, -1, 2, -2 };
	for ( int i = 0; i < 5; ++i )
	{
		cmInputsPush( in, small[i] );
	}

	for ( int k = -CM_HARD_K; k <= CM_HARD_K; ++k )
	{
		int64_t centre = cmRoundNearestEven( (double)k * ( CM_PI_2 * 65536.0 ) );
		for ( int d = -CM_HARD_SPREAD; d <= CM_HARD_SPREAD; ++d )
		{
			cmInputsPush( in, centre + d );
		}
	}

	for ( int p = 20; p <= 62; ++p )
	{
		int64_t v = (int64_t)1 << p;
		cmInputsPush( in, v );
		cmInputsPush( in, -v );
	}
	cmInputsPush( in, INT64_MAX );
	cmInputsPush( in, INT64_MIN + 1 );

	for ( int i = 0; i < edgeCount; ++i )
	{
		int64_t e = edges[i];
		if ( e > INT64_MIN )
		{
			cmInputsPush( in, e - 1 );
		}
		cmInputsPush( in, e );
		if ( e < INT64_MAX )
		{
			cmInputsPush( in, e + 1 );
		}
	}

	return in->raws != NULL;
}

// Dense input counts: 2^20 per row in CI; 2^28 random raws or pairs when exhaustive.
static inline int64_t cmRandomCount( void )
{
	return CoreMathExhaustive() ? ( (int64_t)1 << 28 ) : ( (int64_t)1 << 20 );
}

// The exhaustive univariate range is every raw in [-2^24, 2^24].
#define CM_EXHAUSTIVE_RANGE ( (int64_t)1 << 24 )

// ---------------------------------------------------------------------------------------------
// The row runner
// ---------------------------------------------------------------------------------------------

typedef struct cmRow
{
	const char* name;
	int64_t inputs;
	uint64_t maxUlp;
	int64_t worst;
} cmRow;

typedef int64_t cmFn1( int64_t raw );

static inline void cmRowBegin( cmRow* row, const char* name )
{
	row->name = name;
	row->inputs = 0;
	row->maxUlp = 0;
	row->worst = 0;
}

// Record one input: got is the fixed-point result, want the reference. worst is the first
// input that reached the row's maxulp (for a bivariate row, the row chooses which raw to name).
static inline void cmRowAdd( cmRow* row, int64_t input, int64_t got, int64_t want )
{
	uint64_t ulp = cmUlpDistance( got, want );
	if ( ulp > row->maxUlp || row->inputs == 0 )
	{
		row->maxUlp = ulp;
		row->worst = input;
	}
	row->inputs += 1;
}

// Print the row line; 1 when maxulp > bound, else 0.
static inline int cmRowEnd( const cmRow* row, uint64_t bound )
{
	printf( "core-math row=%s inputs=%" PRId64 " maxulp=%" PRIu64 " bound=%" PRIu64 " worst=%" PRId64 "\n", row->name,
			row->inputs, row->maxUlp, bound, row->worst );
	return row->maxUlp > bound ? 1 : 0;
}

static inline void cmRowRun1Array( cmRow* row, cmFn1* fn, cmFn1* ref, const int64_t* raws, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		cmRowAdd( row, raws[i], fn( raws[i] ), ref( raws[i] ) );
	}
}

// Every raw in [lo, hi], inclusive.
static inline void cmRowRun1Range( cmRow* row, cmFn1* fn, cmFn1* ref, int64_t lo, int64_t hi )
{
	for ( int64_t raw = lo;; ++raw )
	{
		cmRowAdd( row, raw, fn( raw ), ref( raw ) );
		if ( raw == hi )
		{
			break;
		}
	}
}

// count raws drawn uniformly from [lo, hi] by the seeded generator in *state.
static inline void cmRowRun1Random( cmRow* row, cmFn1* fn, cmFn1* ref, uint64_t* state, int64_t count, int64_t lo,
									int64_t hi )
{
	for ( int64_t i = 0; i < count; ++i )
	{
		int64_t raw = cmRandomRaw( state, lo, hi );
		cmRowAdd( row, raw, fn( raw ), ref( raw ) );
	}
}
