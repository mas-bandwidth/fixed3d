// SPDX-FileCopyrightText: 2026 Fixed3D core-math harness
// SPDX-License-Identifier: MIT
//
// Row atan2: fixAtan2 (extern/fixed) and b3Atan2 (src/math_functions.c) against
// CORE-MATH's cr_atan2, at the Q48.16 bound in docs/determinism-math.md: 2 ulps for
// every pair but (0, 0), which returns 0. The raw ratio is scale-free, so the raws go
// to the reference as doubles.

#include "core_math_harness.h"
#include "test_macros.h"

#define CM_ATAN2_BOUND 2u
#define CM_ATAN2_HARD_SUBSET 256
#define CM_ATAN2_CIRCLE_POINTS 4096

typedef int64_t cmFn2( int64_t y, int64_t x );

static int64_t CoreMathFixAtan2( int64_t y, int64_t x )
{
	return fixAtan2( y, x );
}

static int64_t CoreMathB3Atan2( int64_t y, int64_t x )
{
	return b3Atan2( y, x );
}

static int64_t cmRefAtan2( int64_t y, int64_t x )
{
	return cmRefFromDouble( cr_atan2( (double)y, (double)x ) );
}

typedef struct Atan2Row
{
	cmRow row;
	int64_t worstY;
	int64_t worstX;
	int64_t worstGot;
	int64_t worstWant;
} Atan2Row;

// Record one ordered pair. cmRowAdd counts the input and tracks the row's maxulp; the
// pair itself is kept alongside so a red can name the exact y, x, got and want.
static void cmAtan2Add( Atan2Row* measure, cmFn2* fn, int64_t y, int64_t x )
{
	int64_t got = fn( y, x );
	int64_t want = cmRefAtan2( y, x );
	uint64_t ulp = cmUlpDistance( got, want );
	if ( ulp > measure->row.maxUlp || measure->row.inputs == 0 )
	{
		measure->worstY = y;
		measure->worstX = x;
		measure->worstGot = got;
		measure->worstWant = want;
	}
	cmRowAdd( &measure->row, y, got, want );
}

static void cmAtan2RandomPairs( Atan2Row* measure, cmFn2* fn, uint64_t* state, int64_t count, int64_t lo, int64_t hi )
{
	for ( int64_t i = 0; i < count; ++i )
	{
		int64_t y = cmRandomRaw( state, lo, hi );
		int64_t x = cmRandomRaw( state, lo, hi );
		if ( y != 0 || x != 0 )
		{
			cmAtan2Add( measure, fn, y, x );
		}
	}
}

// Run one row: every ordered pair of a 256-entry stride through the hard cases (skipping
// (0, 0)), random pairs in raw +-2^20 and +-2^46, and points on the unit circle. At CI
// size the random counts are 2^20 and 2^18; exhaustive runs the harness's 2^28 random
// pairs in total, split 4:1 between the two ranges.
static int CoreMathRunAtan2Row( const char* name, cmFn2* fn, bool exhaustive )
{
	Atan2Row measure = { 0 };
	cmRowBegin( &measure.row, name );

	cmInputs hard = { 0 };
	if ( cmHardCases( &hard, NULL, 0 ) == false )
	{
		printf( "core-math row=%s hard cases: allocation failed\n", name );
		return 1;
	}

	int stride = hard.count / CM_ATAN2_HARD_SUBSET;
	if ( stride < 1 )
	{
		stride = 1;
	}
	int64_t subset[CM_ATAN2_HARD_SUBSET];
	for ( int i = 0; i < CM_ATAN2_HARD_SUBSET; ++i )
	{
		int index = i * stride;
		if ( index >= hard.count )
		{
			index = hard.count - 1;
		}
		subset[i] = hard.raws[index];
	}
	for ( int i = 0; i < CM_ATAN2_HARD_SUBSET; ++i )
	{
		for ( int j = 0; j < CM_ATAN2_HARD_SUBSET; ++j )
		{
			if ( subset[i] != 0 || subset[j] != 0 )
			{
				cmAtan2Add( &measure, fn, subset[i], subset[j] );
			}
		}
	}
	cmInputsFree( &hard );

	uint64_t state = CM_SEED;
	int64_t narrowCount = exhaustive ? 0 : ( (int64_t)1 << 20 );
	int64_t wideCount = exhaustive ? 0 : ( (int64_t)1 << 18 );
	if ( exhaustive )
	{
		int64_t total = cmRandomCount();
		wideCount = total / 5;
		narrowCount = total - wideCount;
	}
	cmAtan2RandomPairs( &measure, fn, &state, narrowCount, -( (int64_t)1 << 20 ), (int64_t)1 << 20 );
	cmAtan2RandomPairs( &measure, fn, &state, wideCount, -( (int64_t)1 << 46 ), (int64_t)1 << 46 );

	for ( int i = 0; i < CM_ATAN2_CIRCLE_POINTS; ++i )
	{
		double t = 2.0 * CM_PI_2 * ( (double)i / (double)CM_ATAN2_CIRCLE_POINTS );
		int64_t x = cmRoundNearestEven( cr_cos( t ) * 65536.0 );
		int64_t y = cmRoundNearestEven( cr_sin( t ) * 65536.0 );
		if ( x != 0 || y != 0 )
		{
			cmAtan2Add( &measure, fn, y, x );
		}
	}

	int failed = cmRowEnd( &measure.row, CM_ATAN2_BOUND );
	if ( failed )
	{
		printf( "core-math row=%s worst pair y=%" PRId64 " x=%" PRId64 " got=%" PRId64 " want=%" PRId64 "\n",
				name, measure.worstY, measure.worstX, measure.worstGot, measure.worstWant );
	}
	return failed;
}

int CoreMathAtan2Test( void )
{
	// The reference itself, on values worked by hand: atan2(1, 1) = pi/4, nearest raw
	// 51472; atan2(0, 1) = 0; atan2(0, -1) = pi, nearest raw 205887.
	ENSURE( cmRefAtan2( 1, 1 ) == 51472 );
	ENSURE( cmRefAtan2( 0, 1 ) == 0 );
	ENSURE( cmRefAtan2( 0, -1 ) == 205887 );

	// The origin returns 0 exactly, per the policy.
	ENSURE( fixAtan2( 0, 0 ) == 0 );
	ENSURE( b3Atan2( 0, 0 ) == 0 );

	bool exhaustive = CoreMathExhaustive();
	ENSURE( CoreMathRunAtan2Row( "atan2-fix", CoreMathFixAtan2, exhaustive ) == 0 );
	ENSURE( CoreMathRunAtan2Row( "atan2-b3", CoreMathB3Atan2, exhaustive ) == 0 );
	return 0;
}
