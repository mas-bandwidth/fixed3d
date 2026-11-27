// SPDX-FileCopyrightText: 2026 Fixed3D core-math harness
// SPDX-License-Identifier: MIT
//
// Row log2: b3FixLog2 (fixLog2 in extern/fixed) against CORE-MATH's cr_log2, at the Q48.16
// bound in docs/determinism-math.md: 0 ulps for every raw > 0. The domain ends at raw <= 0,
// where fixLog2 returns INT64_MIN, log2 having no value there.

#include "core_math_harness.h"
#include "test_macros.h"

#define CM_LOG2_BOUND 0u

static int64_t CoreMathFixLog2( int64_t raw )
{
	return b3FixLog2( raw );
}

static int64_t cmRefLog2( int64_t raw )
{
	return cmRefFromDouble( cr_log2( cmRawToDouble( raw ) ) );
}

typedef struct CoreMathLog2Result
{
	int failed;
	uint64_t maxUlp;
	int64_t worst;
	int nonPositiveWrong;
} CoreMathLog2Result;

// Run the log2 row on fn: hard cases (the positive ones), every raw in [1, 2^20] (CI) or
// [1, 2^24] (exhaustive), powers of two 2^0..2^62 raw, INT64_MAX, and 2^20 (CI) or 2^28
// (exhaustive) random positive raws. Every raw <= 0 must return INT64_MIN exactly.
static CoreMathLog2Result CoreMathRunLog2Row( const char* name, cmFn1* fn, bool exhaustive )
{
	CoreMathLog2Result result = { 0 };
	cmRow row;
	cmRowBegin( &row, name );

	cmInputs hard = { 0 };
	if ( cmHardCases( &hard, NULL, 0 ) == false )
	{
		printf( "core-math row=%s hard cases: allocation failed\n", name );
		result.failed = 1;
		return result;
	}
	for ( int i = 0; i < hard.count; ++i )
	{
		int64_t raw = hard.raws[i];
		if ( raw > 0 )
		{
			cmRowAdd( &row, raw, fn( raw ), cmRefLog2( raw ) );
		}
		else if ( fn( raw ) != INT64_MIN )
		{
			result.nonPositiveWrong += 1;
		}
	}
	cmInputsFree( &hard );

	if ( exhaustive )
	{
		for ( int64_t raw = -CM_EXHAUSTIVE_RANGE; raw <= 0; ++raw )
		{
			if ( fn( raw ) != INT64_MIN )
			{
				result.nonPositiveWrong += 1;
			}
		}
		cmRowRun1Range( &row, fn, cmRefLog2, 1, CM_EXHAUSTIVE_RANGE );
	}
	else
	{
		cmRowRun1Range( &row, fn, cmRefLog2, 1, (int64_t)1 << 20 );
	}

	for ( int p = 0; p <= 62; ++p )
	{
		int64_t raw = (int64_t)1 << p;
		cmRowAdd( &row, raw, fn( raw ), cmRefLog2( raw ) );
	}
	cmRowAdd( &row, INT64_MAX, fn( INT64_MAX ), cmRefLog2( INT64_MAX ) );

	uint64_t state = CM_SEED;
	int64_t randomCount = cmRandomCount();
	cmRowRun1Random( &row, fn, cmRefLog2, &state, randomCount, 1, INT64_MAX );

	result.failed = cmRowEnd( &row, CM_LOG2_BOUND );
	result.maxUlp = row.maxUlp;
	result.worst = row.worst;
	if ( result.nonPositiveWrong != 0 )
	{
		printf( "core-math row=%s raw<=0 not INT64_MIN on %d inputs\n", name, result.nonPositiveWrong );
		result.failed = 1;
	}
	return result;
}

int CoreMathLog2Test( void )
{
	// The reference itself, on values worked by hand: log2(1.0) = 0; log2(2.0) = 1;
	// log2(8.0) = 3; log2(2^-16) = -16. raw 2^16 is 1.0.
	ENSURE( cmRefLog2( 65536 ) == 0 );
	ENSURE( cmRefLog2( 2 * 65536 ) == 65536 );
	ENSURE( cmRefLog2( 8 * 65536 ) == 3 * 65536 );
	ENSURE( cmRefLog2( 1 ) == -16 * 65536 );
	ENSURE( b3FixLog2( 65536 ) == 0 );
	ENSURE( b3FixLog2( 0 ) == INT64_MIN );
	ENSURE( b3FixLog2( -1 ) == INT64_MIN );
	ENSURE( b3FixLog2( INT64_MIN + 1 ) == INT64_MIN );

	CoreMathLog2Result result = CoreMathRunLog2Row( "log2", CoreMathFixLog2, CoreMathExhaustive() );
	ENSURE( result.failed == 0 );
	return 0;
}
