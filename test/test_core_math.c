// SPDX-FileCopyrightText: 2026 Fixed3D core-math harness
// SPDX-License-Identifier: MIT
//
// CoreMathTest: the fixed-point math functions against their bit-exact references, under the
// policy in docs/determinism-math.md. This file holds the sqrt row, the reversed witness that
// proves the row runner can fail, and the switch for the four transcendental rows, each of
// which arrives as one file under test/core_math/ (see test/core_math/README.md).

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "core_math_harness.h"
#include "test_macros.h"

#include <string.h>

bool CoreMathExhaustive( void )
{
	const char* value = getenv( "B3_CORE_MATH_EXHAUSTIVE" );
	return value != NULL && strcmp( value, "1" ) == 0;
}

#if B3_CORE_MATH_ROW_ATAN2
int CoreMathAtan2Test( void );
#endif
#if B3_CORE_MATH_ROW_SINCOS
int CoreMathSinCosTest( void );
#endif
#if B3_CORE_MATH_ROW_LOG2
int CoreMathLog2Test( void );
#endif
#if B3_CORE_MATH_ROW_EXP2
int CoreMathExp2Test( void );
#endif

// ---------------------------------------------------------------------------------------------
// Row sqrt: b3FixSqrt (fixSqrt in extern/fixed) against exact integer round-to-nearest.
// Bound 1 ulp for every raw > 0: fixSqrt returns the floor, so it is 0 or 1 below the nearest.
// raw <= 0 returns 0, checked exactly.
// ---------------------------------------------------------------------------------------------

#define CM_SQRT_BOUND 1u

static int64_t CoreMathFixSqrt( int64_t raw )
{
	return b3FixSqrt( raw );
}

// The seed of the sqrt row's random raws, shared by the row and the witness so the witness
// plants into an input the row really runs.
#define CM_SQRT_RANDOM_SEED CM_SEED

typedef struct CoreMathSqrtResult
{
	int failed;
	uint64_t maxUlp;
	int64_t worst;
	int nonPositiveWrong;
} CoreMathSqrtResult;

// Run the sqrt row on fn: hard cases; every raw in [1, 2^20] (CI) or [-2^24, 2^24]
// (exhaustive); 2^20 (CI) or 2^28 (exhaustive) random positive raws.
static CoreMathSqrtResult CoreMathRunSqrtRow( const char* name, cmFn1* fn, bool exhaustive )
{
	CoreMathSqrtResult result = { 0 };
	cmRow row;
	cmRowBegin( &row, name );

	static const int64_t edges[] = { 1, (int64_t)1 << 48 };
	cmInputs hard = { 0 };
	if ( cmHardCases( &hard, edges, ARRAY_COUNT( edges ) ) == false )
	{
		printf( "core-math row=%s hard cases: allocation failed\n", name );
		result.failed = 1;
		return result;
	}
	cmRowRun1Array( &row, fn, cmRefSqrt, hard.raws, hard.count );
	for ( int i = 0; i < hard.count; ++i )
	{
		if ( hard.raws[i] <= 0 && fn( hard.raws[i] ) != 0 )
		{
			result.nonPositiveWrong += 1;
		}
	}
	cmInputsFree( &hard );

	if ( exhaustive )
	{
		for ( int64_t raw = -CM_EXHAUSTIVE_RANGE; raw <= 0; ++raw )
		{
			if ( fn( raw ) != 0 )
			{
				result.nonPositiveWrong += 1;
			}
		}
		cmRowRun1Range( &row, fn, cmRefSqrt, -CM_EXHAUSTIVE_RANGE, CM_EXHAUSTIVE_RANGE );
	}
	else
	{
		cmRowRun1Range( &row, fn, cmRefSqrt, 1, (int64_t)1 << 20 );
	}

	uint64_t state = CM_SQRT_RANDOM_SEED;
	int64_t randomCount = exhaustive ? ( (int64_t)1 << 28 ) : ( (int64_t)1 << 20 );
	cmRowRun1Random( &row, fn, cmRefSqrt, &state, randomCount, 1, INT64_MAX );

	result.failed = cmRowEnd( &row, CM_SQRT_BOUND );
	result.maxUlp = row.maxUlp;
	result.worst = row.worst;
	if ( result.nonPositiveWrong != 0 )
	{
		printf( "core-math row=%s raw<=0 not 0 on %d inputs\n", name, result.nonPositiveWrong );
		result.failed = 1;
	}
	return result;
}

static int CoreMathSqrtTest( void )
{
	// The reference itself, on values worked by hand: sqrt(1.0) = 1.0; sqrt(4.0) = 2.0;
	// raw 10 is sqrt(10 * 65536) = 809.54..., nearest 810; raw INT64_MAX is
	// sqrt((2^63 - 1) * 2^16) = 777472127993.87..., nearest 777472127994.
	ENSURE( cmRefSqrt( 65536 ) == 65536 );
	ENSURE( cmRefSqrt( 4 * 65536 ) == 2 * 65536 );
	ENSURE( cmRefSqrt( 10 ) == 810 );
	ENSURE( b3FixSqrt( 10 ) == 809 );
	ENSURE( cmRefSqrt( INT64_MAX ) == INT64_C( 777472127994 ) );
	ENSURE( cmRefSqrt( 0 ) == 0 && cmRefSqrt( -1 ) == 0 );

	CoreMathSqrtResult result = CoreMathRunSqrtRow( "sqrt", CoreMathFixSqrt, CoreMathExhaustive() );
	ENSURE( result.failed == 0 );
	return 0;
}

// ---------------------------------------------------------------------------------------------
// The reversed witness: the sqrt row on a planted function equal to b3FixSqrt except one ulp
// lower at a single input where fixSqrt is already one below the nearest, so the plant is 2
// off there. The row must go red and name exactly that input. Always run at CI size.
// ---------------------------------------------------------------------------------------------

static int64_t cmPlantRaw;

static int64_t CoreMathPlantedSqrt( int64_t raw )
{
	int64_t r = b3FixSqrt( raw );
	return raw == cmPlantRaw ? r - 1 : r;
}

static int CoreMathWitnessAt( int64_t plant )
{
	ENSURE( cmRefSqrt( plant ) - b3FixSqrt( plant ) == 1 );
	cmPlantRaw = plant;
	CoreMathSqrtResult result = CoreMathRunSqrtRow( "sqrt-witness", CoreMathPlantedSqrt, false );
	ENSURE( result.failed == 1 );
	ENSURE( result.maxUlp == 2 );
	ENSURE( result.worst == plant );
	printf( "core-math witness red maxulp=%" PRIu64 "\n", result.maxUlp );
	return 0;
}

static int CoreMathWitnessTest( void )
{
	// raw 10: fixSqrt(10) = 809, nearest 810.
	if ( CoreMathWitnessAt( 10 ) != 0 )
	{
		return 1;
	}

	// The first of the row's random dense raws where fixSqrt is one below the nearest, found by
	// replaying the row's own generator.
	uint64_t state = CM_SQRT_RANDOM_SEED;
	int64_t plant = 0;
	for ( int i = 0; i < ( 1 << 20 ); ++i )
	{
		int64_t raw = cmRandomRaw( &state, 1, INT64_MAX );
		if ( cmRefSqrt( raw ) - b3FixSqrt( raw ) == 1 )
		{
			plant = raw;
			break;
		}
	}
	ENSURE( plant != 0 );
	return CoreMathWitnessAt( plant );
}

// The five CORE-MATH references are linked and give the textbook values. pi/4 in Q48.16 is
// 51471.85... raw, nearest 51472.
static int CoreMathReferenceLinkCheck( void )
{
	ENSURE( cr_atan2( 0.0, 1.0 ) == 0.0 );
	ENSURE( cmRefFromDouble( cr_atan2( 1.0, 1.0 ) ) == 51472 );
	ENSURE( cr_sin( 0.0 ) == 0.0 );
	ENSURE( cr_cos( 0.0 ) == 1.0 );
	ENSURE( cr_log2( 8.0 ) == 3.0 );
	ENSURE( cr_exp2( -1.0 ) == 0.5 );
	ENSURE( cmRoundNearestEven( 2.5 ) == 2 && cmRoundNearestEven( 3.5 ) == 4 && cmRoundNearestEven( -2.5 ) == -2 );
	ENSURE( cmUlpDistance( INT64_MAX, INT64_MIN ) == UINT64_MAX );
	printf( "core-math reference linked: cr_atan2 cr_sin cr_cos cr_log2 cr_exp2\n" );
	return 0;
}

int CoreMathTest( void )
{
	if ( CoreMathReferenceLinkCheck() != 0 )
	{
		return 1;
	}

	RUN_SUBTEST( CoreMathSqrtTest );
	RUN_SUBTEST( CoreMathWitnessTest );

#if B3_CORE_MATH_ROW_ATAN2
	RUN_SUBTEST( CoreMathAtan2Test );
#else
	printf( "core-math row=atan2 pending: cm-fixed3d-atan2\n" );
#endif
#if B3_CORE_MATH_ROW_SINCOS
	RUN_SUBTEST( CoreMathSinCosTest );
#else
	printf( "core-math row=sincos pending: cm-fixed3d-sincos\n" );
#endif
#if B3_CORE_MATH_ROW_LOG2
	RUN_SUBTEST( CoreMathLog2Test );
#else
	printf( "core-math row=log2 pending: cm-fixed3d-log2\n" );
#endif
#if B3_CORE_MATH_ROW_EXP2
	RUN_SUBTEST( CoreMathExp2Test );
#else
	printf( "core-math row=exp2 pending: cm-fixed3d-exp2\n" );
#endif

	return 0;
}
