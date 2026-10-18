// SPDX-FileCopyrightText: 2026 Fixed3D core-math harness
// SPDX-License-Identifier: MIT
//
// Measure deterministic fixed-point sine and cosine implementations against CORE-MATH.

#include "core_math_harness.h"
#include "test_macros.h"

#define CM_SINCOS_SMALL_BOUND 107u
#define CM_SINCOS_SMALL_RAW ( INT64_C( 32 ) * 65536 )
#define CM_SINCOS_MID_RAW ( INT64_C( 1 ) << 37 )

typedef struct SinCosMeasure
{
	cmRow row;
	int64_t maxExcess;
} SinCosMeasure;

static uint64_t cmAbsRaw( int64_t raw )
{
	return raw < 0 ? (uint64_t)( -( raw + 1 ) ) + 1u : (uint64_t)raw;
}

static int64_t CoreMathSinFix( int64_t raw )
{
	return fixComputeCosSin( raw ).sine;
}

static int64_t CoreMathCosFix( int64_t raw )
{
	return fixComputeCosSin( raw ).cosine;
}

static int64_t CoreMathSinB3( int64_t raw )
{
	return b3ComputeCosSin( raw ).sine;
}

static int64_t CoreMathCosB3( int64_t raw )
{
	return b3ComputeCosSin( raw ).cosine;
}

static int64_t cmRefSin( int64_t raw )
{
	return cmRefFromDouble( cr_sin( cmRawToDouble( raw ) ) );
}

static int64_t cmRefCos( int64_t raw )
{
	return cmRefFromDouble( cr_cos( cmRawToDouble( raw ) ) );
}

static void cmSinCosAdd( SinCosMeasure* measure, int64_t raw, int64_t got, int64_t want )
{
	cmRowAdd( &measure->row, raw, got, want );
}

static int cmSmallEnd( const SinCosMeasure* measure )
{
	return cmRowEnd( &measure->row, CM_SINCOS_SMALL_BOUND );
}

static int cmMidEnd( const SinCosMeasure* measure )
{
	printf( "core-math row=%s inputs=%" PRId64 " maxulp=%" PRIu64 " excess=%" PRId64 " worst=%" PRId64 "\n",
			measure->row.name, measure->row.inputs, measure->row.maxUlp, measure->maxExcess, measure->row.worst );
	return measure->maxExcess > 0;
}

static void cmSinCosAddMid( SinCosMeasure* measure, int64_t raw, int64_t got, int64_t want )
{
	cmSinCosAdd( measure, raw, got, want );
	uint64_t magnitude = cmAbsRaw( raw );
	const uint64_t sixteenth = UINT64_C( 16 ) * 65536u;
	uint64_t bound = 108u + ( magnitude + sixteenth - 1u ) / sixteenth;
	int64_t excess = (int64_t)cmUlpDistance( got, want ) - (int64_t)bound;
	if ( measure->row.inputs == 1 || excess > measure->maxExcess )
	{
		measure->maxExcess = excess;
	}
}

static void cmSinCosRecordGroup( SinCosMeasure measures[4][3], cmFn1* functions[4], int64_t raw, int piece )
{
	int64_t refs[2] = { cmRefSin( raw ), cmRefCos( raw ) };
	for ( int i = 0; i < 4; ++i )
	{
		int64_t got = functions[i]( raw );
		if ( piece == 1 )
		{
			cmSinCosAddMid( &measures[i][piece], raw, got, refs[i & 1] );
		}
		else
		{
			cmSinCosAdd( &measures[i][piece], raw, got, refs[i & 1] );
		}
	}
}

int CoreMathSinCosTest( void )
{
	const char* names[4] = { "sin-fix", "cos-fix", "sin-b3", "cos-b3" };
	cmFn1* functions[4] = { CoreMathSinFix, CoreMathCosFix, CoreMathSinB3, CoreMathCosB3 };
	SinCosMeasure measures[4][3] = { 0 };
	char rowNames[4][3][32];
	for ( int i = 0; i < 4; ++i )
	{
		snprintf( rowNames[i][0], sizeof( rowNames[i][0] ), "%s[0,32)", names[i] );
		snprintf( rowNames[i][1], sizeof( rowNames[i][1] ), "%s[32,2^21)", names[i] );
		snprintf( rowNames[i][2], sizeof( rowNames[i][2] ), "%s[2^21,inf)", names[i] );
		cmRowBegin( &measures[i][0].row, rowNames[i][0] );
		cmRowBegin( &measures[i][1].row, rowNames[i][1] );
		cmRowBegin( &measures[i][2].row, rowNames[i][2] );
		measures[i][1].maxExcess = INT64_MIN;
	}

	const int64_t edges[] = { -CM_SINCOS_MID_RAW, -CM_SINCOS_SMALL_RAW, CM_SINCOS_SMALL_RAW, CM_SINCOS_MID_RAW };
	cmInputs hard = { 0 };
	if ( cmHardCases( &hard, edges, 4 ) == false )
	{
		printf( "core-math row=sincos hard cases: allocation failed\n" );
		return 1;
	}

	for ( int i = 0; i < hard.count; ++i )
	{
		int64_t raw = hard.raws[i];
		uint64_t magnitude = cmAbsRaw( raw );
		int piece = magnitude < (uint64_t)CM_SINCOS_SMALL_RAW ? 0 :
					magnitude < (uint64_t)CM_SINCOS_MID_RAW ? 1 : 2;
		cmSinCosRecordGroup( measures, functions, raw, piece );
	}

	if ( CoreMathExhaustive() )
	{
		for ( int64_t raw = -CM_SINCOS_SMALL_RAW + 1; raw < CM_SINCOS_SMALL_RAW; ++raw )
		{
			cmSinCosRecordGroup( measures, functions, raw, 0 );
		}
	}
	else
	{
		const int64_t count = (int64_t)1 << 20;
		const int64_t step = 2 * CM_SINCOS_SMALL_RAW / count;
		for ( int64_t i = 0; i < count; ++i )
		{
			int64_t raw = -CM_SINCOS_SMALL_RAW + step / 2 + i * step;
			if ( cmAbsRaw( raw ) < (uint64_t)CM_SINCOS_SMALL_RAW )
			{
				cmSinCosRecordGroup( measures, functions, raw, 0 );
			}
		}
	}

	// Split 2^20 deterministic random inputs evenly across octaves [2^5, 2^21) rad.
	uint64_t state = CM_SEED;
	const int64_t perOctave = cmRandomCount() / 16;
	for ( int octave = 5; octave <= 20; ++octave )
	{
		uint64_t lo = UINT64_C( 1 ) << ( octave + 16 );
		uint64_t range = UINT64_C( 1 ) << ( octave + 16 );
		for ( int64_t i = 0; i < perOctave; ++i )
		{
			int64_t raw = (int64_t)( lo + ( cmNext( &state ) & ( range - 1u ) ) );
			if ( ( i & 1 ) != 0 )
			{
				raw = -raw;
			}
			cmSinCosRecordGroup( measures, functions, raw, 1 );
		}
	}

	cmInputsFree( &hard );
	int failed = 0;
	for ( int i = 0; i < 4; ++i )
	{
		failed |= cmSmallEnd( &measures[i][0] );
		failed |= cmMidEnd( &measures[i][1] );
		printf( "core-math row=%s[2^21,inf) unbounded maxulp=%" PRIu64 "\n",
				names[i], measures[i][2].row.maxUlp );
	}
	ENSURE( failed == 0 );
	return 0;
}
