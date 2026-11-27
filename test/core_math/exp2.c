// SPDX-FileCopyrightText: 2026 Fixed3D core-math harness
// SPDX-License-Identifier: MIT
//
// Row exp2: b3FixExp2 (fixExp2) against CORE-MATH's cr_exp2, under the policy in
// docs/determinism-math.md. Bound: 1 ulp for x < 8; 5 ulps for 8 <= x < 16; relative
// 2^-29 for 16 <= x < 47; exactly INT64_MAX at and above 47.

#include "core_math_harness.h"
#include "test_macros.h"

static int64_t CoreMathFixExp2( int64_t raw )
{
	return b3FixExp2( raw );
}

static int64_t CoreMathRefExp2( int64_t raw )
{
	return cmRefFromDouble( cr_exp2( cmRawToDouble( raw ) ) );
}

// The relative row for [16, 47). The error per input is |got - want| / want; the largest is
// printed as a hex float (%a) against the 2^-29 bound, with the raw that reached it.
typedef struct Exp2RelRow
{
	int64_t inputs;
	double maxRel;
	int64_t worst;
} Exp2RelRow;

static void exp2RelRowAdd( Exp2RelRow* row, int64_t raw, int64_t got, int64_t want )
{
	int64_t diff = got > want ? got - want : want - got;
	double rel = (double)diff / (double)want;
	if ( rel > row->maxRel || row->inputs == 0 )
	{
		row->maxRel = rel;
		row->worst = raw;
	}
	row->inputs += 1;
}

static void exp2RelRowRandom( Exp2RelRow* row, uint64_t* state, int64_t count, int64_t lo, int64_t hi )
{
	for ( int64_t i = 0; i < count; ++i )
	{
		int64_t raw = cmRandomRaw( state, lo, hi );
		exp2RelRowAdd( row, raw, CoreMathFixExp2( raw ), CoreMathRefExp2( raw ) );
	}
}

static int exp2RelRowEnd( const Exp2RelRow* row )
{
	printf( "core-math row=exp2[16,47) inputs=%" PRId64 " maxrel=%a bound=0x1p-29 worst=%" PRId64 "\n",
			row->inputs, row->maxRel, row->worst );
	return row->maxRel > 0x1p-29 ? 1 : 0;
}

int CoreMathExp2Test( void )
{
	// Saturation at and above 47, per the policy.
	ENSURE( b3FixExp2( 47 * 65536 ) == INT64_MAX );
	ENSURE( b3FixExp2( 47 * 65536 + 1 ) == INT64_MAX );
	ENSURE( b3FixExp2( INT64_MAX ) == INT64_MAX );

	bool exhaustive = CoreMathExhaustive();
	int failed = 0;

	// Piece [-48, 8), bound 1 ulp: every raw in [-17*65536, 8*65536) plus 2^18 random raws
	// in [-48, -17) at CI; every raw in the whole piece at exhaustive.
	cmRow row1;
	cmRowBegin( &row1, "exp2[-48,8)" );
	if ( exhaustive )
	{
		cmRowRun1Range( &row1, CoreMathFixExp2, CoreMathRefExp2, -48 * 65536, 8 * 65536 - 1 );
	}
	else
	{
		cmRowRun1Range( &row1, CoreMathFixExp2, CoreMathRefExp2, -17 * 65536, 8 * 65536 - 1 );
		uint64_t state1 = CM_SEED;
		cmRowRun1Random( &row1, CoreMathFixExp2, CoreMathRefExp2, &state1, (int64_t)1 << 18, -48 * 65536,
						 -17 * 65536 - 1 );
	}
	failed |= cmRowEnd( &row1, 1u );

	// Piece [8, 16), bound 5 ulps: every raw.
	cmRow row2;
	cmRowBegin( &row2, "exp2[8,16)" );
	cmRowRun1Range( &row2, CoreMathFixExp2, CoreMathRefExp2, 8 * 65536, 16 * 65536 - 1 );
	failed |= cmRowEnd( &row2, 5u );

	// Piece [16, 47), relative bound 2^-29: 2^20 random raws plus the last 4096 raws below
	// 47*65536 at CI; every raw in the whole piece at exhaustive.
	Exp2RelRow rel = { 0 };
	if ( exhaustive )
	{
		for ( int64_t raw = 16 * 65536;; ++raw )
		{
			exp2RelRowAdd( &rel, raw, CoreMathFixExp2( raw ), CoreMathRefExp2( raw ) );
			if ( raw == 47 * 65536 - 1 )
			{
				break;
			}
		}
	}
	else
	{
		uint64_t state3 = CM_SEED;
		exp2RelRowRandom( &rel, &state3, (int64_t)1 << 20, 16 * 65536, 47 * 65536 - 1 );
		for ( int64_t raw = 47 * 65536 - 4096; raw < 47 * 65536; ++raw )
		{
			exp2RelRowAdd( &rel, raw, CoreMathFixExp2( raw ), CoreMathRefExp2( raw ) );
		}
	}
	failed |= exp2RelRowEnd( &rel );

	ENSURE( failed == 0 );
	return 0;
}
