// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "math_internal.h"
#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"


static int SegmentDistanceTest( void )
{
	b3Vec3 p1 = { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) };
	b3Vec3 q1 = { -B3_FIX( 1.0f ), B3_FIX( 1.0f ) };
	b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 0.0f ) };
	b3Vec3 q2 = { B3_FIX( 1.0f ), B3_FIX( 0.0f ) };

	b3SegmentDistanceResult result = b3SegmentDistance( p1, q1, p2, q2 );

	ENSURE_SMALL( result.fraction1 - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.fraction2 - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.point1.x + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.point1.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.point1.z, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.point2.x - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.point2.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.point2.z, 8 * B3_FIXED_EPSILON );

	return 0;
}

static int ShapeDistanceTest( void )
{
	b3Vec3 vas[] = { (b3Vec3){ -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, (b3Vec3){ B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ) }, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };

	b3Vec3 vbs[] = {
		(b3Vec3){ B3_FIX( 2.0f ), -B3_FIX( 1.0f ) },
		(b3Vec3){ B3_FIX( 2.0f ), B3_FIX( 1.0f ) },
	};

	b3DistanceInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), B3_FIX( 0.0f ) };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	input.transform = b3Transform_identity;
	input.useRadii = false;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );

	ENSURE_SMALL( output.distance - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );

	return 0;
}

static int ShapeCastTest( void )
{
	b3Vec3 vas[] = { (b3Vec3){ -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 1.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
					 (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };

	b3Vec3 vbs[] = {
		(b3Vec3){ B3_FIX( 2.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
		(b3Vec3){ B3_FIX( 2.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	};

	b3ShapeCastPairInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), B3_FIX( 0.0f ) };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	input.transform = b3Transform_identity;
	input.translationB = (b3Vec3){ -B3_FIX( 2.0f ), B3_FIX( 0.0f ) };
	input.maxFraction = B3_FIX( 1.0f );
	input.canEncroach = false;

	b3CastOutput output = b3ShapeCast( &input );

	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction - B3_FIX( 0.5f ), B3_FIX( 0.005f ) );

	return 0;
}

// unit box with a segment resting 2mm from its +x face: inside the cast
// target distance, not overlapped.
// Q48.16: B3_FIX( 1.002f ) is raw 65667, so the gap is raw 131 (~0.001999 m),
// which is above zero and at or below 2 * B3_LINEAR_SLOP (raw 656). That band
// is exactly the encroach arm's domain. One quantum is raw 1, so 2mm is 131
// quanta of separation and nothing here is near the resolution floor.
static b3ShapeCastPairInput EncroachCastInput( b3Vec3 translationB )
{
	static const b3Vec3 vas[] = { { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
								  { B3_FIX( 1.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
								  { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
								  { -B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) } };
	static const b3Vec3 vbs[] = {
		{ B3_FIX( 1.002f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 1.002f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	};

	b3ShapeCastPairInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), B3_FIX( 0.0f ) };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	input.transform = b3Transform_identity;
	input.translationB = translationB;
	input.maxFraction = B3_FIX( 1.0f );
	input.canEncroach = true;
	return input;
}

static int ShapeCastEncroachRecedeTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit == false );

	return 0;
}

static int ShapeCastEncroachSlideTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } );
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit == false );

	return 0;
}

static int ShapeCastEncroachClosingTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( output.normal.x - B3_FIX( 1.0f ), B3_FIX( 0.001f ) );

	return 0;
}

static int ShapeCastEncroachGradedTest( void )
{
	// 5mm gap: closing consumes the gap down to the retained rest separation.
	// B3_FIX( 1.005f ) is raw 65864, so the gap is raw 328 and the retained
	// half slop is raw 164, leaving raw 164 needed against a raw 65536 approach:
	// fraction raw 164, and B3_FIX( 0.0025f ) is raw 164 as well.
	static const b3Vec3 vbs[] = {
		{ B3_FIX( 1.005f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 1.005f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	};

	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction - B3_FIX( 0.0025f ), B3_FIX( 0.0005f ) );
	ENSURE_SMALL( output.normal.x - B3_FIX( 1.0f ), B3_FIX( 0.001f ) );

	return 0;
}

static int ShapeCastEncroachMaxFractionTest( void )
{
	// 5mm gap closes at fraction 0.0025 (raw 164). B3_FIX( 0.002f ) is raw 131
	// and B3_FIX( 0.003f ) is raw 197, so the raw fraction straddles them.
	static const b3Vec3 vbs[] = {
		{ B3_FIX( 1.005f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 1.005f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	};

	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	input.maxFraction = B3_FIX( 0.002f );
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit == false );

	input.maxFraction = B3_FIX( 0.003f );
	output = b3ShapeCast( &input );
	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction - B3_FIX( 0.0025f ), B3_FIX( 0.0005f ) );

	return 0;
}

static int ShapeCastEncroachGrazeTest( void )
{
	// mostly tangential slide with a slight closing component: free.
	// The closing component is raw 66 against a raw 164 need, so the first
	// order approach cannot consume the gap and the sweep misses.
	static const b3Vec3 vbs[] = {
		{ B3_FIX( 1.005f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 1.005f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	};

	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ -B3_FIX( 0.001f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) } );
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit == false );

	return 0;
}

// CONTROL, not evidence of the fix: canEncroach false must keep the old
// initial-overlap behavior for a shape inside the target band.
static int ShapeCastStartTouchingDefaultTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	input.canEncroach = false;
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );

	return 0;
}

// CONTROL, not evidence of the fix: a genuinely overlapped start still reports
// the initial overlap even with canEncroach set, because the new arm is gated
// on a strictly positive distance.
static int ShapeCastEncroachOverlapTest( void )
{
	static const b3Vec3 vcs[] = {
		{ B3_FIX( 0.5f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 0.5f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	};

	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	input.proxyB = (b3ShapeProxy){ vcs, ARRAY_COUNT( vcs ), B3_FIX( 0.0f ) };
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );

	return 0;
}

// The retained rest separation must be b3FixMul( B3_FIX( 0.5f ), linearSlop ) and
// NOT an integer halving of the raw. At the DEFAULT slop (raw 328, even) the two
// spellings agree at 164, so nothing in the suite above can tell them apart. They
// differ on an ODD slop raw, which b3SetLengthUnitsPerMeter can produce.
//
// MEASURED on this tree, raw values, with a probe linked against the library:
//   - lengthUnits raw 100..299 give linearSlop raw 1 (halving 0, b3FixMul 1)
//   - lengthUnits raw 500..699 give linearSlop raw 3 (halving 1, b3FixMul 2)
// Raw 1 is unreachable for a zero-radius cast: tolerance quantizes to 0 and
// target is 1, so iteration 0 is entered only at distance 0, which is the
// initial-overlap arm. Raw 3 IS reachable: tolerance is raw 1, target is raw 3,
// so iteration 0 takes any distance up to raw 3 and the gate ( 0, 2 * slop = 6 ]
// admits it.
//
// At lengthUnits raw 512 (linearSlop raw 3) a closing sweep of raw 2 against a
// core gap of raw 3 discriminates:
//   halving:  retained 1, needed 2, approach 2 -> approach <= needed -> MISS
//   b3FixMul: retained 2, needed 1, approach 2 -> fraction = 1/2 -> HIT at 0.5
// Upstream's float retains 0.5 raw and hits at fraction 0.75, so the b3FixMul
// spelling keeps upstream's HIT where the halving turns it into a miss.
static int ShapeCastEncroachOddSlopRetainedTest( void )
{
	static const b3Vec3 vas[] = { { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
								  { B3_FIX( 1.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
								  { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
								  { -B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) } };

	// Three raw units of core separation, so the gap is odd-slop sized rather than
	// authored in meters: B3_FIX( 1.0f ) + 3 is raw 65539.
	static const b3Vec3 vbs[] = {
		{ B3_FIX( 1.0f ) + 3, -B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 1.0f ) + 3, B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	};

	b3SetLengthUnitsPerMeter( 512 );

	b3Fixed slop = B3_LINEAR_SLOP;

	b3ShapeCastPairInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), B3_FIX( 0.0f ) };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	input.transform = b3Transform_identity;
	input.translationB = (b3Vec3){ -2, B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	input.maxFraction = B3_FIX( 1.0f );
	input.canEncroach = true;

	b3CastOutput closing = b3ShapeCast( &input );

	// A raw 3 approach against the same gap: the fraction itself discriminates.
	// halving needed 2 / approach 3 = raw 43690; b3FixMul needed 1 / 3 = raw 21845.
	input.translationB = (b3Vec3){ -3, B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3CastOutput graded = b3ShapeCast( &input );

	// Restore the default before asserting, so a failure here cannot change the
	// length scale under every test that runs after this one.
	b3SetLengthUnitsPerMeter( B3_FIX( 1.0f ) );

	ENSURE( slop == 3 );
	ENSURE( closing.hit );
	ENSURE( closing.fraction == B3_FIX( 0.5f ) );
	ENSURE( graded.hit );
	ENSURE( graded.fraction == 21845 );

	return 0;
}

static int TimeOfImpactTest( void )
{
	b3Vec3 vas[] = { { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, { B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ) }, { -B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };

	b3Vec3 vbs[] = {
		{ B3_FIX( 2.0f ), -B3_FIX( 1.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 1.0f ) },
	};

	b3TOIInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), B3_FIX( 0.0f ) };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), B3_FIX( 0.0f ) };
	input.sweepA = (b3Sweep){ b3Vec3_zero, b3Vec3_zero, b3Vec3_zero, b3Quat_identity, b3Quat_identity };
	input.sweepB = (b3Sweep){ b3Vec3_zero, b3Vec3_zero, (b3Vec3){ -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity, b3Quat_identity };
	input.maxFraction = B3_FIX( 1.0f );

	b3TOIOutput output = b3TimeOfImpact( &input );

	ENSURE( output.state == b3_toiStateHit );
	ENSURE_SMALL( output.fraction - B3_FIX( 0.5f ), B3_FIX( 0.005f ) );

	return 0;
}

static int LongEdgeDistanceTest( void )
{
	// A point above an interior point of a segment has an analytical witness.
	// At length > 256 the Q48.16 reciprocal of the squared length is zero.
	const int lengths[] = { 128, 256, 257, 512, 1024 };
	for ( int i = 0; i < ARRAY_COUNT( lengths ); ++i )
	{
		b3Fixed length = b3FixFromInt( lengths[i] );
		b3Vec3 edge[2] = { { B3_FIX( 3.0f ), B3_FIX( 4.0f ), 0 }, { B3_FIX( 3.0f ) + length, B3_FIX( 4.0f ), 0 } };
		// The extra unit makes the weight nonrepresentable for non-power-of-two edges.
		b3Vec3 point = { B3_FIX( 4.0f ) + length / 4, B3_FIX( 6.0f ), 0 };
		b3DistanceInput input = { 0 };
		input.proxyA = (b3ShapeProxy){ edge, 2, 0 };
		input.proxyB = (b3ShapeProxy){ &point, 1, 0 };
		input.transform = b3Transform_identity;
		b3SimplexCache cache = { 0 };
		for ( int repeat = 0; repeat < 2; ++repeat )
		{
			b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
			ENSURE_SMALL( output.distance - B3_FIX( 2.0f ), B3_FIX( 0.001f ) );
			ENSURE_SMALL( output.pointA.x - point.x, lengths[i] * B3_FIXED_EPSILON + 2 );
			ENSURE_SMALL( output.pointA.y - B3_FIX( 4.0f ), 2 * B3_FIXED_EPSILON );
			ENSURE_SMALL( output.pointB.x - point.x, 2 * B3_FIXED_EPSILON );
			ENSURE_SMALL( output.pointB.y - point.y, 2 * B3_FIXED_EPSILON );
		}
	}
	return 0;
}

int DistanceTest( void )
{
	RUN_SUBTEST( LongEdgeDistanceTest );
	RUN_SUBTEST( SegmentDistanceTest );
	RUN_SUBTEST( ShapeDistanceTest );
	RUN_SUBTEST( ShapeCastTest );
	RUN_SUBTEST( ShapeCastEncroachRecedeTest );
	RUN_SUBTEST( ShapeCastEncroachSlideTest );
	RUN_SUBTEST( ShapeCastEncroachClosingTest );
	RUN_SUBTEST( ShapeCastEncroachGradedTest );
	RUN_SUBTEST( ShapeCastEncroachMaxFractionTest );
	RUN_SUBTEST( ShapeCastEncroachGrazeTest );
	RUN_SUBTEST( ShapeCastStartTouchingDefaultTest );
	RUN_SUBTEST( ShapeCastEncroachOverlapTest );
	RUN_SUBTEST( ShapeCastEncroachOddSlopRetainedTest );
	RUN_SUBTEST( TimeOfImpactTest );

	return 0;
}
