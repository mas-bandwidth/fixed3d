// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "math_internal.h"
#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>


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

// xorshift64 state / next for the seeded generator
static uint64_t GSBState;
static uint64_t GSBNext( void )
{
	uint64_t x = GSBState;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	GSBState = x;
	return x;
}

// uniform raw Q48.16 integer in [lo, hi]
static b3Fixed GSBUniform( b3Fixed lo, b3Fixed hi )
{
	uint64_t span = (uint64_t)( hi - lo + 1 );
	return lo + (b3Fixed)( GSBNext() % span );
}

// FNV-1a 64 over raw bytes
static uint64_t GSBHashBytes( uint64_t hash, const void* data, size_t size )
{
	const unsigned char* bytes = (const unsigned char*)data;
	for ( size_t i = 0; i < size; ++i )
	{
		hash ^= bytes[i];
		hash *= 0x100000001b3ull;
	}
	return hash;
}

// Oracle: b3ShapeDistance, sphere (proxyB) against an axis-aligned box (proxyA),
// versus the closed-form distance. Box3D's contract with useRadii: closest points
// moved onto the surfaces, normal from A to B. The box proxyA is the 8 corners at
// +-hx, +-hy, +-hz with radius 0; the sphere proxyB is one point at the origin with
// radius r, placed by input.transform.p in A's frame.
static int GjkSphereBoxOracleTest( void )
{
	GSBState = 0x5EED0002ull;

	double maxErrDistance = 0.0;
	double maxErrPointA = 0.0;
	double maxErrPointB = 0.0;
	double maxErrNormal = 0.0;

	uint64_t hash = 0xcbf29ce484222325ull;

	int accepted = 0;
	while ( accepted < 256 )
	{
		b3Fixed hx = GSBUniform( 6553, 131072 ); // [0.1, 2.0]
		b3Fixed hy = GSBUniform( 6553, 131072 );
		b3Fixed hz = GSBUniform( 6553, 131072 );
		b3Fixed r = GSBUniform( 3276, 32768 );  // [0.05, 0.5]
		b3Fixed px = GSBUniform( -262144, 262144 ); // [-4.0, 4.0]
		b3Fixed py = GSBUniform( -262144, 262144 );
		b3Fixed pz = GSBUniform( -262144, 262144 );

		b3Vec3 vas[8];
		for ( int i = 0; i < 8; ++i )
		{
			int sx = ( i & 1 ) ? 1 : -1;
			int sy = ( i & 2 ) ? 1 : -1;
			int sz = ( i & 4 ) ? 1 : -1;
			vas[i] = (b3Vec3){ sx * hx, sy * hy, sz * hz };
		}

		b3Vec3 bpt = { 0, 0, 0 };

		b3DistanceInput input;
		input.proxyA = (b3ShapeProxy){ vas, 8, 0 };
		input.proxyB = (b3ShapeProxy){ &bpt, 1, r };
		input.transform.p = (b3Vec3){ px, py, pz };
		input.transform.q = b3Quat_identity;
		input.useRadii = true;

		b3SimplexCache cache = { 0 };
		b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );

		double hx_d = b3FixToDouble( hx );
		double hy_d = b3FixToDouble( hy );
		double hz_d = b3FixToDouble( hz );
		double r_d = b3FixToDouble( r );
		double p_x = b3FixToDouble( px );
		double p_y = b3FixToDouble( py );
		double p_z = b3FixToDouble( pz );

		double qx = p_x < -hx_d ? -hx_d : ( p_x > hx_d ? hx_d : p_x );
		double qy = p_y < -hy_d ? -hy_d : ( p_y > hy_d ? hy_d : p_y );
		double qz = p_z < -hz_d ? -hz_d : ( p_z > hz_d ? hz_d : p_z );

		double dx = p_x - qx;
		double dy = p_y - qy;
		double dz = p_z - qz;
		double c = sqrt( dx * dx + dy * dy + dz * dz );
		double dist = c - r_d;

		if ( dist < 0.01 )
		{
			continue;
		}

		double nx = dx / c;
		double ny = dy / c;
		double nz = dz / c;

		double oracleDistance = dist;
		double oraclePointA[3] = { qx, qy, qz };
		double oraclePointB[3] = { p_x - r_d * nx, p_y - r_d * ny, p_z - r_d * nz };
		double oracleNormal[3] = { nx, ny, nz };

		double gotDistance = b3FixToDouble( output.distance );
		double gotPointA[3] = { b3FixToDouble( output.pointA.x ), b3FixToDouble( output.pointA.y ), b3FixToDouble( output.pointA.z ) };
		double gotPointB[3] = { b3FixToDouble( output.pointB.x ), b3FixToDouble( output.pointB.y ), b3FixToDouble( output.pointB.z ) };
		double gotNormal[3] = { b3FixToDouble( output.normal.x ), b3FixToDouble( output.normal.y ), b3FixToDouble( output.normal.z ) };

		double errDistance = fabs( gotDistance - oracleDistance );
		if ( errDistance > maxErrDistance )
		{
			maxErrDistance = errDistance;
		}
		ENSURE( errDistance <= 2e-4 );

		for ( int i = 0; i < 3; ++i )
		{
			double errA = fabs( gotPointA[i] - oraclePointA[i] );
			double errB = fabs( gotPointB[i] - oraclePointB[i] );
			double errN = fabs( gotNormal[i] - oracleNormal[i] );
			if ( errA > maxErrPointA )
			{
				maxErrPointA = errA;
			}
			if ( errB > maxErrPointB )
			{
				maxErrPointB = errB;
			}
			if ( errN > maxErrNormal )
			{
				maxErrNormal = errN;
			}
			ENSURE( errA <= 2e-4 );
			ENSURE( errB <= 2e-4 );
			ENSURE( errN <= 1e-3 );
		}

		hash = GSBHashBytes( hash, &output.distance, sizeof( output.distance ) );
		hash = GSBHashBytes( hash, &output.pointA, sizeof( output.pointA ) );
		hash = GSBHashBytes( hash, &output.pointB, sizeof( output.pointB ) );
		hash = GSBHashBytes( hash, &output.normal, sizeof( output.normal ) );
		hash = GSBHashBytes( hash, &output.iterations, sizeof( output.iterations ) );

		++accepted;
	}

	printf( "  GjkSphereBoxOracleTest hash=0x%016llx\n", (unsigned long long)hash );
	printf( "  GjkSphereBoxOracleTest maxErr distance=%.3g pointA=%.3g pointB=%.3g normal=%.3g\n",
			maxErrDistance, maxErrPointA, maxErrPointB, maxErrNormal );

	// Golden harvested from build-gcc; build-clang and build-ludicrous give the
	// same value, so a single golden holds across all three builds.
	ENSURE( hash == 0xfcae1d0ac36fc733ull );

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
	RUN_SUBTEST( GjkSphereBoxOracleTest );

	return 0;
}
