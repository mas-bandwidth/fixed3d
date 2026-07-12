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

	b3SimplexCache cache = { b3FixFromInt( 0 ) };
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

int DistanceTest( void )
{
	RUN_SUBTEST( SegmentDistanceTest );
	RUN_SUBTEST( ShapeDistanceTest );
	RUN_SUBTEST( ShapeCastTest );
	RUN_SUBTEST( TimeOfImpactTest );

	return 0;
}
