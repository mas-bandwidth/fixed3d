// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "shape.h"
#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"


static int AABBTest( void )
{
	b3AABB a;
	a.lowerBound = (b3Vec3){ -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) };
	a.upperBound = (b3Vec3){ -B3_FIX( 2.0f ), -B3_FIX( 2.0f ), -B3_FIX( 2.0f ) };

	ENSURE( b3IsValidAABB( a ) == false );

	a.upperBound = (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ) };
	ENSURE( b3IsValidAABB( a ) == true );

	b3AABB b = { { B3_FIX( 2.0f ), B3_FIX( 2.0f ) }, { B3_FIX( 4.0f ), B3_FIX( 4.0f ) } };
	ENSURE( b3AABB_Overlaps( a, b ) == false );
	ENSURE( b3AABB_Contains( a, b ) == false );

	return 0;
}

static int TestRayAABBIntersection( void )
{
	// Test 1: Ray passing through center of AABB
	{
		b3AABB a = { { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3FixAbs( minFraction - B3_FIX( 0.25f ) ) < B3_FIX( 0.001f ) ); // Enters at 25% of ray
		ENSURE( b3FixAbs( maxFraction - B3_FIX( 0.75f ) ) < B3_FIX( 0.001f ) ); // Exits at 75% of ray
	}

	// Test 2: Ray starting inside AABB
	{
		b3AABB a = { { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( minFraction == B3_FIX( 0.0f ) );						 // Starts inside
		ENSURE( b3FixAbs( maxFraction - B3_FIX( 0.5f ) ) < B3_FIX( 0.001f ) ); // Exits at 50% of ray
	}

	// Test 3: Ray ending inside AABB
	{
		b3AABB a = { { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Vec3 p2 = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3FixAbs( minFraction - B3_FIX( 0.5f ) ) < B3_FIX( 0.001f ) ); // Enters at 50% of ray
		ENSURE( maxFraction == B3_FIX( 1.0f ) );						 // Ends inside
	}

	// Test 4: Ray completely inside AABB
	{
		b3AABB a = { { -B3_FIX( 2.0f ), -B3_FIX( 2.0f ), -B3_FIX( 2.0f ) }, { B3_FIX( 2.0f ), B3_FIX( 2.0f ), B3_FIX( 2.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Vec3 p2 = { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( minFraction == B3_FIX( 0.0f ) );
		ENSURE( maxFraction == B3_FIX( 1.0f ) );
	}

	// Test 5: Ray missing AABB
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 0.5f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 2.0f ), B3_FIX( 0.5f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 6: Ray parallel to AABB face (no intersection)
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 0.5f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 2.0f ), B3_FIX( 0.5f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 7: Ray parallel to AABB face (within bounds)
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 1.0f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3FixAbs( minFraction - b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ) ) < B3_FIX( 0.001f ) );
		ENSURE( b3FixAbs( maxFraction - b3FixDiv( B3_FIX( 2.0f ) , B3_FIX( 3.0f ) ) ) < B3_FIX( 0.001f ) );
	}

	// Test 8: Degenerate ray (point) inside AABB
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) };
		b3Vec3 p2 = { B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( minFraction == B3_FIX( 0.0f ) );
		ENSURE( maxFraction == B3_FIX( 0.0f ) );
	}

	// Test 9: Degenerate ray (point) outside AABB
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { B3_FIX( 2.0f ), B3_FIX( 2.0f ), B3_FIX( 2.0f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 2.0f ), B3_FIX( 2.0f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 10: Ray pointing away from AABB
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 1.0f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) };
		b3Vec3 p2 = { -B3_FIX( 2.0f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == false );
	}

	// Test 11: Ray hitting corner of AABB
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 2.0f ), B3_FIX( 2.0f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3FixAbs( minFraction - b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ) ) < B3_FIX( 0.001f ) );
		ENSURE( b3FixAbs( maxFraction - b3FixDiv( B3_FIX( 2.0f ) , B3_FIX( 3.0f ) ) ) < B3_FIX( 0.001f ) );
	}

	// Test 12: Ray grazing edge of AABB
	{
		b3AABB a = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };
		b3Vec3 p1 = { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.5f ) };
		b3Vec3 p2 = { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.5f ) };
		b3Fixed minFraction, maxFraction;

		bool hit = b3RayCastAABB( a, p1, p2, &minFraction, &maxFraction );

		ENSURE( hit == true );
		ENSURE( b3FixAbs( minFraction - b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ) ) < B3_FIX( 0.001f ) );
		ENSURE( b3FixAbs( maxFraction - b3FixDiv( B3_FIX( 2.0f ) , B3_FIX( 3.0f ) ) ) < B3_FIX( 0.001f ) );
	}

	return 0;
}

// The narrow phase differences the two world positions in double then works in frame A, so a
// manifold far from the origin must match the same manifold at the origin. Float loses this past
// ~1e7 m where the ULP grows larger than the overlap, which is the whole point of large world mode.
static int LargeWorldManifoldTest( void )
{
	b3BoxHull boxA = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3BoxHull boxB = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	// Centers 0.9 apart so the cubes overlap by 0.1 along x
	b3Vec3 sep = { B3_FIX( 0.9f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };

	b3LocalManifoldPoint pointsOrigin[8];
	b3LocalManifold mOrigin = { 0 };
	mOrigin.points = pointsOrigin;

	b3WorldTransform xfAo = b3WorldTransform_identity;
	b3WorldTransform xfBo = { b3OffsetPos( b3Pos_zero, sep ), b3Quat_identity };
	b3SATCache cacheOrigin = { 0 };
	b3CollideHulls( &mOrigin, 8, &boxA.base, &boxB.base, b3InvMulWorldTransforms( xfAo, xfBo ), &cacheOrigin );

	// Two cube faces overlap, so the clipped manifold has four points
	ENSURE( mOrigin.pointCount == 4 );
	for ( int i = 0; i < mOrigin.pointCount; ++i )
	{
		ENSURE_SMALL( mOrigin.points[i].separation + B3_FIX( 0.1f ), B3_FIX( 0.01f ) );
	}

	return 0;
}

// Broad-phase AABBs are built in double and narrowed to b3Fixed with directed outward rounding, so a
// shape and its speculative margin stay inside their box far from the origin. A b3Fixed build would
// round the extent away into the ULP (~1 m at 1e7) and clip the shape out of its own box.
static int LargeWorldAABBTest( void )
{
	// Unit cube, so the tight extent is 0.5 each way
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3Shape shape = { 0 };
	shape.type = b3_hullShape;
	shape.hull = &box.base;

	b3AABB aabbOrigin = b3ComputeFatShapeAABB( &shape, b3WorldTransform_identity, B3_FIX( 0.0f ) );
	ENSURE_SMALL( aabbOrigin.lowerBound.x + B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( aabbOrigin.lowerBound.y + B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( aabbOrigin.lowerBound.z + B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( aabbOrigin.upperBound.x - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( aabbOrigin.upperBound.y - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( aabbOrigin.upperBound.z - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );

	return 0;
}

int CollisionTest( void )
{
	RUN_SUBTEST( AABBTest );
	RUN_SUBTEST( TestRayAABBIntersection );
	RUN_SUBTEST( LargeWorldManifoldTest );
	RUN_SUBTEST( LargeWorldAABBTest );

	return 0;
}
