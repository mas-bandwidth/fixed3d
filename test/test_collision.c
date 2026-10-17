// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "shape.h"
#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stdint.h>


static int AABBTest( void )
{
	b3AABB a;
	a.lowerBound = b3Vec3ToBound( (b3Vec3){ -B3_FIX( 1.0f ), -B3_FIX( 1.0f ), -B3_FIX( 1.0f ) } );
	a.upperBound = b3Vec3ToBound( (b3Vec3){ -B3_FIX( 2.0f ), -B3_FIX( 2.0f ), -B3_FIX( 2.0f ) } );

	ENSURE( b3IsValidAABB( a ) == false );

	a.upperBound = b3Vec3ToBound( (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ) } );
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

static uint64_t CBF_State;
static uint64_t CBF_Hash;
static double CBF_MaxErrNormal;
static double CBF_MaxErrSeparation;
static double CBF_MaxErrPoint;

static uint64_t CBF_XorShift64( void )
{
	uint64_t x = CBF_State;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	CBF_State = x;
	return x;
}

static b3Fixed CBF_RandRange( b3Fixed lo, b3Fixed hi )
{
	if ( hi <= lo )
	{
		return lo;
	}
	uint64_t span = (uint64_t)( hi - lo ) + 1;
	uint64_t v = CBF_XorShift64();
	return lo + (b3Fixed)( v % span );
}

static void CBF_HashRaw( int64_t value )
{
	uint64_t v = (uint64_t)value;
	for ( int i = 0; i < 8; ++i )
	{
		CBF_Hash ^= (uint64_t)( ( v >> ( 8 * i ) ) & 0xFFULL );
		CBF_Hash *= 0x100000001b3ULL;
	}
}

// Oracle: b3CollideHullAndCapsule, shallow branch. When the closest-points direction is within
// 0.998 of the reference face normal and the clipped segment keeps both ends, the manifold is two
// points: for each end e with plane distance d = n . e - planeOffset, separation = d - r and
// point = e - 0.5 * (r + d) * n, with normal = n. The capsule is laid on a box's top face so the
// reference face is the +y face and the closed form below is Box3D's contract.
static int CapsuleBoxFaceManifoldOracleTest( void )
{
	CBF_State = 0x5EED000AULL;
	CBF_Hash = 0xcbf29ce484222325ULL;
	CBF_MaxErrNormal = 0.0;
	CBF_MaxErrSeparation = 0.0;
	CBF_MaxErrPoint = 0.0;

	b3Fixed hLo = b3FixFromDouble( 0.3 );
	b3Fixed hHi = b3FixFromDouble( 2.0 );
	b3Fixed rLo = b3FixFromDouble( 0.05 );
	b3Fixed rHi = b3FixFromDouble( 0.3 );
	b3Fixed gLo = b3FixFromDouble( -0.01 );
	b3Fixed gHi = b3FixFromDouble( 0.015 );

	int accepted = 0;
	while ( accepted < 256 )
	{
		b3Fixed hx = CBF_RandRange( hLo, hHi );
		b3Fixed hy = CBF_RandRange( hLo, hHi );
		b3Fixed hz = CBF_RandRange( hLo, hHi );
		b3Fixed r = CBF_RandRange( rLo, rHi );

		b3Fixed xLim = b3FixFromDouble( b3FixToDouble( hx ) - 0.05 );
		b3Fixed zLim = b3FixFromDouble( b3FixToDouble( hz ) - 0.05 );

		b3Fixed e1x = CBF_RandRange( -xLim, xLim );
		b3Fixed e1z = CBF_RandRange( -zLim, zLim );
		b3Fixed e1g = CBF_RandRange( gLo, gHi );
		b3Fixed e1y = hy + r + e1g;

		b3Fixed e2x = CBF_RandRange( -xLim, xLim );
		b3Fixed e2z = CBF_RandRange( -zLim, zLim );
		b3Fixed e2g = CBF_RandRange( gLo, gHi );
		b3Fixed e2y = hy + r + e2g;

		double dx = b3FixToDouble( e2x ) - b3FixToDouble( e1x );
		double dz = b3FixToDouble( e2z ) - b3FixToDouble( e1z );
		if ( sqrt( dx * dx + dz * dz ) < 0.1 )
		{
			continue;
		}

		b3BoxHull hull = b3MakeBoxHull( hx, hy, hz );
		b3Capsule capsule;
		capsule.center1 = (b3Vec3){ e1x, e1y, e1z };
		capsule.center2 = (b3Vec3){ e2x, e2y, e2z };
		capsule.radius = r;

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndCapsule( &manifold, 8, &hull.base, &capsule, b3Transform_identity, &cache );

		ENSURE( manifold.pointCount == 2 );

		double nx = b3FixToDouble( manifold.normal.x );
		double ny = b3FixToDouble( manifold.normal.y );
		double nz = b3FixToDouble( manifold.normal.z );
		double nerr = fmax( fmax( fabs( nx ), fabs( ny - 1.0 ) ), fabs( nz ) );
		if ( nerr > CBF_MaxErrNormal )
		{
			CBF_MaxErrNormal = nerr;
		}
		ENSURE( nerr < 1e-3 );

		double hyD = b3FixToDouble( hy );
		double rD = b3FixToDouble( r );
		double ex[2] = { b3FixToDouble( e1x ), b3FixToDouble( e2x ) };
		double ey[2] = { b3FixToDouble( e1y ), b3FixToDouble( e2y ) };
		double ez[2] = { b3FixToDouble( e1z ), b3FixToDouble( e2z ) };

		double sepO[2], pxO[2], pyO[2], pzO[2];
		for ( int i = 0; i < 2; ++i )
		{
			double d = ey[i] - hyD;
			sepO[i] = d - rD;
			pxO[i] = ex[i];
			pyO[i] = ey[i] - 0.5 * ( rD + d );
			pzO[i] = ez[i];
		}

		double mx[2] = { b3FixToDouble( manifold.points[0].point.x ), b3FixToDouble( manifold.points[1].point.x ) };
		double my[2] = { b3FixToDouble( manifold.points[0].point.y ), b3FixToDouble( manifold.points[1].point.y ) };
		double mz[2] = { b3FixToDouble( manifold.points[0].point.z ), b3FixToDouble( manifold.points[1].point.z ) };
		double ms[2] = { b3FixToDouble( manifold.points[0].separation ), b3FixToDouble( manifold.points[1].separation ) };

		bool used[2] = { false, false };
		for ( int i = 0; i < 2; ++i )
		{
			int best = -1;
			double bestD = 1e30;
			for ( int j = 0; j < 2; ++j )
			{
				if ( used[j] )
				{
					continue;
				}
				double ddx = mx[j] - ex[i];
				double ddy = my[j] - ey[i];
				double ddz = mz[j] - ez[i];
				double dd = ddx * ddx + ddy * ddy + ddz * ddz;
				if ( dd < bestD )
				{
					bestD = dd;
					best = j;
				}
			}
			used[best] = true;
			double seperr = fabs( ms[best] - sepO[i] );
			if ( seperr > CBF_MaxErrSeparation )
			{
				CBF_MaxErrSeparation = seperr;
			}
			double perr = fmax( fmax( fabs( mx[best] - pxO[i] ), fabs( my[best] - pyO[i] ) ), fabs( mz[best] - pzO[i] ) );
			if ( perr > CBF_MaxErrPoint )
			{
				CBF_MaxErrPoint = perr;
			}
			ENSURE( seperr < 2e-4 );
			ENSURE( perr < 2e-4 );
		}

		CBF_HashRaw( (int64_t)manifold.pointCount );
		CBF_HashRaw( manifold.normal.x );
		CBF_HashRaw( manifold.normal.y );
		CBF_HashRaw( manifold.normal.z );
		for ( int k = 0; k < 2; ++k )
		{
			CBF_HashRaw( manifold.points[k].point.x );
			CBF_HashRaw( manifold.points[k].point.y );
			CBF_HashRaw( manifold.points[k].point.z );
			CBF_HashRaw( manifold.points[k].separation );
		}

		++accepted;
	}

	printf( "  CapsuleBoxFaceManifoldOracleTest hash=0x%016llx\n", (unsigned long long)CBF_Hash );
	printf( "  CapsuleBoxFaceManifoldOracleTest max error normal=%g sep=%g point=%g\n", CBF_MaxErrNormal, CBF_MaxErrSeparation, CBF_MaxErrPoint );

	ENSURE( CBF_Hash == 0x0706a59c6e4e1cefULL );

	return 0;
}

int CollisionTest( void )
{
	RUN_SUBTEST( AABBTest );
	RUN_SUBTEST( TestRayAABBIntersection );
	RUN_SUBTEST( LargeWorldManifoldTest );
	RUN_SUBTEST( LargeWorldAABBTest );
	RUN_SUBTEST( CapsuleBoxFaceManifoldOracleTest );

	return 0;
}
