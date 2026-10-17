// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "manifold.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <math.h>

// A face contact keeps the whole clipped polygon when the manifold has room for it, so a build
// with a larger B3_MAX_MANIFOLD_POINTS legitimately reports more than four points. At the default
// this is still an exact count.
#define ENSURE_FACE_POINTS( count ) ENSURE( 4 <= ( count ) && ( count ) <= B3_MAX_MANIFOLD_POINTS )

static const b3Fixed kRoot2 = B3_FIX( 1.41421356f );
static const b3Fixed kHalfRoot2 = B3_FIX( 0.70710678f );

static const b3Vec3 kAxisX = { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
static const b3Vec3 kAxisY = { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
static const b3Vec3 kAxisZ = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };

// b3ComputeCosSin is a rational approximation good to about 1e-3. That is coarse enough to
// shift an edge off the position the analytic result expects, so these fixtures need libm.
static b3Quat ExactQuat( b3Vec3 axis, b3Fixed radians )
{
	double half = 0.5 * b3FixToDouble( radians );
	b3Fixed s = b3FixFromDouble( sin( half ) );
	b3Fixed c = b3FixFromDouble( cos( half ) );
	return (b3Quat){ { b3FixMul( s, axis.x ), b3FixMul( s, axis.y ), b3FixMul( s, axis.z ) }, c };
}

static b3Transform ExactRotation( b3Vec3 axis, b3Fixed radians )
{
	return (b3Transform){ b3Vec3_zero, ExactQuat( axis, radians ) };
}

// Cube A yawed 45 degrees presents an edge along y at x = +h*root2.
// Cube B rolled 45 degrees presents an edge along z at x = -h*root2.
// Sliding B along x makes those two edges the closest features, so the axis of minimum
// penetration is x, the separation is d - 2*h*root2 and the contact point sits at x = d/2.
// Both hulls are far from a face axis here, which keeps the edge query in charge.
static void MakeCrossedEdgeHulls( b3BoxHull* hullA, b3BoxHull* hullB, b3Fixed halfWidth )
{
	*hullA = b3MakeTransformedBoxHull( halfWidth, halfWidth, halfWidth, ExactRotation( kAxisY, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );
	*hullB = b3MakeTransformedBoxHull( halfWidth, halfWidth, halfWidth, ExactRotation( kAxisZ, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );
}

static b3Transform SlideX( b3Fixed x )
{
	return (b3Transform){ { x, B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
}

static b3Transform SlideY( b3Fixed y )
{
	return (b3Transform){ { B3_FIX( 0.0f ), y, B3_FIX( 0.0f ) }, b3Quat_identity };
}

static b3Fixed MinSeparation( const b3LocalManifold* manifold )
{
	b3Fixed minSeparation = B3_FIXED_MAX;
	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		minSeparation = b3FixMin( minSeparation, manifold->points[i].separation );
	}

	return minSeparation;
}

// The edge pair axis is built by intersecting the two Gauss map arcs. Walk the crossed edges from
// speculative contact into deep overlap and check the axis, the separation and the point.
static int CrossedEdgeTest( void )
{
	b3BoxHull hullA, hullB;
	MakeCrossedEdgeHulls( &hullA, &hullB, B3_FIX( 0.5f ) );

	b3Fixed distances[] = { B3_FIX( 1.42f ), kRoot2, B3_FIX( 1.41f ), B3_FIX( 1.3f ) };

	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		b3Fixed d = distances[i];
		b3Fixed expected = d - kRoot2;

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( d ), &cache );

		ENSURE( manifold.pointCount == 1 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( manifold.normal.x - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].separation - expected, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].point.x - b3FixMul( B3_FIX( 0.5f ) , d ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].point.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].point.z, 8 * B3_FIXED_EPSILON );

		// The forced edge query must agree with what the full solver chose
		b3LocalManifold manual = { 0 };
		manual.points = points;
		b3SATCache manualCache = { .type = b3_manualEdgePairAxis };
		b3CollideHulls( &manual, 8, &hullA.base, &hullB.base, SlideX( d ), &manualCache );

		ENSURE( manual.pointCount == 1 );
		ENSURE_SMALL( manual.points[0].separation - expected, 8 * B3_FIXED_EPSILON );
	}

	// Beyond the speculative distance the query reports the axis without building a contact.
	// The axis carries its own orientation now, so a sign error here would read as deep overlap.
	{
		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( B3_FIX( 1.5f ) ), &cache );

		ENSURE( manifold.pointCount == 0 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( cache.separation - ( B3_FIX( 1.5f ) - kRoot2 ), 8 * B3_FIXED_EPSILON );
	}

	return 0;
}

// The parallel edge rejection compares dot products against the edge length, so it is a sine
// threshold and must hold at any size.
static int EdgeAxisScaleTest( void )
{
	b3Fixed scales[] = { B3_FIX( 100.0f ), B3_FIX( 1.0f ), B3_FIX( 0.2f ) };

	for ( int i = 0; i < ARRAY_COUNT( scales ); ++i )
	{
		b3Fixed s = scales[i];
		b3BoxHull hullA, hullB;
		MakeCrossedEdgeHulls( &hullA, &hullB, b3FixMul( B3_FIX( 0.5f ) , s ) );

		b3Fixed expected = -B3_FIX( 0.002f );
		b3Fixed d = b3FixMul( s , kRoot2 ) + expected;

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( d ), &cache );

		// Differencing coordinates of magnitude d costs precision proportional to the scale
		b3Fixed tolerance = b3FixMul( ( 8 * B3_FIXED_EPSILON ), s ) + 48 * B3_FIXED_EPSILON;

		ENSURE( manifold.pointCount == 1 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( manifold.normal.x - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].separation - expected, tolerance );
		ENSURE_SMALL( manifold.points[0].point.x - b3FixMul( B3_FIX( 0.5f ) , d ), tolerance );
		ENSURE_SMALL( manifold.points[0].point.y, tolerance );
		ENSURE_SMALL( manifold.points[0].point.z, tolerance );
	}

	return 0;
}

// Large crossed boxes exercise edge-axis rejection on both sides of the
// int64 dot bound. A clear gap isolates the separating-axis result from
// contact reconstruction, and the separating edge has an analytic answer.
static int EdgeAxisLargeScaleTest( void )
{
	b3Fixed scales[] = { B3_FIX( 8192.0f ), B3_FIX( 16384.0f ), B3_FIX( 32768.0f ) };
	for ( int i = 0; i < ARRAY_COUNT( scales ); ++i )
	{
		b3Fixed s = scales[i];
		b3BoxHull hullA, hullB;
		MakeCrossedEdgeHulls( &hullA, &hullB, b3FixMul( B3_FIX( 0.5f ), s ) );
		b3Fixed expected = B3_FIX( 1.0f );
		b3Fixed d = b3FixMul( s, kRoot2 ) + expected;
		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( d ), &cache );
		ENSURE( manifold.pointCount == 0 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( cache.separation - expected, b3FixMul( 8 * B3_FIXED_EPSILON, s ) );
	}
	return 0;
}

// The cached edge pair rebuilds the axis without a fresh query. An untouched cache proves the
// cached branch answered rather than falling through to the full SAT.
static int EdgeCacheTest( void )
{
	b3BoxHull hullA, hullB;
	MakeCrossedEdgeHulls( &hullA, &hullB, B3_FIX( 0.5f ) );

	b3LocalManifoldPoint points[8];
	b3LocalManifold manifold = { 0 };
	manifold.points = points;
	b3SATCache cache = { 0 };

	b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( B3_FIX( 1.41f ) ), &cache );
	ENSURE( manifold.pointCount == 1 );
	ENSURE( cache.type == b3_edgePairAxis );
	// Cached edges are the even half of each twin pair
	ENSURE( ( cache.indexA & 1 ) == 0 && cache.indexA < hullA.base.edgeCount );
	ENSURE( ( cache.indexB & 1 ) == 0 && cache.indexB < hullB.base.edgeCount );

	b3Fixed seededSeparation = cache.separation;
	ENSURE_SMALL( seededSeparation - ( B3_FIX( 1.41f ) - kRoot2 ), 8 * B3_FIXED_EPSILON );

	// Small motion, the cached features still describe the contact
	b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( B3_FIX( 1.4105f ) ), &cache );
	ENSURE( manifold.pointCount == 1 );
	ENSURE( cache.separation == seededSeparation );
	ENSURE_SMALL( manifold.points[0].separation - ( B3_FIX( 1.4105f ) - kRoot2 ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( manifold.normal.x - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );

	// Jump past the speculative distance. The cached axis alone must report the separation.
	b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, SlideX( B3_FIX( 1.5f ) ), &cache );
	ENSURE( manifold.pointCount == 0 );
	ENSURE( cache.separation == seededSeparation );

	return 0;
}

// Sliding B along the direction of edge A walks the closest point off the end of the segment.
static int EdgeEndpointTest( void )
{
	b3BoxHull hullA, hullB;
	MakeCrossedEdgeHulls( &hullA, &hullB, B3_FIX( 0.5f ) );

	b3Fixed d = B3_FIX( 1.41f );
	b3Fixed expected = d - kRoot2;

	// Just inside the end of edge A
	{
		b3Transform transform = { { d, B3_FIX( 0.49f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { .type = b3_manualEdgePairAxis };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount == 1 );
		ENSURE_SMALL( manifold.points[0].separation - expected, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].point.y - B3_FIX( 0.49f ), 8 * B3_FIXED_EPSILON );
	}

	// Off the end. The edge pair no longer describes a contact, so the builder rejects it and
	// clears the cache rather than clamping to a point that is not on the hulls.
	{
		b3Transform transform = { { d, B3_FIX( 0.55f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { .type = b3_manualEdgePairAxis };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount == 0 );
		ENSURE( cache.type == b3_invalidAxis );

		// The true gap is a vertex to edge distance well past the speculative distance
		b3SATCache freshCache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &freshCache );
		ENSURE( manifold.pointCount == 0 );
		ENSURE( freshCache.separation > B3_FIX( 0.0f ) );
	}

	return 0;
}

static const b3Vec3 kTiltAxes[] = {
	{ B3_FIX( 0.57735027f ), B3_FIX( 0.57735027f ), B3_FIX( 0.57735027f ) },
	{ B3_FIX( 0.70710678f ), B3_FIX( 0.0f ), B3_FIX( 0.70710678f ) },
	{ B3_FIX( 0.26726124f ), B3_FIX( 0.53452248f ), B3_FIX( 0.80178373f ) },
	{ -B3_FIX( 0.48507125f ), B3_FIX( 0.72760688f ), -B3_FIX( 0.48507125f ) },
};

// Angles that straddle the 0.005 rejection threshold
static const b3Fixed kTiltAngles[] = { B3_FIX( 0.0f ), ( 8 * B3_FIXED_EPSILON ), ( 8 * B3_FIXED_EPSILON ), ( 8 * B3_FIXED_EPSILON ), B3_FIX( 1e-4f ), B3_FIX( 1e-3f ), B3_FIX( 0.004f ), B3_FIX( 0.005f ), B3_FIX( 0.006f ), B3_FIX( 0.01f ), B3_FIX( 0.05f ) };

// A cube corner is root3/2 from the center of rotation
static const b3Fixed kHalfDiagonal = B3_FIX( 0.87f );

// Cubes stacked face to face and tipped by a hair. A third of the edge pairs are then nearly
// parallel, the angle between them is at the noise floor and the arc intersection carries no
// information. The face contact has to survive that untouched.
static int ParallelEdgeTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3BoxHull hullB = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	b3Fixed overlap = B3_FIX( 0.01f );

	for ( int i = 0; i < ARRAY_COUNT( kTiltAxes ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( kTiltAngles ); ++j )
		{
			b3Fixed angle = kTiltAngles[j];
			b3Transform transform = { { B3_FIX( 0.0f ), B3_FIX( 1.0f ) - overlap, B3_FIX( 0.0f ) }, ExactQuat( kTiltAxes[i], angle ) };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { 0 };
			b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

			ENSURE_FACE_POINTS( manifold.pointCount );
			ENSURE( cache.type == b3_faceAxisA || cache.type == b3_faceAxisB );
			ENSURE( b3Dot( manifold.normal, kAxisY ) > B3_FIX( 0.998f ) );

			// The tilt can only lift or sink a face point by the length of the arc it sweeps
			b3Fixed bound = b3FixMul( kHalfDiagonal , angle ) + ( 8 * B3_FIXED_EPSILON );
			for ( int k = 0; k < manifold.pointCount; ++k )
			{
				ENSURE_SMALL( manifold.points[k].separation + overlap, bound );
			}
		}
	}

	return 0;
}

// Same stack, but force the edge query to answer. With the edges exactly parallel no pair forms a
// Minkowski face at all, and once a pair does form its separation can never be positive because
// the hulls overlap.
static int ParallelEdgeManualTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3BoxHull hullB = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	b3Fixed overlap = B3_FIX( 0.01f );

	for ( int i = 0; i < ARRAY_COUNT( kTiltAxes ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( kTiltAngles ); ++j )
		{
			b3Fixed angle = kTiltAngles[j];
			b3Transform transform = { { B3_FIX( 0.0f ), B3_FIX( 1.0f ) - overlap, B3_FIX( 0.0f ) }, ExactQuat( kTiltAxes[i], angle ) };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { .type = b3_manualEdgePairAxis };
			b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

			if ( angle == B3_FIX( 0.0f ) )
			{
				// Every pair is parallel so the query finds nothing and leaves the cache alone
				ENSURE( manifold.pointCount == 0 );
				ENSURE( cache.type == b3_manualEdgePairAxis );
				continue;
			}

			// The closest points can fall off the ends of the segments
			if ( manifold.pointCount == 0 )
			{
				continue;
			}

			ENSURE( manifold.pointCount == 1 );
			ENSURE( b3Dot( manifold.normal, kAxisY ) > B3_FIX( 0.99f ) );

			b3Fixed separation = manifold.points[0].separation;
			ENSURE( separation <= B3_FIX( 0.0f ) );
			ENSURE( separation >= -overlap - b3FixMul( kHalfDiagonal , angle ) - B3_FIX( 1e-4f ) );
		}
	}

	return 0;
}

static uint32_t g_seed = 12345;

static b3Fixed NextFloat( b3Fixed lower, b3Fixed upper )
{
	g_seed = 1664525u * g_seed + 1013904223u;
	b3Fixed t = b3FixDiv( (b3Fixed)b3FixFromInt( ( g_seed >> 8 ) ) , (b3Fixed)b3FixFromInt( ( 1 << 24 ) ) );
	return lower + b3FixMul( t , ( upper - lower ) );
}

static b3Vec3 NextDirection( void )
{
	b3Vec3 v = { NextFloat( -B3_FIX( 1.0f ), B3_FIX( 1.0f ) ), NextFloat( -B3_FIX( 1.0f ), B3_FIX( 1.0f ) ), NextFloat( -B3_FIX( 1.0f ), B3_FIX( 1.0f ) ) };
	return b3Normalize( v );
}

// Overlapping hulls admit no separating axis, so an edge separation that comes back positive is
// always noise. It shows up as a manifold with no points, which the solver reads as no contact.
static int OverlapNeverEmptyTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3BoxHull hullB = b3MakeBoxHull( B3_FIX( 0.4f ), B3_FIX( 0.6f ), B3_FIX( 0.5f ) );

	for ( int i = 0; i < 2000; ++i )
	{
		b3Vec3 axis = NextDirection();

		// Half the samples are nearly aligned, where the edge cross products are smallest
		b3Fixed angle = ( i & 1 ) ? NextFloat( -B3_FIX( 0.01f ), B3_FIX( 0.01f ) ) : NextFloat( -B3_PI, B3_PI );

		// Shorter than the smallest half width, so the center of B is inside A
		b3Vec3 offset = b3MulSV( B3_FIX( 0.4f ), NextDirection() );

		b3Transform transform = { offset, ExactQuat( axis, angle ) };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		ENSURE( manifold.pointCount > 0 );
		ENSURE_SMALL( b3Length( manifold.normal ) - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );

		// Clipping keeps points that are separated, but the deepest one must penetrate
		ENSURE( MinSeparation( &manifold ) < B3_FIX( 0.0f ) );
	}

	return 0;
}

// A cube pitched 45 degrees rests on an edge along x at y = -h*root2. The two faces meeting there
// have normals (0,-r,r) and (0,-r,-r), so the arc between them spans the whole lower quadrant.
// A triangle edge crossing under it at an angle picks out an interior point of that arc, which is
// where a wrong lerp parameter would show up.
static int TriangleEdgeTest( void )
{
	b3Fixed beta = b3FixDiv( b3FixMul( B3_FIX( 20.0f ) , B3_PI ) , B3_FIX( 180.0f ) );
	b3Fixed gamma = b3FixDiv( b3FixMul( B3_FIX( 30.0f ) , B3_PI ) , B3_FIX( 180.0f ) );

	b3BoxHull hull = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ), ExactRotation( kAxisX, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );

	// Tipping the triangle plane about z keeps its normal off the hull edge, which the Minkowski
	// test needs. Tipping the edge within that plane moves the arc intersection off the midpoint.
	b3Vec3 triNormal = { b3Sin( beta ), b3Cos( beta ), B3_FIX( 0.0f ) };
	b3Vec3 triEdge = { b3FixMul( b3Sin( gamma ) , b3Cos( beta ) ), b3FixMul( -b3Sin( gamma ) , b3Sin( beta ) ), b3Cos( gamma ) };

	// Perpendicular to both edges and pointing out of the hull
	b3Vec3 axis = b3Normalize( b3Cross( kAxisX, triEdge ) );
	b3Vec3 hullPoint = { B3_FIX( 0.0f ), -kHalfRoot2, B3_FIX( 0.0f ) };

	b3Fixed gaps[] = { B3_FIX( 0.03f ), B3_FIX( 0.01f ), B3_FIX( 0.0f ), -B3_FIX( 0.01f ), -B3_FIX( 0.1f ) };

	for ( int i = 0; i < ARRAY_COUNT( gaps ); ++i )
	{
		b3Fixed gap = gaps[i];
		b3Vec3 trianglePoint = b3MulAdd( hullPoint, gap, axis );

		b3Vec3 v1 = b3MulAdd( trianglePoint, -B3_FIX( 1.0f ), triEdge );
		b3Vec3 v2 = b3MulAdd( trianglePoint, B3_FIX( 1.0f ), triEdge );
		b3Vec3 v3 = b3MulAdd( v1, B3_FIX( 1.5f ), b3Cross( triNormal, triEdge ) );

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { .type = b3_manualEdgePairAxis };
		b3CollideTriangleAndHull( &manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true );

		b3Vec3 expectedNormal = b3Neg( axis );
		b3Vec3 expectedPoint = b3MulAdd( hullPoint, b3FixMul( B3_FIX( 0.5f ) , gap ), axis );

		ENSURE( manifold.pointCount == 1 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( manifold.normal.x - expectedNormal.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.y - expectedNormal.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.z - expectedNormal.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].separation - gap, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].point.x - expectedPoint.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].point.y - expectedPoint.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.points[0].point.z - expectedPoint.z, 8 * B3_FIXED_EPSILON );
	}

	// The tipped triangle plane buries a corner of the hull, so neither face axis separates and
	// the edge axis has to carry the speculative cull on its own.
	b3Fixed culled[] = { B3_FIX( 0.03f ), B3_FIX( 0.05f ) };

	for ( int i = 0; i < ARRAY_COUNT( culled ); ++i )
	{
		b3Fixed gap = culled[i];
		b3Vec3 trianglePoint = b3MulAdd( hullPoint, gap, axis );

		b3Vec3 v1 = b3MulAdd( trianglePoint, -B3_FIX( 1.0f ), triEdge );
		b3Vec3 v2 = b3MulAdd( trianglePoint, B3_FIX( 1.0f ), triEdge );
		b3Vec3 v3 = b3MulAdd( v1, B3_FIX( 1.5f ), b3Cross( triNormal, triEdge ) );

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideTriangleAndHull( &manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true );

		ENSURE( manifold.pointCount == 0 );
		ENSURE( cache.type == b3_edgePairAxis );
		ENSURE_SMALL( cache.separation - gap, 8 * B3_FIXED_EPSILON );
	}

	return 0;
}

// The same cube resting its bottom edge on a triangle whose first edge runs along x. Tipping the
// triangle takes that pair from exactly parallel through the rejection threshold.
static int TriangleParallelEdgeTest( void )
{
	b3BoxHull hull = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ), ExactRotation( kAxisX, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );

	b3Fixed overlap = B3_FIX( 0.01f );
	b3Fixed y = -kHalfRoot2 + overlap;

	for ( int i = 0; i < ARRAY_COUNT( kTiltAxes ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( kTiltAngles ); ++j )
		{
			b3Quat q = ExactQuat( kTiltAxes[i], kTiltAngles[j] );

			b3Vec3 v1 = b3RotateVector( q, ( b3Vec3 ){ -B3_FIX( 2.0f ), y, -B3_FIX( 1.0f ) } );
			b3Vec3 v2 = b3RotateVector( q, ( b3Vec3 ){ B3_FIX( 0.0f ), y, B3_FIX( 2.0f ) } );
			b3Vec3 v3 = b3RotateVector( q, ( b3Vec3 ){ B3_FIX( 2.0f ), y, -B3_FIX( 1.0f ) } );

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { 0 };
			b3CollideTriangleAndHull( &manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true );

			ENSURE_FACE_POINTS( manifold.pointCount );
			ENSURE( cache.type == b3_faceAxisA );
			ENSURE( b3Dot( manifold.normal, kAxisY ) > B3_FIX( 0.99f ) );

			// The tilt can only sink the contact by the length of the arc it sweeps
			b3Fixed bound = b3FixMul( kHalfDiagonal , kTiltAngles[j] ) + ( 8 * B3_FIXED_EPSILON );
			ENSURE_SMALL( MinSeparation( &manifold ) + overlap, bound );
		}
	}

	return 0;
}

// A crossed ridge pair must land on a four point roof face contact. The clipped face
// separation can be no deeper than root2 times the vertical overlap.
static int CheckRoofFaceContact( const b3LocalManifold* manifold, const b3SATCache* cache, b3Fixed overlap )
{
	ENSURE_FACE_POINTS( manifold->pointCount );
	ENSURE( cache->type == b3_faceAxisA || cache->type == b3_faceAxisB );

	// A roof face of one hull, so 45 degrees off the vertical
	ENSURE_SMALL( manifold->normal.y - kHalfRoot2, B3_FIX( 1e-4f ) );

	b3Fixed minSeparation = MinSeparation( manifold );
	ENSURE( minSeparation < b3FixMul( -kHalfRoot2 , overlap ) + B3_FIX( 1e-4f ) );
	ENSURE( minSeparation > b3FixMul( -kRoot2 , overlap ) - B3_FIX( 1e-4f ) );

	return 0;
}

// Two long roof ridges laid across each other. The axis of minimum penetration is the edge
// pair, but a one point edge contact is weak for stacking. The collider builds the roof face
// contact first and only switches to the edge contact when the edge axis beats the clipped
// face separation by more than the slop. This pins all three regimes of that policy.
static int RidgeCrossingTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( B3_FIX( 1.5f ), B3_FIX( 0.1f ), B3_FIX( 0.1f ), ExactRotation( kAxisX, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );
	b3BoxHull hullB = b3MakeTransformedBoxHull( B3_FIX( 1.5f ), B3_FIX( 0.1f ), B3_FIX( 0.1f ), ExactRotation( kAxisX, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );

	b3Fixed ridgeY = b3FixMul( B3_FIX( 0.1f ) , kRoot2 );

	// Shallow overlap. The edge axis is better by only ( root2 - 1 ) * overlap, inside the
	// slop, so the four point face contact carries the crossing at every angle.
	{
		b3Fixed overlap = B3_FIX( 0.01f );
		b3Fixed lift = b3FixMul( B3_FIX( 2.0f ) , ridgeY ) - overlap;
		b3Fixed crossingAngles[] = { B3_FIX( 0.0f ), B3_FIX( 1e-3f ), B3_FIX( 0.02f ), B3_FIX( 0.1f ), B3_FIX( 0.5f ) };

		for ( int i = 0; i < ARRAY_COUNT( crossingAngles ); ++i )
		{
			b3Transform transform = { { B3_FIX( 0.0f ), lift, B3_FIX( 0.0f ) }, ExactQuat( kAxisY, crossingAngles[i] ) };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { 0 };
			b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

			if ( CheckRoofFaceContact( &manifold, &cache, overlap ) != 0 )
			{
				return 1;
			}
		}
	}

	// Deep overlap at a clear crossing. The edge axis now beats the clipped face separation
	// by more than the slop, so the edge contact replaces the face contact.
	{
		b3Fixed overlap = B3_FIX( 0.05f );
		b3Fixed lift = b3FixMul( B3_FIX( 2.0f ) , ridgeY ) - overlap;
		b3Fixed crossingAngles[] = { B3_FIX( 0.05f ), B3_FIX( 0.1f ), B3_FIX( 0.2f ), B3_FIX( 0.5f ) };

		for ( int i = 0; i < ARRAY_COUNT( crossingAngles ); ++i )
		{
			b3Transform transform = { { B3_FIX( 0.0f ), lift, B3_FIX( 0.0f ) }, ExactQuat( kAxisY, crossingAngles[i] ) };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { 0 };
			b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

			ENSURE( manifold.pointCount == 1 );
			ENSURE( cache.type == b3_edgePairAxis );

			// Upstream uses 1e-4 here, which is under this format's 8 ulp floor. The edge
			// normal comes from lerp + normalize on quantized face normals; measured error
			// at the shallowest admitted crossing (0.05 rad) is 12 ulps, so allow 16.
			ENSURE_SMALL( manifold.normal.x, 16 * B3_FIXED_EPSILON );
			ENSURE_SMALL( manifold.normal.y - B3_FIX( 1.0f ), 16 * B3_FIXED_EPSILON );
			ENSURE_SMALL( manifold.normal.z, 16 * B3_FIXED_EPSILON );
			ENSURE_SMALL( manifold.points[0].separation + overlap, B3_FIX( 1e-4f ) );
			ENSURE_SMALL( manifold.points[0].point.y - ( ridgeY - b3FixMul( B3_FIX( 0.5f ) , overlap ) ), B3_FIX( 1e-4f ) );

			// Only has to land near the crossing, not at the end of a three meter beam
			ENSURE_SMALL( manifold.points[0].point.x, B3_FIX( 0.01f ) );
			ENSURE_SMALL( manifold.points[0].point.z, B3_FIX( 0.01f ) );
		}
	}

	// Deep overlap near parallel. A one point edge contact off a parallel pair would have a
	// normal built from noise, so the roof faces keep the contact.
	{
		b3Fixed overlap = B3_FIX( 0.05f );
		b3Fixed lift = b3FixMul( B3_FIX( 2.0f ) , ridgeY ) - overlap;
		b3Fixed shallowAngles[] = { B3_FIX( 0.0f ), B3_FIX( 1e-4f ), B3_FIX( 1e-3f ), B3_FIX( 0.003f ) };

		for ( int i = 0; i < ARRAY_COUNT( shallowAngles ); ++i )
		{
			b3Transform transform = { { B3_FIX( 0.0f ), lift, B3_FIX( 0.0f ) }, ExactQuat( kAxisY, shallowAngles[i] ) };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SATCache cache = { 0 };
			b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

			if ( CheckRoofFaceContact( &manifold, &cache, overlap ) != 0 )
			{
				return 1;
			}
		}
	}

	return 0;
}

// The edge pair axis produced by the arc intersection must match the classic edge cross product.
// Rebuild the axis, the separation and the contact point from the two contributing edges and the
// convex radius, then compare against the manifold. orientRef fixes the sign of the axis to match
// the manifold normal convention for the shape pair. e1 belongs to the shape whose contact point is
// pulled in by the radius (the hull or triangle when it meets a capsule), e2 to the other edge.
static int CheckEdgeContact( const b3LocalManifold* manifold, b3Vec3 p1, b3Vec3 e1, b3Vec3 p2, b3Vec3 e2, b3Vec3 orientRef,
							 b3Fixed radius, b3Fixed normalTol, b3Fixed sepTol, b3Fixed pointTol )
{
	b3Vec3 axis = b3Normalize( b3Cross( e1, e2 ) );
	if ( b3Dot( axis, orientRef ) < B3_FIX( 0.0f ) )
	{
		axis = b3Neg( axis );
	}

	// Normal matches the cross product and is perpendicular to both edges
	ENSURE_SMALL( manifold->normal.x - axis.x, normalTol );
	ENSURE_SMALL( manifold->normal.y - axis.y, normalTol );
	ENSURE_SMALL( manifold->normal.z - axis.z, normalTol );
	ENSURE_SMALL( b3Dot( manifold->normal, b3Normalize( e1 ) ), normalTol );
	ENSURE_SMALL( b3Dot( manifold->normal, b3Normalize( e2 ) ), normalTol );

	b3SegmentDistanceResult closest = b3LineDistance( p1, e1, p2, e2 );

	// Signed gap between the edge lines along the axis, less the capsule radius
	b3Fixed expectedSeparation = b3Dot( axis, b3Sub( closest.point2, closest.point1 ) ) - radius;
	ENSURE_SMALL( manifold->points[0].separation - expectedSeparation, sepTol );

	// Midpoint of the closest approach, pulling the first point in by the radius
	b3Vec3 expectedPoint = b3MulSV( B3_FIX( 0.5f ), b3Add( b3MulSub( closest.point1, radius, axis ), closest.point2 ) );
	ENSURE_SMALL( manifold->points[0].point.x - expectedPoint.x, pointTol );
	ENSURE_SMALL( manifold->points[0].point.y - expectedPoint.y, pointTol );
	ENSURE_SMALL( manifold->points[0].point.z - expectedPoint.z, pointTol );

	return 0;
}

static void HullEdgeSegment( const b3HullData* hull, int edgeIndex, b3Transform transform, b3Vec3* point, b3Vec3* edge )
{
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3HullHalfEdge* e = edges + edgeIndex;
	b3Vec3 tail = b3TransformPoint( transform, points[e->origin] );
	b3Vec3 head = b3TransformPoint( transform, points[edges[e->twin].origin] );
	*point = tail;
	*edge = b3Sub( head, tail );
}

// Two boxes crossing edge to edge. A holds a vertical edge, B is rolled to present a crossing edge
// and yawed so the arc intersection walks off the midpoint. For every configuration that resolves
// to an edge pair the recovered axis, separation and point must match the cross product oracle to
// tight tolerance. The oracle reads the edges the solver actually latched onto, so the check is
// exact regardless of which pair wins.
static int EdgeAxisOracleTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ), ExactRotation( kAxisY, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );

	b3Fixed rolls[] = { b3FixMul( B3_FIX( 0.18f ) , B3_PI ), b3FixMul( B3_FIX( 0.25f ) , B3_PI ), b3FixMul( B3_FIX( 0.32f ) , B3_PI ) };
	b3Fixed yaws[] = { -B3_FIX( 0.35f ), -B3_FIX( 0.15f ), B3_FIX( 0.0f ), B3_FIX( 0.15f ), B3_FIX( 0.35f ) };
	b3Fixed distances[] = { B3_FIX( 1.38f ), B3_FIX( 1.40f ), kRoot2, B3_FIX( 1.44f ) };

	int edgeContacts = 0;

	for ( int i = 0; i < ARRAY_COUNT( rolls ); ++i )
	{
		b3BoxHull hullB = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ), ExactRotation( kAxisZ, rolls[i] ) );

		for ( int j = 0; j < ARRAY_COUNT( yaws ); ++j )
		{
			for ( int k = 0; k < ARRAY_COUNT( distances ); ++k )
			{
				b3Transform transform = { { distances[k], B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, ExactQuat( kAxisY, yaws[j] ) };

				b3LocalManifoldPoint points[8];
				b3LocalManifold manifold = { 0 };
				manifold.points = points;
				b3SATCache cache = { 0 };
				b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

				if ( cache.type != b3_edgePairAxis || manifold.pointCount != 1 )
				{
					continue;
				}

				b3Vec3 p1, e1, p2, e2;
				HullEdgeSegment( &hullA.base, cache.indexA, b3Transform_identity, &p1, &e1 );
				HullEdgeSegment( &hullB.base, cache.indexB, transform, &p2, &e2 );

				b3Vec3 centerA = hullA.base.center;
				b3Vec3 centerB = b3TransformPoint( transform, hullB.base.center );
				b3Vec3 orientRef = b3Sub( centerB, centerA );

				if ( CheckEdgeContact( &manifold, p1, e1, p2, e2, orientRef, B3_FIX( 0.0f ), B3_FIX( 2e-4f ), B3_FIX( 2e-4f ), B3_FIX( 2e-3f ) ) != 0 )
				{
					return 1;
				}

				++edgeContacts;
			}
		}
	}

	// The sweep is only meaningful if it actually drove the edge path
	ENSURE( edgeContacts >= 15 );

	return 0;
}

// The same oracle over randomly oriented box pairs. Whenever the solver reports an edge pair the
// recovered axis must be perpendicular to both edges and match the cross product. This casts a wide
// net over the arc that the structured sweep cannot reach.
static int EdgeAxisRandomOracleTest( void )
{
	g_seed = 246813579u;

	int edgeContacts = 0;

	for ( int i = 0; i < 2000; ++i )
	{
		b3Fixed angleA = b3FixMul( NextFloat( B3_FIX( 0.2f ), B3_FIX( 0.5f ) ) , B3_PI );
		b3Fixed angleB = b3FixMul( NextFloat( B3_FIX( 0.2f ), B3_FIX( 0.5f ) ) , B3_PI );
		b3BoxHull hullA = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ), ExactRotation( NextDirection(), angleA ) );
		b3BoxHull hullB = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

		b3Fixed d = NextFloat( B3_FIX( 1.2f ), B3_FIX( 1.55f ) );
		b3Transform transform = { b3MulSV( d, NextDirection() ), ExactQuat( NextDirection(), angleB ) };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 8, &hullA.base, &hullB.base, transform, &cache );

		if ( cache.type != b3_edgePairAxis || manifold.pointCount != 1 )
		{
			continue;
		}

		b3Vec3 p1, e1, p2, e2;
		HullEdgeSegment( &hullA.base, cache.indexA, b3Transform_identity, &p1, &e1 );
		HullEdgeSegment( &hullB.base, cache.indexB, transform, &p2, &e2 );

		// Skip crossings near parallel where the closest point solve is ill conditioned. The
		// parallel rejection itself is covered by ParallelEdgeTest.
		b3Fixed sine = b3Length( b3Cross( b3Normalize( e1 ), b3Normalize( e2 ) ) );
		if ( sine < B3_FIX( 0.1f ) )
		{
			continue;
		}

		b3Vec3 orientRef = b3Sub( b3TransformPoint( transform, hullB.base.center ), hullA.base.center );

		if ( CheckEdgeContact( &manifold, p1, e1, p2, e2, orientRef, B3_FIX( 0.0f ), B3_FIX( 1e-3f ), B3_FIX( 1e-3f ), B3_FIX( 5e-3f ) ) != 0 )
		{
			return 1;
		}

		++edgeContacts;
	}

	ENSURE( edgeContacts >= 100 );

	return 0;
}

// A thin capsule stabbed through the +x +y edge of a box so the edge pair is the axis of minimum
// penetration. This drives the isolated edge axis (arc versus circle on the Gauss map) that a
// capsule presents. The edge is nearly parallel to a box face normal, exactly where the old center
// based orientation flickered, so the axis, the penetration and the point are all checked.
static int HullCapsuleEdgeDeepTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	// The +x +y edge runs along z between the +x and +y faces
	b3Vec3 edgePoint = { B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) };
	b3Vec3 edgeDir = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
	b3Vec3 outward = b3Normalize( ( b3Vec3 ){ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );

	// Penetrate far enough that the core segment clearly overlaps the box so the deep path runs,
	// but keep the radius small enough that the edge stays the axis of minimum penetration.
	b3Fixed depths[] = { B3_FIX( 0.12f ), B3_FIX( 0.18f ), B3_FIX( 0.25f ) };
	b3Fixed radii[] = { B3_FIX( 0.05f ), B3_FIX( 0.1f ), B3_FIX( 0.2f ) };
	b3Fixed tilts[] = { B3_FIX( 0.0f ), B3_FIX( 0.25f ), -B3_FIX( 0.25f ) };

	int count = 0;

	for ( int i = 0; i < ARRAY_COUNT( depths ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( radii ); ++j )
		{
			for ( int k = 0; k < ARRAY_COUNT( tilts ); ++k )
			{
				b3Vec3 capsuleDir = b3Normalize( ( b3Vec3 ){ B3_FIX( 1.0f ), -B3_FIX( 1.0f ), tilts[k] } );
				b3Vec3 mid = b3MulAdd( edgePoint, -depths[i], outward );
				b3Vec3 c1 = b3MulAdd( mid, -B3_FIX( 0.5f ), capsuleDir );
				b3Vec3 c2 = b3MulAdd( mid, B3_FIX( 0.5f ), capsuleDir );
				b3Capsule capsule = { c1, c2, radii[j] };

				b3LocalManifoldPoint points[8];
				b3LocalManifold manifold = { 0 };
				manifold.points = points;
				b3SimplexCache cache = { 0 };
				b3CollideHullAndCapsule( &manifold, 8, &hull.base, &capsule, b3Transform_identity, &cache );

				ENSURE( manifold.pointCount == 1 );
				ENSURE( manifold.points[0].separation < B3_FIX( 0.0f ) );

				// Hull edge is e1, capsule axis is e2, normal points out of the hull
				if ( CheckEdgeContact( &manifold, edgePoint, edgeDir, c1, b3Sub( c2, c1 ), outward, radii[j], B3_FIX( 1e-4f ), B3_FIX( 1e-4f ),
									   B3_FIX( 1e-4f ) ) != 0 )
				{
					return 1;
				}

				++count;
			}
		}
	}

	ENSURE( count == ARRAY_COUNT( depths ) * ARRAY_COUNT( radii ) * ARRAY_COUNT( tilts ) );

	return 0;
}

// Force the triangle versus hull edge query over a broad sweep of crossing geometries. A cube tipped
// 45 degrees rests an edge along x at y = -h*root2. A triangle edge is laid across it at a range of
// yaws, plane tips and gaps so the arc intersection lands all over the arc. The manual axis hands
// the winning pair to the builder, and the recovered axis must match the cross product of the chosen
// triangle and hull edges and point from the triangle into the hull.
static int TriangleHullEdgeSweepTest( void )
{
	b3BoxHull hull = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ), ExactRotation( kAxisX, b3FixMul( B3_FIX( 0.25f ) , B3_PI ) ) );
	b3Vec3 hullEdgePoint = { B3_FIX( 0.0f ), -kHalfRoot2, B3_FIX( 0.0f ) };

	// Degrees: triangle plane tip about z, and triangle edge yaw
	b3Fixed betas[] = { B3_FIX( 8.0f ), B3_FIX( 20.0f ), B3_FIX( 32.0f ) };
	b3Fixed gammas[] = { B3_FIX( 20.0f ), B3_FIX( 35.0f ), B3_FIX( 50.0f ), B3_FIX( 70.0f ) };
	b3Fixed gaps[] = { B3_FIX( 0.02f ), B3_FIX( 0.0f ), -B3_FIX( 0.03f ), -B3_FIX( 0.08f ) };

	int edgeContacts = 0;

	for ( int a = 0; a < ARRAY_COUNT( betas ); ++a )
	{
		for ( int b = 0; b < ARRAY_COUNT( gammas ); ++b )
		{
			for ( int c = 0; c < ARRAY_COUNT( gaps ); ++c )
			{
				b3Fixed beta = b3FixDiv( b3FixMul( betas[a] , B3_PI ) , B3_FIX( 180.0f ) );
				b3Fixed gamma = b3FixDiv( b3FixMul( gammas[b] , B3_PI ) , B3_FIX( 180.0f ) );

				// Tip the plane off the hull edge so the Minkowski test holds, then yaw the edge
				b3Vec3 triNormal = { b3Sin( beta ), b3Cos( beta ), B3_FIX( 0.0f ) };
				b3Vec3 triEdge = { b3FixMul( b3Sin( gamma ) , b3Cos( beta ) ), b3FixMul( -b3Sin( gamma ) , b3Sin( beta ) ), b3Cos( gamma ) };

				// Perpendicular to both edges and pointing out of the hull
				b3Vec3 axis = b3Normalize( b3Cross( kAxisX, triEdge ) );
				b3Vec3 trianglePoint = b3MulAdd( hullEdgePoint, gaps[c], axis );

				b3Vec3 v1 = b3MulAdd( trianglePoint, -B3_FIX( 1.0f ), triEdge );
				b3Vec3 v2 = b3MulAdd( trianglePoint, B3_FIX( 1.0f ), triEdge );
				b3Vec3 v3 = b3MulAdd( v1, B3_FIX( 1.5f ), b3Cross( triNormal, triEdge ) );

				b3Vec3 triangleVerts[] = { v1, v2, v3 };
				b3Vec3 triangleEdges[] = { b3Sub( v2, v1 ), b3Sub( v3, v2 ), b3Sub( v1, v3 ) };
				b3Vec3 triangleCenter = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), b3Add( v1, b3Add( v2, v3 ) ) );

				b3LocalManifoldPoint points[8];
				b3LocalManifold manifold = { 0 };
				manifold.points = points;
				b3SATCache cache = { .type = b3_manualEdgePairAxis };
				b3CollideTriangleAndHull( &manifold, 8, v1, v2, v3, 0, &hull.base, &cache, true );

				if ( cache.type != b3_edgePairAxis || manifold.pointCount != 1 )
				{
					continue;
				}

				b3Vec3 p1 = triangleVerts[cache.indexA];
				b3Vec3 e1 = triangleEdges[cache.indexA];

				b3Vec3 p2, e2;
				HullEdgeSegment( &hull.base, cache.indexB, b3Transform_identity, &p2, &e2 );

				// Normal points from the triangle into the hull
				b3Vec3 orientRef = b3Sub( hull.base.center, triangleCenter );

				if ( CheckEdgeContact( &manifold, p1, e1, p2, e2, orientRef, B3_FIX( 0.0f ), B3_FIX( 1e-4f ), B3_FIX( 1e-4f ), B3_FIX( 1e-3f ) ) != 0 )
				{
					return 1;
				}

				++edgeContacts;
			}
		}
	}

	ENSURE( edgeContacts >= 30 );

	return 0;
}

// A capsule laid nearly in a triangle plane and pushed across one edge so the edge pair drives the
// deep contact. This exercises the two sided triangle edge, where the side normal trick chooses
// which half of the arc holds the axis. Validate every edge contact the sweep produces.
static int CapsuleTriangleEdgeDeepTest( void )
{
	// Triangle in the y = 0 plane. The v1 v2 edge runs along x at z = 0, the interior lies at z < 0.
	b3Vec3 v1 = { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3Vec3 v2 = { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3Vec3 v3 = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) };
	b3Vec3 triangle[] = { v1, v2, v3 };
	b3Vec3 triangleEdges[] = { b3Sub( v2, v1 ), b3Sub( v3, v2 ), b3Sub( v1, v3 ) };
	b3Vec3 triangleCenter = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), b3Add( v1, b3Add( v2, v3 ) ) );

	// A nearly in plane core crossing the edge at (0,0,z0) with a small out of plane tilt. The core
	// pierces the triangle just inside the edge so the deep path runs and the tilted edge pair wins.
	b3Fixed z0s[] = { -B3_FIX( 0.05f ), -B3_FIX( 0.03f ), -B3_FIX( 0.01f ) };
	b3Fixed tilts[] = { B3_FIX( 0.2f ), B3_FIX( 0.3f ), B3_FIX( 0.4f ) };
	b3Fixed yaws[] = { B3_FIX( 0.4f ), B3_FIX( 0.6f ), B3_FIX( 0.8f ) };
	b3Fixed radii[] = { B3_FIX( 0.05f ), B3_FIX( 0.1f ) };

	int edgeContacts = 0;

	for ( int i = 0; i < ARRAY_COUNT( z0s ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( tilts ); ++j )
		{
			for ( int y = 0; y < ARRAY_COUNT( yaws ); ++y )
			{
				for ( int r = 0; r < ARRAY_COUNT( radii ); ++r )
				{
					b3Vec3 capsuleDir = b3Normalize( ( b3Vec3 ){ b3Sin( yaws[y] ), tilts[j], b3Cos( yaws[y] ) } );
					b3Vec3 mid = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), z0s[i] };
					b3Vec3 c1 = b3MulAdd( mid, -B3_FIX( 0.6f ), capsuleDir );
					b3Vec3 c2 = b3MulAdd( mid, B3_FIX( 0.6f ), capsuleDir );
					b3Capsule capsule = { c1, c2, radii[r] };

					b3LocalManifoldPoint points[8];
					b3LocalManifold manifold = { 0 };
					manifold.points = points;
					b3SimplexCache cache = { 0 };
					b3CollideTriangleAndCapsule( &manifold, 8, triangle, &capsule, &cache );

					// Only the edge contacts exercise the new axis. Face contacts are handled elsewhere.
					if ( manifold.pointCount != 1 || manifold.feature < b3_featureEdge1 || manifold.feature > b3_featureEdge3 )
					{
						continue;
					}

					int edgeIndex = manifold.feature - b3_featureEdge1;
					b3Vec3 p1 = triangle[edgeIndex];
					b3Vec3 e1 = triangleEdges[edgeIndex];

					// Normal points from the triangle toward the capsule
					b3Vec3 capsuleEdge = b3Sub( c2, c1 );
					b3Vec3 capsuleCenter = b3Lerp( c1, c2, B3_FIX( 0.5f ) );
					b3Vec3 orientRef = b3Sub( capsuleCenter, triangleCenter );

					if ( CheckEdgeContact( &manifold, p1, e1, c1, capsuleEdge, orientRef, radii[r], B3_FIX( 1e-4f ), B3_FIX( 1e-4f ), B3_FIX( 2e-4f ) ) != 0 )
					{
						return 1;
					}

					++edgeContacts;
				}
			}
		}
	}

	// The sweep must actually reach the edge path
	ENSURE( edgeContacts >= 15 );

	return 0;
}

// A sphere driven straight through a box face, from separated, across the surface where the collider
// switches from GJK closest points to the SAT face pick, and on into deep overlap. The separation
// must stay the analytic gap the whole way and the normal must not flip. A jump at the seam would
// read as a pop in the solver. The sweep is fine enough to land samples on both sides of the seam.
static int SphereHullSeamTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3Fixed radius = B3_FIX( 0.15f );

	b3Fixed yStart = B3_FIX( 0.5f ) + radius + b3FixMul( B3_FIX( 0.4f ) , B3_SPECULATIVE_DISTANCE );
	b3Fixed yEnd = B3_FIX( 0.1f );
	int steps = 400;
	b3Fixed dy = b3FixDiv( ( yStart - yEnd ) , b3FixFromInt( steps ) );

	b3Fixed previous = B3_FIX( 0.0f );
	int shallowSamples = 0;
	int deepSamples = 0;

	for ( int i = 0; i <= steps; ++i )
	{
		b3Fixed y = yStart - b3FixMul( b3FixFromInt( i ) , dy );
		b3Sphere sphere = { { B3_FIX( 0.0f ), y, B3_FIX( 0.0f ) }, radius };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndSphere( &manifold, 8, &hull.base, &sphere, b3Transform_identity, &cache );

		ENSURE( manifold.pointCount == 1 );

		b3Fixed separation = manifold.points[0].separation;
		b3Fixed expected = ( y - B3_FIX( 0.5f ) ) - radius;

		// Separation is the analytic gap on both sides of the seam
		ENSURE_SMALL( separation - expected, 8 * B3_FIXED_EPSILON );

		// Normal holds the face direction with no flip
		ENSURE_SMALL( manifold.normal.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.y - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.z, 8 * B3_FIXED_EPSILON );

		// No jump across the seam: consecutive separations track the step
		if ( i > 0 )
		{
			ENSURE_SMALL( ( previous - separation ) - dy, 8 * B3_FIXED_EPSILON );
		}
		previous = separation;

		if ( y > B3_FIX( 0.5f ) )
		{
			shallowSamples += 1;
		}
		else
		{
			deepSamples += 1;
		}
	}

	// The sweep must straddle the surface so both the GJK and the SAT branch run
	ENSURE( shallowSamples > 0 && deepSamples > 0 );

	return 0;
}

// The same seam for a capsule laid parallel to the face. Above the surface the shallow path clips two
// points, in overlap the face path builds two, and every point must sit at the analytic gap through
// the transition with the normal fixed on the face.
static int CapsuleHullSeamTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3Fixed radius = B3_FIX( 0.15f );
	b3Fixed halfLength = B3_FIX( 0.3f );

	b3Fixed yStart = B3_FIX( 0.5f ) + radius + b3FixMul( B3_FIX( 0.4f ) , B3_SPECULATIVE_DISTANCE );
	b3Fixed yEnd = B3_FIX( 0.1f );
	int steps = 400;
	b3Fixed dy = b3FixDiv( ( yStart - yEnd ) , b3FixFromInt( steps ) );

	b3Fixed previous = B3_FIX( 0.0f );
	int shallowSamples = 0;
	int deepSamples = 0;

	for ( int i = 0; i <= steps; ++i )
	{
		b3Fixed y = yStart - b3FixMul( b3FixFromInt( i ) , dy );
		b3Capsule capsule = { { -halfLength, y, B3_FIX( 0.0f ) }, { halfLength, y, B3_FIX( 0.0f ) }, radius };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndCapsule( &manifold, 8, &hull.base, &capsule, b3Transform_identity, &cache );

		ENSURE( manifold.pointCount >= 1 );

		b3Fixed expected = ( y - B3_FIX( 0.5f ) ) - radius;

		// Every point sits at the analytic gap
		for ( int k = 0; k < manifold.pointCount; ++k )
		{
			ENSURE_SMALL( manifold.points[k].separation - expected, 8 * B3_FIXED_EPSILON );
		}

		// Normal holds the face direction with no flip
		ENSURE_SMALL( manifold.normal.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.y - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( manifold.normal.z, 8 * B3_FIXED_EPSILON );

		// No jump across the seam
		b3Fixed minSeparation = MinSeparation( &manifold );
		if ( i > 0 )
		{
			ENSURE_SMALL( ( previous - minSeparation ) - dy, 8 * B3_FIXED_EPSILON );
		}
		previous = minSeparation;

		if ( y > B3_FIX( 0.5f ) )
		{
			shallowSamples += 1;
		}
		else
		{
			deepSamples += 1;
		}
	}

	ENSURE( shallowSamples > 0 && deepSamples > 0 );

	return 0;
}

// A capsule core straddling a triangle face inside the interior. The core pierces the plane so the
// deep path runs, and with the tilt kept small the face stays the axis of minimum penetration, so the
// clip must return two points on the triangle face. This is the branch that had no coverage.
static int CapsuleTriangleFaceDeepTest( void )
{
	// Triangle in the y = 0 plane, normal +y, centroid at the origin
	b3Vec3 v1 = { -B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) };
	b3Vec3 v2 = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 4.0f ) };
	b3Vec3 v3 = { B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) };
	b3Vec3 triangle[] = { v1, v2, v3 };

	b3Fixed yaws[] = { B3_FIX( 0.0f ), B3_FIX( 0.6f ), B3_FIX( 1.2f ), B3_FIX( 1.8f ), B3_FIX( 2.4f ) };
	b3Fixed tilts[] = { B3_FIX( 0.06f ), B3_FIX( 0.1f ), B3_FIX( 0.15f ) };
	b3Fixed radii[] = { B3_FIX( 0.05f ), B3_FIX( 0.1f ), B3_FIX( 0.2f ) };

	// Bias the center just above the plane so the back side cull passes while the lower endpoint dips through
	b3Fixed bias = B3_FIX( 0.01f );
	b3Fixed halfLength = B3_FIX( 1.0f );

	// The analytic endpoint heights are built from libm, so the only error in the expected separation
	// is the engine's own quantization. Eight quanta is the tree's floor for a below-resolution tolerance.
	const b3Fixed tol = 8 * B3_FIXED_EPSILON;

	int faceContacts = 0;

	for ( int i = 0; i < ARRAY_COUNT( yaws ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( tilts ); ++j )
		{
			for ( int r = 0; r < ARRAY_COUNT( radii ); ++r )
			{
				// Axis is the in-plane heading tipped up so the segment straddles the plane.
				// libm rather than b3Sin/b3Cos: this is reference math, not simulation.
				double tilt = b3FixToDouble( tilts[j] );
				double yaw = b3FixToDouble( yaws[i] );
				b3Vec3 axis = { b3FixFromDouble( cos( tilt ) * cos( yaw ) ), b3FixFromDouble( sin( tilt ) ),
								b3FixFromDouble( cos( tilt ) * sin( yaw ) ) };
				b3Vec3 center = { B3_FIX( 0.0f ), bias, B3_FIX( 0.0f ) };
				b3Vec3 c1 = b3MulAdd( center, -halfLength, axis );
				b3Vec3 c2 = b3MulAdd( center, halfLength, axis );
				b3Capsule capsule = { c1, c2, radii[r] };

				b3LocalManifoldPoint points[8];
				b3LocalManifold manifold = { 0 };
				manifold.points = points;
				b3SimplexCache cache = { 0 };
				b3CollideTriangleAndCapsule( &manifold, 8, triangle, &capsule, &cache );

				// Two points on the triangle face with the plane normal
				ENSURE( manifold.pointCount == 2 );
				ENSURE( manifold.feature == b3_featureTriangleFace );
				ENSURE_SMALL( manifold.normal.x, tol );
				ENSURE_SMALL( manifold.normal.y - B3_FIX( 1.0f ), tol );
				ENSURE_SMALL( manifold.normal.z, tol );

				// Separations are the endpoint heights pulled in by the radius. The lower endpoint is below
				// the plane, so the deepest separation is negative.
				b3Fixed lower = b3FixMin( c1.y, c2.y ) - radii[r];
				b3Fixed upper = b3FixMax( c1.y, c2.y ) - radii[r];
				b3Fixed minSep = b3FixMin( manifold.points[0].separation, manifold.points[1].separation );
				b3Fixed maxSep = b3FixMax( manifold.points[0].separation, manifold.points[1].separation );
				ENSURE_SMALL( minSep - lower, tol );
				ENSURE_SMALL( maxSep - upper, tol );
				ENSURE( minSep < B3_FIX( 0.0f ) );

				++faceContacts;
			}
		}
	}

	// Every configuration must reach the face path
	ENSURE( faceContacts == ARRAY_COUNT( yaws ) * ARRAY_COUNT( tilts ) * ARRAY_COUNT( radii ) );

	return 0;
}

// A capsule laid flat with its core below a box top face. The core sits inside the box so the deep
// path runs, and across a sweep of depths, headings and radii the face clip must return two points.
static int HullCapsuleFaceDeepTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	b3Fixed depths[] = { B3_FIX( 0.1f ), B3_FIX( 0.2f ), B3_FIX( 0.3f ), B3_FIX( 0.4f ), B3_FIX( 0.45f ) };
	b3Fixed yaws[] = { B3_FIX( 0.0f ), B3_FIX( 0.4f ), B3_FIX( 0.8f ), B3_FIX( 1.2f ) };
	b3Fixed radii[] = { B3_FIX( 0.1f ), B3_FIX( 0.15f ), B3_FIX( 0.2f ) };
	b3Fixed offsets[] = { -B3_FIX( 0.1f ), B3_FIX( 0.0f ), B3_FIX( 0.1f ) };
	b3Fixed halfLength = B3_FIX( 0.3f );

	const b3Fixed tol = 8 * B3_FIXED_EPSILON;

	int faceContacts = 0;

	for ( int i = 0; i < ARRAY_COUNT( depths ); ++i )
	{
		for ( int j = 0; j < ARRAY_COUNT( yaws ); ++j )
		{
			for ( int r = 0; r < ARRAY_COUNT( radii ); ++r )
			{
				for ( int o = 0; o < ARRAY_COUNT( offsets ); ++o )
				{
					b3Fixed y = depths[i];
					double yaw = b3FixToDouble( yaws[j] );
					b3Vec3 dir = { b3FixFromDouble( cos( yaw ) ), B3_FIX( 0.0f ), b3FixFromDouble( sin( yaw ) ) };
					b3Vec3 center = { offsets[o], y, B3_FIX( 0.0f ) };
					b3Vec3 c1 = b3MulAdd( center, -halfLength, dir );
					b3Vec3 c2 = b3MulAdd( center, halfLength, dir );
					b3Capsule capsule = { c1, c2, radii[r] };

					b3LocalManifoldPoint points[8];
					b3LocalManifold manifold = { 0 };
					manifold.points = points;
					b3SimplexCache cache = { 0 };
					b3CollideHullAndCapsule( &manifold, 8, &hull.base, &capsule, b3Transform_identity, &cache );

					// Two points on the top face. The hull path does not tag a feature, so the face is
					// identified by the normal and the point count.
					ENSURE( manifold.pointCount == 2 );
					ENSURE_SMALL( manifold.normal.x, tol );
					ENSURE_SMALL( manifold.normal.y - B3_FIX( 1.0f ), tol );
					ENSURE_SMALL( manifold.normal.z, tol );

					// Flat capsule, so both points sit at the same analytic gap
					b3Fixed expected = ( y - B3_FIX( 0.5f ) ) - radii[r];
					ENSURE_SMALL( manifold.points[0].separation - expected, tol );
					ENSURE_SMALL( manifold.points[1].separation - expected, tol );
					ENSURE( expected < B3_FIX( 0.0f ) );

					++faceContacts;
				}
			}
		}
	}

	ENSURE( faceContacts == ARRAY_COUNT( depths ) * ARRAY_COUNT( yaws ) * ARRAY_COUNT( radii ) * ARRAY_COUNT( offsets ) );

	return 0;
}

// A broad edge-contact oracle sweep: 120 headings over the full circle against a scalene triangle,
// running CheckEdgeContact on every edge contact that comes back, which rebuilds the normal,
// separation and point from the REPORTED edge index. This widens the three-yaw coverage of
// CapsuleTriangleEdgeDeepTest above.
//
// WHAT IT DOES NOT DO, stated so nobody reads it as the acceptance it is not: it does not
// discriminate the f42be21 edge-walk advance (the `v1 = v2; edgeIndex = index;` in the parallel
// skip). Measured 2026-08-22: that skip requires the capsule core to be simultaneously in the
// triangle plane and parallel to one of its edges, and with any out-of-plane tilt the plane dot
// alone puts the test far above tolerance. Instrumented, the skip fires ZERO times across the
// entire suite; a deliberately constructed in-plane axis-parallel capsule fires it five times and
// STILL produces byte-identical manifolds either way, because a single skip perturbs exactly one
// following iteration and that iteration loses the max-separation comparison. The fix is correct
// and defensive; it is not currently reachable from an observable difference in this tree.
static int CapsuleTriangleEdgeWalkTest( void )
{
	// Scalene on purpose: three distinct edge directions, so each parallel yaw is hit separately.
	b3Vec3 v1 = { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3Vec3 v2 = { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3Vec3 v3 = { -B3_FIX( 0.5f ), B3_FIX( 0.0f ), -B3_FIX( 2.5f ) };
	b3Vec3 triangle[] = { v1, v2, v3 };
	b3Vec3 triangleEdges[] = { b3Sub( v2, v1 ), b3Sub( v3, v2 ), b3Sub( v1, v3 ) };
	b3Vec3 triangleCenter = b3MulSV( b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 3.0f ) ), b3Add( v1, b3Add( v2, v3 ) ) );

	const b3Fixed radius = B3_FIX( 0.08f );
	const b3Fixed halfLength = B3_FIX( 0.6f );

	int edgeContacts = 0;
	int parallelYaws = 0;

	// 120 headings over the full circle, so every edge direction is crossed and the yaws either
	// side of each parallel case are exercised too.
	for ( int y = 0; y < 120; ++y )
	{
		double yaw = ( 2.0 * 3.14159265358979323846 * y ) / 120.0;

		// Track whether this heading is parallel to some triangle edge, so the test can prove it
		// actually visited the branch it exists to cover.
		b3Vec3 flat = { b3FixFromDouble( cos( yaw ) ), B3_FIX( 0.0f ), b3FixFromDouble( sin( yaw ) ) };
		for ( int e = 0; e < 3; ++e )
		{
			b3Vec3 dir = b3Normalize( triangleEdges[e] );
			b3Fixed d = b3FixAbs( b3Dot( flat, dir ) );
			if ( d > B3_FIX( 0.9995f ) )
			{
				parallelYaws += 1;
				break;
			}
		}

		for ( int t = 0; t < 3; ++t )
		{
			b3Fixed tilt = B3_FIX( 0.2f ) + t * B3_FIX( 0.1f );
			double tiltD = b3FixToDouble( tilt );
			b3Vec3 axis = { b3FixFromDouble( cos( tiltD ) * cos( yaw ) ), b3FixFromDouble( sin( tiltD ) ),
							b3FixFromDouble( cos( tiltD ) * sin( yaw ) ) };

			// Sit the core just inside the v1 v2 edge so the deep path runs.
			b3Vec3 mid = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 0.03f ) };
			b3Vec3 c1 = b3MulAdd( mid, -halfLength, axis );
			b3Vec3 c2 = b3MulAdd( mid, halfLength, axis );
			b3Capsule capsule = { c1, c2, radius };

			b3LocalManifoldPoint points[8];
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SimplexCache cache = { 0 };
			b3CollideTriangleAndCapsule( &manifold, 8, triangle, &capsule, &cache );

			if ( manifold.pointCount != 1 || manifold.feature < b3_featureEdge1 || manifold.feature > b3_featureEdge3 )
			{
				continue;
			}

			int edgeIndex = manifold.feature - b3_featureEdge1;
			b3Vec3 capsuleEdge = b3Sub( c2, c1 );
			b3Vec3 capsuleCenter = b3Lerp( c1, c2, B3_FIX( 0.5f ) );
			b3Vec3 orientRef = b3Sub( capsuleCenter, triangleCenter );

			// The oracle rebuilds everything from the REPORTED index, which is the thing the
			// missing advance corrupted.
			if ( CheckEdgeContact( &manifold, triangle[edgeIndex], triangleEdges[edgeIndex], c1, capsuleEdge, orientRef,
								   radius, B3_FIX( 1e-4f ), B3_FIX( 1e-4f ), B3_FIX( 2e-4f ) ) != 0 )
			{
				return 1;
			}

			++edgeContacts;
		}
	}

	// Guards against the sweep going vacuous. parallelYaws counts headings ALIGNED with an edge,
	// which is not the same as the skip branch firing — see the note above.
	ENSURE( parallelYaws > 0 );
	ENSURE( edgeContacts > 0 );

	return 0;
}

// A capsule core laid across a box corner so a clip plane (a side plane of the reference
// face) is straddled by a few quanta. b3ClipSegment decides the crossing with the product
// b3FixMul(distance1, distance2) < 0, and a product of two separations under ~0.004 quantizes
// to zero, so a shallow crossing is dropped and the face contact loses the crossing point.
// The sign comparison keeps it. A large crossing (where the product cannot quantize) is the
// control that pins the expected two point face contact.
static int ClipSegmentSmallCrossingTest( void )
{
	b3BoxHull hull = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3Fixed radius = B3_FIX( 0.05f );

	// dx1/dx2 are the two endpoint offsets from x = 0.5 in raw ulps; the core is long in z and
	// pinned just under the top face (y = 0.5 minus a couple of quanta) so the top face is the
	// reference face and x = +0.5 is one of its side planes.
	int offsets[2][2] = {
		{ -5, 3 },		 // crossing by a few quanta: the old product reads zero
		{ -500, 300 }, // the same crossing, large enough that the product cannot quantize
	};

	for ( int i = 0; i < 2; ++i )
	{
		b3Capsule capsule;
		capsule.radius = radius;
		capsule.center1 = ( b3Vec3 ){ B3_FIX( 0.5f ) + offsets[i][0] * B3_FIXED_EPSILON,
									   B3_FIX( 0.5f ) - 2 * B3_FIXED_EPSILON, -B3_FIX( 0.3f ) };
		capsule.center2 = ( b3Vec3 ){ B3_FIX( 0.5f ) + offsets[i][1] * B3_FIXED_EPSILON,
									   B3_FIX( 0.5f ) - 3 * B3_FIXED_EPSILON, B3_FIX( 0.3f ) };

		b3LocalManifoldPoint points[8];
		b3LocalManifold manifold = { 0 };
		manifold.points = points;
		b3SimplexCache cache = { 0 };
		b3CollideHullAndCapsule( &manifold, 8, &hull.base, &capsule, b3Transform_identity, &cache );

		ENSURE( manifold.pointCount == 2 );
	}

	return 0;
}

static int CheckSpeculativePoints( const b3LocalManifold* manifold, b3Fixed gap )
{
	ENSURE( manifold->pointCount > 0 );
	ENSURE_SMALL( MinSeparation( manifold ) - gap, 8 * B3_FIXED_EPSILON );
	return 0;
}

// Contact recycling treats a pair without points as separated by at least the speculative distance.
// Every manifold routine must report a pair inside that distance or recycling can skip real overlap.
// Discussion #175.
static int SpeculativeMarginTest( void )
{
	b3Fixed gaps[] = { B3_FIX( 0.0f ), b3FixMul( B3_FIX( 0.25f ) , B3_SPECULATIVE_DISTANCE ),
						b3FixMul( B3_FIX( 0.75f ) , B3_SPECULATIVE_DISTANCE ) };
	int gapCount = sizeof( gaps ) / sizeof( gaps[0] );

	b3Sphere sphere = { b3Vec3_zero, B3_FIX( 0.25f ) };
	b3Sphere bigSphere = { b3Vec3_zero, B3_FIX( 0.5f ) };
	b3Capsule capsuleX = { { -B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
						   { B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.25f ) };
	b3Capsule capsuleZ = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 0.5f ) },
						   { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.5f ) }, B3_FIX( 0.25f ) };
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	// Normal is +y
	b3Vec3 triangle[3] = { { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) },
						   { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 2.0f ) },
						   { B3_FIX( 2.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) } };

	b3LocalManifoldPoint points[B3_MAX_MANIFOLD_POINTS];
	b3LocalManifold manifold = { 0 };
	manifold.points = points;

	for ( int i = 0; i < gapCount; ++i )
	{
		b3Fixed gap = gaps[i];

		manifold.pointCount = 0;
		b3CollideSpheres( &manifold, B3_MAX_MANIFOLD_POINTS, &bigSphere, &sphere, SlideY( B3_FIX( 0.75f ) + gap ) );
		ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );

		manifold.pointCount = 0;
		b3CollideCapsuleAndSphere( &manifold, B3_MAX_MANIFOLD_POINTS, &capsuleX, &sphere, SlideY( B3_FIX( 0.5f ) + gap ) );
		ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );

		manifold.pointCount = 0;
		b3CollideCapsules( &manifold, B3_MAX_MANIFOLD_POINTS, &capsuleX, &capsuleZ, SlideY( B3_FIX( 0.5f ) + gap ) );
		ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );

		{
			manifold.pointCount = 0;
			b3SimplexCache cache = { 0 };
			b3CollideHullAndSphere( &manifold, B3_MAX_MANIFOLD_POINTS, &box.base, &sphere, SlideY( B3_FIX( 0.75f ) + gap ), &cache );
			ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );
		}

		{
			manifold.pointCount = 0;
			b3SimplexCache cache = { 0 };
			b3CollideHullAndCapsule( &manifold, B3_MAX_MANIFOLD_POINTS, &box.base, &capsuleX, SlideY( B3_FIX( 0.75f ) + gap ), &cache );
			ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );
		}

		{
			manifold.pointCount = 0;
			b3SATCache cache = { 0 };
			b3CollideHulls( &manifold, B3_MAX_MANIFOLD_POINTS, &box.base, &box.base, SlideY( B3_FIX( 1.0f ) + gap ), &cache );
			ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );
		}

		{
			manifold.pointCount = 0;
			b3Sphere raised = { { B3_FIX( 0.0f ), B3_FIX( 0.25f ) + gap, B3_FIX( 0.0f ) }, B3_FIX( 0.25f ) };
			b3CollideTriangleAndSphere( &manifold, B3_MAX_MANIFOLD_POINTS, triangle, &raised );
			ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );
		}

		{
			manifold.pointCount = 0;
			b3Capsule raised = capsuleX;
			raised.center1.y = B3_FIX( 0.25f ) + gap;
			raised.center2.y = B3_FIX( 0.25f ) + gap;
			b3SimplexCache cache = { 0 };
			b3CollideTriangleAndCapsule( &manifold, B3_MAX_MANIFOLD_POINTS, triangle, &raised, &cache );
			ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );
		}

		{
			manifold.pointCount = 0;
			b3BoxHull raised = b3MakeTransformedBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ), SlideY( B3_FIX( 0.5f ) + gap ) );
			b3SATCache cache = { 0 };
			b3CollideTriangleAndHull( &manifold, B3_MAX_MANIFOLD_POINTS, triangle[0], triangle[1], triangle[2], 0, &raised.base,
									  &cache, true );
			ENSURE( CheckSpeculativePoints( &manifold, gap ) == 0 );
		}
	}

	// Beyond the speculative distance the closed form routines report nothing
	b3Fixed farGap = b3FixMul( B3_FIX( 1.5f ) , B3_SPECULATIVE_DISTANCE );

	manifold.pointCount = 0;
	b3CollideSpheres( &manifold, B3_MAX_MANIFOLD_POINTS, &bigSphere, &sphere, SlideY( B3_FIX( 0.75f ) + farGap ) );
	ENSURE( manifold.pointCount == 0 );

	manifold.pointCount = 0;
	b3CollideCapsuleAndSphere( &manifold, B3_MAX_MANIFOLD_POINTS, &capsuleX, &sphere, SlideY( B3_FIX( 0.5f ) + farGap ) );
	ENSURE( manifold.pointCount == 0 );

	return 0;
}

static void FillPoints2D( b3HullPoint2D* pts, const b3Vec2* input, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		pts[i].p = input[i];
		pts[i].separation = B3_FIX( 0.0f );
		pts[i].originalIndex = i;
	}
}

static b3Fixed Hull2DArea( const b3HullPoint2D* hull, int count )
{
	b3Fixed sum = B3_FIX( 0.0f );
	for ( int i = 0; i < count; ++i )
	{
		int next = ( i + 1 ) % count;
		sum += b3FixMul( hull[i].p.x , hull[next].p.y ) - b3FixMul( hull[i].p.y , hull[next].p.x );
	}

	return sum / 2;
}

// The 2D hull reduction (box3d 954cf87) builds a convex hull of the candidate manifold points
// and simplifies it to the target by dropping the vertices that contribute the least area. It is
// only wired into b3ReduceManifoldPoints when B3_MAX_MANIFOLD_POINTS > 4, but the hull and
// simplify routines are always compiled so they can be exercised directly here.
static int Hull2DReductionTest( void )
{
	// Interior points contribute no area, so they must not survive the hull.
	{
		b3Vec2 input[8] = {
			{ B3_FIX( -1.0f ), B3_FIX( -1.0f ) }, { B3_FIX( 1.0f ), B3_FIX( -1.0f ) },
			{ B3_FIX( 1.0f ), B3_FIX( 1.0f ) },	  { B3_FIX( -1.0f ), B3_FIX( 1.0f ) },
			{ B3_FIX( 0.0f ), B3_FIX( 0.0f ) },	  { B3_FIX( 0.5f ), B3_FIX( 0.25f ) },
			{ B3_FIX( -0.3f ), B3_FIX( 0.6f ) },  { B3_FIX( 0.1f ), B3_FIX( -0.4f ) },
		};

		b3HullPoint2D pts[8];
		b3HullPoint2D hull[16];
		FillPoints2D( pts, input, 8 );

		int count = b3Hull2D( pts, 8, hull );
		ENSURE( count == 4 );

		// The reduction code assumes counter clockwise winding
		ENSURE( Hull2DArea( hull, count ) > B3_FIX( 0.0f ) );

		bool onHull[8] = { false };
		for ( int i = 0; i < count; ++i )
		{
			ENSURE( 0 <= hull[i].originalIndex && hull[i].originalIndex < 8 );
			onHull[hull[i].originalIndex] = true;
		}

		for ( int i = 0; i < 4; ++i )
		{
			ENSURE( onHull[i] == true );
		}

		for ( int i = 4; i < 8; ++i )
		{
			ENSURE( onHull[i] == false );
		}
	}

	// Collinear points collapse to a segment.
	{
		b3Vec2 input[5] = {
			{ B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
			{ B3_FIX( 2.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 3.0f ), B3_FIX( 0.0f ) },
			{ B3_FIX( 4.0f ), B3_FIX( 0.0f ) },
		};

		b3HullPoint2D pts[5];
		b3HullPoint2D hull[10];
		FillPoints2D( pts, input, 5 );

		ENSURE( b3Hull2D( pts, 5, hull ) == 2 );
	}

	// Coincident points weld and the lower input index wins, so a sort order change cannot
	// swap which of the two survives.
	{
		b3Fixed offset = b3FixMul( B3_FIX( 0.1f ) , B3_LINEAR_SLOP );

		b3Vec2 input[5] = {
			{ B3_FIX( -1.0f ), B3_FIX( -1.0f ) },
			{ B3_FIX( 1.0f ), B3_FIX( -1.0f ) },
			{ B3_FIX( 1.0f ), B3_FIX( 1.0f ) },
			{ B3_FIX( -1.0f ), B3_FIX( 1.0f ) },
			{ B3_FIX( -1.0f ) + offset, B3_FIX( -1.0f ) + offset },
		};

		b3HullPoint2D pts[5];
		b3HullPoint2D hull[10];
		FillPoints2D( pts, input, 5 );

		int count = b3Hull2D( pts, 5, hull );
		ENSURE( count == 4 );

		for ( int i = 0; i < count; ++i )
		{
			ENSURE( hull[i].originalIndex != 4 );
		}
	}

	// The surviving point set must not depend on the order the points arrive in.
	{
		b3Vec2 input[8] = {
			{ B3_FIX( -2.0f ), B3_FIX( -1.0f ) }, { B3_FIX( 0.0f ), B3_FIX( -1.5f ) },
			{ B3_FIX( 2.0f ), B3_FIX( -1.0f ) },	 { B3_FIX( 2.5f ), B3_FIX( 0.5f ) },
			{ B3_FIX( 1.0f ), B3_FIX( 2.0f ) },	 { B3_FIX( -1.0f ), B3_FIX( 1.8f ) },
			{ B3_FIX( -2.5f ), B3_FIX( 0.4f ) },	 { B3_FIX( 0.2f ), B3_FIX( 0.1f ) },
		};

		int permutation[8] = { 5, 2, 7, 0, 4, 6, 1, 3 };

		b3HullPoint2D pts[8];
		b3HullPoint2D hull[16];
		FillPoints2D( pts, input, 8 );

		int count1 = b3Hull2D( pts, 8, hull );
		ENSURE( count1 == 7 );

		bool onHull1[8] = { false };
		for ( int i = 0; i < count1; ++i )
		{
			onHull1[hull[i].originalIndex] = true;
		}

		b3HullPoint2D shuffled[8];
		b3HullPoint2D shuffledHull[16];
		for ( int i = 0; i < 8; ++i )
		{
			shuffled[i].p = input[permutation[i]];
			shuffled[i].separation = B3_FIX( 0.0f );
			shuffled[i].originalIndex = i;
		}

		int count2 = b3Hull2D( shuffled, 8, shuffledHull );
		ENSURE( count2 == count1 );

		bool onHull2[8] = { false };
		for ( int i = 0; i < count2; ++i )
		{
			onHull2[permutation[shuffledHull[i].originalIndex]] = true;
		}

		for ( int i = 0; i < 8; ++i )
		{
			ENSURE( onHull1[i] == onHull2[i] );
		}
	}

	return 0;
}

static int Hull2DDegenerateTest( void )
{
	const b3Fixed tolerance = B3_LINEAR_SLOP / 4;
	const b3Vec2 zero = { 0, 0 };
	b3HullPoint2D points[8];
	b3HullPoint2D hull[16];

	// Sorted neighbours closer than the weld radius collapse; those farther
	// apart remain distinct. Squaring this tolerance at Q48.16 loses the radius.
	const b3Vec2 nearPair[2] = { zero, { tolerance / 2, 0 } };
	FillPoints2D( points, nearPair, 2 );
	ENSURE( b3Hull2D( points, 2, hull ) == 1 );

	const b3Vec2 farPair[2] = { zero, { tolerance + 8 * B3_FIXED_EPSILON, 0 } };
	FillPoints2D( points, farPair, 2 );
	ENSURE( b3Hull2D( points, 2, hull ) == 2 );

	const b3Vec2 coincident[4] = { zero, zero, zero, zero };
	FillPoints2D( points, coincident, 4 );
	ENSURE( b3Hull2D( points, 4, hull ) == 1 );

	const b3Vec2 collinear[5] = { { 0, 0 }, { B3_FIX( 1.0f ), 0 }, { B3_FIX( 2.0f ), 0 },
		{ B3_FIX( 3.0f ), 0 }, { B3_FIX( 4.0f ), 0 } };
	FillPoints2D( points, collinear, 5 );
	ENSURE( b3Hull2D( points, 5, hull ) == 2 );

	const b3Vec2 ulpPair[2] = { zero, { B3_FIXED_EPSILON, B3_FIXED_EPSILON } };
	FillPoints2D( points, ulpPair, 2 );
	ENSURE( b3Hull2D( points, 2, hull ) == 1 );
	return 0;
}

#if B3_MAX_MANIFOLD_POINTS >= 8
static int Hull2DBoxFaceClipTest( void )
{
	// Equal square faces, one yawed by 45 degrees, clip to an octagon.
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) );
	b3LocalManifoldPoint points[B3_MAX_MANIFOLD_POINTS];
	b3LocalManifold manifold = { 0 };
	manifold.points = points;
	b3SATCache cache = { 0 };
	b3Transform transform = { { 0, B3_FIX( 1.99f ), 0 }, ExactQuat( kAxisY, B3_PI / 4 ) };
	b3CollideHulls( &manifold, B3_MAX_MANIFOLD_POINTS, &box.base, &box.base, transform, &cache );
	ENSURE( manifold.pointCount == 8 );
	b3Fixed minX = B3_FIXED_MAX, maxX = B3_FIXED_MIN;
	b3Fixed minZ = B3_FIXED_MAX, maxZ = B3_FIXED_MIN;
	for ( int i = 0; i < manifold.pointCount; ++i )
	{
		minX = b3FixMin( minX, points[i].point.x );
		maxX = b3FixMax( maxX, points[i].point.x );
		minZ = b3FixMin( minZ, points[i].point.z );
		maxZ = b3FixMax( maxZ, points[i].point.z );
	}
	ENSURE_SMALL( minX + B3_FIX( 1.0f ), B3_FIX( 0.001f ) );
	ENSURE_SMALL( maxX - B3_FIX( 1.0f ), B3_FIX( 0.001f ) );
	ENSURE_SMALL( minZ + B3_FIX( 1.0f ), B3_FIX( 0.001f ) );
	ENSURE_SMALL( maxZ - B3_FIX( 1.0f ), B3_FIX( 0.001f ) );
	return 0;
}

// Eight perimeter vertices fill the cap. The ninth point is interior but has
// the deepest separation; keeping it must not evict any cardinal extreme.
static int Hull2DDeepestExtremeTest( void )
{
	const b3Vec2 ring[8] = {
		{ -B3_FIX( 3.0f ), 0 }, { -B3_FIX( 2.0f ), -B3_FIX( 2.0f ) },
		{ 0, -B3_FIX( 3.0f ) }, { B3_FIX( 2.0f ), -B3_FIX( 2.0f ) },
		{ B3_FIX( 3.0f ), 0 }, { B3_FIX( 2.0f ), B3_FIX( 2.0f ) },
		{ 0, B3_FIX( 3.0f ) }, { -B3_FIX( 2.0f ), B3_FIX( 2.0f ) },
	};
	b3HullPoint2D hull[9];
	for ( int i = 0; i < 8; ++i )
	{
		hull[i] = (b3HullPoint2D){ .p = ring[i], .separation = 0, .originalIndex = i };
	}
	b3HullPoint2D deepest = { .p = { 0, 0 }, .separation = -B3_FIX( 1.0f ), .originalIndex = 8 };
	int count = b3IncludeDeepestHull2D( hull, 8, 8, deepest );
	ENSURE( count == 8 );
	bool seen[9] = { false };
	for ( int i = 0; i < count; ++i )
	{
		ENSURE( 0 <= hull[i].originalIndex && hull[i].originalIndex < 9 );
		seen[hull[i].originalIndex] = true;
	}
	ENSURE( seen[8] );
	ENSURE( seen[0] && seen[2] && seen[4] && seen[6] );
	return 0;
}

// A unique extreme can contribute little area when its adjacent facets are
// nearly collinear. Simplifying ten vertices to eight must retain it.
static int Hull2DExtremaSimplifyTest( void )
{
	const b3Vec2 ring[10] = {
		{ -B3_FIX( 3.0f ), 0 }, { -B3_FIX( 2.9f ), -B3_FIX( 1.0f ) },
		{ -B3_FIX( 2.0f ), -B3_FIX( 2.0f ) }, { 0, -B3_FIX( 3.0f ) },
		{ B3_FIX( 2.0f ), -B3_FIX( 2.0f ) }, { B3_FIX( 3.0f ), 0 },
		{ B3_FIX( 2.0f ), B3_FIX( 2.0f ) }, { 0, B3_FIX( 3.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 2.0f ) }, { -B3_FIX( 2.9f ), B3_FIX( 1.0f ) },
	};
	b3HullPoint2D hull[10];
	for ( int i = 0; i < 10; ++i )
	{
		hull[i] = (b3HullPoint2D){ .p = ring[i], .separation = 0, .originalIndex = i };
	}
	int count = b3SimplifyHull2D( hull, 10, 8 );
	ENSURE( count == 8 );
	bool seen[10] = { false };
	for ( int i = 0; i < count; ++i )
	{
		seen[hull[i].originalIndex] = true;
	}
	ENSURE( seen[0] && seen[3] && seen[5] && seen[7] );
	return 0;
}
#endif

// The default four-point path must remain byte-for-byte stable across this port.
#if B3_MAX_MANIFOLD_POINTS == 4
static int Hull2DDefaultHashTest( void )
{
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) );
	uint32_t seed = 0x954CF87u;
	uint32_t hash = 2166136261u;
	for ( int contact = 0; contact < 50; ++contact )
	{
		seed = seed * 1664525u + 1013904223u;
		b3Fixed x = (b3Fixed)( (int32_t)( ( seed >> 16 ) & 0x3fff ) - 0x2000 );
		seed = seed * 1664525u + 1013904223u;
		b3Fixed z = (b3Fixed)( (int32_t)( ( seed >> 16 ) & 0x3fff ) - 0x2000 );
		b3Transform transform = { { x, B3_FIX( 1.9f ), z }, b3Quat_identity };
		b3LocalManifoldPoint points[4];
		b3LocalManifold manifold = { .points = points };
		b3SATCache cache = { 0 };
		b3CollideHulls( &manifold, 4, &box.base, &box.base, transform, &cache );
		ENSURE( manifold.pointCount == 4 );
		for ( int i = -1; i < manifold.pointCount; ++i )
		{
			b3Fixed values[4] = { 0, 0, 0, 0 };
			if ( i < 0 )
			{
				values[0] = (b3Fixed)manifold.pointCount; // intentional raw integer tag
			}
			else
			{
				values[0] = points[i].point.x;
				values[1] = points[i].point.y;
				values[2] = points[i].point.z;
				values[3] = points[i].separation;
			}
			for ( int j = 0; j < 4; ++j )
			{
				uint64_t bits = (uint64_t)values[j];
				for ( int k = 0; k < 8; ++k )
				{
					hash = ( hash ^ (uint8_t)bits ) * 16777619u;
					bits >>= 8;
				}
			}
		}
	}
	// The unchanged base and this branch both produce 0x59A27095 for all 50 contacts.
	ENSURE( hash == 0x59A27095u );
	return 0;
}
#endif

static int SimplifyHull2DTest( void )
{
	// A 20-gon simplifies to the requested target.
	{
		b3HullPoint2D pts[20];
		b3HullPoint2D hull[40];

		for ( int i = 0; i < 20; ++i )
		{
            double angle = 2.0 * b3FixToDouble( B3_PI ) * (double)i / 20.0;
			pts[i].p = (b3Vec2){ b3FixFromDouble( cos( angle ) ), b3FixFromDouble( sin( angle ) ) };
			pts[i].separation = B3_FIX( 0.0f );
			pts[i].originalIndex = i;
		}

		int hullCount = b3Hull2D( pts, 20, hull );
		ENSURE( hullCount == 20 );

		int count = b3SimplifyHull2D( hull, hullCount, 6 );
		ENSURE( count == 6 );
		ENSURE( Hull2DArea( hull, count ) > B3_FIX( 0.0f ) );
	}

	// A square is already at or below the target, so simplification is a no-op.
	{
		b3Vec2 input[4] = {
			{ B3_FIX( -1.0f ), B3_FIX( -1.0f ) },
			{ B3_FIX( 1.0f ), B3_FIX( -1.0f ) },
			{ B3_FIX( 1.0f ), B3_FIX( 1.0f ) },
			{ B3_FIX( -1.0f ), B3_FIX( 1.0f ) },
		};

		b3HullPoint2D pts[4];
		b3HullPoint2D hull[8];
		FillPoints2D( pts, input, 4 );

		int hullCount = b3Hull2D( pts, 4, hull );
		ENSURE( hullCount == 4 );

		ENSURE( b3SimplifyHull2D( hull, hullCount, 8 ) == 4 );
		ENSURE( b3SimplifyHull2D( hull, hullCount, 4 ) == 4 );
	}

	return 0;
}

// ---- Oracle: b3CollideHullAndSphere on a box against the closed-form contact. ----
// Box3D's contract (upstream erincatto/box3d src/convex_manifold.c at a2e4816): no
// point when the core distance exceeds r + B3_SPECULATIVE_DISTANCE. Shallow (center
// outside the hull): one point, normal = (c - q)/|c - q| with q the closest hull point,
// separation = |c - q| - r, point = midpoint(q, c - r*n). Deep (center inside): the face
// of largest plane separation s gives the normal and separation = s - r.
static uint64_t SBMNextXorShift64( uint64_t* s )
{
	uint64_t x = *s;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	*s = x;
	return x;
}

static b3Fixed SBMRange( uint64_t* s, int64_t lo, int64_t hi )
{
	uint64_t span = (uint64_t)( hi - lo );
	uint64_t x = SBMNextXorShift64( s );
	return (b3Fixed)( lo + (int64_t)( x % ( span + 1 ) ) );
}

static uint64_t SBMPutU64( uint64_t hash, uint64_t v )
{
	for ( int i = 0; i < 8; ++i )
	{
		hash ^= (uint8_t)( v & 0xff );
		hash *= 0x100000001b3ULL;
		v >>= 8;
	}
	return hash;
}

static int SphereBoxManifoldOracleTest( void )
{
	const b3Fixed kHalfExtLo = (b3Fixed)13107;
	const b3Fixed kHalfExtHi = (b3Fixed)98304;
	const b3Fixed kRadiusLo = (b3Fixed)3276;
	const b3Fixed kRadiusHi = (b3Fixed)32768;
	const b3Fixed kPad = (b3Fixed)1310;
	const double kSpec = b3FixToDouble( B3_SPECULATIVE_DISTANCE );

	uint64_t hash = 0xcbf29ce484222325ULL;

	int faceRegions = 0, edgeRegions = 0, vertexRegions = 0;
	double maxSepErr = 0.0, maxPointErr = 0.0, maxNormalErr = 0.0;
	int oracleViolations = 0;

	// Family O (outside): shallow contract, one point on the closest hull feature.
	{
		uint64_t seed = 0x5EED0008ULL;
		int accepted = 0;
		int guard = 0;
		int pointCountViolations = 0;
		int sepViolations = 0;
		int pointViolations = 0;
		int normalViolations = 0;

		struct SBMWorstCase
		{
			int64_t raw[7];
			int enginePointCount;
			double engineNormal[3];
			double enginePoint[3];
			double engineSep;
			double oracleNormal[3];
			double oraclePoint[3];
			double oracleSep;
			double sepErr;
			double pointErr;
			double normalErr;
			double rank;
		};

		struct SBMWorstCase worst[3];
		for ( int i = 0; i < 3; ++i )
		{
			worst[i].rank = -1.0;
		}

		while ( accepted < 256 && guard++ < 400000 )
		{
			b3Fixed hx = SBMRange( &seed, kHalfExtLo, kHalfExtHi );
			b3Fixed hy = SBMRange( &seed, kHalfExtLo, kHalfExtHi );
			b3Fixed hz = SBMRange( &seed, kHalfExtLo, kHalfExtHi );
			b3Fixed radius = SBMRange( &seed, kRadiusLo, kRadiusHi );

			double hx_d = b3FixToDouble( hx );
			double hy_d = b3FixToDouble( hy );
			double hz_d = b3FixToDouble( hz );
			double r_d = b3FixToDouble( radius );

			int64_t bx = hx + radius + kPad;
			int64_t by = hy + radius + kPad;
			int64_t bz = hz + radius + kPad;
			b3Fixed cx = SBMRange( &seed, -bx, bx );
			b3Fixed cy = SBMRange( &seed, -by, by );
			b3Fixed cz = SBMRange( &seed, -bz, bz );

			double c[3] = { b3FixToDouble( cx ), b3FixToDouble( cy ), b3FixToDouble( cz ) };
			double h[3] = { hx_d, hy_d, hz_d };

			double q[3];
			int nClamped = 0;
			double distSq = 0.0;
			for ( int ax = 0; ax < 3; ++ax )
			{
				q[ax] = c[ax];
				if ( q[ax] > h[ax] ) { q[ax] = h[ax]; nClamped++; }
				else if ( q[ax] < -h[ax] ) { q[ax] = -h[ax]; nClamped++; }
				double d = c[ax] - q[ax];
				distSq += d * d;
			}

			// The card's Family O acceptance range, verbatim: exactly one shallow point
			// is expected from 1 mm off the hull up to the speculative margin.
			double upperDist = r_d + kSpec - 0.001;
			if ( !( 0.001 * 0.001 < distSq && distSq < upperDist * upperDist ) )
			{
				continue;
			}

			double dist = sqrt( distSq );
			double n[3], pt[3];
			for ( int ax = 0; ax < 3; ++ax )
			{
				n[ax] = ( c[ax] - q[ax] ) / dist;
				pt[ax] = 0.5 * ( q[ax] + c[ax] - r_d * n[ax] );
			}
			double sep = dist - r_d;

			b3BoxHull hull = b3MakeBoxHull( hx, hy, hz );
			b3Sphere sphere = { { cx, cy, cz }, radius };
			b3LocalManifoldPoint points[8] = { 0 };
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SimplexCache cache = { 0 };
			b3CollideHullAndSphere( &manifold, 8, &hull.base, &sphere, b3Transform_identity, &cache );

			double sepEng = b3FixToDouble( points[0].separation );
			double nEng[3] = { b3FixToDouble( manifold.normal.x ), b3FixToDouble( manifold.normal.y ), b3FixToDouble( manifold.normal.z ) };
			double pEng[3] = { b3FixToDouble( points[0].point.x ), b3FixToDouble( points[0].point.y ), b3FixToDouble( points[0].point.z ) };

			double sepErr = fabs( sepEng - sep );
			double pointErr = 0.0;
			double normalErr = 0.0;
			for ( int ax = 0; ax < 3; ++ax )
			{
				double pe = fabs( pEng[ax] - pt[ax] );
				if ( pe > pointErr ) pointErr = pe;
				double ne = fabs( nEng[ax] - n[ax] );
				if ( ne > normalErr ) normalErr = ne;
			}
			if ( sepErr > maxSepErr ) maxSepErr = sepErr;
			if ( pointErr > maxPointErr ) maxPointErr = pointErr;
			if ( normalErr > maxNormalErr ) maxNormalErr = normalErr;

			if ( manifold.pointCount != 1 ) pointCountViolations++;
			if ( sepErr > 2e-4 ) sepViolations++;
			if ( pointErr > 2e-4 ) pointViolations++;
			if ( normalErr > 1e-3 ) normalViolations++;

			struct SBMWorstCase rec;
			rec.raw[0] = hx;
			rec.raw[1] = hy;
			rec.raw[2] = hz;
			rec.raw[3] = radius;
			rec.raw[4] = cx;
			rec.raw[5] = cy;
			rec.raw[6] = cz;
			rec.enginePointCount = manifold.pointCount;
			rec.engineSep = sepEng;
			rec.oracleSep = sep;
			rec.sepErr = sepErr;
			rec.pointErr = pointErr;
			rec.normalErr = normalErr;
			rec.rank = sepErr;
			if ( pointErr > rec.rank ) rec.rank = pointErr;
			if ( normalErr > rec.rank ) rec.rank = normalErr;
			for ( int ax = 0; ax < 3; ++ax )
			{
				rec.engineNormal[ax] = nEng[ax];
				rec.enginePoint[ax] = pEng[ax];
				rec.oracleNormal[ax] = n[ax];
				rec.oraclePoint[ax] = pt[ax];
			}
			for ( int i = 0; i < 3; ++i )
			{
				if ( rec.rank > worst[i].rank )
				{
					for ( int j = 2; j > i; --j )
					{
						worst[j] = worst[j - 1];
					}
					worst[i] = rec;
					break;
				}
			}

			if ( nClamped == 1 ) faceRegions++;
			else if ( nClamped == 2 ) edgeRegions++;
			else vertexRegions++;

			hash = SBMPutU64( hash, (uint64_t)manifold.pointCount );
			hash = SBMPutU64( hash, (uint64_t)manifold.normal.x );
			hash = SBMPutU64( hash, (uint64_t)manifold.normal.y );
			hash = SBMPutU64( hash, (uint64_t)manifold.normal.z );
			hash = SBMPutU64( hash, (uint64_t)points[0].point.x );
			hash = SBMPutU64( hash, (uint64_t)points[0].point.y );
			hash = SBMPutU64( hash, (uint64_t)points[0].point.z );
			hash = SBMPutU64( hash, (uint64_t)points[0].separation );

			accepted++;
		}
		printf( "  SphereBoxManifoldOracleTest familyO pointCount=%d sep=%d point=%d normal=%d\n",
				pointCountViolations, sepViolations, pointViolations, normalViolations );
		for ( int i = 0; i < 3; ++i )
		{
			if ( worst[i].rank < 0.0 ) continue;
			printf( "  SphereBoxManifoldOracleTest worst%d raw=(%lld,%lld,%lld,%lld,%lld,%lld,%lld) count=%d engineN=(%g,%g,%g) oracleN=(%g,%g,%g) engineP=(%g,%g,%g) oracleP=(%g,%g,%g) engineSep=%g oracleSep=%g sepErr=%g pointErr=%g normalErr=%g\n",
					i, (long long)worst[i].raw[0], (long long)worst[i].raw[1], (long long)worst[i].raw[2], (long long)worst[i].raw[3],
					(long long)worst[i].raw[4], (long long)worst[i].raw[5], (long long)worst[i].raw[6], worst[i].enginePointCount,
					worst[i].engineNormal[0], worst[i].engineNormal[1], worst[i].engineNormal[2],
					worst[i].oracleNormal[0], worst[i].oracleNormal[1], worst[i].oracleNormal[2],
					worst[i].enginePoint[0], worst[i].enginePoint[1], worst[i].enginePoint[2],
					worst[i].oraclePoint[0], worst[i].oraclePoint[1], worst[i].oraclePoint[2],
					worst[i].engineSep, worst[i].oracleSep, worst[i].sepErr, worst[i].pointErr, worst[i].normalErr );
		}
		ENSURE( accepted == 256 );
		ENSURE( faceRegions > 0 && edgeRegions > 0 && vertexRegions > 0 );
		// The disagreements are the finding: they are reported above and fail the subtest
		// after the hash and the measured maxima have been printed, never hidden by a filter.
		oracleViolations += pointCountViolations + sepViolations + pointViolations + normalViolations;
	}

	// Family I (inside): deep contract, one point on the face of largest plane separation.
	{
		uint64_t seed = 0x5EED0009ULL;
		int accepted = 0;
		int guard = 0;
		while ( accepted < 128 && guard++ < 400000 )
		{
			b3Fixed hx = SBMRange( &seed, kHalfExtLo, kHalfExtHi );
			b3Fixed hy = SBMRange( &seed, kHalfExtLo, kHalfExtHi );
			b3Fixed hz = SBMRange( &seed, kHalfExtLo, kHalfExtHi );
			b3Fixed radius = SBMRange( &seed, kRadiusLo, kRadiusHi );

			double hx_d = b3FixToDouble( hx );
			double hy_d = b3FixToDouble( hy );
			double hz_d = b3FixToDouble( hz );
			double r_d = b3FixToDouble( radius );

			int64_t bx = hx - (int64_t)655;
			int64_t by = hy - (int64_t)655;
			int64_t bz = hz - (int64_t)655;
			b3Fixed cx = SBMRange( &seed, -bx, bx );
			b3Fixed cy = SBMRange( &seed, -by, by );
			b3Fixed cz = SBMRange( &seed, -bz, bz );

			double c[3] = { b3FixToDouble( cx ), b3FixToDouble( cy ), b3FixToDouble( cz ) };
			double h[3] = { hx_d, hy_d, hz_d };

			double s[6];
			s[0] = c[0] - h[0];
			s[1] = -c[0] - h[0];
			s[2] = c[1] - h[1];
			s[3] = -c[1] - h[1];
			s[4] = c[2] - h[2];
			s[5] = -c[2] - h[2];

			int best = 0;
			for ( int i = 1; i < 6; ++i ) if ( s[i] > s[best] ) best = i;
			int second = ( best == 0 ) ? 1 : 0;
			for ( int i = 0; i < 6; ++i ) if ( i != best && s[i] > s[second] ) second = i;
			if ( s[best] - s[second] < 0.01 )
			{
				continue;
			}

			double n[3] = { 0.0, 0.0, 0.0 };
			n[best / 2] = ( best % 2 == 0 ) ? 1.0 : -1.0;
			double sep = s[best] - r_d;

			b3BoxHull hull = b3MakeBoxHull( hx, hy, hz );
			b3Sphere sphere = { { cx, cy, cz }, radius };
			b3LocalManifoldPoint points[8] = { 0 };
			b3LocalManifold manifold = { 0 };
			manifold.points = points;
			b3SimplexCache cache = { 0 };
			b3CollideHullAndSphere( &manifold, 8, &hull.base, &sphere, b3Transform_identity, &cache );

			ENSURE( manifold.pointCount == 1 );

			double sepEng = b3FixToDouble( points[0].separation );
			double sepErr = fabs( sepEng - sep );
			if ( sepErr > maxSepErr ) maxSepErr = sepErr;
			ENSURE( sepErr <= 2e-4 );

			double nEng[3] = { b3FixToDouble( manifold.normal.x ), b3FixToDouble( manifold.normal.y ), b3FixToDouble( manifold.normal.z ) };
			for ( int ax = 0; ax < 3; ++ax )
			{
				double ne = fabs( nEng[ax] - n[ax] );
				if ( ne > maxNormalErr ) maxNormalErr = ne;
				ENSURE( ne <= 1e-3 );
			}

			hash = SBMPutU64( hash, (uint64_t)manifold.pointCount );
			hash = SBMPutU64( hash, (uint64_t)manifold.normal.x );
			hash = SBMPutU64( hash, (uint64_t)manifold.normal.y );
			hash = SBMPutU64( hash, (uint64_t)manifold.normal.z );
			hash = SBMPutU64( hash, (uint64_t)points[0].point.x );
			hash = SBMPutU64( hash, (uint64_t)points[0].point.y );
			hash = SBMPutU64( hash, (uint64_t)points[0].point.z );
			hash = SBMPutU64( hash, (uint64_t)points[0].separation );

			accepted++;
		}
		ENSURE( accepted == 128 );
	}

	printf( "  SphereBoxManifoldOracleTest hash=0x%016llx\n", (unsigned long long)hash );
	printf( "  SphereBoxManifoldOracleTest maxSepErr=%g maxPointErr=%g maxNormalErr=%g face=%d edge=%d vertex=%d\n",
			maxSepErr, maxPointErr, maxNormalErr, faceRegions, edgeRegions, vertexRegions );
	// GOLDEN harvested from build-gcc; identical in build-clang and build-ludicrous.
	ENSURE( hash == 0x9a3fb8b692f63667ULL );

	// FINDING: the exact Family O range the card specifies (1 mm to r + speculative - 1 mm,
	// no near-boundary exclusion) puts fixed3d's GJK-derived shallow contact outside the
	// stated tolerances near the hull. The normal drifts toward an adjacent edge and the
	// point follows it; separation stays inside its tolerance. This assert is deliberately
	// left red: widening the tolerance or narrowing the generator would hide a real
	// disagreement between b3CollideHullAndSphere and Box3D's closed-form contract.
	ENSURE( oracleViolations == 0 );

	return 0;
}

// The library reports the point count it was built with. A mismatch against the manifold
// this translation unit sees means the app and the library disagree on b3Manifold.
static int MaxPointCountTest( void )
{
	b3Manifold manifold;
	ENSURE( b3GetMaxManifoldPoints() == (int)( sizeof( manifold.points ) / sizeof( manifold.points[0] ) ) );

	return 0;
}

int ManifoldTest( void )
{
	RUN_SUBTEST( MaxPointCountTest );
	RUN_SUBTEST( Hull2DReductionTest );
	RUN_SUBTEST( Hull2DDegenerateTest );
#if B3_MAX_MANIFOLD_POINTS == 4
	RUN_SUBTEST( Hull2DDefaultHashTest );
#endif
#if B3_MAX_MANIFOLD_POINTS >= 8
	RUN_SUBTEST( Hull2DBoxFaceClipTest );
	RUN_SUBTEST( Hull2DDeepestExtremeTest );
	RUN_SUBTEST( Hull2DExtremaSimplifyTest );
#endif
	RUN_SUBTEST( SimplifyHull2DTest );
	RUN_SUBTEST( CrossedEdgeTest );
	RUN_SUBTEST( EdgeAxisScaleTest );
	RUN_SUBTEST( EdgeAxisLargeScaleTest );
	RUN_SUBTEST( EdgeCacheTest );
	RUN_SUBTEST( EdgeEndpointTest );
	RUN_SUBTEST( ParallelEdgeTest );
	RUN_SUBTEST( ParallelEdgeManualTest );
	RUN_SUBTEST( OverlapNeverEmptyTest );
	RUN_SUBTEST( RidgeCrossingTest );
	RUN_SUBTEST( TriangleEdgeTest );
	RUN_SUBTEST( TriangleParallelEdgeTest );
	RUN_SUBTEST( EdgeAxisOracleTest );
	RUN_SUBTEST( EdgeAxisRandomOracleTest );
	RUN_SUBTEST( HullCapsuleEdgeDeepTest );
	RUN_SUBTEST( TriangleHullEdgeSweepTest );
	RUN_SUBTEST( CapsuleTriangleEdgeDeepTest );
	RUN_SUBTEST( CapsuleTriangleEdgeWalkTest );
	RUN_SUBTEST( SphereHullSeamTest );
	RUN_SUBTEST( CapsuleHullSeamTest );
	RUN_SUBTEST( ClipSegmentSmallCrossingTest );
	RUN_SUBTEST( CapsuleTriangleFaceDeepTest );
	RUN_SUBTEST( HullCapsuleFaceDeepTest );
	RUN_SUBTEST( SpeculativeMarginTest );
	RUN_SUBTEST( SphereBoxManifoldOracleTest );

	return 0;
}
