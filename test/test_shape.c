// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

// b3GetWorldFromId and the shape material storage internals are needed to inspect stored bytes
#include "physics_world.h"
#include "shape.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static b3Capsule capsule = { { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
static b3Sphere sphere = { { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
static b3BoxHull box = { 0 };

#define N 4

static int ShapeMassTest( void )
{
	// Sphere
	{
		b3MassData md = b3ComputeSphereMass( &sphere, B3_FIX( 1.0f ) );
		b3Fixed mass = b3FixMul( b3FixDiv( B3_FIX( 4.0f ) , B3_FIX( 3.0f ) ) , B3_PI );
		ENSURE_SMALL( md.mass - mass, 8 * B3_FIXED_EPSILON );
		ENSURE( md.center.x == B3_FIX( 1.0f ) && md.center.y == B3_FIX( 0.0f ) );

		// Inertia is now about the shape center of mass, so the offset does not appear.
		b3Fixed inertia = b3FixMul( b3FixDiv( B3_FIX( 2.0f ) , B3_FIX( 5.0f ) ) , mass );
		ENSURE_SMALL( md.inertia.cx.x - inertia, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( md.inertia.cy.y - inertia, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( md.inertia.cz.z - inertia, 8 * B3_FIXED_EPSILON );
	}

	// Analytic box hull
	{
		b3MassData md = b3ComputeHullMass( &box.base, B3_FIX( 1.0f ) );
		b3Fixed mass = b3FixMul( b3FixMul( B3_FIX( 2.0f ) , B3_FIX( 2.0f ) ) , B3_FIX( 2.0f ) );
		ENSURE_SMALL( md.mass - mass, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( md.center.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( md.center.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( md.center.z, 8 * B3_FIXED_EPSILON );
		// Divide last so the reference does not lose precision to a truncated reciprocal
		b3Fixed inertia = b3FixDiv( b3FixMul( mass, b3FixMul( B3_FIX( 2.0f ), B3_FIX( 2.0f ) ) + b3FixMul( B3_FIX( 2.0f ), B3_FIX( 2.0f ) ) ), B3_FIX( 12.0f ) );
		ENSURE_SMALL( md.inertia.cx.x - inertia, 2 * B3_FIXED_EPSILON );
		ENSURE_SMALL( md.inertia.cy.y - inertia, 2 * B3_FIXED_EPSILON );
		ENSURE_SMALL( md.inertia.cz.z - inertia, 2 * B3_FIXED_EPSILON );
	}

	// Translated box
	{
		b3Vec3 offset = { B3_FIX( 0.4f ), -B3_FIX( 0.7f ), B3_FIX( 0.1f ) };
		b3Transform transform = {
			.p = offset,
			.q = b3Quat_identity,
		};
		b3Vec3 h = { B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.3f ) };
		b3BoxHull b1 = b3MakeBoxHull( h.x, h.y, h.z );
		b3BoxHull b2 = b3MakeTransformedBoxHull( h.x, h.y, h.z, transform );

		b3MassData m1 = b3ComputeHullMass( &b1.base, B3_FIX( 1.0f ) );
		b3MassData m2 = b3ComputeHullMass( &b2.base, B3_FIX( 1.0f ) );

		ENSURE_SMALL( m1.mass - m2.mass, 8 * B3_FIXED_EPSILON );

		b3Matrix3 d = b3SubMM( b1.base.centralInertia, b2.base.centralInertia );
		ENSURE_SMALL( d.cx.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cx.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cx.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.z, 8 * B3_FIXED_EPSILON );

		ENSURE_SMALL( m2.center.x - offset.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( m2.center.y - offset.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( m2.center.z - offset.z, 8 * B3_FIXED_EPSILON );
	}

	// Rotated box
	{
		b3Vec3 h1 = { B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.3f ) };
		b3Vec3 h2 = { B3_FIX( 0.25f ), B3_FIX( 0.3f ), B3_FIX( 0.5f ) };
		b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, b3Vec3_axisZ );
		b3Transform transform = {
			.p = b3Vec3_zero,
			.q = q,
		};
		b3BoxHull b1 = b3MakeTransformedBoxHull( h1.x, h1.y, h1.z, transform );
		b3BoxHull b2 = b3MakeBoxHull( h2.x, h2.y, h2.z );

		b3MassData m1 = b3ComputeHullMass( &b1.base, B3_FIX( 1.0f ) );
		b3MassData m2 = b3ComputeHullMass( &b2.base, B3_FIX( 1.0f ) );

		ENSURE_SMALL( m1.mass - m2.mass, 8 * B3_FIXED_EPSILON );

		b3Matrix3 d = b3SubMM( b1.base.centralInertia, b2.base.centralInertia );
		ENSURE_SMALL( d.cx.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cx.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cx.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.z, 8 * B3_FIXED_EPSILON );

		ENSURE_SMALL( m1.center.x - m2.center.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( m1.center.y - m2.center.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( m1.center.z - m2.center.z, 8 * B3_FIXED_EPSILON );
	}

	// Transformed box
	{
		b3Vec3 offset = { B3_FIX( 0.4f ), -B3_FIX( 0.7f ), B3_FIX( 0.1f ) };
		b3Vec3 h1 = { B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.3f ) };
		b3Vec3 h2 = { B3_FIX( 0.25f ), B3_FIX( 0.3f ), B3_FIX( 0.5f ) };
		b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, b3Vec3_axisZ );
		b3Transform transform = {
			.p = offset,
			.q = q,
		};
		b3BoxHull b1 = b3MakeTransformedBoxHull( h1.x, h1.y, h1.z, transform );
		b3BoxHull b2 = b3MakeBoxHull( h2.x, h2.y, h2.z );

		b3MassData m1 = b3ComputeHullMass( &b1.base, B3_FIX( 1.0f ) );
		b3MassData m2 = b3ComputeHullMass( &b2.base, B3_FIX( 1.0f ) );

		ENSURE_SMALL( m1.mass - m2.mass, 8 * B3_FIXED_EPSILON );

		b3Matrix3 d = b3SubMM( b1.base.centralInertia, b2.base.centralInertia );
		ENSURE_SMALL( d.cx.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cx.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cx.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cy.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( d.cz.z, 8 * B3_FIXED_EPSILON );

		ENSURE_SMALL( m1.center.x - offset.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( m1.center.y - offset.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( m1.center.z - offset.z, 8 * B3_FIXED_EPSILON );
	}

	// Capsule
	{
		b3Fixed radius = capsule.radius;
		b3Fixed length = b3Distance( capsule.center1, capsule.center2 );

		// Capsule along x-axis
		b3MassData md = b3ComputeCapsuleMass( &capsule, B3_FIX( 1.0f ) );

		// Box that fully contains capsule. Upper bound on capsule mass.
		b3BoxHull r = b3MakeBoxHull( radius + b3FixMul( B3_FIX( 0.5f ) , length ), radius, radius );
		b3MassData mdUpper = b3ComputeHullMass( &r.base, B3_FIX( 1.0f ) );

		// Approximate capsule using convex hull. This should be a lower bound on the
		// capsule mass.
		b3Vec3 points[2 * N * N];
		b3Fixed d = b3FixDiv( B3_PI, b3FixFromInt( N - 1 ) );
		b3Fixed angle1 = b3FixMul( -B3_FIX( 0.5f ) , B3_PI );
		int index = 0;
		for ( int i = 0; i < N; ++i )
		{
			b3Fixed s1 = b3Sin( angle1 );
			b3Fixed c1 = b3Cos( angle1 );
			b3Fixed angle2 = b3FixMul( -B3_FIX( 0.5f ) , B3_PI );
			for ( int j = 0; j < N; ++j )
			{
				points[index].x = B3_FIX( 1.0f ) + b3FixMul( radius , c1 );
				points[index].y = b3FixMul( b3FixMul( radius , s1 ) , b3Cos( angle2 ) );
				points[index].z = b3FixMul( b3FixMul( radius , s1 ) , b3Sin( angle2 ) );
				angle2 += d;
				index += 1;
			}

			angle1 += d;
		}

		angle1 = b3FixMul( B3_FIX( 0.5f ) , B3_PI );
		for ( int i = 0; i < N; ++i )
		{
			b3Fixed s1 = b3Sin( angle1 );
			b3Fixed c1 = b3Cos( angle1 );
			b3Fixed angle2 = b3FixMul( -B3_FIX( 0.5f ) , B3_PI );
			for ( int j = 0; j < N; ++j )
			{
				points[index].x = -B3_FIX( 1.0f ) + b3FixMul( radius , c1 );
				points[index].y = b3FixMul( b3FixMul( radius , s1 ) , b3Cos( angle2 ) );
				points[index].z = b3FixMul( b3FixMul( radius , s1 ) , b3Sin( angle2 ) );
				angle2 += d;
				index += 1;
			}

			angle1 += d;
		}

		ENSURE( index == 2 * N * N );

		b3HullData* hull = b3CreateHull( points, 2 * N * N, 2 * N * N );
		b3MassData mdLower = b3ComputeHullMass( hull, B3_FIX( 1.0f ) );

		ENSURE( mdLower.mass < md.mass && md.mass < mdUpper.mass );
		ENSURE( mdLower.inertia.cx.x < md.inertia.cx.x && md.inertia.cx.x < mdUpper.inertia.cx.x );
		ENSURE( mdLower.inertia.cy.y < md.inertia.cy.y && md.inertia.cy.y < mdUpper.inertia.cy.y );
		ENSURE( mdLower.inertia.cz.z < md.inertia.cz.z && md.inertia.cz.z < mdUpper.inertia.cz.z );

		b3DestroyHull( hull );
	}

	return 0;
}

static int ShapeAABBTest( void )
{
	{
		b3AABB b = b3ComputeSphereAABB( &sphere, b3Transform_identity );
		ENSURE_SMALL( b.lowerBound.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.lowerBound.z + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.x - B3_FIX( 2.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.y - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.z - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
	}

	{
		b3AABB b = b3ComputeCapsuleAABB( &capsule, b3Transform_identity );
		ENSURE_SMALL( b.lowerBound.x + B3_FIX( 2.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.lowerBound.z + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.x - B3_FIX( 2.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.y - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.z - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
	}
	{
		b3AABB b = b3ComputeHullAABB( &box.base, b3Transform_identity );
		ENSURE_SMALL( b.lowerBound.x + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.lowerBound.y + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.lowerBound.z + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.x - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.y - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( b.upperBound.z - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
	}

	return 0;
}

#if 0
static int PointInShapeTest( void )
{
	b3Vec2 p1 = { 0.5f, 0.5f };
	b3Vec2 p2 = { 4.0f, -4.0f };

	{
		bool hit;
		hit = b3PointInSphere( p1, &sphere );
		ENSURE( hit == true );
		hit = b3PointInSphere( p2, &sphere );
		ENSURE( hit == false );
	}

	{
		bool hit;
		hit = b3PointInPolygon( p1, &box );
		ENSURE( hit == true );
		hit = b3PointInPolygon( p2, &box );
		ENSURE( hit == false );
	}

	return 0;
}
#endif

// Shared assertions for a surface hit. The normal points outward toward the ray,
// the point sits on the surface, and the point lies on the ray at the reported fraction.
static int CheckCastHit( b3CastOutput out, b3Vec3 origin, b3Vec3 translation, b3Vec3 point, b3Vec3 normal, b3Fixed fraction,
						 b3Fixed tol )
{
	ENSURE( out.hit );
	ENSURE_SMALL( out.fraction - fraction, tol );
	ENSURE_SMALL( out.point.x - point.x, tol );
	ENSURE_SMALL( out.point.y - point.y, tol );
	ENSURE_SMALL( out.point.z - point.z, tol );
	ENSURE_SMALL( out.normal.x - normal.x, tol );
	ENSURE_SMALL( out.normal.y - normal.y, tol );
	ENSURE_SMALL( out.normal.z - normal.z, tol );

	b3Vec3 onRay = b3MulAdd( origin, out.fraction, translation );
	// The reported fraction is quantized, so the on-ray reconstruction error
	// scales with the ray length.
	b3Fixed onRayTol = tol + b3FixMul( b3Length( translation ), 2 * B3_FIXED_EPSILON );
	ENSURE_SMALL( b3Distance( out.point, onRay ), onRayTol );
	return 0;
}

// The shared initial overlap convention: a ray starting inside a solid reports the origin
// with zero fraction and no normal.
static int CheckInitialOverlap( b3CastOutput out, b3Vec3 origin )
{
	ENSURE( out.hit );
	ENSURE( out.fraction == B3_FIX( 0.0f ) );
	ENSURE_SMALL( b3Distance( out.point, origin ), 8 * B3_FIXED_EPSILON );
	ENSURE( out.normal.x == B3_FIX( 0.0f ) && out.normal.y == B3_FIX( 0.0f ) && out.normal.z == B3_FIX( 0.0f ) );
	return 0;
}

static int RayCastSphereHitTest( void )
{
	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

	// Hit along each principal axis. Surface at distance 3 over a length 8 ray.
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
						   b3FixDiv( B3_FIX( 3.0f ) , B3_FIX( 8.0f ) ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 8.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
						   b3FixDiv( B3_FIX( 3.0f ) , B3_FIX( 8.0f ) ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 4.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 8.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
						   b3FixDiv( B3_FIX( 3.0f ) , B3_FIX( 8.0f ) ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}

	// Offset center, hit partway along the ray.
	{
		b3Sphere s2 = { { B3_FIX( 5.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 2.0f ) };
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 10.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastSphere( &s2, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
						   B3_FIX( 0.3f ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}

	// Diagonal ray straight through the center.
	{
		b3Fixed k = B3_FIX( 0.70710678f );
		b3RayCastInput input = { { -B3_FIX( 3.0f ), -B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 6.0f ), B3_FIX( 6.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -k, -k, B3_FIX( 0.0f ) }, (b3Vec3){ -k, -k, B3_FIX( 0.0f ) }, B3_FIX( 0.382149f ),
						   B3_FIX( 1e-4f ) ) )
			return 1;
	}

	return 0;
}

static int RayCastSphereMissTest( void )
{
	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

	// Pointing away.
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { -B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	// Passes wide of the sphere.
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	// Aimed at the sphere but the translation stops short.
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	return 0;
}

static int RayCastSphereClipTest( void )
{
	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

	// The surface is reached at fraction 3/8. Straddle it with maxFraction.
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.374f ) };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.376f ) };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( out.fraction - b3FixDiv( B3_FIX( 3.0f ) , B3_FIX( 8.0f ) ), 8 * B3_FIXED_EPSILON );
	}
	return 0;
}

static int RayCastSphereInteriorTest( void )
{
	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

	// Origin inside reports the origin with zero fraction.
	{
		b3RayCastInput input = { { B3_FIX( 0.3f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		ENSURE( out.hit );
		ENSURE( out.fraction == B3_FIX( 0.0f ) );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), 8 * B3_FIXED_EPSILON );
	}
	// Zero length ray inside.
	{
		b3RayCastInput input = { { B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastSphere( &s, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), 8 * B3_FIXED_EPSILON );
	}
	// Zero length ray outside.
	{
		b3RayCastInput input = { { B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	return 0;
}

static int RayCastSphereGrazeTest( void )
{
	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

	// Just inside the radius grazes a hit, just outside misses.
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 0.999f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastSphere( &s, &input ).hit );
	}
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 1.001f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastSphere( &s, &input ).hit == false );
	}
	return 0;
}

// Capsule along x from -2 to 2, radius 1. Reused by the capsule ray cast subtests.
static const b3Capsule rayCapsule = { { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

static int RayCastCapsuleSideTest( void )
{
	// Perpendicular hit on the cylindrical side. Surface at distance 2 over a length 6 ray.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 6.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
						   b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}
	// Same from +z to exercise the other transverse direction.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 3.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 6.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
						   b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}
	// Side hit nearer the c1 end.
	{
		b3RayCastInput input = { { -B3_FIX( 1.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 6.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
						   b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}
	return 0;
}

static int RayCastCapsuleObliqueTest( void )
{
	// Oblique ray in the z=0 plane. It crosses y=1 inside the cylinder span, so the
	// normal stays transverse. Exercises the non perpendicular ray/axis solve where
	// dot(axis, rayAxis) != 0.
	b3RayCastInput input = { { -B3_FIX( 3.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 4.0f ), -B3_FIX( 4.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
	b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
	return CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ),
						 B3_FIX( 1e-4f ) );
}

static int RayCastCapsuleCapTest( void )
{
	b3Fixed k = B3_FIX( 0.70710678f );

	// Collinear ray hits the c2 hemisphere from beyond the end.
	{
		b3RayCastInput input = { { B3_FIX( 5.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { -B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
						   b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 4.0f ) ), ( 8 * B3_FIXED_EPSILON ) ) )
			return 1;
	}
	// Off-axis ray through the c2 cap center, approaching from outside the cylinder.
	{
		b3RayCastInput input = { { B3_FIX( 4.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) }, { -B3_FIX( 4.0f ), -B3_FIX( 4.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ B3_FIX( 2.0f ) + k, k, B3_FIX( 0.0f ) }, (b3Vec3){ k, k, B3_FIX( 0.0f ) }, B3_FIX( 0.323223f ),
						   B3_FIX( 1e-4f ) ) )
			return 1;
	}
	// Mirror through the c1 cap center.
	{
		b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 4.0f ), -B3_FIX( 4.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		if ( CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -B3_FIX( 2.0f ) - k, k, B3_FIX( 0.0f ) }, (b3Vec3){ -k, k, B3_FIX( 0.0f ) },
						   B3_FIX( 0.323223f ), B3_FIX( 1e-4f ) ) )
			return 1;
	}
	return 0;
}

static int RayCastCapsuleMissTest( void )
{
	// Pointing away.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Crosses above the axis more than a radius away.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 2.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 8.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Aimed at the side but the translation stops short.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Parallel to the axis and outside the cylinder.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	// Descends past the rounded c2 end, beyond cap reach.
	{
		b3RayCastInput input = { { B3_FIX( 4.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 6.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	return 0;
}

static int RayCastCapsuleInteriorTest( void )
{
	// Origin on the axis between the caps.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 5.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE( out.fraction == B3_FIX( 0.0f ) );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), 8 * B3_FIXED_EPSILON );
	}
	// Origin inside the c2 hemisphere, past the cylinder end.
	{
		b3RayCastInput input = { { B3_FIX( 2.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 5.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE( out.fraction == B3_FIX( 0.0f ) );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), 8 * B3_FIXED_EPSILON );
	}
	// Zero length ray inside.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( b3Distance( out.point, input.origin ), 8 * B3_FIXED_EPSILON );
	}
	// Zero length ray outside.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	return 0;
}

static int RayCastCapsuleDegenerateTest( void )
{
	// Coincident centers collapse to a sphere.
	b3Capsule c = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
	b3RayCastInput input = { { -B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
	b3CastOutput out = b3RayCastCapsule( &c, &input );
	return CheckCastHit( out, input.origin, input.translation, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
						 b3FixDiv( B3_FIX( 3.0f ) , B3_FIX( 8.0f ) ), ( 8 * B3_FIXED_EPSILON ) );
}

static int RayCastCapsuleClipTest( void )
{
	// The side hit occurs at fraction 1/3. Straddle it with maxFraction.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 6.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), -B3_FIX( 6.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		ENSURE_SMALL( out.fraction - b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), 8 * B3_FIXED_EPSILON );
	}
	return 0;
}

// A ray within a hair of the capsule axis must still hit when it slowly converges onto the
// surface. The closest point solver is ill conditioned in this band, so this guards the near
// parallel fallback that intersects the infinite cylinder directly.
static int RayCastCapsuleParallelTest( void )
{
	// Capsule along y. A long ray almost parallel to the axis drifts inward from just outside the
	// cylinder and dips through the far endcap. The naive solve loses this hit to a determinant of zero.
	b3Capsule axisY = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 10.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
	{
		b3RayCastInput input = { { B3_FIX( 1.0001f ), B3_FIX( 100.0f ), B3_FIX( 0.0f ) }, { -B3_FIX( 0.001f ), -B3_FIX( 200.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &axisY, &input );
		ENSURE( out.hit );

		// The hit lands on the capsule surface and on the ray.
		b3Vec3 onSeg = b3PointToSegmentDistance( axisY.center1, axisY.center2, out.point );
		ENSURE_SMALL( b3Distance( out.point, onSeg ) - axisY.radius, B3_FIX( 1e-3f ) );
		b3Vec3 onRay = b3MulAdd( input.origin, out.fraction, input.translation );
		ENSURE_SMALL( b3Distance( out.point, onRay ), B3_FIX( 1e-3f ) );
	}

	// Near parallel ray converging onto the x-axis capsule from far away.
	{
		b3RayCastInput input = { { -B3_FIX( 1000.0f ), B3_FIX( 1.0001f ), B3_FIX( 0.0f ) }, { B3_FIX( 2000.0f ), -B3_FIX( 0.001f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3CastOutput out = b3RayCastCapsule( &rayCapsule, &input );
		ENSURE( out.hit );
		b3Vec3 onSeg = b3PointToSegmentDistance( rayCapsule.center1, rayCapsule.center2, out.point );
		ENSURE_SMALL( b3Distance( out.point, onSeg ) - rayCapsule.radius, B3_FIX( 1e-3f ) );
	}

	// Exactly parallel and outside the cylinder still misses.
	{
		b3RayCastInput input = { { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		ENSURE( b3RayCastCapsule( &rayCapsule, &input ).hit == false );
	}

	return 0;
}

// Zero length rays and initial overlap behave the same across the solid shapes. A moving ray
// and a zero length ray that both start inside report the origin with zero fraction, and a zero
// length ray that starts outside misses.
static int RayCastOverlapConventionTest( void )
{
	b3Vec3 zero = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3Vec3 ray = { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };

	// Sphere
	{
		b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
		b3Vec3 inside = { B3_FIX( 0.2f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Vec3 outside = { B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3RayCastInput moving = { inside, ray, B3_FIX( 1.0f ) };
		b3RayCastInput pointInside = { inside, zero, B3_FIX( 1.0f ) };
		b3RayCastInput pointOutside = { outside, zero, B3_FIX( 1.0f ) };
		if ( CheckInitialOverlap( b3RayCastSphere( &s, &moving ), inside ) )
			return 1;
		if ( CheckInitialOverlap( b3RayCastSphere( &s, &pointInside ), inside ) )
			return 1;
		ENSURE( b3RayCastSphere( &s, &pointOutside ).hit == false );
	}

	// Capsule
	{
		b3Vec3 inside = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3Vec3 outside = { B3_FIX( 0.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) };
		b3RayCastInput moving = { inside, ray, B3_FIX( 1.0f ) };
		b3RayCastInput pointInside = { inside, zero, B3_FIX( 1.0f ) };
		b3RayCastInput pointOutside = { outside, zero, B3_FIX( 1.0f ) };
		if ( CheckInitialOverlap( b3RayCastCapsule( &rayCapsule, &moving ), inside ) )
			return 1;
		if ( CheckInitialOverlap( b3RayCastCapsule( &rayCapsule, &pointInside ), inside ) )
			return 1;
		ENSURE( b3RayCastCapsule( &rayCapsule, &pointOutside ).hit == false );
	}

	// Hull
	{
		b3Vec3 inside = { B3_FIX( 0.3f ), B3_FIX( 0.2f ), B3_FIX( 0.1f ) };
		b3Vec3 outside = { B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		b3RayCastInput moving = { inside, ray, B3_FIX( 1.0f ) };
		b3RayCastInput pointInside = { inside, zero, B3_FIX( 1.0f ) };
		b3RayCastInput pointOutside = { outside, zero, B3_FIX( 1.0f ) };
		if ( CheckInitialOverlap( b3RayCastHull( &box.base, &moving ), inside ) )
			return 1;
		if ( CheckInitialOverlap( b3RayCastHull( &box.base, &pointInside ), inside ) )
			return 1;
		ENSURE( b3RayCastHull( &box.base, &pointOutside ).hit == false );
	}

	return 0;
}

// Distance, in double precision, from a single precision hit point to the analytic first
// ray/sphere intersection of the same b3Fixed ray. Isolates the single precision method error:
// the reference carries no b3Fixed rounding, so what remains is purely what the method lost.
static double SphereHitError( b3Sphere shape, b3RayCastInput input, b3Vec3 point )
{
	double r = b3FixToDouble( shape.radius );
	double sx = (double)b3FixToDouble( input.origin.x ) - b3FixToDouble( shape.center.x );
	double sy = (double)b3FixToDouble( input.origin.y ) - b3FixToDouble( shape.center.y );
	double sz = (double)b3FixToDouble( input.origin.z ) - b3FixToDouble( shape.center.z );
	double tx = b3FixToDouble( input.translation.x ), ty = b3FixToDouble( input.translation.y ), tz = b3FixToDouble( input.translation.z );
	double len = sqrt( tx * tx + ty * ty + tz * tz );
	double dx = tx / len, dy = ty / len, dz = tz / len;
	double b = sx * dx + sy * dy + sz * dz;
	double c = sx * sx + sy * sy + sz * sz - r * r;
	double t = -b - sqrt( b * b - c );
	double ex = b3FixToDouble( point.x ) - ( b3FixToDouble( input.origin.x ) + t * dx );
	double ey = b3FixToDouble( point.y ) - ( b3FixToDouble( input.origin.y ) + t * dy );
	double ez = b3FixToDouble( point.z ) - ( b3FixToDouble( input.origin.z ) + t * dz );
	return sqrt( ex * ex + ey * ey + ez * ez );
}

// Same idea for the ray/infinite-cylinder intersection, used where the hit lands on the side.
static double CapsuleHitError( b3Capsule shape, b3RayCastInput input, b3Vec3 point )
{
	double ax = (double)b3FixToDouble( shape.center2.x ) - b3FixToDouble( shape.center1.x );
	double ay = (double)b3FixToDouble( shape.center2.y ) - b3FixToDouble( shape.center1.y );
	double az = (double)b3FixToDouble( shape.center2.z ) - b3FixToDouble( shape.center1.z );
	double alen = sqrt( ax * ax + ay * ay + az * az );
	ax /= alen, ay /= alen, az /= alen;

	double sx = (double)b3FixToDouble( input.origin.x ) - b3FixToDouble( shape.center1.x );
	double sy = (double)b3FixToDouble( input.origin.y ) - b3FixToDouble( shape.center1.y );
	double sz = (double)b3FixToDouble( input.origin.z ) - b3FixToDouble( shape.center1.z );
	double tx = b3FixToDouble( input.translation.x ), ty = b3FixToDouble( input.translation.y ), tz = b3FixToDouble( input.translation.z );
	double tlen = sqrt( tx * tx + ty * ty + tz * tz );
	double dx = tx / tlen, dy = ty / tlen, dz = tz / tlen;

	double sa = sx * ax + sy * ay + sz * az;
	double da = dx * ax + dy * ay + dz * az;
	double spx = sx - sa * ax, spy = sy - sa * ay, spz = sz - sa * az;
	double dpx = dx - da * ax, dpy = dy - da * ay, dpz = dz - da * az;
	double A = dpx * dpx + dpy * dpy + dpz * dpz;
	double B = 2.0 * ( spx * dpx + spy * dpy + spz * dpz );
	double r = b3FixToDouble( shape.radius );
	double C = spx * spx + spy * spy + spz * spz - r * r;
	double tau = ( -B - sqrt( B * B - 4.0 * A * C ) ) / ( 2.0 * A );
	double ex = b3FixToDouble( point.x ) - ( b3FixToDouble( input.origin.x ) + tau * dx );
	double ey = b3FixToDouble( point.y ) - ( b3FixToDouble( input.origin.y ) + tau * dy );
	double ez = b3FixToDouble( point.z ) - ( b3FixToDouble( input.origin.z ) + tau * dz );
	return sqrt( ex * ex + ey * ey + ez * ez );
}

// A miss is the worst possible outcome, so fold it into the error as a large sentinel.
#define RAY_MISS 1.0e30

static int RayCastFarOriginTest( void )
{
	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
	b3Capsule c = { { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };

	// (0,0,1) lies on the unit sphere and on the capsule side. The ray dives in from a far origin
	// along H + D*u for a fan of directions u skewed off the surface normal, so the capsule solve
	// sees a real perpendicular gap rather than a free exact cancellation.
	b3Vec3 h = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
	b3Fixed offsets[] = { -B3_FIX( 0.7f ), B3_FIX( 0.0f ), B3_FIX( 0.7f ) };
	b3Fixed distances[] = { B3_FIX( 1e1f ), B3_FIX( 1e2f ), B3_FIX( 1e3f ), B3_FIX( 1e4f ), B3_FIX( 1e5f ), B3_FIX( 1e6f ), B3_FIX( 1e7f ) };

	double worstSphere[ARRAY_COUNT( distances )];
	double worstCapsule[ARRAY_COUNT( distances )];

	printf( "    worst hit point error over a fan of skew rays, by origin distance:\n" );
	printf( "    %-9s %-13s %-13s\n", "distance", "sphere", "capsule" );
	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		b3Fixed d = distances[i];
		double maxS = 0.0, maxC = 0.0;

		for ( int ia = 0; ia < ARRAY_COUNT( offsets ); ++ia )
		{
			for ( int ib = 0; ib < ARRAY_COUNT( offsets ); ++ib )
			{
				b3Vec3 u = b3Normalize( (b3Vec3){ offsets[ia], offsets[ib], B3_FIX( 1.0f ) } );
				b3Vec3 origin = b3MulAdd( h, d, u );
				b3Vec3 translation = b3MulSV( b3FixMul( -B3_FIX( 2.0f ) , d ), u );
				b3RayCastInput input = { origin, translation, B3_FIX( 1.0f ) };

				b3CastOutput os = b3RayCastSphere( &s, &input );
				b3CastOutput oc = b3RayCastCapsule( &c, &input );

				double errS = os.hit ? SphereHitError( s, input, os.point ) : RAY_MISS;
				double errC = oc.hit ? CapsuleHitError( c, input, oc.point ) : RAY_MISS;

				maxS = errS > maxS ? errS : maxS;
				maxC = errC > maxC ? errC : maxC;
			}
		}

		worstSphere[i] = maxS;
		worstCapsule[i] = maxC;
		printf( "    %-9.0e %-13.3e %-13.3e\n", b3FixToDouble( d ), maxS, maxC );
	}

	// The raw-translation quadratic keeps the error at the fixed-point floor: it grows only
	// linearly with origin distance, error ~ distance * resolution, with no catastrophic loss.
	// This holds out to a hundred thousand units for capsules. Beyond that the shifted 128-bit
	// discriminant loses the bits that decide grazing hits, so those rows are printed for
	// insight but left unasserted rather than baking in the breakdown.
	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		if ( distances[i] < B3_FIX( 1.0e6f ) )
		{
			double floor = 16.0 * b3FixToDouble( distances[i] ) * b3FixToDouble( B3_FIXED_EPSILON ) + 2.0e-6;
			ENSURE( worstSphere[i] < floor );
			ENSURE( worstCapsule[i] < floor );
		}
	}

	// Still a clean sub-meter hit at a million units out.
	ENSURE( worstSphere[5] < 0.5 && worstCapsule[5] < 0.5 );

	return 0;
}

static int RayCastShapeTest( void )
{
	b3RayCastInput input = {
		.origin = { -B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
		.translation = { B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
		.maxFraction = B3_FIX( 1.0f ),
	};

	{
		b3CastOutput output = b3RayCastSphere( &sphere, &input );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.normal.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.normal.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.fraction - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	}

	{
		b3CastOutput output = b3RayCastCapsule( &capsule, &input );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.normal.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.normal.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.fraction - b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 4.0f ) ), 8 * B3_FIXED_EPSILON );
	}

	{
		b3CastOutput output = b3RayCastHull( &box.base, &input );
		ENSURE( output.hit );
		ENSURE_SMALL( output.normal.x + B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.normal.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.normal.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( output.fraction - b3FixDiv( B3_FIX( 3.0f ) , B3_FIX( 8.0f ) ), 8 * B3_FIXED_EPSILON );
	}

	return 0;
}

static int CheckShapeName( b3ShapeId shapeId, const char* expected )
{
	const char* got = b3Shape_GetName( shapeId );
	ENSURE( got != NULL || expected == NULL );
	if ( got == NULL )
	{
		return 0;
	}

	int srcLen = expected != NULL ? (int)strlen( expected ) : 0;
	ENSURE( (int)strlen( got ) == srcLen );
	if ( srcLen > 0 )
	{
		ENSURE( strncmp( got, expected, (size_t)srcLen ) == 0 );
	}
	return 0;
}

// Cover the def path, the setter, over-length truncation, and the NULL clear. A zero cap collapses every
// case to the empty string, which the length-derived check handles without a special branch.
static int ShapeNameTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };

	// Default def leaves the name empty.
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	b3ShapeId shapeId = b3CreateSphereShape( bodyId, &shapeDef, &s );
	if ( CheckShapeName( shapeId, NULL ) )
		return 1;

	// Name carried on the def.
	shapeDef.name = "box";
	b3ShapeId namedId = b3CreateSphereShape( bodyId, &shapeDef, &s );
	if ( CheckShapeName( namedId, "box" ) )
		return 1;

	// Setter overwrites.
	b3Shape_SetName( shapeId, "wheel" );
	if ( CheckShapeName( shapeId, "wheel" ) )
		return 1;

	// Over-length name truncates to the cap.
	b3Shape_SetName( shapeId, "abcdefghijklmnopqrstuvwxyz" );
	if ( CheckShapeName( shapeId, "abcdefghijklmnopqrstuvwxyz" ) )
		return 1;

	// NULL clears the name.
	b3Shape_SetName( shapeId, NULL );
	if ( CheckShapeName( shapeId, NULL ) )
		return 1;

	b3DestroyWorld( worldId );
	return 0;
}

// The event enables moved from separate bools to a shared bit field. Each accessor must touch only its own
// bit, so start with all set and flip them one at a time, checking the rest hold.
static int ShapeFlagsTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere s = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	shapeDef.enableSensorEvents = true;
	shapeDef.enableContactEvents = true;
	shapeDef.enableHitEvents = true;
	shapeDef.enablePreSolveEvents = true;
	b3ShapeId shapeId = b3CreateSphereShape( bodyId, &shapeDef, &s );

	// The def carries every enable through to the shape.
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) );

	// Clearing one leaves the rest set.
	b3Shape_EnableSensorEvents( shapeId, false );
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) );

	// Setting it back does not disturb the others.
	b3Shape_EnableSensorEvents( shapeId, true );
	b3Shape_EnableHitEvents( shapeId, false );
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) );

	// Clear two more and confirm only the remaining bit stays set.
	b3Shape_EnableContactEvents( shapeId, false );
	b3Shape_EnablePreSolveEvents( shapeId, false );
	ENSURE( b3Shape_AreSensorEventsEnabled( shapeId ) );
	ENSURE( b3Shape_AreContactEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_AreHitEventsEnabled( shapeId ) == false );
	ENSURE( b3Shape_ArePreSolveEventsEnabled( shapeId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// b3SurfaceMaterial has tail padding and stored shape materials are serialized raw into world
// snapshots, so material storage must be staged with zeroed padding (b3StageMaterial) or two
// identically-defined worlds produce byte-different snapshots. Fill the def materials' padding
// with a different garbage pattern each pass and require the stored bytes to equal a staged
// reference exactly, through both creation paths (inline single material, multi-material heap
// array) and both setters.
static int ShapeMaterialStagingTest( void )
{
	const uint8_t fills[2] = { 0xAA, 0x55 };
	for ( int pass = 0; pass < 2; ++pass )
	{
		uint8_t fill = fills[pass];

		b3SurfaceMaterial single;
		memset( &single, fill, sizeof( single ) );
		single.friction = B3_FIX( 0.3f );
		single.restitution = B3_FIX( 0.1f );
		single.rollingResistance = B3_FIX( 0.0f );
		single.tangentVelocity = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		single.userMaterialId = 7;
		single.customColor = 0;

		b3SurfaceMaterial multi[2];
		memset( multi, fill, sizeof( multi ) );
		for ( int i = 0; i < 2; ++i )
		{
			multi[i].friction = B3_FIX( 0.5f );
			multi[i].restitution = B3_FIX( 0.0f );
			multi[i].rollingResistance = B3_FIX( 0.0f );
			multi[i].tangentVelocity = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
			multi[i].userMaterialId = (uint64_t)( 10 + i );
			multi[i].customColor = 0;
		}

		b3WorldDef worldDef = b3DefaultWorldDef();
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef singleShapeDef = b3DefaultShapeDef();
		singleShapeDef.materials = &single;
		singleShapeDef.materialCount = 1;
		b3ShapeId singleId = b3CreateSphereShape( bodyId, &singleShapeDef, &sphere );

		b3ShapeDef multiShapeDef = b3DefaultShapeDef();
		multiShapeDef.materials = multi;
		multiShapeDef.materialCount = 2;
		b3ShapeId multiId = b3CreateCapsuleShape( bodyId, &multiShapeDef, &capsule );

		b3World* world = b3GetWorldFromId( worldId );
		const b3Shape* singleShape = world->shapes.data + ( singleId.index1 - 1 );
		const b3Shape* multiShape = world->shapes.data + ( multiId.index1 - 1 );

		b3SurfaceMaterial expected;
		b3StageMaterial( &expected, &single );
		ENSURE( singleShape->materials == NULL && singleShape->materialCount == 1 );
		ENSURE( memcmp( &singleShape->material, &expected, sizeof( expected ) ) == 0 );

		ENSURE( multiShape->materials != NULL && multiShape->materialCount == 2 );
		for ( int i = 0; i < 2; ++i )
		{
			b3StageMaterial( &expected, multi + i );
			ENSURE( memcmp( multiShape->materials + i, &expected, sizeof( expected ) ) == 0 );
		}

		// The setters take the material by value, so the copy's padding is stack garbage too.
		b3SurfaceMaterial set;
		memset( &set, fill, sizeof( set ) );
		set.friction = B3_FIX( 0.6f );
		set.restitution = B3_FIX( 0.2f );
		set.rollingResistance = B3_FIX( 0.0f );
		set.tangentVelocity = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		set.userMaterialId = 20;
		set.customColor = 0;

		b3Shape_SetSurfaceMaterial( singleId, set );
		b3StageMaterial( &expected, &set );
		ENSURE( memcmp( &singleShape->material, &expected, sizeof( expected ) ) == 0 );

		set.userMaterialId = 21;
		b3Shape_SetMeshMaterial( multiId, set, 1 );
		b3StageMaterial( &expected, &set );
		ENSURE( memcmp( multiShape->materials + 1, &expected, sizeof( expected ) ) == 0 );

		b3DestroyWorld( worldId );
	}

	return 0;
}

// Oracle for b3ShapeCast against Box3D's stopping band. Two families:
//   S: sphere (proxy A, radius rA) vs sphere (proxy B, radius rB), both proxied
//      by a single point at the origin; the cast resolves the first fraction where
//      the CORE distance |c0 + t*d| equals target = max(slop, totalRadius - slop).
//   B: box (proxy A, 8 corners, radius 0) vs a point (proxy B, radius rB); the
//      point drops onto the top face and resolves t = (c0.y - hy - target)/(-d.y).
// Inputs are raw Q48.16 integers from a seeded xorshift64. The oracle runs in
// double (libm) so fixed3d and the oracle see identical numbers, only the
// algorithm differs. Hash is FNV-1a 64 over the raw int64 bytes of hit, fraction,
// normal.xyz, point.xyz, iterations, in that order, per accepted case.
#define SCS_FNV_OFFSET UINT64_C( 14695981039346656037 )
#define SCS_FNV_PRIME UINT64_C( 1099511628211 )
#define SCS_SHAPE_CAST_HASH_GOLDEN UINT64_C( 0x5d08b7b17392bd08 )

static uint64_t SCS_Next( uint64_t* state )
{
	uint64_t x = *state;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	*state = x;
	return x;
}

static int64_t SCS_RawRange( uint64_t* state, int64_t lo, int64_t hi )
{
	uint64_t span = (uint64_t)( hi - lo ) + 1;
	return lo + (int64_t)( SCS_Next( state ) % span );
}

static uint64_t SCS_FNVStep( uint64_t hash, uint64_t byte )
{
	return ( hash ^ byte ) * SCS_FNV_PRIME;
}

static uint64_t SCS_FNVValue( uint64_t hash, int64_t value )
{
	uint64_t bits = (uint64_t)value;
	for ( int i = 0; i < 8; ++i )
	{
		hash = SCS_FNVStep( hash, bits & 255 );
		bits >>= 8;
	}
	return hash;
}

static int64_t SCS_BoolAsInt64( bool b )
{
	return b ? 1 : 0;
}

static void SCS_HashOutput( uint64_t* hash, const b3CastOutput* out )
{
	*hash = SCS_FNVValue( *hash, SCS_BoolAsInt64( out->hit ) );
	*hash = SCS_FNVValue( *hash, out->fraction );
	*hash = SCS_FNVValue( *hash, out->normal.x );
	*hash = SCS_FNVValue( *hash, out->normal.y );
	*hash = SCS_FNVValue( *hash, out->normal.z );
	*hash = SCS_FNVValue( *hash, out->point.x );
	*hash = SCS_FNVValue( *hash, out->point.y );
	*hash = SCS_FNVValue( *hash, out->point.z );
	*hash = SCS_FNVValue( *hash, out->iterations );
}

static double SCS_AABBPointDist( double px, double py, double pz, double hx, double hy, double hz )
{
	double qx = fmax( fabs( px ) - hx, 0.0 );
	double qy = fmax( fabs( py ) - hy, 0.0 );
	double qz = fmax( fabs( pz ) - hz, 0.0 );
	return sqrt( qx * qx + qy * qy + qz * qz );
}

static int ShapeCastSphereOracleTest( void )
{
	const double slop = b3FixToDouble( B3_LINEAR_SLOP );
	const double tolerance = 0.25 * slop;
	const double distBand = tolerance + 2.0e-4;
	const double normalTol = 2.0e-3;

	uint64_t hash = SCS_FNV_OFFSET;
	double maxDistErr = 0.0;
	double maxNormalErr = 0.0;

	// Family S: sphere vs sphere.
	{
		uint64_t state = UINT64_C( 0x5EED0004 );
		b3Vec3 origin = { 0, 0, 0 };
		int accepted = 0;
		while ( accepted < 128 )
		{
			b3Fixed rA = SCS_RawRange( &state, 3277, 32768 );
			b3Fixed rB = SCS_RawRange( &state, 3277, 32768 );
			b3Vec3 c0;
			c0.x = SCS_RawRange( &state, -196608, 196608 );
			c0.y = SCS_RawRange( &state, -196608, 196608 );
			c0.z = SCS_RawRange( &state, -196608, 196608 );

			double c0x = b3FixToDouble( c0.x );
			double c0y = b3FixToDouble( c0.y );
			double c0z = b3FixToDouble( c0.z );
			double dc0 = sqrt( c0x * c0x + c0y * c0y + c0z * c0z );
			double rAd = b3FixToDouble( rA );
			double rBd = b3FixToDouble( rB );
			if ( dc0 <= rAd + rBd + 0.1 )
			{
				continue;
			}

			int64_t sum = (int64_t)rA + (int64_t)rB;
			int64_t lim = sum * 3 / 10;
			b3Vec3 aim;
			aim.x = SCS_RawRange( &state, -lim, lim );
			aim.y = SCS_RawRange( &state, -lim, lim );
			aim.z = SCS_RawRange( &state, -lim, lim );
			b3Fixed s = SCS_RawRange( &state, 78643, 131072 );

			b3Vec3 d;
			d.x = b3FixMul( (b3Fixed)( (int64_t)aim.x - (int64_t)c0.x ), s );
			d.y = b3FixMul( (b3Fixed)( (int64_t)aim.y - (int64_t)c0.y ), s );
			d.z = b3FixMul( (b3Fixed)( (int64_t)aim.z - (int64_t)c0.z ), s );

			double totalRadius = rAd + rBd;
			double target = fmax( slop, totalRadius - slop );

			double dx = b3FixToDouble( d.x );
			double dy = b3FixToDouble( d.y );
			double dz = b3FixToDouble( d.z );
			double A = dx * dx + dy * dy + dz * dz;
			if ( A <= 0.0 )
			{
				continue;
			}
			double B = 2.0 * ( c0x * dx + c0y * dy + c0z * dz );
			double C = c0x * c0x + c0y * c0y + c0z * c0z - target * target;
			double disc = B * B - 4.0 * A * C;
			if ( disc < 0.0 )
			{
				continue;
			}
			double sd = sqrt( disc );
			double t1 = ( -B - sd ) / ( 2.0 * A );
			double t2 = ( -B + sd ) / ( 2.0 * A );
			double tbest = 2.0;
			if ( t1 >= 0.0 && t1 <= 1.0 && t1 < tbest )
			{
				tbest = t1;
			}
			if ( t2 >= 0.0 && t2 <= 1.0 && t2 < tbest )
			{
				tbest = t2;
			}
			if ( tbest > 1.0 )
			{
				continue;
			}
			double t = tbest;

			double px = c0x + t * dx;
			double py = c0y + t * dy;
			double pz = c0z + t * dz;
			double pl = sqrt( px * px + py * py + pz * pz );
			if ( pl <= 1.0e-12 )
			{
				continue;
			}
			double nx = px / pl;
			double ny = py / pl;
			double nz = pz / pl;

			b3ShapeCastPairInput input;
			input.proxyA = (b3ShapeProxy){ &origin, 1, rA };
			input.proxyB = (b3ShapeProxy){ &origin, 1, rB };
			input.transform = b3Transform_identity;
			input.transform.p = c0;
			input.translationB = d;
			input.maxFraction = B3_FIX( 1.0f );
			input.canEncroach = false;

			b3CastOutput out = b3ShapeCast( &input );

			ENSURE( out.hit );

			double f = b3FixToDouble( out.fraction );
			double Px = c0x + f * dx;
			double Py = c0y + f * dy;
			double Pz = c0z + f * dz;
			double D = sqrt( Px * Px + Py * Py + Pz * Pz );
			double distErr = fabs( D - target );
			if ( distErr > maxDistErr )
			{
				maxDistErr = distErr;
			}
			ENSURE( distErr <= distBand );

			double onx = b3FixToDouble( out.normal.x );
			double ony = b3FixToDouble( out.normal.y );
			double onz = b3FixToDouble( out.normal.z );
			double normalErr = fmax( fabs( onx - nx ), fmax( fabs( ony - ny ), fabs( onz - nz ) ) );
			if ( normalErr > maxNormalErr )
			{
				maxNormalErr = normalErr;
			}
			ENSURE( normalErr <= normalTol );

			SCS_HashOutput( &hash, &out );
			++accepted;
		}
	}

	// Family B: box vs point onto the top face.
	{
		uint64_t state = UINT64_C( 0x5EED0005 );
		b3Vec3 bp = { 0, 0, 0 };
		int accepted = 0;
		while ( accepted < 128 )
		{
			b3Fixed hx = SCS_RawRange( &state, 13107, 131072 );
			b3Fixed hy = SCS_RawRange( &state, 13107, 131072 );
			b3Fixed hz = SCS_RawRange( &state, 13107, 131072 );
			b3Fixed rB = SCS_RawRange( &state, 3277, 32768 );

			int64_t qx = (int64_t)hx * 4 / 5;
			int64_t qz = (int64_t)hz * 4 / 5;
			b3Fixed aimx = SCS_RawRange( &state, -qx, qx );
			b3Fixed aimz = SCS_RawRange( &state, -qz, qz );
			b3Fixed aimy = hy;

			b3Fixed H = SCS_RawRange( &state, 13107, 131072 );
			int64_t olim = (int64_t)H * 3 / 10;
			b3Fixed ox = SCS_RawRange( &state, -olim, olim );
			b3Fixed oz = SCS_RawRange( &state, -olim, olim );

			b3Vec3 c0;
			c0.x = (b3Fixed)( (int64_t)aimx + (int64_t)ox );
			c0.y = (b3Fixed)( (int64_t)aimy + (int64_t)H );
			c0.z = (b3Fixed)( (int64_t)aimz + (int64_t)oz );

			b3Fixed s = SCS_RawRange( &state, 72090, 98304 );
			b3Vec3 d;
			d.x = b3FixMul( (b3Fixed)( (int64_t)aimx - (int64_t)c0.x ), s );
			d.y = b3FixMul( (b3Fixed)( (int64_t)aimy - (int64_t)c0.y ), s );
			d.z = b3FixMul( (b3Fixed)( (int64_t)aimz - (int64_t)c0.z ), s );

			double hxd = b3FixToDouble( hx );
			double hyd = b3FixToDouble( hy );
			double hzd = b3FixToDouble( hz );
			double rBd = b3FixToDouble( rB );
			double target = fmax( slop, rBd - slop );

			double c0y = b3FixToDouble( c0.y );
			double dy = b3FixToDouble( d.y );
			if ( dy >= 0.0 )
			{
				continue;
			}
			double t = ( c0y - hyd - target ) / ( -dy );
			if ( t < 0.0 || t > 1.0 )
			{
				continue;
			}
			double px = b3FixToDouble( c0.x ) + t * b3FixToDouble( d.x );
			double pz = b3FixToDouble( c0.z ) + t * b3FixToDouble( d.z );
			if ( fabs( px ) > hxd || fabs( pz ) > hzd )
			{
				continue;
			}

			b3Vec3 corners[8];
			corners[0] = (b3Vec3){ hx, hy, hz };
			corners[1] = (b3Vec3){ hx, hy, -hz };
			corners[2] = (b3Vec3){ hx, -hy, hz };
			corners[3] = (b3Vec3){ hx, -hy, -hz };
			corners[4] = (b3Vec3){ -hx, hy, hz };
			corners[5] = (b3Vec3){ -hx, hy, -hz };
			corners[6] = (b3Vec3){ -hx, -hy, hz };
			corners[7] = (b3Vec3){ -hx, -hy, -hz };

			b3ShapeCastPairInput input;
			input.proxyA = (b3ShapeProxy){ corners, 8, 0 };
			input.proxyB = (b3ShapeProxy){ &bp, 1, rB };
			input.transform = b3Transform_identity;
			input.transform.p = c0;
			input.translationB = d;
			input.maxFraction = B3_FIX( 1.0f );
			input.canEncroach = false;

			b3CastOutput out = b3ShapeCast( &input );

			ENSURE( out.hit );

			double f = b3FixToDouble( out.fraction );
			double Px = b3FixToDouble( c0.x ) + f * b3FixToDouble( d.x );
			double Py = b3FixToDouble( c0.y ) + f * b3FixToDouble( d.y );
			double Pz = b3FixToDouble( c0.z ) + f * b3FixToDouble( d.z );
			double D = SCS_AABBPointDist( Px, Py, Pz, hxd, hyd, hzd );
			double distErr = fabs( D - target );
			if ( distErr > maxDistErr )
			{
				maxDistErr = distErr;
			}
			ENSURE( distErr <= distBand );

			double onx = b3FixToDouble( out.normal.x );
			double ony = b3FixToDouble( out.normal.y );
			double onz = b3FixToDouble( out.normal.z );
			double normalErr = fmax( fabs( onx ), fmax( fabs( ony - 1.0 ), fabs( onz ) ) );
			if ( normalErr > maxNormalErr )
			{
				maxNormalErr = normalErr;
			}
			ENSURE( normalErr <= normalTol );

			SCS_HashOutput( &hash, &out );
			++accepted;
		}
	}

	printf( "  ShapeCastSphereOracleTest hash=0x%016llx\n", (unsigned long long)hash );
	printf( "  ShapeCastSphereOracleTest max distance error = %g, max normal error = %g\n", maxDistErr, maxNormalErr );

	ENSURE( hash == SCS_SHAPE_CAST_HASH_GOLDEN );

	return 0;
}

int ShapeTest( void )
{
	box = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) );

	RUN_SUBTEST( ShapeMassTest );
	RUN_SUBTEST( ShapeAABBTest );
	RUN_SUBTEST( ShapeNameTest );
	RUN_SUBTEST( ShapeFlagsTest );
	RUN_SUBTEST( ShapeMaterialStagingTest );
	// RUN_SUBTEST( PointInShapeTest );
	RUN_SUBTEST( RayCastShapeTest );

	RUN_SUBTEST( RayCastSphereHitTest );
	RUN_SUBTEST( RayCastSphereMissTest );
	RUN_SUBTEST( RayCastSphereClipTest );
	RUN_SUBTEST( RayCastSphereInteriorTest );
	RUN_SUBTEST( RayCastSphereGrazeTest );

	RUN_SUBTEST( RayCastCapsuleSideTest );
	RUN_SUBTEST( RayCastCapsuleObliqueTest );
	RUN_SUBTEST( RayCastCapsuleCapTest );
	RUN_SUBTEST( RayCastCapsuleMissTest );
	RUN_SUBTEST( RayCastCapsuleInteriorTest );
	RUN_SUBTEST( RayCastCapsuleDegenerateTest );
	RUN_SUBTEST( RayCastCapsuleClipTest );
	RUN_SUBTEST( RayCastCapsuleParallelTest );

	RUN_SUBTEST( RayCastOverlapConventionTest );
	RUN_SUBTEST( RayCastFarOriginTest );
	RUN_SUBTEST( ShapeCastSphereOracleTest );

	return 0;
}
