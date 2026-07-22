// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT

#include "box3d/math_functions.h"

#include "math_internal.h"

#include "box3d/collision.h"
#include "box3d/constants.h"

#include <math.h>
#include <string.h>

bool b3IsValidFixed( b3Fixed a )
{
	// Fixed point has no NaN, and the saturation values are legal quantities:
	// B3_FIXED_MAX plays the role float's FLT_MAX did (joint thresholds and
	// spring force limits default to it, mirroring isfinite( FLT_MAX ) == true).
	// Only INT64_MIN is unrepresentable (reserved so negation cannot overflow).
	return a != INT64_MIN;
}

bool b3IsValidVec3( b3Vec3 a )
{
	return b3IsValidFixed( a.x ) && b3IsValidFixed( a.y ) && b3IsValidFixed( a.z );
}

bool b3IsValidQuat( b3Quat a )
{
	if ( !b3IsValidFixed( a.v.x ) || !b3IsValidFixed( a.v.y ) || !b3IsValidFixed( a.v.z ) || !b3IsValidFixed( a.s ) )
	{
		return false;
	}

	return b3IsNormalizedQuat( a );
}

bool b3IsValidTransform( b3Transform a )
{
	return b3IsValidVec3( a.p ) && b3IsValidQuat( a.q );
}

bool b3IsValidMatrix3( b3Matrix3 a )
{
	return b3IsValidVec3( a.cx ) && b3IsValidVec3( a.cy ) && b3IsValidVec3( a.cz );
}

bool b3IsValidAABB( b3AABB a )
{
	if ( b3IsValidVec3( a.lowerBound ) == false )
	{
		return false;
	}

	if ( b3IsValidVec3( a.upperBound ) == false )
	{
		return false;
	}

	if ( a.lowerBound.x > a.upperBound.x )
	{
		return false;
	}

	if ( a.lowerBound.y > a.upperBound.y )
	{
		return false;
	}

	if ( a.lowerBound.z > a.upperBound.z )
	{
		return false;
	}

	return true;
}

bool b3IsBoundedAABB( b3AABB a )
{
	if ( a.lowerBound.x < -B3_HUGE || a.lowerBound.y < -B3_HUGE || a.lowerBound.z < -B3_HUGE )
	{
		return false;
	}

	if ( a.upperBound.x > B3_HUGE || a.upperBound.y > B3_HUGE || a.upperBound.z > B3_HUGE )
	{
		return false;
	}

	return true;
}

bool b3IsSaneAABB( b3AABB a )
{
	if ( b3IsValidAABB( a ) == false )
	{
		return false;
	}

	if ( a.lowerBound.x < -B3_HUGE || a.lowerBound.y < -B3_HUGE || a.lowerBound.z < -B3_HUGE )
	{
		return false;
	}

	if ( a.upperBound.x > B3_HUGE || a.upperBound.y > B3_HUGE || a.upperBound.z > B3_HUGE )
	{
		return false;
	}

	return true;
}

bool b3IsValidPlane( b3Plane a )
{
	if ( b3IsValidVec3( a.normal ) == false )
	{
		return false;
	}

	if ( b3IsNormalized( a.normal ) == false )
	{
		return false;
	}

	return b3IsValidFixed( a.offset );
}

bool b3IsValidPosition( b3Pos p )
{
	return b3IsValidVec3( p );
}

bool b3IsValidWorldTransform( b3WorldTransform t )
{
	return b3IsValidPosition( t.p ) && b3IsValidQuat( t.q );
}

// The deterministic transcendentals (b3Atan2, b3ComputeCosSin) and their Q32.32 helpers
// now live in the vendored `fixed` library (extern/fixed/src/fixed_math.c), compiled into
// box3d below. Only the physics math that CONSUMES them remains in this file.

// b3MakeQuatFromMatrix and b3ComputeQuatBetweenUnitVectors now live in the vendored
// `fixed` library (extern/fixed/src/fixed_vec.c), compiled into box3d.

b3SegmentDistanceResult b3LineDistance( b3Vec3 p1, b3Vec3 d1, b3Vec3 p2, b3Vec3 d2 )
{
	b3SegmentDistanceResult result;

	// Solve A*x = b
	b3Fixed a11 = b3Dot( d1, d1 );
	b3Fixed a12 = -b3Dot( d1, d2 );
	b3Fixed a21 = b3Dot( d2, d1 );
	b3Fixed a22 = -b3Dot( d2, d2 );

	b3Vec3 w = b3Sub( p1, p2 );
	b3Fixed b1 = -b3Dot( d1, w );
	b3Fixed b2 = -b3Dot( d2, w );

	b3Fixed det = b3FixMul( a11 , a22 ) - b3FixMul( a12 , a21 );
	if ( det == 0 )
	{
		// Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
		b3Fixed s1 = b3FixDiv( b3Dot( b3Sub( p2, p1 ), d1 ) , b3Dot( d1, d1 ) );
		b3Fixed s2 = B3_FIX( 0.0f );

		result.point1 = b3MulAdd( p1, s1, d1 );
		result.fraction1 = s1;
		result.point2 = b3MulAdd( p2, s2, d2 );
		result.fraction2 = s2;

		return result;
	}

	b3Fixed s1 = b3FixDiv( ( b3FixMul( a22 , b1 ) - b3FixMul( a12 , b2 ) ) , det );
	b3Fixed s2 = b3FixDiv( ( b3FixMul( a11 , b2 ) - b3FixMul( a21 , b1 ) ) , det );

	result.point1 = b3MulAdd( p1, s1, d1 );
	result.fraction1 = s1;
	result.point2 = b3MulAdd( p2, s2, d2 );
	result.fraction2 = s2;
	return result;
}

b3SegmentDistanceResult b3SegmentDistance( b3Vec3 p1, b3Vec3 q1, b3Vec3 p2, b3Vec3 q2 )
{
	b3SegmentDistanceResult result;

	b3Vec3 d1 = b3Sub( q1, p1 );
	b3Vec3 d2 = b3Sub( q2, p2 );
	b3Vec3 r = b3Sub( p1, p2 );

	b3Fixed a = b3Dot( d1, d1 );
	b3Fixed b = b3Dot( d1, d2 );
	b3Fixed c = b3Dot( d1, r );
	b3Fixed e = b3Dot( d2, d2 );
	b3Fixed f = b3Dot( d2, r );

	// Check if one of the segments degenerates into a point
	if ( a < 100 * B3_FIXED_EPSILON && e < 100 * B3_FIXED_EPSILON )
	{
		// Both segments degenerate into points
		result.point1 = p1;
		result.fraction1 = B3_FIX( 0.0f );
		result.point2 = p2;
		result.fraction2 = B3_FIX( 0.0f );

		return result;
	}

	if ( a < 100 * B3_FIXED_EPSILON )
	{
		// First segment degenerates into a point
		b3Fixed s2 = b3FixClamp( b3FixDiv( f , e ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );

		result.point1 = p1;
		result.fraction1 = B3_FIX( 0.0f );
		result.point2 = b3MulAdd( p2, s2, d2 );
		result.fraction2 = s2;

		return result;
	}

	if ( e < 100 * B3_FIXED_EPSILON )
	{
		// Second segment degenerates into a point
		b3Fixed s1 = b3FixClamp( b3FixDiv( -c , a ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );

		result.point1 = b3MulAdd( p1, s1, d1 );
		result.fraction1 = s1;
		result.point2 = p2;
		result.fraction2 = B3_FIX( 0.0f );

		return result;
	}

	// Non-degenerate case
	b3Fixed denom = b3FixMul( a , e ) - b3FixMul( b , b );
	b3Fixed s1 = denom > 0 ? b3FixClamp( b3FixDiv( ( b3FixMul( b , f ) - b3FixMul( c , e ) ) , denom ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) ) : B3_FIX( 0.0f );
	b3Fixed s2 = b3FixDiv( ( b3FixMul( b , s1 ) + f ) , e );

	// Clamp lambda2 and recompute lambda1 if necessary
	if ( s2 < B3_FIX( 0.0f ) )
	{
		s1 = b3FixClamp( b3FixDiv( -c , a ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
		s2 = B3_FIX( 0.0f );
	}
	else if ( s2 > B3_FIX( 1.0f ) )
	{
		s1 = b3FixClamp( b3FixDiv( ( b - c ) , a ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
		s2 = B3_FIX( 1.0f );
	}

	result.point1 = b3MulAdd( p1, s1, d1 );
	result.fraction1 = s1;
	result.point2 = b3MulAdd( p2, s2, d2 );
	result.fraction2 = s2;

	return result;
}

b3Vec3 b3PointToSegmentDistance( b3Vec3 a, b3Vec3 b, b3Vec3 q )
{
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 aq = b3Sub( q, a );

	b3Fixed alpha = b3Dot( ab, aq );

	if ( alpha <= B3_FIX( 0.0f ) )
	{
		// q projects outside interval [a, b] on the side of a
		return a;
	}
	else
	{
		b3Fixed denominator = b3Dot( ab, ab );
		if ( alpha > denominator )
		{
			// q projects outside interval [a, b] on the side of b
			return b;
		}
		else
		{
			// q projects inside interval [a, b]
			alpha = b3FixDiv( alpha, denominator );
			return b3MulAdd( a, alpha, ab );
		}
	}
}

b3TrianglePoint b3ClosestPointOnTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 q )
{
	// Check if P lies in vertex region of A
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 ac = b3Sub( c, a );
	b3Vec3 aq = b3Sub( q, a );

	b3Fixed d1 = b3Dot( ab, aq );
	b3Fixed d2 = b3Dot( ac, aq );
	if ( d1 <= B3_FIX( 0.0f ) && d2 <= B3_FIX( 0.0f ) )
	{
		return (b3TrianglePoint){ a, b3_featureVertex1 };
	}

	// Check if P lies in vertex region of B
	b3Vec3 bq = b3Sub( q, b );

	b3Fixed d3 = b3Dot( ab, bq );
	b3Fixed d4 = b3Dot( ac, bq );
	if ( d3 > B3_FIX( 0.0f ) && d4 <= d3 )
	{
		return (b3TrianglePoint){ b, b3_featureVertex2 };
	}

	// Check if P lies in edge region AB
	b3Fixed vc = b3FixMul( d1 , d4 ) - b3FixMul( d3 , d2 );
	if ( vc <= B3_FIX( 0.0f ) && d1 >= B3_FIX( 0.0f ) && d3 <= B3_FIX( 0.0f ) )
	{
		b3Fixed t = b3FixDiv( d1 , ( d1 - d3 ) );
		return (b3TrianglePoint){ b3MulAdd( a, t, ab ), b3_featureEdge1 };
	}

	// Check if P lies in vertex region of C
	b3Vec3 cq = b3Sub( q, c );

	b3Fixed d5 = b3Dot( ab, cq );
	b3Fixed d6 = b3Dot( ac, cq );
	if ( d6 >= B3_FIX( 0.0f ) && d5 <= d6 )
	{
		return (b3TrianglePoint){ c, b3_featureVertex3 };
	}

	// Check if P lies in edge region AC
	b3Fixed vb = b3FixMul( d5 , d2 ) - b3FixMul( d1 , d6 );
	if ( vb <= B3_FIX( 0.0f ) && d2 >= B3_FIX( 0.0f ) && d6 <= B3_FIX( 0.0f ) )
	{
		b3Fixed t = b3FixDiv( d2 , ( d2 - d6 ) );
		return (b3TrianglePoint){ b3MulAdd( a, t, ac ), b3_featureEdge3 };
	}

	// Check if P lies in edge region of BC
	b3Fixed va = b3FixMul( d3 , d6 ) - b3FixMul( d5 , d4 );
	if ( va <= B3_FIX( 0.0f ) && d4 >= d3 && d5 >= d6 )
	{
		b3Vec3 bc = b3Sub( c, b );

		b3Fixed t = b3FixDiv( ( d4 - d3 ) , ( ( d4 - d3 ) + ( d5 - d6 ) ) );
		return (b3TrianglePoint){ b3MulAdd( b, t, bc ), b3_featureEdge2 };
	}

	// P inside face region ABC
	b3Fixed t1 = b3FixDiv( vb , ( va + vb + vc ) );
	b3Fixed t2 = b3FixDiv( vc , ( va + vb + vc ) );

	b3Vec3 p = b3MulAdd( a, t1, ab );
	p = b3MulAdd( p, t2, ac );
	return (b3TrianglePoint){ p, b3_featureTriangleFace };
}

b3Matrix3 b3SphereInertia( b3Fixed mass, b3Fixed radius )
{
	b3Fixed i = b3FixMul( b3FixMul( b3FixMul( B3_FIX( 0.4f ) , mass ) , radius ) , radius );
	return b3MakeDiagonalMatrix( i, i, i );
}

b3Matrix3 b3CylinderInertia( b3Fixed mass, b3Fixed radius, b3Fixed height )
{
	b3Fixed ixx = b3FixDiv( b3FixMul( mass , ( b3FixMul( b3FixMul( b3FixFromInt( 3 ) , radius ) , radius ) + b3FixMul( height , height ) ) ) , B3_FIX( 12.0f ) );
	b3Fixed iyy = b3FixMul( b3FixMul( b3FixMul( B3_FIX( 0.5f ) , mass ) , radius ) , radius );
	return b3MakeDiagonalMatrix( ixx, iyy, ixx );
}

b3Matrix3 b3BoxInertia( b3Fixed mass, b3Vec3 min, b3Vec3 max )
{
	b3Vec3 delta = b3Sub( max, min );
	b3Fixed ixx = b3FixDiv( b3FixMul( mass , ( b3FixMul( delta.y , delta.y ) + b3FixMul( delta.z , delta.z ) ) ) , B3_FIX( 12.0f ) );
	b3Fixed iyy = b3FixDiv( b3FixMul( mass , ( b3FixMul( delta.x , delta.x ) + b3FixMul( delta.z , delta.z ) ) ) , B3_FIX( 12.0f ) );
	b3Fixed izz = b3FixDiv( b3FixMul( mass , ( b3FixMul( delta.x , delta.x ) + b3FixMul( delta.y , delta.y ) ) ) , B3_FIX( 12.0f ) );

	return b3MakeDiagonalMatrix( ixx, iyy, izz );
}

// https://en.wikipedia.org/wiki/Parallel_axis_theorem
b3Matrix3 b3Steiner( b3Fixed mass, b3Vec3 origin )
{
	// Usage: Io = Ic + Is and Ic = Io - Is
	b3Fixed ixx = b3FixMul( mass , ( b3FixMul( origin.y , origin.y ) + b3FixMul( origin.z , origin.z ) ) );
	b3Fixed iyy = b3FixMul( mass , ( b3FixMul( origin.x , origin.x ) + b3FixMul( origin.z , origin.z ) ) );
	b3Fixed izz = b3FixMul( mass , ( b3FixMul( origin.x , origin.x ) + b3FixMul( origin.y , origin.y ) ) );
	b3Fixed ixy = b3FixMul( b3FixMul( -mass , origin.x ) , origin.y );
	b3Fixed ixz = b3FixMul( b3FixMul( -mass , origin.x ) , origin.z );
	b3Fixed iyz = b3FixMul( b3FixMul( -mass , origin.y ) , origin.z );

	// Write
	b3Matrix3 out;
	out.cx.x = ixx;
	out.cy.x = ixy;
	out.cz.x = ixz;
	out.cx.y = ixy;
	out.cy.y = iyy;
	out.cz.y = iyz;
	out.cx.z = ixz;
	out.cy.z = iyz;
	out.cz.z = izz;

	return out;
}

bool b3IsValidRay( const b3RayCastInput* input )
{
	bool isValid = b3IsValidVec3( input->origin ) && b3IsValidVec3( input->translation ) &&
				   b3IsValidFixed( input->maxFraction ) && B3_FIX( 0.0f ) <= input->maxFraction && input->maxFraction < B3_HUGE;
	return isValid;
}
