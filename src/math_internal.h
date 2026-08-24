// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include "inverse.h"

struct b3Sweep;
struct b3Plane;

#define B3_TWO_PI B3_FIX( 6.283185307f )
#define B3_PI_OVER_TWO B3_FIX( 1.570796327f )
#define B3_PI_OVER_FOUR B3_FIX( 0.785398163f )
#define B3_SQRT3 B3_FIX( 1.732050808f )

// todo eliminate this
static const b3AABB B3_BOUNDS3_EMPTY = { { B3_FIXED_MAX, B3_FIXED_MAX, B3_FIXED_MAX }, { -B3_FIXED_MAX, -B3_FIXED_MAX, -B3_FIXED_MAX } };

typedef struct b3Matrix2
{
	b3Vec2 cx, cy;
} b3Matrix2;

typedef struct b3Triangle
{
	b3Vec3 vertices[3];
	int i1, i2, i3;
	int flags;
} b3Triangle;

typedef struct b3TrianglePoint
{
	b3Vec3 point;
	b3TriangleFeature feature;
} b3TrianglePoint;

typedef struct b3ShapeExtent
{
	b3Fixed minExtent;
	b3Vec3 maxExtent;
} b3ShapeExtent;

b3TrianglePoint b3ClosestPointOnTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 q );

b3Fixed b3IntersectSegmentTriangle( b3Vec3 p, b3Vec3 q, b3Vec3 a, b3Vec3 b, b3Vec3 c );
b3Fixed b3IntersectSegmentSphere( b3Vec3 p, b3Vec3 q, b3Vec3 c, b3Fixed r );

b3MassData b3ComputeMassProperties( int triangleCount, const int* triangles, int vertexCount, const b3Vec3* vertices,
									b3Fixed density );

bool b3IsValidMassData( const b3MassData* massData );

b3Matrix3 b3SphereInertia( b3Fixed mass, b3Fixed radius );
b3Matrix3 b3CylinderInertia( b3Fixed mass, b3Fixed radius, b3Fixed height );
b3Matrix3 b3BoxInertia( b3Fixed mass, b3Vec3 min, b3Vec3 max );

// Inertia helper (Io = Ic + Is and Ic = Io - Is)
int b3GetProxySupport( const b3ShapeProxy* proxy, b3Vec3 axis );
int b3GetPointSupport( const b3Vec3* points, int count, b3Vec3 axis );

static inline size_t b3AlignUp8( size_t x )
{
	return ( x + 7u ) & ~(size_t)7u;
}

// Round up to any power-of-two alignment. Blob section offsets round to the
// _Alignof of the element type they hold: a no-op in the narrow builds (every
// blob type needs <= 8 and offsets are already 8-aligned), real padding in
// ludicrous mode where b3AABB-bearing types (b3TreeNode, b3HullData,
// b3MeshData) need 16.
static inline size_t b3AlignUp( size_t x, size_t align )
{
	return ( x + align - 1 ) & ~( align - 1 );
}

// https://en.wikipedia.org/wiki/Floor_and_ceiling_functions
static inline int b3CeilingInt( int numerator, int denominator )
{
	B3_VALIDATE( denominator > 0 );
	return ( numerator + denominator - 1 ) / denominator;
}

// Assumes denominator == 2^exponent
static inline int b3CeilingPow2( int numerator, int denominator, int exponent )
{
	B3_VALIDATE( exponent > 0 && ( denominator == 1 << exponent ) );
	return ( numerator + denominator - 1 ) >> exponent;
}

bool b3IsSweepNormalized( b3Sweep* sweep );

static inline b3Fixed b3Dot2( b3Vec2 v1, b3Vec2 v2 )
{
	return b3FixMul( v1.x , v2.x ) + b3FixMul( v1.y , v2.y );
}

static inline b3Fixed b3Length2( b3Vec2 v )
{
	return b3FixSqrt( b3Dot2( v, v ) );
}

static inline b3Fixed b3LengthSquared2( b3Vec2 v )
{
	return b3Dot2( v, v );
}

static inline b3Vec2 b3MinVec2( b3Vec2 v1, b3Vec2 v2 )
{
	b3Vec2 v;
	v.x = b3FixMin( v1.x, v2.x );
	v.y = b3FixMin( v1.y, v2.y );
	return v;
}

static inline b3Vec2 b3MaxVec2( b3Vec2 v1, b3Vec2 v2 )
{
	b3Vec2 v;
	v.x = b3FixMax( v1.x, v2.x );
	v.y = b3FixMax( v1.y, v2.y );
	return v;
}

static inline void b3Store( b3Fixed* dst, b3Vec3 src )
{
	dst[0] = src.x;
	dst[1] = src.y;
	dst[2] = src.z;
}

static inline b3Vec3 b3ClampLength( b3Vec3 v, b3Fixed maxLength )
{
	b3Fixed lengthSq = b3LengthSquared( v );
	if ( lengthSq <= b3FixMul( maxLength , maxLength ) )
	{
		return v;
	}

	b3Fixed length = b3FixSqrt( lengthSq );
	return b3MulSV( b3FixDiv( maxLength , length ), v );
}

// Assume v is a unit vector
static inline b3Vec3 b3ArbitraryPerp( b3Vec3 v )
{
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	b3Vec3 p;
	if ( v.x < -B3_FIX( 0.5f ) || B3_FIX( 0.5f ) < v.x )
	{
		// x is non-zero and it should not go into the x component
		// dot([ay + bz, cx, dx], [x, y, z]) = ayx + bzx + cxy + dzx
		// for the dot product to be zero need: c = -a, d = -b
		b3Fixed a = B3_FIX( 0.67f );
		b3Fixed b = -B3_FIX( 0.42f );
		p = B3_LITERAL( b3Vec3 ){ b3FixMul( a , v.y ) + b3FixMul( b , v.z ), b3FixMul( -a , v.x ), b3FixMul( -b , v.x ) };
	}
	else if ( v.y < -B3_FIX( 0.5f ) || B3_FIX( 0.5f ) < v.y )
	{
		// y is non-zero and it should not go into the y component
		// p = [ay, bx + cz, dy]
		// axy + bxy + cyz + dyz = 0
		// b = -a, d = -c
		b3Fixed a = B3_FIX( 0.67f );
		b3Fixed c = -B3_FIX( 0.42f );
		p = B3_LITERAL( b3Vec3 ){ b3FixMul( a , v.y ), b3FixMul( -a , v.x ) + b3FixMul( c , v.z ), b3FixMul( -c , v.y ) };
	}
	else
	{
		// This would trip if the input is not a unit vector
		B3_VALIDATE( v.z < -B3_FIX( 0.5f ) || B3_FIX( 0.5f ) < v.z );

		// z is non-zero and it should not go into the z component
		// p = [az, bz, cx + dy]
		// axz + byz + cxz + dyz = 0
		// c = -a, d = -b
		b3Fixed a = B3_FIX( 0.67f );
		b3Fixed b = -B3_FIX( 0.42f );
		p = B3_LITERAL( b3Vec3 ){ b3FixMul( a , v.z ), b3FixMul( b , v.z ), b3FixMul( -a , v.x ) - b3FixMul( b , v.y ) };
	}

	B3_VALIDATE( b3LengthSquared( p ) > B3_FIX( 0.1f ) );
	B3_VALIDATE( b3FixAbs( b3Dot( p, v ) ) < 100 * B3_FIXED_EPSILON );

	return b3Normalize( p );
}

static inline b3Quat b3QuatFromExponentialMap( b3Vec3 v )
{
	// Exponential map (Grassia)
	b3Fixed threshold = B3_FIX( 0.018581361f );

	b3Fixed angle = b3Length( v );
	if ( angle < threshold )
	{
		// Taylor expansion
		b3Quat out;
		out.v = b3MulSV( B3_FIX( 0.5f ) + b3FixDiv( b3FixMul( angle , angle ) , B3_FIX( 48.0f ) ), v );
		out.s = b3Cos( b3FixMul( B3_FIX( 0.5f ) , angle ) );

		return out;
	}

	return b3MakeQuatFromAxisAngle( b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , angle ), v ), angle );
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaRotation the angular displacement vector in radians (angular velocity multiplied by the time step)
/// q2 = q1 + 0.5 * omega * q1
static inline b3Quat b3IntegrateRotation( b3Quat q1, b3Vec3 deltaRotation )
{
#if 1
	// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
	b3Quat qd = { b3MulSV( B3_FIX( 0.5f ), deltaRotation ), B3_FIX( 0.0f ) };
	qd = b3MulQuat( qd, q1 );
	b3Quat q2 = { b3Add( q1.v, qd.v ), qd.s + q1.s };
	q2 = b3NormalizeQuat( q2 );
	return q2;
#else
	return b3NormalizeQuat( b3MulQuat(b3QuatFromExponentialMap( deltaRotation ), q1) );
#endif
}

// Pseudo angular velocity from a quaternion target
// w = 2 * (target - q) * conj(q)
static inline b3Vec3 b3DeltaQuatToRotation( b3Quat q, b3Quat target )
{
	b3Quat s = q;
	if ( b3DotQuat( q, target ) < B3_FIX( 0.0f ) )
	{
		// Correct polarity
		s = b3NegateQuat( q );
	}

	b3Quat diff = { b3Sub( target.v, s.v ), target.s - s.s };
	b3Quat product = b3MulQuat( diff, b3Conjugate( s ) );
	return b3MulSV( B3_FIX( 2.0f ), product.v );
}

static inline b3Fixed b3ScalarTripleProduct( b3Vec3 a, b3Vec3 b, b3Vec3 c )
{
	b3Vec3 d;
	d.x = b3FixMul( b.y , c.z ) - b3FixMul( b.z , c.y );
	d.y = b3FixMul( b.z , c.x ) - b3FixMul( b.x , c.z );
	d.z = b3FixMul( b.x , c.y ) - b3FixMul( b.y , c.x );
	return b3FixMul( a.x , d.x ) + b3FixMul( a.y , d.y ) + b3FixMul( a.z , d.z );
}

// Get a value by index. Avoid undefined behavior of code like (&v.x)[2].
static inline b3Fixed b3GetByIndex( b3Vec3 v, int index )
{
	B3_VALIDATE( 0 <= index && index < 3 );
	b3Fixed temp[3] = { v.x, v.y, v.z };
	return temp[index];
}

static inline int b3MajorAxis( b3Vec3 v )
{
	return v.x < v.y ? ( v.y < v.z ? 2 : 1 ) : ( v.x < v.z ? 2 : 0 );
}

static inline b3Fixed b3MinElement( b3Vec3 v )
{
	return b3FixMin( v.x, b3FixMin( v.y, v.z ) );
}

static inline b3Fixed b3MaxElement( b3Vec3 v )
{
	return b3FixMax( v.x, b3FixMax( v.y, v.z ) );
}

static inline int b3MaxElementIndex( b3Vec3 v )
{
	return v.x < v.y ? ( v.y < v.z ? 2 : 1 ) : ( v.x < v.z ? 2 : 0 );
}

static inline b3Vec2 b3Add2( b3Vec2 a, b3Vec2 b )
{
	b3Vec2 c = { a.x + b.x, a.y + b.y };
	return c;
}

static inline b3Vec2 b3Sub2( b3Vec2 a, b3Vec2 b )
{
	b3Vec2 c = { a.x - b.x, a.y - b.y };
	return c;
}

static inline b3Vec2 b3Neg2( b3Vec2 v )
{
	b3Vec2 c = { -v.x, -v.y };
	return c;
}

static inline b3Vec2 b3MulSV2( b3Fixed s, b3Vec2 v )
{
	b3Vec2 c = { b3FixMul( s , v.x ), b3FixMul( s , v.y ) };
	return c;
}

// a + s * b
static inline b3Vec2 b3MulAdd2( b3Vec2 a, b3Fixed s, b3Vec2 b )
{
	b3Vec2 c = { a.x + b3FixMul( s , b.x ), a.y + b3FixMul( s , b.y ) };
	return c;
}

// a - s * b
static inline b3Vec2 b3MulSub2( b3Vec2 a, b3Fixed s, b3Vec2 b )
{
	b3Vec2 c = { a.x - b3FixMul( s , b.x ), a.y - b3FixMul( s , b.y ) };
	return c;
}

static inline b3Fixed b3Cross2( b3Vec2 a, b3Vec2 b )
{
	return b3FixMul( a.x , b.y ) - b3FixMul( a.y , b.x );
}

static inline b3Fixed b3DistanceSquared2( b3Vec2 a, b3Vec2 b )
{
	b3Fixed dx = b.x - a.x;
	b3Fixed dy = b.y - a.y;
	return b3FixMul( dx , dx ) + b3FixMul( dy , dy );
}

static inline b3Vec2 b3MulMV2( b3Matrix2 m, b3Vec2 a )
{
	b3Vec2 b = { b3FixMul( m.cx.x , a.x ) + b3FixMul( m.cy.x , a.y ), b3FixMul( m.cx.y , a.x ) + b3FixMul( m.cy.y , a.y ) };
	return b;
}

static inline b3Matrix2 b3MulMM2( b3Matrix2 m1, b3Matrix2 m2 )
{
	b3Matrix2 out;
	out.cx = b3MulMV2( m1, m2.cx );
	out.cy = b3MulMV2( m1, m2.cy );
	return out;
}

static inline b3Fixed b3Det2( b3Matrix2 m )
{
	return b3FixMul( m.cx.x , m.cy.y ) - b3FixMul( m.cx.y , m.cy.x );
}

static inline b3Matrix2 b3Invert2( b3Matrix2 m )
{
	// The determinant of a matrix with small entries underflows Q48.16, so
	// compute it and the division at full 128-bit precision.
	b3Int128 det = (b3Int128)m.cx.x * m.cy.y - (b3Int128)m.cx.y * m.cy.x; // Q32.32 in 128 bits
	if ( det != 0 )
	{
		return B3_LITERAL( b3Matrix2 ){
			{ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( (b3Int128)m.cy.y, 32 ), det ),
			  (b3Fixed)b3Int128Div( b3Int128ShiftLeft( -(b3Int128)m.cx.y, 32 ), det ) },
			{ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( -(b3Int128)m.cy.x, 32 ), det ),
			  (b3Fixed)b3Int128Div( b3Int128ShiftLeft( (b3Int128)m.cx.x, 32 ), det ) },
		};
	}

	return B3_LITERAL( b3Matrix2 ){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ) } };
}

/// A 2x2 inverse across the two scales: the tangent (friction) mass, built from an
/// inverse-scaled k and used as an ordinary mass. Same shift as the 3x3 for the same
/// reason, and 128 bits is enough here because a 2x2 determinant is two products rather
/// than a cofactor against a third entry.
static inline b3Matrix2 b3Invert2AcrossScales( b3Matrix2 m )
{
	b3Int128 det = fixInt128Sub( fixInt128MulI64( m.cx.x, m.cy.y ), fixInt128MulI64( m.cx.y, m.cy.x ) );
	if ( fixInt128Eq( det, FIX_INT128_ZERO ) )
	{
		return B3_LITERAL( b3Matrix2 ){ { B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ) } };
	}

	const int shift = B3_FIXED_FRACTION_BITS + B3_INVERSE_FRACTION_BITS;

	return B3_LITERAL( b3Matrix2 ){
		{ fixDivShifted( fixInt128FromI64( m.cy.y ), shift, det ), fixDivShifted( fixInt128FromI64( -m.cx.y ), shift, det ) },
		{ fixDivShifted( fixInt128FromI64( -m.cy.x ), shift, det ), fixDivShifted( fixInt128FromI64( m.cx.x ), shift, det ) },
	};
}

// Assumes positive semi-definite
static inline b3Vec2 b3Solve2( b3Matrix2 m, b3Vec2 b )
{
	// 128-bit determinant and division, see b3Invert2
	b3Int128 det = (b3Int128)m.cx.x * m.cy.y - (b3Int128)m.cx.y * m.cy.x; // Q32.32 in 128 bits
	if ( det > 0 )
	{
		b3Int128 nx = (b3Int128)m.cy.y * b.x - (b3Int128)m.cy.x * b.y; // Q32.32
		b3Int128 ny = (b3Int128)m.cx.x * b.y - (b3Int128)m.cx.y * b.x;
		return B3_LITERAL( b3Vec2 ){
			(b3Fixed)b3Int128Div( b3Int128ShiftLeft( nx, 16 ), det ),
			(b3Fixed)b3Int128Div( b3Int128ShiftLeft( ny, 16 ), det ),
		};
	}

	return B3_LITERAL( b3Vec2 ){ B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
}

// Convenience function: s * a + t * b + u * c
static inline b3Vec3 b3Blend3( b3Fixed s, b3Vec3 a, b3Fixed t, b3Vec3 b, b3Fixed u, b3Vec3 c )
{
	b3Vec3 d = {
		b3FixMul( s , a.x ) + b3FixMul( t , b.x ) + b3FixMul( u , c.x ),
		b3FixMul( s , a.y ) + b3FixMul( t , b.y ) + b3FixMul( u , c.y ),
		b3FixMul( s , a.z ) + b3FixMul( t , b.z ) + b3FixMul( u , c.z ),
	};
	return d;
}

static inline b3Vec3 b3ModifiedCross( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 c;
	c.x = b3FixMul( a.y , b.z ) + b3FixMul( a.z , b.y );
	c.y = b3FixMul( a.z , b.x ) + b3FixMul( a.x , b.z );
	c.z = b3FixMul( a.x , b.y ) + b3FixMul( a.y , b.x );
	return c;
}

static inline b3Matrix3 b3MakeDiagonalMatrix( b3Fixed a, b3Fixed b, b3Fixed c )
{
	return (b3Matrix3){ { a, B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), b, B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), c } };
}

static inline b3Matrix3 b3Skew( b3Vec3 v )
{
	b3Matrix3 out;
	out.cx = (b3Vec3){ b3FixFromInt( 0 ), v.z, -v.y };
	out.cy = (b3Vec3){ -v.z, b3FixFromInt( 0 ), v.x };
	out.cz = (b3Vec3){ v.y, -v.x, b3FixFromInt( 0 ) };

	return out;
}

static inline b3Plane b3NormalizePlane( b3Plane plane )
{
	b3Fixed invLength = b3FixDiv( B3_FIX( 1.0f ) , b3Length( plane.normal ) );
	return (b3Plane){ b3MulSV( invLength, plane.normal ), b3FixMul( invLength , plane.offset ) };
}

static inline b3Plane b3MakePlaneFromNormalAndPoint( b3Vec3 normal, b3Vec3 point )
{
	return (b3Plane){ normal, b3Dot( normal, point ) };
}

static inline b3Plane b3MakePlaneFromPoints( b3Vec3 point1, b3Vec3 point2, b3Vec3 point3 )
{
	b3Plane plane;
	plane.normal = b3Cross( b3Sub( point2, point1 ), b3Sub( point3, point1 ) );
	plane.normal = b3Normalize( plane.normal );
	plane.offset = b3Dot( plane.normal, point1 );
	return plane;
}

static inline b3Vec3 b3MakeNormalFromPoints( b3Vec3 point1, b3Vec3 point2, b3Vec3 point3 )
{
	b3Vec3 normal = b3Cross( b3Sub( point2, point1 ), b3Sub( point3, point1 ) );
	return b3Normalize( normal );
}

// normal2 = q * normal1
// offset2 = dot(normal2, p) + offset1
static inline b3Plane b3TransformPlane( b3Transform transform, b3Plane plane )
{
	b3Vec3 normal = b3RotateVector( transform.q, plane.normal );
	return B3_LITERAL( b3Plane ){ normal, plane.offset + b3Dot( normal, transform.p ) };
}

/// Signed separation of a point from a plane
static inline b3Fixed b3PlaneSeparation( b3Plane plane, b3Vec3 point )
{
	return b3Dot( plane.normal, point ) - plane.offset;
}

// Negative if p is below the triangle v1-v2-v3
static inline b3Fixed b3SignedVolume( b3Vec3 v1, b3Vec3 v2, b3Vec3 v3, b3Vec3 p )
{
	b3Vec3 e1 = b3Sub( v2, v1 );
	b3Vec3 e2 = b3Sub( v3, v1 );
	b3Vec3 n = b3Cross( e1, e2 );
	return b3Dot( n, b3Sub( p, v1 ) );
}

// todo eliminate this
static inline bool b3IsWithinSegments( const b3SegmentDistanceResult* result )
{
	return ( B3_FIX( 0.0f ) <= result->fraction1 && result->fraction1 <= B3_FIX( 1.0f ) ) &&
		   ( B3_FIX( 0.0f ) <= result->fraction2 && result->fraction2 <= B3_FIX( 1.0f ) );
}

static inline b3Matrix3 b3RotateInertia( b3Quat q, b3Matrix3 centralInertia )
{
	b3Matrix3 rotationMatrix = b3MakeMatrixFromQuat( q );
	b3Matrix3 inertia = b3MulMM( rotationMatrix, b3MulMM( centralInertia, b3Transpose( rotationMatrix ) ) );
	return inertia;
}

static inline b3Matrix3 b3TransformInertia( b3Transform transform, b3Matrix3 centralInertia, b3Fixed mass )
{
	b3Matrix3 inertia = b3RotateInertia( transform.q, centralInertia );
	inertia = b3AddMM( inertia, b3Steiner( mass, transform.p ) );
	return inertia;
}

// Add a point to an AABB.
static inline b3AABB b3AABB_AddPoint( b3AABB a, b3Vec3 point )
{
#if defined( BOX3D_LUDICROUS_MODE )
	b3AABB out = a;
	out.lowerBound.x = a.lowerBound.x < point.x ? a.lowerBound.x : point.x;
	out.lowerBound.y = a.lowerBound.y < point.y ? a.lowerBound.y : point.y;
	out.lowerBound.z = a.lowerBound.z < point.z ? a.lowerBound.z : point.z;
	out.upperBound.x = a.upperBound.x > point.x ? a.upperBound.x : point.x;
	out.upperBound.y = a.upperBound.y > point.y ? a.upperBound.y : point.y;
	out.upperBound.z = a.upperBound.z > point.z ? a.upperBound.z : point.z;
	return out;
#else
	return (b3AABB){ b3Min( a.lowerBound, point ), b3Max( a.upperBound, point ) };
#endif
}
