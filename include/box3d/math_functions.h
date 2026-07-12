// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "fixed.h"

#include <stdbool.h>

/**
 * @defgroup math Math
 * @brief Vector math types and functions
 * @{
 */

/// https://en.wikipedia.org/wiki/Pi
#define B3_PI B3_FIX( 3.14159265359f )

/// Convenience macro to convert from degrees to radians.
#define B3_DEG_TO_RAD B3_FIX( 0.01745329251f )

/// Convenience macro to convert from radians to degrees.
#define B3_RAD_TO_DEG B3_FIX( 57.2957795131f )

/// Minimum scale used for scaling collision meshes, etc.
#define B3_MIN_SCALE B3_FIX( 0.01f )

/// A 2D vector.
typedef struct b3Vec2
{
	b3Fixed x;
	b3Fixed y;
} b3Vec2;

/// A 3D vector.
typedef struct b3Vec3
{
	b3Fixed x;
	b3Fixed y;
	b3Fixed z;
} b3Vec3;

/// Cosine and sine pair.
/// This uses a custom implementation designed for cross-platform determinism.
typedef struct b3CosSin
{
	/// cosine and sine
	b3Fixed cosine;
	b3Fixed sine;
} b3CosSin;

/// A quaternion.
typedef struct b3Quat
{
	b3Vec3 v;
	b3Fixed s;
} b3Quat;

/// A rigid transform.
typedef struct b3Transform
{
	b3Vec3 p;
	b3Quat q;
} b3Transform;

#if defined( BOX3D_DOUBLE_PRECISION )
// Fixed point has uniform absolute precision across the whole world, so the
// double precision large-world mode is unnecessary and no longer supported.
#error "BOX3D_DOUBLE_PRECISION is not supported with fixed-point math"
#endif

/// A world position. Fixed point has uniform precision everywhere, so world
/// positions use the same representation as local vectors.
typedef b3Vec3 b3Pos;

/// A world transform. Same representation as a local transform in fixed point.
typedef b3Transform b3WorldTransform;

/// A 3x3 matrix.
typedef struct b3Matrix3
{
	b3Vec3 cx, cy, cz;
} b3Matrix3;

/// Axis aligned bounding box.
typedef struct b3AABB
{
	b3Vec3 lowerBound;
	b3Vec3 upperBound;
} b3AABB;

/// A plane.
/// separation = dot(normal, point) - offset
typedef struct b3Plane
{
	b3Vec3 normal;
	b3Fixed offset;
} b3Plane;

static const b3Vec3 b3Vec3_zero = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
static const b3Vec3 b3Vec3_one = { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) };
static const b3Vec3 b3Vec3_axisX = { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
static const b3Vec3 b3Vec3_axisY = { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
static const b3Vec3 b3Vec3_axisZ = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
static const b3Quat b3Quat_identity = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) };
static const b3Transform b3Transform_identity = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) } };
static const b3Matrix3 b3Mat3_zero = {
	{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
	{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
	{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
};
static const b3Matrix3 b3Mat3_identity = {
	{ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
	{ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) },
	{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
};

// Valid in both modes: 0.0f promotes to double, the identity rotation stays b3Fixed
static const b3Pos b3Pos_zero = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
static const b3WorldTransform b3WorldTransform_identity = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) } };

/// @return the minimum of two integers.
B3_INLINE int b3MinInt( int a, int b )
{
	return a < b ? a : b;
}

/// @return the maximum of two integers.
B3_INLINE int b3MaxInt( int a, int b )
{
	return a > b ? a : b;
}

/// @return an integer clamped between a lower and upper bound.
B3_INLINE int b3ClampInt( int a, int lower, int upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

// b3FixAbs, b3FixMin, b3FixMax, and b3FixClamp live in fixed.h

/// Interpolate a scalar.
B3_INLINE b3Fixed b3FixLerp( b3Fixed a, b3Fixed b, b3Fixed alpha )
{
	return b3FixMul( ( B3_FIX( 1.0f ) - alpha ) , a ) + b3FixMul( alpha , b );
}

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
///	Accurate to around 0.0023 degrees.
B3_API b3Fixed b3Atan2( b3Fixed y, b3Fixed x );

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
B3_API b3CosSin b3ComputeCosSin( b3Fixed radians );

/// @deprecated 
B3_INLINE b3Fixed b3Sin( b3Fixed radians )
{
	b3CosSin cs = b3ComputeCosSin( radians );
	return cs.sine;
}

/// @deprecated 
B3_INLINE b3Fixed b3Cos( b3Fixed radians )
{
	b3CosSin cs = b3ComputeCosSin( radians );
	return cs.cosine;
}

/// Convert any angle into the range [-pi, pi].
B3_INLINE b3Fixed b3UnwindAngle( b3Fixed radians )
{
	// remainder( radians, 2 * pi ) with a round-to-nearest quotient,
	// matching the semantics of remainderf
	const b3Fixed twoPi = B3_FIX( 6.28318530718 );
	int64_t n = b3FixRoundToInt( b3FixDiv( radians, twoPi ) );
	return radians - n * twoPi;
}

/// Vector addition.
B3_INLINE b3Vec3 b3Add( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x + b.x, a.y + b.y, a.z + b.z };
}

/// Vector subtraction.
B3_INLINE b3Vec3 b3Sub( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x - b.x, a.y - b.y, a.z - b.z };
}

/// Vector component-wise multiplication.
B3_INLINE b3Vec3 b3Mul( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ b3FixMul( a.x , b.x ), b3FixMul( a.y , b.y ), b3FixMul( a.z , b.z ) };
}

/// Vector negation.
B3_INLINE b3Vec3 b3Neg( b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){ -a.x, -a.y, -a.z };
}

/// Vector dot product.
B3_INLINE b3Fixed b3Dot( b3Vec3 a, b3Vec3 b )
{
	return b3FixMul( a.x , b.x ) + b3FixMul( a.y , b.y ) + b3FixMul( a.z , b.z );
}

/// Vector length. Computed from the exact 128-bit sum of squared components, so
/// it is accurate even for vectors far below unit length.
B3_INLINE b3Fixed b3Length( b3Vec3 v )
{
	b3Int128 ls = (b3Int128)v.x * v.x + (b3Int128)v.y * v.y + (b3Int128)v.z * v.z; // Q32.32 in 128 bits
	return (b3Fixed)b3ISqrt128High( (uint64_t)( (unsigned __int128)ls >> 64 ), (uint64_t)ls );
}

/// Vector length squared.
B3_INLINE b3Fixed b3LengthSquared( b3Vec3 a )
{
	return b3FixMul( a.x , a.x ) + b3FixMul( a.y , a.y ) + b3FixMul( a.z , a.z );
}

/// Distance between two points.
B3_INLINE b3Fixed b3Distance( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return b3Length( dv );
}

/// Squared distance between two points.
B3_INLINE b3Fixed b3DistanceSquared( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return b3FixMul( dv.x , dv.x ) + b3FixMul( dv.y , dv.y ) + b3FixMul( dv.z , dv.z );
}

/// Normalize a vector. Returns a zero vector if the input vector is zero.
/// The squared length and the division run at 128-bit precision, so even
/// vectors far below unit length normalize to within an ulp of unit length.
B3_INLINE b3Vec3 b3Normalize( b3Vec3 a )
{
	b3Int128 ls = (b3Int128)a.x * a.x + (b3Int128)a.y * a.y + (b3Int128)a.z * a.z; // Q32.32 in 128 bits
	if ( ls > 0 )
	{
		b3Fixed length = (b3Fixed)b3ISqrt128High( (uint64_t)( (unsigned __int128)ls >> 64 ), (uint64_t)ls );
		b3Vec3 u = {
			(b3Fixed)( ( (b3Int128)a.x << 16 ) / length ),
			(b3Fixed)( ( (b3Int128)a.y << 16 ) / length ),
			(b3Fixed)( ( (b3Int128)a.z << 16 ) / length ),
		};
		return u;
	}

	return B3_LITERAL( b3Vec3 ){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
}

/// Normalize a vector and return the length. Returns a zero vector
/// if the input is zero.
B3_INLINE b3Vec3 b3GetLengthAndNormalize( b3Fixed* length, b3Vec3 a )
{
	*length = b3Length( a );
	if ( *length < B3_FIXED_EPSILON )
	{
		return b3Vec3_zero;
	}

	b3Vec3 n = {
		(b3Fixed)( ( (b3Int128)a.x << 16 ) / *length ),
		(b3Fixed)( ( (b3Int128)a.y << 16 ) / *length ),
		(b3Fixed)( ( (b3Int128)a.z << 16 ) / *length ),
	};
	return n;
}

/// Get a unit vector that is perpendicular to the supplied vector.
B3_INLINE b3Vec3 b3Perp( b3Vec3 a )
{
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	b3Vec3 p;
	if ( a.x < -B3_FIX( 0.5f ) || B3_FIX( 0.5f ) < a.x )
	{
		p = B3_LITERAL( b3Vec3 ){ a.y, -a.x, B3_FIX( 0.0f ) };
	}
	else
	{
		p = B3_LITERAL( b3Vec3 ){ B3_FIX( 0.0f ), a.z, -a.y };
	}

	return b3Normalize( p );
}

/// Is a vector normalized? In other words, does it have unit length?
B3_INLINE bool b3IsNormalized( b3Vec3 a )
{
	b3Fixed aa = b3Dot( a, a );
	return b3FixAbs( B3_FIX( 1.0f ) - aa ) < 100 * B3_FIXED_EPSILON;
}

/// a + s * b
B3_INLINE b3Vec3 b3MulAdd( b3Vec3 a, b3Fixed s, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x + b3FixMul( s , b.x ), a.y + b3FixMul( s , b.y ), a.z + b3FixMul( s , b.z ) };
}

/// a - s * b
B3_INLINE b3Vec3 b3MulSub( b3Vec3 a, b3Fixed s, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){ a.x - b3FixMul( s , b.x ), a.y - b3FixMul( s , b.y ), a.z - b3FixMul( s , b.z ) };
}

/// s * a
B3_INLINE b3Vec3 b3MulSV( b3Fixed s, b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){ b3FixMul( s , a.x ), b3FixMul( s , a.y ), b3FixMul( s , a.z ) };
}

/// https://en.wikipedia.org/wiki/Cross_product
B3_INLINE b3Vec3 b3Cross( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 c;
	c.x = b3FixMul( a.y , b.z ) - b3FixMul( a.z , b.y );
	c.y = b3FixMul( a.z , b.x ) - b3FixMul( a.x , b.z );
	c.z = b3FixMul( a.x , b.y ) - b3FixMul( a.y , b.x );
	return c;
}

/// Linearly interpolate between two vectors.
B3_INLINE b3Vec3 b3Lerp( b3Vec3 a, b3Vec3 b, b3Fixed alpha )
{
	B3_ASSERT( B3_FIX( 0.0f ) <= alpha && alpha <= B3_FIX( 1.0f ) );

	b3Vec3 c = {
		b3FixMul( ( B3_FIX( 1.0f ) - alpha ) , a.x ) + b3FixMul( alpha , b.x ),
		b3FixMul( ( B3_FIX( 1.0f ) - alpha ) , a.y ) + b3FixMul( alpha , b.y ),
		b3FixMul( ( B3_FIX( 1.0f ) - alpha ) , a.z ) + b3FixMul( alpha , b.z ),
	};
	return c;
}

/// Blend two vectors: s * a + t * b
B3_INLINE b3Vec3 b3Blend2( b3Fixed s, b3Vec3 a, b3Fixed t, b3Vec3 b )
{
	b3Vec3 d = {
		b3FixMul( s , a.x ) + b3FixMul( t , b.x ),
		b3FixMul( s , a.y ) + b3FixMul( t , b.y ),
		b3FixMul( s , a.z ) + b3FixMul( t , b.z ),
	};
	return d;
}

/// Component-wise absolute value.
B3_INLINE b3Vec3 b3Abs( b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){
		b3FixAbs( a.x ),
		b3FixAbs( a.y ),
		b3FixAbs( a.z ),
	};
}

/// Component-wise -1 or 1 (1 if zero).
B3_INLINE b3Vec3 b3Sign( b3Vec3 a )
{
	return B3_LITERAL( b3Vec3 ){
		a.x >= B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : -B3_FIX( 1.0f ),
		a.y >= B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : -B3_FIX( 1.0f ),
		a.z >= B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : -B3_FIX( 1.0f ),
	};
}

/// Component-wise minimum value.
B3_INLINE b3Vec3 b3Min( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){
		b3FixMin( a.x, b.x ),
		b3FixMin( a.y, b.y ),
		b3FixMin( a.z, b.z ),
	};
}

/// Component-wise maximum value.
B3_INLINE b3Vec3 b3Max( b3Vec3 a, b3Vec3 b )
{
	return B3_LITERAL( b3Vec3 ){
		b3FixMax( a.x, b.x ),
		b3FixMax( a.y, b.y ),
		b3FixMax( a.z, b.z ),
	};
}

/// Component-wise clamped value.
B3_INLINE b3Vec3 b3Clamp( b3Vec3 a, b3Vec3 lower, b3Vec3 upper )
{
	b3Vec3 b;
	b.x = b3FixClamp( a.x, lower.x, upper.x );
	b.y = b3FixClamp( a.y, lower.y, upper.y );
	b.z = b3FixClamp( a.z, lower.z, upper.z );
	return b;
}

/// Create a safe scaling value for scaling collision. This allows
/// negative scale, but keeps scale sufficiently far from zero.
B3_INLINE b3Vec3 b3SafeScale( b3Vec3 a )
{
	b3Vec3 absScale = b3Abs( a );
	b3Vec3 minScale = { B3_MIN_SCALE, B3_MIN_SCALE, B3_MIN_SCALE };
	b3Vec3 safeScale = b3Mul( b3Sign( a ), b3Max( absScale, minScale ) );
	return safeScale;
}

/// Does the supplied quaternion have unit length?
B3_INLINE bool b3IsNormalizedQuat( b3Quat q )
{
	b3Fixed qq = b3FixMul( q.v.x , q.v.x ) + b3FixMul( q.v.y , q.v.y ) + b3FixMul( q.v.z , q.v.z ) + b3FixMul( q.s , q.s );
	return B3_FIX( 1.0f ) - 100 * B3_FIXED_EPSILON < qq && qq < B3_FIX( 1.0f ) + 100 * B3_FIXED_EPSILON;
}

/// Rotate a vector.
B3_INLINE b3Vec3 b3RotateVector( b3Quat q, b3Vec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	b3Vec3 t1 = b3Cross( q.v, v );
	b3Vec3 t2 = b3MulAdd( t1, q.s, v );
	b3Vec3 t3 = b3Cross( q.v, t2 );
	return b3MulAdd( v, B3_FIX( 2.0f ), t3 );
}

/// Inverse rotate a vector.
B3_INLINE b3Vec3 b3InvRotateVector( b3Quat q, b3Vec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
	// B3_ASSERT( b3IsNormalizedQuat( q ) );
	b3Vec3 t1 = b3Cross( q.v, v );
	b3Vec3 t2 = b3MulSub( t1, q.s, v );
	b3Vec3 t3 = b3Cross( q.v, t2 );
	return b3MulAdd( v, B3_FIX( 2.0f ), t3 );
}

/// Compute dot product of two quaternions. Useful for polarity tests.
B3_INLINE b3Fixed b3DotQuat( b3Quat a, b3Quat b )
{
	return b3FixMul( a.v.x , b.v.x ) + b3FixMul( a.v.y , b.v.y ) + b3FixMul( a.v.z , b.v.z ) + b3FixMul( a.s , b.s );
}

/// Multiply two quaternions.
B3_INLINE b3Quat b3MulQuat( b3Quat q1, b3Quat q2 )
{
	b3Vec3 t1 = b3Cross( q1.v, q2.v );
	b3Vec3 t2 = b3MulAdd( t1, q1.s, q2.v );
	b3Vec3 t3 = b3MulAdd( t2, q2.s, q1.v );
	b3Quat q = { t3, b3FixMul( q1.s , q2.s ) - b3Dot( q1.v, q2.v ) };
	return q;
}

/// Compute a relative quaternion.
/// inv(q1) * q2
B3_INLINE b3Quat b3InvMulQuat( b3Quat q1, b3Quat q2 )
{
	b3Vec3 t1 = b3Cross( q2.v, q1.v );
	b3Vec3 t2 = b3MulAdd( t1, q1.s, q2.v );
	b3Vec3 t3 = b3MulSub( t2, q2.s, q1.v );
	b3Quat q = { t3, b3FixMul( q1.s , q2.s ) + b3Dot( q1.v, q2.v ) };
	return q;
}

/// Quaternion conjugate (cheap inverse).
B3_INLINE b3Quat b3Conjugate( b3Quat q )
{
	return B3_LITERAL( b3Quat ){ { -q.v.x, -q.v.y, -q.v.z }, q.s };
}

/// Component-wise quaternion negation.
B3_INLINE b3Quat b3NegateQuat( b3Quat q )
{
	return B3_LITERAL( b3Quat ){ { -q.v.x, -q.v.y, -q.v.z }, -q.s };
}

/// Normalize a quaternion at 128-bit precision, see b3Normalize.
B3_INLINE b3Quat b3NormalizeQuat( b3Quat q )
{
	b3Int128 ls = (b3Int128)q.v.x * q.v.x + (b3Int128)q.v.y * q.v.y + (b3Int128)q.v.z * q.v.z + (b3Int128)q.s * q.s;
	if ( ls > 0 )
	{
		b3Fixed length = (b3Fixed)b3ISqrt128High( (uint64_t)( (unsigned __int128)ls >> 64 ), (uint64_t)ls );
		b3Quat qn = {
			{
				(b3Fixed)( ( (b3Int128)q.v.x << 16 ) / length ),
				(b3Fixed)( ( (b3Int128)q.v.y << 16 ) / length ),
				(b3Fixed)( ( (b3Int128)q.v.z << 16 ) / length ),
			},
			(b3Fixed)( ( (b3Int128)q.s << 16 ) / length ),
		};
		return qn;
	}

	return b3Quat_identity;
}

/// Make a quaternion that is equivalent to rotating around an axis by a specified angle.
B3_INLINE b3Quat b3MakeQuatFromAxisAngle( b3Vec3 axis, b3Fixed radians )
{
	B3_ASSERT( b3IsNormalized( axis ) );
	b3CosSin cs = b3ComputeCosSin( b3FixMul( B3_FIX( 0.5f ) , radians ) );
	b3Quat q = { { b3FixMul( cs.sine , axis.x ), b3FixMul( cs.sine , axis.y ), b3FixMul( cs.sine , axis.z ) }, cs.cosine };
	return q;
}

/// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
B3_INLINE b3Vec3 b3GetAxisAngle( b3Fixed* radians, b3Quat q )
{
	b3Fixed length = b3FixSqrt( b3FixMul( q.v.x , q.v.x ) + b3FixMul( q.v.y , q.v.y ) + b3FixMul( q.v.z , q.v.z ) );
	*radians = b3FixMul( B3_FIX( 2.0f ) , b3Atan2( length, q.s ) );
	if ( length > B3_FIX( 0.0f ) )
	{
		b3Fixed invLength = b3FixDiv( B3_FIX( 1.0f ) , length );
		b3Vec3 axis = { b3FixMul( invLength , q.v.x ), b3FixMul( invLength , q.v.y ), b3FixMul( invLength , q.v.z ) };
		return axis;
	}

	return b3Vec3_zero;
}

/// Get the angle for a quaternion in radians
B3_INLINE b3Fixed b3GetQuatAngle( b3Quat q )
{
	b3Fixed length = b3FixSqrt( b3FixMul( q.v.x , q.v.x ) + b3FixMul( q.v.y , q.v.y ) + b3FixMul( q.v.z , q.v.z ) );
	return b3FixMul( B3_FIX( 2.0f ) , b3Atan2( length, q.s ) );
}

/// Extract a quaternion from a rotation matrix.
B3_API b3Quat b3MakeQuatFromMatrix( const b3Matrix3* m );

/// Find a quaternion that rotates one vector to another.
B3_API b3Quat b3ComputeQuatBetweenUnitVectors( b3Vec3 v1, b3Vec3 v2 );

/// Twist angle around the z-axis, used for twist limit and revolute angle limit
B3_INLINE b3Fixed b3GetTwistAngle( b3Quat q )
{
	// Account for polarity to keep the twist angle in range.
	// This is simpler than asking the user to check polarity or unwinding.
	b3Fixed twist = q.s < B3_FIX( 0.0f ) ? b3Atan2( -q.v.z, -q.s ) : b3Atan2( q.v.z, q.s );
	twist = b3FixMul( twist, B3_FIX( 2.0f ) );
	B3_ASSERT( -B3_PI - 2 * B3_FIXED_EPSILON <= twist && twist <= B3_PI + 2 * B3_FIXED_EPSILON );
	return twist;
}

/// Swing angle used for cone limit
B3_INLINE b3Fixed b3GetSwingAngle( b3Quat q )
{
	// Polarity should not matter because all terms are squared.
	b3Fixed x = b3FixSqrt( b3FixMul( q.v.z , q.v.z ) + b3FixMul( q.s , q.s ) );
	b3Fixed y = b3FixSqrt( b3FixMul( q.v.x , q.v.x ) + b3FixMul( q.v.y , q.v.y ) );
	b3Fixed swing = b3FixMul( B3_FIX( 2.0f ) , b3Atan2( y, x ) );
	B3_ASSERT( B3_FIX( 0.0f ) <= swing && swing <= B3_PI + 2 * B3_FIXED_EPSILON );
	return swing;
}

/// Linearly interpolate and normalize between two quaternions
B3_INLINE b3Quat b3NLerp( b3Quat q1, b3Quat q2, b3Fixed alpha )
{
	B3_VALIDATE( B3_FIX( 0.0f ) <= alpha && alpha <= B3_FIX( 1.0f ) );
	if ( b3DotQuat( q1, q2 ) < B3_FIX( 0.0f ) )
	{
		q1 = B3_LITERAL( b3Quat ){ { -q1.v.x, -q1.v.y, -q1.v.z }, -q1.s };
	}

	b3Quat q;
	q.v = b3Lerp( q1.v, q2.v, alpha );
	q.s = b3FixMul( ( B3_FIX( 1.0f ) - alpha ) , q1.s ) + b3FixMul( alpha , q2.s );

	return b3NormalizeQuat( q );
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame. This is useful if frame B is a child of frame A.
B3_INLINE b3Transform b3MulTransforms( b3Transform a, b3Transform b )
{
	b3Transform out;
	out.p = b3Add( b3RotateVector( a.q, b.p ), a.p );
	out.q = b3MulQuat( a.q, b.q );
	return out;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// This is useful for transforming points between the local spaces of two frames that are
/// in world space.
B3_FORCE_INLINE b3Transform b3InvMulTransforms( b3Transform a, b3Transform b )
{
	b3Transform out;
	out.p = b3InvRotateVector( a.q, b3Sub( b.p, a.p ) );
	out.q = b3InvMulQuat( a.q, b.q );
	return out;
}

/// Get the inverse of a transform.
B3_INLINE b3Transform b3InvertTransform( b3Transform t )
{
	b3Transform out;
	out.p = b3InvRotateVector( t.q, b3Neg( t.p ) );
	out.q = b3Conjugate( t.q );
	return out;
}

/// Transform a point.
B3_INLINE b3Vec3 b3TransformPoint( b3Transform t, b3Vec3 v )
{
	b3Vec3 rv = b3RotateVector( t.q, v );
	return b3Add( rv, t.p );
}

/// Inverse transform a point.
B3_INLINE b3Vec3 b3InvTransformPoint( b3Transform t, b3Vec3 v )
{
	return b3InvRotateVector( t.q, b3Sub( v, t.p ) );
}

// World position boundary. These cross between the double precision world space at the public
// boundary and the b3Fixed interior. One set of bodies serves both modes: the typedefs collapse
// the types in b3Fixed mode and the explicit b3Fixed casts become no-ops.

/// Convert a vector to a world position.
B3_INLINE b3Pos b3ToPos( b3Vec3 v )
{
	return B3_LITERAL( b3Pos ){ v.x, v.y, v.z };
}

/// Lossy conversion of a world position to a b3Fixed vector.
B3_INLINE b3Vec3 b3ToVec3( b3Pos p )
{
	return B3_LITERAL( b3Vec3 ){ (b3Fixed)p.x, (b3Fixed)p.y, (b3Fixed)p.z };
}

/// Narrow a world coordinate. World coordinates are the same fixed-point type as
/// local coordinates, so this is the identity. Kept for API compatibility with the
/// old large-world mode.
B3_INLINE b3Fixed b3RoundDownFloat( b3Fixed x )
{
	return x;
}

/// Narrow a world coordinate. The identity in fixed point.
B3_INLINE b3Fixed b3RoundUpFloat( b3Fixed x )
{
	return x;
}

/// a - b, demoted to b3Fixed. The primary precision boundary operation.
B3_INLINE b3Vec3 b3SubPos( b3Pos a, b3Pos b )
{
	return B3_LITERAL( b3Vec3 ){ (b3Fixed)( a.x - b.x ), (b3Fixed)( a.y - b.y ), (b3Fixed)( a.z - b.z ) };
}

/// p + d
B3_INLINE b3Pos b3OffsetPos( b3Pos p, b3Vec3 d )
{
	return B3_LITERAL( b3Pos ){ p.x + d.x, p.y + d.y, p.z + d.z };
}

/// World position interpolation for sweeps and sampling.
B3_INLINE b3Pos b3LerpPosition( b3Pos a, b3Pos b, b3Fixed t )
{
	return B3_LITERAL( b3Pos ){
		b3FixMul( ( B3_FIX( 1.0f ) - t ) , a.x ) + b3FixMul( t , b.x ),
		b3FixMul( ( B3_FIX( 1.0f ) - t ) , a.y ) + b3FixMul( t , b.y ),
		b3FixMul( ( B3_FIX( 1.0f ) - t ) , a.z ) + b3FixMul( t , b.z ),
	};
}

/// Transform a local point to a world position. Rotation in b3Fixed, translation in double.
B3_INLINE b3Pos b3TransformWorldPoint( b3WorldTransform t, b3Vec3 p )
{
	b3Vec3 r = b3RotateVector( t.q, p );
	return B3_LITERAL( b3Pos ){ t.p.x + r.x, t.p.y + r.y, t.p.z + r.z };
}

/// Transform a world position to a local point. One double subtraction, then b3Fixed.
B3_INLINE b3Vec3 b3InvTransformWorldPoint( b3WorldTransform t, b3Pos p )
{
	b3Vec3 d = { (b3Fixed)( p.x - t.p.x ), (b3Fixed)( p.y - t.p.y ), (b3Fixed)( p.z - t.p.z ) };
	return b3InvRotateVector( t.q, d );
}

/// Relative transform of frame B in frame A. The narrow phase boundary.
B3_INLINE b3Transform b3InvMulWorldTransforms( b3WorldTransform A, b3WorldTransform B )
{
	b3Transform C;
	C.q = b3InvMulQuat( A.q, B.q );
	b3Vec3 d = { (b3Fixed)( B.p.x - A.p.x ), (b3Fixed)( B.p.y - A.p.y ), (b3Fixed)( B.p.z - A.p.z ) };
	C.p = b3InvRotateVector( A.q, d );
	return C;
}

/// Compose a world transform with a local transform.
B3_INLINE b3WorldTransform b3MulWorldTransforms( b3WorldTransform A, b3Transform B )
{
	b3WorldTransform C;
	C.q = b3MulQuat( A.q, B.q );
	b3Vec3 r = b3RotateVector( A.q, B.p );
	C.p = B3_LITERAL( b3Pos ){ A.p.x + r.x, A.p.y + r.y, A.p.z + r.z };
	return C;
}

/// Shift a world transform into the frame of a base position.
B3_INLINE b3Transform b3ToRelativeTransform( b3WorldTransform t, b3Pos base )
{
	b3Transform r;
	r.q = t.q;
	r.p = B3_LITERAL( b3Vec3 ){ (b3Fixed)( t.p.x - base.x ), (b3Fixed)( t.p.y - base.y ), (b3Fixed)( t.p.z - base.z ) };
	return r;
}

/// Promote a b3Fixed transform to a world transform. Lossless.
B3_INLINE b3WorldTransform b3MakeWorldTransform( b3Transform t )
{
	b3WorldTransform w;
	w.p = b3ToPos( t.p );
	w.q = t.q;
	return w;
}

/// Translate a local AABB by a world origin. Fixed-point addition is exact, so no
/// outward rounding is needed: the translated box is the translated box.
B3_INLINE b3AABB b3OffsetAABB( b3AABB localBox, b3Pos origin )
{
	b3AABB out;
	out.lowerBound.x = origin.x + localBox.lowerBound.x;
	out.lowerBound.y = origin.y + localBox.lowerBound.y;
	out.lowerBound.z = origin.z + localBox.lowerBound.z;
	out.upperBound.x = origin.x + localBox.upperBound.x;
	out.upperBound.y = origin.y + localBox.upperBound.y;
	out.upperBound.z = origin.z + localBox.upperBound.z;
	return out;
}

/// Compute the determinant of a 3-by-3 matrix.
B3_INLINE b3Fixed b3Det( b3Matrix3 m )
{
	return b3Dot( m.cx, b3Cross( m.cy, m.cz ) );
}

#if B3_HAS_INT128
// Internal: 3x3 cofactors at Q32.32 in 128 bits and the determinant at Q16.48.
// The Q48.16 determinant of a matrix with small entries (like the inertia of a
// small body) underflows to zero, so the inverse and solve helpers work at
// full precision internally.
B3_INLINE b3Int128 b3Cofactor128( b3Fixed a, b3Fixed b, b3Fixed c, b3Fixed d )
{
	return (b3Int128)a * b - (b3Int128)c * d; // Q32.32
}
#endif

/// Multiply a matrix times a column vector.
B3_INLINE b3Vec3 b3MulMV( b3Matrix3 m, b3Vec3 a )
{
	b3Vec3 b = {
		b3FixMul( m.cx.x , a.x ) + b3FixMul( m.cy.x , a.y ) + b3FixMul( m.cz.x , a.z ),
		b3FixMul( m.cx.y , a.x ) + b3FixMul( m.cy.y , a.y ) + b3FixMul( m.cz.y , a.z ),
		b3FixMul( m.cx.z , a.x ) + b3FixMul( m.cy.z , a.y ) + b3FixMul( m.cz.z , a.z ),
	};
	return b;
}

/// Negate a matrix.
B3_INLINE b3Matrix3 b3NegateMat3( b3Matrix3 a )
{
	return B3_LITERAL( b3Matrix3 ){
		{ -a.cx.x, -a.cx.y, -a.cx.z },
		{ -a.cy.x, -a.cy.y, -a.cy.z },
		{ -a.cz.x, -a.cz.y, -a.cz.z },
	};
}

/// Matrix addition.
/// @return a + b
B3_INLINE b3Matrix3 b3AddMM( b3Matrix3 a, b3Matrix3 b )
{
	return B3_LITERAL( b3Matrix3 ){
		{ a.cx.x + b.cx.x, a.cx.y + b.cx.y, a.cx.z + b.cx.z },
		{ a.cy.x + b.cy.x, a.cy.y + b.cy.y, a.cy.z + b.cy.z },
		{ a.cz.x + b.cz.x, a.cz.y + b.cz.y, a.cz.z + b.cz.z },
	};
}

/// Matrix subtraction.
/// @return a - b
B3_INLINE b3Matrix3 b3SubMM( b3Matrix3 a, b3Matrix3 b )
{
	return B3_LITERAL( b3Matrix3 ){
		{ a.cx.x - b.cx.x, a.cx.y - b.cx.y, a.cx.z - b.cx.z },
		{ a.cy.x - b.cy.x, a.cy.y - b.cy.y, a.cy.z - b.cy.z },
		{ a.cz.x - b.cz.x, a.cz.y - b.cz.y, a.cz.z - b.cz.z },
	};
}

/// Multiply a matrix by a scalar, component-wise.
B3_INLINE b3Matrix3 b3MulSM( b3Fixed s, b3Matrix3 a )
{
	return B3_LITERAL( b3Matrix3 ){
		{ b3FixMul( s , a.cx.x ), b3FixMul( s , a.cx.y ), b3FixMul( s , a.cx.z ) },
		{ b3FixMul( s , a.cy.x ), b3FixMul( s , a.cy.y ), b3FixMul( s , a.cy.z ) },
		{ b3FixMul( s , a.cz.x ), b3FixMul( s , a.cz.y ), b3FixMul( s , a.cz.z ) },
	};
}

/// Matrix multiplication.
/// @return a * b
B3_INLINE b3Matrix3 b3MulMM( b3Matrix3 a, b3Matrix3 b )
{
	b3Matrix3 out;
	out.cx = b3MulMV( a, b.cx );
	out.cy = b3MulMV( a, b.cy );
	out.cz = b3MulMV( a, b.cz );
	return out;
}

/// Matrix transpose.
B3_INLINE b3Matrix3 b3Transpose( b3Matrix3 m )
{
	b3Matrix3 out;
	out.cx = B3_LITERAL( b3Vec3 ){ m.cx.x, m.cy.x, m.cz.x };
	out.cy = B3_LITERAL( b3Vec3 ){ m.cx.y, m.cy.y, m.cz.y };
	out.cz = B3_LITERAL( b3Vec3 ){ m.cx.z, m.cy.z, m.cz.z };

	return out;
}

/// General matrix inverse.
B3_INLINE b3Matrix3 b3InvertMatrix( b3Matrix3 m )
{
	// Full precision cofactors (Q32.32 in 128 bits) so small matrices like the
	// inertia of tiny bodies stay invertible: a Q48.16 determinant underflows.
	b3Int128 c00 = b3Cofactor128( m.cy.y, m.cz.z, m.cy.z, m.cz.y );
	b3Int128 c01 = b3Cofactor128( m.cy.z, m.cz.x, m.cy.x, m.cz.z );
	b3Int128 c02 = b3Cofactor128( m.cy.x, m.cz.y, m.cy.y, m.cz.x );
	b3Int128 c10 = b3Cofactor128( m.cz.y, m.cx.z, m.cz.z, m.cx.y );
	b3Int128 c11 = b3Cofactor128( m.cz.z, m.cx.x, m.cz.x, m.cx.z );
	b3Int128 c12 = b3Cofactor128( m.cz.x, m.cx.y, m.cz.y, m.cx.x );
	b3Int128 c20 = b3Cofactor128( m.cx.y, m.cy.z, m.cx.z, m.cy.y );
	b3Int128 c21 = b3Cofactor128( m.cx.z, m.cy.x, m.cx.x, m.cy.z );
	b3Int128 c22 = b3Cofactor128( m.cx.x, m.cy.y, m.cx.y, m.cy.x );

	b3Int128 limit = (b3Int128)1 << 62;
	if ( -limit < c00 && c00 < limit && -limit < c10 && c10 < limit && -limit < c20 && c20 < limit )
	{
		// Exact path: cofactors fit in 64 bits, determinant at Q16.48
		b3Int128 det = (b3Int128)m.cx.x * (int64_t)c00 + (b3Int128)m.cy.x * (int64_t)c10 + (b3Int128)m.cz.x * (int64_t)c20;
		if ( det != 0 )
		{
			// inverse_ij = cofactor_ji / det: (Q32.32 << 32) / Q16.48 -> Q48.16
			b3Matrix3 out;
			out.cx = B3_LITERAL( b3Vec3 ){ (b3Fixed)( ( c00 << 32 ) / det ), (b3Fixed)( ( c10 << 32 ) / det ),
										   (b3Fixed)( ( c20 << 32 ) / det ) };
			out.cy = B3_LITERAL( b3Vec3 ){ (b3Fixed)( ( c01 << 32 ) / det ), (b3Fixed)( ( c11 << 32 ) / det ),
										   (b3Fixed)( ( c21 << 32 ) / det ) };
			out.cz = B3_LITERAL( b3Vec3 ){ (b3Fixed)( ( c02 << 32 ) / det ), (b3Fixed)( ( c12 << 32 ) / det ),
										   (b3Fixed)( ( c22 << 32 ) / det ) };
			return out;
		}
		return b3Mat3_zero;
	}

	// Huge matrix path: drop 16 fraction bits from the cofactors to keep the
	// determinant accumulation in range
	b3Int128 det = (b3Int128)m.cx.x * (int64_t)( c00 >> 16 ) + (b3Int128)m.cy.x * (int64_t)( c10 >> 16 ) +
				   (b3Int128)m.cz.x * (int64_t)( c20 >> 16 ); // ~Q16.32
	if ( det != 0 )
	{
		b3Matrix3 out;
		out.cx = B3_LITERAL( b3Vec3 ){ (b3Fixed)( ( c00 << 16 ) / det ), (b3Fixed)( ( c10 << 16 ) / det ),
									   (b3Fixed)( ( c20 << 16 ) / det ) };
		out.cy = B3_LITERAL( b3Vec3 ){ (b3Fixed)( ( c01 << 16 ) / det ), (b3Fixed)( ( c11 << 16 ) / det ),
									   (b3Fixed)( ( c21 << 16 ) / det ) };
		out.cz = B3_LITERAL( b3Vec3 ){ (b3Fixed)( ( c02 << 16 ) / det ), (b3Fixed)( ( c12 << 16 ) / det ),
									   (b3Fixed)( ( c22 << 16 ) / det ) };
		return out;
	}

	return b3Mat3_zero;
}

/// Solve a matrix equation.
/// @return inv(m) * a
/// Solves directly from the 128-bit cofactors with three divisions rather than
/// inverting (nine divisions) and multiplying.
B3_INLINE b3Vec3 b3Solve3( b3Matrix3 m, b3Vec3 a )
{
	b3Int128 c00 = b3Cofactor128( m.cy.y, m.cz.z, m.cy.z, m.cz.y );
	b3Int128 c01 = b3Cofactor128( m.cy.z, m.cz.x, m.cy.x, m.cz.z );
	b3Int128 c02 = b3Cofactor128( m.cy.x, m.cz.y, m.cy.y, m.cz.x );
	b3Int128 c10 = b3Cofactor128( m.cz.y, m.cx.z, m.cz.z, m.cx.y );
	b3Int128 c11 = b3Cofactor128( m.cz.z, m.cx.x, m.cz.x, m.cx.z );
	b3Int128 c12 = b3Cofactor128( m.cz.x, m.cx.y, m.cz.y, m.cx.x );
	b3Int128 c20 = b3Cofactor128( m.cx.y, m.cy.z, m.cx.z, m.cy.y );
	b3Int128 c21 = b3Cofactor128( m.cx.z, m.cy.x, m.cx.x, m.cy.z );
	b3Int128 c22 = b3Cofactor128( m.cx.x, m.cy.y, m.cx.y, m.cy.x );

	b3Int128 limit = (b3Int128)1 << 62;
	if ( -limit < c00 && c00 < limit && -limit < c10 && c10 < limit && -limit < c20 && c20 < limit )
	{
		// Exact path: cofactors fit in 64 bits, determinant at Q16.48
		b3Int128 det = (b3Int128)m.cx.x * (int64_t)c00 + (b3Int128)m.cy.x * (int64_t)c10 + (b3Int128)m.cz.x * (int64_t)c20;
		if ( det != 0 )
		{
			// x_i = ( sum_j cofactor_ji * a_j ) / det: (Q32.32 * Q48.16 << 16) / Q16.48 -> Q48.16
			b3Int128 nx = (b3Int128)(int64_t)c00 * a.x + (b3Int128)(int64_t)c01 * a.y + (b3Int128)(int64_t)c02 * a.z;
			b3Int128 ny = (b3Int128)(int64_t)c10 * a.x + (b3Int128)(int64_t)c11 * a.y + (b3Int128)(int64_t)c12 * a.z;
			b3Int128 nz = (b3Int128)(int64_t)c20 * a.x + (b3Int128)(int64_t)c21 * a.y + (b3Int128)(int64_t)c22 * a.z;

			b3Vec3 b = {
				(b3Fixed)( ( nx << 16 ) / det ),
				(b3Fixed)( ( ny << 16 ) / det ),
				(b3Fixed)( ( nz << 16 ) / det ),
			};
			return b;
		}
		return b3Vec3_zero;
	}

	// Huge matrix path
	b3Matrix3 inv = b3InvertMatrix( m );
	return b3MulMV( inv, a );
}

/// Inverse transpose of a matrix. Identical to the inverse for the symmetric
/// matrices (like inertia tensors) this is used with.
B3_INLINE b3Matrix3 b3InvertT( b3Matrix3 m )
{
	b3Matrix3 out = b3InvertMatrix( m );
	return b3Transpose( out );
}

/// Get the component-wise absolute value of a matrix.
B3_INLINE b3Matrix3 b3AbsMatrix3( b3Matrix3 m )
{
	b3Matrix3 out;
	out.cx = b3Abs( m.cx );
	out.cy = b3Abs( m.cy );
	out.cz = b3Abs( m.cz );

	return out;
}

/// Make a matrix from a quaternion. This is useful if you need to
/// rotate many vectors.
/// The force inline improves the performance of b3ShapeDistance.
B3_FORCE_INLINE b3Matrix3 b3MakeMatrixFromQuat( b3Quat q )
{
	b3Fixed xx = b3FixMul( q.v.x , q.v.x );
	b3Fixed yy = b3FixMul( q.v.y , q.v.y );
	b3Fixed zz = b3FixMul( q.v.z , q.v.z );
	b3Fixed xy = b3FixMul( q.v.x , q.v.y );
	b3Fixed xz = b3FixMul( q.v.x , q.v.z );
	b3Fixed xw = b3FixMul( q.v.x , q.s );
	b3Fixed yz = b3FixMul( q.v.y , q.v.z );
	b3Fixed yw = b3FixMul( q.v.y , q.s );
	b3Fixed zw = b3FixMul( q.v.z , q.s );

	return B3_LITERAL( b3Matrix3 ){
		{ B3_FIX( 1.0f ) - b3FixMul( B3_FIX( 2.0f ) , ( yy + zz ) ), b3FixMul( B3_FIX( 2.0f ) , ( xy + zw ) ), b3FixMul( B3_FIX( 2.0f ) , ( xz - yw ) ) },
		{ b3FixMul( B3_FIX( 2.0f ) , ( xy - zw ) ), B3_FIX( 1.0f ) - b3FixMul( B3_FIX( 2.0f ) , ( xx + zz ) ), b3FixMul( B3_FIX( 2.0f ) , ( yz + xw ) ) },
		{ b3FixMul( B3_FIX( 2.0f ) , ( xz + yw ) ), b3FixMul( B3_FIX( 2.0f ) , ( yz - xw ) ), B3_FIX( 1.0f ) - b3FixMul( B3_FIX( 2.0f ) , ( xx + yy ) ) },
	};
}

/// Get the inertia tensor of an offset point.
/// https://en.wikipedia.org/wiki/Parallel_axis_theorem
B3_API b3Matrix3 b3Steiner( b3Fixed mass, b3Vec3 origin );

/// Get the AABB of a point cloud.
B3_INLINE b3AABB b3MakeAABB( const b3Vec3* points, int count, b3Fixed radius )
{
	B3_ASSERT( count > 0 );
	b3AABB a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound = b3Min( a.lowerBound, points[i] );
		a.upperBound = b3Max( a.upperBound, points[i] );
	}

	b3Vec3 r = { radius, radius, radius };
	a.lowerBound = b3Sub( a.lowerBound, r );
	a.upperBound = b3Add( a.upperBound, r );

	return a;
}

/// Does a fully contain b?
B3_INLINE bool b3AABB_Contains( b3AABB a, b3AABB b )
{
	if ( a.lowerBound.x > b.lowerBound.x || b.upperBound.x > a.upperBound.x )
		return false;
	if ( a.lowerBound.y > b.lowerBound.y || b.upperBound.y > a.upperBound.y )
		return false;
	if ( a.lowerBound.z > b.lowerBound.z || b.upperBound.z > a.upperBound.z )
		return false;

	return true;
}

/// Get the surface area of an axis-aligned bounding box.
B3_INLINE b3Fixed b3AABB_Area( b3AABB a )
{
	b3Vec3 delta = b3Sub( a.upperBound, a.lowerBound );
	return b3FixMul( B3_FIX( 2.0f ) , ( b3FixMul( delta.x , delta.y ) + b3FixMul( delta.y , delta.z ) + b3FixMul( delta.z , delta.x ) ) );
}

/// Get the center of an axis-aligned bounding box.
B3_INLINE b3Vec3 b3AABB_Center( b3AABB a )
{
	return b3MulSV( B3_FIX( 0.5f ), b3Add( a.upperBound, a.lowerBound ) );
}

/// Get the extents (half-widths) of an axis-aligned bounding box.
B3_INLINE b3Vec3 b3AABB_Extents( b3AABB a )
{
	return b3MulSV( B3_FIX( 0.5f ), b3Sub( a.upperBound, a.lowerBound ) );
}

/// Get the union of two axis-aligned bounding boxes.
B3_INLINE b3AABB b3AABB_Union( b3AABB a, b3AABB b )
{
	b3AABB out;
	out.lowerBound = b3Min( a.lowerBound, b.lowerBound );
	out.upperBound = b3Max( a.upperBound, b.upperBound );
	return out;
}

/// Add uniform padding to an axis-aligned bounding box.
B3_INLINE b3AABB b3AABB_Inflate( b3AABB a, b3Fixed extension )
{
	b3Vec3 radius = { extension, extension, extension };

	b3AABB out;
	out.lowerBound = b3Sub( a.lowerBound, radius );
	out.upperBound = b3Add( a.upperBound, radius );
	return out;
}

/// Do two axis-aligned boxes overlap?
B3_INLINE bool b3AABB_Overlaps( b3AABB a, b3AABB b )
{
	// No intersection if separated along one axis
	if ( a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x )
		return false;
	if ( a.upperBound.y < b.lowerBound.y || a.lowerBound.y > b.upperBound.y )
		return false;
	if ( a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z )
		return false;

	// Overlapping on all axis means bounds are intersecting
	return true;
}

/// Transform an axis-aligned bounding box. This can create a larger box
/// than if you recomputed the AABB of the original shape with the transform
/// applied.
B3_INLINE b3AABB b3AABB_Transform( b3Transform transform, b3AABB a )
{
	b3Vec3 center = b3TransformPoint( transform, b3AABB_Center( a ) );
	b3Matrix3 m = b3MakeMatrixFromQuat( transform.q );
	b3Vec3 extent = b3MulMV( b3AbsMatrix3( m ), b3AABB_Extents( a ) );
	b3AABB out = { b3Sub( center, extent ), b3Add( center, extent ) };
	return out;
}

/// Get the closest point on an axis-aligned bounding box.
B3_INLINE b3Vec3 b3ClosestPointToAABB( b3Vec3 point, b3AABB a )
{
	return b3Clamp( point, a.lowerBound, a.upperBound );
}

/// The closest points between to segments or infinite lines.
typedef struct b3SegmentDistanceResult
{
	b3Vec3 point1;
	b3Fixed fraction1;
	b3Vec3 point2;
	b3Fixed fraction2;
} b3SegmentDistanceResult;

/// Compute the closest point on the segment a-b to the target q.
B3_API b3Vec3 b3PointToSegmentDistance( b3Vec3 a, b3Vec3 b, b3Vec3 q );

/// Compute the closest points on two infinite lines.
B3_API b3SegmentDistanceResult b3LineDistance( b3Vec3 p1, b3Vec3 d1, b3Vec3 p2, b3Vec3 d2 );

/// Compute the closest points on two line segments.
B3_API b3SegmentDistanceResult b3SegmentDistance( b3Vec3 p1, b3Vec3 q1, b3Vec3 p2, b3Vec3 q2 );

/// Is this a valid number? Not NaN or infinity.
B3_API bool b3IsValidFixed( b3Fixed a );

/// Is this a valid vector? Not NaN or infinity.
B3_API bool b3IsValidVec3( b3Vec3 a );

/// Is this a valid quaternion? Not NaN or infinity. Is normalized.
B3_API bool b3IsValidQuat( b3Quat q );

/// Is this a valid transform? Not NaN or infinity. Is normalized.
B3_API bool b3IsValidTransform( b3Transform a );

/// Is this a valid matrix? Not NaN or infinity.
B3_API bool b3IsValidMatrix3( b3Matrix3 a );

/// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
B3_API bool b3IsValidAABB( b3AABB a );

/// Is this AABB reasonably close to the origin? See B3_HUGE.
B3_API bool b3IsBoundedAABB( b3AABB a );

/// Is this AABB valid and reasonable?
B3_API bool b3IsSaneAABB( b3AABB a );

/// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
B3_API bool b3IsValidPlane( b3Plane a );

/// Is this a valid world position? Not NaN or infinity.
B3_API bool b3IsValidPosition( b3Pos p );

/// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
B3_API bool b3IsValidWorldTransform( b3WorldTransform t );

/**@}*/ // math

/**
 * @defgroup math_cpp C++ Math
 * @brief Math operator overloads for C++
 * Some of the simpler ones are expanded to improve debug performance.
 * See math_functions.h for details.
 * @{
 */

#ifdef __cplusplus

/// Vector addition.
B3_FORCE_INLINE b3Vec3& operator+=( b3Vec3& a, b3Vec3 b )
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}

/// Vector subtraction.
B3_FORCE_INLINE b3Vec3& operator-=( b3Vec3& a, b3Vec3 b )
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return a;
}

/// Vector scaling.
B3_FORCE_INLINE b3Vec3& operator*=( b3Vec3& a, b3Fixed s )
{
	a.x = b3FixMul( a.x, s );
	a.y = b3FixMul( a.y, s );
	a.z = b3FixMul( a.z, s );
	return a;
}

/// Vector negation.
B3_FORCE_INLINE b3Vec3 operator-( b3Vec3 a )
{
	return { -a.x, -a.y, -a.z };
}

/// Vector scaling.
B3_FORCE_INLINE b3Vec3 operator*( b3Fixed s, b3Vec3 a )
{
	return { b3FixMul( s, a.x ), b3FixMul( s, a.y ), b3FixMul( s, a.z ) };
}

/// Vector scaling.
B3_FORCE_INLINE b3Vec3 operator*( b3Vec3 a, b3Fixed s )
{
	return { b3FixMul( s, a.x ), b3FixMul( s, a.y ), b3FixMul( s, a.z ) };
}

/// Component-wise vector multiplication.
B3_FORCE_INLINE b3Vec3 operator*( b3Vec3 a, b3Vec3 b )
{
	return { b3FixMul( a.x, b.x ), b3FixMul( a.y, b.y ), b3FixMul( a.z, b.z ) };
}

/// Vector addition.
B3_FORCE_INLINE b3Vec3 operator+( b3Vec3 a, b3Vec3 b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}

/// Vector subtraction.
B3_FORCE_INLINE b3Vec3 operator-( b3Vec3 a, b3Vec3 b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}

#endif

/**@}*/ // math_cpp
