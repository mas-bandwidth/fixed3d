// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Fixed-point vector / quaternion / matrix / transform types. In fixed point, world
// positions have uniform precision everywhere, so b3Pos is just b3Vec3 -- no separate
// wide-world type. Ops on these types are being migrated here incrementally.
#pragma once
#include "fixed/base.h"
#include "fixed/fixed.h"
#include "fixed/fixed_math.h"

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

/// Pi in Q48.16.
#define B3_PI B3_FIX( 3.14159265359f )

/// Minimum representable scale used by b3SafeScale.
#define B3_MIN_SCALE B3_FIX( 0.01f )

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

// Float utilities for rendering, UI, and other non-simulation code. The
// simulation itself never uses these (see the fixed-point conversion notes).

/// @return the minimum of two floats.
B3_INLINE float b3MinFloat( float a, float b )
{
	return a < b ? a : b;
}

/// @return the maximum of two floats.
B3_INLINE float b3MaxFloat( float a, float b )
{
	return a > b ? a : b;
}

/// @return a float clamped between a lower and upper bound.
B3_INLINE float b3ClampFloat( float a, float lower, float upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

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

// b3Atan2, b3ComputeCosSin, b3Sin, b3Cos, and b3UnwindAngle are provided by
// fixed/fixed_math.h (included above). fixed3d's physics calls them; the vendored
// `fixed` library owns their declarations and definitions.

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

/// Exact dot product accumulated at 128 bits, scaled by 2^(2*B3_FIXED_FRACTION_BITS).
/// No per-component rounding or saturation, so sign tests and comparisons on the
/// raw value are exact even for sub-resolution results.
B3_INLINE b3Int128 b3DotRaw( b3Vec3 a, b3Vec3 b )
{
	return (b3Int128)a.x * b.x + (b3Int128)a.y * b.y + (b3Int128)a.z * b.z;
}

/// Round a raw 128-bit dot product to fixed point with a single round-half-up
/// step (divide last), matching b3FixMul rounding and overflow policy.
B3_INLINE b3Fixed b3FixFromDotRaw( b3Int128 raw )
{
	b3Int128 r = ( raw + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS;
#if defined( BOX3D_FIXED_SATURATE )
	if ( r > (b3Int128)INT64_MAX )
	{
		return B3_FIXED_MAX;
	}
	if ( r < -(b3Int128)INT64_MAX )
	{
		return B3_FIXED_MIN;
	}
#endif
	return (b3Fixed)r;
}

/// Vector dot product. Accumulated at 128 bits with a single rounding (divide last).
B3_INLINE b3Fixed b3Dot( b3Vec3 a, b3Vec3 b )
{
	return b3FixFromDotRaw( b3DotRaw( a, b ) );
}

/// Vector length. Computed from the exact 128-bit sum of squared components, so
/// it is accurate even for vectors far below unit length.
B3_INLINE b3Fixed b3Length( b3Vec3 v )
{
	b3Int128 ls = (b3Int128)v.x * v.x + (b3Int128)v.y * v.y + (b3Int128)v.z * v.z; // Q32.32 in 128 bits
	return (b3Fixed)b3ISqrt128High( (uint64_t)( (b3UInt128)ls >> 64 ), (uint64_t)ls );
}

/// Vector length squared. One rounding on the exact 128-bit sum of squares.
B3_INLINE b3Fixed b3LengthSquared( b3Vec3 a )
{
	return b3FixFromDotRaw( b3DotRaw( a, a ) );
}

/// Distance between two points.
B3_INLINE b3Fixed b3Distance( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return b3Length( dv );
}

/// Squared distance between two points. One rounding on the exact 128-bit sum.
B3_INLINE b3Fixed b3DistanceSquared( b3Vec3 a, b3Vec3 b )
{
	b3Vec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return b3FixFromDotRaw( b3DotRaw( dv, dv ) );
}

/// Normalize a vector. Returns a zero vector if the input vector is zero.
/// The squared length and the division run at 128-bit precision, so even
/// vectors far below unit length normalize to within an ulp of unit length.
B3_INLINE b3Vec3 b3Normalize( b3Vec3 a )
{
	b3Int128 ls = (b3Int128)a.x * a.x + (b3Int128)a.y * a.y + (b3Int128)a.z * a.z; // Q32.32 in 128 bits
	if ( ls > 0 )
	{
		b3Fixed length = (b3Fixed)b3ISqrt128High( (uint64_t)( (b3UInt128)ls >> 64 ), (uint64_t)ls );
		// b3FixDiv computes the same truncating 128-bit quotient, with a single
		// hardware divide when the component fits in 47 bits (the common case)
		b3Vec3 u = {
			b3FixDiv( a.x, length ),
			b3FixDiv( a.y, length ),
			b3FixDiv( a.z, length ),
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
		b3FixDiv( a.x, *length ),
		b3FixDiv( a.y, *length ),
		b3FixDiv( a.z, *length ),
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
/// Kept in the two-cross form: fused single-rounding variants of this and
/// b3Lerp/b3MulMV/b3Cross perturb knife-edge equilibria (mesh-drop sleep,
/// convex pile SAT cache). See the round-3 notes in CLAUDE.md.
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
/// One rounding on the exact 128-bit sum.
B3_INLINE b3Fixed b3DotQuat( b3Quat a, b3Quat b )
{
	return b3FixFromDotRaw( (b3Int128)a.v.x * b.v.x + (b3Int128)a.v.y * b.v.y + (b3Int128)a.v.z * b.v.z +
							(b3Int128)a.s * b.s );
}

/// Multiply two quaternions. Each component is a fused 128-bit reduction with
/// a single rounding.
B3_INLINE b3Quat b3MulQuat( b3Quat q1, b3Quat q2 )
{
	// v = cross(q1.v, q2.v) + q1.s * q2.v + q2.s * q1.v
	// s = q1.s * q2.s - dot(q1.v, q2.v)
	b3Quat q = {
		{
			b3FixFromDotRaw( (b3Int128)q1.v.y * q2.v.z - (b3Int128)q1.v.z * q2.v.y + (b3Int128)q1.s * q2.v.x +
							 (b3Int128)q2.s * q1.v.x ),
			b3FixFromDotRaw( (b3Int128)q1.v.z * q2.v.x - (b3Int128)q1.v.x * q2.v.z + (b3Int128)q1.s * q2.v.y +
							 (b3Int128)q2.s * q1.v.y ),
			b3FixFromDotRaw( (b3Int128)q1.v.x * q2.v.y - (b3Int128)q1.v.y * q2.v.x + (b3Int128)q1.s * q2.v.z +
							 (b3Int128)q2.s * q1.v.z ),
		},
		b3FixFromDotRaw( (b3Int128)q1.s * q2.s - (b3Int128)q1.v.x * q2.v.x - (b3Int128)q1.v.y * q2.v.y -
						 (b3Int128)q1.v.z * q2.v.z ),
	};
	return q;
}

/// Compute a relative quaternion.
/// inv(q1) * q2
B3_INLINE b3Quat b3InvMulQuat( b3Quat q1, b3Quat q2 )
{
	// v = cross(q2.v, q1.v) + q1.s * q2.v - q2.s * q1.v
	// s = q1.s * q2.s + dot(q1.v, q2.v)
	b3Quat q = {
		{
			b3FixFromDotRaw( (b3Int128)q2.v.y * q1.v.z - (b3Int128)q2.v.z * q1.v.y + (b3Int128)q1.s * q2.v.x -
							 (b3Int128)q2.s * q1.v.x ),
			b3FixFromDotRaw( (b3Int128)q2.v.z * q1.v.x - (b3Int128)q2.v.x * q1.v.z + (b3Int128)q1.s * q2.v.y -
							 (b3Int128)q2.s * q1.v.y ),
			b3FixFromDotRaw( (b3Int128)q2.v.x * q1.v.y - (b3Int128)q2.v.y * q1.v.x + (b3Int128)q1.s * q2.v.z -
							 (b3Int128)q2.s * q1.v.z ),
		},
		b3FixFromDotRaw( (b3Int128)q1.s * q2.s + (b3Int128)q1.v.x * q2.v.x + (b3Int128)q1.v.y * q2.v.y +
						 (b3Int128)q1.v.z * q2.v.z ),
	};
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
		b3Fixed length = (b3Fixed)b3ISqrt128High( (uint64_t)( (b3UInt128)ls >> 64 ), (uint64_t)ls );
		// b3FixDiv computes the same truncating quotient, with a single hardware
		// divide for the near-unit components this always sees
		b3Quat qn = {
			{
				b3FixDiv( q.v.x, length ),
				b3FixDiv( q.v.y, length ),
				b3FixDiv( q.v.z, length ),
			},
			b3FixDiv( q.s, length ),
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

