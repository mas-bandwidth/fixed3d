// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Fixed-point vector / quaternion / matrix / transform types. In fixed point, world
// positions have uniform precision everywhere, so fixPos is just fixVec3 -- no separate
// wide-world type. Ops on these types are being migrated here incrementally.
#pragma once
#include "fixed/base.h"
#include "fixed/fixed.h"
#include "fixed/fixed_int256.h"
#include "fixed/fixed_math.h"

/// A 2D vector.
typedef struct fixVec2
{
	fixed_t x;
	fixed_t y;
} fixVec2;

/// A 3D vector.
typedef struct fixVec3
{
	fixed_t x;
	fixed_t y;
	fixed_t z;
} fixVec3;

/// A quaternion.
typedef struct fixQuat
{
	fixVec3 v;
	fixed_t s;
} fixQuat;

/// A rigid transform.
typedef struct fixTransform
{
	fixVec3 p;
	fixQuat q;
} fixTransform;

/// A world position. Fixed point has uniform precision everywhere, so world
/// positions use the same representation as local vectors.
typedef fixVec3 fixPos;

/// A world transform. Same representation as a local transform in fixed point.
typedef fixTransform fixWorldTransform;

/// A 3x3 matrix.
typedef struct fixMatrix3
{
	fixVec3 cx, cy, cz;
} fixMatrix3;

/// Pi in Q48.16.
#define FIX_PI FIX( 3.14159265359f )

/// Minimum representable scale used by fixSafeScale.
#define FIX_MIN_SCALE FIX( 0.01f )

static const fixVec3 fixVec3_zero = { FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) };
static const fixVec3 fixVec3_one = { FIX( 1.0f ), FIX( 1.0f ), FIX( 1.0f ) };
static const fixVec3 fixVec3_axisX = { FIX( 1.0f ), FIX( 0.0f ), FIX( 0.0f ) };
static const fixVec3 fixVec3_axisY = { FIX( 0.0f ), FIX( 1.0f ), FIX( 0.0f ) };
static const fixVec3 fixVec3_axisZ = { FIX( 0.0f ), FIX( 0.0f ), FIX( 1.0f ) };
static const fixQuat fixQuat_identity = { { FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) }, FIX( 1.0f ) };
static const fixTransform fixTransform_identity = { { FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) }, { { FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) }, FIX( 1.0f ) } };
static const fixMatrix3 fixMat3_zero = {
	{ FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) },
	{ FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) },
	{ FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) },
};
static const fixMatrix3 fixMat3_identity = {
	{ FIX( 1.0f ), FIX( 0.0f ), FIX( 0.0f ) },
	{ FIX( 0.0f ), FIX( 1.0f ), FIX( 0.0f ) },
	{ FIX( 0.0f ), FIX( 0.0f ), FIX( 1.0f ) },
};

// Valid in both modes: 0.0f promotes to double, the identity rotation stays fixed_t
static const fixPos fixPos_zero = { FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) };
static const fixWorldTransform fixWorldTransform_identity = { { FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) }, { { FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) }, FIX( 1.0f ) } };

// Float utilities for rendering, UI, and other non-simulation code. The
// simulation itself never uses these (see the fixed-point conversion notes).

/// @return the minimum of two floats.
FIX_INLINE float fixMinFloat( float a, float b )
{
	return a < b ? a : b;
}

/// @return the maximum of two floats.
FIX_INLINE float fixMaxFloat( float a, float b )
{
	return a > b ? a : b;
}

/// @return a float clamped between a lower and upper bound.
FIX_INLINE float fixClampFloat( float a, float lower, float upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

/// @return the minimum of two integers.
FIX_INLINE int fixMinInt( int a, int b )
{
	return a < b ? a : b;
}

/// @return the maximum of two integers.
FIX_INLINE int fixMaxInt( int a, int b )
{
	return a > b ? a : b;
}

/// @return an integer clamped between a lower and upper bound.
FIX_INLINE int fixClampInt( int a, int lower, int upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

// fixAbs, fixMin, fixMax, and fixClamp live in fixed.h

// fixLerp, fixAtan2, fixComputeCosSin, fixSin, fixCos, and fixUnwindAngle are provided
// by fixed/fixed_math.h (included above). fixed3d's physics calls them; the vendored
// `fixed` library owns their declarations and definitions. fixLerp is scalar math and
// lives with the rest of the scalar math, not in the vector header that happens to have
// been its first caller.

/// Vector addition.
FIX_INLINE fixVec3 fixVecAdd( fixVec3 a, fixVec3 b )
{
	return FIX_LITERAL( fixVec3 ){ a.x + b.x, a.y + b.y, a.z + b.z };
}

/// Vector subtraction.
FIX_INLINE fixVec3 fixVecSub( fixVec3 a, fixVec3 b )
{
	return FIX_LITERAL( fixVec3 ){ a.x - b.x, a.y - b.y, a.z - b.z };
}

/// Vector component-wise multiplication.
FIX_INLINE fixVec3 fixVecMul( fixVec3 a, fixVec3 b )
{
	return FIX_LITERAL( fixVec3 ){ fixMul( a.x , b.x ), fixMul( a.y , b.y ), fixMul( a.z , b.z ) };
}

/// Vector negation.
FIX_INLINE fixVec3 fixVecNeg( fixVec3 a )
{
	return FIX_LITERAL( fixVec3 ){ -a.x, -a.y, -a.z };
}

/// Exact dot product accumulated at 128 bits, scaled by 2^(2*FIX_FRACTION_BITS).
/// No per-component rounding or saturation, so sign tests and comparisons on the
/// raw value are exact even for sub-resolution results.
FIX_INLINE fixInt128 fixDotRaw( fixVec3 a, fixVec3 b )
{
	return fixInt128Add( fixInt128Add( fixInt128MulI64( a.x, b.x ), fixInt128MulI64( a.y, b.y ) ),
						 fixInt128MulI64( a.z, b.z ) );
}

/// Round a raw 128-bit dot product to fixed point with a single round-half-up
/// step (divide last), matching fixMul rounding and overflow policy.
FIX_INLINE fixed_t fixFromDotRaw( fixInt128 raw )
{
	fixInt128 r = fixInt128Shr( fixInt128Add( raw, fixInt128FromI64( FIX_HALF ) ), FIX_FRACTION_BITS );
#if defined( FIX_SATURATE )
	if ( fixInt128Gt( r, fixInt128FromI64( INT64_MAX ) ) )
	{
		return FIX_MAX;
	}
	if ( fixInt128Lt( r, fixInt128FromI64( -INT64_MAX ) ) )
	{
		return FIX_MIN;
	}
#endif
	return (fixed_t)fixInt128ToI64( r );
}

/// Vector dot product. Accumulated at 128 bits with a single rounding (divide last).
FIX_INLINE fixed_t fixDot( fixVec3 a, fixVec3 b )
{
	return fixFromDotRaw( fixDotRaw( a, b ) );
}

/// Vector length. Computed from the exact 128-bit sum of squared components, so
/// it is accurate even for vectors far below unit length.
FIX_INLINE fixed_t fixLength( fixVec3 v )
{
	fixInt128 ls = fixDotRaw( v, v ); // Q32.32 in 128 bits
	return (fixed_t)fixISqrt128High( fixInt128Hi( ls ), fixInt128Lo( ls ) );
}

/// Vector length squared. One rounding on the exact 128-bit sum of squares.
FIX_INLINE fixed_t fixLengthSquared( fixVec3 a )
{
	return fixFromDotRaw( fixDotRaw( a, a ) );
}

/// Distance between two points.
FIX_INLINE fixed_t fixDistance( fixVec3 a, fixVec3 b )
{
	fixVec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return fixLength( dv );
}

/// Squared distance between two points. One rounding on the exact 128-bit sum.
FIX_INLINE fixed_t fixDistanceSquared( fixVec3 a, fixVec3 b )
{
	fixVec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return fixFromDotRaw( fixDotRaw( dv, dv ) );
}

/// Normalize a vector. Returns a zero vector if the input vector is zero.
/// The squared length and the division run at 128-bit precision, so even
/// vectors far below unit length normalize to within an ulp of unit length.
FIX_INLINE fixVec3 fixNormalize( fixVec3 a )
{
	fixInt128 ls = fixDotRaw( a, a ); // Q32.32 in 128 bits
	if ( fixInt128Gt( ls, FIX_INT128_ZERO ) )
	{
		fixed_t length = (fixed_t)fixISqrt128High( fixInt128Hi( ls ), fixInt128Lo( ls ) );
		// fixDiv computes the same truncating 128-bit quotient, with a single
		// hardware divide when the component fits in 47 bits (the common case)
		fixVec3 u = {
			fixDiv( a.x, length ),
			fixDiv( a.y, length ),
			fixDiv( a.z, length ),
		};
		return u;
	}

	return FIX_LITERAL( fixVec3 ){ FIX( 0.0f ), FIX( 0.0f ), FIX( 0.0f ) };
}

/// Normalize a vector and return the length. Returns a zero vector
/// if the input is zero.
FIX_INLINE fixVec3 fixGetLengthAndNormalize( fixed_t* length, fixVec3 a )
{
	*length = fixLength( a );
	if ( *length < FIX_EPSILON )
	{
		return fixVec3_zero;
	}

	fixVec3 n = {
		fixDiv( a.x, *length ),
		fixDiv( a.y, *length ),
		fixDiv( a.z, *length ),
	};
	return n;
}

/// Get a unit vector that is perpendicular to the supplied vector.
FIX_INLINE fixVec3 fixPerp( fixVec3 a )
{
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	fixVec3 p;
	if ( a.x < -FIX( 0.5f ) || FIX( 0.5f ) < a.x )
	{
		p = FIX_LITERAL( fixVec3 ){ a.y, -a.x, FIX( 0.0f ) };
	}
	else
	{
		p = FIX_LITERAL( fixVec3 ){ FIX( 0.0f ), a.z, -a.y };
	}

	return fixNormalize( p );
}

/// Is a vector normalized? In other words, does it have unit length?
FIX_INLINE bool fixIsNormalized( fixVec3 a )
{
	fixed_t aa = fixDot( a, a );
	return fixAbs( FIX( 1.0f ) - aa ) < 100 * FIX_EPSILON;
}

/// a + s * b
FIX_INLINE fixVec3 fixMulAdd( fixVec3 a, fixed_t s, fixVec3 b )
{
	return FIX_LITERAL( fixVec3 ){ a.x + fixMul( s , b.x ), a.y + fixMul( s , b.y ), a.z + fixMul( s , b.z ) };
}

/// a - s * b
FIX_INLINE fixVec3 fixMulSub( fixVec3 a, fixed_t s, fixVec3 b )
{
	return FIX_LITERAL( fixVec3 ){ a.x - fixMul( s , b.x ), a.y - fixMul( s , b.y ), a.z - fixMul( s , b.z ) };
}

/// s * a
FIX_INLINE fixVec3 fixMulSV( fixed_t s, fixVec3 a )
{
	return FIX_LITERAL( fixVec3 ){ fixMul( s , a.x ), fixMul( s , a.y ), fixMul( s , a.z ) };
}

/// https://en.wikipedia.org/wiki/Cross_product
FIX_INLINE fixVec3 fixCross( fixVec3 a, fixVec3 b )
{
	fixVec3 c;
	c.x = fixMul( a.y , b.z ) - fixMul( a.z , b.y );
	c.y = fixMul( a.z , b.x ) - fixMul( a.x , b.z );
	c.z = fixMul( a.x , b.y ) - fixMul( a.y , b.x );
	return c;
}

/// Linearly interpolate between two vectors.
FIX_INLINE fixVec3 fixVecLerp( fixVec3 a, fixVec3 b, fixed_t alpha )
{
	FIX_ASSERT( FIX( 0.0f ) <= alpha && alpha <= FIX( 1.0f ) );

	fixVec3 c = {
		fixMul( ( FIX( 1.0f ) - alpha ) , a.x ) + fixMul( alpha , b.x ),
		fixMul( ( FIX( 1.0f ) - alpha ) , a.y ) + fixMul( alpha , b.y ),
		fixMul( ( FIX( 1.0f ) - alpha ) , a.z ) + fixMul( alpha , b.z ),
	};
	return c;
}

/// Blend two vectors: s * a + t * b
FIX_INLINE fixVec3 fixBlend2( fixed_t s, fixVec3 a, fixed_t t, fixVec3 b )
{
	fixVec3 d = {
		fixMul( s , a.x ) + fixMul( t , b.x ),
		fixMul( s , a.y ) + fixMul( t , b.y ),
		fixMul( s , a.z ) + fixMul( t , b.z ),
	};
	return d;
}

/// Component-wise absolute value.
FIX_INLINE fixVec3 fixVecAbs( fixVec3 a )
{
	return FIX_LITERAL( fixVec3 ){
		fixAbs( a.x ),
		fixAbs( a.y ),
		fixAbs( a.z ),
	};
}

/// Component-wise -1 or 1 (1 if zero).
FIX_INLINE fixVec3 fixSign( fixVec3 a )
{
	return FIX_LITERAL( fixVec3 ){
		a.x >= FIX( 0.0f ) ? FIX( 1.0f ) : -FIX( 1.0f ),
		a.y >= FIX( 0.0f ) ? FIX( 1.0f ) : -FIX( 1.0f ),
		a.z >= FIX( 0.0f ) ? FIX( 1.0f ) : -FIX( 1.0f ),
	};
}

/// Component-wise minimum value.
FIX_INLINE fixVec3 fixVecMin( fixVec3 a, fixVec3 b )
{
	return FIX_LITERAL( fixVec3 ){
		fixMin( a.x, b.x ),
		fixMin( a.y, b.y ),
		fixMin( a.z, b.z ),
	};
}

/// Component-wise maximum value.
FIX_INLINE fixVec3 fixVecMax( fixVec3 a, fixVec3 b )
{
	return FIX_LITERAL( fixVec3 ){
		fixMax( a.x, b.x ),
		fixMax( a.y, b.y ),
		fixMax( a.z, b.z ),
	};
}

/// Component-wise clamped value.
FIX_INLINE fixVec3 fixVecClamp( fixVec3 a, fixVec3 lower, fixVec3 upper )
{
	fixVec3 b;
	b.x = fixClamp( a.x, lower.x, upper.x );
	b.y = fixClamp( a.y, lower.y, upper.y );
	b.z = fixClamp( a.z, lower.z, upper.z );
	return b;
}

/// Create a safe scaling value for scaling collision. This allows
/// negative scale, but keeps scale sufficiently far from zero.
FIX_INLINE fixVec3 fixSafeScale( fixVec3 a )
{
	fixVec3 absScale = fixVecAbs( a );
	fixVec3 minScale = { FIX_MIN_SCALE, FIX_MIN_SCALE, FIX_MIN_SCALE };
	fixVec3 safeScale = fixVecMul( fixSign( a ), fixVecMax( absScale, minScale ) );
	return safeScale;
}

/// Does the supplied quaternion have unit length?
FIX_INLINE bool fixIsNormalizedQuat( fixQuat q )
{
	fixed_t qq = fixMul( q.v.x , q.v.x ) + fixMul( q.v.y , q.v.y ) + fixMul( q.v.z , q.v.z ) + fixMul( q.s , q.s );
	return FIX( 1.0f ) - 100 * FIX_EPSILON < qq && qq < FIX( 1.0f ) + 100 * FIX_EPSILON;
}

/// Rotate a vector.
/// Kept in the two-cross form: fused single-rounding variants of this and
/// fixVecLerp/fixMulMV/fixCross perturb knife-edge equilibria (mesh-drop sleep,
/// convex pile SAT cache).
FIX_INLINE fixVec3 fixRotateVector( fixQuat q, fixVec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	// FIX_ASSERT( fixIsNormalizedQuat( q ) );
	fixVec3 t1 = fixCross( q.v, v );
	fixVec3 t2 = fixMulAdd( t1, q.s, v );
	fixVec3 t3 = fixCross( q.v, t2 );
	return fixMulAdd( v, FIX( 2.0f ), t3 );
}

/// Inverse rotate a vector.
FIX_INLINE fixVec3 fixInvRotateVector( fixQuat q, fixVec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
	// FIX_ASSERT( fixIsNormalizedQuat( q ) );
	fixVec3 t1 = fixCross( q.v, v );
	fixVec3 t2 = fixMulSub( t1, q.s, v );
	fixVec3 t3 = fixCross( q.v, t2 );
	return fixMulAdd( v, FIX( 2.0f ), t3 );
}

/// Compute dot product of two quaternions. Useful for polarity tests.
/// One rounding on the exact 128-bit sum.
FIX_INLINE fixed_t fixDotQuat( fixQuat a, fixQuat b )
{
	return fixFromDotRaw( fixInt128Add( fixDotRaw( a.v, b.v ), fixInt128MulI64( a.s, b.s ) ) );
}

/// Multiply two quaternions. Each component is a fused 128-bit reduction with
/// a single rounding.
FIX_INLINE fixQuat fixMulQuat( fixQuat q1, fixQuat q2 )
{
	// v = cross(q1.v, q2.v) + q1.s * q2.v + q2.s * q1.v
	// s = q1.s * q2.s - dot(q1.v, q2.v)
	fixQuat q = {
		{
			fixFromDotRaw( fixInt128Add(
				fixInt128Add( fixInt128Sub( fixInt128MulI64( q1.v.y, q2.v.z ), fixInt128MulI64( q1.v.z, q2.v.y ) ),
							  fixInt128MulI64( q1.s, q2.v.x ) ),
				fixInt128MulI64( q2.s, q1.v.x ) ) ),
			fixFromDotRaw( fixInt128Add(
				fixInt128Add( fixInt128Sub( fixInt128MulI64( q1.v.z, q2.v.x ), fixInt128MulI64( q1.v.x, q2.v.z ) ),
							  fixInt128MulI64( q1.s, q2.v.y ) ),
				fixInt128MulI64( q2.s, q1.v.y ) ) ),
			fixFromDotRaw( fixInt128Add(
				fixInt128Add( fixInt128Sub( fixInt128MulI64( q1.v.x, q2.v.y ), fixInt128MulI64( q1.v.y, q2.v.x ) ),
							  fixInt128MulI64( q1.s, q2.v.z ) ),
				fixInt128MulI64( q2.s, q1.v.z ) ) ),
		},
		fixFromDotRaw( fixInt128Sub( fixInt128MulI64( q1.s, q2.s ), fixDotRaw( q1.v, q2.v ) ) ),
	};
	return q;
}

/// Compute a relative quaternion.
/// inv(q1) * q2
FIX_INLINE fixQuat fixInvMulQuat( fixQuat q1, fixQuat q2 )
{
	// v = cross(q2.v, q1.v) + q1.s * q2.v - q2.s * q1.v
	// s = q1.s * q2.s + dot(q1.v, q2.v)
	fixQuat q = {
		{
			fixFromDotRaw( fixInt128Sub(
				fixInt128Add( fixInt128Sub( fixInt128MulI64( q2.v.y, q1.v.z ), fixInt128MulI64( q2.v.z, q1.v.y ) ),
							  fixInt128MulI64( q1.s, q2.v.x ) ),
				fixInt128MulI64( q2.s, q1.v.x ) ) ),
			fixFromDotRaw( fixInt128Sub(
				fixInt128Add( fixInt128Sub( fixInt128MulI64( q2.v.z, q1.v.x ), fixInt128MulI64( q2.v.x, q1.v.z ) ),
							  fixInt128MulI64( q1.s, q2.v.y ) ),
				fixInt128MulI64( q2.s, q1.v.y ) ) ),
			fixFromDotRaw( fixInt128Sub(
				fixInt128Add( fixInt128Sub( fixInt128MulI64( q2.v.x, q1.v.y ), fixInt128MulI64( q2.v.y, q1.v.x ) ),
							  fixInt128MulI64( q1.s, q2.v.z ) ),
				fixInt128MulI64( q2.s, q1.v.z ) ) ),
		},
		fixFromDotRaw( fixInt128Add( fixInt128MulI64( q1.s, q2.s ), fixDotRaw( q1.v, q2.v ) ) ),
	};
	return q;
}

/// Quaternion conjugate (cheap inverse).
FIX_INLINE fixQuat fixConjugate( fixQuat q )
{
	return FIX_LITERAL( fixQuat ){ { -q.v.x, -q.v.y, -q.v.z }, q.s };
}

/// Component-wise quaternion negation.
FIX_INLINE fixQuat fixNegateQuat( fixQuat q )
{
	return FIX_LITERAL( fixQuat ){ { -q.v.x, -q.v.y, -q.v.z }, -q.s };
}

/// Normalize a quaternion at 128-bit precision, see fixNormalize.
FIX_INLINE fixQuat fixNormalizeQuat( fixQuat q )
{
	fixInt128 ls = fixInt128Add( fixDotRaw( q.v, q.v ), fixInt128MulI64( q.s, q.s ) );
	if ( fixInt128Gt( ls, FIX_INT128_ZERO ) )
	{
		fixed_t length = (fixed_t)fixISqrt128High( fixInt128Hi( ls ), fixInt128Lo( ls ) );
		// fixDiv computes the same truncating quotient, with a single hardware
		// divide for the near-unit components this always sees
		fixQuat qn = {
			{
				fixDiv( q.v.x, length ),
				fixDiv( q.v.y, length ),
				fixDiv( q.v.z, length ),
			},
			fixDiv( q.s, length ),
		};
		return qn;
	}

	return fixQuat_identity;
}

/// Make a quaternion that is equivalent to rotating around an axis by a specified angle.
FIX_INLINE fixQuat fixMakeQuatFromAxisAngle( fixVec3 axis, fixed_t radians )
{
	FIX_ASSERT( fixIsNormalized( axis ) );
	fixCosSin cs = fixComputeCosSin( fixMul( FIX( 0.5f ) , radians ) );
	fixQuat q = { { fixMul( cs.sine , axis.x ), fixMul( cs.sine , axis.y ), fixMul( cs.sine , axis.z ) }, cs.cosine };
	return q;
}

/// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
FIX_INLINE fixVec3 fixGetAxisAngle( fixed_t* radians, fixQuat q )
{
	fixed_t length = fixSqrt( fixMul( q.v.x , q.v.x ) + fixMul( q.v.y , q.v.y ) + fixMul( q.v.z , q.v.z ) );
	*radians = fixMul( FIX( 2.0f ) , fixAtan2( length, q.s ) );
	if ( length > FIX( 0.0f ) )
	{
		fixed_t invLength = fixDiv( FIX( 1.0f ) , length );
		fixVec3 axis = { fixMul( invLength , q.v.x ), fixMul( invLength , q.v.y ), fixMul( invLength , q.v.z ) };
		return axis;
	}

	return fixVec3_zero;
}

/// Get the angle for a quaternion in radians
FIX_INLINE fixed_t fixGetQuatAngle( fixQuat q )
{
	fixed_t length = fixSqrt( fixMul( q.v.x , q.v.x ) + fixMul( q.v.y , q.v.y ) + fixMul( q.v.z , q.v.z ) );
	return fixMul( FIX( 2.0f ) , fixAtan2( length, q.s ) );
}

/// Extract a quaternion from a rotation matrix.
FIX_API fixQuat fixMakeQuatFromMatrix( const fixMatrix3* m );

/// Find a quaternion that rotates one vector to another.
FIX_API fixQuat fixComputeQuatBetweenUnitVectors( fixVec3 v1, fixVec3 v2 );

/// Twist angle around the z-axis, used for twist limit and revolute angle limit
FIX_INLINE fixed_t fixGetTwistAngle( fixQuat q )
{
	// Account for polarity to keep the twist angle in range.
	// This is simpler than asking the user to check polarity or unwinding.
	fixed_t twist = q.s < FIX( 0.0f ) ? fixAtan2( -q.v.z, -q.s ) : fixAtan2( q.v.z, q.s );
	twist = fixMul( twist, FIX( 2.0f ) );
	FIX_ASSERT( -FIX_PI - 2 * FIX_EPSILON <= twist && twist <= FIX_PI + 2 * FIX_EPSILON );
	return twist;
}

/// Swing angle used for cone limit
FIX_INLINE fixed_t fixGetSwingAngle( fixQuat q )
{
	// Polarity should not matter because all terms are squared.
	fixed_t x = fixSqrt( fixMul( q.v.z , q.v.z ) + fixMul( q.s , q.s ) );
	fixed_t y = fixSqrt( fixMul( q.v.x , q.v.x ) + fixMul( q.v.y , q.v.y ) );
	fixed_t swing = fixMul( FIX( 2.0f ) , fixAtan2( y, x ) );
	FIX_ASSERT( FIX( 0.0f ) <= swing && swing <= FIX_PI + 2 * FIX_EPSILON );
	return swing;
}

/// Linearly interpolate and normalize between two quaternions
FIX_INLINE fixQuat fixNLerp( fixQuat q1, fixQuat q2, fixed_t alpha )
{
	FIX_VALIDATE( FIX( 0.0f ) <= alpha && alpha <= FIX( 1.0f ) );
	if ( fixDotQuat( q1, q2 ) < FIX( 0.0f ) )
	{
		q1 = FIX_LITERAL( fixQuat ){ { -q1.v.x, -q1.v.y, -q1.v.z }, -q1.s };
	}

	fixQuat q;
	q.v = fixVecLerp( q1.v, q2.v, alpha );
	q.s = fixMul( ( FIX( 1.0f ) - alpha ) , q1.s ) + fixMul( alpha , q2.s );

	return fixNormalizeQuat( q );
}

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame. This is useful if frame B is a child of frame A.
FIX_INLINE fixTransform fixMulTransforms( fixTransform a, fixTransform b )
{
	fixTransform out;
	out.p = fixVecAdd( fixRotateVector( a.q, b.p ), a.p );
	out.q = fixMulQuat( a.q, b.q );
	return out;
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// This is useful for transforming points between the local spaces of two frames that are
/// in world space.
FIX_FORCE_INLINE fixTransform fixInvMulTransforms( fixTransform a, fixTransform b )
{
	fixTransform out;
	out.p = fixInvRotateVector( a.q, fixVecSub( b.p, a.p ) );
	out.q = fixInvMulQuat( a.q, b.q );
	return out;
}

/// Get the inverse of a transform.
FIX_INLINE fixTransform fixInvertTransform( fixTransform t )
{
	fixTransform out;
	out.p = fixInvRotateVector( t.q, fixVecNeg( t.p ) );
	out.q = fixConjugate( t.q );
	return out;
}

/// Transform a point.
FIX_INLINE fixVec3 fixTransformPoint( fixTransform t, fixVec3 v )
{
	fixVec3 rv = fixRotateVector( t.q, v );
	return fixVecAdd( rv, t.p );
}

/// Inverse transform a point.
FIX_INLINE fixVec3 fixInvTransformPoint( fixTransform t, fixVec3 v )
{
	return fixInvRotateVector( t.q, fixVecSub( v, t.p ) );
}

/// Convert a vector to a world position.
FIX_INLINE fixPos fixToPos( fixVec3 v )
{
	return FIX_LITERAL( fixPos ){ v.x, v.y, v.z };
}

/// Lossy conversion of a world position to a fixed_t vector.
FIX_INLINE fixVec3 fixToVec3( fixPos p )
{
	return FIX_LITERAL( fixVec3 ){ (fixed_t)p.x, (fixed_t)p.y, (fixed_t)p.z };
}

/// Narrow a world coordinate. World coordinates are the same fixed-point type as
/// local coordinates, so this is the identity. Kept for API compatibility with the
/// old large-world mode.
FIX_INLINE fixed_t fixRoundDownFloat( fixed_t x )
{
	return x;
}

/// Narrow a world coordinate. The identity in fixed point.
FIX_INLINE fixed_t fixRoundUpFloat( fixed_t x )
{
	return x;
}

/// a - b, demoted to fixed_t. The primary precision boundary operation.
FIX_INLINE fixVec3 fixSubPos( fixPos a, fixPos b )
{
	return FIX_LITERAL( fixVec3 ){ (fixed_t)( a.x - b.x ), (fixed_t)( a.y - b.y ), (fixed_t)( a.z - b.z ) };
}

/// p + d
FIX_INLINE fixPos fixOffsetPos( fixPos p, fixVec3 d )
{
	return FIX_LITERAL( fixPos ){ p.x + d.x, p.y + d.y, p.z + d.z };
}

/// World position interpolation for sweeps and sampling.
FIX_INLINE fixPos fixLerpPosition( fixPos a, fixPos b, fixed_t t )
{
	return FIX_LITERAL( fixPos ){
		fixMul( ( FIX( 1.0f ) - t ) , a.x ) + fixMul( t , b.x ),
		fixMul( ( FIX( 1.0f ) - t ) , a.y ) + fixMul( t , b.y ),
		fixMul( ( FIX( 1.0f ) - t ) , a.z ) + fixMul( t , b.z ),
	};
}

/// Transform a local point to a world position. Rotation in fixed_t, translation in double.
FIX_INLINE fixPos fixTransformWorldPoint( fixWorldTransform t, fixVec3 p )
{
	fixVec3 r = fixRotateVector( t.q, p );
	return FIX_LITERAL( fixPos ){ t.p.x + r.x, t.p.y + r.y, t.p.z + r.z };
}

/// Transform a world position to a local point. One double subtraction, then fixed_t.
FIX_INLINE fixVec3 fixInvTransformWorldPoint( fixWorldTransform t, fixPos p )
{
	fixVec3 d = { (fixed_t)( p.x - t.p.x ), (fixed_t)( p.y - t.p.y ), (fixed_t)( p.z - t.p.z ) };
	return fixInvRotateVector( t.q, d );
}

/// Relative transform of frame B in frame A. The narrow phase boundary.
FIX_INLINE fixTransform fixInvMulWorldTransforms( fixWorldTransform A, fixWorldTransform B )
{
	fixTransform C;
	C.q = fixInvMulQuat( A.q, B.q );
	fixVec3 d = { (fixed_t)( B.p.x - A.p.x ), (fixed_t)( B.p.y - A.p.y ), (fixed_t)( B.p.z - A.p.z ) };
	C.p = fixInvRotateVector( A.q, d );
	return C;
}

/// Compose a world transform with a local transform.
FIX_INLINE fixWorldTransform fixMulWorldTransforms( fixWorldTransform A, fixTransform B )
{
	fixWorldTransform C;
	C.q = fixMulQuat( A.q, B.q );
	fixVec3 r = fixRotateVector( A.q, B.p );
	C.p = FIX_LITERAL( fixPos ){ A.p.x + r.x, A.p.y + r.y, A.p.z + r.z };
	return C;
}

/// Shift a world transform into the frame of a base position.
FIX_INLINE fixTransform fixToRelativeTransform( fixWorldTransform t, fixPos base )
{
	fixTransform r;
	r.q = t.q;
	r.p = FIX_LITERAL( fixVec3 ){ (fixed_t)( t.p.x - base.x ), (fixed_t)( t.p.y - base.y ), (fixed_t)( t.p.z - base.z ) };
	return r;
}

/// Promote a fixed_t transform to a world transform. Lossless.
FIX_INLINE fixWorldTransform fixMakeWorldTransform( fixTransform t )
{
	fixWorldTransform w;
	w.p = fixToPos( t.p );
	w.q = t.q;
	return w;
}

/// Compute the determinant of a 3-by-3 matrix.
FIX_INLINE fixed_t fixDet( fixMatrix3 m )
{
	return fixDot( m.cx, fixCross( m.cy, m.cz ) );
}

// Internal: 3x3 cofactors at Q32.32 in 128 bits and the determinant at Q16.48.
// The Q48.16 determinant of a matrix with small entries (like the inertia of a
// small body) underflows to zero, so the inverse and solve helpers work at
// full precision internally.
FIX_INLINE fixInt128 fixCofactor128( fixed_t a, fixed_t b, fixed_t c, fixed_t d )
{
	return fixInt128Sub( fixInt128MulI64( a, b ), fixInt128MulI64( c, d ) ); // Q32.32
}

/// Internal: ( numerator << shift ) / denominator, truncated to Q48.16. The shift routes
/// through fixInt128ShiftLeft because the numerators here are signed cofactors and a raw
/// left shift of a negative value is undefined.
///
/// SATURATES rather than truncating to 64 bits. A ratio outside Q48.16 range -- the
/// inverse of a matrix close enough to singular that its inverse does not fit -- used to
/// keep the low 64 bits of the quotient, which is a value of arbitrary magnitude AND
/// arbitrary sign. Saturating keeps the one property every caller relies on: a large
/// inverse reads as large, and its sign is the sign of the true ratio.
FIX_INLINE fixed_t fixDivShifted( fixInt128 numerator, int shift, fixInt128 denominator )
{
	fixInt128 quotient = fixInt128Div( fixInt128ShiftLeft( numerator, shift ), denominator );

	bool representable = fixInt128Le( quotient, fixInt128FromI64( FIX_MAX ) ) && fixInt128Ge( quotient, fixInt128FromI64( FIX_MIN ) );
	FIX_VALIDATE( representable );
	if ( representable == false )
	{
		return fixInt128IsNegative( quotient ) ? FIX_MIN : FIX_MAX;
	}

	return (fixed_t)fixInt128ToI64( quotient );
}

/// Internal: is a 128-bit value strictly inside +/- limit?
///
/// The range test that decides whether the 128-bit arm of the inverse and the solve can
/// stay exact. Those arms cast cofactors to int64 and shift numerators left, so every
/// value they touch has to be small enough for both -- which is why the tests below name
/// all nine cofactors rather than the three the determinant expands along.
FIX_ALWAYS_INLINE bool fixInt128InRange( fixInt128 c, fixInt128 limit )
{
	return fixInt128Lt( fixInt128Neg( limit ), c ) && fixInt128Lt( c, limit );
}

/// Validity predicates for the fixed-point math types.
///
/// Extracted from fixed3d's math_functions.c, unchanged in behaviour. In fixed3d these
/// are FIX_API functions in a translation unit; here they are inline, matching the rest
/// of this header. The bodies are byte-equivalent.
///
/// Deliberately NOT extracted, and the reason matters:
///   - b3IsValidRay takes b3RayCastInput, a box3d collision type. Physics, not
///     fixed-point math; it stays with the solver.
///   - b3IsBoundedAABB / b3IsSaneAABB resolve B3_HUGE through box3d's mutable
///     b3GetLengthUnitsPerMeter() global. They encode world-scale POLICY, and a math
///     library should hold no opinion about how big a world is.
///
/// The position and bounds validators DID move, and the design fork that once blocked
/// them is resolved: this library exports BOTH widths unconditionally (fixIsValidVec3
/// and fixIsValidPosWide, fixIsValidAABB and fixIsValidAABBWide), so nothing here is
/// selected by a compile flag. box3d picks a width in its own compatibility header,
/// where its own flag belongs.

/// True if a is a representable fixed-point quantity.
///
/// Fixed point has no NaN, and the saturation values are legal quantities: FIX_MAX
/// plays the role FLT_MAX did (joint thresholds and spring force limits default to it,
/// mirroring isfinite( FLT_MAX ) == true). Only INT64_MIN is unrepresentable -- reserved
/// so that negation cannot overflow.
FIX_INLINE bool fixIsValidFixed( fixed_t a )
{
	return a != INT64_MIN;
}

/// True if every component of a is a representable fixed-point quantity.
FIX_INLINE bool fixIsValidVec3( fixVec3 a )
{
	return fixIsValidFixed( a.x ) && fixIsValidFixed( a.y ) && fixIsValidFixed( a.z );
}

/// True if q is representable AND normalized. A non-normalized quaternion is not a
/// rotation, so the normalization check is part of validity, not a separate question.
FIX_INLINE bool fixIsValidQuat( fixQuat a )
{
	if ( !fixIsValidFixed( a.v.x ) || !fixIsValidFixed( a.v.y ) || !fixIsValidFixed( a.v.z ) || !fixIsValidFixed( a.s ) )
	{
		return false;
	}

	return fixIsNormalizedQuat( a );
}

/// True if both the translation and the rotation of a are valid.
FIX_INLINE bool fixIsValidTransform( fixTransform a )
{
	return fixIsValidVec3( a.p ) && fixIsValidQuat( a.q );
}

/// True if every column of a is a valid vector.
FIX_INLINE bool fixIsValidMatrix3( fixMatrix3 a )
{
	return fixIsValidVec3( a.cx ) && fixIsValidVec3( a.cy ) && fixIsValidVec3( a.cz );
}

/// A plane: unit normal and offset along it.
typedef struct fixPlane
{
	fixVec3 normal;
	fixed_t offset;
} fixPlane;

/// Is this a valid plane? Normal must be finite AND unit length.
FIX_INLINE bool fixIsValidPlane( fixPlane a )
{
	if ( fixIsValidVec3( a.normal ) == false )
	{
		return false;
	}

	if ( fixIsNormalized( a.normal ) == false )
	{
		return false;
	}

	return fixIsValidFixed( a.offset );
}

/// An axis-aligned bounding box in local (Q48.16) space.
///
/// The wide (Q112.16) counterpart is fixAABBWide in fixed_wide.h. Both are exported
/// unconditionally: a consumer picks narrow or wide by which type it names, not by a
/// compile flag that silently changes an ABI. In box3d these two live in opposite
/// branches of BOX3D_LUDICROUS_MODE, which means the wide half only compiles when that
/// flag is set -- and it is off by default, so that code is dark in every ordinary
/// build. Here both compile and both are tested on every run.
typedef struct fixAABB
{
	fixVec3 lowerBound;
	fixVec3 upperBound;
} fixAABB;

/// Get the AABB of a point cloud, expanded by a uniform radius.
FIX_INLINE fixAABB fixMakeAABB( const fixVec3* points, int count, fixed_t radius )
{
	FIX_ASSERT( count > 0 );
	fixAABB a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound = fixVecMin( a.lowerBound, points[i] );
		a.upperBound = fixVecMax( a.upperBound, points[i] );
	}

	fixVec3 r = { radius, radius, radius };
	a.lowerBound = fixVecSub( a.lowerBound, r );
	a.upperBound = fixVecAdd( a.upperBound, r );

	return a;
}

/// Does a fully contain b?
FIX_INLINE bool fixAABB_Contains( fixAABB a, fixAABB b )
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
FIX_INLINE fixed_t fixAABB_Area( fixAABB a )
{
	fixVec3 delta = fixVecSub( a.upperBound, a.lowerBound );
	return fixMul( FIX( 2.0f ) , ( fixMul( delta.x , delta.y ) + fixMul( delta.y , delta.z ) + fixMul( delta.z , delta.x ) ) );
}

/// Get the center of an axis-aligned bounding box.
FIX_INLINE fixVec3 fixAABB_Center( fixAABB a )
{
	return fixMulSV( FIX( 0.5f ), fixVecAdd( a.upperBound, a.lowerBound ) );
}

/// Get the extents (half-widths) of an axis-aligned bounding box.
FIX_INLINE fixVec3 fixAABB_Extents( fixAABB a )
{
	return fixMulSV( FIX( 0.5f ), fixVecSub( a.upperBound, a.lowerBound ) );
}

/// Get the union of two axis-aligned bounding boxes.
FIX_INLINE fixAABB fixAABB_Union( fixAABB a, fixAABB b )
{
	fixAABB out;
	out.lowerBound = fixVecMin( a.lowerBound, b.lowerBound );
	out.upperBound = fixVecMax( a.upperBound, b.upperBound );
	return out;
}

/// Add uniform padding to an axis-aligned bounding box.
FIX_INLINE fixAABB fixAABB_Inflate( fixAABB a, fixed_t extension )
{
	fixVec3 radius = { extension, extension, extension };

	fixAABB out;
	out.lowerBound = fixVecSub( a.lowerBound, radius );
	out.upperBound = fixVecAdd( a.upperBound, radius );
	return out;
}

/// Do two axis-aligned boxes overlap?
FIX_INLINE bool fixAABB_Overlaps( fixAABB a, fixAABB b )
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

/// Is this a valid AABB? Both bounds finite, and lower <= upper on every axis.
FIX_INLINE bool fixIsValidAABB( fixAABB a )
{
	if ( fixIsValidVec3( a.lowerBound ) == false )
	{
		return false;
	}

	if ( fixIsValidVec3( a.upperBound ) == false )
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

/// Multiply a matrix times a column vector.
FIX_INLINE fixVec3 fixMulMV( fixMatrix3 m, fixVec3 a )
{
	// Kept as per-product rounding: the single-rounding form shifted the SAT
	// edge-query geometry by an ulp and put convex hull piles into a persistent
	// cache-miss regime (convex_pile +40%).
	fixVec3 b = {
		fixMul( m.cx.x , a.x ) + fixMul( m.cy.x , a.y ) + fixMul( m.cz.x , a.z ),
		fixMul( m.cx.y , a.x ) + fixMul( m.cy.y , a.y ) + fixMul( m.cz.y , a.z ),
		fixMul( m.cx.z , a.x ) + fixMul( m.cy.z , a.y ) + fixMul( m.cz.z , a.z ),
	};
	return b;
}

/// Negate a matrix.
FIX_INLINE fixMatrix3 fixNegateMat3( fixMatrix3 a )
{
	return FIX_LITERAL( fixMatrix3 ){
		{ -a.cx.x, -a.cx.y, -a.cx.z },
		{ -a.cy.x, -a.cy.y, -a.cy.z },
		{ -a.cz.x, -a.cz.y, -a.cz.z },
	};
}

/// Matrix addition.
/// @return a + b
FIX_INLINE fixMatrix3 fixAddMM( fixMatrix3 a, fixMatrix3 b )
{
	return FIX_LITERAL( fixMatrix3 ){
		{ a.cx.x + b.cx.x, a.cx.y + b.cx.y, a.cx.z + b.cx.z },
		{ a.cy.x + b.cy.x, a.cy.y + b.cy.y, a.cy.z + b.cy.z },
		{ a.cz.x + b.cz.x, a.cz.y + b.cz.y, a.cz.z + b.cz.z },
	};
}

/// Matrix subtraction.
/// @return a - b
FIX_INLINE fixMatrix3 fixSubMM( fixMatrix3 a, fixMatrix3 b )
{
	return FIX_LITERAL( fixMatrix3 ){
		{ a.cx.x - b.cx.x, a.cx.y - b.cx.y, a.cx.z - b.cx.z },
		{ a.cy.x - b.cy.x, a.cy.y - b.cy.y, a.cy.z - b.cy.z },
		{ a.cz.x - b.cz.x, a.cz.y - b.cz.y, a.cz.z - b.cz.z },
	};
}

/// Multiply a matrix by a scalar, component-wise.
FIX_INLINE fixMatrix3 fixMulSM( fixed_t s, fixMatrix3 a )
{
	return FIX_LITERAL( fixMatrix3 ){
		{ fixMul( s , a.cx.x ), fixMul( s , a.cx.y ), fixMul( s , a.cx.z ) },
		{ fixMul( s , a.cy.x ), fixMul( s , a.cy.y ), fixMul( s , a.cy.z ) },
		{ fixMul( s , a.cz.x ), fixMul( s , a.cz.y ), fixMul( s , a.cz.z ) },
	};
}

/// Matrix multiplication.
/// @return a * b
FIX_INLINE fixMatrix3 fixMulMM( fixMatrix3 a, fixMatrix3 b )
{
	fixMatrix3 out;
	out.cx = fixMulMV( a, b.cx );
	out.cy = fixMulMV( a, b.cy );
	out.cz = fixMulMV( a, b.cz );
	return out;
}

/// Matrix transpose.
FIX_INLINE fixMatrix3 fixTranspose( fixMatrix3 m )
{
	fixMatrix3 out;
	out.cx = FIX_LITERAL( fixVec3 ){ m.cx.x, m.cy.x, m.cz.x };
	out.cy = FIX_LITERAL( fixVec3 ){ m.cx.y, m.cy.y, m.cz.y };
	out.cz = FIX_LITERAL( fixVec3 ){ m.cx.z, m.cy.z, m.cz.z };

	return out;
}

/// Internal: the exact ratio ( cofactor << shift ) / determinant, at 256 bits.
///
/// The wide arm's counterpart to fixDivShifted, and the reason the wide arm exists: the
/// numerator reaches 158 bits and the denominator 190, so neither the shift nor the
/// division fits in 128. Truncated toward zero, matching fixInt128Div, and saturating on
/// a ratio outside Q48.16 range, matching fixDivShifted.
///
/// A ratio whose magnitude is below one quantum comes back as zero. That is the exact
/// answer and not a failure: the inverse of a matrix this large IS smaller than Q48.16
/// can hold, and the honest report of an unrepresentable inverse is that it rounds away.
/// Callers that care about the difference between "zero because static" and "zero because
/// too large to invert" test the input, not the output -- see fixed3d's b3UpdateBodyMassData.
FIX_INLINE fixed_t fixDivShifted256( fixUInt256 numerator, int shift, fixUInt256 absDeterminant, bool determinantNegative )
{
	bool numeratorNegative = fixInt256IsNegative( numerator );
	fixUInt256 scaled = fixUInt256Shl( fixInt256Abs( numerator ), shift );

	fixUInt256 quotient;
	fixUInt256 remainder;
	fixUInt256DivMod( scaled, absDeterminant, &quotient, &remainder );

	bool negative = numeratorNegative != determinantNegative;

	bool representable = fixUInt128Eq( quotient.hi, FIX_UINT128_ZERO ) &&
						 fixUInt128Le( quotient.lo, fixUInt128FromU64( (uint64_t)FIX_MAX ) );
	FIX_VALIDATE( representable );
	if ( representable == false )
	{
		return negative ? FIX_MIN : FIX_MAX;
	}

	int64_t magnitude = (int64_t)fixUInt128Lo( quotient.lo );
	return negative ? -magnitude : magnitude;
}

/// Internal: the same ratio for a numerator that is still only 128 bits wide, which is
/// every entry of the inverse. Widening first and dividing once keeps one implementation
/// of the rounding and the saturation rather than two that must be kept in step.
FIX_INLINE fixed_t fixDivShiftedWide( fixInt128 numerator, int shift, fixUInt256 absDeterminant, bool determinantNegative )
{
	bool numeratorNegative = fixInt128IsNegative( numerator );
	fixUInt128 absNumerator =
		numeratorNegative ? fixUInt128Neg( fixInt128ToUnsigned( numerator ) ) : fixInt128ToUnsigned( numerator );

	fixUInt256 widened = fixUInt256FromU128( absNumerator );
	return fixDivShifted256( numeratorNegative ? fixUInt256Neg( widened ) : widened, shift, absDeterminant, determinantNegative );
}

/// Internal: the exact inverse of a matrix whose cofactors are too large for 128-bit
/// arithmetic. Same value the 128-bit arm computes, carried at a width that cannot
/// overflow, so the two arms differ only in cost.
///
/// This replaces a reduction that dropped 16 fraction bits from each cofactor and then
/// cast the result to int64 to accumulate the determinant. Both steps were silent: the
/// cast wrapped for any cofactor past 2^79, and the determinant itself overflowed 128
/// bits shortly after, so the answer became a number of arbitrary sign rather than a
/// refusal. Nothing about the wide arm can wrap -- the determinant is exact at 256 bits,
/// every entry is the exact truncated ratio, and a ratio outside Q48.16 saturates.
///
/// The cofactors are recomputed rather than handed in. This arm runs at mass-data time,
/// where nine extra multiplies cost nothing, and a nine-argument signature would be
/// carried by the 128-bit arm as well.
FIX_INLINE fixMatrix3 fixInvertMatrixWide( fixMatrix3 m )
{
	fixInt128 c00 = fixCofactor128( m.cy.y, m.cz.z, m.cy.z, m.cz.y );
	fixInt128 c01 = fixCofactor128( m.cy.z, m.cz.x, m.cy.x, m.cz.z );
	fixInt128 c02 = fixCofactor128( m.cy.x, m.cz.y, m.cy.y, m.cz.x );
	fixInt128 c10 = fixCofactor128( m.cz.y, m.cx.z, m.cz.z, m.cx.y );
	fixInt128 c11 = fixCofactor128( m.cz.z, m.cx.x, m.cz.x, m.cx.z );
	fixInt128 c12 = fixCofactor128( m.cz.x, m.cx.y, m.cz.y, m.cx.x );
	fixInt128 c20 = fixCofactor128( m.cx.y, m.cy.z, m.cx.z, m.cy.y );
	fixInt128 c21 = fixCofactor128( m.cx.z, m.cy.x, m.cx.x, m.cy.z );
	fixInt128 c22 = fixCofactor128( m.cx.x, m.cy.y, m.cx.y, m.cy.x );

	// Determinant by expansion along the first row of the cofactor matrix, exact at
	// Q**.48 in 256 bits.
	fixUInt256 det = fixUInt256Add( fixUInt256Add( fixInt256MulI128ByI64( c00, m.cx.x ), fixInt256MulI128ByI64( c10, m.cy.x ) ),
									fixInt256MulI128ByI64( c20, m.cz.x ) );

	if ( fixUInt256IsZero( det ) )
	{
		return fixMat3_zero;
	}

	bool negative = fixInt256IsNegative( det );
	fixUInt256 absDet = fixInt256Abs( det );

	// inverse_ij = cofactor_ji / det: (Q32.32 << 32) / Q**.48 -> Q48.16
	fixMatrix3 out;
	out.cx = FIX_LITERAL( fixVec3 ){ fixDivShiftedWide( c00, 32, absDet, negative ), fixDivShiftedWide( c10, 32, absDet, negative ),
								   fixDivShiftedWide( c20, 32, absDet, negative ) };
	out.cy = FIX_LITERAL( fixVec3 ){ fixDivShiftedWide( c01, 32, absDet, negative ), fixDivShiftedWide( c11, 32, absDet, negative ),
								   fixDivShiftedWide( c21, 32, absDet, negative ) };
	out.cz = FIX_LITERAL( fixVec3 ){ fixDivShiftedWide( c02, 32, absDet, negative ), fixDivShiftedWide( c12, 32, absDet, negative ),
								   fixDivShiftedWide( c22, 32, absDet, negative ) };
	return out;
}

/// General matrix inverse.
///
/// REPRESENTABILITY. The result is Q48.16, so an inverse entry below one quantum
/// (1/65536) rounds to zero and an entry past FIX_MAX saturates. For an inertia tensor
/// that means a body whose principal inertia exceeds 65,536 units has no representable
/// inverse inertia at all and gets a zero matrix -- a uniform solid cube crosses that
/// line at about 13.1 units on a side. This is a property of the format, not of the
/// algorithm: both arms below compute the same exact ratio, and both round it the same
/// way. What changed is that they now say so instead of returning wrapped bits.
FIX_INLINE fixMatrix3 fixInvertMatrix( fixMatrix3 m )
{
	// Full precision cofactors (Q32.32 in 128 bits) so small matrices like the
	// inertia of tiny bodies stay invertible: a Q48.16 determinant underflows.
	fixInt128 c00 = fixCofactor128( m.cy.y, m.cz.z, m.cy.z, m.cz.y );
	fixInt128 c01 = fixCofactor128( m.cy.z, m.cz.x, m.cy.x, m.cz.z );
	fixInt128 c02 = fixCofactor128( m.cy.x, m.cz.y, m.cy.y, m.cz.x );
	fixInt128 c10 = fixCofactor128( m.cz.y, m.cx.z, m.cz.z, m.cx.y );
	fixInt128 c11 = fixCofactor128( m.cz.z, m.cx.x, m.cz.x, m.cx.z );
	fixInt128 c12 = fixCofactor128( m.cz.x, m.cx.y, m.cz.y, m.cx.x );
	fixInt128 c20 = fixCofactor128( m.cx.y, m.cy.z, m.cx.z, m.cy.y );
	fixInt128 c21 = fixCofactor128( m.cx.z, m.cy.x, m.cx.x, m.cy.z );
	fixInt128 c22 = fixCofactor128( m.cx.x, m.cy.y, m.cx.y, m.cy.x );

	// EVERY cofactor inside the limit, not just the three the determinant expands along.
	// The arm below casts all nine to int64 and shifts all nine left by 32, so a large
	// off-column cofactor overflows it just as surely as a large in-column one -- it was
	// simply never tested, because the three checked cofactors dominate for a symmetric
	// inertia tensor and hid the rest.
	fixInt128 limit = fixInt128ShiftLeft( fixInt128FromI64( 1 ), 62 );
	if ( fixInt128InRange( c00, limit ) && fixInt128InRange( c01, limit ) && fixInt128InRange( c02, limit ) &&
		 fixInt128InRange( c10, limit ) && fixInt128InRange( c11, limit ) && fixInt128InRange( c12, limit ) &&
		 fixInt128InRange( c20, limit ) && fixInt128InRange( c21, limit ) && fixInt128InRange( c22, limit ) )
	{
		// Exact path: cofactors fit in 64 bits, determinant at Q16.48. Every product
		// below is bounded by 2^125 and the sum of three by 2^127, so nothing here can
		// overflow -- the limit is what buys that.
		fixInt128 det = fixInt128Add( fixInt128Add( fixInt128MulI64( m.cx.x, fixInt128ToI64( c00 ) ),
												   fixInt128MulI64( m.cy.x, fixInt128ToI64( c10 ) ) ),
									  fixInt128MulI64( m.cz.x, fixInt128ToI64( c20 ) ) );
		if ( !fixInt128Eq( det, FIX_INT128_ZERO ) )
		{
			// inverse_ij = cofactor_ji / det: (Q32.32 << 32) / Q16.48 -> Q48.16
			fixMatrix3 out;
			out.cx = FIX_LITERAL( fixVec3 ){ fixDivShifted( c00, 32, det ), fixDivShifted( c10, 32, det ),
										   fixDivShifted( c20, 32, det ) };
			out.cy = FIX_LITERAL( fixVec3 ){ fixDivShifted( c01, 32, det ), fixDivShifted( c11, 32, det ),
										   fixDivShifted( c21, 32, det ) };
			out.cz = FIX_LITERAL( fixVec3 ){ fixDivShifted( c02, 32, det ), fixDivShifted( c12, 32, det ),
										   fixDivShifted( c22, 32, det ) };
			return out;
		}
		return fixMat3_zero;
	}

	// Anything larger goes wide, where the same ratio is computed at a width that cannot
	// overflow. The two arms agree exactly wherever both are defined, so which one runs
	// is a question of cost and never of answer.
	return fixInvertMatrixWide( m );
}

/// Internal: the exact solve of a system whose cofactors are too large for 128-bit
/// arithmetic. The wide counterpart of the direct solve, not of the inverse -- it keeps
/// the one-division-per-component form all the way out to the top of the range.
///
/// Each numerator is a cofactor (126 bits) against a component (64), summed three ways
/// and then shifted up 16, which reaches 208 bits. The determinant reaches 190. Both are
/// exact here, so the result is the same truncated rational the 128-bit arm computes and
/// the two arms differ only in cost.
FIX_INLINE fixVec3 fixSolve3Wide( fixMatrix3 m, fixVec3 a )
{
	fixInt128 c00 = fixCofactor128( m.cy.y, m.cz.z, m.cy.z, m.cz.y );
	fixInt128 c01 = fixCofactor128( m.cy.z, m.cz.x, m.cy.x, m.cz.z );
	fixInt128 c02 = fixCofactor128( m.cy.x, m.cz.y, m.cy.y, m.cz.x );
	fixInt128 c10 = fixCofactor128( m.cz.y, m.cx.z, m.cz.z, m.cx.y );
	fixInt128 c11 = fixCofactor128( m.cz.z, m.cx.x, m.cz.x, m.cx.z );
	fixInt128 c12 = fixCofactor128( m.cz.x, m.cx.y, m.cz.y, m.cx.x );
	fixInt128 c20 = fixCofactor128( m.cx.y, m.cy.z, m.cx.z, m.cy.y );
	fixInt128 c21 = fixCofactor128( m.cx.z, m.cy.x, m.cx.x, m.cy.z );
	fixInt128 c22 = fixCofactor128( m.cx.x, m.cy.y, m.cx.y, m.cy.x );

	fixUInt256 det = fixUInt256Add( fixUInt256Add( fixInt256MulI128ByI64( c00, m.cx.x ), fixInt256MulI128ByI64( c10, m.cy.x ) ),
									fixInt256MulI128ByI64( c20, m.cz.x ) );
	if ( fixUInt256IsZero( det ) )
	{
		return fixVec3_zero;
	}

	bool negative = fixInt256IsNegative( det );
	fixUInt256 absDet = fixInt256Abs( det );

	// Two's complement sums: a wrapping 256-bit add is the signed add, and nothing here
	// comes close to the width.
	fixUInt256 nx = fixUInt256Add( fixUInt256Add( fixInt256MulI128ByI64( c00, a.x ), fixInt256MulI128ByI64( c01, a.y ) ),
								   fixInt256MulI128ByI64( c02, a.z ) );
	fixUInt256 ny = fixUInt256Add( fixUInt256Add( fixInt256MulI128ByI64( c10, a.x ), fixInt256MulI128ByI64( c11, a.y ) ),
								   fixInt256MulI128ByI64( c12, a.z ) );
	fixUInt256 nz = fixUInt256Add( fixUInt256Add( fixInt256MulI128ByI64( c20, a.x ), fixInt256MulI128ByI64( c21, a.y ) ),
								   fixInt256MulI128ByI64( c22, a.z ) );

	fixVec3 b = {
		fixDivShifted256( nx, 16, absDet, negative ),
		fixDivShifted256( ny, 16, absDet, negative ),
		fixDivShifted256( nz, 16, absDet, negative ),
	};
	return b;
}

/// Solve a matrix equation.
/// @return inv(m) * a
/// Solves directly from the 128-bit cofactors with three divisions rather than
/// inverting (nine divisions) and multiplying.
FIX_INLINE fixVec3 fixSolve3( fixMatrix3 m, fixVec3 a )
{
	fixInt128 c00 = fixCofactor128( m.cy.y, m.cz.z, m.cy.z, m.cz.y );
	fixInt128 c01 = fixCofactor128( m.cy.z, m.cz.x, m.cy.x, m.cz.z );
	fixInt128 c02 = fixCofactor128( m.cy.x, m.cz.y, m.cy.y, m.cz.x );
	fixInt128 c10 = fixCofactor128( m.cz.y, m.cx.z, m.cz.z, m.cx.y );
	fixInt128 c11 = fixCofactor128( m.cz.z, m.cx.x, m.cz.x, m.cx.z );
	fixInt128 c12 = fixCofactor128( m.cz.x, m.cx.y, m.cz.y, m.cx.x );
	fixInt128 c20 = fixCofactor128( m.cx.y, m.cy.z, m.cx.z, m.cy.y );
	fixInt128 c21 = fixCofactor128( m.cx.z, m.cy.x, m.cx.x, m.cy.z );
	fixInt128 c22 = fixCofactor128( m.cx.x, m.cy.y, m.cx.y, m.cy.x );

	// EVERY cofactor inside the limit. The numerators below multiply all nine through
	// fixInt128ToI64, not just the three the determinant expands along, so the three-way
	// test this replaces let the other six wrap silently -- and unlike the inverse, this
	// runs per substep in fixed3d's gyroscopic solve, where a wrapped cofactor becomes an
	// angular velocity.
	fixInt128 limit = fixInt128ShiftLeft( fixInt128FromI64( 1 ), 62 );
	if ( fixInt128InRange( c00, limit ) && fixInt128InRange( c01, limit ) && fixInt128InRange( c02, limit ) &&
		 fixInt128InRange( c10, limit ) && fixInt128InRange( c11, limit ) && fixInt128InRange( c12, limit ) &&
		 fixInt128InRange( c20, limit ) && fixInt128InRange( c21, limit ) && fixInt128InRange( c22, limit ) )
	{
		// Exact path: cofactors fit in 64 bits, determinant at Q16.48
		fixInt128 det = fixInt128Add( fixInt128Add( fixInt128MulI64( m.cx.x, fixInt128ToI64( c00 ) ),
												   fixInt128MulI64( m.cy.x, fixInt128ToI64( c10 ) ) ),
									  fixInt128MulI64( m.cz.x, fixInt128ToI64( c20 ) ) );
		if ( !fixInt128Eq( det, FIX_INT128_ZERO ) )
		{
			// x_i = ( sum_j cofactor_ji * a_j ) / det: (Q32.32 * Q48.16 << 16) / Q16.48 -> Q48.16
			fixInt128 nx = fixInt128Add( fixInt128Add( fixInt128MulI64( fixInt128ToI64( c00 ), a.x ),
													  fixInt128MulI64( fixInt128ToI64( c01 ), a.y ) ),
										 fixInt128MulI64( fixInt128ToI64( c02 ), a.z ) );
			fixInt128 ny = fixInt128Add( fixInt128Add( fixInt128MulI64( fixInt128ToI64( c10 ), a.x ),
													  fixInt128MulI64( fixInt128ToI64( c11 ), a.y ) ),
										 fixInt128MulI64( fixInt128ToI64( c12 ), a.z ) );
			fixInt128 nz = fixInt128Add( fixInt128Add( fixInt128MulI64( fixInt128ToI64( c20 ), a.x ),
													  fixInt128MulI64( fixInt128ToI64( c21 ), a.y ) ),
										 fixInt128MulI64( fixInt128ToI64( c22 ), a.z ) );

			// The numerators are about to be shifted left 16 places, so they need room
			// for it. A cofactor inside the limit bounds each product by 2^125 and each
			// numerator by 2^127, which is NOT enough -- the shift needs 2^111. Simulation
			// values land far below this (a in-range angular residual keeps the
			// numerators near 2^102), so the wide arm is the rare case rather than the
			// working one, but "rare" and "impossible" are different words.
			fixInt128 shiftLimit = fixInt128ShiftLeft( fixInt128FromI64( 1 ), 110 );
			if ( fixInt128InRange( nx, shiftLimit ) && fixInt128InRange( ny, shiftLimit ) &&
				 fixInt128InRange( nz, shiftLimit ) )
			{
				fixVec3 b = {
					fixDivShifted( nx, 16, det ),
					fixDivShifted( ny, 16, det ),
					fixDivShifted( nz, 16, det ),
				};
				return b;
			}
		}
		else
		{
			return fixVec3_zero;
		}
	}

	// Anything larger solves wide, which is still one division per component and still
	// one rounding. Inverting and multiplying was the old fallback and it is a worse
	// answer for the same work: the inverse of a large matrix is a handful of quanta or
	// none at all, so rounding it to Q48.16 and then multiplying rounds twice through the
	// coarsest value in the calculation.
	return fixSolve3Wide( m, a );
}

/// Inverse transpose of a matrix. Identical to the inverse for the symmetric
/// matrices (like inertia tensors) this is used with.
FIX_INLINE fixMatrix3 fixInvertT( fixMatrix3 m )
{
	fixMatrix3 out = fixInvertMatrix( m );
	return fixTranspose( out );
}

/// Get the component-wise absolute value of a matrix.
FIX_INLINE fixMatrix3 fixAbsMatrix3( fixMatrix3 m )
{
	fixMatrix3 out;
	out.cx = fixVecAbs( m.cx );
	out.cy = fixVecAbs( m.cy );
	out.cz = fixVecAbs( m.cz );

	return out;
}

/// Make a matrix from a quaternion. This is useful if you need to
/// rotate many vectors.
/// The force inline improves the performance of b3ShapeDistance (box3d).
FIX_FORCE_INLINE fixMatrix3 fixMakeMatrixFromQuat( fixQuat q )
{
	fixed_t xx = fixMul( q.v.x , q.v.x );
	fixed_t yy = fixMul( q.v.y , q.v.y );
	fixed_t zz = fixMul( q.v.z , q.v.z );
	fixed_t xy = fixMul( q.v.x , q.v.y );
	fixed_t xz = fixMul( q.v.x , q.v.z );
	fixed_t xw = fixMul( q.v.x , q.s );
	fixed_t yz = fixMul( q.v.y , q.v.z );
	fixed_t yw = fixMul( q.v.y , q.s );
	fixed_t zw = fixMul( q.v.z , q.s );

	return FIX_LITERAL( fixMatrix3 ){
		{ FIX( 1.0f ) - fixMul( FIX( 2.0f ) , ( yy + zz ) ), fixMul( FIX( 2.0f ) , ( xy + zw ) ), fixMul( FIX( 2.0f ) , ( xz - yw ) ) },
		{ fixMul( FIX( 2.0f ) , ( xy - zw ) ), FIX( 1.0f ) - fixMul( FIX( 2.0f ) , ( xx + zz ) ), fixMul( FIX( 2.0f ) , ( yz + xw ) ) },
		{ fixMul( FIX( 2.0f ) , ( xz + yw ) ), fixMul( FIX( 2.0f ) , ( yz - xw ) ), FIX( 1.0f ) - fixMul( FIX( 2.0f ) , ( xx + yy ) ) },
	};
}

/// Transform an axis-aligned bounding box. This can create a larger box than if you
/// recomputed the AABB of the original shape with the transform applied.
///
/// Defined here rather than beside the other AABB operations because it needs
/// fixTransformPoint, fixMakeMatrixFromQuat, fixAbsMatrix3 and fixMulMV, all of which are
/// declared further down this header than the AABB block.
FIX_INLINE fixAABB fixAABB_Transform( fixTransform transform, fixAABB a )
{
	fixVec3 center = fixTransformPoint( transform, fixAABB_Center( a ) );
	fixMatrix3 m = fixMakeMatrixFromQuat( transform.q );
	fixVec3 extent = fixMulMV( fixAbsMatrix3( m ), fixAABB_Extents( a ) );
	fixAABB out = { fixVecSub( center, extent ), fixVecAdd( center, extent ) };
	return out;
}

/// Get the closest point on an axis-aligned bounding box.
FIX_INLINE fixVec3 fixClosestPointToAABB( fixVec3 point, fixAABB a )
{
	return fixVecClamp( point, a.lowerBound, a.upperBound );
}
