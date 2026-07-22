// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "fixed.h"

// The fixed-point vector/quat/matrix/transform types (b3Vec2/3, b3Quat, b3Transform,
// b3Matrix3, b3Pos, b3WorldTransform) live in the vendored `fixed` library.
#include "fixed/fixed_vec.h"

// The scalar fixed-point transcendentals (b3CosSin, b3Atan2, b3ComputeCosSin, b3Sin,
// b3Cos, b3UnwindAngle) live in the vendored `fixed` library, not here. Included after
// box3d/base.h so box3d's B3_API (with its export decoration) wins.
#include "fixed/fixed_math.h"

#include <stdbool.h>

/**
 * @defgroup math Math
 * @brief Vector math types and functions
 * @{
 */

/// https://en.wikipedia.org/wiki/Pi

/// Convenience macro to convert from degrees to radians.
#define B3_DEG_TO_RAD B3_FIX( 0.01745329251f )

/// Convenience macro to convert from radians to degrees.
#define B3_RAD_TO_DEG B3_FIX( 57.2957795131f )

/// Minimum scale used for scaling collision meshes, etc.

// b3Vec2, b3Vec3, b3Quat, b3Transform, b3Matrix3, b3Pos, b3WorldTransform, and b3CosSin
// are provided by the vendored `fixed` library (fixed/fixed_vec.h + fixed/fixed_math.h,
// included above). The ops on these types remain below and are migrating to `fixed`
// incrementally.

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

// The vector/quaternion/matrix/transform/position CONSTANTS and OPS (b3Vec3_zero,
// b3Add..b3NLerp, transforms, world-position helpers, b3MakeQuatFromMatrix,
// b3ComputeQuatBetweenUnitVectors) live in the vendored `fixed` library
// (fixed/fixed_vec.h, included above), as do B3_PI and B3_MIN_SCALE. The AABB/
// plane geometry below stays here; remaining interleaved matrix ops migrate in a
// follow-up pass.
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
	// Kept as per-product rounding: the single-rounding form shifted the SAT
	// edge-query geometry by an ulp and put convex hull piles into a persistent
	// cache-miss regime (convex_pile +40%). See the round-3 notes in CLAUDE.md.
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
			out.cx = B3_LITERAL( b3Vec3 ){ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c00, 32 ), det ),
										   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c10, 32 ), det ),
										   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c20, 32 ), det ) };
			out.cy = B3_LITERAL( b3Vec3 ){ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c01, 32 ), det ),
										   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c11, 32 ), det ),
										   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c21, 32 ), det ) };
			out.cz = B3_LITERAL( b3Vec3 ){ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c02, 32 ), det ),
										   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c12, 32 ), det ),
										   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c22, 32 ), det ) };
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
		// b3Int128ShiftLeft: the raw << the float era used here is UB for the
		// negative cofactors this path exists for (same bits, defined behavior)
		b3Matrix3 out;
		out.cx = B3_LITERAL( b3Vec3 ){ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c00, 16 ), det ),
									   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c10, 16 ), det ),
									   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c20, 16 ), det ) };
		out.cy = B3_LITERAL( b3Vec3 ){ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c01, 16 ), det ),
									   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c11, 16 ), det ),
									   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c21, 16 ), det ) };
		out.cz = B3_LITERAL( b3Vec3 ){ (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c02, 16 ), det ),
									   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c12, 16 ), det ),
									   (b3Fixed)b3Int128Div( b3Int128ShiftLeft( c22, 16 ), det ) };
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
				(b3Fixed)b3Int128Div( b3Int128ShiftLeft( nx, 16 ), det ),
				(b3Fixed)b3Int128Div( b3Int128ShiftLeft( ny, 16 ), det ),
				(b3Fixed)b3Int128Div( b3Int128ShiftLeft( nz, 16 ), det ),
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
