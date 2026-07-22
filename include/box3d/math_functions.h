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

// The 3x3 matrix ops (b3Det..b3AbsMatrix3, including the 128-bit cofactor
// inverse/solve) live in the vendored `fixed` library (fixed/fixed_vec.h).
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
