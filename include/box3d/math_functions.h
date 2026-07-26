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

/// Convenience macro to convert from degrees to radians.
#define B3_DEG_TO_RAD B3_FIX( 0.01745329251f )

/// Convenience macro to convert from radians to degrees.
#define B3_RAD_TO_DEG B3_FIX( 57.2957795131f )

/// Minimum scale used for scaling collision meshes, etc.
// The vector, quaternion, matrix, transform and plane types and their arithmetic
// come from mas-bandwidth/fixed via box3d/fixed_compat.h, reached through
// base.h -> fixed.h. Box3D names, signatures, layouts and linkage unchanged:
// this fork stays drop-in compatible with Box3D by design.
//
// STILL DEFINED HERE, and each for a reason:
//   - the B3_API declarations: base.h includes fixed.h BEFORE defining B3_API,
//     so an exported declaration cannot live in the early-included compat header.
//   - the WORLD-POSITION family (b3Pos, b3WorldTransform, b3SubPos, b3OffsetPos,
//     b3LerpPosition, b3ToPos/b3ToVec3, b3*WorldTransform*): these change shape
//     with BOX3D_LUDICROUS_MODE. `fixed` exports narrow and wide variants under
//     distinct names; box3d needs ONE name whose meaning follows its own flag,
//     so the selection stays here rather than being aliased.
//   - the AABB family: moving in its own commit, because it carries the one
//     behaviour change in the extraction and its goldens deserve their own diff.
//   - box3d's own: Bound crossing vocabulary, segment/line distance, b3Steiner,
//     and the world-scale predicates that read b3GetLengthUnitsPerMeter().

#if defined( BOX3D_DOUBLE_PRECISION )
// Fixed point has uniform absolute precision across the whole world, so the
// double precision large-world mode is unnecessary and no longer supported.
#error "BOX3D_DOUBLE_PRECISION is not supported with fixed-point math"
#endif
// The world-position family (b3Pos, b3WorldTransform and their arithmetic) and the
// AABB family now come from mas-bandwidth/fixed via box3d/fixed_compat.h, with the
// width chosen there next to BOX3D_LUDICROUS_MODE.
//
// They moved TOGETHER because they are coupled: the wide b3AABB is built from
// b3Pos, so moving one while the other stayed gave "assigning to fixPosWide from
// incompatible type b3Pos". The narrow build compiled fine either way, which is
// why this only surfaced in the ludicrous configuration.
//
// STILL HERE, and each on purpose: box3d's Bound crossing vocabulary
// (b3BoundToPos, b3PosToBound, b3BoundToVec3, b3Vec3ToBound, b3OffsetAABB), which
// marks a boundary rather than doing arithmetic; b3IsBoundedAABB and b3IsSaneAABB,
// which resolve B3_HUGE through the mutable b3GetLengthUnitsPerMeter() global and
// so encode world-scale POLICY; and the segment/line distance queries and
// b3Steiner, which are physics.
// Valid in both modes: 0.0f promotes to double, the identity rotation stays b3Fixed
static const b3Pos b3Pos_zero = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
static const b3WorldTransform b3WorldTransform_identity = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 1.0f ) } };

// Float utilities for rendering, UI, and other non-simulation code. The
// simulation itself never uses these (see the fixed-point conversion notes).

// b3FixAbs, b3FixMin, b3FixMax, and b3FixClamp live in fixed.h

/// Compute an approximate arctangent in the range [-pi, pi]
/// This is hand coded for cross-platform determinism. The atan2f
/// function in the standard library is not cross-platform deterministic.
///	Accurate to around 0.0023 degrees.
B3_API b3Fixed b3Atan2( b3Fixed y, b3Fixed x );

/// Compute the cosine and sine of an angle in radians. Implemented
/// for cross-platform determinism.
B3_API b3CosSin b3ComputeCosSin( b3Fixed radians );

/// Extract a quaternion from a rotation matrix.
B3_API b3Quat b3MakeQuatFromMatrix( const b3Matrix3* m );

/// Find a quaternion that rotates one vector to another.
B3_API b3Quat b3ComputeQuatBetweenUnitVectors( b3Vec3 v1, b3Vec3 v2 );

/// Bridge b3Vec3-based math to the AABB bound storage type. Bounds are 128-bit b3Pos in
/// ludicrous mode (BOX3D_LUDICROUS_MODE) and b3Vec3 in the default build, so these are the
/// identity there. Bound-touching code uses them to compile in both builds without #if
/// (analogous to b3ToPos/b3ToVec3 for positions, but marking a BOUND crossing — keep the
/// distinction so bound flows stay auditable).
#if defined( BOX3D_LUDICROUS_MODE )
B3_INLINE b3Vec3 b3BoundToVec3( b3Pos p ) { return b3ToVec3( p ); }
B3_INLINE b3Pos b3Vec3ToBound( b3Vec3 v ) { return b3ToPos( v ); }
/// Bound <-> world position: the identity (bounds ARE b3Pos here).
B3_INLINE b3Pos b3BoundToPos( b3Pos p ) { return p; }
B3_INLINE b3Pos b3PosToBound( b3Pos p ) { return p; }
#else
B3_INLINE b3Vec3 b3BoundToVec3( b3Vec3 v ) { return v; }
B3_INLINE b3Vec3 b3Vec3ToBound( b3Vec3 v ) { return v; }
/// Bound <-> world position: also the identity (b3Pos aliases b3Vec3 in the default build).
B3_INLINE b3Pos b3BoundToPos( b3Vec3 v ) { return b3ToPos( v ); }
B3_INLINE b3Vec3 b3PosToBound( b3Pos p ) { return b3ToVec3( p ); }
#endif

/// World position interpolation for sweeps and sampling.
#if defined( BOX3D_LUDICROUS_MODE )
#else
#endif

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

#if B3_HAS_INT128
// Internal: 3x3 cofactors at Q32.32 in 128 bits and the determinant at Q16.48.
// The Q48.16 determinant of a matrix with small entries (like the inertia of a
// small body) underflows to zero, so the inverse and solve helpers work at
// full precision internally.
#endif

/// Get the inertia tensor of an offset point.
/// https://en.wikipedia.org/wiki/Parallel_axis_theorem
B3_API b3Matrix3 b3Steiner( b3Fixed mass, b3Vec3 origin );

#if defined( BOX3D_LUDICROUS_MODE )
#else
#endif

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


