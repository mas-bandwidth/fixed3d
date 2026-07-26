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

#if defined( BOX3D_LUDICROUS_MODE )

/// A world position in ludicrous mode: Q112.16 fixed point in a 128-bit
/// integer. Same 16 fraction bits as b3Fixed (so the boundary subtract to local
/// space is an exact truncation, not a rescale), with all 64 extra bits going to
/// integer *range* (±2.6e33 units, far past a light-year in metres). Uniform
/// 15-micron resolution — the thesis — is preserved exactly at any distance. A
/// distinct struct on purpose: every world/local coordinate crossing that is not
/// routed through the boundary vocabulary becomes a compile error.
typedef struct b3Pos
{
	b3Int128 x;
	b3Int128 y;
	b3Int128 z;
} b3Pos;

/// A world transform: 128-bit translation, narrow (b3Fixed) rotation. Rotation is
/// frame-local and never needs range, so the quaternion stays Q48.16.
typedef struct b3WorldTransform
{
	b3Pos p;
	b3Quat q;
} b3WorldTransform;

#else

/// A world position. Fixed point has uniform precision everywhere, so world
/// positions use the same representation as local vectors.
typedef b3Vec3 b3Pos;

/// A world transform. Same representation as a local transform in fixed point.
typedef b3Transform b3WorldTransform;

#endif

/// Axis aligned bounding box.
#if defined( BOX3D_LUDICROUS_MODE )
/// Ludicrous mode: AABB bounds are 128-bit world positions, so the broadphase tree is
/// collision-active across the full wide-position range (past a light-year). Puts int128
/// into the hot tree. Nobody needs this; it exists to be measured (and marveled at).
typedef struct b3AABB
{
	b3Pos lowerBound;
	b3Pos upperBound;
} b3AABB;
#else
typedef struct b3AABB
{
	b3Vec3 lowerBound;
	b3Vec3 upperBound;
} b3AABB;
#endif

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
#if defined( BOX3D_LUDICROUS_MODE )
/// Wide build: the two-rounding form ((1-t)*a + t*b) would multiply b3FixMul by
/// an absolute 128-bit coordinate, which overflows and truncates. Reformulate as
/// a + t*(b-a): the difference (b-a) is in local range, so the multiply is a
/// safe b3FixMul on b3Fixed, and the result adds back onto the 128-bit base. This
/// is one rounding instead of two, so it is NOT bit-identical to the narrow build
/// (the wide build carries its own goldens).
B3_INLINE b3Pos b3LerpPosition( b3Pos a, b3Pos b, b3Fixed t )
{
	return B3_LITERAL( b3Pos ){
		a.x + b3FixMul( t , (b3Fixed)( b.x - a.x ) ),
		a.y + b3FixMul( t , (b3Fixed)( b.y - a.y ) ),
		a.z + b3FixMul( t , (b3Fixed)( b.z - a.z ) ),
	};
}
#else
B3_INLINE b3Pos b3LerpPosition( b3Pos a, b3Pos b, b3Fixed t )
{
	return B3_LITERAL( b3Pos ){
		b3FixMul( ( B3_FIX( 1.0f ) - t ) , a.x ) + b3FixMul( t , b.x ),
		b3FixMul( ( B3_FIX( 1.0f ) - t ) , a.y ) + b3FixMul( t , b.y ),
		b3FixMul( ( B3_FIX( 1.0f ) - t ) , a.z ) + b3FixMul( t , b.z ),
	};
}
#endif

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
// Ludicrous mode: AABB bounds are 128-bit. Storage, min/max, union, contains and overlap stay
// 128-bit (the hot tree ops); extent/center narrow to b3Fixed for the b3Vec3-returning API
// (exact whenever the box fits local range, which every in-range scene does).
B3_FIXED_INLINE b3Int128 b3W_min128( b3Int128 a, b3Int128 b ) { return a < b ? a : b; }
B3_FIXED_INLINE b3Int128 b3W_max128( b3Int128 a, b3Int128 b ) { return a > b ? a : b; }

B3_INLINE b3AABB b3MakeAABB( const b3Vec3* points, int count, b3Fixed radius )
{
	B3_ASSERT( count > 0 );
	b3AABB a = { { points[0].x, points[0].y, points[0].z }, { points[0].x, points[0].y, points[0].z } };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound.x = b3W_min128( a.lowerBound.x, points[i].x );
		a.lowerBound.y = b3W_min128( a.lowerBound.y, points[i].y );
		a.lowerBound.z = b3W_min128( a.lowerBound.z, points[i].z );
		a.upperBound.x = b3W_max128( a.upperBound.x, points[i].x );
		a.upperBound.y = b3W_max128( a.upperBound.y, points[i].y );
		a.upperBound.z = b3W_max128( a.upperBound.z, points[i].z );
	}
	a.lowerBound.x -= radius; a.lowerBound.y -= radius; a.lowerBound.z -= radius;
	a.upperBound.x += radius; a.upperBound.y += radius; a.upperBound.z += radius;
	return a;
}

B3_INLINE bool b3AABB_Contains( b3AABB a, b3AABB b )
{
	if ( a.lowerBound.x > b.lowerBound.x || b.upperBound.x > a.upperBound.x ) return false;
	if ( a.lowerBound.y > b.lowerBound.y || b.upperBound.y > a.upperBound.y ) return false;
	if ( a.lowerBound.z > b.lowerBound.z || b.upperBound.z > a.upperBound.z ) return false;
	return true;
}

B3_INLINE b3Fixed b3AABB_Area( b3AABB a )
{
	b3Fixed dx = (b3Fixed)( a.upperBound.x - a.lowerBound.x );
	b3Fixed dy = (b3Fixed)( a.upperBound.y - a.lowerBound.y );
	b3Fixed dz = (b3Fixed)( a.upperBound.z - a.lowerBound.z );
	return b3FixMul( B3_FIX( 2.0f ) , ( b3FixMul( dx , dy ) + b3FixMul( dy , dz ) + b3FixMul( dz , dx ) ) );
}

B3_INLINE b3Vec3 b3AABB_Center( b3AABB a )
{
	// narrow the bounds to b3Fixed (exact in local range) then use the SAME half-up rounding
	// as the narrow build's b3MulSV(0.5, ...) — integer /2 would truncate and shift the box.
	return b3MulSV( B3_FIX( 0.5f ), b3Add( b3ToVec3( a.upperBound ), b3ToVec3( a.lowerBound ) ) );
}

B3_INLINE b3Vec3 b3AABB_Extents( b3AABB a )
{
	return b3MulSV( B3_FIX( 0.5f ), b3Sub( b3ToVec3( a.upperBound ), b3ToVec3( a.lowerBound ) ) );
}

B3_INLINE b3AABB b3AABB_Union( b3AABB a, b3AABB b )
{
	b3AABB out;
	out.lowerBound.x = b3W_min128( a.lowerBound.x, b.lowerBound.x );
	out.lowerBound.y = b3W_min128( a.lowerBound.y, b.lowerBound.y );
	out.lowerBound.z = b3W_min128( a.lowerBound.z, b.lowerBound.z );
	out.upperBound.x = b3W_max128( a.upperBound.x, b.upperBound.x );
	out.upperBound.y = b3W_max128( a.upperBound.y, b.upperBound.y );
	out.upperBound.z = b3W_max128( a.upperBound.z, b.upperBound.z );
	return out;
}

B3_INLINE b3AABB b3AABB_Inflate( b3AABB a, b3Fixed extension )
{
	b3AABB out = a;
	out.lowerBound.x -= extension; out.lowerBound.y -= extension; out.lowerBound.z -= extension;
	out.upperBound.x += extension; out.upperBound.y += extension; out.upperBound.z += extension;
	return out;
}

B3_INLINE bool b3AABB_Overlaps( b3AABB a, b3AABB b )
{
	if ( a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x ) return false;
	if ( a.upperBound.y < b.lowerBound.y || a.lowerBound.y > b.upperBound.y ) return false;
	if ( a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z ) return false;
	return true;
}

B3_INLINE b3AABB b3AABB_Transform( b3Transform transform, b3AABB a )
{
	b3Vec3 center = b3TransformPoint( transform, b3AABB_Center( a ) );
	b3Matrix3 m = b3MakeMatrixFromQuat( transform.q );
	b3Vec3 extent = b3MulMV( b3AbsMatrix3( m ), b3AABB_Extents( a ) );
	b3Vec3 lo = b3Sub( center, extent ), hi = b3Add( center, extent );
	b3AABB out = { { lo.x, lo.y, lo.z }, { hi.x, hi.y, hi.z } };
	return out;
}

B3_INLINE b3Vec3 b3ClosestPointToAABB( b3Vec3 point, b3AABB a )
{
	b3Vec3 lo = { (b3Fixed)a.lowerBound.x, (b3Fixed)a.lowerBound.y, (b3Fixed)a.lowerBound.z };
	b3Vec3 hi = { (b3Fixed)a.upperBound.x, (b3Fixed)a.upperBound.y, (b3Fixed)a.upperBound.z };
	return b3Clamp( point, lo, hi );
}
#else
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

