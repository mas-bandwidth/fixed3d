// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
// Wide (Q112.16) fixed-point primitives: 128-bit world coordinates that share
// fixed_t's 16 fraction bits. Sharing the fraction count is the crux — the
// boundary between wide world space and Q48.16 local space is then an exact
// integer subtract plus a range check, never an arithmetic rescale. See
// fixed3d's docs/design/wide-world-positions.md for the architecture these
// primitives serve.
#pragma once

#include "fixed/fixed.h"
#include "fixed/fixed_vec.h"

/// Wide fixed-point scalar: Q112.16 in a 128-bit integer. Same resolution as
/// fixed_t (1/65536); all 64 extra bits go to integer range (~±2.6e33 units).
///
/// On a compiler with __int128 this is that type and the operators work on it directly.
/// On plain MSVC it is the emulated pair from fixed_int128.h and the operators do not,
/// so PORTABLE CONSUMER CODE USES THE FUNCTIONS BELOW rather than `+` and `<`. Every
/// operation this header needs has a named form for exactly that reason, comparisons
/// included.
typedef fixInt128 fixedWide_t;

/// Wide world position: three Q112.16 coordinates.
typedef struct fixPosWide
{
	fixedWide_t x, y, z;
} fixPosWide;

/// Widen a local Q48.16 value to Q112.16. Exact: the fraction points align.
FIX_ALWAYS_INLINE fixedWide_t fixWideFromFixed( fixed_t a )
{
	return fixInt128FromI64( a );
}

/// Narrow a Q112.16 value to local Q48.16, saturating out-of-range values to
/// INT64_MAX/INT64_MIN. Exact whenever the value fits local range.
FIX_ALWAYS_INLINE fixed_t fixWideToFixed( fixedWide_t a )
{
	if ( fixInt128Gt( a, fixInt128FromI64( INT64_MAX ) ) )
	{
		return INT64_MAX;
	}
	if ( fixInt128Lt( a, fixInt128FromI64( INT64_MIN ) ) )
	{
		return INT64_MIN;
	}
	return (fixed_t)fixInt128ToI64( a );
}

/// Wide add. Exact 128-bit integer addition.
FIX_ALWAYS_INLINE fixedWide_t fixWideAdd( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Add( a, b );
}

/// Wide subtract. Exact 128-bit integer subtraction.
FIX_ALWAYS_INLINE fixedWide_t fixWideSub( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Sub( a, b );
}

/// Wide comparison. These exist because `<` does not compile on the emulated
/// representation: a consumer that compares wide coordinates portably needs a name.
FIX_ALWAYS_INLINE bool fixWideEq( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Eq( a, b );
}

FIX_ALWAYS_INLINE bool fixWideLt( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Lt( a, b );
}

FIX_ALWAYS_INLINE bool fixWideGt( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Gt( a, b );
}

FIX_ALWAYS_INLINE bool fixWideLe( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Le( a, b );
}

FIX_ALWAYS_INLINE bool fixWideGe( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Ge( a, b );
}

/// Wide negation.
FIX_ALWAYS_INLINE fixedWide_t fixWideNeg( fixedWide_t a )
{
	return fixInt128Neg( a );
}

/// Min/max on the wide (128-bit) fixed-point type.
///
/// Extracted from fixed3d, where they live behind BOX3D_LUDICROUS_MODE because that is
/// the only build with 128-bit AABB bounds. Here they are unconditional: this library
/// exports the wide type on every build, so a consumer selects narrow or wide by which
/// type it uses, not by a compile flag that changes an ABI.
FIX_ALWAYS_INLINE fixedWide_t fixWideMin( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Lt( a, b ) ? a : b;
}

FIX_ALWAYS_INLINE fixedWide_t fixWideMax( fixedWide_t a, fixedWide_t b )
{
	return fixInt128Gt( a, b ) ? a : b;
}

/// Offset a wide coordinate by a local delta (the once-per-step delta-fold).
/// Exact: int128 += widened int64, fraction points aligned.
FIX_ALWAYS_INLINE fixedWide_t fixWideOffset( fixedWide_t a, fixed_t d )
{
	return fixInt128Add( a, fixInt128FromI64( d ) );
}

/// The boundary operation: difference two wide world coordinates into local
/// Q48.16 space. Exact whenever the separation fits local range (any contact
/// pair, joint, or reach-bounded query); saturates otherwise. This is the
/// fixed-point replacement for float's entire directed-rounding apparatus.
FIX_ALWAYS_INLINE fixed_t fixWideSubToFixed( fixedWide_t a, fixedWide_t b )
{
	return fixWideToFixed( fixInt128Sub( a, b ) );
}

/// Widen a local position/vector to a wide world position. Exact.
FIX_ALWAYS_INLINE fixPosWide fixPosWideFromVec3( fixVec3 v )
{
	fixPosWide p = { fixInt128FromI64( v.x ), fixInt128FromI64( v.y ), fixInt128FromI64( v.z ) };
	return p;
}

/// Narrow a wide position to a local vector, saturating per component.
FIX_ALWAYS_INLINE fixVec3 fixPosWideToVec3( fixPosWide p )
{
	fixVec3 v = { fixWideToFixed( p.x ), fixWideToFixed( p.y ), fixWideToFixed( p.z ) };
	return v;
}

/// Difference two wide positions into local space (per-component boundary op).
FIX_ALWAYS_INLINE fixVec3 fixPosWideSub( fixPosWide a, fixPosWide b )
{
	fixVec3 v = { fixWideSubToFixed( a.x, b.x ), fixWideSubToFixed( a.y, b.y ), fixWideSubToFixed( a.z, b.z ) };
	return v;
}

/// Offset a wide position by a local delta vector. Exact.
FIX_ALWAYS_INLINE fixPosWide fixPosWideOffset( fixPosWide p, fixVec3 d )
{
	fixPosWide out = { fixWideOffset( p.x, d.x ), fixWideOffset( p.y, d.y ), fixWideOffset( p.z, d.z ) };
	return out;
}

/// A world transform with a wide translation. Rotation is frame-local and never needs
/// range, so the quaternion stays Q48.16.
typedef struct fixWorldTransformWide
{
	fixPosWide p;
	fixQuat q;
} fixWorldTransformWide;

/// Is this a valid wide coordinate? Mirrors fixIsValidFixed: every value is a legal
/// quantity except the 128-bit minimum, which is reserved so negation cannot overflow.
FIX_ALWAYS_INLINE bool fixIsValidWideCoord( fixedWide_t x )
{
	return !fixInt128Eq( x, fixInt128Make( UINT64_C( 0x8000000000000000 ), 0 ) );
}

FIX_ALWAYS_INLINE bool fixIsValidPosWide( fixPosWide p )
{
	return fixIsValidWideCoord( p.x ) && fixIsValidWideCoord( p.y ) && fixIsValidWideCoord( p.z );
}

FIX_ALWAYS_INLINE bool fixIsValidWorldTransformWide( fixWorldTransformWide t )
{
	return fixIsValidPosWide( t.p ) && fixIsValidQuat( t.q );
}

// ---- the wide world-position family ------------------------------------------------
//
// Ported from box3d's ludicrous build. The narrow forms live in fixed_vec.h and take
// fixPos; these take fixPosWide, which is the whole reason they exist -- a consumer whose
// world positions are 128-bit cannot call the narrow ones, and that gap is what stopped
// box3d's wide configuration from consuming this library at all.
//
// Note what is NOT here because it already exists above: fixPosWideSub is the wide
// SubPos, fixPosWideOffset is the wide OffsetPos, and fixPosWideFromVec3/ToVec3 are the
// wide ToPos/ToVec3. Adding second names for them would be two spellings of one truth.

/// World position interpolation, wide.
///
/// NOT a mechanical widening of the narrow form, and the difference is load-bearing.
/// The narrow build computes (1-t)*a + t*b. Widened directly that multiplies an
/// ABSOLUTE 128-bit coordinate, which overflows and truncates. Reformulated as
/// a + t*(b-a): the difference is in local range, so the multiply is a safe fixMul on
/// fixed_t and the result adds back onto the 128-bit base.
///
/// That is ONE rounding instead of two, so it is deliberately NOT bit-identical to the
/// narrow build. A consumer running both widths carries separate goldens for this, and
/// no amount of care will make them agree -- the wide answer is the more accurate one.
FIX_ALWAYS_INLINE fixPosWide fixLerpPositionWide( fixPosWide a, fixPosWide b, fixed_t t )
{
	fixPosWide out = {
		fixWideOffset( a.x, fixMul( t, fixWideSubToFixed( b.x, a.x ) ) ),
		fixWideOffset( a.y, fixMul( t, fixWideSubToFixed( b.y, a.y ) ) ),
		fixWideOffset( a.z, fixMul( t, fixWideSubToFixed( b.z, a.z ) ) ),
	};
	return out;
}

/// Transform a local point into wide world space. Rotation is narrow; only the
/// translation is wide.
FIX_ALWAYS_INLINE fixPosWide fixTransformWorldPointWide( fixWorldTransformWide t, fixVec3 p )
{
	fixVec3 r = fixRotateVector( t.q, p );
	return fixPosWideOffset( t.p, r );
}

/// Transform a wide world position into local space. One wide subtraction, then narrow.
FIX_ALWAYS_INLINE fixVec3 fixInvTransformWorldPointWide( fixWorldTransformWide t, fixPosWide p )
{
	return fixInvRotateVector( t.q, fixPosWideSub( p, t.p ) );
}

/// Promote a local transform to a wide world transform. Lossless.
FIX_ALWAYS_INLINE fixWorldTransformWide fixMakeWorldTransformWide( fixTransform t )
{
	fixWorldTransformWide w = { fixPosWideFromVec3( t.p ), t.q };
	return w;
}

/// Compose a wide world transform with a local transform.
FIX_ALWAYS_INLINE fixWorldTransformWide fixMulWorldTransformsWide( fixWorldTransformWide A, fixTransform B )
{
	fixWorldTransformWide C;
	C.q = fixMulQuat( A.q, B.q );
	C.p = fixPosWideOffset( A.p, fixRotateVector( A.q, B.p ) );
	return C;
}

/// Relative transform of frame B in frame A -- the narrow-phase boundary. The wide
/// difference lands in local range for any pair close enough to interact, which is the
/// entire premise of the wide-position design.
FIX_ALWAYS_INLINE fixTransform fixInvMulWorldTransformsWide( fixWorldTransformWide A, fixWorldTransformWide B )
{
	fixTransform C;
	C.q = fixInvMulQuat( A.q, B.q );
	C.p = fixInvRotateVector( A.q, fixPosWideSub( B.p, A.p ) );
	return C;
}

/// Shift a wide world transform into the frame of a base position.
FIX_ALWAYS_INLINE fixTransform fixToRelativeTransformWide( fixWorldTransformWide t, fixPosWide base )
{
	fixTransform r = { fixPosWideSub( t.p, base ), t.q };
	return r;
}

/// An axis-aligned bounding box in wide (Q112.16) world space.
///
/// The narrow counterpart is fixAABB in fixed_vec.h. Storage, min/max, union, contains
/// and overlap stay 128-bit -- those are the hot broadphase-tree operations. Center and
/// extents narrow to fixed_t for the fixVec3-returning API, which is exact whenever the
/// box fits local range.
typedef struct fixAABBWide
{
	fixPosWide lowerBound;
	fixPosWide upperBound;
} fixAABBWide;

/// Get the wide AABB of a wide point cloud, expanded by a uniform local radius.
FIX_ALWAYS_INLINE fixAABBWide fixMakeAABBWide( const fixPosWide* points, int count, fixed_t radius )
{
	FIX_ASSERT( count > 0 );
	fixAABBWide a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound.x = fixWideMin( a.lowerBound.x, points[i].x );
		a.lowerBound.y = fixWideMin( a.lowerBound.y, points[i].y );
		a.lowerBound.z = fixWideMin( a.lowerBound.z, points[i].z );
		a.upperBound.x = fixWideMax( a.upperBound.x, points[i].x );
		a.upperBound.y = fixWideMax( a.upperBound.y, points[i].y );
		a.upperBound.z = fixWideMax( a.upperBound.z, points[i].z );
	}

	// Subtracting the widened radius rather than offsetting by its negation: the two agree
	// everywhere except at INT64_MIN, where negating a fixed_t has no value.
	fixedWide_t wideRadius = fixWideFromFixed( radius );
	a.lowerBound.x = fixWideSub( a.lowerBound.x, wideRadius );
	a.lowerBound.y = fixWideSub( a.lowerBound.y, wideRadius );
	a.lowerBound.z = fixWideSub( a.lowerBound.z, wideRadius );
	a.upperBound.x = fixWideAdd( a.upperBound.x, wideRadius );
	a.upperBound.y = fixWideAdd( a.upperBound.y, wideRadius );
	a.upperBound.z = fixWideAdd( a.upperBound.z, wideRadius );

	return a;
}

/// Does a fully contain b?
FIX_ALWAYS_INLINE bool fixAABBWide_Contains( fixAABBWide a, fixAABBWide b )
{
	if ( fixWideGt( a.lowerBound.x, b.lowerBound.x ) || fixWideGt( b.upperBound.x, a.upperBound.x ) ) return false;
	if ( fixWideGt( a.lowerBound.y, b.lowerBound.y ) || fixWideGt( b.upperBound.y, a.upperBound.y ) ) return false;
	if ( fixWideGt( a.lowerBound.z, b.lowerBound.z ) || fixWideGt( b.upperBound.z, a.upperBound.z ) ) return false;
	return true;
}

/// Surface area. The deltas narrow to local range first -- a box whose extent exceeds
/// Q48.16 range has no meaningful area in fixed_t, and saturating is the honest answer.
FIX_ALWAYS_INLINE fixed_t fixAABBWide_Area( fixAABBWide a )
{
	fixed_t dx = fixWideSubToFixed( a.upperBound.x, a.lowerBound.x );
	fixed_t dy = fixWideSubToFixed( a.upperBound.y, a.lowerBound.y );
	fixed_t dz = fixWideSubToFixed( a.upperBound.z, a.lowerBound.z );
	return fixMul( FIX( 2.0f ) , ( fixMul( dx , dy ) + fixMul( dy , dz ) + fixMul( dz , dx ) ) );
}

/// Center, in local space, rounded half up. Average at full width before
/// narrowing, splitting each endpoint first so even a full-range wide sum
/// cannot overflow. A center outside the local domain saturates to FIX_MIN/MAX.
FIX_ALWAYS_INLINE fixVec3 fixAABBWide_Center( fixAABBWide a )
{
	fixInt128 lx = fixInt128Shr( a.lowerBound.x, 1 ), ux = fixInt128Shr( a.upperBound.x, 1 );
	fixInt128 ly = fixInt128Shr( a.lowerBound.y, 1 ), uy = fixInt128Shr( a.upperBound.y, 1 );
	fixInt128 lz = fixInt128Shr( a.lowerBound.z, 1 ), uz = fixInt128Shr( a.upperBound.z, 1 );
	fixVec3 out = {
		fixAABBNarrowBound( fixInt128Add( fixInt128Add( lx, ux ), fixInt128FromU64( ( fixInt128Lo( a.lowerBound.x ) | fixInt128Lo( a.upperBound.x ) ) & 1 ) ) ),
		fixAABBNarrowBound( fixInt128Add( fixInt128Add( ly, uy ), fixInt128FromU64( ( fixInt128Lo( a.lowerBound.y ) | fixInt128Lo( a.upperBound.y ) ) & 1 ) ) ),
		fixAABBNarrowBound( fixInt128Add( fixInt128Add( lz, uz ), fixInt128FromU64( ( fixInt128Lo( a.lowerBound.z ) | fixInt128Lo( a.upperBound.z ) ) & 1 ) ) ),
	};
	return out;
}

// Internal: subtract as unsigned (ordered endpoints have a non-negative
// difference up to 128 bits), halve and round before narrowing. A representable
// half-width survives even when the full width exceeds the local domain.
FIX_ALWAYS_INLINE fixed_t fixWideHalfExtent( fixedWide_t lower, fixedWide_t upper )
{
	fixUInt128 difference = fixInt128ToUnsigned( fixWideSub( upper, lower ) );
	fixUInt128 half = fixUInt128Add( fixUInt128Shr( difference, 1 ), fixUInt128FromU64( fixUInt128Lo( difference ) & 1 ) );
	return fixUInt128Gt( half, fixUInt128FromU64( (uint64_t)FIX_MAX ) ) ? FIX_MAX : (fixed_t)fixUInt128Lo( half );
}

/// Half-widths rounded half up, saturated to FIX_MAX, independent of distance.
FIX_ALWAYS_INLINE fixVec3 fixAABBWide_Extents( fixAABBWide a )
{
	return FIX_LITERAL( fixVec3 ){ fixWideHalfExtent( a.lowerBound.x, a.upperBound.x ),
		fixWideHalfExtent( a.lowerBound.y, a.upperBound.y ), fixWideHalfExtent( a.lowerBound.z, a.upperBound.z ) };
}

/// Union of two wide boxes.
FIX_ALWAYS_INLINE fixAABBWide fixAABBWide_Union( fixAABBWide a, fixAABBWide b )
{
	fixAABBWide out;
	out.lowerBound.x = fixWideMin( a.lowerBound.x, b.lowerBound.x );
	out.lowerBound.y = fixWideMin( a.lowerBound.y, b.lowerBound.y );
	out.lowerBound.z = fixWideMin( a.lowerBound.z, b.lowerBound.z );
	out.upperBound.x = fixWideMax( a.upperBound.x, b.upperBound.x );
	out.upperBound.y = fixWideMax( a.upperBound.y, b.upperBound.y );
	out.upperBound.z = fixWideMax( a.upperBound.z, b.upperBound.z );
	return out;
}

/// Add uniform local padding to a wide box.
FIX_ALWAYS_INLINE fixAABBWide fixAABBWide_Inflate( fixAABBWide a, fixed_t extension )
{
	fixedWide_t wideExtension = fixWideFromFixed( extension );
	fixAABBWide out = a;
	out.lowerBound.x = fixWideSub( out.lowerBound.x, wideExtension );
	out.lowerBound.y = fixWideSub( out.lowerBound.y, wideExtension );
	out.lowerBound.z = fixWideSub( out.lowerBound.z, wideExtension );
	out.upperBound.x = fixWideAdd( out.upperBound.x, wideExtension );
	out.upperBound.y = fixWideAdd( out.upperBound.y, wideExtension );
	out.upperBound.z = fixWideAdd( out.upperBound.z, wideExtension );
	return out;
}

/// Do two wide boxes overlap?
FIX_ALWAYS_INLINE bool fixAABBWide_Overlaps( fixAABBWide a, fixAABBWide b )
{
	if ( fixWideLt( a.upperBound.x, b.lowerBound.x ) || fixWideGt( a.lowerBound.x, b.upperBound.x ) ) return false;
	if ( fixWideLt( a.upperBound.y, b.lowerBound.y ) || fixWideGt( a.lowerBound.y, b.upperBound.y ) ) return false;
	if ( fixWideLt( a.upperBound.z, b.lowerBound.z ) || fixWideGt( a.lowerBound.z, b.upperBound.z ) ) return false;
	return true;
}

/// Place a local box at a wide world origin. Exact: fixed-point addition, aligned
/// fraction points, so no outward rounding is needed -- the translated box is the
/// translated box.
FIX_ALWAYS_INLINE fixAABBWide fixOffsetAABBWide( fixAABB localBox, fixPosWide origin )
{
	fixAABBWide out;
	out.lowerBound = fixPosWideOffset( origin, localBox.lowerBound );
	out.upperBound = fixPosWideOffset( origin, localBox.upperBound );
	return out;
}

/// Closest point on a wide box to a LOCAL point.
///
/// Ported as box3d has it: the bounds narrow to local space and the clamp happens
/// there, because the incoming point is local. Clamping wide instead would be a
/// different function with a different signature, and inventing it here would mean
/// shipping an untested behaviour change wearing an extraction's clothes.
FIX_ALWAYS_INLINE fixVec3 fixClosestPointToAABBWide( fixVec3 point, fixAABBWide a )
{
	fixVec3 lo = fixPosWideToVec3( a.lowerBound );
	fixVec3 hi = fixPosWideToVec3( a.upperBound );
	return fixVecClamp( point, lo, hi );
}

/// Build a wide box from LOCAL points placed at a wide origin.
///
/// The companion to fixMakeAABBWide, and the shape box3d actually needs: mesh and hull
/// vertices are local (Q48.16) even when the body sits a light-year out, so the common
/// case is narrow points plus a wide base, not an array of wide points. Building the box
/// narrow first and widening once is also exact and cheaper than widening every point.
FIX_ALWAYS_INLINE fixAABBWide fixMakeAABBWideAt( const fixVec3* points, int count, fixed_t radius, fixPosWide origin )
{
	fixAABB local = fixMakeAABB( points, count, radius );
	return fixOffsetAABBWide( local, origin );
}

/// Transform a wide box by a local transform, with the same unrounded
/// coefficients and outward error allowance as fixAABB_Transform.
/// The inherited local-center limit remains: a center outside Q48.16 saturates
/// before rotation. Extents are computed from the full-width difference.
FIX_ALWAYS_INLINE fixAABBWide fixAABBWide_Transform( fixTransform transform, fixAABBWide a )
{
	fixAABB local = fixTransformAABBCenterExtents( transform, fixAABBWide_Center( a ), fixAABBWide_Extents( a ) );
	fixAABBWide out = { fixPosWideFromVec3( local.lowerBound ), fixPosWideFromVec3( local.upperBound ) };
	return out;
}

/// Is this a valid wide AABB? Both bounds valid, and lower <= upper on every axis.
FIX_ALWAYS_INLINE bool fixIsValidAABBWide( fixAABBWide a )
{
	if ( fixIsValidPosWide( a.lowerBound ) == false ) return false;
	if ( fixIsValidPosWide( a.upperBound ) == false ) return false;
	if ( fixWideGt( a.lowerBound.x, a.upperBound.x ) ) return false;
	if ( fixWideGt( a.lowerBound.y, a.upperBound.y ) ) return false;
	if ( fixWideGt( a.lowerBound.z, a.upperBound.z ) ) return false;
	return true;
}
