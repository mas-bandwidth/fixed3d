// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
// Wide (Q112.16) fixed-point primitives: 128-bit world coordinates that share
// b3Fixed's 16 fraction bits. Sharing the fraction count is the crux — the
// boundary between wide world space and Q48.16 local space is then an exact
// integer subtract plus a range check, never an arithmetic rescale. See
// fixed3d's docs/design/wide-world-positions.md for the architecture these
// primitives serve.
#pragma once

#include "fixed/fixed.h"
#include "fixed/fixed_vec.h"

#if !B3_HAS_INT128
#error "fixed_wide.h requires 128-bit integer support (clang, gcc, or clang-cl)"
#endif

/// Wide fixed-point scalar: Q112.16 in a 128-bit integer. Same resolution as
/// b3Fixed (1/65536); all 64 extra bits go to integer range (~±2.6e33 units).
typedef b3Int128 b3FixedWide;

/// Wide world position: three Q112.16 coordinates.
typedef struct b3PosWide
{
	b3FixedWide x, y, z;
} b3PosWide;

/// Widen a local Q48.16 value to Q112.16. Exact: the fraction points align.
B3_FIXED_INLINE b3FixedWide b3WideFromFixed( b3Fixed a )
{
	return (b3FixedWide)a;
}

/// Narrow a Q112.16 value to local Q48.16, saturating out-of-range values to
/// INT64_MAX/INT64_MIN. Exact whenever the value fits local range.
B3_FIXED_INLINE b3Fixed b3WideToFixed( b3FixedWide a )
{
	if ( a > (b3FixedWide)INT64_MAX )
	{
		return INT64_MAX;
	}
	if ( a < (b3FixedWide)INT64_MIN )
	{
		return INT64_MIN;
	}
	return (b3Fixed)a;
}

/// Wide add. Exact 128-bit integer addition.
B3_FIXED_INLINE b3FixedWide b3WideAdd( b3FixedWide a, b3FixedWide b )
{
	return a + b;
}

/// Wide subtract. Exact 128-bit integer subtraction.
B3_FIXED_INLINE b3FixedWide b3WideSub( b3FixedWide a, b3FixedWide b )
{
	return a - b;
}

/// Offset a wide coordinate by a local delta (the once-per-step delta-fold).
/// Exact: int128 += widened int64, fraction points aligned.
B3_FIXED_INLINE b3FixedWide b3WideOffset( b3FixedWide a, b3Fixed d )
{
	return a + (b3FixedWide)d;
}

/// The boundary operation: difference two wide world coordinates into local
/// Q48.16 space. Exact whenever the separation fits local range (any contact
/// pair, joint, or reach-bounded query); saturates otherwise. This is the
/// fixed-point replacement for float's entire directed-rounding apparatus.
B3_FIXED_INLINE b3Fixed b3WideSubToFixed( b3FixedWide a, b3FixedWide b )
{
	return b3WideToFixed( a - b );
}

/// Widen a local position/vector to a wide world position. Exact.
B3_FIXED_INLINE b3PosWide b3PosWideFromVec3( b3Vec3 v )
{
	b3PosWide p = { (b3FixedWide)v.x, (b3FixedWide)v.y, (b3FixedWide)v.z };
	return p;
}

/// Narrow a wide position to a local vector, saturating per component.
B3_FIXED_INLINE b3Vec3 b3PosWideToVec3( b3PosWide p )
{
	b3Vec3 v = { b3WideToFixed( p.x ), b3WideToFixed( p.y ), b3WideToFixed( p.z ) };
	return v;
}

/// Difference two wide positions into local space (per-component boundary op).
B3_FIXED_INLINE b3Vec3 b3PosWideSub( b3PosWide a, b3PosWide b )
{
	b3Vec3 v = { b3WideSubToFixed( a.x, b.x ), b3WideSubToFixed( a.y, b.y ), b3WideSubToFixed( a.z, b.z ) };
	return v;
}

/// Offset a wide position by a local delta vector. Exact.
B3_FIXED_INLINE b3PosWide b3PosWideOffset( b3PosWide p, b3Vec3 d )
{
	b3PosWide out = { b3WideOffset( p.x, d.x ), b3WideOffset( p.y, d.y ), b3WideOffset( p.z, d.z ) };
	return out;
}
