// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Deterministic fixed-point transcendentals. Cross-platform bit-identical:
// integer-only rational/minimax approximations, no libm.
#pragma once
#include "fixed/base.h"
#include "fixed/fixed.h"

/// Cosine and sine pair.
typedef struct b3CosSin
{
	b3Fixed cosine;
	b3Fixed sine;
} b3CosSin;

/// atan2 accurate to ~0.0023 degrees. Deterministic (unlike the standard library).
B3_API b3Fixed b3Atan2( b3Fixed y, b3Fixed x );

/// Cosine and sine of an angle in radians. Deterministic.
B3_API b3CosSin b3ComputeCosSin( b3Fixed radians );

B3_INLINE b3Fixed b3Sin( b3Fixed radians ) { b3CosSin cs = b3ComputeCosSin( radians ); return cs.sine; }
B3_INLINE b3Fixed b3Cos( b3Fixed radians ) { b3CosSin cs = b3ComputeCosSin( radians ); return cs.cosine; }

/// Convert any angle into the range [-pi, pi].
B3_INLINE b3Fixed b3UnwindAngle( b3Fixed radians )
{
	const b3Fixed twoPi = B3_FIX( 6.28318530718 );
	int64_t n = ( b3FixDiv( radians, twoPi ) + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS;
	return (b3Fixed)( radians - (b3Int128)n * twoPi );
}
