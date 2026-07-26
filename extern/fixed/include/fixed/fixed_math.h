// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Deterministic fixed-point transcendentals. Cross-platform bit-identical:
// integer-only rational/minimax approximations, no libm.
#pragma once
#include "fixed/base.h"
#include "fixed/fixed.h"

/// Cosine and sine pair.
typedef struct fixCosSin
{
	fixed_t cosine;
	fixed_t sine;
} fixCosSin;

/// atan2 accurate to ~0.0023 degrees. Deterministic (unlike the standard library).
FIX_API fixed_t fixAtan2( fixed_t y, fixed_t x );

/// Cosine and sine of an angle in radians. Deterministic.
FIX_API fixCosSin fixComputeCosSin( fixed_t radians );

FIX_INLINE fixed_t fixSin( fixed_t radians ) { fixCosSin cs = fixComputeCosSin( radians ); return cs.sine; }
FIX_INLINE fixed_t fixCos( fixed_t radians ) { fixCosSin cs = fixComputeCosSin( radians ); return cs.cosine; }

/// Convert any angle into the range [-pi, pi].
FIX_INLINE fixed_t fixUnwindAngle( fixed_t radians )
{
	const fixed_t twoPi = FIX( 6.28318530718 );
	int64_t n = ( fixDiv( radians, twoPi ) + FIX_HALF ) >> FIX_FRACTION_BITS;
	return (fixed_t)( radians - (fixInt128)n * twoPi );
}
