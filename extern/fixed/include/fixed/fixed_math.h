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

/// Cosine and sine to within one ulp of Q48.16 (|error| <= 2^-16 against the true cosine and
/// sine of the input angle) over the whole representable range. Range reduction by pi/2 in Q2.62
/// at 128 bits: the angle itself is exact there, and the one error is pi/2 rounded to 62 fraction
/// bits (0.38 of its last unit), which the quadrant count k multiplies. That shifts the reduced
/// angle by up to 7.5e-6 rad, 0.49 ulp of Q48.16, at the top of the range (|radians| near 2^47;
/// about 2^-17 ulp at 2^31 radians), on top of the 0.5 ulp of the final rounding. Then degree 11/12
/// Taylor polynomials in Q32.32. Deterministic. About
/// four times the cost of fixComputeCosSin, whose error is ~0.0017: use this one where the
/// result feeds geometry (tessellation, rotation matrices built from angles).
FIX_API fixCosSin fixComputeCosSinPrecise( fixed_t radians );

FIX_INLINE fixed_t fixSin( fixed_t radians ) { fixCosSin cs = fixComputeCosSin( radians ); return cs.sine; }
FIX_INLINE fixed_t fixCos( fixed_t radians ) { fixCosSin cs = fixComputeCosSin( radians ); return cs.cosine; }

/// Convert any angle into the range [-pi, pi].
FIX_INLINE fixed_t fixUnwindAngle( fixed_t radians )
{
	const fixed_t twoPi = FIX( 6.28318530718 );
	int64_t n = ( fixDiv( radians, twoPi ) + FIX_HALF ) >> FIX_FRACTION_BITS;
	return (fixed_t)fixInt128ToI64( fixInt128Sub( fixInt128FromI64( radians ), fixInt128MulI64( n, twoPi ) ) );
}

/// Interpolate a scalar.
/// Under FIX_SATURATE the subtract and the add saturate.
FIX_INLINE fixed_t fixLerp( fixed_t a, fixed_t b, fixed_t alpha )
{
#if defined( FIX_SATURATE )
	return fixAddSaturate( fixMul( fixSubSaturate( FIX( 1.0f ), alpha ), a ), fixMul( alpha, b ) );
#else
	return fixMul( ( FIX( 1.0f ) - alpha ) , a ) + fixMul( alpha , b );
#endif
}

// ---------------------------------------------------------------------------------------------
// The exp2 / log2 / pow ladder. Determinism over accuracy: every operation is pure integer
// arithmetic, so the results are bit-identical on every platform, which libm's pow is not.
// ---------------------------------------------------------------------------------------------

/// Base-2 logarithm. The mantissa's log2 bits are produced exactly by repeated squaring, so
/// this is deterministic. Returns INT64_MIN for a <= 0, log2 having no value there.
FIX_API fixed_t fixLog2( fixed_t a );

/// Base-2 exponential, by binary exponentiation over the fraction bits. Saturates to
/// INT64_MAX at and above 2^47 (the top of the Q48.16 whole-unit domain) and returns 0
/// below the smallest representable value.
FIX_API fixed_t fixExp2( fixed_t a );

/// base raised to exponent, as fixExp2( exponent * fixLog2( base ) ). Returns 0 for
/// base <= 0 and one for a zero exponent.
FIX_API fixed_t fixPow( fixed_t base, fixed_t exponent );

// ---------------------------------------------------------------------------------------------
// Q2.30 normalization
// ---------------------------------------------------------------------------------------------

/// One normalized Q2.30 component: raw * 2^30 / length, rounded to nearest and clamped to
/// [-1, 1] (the rounded divide can otherwise overshoot one by an ulp). length must be
/// non-zero; a zero length has no direction to normalize toward.
FIX_API int32_t fixNormalizeComponent30( int64_t raw, uint64_t length );

// ---------------------------------------------------------------------------------------------
// Critically damped smoothing on fixed point (deterministic control dynamics).
// Mirrors the double-precision formula: omega = 2 pi / smoothTime;
// velocity = ( velocity - omega^2 * dt * ( current - target ) ) / ( 1 + omega * dt )^2;
// result = current + velocity * dt.
// ---------------------------------------------------------------------------------------------

/// Critically damped smoothing toward a target. Updates velocity in place.
FIX_API fixed_t fixSmoothCriticallyDamped( fixed_t current, fixed_t target, fixed_t* velocity, fixed_t smoothTime,
										   fixed_t deltaTime );

/// Critically damped smoothing with separate smoothing times for moving up (toward a larger
/// magnitude) and down. The time is selected by comparing |target| to |current|.
FIX_API fixed_t fixSmoothCriticallyDampedUpDown( fixed_t current, fixed_t target, fixed_t* velocity,
												 fixed_t smoothTimeUp, fixed_t smoothTimeDown, fixed_t deltaTime );
