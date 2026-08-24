// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// WIDER STORAGE FOR INVERSE QUANTITIES.
//
// The solver never uses a body's mass or its inertia tensor. It uses their INVERSES, and
// an inverse is small exactly where the thing it inverts is large. Q48.16 holds no
// positive value below 1/65536, so a body past 65,536 units of mass or inertia has no
// representable inverse at all -- its inverse mass reads zero, which this engine spells
// STATIC, and its inverse inertia reads zero, which is a body that will not rotate. A
// uniform solid cube of density 1 crosses that line at about 13.1 units on a side.
//
// So inverse quantities carry MORE FRACTION BITS than everything else. They are still
// b3Fixed and still b3Matrix3 -- what changes is the scale they are interpreted at, and
// this header is where that scale is named and where every crossing between the two
// scales happens.
//
// ================================================================================
// THE ONE THING TO UNDERSTAND: MOST ARITHMETIC NEEDS NO CHANGE
// ================================================================================
//
// b3FixMul(a, b) is (a * b) >> 16. Feed it an inverse (scaled 2^40) and an ordinary value
// (scaled 2^16) and the result is scaled 2^40 -- an inverse again. That is exactly what
// the effective-mass machinery wants, so it keeps working verbatim:
//
//     kNormal = invMassA + invMassB + dot( rnA, invIA * rnA ) + dot( rnB, invIB * rnB )
//
// Every term there is a per-mass quantity, every operation is b3FixMul or a sum of them,
// and the whole expression stays in the inverse scale without a single edit. The same is
// true of b3AddMM on two inverse tensors, b3MulMV of an inverse tensor by an ordinary
// vector, b3Dot of an ordinary vector with one of those results, and the similarity
// transform R * invI * R^T -- because a rotation matrix is dimensionless.
//
// WHAT DOES NEED AN EDIT is every place an inverse turns back into an ordinary quantity,
// and there are only two shapes of those:
//
//   1. Applying an inverse to get a velocity:  dv = invMass * J,  dw = invI * torque.
//      The result is an ordinary Q48.16 value, so the shift is 40, not 16. Use b3InvMul
//      or b3InvMulMV. Using b3FixMul here is a silent factor of 2^24.
//
//   2. Inverting an inverse:  effectiveMass = 1 / kNormal. Use b3InvReciprocal.
//
// Getting one of those wrong is a factor of 16,777,216, which is loud in any test that
// looks at a velocity, and invisible in one that does not.
//
// ================================================================================
// WHY 40 AND NOT 48
// ================================================================================
//
// Widening the fraction buys range at the small end and SPENDS it at the large end, and
// the large end is real: a small body has a large inverse. Inverse mass and inverse
// inertia have to share one format, because the contact solver adds them together in the
// kNormal expression above.
//
// The four constraints, measured rather than assumed. The large-body target is space's
// shipping asteroid, a 250-unit cube of mass 15,625,000 and inertia 8.1e10. The
// small-body limits are what this engine can actually produce: b3FloorInertia floors the
// inertia diagonal at 4 ULP, so the largest inverse inertia is 1/(4/65536) = 16,384;
// mass has no floor, and a 0.01-radius sphere at density 1 gives an inverse mass of
// about 2.4e5.
//
//     inverse mass:    asteroid 6.4e-08  needs f >= 24 ;  2.4e5 must fit -> f <= 45
//     inverse inertia: asteroid 1.23e-11 needs f >= 37 ;  16384 must fit -> f <= 49
//
// The intersection is f in [37, 45]:
//
//     format    invI asteroid    invM asteroid    max value    headroom on a 1cm sphere
//     Q26.37              1.7             8796     6.71e+07                      280x
//     Q23.40             13.5            70369     8.39e+06                       35x
//     Q19.44            216.4          1125900     5.24e+05                      2.2x
//     Q15.48           3462.1         18014399     3.28e+04            0.1x -- OVERFLOWS
//
// Q16.48 IS EXCLUDED BY MEASUREMENT, and it was the assumption this work started from.
// Its ceiling is 32,768 and a one-centimetre sphere needs 240,000, so it would have
// traded a large-body bug for a small-body bug -- a worse trade, because small bodies are
// common and large ones are not. 40 is chosen for headroom at both ends rather than for
// resolution at one: 13.5 quanta of inverse inertia for the asteroid is coarse, but it is
// more graded response than the hand-tuned constant it replaces, and it keeps 35x of room
// before a small body overflows.
//
// THE FLOOR AND THE FORMAT ARE COUPLED. The f <= 49 bound is b3FloorInertia's 4-ULP
// constant and nothing else. Lowering that floor raises the largest inverse inertia and
// eats this headroom, so the two must be changed together. That sentence is the reason
// this comment exists.
#pragma once

#include "box3d/math_functions.h"
#include "box3d/types.h"

/// Fraction bits for stored inverse quantities. See the analysis above before changing
/// it: it is one constant precisely so the choice can be revisited cheaply, but it is
/// bounded on both sides and neither bound is arbitrary.
#define B3_INVERSE_FRACTION_BITS 40

/// How much wider an inverse is than an ordinary value. Every conversion below shifts by
/// either this or B3_INVERSE_FRACTION_BITS.
#define B3_INVERSE_EXTRA_BITS ( B3_INVERSE_FRACTION_BITS - B3_FIXED_FRACTION_BITS )

/// One quantum of an inverse quantity: the smallest value this format tells apart from
/// zero, and therefore the floor a finite dynamic mass clamps to.
#define B3_INVERSE_EPSILON ( (b3Fixed)1 )

/// The largest inverse quantity that fits. Named so the small-body bound is testable
/// rather than only described.
#define B3_INVERSE_MAX_VALUE ( B3_FIXED_MAX >> B3_INVERSE_FRACTION_BITS )

/// An ordinary Q48.16 value seen as an inverse quantity. Exact, and overflows above
/// B3_INVERSE_MAX_VALUE -- only for literals and configured values, never for a computed
/// inverse, which b3InvFromRatio produces already scaled.
B3_INLINE b3Fixed b3InvFromFixed( b3Fixed a )
{
	return a << B3_INVERSE_EXTRA_BITS;
}

/// An inverse quantity seen as an ordinary Q48.16 value, truncating toward zero. Lossy by
/// construction: this is the conversion that used to be the storage, and it is what the
/// public API returns to callers who ask for an inverse mass.
B3_INLINE b3Fixed b3FixedFromInv( b3Fixed a )
{
	return a >> B3_INVERSE_EXTRA_BITS;
}

/// 1/a, exactly, in the inverse scale. This is the whole point of the format.
///
/// The numerator is 2^(16+40) = 2^56, which fits in an int64 with room to spare, so one
/// hardware divide gives the exact truncated reciprocal and no 128-bit arithmetic is
/// involved. A zero denominator returns zero: zero mass is the degenerate body, not
/// infinity, and every caller here already treats it that way.
B3_INLINE b3Fixed b3InvFromRatio( b3Fixed a )
{
	if ( a == 0 )
	{
		return 0;
	}

	return ( (int64_t)1 << ( B3_FIXED_FRACTION_BITS + B3_INVERSE_FRACTION_BITS ) ) / a;
}

/// 1 / k, where k is an inverse quantity and the answer is an ordinary one: the effective
/// mass of a constraint. Same single divide, same zero convention.
B3_INLINE b3Fixed b3InvReciprocal( b3Fixed k )
{
	if ( k == 0 )
	{
		return B3_FIX( 0.0f );
	}

	return ( (int64_t)1 << ( B3_FIXED_FRACTION_BITS + B3_INVERSE_FRACTION_BITS ) ) / k;
}

/// An inverse quantity applied to an ordinary one, giving an ORDINARY one: dv = invMass*J.
///
/// Shift by the inverse's fraction bits, not by 16. Round half up, matching b3FixMul, and
/// at 128 bits because a large inverse against a large impulse leaves 64.
B3_INLINE b3Fixed b3InvMul( b3Fixed a, b3Fixed b )
{
	fixInt128 product = fixInt128MulI64( a, b );
	fixInt128 rounded = fixInt128Add( product, fixInt128FromI64( (int64_t)1 << ( B3_INVERSE_FRACTION_BITS - 1 ) ) );
	return (b3Fixed)fixInt128ToI64( fixInt128Shr( rounded, B3_INVERSE_FRACTION_BITS ) );
}

/// An inverse scalar applied to an ordinary vector, giving an ORDINARY vector:
/// dv = invMass * impulse.
B3_INLINE b3Vec3 b3InvMulSV( b3Fixed a, b3Vec3 v )
{
	b3Vec3 out = { b3InvMul( a, v.x ), b3InvMul( a, v.y ), b3InvMul( a, v.z ) };
	return out;
}

/// A whole inverse tensor seen as ordinary Q48.16, for the public getters. Lossy in
/// exactly the way the old storage was: a large body's inverse inertia reads zero here,
/// which is now a property of the reporting rather than of the simulation.
B3_INLINE b3Matrix3 b3FixedFromInvMatrix( b3Matrix3 m )
{
	b3Matrix3 out;
	out.cx = B3_LITERAL( b3Vec3 ){ b3FixedFromInv( m.cx.x ), b3FixedFromInv( m.cx.y ), b3FixedFromInv( m.cx.z ) };
	out.cy = B3_LITERAL( b3Vec3 ){ b3FixedFromInv( m.cy.x ), b3FixedFromInv( m.cy.y ), b3FixedFromInv( m.cy.z ) };
	out.cz = B3_LITERAL( b3Vec3 ){ b3FixedFromInv( m.cz.x ), b3FixedFromInv( m.cz.y ), b3FixedFromInv( m.cz.z ) };
	return out;
}

/// The inverse of an inertia tensor, computed DIRECTLY at the inverse scale.
///
/// The obvious implementation -- invert to Q48.16 and shift left by 24 -- is exactly the
/// thing this format exists to avoid: the Q48.16 inverse of a large tensor is already
/// zero, and shifting zero gains nothing. So the scale is applied inside the division,
/// where the precision still exists.
///
/// An inverse entry is cofactor/determinant with the cofactor at Q32.32 and the
/// determinant at Q**.48, so an answer scaled by 2^40 is ( cofactor << 56 ) / determinant.
/// A cofactor reaches 126 bits and shifting it up 56 leaves 128, so the whole calculation
/// runs at 256 bits -- unconditionally, rather than picking an arm by magnitude. This
/// runs when mass data is set and never in the solver, so the only thing that matters
/// about its cost is that it is finite.
///
/// A singular tensor gives the zero matrix, as everywhere else in this engine.
B3_INLINE b3Matrix3 b3InvertInertiaWide( b3Matrix3 m )
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
		return b3Mat3_zero;
	}

	bool negative = fixInt256IsNegative( det );
	fixUInt256 absDet = fixInt256Abs( det );

	const int shift = B3_FIXED_FRACTION_BITS + B3_INVERSE_FRACTION_BITS;

	b3Matrix3 out;
	out.cx = B3_LITERAL( b3Vec3 ){ fixDivShiftedWide( c00, shift, absDet, negative ),
								   fixDivShiftedWide( c10, shift, absDet, negative ),
								   fixDivShiftedWide( c20, shift, absDet, negative ) };
	out.cy = B3_LITERAL( b3Vec3 ){ fixDivShiftedWide( c01, shift, absDet, negative ),
								   fixDivShiftedWide( c11, shift, absDet, negative ),
								   fixDivShiftedWide( c21, shift, absDet, negative ) };
	out.cz = B3_LITERAL( b3Vec3 ){ fixDivShiftedWide( c02, shift, absDet, negative ),
								   fixDivShiftedWide( c12, shift, absDet, negative ),
								   fixDivShiftedWide( c22, shift, absDet, negative ) };
	return out;
}

/// An inverse tensor applied to an ordinary vector, giving an ORDINARY vector:
/// dw = invInertiaWorld * angularImpulse. The matrix-vector form of b3InvMul, with the
/// same per-product rounding b3MulMV uses.
B3_INLINE b3Vec3 b3InvMulMV( b3Matrix3 m, b3Vec3 v )
{
	b3Vec3 out = {
		b3InvMul( m.cx.x, v.x ) + b3InvMul( m.cy.x, v.y ) + b3InvMul( m.cz.x, v.z ),
		b3InvMul( m.cx.y, v.x ) + b3InvMul( m.cy.y, v.y ) + b3InvMul( m.cz.y, v.z ),
		b3InvMul( m.cx.z, v.x ) + b3InvMul( m.cy.z, v.y ) + b3InvMul( m.cz.z, v.z ),
	};
	return out;
}
