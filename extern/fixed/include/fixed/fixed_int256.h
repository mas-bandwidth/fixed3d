// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// 256-BIT INTERMEDIATES FOR THE EXACT 3x3 MATRIX INVERSE.
//
// The inverse of a Q48.16 matrix is a ratio of two quantities that do not both fit in
// 128 bits. The cofactors are Q32.32 products of two Q48.16 entries, so a cofactor
// reaches 126 bits; the determinant is a cofactor multiplied by another entry, so it
// reaches 190; and the numerator of an inverse entry is a cofactor shifted up 32 places,
// so it reaches 158. Any reduction that squeezes those into 128 bits either truncates
// the cofactor or overflows the determinant, and both failures are silent -- they return
// a number of arbitrary sign rather than refusing.
//
// So the wide arm of fixInvertMatrix carries the full width instead, and this header is
// the arithmetic it needs: unsigned 256-bit add, subtract, shift, compare, a 128x64
// widening multiply, and a division. Nothing more. It is NOT a general big-integer
// facility and should not grow into one -- every operation here exists because one
// specific line of the inverse needs it.
//
// BUILT ON THE 128-BIT SEAM, NOT ON __int128. Every operation below is expressed in the
// fixUInt128 vocabulary from fixed_int128.h, so the 256-bit layer is native where the
// compiler has __int128 and emulated where it does not, with no second implementation to
// keep in step. The emulated arm of the test suite therefore covers this header for free.
//
// WHERE THE COST LANDS: the wide arm runs at mass-data time (body creation, SetMassData),
// never in the solver. The division is an unrolled shift-subtract, which is slow by the
// standards of a hardware divide and irrelevant by the standards of the thing that calls
// it. The solver's arm of fixInvertMatrix is 128-bit and unchanged.
#pragma once

#include "fixed/fixed_int128.h"

/// Unsigned 256-bit integer as an ordered pair of 128-bit halves.
typedef struct fixUInt256
{
	fixUInt128 hi;
	fixUInt128 lo;
} fixUInt256;

/// Widen a 128-bit value. Exact.
FIX_ALWAYS_INLINE fixUInt256 fixUInt256FromU128( fixUInt128 a )
{
	fixUInt256 r;
	r.hi = FIX_UINT128_ZERO;
	r.lo = a;
	return r;
}

FIX_ALWAYS_INLINE fixUInt256 fixUInt256FromU64( uint64_t a )
{
	return fixUInt256FromU128( fixUInt128FromU64( a ) );
}

FIX_ALWAYS_INLINE bool fixUInt256IsZero( fixUInt256 a )
{
	return fixUInt128Eq( a.hi, FIX_UINT128_ZERO ) && fixUInt128Eq( a.lo, FIX_UINT128_ZERO );
}

/// Wrapping 256-bit addition. The carry out of the low half is detected by the
/// standard unsigned test -- a sum that wrapped is smaller than either addend.
FIX_ALWAYS_INLINE fixUInt256 fixUInt256Add( fixUInt256 a, fixUInt256 b )
{
	fixUInt256 r;
	r.lo = fixUInt128Add( a.lo, b.lo );
	fixUInt128 carry = fixUInt128Lt( r.lo, a.lo ) ? fixUInt128FromU64( 1 ) : FIX_UINT128_ZERO;
	r.hi = fixUInt128Add( fixUInt128Add( a.hi, b.hi ), carry );
	return r;
}

/// Wrapping 256-bit subtraction.
FIX_ALWAYS_INLINE fixUInt256 fixUInt256Sub( fixUInt256 a, fixUInt256 b )
{
	fixUInt256 r;
	r.lo = fixUInt128Sub( a.lo, b.lo );
	fixUInt128 borrow = fixUInt128Lt( a.lo, b.lo ) ? fixUInt128FromU64( 1 ) : FIX_UINT128_ZERO;
	r.hi = fixUInt128Sub( fixUInt128Sub( a.hi, b.hi ), borrow );
	return r;
}

/// Two's complement negation.
FIX_ALWAYS_INLINE fixUInt256 fixUInt256Neg( fixUInt256 a )
{
	fixUInt256 zero;
	zero.hi = FIX_UINT128_ZERO;
	zero.lo = FIX_UINT128_ZERO;
	return fixUInt256Sub( zero, a );
}

FIX_ALWAYS_INLINE bool fixUInt256Eq( fixUInt256 a, fixUInt256 b )
{
	return fixUInt128Eq( a.hi, b.hi ) && fixUInt128Eq( a.lo, b.lo );
}

FIX_ALWAYS_INLINE bool fixUInt256Lt( fixUInt256 a, fixUInt256 b )
{
	if ( !fixUInt128Eq( a.hi, b.hi ) )
	{
		return fixUInt128Lt( a.hi, b.hi );
	}
	return fixUInt128Lt( a.lo, b.lo );
}

FIX_ALWAYS_INLINE bool fixUInt256Ge( fixUInt256 a, fixUInt256 b )
{
	return !fixUInt256Lt( a, b );
}

/// Left shift by 0..255. Shifts of 128 or more move the low half into the high half
/// wholesale; the cross-half term is guarded because a 128-place shift of a 128-bit
/// value has no defined answer on the native arm.
FIX_ALWAYS_INLINE fixUInt256 fixUInt256Shl( fixUInt256 a, int shift )
{
	fixUInt256 r;
	if ( shift <= 0 )
	{
		return a;
	}
	if ( shift >= 256 )
	{
		r.hi = FIX_UINT128_ZERO;
		r.lo = FIX_UINT128_ZERO;
		return r;
	}
	if ( shift >= 128 )
	{
		r.hi = fixUInt128Shl( a.lo, shift - 128 );
		r.lo = FIX_UINT128_ZERO;
		return r;
	}
	fixUInt128 spill = fixUInt128Shr( a.lo, 128 - shift );
	r.hi = fixUInt128Or( fixUInt128Shl( a.hi, shift ), spill );
	r.lo = fixUInt128Shl( a.lo, shift );
	return r;
}

/// Right shift by 0..255.
FIX_ALWAYS_INLINE fixUInt256 fixUInt256Shr( fixUInt256 a, int shift )
{
	fixUInt256 r;
	if ( shift <= 0 )
	{
		return a;
	}
	if ( shift >= 256 )
	{
		r.hi = FIX_UINT128_ZERO;
		r.lo = FIX_UINT128_ZERO;
		return r;
	}
	if ( shift >= 128 )
	{
		r.lo = fixUInt128Shr( a.hi, shift - 128 );
		r.hi = FIX_UINT128_ZERO;
		return r;
	}
	fixUInt128 spill = fixUInt128Shl( a.hi, 128 - shift );
	r.lo = fixUInt128Or( fixUInt128Shr( a.lo, shift ), spill );
	r.hi = fixUInt128Shr( a.hi, shift );
	return r;
}

/// Index of the highest set bit plus one; zero for zero. Used to start the division at
/// the first bit that can matter instead of at bit 255.
FIX_ALWAYS_INLINE int fixUInt256BitLength( fixUInt256 a )
{
	for ( int i = 255; i >= 0; --i )
	{
		fixUInt128 half = i >= 128 ? a.hi : a.lo;
		int bit = i >= 128 ? i - 128 : i;
		if ( !fixUInt128Eq( fixUInt128And( fixUInt128Shr( half, bit ), fixUInt128FromU64( 1 ) ), FIX_UINT128_ZERO ) )
		{
			return i + 1;
		}
	}
	return 0;
}

FIX_ALWAYS_INLINE bool fixUInt256Bit( fixUInt256 a, int i )
{
	fixUInt128 half = i >= 128 ? a.hi : a.lo;
	int bit = i >= 128 ? i - 128 : i;
	return !fixUInt128Eq( fixUInt128And( fixUInt128Shr( half, bit ), fixUInt128FromU64( 1 ) ), FIX_UINT128_ZERO );
}

/// Widening multiply of a 128-bit value by a 64-bit value. Exact: the product of a
/// 128-bit and a 64-bit value needs at most 192 bits.
FIX_ALWAYS_INLINE fixUInt256 fixUInt256MulU128ByU64( fixUInt128 a, uint64_t b )
{
	fixUInt128 lowPart = fixUInt128MulU64( fixUInt128Lo( a ), b );
	fixUInt128 highPart = fixUInt128MulU64( fixUInt128Hi( a ), b );

	// highPart occupies bits 64..191, so it splits across the halves.
	fixUInt256 shifted;
	shifted.hi = fixUInt128Shr( highPart, 64 );
	shifted.lo = fixUInt128Shl( highPart, 64 );

	return fixUInt256Add( shifted, fixUInt256FromU128( lowPart ) );
}

/// Number of leading zero bits, written out rather than taken from a builtin so the
/// plain-MSVC arm needs no intrinsic and every arm folds the same way.
FIX_ALWAYS_INLINE int fixCountLeadingZeros64( uint64_t x )
{
	if ( x == 0 )
	{
		return 64;
	}

	int n = 0;
	if ( ( x >> 32 ) == 0 ) { n += 32; x <<= 32; }
	if ( ( x >> 48 ) == 0 ) { n += 16; x <<= 16; }
	if ( ( x >> 56 ) == 0 ) { n += 8; x <<= 8; }
	if ( ( x >> 60 ) == 0 ) { n += 4; x <<= 4; }
	if ( ( x >> 62 ) == 0 ) { n += 2; x <<= 2; }
	if ( ( x >> 63 ) == 0 ) { n += 1; }
	return n;
}

FIX_ALWAYS_INLINE void fixUInt256ToLimbs( fixUInt256 a, uint64_t limbs[4] )
{
	limbs[0] = fixUInt128Lo( a.lo );
	limbs[1] = fixUInt128Hi( a.lo );
	limbs[2] = fixUInt128Lo( a.hi );
	limbs[3] = fixUInt128Hi( a.hi );
}

FIX_ALWAYS_INLINE fixUInt256 fixUInt256FromLimbs( const uint64_t limbs[4] )
{
	fixUInt256 r;
	r.lo = fixUInt128Make( limbs[1], limbs[0] );
	r.hi = fixUInt128Make( limbs[3], limbs[2] );
	return r;
}

/// Internal: shift a limb left by s, bringing in the top s bits of the limb below.
/// The s == 0 case is spelled out because `x >> 64` is undefined, and that is the single
/// most common way to get a normalizing shift wrong.
FIX_ALWAYS_INLINE uint64_t fixShiftInLimb( uint64_t high, uint64_t low, int s )
{
	if ( s == 0 )
	{
		return high;
	}

	return ( high << s ) | ( low >> ( 64 - s ) );
}

/// Exact unsigned division with remainder: Knuth Algorithm D over 64-bit limbs.
///
/// This was a shift-subtract loop -- one iteration per bit of the dividend, around two
/// hundred of them, each doing 256-bit shifts, compares and subtracts. Exact, and slow in
/// a place that matters: a consumer storing inverse quantities at a wider scale puts a 3x3
/// solve of large values in a per-substep path, and every one of those divisions landed
/// here. Algorithm D replaces the bit loop with at most three 128-by-64 divides.
///
/// A zero divisor yields a zero quotient and remainder rather than trapping: every caller
/// in this library tests the divisor first, and returning keeps a caller's mistake from
/// killing a process. That contract is unchanged.
///
/// The three parts that are easy to get wrong, and how each is handled here:
///
///   NORMALIZATION shifts the divisor so its top limb has its top bit set, which is what
///   bounds the quotient estimate's error at two. The shift of zero is spelled out
///   separately in fixShiftInLimb, because `x >> 64` is undefined behavior and a
///   normalized divisor is exactly the input that asks for it.
///
///   THE ESTIMATE can exceed the true quotient limb by at most two once normalized, and
///   the loop below walks it down using the next limb of each operand. The rare case
///   where the top limbs are equal is handled by starting at the largest possible limb
///   rather than by dividing, since that division would overflow.
///
///   THE ADD-BACK fires when the estimate was still one too large after correction, which
///   happens for roughly one input in 2^63. It is the step most likely to be missing from
///   an implementation that passes a casual test, so test/int256_division_test.c carries
///   constructed vectors for it rather than hoping a random sweep lands on one.
FIX_ALWAYS_INLINE void fixUInt256DivMod( fixUInt256 dividend, fixUInt256 divisor, fixUInt256* quotientOut, fixUInt256* remainderOut )
{
	uint64_t u[4], v[4];
	fixUInt256ToLimbs( dividend, u );
	fixUInt256ToLimbs( divisor, v );

	uint64_t q[4] = { 0, 0, 0, 0 };

	int n = 4;
	while ( n > 0 && v[n - 1] == 0 ) { n--; }

	if ( n == 0 )
	{
		fixUInt256 zero;
		zero.hi = FIX_UINT128_ZERO;
		zero.lo = FIX_UINT128_ZERO;
		*quotientOut = zero;
		*remainderOut = zero;
		return;
	}

	int m = 4;
	while ( m > 0 && u[m - 1] == 0 ) { m--; }

	if ( m < n )
	{
		// The quotient is zero and the dividend is the remainder. Also covers a zero
		// dividend, where m is 0.
		fixUInt256 zero;
		zero.hi = FIX_UINT128_ZERO;
		zero.lo = FIX_UINT128_ZERO;
		*quotientOut = zero;
		*remainderOut = dividend;
		return;
	}

	if ( n == 1 )
	{
		// Short division: one 128-by-64 divide per limb, the remainder feeding the next.
		// The precondition of fixUInt128DivRemBy64 holds inductively because each
		// remainder is already below the divisor.
		uint64_t rest = 0;
		for ( int i = m - 1; i >= 0; i-- )
		{
			q[i] = fixUInt128DivRemBy64( fixUInt128Make( rest, u[i] ), v[0], &rest );
		}

		*quotientOut = fixUInt256FromLimbs( q );
		*remainderOut = fixUInt256FromU64( rest );
		return;
	}

	// Normalize. un needs one limb more than the dividend for the bits shifted out of it.
	int s = fixCountLeadingZeros64( v[n - 1] );

	uint64_t vn[4];
	for ( int i = n - 1; i > 0; i-- ) { vn[i] = fixShiftInLimb( v[i], v[i - 1], s ); }
	vn[0] = v[0] << s;

	uint64_t un[5];
	un[m] = s == 0 ? 0 : ( u[m - 1] >> ( 64 - s ) );
	for ( int i = m - 1; i > 0; i-- ) { un[i] = fixShiftInLimb( u[i], u[i - 1], s ); }
	un[0] = u[0] << s;

	for ( int j = m - n; j >= 0; j-- )
	{
		uint64_t qhat;
		uint64_t rhat;
		bool rhatFits;

		if ( un[j + n] >= vn[n - 1] )
		{
			// The estimate would be the base itself, which does not fit a limb; start at
			// the largest limb there is and let the correction below walk it down.
			qhat = UINT64_MAX;

			fixUInt128 numerator = fixUInt128Make( un[j + n], un[j + n - 1] );
			fixUInt128 product = fixUInt128MulU64( qhat, vn[n - 1] );
			fixUInt128 rest = fixUInt128Sub( numerator, product );
			rhatFits = fixUInt128Hi( rest ) == 0;
			rhat = fixUInt128Lo( rest );
		}
		else
		{
			qhat = fixUInt128DivRemBy64( fixUInt128Make( un[j + n], un[j + n - 1] ), vn[n - 1], &rhat );
			rhatFits = true;
		}

		// Walk the estimate down while it is provably too large. Once rhat leaves 64 bits
		// the test cannot fire again, so the loop stops there.
		while ( rhatFits && qhat != 0 &&
				fixUInt128Gt( fixUInt128MulU64( qhat, vn[n - 2] ), fixUInt128Make( rhat, un[j + n - 2] ) ) )
		{
			qhat--;
			fixUInt128 widened = fixUInt128Add( fixUInt128FromU64( rhat ), fixUInt128FromU64( vn[n - 1] ) );
			rhatFits = fixUInt128Hi( widened ) == 0;
			rhat = fixUInt128Lo( widened );
		}

		// Multiply and subtract, carrying the borrow at 128 bits so the wrap cases need no
		// reasoning about which of two subtractions overflowed.
		uint64_t borrow = 0;
		uint64_t carry = 0;
		for ( int i = 0; i < n; i++ )
		{
			fixUInt128 product = fixUInt128Add( fixUInt128MulU64( qhat, vn[i] ), fixUInt128FromU64( carry ) );
			carry = fixUInt128Hi( product );

			fixUInt128 left = fixUInt128FromU64( un[i + j] );
			fixUInt128 right = fixUInt128Add( fixUInt128FromU64( fixUInt128Lo( product ) ), fixUInt128FromU64( borrow ) );
			if ( fixUInt128Ge( left, right ) )
			{
				un[i + j] = fixUInt128Lo( fixUInt128Sub( left, right ) );
				borrow = 0;
			}
			else
			{
				un[i + j] = fixUInt128Lo( fixUInt128Sub( fixUInt128Add( left, fixUInt128Shl( fixUInt128FromU64( 1 ), 64 ) ), right ) );
				borrow = 1;
			}
		}

		bool negative;
		{
			fixUInt128 left = fixUInt128FromU64( un[j + n] );
			fixUInt128 right = fixUInt128Add( fixUInt128FromU64( carry ), fixUInt128FromU64( borrow ) );
			if ( fixUInt128Ge( left, right ) )
			{
				un[j + n] = fixUInt128Lo( fixUInt128Sub( left, right ) );
				negative = false;
			}
			else
			{
				un[j + n] = fixUInt128Lo( fixUInt128Sub( fixUInt128Add( left, fixUInt128Shl( fixUInt128FromU64( 1 ), 64 ) ), right ) );
				negative = true;
			}
		}

		q[j] = qhat;

#ifdef FIX_INT256_NO_ADDBACK
		// The negative control removes the add-back, which is the rarest branch here and
		// the one an implementation is most likely to be missing. The control build MUST
		// fail: if it passes, the suite has stopped reaching this path and has gone blind
		// on it -- which is exactly what happened to an earlier version of the test, whose
		// hand-written "add-back vectors" reached this branch zero times.
		negative = false;
#endif

		if ( negative )
		{
			// The estimate was one too large after all: give the limb back and add the
			// divisor in again. The carry out of that addition cancels the borrow above.
			q[j] = qhat - 1;

			uint64_t addCarry = 0;
			for ( int i = 0; i < n; i++ )
			{
				fixUInt128 sum = fixUInt128Add( fixUInt128Add( fixUInt128FromU64( un[i + j] ), fixUInt128FromU64( vn[i] ) ),
												fixUInt128FromU64( addCarry ) );
				un[i + j] = fixUInt128Lo( sum );
				addCarry = fixUInt128Hi( sum );
			}
			un[j + n] += addCarry;
		}
	}

	// Denormalize the remainder. Limbs at or above n are zero by construction.
	uint64_t r[4] = { 0, 0, 0, 0 };
	for ( int i = 0; i < n - 1; i++ ) { r[i] = ( un[i] >> s ) | ( s == 0 ? 0 : ( un[i + 1] << ( 64 - s ) ) ); }
	r[n - 1] = un[n - 1] >> s;

	*quotientOut = fixUInt256FromLimbs( q );
	*remainderOut = fixUInt256FromLimbs( r );
}

/// True if a two's complement 256-bit value is negative.
FIX_ALWAYS_INLINE bool fixInt256IsNegative( fixUInt256 a )
{
	return !fixUInt128Eq( fixUInt128And( fixUInt128Shr( a.hi, 127 ), fixUInt128FromU64( 1 ) ), FIX_UINT128_ZERO );
}

/// Magnitude of a two's complement 256-bit value.
FIX_ALWAYS_INLINE fixUInt256 fixInt256Abs( fixUInt256 a )
{
	return fixInt256IsNegative( a ) ? fixUInt256Neg( a ) : a;
}

/// Widening signed multiply of a 128-bit value by a 64-bit value, result in two's
/// complement 256-bit form.
FIX_ALWAYS_INLINE fixUInt256 fixInt256MulI128ByI64( fixInt128 a, int64_t b )
{
	bool negative = fixInt128IsNegative( a ) != ( b < 0 );

	fixUInt128 absA = fixInt128IsNegative( a ) ? fixUInt128Neg( fixInt128ToUnsigned( a ) ) : fixInt128ToUnsigned( a );
	// Negating through unsigned so the most negative value has a defined magnitude.
	uint64_t absB = b < 0 ? (uint64_t)0 - (uint64_t)b : (uint64_t)b;

	fixUInt256 product = fixUInt256MulU128ByU64( absA, absB );
	return negative ? fixUInt256Neg( product ) : product;
}
