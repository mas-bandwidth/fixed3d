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

/// Exact unsigned division with remainder. Shift-subtract, entered at the highest bit
/// that can contribute, so the loop length tracks the operands rather than the type.
/// A zero divisor yields a zero quotient and remainder: every caller in this library
/// tests the divisor first, and returning rather than trapping keeps a caller's mistake
/// from killing a process.
FIX_ALWAYS_INLINE void fixUInt256DivMod( fixUInt256 dividend, fixUInt256 divisor, fixUInt256* quotientOut, fixUInt256* remainderOut )
{
	fixUInt256 quotient;
	quotient.hi = FIX_UINT128_ZERO;
	quotient.lo = FIX_UINT128_ZERO;
	fixUInt256 remainder = quotient;

	if ( fixUInt256IsZero( divisor ) )
	{
		*quotientOut = quotient;
		*remainderOut = remainder;
		return;
	}

	for ( int i = fixUInt256BitLength( dividend ) - 1; i >= 0; --i )
	{
		remainder = fixUInt256Shl( remainder, 1 );
		if ( fixUInt256Bit( dividend, i ) )
		{
			remainder = fixUInt256Add( remainder, fixUInt256FromU64( 1 ) );
		}
		if ( fixUInt256Ge( remainder, divisor ) )
		{
			remainder = fixUInt256Sub( remainder, divisor );
			quotient = fixUInt256Add( quotient, fixUInt256Shl( fixUInt256FromU64( 1 ), i ) );
		}
	}

	*quotientOut = quotient;
	*remainderOut = remainder;
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
