// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// 128-BIT INTEGERS ON EVERY COMPILER, INCLUDING PLAIN MSVC.
//
// The fixed-point core needs 128-bit intermediates: fixMul widens before it rounds,
// fixDiv shifts a 64-bit numerator up by 16 before dividing, fixLength accumulates an
// exact sum of squares, and the whole wide-world family is 128-bit by definition. GCC,
// clang and clang-cl provide __int128. Plain MSVC does not, and Visual C++ support is a
// hard requirement, so this header supplies the missing arithmetic instead of excluding
// the compiler.
//
// TWO LAYERS, AND THE SEPARATION IS THE POINT:
//
//   1. fixEmuUInt128 / fixEmuInt128 -- an emulated pair of two uint64_t lanes with a
//      complete operation vocabulary. COMPILED ON EVERY PLATFORM, unconditionally, even
//      where native __int128 exists. That is what lets test/int128_test.c check every
//      emulated operation against the native one, input by input, on the machines that
//      have both. An emulation that only compiles where it cannot be checked is an
//      emulation nobody has checked.
//
//   2. fixInt128 / fixUInt128 plus the fixInt128*/fixUInt128* vocabulary -- the types the
//      rest of the library speaks. They are native __int128 where the compiler has it,
//      and the emulated pair where it does not. The vocabulary functions are
//      FIX_ALWAYS_INLINE one-liners over the native operators, so native builds get
//      byte-identical code generation to the operators they replaced -- verified by
//      diffing the compiled library, not assumed.
//
// The emulated semantics are captured from the serialize library's serialize_uint128_t /
// serialize_int128_t pair (a sibling project under the same estate), renamed into this
// library's namespace per Glenn's ruling: fixed does not depend on serialize. serialize's
// pair is C++-operator-shaped and this one is C-function-shaped, because fixed is a C
// library; the arithmetic and every documented edge choice are the same.
//
// SEMANTICS MATCH NATIVE __int128 EXACTLY, with documented choices where native has none:
//
//   - Shift counts outside [0, 127] yield zero for << and for the unsigned >>, and all
//     sign bits for the signed arithmetic >> (0 for non-negative, -1 for negative), which
//     is the limit of shifting further. Native shifts by 128 or more are undefined
//     behavior, so there is no native answer to match.
//   - Signed INT128_MIN / -1 wraps to INT128_MIN with a zero remainder, the bit pattern
//     two's complement hardware produces.
//   - DIVISION BY ZERO IS UNDEFINED, exactly as it is for native __int128. arm64 and
//     x86-64 do not even agree with each other (arm64 returns zero, x86-64 raises a
//     hardware exception), so there is no portable behavior available to match. What the
//     emulation guarantees is only that it is TOTAL: it returns a zero quotient rather
//     than trapping, so a caller's mistake cannot kill a process. That is an
//     implementation detail and not a contract, and the differential test deliberately
//     excludes zero divisors because agreement there is not a property either side can
//     promise. Every divide in this library guards its divisor first.
//
// FORCING THE EMULATED PATH: define FIX_FORCE_EMULATED_INT128 and the whole library runs
// on the emulated pair even where native __int128 exists. CI builds the entire test suite
// that way on all three operating systems and asserts THE SAME FROZEN DETERMINISM HASHES,
// so the emulated arithmetic is held to bit-identity with native by every test in the
// repository rather than by one dedicated test.
#pragma once

// Self-contained, and deliberately does NOT include base.h: the FIX_ALWAYS_INLINE
// definition below keys off whether the consumer has already pulled in base.h's macro
// block, so including base.h from here would answer its own question.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The always-inline spelling this library's header-inline math uses. A consumer that has
// supplied base.h's macros gets FIX_FORCE_INLINE, which it may have overridden; a
// consumer including this header on its own gets the compiler's spelling directly.
#ifndef FIX_INLINE
	#if defined( _MSC_VER )
		#define FIX_ALWAYS_INLINE static __forceinline
	#elif defined( __GNUC__ ) || defined( __clang__ )
		#define FIX_ALWAYS_INLINE static inline __attribute__( ( always_inline ) )
	#else
		#define FIX_ALWAYS_INLINE static inline
	#endif
#else
	#define FIX_ALWAYS_INLINE FIX_FORCE_INLINE
#endif

// ================================================================================
// LAYER 1: the emulated pair. Always compiled, on every platform.
// ================================================================================

/// Emulated unsigned 128-bit integer: two 64-bit lanes, low half first to match the
/// little-endian layout and the wire order of the sibling projects that carry this shape.
typedef struct fixEmuUInt128
{
	uint64_t lo;
	uint64_t hi;
} fixEmuUInt128;

/// Emulated signed 128-bit integer. The same two lanes; the top bit of hi is the sign.
/// Addition, subtraction, multiplication and the bitwise operations produce identical bit
/// patterns to the unsigned type (two's complement), so they delegate to it rather than
/// duplicating the arithmetic. The signed-specific pieces are the comparisons, the
/// arithmetic right shift, division, and negation.
typedef struct fixEmuInt128
{
	uint64_t lo;
	uint64_t hi;
} fixEmuInt128;

// ---- unsigned construction and lane access -------------------------------------

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Make( uint64_t hi, uint64_t lo )
{
	fixEmuUInt128 r;
	r.lo = lo;
	r.hi = hi;
	return r;
}

/// Zero-extend a 64-bit unsigned value, matching native conversion to unsigned __int128.
FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128FromU64( uint64_t v )
{
	return fixEmuUInt128Make( 0, v );
}

/// Sign-extend a 64-bit signed value. Native conversion of a negative int64_t to
/// unsigned __int128 wraps modulo 2^128, which fills the high lane with ones.
FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128FromI64( int64_t v )
{
	return fixEmuUInt128Make( v < 0 ? UINT64_C( 0xFFFFFFFFFFFFFFFF ) : 0, (uint64_t)v );
}

FIX_ALWAYS_INLINE uint64_t fixEmuUInt128Lo( fixEmuUInt128 a )
{
	return a.lo;
}

FIX_ALWAYS_INLINE uint64_t fixEmuUInt128Hi( fixEmuUInt128 a )
{
	return a.hi;
}

// ---- signed construction and lane access ---------------------------------------

FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Make( uint64_t hi, uint64_t lo )
{
	fixEmuInt128 r;
	r.lo = lo;
	r.hi = hi;
	return r;
}

/// Sign-extend a 64-bit signed value, matching native conversion to __int128.
FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128FromI64( int64_t v )
{
	return fixEmuInt128Make( v < 0 ? UINT64_C( 0xFFFFFFFFFFFFFFFF ) : 0, (uint64_t)v );
}

/// Zero-extend a 64-bit unsigned value. Every uint64_t is below 2^127, so native
/// conversion to __int128 is value-preserving with a zero high lane.
FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128FromU64( uint64_t v )
{
	return fixEmuInt128Make( 0, v );
}

/// The low lane, wrapping two's complement like a native narrowing conversion.
FIX_ALWAYS_INLINE int64_t fixEmuInt128ToI64( fixEmuInt128 a )
{
	return (int64_t)a.lo;
}

FIX_ALWAYS_INLINE uint64_t fixEmuInt128Lo( fixEmuInt128 a )
{
	return a.lo;
}

FIX_ALWAYS_INLINE uint64_t fixEmuInt128Hi( fixEmuInt128 a )
{
	return a.hi;
}

/// Bit-preserving conversions between the two emulated types, matching a native cast.
FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuToUnsigned( fixEmuInt128 a )
{
	return fixEmuUInt128Make( a.hi, a.lo );
}

FIX_ALWAYS_INLINE fixEmuInt128 fixEmuToSigned( fixEmuUInt128 a )
{
	return fixEmuInt128Make( a.hi, a.lo );
}

FIX_ALWAYS_INLINE bool fixEmuInt128IsNegative( fixEmuInt128 a )
{
	return ( a.hi >> 63 ) != 0;
}

// ---- unsigned comparison --------------------------------------------------------

FIX_ALWAYS_INLINE bool fixEmuUInt128Eq( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return a.lo == b.lo && a.hi == b.hi;
}

FIX_ALWAYS_INLINE bool fixEmuUInt128Lt( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return ( a.hi != b.hi ) ? ( a.hi < b.hi ) : ( a.lo < b.lo );
}

FIX_ALWAYS_INLINE bool fixEmuUInt128Gt( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return fixEmuUInt128Lt( b, a );
}

FIX_ALWAYS_INLINE bool fixEmuUInt128Le( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return !fixEmuUInt128Lt( b, a );
}

FIX_ALWAYS_INLINE bool fixEmuUInt128Ge( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return !fixEmuUInt128Lt( a, b );
}

// ---- signed comparison ----------------------------------------------------------

FIX_ALWAYS_INLINE bool fixEmuInt128Eq( fixEmuInt128 a, fixEmuInt128 b )
{
	return a.lo == b.lo && a.hi == b.hi;
}

/// Signed ordering: the high lanes compare signed, the low lanes break ties unsigned.
FIX_ALWAYS_INLINE bool fixEmuInt128Lt( fixEmuInt128 a, fixEmuInt128 b )
{
	if ( a.hi != b.hi )
	{
		return (int64_t)a.hi < (int64_t)b.hi;
	}
	return a.lo < b.lo;
}

FIX_ALWAYS_INLINE bool fixEmuInt128Gt( fixEmuInt128 a, fixEmuInt128 b )
{
	return fixEmuInt128Lt( b, a );
}

FIX_ALWAYS_INLINE bool fixEmuInt128Le( fixEmuInt128 a, fixEmuInt128 b )
{
	return !fixEmuInt128Lt( b, a );
}

FIX_ALWAYS_INLINE bool fixEmuInt128Ge( fixEmuInt128 a, fixEmuInt128 b )
{
	return !fixEmuInt128Lt( a, b );
}

// ---- unsigned bitwise -----------------------------------------------------------

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Not( fixEmuUInt128 a )
{
	return fixEmuUInt128Make( ~a.hi, ~a.lo );
}

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128And( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return fixEmuUInt128Make( a.hi & b.hi, a.lo & b.lo );
}

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Or( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return fixEmuUInt128Make( a.hi | b.hi, a.lo | b.lo );
}

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Xor( fixEmuUInt128 a, fixEmuUInt128 b )
{
	return fixEmuUInt128Make( a.hi ^ b.hi, a.lo ^ b.lo );
}

// ---- unsigned shifts ------------------------------------------------------------

/// Shifting a uint64_t lane by 64 is undefined behavior, so the half boundary is an
/// explicit branch. Shift counts outside [0, 127] yield zero.
FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Shl( fixEmuUInt128 a, int shift )
{
	if ( shift == 0 )
	{
		return a;
	}
	if ( shift > 0 && shift < 64 )
	{
		return fixEmuUInt128Make( ( a.hi << shift ) | ( a.lo >> ( 64 - shift ) ), a.lo << shift );
	}
	if ( shift >= 64 && shift < 128 )
	{
		return fixEmuUInt128Make( a.lo << ( shift - 64 ), 0 );
	}
	return fixEmuUInt128Make( 0, 0 );
}

/// Logical right shift. Shift counts outside [0, 127] yield zero.
FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Shr( fixEmuUInt128 a, int shift )
{
	if ( shift == 0 )
	{
		return a;
	}
	if ( shift > 0 && shift < 64 )
	{
		return fixEmuUInt128Make( a.hi >> shift, ( a.lo >> shift ) | ( a.hi << ( 64 - shift ) ) );
	}
	if ( shift >= 64 && shift < 128 )
	{
		return fixEmuUInt128Make( 0, a.hi >> ( shift - 64 ) );
	}
	return fixEmuUInt128Make( 0, 0 );
}

// ---- unsigned arithmetic --------------------------------------------------------

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Add( fixEmuUInt128 a, fixEmuUInt128 b )
{
	uint64_t lo = a.lo + b.lo;
	uint64_t hi = a.hi + b.hi + ( ( lo < a.lo ) ? 1 : 0 ); // carry out of the low lane
	return fixEmuUInt128Make( hi, lo );
}

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Sub( fixEmuUInt128 a, fixEmuUInt128 b )
{
	uint64_t lo = a.lo - b.lo;
	uint64_t hi = a.hi - b.hi - ( ( a.lo < b.lo ) ? 1 : 0 ); // borrow out of the low lane
	return fixEmuUInt128Make( hi, lo );
}

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Neg( fixEmuUInt128 a )
{
	return fixEmuUInt128Sub( fixEmuUInt128Make( 0, 0 ), a );
}

/// Exact 64x64 -> 128 unsigned product, schoolbook in 32-bit limbs. This is the
/// PORTABLE spelling, kept under its own name so test/int128_test.c can hold the
/// intrinsic-accelerated fixEmuUInt128MulU64 below to it on the compilers that have an
/// intrinsic. On every other compiler the two are the same function and the check is a
/// tautology -- which is exactly why it must not be the only test of this multiply.
FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128MulU64Schoolbook( uint64_t a, uint64_t b )
{
	const uint64_t aLow = a & UINT64_C( 0xFFFFFFFF );
	const uint64_t aHigh = a >> 32;
	const uint64_t bLow = b & UINT64_C( 0xFFFFFFFF );
	const uint64_t bHigh = b >> 32;

	const uint64_t productLowLow = aLow * bLow;
	const uint64_t productLowHigh = aLow * bHigh;
	const uint64_t productHighLow = aHigh * bLow;
	const uint64_t productHighHigh = aHigh * bHigh;

	const uint64_t carry =
		( ( productLowLow >> 32 ) + ( productLowHigh & UINT64_C( 0xFFFFFFFF ) ) + ( productHighLow & UINT64_C( 0xFFFFFFFF ) ) ) >> 32;

	uint64_t lo = productLowLow + ( productLowHigh << 32 ) + ( productHighLow << 32 );
	uint64_t hi = productHighHigh + ( productLowHigh >> 32 ) + ( productHighLow >> 32 ) + carry;
	return fixEmuUInt128Make( hi, lo );
}

// The one place an intrinsic earns its keep: this multiply is on the path of every
// fixMul, so an emulated build runs it once per multiply in the solver. _umul128 and
// __umulh are the exact 128-bit product -- there is no semantic freedom in "the exact
// product", so the substitution cannot change a result, and the schoolbook form above
// stays compiled as the differential partner.
#if defined( _MSC_VER ) && !defined( __clang__ ) && ( defined( _M_X64 ) || defined( _M_ARM64 ) )
	#include <intrin.h>
	#define FIX_INT128_HAS_MUL_INTRINSIC 1
#else
	#define FIX_INT128_HAS_MUL_INTRINSIC 0
#endif

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128MulU64( uint64_t a, uint64_t b )
{
#if FIX_INT128_HAS_MUL_INTRINSIC && defined( _M_X64 )
	uint64_t hi;
	uint64_t lo = _umul128( a, b, &hi );
	return fixEmuUInt128Make( hi, lo );
#elif FIX_INT128_HAS_MUL_INTRINSIC && defined( _M_ARM64 )
	return fixEmuUInt128Make( __umulh( a, b ), a * b );
#else
	return fixEmuUInt128MulU64Schoolbook( a, b );
#endif
}

/// Full 128x128 -> 128 product (the high half of the true 256-bit product is discarded,
/// exactly as native two's complement multiplication discards it).
FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Mul( fixEmuUInt128 a, fixEmuUInt128 b )
{
	fixEmuUInt128 r = fixEmuUInt128MulU64( a.lo, b.lo );
	r.hi += a.lo * b.hi + a.hi * b.lo;
	return r;
}

/// Shift-subtract long division, producing quotient and remainder together.
/// Division by zero is undefined; this returns zero for both to stay total (see the
/// header comment). Out-parameters may be NULL if only one half is wanted.
FIX_ALWAYS_INLINE void fixEmuUInt128DivMod( fixEmuUInt128 dividend, fixEmuUInt128 divisor, fixEmuUInt128* quotientOut,
											fixEmuUInt128* remainderOut )
{
	fixEmuUInt128 quotient = fixEmuUInt128Make( 0, 0 );
	fixEmuUInt128 remainder = fixEmuUInt128Make( 0, 0 );

	if ( divisor.hi == 0 && divisor.lo == 0 )
	{
		if ( quotientOut != NULL ) *quotientOut = quotient;
		if ( remainderOut != NULL ) *remainderOut = remainder;
		return;
	}

	if ( dividend.hi == 0 && divisor.hi == 0 )
	{
		quotient = fixEmuUInt128FromU64( dividend.lo / divisor.lo );
		remainder = fixEmuUInt128FromU64( dividend.lo % divisor.lo );
		if ( quotientOut != NULL ) *quotientOut = quotient;
		if ( remainderOut != NULL ) *remainderOut = remainder;
		return;
	}

	for ( int i = 127; i >= 0; i-- )
	{
		// The reference spells this ( dividend >> i ).lo & 1; selecting the lane directly
		// is the same bit for every i in [0, 127] and skips a 128-bit shift per iteration.
		uint64_t bit = ( i < 64 ) ? ( ( dividend.lo >> i ) & 1 ) : ( ( dividend.hi >> ( i - 64 ) ) & 1 );

		remainder = fixEmuUInt128Shl( remainder, 1 );
		remainder.lo |= bit;

		if ( fixEmuUInt128Ge( remainder, divisor ) )
		{
			remainder = fixEmuUInt128Sub( remainder, divisor );
			quotient = fixEmuUInt128Or( quotient, fixEmuUInt128Shl( fixEmuUInt128FromU64( 1 ), i ) );
		}
	}

	if ( quotientOut != NULL ) *quotientOut = quotient;
	if ( remainderOut != NULL ) *remainderOut = remainder;
}

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Div( fixEmuUInt128 a, fixEmuUInt128 b )
{
	fixEmuUInt128 quotient;
	fixEmuUInt128DivMod( a, b, &quotient, NULL );
	return quotient;
}

FIX_ALWAYS_INLINE fixEmuUInt128 fixEmuUInt128Mod( fixEmuUInt128 a, fixEmuUInt128 b )
{
	fixEmuUInt128 remainder;
	fixEmuUInt128DivMod( a, b, NULL, &remainder );
	return remainder;
}

// ---- signed arithmetic ----------------------------------------------------------
//
// Two's complement: addition, subtraction, multiplication and the bitwise operations
// are the same bit patterns as unsigned, so they delegate. Signed overflow wraps by
// construction, exactly like the underlying hardware.

FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Add( fixEmuInt128 a, fixEmuInt128 b )
{
	return fixEmuToSigned( fixEmuUInt128Add( fixEmuToUnsigned( a ), fixEmuToUnsigned( b ) ) );
}

FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Sub( fixEmuInt128 a, fixEmuInt128 b )
{
	return fixEmuToSigned( fixEmuUInt128Sub( fixEmuToUnsigned( a ), fixEmuToUnsigned( b ) ) );
}

FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Mul( fixEmuInt128 a, fixEmuInt128 b )
{
	return fixEmuToSigned( fixEmuUInt128Mul( fixEmuToUnsigned( a ), fixEmuToUnsigned( b ) ) );
}

/// Two's complement negation. -INT128_MIN wraps to itself, like native.
FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Neg( fixEmuInt128 a )
{
	return fixEmuToSigned( fixEmuUInt128Neg( fixEmuToUnsigned( a ) ) );
}

/// Widening signed 64x64 -> 128 product. The hot one: this is the multiply inside
/// fixMul, fixDot, fixCofactor128 and every other exact accumulation in the library.
/// Computed as the unsigned product of the bit patterns, which is the same low 128 bits
/// as the signed product in two's complement.
FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128MulI64( int64_t a, int64_t b )
{
	fixEmuUInt128 product = fixEmuUInt128MulU64( (uint64_t)a, (uint64_t)b );
	// Fold in the sign-extension lanes: the full product of the sign-extended operands is
	// unsigned_product - ( a < 0 ? b : 0 ) * 2^64 - ( b < 0 ? a : 0 ) * 2^64.
	if ( a < 0 )
	{
		product.hi -= (uint64_t)b;
	}
	if ( b < 0 )
	{
		product.hi -= (uint64_t)a;
	}
	return fixEmuToSigned( product );
}

/// Logical left shift of the bit pattern, matching native two's complement hardware.
/// Shift counts outside [0, 127] yield zero.
FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Shl( fixEmuInt128 a, int shift )
{
	return fixEmuToSigned( fixEmuUInt128Shl( fixEmuToUnsigned( a ), shift ) );
}

/// ARITHMETIC right shift: the vacated high bits fill with the sign. Shift counts
/// outside [0, 127] yield all sign bits, the limit of shifting further.
FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Shr( fixEmuInt128 a, int shift )
{
	if ( shift < 0 || shift >= 128 )
	{
		return fixEmuInt128IsNegative( a ) ? fixEmuInt128FromI64( -1 ) : fixEmuInt128FromI64( 0 );
	}

	fixEmuUInt128 result = fixEmuUInt128Shr( fixEmuToUnsigned( a ), shift );
	if ( fixEmuInt128IsNegative( a ) && shift > 0 )
	{
		result = fixEmuUInt128Or( result, fixEmuUInt128Shl( fixEmuUInt128Not( fixEmuUInt128Make( 0, 0 ) ), 128 - shift ) );
	}
	return fixEmuToSigned( result );
}

/// Sign extraction, unsigned division on the magnitudes, sign application: C semantics,
/// truncation toward zero with the remainder's sign following the dividend.
FIX_ALWAYS_INLINE void fixEmuInt128DivMod( fixEmuInt128 dividend, fixEmuInt128 divisor, fixEmuInt128* quotientOut,
										   fixEmuInt128* remainderOut )
{
	const bool dividendNegative = fixEmuInt128IsNegative( dividend );
	const bool divisorNegative = fixEmuInt128IsNegative( divisor );

	fixEmuUInt128 dividendMagnitude = fixEmuToUnsigned( dividend );
	fixEmuUInt128 divisorMagnitude = fixEmuToUnsigned( divisor );
	if ( dividendNegative ) dividendMagnitude = fixEmuUInt128Neg( dividendMagnitude );
	if ( divisorNegative ) divisorMagnitude = fixEmuUInt128Neg( divisorMagnitude );

	fixEmuUInt128 quotient;
	fixEmuUInt128 remainder;
	fixEmuUInt128DivMod( dividendMagnitude, divisorMagnitude, &quotient, &remainder );

	if ( quotientOut != NULL )
	{
		*quotientOut = fixEmuToSigned( ( dividendNegative != divisorNegative ) ? fixEmuUInt128Neg( quotient ) : quotient );
	}
	if ( remainderOut != NULL )
	{
		*remainderOut = fixEmuToSigned( dividendNegative ? fixEmuUInt128Neg( remainder ) : remainder );
	}
}

FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Div( fixEmuInt128 a, fixEmuInt128 b )
{
	fixEmuInt128 quotient;
	fixEmuInt128DivMod( a, b, &quotient, NULL );
	return quotient;
}

FIX_ALWAYS_INLINE fixEmuInt128 fixEmuInt128Mod( fixEmuInt128 a, fixEmuInt128 b )
{
	fixEmuInt128 remainder;
	fixEmuInt128DivMod( a, b, NULL, &remainder );
	return remainder;
}

// ================================================================================
// LAYER 2: fixInt128 / fixUInt128 -- the types the library speaks.
// ================================================================================

#if defined( __SIZEOF_INT128__ ) && !defined( FIX_FORCE_EMULATED_INT128 )

/// 1 when this build carries 128-bit integers. Always 1 -- the emulated pair covers the
/// compilers that lack __int128. Kept as a macro because consumers test it.
#define FIX_HAS_INT128 1

/// 1 when fixInt128 is the emulated pair rather than a compiler __int128.
#define FIX_INT128_EMULATED 0

// __extension__ keeps -Wpedantic quiet: __int128 is not ISO C.
__extension__ typedef __int128 fixInt128;
__extension__ typedef unsigned __int128 fixUInt128;

#else

#define FIX_HAS_INT128 1
#define FIX_INT128_EMULATED 1

typedef fixEmuInt128 fixInt128;
typedef fixEmuUInt128 fixUInt128;

#endif

// The vocabulary. Every 128-bit operation in this library goes through one of these
// names, which is what makes the emulated arm possible at all.
//
// ONE FUNCTION IS EXEMPT, and it is written down here so the exemption stays countable:
// fixMul in fixed.h spells its rounding expression a second time in bare native
// operators, under `#if FIX_INT128_EMULATED`, because at -O0 the per-call stack traffic
// of the seam spelling costs a consumer 40% of its debug test run. It is a transcription
// of these bodies rather than a second algorithm, and the emulated build of the entire
// test suite asserts the same frozen hashes, so the two arms cannot drift apart in
// silence. Anything else reaching for a bare operator should use these names instead.
//
// On the native arm each one is a FIX_ALWAYS_INLINE one-liner over the operator it
// replaced, so -O2 code generation is unchanged. That is checked rather than asserted:
// the pull request that introduced this header diffed the compiled library object
// before and after and found the instruction stream identical.

#if FIX_INT128_EMULATED == 0

FIX_ALWAYS_INLINE fixUInt128 fixUInt128Make( uint64_t hi, uint64_t lo ) { return ( (fixUInt128)hi << 64 ) | lo; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128FromU64( uint64_t v ) { return (fixUInt128)v; }
FIX_ALWAYS_INLINE uint64_t fixUInt128Lo( fixUInt128 a ) { return (uint64_t)a; }
FIX_ALWAYS_INLINE uint64_t fixUInt128Hi( fixUInt128 a ) { return (uint64_t)( a >> 64 ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Add( fixUInt128 a, fixUInt128 b ) { return a + b; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Sub( fixUInt128 a, fixUInt128 b ) { return a - b; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Mul( fixUInt128 a, fixUInt128 b ) { return a * b; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128MulU64( uint64_t a, uint64_t b ) { return (fixUInt128)a * b; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Neg( fixUInt128 a ) { return -a; }

/// Divide a 128-bit value by a 64-bit divisor, returning the quotient and, through the
/// pointer, the remainder.
///
/// PRECONDITION: fixUInt128Hi( a ) < d. That is what makes the quotient fit in 64 bits,
/// and it is the caller's job -- this is the inner primitive of Knuth Algorithm D, whose
/// normalization step establishes exactly that invariant, rather than a general-purpose
/// divide. Violating it on the native arm truncates; there is no check, because the one
/// caller proves the precondition and a branch here would sit inside the division loop.
FIX_ALWAYS_INLINE uint64_t fixUInt128DivRemBy64( fixUInt128 a, uint64_t d, uint64_t* remainder )
{
#if defined( __x86_64__ )
	// The hardware 128/64 divide, which is exactly this operation. divq traps when the
	// quotient overflows 64 bits, and the precondition above -- high half below the
	// divisor -- is precisely what rules that out, so it cannot fault here. volatile for
	// the same reason fixInt128Div needs it: a non-volatile asm is assumed side-effect
	// free and can be hoisted above the guard that makes it safe.
	uint64_t quotient, rest;
	uint64_t high = (uint64_t)( a >> 64 );
	uint64_t low = (uint64_t)a;
	__asm__ volatile( "divq %[v]" : "=a"( quotient ), "=d"( rest ) : [v] "r"( d ), "a"( low ), "d"( high ) );
	*remainder = rest;
	return quotient;
#elif defined( _WIN32 )
	// ClangCL does not link compiler-rt builtins, so native 128-bit division (__udivti3)
	// is unavailable and this arm restores shift-subtract. Bit-identical, and reached only
	// on Windows targets without the instruction above -- arm64 clang-cl today. The same
	// reasoning and the same fallback as fixInt128Div.
	fixUInt128 quotient = 0;
	fixUInt128 rest = 0;
	for ( int i = 127; i >= 0; i-- )
	{
		rest = ( rest << 1 ) | ( ( a >> i ) & 1 );
		if ( rest >= (fixUInt128)d )
		{
			rest -= (fixUInt128)d;
			quotient |= ( (fixUInt128)1 ) << i;
		}
	}
	*remainder = (uint64_t)rest;
	return (uint64_t)quotient;
#else
	fixUInt128 quotient = a / d;
	*remainder = (uint64_t)( a - quotient * d );
	return (uint64_t)quotient;
#endif
}
// The shifts are the plain operators, with no guard on the count. That is deliberate and
// it is the one place the two arms differ: a shift count outside [0, 127] is undefined
// behavior for native __int128, so there is no native answer for the emulation to match,
// and a guard here would put a compare and a select on the path of every fixMul and
// fixDiv to defend against an input this library never produces. The emulated arm is
// TOTAL instead (zero, or all sign bits) so that it cannot trap; the differential test
// restricts itself to [0, 127] for the same reason it excludes zero divisors.
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Shl( fixUInt128 a, int shift ) { return a << shift; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Shr( fixUInt128 a, int shift ) { return a >> shift; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Or( fixUInt128 a, fixUInt128 b ) { return a | b; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128And( fixUInt128 a, fixUInt128 b ) { return a & b; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Xor( fixUInt128 a, fixUInt128 b ) { return a ^ b; }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Not( fixUInt128 a ) { return ~a; }
FIX_ALWAYS_INLINE bool fixUInt128Eq( fixUInt128 a, fixUInt128 b ) { return a == b; }
FIX_ALWAYS_INLINE bool fixUInt128Lt( fixUInt128 a, fixUInt128 b ) { return a < b; }
FIX_ALWAYS_INLINE bool fixUInt128Gt( fixUInt128 a, fixUInt128 b ) { return a > b; }
FIX_ALWAYS_INLINE bool fixUInt128Le( fixUInt128 a, fixUInt128 b ) { return a <= b; }
FIX_ALWAYS_INLINE bool fixUInt128Ge( fixUInt128 a, fixUInt128 b ) { return a >= b; }

FIX_ALWAYS_INLINE fixInt128 fixInt128Make( uint64_t hi, uint64_t lo ) { return (fixInt128)( ( (fixUInt128)hi << 64 ) | lo ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128FromI64( int64_t v ) { return (fixInt128)v; }
FIX_ALWAYS_INLINE fixInt128 fixInt128FromU64( uint64_t v ) { return (fixInt128)v; }
FIX_ALWAYS_INLINE int64_t fixInt128ToI64( fixInt128 a ) { return (int64_t)a; }
FIX_ALWAYS_INLINE uint64_t fixInt128Lo( fixInt128 a ) { return (uint64_t)a; }
FIX_ALWAYS_INLINE uint64_t fixInt128Hi( fixInt128 a ) { return (uint64_t)( (fixUInt128)a >> 64 ); }
FIX_ALWAYS_INLINE fixUInt128 fixInt128ToUnsigned( fixInt128 a ) { return (fixUInt128)a; }
FIX_ALWAYS_INLINE fixInt128 fixInt128FromUnsigned( fixUInt128 a ) { return (fixInt128)a; }
// THROUGH THE UNSIGNED TYPE, and this is a correctness matter rather than a style one.
// The emulated arm wraps on overflow because two's complement lanes wrap; native signed
// overflow is UNDEFINED. If the native arm kept the bare signed operators the two arms
// would not agree at the boundary -- which is the one thing this seam exists to promise --
// and the difference would be invisible until a compiler decided to exploit the UB. The
// cast is free: the same instruction, with defined semantics, verified by diffing the
// compiled consumer. It is the idiom fixShiftLeft already uses for the same reason.
FIX_ALWAYS_INLINE fixInt128 fixInt128Add( fixInt128 a, fixInt128 b )
{
	return (fixInt128)( (fixUInt128)a + (fixUInt128)b );
}
FIX_ALWAYS_INLINE fixInt128 fixInt128Sub( fixInt128 a, fixInt128 b )
{
	return (fixInt128)( (fixUInt128)a - (fixUInt128)b );
}
FIX_ALWAYS_INLINE fixInt128 fixInt128Mul( fixInt128 a, fixInt128 b )
{
	return (fixInt128)( (fixUInt128)a * (fixUInt128)b );
}
// The widening multiply cannot overflow: the largest product of two int64 values is 2^126.
FIX_ALWAYS_INLINE fixInt128 fixInt128MulI64( int64_t a, int64_t b ) { return (fixInt128)a * b; }
FIX_ALWAYS_INLINE fixInt128 fixInt128Neg( fixInt128 a ) { return (fixInt128)( -(fixUInt128)a ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128Shr( fixInt128 a, int shift ) { return a >> shift; }
FIX_ALWAYS_INLINE bool fixInt128Eq( fixInt128 a, fixInt128 b ) { return a == b; }
FIX_ALWAYS_INLINE bool fixInt128Lt( fixInt128 a, fixInt128 b ) { return a < b; }
FIX_ALWAYS_INLINE bool fixInt128Gt( fixInt128 a, fixInt128 b ) { return a > b; }
FIX_ALWAYS_INLINE bool fixInt128Le( fixInt128 a, fixInt128 b ) { return a <= b; }
FIX_ALWAYS_INLINE bool fixInt128Ge( fixInt128 a, fixInt128 b ) { return a >= b; }
FIX_ALWAYS_INLINE bool fixInt128IsNegative( fixInt128 a ) { return a < 0; }

#else

FIX_ALWAYS_INLINE fixUInt128 fixUInt128Make( uint64_t hi, uint64_t lo ) { return fixEmuUInt128Make( hi, lo ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128FromU64( uint64_t v ) { return fixEmuUInt128FromU64( v ); }
FIX_ALWAYS_INLINE uint64_t fixUInt128Lo( fixUInt128 a ) { return fixEmuUInt128Lo( a ); }
FIX_ALWAYS_INLINE uint64_t fixUInt128Hi( fixUInt128 a ) { return fixEmuUInt128Hi( a ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Add( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Add( a, b ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Sub( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Sub( a, b ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Mul( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Mul( a, b ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128MulU64( uint64_t a, uint64_t b ) { return fixEmuUInt128MulU64( a, b ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Neg( fixUInt128 a ) { return fixEmuUInt128Neg( a ); }

/// Divide a 128-bit value by a 64-bit divisor, returning the quotient and, through the
/// pointer, the remainder.
///
/// PRECONDITION: fixUInt128Hi( a ) < d. That is what makes the quotient fit in 64 bits,
/// and it is the caller's job -- this is the inner primitive of Knuth Algorithm D, whose
/// normalization step establishes exactly that invariant, rather than a general-purpose
/// divide. Violating it on the native arm truncates; there is no check, because the one
/// caller proves the precondition and a branch here would sit inside the division loop.
FIX_ALWAYS_INLINE uint64_t fixUInt128DivRemBy64( fixUInt128 a, uint64_t d, uint64_t* remainder )
{
	fixEmuUInt128 quotient, rest;
	fixEmuUInt128DivMod( a, fixEmuUInt128FromU64( d ), &quotient, &rest );
	*remainder = fixEmuUInt128Lo( rest );
	return fixEmuUInt128Lo( quotient );
}
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Shl( fixUInt128 a, int shift ) { return fixEmuUInt128Shl( a, shift ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Shr( fixUInt128 a, int shift ) { return fixEmuUInt128Shr( a, shift ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Or( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Or( a, b ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128And( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128And( a, b ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Xor( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Xor( a, b ); }
FIX_ALWAYS_INLINE fixUInt128 fixUInt128Not( fixUInt128 a ) { return fixEmuUInt128Not( a ); }
FIX_ALWAYS_INLINE bool fixUInt128Eq( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Eq( a, b ); }
FIX_ALWAYS_INLINE bool fixUInt128Lt( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Lt( a, b ); }
FIX_ALWAYS_INLINE bool fixUInt128Gt( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Gt( a, b ); }
FIX_ALWAYS_INLINE bool fixUInt128Le( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Le( a, b ); }
FIX_ALWAYS_INLINE bool fixUInt128Ge( fixUInt128 a, fixUInt128 b ) { return fixEmuUInt128Ge( a, b ); }

FIX_ALWAYS_INLINE fixInt128 fixInt128Make( uint64_t hi, uint64_t lo ) { return fixEmuInt128Make( hi, lo ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128FromI64( int64_t v ) { return fixEmuInt128FromI64( v ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128FromU64( uint64_t v ) { return fixEmuInt128FromU64( v ); }
FIX_ALWAYS_INLINE int64_t fixInt128ToI64( fixInt128 a ) { return fixEmuInt128ToI64( a ); }
FIX_ALWAYS_INLINE uint64_t fixInt128Lo( fixInt128 a ) { return fixEmuInt128Lo( a ); }
FIX_ALWAYS_INLINE uint64_t fixInt128Hi( fixInt128 a ) { return fixEmuInt128Hi( a ); }
FIX_ALWAYS_INLINE fixUInt128 fixInt128ToUnsigned( fixInt128 a ) { return fixEmuToUnsigned( a ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128FromUnsigned( fixUInt128 a ) { return fixEmuToSigned( a ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128Add( fixInt128 a, fixInt128 b ) { return fixEmuInt128Add( a, b ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128Sub( fixInt128 a, fixInt128 b ) { return fixEmuInt128Sub( a, b ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128Mul( fixInt128 a, fixInt128 b ) { return fixEmuInt128Mul( a, b ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128MulI64( int64_t a, int64_t b ) { return fixEmuInt128MulI64( a, b ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128Neg( fixInt128 a ) { return fixEmuInt128Neg( a ); }
FIX_ALWAYS_INLINE fixInt128 fixInt128Shr( fixInt128 a, int shift ) { return fixEmuInt128Shr( a, shift ); }
FIX_ALWAYS_INLINE bool fixInt128Eq( fixInt128 a, fixInt128 b ) { return fixEmuInt128Eq( a, b ); }
FIX_ALWAYS_INLINE bool fixInt128Lt( fixInt128 a, fixInt128 b ) { return fixEmuInt128Lt( a, b ); }
FIX_ALWAYS_INLINE bool fixInt128Gt( fixInt128 a, fixInt128 b ) { return fixEmuInt128Gt( a, b ); }
FIX_ALWAYS_INLINE bool fixInt128Le( fixInt128 a, fixInt128 b ) { return fixEmuInt128Le( a, b ); }
FIX_ALWAYS_INLINE bool fixInt128Ge( fixInt128 a, fixInt128 b ) { return fixEmuInt128Ge( a, b ); }
FIX_ALWAYS_INLINE bool fixInt128IsNegative( fixInt128 a ) { return fixEmuInt128IsNegative( a ); }

#endif

/// Zero and one, spelled once so call sites do not have to.
#define FIX_INT128_ZERO fixInt128FromI64( 0 )
#define FIX_UINT128_ZERO fixUInt128FromU64( 0 )
