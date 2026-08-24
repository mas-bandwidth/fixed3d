// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// DOMAIN CROSSING: arbitrary-scale conversion, and moving between two Q formats.
//
// The rest of this library is pinned to its own formats -- fixFromDouble speaks Q48.16,
// fix30FromFix speaks Q48.16 to Q2.30. This header is the general case: any scale, and
// any shift between two integer domains. It exists because a consumer's wire format is
// rarely one of this library's formats. A rotation sent at 10 fraction bits, a position
// in hundredths of a unit, a component held at 30 bits in simulation and sent at 10 --
// none of those have a pinned converter and all of them are the same two operations.
//
// Ported from delta (mas-bandwidth/delta, delta_quantize.h), whose spelling is the
// reference: delta and this library must agree bit for bit, because delta is about to
// depend on fixed and delete its own copies. test/quantize_test.c holds every function
// here to a verbatim transcription of delta's body.
//
// THE ONE RULE THAT MATTERS MORE THAN ALL THE FUNCTIONS BELOW, carried over from delta
// because it is the thing that actually goes wrong:
//
//     QUANTIZE ON THE AUTHORITY, ONCE, AND TREAT THE RESULT AS THE TRUTH.
//
// The quantized integer is what goes into the baseline, what a delta is computed against,
// and what the client decodes. If instead the float stays the truth and each side
// quantizes independently, there are two simulations rounding separately, the baselines
// drift apart, and the delta decodes to a value the server never held. That failure is
// subtle, delayed, and invisible until scale.
//
// TWO DIFFERENT ROUNDING RULES LIVE IN THIS HEADER, and the asymmetry is deliberate:
//
//   fixQuantize rounds HALF AWAY FROM ZERO. It is symmetric about the origin, which
//   matters for a signed world -- half up would bias every negative coordinate toward
//   the origin and every positive one away from it. It runs on ONE machine (the
//   authority), so it can afford symmetry, and it takes a double, so it could not be
//   made portable anyway.
//
//   fixNarrow rounds HALF TOWARD POSITIVE INFINITY, which is what the arithmetic shift
//   gives for free. BOTH SIDES run it, so it can afford only agreement, and it is
//   integer-only for exactly that reason. This is the same rule the Q2.30 converters in
//   fixed.h already use -- fixFromFix30( a ) is fixNarrow( a.raw, FIX30_SHIFT ), and the
//   test asserts it -- so the general form and the pinned form are one rule, not two.
//
// NOT TO BE CONFUSED WITH THE WIDE FAMILY. fixNarrow and fixWiden move a value between
// two Q formats inside 64 bits by shifting the fraction point. fixWideToFixed and
// fixWideFromFixed in fixed_wide.h move between 64-bit and 128-bit STORAGE at the same
// fraction point. Different axis entirely: one changes precision, the other changes
// range.
#pragma once

#include "fixed/base.h"
#include "fixed/fixed.h"

#include <stdint.h>

// ---------------------------------------------------------------------------------------------
// ARBITRARY SCALE
//
// `scale` is the integer value that represents 1.0 in the target domain: FIX_ONE for a
// Q48.16 component, 1024 for a rotation at 10 fraction bits, 100 for hundredths of a unit.
// ---------------------------------------------------------------------------------------------

/// Convert a value to its raw integer at an arbitrary scale, rounding half away from zero.
///
/// A BOUNDARY HELPER, and the caveat is the same one fixFromDouble carries: this takes a
/// double, so the compiler's choice of instruction can move the result by one unit at a
/// tie. That is fine where quantization happens on the authority only -- one machine, one
/// answer, distributed as an integer. It is NOT fine anywhere both sides must agree, which
/// is why fixNarrow and fixWiden below are integer-only.
FIX_ALWAYS_INLINE int64_t fixQuantize( double value, int64_t scale )
{
	FIX_ASSERT( scale > 0 );

	const double scaled = value * (double)scale;

	return ( scaled >= 0.0 ) ? (int64_t)( scaled + 0.5 ) : -(int64_t)( -scaled + 0.5 );
}

/// Convert a raw integer at an arbitrary scale back to a value. Debug, display, and the
/// far side of a boundary that has already been crossed once.
FIX_ALWAYS_INLINE double fixDequantize( int64_t raw, int64_t scale )
{
	FIX_ASSERT( scale > 0 );

	return (double)raw / (double)scale;
}

/// The same crossing with the domain bound applied. USE THIS ONE at the authority
/// boundary rather than the bare form above, and treat the clamp as a DIAGNOSTIC rather
/// than a convenience: an object outside the declared world is a simulation bug, and
/// whatever consumes the raw value next will complain about it somewhere further from the
/// cause.
///
/// minRaw and maxRaw are in the RAW domain -- for a Q(i.f) field they are
/// minUnits << f and maxUnits << f.
FIX_ALWAYS_INLINE int64_t fixQuantizeClamped( double value, int64_t scale, int64_t minRaw, int64_t maxRaw )
{
	FIX_ASSERT( minRaw <= maxRaw );

	int64_t raw = fixQuantize( value, scale );

	if ( raw < minRaw )
	{
		raw = minRaw;
	}
	else if ( raw > maxRaw )
	{
		raw = maxRaw;
	}

	return raw;
}

/// Is this raw value inside the declared domain? The predicate behind the clamp above,
/// separated so a caller can treat the answer as a bug report instead of clamping.
FIX_ALWAYS_INLINE bool fixFits( int64_t raw, int64_t minRaw, int64_t maxRaw )
{
	return raw >= minRaw && raw <= maxRaw;
}

// ---------------------------------------------------------------------------------------------
// NARROWING AND WIDENING BETWEEN TWO INTEGER DOMAINS
//
// The pair for carrying a value at one precision in simulation and a coarser one on the
// wire. BOTH SIDES RUN THIS, so it is integer-only and exactly specified.
//
// fixWiden is exact and lossless, and fixNarrow( fixWiden( v, n ), n ) is the identity.
// The other order is not, and that is the whole point -- narrowing is where the bits go.
// ---------------------------------------------------------------------------------------------

/// Drop `shift` fraction bits, rounding half toward positive infinity.
///
/// The rounding is the arithmetic shift's own behaviour rather than a choice layered on
/// top, which is why this is the rule both sides can rely on: there is nothing here for a
/// compiler to do differently.
FIX_ALWAYS_INLINE int64_t fixNarrow( int64_t raw, int shift )
{
	FIX_ASSERT( shift >= 0 );
	FIX_ASSERT( shift < 63 );

	const int64_t half = ( (int64_t)1 << shift ) >> 1;

	FIX_ASSERT( raw <= INT64_MAX - half );

	// The add is unsigned so that a caller who ignores the assertion above gets defined
	// two's-complement wrap instead of signed-overflow undefined behaviour. Identical bits
	// for every input the assertion admits; the same form fixRoundToInt already uses.
	return (int64_t)( (uint64_t)raw + (uint64_t)half ) >> shift;
}

/// Add `shift` fraction bits. Exact and lossless within the asserted range.
FIX_ALWAYS_INLINE int64_t fixWiden( int64_t raw, int shift )
{
	FIX_ASSERT( shift >= 0 );
	FIX_ASSERT( shift < 63 );
	FIX_ASSERT( raw >= ( INT64_MIN >> shift ) && raw <= ( INT64_MAX >> shift ) );

	// Through fixShiftLeft: shifting a negative value left is undefined behaviour in C
	// even when it does not overflow, and half the values a signed world produces are
	// negative. Same bits as the raw shift, and it keeps UBSan quiet.
	return fixShiftLeft( raw, shift );
}
