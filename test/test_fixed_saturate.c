// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// BOX3D_FIXED_SATURATE REACHES THE LIBRARY. This file alone is compiled with
// BOX3D_FIXED_SATURATE (test/CMakeLists.txt), and box3d/fixed_compat.h must turn that into
// the library's FIX_SATURATE before fixed/fixed.h defines its inlines. It used to define it
// after the include, so every product below wrapped in a build that asked for saturation.
//
// Only the inline arithmetic is asserted, and only on operations whose saturating form the
// pinned `fixed` already has (the fixMul family). The rest of the suite runs the default
// policy; the inlines are static in C, so the two policies cannot meet across files.

#include "test_macros.h"

#include "box3d/math_functions.h"

#if !defined( BOX3D_FIXED_SATURATE )
	#error "test_fixed_saturate.c is built with BOX3D_FIXED_SATURATE"
#endif

#if !defined( FIX_SATURATE )
	#error "BOX3D_FIXED_SATURATE did not reach fixed as FIX_SATURATE"
#endif

static int SaturatingProducts( void )
{
	b3Fixed big = B3_FIX( 1.0e12 );

	// 1e24 is past the 1.4e14 range: a wrapping product keeps the low 64 bits.
	ENSURE( b3FixMul( big, big ) == B3_FIXED_MAX );
	ENSURE( b3FixMul( big, -big ) == B3_FIXED_MIN );
	ENSURE( b3FixMul( B3_FIXED_MAX, B3_FIX( 2.0f ) ) == B3_FIXED_MAX );
	ENSURE( b3FixMul( B3_FIXED_MIN, B3_FIX( 2.0f ) ) == B3_FIXED_MIN );

	b3Vec3 v = { big, -big, big };
	b3Vec3 scaled = b3MulSV( big, v );
	ENSURE( scaled.x == B3_FIXED_MAX && scaled.y == B3_FIXED_MIN && scaled.z == B3_FIXED_MAX );

	b3Vec3 product = b3Mul( v, v );
	ENSURE( product.x == B3_FIXED_MAX && product.y == B3_FIXED_MAX && product.z == B3_FIXED_MAX );

	// cross( x, y ) = z: each component is one product less another, and here the one that
	// is not zero is far past the range.
	b3Vec3 x = { big, 0, 0 };
	b3Vec3 y = { 0, big, 0 };
	b3Vec3 z = b3Cross( x, y );
	ENSURE( z.x == 0 && z.y == 0 && z.z == B3_FIXED_MAX );
	z = b3Cross( y, x );
	ENSURE( z.x == 0 && z.y == 0 && z.z == B3_FIXED_MIN );

	// In range the policy changes nothing.
	ENSURE( b3FixMul( B3_FIX( 3.0f ), B3_FIX( 0.5f ) ) == B3_FIX( 1.5f ) );
	return 0;
}

int FixedSaturateTest( void )
{
	RUN_SUBTEST( SaturatingProducts );
	return 0;
}
