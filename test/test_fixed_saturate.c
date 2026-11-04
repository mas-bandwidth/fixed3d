// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// BOX3D_FIXED_SATURATE REACHES THE LIBRARY. This file alone is compiled with
// BOX3D_FIXED_SATURATE (test/CMakeLists.txt), and box3d/fixed_compat.h must turn that into
// the library's FIX_SATURATE before fixed/fixed.h defines its inlines. It used to define it
// after the include, so every product below wrapped in a build that asked for saturation.
//
// Only the inline arithmetic is asserted: the fixMul family, and since the pin moved past
// fixed #59-#62 the adds, subtracts and 128-bit reductions built on it. The rest of the
// suite runs the default policy; the inlines are static in C, so the two policies cannot
// meet across files.

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

// The sums. Each of these wrapped under BOX3D_FIXED_SATURATE until the pin reached fixed
// #59-#62: past the range they hold the edge with the sign of the exact result.
static int SaturatingSums( void )
{
	b3Vec3 top = { B3_FIXED_MAX, B3_FIXED_MAX, B3_FIXED_MAX };
	b3Vec3 bottom = { B3_FIXED_MIN, B3_FIXED_MIN, B3_FIXED_MIN };
	b3Vec3 one = { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) };

	b3Vec3 sum = b3Add( top, one );
	ENSURE( sum.x == B3_FIXED_MAX && sum.y == B3_FIXED_MAX && sum.z == B3_FIXED_MAX );
	b3Vec3 difference = b3Sub( bottom, one );
	ENSURE( difference.x == B3_FIXED_MIN && difference.y == B3_FIXED_MIN && difference.z == B3_FIXED_MIN );
	difference = b3Sub( top, bottom );
	ENSURE( difference.x == B3_FIXED_MAX );

	// The three squares of 2^47 - 1 sum to 3 * 2^126, past int128: this read as B3_FIXED_MIN.
	b3Fixed big = (b3Fixed)( ( ( (uint64_t)1 << 47 ) - 1 ) << 16 );
	b3Vec3 v = { big, big, big };
	ENSURE( b3Dot( v, v ) == B3_FIXED_MAX );
	ENSURE( b3Dot( v, b3Neg( v ) ) == B3_FIXED_MIN );
	ENSURE( b3LengthSquared( v ) == B3_FIXED_MAX );
	ENSURE( b3Distance( bottom, top ) == B3_FIXED_MAX );

	// Four squares of B3_FIXED_MAX reach 2^128.
	b3Quat q = { top, B3_FIXED_MAX };
	ENSURE( b3DotQuat( q, q ) == B3_FIXED_MAX );
	ENSURE( b3MulQuat( q, b3NegateQuat( q ) ).s == B3_FIXED_MAX );
	ENSURE( b3InvMulQuat( q, q ).s == B3_FIXED_MAX );

	b3Vec3 r = b3MulAdd( top, B3_FIX( 1.0f ), one );
	ENSURE( r.x == B3_FIXED_MAX );
	r = b3MulSub( bottom, B3_FIX( 1.0f ), one );
	ENSURE( r.x == B3_FIXED_MIN );
	r = b3Lerp( top, top, B3_FIX( 0.5f ) );
	ENSURE( r.x == B3_FIXED_MAX );

	// Matrix times vector: two terms past the top, the third bringing the total back to 5.
	b3Matrix3 m = { { B3_FIXED_MAX, 0, 0 }, { B3_FIX( 5.0f ), 0, 0 }, { B3_FIXED_MIN, 0, 0 } };
	r = b3MulMV( m, one );
	ENSURE( r.x == B3_FIX( 5.0f ) );

	// A quarter turn of ( M, M, 0 ): the inner sum leaves the range and is held there.
	b3Fixed c = B3_FIX( 0.70710678f );
	b3Quat quarter = { { 0, 0, c }, c };
	b3Vec3 corner = { B3_FIXED_MAX, B3_FIXED_MAX, 0 };
	r = b3RotateVector( quarter, corner );
	ENSURE( r.x == 0 && r.y == B3_FIXED_MAX && r.z == 0 );

	// In range the policy changes nothing.
	ENSURE( b3Dot( one, one ) == B3_FIX( 3.0f ) );
	sum = b3Add( one, one );
	ENSURE( sum.x == B3_FIX( 2.0f ) );
	return 0;
}

int FixedSaturateTest( void )
{
	RUN_SUBTEST( SaturatingProducts );
	RUN_SUBTEST( SaturatingSums );
	return 0;
}
