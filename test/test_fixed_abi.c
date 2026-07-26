// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// ABI GUARD for the `fixed` compatibility seam.
//
// Staying drop-in compatible with Box3D is a design goal of this fork, so the public
// types are not ours to evolve. box3d/fixed_compat.h aliases box3d's b3Fixed, B3_FIX and
// the scalar arithmetic onto mas-bandwidth/fixed's fixed_t, FIX and fixMul; this file
// asserts the result is LAYOUT-IDENTICAL to what box3d had before the library was wired
// in. Every type below is built on b3Fixed, so a change to the scalar propagates to all
// of them -- which is exactly why the assertions cover the aggregates too.
//
// The numbers below are not aspirational. They were measured from the build immediately
// preceding the switch and pasted here. If a future change to `fixed` alters a struct --
// reorders a member, widens a scalar, adds padding -- every consumer's ABI breaks
// silently, because the code still compiles and still runs and merely disagrees about
// where `z` lives. A static assertion is the only thing that turns that into a build
// failure instead of a field report.
//
// If one of these fires, do not edit the number. The number is the contract.

#include "box3d/math_functions.h"
#include <stddef.h>
#include <stdint.h>

#if defined( __cplusplus )
	#define ABI_ASSERT( c, m ) static_assert( c, m )
#elif __STDC_VERSION__ >= 201112L
	#define ABI_ASSERT( c, m ) _Static_assert( c, m )
#else
	#define ABI_ASSERT( c, m ) typedef char abi_assert_##__LINE__[( c ) ? 1 : -1]
#endif

// ---- scalar ----
ABI_ASSERT( sizeof( b3Fixed ) == 8, "b3Fixed must stay 64-bit" );
ABI_ASSERT( _Alignof( b3Fixed ) == 8, "b3Fixed alignment" );
ABI_ASSERT( sizeof( b3Int128 ) == 16, "b3Int128 must stay 128-bit" );

// ---- vectors and rotation ----
ABI_ASSERT( sizeof( b3Vec2 ) == 16, "b3Vec2 size" );
ABI_ASSERT( sizeof( b3Vec3 ) == 24, "b3Vec3 size" );
ABI_ASSERT( offsetof( b3Vec3, x ) == 0, "b3Vec3.x offset" );
ABI_ASSERT( offsetof( b3Vec3, y ) == 8, "b3Vec3.y offset" );
ABI_ASSERT( offsetof( b3Vec3, z ) == 16, "b3Vec3.z offset" );

ABI_ASSERT( sizeof( b3Quat ) == 32, "b3Quat size" );
ABI_ASSERT( offsetof( b3Quat, v ) == 0, "b3Quat.v offset" );
ABI_ASSERT( offsetof( b3Quat, s ) == 24, "b3Quat.s offset" );

ABI_ASSERT( sizeof( b3Transform ) == 56, "b3Transform size" );
ABI_ASSERT( offsetof( b3Transform, p ) == 0, "b3Transform.p offset" );
ABI_ASSERT( offsetof( b3Transform, q ) == 24, "b3Transform.q offset" );

ABI_ASSERT( sizeof( b3Matrix3 ) == 72, "b3Matrix3 size" );
ABI_ASSERT( sizeof( b3Plane ) == 32, "b3Plane size" );

// ---- bounds ----
ABI_ASSERT( offsetof( b3AABB, lowerBound ) == 0, "b3AABB.lowerBound offset" );

#if defined( BOX3D_LUDICROUS_MODE )
// Wide build: 128-bit world coordinates. b3Pos is three int128, b3WorldTransform is that
// plus a narrow quaternion, and AABB bounds are wide positions. These are the sizes the
// determinism test's separate ludicrous goldens are computed against.
ABI_ASSERT( sizeof( b3Pos ) == 48, "b3Pos size (ludicrous)" );
ABI_ASSERT( sizeof( b3WorldTransform ) == 80, "b3WorldTransform size (ludicrous)" );
ABI_ASSERT( sizeof( b3AABB ) == 96, "b3AABB size (ludicrous)" );
ABI_ASSERT( offsetof( b3AABB, upperBound ) == 48, "b3AABB.upperBound offset (ludicrous)" );
#else
ABI_ASSERT( sizeof( b3Pos ) == 24, "b3Pos size (narrow)" );
ABI_ASSERT( sizeof( b3WorldTransform ) == 56, "b3WorldTransform size (narrow)" );
ABI_ASSERT( sizeof( b3AABB ) == 48, "b3AABB size (narrow)" );
ABI_ASSERT( offsetof( b3AABB, upperBound ) == 24, "b3AABB.upperBound offset (narrow)" );
ABI_ASSERT( sizeof( b3Pos ) == sizeof( b3Vec3 ), "narrow b3Pos aliases b3Vec3" );
#endif

// ---- the forwarders are real symbols, not macros ----
// The whole reason these are static inline rather than #define. If someone converts them
// back to macros to save a line, taking an address stops compiling and this fails loudly
// rather than the breakage surfacing in a consumer's debugger months later.
static b3Fixed ( *const fn_mul )( b3Fixed, b3Fixed ) = b3FixMul;
static b3Fixed ( *const fn_sqrt )( b3Fixed ) = b3FixSqrt;
static b3Fixed ( *const fn_div )( b3Fixed, b3Fixed ) = b3FixDiv;

int b3FixedAbiGuard( void );
int b3FixedAbiGuard( void )
{
	// Touch the pointers so no compiler decides the whole file is dead.
	b3Fixed one = B3_FIX( 1.0f );
	b3Fixed four = B3_FIX( 4.0f );
	return fn_mul( four, one ) == four && fn_sqrt( four ) == B3_FIX( 2.0f ) && fn_div( four, one ) == four;
}
