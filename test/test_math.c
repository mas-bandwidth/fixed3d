// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "math_internal.h"
#include "utils.h"
#include "test_macros.h"

#include <math.h>
#include <stdio.h>

// The minimax polynomial is accurate to 0.0023 degrees (4e-5 radians); add the
// Q48.16 output resolution (1.5e-5) on top.
#define ATAN_TOL B3_FIX( 0.0001f )

int MathTest( void )
{
	// Compare the fixed-point trig against double precision references from libm.
	// The fixed-point results carry the approximation error of the underlying
	// polynomial plus Q48.16 output quantization (about 1.5e-5).
	for ( double t = -10.0; t < 10.0; t += 0.01 )
	{
		double angle = 3.14159265358979323846 * t;
		b3Fixed fixedAngle = b3FixFromDouble( angle );
		double quantizedAngle = b3FixToDouble( fixedAngle );

		b3CosSin cs = b3ComputeCosSin( fixedAngle );
		double c = cos( quantizedAngle );
		double s = sin( quantizedAngle );

		// The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
		ENSURE_SMALL( cs.cosine - b3FixFromDouble( c ), B3_FIX( 0.002f ) );
		ENSURE_SMALL( cs.sine - b3FixFromDouble( s ), B3_FIX( 0.002f ) );

		b3Fixed xn = b3UnwindAngle( fixedAngle );
		b3Fixed a = b3Atan2( cs.sine, cs.cosine );
		ENSURE( b3IsValidFixed( a ) );

		// atan2 of the approximate cos/sin recovers the unwound angle to within
		// the cos/sin approximation error
		b3Fixed diff = b3FixAbs( a - xn );

		// The two results can be off by 360 degrees (-pi and pi)
		if ( diff > B3_PI )
		{
			diff -= 2 * B3_PI;
			diff = b3FixAbs( diff );
		}

		ENSURE_SMALL( diff, B3_FIX( 0.002f ) );
	}

	for ( double y = -1.0; y <= 1.0; y += 0.01 )
	{
		for ( double x = -1.0; x <= 1.0; x += 0.01 )
		{
			b3Fixed fy = b3FixFromDouble( y );
			b3Fixed fx = b3FixFromDouble( x );
			if ( fx == 0 && fy == 0 )
			{
				continue;
			}

			b3Fixed a1 = b3Atan2( fy, fx );
			b3Fixed a2 = b3FixFromDouble( atan2( b3FixToDouble( fy ), b3FixToDouble( fx ) ) );
			b3Fixed diff = b3FixAbs( a1 - a2 );
			ENSURE( b3IsValidFixed( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}
	}

	{
		b3Fixed a1 = b3Atan2( B3_FIX( 1.0f ), 0 );
		b3Fixed a2 = b3FixFromDouble( atan2( 1.0, 0.0 ) );
		ENSURE( b3IsValidFixed( a1 ) );
		ENSURE_SMALL( a1 - a2, ATAN_TOL );
	}

	{
		b3Fixed a1 = b3Atan2( -B3_FIX( 1.0f ), 0 );
		b3Fixed a2 = b3FixFromDouble( atan2( -1.0, 0.0 ) );
		ENSURE( b3IsValidFixed( a1 ) );
		ENSURE_SMALL( a1 - a2, ATAN_TOL );
	}

	{
		b3Fixed a1 = b3Atan2( 0, B3_FIX( 1.0f ) );
		b3Fixed a2 = b3FixFromDouble( atan2( 0.0, 1.0 ) );
		ENSURE( b3IsValidFixed( a1 ) );
		ENSURE_SMALL( a1 - a2, ATAN_TOL );
	}

	{
		b3Fixed a1 = b3Atan2( 0, -B3_FIX( 1.0f ) );
		b3Fixed a2 = b3FixFromDouble( atan2( 0.0, -1.0 ) );
		ENSURE( b3IsValidFixed( a1 ) );
		ENSURE_SMALL( a1 - a2, ATAN_TOL );
	}

	{
		// atan2(0, 0) == 0 by convention
		b3Fixed a1 = b3Atan2( 0, 0 );
		ENSURE( a1 == 0 );
	}

	b3Vec3 zero = { 0 };
	b3Vec3 one = { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) };
	b3Vec3 two = { B3_FIX( 2.0f ), B3_FIX( 2.0f ), B3_FIX( 2.0f ) };

	b3Vec3 v = b3Add( one, two );
	ENSURE( v.x == B3_FIX( 3.0f ) && v.y == B3_FIX( 3.0f ) );

	v = b3Sub( zero, two );
	ENSURE( v.x == -B3_FIX( 2.0f ) && v.y == -B3_FIX( 2.0f ) );

	v = b3Add( two, two );
	ENSURE( v.x != B3_FIX( 5.0f ) && v.y != B3_FIX( 5.0f ) );

	b3Vec3 axis = b3Normalize( (b3Vec3){ -B3_FIX( 0.75f ), B3_FIX( 0.5f ), B3_FIX( 1.0f ) } );
	b3Transform transform1 = { { -B3_FIX( 2.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
	b3Transform transform2 = { { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3MakeQuatFromAxisAngle( axis, B3_PI ) };

	b3Transform transform = b3MulTransforms( transform2, transform1 );

	v = b3TransformPoint( transform2, b3TransformPoint( transform1, two ) );

	b3Vec3 u = b3TransformPoint( transform, two );

	ENSURE_SMALL( u.x - v.x, 10 * B3_FIXED_EPSILON );
	ENSURE_SMALL( u.y - v.y, 10 * B3_FIXED_EPSILON );

	v = b3TransformPoint( transform1, two );
	v = b3InvTransformPoint( transform1, v );

	ENSURE_SMALL( v.x - two.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( v.y - two.y, 8 * B3_FIXED_EPSILON );

	b3Transform relTransform = b3InvMulTransforms( transform1, transform2 );
	v = b3InvTransformPoint( transform1, b3TransformPoint( transform2, two ) );
	u = b3TransformPoint( relTransform, two );
	ENSURE_SMALL( u.x - v.x, 10 * B3_FIXED_EPSILON );
	ENSURE_SMALL( u.y - v.y, 10 * B3_FIXED_EPSILON );

	{
		axis = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
		b3Quat q1 = b3MakeQuatFromAxisAngle( axis, b3FixMul( -B3_FIX( 0.5f ) , B3_PI ) );
		b3Quat q2 = b3ComputeQuatBetweenUnitVectors( (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );

		ENSURE_SMALL( q1.v.x - q2.v.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( q1.v.y - q2.v.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( q1.v.z - q2.v.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( q1.s - q2.s, 8 * B3_FIXED_EPSILON );

		b3Quat q3 = b3NormalizeQuat( (b3Quat){ { B3_FIX( 1.0f ), -B3_FIX( 2.0f ), B3_FIX( 3.0f ) }, B3_FIX( 4.0f ) } );
		b3Quat q4 = b3InvMulQuat( q3, q1 );
		b3Quat q5 = b3MulQuat( q3, q4 );
		ENSURE_SMALL( q1.v.x - q5.v.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( q1.v.y - q5.v.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( q1.v.z - q5.v.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( q1.s - q5.s, 8 * B3_FIXED_EPSILON );

		b3Quat q6 = b3ComputeQuatBetweenUnitVectors( (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) }, (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		ENSURE_SMALL( q6.s, 8 * B3_FIXED_EPSILON );
		(void)q6;
	}

	v = b3Normalize( (b3Vec3){ B3_FIX( 0.2f ), -B3_FIX( 0.5f ), B3_FIX( 3.0f ) } );
	for ( b3Fixed z = -B3_FIX( 1.0f ); z <= B3_FIX( 1.0f ); z += B3_FIX( 0.02f ) )
	{
		for ( b3Fixed y = -B3_FIX( 1.0f ); y <= B3_FIX( 1.0f ); y += B3_FIX( 0.02f ) )
		{
			for ( b3Fixed x = -B3_FIX( 1.0f ); x <= B3_FIX( 1.0f ); x += B3_FIX( 0.02f ) )
			{
				if ( x == B3_FIX( 0.0f ) && y == B3_FIX( 0.0f ) && z == B3_FIX( 0.0f ) )
				{
					continue;
				}

				u = b3Normalize( (b3Vec3){ x, y, z } );
				if ( b3IsNormalized( u ) == false )
				{
					// Vectors shorter than the fixed-point resolution cannot be normalized
					continue;
				}

				b3Quat r = b3ComputeQuatBetweenUnitVectors( v, u );
				ENSURE( b3IsValidQuat( r ) );

				b3Vec3 w = b3RotateVector( r, v );

				ENSURE_SMALL( b3Dot( r.v, b3Cross( u, w ) ) - b3ScalarTripleProduct( r.v, u, w ), 8 * B3_FIXED_EPSILON );

				// The quaternion between vectors has lots of round off error at large angles.
				// The rotation axis is derived from the half vector between u and v, whose
				// length shrinks toward zero at 180 degrees, amplifying fixed-point
				// quantization noise by one over the half-vector length.
				b3Fixed d = b3Dot( u, v );
				b3Fixed halfLen = b3FixSqrt( b3FixMax( b3FixMul( B3_FIX( 0.5f ), B3_FIX( 1.0f ) + d ), B3_FIXED_EPSILON ) );
				b3Fixed tol = B3_FIX( 0.001f ) + b3FixDiv( B3_FIX( 0.0008f ), halfLen );
				ENSURE_SMALL( w.x - u.x, tol );
				ENSURE_SMALL( w.y - u.y, tol );
				ENSURE_SMALL( w.z - u.z, tol );

				// Twist angle testing
				b3Fixed twist = r.s < B3_FIX( 0.0f ) ? b3Atan2( -r.v.z, -r.s ) : b3Atan2( r.v.z, r.s );
				twist = b3FixMul( twist, B3_FIX( 2.0f ) );
				ENSURE( -B3_PI - 2 * B3_FIXED_EPSILON <= twist && twist <= B3_PI + 2 * B3_FIXED_EPSILON );
			}
		}
	}

	{
		// More twist angle testing
		b3Quat q = { .v = { -B3_FIX( 0.0558656752f ), -B3_FIX( 0.188799798f ), B3_FIX( 0.00689807534f ) }, .s = -B3_FIX( 0.980401039f ) };
		b3Fixed twist = q.s < B3_FIX( 0.0f ) ? b3Atan2( -q.v.z, -q.s ) : b3Atan2( q.v.z, q.s );
		twist = b3FixMul( twist, B3_FIX( 2.0f ) );
		ENSURE( -B3_PI - 2 * B3_FIXED_EPSILON <= twist && twist <= B3_PI + 2 * B3_FIXED_EPSILON );
	}

	{
		b3Matrix3 m = { { B3_FIX( 3.0f ), B3_FIX( 1.0f ), -B3_FIX( 1.0f ) }, { -B3_FIX( 1.0f ), B3_FIX( 3.0f ), B3_FIX( 1.0f ) }, { B3_FIX( 1.0f ), -B3_FIX( 1.0f ), B3_FIX( 3.0f ) } };
		b3Matrix3 invM = b3InvertMatrix( m );
		b3Matrix3 a = b3MulMM( m, invM );
		ENSURE_SMALL( a.cx.x - B3_FIX( 1.0f ), 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cx.y, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cx.z, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cy.x, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cy.y - B3_FIX( 1.0f ), 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cy.z, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cz.x, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cz.y, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cz.z - B3_FIX( 1.0f ), 32 * B3_FIXED_EPSILON );

		v = (b3Vec3){ B3_FIX( 1.0f ), -B3_FIX( 2.0f ), B3_FIX( 3.0f ) };
		u = b3MulMV( invM, b3MulMV( m, v ) );
		ENSURE_SMALL( v.x - u.x, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( v.y - u.y, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( v.z - u.z, 32 * B3_FIXED_EPSILON );

		b3Vec3 w = b3MulMV( invM, v );
		u = b3Solve3( m, v );
		ENSURE_SMALL( w.x - u.x, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( w.y - u.y, 32 * B3_FIXED_EPSILON );
		ENSURE_SMALL( w.z - u.z, 32 * B3_FIXED_EPSILON );
	}

	{
		b3Matrix2 m = { { B3_FIX( 3.0f ), B3_FIX( 1.0f ) }, { -B3_FIX( 1.0f ), B3_FIX( 3.0f ) } };
		b3Matrix2 invM = b3Invert2( m );
		b3Matrix2 a = b3MulMM2( m, invM );
		ENSURE_SMALL( a.cx.x - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cx.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cy.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( a.cy.y - B3_FIX( 1.0f ), 8 * B3_FIXED_EPSILON );

		b3Vec2 v2 = { B3_FIX( 1.0f ), -B3_FIX( 2.0f ) };
		b3Vec2 u2 = b3MulMV2( invM, b3MulMV2( m, v2 ) );
		ENSURE_SMALL( v2.x - u2.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( v2.y - u2.y, 8 * B3_FIXED_EPSILON );

		b3Vec2 w = b3MulMV2( invM, v2 );
		u2 = b3Solve2( m, v2 );
		ENSURE_SMALL( w.x - u2.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( w.y - u2.y, 8 * B3_FIXED_EPSILON );

		w = b3MulMV2( m, u2 );
		ENSURE_SMALL( w.x - v2.x, 10 * B3_FIXED_EPSILON );
		ENSURE_SMALL( w.y - v2.y, 10 * B3_FIXED_EPSILON );
	}

	for (int i = 0; i < 100; ++i)
	{
		b3Fixed a = RandomFloat();
		double b = b3FixToDouble( a );
		b3Fixed c = (b3Fixed)b3FixFromDouble( b );
		ENSURE( c == a );
	}

	b3Quat q1 = b3Quat_identity;
	b3Quat q2 = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, b3FixMul( B3_FIX( 0.5f ) , B3_PI ) );
	int n = 100;
	for ( int i = 0; i <= n; ++i )
	{
		b3Fixed alpha = b3FixDiv( (b3Fixed)b3FixFromInt( i ) , (b3Fixed)b3FixFromInt( n ) );
		b3Quat q = b3NLerp( q1, q2, alpha );
		b3Fixed angle = b3GetTwistAngle( q );
		ENSURE_SMALL( b3FixMul( b3FixMul( alpha, B3_FIX( 0.5f ) ), B3_PI ) - angle, B3_DEG_TO_RAD );
		//printf( "angle = [%g %g %g]\n", alpha, alpha * 0.5f * B3_PI, angle );
	}

	{
		b3Vec3 normal = { B3_FIX( 0.504055440f ), B3_FIX( 0.621548057f ), B3_FIX( 0.599671543f ) };
		b3Vec3 perp = b3ArbitraryPerp( normal );
		ENSURE_SMALL( b3Dot( normal, perp ), 2 * B3_FIXED_EPSILON );
	}

	{
		// World position boundary helpers. The query agrees with the built type sizes.
		ENSURE( b3IsDoublePrecision() == ( sizeof( b3Pos ) > sizeof( b3Vec3 ) ) );

		// Deltas and offsets round trip exactly for representable inputs in both modes.
		b3Vec3 a = { B3_FIX( 3.0f ), -B3_FIX( 5.0f ), B3_FIX( 2.0f ) };
		b3Vec3 b = { B3_FIX( 1.0f ), B3_FIX( 4.0f ), -B3_FIX( 6.0f ) };
		b3Pos pa = b3ToPos( a );
		b3Pos pb = b3ToPos( b );

		b3Vec3 d = b3SubPos( pa, pb );
		b3Vec3 sub = b3Sub( a, b );
		ENSURE( d.x == sub.x && d.y == sub.y && d.z == sub.z );

		b3Vec3 back = b3SubPos( b3OffsetPos( pb, sub ), pa );
		ENSURE( back.x == B3_FIX( 0.0f ) && back.y == B3_FIX( 0.0f ) && back.z == B3_FIX( 0.0f ) );

		b3Vec3 r = b3ToVec3( pa );
		ENSURE( r.x == a.x && r.y == a.y && r.z == a.z );

		ENSURE( b3IsValidPosition( pa ) );

		// World transform relative ops match the pure b3Fixed transform ops. Float mode is
		// bit identical, double mode keeps the relative result in b3Fixed.
		b3Vec3 axis = b3Normalize( (b3Vec3){ B3_FIX( 0.3f ), -B3_FIX( 0.7f ), B3_FIX( 0.5f ) } );
		b3Transform tA = { a, b3MakeQuatFromAxisAngle( axis, B3_FIX( 0.4f ) ) };
		b3Transform tB = { b, b3MakeQuatFromAxisAngle( axis, -B3_FIX( 1.1f ) ) };
		b3WorldTransform wA = b3MakeWorldTransform( tA );
		b3WorldTransform wB = b3MakeWorldTransform( tB );
		ENSURE( b3IsValidWorldTransform( wA ) );

		b3Transform relRef = b3InvMulTransforms( tA, tB );
		b3Transform rel = b3InvMulWorldTransforms( wA, wB );
		ENSURE_SMALL( rel.p.x - relRef.p.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( rel.p.y - relRef.p.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( rel.p.z - relRef.p.z, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( rel.q.s - relRef.q.s, 8 * B3_FIXED_EPSILON );

		// Local point to world and back.
		b3Vec3 local = { B3_FIX( 0.5f ), -B3_FIX( 0.25f ), B3_FIX( 1.5f ) };
		b3Vec3 back2 = b3InvTransformWorldPoint( wA, b3TransformWorldPoint( wA, local ) );
		ENSURE_SMALL( back2.x - local.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( back2.y - local.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( back2.z - local.z, 8 * B3_FIXED_EPSILON );

		// Compose with a local transform, then strip it back off.
		b3Transform relAB = b3InvMulWorldTransforms( wA, b3MulWorldTransforms( wA, tB ) );
		ENSURE_SMALL( relAB.p.x - tB.p.x, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( relAB.p.y - tB.p.y, 8 * B3_FIXED_EPSILON );
		ENSURE_SMALL( relAB.p.z - tB.p.z, 8 * B3_FIXED_EPSILON );
	}

#if defined( BOX3D_DOUBLE_PRECISION )
	{
		// Far from the origin the double layer keeps the relative result accurate where pure
		// b3Fixed would quantize. Two poses one meter apart at x = 1e8.
		b3Pos base = { 1.0e8, 0.0, 0.0 };
		b3WorldTransform wA = { base, b3Quat_identity };
		b3WorldTransform wB = { b3OffsetPos( base, (b3Vec3){ 1.0f, 0.0f, 0.0f } ), b3Quat_identity };
		b3Transform rel = b3InvMulWorldTransforms( wA, wB );
		ENSURE( rel.p.x == 1.0f && rel.p.y == 0.0f && rel.p.z == 0.0f );
	}
#endif

	return 0;
}
