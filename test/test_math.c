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

// ---------------------------------------------------------------------------------------------
// The game conversion roster: the exp2/log2/pow ladder, Q2.30 quaternions, critically damped
// smoothing, random quaternions. Every one of these is a name the game asks box3d for, so the
// tests measure them through box3d's spelling even where the arithmetic lives in `fixed`.
// ---------------------------------------------------------------------------------------------

static int Exp2Log2PowTest( void )
{
	// exact powers round-trip exactly
	ENSURE( b3FixLog2( B3_FIXED_ONE ) == 0 );
	ENSURE( b3FixLog2( B3_FIX( 2.0f ) ) == B3_FIXED_ONE );
	ENSURE( b3FixLog2( B3_FIX( 0.5f ) ) == -B3_FIXED_ONE );
	ENSURE( b3FixExp2( 0 ) == B3_FIXED_ONE );
	ENSURE( b3FixExp2( B3_FIXED_ONE ) == B3_FIX( 2.0f ) );
	ENSURE( b3FixExp2( -B3_FIXED_ONE ) == B3_FIX( 0.5f ) );

	// domain edges
	ENSURE( b3FixLog2( 0 ) == INT64_MIN );
	ENSURE( b3FixLog2( -B3_FIXED_ONE ) == INT64_MIN );
	ENSURE( b3FixPow( 0, B3_FIXED_ONE ) == 0 );
	ENSURE( b3FixPow( -B3_FIXED_ONE, B3_FIXED_ONE ) == 0 );
	ENSURE( b3FixPow( B3_FIX( 1.5f ), 0 ) == B3_FIXED_ONE );
	ENSURE( b3FixExp2( B3_FIX( 47.0f ) + 1 ) == INT64_MAX );
	ENSURE( b3FixExp2( B3_FIX( -18.0f ) ) == 0 );

	// log2 against libm over several magnitudes
	for ( double t = -14.0; t <= 14.0; t += 0.03125 )
	{
		double value = pow( 2.0, t );
		b3Fixed fixedValue = b3FixFromDouble( value );
		if ( fixedValue <= 0 )
		{
			continue;
		}
		double quantized = b3FixToDouble( fixedValue );
		double expected = log2( quantized );
		double got = b3FixToDouble( b3FixLog2( fixedValue ) );
		// exact binary digits rounded to Q48.16: within one output ulp plus input quantization
		ENSURE( fabs( got - expected ) < 3.1e-5 );
	}

	// exp2 against libm
	for ( double t = -16.0; t <= 16.0; t += 0.015625 )
	{
		b3Fixed fixedT = b3FixFromDouble( t );
		double quantized = b3FixToDouble( fixedT );
		double expected = pow( 2.0, quantized );
		double got = b3FixToDouble( b3FixExp2( fixedT ) );
		// relative error: table product carries ~16 rounded Q32 multiplies
		double scale = expected > 1.0 ? expected : 1.0;
		ENSURE( fabs( got - expected ) < 3.1e-5 * scale + 3.1e-5 );
	}

	// pow over the CONTROL DOMAIN ( base in [2^-14, 2], exponent in [1/4, 4] ), against double
	// pow. The honest bound is COMPOSITE: Q48.16 output quantization dominates small results
	// (one output ulp is 1.5e-5) and the ladder's relative error dominates large ones, so each
	// point must satisfy error <= 2 ulps + 1e-4 * expected. Both components are reported; the
	// documented claim in math_functions.h mirrors this bound.
	double ulp = 1.0 / (double)B3_FIXED_ONE;
	double worstRelativeLarge = 0.0; // over results >= 0.5, where quantization noise < 3.1e-5
	double worstMargin = 0.0;		 // error / ( 2 ulps + 1e-4 * expected ), must stay < 1
	for ( double b = 0.00006103515625; b <= 2.0; b *= 1.0442737824274138 ) // 2^(1/16) steps
	{
		for ( double e = 0.25; e <= 4.0; e += 0.0625 )
		{
			b3Fixed fixedBase = b3FixFromDouble( b );
			b3Fixed fixedExponent = b3FixFromDouble( e );
			if ( fixedBase <= 0 )
			{
				continue;
			}
			double expected = pow( b3FixToDouble( fixedBase ), b3FixToDouble( fixedExponent ) );
			double got = b3FixToDouble( b3FixPow( fixedBase, fixedExponent ) );
			double error = fabs( got - expected );

			double margin = error / ( 2.0 * ulp + 1.0e-4 * expected );
			if ( margin > worstMargin )
			{
				worstMargin = margin;
			}

			if ( expected >= 0.5 )
			{
				double relative = error / expected;
				if ( relative > worstRelativeLarge )
				{
					worstRelativeLarge = relative;
				}
			}
		}
	}
	printf( "  pow over the control domain: worst %.3g relative (results >= 0.5), worst composite margin %.2f of bound\n",
			worstRelativeLarge, worstMargin );
	ENSURE( worstRelativeLarge < 1.0e-4 );
	ENSURE( worstMargin < 1.0 );

	return 0;
}

static int Quat30Test( void )
{
	// pack/unpack round trip: packing gains precision, so Q48.16 -> Q2.30 -> Q48.16 is exact
	for ( double t = -1.0; t <= 1.0; t += 0.001953125 )
	{
		b3Fixed a = b3FixFromDouble( t );
		b3Fixed back = b3FixFromFix30( b3Fix30FromFix( a ) );
		ENSURE( back == a );
	}

	// the pinned rounding rule on unpack: raw 2^13 (half of the drop) rounds up to 1 Q48.16 ulp
	{
		b3Fixed30 half = { (int32_t)1 << 13 };
		ENSURE( b3FixFromFix30( half ) == 1 );
		b3Fixed30 justUnder = { ( (int32_t)1 << 13 ) - 1 };
		ENSURE( b3FixFromFix30( justUnder ) == 0 );
	}

	// pack saturates outside [-2, 2)
	{
		b3Fixed30 top = b3Fix30FromFix( B3_FIX( 3.0f ) );
		ENSURE( top.raw == INT32_MAX );
		b3Fixed30 bottom = b3Fix30FromFix( B3_FIX( -3.0f ) );
		ENSURE( bottom.raw == INT32_MIN );
	}

	// double converters: Q2.30 resolution round trips through double exactly
	{
		b3Fixed30 v = b3Fix30FromDouble( 0.333333333333 );
		double back = b3Fix30ToDouble( v );
		ENSURE( fabs( back - 0.333333333333 ) < 1.0e-9 );
		b3Fixed30 again = b3Fix30FromDouble( back );
		ENSURE( again.raw == v.raw );
	}

	// normalize IN the Q2.30 domain: unit result at 30-bit precision, components clamped
	{
		b3Quat30 q;
		q.x = b3Fix30FromDouble( 0.3 );
		q.y = b3Fix30FromDouble( -0.5 );
		q.z = b3Fix30FromDouble( 0.7 );
		q.w = b3Fix30FromDouble( 0.4 );
		b3Quat30 n = b3NormalizeQuat30( q );
		ENSURE( b3IsNormalizedQuat30( n ) );

		double nx = b3Fix30ToDouble( n.x );
		double ny = b3Fix30ToDouble( n.y );
		double nz = b3Fix30ToDouble( n.z );
		double nw = b3Fix30ToDouble( n.w );
		double length = sqrt( nx * nx + ny * ny + nz * nz + nw * nw );
		ENSURE( fabs( length - 1.0 ) < 1.0e-8 );

		// direction preserved
		double scale = b3Fix30ToDouble( q.x ) / nx;
		ENSURE( fabs( b3Fix30ToDouble( q.y ) / ny - scale ) < 1.0e-6 );

		// components never exceed one
		ENSURE( n.x.raw <= B3_FIXED30_ONE && n.x.raw >= -B3_FIXED30_ONE );
		ENSURE( n.w.raw <= B3_FIXED30_ONE && n.w.raw >= -B3_FIXED30_ONE );
	}

	// an axis-aligned unit stays exactly unit, and degenerate becomes identity
	{
		b3Quat30 axis = { { 0 }, { 0 }, { 0 }, { B3_FIXED30_ONE } };
		b3Quat30 n = b3NormalizeQuat30( axis );
		ENSURE( n.w.raw == B3_FIXED30_ONE && n.x.raw == 0 );

		b3Quat30 zero = { { 0 }, { 0 }, { 0 }, { 0 } };
		b3Quat30 identity = b3NormalizeQuat30( zero );
		ENSURE( identity.w.raw == B3_FIXED30_ONE );
	}

	// The four-INT32_MIN corner: the squared length is exactly 2^64, one bit past a uint64,
	// and it is the ONLY input that reaches there. A single-word accumulator wraps it to
	// zero and the quaternion is then read as degenerate, so it comes back as IDENTITY --
	// which is normalized, and which "is the result a unit quaternion?" therefore cannot
	// tell apart from the right answer. So pin the VALUE: four equal components normalize
	// to four equal components, each -1/2, and identity is not that.
	{
		b3Quat30 extreme = { { INT32_MIN }, { INT32_MIN }, { INT32_MIN }, { INT32_MIN } };
		ENSURE( b3IsNormalizedQuat30( extreme ) == false );

		b3Quat30 n = b3NormalizeQuat30( extreme );
		ENSURE( b3IsNormalizedQuat30( n ) );
		ENSURE( n.x.raw == -( B3_FIXED30_ONE / 2 ) );
		ENSURE( n.y.raw == -( B3_FIXED30_ONE / 2 ) );
		ENSURE( n.z.raw == -( B3_FIXED30_ONE / 2 ) );
		ENSURE( n.w.raw == -( B3_FIXED30_ONE / 2 ) );
	}

	// Q48.16 quaternion pack path
	{
		b3Quat q = b3MakeQuatFromAxisAngle( ( b3Vec3 ){ 0, B3_FIXED_ONE, 0 }, B3_FIX( 0.7f ) );
		b3Quat30 packed = b3Quat30FromQuat( q );
		b3Quat back = b3QuatFromQuat30( packed );
		ENSURE( back.v.x == q.v.x && back.v.y == q.v.y && back.v.z == q.v.z && back.s == q.s );
	}

	return 0;
}

static int SmoothingTest( void )
{
	// the fixed smoothing must track the double-precision formula closely over a control-style
	// trajectory (stick magnitudes, sub-second smooth times, 60 Hz steps)
	double current = 0.0, velocity = 0.0;
	b3Fixed fixedCurrent = 0, fixedVelocity = 0;

	b3Fixed fixedSmoothTime = b3FixFromDouble( 0.25 );
	b3Fixed fixedDeltaTime = b3FixFromDouble( 1.0 / 60.0 );
	double smoothTime = 0.25;
	double deltaTime = b3FixToDouble( fixedDeltaTime ); // compare against the same quantized dt

	for ( int i = 0; i < 240; ++i )
	{
		double target = ( i < 120 ) ? 1.0 : -0.5;
		b3Fixed fixedTarget = b3FixFromDouble( target );

		// the double reference uses the SAME pi constant as B3_PI quantizes from
		double w = 2.0 * (double)B3_PI / (double)B3_FIXED_ONE / smoothTime;
		velocity = ( velocity - w * w * deltaTime * ( current - target ) ) /
				   ( ( 1.0 + w * deltaTime ) * ( 1.0 + w * deltaTime ) );
		current = current + velocity * deltaTime;

		fixedCurrent =
			b3SmoothCriticallyDamped( fixedCurrent, fixedTarget, &fixedVelocity, fixedSmoothTime, fixedDeltaTime );

		ENSURE( fabs( b3FixToDouble( fixedCurrent ) - current ) < 0.002 );
	}

	// converged
	ENSURE( fabs( b3FixToDouble( fixedCurrent ) - ( -0.5 ) ) < 0.01 );

	// zero smooth time snaps to target; zero dt holds
	b3Fixed v0 = 0;
	ENSURE( b3SmoothCriticallyDamped( 0, B3_FIXED_ONE, &v0, 0, fixedDeltaTime ) == B3_FIXED_ONE );
	ENSURE( b3SmoothCriticallyDamped( 0, B3_FIXED_ONE, &v0, fixedSmoothTime, 0 ) == 0 );

	// up/down selection: moving toward larger magnitude uses the up time
	{
		b3Fixed vUp = 0, vRef = 0;
		b3Fixed up = b3SmoothCriticallyDampedUpDown( 0, B3_FIXED_ONE, &vUp, fixedSmoothTime, b3FixFromDouble( 1.0 ),
													 fixedDeltaTime );
		b3Fixed reference = b3SmoothCriticallyDamped( 0, B3_FIXED_ONE, &vRef, fixedSmoothTime, fixedDeltaTime );
		ENSURE( up == reference );
	}

	// vector variant matches the scalar componentwise
	{
		b3Vec3 c = { 0, 0, 0 };
		b3Vec3 t = { B3_FIX( 0.5f ), -B3_FIX( 0.25f ), 0 };
		b3Vec3 v = { 0, 0, 0 };
		b3Fixed sx = 0, sy = 0;
		b3Fixed vx = 0, vy = 0;
		for ( int i = 0; i < 60; ++i )
		{
			c = b3SmoothCriticallyDampedVec3( c, t, &v, fixedSmoothTime, fixedDeltaTime );
			sx = b3SmoothCriticallyDamped( sx, t.x, &vx, fixedSmoothTime, fixedDeltaTime );
			sy = b3SmoothCriticallyDamped( sy, t.y, &vy, fixedSmoothTime, fixedDeltaTime );
		}
		ENSURE( c.x == sx && c.y == sy && c.z == 0 );
	}

	// the vector up/down form selects ONE time by length, not per component: a target
	// longer than the current vector takes the up time for every component at once
	{
		b3Vec3 c = { 0, 0, 0 };
		b3Vec3 t = { B3_FIX( 0.5f ), -B3_FIX( 0.25f ), 0 };
		b3Vec3 v = { 0, 0, 0 };
		b3Vec3 cRef = { 0, 0, 0 };
		b3Vec3 vRef = { 0, 0, 0 };
		b3Vec3 got = b3SmoothCriticallyDampedUpDownVec3( c, t, &v, fixedSmoothTime, b3FixFromDouble( 1.0 ),
														 fixedDeltaTime );
		b3Vec3 expected = b3SmoothCriticallyDampedVec3( cRef, t, &vRef, fixedSmoothTime, fixedDeltaTime );
		ENSURE( got.x == expected.x && got.y == expected.y && got.z == expected.z );
	}

	return 0;
}

// a small deterministic xorshift for the random-quat test
static b3Fixed RandomFixedSource( void* context )
{
	uint64_t* state = (uint64_t*)context;
	uint64_t x = *state;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	*state = x;
	// uniform in [-1, 1] at Q48.16: 17 bits of entropy mapped onto [-65536, 65536]
	return (b3Fixed)( (int64_t)( x % ( 2 * 65536 + 1 ) ) - 65536 );
}

static int RandomQuatTest( void )
{
	uint64_t state = 0x123456789ABCDEFull;

	for ( int i = 0; i < 1000; ++i )
	{
		b3Quat q = b3MakeRandomQuat( RandomFixedSource, &state );
		ENSURE( b3IsNormalizedQuat( q ) );
	}

	// deterministic: same seed, same sequence
	uint64_t stateA = 42, stateB = 42;
	b3Quat a = b3MakeRandomQuat( RandomFixedSource, &stateA );
	b3Quat b = b3MakeRandomQuat( RandomFixedSource, &stateB );
	ENSURE( a.v.x == b.v.x && a.v.y == b.v.y && a.v.z == b.v.z && a.s == b.s );

	return 0;
}

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

	// the game conversion roster
	RUN_SUBTEST( Exp2Log2PowTest );
	RUN_SUBTEST( Quat30Test );
	RUN_SUBTEST( SmoothingTest );
	RUN_SUBTEST( RandomQuatTest );

	return 0;
}
