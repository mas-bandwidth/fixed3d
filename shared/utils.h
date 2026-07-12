// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/math_functions.h"

#define RAND_LIMIT 32767
#define RAND_SEED 12345

// Global seed for simple random number generator.

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t g_randomSeed;

int GetNumberOfCores( void );

#ifdef __cplusplus
}
#endif

// Simple random number generator. Using this instead of rand() for cross platform determinism.
B3_INLINE int RandomInt()
{
	// XorShift32 algorithm
	uint32_t x = g_randomSeed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	g_randomSeed = x;

	// Map the 32-bit value to the range 0 to RAND_LIMIT
	return (int)( x % ( RAND_LIMIT + 1 ) );
}

// Random integer in range [lo, hi]
B3_INLINE b3Fixed RandomIntRange( int lo, int hi )
{
	return b3FixFromInt( lo + RandomInt() % ( hi - lo + 1 ) );
}

// Random number in range [-1,1]
B3_INLINE b3Fixed RandomFloat()
{
	b3Fixed r = (b3Fixed)b3FixFromInt( ( RandomInt() & ( RAND_LIMIT ) ) );
	r /= RAND_LIMIT;
	r = b3FixMul( B3_FIX( 2.0f ) , r ) - B3_FIX( 1.0f );
	return r;
}

// Random floating point number in range [lo, hi]
B3_INLINE b3Fixed RandomFloatRange( b3Fixed lo, b3Fixed hi )
{
	b3Fixed r = (b3Fixed)b3FixFromInt( ( RandomInt() & ( RAND_LIMIT ) ) );
	r /= RAND_LIMIT;
	r = b3FixMul( ( hi - lo ) , r ) + lo;
	return r;
}

// Random vector with coordinates in range [lo, hi]
B3_INLINE b3Vec3 RandomVec3( b3Vec3 lo, b3Vec3 hi )
{
	b3Vec3 v;
	v.x = RandomFloatRange( lo.x, hi.x );
	v.y = RandomFloatRange( lo.y, hi.y );
	v.z = RandomFloatRange( lo.z, hi.z );
	return v;
}

// Random world position with coordinates in range [lo, hi]
B3_INLINE b3Pos RandomPos( b3Vec3 lo, b3Vec3 hi )
{
	b3Pos v;
	v.x = RandomFloatRange( lo.x, hi.x );
	v.y = RandomFloatRange( lo.y, hi.y );
	v.z = RandomFloatRange( lo.z, hi.z );
	return v;
}

B3_INLINE b3Vec3 RandomVec3Uniform( b3Fixed lo, b3Fixed hi )
{
	b3Vec3 v;
	v.x = RandomFloatRange( lo, hi );
	v.y = RandomFloatRange( lo, hi );
	v.z = RandomFloatRange( lo, hi );
	return v;
}

B3_INLINE b3Vec3 RandomUnitVector()
{
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	b3Fixed u1 = RandomFloatRange( B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
	b3Fixed u2 = RandomFloatRange( B3_FIX( 0.0f ), b3FixMul( B3_FIX( 2.0f ) , B3_PI ) );
	b3Fixed u3 = RandomFloatRange( B3_FIX( 0.0f ), b3FixMul( B3_FIX( 2.0f ) , B3_PI ) );

	b3Fixed sqrt1MinusU1 = b3FixSqrt( B3_FIX( 1.0f ) - u1 );
	b3Fixed sqrtU1 = b3FixSqrt( u1 );

	b3Vec3 v;
	v.x = b3FixMul( sqrt1MinusU1 , b3Sin( u2 ) );
	v.y = b3FixMul( sqrt1MinusU1 , b3Cos( u2 ) );
	v.z = b3FixMul( sqrtU1 , b3Sin( u3 ) );

	return v;
}

B3_INLINE b3Quat RandomQuat()
{
	// Generate uniformly distributed random quaternion using Shoemake's method
	// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992

	b3Fixed u1 = RandomFloatRange( B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
	b3Fixed u2 = RandomFloatRange( B3_FIX( 0.0f ), b3FixMul( B3_FIX( 2.0f ) , B3_PI ) );
	b3Fixed u3 = RandomFloatRange( B3_FIX( 0.0f ), b3FixMul( B3_FIX( 2.0f ) , B3_PI ) );

	b3Fixed sqrt1MinusU1 = b3FixSqrt( B3_FIX( 1.0f ) - u1 );
	b3Fixed sqrtU1 = b3FixSqrt( u1 );

	b3Quat q;
	q.v.x = b3FixMul( sqrt1MinusU1 , b3Sin( u2 ) );
	q.v.y = b3FixMul( sqrt1MinusU1 , b3Cos( u2 ) );
	q.v.z = b3FixMul( sqrtU1 , b3Sin( u3 ) );
	q.s = b3FixMul( sqrtU1 , b3Cos( u3 ) );

	return q;
}
