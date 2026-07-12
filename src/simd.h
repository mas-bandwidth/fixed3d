// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stdbool.h>


// I don't expect the use case of b3V32 to benefit from Neon code.
// In particular the cross product is very complex in Neon.

// scalar math
typedef struct b3V32
{
	b3Fixed x, y, z;
} b3V32;

typedef union b3128
{
	b3V32 v;
	b3Fixed f[3];
} b3128;

static const b3V32 b3_zeroV = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
static const b3V32 b3_halfV = { B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) };
static const b3V32 b3_oneV = { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) };

static inline b3V32 b3AddV( b3V32 a, b3V32 b )
{
	return B3_LITERAL( b3V32 ){
		a.x + b.x,
		a.y + b.y,
		a.z + b.z,
	};
}

static inline b3V32 b3SubV( b3V32 a, b3V32 b )
{
	return B3_LITERAL( b3V32 ){
		a.x - b.x,
		a.y - b.y,
		a.z - b.z,
	};
}

static inline b3V32 b3MulV( b3V32 a, b3V32 b )
{
	return B3_LITERAL( b3V32 ){
		b3FixMul( a.x , b.x ),
		b3FixMul( a.y , b.y ),
		b3FixMul( a.z , b.z ),
	};
}

static inline b3V32 b3DivV( b3V32 a, b3V32 b )
{
	return B3_LITERAL( b3V32 ){
		b3FixDiv( a.x , b.x ),
		b3FixDiv( a.y , b.y ),
		b3FixDiv( a.z , b.z ),
	};
}

static inline b3V32 b3NegV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){
		-a.x,
		-a.y,
		-a.z,
	};
}

// Unaligned loads are much faster on recent hardware with little to no penalty
static inline b3V32 b3LoadV( const b3Fixed* src )
{
	return B3_LITERAL( b3V32 ){ src[0], src[1], src[2] };
}

static inline b3V32 b3ZeroV( void )
{
	return B3_LITERAL( b3V32 ){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
}

static inline b3Fixed b3GetXV( b3V32 a )
{
	return a.x;
}

static inline b3Fixed b3GetYV( b3V32 a )
{
	return a.y;
}

static inline b3Fixed b3GetZV( b3V32 a )
{
	return a.z;
}

static inline b3Fixed b3GetV( b3V32 a, int index )
{
	b3128 b;
	b.v = a;
	return b.f[index];
}

static inline b3V32 b3SplatV( b3Fixed x )
{
	return B3_LITERAL( b3V32 ){ x, x, x };
}

static inline b3V32 b3AbsV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){
		a.x < B3_FIX( 0.0f ) ? -a.x : a.x,
		a.y < B3_FIX( 0.0f ) ? -a.y : a.y,
		a.z < B3_FIX( 0.0f ) ? -a.z : a.z,
	};
}

static inline b3V32 b3MinV( b3V32 a, b3V32 b )
{
	return B3_LITERAL( b3V32 ){
		a.x < b.x ? a.x : b.x,
		a.y < b.y ? a.y : b.y,
		a.z < b.z ? a.z : b.z,
	};
}

static inline b3V32 b3MaxV( b3V32 a, b3V32 b )
{
	return B3_LITERAL( b3V32 ){
		a.x > b.x ? a.x : b.x,
		a.y > b.y ? a.y : b.y,
		a.z > b.z ? a.z : b.z,
	};
}

static inline b3V32 b3CrossV( b3V32 a, b3V32 b )
{
	b3V32 c;
	c.x = b3FixMul( a.y , b.z ) - b3FixMul( a.z , b.y );
	c.y = b3FixMul( a.z , b.x ) - b3FixMul( a.x , b.z );
	c.z = b3FixMul( a.x , b.y ) - b3FixMul( a.y , b.x );
	return c;
}

static inline b3V32 b3ModifiedCrossV( b3V32 a, b3V32 b )
{
	b3V32 c;
	c.x = b3FixMul( a.y , b.z ) + b3FixMul( a.z , b.y );
	c.y = b3FixMul( a.z , b.x ) + b3FixMul( a.x , b.z );
	c.z = b3FixMul( a.x , b.y ) + b3FixMul( a.y , b.x );
	return c;
}

static inline bool b3AnyLess3V( b3V32 a, b3V32 b )
{
	return a.x < b.x || a.y < b.y || a.z < b.z;
}

static inline bool b3AnyLessEq3V( b3V32 a, b3V32 b )
{
	return a.x <= b.x || a.y <= b.y || a.z <= b.z;
}

static inline bool b3AnyGreater3V( b3V32 a, b3V32 b )
{
	return a.x > b.x || a.y > b.y || a.z > b.z;
}

static inline bool b3AllLessEq3V( b3V32 a, b3V32 b )
{
	return a.x <= b.x && a.y <= b.y && a.z <= b.z;
}


static inline bool b3TestBoundsOverlap( b3V32 nodeMin1, b3V32 nodeMax1, b3V32 nodeMin2, b3V32 nodeMax2 )
{
	b3V32 separation = b3MaxV( b3SubV( nodeMin2, nodeMax1 ), b3SubV( nodeMin1, nodeMax2 ) );
	return b3AllLessEq3V( separation, b3_zeroV );
}

// Test a ray for edge separation with an AABB (Gino, p80).
static inline bool b3TestBoundsRayOverlap( b3V32 nodeMin, b3V32 nodeMax, b3V32 rayStart, b3V32 rayDelta )
{
	// Setup node
	b3V32 nodeCenter = b3MulV( b3_halfV, b3AddV( nodeMin, nodeMax ) );
	b3V32 nodeExtent = b3SubV( nodeMax, nodeCenter );

	// Setup ray
	rayStart = b3SubV( rayStart, nodeCenter );

	// SAT: Edge separation
	b3V32 edgeSeparation = b3SubV( b3AbsV( b3CrossV( rayDelta, rayStart ) ), b3ModifiedCrossV( b3AbsV( rayDelta ), nodeExtent ) );
	return b3AllLessEq3V( edgeSeparation, b3_zeroV );
}

bool b3TestBoundsTriangleOverlap( b3V32 nodeCenter, b3V32 nodeExtent, b3V32 vertex1, b3V32 vertex2, b3V32 vertex3 );
b3Fixed b3IntersectRayTriangle( b3V32 rayStart, b3V32 rayDelta, b3V32 vertex1, b3V32 vertex2, b3V32 vertex3 );
