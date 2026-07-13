// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Dirk Gregorius contributed portions of this code

#include "simd.h"

// The float SSE2 variant of these helpers was removed with the rest of the
// float SIMD: simd.h only defines the scalar fixed-point b3V32 now, so the
// intrinsic branch could never compile again.

#define B3_TRANSPOSE3( C1, C2, C3 )                                                                                              \
	{                                                                                                                            \
		b3Fixed temp1 = C1.y;                                                                                                      \
		b3Fixed temp2 = C1.z;                                                                                                      \
		b3Fixed temp3 = C2.z;                                                                                                      \
                                                                                                                                 \
		C1.y = C2.x;                                                                                                             \
		C1.z = C3.x;                                                                                                             \
		C2.z = C3.y;                                                                                                             \
                                                                                                                                 \
		C2.x = temp1;                                                                                                            \
		C3.x = temp2;                                                                                                            \
		C3.y = temp3;                                                                                                            \
	}

static inline b3V32 b3SplatXV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){ a.x, a.x, a.x };
}

static inline b3V32 b3SplatYV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){ a.y, a.y, a.y };
}

static inline b3V32 b3SplatZV( b3V32 a )
{
	return B3_LITERAL( b3V32 ){ a.z, a.z, a.z };
}

static inline bool b3AnyGreaterEq3V( b3V32 a, b3V32 b )
{
	return a.x >= b.x || a.y >= b.y || a.z >= b.z;
}

static inline b3V32 b3Dot3V( b3V32 a, b3V32 b )
{
	b3Fixed d = b3FixMul( a.x , b.x ) + b3FixMul( a.y , b.y ) + b3FixMul( a.z , b.z );
	return B3_LITERAL( b3V32 ){ d, d, d };
}

bool b3TestBoundsTriangleOverlap( b3V32 nodeCenter, b3V32 nodeExtent, b3V32 vertex1, b3V32 vertex2, b3V32 vertex3 )
{
	b3V32 two = b3SplatV( B3_FIX( 2.0f ) );

	// Setup triangle
	vertex1 = b3SubV( vertex1, nodeCenter );
	vertex2 = b3SubV( vertex2, nodeCenter );
	vertex3 = b3SubV( vertex3, nodeCenter );

	// Face separation
	b3V32 triangleMin = b3MinV( vertex1, b3MinV( vertex2, vertex3 ) );
	b3V32 triangleMax = b3MaxV( vertex1, b3MaxV( vertex2, vertex3 ) );

	b3V32 separation1 = b3SubV( triangleMin, nodeExtent );
	b3V32 separation2 = b3AddV( triangleMax, nodeExtent );

	b3V32 faceSeparation = b3MaxV( separation1, b3NegV( separation2 ) );
	if ( b3AnyGreater3V( faceSeparation, b3_zeroV ) )
	{
		return false;
	}

	// SAT: Face separation
	b3V32 edge1 = b3SubV( vertex2, vertex1 );
	b3V32 edge2 = b3SubV( vertex3, vertex2 );
	b3V32 edge3 = b3SubV( vertex1, vertex3 );

	b3V32 normal = b3CrossV( edge1, edge2 );

	b3V32 triangleSeparation = b3SubV( b3AbsV( b3Dot3V( normal, vertex1 ) ), b3Dot3V( b3AbsV( normal ), nodeExtent ) );
	if ( b3AnyGreater3V( triangleSeparation, b3_zeroV ) )
	{
		return false;
	}

	// SAT: Edge separation
	b3V32 edgeSeparation1 = b3SubV( b3SubV( b3AbsV( b3CrossV( edge1, b3AddV( vertex1, vertex3 ) ) ), b3AbsV( b3CrossV( edge1, edge3 ) ) ),
									b3MulV( two, b3ModifiedCrossV( b3AbsV( edge1 ), nodeExtent ) ) );
	if ( b3AnyGreater3V( edgeSeparation1, b3_zeroV ) )
	{
		return false;
	}

	b3V32 edgeSeparation2 = b3SubV( b3SubV( b3AbsV( b3CrossV( edge2, b3AddV( vertex1, vertex2 ) ) ), b3AbsV( b3CrossV( edge2, edge1 ) ) ),
									b3MulV( two, b3ModifiedCrossV( b3AbsV( edge2 ), nodeExtent ) ) );
	if ( b3AnyGreater3V( edgeSeparation2, b3_zeroV ) )
	{
		return false;
	}

	b3V32 edgeSeparation3 = b3SubV( b3SubV( b3AbsV( b3CrossV( edge3, b3AddV( vertex2, vertex3 ) ) ), b3AbsV( b3CrossV( edge3, edge2 ) ) ),
									b3MulV( two, b3ModifiedCrossV( b3AbsV( edge3 ), nodeExtent ) ) );
	if ( b3AnyGreater3V( edgeSeparation3, b3_zeroV ) )
	{
		return false;
	}

	return true;
}

b3Fixed b3IntersectRayTriangle( b3V32 rayStart, b3V32 rayDelta, b3V32 vertex1, b3V32 vertex2, b3V32 vertex3 )
{
	// Test if ray intersects this triangle sharing same calculations for each triangle
	{
		b3V32 edge1 = b3SubV( vertex3, vertex2 );
		b3V32 edge2 = b3SubV( vertex1, vertex3 );
		b3V32 edge3 = b3SubV( vertex2, vertex1 );

		b3V32 midPoint1 = b3MulV( b3_halfV, b3AddV( vertex2, vertex3 ) );
		b3V32 midPoint2 = b3MulV( b3_halfV, b3AddV( vertex3, vertex1 ) );
		b3V32 midPoint3 = b3MulV( b3_halfV, b3AddV( vertex1, vertex2 ) );

		b3V32 normal1 = b3CrossV( edge1, b3SubV( midPoint1, rayStart ) );
		b3V32 normal2 = b3CrossV( edge2, b3SubV( midPoint2, rayStart ) );
		b3V32 normal3 = b3CrossV( edge3, b3SubV( midPoint3, rayStart ) );
		B3_TRANSPOSE3( normal1, normal2, normal3 );

		b3V32 rayDeltaX = b3SplatXV( rayDelta );
		b3V32 rayDeltaY = b3SplatYV( rayDelta );
		b3V32 rayDeltaZ = b3SplatZV( rayDelta );

		b3V32 volumes = b3AddV( b3AddV( b3MulV( normal1, rayDeltaX ), b3MulV( normal2, rayDeltaY ) ), b3MulV( normal3, rayDeltaZ ) );
		if ( b3AnyLess3V( volumes, b3_zeroV ) )
		{
			return B3_FIX( 1.0f );
		}
	}

	// Compute intersection with triangle plane
	b3V32 edge1 = b3SubV( vertex2, vertex1 );
	b3V32 edge2 = b3SubV( vertex3, vertex1 );
	b3V32 normal = b3CrossV( edge1, edge2 );

	b3V32 denominator = b3Dot3V( normal, rayDelta );
	if ( b3AnyGreaterEq3V( denominator, b3_zeroV ) )
	{
		return B3_FIX( 1.0f );
	}

	b3V32 lambda = b3DivV( b3Dot3V( normal, b3SubV( vertex1, rayStart ) ), denominator );
	if ( b3AnyLessEq3V( lambda, b3_zeroV ) )
	{
		return B3_FIX( 1.0f );
	}

	lambda = b3MinV( lambda, b3_oneV );
	return b3GetXV( lambda );
}
