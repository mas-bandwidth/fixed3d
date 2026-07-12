// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"

#include "math_internal.h"

#include "box3d/math_functions.h"


// Similar to Real-time Collision Detection, p179.
// todo try
// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
bool b3RayCastAABB( b3AABB a, b3Vec3 p1, b3Vec3 p2, b3Fixed* minFraction, b3Fixed* maxFraction )
{
	// Ray direction and length
	b3Vec3 d = b3Sub( p2, p1 );
	b3Fixed rayLength = b3Length( d );

	// Handle degenerate ray
	if ( rayLength < B3_FIXED_EPSILON )
	{
		// Check if point is inside AABB
		if ( p1.x >= a.lowerBound.x && p1.x <= a.upperBound.x && p1.y >= a.lowerBound.y && p1.y <= a.upperBound.y &&
			 p1.z >= a.lowerBound.z && p1.z <= a.upperBound.z )
		{
			*minFraction = B3_FIX( 0.0f );
			*maxFraction = B3_FIX( 0.0f );
			return true;
		}

		return false;
	}

	b3Vec3 rayDir = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , rayLength ), d );

	// Slab method for ray-AABB intersection
	b3Fixed tMin = B3_FIX( 0.0f );
	b3Fixed tMax = rayLength;

	// x-axis
	{
		b3Fixed rayComponent = rayDir.x;
		b3Fixed rayStart = p1.x;
		b3Fixed boxMin = a.lowerBound.x;
		b3Fixed boxMax = a.upperBound.x;

		if ( b3FixAbs( rayComponent ) < B3_FIXED_EPSILON )
		{
			// Ray is parallel to slab, check if ray origin is within slab
			if ( rayStart < boxMin || rayStart > boxMax )
			{
				return false;
			}
		}
		else
		{
			// Compute intersection distances
			b3Fixed t1 = b3FixDiv( ( boxMin - rayStart ) , rayComponent );
			b3Fixed t2 = b3FixDiv( ( boxMax - rayStart ) , rayComponent );

			// Ensure t1 <= t2
			if ( t1 > t2 )
			{
				b3Fixed temp = t1;
				t1 = t2;
				t2 = temp;
			}

			// Update intersection interval
			tMin = b3FixMax( tMin, t1 );
			tMax = b3FixMin( tMax, t2 );

			// Check for no intersection
			if ( tMin > tMax )
			{
				return false;
			}
		}
	}
	
	// y-axis
	{
		b3Fixed rayComponent = rayDir.y;
		b3Fixed rayStart = p1.y;
		b3Fixed boxMin = a.lowerBound.y;
		b3Fixed boxMax = a.upperBound.y;

		if ( b3FixAbs( rayComponent ) < B3_FIXED_EPSILON )
		{
			// Ray is parallel to slab, check if ray origin is within slab
			if ( rayStart < boxMin || rayStart > boxMax )
			{
				return false;
			}
		}
		else
		{
			// Compute intersection distances
			b3Fixed t1 = b3FixDiv( ( boxMin - rayStart ) , rayComponent );
			b3Fixed t2 = b3FixDiv( ( boxMax - rayStart ) , rayComponent );

			// Ensure t1 <= t2
			if ( t1 > t2 )
			{
				b3Fixed temp = t1;
				t1 = t2;
				t2 = temp;
			}

			// Update intersection interval
			tMin = b3FixMax( tMin, t1 );
			tMax = b3FixMin( tMax, t2 );

			// Check for no intersection
			if ( tMin > tMax )
			{
				return false;
			}
		}
	}

	// z-axis
	{
		b3Fixed rayComponent = rayDir.z;
		b3Fixed rayStart = p1.z;
		b3Fixed boxMin = a.lowerBound.z;
		b3Fixed boxMax = a.upperBound.z;

		if ( b3FixAbs( rayComponent ) < B3_FIXED_EPSILON )
		{
			// Ray is parallel to slab, check if ray origin is within slab
			if ( rayStart < boxMin || rayStart > boxMax )
			{
				return false;
			}
		}
		else
		{
			// Compute intersection distances
			b3Fixed t1 = b3FixDiv( ( boxMin - rayStart ) , rayComponent );
			b3Fixed t2 = b3FixDiv( ( boxMax - rayStart ) , rayComponent );

			// Ensure t1 <= t2
			if ( t1 > t2 )
			{
				b3Fixed temp = t1;
				t1 = t2;
				t2 = temp;
			}

			// Update intersection interval
			tMin = b3FixMax( tMin, t1 );
			tMax = b3FixMin( tMax, t2 );

			// Check for no intersection
			if ( tMin > tMax )
			{
				return false;
			}
		}
	}

	// Check if intersection is behind ray start
	if ( tMax < B3_FIX( 0.0f ) )
	{
		return false;
	}

	// Convert distances to fractions
	*minFraction = b3FixClamp( b3FixDiv( tMin , rayLength ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
	*maxFraction = b3FixClamp( b3FixDiv( tMax , rayLength ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );

	return true;
}
