// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
#include "fixed/fixed_vec.h"

b3Quat b3MakeQuatFromMatrix( const b3Matrix3* m )
{
	b3Vec3 c1 = m->cx;
	b3Vec3 c2 = m->cy;
	b3Vec3 c3 = m->cz;

	b3Quat q;

	b3Fixed trace = m->cx.x + m->cy.y + m->cz.z;
	if ( trace >= B3_FIX( 0.0f ) )
	{
		q.v.x = c2.z - c3.y;
		q.v.y = c3.x - c1.z;
		q.v.z = c1.y - c2.x;
		q.s = trace + B3_FIX( 1.0f );
	}
	else
	{
		if ( c1.x > c2.y && c1.x > c3.z )
		{
			q.v.x = c1.x - c2.y - c3.z + B3_FIX( 1.0f );
			q.v.y = c2.x + c1.y;
			q.v.z = c3.x + c1.z;
			q.s = c2.z - c3.y;
		}
		else if ( c2.y > c3.z )
		{
			q.v.x = c1.y + c2.x;
			q.v.y = c2.y - c3.z - c1.x + B3_FIX( 1.0f );
			q.v.z = c3.y + c2.z;
			q.s = c3.x - c1.z;
		}
		else
		{
			q.v.x = c1.z + c3.x;
			q.v.y = c2.z + c3.y;
			q.v.z = c3.z - c1.x - c2.y + B3_FIX( 1.0f );
			q.s = c1.y - c2.x;
		}
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return b3NormalizeQuat( q );
}

b3Quat b3ComputeQuatBetweenUnitVectors( b3Vec3 v1, b3Vec3 v2 )
{
	B3_ASSERT( b3IsNormalized( v1 ) );
	B3_ASSERT( b3IsNormalized( v2 ) );

	b3Quat out;

	b3Vec3 m = b3Lerp( v1, v2, B3_FIX( 0.5f ) );
	// Nearly anti-parallel vectors need the perpendicular fallback. In fixed point
	// the threshold must sit well above the Q48.16 resolution of the squared length,
	// because normalizing a short vector amplifies quantization error.
	if ( b3LengthSquared( m ) > B3_FIX( 0.0001f ) )
	{
		// Normalize first so the quaternion is algebraically unit length
		m = b3Normalize( m );
		out.v = b3Cross( v1, m );
		out.s = b3Dot( v1, m );
	}
	else
	{
		// Anti-parallel: Use a perpendicular vector
		if ( b3FixAbs( v1.x ) > B3_FIX( 0.5f ) )
		{
			out.v.x = v1.y;
			out.v.y = -v1.x;
			out.v.z = B3_FIX( 0.0f );
		}
		else
		{
			out.v.x = B3_FIX( 0.0f );
			out.v.y = v1.z;
			out.v.z = -v1.y;
		}

		out.s = B3_FIX( 0.0f );
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return b3NormalizeQuat( out );
}
