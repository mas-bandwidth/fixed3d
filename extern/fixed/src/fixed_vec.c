// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
#include "fixed/fixed_vec.h"

#if defined( FIX_SATURATE )
// Internal: a + b + c + d, exact at 128 bits and saturated once.
static fixed_t fixSum4Saturate( fixed_t a, fixed_t b, fixed_t c, fixed_t d )
{
	return fixFromInt128Saturate( fixInt128Add( fixInt128Add( fixInt128FromI64( a ), fixInt128FromI64( b ) ),
												fixInt128Add( fixInt128FromI64( c ), fixInt128FromI64( d ) ) ) );
}

// Under FIX_SATURATE every sum is exact at 128 bits and saturated once, and the branch
// reads the exact trace, so wherever the default build's sums are in range this is the
// default build's quaternion. Negating a valid fixed_t cannot overflow.
fixQuat fixMakeQuatFromMatrix( const fixMatrix3* m )
{
	fixVec3 c1 = m->cx;
	fixVec3 c2 = m->cy;
	fixVec3 c3 = m->cz;

	fixQuat q;

	fixInt128 trace = fixInt128Add( fixInt128Add( fixInt128FromI64( m->cx.x ), fixInt128FromI64( m->cy.y ) ), fixInt128FromI64( m->cz.z ) );
	if ( !fixInt128IsNegative( trace ) )
	{
		q.v.x = fixSubSaturate( c2.z, c3.y );
		q.v.y = fixSubSaturate( c3.x, c1.z );
		q.v.z = fixSubSaturate( c1.y, c2.x );
		q.s = fixFromInt128Saturate( fixInt128Add( trace, fixInt128FromI64( FIX( 1.0f ) ) ) );
	}
	else
	{
		if ( c1.x > c2.y && c1.x > c3.z )
		{
			q.v.x = fixSum4Saturate( c1.x, -c2.y, -c3.z, FIX( 1.0f ) );
			q.v.y = fixAddSaturate( c2.x, c1.y );
			q.v.z = fixAddSaturate( c3.x, c1.z );
			q.s = fixSubSaturate( c2.z, c3.y );
		}
		else if ( c2.y > c3.z )
		{
			q.v.x = fixAddSaturate( c1.y, c2.x );
			q.v.y = fixSum4Saturate( c2.y, -c3.z, -c1.x, FIX( 1.0f ) );
			q.v.z = fixAddSaturate( c3.y, c2.z );
			q.s = fixSubSaturate( c3.x, c1.z );
		}
		else
		{
			q.v.x = fixAddSaturate( c1.z, c3.x );
			q.v.y = fixAddSaturate( c2.z, c3.y );
			q.v.z = fixSum4Saturate( c3.z, -c1.x, -c2.y, FIX( 1.0f ) );
			q.s = fixSubSaturate( c1.y, c2.x );
		}
	}

	return fixNormalizeQuat( q );
}
#else
fixQuat fixMakeQuatFromMatrix( const fixMatrix3* m )
{
	fixVec3 c1 = m->cx;
	fixVec3 c2 = m->cy;
	fixVec3 c3 = m->cz;

	fixQuat q;

	fixed_t trace = m->cx.x + m->cy.y + m->cz.z;
	if ( trace >= FIX( 0.0f ) )
	{
		q.v.x = c2.z - c3.y;
		q.v.y = c3.x - c1.z;
		q.v.z = c1.y - c2.x;
		q.s = trace + FIX( 1.0f );
	}
	else
	{
		if ( c1.x > c2.y && c1.x > c3.z )
		{
			q.v.x = c1.x - c2.y - c3.z + FIX( 1.0f );
			q.v.y = c2.x + c1.y;
			q.v.z = c3.x + c1.z;
			q.s = c2.z - c3.y;
		}
		else if ( c2.y > c3.z )
		{
			q.v.x = c1.y + c2.x;
			q.v.y = c2.y - c3.z - c1.x + FIX( 1.0f );
			q.v.z = c3.y + c2.z;
			q.s = c3.x - c1.z;
		}
		else
		{
			q.v.x = c1.z + c3.x;
			q.v.y = c2.z + c3.y;
			q.v.z = c3.z - c1.x - c2.y + FIX( 1.0f );
			q.s = c1.y - c2.x;
		}
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return fixNormalizeQuat( q );
}
#endif

fixQuat fixComputeQuatBetweenUnitVectors( fixVec3 v1, fixVec3 v2 )
{
	FIX_ASSERT( fixIsNormalized( v1 ) );
	FIX_ASSERT( fixIsNormalized( v2 ) );

	fixQuat out;

	fixVec3 m = fixVecLerp( v1, v2, FIX( 0.5f ) );
	// Nearly anti-parallel vectors need the perpendicular fallback. In fixed point
	// the threshold must sit well above the Q48.16 resolution of the squared length,
	// because normalizing a short vector amplifies quantization error.
	if ( fixLengthSquared( m ) > FIX( 0.0001f ) )
	{
		// Normalize first so the quaternion is algebraically unit length
		m = fixNormalize( m );
		out.v = fixCross( v1, m );
		out.s = fixDot( v1, m );
	}
	else
	{
		// Anti-parallel: Use a perpendicular vector
		if ( fixAbs( v1.x ) > FIX( 0.5f ) )
		{
			out.v.x = v1.y;
			out.v.y = -v1.x;
			out.v.z = FIX( 0.0f );
		}
		else
		{
			out.v.x = FIX( 0.0f );
			out.v.y = v1.z;
			out.v.z = -v1.y;
		}

		out.s = FIX( 0.0f );
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return fixNormalizeQuat( out );
}
