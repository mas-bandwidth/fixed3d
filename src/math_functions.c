// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/math_functions.h"

#include "math_internal.h"

#include "box3d/collision.h"
#include "box3d/constants.h"

#include <math.h>
#include <string.h>

bool b3IsValidFixed( b3Fixed a )
{
	// Fixed point has no NaN, and the saturation values are legal quantities:
	// B3_FIXED_MAX plays the role float's FLT_MAX did (joint thresholds and
	// spring force limits default to it, mirroring isfinite( FLT_MAX ) == true).
	// Only INT64_MIN is unrepresentable (reserved so negation cannot overflow).
	return a != INT64_MIN;
}

bool b3IsValidVec3( b3Vec3 a )
{
	return b3IsValidFixed( a.x ) && b3IsValidFixed( a.y ) && b3IsValidFixed( a.z );
}

bool b3IsValidQuat( b3Quat a )
{
	if ( !b3IsValidFixed( a.v.x ) || !b3IsValidFixed( a.v.y ) || !b3IsValidFixed( a.v.z ) || !b3IsValidFixed( a.s ) )
	{
		return false;
	}

	return b3IsNormalizedQuat( a );
}

bool b3IsValidTransform( b3Transform a )
{
	return b3IsValidVec3( a.p ) && b3IsValidQuat( a.q );
}

bool b3IsValidMatrix3( b3Matrix3 a )
{
	return b3IsValidVec3( a.cx ) && b3IsValidVec3( a.cy ) && b3IsValidVec3( a.cz );
}

bool b3IsValidAABB( b3AABB a )
{
	if ( b3IsValidVec3( b3BoundToVec3( a.lowerBound ) ) == false )
	{
		return false;
	}

	if ( b3IsValidVec3( b3BoundToVec3( a.upperBound ) ) == false )
	{
		return false;
	}

	if ( a.lowerBound.x > a.upperBound.x )
	{
		return false;
	}

	if ( a.lowerBound.y > a.upperBound.y )
	{
		return false;
	}

	if ( a.lowerBound.z > a.upperBound.z )
	{
		return false;
	}

	return true;
}

bool b3IsBoundedAABB( b3AABB a )
{
	if ( a.lowerBound.x < -B3_HUGE || a.lowerBound.y < -B3_HUGE || a.lowerBound.z < -B3_HUGE )
	{
		return false;
	}

	if ( a.upperBound.x > B3_HUGE || a.upperBound.y > B3_HUGE || a.upperBound.z > B3_HUGE )
	{
		return false;
	}

	return true;
}

bool b3IsSaneAABB( b3AABB a )
{
	if ( b3IsValidAABB( a ) == false )
	{
		return false;
	}

	if ( a.lowerBound.x < -B3_HUGE || a.lowerBound.y < -B3_HUGE || a.lowerBound.z < -B3_HUGE )
	{
		return false;
	}

	if ( a.upperBound.x > B3_HUGE || a.upperBound.y > B3_HUGE || a.upperBound.z > B3_HUGE )
	{
		return false;
	}

	return true;
}

bool b3IsValidPlane( b3Plane a )
{
	if ( b3IsValidVec3( a.normal ) == false )
	{
		return false;
	}

	if ( b3IsNormalized( a.normal ) == false )
	{
		return false;
	}

	return b3IsValidFixed( a.offset );
}

#if defined( BOX3D_LUDICROUS_MODE )
// Wide world coordinate: mirror b3IsValidFixed — every value is a legal quantity
// except the 128-bit minimum, which is reserved so negation cannot overflow.
static inline bool b3IsValidWideCoord( b3Int128 x )
{
	return x != (b3Int128)( (b3UInt128)1 << 127 );
}

bool b3IsValidPosition( b3Pos p )
{
	return b3IsValidWideCoord( p.x ) && b3IsValidWideCoord( p.y ) && b3IsValidWideCoord( p.z );
}
#else
bool b3IsValidPosition( b3Pos p )
{
	return b3IsValidVec3( p );
}
#endif

bool b3IsValidWorldTransform( b3WorldTransform t )
{
	return b3IsValidPosition( t.p ) && b3IsValidQuat( t.q );
}

// Internal Q32.32 helpers for the deterministic trig routines. Angles are in
// [-pi, pi] so the extra fraction bits keep the approximations accurate well
// below the Q48.16 output resolution.

static inline int64_t b3Q32Mul( int64_t a, int64_t b )
{
	return (int64_t)( ( (b3Int128)a * b ) >> 32 );
}

static inline int64_t b3Q32Div( int64_t a, int64_t b )
{
	return (int64_t)b3Int128Div( b3Int128ShiftLeft( a, 32 ), b );
}


static inline int64_t b3Q32Sqrt( int64_t a )
{
	if ( a <= 0 )
	{
		return 0;
	}
	uint64_t hi = (uint64_t)a >> 32;
	uint64_t lo = (uint64_t)a << 32;
	return (int64_t)b3ISqrt128High( hi, lo );
}

// Round a Q32.32 value to Q48.16
static inline b3Fixed b3Q32ToFix( int64_t a )
{
	return ( a + ( (int64_t)1 << 15 ) ) >> 16;
}

#define B3_Q32_PI 13493037705LL	   // round(pi * 2^32)
#define B3_Q32_HALF_PI 6746518852LL // round(pi/2 * 2^32)

// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
b3Fixed b3Atan2( b3Fixed y, b3Fixed x )
{
	// Added check for (0,0) to match atan2f and avoid a zero divide
	if ( x == 0 && y == 0 )
	{
		return 0;
	}

	b3Fixed ax = b3FixAbs( x );
	b3Fixed ay = b3FixAbs( y );
	b3Fixed mx = b3FixMax( ay, ax );
	b3Fixed mn = b3FixMin( ay, ax );

	// a = mn / mx in [0, 1], evaluated in Q32.32. mn >= 0 so the raw shift
	// would be fine, but the helper is the convention for signed shifts.
	int64_t a = (int64_t)b3Int128Div( b3Int128ShiftLeft( (b3Int128)mn, 32 ), mx );

	// Minimax polynomial approximation to atan(a) on [0,1]
	int64_t s = b3Q32Mul( a, a );
	int64_t c = b3Q32Mul( s, a );
	int64_t q = b3Q32Mul( s, s );
	int64_t r = b3Q32Mul( 106688212LL, q ) + 802360794LL;	  // 0.024840285, 0.18681418
	int64_t t = b3Q32Mul( -404147609LL, q ) - 1426490580LL; // -0.094097948, -0.33213072
	r = b3Q32Mul( r, s ) + t;
	r = b3Q32Mul( r, c ) + a;

	// Map to full circle
	if ( ay > ax )
	{
		r = B3_Q32_HALF_PI - r;
	}

	if ( x < 0 )
	{
		r = B3_Q32_PI - r;
	}

	if ( y < 0 )
	{
		r = -r;
	}

	return b3Q32ToFix( r );
}

// Approximate cosine and sine for determinism, evaluated in pure integer math.
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
b3CosSin b3ComputeCosSin( b3Fixed radians )
{
	const int64_t pi = B3_Q32_PI;
	const int64_t halfPi = B3_Q32_HALF_PI;
	const int64_t pi2 = 42389628127LL;		 // pi^2 in Q32.32
	const int64_t fivePi2 = 211948140636LL; // 5*pi^2 in Q32.32

	// The unwound angle is in [-pi, pi] to within an ulp; promote to Q32.32
	int64_t x = b3FixShiftLeft( b3UnwindAngle( radians ), 16 );
	x = x < -pi ? -pi : ( x > pi ? pi : x );

	// cosine needs angle in [-pi/2, pi/2]
	int64_t c;
	if ( x < -halfPi )
	{
		int64_t y = x + pi;
		int64_t y2 = b3Q32Mul( y, y );
		c = -b3Q32Div( pi2 - 4 * y2, pi2 + y2 );
	}
	else if ( x > halfPi )
	{
		int64_t y = x - pi;
		int64_t y2 = b3Q32Mul( y, y );
		c = -b3Q32Div( pi2 - 4 * y2, pi2 + y2 );
	}
	else
	{
		int64_t y2 = b3Q32Mul( x, x );
		c = b3Q32Div( pi2 - 4 * y2, pi2 + y2 );
	}

	// sine needs angle in [0, pi]
	int64_t s;
	if ( x < 0 )
	{
		int64_t y = x + pi;
		int64_t u = b3Q32Mul( y, pi - y );
		s = -b3Q32Div( 16 * u, fivePi2 - 4 * u );
	}
	else
	{
		int64_t u = b3Q32Mul( x, pi - x );
		s = b3Q32Div( 16 * u, fivePi2 - 4 * u );
	}

	// Normalize so the pair lies on the unit circle
	int64_t mag = b3Q32Sqrt( b3Q32Mul( s, s ) + b3Q32Mul( c, c ) );
	if ( mag > 0 )
	{
		c = b3Q32Div( c, mag );
		s = b3Q32Div( s, mag );
	}

	b3CosSin cs = { b3Q32ToFix( c ), b3Q32ToFix( s ) };
	return cs;
}

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

b3SegmentDistanceResult b3LineDistance( b3Vec3 p1, b3Vec3 d1, b3Vec3 p2, b3Vec3 d2 )
{
	b3SegmentDistanceResult result;

	// Solve A*x = b
	b3Fixed a11 = b3Dot( d1, d1 );
	b3Fixed a12 = -b3Dot( d1, d2 );
	b3Fixed a21 = b3Dot( d2, d1 );
	b3Fixed a22 = -b3Dot( d2, d2 );

	b3Vec3 w = b3Sub( p1, p2 );
	b3Fixed b1 = -b3Dot( d1, w );
	b3Fixed b2 = -b3Dot( d2, w );

	b3Fixed det = b3FixMul( a11 , a22 ) - b3FixMul( a12 , a21 );
	if ( det == 0 )
	{
		// Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
		b3Fixed s1 = b3FixDiv( b3Dot( b3Sub( p2, p1 ), d1 ) , b3Dot( d1, d1 ) );
		b3Fixed s2 = B3_FIX( 0.0f );

		result.point1 = b3MulAdd( p1, s1, d1 );
		result.fraction1 = s1;
		result.point2 = b3MulAdd( p2, s2, d2 );
		result.fraction2 = s2;

		return result;
	}

	b3Fixed s1 = b3FixDiv( ( b3FixMul( a22 , b1 ) - b3FixMul( a12 , b2 ) ) , det );
	b3Fixed s2 = b3FixDiv( ( b3FixMul( a11 , b2 ) - b3FixMul( a21 , b1 ) ) , det );

	result.point1 = b3MulAdd( p1, s1, d1 );
	result.fraction1 = s1;
	result.point2 = b3MulAdd( p2, s2, d2 );
	result.fraction2 = s2;
	return result;
}

b3SegmentDistanceResult b3SegmentDistance( b3Vec3 p1, b3Vec3 q1, b3Vec3 p2, b3Vec3 q2 )
{
	b3SegmentDistanceResult result;

	b3Vec3 d1 = b3Sub( q1, p1 );
	b3Vec3 d2 = b3Sub( q2, p2 );
	b3Vec3 r = b3Sub( p1, p2 );

	b3Fixed a = b3Dot( d1, d1 );
	b3Fixed b = b3Dot( d1, d2 );
	b3Fixed c = b3Dot( d1, r );
	b3Fixed e = b3Dot( d2, d2 );
	b3Fixed f = b3Dot( d2, r );

	// Check if one of the segments degenerates into a point
	if ( a < 100 * B3_FIXED_EPSILON && e < 100 * B3_FIXED_EPSILON )
	{
		// Both segments degenerate into points
		result.point1 = p1;
		result.fraction1 = B3_FIX( 0.0f );
		result.point2 = p2;
		result.fraction2 = B3_FIX( 0.0f );

		return result;
	}

	if ( a < 100 * B3_FIXED_EPSILON )
	{
		// First segment degenerates into a point
		b3Fixed s2 = b3FixClamp( b3FixDiv( f , e ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );

		result.point1 = p1;
		result.fraction1 = B3_FIX( 0.0f );
		result.point2 = b3MulAdd( p2, s2, d2 );
		result.fraction2 = s2;

		return result;
	}

	if ( e < 100 * B3_FIXED_EPSILON )
	{
		// Second segment degenerates into a point
		b3Fixed s1 = b3FixClamp( b3FixDiv( -c , a ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );

		result.point1 = b3MulAdd( p1, s1, d1 );
		result.fraction1 = s1;
		result.point2 = p2;
		result.fraction2 = B3_FIX( 0.0f );

		return result;
	}

	// Non-degenerate case
	b3Fixed denom = b3FixMul( a , e ) - b3FixMul( b , b );
	b3Fixed s1 = denom > 0 ? b3FixClamp( b3FixDiv( ( b3FixMul( b , f ) - b3FixMul( c , e ) ) , denom ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) ) : B3_FIX( 0.0f );
	b3Fixed s2 = b3FixDiv( ( b3FixMul( b , s1 ) + f ) , e );

	// Clamp lambda2 and recompute lambda1 if necessary
	if ( s2 < B3_FIX( 0.0f ) )
	{
		s1 = b3FixClamp( b3FixDiv( -c , a ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
		s2 = B3_FIX( 0.0f );
	}
	else if ( s2 > B3_FIX( 1.0f ) )
	{
		s1 = b3FixClamp( b3FixDiv( ( b - c ) , a ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
		s2 = B3_FIX( 1.0f );
	}

	result.point1 = b3MulAdd( p1, s1, d1 );
	result.fraction1 = s1;
	result.point2 = b3MulAdd( p2, s2, d2 );
	result.fraction2 = s2;

	return result;
}

b3Vec3 b3PointToSegmentDistance( b3Vec3 a, b3Vec3 b, b3Vec3 q )
{
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 aq = b3Sub( q, a );

	b3Fixed alpha = b3Dot( ab, aq );

	if ( alpha <= B3_FIX( 0.0f ) )
	{
		// q projects outside interval [a, b] on the side of a
		return a;
	}
	else
	{
		b3Fixed denominator = b3Dot( ab, ab );
		if ( alpha > denominator )
		{
			// q projects outside interval [a, b] on the side of b
			return b;
		}
		else
		{
			// q projects inside interval [a, b]
			alpha = b3FixDiv( alpha, denominator );
			return b3MulAdd( a, alpha, ab );
		}
	}
}

b3TrianglePoint b3ClosestPointOnTriangle( b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 q )
{
	// Check if P lies in vertex region of A
	b3Vec3 ab = b3Sub( b, a );
	b3Vec3 ac = b3Sub( c, a );
	b3Vec3 aq = b3Sub( q, a );

	b3Fixed d1 = b3Dot( ab, aq );
	b3Fixed d2 = b3Dot( ac, aq );
	if ( d1 <= B3_FIX( 0.0f ) && d2 <= B3_FIX( 0.0f ) )
	{
		return (b3TrianglePoint){ a, b3_featureVertex1 };
	}

	// Check if P lies in vertex region of B
	b3Vec3 bq = b3Sub( q, b );

	b3Fixed d3 = b3Dot( ab, bq );
	b3Fixed d4 = b3Dot( ac, bq );
	if ( d3 > B3_FIX( 0.0f ) && d4 <= d3 )
	{
		return (b3TrianglePoint){ b, b3_featureVertex2 };
	}

	// Check if P lies in edge region AB
	b3Fixed vc = b3FixMul( d1 , d4 ) - b3FixMul( d3 , d2 );
	if ( vc <= B3_FIX( 0.0f ) && d1 >= B3_FIX( 0.0f ) && d3 <= B3_FIX( 0.0f ) )
	{
		b3Fixed t = b3FixDiv( d1 , ( d1 - d3 ) );
		return (b3TrianglePoint){ b3MulAdd( a, t, ab ), b3_featureEdge1 };
	}

	// Check if P lies in vertex region of C
	b3Vec3 cq = b3Sub( q, c );

	b3Fixed d5 = b3Dot( ab, cq );
	b3Fixed d6 = b3Dot( ac, cq );
	if ( d6 >= B3_FIX( 0.0f ) && d5 <= d6 )
	{
		return (b3TrianglePoint){ c, b3_featureVertex3 };
	}

	// Check if P lies in edge region AC
	b3Fixed vb = b3FixMul( d5 , d2 ) - b3FixMul( d1 , d6 );
	if ( vb <= B3_FIX( 0.0f ) && d2 >= B3_FIX( 0.0f ) && d6 <= B3_FIX( 0.0f ) )
	{
		b3Fixed t = b3FixDiv( d2 , ( d2 - d6 ) );
		return (b3TrianglePoint){ b3MulAdd( a, t, ac ), b3_featureEdge3 };
	}

	// Check if P lies in edge region of BC
	b3Fixed va = b3FixMul( d3 , d6 ) - b3FixMul( d5 , d4 );
	if ( va <= B3_FIX( 0.0f ) && d4 >= d3 && d5 >= d6 )
	{
		b3Vec3 bc = b3Sub( c, b );

		b3Fixed t = b3FixDiv( ( d4 - d3 ) , ( ( d4 - d3 ) + ( d5 - d6 ) ) );
		return (b3TrianglePoint){ b3MulAdd( b, t, bc ), b3_featureEdge2 };
	}

	// P inside face region ABC
	b3Fixed t1 = b3FixDiv( vb , ( va + vb + vc ) );
	b3Fixed t2 = b3FixDiv( vc , ( va + vb + vc ) );

	b3Vec3 p = b3MulAdd( a, t1, ab );
	p = b3MulAdd( p, t2, ac );
	return (b3TrianglePoint){ p, b3_featureTriangleFace };
}

b3Matrix3 b3SphereInertia( b3Fixed mass, b3Fixed radius )
{
	b3Fixed i = b3FixMul( b3FixMul( b3FixMul( B3_FIX( 0.4f ) , mass ) , radius ) , radius );
	return b3MakeDiagonalMatrix( i, i, i );
}

b3Matrix3 b3CylinderInertia( b3Fixed mass, b3Fixed radius, b3Fixed height )
{
	b3Fixed ixx = b3FixDiv( b3FixMul( mass , ( b3FixMul( b3FixMul( b3FixFromInt( 3 ) , radius ) , radius ) + b3FixMul( height , height ) ) ) , B3_FIX( 12.0f ) );
	b3Fixed iyy = b3FixMul( b3FixMul( b3FixMul( B3_FIX( 0.5f ) , mass ) , radius ) , radius );
	return b3MakeDiagonalMatrix( ixx, iyy, ixx );
}

b3Matrix3 b3BoxInertia( b3Fixed mass, b3Vec3 min, b3Vec3 max )
{
	b3Vec3 delta = b3Sub( max, min );
	b3Fixed ixx = b3FixDiv( b3FixMul( mass , ( b3FixMul( delta.y , delta.y ) + b3FixMul( delta.z , delta.z ) ) ) , B3_FIX( 12.0f ) );
	b3Fixed iyy = b3FixDiv( b3FixMul( mass , ( b3FixMul( delta.x , delta.x ) + b3FixMul( delta.z , delta.z ) ) ) , B3_FIX( 12.0f ) );
	b3Fixed izz = b3FixDiv( b3FixMul( mass , ( b3FixMul( delta.x , delta.x ) + b3FixMul( delta.y , delta.y ) ) ) , B3_FIX( 12.0f ) );

	return b3MakeDiagonalMatrix( ixx, iyy, izz );
}

// https://en.wikipedia.org/wiki/Parallel_axis_theorem
b3Matrix3 b3Steiner( b3Fixed mass, b3Vec3 origin )
{
	// Usage: Io = Ic + Is and Ic = Io - Is
	b3Fixed ixx = b3FixMul( mass , ( b3FixMul( origin.y , origin.y ) + b3FixMul( origin.z , origin.z ) ) );
	b3Fixed iyy = b3FixMul( mass , ( b3FixMul( origin.x , origin.x ) + b3FixMul( origin.z , origin.z ) ) );
	b3Fixed izz = b3FixMul( mass , ( b3FixMul( origin.x , origin.x ) + b3FixMul( origin.y , origin.y ) ) );
	b3Fixed ixy = b3FixMul( b3FixMul( -mass , origin.x ) , origin.y );
	b3Fixed ixz = b3FixMul( b3FixMul( -mass , origin.x ) , origin.z );
	b3Fixed iyz = b3FixMul( b3FixMul( -mass , origin.y ) , origin.z );

	// Write
	b3Matrix3 out;
	out.cx.x = ixx;
	out.cy.x = ixy;
	out.cz.x = ixz;
	out.cx.y = ixy;
	out.cy.y = iyy;
	out.cz.y = iyz;
	out.cx.z = ixz;
	out.cy.z = iyz;
	out.cz.z = izz;

	return out;
}

bool b3IsValidRay( const b3RayCastInput* input )
{
	bool isValid = b3IsValidVec3( input->origin ) && b3IsValidVec3( input->translation ) &&
				   b3IsValidFixed( input->maxFraction ) && B3_FIX( 0.0f ) <= input->maxFraction && input->maxFraction < B3_HUGE;
	return isValid;
}


// ---------------------------------------------------------------------------------------------
// Fixed-point exp2 / log2 / pow (the deterministic transcendental ladder for the game control
// path). Determinism over accuracy: every operation is pure integer arithmetic.
// ---------------------------------------------------------------------------------------------

// 2^(2^-k) in Q32, k = 1..16: the binary-exponentiation table for the fractional part of exp2.
static const uint64_t b3_exp2Table[16] = {
	0x000000016A09E668ull, // 2^(2^-1)
	0x00000001306FE0A3ull, // 2^(2^-2)
	0x00000001172B83C8ull, // 2^(2^-3)
	0x000000010B5586D0ull, // 2^(2^-4)
	0x00000001059B0D31ull, // 2^(2^-5)
	0x0000000102C9A3E7ull, // 2^(2^-6)
	0x000000010163DAA0ull, // 2^(2^-7)
	0x0000000100B1AFA6ull, // 2^(2^-8)
	0x000000010058C86Eull, // 2^(2^-9)
	0x00000001002C605Eull, // 2^(2^-10)
	0x0000000100162F39ull, // 2^(2^-11)
	0x00000001000B175Full, // 2^(2^-12)
	0x0000000100058BA0ull, // 2^(2^-13)
	0x000000010002C5CCull, // 2^(2^-14)
	0x00000001000162E5ull, // 2^(2^-15)
	0x000000010000B172ull, // 2^(2^-16)
};

b3Fixed b3FixLog2( b3Fixed a )
{
	if ( a <= 0 )
	{
		return INT64_MIN;
	}

	// integer part: position of the leading bit relative to the fraction point
	int msb = 63 - __builtin_clzll( (uint64_t)a );
	int64_t integerPart = (int64_t)( msb - B3_FIXED_FRACTION_BITS );

	// mantissa m in [1, 2), carried in Q2.62 (fits u64: raw in [2^62, 2^63)): the exact binary
	// digits of log2( m ) fall out of repeated squaring -- square m; if the square reaches
	// [2, 4), the digit is 1 and the square halves back into [1, 2).
	uint64_t m;
	if ( msb <= 62 )
	{
		m = (uint64_t)a << ( 62 - msb );
	}
	else
	{
		m = (uint64_t)a >> ( msb - 62 );
	}

	uint64_t fraction32 = 0;
	for ( int i = 0; i < 32; ++i )
	{
		b3UInt128 sq = (b3UInt128)m * m;	// Q4.124, value in [1, 4)
		fraction32 <<= 1;
		if ( sq >= ( (b3UInt128)1 << 125 ) )
		{
			fraction32 |= 1;
			m = (uint64_t)( sq >> 63 );		// halve back into [1, 2) at Q2.62
		}
		else
		{
			m = (uint64_t)( sq >> 62 );		// renormalize to Q2.62
		}
	}

	// assemble: integer part in Q48.16 plus the 32 exact fraction bits rounded to 16
	int64_t fraction16 = (int64_t)( ( fraction32 + ( 1ull << 15 ) ) >> 16 );
	return ( integerPart << B3_FIXED_FRACTION_BITS ) + fraction16;
}

b3Fixed b3FixExp2( b3Fixed a )
{
	// split into integer floor and fractional part in [0, 1)
	int64_t n = a >> B3_FIXED_FRACTION_BITS;
	uint64_t f = (uint64_t)( a - ( n << B3_FIXED_FRACTION_BITS ) );	// Q16 fraction, [0, 2^16)

	if ( n >= 47 )
	{
		return INT64_MAX;		// saturate: 2^47 is the top of the Q48.16 whole-unit domain
	}

	if ( n < -17 )
	{
		return 0;				// underflow below the smallest representable value
	}

	// 2^f as a product over the set bits of f, in Q32: r in [ 2^32, 2^33 )
	uint64_t r = 1ull << 32;
	for ( int k = 0; k < 16; ++k )
	{
		// bit for 2^-( k + 1 ) is fraction bit ( 15 - k )
		if ( f & ( 1ull << ( 15 - k ) ) )
		{
			r = (uint64_t)( ( (b3UInt128)r * b3_exp2Table[k] ) >> 32 );
		}
	}

	// scale by 2^n: result raw = r * 2^n / 2^16 (r is Q32, output is Q16)
	int shift = 16 - (int)n;
	if ( shift <= 0 )
	{
		return (b3Fixed)( r << ( -shift ) );
	}
	if ( shift >= 64 )
	{
		return 0;
	}
	// round to nearest on the dropped bits, the pinned ( raw + half ) >> drop form
	return (b3Fixed)( ( r + ( 1ull << ( shift - 1 ) ) ) >> shift );
}

b3Fixed b3FixPow( b3Fixed base, b3Fixed exponent )
{
	if ( base <= 0 )
	{
		return 0;
	}

	if ( exponent == 0 )
	{
		return B3_FIXED_ONE;
	}

	return b3FixExp2( b3FixMul( exponent, b3FixLog2( base ) ) );
}

// ---------------------------------------------------------------------------------------------
// Q2.30 quaternions
// ---------------------------------------------------------------------------------------------

b3Quat30 b3Quat30FromQuat( b3Quat q )
{
	b3Quat30 result;
	result.x = b3Fix30FromFix( q.v.x );
	result.y = b3Fix30FromFix( q.v.y );
	result.z = b3Fix30FromFix( q.v.z );
	result.w = b3Fix30FromFix( q.s );
	return result;
}

b3Quat b3QuatFromQuat30( b3Quat30 q )
{
	b3Quat result;
	result.v.x = b3FixFromFix30( q.x );
	result.v.y = b3FixFromFix30( q.y );
	result.v.z = b3FixFromFix30( q.z );
	result.s = b3FixFromFix30( q.w );
	return result;
}

// one normalized Q2.30 component: raw * 2^30 / length, round to nearest, clamped to [-1, 1]
static int32_t b3NormalizeComponent30( int64_t raw, uint64_t length )
{
	uint64_t magnitude = (uint64_t)( raw < 0 ? -raw : raw );
	b3UInt128 numerator = ( (b3UInt128)magnitude << B3_FIXED30_FRACTION_BITS ) + ( length >> 1 );
	uint64_t q = (uint64_t)( numerator / length );
	if ( q > (uint64_t)B3_FIXED30_ONE )
	{
		q = (uint64_t)B3_FIXED30_ONE;		// the rounded divide can overshoot one by an ulp
	}
	return raw < 0 ? -(int32_t)q : (int32_t)q;
}

b3Quat30 b3NormalizeQuat30( b3Quat30 q )
{
	int64_t x = q.x.raw;
	int64_t y = q.y.raw;
	int64_t z = q.z.raw;
	int64_t w = q.w.raw;

	// squared length in Q4.60: components are at most 2^31 in magnitude, so each square is at
	// most 2^62 and the 128-bit accumulate cannot overflow.
	b3UInt128 lengthSquared =
		(b3UInt128)( x * x ) + (b3UInt128)( y * y ) + (b3UInt128)( z * z ) + (b3UInt128)( w * w );

	if ( lengthSquared == 0 )
	{
		b3Quat30 identity = { { 0 }, { 0 }, { 0 }, { B3_FIXED30_ONE } };
		return identity;
	}

	// sqrt of Q4.60 raw is the Q2.30 length raw
	uint64_t length = b3ISqrt128High( (uint64_t)( lengthSquared >> 64 ), (uint64_t)lengthSquared );

	b3Quat30 result;
	result.x.raw = b3NormalizeComponent30( x, length );
	result.y.raw = b3NormalizeComponent30( y, length );
	result.z.raw = b3NormalizeComponent30( z, length );
	result.w.raw = b3NormalizeComponent30( w, length );
	return result;
}

bool b3IsNormalizedQuat30( b3Quat30 q )
{
	int64_t x = q.x.raw;
	int64_t y = q.y.raw;
	int64_t z = q.z.raw;
	int64_t w = q.w.raw;

	b3UInt128 lengthSquared =
		(b3UInt128)( x * x ) + (b3UInt128)( y * y ) + (b3UInt128)( z * z ) + (b3UInt128)( w * w );

	// |length^2 - 1| < 2^-20 in Q4.60 terms (mirrors b3IsNormalizedQuat's tolerance spirit)
	b3UInt128 one = (b3UInt128)1 << ( 2 * B3_FIXED30_FRACTION_BITS );
	b3UInt128 tolerance = (b3UInt128)1 << ( 2 * B3_FIXED30_FRACTION_BITS - 20 );
	b3UInt128 difference = lengthSquared > one ? lengthSquared - one : one - lengthSquared;
	return difference < tolerance;
}

// ---------------------------------------------------------------------------------------------
// Critically damped smoothing (the game control dynamics on fixed point)
// ---------------------------------------------------------------------------------------------

b3Fixed b3SmoothCriticallyDamped( b3Fixed current, b3Fixed target, b3Fixed* velocity, b3Fixed smoothTime,
								  b3Fixed deltaTime )
{
	if ( smoothTime <= 0 )
	{
		return target;
	}

	if ( deltaTime <= 0 )
	{
		return current;
	}

	b3Fixed omega = b3FixDiv( 2 * B3_PI, smoothTime );
	b3Fixed onePlus = B3_FIXED_ONE + b3FixMul( omega, deltaTime );
	b3Fixed denominator = b3FixMul( onePlus, onePlus );

	b3Fixed spring = b3FixMul( b3FixMul( b3FixMul( omega, omega ), deltaTime ), current - target );

	*velocity = b3FixDiv( *velocity - spring, denominator );

	return current + b3FixMul( *velocity, deltaTime );
}

b3Fixed b3SmoothCriticallyDampedUpDown( b3Fixed current, b3Fixed target, b3Fixed* velocity, b3Fixed smoothTimeUp,
										b3Fixed smoothTimeDown, b3Fixed deltaTime )
{
	b3Fixed smoothTime = b3FixAbs( target ) >= b3FixAbs( current ) ? smoothTimeUp : smoothTimeDown;
	return b3SmoothCriticallyDamped( current, target, velocity, smoothTime, deltaTime );
}

b3Vec3 b3SmoothCriticallyDampedVec3( b3Vec3 current, b3Vec3 target, b3Vec3* velocity, b3Fixed smoothTime,
									 b3Fixed deltaTime )
{
	b3Vec3 result;
	result.x = b3SmoothCriticallyDamped( current.x, target.x, &velocity->x, smoothTime, deltaTime );
	result.y = b3SmoothCriticallyDamped( current.y, target.y, &velocity->y, smoothTime, deltaTime );
	result.z = b3SmoothCriticallyDamped( current.z, target.z, &velocity->z, smoothTime, deltaTime );
	return result;
}

b3Vec3 b3SmoothCriticallyDampedUpDownVec3( b3Vec3 current, b3Vec3 target, b3Vec3* velocity, b3Fixed smoothTimeUp,
										   b3Fixed smoothTimeDown, b3Fixed deltaTime )
{
	// ONE smoothing time for the whole vector, selected by length -- not per component
	b3Fixed smoothTime = b3Length( target ) >= b3Length( current ) ? smoothTimeUp : smoothTimeDown;
	return b3SmoothCriticallyDampedVec3( current, target, velocity, smoothTime, deltaTime );
}

// ---------------------------------------------------------------------------------------------
// Random unit quaternion (Marsaglia rejection sampling: sqrt only, no trig)
// ---------------------------------------------------------------------------------------------

b3Quat b3MakeRandomQuat( b3RandomFcn* random, void* context )
{
	b3Fixed x, y, z, u, v, w;

	do
	{
		x = random( context );
		y = random( context );
		z = b3FixMul( x, x ) + b3FixMul( y, y );
	}
	while ( z >= B3_FIXED_ONE || z == 0 );

	do
	{
		u = random( context );
		v = random( context );
		w = b3FixMul( u, u ) + b3FixMul( v, v );
	}
	while ( w >= B3_FIXED_ONE || w == 0 );

	b3Fixed s = b3FixSqrt( b3FixDiv( B3_FIXED_ONE - z, w ) );

	b3Quat result;
	result.v.x = x;
	result.v.y = y;
	result.v.z = b3FixMul( s, u );
	result.s = b3FixMul( s, v );
	return b3NormalizeQuat( result );
}
