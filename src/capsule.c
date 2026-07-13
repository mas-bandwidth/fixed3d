// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "math_internal.h"
#include "shape.h"

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/constants.h"

b3MassData b3ComputeCapsuleMass( const b3Capsule* shape, b3Fixed density )
{
	b3Vec3 c1 = shape->center1;
	b3Vec3 c2 = shape->center2;
	b3Fixed r = shape->radius;

	// Cylinder
	b3Fixed cylinderHeight = b3Distance( c1, c2 );
	b3Fixed cylinderVolume = b3FixMul( b3FixMul( b3FixMul( B3_PI , r ) , r ) , cylinderHeight );
	b3Fixed cylinderMass = b3FixMul( cylinderVolume , density );

	// Sphere
	b3Fixed sphereVolume = b3FixMul( b3FixMul( b3FixMul( b3FixMul( ( b3FixDiv( B3_FIX( 4.0f ) , B3_FIX( 3.0f ) ) ) , B3_PI ) , r ) , r ) , r );
	b3Fixed sphereMass = b3FixMul( sphereVolume , density );

	// Local accumulated inertia
	b3Matrix3 inertia = b3AddMM( b3CylinderInertia( cylinderMass, r, cylinderHeight ), b3SphereInertia( sphereMass, r ) );

	b3Fixed steiner = b3FixMul( b3FixMul( b3FixMul( B3_FIX( 0.125f ) , sphereMass ) , ( b3FixMul( B3_FIX( 3.0f ) , r ) + b3FixMul( B3_FIX( 2.0f ) , cylinderHeight ) ) ) , cylinderHeight );
	inertia.cx.x += steiner;
	inertia.cz.z += steiner;

	// Align capsule axis with chosen up-axis
	b3Matrix3 rotation = b3Mat3_identity;
	if ( cylinderHeight > 0 )
	{
		b3Vec3 direction = b3Normalize( b3Sub( c2, c1 ) );
		b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisY, direction );
		rotation = b3MakeMatrixFromQuat( q );
	}

	b3Fixed mass = sphereMass + cylinderMass;
	b3Vec3 center = b3MulSV( B3_FIX( 0.5f ), b3Add( c1, c2 ) );

	b3MassData out;
	out.mass = mass;
	out.center = center;

	// Rotate the central inertia into the shape frame
	out.inertia = b3MulMM( rotation, b3MulMM( inertia, b3Transpose( rotation ) ) );

	return out;
}

b3AABB b3ComputeCapsuleAABB( const b3Capsule* shape, b3Transform transform )
{
	b3Fixed r = shape->radius;

	b3Vec3 center1 = b3TransformPoint( transform, shape->center1 );
	b3Vec3 center2 = b3TransformPoint( transform, shape->center2 );
	b3Vec3 extent = { r, r, r };

	b3AABB aabb;
	aabb.lowerBound = b3Sub( b3Min( center1, center2 ), extent );
	aabb.upperBound = b3Add( b3Max( center1, center2 ), extent );
	return aabb;
}

b3AABB b3ComputeSweptCapsuleAABB( const b3Capsule* shape, b3Transform xf1, b3Transform xf2 )
{
	b3Vec3 r = { shape->radius, shape->radius, shape->radius };
	b3Vec3 a = b3TransformPoint( xf1, shape->center1 );
	b3Vec3 b = b3TransformPoint( xf1, shape->center2 );
	b3Vec3 c = b3TransformPoint( xf2, shape->center1 );
	b3Vec3 d = b3TransformPoint( xf2, shape->center2 );

	b3AABB aabb = {
		.lowerBound = b3Sub( b3Min( b3Min( a, b ), b3Min( c, d ) ), r ),
		.upperBound = b3Add( b3Max( b3Max( a, b ), b3Max( c, d ) ), r ),
	};
	return aabb;
}

bool b3OverlapCapsule( const b3Capsule* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	b3DistanceInput input;
	input.proxyA = (b3ShapeProxy){ &shape->center1, 2, shape->radius };
	input.proxyB = *proxy;
	input.transform = b3InvMulTransforms( shapeTransform, b3Transform_identity );
	input.useRadii = true;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );
	return output.distance < B3_OVERLAP_SLOP;
}

// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
// http://www.codercorner.com/blog/?p=321
b3CastOutput b3RayCastCapsule( const b3Capsule* shape, const b3RayCastInput* input )
{
	B3_ASSERT( b3IsValidRay( input ) );

	b3Vec3 c1 = shape->center1;
	b3Vec3 c2 = shape->center2;
	b3Fixed r = shape->radius;

	// Initialize result structure
	b3CastOutput output = { 0 };

	b3Vec3 d = b3Sub( c2, c1 );

	// Fall back to sphere if the capsule is short
	b3Fixed tol = b3FixMul( B3_FIX( 0.01f ) , B3_LINEAR_SLOP );
	b3Fixed lengthSquared = b3LengthSquared( d );
	if ( lengthSquared < b3FixMul( tol , tol ) )
	{
		b3Vec3 sphereCenter = b3MulSV( B3_FIX( 0.5f ), b3Add( shape->center1, shape->center2 ) );
		b3Sphere sphere = { sphereCenter, shape->radius };
		return b3RayCastSphere( &sphere, input );
	}

	// A far-away origin carries coordinates whose magnitude swamps the precision of
	// the cylinder quadratic below. Advance the ray to a padded bounding sphere of
	// the capsule first, then solve from the nearby origin with small local values.
	{
		b3Fixed halfLength = b3FixMul( B3_FIX( 0.5f ), b3FixSqrt( lengthSquared ) );
		b3Vec3 mid = b3MulSV( B3_FIX( 0.5f ), b3Add( c1, c2 ) );
		b3Fixed boundRadius = halfLength + r + B3_FIX( 1.0f );
		b3Fixed farThreshold = 8 * boundRadius + B3_FIX( 256.0f );

		if ( b3Length( b3Sub( input->origin, mid ) ) > farThreshold )
		{
			b3Sphere bound = { mid, boundRadius };
			b3CastOutput entry = b3RayCastSphere( &bound, input );
			if ( entry.hit == false )
			{
				// The ray never comes near the capsule
				return output;
			}

			b3Fixed t0 = entry.fraction;
			b3Fixed remaining = B3_FIX( 1.0f ) - t0;
			if ( input->maxFraction <= t0 || remaining <= 0 )
			{
				return output;
			}

			b3RayCastInput advanced;
			advanced.origin = b3MulAdd( input->origin, t0, input->translation );
			advanced.translation = b3MulSV( remaining, input->translation );
			advanced.maxFraction = b3FixMin( b3FixDiv( input->maxFraction - t0, remaining ), B3_FIX( 1.0f ) );

			// Single level of recursion: the advanced origin sits on the bounding
			// sphere, inside the far threshold.
			b3CastOutput nearOut = b3RayCastCapsule( shape, &advanced );
			if ( nearOut.hit )
			{
				nearOut.fraction = b3FixClamp( t0 + b3FixMul( nearOut.fraction, remaining ), B3_FIX( 0.0f ), input->maxFraction );
			}
			return nearOut;
		}
	}

	// Vector from first center to ray origin.
	b3Vec3 s = b3Sub( input->origin, c1 );

	// Capsule axis. b3Normalize divides at full precision; a reciprocal-multiply
	// normalization would cost several ulps that near-parallel rays cannot afford.
	b3Fixed length = b3FixSqrt( lengthSquared );
	b3Vec3 axis = b3Normalize( d );

	// Project ray origin onto capsule axis.
	b3Fixed u = b3Dot( s, axis );

	// Closest point on infinite capsule axis, relative to c1.
	b3Vec3 c = b3MulSV( u, axis );

	// Vector from closest point to ray origin
	b3Vec3 sc = b3Sub( s, c );

	// Squared distance from ray origin to capsule axis
	b3Fixed sc2 = b3LengthSquared( sc );

	// Is the ray origin within the infinite cylinder along the capsule axis?
	if ( sc2 < b3FixMul( r , r ) )
	{
		// Clamped barycentric coordinate of ray origin projected onto capsule axis.
		b3Fixed uClamped = b3FixClamp( u, B3_FIX( 0.0f ), length );

		// The closest point on the bounded capsule segment, relative to c1.
		b3Vec3 cp = b3MulSV( uClamped, axis );

		// Vector from ray origin to closest point on segment.
		b3Vec3 scp = b3Sub( s, cp );

		// Squared distance of ray origin from capsule segment.
		b3Fixed scp2 = b3LengthSquared( scp );

		// Is the ray origin within the capsule?
		if ( scp2 < b3FixMul( r , r ) )
		{
			output.hit = true;
			output.point = input->origin;
			return output;
		}

		// The ray can hit an endcap.
		b3Sphere sphere = {
			.center = b3Add( c1, cp ),
			.radius = r,
		};

		return b3RayCastSphere( &sphere, input );
	}

	// Ray axis. A zero length ray reaching here starts outside the capsule, so it misses.
	// Same zero length convention as b3RayCastSphere.
	b3Vec3 dr = input->translation;
	b3Fixed rayLength;
	b3Vec3 rayAxis = b3GetLengthAndNormalize( &rayLength, dr );
	if ( rayLength == B3_FIX( 0.0f ) )
	{
		return output;
	}

	// Barycentric coordinate of ray end point.
	b3Fixed v = u + b3FixMul( input->maxFraction , b3Dot( dr, axis ) );

	// Early out: does the projected ray fall outside the capsule?
	if ( ( u < -r && v < -r ) || ( length + r < u && length + r < v ) )
	{
		return output;
	}

	// Compute the closest point between the ray segment and the capsule segment.
	// See Real-Time Collision Detection, section 5.1.9

	// Closest point on capsule : a1 = segment unit axis, t1 = unknown fraction
	// p1 = t1 * a1

	// Closet point on ray : a2 = ray unit axis, t2 = unknown fraction
	// p2 = s + t2 * a2

	// Closest point perpendicularity conditions.
	// dot(p2 - p1, a1) = 0
	// dot(p2 - p1, a2) = 0

	// Expand
	// dot(t1 * a1 - s - t2 * a2, a1) = 0
	// dot(t1 * a1 - s - t2 * a2, a2) = 0

	// Expand
	// t1 - dot(s, a1) - t2 * dot(a1, a2) = 0
	// t1 * dot(a1, a2) - dot(s, a2) - t2 = 0

	// Group : a12 = dot(a1, a2), sa1 = dot(s, a1), sa2 = dot(s, a2)
	// t1       - a12 * t2 = sa1
	// a12 * t1 -       t2 = sa2

	// Solve
	// https://en.wikipedia.org/wiki/Cramer%27s_rule
	// I've flipped the signs of the numerator and denominator to give a positive determinant.
	// det = 1 - a12 * a12
	// t1 = (sa1 - a12 * sa2) / det
	// t2 = (a12 * sa1 - sa2) / det

	b3Vec3 a1 = axis;
	b3Vec3 a2 = rayAxis;
	b3Fixed a12 = b3Dot( a1, a2 );

	// Ray distance to the near intersection with the infinite cylinder. Length units.
	b3Fixed tr;

	// Intersect the ray with the infinite cylinder by solving the 2D quadratic in
	// the plane perpendicular to the capsule axis: | sc + t * w |^2 = r^2, where w is
	// the perpendicular part of the raw (unnormalized) translation. This form is
	// valid for every ray direction and, evaluated at 128 bits, keeps precision for
	// near-parallel rays (whose perpendicular drift is below the Q48.16 resolution
	// once normalized) and for far-away origins alike.
	{
		// Perpendicular part of the full translation. Length units.
		b3Fixed drDotAxis = b3Dot( dr, a1 );
		b3Vec3 w = b3MulSub( dr, drDotAxis, a1 );

		// Q32.32 raw products at 128 bits
		b3Int128 perp2 = (b3Int128)w.x * w.x + (b3Int128)w.y * w.y + (b3Int128)w.z * w.z;
		b3Int128 beta = (b3Int128)sc.x * w.x + (b3Int128)sc.y * w.y + (b3Int128)sc.z * w.z;
		b3Int128 gamma = b3Int128ShiftLeft( sc2 - b3FixMul( r, r ), 16 ); // Q32.32

		// Casting away from the axis, or no perpendicular motion at all. The ray
		// origin is outside the infinite cylinder here, so there is no hit.
		if ( beta >= 0 || perp2 == 0 )
		{
			return output;
		}

		// Shift the coefficients into a range where the discriminant fits in 128 bits.
		// The shift cancels in the final ratio.
		b3Int128 limit = (b3Int128)1 << 62;
		while ( perp2 >= limit || -beta >= limit || gamma >= limit )
		{
			perp2 >>= 8;
			beta >>= 8;
			gamma >>= 8;
		}

		if ( perp2 == 0 )
		{
			return output;
		}

		// Discriminant in 128 bits
		b3Int128 disc = beta * (int64_t)beta - perp2 * (int64_t)gamma;
		if ( disc < 0 )
		{
			// The perpendicular gap never closes to the radius.
			return output;
		}

		int64_t sqrtDisc = (int64_t)b3ISqrt128High( (uint64_t)( (b3UInt128)disc >> 64 ), (uint64_t)disc );

		// Quadratic near root as a fraction of the translation. Expressed in an
		// alternate form to avoid the (-beta - sqrt) cancellation for near-parallel rays.
		b3Int128 denom = -beta + sqrtDisc;
		if ( denom <= 0 )
		{
			return output;
		}

		b3Int128 tFraction = b3Int128Div( b3Int128ShiftLeft( gamma, 32 ), denom ); // Q32.32 fraction of dr, shift-independent

		// Convert to length units along the normalized ray for the shared code below
		tr = (b3Fixed)( ( tFraction * rayLength ) >> 32 );
	}

	// Outside ray?
	if ( tr < B3_FIX( 0.0f ) || b3FixMul( input->maxFraction , rayLength ) < tr )
	{
		return output;
	}

	// The corresponding distance on the capsule axis. Length units.
	b3Fixed tc = u + b3FixMul( tr , a12 );

	// Outside c1 end?
	if ( tc < B3_FIX( 0.0f ) )
	{
		// Ray cast sphere 1.
		b3Sphere sphere = {
			.center = c1,
			.radius = r,
		};

		return b3RayCastSphere( &sphere, input );
	}

	// Outside c2 end?
	if (length < tc)
	{
		// Ray cast sphere 2.
		b3Sphere sphere = {
			.center = c2,
			.radius = r,
		};

		return b3RayCastSphere( &sphere, input );
	}

	// Hit point on capsule side, relative to c1.
	b3Vec3 p = b3MulAdd( s, tr, rayAxis );

	// Hit normal.
	b3Vec3 normal = b3MulSub( p, tc, axis );
	normal = b3Normalize( normal );

	output.point = b3Add( c1, p );
	output.normal = normal;
	output.fraction = b3FixClamp( b3FixDiv( tr , rayLength ), B3_FIX( 0.0f ), input->maxFraction );
	output.hit = true;
	return output;
}

b3CastOutput b3ShapeCastCapsule( const b3Capsule* capsule, const b3ShapeCastInput* input )
{
	b3ShapeCastPairInput pairInput;
	pairInput.proxyA = (b3ShapeProxy){ &capsule->center1, 2, capsule->radius };
	pairInput.proxyB = input->proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.maxFraction = input->maxFraction;
	pairInput.canEncroach = input->canEncroach;

	b3CastOutput output = b3ShapeCast( &pairInput );
	return output;
}

int b3CollideMoverAndCapsule( b3PlaneResult* result, const b3Capsule* shape, const b3Capsule* mover )
{
	b3Fixed totalRadius = mover->radius + shape->radius;

	b3SegmentDistanceResult approach = b3SegmentDistance( shape->center1, shape->center2, mover->center1, mover->center2 );

	// The normal points from the shape toward the mover.
	b3Fixed distance;
	b3Vec3 normal = b3GetLengthAndNormalize( &distance, b3Sub( approach.point2, approach.point1 ) );

	if ( distance > totalRadius )
	{
		return 0;
	}

	b3Fixed linearSlop = B3_LINEAR_SLOP;
	if ( distance < linearSlop )
	{
		// Deep overlap: the core segments intersect. Pick an arbitrary direction perpendicular
		// the to capsule axis.
		b3Fixed moverLength;
		b3Vec3 moverAxis = b3GetLengthAndNormalize( &moverLength, b3Sub( mover->center2, mover->center1 ) );
		normal = moverLength > linearSlop ? b3Perp( moverAxis ) : b3Vec3_axisY;
		distance = B3_FIX( 0.0f );
	}

	b3Plane plane = { normal, totalRadius - distance };
	*result = (b3PlaneResult){ plane, approach.point1 };
	return 1;
}
