// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "algorithm.h"
#include "ctz.h"
#include "manifold.h"
#include "shape.h"

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/constants.h"

#include <stdbool.h>
#include <stddef.h>

static int b3ClipSegment( b3ClipVertex segment[2], b3Plane plane )
{
	int vertexCount = 0;
	b3ClipVertex vertex1 = segment[0];
	b3ClipVertex vertex2 = segment[1];

	b3Fixed distance1 = b3PlaneSeparation( plane, vertex1.position );
	b3Fixed distance2 = b3PlaneSeparation( plane, vertex2.position );

	// If the points are behind the plane
	if ( distance1 <= B3_FIX( 0.0f ) )
	{
		segment[vertexCount++] = vertex1;
	}
	if ( distance2 <= B3_FIX( 0.0f ) )
	{
		segment[vertexCount++] = vertex2;
	}

	// If the points are on different sides of the plane
	if ( b3FixMul( distance1 , distance2 ) < B3_FIX( 0.0f ) )
	{
		// Find intersection point of edge and plane
		b3Fixed t = b3FixDiv( distance1 , ( distance1 - distance2 ) );
		segment[vertexCount].position = b3Add( b3MulSV( B3_FIX( 1.0f ) - t, vertex1.position ), b3MulSV( t, vertex2.position ) );
		segment[vertexCount].pair = distance1 > B3_FIX( 0.0f ) ? vertex1.pair : vertex2.pair;
		vertexCount++;
	}

	return vertexCount;
}

static int b3ClipSegmentToHullFace( b3ClipVertex segment[2], const b3HullData* hull, int refFace )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	b3Plane refPlane = planes[refFace];

	const b3HullFace* face = faces + refFace;

	int edgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		b3Vec3 vertex1 = points[edge->origin];
		b3Vec3 vertex2 = points[next->origin];
		b3Vec3 tangent = b3Normalize( b3Sub( vertex2, vertex1 ) );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		int pointCount = b3ClipSegment( segment, b3MakePlaneFromNormalAndPoint( binormal, vertex1 ) );
		if ( pointCount < 2 )
		{
			return 0;
		}

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge );

	return 2;
}

static b3SeparatingAxis b3QueryFaceDirectionHullAndCapsule( const b3HullData* hull, const b3Capsule* capsule,
													   b3Transform capsuleTransform )
{
	int maxFaceIndex = -1;
	int maxVertexIndex = -1;
	b3Fixed maxFaceSeparation = -B3_FIXED_MAX;
	const b3Plane* planes = b3GetHullPlanes( hull );

	b3Vec3 capsulePoints[2] = {
		b3TransformPoint( capsuleTransform, capsule->center1 ),
		b3TransformPoint( capsuleTransform, capsule->center2 ),
	};

	for ( int faceIndex = 0; faceIndex < hull->faceCount; ++faceIndex )
	{
		b3Plane plane = planes[faceIndex];

		int vertexIndex = b3GetPointSupport( capsulePoints, 2, b3Neg( plane.normal ) );
		b3Vec3 support = capsulePoints[vertexIndex];
		b3Fixed separation = b3PlaneSeparation( plane, support );
		if ( separation > maxFaceSeparation )
		{
			maxVertexIndex = vertexIndex;
			maxFaceIndex = faceIndex;
			maxFaceSeparation = separation;
		}
	}

	return (b3SeparatingAxis){
		.normal = planes[maxFaceIndex].normal,
		.separation = maxFaceSeparation,
		.indexA = (uint8_t)maxFaceIndex,
		.indexB = (uint8_t)maxVertexIndex,
	};
}

static b3SeparatingAxis b3QueryFaceDirections( const b3HullData* hullA, const b3HullData* hullB, b3Transform relativeTransform )
{
	// We perform all computations in local space of the second hull
	b3Transform transform = b3InvertTransform( relativeTransform );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	int maxFaceIndex = -1;
	int maxVertexIndex = -1;
	b3Fixed maxFaceSeparation = -B3_FIXED_MAX;

	for ( int faceIndex = 0; faceIndex < hullA->faceCount; ++faceIndex )
	{
		b3Plane plane = b3TransformPlane( transform, planesA[faceIndex] );

		int vertexIndex = b3FindHullSupportVertex( hullB, b3Neg( plane.normal ) );
		b3Vec3 support = pointsB[vertexIndex];
		b3Fixed separation = b3PlaneSeparation( plane, support );
		if ( separation > maxFaceSeparation )
		{
			maxFaceIndex = faceIndex;
			maxVertexIndex = vertexIndex;
			maxFaceSeparation = separation;
		}
	}

	// The normal is hull A's plane normal in hull A's own frame. The type is
	// b3_faceAxisA relative to this function's arguments; flipped call sites
	// re-express the result in the b3_faceAxisB convention themselves.
	return (b3SeparatingAxis){
		.normal = planesA[maxFaceIndex].normal,
		.separation = maxFaceSeparation,
		.indexA = (uint8_t)maxFaceIndex,
		.indexB = (uint8_t)maxVertexIndex,
		.type = b3_faceAxisA,
	};
}

static b3SeparatingAxis b3QueryEdgeDirectionHullAndCapsule( const b3HullData* hull, const b3Capsule* capsule,
													   b3Transform capsuleTransform )
{
	// Find axis of minimum penetration
	b3Vec3 maxNormal = b3Vec3_zero;
	b3Fixed maxSeparation = -B3_FIXED_MAX;
	int maxIndexA = B3_NULL_INDEX;
	int maxIndexB = B3_NULL_INDEX;

	// We perform all computations in local space of the hull
	b3Vec3 pA = b3TransformPoint( capsuleTransform, capsule->center1 );
	b3Vec3 qA = b3TransformPoint( capsuleTransform, capsule->center2 );
	b3Vec3 eA = b3Sub( qA, pA );

	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	b3Fixed squaredTolerance = b3FixMul( B3_FIX( B3_PARALLEL_EDGE_TOL ) , B3_FIX( B3_PARALLEL_EDGE_TOL ) );

	for ( int index = 0; index < hull->edgeCount; index += 2 )
	{
		const b3HullHalfEdge* edge = edges + index;
		const b3HullHalfEdge* twin = edges + index + 1;
		B3_ASSERT( edge->twin == index + 1 && twin->twin == index );

		b3Vec3 qB = points[twin->origin];
		b3Vec3 uB = planes[edge->face].normal;
		b3Vec3 vB = planes[twin->face].normal;

		// An isolated edge (e.g. like in a capsule) defines a circle through the
		// origin on the Gauss map. So testing for overlap between this circle and
		// the arc AB simplifies to a plane test.
		// Exact sign test on raw 128-bit dots: cba * dba < 0 means strictly
		// opposite signs and both non-zero.
		b3Int128 cbaRaw = b3DotRaw( uB, eA );
		b3Int128 dbaRaw = b3DotRaw( vB, eA );

		if ( cbaRaw != 0 && dbaRaw != 0 && ( cbaRaw ^ dbaRaw ) < 0 )
		{
			b3Fixed cba = b3FixFromDotRaw( cbaRaw );
			b3Fixed dba = b3FixFromDotRaw( dbaRaw );

			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if ( b3FixMax( b3FixMul( cba , cba ), b3FixMul( dba , dba ) ) < b3FixMul( squaredTolerance , b3LengthSquared( eA ) ) )
			{
				continue;
			}

			// The intersection of the arcs on the Gauss map is the edge pair axis. Cast the
			// arc of hull B (from uB to vB) against the plane containing the arc of hull A:
			// dot(uB + t * (vB - uB), eA) == 0
			// then
			// t = cba / (cba - dba)
			//
			// The signs of cba and dba differ (Minkowski test), so the division is safe.
			// Computed from the raw dots so the quotient keeps full precision (divide last).
			//
			// The axis generated points from B to A by construction since it lands between
			// two face normals on B. This removes the need to orient the separation axis
			// using the hull centers.
			//
			// The axis is perpendicular to both edges so I can use qA and qB as arbitrary
			// points on edgeA and edgeB to measure the separation.

			b3Fixed t = (b3Fixed)( b3Int128ShiftLeft( cbaRaw, B3_FIXED_FRACTION_BITS ) / ( cbaRaw - dbaRaw ) );
			b3Vec3 axis = b3Lerp( uB, vB, t );
			B3_VALIDATE( b3LengthSquared( axis ) > 0 );
			axis = b3Normalize( axis );
			b3Fixed separation = b3Dot( axis, b3Sub( qA, qB ) );

			if ( separation > maxSeparation )
			{
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching and account for the convex radius later.
				maxNormal = axis;
				maxSeparation = separation;
				maxIndexA = 0;
				maxIndexB = index;
			}
		}
	}

	// Save result
	return (b3SeparatingAxis){
		.normal = maxNormal,
		.separation = maxSeparation,
		.indexA = maxIndexA,
		.indexB = maxIndexB,
	};
}

// Test one candidate edge pair. Shared by the scalar scan and the wide reject
// path's survivors so the admission criteria and separation update cannot
// diverge between them.
static inline void b3TestEdgePair( const b3HullHalfEdge* edgesA, const b3Vec3* pointsA, const b3Plane* planesA, int indexA,
								   b3Vec3 eB, b3Vec3 qB, b3Vec3 uB, b3Vec3 vB, b3Fixed squaredTolerance, int indexB,
								   b3SeparatingAxis* best )
{
	const b3HullHalfEdge* edgeA = edgesA + indexA;
	const b3HullHalfEdge* twinA = edgesA + indexA + 1;
	B3_ASSERT( edgeA->twin == indexA + 1 && twinA->twin == indexA );

	b3Vec3 qA = pointsA[twinA->origin];
	b3Vec3 eA = b3Sub( qA, pointsA[edgeA->origin] );

	// See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
	// Two edges build a face on the Minkowski sum if the associated arcs AB and CD intersect on the Gauss map.
	// The associated arcs are defined by the adjacent face normals of each edge.

	// These are signed volumes with an edge optimization to avoid cross products
	// eA parallel to cross(vA, uA)
	// eB parallel to cross(vB, uB)
	// Since only signs are tested, length doesn't matter.

	// The Minkowski sign tests run on raw 128-bit dots: exact signs with no
	// rounding, no saturation, and no fixed-point product just to test a sign.
	// cba * dba < 0 means strictly opposite signs and both non-zero.
	b3Int128 cbaRaw = b3DotRaw( uB, eA );
	b3Int128 dbaRaw = b3DotRaw( vB, eA );
	if ( cbaRaw == 0 || dbaRaw == 0 || ( cbaRaw ^ dbaRaw ) >= 0 )
	{
		return;
	}

	b3Vec3 uA = planesA[edgeA->face].normal;
	b3Vec3 vA = planesA[twinA->face].normal;
	b3Int128 adcRaw = -b3DotRaw( uA, eB );
	b3Int128 bdcRaw = -b3DotRaw( vA, eB );

	// adc * bdc < 0 and cba * bdc > 0
	if ( adcRaw == 0 || bdcRaw == 0 || ( adcRaw ^ bdcRaw ) >= 0 || ( cbaRaw ^ bdcRaw ) < 0 )
	{
		return;
	}

	b3Fixed cba = b3FixFromDotRaw( cbaRaw );
	b3Fixed dba = b3FixFromDotRaw( dbaRaw );

	// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
	if ( b3FixMax( b3FixMul( cba , cba ), b3FixMul( dba , dba ) ) < b3FixMul( squaredTolerance , b3LengthSquared( eA ) ) )
	{
		return;
	}

	// The intersection of the arcs on the Gauss map is the edge pair axis. Cast the
	// arc of hull B (from uB to vB) against the plane containing the arc of hull A:
	// dot(uB + t * (vB - uB), eA) == 0
	// then
	// t = cba / (cba - dba)
	//
	// The signs of cba and dba differ (Minkowski test), so the division is safe.
	// Computed from the raw dots so the quotient keeps full precision (divide last).
	//
	// The axis generated points from B to A by construction since it lands between
	// two face normals on B. This removes the need to orient the separation axis
	// using the hull centers.
	//
	// The axis is perpendicular to both edges so I can use qA and qB as arbitrary
	// points on edgeA and edgeB to measure the separation.
	b3Fixed t = (b3Fixed)( b3Int128ShiftLeft( cbaRaw, B3_FIXED_FRACTION_BITS ) / ( cbaRaw - dbaRaw ) );
	b3Vec3 axis = b3Lerp( uB, vB, t );
	B3_VALIDATE( b3LengthSquared( axis ) > 0 );
	axis = b3Normalize( axis );
	b3Fixed separation = b3FixFromDotRaw( b3DotRaw( axis, b3Sub( qA, qB ) ) );

	if ( separation > best->separation )
	{
		// Continues to find the maximum separating axis
		// Flip normal so it points from A to B
		best->normal = b3Neg( axis );
		best->separation = separation;
		best->indexA = indexA;
		best->indexB = indexB;
	}
}

#if defined( B3_SIMD_AVX512 ) || defined( B3_SIMD_NEON )

#if defined( B3_SIMD_AVX512 )
#include <immintrin.h>
// AVX-512 multiplies 64-bit lanes directly
typedef b3Fixed b3EdgeLane;
#else
#include <arm_neon.h>
// NEON has only 32x32->64 widening multiplies, so the SoA staging narrows to
// int32; the admission check below rejects anything that would not fit and
// the truncated values are then never read
typedef int32_t b3EdgeLane;
#endif

// Hulls index edges with uint8_t, so at most 256 half-edges = 128 edge pairs
#define B3_MAX_HULL_EDGE_PAIRS 128

static inline uint64_t b3EdgeAbsBoundU64( b3Fixed a )
{
	return a < 0 ? ( 0 - (uint64_t)a ) : (uint64_t)a;
}

// The wide reject is exact only when every dot stays inside int64: the
// 3 * bound guards keep the 128-bit products from wrapping when the operand
// bounds approach 2^64 (edgeBound is a sum of two 63-bit magnitudes). NEON
// additionally requires every operand to fit in int32 for smull/smlal.
static inline bool b3EdgeWideAdmissible( uint64_t uvMax, uint64_t edgeBound, uint64_t normalBoundA, uint64_t eBMax )
{
#if defined( B3_SIMD_NEON )
	const uint64_t int32Limit = (uint64_t)1 << 31;
	if ( uvMax >= int32Limit || edgeBound >= int32Limit || normalBoundA >= int32Limit || eBMax >= int32Limit )
	{
		return false;
	}
#endif
	return uvMax <= UINT64_MAX / 3 && (b3UInt128)( 3 * uvMax ) * edgeBound < ( (b3UInt128)1 << 63 ) &&
		   normalBoundA <= UINT64_MAX / 3 && (b3UInt128)( 3 * normalBoundA ) * eBMax < ( (b3UInt128)1 << 63 );
}

#endif

static b3SeparatingAxis b3QueryEdgeDirections( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA )
{
	// Find axis of minimum penetration
	b3SeparatingAxis best = {
		.normal = b3Vec3_zero,
		.separation = -B3_FIXED_MAX,
		.indexA = B3_NULL_INDEX,
		.indexB = B3_NULL_INDEX,
		.type = b3_edgePairAxis,
	};

	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );
	const b3Plane* planesB = b3GetHullPlanes( hullB );

	// Work in frame A
	b3Matrix3 matrix = b3MakeMatrixFromQuat( transformBtoA.q );

	b3Fixed squaredTolerance = b3FixMul( B3_FIX( B3_PARALLEL_EDGE_TOL ) , B3_FIX( B3_PARALLEL_EDGE_TOL ) );

#if defined( B3_SIMD_AVX512 ) || defined( B3_SIMD_NEON )
	// Precompute hull A's edge vectors once, SoA: they are reused for every
	// edge of B and four pairs get the Minkowski reject test per iteration.
	// Nearly every pair dies on that first sign test, so the wide loop only
	// computes the two dots and a mask; the rare survivors take the exact
	// scalar path above in ascending index order, which keeps the separation
	// updates in the same order as the scalar scan.
	int pairCountA = hullA->edgeCount / 2;
	B3_ASSERT( pairCountA <= B3_MAX_HULL_EDGE_PAIRS );
	b3EdgeLane eAx[B3_MAX_HULL_EDGE_PAIRS], eAy[B3_MAX_HULL_EDGE_PAIRS], eAz[B3_MAX_HULL_EDGE_PAIRS];
	for ( int pair = 0; pair < pairCountA; ++pair )
	{
		const b3HullHalfEdge* edgeA = edgesA + 2 * pair;
		const b3HullHalfEdge* twinA = edgesA + 2 * pair + 1;
		b3Vec3 eA = b3Sub( pointsA[twinA->origin], pointsA[edgeA->origin] );
		eAx[pair] = (b3EdgeLane)eA.x;
		eAy[pair] = (b3EdgeLane)eA.y;
		eAz[pair] = (b3EdgeLane)eA.z;
	}

	// Hull A's adjacent face normals per edge pair, SoA, for the wide second
	// Minkowski test, and their magnitude bound for its exactness gate
	b3EdgeLane uAx[B3_MAX_HULL_EDGE_PAIRS], uAy[B3_MAX_HULL_EDGE_PAIRS], uAz[B3_MAX_HULL_EDGE_PAIRS];
	b3EdgeLane vAx[B3_MAX_HULL_EDGE_PAIRS], vAy[B3_MAX_HULL_EDGE_PAIRS], vAz[B3_MAX_HULL_EDGE_PAIRS];
	uint64_t normalBoundA = 0;
	for ( int pair = 0; pair < pairCountA; ++pair )
	{
		b3Vec3 uA = planesA[edgesA[2 * pair].face].normal;
		b3Vec3 vA = planesA[edgesA[2 * pair + 1].face].normal;
		uAx[pair] = (b3EdgeLane)uA.x;
		uAy[pair] = (b3EdgeLane)uA.y;
		uAz[pair] = (b3EdgeLane)uA.z;
		vAx[pair] = (b3EdgeLane)vA.x;
		vAy[pair] = (b3EdgeLane)vA.y;
		vAz[pair] = (b3EdgeLane)vA.z;
		uint64_t candidates[6] = { b3EdgeAbsBoundU64( uA.x ), b3EdgeAbsBoundU64( uA.y ), b3EdgeAbsBoundU64( uA.z ),
								   b3EdgeAbsBoundU64( vA.x ), b3EdgeAbsBoundU64( vA.y ), b3EdgeAbsBoundU64( vA.z ) };
		for ( int i = 0; i < 6; ++i )
		{
			normalBoundA = candidates[i] > normalBoundA ? candidates[i] : normalBoundA;
		}
	}

	// |edge component| <= |aabb.lower| + |aabb.upper| for a difference of two
	// hull points, so the int64 dot bound below is sound for valid hull data
	uint64_t edgeBound = 0;
	{
		uint64_t candidates[3] = {
			b3EdgeAbsBoundU64( hullA->aabb.lowerBound.x ) + b3EdgeAbsBoundU64( hullA->aabb.upperBound.x ),
			b3EdgeAbsBoundU64( hullA->aabb.lowerBound.y ) + b3EdgeAbsBoundU64( hullA->aabb.upperBound.y ),
			b3EdgeAbsBoundU64( hullA->aabb.lowerBound.z ) + b3EdgeAbsBoundU64( hullA->aabb.upperBound.z ),
		};
		for ( int i = 0; i < 3; ++i )
		{
			edgeBound = candidates[i] > edgeBound ? candidates[i] : edgeBound;
		}
	}
#endif

	// Arranged to minimize transform operations
	for ( int indexB = 0; indexB < hullB->edgeCount; indexB += 2 )
	{
		const b3HullHalfEdge* edgeB = edgesB + indexB;
		const b3HullHalfEdge* twinB = edgesB + indexB + 1;
		B3_ASSERT( edgeB->twin == indexB + 1 && twinB->twin == indexB );

		b3Vec3 qB = pointsB[twinB->origin];
		b3Vec3 eB = b3MulMV( matrix, b3Sub( qB, pointsB[edgeB->origin] ) );
		qB = b3Add( b3MulMV( matrix, qB ), transformBtoA.p );

		b3Vec3 uB = b3MulMV( matrix, planesB[edgeB->face].normal );
		b3Vec3 vB = b3MulMV( matrix, planesB[twinB->face].normal );

#if defined( B3_SIMD_AVX512 ) || defined( B3_SIMD_NEON )
		// Gate on the actual rotated normals against the edge bound
		uint64_t uvMax = b3EdgeAbsBoundU64( uB.x );
		{
			uint64_t candidates[5] = { b3EdgeAbsBoundU64( uB.y ), b3EdgeAbsBoundU64( uB.z ), b3EdgeAbsBoundU64( vB.x ),
									   b3EdgeAbsBoundU64( vB.y ), b3EdgeAbsBoundU64( vB.z ) };
			for ( int i = 0; i < 5; ++i )
			{
				uvMax = candidates[i] > uvMax ? candidates[i] : uvMax;
			}
		}

		// The wide second test needs the dots of hull A's normals with eB to
		// stay exact as well
		uint64_t eBMax = b3EdgeAbsBoundU64( eB.x );
		{
			uint64_t candidates[2] = { b3EdgeAbsBoundU64( eB.y ), b3EdgeAbsBoundU64( eB.z ) };
			eBMax = candidates[0] > eBMax ? candidates[0] : eBMax;
			eBMax = candidates[1] > eBMax ? candidates[1] : eBMax;
		}

		if ( pairCountA >= 4 && b3EdgeWideAdmissible( uvMax, edgeBound, normalBoundA, eBMax ) )
		{
			int pair = 0;
#if defined( B3_SIMD_AVX512 )
			__m256i ubx = _mm256_set1_epi64x( uB.x );
			__m256i uby = _mm256_set1_epi64x( uB.y );
			__m256i ubz = _mm256_set1_epi64x( uB.z );
			__m256i vbx = _mm256_set1_epi64x( vB.x );
			__m256i vby = _mm256_set1_epi64x( vB.y );
			__m256i vbz = _mm256_set1_epi64x( vB.z );
			__m256i ebx = _mm256_set1_epi64x( eB.x );
			__m256i eby = _mm256_set1_epi64x( eB.y );
			__m256i ebz = _mm256_set1_epi64x( eB.z );
			__m256i zero = _mm256_setzero_si256();

			for ( ; pair + 4 <= pairCountA; pair += 4 )
			{
				__m256i ex = _mm256_loadu_si256( (const __m256i*)( eAx + pair ) );
				__m256i ey = _mm256_loadu_si256( (const __m256i*)( eAy + pair ) );
				__m256i ez = _mm256_loadu_si256( (const __m256i*)( eAz + pair ) );

				__m256i cba = _mm256_add_epi64(
					_mm256_add_epi64( _mm256_mullo_epi64( ubx, ex ), _mm256_mullo_epi64( uby, ey ) ),
					_mm256_mullo_epi64( ubz, ez ) );
				__m256i dba = _mm256_add_epi64(
					_mm256_add_epi64( _mm256_mullo_epi64( vbx, ex ), _mm256_mullo_epi64( vby, ey ) ),
					_mm256_mullo_epi64( vbz, ez ) );

				// First Minkowski test: survive when cba != 0 and dba != 0
				// and signs differ
				__mmask8 nzc = _mm256_test_epi64_mask( cba, cba );
				__mmask8 nzd = _mm256_test_epi64_mask( dba, dba );
				__mmask8 opposite = _mm256_cmpgt_epi64_mask( zero, _mm256_xor_si256( cba, dba ) );

				// Second test on the un-negated dots: with dA = dot(uA, eB)
				// and dB = dot(vA, eB), the scalar adc = -dA / bdc = -dB
				// conditions become dA != 0, dB != 0, (dA ^ dB) < 0, and
				// (cba ^ dB) < 0 (bdc flips dB's sign, so "same sign as cba"
				// becomes "opposite sign")
				__m256i nax = _mm256_loadu_si256( (const __m256i*)( uAx + pair ) );
				__m256i nay = _mm256_loadu_si256( (const __m256i*)( uAy + pair ) );
				__m256i naz = _mm256_loadu_si256( (const __m256i*)( uAz + pair ) );
				__m256i nbx = _mm256_loadu_si256( (const __m256i*)( vAx + pair ) );
				__m256i nby = _mm256_loadu_si256( (const __m256i*)( vAy + pair ) );
				__m256i nbz = _mm256_loadu_si256( (const __m256i*)( vAz + pair ) );

				__m256i dA = _mm256_add_epi64(
					_mm256_add_epi64( _mm256_mullo_epi64( nax, ebx ), _mm256_mullo_epi64( nay, eby ) ),
					_mm256_mullo_epi64( naz, ebz ) );
				__m256i dB = _mm256_add_epi64(
					_mm256_add_epi64( _mm256_mullo_epi64( nbx, ebx ), _mm256_mullo_epi64( nby, eby ) ),
					_mm256_mullo_epi64( nbz, ebz ) );

				__mmask8 nzA = _mm256_test_epi64_mask( dA, dA );
				__mmask8 nzB = _mm256_test_epi64_mask( dB, dB );
				__mmask8 oppositeAB = _mm256_cmpgt_epi64_mask( zero, _mm256_xor_si256( dA, dB ) );
				__mmask8 cbaOppB = _mm256_cmpgt_epi64_mask( zero, _mm256_xor_si256( cba, dB ) );

				unsigned survivors = (unsigned)( nzc & nzd & opposite & nzA & nzB & oppositeAB & cbaOppB ) & 0xF;

				while ( survivors != 0 )
				{
					int lane = (int)b3CTZ32( survivors );
					survivors &= survivors - 1;
					b3TestEdgePair( edgesA, pointsA, planesA, 2 * ( pair + lane ), eB, qB, uB, vB, squaredTolerance,
									indexB, &best );
				}
			}
#else
			// NEON: four pairs per iteration with smull/smlal widening
			// multiplies over the int32 SoA staging; the admission check
			// proved every dot is int64-exact, so both Minkowski sign tests
			// resolve identically to the scalar 128-bit dots
			int32x4_t ubx4 = vdupq_n_s32( (int32_t)uB.x );
			int32x4_t uby4 = vdupq_n_s32( (int32_t)uB.y );
			int32x4_t ubz4 = vdupq_n_s32( (int32_t)uB.z );
			int32x4_t vbx4 = vdupq_n_s32( (int32_t)vB.x );
			int32x4_t vby4 = vdupq_n_s32( (int32_t)vB.y );
			int32x4_t vbz4 = vdupq_n_s32( (int32_t)vB.z );
			int32x4_t ebx4 = vdupq_n_s32( (int32_t)eB.x );
			int32x4_t eby4 = vdupq_n_s32( (int32_t)eB.y );
			int32x4_t ebz4 = vdupq_n_s32( (int32_t)eB.z );

			for ( ; pair + 4 <= pairCountA; pair += 4 )
			{
				int32x4_t ex = vld1q_s32( eAx + pair );
				int32x4_t ey = vld1q_s32( eAy + pair );
				int32x4_t ez = vld1q_s32( eAz + pair );

				int64x2_t cbaLo = vmull_s32( vget_low_s32( ex ), vget_low_s32( ubx4 ) );
				cbaLo = vmlal_s32( cbaLo, vget_low_s32( ey ), vget_low_s32( uby4 ) );
				cbaLo = vmlal_s32( cbaLo, vget_low_s32( ez ), vget_low_s32( ubz4 ) );
				int64x2_t cbaHi = vmull_high_s32( ex, ubx4 );
				cbaHi = vmlal_high_s32( cbaHi, ey, uby4 );
				cbaHi = vmlal_high_s32( cbaHi, ez, ubz4 );

				int64x2_t dbaLo = vmull_s32( vget_low_s32( ex ), vget_low_s32( vbx4 ) );
				dbaLo = vmlal_s32( dbaLo, vget_low_s32( ey ), vget_low_s32( vby4 ) );
				dbaLo = vmlal_s32( dbaLo, vget_low_s32( ez ), vget_low_s32( vbz4 ) );
				int64x2_t dbaHi = vmull_high_s32( ex, vbx4 );
				dbaHi = vmlal_high_s32( dbaHi, ey, vby4 );
				dbaHi = vmlal_high_s32( dbaHi, ez, vbz4 );

				int32x4_t nax = vld1q_s32( uAx + pair );
				int32x4_t nay = vld1q_s32( uAy + pair );
				int32x4_t naz = vld1q_s32( uAz + pair );
				int32x4_t nbx = vld1q_s32( vAx + pair );
				int32x4_t nby = vld1q_s32( vAy + pair );
				int32x4_t nbz = vld1q_s32( vAz + pair );

				int64x2_t dALo = vmull_s32( vget_low_s32( nax ), vget_low_s32( ebx4 ) );
				dALo = vmlal_s32( dALo, vget_low_s32( nay ), vget_low_s32( eby4 ) );
				dALo = vmlal_s32( dALo, vget_low_s32( naz ), vget_low_s32( ebz4 ) );
				int64x2_t dAHi = vmull_high_s32( nax, ebx4 );
				dAHi = vmlal_high_s32( dAHi, nay, eby4 );
				dAHi = vmlal_high_s32( dAHi, naz, ebz4 );

				int64x2_t dBLo = vmull_s32( vget_low_s32( nbx ), vget_low_s32( ebx4 ) );
				dBLo = vmlal_s32( dBLo, vget_low_s32( nby ), vget_low_s32( eby4 ) );
				dBLo = vmlal_s32( dBLo, vget_low_s32( nbz ), vget_low_s32( ebz4 ) );
				int64x2_t dBHi = vmull_high_s32( nbx, ebx4 );
				dBHi = vmlal_high_s32( dBHi, nby, eby4 );
				dBHi = vmlal_high_s32( dBHi, nbz, ebz4 );

				// First test: cba != 0, dba != 0, opposite signs. Second test
				// on the un-negated dots: dA != 0, dB != 0, (dA ^ dB) < 0,
				// (cba ^ dB) < 0 (the scalar adc/bdc negation flips signs).
				uint64x2_t sLo = vbicq_u64( vcltzq_s64( veorq_s64( cbaLo, dbaLo ) ), vceqzq_s64( cbaLo ) );
				sLo = vbicq_u64( sLo, vceqzq_s64( dbaLo ) );
				uint64x2_t s2Lo = vbicq_u64( vcltzq_s64( veorq_s64( dALo, dBLo ) ), vceqzq_s64( dALo ) );
				s2Lo = vbicq_u64( s2Lo, vceqzq_s64( dBLo ) );
				s2Lo = vandq_u64( s2Lo, vcltzq_s64( veorq_s64( cbaLo, dBLo ) ) );
				sLo = vandq_u64( sLo, s2Lo );

				uint64x2_t sHi = vbicq_u64( vcltzq_s64( veorq_s64( cbaHi, dbaHi ) ), vceqzq_s64( cbaHi ) );
				sHi = vbicq_u64( sHi, vceqzq_s64( dbaHi ) );
				uint64x2_t s2Hi = vbicq_u64( vcltzq_s64( veorq_s64( dAHi, dBHi ) ), vceqzq_s64( dAHi ) );
				s2Hi = vbicq_u64( s2Hi, vceqzq_s64( dBHi ) );
				s2Hi = vandq_u64( s2Hi, vcltzq_s64( veorq_s64( cbaHi, dBHi ) ) );
				sHi = vandq_u64( sHi, s2Hi );

				uint64x2_t any = vorrq_u64( sLo, sHi );
				if ( ( vgetq_lane_u64( any, 0 ) | vgetq_lane_u64( any, 1 ) ) != 0 )
				{
					// Survivors in ascending index order, matching the scalar
					// separation update sequence
					if ( vgetq_lane_u64( sLo, 0 ) != 0 )
					{
						b3TestEdgePair( edgesA, pointsA, planesA, 2 * ( pair + 0 ), eB, qB, uB, vB, squaredTolerance,
										indexB, &best );
					}
					if ( vgetq_lane_u64( sLo, 1 ) != 0 )
					{
						b3TestEdgePair( edgesA, pointsA, planesA, 2 * ( pair + 1 ), eB, qB, uB, vB, squaredTolerance,
										indexB, &best );
					}
					if ( vgetq_lane_u64( sHi, 0 ) != 0 )
					{
						b3TestEdgePair( edgesA, pointsA, planesA, 2 * ( pair + 2 ), eB, qB, uB, vB, squaredTolerance,
										indexB, &best );
					}
					if ( vgetq_lane_u64( sHi, 1 ) != 0 )
					{
						b3TestEdgePair( edgesA, pointsA, planesA, 2 * ( pair + 3 ), eB, qB, uB, vB, squaredTolerance,
										indexB, &best );
					}
				}
			}
#endif

			for ( ; pair < pairCountA; ++pair )
			{
				b3TestEdgePair( edgesA, pointsA, planesA, 2 * pair, eB, qB, uB, vB, squaredTolerance, indexB, &best );
			}

			continue;
		}
#endif

		for ( int indexA = 0; indexA < hullA->edgeCount; indexA += 2 )
		{
			b3TestEdgePair( edgesA, pointsA, planesA, indexA, eB, qB, uB, vB, squaredTolerance, indexB, &best );
		}
	}

	return best;
}

// Reduce the manifold points to a maximum of 4 points.
// Note: this modifies the input point array to improve performance
static void b3ReduceManifoldPoints( b3LocalManifold* manifold, int capacity, b3LocalManifoldPoint* points, int count )
{
	if ( capacity < 4 )
	{
		return;
	}

	if ( count <= 4 )
	{
		for ( int i = 0; i < count; ++i )
		{
			manifold->points[i] = points[i];
		}

		manifold->pointCount = count;
		return;
	}

	b3Vec3 normal = manifold->normal;
	// b3Fixed linearSlop = B3_LINEAR_SLOP;
	b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;
	b3Fixed tolSqr = b3FixMul( speculativeDistance , speculativeDistance );

	// This bias is very important for contact point consistency across time steps.
	// It creates a pecking order to avoid flickering between candidates with similar scores.
	b3Fixed bias = B3_FIX( 0.95f );

	// Step 1: find extreme point that is touching
	int bestIndex = B3_NULL_INDEX;
	b3Fixed bestScore = -B3_FIXED_MAX;

	// Arbitrary tangent direction
	// b3Vec3 perp1 = b3Perp( normal );
	// b3Vec3 perp2 = b3Cross( perp1, normal );
	// b3Vec3 searchDirection = -0.4535961214255773f * perp1 + 0.8912073600614354f * perp2;
	b3Vec3 searchDirection = b3ArbitraryPerp( normal );
	for ( int index = 0; index < count; ++index )
	{
		b3LocalManifoldPoint* pt = points + index;

		if ( pt->separation > speculativeDistance )
		{
			continue;
		}

		// The deeper the better
		b3Fixed score = -pt->separation + b3Dot( searchDirection, pt->point );
		if ( b3FixMul( bias , score ) > bestScore )
		{
			bestIndex = index;
			bestScore = score;
		}
	}

	B3_VALIDATE( 0 <= bestIndex && bestIndex < count );
	if ( bestIndex == B3_NULL_INDEX )
	{
		manifold->pointCount = 0;
		return;
	}

	manifold->points[0] = points[bestIndex];
	manifold->pointCount = 1;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 a = manifold->points[0].point;

	// Step 2: Find farthest point in 2D
	bestScore = B3_FIX( 0.0f );
	bestIndex = B3_NULL_INDEX;

	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		b3Vec3 d = b3Sub( p, a );
		b3Vec3 v = b3MulSub( d, b3Dot( d, normal ), normal );
		b3Fixed distanceSquared = b3LengthSquared( v );
		b3Fixed separation = b3FixMax( B3_FIX( 0.0f ), -points[index].separation );
		b3Fixed score = distanceSquared + b3FixMul( b3FixMul( B3_FIX( 4.0f ) , separation ) , separation );
		if ( b3FixMul( bias , score ) > bestScore )
		{
			bestScore = score;
			bestIndex = index;
		}
	}

	if ( bestScore < tolSqr )
	{
		return;
	}

	B3_ASSERT( 0 <= bestIndex && bestIndex < count );
	manifold->points[1] = points[bestIndex];
	manifold->pointCount = 2;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 b = manifold->points[1].point;

	// Step 3: Find the point with the maximum triangular area
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	b3Fixed bestSignedArea = B3_FIX( 0.0f );
	b3Vec3 ba = b3Sub( b, a );
	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		b3Fixed signedArea = b3Dot( normal, b3Cross( ba, b3Sub( p, a ) ) );
		b3Fixed score = b3FixAbs( signedArea );
		if ( b3FixMul( bias , score ) >= bestScore )
		{
			bestScore = score;
			bestIndex = index;
			bestSignedArea = signedArea;
		}
	}

	if ( bestIndex == B3_NULL_INDEX )
	{
		return;
	}

	B3_ASSERT( bestIndex != B3_NULL_INDEX );

	manifold->points[2] = points[bestIndex];
	manifold->pointCount = 3;
	points[bestIndex] = points[count - 1];
	count -= 1;

	b3Vec3 c = manifold->points[2].point;

	// Step 4: get the point that adds the most area outside the current triangle
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	b3Fixed sign = bestSignedArea < B3_FIX( 0.0f ) ? -B3_FIX( 1.0f ) : B3_FIX( 1.0f );
	for ( int index = 0; index < count; ++index )
	{
		b3Vec3 p = points[index].point;
		b3Fixed u1 = b3FixMul( sign , b3Dot( normal, b3Cross( b3Sub( p, a ), ba ) ) );
		b3Fixed u2 = b3FixMul( sign , b3Dot( normal, b3Cross( b3Sub( p, b ), b3Sub( c, b ) ) ) );
		b3Fixed u3 = b3FixMul( sign , b3Dot( normal, b3Cross( b3Sub( p, c ), b3Sub( a, c ) ) ) );
		b3Fixed score = b3FixMax( u1, b3FixMax( u2, u3 ) );

		if ( b3FixMul( bias , score ) > bestScore )
		{
			bestScore = score;
			bestIndex = index;
		}
	}

	if ( bestIndex != B3_NULL_INDEX )
	{
		manifold->points[manifold->pointCount] = points[bestIndex];
		manifold->pointCount += 1;
	}
}

void b3CollideSpheres( b3LocalManifold* manifold, int capacity, const b3Sphere* sphereA, const b3Sphere* sphereB,
					   b3Transform transformBtoA )
{
	if ( capacity == 0 )
	{
		return;
	}

	// Work in shapeB coordinates
	b3Vec3 center1 = sphereA->center;
	b3Vec3 center2 = b3TransformPoint( transformBtoA, sphereB->center );

	b3Fixed totalRadius = sphereA->radius + sphereB->radius;
	b3Vec3 offset = b3Sub( center2, center1 );
	b3Fixed distanceSq = b3LengthSquared( offset );

	if ( distanceSq > b3FixMul( totalRadius , totalRadius ) )
	{
		// We found a separating axis
		return;
	}

	b3Vec3 normal = { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	b3Fixed distance = b3FixSqrt( distanceSq );
	if ( distance > 0 )
	{
		normal = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , distance ), offset );
	}

	// Contact at the midpoint
	// 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
	b3Vec3 point =
		b3MulSV( B3_FIX( 0.5f ), b3MulSub( b3Add( b3MulAdd( center1, sphereA->radius, normal ), center2 ), sphereB->radius, normal ) );

	// Manifold in frame B
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = distance - totalRadius;
	pt->pair = b3FeaturePair_single;
}

void b3CollideCapsuleAndSphere( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Sphere* sphereB,
								b3Transform transformBtoA )
{
	manifold->pointCount = 0;

	if ( capacity < 1 )
	{
		return;
	}

	// Work in shape B coordinates
	b3Vec3 center = b3TransformPoint( transformBtoA, sphereB->center );
	b3Vec3 center1 = capsuleA->center1;
	b3Vec3 center2 = capsuleA->center2;

	b3Fixed totalRadius = sphereB->radius + capsuleA->radius;

	b3Vec3 closestPoint = b3PointToSegmentDistance( center1, center2, center );
	b3Vec3 offset = b3Sub( center, closestPoint );
	b3Fixed distanceSq = b3LengthSquared( offset );

	if ( distanceSq > b3FixMul( totalRadius , totalRadius ) )
	{
		// We found a separating axis
		return;
	}

	b3Vec3 normal = { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	b3Fixed distance = b3FixSqrt( distanceSq );
	if ( distance > 0 )
	{
		normal = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , distance ), offset );
	}

	// Contact at the midpoint
	// 0.5 * (((center - sB*n) + closestPoint) + cA*n)
	b3Vec3 point =
		b3MulSV( B3_FIX( 0.5f ), b3MulAdd( b3Add( b3MulSub( center, sphereB->radius, normal ), closestPoint ), capsuleA->radius, normal ) );

	// Manifold in frame B
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = distance - totalRadius;
	pt->pair = b3FeaturePair_single;
}

void b3CollideHullAndSphere( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3Sphere* sphereB,
							 b3Transform transformBtoA, b3SimplexCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity == 0 )
	{
		return;
	}

	b3Vec3 center = b3TransformPoint( transformBtoA, sphereB->center );

	const b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;

	// Work in shapeA coordinates

	b3DistanceInput distanceInput;
	distanceInput.proxyA = (b3ShapeProxy){ b3GetHullPoints( hullA ), hullA->vertexCount, B3_FIX( 0.0f ) };
	distanceInput.proxyB = (b3ShapeProxy){ &center, 1, B3_FIX( 0.0f ) };
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	b3Fixed radiusA = B3_FIX( 0.0f );
	b3Fixed radiusB = sphereB->radius;
	b3Fixed radius = radiusA + radiusB;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, NULL, 0 );

	if ( distanceOutput.distance > radius + speculativeDistance )
	{
		// We found a separating axis
		*cache = (b3SimplexCache){ 0 };
		return;
	}

	if ( distanceOutput.distance > 100 * B3_FIXED_EPSILON )
	{
		// Shallow penetration
		b3Vec3 normal = b3Normalize( b3Sub( distanceOutput.pointB, distanceOutput.pointA ) );

		// cA is the projection of the sphere center onto to the hull (pointA if radiusA == 0).
		b3Vec3 cA = b3MulAdd( center, radiusA - b3Dot( b3Sub( center, distanceOutput.pointA ), normal ), normal );

		// cB is the deepest point on the sphere with respect to the reference f
		b3Vec3 cB = b3MulSub( center, radiusB, normal );

		b3Vec3 point = b3Lerp( cA, cB, B3_FIX( 0.5f ) );

		// Manifold in frame A
		manifold->normal = normal;
		manifold->pointCount = 1;

		b3LocalManifoldPoint* pt = manifold->points + 0;
		pt->point = point;
		pt->separation = distanceOutput.distance - radius;
		pt->pair = b3FeaturePair_single;
	}
	else
	{
		// Deep penetration
		int bestIndex = -1;
		b3Fixed bestDistance = -B3_FIXED_MAX;
		const b3Plane* planes = b3GetHullPlanes( hullA );

		for ( int index = 0; index < hullA->faceCount; ++index )
		{
			b3Plane plane = planes[index];

			b3Fixed distance = b3PlaneSeparation( plane, center );
			if ( distance > bestDistance )
			{
				bestIndex = index;
				bestDistance = distance;
			}
		}
		B3_ASSERT( bestIndex >= 0 );

		b3Vec3 normal = planes[bestIndex].normal;

		// cA is the projection of the sphere center onto to the hull
		b3Vec3 cA = b3MulAdd( center, radiusA - b3Dot( b3Sub( center, distanceOutput.pointA ), normal ), normal );

		// cB is the deepest point on the sphere with respect to the reference f
		b3Vec3 cB = b3MulSub( center, radiusB, normal );

		b3Vec3 point = b3Lerp( cA, cB, B3_FIX( 0.5f ) );

		// Manifold in frame A
		manifold->normal = normal;
		manifold->pointCount = 1;

		b3LocalManifoldPoint* pt = manifold->points + 0;
		pt->point = point;
		pt->separation = bestDistance - radius;
		pt->pair = b3FeaturePair_single;
	}
}

void b3CollideCapsules( b3LocalManifold* manifold, int capacity, const b3Capsule* capsuleA, const b3Capsule* capsuleB,
						b3Transform transformBtoA )
{
	manifold->pointCount = 0;

	if ( capacity < 2 )
	{
		return;
	}

	// Work in shapeA coordinates
	b3Vec3 centerA1 = capsuleA->center1;
	b3Vec3 centerA2 = capsuleA->center2;
	b3Vec3 centerB1 = b3TransformPoint( transformBtoA, capsuleB->center1 );
	b3Vec3 centerB2 = b3TransformPoint( transformBtoA, capsuleB->center2 );

	b3Fixed radius = capsuleA->radius + capsuleB->radius;
	b3Fixed maxDistance = radius + B3_SPECULATIVE_DISTANCE;

	b3SegmentDistanceResult result = b3SegmentDistance( centerA1, centerA2, centerB1, centerB2 );
	b3Vec3 offset = b3Sub( result.point2, result.point1 );
	b3Fixed distanceSquared = b3LengthSquared( offset );
	b3Fixed linearSlop = B3_LINEAR_SLOP;
	b3Fixed minDistance = b3FixMul( B3_FIX( 0.01f ) , linearSlop );

	if ( distanceSquared > b3FixMul( maxDistance , maxDistance ) || distanceSquared < b3FixMul( minDistance , minDistance ) )
	{
		// We found a separating axis
		return;
	}

	b3Fixed lengthA;
	b3Vec3 segmentA = b3Sub( centerA2, centerA1 );
	b3Vec3 edgeA = b3GetLengthAndNormalize( &lengthA, segmentA );
	if ( lengthA < B3_MIN_CAPSULE_LENGTH )
	{
		return;
	}

	b3Fixed lengthB;
	b3Vec3 segmentB = b3Sub( centerB2, centerB1 );
	b3Vec3 edgeB = b3GetLengthAndNormalize( &lengthB, segmentB );
	if ( lengthB < B3_MIN_CAPSULE_LENGTH )
	{
		return;
	}

	// Parallel edges: |eA x eB| = sin(alpha)
	const b3Fixed alphaTol = B3_FIX( 0.05f );
	const b3Fixed alphaTolSqr = b3FixMul( alphaTol , alphaTol );
	b3Vec3 axis = b3Cross( edgeA, edgeB );

	// Try to create two contact points if the capsules are nearly parallel
	if ( b3LengthSquared( axis ) < alphaTolSqr )
	{
		// Clip segment B against side planes of segment A

		// Sides planes of A
		b3Plane planesA[2];
		planesA[0].normal = b3Neg( edgeA );
		planesA[0].offset = -b3Dot( edgeA, capsuleA->center1 );
		planesA[1].normal = edgeA;
		planesA[1].offset = b3Dot( edgeA, capsuleA->center2 );

		// Clip points for B
		b3ClipVertex verticesB[2];
		verticesB[0].position = centerB1;
		verticesB[0].separation = B3_FIX( 0.0f );
		verticesB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
		verticesB[1].position = centerB2;
		verticesB[1].separation = B3_FIX( 0.0f );
		verticesB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

		int pointCount = b3ClipSegment( verticesB, planesA[0] );
		if ( pointCount == 2 )
		{
			pointCount = b3ClipSegment( verticesB, planesA[1] );
		}

		if ( pointCount == 2 )
		{
			// Closest points on A to the clipped points on B.
			b3Vec3 closestPoint1 = b3PointToSegmentDistance( centerA1, centerA2, verticesB[0].position );
			b3Vec3 closestPoint2 = b3PointToSegmentDistance( centerA1, centerA2, verticesB[1].position );

			b3Fixed distance1 = b3Distance( closestPoint1, verticesB[0].position );
			b3Fixed distance2 = b3Distance( closestPoint2, verticesB[1].position );
			if ( distance1 <= radius && distance2 <= radius )
			{
				if ( distance1 < minDistance || distance2 < minDistance )
				{
					// Avoid divide by zero
					return;
				}

				b3Vec3 normal1 = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , distance1 ), b3Sub( verticesB[0].position, closestPoint1 ) );
				b3Vec3 normal2 = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , distance2 ), b3Sub( verticesB[1].position, closestPoint2 ) );
				b3Vec3 normal = b3Normalize( b3Add( normal1, normal2 ) );
				b3Fixed radiusA = capsuleA->radius;
				b3Fixed radiusB = capsuleB->radius;

				// Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
				b3Vec3 point1 =
					b3MulSV( B3_FIX( 0.5f ), b3MulSub( b3Add( b3MulAdd( verticesB[0].position, radiusA, normal1 ), closestPoint1 ), radiusB,
											 normal ) );
				b3Vec3 point2 =
					b3MulSV( B3_FIX( 0.5f ), b3MulSub( b3Add( b3MulAdd( verticesB[1].position, radiusA, normal2 ), closestPoint2 ), radiusB,
											 normal ) );

				// Manifold in frame A
				manifold->normal = normal;
				manifold->pointCount = 2;

				b3LocalManifoldPoint* pt1 = manifold->points + 0;
				pt1->point = point1;
				pt1->separation = distance1 - radius;
				pt1->pair = verticesB[0].pair;

				b3LocalManifoldPoint* pt2 = manifold->points + 1;
				pt2->point = point2;
				pt2->separation = distance2 - radius;
				pt2->pair = verticesB[1].pair;

				return;
			}
		}
	}

	b3Fixed distance;
	b3Vec3 normal = b3GetLengthAndNormalize( &distance, offset );
	// Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
	b3Vec3 point = b3MulSV(
		B3_FIX( 0.5f ), b3MulSub( b3Add( b3MulAdd( result.point1, capsuleA->radius, normal ), result.point2 ), capsuleB->radius, normal ) );

	// Manifold in frame A
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = distance - radius;
	pt->pair = b3FeaturePair_single;
}

static bool b3BuildHullFaceAndCapsuleContact( b3LocalManifold* manifold, const b3HullData* hullA, const b3Capsule* capsuleB,
											  b3Transform transformBtoA, b3SeparatingAxis query )
{
	// Work in shapeA coordinates
	const b3Plane* planes = b3GetHullPlanes( hullA );

	// Clip the capsule edge against the side planes of the reference face
	int refFace = query.indexA;
	b3Plane refPlane = planes[refFace];

	b3ClipVertex segmentB[2];
	segmentB[0].position = b3TransformPoint( transformBtoA, capsuleB->center1 );
	segmentB[0].separation = B3_FIX( 0.0f );
	segmentB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
	segmentB[1].position = b3TransformPoint( transformBtoA, capsuleB->center2 );
	segmentB[1].separation = B3_FIX( 0.0f );
	segmentB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

	int pointCount = b3ClipSegmentToHullFace( segmentB, hullA, refFace );
	if ( pointCount < 2 )
	{
		return false;
	}

	b3Fixed distance1 = b3PlaneSeparation( refPlane, segmentB[0].position );
	b3Fixed distance2 = b3PlaneSeparation( refPlane, segmentB[1].position );
	const b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;

	if ( distance1 <= speculativeDistance || distance2 <= speculativeDistance )
	{
		b3Vec3 normal = refPlane.normal;
		b3Vec3 point1 = b3MulSub( segmentB[0].position, b3FixMul( B3_FIX( 0.5f ) , ( distance1 + capsuleB->radius ) ), normal );
		b3Vec3 point2 = b3MulSub( segmentB[1].position, b3FixMul( B3_FIX( 0.5f ) , ( distance2 + capsuleB->radius ) ), normal );

		// Manifold in frame A
		manifold->normal = normal;
		manifold->pointCount = 2;

		b3LocalManifoldPoint* pt1 = manifold->points + 0;
		pt1->point = point1;
		pt1->separation = distance1 - capsuleB->radius;
		pt1->pair = segmentB[0].pair;

		b3LocalManifoldPoint* pt2 = manifold->points + 1;
		pt2->point = point2;
		pt2->separation = distance2 - capsuleB->radius;
		pt2->pair = segmentB[1].pair;

		return true;
	}

	return false;
}

static bool b3BuildHullAndCapsuleEdgeContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA,
											  const b3Capsule* capsuleB, b3Transform transformBtoA, b3SeparatingAxis query )
{
	if ( capacity < 1 )
	{
		return false;
	}

	// Work in shapeA coordinates

	b3Vec3 pc = b3TransformPoint( transformBtoA, capsuleB->center1 );
	b3Vec3 qc = b3TransformPoint( transformBtoA, capsuleB->center2 );
	b3Vec3 ec = b3Sub( qc, pc );

	const b3HullHalfEdge* edges = b3GetHullEdges( hullA );
	const b3Vec3* points = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edge2 = edges + query.indexB;
	const b3HullHalfEdge* twin2 = edges + edge2->twin;
	b3Vec3 ph = points[edge2->origin];
	b3Vec3 qh = points[twin2->origin];
	b3Vec3 eh = b3Sub( qh, ph );

	b3Vec3 normal = query.normal;

	b3SegmentDistanceResult result = b3LineDistance( ph, eh, pc, ec );

	if ( b3IsWithinSegments( &result ) == false )
	{
		// closest point beyond end points
		return false;
	}

	b3Vec3 point = b3MulSV( B3_FIX( 0.5f ), b3Add( b3MulSub( result.point1, capsuleB->radius, normal ), result.point2 ) );
	b3Fixed separation = b3Dot( normal, b3Sub( result.point2, result.point1 ) );
	B3_VALIDATE( b3FixAbs( separation - query.separation ) < B3_LINEAR_SLOP );

	// Manifold in frame A
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = separation - capsuleB->radius;
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );
	return true;
}

void b3CollideHullAndCapsule( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3Capsule* capsuleB,
							  b3Transform transformBtoA, b3SimplexCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity < 2 )
	{
		return;
	}

	// Work in shapeA coordinates
	b3DistanceInput distanceInput;
	distanceInput.proxyA = (b3ShapeProxy){ b3GetHullPoints( hullA ), hullA->vertexCount, B3_FIX( 0.0f ) };
	distanceInput.proxyB = (b3ShapeProxy){ &capsuleB->center1, 2, B3_FIX( 0.0f ) };
	distanceInput.transform = transformBtoA;
	distanceInput.useRadii = false;

	b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, cache, NULL, 0 );
	const b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;

	if ( distanceOutput.distance > capsuleB->radius + speculativeDistance )
	{
		// We found a separating axis
		*cache = (b3SimplexCache){ 0 };
		return;
	}


	if ( distanceOutput.distance > 100 * B3_FIXED_EPSILON )
	{
		const b3Plane* planes = b3GetHullPlanes( hullA );

		// Shallow penetration
		b3Vec3 delta = distanceOutput.normal;
		int refFace = b3FindHullSupportFace( hullA, delta );
		b3Plane refPlane = planes[refFace];

		// Try to create two contact points if closest
		// points difference is nearly parallel to face normal
		const b3Fixed kTolerance = B3_FIX( 0.998f );
		if ( b3FixAbs( b3Dot( refPlane.normal, delta ) ) > kTolerance )
		{
			// Clip capsule segment against side planes of reference face
			b3ClipVertex verticesB[2];
			verticesB[0].position = b3TransformPoint( transformBtoA, capsuleB->center1 );
			verticesB[0].separation = B3_FIX( 0.0f );
			verticesB[0].pair = b3MakeFeaturePair( b3_featureShapeA, 0, b3_featureShapeA, 0 );
			verticesB[1].position = b3TransformPoint( transformBtoA, capsuleB->center2 );
			verticesB[1].separation = B3_FIX( 0.0f );
			verticesB[1].pair = b3MakeFeaturePair( b3_featureShapeA, 1, b3_featureShapeA, 1 );

			int pointCount = b3ClipSegmentToHullFace( verticesB, hullA, refFace );

			if ( pointCount == 2 )
			{
				b3Fixed distance1 = b3PlaneSeparation( refPlane, verticesB[0].position );
				b3Fixed distance2 = b3PlaneSeparation( refPlane, verticesB[1].position );
				if ( distance1 <= capsuleB->radius + speculativeDistance || distance2 <= capsuleB->radius + speculativeDistance )
				{
					b3Vec3 normal = refPlane.normal;
					b3Vec3 point1 = b3MulSub( verticesB[0].position, b3FixMul( B3_FIX( 0.5f ) , ( capsuleB->radius + distance1 ) ), normal );
					b3Vec3 point2 = b3MulSub( verticesB[1].position, b3FixMul( B3_FIX( 0.5f ) , ( capsuleB->radius + distance2 ) ), normal );

					// Manifold in frame A
					manifold->normal = normal;
					manifold->pointCount = 2;

					b3LocalManifoldPoint* pt1 = manifold->points + 0;
					pt1->point = point1;
					pt1->separation = distance1 - capsuleB->radius;
					pt1->pair = verticesB[0].pair;

					b3LocalManifoldPoint* pt2 = manifold->points + 1;
					pt2->point = point2;
					pt2->separation = distance2 - capsuleB->radius;
					pt2->pair = verticesB[1].pair;

					return;
				}
			}
		}

		// Create contact from closest points
		b3Vec3 point =
			b3MulSV( B3_FIX( 0.5f ), b3Add( b3MulSub( distanceOutput.pointA, capsuleB->radius, delta ), distanceOutput.pointB ) );

		// Manifold in frame A
		manifold->normal = delta;
		manifold->pointCount = 1;

		b3LocalManifoldPoint* pt = manifold->points + 0;
		pt->point = point;
		pt->separation = distanceOutput.distance - capsuleB->radius;
		pt->pair = b3FeaturePair_single;
		return;
	}

	// Deep penetration

	b3SeparatingAxis faceQuery = b3QueryFaceDirectionHullAndCapsule( hullA, capsuleB, transformBtoA );
	if ( faceQuery.separation > capsuleB->radius )
	{
		// We found a separating axis
		return;
	}

	b3SeparatingAxis edgeQuery = b3QueryEdgeDirectionHullAndCapsule( hullA, capsuleB, transformBtoA );
	if ( edgeQuery.separation > capsuleB->radius )
	{
		// We found a separating axis
		return;
	}

	// Create face contact
	b3Fixed faceSeparation = faceQuery.separation - capsuleB->radius;
	b3BuildHullFaceAndCapsuleContact( manifold, hullA, capsuleB, transformBtoA, faceQuery );
	B3_VALIDATE( manifold->pointCount == 0 || manifold->pointCount == 2 );
	if ( manifold->pointCount == 2 )
	{
		// This becomes the clipped separation.
		faceSeparation = b3FixMin( manifold->points[0].separation, manifold->points[1].separation );
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	// Upstream 781673b replaced the relative tolerance with a plain slop offset. In fixed point the
	// new form is strictly better: it is an int64 add instead of a b3FixMul, so nothing quantizes.
	const b3Fixed linearSlop = B3_LINEAR_SLOP;
	// The edge query can find no admissible pair (its separation stays at the
	// -B3_FIXED_MAX sentinel, which would wrap under further arithmetic).
	bool haveEdge = edgeQuery.indexB != B3_NULL_INDEX;
	b3Fixed edgeSeparation = haveEdge ? edgeQuery.separation - capsuleB->radius : B3_FIXED_MIN;
	if ( manifold->pointCount == 0 ? haveEdge : ( haveEdge && edgeSeparation > faceSeparation + linearSlop ) )
	{
		// Edge contact
		b3BuildHullAndCapsuleEdgeContact( manifold, capacity, hullA, capsuleB, transformBtoA, edgeQuery );
	}
}

static int b3BuildPolygon( b3ClipVertex* out, b3Transform transform, const b3HullData* hull, int incFace, b3Plane refPlane )
{
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );
	const b3Vec3* points = b3GetHullPoints( hull );

	const b3HullFace* face = faces + incFace;
	int edgeIndex = face->edge;
	B3_ASSERT( edges[edgeIndex].face == incFace );

	int outCount = 0;

	b3Matrix3 matrix = b3MakeMatrixFromQuat( transform.q );

	do
	{
		const b3HullHalfEdge* edge = edges + edgeIndex;

		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edges + nextEdgeIndex;

		b3ClipVertex vertex;
		vertex.position = b3Add( b3MulMV( matrix, points[next->origin] ), transform.p );
		vertex.separation = b3PlaneSeparation( refPlane, vertex.position );
		vertex.pair = b3MakeFeaturePair( b3_featureShapeB, edgeIndex, b3_featureShapeB, nextEdgeIndex );

		out[outCount] = vertex;
		outCount += 1;

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge && outCount < B3_MAX_CLIP_POINTS );

	B3_VALIDATE( b3ValidatePolygon( out, outCount ) );

	return outCount;
}

static bool b3BuildFaceAContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
								 b3Transform transformBtoA, b3SeparatingAxis query, b3SATCache* cache )
{
	B3_VALIDATE( query.type == b3_faceAxisA );
	B3_VALIDATE( 0 <= query.indexA && query.indexA < hullA->faceCount );
	B3_VALIDATE( 0 <= query.indexB && query.indexB < hullB->vertexCount );

	const b3HullFace* facesA = b3GetHullFaces( hullA );
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	// Reference face
	int refFace = query.indexA;
	b3Plane refPlane = planesA[refFace];

	// Find incident face
	b3Vec3 refNormalInB = b3InvRotateVector( transformBtoA.q, refPlane.normal );
	int incFace = b3FindIncidentFace( hullB, refNormalInB, query.indexB );

	// Build clip polygon from incident face in frame A
	b3ClipVertex buffer1[B3_MAX_CLIP_POINTS], buffer2[B3_MAX_CLIP_POINTS];
	int pointCount = b3BuildPolygon( buffer1, transformBtoA, hullB, incFace, refPlane );

	// Clip incident face against side planes of reference face
	b3ClipVertex* input = buffer1;
	b3ClipVertex* output = buffer2;

	const b3HullFace* face = facesA + refFace;
	int edgeIndex = face->edge;

	do
	{
		const b3HullHalfEdge* edge = edgesA + edgeIndex;
		int nextEdgeIndex = edge->next;
		const b3HullHalfEdge* next = edgesA + nextEdgeIndex;
		b3Vec3 vertex1 = pointsA[edge->origin];
		b3Vec3 vertex2 = pointsA[next->origin];
		b3Vec3 tangent = b3Normalize( b3Sub( vertex2, vertex1 ) );
		b3Vec3 binormal = b3Cross( tangent, refPlane.normal );

		b3Plane clipPlane = b3MakePlaneFromNormalAndPoint( binormal, vertex1 );

		pointCount = b3ClipPolygon( output, input, pointCount, clipPlane, edgeIndex, refPlane );
		B3_ASSERT( pointCount <= B3_MAX_CLIP_POINTS );

		B3_SWAP( output, input );

		if ( pointCount < 3 )
		{
			*cache = (b3SATCache){ 0 };
			return false;
		}

		edgeIndex = nextEdgeIndex;
	}
	while ( edgeIndex != face->edge );

	pointCount = b3MinInt( pointCount, B3_MAX_CLIP_POINTS );

	b3LocalManifoldPoint points[B3_MAX_CLIP_POINTS];
	b3Fixed minSeparation = B3_FIXED_MAX;

	manifold->normal = refPlane.normal;

	for ( int i = 0; i < pointCount; ++i )
	{
		b3ClipVertex* clipPoint = input + i;
		b3LocalManifoldPoint* pt = points + i;
		*pt = (b3LocalManifoldPoint){ 0 };

		// Using the half-way point keeps the points in the same position when swapping reference face from A to B.
		b3Vec3 point = b3MulSub( clipPoint->position, b3FixMul( B3_FIX( 0.5f ) , clipPoint->separation ), refPlane.normal );

		// Old way of pushing onto the reference face.
		// b3Vec3 point = clipPoint->position - clipPoint->separation * refPlane.normal;

		pt->point = point;
		pt->separation = clipPoint->separation;
		pt->pair = clipPoint->pair;

		minSeparation = b3FixMin( minSeparation, clipPoint->separation );
	}

	if ( minSeparation >= B3_SPECULATIVE_DISTANCE )
	{
		*cache = (b3SATCache){ 0 };
		return false;
	}

	b3ReduceManifoldPoints( manifold, capacity, points, pointCount );

	// Save cache
	cache->separation = minSeparation;
	cache->type = (uint8_t)b3_faceAxisA;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

static bool b3BuildFaceBContact( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
								 b3Transform transformBtoA, b3SeparatingAxis query, b3SATCache* cache )
{
	B3_VALIDATE( query.type == b3_faceAxisB );

	b3Transform transformAtoB = b3InvertTransform( transformBtoA );
	b3SeparatingAxis flippedQuery = {
		.normal = b3Neg( query.normal ),
		.separation = query.separation,
		.indexA = query.indexB,
		.indexB = query.indexA,
		.type = b3_faceAxisA,
	};

	bool touching = b3BuildFaceAContact( manifold, capacity, hullB, hullA, transformAtoB, flippedQuery, cache );
	if ( touching == false )
	{
		*cache = (b3SATCache){ 0 };
		return false;
	}

	// Results are in frame B, need to transform them into frame A
	b3Matrix3 matrix = b3MakeMatrixFromQuat( transformBtoA.q );

	// Transform and flip normal so it points from A to B, even though the B has the reference face.
	manifold->normal = b3Neg( b3MulMV( matrix, manifold->normal ) );

	// Transform points from frame B to frame A.
	// Also flip the pairs to ensure correct matches.
	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		b3LocalManifoldPoint* pt = manifold->points + i;
		pt->point = b3Add( b3MulMV( matrix, pt->point ), transformBtoA.p );
		pt->pair = b3FlipPair( pt->pair );
	}

	cache->type = (uint8_t)b3_faceAxisB;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

static bool b3BuildEdgeContact( b3LocalManifold* manifold, const b3HullData* hullA, const b3HullData* hullB,
								b3Transform transformBtoA, b3SeparatingAxis query, b3SATCache* cache )
{
	B3_VALIDATE( query.type == b3_edgePairAxis );
	B3_VALIDATE( 0 <= query.indexA && query.indexA < hullA->edgeCount );
	B3_VALIDATE( 0 <= query.indexB && query.indexB < hullB->edgeCount );

	// Work in shapeA coordinates
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	// B3_VALIDATE( query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE );

	const b3HullHalfEdge* edgeA = edgesA + query.indexA;
	const b3HullHalfEdge* twinA = edgesA + edgeA->twin;
	b3Vec3 pA = pointsA[edgeA->origin];
	b3Vec3 qA = pointsA[twinA->origin];
	b3Vec3 eA = b3Sub( qA, pA );

	const b3HullHalfEdge* edgeB = edgesB + query.indexB;
	const b3HullHalfEdge* twinB = edgesB + edgeB->twin;
	b3Vec3 pB = b3TransformPoint( transformBtoA, pointsB[edgeB->origin] );
	b3Vec3 qB = b3TransformPoint( transformBtoA, pointsB[twinB->origin] );
	b3Vec3 eB = b3Sub( qB, pB );

	b3Vec3 normal = query.normal;
	b3SegmentDistanceResult result = b3LineDistance( pA, eA, pB, eB );

	if ( b3IsWithinSegments( &result ) == false )
	{
		*cache = (b3SATCache){ 0 };
		return false;
	}

	// This can slide off the end from caching
	b3Fixed separation = b3Dot( normal, b3Sub( result.point2, result.point1 ) );
	b3Vec3 point = b3MulSV( B3_FIX( 0.5f ), b3Add( result.point1, result.point2 ) );

	// Result in frame A
	manifold->normal = normal;
	manifold->pointCount = 1;

	b3LocalManifoldPoint* pt = manifold->points + 0;
	pt->point = point;
	pt->separation = separation;
	pt->pair = b3MakeFeaturePair( b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB );

	// Save cache
	cache->separation = separation;
	cache->type = (uint8_t)b3_edgePairAxis;
	cache->indexA = (uint8_t)query.indexA;
	cache->indexB = (uint8_t)query.indexB;

	return true;
}

void b3CollideHulls( b3LocalManifold* manifold, int capacity, const b3HullData* hullA, const b3HullData* hullB,
					 b3Transform transformBtoA, b3SATCache* cache )
{
	manifold->pointCount = 0;

	if ( capacity < 4 )
	{
		return;
	}

	// Work in shapeA coordinates
	b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;

	b3Fixed linearSlop = B3_LINEAR_SLOP;
	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Plane* planesA = b3GetHullPlanes( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );

	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Plane* planesB = b3GetHullPlanes( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	// Attempt to use the cache to speed up collision
	switch ( cache->type )
	{
		case b3_invalidAxis:
			*cache = (b3SATCache){ 0 };
			break;

		case b3_faceAxisA:
		{
			B3_ASSERT( cache->indexA < hullA->faceCount );

			// Check for separation using cached face
			b3Plane plane = planesA[cache->indexA];
			b3Vec3 searchDirectionInB = b3Neg( b3InvRotateVector( transformBtoA.q, plane.normal ) );
			int vertexIndex = b3FindHullSupportVertex( hullB, searchDirectionInB );
			b3Vec3 support = b3TransformPoint( transformBtoA, pointsB[vertexIndex] );
			b3Fixed separation = b3PlaneSeparation( plane, support );

			if ( separation >= speculativeDistance )
			{
				// Cache hit, shapes are separated
				return;
			}

			// if ( cache->separation < speculativeDistance )
			{
				// Attempt face contact using cached feature
				b3SeparatingAxis faceQuery;
				faceQuery.normal = plane.normal;
				faceQuery.separation = B3_FIX( 0.0f );
				faceQuery.indexA = cache->indexA;
				faceQuery.indexB = vertexIndex;
				faceQuery.type = b3_faceAxisA;

				b3SATCache localCache = { 0 };
				bool touching = b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache );
				if ( touching == true && b3FixAbs( cache->separation - localCache.separation ) < linearSlop )
				{
					// Cache hit, contact points generated
					return;
				}
			}
		}
		break;

		case b3_faceAxisB:
		{
			B3_ASSERT( cache->indexB < hullB->faceCount );

			// Check for separation using cached face
			b3Plane plane = planesB[cache->indexB];
			b3Vec3 searchDirectionInA = b3Neg( b3RotateVector( transformBtoA.q, plane.normal ) );
			int vertexIndex = b3FindHullSupportVertex( hullA, searchDirectionInA );
			b3Vec3 support = b3InvTransformPoint( transformBtoA, pointsA[vertexIndex] );
			b3Fixed separation = b3PlaneSeparation( plane, support );

			if ( separation >= speculativeDistance )
			{
				// Cache hit, shapes are separated
				return;
			}

			// if ( cache->separation < speculativeDistance )
			{
				// Attempt face contact using cached feature
				b3SeparatingAxis faceQuery;
				faceQuery.normal = b3Neg( plane.normal );
				faceQuery.separation = B3_FIX( 0.0f );
				faceQuery.indexA = vertexIndex;
				faceQuery.indexB = cache->indexB;
				faceQuery.type = b3_faceAxisB;

				b3SATCache localCache = { 0 };
				bool touching = b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, faceQuery, &localCache );
				if ( touching == true && b3FixAbs( cache->separation - localCache.separation ) < linearSlop )
				{
					// Cache hit, contact points generated
					return;
				}
			}
		}
		break;

		case b3_edgePairAxis:
		{
			int indexA = cache->indexA;
			const b3HullHalfEdge* edge1 = edgesA + indexA;
			const b3HullHalfEdge* twin1 = edgesA + indexA + 1;
			B3_ASSERT( edge1->twin == indexA + 1 && twin1->twin == indexA );

			b3Vec3 pA = pointsA[edge1->origin];
			b3Vec3 qA = pointsA[twin1->origin];
			b3Vec3 eA = b3Sub( qA, pA );

			b3Vec3 uA = planesA[edge1->face].normal;
			b3Vec3 vA = planesA[twin1->face].normal;

			int indexB = cache->indexB;
			const b3HullHalfEdge* edge2 = edgesB + indexB;
			const b3HullHalfEdge* twin2 = edgesB + indexB + 1;
			B3_ASSERT( edge2->twin == indexB + 1 && twin2->twin == indexB );

			b3Vec3 pB = b3TransformPoint( transformBtoA, pointsB[edge2->origin] );
			b3Vec3 qB = b3TransformPoint( transformBtoA, pointsB[twin2->origin] );
			b3Vec3 eB = b3Sub( qB, pB );

			b3Vec3 uB = b3RotateVector( transformBtoA.q, planesB[edge2->face].normal );
			b3Vec3 vB = b3RotateVector( transformBtoA.q, planesB[twin2->face].normal );

			// flipping the signs of u2 and v2
			// cross(v2, u2) == cross(-v2, -u2)
			// so we still use -e2
			// but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
			// Exact sign tests on raw 128-bit dots, matching b3QueryEdgeDirections
			// so the cached-axis check agrees with a full requery.
			b3Int128 cbaRaw = b3DotRaw( uB, eA );
			b3Int128 dbaRaw = b3DotRaw( vB, eA );
			b3Int128 adcRaw = -b3DotRaw( uA, eB );
			b3Int128 bdcRaw = -b3DotRaw( vA, eB );

			if ( cbaRaw != 0 && dbaRaw != 0 && ( cbaRaw ^ dbaRaw ) < 0 && adcRaw != 0 && bdcRaw != 0 &&
				 ( adcRaw ^ bdcRaw ) < 0 && ( cbaRaw ^ bdcRaw ) >= 0 )
			{
				b3Fixed cba = b3FixFromDotRaw( cbaRaw );
				b3Fixed dba = b3FixFromDotRaw( dbaRaw );

				// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
				b3Fixed squaredTolerance = b3FixMul( B3_FIX( B3_PARALLEL_EDGE_TOL ) , B3_FIX( B3_PARALLEL_EDGE_TOL ) );
				if ( b3FixMax( b3FixMul( cba , cba ), b3FixMul( dba , dba ) ) >= b3FixMul( squaredTolerance , b3LengthSquared( eA ) ) )
				{
					// Transform reference center of the first hull into local space of the second hull
					b3Fixed t = (b3Fixed)( b3Int128ShiftLeft( cbaRaw, B3_FIXED_FRACTION_BITS ) / ( cbaRaw - dbaRaw ) );
					b3Vec3 axis = b3Lerp( uB, vB, t );
					B3_VALIDATE( b3LengthSquared( axis ) > 0 );
					axis = b3Normalize( axis );
					b3Fixed separation = b3FixFromDotRaw( b3DotRaw( axis, b3Sub( qA, qB ) ) );

					if ( separation > speculativeDistance )
					{
						// Cache hit, shapes are separated
						return;
					}

					// Try to rebuild contact from last features
					b3SeparatingAxis edgeQuery = { 0 };
					edgeQuery.normal = b3Neg( axis );
					edgeQuery.separation = B3_FIX( 0.0f );
					edgeQuery.indexA = cache->indexA;
					edgeQuery.indexB = cache->indexB;
					edgeQuery.type = b3_edgePairAxis;

					b3SATCache localCache = { 0 };
					bool touching = b3BuildEdgeContact( manifold, hullA, hullB, transformBtoA, edgeQuery, &localCache );
					if ( touching && b3FixAbs( cache->separation - localCache.separation ) < linearSlop )
					{
						// Cache hit, contact point generated
						return;
					}
				}
			}
		}
		break;

			// This case is for testing
		case b3_manualFaceAxisA:
		{
			b3SeparatingAxis faceQueryA = b3QueryFaceDirections( hullA, hullB, transformBtoA );
			b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, faceQueryA, cache );
			return;
		}

			// This case is for testing
		case b3_manualFaceAxisB:
		{
			b3SeparatingAxis rawQueryB = b3QueryFaceDirections( hullB, hullA, b3InvertTransform( transformBtoA ) );
			b3SeparatingAxis faceQueryB = {
				.normal = b3Neg( rawQueryB.normal ),
				.separation = rawQueryB.separation,
				.indexA = rawQueryB.indexB,
				.indexB = rawQueryB.indexA,
				.type = b3_faceAxisB,
			};
			b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, faceQueryB, cache );
			return;
		}

			// This case is for testing
		case b3_manualEdgePairAxis:
		{
			b3SeparatingAxis edgeQuery = b3QueryEdgeDirections( hullA, hullB, transformBtoA );
			if ( edgeQuery.indexA != B3_NULL_INDEX )
			{
				b3BuildEdgeContact( manifold, hullA, hullB, transformBtoA, edgeQuery, cache );
			}
			return;
		}

		default:
			B3_ASSERT( false );
			break;
	}

	manifold->pointCount = 0;
	*cache = (b3SATCache){ 0 };

	// Find axis of minimum penetration
	b3SeparatingAxis faceQueryA = b3QueryFaceDirections( hullA, hullB, transformBtoA );
	if ( faceQueryA.separation > speculativeDistance )
	{
		B3_ASSERT( faceQueryA.indexA < hullA->faceCount );
		B3_ASSERT( faceQueryA.indexB < hullB->vertexCount );

		// We found a separating axis
		cache->separation = faceQueryA.separation;
		cache->type = (uint8_t)b3_faceAxisA;
		cache->indexA = (uint8_t)faceQueryA.indexA;
		cache->indexB = (uint8_t)faceQueryA.indexB;
		return;
	}

	// Re-express the flipped query in the b3_faceAxisB convention: indexA is the
	// support vertex on hull A, indexB is the reference face on hull B. The normal
	// is decorative here (the contact builders work from the indices), so the cheap
	// frame-B negation matches upstream's cache-rebuild spelling.
	b3SeparatingAxis rawQueryB = b3QueryFaceDirections( hullB, hullA, b3InvertTransform( transformBtoA ) );
	b3SeparatingAxis faceQueryB = {
		.normal = b3Neg( rawQueryB.normal ),
		.separation = rawQueryB.separation,
		.indexA = rawQueryB.indexB,
		.indexB = rawQueryB.indexA,
		.type = b3_faceAxisB,
	};

	if ( faceQueryB.separation > speculativeDistance )
	{
		B3_ASSERT( faceQueryB.indexA < hullA->vertexCount );
		B3_ASSERT( faceQueryB.indexB < hullB->faceCount );

		// We found a separating axis
		cache->separation = faceQueryB.separation;
		cache->type = (uint8_t)b3_faceAxisB;
		cache->indexA = (uint8_t)faceQueryB.indexA;
		cache->indexB = (uint8_t)faceQueryB.indexB;
		return;
	}

	b3SeparatingAxis edgeQuery = b3QueryEdgeDirections( hullA, hullB, transformBtoA );
	if ( edgeQuery.separation > speculativeDistance )
	{
		// We found a separating axis
		cache->separation = edgeQuery.separation;
		cache->type = (uint8_t)b3_edgePairAxis;
		cache->indexA = (uint8_t)edgeQuery.indexA;
		cache->indexB = (uint8_t)edgeQuery.indexB;
		return;
	}

	// Always build a face contact first: a one point edge contact is weak for
	// stacking (upstream Fixes 07, box3d 2386141).
	B3_VALIDATE( faceQueryA.separation <= speculativeDistance && faceQueryB.separation <= speculativeDistance &&
				 edgeQuery.separation <= speculativeDistance );

	if ( faceQueryA.separation > faceQueryB.separation )
	{
		// Face contact A
		b3BuildFaceAContact( manifold, capacity, hullA, hullB, transformBtoA, faceQueryA, cache );
	}
	else
	{
		// Face contact B
		b3BuildFaceBContact( manifold, capacity, hullA, hullB, transformBtoA, faceQueryB, cache );
	}

	if ( edgeQuery.indexA == B3_NULL_INDEX )
	{
		// There are no valid edge pairs (all edges parallel)
		return;
	}

	b3Fixed clipSeparation = cache->separation;
	b3Fixed edgeTol = linearSlop;

	B3_VALIDATE( edgeQuery.separation <= speculativeDistance );

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	if ( manifold->pointCount == 0 || edgeQuery.separation > clipSeparation + edgeTol )
	{
		B3_ASSERT( 0 <= edgeQuery.indexA && edgeQuery.indexA < hullA->edgeCount );
		B3_ASSERT( 0 <= edgeQuery.indexB && edgeQuery.indexB < hullB->edgeCount );

		// Edge contact
		b3LocalManifold edgeManifold = { 0 };
		b3LocalManifoldPoint edgePoint = { 0 };
		edgeManifold.points = &edgePoint;

		b3SATCache edgeCache = { 0 };
		b3BuildEdgeContact( &edgeManifold, hullA, hullB, transformBtoA, edgeQuery, &edgeCache );

		// It is possible with speculation to have vertex-vertex collision that is missed by SAT,
		// so edge contact yields no points. In that case perhaps the face contact has some points.
		if ( edgeManifold.pointCount == 1 )
		{
			// Copy edge manifold out, being careful to preserve manifold point buffer.
			b3LocalManifoldPoint* points = manifold->points;
			*manifold = edgeManifold;
			manifold->points = points;
			manifold->points[0] = edgePoint;
			*cache = edgeCache;
		}
	}
}
