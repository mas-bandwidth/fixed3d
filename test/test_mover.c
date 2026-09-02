// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

// b3CollideMoverAndSphere / Capsule / Hull are internal
#include "shape.h"

#include "box3d/collision.h"

static int ParallelPlanes( void )
{
	b3CollisionPlane planes[3] = { 0 };
	planes[0].plane.normal = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
	planes[0].plane.offset = B3_FIX( 0.5f );
	planes[0].pushLimit = B3_FIXED_MAX;
	planes[1].plane.normal = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
	planes[1].plane.offset = B3_FIX( 1.0f );
	planes[1].pushLimit = B3_FIXED_MAX;
	// planes[2].plane.normal = b3Normalize((b3Vec3){ 0.2f, 0.0f, 0.9f });
	// planes[2].plane.offset = 0.25f;
	// planes[2].pushLimit = B3_FIXED_MAX;

	b3Vec3 target = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3PlaneSolverResult result = b3SolvePlanes( target, planes, 2 );

	ENSURE( result.iterationCount == 2 );
	ENSURE_SMALL( result.delta.z - B3_FIX( 1.0f ), B3_FIX( 0.0055f ) );

	return 0;
}

static int GamePlanes( void )
{
	// This scenario takes many iterations because the target is deep into the plane.
	b3CollisionPlane planes[3] = { 0 };
	planes[0].plane.normal = (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 0.23941046f ), B3_FIX( 0.970918416f ) };
	planes[0].plane.offset = B3_FIX( 0.390724182f );
	planes[0].pushLimit = B3_FIXED_MAX;
	planes[1].plane.normal = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
	planes[1].plane.offset = B3_FIX( 1.49998093f );
	planes[1].pushLimit = B3_FIXED_MAX;

	b3Vec3 target = { -B3_FIX( 2.5390625f ), B3_FIX( 0.0f ), -B3_FIX( 73.6880798f ) };

	planes[0].plane.offset -= b3Dot( planes[0].plane.normal, target );
	planes[1].plane.offset -= b3Dot( planes[1].plane.normal, target );
	target = b3Vec3_zero;

	b3PlaneSolverResult result = b3SolvePlanes( target, planes, 2 );

	ENSURE( result.iterationCount == 20 );

	return 0;
}

// ---------------------------------------------------------------------------
// Mover-collide overlap handling
//
// b3CollideMoverAndSphere / Capsule / Hull must never emit a plane with a
// degenerate (zero) normal, even when the mover deeply penetrates the shape.
// On deep overlap the GJK path returns a {0,0,0} normal; these tests guard the
// fix that replaces it with an analytic (sphere/capsule) or dropped (hull) result.
// ---------------------------------------------------------------------------

static int MoverSphereSeparated( void )
{
	b3Sphere shape = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3Capsule mover = { { B3_FIX( 4.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 6.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndSphere( &result, &shape, &mover );
	ENSURE( count == 0 );

	return 0;
}

static int MoverSphereTouching( void )
{
	b3Sphere shape = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };

	// Mover core segment runs along X at y = 0.6, leaving it 0.1 inside the
	// 0.7 combined radius.
	b3Capsule mover = { { -B3_FIX( 1.0f ), B3_FIX( 0.6f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.6f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndSphere( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// Push-out points from the sphere straight up toward the mover.
	ENSURE( result.plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE_SMALL( result.plane.offset - B3_FIX( 0.1f ), 8 * B3_FIXED_EPSILON );

	return 0;
}

static int MoverSphereDeepOverlap( void )
{
	b3Sphere shape = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };

	// Mover axis runs straight through the sphere center: the bug case where
	// GJK reports a zero normal.
	b3Capsule mover = { { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndSphere( &result, &shape, &mover );
	ENSURE( count == 1 );

	// The normal must still be a valid unit vector.
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// The fallback axis is perpendicular to the mover axis (X).
	ENSURE_SMALL( result.plane.normal.x, 8 * B3_FIXED_EPSILON );

	// Deepest possible penetration: the full combined radius.
	ENSURE_SMALL( result.plane.offset - B3_FIX( 0.7f ), 8 * B3_FIXED_EPSILON );

	return 0;
}

static int MoverCapsuleSeparated( void )
{
	b3Capsule shape = { { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };
	b3Capsule mover = { { -B3_FIX( 1.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 0 );

	return 0;
}

static int MoverCapsuleTouching( void )
{
	b3Capsule shape = { { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };

	// Parallel mover 0.4 above, leaving it 0.1 inside the 0.5 combined radius.
	b3Capsule mover = { { -B3_FIX( 1.0f ), B3_FIX( 0.4f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.4f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );
	ENSURE( result.plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE_SMALL( result.plane.offset - B3_FIX( 0.1f ), 8 * B3_FIXED_EPSILON );

	return 0;
}

static int MoverCapsuleDeepOverlap( void )
{
	// Shape capsule along X, mover capsule along Z; their core segments cross
	// exactly at the origin, so GJK reports a zero normal.
	b3Capsule shape = { { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };
	b3Capsule mover = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// The separating axis of two crossing segments is perpendicular to both.
	ENSURE_SMALL( result.plane.normal.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.plane.normal.z, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.plane.offset - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );

	return 0;
}

static int MoverCapsuleParallelOverlap( void )
{
	// Mover core segment coincides with the shape core segment: the cross-product
	// axis degenerates, so a perpendicular of the mover axis is used instead.
	b3Capsule shape = { { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };
	b3Capsule mover = { { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndCapsule( &result, &shape, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );

	// The fallback axis is perpendicular to the mover axis (X).
	ENSURE_SMALL( result.plane.normal.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( result.plane.offset - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );

	return 0;
}

static int MoverHullSeparated( void )
{
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3Capsule mover = { { -B3_FIX( 0.3f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.3f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndHull( &result, &box.base, &mover );
	ENSURE( count == 0 );

	return 0;
}

static int MoverHullTouching( void )
{
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	// Mover core segment above the +Y face; the 0.2 radius reaches 0.1 into it.
	b3Capsule mover = { { -B3_FIX( 0.3f ), B3_FIX( 0.6f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.3f ), B3_FIX( 0.6f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndHull( &result, &box.base, &mover );
	ENSURE( count == 1 );
	ENSURE( b3IsNormalized( result.plane.normal ) );
	ENSURE( result.plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE_SMALL( result.plane.offset - B3_FIX( 0.1f ), B3_FIX( 1e-4f ) );

	return 0;
}

static int MoverHullDeepOverlap( void )
{
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	// Mover core segment lies entirely inside the box, so GJK reports overlap.
	b3Capsule mover = { { -B3_FIX( 0.2f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.2f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.1f ) };

	b3PlaneResult result = { 0 };
	int count = b3CollideMoverAndHull( &result, &box.base, &mover );

	// The overlap guard drops the plane rather than emit a zero normal.
	// todo replace with SAT once b3CollideMoverAndHull resolves overlaps.
	ENSURE( count == 0 );

	return 0;
}

// Back-side culling on the MOVER paths. The shape-cast cull has MeshBackSideCull in
// test_mesh.c; instrumenting every cull site across the whole suite showed the two
// b3CollideMoverAnd* sites reached and never fired, so until these two subtests existed
// the highest consequence cull in the change -- a character mover's contact planes --
// was carried by inspection alone. A wrong sign here is a character falling through a
// floor, and no determinism golden can see it because nothing in the corpus is culled.

#define MOVER_PLATE_VERTEX_COUNT 4
#define MOVER_PLATE_TRIANGLE_COUNT 2

// A flat plate in the y = 0 plane, wound CCW seen from above, so its front face is +Y.
static b3MeshData* MakeMoverPlate( void )
{
	static b3Vec3 vertices[MOVER_PLATE_VERTEX_COUNT] = {
		{ -B3_FIX( 2.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 2.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 2.0f ) },
	};

	static int32_t indices[3 * MOVER_PLATE_TRIANGLE_COUNT] = { 0, 3, 1, 1, 3, 2 };

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.indices = indices;
	def.vertexCount = MOVER_PLATE_VERTEX_COUNT;
	def.triangleCount = MOVER_PLATE_TRIANGLE_COUNT;
	return b3CreateMesh( &def, NULL, 0 );
}

// Collide a short vertical mover centred at height y against the plate, and report how
// many contact planes come back.
static int CollideMoverAtHeight( b3MeshData* data, b3Fixed y )
{
	b3Mesh mesh = { data, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };

	b3Capsule mover = {
		{ B3_FIX( 0.0f ), y - B3_FIX( 0.05f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 0.0f ), y + B3_FIX( 0.05f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.25f ),
	};

	b3PlaneResult planes[8] = { 0 };
	return b3CollideMoverAndMesh( planes, ARRAY_COUNT( planes ), &mesh, &mover );
}

static int MoverMeshBackSideCull( void )
{
	b3MeshData* plate = MakeMoverPlate();
	ENSURE( plate != NULL );

	// Above the plate and within the mover radius: the front side, so a plane is reported.
	ENSURE( CollideMoverAtHeight( plate, B3_FIX( 0.2f ) ) > 0 );

	// Below the plate, the same distance away: the back side, so the cull drops it.
	ENSURE( CollideMoverAtHeight( plate, -B3_FIX( 0.2f ) ) == 0 );

	// Far below, out of range either way. Without this the case above could pass merely
	// by being too far from the plate to touch it, which would make the cull untested
	// while looking green.
	ENSURE( CollideMoverAtHeight( plate, -B3_FIX( 3.0f ) ) == 0 );

	// And far above, out of range on the front side, so the in-range case above is
	// reporting contact rather than reporting everything.
	ENSURE( CollideMoverAtHeight( plate, B3_FIX( 3.0f ) ) == 0 );

	b3DestroyMesh( plate );
	return 0;
}

int MoverTest( void )
{
	RUN_SUBTEST( GamePlanes );
	RUN_SUBTEST( ParallelPlanes );

	RUN_SUBTEST( MoverSphereSeparated );
	RUN_SUBTEST( MoverSphereTouching );
	RUN_SUBTEST( MoverSphereDeepOverlap );

	RUN_SUBTEST( MoverCapsuleSeparated );
	RUN_SUBTEST( MoverCapsuleTouching );
	RUN_SUBTEST( MoverCapsuleDeepOverlap );
	RUN_SUBTEST( MoverCapsuleParallelOverlap );

	RUN_SUBTEST( MoverHullSeparated );
	RUN_SUBTEST( MoverHullTouching );
	RUN_SUBTEST( MoverHullDeepOverlap );

	RUN_SUBTEST( MoverMeshBackSideCull );

	return 0;
}
