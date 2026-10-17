// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

// b3CollideMoverAndSphere / Capsule / Hull are internal
#include "shape.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <stdint.h>

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

// Mover queries report which material a contact plane came from.
// b3PlaneResult::materialIndex follows different paths per shape type. Meshes
// report the per triangle material index. Compounds remap the child result
// through the child material table. Convex shapes report index 0.
//
// Tolerances: upstream 1e-4f is 6.55 ulp, under this tree's 8 ulp floor, so
// those checks use 8 * B3_FIXED_EPSILON. Thresholds such as 0.99 stay B3_FIX.
// Scene positions that are not dyadic rationals go through B3_FIX (round to
// nearest). 0.15 and 0.20 quantize so radius - center1.y equals B3_FIX(0.05)
// exactly, which is the penetration these cases pin.
static b3SurfaceMaterial MakeMaterial( b3Fixed friction, uint64_t userId )
{
	b3SurfaceMaterial m = b3DefaultSurfaceMaterial();
	m.friction = friction;
	m.userMaterialId = userId;
	return m;
}

// Two separated upward facing triangles on the y = 0 plane, one per material. The bake may
// reorder triangles but always keeps a triangle paired with its material index.
static b3MeshData* MakeTwoMaterialMesh( void )
{
	b3Vec3 vertices[6] = {
		{ -B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
		{ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
		{ B3_FIX( 1.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
		{ B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
	};
	int32_t indices[6] = { 0, 1, 2, 3, 4, 5 };
	uint8_t materialIndices[2] = { 0, 1 };

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.stride = sizeof( b3Vec3 );
	def.indices = indices;
	def.materialIndices = materialIndices;
	def.vertexCount = 6;
	def.triangleCount = 2;

	return b3CreateMesh( &def, NULL, 0 );
}

// Flat 3x3 vertex field at y = 0, one cell material per entry. Caller destroys.
static b3HeightFieldData* MakeFlatField( uint8_t* materials, bool clockwise )
{
	b3Fixed heights[9] = { 0 };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) };
	def.countX = 3;
	def.countZ = 3;
	def.globalMinimumHeight = -B3_FIX( 1.0f );
	def.globalMaximumHeight = B3_FIX( 1.0f );
	def.clockwiseWinding = clockwise;

	return b3CreateHeightField( &def );
}

typedef struct PlaneCapture
{
	b3PlaneResult planes[16];
	int count;
} PlaneCapture;

static bool CapturePlaneFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	(void)shapeId;
	PlaneCapture* capture = context;
	for ( int i = 0; i < planeCount && capture->count < 16; ++i )
	{
		capture->planes[capture->count++] = planes[i];
	}
	return true;
}

static int MoverWorldMeshMaterials( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MeshData* mesh = MakeTwoMaterialMesh();
	ENSURE( mesh != NULL );
	ENSURE( mesh->materialCount == 2 );

	b3SurfaceMaterial materials[2] = { MakeMaterial( B3_FIX( 0.25f ), 1 ), MakeMaterial( B3_FIX( 0.75f ), 2 ) };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.materials = materials;
	shapeDef.materialCount = 2;
	b3CreateMeshShape( bodyId, &shapeDef, mesh, (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } );

	b3World_Step( worldId, B3_FIX( 1.0f ) / 60, 1 );

	const uint8_t* bakedMaterialIndices = b3GetMeshMaterialIndices( mesh );

	// Mover hanging just above the first triangle so its radius reaches 0.05 into the surface.
	b3Capsule mover = {
		{ -B3_FIX( 2.0f ), B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};

	PlaneCapture capture = { 0 };
	b3World_CollideMover( worldId, b3Pos_zero, &mover, b3DefaultQueryFilter(), CapturePlaneFcn, &capture );

	ENSURE( capture.count == 1 );
	ENSURE( capture.planes[0].plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE_SMALL( capture.planes[0].plane.offset - B3_FIX( 0.05f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( capture.planes[0].point.y, 8 * B3_FIXED_EPSILON );
	ENSURE( capture.planes[0].materialIndex == 0 );
	ENSURE( capture.planes[0].triangleIndex >= 0 );
	ENSURE( capture.planes[0].triangleIndex < mesh->triangleCount );
	ENSURE( bakedMaterialIndices[capture.planes[0].triangleIndex] == 0 );

	// Same mover over the second triangle.
	b3Capsule mover2 = {
		{ B3_FIX( 2.0f ), B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};

	PlaneCapture capture2 = { 0 };
	b3World_CollideMover( worldId, b3Pos_zero, &mover2, b3DefaultQueryFilter(), CapturePlaneFcn, &capture2 );

	ENSURE( capture2.count == 1 );
	ENSURE( capture2.planes[0].plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE( capture2.planes[0].materialIndex == 1 );
	ENSURE( bakedMaterialIndices[capture2.planes[0].triangleIndex] == 1 );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

static int MoverWorldCompoundMeshMaterials( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MeshData* mesh = MakeTwoMaterialMesh();
	ENSURE( mesh != NULL );

	// The hull material and the two mesh materials are distinct, so the compound material
	// table gets one slot per material. Children bake in hull then mesh order.
	b3SurfaceMaterial meshMaterials[2] = { MakeMaterial( B3_FIX( 0.25f ), 101 ), MakeMaterial( B3_FIX( 0.5f ), 202 ) };
	b3SurfaceMaterial hullMaterial = MakeMaterial( B3_FIX( 0.75f ), 303 );

	b3CompoundMeshDef meshChild = {
		.meshData = mesh,
		.transform = b3Transform_identity,
		.scale = { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) },
		.materials = meshMaterials,
		.materialCount = 2,
	};

	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3CompoundHullDef hullChild = {
		.hull = &box.base,
		.transform = { .p = { -B3_FIX( 8.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, .q = b3Quat_identity },
		.material = hullMaterial,
	};

	b3CompoundDef compoundDef = {
		.hulls = &hullChild,
		.hullCount = 1,
		.meshes = &meshChild,
		.meshCount = 1,
	};
	b3CompoundData* compound = b3CreateCompound( &compoundDef );
	ENSURE( compound != NULL );
	ENSURE( compound->materialCount == 3 );

	const b3SurfaceMaterial* bakedMaterials = b3GetCompoundMaterials( compound );
	ENSURE( bakedMaterials[0].userMaterialId == 303 );
	ENSURE( bakedMaterials[1].userMaterialId == 101 );
	ENSURE( bakedMaterials[2].userMaterialId == 202 );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateBakedCompoundShape( bodyId, &shapeDef, compound );

	b3World_Step( worldId, B3_FIX( 1.0f ) / 60, 1 );

	// Mover on top of the hull child face at y = 0.5.
	b3Capsule hullMover = {
		{ -B3_FIX( 8.0f ), B3_FIX( 0.65f ), B3_FIX( 0.0f ) },
		{ -B3_FIX( 8.0f ), B3_FIX( 0.85f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};

	PlaneCapture hullCapture = { 0 };
	b3World_CollideMover( worldId, b3Pos_zero, &hullMover, b3DefaultQueryFilter(), CapturePlaneFcn, &hullCapture );

	ENSURE( hullCapture.count == 1 );
	ENSURE( hullCapture.planes[0].plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE( hullCapture.planes[0].childIndex == 0 );
	ENSURE( hullCapture.planes[0].materialIndex == 0 );
	ENSURE( bakedMaterials[hullCapture.planes[0].materialIndex].userMaterialId == 303 );

	// Mover over the second mesh triangle. The mesh reports triangle material 1, which the
	// compound remaps through the child material table to the shared slot of meshMaterials[1].
	b3Capsule meshMover = {
		{ B3_FIX( 2.0f ), B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};

	PlaneCapture meshCapture = { 0 };
	b3World_CollideMover( worldId, b3Pos_zero, &meshMover, b3DefaultQueryFilter(), CapturePlaneFcn, &meshCapture );

	ENSURE( meshCapture.count == 1 );
	ENSURE( meshCapture.planes[0].plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE( meshCapture.planes[0].childIndex == 1 );
	ENSURE( meshCapture.planes[0].materialIndex == 2 );
	ENSURE( bakedMaterials[meshCapture.planes[0].materialIndex].userMaterialId == 202 );

	b3DestroyWorld( worldId );
	b3DestroyCompound( compound );
	b3DestroyMesh( mesh );
	return 0;
}

static int MoverBodyMaterialIndices( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// One shape of each convex type, spaced out along X.
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	b3BoxHull box = b3MakeTransformedBoxHull(
		B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ),
		(b3Transform){ .p = { B3_FIX( 5.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, .q = b3Quat_identity } );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Capsule capsule = { { B3_FIX( 9.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 11.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };
	b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );

	b3WorldTransform bodyTransform = { .p = b3Pos_zero, .q = b3Quat_identity };

	// Convex shapes report the base material as index 0.
	b3Capsule movers[3] = {
		{ { B3_FIX( 0.0f ), B3_FIX( 1.15f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 1.35f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) },
		{ { B3_FIX( 5.0f ), B3_FIX( 0.65f ), B3_FIX( 0.0f ) }, { B3_FIX( 5.0f ), B3_FIX( 0.85f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) },
		{ { B3_FIX( 10.0f ), B3_FIX( 0.45f ), B3_FIX( 0.0f ) }, { B3_FIX( 10.0f ), B3_FIX( 0.65f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) },
	};

	for ( int i = 0; i < 3; ++i )
	{
		b3BodyPlaneResult planes[4];
		int count = b3Body_CollideMover( bodyId, planes, 4, b3Pos_zero, &movers[i], b3DefaultQueryFilter(), bodyTransform );

		ENSURE( count == 1 );
		ENSURE( b3Shape_IsValid( planes[0].shapeId ) );
		ENSURE( planes[0].result.plane.normal.y > B3_FIX( 0.99f ) );
		ENSURE( planes[0].result.materialIndex == 0 );
		ENSURE( planes[0].result.childIndex == 0 );
	}

	b3DestroyWorld( worldId );
	return 0;
}

static int MoverBodySkipsMeshAndCompound( void )
{
	// The body level mover query handles convex shapes only. Mesh and compound shapes are
	// skipped, so their material indices are only available through b3World_CollideMover.
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();

	b3MeshData* mesh = MakeTwoMaterialMesh();
	ENSURE( mesh != NULL );
	b3CreateMeshShape( bodyId, &shapeDef, mesh, (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } );

	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3CompoundHullDef hullChild = {
		.hull = &box.base,
		.transform = { .p = { B3_FIX( 10.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, .q = b3Quat_identity },
	};
	b3CompoundDef compoundDef = {
		.hulls = &hullChild,
		.hullCount = 1,
	};
	b3CompoundData* compound = b3CreateCompound( &compoundDef );
	ENSURE( compound != NULL );
	b3CreateBakedCompoundShape( bodyId, &shapeDef, compound );

	b3WorldTransform bodyTransform = { .p = b3Pos_zero, .q = b3Quat_identity };

	// Mover over the mesh triangle and mover over the compound hull child both find nothing.
	b3Capsule meshMover = {
		{ -B3_FIX( 2.0f ), B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};
	b3Capsule compoundMover = {
		{ B3_FIX( 10.0f ), B3_FIX( 0.65f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 10.0f ), B3_FIX( 0.85f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};

	b3Capsule movers[2] = { meshMover, compoundMover };
	for ( int i = 0; i < 2; ++i )
	{
		b3BodyPlaneResult planes[4];
		int count = b3Body_CollideMover( bodyId, planes, 4, b3Pos_zero, &movers[i], b3DefaultQueryFilter(), bodyTransform );
		ENSURE( count == 0 );
	}

	b3DestroyWorld( worldId );
	b3DestroyCompound( compound );
	b3DestroyMesh( mesh );
	return 0;
}

// One sided mover collision.
// Mover queries keep only triangles facing the mover. The front side follows the
// baked winding: up for a default mesh or height field, down when the height
// field carries clockwiseWinding. A mirror in the mesh scale must not flip it.
static int MoverMeshBackside( void )
{
	b3MeshData* mesh = MakeTwoMaterialMesh();
	ENSURE( mesh != NULL );
	b3Mesh shape = { .data = mesh, .scale = { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };

	b3PlaneResult planes[4];

	// On the front of the left triangle, so a plane comes back.
	b3Capsule above = {
		{ -B3_FIX( 2.0f ), B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};
	int count = b3CollideMoverAndMesh( planes, 4, &shape, &above );
	ENSURE( count == 1 );
	ENSURE( planes[0].plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE( planes[0].childIndex == 0 );
	ENSURE( planes[0].triangleIndex >= 0 && planes[0].triangleIndex < mesh->triangleCount );

	// The same spot seen from behind the face is culled.
	b3Capsule below = {
		{ -B3_FIX( 2.0f ), -B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		{ -B3_FIX( 2.0f ), -B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};
	count = b3CollideMoverAndMesh( planes, 4, &shape, &below );
	ENSURE( count == 0 );

	b3DestroyMesh( mesh );
	return 0;
}

static int MoverMeshMirroredScale( void )
{
	b3MeshData* mesh = MakeTwoMaterialMesh();
	ENSURE( mesh != NULL );

	// Reflecting the scale flips the triangle winding, the collision swaps it back,
	// so the front stays up. Local x = -2 maps onto the source right triangle, material 1.
	b3Mesh shape = { .data = mesh, .scale = { -B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };

	b3PlaneResult planes[4];

	b3Capsule above = {
		{ -B3_FIX( 2.0f ), B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};
	int count = b3CollideMoverAndMesh( planes, 4, &shape, &above );
	ENSURE( count == 1 );
	ENSURE( planes[0].plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE( planes[0].materialIndex == 1 );
	ENSURE( b3GetMeshMaterialIndices( mesh )[planes[0].triangleIndex] == 1 );

	b3Capsule below = {
		{ -B3_FIX( 2.0f ), -B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		{ -B3_FIX( 2.0f ), -B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};
	count = b3CollideMoverAndMesh( planes, 4, &shape, &below );
	ENSURE( count == 0 );

	b3DestroyMesh( mesh );
	return 0;
}

static int MoverHeightFieldBackside( void )
{
	uint8_t materials[4] = { 0, 0, 0, 0 };
	b3HeightFieldData* hf = MakeFlatField( materials, false );
	ENSURE( hf != NULL );

	b3PlaneResult planes[4];

	// Standing on the front (upper) face of the default winding.
	b3Capsule above = {
		{ B3_FIX( 0.3f ), B3_FIX( 0.15f ), B3_FIX( 0.25f ) },
		{ B3_FIX( 0.3f ), B3_FIX( 0.35f ), B3_FIX( 0.25f ) },
		B3_FIX( 0.2f ),
	};
	int count = b3CollideMoverAndHeightField( planes, 4, hf, &above );
	ENSURE( count == 1 );
	ENSURE( planes[0].plane.normal.y > B3_FIX( 0.99f ) );
	ENSURE_SMALL( planes[0].plane.offset - B3_FIX( 0.05f ), 8 * B3_FIXED_EPSILON );

	// Under the surface is the back side and gets culled.
	b3Capsule below = {
		{ B3_FIX( 0.3f ), -B3_FIX( 0.35f ), B3_FIX( 0.25f ) },
		{ B3_FIX( 0.3f ), -B3_FIX( 0.15f ), B3_FIX( 0.25f ) },
		B3_FIX( 0.2f ),
	};
	count = b3CollideMoverAndHeightField( planes, 4, hf, &below );
	ENSURE( count == 0 );

	b3DestroyHeightField( hf );
	return 0;
}

static int MoverHeightFieldReport( void )
{
	uint8_t materials[4] = { 1, 2, 0, 0 };
	b3HeightFieldData* hf = MakeFlatField( materials, false );
	ENSURE( hf != NULL );

	b3PlaneResult planes[4];

	// (0.3, 0.25) sits on the x + z <= 1 side of cell (0,0), which holds triangle 0.
	b3Capsule first = {
		{ B3_FIX( 0.3f ), B3_FIX( 0.15f ), B3_FIX( 0.25f ) },
		{ B3_FIX( 0.3f ), B3_FIX( 0.35f ), B3_FIX( 0.25f ) },
		B3_FIX( 0.2f ),
	};
	int count = b3CollideMoverAndHeightField( planes, 4, hf, &first );
	ENSURE( count == 1 );
	ENSURE( planes[0].triangleIndex == 0 );
	ENSURE( planes[0].childIndex == 0 );
	ENSURE( planes[0].materialIndex == 1 );

	// (1.3, 0.3) sits on the x + z <= 2 side of cell (1,0), which holds triangle 2.
	b3Capsule second = {
		{ B3_FIX( 1.3f ), B3_FIX( 0.15f ), B3_FIX( 0.3f ) },
		{ B3_FIX( 1.3f ), B3_FIX( 0.35f ), B3_FIX( 0.3f ) },
		B3_FIX( 0.2f ),
	};
	count = b3CollideMoverAndHeightField( planes, 4, hf, &second );
	ENSURE( count == 1 );
	ENSURE( planes[0].triangleIndex == 2 );
	ENSURE( planes[0].materialIndex == 2 );

	b3DestroyHeightField( hf );
	return 0;
}

// A clockwise height field faces down, see HeightFieldWinding. Backside culling
// must respect the flag: below the surface is the front side, above is the back.
static int MoverHeightFieldClockwise( void )
{
	uint8_t materials[4] = { 0, 0, 0, 0 };
	b3HeightFieldData* hf = MakeFlatField( materials, true );
	ENSURE( hf != NULL );

	b3PlaneResult planes[4];

	b3Capsule below = {
		{ B3_FIX( 0.3f ), -B3_FIX( 0.35f ), B3_FIX( 0.25f ) },
		{ B3_FIX( 0.3f ), -B3_FIX( 0.15f ), B3_FIX( 0.25f ) },
		B3_FIX( 0.2f ),
	};
	int count = b3CollideMoverAndHeightField( planes, 4, hf, &below );
	ENSURE( count == 1 );
	ENSURE( planes[0].plane.normal.y < -B3_FIX( 0.99f ) );
	ENSURE_SMALL( planes[0].plane.offset - B3_FIX( 0.05f ), 8 * B3_FIXED_EPSILON );
	ENSURE( planes[0].triangleIndex == 0 || planes[0].triangleIndex == 1 );

	b3Capsule above = {
		{ B3_FIX( 0.3f ), B3_FIX( 0.15f ), B3_FIX( 0.25f ) },
		{ B3_FIX( 0.3f ), B3_FIX( 0.35f ), B3_FIX( 0.25f ) },
		B3_FIX( 0.2f ),
	};
	count = b3CollideMoverAndHeightField( planes, 4, hf, &above );
	ENSURE( count == 0 );

	b3DestroyHeightField( hf );
	return 0;
}

// A mesh can be baked with more materials than the shape carries. The reported index must stay
// inside the shape's material array.
static int MoverWorldMeshMaterialClamp( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MeshData* mesh = MakeTwoMaterialMesh();
	ENSURE( mesh != NULL );
	ENSURE( mesh->materialCount == 2 );

	// Base material only.
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shapeId = b3CreateMeshShape( bodyId, &shapeDef, mesh, (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } );
	int shapeMaterialCount = b3Shape_GetMeshMaterialCount( shapeId );
	ENSURE( shapeMaterialCount == 1 );

	b3World_Step( worldId, B3_FIX( 1.0f ) / 60, 1 );

	// Over the triangle baked with material 1.
	b3Capsule mover = {
		{ B3_FIX( 2.0f ), B3_FIX( 0.15f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.35f ), B3_FIX( 0.0f ) },
		B3_FIX( 0.2f ),
	};
	PlaneCapture capture = { 0 };
	b3World_CollideMover( worldId, b3Pos_zero, &mover, b3DefaultQueryFilter(), CapturePlaneFcn, &capture );

	ENSURE( capture.count == 1 );
	ENSURE( capture.planes[0].materialIndex == shapeMaterialCount - 1 );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
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

	RUN_SUBTEST( MoverWorldMeshMaterials );
	RUN_SUBTEST( MoverWorldCompoundMeshMaterials );
	RUN_SUBTEST( MoverBodyMaterialIndices );
	RUN_SUBTEST( MoverBodySkipsMeshAndCompound );
	RUN_SUBTEST( MoverMeshBackside );
	RUN_SUBTEST( MoverMeshMirroredScale );
	RUN_SUBTEST( MoverHeightFieldBackside );
	RUN_SUBTEST( MoverHeightFieldReport );
	RUN_SUBTEST( MoverHeightFieldClockwise );
	RUN_SUBTEST( MoverWorldMeshMaterialClamp );

	return 0;
}
