// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <string.h>

// Two quads meeting at a concave crease along the shared edge 1-4. The crease exercises
// edge identification, which reads the baked winding rather than the input winding.
#define VALLEY_VERTEX_COUNT 6
#define VALLEY_TRIANGLE_COUNT 4

static void MakeValley( b3Vec3* vertices, int32_t* indices )
{
	b3Vec3 v[VALLEY_VERTEX_COUNT] = {
		{ -B3_FIX( 1.0f ), B3_FIX( 1.0f ), -B3_FIX( 1.0f ) },
		{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
		{ B3_FIX( 1.0f ), B3_FIX( 1.0f ), -B3_FIX( 1.0f ) },
		{ -B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) },
		{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
		{ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) },
	};

	int32_t i[3 * VALLEY_TRIANGLE_COUNT] = {
		0, 3, 1, // left quad
		3, 4, 1,
		1, 4, 2, // right quad
		4, 5, 2,
	};

	memcpy( vertices, v, sizeof( v ) );
	memcpy( indices, i, sizeof( i ) );
}

// A vertex laid out the way a renderer would hand it over. The position sits off the
// front of the struct so a non-zero base offset gets exercised too. b3Vec3 is 8-byte
// aligned here, so the compiler pads after weight and sizeof is a multiple of 8 --
// which is what b3CreateMesh requires of a stride.
typedef struct FatVertex
{
	float weight;
	b3Vec3 position;
	float uv[2];
} FatVertex;

static void MakeFatVertices( FatVertex* fat, const b3Vec3* vertices, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		// Poison the padding so a stride mistake bakes obvious garbage instead of near misses
		fat[i].weight = 1000.0f + (float)i;
		fat[i].position = vertices[i];
		fat[i].uv[0] = -1000.0f;
		fat[i].uv[1] = -2000.0f;
	}
}

static void ReverseWinding( int32_t* indices, int triangleCount )
{
	for ( int i = 0; i < triangleCount; ++i )
	{
		int32_t temp = indices[3 * i + 1];
		indices[3 * i + 1] = indices[3 * i + 2];
		indices[3 * i + 2] = temp;
	}
}

// The hash covers every byte of the mesh block, so this compares the tree, the vertices,
// the baked triangles and the edge flags in one shot.
static bool MeshesMatch( const b3MeshData* mesh1, const b3MeshData* mesh2 )
{
	return mesh1->byteCount == mesh2->byteCount && mesh1->hash == mesh2->hash;
}

// Exact equality rather than a tolerance: welding and the strided copy both move whole
// quanta or nothing at all, so any difference here is a real defect rather than rounding.
static bool VerticesMatch( const b3MeshData* mesh, const b3Vec3* expected, int count )
{
	if ( mesh->vertexCount != count )
	{
		return false;
	}

	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	for ( int i = 0; i < count; ++i )
	{
		if ( vertices[i].x != expected[i].x || vertices[i].y != expected[i].y || vertices[i].z != expected[i].z )
		{
			return false;
		}
	}

	return true;
}

// The valley faces up, so every baked triangle should wind counter clockwise about +Y
static bool FacesUp( const b3MeshData* mesh )
{
	const b3Vec3* vertices = b3GetMeshVertices( mesh );
	const b3MeshTriangle* triangles = b3GetMeshTriangles( mesh );

	for ( int i = 0; i < mesh->triangleCount; ++i )
	{
		b3Vec3 v1 = vertices[triangles[i].index1];
		b3Vec3 v2 = vertices[triangles[i].index2];
		b3Vec3 v3 = vertices[triangles[i].index3];

		b3Vec3 normal = b3Cross( b3Sub( v2, v1 ), b3Sub( v3, v1 ) );
		if ( normal.y <= B3_FIX( 0.0f ) )
		{
			return false;
		}
	}

	return true;
}

static bool HasConcaveEdge( const b3MeshData* mesh )
{
	const uint8_t* flags = b3GetMeshFlags( mesh );

	for ( int i = 0; i < mesh->triangleCount; ++i )
	{
		if ( ( flags[i] & b3_allConcaveEdges ) != 0 )
		{
			return true;
		}
	}

	return false;
}

static b3MeshDef MakeValleyDef( b3Vec3* vertices, int32_t* indices )
{
	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.stride = sizeof( b3Vec3 );
	def.indices = indices;
	def.vertexCount = VALLEY_VERTEX_COUNT;
	def.triangleCount = VALLEY_TRIANGLE_COUNT;
	def.identifyEdges = true;
	return def;
}

// Dense input is the reference the other cases are measured against
static int MeshDenseStride( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef def = MakeValleyDef( vertices, indices );
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	ENSURE( mesh->vertexCount == VALLEY_VERTEX_COUNT );
	ENSURE( mesh->triangleCount == VALLEY_TRIANGLE_COUNT );
	ENSURE( mesh->degenerateCount == 0 );
	ENSURE( VerticesMatch( mesh, vertices, VALLEY_VERTEX_COUNT ) );
	ENSURE( FacesUp( mesh ) );
	ENSURE( HasConcaveEdge( mesh ) );

	b3DestroyMesh( mesh );
	return 0;
}

// A zero stride means contiguous and must bake the same mesh as an explicit dense stride
static int MeshZeroStride( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef denseDef = MakeValleyDef( vertices, indices );
	b3MeshData* denseMesh = b3CreateMesh( &denseDef, NULL, 0 );
	ENSURE( denseMesh != NULL );

	b3MeshDef zeroDef = MakeValleyDef( vertices, indices );
	zeroDef.stride = 0;
	b3MeshData* zeroMesh = b3CreateMesh( &zeroDef, NULL, 0 );
	ENSURE( zeroMesh != NULL );

	ENSURE( MeshesMatch( denseMesh, zeroMesh ) );

	b3DestroyMesh( zeroMesh );
	b3DestroyMesh( denseMesh );
	return 0;
}

// Interleaved input must bake to exactly the same mesh as dense input
static int MeshFatStride( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef denseDef = MakeValleyDef( vertices, indices );
	b3MeshData* denseMesh = b3CreateMesh( &denseDef, NULL, 0 );
	ENSURE( denseMesh != NULL );

	FatVertex fat[VALLEY_VERTEX_COUNT];
	MakeFatVertices( fat, vertices, VALLEY_VERTEX_COUNT );

	b3MeshDef fatDef = MakeValleyDef( &fat[0].position, indices );
	fatDef.stride = sizeof( FatVertex );
	b3MeshData* fatMesh = b3CreateMesh( &fatDef, NULL, 0 );
	ENSURE( fatMesh != NULL );

	ENSURE( VerticesMatch( fatMesh, vertices, VALLEY_VERTEX_COUNT ) );
	ENSURE( FacesUp( fatMesh ) );
	ENSURE( MeshesMatch( denseMesh, fatMesh ) );

	b3DestroyMesh( fatMesh );
	b3DestroyMesh( denseMesh );
	return 0;
}

// A stride that would misalign a b3Vec3 load must be refused rather than read. Upstream
// allows any multiple of 4 because a single precision b3Vec3 is 4-byte aligned; b3Fixed
// is 64 bits here, so the same stride is a misaligned load and b3CreateMesh returns NULL.
static int MeshBadStride( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	// Two packings of the SAME valley. Both are large enough and both carry real vertex
	// data at their own spacing, so the only thing separating them is alignment: 64 is a
	// multiple of alignof( b3Vec3 ) and 28 is not. Without that, a rejection cannot be
	// told apart from a mesh that failed for some other reason -- an earlier version of
	// this test read zeroes through the bad stride, collapsed every triangle, and passed
	// against a build that never checked alignment at all.
	// _Alignas because the buffers are cast to b3Vec3* and read through as 64-bit members.
	// A bare uint8_t array is 1-aligned by the language, so the "aligned" control would be
	// aligned only by the luck of the frame layout -- and ASan does not check alignment, so
	// the config this runs under would not catch it. Stating the alignment makes the aligned
	// buffer aligned by construction and the 28-stride offsets misaligned by construction,
	// which is what the test claims is the only variable between them.
	_Alignas( b3Vec3 ) uint8_t aligned[VALLEY_VERTEX_COUNT * 64];
	_Alignas( b3Vec3 ) uint8_t misaligned[VALLEY_VERTEX_COUNT * 28 + 32];
	memset( aligned, 0, sizeof( aligned ) );
	memset( misaligned, 0, sizeof( misaligned ) );
	for ( int i = 0; i < VALLEY_VERTEX_COUNT; ++i )
	{
		memcpy( aligned + i * 64, &vertices[i], sizeof( b3Vec3 ) );
		memcpy( misaligned + i * 28, &vertices[i], sizeof( b3Vec3 ) );
	}

	b3MeshDef def = MakeValleyDef( vertices, indices );

	// The aligned packing must build the same mesh the dense input does, which pins the
	// refusal below on alignment rather than on the data or the buffer size.
	def.vertices = (b3Vec3*)aligned;
	def.stride = 64;
	b3MeshData* ok = b3CreateMesh( &def, NULL, 0 );
	ENSURE( ok != NULL );
	ENSURE( VerticesMatch( ok, vertices, VALLEY_VERTEX_COUNT ) );
	ENSURE( FacesUp( ok ) );
	b3DestroyMesh( ok );

	// Same valley, same completeness, misaligned for a 64-bit member. Upstream's 4-byte
	// check accepts this and builds a mesh from misaligned loads.
	def.vertices = (b3Vec3*)misaligned;
	def.stride = 28;
	ENSURE( b3CreateMesh( &def, NULL, 0 ) == NULL );

	def.vertices = vertices;

	// Smaller than one vertex
	def.stride = sizeof( b3Vec3 ) - 8;
	ENSURE( b3CreateMesh( &def, NULL, 0 ) == NULL );

	// Beyond the sanity ceiling, which usually means uninitialized memory
	def.stride = 8192;
	ENSURE( b3CreateMesh( &def, NULL, 0 ) == NULL );

	return 0;
}

// Welding reads the source vertices through the stride as well
static int MeshStrideWeld( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	// Split the crease so each quad owns a copy of the shared edge
	b3Vec3 splitVertices[8];
	memcpy( splitVertices, vertices, sizeof( vertices ) );
	splitVertices[6] = vertices[1];
	splitVertices[7] = vertices[4];

	// Point the right quad at the copies
	int32_t splitIndices[3 * VALLEY_TRIANGLE_COUNT];
	memcpy( splitIndices, indices, sizeof( indices ) );
	splitIndices[6] = 6;
	splitIndices[7] = 7;
	splitIndices[9] = 7;

	FatVertex fat[8];
	MakeFatVertices( fat, splitVertices, 8 );

	b3MeshDef def = MakeValleyDef( &fat[0].position, splitIndices );
	def.stride = sizeof( FatVertex );
	def.vertexCount = 8;
	def.weldVertices = true;
	def.weldTolerance = B3_FIX( 0.01f );

	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	ENSURE( mesh->vertexCount == VALLEY_VERTEX_COUNT );
	ENSURE( mesh->triangleCount == VALLEY_TRIANGLE_COUNT );
	ENSURE( VerticesMatch( mesh, vertices, VALLEY_VERTEX_COUNT ) );
	ENSURE( FacesUp( mesh ) );
	ENSURE( HasConcaveEdge( mesh ) );

	b3DestroyMesh( mesh );
	return 0;
}

// Clockwise input plus the flag must bake to the same mesh as counter clockwise input
static int MeshClockWise( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef ccwDef = MakeValleyDef( vertices, indices );
	b3MeshData* ccwMesh = b3CreateMesh( &ccwDef, NULL, 0 );
	ENSURE( ccwMesh != NULL );

	int32_t reversed[3 * VALLEY_TRIANGLE_COUNT];
	memcpy( reversed, indices, sizeof( indices ) );
	ReverseWinding( reversed, VALLEY_TRIANGLE_COUNT );

	b3MeshDef cwDef = MakeValleyDef( vertices, reversed );
	cwDef.clockWiseWinding = true;
	b3MeshData* cwMesh = b3CreateMesh( &cwDef, NULL, 0 );
	ENSURE( cwMesh != NULL );

	ENSURE( FacesUp( cwMesh ) );
	ENSURE( HasConcaveEdge( cwMesh ) );
	ENSURE( MeshesMatch( ccwMesh, cwMesh ) );

	b3DestroyMesh( cwMesh );
	b3DestroyMesh( ccwMesh );
	return 0;
}

// Without the flag the same clockwise input must bake inside out
static int MeshClockWiseIgnored( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );
	ReverseWinding( indices, VALLEY_TRIANGLE_COUNT );

	b3MeshDef def = MakeValleyDef( vertices, indices );
	b3MeshData* mesh = b3CreateMesh( &def, NULL, 0 );
	ENSURE( mesh != NULL );

	ENSURE( FacesUp( mesh ) == false );

	b3DestroyMesh( mesh );
	return 0;
}

// Winding, stride and welding have to compose
static int MeshClockWiseStrideWeld( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef ccwDef = MakeValleyDef( vertices, indices );
	ccwDef.weldVertices = true;
	ccwDef.weldTolerance = B3_FIX( 0.01f );
	b3MeshData* ccwMesh = b3CreateMesh( &ccwDef, NULL, 0 );
	ENSURE( ccwMesh != NULL );

	int32_t reversed[3 * VALLEY_TRIANGLE_COUNT];
	memcpy( reversed, indices, sizeof( indices ) );
	ReverseWinding( reversed, VALLEY_TRIANGLE_COUNT );

	FatVertex fat[VALLEY_VERTEX_COUNT];
	MakeFatVertices( fat, vertices, VALLEY_VERTEX_COUNT );

	b3MeshDef cwDef = MakeValleyDef( &fat[0].position, reversed );
	cwDef.stride = sizeof( FatVertex );
	cwDef.clockWiseWinding = true;
	cwDef.weldVertices = true;
	cwDef.weldTolerance = B3_FIX( 0.01f );
	b3MeshData* cwMesh = b3CreateMesh( &cwDef, NULL, 0 );
	ENSURE( cwMesh != NULL );

	ENSURE( FacesUp( cwMesh ) );
	ENSURE( MeshesMatch( ccwMesh, cwMesh ) );

	b3DestroyMesh( cwMesh );
	b3DestroyMesh( ccwMesh );
	return 0;
}

// The degenerate report is written before the count is bumped, so the first slot is
// filled and the last slot in the caller's buffer is usable.
static int MeshDegenerateReport( void )
{
	// A square split into two triangles, plus two zero-area slivers with distinct indices
	b3Vec3 vertices[6] = {
		{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
		{ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
		{ B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
	};

	int32_t indices[12] = {
		0, 2, 1, // real
		1, 4, 5, // collinear, zero area, distinct indices
		0, 3, 2, // real
		0, 4, 5, // collinear, zero area, distinct indices
	};

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.indices = indices;
	def.vertexCount = 6;
	def.triangleCount = 4;

	int reported[2] = { -1, -1 };
	b3MeshData* mesh = b3CreateMesh( &def, reported, 2 );
	ENSURE( mesh != NULL );

	ENSURE( mesh->degenerateCount == 2 );
	ENSURE( mesh->triangleCount == 2 );

	// Both degenerates fit in the caller's buffer. Slot 0 alone does not discriminate --
	// the old ordering incremented first and then wrote [count - 1], which lands on 0 too.
	// What it never wrote is the LAST slot, because the guard had already consumed the
	// increment, so it filled capacity - 1 entries and reported capacity of them.
	ENSURE( reported[0] == 1 );
	ENSURE( reported[1] == 3 );

	b3DestroyMesh( mesh );
	return 0;
}

// Winding under a scaled mesh. b3QueryMesh reports each triangle in winding order, so the
// sign of the reported normal is a direct read of the winding decision -- no culling, no
// solver, nothing else in the way.
typedef struct WindingContext
{
	int triangleCount;
	int upCount;
} WindingContext;

static bool CountWinding( b3Vec3 a, b3Vec3 b, b3Vec3 c, int triangleIndex, void* context )
{
	(void)triangleIndex;
	WindingContext* wc = (WindingContext*)context;
	b3Vec3 normal = b3Cross( b3Sub( b, a ), b3Sub( c, a ) );
	wc->triangleCount += 1;
	if ( normal.y > B3_FIX( 0.0f ) )
	{
		wc->upCount += 1;
	}

	return true;
}

static int QueryWindingUpCount( b3MeshData* data, b3Vec3 scale, int* triangleCount )
{
	b3Mesh mesh = { data, scale };
	b3AABB bounds = {
		{ -B3_FIX( 1000.0f ), -B3_FIX( 1000.0f ), -B3_FIX( 1000.0f ) },
		{ B3_FIX( 1000.0f ), B3_FIX( 1000.0f ), B3_FIX( 1000.0f ) },
	};

	WindingContext wc = { 0, 0 };
	b3QueryMesh( &mesh, bounds, CountWinding, &wc );
	*triangleCount = wc.triangleCount;
	return wc.upCount;
}

// THE INVARIANT: a scale changes a mesh's size and handedness, never which way its surface
// faces. A mirroring scale flips the geometric normal, and the winding swap exists to undo
// exactly that -- so an upward facing valley must report upward facing triangles under
// every legal scale, positive or reflected, large or small.
//
// The decision is the SIGN of the scale determinant, and forming that determinant with
// b3FixMul quantizes to zero for legal small scales. Measured in Q48.16: the product of
// (0.01, 0.01, 0.01) is exactly 0, and so is the product of (0.01, 0.01, -0.01). A test on
// the product then answers wrongly for both, in opposite directions.
static int MeshScaledWinding( void )
{
	b3Vec3 vertices[VALLEY_VERTEX_COUNT];
	int32_t indices[3 * VALLEY_TRIANGLE_COUNT];
	MakeValley( vertices, indices );

	b3MeshDef def = MakeValleyDef( vertices, indices );
	b3MeshData* data = b3CreateMesh( &def, NULL, 0 );
	ENSURE( data != NULL );

	b3Vec3 scales[] = {
		// Determinant safely positive: holds under any spelling, so it is the reference
		{ B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) },

		// Determinant safely negative: also holds under any spelling
		{ B3_FIX( 1.0f ), B3_FIX( 1.0f ), -B3_FIX( 1.0f ) },

		// Determinant positive, product quantizes to zero. Upstream's `product > 0` reads
		// this as clockwise and swaps every triangle.
		{ B3_FIX( 0.01f ), B3_FIX( 0.01f ), B3_FIX( 0.01f ) },

		// Determinant negative, product quantizes to zero. The older `product < 0` reads
		// this as counter clockwise and fails to swap.
		{ B3_FIX( 0.01f ), B3_FIX( 0.01f ), -B3_FIX( 0.01f ) },

		// Non-uniform, two reflected axes, determinant positive and small
		{ -B3_FIX( 0.02f ), B3_FIX( 0.01f ), -B3_FIX( 0.05f ) },
	};

	for ( int i = 0; i < ARRAY_COUNT( scales ); ++i )
	{
		int count = 0;
		int up = QueryWindingUpCount( data, scales[i], &count );
		ENSURE( count == VALLEY_TRIANGLE_COUNT );
		ENSURE( up == VALLEY_TRIANGLE_COUNT );
	}

	b3DestroyMesh( data );
	return 0;
}

// A flat one-quad mesh in the XZ plane, facing up (+Y)
#define PLATE_VERTEX_COUNT 4
#define PLATE_TRIANGLE_COUNT 2

static b3MeshData* MakePlate( void )
{
	static b3Vec3 vertices[PLATE_VERTEX_COUNT] = {
		{ -B3_FIX( 2.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.0f ), -B3_FIX( 2.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 2.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 2.0f ) },
	};

	// CCW seen from above, so the surface normal is +Y
	static int32_t indices[3 * PLATE_TRIANGLE_COUNT] = { 0, 3, 1, 1, 3, 2 };

	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.indices = indices;
	def.vertexCount = PLATE_VERTEX_COUNT;
	def.triangleCount = PLATE_TRIANGLE_COUNT;
	return b3CreateMesh( &def, NULL, 0 );
}

// Cast a small sphere along a translation and report whether it hit the plate
static bool CastSphereAtPlate( b3MeshData* data, b3Vec3 start, b3Vec3 translation )
{
	b3Mesh mesh = { data, { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) } };

	b3Vec3 points[1] = { start };
	b3ShapeCastInput input = { 0 };
	input.proxy = (b3ShapeProxy){ points, 1, B3_FIX( 0.1f ) };
	input.translation = translation;
	input.maxFraction = B3_FIX( 1.0f );
	input.canEncroach = false;

	b3CastOutput output = b3ShapeCastMesh( &mesh, &input );
	return output.hit;
}

// Back-side culling. Upstream added it to shape casts against meshes and height fields, and
// this is the ONLY test that makes it fire: instrumented across the whole suite before this
// existed, the cull ran 1992 times and fired 0 times, so every determinism golden held for
// the uninteresting reason that nothing was ever culled. An unmoved golden is not evidence
// about a branch that never taken.
static int MeshBackSideCull( void )
{
	b3MeshData* plate = MakePlate();
	ENSURE( plate != NULL );

	// From above, moving down: the front side. Still hits.
	b3Vec3 above = { B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	b3Vec3 down = { B3_FIX( 0.0f ), -B3_FIX( 2.0f ), B3_FIX( 0.0f ) };
	ENSURE( CastSphereAtPlate( plate, above, down ) == true );

	// From below, moving up: the back side. Culled.
	b3Vec3 below = { B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	b3Vec3 up = { B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) };
	ENSURE( CastSphereAtPlate( plate, below, up ) == false );

	// A cast that starts below and never reaches the plate misses either way, which keeps
	// the case above from passing merely because the sphere fell short.
	b3Vec3 shortUp = { B3_FIX( 0.0f ), B3_FIX( 0.1f ), B3_FIX( 0.0f ) };
	ENSURE( CastSphereAtPlate( plate, below, shortUp ) == false );

	b3DestroyMesh( plate );
	return 0;
}

// The built in creators fill their own def, so a missed stride shows up here
static int MeshCreators( void )
{
	b3Vec3 center = { B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 3.0f ) };
	b3Vec3 extent = { B3_FIX( 0.5f ), B3_FIX( 1.0f ), B3_FIX( 1.5f ) };

	b3MeshData* meshes[] = {
		b3CreateGridMesh( 4, 4, B3_FIX( 1.0f ), 2, true ),
		b3CreateWaveMesh( 4, 4, B3_FIX( 1.0f ), B3_FIX( 0.5f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) ),
		b3CreateTorusMesh( 8, 6, B3_FIX( 2.0f ), B3_FIX( 0.5f ) ),
		b3CreateBoxMesh( center, extent, true ),
		b3CreateHollowBoxMesh( center, extent ),
		b3CreatePlatformMesh( center, B3_FIX( 2.0f ), B3_FIX( 1.0f ), B3_FIX( 2.0f ) ),
	};

	for ( int i = 0; i < ARRAY_COUNT( meshes ); ++i )
	{
		ENSURE( meshes[i] != NULL );
		ENSURE( meshes[i]->vertexCount >= 3 );
		ENSURE( meshes[i]->triangleCount >= 1 );
		ENSURE( b3IsSaneAABB( meshes[i]->bounds ) );

		b3DestroyMesh( meshes[i] );
	}

	return 0;
}

int MeshTest( void )
{
	RUN_SUBTEST( MeshDenseStride );
	RUN_SUBTEST( MeshZeroStride );
	RUN_SUBTEST( MeshFatStride );
	RUN_SUBTEST( MeshBadStride );
	RUN_SUBTEST( MeshStrideWeld );
	RUN_SUBTEST( MeshClockWise );
	RUN_SUBTEST( MeshClockWiseIgnored );
	RUN_SUBTEST( MeshClockWiseStrideWeld );
	RUN_SUBTEST( MeshDegenerateReport );
	RUN_SUBTEST( MeshScaledWinding );
	RUN_SUBTEST( MeshBackSideCull );
	RUN_SUBTEST( MeshCreators );

	return 0;
}
