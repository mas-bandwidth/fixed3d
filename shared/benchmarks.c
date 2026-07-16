// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"

#include "human.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#ifdef NDEBUG
#define BENCHMARK_DEBUG 0
#else
#define BENCHMARK_DEBUG 1
#endif

static b3ShapeId g_groundShapeId = { 0 };

b3ShapeId GetGroundShapeId( void )
{
	return g_groundShapeId;
}

void ResetGroundShapeId( void )
{
	g_groundShapeId = b3_nullShapeId;
}

void CreateJointGrid( b3WorldId worldId )
{
	b3World_EnableSleeping( worldId, false );

	int n = BENCHMARK_DEBUG ? 10 : 100;

	// Allocate to avoid huge stack usage
	b3BodyId* bodies = malloc( n * n * sizeof( b3BodyId ) );
	int index = 0;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ~2u;

	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.4f ) };

	b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.enableSleep = false;

	for ( int k = 0; k < n; ++k )
	{
		for ( int i = 0; i < n; ++i )
		{
			b3Fixed fk = (b3Fixed)b3FixFromInt( k );
			b3Fixed fi = (b3Fixed)b3FixFromInt( i );

			if ( i == 0 )
			{
				bodyDef.type = b3_staticBody;
			}
			else
			{
				bodyDef.type = b3_dynamicBody;
			}

			bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ fk, -fi, B3_FIX( 0.0f ) } );

			b3BodyId body = b3CreateBody( worldId, &bodyDef );

			b3CreateSphereShape( body, &shapeDef, &sphere );

			if ( i > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - 1];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 0.5f ), B3_FIX( 0.0f ) };
				jointDef.base.localFrameB.p = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) };
				b3CreateSphericalJoint( worldId, &jointDef );
			}

			if ( k > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - n];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b3Vec3){ B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
				jointDef.base.localFrameB.p = (b3Vec3){ -B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
				b3CreateSphericalJoint( worldId, &jointDef );
			}

			bodies[index++] = body;
		}
	}

	free( bodies );
}

// The 80 block version falls over after 1000 steps.
void CreateLargePyramid( b3WorldId worldId )
{
	b3World_EnableSleeping( worldId, false );

	int baseCount = BENCHMARK_DEBUG ? 20 : 100;

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( B3_FIX( 400.0f ), B3_FIX( 1.0f ), B3_FIX( 400.0f ) );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 100.0f );

	b3Fixed h = B3_FIX( 0.5f );
	b3BoxHull box = b3MakeBoxHull( h, h, h );

	b3Fixed shift = b3FixMul( B3_FIX( 1.0f ) , h );

	for ( int i = 0; i < baseCount; ++i )
	{
		b3Fixed y = b3FixMul( ( b3FixMul( B3_FIX( 2.0f ) , b3FixFromInt( i ) ) + B3_FIX( 1.0f ) ) , shift );

		for ( int j = i; j < baseCount; ++j )
		{
			b3Fixed x = b3FixMul( ( b3FixFromInt( i ) + B3_FIX( 1.0f ) ) , shift ) + b3FixMul( b3FixMul( B3_FIX( 2.0f ) , b3FixFromInt( ( j - i ) ) ) , shift ) - b3FixMul( h , b3FixFromInt( baseCount ) );

			bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ x, y, B3_FIX( 0.0f ) } );

			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

void CreateWidePyramid( b3WorldId worldId )
{
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( B3_FIX( 100.0f ), B3_FIX( 1.0f ), B3_FIX( 100.0f ) );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	const b3Fixed boxSize = B3_FIX( 2.0f );
	const b3Fixed boxSeparation = B3_FIX( 0.5f );
	const b3Fixed halfBoxSize = b3FixMul( B3_FIX( 0.5f ) , boxSize );
	const int pyramidHeight = BENCHMARK_DEBUG ? 5 : 15;

	b3Fixed h = b3FixFromDouble( b3FixToDouble( halfBoxSize ) - 0.025 );
	b3BoxHull box = b3MakeBoxHull( h, h, h );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();

	for ( int i = 0; i < pyramidHeight; ++i )
	{
		for ( int j = i / 2; j < pyramidHeight - ( i + 1 ) / 2; ++j )
		{
			for ( int k = i / 2; k < pyramidHeight - ( i + 1 ) / 2; ++k )
			{
				b3Fixed x = b3FixFromInt( -pyramidHeight ) + b3FixMul( boxSize , b3FixFromInt( j ) ) + ( i & 1 ? halfBoxSize : B3_FIX( 0.0f ) );
				b3Fixed y = B3_FIX( 1.0f ) + b3FixMul( ( boxSize + boxSeparation ) , b3FixFromInt( i ) );
				b3Fixed z = b3FixFromInt( -pyramidHeight ) + b3FixMul( boxSize , b3FixFromInt( k ) ) + ( i & 1 ? halfBoxSize : B3_FIX( 0.0f ) );
				bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ x, y, z } );

				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}
}

static void CreateSmallPyramid( b3WorldId worldId, int baseCount, b3Fixed extent, b3Fixed centerX, b3Fixed baseZ )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 100.0f );

	b3BoxHull box = b3MakeBoxHull( extent, extent, extent );

	for ( int i = 0; i < baseCount; ++i )
	{
		b3Fixed y = b3FixMul( ( b3FixMul( B3_FIX( 2.0f ) , b3FixFromInt( i ) ) + B3_FIX( 1.0f ) ) , extent );

		for ( int j = i; j < baseCount; ++j )
		{
			b3Fixed x = b3FixMul( ( b3FixFromInt( i ) + B3_FIX( 1.0f ) ) , extent ) + b3FixMul( b3FixMul( B3_FIX( 2.0f ) , b3FixFromInt( ( j - i ) ) ) , extent ) + centerX - B3_FIX( 0.5f );
			bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ x, y, baseZ } );

			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}
}

void CreateManyPyramids( b3WorldId worldId )
{
	int baseCount = 10;
	b3Fixed extent = B3_FIX( 0.5f );
	int rowCount = BENCHMARK_DEBUG ? 3 : 14;
	int columnCount = BENCHMARK_DEBUG ? 3 : 14;
	b3Fixed groundExtent = b3FixMul( b3FixMul( extent , b3FixFromInt( columnCount ) ) , ( b3FixFromInt( baseCount ) + B3_FIX( 1.0f ) ) );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3BoxHull box = b3MakeBoxHull( groundExtent, B3_FIX( 1.0f ), groundExtent );
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3Fixed baseWidth = b3FixMul( b3FixMul( B3_FIX( 2.0f ) , extent ) , b3FixFromInt( baseCount ) );
	b3Fixed baseZ = -groundExtent + b3FixMul( B3_FIX( 2.0f ) , extent );
	b3Fixed deltaZ = b3FixDiv( b3FixMul( B3_FIX( 2.0f ) , ( groundExtent - b3FixMul( B3_FIX( 2.0f ) , extent ) ) ) , ( b3FixFromInt( rowCount ) - B3_FIX( 1.0f ) ) );

	for ( int i = 0; i < rowCount; ++i )
	{
		for ( int j = 0; j < columnCount; ++j )
		{
			b3Fixed centerX = -groundExtent + b3FixMul( b3FixFromInt( j ) , ( baseWidth + b3FixMul( B3_FIX( 2.0f ) , extent ) ) ) + b3FixMul( B3_FIX( 2.0f ) , extent );
			CreateSmallPyramid( worldId, baseCount, extent, centerX, baseZ );
		}

		baseZ += deltaZ;
	}
}

#define RAIN_GRID_SIZE B3_FIX( 15.0f )
#define RAIN_LARGE_WORLD 0
#ifdef NDEBUG
#if RAIN_LARGE_WORLD == 1
#define RAIN_GRID_COUNT 700
#define RAIN_GROUP_SIZE 1
#else
#define RAIN_GRID_COUNT 10
#define RAIN_GROUP_SIZE 3
#endif

#else

#if RAIN_LARGE_WORLD == 1
#define RAIN_GRID_COUNT 12
#define RAIN_GROUP_SIZE 1
#else
#define RAIN_GRID_COUNT 3
#define RAIN_GROUP_SIZE 2
#endif
#endif

typedef struct Group
{
	Human humans[RAIN_GROUP_SIZE];
} Group;

typedef struct RainData
{
	Group* groups;
	b3MeshData* gridMesh;
	b3MeshData* torusMesh;
	int columnCount;
	int columnIndex;
} RainData;

RainData g_rainData;

void GetRainCapacity( b3Capacity* capacity )
{
#if RAIN_LARGE_WORLD == 1
#ifdef NDEBUG
	capacity->staticShapeCount = 1000 * 1024;
	capacity->dynamicShapeCount = 2 * 1024;
	capacity->staticBodyCount = 500 * 1024;
	capacity->dynamicBodyCount = 2 * 1024;
#else
	capacity->staticShapeCount = 512;
	capacity->dynamicShapeCount = 128;
	capacity->staticBodyCount = 256;
	capacity->dynamicBodyCount = 128;
#endif
#endif
}

void CreateRain( b3WorldId worldId )
{
	memset( &g_rainData, 0, sizeof( g_rainData ) );

	g_rainData.groups = malloc( RAIN_GRID_COUNT * RAIN_GRID_COUNT * sizeof( Group ) );
	memset( g_rainData.groups, 0, RAIN_GRID_COUNT * RAIN_GRID_COUNT * sizeof( Group ) );

	int halfMeshGridRows = 4;
	b3Fixed meshGridCellWidth = b3FixDiv( RAIN_GRID_SIZE , ( b3FixMul( B3_FIX( 2.0f ) , b3FixFromInt( halfMeshGridRows ) ) ) );
	g_rainData.gridMesh = b3CreateGridMesh( 2 * halfMeshGridRows, 2 * halfMeshGridRows, meshGridCellWidth, 1, true );
	g_rainData.torusMesh = b3CreateTorusMesh( 16, 16, b3FixMul( B3_FIX( 0.25f ) , RAIN_GRID_SIZE ), B3_FIX( 1.0f ) );

	b3Fixed span = RAIN_GRID_SIZE * RAIN_GRID_COUNT;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	b3Vec3 local = { 0 };
	local.x = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( B3_FIX( 0.5f ) , RAIN_GRID_SIZE );
	for ( int i = 0; i < RAIN_GRID_COUNT; ++i )
	{
		local.z = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( B3_FIX( 0.5f ) , RAIN_GRID_SIZE );
		for ( int j = 0; j < RAIN_GRID_COUNT; ++j )
		{
			bodyDef.position = b3OffsetPos( GetSceneOrigin(), local );
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, g_rainData.gridMesh, b3Vec3_one );
			b3CreateMeshShape( body, &shapeDef, g_rainData.torusMesh, b3Vec3_one );

			local.z += RAIN_GRID_SIZE;
		}

		local.x += RAIN_GRID_SIZE;
	}

	// b3World_SetJointTuning( worldId, 60.0f, 1.0f );
}

void DestroyRain( void )
{
	b3DestroyMesh( g_rainData.gridMesh );
	b3DestroyMesh( g_rainData.torusMesh );

	free( g_rainData.groups );
	g_rainData.groups = NULL;
}

void CreateGroup( b3WorldId worldId, int rowIndex, int columnIndex )
{
	assert( rowIndex < RAIN_GRID_COUNT && columnIndex < RAIN_GRID_COUNT );

	int groupIndex = rowIndex * RAIN_GRID_COUNT + columnIndex;

	b3Fixed span = RAIN_GRID_COUNT * RAIN_GRID_SIZE;
	b3Fixed groupDistance = span / RAIN_GRID_COUNT;

	// local offset from the scene origin (b3OffsetPos adds it onto GetSceneOrigin)
	b3Vec3 position;
	position.x = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( groupDistance , ( b3FixFromInt( columnIndex ) + B3_FIX( 0.5f ) ) );
	position.y = B3_FIX( 20.0f );
	position.z = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( groupDistance , ( b3FixFromInt( rowIndex ) + B3_FIX( 0.5f ) ) );

	b3Fixed frictionTorque = B3_FIX( 5.0f );
	b3Fixed hertz = B3_FIX( 1.0f );
	b3Fixed dampingRatio = B3_FIX( 0.7f );
	bool colorize = false;

	for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
	{
		Human* human = g_rainData.groups[groupIndex].humans + i;
		CreateHuman( human, worldId, b3OffsetPos( GetSceneOrigin(), position ), frictionTorque, hertz, dampingRatio, groupIndex, NULL, colorize );
		position.x += B3_FIX( 0.75f );
	}
}

void DestroyGroup( int rowIndex, int columnIndex )
{
	assert( rowIndex < RAIN_GRID_COUNT && columnIndex < RAIN_GRID_COUNT );

	int groupIndex = rowIndex * RAIN_GRID_COUNT + columnIndex;

	for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
	{
		DestroyHuman( g_rainData.groups[groupIndex].humans + i );
	}
}

void StepRain( b3WorldId worldId, int stepCount )
{
	int delay = BENCHMARK_DEBUG ? 0x7F : 0x2F;
	int increment = RAIN_LARGE_WORLD ? 100 : 1;

	if ( ( stepCount & delay ) == 0 )
	{
		if ( g_rainData.columnCount < RAIN_GRID_COUNT )
		{
			for ( int i = 0; i < RAIN_GRID_COUNT; i += increment )
			{
				CreateGroup( worldId, i, g_rainData.columnCount );
			}

			g_rainData.columnCount = b3MinInt( g_rainData.columnCount + increment, RAIN_GRID_COUNT );
		}
		else
		{
			for ( int i = 0; i < RAIN_GRID_COUNT; i += increment )
			{
				DestroyGroup( i, g_rainData.columnIndex );
				CreateGroup( worldId, i, g_rainData.columnIndex );
			}

			g_rainData.columnIndex = g_rainData.columnIndex + increment;
			if ( g_rainData.columnIndex >= RAIN_GRID_COUNT )
			{
				g_rainData.columnIndex = 0;
			}
		}
	}
}

// Static Floor: a huge grid of static box bodies with b3ShapeDef::invokeContactCreation = true,
// plus a small number of dynamic spheres that drop onto it staggered over time. The point of this
// benchmark is to exercise the b3BroadPhase move buffer at steady state when it was once populated
// with ~staticShapeCount entries and now sees only a handful of dynamic moves per step.
#define STATIC_FLOOR_CELL_SIZE B3_FIX( 10.0f )
#if BENCHMARK_DEBUG
#define STATIC_FLOOR_GRID 32
#define STATIC_FLOOR_SPHERES 16
#define STATIC_FLOOR_DROP_INTERVAL 8
#else
#define STATIC_FLOOR_GRID 1000
#define STATIC_FLOOR_SPHERES 100
#define STATIC_FLOOR_DROP_INTERVAL 5
#endif

typedef struct StaticFloorData
{
	int spheresDropped;
} StaticFloorData;

static StaticFloorData g_staticFloorData;

void GetLargeWorldCapacity( b3Capacity* capacity )
{
	int floorCount = STATIC_FLOOR_GRID * STATIC_FLOOR_GRID;
	capacity->staticShapeCount = floorCount;
	capacity->staticBodyCount = floorCount;
	capacity->dynamicShapeCount = STATIC_FLOOR_SPHERES;
	capacity->dynamicBodyCount = STATIC_FLOOR_SPHERES;
	capacity->contactCount = b3MaxInt( 1024, 8 * STATIC_FLOOR_SPHERES );
}

void CreateLargeWorld( b3WorldId worldId )
{
	memset( &g_staticFloorData, 0, sizeof( g_staticFloorData ) );

	b3Fixed cell = STATIC_FLOOR_CELL_SIZE;
	int gridCount = STATIC_FLOOR_GRID;
	b3Fixed halfSpan = b3FixMul( b3FixMul( B3_FIX( 0.5f ) , cell ) , b3FixFromInt( gridCount ) );

	b3BoxHull box = b3MakeBoxHull( b3FixMul( B3_FIX( 0.5f ) , cell ), B3_FIX( 0.25f ), b3FixMul( B3_FIX( 0.5f ) , cell ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// The trigger: every static shape gets buffered into the move set on creation.
	shapeDef.invokeContactCreation = true;

	for ( int i = 0; i < gridCount; ++i )
	{
		b3Fixed x = -halfSpan + b3FixMul( ( b3FixFromInt( i ) + B3_FIX( 0.5f ) ) , cell );
		for ( int j = 0; j < gridCount; ++j )
		{
			b3Fixed z = -halfSpan + b3FixMul( ( b3FixFromInt( j ) + B3_FIX( 0.5f ) ) , cell );
			bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ x, B3_FIX( 0.0f ), z } );
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( body, &shapeDef, &box.base );
		}
	}
}

void StepLargeWorld( b3WorldId worldId, int stepCount )
{
	if ( g_staticFloorData.spheresDropped >= STATIC_FLOOR_SPHERES )
	{
		return;
	}

	if ( stepCount == 0 )
	{
		return;
	}

	if ( ( stepCount % STATIC_FLOOR_DROP_INTERVAL ) != 0 )
	{
		return;
	}

	// Spread spheres in a coarse grid across the floor so they don't all pile on one box.
	int side = 1;
	while ( side * side < STATIC_FLOOR_SPHERES )
	{
		side += 1;
	}

	int idx = g_staticFloorData.spheresDropped;
	int gi = idx % side;
	int gj = idx / side;

	b3Fixed halfSpan = b3FixMul( B3_FIX( 0.5f ) , STATIC_FLOOR_CELL_SIZE ) * STATIC_FLOOR_GRID;
	// Confine drops to the inner 80% of the floor so spheres can't roll off the edge.
	// Exact arithmetic: the previous B3_FIX(0.1f) product chain quantized the
	// inset to 1000.061, landing every sphere 55mm off the box seams the float
	// build lands on exactly. The off-center impact catches the neighbor box's
	// top edge, kicks the sphere into eternal rolling (zero rolling resistance,
	// pure rolling defeats friction), nothing ever sleeps, and the benchmark
	// measures an ever-growing awake set instead of the float-equivalent
	// scenario. 20% of the half span is exact in both number systems.
	b3Fixed inset = halfSpan / 5;
	b3Fixed usable = 2 * halfSpan - 2 * inset;
	b3Fixed x = -halfSpan + inset + b3FixMul( ( b3FixFromInt( gi ) + B3_FIX( 0.5f ) ) , ( b3FixDiv( usable , b3FixFromInt( side ) ) ) );
	b3Fixed z = -halfSpan + inset + b3FixMul( ( b3FixFromInt( gj ) + B3_FIX( 0.5f ) ) , ( b3FixDiv( usable , b3FixFromInt( side ) ) ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ x, B3_FIX( 1.5f ), z } );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };

	b3BodyId body = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( body, &shapeDef, &sphere );

	g_staticFloorData.spheresDropped += 1;
}

void GetWasherCapacity( b3Capacity* capacity )
{
	capacity->staticShapeCount = 16;
	capacity->dynamicShapeCount = 10000;
	capacity->staticBodyCount = 16;
	capacity->dynamicBodyCount = 10000;
	capacity->contactCount = 60000;
}

void CreateWasher( b3WorldId worldId )
{
	bool kinematic = true;

	b3BodyId groundId;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( B3_FIX( 60.0f ), B3_FIX( 1.0f ), B3_FIX( 60.0f ) );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	{
		b3Fixed motorSpeed = B3_FIX( 25.0f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 21.0f ), B3_FIX( 0.0f ) } );

		if ( kinematic == true )
		{
			bodyDef.type = b3_kinematicBody;
			bodyDef.angularVelocity = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), b3FixMul( ( b3FixDiv( B3_PI , B3_FIX( 180.0f ) ) ) , motorSpeed ) };
			bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 0.001f ), -B3_FIX( 0.002f ), B3_FIX( 0.0f ) };
		}
		else
		{
			bodyDef.type = b3_dynamicBody;
		}

		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();

		b3Fixed r0 = B3_FIX( 14.0f );
		b3Fixed r1 = B3_FIX( 16.0f );
		b3Fixed r2 = B3_FIX( 18.0f );
		b3Vec3 nd = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), -B3_FIX( 10.0f ) };
		b3Vec3 pd = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 10.0f ) };

		b3Fixed angle = b3FixDiv( B3_PI , B3_FIX( 18.0f ) );
		b3Quat q = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, angle );
		b3Quat qo = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, b3FixMul( B3_FIX( 0.1f ) , angle ) );
		b3Vec3 u1 = { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
		for ( int i = 0; i < 36; ++i )
		{
			b3Vec3 u2;
			if ( i == 35 )
			{
				u2 = (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
			}
			else
			{
				u2 = b3RotateVector( q, u1 );
			}

			{
				b3Vec3 a1 = b3InvRotateVector( qo, u1 );
				b3Vec3 a2 = b3RotateVector( qo, u2 );
				b3Vec3 p1 = b3MulAdd( nd, r1, a1 );
				b3Vec3 p2 = b3MulAdd( nd, r2, a1 );
				b3Vec3 p3 = b3MulAdd( nd, r1, a2 );
				b3Vec3 p4 = b3MulAdd( nd, r2, a2 );
				b3Vec3 p5 = b3MulAdd( pd, r1, a1 );
				b3Vec3 p6 = b3MulAdd( pd, r2, a1 );
				b3Vec3 p7 = b3MulAdd( pd, r1, a2 );
				b3Vec3 p8 = b3MulAdd( pd, r2, a2 );

				b3Vec3 points[8] = { p1, p2, p3, p4, p5, p6, p7, p8 };
				b3HullData* hull = b3CreateHull( points, 8, 8 );
				b3CreateHullShape( bodyId, &shapeDef, hull );
				b3DestroyHull( hull );
			}

			if ( i % 9 == 0 )
			{
				b3Vec3 p1 = b3MulAdd( nd, r0, u1 );
				b3Vec3 p2 = b3MulAdd( nd, r1, u1 );
				b3Vec3 p3 = b3MulAdd( nd, r0, u2 );
				b3Vec3 p4 = b3MulAdd( nd, r1, u2 );
				b3Vec3 p5 = b3MulAdd( pd, r0, u1 );
				b3Vec3 p6 = b3MulAdd( pd, r1, u1 );
				b3Vec3 p7 = b3MulAdd( pd, r0, u2 );
				b3Vec3 p8 = b3MulAdd( pd, r1, u2 );

				b3Vec3 points[8] = { p1, p2, p3, p4, p5, p6, p7, p8 };
				b3HullData* hull = b3CreateHull( points, 8, 8 );
				b3CreateHullShape( bodyId, &shapeDef, hull );
				b3DestroyHull( hull );
			}

			u1 = u2;
		}

		if ( kinematic == false )
		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p.y = B3_FIX( 10.0f );
			jointDef.motorSpeed = b3FixMul( ( b3FixDiv( B3_PI , B3_FIX( 180.0f ) ) ) , motorSpeed );
			jointDef.maxMotorTorque = B3_FIX( 1e8f );
			jointDef.enableMotor = true;

			b3CreateRevoluteJoint( worldId, &jointDef );
		}
	}

	int gridCount = BENCHMARK_DEBUG ? 8 : 20;
	b3Fixed a = B3_FIX( 0.2f );

	b3BoxHull cube = b3MakeBoxHull( a, a, a );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	b3Fixed x = b3FixMul( b3FixMul( -B3_FIX( 2.0f ) , a ) , b3FixFromInt( gridCount ) );
	for ( int i = 0; i < gridCount; ++i )
	{
		b3Fixed y = b3FixMul( b3FixMul( -B3_FIX( 2.0f ) , a ) , b3FixFromInt( gridCount ) ) + B3_FIX( 21.0f );
		for ( int j = 0; j < gridCount; ++j )
		{
			b3Fixed z = b3FixMul( b3FixMul( -B3_FIX( 2.0f ) , a ) , b3FixFromInt( gridCount ) );
			for ( int k = 0; k < gridCount; ++k )
			{
				bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ x, y, z } );
				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

				b3CreateHullShape( bodyId, &shapeDef, &cube.base );
				z += b3FixMul( B3_FIX( 4.0f ) , a );
			}

			y += b3FixMul( B3_FIX( 4.0f ) , a );
		}

		x += b3FixMul( B3_FIX( 4.0f ) , a );
	}
}

struct
{
	b3MeshData* meshData;
} g_treeData;

static void CreateTrees( b3WorldId worldId, int scale )
{
	memset( &g_treeData, 0, sizeof( g_treeData ) );

	// b3Fixed tilt = 0.15f * B3_PI;
	b3Fixed tilt = b3FixMul( B3_FIX( 0.0f ) , B3_PI );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	bodyDef.rotation = b3MakeQuatFromAxisAngle( (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0 ) }, tilt );
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	int xCount = scale * 150;
	// int zCount = BENCHMARK_DEBUG ? 100 : 400;
	int zCount = scale * 200;

	b3Fixed cellWidth = b3FixDiv( B3_FIX( 1.0f ) , b3FixFromInt( scale ) );
	b3Fixed amplitude = B3_FIX( 0.4f );
	b3Fixed rowHz = B3_FIX( 0.05f );
	b3Fixed columnHz = B3_FIX( 0.1f );

	g_treeData.meshData = b3CreateWaveMesh( xCount, zCount, cellWidth, amplitude, rowHz, columnHz );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundId, &shapeDef, g_treeData.meshData, b3Vec3_one );

	bodyDef.type = b3_dynamicBody;
	bodyDef.sleepThreshold = B3_FIX( 0.2f );
	bodyDef.rotation = b3Quat_identity;

	int bodyCount = BENCHMARK_DEBUG ? 10 : 50;

	shapeDef.baseMaterial.friction = B3_FIX( 0.9f );
	shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.05f );
	shapeDef.updateBodyMass = false;
	shapeDef.density = B3_FIX( 1.0f );

	int hullCount = 22;
	b3HullData* hulls[22] = { 0 };

	b3Fixed y = B3_FIX( 1.0f );
	b3Fixed r = B3_FIX( 0.75f );
	b3Fixed l = B3_FIX( 1.5f );
	for ( int i = 0; i < hullCount; ++i )
	{
		hulls[i] = b3CreateCylinder( l + b3FixMul( B3_FIX( 2.0f ) , r ), r, y - r, 6 );
		y += l + b3FixMul( B3_FIX( 2.0f ) , r );
		r = b3FixMul( B3_FIX( 0.95f ) , r );
	}

	b3Fixed angularVelocity = -B3_FIX( 0.5f );
	b3Fixed z = BENCHMARK_DEBUG ? -B3_FIX( 15.0f ) : -B3_FIX( 70.0f );
	b3CosSin cs = b3ComputeCosSin( tilt );
	b3Fixed yTilt = b3FixDiv( cs.sine , cs.cosine );
	for ( int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex )
	{
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0 ), B3_FIX( 1.0f ) - b3FixMul( z , yTilt ), z } );
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		for ( int shapeIndex = 0; shapeIndex < 22; ++shapeIndex )
		{
			// Random rotation provided no benefit to behavior.
			// b3Fixed angle = B3_PI * RandomFloat();
			// b3Transform xf;
			// xf.p = b3Vec3_zero;
			// xf.q = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, angle );
			// b3Vec3 scale = {1.0f, 1.0f, 1.0f};
			// b3CreateTransformedHullShape( bodyId, &shapeDef, hulls[shapeIndex], xf, scale );
			b3CreateHullShape( bodyId, &shapeDef, hulls[shapeIndex] );
		}

		b3Fixed velocityScale = B3_FIX( 0.5f ) + b3FixDiv( ( b3FixMul( B3_FIX( 0.5f ) , b3FixFromInt( bodyIndex ) ) ) , b3FixFromInt( bodyCount ) );
		b3Body_ApplyMassFromShapes( bodyId );
		b3Pos center = b3Body_GetWorldCenter( bodyId );
		b3Vec3 omega = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), b3FixMul( velocityScale , angularVelocity ) };
		b3Vec3 v = b3Cross( omega, b3SubPos( center, bodyDef.position ) );
		b3Body_SetAngularVelocity( bodyId, omega );
		b3Body_SetLinearVelocity( bodyId, v );

		z += B3_FIX( 3.0f );
		angularVelocity = -angularVelocity;
	}

	for ( int i = 0; i < hullCount; ++i )
	{
		b3DestroyHull( hulls[i] );
	}
}

void CreateTrees25( b3WorldId worldId )
{
	CreateTrees( worldId, 4 );
}

void CreateTrees50( b3WorldId worldId )
{
	CreateTrees( worldId, 2 );
}

void CreateTrees100( b3WorldId worldId )
{
	CreateTrees( worldId, 1 );
}

void DestroyTrees( void )
{
	b3DestroyMesh( g_treeData.meshData );
	memset( &g_treeData, 0, sizeof( g_treeData ) );
}

struct JunkyardData
{
	b3BodyId pusherId;
	b3Fixed degrees;
	b3Fixed radius;
} g_junkyardData;

void CreateJunkyard( b3WorldId worldId )
{
	b3BodyId groundId;
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		groundId = b3CreateBody( worldId, &bodyDef );
	}

	{
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		{
			b3BoxHull box = b3MakeBoxHull( B3_FIX( 120.0f ), B3_FIX( 1.0f ), B3_FIX( 120.0f ) );
			g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { -B3_FIX( 50.0f ), B3_FIX( 8.0f ), B3_FIX( 0.0f ) };
			b3BoxHull box = b3MakeOffsetBoxHull( B3_FIX( 1.0f ), B3_FIX( 8.0f ), B3_FIX( 50.0f ), offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { B3_FIX( 50.0f ), B3_FIX( 8.0f ), B3_FIX( 0.0f ) };
			b3BoxHull box = b3MakeOffsetBoxHull( B3_FIX( 1.0f ), B3_FIX( 8.0f ), B3_FIX( 50.0f ), offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { B3_FIX( 0.0f ), B3_FIX( 8.0f ), -B3_FIX( 50.0f ) };
			b3BoxHull box = b3MakeOffsetBoxHull( B3_FIX( 50.0f ), B3_FIX( 8.0f ), B3_FIX( 1.0f ), offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
		{
			b3Vec3 offset = { B3_FIX( 0.0f ), B3_FIX( 8.0f ), B3_FIX( 50.0f ) };
			b3BoxHull box = b3MakeOffsetBoxHull( B3_FIX( 50.0f ), B3_FIX( 8.0f ), B3_FIX( 1.0f ), offset );
			b3CreateHullShape( groundId, &shapeDef, &box.base );
		}
	}
	{
		b3HullData* rockHull = b3CreateRock( B3_FIX( 1.5f ) );

		int count = BENCHMARK_DEBUG ? 2 : 24;
		b3Fixed height = B3_FIX( 24.0f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int Y = 0; Y < count; ++Y )
		{
			for ( int X = 0; X <= 20; ++X )
			{
				for ( int Z = 0; Z <= 20; ++Z )
				{
					b3Vec3 local;
					local.x = -B3_FIX( 40.0f ) + b3FixMul( B3_FIX( 4.0f ) , b3FixFromInt( X ) );
					local.y = b3FixMul( B3_FIX( 4.0f ) , b3FixFromInt( Y ) ) + height + B3_FIX( 1.0f );
					local.z = -B3_FIX( 40.0f ) + b3FixMul( B3_FIX( 4.0f ) , b3FixFromInt( Z ) );
					bodyDef.position = b3OffsetPos( GetSceneOrigin(), local );
					b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
					b3CreateHullShape( bodyId, &shapeDef, rockHull );
				}
			}
		}

		b3DestroyHull( rockHull );
	}

	g_junkyardData.radius = B3_FIX( 35.0f );
	b3Fixed mHeight = B3_FIX( 24.0f );

	b3HullData* hull = b3CreateCylinder( mHeight, B3_FIX( 4.0f ), B3_FIX( 0.0f ), 16 );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ g_junkyardData.radius, B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	g_junkyardData.pusherId = b3CreateBody( worldId, &bodyDef );
	g_junkyardData.degrees = B3_FIX( 0.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( g_junkyardData.pusherId, &shapeDef, hull );
	b3DestroyHull( hull );
}

void StepJunkyard( b3WorldId worldId, int stepCount )
{
	(void)worldId;
	(void)stepCount;

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
	const b3Fixed omega = -B3_FIX( 6.0f );
	g_junkyardData.degrees += b3FixMul( omega , timeStep );
	b3CosSin cs = b3ComputeCosSin( b3FixDiv( b3FixMul( g_junkyardData.degrees , B3_PI ) , B3_FIX( 180.0f ) ) );
	b3Fixed r = g_junkyardData.radius;
	b3Pos targetPos = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ b3FixMul( r , cs.cosine ), B3_FIX( 0.0f ), b3FixMul( r , cs.sine ) } );
	b3WorldTransform target = { .p = targetPos, .q = b3Quat_identity };
	b3Body_SetTargetTransform( g_junkyardData.pusherId, target, timeStep, false );
}

// Huge pile of large convexes, ported from PEEL. Each convex is the hull of 32 random points on a
// sphere. A fixed LCG seed makes the hull identical across runs so results compare directly.

// PEEL's BasicRandom, kept verbatim so the point set matches the original
typedef struct ConvexPileRandom
{
	unsigned int state;
} ConvexPileRandom;

static unsigned int NextConvexPileRandom( ConvexPileRandom* rng )
{
	rng->state = rng->state * 2147001325u + 715136305u;
	return rng->state;
}

// Float in [-0.5, 0.5]
static b3Fixed ConvexPileRandomFloat( ConvexPileRandom* rng )
{
	return b3FixDiv( (b3Fixed)b3FixFromInt( ( NextConvexPileRandom( rng ) & 0xffff ) ) , B3_FIX( 65535.0f ) ) - B3_FIX( 0.5f );
}

// Uniform random direction, rejection sampled inside the unit sphere then pushed to the surface
static b3Vec3 UnitRandomPoint( ConvexPileRandom* rng )
{
	b3Vec3 point;
	b3Fixed lengthSq;
	do
	{
		point.x = ConvexPileRandomFloat( rng );
		point.y = ConvexPileRandomFloat( rng );
		point.z = ConvexPileRandomFloat( rng );
		lengthSq = b3Dot( point, point );
	} while ( lengthSq > B3_FIX( 0.25f ) );

	return b3Normalize( point );
}

void CreateConvexPile( b3WorldId worldId )
{
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( B3_FIX( 250.0f ), B3_FIX( 1.0f ), B3_FIX( 250.0f ) );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		g_groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	int countX = 8;
	int countZ = 8;
	int layers = BENCHMARK_DEBUG ? 10 : 80;
	b3Fixed amplitude = B3_FIX( 2.0f );
	int pointCount = 32;
	b3Fixed scatter = b3FixMul( B3_FIX( 2.0f ) , amplitude );

	// Hull around random points on a sphere of radius amplitude
	assert( pointCount <= 64 );
	b3Vec3 points[64];
	ConvexPileRandom rng = { 42 };
	for ( int i = 0; i < pointCount; ++i )
	{
		points[i] = b3MulSV( amplitude, UnitRandomPoint( &rng ) );
	}

	b3HullData* convex = b3CreateHull( points, pointCount, pointCount );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// Grid tall enough to collapse into a pile
	for ( int layer = 0; layer < layers; ++layer )
	{
		for ( int z = 0; z < countZ; ++z )
		{
			for ( int x = 0; x < countX; ++x )
			{
				b3Fixed posX = b3FixMul( ( (b3Fixed)b3FixFromInt( x ) - b3FixMul( B3_FIX( 0.5f ) , (b3Fixed)b3FixFromInt( countX ) ) ) , scatter );
				b3Fixed posZ = b3FixMul( ( (b3Fixed)b3FixFromInt( z ) - b3FixMul( B3_FIX( 0.5f ) , (b3Fixed)b3FixFromInt( countZ ) ) ) , scatter );
				b3Fixed posY = amplitude + b3FixMul( b3FixMul( B3_FIX( 2.0f ) , amplitude ) , (b3Fixed)b3FixFromInt( layer ) );

				bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ posX, posY, posZ } );
				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
				b3CreateHullShape( bodyId, &shapeDef, convex );
			}
		}
	}

	b3DestroyHull( convex );
}
