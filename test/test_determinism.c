// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "determinism.h"
#include "stability.h"
#include "test_macros.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef BOX3D_PROFILE
	#include <tracy/TracyC.h>
#else
	#define TracyCFrameMark
#endif

// Golden values for the fixed-point build. Fixed-point math is exactly
// reproducible across platforms and worker counts, so these hold everywhere.
// The scene builds at GetSceneOrigin() — 100 km out on every axis — so this
// test also enforces determinism far from the origin. The sleep step is
// identical to the value the scene had at (0,0,0); only the hash moved,
// because it covers the absolute transform bytes: an exactly representable
// origin shift is a bit-exact rigid translation of the whole trajectory.
#define RAGDOLL_SLEEP_STEP 287
// The sleep step is identical in both builds — an exactly representable origin is a
// bit-exact rigid translation of the trajectory. The hash differs only because it covers
// the absolute transform bytes, and the wide build stores 128-bit positions (80-byte
// b3WorldTransform vs 56-byte), so it carries its own golden. Full 128 bits are hashed.
#if defined( BOX3D_LUDICROUS_MODE )
#define RAGDOLL_HASH 0x886BE415
#else
#define RAGDOLL_HASH 0xB222C195
#endif

// Goldens for the c37cfe4-ported scenarios (wave pile, query spawn, mesh drop).
// Fixed-point goldens hold across platforms and worker counts by construction.
// The sleep steps are shared between builds; the hashes cover absolute transform
// bytes, so the ludicrous build (128-bit positions, 80-byte b3WorldTransform)
// carries its own values. The wave pile and mesh drop hashes moved with the
// dfa5e6a triangle-vs-hull admission change (hulls resting on mesh terrain take
// slightly different manifolds); their sleep steps and everything about query
// spawn carried over unchanged.
#define WAVE_PILE_SLEEP_STEP 279
#define QUERY_SPAWN_SLEEP_STEP 243
#define QUERY_SPAWN_HIT_COUNT 59
#define QUERY_SPAWN_QUERY_HASH 0xE583B246
#define MESH_DROP_SLEEP_STEP 250
#if defined( BOX3D_LUDICROUS_MODE )
#define WAVE_PILE_HASH 0xCD74B232
#define QUERY_SPAWN_HASH 0x72EDD20C
#define MESH_DROP_HASH 0xEE4D0F7A
#else
#define WAVE_PILE_HASH 0x6FA2F3B2
#define QUERY_SPAWN_HASH 0x28042A4C
#define MESH_DROP_HASH 0x777F3CB6
#endif

static int SingleMultithreadingTest( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	FallingRagdollData data = CreateFallingRagdolls( worldId );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );

	int stepLimit = 500;
	for ( int i = 0; i < stepLimit; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		bool done = UpdateFallingRagdolls( worldId, &data );
		if ( done )
		{
			break;
		}
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != RAGDOLL_SLEEP_STEP || data.hash != RAGDOLL_HASH )
	{
		printf( "  workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == RAGDOLL_SLEEP_STEP );
	ENSURE( data.hash == RAGDOLL_HASH );

	DestroyFallingRagdolls( &data );

	return 0;
}

// Test multithreaded determinism.
static int MultithreadingTest( void )
{
	for ( int workerCount = 1; workerCount < 6; ++workerCount )
	{
		int result = SingleMultithreadingTest( workerCount );
		ENSURE( result == 0 );
	}

	return 0;
}

// Test cross platform determinism.
static int CrossPlatformTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	FallingRagdollData data = CreateFallingRagdolls( worldId );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );

	bool done = false;
	while ( done == false )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateFallingRagdolls( worldId, &data );
	}

	if ( data.sleepStep != RAGDOLL_SLEEP_STEP || data.hash != RAGDOLL_HASH )
	{
		printf( "  cross-platform sleepStep=%d hash=0x%08X\n", data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == RAGDOLL_SLEEP_STEP );
	ENSURE( data.hash == RAGDOLL_HASH );

	DestroyFallingRagdolls( &data );

	b3DestroyWorld( worldId );

	return 0;
}

static int SingleWavePileTest( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	WavePileData data = CreateWavePile( worldId );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );

	// Rolling resistance must put the pile to sleep within 500 steps
	bool done = false;
	for ( int i = 0; i < 500 && done == false; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateWavePile( worldId, &data );
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != WAVE_PILE_SLEEP_STEP || data.hash != WAVE_PILE_HASH )
	{
		printf( "  wave pile workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE( done == true );
	ENSURE( data.sleepStep == WAVE_PILE_SLEEP_STEP );
	ENSURE( data.hash == WAVE_PILE_HASH );

	DestroyWavePile( &data );

	return 0;
}

// Test multithreaded determinism of a mixed convex pile on a wave height field.
static int WavePileTest( void )
{
	for ( int workerCount = 1; workerCount <= 4; ++workerCount )
	{
		int result = SingleWavePileTest( workerCount );
		ENSURE( result == 0 );
	}

	return 0;
}

static int SingleQuerySpawnTest( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	QuerySpawnData data = CreateQuerySpawn( worldId );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );

	bool done = false;
	for ( int i = 0; i < 1000 && done == false; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateQuerySpawn( worldId, &data );
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != QUERY_SPAWN_SLEEP_STEP || data.hash != QUERY_SPAWN_HASH || data.queryHitCount != QUERY_SPAWN_HIT_COUNT ||
		 data.queryHash != QUERY_SPAWN_QUERY_HASH )
	{
		printf( "  query spawn workers=%d sleepStep=%d hash=0x%08X hits=%d queryHash=0x%08X\n", workerCount, data.sleepStep,
				data.hash, data.queryHitCount, data.queryHash );
	}

	ENSURE( done == true );
	ENSURE( data.spawnCount == QUERY_SPAWN_COUNT );
	ENSURE( data.sleepStep == QUERY_SPAWN_SLEEP_STEP );
	ENSURE( data.hash == QUERY_SPAWN_HASH );
	ENSURE( data.queryHitCount == QUERY_SPAWN_HIT_COUNT );
	ENSURE( data.queryHash == QUERY_SPAWN_QUERY_HASH );

	DestroyQuerySpawn( &data );

	return 0;
}

// Test determinism of world queries by feeding their results back into the simulation.
static int QuerySpawnTest( void )
{
	for ( int workerCount = 1; workerCount <= 4; ++workerCount )
	{
		int result = SingleQuerySpawnTest( workerCount );
		ENSURE( result == 0 );
	}

	return 0;
}

static int SingleMeshDropTest( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;

	b3WorldId worldId = b3CreateWorld( &worldDef );

	MeshDropData data = CreateMeshDrop( worldId, b3Pos_zero );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );

	bool done = false;
	for ( int i = 0; i < 400 && done == false; ++i )
	{
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );
		TracyCFrameMark;

		done = UpdateMeshDrop( worldId, &data );
	}

	b3DestroyWorld( worldId );

	if ( data.sleepStep != MESH_DROP_SLEEP_STEP || data.hash != MESH_DROP_HASH )
	{
		printf( "  mesh drop workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash );
	}

	ENSURE( done == true );
	ENSURE( data.sleepStep == MESH_DROP_SLEEP_STEP );
	ENSURE( data.hash == MESH_DROP_HASH );

	DestroyMeshDrop( &data );

	return 0;
}

// Test continuous collision determinism. Thin fast boxes need CCD against the wave mesh.
// The scene is large, so only the single threaded and widest schedules run.
static int MeshDropTest( void )
{
	int workerCounts[2] = { 1, 4 };
	for ( int i = 0; i < 2; ++i )
	{
		int result = SingleMeshDropTest( workerCounts[i] );
		ENSURE( result == 0 );
	}

	return 0;
}

int DeterminismTest( void )
{
	RUN_SUBTEST( MultithreadingTest );
	RUN_SUBTEST( CrossPlatformTest );
	RUN_SUBTEST( WavePileTest );
	RUN_SUBTEST( QuerySpawnTest );
	RUN_SUBTEST( MeshDropTest );

	return 0;
}
