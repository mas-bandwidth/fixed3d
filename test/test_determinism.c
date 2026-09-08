// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "determinism.h"
#include "stability.h"
#include "test_macros.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>

#ifdef BOX3D_PROFILE
	#include <tracy/TracyC.h>
#else
	#define TracyCFrameMark
#endif

// Golden values for the fixed-point build. Fixed-point math is exactly
// reproducible across platforms and worker counts, so these hold everywhere.
// The scene builds at GetSceneOrigin() — 100 km out on every axis — so this
// test also enforces determinism far from the origin.
//
// The sleep step is shared by both builds: an exactly representable origin shift is a
// bit-exact rigid translation of the whole trajectory, so the scene settles on the same
// step it would at (0,0,0). Only the hash is per-build, because it covers the absolute
// transform bytes and the wide build stores 128-bit positions (80-byte b3WorldTransform
// vs 56-byte). Full 128 bits are hashed.
//
// A SLEEP STEP OF 0 MEANS THE RAGDOLLS NEVER SETTLED, which is what a missed scale
// crossing looks like from here. Read a zero as that before re-capturing anything.
// These references combine the inverse-scaled joint/GJK repairs with fixed
// 2f1ed91's corrected normalization, small angles and conservative bounds.
// Independent captures agree across workers 1-5, Debug/optimized and narrow/wide
// builds. Query-spawn and mesh-drop retain the preceding repaired references.
#define RAGDOLL_SLEEP_STEP 404
#if defined( BOX3D_LUDICROUS_MODE )
#define RAGDOLL_HASH 0x1DA55033
#else
#define RAGDOLL_HASH 0xF97922B3
#endif

// Goldens for the wave pile, query spawn and mesh drop scenarios. Fixed-point goldens
// hold across platforms and worker counts by construction. The sleep steps are shared
// between builds; the hashes cover absolute transform bytes, so the ludicrous build
// (128-bit positions, 80-byte b3WorldTransform) carries its own values.
#define WAVE_PILE_SLEEP_STEP 275
#define QUERY_SPAWN_SLEEP_STEP 243
#define QUERY_SPAWN_HIT_COUNT 59
#define QUERY_SPAWN_QUERY_HASH 0xE583B246
#define MESH_DROP_SLEEP_STEP 210
#if defined( BOX3D_LUDICROUS_MODE )
#define WAVE_PILE_HASH 0x253BEF8E
#define QUERY_SPAWN_HASH 0x7C6B3268
#define MESH_DROP_HASH 0xDB1DA8B9
#else
#define WAVE_PILE_HASH 0x22BF35CE
#define QUERY_SPAWN_HASH 0x49ECDEA8
#define MESH_DROP_HASH 0x491E324B
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

// Hash numeric fields in a fixed byte order, excluding padding. Include all
// position bits in the wide build, as well as velocities and joint impulses.
static uint64_t SphericalHashValue( uint64_t hash, uint64_t value )
{
	for ( int i = 0; i < 8; ++i )
	{
		hash = ( hash ^ ( value & 255 ) ) * UINT64_C( 1099511628211 );
		value >>= 8;
	}
	return hash;
}

static uint64_t SphericalHashVector( uint64_t hash, b3Vec3 v )
{
	return SphericalHashValue( SphericalHashValue( SphericalHashValue( hash, (uint64_t)v.x ), (uint64_t)v.y ), (uint64_t)v.z );
}

enum
{
	sphericalFrameCount = 120
};

static int RunSphericalGeometry( int workerCount, int scale, uint64_t* hashes )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	worldDef.gravity = b3Vec3_zero;
	b3WorldId world = b3CreateWorld( &worldDef );
	b3Fixed size = b3FixFromInt( scale == 0 ? 1 : 250 );
	b3BodyId bodies[99];
	b3JointId joints[96];
	b3ShapeDef shape = b3DefaultShapeDef();
	shape.filter.maskBits = 0;
	// At size 250 this still gives a mass of 15,625,000. The default density
	// makes the locked chain exceed the uncached solver's impulse range.
	shape.density = B3_FIXED_ONE;
	b3BoxHull cube = b3MakeCubeHull( size / 2 );

	// Two chains provide 32 joints per occupied color, spread over multiple
	// solver blocks. The dynamic star also exercises serial overflow joints.
	for ( int group = 0; group < 3; ++group )
	{
		for ( int i = 0; i < 33; ++i )
		{
			b3BodyDef body = b3DefaultBodyDef();
			body.type = i == 0 && group != 2 ? b3_staticBody : b3_dynamicBody;
			b3Vec3 offset = { size * i, 0, 0 };
			if ( group == 2 && i > 0 )
			{
				offset = (b3Vec3){ size * ( i % 3 - 1 ), size * ( i / 3 % 3 - 1 ), size * ( i / 9 % 3 - 1 ) };
			}
			body.position = b3ToPos( b3Add( offset, (b3Vec3){ 0, size * group * 4, 0 } ) );
			body.angularVelocity = (b3Vec3){ B3_FIX( 0.01f ), i % 2 ? B3_FIX( 0.02f ) : 0, B3_FIX( -0.01f ) };
			body.linearVelocity.y = size / 100;
			body.rotation.s = i % 3 == 0 ? -B3_FIXED_ONE : B3_FIXED_ONE;
			int index = group * 33 + i;
			bodies[index] = b3CreateBody( world, &body );
			b3CreateHullShape( bodies[index], &shape, &cube.base );
			if ( i == 0 )
			{
				continue;
			}
			b3SphericalJointDef joint = b3DefaultSphericalJointDef();
			joint.base.bodyIdA = bodies[group == 2 ? group * 33 : index - 1];
			joint.base.bodyIdB = bodies[index];
			joint.base.localFrameA.p = group == 2 ? offset : (b3Vec3){ size / 2, 0, 0 };
			joint.base.localFrameB.p.x = group == 2 ? 0 : -size / 2;
			joint.enableSpring = group == 1;
			joint.enableMotor = group == 1;
			joint.hertz = B3_FIX( 2.0f );
			joint.maxMotorTorque = B3_FIX( 100.0f );
			joint.motorVelocity.y = B3_FIX( 0.01f );
			joint.coneAngle = B3_FIX( 0.3f );
			joint.lowerTwistAngle = B3_FIX( -0.2f );
			joint.upperTwistAngle = B3_FIX( 0.2f );
			joints[group * 32 + i - 1] = b3CreateSphericalJoint( world, &joint );
		}
	}

	const int substeps[] = { 1, 2, 4, 8 };
	for ( int frame = 0; frame < sphericalFrameCount; ++frame )
	{
		if ( frame == 15 || frame == 25 )
		{
			b3World_EnableWarmStarting( world, frame == 25 );
		}
		if ( frame == 30 || frame == 50 || frame == 70 )
		{
			for ( int i = 0; i < 96; ++i )
			{
				b3SphericalJoint_EnableSpring( joints[i], frame == 30 );
				b3SphericalJoint_EnableMotor( joints[i], frame == 70 );
				b3SphericalJoint_EnableConeLimit( joints[i], frame == 50 );
				b3SphericalJoint_EnableTwistLimit( joints[i], frame == 50 );
			}
		}
		if ( frame == 80 )
		{
			b3Body_Disable( bodies[16] );
		}
		if ( frame == 82 )
		{
			b3Body_Enable( bodies[16] );
		}
		if ( frame == 84 )
		{
			b3Body_SetAwake( bodies[8], false );
		}
		if ( frame == 86 )
		{
			b3Body_SetAwake( bodies[8], true );
		}
		if ( frame == 90 || frame == 100 )
		{
			b3MotionLocks locks = { 0 };
			locks.angularX = locks.angularY = locks.angularZ = frame == 90;
			for ( int i = 1; i < 33; ++i )
			{
				b3Body_SetMotionLocks( bodies[i], locks );
			}
		}
		if ( frame == 110 )
		{
			b3WorldTransform t = b3Body_GetTransform( bodies[12] );
			t.p.y += size / 10;
			b3Body_SetTransform( bodies[12], t.p, b3Quat_identity );
		}
		b3Fixed dt = frame == 10 ? 0 : b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) );
		b3World_Step( world, dt, substeps[frame % 4] );
		if ( frame == 0 )
		{
			b3Counters counters = b3World_GetCounters( world );
			if ( counters.colorCounts[B3_GRAPH_COLOR_COUNT - 1] == 0 || counters.colorCounts[0] < 32 )
			{
				b3DestroyWorld( world );
				ENSURE( false );
			}
		}
		uint64_t hash = UINT64_C( 14695981039346656037 );
		for ( int i = 0; i < 99; ++i )
		{
			b3WorldTransform t = b3Body_GetTransform( bodies[i] );
			hash = SphericalHashValue( SphericalHashValue( SphericalHashValue( hash, (uint64_t)t.p.x ), (uint64_t)t.p.y ),
									   (uint64_t)t.p.z );
#if defined( BOX3D_LUDICROUS_MODE )
			hash = SphericalHashValue( hash, (uint64_t)( (b3UInt128)t.p.x >> 64 ) );
			hash = SphericalHashValue( hash, (uint64_t)( (b3UInt128)t.p.y >> 64 ) );
			hash = SphericalHashValue( hash, (uint64_t)( (b3UInt128)t.p.z >> 64 ) );
#endif
			hash = SphericalHashVector( hash, t.q.v );
			hash = SphericalHashValue( hash, (uint64_t)t.q.s );
			hash = SphericalHashVector( hash, b3Body_GetLinearVelocity( bodies[i] ) );
			hash = SphericalHashVector( hash, b3Body_GetAngularVelocity( bodies[i] ) );
			hash = SphericalHashValue( hash, b3Body_IsAwake( bodies[i] ) );
		}
		for ( int i = 0; i < 96; ++i )
		{
			hash = SphericalHashVector( hash, b3Joint_GetConstraintForce( joints[i] ) );
			hash = SphericalHashVector( hash, b3Joint_GetConstraintTorque( joints[i] ) );
		}
		hashes[frame] = hash;
	}
	b3DestroyWorld( world );
	return 0;
}

static int SphericalGeometryTest( void )
{
	// Captured from the uncached solver. A stale but consistently wrong cache
	// must fail too, even if its results happen to agree across worker counts.
#if defined( BOX3D_LUDICROUS_MODE )
	const uint64_t goldens[2] = { UINT64_C( 0xee343d0d530f22f1 ), UINT64_C( 0x46256d278ff09af2 ) };
#else
	const uint64_t goldens[2] = { UINT64_C( 0xccf165f88a73ebb7 ), UINT64_C( 0x2add50ab7852c3ce ) };
#endif
	for ( int scale = 0; scale < 2; ++scale )
	{
		uint64_t reference[sphericalFrameCount];
		ENSURE( RunSphericalGeometry( 1, scale, reference ) == 0 );
		uint64_t hash = UINT64_C( 14695981039346656037 );
		for ( int frame = 0; frame < sphericalFrameCount; ++frame )
		{
			hash = SphericalHashValue( hash, reference[frame] );
		}
		if ( hash != goldens[scale] )
		{
			printf( "  spherical scale=%d hash=%016" PRIx64 "\n", scale, hash );
		}
		ENSURE( hash == goldens[scale] );
		for ( int workers = 2; workers <= 8; ++workers )
		{
			uint64_t actual[sphericalFrameCount];
			ENSURE( RunSphericalGeometry( workers, scale, actual ) == 0 );
			for ( int frame = 0; frame < sphericalFrameCount; ++frame )
			{
				if ( actual[frame] != reference[frame] )
				{
					printf( "  spherical scale=%d workers=%d frame=%d\n", scale, workers, frame );
				}
				ENSURE( actual[frame] == reference[frame] );
			}
		}
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
	RUN_SUBTEST( SphericalGeometryTest );

	return 0;
}
