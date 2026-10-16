// SPDX-FileCopyrightText: 2026 Fixed3D perf harness
// SPDX-License-Identifier: MIT
//
// PerfStateHashGoldenTest: a tiny hand-built world (10 dynamic boxes on a ground
// box, 30 steps at a fixed dt, one worker) is hashed with FNV-1a/64 over every
// body's position, rotation, linear and angular velocity in body-id order, and the
// result is compared to a constant. The same world is also stepped with four
// workers and its hash must agree bit-for-bit (the determinism claim). No files,
// no wall clock: the step count and dt are arguments.

#include "box3d/box3d.h"
#include "test_macros.h"

#include <inttypes.h>
#include <stdio.h>

#define PERF_NUM_BODIES 10
#define PERF_NUM_STEPS 30
#define PERF_WORKERS_ORDERED 1
#define PERF_WORKERS_AGREE 4

// FNV-1a 64-bit constants (as in tools/benchmark/contact_state_hash.c).
#define PERF_FNV_INIT UINT64_C( 14695981039346656037 )
#define PERF_FNV_PRIME UINT64_C( 1099511628211 )

// Captured on the scalar RelWithDebInfo+VALIDATE build, 2026-10-15. Workers 1 and 4
// printed the same value before this constant was pinned (red line: hashOne == 0).
// Re-harvested for the efca0c1 isotropic inertia tolerance, 0xc4f45e4c6b5534e6 ->
// 0xe6a944fb4ffb2707: the ten dynamic boxes here are cubes, so every one of them reads as
// isotropic and the gate turns the gyroscopic solve off for all of them -- counter on the
// new gating branch read tested=20 fired=20. Workers 1 and 4 print the same value, and it
// is the same in the narrow, ludicrous and Debug+ASan builds (this scene has no mesh and
// no large coordinate, so the hash is numeric-mode independent). The counter was removed
// before the commit.
#define PERF_STATE_HASH_GOLDEN UINT64_C( 0xe6a944fb4ffb2707 )

static uint64_t PerfHashValue( uint64_t hash, int64_t value )
{
	uint64_t bits = (uint64_t)value;
	for ( int i = 0; i < 8; ++i )
	{
		hash = ( hash ^ ( bits & 255 ) ) * PERF_FNV_PRIME;
		bits >>= 8;
	}
	return hash;
}

static uint64_t PerfHashVector( uint64_t hash, b3Vec3 v )
{
	return PerfHashValue( PerfHashValue( PerfHashValue( hash, v.x ), v.y ), v.z );
}

typedef struct PerfWorld
{
	b3BodyId bodies[PERF_NUM_BODIES];
} PerfWorld;

static uint64_t PerfHashWorld( b3WorldId worldId, const PerfWorld* world )
{
	uint64_t hash = PERF_FNV_INIT;
	for ( int i = 0; i < PERF_NUM_BODIES; ++i )
	{
		b3WorldTransform t = b3Body_GetTransform( world->bodies[i] );
		hash = PerfHashValue( PerfHashValue( PerfHashValue( hash, t.p.x ), t.p.y ), t.p.z );
		hash = PerfHashVector( hash, t.q.v );
		hash = PerfHashValue( hash, t.q.s );
		hash = PerfHashVector( hash, b3Body_GetLinearVelocity( world->bodies[i] ) );
		hash = PerfHashVector( hash, b3Body_GetAngularVelocity( world->bodies[i] ) );
	}
	return hash;
}

static uint64_t PerfStepAndHash( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Ground box across the whole scene.
	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId ground = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( B3_FIX( 10.0f ), B3_FIX( 0.5f ), B3_FIX( 10.0f ) );
	b3CreateHullShape( ground, &groundShapeDef, &groundBox.base );

	PerfWorld world = { { 0 } };
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	for ( int i = 0; i < PERF_NUM_BODIES; ++i )
	{
		bodyDef.position = (b3Pos){ b3FixFromInt( ( i % 5 ) - 2 ), B3_FIX( 1.0f ) + b3FixFromInt( i / 5 ), b3FixFromInt( ( i % 4 ) - 1 ) };
		bodyDef.rotation = b3Quat_identity;
		world.bodies[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( world.bodies[i], &shapeDef, &box.base );
	}

	b3Fixed timeStep = b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) );
	for ( int step = 0; step < PERF_NUM_STEPS; ++step )
	{
		b3World_Step( worldId, timeStep, 1 );
	}

	uint64_t hash = PerfHashWorld( worldId, &world );

	b3DestroyWorld( worldId );
	return hash;
}

static int PerfStateHashGolden( void )
{
	uint64_t hashOne = PerfStepAndHash( PERF_WORKERS_ORDERED );
	uint64_t hashFour = PerfStepAndHash( PERF_WORKERS_AGREE );

	printf( "  perf state hash (1 worker) = %016" PRIx64 "\n", hashOne );
	printf( "  perf state hash (4 worker) = %016" PRIx64 "\n", hashFour );

	// Determinism: workers 1 and 4 must agree bit-for-bit.
	ENSURE( hashOne == hashFour );

	// Golden: one worker matches the frozen constant.
	ENSURE( hashOne == PERF_STATE_HASH_GOLDEN );

	return 0;
}

int PerfStateHashGoldenTest( void )
{
	RUN_SUBTEST( PerfStateHashGolden );
	return 0;
}
