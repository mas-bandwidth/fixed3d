// SPDX-FileCopyrightText: 2026 mas-bandwidth
// SPDX-License-Identifier: MIT

// Pins the perf-harness body-state hash (the FNV-1a/64 in shared/benchmarks.c) to a
// constant so any change that moves a single body bit fails here. It also asserts the
// hash is identical across worker counts, which is the determinism claim the oracle
// depends on. No files, no wall clock, no sockets: time is a fixed dt passed to
// b3World_Step, and the step count is a plain counter.

#include "benchmarks.h"
#include "test_macros.h"

#include "box3d/box3d.h"

#include <inttypes.h>
#include <stdio.h>

#define PERF_HASH_BOX_COUNT 10
#define PERF_HASH_STEP_COUNT 30

#if defined( BOX3D_LUDICROUS_MODE )
#define PERF_HASH_GOLDEN UINT64_C( 0x70fd3f65c8cba2ed )
#else
#define PERF_HASH_GOLDEN UINT64_C( 0xbeaf9169c082a5e5 )
#endif

// Ten dynamic boxes dropped in a line so they collide and settle, stepped at a fixed dt.
static uint64_t RunPerfHash( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId bodies[PERF_HASH_BOX_COUNT];
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	for ( int i = 0; i < PERF_HASH_BOX_COUNT; ++i )
	{
		bodyDef.position = (b3Pos){ b3FixFromInt( i - PERF_HASH_BOX_COUNT / 2 ), b3FixFromInt( 2 + 2 * i ), B3_FIX( 0.0f ) };
		bodies[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodies[i], &shapeDef, &box.base );
	}

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) );
	for ( int step = 0; step < PERF_HASH_STEP_COUNT; ++step )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	uint64_t hash = UINT64_C( 14695981039346656037 );
	for ( int i = 0; i < PERF_HASH_BOX_COUNT; ++i )
	{
		hash = b3HashBodyState( hash, bodies[i] );
	}

	b3DestroyWorld( worldId );

	return hash;
}

static int PerfStateHashGoldenTest( void )
{
	uint64_t hash1 = RunPerfHash( 1 );
	if ( hash1 != PERF_HASH_GOLDEN )
	{
		printf( "  perf hash workers=1 hash=%016" PRIx64 " golden=%016" PRIx64 "\n", hash1, PERF_HASH_GOLDEN );
	}
	ENSURE( hash1 == PERF_HASH_GOLDEN );

	uint64_t hash4 = RunPerfHash( 4 );
	if ( hash4 != hash1 )
	{
		printf( "  perf hash workers=4 hash=%016" PRIx64 " workers=1=%016" PRIx64 "\n", hash4, hash1 );
	}
	ENSURE( hash4 == hash1 );

	return 0;
}

int PerfHashTest( void )
{
	RUN_SUBTEST( PerfStateHashGoldenTest );

	return 0;
}
