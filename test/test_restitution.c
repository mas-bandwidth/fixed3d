// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "physics_world.h"
#include "recording.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <stdio.h>

#define TIME_STEP b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) )
#define SUB_STEP_COUNT 4

// Impact speed shared by every scenario that measures a coefficient. Well above the default
// restitution threshold so the bounce is always armed.
#define IMPACT_SPEED B3_FIX( 5.0f )

static b3WorldId MakeWorld( b3Fixed gravityY )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ B3_FIX( 0.0f ), gravityY, B3_FIX( 0.0f ) };
	worldDef.enableSleep = false;
	return b3CreateWorld( &worldDef );
}

// Ground with its top surface at y = 0
static void MakeGround( b3WorldId worldId, b3Fixed restitution )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
	shapeDef.baseMaterial.restitution = restitution;
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 40.0f ), B3_FIX( 1.0f ), B3_FIX( 40.0f ) );
	b3CreateHullShape( groundId, &shapeDef, &box.base );
}

static b3BodyId MakeBall( b3WorldId worldId, b3Fixed x, b3Fixed y, b3Fixed velocityY, b3Fixed restitution )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ x, y, B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 0.0f ), velocityY, B3_FIX( 0.0f ) };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
	shapeDef.baseMaterial.restitution = restitution;
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	return bodyId;
}

// The new world-def API: iterations clamp, get/set round-trip, propagation flag.
static int IterationsApiTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	// Default matches B3_RESTITUTION_ITERATIONS (1) so goldens do not move.
	ENSURE( worldDef.restitutionIterations == B3_RESTITUTION_ITERATIONS );
	ENSURE( worldDef.enableRestitutionPropagation == false );

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	ENSURE( b3World_GetRestitutionIterations( worldId ) == B3_RESTITUTION_ITERATIONS );
	ENSURE( b3World_IsRestitutionPropagationEnabled( worldId ) == false );

	b3World_SetRestitutionIterations( worldId, 4 );
	ENSURE( b3World_GetRestitutionIterations( worldId ) == 4 );

	b3World_SetRestitutionIterations( worldId, 1000 );
	ENSURE( b3World_GetRestitutionIterations( worldId ) == B3_MAX_RESTITUTION_ITERATIONS );

	b3World_SetRestitutionIterations( worldId, -5 );
	ENSURE( b3World_GetRestitutionIterations( worldId ) == 0 );

	b3World_EnableRestitutionPropagation( worldId, true );
	ENSURE( b3World_IsRestitutionPropagationEnabled( worldId ) == true );

	b3DestroyWorld( worldId );
	return 0;
}

// Two free spheres closing head on with no gravity. The coefficient is the ratio of relative normal
// speeds at the contact, which is the mass independent definition.
static b3Fixed MeasureHeadOn( b3Fixed restitution )
{
	b3WorldId worldId = MakeWorld( B3_FIX( 0.0f ) );

	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
	shapeDef.baseMaterial.restitution = restitution;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ b3FixDiv( IMPACT_SPEED, B3_FIX( 2.0f ) ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId idA = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( idA, &shapeDef, &sphere );

	bodyDef.position = (b3Pos){ B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ -b3FixDiv( IMPACT_SPEED, B3_FIX( 2.0f ) ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId idB = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( idB, &shapeDef, &sphere );

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	b3Fixed vA = b3Body_GetLinearVelocity( idA ).x;
	b3Fixed vB = b3Body_GetLinearVelocity( idB ).x;

	b3DestroyWorld( worldId );

	return b3FixDiv( vB - vA, IMPACT_SPEED );
}

static int HeadOnTest( void )
{
	static const b3Fixed restitutions[] = { B3_FIX( 0.0f ), B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.75f ),
											B3_FIX( 1.0f ) };
	// The coefficient is a difference of velocities divided by the impact speed; the whole
	// pipeline is a handful of quanta, so 0.05 (about 400 quanta) is generous.
	const b3Fixed tolerance = B3_FIX( 0.05f );

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
	{
		b3Fixed measured = MeasureHeadOn( restitutions[j] );
		b3Fixed error = measured - restitutions[j];
		if ( error < B3_FIX( 0.0f ) )
		{
			error = -error;
		}
		printf( "    head on e %.2f -> %.4f\n", b3FixToDouble( restitutions[j] ), b3FixToDouble( measured ) );

		if ( error > tolerance )
		{
			failed = 1;
		}
	}

	return failed;
}

static int ThresholdTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3Fixed threshold = worldDef.restitutionThreshold;

	static const b3Fixed scales[] = { B3_FIX( 0.5f ), B3_FIX( 0.9f ), B3_FIX( 1.5f ), B3_FIX( 4.0f ) };

	int failed = 0;

	for ( int j = 0; j < ARRAY_COUNT( scales ); ++j )
	{
		b3Fixed speed = b3FixMul( scales[j], threshold );

		b3WorldId worldId = MakeWorld( B3_FIX( 0.0f ) );
		MakeGround( worldId, B3_FIX( 0.0f ) );
		b3BodyId ballId = MakeBall( worldId, B3_FIX( 0.0f ), b3FixMul( B3_FIX( 0.5f ), speed ) + b3FixMul( B3_FIX( 0.5f ), TIME_STEP ),
									-speed, B3_FIX( 1.0f ) );

		for ( int i = 0; i < 60; ++i )
		{
			b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		}

		b3Fixed ratio = b3FixDiv( b3Body_GetLinearVelocity( ballId ).y, speed );
		b3DestroyWorld( worldId );

		printf( "    threshold %.2fx -> %.4f\n", b3FixToDouble( scales[j] ), b3FixToDouble( ratio ) );

		if ( scales[j] < B3_FIX( 1.0f ) && ratio > B3_FIX( 0.05f ) )
		{
			failed = 1;
		}

		if ( scales[j] > B3_FIX( 1.0f ) && ratio < B3_FIX( 0.9f ) )
		{
			failed = 1;
		}
	}

	return failed;
}

// High restitution must not wake a settled stack back up. The apex of a high-restitution stack can
// jitter by a few quanta while it sleeps, so the drift bound is widened past the single-quantum
// no-op and measured.
static int RestingTest( void )
{
	b3WorldId worldId = MakeWorld( -B3_FIX( 10.0f ) );
	MakeGround( worldId, B3_FIX( 0.9f ) );

	const int boxCount = 10;
	b3BodyId boxIds[10];

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = B3_FIX( 0.6f );
	shapeDef.baseMaterial.restitution = B3_FIX( 0.9f );
	b3BoxHull box = b3MakeCubeHull( B3_FIX( 0.5f ) );

	for ( int i = 0; i < boxCount; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 0.5f ) + b3FixFromInt( i ), B3_FIX( 0.0f ) };
		boxIds[i] = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( boxIds[i], &shapeDef, &box.base );
	}

	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	b3Fixed settled = b3Body_GetPosition( boxIds[boxCount - 1] ).y;
	b3Fixed drift = B3_FIX( 0.0f );
	b3Fixed peakSpeed = B3_FIX( 0.0f );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );

		b3Fixed y = b3Body_GetPosition( boxIds[boxCount - 1] ).y;
		b3Fixed d = y - settled;
		if ( d < B3_FIX( 0.0f ) )
		{
			d = -d;
		}
		drift = b3FixMax( drift, d );

		for ( int j = 0; j < boxCount; ++j )
		{
			peakSpeed = b3FixMax( peakSpeed, b3Length( b3Body_GetLinearVelocity( boxIds[j] ) ) );
		}
	}

	b3DestroyWorld( worldId );

	printf( "    resting drift %.5f peak speed %.5f\n", b3FixToDouble( drift ), b3FixToDouble( peakSpeed ) );

	ENSURE( drift < B3_FIX( 0.01f ) );
	ENSURE( peakSpeed < B3_FIX( 0.05f ) );
	return 0;
}

static uint64_t RunWorkerScene( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 10.0f ), B3_FIX( 0.0f ) };
	worldDef.enableSleep = false;
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	MakeGround( worldId, B3_FIX( 0.6f ) );

	for ( int i = 0; i < 20; ++i )
	{
		MakeBall( worldId, -B3_FIX( 10.0f ) + b3FixMul( B3_FIX( 1.05f ), b3FixFromInt( i ) ),
				  B3_FIX( 3.0f ) + b3FixMul( B3_FIX( 0.13f ), b3FixFromInt( i ) ), B3_FIX( 0.0f ), B3_FIX( 0.6f ) );
	}

	for ( int i = 0; i < 200; ++i )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
	}

	uint64_t hash = b3HashWorldState( b3GetWorldFromId( worldId ) );
	b3DestroyWorld( worldId );
	return hash;
}

// The per-point Poisson accumulator and the approach speed must survive the split across workers.
static int WorkerParityTest( void )
{
	uint64_t hash1 = RunWorkerScene( 1 );
	uint64_t hash2 = RunWorkerScene( 2 );
	uint64_t hash4 = RunWorkerScene( 4 );

	printf( "    worker hashes 0x%016llx 0x%016llx 0x%016llx\n", (unsigned long long)hash1, (unsigned long long)hash2,
			(unsigned long long)hash4 );

	ENSURE( hash1 == hash2 );
	ENSURE( hash1 == hash4 );
	return 0;
}

int RestitutionTest( void )
{
	RUN_SUBTEST( IterationsApiTest );
	RUN_SUBTEST( HeadOnTest );
	RUN_SUBTEST( ThresholdTest );
	RUN_SUBTEST( RestingTest );
	RUN_SUBTEST( WorkerParityTest );
	return 0;
}