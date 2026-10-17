// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
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
#define RESTITUTION_ITERATIONS 4

// Run the entire measurement battery so a failing case does not conceal later measurements.
#define RUN_MEASUREMENT( T )                                                                                                     \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( T() != 0 )                                                                                                          \
		{                                                                                                                        \
			printf( "  subtest failed: " #T "\n" );                                                                              \
			failureCount += 1;                                                                                                   \
		}                                                                                                                        \
		else                                                                                                                     \
		{                                                                                                                        \
			printf( "  subtest passed: " #T "\n" );                                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

// These exact upstream assertions are retained for a named follow-up card. An unexpected pass is
// reported and fails this suite so the marker cannot silently become stale.
#define RUN_BASELINE_XFAIL( T, CARD )                                                                                                  \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( T() == 0 )                                                                                                          \
		{                                                                                                                        \
			printf( "  XPASS: " #T " (" CARD ")\n" );                                                                    \
			failureCount += 1;                                                                                                   \
		}                                                                                                                        \
		else                                                                                                                     \
		{                                                                                                                        \
			printf( "  XFAIL: " #T " (" CARD ")\n" );                                                                    \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static b3WorldId MakeWorld( b3Fixed gravityY )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ B3_FIX( 0.0f ), gravityY, B3_FIX( 0.0f ) };
	worldDef.enableSleep = false;
	return b3CreateWorld( &worldDef );
}

static b3WorldId MakeWorldIterations( b3Fixed gravityY, int iterations )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ B3_FIX( 0.0f ), gravityY, B3_FIX( 0.0f ) };
	worldDef.enableSleep = false;
	worldDef.restitutionIterations = iterations;
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
	shapeDef.density = B3_FIX( 1.0f );
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
	// The upstream 79fc559 default is two stage sweeps.
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
	static const b3Fixed restitutions[] = { B3_FIX( 0.0f ), B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.75f ), B3_FIX( 1.0f ) };
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

// Unequal masses pin momentum conservation in the two-body Poisson stage.
static int HeadOnMassTest( void )
{
	static const b3Fixed e[] = { B3_FIX( 0.0f ), B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.75f ), B3_FIX( 1.0f ) };
	static const b3Fixed densities[] = { B3_FIX( 1.0f ), B3_FIX( 10.0f ), B3_FIX( 100.0f ) };
	int failed = 0;
	for ( int j = 0; j < ARRAY_COUNT( densities ); ++j )
	{
		double worstError = 0.0, worstMomentum = 0.0;
		for ( int k = 0; k < ARRAY_COUNT( e ); ++k )
		{
			b3WorldId worldId = MakeWorld( B3_FIX( 0.0f ) );
			b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			shapeDef.density = B3_FIX( 1.0f );
			shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
			shapeDef.baseMaterial.restitution = e[k];
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
			bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 2.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
			b3BodyId a = b3CreateBody( worldId, &bodyDef );
			b3CreateSphereShape( a, &shapeDef, &sphere );
			bodyDef.position.x = B3_FIX( 1.0f );
			bodyDef.linearVelocity.x = -B3_FIX( 2.5f );
			b3BodyId b = b3CreateBody( worldId, &bodyDef );
			shapeDef.density = densities[j];
			b3CreateSphereShape( b, &shapeDef, &sphere );
			double massA = b3FixToDouble( b3Body_GetMass( a ) ), massB = b3FixToDouble( b3Body_GetMass( b ) );
			double momentum0 = 2.5 * ( massA - massB );
			for ( int i = 0; i < 120; ++i )
				b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
			double va = b3FixToDouble( b3Body_GetLinearVelocity( a ).x );
			double vb = b3FixToDouble( b3Body_GetLinearVelocity( b ).x );
			double measured = ( vb - va ) / 5.0;
			double error = measured - b3FixToDouble( e[k] );
			if ( error < 0 )
				error = -error;
			double momentum = ( massA * va + massB * vb - momentum0 ) / ( massA + massB );
			if ( momentum < 0 )
				momentum = -momentum;
			if ( error > worstError )
				worstError = error;
			if ( momentum > worstMomentum )
				worstMomentum = momentum;
			b3DestroyWorld( worldId );
		}
		printf( "    head on density %.0f worst coefficient error %.6f momentum drift %.6g\n", b3FixToDouble( densities[j] ),
				worstError, worstMomentum );
		if ( worstError > .02 || worstMomentum > 8.0 * b3FixToDouble( B3_FIXED_EPSILON ) )
			failed = 1;
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
		b3BodyId ballId =
			MakeBall( worldId, B3_FIX( 0.0f ), b3FixMul( B3_FIX( 0.5f ), speed ) + b3FixMul( B3_FIX( 0.5f ), TIME_STEP ), -speed,
					  B3_FIX( 1.0f ) );

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
	shapeDef.density = B3_FIX( 1.0f );
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

// The phase of the first contact within a fixed time step must not change the coefficient.
static int PhaseTest( void )
{
	static const b3Fixed restitutions[] = { B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.9f ) };
	int failed = 0;
	for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
	{
		double lo = 1e30, hi = -1e30, sum = 0.0;
		for ( int i = 0; i < 16; ++i )
		{
			b3WorldId worldId = MakeWorld( B3_FIX( 0.0f ) );
			MakeGround( worldId, B3_FIX( 0.0f ) );
			b3Fixed gap = b3FixDiv( b3FixMul( b3FixFromInt( i ), b3FixMul( IMPACT_SPEED, TIME_STEP ) ), B3_FIX( 16.0f ) );
			b3BodyId ball = MakeBall( worldId, B3_FIX( 0.0f ), B3_FIX( 0.5f ) + gap, -IMPACT_SPEED, restitutions[j] );
			for ( int k = 0; k < 60; ++k )
				b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
			double value = b3FixToDouble( b3Body_GetLinearVelocity( ball ).y ) / 5.0;
			if ( value < lo )
				lo = value;
			if ( value > hi )
				hi = value;
			sum += value;
			b3DestroyWorld( worldId );
		}
		double mean = sum / 16.0, target = b3FixToDouble( restitutions[j] );
		printf( "    phase e %.2f -> mean %.4f spread %.4f [%.4f, %.4f]\n", target, mean, hi - lo, lo, hi );
		if ( hi - lo > 0.05 || mean - target > 0.05 || target - mean > 0.05 )
			failed = 1;
	}
	return failed;
}

static int SupportedTest( void )
{
	static const b3Fixed restitutions[] = { B3_FIX( 0.5f ), B3_FIX( 0.9f ) };
	int failed = 0;
	for ( int propagate = 0; propagate <= 1; ++propagate )
		for ( int j = 0; j < ARRAY_COUNT( restitutions ); ++j )
			for ( int n = 0; n <= 3; ++n )
			{
				b3WorldDef def = b3DefaultWorldDef();
				def.gravity = b3Vec3_zero;
				def.enableSleep = false;
				def.enableRestitutionPropagation = propagate != 0;
				def.restitutionIterations = RESTITUTION_ITERATIONS;
				b3WorldId worldId = b3CreateWorld( &def );
				MakeGround( worldId, B3_FIX( 0.0f ) );
				for ( int i = 0; i < n; ++i )
					MakeBall( worldId, B3_FIX( 0.0f ), B3_FIX( 0.5f ) + b3FixFromInt( i ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) );
				b3Fixed y = B3_FIX( 0.5f ) + b3FixFromInt( n ) + b3FixMul( B3_FIX( 2.5f ), TIME_STEP );
				b3BodyId ball = MakeBall( worldId, B3_FIX( 0.0f ), y, -IMPACT_SPEED, restitutions[j] );
				for ( int i = 0; i < 60; ++i )
					b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
				double value = b3FixToDouble( b3Body_GetLinearVelocity( ball ).y ) / 5.0;
				double target = b3FixToDouble( restitutions[j] );
				printf( "    supported e %.2f supports %d propagation %d -> %.4f\n", target, n, propagate, value );
				if ( value > target + 0.01 || value < ( propagate ? 0.75 : 0.5 ) * target )
					failed = 1;
				b3DestroyWorld( worldId );
			}
	return failed;
}

static void MakeFlatBox( b3WorldId worldId, b3Fixed restitution, b3Fixed spin, b3BodyId* id )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 0.25f ) + b3FixMul( B3_FIX( 2.5f ), TIME_STEP ), B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 0.0f ), -IMPACT_SPEED, B3_FIX( 0.0f ) };
	bodyDef.angularVelocity = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), spin };
	*id = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
	shapeDef.baseMaterial.restitution = restitution;
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 0.25f ), B3_FIX( 1.0f ) );
	b3CreateHullShape( *id, &shapeDef, &box.base );
}

static int FlatLandingTest( void )
{
	static const b3Fixed e[] = { B3_FIX( 0.5f ), B3_FIX( 0.9f ) };
	int failed = 0;
	for ( int j = 0; j < ARRAY_COUNT( e ); ++j )
		for ( int substeps = 4; substeps <= 8; substeps += 4 )
		{
			b3WorldId worldId = MakeWorldIterations( B3_FIX( 0.0f ), RESTITUTION_ITERATIONS );
			MakeGround( worldId, B3_FIX( 0.0f ) );
			b3BodyId box;
			MakeFlatBox( worldId, e[j], B3_FIX( 0.0f ), &box );
			for ( int i = 0; i < 60; ++i )
				b3World_Step( worldId, TIME_STEP, substeps );
			double speed = b3FixToDouble( b3Body_GetLinearVelocity( box ).y ) / 5.0;
			double spin = b3FixToDouble( b3Length( b3Body_GetAngularVelocity( box ) ) );
			printf( "    flat e %.2f substeps %d -> %.4f spin %.4f\n", b3FixToDouble( e[j] ), substeps, speed, spin );
			if ( substeps == 4 &&
				 ( speed > b3FixToDouble( e[j] ) + 0.01 || speed < b3FixToDouble( e[j] ) - 0.01 || spin > 0.03 ) )
				failed = 1;
			b3DestroyWorld( worldId );
		}
	return failed;
}

static int SpinTest( void )
{
	static const b3Fixed spins[] = { B3_FIX( 0.0f ), B3_FIX( 0.5f ), B3_FIX( 1.0f ), B3_FIX( 2.0f ) };
	int failed = 0;
	for ( int j = 0; j < ARRAY_COUNT( spins ); ++j )
	{
		b3WorldId worldId = MakeWorldIterations( B3_FIX( 0.0f ), RESTITUTION_ITERATIONS );
		MakeGround( worldId, B3_FIX( 0.0f ) );
		b3BodyId box;
		MakeFlatBox( worldId, B3_FIX( 1.0f ), spins[j], &box );
		for ( int i = 0; i < 60; ++i )
			b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		double vy = b3FixToDouble( b3Body_GetLinearVelocity( box ).y );
		double wz = b3FixToDouble( b3Body_GetAngularVelocity( box ).z );
		double target = b3FixToDouble( spins[j] );
		printf( "    spin in %+.2f -> vy %+.4f wz %+.4f\n", target, vy, wz );
		if ( vy > 5.05 || vy < 4.95 || wz > -target + 0.05 || wz < -target - 0.05 )
			failed = 1;
		b3DestroyWorld( worldId );
	}
	return failed;
}

static int DropTest( void )
{
	static const b3Fixed heights[] = { B3_FIX( 40.0f ), B3_FIX( 20.0f ), B3_FIX( 10.0f ) };
	int failed = 0;
	for ( int j = 0; j < ARRAY_COUNT( heights ); ++j )
	{
		b3WorldId worldId = MakeWorld( -B3_FIX( 10.0f ) );
		MakeGround( worldId, B3_FIX( 0.0f ) );
		b3BodyId ball = MakeBall( worldId, B3_FIX( 0.0f ), heights[j], B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
		double apex[6] = { 0 };
		int count = 0;
		b3Fixed before = B3_FIX( 0.0f );
		for ( int i = 0; i < 4000 && count < 6; ++i )
		{
			b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
			b3Fixed speed = b3Body_GetLinearVelocity( ball ).y;
			if ( before > 0 && speed <= 0 )
				apex[count++] = b3FixToDouble( b3ToVec3( b3Body_GetPosition( ball ) ).y );
			before = speed;
		}
		printf( "    drop %.1f count %d", b3FixToDouble( heights[j] ), count );
		double highest = apex[0];
		for ( int i = 0; i < count; ++i )
		{
			printf( " %.3f", apex[i] );
			if ( apex[i] > highest )
				highest = apex[i];
		}
		printf( "\n" );
		if ( count < 6 || highest > 1.001 * apex[0] || apex[0] > 1.001 * b3FixToDouble( heights[j] ) || apex[5] < 0.8 * apex[0] )
			failed = 1;
		b3DestroyWorld( worldId );
	}
	return failed;
}

static int SingleBoxTest( void )
{
	b3WorldId worldId = MakeWorldIterations( -B3_FIX( 10.0f ), RESTITUTION_ITERATIONS );
	MakeGround( worldId, B3_FIX( 0.0f ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
	shapeDef.baseMaterial.restitution = B3_FIX( 1.0f );
	b3BoxHull cube = b3MakeCubeHull( B3_FIX( 0.5f ) );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 10.0f ), B3_FIX( 0.0f ) };
	bodyDef.safetyFactor = B3_FIX( 0.01f );
	b3BodyId box = b3CreateBody( worldId, &bodyDef );
	b3CreateHullShape( box, &shapeDef, &cube.base );
	double firstSpin = 0.0, apexes[4] = { 0 };
	int count = 0;
	b3Fixed before = B3_FIX( 0.0f );
	for ( int i = 0; i < 1200 && count < 4; ++i )
	{
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		b3Fixed speed = b3Body_GetLinearVelocity( box ).y;
		if ( count == 0 && before <= 0 && speed > 0 )
			firstSpin = b3FixToDouble( b3Length( b3Body_GetAngularVelocity( box ) ) );
		if ( before > 0 && speed <= 0 )
			apexes[count++] = b3FixToDouble( b3ToVec3( b3Body_GetPosition( box ) ).y );
		before = speed;
	}
	b3DestroyWorld( worldId );
	printf( "    single box first bounce spin %.4f apexes", firstSpin );
	int failed = count != 4 || firstSpin > 0.01;
	for ( int i = 0; i < count; ++i )
	{
		printf( " %.3f", apexes[i] );
		if ( apexes[i] < 9.5 || apexes[i] > 10.01 )
			failed = 1;
	}
	printf( "\n" );
	return failed;
}

static b3Fixed MeasureFirstTouchNormalVelocity( bool enableHitEvents )
{
	b3WorldId worldId = MakeWorld( B3_FIX( 0.0f ) );
	MakeGround( worldId, B3_FIX( 0.0f ) );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 0.6f ), B3_FIX( 0.0f ) };
	b3BodyId ball = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = B3_FIX( 0.0f );
	shapeDef.enableHitEvents = enableHitEvents;
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3CreateSphereShape( ball, &shapeDef, &sphere );
	b3Fixed normalVelocity = B3_FIXED_MAX;
	for ( int i = 0; i < 20; ++i )
	{
		b3Body_SetLinearVelocity( ball, (b3Vec3){ B3_FIX( 0.0f ), -IMPACT_SPEED, B3_FIX( 0.0f ) } );
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		b3ContactData data;
		if ( b3Body_GetContactData( ball, &data, 1 ) == 1 && data.manifoldCount > 0 && data.manifolds[0].pointCount > 0 )
		{
			normalVelocity = data.manifolds[0].points[0].normalVelocity;
			break;
		}
	}
	b3DestroyWorld( worldId );
	return normalVelocity;
}

static int NormalVelocityTest( void )
{
	b3Fixed quiet = MeasureFirstTouchNormalVelocity( false );
	b3Fixed published = MeasureFirstTouchNormalVelocity( true );
	printf( "    normal velocity hit events off %.4f on %.4f\n", b3FixToDouble( quiet ), b3FixToDouble( published ) );
	return quiet != B3_FIX( 0.0f ) || published > -B3_FIX( 4.0f );
}

// Match upstream's symplectic-Euler energy observable at step boundaries. The conversion to
// double is a test-only measurement boundary; the simulated world remains entirely fixed point.
static double MeasureEnergy( b3BodyId bodyId, b3WorldId worldId )
{
	b3MassData md = b3Body_GetMassData( bodyId );
	b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
	b3Vec3 g = b3MulSV( b3Body_GetGravityScale( bodyId ), b3World_GetGravity( worldId ) );
	b3Vec3 w = b3InvRotateVector( b3Body_GetRotation( bodyId ), b3Body_GetAngularVelocity( bodyId ) );
	double mass = b3FixToDouble( md.mass );
	double kinetic = 0.5 * mass * b3FixToDouble( b3Dot( v, v ) ) + 0.5 * b3FixToDouble( b3Dot( w, b3MulMV( md.inertia, w ) ) );
	double potential = -mass * b3FixToDouble( b3Dot( g, b3ToVec3( b3Body_GetWorldCenter( bodyId ) ) ) );
	double phase = 0.5 * ( b3FixToDouble( TIME_STEP ) / SUB_STEP_COUNT ) * mass * b3FixToDouble( b3Dot( v, g ) );
	return kinetic + potential + phase;
}

typedef struct OvershootResult
{
	double firstApex;
	double firstEnergyRatio;
	double peakEnergyRatio;
	double speedRatio;
	int bounceCount;
} OvershootResult;

static OvershootResult MeasureOvershoot( bool continuous )
{
	b3WorldId worldId = MakeWorldIterations( -B3_FIX( 10.0f ), RESTITUTION_ITERATIONS );
	b3BodyDef floorDef = b3DefaultBodyDef();
	floorDef.position = (b3Pos){ B3_FIX( 0.0f ), -B3_FIX( 0.25f ), B3_FIX( 0.0f ) };
	b3BodyId floorId = b3CreateBody( worldId, &floorDef );
	b3ShapeDef floorShape = b3DefaultShapeDef();
	b3BoxHull floor = b3MakeBoxHull( B3_FIX( 0.375f ), B3_FIX( 0.25f ), B3_FIX( 0.375f ) );
	b3CreateHullShape( floorId, &floorShape, &floor.base );
	b3BodyDef boxDef = b3DefaultBodyDef();
	boxDef.type = b3_dynamicBody;
	boxDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 10.0f ), B3_FIX( 0.0f ) };
	if ( continuous )
		boxDef.safetyFactor = B3_FIX( 0.1f );
	b3BodyId boxId = b3CreateBody( worldId, &boxDef );
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.baseMaterial.restitution = B3_FIX( 1.0f );
	b3BoxHull box = b3MakeCubeHull( B3_FIX( 0.5f ) );
	b3CreateHullShape( boxId, &boxShape, &box.base );
	OvershootResult result = { 0 };
	double start = MeasureEnergy( boxId, worldId ), impactSpeed = 0.0, reboundSpeed = 0.0, beforeHeight = 0.0;
	b3Fixed previous = B3_FIX( 0.0f );
	for ( int i = 0; i < 600 && result.bounceCount < 2; ++i )
	{
		b3Fixed before = b3Body_GetLinearVelocity( boxId ).y;
		double height = b3FixToDouble( b3ToVec3( b3Body_GetPosition( boxId ) ).y );
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		b3Fixed speed = b3Body_GetLinearVelocity( boxId ).y;
		if ( reboundSpeed == 0.0 && before < 0 && speed > 0 )
		{
			impactSpeed = -b3FixToDouble( before );
			reboundSpeed = b3FixToDouble( speed );
			beforeHeight = height;
		}
		if ( reboundSpeed == 0.0 )
			continue;
		double ratio = MeasureEnergy( boxId, worldId ) / start;
		if ( ratio > result.peakEnergyRatio )
			result.peakEnergyRatio = ratio;
		if ( previous > 0 && speed <= 0 )
		{
			if ( result.bounceCount == 0 )
			{
				result.firstApex = b3FixToDouble( b3ToVec3( b3Body_GetPosition( boxId ) ).y );
				result.firstEnergyRatio = ratio;
			}
			result.bounceCount++;
		}
		previous = speed;
	}
	result.speedRatio = impactSpeed > 0.0 ? reboundSpeed / impactSpeed : 0.0;
	printf( "    overshoot continuous %d apex %.4f at impact y %.4f speed ratio %.4f energy %.4f peak %.4f bounces %d\n",
			continuous ? 1 : 0, result.firstApex, beforeHeight, result.speedRatio, result.firstEnergyRatio,
			result.peakEnergyRatio, result.bounceCount );
	b3DestroyWorld( worldId );
	return result;
}

static int OvershootTest( void )
{
	OvershootResult discrete = MeasureOvershoot( false );
	OvershootResult continuous = MeasureOvershoot( true );
	// The fixed-point speed floor is eight quanta divided by the ~14 m/s approach.
	const double speedTolerance = 8.0 * b3FixToDouble( B3_FIXED_EPSILON );
	return discrete.speedRatio > 1.0 + speedTolerance || continuous.speedRatio > 1.0 + speedTolerance ||
		   continuous.firstApex > 10.0 || continuous.peakEnergyRatio > 1.001 || discrete.firstApex > 10.15 ||
		   discrete.firstEnergyRatio > 1.02 || discrete.bounceCount < 2 || continuous.bounceCount < 1;
}

typedef struct EnergyScene
{
	const char* name;
	double safetyFactor, friction, angle, spin, height, velocityY, halfWidth, halfHeight;
	bool bullet, stack;
	int stepCount;
} EnergyScene;

typedef struct EnergyImpact
{
	double beforeRatio, afterRatio, spinBefore;
} EnergyImpact;

static double MeasurePeakEnergy( const EnergyScene* scene, b3Fixed restitution, int iterations, EnergyImpact* impact )
{
	b3WorldId worldId = iterations == 0 ? MakeWorld( -B3_FIX( 10.0f ) ) : MakeWorldIterations( -B3_FIX( 10.0f ), iterations );
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	groundShape.baseMaterial.friction = b3FixFromDouble( scene->friction );
	b3BoxHull ground = b3MakeBoxHull( B3_FIX( 40.0f ), B3_FIX( 1.0f ), B3_FIX( 40.0f ) );
	b3CreateHullShape( groundId, &groundShape, &ground.base );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	shapeDef.baseMaterial.friction = b3FixFromDouble( scene->friction );
	shapeDef.baseMaterial.restitution = restitution;
	b3BoxHull box = b3MakeBoxHull( b3FixFromDouble( scene->halfWidth ), b3FixFromDouble( scene->halfHeight ),
								   b3FixFromDouble( scene->halfWidth ) );
	b3BodyId bodies[2];
	int count = 0;
	if ( scene->stack )
	{
		b3BodyDef def = b3DefaultBodyDef();
		def.type = b3_dynamicBody;
		def.position = (b3Pos){ B3_FIX( 0.0f ), b3FixFromDouble( scene->halfHeight ), B3_FIX( 0.0f ) };
		def.safetyFactor = b3FixFromDouble( scene->safetyFactor );
		bodies[count] = b3CreateBody( worldId, &def );
		b3CreateHullShape( bodies[count++], &shapeDef, &box.base );
	}
	b3BodyDef def = b3DefaultBodyDef();
	def.type = b3_dynamicBody;
	def.position = (b3Pos){ B3_FIX( 0.0f ), b3FixFromDouble( scene->height ), B3_FIX( 0.0f ) };
	def.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, b3FixFromDouble( scene->angle ) );
	def.angularVelocity = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), b3FixFromDouble( scene->spin ) };
	def.linearVelocity = (b3Vec3){ B3_FIX( 0.0f ), b3FixFromDouble( scene->velocityY ), B3_FIX( 0.0f ) };
	def.safetyFactor = b3FixFromDouble( scene->safetyFactor );
	def.isBullet = scene->bullet;
	bodies[count] = b3CreateBody( worldId, &def );
	b3CreateHullShape( bodies[count++], &shapeDef, &box.base );
	double start = 0.0;
	for ( int i = 0; i < count; ++i )
		start += MeasureEnergy( bodies[i], worldId );
	double peak = start;
	int peakStep = -1;
	for ( int step = 0; step < scene->stepCount; ++step )
	{
		if ( impact != NULL && step == 257 )
			impact->spinBefore = b3FixToDouble( b3Length( b3Body_GetAngularVelocity( bodies[count - 1] ) ) );
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		double energy = 0.0;
		for ( int i = 0; i < count; ++i )
			energy += MeasureEnergy( bodies[i], worldId );
		if ( impact != NULL && step == 256 )
			impact->beforeRatio = energy / start;
		if ( impact != NULL && step == 257 )
			impact->afterRatio = energy / start;
		if ( energy > peak )
		{
			peak = energy;
			peakStep = step;
		}
	}
	b3DestroyWorld( worldId );
	double ratio = peak / start;
	printf( "    energy %-24s peak %.5f at step %d\n", scene->name, ratio, peakStep );
	return ratio;
}

static const EnergyScene energyScenes[] = {
	{ "spinning edge", .01, .6, -1.43, 7.9, .7, -1.9, .5, .5, false, false, 300 },
	{ "spinning edge fast", .01, .6, -.8, 12, 3, -8, .5, .5, false, false, 300 },
	{ "tumble", .01, .6, -.3, 3, 8, 0, .5, .5, false, false, 600 },
	{ "tumble discrete", .5, .6, -.3, 3, 8, 0, .5, .5, false, false, 600 },
	{ "flat with spin", .01, .6, 0, 2, 2, -5, .5, .5, false, false, 300 },
	{ "flat frictionless", .01, 0, 0, 0, 10, 0, .5, .5, false, false, 1200 },
	{ "edge", .01, .6, -.2, 0, 5, 0, .5, .5, false, false, 600 },
	{ "two box", .02, .6, 0, 0, 4, 0, .5, .5, true, true, 1500 },
	{ "two box discrete", .5, .6, 0, 0, 4, 0, .5, .5, false, true, 1500 },
	{ "tall flat frictionless", .01, 0, 0, 0, 10, 0, .5, 1, false, false, 1200 },
	{ "tall flat friction", .01, .6, 0, 0, 10, 0, .5, 1, false, false, 1200 },
	{ "narrow flat frictionless", .01, 0, 0, 0, 10, 0, .25, 1, false, false, 1200 },
	{ "narrow flat friction", .01, .6, 0, 0, 10, 0, .25, 1, false, false, 1200 },
};

static int EnergyTest( void )
{
	int failed = 0;
	for ( int i = 0; i < ARRAY_COUNT( energyScenes ); ++i )
	{
		double ratio = MeasurePeakEnergy( energyScenes + i, B3_FIX( 1.0f ), RESTITUTION_ITERATIONS, NULL );
		double limit = energyScenes[i].safetyFactor < .1 ? .005 : .02;
		if ( ratio > 1.0 + limit )
			failed = 1;
	}
	return failed;
}

// Clean b79953a was measured with the same scenarios and restitution disabled. This checks
// that the new stage does not perturb the already-red baseline. Five elastic scenarios must
// also improve materially over the original one-sweep stage when pinned to four sweeps.
static int StageImprovementTest( void )
{
	static const double baselineZero[] = { 1.01079, 1.00232, 1.00198, 1.00000, 1.00001, 1.00972, 1.02509,
										   1.01935, 1.00000, 1.01412, 1.01412, 1.01412, 1.01412 };
	static const double baselineElastic[] = { 2.05140, 1.97161, 1.40277, 1.32737, 1.40940 };
	static const int improvedCases[] = { 5, 9, 10, 11, 12 };
	const double quantumFloor = 8.0 * b3FixToDouble( B3_FIXED_EPSILON );
	int failed = 0;
	for ( int i = 0; i < ARRAY_COUNT( energyScenes ); ++i )
	{
		double ratio = MeasurePeakEnergy( energyScenes + i, B3_FIX( 0.0f ), RESTITUTION_ITERATIONS, NULL );
		if ( ratio > baselineZero[i] + quantumFloor )
		{
			printf( "    e=0 regression %s: %.5f > %.5f\n", energyScenes[i].name, ratio, baselineZero[i] + quantumFloor );
			failed = 1;
		}
	}
	for ( int i = 0; i < ARRAY_COUNT( improvedCases ); ++i )
	{
		int index = improvedCases[i];
		double ratio = MeasurePeakEnergy( energyScenes + index, B3_FIX( 1.0f ), RESTITUTION_ITERATIONS, NULL );
		if ( ratio >= baselineElastic[i] - 0.01 )
		{
			printf( "    e=1 not improved %s: %.5f vs base %.5f\n", energyScenes[index].name, ratio, baselineElastic[i] );
			failed = 1;
		}
	}
	return failed;
}

// The configured default is a public behavior, so it must also improve over the
// clean-base one-sweep measurements. An explicit four-sweep test alone cannot
// establish this because callers that do not set the new field use this default.
static int DefaultRestitutionNoRegressionTest( void )
{
	static const int cases[] = { 5, 9, 10, 11, 12 };
	static const double baselineElastic[] = { 2.05140, 1.97161, 1.40277, 1.32737, 1.40940 };
	const double quantumFloor = 8.0 * b3FixToDouble( B3_FIXED_EPSILON );
	int failed = 0;
	for ( int i = 0; i < ARRAY_COUNT( cases ); ++i )
	{
		int index = cases[i];
		double ratio = MeasurePeakEnergy( energyScenes + index, B3_FIX( 1.0f ), 0, NULL );
		printf( "    default e=1 %-24s %.5f vs base %.5f\n", energyScenes[index].name, ratio, baselineElastic[i] );
		if ( ratio > baselineElastic[i] + quantumFloor )
			failed = 1;
	}
	return failed;
}

// A thin spinning box changes orientation substantially during its contact step.
// The restitution impulse must use inertia in that final orientation. The stale
// pre-step inertia raised normalized energy by 0.409 in this one step; with the
// current inertia the increase is 0.044. Keep this local check separate from
// the scene-wide peak comparison so the failure identifies its contact.
static int RotatedInertiaContactTest( void )
{
	EnergyImpact impact = { 0 };
	MeasurePeakEnergy( energyScenes + 11, B3_FIX( 1.0f ), 0, &impact );
	printf( "    rotated inertia contact spin %.3f energy %.5f -> %.5f\n", impact.spinBefore, impact.beforeRatio,
			impact.afterRatio );
	return impact.spinBefore < 10.0 || impact.afterRatio > impact.beforeRatio + 0.10;
}

typedef struct ImpulseResult
{
	double worstError, approachSpeed, firstSeparation, bounceImpulse;
	int contactSteps, toiSteps, toiImpulseSteps;
} ImpulseResult;

static ImpulseResult MeasureDropImpulse( b3Fixed restitution, b3Fixed dropHeight )
{
	b3WorldId worldId = MakeWorld( -B3_FIX( 10.0f ) );
	MakeGround( worldId, B3_FIX( 0.0f ) );
	b3BodyId ballId = MakeBall( worldId, B3_FIX( 0.0f ), B3_FIX( 0.5f ) + dropHeight, B3_FIX( 0.0f ), restitution );
	b3World* world = b3GetWorldFromId( worldId );
	double mass = b3FixToDouble( b3Body_GetMass( ballId ) );
	double gravity = b3FixToDouble( b3World_GetGravity( worldId ).y );
	ImpulseResult result = { 0 };
	bool touched = false, bouncing = false;
	for ( int i = 0; i < 240; ++i )
	{
		double before = b3FixToDouble( b3Body_GetLinearVelocity( ballId ).y );
		b3World_Step( worldId, TIME_STEP, SUB_STEP_COUNT );
		double after = b3FixToDouble( b3Body_GetLinearVelocity( ballId ).y );
		double measured = 0.0, separation = 0.0;
		b3ContactData data[4];
		int contactCount = b3Body_GetContactData( ballId, data, ARRAY_COUNT( data ) );
		for ( int c = 0; c < contactCount; ++c )
			for ( int m = 0; m < data[c].manifoldCount; ++m )
			{
				const b3Manifold* manifold = data[c].manifolds + m;
				for ( int point = 0; point < manifold->pointCount; ++point )
				{
					measured += b3FixToDouble( manifold->points[point].totalNormalImpulse );
					separation = b3FixToDouble( manifold->points[point].separation );
				}
			}
		b3Body* body = b3GetBodyFullId( world, ballId );
		b3BodySim* sim = b3GetBodySim( world, body );
		if ( ( sim->flags & b3_hadTimeOfImpact ) != 0 )
		{
			result.toiSteps++;
			if ( measured != 0.0 )
				result.toiImpulseSteps++;
			continue;
		}
		double expected = mass * ( after - before ) - mass * gravity * b3FixToDouble( TIME_STEP );
		double error = measured - expected;
		if ( error < 0.0 )
			error = -error;
		if ( error > result.worstError )
			result.worstError = error;
		if ( contactCount > 0 )
		{
			if ( !touched )
			{
				touched = bouncing = true;
				result.approachSpeed = -before;
				result.firstSeparation = separation;
			}
			if ( bouncing )
			{
				result.bounceImpulse += measured;
				result.contactSteps++;
			}
		}
		else
			bouncing = false;
	}
	b3DestroyWorld( worldId );
	return result;
}

static int ImpulseTest( void )
{
	static const b3Fixed e[] = { B3_FIX( 0.0f ), B3_FIX( 0.25f ), B3_FIX( 0.5f ), B3_FIX( 0.75f ), B3_FIX( 1.0f ) };
	static const b3Fixed heights[] = { B3_FIX( 1.0f ), B3_FIX( 5.0f ), B3_FIX( 20.0f ), B3_FIX( 45.0f ) };
	b3WorldId worldId = MakeWorld( -B3_FIX( 10.0f ) );
	b3BodyId ball = MakeBall( worldId, B3_FIX( 0.0f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) );
	double mass = b3FixToDouble( b3Body_GetMass( ball ) );
	double fast = b3FixToDouble( b3Body_GetSafetyFactor( ball ) ) * b3FixToDouble( b3Body_GetMinExtent( ball ) ) /
				  b3FixToDouble( TIME_STEP );
	b3DestroyWorld( worldId );
	double weight = mass * 10.0 * b3FixToDouble( TIME_STEP );
	int failed = 0;
	for ( int j = 0; j < ARRAY_COUNT( heights ); ++j )
	{
		double h = b3FixToDouble( heights[j] );
		double impactSpeed = b3FixToDouble( b3FixSqrt( b3FixMul( B3_FIX( 20.0f ), heights[j] ) ) );
		bool expectToi = impactSpeed > fast;
		// Mass-scaled eight-quantum floor replaces upstream's sub-quantum 1e-5 term.
		double tolerance = 8.0 * b3FixToDouble( B3_FIXED_EPSILON ) * mass * ( 10.0 + impactSpeed );
		for ( int k = 0; k < ARRAY_COUNT( e ); ++k )
		{
			ImpulseResult r = MeasureDropImpulse( e[k], heights[j] );
			double reversal = ( 1.0 + b3FixToDouble( e[k] ) ) * mass * r.approachSpeed;
			double lower = reversal - weight, upper = reversal + weight * r.contactSteps;
			printf( "    impulse drop %.1f e %.2f toi %d sep %+.4f step error %.6g bounce %.3f range [%.3f, %.3f] steps %d\n", h,
					b3FixToDouble( e[k] ), r.toiSteps, r.firstSeparation, r.worstError, r.bounceImpulse, lower, upper,
					r.contactSteps );
			double slack = r.contactSteps * tolerance;
			if ( r.worstError > tolerance || r.contactSteps == 0 || r.toiImpulseSteps > 0 || r.bounceImpulse < lower - slack ||
				 r.bounceImpulse > upper + slack || ( ( r.toiSteps > 0 ) != expectToi ) )
				failed = 1;
		}
	}
	return failed;
}

int RestitutionTest( void )
{
	int failureCount = 0;
	RUN_MEASUREMENT( IterationsApiTest );
	RUN_MEASUREMENT( HeadOnTest );
	RUN_MEASUREMENT( HeadOnMassTest );
	RUN_MEASUREMENT( PhaseTest );
	RUN_MEASUREMENT( SupportedTest );
	RUN_MEASUREMENT( FlatLandingTest );
	RUN_MEASUREMENT( SpinTest );
	RUN_MEASUREMENT( DropTest );
	RUN_MEASUREMENT( SingleBoxTest );
	RUN_MEASUREMENT( ThresholdTest );
	// Passes since the f555ee4 broad-phase port: sorted pair keys make the lower shape index
	// shape A, which changes the contact order of the stack, and the peak speed fell from 0.061
	// to 0.025 m/s, under the 0.05 bound. The deadband itself is not fixed: upstream reads
	// 0.00002 m/s here. Card fixed3d-orientation-integration-deadband stays open.
	RUN_MEASUREMENT( RestingTest );
	RUN_MEASUREMENT( NormalVelocityTest );
	RUN_MEASUREMENT( OvershootTest );
	RUN_MEASUREMENT( EnergyTest );
	RUN_MEASUREMENT( ImpulseTest );
	RUN_MEASUREMENT( StageImprovementTest );
	RUN_MEASUREMENT( RotatedInertiaContactTest );
	RUN_MEASUREMENT( DefaultRestitutionNoRegressionTest );
	RUN_MEASUREMENT( WorkerParityTest );
	return failureCount != 0;
}
