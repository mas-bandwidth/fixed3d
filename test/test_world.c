// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"
#include "overflow_color.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <stdio.h>
#include <string.h>

// This is a simple example of building and running a simulation
// using Fixed3D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Fixed3D is meant to be used
// with your rendering engine in your game engine.
int HelloWorld( void )
{
	// Construct a world object, which will hold and simulate the rigid bodies.
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 10.0f ), B3_FIX( 0.0f ) };

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	// Define the ground body.
	b3BodyDef groundBodyDef = b3DefaultBodyDef();
	groundBodyDef.position = (b3Pos){ B3_FIX( 0.0f ), -B3_FIX( 10.0f ), B3_FIX( 0.0f ) };

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	b3BodyId groundId = b3CreateBody( worldId, &groundBodyDef );
	ENSURE( b3Body_IsValid( groundId ) );

	// Define the ground box shape. The extents are the half-widths of the box.
	b3BoxHull groundBox = b3MakeBoxHull( B3_FIX( 50.0f ), B3_FIX( 10.0f ), B3_FIX( 50.0f ) );

	// Add the box shape to the ground body.
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Define the dynamic body. We set its position and call the body factory.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };

	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// Define another box shape for our dynamic body.
	b3BoxHull dynamicBox = b3MakeCubeHull( B3_FIX( 1.0f ) );

	// Define the dynamic body shape
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = B3_FIX( 1.0f );

	// Override the default friction.
	shapeDef.baseMaterial.friction = B3_FIX( 0.3f );

	// Add the shape to the body.
	b3CreateHullShape( bodyId, &shapeDef, &dynamicBox.base );

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
	int subStepCount = 4;

	b3Pos position = b3Body_GetPosition( bodyId );
	b3Quat rotation = b3Body_GetRotation( bodyId );

	// This is our little game loop.
	for ( int i = 0; i < 90; ++i )
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b3World_Step( worldId, timeStep, subStepCount );

		// Now print the position and angle of the body.
		position = b3Body_GetPosition( bodyId );
		rotation = b3Body_GetRotation( bodyId );

		// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b3Rot_GetAngle(rotation));
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned ids, so be careful about your world management.
	b3DestroyWorld( worldId );

	ENSURE_SMALL( position.y - B3_FIX( 1.00f ), B3_FIX( 0.01f ) );
	ENSURE_SMALL( rotation.v.x, B3_FIX( 0.01f ) );
	ENSURE_SMALL( rotation.v.z, B3_FIX( 0.01f ) );

	return 0;
}

int EmptyWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) == true );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
	int subStepCount = 1;

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );

	return 0;
}

#define BODY_COUNT 10
int DestroyAllBodiesWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) == true );

	int count = 0;
	bool creating = true;

	b3BodyId bodyIds[BODY_COUNT];
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BoxHull cube = b3MakeCubeHull( B3_FIX( 0.5f ) );

	for ( int i = 0; i < 2 * BODY_COUNT + 10; ++i )
	{
		if ( creating )
		{
			if ( count < BODY_COUNT )
			{
				bodyIds[count] = b3CreateBody( worldId, &bodyDef );

				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3CreateHullShape( bodyIds[count], &shapeDef, &cube.base );
				count += 1;
			}
			else
			{
				creating = false;
			}
		}
		else if ( count > 0 )
		{
			b3DestroyBody( bodyIds[count - 1] );
			bodyIds[count - 1] = b3_nullBodyId;
			count -= 1;
		}

		b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 3 );
	}

	b3Counters counters = b3World_GetCounters( worldId );
	ENSURE( counters.bodyCount == 0 );

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );

	return 0;
}

static int TestIsValid( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();

	b3BodyId bodyId1 = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId1 ) == true );

	b3BodyId bodyId2 = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId2 ) == true );

	b3DestroyBody( bodyId1 );
	ENSURE( b3Body_IsValid( bodyId1 ) == false );

	b3DestroyBody( bodyId2 );
	ENSURE( b3Body_IsValid( bodyId2 ) == false );

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );
	ENSURE( b3Body_IsValid( bodyId2 ) == false );
	ENSURE( b3Body_IsValid( bodyId1 ) == false );

	return 0;
}

#define WORLD_COUNT ( B3_MAX_WORLDS / 2 )

int TestWorldRecycle( void )
{
	_Static_assert( WORLD_COUNT > 0, "world count" );

	int count = 100;

	b3WorldId worldIds[WORLD_COUNT];

	for ( int i = 0; i < count; ++i )
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			worldIds[j] = b3CreateWorld( &worldDef );
			ENSURE( b3World_IsValid( worldIds[j] ) == true );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3CreateBody( worldIds[j], &bodyDef );
		}

		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
			int subStepCount = 1;

			for ( int k = 0; k < 10; ++k )
			{
				b3World_Step( worldIds[j], timeStep, subStepCount );
			}
		}

		for ( int j = WORLD_COUNT - 1; j >= 0; --j )
		{
			b3DestroyWorld( worldIds[j] );
			ENSURE( b3World_IsValid( worldIds[j] ) == false );
			worldIds[j] = b3_nullWorldId;
		}
	}

	return 0;
}

static bool CustomFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	ENSURE( context == NULL );
	return true;
}

static bool PreSolveStatic( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	(void)point;
	(void)normal;
	ENSURE( context == NULL );
	return false;
}

// This test is here to ensure all API functions link correctly.
int TestWorldCoverage( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3World_EnableSleeping( worldId, true );
	b3World_EnableSleeping( worldId, false );
	bool flag = b3World_IsSleepingEnabled( worldId );
	ENSURE( flag == false );

	b3World_EnableContinuous( worldId, false );
	b3World_EnableContinuous( worldId, true );
	flag = b3World_IsContinuousEnabled( worldId );
	ENSURE( flag == true );

	b3World_SetRestitutionThreshold( worldId, B3_FIX( 0.0f ) );
	b3World_SetRestitutionThreshold( worldId, B3_FIX( 2.0f ) );
	b3Fixed value = b3World_GetRestitutionThreshold( worldId );
	ENSURE( value == B3_FIX( 2.0f ) );

	b3World_SetHitEventThreshold( worldId, B3_FIX( 0.0f ) );
	b3World_SetHitEventThreshold( worldId, B3_FIX( 100.0f ) );
	value = b3World_GetHitEventThreshold( worldId );
	ENSURE( value == B3_FIX( 100.0f ) );

	b3World_SetCustomFilterCallback( worldId, CustomFilter, NULL );
	b3World_SetPreSolveCallback( worldId, PreSolveStatic, NULL );

	b3Vec3 g = { B3_FIX( 1.0f ), B3_FIX( 2.0f ) };
	b3World_SetGravity( worldId, g );
	b3Vec3 v = b3World_GetGravity( worldId );
	ENSURE( v.x == g.x );
	ENSURE( v.y == g.y );

	b3ExplosionDef explosionDef = b3DefaultExplosionDef();
	b3World_Explode( worldId, &explosionDef );

	b3World_SetContactTuning( worldId, B3_FIX( 10.0f ), B3_FIX( 2.0f ), B3_FIX( 4.0f ) );

	b3World_SetMaximumLinearSpeed( worldId, B3_FIX( 10.0f ) );
	value = b3World_GetMaximumLinearSpeed( worldId );
	ENSURE( value == B3_FIX( 10.0f ) );

	b3World_EnableWarmStarting( worldId, true );
	flag = b3World_IsWarmStartingEnabled( worldId );
	ENSURE( flag == true );

	int count = b3World_GetAwakeBodyCount( worldId );
	ENSURE( count == 0 );

	b3World_SetUserData( worldId, &value );
	void* userData = b3World_GetUserData( worldId );
	ENSURE( userData == &value );

	b3World_Step( worldId, B3_FIX( 1.0f ), 1 );

	b3DestroyWorld( worldId );

	return 0;
}

static int TestSensor( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Wall from x = 1 to x = 2
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ B3_FIX( 1.5f ), B3_FIX( 11.0f ), B3_FIX( 0.0f ) };
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 10.0f ), B3_FIX( 1.0f ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b3CreateHullShape( wallId, &shapeDef, &box.base );

	// Bullet fired towards the wall
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.gravityScale = B3_FIX( 0.0f );
	bodyDef.position = (b3Pos){ B3_FIX( 7.39814f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ -B3_FIX( 20.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId bulletId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.isSensor = true;
	shapeDef.enableSensorEvents = true;
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.1f ) };
	b3CreateSphereShape( bulletId, &shapeDef, &sphere );

	int beginCount = 0;
	int endCount = 0;

	while ( true )
	{
		b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );

		b3Pos bulletPos = b3Body_GetPosition( bulletId );
		// printf( "Bullet pos: %g %g\n", bulletPos.x, bulletPos.y );

		b3SensorEvents events = b3World_GetSensorEvents( worldId );

		if ( events.beginCount > 0 )
		{
			beginCount += 1;
		}

		if ( events.endCount > 0 )
		{
			endCount += 1;
		}

		if ( bulletPos.x < -B3_FIX( 1.0f ) )
		{
			break;
		}
	}

	b3DestroyWorld( worldId );

	ENSURE( beginCount == 1 );
	ENSURE( endCount == 1 );

	return 0;
}

// Upstream 3fc20f5 narrowed sensor visitors from "not mesh vs mesh" to "must be convex".
// The case that actually moved is a COMPOUND visitor: the old rule let it through, the new
// rule does not. A hull visitor in the same place is the control, so a sensor query that
// silently found nothing could not make the compound half pass by accident.
//
// THIS TEST ONLY BITES IN A VALIDATE BUILD, and that is the whole point of the change. Under
// the old rule a non-convex visitor reached b3MakeShapeProxy, whose switch handles only
// sphere, capsule and hull: in Debug+VALIDATE it hits B3_ASSERT( false ) and traps (measured:
// exit 133, SIGTRAP), while in Release it returns a zeroed proxy and quietly reports nothing,
// which is indistinguishable from the new behaviour. So restoring the old rule fails this test
// in Debug+VALIDATE and PASSES it in Release and in any NDEBUG config, including the two
// RelWithDebInfo CI jobs.
static int TestSensorVisitorMustBeConvex( void )
{
	for ( int useCompound = 0; useCompound < 2; ++useCompound )
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		b3WorldId worldId = b3CreateWorld( &worldDef );

		// A static box at the origin, as a one-hull compound or as the bare hull.
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId visitorBodyId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef visitorDef = b3DefaultShapeDef();
		visitorDef.enableSensorEvents = true;

		b3BoxHull box = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) );
		b3CompoundData* compound = NULL;
		if ( useCompound )
		{
			b3CompoundHullDef hullDef = { .hull = &box.base,
										  .transform = b3Transform_identity,
										  .material = b3DefaultSurfaceMaterial() };
			b3CompoundDef def = { .hulls = &hullDef, .hullCount = 1 };
			compound = b3CreateCompound( &def );
			ENSURE( compound != NULL );
			b3CreateBakedCompoundShape( visitorBodyId, &visitorDef, compound );
		}
		else
		{
			b3CreateHullShape( visitorBodyId, &visitorDef, &box.base );
		}

		// A weightless dynamic sensor sphere sitting inside the box, so the overlap is there
		// from the first step and nothing has to move for it to be found.
		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = B3_FIX( 0.0f );
		b3BodyId sensorId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef sensorDef = b3DefaultShapeDef();
		sensorDef.isSensor = true;
		sensorDef.enableSensorEvents = true;
		b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
		b3CreateSphereShape( sensorId, &sensorDef, &sphere );

		int beginCount = 0;
		b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) );
		for ( int i = 0; i < 4; ++i )
		{
			b3World_Step( worldId, timeStep, 4 );
			beginCount += b3World_GetSensorEvents( worldId ).beginCount;
		}

		if ( useCompound )
		{
			// Non-convex visitor: no events. This is the behaviour change.
			ENSURE( beginCount == 0 );
		}
		else
		{
			// Convex visitor at the same place: the sensor really does see it.
			ENSURE( beginCount == 1 );
		}

		if ( compound != NULL )
		{
			b3DestroyCompound( compound );
		}
		b3DestroyWorld( worldId );
	}

	return 0;
}

static int TestContactEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Static ground
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), -B3_FIX( 0.5f ), B3_FIX( 0.0f ) };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull groundBox = b3MakeBoxHull( B3_FIX( 10.0f ), B3_FIX( 0.5f ), B3_FIX( 10.0f ) );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3ShapeId groundShapeId = b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Dynamic sphere dropped onto the ground; restitution causes it to bounce so we get end events
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 5.0f ), B3_FIX( 0.0f ) };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	shapeDef.enableContactEvents = true;
	shapeDef.baseMaterial.restitution = B3_FIX( 0.6f );
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	int beginCount = 0;
	int endCount = 0;
	bool idsChecked = false;

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 4 );

		b3ContactEvents events = b3World_GetContactEvents( worldId );

		if ( events.beginCount > 0 && idsChecked == false )
		{
			b3ContactBeginTouchEvent be = events.beginEvents[0];
			bool aIsSphere = B3_ID_EQUALS( be.shapeIdA, sphereShapeId );
			bool bIsSphere = B3_ID_EQUALS( be.shapeIdB, sphereShapeId );
			bool aIsGround = B3_ID_EQUALS( be.shapeIdA, groundShapeId );
			bool bIsGround = B3_ID_EQUALS( be.shapeIdB, groundShapeId );
			ENSURE( ( aIsSphere && bIsGround ) || ( aIsGround && bIsSphere ) );
			ENSURE( b3Contact_IsValid( be.contactId ) );
			idsChecked = true;
		}

		beginCount += events.beginCount;
		endCount += events.endCount;
	}

	b3DestroyWorld( worldId );

	ENSURE( idsChecked );
	ENSURE( beginCount >= 1 );
	ENSURE( endCount >= 1 );

	return 0;
}

static int TestHitEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.hitEventThreshold = B3_FIX( 1.0f );
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Static ground
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), -B3_FIX( 0.5f ), B3_FIX( 0.0f ) };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull groundBox = b3MakeBoxHull( B3_FIX( 10.0f ), B3_FIX( 0.5f ), B3_FIX( 10.0f ) );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Sphere driven into the ground fast enough to clear the hit threshold
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.gravityScale = B3_FIX( 0.0f );
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 2.0f ), B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 30.0f ), B3_FIX( 0.0f ) };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	shapeDef.enableHitEvents = true;
	shapeDef.baseMaterial.userMaterialId = 7;
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	int hitCount = 0;
	b3Fixed capturedSpeed = B3_FIX( 0.0f );
	uint64_t capturedMaterialA = 0;
	uint64_t capturedMaterialB = 0;
	b3Vec3 capturedNormal = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 4 );

		b3ContactEvents events = b3World_GetContactEvents( worldId );
		if ( events.hitCount > 0 && hitCount == 0 )
		{
			b3ContactHitEvent hit = events.hitEvents[0];
			capturedSpeed = hit.approachSpeed;
			capturedNormal = hit.normal;
			capturedMaterialA = hit.userMaterialIdA;
			capturedMaterialB = hit.userMaterialIdB;
		}

		hitCount += events.hitCount;
	}

	b3DestroyWorld( worldId );

	ENSURE( hitCount >= 1 );
	ENSURE( capturedSpeed > B3_FIX( 1.0f ) );
	// Head-on vertical impact: normal lies along Y
	ENSURE_SMALL( capturedNormal.x, B3_FIX( 0.01f ) );
	ENSURE_SMALL( capturedNormal.z, B3_FIX( 0.01f ) );
	// One side of the contact carries the sphere's user material
	ENSURE( capturedMaterialA == 7 || capturedMaterialB == 7 );

	return 0;
}

// Hit-event material lookup must respect the compound child that participated in the
// contact. Two children with distinct userMaterialIds at separated positions, dropped
// sphere strikes one specifically. Without the fix, both children would report
// materials[0] and the strike on hull 1 would be misattributed.
static int TestCompoundHitEvents( void )
{
	const uint64_t kHullMaterialA = 11;
	const uint64_t kHullMaterialB = 22;
	const uint64_t kSphereMaterial = 99;
	const b3Fixed kHullCenterX = B3_FIX( 3.0f );

	for ( int side = 0; side < 2; ++side )
	{
		uint64_t expectedHullMaterial = ( side == 0 ) ? kHullMaterialA : kHullMaterialB;
		b3Fixed spawnX = ( side == 0 ) ? -kHullCenterX : kHullCenterX;

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.hitEventThreshold = B3_FIX( 1.0f );
		b3WorldId worldId = b3CreateWorld( &worldDef );

		// Build a compound with two hulls at opposite x positions, distinct userMaterialIds
		b3BoxHull boxA = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) );
		b3BoxHull boxB = b3MakeBoxHull( B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) );

		b3SurfaceMaterial matA = b3DefaultSurfaceMaterial();
		matA.userMaterialId = kHullMaterialA;

		b3SurfaceMaterial matB = b3DefaultSurfaceMaterial();
		matB.userMaterialId = kHullMaterialB;

		b3CompoundHullDef hulls[2];
		hulls[0].hull = &boxA.base;
		hulls[0].transform = (b3Transform){ { -kHullCenterX, B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		hulls[0].material = matA;
		hulls[1].hull = &boxB.base;
		hulls[1].transform = (b3Transform){ { kHullCenterX, B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity };
		hulls[1].material = matB;

		b3CompoundDef compoundDef = { 0 };
		compoundDef.hulls = hulls;
		compoundDef.hullCount = 2;
		b3CompoundData* compound = b3CreateCompound( &compoundDef );
		ENSURE( compound != NULL );

		// Static body holds the compound
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId compoundBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef compoundShapeDef = b3DefaultShapeDef();
		b3CreateBakedCompoundShape( compoundBodyId, &compoundShapeDef, compound );

		// Sphere driven straight down onto the chosen child
		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = B3_FIX( 0.0f );
		bodyDef.position = (b3Pos){ spawnX, B3_FIX( 3.0f ), B3_FIX( 0.0f ) };
		bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 30.0f ), B3_FIX( 0.0f ) };
		b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
		sphereShapeDef.density = B3_FIX( 1.0f );
		sphereShapeDef.enableHitEvents = true;
		sphereShapeDef.baseMaterial.userMaterialId = kSphereMaterial;
		b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
		b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere );

		int hitCount = 0;
		uint64_t capturedMaterialA = 0;
		uint64_t capturedMaterialB = 0;

		for ( int i = 0; i < 30; ++i )
		{
			b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 4 );

			b3ContactEvents events = b3World_GetContactEvents( worldId );
			if ( events.hitCount > 0 && hitCount == 0 )
			{
				b3ContactHitEvent hit = events.hitEvents[0];
				capturedMaterialA = hit.userMaterialIdA;
				capturedMaterialB = hit.userMaterialIdB;
			}
			hitCount += events.hitCount;
		}

		b3DestroyWorld( worldId );
		b3DestroyCompound( compound );

		ENSURE( hitCount >= 1 );
		// Sphere material on one side
		ENSURE( capturedMaterialA == kSphereMaterial || capturedMaterialB == kSphereMaterial );
		// Struck compound child's material on the other side. The pre-fix code returned
		// materials[0] (kHullMaterialA) for both sides, so a strike on the +x child would fail.
		ENSURE( capturedMaterialA == expectedHullMaterial || capturedMaterialB == expectedHullMaterial );
	}

	return 0;
}

enum
{
	kChild0MaterialId = 101,
	kChild1MaterialId = 202,
	kProbeMaterialId = 999,
};

// No context pointer on mixing callbacks, so capture through file scope
static struct
{
	int callCount;
	bool sawChild0;
	bool sawChild1;
	b3Fixed mixedFriction;
} materialCapture;

static b3Fixed CaptureFrictionMix( b3Fixed frictionA, uint64_t userMaterialIdA, b3Fixed frictionB, uint64_t userMaterialIdB )
{
	materialCapture.callCount += 1;

	if ( userMaterialIdA == kChild0MaterialId || userMaterialIdB == kChild0MaterialId )
	{
		materialCapture.sawChild0 = true;
	}

	if ( userMaterialIdA == kChild1MaterialId || userMaterialIdB == kChild1MaterialId )
	{
		materialCapture.sawChild1 = true;
		materialCapture.mixedFriction = b3FixSqrt( b3FixMul( frictionA, frictionB ) );
	}

	return b3FixSqrt( b3FixMul( frictionA, frictionB ) );
}

// Contact material selection must use the struck compound child's material, not entry 0
// of the compound material table. Child 0 gets low friction, child 1 high friction, and a
// unit friction sphere strikes only child 1, so the mixing callback must see child 1's
// values. Covers sphere, capsule, and hull children. Issue #69.
static int TestCompoundContactMaterials( void )
{
	b3SurfaceMaterial mat0 = b3DefaultSurfaceMaterial();
	mat0.friction = B3_FIX( 0.04f );
	mat0.userMaterialId = kChild0MaterialId;

	b3SurfaceMaterial mat1 = b3DefaultSurfaceMaterial();
	mat1.friction = B3_FIX( 0.81f );
	mat1.userMaterialId = kChild1MaterialId;

	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	// Children at x = -3 and x = +3, all with their top face or surface at y = 0.5
	b3CompoundSphereDef spheres[2] = {
		{ .sphere = { { -B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) }, .material = mat0 },
		{ .sphere = { { B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) }, .material = mat1 },
	};

	b3CompoundCapsuleDef capsules[2] = {
		{ .capsule = { { -B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 0.5f ) }, { -B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.5f ) },
					   B3_FIX( 0.5f ) },
		  .material = mat0 },
		{ .capsule = { { B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 0.5f ) }, { B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.5f ) },
					   B3_FIX( 0.5f ) },
		  .material = mat1 },
	};

	b3CompoundHullDef hulls[2] = {
		{ .hull = &box.base,
		  .transform = { { -B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity },
		  .material = mat0 },
		{ .hull = &box.base,
		  .transform = { { B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Quat_identity },
		  .material = mat1 },
	};

	for ( int childType = 0; childType < 3; ++childType )
	{
		b3CompoundDef compoundDef = { 0 };
		if ( childType == 0 )
		{
			compoundDef.spheres = spheres;
			compoundDef.sphereCount = 2;
		}
		else if ( childType == 1 )
		{
			compoundDef.capsules = capsules;
			compoundDef.capsuleCount = 2;
		}
		else
		{
			compoundDef.hulls = hulls;
			compoundDef.hullCount = 2;
		}

		b3CompoundData* compound = b3CreateCompound( &compoundDef );
		ENSURE( compound != NULL );

		memset( &materialCapture, 0, sizeof( materialCapture ) );

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.frictionCallback = CaptureFrictionMix;
		b3WorldId worldId = b3CreateWorld( &worldDef );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId compoundBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef compoundShapeDef = b3DefaultShapeDef();
		b3CreateBakedCompoundShape( compoundBodyId, &compoundShapeDef, compound );

		// Sphere driven straight down onto child 1
		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = B3_FIX( 0.0f );
		bodyDef.position = (b3Pos){ B3_FIX( 3.0f ), B3_FIX( 3.0f ), B3_FIX( 0.0f ) };
		bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 30.0f ), B3_FIX( 0.0f ) };
		b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
		sphereShapeDef.density = B3_FIX( 1.0f );
		sphereShapeDef.baseMaterial.friction = B3_FIX( 1.0f );
		sphereShapeDef.baseMaterial.userMaterialId = kProbeMaterialId;
		b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
		b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere );

		for ( int i = 0; i < 30; ++i )
		{
			b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 4 );
		}

		b3DestroyWorld( worldId );
		b3DestroyCompound( compound );

		ENSURE( materialCapture.callCount > 0 );
		// The pre-fix code fed material table entry 0 to the mixing callback for every child
		ENSURE( materialCapture.sawChild0 == false );
		ENSURE( materialCapture.sawChild1 == true );
		// sqrt(0.81 * 1.0)
		ENSURE_SMALL( materialCapture.mixedFriction - B3_FIX( 0.9f ), 8 * B3_FIXED_EPSILON );
	}

	return 0;
}

static int TestSetWorkerCount( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	CreateJunkyard( worldId );
	StepJunkyard( worldId, 1 );

	b3World_SetWorkerCount( worldId, 4 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 2 );

	b3World_SetWorkerCount( worldId, 4 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 3 );

	b3World_SetWorkerCount( worldId, 0 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 4 );

	b3World_SetWorkerCount( worldId, -5 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 5 );

	b3World_SetWorkerCount( worldId, B3_MAX_WORKERS + 10 );
	ENSURE( b3World_GetWorkerCount( worldId ) == B3_MAX_WORKERS );

	StepJunkyard( worldId, 2 );

	b3DestroyWorld( worldId );

	return 0;
}

// Regression for the impulse-cap wrap fix in contact_solver.c. The friction
// clamp compares a squared impulse length against b3FixMul( maxImpulse,
// maxImpulse ), and those squared forms wrap int64 once an operand reaches
// 2^23.5 ~ 1.19e7 units. A dense cube slammed into high-friction ground is
// physically legal content that crosses that line: the impact normal impulse
// is ~4.6e6 units, so the friction cap (friction 3.0, the Driving sample's
// tire value) is ~1.4e7 - inside [1.19e7, 1.68e7), where the wrapped squared
// cap comes out negative and the clamp fires on EVERY compare. The misfired
// clamp rescales the friction impulse UP to the cap, so "friction" pumped
// energy INTO the slide and the box never stopped sliding. With the gated
// 128-bit cold path the clamp is exact and friction only ever removes energy.
// Runs on both a hull ground (wide convex solver) and a flat mesh ground
// (mesh solver).
static int FrictionCapWrapScene( bool useMeshGround )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3MeshData* meshData = NULL;
	{
		b3BodyDef groundBodyDef = b3DefaultBodyDef();
		b3ShapeDef groundShapeDef = b3DefaultShapeDef();
		groundShapeDef.baseMaterial.friction = B3_FIX( 3.0f );

		if ( useMeshGround )
		{
			b3BodyId groundId = b3CreateBody( worldId, &groundBodyDef );
			meshData = b3CreateWaveMesh( 20, 20, B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) );
			b3CreateMeshShape( groundId, &groundShapeDef, meshData, b3Vec3_one );
		}
		else
		{
			groundBodyDef.position = (b3Pos){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
			b3BodyId groundId = b3CreateBody( worldId, &groundBodyDef );
			b3BoxHull groundBox = b3MakeBoxHull( B3_FIX( 100.0f ), B3_FIX( 1.0f ), B3_FIX( 100.0f ) );
			b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );
		}
	}

	// Start the cube just above the ground so the contact pair exists before
	// the impact (no dependence on continuous collision at these speeds)
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 0.51f ), B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ B3_FIX( 3.0f ), -B3_FIX( 70.0f ), B3_FIX( 1.0f ) };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull box = b3MakeCubeHull( B3_FIX( 0.5f ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	// 1 m^3 at this density puts the impact normal impulse past the cap wrap
	// point when multiplied by friction 3.0 (heavier is not possible: 65536 is
	// the largest mass whose inverse is representable)
	shapeDef.density = B3_FIX( 65536.0f );
	shapeDef.baseMaterial.friction = B3_FIX( 3.0f );
	shapeDef.baseMaterial.rollingResistance = B3_FIX( 1.0f );
	b3CreateHullShape( bodyId, &shapeDef, &box.base );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
	int subStepCount = 4;

	b3Fixed startSpeed = b3Length( bodyDef.linearVelocity );
	b3Fixed maxSpeed = startSpeed;

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
		b3Fixed speed = b3Length( b3Body_GetLinearVelocity( bodyId ) );
		maxSpeed = b3FixMax( maxSpeed, speed );
	}

	b3Fixed finalSpeed = b3Length( b3Body_GetLinearVelocity( bodyId ) );
	b3Fixed finalSpin = b3Length( b3Body_GetAngularVelocity( bodyId ) );
	b3Pos position = b3Body_GetPosition( bodyId );

	b3DestroyWorld( worldId );
	if ( meshData != NULL )
	{
		b3DestroyMesh( meshData );
	}

	// A correct clamp only ever removes energy; the wrapped clamp made the box
	// gain speed from friction
	ENSURE( maxSpeed <= startSpeed + B3_FIX( 1.0f ) );
	ENSURE( finalSpeed < B3_FIX( 1.0f ) );
	ENSURE( finalSpin < B3_FIX( 1.0f ) );
	ENSURE( position.y > -B3_FIX( 1.0f ) && position.y < B3_FIX( 5.0f ) );

	return 0;
}

static int TestFrictionCapWrap( void )
{
	int result = FrictionCapWrapScene( false );
	if ( result != 0 )
	{
		return result;
	}

	return FrictionCapWrapScene( true );
}

// Verifies the b3*_Overflow solver path. The scene puts >B3_DYNAMIC_COLOR_COUNT
// dyn-dyn contacts on a single hub body so several land in the overflow color.
// The new Prepare/Store contactId-pairing asserts in contact_solver.c fire here
// if Store reads constraints from the wrong (base, spans) pair, so the test
// catches the bug even though world state ends up plausible (memory layout is
// stable within a build, so determinism alone is not a witness).
static int TestOverflowColorPile( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	OverflowColorPileData data = CreateOverflowColorPile( worldId );
	(void)data;

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
	int subStepCount = 4;

	// One step would be enough to trip the asserts, but several steps also
	// exercise the warm-start path (Store -> manifold impulse -> Prepare).
	int stepCount = 10;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Confirm the scene actually populated the overflow color. Without this,
	// a future change to graph coloring could silently turn the test into a
	// no-op.
	b3Counters counters = b3World_GetCounters( worldId );
	int overflowContacts = counters.colorCounts[B3_GRAPH_COLOR_COUNT - 1];

	b3DestroyWorld( worldId );

	ENSURE( overflowContacts > 0 );
	return 0;
}

// Exposes that b3Body_EnableSleep mutates body->flags but never syncs
// bodySim->flags / bodyState->flags. When a body created with
// enableSleep=false is later flipped on, body->flags gains b3_enableSleep
// but bodySim/bodyState do not. The flag-sync assertion in
// b3ValidateSolverSets then fires on the next world step.
//
// Under Debug + BOX3D_VALIDATE this crashes; after b3Body_EnableSleep
// calls b3SyncBodyFlags the test runs cleanly.
static int EnableSleepFlagSyncTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	ENSURE( b3Body_IsSleepEnabled( bodyId ) == false );

	b3Body_EnableSleep( bodyId, true );
	ENSURE( b3Body_IsSleepEnabled( bodyId ) == true );

	b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 4 );

	b3DestroyWorld( worldId );
	return 0;
}

// Exposes that b3Body_SetBullet writes only bodySim->flags without
// touching body->flags, while b3SyncBodyFlags overwrites bodySim from
// body->flags. Any subsequent b3Body_SetMotionLocks or b3Body_SetType
// silently wipes (or re-asserts) the bullet bit.
//
// Reproducer A: create with isBullet=false, SetBullet(true), then
// SetMotionLocks. b3Body_IsBullet returns false today (wiped); should be
// true after fix.
//
// Reproducer B: create with isBullet=true, SetBullet(false), then
// SetMotionLocks. b3Body_IsBullet returns true today (re-asserted); should
// be false after fix.
static int SetBulletDriftTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = false;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		ENSURE( b3Body_IsBullet( bodyId ) == false );

		b3Body_SetBullet( bodyId, true );
		ENSURE( b3Body_IsBullet( bodyId ) == true );

		b3MotionLocks locks = { 0 };
		locks.linearX = true;
		b3Body_SetMotionLocks( bodyId, locks );

		ENSURE( b3Body_IsBullet( bodyId ) == true );
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		ENSURE( b3Body_IsBullet( bodyId ) == true );

		b3Body_SetBullet( bodyId, false );
		ENSURE( b3Body_IsBullet( bodyId ) == false );

		b3MotionLocks locks = { 0 };
		locks.linearX = true;
		b3Body_SetMotionLocks( bodyId, locks );

		ENSURE( b3Body_IsBullet( bodyId ) == false );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// Regression: b3Body_EnableSleep used to set world->locked = true before checking
// for a no-op change, then early-return without unlocking. The next mutator call
// would then trip the assert in b3GetUnlockedWorld.
static int EnableSleepNoopUnlockTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = true;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// No-op: enableSleep is already true. Must not leak the world lock.
	b3Body_EnableSleep( bodyId, true );

	// Would assert in b3GetUnlockedWorld if the lock had leaked.
	b3Body_EnableSleep( bodyId, false );
	ENSURE( b3Body_IsSleepEnabled( bodyId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int EnableContactRecyclingTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	// Default is enabled
	b3BodyId bodyA = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == true );

	b3Body_EnableContactRecycling( bodyA, false );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == false );

	b3Body_EnableContactRecycling( bodyA, true );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == true );

	// Per-def opt-out at creation
	bodyDef.enableContactRecycling = false;
	b3BodyId bodyB = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyB ) == false );

	// Stepping after toggling must not trip the flag-sync validator
	b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 4 );

	b3DestroyWorld( worldId );
	return 0;
}

// Identical hull data is shared through a reference counted world database.
static int TestHullDatabase( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyA = b3CreateBody( worldId, &bodyDef );
	b3BodyId bodyB = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// Two shapes built from identical data share one owned copy in the world database.
	b3ShapeId shapeA = b3CreateHullShape( bodyA, &shapeDef, &box.base );
	b3ShapeId shapeB = b3CreateHullShape( bodyB, &shapeDef, &box.base );

	const b3HullData* gotA = b3Shape_GetHull( shapeA );
	const b3HullData* gotB = b3Shape_GetHull( shapeB );

	// Both shapes point at the single shared copy
	ENSURE( gotA == gotB );

	// The shared copy is owned by the world, not the caller's stack hull
	ENSURE( gotA != &box.base );

	// A box built on an independent stack frame must de-duplicate to the same shared copy.
	// This holds only if content hashing sees deterministic padding bytes.
	b3BoxHull box2 = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3BodyId bodyC = b3CreateBody( worldId, &bodyDef );
	b3ShapeId shapeC = b3CreateHullShape( bodyC, &shapeDef, &box2.base );
	ENSURE( b3Shape_GetHull( shapeC ) == gotA );
	b3DestroyShape( shapeC, true );

	// Setting a shape's hull to its own sole shared copy must not free it mid update.
	b3BoxHull box3 = b3MakeBoxHull( B3_FIX( 0.3f ), B3_FIX( 0.3f ), B3_FIX( 0.3f ) );
	b3BodyId bodyD = b3CreateBody( worldId, &bodyDef );
	b3ShapeId shapeD = b3CreateHullShape( bodyD, &shapeDef, &box3.base );
	const b3HullData* gotD = b3Shape_GetHull( shapeD );
	b3Shape_SetHull( shapeD, gotD );
	ENSURE( b3Shape_GetHull( shapeD ) == gotD );
	b3DestroyShape( shapeD, true );

	// Releasing one reference keeps the other alive
	b3DestroyShape( shapeA, true );
	const b3HullData* stillB = b3Shape_GetHull( shapeB );
	ENSURE( stillB == gotB );

	b3DestroyShape( shapeB, true );

	// World destroy asserts the database drained to zero references
	b3DestroyWorld( worldId );
	return 0;
}

typedef struct ExplosionResult
{
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
} ExplosionResult;

// Explode just off the +x side of a centered sphere and capture the impulse it receives.
// The shape, the blast point, and the witness math all run in the body local frame, so the
// result must not depend on how far the body sits from the world origin.
static ExplosionResult RunExplosion( b3Pos base )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = base;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere = { b3Vec3_zero, B3_FIX( 1.0f ) };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// Blast sits 3 units along +x, so the body is pushed back along -x
	b3ExplosionDef explosionDef = b3DefaultExplosionDef();
	explosionDef.position = b3OffsetPos( base, (b3Vec3){ B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	explosionDef.radius = B3_FIX( 5.0f );
	explosionDef.falloff = B3_FIX( 0.0f );
	explosionDef.impulsePerArea = B3_FIX( 10.0f );
	b3World_Explode( worldId, &explosionDef );

	ExplosionResult result;
	result.linearVelocity = b3Body_GetLinearVelocity( bodyId );
	result.angularVelocity = b3Body_GetAngularVelocity( bodyId );

	b3DestroyWorld( worldId );
	return result;
}

static int TestExplosion( void )
{
	ExplosionResult origin = RunExplosion( b3Pos_zero );

	// Pushed away from the blast along -x. A centered sphere has no transverse or angular component.
	ENSURE( origin.linearVelocity.x < -B3_FIX( 1.0e-4f ) );
	ENSURE_SMALL( origin.linearVelocity.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( origin.linearVelocity.z, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( b3Length( origin.angularVelocity ), 8 * B3_FIXED_EPSILON );

	// The same blast far from the origin must produce the same impulse. The world position only
	// reaches b3Fixed in the relative difference, so the result holds where a naive cast would not.
	ExplosionResult far = RunExplosion( (b3Pos){ B3_FIX( 1.0e7f ), B3_FIX( 1.0e7f ), B3_FIX( 1.0e7f ) } );
	ENSURE_SMALL( far.linearVelocity.x - origin.linearVelocity.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( far.linearVelocity.y - origin.linearVelocity.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( far.linearVelocity.z - origin.linearVelocity.z, 8 * B3_FIXED_EPSILON );

	return 0;
}

// Ensure correct move events from bodies involved in CCD.
static int TestContinuousMoveEvent( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableContinuous( worldId, true );

	// Thin static wall, near face at x = 0.1
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull wallBox = b3MakeBoxHull( B3_FIX( 0.1f ), B3_FIX( 5.0f ), B3_FIX( 5.0f ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( wallId, &shapeDef, &wallBox.base );

	// Fast dynamic sphere fired at the wall.
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.gravityScale = B3_FIX( 0.0f );
	bodyDef.position = (b3Pos){ B3_FIX( 3.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	bodyDef.linearVelocity = (b3Vec3){ -B3_FIX( 30.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.25f ) };
	b3CreateSphereShape( ballId, &shapeDef, &sphere );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
	int subStepCount = 4;
	bool haveMove = false;

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, timeStep, subStepCount );

		b3WorldTransform xf = b3Body_GetTransform( ballId );

		b3BodyEvents events = b3World_GetBodyEvents( worldId );
		for ( int i = 0; i < events.moveCount; ++i )
		{
			b3BodyMoveEvent* event = events.moveEvents + i;
			if ( B3_ID_EQUALS( event->bodyId, ballId ) == false )
			{
				continue;
			}

			haveMove = true;

			// The move event must carry the same pose the body reports, CCD rewind included
			ENSURE( event->transform.p.x == xf.p.x );
			ENSURE( event->transform.p.y == xf.p.y );
			ENSURE( event->transform.p.z == xf.p.z );
			ENSURE( event->transform.q.v.x == xf.q.v.x );
			ENSURE( event->transform.q.v.y == xf.q.v.y );
			ENSURE( event->transform.q.v.z == xf.q.v.z );
			ENSURE( event->transform.q.s == xf.q.s );
		}
	}

	ENSURE( haveMove == true );

	// Tunnel check
	b3Pos finalPos = b3Body_GetPosition( ballId );
	ENSURE( B3_FIX( 0.2f ) < finalPos.x && finalPos.x < B3_FIX( 0.8f ) );

	b3DestroyWorld( worldId );

	return 0;
}

int WorldTest( void )
{
	RUN_SUBTEST( HelloWorld );
	RUN_SUBTEST( EmptyWorld );
	RUN_SUBTEST( DestroyAllBodiesWorld );
	RUN_SUBTEST( TestIsValid );
	RUN_SUBTEST( TestWorldRecycle );
	RUN_SUBTEST( TestWorldCoverage );
	RUN_SUBTEST( TestExplosion );
	RUN_SUBTEST( TestSensor );
	RUN_SUBTEST( TestSensorVisitorMustBeConvex );
	RUN_SUBTEST( TestContinuousMoveEvent );
	RUN_SUBTEST( TestContactEvents );
	RUN_SUBTEST( TestHitEvents );
	RUN_SUBTEST( TestCompoundHitEvents );
	RUN_SUBTEST( TestCompoundContactMaterials );
	RUN_SUBTEST( TestFrictionCapWrap );
	RUN_SUBTEST( TestOverflowColorPile );
	RUN_SUBTEST( SetBulletDriftTest );
	RUN_SUBTEST( EnableSleepFlagSyncTest );
	RUN_SUBTEST( EnableSleepNoopUnlockTest );
	RUN_SUBTEST( EnableContactRecyclingTest );
	RUN_SUBTEST( TestSetWorkerCount );
	RUN_SUBTEST( TestHullDatabase );

	return 0;
}
