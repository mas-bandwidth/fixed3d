// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// From box3d 79fc559 (the broad-phase half of "Optimizations (#168)"), carried with the f555ee4
// broad-phase port: the static rebuild fix it tests was written against the f555ee4 tree.

#include "body.h"
#include "broad_phase.h"
#include "physics_world.h"
#include "shape.h"
#include "table.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include <stdio.h>

// Brute force check of the broad-phase pair set against every live shape pair. Must run right
// after a zero time step so the pair update and the disjoint contact purge have both happened
// against the current fat boxes.
static int CheckPairSet( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	b3BroadPhase* bp = &world->broadPhase;
	const b3Shape* shapes = world->shapes.data;
	const b3AABB* fatAABBs = world->fatAABBs.data;
	int shapeCount = world->shapes.count;

	for ( int i = 0; i < shapeCount; ++i )
	{
		const b3Shape* shapeA = shapes + i;
		if ( shapeA->id == B3_NULL_INDEX || shapeA->proxyKey == B3_NULL_INDEX || shapeA->sensorIndex != B3_NULL_INDEX )
		{
			continue;
		}

		b3Body* bodyA = b3Array_Get( world->bodies, shapeA->bodyId );

		for ( int j = i + 1; j < shapeCount; ++j )
		{
			const b3Shape* shapeB = shapes + j;
			if ( shapeB->id == B3_NULL_INDEX || shapeB->proxyKey == B3_NULL_INDEX || shapeB->sensorIndex != B3_NULL_INDEX )
			{
				continue;
			}

			if ( shapeA->bodyId == shapeB->bodyId )
			{
				continue;
			}

			b3Body* bodyB = b3Array_Get( world->bodies, shapeB->bodyId );

			bool expected = b3ShouldShapesCollide( shapeA->filter, shapeB->filter ) &&
							b3ShouldBodiesCollide( world, bodyA, bodyB ) && b3AABB_Overlaps( fatAABBs[i], fatAABBs[j] );

			bool actual = b3ContainsKey( &bp->pairSet, b3ShapePairKey( i, j, 0 ) );

			// A sleeping contact is never refreshed, so a stale pair can outlive the overlap when
			// the other body is teleported, and a set woken by the narrow phase carries its stale
			// contacts into the awake set one collide late. With sleeping on only a missing pair
			// is a bug.
			if ( world->enableSleep && actual && expected == false )
			{
				continue;
			}

			if ( expected != actual )
			{
				printf( "  pair mismatch: shapes %d (body %d type %d set %d) and %d (body %d type %d set %d), expected %d, "
						"actual %d\n",
						i, shapeA->bodyId, bodyA->type, bodyA->setIndex, j, shapeB->bodyId, bodyB->type, bodyB->setIndex,
						expected, actual );
				fflush( stdout );
				return 1;
			}
		}
	}

	return 0;
}

typedef struct RandomState
{
	uint32_t seed;
} RandomState;

static uint32_t NextRandom( RandomState* state )
{
	// xorshift32
	uint32_t x = state->seed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state->seed = x;
	return x;
}

// A uniform draw on integers: 24 random bits scale the span, so the swarm is the same on every
// compiler and architecture.
static b3Fixed RandomFixed( RandomState* state, b3Fixed lower, b3Fixed upper )
{
	int64_t u = (int64_t)( NextRandom( state ) & 0xFFFFFF );
	return lower + (b3Fixed)( ( ( upper - lower ) * u ) >> 24 );
}

static int RandomInt( RandomState* state, int count )
{
	return (int)( NextRandom( state ) % (uint32_t)count );
}

// Upstream runs three seeds for 400 steps. That sweep takes about 30 s in a fixed3d Debug build,
// half the one minute CI budget, so the suite runs one seed for 120 steps in each of the three
// worker and sleep configurations. Define BROAD_PHASE_STRESS_FULL for the upstream sweep.
#if defined( BROAD_PHASE_STRESS_FULL )
#define BROAD_PHASE_STRESS_SEEDS 3
#define BROAD_PHASE_STRESS_STEPS 400
#else
#define BROAD_PHASE_STRESS_SEEDS 1
#define BROAD_PHASE_STRESS_STEPS 120
#endif

#define MAX_BODIES 64
#define STATIC_COUNT 60
#define TILE_COUNT 9

typedef struct BodySlot
{
	b3BodyId bodyId;
	bool active;
} BodySlot;

typedef struct StressContext
{
	b3WorldId worldId;
	RandomState random;
	b3Pos origin;
	b3Fixed extent;
	BodySlot slots[MAX_BODIES];
	b3JointId joints[4];
	int jointCount;
	b3BoxHull box;
} StressContext;

static b3Pos RandomPosition( StressContext* context )
{
	b3Fixed e = context->extent;
	b3Vec3 offset = { RandomFixed( &context->random, -e, e ), RandomFixed( &context->random, -e, e ),
					  RandomFixed( &context->random, -e, e ) };
	return b3OffsetPos( context->origin, offset );
}

static b3Vec3 RandomVelocity( StressContext* context )
{
	// Some bodies outrun their margin every step and go through continuous collision
	b3Fixed s = RandomInt( &context->random, 4 ) == 0 ? B3_FIX( 40.0f ) : B3_FIX( 12.0f );
	return (b3Vec3){ RandomFixed( &context->random, -s, s ), RandomFixed( &context->random, -s, s ),
					 RandomFixed( &context->random, -s, s ) };
}

static void CreateSlotBody( StressContext* context, int slot )
{
	RandomState* random = &context->random;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = RandomInt( random, 10 ) == 0 ? b3_kinematicBody : b3_dynamicBody;
	bodyDef.position = RandomPosition( context );
	bodyDef.linearVelocity = RandomVelocity( context );
	bodyDef.isBullet = RandomInt( random, 3 ) == 0;
	bodyDef.gravityScale = B3_FIX( 0.0f );
	b3BodyId bodyId = b3CreateBody( context->worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = 1ull << RandomInt( random, 3 );
	shapeDef.filter.maskBits = RandomInt( random, 4 ) == 0 ? 0x5ull : B3_DEFAULT_MASK_BITS;

	int shapeCount = 1 + RandomInt( random, 2 );
	for ( int i = 0; i < shapeCount; ++i )
	{
		if ( RandomInt( random, 2 ) == 0 )
		{
			b3Sphere sphere = { { RandomFixed( random, B3_FIX( -1.0f ), B3_FIX( 1.0f ) ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
								RandomFixed( random, B3_FIX( 0.2f ), B3_FIX( 1.5f ) ) };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
		}
		else
		{
			b3CreateHullShape( bodyId, &shapeDef, &context->box.base );
		}
	}

	context->slots[slot].bodyId = bodyId;
	context->slots[slot].active = true;
}

// A swarm of bodies churning through every proxy mutation the world API offers, with the pair
// set checked against a brute force scan after every step.
static int RunBroadPhaseStress( int workerCount, uint32_t seed, bool enableSleep )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	worldDef.enableSleep = enableSleep;
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	StressContext context = { 0 };
	context.worldId = worldId;
	context.random.seed = seed;
	context.origin = b3ToPos( (b3Vec3){ b3FixFromInt( -1900 ), B3_FIX( 0.0f ), b3FixFromInt( 1330 ) } );
	context.extent = B3_FIX( 12.0f );
	context.box = b3MakeBoxHull( B3_FIX( 0.95f ), B3_FIX( 0.6f ), B3_FIX( 2.2f ) );

	// Kilometre tiles far from the origin under the swarm, like a streamed terrain
	b3HeightFieldData* tile = b3CreateGrid( 65, 65, (b3Vec3){ B3_FIX( 16.0f ), B3_FIX( 1.0f ), B3_FIX( 16.0f ) }, false );
	for ( int i = 0; i < TILE_COUNT; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		bodyDef.position = b3ToPos( (b3Vec3){ b3FixFromInt( -3072 + 1024 * ( i % 3 ) ), b3FixFromInt( -30 ), b3FixFromInt( 1024 * ( i / 3 ) ) } );
		b3BodyId tileId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.enableSpeculativeContact = false;
		b3CreateHeightFieldShape( tileId, &shapeDef, tile );
	}

	// Static clutter inside the swarm. Some pieces ask for contacts on creation.
	b3BodyId staticIds[STATIC_COUNT];
	b3HeightFieldData* grid = b3CreateGrid( 5, 5, (b3Vec3){ B3_FIX( 2.0f ), B3_FIX( 1.0f ), B3_FIX( 2.0f ) }, false );
	b3BoxHull staticBox = b3MakeBoxHull( B3_FIX( 3.0f ), B3_FIX( 0.5f ), B3_FIX( 3.0f ) );
	for ( int i = 0; i < STATIC_COUNT; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		bodyDef.position = RandomPosition( &context );
		staticIds[i] = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.invokeContactCreation = ( i % 4 ) == 0;
		shapeDef.enableSpeculativeContact = ( i % 2 ) == 0;

		if ( i % 3 == 0 )
		{
			b3CreateHeightFieldShape( staticIds[i], &shapeDef, grid );
		}
		else
		{
			b3CreateHullShape( staticIds[i], &shapeDef, &staticBox.base );
		}
	}

	for ( int i = 0; i < MAX_BODIES; ++i )
	{
		if ( RandomInt( &context.random, 4 ) != 0 )
		{
			CreateSlotBody( &context, i );
		}
	}

	b3Fixed timeStep = b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) );
	int stepCount = BROAD_PHASE_STRESS_STEPS;

	for ( int step = 0; step < stepCount; ++step )
	{
		RandomState* random = &context.random;

		int mutationCount = RandomInt( random, 4 );
		for ( int m = 0; m < mutationCount; ++m )
		{
			int slot = RandomInt( random, MAX_BODIES );
			BodySlot* body = context.slots + slot;
			int action = RandomInt( random, 12 );

			if ( body->active == false )
			{
				if ( action < 6 )
				{
					CreateSlotBody( &context, slot );
				}
				continue;
			}

			switch ( action )
			{
				case 0:
					b3DestroyBody( body->bodyId );
					body->active = false;
					break;

				case 1:
				case 2:
					b3Body_SetTransform( body->bodyId, RandomPosition( &context ), b3Quat_identity );
					break;

				case 3:
					b3Body_SetLinearVelocity( body->bodyId, RandomVelocity( &context ) );
					break;

				case 4:
					b3Body_Disable( body->bodyId );
					break;

				case 5:
					b3Body_Enable( body->bodyId );
					break;

				case 6:
					b3Body_SetBullet( body->bodyId, RandomInt( random, 2 ) == 0 );
					break;

				case 7:
				{
					b3BodyType types[3] = { b3_staticBody, b3_kinematicBody, b3_dynamicBody };
					b3Body_SetType( body->bodyId, types[RandomInt( random, 3 )] );
					if ( b3Body_GetType( body->bodyId ) != b3_staticBody )
					{
						b3Body_SetLinearVelocity( body->bodyId, RandomVelocity( &context ) );
					}
				}
				break;

				case 8:
				{
					// Add a joint that blocks collision, or toggle one
					if ( context.jointCount < 4 )
					{
						int otherSlot = RandomInt( random, MAX_BODIES );
						BodySlot* other = context.slots + otherSlot;
						if ( otherSlot != slot && other->active && b3Body_GetType( body->bodyId ) == b3_dynamicBody &&
							 b3Body_GetType( other->bodyId ) == b3_dynamicBody )
						{
							b3WeldJointDef jointDef = b3DefaultWeldJointDef();
							jointDef.base.bodyIdA = body->bodyId;
							jointDef.base.bodyIdB = other->bodyId;
							jointDef.base.collideConnected = false;
							context.joints[context.jointCount++] = b3CreateWeldJoint( worldId, &jointDef );
						}
					}
					else
					{
						b3JointId jointId = context.joints[RandomInt( random, context.jointCount )];
						if ( b3Joint_IsValid( jointId ) )
						{
							b3Joint_SetCollideConnected( jointId, RandomInt( random, 2 ) == 0 );
						}
					}
				}
				break;

				case 9:
				{
					int staticIndex = RandomInt( random, STATIC_COUNT );
					b3Body_SetTransform( staticIds[staticIndex], RandomPosition( &context ), b3Quat_identity );
				}
				break;

				case 10:
					b3World_RebuildStaticTree( worldId );
					break;

				default:
					break;
			}
		}

		// Joints whose bodies were destroyed are gone too
		for ( int j = 0; j < context.jointCount; )
		{
			if ( b3Joint_IsValid( context.joints[j] ) == false )
			{
				context.joints[j] = context.joints[context.jointCount - 1];
				context.jointCount -= 1;
			}
			else
			{
				j += 1;
			}
		}

		// Keep the swarm inside the clutter
		for ( int i = 0; i < MAX_BODIES; ++i )
		{
			BodySlot* body = context.slots + i;
			if ( body->active == false || b3Body_IsEnabled( body->bodyId ) == false )
			{
				continue;
			}

			b3Pos p = b3Body_GetPosition( body->bodyId );
			b3Vec3 d = b3SubPos( p, context.origin );
			b3Fixed limit = 2 * context.extent;
			if ( b3FixAbs( d.x ) > limit || b3FixAbs( d.y ) > limit || b3FixAbs( d.z ) > limit )
			{
				b3Body_SetTransform( body->bodyId, RandomPosition( &context ), b3Quat_identity );
				b3Body_SetLinearVelocity( body->bodyId, RandomVelocity( &context ) );
			}
		}

		b3World_Step( worldId, timeStep, 4 );

		// A zero step finds pairs and purges disjoint contacts without moving anything
		b3World_Step( worldId, B3_FIX( 0.0f ), 1 );

		if ( CheckPairSet( worldId ) != 0 )
		{
			printf( "  workers=%d seed=%u sleep=%d step=%d\n", workerCount, seed, enableSleep, step );
			fflush( stdout );
			b3DestroyWorld( worldId );
			b3DestroyHeightField( grid );
			b3DestroyHeightField( tile );
			return 1;
		}
	}

	b3DestroyWorld( worldId );
	b3DestroyHeightField( grid );
	b3DestroyHeightField( tile );
	return 0;
}

static int BroadPhaseStressTest( void )
{
	uint32_t seeds[3] = { 12345u, 0xC0FFEEu, 777u };
	for ( int i = 0; i < BROAD_PHASE_STRESS_SEEDS; ++i )
	{
		ENSURE( RunBroadPhaseStress( 1, seeds[i], false ) == 0 );
		ENSURE( RunBroadPhaseStress( 4, seeds[i], false ) == 0 );
		ENSURE( RunBroadPhaseStress( 3, seeds[i], true ) == 0 );
	}

	return 0;
}

// The broad-phase keeps its pending pair marks inside the static tree, so a full rebuild between
// a static mutation and the next step used to wipe them and the contact never appeared. Covers
// both ways a static proxy gets marked: a teleport and a shape created with invokeContactCreation.
static int RunStaticRebuildKeepsMarks( bool useInvoke )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = b3ToPos( (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
	b3BodyId sphereId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };
	b3CreateSphereShape( sphereId, &shapeDef, &sphere );

	bodyDef.type = b3_staticBody;
	bodyDef.position = b3ToPos( (b3Vec3){ B3_FIX( 100.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) } );
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 2.0f ), B3_FIX( 0.5f ), B3_FIX( 2.0f ) );
	if ( useInvoke == false )
	{
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	// Settle so the resting sphere carries no mark of its own
	for ( int i = 0; i < 3; ++i )
	{
		b3World_Step( worldId, b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) ), 4 );
	}

	b3Body_SetTransform( groundId, b3Pos_zero, b3Quat_identity );
	if ( useInvoke )
	{
		b3ShapeDef invokeDef = b3DefaultShapeDef();
		invokeDef.invokeContactCreation = true;
		b3CreateHullShape( groundId, &invokeDef, &box.base );
	}

	b3World_RebuildStaticTree( worldId );
	b3World_Step( worldId, b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) ), 4 );

	ENSURE( b3Body_GetContactCapacity( sphereId ) == 1 );

	b3DestroyWorld( worldId );
	return 0;
}

static int StaticRebuildKeepsMarks( void )
{
	ENSURE( RunStaticRebuildKeepsMarks( false ) == 0 );
	ENSURE( RunStaticRebuildKeepsMarks( true ) == 0 );
	return 0;
}

static int FlushingAssert( const char* condition, const char* fileName, int lineNumber )
{
	printf( "BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stdout );
	return 1;
}

int BroadPhaseTest( void )
{
	// A crash through a pipe loses buffered output, so flush the assertion first
	b3SetAssertFcn( FlushingAssert );

	RUN_SUBTEST( StaticRebuildKeepsMarks );
	RUN_SUBTEST( BroadPhaseStressTest );
	return 0;
}
