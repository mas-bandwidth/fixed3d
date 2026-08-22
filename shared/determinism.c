// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"

#include "human.h"
#include "utils.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stdlib.h>

#define GRID_SIZE B3_FIX( 15.0f )

static void CreateGroup( FallingRagdollData* data, b3WorldId worldId, int rowIndex, int columnIndex )
{
	assert( rowIndex < RAGDOLL_GRID_COUNT && columnIndex < RAGDOLL_GRID_COUNT );

	int groupIndex = rowIndex * RAGDOLL_GRID_COUNT + columnIndex;

	b3Fixed span = RAGDOLL_GRID_COUNT * GRID_SIZE;
	b3Fixed groupDistance = span / RAGDOLL_GRID_COUNT;

	// local offset from the scene origin (b3OffsetPos adds it onto GetSceneOrigin)
	b3Vec3 position;
	position.x = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( groupDistance , ( b3FixFromInt( columnIndex ) + B3_FIX( 0.5f ) ) );
	position.y = B3_FIX( 15.0f );
	position.z = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( groupDistance , ( b3FixFromInt( rowIndex ) + B3_FIX( 0.5f ) ) );

	b3Fixed frictionTorque = B3_FIX( 5.0f );
	b3Fixed hertz = B3_FIX( 1.0f );
	b3Fixed dampingRatio = B3_FIX( 0.7f );
	bool colorize = false;

	for ( int i = 0; i < RAGDOLL_GROUP_SIZE; ++i )
	{
		Human* human = data->groups[groupIndex].humans + i;
		CreateHuman( human, worldId, b3OffsetPos( GetSceneOrigin(), position ), frictionTorque, hertz, dampingRatio, groupIndex,
					 NULL, colorize );
		position.x += B3_FIX( 0.75f );
	}
}

FallingRagdollData CreateFallingRagdolls( b3WorldId worldId )
{
	FallingRagdollData data = { 0 };

	int halfMeshGridRows = 4;
	b3Fixed meshGridCellWidth = b3FixDiv( GRID_SIZE , ( b3FixMul( B3_FIX( 2.0f ) , b3FixFromInt( halfMeshGridRows ) ) ) );
	data.gridMesh = b3CreateGridMesh( 2 * halfMeshGridRows, 2 * halfMeshGridRows, meshGridCellWidth, 0, true );
	data.torusMesh = b3CreateTorusMesh( 16, 16, b3FixMul( B3_FIX( 0.25f ) , GRID_SIZE ), B3_FIX( 1.0f ) );

	b3Fixed span = GRID_SIZE * RAGDOLL_GRID_COUNT;
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	b3Vec3 local = b3Vec3_zero;
	local.x = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( B3_FIX( 0.5f ) , GRID_SIZE );
	for ( int i = 0; i < RAGDOLL_GRID_COUNT; ++i )
	{
		local.z = b3FixMul( -B3_FIX( 0.5f ) , span ) + b3FixMul( B3_FIX( 0.5f ) , GRID_SIZE );
		for ( int j = 0; j < RAGDOLL_GRID_COUNT; ++j )
		{
			bodyDef.position = b3OffsetPos( GetSceneOrigin(), local );
			b3BodyId body = b3CreateBody( worldId, &bodyDef );
			b3CreateMeshShape( body, &shapeDef, data.gridMesh, b3Vec3_one );
			b3CreateMeshShape( body, &shapeDef, data.torusMesh, b3Vec3_one );

			CreateGroup( &data, worldId, i, j );

			local.z += GRID_SIZE;
		}

		local.x += GRID_SIZE;
	}

	return data;
}

bool UpdateFallingRagdolls( b3WorldId worldId, FallingRagdollData* data )
{
	if ( data->hash == 0 )
	{
		b3BodyEvents bodyEvents = b3World_GetBodyEvents( worldId );

		if ( bodyEvents.moveCount == 0 )
		{
			int awakeCount = b3World_GetAwakeBodyCount( worldId );
			assert( awakeCount == 0 );
			// assert compiles away under NDEBUG, and clang-cl's does not even name its argument,
			// so the variable reads as unused in an optimized build with warnings as errors.
			(void)awakeCount;

			data->hash = B3_HASH_INIT;
			for ( int i = 0; i < RAGDOLL_GRID_COUNT; ++i )
			{
				for ( int j = 0; j < RAGDOLL_GRID_COUNT; ++j )
				{
					for ( int k = 0; k < RAGDOLL_GROUP_SIZE; ++k )
					{
						int groupIndex = i * RAGDOLL_GRID_COUNT + j;

						Human* human = data->groups[groupIndex].humans + k;

						for ( int b = 0; b < bone_count; ++b )
						{
							b3BodyId bodyId = human->bones[b].bodyId;
							b3WorldTransform xf = b3Body_GetTransform( bodyId );
							data->hash = b3Hash( data->hash, (uint8_t*)( &xf ), sizeof( b3WorldTransform ) );
						}
					}
				}
			}

			data->sleepStep = data->stepCount;
		}
	}

	data->stepCount += 1;

	return data->hash != 0;
}

void DestroyFallingRagdolls( FallingRagdollData* data )
{
	b3DestroyMesh( data->gridMesh );
	b3DestroyMesh( data->torusMesh );

	data->gridMesh = NULL;
	data->torusMesh = NULL;
}

#define WAVE_PILE_GRID 5
#define WAVE_PILE_LAYERS 4

WavePileData CreateWavePile( b3WorldId worldId )
{
	WavePileData data = { 0 };

	g_randomSeed = 52977;

	// Height fields grow from a corner, offset the body to center the patch.
	int fieldCount = 21;
	b3Vec3 fieldScale = { B3_FIX( 1.0f ), B3_FIX( 0.6f ), B3_FIX( 1.0f ) };
	data.heightField = b3CreateWave( fieldCount, fieldCount, fieldScale, B3_FIX( 0.08f ), B3_FIX( 0.06f ), false );

	{
		b3Fixed extent = b3FixMul( fieldScale.x , b3FixFromInt( fieldCount - 1 ) );
		b3Vec3 local = b3Vec3_zero;
		local.x = b3FixMul( -B3_FIX( 0.5f ) , extent );
		local.z = b3FixMul( -B3_FIX( 0.5f ) , extent );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), local );
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHeightFieldShape( groundId, &shapeDef, data.heightField );
	}

	b3HullData* rock = b3CreateRock( B3_FIX( 0.55f ) );
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.45f ), B3_FIX( 0.3f ), B3_FIX( 0.55f ) );
	b3Sphere sphere = { b3Vec3_zero, B3_FIX( 0.5f ) };
	b3Capsule capsule = { { B3_FIX( 0.0f ), -B3_FIX( 0.3f ), B3_FIX( 0.0f ) },
						  { B3_FIX( 0.0f ), B3_FIX( 0.3f ), B3_FIX( 0.0f ) },
						  B3_FIX( 0.35f ) };

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	// Rolling resistance so the pile sleeps quickly.
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.3f );

	b3Fixed spacing = B3_FIX( 1.7f );
	int index = 0;
	for ( int layer = 0; layer < WAVE_PILE_LAYERS; ++layer )
	{
		for ( int i = 0; i < WAVE_PILE_GRID; ++i )
		{
			for ( int j = 0; j < WAVE_PILE_GRID; ++j )
			{
				b3Vec3 jitter = RandomVec3Uniform( -B3_FIX( 0.3f ), B3_FIX( 0.3f ) );
				b3Vec3 local;
				local.x = b3FixMul( spacing , ( b3FixFromInt( i ) - b3FixMul( B3_FIX( 0.5f ) , b3FixFromInt( WAVE_PILE_GRID - 1 ) ) ) ) + jitter.x;
				local.y = B3_FIX( 2.5f ) + b3FixMul( B3_FIX( 1.6f ) , b3FixFromInt( layer ) ) + b3FixMul( B3_FIX( 0.3f ) , jitter.y );
				local.z = b3FixMul( spacing , ( b3FixFromInt( j ) - b3FixMul( B3_FIX( 0.5f ) , b3FixFromInt( WAVE_PILE_GRID - 1 ) ) ) ) + jitter.z;
				bodyDef.position = b3OffsetPos( GetSceneOrigin(), local );
				bodyDef.rotation = RandomQuat();

				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
				data.bodies[index] = bodyId;

				switch ( index % 4 )
				{
					case 0:
						b3CreateSphereShape( bodyId, &shapeDef, &sphere );
						break;
					case 1:
						b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
						break;
					case 2:
						b3CreateHullShape( bodyId, &shapeDef, &box.base );
						break;
					default:
						b3CreateHullShape( bodyId, &shapeDef, rock );
						break;
				}

				index += 1;
			}
		}
	}

	assert( index == WAVE_PILE_BODY_COUNT );

	// The world keeps its own copy.
	b3DestroyHull( rock );

	return data;
}

bool UpdateWavePile( b3WorldId worldId, WavePileData* data )
{
	if ( data->hash == 0 )
	{
		if ( b3World_GetAwakeBodyCount( worldId ) == 0 )
		{
			data->hash = B3_HASH_INIT;
			for ( int i = 0; i < WAVE_PILE_BODY_COUNT; ++i )
			{
				b3WorldTransform xf = b3Body_GetTransform( data->bodies[i] );
				data->hash = b3Hash( data->hash, (uint8_t*)&xf, sizeof( b3WorldTransform ) );
			}

			data->sleepStep = data->stepCount;
		}
	}

	data->stepCount += 1;

	return data->hash != 0;
}

void DestroyWavePile( WavePileData* data )
{
	b3DestroyHeightField( data->heightField );
	data->heightField = NULL;
}

typedef struct QuerySpawnOverlapContext
{
	QuerySpawnData* data;
	int count;
} QuerySpawnOverlapContext;

static bool QuerySpawnOverlapCallback( b3ShapeId shapeId, void* context )
{
	QuerySpawnOverlapContext* overlap = context;
	overlap->count += 1;
	overlap->data->queryHitCount += 1;
	overlap->data->queryHash = b3Hash( overlap->data->queryHash, (uint8_t*)&shapeId.index1, sizeof( shapeId.index1 ) );
	return true;
}

static b3Fixed QuerySpawnCastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, b3Fixed fraction, uint64_t userMaterialId,
									   int triangleIndex, int childIndex, void* context )
{
	(void)shapeId, (void)point, (void)normal, (void)userMaterialId, (void)triangleIndex, (void)childIndex;
	b3Fixed* closest = context;
	*closest = fraction;
	return fraction;
}

static void QuerySpawnOnce( b3WorldId worldId, QuerySpawnData* data )
{
	b3QueryFilter filter = b3DefaultQueryFilter();

	// Ray result decides the spawn position, a miss seeds the cloud near the scene origin.
	b3Vec3 rayLocal = RandomVec3( (b3Vec3){ -B3_FIX( 12.0f ), -B3_FIX( 12.0f ), -B3_FIX( 12.0f ) },
								  (b3Vec3){ B3_FIX( 12.0f ), B3_FIX( 12.0f ), B3_FIX( 12.0f ) } );
	b3Pos rayOrigin = b3OffsetPos( GetSceneOrigin(), rayLocal );
	b3Vec3 rayTranslation = b3MulSV( B3_FIX( 30.0f ), RandomUnitVector() );

	b3Pos spawnPosition;
	b3RayResult ray = b3World_CastRayClosest( worldId, rayOrigin, rayTranslation, filter );
	if ( ray.hit )
	{
		data->queryHitCount += 1;
		data->queryHash = b3Hash( data->queryHash, (uint8_t*)&ray.fraction, sizeof( ray.fraction ) );
		data->queryHash = b3Hash( data->queryHash, (uint8_t*)&ray.normal, sizeof( ray.normal ) );
		spawnPosition = b3OffsetPos( ray.point, b3MulSV( B3_FIX( 1.2f ), ray.normal ) );
	}
	else
	{
		b3Vec3 spawnLocal = RandomVec3( (b3Vec3){ -B3_FIX( 6.0f ), -B3_FIX( 6.0f ), -B3_FIX( 6.0f ) },
										(b3Vec3){ B3_FIX( 6.0f ), B3_FIX( 6.0f ), B3_FIX( 6.0f ) } );
		spawnPosition = b3OffsetPos( GetSceneOrigin(), spawnLocal );
	}

	data->rayOrigin = rayOrigin;
	data->rayTranslation = rayTranslation;
	data->rayDidHit = ray.hit;
	data->rayPoint = ray.hit ? ray.point : b3OffsetPos( rayOrigin, rayTranslation );
	data->rayNormal = ray.hit ? ray.normal : b3Vec3_zero;

	// Overlap count picks the shape type. Bounds go through b3PosToBound so this
	// compiles in the ludicrous build where b3AABB carries wide bounds.
	b3Vec3 center = RandomVec3Uniform( -B3_FIX( 10.0f ), B3_FIX( 10.0f ) );
	b3Fixed extent = RandomFloatRange( B3_FIX( 1.0f ), B3_FIX( 4.0f ) );
	b3AABB aabb;
	aabb.lowerBound =
		b3PosToBound( b3OffsetPos( GetSceneOrigin(), (b3Vec3){ center.x - extent, center.y - extent, center.z - extent } ) );
	aabb.upperBound =
		b3PosToBound( b3OffsetPos( GetSceneOrigin(), (b3Vec3){ center.x + extent, center.y + extent, center.z + extent } ) );

	QuerySpawnOverlapContext overlap = { data, 0 };
	b3World_OverlapAABB( worldId, aabb, filter, QuerySpawnOverlapCallback, &overlap );

	data->overlapBounds = aabb;
	data->overlapCount = overlap.count;

	// Sphere cast fraction sets the spawn size.
	b3Fixed fraction = B3_FIX( 1.0f );
	b3Vec3 proxyPoint = b3Vec3_zero;
	b3ShapeProxy proxy = { &proxyPoint, 1, QUERY_SPAWN_CAST_RADIUS };
	b3World_CastShape( worldId, rayOrigin, &proxy, rayTranslation, filter, QuerySpawnCastCallback, &fraction );
	if ( fraction < B3_FIX( 1.0f ) )
	{
		data->queryHitCount += 1;
		data->queryHash = b3Hash( data->queryHash, (uint8_t*)&fraction, sizeof( fraction ) );
	}

	data->castFraction = fraction;

	b3Fixed size = B3_FIX( 0.3f ) + b3FixMul( B3_FIX( 0.2f ) , fraction );

	// Damping guarantees everything comes to rest.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = spawnPosition;
	bodyDef.rotation = RandomQuat();
	bodyDef.linearVelocity = RandomVec3Uniform( -B3_FIX( 0.2f ), B3_FIX( 0.2f ) );
	bodyDef.angularVelocity = RandomVec3Uniform( -B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	bodyDef.linearDamping = B3_FIX( 1.0f );
	bodyDef.angularDamping = B3_FIX( 1.0f );

	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.2f );

	switch ( ( data->spawnCount + overlap.count ) % 3 )
	{
		case 0:
		{
			b3Sphere sphere = { b3Vec3_zero, size };
			b3CreateSphereShape( bodyId, &shapeDef, &sphere );
			break;
		}
		case 1:
		{
			b3Capsule capsule = { { B3_FIX( 0.0f ), -size, B3_FIX( 0.0f ) },
								  { B3_FIX( 0.0f ), size, B3_FIX( 0.0f ) },
								  b3FixMul( B3_FIX( 0.7f ) , size ) };
			b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			break;
		}
		default:
		{
			b3BoxHull box = b3MakeBoxHull( size, b3FixMul( B3_FIX( 0.7f ) , size ), b3FixMul( B3_FIX( 0.5f ) , size ) );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
			break;
		}
	}

	data->bodies[data->spawnCount] = bodyId;
	data->spawnCount += 1;
	data->lastSpawnPosition = spawnPosition;
}

QuerySpawnData CreateQuerySpawn( b3WorldId worldId )
{
	QuerySpawnData data = { 0 };

	g_randomSeed = 71689;

	// Empty space, motion comes only from spawn velocities and overlap pushes.
	b3World_SetGravity( worldId, b3Vec3_zero );

	return data;
}

bool UpdateQuerySpawn( b3WorldId worldId, QuerySpawnData* data )
{
	if ( data->spawnCount < QUERY_SPAWN_COUNT )
	{
		QuerySpawnOnce( worldId, data );
	}
	else if ( data->hash == 0 && b3World_GetAwakeBodyCount( worldId ) == 0 )
	{
		data->hash = B3_HASH_INIT;
		for ( int i = 0; i < QUERY_SPAWN_COUNT; ++i )
		{
			b3WorldTransform xf = b3Body_GetTransform( data->bodies[i] );
			data->hash = b3Hash( data->hash, (uint8_t*)&xf, sizeof( b3WorldTransform ) );
		}

		data->sleepStep = data->stepCount;
	}

	data->stepCount += 1;

	return data->hash != 0;
}

void DestroyQuerySpawn( QuerySpawnData* data )
{
	(void)data;
}
