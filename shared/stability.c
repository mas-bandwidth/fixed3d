// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "stability.h"

#include "utils.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stdlib.h>

MeshDropData CreateMeshDrop( b3WorldId worldId, b3Pos origin )
{
	MeshDropData data = { 0 };
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = origin;
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		int gridCount = 40;
		b3Fixed cellWidth = B3_FIX( 1.0f );
		b3Fixed rowHz = B3_FIX( 0.1f );
		b3Fixed columnHz = B3_FIX( 0.2f );
		b3Fixed groundAmplitude = B3_FIX( 0.5f );

		data.mesh = b3CreateWaveMesh( gridCount, gridCount, cellWidth, groundAmplitude, rowHz, columnHz );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		b3CreateMeshShape( groundId, &shapeDef, data.mesh, b3Vec3_one );
	}

	{
		b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.02f ), B3_FIX( 0.2f ), B3_FIX( 0.04f ) );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;

		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.rollingResistance = B3_FIX( 0.1f );

		// Don't allow shapes to collide with each other.
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = 1;

		g_randomSeed = 3963634789;

		int gridCount = MESH_DROP_GRID_COUNT;

		for ( int i = 0; i < gridCount; ++i )
		{
			for ( int j = 0; j < gridCount; ++j )
			{
				b3Vec3 linearVelocity = RandomVec3Uniform( -B3_FIX( 1.0f ), B3_FIX( 1.0f ) );
				b3Vec3 angularVelocity = RandomVec3Uniform( -B3_FIX( 5.0f ), B3_FIX( 5.0f ) );

				bodyDef.position =
					b3OffsetPos( origin, (b3Vec3){ b3FixMul( B3_FIX( 0.5f ) , ( b3FixFromInt( i ) - b3FixMul( B3_FIX( 0.5f ) , b3FixFromInt( gridCount ) ) ) ), B3_FIX( 5.0f ), b3FixMul( B3_FIX( 0.5f ) , ( b3FixFromInt( j ) - b3FixMul( B3_FIX( 0.5f ) , b3FixFromInt( gridCount ) ) ) ) } );
				bodyDef.linearVelocity = linearVelocity;
				bodyDef.angularVelocity = angularVelocity;
				b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
				data.bodies[i * gridCount + j] = bodyId;

				b3CreateHullShape( bodyId, &shapeDef, &box.base );
			}
		}
	}

	return data;
}

bool UpdateMeshDrop( b3WorldId worldId, MeshDropData* data )
{
	if ( data->hash == 0 )
	{
		if ( b3World_GetAwakeBodyCount( worldId ) == 0 )
		{
			data->hash = B3_HASH_INIT;
			int bodyCount = MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT;
			for ( int i = 0; i < bodyCount; ++i )
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

void DestroyMeshDrop( MeshDropData* data )
{
	b3DestroyMesh( data->mesh );
}
