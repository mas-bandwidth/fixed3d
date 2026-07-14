// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "overflow_color.h"

#include "utils.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>

#define OVERFLOW_PILE_RING_COUNT 5
#define OVERFLOW_PILE_PER_RING 5

OverflowColorPileData CreateOverflowColorPile( b3WorldId worldId )
{
	OverflowColorPileData data = { 0 };
	data.neighborCount = OVERFLOW_PILE_RING_COUNT * OVERFLOW_PILE_PER_RING;

	// Static ground (top surface at y = 0)
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( B3_FIX( 20.0f ), B3_FIX( 1.0f ), B3_FIX( 20.0f ) );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		data.groundShapeId = b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	// Tall, heavy hub. Tall so neighbors can ring around it in multiple
	// vertical layers; heavy so it stays roughly still under uneven inward
	// pressure from the ring.
	b3Fixed hubHalfX = B3_FIX( 0.5f );
	b3Fixed hubHalfY = B3_FIX( 2.5f );
	b3Fixed hubHalfZ = B3_FIX( 0.5f );
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = b3OffsetPos( GetSceneOrigin(), (b3Vec3){ B3_FIX( 0.0f ), hubHalfY, B3_FIX( 0.0f ) } );
		data.hubId = b3CreateBody( worldId, &bodyDef );

		b3BoxHull box = b3MakeBoxHull( hubHalfX, hubHalfY, hubHalfZ );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.density = B3_FIX( 50.0f );
		b3CreateHullShape( data.hubId, &shapeDef, &box.base );
	}

	// Neighbors: vertical rings around the hub, each box slightly overlapping
	// the hub so a contact exists on the very first step.
	b3Fixed neighborHalf = B3_FIX( 0.2f );
	b3Fixed ringRadius = hubHalfX + neighborHalf - B3_FIX( 0.03f );

	b3BoxHull neighborBox = b3MakeBoxHull( neighborHalf, neighborHalf, neighborHalf );
	b3ShapeDef neighborShape = b3DefaultShapeDef();

	b3Fixed ringSpacing = B3_FIX( 0.5f );
	b3Fixed baseY = neighborHalf + B3_FIX( 0.05f );

	for ( int ring = 0; ring < OVERFLOW_PILE_RING_COUNT; ++ring )
	{
		b3Fixed y = baseY + b3FixMul( ringSpacing , b3FixFromInt( ring ) );

		// Offset alternate rings by half a slot so neighbors don't sit
		// directly above each other.
		// OVERFLOW_PILE_PER_RING is a plain int: divide with native `/`
		// (b3FixDiv would treat it as a b3Fixed of 5 ulps, 65536x too big).
		b3Fixed thetaOffset = ( ring & 1 ) ? ( B3_PI / OVERFLOW_PILE_PER_RING ) : B3_FIX( 0.0f );

		for ( int slot = 0; slot < OVERFLOW_PILE_PER_RING; ++slot )
		{
			b3Fixed theta = thetaOffset + ( b3FixMul( b3FixMul( B3_FIX( 2.0f ) , B3_PI ) , b3FixFromInt( slot ) ) ) / OVERFLOW_PILE_PER_RING;

			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position =
				b3OffsetPos( GetSceneOrigin(), (b3Vec3){ b3FixMul( ringRadius , b3Cos( theta ) ), y, b3FixMul( ringRadius , b3Sin( theta ) ) } );
			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

			b3CreateHullShape( bodyId, &neighborShape, &neighborBox.base );
		}
	}

	return data;
}
