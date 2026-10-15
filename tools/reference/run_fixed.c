// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// The fixed3d side of the reference harness: links the in-tree box3d and converts
// every decimal in the scene to b3Fixed at the one sanctioned boundary (b3FixFromDouble).
// Outputs the shared trace format on stdout. See ref_scene.h.

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "ref_scene.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static b3Vec3 vecFromDouble( const double v[3] )
{
	b3Vec3 o = { b3FixFromDouble( v[0] ), b3FixFromDouble( v[1] ), b3FixFromDouble( v[2] ) };
	return o;
}

static b3Quat quatFromDouble( const double v[4] )
{
	b3Quat q = { { b3FixFromDouble( v[0] ), b3FixFromDouble( v[1] ), b3FixFromDouble( v[2] ) }, b3FixFromDouble( v[3] ) };
	return q;
}

typedef struct OverlapContext
{
	int count;
} OverlapContext;

static bool CountOverlapCallback( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	OverlapContext* ctx = (OverlapContext*)context;
	ctx->count += 1;
	return true;
}

typedef struct CastContext
{
	bool hit;
	b3Fixed fraction;
	b3Pos point;
	b3Vec3 normal;
} CastContext;

static b3Fixed ClosestCastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, b3Fixed fraction, uint64_t userMaterialId,
									int triangleIndex, int childIndex, void* context )
{
	(void)shapeId;
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;
	CastContext* ctx = (CastContext*)context;
	if ( ctx->hit == false || fraction < ctx->fraction )
	{
		ctx->hit = true;
		ctx->fraction = fraction;
		ctx->point = point;
		ctx->normal = normal;
	}
	return fraction; // continue searching for a closer hit
}

static b3ShapeProxy makeProxy( b3RefQuery* q, b3BoxHull* boxOut )
{
	b3ShapeProxy proxy;
	if ( q->proxyKind == b3_refBox )
	{
		*boxOut = b3MakeBoxHull( b3FixFromDouble( q->proxyHalfExtents[0] ), b3FixFromDouble( q->proxyHalfExtents[1] ),
								 b3FixFromDouble( q->proxyHalfExtents[2] ) );
		proxy.points = boxOut->boxPoints;
		proxy.count = boxOut->base.vertexCount;
		proxy.radius = B3_FIX( 0.0f );
	}
	else if ( q->proxyKind == b3_refCapsule )
	{
		b3Vec3 axis = b3Normalize( vecFromDouble( q->proxyAxis ) );
		b3Fixed half = b3FixFromDouble( q->proxyHalfLength );
		b3Fixed rad = b3FixFromDouble( q->proxyRadius );
		boxOut->boxPoints[0] = (b3Vec3){ b3FixMul( -axis.x, half ), b3FixMul( -axis.y, half ), b3FixMul( -axis.z, half ) };
		boxOut->boxPoints[1] = (b3Vec3){ b3FixMul( axis.x, half ), b3FixMul( axis.y, half ), b3FixMul( axis.z, half ) };
		proxy.points = boxOut->boxPoints;
		proxy.count = 2;
		proxy.radius = rad;
	}
	else // sphere
	{
		boxOut->boxPoints[0] = b3Vec3_zero;
		proxy.points = boxOut->boxPoints;
		proxy.count = 1;
		proxy.radius = b3FixFromDouble( q->proxyRadius );
	}
	return proxy;
}

static void createJointsAtStep( b3RefScene* scene, b3WorldId worldId, b3BodyId* bodies, int step )
{
	for ( int i = 0; i < scene->jointCount; ++i )
	{
		b3RefJoint* rj = &scene->joints[i];
		if ( rj->atStep != step )
		{
			continue;
		}
		b3Vec3 a = vecFromDouble( rj->anchorA );
		b3Vec3 b = vecFromDouble( rj->anchorB );
		switch ( rj->kind )
		{
			case b3_refRevolute:
			{
				b3RevoluteJointDef jd = b3DefaultRevoluteJointDef();
				jd.base.bodyIdA = bodies[rj->bodyA];
				jd.base.bodyIdB = bodies[rj->bodyB];
				jd.base.localFrameA = (b3Transform){ a, b3Quat_identity };
				jd.base.localFrameB = (b3Transform){ b, b3Quat_identity };
				jd.base.collideConnected = rj->collideConnected;
				b3CreateRevoluteJoint( worldId, &jd );
			}
			break;
			case b3_refSpherical:
			{
				b3SphericalJointDef jd = b3DefaultSphericalJointDef();
				jd.base.bodyIdA = bodies[rj->bodyA];
				jd.base.bodyIdB = bodies[rj->bodyB];
				jd.base.localFrameA = (b3Transform){ a, b3Quat_identity };
				jd.base.localFrameB = (b3Transform){ b, b3Quat_identity };
				jd.base.collideConnected = rj->collideConnected;
				b3CreateSphericalJoint( worldId, &jd );
			}
			break;
			case b3_refDistance:
			{
				b3DistanceJointDef jd = b3DefaultDistanceJointDef();
				jd.base.bodyIdA = bodies[rj->bodyA];
				jd.base.bodyIdB = bodies[rj->bodyB];
				jd.base.localFrameA = (b3Transform){ a, b3Quat_identity };
				jd.base.localFrameB = (b3Transform){ b, b3Quat_identity };
				jd.length = b3FixFromDouble( rj->length );
				jd.base.collideConnected = rj->collideConnected;
				b3CreateDistanceJoint( worldId, &jd );
			}
			break;
			case b3_refWeld:
			{
				b3WeldJointDef jd = b3DefaultWeldJointDef();
				jd.base.bodyIdA = bodies[rj->bodyA];
				jd.base.bodyIdB = bodies[rj->bodyB];
				jd.base.localFrameA = (b3Transform){ a, b3Quat_identity };
				jd.base.localFrameB = (b3Transform){ b, b3Quat_identity };
				jd.base.collideConnected = rj->collideConnected;
				b3CreateWeldJoint( worldId, &jd );
			}
			break;
		}
	}
}

int main( int argc, char** argv )
{
	if ( argc < 2 )
	{
		fprintf( stderr, "usage: run_fixed <scene>\n" );
		return 1;
	}

	b3RefScene scene;
	char error[256];
	if ( !b3RefParseSceneFile( argv[1], &scene, error, sizeof( error ) ) )
	{
		fprintf( stderr, "parse error: %s\n", error );
		return 1;
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = vecFromDouble( scene.world.gravity );
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId bodies[B3_REF_MAX_BODIES];
	for ( int i = 0; i < scene.bodyCount; ++i )
	{
		b3RefBody* rb = &scene.bodies[i];
		b3BodyDef def = b3DefaultBodyDef();
		switch ( rb->type )
		{
			case b3_refStatic:
				def.type = b3_staticBody;
				break;
			case b3_refKinematic:
				def.type = b3_kinematicBody;
				break;
			default:
				def.type = b3_dynamicBody;
				break;
		}
		def.position = b3ToPos( vecFromDouble( rb->position ) );
		def.rotation = b3NormalizeQuat( quatFromDouble( rb->rotation ) );
		def.linearVelocity = vecFromDouble( rb->linearVelocity );
		def.angularVelocity = vecFromDouble( rb->angularVelocity );
		def.isBullet = rb->isBullet;
		def.allowFastRotation = rb->allowFastRotation;
		def.enableSleep = rb->enableSleep;
		def.safetyFactor = b3FixFromDouble( rb->safetyFactor );
		bodies[i] = b3CreateBody( worldId, &def );
	}

	for ( int i = 0; i < scene.shapeCount; ++i )
	{
		b3RefShape* rs = &scene.shapes[i];
		b3ShapeDef def = b3DefaultShapeDef();
		def.density = b3FixFromDouble( rs->density );
		def.baseMaterial.friction = b3FixFromDouble( rs->friction );
		def.baseMaterial.restitution = b3FixFromDouble( rs->restitution );
		def.baseMaterial.rollingResistance = b3FixFromDouble( rs->rollingResistance );
		def.isSensor = rs->isSensor;

		switch ( rs->kind )
		{
			case b3_refBox:
			{
				b3BoxHull hull = b3MakeBoxHull( b3FixFromDouble( rs->halfExtents[0] ), b3FixFromDouble( rs->halfExtents[1] ),
												b3FixFromDouble( rs->halfExtents[2] ) );
				b3CreateHullShape( bodies[rs->bodyIndex], &def, &hull.base );
			}
			break;
			case b3_refSphere:
			{
				b3Sphere sphere = { vecFromDouble( rs->center ), b3FixFromDouble( rs->radius ) };
				b3CreateSphereShape( bodies[rs->bodyIndex], &def, &sphere );
			}
			break;
			case b3_refCapsule:
			{
				b3Vec3 axis = b3Normalize( vecFromDouble( rs->capsuleAxis ) );
				b3Fixed half = b3FixFromDouble( rs->halfLength );
				b3Vec3 neg = { -axis.x, -axis.y, -axis.z };
				b3Capsule capsule = {
					{ b3FixMul( neg.x, half ), b3FixMul( neg.y, half ), b3FixMul( neg.z, half ) },
					{ b3FixMul( axis.x, half ), b3FixMul( axis.y, half ), b3FixMul( axis.z, half ) },
					b3FixFromDouble( rs->radius ),
				};
				b3CreateCapsuleShape( bodies[rs->bodyIndex], &def, &capsule );
			}
			break;
			case b3_refHull:
			{
				b3Vec3 pts[B3_REF_MAX_HULL_POINTS];
				for ( int k = 0; k < rs->hullCount; ++k )
				{
					pts[k] = vecFromDouble( rs->hullPoints[k] );
				}
				b3HullData* hull = b3CreateHull( pts, rs->hullCount, B3_MAX_HULL_VERTICES );
				if ( hull != NULL )
				{
					b3CreateHullShape( bodies[rs->bodyIndex], &def, hull );
					b3DestroyHull( hull );
				}
			}
			break;
		}
	}

	b3CompoundData* compounds[B3_REF_MAX_BODIES] = { 0 };
	for ( int i = 0; i < scene.bodyCount; ++i )
	{
		if ( scene.compoundChildCount[i] == 0 )
		{
			continue;
		}
		b3CompoundSphereDef spheres[B3_REF_MAX_COMPOUND_CHILDREN];
		for ( int k = 0; k < scene.compoundChildCount[i]; ++k )
		{
			spheres[k].sphere = (b3Sphere){ vecFromDouble( scene.compoundChildren[i][k].center ),
											b3FixFromDouble( scene.compoundChildren[i][k].radius ) };
			spheres[k].material = b3DefaultSurfaceMaterial();
		}
		b3CompoundDef compoundDef = { 0 };
		compoundDef.spheres = spheres;
		compoundDef.sphereCount = scene.compoundChildCount[i];
		b3CompoundData* compound = b3CreateCompound( &compoundDef );
		if ( compound != NULL )
		{
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateBakedCompoundShape( bodies[i], &shapeDef, compound );
			compounds[i] = compound; // the shape references this data; free it after the world
		}
	}

	b3Fixed dt = b3FixDiv( b3FixFromDouble( scene.world.dtNumerator ), b3FixFromDouble( scene.world.dtDenominator ) );
	int steps = scene.world.steps;
	for ( int s = 0; s < steps; ++s )
	{
		createJointsAtStep( &scene, worldId, bodies, s );
		for ( int e = 0; e < scene.eventCount; ++e )
		{
			if ( scene.events[e].step == s )
			{
				if ( scene.events[e].kind == b3_refEventSetMass )
				{
					b3MassData massData = b3Body_GetMassData( bodies[scene.events[e].bodyIndex] );
					massData.center = vecFromDouble( scene.events[e].center );
					b3Body_SetMassData( bodies[scene.events[e].bodyIndex], massData );
				}
			}
		}
		b3World_Step( worldId, dt, scene.world.subSteps );

		b3Counters counters = b3World_GetCounters( worldId );
		int awake = b3World_GetAwakeBodyCount( worldId );
		printf( "step %d counters %d %d %d\n", s, counters.contactCount, awake, counters.jointCount );
		for ( int i = 0; i < scene.bodyCount; ++i )
		{
			b3Pos p = b3Body_GetPosition( bodies[i] );
			b3Quat q = b3Body_GetRotation( bodies[i] );
			b3Vec3 v = b3Body_GetLinearVelocity( bodies[i] );
			b3Vec3 w = b3Body_GetAngularVelocity( bodies[i] );
			printf( "body %d p %.9g %.9g %.9g q %.9g %.9g %.9g %.9g v %.9g %.9g %.9g w %.9g %.9g %.9g\n", i, b3FixToDouble( p.x ),
					b3FixToDouble( p.y ), b3FixToDouble( p.z ), b3FixToDouble( q.v.x ), b3FixToDouble( q.v.y ),
					b3FixToDouble( q.v.z ), b3FixToDouble( q.s ), b3FixToDouble( v.x ), b3FixToDouble( v.y ),
					b3FixToDouble( v.z ), b3FixToDouble( w.x ), b3FixToDouble( w.y ), b3FixToDouble( w.z ) );
		}

		// Predicate-mode observable: the lowest transformed hull vertex per
		// hull-shape body (the resting height). Emitted only when the scene's
		// expect lines are predicates, so v1 traces stay byte-identical.
		if ( scene.expect.predicateCount > 0 )
		{
			for ( int i = 0; i < scene.bodyCount; ++i )
			{
				int hullShape = -1;
				for ( int k = 0; k < scene.shapeCount; ++k )
				{
					if ( scene.shapes[k].bodyIndex == i && scene.shapes[k].kind == b3_refHull )
					{
						hullShape = k;
						break;
					}
				}
				if ( hullShape < 0 )
				{
					continue;
				}
				b3Pos c = b3Body_GetPosition( bodies[i] );
				b3Quat q = b3Body_GetRotation( bodies[i] );
				b3Pos lowest = c;
				bool have = false;
				for ( int k = 0; k < scene.shapes[hullShape].hullCount; ++k )
				{
					b3Vec3 local = vecFromDouble( scene.shapes[hullShape].hullPoints[k] );
					b3Pos wp = b3OffsetPos( c, b3RotateVector( q, local ) );
					if ( !have || wp.y < lowest.y )
					{
						lowest = wp;
						have = true;
					}
				}
				printf( "low %d %.9g\n", i, b3FixToDouble( lowest.y ) );
			}
		}
	}

	b3QueryFilter filter = b3DefaultQueryFilter();
	for ( int i = 0; i < scene.queryCount; ++i )
	{
		b3RefQuery* q = &scene.queries[i];
		switch ( q->kind )
		{
			case b3_refRayCast:
			{
				b3RayResult r = b3World_CastRayClosest( worldId, b3ToPos( vecFromDouble( q->origin ) ),
														vecFromDouble( q->translation ), filter );
				printf( "query %d raycast %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", i, r.hit ? 1 : 0,
						b3FixToDouble( r.fraction ), b3FixToDouble( r.point.x ), b3FixToDouble( r.point.y ),
						b3FixToDouble( r.point.z ), b3FixToDouble( r.normal.x ), b3FixToDouble( r.normal.y ),
						b3FixToDouble( r.normal.z ) );
			}
			break;
			case b3_refOverlapAABB:
			{
				b3AABB aabb = { b3ToPos( vecFromDouble( q->lower ) ), b3ToPos( vecFromDouble( q->upper ) ) };
				OverlapContext ctx = { 0 };
				b3World_OverlapAABB( worldId, aabb, filter, CountOverlapCallback, &ctx );
				printf( "query %d overlap_aabb %d\n", i, ctx.count );
			}
			break;
			case b3_refOverlapShape:
			{
				b3BoxHull scratch;
				b3ShapeProxy proxy = makeProxy( q, &scratch );
				OverlapContext ctx = { 0 };
				b3World_OverlapShape( worldId, b3ToPos( vecFromDouble( q->origin ) ), &proxy, filter, CountOverlapCallback, &ctx );
				printf( "query %d overlap_shape %d\n", i, ctx.count );
			}
			break;
			case b3_refShapeCast:
			{
				b3BoxHull scratch;
				b3ShapeProxy proxy = makeProxy( q, &scratch );
				CastContext ctx = { 0 };
				b3World_CastShape( worldId, b3ToPos( vecFromDouble( q->origin ) ), &proxy, vecFromDouble( q->translation ),
								   filter, ClosestCastCallback, &ctx );
				if ( ctx.hit )
				{
					printf( "query %d shape_cast 1 %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", i, b3FixToDouble( ctx.fraction ),
							b3FixToDouble( ctx.point.x ), b3FixToDouble( ctx.point.y ), b3FixToDouble( ctx.point.z ),
							b3FixToDouble( ctx.normal.x ), b3FixToDouble( ctx.normal.y ), b3FixToDouble( ctx.normal.z ) );
				}
				else
				{
					printf( "query %d shape_cast 0 1 0 0 0 0 0 0\n", i );
				}
			}
			break;
			case b3_refClosestPoints:
			{
				b3Vec3 target = b3ToVec3( b3Body_GetPosition( bodies[q->bodyB] ) );
				b3Vec3 point = b3Vec3_zero;
				b3Fixed dist = b3Body_GetClosestPoint( bodies[q->bodyA], &point, target );
				printf( "query %d closest_points %.9g %.9g %.9g %.9g\n", i, b3FixToDouble( dist ), b3FixToDouble( point.x ),
						b3FixToDouble( point.y ), b3FixToDouble( point.z ) );
			}
			break;
			case b3_refMover:
			{
				b3Vec3 axis = b3Normalize( vecFromDouble( q->proxyAxis ) );
				b3Fixed half = b3FixFromDouble( q->proxyHalfLength );
				b3Vec3 neg = { -axis.x, -axis.y, -axis.z };
				b3Capsule mover = { { b3FixMul( neg.x, half ), b3FixMul( neg.y, half ), b3FixMul( neg.z, half ) },
									{ b3FixMul( axis.x, half ), b3FixMul( axis.y, half ), b3FixMul( axis.z, half ) },
									b3FixFromDouble( q->proxyRadius ) };
				b3Fixed frac = b3World_CastMover( worldId, b3ToPos( vecFromDouble( q->origin ) ), &mover,
												  vecFromDouble( q->translation ), filter, NULL, NULL );
				printf( "query %d mover %.9g\n", i, b3FixToDouble( frac ) );
			}
			break;
		}
	}

	b3DestroyWorld( worldId );
	for ( int i = 0; i < scene.bodyCount; ++i )
	{
		if ( compounds[i] != NULL )
		{
			b3DestroyCompound( compounds[i] );
		}
	}
	return 0;
}
