// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// The vanilla-box3d side of the reference harness: compiled against erincatto/box3d
// at the commit named by -DBOX3D_REFERENCE_REV=<sha> (fetched via ExternalProject).
// Numbers stay float/double; there is no fixed-point conversion. Outputs the SAME
// trace format as run_fixed.c. See ref_scene.h.

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"
#include "ref_scene.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Report vanilla box3d asserts to stderr (unbuffered) and continue rather than
// trapping, so a reference-side assert surfaces as a comparison divergence instead
// of silently killing the harness.
static int ReferenceAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	fprintf( stderr, "BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stderr );
	return 0;
}

static b3Vec3 vecFromDouble( const double v[3] )
{
	b3Vec3 o = { (float)v[0], (float)v[1], (float)v[2] };
	return o;
}

static b3Quat quatFromDouble( const double v[4] )
{
	b3Quat q = { { (float)v[0], (float)v[1], (float)v[2] }, (float)v[3] };
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
	float fraction;
	b3Pos point;
	b3Vec3 normal;
} CastContext;

static float ClosestCastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
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
	return fraction;
}

static b3ShapeProxy makeProxy( b3RefQuery* q, b3BoxHull* boxOut )
{
	b3ShapeProxy proxy;
	if ( q->proxyKind == b3_refBox )
	{
		*boxOut = b3MakeBoxHull( (float)q->proxyHalfExtents[0], (float)q->proxyHalfExtents[1], (float)q->proxyHalfExtents[2] );
		proxy.points = boxOut->boxPoints;
		proxy.count = boxOut->base.vertexCount;
		proxy.radius = 0.0f;
	}
	else if ( q->proxyKind == b3_refCapsule )
	{
		b3Vec3 axis = b3Normalize( vecFromDouble( q->proxyAxis ) );
		float half = (float)q->proxyHalfLength;
		float rad = (float)q->proxyRadius;
		boxOut->boxPoints[0] = (b3Vec3){ -axis.x * half, -axis.y * half, -axis.z * half };
		boxOut->boxPoints[1] = (b3Vec3){ axis.x * half, axis.y * half, axis.z * half };
		proxy.points = boxOut->boxPoints;
		proxy.count = 2;
		proxy.radius = rad;
	}
	else // sphere
	{
		boxOut->boxPoints[0] = (b3Vec3){ 0.0f, 0.0f, 0.0f };
		proxy.points = boxOut->boxPoints;
		proxy.count = 1;
		proxy.radius = (float)q->proxyRadius;
	}
	return proxy;
}

int main( int argc, char** argv )
{
	if ( argc < 2 )
	{
		fprintf( stderr, "usage: run_float <scene>\n" );
		return 1;
	}

	b3RefScene scene;
	char error[256];
	if ( !b3RefParseSceneFile( argv[1], &scene, error, sizeof( error ) ) )
	{
		fprintf( stderr, "parse error: %s\n", error );
		return 1;
	}

	b3SetAssertFcn( ReferenceAssertFcn );

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
		def.position = vecFromDouble( rb->position );
		def.rotation = b3NormalizeQuat( quatFromDouble( rb->rotation ) );
		def.linearVelocity = vecFromDouble( rb->linearVelocity );
		def.angularVelocity = vecFromDouble( rb->angularVelocity );
		def.isBullet = rb->isBullet;
		def.allowFastRotation = rb->allowFastRotation;
		def.enableSleep = rb->enableSleep;
		bodies[i] = b3CreateBody( worldId, &def );
	}

	for ( int i = 0; i < scene.shapeCount; ++i )
	{
		b3RefShape* rs = &scene.shapes[i];
		b3ShapeDef def = b3DefaultShapeDef();
		def.density = (float)rs->density;
		def.baseMaterial.friction = (float)rs->friction;
		def.baseMaterial.restitution = (float)rs->restitution;
		def.baseMaterial.rollingResistance = (float)rs->rollingResistance;
		def.isSensor = rs->isSensor;

		switch ( rs->kind )
		{
			case b3_refBox:
			{
				b3BoxHull hull = b3MakeBoxHull( (float)rs->halfExtents[0], (float)rs->halfExtents[1], (float)rs->halfExtents[2] );
				b3CreateHullShape( bodies[rs->bodyIndex], &def, &hull.base );
			}
			break;
			case b3_refSphere:
			{
				b3Sphere sphere = { vecFromDouble( rs->center ), (float)rs->radius };
				b3CreateSphereShape( bodies[rs->bodyIndex], &def, &sphere );
			}
			break;
			case b3_refCapsule:
			{
				b3Vec3 axis = b3Normalize( vecFromDouble( rs->capsuleAxis ) );
				float half = (float)rs->halfLength;
				b3Capsule capsule = { { -axis.x * half, -axis.y * half, -axis.z * half },
									  { axis.x * half, axis.y * half, axis.z * half }, (float)rs->radius };
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

	for ( int i = 0; i < scene.jointCount; ++i )
	{
		b3RefJoint* rj = &scene.joints[i];
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
				jd.length = (float)rj->length;
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
				b3CreateWeldJoint( worldId, &jd );
			}
			break;
		}
	}

	float dt = (float)( scene.world.dtNumerator / scene.world.dtDenominator );
	int steps = scene.world.steps;
	for ( int s = 0; s < steps; ++s )
	{
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
			printf( "body %d p %.9g %.9g %.9g q %.9g %.9g %.9g %.9g v %.9g %.9g %.9g w %.9g %.9g %.9g\n", i, (double)p.x,
					(double)p.y, (double)p.z, (double)q.v.x, (double)q.v.y, (double)q.v.z, (double)q.s, (double)v.x, (double)v.y,
					(double)v.z, (double)w.x, (double)w.y, (double)w.z );
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
				b3RayResult r = b3World_CastRayClosest( worldId, vecFromDouble( q->origin ), vecFromDouble( q->translation ), filter );
				printf( "query %d raycast %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", i, r.hit ? 1 : 0, (double)r.fraction,
						(double)r.point.x, (double)r.point.y, (double)r.point.z, (double)r.normal.x, (double)r.normal.y,
						(double)r.normal.z );
			}
			break;
			case b3_refOverlapAABB:
			{
				b3AABB aabb = { vecFromDouble( q->lower ), vecFromDouble( q->upper ) };
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
				b3World_OverlapShape( worldId, vecFromDouble( q->origin ), &proxy, filter, CountOverlapCallback, &ctx );
				printf( "query %d overlap_shape %d\n", i, ctx.count );
			}
			break;
			case b3_refShapeCast:
			{
				b3BoxHull scratch;
				b3ShapeProxy proxy = makeProxy( q, &scratch );
				CastContext ctx = { 0 };
				b3World_CastShape( worldId, vecFromDouble( q->origin ), &proxy, vecFromDouble( q->translation ), filter,
								   ClosestCastCallback, &ctx );
				if ( ctx.hit )
				{
					printf( "query %d shape_cast 1 %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", i, (double)ctx.fraction, (double)ctx.point.x,
							(double)ctx.point.y, (double)ctx.point.z, (double)ctx.normal.x, (double)ctx.normal.y,
							(double)ctx.normal.z );
				}
				else
				{
					printf( "query %d shape_cast 0 1 0 0 0 0 0 0\n", i );
				}
			}
			break;
			case b3_refClosestPoints:
			{
				b3Vec3 target = b3Body_GetPosition( bodies[q->bodyB] );
				b3Vec3 point = { 0.0f, 0.0f, 0.0f };
				float dist = b3Body_GetClosestPoint( bodies[q->bodyA], &point, target );
				printf( "query %d closest_points %.9g %.9g %.9g %.9g\n", i, (double)dist, (double)point.x, (double)point.y,
						(double)point.z );
			}
			break;
			case b3_refMover:
			{
				b3Vec3 axis = b3Normalize( vecFromDouble( q->proxyAxis ) );
				float half = (float)q->proxyHalfLength;
				b3Capsule mover = { { -axis.x * half, -axis.y * half, -axis.z * half },
									{ axis.x * half, axis.y * half, axis.z * half }, (float)q->proxyRadius };
				float frac = b3World_CastMover( worldId, vecFromDouble( q->origin ), &mover, vecFromDouble( q->translation ), filter,
												NULL, NULL );
				printf( "query %d mover %.9g\n", i, (double)frac );
			}
			break;
		}
	}

	b3DestroyWorld( worldId );
	return 0;
}
