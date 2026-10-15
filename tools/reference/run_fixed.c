// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// The fixed3d side of the reference harness: links the in-tree box3d and converts
// every decimal in the scene to b3Fixed at the one sanctioned boundary (b3FixFromDouble).
// Outputs the shared trace format on stdout. See ref_scene.h.

#include "ref_scene.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

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

// Own the baked bytes until after the world/player is destroyed.
static b3CompoundData* makeCompound( const b3RefScene* scene, const b3RefShape* rs )
{
	b3CompoundSphereDef spheres[B3_REF_MAX_CHILDREN] = { 0 };
	b3CompoundCapsuleDef capsules[B3_REF_MAX_CHILDREN] = { 0 };
	b3CompoundHullDef hulls[B3_REF_MAX_CHILDREN] = { 0 };
	b3BoxHull boxes[B3_REF_MAX_CHILDREN];
	b3HullData* owned[B3_REF_MAX_CHILDREN] = { 0 };
	b3CompoundDef def = { 0 };
	def.spheres = spheres;
	def.capsules = capsules;
	def.hulls = hulls;
	bool valid = true;
	for ( int i = 0; i < rs->childCount; ++i )
	{
		const b3RefChild* c = &scene->children[rs->childStart + i];
		b3Transform transform = { vecFromDouble( c->position ), b3NormalizeQuat( quatFromDouble( c->rotation ) ) };
		b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
		material.friction = b3FixFromDouble( c->friction );
		material.restitution = b3FixFromDouble( c->restitution );
		if ( c->kind == b3_refSphere )
		{
			b3CompoundSphereDef* child = &spheres[def.sphereCount++];
			child->sphere = (b3Sphere){ transform.p, b3FixFromDouble( c->radius ) };
			child->material = material;
		}
		else if ( c->kind == b3_refCapsule )
		{
			b3CompoundCapsuleDef* child = &capsules[def.capsuleCount++];
			b3Vec3 p1 = { 0, -b3FixFromDouble( c->halfLength ), 0 };
			b3Vec3 p2 = { 0, b3FixFromDouble( c->halfLength ), 0 };
			child->capsule =
				(b3Capsule){ b3TransformPoint( transform, p1 ), b3TransformPoint( transform, p2 ), b3FixFromDouble( c->radius ) };
			child->material = material;
		}
		else
		{
			b3CompoundHullDef* child = &hulls[def.hullCount++];
			if ( c->kind == b3_refBox )
			{
				boxes[i] = b3MakeBoxHull( b3FixFromDouble( c->halfExtents[0] ), b3FixFromDouble( c->halfExtents[1] ),
										  b3FixFromDouble( c->halfExtents[2] ) );
				child->hull = &boxes[i].base;
			}
			else
			{
				b3Vec3 points[B3_REF_MAX_HULL_POINTS];
				for ( int j = 0; j < c->hullCount; ++j )
					points[j] = vecFromDouble( c->hullPoints[j] );
				owned[i] = b3CreateHull( points, c->hullCount, B3_MAX_HULL_VERTICES );
				child->hull = owned[i];
				if ( owned[i] == NULL )
				{
					valid = false;
					break;
				}
			}
			child->transform = transform;
			child->material = material;
		}
	}
	b3CompoundData* compound = valid ? b3CreateCompound( &def ) : NULL;
	for ( int i = 0; i < rs->childCount; ++i )
		if ( owned[i] != NULL )
			b3DestroyHull( owned[i] );
	return compound;
}

static b3MeshData* makeMesh( const b3RefShape* rs )
{
	b3Vec3 vertices[B3_REF_MAX_MESH_VERTICES];
	int32_t indices[3 * B3_REF_MAX_MESH_TRIANGLES];
	uint8_t materials[B3_REF_MAX_MESH_TRIANGLES];
	for ( int i = 0; i < rs->vertexCount; ++i )
		vertices[i] = vecFromDouble( rs->vertices[i] );
	for ( int i = 0; i < rs->triangleCount; ++i )
	{
		for ( int j = 0; j < 3; ++j )
			indices[3 * i + j] = rs->triangles[i][j];
		materials[i] = (uint8_t)rs->materialIndices[i];
	}
	b3MeshDef def = { 0 };
	def.vertices = vertices;
	def.indices = indices;
	def.materialIndices = materials;
	def.vertexCount = rs->vertexCount;
	def.triangleCount = rs->triangleCount;
	def.identifyEdges = true;
	return b3CreateMesh( &def, NULL, 0 );
}

static int shapeOrdinal( b3ShapeId shape, b3BodyId* bodies, int bodyCount )
{
	b3BodyId owner = b3Shape_GetBody( shape );
	for ( int i = 0; i < bodyCount; ++i )
	{
		if ( !B3_ID_EQUALS( owner, bodies[i] ) )
			continue;
		b3ShapeId shapes[B3_REF_MAX_SHAPES + 1];
		int count = b3Body_GetShapes( owner, shapes, B3_REF_MAX_SHAPES + 1 );
		for ( int j = 0; j < count; ++j )
			if ( B3_ID_EQUALS( shape, shapes[j] ) )
				return i * ( B3_REF_MAX_SHAPES + 1 ) + j;
	}
	return -1;
}

static bool traceContacts( b3BodyId* bodies, int bodyCount )
{
	b3RefContactSample rows[B3_REF_MAX_CONTACTS];
	int rowCount = 0;
	for ( int i = 0; i < bodyCount; ++i )
	{
		int capacity = b3Body_GetContactCapacity( bodies[i] );
		if ( capacity == 0 )
			continue;
		b3ContactData* data = malloc( (size_t)capacity * sizeof( *data ) );
		if ( data == NULL )
			return false;
		int count = b3Body_GetContactData( bodies[i], data, capacity );
		for ( int j = 0; j < count; ++j )
		{
			// Body enumeration sees each contact twice; emit from the canonical
			// lower shape's body. IDs themselves remain opaque.
			b3BodyId bodyA = b3Shape_GetBody( data[j].shapeIdA );
			b3BodyId bodyB = b3Shape_GetBody( data[j].shapeIdB );
			int a = shapeOrdinal( data[j].shapeIdA, bodies, bodyCount );
			int b = shapeOrdinal( data[j].shapeIdB, bodies, bodyCount );
			if ( a < 0 || b < 0 )
			{
				free( data );
				return false;
			}
			b3BodyId emitBody = a < b ? bodyA : bodyB;
			if ( !B3_ID_EQUALS( emitBody, bodies[i] ) )
				continue;
			for ( int m = 0; m < data[j].manifoldCount; ++m )
			{
				const b3Manifold* manifold = &data[j].manifolds[m];
				if ( manifold->pointCount == 0 )
					continue;
				if ( rowCount >= B3_REF_MAX_CONTACTS || manifold->pointCount > B3_REF_MAX_CONTACT_POINTS )
				{
					free( data );
					return false;
				}
				b3RefContactSample* row = &rows[rowCount++];
				row->shapeA = a < b ? a : b;
				row->shapeB = a < b ? b : a;
				row->pointCount = manifold->pointCount;
				for ( int k = 0; k < row->pointCount; ++k )
				{
					const b3ManifoldPoint* point = &manifold->points[k];
					b3Pos pa = b3OffsetPos( b3Body_GetWorldCenter( bodyA ), point->anchorA );
					b3Pos pb = b3OffsetPos( b3Body_GetWorldCenter( bodyB ), point->anchorB );
					row->points[k][0] = 0.5 * ( b3FixToDouble( pa.x ) + b3FixToDouble( pb.x ) );
					row->points[k][1] = 0.5 * ( b3FixToDouble( pa.y ) + b3FixToDouble( pb.y ) );
					row->points[k][2] = 0.5 * ( b3FixToDouble( pa.z ) + b3FixToDouble( pb.z ) );
					row->points[k][3] = b3FixToDouble( point->separation );
				}
			}
		}
		free( data );
	}
	b3RefSortContacts( rows, rowCount );
	printf( "contacts %d\n", rowCount );
	for ( int i = 0; i < rowCount; ++i )
	{
		b3RefContactSample* c = &rows[i];
		printf( "contact %d %d %d", c->shapeA, c->shapeB, c->pointCount );
		for ( int j = 0; j < c->pointCount; ++j )
			printf( " %.9g %.9g %.9g %.9g", c->points[j][0], c->points[j][1], c->points[j][2], c->points[j][3] );
		printf( "\n" );
	}
	return true;
}

int main( int argc, char** argv )
{
	if ( argc < 2 )
	{
		fprintf( stderr, "usage: run_fixed <scene>\n" );
		return 1;
	}

	static b3RefScene scene;
	char error[256];
	if ( !b3RefParseSceneFile( argv[1], &scene, error, sizeof( error ) ) )
	{
		fprintf( stderr, "parse error: %s\n", error );
		return 1;
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = vecFromDouble( scene.world.gravity );
	if ( scene.world.restitutionIterations > 0 )
	{
		worldDef.restitutionIterations = scene.world.restitutionIterations;
	}
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

	b3CompoundData* baked[B3_REF_MAX_SHAPES] = { 0 };
	b3MeshData* meshes[B3_REF_MAX_SHAPES] = { 0 };
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
			case b3_refCompound:
			{
				baked[i] = makeCompound( &scene, rs );
				if ( baked[i] == NULL )
				{
					fprintf( stderr, "compound build failed for shape %d\n", i );
					return 1;
				}
				b3ShapeId id = b3CreateBakedCompoundShape( bodies[rs->bodyIndex], &def, baked[i] );
				if ( B3_IS_NULL( id ) )
				{
					fprintf( stderr, "compound shape creation refused\n" );
					return 1;
				}
			}
			break;
			case b3_refMesh:
			{
				meshes[i] = makeMesh( rs );
				if ( meshes[i] == NULL || meshes[i]->triangleCount != rs->triangleCount )
				{
					fprintf( stderr, "mesh build failed or degenerate triangles in shape %d\n", i );
					return 1;
				}
				b3SurfaceMaterial materials[B3_REF_MAX_MATERIALS];
				for ( int j = 0; j < rs->materialCount; ++j )
				{
					materials[j] = b3DefaultSurfaceMaterial();
					materials[j].friction = b3FixFromDouble( rs->materials[j][0] );
					materials[j].restitution = b3FixFromDouble( rs->materials[j][1] );
				}
				def.materials = materials;
				def.materialCount = rs->materialCount;
				b3Vec3 scale = { b3FixFromDouble( 1 ), b3FixFromDouble( 1 ), b3FixFromDouble( 1 ) };
				b3ShapeId id = b3CreateMeshShape( bodies[rs->bodyIndex], &def, meshes[i], scale );
				if ( B3_IS_NULL( id ) )
				{
					fprintf( stderr, "mesh shape creation refused\n" );
					return 1;
				}
			}
			break;

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

	b3Recording* snapshot = NULL;
	b3RecPlayer* restored = NULL;
	int steps = scene.world.steps;
	for ( int s = 0; s < steps; ++s )
	{

		// Snapshot boundaries are BEFORE this numbered step, including its events.
		if ( s == scene.snapshotSaveStep )
		{
			snapshot = b3CreateRecording( 0 );
			if ( snapshot == NULL )
				return 1;
			b3World_StartRecording( worldId, snapshot );
			b3World_StopRecording( worldId );
		}
		if ( s == scene.snapshotRestoreStep )
		{
			restored = b3CreatePlayer( b3Recording_GetData( snapshot ), b3Recording_GetSize( snapshot ), 1 );
			if ( restored == NULL || b3RecPlayer_GetBodyCount( restored ) != scene.bodyCount )
			{
				fprintf( stderr, "snapshot restore failed\n" );
				return 1;
			}
			b3DestroyWorld( worldId );
			worldId = b3RecPlayer_GetWorldId( restored );
			for ( int i = 0; i < scene.bodyCount; ++i )
				bodies[i] = b3RecPlayer_GetBodyId( restored, i );
		}
		// Restore rewinds simulation time. Events in the rewound interval repeat.
		int logicalStep = s < scene.snapshotRestoreStep || scene.snapshotRestoreStep < 0
							  ? s
							  : scene.snapshotSaveStep + s - scene.snapshotRestoreStep;
		createJointsAtStep( &scene, worldId, bodies, logicalStep );
		for ( int e = 0; e < scene.eventCount; ++e )
		{
			if ( scene.events[e].step == logicalStep )
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

		if ( scene.traceContacts && !traceContacts( bodies, scene.bodyCount ) )
		{
			fprintf( stderr, "contact tracing failed or exceeded its capacity\n" );
			return 1;
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
				printf( "query %d raycast %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", i, r.hit ? 1 : 0, b3FixToDouble( r.fraction ),
						b3FixToDouble( r.point.x ), b3FixToDouble( r.point.y ), b3FixToDouble( r.point.z ),
						b3FixToDouble( r.normal.x ), b3FixToDouble( r.normal.y ), b3FixToDouble( r.normal.z ) );
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
				b3World_OverlapShape( worldId, b3ToPos( vecFromDouble( q->origin ) ), &proxy, filter, CountOverlapCallback,
									  &ctx );
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

	if ( restored != NULL )
		b3DestroyPlayer( restored );
	else
		b3DestroyWorld( worldId );
	b3DestroyRecording( snapshot );
	for ( int i = 0; i < scene.shapeCount; ++i )
	{
		if ( baked[i] != NULL )
			b3DestroyCompound( baked[i] );
		if ( meshes[i] != NULL )
			b3DestroyMesh( meshes[i] );
	}
	for ( int i = 0; i < scene.bodyCount; ++i )
	{
		if ( compounds[i] != NULL )
		{
			b3DestroyCompound( compounds[i] );
		}
	}
	return 0;
}
