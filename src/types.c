// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box3d/types.h"

#include "core.h"

#include "box3d/constants.h"

b3WorldDef b3DefaultWorldDef( void )
{
	b3Fixed lengthUnits = b3GetLengthUnitsPerMeter();

	b3WorldDef def = { b3FixFromInt( 0 ) };
	def.gravity.x = B3_FIX( 0.0f );
	def.gravity.y = -B3_FIX( 10.0f );
	def.hitEventThreshold = b3FixMul( B3_FIX( 1.0f ) , lengthUnits );
	def.restitutionThreshold = b3FixMul( B3_FIX( 1.0f ) , lengthUnits );
	def.contactSpeed = b3FixMul( B3_FIX( 3.0f ) , lengthUnits );
	def.contactHertz = B3_FIX( 30.0f );
	def.contactDampingRatio = B3_FIX( 10.0f );

	// 400 meters per second, faster than the speed of sound
	def.maximumLinearSpeed = b3FixMul( B3_FIX( 400.0f ) , lengthUnits );

	def.enableSleep = true;
	def.enableContinuous = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3BodyDef b3DefaultBodyDef( void )
{
	b3BodyDef def = { 0 };
	def.type = b3_staticBody;
	def.rotation = b3Quat_identity;
	def.sleepThreshold = b3FixMul( B3_FIX( 0.05f ) , b3GetLengthUnitsPerMeter() );
	def.gravityScale = B3_FIX( 1.0f );
	def.enableSleep = true;
	def.isAwake = true;
	def.isEnabled = true;
	def.enableContactRecycling = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3Filter b3DefaultFilter( void )
{
	b3Filter filter = { B3_DEFAULT_CATEGORY_BITS, B3_DEFAULT_MASK_BITS, 0 };
	return filter;
}

b3QueryFilter b3DefaultQueryFilter( void )
{
	b3QueryFilter filter = { B3_DEFAULT_CATEGORY_BITS, B3_DEFAULT_MASK_BITS, 0, NULL };
	return filter;
}

b3SurfaceMaterial b3DefaultSurfaceMaterial( void )
{
	b3SurfaceMaterial surfaceMaterial = { b3FixFromInt( 0 ) };
	surfaceMaterial.friction = B3_FIX( 0.6f );
	return surfaceMaterial;
}

b3ShapeDef b3DefaultShapeDef( void )
{
	b3Fixed lengthUnits = b3GetLengthUnitsPerMeter();

	b3ShapeDef def = { 0 };
	def.baseMaterial = b3DefaultSurfaceMaterial();
	// density of water
	def.density = b3FixDiv( B3_FIX( 1000.0f ) , ( b3FixMul( b3FixMul( lengthUnits , lengthUnits ) , lengthUnits ) ) );
	def.explosionScale = B3_FIX( 1.0f );
	def.filter = b3DefaultFilter();
	def.updateBodyMass = true;
	def.invokeContactCreation = true;
	def.enableSpeculativeContact = true;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

static bool b3EmptyDrawShape( void* userShape, b3WorldTransform transform, b3HexColor color, void* context )
{
	B3_UNUSED( userShape, transform, color, context );
	return false;
}

static void b3EmptyDrawSegment( b3Pos p1, b3Pos p2, b3HexColor color, void* context )
{
	B3_UNUSED( p1, p2, color, context );
}

static void b3EmptyDrawTransform( b3WorldTransform transform, void* context )
{
	B3_UNUSED( transform, context );
}

static void b3EmptyDrawPoint( b3Pos p, b3Fixed size, b3HexColor color, void* context )
{
	B3_UNUSED( p, size, color, context );
}

static void b3EmptyDrawSphere( b3Pos p, b3Fixed radius, b3HexColor color, b3Fixed alpha, void* context )
{
	B3_UNUSED( p, radius, color, alpha, context );
}

static void b3EmptyDrawCapsule( b3Pos p1, b3Pos p2, b3Fixed radius, b3HexColor color, b3Fixed alpha, void* context )
{
	B3_UNUSED( p1, p2, radius, color, alpha, context );
}

static void b3EmptyDrawBounds( b3AABB aabb, b3HexColor color, void* context )
{
	B3_UNUSED( aabb, color, context );
}

static void b3EmptyDrawBox( b3Vec3 extents, b3WorldTransform transform, b3HexColor color, void* context )
{
	B3_UNUSED( extents, transform, color, context );
}

static void b3EmptyDrawString( b3Pos p, const char* s, b3HexColor color, void* context )
{
	B3_UNUSED( p, s, color, context );
}

b3DebugDraw b3DefaultDebugDraw( void )
{
	b3DebugDraw draw = { 0 };

	// These allow the user to skip some implementations and not hit null exceptions.
	draw.DrawShapeFcn = b3EmptyDrawShape;
	draw.DrawSegmentFcn = b3EmptyDrawSegment;
	draw.DrawTransformFcn = b3EmptyDrawTransform;
	draw.DrawPointFcn = b3EmptyDrawPoint;
	draw.DrawSphereFcn = b3EmptyDrawSphere;
	draw.DrawCapsuleFcn = b3EmptyDrawCapsule;
	draw.DrawBoundsFcn = b3EmptyDrawBounds;
	draw.DrawBoxFcn = b3EmptyDrawBox;
	draw.DrawStringFcn = b3EmptyDrawString;

	// Not too small, not too big.
	b3Fixed h = b3FixMul( B3_FIX( 100.0f ) , b3GetLengthUnitsPerMeter() );
	draw.drawingBounds = (b3AABB){
		.lowerBound = { -h, -h, -h },
		.upperBound = { h, h, h },
	};

	draw.jointScale = B3_FIX( 1.0f );
	draw.forceScale = B3_FIX( 1.0f );

	return draw;
}
