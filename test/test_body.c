// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

// Reach into internals to observe body extents and the dirty mass flag.
#include "body.h"
#include "physics_world.h"


// b3UpdateBodyMassData shifts each shape's inertia to the body center of mass with the parallel
// axis theorem. When shapes sit far from the body origin the shift term dwarfs the central inertia,
// so any error in the per shape framing blows up the tensor. Spheres make a clean oracle: the
// central inertia is isotropic and independent of placement, so the shift is the only thing tested.

static b3MassData SphereBodyMass( const b3Vec3* centers, int count, b3Fixed radius, b3Fixed density )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = density;

	for ( int i = 0; i < count; ++i )
	{
		b3Sphere sphere = { centers[i], radius };
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	b3Body_ApplyMassFromShapes( bodyId );
	b3MassData massData = b3Body_GetMassData( bodyId );

	b3DestroyWorld( worldId );
	return massData;
}

// One sphere far from the body origin. The center of mass lands on the sphere and the inertia about
// it must be the bare central inertia, with no trace of the offset.
static int FarSingleSphereMass( void )
{
	b3Fixed radius = B3_FIX( 0.5f );
	b3Fixed density = B3_FIX( 1.0f );
	b3Vec3 center = { B3_FIX( 100.0f ), -B3_FIX( 50.0f ), B3_FIX( 75.0f ) };
	b3MassData md = SphereBodyMass( &center, 1, radius, density );

	b3Fixed mass = b3FixMul( b3FixMul( b3FixMul( b3FixMul( b3FixMul( density , ( b3FixDiv( B3_FIX( 4.0f ) , B3_FIX( 3.0f ) ) ) ) , B3_PI ) , radius ) , radius ) , radius );
	b3Fixed central = b3FixMul( b3FixMul( b3FixMul( B3_FIX( 0.4f ) , mass ) , radius ) , radius );

	ENSURE_SMALL( md.mass - mass, B3_FIX( 1e-4f ) );

	ENSURE_SMALL( md.center.x - center.x, B3_FIX( 1e-3f ) );
	ENSURE_SMALL( md.center.y - center.y, B3_FIX( 1e-3f ) );
	ENSURE_SMALL( md.center.z - center.z, B3_FIX( 1e-3f ) );

	ENSURE_SMALL( md.inertia.cx.x - central, B3_FIX( 1e-3f ) );
	ENSURE_SMALL( md.inertia.cy.y - central, B3_FIX( 1e-3f ) );
	ENSURE_SMALL( md.inertia.cz.z - central, B3_FIX( 1e-3f ) );

	ENSURE_SMALL( md.inertia.cy.x, B3_FIX( 1e-3f ) );
	ENSURE_SMALL( md.inertia.cz.x, B3_FIX( 1e-3f ) );
	ENSURE_SMALL( md.inertia.cz.y, B3_FIX( 1e-3f ) );

	return 0;
}

// Eight equal spheres on the corners of a cube, the whole cube parked far from the body origin.
// The center of mass is the cube center and the products of inertia cancel by symmetry, so the
// tensor stays diagonal no matter how far out the cube sits.
static int FarCubeSphereMass( void )
{
	b3Fixed radius = B3_FIX( 0.5f );
	b3Fixed density = B3_FIX( 1.0f );
	b3Fixed h = B3_FIX( 1.0f );
	b3Vec3 p = { B3_FIX( 100.0f ), B3_FIX( 100.0f ), B3_FIX( 100.0f ) };

	b3Vec3 centers[8];
	int k = 0;
	for ( int sx = -1; sx <= 1; sx += 2 )
	{
		for ( int sy = -1; sy <= 1; sy += 2 )
		{
			for ( int sz = -1; sz <= 1; sz += 2 )
			{
				centers[k++] = (b3Vec3){ p.x + b3FixMul( b3FixFromInt( sx ) , h ), p.y + b3FixMul( b3FixFromInt( sy ) , h ), p.z + b3FixMul( b3FixFromInt( sz ) , h ) };
			}
		}
	}

	b3MassData md = SphereBodyMass( centers, 8, radius, density );

	b3Fixed mass = b3FixMul( b3FixMul( b3FixMul( b3FixMul( b3FixMul( density , ( b3FixDiv( B3_FIX( 4.0f ) , B3_FIX( 3.0f ) ) ) ) , B3_PI ) , radius ) , radius ) , radius );
	b3Fixed totalMass = b3FixMul( B3_FIX( 8.0f ) , mass );

	// Per sphere central inertia summed, plus the parallel axis term for each corner offset
	// (dy^2 + dz^2) = (h^2 + h^2) about every axis.
	b3Fixed diag = b3FixMul( b3FixMul( b3FixMul( b3FixMul( B3_FIX( 8.0f ) , B3_FIX( 0.4f ) ) , mass ) , radius ) , radius ) + b3FixMul( b3FixMul( b3FixMul( B3_FIX( 16.0f ) , mass ) , h ) , h );

	ENSURE_SMALL( md.mass - totalMass, B3_FIX( 1e-3f ) );

	ENSURE_SMALL( md.center.x - p.x, B3_FIX( 1e-2f ) );
	ENSURE_SMALL( md.center.y - p.y, B3_FIX( 1e-2f ) );
	ENSURE_SMALL( md.center.z - p.z, B3_FIX( 1e-2f ) );

	ENSURE_SMALL( md.inertia.cx.x - diag, B3_FIX( 1e-2f ) );
	ENSURE_SMALL( md.inertia.cy.y - diag, B3_FIX( 1e-2f ) );
	ENSURE_SMALL( md.inertia.cz.z - diag, B3_FIX( 1e-2f ) );

	ENSURE_SMALL( md.inertia.cy.x, B3_FIX( 1e-2f ) );
	ENSURE_SMALL( md.inertia.cz.x, B3_FIX( 1e-2f ) );
	ENSURE_SMALL( md.inertia.cz.y, B3_FIX( 1e-2f ) );

	return 0;
}

// Shapes added with updateBodyMass = false defer the mass update, which is also the only place
// body extents are computed. A body that reaches the solver with minExtent == B3_HUGE never passes
// the continuous collision gate. The dirty mass flag must track the deferral, and both
// ApplyMassFromShapes and SetMassData must leave finite extents behind.
static int DeferredMassExtents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	shapeDef.updateBodyMass = false;

	b3Sphere sphere = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.5f ) };

	// Deferred create leaves mass and extents untouched but marks the body dirty.
	b3BodyId applyId = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( applyId, &shapeDef, &sphere );

	b3World* world = b3GetWorld( applyId.world0 );
	b3Body* applyBody = b3GetBodyFullId( world, applyId );
	b3BodySim* applySim = b3GetBodySim( world, applyBody );

	ENSURE( ( applyBody->flags & b3_dirtyMass ) != 0 );
	ENSURE( applySim->minExtent == B3_HUGE );

	// ApplyMassFromShapes computes extents and clears the flag.
	b3Body_ApplyMassFromShapes( applyId );
	ENSURE( ( applyBody->flags & b3_dirtyMass ) == 0 );
	ENSURE( applySim->minExtent < B3_HUGE );

	// SetMassData alone must also produce finite extents and clear the flag (the issue #35 repro).
	b3BodyId massId = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( massId, &shapeDef, &sphere );

	b3Body* massBody = b3GetBodyFullId( world, massId );
	b3BodySim* massSim = b3GetBodySim( world, massBody );
	ENSURE( ( massBody->flags & b3_dirtyMass ) != 0 );

	b3Matrix3 inertia = { { B3_FIX( 0.2f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.2f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.2f ) } };
	b3MassData massData = { B3_FIX( 2.0f ), { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, inertia };
	b3Body_SetMassData( massId, massData );

	ENSURE( ( massBody->flags & b3_dirtyMass ) == 0 );
	ENSURE( massSim->minExtent < B3_HUGE );

	b3DestroyWorld( worldId );
	return 0;
}

// b3Body_SetMassData overrides the mass properties directly, bypassing the shapes. It must derive
// everything the solver reads from the supplied tensor: the inverse mass, the local inverse inertia,
// and the world inverse inertia rotated by the body orientation. Fixed rotation zeros the angular part.
// These tests drive it through the public getters, no shapes required.

// Diagonal inertia with inverses that are exact in b3Fixed, so tolerances stay tight.
static const b3Matrix3 kDiagInertia = { { B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 8.0f ) } };

static int SetMassDataRoundTrip( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 5.0f ), -B3_FIX( 3.0f ), B3_FIX( 2.0f ) };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Vec3 center = { B3_FIX( 0.1f ), B3_FIX( 0.2f ), B3_FIX( 0.3f ) };
	b3MassData massData = { B3_FIX( 3.0f ), center, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	ENSURE_SMALL( b3Body_GetMass( bodyId ) - B3_FIX( 3.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( b3Body_GetInverseMass( bodyId ) - b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), 8 * B3_FIXED_EPSILON );

	b3MassData md = b3Body_GetMassData( bodyId );
	ENSURE_SMALL( md.mass - B3_FIX( 3.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.center.x - center.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.center.y - center.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.center.z - center.z, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.inertia.cx.x - B3_FIX( 2.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.inertia.cy.y - B3_FIX( 4.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.inertia.cz.z - B3_FIX( 8.0f ), 8 * B3_FIXED_EPSILON );

	b3Vec3 localCenter = b3Body_GetLocalCenter( bodyId );
	ENSURE_SMALL( localCenter.x - center.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localCenter.y - center.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localCenter.z - center.z, 8 * B3_FIXED_EPSILON );

	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x - B3_FIX( 2.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cy.y - B3_FIX( 4.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cz.z - B3_FIX( 8.0f ), 8 * B3_FIXED_EPSILON );

	// Identity rotation, so the world inverse inertia is just the local inverse: diag(1/2, 1/4, 1/8).
	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x - B3_FIX( 0.5f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cy.y - B3_FIX( 0.25f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cz.z - B3_FIX( 0.125f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cy.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cz.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cx.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cz.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cx.z, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cy.z, 8 * B3_FIXED_EPSILON );

	// World center of mass is the body origin plus the local center under identity rotation.
	b3Pos worldCenter = b3Body_GetWorldCenter( bodyId );
	ENSURE_SMALL( worldCenter.x - ( B3_FIX( 5.0f ) + center.x ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( worldCenter.y - ( -B3_FIX( 3.0f ) + center.y ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( worldCenter.z - ( B3_FIX( 2.0f ) + center.z ), 8 * B3_FIXED_EPSILON );

	b3DestroyWorld( worldId );
	return 0;
}

// The world inverse inertia must be the local inverse rotated into world space. A 90 degree turn about
// z swaps the x and y principal moments, so diag(1/2, 1/4, 1/8) becomes diag(1/4, 1/2, 1/8). This is the
// regression guard: before SetMassData rotated the tensor it left the world inverse inertia stale.
static int SetMassDataWorldInertiaRotated( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, b3FixMul( B3_FIX( 0.5f ) , B3_PI ) );
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MassData massData = { B3_FIX( 1.0f ), { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	// The local inertia is stored untouched by the world transform.
	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x - B3_FIX( 2.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cy.y - B3_FIX( 4.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cz.z - B3_FIX( 8.0f ), 8 * B3_FIXED_EPSILON );

	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x - B3_FIX( 0.25f ), B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cy.y - B3_FIX( 0.5f ), B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cz.z - B3_FIX( 0.125f ), B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cy.x, B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cz.x, B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cx.y, B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cz.y, B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cx.z, B3_FIX( 1e-4f ) );
	ENSURE_SMALL( invWorld.cy.z, B3_FIX( 1e-4f ) );

	b3DestroyWorld( worldId );
	return 0;
}

// Fixed rotation must leave the mass intact but drive the whole angular inertia to zero, even when the
// caller hands in a real tensor.
static int SetMassDataFixedRotation( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MassData massData = { B3_FIX( 5.0f ), { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	ENSURE_SMALL( b3Body_GetMass( bodyId ) - B3_FIX( 5.0f ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( b3Body_GetInverseMass( bodyId ) - B3_FIX( 0.2f ), 8 * B3_FIXED_EPSILON );

	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cy.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cz.z, 8 * B3_FIXED_EPSILON );

	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cy.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cz.z, 8 * B3_FIXED_EPSILON );

	b3MassData md = b3Body_GetMassData( bodyId );
	ENSURE_SMALL( md.inertia.cx.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.inertia.cy.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( md.inertia.cz.z, 8 * B3_FIXED_EPSILON );

	b3DestroyWorld( worldId );
	return 0;
}

// Zero mass and a zero tensor have zero determinant, so the inverses must collapse to zero rather than
// divide by it.
static int SetMassDataZeroMass( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3MassData massData = { B3_FIX( 0.0f ), { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, b3Mat3_zero };
	b3Body_SetMassData( bodyId, massData );

	ENSURE_SMALL( b3Body_GetInverseMass( bodyId ), 8 * B3_FIXED_EPSILON );

	b3Matrix3 localInertia = b3Body_GetLocalRotationalInertia( bodyId );
	ENSURE_SMALL( localInertia.cx.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cy.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( localInertia.cz.z, 8 * B3_FIXED_EPSILON );

	b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertia( bodyId );
	ENSURE_SMALL( invWorld.cx.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cy.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( invWorld.cz.z, 8 * B3_FIXED_EPSILON );

	b3DestroyWorld( worldId );
	return 0;
}

// The stored linear velocity tracks the center of mass. Moving the center picks a different material
// point, so a spinning body must have its velocity re-referenced by omega x (newCenter - oldCenter),
// otherwise the mass edit silently injects or drains kinetic energy. Under identity rotation the world
// shift equals the supplied local center, keeping the expected values exact in b3Fixed.
static int SetMassDataConsistentVelocity( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 7.0f ), B3_FIX( 1.0f ), -B3_FIX( 4.0f ) };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// Spin about the origin center, then shift the center of mass off the origin.
	b3Vec3 omega = { B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 4.0f ) };
	b3Body_SetLinearVelocity( bodyId, (b3Vec3){ B3_FIX( 1.0f ), -B3_FIX( 2.0f ), B3_FIX( 3.0f ) } );
	b3Body_SetAngularVelocity( bodyId, omega );

	b3Vec3 center = { B3_FIX( 0.5f ), B3_FIX( 0.25f ), B3_FIX( 0.125f ) };
	b3MassData massData = { B3_FIX( 3.0f ), center, kDiagInertia };
	b3Body_SetMassData( bodyId, massData );

	// omega x center = ( 2*0.125 - 4*0.25, 4*0.5 - 1*0.125, 1*0.25 - 2*0.5 ) = ( -0.75, 1.875, -0.75 )
	b3Vec3 v = b3Body_GetLinearVelocity( bodyId );
	ENSURE_SMALL( v.x - ( B3_FIX( 1.0f ) - B3_FIX( 0.75f ) ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( v.y - ( -B3_FIX( 2.0f ) + B3_FIX( 1.875f ) ), 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( v.z - ( B3_FIX( 3.0f ) - B3_FIX( 0.75f ) ), 8 * B3_FIXED_EPSILON );

	// Only the reference point moved, the angular velocity is untouched.
	b3Vec3 w = b3Body_GetAngularVelocity( bodyId );
	ENSURE_SMALL( w.x - omega.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( w.y - omega.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( w.z - omega.z, 8 * B3_FIXED_EPSILON );

	b3DestroyWorld( worldId );
	return 0;
}

// Zero inverse mass has exactly one meaning in this engine: the body is static. It must
// never also mean "your mass was large", which is what b3FixDiv( 1, mass ) produced --
// truncating toward zero, so every mass above 65,536 units inverted to exactly 0 and the
// body became immovable by any impulse while still reporting itself dynamic. A silent
// dynamic-to-static category flip, and it diverges from the float reference, where 1/huge
// is tiny but never zero.
//
// The boundary is pinned on both sides because it is the whole point: 65,536 is the
// largest mass whose inverse is representable in Q48.16, and one unit past it the honest
// answer is one quantum rather than none.
//
// COARSENESS NEAR THE CEILING is real and deliberate. One quantum of inverse mass is an
// effective mass of 65,536, and the next quantum up is 32,768 -- so masses in that range
// do not resolve. A consumer wanting graded response keeps configured masses well under
// the ceiling; a consumer past it gets a body that is very heavy rather than one that is
// secretly nailed down.
static int InverseMassQuantumFloor( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	// At the line: 1 / 65,536 is exactly one quantum, with no clamping involved.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 65536.0f ), b3Vec3_zero, kDiagInertia };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMass( bodyId ) == 1 );
	}

	// One unit past it, where the true inverse is 0.99998 of a quantum. Truncation gives
	// zero; the floor gives the smallest inverse mass this format has.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 65537.0f ), b3Vec3_zero, kDiagInertia };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMass( bodyId ) == 1 );
	}

	// Far past it, the case space actually contains. Still dynamic, still movable.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 1.0e7f ), b3Vec3_zero, kDiagInertia };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMass( bodyId ) > 0 );
	}

	// And the floor is a floor, not a rewrite: an ordinary mass is unaffected.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 4.0f ), b3Vec3_zero, kDiagInertia };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMass( bodyId ) == b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 4.0f ) ) );
	}

	// A static body keeps zero inverse mass through its own explicit path, which is the
	// distinction the floor exists to protect.
	{
		b3BodyDef staticDef = b3DefaultBodyDef();
		staticDef.type = b3_staticBody;
		b3BodyId bodyId = b3CreateBody( worldId, &staticDef );
		ENSURE( b3Body_GetInverseMass( bodyId ) == 0 );
	}

	// Zero mass on a dynamic body is not a large mass and must not be floored -- the
	// engine reads it as the degenerate case it is.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 0.0f ), b3Vec3_zero, b3Mat3_zero };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMass( bodyId ) == 0 );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// The same floor on the path that computes mass from shapes and density rather than
// taking it from the caller, which is how a real heavy body is usually built. A 40-unit
// box at density 1 masses 64,000 -- under the line -- so the density is raised to carry
// it past.
static int InverseMassQuantumFloorFromShapes( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull hull = b3MakeBoxHull( B3_FIX( 20.0f ), B3_FIX( 20.0f ), B3_FIX( 20.0f ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 10.0f );
	b3CreateHullShape( bodyId, &shapeDef, &hull.base );

	ENSURE( b3Body_GetMass( bodyId ) > B3_FIX( 65536.0f ) );
	ENSURE( b3Body_GetInverseMass( bodyId ) > 0 );

	b3DestroyWorld( worldId );
	return 0;
}

int BodyTest( void )
{
	RUN_SUBTEST( InverseMassQuantumFloor );
	RUN_SUBTEST( InverseMassQuantumFloorFromShapes );
	RUN_SUBTEST( FarSingleSphereMass );
	RUN_SUBTEST( FarCubeSphereMass );
	RUN_SUBTEST( DeferredMassExtents );
	RUN_SUBTEST( SetMassDataRoundTrip );
	RUN_SUBTEST( SetMassDataWorldInertiaRotated );
	RUN_SUBTEST( SetMassDataFixedRotation );
	RUN_SUBTEST( SetMassDataZeroMass );
	RUN_SUBTEST( SetMassDataConsistentVelocity );
	return 0;
}
