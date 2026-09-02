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

	// 65,536 was the old cliff -- the largest mass whose inverse was representable at all.
	// It is now unremarkable, and the point of these two cases is that it no longer marks
	// anything: the inverse is exact on both sides of it.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 65536.0f ), b3Vec3_zero, kDiagInertia };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMassPrecise( bodyId ) == ( (int64_t)1 << 24 ) );
		ENSURE( b3Body_GetInverseMass( bodyId ) == B3_FIXED_EPSILON );
	}

	// One unit past it. The stored inverse is a real, graded value -- the whole point of
	// the wider format -- while the legacy getter truncates it to zero, which is the
	// documented lossiness rather than a body that stopped moving.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 65537.0f ), b3Vec3_zero, kDiagInertia };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMassPrecise( bodyId ) > 0 );
		ENSURE( b3Body_GetInverseMassPrecise( bodyId ) < ( (int64_t)1 << 24 ) );
		ENSURE( b3Body_GetInverseMass( bodyId ) == 0 );
	}

	// Far past it, the case space actually contains. Still dynamic, still movable.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 1.0e7f ), b3Vec3_zero, kDiagInertia };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMassPrecise( bodyId ) > 0 );
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
		ENSURE( b3Body_GetInverseMassPrecise( bodyId ) == 0 );
	}

	// Zero mass on a dynamic body is not a large mass and must not be floored -- the
	// engine reads it as the degenerate case it is.
	{
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3MassData massData = { B3_FIX( 0.0f ), b3Vec3_zero, b3Mat3_zero };
		b3Body_SetMassData( bodyId, massData );
		ENSURE( b3Body_GetInverseMassPrecise( bodyId ) == 0 );
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
	ENSURE( b3Body_GetInverseMassPrecise( bodyId ) > 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// THE INVERSE INERTIA SCALE ENVELOPE.
//
// Inertia grows as the fifth power of size, so a body 20 times larger has an inertia
// 3.2 million times larger, and the inverse the solver actually uses runs off the bottom
// of Q48.16 long before anything else complains. Every dynamic body in this repository's
// samples is under 2.5 units, a factor of five inside the range where the arithmetic was
// correct, which is exactly why the failure past it went unseen for so long.
//
// What is asserted here is the contract, not a particular number:
//
//   - An inverse inertia is NEVER NEGATIVE. The tensor is positive-definite, so its
//     inverse is too, and a negative diagonal is a body that accelerates against its own
//     torque. This is what the wide arm used to produce -- measured at cube side 250 as
//     an inverse of the wrong sign and 1.13e9 times the true magnitude.
//   - It never grows with the body. A larger body has a smaller inverse inertia, at every
//     size, monotonically. A wrapped reduction breaks that immediately.
//   - Past the representability line it is exactly zero, and it STAYS zero through a
//     transform change and a world step, which is where a refused tensor used to come
//     back to life.
//
// The line falls at 65,536 units of inertia, which for a uniform cube of density 1 is
// about 13.1 units on a side. A zero inverse inertia means the body does not rotate --
// which is a real limitation of the storage format, and a separate question from this
// test, which is only that the engine is honest about it rather than wrong about it.
static int InverseInertiaScaleEnvelope( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Cube sides spanning the whole useful range, in half units so the arithmetic is exact.
	static const int halfSides[] = { 2, 10, 22, 26, 27, 40, 75, 88, 160, 240, 500, 734, 1000 };

	b3Fixed previousDiagonal = B3_FIXED_MAX;

	for ( int i = 0; i < ARRAY_COUNT( halfSides ); ++i )
	{
		// Uniform solid cube of density 1: mass = s^3 and inertia = s^5/6, built from
		// integers so the input carries no conversion error of its own.
		int64_t s2 = halfSides[i];
		int64_t massRaw = ( s2 * s2 * s2 * 65536 ) / 8;
		int64_t inertiaRaw = ( ( ( s2 * s2 * s2 * s2 ) / 3 ) * s2 * 1024 );

		b3Matrix3 inertia = b3Mat3_zero;
		inertia.cx.x = (b3Fixed)inertiaRaw;
		inertia.cy.y = (b3Fixed)inertiaRaw;
		inertia.cz.z = (b3Fixed)inertiaRaw;

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		b3MassData massData = { (b3Fixed)massRaw, b3Vec3_zero, inertia };
		b3Body_SetMassData( bodyId, massData );

		// Created at identity rotation, so the world tensor IS the local one here.
		b3Matrix3 invLocal = b3Body_GetWorldInverseRotationalInertiaPrecise( bodyId );

		// THE ORACLE, and it shares no arithmetic with the implementation: for a diagonal
		// tensor the inverse entry is exactly 2^(16 + fractionBits) / inertiaRaw,
		// truncated toward zero. An equality at every size, including the sizes where the
		// right answer is zero.
		// 2^(16 + fraction bits) is 2^56 at the current format and fits an int64 with room
		// to spare, so the oracle needs no wide arithmetic of its own -- which is the
		// point of it.
		b3Fixed expected = (b3Fixed)( ( (int64_t)1 << ( 16 + b3GetInverseFractionBits() ) ) / inertiaRaw );
		ENSURE( invLocal.cx.x == expected );
		ENSURE( invLocal.cy.y == expected );
		ENSURE( invLocal.cz.z == expected );

		// Never negative, and a diagonal tensor inverts to a diagonal one.
		ENSURE( invLocal.cx.x >= 0 && invLocal.cy.y >= 0 && invLocal.cz.z >= 0 );
		ENSURE( invLocal.cx.y == 0 && invLocal.cx.z == 0 );
		ENSURE( invLocal.cy.x == 0 && invLocal.cy.z == 0 );
		ENSURE( invLocal.cz.x == 0 && invLocal.cz.y == 0 );

		// Monotone: a bigger body never has a bigger inverse inertia.
		ENSURE( invLocal.cx.x <= previousDiagonal );
		previousDiagonal = invLocal.cx.x;

		// THE UNLOCK. Everything from side 13.5 to side 250 used to be zero -- rotation
		// locked, spin never damping -- and is now a real graded value. This is the
		// acceptance criterion for the whole widening, so it is asserted per size rather
		// than described.
		if ( s2 <= 500 )
		{
			ENSURE( invLocal.cx.x > 0 );
		}

		// The world tensor is a similarity transform of the local one, so it inherits the
		// contract. Rotating must not manufacture a negative entry.
		b3Body_SetTransform( bodyId, (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
							 b3MakeQuatFromAxisAngle( b3Normalize( (b3Vec3){ B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 3.0f ) } ),
													  B3_FIX( 0.7f ) ) );

		b3Matrix3 invWorld = b3Body_GetWorldInverseRotationalInertiaPrecise( bodyId );
		ENSURE( invWorld.cx.x >= 0 && invWorld.cy.y >= 0 && invWorld.cz.z >= 0 );

		if ( invLocal.cx.x == 0 )
		{
			// Past the line the refusal still has to stick, through a transform change and
			// through a step of the world.
			ENSURE( invWorld.cx.x == 0 && invWorld.cy.y == 0 && invWorld.cz.z == 0 );

			b3World_Step( worldId, B3_FIX( 1.0f ) / 60, 4 );

			invWorld = b3Body_GetWorldInverseRotationalInertiaPrecise( bodyId );
			ENSURE( invWorld.cx.x == 0 && invWorld.cy.y == 0 && invWorld.cz.z == 0 );
		}
	}

	// The walk still has to arrive at zero, or the monotonicity above would be satisfied
	// by a constant. The line has moved from about side 13 to about side 367 -- a factor
	// of 28 in length, and of 17 million in inertia.
	ENSURE( previousDiagonal == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// Extents bound the shapes about the center of mass, per axis. An offset shape must count its
// offset, not just its own size. Upstream's tolerances of 1e-5 sit BELOW this engine's
// 1.5e-5 quantum, so they are floored to 8 * B3_FIXED_EPSILON per the tree's convention.
static int ShapeExtents( void )
{
	const b3Fixed tol = 8 * B3_FIXED_EPSILON;
	const b3Fixed meshTol = B3_FIX( 0.0001f );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );

	// Kinematic bodies measure from the body origin
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;

	b3BodyId capsuleId = b3CreateBody( worldId, &bodyDef );
	b3Capsule capsule = { { -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
						  { -B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) },
						  B3_FIX( 0.2f ) };
	b3CreateCapsuleShape( capsuleId, &shapeDef, &capsule );

	// Both capsule centers sit on the -x side of the origin, so a componentwise max without
	// b3Abs picks the NEARER end and under-reports the extent by a whole unit.
	b3Vec3 maxExtent = b3Body_GetMaxExtent( capsuleId );
	ENSURE_SMALL( maxExtent.x - B3_FIX( 2.2f ), tol );
	ENSURE_SMALL( maxExtent.y - B3_FIX( 0.2f ), tol );
	ENSURE_SMALL( maxExtent.z - B3_FIX( 0.2f ), tol );
	ENSURE_SMALL( b3Body_GetMinExtent( capsuleId ) - B3_FIX( 0.2f ), tol );

	b3BodyId sphereId = b3CreateBody( worldId, &bodyDef );
	b3Sphere sphere = { { B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 3.0f ) }, B3_FIX( 0.5f ) };
	b3CreateSphereShape( sphereId, &shapeDef, &sphere );

	maxExtent = b3Body_GetMaxExtent( sphereId );
	ENSURE_SMALL( maxExtent.x - B3_FIX( 1.5f ), tol );
	ENSURE_SMALL( maxExtent.y - B3_FIX( 2.5f ), tol );
	ENSURE_SMALL( maxExtent.z - B3_FIX( 3.5f ), tol );
	ENSURE_SMALL( b3Body_GetMinExtent( sphereId ) - B3_FIX( 0.5f ), tol );

	// Dynamic bodies measure from the center of mass. A light sphere hung off a cube pulls the
	// center toward it, so the far side of the sphere is the widest point.
	bodyDef.type = b3_dynamicBody;
	b3BodyId cubeId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull cube = b3MakeCubeHull( B3_FIX( 0.5f ) );
	b3CreateHullShape( cubeId, &shapeDef, &cube.base );
	b3Sphere offsetSphere = { { B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) }, B3_FIX( 0.2f ) };
	b3CreateSphereShape( cubeId, &shapeDef, &offsetSphere );

	b3Vec3 localCenter = b3Body_GetLocalCenter( cubeId );
	ENSURE( B3_FIX( 0.0f ) < localCenter.x && localCenter.x < B3_FIX( 0.5f ) );

	maxExtent = b3Body_GetMaxExtent( cubeId );
	ENSURE_SMALL( maxExtent.x - ( B3_FIX( 1.2f ) - localCenter.x ), tol );
	ENSURE_SMALL( maxExtent.y - B3_FIX( 0.5f ), tol );
	ENSURE_SMALL( maxExtent.z - B3_FIX( 0.5f ), tol );
	ENSURE_SMALL( b3Body_GetMinExtent( cubeId ) - B3_FIX( 0.2f ), tol );

	b3Vec3 originExtent = b3Body_GetMaxExtentOrigin( cubeId );
	ENSURE_SMALL( originExtent.x - B3_FIX( 1.2f ), tol );
	ENSURE_SMALL( originExtent.y - B3_FIX( 0.5f ), tol );
	ENSURE_SMALL( originExtent.z - B3_FIX( 0.5f ), tol );

	// A mesh has no mass, so the sphere alone places the center at x = 1 and the far edge of the
	// mesh at x = -3 is four units out.
	b3Vec3 vertices[6] = {
		{ -B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
		{ -B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
		{ -B3_FIX( 1.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
		{ B3_FIX( 1.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
		{ B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) },
		{ B3_FIX( 3.0f ), B3_FIX( 0.0f ), -B3_FIX( 1.0f ) },
	};
	int32_t indices[6] = { 0, 1, 2, 3, 4, 5 };
	b3MeshDef meshDef = { 0 };
	meshDef.vertices = vertices;
	meshDef.stride = sizeof( b3Vec3 );
	meshDef.indices = indices;
	meshDef.vertexCount = 6;
	meshDef.triangleCount = 2;
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );
	ENSURE( mesh != NULL );

	b3BodyId meshId = b3CreateBody( worldId, &bodyDef );
	b3CreateSphereShape( meshId, &shapeDef, &offsetSphere );
	b3Vec3 unitScale = { B3_FIX( 1.0f ), B3_FIX( 1.0f ), B3_FIX( 1.0f ) };
	b3CreateMeshShape( meshId, &shapeDef, mesh, unitScale );

	localCenter = b3Body_GetLocalCenter( meshId );
	ENSURE_SMALL( localCenter.x - B3_FIX( 1.0f ), tol );

	// Without subtracting the local center this reports |p| rather than |p - c|
	maxExtent = b3Body_GetMaxExtent( meshId );
	ENSURE_SMALL( maxExtent.x - B3_FIX( 4.0f ), meshTol );
	ENSURE_SMALL( maxExtent.y - B3_FIX( 0.2f ), meshTol );
	ENSURE_SMALL( maxExtent.z - B3_FIX( 1.0f ), meshTol );
	ENSURE_SMALL( b3Body_GetMinExtent( meshId ) - B3_FIX( 0.2f ), tol );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

int BodyTest( void )
{
	RUN_SUBTEST( InverseMassQuantumFloor );
	RUN_SUBTEST( InverseInertiaScaleEnvelope );
	RUN_SUBTEST( InverseMassQuantumFloorFromShapes );
	RUN_SUBTEST( FarSingleSphereMass );
	RUN_SUBTEST( FarCubeSphereMass );
	RUN_SUBTEST( DeferredMassExtents );
	RUN_SUBTEST( SetMassDataRoundTrip );
	RUN_SUBTEST( SetMassDataWorldInertiaRotated );
	RUN_SUBTEST( SetMassDataFixedRotation );
	RUN_SUBTEST( SetMassDataZeroMass );
	RUN_SUBTEST( SetMassDataConsistentVelocity );
	RUN_SUBTEST( ShapeExtents );
	return 0;
}
