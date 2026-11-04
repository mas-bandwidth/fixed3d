// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include "inverse.h"
#include "math_internal.h"

#include <math.h>
#include <stdint.h>

// One sub-test per joint type. Each creates the joint, exercises the shared
// b3Joint_* API plus every type-specific accessor, then steps to make sure the
// joint solves without tripping a validation assert.

typedef struct JointFixture
{
	b3WorldId worldId;
	b3BodyId groundId;
	b3BodyId bodyId;
} JointFixture;

// Static ground plus a dynamic box, anchored so a point-coincident joint starts
// satisfied. Gravity is off so the body stays put across the handful of steps
// each sub-test takes.
static JointFixture CreateJointFixture( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;

	JointFixture f;
	f.worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	f.groundId = b3CreateBody( f.worldId, &groundDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };
	f.bodyId = b3CreateBody( f.worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	b3BoxHull box = b3MakeCubeHull( B3_FIX( 0.5f ) );
	b3CreateHullShape( f.bodyId, &shapeDef, &box.base );

	return f;
}

// Place the joint anchor at the dynamic body so both local frames map to the
// same world point.
static void SetCommonFrames( b3JointDef* base, const JointFixture* f )
{
	base->bodyIdA = f->groundId;
	base->bodyIdB = f->bodyId;
	base->localFrameA.p = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };
	base->localFrameB.p = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
}

// Step a few times, destroy the joint, then the world. Destroying the joint
// explicitly also covers b3DestroyJoint and stale-handle detection.
static int FinishJoint( b3JointId jointId, b3WorldId worldId )
{
	for ( int i = 0; i < 8; ++i )
	{
		b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) ), 4 );
	}

	b3DestroyJoint( jointId, true );
	ENSURE( b3Joint_IsValid( jointId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// Exercise the API shared by every joint type. Frames are saved and restored so
// the caller's setup survives.
static int ExerciseJointBase( b3JointId jointId, b3WorldId worldId, b3BodyId bodyIdA, b3BodyId bodyIdB, b3JointType expectedType )
{
	ENSURE( b3Joint_IsValid( jointId ) );
	ENSURE( b3Joint_GetType( jointId ) == expectedType );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyA( jointId ), bodyIdA ) );
	ENSURE( B3_ID_EQUALS( b3Joint_GetBodyB( jointId ), bodyIdB ) );

	// B3_ID_EQUALS does not work for world ids
	b3WorldId gotWorld = b3Joint_GetWorld( jointId );
	ENSURE( gotWorld.index1 == worldId.index1 && gotWorld.generation == worldId.generation );

	b3Transform originalA = b3Joint_GetLocalFrameA( jointId );
	b3Transform originalB = b3Joint_GetLocalFrameB( jointId );

	b3Transform frameA = { { B3_FIX( 0.1f ), B3_FIX( 0.2f ), B3_FIX( 0.3f ) }, b3Quat_identity };
	b3Joint_SetLocalFrameA( jointId, frameA );
	b3Transform gotA = b3Joint_GetLocalFrameA( jointId );
	ENSURE( gotA.p.x == frameA.p.x && gotA.p.y == frameA.p.y && gotA.p.z == frameA.p.z );

	b3Transform frameB = { { -B3_FIX( 0.4f ), B3_FIX( 0.5f ), -B3_FIX( 0.6f ) }, b3Quat_identity };
	b3Joint_SetLocalFrameB( jointId, frameB );
	b3Transform gotB = b3Joint_GetLocalFrameB( jointId );
	ENSURE( gotB.p.x == frameB.p.x && gotB.p.y == frameB.p.y && gotB.p.z == frameB.p.z );

	b3Joint_SetCollideConnected( jointId, true );
	ENSURE( b3Joint_GetCollideConnected( jointId ) == true );
	b3Joint_SetCollideConnected( jointId, false );
	ENSURE( b3Joint_GetCollideConnected( jointId ) == false );

	int userData = 0;
	b3Joint_SetUserData( jointId, &userData );
	ENSURE( b3Joint_GetUserData( jointId ) == &userData );

	b3Joint_SetConstraintTuning( jointId, B3_FIX( 90.0f ), B3_FIX( 3.0f ) );
	b3Fixed hertz = B3_FIX( 0.0f );
	b3Fixed dampingRatio = B3_FIX( 0.0f );
	b3Joint_GetConstraintTuning( jointId, &hertz, &dampingRatio );
	ENSURE( hertz == B3_FIX( 90.0f ) );
	ENSURE( dampingRatio == B3_FIX( 3.0f ) );

	b3Joint_SetForceThreshold( jointId, B3_FIX( 100.0f ) );
	ENSURE( b3Joint_GetForceThreshold( jointId ) == B3_FIX( 100.0f ) );

	b3Joint_SetTorqueThreshold( jointId, B3_FIX( 200.0f ) );
	ENSURE( b3Joint_GetTorqueThreshold( jointId ) == B3_FIX( 200.0f ) );

	b3Joint_WakeBodies( jointId );

	// No stable value to assert before the first step, call for coverage
	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	b3Vec3 torque = b3Joint_GetConstraintTorque( jointId );
	b3Fixed linearSeparation = b3Joint_GetLinearSeparation( jointId );
	MAYBE_UNUSED( force );
	MAYBE_UNUSED( torque );
	MAYBE_UNUSED( linearSeparation );

	// Wheel joint angular separation is an unimplemented todo in joint.c that
	// asserts. Every other type computes it.
	if ( expectedType != b3_wheelJoint )
	{
		b3Fixed angularSeparation = b3Joint_GetAngularSeparation( jointId );
		MAYBE_UNUSED( angularSeparation );
	}

	b3Joint_SetLocalFrameA( jointId, originalA );
	b3Joint_SetLocalFrameB( jointId, originalB );

	return 0;
}

static int TestParallelJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3ParallelJointDef def = b3DefaultParallelJointDef();
	SetCommonFrames( &def.base, &f );
	def.hertz = B3_FIX( 2.0f );
	def.dampingRatio = B3_FIX( 0.5f );
	def.maxTorque = B3_FIX( 100.0f );
	b3JointId jointId = b3CreateParallelJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_parallelJoint ) != 0 )
	{
		return 1;
	}

	b3ParallelJoint_SetSpringHertz( jointId, B3_FIX( 5.0f ) );
	ENSURE( b3ParallelJoint_GetSpringHertz( jointId ) == B3_FIX( 5.0f ) );

	b3ParallelJoint_SetSpringDampingRatio( jointId, B3_FIX( 0.7f ) );
	ENSURE( b3ParallelJoint_GetSpringDampingRatio( jointId ) == B3_FIX( 0.7f ) );

	b3ParallelJoint_SetMaxTorque( jointId, B3_FIX( 250.0f ) );
	ENSURE( b3ParallelJoint_GetMaxTorque( jointId ) == B3_FIX( 250.0f ) );

	return FinishJoint( jointId, f.worldId );
}

// Regression for the sentinel impulse-cap wrap: the default maxTorque is the
// B3_FIXED_MAX "unlimited" sentinel (float used FLT_MAX, whose squared compare
// overflowed to infinity and never fired). b3FixMul( h * B3_FIXED_MAX,
// h * B3_FIXED_MAX ) wraps int64 — to exactly zero at h = 1/240 — so the
// spurious clamp fired on the first nonzero impulse and SCALED IT UP to
// ~5.8e11, injecting ~1e10 rad/s (found via the Joints/Driving sample: four
// wheels touching a height field exploded the chassis and then stalled CCD
// for seconds per frame). Verified red before the 128-bit cap compare fix.
static int TestParallelJointSentinelTorqueCap( void )
{
	JointFixture f = CreateJointFixture();

	b3ParallelJointDef def = b3DefaultParallelJointDef();
	SetCommonFrames( &def.base, &f );
	// Keep the sentinel default maxTorque; give the constraint a small error to
	// produce a nonzero corrective impulse.
	b3JointId jointId = b3CreateParallelJoint( f.worldId, &def );

	b3Body_SetAngularVelocity( f.bodyId, (b3Vec3){ B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.25f ) } );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) );
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( f.worldId, timeStep, 4 );
	}

	b3Vec3 w = b3Body_GetAngularVelocity( f.bodyId );
	ENSURE( b3Length( w ) < B3_FIX( 10.0f ) );

	return FinishJoint( jointId, f.worldId );
}

static int TestDistanceJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3DistanceJointDef def = b3DefaultDistanceJointDef();
	SetCommonFrames( &def.base, &f );
	def.length = B3_FIX( 2.0f );
	b3JointId jointId = b3CreateDistanceJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_distanceJoint ) != 0 )
	{
		return 1;
	}

	b3DistanceJoint_SetLength( jointId, B3_FIX( 3.0f ) );
	ENSURE( b3DistanceJoint_GetLength( jointId ) == B3_FIX( 3.0f ) );

	b3DistanceJoint_EnableSpring( jointId, true );
	ENSURE( b3DistanceJoint_IsSpringEnabled( jointId ) == true );

	b3DistanceJoint_SetSpringForceRange( jointId, -B3_FIX( 50.0f ), B3_FIX( 75.0f ) );
	b3Fixed lowerForce = B3_FIX( 0.0f );
	b3Fixed upperForce = B3_FIX( 0.0f );
	b3DistanceJoint_GetSpringForceRange( jointId, &lowerForce, &upperForce );
	ENSURE( lowerForce == -B3_FIX( 50.0f ) && upperForce == B3_FIX( 75.0f ) );

	b3DistanceJoint_SetSpringHertz( jointId, B3_FIX( 4.0f ) );
	ENSURE( b3DistanceJoint_GetSpringHertz( jointId ) == B3_FIX( 4.0f ) );

	b3DistanceJoint_SetSpringDampingRatio( jointId, B3_FIX( 0.6f ) );
	ENSURE( b3DistanceJoint_GetSpringDampingRatio( jointId ) == B3_FIX( 0.6f ) );

	b3DistanceJoint_EnableLimit( jointId, true );
	ENSURE( b3DistanceJoint_IsLimitEnabled( jointId ) == true );

	b3DistanceJoint_SetLengthRange( jointId, B3_FIX( 1.0f ), B3_FIX( 5.0f ) );
	ENSURE( b3DistanceJoint_GetMinLength( jointId ) == B3_FIX( 1.0f ) );
	ENSURE( b3DistanceJoint_GetMaxLength( jointId ) == B3_FIX( 5.0f ) );

	b3Fixed currentLength = b3DistanceJoint_GetCurrentLength( jointId );
	MAYBE_UNUSED( currentLength );

	b3DistanceJoint_EnableMotor( jointId, true );
	ENSURE( b3DistanceJoint_IsMotorEnabled( jointId ) == true );

	b3DistanceJoint_SetMotorSpeed( jointId, B3_FIX( 1.5f ) );
	ENSURE( b3DistanceJoint_GetMotorSpeed( jointId ) == B3_FIX( 1.5f ) );

	b3DistanceJoint_SetMaxMotorForce( jointId, B3_FIX( 25.0f ) );
	ENSURE( b3DistanceJoint_GetMaxMotorForce( jointId ) == B3_FIX( 25.0f ) );

	b3Fixed motorForce = b3DistanceJoint_GetMotorForce( jointId );
	MAYBE_UNUSED( motorForce );

	return FinishJoint( jointId, f.worldId );
}

static int TestFilterJoint( void )
{
	JointFixture f = CreateJointFixture();

	// The filter joint has no type-specific API. It only disables collision and
	// keeps both bodies in the same island.
	b3FilterJointDef def = b3DefaultFilterJointDef();
	def.base.bodyIdA = f.groundId;
	def.base.bodyIdB = f.bodyId;
	b3JointId jointId = b3CreateFilterJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_filterJoint ) != 0 )
	{
		return 1;
	}

	return FinishJoint( jointId, f.worldId );
}

static int TestMotorJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3MotorJointDef def = b3DefaultMotorJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateMotorJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_motorJoint ) != 0 )
	{
		return 1;
	}

	b3Vec3 linearVelocity = { B3_FIX( 1.0f ), B3_FIX( 2.0f ), B3_FIX( 3.0f ) };
	b3MotorJoint_SetLinearVelocity( jointId, linearVelocity );
	b3Vec3 gotLinear = b3MotorJoint_GetLinearVelocity( jointId );
	ENSURE( gotLinear.x == B3_FIX( 1.0f ) && gotLinear.y == B3_FIX( 2.0f ) && gotLinear.z == B3_FIX( 3.0f ) );

	b3Vec3 angularVelocity = { B3_FIX( 0.1f ), B3_FIX( 0.2f ), B3_FIX( 0.3f ) };
	b3MotorJoint_SetAngularVelocity( jointId, angularVelocity );
	b3Vec3 gotAngular = b3MotorJoint_GetAngularVelocity( jointId );
	ENSURE( gotAngular.x == B3_FIX( 0.1f ) && gotAngular.y == B3_FIX( 0.2f ) && gotAngular.z == B3_FIX( 0.3f ) );

	b3MotorJoint_SetMaxVelocityForce( jointId, B3_FIX( 500.0f ) );
	ENSURE( b3MotorJoint_GetMaxVelocityForce( jointId ) == B3_FIX( 500.0f ) );

	b3MotorJoint_SetMaxVelocityTorque( jointId, B3_FIX( 600.0f ) );
	ENSURE( b3MotorJoint_GetMaxVelocityTorque( jointId ) == B3_FIX( 600.0f ) );

	b3MotorJoint_SetLinearHertz( jointId, B3_FIX( 3.0f ) );
	ENSURE( b3MotorJoint_GetLinearHertz( jointId ) == B3_FIX( 3.0f ) );

	b3MotorJoint_SetLinearDampingRatio( jointId, B3_FIX( 0.8f ) );
	ENSURE( b3MotorJoint_GetLinearDampingRatio( jointId ) == B3_FIX( 0.8f ) );

	b3MotorJoint_SetAngularHertz( jointId, B3_FIX( 4.0f ) );
	ENSURE( b3MotorJoint_GetAngularHertz( jointId ) == B3_FIX( 4.0f ) );

	b3MotorJoint_SetAngularDampingRatio( jointId, B3_FIX( 0.9f ) );
	ENSURE( b3MotorJoint_GetAngularDampingRatio( jointId ) == B3_FIX( 0.9f ) );

	b3MotorJoint_SetMaxSpringForce( jointId, B3_FIX( 700.0f ) );
	ENSURE( b3MotorJoint_GetMaxSpringForce( jointId ) == B3_FIX( 700.0f ) );

	b3MotorJoint_SetMaxSpringTorque( jointId, B3_FIX( 800.0f ) );
	ENSURE( b3MotorJoint_GetMaxSpringTorque( jointId ) == B3_FIX( 800.0f ) );

	return FinishJoint( jointId, f.worldId );
}

static int TestMoverJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3MoverJointDef def = b3DefaultMoverJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateMoverJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_moverJoint ) != 0 )
	{
		return 1;
	}

	b3Vec3 velocity = { B3_FIX( 1.0f ), B3_FIX( 2.0f ), -B3_FIX( 3.0f ) };
	b3MoverJoint_SetLinearVelocity( jointId, velocity );
	b3Vec3 gotVelocity = b3MoverJoint_GetLinearVelocity( jointId );
	ENSURE( gotVelocity.x == B3_FIX( 1.0f ) && gotVelocity.y == B3_FIX( 2.0f ) && gotVelocity.z == -B3_FIX( 3.0f ) );

	// The y axis has no force budget, so it must stay at rest while x and z reach the target
	b3Fixed maxForce = B3_FIX( 1000.0f );
	b3MoverJoint_SetMaxVelocityForce( jointId, maxForce );
	ENSURE( b3MoverJoint_GetMaxVelocityForce( jointId ) == B3_FIX( 1000.0f ) );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( f.worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
	}

	b3Vec3 v = b3Body_GetLinearVelocity( f.bodyId );
	ENSURE_SMALL( v.x - B3_FIX( 1.0f ), B3_FIX( 1.0e-4f ) );
	ENSURE_SMALL( v.y, B3_FIX( 1.0e-4f ) );
	ENSURE_SMALL( v.z + B3_FIX( 3.0f ), B3_FIX( 1.0e-4f ) );

	b3Vec3 w = b3Body_GetAngularVelocity( f.bodyId );
	ENSURE( w.x == B3_FIX( 0.0f ) && w.y == B3_FIX( 0.0f ) && w.z == B3_FIX( 0.0f ) );

	return FinishJoint( jointId, f.worldId );
}

// The mover's circular clamp squares h * maxForce. With the B3_FIXED_MAX "unlimited" sentinel that
// square wraps int64 (to exactly zero at h = 1/240), so a 64-bit compare would fire on the first
// nonzero impulse and rescale it UP to h * B3_FIXED_MAX. The 128-bit compare must leave an
// unlimited mover reaching its target like any other.
static int TestMoverJointSentinelForce( void )
{
	JointFixture f = CreateJointFixture();

	// A mass whose inverse is not a power of two. With unit mass the wrapped impulses telescope
	// exactly in modular arithmetic and the velocity lands on the target anyway (measured).
	b3MassData massData = b3Body_GetMassData( f.bodyId );
	massData.mass = B3_FIX( 3.0f );
	b3Body_SetMassData( f.bodyId, massData );

	b3MoverJointDef def = b3DefaultMoverJointDef();
	SetCommonFrames( &def.base, &f );
	def.linearVelocity = (b3Vec3){ B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) };
	def.maxForce = B3_FIXED_MAX;
	b3JointId jointId = b3CreateMoverJoint( f.worldId, &def );

	// Unlimited force reaches the target within the first step and holds it. Checked every step:
	// the wrapped compare blows the impulse up to h * B3_FIXED_MAX and back, and can land near the
	// target again by the end, so a final-state check alone misses it (measured).
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( f.worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );

		b3Vec3 v = b3Body_GetLinearVelocity( f.bodyId );
		ENSURE_SMALL( v.x - B3_FIX( 2.0f ), B3_FIX( 1.0e-4f ) );
		ENSURE_SMALL( v.y, B3_FIX( 1.0e-4f ) );
		ENSURE_SMALL( v.z - B3_FIX( 1.0f ), B3_FIX( 1.0e-4f ) );

		// Holding a constant velocity takes no force after the first step
		b3Fixed force = b3Length( b3Joint_GetConstraintForce( jointId ) );
		ENSURE( force < B3_FIX( 1000.0f ) );
	}

	return FinishJoint( jointId, f.worldId );
}

// A small force budget saturates: the body accelerates at maxForce / mass, and the reported force
// never exceeds the budget. Lowering the budget between steps re-clamps the warm-started impulse.
// The cap is h * maxForce rounded once to the impulse grid and the force reads back as
// impulse * inv_h, so one impulse ULP shows up as inv_h ULP (240 at 60 Hz x 4) of force: the
// slack below is that, not solver error.
static int TestMoverJointForceBudget( void )
{
	JointFixture f = CreateJointFixture();
	b3Fixed mass = b3Body_GetMass( f.bodyId );

	b3MoverJointDef def = b3DefaultMoverJointDef();
	SetCommonFrames( &def.base, &f );
	def.linearVelocity = (b3Vec3){ B3_FIX( 100.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	def.maxForce = B3_FIX( 2.0f );
	b3JointId jointId = b3CreateMoverJoint( f.worldId, &def );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) );
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( f.worldId, timeStep, 4 );
		b3Fixed force = b3Length( b3Joint_GetConstraintForce( jointId ) );
		ENSURE( force <= def.maxForce + B3_FIX( 0.005f ) );
	}

	// One second at 2 N on the measured mass
	b3Vec3 v = b3Body_GetLinearVelocity( f.bodyId );
	ENSURE_SMALL( v.x - b3FixDiv( def.maxForce, mass ), B3_FIX( 0.002f ) );

	b3MoverJoint_SetMaxVelocityForce( jointId, B3_FIX( 0.5f ) );
	b3World_Step( f.worldId, timeStep, 4 );
	ENSURE( b3Length( b3Joint_GetConstraintForce( jointId ) ) <= B3_FIX( 0.5f ) + B3_FIX( 0.005f ) );

	return FinishJoint( jointId, f.worldId );
}

// A box held up by a pogo spring under gravity should settle at the rest length
static int TestPogoJoint( void )
{
	JointFixture f = CreateJointFixture();
	b3World_SetGravity( f.worldId, (b3Vec3){ B3_FIX( 0.0f ), -B3_FIX( 10.0f ), B3_FIX( 0.0f ) } );

	b3Fixed restLength = B3_FIX( 2.0f );
	b3Fixed mass = b3Body_GetMass( f.bodyId );
	b3Fixed weight = b3FixMul( B3_FIX( 10.0f ), mass );

	b3PogoJointDef def = b3DefaultPogoJointDef();
	def.base.bodyIdA = f.groundId;
	def.base.bodyIdB = f.bodyId;
	def.base.localFrameA.p = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	def.base.localFrameB.p = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	def.normal = b3Vec3_axisY;
	def.restLength = restLength;
	def.hertz = B3_FIX( 2.0f );
	def.dampingRatio = B3_FIX( 1.0f );
	def.maxCompressionForce = b3FixMul( B3_FIX( 100.0f ), weight );
	def.maxTensionForce = b3FixMul( B3_FIX( 100.0f ), weight );
	b3JointId jointId = b3CreatePogoJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_pogoJoint ) != 0 )
	{
		return 1;
	}

	b3PogoJoint_SetRestLength( jointId, restLength );
	ENSURE( b3PogoJoint_GetRestLength( jointId ) == restLength );

	b3PogoJoint_SetSpringHertz( jointId, B3_FIX( 3.0f ) );
	ENSURE( b3PogoJoint_GetSpringHertz( jointId ) == B3_FIX( 3.0f ) );

	b3PogoJoint_SetSpringDampingRatio( jointId, B3_FIX( 0.9f ) );
	ENSURE( b3PogoJoint_GetSpringDampingRatio( jointId ) == B3_FIX( 0.9f ) );

	b3Fixed maxTension = b3FixMul( B3_FIX( 50.0f ), weight );
	b3PogoJoint_SetMaxTensionForce( jointId, maxTension );
	ENSURE( b3PogoJoint_GetMaxTensionForce( jointId ) == maxTension );

	b3Fixed maxCompression = b3FixMul( B3_FIX( 60.0f ), weight );
	b3PogoJoint_SetMaxCompressionForce( jointId, maxCompression );
	ENSURE( b3PogoJoint_GetMaxCompressionForce( jointId ) == maxCompression );

	b3Vec3 normal = b3Normalize( (b3Vec3){ B3_FIX( 0.2f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) } );
	b3PogoJoint_SetNormal( jointId, normal );
	b3Vec3 gotNormal = b3PogoJoint_GetNormal( jointId );
	ENSURE( gotNormal.x == normal.x && gotNormal.y == normal.y && gotNormal.z == normal.z );
	b3PogoJoint_SetNormal( jointId, b3Vec3_axisY );

	// Starts stretched by one meter. Positions are exact here, so the length is too.
	ENSURE( b3PogoJoint_GetCurrentLength( jointId ) == B3_FIX( 3.0f ) );

	for ( int i = 0; i < 300; ++i )
	{
		b3World_Step( f.worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
	}

	ENSURE_SMALL( b3PogoJoint_GetCurrentLength( jointId ) - restLength, B3_FIX( 0.01f ) );
	ENSURE_SMALL( b3Body_GetLinearVelocity( f.bodyId ).y, B3_FIX( 0.01f ) );

	// Holding the weight takes an impulse of m * g * h each sub-step
	ENSURE_SMALL( b3PogoJoint_GetImpulse( jointId ) - b3FixDiv( weight, B3_FIX( 240.0f ) ), b3FixMul( B3_FIX( 0.001f ), mass ) );

	b3Vec3 force = b3Joint_GetConstraintForce( jointId );
	ENSURE_SMALL( force.y - weight, b3FixMul( B3_FIX( 0.1f ), mass ) );

	return FinishJoint( jointId, f.worldId );
}

// The pogo clamp is linear, [-h * maxTension, h * maxCompression]. A zero tension budget means the
// spring can only push: stretched far beyond its rest length it must not pull the body down.
static int TestPogoJointTensionClamp( void )
{
	JointFixture f = CreateJointFixture();
	b3Fixed mass = b3Body_GetMass( f.bodyId );

	b3PogoJointDef def = b3DefaultPogoJointDef();
	def.base.bodyIdA = f.groundId;
	def.base.bodyIdB = f.bodyId;
	def.base.localFrameA.p = (b3Vec3){ B3_FIX( 0.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ) };
	def.restLength = B3_FIX( 1.0f );
	def.hertz = B3_FIX( 5.0f );
	def.dampingRatio = B3_FIX( 0.7f );
	def.maxCompressionForce = b3FixMul( B3_FIX( 1000.0f ), mass );
	def.maxTensionForce = B3_FIX( 0.0f );
	b3JointId jointId = b3CreatePogoJoint( f.worldId, &def );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( f.worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
		ENSURE( b3PogoJoint_GetImpulse( jointId ) >= B3_FIX( 0.0f ) );
	}

	// Gravity is off in the fixture, so nothing moved the body
	ENSURE( b3Body_GetLinearVelocity( f.bodyId ).y == B3_FIX( 0.0f ) );

	// Zero hertz disables the spring and clears its state
	b3PogoJoint_SetSpringHertz( jointId, B3_FIX( 0.0f ) );
	b3World_Step( f.worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
	ENSURE( b3PogoJoint_GetImpulse( jointId ) == B3_FIX( 0.0f ) );
	ENSURE( b3PogoJoint_GetCurrentSpeed( jointId ) == B3_FIX( 0.0f ) );

	return FinishJoint( jointId, f.worldId );
}

// b3SpringDamper against a double-precision reference on the same quantized inputs. The tolerance
// is a few ULP: one rounding in omega, one in the stiffness product and one in the divide.
static int TestSpringDamper( void )
{
	b3Fixed h = b3FixDiv( b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), B3_FIX( 4.0f ) );
	const float hertzValues[] = { 0.5f, 3.0f, 10.0f };
	const float positions[] = { -1.0f, 0.1f, 0.003f };
	const float velocities[] = { 0.0f, -2.0f };
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			for ( int k = 0; k < 2; ++k )
			{
				b3Fixed hertz = b3FixFromDouble( hertzValues[i] );
				b3Fixed zeta = B3_FIX( 0.8f );
				b3Fixed x = b3FixFromDouble( positions[j] );
				b3Fixed v = b3FixFromDouble( velocities[k] );
				b3Fixed got = b3SpringDamper( hertz, zeta, x, v, h );

				double omega = 2.0 * 3.14159265358979323846 * b3FixToDouble( hertz );
				double oh = omega * b3FixToDouble( h );
				double ref = ( b3FixToDouble( v ) - omega * oh * b3FixToDouble( x ) ) /
							 ( 1.0 + 2.0 * b3FixToDouble( zeta ) * oh + oh * oh );
				ENSURE_SMALL( got - b3FixFromDouble( ref ), 8 * B3_FIXED_EPSILON );
			}
		}
	}

	// Zero position and velocity stay at rest exactly
	ENSURE( b3SpringDamper( B3_FIX( 3.0f ), B3_FIX( 1.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ), h ) == B3_FIX( 0.0f ) );
	return 0;
}

static int TestPrismaticJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreatePrismaticJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_prismaticJoint ) != 0 )
	{
		return 1;
	}

	b3PrismaticJoint_EnableSpring( jointId, true );
	ENSURE( b3PrismaticJoint_IsSpringEnabled( jointId ) == true );

	b3PrismaticJoint_SetSpringHertz( jointId, B3_FIX( 5.0f ) );
	ENSURE( b3PrismaticJoint_GetSpringHertz( jointId ) == B3_FIX( 5.0f ) );

	b3PrismaticJoint_SetSpringDampingRatio( jointId, B3_FIX( 0.5f ) );
	ENSURE( b3PrismaticJoint_GetSpringDampingRatio( jointId ) == B3_FIX( 0.5f ) );

	b3PrismaticJoint_SetTargetTranslation( jointId, B3_FIX( 1.0f ) );
	ENSURE( b3PrismaticJoint_GetTargetTranslation( jointId ) == B3_FIX( 1.0f ) );

	b3PrismaticJoint_EnableLimit( jointId, true );
	ENSURE( b3PrismaticJoint_IsLimitEnabled( jointId ) == true );

	b3PrismaticJoint_SetLimits( jointId, -B3_FIX( 2.0f ), B3_FIX( 2.0f ) );
	ENSURE( b3PrismaticJoint_GetLowerLimit( jointId ) == -B3_FIX( 2.0f ) );
	ENSURE( b3PrismaticJoint_GetUpperLimit( jointId ) == B3_FIX( 2.0f ) );

	b3PrismaticJoint_EnableMotor( jointId, true );
	ENSURE( b3PrismaticJoint_IsMotorEnabled( jointId ) == true );

	b3PrismaticJoint_SetMotorSpeed( jointId, B3_FIX( 1.5f ) );
	ENSURE( b3PrismaticJoint_GetMotorSpeed( jointId ) == B3_FIX( 1.5f ) );

	b3PrismaticJoint_SetMaxMotorForce( jointId, B3_FIX( 30.0f ) );
	ENSURE( b3PrismaticJoint_GetMaxMotorForce( jointId ) == B3_FIX( 30.0f ) );

	b3Fixed motorForce = b3PrismaticJoint_GetMotorForce( jointId );
	b3Fixed translation = b3PrismaticJoint_GetTranslation( jointId );
	b3Fixed speed = b3PrismaticJoint_GetSpeed( jointId );
	MAYBE_UNUSED( motorForce );
	MAYBE_UNUSED( translation );
	MAYBE_UNUSED( speed );

	return FinishJoint( jointId, f.worldId );
}

static int TestRevoluteJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateRevoluteJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_revoluteJoint ) != 0 )
	{
		return 1;
	}

	b3RevoluteJoint_EnableSpring( jointId, true );
	ENSURE( b3RevoluteJoint_IsSpringEnabled( jointId ) == true );

	b3RevoluteJoint_SetSpringHertz( jointId, B3_FIX( 5.0f ) );
	ENSURE( b3RevoluteJoint_GetSpringHertz( jointId ) == B3_FIX( 5.0f ) );

	b3RevoluteJoint_SetSpringDampingRatio( jointId, B3_FIX( 0.5f ) );
	ENSURE( b3RevoluteJoint_GetSpringDampingRatio( jointId ) == B3_FIX( 0.5f ) );

	b3RevoluteJoint_SetTargetAngle( jointId, B3_FIX( 0.5f ) );
	ENSURE( b3RevoluteJoint_GetTargetAngle( jointId ) == B3_FIX( 0.5f ) );

	b3Fixed angle = b3RevoluteJoint_GetAngle( jointId );
	MAYBE_UNUSED( angle );

	b3RevoluteJoint_EnableLimit( jointId, true );
	ENSURE( b3RevoluteJoint_IsLimitEnabled( jointId ) == true );

	b3RevoluteJoint_SetLimits( jointId, -B3_FIX( 1.0f ), B3_FIX( 1.0f ) );
	ENSURE( b3RevoluteJoint_GetLowerLimit( jointId ) == -B3_FIX( 1.0f ) );
	ENSURE( b3RevoluteJoint_GetUpperLimit( jointId ) == B3_FIX( 1.0f ) );

	b3RevoluteJoint_EnableMotor( jointId, true );
	ENSURE( b3RevoluteJoint_IsMotorEnabled( jointId ) == true );

	b3RevoluteJoint_SetMotorSpeed( jointId, B3_FIX( 2.0f ) );
	ENSURE( b3RevoluteJoint_GetMotorSpeed( jointId ) == B3_FIX( 2.0f ) );

	b3RevoluteJoint_SetMaxMotorTorque( jointId, B3_FIX( 40.0f ) );
	ENSURE( b3RevoluteJoint_GetMaxMotorTorque( jointId ) == B3_FIX( 40.0f ) );

	b3Fixed motorTorque = b3RevoluteJoint_GetMotorTorque( jointId );
	MAYBE_UNUSED( motorTorque );

	return FinishJoint( jointId, f.worldId );
}

static int TestSphericalJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3SphericalJointDef def = b3DefaultSphericalJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateSphericalJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_sphericalJoint ) != 0 )
	{
		return 1;
	}

	b3SphericalJoint_EnableConeLimit( jointId, true );
	ENSURE( b3SphericalJoint_IsConeLimitEnabled( jointId ) == true );

	b3SphericalJoint_SetConeLimit( jointId, B3_FIX( 0.5f ) );
	ENSURE( b3SphericalJoint_GetConeLimit( jointId ) == B3_FIX( 0.5f ) );

	b3Fixed coneAngle = b3SphericalJoint_GetConeAngle( jointId );
	MAYBE_UNUSED( coneAngle );

	b3SphericalJoint_EnableTwistLimit( jointId, true );
	ENSURE( b3SphericalJoint_IsTwistLimitEnabled( jointId ) == true );

	b3SphericalJoint_SetTwistLimits( jointId, -B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	ENSURE( b3SphericalJoint_GetLowerTwistLimit( jointId ) == -B3_FIX( 0.5f ) );
	ENSURE( b3SphericalJoint_GetUpperTwistLimit( jointId ) == B3_FIX( 0.5f ) );

	b3Fixed twistAngle = b3SphericalJoint_GetTwistAngle( jointId );
	MAYBE_UNUSED( twistAngle );

	b3SphericalJoint_EnableSpring( jointId, true );
	ENSURE( b3SphericalJoint_IsSpringEnabled( jointId ) == true );

	b3SphericalJoint_SetSpringHertz( jointId, B3_FIX( 5.0f ) );
	ENSURE( b3SphericalJoint_GetSpringHertz( jointId ) == B3_FIX( 5.0f ) );

	b3SphericalJoint_SetSpringDampingRatio( jointId, B3_FIX( 0.5f ) );
	ENSURE( b3SphericalJoint_GetSpringDampingRatio( jointId ) == B3_FIX( 0.5f ) );

	// 90 degrees about z, a unit quaternion that round-trips through storage
	b3Quat targetRotation = { { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.7071068f ) }, B3_FIX( 0.7071068f ) };
	b3SphericalJoint_SetTargetRotation( jointId, targetRotation );
	b3Quat gotRotation = b3SphericalJoint_GetTargetRotation( jointId );
	ENSURE_SMALL( gotRotation.v.x - targetRotation.v.x, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( gotRotation.v.y - targetRotation.v.y, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( gotRotation.v.z - targetRotation.v.z, 8 * B3_FIXED_EPSILON );
	ENSURE_SMALL( gotRotation.s - targetRotation.s, 8 * B3_FIXED_EPSILON );

	b3SphericalJoint_EnableMotor( jointId, true );
	ENSURE( b3SphericalJoint_IsMotorEnabled( jointId ) == true );

	b3Vec3 motorVelocity = { B3_FIX( 0.1f ), B3_FIX( 0.2f ), B3_FIX( 0.3f ) };
	b3SphericalJoint_SetMotorVelocity( jointId, motorVelocity );
	b3Vec3 gotMotorVelocity = b3SphericalJoint_GetMotorVelocity( jointId );
	ENSURE( gotMotorVelocity.x == B3_FIX( 0.1f ) && gotMotorVelocity.y == B3_FIX( 0.2f ) && gotMotorVelocity.z == B3_FIX( 0.3f ) );

	b3SphericalJoint_SetMaxMotorTorque( jointId, B3_FIX( 50.0f ) );
	ENSURE( b3SphericalJoint_GetMaxMotorTorque( jointId ) == B3_FIX( 50.0f ) );

	b3Vec3 motorTorque = b3SphericalJoint_GetMotorTorque( jointId );
	MAYBE_UNUSED( motorTorque );

	return FinishJoint( jointId, f.worldId );
}

static int TestWeldJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3WeldJointDef def = b3DefaultWeldJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateWeldJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_weldJoint ) != 0 )
	{
		return 1;
	}

	b3WeldJoint_SetLinearHertz( jointId, B3_FIX( 3.0f ) );
	ENSURE( b3WeldJoint_GetLinearHertz( jointId ) == B3_FIX( 3.0f ) );

	b3WeldJoint_SetLinearDampingRatio( jointId, B3_FIX( 0.5f ) );
	ENSURE( b3WeldJoint_GetLinearDampingRatio( jointId ) == B3_FIX( 0.5f ) );

	b3WeldJoint_SetAngularHertz( jointId, B3_FIX( 4.0f ) );
	ENSURE( b3WeldJoint_GetAngularHertz( jointId ) == B3_FIX( 4.0f ) );

	b3WeldJoint_SetAngularDampingRatio( jointId, B3_FIX( 0.7f ) );
	ENSURE( b3WeldJoint_GetAngularDampingRatio( jointId ) == B3_FIX( 0.7f ) );

	return FinishJoint( jointId, f.worldId );
}

static int TestWheelJoint( void )
{
	JointFixture f = CreateJointFixture();

	b3WheelJointDef def = b3DefaultWheelJointDef();
	SetCommonFrames( &def.base, &f );
	b3JointId jointId = b3CreateWheelJoint( f.worldId, &def );

	if ( ExerciseJointBase( jointId, f.worldId, f.groundId, f.bodyId, b3_wheelJoint ) != 0 )
	{
		return 1;
	}

	b3WheelJoint_EnableSuspension( jointId, true );
	ENSURE( b3WheelJoint_IsSuspensionEnabled( jointId ) == true );

	b3WheelJoint_SetSuspensionHertz( jointId, B3_FIX( 5.0f ) );
	ENSURE( b3WheelJoint_GetSuspensionHertz( jointId ) == B3_FIX( 5.0f ) );

	b3WheelJoint_SetSuspensionDampingRatio( jointId, B3_FIX( 0.5f ) );
	ENSURE( b3WheelJoint_GetSuspensionDampingRatio( jointId ) == B3_FIX( 0.5f ) );

	b3WheelJoint_EnableSuspensionLimit( jointId, true );
	ENSURE( b3WheelJoint_IsSuspensionLimitEnabled( jointId ) == true );

	b3WheelJoint_SetSuspensionLimits( jointId, -B3_FIX( 1.0f ), B3_FIX( 1.0f ) );
	ENSURE( b3WheelJoint_GetLowerSuspensionLimit( jointId ) == -B3_FIX( 1.0f ) );
	ENSURE( b3WheelJoint_GetUpperSuspensionLimit( jointId ) == B3_FIX( 1.0f ) );

	b3WheelJoint_EnableSpinMotor( jointId, true );
	ENSURE( b3WheelJoint_IsSpinMotorEnabled( jointId ) == true );

	b3WheelJoint_SetSpinMotorSpeed( jointId, B3_FIX( 6.0f ) );
	ENSURE( b3WheelJoint_GetSpinMotorSpeed( jointId ) == B3_FIX( 6.0f ) );

	b3WheelJoint_SetMaxSpinTorque( jointId, B3_FIX( 35.0f ) );
	ENSURE( b3WheelJoint_GetMaxSpinTorque( jointId ) == B3_FIX( 35.0f ) );

	b3Fixed spinSpeed = b3WheelJoint_GetSpinSpeed( jointId );
	b3Fixed spinTorque = b3WheelJoint_GetSpinTorque( jointId );
	MAYBE_UNUSED( spinSpeed );
	MAYBE_UNUSED( spinTorque );

	b3WheelJoint_EnableSteering( jointId, true );
	ENSURE( b3WheelJoint_IsSteeringEnabled( jointId ) == true );

	b3WheelJoint_SetSteeringHertz( jointId, B3_FIX( 7.0f ) );
	ENSURE( b3WheelJoint_GetSteeringHertz( jointId ) == B3_FIX( 7.0f ) );

	b3WheelJoint_SetSteeringDampingRatio( jointId, B3_FIX( 0.8f ) );
	ENSURE( b3WheelJoint_GetSteeringDampingRatio( jointId ) == B3_FIX( 0.8f ) );

	b3WheelJoint_SetMaxSteeringTorque( jointId, B3_FIX( 45.0f ) );
	ENSURE( b3WheelJoint_GetMaxSteeringTorque( jointId ) == B3_FIX( 45.0f ) );

	b3WheelJoint_EnableSteeringLimit( jointId, true );
	ENSURE( b3WheelJoint_IsSteeringLimitEnabled( jointId ) == true );

	b3WheelJoint_SetSteeringLimits( jointId, -B3_FIX( 0.6f ), B3_FIX( 0.6f ) );
	ENSURE( b3WheelJoint_GetLowerSteeringLimit( jointId ) == -B3_FIX( 0.6f ) );
	ENSURE( b3WheelJoint_GetUpperSteeringLimit( jointId ) == B3_FIX( 0.6f ) );

	b3WheelJoint_SetTargetSteeringAngle( jointId, B3_FIX( 0.25f ) );
	ENSURE( b3WheelJoint_GetTargetSteeringAngle( jointId ) == B3_FIX( 0.25f ) );

	b3Fixed steeringAngle = b3WheelJoint_GetSteeringAngle( jointId );
	b3Fixed steeringTorque = b3WheelJoint_GetSteeringTorque( jointId );
	MAYBE_UNUSED( steeringAngle );
	MAYBE_UNUSED( steeringTorque );

	return FinishJoint( jointId, f.worldId );
}

// Exercise the five 2x2 constraint solves with actual velocity errors. Accessor
// tests with an already-satisfied joint cannot detect an inverse-scale mismatch.
static int TestJointInverseScaleCorrection( void )
{
	const b3JointType types[] = { b3_parallelJoint, b3_prismaticJoint, b3_revoluteJoint, b3_wheelJoint };
	int failures = 0;
	for ( int index = 0; index < ARRAY_COUNT( types ); ++index )
	{
		JointFixture f = CreateJointFixture();
		b3JointType type = types[index];
		b3JointId jointId = b3_nullJointId;
		switch ( type )
		{
			case b3_parallelJoint:
			{
				b3ParallelJointDef def = b3DefaultParallelJointDef();
				SetCommonFrames( &def.base, &f );
				def.hertz = B3_FIX( 5.0f );
				jointId = b3CreateParallelJoint( f.worldId, &def );
			}
			break;
			case b3_prismaticJoint:
			{
				b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
				SetCommonFrames( &def.base, &f );
				jointId = b3CreatePrismaticJoint( f.worldId, &def );
			}
			break;
			case b3_revoluteJoint:
			{
				b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
				SetCommonFrames( &def.base, &f );
				jointId = b3CreateRevoluteJoint( f.worldId, &def );
			}
			break;
			case b3_wheelJoint:
			{
				b3WheelJointDef def = b3DefaultWheelJointDef();
				SetCommonFrames( &def.base, &f );
				def.enableSuspensionSpring = false;
				jointId = b3CreateWheelJoint( f.worldId, &def );
			}
			break;
			default:
				break;
		}

		bool checkLinear = type == b3_prismaticJoint || type == b3_wheelJoint;
		bool checkAngular = type != b3_prismaticJoint;
		if ( checkLinear )
		{
			b3Body_SetLinearVelocity( f.bodyId, (b3Vec3){ 0, B3_FIX( 1.0f ), -B3_FIX( 0.5f ) } );
		}
		if ( checkAngular )
		{
			b3Body_SetAngularVelocity( f.bodyId, (b3Vec3){ B3_FIX( 0.5f ), -B3_FIX( 0.25f ), 0 } );
		}

		for ( int step = 0; step < 30; ++step )
		{
			b3World_Step( f.worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
		}

		b3Vec3 v = b3Body_GetLinearVelocity( f.bodyId );
		b3Vec3 w = b3Body_GetAngularVelocity( f.bodyId );
		bool linearCorrected = !checkLinear || ( b3FixAbs( v.y ) < B3_FIX( 0.025f ) && b3FixAbs( v.z ) < B3_FIX( 0.025f ) );
		bool angularCorrected = !checkAngular || ( b3FixAbs( w.x ) < B3_FIX( 0.025f ) && b3FixAbs( w.y ) < B3_FIX( 0.025f ) );
		if ( !linearCorrected || !angularCorrected )
		{
			printf( "  joint type %d: lateral velocity=(%g,%g), angular velocity=(%g,%g)\n", type, b3FixToDouble( v.y ),
					b3FixToDouble( v.z ), b3FixToDouble( w.x ), b3FixToDouble( w.y ) );
			failures += 1;
		}
		b3DestroyJoint( jointId, true );
		b3DestroyWorld( f.worldId );
	}
	ENSURE( failures == 0 );
	return 0;
}

// Upstream box3d issue #117: a joint created with collideConnected false must clear the
// contacts that already exist between the two bodies. Only b3Joint_SetCollideConnected
// used to do that, so a joint born with the flag off left the touching contact behind and
// the solver kept pushing the bodies apart through the joint.
static int TestCreateJointClearsContacts( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId bodyIdA = b3CreateBody( worldId, &bodyDef );

	// Overlapping so a touching contact forms
	bodyDef.position = (b3Pos){ B3_FIX( 0.5f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId bodyIdB = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	b3BoxHull box = b3MakeBoxHull( B3_FIX( 0.5f ), B3_FIX( 0.5f ), B3_FIX( 0.5f ) );
	b3CreateHullShape( bodyIdA, &shapeDef, &box.base );
	b3CreateHullShape( bodyIdB, &shapeDef, &box.base );

	b3Fixed timeStep = b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) );
	b3World_Step( worldId, timeStep, 4 );
	ENSURE( b3Body_GetContactCapacity( bodyIdA ) > 0 );

	b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
	jointDef.base.bodyIdA = bodyIdA;
	jointDef.base.bodyIdB = bodyIdB;
	jointDef.base.collideConnected = false;
	b3CreateRevoluteJoint( worldId, &jointDef );

	ENSURE( b3Body_GetContactCapacity( bodyIdA ) == 0 );
	ENSURE( b3Body_GetContactCapacity( bodyIdB ) == 0 );

	b3World_Step( worldId, timeStep, 4 );
	ENSURE( b3Body_GetContactCapacity( bodyIdA ) == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestNonDynamicJointDisable( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyIdA = b3CreateBody( worldId, &bodyDef );
	bodyDef.position = (b3Pos){ B3_FIX( 2.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId bodyIdB = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull box = b3MakeCubeHull( B3_FIX( 0.5f ) );
	b3CreateHullShape( bodyIdA, &shapeDef, &box.base );
	b3CreateHullShape( bodyIdB, &shapeDef, &box.base );

	b3WeldJointDef jointDef = b3DefaultWeldJointDef();
	jointDef.base.bodyIdA = bodyIdA;
	jointDef.base.bodyIdB = bodyIdB;
	b3JointId weldId = b3CreateWeldJoint( worldId, &jointDef );

	b3Body_SetType( bodyIdA, b3_staticBody );
	b3Body_SetType( bodyIdB, b3_kinematicBody );
	b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );

	b3Body_Disable( bodyIdB );
	b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
	b3Body_Enable( bodyIdB );
	b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
	ENSURE( b3Joint_IsValid( weldId ) );

	b3Body_SetType( bodyIdB, b3_dynamicBody );
	b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
	ENSURE( b3Joint_IsAwake( weldId ) );

	// Created directly between a static and a kinematic body
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 4.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3BodyId bodyIdC = b3CreateBody( worldId, &bodyDef );
	b3CreateHullShape( bodyIdC, &shapeDef, &box.base );

	b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
	revoluteDef.base.bodyIdA = bodyIdA;
	revoluteDef.base.bodyIdB = bodyIdC;
	b3JointId revoluteId = b3CreateRevoluteJoint( worldId, &revoluteDef );

	b3Body_Disable( bodyIdC );
	b3Body_Enable( bodyIdC );
	b3World_Step( worldId, b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) ), 4 );
	ENSURE( b3Joint_IsAwake( revoluteId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

static double RandUnit( uint64_t* state )
{
	*state = *state * 6364136223846793005ull + 1442695040888963407ull;
	return (double)( *state >> 11 ) * ( 1.0 / 9007199254740992.0 );
}

static double RandRange( uint64_t* state, double lo, double hi )
{
	return lo + ( hi - lo ) * RandUnit( state );
}

static double Norm3( const double v[3] )
{
	return sqrt( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
}

static void QuatToRot( const double q[4], double R[3][3] )
{
	double x = q[0], y = q[1], z = q[2], w = q[3];
	R[0][0] = 1.0 - 2.0 * y * y - 2.0 * z * z;
	R[0][1] = 2.0 * x * y - 2.0 * z * w;
	R[0][2] = 2.0 * x * z + 2.0 * y * w;
	R[1][0] = 2.0 * x * y + 2.0 * z * w;
	R[1][1] = 1.0 - 2.0 * x * x - 2.0 * z * z;
	R[1][2] = 2.0 * y * z - 2.0 * x * w;
	R[2][0] = 2.0 * x * z - 2.0 * y * w;
	R[2][1] = 2.0 * y * z + 2.0 * x * w;
	R[2][2] = 1.0 - 2.0 * x * x - 2.0 * y * y;
}

static void SymPSD( uint64_t* state, double A[3][3] )
{
	double q[4];
	double n = 0.0;
	for ( int k = 0; k < 4; ++k )
	{
		q[k] = RandRange( state, -1.0, 1.0 );
		n += q[k] * q[k];
	}
	double inv = 1.0 / sqrt( n );
	for ( int k = 0; k < 4; ++k )
	{
		q[k] *= inv;
	}

	double R[3][3];
	QuatToRot( q, R );

	double d[3] = { RandRange( state, 0.0, 10.0 ), RandRange( state, 0.0, 10.0 ), RandRange( state, 0.0, 10.0 ) };

	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			double s = 0.0;
			for ( int k = 0; k < 3; ++k )
			{
				s += R[i][k] * d[k] * R[j][k];
			}
			A[i][j] = s;
		}
	}
}

static int Inv3( const double M[3][3], double Minv[3][3] )
{
	double det = M[0][0] * ( M[1][1] * M[2][2] - M[1][2] * M[2][1] )
	           - M[0][1] * ( M[1][0] * M[2][2] - M[1][2] * M[2][0] )
	           + M[0][2] * ( M[1][0] * M[2][1] - M[1][1] * M[2][0] );
	if ( fabs( det ) < 1e-15 )
	{
		return 0;
	}
	double id = 1.0 / det;
	Minv[0][0] = ( M[1][1] * M[2][2] - M[1][2] * M[2][1] ) * id;
	Minv[0][1] = ( M[0][2] * M[2][1] - M[0][1] * M[2][2] ) * id;
	Minv[0][2] = ( M[0][1] * M[1][2] - M[0][2] * M[1][1] ) * id;
	Minv[1][0] = ( M[1][2] * M[2][0] - M[1][0] * M[2][2] ) * id;
	Minv[1][1] = ( M[0][0] * M[2][2] - M[0][2] * M[2][0] ) * id;
	Minv[1][2] = ( M[0][2] * M[1][0] - M[0][0] * M[1][2] ) * id;
	Minv[2][0] = ( M[1][0] * M[2][1] - M[1][1] * M[2][0] ) * id;
	Minv[2][1] = ( M[0][1] * M[2][0] - M[0][0] * M[2][1] ) * id;
	Minv[2][2] = ( M[0][0] * M[1][1] - M[0][1] * M[1][0] ) * id;
	return 1;
}

static double CondInf( const double M[3][3] )
{
	double Minv[3][3];
	if ( !Inv3( M, Minv ) )
	{
		return 1e30;
	}
	double n1 = 0.0, n2 = 0.0;
	for ( int i = 0; i < 3; ++i )
	{
		double rs = fabs( M[i][0] ) + fabs( M[i][1] ) + fabs( M[i][2] );
		double ri = fabs( Minv[i][0] ) + fabs( Minv[i][1] ) + fabs( Minv[i][2] );
		if ( rs > n1 )
		{
			n1 = rs;
		}
		if ( ri > n2 )
		{
			n2 = ri;
		}
	}
	return n1 * n2;
}

static int TestPointConstraintMatchesUpstreamExpansion( void )
{
	const double kEps = 1.0 / 65536.0;
	const int caseCount = 1000;
	const uint64_t seed = 0x9E3779B97F4A7C15ull;

	int skippedTotal = 0;
	double maxKError = 0.0;
	double maxSolveRelError = 0.0;
	double globalKMag = 0.0;
	uint64_t state;

	for ( int pass = 0; pass < 2; ++pass )
	{
		state = seed;
		int skipped = 0;
		for ( int i = 0; i < caseCount; ++i )
		{
			int staticA = ( i % 11 ) == 0;
			double mA = staticA ? 0.0 : RandRange( &state, 0.0, 10.0 );
			double mB = RandRange( &state, 0.0, 10.0 );
			double iA[3][3], iB[3][3];
			SymPSD( &state, iA );
			SymPSD( &state, iB );
			double rA[3] = { RandRange( &state, -5.0, 5.0 ), RandRange( &state, -5.0, 5.0 ), RandRange( &state, -5.0, 5.0 ) };
			double rB[3] = { RandRange( &state, -5.0, 5.0 ), RandRange( &state, -5.0, 5.0 ), RandRange( &state, -5.0, 5.0 ) };
			double rhs[3] = { RandRange( &state, -10.0, 10.0 ), RandRange( &state, -10.0, 10.0 ), RandRange( &state, -10.0, 10.0 ) };

			b3Fixed mA_f = b3FixFromDouble( mA );
			b3Fixed mB_f = b3FixFromDouble( mB );
			b3Vec3 rA_f = { b3FixFromDouble( rA[0] ), b3FixFromDouble( rA[1] ), b3FixFromDouble( rA[2] ) };
			b3Vec3 rB_f = { b3FixFromDouble( rB[0] ), b3FixFromDouble( rB[1] ), b3FixFromDouble( rB[2] ) };
			b3Vec3 rhs_f = { b3FixFromDouble( rhs[0] ), b3FixFromDouble( rhs[1] ), b3FixFromDouble( rhs[2] ) };

			b3Matrix3 iA_f, iB_f;
			iA_f.cx = (b3Vec3){ b3FixFromDouble( iA[0][0] ), b3FixFromDouble( iA[1][0] ), b3FixFromDouble( iA[2][0] ) };
			iA_f.cy = (b3Vec3){ b3FixFromDouble( iA[0][1] ), b3FixFromDouble( iA[1][1] ), b3FixFromDouble( iA[2][1] ) };
			iA_f.cz = (b3Vec3){ b3FixFromDouble( iA[0][2] ), b3FixFromDouble( iA[1][2] ), b3FixFromDouble( iA[2][2] ) };
			iB_f.cx = (b3Vec3){ b3FixFromDouble( iB[0][0] ), b3FixFromDouble( iB[1][0] ), b3FixFromDouble( iB[2][0] ) };
			iB_f.cy = (b3Vec3){ b3FixFromDouble( iB[0][1] ), b3FixFromDouble( iB[1][1] ), b3FixFromDouble( iB[2][1] ) };
			iB_f.cz = (b3Vec3){ b3FixFromDouble( iB[0][2] ), b3FixFromDouble( iB[1][2] ), b3FixFromDouble( iB[2][2] ) };

			b3Matrix3 kA = b3SkewSandwich( iA_f, rA_f );
			b3Matrix3 kB = b3SkewSandwich( iB_f, rB_f );
			b3Matrix3 k = b3NegateMat3( b3AddMM( kA, kB ) );
			k.cx.x += mA_f + mB_f;
			k.cy.y += mA_f + mB_f;
			k.cz.z += mA_f + mB_f;
			b3Vec3 x_f = b3Solve3AcrossScales( k, rhs_f );

			double mA_u = b3FixToDouble( mA_f );
			double mB_u = b3FixToDouble( mB_f );
			double iAcx[3] = { b3FixToDouble( iA_f.cx.x ), b3FixToDouble( iA_f.cx.y ), b3FixToDouble( iA_f.cx.z ) };
			double iAcy[3] = { b3FixToDouble( iA_f.cy.x ), b3FixToDouble( iA_f.cy.y ), b3FixToDouble( iA_f.cy.z ) };
			double iAcz[3] = { b3FixToDouble( iA_f.cz.x ), b3FixToDouble( iA_f.cz.y ), b3FixToDouble( iA_f.cz.z ) };
			double iBcx[3] = { b3FixToDouble( iB_f.cx.x ), b3FixToDouble( iB_f.cx.y ), b3FixToDouble( iB_f.cx.z ) };
			double iBcy[3] = { b3FixToDouble( iB_f.cy.x ), b3FixToDouble( iB_f.cy.y ), b3FixToDouble( iB_f.cy.z ) };
			double iBcz[3] = { b3FixToDouble( iB_f.cz.x ), b3FixToDouble( iB_f.cz.y ), b3FixToDouble( iB_f.cz.z ) };
			double rAu[3] = { b3FixToDouble( rA_f.x ), b3FixToDouble( rA_f.y ), b3FixToDouble( rA_f.z ) };
			double rBu[3] = { b3FixToDouble( rB_f.x ), b3FixToDouble( rB_f.y ), b3FixToDouble( rB_f.z ) };
			double rhsu[3] = { b3FixToDouble( rhs_f.x ), b3FixToDouble( rhs_f.y ), b3FixToDouble( rhs_f.z ) };

			double axy = rAu[2] * iAcy[1] - rAu[1] * iAcz[1];
			double axz = rAu[2] * iAcy[2] - rAu[1] * iAcz[2];
			double ayx = rAu[0] * iAcz[0] - rAu[2] * iAcx[0];
			double ayy = rAu[0] * iAcz[1] - rAu[2] * iAcx[1];
			double ayz = rAu[0] * iAcz[2] - rAu[2] * iAcx[2];
			double azx = rAu[1] * iAcx[0] - rAu[0] * iAcy[0];
			double azy = rAu[1] * iAcx[1] - rAu[0] * iAcy[1];
			double azz = rAu[1] * iAcx[2] - rAu[0] * iAcy[2];

			double bxy = rBu[2] * iBcy[1] - rBu[1] * iBcz[1];
			double bxz = rBu[2] * iBcy[2] - rBu[1] * iBcz[2];
			double byx = rBu[0] * iBcz[0] - rBu[2] * iBcx[0];
			double byy = rBu[0] * iBcz[1] - rBu[2] * iBcx[1];
			double byz = rBu[0] * iBcz[2] - rBu[2] * iBcx[2];
			double bzx = rBu[1] * iBcx[0] - rBu[0] * iBcy[0];
			double bzy = rBu[1] * iBcx[1] - rBu[0] * iBcy[1];
			double bzz = rBu[1] * iBcx[2] - rBu[0] * iBcy[2];

			double m = mA_u + mB_u;
			double kupxx = ( axy * rAu[2] - axz * rAu[1] ) + ( bxy * rBu[2] - bxz * rBu[1] ) + m;
			double kupyy = ( ayz * rAu[0] - ayx * rAu[2] ) + ( byz * rBu[0] - byx * rBu[2] ) + m;
			double kupzz = ( azx * rAu[1] - azy * rAu[0] ) + ( bzx * rBu[1] - bzy * rBu[0] ) + m;
			double kupxy = ( ayy * rAu[2] - ayz * rAu[1] ) + ( byy * rBu[2] - byz * rBu[1] );
			double kupxz = ( azy * rAu[2] - azz * rAu[1] ) + ( bzy * rBu[2] - bzz * rBu[1] );
			double kupyz = ( azz * rAu[0] - azx * rAu[2] ) + ( bzz * rBu[0] - bzx * rBu[2] );

			double Ku[3][3] = {
				{ kupxx, kupxy, kupxz },
				{ kupxy, kupyy, kupyz },
				{ kupxz, kupyz, kupzz }
			};
			double Kf[3][3] = {
				{ b3FixToDouble( k.cx.x ), b3FixToDouble( k.cy.x ), b3FixToDouble( k.cz.x ) },
				{ b3FixToDouble( k.cx.y ), b3FixToDouble( k.cy.y ), b3FixToDouble( k.cz.y ) },
				{ b3FixToDouble( k.cx.z ), b3FixToDouble( k.cy.z ), b3FixToDouble( k.cz.z ) }
			};

			double mag = 0.0;
			for ( int r = 0; r < 3; ++r )
			{
				for ( int c = 0; c < 3; ++c )
				{
					double a = fabs( Kf[r][c] ), b = fabs( Ku[r][c] );
					if ( a > mag )
					{
						mag = a;
					}
					if ( b > mag )
					{
						mag = b;
					}
				}
			}
			if ( mag > globalKMag )
			{
				globalKMag = mag;
			}

			if ( pass == 0 )
			{
				for ( int r = 0; r < 3; ++r )
				{
					for ( int c = 0; c < 3; ++c )
					{
						double e = fabs( Kf[r][c] - Ku[r][c] );
						if ( e > maxKError )
						{
							maxKError = e;
						}
					}
				}
			}
			else
			{
				double kTol = 8.0 * kEps > 1e-6 * globalKMag ? 8.0 * kEps : 1e-6 * globalKMag;
				for ( int r = 0; r < 3; ++r )
				{
					for ( int c = 0; c < 3; ++c )
					{
						double e = fabs( Kf[r][c] - Ku[r][c] );
						if ( e > kTol )
						{
							printf( "condition false: K[%d][%d] err %g > tol %g\n", r, c, e, kTol );
							return 1;
						}
					}
				}
			}

			double ccxx = kupyy * kupzz - kupyz * kupyz;
			double cyy = kupxx * kupzz - kupxz * kupxz;
			double czz = kupxx * kupyy - kupxy * kupxy;
			double cxy = kupxz * kupyz - kupxy * kupzz;
			double cxz = kupxy * kupyz - kupyy * kupxz;
			double cyz = kupxy * kupxz - kupxx * kupyz;
			double det = kupxx * ccxx + kupxy * cxy + kupxz * cxz;

			if ( fabs( det ) < 1e-9 )
			{
				skipped++;
			}
			else
			{
				double xU[3] = {
					( ccxx * rhsu[0] + cxy * rhsu[1] + cxz * rhsu[2] ) / det,
					( cxy * rhsu[0] + cyy * rhsu[1] + cyz * rhsu[2] ) / det,
					( cxz * rhsu[0] + cyz * rhsu[1] + czz * rhsu[2] ) / det
				};
				if ( CondInf( Ku ) < 1e6 )
				{
					double xFM[3] = { b3FixToDouble( x_f.x ), b3FixToDouble( x_f.y ), b3FixToDouble( x_f.z ) };
					double nU = Norm3( xU );
					double rel;
					if ( nU < 1e-12 )
					{
						rel = 0.0;
					}
					else
					{
						double d[3] = { xFM[0] - xU[0], xFM[1] - xU[1], xFM[2] - xU[2] };
						rel = Norm3( d ) / nU;
					}

					if ( pass == 0 )
					{
						if ( rel > maxSolveRelError )
						{
							maxSolveRelError = rel;
						}
					}
					else
					{
						double solveBound = maxSolveRelError * 2.0 > 8.0 * kEps ? maxSolveRelError * 2.0 : 8.0 * kEps;
						if ( rel > solveBound )
						{
							printf( "condition false: solve rel err %g > bound %g\n", rel, solveBound );
							return 1;
						}
					}
				}
			}
		}

		if ( pass == 0 )
		{
			skippedTotal = skipped;
		}
	}

	printf( "PointConstraint cases=%d skipped=%d maxKError=%g maxSolveRelError=%g\n", caseCount, skippedTotal, maxKError, maxSolveRelError );
	return 0;
}

int JointTest( void )
{
	RUN_SUBTEST( TestCreateJointClearsContacts );
	RUN_SUBTEST( TestJointInverseScaleCorrection );
	RUN_SUBTEST( TestParallelJoint );
	RUN_SUBTEST( TestParallelJointSentinelTorqueCap );
	RUN_SUBTEST( TestDistanceJoint );
	RUN_SUBTEST( TestFilterJoint );
	RUN_SUBTEST( TestMotorJoint );
	RUN_SUBTEST( TestMoverJoint );
	RUN_SUBTEST( TestMoverJointSentinelForce );
	RUN_SUBTEST( TestMoverJointForceBudget );
	RUN_SUBTEST( TestPogoJoint );
	RUN_SUBTEST( TestPogoJointTensionClamp );
	RUN_SUBTEST( TestSpringDamper );
	RUN_SUBTEST( TestPrismaticJoint );
	RUN_SUBTEST( TestRevoluteJoint );
	RUN_SUBTEST( TestSphericalJoint );
	RUN_SUBTEST( TestWeldJoint );
	RUN_SUBTEST( TestWheelJoint );
	RUN_SUBTEST( TestNonDynamicJointDisable );
	RUN_SUBTEST( TestPointConstraintMatchesUpstreamExpansion );

	return 0;
}
