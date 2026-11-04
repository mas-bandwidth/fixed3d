// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "inverse.h"
#include "joint.h"
#include "math_internal.h"
#include "physics_world.h"
#include "recording.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

// The accumulated impulse is the joint's only state that survives a step. Its lifecycle (zeroed
// at create, warm started, clamped to h * maxForce on every solve, carried unchanged through sleep,
// wake and snapshot) is modelled in tla/MoverJoints.tla, including a setter lowering maxForce
// between steps.

void b3MoverJoint_SetLinearVelocity( b3JointId jointId, b3Vec3 velocity )
{
	B3_ASSERT( b3IsValidVec3( velocity ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, MoverJointSetLinearVelocity, jointId, velocity );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	joint->moverJoint.linearVelocity = velocity;
}

b3Vec3 b3MoverJoint_GetLinearVelocity( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	return joint->moverJoint.linearVelocity;
}

void b3MoverJoint_SetMaxVelocityForce( b3JointId jointId, b3Fixed maxForce )
{
	B3_ASSERT( b3IsValidFixed( maxForce ) && maxForce >= B3_FIX( 0.0f ) );
	b3World* world = b3GetWorld( jointId.world0 );
	B3_REC( world, MoverJointSetMaxVelocityForce, jointId, maxForce );
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	joint->moverJoint.maxForce = maxForce;
}

b3Fixed b3MoverJoint_GetMaxVelocityForce( b3JointId jointId )
{
	b3JointSim* joint = b3GetJointSimCheckType( jointId, b3_moverJoint );
	return joint->moverJoint.maxForce;
}

b3Vec3 b3GetMoverJointForce( b3World* world, b3JointSim* base )
{
	b3MoverJoint* joint = &base->moverJoint;
	b3Vec3 P = b3Blend2( joint->impulse.x, joint->axisX, joint->impulse.y, joint->axisZ );
	return b3MulSV( world->inv_h, P );
}

void b3PrepareMoverJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_moverJoint );

	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b3World* world = context->world;

	b3Body* bodyA = b3Array_Get( world->bodies, idA );
	b3Body* bodyB = b3Array_Get( world->bodies, idB );

	B3_ASSERT( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet );

	b3SolverSet* setA = b3Array_Get( world->solverSets, bodyA->setIndex );
	b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b3BodySim* bodySimA = b3Array_Get( setA->bodySims, localIndexA );
	b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB );

	// Inverse masses are in the inverse scale (inverse.h). Their sum stays there and the effective
	// mass comes back to the ordinary scale through b3InvReciprocal.
	b3Fixed mA = bodySimA->invMass;
	b3Fixed mB = bodySimB->invMass;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = b3Mat3_zero;
	base->invIB = b3Mat3_zero;

	b3MoverJoint* joint = &base->moverJoint;
	joint->indexA = bodyA->setIndex == b3_awakeSet ? localIndexA : B3_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b3_awakeSet ? localIndexB : B3_NULL_INDEX;

	b3Quat quatB = b3MulQuat( bodySimB->transform.q, base->localFrameB.q );
	b3Matrix3 matrixB = b3MakeMatrixFromQuat( quatB );
	joint->axisX = matrixB.cx;
	joint->axisZ = matrixB.cz;

	b3Fixed k = mA + mB;
	joint->linearMass = k > B3_FIX( 0.0f ) ? b3InvReciprocal( k ) : B3_FIX( 0.0f );

	if ( context->enableWarmStarting == false )
	{
		joint->impulse = (b3Vec2){ 0 };
	}
}

void b3WarmStartMoverJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_moverJoint );

	b3Fixed mA = base->invMassA;
	b3Fixed mB = base->invMassB;

	b3MoverJoint* joint = &base->moverJoint;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 P = b3Blend2( joint->impulse.x, joint->axisX, joint->impulse.y, joint->axisZ );
	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = b3Sub( stateA->linearVelocity, b3InvMulSV( mA, P ) );
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = b3Add( stateB->linearVelocity, b3InvMulSV( mB, P ) );
	}
}

void b3SolveMoverJoint( b3JointSim* base, b3StepContext* context )
{
	B3_ASSERT( base->type == b3_moverJoint );

	b3Fixed mA = base->invMassA;
	b3Fixed mB = base->invMassB;

	// dummy state for static bodies
	b3BodyState dummyState = b3_identityBodyState;

	b3MoverJoint* joint = &base->moverJoint;
	b3BodyState* stateA = joint->indexA == B3_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b3BodyState* stateB = joint->indexB == B3_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b3Vec3 vA = stateA->linearVelocity;
	b3Vec3 vB = stateB->linearVelocity;

	b3Vec3 vr = b3Sub( b3Sub( vB, vA ), joint->linearVelocity );
	b3Vec2 cdot = { b3Dot( vr, joint->axisX ), b3Dot( vr, joint->axisZ ) };
	b3Vec2 deltaImpulse = b3MulSV2( -joint->linearMass, cdot );
	b3Vec2 oldImpulse = joint->impulse;
	b3Vec2 newImpulse = {
		oldImpulse.x + deltaImpulse.x,
		oldImpulse.y + deltaImpulse.y,
	};

	// The circular clamp compares at 128 bits (joint.h): h * maxForce is safe to form, but its
	// square wraps for a large force limit, and a wrapped compare would rescale the impulse up.
	b3Fixed maxImpulse = b3FixMul( context->h, joint->maxForce );
	if ( b3ImpulseOverCap2( newImpulse, maxImpulse ) )
	{
		b3Fixed scale = b3FixDiv( maxImpulse, b3Length2( newImpulse ) );
		newImpulse = (b3Vec2){ b3FixMul( scale, newImpulse.x ), b3FixMul( scale, newImpulse.y ) };
	}

	deltaImpulse = b3Sub2( newImpulse, oldImpulse );
	joint->impulse = newImpulse;

	b3Vec3 P = b3Blend2( deltaImpulse.x, joint->axisX, deltaImpulse.y, joint->axisZ );
	vA = b3Sub( vA, b3InvMulSV( mA, P ) );
	vB = b3Add( vB, b3InvMulSV( mB, P ) );

	if ( stateA->flags & b3_dynamicFlag )
	{
		stateA->linearVelocity = vA;
	}

	if ( stateB->flags & b3_dynamicFlag )
	{
		stateB->linearVelocity = vB;
	}
}

// Integer channel lerp, t in [0, 1]. Debug draw only.
static b3HexColor b3LerpColor( b3HexColor a, b3HexColor b, b3Fixed t )
{
	uint32_t result = 0;
	for ( int shift = 0; shift < 24; shift += 8 )
	{
		int64_t ca = (int64_t)( ( (uint32_t)a >> shift ) & 0xFF );
		int64_t cb = (int64_t)( ( (uint32_t)b >> shift ) & 0xFF );
		int64_t c = ca + ( ( ( cb - ca ) * t + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS );
		result |= (uint32_t)c << shift;
	}
	return (b3HexColor)result;
}

static void b3DrawArrow( b3DebugDraw* draw, b3Pos a, b3Pos b, b3Vec3 planeNormal, b3Fixed size, b3HexColor color )
{
	draw->DrawSegmentFcn( a, b, color, draw->context );

	b3Fixed shaftLen;
	b3Vec3 dir = b3GetLengthAndNormalize( &shaftLen, b3SubPos( b, a ) );
	if ( shaftLen == B3_FIX( 0.0f ) )
	{
		return;
	}

	b3Fixed headLen = size;

	b3Vec3 perp = b3Normalize( b3Cross( planeNormal, dir ) );
	b3Vec3 backFromTip = b3MulSV( -headLen, dir );
	b3Vec3 sideStep = b3MulSV( b3FixMul( headLen, B3_FIX( 0.5f ) ), perp );
	b3Pos tip1 = b3OffsetPos( b, b3Add( backFromTip, sideStep ) );
	b3Pos tip2 = b3OffsetPos( b, b3Sub( backFromTip, sideStep ) );
	draw->DrawSegmentFcn( b, tip1, color, draw->context );
	draw->DrawSegmentFcn( b, tip2, color, draw->context );
}

void b3DrawMoverJoint( b3DebugDraw* draw, b3World* world, b3JointSim* base, b3WorldTransform transformA,
					   b3WorldTransform transformB )
{
	B3_ASSERT( base->type == b3_moverJoint );
	B3_UNUSED( transformA );

	b3MoverJoint* joint = &base->moverJoint;

	b3WorldTransform frameB = b3MulWorldTransforms( transformB, base->localFrameB );
	b3Matrix3 matrixB = b3MakeMatrixFromQuat( frameB.q );
	b3Vec3 axisX = matrixB.cx;
	b3Vec3 axisZ = matrixB.cz;

	b3Body* bodyA = b3Array_Get( world->bodies, base->bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, base->bodyIdB );
	b3BodyState* stateA = b3GetBodyState( world, bodyA );
	b3BodyState* stateB = b3GetBodyState( world, bodyB );
	b3Vec3 vA = stateA != NULL ? stateA->linearVelocity : b3Vec3_zero;
	b3Vec3 vB = stateB != NULL ? stateB->linearVelocity : b3Vec3_zero;

	b3Vec3 velocityError = b3Sub( b3Sub( vB, vA ), joint->linearVelocity );
	b3Vec2 planarVelocityError = { b3Dot( velocityError, axisX ), b3Dot( velocityError, axisZ ) };
	b3Vec2 targetVelocity = { b3Dot( joint->linearVelocity, axisX ), b3Dot( joint->linearVelocity, axisZ ) };

	// Colorize ring by ratio of velocity error magnitude to target speed.
	b3HexColor color = b3_colorWhite;
	b3Fixed targetSpeed = b3Length2( targetVelocity );
	if ( targetSpeed > B3_FIX( 0.0f ) )
	{
		b3Fixed t = b3FixClamp( b3FixDiv( b3Length2( planarVelocityError ), targetSpeed ), B3_FIX( 0.0f ), B3_FIX( 1.0f ) );
		color = b3LerpColor( b3_colorWhite, b3_colorBlue, t );
	}

	b3Fixed radius = draw->jointScale;
	b3Vec3 rx = b3MulSV( radius, axisX );
	b3Vec3 rz = b3MulSV( radius, axisZ );
	const int segmentCount = 32;
	b3CosSin cs = b3ComputeCosSin( b3FixDiv( b3FixMul( B3_FIX( 2.0f ), B3_PI ), b3FixFromInt( segmentCount ) ) );
	b3Fixed x1 = B3_FIX( 1.0f ), y1 = B3_FIX( 0.0f );
	b3Pos p1 = b3OffsetPos( frameB.p, rx );
	for ( int i = 0; i < segmentCount; ++i )
	{
		b3Fixed x2 = b3FixMul( cs.cosine, x1 ) - b3FixMul( cs.sine, y1 );
		b3Fixed y2 = b3FixMul( cs.sine, x1 ) + b3FixMul( cs.cosine, y1 );
		b3Pos p2 = b3OffsetPos( frameB.p, b3Blend2( x2, rx, y2, rz ) );
		draw->DrawSegmentFcn( p1, p2, color, draw->context );
		x1 = x2;
		y1 = y2;
		p1 = p2;
	}

	if ( joint->maxForce > B3_FIX( 0.0f ) )
	{
		// The arrow length is the force as a fraction of the limit, times the ring radius. The
		// ratio impulse * inv_h / maxForce is formed with one rounding so a large force limit does
		// not quantize it to zero.
		b3Vec2 ratio = {
			fixMulDiv( joint->impulse.x, world->inv_h, joint->maxForce ),
			fixMulDiv( joint->impulse.y, world->inv_h, joint->maxForce ),
		};
		b3Vec3 force = b3Blend2( ratio.x, rx, ratio.y, rz );
		b3Vec3 dir = b3Normalize( force );
		b3Pos origin = b3OffsetPos( frameB.p, b3MulSV( -radius, dir ) );
		b3Pos tip = b3OffsetPos( origin, force );
		b3DrawArrow( draw, origin, tip, matrixB.cy, b3FixMul( B3_FIX( 0.1f ), draw->jointScale ), b3_colorWhite );
	}
}
