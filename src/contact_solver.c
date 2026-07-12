// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact_solver.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "math_internal.h"
#include "physics_world.h"
#include "platform.h"
#include "solver_set.h"

#if B3_ENABLE_VALIDATION
#include "shape.h"
#endif

#define FIXED_ANCHORS 1

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

// Prepare a mesh constraints
void b3PrepareContacts_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( prepare_contact, "Prepare Contact", b3_colorYellow, true );

	b3World* world = context->world;
	b3BodySim* bodySims = context->sims;
	b3BodyState* bodyStates = context->states;

	b3Fixed warmStartScale = world->enableWarmStarting ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );

	// Need to use spans in order to find the associated b2Contact, which is per color
	b3ContactPrepareSpan* spans = context->contactPrepareSpans;
	b3ManifoldConstraint* manifoldBase = context->manifoldConstraints;
	b3ContactConstraint* base = context->contactConstraints;

	// Overflow constraints are stored separately
	if ( block.blockType == b3_overflowBlock )
	{
		b3GraphColor* overflow = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
		spans = context->overflowSpans;
		manifoldBase = overflow->manifoldConstraints;
		base = overflow->contactConstraints;
	}

	int index = block.startIndex;
	int endIndex = block.startIndex + block.count;

	// Find color for start index. Linear search but fast.
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= index )
	{
		colorIndex += 1;
	}

	// Loop over block
	while ( index < endIndex )
	{
		int colorStart = spans[colorIndex].start;
		int colorEndIndex = b3MinInt( spans[colorIndex + 1].start, endIndex );
		b3ContactSpec* specs = spans[colorIndex].contacts;

		// Loop over color
		for ( ; index < colorEndIndex; ++index )
		{
			b3ContactConstraint* contactConstraint = base + index;

			int localIndex = index - colorStart;
			B3_ASSERT( 0 <= localIndex && localIndex < spans[colorIndex].count );
			int contactId = specs[localIndex].contactId;
			b3Contact* contact = b3Array_Get( world->contacts, contactId  );
			B3_ASSERT( contact->contactId == contactId );

			int indexA = contact->bodySimIndexA;
			int indexB = contact->bodySimIndexB;

#if B3_ENABLE_VALIDATION
			if ( indexA != B3_NULL_INDEX )
			{
				b3Body* bodyA = b3Array_Get( world->bodies, contact->edges[0].bodyId  );
				B3_ASSERT( indexA == bodyA->localIndex );
			}

			if ( indexB != B3_NULL_INDEX )
			{
				b3Body* bodyB = b3Array_Get( world->bodies, contact->edges[1].bodyId  );
				B3_ASSERT( indexB == bodyB->localIndex );
			}
#endif

			// Body A data
			b3Fixed mA;
			b3Matrix3 iA;
			b3Vec3 vA;
			b3Vec3 wA;

			if ( indexA == B3_NULL_INDEX )
			{
				mA = B3_FIX( 0.0f );
				iA = b3Mat3_zero;
				vA = b3Vec3_zero;
				wA = b3Vec3_zero;
			}
			else
			{
				b3BodySim* simA = bodySims + indexA;
				mA = simA->invMass;
				iA = simA->invInertiaWorld;

				b3BodyState* stateA = bodyStates + indexA;
				vA = stateA->linearVelocity;
				wA = stateA->angularVelocity;
			}

			// Body B data
			b3Fixed mB;
			b3Matrix3 iB;
			b3Vec3 vB;
			b3Vec3 wB;

			if ( indexB == B3_NULL_INDEX )
			{
				mB = B3_FIX( 0.0f );
				iB = b3Mat3_zero;
				vB = b3Vec3_zero;
				wB = b3Vec3_zero;
			}
			else
			{
				b3BodySim* simB = bodySims + indexB;
				mB = simB->invMass;
				iB = simB->invInertiaWorld;

				b3BodyState* stateB = bodyStates + indexB;
				vB = stateB->linearVelocity;
				wB = stateB->angularVelocity;
			}

			int manifoldCount = contact->manifoldCount;
			contactConstraint->contact = contact;
			contactConstraint->manifoldCount = manifoldCount;
			contactConstraint->indexA = indexA;
			contactConstraint->indexB = indexB;
			contactConstraint->invIA = iA;
			contactConstraint->invMassA = mA;
			contactConstraint->invIB = iB;
			contactConstraint->invMassB = mB;
			// The 128-bit matrix inversion is only needed when rolling resistance is active
			contactConstraint->rollingMass =
				contact->rollingResistance > B3_FIX( 0.0f ) ? b3InvertMatrix( b3AddMM( iA, iB ) ) : b3Mat3_zero;
			contactConstraint->softness =
				( contact->flags & b3_contactStaticFlag ) != 0 ? context->staticSoftness : context->contactSoftness;
			contactConstraint->friction = contact->friction;
			contactConstraint->restitution = contact->restitution;
			contactConstraint->rollingResistance = contact->rollingResistance;

			b3ManifoldConstraint* manifoldConstraints = manifoldBase + specs[localIndex].manifoldStart;
			contactConstraint->constraints = manifoldConstraints;

			for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
			{
				b3Manifold* manifold = contact->manifolds + manifoldIndex;
				b3ManifoldConstraint* constraint = manifoldConstraints + manifoldIndex;
				int pointCount = manifold->pointCount;
				b3Vec3 normal = manifold->normal;
				b3Vec3 tangent1 = b3Perp( normal );
				b3Vec3 tangent2 = b3Cross( tangent1, normal );

				constraint->pointCount = pointCount;
				constraint->normal = normal;
				constraint->tangent1 = tangent1;
				constraint->tangent2 = tangent2;

				// Stiffer for static contacts to avoid bodies getting pushed through the ground
				constraint->tangentVelocity1 = b3Dot( contact->tangentVelocity, constraint->tangent1 );
				constraint->tangentVelocity2 = b3Dot( contact->tangentVelocity, constraint->tangent2 );

				b3Vec3 centerA = b3Vec3_zero;
				b3Vec3 centerB = b3Vec3_zero;

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

					// Copy data from manifold point
					b3ManifoldPoint* mp = manifold->points + pointIndex;
					cp->rA = mp->anchorA;
					cp->rB = mp->anchorB;
					cp->baseSeparation = mp->separation - b3Dot( b3Sub( cp->rB, cp->rA ), normal );
					cp->normalImpulse = b3FixMul( warmStartScale , mp->normalImpulse );
					cp->totalNormalImpulse = B3_FIX( 0.0f );

					b3Vec3 rA = cp->rA;
					b3Vec3 rB = cp->rB;

					b3Vec3 rnA = b3Cross( rA, normal );
					b3Vec3 rnB = b3Cross( rB, normal );
					b3Fixed kNormal = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) );
					cp->normalMass = kNormal > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ) , kNormal ) : B3_FIX( 0.0f );

					// Save relative velocity for restitution
					b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
					b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
					cp->relativeVelocity = b3Dot( normal, b3Sub( vrB, vrA ) );

					centerA = b3Add( centerA, rA );
					centerB = b3Add( centerB, rB );
				}

				b3Fixed invCount = b3FixDiv( B3_FIX( 1.0f ) , b3FixFromInt( pointCount ) );
				centerA = b3MulSV( invCount, centerA );
				centerB = b3MulSV( invCount, centerB );
				constraint->originA = centerA;
				constraint->originB = centerB;

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;
					cp->leverArm = b3Distance( cp->rA, centerA );
				}

				b3Vec3 rtA1 = b3Cross( centerA, tangent1 );
				b3Vec3 rtA2 = b3Cross( centerA, tangent2 );
				b3Vec3 rtB1 = b3Cross( centerB, tangent1 );
				b3Vec3 rtB2 = b3Cross( centerB, tangent2 );

				{
					b3Matrix2 k;
					k.cx.x = mA + mB + b3Dot( rtA1, b3MulMV( iA, rtA1 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB1 ) );
					k.cy.y = mA + mB + b3Dot( rtA2, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB2, b3MulMV( iB, rtB2 ) );
					k.cx.y = k.cy.x = b3Dot( rtA1, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB2 ) );

					constraint->tangentMass = b3Invert2( k );
					constraint->frictionImpulse.x = b3FixMul( warmStartScale , b3Dot( manifold->frictionImpulse, tangent1 ) );
					constraint->frictionImpulse.y = b3FixMul( warmStartScale , b3Dot( manifold->frictionImpulse, tangent2 ) );
				}

				{
					b3Fixed k = b3Dot( normal, b3MulMV( b3AddMM( iA, iB ), normal ) );
					constraint->twistMass = k > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ) , k ) : B3_FIX( 0.0f );
					constraint->twistImpulse = b3FixMul( warmStartScale , manifold->twistImpulse );
				}

				{
					constraint->rollingImpulse = b3MulSV( warmStartScale, manifold->rollingImpulse );
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	b3TracyCZoneEnd( prepare_contact );
}

void b3WarmStartContacts_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + block.colorIndex;
	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet  );
	b3BodyState* states = awakeSet->bodyStates.data;
	b3ContactConstraint* constraints = color->contactConstraints;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b3BodyState dummyState = b3_identityBodyState;

	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		const b3ContactConstraint* contactConstraint = constraints + constraintIndex;
		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;

		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;

		b3Fixed mA = contactConstraint->invMassA;
		b3Matrix3 iA = contactConstraint->invIA;
		b3Fixed mB = contactConstraint->invMassB;
		b3Matrix3 iB = contactConstraint->invIB;

		int manifoldCount = contactConstraint->manifoldCount;
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + manifoldIndex;

			// Normal impulses
			b3Vec3 normal = constraint->normal;
			int pointCount = constraint->pointCount;
			for ( int j = 0; j < pointCount; ++j )
			{
				const b3ManifoldConstraintPoint* cp = constraint->points + j;

				// fixed anchors
				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				b3Vec3 impulse = b3MulSV( cp->normalImpulse, normal );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, impulse ) ) );
				vA = b3MulSub( vA, mA, impulse );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, impulse ) ) );
				vB = b3MulAdd( vB, mB, impulse );
			}

			// Central friction
			{
				b3Vec3 rA = constraint->originA;
				b3Vec3 rB = constraint->originB;
				b3Vec3 impulse = b3MulSV( constraint->frictionImpulse.x, constraint->tangent1 );
				impulse = b3Add( impulse, b3MulSV( constraint->frictionImpulse.y, constraint->tangent2 ) );

				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, impulse ) ) );
				vA = b3MulSub( vA, mA, impulse );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, impulse ) ) );
				vB = b3MulAdd( vB, mB, impulse );
			}

			// Central twist friction
			{
				b3Vec3 impulse = b3MulSV( constraint->twistImpulse, constraint->normal );
				wA = b3Sub( wA, b3MulMV( iA, impulse ) );
				wB = b3Add( wB, b3MulMV( iB, impulse ) );
			}

			// Rolling resistance
			{
				b3Vec3 impulse = constraint->rollingImpulse;
				wA = b3Sub( wA, b3MulMV( iA, impulse ) );
				wB = b3Add( wB, b3MulMV( iB, impulse ) );
			}
		}

		if ( stateA->flags & b3_dynamicFlag )
		{
			stateA->linearVelocity = vA;
			stateA->angularVelocity = wA;
		}

		if ( stateB->flags & b3_dynamicFlag )
		{
			stateB->linearVelocity = vB;
			stateB->angularVelocity = wB;
		}
	}
}

// Merged normal and friction loops. This is much more stable for the Jenga stack.
void b3SolveContacts_Mesh( b3SolverBlock block, b3StepContext* context, bool useBias )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + block.colorIndex;
	b3ContactConstraint* contactConstraints = color->contactConstraints;
	b3BodyState* states = context->states;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b3BodyState dummyState = b3_identityBodyState;

	// The last block might not be full
	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	b3Fixed inv_h = context->inv_h;
	const b3Fixed contactSpeed = context->world->contactSpeed;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b3ContactConstraint* contactConstraint = contactConstraints + i;
		int manifoldCount = contactConstraint->manifoldCount;

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		b3Fixed mA = contactConstraint->invMassA;
		b3Matrix3 iA = contactConstraint->invIA;
		b3Fixed mB = contactConstraint->invMassB;
		b3Matrix3 iB = contactConstraint->invIB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Quat dqA = stateA->deltaRotation;

		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;
		b3Quat dqB = stateB->deltaRotation;

		b3Vec3 dp = b3Sub( stateB->deltaPosition, stateA->deltaPosition );
		b3Softness softness = contactConstraint->softness;
		b3Fixed friction = contactConstraint->friction;
		b3Fixed rollingResistance = contactConstraint->rollingResistance;

		for ( int j = 0; j < manifoldCount; ++j )
		{
			b3ManifoldConstraint* constraint = contactConstraint->constraints + j;

			int pointCount = constraint->pointCount;
			b3Vec3 normal = constraint->normal;

			b3Fixed totalNormalImpulse = B3_FIX( 0.0f );
			b3Fixed totalTwistLimit = B3_FIX( 0.0f );

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

				// Fixed anchor points for applying impulses
				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				b3Vec3 ds = b3Add( dp, b3Sub( b3RotateVector( dqB, rB ), b3RotateVector( dqA, rA ) ) );
				b3Fixed s = b3Dot( ds, normal ) + cp->baseSeparation;

				b3Fixed velocityBias = B3_FIX( 0.0f );
				b3Fixed massScale = B3_FIX( 1.0f );
				b3Fixed impulseScale = B3_FIX( 0.0f );
				if ( s > B3_FIX( 0.0f ) )
				{
					// speculative bias
					velocityBias = b3FixMul( s , inv_h );
				}
				else if ( useBias )
				{
					velocityBias = b3FixMax( b3FixMul( b3FixMul( softness.massScale , softness.biasRate ) , s ), -contactSpeed );
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				// relative normal velocity at contact
				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				b3Fixed vn = b3Dot( b3Sub( vrB, vrA ), normal );

				// incremental normal impulse
				b3Fixed deltaImpulse = b3FixMul( -cp->normalMass , ( b3FixMul( massScale , vn ) + velocityBias ) ) - b3FixMul( impulseScale , cp->normalImpulse );

				// clamp the accumulated impulse
				b3Fixed newImpulse = b3FixMax( cp->normalImpulse + deltaImpulse, B3_FIX( 0.0f ) );
				deltaImpulse = newImpulse - cp->normalImpulse;
				cp->normalImpulse = newImpulse;
				cp->totalNormalImpulse += newImpulse;

				totalNormalImpulse += newImpulse;
				totalTwistLimit += b3FixMul( cp->leverArm , cp->normalImpulse );

				// apply normal impulse
				b3Vec3 P = b3MulSV( deltaImpulse, normal );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );

				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
			}

			// No friction when applying bias
			if ( useBias == true )
			{
				// Go to next manifold
				continue;
			}

			// Central twist friction
			{
				b3Fixed twistSpeed = b3Dot( constraint->normal, b3Sub( wB, wA ) );
				b3Fixed maxImpulse = b3FixMul( friction , totalTwistLimit );
				b3Fixed deltaImpulse = b3FixMul( -constraint->twistMass , twistSpeed );
				b3Fixed oldImpulse = constraint->twistImpulse;
				constraint->twistImpulse = b3FixClamp( oldImpulse + deltaImpulse, -maxImpulse, maxImpulse );
				deltaImpulse = constraint->twistImpulse - oldImpulse;

				wA = b3Sub( wA, b3MulMV( iA, b3MulSV( deltaImpulse, constraint->normal ) ) );
				wB = b3Add( wB, b3MulMV( iB, b3MulSV( deltaImpulse, constraint->normal ) ) );
			}

			// Rolling resistance
			if ( rollingResistance > B3_FIX( 0.0f ) )
			{
				b3Vec3 deltaImpulse = b3Neg( b3MulMV( contactConstraint->rollingMass, b3Sub( wB, wA ) ) );
				b3Vec3 oldImpulse = constraint->rollingImpulse;
				constraint->rollingImpulse = b3Add( oldImpulse, deltaImpulse );

				b3Fixed maxImpulse = b3FixMul( rollingResistance , totalNormalImpulse );
				b3Fixed magSqr = b3Dot( constraint->rollingImpulse, constraint->rollingImpulse );
				if ( magSqr > b3FixMul( maxImpulse , maxImpulse ) + B3_FIXED_EPSILON )
				{
					constraint->rollingImpulse = b3MulSV( b3FixDiv( maxImpulse , b3FixSqrt( magSqr ) ), constraint->rollingImpulse );
				}

				deltaImpulse = b3Sub( constraint->rollingImpulse, oldImpulse );

				wA = b3Sub( wA, b3MulMV( iA, deltaImpulse ) );
				wB = b3Add( wB, b3MulMV( iB, deltaImpulse ) );
			}

			// Central friction
			{
				b3Vec3 tangent1 = constraint->tangent1;
				b3Vec3 tangent2 = constraint->tangent2;

				// Fixed anchor points for applying impulses
				b3Vec3 rA = constraint->originA;
				b3Vec3 rB = constraint->originB;

				// Relative tangent velocity at contact
				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				b3Vec3 vr = b3Sub( vrB, vrA );
				b3Vec2 vt = {
					b3Dot( vr, tangent1 ) - constraint->tangentVelocity1,
					b3Dot( vr, tangent2 ) - constraint->tangentVelocity2,
				};

				// Incremental tangent impulse
				b3Vec2 tm = b3MulMV2( constraint->tangentMass, vt );
				b3Vec2 deltaImpulse = { -tm.x, -tm.y };
				b3Vec2 newImpulse = {
					constraint->frictionImpulse.x + deltaImpulse.x,
					constraint->frictionImpulse.y + deltaImpulse.y,
				};

				b3Fixed maxImpulse = b3FixMul( friction , totalNormalImpulse );

				// Clamp the accumulated impulse
				b3Fixed lengthSquared = b3Dot2( newImpulse, newImpulse );
				if ( lengthSquared > b3FixMul( maxImpulse , maxImpulse ) )
				{
					b3Fixed scale = b3FixDiv( maxImpulse , b3FixSqrt( lengthSquared ) );
					newImpulse.x = b3FixMul( newImpulse.x, scale );
					newImpulse.y = b3FixMul( newImpulse.y, scale );
				}
				deltaImpulse = b3Sub2( newImpulse, constraint->frictionImpulse );
				constraint->frictionImpulse = newImpulse;

				// Apply delta impulse
				b3Vec3 P = b3Blend2( deltaImpulse.x, tangent1, deltaImpulse.y, tangent2 );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );
				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
			}
		}

		if ( stateA->flags & b3_dynamicFlag )
		{
			stateA->linearVelocity = vA;
			stateA->angularVelocity = wA;
		}

		if ( stateB->flags & b3_dynamicFlag )
		{
			stateB->linearVelocity = vB;
			stateB->angularVelocity = wB;
		}
	}
}

void b3ApplyRestitution_Mesh( b3SolverBlock block, b3StepContext* context )
{
	b3World* world = context->world;
	b3GraphColor* color = world->constraintGraph.colors + block.colorIndex;
	b3BodyState* states = context->states;
	b3ContactConstraint* constraints = color->contactConstraints;

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	b3BodyState dummyState = b3_identityBodyState;

	int startIndex = block.startIndex;
	int endIndex = startIndex + block.count;

	b3Fixed threshold = context->world->restitutionThreshold;

	for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	{
		const b3ContactConstraint* contactConstraint = constraints + constraintIndex;
		b3Fixed restitution = contactConstraint->restitution;
		if ( restitution == B3_FIX( 0.0f ) )
		{
			continue;
		}

		int indexA = contactConstraint->indexA;
		int indexB = contactConstraint->indexB;

		b3BodyState* stateA = indexA == B3_NULL_INDEX ? &dummyState : states + indexA;
		b3BodyState* stateB = indexB == B3_NULL_INDEX ? &dummyState : states + indexB;

		b3Vec3 vA = stateA->linearVelocity;
		b3Vec3 wA = stateA->angularVelocity;
		b3Vec3 vB = stateB->linearVelocity;
		b3Vec3 wB = stateB->angularVelocity;

		b3Fixed mA = contactConstraint->invMassA;
		b3Matrix3 iA = contactConstraint->invIA;
		b3Fixed mB = contactConstraint->invMassB;
		b3Matrix3 iB = contactConstraint->invIB;

		int manifoldCount = contactConstraint->manifoldCount;
		for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraint* cm = contactConstraint->constraints + manifoldIndex;

			b3Vec3 normal = cm->normal;
			int pointCount = cm->pointCount;
			B3_ASSERT( 0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS );

			for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
			{
				b3ManifoldConstraintPoint* cp = cm->points + pointIndex;

				// If the total normal impulse is zero then there was no collision
				// this skips speculative contact points that didn't generate an impulse
				// The max normal impulse is used in case there was a collision that moved away within the sub-step
				// process
				if ( cp->relativeVelocity > -threshold || cp->totalNormalImpulse == B3_FIX( 0.0f ) )
				{
					continue;
				}

				// fixed anchor points
				b3Vec3 rA = cp->rA;
				b3Vec3 rB = cp->rB;

				// relative normal velocity at contact
				b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
				b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
				b3Fixed vn = b3Dot( b3Sub( vrB, vrA ), normal );

				// compute normal impulse
				b3Fixed impulse = b3FixMul( -cp->normalMass , ( vn + b3FixMul( restitution , cp->relativeVelocity ) ) );

				// clamp the accumulated impulse
				b3Fixed newImpulse = b3FixMax( cp->normalImpulse + impulse, B3_FIX( 0.0f ) );
				impulse = newImpulse - cp->normalImpulse;
				cp->normalImpulse = newImpulse;
				cp->totalNormalImpulse += impulse;

				// apply contact impulse
				b3Vec3 P = b3MulSV( impulse, normal );
				vA = b3MulSub( vA, mA, P );
				wA = b3Sub( wA, b3MulMV( iA, b3Cross( rA, P ) ) );
				vB = b3MulAdd( vB, mB, P );
				wB = b3Add( wB, b3MulMV( iB, b3Cross( rB, P ) ) );
			}

			if ( stateA->flags & b3_dynamicFlag )
			{
				stateA->linearVelocity = vA;
				stateA->angularVelocity = wA;
			}

			if ( stateB->flags & b3_dynamicFlag )
			{
				stateB->linearVelocity = vB;
				stateB->angularVelocity = wB;
			}
		}
	}
}

// Don't need to use spans for colors for this because the constraint to contact association
// is already linked by pointer.
void b3StoreImpulses_Mesh( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
	b3World* world = context->world;

	// Mirror b3PrepareContacts_Mesh: the per-color flat arrays and the overflow color
	// each have their own (base, spans, manifoldBase).
	b3ContactPrepareSpan* spans = context->contactPrepareSpans;
	b3ContactConstraint* base = context->contactConstraints;

	if ( block.blockType == b3_overflowBlock )
	{
		b3GraphColor* overflow = world->constraintGraph.colors + B3_OVERFLOW_INDEX;
		spans = context->overflowSpans;
		base = overflow->contactConstraints;
	}

	b3TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b3BitSet* hitEventBitSet = &taskContext->hitEventBitSet;
	bool hasHitEvents = taskContext->hasHitEvents;
	b3Fixed negHitThreshold = -world->hitEventThreshold;

	int index = block.startIndex;
	int endIndex = block.startIndex + block.count;

	// Find color for start index. Linear search but fast.
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= index )
	{
		colorIndex += 1;
	}

	// Loop over block
	while ( index < endIndex )
	{
		int colorStart = spans[colorIndex].start;
		int colorEndIndex = b3MinInt( spans[colorIndex + 1].start, endIndex );

		// Loop over color
		for ( ; index < colorEndIndex; ++index )
		{
			b3ContactConstraint* contactConstraint = base + index;

			int localIndex = index - colorStart;
			B3_UNUSED( localIndex );
			B3_ASSERT( 0 <= localIndex && localIndex < spans[colorIndex].count );

			// Having this contact pointer simplifies impulse storage
			b3Contact* contact = contactConstraint->contact;
			B3_ASSERT( contact != NULL );

			// Catches the wrong-(base, spans) pairing: the contact pointer stashed by
			// b3PrepareContacts_Mesh at this flat slot must reference the same contact
			// the span at this slot describes.
			B3_VALIDATE( contact->contactId == spans[colorIndex].contacts[localIndex].contactId );

			int manifoldCount = contactConstraint->manifoldCount;
			B3_ASSERT( manifoldCount == contact->manifoldCount );

			bool checkHitEvents = ( contact->flags & b3_simEnableHitEvent ) != 0;
			bool flagged = false;

			for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
			{
				b3Manifold* manifold = contact->manifolds + manifoldIndex;
				b3ManifoldConstraint* constraint = contactConstraint->constraints + manifoldIndex;
				manifold->twistImpulse = constraint->twistImpulse;
				manifold->frictionImpulse = b3Blend2( constraint->frictionImpulse.x, constraint->tangent1,
													  constraint->frictionImpulse.y, constraint->tangent2 );
				manifold->rollingImpulse = constraint->rollingImpulse;

				int count = constraint->pointCount;
				B3_ASSERT( count == manifold->pointCount );
				for ( int pointIndex = 0; pointIndex < count; ++pointIndex )
				{
					b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;
					b3ManifoldPoint* mp = manifold->points + pointIndex;
					mp->normalImpulse = cp->normalImpulse;
					mp->totalNormalImpulse = cp->totalNormalImpulse;
					mp->normalVelocity = cp->relativeVelocity;

					if ( checkHitEvents && flagged == false &&
						 mp->normalVelocity < negHitThreshold && mp->totalNormalImpulse > B3_FIX( 0.0f ) )
					{
						b3SetBit( hitEventBitSet, contact->contactId );
						hasHitEvents = true;
						flagged = true;
					}
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	taskContext->hasHitEvents = hasHitEvents;
}


// scalar math
typedef struct b3FloatW
{
	b3Fixed x, y, z, w;
} b3FloatW;


// Wide vec2
typedef struct b3Vec2W
{
	b3FloatW x, y;
} b3Vec2W;

// Wide vec3
typedef struct b3Vec3W
{
	b3FloatW X, Y, Z;
} b3Vec3W;

// Wide quaternion
typedef struct b3QuatW
{
	b3Vec3W V;
	b3FloatW S;
} b3QuatW;

// Wide symmetric matrix2
typedef struct b3SymMatrix2W
{
	b3FloatW cxx, cxy, cyy;
} b3SymMatrix2W;

// Wide symmetric matrix3
typedef struct b3SymMatrix3W
{
	b3FloatW cxx, cxy, cxz, cyy, cyz, czz;
} b3SymMatrix3W;


static inline b3FloatW b3ZeroW( void )
{
	return (b3FloatW){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
}

static inline b3FloatW b3SplatW( b3Fixed scalar )
{
	return (b3FloatW){ scalar, scalar, scalar, scalar };
}

static inline b3FloatW b3NegW( b3FloatW a )
{
	return (b3FloatW){ -a.x, -a.y, -a.z, -a.w };
}

static inline b3FloatW b3AddW( b3FloatW a, b3FloatW b )
{
	return (b3FloatW){ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

static inline b3FloatW b3SubW( b3FloatW a, b3FloatW b )
{
	return (b3FloatW){ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

static inline b3FloatW b3MulW( b3FloatW a, b3FloatW b )
{
	return (b3FloatW){ b3FixMul( a.x , b.x ), b3FixMul( a.y , b.y ), b3FixMul( a.z , b.z ), b3FixMul( a.w , b.w ) };
}

static inline b3FloatW b3DivW( b3FloatW a, b3FloatW b )
{
	return (b3FloatW){ b3FixDiv( a.x , b.x ), b3FixDiv( a.y , b.y ), b3FixDiv( a.z , b.z ), b3FixDiv( a.w , b.w ) };
}

static inline b3FloatW b3SqrtW( b3FloatW a )
{
	return (b3FloatW){ b3FixSqrt( a.x ), b3FixSqrt( a.y ), b3FixSqrt( a.z ), b3FixSqrt( a.w ) };
}

static inline b3FloatW b3MulAddW( b3FloatW a, b3FloatW b, b3FloatW c )
{
	return (b3FloatW){ a.x + b3FixMul( b.x , c.x ), a.y + b3FixMul( b.y , c.y ), a.z + b3FixMul( b.z , c.z ), a.w + b3FixMul( b.w , c.w ) };
}

// static inline b3FloatW b3MulSubW( b3FloatW a, b3FloatW b, b3FloatW c )
//{
//	return { a.x - b.x * c.x, a.y - b.y * c.y, a.z - b.z * c.z, a.w - b.w * c.w };
// }

// static inline b3FloatW b3MinW( b3FloatW a, b3FloatW b )
//{
//	b3FloatW r;
//	r.x = a.x <= b.x ? a.x : b.x;
//	r.y = a.y <= b.y ? a.y : b.y;
//	r.z = a.z <= b.z ? a.z : b.z;
//	r.w = a.w <= b.w ? a.w : b.w;
//	return r;
// }

static inline b3FloatW b3MaxW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
}

// clamp a to [-b, b]
static inline b3FloatW b3SymClampW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	r.x = r.x <= -b.x ? -b.x : r.x;
	r.y = r.y <= -b.y ? -b.y : r.y;
	r.z = r.z <= -b.z ? -b.z : r.z;
	r.w = r.w <= -b.w ? -b.w : r.w;
	return r;
}

static inline b3FloatW b3OrW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x != B3_FIX( 0.0f ) || b.x != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.y = a.y != B3_FIX( 0.0f ) || b.y != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.z = a.z != B3_FIX( 0.0f ) || b.z != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.w = a.w != B3_FIX( 0.0f ) || b.w != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	return r;
}

static inline b3FloatW b3GreaterThanW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x > b.x ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.y = a.y > b.y ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.z = a.z > b.z ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.w = a.w > b.w ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	return r;
}

static inline b3FloatW b3EqualsW( b3FloatW a, b3FloatW b )
{
	b3FloatW r;
	r.x = a.x == b.x ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.y = a.y == b.y ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.z = a.z == b.z ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.w = a.w == b.w ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	return r;
}

static inline bool b3AllZeroW( b3FloatW a )
{
	return a.x == B3_FIX( 0.0f ) && a.y == B3_FIX( 0.0f ) && a.z == B3_FIX( 0.0f ) && a.w == B3_FIX( 0.0f );
}

// component-wise returns mask ? b : a
static inline b3FloatW b3BlendW( b3FloatW a, b3FloatW b, b3FloatW mask )
{
	b3FloatW r;
	r.x = mask.x != B3_FIX( 0.0f ) ? b.x : a.x;
	r.y = mask.y != B3_FIX( 0.0f ) ? b.y : a.y;
	r.z = mask.z != B3_FIX( 0.0f ) ? b.z : a.z;
	r.w = mask.w != B3_FIX( 0.0f ) ? b.w : a.w;
	return r;
}


// Per-lane 128-bit product reductions with a single rounding (divide last).
// Cheaper than chaining b3FixMul (one rounding and saturation check instead of
// one per product) and sub-resolution products don't quantize to zero.

// a*b + c*d
static inline b3FloatW b3Dot2W( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
	return (b3FloatW){
		b3FixFromDotRaw( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x ),
		b3FixFromDotRaw( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y ),
		b3FixFromDotRaw( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z ),
		b3FixFromDotRaw( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w ),
	};
}

// a*b + c*d + e*f
static inline b3FloatW b3Dot3W( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d, b3FloatW e, b3FloatW f )
{
	return (b3FloatW){
		b3FixFromDotRaw( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x + (b3Int128)e.x * f.x ),
		b3FixFromDotRaw( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y + (b3Int128)e.y * f.y ),
		b3FixFromDotRaw( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z + (b3Int128)e.z * f.z ),
		b3FixFromDotRaw( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w + (b3Int128)e.w * f.w ),
	};
}

// acc + a*b + c*d + e*f
static inline b3FloatW b3AddDot3W( b3FloatW acc, b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d, b3FloatW e, b3FloatW f )
{
	return (b3FloatW){
		b3FixFromDotRaw( ( (b3Int128)acc.x << B3_FIXED_FRACTION_BITS ) + (b3Int128)a.x * b.x + (b3Int128)c.x * d.x + (b3Int128)e.x * f.x ),
		b3FixFromDotRaw( ( (b3Int128)acc.y << B3_FIXED_FRACTION_BITS ) + (b3Int128)a.y * b.y + (b3Int128)c.y * d.y + (b3Int128)e.y * f.y ),
		b3FixFromDotRaw( ( (b3Int128)acc.z << B3_FIXED_FRACTION_BITS ) + (b3Int128)a.z * b.z + (b3Int128)c.z * d.z + (b3Int128)e.z * f.z ),
		b3FixFromDotRaw( ( (b3Int128)acc.w << B3_FIXED_FRACTION_BITS ) + (b3Int128)a.w * b.w + (b3Int128)c.w * d.w + (b3Int128)e.w * f.w ),
	};
}

// acc - (a*b + c*d + e*f)
static inline b3FloatW b3SubDot3W( b3FloatW acc, b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d, b3FloatW e, b3FloatW f )
{
	return (b3FloatW){
		b3FixFromDotRaw( ( (b3Int128)acc.x << B3_FIXED_FRACTION_BITS ) - (b3Int128)a.x * b.x - (b3Int128)c.x * d.x - (b3Int128)e.x * f.x ),
		b3FixFromDotRaw( ( (b3Int128)acc.y << B3_FIXED_FRACTION_BITS ) - (b3Int128)a.y * b.y - (b3Int128)c.y * d.y - (b3Int128)e.y * f.y ),
		b3FixFromDotRaw( ( (b3Int128)acc.z << B3_FIXED_FRACTION_BITS ) - (b3Int128)a.z * b.z - (b3Int128)c.z * d.z - (b3Int128)e.z * f.z ),
		b3FixFromDotRaw( ( (b3Int128)acc.w << B3_FIXED_FRACTION_BITS ) - (b3Int128)a.w * b.w - (b3Int128)c.w * d.w - (b3Int128)e.w * f.w ),
	};
}

// Relative velocity along an axis using precomputed cross(r, axis) rows:
// dot(dv, axis) + dot(wB, rbxa) - dot(wA, raxa), nine products accumulated
// at 128 bits with a single rounding.
static inline b3FloatW b3RelVelocityW( b3Vec3W dv, b3Vec3W axis, b3Vec3W wB, b3Vec3W rbxa, b3Vec3W wA, b3Vec3W raxa )
{
	return (b3FloatW){
		b3FixFromDotRaw( (b3Int128)dv.X.x * axis.X.x + (b3Int128)dv.Y.x * axis.Y.x + (b3Int128)dv.Z.x * axis.Z.x +
						 (b3Int128)wB.X.x * rbxa.X.x + (b3Int128)wB.Y.x * rbxa.Y.x + (b3Int128)wB.Z.x * rbxa.Z.x -
						 (b3Int128)wA.X.x * raxa.X.x - (b3Int128)wA.Y.x * raxa.Y.x - (b3Int128)wA.Z.x * raxa.Z.x ),
		b3FixFromDotRaw( (b3Int128)dv.X.y * axis.X.y + (b3Int128)dv.Y.y * axis.Y.y + (b3Int128)dv.Z.y * axis.Z.y +
						 (b3Int128)wB.X.y * rbxa.X.y + (b3Int128)wB.Y.y * rbxa.Y.y + (b3Int128)wB.Z.y * rbxa.Z.y -
						 (b3Int128)wA.X.y * raxa.X.y - (b3Int128)wA.Y.y * raxa.Y.y - (b3Int128)wA.Z.y * raxa.Z.y ),
		b3FixFromDotRaw( (b3Int128)dv.X.z * axis.X.z + (b3Int128)dv.Y.z * axis.Y.z + (b3Int128)dv.Z.z * axis.Z.z +
						 (b3Int128)wB.X.z * rbxa.X.z + (b3Int128)wB.Y.z * rbxa.Y.z + (b3Int128)wB.Z.z * rbxa.Z.z -
						 (b3Int128)wA.X.z * raxa.X.z - (b3Int128)wA.Y.z * raxa.Y.z - (b3Int128)wA.Z.z * raxa.Z.z ),
		b3FixFromDotRaw( (b3Int128)dv.X.w * axis.X.w + (b3Int128)dv.Y.w * axis.Y.w + (b3Int128)dv.Z.w * axis.Z.w +
						 (b3Int128)wB.X.w * rbxa.X.w + (b3Int128)wB.Y.w * rbxa.Y.w + (b3Int128)wB.Z.w * rbxa.Z.w -
						 (b3Int128)wA.X.w * raxa.X.w - (b3Int128)wA.Y.w * raxa.Y.w - (b3Int128)wA.Z.w * raxa.Z.w ),
	};
}

// s * a
static inline b3Vec3W b3MulSVW( b3FloatW s, b3Vec3W a )
{
	return (b3Vec3W){ b3MulW( s, a.X ), b3MulW( s, a.Y ), b3MulW( s, a.Z ) };
}

// a - s * b
static inline b3Vec3W b3MulSubSVW( b3Vec3W a, b3FloatW s, b3Vec3W b )
{
	return (b3Vec3W){ b3SubW( a.X, b3MulW( s, b.X ) ), b3SubW( a.Y, b3MulW( s, b.Y ) ), b3SubW( a.Z, b3MulW( s, b.Z ) ) };
}

// a + s * b
static inline b3Vec3W b3MulAddSVW( b3Vec3W a, b3FloatW s, b3Vec3W b )
{
	return (b3Vec3W){ b3AddW( a.X, b3MulW( s, b.X ) ), b3AddW( a.Y, b3MulW( s, b.Y ) ), b3AddW( a.Z, b3MulW( s, b.Z ) ) };
}

// a + b
static inline b3Vec2W b3AddV2W( b3Vec2W a, b3Vec2W b )
{
	return (b3Vec2W){
		b3AddW( a.x, b.x ),
		b3AddW( a.y, b.y ),
	};
}

// a - b
static inline b3Vec3W b3SubVW( b3Vec3W a, b3Vec3W b )
{
	return (b3Vec3W){
		b3SubW( a.X, b.X ),
		b3SubW( a.Y, b.Y ),
		b3SubW( a.Z, b.Z ),
	};
}

// a + b
static inline b3Vec3W b3AddVW( b3Vec3W a, b3Vec3W b )
{
	return (b3Vec3W){
		b3AddW( a.X, b.X ),
		b3AddW( a.Y, b.Y ),
		b3AddW( a.Z, b.Z ),
	};
}

// m * a
static inline b3Vec2W b3MulMV2W( b3SymMatrix2W m, b3Vec2W a )
{
	b3Vec2W b = {
		b3Dot2W( m.cxx, a.x, m.cxy, a.y ),
		b3Dot2W( m.cxy, a.x, m.cyy, a.y ),
	};

	return b;
}

// m * a
static inline b3Vec3W b3MulMVW( b3SymMatrix3W m, b3Vec3W a )
{
	b3Vec3W b = {
		b3Dot3W( m.cxx, a.X, m.cxy, a.Y, m.cxz, a.Z ),
		b3Dot3W( m.cxy, a.X, m.cyy, a.Y, m.cyz, a.Z ),
		b3Dot3W( m.cxz, a.X, m.cyz, a.Y, m.czz, a.Z ),
	};

	return b;
}

// a - m * b
static inline b3Vec3W b3MulSubMVW( b3Vec3W a, b3SymMatrix3W m, b3Vec3W b )
{
	return (b3Vec3W){
		b3SubDot3W( a.X, m.cxx, b.X, m.cxy, b.Y, m.cxz, b.Z ),
		b3SubDot3W( a.Y, m.cxy, b.X, m.cyy, b.Y, m.cyz, b.Z ),
		b3SubDot3W( a.Z, m.cxz, b.X, m.cyz, b.Y, m.czz, b.Z ),
	};
}

// a + m * b
static inline b3Vec3W b3MulAddMVW( b3Vec3W a, b3SymMatrix3W m, b3Vec3W b )
{
	return (b3Vec3W){
		b3AddDot3W( a.X, m.cxx, b.X, m.cxy, b.Y, m.cxz, b.Z ),
		b3AddDot3W( a.Y, m.cxy, b.X, m.cyy, b.Y, m.cyz, b.Z ),
		b3AddDot3W( a.Z, m.cxz, b.X, m.cyz, b.Y, m.czz, b.Z ),
	};
}

static inline b3FloatW b3DotW( b3Vec3W a, b3Vec3W b )
{
	return b3Dot3W( a.X, b.X, a.Y, b.Y, a.Z, b.Z );
}

// Wide 3x3 rotation matrix stored as rows so a rotate is three b3Dot3W reductions
typedef struct b3Matrix3W
{
	b3Vec3W rx, ry, rz;
} b3Matrix3W;

// Rotation matrix entries as fused 128-bit reductions with one rounding each.
// The doubling is an exact shift on the raw sum, not a fixed-point multiply.

// 1 - 2*(a*b + c*d)
static inline b3FloatW b3RotDiagW( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
	const b3Int128 one = (b3Int128)B3_FIXED_ONE << B3_FIXED_FRACTION_BITS;
	return (b3FloatW){
		b3FixFromDotRaw( one - ( ( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x ) << 1 ) ),
		b3FixFromDotRaw( one - ( ( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y ) << 1 ) ),
		b3FixFromDotRaw( one - ( ( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z ) << 1 ) ),
		b3FixFromDotRaw( one - ( ( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w ) << 1 ) ),
	};
}

// 2*(a*b + c*d)
static inline b3FloatW b3RotAddW( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
	return (b3FloatW){
		b3FixFromDotRaw( ( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x ) << 1 ),
		b3FixFromDotRaw( ( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y ) << 1 ),
		b3FixFromDotRaw( ( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z ) << 1 ),
		b3FixFromDotRaw( ( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w ) << 1 ),
	};
}

// 2*(a*b - c*d)
static inline b3FloatW b3RotSubW( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
	return (b3FloatW){
		b3FixFromDotRaw( ( (b3Int128)a.x * b.x - (b3Int128)c.x * d.x ) << 1 ),
		b3FixFromDotRaw( ( (b3Int128)a.y * b.y - (b3Int128)c.y * d.y ) << 1 ),
		b3FixFromDotRaw( ( (b3Int128)a.z * b.z - (b3Int128)c.z * d.z ) << 1 ),
		b3FixFromDotRaw( ( (b3Int128)a.w * b.w - (b3Int128)c.w * d.w ) << 1 ),
	};
}

// Same element layout as the scalar b3MakeMatrixFromQuat, stored as rows
static inline b3Matrix3W b3MakeMatrixFromQuatW( b3QuatW q )
{
	b3FloatW x = q.V.X;
	b3FloatW y = q.V.Y;
	b3FloatW z = q.V.Z;
	b3FloatW s = q.S;

	b3Matrix3W m;
	m.rx = (b3Vec3W){ b3RotDiagW( y, y, z, z ), b3RotSubW( x, y, z, s ), b3RotAddW( x, z, y, s ) };
	m.ry = (b3Vec3W){ b3RotAddW( x, y, z, s ), b3RotDiagW( x, x, z, z ), b3RotSubW( y, z, x, s ) };
	m.rz = (b3Vec3W){ b3RotSubW( x, z, y, s ), b3RotAddW( y, z, x, s ), b3RotDiagW( x, x, y, y ) };
	return m;
}

// m * v, one rounding per row
static inline b3Vec3W b3MulMV3W( b3Matrix3W m, b3Vec3W v )
{
	return (b3Vec3W){
		b3Dot3W( m.rx.X, v.X, m.rx.Y, v.Y, m.rx.Z, v.Z ),
		b3Dot3W( m.ry.X, v.X, m.ry.Y, v.Y, m.ry.Z, v.Z ),
		b3Dot3W( m.rz.X, v.X, m.rz.Y, v.Y, m.rz.Z, v.Z ),
	};
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

typedef struct b3ContactConstraintPointWide
{
	b3Vec3W anchorAs, anchorBs;

	// Precomputed normal Jacobian rows: cross(r, normal) for the velocity
	// projection and invI * cross(r, normal) for the impulse application.
	// Each 128-bit fixed multiply costs several ops, so trading memory for
	// multiplies pays here (unlike the float SIMD solver).
	b3Vec3W rnAs, rnBs;
	b3Vec3W iRnAs, iRnBs;

	b3FloatW baseSeparations;
	b3FloatW normalImpulses;
	b3FloatW totalNormalImpulses;
	b3FloatW normalMasses;
	b3FloatW leverArms;
	b3FloatW relativeVelocities;
} b3ContactConstraintPointWide;

// Solves four points
typedef struct b3ContactConstraintWide
{
	// These are base 1
	int indexA[B3_SIMD_WIDTH];
	int indexB[B3_SIMD_WIDTH];

	b3FloatW invMassA, invMassB;
	b3SymMatrix3W invIA, invIB;
	b3Vec3W normal;

	// Precomputed per-constraint Jacobian terms: invMass * normal for the
	// linear impulse, invI * normal for the twist impulse, and
	// cross(origin, tangent) for central friction
	b3Vec3W mNormalA, mNormalB;
	b3Vec3W iNormalA, iNormalB;
	b3Vec3W rtA1s, rtA2s, rtB1s, rtB2s;

	// todo test computing the tangents on the fly, at least tangent2
	b3Vec3W tangent1;
	b3Vec3W tangent2;

	b3Vec3W originA, originB;
	b3FloatW twistMass;
	b3FloatW twistImpulse;
	b3SymMatrix2W tangentMass;
	b3Vec2W frictionImpulse;
	b3SymMatrix3W rollingMass;
	b3Vec3W rollingImpulse;
	b3FloatW friction;
	b3FloatW rollingResistance;
	b3FloatW tangentVelocity1;
	b3FloatW tangentVelocity2;

	b3FloatW biasRate;
	b3FloatW massScale;
	b3FloatW impulseScale;
	b3FloatW restitution;

	b3Manifold* manifolds[B3_SIMD_WIDTH];

	// todo store the maximum point count per wide constraint
	// to make this work I need zero initialization which is too
	// expensive for all the wide constraint data. Instead
	// the graph color should store the point count as a compact secondary
	// transient array with zero initialization.
	b3ContactConstraintPointWide points[B3_MAX_MANIFOLD_POINTS];

} b3ContactConstraintWide;

int b3GetWideContactConstraintByteCount( void )
{
	return sizeof( b3ContactConstraintWide );
}

// wide version of b3BodyState
typedef struct b3BodyStateW
{
	b3Vec3W v;
	b3Vec3W w;
	b3Vec3W dp;
	b3QuatW dq;
} b3BodyStateW;


static b3BodyStateW b3GatherBodies( const b3BodyState* states, int* indices )
{
	b3BodyState identity = b3_identityBodyState;

	b3BodyState s1 = indices[0] == 0 ? identity : states[indices[0] - 1];
	b3BodyState s2 = indices[1] == 0 ? identity : states[indices[1] - 1];
	b3BodyState s3 = indices[2] == 0 ? identity : states[indices[2] - 1];
	b3BodyState s4 = indices[3] == 0 ? identity : states[indices[3] - 1];

	b3BodyStateW simdBody;
	simdBody.v.X = (b3FloatW){ s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x };
	simdBody.v.Y = (b3FloatW){ s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y };
	simdBody.v.Z = (b3FloatW){ s1.linearVelocity.z, s2.linearVelocity.z, s3.linearVelocity.z, s4.linearVelocity.z };
	simdBody.w.X = (b3FloatW){ s1.angularVelocity.x, s2.angularVelocity.x, s3.angularVelocity.x, s4.angularVelocity.x };
	simdBody.w.Y = (b3FloatW){ s1.angularVelocity.y, s2.angularVelocity.y, s3.angularVelocity.y, s4.angularVelocity.y };
	simdBody.w.Z = (b3FloatW){ s1.angularVelocity.z, s2.angularVelocity.z, s3.angularVelocity.z, s4.angularVelocity.z };
	simdBody.dp.X = (b3FloatW){ s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x };
	simdBody.dp.Y = (b3FloatW){ s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y };
	simdBody.dp.Z = (b3FloatW){ s1.deltaPosition.z, s2.deltaPosition.z, s3.deltaPosition.z, s4.deltaPosition.z };
	simdBody.dq.V.X = (b3FloatW){ s1.deltaRotation.v.x, s2.deltaRotation.v.x, s3.deltaRotation.v.x, s4.deltaRotation.v.x };
	simdBody.dq.V.Y = (b3FloatW){ s1.deltaRotation.v.y, s2.deltaRotation.v.y, s3.deltaRotation.v.y, s4.deltaRotation.v.y };
	simdBody.dq.V.Z = (b3FloatW){ s1.deltaRotation.v.z, s2.deltaRotation.v.z, s3.deltaRotation.v.z, s4.deltaRotation.v.z };
	simdBody.dq.S = (b3FloatW){ s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s };

	return simdBody;
}

// This writes only the velocities back to the solver bodies
static void b3ScatterBodies( b3BodyState* states, int* indices, const b3BodyStateW* simdBody )
{
	int index1 = indices[0] - 1;
	if ( index1 != -1 && ( states[index1].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index1;
		state->linearVelocity.x = simdBody->v.X.x;
		state->linearVelocity.y = simdBody->v.Y.x;
		state->linearVelocity.z = simdBody->v.Z.x;
		state->angularVelocity.x = simdBody->w.X.x;
		state->angularVelocity.y = simdBody->w.Y.x;
		state->angularVelocity.z = simdBody->w.Z.x;
	}

	int index2 = indices[1] - 1;
	if ( index2 != -1 && ( states[index2].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index2;
		state->linearVelocity.x = simdBody->v.X.y;
		state->linearVelocity.y = simdBody->v.Y.y;
		state->linearVelocity.z = simdBody->v.Z.y;
		state->angularVelocity.x = simdBody->w.X.y;
		state->angularVelocity.y = simdBody->w.Y.y;
		state->angularVelocity.z = simdBody->w.Z.y;
	}

	int index3 = indices[2] - 1;
	if ( index3 != -1 && ( states[index3].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index3;
		state->linearVelocity.x = simdBody->v.X.z;
		state->linearVelocity.y = simdBody->v.Y.z;
		state->linearVelocity.z = simdBody->v.Z.z;
		state->angularVelocity.x = simdBody->w.X.z;
		state->angularVelocity.y = simdBody->w.Y.z;
		state->angularVelocity.z = simdBody->w.Z.z;
	}

	int index4 = indices[3] - 1;
	if ( index4 != -1 && ( states[index4].flags & b3_dynamicFlag ) != 0 )
	{
		b3BodyState* state = states + index4;
		state->linearVelocity.x = simdBody->v.X.w;
		state->linearVelocity.y = simdBody->v.Y.w;
		state->linearVelocity.z = simdBody->v.Z.w;
		state->angularVelocity.x = simdBody->w.X.w;
		state->angularVelocity.y = simdBody->w.Y.w;
		state->angularVelocity.z = simdBody->w.Z.w;
	}
}

// Prepare convex contact constraints
void b3PrepareContacts_Convex( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( prepare_contact, "Prepare Contact", b3_colorYellow, true );
	b3World* world = context->world;
	b3BodySim* sims = context->sims;
	b3BodyState* states = context->states;
#if B3_ENABLE_VALIDATION
	b3Body* bodies = world->bodies.data;
#endif
	b3WidePrepareSpan* spans = context->widePrepareSpans;
	b3ContactConstraintWide* wideBase = context->wideConstraints;

	// Stiffer for static contacts to avoid bodies getting pushed through the ground
	b3Softness contactSoftness = context->contactSoftness;
	b3Softness staticSoftness = context->staticSoftness;

	b3Fixed warmStartScale = world->enableWarmStarting ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );

	int wideIndex = block.startIndex;
	int endWideIndex = block.startIndex + block.count;

	// Find color for start index. Linear search but fast.
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= wideIndex )
	{
		colorIndex += 1;
	}

	// Loop over block
	while ( wideIndex < endWideIndex )
	{
		int colorWideStart = spans[colorIndex].start;
		int colorWideEndIndex = b3MinInt( spans[colorIndex + 1].start, endWideIndex );
		int colorContactCount = spans[colorIndex].count;
		int* contactIds = spans[colorIndex].contacts;

		// Loop over color
		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			b3ContactConstraintWide* constraint = wideBase + wideIndex;
			int localWideIndex = wideIndex - colorWideStart;

			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex >= colorContactCount )
				{
					// Remainder lanes were zeroed in solver setup.
					break;
				}

				int contactId = contactIds[contactIndex];
				b3Contact* contact = b3Array_Get( world->contacts, contactId  );
				B3_ASSERT( contact->manifoldCount == 1 );
				b3Manifold* manifold = contact->manifolds + 0;

				int indexA = contact->bodySimIndexA;
				int indexB = contact->bodySimIndexB;

#if B3_ENABLE_VALIDATION
				b3Body* bodyA = bodies + contact->edges[0].bodyId;
				int validIndexA = bodyA->setIndex == b3_awakeSet ? bodyA->localIndex : B3_NULL_INDEX;
				b3Body* bodyB = bodies + contact->edges[1].bodyId;
				int validIndexB = bodyB->setIndex == b3_awakeSet ? bodyB->localIndex : B3_NULL_INDEX;
				B3_ASSERT( indexA == validIndexA );
				B3_ASSERT( indexB == validIndexB );
#endif

				// 0 for null
				constraint->indexA[lane] = indexA + 1;
				constraint->indexB[lane] = indexB + 1;
				constraint->manifolds[lane] = manifold;

				// Body A data
				b3Fixed mA;
				b3Matrix3 iA;
				b3Vec3 vA;
				b3Vec3 wA;

				if ( indexA == B3_NULL_INDEX )
				{
					mA = B3_FIX( 0.0f );
					iA = b3Mat3_zero;
					vA = b3Vec3_zero;
					wA = b3Vec3_zero;
				}
				else
				{
					b3BodySim* simA = sims + indexA;
					mA = simA->invMass;
					iA = simA->invInertiaWorld;

					b3BodyState* stateA = states + indexA;
					vA = stateA->linearVelocity;
					wA = stateA->angularVelocity;
				}

				// Body B data
				b3Fixed mB;
				b3Matrix3 iB;
				b3Vec3 vB;
				b3Vec3 wB;

				if ( indexB == B3_NULL_INDEX )
				{
					mB = B3_FIX( 0.0f );
					iB = b3Mat3_zero;
					vB = b3Vec3_zero;
					wB = b3Vec3_zero;
				}
				else
				{
					b3BodySim* simB = sims + indexB;
					mB = simB->invMass;
					iB = simB->invInertiaWorld;

					b3BodyState* stateB = states + indexB;
					vB = stateB->linearVelocity;
					wB = stateB->angularVelocity;
				}

				( (b3Fixed*)&constraint->invMassA )[lane] = mA;
				( (b3Fixed*)&constraint->invMassB )[lane] = mB;

				( (b3Fixed*)&constraint->invIA.cxx )[lane] = iA.cx.x;
				( (b3Fixed*)&constraint->invIA.cxy )[lane] = iA.cx.y;
				( (b3Fixed*)&constraint->invIA.cxz )[lane] = iA.cx.z;
				( (b3Fixed*)&constraint->invIA.cyy )[lane] = iA.cy.y;
				( (b3Fixed*)&constraint->invIA.cyz )[lane] = iA.cy.z;
				( (b3Fixed*)&constraint->invIA.czz )[lane] = iA.cz.z;

				( (b3Fixed*)&constraint->invIB.cxx )[lane] = iB.cx.x;
				( (b3Fixed*)&constraint->invIB.cxy )[lane] = iB.cx.y;
				( (b3Fixed*)&constraint->invIB.cxz )[lane] = iB.cx.z;
				( (b3Fixed*)&constraint->invIB.cyy )[lane] = iB.cy.y;
				( (b3Fixed*)&constraint->invIB.cyz )[lane] = iB.cy.z;
				( (b3Fixed*)&constraint->invIB.czz )[lane] = iB.cz.z;

				b3Softness soft = ( indexA == B3_NULL_INDEX || indexB == B3_NULL_INDEX ) ? staticSoftness : contactSoftness;

				b3Vec3 normal = manifold->normal;
				( (b3Fixed*)&constraint->normal.X )[lane] = normal.x;
				( (b3Fixed*)&constraint->normal.Y )[lane] = normal.y;
				( (b3Fixed*)&constraint->normal.Z )[lane] = normal.z;

				b3Vec3 tangent1 = b3Perp( normal );
				( (b3Fixed*)&constraint->tangent1.X )[lane] = tangent1.x;
				( (b3Fixed*)&constraint->tangent1.Y )[lane] = tangent1.y;
				( (b3Fixed*)&constraint->tangent1.Z )[lane] = tangent1.z;

				b3Vec3 tangent2 = b3Cross( tangent1, normal );
				( (b3Fixed*)&constraint->tangent2.X )[lane] = tangent2.x;
				( (b3Fixed*)&constraint->tangent2.Y )[lane] = tangent2.y;
				( (b3Fixed*)&constraint->tangent2.Z )[lane] = tangent2.z;

				( (b3Fixed*)&constraint->friction )[lane] = contact->friction;
				( (b3Fixed*)&constraint->restitution )[lane] = contact->restitution;
				( (b3Fixed*)&constraint->rollingResistance )[lane] = contact->rollingResistance;

				( (b3Fixed*)&constraint->tangentVelocity1 )[lane] = b3Dot( contact->tangentVelocity, tangent1 );
				( (b3Fixed*)&constraint->tangentVelocity2 )[lane] = b3Dot( contact->tangentVelocity, tangent2 );

				( (b3Fixed*)&constraint->biasRate )[lane] = soft.biasRate;
				( (b3Fixed*)&constraint->massScale )[lane] = soft.massScale;
				( (b3Fixed*)&constraint->impulseScale )[lane] = soft.impulseScale;

				int pointCount = manifold->pointCount;
				b3Vec3 originA = b3Vec3_zero;
				b3Vec3 originB = b3Vec3_zero;

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const b3ManifoldPoint* mp = manifold->points + pointIndex;
					b3ContactConstraintPointWide* cp = constraint->points + pointIndex;

					b3Vec3 rA = mp->anchorA;
					b3Vec3 rB = mp->anchorB;
					originA = b3Add( originA, rA );
					originB = b3Add( originB, rB );

					( (b3Fixed*)&cp->anchorAs.X )[lane] = rA.x;
					( (b3Fixed*)&cp->anchorAs.Y )[lane] = rA.y;
					( (b3Fixed*)&cp->anchorAs.Z )[lane] = rA.z;

					( (b3Fixed*)&cp->anchorBs.X )[lane] = rB.x;
					( (b3Fixed*)&cp->anchorBs.Y )[lane] = rB.y;
					( (b3Fixed*)&cp->anchorBs.Z )[lane] = rB.z;

					b3Fixed baseSeparation = mp->separation - b3Dot( b3Sub( rB, rA ), normal );
					( (b3Fixed*)&cp->baseSeparations )[lane] = baseSeparation;

					( (b3Fixed*)&cp->normalImpulses )[lane] = b3FixMul( warmStartScale , mp->normalImpulse );
					( (b3Fixed*)&cp->totalNormalImpulses )[lane] = B3_FIX( 0.0f );

					b3Vec3 rnA = b3Cross( rA, normal );
					b3Vec3 rnB = b3Cross( rB, normal );
					b3Vec3 iRnA = b3MulMV( iA, rnA );
					b3Vec3 iRnB = b3MulMV( iB, rnB );
					b3Fixed kNormal = mA + mB + b3Dot( rnA, iRnA ) + b3Dot( rnB, iRnB );
					( (b3Fixed*)&cp->normalMasses )[lane] = kNormal > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ) , kNormal ) : B3_FIX( 0.0f );

					// Precomputed normal Jacobian rows for the solve and warm start
					( (b3Fixed*)&cp->rnAs.X )[lane] = rnA.x;
					( (b3Fixed*)&cp->rnAs.Y )[lane] = rnA.y;
					( (b3Fixed*)&cp->rnAs.Z )[lane] = rnA.z;
					( (b3Fixed*)&cp->rnBs.X )[lane] = rnB.x;
					( (b3Fixed*)&cp->rnBs.Y )[lane] = rnB.y;
					( (b3Fixed*)&cp->rnBs.Z )[lane] = rnB.z;
					( (b3Fixed*)&cp->iRnAs.X )[lane] = iRnA.x;
					( (b3Fixed*)&cp->iRnAs.Y )[lane] = iRnA.y;
					( (b3Fixed*)&cp->iRnAs.Z )[lane] = iRnA.z;
					( (b3Fixed*)&cp->iRnBs.X )[lane] = iRnB.x;
					( (b3Fixed*)&cp->iRnBs.Y )[lane] = iRnB.y;
					( (b3Fixed*)&cp->iRnBs.Z )[lane] = iRnB.z;

					// Save relative velocity for restitution
					b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
					b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
					( (b3Fixed*)&cp->relativeVelocities )[lane] = b3Dot( normal, b3Sub( vrB, vrA ) );
				}

				b3Fixed invCount = b3FixDiv( B3_FIX( 1.0f ) , b3FixFromInt( pointCount ) );
				originA = b3MulSV( invCount, originA );
				originB = b3MulSV( invCount, originB );

				( (b3Fixed*)&constraint->originA.X )[lane] = originA.x;
				( (b3Fixed*)&constraint->originA.Y )[lane] = originA.y;
				( (b3Fixed*)&constraint->originA.Z )[lane] = originA.z;
				( (b3Fixed*)&constraint->originB.X )[lane] = originB.x;
				( (b3Fixed*)&constraint->originB.Y )[lane] = originB.y;
				( (b3Fixed*)&constraint->originB.Z )[lane] = originB.z;

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const b3ManifoldPoint* mp = manifold->points + pointIndex;
					b3ContactConstraintPointWide* cp = constraint->points + pointIndex;
					( (b3Fixed*)&cp->leverArms )[lane] = b3Distance( mp->anchorA, originA );
				}

				b3Vec3 rtA1 = b3Cross( originA, tangent1 );
				b3Vec3 rtA2 = b3Cross( originA, tangent2 );

				b3Vec3 rtB1 = b3Cross( originB, tangent1 );
				b3Vec3 rtB2 = b3Cross( originB, tangent2 );

				// Precomputed friction Jacobian rows
				( (b3Fixed*)&constraint->rtA1s.X )[lane] = rtA1.x;
				( (b3Fixed*)&constraint->rtA1s.Y )[lane] = rtA1.y;
				( (b3Fixed*)&constraint->rtA1s.Z )[lane] = rtA1.z;
				( (b3Fixed*)&constraint->rtA2s.X )[lane] = rtA2.x;
				( (b3Fixed*)&constraint->rtA2s.Y )[lane] = rtA2.y;
				( (b3Fixed*)&constraint->rtA2s.Z )[lane] = rtA2.z;
				( (b3Fixed*)&constraint->rtB1s.X )[lane] = rtB1.x;
				( (b3Fixed*)&constraint->rtB1s.Y )[lane] = rtB1.y;
				( (b3Fixed*)&constraint->rtB1s.Z )[lane] = rtB1.z;
				( (b3Fixed*)&constraint->rtB2s.X )[lane] = rtB2.x;
				( (b3Fixed*)&constraint->rtB2s.Y )[lane] = rtB2.y;
				( (b3Fixed*)&constraint->rtB2s.Z )[lane] = rtB2.z;

				// Precomputed linear normal Jacobian
				b3Vec3 mNormalA = b3MulSV( mA, normal );
				b3Vec3 mNormalB = b3MulSV( mB, normal );
				( (b3Fixed*)&constraint->mNormalA.X )[lane] = mNormalA.x;
				( (b3Fixed*)&constraint->mNormalA.Y )[lane] = mNormalA.y;
				( (b3Fixed*)&constraint->mNormalA.Z )[lane] = mNormalA.z;
				( (b3Fixed*)&constraint->mNormalB.X )[lane] = mNormalB.x;
				( (b3Fixed*)&constraint->mNormalB.Y )[lane] = mNormalB.y;
				( (b3Fixed*)&constraint->mNormalB.Z )[lane] = mNormalB.z;

				{
					b3Matrix2 k;
					k.cx.x = mA + mB + b3Dot( rtA1, b3MulMV( iA, rtA1 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB1 ) );
					k.cy.y = mA + mB + b3Dot( rtA2, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB2, b3MulMV( iB, rtB2 ) );
					k.cx.y = k.cy.x = b3Dot( rtA1, b3MulMV( iA, rtA2 ) ) + b3Dot( rtB1, b3MulMV( iB, rtB2 ) );
					b3Matrix2 tangentMass = b3Invert2( k );

					( (b3Fixed*)&constraint->tangentMass.cxx )[lane] = tangentMass.cx.x;
					( (b3Fixed*)&constraint->tangentMass.cxy )[lane] = tangentMass.cx.y;
					( (b3Fixed*)&constraint->tangentMass.cyy )[lane] = tangentMass.cy.y;

					( (b3Fixed*)&constraint->frictionImpulse.x )[lane] =
						b3FixMul( warmStartScale , b3Dot( manifold->frictionImpulse, tangent1 ) );
					( (b3Fixed*)&constraint->frictionImpulse.y )[lane] =
						b3FixMul( warmStartScale , b3Dot( manifold->frictionImpulse, tangent2 ) );
				}

				{
					// Precomputed angular normal Jacobian, also used for the twist mass
					b3Vec3 iNormalA = b3MulMV( iA, normal );
					b3Vec3 iNormalB = b3MulMV( iB, normal );
					( (b3Fixed*)&constraint->iNormalA.X )[lane] = iNormalA.x;
					( (b3Fixed*)&constraint->iNormalA.Y )[lane] = iNormalA.y;
					( (b3Fixed*)&constraint->iNormalA.Z )[lane] = iNormalA.z;
					( (b3Fixed*)&constraint->iNormalB.X )[lane] = iNormalB.x;
					( (b3Fixed*)&constraint->iNormalB.Y )[lane] = iNormalB.y;
					( (b3Fixed*)&constraint->iNormalB.Z )[lane] = iNormalB.z;

					b3Fixed k = b3Dot( normal, iNormalA ) + b3Dot( normal, iNormalB );
					( (b3Fixed*)&constraint->twistMass )[lane] = k > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ) , k ) : B3_FIX( 0.0f );
					( (b3Fixed*)&constraint->twistImpulse )[lane] = b3FixMul( warmStartScale , manifold->twistImpulse );
				}

				{
					// The 128-bit matrix inversion is only needed when rolling resistance is active
					b3Matrix3 rollingMass =
						contact->rollingResistance > B3_FIX( 0.0f ) ? b3InvertMatrix( b3AddMM( iA, iB ) ) : b3Mat3_zero;

					( (b3Fixed*)&constraint->rollingMass.cxx )[lane] = rollingMass.cx.x;
					( (b3Fixed*)&constraint->rollingMass.cxy )[lane] = rollingMass.cx.y;
					( (b3Fixed*)&constraint->rollingMass.cxz )[lane] = rollingMass.cx.z;
					( (b3Fixed*)&constraint->rollingMass.cyy )[lane] = rollingMass.cy.y;
					( (b3Fixed*)&constraint->rollingMass.cyz )[lane] = rollingMass.cy.z;
					( (b3Fixed*)&constraint->rollingMass.czz )[lane] = rollingMass.cz.z;

					( (b3Fixed*)&constraint->rollingImpulse.X )[lane] = b3FixMul( warmStartScale , manifold->rollingImpulse.x );
					( (b3Fixed*)&constraint->rollingImpulse.Y )[lane] = b3FixMul( warmStartScale , manifold->rollingImpulse.y );
					( (b3Fixed*)&constraint->rollingImpulse.Z )[lane] = b3FixMul( warmStartScale , manifold->rollingImpulse.z );
				}

				// zero remaining points
				for ( int pointIndex = pointCount; pointIndex < B3_MAX_MANIFOLD_POINTS; ++pointIndex )
				{
					b3ContactConstraintPointWide* cp = constraint->points + pointIndex;
					( (b3Fixed*)&cp->anchorAs.X )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->anchorAs.Y )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->anchorAs.Z )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->anchorBs.X )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->anchorBs.Y )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->anchorBs.Z )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->rnAs.X )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->rnAs.Y )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->rnAs.Z )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->rnBs.X )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->rnBs.Y )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->rnBs.Z )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->iRnAs.X )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->iRnAs.Y )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->iRnAs.Z )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->iRnBs.X )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->iRnBs.Y )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->iRnBs.Z )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->baseSeparations )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->normalImpulses )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->totalNormalImpulses )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->normalMasses )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->relativeVelocities )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->leverArms )[lane] = B3_FIX( 0.0f );
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	b3TracyCZoneEnd( prepare_contact );
}

void b3WarmStartContacts_Convex( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( warm_start_contact, "Warm Start", b3_colorGreen, true );

	b3BodyState* states = context->states;
	b3ContactConstraintWide* constraints = context->graph->colors[block.colorIndex].wideConstraints;

	for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
	{
		b3ContactConstraintWide* c = constraints + i;
		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		// Normal impulses, applied through the precomputed Jacobian rows
		for ( int j = 0; j < B3_MAX_MANIFOLD_POINTS; ++j )
		{
			b3ContactConstraintPointWide* cp = c->points + j;

			bA.v = b3MulSubSVW( bA.v, cp->normalImpulses, c->mNormalA );
			bA.w = b3MulSubSVW( bA.w, cp->normalImpulses, cp->iRnAs );
			bB.v = b3MulAddSVW( bB.v, cp->normalImpulses, c->mNormalB );
			bB.w = b3MulAddSVW( bB.w, cp->normalImpulses, cp->iRnBs );
		}

		// Central friction
		{
			b3Vec3W impulse = b3MulSVW( c->frictionImpulse.x, c->tangent1 );
			impulse = b3MulAddSVW( impulse, c->frictionImpulse.y, c->tangent2 );

			// cross(origin, P) expanded over the precomputed tangent rows
			b3Vec3W LA = b3MulSVW( c->frictionImpulse.x, c->rtA1s );
			LA = b3MulAddSVW( LA, c->frictionImpulse.y, c->rtA2s );
			b3Vec3W LB = b3MulSVW( c->frictionImpulse.x, c->rtB1s );
			LB = b3MulAddSVW( LB, c->frictionImpulse.y, c->rtB2s );

			bA.w = b3MulSubMVW( bA.w, c->invIA, LA );
			bA.v = b3MulSubSVW( bA.v, c->invMassA, impulse );
			bB.w = b3MulAddMVW( bB.w, c->invIB, LB );
			bB.v = b3MulAddSVW( bB.v, c->invMassB, impulse );
		}

		// Central twist friction
		{
			bA.w = b3MulSubSVW( bA.w, c->twistImpulse, c->iNormalA );
			bB.w = b3MulAddSVW( bB.w, c->twistImpulse, c->iNormalB );
		}

		// Rolling resistance
		{
			b3Vec3W impulse = c->rollingImpulse;
			bA.w = b3MulSubMVW( bA.w, c->invIA, impulse );
			bB.w = b3MulAddMVW( bB.w, c->invIB, impulse );
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );
	}

	b3TracyCZoneEnd( warm_start_contact );
}

void b3SolveContacts_Convex( b3SolverBlock block, b3StepContext* context, bool useBias )
{
	b3TracyCZoneNC( solve_contact, "Solve Contact", b3_colorAliceBlue, true );

	b3BodyState* states = context->states;
	b3ContactConstraintWide* constraints = context->graph->colors[block.colorIndex].wideConstraints;
	b3FloatW inv_h = b3SplatW( context->inv_h );
	b3FloatW contactSpeed = b3SplatW( -context->world->contactSpeed );
	b3FloatW oneW = b3SplatW( B3_FIX( 1.0f ) );
	b3FloatW epsilonW = b3SplatW( B3_FIXED_EPSILON );

	for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
	{
		b3ContactConstraintWide* c = constraints + wideIndex;

		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		b3FloatW biasRate, massScale, impulseScale;
		if ( useBias )
		{
			biasRate = b3MulW( c->massScale, c->biasRate );
			massScale = c->massScale;
			impulseScale = c->impulseScale;
		}
		else
		{
			biasRate = b3ZeroW();
			massScale = oneW;
			impulseScale = b3ZeroW();
		}

		b3Vec3W dp = b3SubVW( bB.dp, bA.dp );

		// The delta rotations are constant across the four manifold points, so build
		// them as matrices once and rotate each anchor with three fused reductions
		b3Matrix3W rotA = b3MakeMatrixFromQuatW( bA.dq );
		b3Matrix3W rotB = b3MakeMatrixFromQuatW( bB.dq );

		b3FloatW totalNormalImpulse = b3ZeroW();
		b3FloatW totalTwistLimit = b3ZeroW();

		// todo_erin use the max point count of the four manifolds
		for ( int pointIndex = 0; pointIndex < B3_MAX_MANIFOLD_POINTS; ++pointIndex )
		{
			b3ContactConstraintPointWide* cp = c->points + pointIndex;

			// Fixed anchor points for applying impulses
			b3Vec3W rA = cp->anchorAs;
			b3Vec3W rB = cp->anchorBs;

			// Moving anchors for current separation
			b3Vec3W rsA = b3MulMV3W( rotA, rA );
			b3Vec3W rsB = b3MulMV3W( rotB, rB );

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b3Vec3W ds = b3AddVW( dp, b3SubVW( rsB, rsA ) );
			b3FloatW s = b3AddW( b3DotW( c->normal, ds ), cp->baseSeparations );

			// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
			b3FloatW mask = b3GreaterThanW( s, b3ZeroW() );
			b3FloatW specBias = b3MulW( s, inv_h );
			b3FloatW softBias = b3MaxW( b3MulW( biasRate, s ), contactSpeed );
			b3FloatW bias = b3BlendW( softBias, specBias, mask );

			b3FloatW pointMassScale = b3BlendW( massScale, oneW, mask );
			b3FloatW pointImpulseScale = b3BlendW( impulseScale, b3ZeroW(), mask );

			// Relative velocity at contact, projected on the normal through the
			// precomputed cross(r, normal) rows with a single rounding
			b3FloatW vn = b3RelVelocityW( b3SubVW( bB.v, bA.v ), c->normal, bB.w, cp->rnBs, bA.w, cp->rnAs );

			// Compute normal impulse
			b3FloatW negImpulse = b3AddW( b3MulW( cp->normalMasses, b3AddW( b3MulW( pointMassScale, vn ), bias ) ),
										  b3MulW( pointImpulseScale, cp->normalImpulses ) );

			// Clamp the accumulated impulse
			b3FloatW newImpulse = b3MaxW( b3SubW( cp->normalImpulses, negImpulse ), b3ZeroW() );
			b3FloatW deltaImpulse = b3SubW( newImpulse, cp->normalImpulses );
			cp->normalImpulses = newImpulse;
			cp->totalNormalImpulses = b3AddW( cp->totalNormalImpulses, newImpulse );

			totalNormalImpulse = b3AddW( totalNormalImpulse, newImpulse );
			totalTwistLimit = b3AddW( totalTwistLimit, b3MulW( cp->leverArms, newImpulse ) );

			// Apply contact impulse through the precomputed Jacobian rows
			bA.v = b3MulSubSVW( bA.v, deltaImpulse, c->mNormalA );
			bA.w = b3MulSubSVW( bA.w, deltaImpulse, cp->iRnAs );
			bB.v = b3MulAddSVW( bB.v, deltaImpulse, c->mNormalB );
			bB.w = b3MulAddSVW( bB.w, deltaImpulse, cp->iRnBs );
		}

		// No friction when applying bias
		if ( useBias == false )
		{
			// Rolling resistance
			if ( b3AllZeroW( c->rollingResistance ) == false )
			{
				// flip A/B order to negate
				b3Vec3W deltaImpulse = b3MulMVW( c->rollingMass, b3SubVW( bA.w, bB.w ) );
				b3Vec3W oldImpulse = c->rollingImpulse;
				c->rollingImpulse = b3AddVW( oldImpulse, deltaImpulse );

				b3FloatW maxImpulse = b3MulW( c->rollingResistance, totalNormalImpulse );
				b3FloatW lengthSquared = b3DotW( c->rollingImpulse, c->rollingImpulse );

				// if ( magSqr > maxLambda * maxLambda + B3_FIXED_EPSILON )
				//{
				//	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
				// }

				b3FloatW mask = b3GreaterThanW( lengthSquared, b3MulAddW( epsilonW, maxImpulse, maxImpulse ) );

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
				b3FloatW normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				b3FloatW scale = b3BlendW( oneW, normalize, mask );

				// Ensure zero rolling resistance yields no impulse
				b3FloatW rollingMask = b3GreaterThanW( c->rollingResistance, b3ZeroW() );
				scale = b3BlendW( b3ZeroW(), scale, rollingMask );

				c->rollingImpulse = b3MulSVW( scale, c->rollingImpulse );

				deltaImpulse = b3SubVW( c->rollingImpulse, oldImpulse );

				bA.w = b3MulSubMVW( bA.w, c->invIA, deltaImpulse );
				bB.w = b3MulAddMVW( bB.w, c->invIB, deltaImpulse );
			}

			// Central twist friction
			{
				b3FloatW twistSpeed = b3DotW( c->normal, b3SubVW( bB.w, bA.w ) );
				b3FloatW maxLambda = b3MulW( c->friction, totalTwistLimit );
				b3FloatW deltaImpulse = b3NegW( b3MulW( c->twistMass, twistSpeed ) );
				b3FloatW oldImpulse = c->twistImpulse;
				c->twistImpulse = b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxLambda );
				deltaImpulse = b3SubW( c->twistImpulse, oldImpulse );

				bA.w = b3MulSubSVW( bA.w, deltaImpulse, c->iNormalA );
				bB.w = b3MulAddSVW( bB.w, deltaImpulse, c->iNormalB );
			}

			// Central friction
			{
				b3Vec3W tangent1 = c->tangent1;
				b3Vec3W tangent2 = c->tangent2;

				// Relative tangent velocity at contact through the precomputed
				// cross(origin, tangent) rows, one rounding per axis
				b3Vec3W dv = b3SubVW( bB.v, bA.v );
				b3Vec2W vt = {
					b3SubW( b3RelVelocityW( dv, tangent1, bB.w, c->rtB1s, bA.w, c->rtA1s ), c->tangentVelocity1 ),
					b3SubW( b3RelVelocityW( dv, tangent2, bB.w, c->rtB2s, bA.w, c->rtA2s ), c->tangentVelocity2 ),
				};

				// Incremental tangent impulse
				b3Vec2W deltaImpulse = b3MulMV2W( c->tangentMass, vt );
				deltaImpulse = (b3Vec2W){ b3NegW( deltaImpulse.x ), b3NegW( deltaImpulse.y ) };
				b3Vec2W newImpulse = b3AddV2W( c->frictionImpulse, deltaImpulse );

				b3FloatW friction = c->friction;
				b3FloatW maxImpulse = b3MulW( friction, totalNormalImpulse );

				// Clamp the accumulated impulse
				b3FloatW lengthSquared = b3AddW( b3MulW( newImpulse.x, newImpulse.x ), b3MulW( newImpulse.y, newImpulse.y ) );

				// Max impulse can be zero
				b3FloatW mask = b3GreaterThanW( lengthSquared, b3MulW( maxImpulse, maxImpulse ) );

				// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
				// zero.
				b3FloatW normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				b3FloatW scale = b3BlendW( oneW, normalize, mask );
				newImpulse = (b3Vec2W){
					b3MulW( scale, newImpulse.x ),
					b3MulW( scale, newImpulse.y ),
				};

				deltaImpulse = (b3Vec2W){
					b3SubW( newImpulse.x, c->frictionImpulse.x ),
					b3SubW( newImpulse.y, c->frictionImpulse.y ),
				};

				c->frictionImpulse = newImpulse;

				// Apply delta impulse; cross(origin, P) expands over the tangent rows
				b3Vec3W P = b3AddVW( b3MulSVW( deltaImpulse.x, tangent1 ), b3MulSVW( deltaImpulse.y, tangent2 ) );
				b3Vec3W LA = b3AddVW( b3MulSVW( deltaImpulse.x, c->rtA1s ), b3MulSVW( deltaImpulse.y, c->rtA2s ) );
				b3Vec3W LB = b3AddVW( b3MulSVW( deltaImpulse.x, c->rtB1s ), b3MulSVW( deltaImpulse.y, c->rtB2s ) );
				bA.w = b3MulSubMVW( bA.w, c->invIA, LA );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bB.w = b3MulAddMVW( bB.w, c->invIB, LB );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
			}
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );
	}

	b3TracyCZoneEnd( solve_contact );
}

void b3ApplyRestitution_Convex( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( restitution, "Restitution", b3_colorDodgerBlue, true );

	b3BodyState* states = context->states;
	b3ContactConstraintWide* constraints = context->graph->colors[block.colorIndex].wideConstraints;
	b3FloatW threshold = b3SplatW( context->world->restitutionThreshold );
	b3FloatW zero = b3ZeroW();

	for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
	{
		b3ContactConstraintWide* c = constraints + i;

		if ( b3AllZeroW( c->restitution ) )
		{
			// No lanes have restitution. Common case.
			continue;
		}

		// Single gather for all manifolds
		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		// Create a mask based on restitution so that lanes with no restitution are not affected
		// by the calculations below.
		b3FloatW restitutionMask = b3EqualsW( c->restitution, zero );

		for ( int pointIndex = 0; pointIndex < B3_MAX_MANIFOLD_POINTS; ++pointIndex )
		{
			b3ContactConstraintPointWide* cp = c->points + pointIndex;

			// Set effective mass to zero if restitution should not be applied
			b3FloatW mask1 = b3GreaterThanW( b3AddW( cp->relativeVelocities, threshold ), zero );
			b3FloatW mask2 = b3EqualsW( cp->totalNormalImpulses, zero );
			b3FloatW mask = b3OrW( b3OrW( mask1, mask2 ), restitutionMask );
			b3FloatW mass = b3BlendW( cp->normalMasses, zero, mask );

			// Relative velocity at contact through the precomputed Jacobian rows
			b3FloatW vn = b3RelVelocityW( b3SubVW( bB.v, bA.v ), c->normal, bB.w, cp->rnBs, bA.w, cp->rnAs );

			// Compute normal impulse
			b3FloatW negImpulse = b3MulW( mass, b3AddW( vn, b3MulW( c->restitution, cp->relativeVelocities ) ) );

			// Clamp the accumulated impulse
			b3FloatW newImpulse = b3MaxW( b3SubW( cp->normalImpulses, negImpulse ), b3ZeroW() );
			b3FloatW deltaImpulse = b3SubW( newImpulse, cp->normalImpulses );
			cp->normalImpulses = newImpulse;
			cp->totalNormalImpulses = b3AddW( cp->totalNormalImpulses, deltaImpulse );

			// Apply contact impulse through the precomputed Jacobian rows
			bA.v = b3MulSubSVW( bA.v, deltaImpulse, c->mNormalA );
			bA.w = b3MulSubSVW( bA.w, deltaImpulse, cp->iRnAs );
			bB.v = b3MulAddSVW( bB.v, deltaImpulse, c->mNormalB );
			bB.w = b3MulAddSVW( bB.w, deltaImpulse, cp->iRnBs );
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );
	}

	b3TracyCZoneEnd( restitution );
}

// Store impulses by contact constraint
void b3StoreImpulses_Convex( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
	b3TracyCZoneNC( store_impulses, "Store", b3_colorFireBrick, true );

	b3World* world = context->world;
	b3WidePrepareSpan* spans = context->widePrepareSpans;
	const b3ContactConstraintWide* wideBase = context->wideConstraints;
	b3TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b3BitSet* hitEventBitSet = &taskContext->hitEventBitSet;
	bool hasHitEvents = taskContext->hasHitEvents;
	b3Fixed negHitThreshold = -world->hitEventThreshold;

	int wideIndex = block.startIndex;
	int endWideIndex = block.startIndex + block.count;

	// Find color for start index
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= wideIndex )
	{
		colorIndex += 1;
	}

	while ( wideIndex < endWideIndex )
	{
		int colorWideStart = spans[colorIndex].start;
		int colorWideEndIndex = b3MinInt( spans[colorIndex + 1].start, endWideIndex );
		int colorContactCount = spans[colorIndex].count;
		int* contactIds = spans[colorIndex].contacts;

		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			const b3ContactConstraintWide* c = wideBase + wideIndex;
			const b3Fixed* frictionImpulse1 = (b3Fixed*)&c->frictionImpulse.x;
			const b3Fixed* frictionImpulse2 = (b3Fixed*)&c->frictionImpulse.y;
			const b3Fixed* tangent1X = (b3Fixed*)&c->tangent1.X;
			const b3Fixed* tangent1Y = (b3Fixed*)&c->tangent1.Y;
			const b3Fixed* tangent1Z = (b3Fixed*)&c->tangent1.Z;
			const b3Fixed* tangent2X = (b3Fixed*)&c->tangent2.X;
			const b3Fixed* tangent2Y = (b3Fixed*)&c->tangent2.Y;
			const b3Fixed* tangent2Z = (b3Fixed*)&c->tangent2.Z;
			const b3Fixed* twistImpulse = (b3Fixed*)&c->twistImpulse;
			const b3Fixed* rollingImpulseX = (b3Fixed*)&c->rollingImpulse.X;
			const b3Fixed* rollingImpulseY = (b3Fixed*)&c->rollingImpulse.Y;
			const b3Fixed* rollingImpulseZ = (b3Fixed*)&c->rollingImpulse.Z;

			int localWideIndex = wideIndex - colorWideStart;

			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex >= colorContactCount )
				{
					break;
				}

				b3Manifold* m = c->manifolds[lane];
				if ( m == NULL )
				{
					continue;
				}

				b3Fixed f1 = frictionImpulse1[lane];
				b3Fixed f2 = frictionImpulse2[lane];
				m->frictionImpulse = (b3Vec3){
					b3FixMul( f1 , tangent1X[lane] ) + b3FixMul( f2 , tangent2X[lane] ),
					b3FixMul( f1 , tangent1Y[lane] ) + b3FixMul( f2 , tangent2Y[lane] ),
					b3FixMul( f1 , tangent1Z[lane] ) + b3FixMul( f2 , tangent2Z[lane] ),
				};
				m->twistImpulse = twistImpulse[lane];
				m->rollingImpulse = (b3Vec3){
					rollingImpulseX[lane],
					rollingImpulseY[lane],
					rollingImpulseZ[lane],
				};

				int pointCount = m->pointCount;
				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const b3ContactConstraintPointWide* cp = c->points + pointIndex;
					const b3Fixed* normalImpulse = (b3Fixed*)&cp->normalImpulses;
					const b3Fixed* totalNormalImpulse = (b3Fixed*)&cp->totalNormalImpulses;
					const b3Fixed* normalVelocity = (b3Fixed*)&cp->relativeVelocities;

					b3ManifoldPoint* mp = m->points + pointIndex;
					mp->normalImpulse = normalImpulse[lane];
					mp->totalNormalImpulse = totalNormalImpulse[lane];
					mp->normalVelocity = normalVelocity[lane];
				}

				int contactId = contactIds[contactIndex];
				b3Contact* contact = b3Array_Get( world->contacts, contactId );
				if ( ( contact->flags & b3_simEnableHitEvent ) != 0 )
				{
					for ( int k = 0; k < pointCount; ++k )
					{
						b3ManifoldPoint* mp = m->points + k;

						// Need to check total impulse because the point may be speculative and not colliding
						if ( mp->normalVelocity < negHitThreshold && mp->totalNormalImpulse > B3_FIX( 0.0f ) )
						{
							b3SetBit( hitEventBitSet, contact->contactId );
							hasHitEvents = true;
							break;
						}
					}
				}
			}
		}

		colorIndex += 1;
	}

	taskContext->hasHitEvents = hasHitEvents;

	b3TracyCZoneEnd( store_impulses );
}

void b3PrepareContacts_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if (count == 0)
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3PrepareContacts_Mesh( block, context );
}

void b3WarmStartContacts_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3WarmStartContacts_Mesh( block, context );
}

void b3SolveContacts_Overflow( b3StepContext* context, bool useBias )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3SolveContacts_Mesh( block, context, useBias );
}

void b3ApplyRestitution_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3ApplyRestitution_Mesh( block, context );
}

void b3StoreImpulses_Overflow( b3StepContext* context )
{
	b3ConstraintGraph* graph = context->graph;
	b3GraphColor* color = graph->colors + B3_OVERFLOW_INDEX;

	uint16_t count = (uint16_t)color->contacts.count;
	if ( count == 0 )
	{
		return;
	}

	b3SolverBlock block = {
		.startIndex = 0,
		.count = count,
		.blockType = b3_overflowBlock,
		.colorIndex = B3_OVERFLOW_INDEX,
	};

	b3StoreImpulses_Mesh( block, context, 0 );
}
