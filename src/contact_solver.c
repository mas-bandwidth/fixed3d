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

	// Used for friction center weighting.
	b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;

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
			b3Contact* contact = b3Array_Get( world->contacts, contactId );
			B3_ASSERT( contact->contactId == contactId );

			int indexA = contact->bodySimIndexA;
			int indexB = contact->bodySimIndexB;

#if B3_ENABLE_VALIDATION
			if ( indexA != B3_NULL_INDEX )
			{
				b3Body* bodyA = b3Array_Get( world->bodies, contact->edges[0].bodyId );
				B3_ASSERT( indexA == bodyA->localIndex );
			}

			if ( indexB != B3_NULL_INDEX )
			{
				b3Body* bodyB = b3Array_Get( world->bodies, contact->edges[1].bodyId );
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
				b3Fixed totalFrictionWeight = B3_FIX( 0.0f );

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					b3ManifoldConstraintPoint* cp = constraint->points + pointIndex;

					// Copy data from manifold point
					b3ManifoldPoint* mp = manifold->points + pointIndex;
					cp->rA = mp->anchorA;
					cp->rB = mp->anchorB;

					b3Fixed s = mp->separation;
					cp->baseSeparation = s - b3Dot( b3Sub( cp->rB, cp->rA ), normal );
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

					// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
					// Contacts with separation greater than twice the speculative distance only matter for CCD and
					// should not contribute to the friction center. They are not important for jitter reduction. Closer
					// points may begin to touch on and off, so the friction center needs to move smoothly.
					// Floor to avoid a divide by zero below. Small enough to get washed out normally.
					b3Fixed weight =
						b3FixClamp( B3_FIX( 2.0f ) - b3FixDiv( s, speculativeDistance ), B3_MIN_FRICTION_WEIGHT, B3_FIX( 1.0f ) );
					centerA = b3MulAdd( centerA, weight, rA );
					centerB = b3MulAdd( centerB, weight, rB );
					totalFrictionWeight += weight;
				}

				b3Fixed invWeight = b3FixDiv( B3_FIX( 1.0f ) , totalFrictionWeight );
				centerA = b3MulSV( invWeight, centerA );
				centerB = b3MulSV( invWeight, centerB );
				constraint->centerA = centerA;
				constraint->centerB = centerB;

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
	b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
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
				b3Vec3 rA = constraint->centerA;
				b3Vec3 rB = constraint->centerB;
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

// ---------------------------------------------------------------------------
// Impulse-cap clamp guards
//
// The friction and rolling-resistance clamps compare a squared impulse length
// against b3FixMul( maxImpulse, maxImpulse ). These caps are physical values
// (friction * normal impulse sum), and the 64-bit squared forms wrap int64
// once an operand reaches 2^23.5 ~ 1.19e7 units. A wrapped compare can fire
// the clamp against a wrapped squared length whose b3FixSqrt is tiny or zero,
// so the "clamp" SCALES THE IMPULSE UP - energy injection, the same failure
// class as the joint sentinel caps (b3ImpulseOverCap in joint.h) except these
// operands are reachable by heavy high-friction content instead of sentinels
// (convex_pile at friction 0.6 already measures caps within 12x of the wrap;
// the multipliers - friction, density, body scale - are all linear).
//
// Every clamp gates its compare operands on B3_IMPULSE_CAP_GATE: below it no
// 64-bit intermediate can wrap (bound proof at the constant) and the
// historical code runs unchanged, so all existing content is bit-identical.
// The cold path mirrors each site's exact 64-bit rounding at 128 bits - any
// input the 64-bit forms handle without wrapping clamps identically, and
// inputs that would have wrapped clamp correctly instead. The gate also makes
// the hot path's int64 additions unreachable for operands that could overflow
// them (signed overflow is UB). The central-twist caps are plain symmetric
// clamps with no squaring and stay unguarded: b3FixMul( friction,
// totalTwistLimit ) cannot wrap before the impulses feeding it wrap first.

// 2^38 raw = 4.19e6 units. With every operand |q| < 2^38: a rounded square
// ( q*q + half ) >> 16 stays below 2^60, the unfused two-term sum below 2^61,
// and the fused three-term raw dot below 3 * 2^76 so its rounded form stays
// below 2^62 - nothing reaches 2^63.
#define B3_IMPULSE_CAP_GATE ( (b3Fixed)1 << 38 )

// Every operand inside [-B3_IMPULSE_CAP_GATE, B3_IMPULSE_CAP_GATE), tested
// with one compare: bias each operand by the gate (in-range values land in
// [0, 2^39)) and OR the results — OR never clears bits, so any out-of-range
// operand (biased >= 2^39, or wrapped huge for very negative values) keeps a
// bit at or above position 39 set.
static inline bool b3CapOperandsSafe3( b3Fixed a, b3Fixed b, b3Fixed c )
{
	uint64_t g = (uint64_t)B3_IMPULSE_CAP_GATE;
	uint64_t o = ( (uint64_t)a + g ) | ( (uint64_t)b + g ) | ( (uint64_t)c + g );
	return o < 2 * g;
}

static inline bool b3CapOperandsSafe4( b3Fixed a, b3Fixed b, b3Fixed c, b3Fixed d )
{
	uint64_t g = (uint64_t)B3_IMPULSE_CAP_GATE;
	uint64_t o = ( (uint64_t)a + g ) | ( (uint64_t)b + g ) | ( (uint64_t)c + g ) | ( (uint64_t)d + g );
	return o < 2 * g;
}

// Cold path for the 2D friction clamp. Exact over-cap test mirroring the hot
// path's rounding (b3Dot2 is unfused: two b3FixMul roundings, then the sum);
// when the clamp fires, the divisor length comes from the 64-bit value when
// that value did not wrap (bit-identical to the hot path) and from the raw
// 128-bit sum when it did. Returns true and writes *scale when the stored
// impulse must be rescaled.
static bool b3ClampImpulseCold2( b3Fixed x, b3Fixed y, b3Fixed maxImpulse, bool epsDivisor, b3Fixed* scale )
{
	b3Int128 xx = ( (b3Int128)x * x + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS;
	b3Int128 yy = ( (b3Int128)y * y + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS;
	b3Int128 lengthSquared = xx + yy;
	b3Int128 cap = ( (b3Int128)maxImpulse * maxImpulse + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS;
	if ( lengthSquared <= cap )
	{
		return false;
	}

	b3Fixed length;
	if ( lengthSquared <= (b3Int128)INT64_MAX )
	{
		length = b3FixSqrt( (b3Fixed)lengthSquared );
	}
	else
	{
		b3UInt128 raw = (b3UInt128)( (b3Int128)x * x ) + (b3UInt128)( (b3Int128)y * y ); // Q32.32
		length = (b3Fixed)b3ISqrt128High( (uint64_t)( raw >> 64 ), (uint64_t)raw );
	}

	*scale = b3FixDiv( maxImpulse, epsDivisor ? length + B3_FIXED_EPSILON : length );
	return true;
}

// Cold path for the 3D rolling-resistance clamp (fused: one rounding on the
// exact 128-bit dot, and the rolling sites add B3_FIXED_EPSILON to the
// squared cap before comparing).
static bool b3ClampImpulseCold3( b3Fixed x, b3Fixed y, b3Fixed z, b3Fixed maxImpulse, bool epsDivisor, b3Fixed* scale )
{
	b3Int128 raw = (b3Int128)x * x + (b3Int128)y * y + (b3Int128)z * z; // Q32.32
	b3Int128 magSqr = ( raw + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS;
	b3Int128 cap = ( ( (b3Int128)maxImpulse * maxImpulse + B3_FIXED_HALF ) >> B3_FIXED_FRACTION_BITS ) + B3_FIXED_EPSILON;
	if ( magSqr <= cap )
	{
		return false;
	}

	b3Fixed length;
	if ( magSqr <= (b3Int128)INT64_MAX )
	{
		length = b3FixSqrt( (b3Fixed)magSqr );
	}
	else
	{
		length = (b3Fixed)b3ISqrt128High( (uint64_t)( (b3UInt128)raw >> 64 ), (uint64_t)raw );
	}

	*scale = b3FixDiv( maxImpulse, epsDivisor ? length + B3_FIXED_EPSILON : length );
	return true;
}

#if defined( BOX3D_RANGE_AUDIT )

// Range audit for the Q16.16 32-bit lane feasibility study: tracks the maximum
// |value| of each quantity flowing through the contact solvers. Build with
// -DBOX3D_RANGE_AUDIT, run the benchmarks, and a report prints at process exit.
// A 32-bit Q16.16 lane holds |value| < 32768 with the same 1/65536 resolution.
// The impulse-cap channels compare against a different limit: the squared cap
// compares (b3FixMul( maxImpulse, maxImpulse ) and the b3Dot/b3Dot2 length
// forms) wrap int64 when the operand reaches 2^23.5 ~ 1.19e7 units.

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>

enum
{
	b3_auditBodyV,
	b3_auditBodyW,
	b3_auditDeltaPos,
	b3_auditAnchor,
	b3_auditRn,
	b3_auditIRn,
	b3_auditInvMass,
	b3_auditInvInertia,
	b3_auditNormalMass,
	b3_auditTangentMass,
	b3_auditTwistMass,
	b3_auditMNormal,
	b3_auditINormal,
	b3_auditRt,
	b3_auditVn,
	b3_auditBias,
	b3_auditNormalImpulse,
	b3_auditTotalNormalImpulse,
	b3_auditFrictionImpulse,
	b3_auditTwistImpulse,
	b3_auditSeparation,
	b3_auditIterNormalSum,
	b3_auditFrictionCap,
	b3_auditRollingCap,
	b3_auditTwistCap,
	b3_auditCount
};

static const char* b3_auditNames[b3_auditCount] = {
	"body v",		 "body w",		   "delta pos",		 "anchor",		   "cross(r,n)",	 "invI*cross(r,n)",
	"inv mass",		 "inv inertia",	   "normal mass",	 "tangent mass",   "twist mass",	 "invMass*n",
	"invI*n",		 "cross(o,t)",	   "vn",			 "bias",		   "normal impulse", "total normal impulse",
	"friction impulse", "twist impulse", "separation",	 "iter normal sum", "friction cap",	 "rolling cap",
	"twist cap",
};

static _Atomic int64_t b3_auditMax[b3_auditCount];
static atomic_bool b3_auditRegistered;

static void b3RangeAuditReport( void )
{
	fprintf( stderr, "\n=== contact solver range audit (max |value| in units, Q16.16 limit 32768) ===\n" );
	fprintf( stderr, "    (squared-cap wrap limit for the *cap channels: 2^23.5 ~ 11.86e6 units)\n" );
	for ( int i = 0; i < b3_auditCount; ++i )
	{
		double v = (double)atomic_load( b3_auditMax + i ) / 65536.0;
		fprintf( stderr, "%22s  %16.4f%s\n", b3_auditNames[i], v, v >= 4096.0 ? "  <-- LOW HEADROOM" : "" );
	}
}

static inline void b3AuditFix( int slot, b3Fixed value )
{
	if ( atomic_exchange_explicit( &b3_auditRegistered, true, memory_order_relaxed ) == false )
	{
		atexit( b3RangeAuditReport );
	}

	int64_t magnitude = value < 0 ? -value : value;
	int64_t seen = atomic_load_explicit( b3_auditMax + slot, memory_order_relaxed );
	while ( magnitude > seen && !atomic_compare_exchange_weak_explicit( b3_auditMax + slot, &seen, magnitude,
																	    memory_order_relaxed, memory_order_relaxed ) )
	{
	}
}

static inline void b3AuditV3( int slot, b3Vec3 value )
{
	b3AuditFix( slot, value.x );
	b3AuditFix( slot, value.y );
	b3AuditFix( slot, value.z );
}

static inline void b3AuditM3( int slot, b3Matrix3 value )
{
	b3AuditV3( slot, value.cx );
	b3AuditV3( slot, value.cy );
	b3AuditV3( slot, value.cz );
}

#define B3_AUDIT_FIX( slot, value ) b3AuditFix( slot, value )
#define B3_AUDIT_V3( slot, value ) b3AuditV3( slot, value )
#define B3_AUDIT_M3( slot, value ) b3AuditM3( slot, value )

#else

#define B3_AUDIT_FIX( slot, value )
#define B3_AUDIT_V3( slot, value )
#define B3_AUDIT_M3( slot, value )

#endif

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
				B3_AUDIT_FIX( b3_auditTwistCap, maxImpulse );
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
				B3_AUDIT_FIX( b3_auditRollingCap, maxImpulse );
				// The squared compare wraps int64 past B3_IMPULSE_CAP_GATE; the cold
				// path is exact (see the cap-guard block above this function)
				if ( b3CapOperandsSafe4( constraint->rollingImpulse.x, constraint->rollingImpulse.y,
										 constraint->rollingImpulse.z, maxImpulse ) )
				{
					b3Fixed magSqr = b3Dot( constraint->rollingImpulse, constraint->rollingImpulse );
					if ( magSqr > b3FixMul( maxImpulse , maxImpulse ) + B3_FIXED_EPSILON )
					{
						constraint->rollingImpulse =
							b3MulSV( b3FixDiv( maxImpulse , b3FixSqrt( magSqr ) ), constraint->rollingImpulse );
					}
				}
				else
				{
					b3Fixed scale;
					if ( b3ClampImpulseCold3( constraint->rollingImpulse.x, constraint->rollingImpulse.y,
											  constraint->rollingImpulse.z, maxImpulse, false, &scale ) )
					{
						constraint->rollingImpulse = b3MulSV( scale, constraint->rollingImpulse );
					}
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
				b3Vec3 rA = constraint->centerA;
				b3Vec3 rB = constraint->centerB;

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
				B3_AUDIT_FIX( b3_auditIterNormalSum, totalNormalImpulse );
				B3_AUDIT_FIX( b3_auditFrictionCap, maxImpulse );

				// Clamp the accumulated impulse. The squared compare wraps int64 past
				// B3_IMPULSE_CAP_GATE; the cold path is exact (see the cap-guard block
				// above this function).
				if ( b3CapOperandsSafe3( newImpulse.x, newImpulse.y, maxImpulse ) )
				{
					b3Fixed lengthSquared = b3Dot2( newImpulse, newImpulse );
					if ( lengthSquared > b3FixMul( maxImpulse , maxImpulse ) )
					{
						b3Fixed scale = b3FixDiv( maxImpulse , b3FixSqrt( lengthSquared ) );
						newImpulse.x = b3FixMul( newImpulse.x, scale );
						newImpulse.y = b3FixMul( newImpulse.y, scale );
					}
				}
				else
				{
					b3Fixed scale;
					if ( b3ClampImpulseCold2( newImpulse.x, newImpulse.y, maxImpulse, false, &scale ) )
					{
						newImpulse.x = b3FixMul( newImpulse.x, scale );
						newImpulse.y = b3FixMul( newImpulse.y, scale );
					}
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


#if defined( B3_SIMD_AVX512 )

#include <immintrin.h>

// Four Q48.16 lanes in one 256-bit register. The anonymous struct keeps all
// per-lane member access (gather/scatter, lane stores, div/sqrt) working
// unchanged; union member punning is the sanctioned C way to view the lanes
// both ways. Requires B3_ALIGNMENT >= 32 (see core.h) because wide constraint
// blocks are accessed with aligned vector loads and stores.
typedef union b3FloatW
{
	struct
	{
		b3Fixed x, y, z, w;
	};
	__m256i v;
} b3FloatW;

#else

// scalar math
typedef struct b3FloatW
{
	b3Fixed x, y, z, w;
} b3FloatW;

#endif

// Build from four lanes. Member assignment instead of a compound literal so
// the same code initializes both the struct and the union form warning-free.
static inline b3FloatW b3MakeW( b3Fixed x, b3Fixed y, b3Fixed z, b3Fixed w )
{
	b3FloatW r;
	r.x = x;
	r.y = y;
	r.z = z;
	r.w = w;
	return r;
}


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

// Narrow Q16.16 storage for constraint geometry. These values are bounded by
// hull extents (range audit: < 225 units, limit 32768), so the low 32 bits of
// the Q48.16 representation hold them losslessly and widening is exact. This
// halves the memory the solver streams per iteration; all arithmetic stays
// 64-bit after widening.
typedef struct b3FloatWN
{
	int32_t x, y, z, w;
} b3FloatWN;

typedef struct b3Vec3WN
{
	b3FloatWN X, Y, Z;
} b3Vec3WN;

static inline b3FloatW b3WidenW( b3FloatWN a )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_cvtepi32_epi64( _mm_loadu_si128( (const __m128i*)&a ) ) };
#else
	return (b3FloatW){ a.x, a.y, a.z, a.w };
#endif
}

static inline b3Vec3W b3WidenVW( b3Vec3WN a )
{
	return (b3Vec3W){ b3WidenW( a.X ), b3WidenW( a.Y ), b3WidenW( a.Z ) };
}

// Store one lane of a narrow field. Values are bounded by hull extents; the
// clamp keeps a pathological world deterministic instead of wrapping.
static inline void b3StoreNarrow( b3FloatWN* target, int lane, b3Fixed value )
{
	B3_ASSERT( -INT32_MAX <= value && value <= INT32_MAX );
	( (int32_t*)target )[lane] = (int32_t)b3FixClamp( value, -(b3Fixed)INT32_MAX, (b3Fixed)INT32_MAX );
}


#if defined( B3_SIMD_AVX512 )

// ---- AVX-512 exact Q48.16 lane arithmetic ---------------------------------
//
// Bit-identical to the scalar helpers for ALL inputs: exact intermediates, one
// round-half-up on the final shift, results truncated mod 2^64 exactly like
// the scalar int128-then-cast paths. This implements the default wrapping
// b3FixMul only; core.h keeps this path off under BOX3D_FIXED_SATURATE.
//
// The key identity, with bh = b >> 16 (arithmetic) and bl = b & 0xffff:
//
//     a * b == ((a * bh) << 16) + a * bl
//
// vpmullq gives a * bh mod 2^64 (wrap-consistent under the final shifts), and
// a * bl is made exact by splitting a into 32-bit halves:
//
//     a * bl == ((ahi * bl) << 32) + alo * bl
//
// where |ahi * bl| < 2^47 (vpmuldq) and alo * bl < 2^48 (vpmuludq) are exact
// in 64-bit lanes. Zen 4 executes vpmullq as a single fast uop, so a full
// four-lane fixed multiply costs three hardware multiplies plus shifts/adds.

// (a * b + B3_FIXED_HALF) >> B3_FIXED_FRACTION_BITS per lane; == b3FixMul.
// (a * bl + half) >> 16 == ((ahi * bl) << 16) + ((alo * bl + half) >> 16)
// because the first addend is a multiple of 2^16 and alo * bl + half >= 0.
static inline __m256i b3MulLanesAVX( __m256i a, __m256i b )
{
	__m256i bh = _mm256_srai_epi64( b, B3_FIXED_FRACTION_BITS );
	__m256i bl = _mm256_and_si256( b, _mm256_set1_epi64x( 0xFFFF ) );
	__m256i ahi = _mm256_srai_epi64( a, 32 );
	__m256i t = _mm256_mullo_epi64( a, bh );
	__m256i p1 = _mm256_mul_epu32( a, bl );	  // alo * bl
	__m256i p2 = _mm256_mul_epi32( ahi, bl ); // ahi * bl
	__m256i low = _mm256_srli_epi64( _mm256_add_epi64( p1, _mm256_set1_epi64x( B3_FIXED_HALF ) ), B3_FIXED_FRACTION_BITS );
	return _mm256_add_epi64( t, _mm256_add_epi64( _mm256_slli_epi64( p2, B3_FIXED_FRACTION_BITS ), low ) );
}

// Reduction accumulator for the raw-128 dot helpers. A sum of products is
// carried as three 64-bit lanes per element with no carry propagation:
//
//     sum(+-a_i * b_i) == (K << 16) + (P2 << 32) + M   (mod 2^80)
//
// K accumulates the wrapped a * bh terms (2^16 * (x mod 2^64) is congruent to
// 2^16 * x mod 2^80), while P2 (|term| < 2^47) and M (term < 2^48) stay exact
// for up to nine products plus a doubling: |P2| < 2^51, |M| < 2^53, both far
// below the 2^63 lane limit. The final rounded shift needs only bits 16..79
// of the sum, which these three parts determine.
typedef struct b3DotAccAVX
{
	__m256i K, P2, M;
} b3DotAccAVX;

static inline b3DotAccAVX b3DotBeginAVX( __m256i a, __m256i b )
{
	b3DotAccAVX acc;
	__m256i bh = _mm256_srai_epi64( b, B3_FIXED_FRACTION_BITS );
	__m256i bl = _mm256_and_si256( b, _mm256_set1_epi64x( 0xFFFF ) );
	__m256i ahi = _mm256_srai_epi64( a, 32 );
	acc.K = _mm256_mullo_epi64( a, bh );
	acc.P2 = _mm256_mul_epi32( ahi, bl );
	acc.M = _mm256_mul_epu32( a, bl );
	return acc;
}

static inline b3DotAccAVX b3DotAddAVX( b3DotAccAVX acc, __m256i a, __m256i b )
{
	b3DotAccAVX t = b3DotBeginAVX( a, b );
	acc.K = _mm256_add_epi64( acc.K, t.K );
	acc.P2 = _mm256_add_epi64( acc.P2, t.P2 );
	acc.M = _mm256_add_epi64( acc.M, t.M );
	return acc;
}

static inline b3DotAccAVX b3DotSubAVX( b3DotAccAVX acc, __m256i a, __m256i b )
{
	b3DotAccAVX t = b3DotBeginAVX( a, b );
	acc.K = _mm256_sub_epi64( acc.K, t.K );
	acc.P2 = _mm256_sub_epi64( acc.P2, t.P2 );
	acc.M = _mm256_sub_epi64( acc.M, t.M );
	return acc;
}

// Doubling for the 2*(a*b +- c*d) rotation matrix entries: exact on P2 and M,
// wrap-consistent on K. Matches the scalar << 1 on the raw 128-bit sum.
static inline b3DotAccAVX b3DotDoubleAVX( b3DotAccAVX acc )
{
	acc.K = _mm256_slli_epi64( acc.K, 1 );
	acc.P2 = _mm256_slli_epi64( acc.P2, 1 );
	acc.M = _mm256_slli_epi64( acc.M, 1 );
	return acc;
}

static inline b3DotAccAVX b3DotNegAVX( b3DotAccAVX acc )
{
	__m256i zero = _mm256_setzero_si256();
	acc.K = _mm256_sub_epi64( zero, acc.K );
	acc.P2 = _mm256_sub_epi64( zero, acc.P2 );
	acc.M = _mm256_sub_epi64( zero, acc.M );
	return acc;
}

// == b3FixFromDotRaw of the accumulated sum: one round-half-up, mod 2^64.
// (M + half) >> 16 must be an arithmetic shift; M can be negative.
static inline __m256i b3DotFinishAVX( b3DotAccAVX acc )
{
	__m256i low = _mm256_srai_epi64( _mm256_add_epi64( acc.M, _mm256_set1_epi64x( B3_FIXED_HALF ) ), B3_FIXED_FRACTION_BITS );
	__m256i hi = _mm256_add_epi64( acc.K, _mm256_slli_epi64( acc.P2, B3_FIXED_FRACTION_BITS ) );
	return _mm256_add_epi64( hi, low );
}

// 0 for false lanes, B3_FIXED_ONE for true lanes, matching the scalar masks
static inline b3FloatW b3MaskW( __mmask8 k )
{
	return (b3FloatW){ .v = _mm256_maskz_mov_epi64( k, _mm256_set1_epi64x( B3_FIXED_ONE ) ) };
}

#endif


static inline b3FloatW b3ZeroW( void )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_setzero_si256() };
#else
	return (b3FloatW){ B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
#endif
}

static inline b3FloatW b3SplatW( b3Fixed scalar )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_set1_epi64x( scalar ) };
#else
	return (b3FloatW){ scalar, scalar, scalar, scalar };
#endif
}

static inline b3FloatW b3NegW( b3FloatW a )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_sub_epi64( _mm256_setzero_si256(), a.v ) };
#else
	return (b3FloatW){ -a.x, -a.y, -a.z, -a.w };
#endif
}

static inline b3FloatW b3AddW( b3FloatW a, b3FloatW b )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_add_epi64( a.v, b.v ) };
#else
	return (b3FloatW){ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
#endif
}

static inline b3FloatW b3SubW( b3FloatW a, b3FloatW b )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_sub_epi64( a.v, b.v ) };
#else
	return (b3FloatW){ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
#endif
}

static inline b3FloatW b3MulW( b3FloatW a, b3FloatW b )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = b3MulLanesAVX( a.v, b.v ) };
#else
	return (b3FloatW){ b3FixMul( a.x , b.x ), b3FixMul( a.y , b.y ), b3FixMul( a.z , b.z ), b3FixMul( a.w , b.w ) };
#endif
}

// Division and square root stay per-lane scalar in both paths: they are
// data-dependent iterative operations with no exact SIMD equivalent.
static inline b3FloatW b3DivW( b3FloatW a, b3FloatW b )
{
	return b3MakeW( b3FixDiv( a.x , b.x ), b3FixDiv( a.y , b.y ), b3FixDiv( a.z , b.z ), b3FixDiv( a.w , b.w ) );
}

static inline b3FloatW b3SqrtW( b3FloatW a )
{
	return b3MakeW( b3FixSqrt( a.x ), b3FixSqrt( a.y ), b3FixSqrt( a.z ), b3FixSqrt( a.w ) );
}

static inline b3FloatW b3MulAddW( b3FloatW a, b3FloatW b, b3FloatW c )
{
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_add_epi64( a.v, b3MulLanesAVX( b.v, c.v ) ) };
#else
	return (b3FloatW){ a.x + b3FixMul( b.x , c.x ), a.y + b3FixMul( b.y , c.y ), a.z + b3FixMul( b.z , c.z ), a.w + b3FixMul( b.w , c.w ) };
#endif
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
#if defined( B3_SIMD_AVX512 )
	return (b3FloatW){ .v = _mm256_max_epi64( a.v, b.v ) };
#else
	b3FloatW r;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
#endif
}

// clamp a to [-b, b]
static inline b3FloatW b3SymClampW( b3FloatW a, b3FloatW b )
{
#if defined( B3_SIMD_AVX512 )
	__m256i r = _mm256_min_epi64( a.v, b.v );
	r = _mm256_max_epi64( r, _mm256_sub_epi64( _mm256_setzero_si256(), b.v ) );
	return (b3FloatW){ .v = r };
#else
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
#endif
}

static inline b3FloatW b3OrW( b3FloatW a, b3FloatW b )
{
#if defined( B3_SIMD_AVX512 )
	__m256i o = _mm256_or_si256( a.v, b.v );
	return b3MaskW( _mm256_test_epi64_mask( o, o ) );
#else
	b3FloatW r;
	r.x = a.x != B3_FIX( 0.0f ) || b.x != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.y = a.y != B3_FIX( 0.0f ) || b.y != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.z = a.z != B3_FIX( 0.0f ) || b.z != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.w = a.w != B3_FIX( 0.0f ) || b.w != B3_FIX( 0.0f ) ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	return r;
#endif
}

static inline b3FloatW b3GreaterThanW( b3FloatW a, b3FloatW b )
{
#if defined( B3_SIMD_AVX512 )
	return b3MaskW( _mm256_cmpgt_epi64_mask( a.v, b.v ) );
#else
	b3FloatW r;
	r.x = a.x > b.x ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.y = a.y > b.y ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.z = a.z > b.z ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.w = a.w > b.w ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	return r;
#endif
}

static inline b3FloatW b3EqualsW( b3FloatW a, b3FloatW b )
{
#if defined( B3_SIMD_AVX512 )
	return b3MaskW( _mm256_cmpeq_epi64_mask( a.v, b.v ) );
#else
	b3FloatW r;
	r.x = a.x == b.x ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.y = a.y == b.y ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.z = a.z == b.z ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	r.w = a.w == b.w ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );
	return r;
#endif
}

static inline bool b3AllZeroW( b3FloatW a )
{
#if defined( B3_SIMD_AVX512 )
	return _mm256_testz_si256( a.v, a.v ) != 0;
#else
	return a.x == B3_FIX( 0.0f ) && a.y == B3_FIX( 0.0f ) && a.z == B3_FIX( 0.0f ) && a.w == B3_FIX( 0.0f );
#endif
}

// true when every lane of every operand vector is inside the cap gate, so the
// impulse-cap clamps may run their 64-bit hot path (see the cap-guard block
// above b3SolveContacts_Mesh). Same bias-and-OR fold as the scalar helpers:
// one compare for the whole gate.
static inline bool b3CapOperandsSafeW3( b3FloatW a, b3FloatW b, b3FloatW c )
{
#if defined( B3_SIMD_AVX512 )
	__m256i g = _mm256_set1_epi64x( B3_IMPULSE_CAP_GATE );
	__m256i o = _mm256_or_si256( _mm256_add_epi64( a.v, g ),
								 _mm256_or_si256( _mm256_add_epi64( b.v, g ), _mm256_add_epi64( c.v, g ) ) );
	return _mm256_cmp_epu64_mask( o, _mm256_set1_epi64x( 2 * B3_IMPULSE_CAP_GATE ), _MM_CMPINT_LT ) == 0xF;
#else
	uint64_t g = (uint64_t)B3_IMPULSE_CAP_GATE;
	uint64_t o = ( ( (uint64_t)a.x + g ) | ( (uint64_t)a.y + g ) | ( (uint64_t)a.z + g ) | ( (uint64_t)a.w + g ) ) |
				 ( ( (uint64_t)b.x + g ) | ( (uint64_t)b.y + g ) | ( (uint64_t)b.z + g ) | ( (uint64_t)b.w + g ) ) |
				 ( ( (uint64_t)c.x + g ) | ( (uint64_t)c.y + g ) | ( (uint64_t)c.z + g ) | ( (uint64_t)c.w + g ) );
	return o < 2 * g;
#endif
}

static inline bool b3CapOperandsSafeW4( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
#if defined( B3_SIMD_AVX512 )
	__m256i g = _mm256_set1_epi64x( B3_IMPULSE_CAP_GATE );
	__m256i o = _mm256_or_si256( _mm256_or_si256( _mm256_add_epi64( a.v, g ), _mm256_add_epi64( b.v, g ) ),
								 _mm256_or_si256( _mm256_add_epi64( c.v, g ), _mm256_add_epi64( d.v, g ) ) );
	return _mm256_cmp_epu64_mask( o, _mm256_set1_epi64x( 2 * B3_IMPULSE_CAP_GATE ), _MM_CMPINT_LT ) == 0xF;
#else
	uint64_t g = (uint64_t)B3_IMPULSE_CAP_GATE;
	uint64_t o = ( ( (uint64_t)a.x + g ) | ( (uint64_t)a.y + g ) | ( (uint64_t)a.z + g ) | ( (uint64_t)a.w + g ) ) |
				 ( ( (uint64_t)b.x + g ) | ( (uint64_t)b.y + g ) | ( (uint64_t)b.z + g ) | ( (uint64_t)b.w + g ) ) |
				 ( ( (uint64_t)c.x + g ) | ( (uint64_t)c.y + g ) | ( (uint64_t)c.z + g ) | ( (uint64_t)c.w + g ) ) |
				 ( ( (uint64_t)d.x + g ) | ( (uint64_t)d.y + g ) | ( (uint64_t)d.z + g ) | ( (uint64_t)d.w + g ) );
	return o < 2 * g;
#endif
}

// Cold per-lane fix-ups for the wide impulse-cap clamps: exact clamp mask
// (B3_FIXED_ONE / 0, matching b3GreaterThanW) and rescale factor per lane.
// Lanes that do not clamp get scale B3_FIXED_ONE, which the callers blend
// away exactly like the hot path's unused divide results.
static void b3ClampImpulseColdW2( b3FloatW x, b3FloatW y, b3FloatW maxImpulse, bool epsDivisor, b3FloatW* maskOut,
								  b3FloatW* scaleOut )
{
	b3FloatW mask = b3ZeroW();
	b3FloatW scale = b3SplatW( B3_FIXED_ONE );
	b3Fixed s;
	if ( b3ClampImpulseCold2( x.x, y.x, maxImpulse.x, epsDivisor, &s ) )
	{
		mask.x = B3_FIXED_ONE;
		scale.x = s;
	}
	if ( b3ClampImpulseCold2( x.y, y.y, maxImpulse.y, epsDivisor, &s ) )
	{
		mask.y = B3_FIXED_ONE;
		scale.y = s;
	}
	if ( b3ClampImpulseCold2( x.z, y.z, maxImpulse.z, epsDivisor, &s ) )
	{
		mask.z = B3_FIXED_ONE;
		scale.z = s;
	}
	if ( b3ClampImpulseCold2( x.w, y.w, maxImpulse.w, epsDivisor, &s ) )
	{
		mask.w = B3_FIXED_ONE;
		scale.w = s;
	}
	*maskOut = mask;
	*scaleOut = scale;
}

static void b3ClampImpulseColdW3( b3Vec3W v, b3FloatW maxImpulse, bool epsDivisor, b3FloatW* maskOut, b3FloatW* scaleOut )
{
	b3FloatW mask = b3ZeroW();
	b3FloatW scale = b3SplatW( B3_FIXED_ONE );
	b3Fixed s;
	if ( b3ClampImpulseCold3( v.X.x, v.Y.x, v.Z.x, maxImpulse.x, epsDivisor, &s ) )
	{
		mask.x = B3_FIXED_ONE;
		scale.x = s;
	}
	if ( b3ClampImpulseCold3( v.X.y, v.Y.y, v.Z.y, maxImpulse.y, epsDivisor, &s ) )
	{
		mask.y = B3_FIXED_ONE;
		scale.y = s;
	}
	if ( b3ClampImpulseCold3( v.X.z, v.Y.z, v.Z.z, maxImpulse.z, epsDivisor, &s ) )
	{
		mask.z = B3_FIXED_ONE;
		scale.z = s;
	}
	if ( b3ClampImpulseCold3( v.X.w, v.Y.w, v.Z.w, maxImpulse.w, epsDivisor, &s ) )
	{
		mask.w = B3_FIXED_ONE;
		scale.w = s;
	}
	*maskOut = mask;
	*scaleOut = scale;
}

// component-wise returns mask ? b : a
static inline b3FloatW b3BlendW( b3FloatW a, b3FloatW b, b3FloatW mask )
{
#if defined( B3_SIMD_AVX512 )
	__mmask8 k = _mm256_test_epi64_mask( mask.v, mask.v );
	return (b3FloatW){ .v = _mm256_mask_blend_epi64( k, a.v, b.v ) };
#else
	b3FloatW r;
	r.x = mask.x != B3_FIX( 0.0f ) ? b.x : a.x;
	r.y = mask.y != B3_FIX( 0.0f ) ? b.y : a.y;
	r.z = mask.z != B3_FIX( 0.0f ) ? b.z : a.z;
	r.w = mask.w != B3_FIX( 0.0f ) ? b.w : a.w;
	return r;
#endif
}


// Per-lane 128-bit product reductions with a single rounding (divide last).
// Cheaper than chaining b3FixMul (one rounding and saturation check instead of
// one per product) and sub-resolution products don't quantize to zero.

// a*b + c*d
static inline b3FloatW b3Dot2W( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
#if defined( B3_SIMD_AVX512 )
	b3DotAccAVX sum = b3DotBeginAVX( a.v, b.v );
	sum = b3DotAddAVX( sum, c.v, d.v );
	return (b3FloatW){ .v = b3DotFinishAVX( sum ) };
#else
	return (b3FloatW){
		b3FixFromDotRaw( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x ),
		b3FixFromDotRaw( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y ),
		b3FixFromDotRaw( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z ),
		b3FixFromDotRaw( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w ),
	};
#endif
}

// a*b + c*d + e*f
static inline b3FloatW b3Dot3W( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d, b3FloatW e, b3FloatW f )
{
#if defined( B3_SIMD_AVX512 )
	b3DotAccAVX sum = b3DotBeginAVX( a.v, b.v );
	sum = b3DotAddAVX( sum, c.v, d.v );
	sum = b3DotAddAVX( sum, e.v, f.v );
	return (b3FloatW){ .v = b3DotFinishAVX( sum ) };
#else
	return (b3FloatW){
		b3FixFromDotRaw( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x + (b3Int128)e.x * f.x ),
		b3FixFromDotRaw( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y + (b3Int128)e.y * f.y ),
		b3FixFromDotRaw( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z + (b3Int128)e.z * f.z ),
		b3FixFromDotRaw( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w + (b3Int128)e.w * f.w ),
	};
#endif
}

// acc + a*b + c*d + e*f
static inline b3FloatW b3AddDot3W( b3FloatW acc, b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d, b3FloatW e, b3FloatW f )
{
#if defined( B3_SIMD_AVX512 )
	// (acc << 16) is a multiple of 2^16, so the single rounded shift splits
	// exactly: b3FixFromDotRaw((acc << 16) + S) == acc + b3FixFromDotRaw(S)
	return b3AddW( acc, b3Dot3W( a, b, c, d, e, f ) );
#else
	return (b3FloatW){
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.x, B3_FIXED_FRACTION_BITS ) + (b3Int128)a.x * b.x + (b3Int128)c.x * d.x + (b3Int128)e.x * f.x ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.y, B3_FIXED_FRACTION_BITS ) + (b3Int128)a.y * b.y + (b3Int128)c.y * d.y + (b3Int128)e.y * f.y ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.z, B3_FIXED_FRACTION_BITS ) + (b3Int128)a.z * b.z + (b3Int128)c.z * d.z + (b3Int128)e.z * f.z ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.w, B3_FIXED_FRACTION_BITS ) + (b3Int128)a.w * b.w + (b3Int128)c.w * d.w + (b3Int128)e.w * f.w ),
	};
#endif
}

// acc - (a*b + c*d + e*f)
static inline b3FloatW b3SubDot3W( b3FloatW acc, b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d, b3FloatW e, b3FloatW f )
{
#if defined( B3_SIMD_AVX512 )
	// Round-half-up is not odd-symmetric, so accumulate the negated sum and
	// round that, exactly like the scalar (acc << 16) - ab - cd - ef.
	b3DotAccAVX sum = b3DotBeginAVX( a.v, b.v );
	sum = b3DotAddAVX( sum, c.v, d.v );
	sum = b3DotAddAVX( sum, e.v, f.v );
	return b3AddW( acc, (b3FloatW){ .v = b3DotFinishAVX( b3DotNegAVX( sum ) ) } );
#else
	return (b3FloatW){
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.x, B3_FIXED_FRACTION_BITS ) - (b3Int128)a.x * b.x - (b3Int128)c.x * d.x - (b3Int128)e.x * f.x ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.y, B3_FIXED_FRACTION_BITS ) - (b3Int128)a.y * b.y - (b3Int128)c.y * d.y - (b3Int128)e.y * f.y ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.z, B3_FIXED_FRACTION_BITS ) - (b3Int128)a.z * b.z - (b3Int128)c.z * d.z - (b3Int128)e.z * f.z ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)acc.w, B3_FIXED_FRACTION_BITS ) - (b3Int128)a.w * b.w - (b3Int128)c.w * d.w - (b3Int128)e.w * f.w ),
	};
#endif
}

// Relative velocity along an axis using precomputed cross(r, axis) rows:
// dot(dv, axis) + dot(wB, rbxa) - dot(wA, raxa), nine products accumulated
// at 128 bits with a single rounding.
static inline b3FloatW b3RelVelocityW( b3Vec3W dv, b3Vec3W axis, b3Vec3W wB, b3Vec3W rbxa, b3Vec3W wA, b3Vec3W raxa )
{
#if defined( B3_SIMD_AVX512 )
	b3DotAccAVX sum = b3DotBeginAVX( dv.X.v, axis.X.v );
	sum = b3DotAddAVX( sum, dv.Y.v, axis.Y.v );
	sum = b3DotAddAVX( sum, dv.Z.v, axis.Z.v );
	sum = b3DotAddAVX( sum, wB.X.v, rbxa.X.v );
	sum = b3DotAddAVX( sum, wB.Y.v, rbxa.Y.v );
	sum = b3DotAddAVX( sum, wB.Z.v, rbxa.Z.v );
	sum = b3DotSubAVX( sum, wA.X.v, raxa.X.v );
	sum = b3DotSubAVX( sum, wA.Y.v, raxa.Y.v );
	sum = b3DotSubAVX( sum, wA.Z.v, raxa.Z.v );
	return (b3FloatW){ .v = b3DotFinishAVX( sum ) };
#else
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
#endif
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
#if defined( B3_SIMD_AVX512 )
	// one == 2^16 << 16, a multiple of 2^16, so it splits out of the rounded
	// shift exactly: result == 2^16 + b3FixFromDotRaw( -2 * (ab + cd) )
	b3DotAccAVX sum = b3DotBeginAVX( a.v, b.v );
	sum = b3DotAddAVX( sum, c.v, d.v );
	sum = b3DotNegAVX( b3DotDoubleAVX( sum ) );
	return (b3FloatW){ .v = _mm256_add_epi64( _mm256_set1_epi64x( B3_FIXED_ONE ), b3DotFinishAVX( sum ) ) };
#else
	const b3Int128 one = (b3Int128)B3_FIXED_ONE << B3_FIXED_FRACTION_BITS;
	return (b3FloatW){
		b3FixFromDotRaw( one - ( ( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x ) << 1 ) ),
		b3FixFromDotRaw( one - ( ( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y ) << 1 ) ),
		b3FixFromDotRaw( one - ( ( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z ) << 1 ) ),
		b3FixFromDotRaw( one - ( ( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w ) << 1 ) ),
	};
#endif
}

// 2*(a*b + c*d)
static inline b3FloatW b3RotAddW( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
#if defined( B3_SIMD_AVX512 )
	b3DotAccAVX sum = b3DotBeginAVX( a.v, b.v );
	sum = b3DotAddAVX( sum, c.v, d.v );
	return (b3FloatW){ .v = b3DotFinishAVX( b3DotDoubleAVX( sum ) ) };
#else
	return (b3FloatW){
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.x * b.x + (b3Int128)c.x * d.x, 1 ) ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.y * b.y + (b3Int128)c.y * d.y, 1 ) ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.z * b.z + (b3Int128)c.z * d.z, 1 ) ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.w * b.w + (b3Int128)c.w * d.w, 1 ) ),
	};
#endif
}

// 2*(a*b - c*d)
static inline b3FloatW b3RotSubW( b3FloatW a, b3FloatW b, b3FloatW c, b3FloatW d )
{
#if defined( B3_SIMD_AVX512 )
	b3DotAccAVX sum = b3DotBeginAVX( a.v, b.v );
	sum = b3DotSubAVX( sum, c.v, d.v );
	return (b3FloatW){ .v = b3DotFinishAVX( b3DotDoubleAVX( sum ) ) };
#else
	return (b3FloatW){
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.x * b.x - (b3Int128)c.x * d.x, 1 ) ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.y * b.y - (b3Int128)c.y * d.y, 1 ) ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.z * b.z - (b3Int128)c.z * d.z, 1 ) ),
		b3FixFromDotRaw( b3Int128ShiftLeft( (b3Int128)a.w * b.w - (b3Int128)c.w * d.w, 1 ) ),
	};
#endif
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

// exact int64 negation per lane
static inline b3Vec3W b3NegVW( b3Vec3W a )
{
	return (b3Vec3W){ b3NegW( a.X ), b3NegW( a.Y ), b3NegW( a.Z ) };
}

// component-wise returns mask ? b : a
static inline b3Vec3W b3BlendVW( b3Vec3W a, b3Vec3W b, b3FloatW mask )
{
	return (b3Vec3W){ b3BlendW( a.X, b.X, mask ), b3BlendW( a.Y, b.Y, mask ), b3BlendW( a.Z, b.Z, mask ) };
}

// Wide cross product with the scalar b3Cross rounding: per-product rounding,
// then the difference (deliberately NOT fused, see the round-3 notes).
static inline b3Vec3W b3CrossUnfusedW( b3Vec3W a, b3Vec3W b )
{
	return (b3Vec3W){
		b3SubW( b3MulW( a.Y, b.Z ), b3MulW( a.Z, b.Y ) ),
		b3SubW( b3MulW( a.Z, b.X ), b3MulW( a.X, b.Z ) ),
		b3SubW( b3MulW( a.X, b.Y ), b3MulW( a.Y, b.X ) ),
	};
}

// Full 3x3 matrix, four lanes per entry. invInertiaWorld = (R*I)*R^T rounds
// each entry independently, so it is NOT guaranteed bitwise symmetric; paths
// that must match the scalar b3MulMV bit for bit multiply through all nine
// entries.
typedef struct b3Matrix3FullW
{
	b3FloatW cxx, cxy, cxz; // column x
	b3FloatW cyx, cyy, cyz; // column y
	b3FloatW czx, czy, czz; // column z
} b3Matrix3FullW;

// Wide m * v with the scalar b3MulMV rounding: per-product rounding, plain
// adds (deliberately NOT fused, see the round-3 notes).
static inline b3Vec3W b3MulMVFullW( const b3Matrix3FullW* m, b3Vec3W a )
{
	return (b3Vec3W){
		b3AddW( b3AddW( b3MulW( m->cxx, a.X ), b3MulW( m->cyx, a.Y ) ), b3MulW( m->czx, a.Z ) ),
		b3AddW( b3AddW( b3MulW( m->cxy, a.X ), b3MulW( m->cyy, a.Y ) ), b3MulW( m->czy, a.Z ) ),
		b3AddW( b3AddW( b3MulW( m->cxz, a.X ), b3MulW( m->cyz, a.Y ) ), b3MulW( m->czz, a.Z ) ),
	};
}

// Symmetric 2x2 times vec2 with the scalar b3MulMV2 rounding: per-product
// rounding, plain adds. The fused b3MulMV2W does not match b3MulMV2.
static inline b3Vec2W b3MulMV2UnfusedW( b3SymMatrix2W m, b3Vec2W a )
{
	return (b3Vec2W){
		b3AddW( b3MulW( m.cxx, a.x ), b3MulW( m.cxy, a.y ) ),
		b3AddW( b3MulW( m.cxy, a.x ), b3MulW( m.cyy, a.y ) ),
	};
}

// Wide b3RotateVector with the exact scalar rounding: unfused crosses and
// per-product b3FixMul round-half-up. The fused b3Matrix3W rotation trick is
// NOT bit-identical to b3RotateVector (differential-tested: ulp differences on
// essentially every input), and the wide mesh path must match the scalar mesh
// solver bit for bit, so the two-cross form is required here.
static inline b3Vec3W b3RotateVectorW( b3QuatW q, b3Vec3W v )
{
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	b3Vec3W t1 = b3CrossUnfusedW( q.V, v );
	b3Vec3W t2 = b3MulAddSVW( t1, q.S, v );
	b3Vec3W t3 = b3CrossUnfusedW( q.V, t2 );
	return b3MulAddSVW( v, b3SplatW( B3_FIX( 2.0f ) ), t3 );
}

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

typedef struct b3ContactConstraintPointWide
{
	// Narrow Q16.16 storage: bounded by hull extents, widened exactly on load
	b3Vec3WN anchorAs, anchorBs;

	// Precomputed normal Jacobian rows: cross(r, normal) for the velocity
	// projection and invI * cross(r, normal) for the impulse application.
	// Each 128-bit fixed multiply costs several ops, so trading memory for
	// multiplies pays here (unlike the float SIMD solver).
	b3Vec3WN rnAs, rnBs;
	b3Vec3WN iRnAs, iRnBs;

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

	// Narrow Q16.16 storage: unit vectors and hull-extent-bounded Jacobian
	// terms, widened exactly on load. invMass * normal for the linear impulse,
	// invI * normal for the twist impulse, cross(origin, tangent) for central
	// friction.
	b3Vec3WN normal;
	b3Vec3WN mNormalA, mNormalB;
	b3Vec3WN iNormalA, iNormalB;
	b3Vec3WN rtA1s, rtA2s, rtB1s, rtB2s;

	// todo test computing the tangents on the fly, at least tangent2
	b3Vec3WN tangent1;
	b3Vec3WN tangent2;
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

	// Widest manifold across the four lanes, written by prepare (which visits
	// every wide constraint, so no zero initialization is needed). Warm start,
	// solve, and restitution loop only this far: the slots beyond it hold
	// exact zeros in every lane and contribute nothing, so skipping them is
	// bit-identical and one-point-manifold scenes skip 3/4 of the point work.
	int maxPointCount;

	b3ContactConstraintPointWide points[B3_MAX_MANIFOLD_POINTS];

} b3ContactConstraintWide;

int b3GetWideContactConstraintByteCount( void )
{
	return sizeof( b3ContactConstraintWide );
}

// ---- wide mesh contact constraints -----------------------------------------
//
// Colored mesh contacts (one contact may carry many manifolds) are solved four
// contacts per wide constraint: lane = whole contact, so graph coloring keeps
// the eight gathered bodies disjoint, and the manifold dimension serializes
// in-register exactly like the scalar Gauss-Seidel loop over manifolds.
// Every operation reproduces the scalar mesh solver's rounding bit for bit
// (unfused crosses and 3x3 products, full nine-entry inertia matrices,
// negate-before-multiply impulse forms, per-lane divides and square roots),
// so each lane is bit-identical to b3SolveContacts_Mesh and the overflow
// color (which stays on the scalar path) remains consistent with it.
// Inactive (lane, manifold) and (lane, point) slots hold exact zeros and
// compute exact zero deltas; the only place zeros are not self-neutralizing
// is rolling resistance (the rolling mass is per-contact), which blends the
// stored impulse per lane instead.

typedef struct b3ManifoldConstraintPointMeshWide
{
	b3Vec3W rA, rB;
	b3FloatW baseSeparation;
	b3FloatW normalImpulse;
	b3FloatW totalNormalImpulse;
	b3FloatW normalMass;
	b3FloatW leverArm;
	b3FloatW relativeVelocity;
} b3ManifoldConstraintPointMeshWide;

typedef struct b3ManifoldConstraintMeshWide
{
	b3ManifoldConstraintPointMeshWide points[B3_MAX_MANIFOLD_POINTS];
	b3Vec3W normal;
	b3Vec3W tangent1;
	b3Vec3W tangent2;
	// Friction centers
	b3Vec3W centerA, centerB;
	b3FloatW twistMass;
	b3FloatW twistImpulse;
	// b3Invert2 of a bitwise-symmetric matrix is bitwise symmetric, so the
	// three stored entries reproduce the scalar b3Matrix2 exactly
	b3SymMatrix2W tangentMass;
	b3Vec2W frictionImpulse;
	b3Vec3W rollingImpulse;
	b3FloatW tangentVelocity1;
	b3FloatW tangentVelocity2;

	// Widest lane pointCount for this manifold slot. Point slots past a lane's
	// own count hold exact zeros and visiting them is bit-identical.
	int maxPointCount;
} b3ManifoldConstraintMeshWide;

typedef struct b3ContactConstraintMeshWide
{
	// These are base 1
	int indexA[B3_SIMD_WIDTH];
	int indexB[B3_SIMD_WIDTH];

	b3FloatW invMassA, invMassB;

	// Full nine-entry matrices: the scalar solver multiplies through b3MulMV
	// with per-product rounding and (R*I)*R^T is not bitwise symmetric
	b3Matrix3FullW invIA, invIB;
	b3Matrix3FullW rollingMass;

	b3FloatW friction;
	b3FloatW restitution;
	b3FloatW rollingResistance;
	b3FloatW biasRate;
	b3FloatW massScale;
	b3FloatW impulseScale;

	// Per-lane manifold counts as raw integer lanes, for the rolling mask
	b3FloatW manifoldCounts;

	struct b3Contact* contacts[B3_SIMD_WIDTH];

	// maxManifoldCount wide manifold slots, sliced from the flat step-context
	// array by prepare using the per-slot start table from solver setup
	b3ManifoldConstraintMeshWide* manifoldConstraints;
	int maxManifoldCount;
} b3ContactConstraintMeshWide;

int b3GetMeshWideContactConstraintByteCount( void )
{
	return sizeof( b3ContactConstraintMeshWide );
}

int b3GetMeshWideManifoldConstraintByteCount( void )
{
	return sizeof( b3ManifoldConstraintMeshWide );
}

// wide version of b3BodyState
typedef struct b3BodyStateW
{
	b3Vec3W v;
	b3Vec3W w;
	b3Vec3W dp;
	b3QuatW dq;
} b3BodyStateW;

#if defined( BOX3D_RANGE_AUDIT )

// Wide-lane variants of the range audit helpers defined above the mesh solver
// (the scalar block sits up there because b3FloatW is not defined yet at that
// point in the file).

static inline void b3AuditW( int slot, b3FloatW value )
{
	b3AuditFix( slot, value.x );
	b3AuditFix( slot, value.y );
	b3AuditFix( slot, value.z );
	b3AuditFix( slot, value.w );
}

static inline void b3AuditV3W( int slot, b3Vec3W value )
{
	b3AuditW( slot, value.X );
	b3AuditW( slot, value.Y );
	b3AuditW( slot, value.Z );
}

#define B3_AUDIT_W( slot, value ) b3AuditW( slot, value )
#define B3_AUDIT_V3W( slot, value ) b3AuditV3W( slot, value )

#else

#define B3_AUDIT_W( slot, value )
#define B3_AUDIT_V3W( slot, value )

#endif


#if defined( B3_SIMD_AVX512 )

// 4x4 transpose of 64-bit lanes: rows in, columns out
static inline void b3Transpose4x4AVX( __m256i r0, __m256i r1, __m256i r2, __m256i r3, __m256i* c0, __m256i* c1, __m256i* c2,
									  __m256i* c3 )
{
	__m256i t0 = _mm256_unpacklo_epi64( r0, r1 );
	__m256i t1 = _mm256_unpackhi_epi64( r0, r1 );
	__m256i t2 = _mm256_unpacklo_epi64( r2, r3 );
	__m256i t3 = _mm256_unpackhi_epi64( r2, r3 );
	*c0 = _mm256_permute2x128_si256( t0, t2, 0x20 );
	*c1 = _mm256_permute2x128_si256( t1, t3, 0x20 );
	*c2 = _mm256_permute2x128_si256( t0, t2, 0x31 );
	*c3 = _mm256_permute2x128_si256( t1, t3, 0x31 );
}

// AoS -> SoA gather through three 4x4 transposes over the 13 contiguous
// b3Fixed fields of b3BodyState (deltaRotation.s stays scalar: a fourth
// vector load would read past the end of the states array for the last
// body). Pure data movement, bit-identical to the scalar member loads.
static b3BodyStateW b3GatherBodies( const b3BodyState* states, int* indices )
{
	_Static_assert( offsetof( b3BodyState, angularVelocity ) == 3 * sizeof( b3Fixed ), "b3BodyState layout" );
	_Static_assert( offsetof( b3BodyState, deltaPosition ) == 6 * sizeof( b3Fixed ), "b3BodyState layout" );
	_Static_assert( offsetof( b3BodyState, deltaRotation ) == 9 * sizeof( b3Fixed ), "b3BodyState layout" );

	const b3BodyState* s1 = indices[0] == 0 ? &b3_identityBodyState : states + indices[0] - 1;
	const b3BodyState* s2 = indices[1] == 0 ? &b3_identityBodyState : states + indices[1] - 1;
	const b3BodyState* s3 = indices[2] == 0 ? &b3_identityBodyState : states + indices[2] - 1;
	const b3BodyState* s4 = indices[3] == 0 ? &b3_identityBodyState : states + indices[3] - 1;

	b3BodyStateW simdBody;

	// fields 0..3: v.x v.y v.z w.x
	b3Transpose4x4AVX( _mm256_loadu_si256( (const __m256i*)&s1->linearVelocity.x ),
					   _mm256_loadu_si256( (const __m256i*)&s2->linearVelocity.x ),
					   _mm256_loadu_si256( (const __m256i*)&s3->linearVelocity.x ),
					   _mm256_loadu_si256( (const __m256i*)&s4->linearVelocity.x ), &simdBody.v.X.v, &simdBody.v.Y.v,
					   &simdBody.v.Z.v, &simdBody.w.X.v );

	// fields 4..7: w.y w.z dp.x dp.y
	b3Transpose4x4AVX( _mm256_loadu_si256( (const __m256i*)&s1->angularVelocity.y ),
					   _mm256_loadu_si256( (const __m256i*)&s2->angularVelocity.y ),
					   _mm256_loadu_si256( (const __m256i*)&s3->angularVelocity.y ),
					   _mm256_loadu_si256( (const __m256i*)&s4->angularVelocity.y ), &simdBody.w.Y.v, &simdBody.w.Z.v,
					   &simdBody.dp.X.v, &simdBody.dp.Y.v );

	// fields 8..11: dp.z dq.v.x dq.v.y dq.v.z
	b3Transpose4x4AVX( _mm256_loadu_si256( (const __m256i*)&s1->deltaPosition.z ),
					   _mm256_loadu_si256( (const __m256i*)&s2->deltaPosition.z ),
					   _mm256_loadu_si256( (const __m256i*)&s3->deltaPosition.z ),
					   _mm256_loadu_si256( (const __m256i*)&s4->deltaPosition.z ), &simdBody.dp.Z.v, &simdBody.dq.V.X.v,
					   &simdBody.dq.V.Y.v, &simdBody.dq.V.Z.v );

	simdBody.dq.S = b3MakeW( s1->deltaRotation.s, s2->deltaRotation.s, s3->deltaRotation.s, s4->deltaRotation.s );

	B3_AUDIT_V3W( b3_auditBodyV, simdBody.v );
	B3_AUDIT_V3W( b3_auditBodyW, simdBody.w );
	B3_AUDIT_V3W( b3_auditDeltaPos, simdBody.dp );

	return simdBody;
}

// SoA -> AoS scatter of the velocities: one 32-byte store (v.xyz, w.x) and
// one 16-byte store (w.y, w.z) per dynamic body, exactly the six fields the
// scalar version writes.
static void b3ScatterBodies( b3BodyState* states, int* indices, const b3BodyStateW* simdBody )
{
	__m256i row[4];
	b3Transpose4x4AVX( simdBody->v.X.v, simdBody->v.Y.v, simdBody->v.Z.v, simdBody->w.X.v, row + 0, row + 1, row + 2,
					   row + 3 );

	__m256i lo = _mm256_unpacklo_epi64( simdBody->w.Y.v, simdBody->w.Z.v ); // [y0 z0 y2 z2]
	__m256i hi = _mm256_unpackhi_epi64( simdBody->w.Y.v, simdBody->w.Z.v ); // [y1 z1 y3 z3]
	__m128i half[4];
	half[0] = _mm256_castsi256_si128( lo );
	half[1] = _mm256_castsi256_si128( hi );
	half[2] = _mm256_extracti128_si256( lo, 1 );
	half[3] = _mm256_extracti128_si256( hi, 1 );

	for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
	{
		int index = indices[lane] - 1;
		if ( index != -1 && ( states[index].flags & b3_dynamicFlag ) != 0 )
		{
			b3BodyState* state = states + index;
			_mm256_storeu_si256( (__m256i*)&state->linearVelocity.x, row[lane] );
			_mm_storeu_si128( (__m128i*)&state->angularVelocity.y, half[lane] );
		}
	}
}

#else

static b3BodyStateW b3GatherBodies( const b3BodyState* states, int* indices )
{
	b3BodyState identity = b3_identityBodyState;

	b3BodyState s1 = indices[0] == 0 ? identity : states[indices[0] - 1];
	b3BodyState s2 = indices[1] == 0 ? identity : states[indices[1] - 1];
	b3BodyState s3 = indices[2] == 0 ? identity : states[indices[2] - 1];
	b3BodyState s4 = indices[3] == 0 ? identity : states[indices[3] - 1];

	b3BodyStateW simdBody;
	simdBody.v.X = b3MakeW( s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x );
	simdBody.v.Y = b3MakeW( s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y );
	simdBody.v.Z = b3MakeW( s1.linearVelocity.z, s2.linearVelocity.z, s3.linearVelocity.z, s4.linearVelocity.z );
	simdBody.w.X = b3MakeW( s1.angularVelocity.x, s2.angularVelocity.x, s3.angularVelocity.x, s4.angularVelocity.x );
	simdBody.w.Y = b3MakeW( s1.angularVelocity.y, s2.angularVelocity.y, s3.angularVelocity.y, s4.angularVelocity.y );
	simdBody.w.Z = b3MakeW( s1.angularVelocity.z, s2.angularVelocity.z, s3.angularVelocity.z, s4.angularVelocity.z );
	simdBody.dp.X = b3MakeW( s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x );
	simdBody.dp.Y = b3MakeW( s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y );
	simdBody.dp.Z = b3MakeW( s1.deltaPosition.z, s2.deltaPosition.z, s3.deltaPosition.z, s4.deltaPosition.z );
	simdBody.dq.V.X = b3MakeW( s1.deltaRotation.v.x, s2.deltaRotation.v.x, s3.deltaRotation.v.x, s4.deltaRotation.v.x );
	simdBody.dq.V.Y = b3MakeW( s1.deltaRotation.v.y, s2.deltaRotation.v.y, s3.deltaRotation.v.y, s4.deltaRotation.v.y );
	simdBody.dq.V.Z = b3MakeW( s1.deltaRotation.v.z, s2.deltaRotation.v.z, s3.deltaRotation.v.z, s4.deltaRotation.v.z );
	simdBody.dq.S = b3MakeW( s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s );

	B3_AUDIT_V3W( b3_auditBodyV, simdBody.v );
	B3_AUDIT_V3W( b3_auditBodyW, simdBody.w );
	B3_AUDIT_V3W( b3_auditDeltaPos, simdBody.dp );

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

#endif // B3_SIMD_AVX512

#if defined( B3_SIMD_AVX512 )

// ---- wide prepare helpers -------------------------------------------------

// Wide b3StoreNarrow: same assert, same clamp to +/-INT32_MAX, same truncation,
// four lanes per store.
static inline void b3StoreNarrowW( b3FloatWN* target, b3FloatW value )
{
	B3_ASSERT( -INT32_MAX <= value.x && value.x <= INT32_MAX );
	B3_ASSERT( -INT32_MAX <= value.y && value.y <= INT32_MAX );
	B3_ASSERT( -INT32_MAX <= value.z && value.z <= INT32_MAX );
	B3_ASSERT( -INT32_MAX <= value.w && value.w <= INT32_MAX );
	__m256i clamped = _mm256_max_epi64( value.v, _mm256_set1_epi64x( -(int64_t)INT32_MAX ) );
	clamped = _mm256_min_epi64( clamped, _mm256_set1_epi64x( INT32_MAX ) );
	_mm_storeu_si128( (__m128i*)target, _mm256_cvtepi64_epi32( clamped ) );
}

static inline void b3StoreNarrowVW( b3Vec3WN* target, b3Vec3W value )
{
	b3StoreNarrowW( &target->X, value.X );
	b3StoreNarrowW( &target->Y, value.Y );
	b3StoreNarrowW( &target->Z, value.Z );
}

// value where the mask is set, zero elsewhere
static inline b3FloatW b3MaskKeepW( __mmask8 k, b3FloatW value )
{
	return (b3FloatW){ .v = _mm256_maskz_mov_epi64( k, value.v ) };
}

// b3FixClamp per lane: max then min gives identical results for lower <= upper
static inline b3FloatW b3ClampW( b3FloatW value, b3FloatW lower, b3FloatW upper )
{
	return (b3FloatW){ .v = _mm256_min_epi64( _mm256_max_epi64( value.v, lower.v ), upper.v ) };
}

static inline b3Vec3W b3ZeroVW( void )
{
	return (b3Vec3W){ b3ZeroW(), b3ZeroW(), b3ZeroW() };
}

// Prepare convex contact constraints, wide-math variant of the scalar version
// below. The per-lane pass gathers inputs and keeps the branchy or iterative
// pieces scalar (b3Perp, the 2x2/3x3 inversions, lever-arm square roots);
// everything else runs four lanes at a time. Every operation reproduces the
// exact rounding pattern of its scalar counterpart (unfused crosses and
// matrix products, fused raw-128 dots, divides through b3FixDiv), and
// inactive (lane, point) slots are fed zeros or masked so the stored bytes
// match the scalar zero-fill and the solver-setup memset bit for bit.
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

	b3FloatW warmStartScaleW = b3SplatW( world->enableWarmStarting ? B3_FIX( 1.0f ) : B3_FIX( 0.0f ) );

	// Used for friction center weighting.
	b3FloatW speculativeDistanceW = b3SplatW( B3_SPECULATIVE_DISTANCE );
	b3FloatW minFrictionWeightW = b3SplatW( B3_MIN_FRICTION_WEIGHT );
	b3FloatW oneW = b3SplatW( B3_FIX( 1.0f ) );
	b3FloatW twoW = b3SplatW( B3_FIX( 2.0f ) );

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

			// Wide staging, zeroed so inactive lanes and point slots compute
			// benign zeros that match the scalar zero-fill exactly.
			b3FloatW mAW = b3ZeroW(), mBW = b3ZeroW();
			b3Matrix3FullW iAW, iBW;
			memset( &iAW, 0, sizeof( iAW ) );
			memset( &iBW, 0, sizeof( iBW ) );
			b3Vec3W normalW = b3ZeroVW(), tangent1W = b3ZeroVW();
			b3Vec3W vAW = b3ZeroVW(), wAW = b3ZeroVW(), vBW = b3ZeroVW(), wBW = b3ZeroVW();
			b3Vec3W tangentVelocityW = b3ZeroVW(), frictionImpulseW = b3ZeroVW(), rollingImpulseW = b3ZeroVW();
			b3FloatW twistImpulseW = b3ZeroW();
			b3Vec3W anchorAW[B3_MAX_MANIFOLD_POINTS], anchorBW[B3_MAX_MANIFOLD_POINTS];
			b3FloatW separationW[B3_MAX_MANIFOLD_POINTS], normalImpulseW[B3_MAX_MANIFOLD_POINTS];
			for ( int pointIndex = 0; pointIndex < B3_MAX_MANIFOLD_POINTS; ++pointIndex )
			{
				anchorAW[pointIndex] = b3ZeroVW();
				anchorBW[pointIndex] = b3ZeroVW();
				separationW[pointIndex] = b3ZeroW();
				normalImpulseW[pointIndex] = b3ZeroW();
			}
			int64_t laneCounts[B3_SIMD_WIDTH] = { 0 };
			b3Matrix3 iAFull[B3_SIMD_WIDTH], iBFull[B3_SIMD_WIDTH];
			b3Fixed laneRollingResistance[B3_SIMD_WIDTH] = { 0 };
			int laneCount = 0;

			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex >= colorContactCount )
				{
					// Remainder lanes were zeroed in solver setup.
					break;
				}

				laneCount = lane + 1;

				int contactId = contactIds[contactIndex];
				b3Contact* contact = b3Array_Get( world->contacts, contactId );
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

				// Stage the wide math inputs
				( (b3Fixed*)&mAW )[lane] = mA;
				( (b3Fixed*)&mBW )[lane] = mB;

				( (b3Fixed*)&iAW.cxx )[lane] = iA.cx.x;
				( (b3Fixed*)&iAW.cxy )[lane] = iA.cx.y;
				( (b3Fixed*)&iAW.cxz )[lane] = iA.cx.z;
				( (b3Fixed*)&iAW.cyx )[lane] = iA.cy.x;
				( (b3Fixed*)&iAW.cyy )[lane] = iA.cy.y;
				( (b3Fixed*)&iAW.cyz )[lane] = iA.cy.z;
				( (b3Fixed*)&iAW.czx )[lane] = iA.cz.x;
				( (b3Fixed*)&iAW.czy )[lane] = iA.cz.y;
				( (b3Fixed*)&iAW.czz )[lane] = iA.cz.z;

				( (b3Fixed*)&iBW.cxx )[lane] = iB.cx.x;
				( (b3Fixed*)&iBW.cxy )[lane] = iB.cx.y;
				( (b3Fixed*)&iBW.cxz )[lane] = iB.cx.z;
				( (b3Fixed*)&iBW.cyx )[lane] = iB.cy.x;
				( (b3Fixed*)&iBW.cyy )[lane] = iB.cy.y;
				( (b3Fixed*)&iBW.cyz )[lane] = iB.cy.z;
				( (b3Fixed*)&iBW.czx )[lane] = iB.cz.x;
				( (b3Fixed*)&iBW.czy )[lane] = iB.cz.y;
				( (b3Fixed*)&iBW.czz )[lane] = iB.cz.z;

				iAFull[lane] = iA;
				iBFull[lane] = iB;

				( (b3Fixed*)&vAW.X )[lane] = vA.x;
				( (b3Fixed*)&vAW.Y )[lane] = vA.y;
				( (b3Fixed*)&vAW.Z )[lane] = vA.z;
				( (b3Fixed*)&wAW.X )[lane] = wA.x;
				( (b3Fixed*)&wAW.Y )[lane] = wA.y;
				( (b3Fixed*)&wAW.Z )[lane] = wA.z;
				( (b3Fixed*)&vBW.X )[lane] = vB.x;
				( (b3Fixed*)&vBW.Y )[lane] = vB.y;
				( (b3Fixed*)&vBW.Z )[lane] = vB.z;
				( (b3Fixed*)&wBW.X )[lane] = wB.x;
				( (b3Fixed*)&wBW.Y )[lane] = wB.y;
				( (b3Fixed*)&wBW.Z )[lane] = wB.z;

				b3Softness soft = ( indexA == B3_NULL_INDEX || indexB == B3_NULL_INDEX ) ? staticSoftness : contactSoftness;

				b3Vec3 normal = manifold->normal;
				b3StoreNarrow( &constraint->normal.X, lane, normal.x );
				b3StoreNarrow( &constraint->normal.Y, lane, normal.y );
				b3StoreNarrow( &constraint->normal.Z, lane, normal.z );
				( (b3Fixed*)&normalW.X )[lane] = normal.x;
				( (b3Fixed*)&normalW.Y )[lane] = normal.y;
				( (b3Fixed*)&normalW.Z )[lane] = normal.z;

				// Branchy axis selection stays scalar
				b3Vec3 tangent1 = b3Perp( normal );
				b3StoreNarrow( &constraint->tangent1.X, lane, tangent1.x );
				b3StoreNarrow( &constraint->tangent1.Y, lane, tangent1.y );
				b3StoreNarrow( &constraint->tangent1.Z, lane, tangent1.z );
				( (b3Fixed*)&tangent1W.X )[lane] = tangent1.x;
				( (b3Fixed*)&tangent1W.Y )[lane] = tangent1.y;
				( (b3Fixed*)&tangent1W.Z )[lane] = tangent1.z;

				( (b3Fixed*)&constraint->friction )[lane] = contact->friction;
				( (b3Fixed*)&constraint->restitution )[lane] = contact->restitution;
				( (b3Fixed*)&constraint->rollingResistance )[lane] = contact->rollingResistance;
				laneRollingResistance[lane] = contact->rollingResistance;

				( (b3Fixed*)&tangentVelocityW.X )[lane] = contact->tangentVelocity.x;
				( (b3Fixed*)&tangentVelocityW.Y )[lane] = contact->tangentVelocity.y;
				( (b3Fixed*)&tangentVelocityW.Z )[lane] = contact->tangentVelocity.z;

				( (b3Fixed*)&constraint->biasRate )[lane] = soft.biasRate;
				( (b3Fixed*)&constraint->massScale )[lane] = soft.massScale;
				( (b3Fixed*)&constraint->impulseScale )[lane] = soft.impulseScale;

				( (b3Fixed*)&frictionImpulseW.X )[lane] = manifold->frictionImpulse.x;
				( (b3Fixed*)&frictionImpulseW.Y )[lane] = manifold->frictionImpulse.y;
				( (b3Fixed*)&frictionImpulseW.Z )[lane] = manifold->frictionImpulse.z;
				( (b3Fixed*)&twistImpulseW )[lane] = manifold->twistImpulse;
				( (b3Fixed*)&rollingImpulseW.X )[lane] = manifold->rollingImpulse.x;
				( (b3Fixed*)&rollingImpulseW.Y )[lane] = manifold->rollingImpulse.y;
				( (b3Fixed*)&rollingImpulseW.Z )[lane] = manifold->rollingImpulse.z;

				int pointCount = manifold->pointCount;
				laneCounts[lane] = pointCount;
				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const b3ManifoldPoint* mp = manifold->points + pointIndex;
					( (b3Fixed*)&anchorAW[pointIndex].X )[lane] = mp->anchorA.x;
					( (b3Fixed*)&anchorAW[pointIndex].Y )[lane] = mp->anchorA.y;
					( (b3Fixed*)&anchorAW[pointIndex].Z )[lane] = mp->anchorA.z;
					( (b3Fixed*)&anchorBW[pointIndex].X )[lane] = mp->anchorB.x;
					( (b3Fixed*)&anchorBW[pointIndex].Y )[lane] = mp->anchorB.y;
					( (b3Fixed*)&anchorBW[pointIndex].Z )[lane] = mp->anchorB.z;
					( (b3Fixed*)&separationW[pointIndex] )[lane] = mp->separation;
					( (b3Fixed*)&normalImpulseW[pointIndex] )[lane] = mp->normalImpulse;
				}

				B3_AUDIT_FIX( b3_auditInvMass, mA );
				B3_AUDIT_FIX( b3_auditInvMass, mB );
				B3_AUDIT_M3( b3_auditInvInertia, iA );
				B3_AUDIT_M3( b3_auditInvInertia, iB );
			}

			// ---- wide math ----

			__m256i pointCountsV = _mm256_loadu_si256( (const __m256i*)laneCounts );

			b3Vec3W tangent2W = b3CrossUnfusedW( tangent1W, normalW );
			b3StoreNarrowVW( &constraint->tangent2, tangent2W );

			constraint->tangentVelocity1 = b3DotW( tangentVelocityW, tangent1W );
			constraint->tangentVelocity2 = b3DotW( tangentVelocityW, tangent2W );

			b3Vec3W centerAW = b3ZeroVW();
			b3Vec3W centerBW = b3ZeroVW();
			b3FloatW totalFrictionWeightW = b3ZeroW();

			// Only run the per-point math up to the widest manifold in the
			// block: sphere-heavy scenes are mostly one-point manifolds and
			// the remaining slots are all-lane inactive anyway.
			int maxPointCount = 0;
			for ( int lane = 0; lane < laneCount; ++lane )
			{
				maxPointCount = b3MaxInt( maxPointCount, (int)laneCounts[lane] );
			}
			constraint->maxPointCount = maxPointCount;

			for ( int pointIndex = 0; pointIndex < maxPointCount; ++pointIndex )
			{
				b3ContactConstraintPointWide* cp = constraint->points + pointIndex;
				__mmask8 laneValid = _mm256_cmpgt_epi64_mask( pointCountsV, _mm256_set1_epi64x( pointIndex ) );

				b3Vec3W rA = anchorAW[pointIndex];
				b3Vec3W rB = anchorBW[pointIndex];
				b3FloatW s = separationW[pointIndex];

				// C0 friction center decay, masked so only real points
				// contribute (the scalar loop simply does not visit the rest)
				b3FloatW weight = b3ClampW( b3SubW( twoW, b3DivW( s, speculativeDistanceW ) ), minFrictionWeightW, oneW );
				weight = b3MaskKeepW( laneValid, weight );
				centerAW = b3MulAddSVW( centerAW, weight, rA );
				centerBW = b3MulAddSVW( centerBW, weight, rB );
				totalFrictionWeightW = b3AddW( totalFrictionWeightW, weight );

				b3StoreNarrowVW( &cp->anchorAs, rA );
				b3StoreNarrowVW( &cp->anchorBs, rB );

				cp->baseSeparations = b3SubW( s, b3DotW( b3SubVW( rB, rA ), normalW ) );

				cp->normalImpulses = b3MulW( warmStartScaleW, normalImpulseW[pointIndex] );
				cp->totalNormalImpulses = b3ZeroW();

				b3Vec3W rnA = b3CrossUnfusedW( rA, normalW );
				b3Vec3W rnB = b3CrossUnfusedW( rB, normalW );
				b3Vec3W iRnA = b3MulMVFullW( &iAW, rnA );
				b3Vec3W iRnB = b3MulMVFullW( &iBW, rnB );
				b3FloatW kNormal = b3AddW( b3AddW( mAW, mBW ), b3AddW( b3DotW( rnA, iRnA ), b3DotW( rnB, iRnB ) ) );
				__mmask8 kPositive = _mm256_cmpgt_epi64_mask( kNormal.v, _mm256_setzero_si256() );
				cp->normalMasses = b3MaskKeepW( laneValid & kPositive, b3DivW( oneW, kNormal ) );

				B3_AUDIT_V3W( b3_auditAnchor, rA );
				B3_AUDIT_V3W( b3_auditAnchor, rB );
				B3_AUDIT_V3W( b3_auditRn, rnA );
				B3_AUDIT_V3W( b3_auditRn, rnB );
				B3_AUDIT_V3W( b3_auditIRn, iRnA );
				B3_AUDIT_V3W( b3_auditIRn, iRnB );
				B3_AUDIT_W( b3_auditNormalMass, cp->normalMasses );

				// Precomputed normal Jacobian rows for the solve and warm start
				b3StoreNarrowVW( &cp->rnAs, rnA );
				b3StoreNarrowVW( &cp->rnBs, rnB );
				b3StoreNarrowVW( &cp->iRnAs, iRnA );
				b3StoreNarrowVW( &cp->iRnBs, iRnB );

				// Save relative velocity for restitution
				b3Vec3W vrA = b3AddVW( vAW, b3CrossUnfusedW( wAW, rA ) );
				b3Vec3W vrB = b3AddVW( vBW, b3CrossUnfusedW( wBW, rB ) );
				cp->relativeVelocities = b3MaskKeepW( laneValid, b3DotW( normalW, b3SubVW( vrB, vrA ) ) );
			}

			// Slots past the widest manifold are inactive in every lane; store
			// the zeros the scalar zero-fill would (the per-point math above
			// would compute exactly these from the zeroed staging).
			for ( int pointIndex = maxPointCount; pointIndex < B3_MAX_MANIFOLD_POINTS; ++pointIndex )
			{
				b3ContactConstraintPointWide* cp = constraint->points + pointIndex;
				b3StoreNarrowVW( &cp->anchorAs, b3ZeroVW() );
				b3StoreNarrowVW( &cp->anchorBs, b3ZeroVW() );
				b3StoreNarrowVW( &cp->rnAs, b3ZeroVW() );
				b3StoreNarrowVW( &cp->rnBs, b3ZeroVW() );
				b3StoreNarrowVW( &cp->iRnAs, b3ZeroVW() );
				b3StoreNarrowVW( &cp->iRnBs, b3ZeroVW() );
				cp->baseSeparations = b3ZeroW();
				cp->normalImpulses = b3ZeroW();
				cp->totalNormalImpulses = b3ZeroW();
				cp->normalMasses = b3ZeroW();
				cp->relativeVelocities = b3ZeroW();
			}

			b3FloatW invWeightW = b3DivW( oneW, totalFrictionWeightW );
			centerAW = b3MulSVW( invWeightW, centerAW );
			centerBW = b3MulSVW( invWeightW, centerBW );

			// Lever arms need a square root per (lane, point): scalar
			for ( int lane = 0; lane < laneCount; ++lane )
			{
				b3Vec3 centerA = { ( (b3Fixed*)&centerAW.X )[lane], ( (b3Fixed*)&centerAW.Y )[lane],
								   ( (b3Fixed*)&centerAW.Z )[lane] };
				int pointCount = (int)laneCounts[lane];
				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					b3Vec3 anchorA = { ( (b3Fixed*)&anchorAW[pointIndex].X )[lane], ( (b3Fixed*)&anchorAW[pointIndex].Y )[lane],
									   ( (b3Fixed*)&anchorAW[pointIndex].Z )[lane] };
					( (b3Fixed*)&constraint->points[pointIndex].leverArms )[lane] = b3Distance( anchorA, centerA );
				}
				for ( int pointIndex = pointCount; pointIndex < B3_MAX_MANIFOLD_POINTS; ++pointIndex )
				{
					( (b3Fixed*)&constraint->points[pointIndex].leverArms )[lane] = B3_FIX( 0.0f );
				}
			}

			b3Vec3W rtA1W = b3CrossUnfusedW( centerAW, tangent1W );
			b3Vec3W rtA2W = b3CrossUnfusedW( centerAW, tangent2W );
			b3Vec3W rtB1W = b3CrossUnfusedW( centerBW, tangent1W );
			b3Vec3W rtB2W = b3CrossUnfusedW( centerBW, tangent2W );

			// Precomputed friction Jacobian rows
			b3StoreNarrowVW( &constraint->rtA1s, rtA1W );
			b3StoreNarrowVW( &constraint->rtA2s, rtA2W );
			b3StoreNarrowVW( &constraint->rtB1s, rtB1W );
			b3StoreNarrowVW( &constraint->rtB2s, rtB2W );

			// Precomputed linear normal Jacobian
			b3Vec3W mNormalAW = b3MulSVW( mAW, normalW );
			b3Vec3W mNormalBW = b3MulSVW( mBW, normalW );
			b3StoreNarrowVW( &constraint->mNormalA, mNormalAW );
			b3StoreNarrowVW( &constraint->mNormalB, mNormalBW );

			B3_AUDIT_V3W( b3_auditMNormal, mNormalAW );
			B3_AUDIT_V3W( b3_auditMNormal, mNormalBW );
			B3_AUDIT_V3W( b3_auditRt, rtA1W );
			B3_AUDIT_V3W( b3_auditRt, rtA2W );
			B3_AUDIT_V3W( b3_auditRt, rtB1W );
			B3_AUDIT_V3W( b3_auditRt, rtB2W );

			{
				b3FloatW mSumW = b3AddW( mAW, mBW );
				b3FloatW kxx = b3AddW(
					mSumW, b3AddW( b3DotW( rtA1W, b3MulMVFullW( &iAW, rtA1W ) ), b3DotW( rtB1W, b3MulMVFullW( &iBW, rtB1W ) ) ) );
				b3FloatW kyy = b3AddW(
					mSumW, b3AddW( b3DotW( rtA2W, b3MulMVFullW( &iAW, rtA2W ) ), b3DotW( rtB2W, b3MulMVFullW( &iBW, rtB2W ) ) ) );
				b3FloatW kxy =
					b3AddW( b3DotW( rtA1W, b3MulMVFullW( &iAW, rtA2W ) ), b3DotW( rtB1W, b3MulMVFullW( &iBW, rtB2W ) ) );

				// 2x2 inversion divides through 128-bit determinants: scalar
				for ( int lane = 0; lane < laneCount; ++lane )
				{
					b3Matrix2 k;
					k.cx.x = ( (b3Fixed*)&kxx )[lane];
					k.cy.y = ( (b3Fixed*)&kyy )[lane];
					k.cx.y = k.cy.x = ( (b3Fixed*)&kxy )[lane];
					b3Matrix2 tangentMass = b3Invert2( k );

					( (b3Fixed*)&constraint->tangentMass.cxx )[lane] = tangentMass.cx.x;
					( (b3Fixed*)&constraint->tangentMass.cxy )[lane] = tangentMass.cx.y;
					( (b3Fixed*)&constraint->tangentMass.cyy )[lane] = tangentMass.cy.y;
				}

				constraint->frictionImpulse.x = b3MulW( warmStartScaleW, b3DotW( frictionImpulseW, tangent1W ) );
				constraint->frictionImpulse.y = b3MulW( warmStartScaleW, b3DotW( frictionImpulseW, tangent2W ) );
			}

			{
				// Precomputed angular normal Jacobian, also used for the twist mass
				b3Vec3W iNormalAW = b3MulMVFullW( &iAW, normalW );
				b3Vec3W iNormalBW = b3MulMVFullW( &iBW, normalW );
				b3StoreNarrowVW( &constraint->iNormalA, iNormalAW );
				b3StoreNarrowVW( &constraint->iNormalB, iNormalBW );

				b3FloatW kTwist = b3AddW( b3DotW( normalW, iNormalAW ), b3DotW( normalW, iNormalBW ) );
				__mmask8 kPositive = _mm256_cmpgt_epi64_mask( kTwist.v, _mm256_setzero_si256() );
				constraint->twistMass = b3MaskKeepW( kPositive, b3DivW( oneW, kTwist ) );
				constraint->twistImpulse = b3MulW( warmStartScaleW, twistImpulseW );

				B3_AUDIT_V3W( b3_auditINormal, iNormalAW );
				B3_AUDIT_V3W( b3_auditINormal, iNormalBW );
				B3_AUDIT_W( b3_auditTwistMass, constraint->twistMass );
			}

			{
				// The 128-bit matrix inversion is only needed when rolling resistance is active
				for ( int lane = 0; lane < laneCount; ++lane )
				{
					b3Matrix3 rollingMass = laneRollingResistance[lane] > B3_FIX( 0.0f )
												? b3InvertMatrix( b3AddMM( iAFull[lane], iBFull[lane] ) )
												: b3Mat3_zero;

					( (b3Fixed*)&constraint->rollingMass.cxx )[lane] = rollingMass.cx.x;
					( (b3Fixed*)&constraint->rollingMass.cxy )[lane] = rollingMass.cx.y;
					( (b3Fixed*)&constraint->rollingMass.cxz )[lane] = rollingMass.cx.z;
					( (b3Fixed*)&constraint->rollingMass.cyy )[lane] = rollingMass.cy.y;
					( (b3Fixed*)&constraint->rollingMass.cyz )[lane] = rollingMass.cy.z;
					( (b3Fixed*)&constraint->rollingMass.czz )[lane] = rollingMass.cz.z;
				}

				constraint->rollingImpulse.X = b3MulW( warmStartScaleW, rollingImpulseW.X );
				constraint->rollingImpulse.Y = b3MulW( warmStartScaleW, rollingImpulseW.Y );
				constraint->rollingImpulse.Z = b3MulW( warmStartScaleW, rollingImpulseW.Z );
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	b3TracyCZoneEnd( prepare_contact );
}

#else

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

	// Used for friction center weighting.
	b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;

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
			int maxPointCount = 0;

			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex >= colorContactCount )
				{
					// Remainder lanes were zeroed in solver setup.
					break;
				}

				int contactId = contactIds[contactIndex];
				b3Contact* contact = b3Array_Get( world->contacts, contactId );
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
				b3StoreNarrow( &constraint->normal.X, lane, normal.x );
				b3StoreNarrow( &constraint->normal.Y, lane, normal.y );
				b3StoreNarrow( &constraint->normal.Z, lane, normal.z );

				b3Vec3 tangent1 = b3Perp( normal );
				b3StoreNarrow( &constraint->tangent1.X, lane, tangent1.x );
				b3StoreNarrow( &constraint->tangent1.Y, lane, tangent1.y );
				b3StoreNarrow( &constraint->tangent1.Z, lane, tangent1.z );

				b3Vec3 tangent2 = b3Cross( tangent1, normal );
				b3StoreNarrow( &constraint->tangent2.X, lane, tangent2.x );
				b3StoreNarrow( &constraint->tangent2.Y, lane, tangent2.y );
				b3StoreNarrow( &constraint->tangent2.Z, lane, tangent2.z );

				( (b3Fixed*)&constraint->friction )[lane] = contact->friction;
				( (b3Fixed*)&constraint->restitution )[lane] = contact->restitution;
				( (b3Fixed*)&constraint->rollingResistance )[lane] = contact->rollingResistance;

				( (b3Fixed*)&constraint->tangentVelocity1 )[lane] = b3Dot( contact->tangentVelocity, tangent1 );
				( (b3Fixed*)&constraint->tangentVelocity2 )[lane] = b3Dot( contact->tangentVelocity, tangent2 );

				( (b3Fixed*)&constraint->biasRate )[lane] = soft.biasRate;
				( (b3Fixed*)&constraint->massScale )[lane] = soft.massScale;
				( (b3Fixed*)&constraint->impulseScale )[lane] = soft.impulseScale;

				int pointCount = manifold->pointCount;
				maxPointCount = b3MaxInt( maxPointCount, pointCount );
				b3Vec3 centerA = b3Vec3_zero;
				b3Vec3 centerB = b3Vec3_zero;
				b3Fixed totalFrictionWeight = B3_FIX( 0.0f );

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const b3ManifoldPoint* mp = manifold->points + pointIndex;
					b3ContactConstraintPointWide* cp = constraint->points + pointIndex;

					b3Vec3 rA = mp->anchorA;
					b3Vec3 rB = mp->anchorB;
					b3Fixed s = mp->separation;

					// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
					// See details in b3PrepareContacts_Mesh. This code should stay in sync.
					b3Fixed weight =
						b3FixClamp( B3_FIX( 2.0f ) - b3FixDiv( s, speculativeDistance ), B3_MIN_FRICTION_WEIGHT, B3_FIX( 1.0f ) );
					centerA = b3MulAdd( centerA, weight, rA );
					centerB = b3MulAdd( centerB, weight, rB );
					totalFrictionWeight += weight;

					b3StoreNarrow( &cp->anchorAs.X, lane, rA.x );
					b3StoreNarrow( &cp->anchorAs.Y, lane, rA.y );
					b3StoreNarrow( &cp->anchorAs.Z, lane, rA.z );

					b3StoreNarrow( &cp->anchorBs.X, lane, rB.x );
					b3StoreNarrow( &cp->anchorBs.Y, lane, rB.y );
					b3StoreNarrow( &cp->anchorBs.Z, lane, rB.z );

					b3Fixed baseSeparation = s - b3Dot( b3Sub( rB, rA ), normal );
					( (b3Fixed*)&cp->baseSeparations )[lane] = baseSeparation;

					( (b3Fixed*)&cp->normalImpulses )[lane] = b3FixMul( warmStartScale , mp->normalImpulse );
					( (b3Fixed*)&cp->totalNormalImpulses )[lane] = B3_FIX( 0.0f );

					b3Vec3 rnA = b3Cross( rA, normal );
					b3Vec3 rnB = b3Cross( rB, normal );
					b3Vec3 iRnA = b3MulMV( iA, rnA );
					b3Vec3 iRnB = b3MulMV( iB, rnB );
					b3Fixed kNormal = mA + mB + b3Dot( rnA, iRnA ) + b3Dot( rnB, iRnB );
					( (b3Fixed*)&cp->normalMasses )[lane] = kNormal > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ) , kNormal ) : B3_FIX( 0.0f );

					B3_AUDIT_V3( b3_auditAnchor, rA );
					B3_AUDIT_V3( b3_auditAnchor, rB );
					B3_AUDIT_V3( b3_auditRn, rnA );
					B3_AUDIT_V3( b3_auditRn, rnB );
					B3_AUDIT_V3( b3_auditIRn, iRnA );
					B3_AUDIT_V3( b3_auditIRn, iRnB );
					B3_AUDIT_FIX( b3_auditInvMass, mA );
					B3_AUDIT_FIX( b3_auditInvMass, mB );
					B3_AUDIT_M3( b3_auditInvInertia, iA );
					B3_AUDIT_M3( b3_auditInvInertia, iB );
					B3_AUDIT_FIX( b3_auditNormalMass, ( (b3Fixed*)&cp->normalMasses )[lane] );

					// Precomputed normal Jacobian rows for the solve and warm start
					b3StoreNarrow( &cp->rnAs.X, lane, rnA.x );
					b3StoreNarrow( &cp->rnAs.Y, lane, rnA.y );
					b3StoreNarrow( &cp->rnAs.Z, lane, rnA.z );
					b3StoreNarrow( &cp->rnBs.X, lane, rnB.x );
					b3StoreNarrow( &cp->rnBs.Y, lane, rnB.y );
					b3StoreNarrow( &cp->rnBs.Z, lane, rnB.z );
					b3StoreNarrow( &cp->iRnAs.X, lane, iRnA.x );
					b3StoreNarrow( &cp->iRnAs.Y, lane, iRnA.y );
					b3StoreNarrow( &cp->iRnAs.Z, lane, iRnA.z );
					b3StoreNarrow( &cp->iRnBs.X, lane, iRnB.x );
					b3StoreNarrow( &cp->iRnBs.Y, lane, iRnB.y );
					b3StoreNarrow( &cp->iRnBs.Z, lane, iRnB.z );

					// Save relative velocity for restitution
					b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
					b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
					( (b3Fixed*)&cp->relativeVelocities )[lane] = b3Dot( normal, b3Sub( vrB, vrA ) );
				}

				b3Fixed invWeight = b3FixDiv( B3_FIX( 1.0f ) , totalFrictionWeight );
				centerA = b3MulSV( invWeight, centerA );
				centerB = b3MulSV( invWeight, centerB );

				for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
				{
					const b3ManifoldPoint* mp = manifold->points + pointIndex;
					b3ContactConstraintPointWide* cp = constraint->points + pointIndex;
					( (b3Fixed*)&cp->leverArms )[lane] = b3Distance( mp->anchorA, centerA );
				}

				b3Vec3 rtA1 = b3Cross( centerA, tangent1 );
				b3Vec3 rtA2 = b3Cross( centerA, tangent2 );

				b3Vec3 rtB1 = b3Cross( centerB, tangent1 );
				b3Vec3 rtB2 = b3Cross( centerB, tangent2 );

				// Precomputed friction Jacobian rows
				b3StoreNarrow( &constraint->rtA1s.X, lane, rtA1.x );
				b3StoreNarrow( &constraint->rtA1s.Y, lane, rtA1.y );
				b3StoreNarrow( &constraint->rtA1s.Z, lane, rtA1.z );
				b3StoreNarrow( &constraint->rtA2s.X, lane, rtA2.x );
				b3StoreNarrow( &constraint->rtA2s.Y, lane, rtA2.y );
				b3StoreNarrow( &constraint->rtA2s.Z, lane, rtA2.z );
				b3StoreNarrow( &constraint->rtB1s.X, lane, rtB1.x );
				b3StoreNarrow( &constraint->rtB1s.Y, lane, rtB1.y );
				b3StoreNarrow( &constraint->rtB1s.Z, lane, rtB1.z );
				b3StoreNarrow( &constraint->rtB2s.X, lane, rtB2.x );
				b3StoreNarrow( &constraint->rtB2s.Y, lane, rtB2.y );
				b3StoreNarrow( &constraint->rtB2s.Z, lane, rtB2.z );

				// Precomputed linear normal Jacobian
				b3Vec3 mNormalA = b3MulSV( mA, normal );
				b3Vec3 mNormalB = b3MulSV( mB, normal );
				b3StoreNarrow( &constraint->mNormalA.X, lane, mNormalA.x );
				b3StoreNarrow( &constraint->mNormalA.Y, lane, mNormalA.y );
				b3StoreNarrow( &constraint->mNormalA.Z, lane, mNormalA.z );
				b3StoreNarrow( &constraint->mNormalB.X, lane, mNormalB.x );
				b3StoreNarrow( &constraint->mNormalB.Y, lane, mNormalB.y );
				b3StoreNarrow( &constraint->mNormalB.Z, lane, mNormalB.z );

				B3_AUDIT_V3( b3_auditMNormal, mNormalA );
				B3_AUDIT_V3( b3_auditMNormal, mNormalB );
				B3_AUDIT_V3( b3_auditRt, rtA1 );
				B3_AUDIT_V3( b3_auditRt, rtA2 );
				B3_AUDIT_V3( b3_auditRt, rtB1 );
				B3_AUDIT_V3( b3_auditRt, rtB2 );

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
					b3StoreNarrow( &constraint->iNormalA.X, lane, iNormalA.x );
					b3StoreNarrow( &constraint->iNormalA.Y, lane, iNormalA.y );
					b3StoreNarrow( &constraint->iNormalA.Z, lane, iNormalA.z );
					b3StoreNarrow( &constraint->iNormalB.X, lane, iNormalB.x );
					b3StoreNarrow( &constraint->iNormalB.Y, lane, iNormalB.y );
					b3StoreNarrow( &constraint->iNormalB.Z, lane, iNormalB.z );

					b3Fixed k = b3Dot( normal, iNormalA ) + b3Dot( normal, iNormalB );
					( (b3Fixed*)&constraint->twistMass )[lane] = k > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ) , k ) : B3_FIX( 0.0f );
					( (b3Fixed*)&constraint->twistImpulse )[lane] = b3FixMul( warmStartScale , manifold->twistImpulse );

					B3_AUDIT_V3( b3_auditINormal, iNormalA );
					B3_AUDIT_V3( b3_auditINormal, iNormalB );
					B3_AUDIT_FIX( b3_auditTwistMass, ( (b3Fixed*)&constraint->twistMass )[lane] );
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
					b3StoreNarrow( &cp->anchorAs.X, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->anchorAs.Y, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->anchorAs.Z, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->anchorBs.X, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->anchorBs.Y, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->anchorBs.Z, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->rnAs.X, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->rnAs.Y, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->rnAs.Z, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->rnBs.X, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->rnBs.Y, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->rnBs.Z, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->iRnAs.X, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->iRnAs.Y, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->iRnAs.Z, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->iRnBs.X, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->iRnBs.Y, lane, B3_FIX( 0.0f ) );
					b3StoreNarrow( &cp->iRnBs.Z, lane, B3_FIX( 0.0f ) );
					( (b3Fixed*)&cp->baseSeparations )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->normalImpulses )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->totalNormalImpulses )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->normalMasses )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->relativeVelocities )[lane] = B3_FIX( 0.0f );
					( (b3Fixed*)&cp->leverArms )[lane] = B3_FIX( 0.0f );
				}
			}

			constraint->maxPointCount = maxPointCount;
		}

		// Advance to next color
		colorIndex += 1;
	}

	b3TracyCZoneEnd( prepare_contact );
}

#endif // B3_SIMD_AVX512

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

		// Normal impulses, applied through the precomputed Jacobian rows.
		// Point slots past maxPointCount hold exact zeros in every lane and
		// apply nothing; skipping them is bit-identical.
		for ( int j = 0; j < c->maxPointCount; ++j )
		{
			b3ContactConstraintPointWide* cp = c->points + j;

			bA.v = b3MulSubSVW( bA.v, cp->normalImpulses, b3WidenVW( c->mNormalA ) );
			bA.w = b3MulSubSVW( bA.w, cp->normalImpulses, b3WidenVW( cp->iRnAs ) );
			bB.v = b3MulAddSVW( bB.v, cp->normalImpulses, b3WidenVW( c->mNormalB ) );
			bB.w = b3MulAddSVW( bB.w, cp->normalImpulses, b3WidenVW( cp->iRnBs ) );
		}

		// Central friction
		{
			b3Vec3W impulse = b3MulSVW( c->frictionImpulse.x, b3WidenVW( c->tangent1 ) );
			impulse = b3MulAddSVW( impulse, c->frictionImpulse.y, b3WidenVW( c->tangent2 ) );

			// cross(origin, P) expanded over the precomputed tangent rows
			b3Vec3W LA = b3MulSVW( c->frictionImpulse.x, b3WidenVW( c->rtA1s ) );
			LA = b3MulAddSVW( LA, c->frictionImpulse.y, b3WidenVW( c->rtA2s ) );
			b3Vec3W LB = b3MulSVW( c->frictionImpulse.x, b3WidenVW( c->rtB1s ) );
			LB = b3MulAddSVW( LB, c->frictionImpulse.y, b3WidenVW( c->rtB2s ) );

			bA.w = b3MulSubMVW( bA.w, c->invIA, LA );
			bA.v = b3MulSubSVW( bA.v, c->invMassA, impulse );
			bB.w = b3MulAddMVW( bB.w, c->invIB, LB );
			bB.v = b3MulAddSVW( bB.v, c->invMassB, impulse );
		}

		// Central twist friction
		{
			bA.w = b3MulSubSVW( bA.w, c->twistImpulse, b3WidenVW( c->iNormalA ) );
			bB.w = b3MulAddSVW( bB.w, c->twistImpulse, b3WidenVW( c->iNormalB ) );
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
		b3Vec3W normal = b3WidenVW( c->normal );

		// The delta rotations are constant across the four manifold points, so build
		// them as matrices once and rotate each anchor with three fused reductions
		b3Matrix3W rotA = b3MakeMatrixFromQuatW( bA.dq );
		b3Matrix3W rotB = b3MakeMatrixFromQuatW( bB.dq );

		b3FloatW totalNormalImpulse = b3ZeroW();
		b3FloatW totalTwistLimit = b3ZeroW();

		// Point slots past maxPointCount hold exact zeros in every lane
		// (normalMasses 0, impulses 0), produce zero deltas, and store back
		// the zeros already there; skipping them is bit-identical.
		for ( int pointIndex = 0; pointIndex < c->maxPointCount; ++pointIndex )
		{
			b3ContactConstraintPointWide* cp = c->points + pointIndex;

			// Fixed anchor points for applying impulses
			b3Vec3W rA = b3WidenVW( cp->anchorAs );
			b3Vec3W rB = b3WidenVW( cp->anchorBs );

			// Moving anchors for current separation
			b3Vec3W rsA = b3MulMV3W( rotA, rA );
			b3Vec3W rsB = b3MulMV3W( rotB, rB );

			// compute current separation
			// this is subject to round-off error if the anchor is far from the body center of mass
			b3Vec3W ds = b3AddVW( dp, b3SubVW( rsB, rsA ) );
			b3FloatW s = b3AddW( b3DotW( normal, ds ), cp->baseSeparations );

			// Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
			b3FloatW mask = b3GreaterThanW( s, b3ZeroW() );
			b3FloatW specBias = b3MulW( s, inv_h );
			b3FloatW softBias = b3MaxW( b3MulW( biasRate, s ), contactSpeed );
			b3FloatW bias = b3BlendW( softBias, specBias, mask );

			B3_AUDIT_W( b3_auditSeparation, s );
			B3_AUDIT_W( b3_auditBias, bias );

			b3FloatW pointMassScale = b3BlendW( massScale, oneW, mask );
			b3FloatW pointImpulseScale = b3BlendW( impulseScale, b3ZeroW(), mask );

			// Relative velocity at contact, projected on the normal through the
			// precomputed cross(r, normal) rows with a single rounding
			b3FloatW vn = b3RelVelocityW( b3SubVW( bB.v, bA.v ), normal, bB.w, b3WidenVW( cp->rnBs ), bA.w,
										  b3WidenVW( cp->rnAs ) );

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

			B3_AUDIT_W( b3_auditVn, vn );
			B3_AUDIT_W( b3_auditNormalImpulse, newImpulse );
			B3_AUDIT_W( b3_auditTotalNormalImpulse, cp->totalNormalImpulses );

			// Apply contact impulse through the precomputed Jacobian rows
			bA.v = b3MulSubSVW( bA.v, deltaImpulse, b3WidenVW( c->mNormalA ) );
			bA.w = b3MulSubSVW( bA.w, deltaImpulse, b3WidenVW( cp->iRnAs ) );
			bB.v = b3MulAddSVW( bB.v, deltaImpulse, b3WidenVW( c->mNormalB ) );
			bB.w = b3MulAddSVW( bB.w, deltaImpulse, b3WidenVW( cp->iRnBs ) );
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
				B3_AUDIT_W( b3_auditRollingCap, maxImpulse );

				// if ( magSqr > maxLambda * maxLambda + B3_FIXED_EPSILON )
				//{
				//	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
				// }

				// The squared compare wraps int64 past B3_IMPULSE_CAP_GATE; the cold
				// path is exact (see the cap-guard block above b3SolveContacts_Mesh)
				b3FloatW mask, normalize;
				if ( b3CapOperandsSafeW4( c->rollingImpulse.X, c->rollingImpulse.Y, c->rollingImpulse.Z, maxImpulse ) )
				{
					b3FloatW lengthSquared = b3DotW( c->rollingImpulse, c->rollingImpulse );
					mask = b3GreaterThanW( lengthSquared, b3MulAddW( epsilonW, maxImpulse, maxImpulse ) );

					// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
					normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				}
				else
				{
					b3ClampImpulseColdW3( c->rollingImpulse, maxImpulse, true, &mask, &normalize );
				}
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
				b3FloatW twistSpeed = b3DotW( normal, b3SubVW( bB.w, bA.w ) );
				b3FloatW maxLambda = b3MulW( c->friction, totalTwistLimit );
				B3_AUDIT_W( b3_auditTwistCap, maxLambda );
				b3FloatW deltaImpulse = b3NegW( b3MulW( c->twistMass, twistSpeed ) );
				b3FloatW oldImpulse = c->twistImpulse;
				c->twistImpulse = b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxLambda );
				deltaImpulse = b3SubW( c->twistImpulse, oldImpulse );

				bA.w = b3MulSubSVW( bA.w, deltaImpulse, b3WidenVW( c->iNormalA ) );
				bB.w = b3MulAddSVW( bB.w, deltaImpulse, b3WidenVW( c->iNormalB ) );
			}

			// Central friction
			{
				b3Vec3W tangent1 = b3WidenVW( c->tangent1 );
				b3Vec3W tangent2 = b3WidenVW( c->tangent2 );

				// Relative tangent velocity at contact through the precomputed
				// cross(origin, tangent) rows, one rounding per axis
				b3Vec3W dv = b3SubVW( bB.v, bA.v );
				b3Vec2W vt = {
					b3SubW( b3RelVelocityW( dv, tangent1, bB.w, b3WidenVW( c->rtB1s ), bA.w, b3WidenVW( c->rtA1s ) ),
							c->tangentVelocity1 ),
					b3SubW( b3RelVelocityW( dv, tangent2, bB.w, b3WidenVW( c->rtB2s ), bA.w, b3WidenVW( c->rtA2s ) ),
							c->tangentVelocity2 ),
				};

				// Incremental tangent impulse
				b3Vec2W deltaImpulse = b3MulMV2W( c->tangentMass, vt );
				deltaImpulse = (b3Vec2W){ b3NegW( deltaImpulse.x ), b3NegW( deltaImpulse.y ) };
				b3Vec2W newImpulse = b3AddV2W( c->frictionImpulse, deltaImpulse );

				b3FloatW friction = c->friction;
				b3FloatW maxImpulse = b3MulW( friction, totalNormalImpulse );
				B3_AUDIT_W( b3_auditIterNormalSum, totalNormalImpulse );
				B3_AUDIT_W( b3_auditFrictionCap, maxImpulse );

				// Clamp the accumulated impulse. The squared compare wraps int64 past
				// B3_IMPULSE_CAP_GATE; the cold path is exact (see the cap-guard block
				// above b3SolveContacts_Mesh).
				b3FloatW mask, normalize;
				if ( b3CapOperandsSafeW3( newImpulse.x, newImpulse.y, maxImpulse ) )
				{
					b3FloatW lengthSquared = b3AddW( b3MulW( newImpulse.x, newImpulse.x ), b3MulW( newImpulse.y, newImpulse.y ) );

					// Max impulse can be zero
					mask = b3GreaterThanW( lengthSquared, b3MulW( maxImpulse, maxImpulse ) );

					// No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
					// zero.
					normalize = b3DivW( maxImpulse, b3AddW( b3SqrtW( lengthSquared ), epsilonW ) );
				}
				else
				{
					b3ClampImpulseColdW2( newImpulse.x, newImpulse.y, maxImpulse, true, &mask, &normalize );
				}
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

				B3_AUDIT_W( b3_auditFrictionImpulse, newImpulse.x );
				B3_AUDIT_W( b3_auditFrictionImpulse, newImpulse.y );
				B3_AUDIT_W( b3_auditTwistImpulse, c->twistImpulse );
				B3_AUDIT_W( b3_auditTangentMass, c->tangentMass.cxx );
				B3_AUDIT_W( b3_auditTangentMass, c->tangentMass.cyy );

				// Apply delta impulse; cross(origin, P) expands over the tangent rows
				b3Vec3W P = b3AddVW( b3MulSVW( deltaImpulse.x, tangent1 ), b3MulSVW( deltaImpulse.y, tangent2 ) );
				b3Vec3W LA = b3AddVW( b3MulSVW( deltaImpulse.x, b3WidenVW( c->rtA1s ) ),
									  b3MulSVW( deltaImpulse.y, b3WidenVW( c->rtA2s ) ) );
				b3Vec3W LB = b3AddVW( b3MulSVW( deltaImpulse.x, b3WidenVW( c->rtB1s ) ),
									  b3MulSVW( deltaImpulse.y, b3WidenVW( c->rtB2s ) ) );
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

		// Inactive point slots have totalNormalImpulses == 0, which masks the
		// effective mass to zero; skipping them is bit-identical.
		for ( int pointIndex = 0; pointIndex < c->maxPointCount; ++pointIndex )
		{
			b3ContactConstraintPointWide* cp = c->points + pointIndex;

			// Set effective mass to zero if restitution should not be applied
			b3FloatW mask1 = b3GreaterThanW( b3AddW( cp->relativeVelocities, threshold ), zero );
			b3FloatW mask2 = b3EqualsW( cp->totalNormalImpulses, zero );
			b3FloatW mask = b3OrW( b3OrW( mask1, mask2 ), restitutionMask );
			b3FloatW mass = b3BlendW( cp->normalMasses, zero, mask );

			// Relative velocity at contact through the precomputed Jacobian rows
			b3FloatW vn = b3RelVelocityW( b3SubVW( bB.v, bA.v ), b3WidenVW( c->normal ), bB.w, b3WidenVW( cp->rnBs ),
										  bA.w, b3WidenVW( cp->rnAs ) );

			// Compute normal impulse
			b3FloatW negImpulse = b3MulW( mass, b3AddW( vn, b3MulW( c->restitution, cp->relativeVelocities ) ) );

			// Clamp the accumulated impulse
			b3FloatW newImpulse = b3MaxW( b3SubW( cp->normalImpulses, negImpulse ), b3ZeroW() );
			b3FloatW deltaImpulse = b3SubW( newImpulse, cp->normalImpulses );
			cp->normalImpulses = newImpulse;
			cp->totalNormalImpulses = b3AddW( cp->totalNormalImpulses, deltaImpulse );

			// Apply contact impulse through the precomputed Jacobian rows
			bA.v = b3MulSubSVW( bA.v, deltaImpulse, b3WidenVW( c->mNormalA ) );
			bA.w = b3MulSubSVW( bA.w, deltaImpulse, b3WidenVW( cp->iRnAs ) );
			bB.v = b3MulAddSVW( bB.v, deltaImpulse, b3WidenVW( c->mNormalB ) );
			bB.w = b3MulAddSVW( bB.w, deltaImpulse, b3WidenVW( cp->iRnBs ) );
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
			const int32_t* tangent1X = (const int32_t*)&c->tangent1.X;
			const int32_t* tangent1Y = (const int32_t*)&c->tangent1.Y;
			const int32_t* tangent1Z = (const int32_t*)&c->tangent1.Z;
			const int32_t* tangent2X = (const int32_t*)&c->tangent2.X;
			const int32_t* tangent2Y = (const int32_t*)&c->tangent2.Y;
			const int32_t* tangent2Z = (const int32_t*)&c->tangent2.Z;
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

// Prepare the wide mesh constraints. All math here is the scalar
// b3PrepareContacts_Mesh, staged per lane into the wide layout, so the stored
// values are bit-identical to the scalar path. The wide constraint and its
// manifold slots are memset first; active lanes overwrite their elements and
// everything else keeps the exact zeros the solve stages rely on.
void b3PrepareContacts_MeshWide( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( prepare_contact, "Prepare Contact", b3_colorYellow, true );

	b3World* world = context->world;
	b3BodySim* bodySims = context->sims;
	b3BodyState* bodyStates = context->states;

	b3Fixed warmStartScale = world->enableWarmStarting ? B3_FIX( 1.0f ) : B3_FIX( 0.0f );

	// Used for friction center weighting.
	b3Fixed speculativeDistance = B3_SPECULATIVE_DISTANCE;

	b3MeshWidePrepareSpan* spans = context->meshWidePrepareSpans;
	b3ContactConstraintMeshWide* wideBase = context->meshWideConstraints;
	b3ManifoldConstraintMeshWide* slotBase = context->meshWideManifolds;
	const int* slotStarts = context->meshWideManifoldStarts;

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
		b3ContactSpec* specs = spans[colorIndex].contacts;

		// Loop over color
		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			b3ContactConstraintMeshWide* constraint = wideBase + wideIndex;
			b3ManifoldConstraintMeshWide* slots = slotBase + slotStarts[wideIndex];
			int localWideIndex = wideIndex - colorWideStart;

			// The widest lane decides how many manifold slots this group owns;
			// solver setup sized the slot array from the same spec counts.
			int maxManifoldCount = 0;
			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex >= colorContactCount )
				{
					break;
				}
				maxManifoldCount = b3MaxInt( maxManifoldCount, specs[contactIndex].manifoldCount );
			}

			memset( constraint, 0, sizeof( *constraint ) );
			memset( slots, 0, maxManifoldCount * sizeof( *slots ) );
			constraint->manifoldConstraints = slots;
			constraint->maxManifoldCount = maxManifoldCount;

			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex >= colorContactCount )
				{
					// Remainder lanes keep the zeros from the memset above.
					break;
				}

				int contactId = specs[contactIndex].contactId;
				b3Contact* contact = b3Array_Get( world->contacts, contactId );
				B3_ASSERT( contact->contactId == contactId );

				int indexA = contact->bodySimIndexA;
				int indexB = contact->bodySimIndexB;

#if B3_ENABLE_VALIDATION
				if ( indexA != B3_NULL_INDEX )
				{
					b3Body* bodyA = b3Array_Get( world->bodies, contact->edges[0].bodyId );
					B3_ASSERT( indexA == bodyA->localIndex );
				}

				if ( indexB != B3_NULL_INDEX )
				{
					b3Body* bodyB = b3Array_Get( world->bodies, contact->edges[1].bodyId );
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
				B3_ASSERT( manifoldCount == (int)specs[contactIndex].manifoldCount );

				// 0 for null
				constraint->indexA[lane] = indexA + 1;
				constraint->indexB[lane] = indexB + 1;
				constraint->contacts[lane] = contact;
				( (b3Fixed*)&constraint->manifoldCounts )[lane] = manifoldCount;

				( (b3Fixed*)&constraint->invMassA )[lane] = mA;
				( (b3Fixed*)&constraint->invMassB )[lane] = mB;

				( (b3Fixed*)&constraint->invIA.cxx )[lane] = iA.cx.x;
				( (b3Fixed*)&constraint->invIA.cxy )[lane] = iA.cx.y;
				( (b3Fixed*)&constraint->invIA.cxz )[lane] = iA.cx.z;
				( (b3Fixed*)&constraint->invIA.cyx )[lane] = iA.cy.x;
				( (b3Fixed*)&constraint->invIA.cyy )[lane] = iA.cy.y;
				( (b3Fixed*)&constraint->invIA.cyz )[lane] = iA.cy.z;
				( (b3Fixed*)&constraint->invIA.czx )[lane] = iA.cz.x;
				( (b3Fixed*)&constraint->invIA.czy )[lane] = iA.cz.y;
				( (b3Fixed*)&constraint->invIA.czz )[lane] = iA.cz.z;

				( (b3Fixed*)&constraint->invIB.cxx )[lane] = iB.cx.x;
				( (b3Fixed*)&constraint->invIB.cxy )[lane] = iB.cx.y;
				( (b3Fixed*)&constraint->invIB.cxz )[lane] = iB.cx.z;
				( (b3Fixed*)&constraint->invIB.cyx )[lane] = iB.cy.x;
				( (b3Fixed*)&constraint->invIB.cyy )[lane] = iB.cy.y;
				( (b3Fixed*)&constraint->invIB.cyz )[lane] = iB.cy.z;
				( (b3Fixed*)&constraint->invIB.czx )[lane] = iB.cz.x;
				( (b3Fixed*)&constraint->invIB.czy )[lane] = iB.cz.y;
				( (b3Fixed*)&constraint->invIB.czz )[lane] = iB.cz.z;

				// The 128-bit matrix inversion is only needed when rolling resistance is active
				b3Matrix3 rollingMass =
					contact->rollingResistance > B3_FIX( 0.0f ) ? b3InvertMatrix( b3AddMM( iA, iB ) ) : b3Mat3_zero;
				( (b3Fixed*)&constraint->rollingMass.cxx )[lane] = rollingMass.cx.x;
				( (b3Fixed*)&constraint->rollingMass.cxy )[lane] = rollingMass.cx.y;
				( (b3Fixed*)&constraint->rollingMass.cxz )[lane] = rollingMass.cx.z;
				( (b3Fixed*)&constraint->rollingMass.cyx )[lane] = rollingMass.cy.x;
				( (b3Fixed*)&constraint->rollingMass.cyy )[lane] = rollingMass.cy.y;
				( (b3Fixed*)&constraint->rollingMass.cyz )[lane] = rollingMass.cy.z;
				( (b3Fixed*)&constraint->rollingMass.czx )[lane] = rollingMass.cz.x;
				( (b3Fixed*)&constraint->rollingMass.czy )[lane] = rollingMass.cz.y;
				( (b3Fixed*)&constraint->rollingMass.czz )[lane] = rollingMass.cz.z;

				// Stiffer for static contacts to avoid bodies getting pushed through the ground.
				// Note: the mesh path selects by the contact flag, not by null body indices.
				b3Softness soft =
					( contact->flags & b3_contactStaticFlag ) != 0 ? context->staticSoftness : context->contactSoftness;
				( (b3Fixed*)&constraint->biasRate )[lane] = soft.biasRate;
				( (b3Fixed*)&constraint->massScale )[lane] = soft.massScale;
				( (b3Fixed*)&constraint->impulseScale )[lane] = soft.impulseScale;

				( (b3Fixed*)&constraint->friction )[lane] = contact->friction;
				( (b3Fixed*)&constraint->restitution )[lane] = contact->restitution;
				( (b3Fixed*)&constraint->rollingResistance )[lane] = contact->rollingResistance;

				for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
				{
					b3Manifold* manifold = contact->manifolds + manifoldIndex;
					b3ManifoldConstraintMeshWide* slot = slots + manifoldIndex;
					int pointCount = manifold->pointCount;
					b3Vec3 normal = manifold->normal;
					b3Vec3 tangent1 = b3Perp( normal );
					b3Vec3 tangent2 = b3Cross( tangent1, normal );

					slot->maxPointCount = b3MaxInt( slot->maxPointCount, pointCount );

					( (b3Fixed*)&slot->normal.X )[lane] = normal.x;
					( (b3Fixed*)&slot->normal.Y )[lane] = normal.y;
					( (b3Fixed*)&slot->normal.Z )[lane] = normal.z;
					( (b3Fixed*)&slot->tangent1.X )[lane] = tangent1.x;
					( (b3Fixed*)&slot->tangent1.Y )[lane] = tangent1.y;
					( (b3Fixed*)&slot->tangent1.Z )[lane] = tangent1.z;
					( (b3Fixed*)&slot->tangent2.X )[lane] = tangent2.x;
					( (b3Fixed*)&slot->tangent2.Y )[lane] = tangent2.y;
					( (b3Fixed*)&slot->tangent2.Z )[lane] = tangent2.z;

					( (b3Fixed*)&slot->tangentVelocity1 )[lane] = b3Dot( contact->tangentVelocity, tangent1 );
					( (b3Fixed*)&slot->tangentVelocity2 )[lane] = b3Dot( contact->tangentVelocity, tangent2 );

					b3Vec3 centerA = b3Vec3_zero;
					b3Vec3 centerB = b3Vec3_zero;
					b3Fixed totalFrictionWeight = B3_FIX( 0.0f );

					for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
					{
						b3ManifoldConstraintPointMeshWide* cp = slot->points + pointIndex;

						// Copy data from manifold point
						b3ManifoldPoint* mp = manifold->points + pointIndex;
						b3Vec3 rA = mp->anchorA;
						b3Vec3 rB = mp->anchorB;

						( (b3Fixed*)&cp->rA.X )[lane] = rA.x;
						( (b3Fixed*)&cp->rA.Y )[lane] = rA.y;
						( (b3Fixed*)&cp->rA.Z )[lane] = rA.z;
						( (b3Fixed*)&cp->rB.X )[lane] = rB.x;
						( (b3Fixed*)&cp->rB.Y )[lane] = rB.y;
						( (b3Fixed*)&cp->rB.Z )[lane] = rB.z;

						b3Fixed s = mp->separation;
						( (b3Fixed*)&cp->baseSeparation )[lane] = s - b3Dot( b3Sub( rB, rA ), normal );
						( (b3Fixed*)&cp->normalImpulse )[lane] = b3FixMul( warmStartScale, mp->normalImpulse );
						// totalNormalImpulse keeps the zero from the memset

						b3Vec3 rnA = b3Cross( rA, normal );
						b3Vec3 rnB = b3Cross( rB, normal );
						b3Fixed kNormal = mA + mB + b3Dot( rnA, b3MulMV( iA, rnA ) ) + b3Dot( rnB, b3MulMV( iB, rnB ) );
						( (b3Fixed*)&cp->normalMass )[lane] =
							kNormal > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ), kNormal ) : B3_FIX( 0.0f );

						// Save relative velocity for restitution
						b3Vec3 vrA = b3Add( vA, b3Cross( wA, rA ) );
						b3Vec3 vrB = b3Add( vB, b3Cross( wB, rB ) );
						( (b3Fixed*)&cp->relativeVelocity )[lane] = b3Dot( normal, b3Sub( vrB, vrA ) );

						// C0 friction center decay. See b3PrepareContacts_Mesh; keep in sync.
						b3Fixed weight = b3FixClamp( B3_FIX( 2.0f ) - b3FixDiv( s, speculativeDistance ),
													 B3_MIN_FRICTION_WEIGHT, B3_FIX( 1.0f ) );
						centerA = b3MulAdd( centerA, weight, rA );
						centerB = b3MulAdd( centerB, weight, rB );
						totalFrictionWeight += weight;
					}

					b3Fixed invWeight = b3FixDiv( B3_FIX( 1.0f ), totalFrictionWeight );
					centerA = b3MulSV( invWeight, centerA );
					centerB = b3MulSV( invWeight, centerB );
					( (b3Fixed*)&slot->centerA.X )[lane] = centerA.x;
					( (b3Fixed*)&slot->centerA.Y )[lane] = centerA.y;
					( (b3Fixed*)&slot->centerA.Z )[lane] = centerA.z;
					( (b3Fixed*)&slot->centerB.X )[lane] = centerB.x;
					( (b3Fixed*)&slot->centerB.Y )[lane] = centerB.y;
					( (b3Fixed*)&slot->centerB.Z )[lane] = centerB.z;

					for ( int pointIndex = 0; pointIndex < pointCount; ++pointIndex )
					{
						b3ManifoldPoint* mp = manifold->points + pointIndex;
						( (b3Fixed*)&slot->points[pointIndex].leverArm )[lane] = b3Distance( mp->anchorA, centerA );
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

						b3Matrix2 tangentMass = b3Invert2( k );
						B3_ASSERT( tangentMass.cx.y == tangentMass.cy.x );
						( (b3Fixed*)&slot->tangentMass.cxx )[lane] = tangentMass.cx.x;
						( (b3Fixed*)&slot->tangentMass.cxy )[lane] = tangentMass.cx.y;
						( (b3Fixed*)&slot->tangentMass.cyy )[lane] = tangentMass.cy.y;

						( (b3Fixed*)&slot->frictionImpulse.x )[lane] =
							b3FixMul( warmStartScale, b3Dot( manifold->frictionImpulse, tangent1 ) );
						( (b3Fixed*)&slot->frictionImpulse.y )[lane] =
							b3FixMul( warmStartScale, b3Dot( manifold->frictionImpulse, tangent2 ) );
					}

					{
						b3Fixed k = b3Dot( normal, b3MulMV( b3AddMM( iA, iB ), normal ) );
						( (b3Fixed*)&slot->twistMass )[lane] =
							k > B3_FIX( 0.0f ) ? b3FixDiv( B3_FIX( 1.0f ), k ) : B3_FIX( 0.0f );
						( (b3Fixed*)&slot->twistImpulse )[lane] = b3FixMul( warmStartScale, manifold->twistImpulse );
					}

					{
						b3Vec3 rollingImpulse = b3MulSV( warmStartScale, manifold->rollingImpulse );
						( (b3Fixed*)&slot->rollingImpulse.X )[lane] = rollingImpulse.x;
						( (b3Fixed*)&slot->rollingImpulse.Y )[lane] = rollingImpulse.y;
						( (b3Fixed*)&slot->rollingImpulse.Z )[lane] = rollingImpulse.z;
					}
				}
			}
		}

		// Advance to next color
		colorIndex += 1;
	}

	b3TracyCZoneEnd( prepare_contact );
}

void b3WarmStartContacts_MeshWide( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( warm_start_contact, "Warm Start", b3_colorGreen, true );

	b3BodyState* states = context->states;
	b3ContactConstraintMeshWide* constraints = context->graph->colors[block.colorIndex].meshWideConstraints;

	for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
	{
		b3ContactConstraintMeshWide* c = constraints + i;
		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		for ( int manifoldIndex = 0; manifoldIndex < c->maxManifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraintMeshWide* m = c->manifoldConstraints + manifoldIndex;
			b3Vec3W normal = m->normal;

			// Normal impulses. Slots past a lane's own manifold or point count
			// hold exact zeros and apply nothing; visiting them is bit-identical.
			for ( int j = 0; j < m->maxPointCount; ++j )
			{
				b3ManifoldConstraintPointMeshWide* cp = m->points + j;

				b3Vec3W impulse = b3MulSVW( cp->normalImpulse, normal );
				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, b3CrossUnfusedW( cp->rA, impulse ) ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, impulse );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, b3CrossUnfusedW( cp->rB, impulse ) ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, impulse );
			}

			// Central friction
			{
				b3Vec3W impulse = b3MulSVW( m->frictionImpulse.x, m->tangent1 );
				impulse = b3AddVW( impulse, b3MulSVW( m->frictionImpulse.y, m->tangent2 ) );

				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, b3CrossUnfusedW( m->centerA, impulse ) ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, impulse );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, b3CrossUnfusedW( m->centerB, impulse ) ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, impulse );
			}

			// Central twist friction
			{
				b3Vec3W impulse = b3MulSVW( m->twistImpulse, normal );
				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, impulse ) );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, impulse ) );
			}

			// Rolling resistance
			{
				b3Vec3W impulse = m->rollingImpulse;
				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, impulse ) );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, impulse ) );
			}
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );
	}

	b3TracyCZoneEnd( warm_start_contact );
}

// Merged normal and friction loops, wide. Each lane reproduces
// b3SolveContacts_Mesh exactly; see the layout comment above.
void b3SolveContacts_MeshWide( b3SolverBlock block, b3StepContext* context, bool useBias )
{
	b3TracyCZoneNC( solve_contact, "Solve Contact", b3_colorAliceBlue, true );

	b3BodyState* states = context->states;
	b3ContactConstraintMeshWide* constraints = context->graph->colors[block.colorIndex].meshWideConstraints;
	b3FloatW inv_h = b3SplatW( context->inv_h );
	b3FloatW negContactSpeed = b3SplatW( -context->world->contactSpeed );
	b3FloatW oneW = b3SplatW( B3_FIX( 1.0f ) );
	b3FloatW zeroW = b3ZeroW();
	b3FloatW epsilonW = b3SplatW( B3_FIXED_EPSILON );

	for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
	{
		b3ContactConstraintMeshWide* c = constraints + wideIndex;

		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		// Scalar form: velocityBias = max(FixMul(FixMul(massScale, biasRate), s), -contactSpeed)
		b3FloatW biasRate, massScale, impulseScale;
		if ( useBias )
		{
			biasRate = b3MulW( c->massScale, c->biasRate );
			massScale = c->massScale;
			impulseScale = c->impulseScale;
		}
		else
		{
			biasRate = zeroW;
			massScale = oneW;
			impulseScale = zeroW;
		}

		b3Vec3W dp = b3SubVW( bB.dp, bA.dp );
		b3FloatW friction = c->friction;

		bool anyRolling = b3AllZeroW( c->rollingResistance ) == false;

		for ( int manifoldIndex = 0; manifoldIndex < c->maxManifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraintMeshWide* m = c->manifoldConstraints + manifoldIndex;

			b3Vec3W normal = m->normal;

			b3FloatW totalNormalImpulse = zeroW;
			b3FloatW totalTwistLimit = zeroW;

			for ( int pointIndex = 0; pointIndex < m->maxPointCount; ++pointIndex )
			{
				b3ManifoldConstraintPointMeshWide* cp = m->points + pointIndex;

				// Fixed anchor points for applying impulses
				b3Vec3W rA = cp->rA;
				b3Vec3W rB = cp->rB;

				// compute current separation with the exact scalar b3RotateVector form
				b3Vec3W ds = b3AddVW( dp, b3SubVW( b3RotateVectorW( bB.dq, rB ), b3RotateVectorW( bA.dq, rA ) ) );
				b3FloatW s = b3AddW( b3DotW( ds, normal ), cp->baseSeparation );

				// Speculative bias for s > 0, otherwise the soft constraint bias
				b3FloatW specMask = b3GreaterThanW( s, zeroW );
				b3FloatW specBias = b3MulW( s, inv_h );
				b3FloatW softBias = b3MaxW( b3MulW( biasRate, s ), negContactSpeed );
				b3FloatW velocityBias = b3BlendW( softBias, specBias, specMask );
				b3FloatW pointMassScale = b3BlendW( massScale, oneW, specMask );
				b3FloatW pointImpulseScale = b3BlendW( impulseScale, zeroW, specMask );

				// relative normal velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossUnfusedW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossUnfusedW( bB.w, rB ) );
				b3FloatW vn = b3DotW( b3SubVW( vrB, vrA ), normal );

				// incremental normal impulse: multiply by the negated mass, like the
				// scalar FixMul(-normalMass, ...) (round-half-up is not odd-symmetric)
				b3FloatW deltaImpulse =
					b3SubW( b3MulW( b3NegW( cp->normalMass ), b3AddW( b3MulW( pointMassScale, vn ), velocityBias ) ),
							b3MulW( pointImpulseScale, cp->normalImpulse ) );

				// clamp the accumulated impulse
				b3FloatW newImpulse = b3MaxW( b3AddW( cp->normalImpulse, deltaImpulse ), zeroW );
				deltaImpulse = b3SubW( newImpulse, cp->normalImpulse );
				cp->normalImpulse = newImpulse;
				cp->totalNormalImpulse = b3AddW( cp->totalNormalImpulse, newImpulse );

				totalNormalImpulse = b3AddW( totalNormalImpulse, newImpulse );
				totalTwistLimit = b3AddW( totalTwistLimit, b3MulW( cp->leverArm, newImpulse ) );

				// apply normal impulse
				b3Vec3W P = b3MulSVW( deltaImpulse, normal );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, b3CrossUnfusedW( rA, P ) ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, b3CrossUnfusedW( rB, P ) ) );
			}

			// No friction when applying bias
			if ( useBias == true )
			{
				// Go to next manifold
				continue;
			}

			// Central twist friction
			{
				b3FloatW twistSpeed = b3DotW( normal, b3SubVW( bB.w, bA.w ) );
				b3FloatW maxImpulse = b3MulW( friction, totalTwistLimit );
				B3_AUDIT_W( b3_auditTwistCap, maxImpulse );
				b3FloatW deltaImpulse = b3MulW( b3NegW( m->twistMass ), twistSpeed );
				b3FloatW oldImpulse = m->twistImpulse;
				m->twistImpulse = b3SymClampW( b3AddW( oldImpulse, deltaImpulse ), maxImpulse );
				deltaImpulse = b3SubW( m->twistImpulse, oldImpulse );

				b3Vec3W P = b3MulSVW( deltaImpulse, normal );
				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, P ) );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, P ) );
			}

			// Rolling resistance. The rolling mass and resistance are per-contact,
			// so a lane whose manifold count ends before this slot (or whose
			// resistance is zero) is not self-neutralizing: blend the stored
			// impulse back to the old value so those lanes apply nothing, exactly
			// like the scalar per-contact branch.
			if ( anyRolling )
			{
				b3FloatW active = b3GreaterThanW( c->manifoldCounts, b3SplatW( (b3Fixed)manifoldIndex ) );
				active = b3BlendW( zeroW, b3GreaterThanW( c->rollingResistance, zeroW ), active );

				b3Vec3W deltaImpulse = b3NegVW( b3MulMVFullW( &c->rollingMass, b3SubVW( bB.w, bA.w ) ) );
				b3Vec3W oldImpulse = m->rollingImpulse;
				b3Vec3W rollingImpulse = b3AddVW( oldImpulse, deltaImpulse );

				b3FloatW maxImpulse = b3MulW( c->rollingResistance, totalNormalImpulse );
				B3_AUDIT_W( b3_auditRollingCap, maxImpulse );

				// The squared compare wraps int64 past B3_IMPULSE_CAP_GATE; the cold
				// path is exact (see the cap-guard block above b3SolveContacts_Mesh)
				b3FloatW clampMask, scale;
				if ( b3CapOperandsSafeW4( rollingImpulse.X, rollingImpulse.Y, rollingImpulse.Z, maxImpulse ) )
				{
					b3FloatW magSqr = b3DotW( rollingImpulse, rollingImpulse );
					clampMask = b3GreaterThanW( magSqr, b3AddW( b3MulW( maxImpulse, maxImpulse ), epsilonW ) );
					scale = b3DivW( maxImpulse, b3SqrtW( magSqr ) );
				}
				else
				{
					b3ClampImpulseColdW3( rollingImpulse, maxImpulse, false, &clampMask, &scale );
				}
				rollingImpulse = b3BlendVW( rollingImpulse, b3MulSVW( scale, rollingImpulse ), clampMask );

				rollingImpulse = b3BlendVW( oldImpulse, rollingImpulse, active );
				m->rollingImpulse = rollingImpulse;

				deltaImpulse = b3SubVW( rollingImpulse, oldImpulse );

				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, deltaImpulse ) );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, deltaImpulse ) );
			}

			// Central friction
			{
				b3Vec3W tangent1 = m->tangent1;
				b3Vec3W tangent2 = m->tangent2;

				// Fixed anchor points for applying impulses
				b3Vec3W rA = m->centerA;
				b3Vec3W rB = m->centerB;

				// Relative tangent velocity at contact
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossUnfusedW( bA.w, rA ) );
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossUnfusedW( bB.w, rB ) );
				b3Vec3W vr = b3SubVW( vrB, vrA );
				b3Vec2W vt = {
					b3SubW( b3DotW( vr, tangent1 ), m->tangentVelocity1 ),
					b3SubW( b3DotW( vr, tangent2 ), m->tangentVelocity2 ),
				};

				// Incremental tangent impulse; the scalar b3MulMV2 is unfused
				b3Vec2W tm = b3MulMV2UnfusedW( m->tangentMass, vt );
				b3Vec2W deltaImpulse = { b3NegW( tm.x ), b3NegW( tm.y ) };
				b3Vec2W newImpulse = b3AddV2W( m->frictionImpulse, deltaImpulse );

				b3FloatW maxImpulse = b3MulW( friction, totalNormalImpulse );
				B3_AUDIT_W( b3_auditIterNormalSum, totalNormalImpulse );
				B3_AUDIT_W( b3_auditFrictionCap, maxImpulse );

				// Clamp the accumulated impulse; the scalar b3Dot2 is unfused. The
				// squared compare wraps int64 past B3_IMPULSE_CAP_GATE; the cold path
				// is exact (see the cap-guard block above b3SolveContacts_Mesh).
				b3FloatW clampMask, scale;
				if ( b3CapOperandsSafeW3( newImpulse.x, newImpulse.y, maxImpulse ) )
				{
					b3FloatW lengthSquared = b3AddW( b3MulW( newImpulse.x, newImpulse.x ), b3MulW( newImpulse.y, newImpulse.y ) );
					clampMask = b3GreaterThanW( lengthSquared, b3MulW( maxImpulse, maxImpulse ) );
					scale = b3DivW( maxImpulse, b3SqrtW( lengthSquared ) );
				}
				else
				{
					b3ClampImpulseColdW2( newImpulse.x, newImpulse.y, maxImpulse, false, &clampMask, &scale );
				}
				newImpulse.x = b3BlendW( newImpulse.x, b3MulW( newImpulse.x, scale ), clampMask );
				newImpulse.y = b3BlendW( newImpulse.y, b3MulW( newImpulse.y, scale ), clampMask );

				deltaImpulse.x = b3SubW( newImpulse.x, m->frictionImpulse.x );
				deltaImpulse.y = b3SubW( newImpulse.y, m->frictionImpulse.y );
				m->frictionImpulse = newImpulse;

				// Apply delta impulse
				b3Vec3W P = b3AddVW( b3MulSVW( deltaImpulse.x, tangent1 ), b3MulSVW( deltaImpulse.y, tangent2 ) );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, b3CrossUnfusedW( rA, P ) ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, b3CrossUnfusedW( rB, P ) ) );
			}
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );
	}

	b3TracyCZoneEnd( solve_contact );
}

void b3ApplyRestitution_MeshWide( b3SolverBlock block, b3StepContext* context )
{
	b3TracyCZoneNC( restitution, "Restitution", b3_colorDodgerBlue, true );

	b3BodyState* states = context->states;
	b3ContactConstraintMeshWide* constraints = context->graph->colors[block.colorIndex].meshWideConstraints;
	b3FloatW negThreshold = b3SplatW( -context->world->restitutionThreshold );
	b3FloatW zeroW = b3ZeroW();

	for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
	{
		b3ContactConstraintMeshWide* c = constraints + i;

		if ( b3AllZeroW( c->restitution ) )
		{
			// No lanes have restitution. Common case.
			continue;
		}

		b3BodyStateW bA = b3GatherBodies( states, c->indexA );
		b3BodyStateW bB = b3GatherBodies( states, c->indexB );

		// Lanes with no restitution keep an effective mass of zero below
		b3FloatW restitutionMask = b3EqualsW( c->restitution, zeroW );

		for ( int manifoldIndex = 0; manifoldIndex < c->maxManifoldCount; ++manifoldIndex )
		{
			b3ManifoldConstraintMeshWide* m = c->manifoldConstraints + manifoldIndex;
			b3Vec3W normal = m->normal;

			for ( int pointIndex = 0; pointIndex < m->maxPointCount; ++pointIndex )
			{
				b3ManifoldConstraintPointMeshWide* cp = m->points + pointIndex;

				// Zero the effective mass for lanes the scalar loop skips:
				// speculative points and points that never generated an impulse
				b3FloatW mask1 = b3GreaterThanW( cp->relativeVelocity, negThreshold );
				b3FloatW mask2 = b3EqualsW( cp->totalNormalImpulse, zeroW );
				b3FloatW mask = b3OrW( b3OrW( mask1, mask2 ), restitutionMask );
				b3FloatW mass = b3BlendW( cp->normalMass, zeroW, mask );

				// relative normal velocity at contact
				b3Vec3W vrB = b3AddVW( bB.v, b3CrossUnfusedW( bB.w, cp->rB ) );
				b3Vec3W vrA = b3AddVW( bA.v, b3CrossUnfusedW( bA.w, cp->rA ) );
				b3FloatW vn = b3DotW( b3SubVW( vrB, vrA ), normal );

				// compute normal impulse with the negated mass (see the solve loop)
				b3FloatW impulse = b3MulW( b3NegW( mass ), b3AddW( vn, b3MulW( c->restitution, cp->relativeVelocity ) ) );

				// clamp the accumulated impulse
				b3FloatW newImpulse = b3MaxW( b3AddW( cp->normalImpulse, impulse ), zeroW );
				impulse = b3SubW( newImpulse, cp->normalImpulse );
				cp->normalImpulse = newImpulse;
				cp->totalNormalImpulse = b3AddW( cp->totalNormalImpulse, impulse );

				// apply contact impulse
				b3Vec3W P = b3MulSVW( impulse, normal );
				bA.v = b3MulSubSVW( bA.v, c->invMassA, P );
				bA.w = b3SubVW( bA.w, b3MulMVFullW( &c->invIA, b3CrossUnfusedW( cp->rA, P ) ) );
				bB.v = b3MulAddSVW( bB.v, c->invMassB, P );
				bB.w = b3AddVW( bB.w, b3MulMVFullW( &c->invIB, b3CrossUnfusedW( cp->rB, P ) ) );
			}
		}

		b3ScatterBodies( states, c->indexA, &bA );
		b3ScatterBodies( states, c->indexB, &bB );
	}

	b3TracyCZoneEnd( restitution );
}

// Store impulses back to the manifolds, mirroring b3StoreImpulses_Mesh per lane
void b3StoreImpulses_MeshWide( b3SolverBlock block, b3StepContext* context, int workerIndex )
{
	b3TracyCZoneNC( store_impulses, "Store", b3_colorFireBrick, true );

	b3World* world = context->world;
	b3MeshWidePrepareSpan* spans = context->meshWidePrepareSpans;
	const b3ContactConstraintMeshWide* wideBase = context->meshWideConstraints;

	b3TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b3BitSet* hitEventBitSet = &taskContext->hitEventBitSet;
	bool hasHitEvents = taskContext->hasHitEvents;
	b3Fixed negHitThreshold = -world->hitEventThreshold;

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
		b3ContactSpec* specs = spans[colorIndex].contacts;
		B3_UNUSED( specs );

		// Loop over color
		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			const b3ContactConstraintMeshWide* c = wideBase + wideIndex;
			int localWideIndex = wideIndex - colorWideStart;

			for ( int lane = 0; lane < B3_SIMD_WIDTH; ++lane )
			{
				int contactIndex = B3_SIMD_WIDTH * localWideIndex + lane;
				if ( contactIndex >= colorContactCount )
				{
					break;
				}

				b3Contact* contact = c->contacts[lane];
				B3_ASSERT( contact != NULL );

				// Catches a wrong-(base, spans) pairing, see b3StoreImpulses_Mesh
				B3_VALIDATE( contact->contactId == specs[contactIndex].contactId );

				int manifoldCount = contact->manifoldCount;
				B3_ASSERT( manifoldCount <= c->maxManifoldCount );

				bool checkHitEvents = ( contact->flags & b3_simEnableHitEvent ) != 0;
				bool flagged = false;

				for ( int manifoldIndex = 0; manifoldIndex < manifoldCount; ++manifoldIndex )
				{
					b3Manifold* manifold = contact->manifolds + manifoldIndex;
					const b3ManifoldConstraintMeshWide* m = c->manifoldConstraints + manifoldIndex;

					b3Fixed f1 = ( (const b3Fixed*)&m->frictionImpulse.x )[lane];
					b3Fixed f2 = ( (const b3Fixed*)&m->frictionImpulse.y )[lane];
					b3Vec3 tangent1 = { ( (const b3Fixed*)&m->tangent1.X )[lane], ( (const b3Fixed*)&m->tangent1.Y )[lane],
										( (const b3Fixed*)&m->tangent1.Z )[lane] };
					b3Vec3 tangent2 = { ( (const b3Fixed*)&m->tangent2.X )[lane], ( (const b3Fixed*)&m->tangent2.Y )[lane],
										( (const b3Fixed*)&m->tangent2.Z )[lane] };

					manifold->twistImpulse = ( (const b3Fixed*)&m->twistImpulse )[lane];
					manifold->frictionImpulse = b3Blend2( f1, tangent1, f2, tangent2 );
					manifold->rollingImpulse = (b3Vec3){
						( (const b3Fixed*)&m->rollingImpulse.X )[lane],
						( (const b3Fixed*)&m->rollingImpulse.Y )[lane],
						( (const b3Fixed*)&m->rollingImpulse.Z )[lane],
					};

					int count = manifold->pointCount;
					B3_ASSERT( count <= m->maxPointCount );
					for ( int pointIndex = 0; pointIndex < count; ++pointIndex )
					{
						const b3ManifoldConstraintPointMeshWide* cp = m->points + pointIndex;
						b3ManifoldPoint* mp = manifold->points + pointIndex;
						mp->normalImpulse = ( (const b3Fixed*)&cp->normalImpulse )[lane];
						mp->totalNormalImpulse = ( (const b3Fixed*)&cp->totalNormalImpulse )[lane];
						mp->normalVelocity = ( (const b3Fixed*)&cp->relativeVelocity )[lane];

						if ( checkHitEvents && flagged == false && mp->normalVelocity < negHitThreshold &&
							 mp->totalNormalImpulse > B3_FIX( 0.0f ) )
						{
							b3SetBit( hitEventBitSet, contact->contactId );
							hasHitEvents = true;
							flagged = true;
						}
					}
				}
			}
		}

		// Advance to next color
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
