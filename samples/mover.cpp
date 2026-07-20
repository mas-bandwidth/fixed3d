// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "mover.h"

#include "gfx/draw.h"
#include "sample.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <float.h>
#include <gfx/keycodes.h>

static bool MoverFilterCallback( b3ShapeId shapeId, void* context )
{
	CharacterMover* self = (CharacterMover*)context;
	for ( int i = 0; i < self->m_ignoreCount; ++i )
	{
		if ( B3_ID_EQUALS( shapeId, self->m_ignoreShapeIds[i] ) )
		{
			return false;
		}
	}

	return true;
}

void CharacterMover::Initialize( Sample* sample, b3Pos position )
{
	m_sample = sample;
	m_transform.p = position;
	m_transform.q = b3Quat_identity;
	m_velocity = { B3_FIX( 0.0f ), B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	m_capsule = { { B3_FIX( 0.0f ), B3_FIX( -0.5f ), B3_FIX( 0.0f ) }, { B3_FIX( 0.0f ), B3_FIX( 0.5f ), B3_FIX( 0.0f ) }, B3_FIX( 0.3f ) };

	m_planeCount = 0;
	m_totalIterations = 0;
	m_pogoVelocity = 0.0f;
	m_onGround = false;
	m_sprint = false;

	m_ignoreShapeIds = nullptr;
	m_ignoreCount = 0;
}

static bool PlaneResultFcn( b3ShapeId shapeId, const b3PlaneResult* planeResults, int planeCount, void* context )
{
	if ( MoverFilterCallback( shapeId, context ) == false )
	{
		// ignore these planes but continue looking for more
		return true;
	}

	CharacterMover* self = static_cast<CharacterMover*>( context );

	// The float code used FLT_MAX as "no limit"; the fixed-point analog is B3_FIXED_MAX.
	// b3FixFromFloat( FLT_MAX ) would overflow the int64 cast, so map huge values explicitly.
	b3Fixed pushLimit = B3_FIXED_MAX;
	bool clipVelocity = true;
	MoverShapeUserData* userData = static_cast<MoverShapeUserData*>( (void*)b3Shape_GetUserData( shapeId ) );
	if ( userData != nullptr )
	{
		pushLimit = userData->maxPush < FLT_MAX ? b3FixFromFloat( userData->maxPush ) : B3_FIXED_MAX;
		clipVelocity = userData->clipVelocity;
	}

	for ( int i = 0; i < planeCount && self->m_planeCount < CharacterMover::m_planeCapacity; ++i )
	{
		assert( b3IsValidPlane( planeResults[i].plane ) );
		self->m_planes[self->m_planeCount] = {
			.plane = planeResults[i].plane,
			.pushLimit = pushLimit,
			.push = B3_FIX( 0.0f ),
			.clipVelocity = clipVelocity,
		};
		self->m_planeExtras[self->m_planeCount] = {
			.point = b3OffsetPos( self->m_transform.p, planeResults[i].point ),
			.shapeId = shapeId,
		};
		self->m_planeCount += 1;
	}

	return true;
}

void CharacterMover::SolveMove( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle, bool clipVelocity )
{
	// Friction
	b3Fixed speed = b3Length( m_velocity );
	if ( speed < b3FixFromFloat( m_minSpeed ) )
	{
		m_velocity.x = B3_FIX( 0.0f );
		m_velocity.z = B3_FIX( 0.0f );
	}
	else
	{
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		b3Fixed stopSpeed = b3FixFromFloat( m_stopSpeed );
		b3Fixed control = speed < stopSpeed ? stopSpeed : speed;

		// friction has units of 1/time
		b3Fixed drop = b3FixMul( control, b3FixFromFloat( m_friction * timeStep ) );
		b3Fixed newSpeed = b3FixMax( B3_FIX( 0.0f ), speed - drop );
		b3Fixed ratio = b3FixDiv( newSpeed, speed );
		m_velocity.x = b3FixMul( m_velocity.x, ratio );
		m_velocity.z = b3FixMul( m_velocity.z, ratio );
	}

	b3Fixed maxSpeed = b3FixFromFloat( m_sprint ? 1.5f * m_maxSpeed : m_maxSpeed );

	b3Vec3 desiredVelocity = b3FixMul( maxSpeed, throttle.x ) * forward + b3FixMul( maxSpeed, throttle.y ) * right;
	b3Fixed desiredSpeed;
	b3Vec3 desiredDirection = b3GetLengthAndNormalize( &desiredSpeed, desiredVelocity );

	if ( desiredSpeed > maxSpeed )
	{
		desiredVelocity *= b3FixDiv( maxSpeed, desiredSpeed );
		desiredSpeed = maxSpeed;
	}

	if ( m_onGround )
	{
		m_velocity.y = B3_FIX( 0.0f );
	}

	// Accelerate
	b3Fixed currentSpeed = b3Dot( m_velocity, desiredDirection );
	b3Fixed addSpeed = desiredSpeed - currentSpeed;
	if ( addSpeed > B3_FIX( 0.0f ) )
	{
		b3Fixed accelSpeed = b3FixMul( b3FixFromFloat( m_accelerate * timeStep ), maxSpeed );
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		m_velocity += accelSpeed * desiredDirection;
	}

	m_velocity.y -= b3FixFromFloat( m_gravity * timeStep );

	b3WorldId worldId = m_sample->m_worldId;

	b3Fixed pogoRestLength = 3 * m_capsule.radius;
	b3Fixed rayLength = pogoRestLength + m_capsule.radius;
	b3Pos rayOrigin = b3TransformWorldPoint( m_transform, m_capsule.center1 );
	b3Vec3 rayTranslation = -rayLength * b3Vec3_axisY;
	b3QueryFilter skipTeamFilter = { 1, ~2u };
	skipTeamFilter.name = "pogo";
	b3RayResult rayResult = b3World_CastRayClosest( worldId, rayOrigin, rayTranslation, skipTeamFilter );

	// After gravity was applied, disable pogo when still moving up.
	// Avoids getting pulled back to the ground when jumping.
	bool suppressPogo = m_velocity.y > B3_FIX( 0.0f );

	if ( rayResult.hit == false || suppressPogo )
	{
		m_onGround = false;
		m_pogoVelocity = 0.0f;

		DrawLine( rayOrigin, b3OffsetPos( rayOrigin, rayTranslation ), MakeColor( b3_colorGray ) );
	}
	else
	{
		m_onGround = true;
		float pogoCurrentLength = b3FixToFloat( b3FixMul( rayResult.fraction, rayLength ) );

		float zeta = 0.7f;
		float hertz = 4.0f;
		float omega = 2.0f * b3FixToFloat( B3_PI ) * hertz;
		float omegaH = omega * timeStep;

		m_pogoVelocity = ( m_pogoVelocity - omega * omegaH * ( pogoCurrentLength - b3FixToFloat( pogoRestLength ) ) ) /
						 ( 1.0f + 2.0f * zeta * omegaH + omegaH * omegaH );
		DrawLine( rayOrigin, rayResult.point, MakeColor( b3_colorGreen ) );
	}

	b3Pos startPosition = m_transform.p;
	b3Pos target =
		m_transform.p + b3FixFromFloat( timeStep ) * m_velocity + b3FixFromFloat( timeStep * m_pogoVelocity ) * b3Vec3_axisY;

	// Want the mover to collide with allies
	b3QueryFilter moverFilter = { .categoryBits = 1, .maskBits = ~0u, .id = 1, .name = "mover_collide" };

	// The cast should ignore allies
	b3QueryFilter castFilter = { .categoryBits = 1, .maskBits = ~2u, .id = 1, .name = "mover_cast" };

	m_totalIterations = 0;
	b3Fixed tolerance = B3_FIX( 0.01f );

	for ( int iteration = 0; iteration < 5; ++iteration )
	{
		m_planeCount = 0;

		b3Capsule mover;
		mover.center1 = m_capsule.center1;
		mover.center2 = m_capsule.center2;
		mover.radius = m_capsule.radius;

		b3World_CollideMover( worldId, m_transform.p, &mover, moverFilter, PlaneResultFcn, this );

		b3Vec3 targetDelta = target - m_transform.p;
		b3PlaneSolverResult result = b3SolvePlanes( targetDelta, m_planes, m_planeCount );

		m_totalIterations += result.iterationCount;

		b3Vec3 delta = result.delta;

		b3Fixed fraction = b3World_CastMover( worldId, m_transform.p, &mover, delta, castFilter, MoverFilterCallback, this );

		delta *= fraction;
		m_transform.p = m_transform.p + delta;

		if ( b3LengthSquared( delta ) < b3FixMul( tolerance, tolerance ) )
		{
			break;
		}
	}

	for ( int i = 0; i < m_planeCount; ++i )
	{
		b3BodyId bodyId = b3Shape_GetBody( m_planeExtras[i].shapeId );
		b3BodyType bodyType = b3Body_GetType( bodyId );
		if ( bodyType != b3_dynamicBody )
		{
			continue;
		}

		b3Pos point = m_planeExtras[i].point;
		b3Vec3 normal = b3Neg( m_planes[i].plane.normal );

		b3Fixed invMassA = B3_FIX( 0.0f );
		b3Fixed invMassB = b3Body_GetInverseMass( bodyId );
		b3Matrix3 invIB = b3Body_GetWorldInverseRotationalInertia( bodyId );

		b3Pos pB = b3Body_GetWorldCenter( bodyId );
		b3Vec3 rB = b3SubPos( point, pB );

		b3Vec3 rnB = b3Cross( rB, normal );
		b3Fixed kNormal = invMassA + invMassB + b3Dot( rnB, b3MulMV( invIB, rnB ) );

		b3Vec3 vB = b3Body_GetLinearVelocity( bodyId );
		b3Vec3 omegaB = b3Body_GetAngularVelocity( bodyId );
		b3Vec3 vrB = b3Add( vB, b3Cross( omegaB, rB ) );
		b3Fixed vn = b3Dot( b3Sub( vrB, m_velocity ), normal );

		// Divide last: -vn / kNormal instead of a quantized reciprocal mass.
		b3Fixed impulse = kNormal > B3_FIX( 0.0f ) ? b3FixMax( b3FixDiv( -vn, kNormal ), B3_FIX( 0.0f ) ) : B3_FIX( 0.0f );

		b3Vec3 P = b3MulSV( impulse, normal );
		m_velocity = b3MulSub( m_velocity, invMassA, P );

		b3Body_ApplyLinearImpulse( bodyId, P, point, true );
	}

	if ( clipVelocity )
	{
		// Using the velocity clipper can avoid picking up velocity from depenetration.
		// This allows the mover to avoid velocity from soft collision depenetration.
		m_velocity = b3ClipVector( m_velocity, m_planes, m_planeCount );
	}
	else if ( timeStep > 0.0f )
	{
		// Using the position delta is more holistic and intuitive in some cases.
		m_velocity = b3FixFromFloat( 1.0f / timeStep ) * ( m_transform.p - startPosition );
	}
}

void CharacterMover::Step( b3ShapeId* ignoreShapes, int ignoreCount, bool clipVelocity )
{
	m_ignoreShapeIds = ignoreShapes;
	m_ignoreCount = ignoreCount;

	b3Vec2 throttle = { B3_FIX( 0.0f ), B3_FIX( 0.0f ) };
	b3Vec3 forward = -m_sample->m_camera->GetForward();
	b3Vec3 right = m_sample->m_camera->GetRight();

	forward.y = B3_FIX( 0.0f );
	forward = b3Normalize( forward );

	if ( m_sample->m_camera->m_thirdPerson )
	{
		if ( IsKeyDown( KEY_W ) )
		{
			throttle.x += B3_FIX( 1.0f );
		}

		if ( IsKeyDown( KEY_S ) )
		{
			throttle.x -= B3_FIX( 1.0f );
		}

		if ( IsKeyDown( KEY_A ) )
		{
			throttle.y -= B3_FIX( 1.0f );
		}

		if ( IsKeyDown( KEY_D ) )
		{
			throttle.y += B3_FIX( 1.0f );
		}

		if ( IsKeyDown( KEY_SPACE ) && m_onGround == true )
		{
			m_velocity.y = b3FixFromFloat( m_jumpSpeed );
			m_onGround = false;
		}

		if ( m_onGround == true )
		{
			m_sprint = IsKeyDown( KEY_LEFT_SHIFT );
		}
		else
		{
			m_sprint = false;
		}
	}

	float hertz = m_sample->m_context->hertz;
	float timeStep = hertz > 0.0f ? 1.0f / hertz : 0.0f;

	// throttle = { 0.0f, 0.0f, -1.0f };

	SolveMove( timeStep, forward, right, throttle, clipVelocity );

	b3Pos position = m_transform.p;

	// Follow the mover and latch the draw origin before drawing, so the overlays below demote
	// against the same eye the view renders from.
	if ( m_sample->m_camera->m_thirdPerson )
	{
		m_sample->m_camera->m_pivot = position;
		m_sample->m_camera->UpdateTransform();
	}

	SetDrawOrigin( m_sample->m_camera->DrawOrigin() );

	int count = m_planeCount;
	for ( int i = 0; i < count; ++i )
	{
		b3Plane plane = m_planes[i].plane;
		b3Pos p1 = position + ( plane.offset - m_capsule.radius ) * plane.normal;
		b3Pos p2 = p1 + B3_FIX( 0.1f ) * plane.normal;
		DrawPoint( p1, 5.0f, MakeColor( b3_colorYellow ) );
		DrawLine( p1, p2, MakeColor( b3_colorYellow ) );
	}

	DrawSolidCapsule( m_transform, m_capsule, MakeColor( b3_colorBlue ) );
	DrawLine( position, position + m_velocity, MakeColor( b3_colorPurple ) );

	m_ignoreShapeIds = nullptr;
	m_ignoreCount = 0;
}
