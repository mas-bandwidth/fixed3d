// Compare spherical-joint state across builds, substeps and worker counts.
#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <inttypes.h>
#include <stdio.h>

static uint64_t HashValue( uint64_t hash, int64_t value )
{
	uint64_t bits = (uint64_t)value;
	for ( int i = 0; i < 8; ++i )
	{
		hash = ( hash ^ ( bits & 255 ) ) * UINT64_C( 1099511628211 );
		bits >>= 8;
	}
	return hash;
}

static uint64_t HashVector( uint64_t hash, b3Vec3 v )
{
	return HashValue( HashValue( HashValue( hash, v.x ), v.y ), v.z );
}

int main( void )
{
	const int substeps[] = { 1, 2, 4, 8 };
	for ( int scale = 0; scale < 2; ++scale )
		for ( int workers = 1; workers <= 8; ++workers )
			for ( int stepping = 0; stepping < 5; ++stepping )
				for ( int mode = 0; mode < 5; ++mode )
				{
					b3Fixed size = b3FixFromInt( scale == 0 ? 1 : 250 );
					b3WorldDef worldDef = b3DefaultWorldDef();
					worldDef.workerCount = workers;
					worldDef.gravity = b3Vec3_zero;
					worldDef.enableSleep = mode == 3;
					b3WorldId world = b3CreateWorld( &worldDef );
					b3BodyId bodies[33];
					b3JointId joints[32];
					b3ShapeDef shape = b3DefaultShapeDef();
					shape.filter.maskBits = 0;
					shape.density = B3_FIXED_ONE;
					b3BoxHull cube = b3MakeCubeHull( size / 2 );
					for ( int i = 0; i < 33; ++i )
					{
						b3BodyDef body = b3DefaultBodyDef();
						body.type = i == 0 && mode != 4 ? b3_staticBody : b3_dynamicBody;
						b3Vec3 offset = { size * i, 0, 0 };
						if ( mode == 4 && i > 0 )
						{
							offset = (b3Vec3){ size * ( i % 3 - 1 ), size * ( i / 3 % 3 - 1 ), size * ( i / 9 % 3 - 1 ) };
						}
						body.position = b3ToPos( offset );
						body.angularVelocity = (b3Vec3){ B3_FIX( 0.01f ), i % 2 ? B3_FIX( 0.02f ) : 0, B3_FIX( -0.01f ) };
						body.linearVelocity.y = size / 100;
						body.rotation.s = i % 3 == 0 ? -B3_FIXED_ONE : B3_FIXED_ONE;
						bodies[i] = b3CreateBody( world, &body );
						b3CreateHullShape( bodies[i], &shape, &cube.base );
						if ( i == 0 )
						{
							continue;
						}
						b3SphericalJointDef joint = b3DefaultSphericalJointDef();
						// The dynamic star in mode 4 forces joints into the overflow color.
						joint.base.bodyIdA = bodies[mode == 4 ? 0 : i - 1];
						joint.base.bodyIdB = bodies[i];
						joint.base.localFrameA.p = mode == 4 ? offset : (b3Vec3){ size / 2, 0, 0 };
						joint.base.localFrameB.p.x = mode == 4 ? 0 : -size / 2;
						joint.enableSpring = mode == 1;
						joint.enableMotor = mode == 1;
						joint.hertz = B3_FIX( 2.0f );
						joint.maxMotorTorque = B3_FIX( 100.0f );
						joint.motorVelocity.y = B3_FIX( 0.01f );
						joint.enableConeLimit = mode == 2;
						joint.enableTwistLimit = mode == 2;
						joint.coneAngle = B3_FIX( 0.3f );
						joint.lowerTwistAngle = B3_FIX( -0.2f );
						joint.upperTwistAngle = B3_FIX( 0.2f );
						joints[i - 1] = b3CreateSphericalJoint( world, &joint );
					}
					for ( int frame = 0; frame < 180; ++frame )
					{
						if ( frame == 35 )
						{
							b3World_EnableWarmStarting( world, false );
						}
						if ( frame == 55 )
						{
							b3World_EnableWarmStarting( world, true );
						}
						if ( mode == 3 )
						{
							if ( frame == 30 || frame == 60 || frame == 90 )
							{
								for ( int i = 0; i < 32; ++i )
								{
									b3SphericalJoint_EnableSpring( joints[i], frame == 30 );
									b3SphericalJoint_EnableMotor( joints[i], frame == 90 );
									b3SphericalJoint_EnableConeLimit( joints[i], frame == 60 );
									b3SphericalJoint_EnableTwistLimit( joints[i], frame == 60 );
								}
							}
							if ( frame == 100 )
							{
								b3Body_Disable( bodies[16] );
							}
							if ( frame == 105 )
							{
								b3Body_Enable( bodies[16] );
							}
							if ( frame == 110 )
							{
								b3Body_SetAwake( bodies[8], false );
							}
							if ( frame == 115 )
							{
								b3Body_SetAwake( bodies[8], true );
							}
							if ( frame == 120 || frame == 140 )
							{
								b3MotionLocks locks = { 0 };
								locks.angularX = locks.angularY = locks.angularZ = frame == 120;
								for ( int i = 1; i < 33; ++i )
								{
									b3Body_SetMotionLocks( bodies[i], locks );
								}
							}
							if ( frame == 150 )
							{
								b3WorldTransform t = b3Body_GetTransform( bodies[12] );
								t.p.y += size / 10;
								b3Body_SetTransform( bodies[12], t.p, b3Quat_identity );
							}
						}
						int count = substeps[stepping == 4 ? frame % 4 : stepping];
						b3Fixed dt = frame == 25 ? 0 : b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) );
						b3World_Step( world, dt, count );
						if ( mode == 4 && frame == 0 && b3World_GetCounters( world ).colorCounts[B3_GRAPH_COLOR_COUNT - 1] == 0 )
						{
							fprintf( stderr, "The star scenario did not exercise the overflow solver.\n" );
							b3DestroyWorld( world );
							return 1;
						}
						uint64_t hash = UINT64_C( 14695981039346656037 );
						for ( int i = 0; i < 33; ++i )
						{
							b3WorldTransform t = b3Body_GetTransform( bodies[i] );
							hash = HashValue( HashValue( HashValue( hash, t.p.x ), t.p.y ), t.p.z );
#if defined( BOX3D_LUDICROUS_MODE )
							hash = HashValue( hash, (int64_t)( (b3UInt128)t.p.x >> 64 ) );
							hash = HashValue( hash, (int64_t)( (b3UInt128)t.p.y >> 64 ) );
							hash = HashValue( hash, (int64_t)( (b3UInt128)t.p.z >> 64 ) );
#endif
							hash = HashVector( hash, t.q.v );
							hash = HashValue( hash, t.q.s );
							hash = HashVector( hash, b3Body_GetLinearVelocity( bodies[i] ) );
							hash = HashVector( hash, b3Body_GetAngularVelocity( bodies[i] ) );
							hash = HashValue( hash, b3Body_IsAwake( bodies[i] ) );
						}
						for ( int i = 0; i < 32; ++i )
						{
							hash = HashVector( hash, b3Joint_GetConstraintForce( joints[i] ) );
							hash = HashVector( hash, b3Joint_GetConstraintTorque( joints[i] ) );
						}
						printf( "%d %d %d %d %d %016" PRIx64 "\n", scale, workers, stepping, mode, frame, hash );
					}
					b3DestroyWorld( world );
				}
	return 0;
}
