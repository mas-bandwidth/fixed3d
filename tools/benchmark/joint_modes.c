// Targeted motor/weld benchmark and full-state comparator.
#include "box3d/box3d.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

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

int main( int argc, char** argv )
{
    // Modes: velocity motor, spring motor, rigid weld, soft weld, locked weld,
    // linear motor, disabled motor, toggled motor, toggled weld.
    int mode = argc > 1 ? atoi( argv[1] ) : 0;
    int workers = argc > 2 ? atoi( argv[2] ) : 4;
    int count = argc > 3 ? atoi( argv[3] ) : 512;
    int frames = argc > 4 ? atoi( argv[4] ) : 240;
    int scale = argc > 5 ? atoi( argv[5] ) : 1;
    if ( mode < 0 || mode > 8 || workers < 1 || workers > 16 || count < 1 || count > 4096 ||
         frames < 1 || frames > 10000 || ( scale != 1 && scale != 250 ) )
    {
        fprintf( stderr, "usage: joint_modes mode[0-8] workers[1-16] pairs[1-4096] frames[1-10000] size[1|250]\n" );
        return 2;
    }
    b3Fixed size = b3FixFromInt( scale );
    b3BodyId* bodies = malloc( 2 * (size_t)count * sizeof( b3BodyId ) );
    b3JointId* joints = malloc( (size_t)count * sizeof( b3JointId ) );
    if ( bodies == NULL || joints == NULL ) { free( bodies ); free( joints ); return 1; }
    b3WorldDef worldDef = b3DefaultWorldDef();
    worldDef.workerCount = workers;
    worldDef.enableSleep = false;
    worldDef.gravity = b3Vec3_zero;
    b3WorldId world = b3CreateWorld( &worldDef );
    b3ShapeDef shape = b3DefaultShapeDef();
    shape.filter.maskBits = 0;
    b3BoxHull cube = b3MakeCubeHull( size / 2 );
    bool motor = mode == 0 || mode == 1 || mode == 5 || mode == 6 || mode == 7;
    for ( int i = 0; i < count; ++i )
    {
        for ( int side = 0; side < 2; ++side )
        {
            b3BodyDef body = b3DefaultBodyDef();
            body.type = side == 0 && i % 3 == 0 ? b3_staticBody : b3_dynamicBody;
            body.rotation.s = side == 1 && i % 2 == 0 ? -B3_FIXED_ONE : B3_FIXED_ONE;
            body.position = (b3Pos){ ( i % 32 * 4 + side ) * size, ( i / 32 * 4 ) * size, 0 };
            body.linearVelocity = (b3Vec3){ 0, side == 1 ? size / 100 : 0, 0 };
            body.angularVelocity = (b3Vec3){ B3_FIX( 0.01f ), side == 1 ? B3_FIX( 0.03f ) : 0, 0 };
            if ( mode == 4 ) { body.motionLocks.angularX = body.motionLocks.angularY = body.motionLocks.angularZ = true; }
            bodies[2 * i + side] = b3CreateBody( world, &body );
            b3CreateHullShape( bodies[2 * i + side], &shape, &cube.base );
        }
        if ( motor )
        {
            b3MotorJointDef joint = b3DefaultMotorJointDef();
            joint.base.bodyIdA = bodies[2 * i];
            joint.base.bodyIdB = bodies[2 * i + 1];
            joint.base.localFrameA.p.x = size / 2;
            joint.base.localFrameB.p.x = -size / 2;
            joint.maxVelocityForce = mode == 0 || mode == 6 || mode == 7 ? 0 : B3_FIX( 100.0f );
            joint.maxVelocityTorque = mode == 5 || mode == 6 || mode == 7 ? 0 : B3_FIX( 100.0f );
            joint.angularVelocity.y = B3_FIX( 0.1f );
            joint.maxSpringForce = mode == 1 ? B3_FIX( 100.0f ) : 0;
            joint.maxSpringTorque = mode == 1 || mode == 7 ? B3_FIX( 100.0f ) : 0;
            joint.linearHertz = B3_FIX( 2.0f );
            joint.angularHertz = mode == 1 ? B3_FIX( 2.0f ) : 0;
            joints[i] = b3CreateMotorJoint( world, &joint );
        }
        else
        {
            b3WeldJointDef joint = b3DefaultWeldJointDef();
            joint.base.bodyIdA = bodies[2 * i];
            joint.base.bodyIdB = bodies[2 * i + 1];
            joint.base.localFrameA.p.x = size / 2;
            joint.base.localFrameB.p.x = -size / 2;
            joint.angularHertz = mode == 3 ? B3_FIX( 2.0f ) : 0;
            joints[i] = b3CreateWeldJoint( world, &joint );
        }
    }
    b3Fixed elapsed = 0;
    for ( int frame = 0; frame < frames; ++frame )
    {
        if ( mode == 7 || mode == 8 )
        {
            if ( frame == 60 || frame == 120 || frame == 180 )
            {
                b3Fixed hertz = frame == 120 ? 0 : B3_FIX( 2.0f );
                for ( int i = 0; i < count; ++i )
                {
                    if ( motor )
                    {
                        b3MotorJoint_SetAngularHertz( joints[i], hertz );
                        b3MotorJoint_SetMaxVelocityTorque( joints[i], frame == 180 ? B3_FIX( 100.0f ) : 0 );
                        b3MotorJoint_SetMaxVelocityForce( joints[i], frame == 120 ? 0 : B3_FIX( 100.0f ) );
                        b3MotorJoint_SetMaxSpringForce( joints[i], frame == 120 ? 0 : B3_FIX( 100.0f ) );
                    }
                    else { b3WeldJoint_SetAngularHertz( joints[i], hertz ); }
                }
            }
        }
        if ( frame == 70 ) { b3World_EnableWarmStarting( world, false ); }
        if ( frame == 130 ) { b3World_EnableWarmStarting( world, true ); }
        if ( mode == 8 && ( frame == 90 || frame == 150 ) )
        {
            b3MotionLocks locks = { 0 };
            locks.angularX = locks.angularY = locks.angularZ = frame == 90;
            for ( int i = 0; i < 2 * count; ++i ) { b3Body_SetMotionLocks( bodies[i], locks ); }
        }
        uint64_t start = b3GetTicks();
        b3World_Step( world, b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) ), 4 );
        elapsed += b3GetMilliseconds( start );
        uint64_t hash = UINT64_C( 14695981039346656037 );
        for ( int i = 0; i < 2 * count; ++i )
        {
            b3WorldTransform t = b3Body_GetTransform( bodies[i] );
            hash = HashValue( HashValue( HashValue( hash, t.p.x ), t.p.y ), t.p.z );
            hash = HashVector( hash, t.q.v );
            hash = HashValue( hash, t.q.s );
            hash = HashVector( hash, b3Body_GetLinearVelocity( bodies[i] ) );
            hash = HashVector( hash, b3Body_GetAngularVelocity( bodies[i] ) );
        }
        for ( int i = 0; i < count; ++i )
        {
            hash = HashVector( hash, b3Joint_GetConstraintForce( joints[i] ) );
            hash = HashVector( hash, b3Joint_GetConstraintTorque( joints[i] ) );
        }
        printf( "%d %d %016" PRIx64 "\n", mode, frame, hash );
    }
    fprintf( stderr, "milliseconds %.6f\n", b3FixToDouble( elapsed ) );
    b3DestroyWorld( world );
    free( joints );
    free( bodies );
    return 0;
}
