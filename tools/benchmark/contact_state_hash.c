// Compare complete body-state hashes every frame against a preserved library.
// Exercises cache refresh with fixed/varying substeps, worker counts, warm-start
// toggles, rotation and both ordinary and asteroid-scale contact geometry.
#include "box3d/box3d.h"
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
    for ( int scaleIndex = 0; scaleIndex < 2; ++scaleIndex )
    for ( int workerIndex = 0; workerIndex < 2; ++workerIndex )
    for ( int mode = 0; mode < 5; ++mode )
    {
        b3Fixed size = b3FixFromInt( scaleIndex == 0 ? 1 : 250 );
        b3WorldDef worldDef = b3DefaultWorldDef();
        worldDef.workerCount = workerIndex == 0 ? 1 : 4;
        worldDef.enableSleep = false;
        b3WorldId world = b3CreateWorld( &worldDef );
        b3BodyDef groundDef = b3DefaultBodyDef();
        groundDef.position.y = -size;
        b3BodyId ground = b3CreateBody( world, &groundDef );
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3BoxHull floor = b3MakeBoxHull( size * 10, size, size * 10 );
        b3CreateHullShape( ground, &shapeDef, &floor.base );
        b3BoxHull cube = b3MakeCubeHull( size / 2 );
        b3BodyId bodies[36];
        for ( int i = 0; i < 36; ++i )
        {
            b3BodyDef bodyDef = b3DefaultBodyDef();
            bodyDef.type = b3_dynamicBody;
            bodyDef.position = (b3Pos){ ( i % 3 - 1 ) * size, size / 2 + ( i / 9 ) * size,
                                                       ( i / 3 % 3 - 1 ) * size };
            bodyDef.angularVelocity = (b3Vec3){ ( i % 3 - 1 ) * B3_FIX( 0.03f ),
                                                        B3_FIX( 0.01f ), B3_FIX( -0.02f ) };
            bodies[i] = b3CreateBody( world, &bodyDef );
            b3CreateHullShape( bodies[i], &shapeDef, &cube.base );
        }
        for ( int frame = 0; frame < 240; ++frame )
        {
            if ( frame == 70 ) { b3World_EnableWarmStarting( world, false ); }
            if ( frame == 130 ) { b3World_EnableWarmStarting( world, true ); }
            if ( frame == 100 )
            {
                b3Body_SetLinearVelocity( bodies[28], (b3Vec3){ size / 5, size / 10, -size / 7 } );
            }
            int count = substeps[mode == 4 ? frame % 4 : mode];
            b3World_Step( world, b3FixDiv( B3_FIXED_ONE, b3FixFromInt( 60 ) ), count );
            uint64_t hash = UINT64_C( 14695981039346656037 );
            for ( int i = 0; i < 36; ++i )
            {
                b3WorldTransform t = b3Body_GetTransform( bodies[i] );
                hash = HashValue( HashValue( HashValue( hash, t.p.x ), t.p.y ), t.p.z );
                hash = HashVector( hash, t.q.v );
                hash = HashValue( hash, t.q.s );
                hash = HashVector( hash, b3Body_GetLinearVelocity( bodies[i] ) );
                hash = HashVector( hash, b3Body_GetAngularVelocity( bodies[i] ) );
            }
            printf( "%d %d %d %d %016" PRIx64 "\n", scaleIndex, workerIndex, mode, frame, hash );
        }
        b3DestroyWorld( world );
    }
    return 0;
}
