// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/types.h"

#include <stdbool.h>
#include <stdint.h>

// This allows benchmarks to be tested on the benchmark app and also visualized in the samples app

typedef struct b3Capacity b3Capacity;

#ifdef __cplusplus
extern "C"
{
#endif

// This is used by the sample app to draw a ground grid. Should only
// be used for box hulls.
b3ShapeId GetGroundShapeId( void );
void ResetGroundShapeId( void );

void CreateTrees25( b3WorldId worldId );
void CreateTrees50( b3WorldId worldId );
void CreateTrees100( b3WorldId worldId );
void DestroyTrees( void );
void CreateJointGrid( b3WorldId worldId );
void CreateJunkyard( b3WorldId worldId );
void GetJunkyardCapacity( b3Capacity* capacity );
void StepJunkyard( b3WorldId worldId, int stepCount );
void CreateLargePyramid( b3WorldId worldId );
void CreateWidePyramid( b3WorldId worldId );
void CreateManyPyramids( b3WorldId worldId );
void GetRainCapacity( b3Capacity* capacity );
void CreateRain( b3WorldId worldId );
void StepRain( b3WorldId worldId, int stepCount );
void DestroyRain( void );
void GetLargeWorldCapacity( b3Capacity* capacity );
void CreateLargeWorld( b3WorldId worldId );
void StepLargeWorld( b3WorldId worldId, int stepCount );
void GetWasherCapacity( b3Capacity* capacity );
void CreateWasher( b3WorldId worldId );
void CreateConvexPile( b3WorldId worldId );
void GetConvexPileCapacity( b3Capacity* capacity );

void GetRagdollPileCapacity( b3Capacity* capacity );
void CreateRagdollPile( b3WorldId worldId );
void DestroyRagdollPile( void );

void GetBodies20kCapacity( b3Capacity* capacity );
void CreateBodies20k( b3WorldId worldId );

// ---- perf-harness state hashing and body registry ----

typedef struct b3BodyDef b3BodyDef;

// FNV-1a/64 over the raw bytes of a signed value, little-endian. Identical to the
// HashValue helper in tools/benchmark/contact_state_hash.c.
uint64_t b3HashValue( uint64_t hash, int64_t value );
uint64_t b3HashVector( uint64_t hash, b3Vec3 v );
// Hash one body's position, rotation, linear and angular velocity (the full wide
// 128-bit position in the ludicrous build).
uint64_t b3HashBodyState( uint64_t hash, b3BodyId bodyId );

// A registry of every body a scene creates, so the benchmark app can hash the whole
// world in deterministic body-id order without a public body-iteration API. Reset it
// before building a scene; b3BenchmarkCreateBody records while it is active (it is a
// transparent b3CreateBody otherwise, so samples pay nothing).
void b3BenchmarkResetBodies( void );
b3BodyId b3BenchmarkCreateBody( b3WorldId worldId, const b3BodyDef* def );
uint64_t b3BenchmarkHashBodies( b3WorldId worldId );

// void CreateSpinner( b3WorldId worldId );
// b3Fixed StepSpinner( b3WorldId worldId, int stepCount );
// void CreateSmash( b3WorldId worldId );
// void CreateTumbler( b3WorldId worldId );

#ifdef __cplusplus
}
#endif
