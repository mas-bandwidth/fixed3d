// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box3d/id.h"

#include <stdbool.h>

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

// 200 ragdolls from the determinism corpus (CreateHuman), dropped to settle.
void GetRagdollPileCapacity( b3Capacity* capacity );
void CreateRagdollPile( b3WorldId worldId );
void DestroyRagdollPile( void );

// 20,000 mixed bodies (sphere, capsule, box) settling. The pyramid scenes are
// the pyramid case; this is the many-bodies case.
void GetBodies20kCapacity( b3Capacity* capacity );
void CreateBodies20k( b3WorldId worldId );

// void CreateSpinner( b3WorldId worldId );
// b3Fixed StepSpinner( b3WorldId worldId, int stepCount );
// void CreateSmash( b3WorldId worldId );
// void CreateTumbler( b3WorldId worldId );

#ifdef __cplusplus
}
#endif
