// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_internal.h"
#include "solver.h"

typedef struct b3ManifoldConstraintPoint
{
	b3Vec3 rA, rB;
	b3Fixed baseSeparation;
	b3Fixed relativeVelocity;
	b3Fixed normalImpulse;
	b3Fixed totalNormalImpulse;
	b3Fixed normalMass;
	b3Fixed leverArm;
} b3ManifoldConstraintPoint;

typedef struct b3ManifoldConstraint
{
	b3ManifoldConstraintPoint points[4];
	int pointCount;
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;
	// Friction centers
	b3Vec3 centerA, centerB;
	b3Fixed twistMass;
	b3Fixed twistImpulse;
	b3Matrix2 tangentMass;
	b3Vec2 frictionImpulse;
	b3Vec3 rollingImpulse;
	b3Fixed tangentVelocity1;
	b3Fixed tangentVelocity2;
} b3ManifoldConstraint;

typedef struct b3ContactConstraint
{
	b3ManifoldConstraint* constraints;
	struct b3Contact* contact;
	int indexA;
	int indexB;
	b3Fixed invMassA, invMassB;
	b3Matrix3 invIA, invIB;
	b3Softness softness;
	b3Matrix3 rollingMass;
	b3Fixed friction;
	b3Fixed restitution;
	b3Fixed rollingResistance;
	int manifoldCount;
} b3ContactConstraint;

int b3GetWideContactConstraintByteCount( void );
int b3GetMeshWideContactConstraintByteCount( void );
int b3GetMeshWideManifoldConstraintByteCount( void );

// Overflow contacts don't fit into the constraint graph coloring
void b3PrepareContacts_Overflow( b3StepContext* context );
void b3WarmStartContacts_Overflow( b3StepContext* context );
void b3SolveContacts_Overflow( b3StepContext* context, bool useBias );
void b3ApplyRestitution_Overflow( b3StepContext* context );
void b3StoreImpulses_Overflow( b3StepContext* context );

// Scalar mesh solver. Used for the overflow color; the colored mesh path runs
// the wide variants below, which are bit-identical per lane by construction.
void b3PrepareContacts_Mesh( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_Mesh( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_Mesh( b3SolverBlock block, b3StepContext* context, bool useBias );
void b3ApplyRestitution_Mesh( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_Mesh( b3SolverBlock block, b3StepContext* context, int workerIndex );

// Four-lane wide mesh solver: lane = whole contact, manifolds serialize
// in-register (Gauss-Seidel across manifolds, like the scalar loop).
void b3PrepareContacts_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_MeshWide( b3SolverBlock block, b3StepContext* context, bool useBias );
void b3ApplyRestitution_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_MeshWide( b3SolverBlock block, b3StepContext* context, int workerIndex );

void b3PrepareContacts_Convex( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_Convex( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_Convex( b3SolverBlock block, b3StepContext* context, bool useBias );
void b3ApplyRestitution_Convex( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_Convex( b3SolverBlock block, b3StepContext* context, int workerIndex );
