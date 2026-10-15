// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_internal.h"
#include "solver.h"

#include "box3d/constants.h"

typedef struct b3ManifoldConstraintPoint
{
	b3Vec3 rA, rB;
	b3Fixed baseSeparation;
	b3Fixed relativeVelocity;
	b3Fixed normalImpulse;
	b3Fixed totalNormalImpulse;
	b3Fixed restitutionImpulse;
	b3Fixed normalMass;
	b3Fixed leverArm;
} b3ManifoldConstraintPoint;

typedef struct b3ManifoldConstraint
{
	b3ManifoldConstraintPoint points[B3_MAX_MANIFOLD_POINTS];
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

// The wide constraint byte counts and the convex / wide mesh kernels are per
// ISA variant and reached through the world's kernel table (src/isa.h).

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

// The wide mesh path pays off where four int64 lanes have hardware (AVX-512).
// On scalar/NEON builds the emulated lanes plus the ragged manifold dimension
// measured 32-46% slower than the scalar mesh solver on M3 (trees scenes,
// min-of-3 interleaved A/B), so the scalar variant keeps the scalar colored
// path: b3IsaKernels.meshWide is true only for variants with lane hardware.
// Both paths are bit-identical per lane, so this cannot affect determinism.
// The four-lane wide mesh kernels (lane = whole contact, manifolds serialize
// in-register, Gauss-Seidel across manifolds like the scalar loop) and the
// convex kernels are declared per variant in src/isa.h.
