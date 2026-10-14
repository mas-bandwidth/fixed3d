// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// The float box3d reference harness: shared, engine-agnostic scene parser and
// trace comparer. This file is deliberately free of any box3d header so that
// one source builds against BOTH engines: run_fixed.c links the in-tree fixed3d
// and run_float.c links vanilla box3d at a pinned commit. Numbers cross the
// engine boundary as double; the drivers own the b3FixFromDouble / double
// conversions (the sanctioned boundaries).
//
// Scene files are plain text under tools/reference/scenes/*.scene, one directive
// per line, decimal numbers only. A header line
//     expect pos <m> rot <rad> vel <m/s> horizon <steps> counts exact|<n>
// states the tolerance and how many steps are compared. The parser stores every
// number as double; the tolerance header decides how the comparer treats them.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define B3_REF_MAX_BODIES 64
#define B3_REF_MAX_SHAPES 128
#define B3_REF_MAX_JOINTS 64
#define B3_REF_MAX_QUERIES 64
#define B3_REF_MAX_HULL_POINTS 128

// ---- expected-result header ---------------------------------------------------

typedef struct b3RefExpect
{
	double posTolerance; // meters
	double rotTolerance; // radians
	double velTolerance; // m/s
	int horizon;		 // steps compared before chaos diverges
	int countsExact;	 // 1 => counters compare exactly, else allowed delta
	int countsDelta;	 // allowed integer delta when countsExact == 0
	bool hasExpect;		 // false until the expect line is parsed
} b3RefExpect;

// ---- directive vocabulary -----------------------------------------------------

typedef enum b3RefBodyType
{
	b3_refStatic = 0,
	b3_refKinematic = 1,
	b3_refDynamic = 2,
} b3RefBodyType;

typedef struct b3RefBody
{
	b3RefBodyType type;
	double position[3];
	double rotation[4]; // x y z w quaternion
	double linearVelocity[3];
	double angularVelocity[3];
	bool isBullet;
	bool allowFastRotation;
	bool enableSleep;
} b3RefBody;

typedef enum b3RefShapeKind
{
	b3_refBox = 0,
	b3_refSphere = 1,
	b3_refCapsule = 2,
	b3_refHull = 3,
} b3RefShapeKind;

typedef struct b3RefShape
{
	int bodyIndex;
	b3RefShapeKind kind;
	double halfExtents[3]; // box
	double radius;		   // sphere / capsule
	double center[3];	   // sphere center
	double capsuleAxis[3]; // capsule direction (normalized by driver)
	double halfLength;	   // capsule half length
	int hullCount;		   // hull point count
	double hullPoints[B3_REF_MAX_HULL_POINTS][3];
	double density;
	double friction;
	double restitution;
	double rollingResistance;
	bool isSensor;
} b3RefShape;

typedef enum b3RefJointKind
{
	b3_refRevolute = 0,
	b3_refSpherical = 1,
	b3_refDistance = 2,
	b3_refWeld = 3,
} b3RefJointKind;

typedef struct b3RefJoint
{
	b3RefJointKind kind;
	int bodyA;
	int bodyB;
	double anchorA[3];
	double anchorB[3]; // spherical uses anchorA only; revolute/weld use both
	double axis[3];	   // revolute axis
	double length;	   // distance rest length
} b3RefJoint;

typedef enum b3RefQueryKind
{
	b3_refRayCast = 0,
	b3_refOverlapAABB = 1,
	b3_refOverlapShape = 2,
	b3_refShapeCast = 3,
	b3_refClosestPoints = 4,
	b3_refMover = 5,
} b3RefQueryKind;

// overlap_shape / shape_cast carry a convex proxy: a box or a sphere or a capsule.
typedef struct b3RefQuery
{
	b3RefQueryKind kind;
	int bodyA; // closest_points pair
	int bodyB;
	double origin[3];
	double translation[3];
	double lower[3]; // overlap_aabb
	double upper[3];
	b3RefShapeKind proxyKind;
	double proxyHalfExtents[3];
	double proxyRadius;
	double proxyHalfLength;
	double proxyAxis[3];
} b3RefQuery;

// ---- the world directive -------------------------------------------------------

typedef struct b3RefWorld
{
	double gravity[3];
	double dtNumerator;
	double dtDenominator;
	int subSteps;
	int steps;
	bool hasWorld;
} b3RefWorld;

// ---- the parsed scene ----------------------------------------------------------

typedef struct b3RefScene
{
	b3RefExpect expect;
	b3RefWorld world;
	int bodyCount;
	b3RefBody bodies[B3_REF_MAX_BODIES];
	int shapeCount;
	b3RefShape shapes[B3_REF_MAX_SHAPES];
	int jointCount;
	b3RefJoint joints[B3_REF_MAX_JOINTS];
	int queryCount;
	b3RefQuery queries[B3_REF_MAX_QUERIES];
} b3RefScene;

// ---- parser --------------------------------------------------------------------

// Parse a scene from an in-memory NUL-terminated string. The parser performs no
// allocation: it writes into the caller-provided scene struct and returns false
// with a message on the first error. `error` and `errorSize` may be NULL.
bool b3RefParseScene( const char* text, b3RefScene* scene, char* error, int errorSize );

// Load and parse a scene file. Returns false (with a message) on any error.
bool b3RefParseSceneFile( const char* path, b3RefScene* scene, char* error, int errorSize );

// ---- trace format --------------------------------------------------------------

// Both drivers write the SAME trace format. The comparer and the golden unit test
// depend on it, so do not change the spelling without changing all three.
//
//   step <n> counters <contactCount> <awakeBodyCount> <jointCount>
//   body <i> p <x> <y> <z> q <x> <y> <z> <w> v <x> <y> <z> w <x> <y> <z>
//   ...
//   query <i> <kind> <payload...>
//
// All numbers are printed %.9g. Bodies are emitted in creation order, one line per
// body per step. Query result lines follow the final step.

// Trace comparison: read two traces (in-memory, NUL-terminated) against a tolerance
// header, print a per-quantity maximum-error table to `report` and the first step
// that breaks the tolerance, and return true (0) only when every quantity is inside
// tolerance. Integer counters compare exactly unless the header says otherwise.
// `report` may be NULL; `reportSize` is the capacity of `report`.
bool b3RefCompareTraces( const char* traceA, const char* traceB, const b3RefExpect* expect, char* report, int reportSize );
