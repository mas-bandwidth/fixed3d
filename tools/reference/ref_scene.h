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

// Predicate form: `expect contactCount >= 1` / `expect restingY > -0.5` state
// absolute sanity bounds on the FINAL step of both traces (a landing card may
// state its observable this way instead of a per-quantity header). A scene with
// predicates is compared in predicate mode: every predicate must hold on BOTH
// traces, the final contact count must be equal, and the final resting height
// (lowest hull vertex, the `low` trace line) must agree within
// B3_REF_RESTING_Y_TOLERANCE (the stated tolerance of the scene that introduced
// the form; see README.md).
typedef enum b3RefPredicateKind
{
	b3_refPredContactCount = 0,
	b3_refPredRestingY = 1,
} b3RefPredicateKind;

enum
{
	B3_REF_OP_GE = 0,
	B3_REF_OP_GT = 1,
	B3_REF_OP_LE = 2,
	B3_REF_OP_LT = 3,
};

typedef struct b3RefPredicate
{
	int kind; // b3RefPredicateKind
	int op;	  // B3_REF_OP_*
	double value;
} b3RefPredicate;

#define B3_REF_MAX_PREDICATES 8
#define B3_REF_RESTING_Y_TOLERANCE 0.05

typedef struct b3RefExpect
{
	double posTolerance; // meters
	double rotTolerance; // radians
	double velTolerance; // m/s
	int horizon;		 // steps compared before chaos diverges
	int countsExact;	 // 1 => counters compare exactly, else allowed delta
	int countsDelta;	 // allowed integer delta when countsExact == 0
	bool traceContacts;	 // require complete per-step contact point sets
	bool hasExpect;		 // false until the expect line is parsed
	int predicateCount;	 // > 0 => predicate mode (see b3RefPredicate)
	b3RefPredicate predicates[B3_REF_MAX_PREDICATES];
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
	double safetyFactor; // b3BodyDef::safetyFactor, default 0.5
} b3RefBody;

typedef enum b3RefShapeKind
{
	b3_refBox = 0,
	b3_refSphere = 1,
	b3_refCapsule = 2,
	b3_refHull = 3,
	b3_refCompound = 4,
	b3_refMesh = 5,
} b3RefShapeKind;

#define B3_REF_MAX_MESH_VERTICES 64
#define B3_REF_MAX_MESH_TRIANGLES 128
#define B3_REF_MAX_MATERIALS 16
#define B3_REF_MAX_CHILDREN 128

typedef struct b3RefChild
{
	b3RefShapeKind kind;
	double position[3], rotation[4];
	double halfExtents[3], radius, halfLength;
	int hullCount;
	double hullPoints[B3_REF_MAX_HULL_POINTS][3];
	double density, friction, restitution;
} b3RefChild;

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
	int childStart, childCount;
	int vertexCount, triangleCount, materialCount;
	double vertices[B3_REF_MAX_MESH_VERTICES][3];
	int triangles[B3_REF_MAX_MESH_TRIANGLES][3];
	int materialIndices[B3_REF_MAX_MESH_TRIANGLES];
	double materials[B3_REF_MAX_MATERIALS][2]; // friction, restitution
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
	double anchorB[3];	   // spherical uses anchorA only; revolute/weld use both
	double axis[3];		   // revolute axis
	double length;		   // distance rest length
	bool collideConnected; // b3JointDef::collideConnected, default true
	int atStep;			   // world step index (0-based) before which the joint is created
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
// `world gravity x y z dt num den substeps n steps n [restitutionIterations n]`
// pins post-solve sweeps on both engines when the optional field is present.

typedef struct b3RefWorld
{
	double gravity[3];
	double dtNumerator;
	double dtDenominator;
	int subSteps;
	int steps;
	int restitutionIterations; // 0 leaves each engine's default unchanged
	bool hasWorld;
} b3RefWorld;

// ---- mid-run events and compound children -------------------------------------

// A per-step action. `at_step <n> set_mass <body> center <x> <y> <z>` moves a
// resting body's center of mass so its cached contact manifolds go stale
// (upstream #155). The event fires before the world step whose index is `step`.
typedef enum b3RefEventKind
{
	b3_refEventSetMass = 0,
} b3RefEventKind;

typedef struct b3RefEvent
{
	int step; // world step index (0-based) before which the action fires
	int kind; // b3RefEventKind
	int bodyIndex;
	double center[3];
} b3RefEvent;

#define B3_REF_MAX_EVENTS 16

// A sphere child of a compound shape, built by repeated `compound <body> sphere ...`
// directives. Only sphere children are expressible: that is all the 954cf87
// compound-overlap scene needs.
typedef struct b3RefCompoundChild
{
	double center[3];
	double radius;
} b3RefCompoundChild;

#define B3_REF_MAX_COMPOUND_CHILDREN 16

// ---- the parsed scene ----------------------------------------------------------

// The generator form (README.md "Generator-form scenes"): `scene <name>`,
// `hull_generator complex <seed> [radius <r>]`, `body hull|box ...` and
// `steps <n> dt <num>/<den> substeps <n>` build the same directive tables as
// the v1 form, with the hull points expanded by the parser from the documented
// LCG so both engines receive one shared point list.
typedef struct b3RefScene
{
	b3RefExpect expect;
	b3RefWorld world;
	int childCount;
	b3RefChild children[B3_REF_MAX_CHILDREN];
	int snapshotSaveStep, snapshotRestoreStep; // before step, -1 when absent
	bool traceContacts;
	int bodyCount;
	b3RefBody bodies[B3_REF_MAX_BODIES];
	int shapeCount;
	b3RefShape shapes[B3_REF_MAX_SHAPES];
	int jointCount;
	b3RefJoint joints[B3_REF_MAX_JOINTS];
	int queryCount;
	b3RefQuery queries[B3_REF_MAX_QUERIES];
	int eventCount;
	b3RefEvent events[B3_REF_MAX_EVENTS];
	int compoundChildCount[B3_REF_MAX_BODIES];
	b3RefCompoundChild compoundChildren[B3_REF_MAX_BODIES][B3_REF_MAX_COMPOUND_CHILDREN];
	char name[64]; // `scene <name>` line, information only
	bool hasName;
	// pending `hull_generator complex <seed> [radius <r>]`, consumed by the next
	// `body hull ...` line of the generator form
	bool hasHullGenerator;
	uint32_t hullGeneratorSeed;
	double hullGeneratorRadius;
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
// Predicate-mode scenes (an `expect` predicate line) additionally carry, per
// step, one line per hull-shape body: the lowest transformed hull vertex,
//   low <i> <y>
// which is the resting-height observable. v1 scenes never emit it.
//
// All numbers are printed %.9g. Bodies are emitted in creation order, one line per
// body per step. Query result lines follow the final step.

// Trace comparison: read two traces (in-memory, NUL-terminated) against a tolerance
// header, print a per-quantity maximum-error table to `report` and the first step
// that breaks the tolerance, and return true (0) only when every quantity is inside
// tolerance. Integer counters compare exactly unless the header says otherwise.
// `report` may be NULL; `reportSize` is the capacity of `report`.
bool b3RefCompareTraces( const char* traceA, const char* traceB, const b3RefExpect* expect, char* report, int reportSize );

// Canonical contact rows, shared by both drivers and the comparer. Shape indices
// are (body creation ordinal * (MAX_SHAPES + 1)) + body-local shape ordinal.
#define B3_REF_MAX_CONTACTS 256
#define B3_REF_MAX_CONTACT_POINTS 4
typedef struct b3RefContactSample
{
	int shapeA, shapeB, pointCount;
	double points[B3_REF_MAX_CONTACT_POINTS][4]; // world midpoint x/y/z, separation
} b3RefContactSample;
void b3RefSortContacts( b3RefContactSample* contacts, int count );
