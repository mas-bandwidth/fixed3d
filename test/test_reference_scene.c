// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// Unit tier for the float box3d reference harness: exercises the scene parser and
// the trace comparer WITHOUT either engine. The parser reads an in-memory scene
// string; the comparer compares hand-written traces inside and outside tolerance.

#include "test_macros.h"
#include "ref_scene.h"

#include <string.h>

// The shared parser and comparer are engine-free C; compile them into this TU so
// the unit tier stays a single self-contained test (no separate library target).
#include "ref_scene.c"

static const char* kScene = "# a small static scene\n"
							"expect pos 0.01 rot 0.001 vel 0.01 horizon 120 counts exact\n"
							"world gravity 0 -10 0 dt 1 60 substeps 4 steps 2\n"
							"body static position 0 0 0 rotation 0 0 0 1 linvel 0 0 0 angvel 0 0 0\n"
							"body dynamic position 0 1 0 rotation 0 0 0 1 linvel 0 0 0 angvel 0 0 0\n"
							"shape 0 box 5 0.5 5 density 1 friction 0.5 restitution 0.1\n"
							"shape 1 sphere 0.5 density 1 friction 0.5 restitution 0.2 rolling 0.05\n"
							"shape 1 capsule 0.25 0.5 axis 0 1 0 density 1 friction 0.5 restitution 0.2\n"
							"shape 1 hull points 4 0.5 0 0.5 -0.5 0 0.5 -0.5 0 -0.5 0.5 0 -0.5 density 1 friction 0.5 restitution 0\n"
							"joint revolute 0 1 anchor 0 0.5 0 axis 0 0 1\n"
							"query raycast origin 0 5 0 translation 0 -10 0\n"
							"query overlap_aabb lower -1 -1 -1 upper 1 1 1\n"
							"query closest_points 0 1\n";

static int ParseDirectivesTest( void )
{
	b3RefScene scene;
	char error[256];
	ENSURE( b3RefParseScene( kScene, &scene, error, sizeof( error ) ) );

	// expect header
	ENSURE( scene.expect.hasExpect );
	ENSURE( scene.expect.posTolerance == 0.01 );
	ENSURE( scene.expect.rotTolerance == 0.001 );
	ENSURE( scene.expect.velTolerance == 0.01 );
	ENSURE( scene.expect.horizon == 120 );
	ENSURE( scene.expect.countsExact == 1 );

	// world
	ENSURE( scene.world.hasWorld );
	ENSURE( scene.world.gravity[1] == -10.0 );
	ENSURE( scene.world.dtNumerator == 1.0 );
	ENSURE( scene.world.dtDenominator == 60.0 );
	ENSURE( scene.world.subSteps == 4 );
	ENSURE( scene.world.steps == 2 );

	// bodies
	ENSURE( scene.bodyCount == 2 );
	ENSURE( scene.bodies[0].type == b3_refStatic );
	ENSURE( scene.bodies[1].type == b3_refDynamic );
	ENSURE( scene.bodies[1].position[1] == 1.0 );

	// shapes
	ENSURE( scene.shapeCount == 4 );
	ENSURE( scene.shapes[0].kind == b3_refBox );
	ENSURE( scene.shapes[0].halfExtents[0] == 5.0 );
	ENSURE( scene.shapes[1].kind == b3_refSphere );
	ENSURE( scene.shapes[1].radius == 0.5 );
	ENSURE( scene.shapes[1].rollingResistance == 0.05 );
	ENSURE( scene.shapes[2].kind == b3_refCapsule );
	ENSURE( scene.shapes[2].halfLength == 0.5 );
	ENSURE( scene.shapes[3].kind == b3_refHull );
	ENSURE( scene.shapes[3].hullCount == 4 );

	// joints
	ENSURE( scene.jointCount == 1 );
	ENSURE( scene.joints[0].kind == b3_refRevolute );
	ENSURE( scene.joints[0].bodyA == 0 );
	ENSURE( scene.joints[0].bodyB == 1 );

	// queries
	ENSURE( scene.queryCount == 3 );
	ENSURE( scene.queries[0].kind == b3_refRayCast );
	ENSURE( scene.queries[1].kind == b3_refOverlapAABB );
	ENSURE( scene.queries[2].kind == b3_refClosestPoints );
	ENSURE( scene.queries[2].bodyA == 0 );
	ENSURE( scene.queries[2].bodyB == 1 );

	// a malformed directive must fail
	const char* bad = "expect pos 0.01 rot 0.001 vel 0.01 horizon 120 counts exact\nbogus line\n";
	ENSURE( b3RefParseScene( bad, &scene, error, sizeof( error ) ) == false );

	return 0;
}

static int GoldenCompareInsideTest( void )
{
	b3RefScene scene;
	char error[256];
	ENSURE( b3RefParseScene( kScene, &scene, error, sizeof( error ) ) );

	const char* traceA = "step 0 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 1 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "step 1 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 0.95 0 q 0 0 0 1 v 0 -0.2 0 w 0 0 0\n"
						 "query 0 raycast 1 0.5 1 2 3 0 0 1\n"
						 "query 1 overlap_aabb 2\n"
						 "query 2 closest_points 0.45 0 0.95 0\n";

	const char* traceB = "step 0 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 1 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "step 1 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 0.9502 0 q 0 0 0 1 v 0 -0.2001 0 w 0 0 0\n"
						 "query 0 raycast 1 0.5001 1 2.0001 3 0 0 1\n"
						 "query 1 overlap_aabb 2\n"
						 "query 2 closest_points 0.4501 0 0.9502 0\n";

	char report[2048];
	ENSURE( b3RefCompareTraces( traceA, traceB, &scene.expect, report, sizeof( report ) ) == true );
	return 0;
}

static int GoldenCompareOutsideTest( void )
{
	b3RefScene scene;
	char error[256];
	ENSURE( b3RefParseScene( kScene, &scene, error, sizeof( error ) ) );

	const char* traceA = "step 0 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 1 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "step 1 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 0.95 0 q 0 0 0 1 v 0 -0.2 0 w 0 0 0\n"
						 "query 0 raycast 1 0.5 1 2 3 0 0 1\n"
						 "query 1 overlap_aabb 2\n"
						 "query 2 closest_points 0.45 0 0.95 0\n";

	const char* traceB = "step 0 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 1 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "step 1 counters 1 2 1\n"
						 "body 0 p 0 0 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						 "body 1 p 0 0.80 0 q 0 0 0 1 v 0 -0.2 0 w 0 0 0\n"
						 "query 0 raycast 1 0.5 1 2 3 0 0 1\n"
						 "query 1 overlap_aabb 2\n"
						 "query 2 closest_points 0.45 0 0.95 0\n";

	char report[2048];
	ENSURE( b3RefCompareTraces( traceA, traceB, &scene.expect, report, sizeof( report ) ) == false );
	return 0;
}

// The generator form (README.md "Generator-form scenes"): the grammar of the
// ref-9f998c8 scene the adec25b card landed for the glob.
static const char* kGeneratorScene = "# complex hull drop\n"
									 "scene ref-test-complex-hull\n"
									 "hull_generator complex 42\n"
									 "body hull position 0 5 0 density 1.0\n"
									 "body box half 10 0.5 10 position 0 -0.5 0 static\n"
									 "steps 120 dt 1/60 substeps 4\n"
									 "expect contactCount >= 1\n"
									 "expect restingY > -0.5\n";

static int GeneratorSceneTest( void )
{
	b3RefScene scene;
	char error[256];
	ENSURE( b3RefParseScene( kGeneratorScene, &scene, error, sizeof( error ) ) );

	// scene name and predicates
	ENSURE( scene.hasName );
	ENSURE( strcmp( scene.name, "ref-test-complex-hull" ) == 0 );
	ENSURE( scene.expect.predicateCount == 2 );
	ENSURE( scene.expect.predicates[0].kind == b3_refPredContactCount );
	ENSURE( scene.expect.predicates[0].op == B3_REF_OP_GE );
	ENSURE( scene.expect.predicates[0].value == 1.0 );
	ENSURE( scene.expect.predicates[1].kind == b3_refPredRestingY );
	ENSURE( scene.expect.predicates[1].op == B3_REF_OP_GT );
	ENSURE( scene.expect.predicates[1].value == -0.5 );

	// world defaults: dt as a fraction, default gravity, no world line
	ENSURE( scene.world.steps == 120 );
	ENSURE( scene.world.dtNumerator == 1.0 );
	ENSURE( scene.world.dtDenominator == 60.0 );
	ENSURE( scene.world.subSteps == 4 );
	ENSURE( scene.world.hasWorld );
	ENSURE( scene.world.gravity[1] == -10.0 );

	// bodies and shapes: hull body 0 (dynamic), ground box body 1 (static)
	ENSURE( scene.bodyCount == 2 );
	ENSURE( scene.shapeCount == 2 );
	ENSURE( scene.bodies[0].type == b3_refDynamic );
	ENSURE( scene.bodies[0].position[1] == 5.0 );
	ENSURE( scene.bodies[1].type == b3_refStatic );
	ENSURE( scene.bodies[1].position[1] == -0.5 );
	ENSURE( scene.shapes[0].kind == b3_refHull );
	ENSURE( scene.shapes[0].bodyIndex == 0 );
	ENSURE( scene.shapes[0].hullCount == 32 );
	ENSURE( scene.shapes[0].density == 1.0 );
	ENSURE( scene.shapes[0].friction == 0.5 );
	ENSURE( scene.shapes[1].kind == b3_refBox );
	ENSURE( scene.shapes[1].bodyIndex == 1 );
	ENSURE( scene.shapes[1].halfExtents[0] == 10.0 );

	// the generated points sit ON the radius-0.5 sphere (normalize + scale)
	for ( int i = 0; i < scene.shapes[0].hullCount; ++i )
	{
		double x = scene.shapes[0].hullPoints[i][0];
		double y = scene.shapes[0].hullPoints[i][1];
		double z = scene.shapes[0].hullPoints[i][2];
		double r = sqrt( x * x + y * y + z * z );
		ENSURE( fabs( r - 0.5 ) < 1e-12 );
		ENSURE( fabs( x ) <= 0.5 && fabs( y ) <= 0.5 && fabs( z ) <= 0.5 );
	}

	// the generator is deterministic: the same seed parses to the same points
	b3RefScene again;
	ENSURE( b3RefParseScene( kGeneratorScene, &again, error, sizeof( error ) ) );
	ENSURE( memcmp( again.shapes[0].hullPoints, scene.shapes[0].hullPoints,
					sizeof( scene.shapes[0].hullPoints ) ) == 0 );

	// a hull body without a pending generator is a parse error
	const char* bad = "expect contactCount >= 1\nbody hull position 0 5 0\n";
	ENSURE( b3RefParseScene( bad, &scene, error, sizeof( error ) ) == false );

	return 0;
}

static int PredicateCompareTest( void )
{
	b3RefScene scene;
	char error[256];
	ENSURE( b3RefParseScene( kGeneratorScene, &scene, error, sizeof( error ) ) );

	// two one-step traces that agree inside the stated tolerance
	const char* insideA = "step 0 counters 0 2 0\n"
						  "body 0 p 0 5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "body 1 p 0 -0.5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "low 0 4.5\n"
						  "step 1 counters 2 1 0\n"
						  "body 0 p 0 0.05 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "body 1 p 0 -0.5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "low 0 0.02\n";
	const char* insideB = "step 0 counters 0 2 0\n"
						  "body 0 p 0 5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "body 1 p 0 -0.5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "low 0 4.5\n"
						  "step 1 counters 2 1 0\n"
						  "body 0 p 0 0.05 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "body 1 p 0 -0.5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						  "low 0 0.03\n";

	char report[2048];
	ENSURE( b3RefCompareTraces( insideA, insideB, &scene.expect, report, sizeof( report ) ) == true );

	// resting heights 0.02 apart pass; 0.06 apart break the stated tolerance
	const char* outsideB = "step 0 counters 0 2 0\n"
						   "body 0 p 0 5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						   "body 1 p 0 -0.5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						   "low 0 4.5\n"
						   "step 1 counters 2 1 0\n"
						   "body 0 p 0 0.05 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						   "body 1 p 0 -0.5 0 q 0 0 0 1 v 0 0 0 w 0 0 0\n"
						   "low 0 0.08\n";
	ENSURE( b3RefCompareTraces( insideA, outsideB, &scene.expect, report, sizeof( report ) ) == false );

	// a predicate bound violated on ONE trace fails even when the two agree
	const char* sunkA = "step 0 counters 0 2 0\n"
						"low 0 4.5\n"
						"step 1 counters 2 1 0\n"
						"low 0 -0.7\n";
	const char* sunkB = "step 0 counters 0 2 0\n"
						"low 0 4.5\n"
						"step 1 counters 2 1 0\n"
						"low 0 -0.7\n";
	ENSURE( b3RefCompareTraces( sunkA, sunkB, &scene.expect, report, sizeof( report ) ) == false );

	// unequal final contact counts fail
	const char* countB = "step 0 counters 0 2 0\n"
						 "low 0 4.5\n"
						 "step 1 counters 3 1 0\n"
						 "low 0 0.02\n";
	ENSURE( b3RefCompareTraces( insideA, countB, &scene.expect, report, sizeof( report ) ) == false );

	return 0;
}

int ReferenceSceneParseTest( void )
{
	RUN_SUBTEST( ParseDirectivesTest );
	RUN_SUBTEST( GoldenCompareInsideTest );
	RUN_SUBTEST( GoldenCompareOutsideTest );
	RUN_SUBTEST( GeneratorSceneTest );
	RUN_SUBTEST( PredicateCompareTest );
	return 0;
}
