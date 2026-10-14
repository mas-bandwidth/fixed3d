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

int ReferenceSceneParseTest( void )
{
	RUN_SUBTEST( ParseDirectivesTest );
	RUN_SUBTEST( GoldenCompareInsideTest );
	RUN_SUBTEST( GoldenCompareOutsideTest );
	return 0;
}
