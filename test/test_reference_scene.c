// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// Unit tier for the float box3d reference harness: exercises the scene parser and
// the trace comparer WITHOUT either engine. The parser reads an in-memory scene
// string; the comparer compares hand-written traces inside and outside tolerance.

#include "ref_scene.h"
#include "test_macros.h"

#include <string.h>

// The shared parser and comparer are engine-free C; compile them into this TU so
// the unit tier stays a single self-contained test (no separate library target).
#include "ref_scene.c"

static const char* kScene =
	"# a small static scene\n"
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
	ENSURE( memcmp( again.shapes[0].hullPoints, scene.shapes[0].hullPoints, sizeof( scene.shapes[0].hullPoints ) ) == 0 );

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

static int ReferenceSceneCompoundMeshSnapshotParseTest( void )
{
	const char* text =
		"expect pos 0.001 rot 0.001 vel 0.001 horizon 90 counts exact\n"
		"world gravity 0 -10 0 dt 1 60 substeps 4 steps 90\n"
		"body box half 1 1 1 position 0 -2 0 static\n"
		"body box half 0.1 0.1 0.1 position 0 3 0 dynamic\n"
		"shape 0 compound children 1 sphere 0.5 position 1 0 0 rotation 0 0 0 1 density 1 friction 0.5 restitution 0.2\n"
		"shape 0 mesh vertices 3 -5 0 -5 0 0 5 5 0 -5 triangles 1 0 1 2 materials 1 0.5 0.2 indices 0\n"
		"snapshot save at 30\n"
		"snapshot restore at 60\n"
		"trace contacts\n";
	b3RefScene scene;
	char error[256];
	bool ok = b3RefParseScene( text, &scene, error, sizeof( error ) );
	if ( !ok )
		printf( "ReferenceSceneCompoundMeshSnapshotParseTest: %s\n", error );
	ENSURE( ok );
	ENSURE( scene.shapeCount == 4 );
	ENSURE( scene.shapes[2].kind == b3_refCompound );
	ENSURE( scene.shapes[2].childCount == 1 );
	ENSURE( scene.children[0].kind == b3_refSphere );
	ENSURE( scene.children[0].position[0] == 1 );
	ENSURE( scene.children[0].rotation[3] == 1 );
	ENSURE( scene.children[0].density == 1 );
	ENSURE( scene.children[0].friction == 0.5 );
	ENSURE( scene.children[0].restitution == 0.2 );
	ENSURE( scene.shapes[3].kind == b3_refMesh );
	ENSURE( scene.shapes[3].vertexCount == 3 );
	ENSURE( scene.shapes[3].triangleCount == 1 );
	ENSURE( scene.shapes[3].triangles[0][2] == 2 );
	ENSURE( scene.shapes[3].materials[0][1] == 0.2 );
	ENSURE( scene.shapes[3].materialIndices[0] == 0 );
	ENSURE( scene.snapshotSaveStep == 30 && scene.snapshotRestoreStep == 60 );
	ENSURE( scene.traceContacts && scene.expect.traceContacts );
	return 0;
}

static int ReferenceSceneGeometryValidationTest( void )
{
	const char* prefix = "expect pos 0.001 rot 0.001 vel 0.001 horizon 90 counts exact\n"
						 "world gravity 0 0 0 dt 1 60 substeps 4 steps 90\n"
						 "body box half 1 1 1 position 0 0 0 static\n"
						 "body box half 1 1 1 position 0 3 0 dynamic\n";
	const char* valid =
		"shape 0 compound children 3 "
		"box 1 2 3 position 0 1 0 rotation 0 0 0 1 density 2 friction 0.3 restitution 0.1 "
		"capsule 0.2 0.4 position 2 0 0 rotation 0 0 0.7071 0.7071 density 2 friction 0.5 restitution 0.2 "
		"hull points 4 0 0 0 1 0 0 0 1 0 0 0 1 position -2 0 0 rotation 0 0 0 1 density 2 friction 0.7 restitution 0.3\n";
	char text[4096], error[256];
	b3RefScene scene;
	snprintf( text, sizeof( text ), "%s%s", prefix, valid );
	ENSURE( b3RefParseScene( text, &scene, error, sizeof( error ) ) );
	ENSURE( scene.children[0].kind == b3_refBox && scene.children[0].halfExtents[2] == 3 );
	ENSURE( scene.children[1].kind == b3_refCapsule && scene.children[1].halfLength == 0.4 );
	ENSURE( scene.children[2].kind == b3_refHull && scene.children[2].hullCount == 4 );
	const char* invalid[] = {
		"shape 1 compound children 1 sphere 1 position 0 0 0 rotation 0 0 0 1 density 1 friction 0 restitution 0\n",
		"shape 0 compound children 0\n",
		"shape 0 compound children 1.5\n",
		"shape 0 compound children 129\n",
		"shape 0 compound children 1 sphere nan position 0 0 0 rotation 0 0 0 1 density 1 friction 0 restitution 0\n",
		"shape 0 compound children 1 sphere 1 position 0 0 0 rotation 0 0 0 0 density 1 friction 0 restitution 0\n",
		"shape 0 mesh vertices 3 0 0 0 1 0 0 0 0 1 triangles 1 0 1 3\n",
		"shape 1 mesh vertices 3 0 0 0 1 0 0 0 0 1 triangles 1 0 1 2\n",
		"shape 0 mesh vertices 3 0 0 0 1 0 0 0 0 1 triangles 1 0 1 2 materials 1 0.5 0 indices 1\n",
		"snapshot save at 30\n",
		"snapshot save at 30\nsnapshot restore at 30\n",
		"snapshot save at 30\nsnapshot restore at 90\n",
		"snapshot save at 30\nsnapshot save at 31\nsnapshot restore at 60\n",
		"trace contacts extra\n",
		"shape 0 compound children 2 sphere 1 position 0 0 0 rotation 0 0 0 1 density 1 friction 0 restitution 0 sphere 1 "
		"position 2 0 0 rotation 0 0 0 1 density 2 friction 0 restitution 0\n",
	};
	for ( int i = 0; i < (int)( sizeof( invalid ) / sizeof( invalid[0] ) ); ++i )
	{
		snprintf( text, sizeof( text ), "%s%s", prefix, invalid[i] );
		ENSURE( !b3RefParseScene( text, &scene, error, sizeof( error ) ) );
	}
	ENSURE( strstr( error, "public baked-compound API" ) != NULL );
	return 0;
}

static int ReferenceContactPointSetCompareTest( void )
{
	b3RefExpect expect = { 0 };
	expect.posTolerance = 0.001;
	expect.rotTolerance = 0.001;
	expect.velTolerance = 0.001;
	expect.horizon = 1;
	expect.countsExact = 1;
	expect.traceContacts = true;
	const char* original = "step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 -1 0 0 -0.01 1 0 0 -0.01\n";
	const char* reordered = "step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 1.0001 0 0 -0.01 -1 0 0 -0.01\n";
	char report[2048];
	ENSURE( b3RefCompareTraces( original, reordered, &expect, report, sizeof( report ) ) );
	const char* tiedA = "step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 0.49998 0 0.5 0 0.50003 0 -0.5 0\n";
	const char* tiedB = "step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 0.5 0 -0.5 0 0.5 0 0.5 0\n";
	ENSURE( b3RefCompareTraces( tiedA, tiedB, &expect, report, sizeof( report ) ) );
	const char* duplicated = "step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 -1 0 0 -0.01 -1 0 0 -0.01\n";
	ENSURE( !b3RefCompareTraces( original, duplicated, &expect, report, sizeof( report ) ) );
	const char* bad[] = {
		"step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 1 -1 0 0 -0.01\n",			   // point count
		"step 0 counters 1 1 0\ncontacts 0\n",											   // manifold count
		"step 0 counters 1 1 0\ncontacts 1\ncontact 0 130 2 -1 0 0 -0.01 1 0 0 -0.01\n",   // pair
		"step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 -1 0 0 -0.01 1.1 0 0 -0.01\n", // position
		"step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 -1 0 0 -0.1 1 0 0 -0.01\n",	   // separation
		"step 0 counters 1 1 0\n",														   // missing opt-in trace
		"step 0 counters 1 1 0\ncontacts 1\n",											   // truncated
		"step 0 counters 1 1 0\ncontacts 1\ncontact 0 129 2 nan 0 0 0 1 0 0 0\n",
	};
	for ( int i = 0; i < (int)( sizeof( bad ) / sizeof( bad[0] ) ); ++i )
		ENSURE( !b3RefCompareTraces( original, bad[i], &expect, report, sizeof( report ) ) );
	ENSURE( !b3RefCompareTraces( bad[5], bad[5], &expect, report, sizeof( report ) ) );
	return 0;
}

int ReferenceSceneParseTest( void )
{
	RUN_SUBTEST( ReferenceSceneCompoundMeshSnapshotParseTest );
	RUN_SUBTEST( ReferenceSceneGeometryValidationTest );
	RUN_SUBTEST( ReferenceContactPointSetCompareTest );
	RUN_SUBTEST( ParseDirectivesTest );
	RUN_SUBTEST( GoldenCompareInsideTest );
	RUN_SUBTEST( GoldenCompareOutsideTest );
	RUN_SUBTEST( GeneratorSceneTest );
	RUN_SUBTEST( PredicateCompareTest );
	return 0;
}

// test/CMakeLists.txt compiles main.c with -Dmain=box3d_real_main. This TU is the
// process entry so the suite is registered without editing main.c (outside PATHS).
extern int box3d_real_main( int argc, char** argv );

int main( int argc, char** argv )
{
	const char* filter = NULL;
	if ( argc > 1 )
	{
		filter = argv[1];
	}

	if ( filter == NULL || strcmp( filter, "ReferenceSceneParseTest" ) == 0 )
	{
		int result = ReferenceSceneParseTest();
		if ( result == 1 )
		{
			printf( "test failed: ReferenceSceneParseTest\n" );
			return 1;
		}
		printf( "test passed: ReferenceSceneParseTest\n" );
	}

	return box3d_real_main( argc, argv );
}
