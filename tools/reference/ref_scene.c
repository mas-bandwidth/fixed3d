// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// Shared scene parser and trace comparer for the float box3d reference harness.
// See ref_scene.h for the format contract. No allocation, no globals, engine-free.

#include "ref_scene.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// token helpers
// ---------------------------------------------------------------------------

static char* skipWs( char* p )
{
	while ( *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' )
	{
		++p;
	}
	return p;
}

// Advance p past a word; returns false if there is none.
static bool takeWord( char** p, const char* word )
{
	char* s = skipWs( *p );
	size_t n = strlen( word );
	if ( strncmp( s, word, n ) != 0 )
	{
		return false;
	}
	// require a word boundary
	char c = s[n];
	if ( c != '\0' && c != ' ' && c != '\t' && c != '\r' && c != '\n' && c != '#' )
	{
		return false;
	}
	*p = s + n;
	return true;
}

// Read a decimal number (strtod semantics, "decimal numbers only").
static bool takeDouble( char** p, double* out )
{
	char* s = skipWs( *p );
	if ( *s == '\0' || *s == '#' || *s == '\n' )
	{
		return false;
	}
	char* end = NULL;
	double v = strtod( s, &end );
	if ( end == s )
	{
		return false;
	}
	*out = v;
	*p = end;
	return true;
}

static bool takeInt( char** p, int* out )
{
	double d;
	if ( !takeDouble( p, &d ) )
	{
		return false;
	}
	*out = (int)d;
	return true;
}

// Require end-of-line (only whitespace / comment remains).
static bool endOfLine( char* p )
{
	p = skipWs( p );
	return *p == '\0' || *p == '\n' || *p == '#';
}

// ---------------------------------------------------------------------------
// expect header
// ---------------------------------------------------------------------------

// `expect pos ...` is the v1 header; `expect <contactCount|restingY> <op> <value>`
// (one line per predicate) is the predicate form. A scene uses one form.
static bool parseExpect( char* p, b3RefScene* scene, char* error, int errorSize )
{
	b3RefExpect* e = &scene->expect;
	char* apex = p;
	if ( takeWord( &apex, "firstBounceApex" ) )
	{
		memset( e, 0, sizeof( *e ) );
		if ( !takeWord( &apex, "body" ) || !takeInt( &apex, &e->apexBody ) || e->apexBody < 0 ||
			 e->apexBody >= B3_REF_MAX_BODIES || !takeWord( &apex, "tolerance" ) ||
			 !takeDouble( &apex, &e->apexTolerance ) || !isfinite( e->apexTolerance ) || e->apexTolerance <= 0.0 ||
			 !takeWord( &apex, "finalAwake" ) || !takeWord( &apex, "exact" ) || !endOfLine( apex ) )
		{
			if ( error && errorSize > 0 )
				snprintf( error, errorSize, "malformed firstBounceApex expectation" );
			return false;
		}
		e->firstBounceApex = true;
		e->hasExpect = true;
		return true;
	}

	// predicate form first: the header form is anchored on the word "pos"
	{
		char* q = p;
		int kind = -1;
		if ( takeWord( &q, "contactCount" ) )
		{
			kind = b3_refPredContactCount;
		}
		else if ( takeWord( &q, "restingY" ) )
		{
			kind = b3_refPredRestingY;
		}
		if ( kind >= 0 )
		{
			int op = -1;
			char* s = skipWs( q );
			if ( s[0] == '>' && s[1] == '=' )
			{
				op = B3_REF_OP_GE;
				q = s + 2;
			}
			else if ( s[0] == '>' )
			{
				op = B3_REF_OP_GT;
				q = s + 1;
			}
			else if ( s[0] == '<' && s[1] == '=' )
			{
				op = B3_REF_OP_LE;
				q = s + 2;
			}
			else if ( s[0] == '<' )
			{
				op = B3_REF_OP_LT;
				q = s + 1;
			}
			if ( op < 0 )
			{
				goto predicateFail;
			}
			b3RefPredicate pred;
			memset( &pred, 0, sizeof( pred ) );
			pred.kind = kind;
			pred.op = op;
			if ( !takeDouble( &q, &pred.value ) || !endOfLine( q ) )
			{
				goto predicateFail;
			}
			if ( e->predicateCount >= B3_REF_MAX_PREDICATES )
			{
				if ( error && errorSize > 0 )
				{
					snprintf( error, errorSize, "too many expect predicates" );
				}
				return false;
			}
			e->predicates[e->predicateCount++] = pred;
			e->hasExpect = true;
			return true;
		}
	}

	goto headerForm;

predicateFail:
	if ( error && errorSize > 0 )
	{
		snprintf( error, errorSize, "malformed expect predicate" );
	}
	return false;

headerForm:;
	memset( e, 0, sizeof( *e ) );
	if ( !takeWord( &p, "pos" ) || !takeDouble( &p, &e->posTolerance ) )
	{
		goto fail;
	}
	if ( !takeWord( &p, "rot" ) || !takeDouble( &p, &e->rotTolerance ) )
	{
		goto fail;
	}
	if ( !takeWord( &p, "vel" ) || !takeDouble( &p, &e->velTolerance ) )
	{
		goto fail;
	}
	if ( !takeWord( &p, "horizon" ) || !takeInt( &p, &e->horizon ) )
	{
		goto fail;
	}
	if ( !takeWord( &p, "counts" ) )
	{
		goto fail;
	}
	{
		char* s = skipWs( p );
		if ( strncmp( s, "exact", 5 ) == 0 )
		{
			e->countsExact = 1;
			e->countsDelta = 0;
			p = s + 5;
		}
		else
		{
			e->countsExact = 0;
			if ( !takeInt( &p, &e->countsDelta ) )
			{
				goto fail;
			}
		}
	}
	if ( !endOfLine( p ) )
	{
		goto fail;
	}
	e->hasExpect = true;
	return true;
fail:
	if ( error && errorSize > 0 )
	{
		snprintf( error, errorSize, "malformed expect header" );
	}
	return false;
}

// ---------------------------------------------------------------------------
// world directive
// ---------------------------------------------------------------------------

static bool parseWorld( char* p, b3RefScene* scene )
{
	b3RefWorld* w = &scene->world;
	if ( !takeWord( &p, "gravity" ) )
	{
		return false;
	}
	if ( !takeDouble( &p, &w->gravity[0] ) || !takeDouble( &p, &w->gravity[1] ) || !takeDouble( &p, &w->gravity[2] ) )
	{
		return false;
	}
	if ( !takeWord( &p, "dt" ) )
	{
		return false;
	}
	if ( !takeDouble( &p, &w->dtNumerator ) || !takeDouble( &p, &w->dtDenominator ) )
	{
		return false;
	}
	if ( w->dtDenominator == 0.0 )
	{
		return false;
	}
	if ( !takeWord( &p, "substeps" ) || !takeInt( &p, &w->subSteps ) )
	{
		return false;
	}
	if ( !takeWord( &p, "steps" ) || !takeInt( &p, &w->steps ) )
	{
		return false;
	}
	if ( takeWord( &p, "restitutionIterations" ) &&
		( !takeInt( &p, &w->restitutionIterations ) || w->restitutionIterations < 1 || w->restitutionIterations > 8 ) )
	{
		return false;
	}
	if ( !endOfLine( p ) )
	{
		return false;
	}
	w->hasWorld = true;
	return true;
}

// ---------------------------------------------------------------------------
// generator-form directives: scene name, hull_generator, steps
// ---------------------------------------------------------------------------

static bool parseSceneName( char* p, b3RefScene* scene )
{
	char* s = skipWs( p );
	size_t len = 0;
	while ( s[len] != '\0' && s[len] != ' ' && s[len] != '\t' && len < sizeof( scene->name ) - 1 )
	{
		++len;
	}
	if ( len == 0 )
	{
		return false;
	}
	memcpy( scene->name, s, len );
	scene->name[len] = '\0';
	scene->hasName = true;
	return endOfLine( s + len ) || ( s[len] == '\0' );
}

// The complex-hull generator, replicated engine-free in double so BOTH engines
// receive one shared point list (README.md "Generator-form scenes"). This is
// b3CreateComplexHull's PEEL BasicRandom: LCG state = state*2147001325 +
// 715136305, low 16 bits mapped to [-0.5, 0.5) by raw/65535 - 0.5, rejection
// sampled inside the unit sphere (len2 <= 0.25), normalized, scaled by radius.
// 32 points, upstream's fixed seed form.
static uint32_t nextComplexHullRandom( uint32_t* state )
{
	*state = *state * 2147001325u + 715136305u;
	return *state;
}

static double complexHullRandomCoordinate( uint32_t* state )
{
	uint32_t raw = nextComplexHullRandom( state ) & 0xffffu;
	return (double)raw / 65535.0 - 0.5;
}

static bool expandComplexHull( uint32_t seed, double radius, double points[][3], int pointCount )
{
	uint32_t state = seed;
	for ( int i = 0; i < pointCount; ++i )
	{
		double x, y, z;
		do
		{
			x = complexHullRandomCoordinate( &state );
			y = complexHullRandomCoordinate( &state );
			z = complexHullRandomCoordinate( &state );
		}
		while ( x * x + y * y + z * z > 0.25 );
		double len = sqrt( x * x + y * y + z * z );
		if ( len <= 0.0 )
		{
			return false; // degenerate triple; the LCG cycle never yields it
		}
		points[i][0] = radius * x / len;
		points[i][1] = radius * y / len;
		points[i][2] = radius * z / len;
	}
	return true;
}

// `hull_generator complex <seed> [radius <r>]` — radius defaults to 0.5 (the
// value every scene of this form states). Consumed by the next `body hull` line.
static bool parseHullGenerator( char* p, b3RefScene* scene )
{
	if ( !takeWord( &p, "complex" ) )
	{
		return false;
	}
	double seed = 0.0;
	if ( !takeDouble( &p, &seed ) || seed < 0.0 || seed > 4294967295.0 || seed != floor( seed ) )
	{
		return false;
	}
	double radius = 0.5;
	if ( takeWord( &p, "radius" ) )
	{
		if ( !takeDouble( &p, &radius ) || radius <= 0.0 )
		{
			return false;
		}
	}
	if ( !endOfLine( p ) )
	{
		return false;
	}
	scene->hullGeneratorSeed = (uint32_t)seed;
	scene->hullGeneratorRadius = radius;
	scene->hasHullGenerator = true;
	return true;
}

// `steps <n> dt <num>/<den> substeps <n>` — the generator-form world line.
// dt must be the exact fraction (fixed steps deterministically at num/den).
static bool parseStepsLine( char* p, b3RefScene* scene )
{
	b3RefWorld* w = &scene->world;
	if ( !takeInt( &p, &w->steps ) || w->steps <= 0 )
	{
		return false;
	}
	if ( !takeWord( &p, "dt" ) )
	{
		return false;
	}
	{
		int num = 0, den = 0;
		char* s = skipWs( p );
		if ( !takeInt( &p, &num ) )
		{
			return false;
		}
		s = skipWs( p );
		if ( *s == '/' )
		{
			p = s + 1;
			if ( !takeInt( &p, &den ) || den <= 0 )
			{
				return false;
			}
		}
		else
		{
			return false; // the generator form requires the num/den spelling
		}
		w->dtNumerator = (double)num;
		w->dtDenominator = (double)den;
	}
	if ( takeWord( &p, "substeps" ) )
	{
		if ( !takeInt( &p, &w->subSteps ) || w->subSteps <= 0 )
		{
			return false;
		}
	}
	else
	{
		w->subSteps = 4;
	}
	if ( !endOfLine( p ) )
	{
		return false;
	}
	return true;
}

// ---------------------------------------------------------------------------
// body directive
// ---------------------------------------------------------------------------

static bool parseBodyType( char** p, b3RefBodyType* type )
{
	char* s = skipWs( *p );
	if ( strncmp( s, "static", 6 ) == 0 && ( s[6] == ' ' || s[6] == '\0' || s[6] == '\n' ) )
	{
		*type = b3_refStatic;
		*p = s + 6;
		return true;
	}
	if ( strncmp( s, "dynamic", 7 ) == 0 && ( s[7] == ' ' || s[7] == '\0' || s[7] == '\n' ) )
	{
		*type = b3_refDynamic;
		*p = s + 7;
		return true;
	}
	if ( strncmp( s, "kinematic", 9 ) == 0 && ( s[9] == ' ' || s[9] == '\0' || s[9] == '\n' ) )
	{
		*type = b3_refKinematic;
		*p = s + 9;
		return true;
	}
	return false;
}

static bool parseBody( char* p, b3RefScene* scene )
{
	if ( scene->bodyCount >= B3_REF_MAX_BODIES )
	{
		return false;
	}

	// Generator form: `body hull position ... [density d]` and
	// `body box half hx hy hz position ... [density d] [static]`. The body and
	// its one shape are created together; omitted material fields take the
	// corpus defaults (density 1, friction 0.5, restitution 0).
	{
		char* q = p;
		if ( takeWord( &q, "hull" ) )
		{
			b3RefBody* b = &scene->bodies[scene->bodyCount];
			memset( b, 0, sizeof( *b ) );
			b->type = b3_refDynamic;
			b->enableSleep = true;
			b->safetyFactor = 0.5;
			if ( !takeWord( &q, "position" ) || !takeDouble( &q, &b->position[0] ) || !takeDouble( &q, &b->position[1] ) ||
				 !takeDouble( &q, &b->position[2] ) )
			{
				return false;
			}
			double density = 1.0;
			if ( takeWord( &q, "density" ) )
			{
				if ( !takeDouble( &q, &density ) || density <= 0.0 )
				{
					return false;
				}
			}
			if ( !endOfLine( q ) )
			{
				return false;
			}
			if ( !scene->hasHullGenerator || scene->shapeCount >= B3_REF_MAX_SHAPES )
			{
				return false;
			}
			b3RefShape* s = &scene->shapes[scene->shapeCount];
			memset( s, 0, sizeof( *s ) );
			s->bodyIndex = scene->bodyCount;
			s->kind = b3_refHull;
			s->hullCount = 32;
			if ( !expandComplexHull( scene->hullGeneratorSeed, scene->hullGeneratorRadius, s->hullPoints, s->hullCount ) )
			{
				return false;
			}
			s->density = density;
			s->friction = 0.5;
			s->restitution = 0.0;
			scene->shapeCount += 1;
			scene->bodyCount += 1;
			return true;
		}
		if ( takeWord( &q, "box" ) )
		{
			b3RefBody* b = &scene->bodies[scene->bodyCount];
			memset( b, 0, sizeof( *b ) );
			b->type = b3_refDynamic;
			b->enableSleep = true;
			b->safetyFactor = 0.5;
			double half[3] = { 0, 0, 0 };
			if ( !takeWord( &q, "half" ) || !takeDouble( &q, &half[0] ) || !takeDouble( &q, &half[1] ) ||
				 !takeDouble( &q, &half[2] ) )
			{
				return false;
			}
			if ( !takeWord( &q, "position" ) || !takeDouble( &q, &b->position[0] ) || !takeDouble( &q, &b->position[1] ) ||
				 !takeDouble( &q, &b->position[2] ) )
			{
				return false;
			}
			double density = 1.0;
			if ( takeWord( &q, "density" ) )
			{
				if ( !takeDouble( &q, &density ) || density <= 0.0 )
				{
					return false;
				}
			}
			b3RefBodyType trailing = b->type;
			if ( parseBodyType( &q, &trailing ) )
			{
				b->type = trailing;
			}
			if ( !endOfLine( q ) )
			{
				return false;
			}
			if ( half[0] <= 0.0 || half[1] <= 0.0 || half[2] <= 0.0 || scene->shapeCount >= B3_REF_MAX_SHAPES )
			{
				return false;
			}
			b3RefShape* s = &scene->shapes[scene->shapeCount];
			memset( s, 0, sizeof( *s ) );
			s->bodyIndex = scene->bodyCount;
			s->kind = b3_refBox;
			s->halfExtents[0] = half[0];
			s->halfExtents[1] = half[1];
			s->halfExtents[2] = half[2];
			s->density = density;
			s->friction = 0.5;
			s->restitution = 0.0;
			scene->shapeCount += 1;
			scene->bodyCount += 1;
			return true;
		}
	}

	b3RefBody* b = &scene->bodies[scene->bodyCount];
	memset( b, 0, sizeof( *b ) );
	b->enableSleep = true;
	b->safetyFactor = 0.5;
	if ( !parseBodyType( &p, &b->type ) )
	{
		return false;
	}
	if ( !takeWord( &p, "position" ) || !takeDouble( &p, &b->position[0] ) || !takeDouble( &p, &b->position[1] ) ||
		 !takeDouble( &p, &b->position[2] ) )
	{
		return false;
	}
	if ( !takeWord( &p, "rotation" ) || !takeDouble( &p, &b->rotation[0] ) || !takeDouble( &p, &b->rotation[1] ) ||
		 !takeDouble( &p, &b->rotation[2] ) || !takeDouble( &p, &b->rotation[3] ) )
	{
		return false;
	}
	if ( !takeWord( &p, "linvel" ) || !takeDouble( &p, &b->linearVelocity[0] ) || !takeDouble( &p, &b->linearVelocity[1] ) ||
		 !takeDouble( &p, &b->linearVelocity[2] ) )
	{
		return false;
	}
	if ( !takeWord( &p, "angvel" ) || !takeDouble( &p, &b->angularVelocity[0] ) || !takeDouble( &p, &b->angularVelocity[1] ) ||
		 !takeDouble( &p, &b->angularVelocity[2] ) )
	{
		return false;
	}
	// optional flags
	while ( !endOfLine( p ) )
	{
		if ( takeWord( &p, "flags" ) )
		{
			continue;
		}
		if ( takeWord( &p, "bullet" ) )
		{
			b->isBullet = true;
		}
		else if ( takeWord( &p, "fastrotation" ) )
		{
			b->allowFastRotation = true;
		}
		else if ( takeWord( &p, "nosleep" ) )
		{
			b->enableSleep = false;
		}
		else if ( takeWord( &p, "sleep" ) )
		{
			b->enableSleep = true;
		}
		else if ( takeWord( &p, "safetyFactor" ) )
		{
			if ( !takeDouble( &p, &b->safetyFactor ) || b->safetyFactor < 0.0 )
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	scene->bodyCount += 1;
	return true;
}

// ---------------------------------------------------------------------------
// shape directives
// ---------------------------------------------------------------------------

// common tail: density friction restitution [rolling <rr>] [sensor]
static bool parseMaterialTail( char* p, b3RefShape* s )
{
	if ( !takeWord( &p, "density" ) || !takeDouble( &p, &s->density ) )
	{
		return false;
	}
	if ( !takeWord( &p, "friction" ) || !takeDouble( &p, &s->friction ) )
	{
		return false;
	}
	if ( !takeWord( &p, "restitution" ) || !takeDouble( &p, &s->restitution ) )
	{
		return false;
	}
	while ( !endOfLine( p ) )
	{
		if ( takeWord( &p, "rolling" ) )
		{
			if ( !takeDouble( &p, &s->rollingResistance ) )
			{
				return false;
			}
		}
		else if ( takeWord( &p, "sensor" ) )
		{
			s->isSensor = true;
		}
		else
		{
			return false;
		}
	}
	return true;
}

// New vocabulary uses bounded integral counts and finite numbers. Reject bad
// geometry before it can reach an engine assertion (the legacy parser is unchanged).
static bool takeCount( char** p, int* out, int min, int max )
{
	double d;
	if ( !takeDouble( p, &d ) || !isfinite( d ) || d < min || d > max || floor( d ) != d )
		return false;
	*out = (int)d;
	return true;
}

static bool takeFinite( char** p, double* out )
{
	return takeDouble( p, out ) && isfinite( *out );
}

static bool takeVector( char** p, double* out, int n )
{
	for ( int i = 0; i < n; ++i )
		if ( !takeFinite( p, out + i ) )
			return false;
	return true;
}

static bool parseNewShape( char* p, b3RefScene* scene, b3RefShape* s, char* error, int errorSize )
{
	if ( s->bodyIndex < 0 || s->bodyIndex >= scene->bodyCount )
		return false;
	if ( takeWord( &p, "compound" ) )
	{
		s->kind = b3_refCompound;
		if ( scene->bodies[s->bodyIndex].type != b3_refStatic )
		{
			if ( error && errorSize > 0 )
				snprintf( error, errorSize, "public baked-compound API requires a static body" );
			return false;
		}
		s->childStart = scene->childCount;
		if ( !takeWord( &p, "children" ) || !takeCount( &p, &s->childCount, 1, B3_REF_MAX_CHILDREN - scene->childCount ) )
			return false;
		for ( int i = 0; i < s->childCount; ++i )
		{
			b3RefChild* c = &scene->children[scene->childCount++];
			if ( takeWord( &p, "box" ) )
			{
				c->kind = b3_refBox;
				if ( !takeVector( &p, c->halfExtents, 3 ) || c->halfExtents[0] <= 0 || c->halfExtents[1] <= 0 ||
					 c->halfExtents[2] <= 0 )
					return false;
			}
			else if ( takeWord( &p, "sphere" ) )
			{
				c->kind = b3_refSphere;
				if ( !takeFinite( &p, &c->radius ) || c->radius <= 0 )
					return false;
			}
			else if ( takeWord( &p, "capsule" ) )
			{
				c->kind = b3_refCapsule;
				if ( !takeFinite( &p, &c->radius ) || !takeFinite( &p, &c->halfLength ) || c->radius <= 0 || c->halfLength <= 0 )
					return false;
			}
			else if ( takeWord( &p, "hull" ) )
			{
				c->kind = b3_refHull;
				if ( !takeWord( &p, "points" ) || !takeCount( &p, &c->hullCount, 4, B3_REF_MAX_HULL_POINTS ) )
					return false;
				for ( int j = 0; j < c->hullCount; ++j )
					if ( !takeVector( &p, c->hullPoints[j], 3 ) )
						return false;
			}
			else
				return false;
			if ( !takeWord( &p, "position" ) || !takeVector( &p, c->position, 3 ) || !takeWord( &p, "rotation" ) ||
				 !takeVector( &p, c->rotation, 4 ) || !takeWord( &p, "density" ) || !takeFinite( &p, &c->density ) ||
				 c->density < 0 || !takeWord( &p, "friction" ) || !takeFinite( &p, &c->friction ) || c->friction < 0 ||
				 !takeWord( &p, "restitution" ) || !takeFinite( &p, &c->restitution ) || c->restitution < 0 )
				return false;
			double norm = 0;
			for ( int j = 0; j < 4; ++j )
				norm += c->rotation[j] * c->rotation[j];
			if ( !isfinite( norm ) || norm < 1e-12 )
				return false;
			// Both public baked-compound builders expose one shape density.
			if ( i == 0 )
				s->density = c->density;
			else if ( c->density != s->density )
			{
				if ( error && errorSize > 0 )
					snprintf( error, errorSize,
							  "compound child densities must match: public baked-compound API exposes one shape density" );
				return false;
			}
		}
	}
	else if ( takeWord( &p, "mesh" ) )
	{
		s->kind = b3_refMesh;
		if ( scene->bodies[s->bodyIndex].type != b3_refStatic )
			return false;
		if ( !takeWord( &p, "vertices" ) || !takeCount( &p, &s->vertexCount, 3, B3_REF_MAX_MESH_VERTICES ) )
			return false;
		for ( int i = 0; i < s->vertexCount; ++i )
			if ( !takeVector( &p, s->vertices[i], 3 ) )
				return false;
		if ( !takeWord( &p, "triangles" ) || !takeCount( &p, &s->triangleCount, 1, B3_REF_MAX_MESH_TRIANGLES ) )
			return false;
		for ( int i = 0; i < s->triangleCount; ++i )
			for ( int j = 0; j < 3; ++j )
				if ( !takeCount( &p, &s->triangles[i][j], 0, s->vertexCount - 1 ) )
					return false;
		s->materialCount = 1;
		s->materials[0][0] = 0.5;
		if ( takeWord( &p, "materials" ) )
		{
			if ( !takeCount( &p, &s->materialCount, 1, B3_REF_MAX_MATERIALS ) )
				return false;
			for ( int i = 0; i < s->materialCount; ++i )
				if ( !takeVector( &p, s->materials[i], 2 ) || s->materials[i][0] < 0 || s->materials[i][1] < 0 )
					return false;
			if ( !takeWord( &p, "indices" ) )
				return false;
			for ( int i = 0; i < s->triangleCount; ++i )
				if ( !takeCount( &p, &s->materialIndices[i], 0, s->materialCount - 1 ) )
					return false;
		}
	}
	else
		return false;
	if ( !endOfLine( p ) )
		return false;
	++scene->shapeCount;
	return true;
}

static bool parseSnapshot( char* p, b3RefScene* scene )
{
	int* step;
	if ( takeWord( &p, "save" ) )
		step = &scene->snapshotSaveStep;
	else if ( takeWord( &p, "restore" ) )
		step = &scene->snapshotRestoreStep;
	else
		return false;
	if ( *step != -1 )
		return false;
	return takeWord( &p, "at" ) && takeCount( &p, step, 0, 2047 ) && endOfLine( p );
}

static bool parseShape( char* p, b3RefScene* scene, char* error, int errorSize )
{
	if ( scene->shapeCount >= B3_REF_MAX_SHAPES )
	{
		return false;
	}
	b3RefShape* s = &scene->shapes[scene->shapeCount];
	memset( s, 0, sizeof( *s ) );
	s->density = 1.0;
	if ( !takeInt( &p, &s->bodyIndex ) )
	{
		return false;
	}
	char* kw = skipWs( p );
	char* probe = p;
	if ( takeWord( &probe, "compound" ) || takeWord( &probe, "mesh" ) )
		return parseNewShape( p, scene, s, error, errorSize );
	if ( takeWord( &p, "box" ) )
	{
		s->kind = b3_refBox;
		if ( !takeDouble( &p, &s->halfExtents[0] ) || !takeDouble( &p, &s->halfExtents[1] ) ||
			 !takeDouble( &p, &s->halfExtents[2] ) )
		{
			return false;
		}
	}
	else if ( takeWord( &p, "sphere" ) )
	{
		s->kind = b3_refSphere;
		if ( !takeDouble( &p, &s->radius ) )
		{
			return false;
		}
		// optional center
		char* save = skipWs( p );
		if ( takeWord( &p, "center" ) )
		{
			if ( !takeDouble( &p, &s->center[0] ) || !takeDouble( &p, &s->center[1] ) || !takeDouble( &p, &s->center[2] ) )
			{
				return false;
			}
		}
		else
		{
			p = save;
		}
	}
	else if ( takeWord( &p, "capsule" ) )
	{
		s->kind = b3_refCapsule;
		s->capsuleAxis[1] = 1.0; // default axis +Y
		if ( !takeDouble( &p, &s->radius ) || !takeDouble( &p, &s->halfLength ) )
		{
			return false;
		}
		char* save = skipWs( p );
		if ( takeWord( &p, "axis" ) )
		{
			if ( !takeDouble( &p, &s->capsuleAxis[0] ) || !takeDouble( &p, &s->capsuleAxis[1] ) ||
				 !takeDouble( &p, &s->capsuleAxis[2] ) )
			{
				return false;
			}
		}
		else
		{
			p = save;
		}
	}
	else if ( takeWord( &p, "hull" ) )
	{
		s->kind = b3_refHull;
		if ( !takeWord( &p, "points" ) || !takeInt( &p, &s->hullCount ) )
		{
			return false;
		}
		if ( s->hullCount < 0 || s->hullCount > B3_REF_MAX_HULL_POINTS )
		{
			return false;
		}
		for ( int i = 0; i < s->hullCount; ++i )
		{
			if ( !takeDouble( &p, &s->hullPoints[i][0] ) || !takeDouble( &p, &s->hullPoints[i][1] ) ||
				 !takeDouble( &p, &s->hullPoints[i][2] ) )
			{
				return false;
			}
		}
	}
	else
	{
		// unknown shape keyword
		p = kw;
		return false;
	}
	if ( !parseMaterialTail( p, s ) )
	{
		return false;
	}
	if ( s->bodyIndex < 0 || s->bodyIndex >= scene->bodyCount )
	{
		return false;
	}
	scene->shapeCount += 1;
	return true;
}

// ---------------------------------------------------------------------------
// joint directives
// ---------------------------------------------------------------------------

static bool parseVec3( char** p, double* v )
{
	return takeDouble( p, &v[0] ) && takeDouble( p, &v[1] ) && takeDouble( p, &v[2] );
}

static bool parseJoint( char* p, b3RefScene* scene )
{
	if ( scene->jointCount >= B3_REF_MAX_JOINTS )
	{
		return false;
	}
	b3RefJoint* j = &scene->joints[scene->jointCount];
	memset( j, 0, sizeof( *j ) );
	j->collideConnected = true;
	j->atStep = 0;
	if ( takeWord( &p, "revolute" ) )
	{
		j->kind = b3_refRevolute;
	}
	else if ( takeWord( &p, "spherical" ) )
	{
		j->kind = b3_refSpherical;
	}
	else if ( takeWord( &p, "distance" ) )
	{
		j->kind = b3_refDistance;
	}
	else if ( takeWord( &p, "weld" ) )
	{
		j->kind = b3_refWeld;
	}
	else
	{
		return false;
	}
	if ( !takeInt( &p, &j->bodyA ) || !takeInt( &p, &j->bodyB ) )
	{
		return false;
	}
	if ( j->bodyA < 0 || j->bodyA >= scene->bodyCount || j->bodyB < 0 || j->bodyB >= scene->bodyCount )
	{
		return false;
	}
	if ( j->kind == b3_refSpherical )
	{
		if ( !takeWord( &p, "anchor" ) || !parseVec3( &p, j->anchorA ) )
		{
			return false;
		}
	}
	else if ( j->kind == b3_refDistance )
	{
		if ( !takeWord( &p, "anchorA" ) || !parseVec3( &p, j->anchorA ) )
		{
			return false;
		}
		if ( !takeWord( &p, "anchorB" ) || !parseVec3( &p, j->anchorB ) )
		{
			return false;
		}
		if ( !takeWord( &p, "length" ) || !takeDouble( &p, &j->length ) )
		{
			return false;
		}
	}
	else // revolute / weld
	{
		if ( !takeWord( &p, "anchor" ) || !parseVec3( &p, j->anchorA ) )
		{
			return false;
		}
		if ( takeWord( &p, "axis" ) )
		{
			if ( !parseVec3( &p, j->axis ) )
			{
				return false;
			}
		}
		else
		{
			j->axis[2] = 1.0; // default +Z for revolute
		}
	}
	if ( takeWord( &p, "collideConnected" ) )
	{
		if ( takeWord( &p, "false" ) )
		{
			j->collideConnected = false;
		}
		else if ( takeWord( &p, "true" ) )
		{
			j->collideConnected = true;
		}
		else
		{
			return false;
		}
	}
	if ( takeWord( &p, "at_step" ) )
	{
		if ( !takeInt( &p, &j->atStep ) || j->atStep < 0 )
		{
			return false;
		}
	}
	if ( !endOfLine( p ) )
	{
		return false;
	}
	scene->jointCount += 1;
	return true;
}

// ---------------------------------------------------------------------------
// query directives
// ---------------------------------------------------------------------------

static bool parseQueryProxy( char** p, b3RefQuery* q )
{
	if ( takeWord( p, "box" ) )
	{
		q->proxyKind = b3_refBox;
		if ( !parseVec3( p, q->proxyHalfExtents ) )
		{
			return false;
		}
		return true;
	}
	if ( takeWord( p, "sphere" ) )
	{
		q->proxyKind = b3_refSphere;
		if ( !takeDouble( p, &q->proxyRadius ) )
		{
			return false;
		}
		return true;
	}
	if ( takeWord( p, "capsule" ) )
	{
		q->proxyKind = b3_refCapsule;
		q->proxyAxis[1] = 1.0;
		if ( !takeDouble( p, &q->proxyRadius ) || !takeDouble( p, &q->proxyHalfLength ) )
		{
			return false;
		}
		char* save = skipWs( *p );
		if ( takeWord( p, "axis" ) )
		{
			if ( !parseVec3( p, q->proxyAxis ) )
			{
				return false;
			}
		}
		else
		{
			*p = save;
		}
		return true;
	}
	return false;
}

static bool parseQuery( char* p, b3RefScene* scene )
{
	if ( scene->queryCount >= B3_REF_MAX_QUERIES )
	{
		return false;
	}
	b3RefQuery* q = &scene->queries[scene->queryCount];
	memset( q, 0, sizeof( *q ) );
	if ( takeWord( &p, "raycast" ) )
	{
		q->kind = b3_refRayCast;
		if ( !takeWord( &p, "origin" ) || !parseVec3( &p, q->origin ) )
		{
			return false;
		}
		if ( !takeWord( &p, "translation" ) || !parseVec3( &p, q->translation ) )
		{
			return false;
		}
	}
	else if ( takeWord( &p, "overlap_aabb" ) )
	{
		q->kind = b3_refOverlapAABB;
		if ( !takeWord( &p, "lower" ) || !parseVec3( &p, q->lower ) )
		{
			return false;
		}
		if ( !takeWord( &p, "upper" ) || !parseVec3( &p, q->upper ) )
		{
			return false;
		}
	}
	else if ( takeWord( &p, "overlap_shape" ) )
	{
		q->kind = b3_refOverlapShape;
		if ( !takeWord( &p, "origin" ) || !parseVec3( &p, q->origin ) )
		{
			return false;
		}
		if ( !parseQueryProxy( &p, q ) )
		{
			return false;
		}
	}
	else if ( takeWord( &p, "shape_cast" ) )
	{
		q->kind = b3_refShapeCast;
		if ( !takeWord( &p, "origin" ) || !parseVec3( &p, q->origin ) )
		{
			return false;
		}
		if ( !takeWord( &p, "translation" ) || !parseVec3( &p, q->translation ) )
		{
			return false;
		}
		if ( !parseQueryProxy( &p, q ) )
		{
			return false;
		}
	}
	else if ( takeWord( &p, "closest_points" ) )
	{
		q->kind = b3_refClosestPoints;
		if ( !takeInt( &p, &q->bodyA ) || !takeInt( &p, &q->bodyB ) )
		{
			return false;
		}
		if ( q->bodyA < 0 || q->bodyA >= scene->bodyCount || q->bodyB < 0 || q->bodyB >= scene->bodyCount )
		{
			return false;
		}
	}
	else if ( takeWord( &p, "mover" ) )
	{
		q->kind = b3_refMover;
		if ( !takeWord( &p, "origin" ) || !parseVec3( &p, q->origin ) )
		{
			return false;
		}
		if ( !takeDouble( &p, &q->proxyRadius ) || !takeDouble( &p, &q->proxyHalfLength ) )
		{
			return false;
		}
		q->proxyKind = b3_refCapsule;
		q->proxyAxis[1] = 1.0;
		if ( !takeWord( &p, "translation" ) || !parseVec3( &p, q->translation ) )
		{
			return false;
		}
	}
	else
	{
		return false;
	}
	if ( !endOfLine( p ) )
	{
		return false;
	}
	scene->queryCount += 1;
	return true;
}

// ---------------------------------------------------------------------------
// top-level parse
// ---------------------------------------------------------------------------

// `at_step <n> set_mass <body> center <x> <y> <z>` — the only mid-run event the
// 954cf87 scenes need. Fires before the world step whose index is <n>.
static bool parseEvent( char* p, b3RefScene* scene )
{
	if ( scene->eventCount >= B3_REF_MAX_EVENTS )
	{
		return false;
	}
	b3RefEvent* e = &scene->events[scene->eventCount];
	memset( e, 0, sizeof( *e ) );
	if ( !takeInt( &p, &e->step ) || e->step < 0 )
	{
		return false;
	}
	if ( takeWord( &p, "set_mass" ) )
	{
		e->kind = b3_refEventSetMass;
		if ( !takeInt( &p, &e->bodyIndex ) || e->bodyIndex < 0 || e->bodyIndex >= scene->bodyCount )
		{
			return false;
		}
		if ( !takeWord( &p, "center" ) || !parseVec3( &p, e->center ) )
		{
			return false;
		}
	}
	else
	{
		return false;
	}
	if ( !endOfLine( p ) )
	{
		return false;
	}
	scene->eventCount += 1;
	return true;
}

// `compound <body> sphere <x> <y> <z> <r>` — appends a sphere child to the baked
// compound shape built on <body>. The driver bakes one compound per body that has
// children, after every shape is created. Spheres are all the 954cf87
// compound-overlap scene needs.
static bool parseCompound( char* p, b3RefScene* scene )
{
	int bodyIndex = 0;
	if ( !takeInt( &p, &bodyIndex ) || bodyIndex < 0 || bodyIndex >= scene->bodyCount )
	{
		return false;
	}
	if ( !takeWord( &p, "sphere" ) )
	{
		return false;
	}
	if ( scene->compoundChildCount[bodyIndex] >= B3_REF_MAX_COMPOUND_CHILDREN )
	{
		return false;
	}
	b3RefCompoundChild* c = &scene->compoundChildren[bodyIndex][scene->compoundChildCount[bodyIndex]];
	if ( !parseVec3( &p, c->center ) || !takeDouble( &p, &c->radius ) || c->radius <= 0.0 )
	{
		return false;
	}
	if ( !endOfLine( p ) )
	{
		return false;
	}
	scene->compoundChildCount[bodyIndex] += 1;
	return true;
}

bool b3RefParseScene( const char* text, b3RefScene* scene, char* error, int errorSize )
{
	if ( error && errorSize > 0 )
		error[0] = 0;
	memset( scene, 0, sizeof( *scene ) );
	scene->snapshotSaveStep = scene->snapshotRestoreStep = -1;

	// The line scanner NULs out newlines in place, so work on a private copy and
	// never mutate the caller's buffer (which may be a string literal).
	char* buf = (char*)malloc( strlen( text ) + 1 );
	if ( buf == NULL )
	{
		if ( error && errorSize > 0 )
		{
			snprintf( error, errorSize, "out of memory" );
		}
		return false;
	}
	strcpy( buf, text );
	char* line = buf;
	bool expectSeen = false;

	while ( line != NULL && *line != '\0' )
	{
		// find end of line
		char* nl = strchr( line, '\n' );
		char* next = NULL;
		if ( nl != NULL )
		{
			*nl = '\0';
			next = nl + 1;
		}
		else
		{
			next = line + strlen( line );
		}

		char* p = skipWs( line );
		if ( *p != '\0' && *p != '#' )
		{
			// first word
			bool ok = false;
			if ( takeWord( &p, "expect" ) )
			{
				ok = parseExpect( p, scene, error, errorSize );
				expectSeen = ok;
			}
			else if ( takeWord( &p, "world" ) )
			{
				ok = parseWorld( p, scene );
			}
			else if ( takeWord( &p, "scene" ) )
			{
				ok = parseSceneName( p, scene );
			}
			else if ( takeWord( &p, "hull_generator" ) )
			{
				ok = parseHullGenerator( p, scene );
			}
			else if ( takeWord( &p, "steps" ) )
			{
				ok = parseStepsLine( p, scene );
			}
			else if ( takeWord( &p, "body" ) )
			{
				ok = parseBody( p, scene );
			}
			else if ( takeWord( &p, "shape" ) )
			{
				ok = parseShape( p, scene, error, errorSize );
			}
			else if ( takeWord( &p, "joint" ) )
			{
				ok = parseJoint( p, scene );
			}
			else if ( takeWord( &p, "query" ) )
			{
				ok = parseQuery( p, scene );
			}
			else if ( takeWord( &p, "at_step" ) )
			{
				ok = parseEvent( p, scene );
			}
			else if ( takeWord( &p, "snapshot" ) )
			{
				ok = parseSnapshot( p, scene );
			}
			else if ( takeWord( &p, "trace" ) )
			{
				ok = !scene->traceContacts && takeWord( &p, "contacts" ) && endOfLine( p );
				scene->traceContacts = ok;
				scene->expect.traceContacts = ok;
			}
			else if ( takeWord( &p, "compound" ) )
			{
				ok = parseCompound( p, scene );
			}
			else
			{
				if ( error && errorSize > 0 )
				{
					snprintf( error, errorSize, "unknown directive: %.32s", p );
				}
				free( buf );
				return false;
			}
			if ( !ok )
			{
				if ( error && errorSize > 0 && error[0] == 0 )
				{
					snprintf( error, errorSize, "malformed directive: %.32s", line );
				}
				free( buf );
				return false;
			}
		}

		line = next;
		if ( nl == NULL )
		{
			break;
		}
	}

	if ( ( scene->snapshotSaveStep >= 0 || scene->snapshotRestoreStep >= 0 ) &&
		 ( scene->snapshotSaveStep < 0 || scene->snapshotRestoreStep <= scene->snapshotSaveStep ||
		   scene->snapshotRestoreStep >= scene->world.steps ) )
	{
		if ( error && errorSize > 0 )
			snprintf( error, errorSize, "snapshot requires 0 <= save < restore < steps" );
		free( buf );
		return false;
	}
	if ( scene->traceContacts && scene->expect.predicateCount > 0 )
	{
		if ( error && errorSize > 0 )
			snprintf( error, errorSize, "trace contacts requires a tolerance header" );
		free( buf );
		return false;
	}
	if ( scene->expect.firstBounceApex && scene->expect.apexBody >= scene->bodyCount )
	{
		if ( error && errorSize > 0 )
			snprintf( error, errorSize, "firstBounceApex body is missing" );
		free( buf );
		return false;
	}
	if ( !expectSeen )
	{
		if ( error && errorSize > 0 )
		{
			snprintf( error, errorSize, "missing expect header" );
		}
		free( buf );
		return false;
	}

	// Generator-form scenes carry no world line: default gravity is b3Default-
	// WorldDef's (0, -10, 0) once a steps line is present.
	if ( !scene->world.hasWorld && scene->world.steps > 0 )
	{
		scene->world.gravity[0] = 0.0;
		scene->world.gravity[1] = -10.0;
		scene->world.gravity[2] = 0.0;
		scene->world.hasWorld = true;
	}
	free( buf );
	return true;
}

bool b3RefParseSceneFile( const char* path, b3RefScene* scene, char* error, int errorSize )
{
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		if ( error && errorSize > 0 )
		{
			snprintf( error, errorSize, "cannot open %s", path );
		}
		return false;
	}
	if ( fseek( f, 0, SEEK_END ) != 0 )
	{
		fclose( f );
		return false;
	}
	long size = ftell( f );
	if ( size < 0 )
	{
		fclose( f );
		return false;
	}
	rewind( f );
	char* buf = (char*)malloc( (size_t)size + 1 );
	if ( buf == NULL )
	{
		fclose( f );
		return false;
	}
	size_t read = fread( buf, 1, (size_t)size, f );
	fclose( f );
	buf[read] = '\0';
	bool ok = b3RefParseScene( buf, scene, error, errorSize );
	free( buf );
	return ok;
}

// ---------------------------------------------------------------------------
// trace comparison
// ---------------------------------------------------------------------------

typedef struct b3RefLineReader
{
	char* cursor;
	char* end;
} b3RefLineReader;

static char* nextLine( b3RefLineReader* r )
{
	if ( r->cursor >= r->end )
	{
		return NULL;
	}
	char* line = r->cursor;
	char* nl = memchr( r->cursor, '\n', (size_t)( r->end - r->cursor ) );
	if ( nl == NULL )
	{
		r->cursor = r->end;
	}
	else
	{
		*nl = '\0';
		r->cursor = nl + 1;
	}
	return line;
}

typedef struct b3RefBodySample
{
	double p[3];
	double q[4];
	double v[3];
	double w[3];
} b3RefBodySample;

static int compareContactPoint( const void* a, const void* b )
{
	const double* pa = a;
	const double* pb = b;
	for ( int i = 0; i < 4; ++i )
	{
		if ( pa[i] < pb[i] )
			return -1;
		if ( pa[i] > pb[i] )
			return 1;
	}
	return 0;
}

static int compareContact( const void* a, const void* b )
{
	const b3RefContactSample* ca = a;
	const b3RefContactSample* cb = b;
	if ( ca->shapeA != cb->shapeA )
		return ca->shapeA < cb->shapeA ? -1 : 1;
	if ( ca->shapeB != cb->shapeB )
		return ca->shapeB < cb->shapeB ? -1 : 1;
	if ( ca->pointCount != cb->pointCount )
		return ca->pointCount < cb->pointCount ? -1 : 1;
	for ( int i = 0; i < ca->pointCount; ++i )
	{
		int order = compareContactPoint( ca->points[i], cb->points[i] );
		if ( order != 0 )
			return order;
	}
	return 0;
}

void b3RefSortContacts( b3RefContactSample* contacts, int count )
{
	for ( int i = 0; i < count; ++i )
		qsort( contacts[i].points, (size_t)contacts[i].pointCount, sizeof( contacts[i].points[0] ), compareContactPoint );
	qsort( contacts, (size_t)count, sizeof( *contacts ), compareContact );
}

// Sorting is the wire canonicalization, not the correspondence rule: rounding
// can reverse nearly equal x coordinates. Match the complete point set with a
// bijection (at most 4! assignments); never greedily reuse the same point.
static double contactPointError( const b3RefContactSample* a, const b3RefContactSample* b, int index, unsigned used,
								 double maximum )
{
	if ( index == a->pointCount )
		return maximum;
	double best = HUGE_VAL;
	for ( int j = 0; j < b->pointCount; ++j )
	{
		if ( used & ( 1u << j ) )
			continue;
		double error = maximum;
		for ( int k = 0; k < 4; ++k )
		{
			double delta = fabs( a->points[index][k] - b->points[j][k] );
			if ( delta > error )
				error = delta;
		}
		double candidate = contactPointError( a, b, index + 1, used | ( 1u << j ), error );
		if ( candidate < best )
			best = candidate;
	}
	return best;
}

static bool matchContactRow( int row, int count, double errors[B3_REF_MAX_CONTACTS][B3_REF_MAX_CONTACTS], double tolerance,
							 int* matches, bool* seen )
{
	for ( int j = 0; j < count; ++j )
	{
		if ( seen[j] || errors[row][j] > tolerance )
			continue;
		seen[j] = true;
		if ( matches[j] < 0 || matchContactRow( matches[j], count, errors, tolerance, matches, seen ) )
		{
			matches[j] = row;
			return true;
		}
	}
	return false;
}

static bool compareContactSets( const b3RefContactSample* a, const b3RefContactSample* b, int count, double tolerance,
								double* maximum )
{
	double errors[B3_REF_MAX_CONTACTS][B3_REF_MAX_CONTACTS];
	int matches[B3_REF_MAX_CONTACTS];
	for ( int i = 0; i < count; ++i )
	{
		matches[i] = -1;
		for ( int j = 0; j < count; ++j )
			errors[i][j] = a[i].shapeA == b[j].shapeA && a[i].shapeB == b[j].shapeB && a[i].pointCount == b[j].pointCount
							   ? contactPointError( &a[i], &b[j], 0, 0, 0 )
							   : HUGE_VAL;
	}
	for ( int i = 0; i < count; ++i )
	{
		bool seen[B3_REF_MAX_CONTACTS] = { false };
		if ( !matchContactRow( i, count, errors, tolerance, matches, seen ) )
		{
			double nearest = HUGE_VAL;
			for ( int j = 0; j < count; ++j )
				if ( errors[i][j] < nearest )
					nearest = errors[i][j];
			*maximum = nearest;
			return false;
		}
	}
	for ( int j = 0; j < count; ++j )
		if ( errors[matches[j]][j] > *maximum )
			*maximum = errors[matches[j]][j];
	return true;
}

typedef struct b3RefStepSample
{
	int step;
	int counters[3]; // contact, awake, joint
	uint64_t awakeBits, awakeSeenBits; // apex mode: final per-body awake set
	bool hasContacts;
	int contactCount, expectedContacts;
	b3RefContactSample contacts[B3_REF_MAX_CONTACTS];
	int bodyCount;
	b3RefBodySample bodies[B3_REF_MAX_BODIES];
	// predicate-mode traces only: lowest transformed hull vertex per hull body
	double lowY[B3_REF_MAX_BODIES];
	bool hasLow[B3_REF_MAX_BODIES];
} b3RefStepSample;

typedef struct b3RefQuerySample
{
	int index;
	double values[16];
	int valueCount;
} b3RefQuerySample;

typedef struct b3RefTrace
{
	int stepCount;
	b3RefStepSample steps[2048];
	int queryCount;
	b3RefQuerySample queries[B3_REF_MAX_QUERIES];
} b3RefTrace;

// Parse a "body i p x y z q x y z w v x y z w x y z" line.
static bool parseTraceBodyLine( char* p, b3RefBodySample* b, int* index )
{
	if ( !takeWord( &p, "body" ) )
	{
		return false;
	}
	if ( !takeInt( &p, index ) )
	{
		return false;
	}
	if ( !takeWord( &p, "p" ) || !takeDouble( &p, &b->p[0] ) || !takeDouble( &p, &b->p[1] ) || !takeDouble( &p, &b->p[2] ) )
	{
		return false;
	}
	if ( !takeWord( &p, "q" ) || !takeDouble( &p, &b->q[0] ) || !takeDouble( &p, &b->q[1] ) || !takeDouble( &p, &b->q[2] ) ||
		 !takeDouble( &p, &b->q[3] ) )
	{
		return false;
	}
	if ( !takeWord( &p, "v" ) || !takeDouble( &p, &b->v[0] ) || !takeDouble( &p, &b->v[1] ) || !takeDouble( &p, &b->v[2] ) )
	{
		return false;
	}
	if ( !takeWord( &p, "w" ) || !takeDouble( &p, &b->w[0] ) || !takeDouble( &p, &b->w[1] ) || !takeDouble( &p, &b->w[2] ) )
	{
		return false;
	}
	return true;
}

static bool parseTraceCountersLine( char* p, int* counters )
{
	if ( !takeWord( &p, "counters" ) )
	{
		return false;
	}
	if ( !takeInt( &p, &counters[0] ) || !takeInt( &p, &counters[1] ) || !takeInt( &p, &counters[2] ) )
	{
		return false;
	}
	return true;
}

// A query line is "query i kind payload...". We parse the index and then capture
// up to 16 trailing numbers generically, comparing by position.
static bool parseTraceQueryLine( char* p, b3RefQuerySample* q )
{
	if ( !takeWord( &p, "query" ) )
	{
		return false;
	}
	if ( !takeInt( &p, &q->index ) )
	{
		return false;
	}
	// skip the kind word
	char* s = skipWs( p );
	while ( *s != '\0' && *s != ' ' && *s != '\t' )
	{
		++s;
	}
	p = s;
	q->valueCount = 0;
	while ( q->valueCount < 16 && takeDouble( &p, &q->values[q->valueCount] ) )
	{
		q->valueCount += 1;
	}
	return true;
}

// Parse a trace into a b3RefTrace (only works if the buffer is mutable).
static bool parseTrace( char* text, b3RefTrace* trace )
{
	memset( trace, 0, sizeof( *trace ) );
	b3RefLineReader r = { text, text + strlen( text ) };
	char* line;
	int curStep = -1;
	while ( ( line = nextLine( &r ) ) != NULL )
	{
		char* p = skipWs( line );
		if ( *p == '\0' || *p == '#' )
		{
			continue;
		}
		if ( takeWord( &p, "step" ) )
		{
			int n = 0;
			if ( !takeInt( &p, &n ) )
			{
				return false;
			}
			if ( trace->stepCount >= 2048 )
			{
				return false;
			}
			curStep = trace->stepCount;
			memset( &trace->steps[curStep], 0, sizeof( trace->steps[curStep] ) );
			trace->steps[curStep].step = n;
			trace->stepCount += 1;
			// the drivers emit "step <n> counters <c> <a> <j>" on ONE line; the
			// counters tail must be consumed here or it is silently dropped (a
			// standalone counters line is also accepted, e.g. hand-written traces)
			if ( takeWord( &p, "counters" ) )
			{
				if ( !takeInt( &p, &trace->steps[curStep].counters[0] ) || !takeInt( &p, &trace->steps[curStep].counters[1] ) ||
					 !takeInt( &p, &trace->steps[curStep].counters[2] ) )
				{
					return false;
				}
				if ( !endOfLine( p ) )
				{
					return false;
				}
			}
		}
		else if ( takeWord( &p, "counters" ) )
		{
			p = skipWs( line );
			if ( curStep < 0 || !parseTraceCountersLine( p, trace->steps[curStep].counters ) )
			{
				return false;
			}
		}
		else if ( takeWord( &p, "body" ) )
		{
			p = skipWs( line );
			if ( curStep < 0 )
			{
				return false;
			}
			b3RefStepSample* st = &trace->steps[curStep];
			if ( st->bodyCount >= B3_REF_MAX_BODIES )
			{
				return false;
			}
			int index = 0;
			if ( !parseTraceBodyLine( p, &st->bodies[st->bodyCount], &index ) )
			{
				return false;
			}
			st->bodyCount += 1;
		}
		else if ( takeWord( &p, "awake" ) )
		{
			int index = -1, value = -1;
			if ( curStep < 0 || !takeInt( &p, &index ) || !takeInt( &p, &value ) || !endOfLine( p ) ||
				 index < 0 || index >= B3_REF_MAX_BODIES || ( value != 0 && value != 1 ) )
				return false;
			b3RefStepSample* st = &trace->steps[curStep];
			uint64_t bit = UINT64_C( 1 ) << index;
			if ( st->awakeSeenBits & bit )
				return false;
			st->awakeSeenBits |= bit;
			if ( value != 0 )
				st->awakeBits |= bit;
		}
		else if ( takeWord( &p, "contacts" ) )
		{
			if ( curStep < 0 || trace->steps[curStep].hasContacts )
				return false;
			b3RefStepSample* st = &trace->steps[curStep];
			if ( !takeCount( &p, &st->expectedContacts, 0, B3_REF_MAX_CONTACTS ) || !endOfLine( p ) )
				return false;
			st->hasContacts = true;
		}
		else if ( takeWord( &p, "contact" ) )
		{
			if ( curStep < 0 )
				return false;
			b3RefStepSample* st = &trace->steps[curStep];
			if ( !st->hasContacts || st->contactCount >= st->expectedContacts )
				return false;
			b3RefContactSample* c = &st->contacts[st->contactCount++];
			if ( !takeCount( &p, &c->shapeA, 0, B3_REF_MAX_BODIES * ( B3_REF_MAX_SHAPES + 1 ) ) ||
				 !takeCount( &p, &c->shapeB, c->shapeA + 1, B3_REF_MAX_BODIES * ( B3_REF_MAX_SHAPES + 1 ) ) ||
				 !takeCount( &p, &c->pointCount, 1, B3_REF_MAX_CONTACT_POINTS ) )
				return false;
			for ( int i = 0; i < c->pointCount; ++i )
				if ( !takeVector( &p, c->points[i], 4 ) )
					return false;
			if ( !endOfLine( p ) )
				return false;
		}
		else if ( takeWord( &p, "low" ) )
		{
			// predicate-mode resting-height line: "low <i> <y>"
			if ( curStep < 0 )
			{
				return false;
			}
			int index = 0;
			double y = 0.0;
			if ( !takeInt( &p, &index ) || !takeDouble( &p, &y ) || !endOfLine( p ) )
			{
				return false;
			}
			if ( index < 0 || index >= B3_REF_MAX_BODIES )
			{
				return false;
			}
			trace->steps[curStep].lowY[index] = y;
			trace->steps[curStep].hasLow[index] = true;
		}
		else if ( takeWord( &p, "query" ) )
		{
			p = skipWs( line );
			if ( trace->queryCount >= B3_REF_MAX_QUERIES )
			{
				return false;
			}
			if ( !parseTraceQueryLine( p, &trace->queries[trace->queryCount] ) )
			{
				return false;
			}
			trace->queryCount += 1;
		}
		else
		{
			return false;
		}
	}
	for ( int i = 0; i < trace->stepCount; ++i )
	{
		b3RefStepSample* st = &trace->steps[i];
		if ( st->contactCount != st->expectedContacts )
			return false;
		b3RefSortContacts( st->contacts, st->contactCount );
	}
	return true;
}

// Find the first complete upward arc after a downward impact. The contact may
// begin on different steps, so compare its apex height rather than aligned
// per-step velocities. Requiring the return to nonpositive velocity prevents
// an incomplete trace from passing as an apex.
static bool firstBounceApex( const b3RefTrace* trace, int body, double* height, int* step )
{
	bool rising = false;
	for ( int i = 1; i < trace->stepCount; ++i )
	{
		const b3RefStepSample* prev = trace->steps + i - 1;
		const b3RefStepSample* cur = trace->steps + i;
		if ( body >= prev->bodyCount || body >= cur->bodyCount )
			return false;
		const b3RefBodySample* before = prev->bodies + body;
		const b3RefBodySample* after = cur->bodies + body;
		if ( !isfinite( after->p[1] ) || !isfinite( after->v[1] ) )
			return false;
		if ( !rising )
		{
			if ( before->v[1] < 0.0 && after->v[1] > 0.0 )
			{
				rising = true;
				*height = after->p[1];
				*step = cur->step;
			}
		}
		else
		{
			if ( after->p[1] > *height )
			{
				*height = after->p[1];
				*step = cur->step;
			}
			if ( after->v[1] <= 0.0 )
				return true;
		}
	}
	return false;
}

bool b3RefCompareTraces( const char* traceA, const char* traceB, const b3RefExpect* expect, char* report, int reportSize )
{
	// The traces are several MB each (2048 steps x 64 bodies), so they live on the
	// heap rather than the stack.
	b3RefTrace* a = (b3RefTrace*)malloc( sizeof( b3RefTrace ) );
	b3RefTrace* b = (b3RefTrace*)malloc( sizeof( b3RefTrace ) );
	char* copyA = (char*)malloc( strlen( traceA ) + 1 );
	char* copyB = (char*)malloc( strlen( traceB ) + 1 );
	if ( a == NULL || b == NULL || copyA == NULL || copyB == NULL )
	{
		free( a );
		free( b );
		free( copyA );
		free( copyB );
		return false;
	}
	strcpy( copyA, traceA );
	strcpy( copyB, traceB );
	bool okA = parseTrace( copyA, a );
	bool okB = parseTrace( copyB, b );
	if ( !okA || !okB )
	{
		if ( report && reportSize > 0 )
		{
			snprintf( report, reportSize, "trace parse error (A=%s B=%s)", okA ? "ok" : "bad", okB ? "ok" : "bad" );
		}
		free( copyA );
		free( copyB );
		free( a );
		free( b );
		return false;
	}

	double maxPos = 0.0, maxRot = 0.0, maxVel = 0.0, maxAng = 0.0;
	int maxContactDelta = 0, maxAwakeDelta = 0, maxJointDelta = 0;
	double maxQuery = 0.0;
	int firstFailStep = -1;
	int firstFailQuantity = -1; // 0 pos, 1 rot, 2 vel, 3 angvel, 4 counters, 5 query

	// structure mismatch guard
	if ( a->stepCount != b->stepCount )
	{
		if ( report && reportSize > 0 )
		{
			snprintf( report, reportSize, "step count mismatch: %d vs %d", a->stepCount, b->stepCount );
		}
		free( copyA );
		free( copyB );
		free( a );
		free( b );
		return false;
	}

	if ( expect->firstBounceApex )
	{
		double apexA = 0.0, apexB = 0.0;
		int apexStepA = -1, apexStepB = -1;
		bool foundA = firstBounceApex( a, expect->apexBody, &apexA, &apexStepA );
		bool foundB = firstBounceApex( b, expect->apexBody, &apexB, &apexStepB );
		const b3RefStepSample* finalA = a->stepCount > 0 ? a->steps + a->stepCount - 1 : NULL;
		const b3RefStepSample* finalB = b->stepCount > 0 ? b->steps + b->stepCount - 1 : NULL;
		bool complete = false, awakeMatch = false;
		if ( finalA != NULL && finalB != NULL && finalA->bodyCount == finalB->bodyCount && finalA->bodyCount > 0 )
		{
			uint64_t mask = finalA->bodyCount == B3_REF_MAX_BODIES ? UINT64_MAX :
														 ( UINT64_C( 1 ) << finalA->bodyCount ) - 1;
			complete = finalA->awakeSeenBits == mask && finalB->awakeSeenBits == mask;
			awakeMatch = finalA->awakeBits == finalB->awakeBits && finalA->counters[1] == finalB->counters[1];
		}
		double difference = fabs( apexA - apexB );
		bool passed = foundA && foundB && complete && awakeMatch && difference <= expect->apexTolerance;
		if ( report && reportSize > 0 )
			snprintf( report, reportSize,
					  "first-bounce apex body %d: %.9g (step %d) vs %.9g (step %d), difference %.9g (tol %.9g); final awake set 0x%llx vs 0x%llx (complete=%d)",
					  expect->apexBody, apexA, apexStepA, apexB, apexStepB, difference, expect->apexTolerance,
					  (unsigned long long)( finalA ? finalA->awakeBits : 0 ), (unsigned long long)( finalB ? finalB->awakeBits : 0 ),
					  complete );
		free( copyA );
		free( copyB );
		free( a );
		free( b );
		return passed;
	}

	// Predicate mode: the expect lines are absolute final-step bounds, not a
	// per-quantity header. Every predicate must hold on BOTH traces, the final
	// contact count must be equal, and the final resting height (the `low` line
	// of the first hull body) must agree within B3_REF_RESTING_Y_TOLERANCE.
	if ( expect->predicateCount > 0 )
	{
		bool predPassed = true;
		char lines[B3_REF_MAX_PREDICATES][96];
		int lineCount = 0;
		if ( a->stepCount == 0 || a->stepCount != b->stepCount )
		{
			if ( report && reportSize > 0 )
			{
				snprintf( report, reportSize, "predicate mode: step count mismatch (%d vs %d)", a->stepCount, b->stepCount );
			}
			free( copyA );
			free( copyB );
			free( a );
			free( b );
			return false;
		}
		int finalA = a->stepCount - 1;
		int finalB = b->stepCount - 1;
		int contactA = a->steps[finalA].counters[0];
		int contactB = b->steps[finalB].counters[0];

		int hullBody = -1;
		for ( int i = 0; i < B3_REF_MAX_BODIES; ++i )
		{
			if ( a->steps[finalA].hasLow[i] || b->steps[finalB].hasLow[i] )
			{
				hullBody = i;
				break;
			}
		}
		double restA = 0.0, restB = 0.0;
		bool hasRest = hullBody >= 0 && a->steps[finalA].hasLow[hullBody] && b->steps[finalB].hasLow[hullBody];
		if ( hasRest )
		{
			restA = a->steps[finalA].lowY[hullBody];
			restB = b->steps[finalB].lowY[hullBody];
		}

		for ( int i = 0; i < expect->predicateCount; ++i )
		{
			const b3RefPredicate* pr = &expect->predicates[i];
			const char* pname = pr->kind == b3_refPredContactCount ? "contactCount" : "restingY";
			double va = pr->kind == b3_refPredContactCount ? (double)contactA : restA;
			double vb = pr->kind == b3_refPredContactCount ? (double)contactB : restB;
			if ( pr->kind == b3_refPredRestingY && !hasRest )
			{
				predPassed = false;
				snprintf( lines[lineCount++], sizeof( lines[0] ), "%s: no low line in trace (FAIL)", pname );
				continue;
			}
			bool okA = ( pr->op == B3_REF_OP_GE )	? ( va >= pr->value )
					   : ( pr->op == B3_REF_OP_GT ) ? ( va > pr->value )
					   : ( pr->op == B3_REF_OP_LE ) ? ( va <= pr->value )
													: ( va < pr->value );
			bool okB = ( pr->op == B3_REF_OP_GE )	? ( vb >= pr->value )
					   : ( pr->op == B3_REF_OP_GT ) ? ( vb > pr->value )
					   : ( pr->op == B3_REF_OP_LE ) ? ( vb <= pr->value )
													: ( vb < pr->value );
			if ( !okA || !okB )
			{
				predPassed = false;
			}
			snprintf( lines[lineCount++], sizeof( lines[0] ), "%s: fixed %.9g float %.9g (bound %s %.9g) %s", pname, va, vb,
					  pr->op == B3_REF_OP_GE   ? ">="
					  : pr->op == B3_REF_OP_GT ? ">"
					  : pr->op == B3_REF_OP_LE ? "<="
											   : "<",
					  pr->value, ( okA && okB ) ? "ok" : "FAIL" );
		}

		double restDelta = hasRest ? fabs( restA - restB ) : 0.0;
		bool restOk = hasRest && restDelta <= B3_REF_RESTING_Y_TOLERANCE;
		bool countOk = contactA == contactB;
		if ( !restOk || !countOk )
		{
			predPassed = false;
		}

		if ( report && reportSize > 0 )
		{
			int off = 0;
			int room = reportSize - 1;
			off += snprintf( report + off, (size_t)room, "predicate mode (final step %d)\n", a->steps[finalA].step );
			room = reportSize - 1 - off;
			for ( int i = 0; i < lineCount && room > 0; ++i )
			{
				off += snprintf( report + off, (size_t)room, "%s\n", lines[i] );
				room = reportSize - 1 - off;
			}
			if ( room > 0 )
			{
				snprintf( report + off, (size_t)room,
						  "resting height: fixed %.9g float %.9g delta %.9g (tol %.9g) %s\n"
						  "final contact count: fixed %d float %d %s",
						  restA, restB, restDelta, (double)B3_REF_RESTING_Y_TOLERANCE, restOk ? "ok" : "FAIL", contactA, contactB,
						  countOk ? "equal" : "FAIL" );
			}
		}
		free( copyA );
		free( copyB );
		free( a );
		free( b );
		return predPassed;
	}

	// Only the first `horizon` steps are compared: chaotic scenes diverge past a
	// horizon in any number system, so the header states how many steps matter.
	int compareSteps = a->stepCount;
	if ( expect->horizon > 0 && expect->horizon < compareSteps )
	{
		compareSteps = expect->horizon;
	}
	for ( int s = 0; s < compareSteps; ++s )
	{
		b3RefStepSample* sa = &a->steps[s];
		b3RefStepSample* sb = &b->steps[s];
		if ( sa->bodyCount != sb->bodyCount )
		{
			if ( report && reportSize > 0 )
			{
				snprintf( report, reportSize, "body count mismatch at step %d: %d vs %d", sa->step, sa->bodyCount,
						  sb->bodyCount );
			}
			free( copyA );
			free( copyB );
			free( a );
			free( b );
			return false;
		}
		if ( expect->traceContacts || sa->hasContacts || sb->hasContacts )
		{
			const char* failure = NULL;
			double maximum = 0;
			if ( !sa->hasContacts || !sb->hasContacts )
				failure = "missing contact point set";
			else if ( sa->contactCount != sb->contactCount )
				failure = "manifold count differs";
			else if ( !compareContactSets( sa->contacts, sb->contacts, sa->contactCount, expect->posTolerance, &maximum ) )
				failure = "shape pair, point count, or point position/separation differs";
			if ( maximum > maxPos )
				maxPos = maximum;
			if ( failure != NULL )
			{
				if ( report && reportSize > 0 )
					snprintf( report, reportSize,
							  "contact mismatch at step %d: %s (manifolds %d/%d, max point error %.9g, tolerance %.9g)", sa->step,
							  failure, sa->contactCount, sb->contactCount, maximum, expect->posTolerance );
				free( copyA );
				free( copyB );
				free( a );
				free( b );
				return false;
			}
		}
		for ( int i = 0; i < sa->bodyCount; ++i )
		{
			b3RefBodySample* ba = &sa->bodies[i];
			b3RefBodySample* bb = &sb->bodies[i];
			for ( int c = 0; c < 3; ++c )
			{
				double d = fabs( ba->p[c] - bb->p[c] );
				if ( d > maxPos )
				{
					maxPos = d;
				}
				if ( d > expect->posTolerance && firstFailStep < 0 )
				{
					firstFailStep = sa->step;
					firstFailQuantity = 0;
				}
				d = fabs( ba->v[c] - bb->v[c] );
				if ( d > maxVel )
				{
					maxVel = d;
				}
				if ( d > expect->velTolerance && firstFailStep < 0 )
				{
					firstFailStep = sa->step;
					firstFailQuantity = 2;
				}
				d = fabs( ba->w[c] - bb->w[c] );
				if ( d > maxAng )
				{
					maxAng = d;
				}
				if ( d > expect->velTolerance && firstFailStep < 0 )
				{
					firstFailStep = sa->step;
					firstFailQuantity = 3;
				}
			}
			for ( int c = 0; c < 4; ++c )
			{
				double d = fabs( ba->q[c] - bb->q[c] );
				if ( d > maxRot )
				{
					maxRot = d;
				}
				if ( d > expect->rotTolerance && firstFailStep < 0 )
				{
					firstFailStep = sa->step;
					firstFailQuantity = 1;
				}
			}
		}
		int cd = abs( sa->counters[0] - sb->counters[0] );
		int ad = abs( sa->counters[1] - sb->counters[1] );
		int jd = abs( sa->counters[2] - sb->counters[2] );
		if ( cd > maxContactDelta )
		{
			maxContactDelta = cd;
		}
		if ( ad > maxAwakeDelta )
		{
			maxAwakeDelta = ad;
		}
		if ( jd > maxJointDelta )
		{
			maxJointDelta = jd;
		}
		int allow = expect->countsExact ? 0 : expect->countsDelta;
		if ( ( cd > allow || ad > allow || jd > allow ) && firstFailStep < 0 )
		{
			firstFailStep = sa->step;
			firstFailQuantity = 4;
		}
	}

	if ( a->queryCount != b->queryCount )
	{
		if ( report && reportSize > 0 )
		{
			snprintf( report, reportSize, "query count mismatch: %d vs %d", a->queryCount, b->queryCount );
		}
		free( copyA );
		free( copyB );
		free( a );
		free( b );
		return false;
	}
	for ( int i = 0; i < a->queryCount; ++i )
	{
		b3RefQuerySample* qa = &a->queries[i];
		b3RefQuerySample* qb = &b->queries[i];
		if ( qa->valueCount != qb->valueCount )
		{
			if ( report && reportSize > 0 )
			{
				snprintf( report, reportSize, "query %d arity mismatch", i );
			}
			free( copyA );
			free( copyB );
			free( a );
			free( b );
			return false;
		}
		for ( int c = 0; c < qa->valueCount; ++c )
		{
			double d = fabs( qa->values[c] - qb->values[c] );
			if ( d > maxQuery )
			{
				maxQuery = d;
			}
			if ( d > expect->posTolerance && firstFailStep < 0 )
			{
				firstFailStep = -2; // query-stage failure marker
				firstFailQuantity = 5;
			}
		}
	}

	// Verdict: every quantity inside tolerance.
	bool passed = ( maxPos <= expect->posTolerance && maxRot <= expect->rotTolerance && maxVel <= expect->velTolerance &&
					maxAng <= expect->velTolerance && maxQuery <= expect->posTolerance &&
					( expect->countsExact ? ( maxContactDelta == 0 && maxAwakeDelta == 0 && maxJointDelta == 0 )
										  : ( maxContactDelta <= expect->countsDelta && maxAwakeDelta <= expect->countsDelta &&
											  maxJointDelta <= expect->countsDelta ) ) );

	if ( report && reportSize > 0 )
	{
		char tbl[1024];
		snprintf( tbl, sizeof( tbl ),
				  "pos %.9g (tol %.9g)  rot %.9g (tol %.9g)  vel %.9g (tol %.9g)  angvel %.9g (tol %.9g)\n"
				  "counters contact %d awake %d joint %d (exact=%d delta=%d)\nquery %.9g (tol %.9g)",
				  maxPos, expect->posTolerance, maxRot, expect->rotTolerance, maxVel, expect->velTolerance, maxAng,
				  expect->velTolerance, maxContactDelta, maxAwakeDelta, maxJointDelta, expect->countsExact, expect->countsDelta,
				  maxQuery, expect->posTolerance );
		snprintf( report, reportSize, "%s\nfirst break: %s", tbl,
				  firstFailStep < 0 ? "none" : ( firstFailStep == -2 ? "query stage" : "" ) );
		if ( firstFailStep >= 0 )
		{
			char extra[64];
			snprintf( extra, sizeof( extra ), " at step %d (quantity %d)", firstFailStep, firstFailQuantity );
			strncat( report, extra, (size_t)reportSize - strlen( report ) - 1 );
		}
	}

	free( copyA );
	free( copyB );
	free( a );
	free( b );
	return passed;
}
