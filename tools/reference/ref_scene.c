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

static bool parseExpect( char* p, b3RefScene* scene, char* error, int errorSize )
{
	b3RefExpect* e = &scene->expect;
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
	if ( !endOfLine( p ) )
	{
		return false;
	}
	w->hasWorld = true;
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
	b3RefBody* b = &scene->bodies[scene->bodyCount];
	memset( b, 0, sizeof( *b ) );
	b->enableSleep = true;
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
	if ( !takeWord( &p, "angvel" ) || !takeDouble( &p, &b->angularVelocity[0] ) ||
		 !takeDouble( &p, &b->angularVelocity[1] ) || !takeDouble( &p, &b->angularVelocity[2] ) )
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

static bool parseShape( char* p, b3RefScene* scene )
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

bool b3RefParseScene( const char* text, b3RefScene* scene, char* error, int errorSize )
{
	memset( scene, 0, sizeof( *scene ) );

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
			else if ( takeWord( &p, "body" ) )
			{
				ok = parseBody( p, scene );
			}
			else if ( takeWord( &p, "shape" ) )
			{
				ok = parseShape( p, scene );
			}
			else if ( takeWord( &p, "joint" ) )
			{
				ok = parseJoint( p, scene );
			}
			else if ( takeWord( &p, "query" ) )
			{
				ok = parseQuery( p, scene );
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
				if ( error && errorSize > 0 )
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

	if ( !expectSeen )
	{
		if ( error && errorSize > 0 )
		{
			snprintf( error, errorSize, "missing expect header" );
		}
		free( buf );
		return false;
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

typedef struct b3RefStepSample
{
	int step;
	int counters[3]; // contact, awake, joint
	int bodyCount;
	b3RefBodySample bodies[B3_REF_MAX_BODIES];
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
	if ( !takeWord( &p, "q" ) || !takeDouble( &p, &b->q[0] ) || !takeDouble( &p, &b->q[1] ) ||
		 !takeDouble( &p, &b->q[2] ) || !takeDouble( &p, &b->q[3] ) )
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
	return true;
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
				snprintf( report, reportSize, "body count mismatch at step %d: %d vs %d", sa->step, sa->bodyCount, sb->bodyCount );
			}
			free( copyA );
			free( copyB );
			free( a );
			free( b );
			return false;
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
					( expect->countsExact
						  ? ( maxContactDelta == 0 && maxAwakeDelta == 0 && maxJointDelta == 0 )
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
