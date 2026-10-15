// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// Trace comparer: reads a scene header and two traces, prints a per-quantity
// maximum-error table and the first step that breaks the tolerance, and exits 0
// only inside tolerance.

#include "ref_scene.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char* readFile( const char* path )
{
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		return NULL;
	}
	fseek( f, 0, SEEK_END );
	long size = ftell( f );
	if ( size < 0 )
	{
		fclose( f );
		return NULL;
	}
	rewind( f );
	char* buf = (char*)malloc( (size_t)size + 1 );
	if ( buf == NULL )
	{
		fclose( f );
		return NULL;
	}
	size_t n = fread( buf, 1, (size_t)size, f );
	fclose( f );
	buf[n] = '\0';
	return buf;
}

int main( int argc, char** argv )
{
	if ( argc < 4 )
	{
		fprintf( stderr, "usage: ref_compare <scene> <traceA> <traceB>\n" );
		return 2;
	}

	b3RefScene scene;
	char error[256];
	if ( !b3RefParseSceneFile( argv[1], &scene, error, sizeof( error ) ) )
	{
		fprintf( stderr, "scene parse error: %s\n", error );
		return 2;
	}

	char* traceA = readFile( argv[2] );
	char* traceB = readFile( argv[3] );
	if ( traceA == NULL || traceB == NULL )
	{
		fprintf( stderr, "cannot read traces\n" );
		free( traceA );
		free( traceB );
		return 2;
	}

	char report[2048];
	bool ok = b3RefCompareTraces( traceA, traceB, &scene.expect, report, sizeof( report ) );
	printf( "%s\n", report );
	printf( "result: %s\n", ok ? "PASS (inside tolerance)" : "FAIL (outside tolerance)" );

	free( traceA );
	free( traceB );
	return ok ? 0 : 1;
}
