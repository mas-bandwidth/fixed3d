// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_allocator.h"
#include "core.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"
#include "box3d/types.h"

#include <stdlib.h>

typedef struct
{
	int64_t allocCount;
	int64_t freeCount;
	int64_t totalAllocated;
	int64_t totalFreed;
	int maxSize;
	bool badAlignment;
} CountAllocator;

static CountAllocator g_alloc = { 0, 0, 0, 0, 0, false };

// The MSVC C runtime provides no aligned_alloc. Its pair is _aligned_malloc, whose
// arguments are reversed from C11's, and that memory may only be released by
// _aligned_free. This mirrors the spelling b3Alloc and b3Free use in src/core.c.
static void* TestAlignedAlloc( size_t size, size_t alignment )
{
#ifdef B3_PLATFORM_WINDOWS
	return _aligned_malloc( size, alignment );
#else
	// C11 requires the size to be a multiple of the alignment; the caller checks that
	return aligned_alloc( alignment, size );
#endif
}

static void TestAlignedFree( void* mem )
{
#ifdef B3_PLATFORM_WINDOWS
	_aligned_free( mem );
#else
	free( mem );
#endif
}

static void* CountingAlloc( size_t size, int32_t alignment )
{
	g_alloc.allocCount += 1;
	g_alloc.totalAllocated += (int64_t)size;

	if ( ( size & ( B3_ALIGNMENT - 1 ) ) != 0 )
	{
		g_alloc.badAlignment = true;
	}

	if ( alignment != B3_ALIGNMENT )
	{
		g_alloc.badAlignment = true;
	}

	if ( (int)size > g_alloc.maxSize )
	{
		g_alloc.maxSize = (int)size;
	}

	return TestAlignedAlloc( size, (size_t)alignment );
}

static void CountingFree( void* mem, size_t size )
{
	g_alloc.freeCount += 1;
	g_alloc.totalFreed += (int64_t)size;

	if ( ( size & ( B3_ALIGNMENT - 1 ) ) != 0 )
	{
		g_alloc.badAlignment = true;
	}

	TestAlignedFree( mem );
}

// The allocator API widening (b3AllocFcn/b3FreeFcn/b3GetByteCount). Installs a
// counting allocator through the widened signatures and checks the sizes and
// alignments the engine passes for a world create/step/destroy, and that every
// byte is freed at destroy.
static int AllocatorWidenedSignatureTest( void )
{
	g_alloc = (CountAllocator){ 0, 0, 0, 0, 0, false };

	b3SetAllocator( CountingAlloc, CountingFree );

	int64_t base = b3GetByteCount();

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( -10.0f ), B3_FIX( 0.0f ) };
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( B3_FIX( 50.0f ), B3_FIX( 10.0f ), B3_FIX( 50.0f ) );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ B3_FIX( 0.0f ), B3_FIX( 4.0f ), B3_FIX( 0.0f ) };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull dynBox = b3MakeCubeHull( B3_FIX( 1.0f ) );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = B3_FIX( 1.0f );
	b3CreateHullShape( bodyId, &shapeDef, &dynBox.base );

	b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ), B3_FIX( 60.0f ) );
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	b3DestroyWorld( worldId );

	b3SetAllocator( NULL, NULL );

	ENSURE( g_alloc.allocCount > 0 );
	ENSURE( g_alloc.freeCount == g_alloc.allocCount );
	ENSURE( g_alloc.totalFreed == g_alloc.totalAllocated );
	ENSURE( !g_alloc.badAlignment );
	ENSURE( g_alloc.maxSize >= 16 );
	ENSURE( b3GetByteCount() == base );

	return 0;
}

typedef struct
{
	int value1;
	b3Fixed value2;
} Foo;

_Static_assert( sizeof( Foo ) >= sizeof( void* ), "too small" );

static int TestBlockAllocate( void )
{
	b3BlockAllocator allocator = b3CreateBlockAllocator( (int)sizeof( Foo ), 2 );

	Foo* item1 = b3AllocateElement( &allocator );
	ENSURE( item1 != NULL );

	Foo* item2 = b3AllocateElement( &allocator );
	ENSURE( item2 != NULL );

	b3DestroyBlockAllocator( &allocator );
	return 0;
}

static int TestBlockClear( void )
{
	b3BlockAllocator allocator = b3CreateBlockAllocator( (int)sizeof( Foo ), 0 );

	Foo* item1 = b3AllocateElement( &allocator );
	Foo* item2 = b3AllocateElement( &allocator );

	b3FreeElement( &allocator, item1 );
	b3FreeElement( &allocator, item2 );

	b3DestroyBlockAllocator( &allocator );
	return 0;
}

// The free list link is a pointer stored in the element, so a stride that is not a multiple
// of the alignment leaves odd elements on a bad boundary. See box3d issue #78.
static int TestBlockAlignment( void )
{
	// The manifold sizes are the reported case
	int elementSizes[] = { 12, (int)sizeof( b3Manifold ), 3 * (int)sizeof( b3Manifold ) };

	for ( int i = 0; i < ARRAY_COUNT( elementSizes ); ++i )
	{
		b3BlockAllocator allocator = b3CreateBlockAllocator( elementSizes[i], 0 );

		// Hold the elements so the indices advance and a block boundary is crossed
		void* elements[B3_BLOCK_SIZE + 8];
		int count = ARRAY_COUNT( elements );

		for ( int j = 0; j < count; ++j )
		{
			elements[j] = b3AllocateElement( &allocator );
			ENSURE( elements[j] != NULL );
			ENSURE( ( (uintptr_t)elements[j] & ( B3_ALIGNMENT - 1 ) ) == 0 );
		}

		for ( int j = 0; j < count; ++j )
		{
			b3FreeElement( &allocator, elements[j] );
		}

		// Now drain the free list
		for ( int j = 0; j < count; ++j )
		{
			elements[j] = b3AllocateElement( &allocator );
			ENSURE( elements[j] != NULL );
			ENSURE( ( (uintptr_t)elements[j] & ( B3_ALIGNMENT - 1 ) ) == 0 );
		}

		for ( int j = 0; j < count; ++j )
		{
			b3FreeElement( &allocator, elements[j] );
		}

		b3DestroyBlockAllocator( &allocator );
	}

	return 0;
}

int AllocatorTest( void )
{
	RUN_SUBTEST( TestBlockAllocate );
	RUN_SUBTEST( TestBlockClear );
	RUN_SUBTEST( TestBlockAlignment );
	RUN_SUBTEST( AllocatorWidenedSignatureTest );

	return 0;
}
