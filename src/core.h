// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/base.h"

#include <stddef.h>

// clang-format off

#ifdef NDEBUG
	#define B3_DEBUG 0
#else
	#define B3_DEBUG 1
#endif

// Define platform
#if defined(_WIN32) || defined(_WIN64)
	#define B3_PLATFORM_WINDOWS
#elif defined( __ANDROID__ )
	#define B3_PLATFORM_ANDROID
#elif defined( __linux__ )
	#define B3_PLATFORM_LINUX
#elif defined( __APPLE__ )
	#include <TargetConditionals.h>
	#if defined( TARGET_OS_IPHONE ) && !TARGET_OS_IPHONE
		#define B3_PLATFORM_MACOS
	#else
		#define B3_PLATFORM_IOS
	#endif
#elif defined( __EMSCRIPTEN__ )
	#define B3_PLATFORM_WASM
#else
	#define B3_PLATFORM_UNKNOWN
#endif

// Define CPU
#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( _M_IX86 )
	#define B3_CPU_X86_X64
#elif defined( __aarch64__ ) || defined( _M_ARM64 ) || defined( __arm__ ) || defined( _M_ARM )
	#define B3_CPU_ARM
#elif defined( __EMSCRIPTEN__ )
	#define B3_CPU_WASM
#else
	#define B3_CPU_UNKNOWN
#endif

// Define SIMD. Fixed-point math uses 64 bit integer lanes with 128 bit
// intermediates, which do not map onto the old float SSE2/NEON paths. The
// opt-in AVX-512 path (BOX3D_AVX512, x86-64 with AVX512F/DQ/VL) keeps the same
// four-lane layout and is bit-identical to the scalar path: exact product
// decompositions, same round-half-up, same wrapping. It implements the default
// wrapping b3FixMul only, so it is disabled under BOX3D_FIXED_SATURATE.
//
// The opt-in NEON path (BOX3D_NEON, aarch64) covers only the narrow-phase
// scans: NEON has no 64-bit lane multiply (that is SVE2/SME territory, which
// Apple silicon through the M3 does not expose), so the wide solver stays
// scalar on ARM — Apple's scalar core wins the emulation trade. The SAT edge
// query and hull support scans, however, run on values an exactness gate
// proves fit in int32, where NEON's native smull/smlal 32x32->64 widening
// multiplies apply. Bit-identical to the scalar path for all inputs; gated
// values that do not fit fall back to the 128-bit scalar scans.
#if defined( BOX3D_AVX512 ) && !defined( BOX3D_FIXED_SATURATE )
	#if defined( __AVX512F__ ) && defined( __AVX512DQ__ ) && defined( __AVX512VL__ )
		#define B3_SIMD_AVX512
	#else
		#error "BOX3D_AVX512 requires AVX512F/DQ/VL (-mavx512f -mavx512dq -mavx512vl or /arch:AVX512)"
	#endif
#elif defined( BOX3D_NEON )
	#if defined( __aarch64__ ) || defined( _M_ARM64 )
		#define B3_SIMD_NEON
	#else
		#error "BOX3D_NEON requires an aarch64 target"
	#endif
#else
	#define B3_SIMD_NONE
#endif
#define B3_SIMD_WIDTH 4

// Define compiler
#if defined( __clang__ )
	#define B3_COMPILER_CLANG
#elif defined( __GNUC__ )
	#define B3_COMPILER_GCC
#elif defined( _MSC_VER )
	#define B3_COMPILER_MSVC
#endif

/// Tracy profiler instrumentation
/// https://github.com/wolfpld/tracy
#ifdef BOX3D_PROFILE
	#include <tracy/TracyC.h>
	#define b3TracyCZoneC( ctx, color, active ) TracyCZoneC( ctx, color, active )
	#define b3TracyCZoneNC( ctx, name, color, active ) TracyCZoneNC( ctx, name, color, active )
	#define b3TracyCZoneEnd( ctx ) TracyCZoneEnd( ctx )
	#define b3TracyCFrame TracyCFrameMark
#else
	#define b3TracyCZoneC( ctx, color, active )
	#define b3TracyCZoneNC( ctx, name, color, active )
	#define b3TracyCZoneEnd( ctx )
	#define b3TracyCFrame
#endif

// clang-format on

typedef struct b3AtomicInt
{
	int value;
} b3AtomicInt;

typedef struct b3AtomicU32
{
	uint32_t value;
} b3AtomicU32;

// Minimum memory alignment used for all allocations. The AVX-512 solver path
// stores wide constraint lanes as __m256i, so allocations must be 32-byte
// aligned for its aligned vector loads and stores.
#if defined( B3_SIMD_AVX512 )
	#define B3_ALIGNMENT 32
#else
	#define B3_ALIGNMENT 16
#endif

// Returns the number of elements of an array
#define B3_ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

// Used to prevent the compiler from warning about unused variables
#define B3_UNUSED( ... ) (void)sizeof( ( __VA_ARGS__, 0 ) )

// Use to validate definitions. Do not take my cookie.
#define B3_SECRET_COOKIE 1152023

#define B3_CHECK_DEF( DEF ) B3_ASSERT( DEF->internalValue == B3_SECRET_COOKIE )
#define B3_CHECK_JOINT_DEF( DEF ) B3_ASSERT( DEF->base.internalValue == B3_SECRET_COOKIE )

// These macros help avoid sizeof bugs
#define B3_ALLOC( T, N ) (T*)b3Alloc( N * sizeof( T ) );
#define B3_FREE( M, T, N ) b3Free( M, N * sizeof( T ) );

void* b3Alloc( size_t size );
void* b3AllocZeroed( size_t size );
void b3Free( void* mem, size_t size );
void* b3GrowAlloc( void* oldMem, int oldSize, int newSize );

#if defined( __GNUC__ ) || defined( __clang__ )
void b3Log( const char* format, ... ) __attribute__( ( format( printf, 1, 2 ) ) );
#else
void b3Log( const char* format, ... );
#endif

// Geometry content hashes reserve zero to mean unhashed
static inline uint32_t b3NonZeroHash( uint32_t hash )
{
	return hash != 0 ? hash : 1;
}

typedef struct b3Mutex b3Mutex;
b3Mutex* b3CreateMutex( void );
void b3DestroyMutex( b3Mutex* m );
void b3LockMutex( b3Mutex* m );
void b3UnlockMutex( b3Mutex* m );

typedef struct b3Semaphore b3Semaphore;
b3Semaphore* b3CreateSemaphore( int initCount );
void b3DestroySemaphore( b3Semaphore* s );
void b3WaitSemaphore( b3Semaphore* s );
void b3SignalSemaphore( b3Semaphore* s );

typedef void b3ThreadFunction( void* context );
typedef struct b3Thread b3Thread;
// Name may be NULL, otherwise it is copied.
b3Thread* b3CreateThread( b3ThreadFunction* function, void* context, const char* name );
void b3JoinThread( b3Thread* t );

void b3StrCpy( char* dst, int size, const char* src );
