// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Minimal macro base for the fixed library. Originally lifted from box3d/base.h so
// box3d could depend on this library without renaming its API -- that premise is gone:
// nothing here is named b3 any more, and a consumer wraps these types in its own
// vocabulary rather than sharing a namespace with them.
#pragma once

// EACH MACRO IS GUARDED SEPARATELY, and that is the whole point of the shape below.
//
// These were previously wrapped in one `#ifndef FIX_API` around the entire block, which
// looked equivalent and is not: a consumer that supplies only FIX_API -- the common case,
// because FIX_API is the one that carries an export decoration -- silently loses
// FIX_INLINE, FIX_FORCE_INLINE, FIX_LITERAL and FIX_ZERO_INIT along with it. The failure
// is a wall of "unknown type name 'FIX_INLINE'" pointing at this header, from a consumer
// who touched exactly one macro and reasonably expected to override exactly one.
//
// box3d hit this while vendoring the library and had to hand the whole set down to work
// around it. Guarding per macro means a consumer overrides what it means to override.
#include <stdbool.h>

#ifdef __cplusplus
	#ifndef FIX_API
		#define FIX_API extern "C"
	#endif
	#ifndef FIX_INLINE
		#define FIX_INLINE inline
	#endif
	#ifndef FIX_FORCE_INLINE
		#if defined( _MSC_VER )
			#define FIX_FORCE_INLINE __forceinline
		#elif defined( __GNUC__ ) || defined( __clang__ )
			#define FIX_FORCE_INLINE inline __attribute__((always_inline))
		#else
			#define FIX_FORCE_INLINE inline
		#endif
	#endif
	#ifndef FIX_LITERAL
		#define FIX_LITERAL(T) T
	#endif
	#ifndef FIX_ZERO_INIT
		#define FIX_ZERO_INIT {}
	#endif
#else
	#ifndef FIX_API
		#define FIX_API
	#endif
	#ifndef FIX_INLINE
		#define FIX_INLINE static inline
	#endif
	#ifndef FIX_FORCE_INLINE
		#if defined( _MSC_VER )
			#define FIX_FORCE_INLINE static __forceinline
		#elif defined( __GNUC__ ) || defined( __clang__ )
			#define FIX_FORCE_INLINE static inline __attribute__((always_inline))
		#else
			#define FIX_FORCE_INLINE static inline
		#endif
	#endif
	#ifndef FIX_LITERAL
		#define FIX_LITERAL(T) (T)
	#endif
	#ifndef FIX_ZERO_INIT
		#define FIX_ZERO_INIT {0}
	#endif
#endif

// Guarded assert hooks: a consumer that defines these first wins, so it can route
// this library's assertions into its own diagnostics. Standalone users of `fixed`
// get plain assert() semantics.
#include <assert.h>
#ifndef FIX_ASSERT
	#define FIX_ASSERT( ... ) assert( ( __VA_ARGS__ ) )
#endif
#ifndef FIX_VALIDATE
	#define FIX_VALIDATE( ... ) ( (void)0 )
#endif
