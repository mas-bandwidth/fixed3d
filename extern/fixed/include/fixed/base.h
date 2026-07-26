// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Minimal macro base for the fixed library. Originally lifted from box3d/base.h so
// box3d could depend on this library without renaming its API -- that premise is gone:
// nothing here is named b3 any more, and a consumer wraps these types in its own
// vocabulary rather than sharing a namespace with them.
#pragma once

#include <stdbool.h>
// Guarded so a consumer that needs its own definitions -- an export decoration on
// FIX_API, a different inline policy -- can supply them before including this header
// and have them win. Standalone users of `fixed` get the definitions below.
#ifndef FIX_API
#ifdef __cplusplus
	#define FIX_API extern "C"
	#define FIX_INLINE inline
	#if defined( _MSC_VER )
		#define FIX_FORCE_INLINE __forceinline
	#elif defined( __GNUC__ ) || defined( __clang__ )
		#define FIX_FORCE_INLINE inline __attribute__((always_inline))
	#else
		#define FIX_FORCE_INLINE inline
	#endif
	#define FIX_LITERAL(T) T
	#define FIX_ZERO_INIT {}
#else
	#define FIX_API
	#define FIX_INLINE static inline
	#if defined( _MSC_VER )
		#define FIX_FORCE_INLINE static __forceinline
	#elif defined( __GNUC__ ) || defined( __clang__ )
		#define FIX_FORCE_INLINE static inline __attribute__((always_inline))
	#else
		#define FIX_FORCE_INLINE static inline
	#endif
	#define FIX_LITERAL(T) (T)
	#define FIX_ZERO_INIT {0}
#endif
#endif // FIX_API

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
