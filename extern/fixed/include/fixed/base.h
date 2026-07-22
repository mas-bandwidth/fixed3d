// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Minimal macro base for the fixed library, extracted from box3d/base.h so box3d
// can depend on this library without renaming its API.
#pragma once

#include <stdbool.h>
// Guarded so a host that already defines these (e.g. box3d/base.h, whose B3_API
// carries an export decoration) wins when both headers are in the same translation
// unit. Standalone users of `fixed` get the definitions below.
#ifndef B3_API
#ifdef __cplusplus
	#define B3_API extern "C"
	#define B3_INLINE inline
	#if defined( _MSC_VER )
		#define B3_FORCE_INLINE __forceinline
	#elif defined( __GNUC__ ) || defined( __clang__ )
		#define B3_FORCE_INLINE inline __attribute__((always_inline))
	#else
		#define B3_FORCE_INLINE inline
	#endif
	#define B3_LITERAL(T) T
	#define B3_ZERO_INIT {}
#else
	#define B3_API
	#define B3_INLINE static inline
	#if defined( _MSC_VER )
		#define B3_FORCE_INLINE static __forceinline
	#elif defined( __GNUC__ ) || defined( __clang__ )
		#define B3_FORCE_INLINE static inline __attribute__((always_inline))
	#else
		#define B3_FORCE_INLINE static inline
	#endif
	#define B3_LITERAL(T) (T)
	#define B3_ZERO_INIT {0}
#endif
#endif // B3_API

// Guarded assert hooks: a host (box3d) that defines these first wins; standalone
// users of `fixed` get plain assert() semantics.
#include <assert.h>
#ifndef B3_ASSERT
	#define B3_ASSERT( ... ) assert( ( __VA_ARGS__ ) )
#endif
#ifndef B3_VALIDATE
	#define B3_VALIDATE( ... ) ( (void)0 )
#endif
