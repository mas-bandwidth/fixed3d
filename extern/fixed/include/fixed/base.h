// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Minimal macro base for the fixed library, extracted from box3d/base.h so box3d
// can depend on this library without renaming its API.
#pragma once
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
