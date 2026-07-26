// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC
// SPDX-License-Identifier: MIT
//
// Force-included into the VENDORED `fixed` sources (extern/fixed/src/*.c) by
// src/CMakeLists.txt. Not included by anything else, and not part of box3d's API.
//
// WHY THIS EXISTS, and it is not obvious:
//
// The vendored .c files include only the library's own headers. They never see
// box3d/fixed_compat.h, so the plumbing box3d hands down there -- FIX_API and friends --
// does not reach them. They compile with the library's own defaults, where FIX_API is a
// bare `extern "C"` with no export decoration.
//
// box3d builds with -fvisibility=hidden. So fixAtan2 and fixComputeCosSin landed in
// libbox3d as HIDDEN symbols, while the inline forwarders in box3d's public headers
// reference them from consumer code. Static links never noticed. The shared-library
// sample builds failed with:
//
//     Undefined symbols for architecture arm64:
//       "_fixAtan2", referenced from: ...
//
// Force-including this header in front of those two translation units gives them box3d's
// export decoration, so they leave the shared library like the rest of box3d's math.
//
// The alternative was per-file visibility flags, which would have needed one spelling for
// GCC/Clang and another for MSVC, and would have exported more than intended.
#pragma once

#include "box3d/base.h"

// base.h defines B3_API and the inline/literal conventions. Hand the whole set across:
// the library guards its macro block on `#ifndef FIX_API`, so defining that one alone
// would skip the rest of the block with it.
#ifndef FIX_API
	#define FIX_API B3_API
#endif
#ifndef FIX_INLINE
	#define FIX_INLINE B3_INLINE
#endif
#ifndef FIX_FORCE_INLINE
	#define FIX_FORCE_INLINE B3_FORCE_INLINE
#endif
#ifndef FIX_LITERAL
	#define FIX_LITERAL B3_LITERAL
#endif
#ifndef FIX_ZERO_INIT
	#define FIX_ZERO_INIT B3_ZERO_INIT
#endif
#ifndef FIX_ASSERT
	#define FIX_ASSERT B3_ASSERT
#endif
#ifndef FIX_VALIDATE
	#define FIX_VALIDATE B3_VALIDATE
#endif
