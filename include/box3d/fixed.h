// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

// box3d's fixed-point core now lives in the vendored `fixed` library (extern/fixed).
// This header is a shim so existing `#include "box3d/fixed.h"` / `#include "fixed.h"` sites
// keep working while the fixed-point core is maintained separately (mas-bandwidth/fixed) and
// vendored in. Evolve the type/API there; box3d picks it up through this include.
#include "fixed/fixed.h"
