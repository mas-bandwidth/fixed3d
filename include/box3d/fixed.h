// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
//
// Box3D's fixed-point core lives in mas-bandwidth/fixed, vendored at extern/fixed and
// pinned in tools/revendor-fixed.sh.
//
// This file stays because staying drop-in compatible with Box3D is a design goal:
// anything that includes <box3d/fixed.h> gets the Q48.16 type and its arithmetic under
// exactly the names Box3D uses. The seam itself is box3d/fixed_compat.h.
#pragma once

#include "box3d/fixed_compat.h"
