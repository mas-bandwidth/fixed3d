// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
//
// Box3D's fixed-point core now lives in mas-bandwidth/fixed, vendored at
// extern/fixed and pinned in tools/revendor-fixed.sh.
//
// This file used to hold the Q48.16 type and its arithmetic directly. It is kept --
// rather than deleted and its include sites rewritten -- precisely because staying
// drop-in compatible with Box3D is a design goal: anything that included
// <box3d/fixed.h> still gets exactly what it got before, under exactly the same names.
//
// The seam itself is box3d/fixed_compat.h.
#pragma once

#include "box3d/fixed_compat.h"
