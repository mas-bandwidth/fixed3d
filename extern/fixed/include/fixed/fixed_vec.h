// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D (https://github.com/erincatto/box3d)
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
// Fixed-point vector / quaternion / matrix / transform types. In fixed point, world
// positions have uniform precision everywhere, so b3Pos is just b3Vec3 -- no separate
// wide-world type. Ops on these types are being migrated here incrementally.
#pragma once
#include "fixed/base.h"
#include "fixed/fixed.h"

/// A 2D vector.
typedef struct b3Vec2
{
	b3Fixed x;
	b3Fixed y;
} b3Vec2;

/// A 3D vector.
typedef struct b3Vec3
{
	b3Fixed x;
	b3Fixed y;
	b3Fixed z;
} b3Vec3;

/// A quaternion.
typedef struct b3Quat
{
	b3Vec3 v;
	b3Fixed s;
} b3Quat;

/// A rigid transform.
typedef struct b3Transform
{
	b3Vec3 p;
	b3Quat q;
} b3Transform;

/// A world position. Fixed point has uniform precision everywhere, so world
/// positions use the same representation as local vectors.
typedef b3Vec3 b3Pos;

/// A world transform. Same representation as a local transform in fixed point.
typedef b3Transform b3WorldTransform;

/// A 3x3 matrix.
typedef struct b3Matrix3
{
	b3Vec3 cx, cy, cz;
} b3Matrix3;
