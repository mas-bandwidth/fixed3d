// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

// The baked blob stores only offsets. A stack tree view supplies the legacy traversal API.
static inline b3DynamicTree b3MakeCompoundTree( const b3CompoundData* compound )
{
	b3DynamicTree tree = { 0 };
	tree.version = B3_DYNAMIC_TREE_VERSION;
	tree.nodes = (b3TreeNode*)( (uintptr_t)compound + (uintptr_t)compound->nodeOffset );
	tree.root = compound->root;
	tree.nodeCount = compound->nodeCount;
	tree.nodeCapacity = compound->nodeCount;
	return tree;
}

b3TOIOutput b3CompoundTimeOfImpact( const b3CompoundData* compound, b3Transform transform, const b3ShapeProxy* proxy,
									const b3Sweep* sweep, b3Fixed maxFraction );

// Transforms a sweep for a compound child shape
b3Sweep b3MakeCompoundChildSweep( b3Transform compoundTransform, b3Transform childTransform );

int b3CollideMoverAndCompound( b3PlaneResult* planes, int capacity, const b3CompoundData* shape, const b3Capsule* mover );
