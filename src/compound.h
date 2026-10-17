// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

// The baked blob stores only offsets. A stack tree view supplies the traversal API. The view is
// read only: a baked tree is never inserted into, so it has no parent array.
static inline b3DynamicTree b3MakeCompoundTree( const b3CompoundData* compound )
{
	int childCount = compound->capsuleCount + compound->hullCount + compound->meshCount + compound->sphereCount;
	b3DynamicTree tree = { 0 };
	tree.version = B3_DYNAMIC_TREE_VERSION;
	tree.nodes = (b3TreeNode*)( (uintptr_t)compound + (uintptr_t)compound->nodeOffset );
	tree.proxies = (b3TreeProxy*)( (uintptr_t)compound + (uintptr_t)compound->proxyOffset );
	tree.nodeEnd = compound->nodeCount;
	tree.nodeCapacity = compound->nodeCount;
	tree.pairFreeList = B3_NULL_INDEX;
	tree.proxyCount = childCount;
	tree.proxyCapacity = childCount;
	tree.proxyFreeList = B3_NULL_INDEX;
	tree.dfsOrdered = true;
	return tree;
}

b3TOIOutput b3CompoundTimeOfImpact( const b3CompoundData* compound, b3Transform transform, const b3ShapeProxy* proxy,
									const b3Sweep* sweep, b3Fixed maxFraction );

// Transforms a sweep for a compound child shape
b3Sweep b3MakeCompoundChildSweep( b3Transform compoundTransform, b3Transform childTransform );

int b3CollideMoverAndCompound( b3PlaneResult* planes, int capacity, const b3CompoundData* shape, const b3Capsule* mover );
