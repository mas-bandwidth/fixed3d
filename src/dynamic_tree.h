// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#define B3_TREE_STACK_SIZE 512

// Used to mark a node as being moved such that pairs need to be generated and the tree
// may need to be rebuilt. This mark comes from a few sources:
// - updating the transform and AABB in the solver from movement
// - a joint disabling collision
// - creating a proxy
// - setting the transform on a body

#define B3_MOVED_NODE ( 1u << 30 )
#define B3_LEAF_NODE ( 1u << 31 )
#define B3_NODE_INDEX_MASK ( 0xFFFFFFFFu & ~( B3_MOVED_NODE | B3_LEAF_NODE ) )

// Used to indicate empty nodes. It also passes as a leaf so that internal node
// processing will naturally skip it.
#define B3_EMPTY_NODE ( B3_NODE_INDEX_MASK | B3_LEAF_NODE )

#define B3_ROOT_NODE 0

B3_FORCE_INLINE bool b3IsLeaf( const b3TreeNode* node )
{
	return ( node->flagIndex & B3_LEAF_NODE ) == B3_LEAF_NODE;
}

B3_FORCE_INLINE bool b3IsNodeMoved( const b3TreeNode* node )
{
	return ( node->flagIndex & B3_MOVED_NODE ) == B3_MOVED_NODE;
}

B3_FORCE_INLINE bool b3IsEmptyNode( const b3TreeNode* node )
{
	return node->flagIndex == B3_EMPTY_NODE;
}

B3_FORCE_INLINE int b3GetLeftChild( const b3TreeNode* node )
{
	return (int)( node->flagIndex & B3_NODE_INDEX_MASK );
}

B3_FORCE_INLINE int b3GetProxyId( const b3TreeNode* node )
{
	return (int)( node->flagIndex & B3_NODE_INDEX_MASK );
}

B3_FORCE_INLINE int b3GetRootPair( const b3TreeNode* nodes )
{
	const b3TreeNode* root = nodes + B3_ROOT_NODE;
	return b3IsLeaf( root ) ? B3_ROOT_NODE : b3GetLeftChild( root );
}

// Upstream gives the empty node infinite inverted bounds. Fixed point has no infinity, so the
// empty node takes the extreme representable values of the bound type, inverted: lower is the
// largest coordinate and upper the smallest, so no box overlaps it. In ludicrous mode the bounds
// are 128-bit and the extremes are the 128-bit ones, because a wide coordinate can pass the
// 64-bit range. The empty node is only compared and unioned, never subtracted or measured, so
// the extremes cannot overflow (see the sentinel limits in tla/README.md).
#if defined( BOX3D_LUDICROUS_MODE )
#define B3_EMPTY_BOUND_MAX fixInt128Make( 0x7FFFFFFFFFFFFFFFull, 0xFFFFFFFFFFFFFFFFull )
#define B3_EMPTY_BOUND_MIN fixInt128Make( 0x8000000000000000ull, 0x0000000000000001ull )
#else
#define B3_EMPTY_BOUND_MAX B3_FIXED_MAX
#define B3_EMPTY_BOUND_MIN B3_FIXED_MIN
#endif

B3_FORCE_INLINE b3TreeNode b3MakeEmptyNode( void )
{
	b3TreeNode node = { 0 };
	node.aabb.lowerBound.x = B3_EMPTY_BOUND_MAX;
	node.aabb.lowerBound.y = B3_EMPTY_BOUND_MAX;
	node.aabb.lowerBound.z = B3_EMPTY_BOUND_MAX;
	node.aabb.upperBound.x = B3_EMPTY_BOUND_MIN;
	node.aabb.upperBound.y = B3_EMPTY_BOUND_MIN;
	node.aabb.upperBound.z = B3_EMPTY_BOUND_MIN;
	node.flagIndex = B3_EMPTY_NODE;
	node.height = 0;
	return node;
}

static inline bool b3HasTreeMoved( const b3DynamicTree* tree )
{
	return b3IsNodeMoved( tree->nodes + B3_ROOT_NODE );
}

static inline bool b3NeedsRebuild( const b3DynamicTree* tree )
{
	return b3IsNodeMoved( tree->nodes + B3_ROOT_NODE ) || tree->dfsOrdered == false;
}

int b3CreateTreeProxyInternal( b3DynamicTree* tree, b3AABB aabb, uint64_t categoryBits, uint64_t userData, bool markMoved );
void b3DynamicTree_MoveProxyInternal( b3DynamicTree* tree, int proxyId, b3AABB aabb, bool markMoved );

void b3DynamicTree_MarkProxyMovedSerial( b3DynamicTree* tree, int proxyId );
void b3DynamicTree_MarkProxyMoved( b3DynamicTree* tree, int proxyId, b3AABB aabb );
void b3DynamicTree_ClearMoved( b3DynamicTree* tree );
int b3DynamicTree_GatherMovedProxies( const b3DynamicTree* tree, int* proxyIds );
void b3DynamicTree_Refit( b3DynamicTree* tree );
