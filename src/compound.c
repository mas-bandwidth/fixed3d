// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "compound.h"

#include "hull.h"
#include "math_internal.h"
#include "shape.h"

#include "box3d/base.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/types.h"

#include <stdint.h>
#include <string.h>

typedef struct b3SharedHull
{
	const b3HullData* hull;
	int hullOffset;
} b3SharedHull;

typedef struct b3SharedMesh
{
	const b3MeshData* meshData;
	int meshOffset;
} b3SharedMesh;

typedef struct b3HullInstance
{
	b3Transform transform;
	uint32_t hullOffset;
	uint16_t materialIndex;
	uint16_t padding;
} b3HullInstance;

_Static_assert( sizeof( b3HullInstance ) == sizeof( b3Transform ) + sizeof( uint32_t ) + 2 * sizeof( uint16_t ),
				"review padding" );

typedef struct b3MeshInstance
{
	b3Transform transform;
	b3Vec3 scale;
	uint32_t meshOffset;
	uint32_t materialOffset;
	uint32_t materialCount;
} b3MeshInstance;

// Fixed-point layout: b3Vec3 is 24 bytes (align 8), so b3Capsule is 56 and b3Sphere is 32.
// The uint16_t materialIndex + uint16_t padding fill the old 4-byte int slot, leaving 4 bytes
// of tail padding to each 8-aligned struct size; the compound blob is memset before these are
// written field-by-field, so the tail bytes are deterministic. LUDICROUS_MODE widens positions
// (b3Pos/b3AABB), not b3Vec3, so these sizes are identical in both configs.
_Static_assert( sizeof( b3CompoundCapsule ) == 64, "review padding" );
_Static_assert( sizeof( b3CompoundSphere ) == 40, "review padding" );
#if defined( BOX3D_LUDICROUS_MODE )
_Static_assert( sizeof( b3CompoundData ) == 176, "review pointer-free compound layout" );
#else
_Static_assert( sizeof( b3CompoundData ) == 128, "review pointer-free compound layout" );
#endif

static inline b3TreeNode* b3GetCompoundNodes( b3CompoundData* compound )
{
	if ( compound->nodeOffset == 0 )
	{
		return NULL;
	}

	return (b3TreeNode*)( (intptr_t)compound + compound->nodeOffset );
}

const b3SurfaceMaterial* b3GetCompoundMaterials( const b3CompoundData* compound )
{
	if ( compound->materialOffset == 0 )
	{
		return NULL;
	}

	return (b3SurfaceMaterial*)( (intptr_t)compound + compound->materialOffset );
}

b3CompoundCapsule b3GetCompoundCapsule( const b3CompoundData* compound, int index )
{
	B3_ASSERT( 0 <= index && index < compound->capsuleCount && compound->capsuleOffset > 0 );

	b3CompoundCapsule result = { 0 };
	if ( compound->capsuleOffset == 0 )
	{
		return result;
	}

	const b3CompoundCapsule* capsules = (const b3CompoundCapsule*)( (intptr_t)compound + compound->capsuleOffset );
	return capsules[index];
}

b3CompoundHull b3GetCompoundHull( const b3CompoundData* compound, int index )
{
	B3_ASSERT( 0 <= index && index < compound->hullCount && compound->hullOffset > 0 );

	b3CompoundHull result = { 0 };
	if ( compound->hullOffset == 0 )
	{
		return result;
	}

	const b3HullInstance* hullInstances = (const b3HullInstance*)( (intptr_t)compound + compound->hullOffset );
	uint32_t hullOffset = hullInstances[index].hullOffset;
	B3_ASSERT( hullOffset >= compound->hullOffset + compound->hullCount * sizeof( b3HullInstance ) );
	result.hull = (const b3HullData*)( (intptr_t)compound + hullOffset );
	result.transform = hullInstances[index].transform;
	result.materialIndex = hullInstances[index].materialIndex;
	return result;
}

b3CompoundMesh b3GetCompoundMesh( const b3CompoundData* compound, int index )
{
	B3_ASSERT( 0 <= index && index < compound->meshCount && compound->meshOffset > 0 );

	b3CompoundMesh result = { 0 };
	if ( compound->meshOffset == 0 )
	{
		return result;
	}

	const b3MeshInstance* meshInstances = (const b3MeshInstance*)( (intptr_t)compound + compound->meshOffset );
	uint32_t meshOffset = meshInstances[index].meshOffset;
	B3_ASSERT( meshOffset >= compound->meshOffset + compound->meshCount * sizeof( b3HullInstance ) );
	result.meshData = (const b3MeshData*)( (intptr_t)compound + meshOffset );
	result.transform = meshInstances[index].transform;
	result.scale = meshInstances[index].scale;
	result.materialIndices = (const uint16_t*)( (intptr_t)compound + meshInstances[index].materialOffset );
	result.materialCount = (int)meshInstances[index].materialCount;
	return result;
}

b3CompoundSphere b3GetCompoundSphere( const b3CompoundData* compound, int index )
{
	B3_ASSERT( 0 <= index && index < compound->sphereCount && compound->sphereOffset > 0 );

	b3CompoundSphere result = { 0 };
	if ( compound->sphereOffset == 0 )
	{
		return result;
	}

	const b3CompoundSphere* spheres = (const b3CompoundSphere*)( (intptr_t)compound + compound->sphereOffset );
	return spheres[index];
}

b3ChildShape b3GetCompoundChild( const b3CompoundData* compound, int childIndex )
{
	// Capsule?
	if ( 0 <= childIndex && childIndex < compound->capsuleCount )
	{
		B3_ASSERT( compound->capsuleOffset > 0 );
		const b3CompoundCapsule* capsules = (const b3CompoundCapsule*)( (intptr_t)compound + compound->capsuleOffset );
		return (b3ChildShape){
			.capsule = capsules[childIndex].capsule,
			.transform = b3Transform_identity,
			.materialIndices = &capsules[childIndex].materialIndex,
			.materialCount = 1,
			.type = b3_capsuleShape,
		};
	}
	childIndex -= compound->capsuleCount;

	// Hull?
	if ( 0 <= childIndex && childIndex < compound->hullCount )
	{
		B3_ASSERT( compound->hullOffset > 0 );
		const b3HullInstance* hullInstances = (const b3HullInstance*)( (intptr_t)compound + compound->hullOffset );
		uint32_t hullOffset = hullInstances[childIndex].hullOffset;
		return (b3ChildShape){
			.hull = (const b3HullData*)( (intptr_t)compound + hullOffset ),
			.transform = hullInstances[childIndex].transform,
			.materialIndices = &hullInstances[childIndex].materialIndex,
			.materialCount = 1,
			.type = b3_hullShape,
		};
	}
	childIndex -= compound->hullCount;

	// Mesh?
	if ( 0 <= childIndex && childIndex < compound->meshCount )
	{
		b3CompoundMesh compoundMesh = b3GetCompoundMesh( compound, childIndex );

		return (b3ChildShape){
			.mesh =
				{
					.data = compoundMesh.meshData,
					.scale = compoundMesh.scale,
				},
			.transform = compoundMesh.transform,
			.materialIndices = compoundMesh.materialIndices,
			.materialCount = compoundMesh.materialCount,
			.type = b3_meshShape,
		};
	}
	childIndex -= compound->meshCount;

	B3_ASSERT( 0 <= childIndex && childIndex < compound->sphereCount );

	// Sphere?
	{
		B3_ASSERT( compound->sphereOffset > 0 );
		const b3CompoundSphere* spheres = (const b3CompoundSphere*)( (intptr_t)compound + compound->sphereOffset );
		return (b3ChildShape){
			.sphere = spheres[childIndex].sphere,
			.transform = b3Transform_identity,
			.materialIndices = &spheres[childIndex].materialIndex,
			.materialCount = 1,
			.type = b3_sphereShape,
		};
	}
}

static inline size_t vt_wyhash( const void* key, size_t len );

static inline uint64_t b3HashMesh( const b3MeshData* mesh )
{
	return vt_wyhash( mesh, mesh->byteCount );
}

static bool b3CompareMeshes( const b3MeshData* mesh1, const b3MeshData* mesh2 )
{
	if ( mesh1 == mesh2 )
	{
		return true;
	}

	if ( mesh1->byteCount != mesh2->byteCount )
	{
		return false;
	}

	int result = memcmp( mesh1, mesh2, mesh1->byteCount );
	return result == 0;
}

#define NAME b3MeshMap
#define KEY_TY const b3MeshData*
#define VAL_TY int
#define HASH_FN b3HashMesh
#define CMPR_FN b3CompareMeshes
#define MALLOC_FN b3Alloc
#define FREE_FN b3Free
#include "verstable.h"

// Fixed-point layout: 3 x b3Fixed + b3Vec3 + uint64 + 2 x uint32 = 64 bytes, no implicit
// padding left (the explicit padding field fills the old 4 tail bytes). Upstream float
// asserts 40 here.
_Static_assert( sizeof( b3SurfaceMaterial ) == 64, "review padding" );

// Keys must be staged with b3StageMaterial (shape.h) so the padding bytes swept here are
// zero. The staged array is also what lands in the compound blob, whose raw bytes
// b3RecInternCompound hashes.
static inline uint64_t b3HashMaterial( const b3SurfaceMaterial* material )
{
	B3_ASSERT( material->padding == 0 );

	return vt_wyhash( material, sizeof( b3SurfaceMaterial ) );
}

static bool b3CompareMaterials( const b3SurfaceMaterial* mat1, const b3SurfaceMaterial* mat2 )
{
	B3_ASSERT( mat1->padding == 0 && mat2->padding == 0 );

	if ( mat1 == mat2 )
	{
		return true;
	}

	int result = memcmp( mat1, mat2, sizeof( b3SurfaceMaterial ) );
	return result == 0;
}

#define NAME b3MaterialMap
#define KEY_TY const b3SurfaceMaterial*
#define VAL_TY int
#define HASH_FN b3HashMaterial
#define CMPR_FN b3CompareMaterials
#define MALLOC_FN b3Alloc
#define FREE_FN b3Free
#include "verstable.h"

b3CompoundData* b3CreateCompound( const b3CompoundDef* def )
{
	int convexCount = def->capsuleCount + def->hullCount + def->sphereCount;
	int shapeCount = convexCount + def->meshCount;

	if ( shapeCount >= B3_MAX_CHILD_SHAPES )
	{
		B3_ASSERT( false );
		return NULL;
	}

	b3DynamicTree tree = b3DynamicTree_Create( shapeCount );

	int childIndex = 0;

	// Instances
	int capsuleCount = def->capsuleCount;
	b3CompoundCapsule* capsuleInstances = b3AllocZeroed( capsuleCount * sizeof( b3CompoundCapsule ) );
	int hullCount = def->hullCount;
	b3HullInstance* hullInstances = b3AllocZeroed( hullCount * sizeof( b3HullInstance ) );
	int meshCount = def->meshCount;
	b3MeshInstance* meshInstances = b3AllocZeroed( meshCount * sizeof( b3MeshInstance ) );
	int sphereCount = def->sphereCount;
	b3CompoundSphere* sphereInstances = b3AllocZeroed( sphereCount * sizeof( b3CompoundSphere ) );

	// Determine material capacity
	int materialCapacity = convexCount;
	for ( int i = 0; i < def->meshCount; ++i )
	{
		B3_ASSERT( def->meshes[i].materialCount > 0 );
		materialCapacity += def->meshes[i].materialCount;
	}

	// Material map for convex material sharing. Mesh materials are not shared for simplicity.
	b3MaterialMap materialMap;
	b3MaterialMap_init( &materialMap );
	b3MaterialMap_reserve( &materialMap, materialCapacity );
	b3SurfaceMaterial* materials = b3AllocZeroed( materialCapacity * sizeof( b3SurfaceMaterial ) );
	int materialCount = 0;

	int meshMaterialTotal = materialCapacity - convexCount;
	uint16_t* meshMaterialStaging = b3AllocZeroed( meshMaterialTotal * sizeof( uint16_t ) );
	int meshMaterialCursor = 0;

	for ( int i = 0; i < def->capsuleCount; ++i )
	{
		const b3CompoundCapsuleDef* capsuleDef = def->capsules + i;
		capsuleInstances[i].capsule = capsuleDef->capsule;

		// Look for an existing material. The candidate is staged into the next
		// materials slot; if it is new, that slot is already its final home.
		b3SurfaceMaterial* candidate = materials + materialCount;
		b3StageMaterial( candidate, &capsuleDef->material );
		b3MaterialMap_itr materialItr = b3MaterialMap_get_or_insert( &materialMap, candidate, materialCount );

		// Get the shared material index
		int materialIndex = materialItr.data->val;
		capsuleInstances[i].materialIndex = (uint16_t)materialIndex;

		// Is this a new material?
		if ( materialIndex == materialCount )
		{
			materialCount += 1;
		}

		b3AABB aabb = b3ComputeCapsuleAABB( &capsuleDef->capsule, b3Transform_identity );
		b3DynamicTree_CreateProxy( &tree, aabb, ~0ull, childIndex );
		childIndex += 1;
	}

	// Hulls
	b3SharedHull* sharedHulls = b3AllocZeroed( hullCount * sizeof( b3SharedHull ) );
	int sharedHullCount = 0;

	if ( hullCount > 0 )
	{
		b3HullMap hullMap;
		b3HullMap_init( &hullMap );
		b3HullMap_reserve( &hullMap, hullCount );

		for ( int i = 0; i < hullCount; ++i )
		{
			const b3CompoundHullDef* hullDef = def->hulls + i;
			const b3HullData* hull = hullDef->hull;
			b3AABB aabb = b3ComputeHullAABB( hull, hullDef->transform );
			b3DynamicTree_CreateProxy( &tree, aabb, ~0ull, childIndex );
			childIndex += 1;

			// Look for an existing material (staged, see the capsule loop)
			b3SurfaceMaterial* candidate = materials + materialCount;
			b3StageMaterial( candidate, &hullDef->material );
			b3MaterialMap_itr materialItr = b3MaterialMap_get_or_insert( &materialMap, candidate, materialCount );

			// Get the shared material index
			int materialIndex = materialItr.data->val;
			hullInstances[i].materialIndex = (uint16_t)materialIndex;

			// Is this a new material?
			if ( materialIndex == materialCount )
			{
				materialCount += 1;
			}

			hullInstances[i].transform = hullDef->transform;

			// Look for an existing matching hull
			b3HullMap_itr itr = b3HullMap_get_or_insert( &hullMap, hull, sharedHullCount );

			// Get the unique index for this hull
			int sharedHullIndex = itr.data->val;

			// The offset isn't known yet, so store the index of the shared hull
			hullInstances[i].hullOffset = sharedHullIndex;

			// Is this a new hull?
			if ( sharedHullIndex == sharedHullCount )
			{
				// Create a shared hull. The offset is determined below.
				sharedHulls[sharedHullIndex].hull = hull;
				sharedHulls[sharedHullIndex].hullOffset = B3_NULL_INDEX;
				sharedHullCount += 1;
			}
		}

		b3HullMap_cleanup( &hullMap );
	}

	// Meshes
	b3SharedMesh* sharedMeshes = b3AllocZeroed( meshCount * sizeof( b3SharedMesh ) );
	int sharedMeshCount = 0;

	if ( meshCount > 0 )
	{
		b3MeshMap meshMap;
		b3MeshMap_init( &meshMap );
		b3MeshMap_reserve( &meshMap, meshCount );

		for ( int i = 0; i < meshCount; ++i )
		{
			const b3CompoundMeshDef* meshDef = def->meshes + i;

			const b3MeshData* meshData = meshDef->meshData;
			b3AABB aabb = b3ComputeMeshAABB( meshData, meshDef->transform, meshDef->scale );
			b3DynamicTree_CreateProxy( &tree, aabb, ~0ull, childIndex );
			childIndex += 1;

			// No effort to share mesh materials. It would be easier to do if the number of materials was limited.
			B3_ASSERT( meshData->materialCount == meshDef->materialCount );

			int meshMaterialStart = meshMaterialCursor;

			for ( int j = 0; j < meshDef->materialCount; ++j )
			{
				// Look for an existing material (staged, see the capsule loop)
				b3SurfaceMaterial* candidate = materials + materialCount;
				b3StageMaterial( candidate, &meshDef->materials[j] );
				b3MaterialMap_itr materialItr = b3MaterialMap_get_or_insert( &materialMap, candidate, materialCount );

				// Get the shared material index
				int materialIndex = materialItr.data->val;
				meshMaterialStaging[meshMaterialCursor] = (uint16_t)materialIndex;
				meshMaterialCursor += 1;

				// Is this a new material?
				if ( materialIndex == materialCount )
				{
					materialCount += 1;
				}
			}

			// Look for an existing matching mesh
			b3MeshMap_itr itr = b3MeshMap_get_or_insert( &meshMap, meshData, sharedMeshCount );

			// Get the shared mesh index
			int sharedMeshIndex = itr.data->val;

			// Create mesh instance
			meshInstances[i].transform = def->meshes[i].transform;
			meshInstances[i].scale = def->meshes[i].scale;

			meshInstances[i].materialOffset = (uint32_t)meshMaterialStart;
			meshInstances[i].materialCount = (uint32_t)meshDef->materialCount;

			// The offset isn't known yet, so store the index of the shared mesh
			meshInstances[i].meshOffset = sharedMeshIndex;

			// Is this a new mesh?
			if ( sharedMeshIndex == sharedMeshCount )
			{
				// Create a shared mesh. The offset is determined below.
				sharedMeshes[sharedMeshIndex].meshData = meshData;
				sharedMeshes[sharedMeshIndex].meshOffset = B3_NULL_INDEX;
				sharedMeshCount += 1;
			}
		}

		b3MeshMap_cleanup( &meshMap );
	}

	// Spheres
	for ( int i = 0; i < def->sphereCount; ++i )
	{
		const b3CompoundSphereDef* sphereDef = def->spheres + i;
		sphereInstances[i].sphere = sphereDef->sphere;

		// Look for an existing material (staged, see the capsule loop)
		b3SurfaceMaterial* candidate = materials + materialCount;
		b3StageMaterial( candidate, &sphereDef->material );
		b3MaterialMap_itr materialItr = b3MaterialMap_get_or_insert( &materialMap, candidate, materialCount );

		// Get the shared material index
		int materialIndex = materialItr.data->val;
		sphereInstances[i].materialIndex = (uint16_t)materialIndex;

		// Is this a new material?
		if ( materialIndex == materialCount )
		{
			materialCount += 1;
		}

		b3AABB aabb = b3ComputeSphereAABB( &sphereDef->sphere, b3Transform_identity );
		b3DynamicTree_CreateProxy( &tree, aabb, ~0ull, childIndex );
		childIndex += 1;
	}

	B3_ASSERT( materialCount <= materialCapacity );
	B3_ASSERT( materialCount <= UINT16_MAX );
	B3_ASSERT( tree.nodeCount > 0 );

	b3DynamicTree_Rebuild( &tree, true );

	// Each section offset rounds up to the alignment of the element type it holds.
	// In the narrow builds every rounding is a no-op (types need <= 8, offsets are
	// already 8-aligned), so the historical layout is unchanged byte for byte. In
	// ludicrous mode b3TreeNode / b3HullData / b3MeshData embed 128-bit AABB bounds
	// and need 16; the padding bytes stay zero from the blob memset, so the content
	// hash stays deterministic.
	int byteCount = sizeof( b3CompoundData );

	// Tree nodes - todo 64 byte alignment
	int nodeOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3TreeNode ) );
	byteCount = nodeOffset + tree.nodeCapacity * (int)sizeof( b3TreeNode );

	int materialOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3SurfaceMaterial ) );
	byteCount = materialOffset + materialCount * (int)sizeof( b3SurfaceMaterial );

	int capsuleOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3CompoundCapsule ) );
	byteCount = capsuleOffset + def->capsuleCount * (int)sizeof( b3CompoundCapsule );

	// Hull data layout has another level of indirection to allow for tight data packing
	// 1. hull instance array : hull count array of b3HullInstance with individual hull transforms and offsets
	// 2. heterogeneous array of shared hull data : each shared hull can have a different byte count, so direct indexing is not
	// possible
	int hullArrayOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3HullInstance ) );

	// Array of hull instances
	byteCount = hullArrayOffset + hullCount * (int)sizeof( b3HullInstance );

	// Packed shared hull blobs
	for ( int i = 0; i < sharedHullCount; ++i )
	{
		sharedHulls[i].hullOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3HullData ) );
		byteCount = sharedHulls[i].hullOffset + sharedHulls[i].hull->byteCount;
	}

	// Mesh data layout has another level of indirection to allow for tight data packing
	// 1. mesh instance array : mesh count array of b3MeshInstance with individual mesh transform, scale, and offset
	// 2. heterogeneous array of shared mesh data : each shared mesh can have a different byte count, so direct indexing is not
	// possible
	int meshArrayOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3MeshInstance ) );

	// Array of mesh instances
	byteCount = meshArrayOffset + meshCount * (int)sizeof( b3MeshInstance );

	int meshMaterialMapOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( uint16_t ) );
	byteCount = meshMaterialMapOffset + meshMaterialTotal * (int)sizeof( uint16_t );

	// Packed shared mesh blobs
	for ( int i = 0; i < sharedMeshCount; ++i )
	{
		sharedMeshes[i].meshOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3MeshData ) );
		byteCount = sharedMeshes[i].meshOffset + sharedMeshes[i].meshData->byteCount;
	}

	int sphereOffset = (int)b3AlignUp( (size_t)byteCount, _Alignof( b3CompoundSphere ) );
	byteCount = sphereOffset + def->sphereCount * (int)sizeof( b3CompoundSphere );

	b3CompoundData* compound = b3Alloc( byteCount );
	memset( compound, 0, byteCount );

	compound->version = B3_COMPOUND_VERSION;
	compound->byteCount = byteCount;
	compound->nodeOffset = nodeOffset;

	compound->bounds = b3DynamicTree_GetRootBounds( &tree );
	compound->treeHeight = b3DynamicTree_GetHeight( &tree );
	compound->nodeCount = tree.nodeCapacity;
	compound->root = tree.root;

	compound->materialOffset = materialOffset;
	compound->materialCount = materialCount;
	compound->capsuleOffset = capsuleOffset;
	compound->capsuleCount = capsuleCount;
	compound->hullOffset = hullArrayOffset;
	compound->hullCount = hullCount;
	compound->meshOffset = meshArrayOffset;
	compound->meshCount = meshCount;
	compound->sphereOffset = sphereOffset;
	compound->sphereCount = sphereCount;

	// Tree nodes
	b3TreeNode* nodes = b3GetCompoundNodes( compound );
	memcpy( nodes, tree.nodes, tree.nodeCapacity * sizeof( b3TreeNode ) );

	// Materials
	B3_ASSERT( materialCount > 0 );
	b3SurfaceMaterial* destinationMaterials = (b3SurfaceMaterial*)( (intptr_t)compound + compound->materialOffset );
	if ( materials != NULL )
	{
		memcpy( destinationMaterials, materials, materialCount * sizeof( b3SurfaceMaterial ) );
	}

	// Capsules
	if ( def->capsuleCount > 0 )
	{
		B3_ASSERT( compound->capsuleOffset > 0 );
		b3CompoundCapsule* capsules = (b3CompoundCapsule*)( (intptr_t)compound + compound->capsuleOffset );
		memcpy( capsules, capsuleInstances, capsuleCount * sizeof( b3CompoundCapsule ) );
	}

	// Hulls
	for ( int i = 0; i < hullCount; ++i )
	{
		// Fix up offsets
		int sharedIndex = hullInstances[i].hullOffset;
		B3_ASSERT( 0 <= sharedIndex && sharedIndex < sharedHullCount );
		hullInstances[i].hullOffset = sharedHulls[sharedIndex].hullOffset;
	}

	b3HullInstance* destinationHullInstances = (b3HullInstance*)( (intptr_t)compound + hullArrayOffset );
	memcpy( destinationHullInstances, hullInstances, hullCount * sizeof( b3HullInstance ) );

	for ( int i = 0; i < sharedHullCount; ++i )
	{
		int offset = sharedHulls[i].hullOffset;
		b3HullData* destinationHull = (b3HullData*)( (intptr_t)compound + offset );
		memcpy( destinationHull, sharedHulls[i].hull, sharedHulls[i].hull->byteCount );
	}

	compound->sharedHullCount = sharedHullCount;

	// Meshes
	for ( int i = 0; i < meshCount; ++i )
	{
		// Fix up offsets
		int sharedIndex = meshInstances[i].meshOffset;
		B3_ASSERT( 0 <= sharedIndex && sharedIndex < sharedMeshCount );
		meshInstances[i].meshOffset = sharedMeshes[sharedIndex].meshOffset;

		uint32_t materialStart = meshInstances[i].materialOffset;
		meshInstances[i].materialOffset = (uint32_t)meshMaterialMapOffset + materialStart * (uint32_t)sizeof( uint16_t );
	}

	b3MeshInstance* destinationMeshInstances = (b3MeshInstance*)( (intptr_t)compound + meshArrayOffset );
	memcpy( destinationMeshInstances, meshInstances, meshCount * sizeof( b3MeshInstance ) );

	if ( meshMaterialTotal > 0 )
	{
		uint16_t* destinationMeshMaterials = (uint16_t*)( (intptr_t)compound + meshMaterialMapOffset );
		memcpy( destinationMeshMaterials, meshMaterialStaging, meshMaterialTotal * sizeof( uint16_t ) );
	}

	for ( int i = 0; i < sharedMeshCount; ++i )
	{
		int offset = sharedMeshes[i].meshOffset;
		b3MeshData* destinationMesh = (b3MeshData*)( (intptr_t)compound + offset );
		memcpy( destinationMesh, sharedMeshes[i].meshData, sharedMeshes[i].meshData->byteCount );
	}

	compound->sharedMeshCount = sharedMeshCount;

	// Spheres
	if ( def->sphereCount > 0 )
	{
		B3_ASSERT( compound->sphereOffset > 0 );
		b3CompoundSphere* spheres = (b3CompoundSphere*)( (intptr_t)compound + compound->sphereOffset );
		memcpy( spheres, sphereInstances, sphereCount * sizeof( b3CompoundSphere ) );
	}

	b3MaterialMap_cleanup( &materialMap );
	b3Free( sharedHulls, hullCount * sizeof( b3SharedHull ) );
	b3Free( sharedMeshes, meshCount * sizeof( b3SharedMesh ) );
	b3Free( capsuleInstances, capsuleCount * sizeof( b3CompoundCapsule ) );
	b3Free( hullInstances, hullCount * sizeof( b3HullInstance ) );
	b3Free( meshInstances, meshCount * sizeof( b3MeshInstance ) );
	b3Free( sphereInstances, sphereCount * sizeof( b3CompoundSphere ) );
	b3Free( materials, materialCapacity * sizeof( b3SurfaceMaterial ) );
	b3Free( meshMaterialStaging, meshMaterialTotal * sizeof( uint16_t ) );
	b3DynamicTree_Destroy( &tree );

	return compound;
}

void b3DestroyCompound( b3CompoundData* compound )
{
	b3Free( compound, compound->byteCount );
}

static bool b3CompoundRegion( int offset, int count, size_t elementSize, size_t alignment, int64_t previousEnd, int byteCount,
							 int64_t* end )
{
	if ( alignment < 8 )
	{
		alignment = 8;
	}
	if ( offset < previousEnd || count < 0 || ( (size_t)offset & ( alignment - 1 ) ) != 0 )
	{
		return false;
	}
	uint64_t start = (uint64_t)offset;
	if ( elementSize != 0 && (uint64_t)count > ( UINT64_MAX - start ) / elementSize )
	{
		return false;
	}
	uint64_t limit = start + (uint64_t)count * elementSize;
	if ( limit > (uint64_t)byteCount )
	{
		return false;
	}
	*end = (int64_t)limit;
	return true;
}

static bool b3CompoundHullValid( const b3HullData* hull )
{
	if ( hull->version != B3_HULL_VERSION || hull->vertexCount <= 0 || hull->edgeCount <= 0 || hull->faceCount <= 0 )
	{
		return false;
	}
	if ( hull->vertexCount > UINT8_MAX + 1 || hull->edgeCount > UINT8_MAX + 1 || hull->faceCount > UINT8_MAX + 1 )
	{
		return false;
	}
	int64_t end = sizeof( b3HullData );
	if ( !( b3CompoundRegion( hull->vertexOffset, hull->vertexCount, sizeof( b3HullVertex ), _Alignof( b3HullVertex ), end,
							 hull->byteCount, &end ) &&
		   b3CompoundRegion( hull->pointOffset, hull->vertexCount, sizeof( b3Vec3 ), _Alignof( b3Vec3 ), end,
							 hull->byteCount, &end ) &&
		   b3CompoundRegion( hull->edgeOffset, hull->edgeCount, sizeof( b3HullHalfEdge ), _Alignof( b3HullHalfEdge ), end,
							 hull->byteCount, &end ) &&
		   b3CompoundRegion( hull->faceOffset, hull->faceCount, sizeof( b3HullFace ), _Alignof( b3HullFace ), end,
							 hull->byteCount, &end ) &&
		   b3CompoundRegion( hull->planeOffset, hull->faceCount, sizeof( b3Plane ), _Alignof( b3Plane ), end,
							 hull->byteCount, &end ) ) )
	{
		return false;
	}
	const uint8_t* bytes = (const uint8_t*)hull;
	const b3HullVertex* vertices = (const b3HullVertex*)( bytes + hull->vertexOffset );
	const b3HullHalfEdge* edges = (const b3HullHalfEdge*)( bytes + hull->edgeOffset );
	const b3HullFace* faces = (const b3HullFace*)( bytes + hull->faceOffset );
	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		if ( vertices[i].edge >= hull->edgeCount )
		{
			return false;
		}
	}
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		if ( edges[i].next >= hull->edgeCount || edges[i].twin >= hull->edgeCount || edges[i].origin >= hull->vertexCount ||
			 edges[i].face >= hull->faceCount || edges[i].twin == i || edges[edges[i].twin].twin != i )
		{
			return false;
		}
	}
	bool seen[UINT8_MAX + 1] = { false };
	for ( int i = 0; i < hull->faceCount; ++i )
	{
		if ( faces[i].edge >= hull->edgeCount )
		{
			return false;
		}
		int edge = faces[i].edge;
		for ( int steps = 0; steps < hull->edgeCount; ++steps )
		{
			if ( seen[edge] || edges[edge].face != i )
			{
				return false;
			}
			seen[edge] = true;
			edge = edges[edge].next;
			if ( edge == faces[i].edge )
			{
				break;
			}
		}
		if ( edge != faces[i].edge )
		{
			return false;
		}
	}
	for ( int i = 0; i < hull->edgeCount; ++i )
	{
		if ( !seen[i] )
		{
			return false;
		}
	}
	return true;
}

// Mesh consumers keep a 256-entry traversal stack. A preorder parse enforces one
// reachable tree (no shared subtrees) and bounds the actual depth, not the header claim.
static bool b3CompoundMeshTreeValid( const b3MeshNode* nodes, int start, int end, int depth, int triangleCount )
{
	if ( start >= end || depth >= 256 )
	{
		return false;
	}
	const b3MeshNode* node = nodes + start;
	if ( node->data.asLeaf.type == 3 )
	{
		return start + 1 == end && node->data.asLeaf.triangleCount > 0 &&
			   (uint64_t)node->triangleOffset + node->data.asLeaf.triangleCount <= (uint64_t)triangleCount;
	}
	int64_t right = (int64_t)start + node->data.asNode.childOffset;
	if ( depth >= 255 || right <= start + 1 || right >= end )
	{
		return false;
	}
	return b3CompoundMeshTreeValid( nodes, start + 1, (int)right, depth + 1, triangleCount ) &&
		   b3CompoundMeshTreeValid( nodes, (int)right, end, depth + 1, triangleCount );
}

static bool b3CompoundMeshValid( const b3MeshData* mesh )
{
	if ( mesh->version != B3_MESH_VERSION || mesh->nodeCount <= 0 || mesh->vertexCount <= 0 || mesh->triangleCount <= 0 ||
		 mesh->materialCount <= 0 )
	{
		return false;
	}
	int64_t end = sizeof( b3MeshData );
	if ( !( b3CompoundRegion( mesh->nodeOffset, mesh->nodeCount, sizeof( b3MeshNode ), _Alignof( b3MeshNode ), end,
							 mesh->byteCount, &end ) &&
		   b3CompoundRegion( mesh->vertexOffset, mesh->vertexCount, sizeof( b3Vec3 ), _Alignof( b3Vec3 ), end,
							 mesh->byteCount, &end ) &&
		   b3CompoundRegion( mesh->triangleOffset, mesh->triangleCount, sizeof( b3MeshTriangle ), _Alignof( b3MeshTriangle ), end,
							 mesh->byteCount, &end ) &&
		   b3CompoundRegion( mesh->materialOffset, mesh->triangleCount, sizeof( uint8_t ), _Alignof( uint8_t ), end,
							 mesh->byteCount, &end ) &&
		   b3CompoundRegion( mesh->flagsOffset, mesh->triangleCount, sizeof( uint8_t ), _Alignof( uint8_t ), end,
							 mesh->byteCount, &end ) ) )
	{
		return false;
	}
	const uint8_t* bytes = (const uint8_t*)mesh;
	const b3MeshNode* nodes = (const b3MeshNode*)( bytes + mesh->nodeOffset );
	const b3MeshTriangle* triangles = (const b3MeshTriangle*)( bytes + mesh->triangleOffset );
	const uint8_t* materials = bytes + mesh->materialOffset;
	if ( !b3CompoundMeshTreeValid( nodes, 0, mesh->nodeCount, 0, mesh->triangleCount ) )
	{
		return false;
	}
	for ( int i = 0; i < mesh->triangleCount; ++i )
	{
		if ( triangles[i].index1 < 0 || triangles[i].index1 >= mesh->vertexCount || triangles[i].index2 < 0 ||
			 triangles[i].index2 >= mesh->vertexCount || triangles[i].index3 < 0 || triangles[i].index3 >= mesh->vertexCount ||
			 materials[i] >= mesh->materialCount )
		{
			return false;
		}
	}
	return true;
}

const b3CompoundData* b3ValidateCompound( const uint8_t* bytes, int byteCount )
{
	if ( bytes == NULL || ( (uintptr_t)bytes & ( _Alignof( b3CompoundData ) - 1 ) ) != 0 ||
		 byteCount < (int)sizeof( b3CompoundData ) )
	{
		return NULL;
	}
	const b3CompoundData* compound = (const b3CompoundData*)bytes;
	if ( compound->version != B3_COMPOUND_VERSION || compound->byteCount != byteCount || compound->padding != 0 || compound->nodeCount < 2 ||
		 compound->root < 0 || compound->root >= compound->nodeCount || compound->materialCount <= 0 )
	{
		return NULL;
	}
	int64_t end = sizeof( b3CompoundData );
	if ( !b3CompoundRegion( compound->nodeOffset, compound->nodeCount, sizeof( b3TreeNode ), _Alignof( b3TreeNode ), end,
						 byteCount, &end ) ||
		 !b3CompoundRegion( compound->materialOffset, compound->materialCount, sizeof( b3SurfaceMaterial ),
							 _Alignof( b3SurfaceMaterial ), end, byteCount, &end ) ||
		 !b3CompoundRegion( compound->capsuleOffset, compound->capsuleCount, sizeof( b3CompoundCapsule ),
							 _Alignof( b3CompoundCapsule ), end, byteCount, &end ) ||
		 !b3CompoundRegion( compound->hullOffset, compound->hullCount, sizeof( b3HullInstance ), _Alignof( b3HullInstance ),
							 end, byteCount, &end ) )
	{
		return NULL;
	}
	int64_t childCount = (int64_t)compound->capsuleCount + compound->hullCount + compound->meshCount + compound->sphereCount;
	const b3TreeNode* nodes = (const b3TreeNode*)( bytes + compound->nodeOffset );
	if ( childCount <= 0 || ( nodes[compound->root].flags & b3_allocatedNode ) == 0 )
	{
		return NULL;
	}
	for ( int i = 0; i < compound->nodeCount; ++i )
	{
		const b3TreeNode* node = nodes + i;
		if ( ( node->flags & b3_allocatedNode ) == 0 )
		{
			continue;
		}
		if ( node->flags & b3_leafNode )
		{
			if ( node->userData >= (uint64_t)childCount )
			{
				return NULL;
			}
		}
		else if ( node->children.child1 < 0 || node->children.child1 >= compound->nodeCount || node->children.child2 < 0 ||
				  node->children.child2 >= compound->nodeCount || node->children.child1 == node->children.child2 ||
				  ( nodes[node->children.child1].flags & b3_allocatedNode ) == 0 ||
				  ( nodes[node->children.child2].flags & b3_allocatedNode ) == 0 ||
				  nodes[node->children.child1].height >= node->height || nodes[node->children.child2].height >= node->height )
		{
			return NULL;
		}
	}
	const b3CompoundCapsule* capsules = (const b3CompoundCapsule*)( bytes + compound->capsuleOffset );
	for ( int i = 0; i < compound->capsuleCount; ++i )
	{
		if ( capsules[i].materialIndex >= compound->materialCount )
		{
			return NULL;
		}
	}
	// Shared hull blobs live between the hull instance array and the mesh instance array.
	if ( compound->meshOffset < end || ( compound->meshOffset & 7 ) != 0 )
	{
		return NULL;
	}
	if ( !b3CompoundRegion( compound->meshOffset, compound->meshCount, sizeof( b3MeshInstance ), _Alignof( b3MeshInstance ),
							 end, byteCount, &end ) )
	{
		return NULL;
	}
	int64_t meshArrayEnd = end;
	if ( compound->sphereOffset < end || ( compound->sphereOffset & 7 ) != 0 ||
		 !b3CompoundRegion( compound->sphereOffset, compound->sphereCount, sizeof( b3CompoundSphere ),
							 _Alignof( b3CompoundSphere ), end, byteCount, &end ) )
	{
		return NULL;
	}
	const b3HullInstance* hulls = (const b3HullInstance*)( bytes + compound->hullOffset );
	for ( int i = 0; i < compound->hullCount; ++i )
	{
		uint64_t offset = hulls[i].hullOffset;
		if ( hulls[i].materialIndex >= compound->materialCount ||
			 offset < (uint64_t)compound->hullOffset + (uint64_t)compound->hullCount * sizeof( b3HullInstance ) ||
			 offset > (uint64_t)compound->meshOffset || ( offset & ( _Alignof( b3HullData ) - 1 ) ) != 0 ||
			 offset + sizeof( b3HullData ) > (uint64_t)compound->meshOffset )
		{
			return NULL;
		}
		const b3HullData* hull = (const b3HullData*)( bytes + offset );
		if ( hull->byteCount < (int)sizeof( b3HullData ) || offset + (uint64_t)hull->byteCount > (uint64_t)compound->meshOffset ||
			 !b3CompoundHullValid( hull ) )
		{
			return NULL;
		}
	}
	const b3MeshInstance* meshes = (const b3MeshInstance*)( bytes + compound->meshOffset );
	uint64_t firstMeshBlob = (uint64_t)compound->sphereOffset;
	for ( int i = 0; i < compound->meshCount; ++i )
	{
		uint64_t offset = meshes[i].meshOffset;
		if ( offset < firstMeshBlob )
		{
			firstMeshBlob = offset;
		}
		if ( offset < (uint64_t)meshArrayEnd || offset + sizeof( b3MeshData ) > (uint64_t)compound->sphereOffset ||
			 ( offset & ( _Alignof( b3MeshData ) - 1 ) ) != 0 || meshes[i].materialCount == 0 ||
			 (uint64_t)meshes[i].materialOffset < (uint64_t)meshArrayEnd ||
			 ( meshes[i].materialOffset & ( _Alignof( uint16_t ) - 1 ) ) != 0 ||
			 (uint64_t)meshes[i].materialOffset + (uint64_t)meshes[i].materialCount * sizeof( uint16_t ) >
				 (uint64_t)compound->sphereOffset )
		{
			return NULL;
		}
		const b3MeshData* mesh = (const b3MeshData*)( bytes + offset );
		if ( mesh->byteCount < (int)sizeof( b3MeshData ) || offset + (uint64_t)mesh->byteCount > (uint64_t)compound->sphereOffset ||
			 meshes[i].materialCount < (uint32_t)mesh->materialCount ||
			 !b3CompoundMeshValid( mesh ) )
		{
			return NULL;
		}
		const uint16_t* map = (const uint16_t*)( bytes + meshes[i].materialOffset );
		for ( uint32_t j = 0; j < meshes[i].materialCount; ++j )
		{
			if ( map[j] >= compound->materialCount )
			{
				return NULL;
			}
		}
	}
	for ( int i = 0; i < compound->meshCount; ++i )
	{
		if ( (uint64_t)meshes[i].materialOffset + (uint64_t)meshes[i].materialCount * sizeof( uint16_t ) > firstMeshBlob )
		{
			return NULL;
		}
	}
	const b3CompoundSphere* spheres = (const b3CompoundSphere*)( bytes + compound->sphereOffset );
	for ( int i = 0; i < compound->sphereCount; ++i )
	{
		if ( spheres[i].materialIndex >= compound->materialCount )
		{
			return NULL;
		}
	}
	return compound;
}

b3AABB b3ComputeCompoundAABB( const b3CompoundData* shape, b3Transform transform )
{
	return b3AABB_Transform( transform, shape->bounds );
}

struct b3CompoundOverlapContext
{
	const b3CompoundData* compound;
	// transform of the compound
	b3Transform transform;
	b3ShapeProxy proxy;
	bool overlap;
};

static bool b3CompoundOverlapCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int childIndex = (int)userData;
	struct b3CompoundOverlapContext* overlapContext = context;
	b3ChildShape child = b3GetCompoundChild( overlapContext->compound, childIndex );

	b3Transform transform = b3MulTransforms( overlapContext->transform, child.transform );

	bool overlap = false;
	switch ( child.type )
	{
		case b3_capsuleShape:
			overlap = b3OverlapCapsule( &child.capsule, transform, &overlapContext->proxy );
			break;

		case b3_hullShape:
			overlap = b3OverlapHull( child.hull, transform, &overlapContext->proxy );
			break;

		case b3_meshShape:
			overlap = b3OverlapMesh( &child.mesh, transform, &overlapContext->proxy );
			break;

		case b3_sphereShape:
			overlap = b3OverlapSphere( &child.sphere, transform, &overlapContext->proxy );
			break;

		default:
			B3_ASSERT( false );
			break;
	}

	if ( overlap )
	{
		// Done
		overlapContext->overlap = true;
		return false;
	}

	// Continue the query if there is no overlap
	return true;
}

bool b3OverlapCompound( const b3CompoundData* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	struct b3CompoundOverlapContext context = {
		.compound = shape,
		.transform = shapeTransform,
		.proxy = *proxy,
		.overlap = false,
	};

	b3AABB aabb = { b3Vec3ToBound( proxy->points[0] ), b3Vec3ToBound( proxy->points[0] ) };
	for ( int i = 1; i < proxy->count; ++i )
	{
		aabb.lowerBound = b3Vec3ToBound( b3Min( b3BoundToVec3( aabb.lowerBound ), proxy->points[i] ) );
		aabb.upperBound = b3Vec3ToBound( b3Max( b3BoundToVec3( aabb.upperBound ), proxy->points[i] ) );
	}

	b3Vec3 r = { proxy->radius, proxy->radius, proxy->radius };
	aabb.lowerBound = b3Vec3ToBound( b3Sub( b3BoundToVec3( aabb.lowerBound ), r ) );
	aabb.upperBound = b3Vec3ToBound( b3Add( b3BoundToVec3( aabb.upperBound ), r ) );

	b3DynamicTree tree = b3MakeCompoundTree( shape );
	(void)b3DynamicTree_Query( &tree, aabb, ~0ull, false, b3CompoundOverlapCallback, &context );

	return context.overlap;
}

struct b3CompoundCastContext
{
	const b3CompoundData* compound;
	b3CastOutput* output;
	// origin of the shape cast, the box cast callback only carries the advancing fraction
	const b3ShapeCastInput* shapeInput;
};

static b3Fixed b3CompoundRayCastCallback( const b3RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	struct b3CompoundCastContext* castContext = context;
	const b3CompoundData* compound = castContext->compound;

	int childIndex = (int)userData;

	b3ChildShape child = b3GetCompoundChild( compound, childIndex );

	b3RayCastInput localInput = *input;
	localInput.origin = b3InvTransformPoint( child.transform, input->origin );
	localInput.translation = b3InvRotateVector( child.transform.q, input->translation );

	b3CastOutput output = { 0 };

	switch ( child.type )
	{
		case b3_capsuleShape:
			output = b3RayCastCapsule( &child.capsule, &localInput );
			output.materialIndex = child.materialIndices[0];
			break;

		case b3_hullShape:
			output = b3RayCastHull( child.hull, &localInput );
			output.materialIndex = child.materialIndices[0];
			break;

		case b3_meshShape:
		{
			output = b3RayCastMesh( &child.mesh, &localInput );
			B3_ASSERT( 0 <= output.materialIndex && output.materialIndex < child.materialCount );
			output.materialIndex = child.materialIndices[output.materialIndex];
		}
		break;

		case b3_sphereShape:
			output = b3RayCastSphere( &child.sphere, &localInput );
			output.materialIndex = child.materialIndices[0];
			break;

		default:
			B3_ASSERT( false );
			break;
	}

	if ( output.hit )
	{
		output.point = b3TransformPoint( child.transform, output.point );
		output.normal = b3RotateVector( child.transform.q, output.normal );
		output.childIndex = childIndex;
		*castContext->output = output;
		return output.fraction;
	}

	return input->maxFraction;
}

b3CastOutput b3RayCastCompound( const b3CompoundData* shape, const b3RayCastInput* input )
{
	b3CastOutput result = { 0 };

	struct b3CompoundCastContext context = {
		.compound = shape,
		.output = &result,
	};
	b3DynamicTree tree = b3MakeCompoundTree( shape );
	(void)b3DynamicTree_RayCast( &tree, input, ~0ull, false, b3CompoundRayCastCallback, &context );
	return result;
}

static b3Fixed b3CompoundShapeCastCallback( const b3BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	struct b3CompoundCastContext* castContext = context;
	const b3CompoundData* compound = castContext->compound;
	const b3ShapeCastInput* shapeInput = castContext->shapeInput;

	int childIndex = (int)userData;

	b3ChildShape child = b3GetCompoundChild( compound, childIndex );

	// Rebuild from the carried shape cast input, taking only the advancing fraction from the tree
	b3ShapeCastInput localInput = *shapeInput;
	localInput.maxFraction = input->maxFraction;
	b3Vec3 localPoints[B3_MAX_SHAPE_CAST_POINTS];

	localInput.proxy.count = b3MinInt( shapeInput->proxy.count, B3_MAX_SHAPE_CAST_POINTS );

	b3Transform invTransform = b3InvertTransform( child.transform );
	b3Matrix3 R = b3MakeMatrixFromQuat( invTransform.q );

	for ( int i = 0; i < localInput.proxy.count; ++i )
	{
		localPoints[i] = b3Add( b3MulMV( R, shapeInput->proxy.points[i] ), invTransform.p );
	}

	localInput.proxy.points = localPoints;
	localInput.translation = b3MulMV( R, shapeInput->translation );

	b3CastOutput output = { 0 };

	switch ( child.type )
	{
		case b3_capsuleShape:
			output = b3ShapeCastCapsule( &child.capsule, &localInput );
			output.materialIndex = child.materialIndices[0];
			break;

		case b3_hullShape:
			output = b3ShapeCastHull( child.hull, &localInput );
			output.materialIndex = child.materialIndices[0];
			break;

		case b3_meshShape:
		{
			output = b3ShapeCastMesh( &child.mesh, &localInput );
			B3_ASSERT( 0 <= output.materialIndex && output.materialIndex < child.materialCount );
			output.materialIndex = child.materialIndices[output.materialIndex];
		}
		break;

		case b3_sphereShape:
			output = b3ShapeCastSphere( &child.sphere, &localInput );
			output.materialIndex = child.materialIndices[0];
			break;

		default:
			B3_ASSERT( false );
			break;
	}

	if ( output.hit )
	{
		output.point = b3TransformPoint( child.transform, output.point );
		output.normal = b3RotateVector( child.transform.q, output.normal );
		output.childIndex = childIndex;
		*castContext->output = output;
		return output.fraction;
	}

	return input->maxFraction;
}

b3CastOutput b3ShapeCastCompound( const b3CompoundData* shape, const b3ShapeCastInput* input )
{
	b3CastOutput result = { 0 };

	if ( input->proxy.count == 0 )
	{
		return result;
	}

	struct b3CompoundCastContext context = {
		.compound = shape,
		.output = &result,
		.shapeInput = input,
	};

	// The compound tree is in the compound local frame, so the proxy box needs no origin offset
	b3AABB box = b3MakeAABB( input->proxy.points, input->proxy.count, input->proxy.radius );
	b3BoxCastInput treeInput = { box, input->translation, input->maxFraction };
	b3DynamicTree tree = b3MakeCompoundTree( shape );
	(void)b3DynamicTree_BoxCast( &tree, &treeInput, ~0ull, false, b3CompoundShapeCastCallback, &context );
	return result;
}

struct b3CompoundQueryContext
{
	const b3CompoundData* compound;
	b3CompoundQueryFcn* fcn;
	void* userContext;
};

static bool TreeQueryCallbackFcn( int proxyId, uint64_t userData, void* treeContext )
{
	B3_UNUSED( proxyId );
	struct b3CompoundQueryContext* context = treeContext;
	return context->fcn( context->compound, (int)userData, context->userContext );
}

void b3QueryCompound( const b3CompoundData* compound, b3AABB aabb, b3CompoundQueryFcn* fcn, void* context )
{
	struct b3CompoundQueryContext compoundContext = {
		.compound = compound,
		.fcn = fcn,
		.userContext = context,
	};

	b3DynamicTree tree = b3MakeCompoundTree( compound );
	b3DynamicTree_Query( &tree, aabb, B3_DEFAULT_MASK_BITS, false, TreeQueryCallbackFcn, &compoundContext );
}

#if 0
struct b3CompoundImpactContext
{
	b3TOIInput toiInput;
	b3TOIOutput toiOutput;
	b3Transform compoundTransform;

	// Bounds local to compound
	b3AABB localSweepBoundsB;

	// Centroid of shape in body B local space
	b3Vec3 localCentroidB;
	b3Fixed fallbackRadius;
};

static bool b3CompoundTimeOfImpactFcn( const b3CompoundData* compound, int childIndex, void* context )
{
	b3CompoundImpactContext* toiContext = (b3CompoundImpactContext*)context;

	b3ChildShape child = b3GetCompoundChild( compound, childIndex );

	b3TOIOutput output = {0 };
	toiContext->toiInput.sweepA = b3MakeCompoundChildSweep( toiContext->compoundTransform, child.transform );

	switch ( child.type )
	{
		case b3_capsuleShape:
		{
			toiContext->toiInput.proxyA.points = &child.capsule.center1;
			toiContext->toiInput.proxyA.count = 2;
			toiContext->toiInput.proxyA.radius = child.capsule.radius;
			output = b3TimeOfImpact( &toiContext->toiInput );
		}
		break;

		case b3_hullShape:
		{
			toiContext->toiInput.proxyA.points = b3GetHullPoints( child.hull );
			toiContext->toiInput.proxyA.count = child.hull->vertexCount;
			toiContext->toiInput.proxyA.radius = 0.0f;
			output = b3TimeOfImpact( &toiContext->toiInput );
		}
		break;

		case b3_meshShape:
		{
			b3MeshImpactContext meshContext = {0};
			meshContext.toiInput = toiContext->toiInput;
			meshContext.isSensor = false;
			meshContext.localCentroidB = toiContext->localCentroidB;
			meshContext.fallbackRadius = toiContext->fallbackRadius;

			b3Transform meshWorldTransform = b3MulTransforms( toiContext->compoundTransform, child.transform );

			const b3Sweep* sweepB = &toiContext->toiInput.sweepB;
			b3Transform xfB1 = {
				.p = sweepB->c1 - b3RotateVector( sweepB->q1, sweepB->localCenter ),
				.q = sweepB->q1,
			};

			b3Transform xfB2 = {
				.p = sweepB->c2 - b3RotateVector( sweepB->q2, sweepB->localCenter ),
				.q = sweepB->q2,
			};

			meshContext.meshLocalCentroidB1 =
				b3InvTransformPoint( meshWorldTransform, b3TransformPoint( xfB1, meshContext.localCentroidB ) );
			meshContext.meshLocalCentroidB2 =
				b3InvTransformPoint( meshWorldTransform, b3TransformPoint( xfB2, meshContext.localCentroidB ) );

			// Bounds local to mesh
			b3AABB localBounds = b3AABB_Transform( b3InvertTransform( child.transform ), toiContext->localSweepBoundsB );

			b3QueryMesh( &child.mesh, localBounds, b3MeshTimeOfImpactFcn, &meshContext );

			output = meshContext.toiOutput;
		}
		break;

		case b3_sphereShape:
		{
			toiContext->toiInput.proxyA.points = &child.sphere.center;
			toiContext->toiInput.proxyA.count = 1;
			toiContext->toiInput.proxyA.radius = child.sphere.radius;
			output = b3TimeOfImpact( &toiContext->toiInput );
		}
		break;

		default:
			B3_ASSERT( false );
			break;
	}

	if ( 0.0f < output.fraction && output.fraction < toiContext->toiInput.maxFraction )
	{
		toiContext->toiOutput = output;
		toiContext->toiInput.maxFraction = output.fraction;
	}

	// Clear this to be safe
	toiContext->toiInput.proxyA = {0};

	// Continue the query
	return true;
}

b3TOIOutput b3CompoundTimeOfImpact(const b3CompoundData* compound, b3Transform transform, const b3ShapeProxy* proxy,
	const b3Sweep* sweep, b3Fixed maxFraction)
{
	b3CompoundImpactContext context = {0};
	context.toiInput.proxyB = b3MakeShapeProxy( shapeB );
	context.toiInput.sweepB = *sweepB;
	context.toiInput.maxFraction = maxFraction;

	context.compoundTransform = {
		.p = sweepA->c1,
		.q = sweepA->q1,
	};

	b3Vec3 localCentroidB = b3GetShapeCentroid( shapeB );
	context.localCentroidB = localCentroidB;

	b3ShapeExtent extents = b3ComputeShapeExtent( shapeB, context.localCentroidB );
	context.fallbackRadius = b3FixMax( 0.5f * extents.minExtent, B3_SPECULATIVE_DISTANCE );

	// Swept bounds of shapeB
	b3AABB aabb = b3ComputeSweptShapeAABB( shapeB, sweepB, maxFraction );

	// Bounds local to mesh
	b3AABB localBounds = b3AABB_Transform( b3InvertTransform( context.compoundTransform ), bounds );
	context.localSweepBoundsB = localBounds;

	b3DynamicTree tree = b3MakeCompoundTree( compound );
	b3DynamicTree_Query( &tree, aabb, B3_DEFAULT_MASK_BITS, false, TreeQueryCallbackFcn, &compoundContext );

	return context.toiOutput;
}
#endif

// xf = xfP * xfC
b3Sweep b3MakeCompoundChildSweep( b3Transform compoundTransform, b3Transform childTransform )
{
	b3Transform xf = b3MulTransforms( compoundTransform, childTransform );
	return (b3Sweep){
		.localCenter = b3Vec3_zero,
		.c1 = xf.p,
		.c2 = xf.p,
		.q1 = xf.q,
		.q2 = xf.q,
	};
}

struct b3CompoundMoverContext
{
	const b3CompoundData* compound;
	b3PlaneResult* planes;
	int planeCapacity;
	int planeCount;
	b3Capsule mover;
};

static bool b3CompoundMoverCallback( int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int childIndex = (int)userData;
	struct b3CompoundMoverContext* moverContext = context;
	b3ChildShape child = b3GetCompoundChild( moverContext->compound, childIndex );

	// Transform mover to child space
	b3Capsule localMover;
	localMover.center1 = b3InvTransformPoint( child.transform, moverContext->mover.center1 );
	localMover.center2 = b3InvTransformPoint( child.transform, moverContext->mover.center2 );
	localMover.radius = moverContext->mover.radius;

	int capacity = moverContext->planeCapacity - moverContext->planeCount;
	B3_ASSERT( capacity > 0 );

	b3PlaneResult* planes = moverContext->planes + moverContext->planeCount;
	int planeCount = 0;

	switch ( child.type )
	{
		case b3_capsuleShape:
			planeCount = b3CollideMoverAndCapsule( planes, &child.capsule, &localMover );
			break;

		case b3_hullShape:
			planeCount = b3CollideMoverAndHull( planes, child.hull, &localMover );
			break;

		case b3_meshShape:
			planeCount = b3CollideMoverAndMesh( planes, capacity, &child.mesh, &localMover );
			break;

		case b3_sphereShape:
			planeCount = b3CollideMoverAndSphere( planes, &child.sphere, &localMover );
			break;

		default:
			B3_ASSERT( false );
			break;
	}

	// Transform results back to shape space
	for ( int i = 0; i < planeCount; ++i )
	{
		planes[i].plane.normal = b3RotateVector( child.transform.q, planes[i].plane.normal );
		planes[i].point = b3TransformPoint( child.transform, planes[i].point );

		planes[i].childIndex = childIndex;
		B3_ASSERT( 0 <= planes[i].materialIndex && planes[i].materialIndex < child.materialCount );
		planes[i].materialIndex = child.materialIndices[planes[i].materialIndex];
	}

	moverContext->planeCount += planeCount;

	// Continue query while there is room for more planes
	return moverContext->planeCount < moverContext->planeCapacity;
}

int b3CollideMoverAndCompound( b3PlaneResult* planes, int capacity, const b3CompoundData* shape, const b3Capsule* mover )
{
	struct b3CompoundMoverContext context = {
		.compound = shape,
		.planes = planes,
		.planeCapacity = capacity,
		.planeCount = 0,
		.mover = *mover,
	};

	b3AABB aabb;
	aabb.lowerBound = b3Vec3ToBound( b3Min( mover->center1, mover->center2 ) );
	aabb.upperBound = b3Vec3ToBound( b3Max( mover->center1, mover->center2 ) );
	b3Vec3 r = { mover->radius, mover->radius, mover->radius };
	aabb.lowerBound = b3Vec3ToBound( b3Sub( b3BoundToVec3( aabb.lowerBound ), r ) );
	aabb.upperBound = b3Vec3ToBound( b3Add( b3BoundToVec3( aabb.upperBound ), r ) );

	b3DynamicTree tree = b3MakeCompoundTree( shape );
	(void)b3DynamicTree_Query( &tree, aabb, ~0ull, false, b3CompoundMoverCallback, &context );

	return context.planeCount;
}
