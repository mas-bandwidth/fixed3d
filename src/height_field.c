// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"
#include "algorithm.h"
#include "body.h"
#include "core.h"
#include "shape.h"
#include "simd.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*
	Convention

	index = row * columnCount + column
	height = minHeight + heightScale * compressedHeights[index];

	column = index % columnCount;
	row = index / columnCount;

	x-axis : columns
	z-axis : rows

	00 --- 01 --- 02 --- 03 X
	|  0   |  1   |  2   |
	04 --- 05 --- 06 --- 07
	|  3   |  4   |  5   |
	08 --- 09 --- 10 --- 11
	|  6   |  7   |  8   |
	12 --- 13 --- 14 --- 15
	Z

	The quads exist before the column and row ends: row < rowCount - 1 and column < columnCount - 1
	quadIndex = row * (columnCount - 1) + column

	Quad origin index from quad index (needs row):
	index = quadIndex + row * columnCount

	Triangle index is related to the quad index
	triangleIndex = 2 * quadIndex + (0/1)
	quadIndex = triangleIndex / 2

	Row and column from quad index:
	row = quadIndex / (columnCount - 1)
	column = quadIndex - row * (columnCount - 1)

	The triangle diagonal is fixed.

	triangle0 = {00, 04, 01} -> {11, 21, 12}
	triangle1 = {04, 05, 01} -> {22, 12, 21}

	11      12
	00 ---- 01
	|     / |
	| 0 / 1 | 1
	| /     |
	04 ---- 05
	21     22

	For adjacency we have

	   NA
	   00 ---- 01 ---- 02
	   |     / |     / |
	NA | 0 / 1 | 2 / 3 |
	   | /     | /     |
	   04 ---- 05 ---- 06
	   |     / |     / |
	NA | 6 / 7 | 8 / 9 |
	   | /     | /     |
	   08 ---- 09 ---- 10

	   0: NA, NA, 1
	   1: 0, 6, 2

	Triangle layouts

	   11  3  12
	   1 ----  3
	   |     /
	 1 | 0 / 2
	   | /
	   2
	   21

			  12
			  2
			/ |
		2 / 1 | 1
		/     |
	   3 ---- 1
	   21  3  22
 */

b3HeightFieldData* b3CreateHeightField( const b3HeightFieldDef* data )
{
	int columnCount = data->countX;
	int rowCount = data->countZ;

	int heightCount = columnCount * rowCount;
	B3_ASSERT( heightCount >= 4 );

	int cellCount = ( columnCount - 1 ) * ( rowCount - 1 );
	int triangleCount = 2 * cellCount;

	// Single blob: struct followed by the height, material, and flag arrays. Layout
	// mirrors b3HullData/b3MeshData so the recording path can copy it with one memcpy.
	size_t byteCount = b3AlignUp8( sizeof( b3HeightFieldData ) );
	int heightsOffset = (int)byteCount;
	byteCount += b3AlignUp8( heightCount * sizeof( uint16_t ) );
	int materialOffset = (int)byteCount;
	byteCount += b3AlignUp8( cellCount * sizeof( uint8_t ) );
	int flagsOffset = (int)byteCount;
	byteCount += b3AlignUp8( triangleCount * sizeof( uint8_t ) );

	// Zero the whole blob so alignment padding is defined. The construction-time hash
	// sweeps raw bytes and would otherwise pick up uninitialized padding.
	b3HeightFieldData* hf = (b3HeightFieldData*)b3Alloc( byteCount );
	memset( hf, 0, byteCount );

	hf->version = B3_HEIGHT_FIELD_VERSION;
	hf->byteCount = (int)byteCount;
	hf->scale = data->scale;
	hf->columnCount = columnCount;
	hf->rowCount = rowCount;
	hf->heightsOffset = heightsOffset;
	hf->materialOffset = materialOffset;
	hf->flagsOffset = flagsOffset;
	hf->clockwise = data->clockwiseWinding;

	uint16_t* compressedHeights = (uint16_t*)( (intptr_t)hf + heightsOffset );
	uint8_t* materialIndices = (uint8_t*)( (intptr_t)hf + materialOffset );
	uint8_t* flags = (uint8_t*)( (intptr_t)hf + flagsOffset );

	const b3Fixed* heights = data->heights;

	B3_ASSERT( data->globalMinimumHeight <= data->globalMaximumHeight );
	hf->minHeight = data->globalMinimumHeight;
	hf->maxHeight = data->globalMaximumHeight;

	b3Fixed height = b3FixMax( hf->maxHeight - hf->minHeight, B3_LINEAR_SLOP );
	hf->heightScale = b3FixDiv( height, b3FixFromInt( UINT16_MAX ) );

	b3Fixed lowerHeightBound = hf->maxHeight;
	b3Fixed upperHeightBound = hf->minHeight;

	b3Fixed invHeightScale = b3FixDiv( B3_FIX( 1.0f ) , hf->heightScale );
	for ( int i = 0; i < heightCount; ++i )
	{
		b3Fixed clampedHeight = b3FixClamp( heights[i], hf->minHeight, hf->maxHeight );
		b3Fixed scaledHeight = b3FixMul( ( clampedHeight - hf->minHeight ) , invHeightScale );
		compressedHeights[i] = (uint16_t)b3FixTruncToInt( ( b3FixMin( scaledHeight, b3FixFromInt( UINT16_MAX ) ) ) );

		lowerHeightBound = b3FixMin( lowerHeightBound, clampedHeight );
		upperHeightBound = b3FixMax( upperHeightBound, clampedHeight );
	}

	// Use decompressed heights for accurate convexity metrics.
	b3Fixed* decompressedHeights = (b3Fixed*)b3Alloc( heightCount * sizeof( b3Fixed ) );
	for ( int i = 0; i < heightCount; ++i )
	{
		decompressedHeights[i] = hf->minHeight + b3FixMul( hf->heightScale , b3FixFromInt( compressedHeights[i] ) );
	}
	heights = decompressedHeights;

	if ( data->materialIndices != NULL )
	{
		for ( int i = 0; i < cellCount; ++i )
		{
			materialIndices[i] = data->materialIndices[i];
		}
	}
	else
	{
		for ( int i = 0; i < cellCount; ++i )
		{
			materialIndices[i] = 0;
		}
	}

	hf->aabb.lowerBound = (b3Vec3){ B3_FIX( 0.0f ), b3FixMul( hf->scale.y , lowerHeightBound ), B3_FIX( 0.0f ) };
	hf->aabb.upperBound =
		(b3Vec3){ b3FixMul( hf->scale.x , b3FixFromInt( ( hf->columnCount - 1 ) ) ), b3FixMul( hf->scale.y , upperHeightBound ), b3FixMul( hf->scale.z , b3FixFromInt( ( hf->rowCount - 1 ) ) ) };

	b3Fixed cos5Deg = B3_FIX( 0.9962f );
	b3Vec3 scale = hf->scale;

	int triangleIndex = 0;
	for ( int row = 0; row < rowCount - 1; ++row )
	{
		for ( int column = 0; column < columnCount - 1; ++column )
		{
			// todo compute convexity flags
			// This requires a couple things
			// - determine all 3 adjacent triangles for each triangle
			// - consider clockwise winding
			// - consider borders where there is no adjacent triangle

			int triangleIndex1 = triangleIndex;
			int triangleIndex2 = triangleIndex + 1;
			triangleIndex += 2;

			int cellIndex = row * ( columnCount - 1 ) + column;

			if ( materialIndices[cellIndex] == B3_HEIGHT_FIELD_HOLE )
			{
				continue;
			}

			int flags1 = 0;
			int flags2 = 0;

			b3Plane plane1, plane2;
			b3Vec3 center1, center2;

			int index11 = row * columnCount + column;
			int index12 = index11 + 1;
			int index21 = ( row + 1 ) * columnCount + column;
			int index22 = index21 + 1;

			{
				b3Fixed height11 = heights[index11];
				b3Fixed height12 = heights[index12];
				b3Fixed height21 = heights[index21];
				b3Fixed height22 = heights[index22];

				b3Fixed x1 = (b3Fixed)b3FixFromInt( ( column ) );
				b3Fixed x2 = (b3Fixed)b3FixFromInt( ( column + 1 ) );
				b3Fixed z1 = (b3Fixed)b3FixFromInt( ( row ) );
				b3Fixed z2 = (b3Fixed)b3FixFromInt( ( row + 1 ) );

				// triangle 0 : 11, 21, 12
				b3Vec3 vs0[3];
				vs0[0] = b3Mul( scale, (b3Vec3){ x1, height11, z1 } );
				vs0[1] = b3Mul( scale, (b3Vec3){ x1, height21, z2 } );
				vs0[2] = b3Mul( scale, (b3Vec3){ x2, height12, z1 } );
				plane1 = b3MakePlaneFromPoints( vs0[0], vs0[1], vs0[2] );

				center1 = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), b3Add( b3Add( vs0[0], vs0[1] ), vs0[2] ) );

				// triangle 1 : 22, 12, 21
				b3Vec3 vs1[3];
				vs1[0] = b3Mul( scale, (b3Vec3){ x2, height22, z2 } );
				vs1[1] = b3Mul( scale, (b3Vec3){ x2, height12, z1 } );
				vs1[2] = b3Mul( scale, (b3Vec3){ x1, height21, z2 } );
				plane2 = b3MakePlaneFromPoints( vs1[0], vs1[1], vs1[2] );

				center2 = b3MulSV( b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 3.0f ) ), b3Add( b3Add( vs1[0], vs1[1] ), vs1[2] ) );

				b3Fixed separation = b3PlaneSeparation( plane1, vs1[0] );
				b3Fixed cosAngle = b3Dot( plane1.normal, plane2.normal );
				if ( separation > B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags1 |= b3_concaveEdge2;
					flags2 |= b3_concaveEdge2;
				}
				if ( separation < B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags1 |= b3_inverseConcaveEdge2;
					flags2 |= b3_inverseConcaveEdge2;
				}
			}

			B3_UNUSED( center1 );
			B3_UNUSED( center2 );

			// top
			int topCellIndex = ( row - 1 ) * ( columnCount - 1 ) + column;
			if ( row > 0 && materialIndices[topCellIndex] != B3_HEIGHT_FIELD_HOLE )
			{
				B3_ASSERT( 0 <= topCellIndex && topCellIndex < cellCount );

				int r = row - 1;
				int c = column;

				int i11 = r * columnCount + c;
				int i12 = i11 + 1;
				int i21 = ( r + 1 ) * columnCount + c;
				int i22 = i21 + 1;

				B3_ASSERT( i21 == index11 );
				B3_ASSERT( i22 == index12 );

				// b3Fixed h11 = heights[i11];
				b3Fixed h12 = heights[i12];
				b3Fixed h21 = heights[i21];
				b3Fixed h22 = heights[i22];

				b3Fixed x1 = (b3Fixed)b3FixFromInt( ( c ) );
				b3Fixed x2 = (b3Fixed)b3FixFromInt( ( c + 1 ) );
				b3Fixed z1 = (b3Fixed)b3FixFromInt( ( r ) );
				b3Fixed z2 = (b3Fixed)b3FixFromInt( ( r + 1 ) );

				// triangle 1
				b3Vec3 vs[3];
				vs[0] = b3Mul( scale, (b3Vec3){ x2, h22, z2 } );
				vs[1] = b3Mul( scale, (b3Vec3){ x2, h12, z1 } );
				vs[2] = b3Mul( scale, (b3Vec3){ x1, h21, z2 } );

				b3Vec3 n = b3Normalize( b3Cross( b3Sub( vs[1], vs[0] ), b3Sub( vs[2], vs[0] ) ) );

				b3Fixed separation = b3PlaneSeparation( plane1, vs[1] );
				b3Fixed cosAngle = b3Dot( plane1.normal, n );
				if ( separation > B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags1 |= b3_concaveEdge3;
				}
				if ( separation < B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags1 |= b3_inverseConcaveEdge3;
				}
			}

			int bottomCellIndex = ( row + 1 ) * ( columnCount - 1 ) + column;
			if ( row + 1 < rowCount - 1 && materialIndices[bottomCellIndex] != B3_HEIGHT_FIELD_HOLE )
			{
				B3_ASSERT( 0 <= bottomCellIndex && bottomCellIndex < cellCount );

				int r = row + 1;
				int c = column;

				int i11 = r * columnCount + c;
				int i12 = i11 + 1;
				int i21 = ( r + 1 ) * columnCount + c;
				// int i22 = i21 + 1;

				B3_ASSERT( i11 == index21 );
				B3_ASSERT( i12 == index22 );

				b3Fixed h11 = heights[i11];
				b3Fixed h12 = heights[i12];
				b3Fixed h21 = heights[i21];
				// b3Fixed h22 = heights[i22];

				b3Fixed x1 = (b3Fixed)b3FixFromInt( ( c ) );
				b3Fixed x2 = (b3Fixed)b3FixFromInt( ( c + 1 ) );
				b3Fixed z1 = (b3Fixed)b3FixFromInt( ( r ) );
				b3Fixed z2 = (b3Fixed)b3FixFromInt( ( r + 1 ) );

				// triangle 0
				b3Vec3 vs[3];
				vs[0] = b3Mul( scale, (b3Vec3){ x1, h11, z1 } );
				vs[1] = b3Mul( scale, (b3Vec3){ x1, h21, z2 } );
				vs[2] = b3Mul( scale, (b3Vec3){ x2, h12, z1 } );

				b3Vec3 n = b3Normalize( b3Cross( b3Sub( vs[1], vs[0] ), b3Sub( vs[2], vs[0] ) ) );

				b3Fixed separation = b3PlaneSeparation( plane2, vs[1] );
				b3Fixed cosAngle = b3Dot( plane2.normal, n );
				if ( separation > B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags2 |= b3_concaveEdge3;
				}
				if ( separation < B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags2 |= b3_inverseConcaveEdge3;
				}
			}

			int leftCellIndex = row * ( columnCount - 1 ) + column - 1;
			if ( column - 1 >= 0 && materialIndices[leftCellIndex] != B3_HEIGHT_FIELD_HOLE )
			{
				B3_ASSERT( 0 <= leftCellIndex && leftCellIndex < cellCount );

				int r = row;
				int c = column - 1;

				int i11 = r * columnCount + c;
				int i12 = i11 + 1;
				int i21 = ( r + 1 ) * columnCount + c;
				int i22 = i21 + 1;

				B3_ASSERT( i12 == index11 );
				B3_ASSERT( i22 == index21 );

				// b3Fixed h11 = heights[i11];
				b3Fixed h12 = heights[i12];
				b3Fixed h21 = heights[i21];
				b3Fixed h22 = heights[i22];

				b3Fixed x1 = (b3Fixed)b3FixFromInt( ( c ) );
				b3Fixed x2 = (b3Fixed)b3FixFromInt( ( c + 1 ) );
				b3Fixed z1 = (b3Fixed)b3FixFromInt( ( r ) );
				b3Fixed z2 = (b3Fixed)b3FixFromInt( ( r + 1 ) );

				// triangle 1
				b3Vec3 vs[3];
				vs[0] = b3Mul( scale, (b3Vec3){ x2, h22, z2 } );
				vs[1] = b3Mul( scale, (b3Vec3){ x2, h12, z1 } );
				vs[2] = b3Mul( scale, (b3Vec3){ x1, h21, z2 } );

				b3Vec3 n = b3Normalize( b3Cross( b3Sub( vs[1], vs[0] ), b3Sub( vs[2], vs[0] ) ) );

				b3Fixed separation = b3PlaneSeparation( plane1, vs[2] );
				b3Fixed cosAngle = b3Dot( plane1.normal, n );
				if ( separation > B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags1 |= b3_concaveEdge1;
				}
				if ( separation < B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags1 |= b3_inverseConcaveEdge1;
				}
			}

			int rightCellIndex = row * ( columnCount - 1 ) + column + 1;
			if ( column + 1 < columnCount - 1 && materialIndices[rightCellIndex] != B3_HEIGHT_FIELD_HOLE )
			{
				B3_ASSERT( 0 <= rightCellIndex && rightCellIndex < cellCount );

				int r = row;
				int c = column + 1;

				int i11 = r * columnCount + c;
				int i12 = i11 + 1;
				int i21 = ( r + 1 ) * columnCount + c;
				// int i22 = i21 + 1;

				B3_ASSERT( i11 == index12 );
				B3_ASSERT( i21 == index22 );

				b3Fixed h11 = heights[i11];
				b3Fixed h12 = heights[i12];
				b3Fixed h21 = heights[i21];
				// b3Fixed h22 = heights[i22];

				b3Fixed x1 = (b3Fixed)b3FixFromInt( ( c ) );
				b3Fixed x2 = (b3Fixed)b3FixFromInt( ( c + 1 ) );
				b3Fixed z1 = (b3Fixed)b3FixFromInt( ( r ) );
				b3Fixed z2 = (b3Fixed)b3FixFromInt( ( r + 1 ) );

				// triangle 0
				b3Vec3 vs[3];
				vs[0] = b3Mul( scale, (b3Vec3){ x1, h11, z1 } );
				vs[1] = b3Mul( scale, (b3Vec3){ x1, h21, z2 } );
				vs[2] = b3Mul( scale, (b3Vec3){ x2, h12, z1 } );

				b3Vec3 n = b3Normalize( b3Cross( b3Sub( vs[1], vs[0] ), b3Sub( vs[2], vs[0] ) ) );

				b3Fixed separation = b3PlaneSeparation( plane2, vs[2] );
				b3Fixed cosAngle = b3Dot( plane2.normal, n );
				if ( separation > B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags2 |= b3_concaveEdge1;
				}
				if ( separation < B3_FIX( 0.0f ) || cosAngle > cos5Deg )
				{
					flags2 |= b3_inverseConcaveEdge1;
				}
			}

			B3_ASSERT( 0 <= flags1 && flags1 <= UINT8_MAX );
			B3_ASSERT( 0 <= flags2 && flags2 <= UINT8_MAX );

			flags[triangleIndex1] = (uint8_t)flags1;
			flags[triangleIndex2] = (uint8_t)flags2;
		}
	}

	B3_ASSERT( triangleIndex == triangleCount );

	b3Free( decompressedHeights, heightCount * sizeof( b3Fixed ) );

	// Content hash over the whole blob with the hash field zeroed, like b3HullData/b3MeshData.
	hf->hash = 0;
	hf->hash = b3NonZeroHash( b3Hash( B3_HASH_INIT, (const uint8_t*)hf, hf->byteCount ) );

	return hf;
}

_Static_assert( b3_concaveEdge3 == 4 * b3_concaveEdge1, "bit math" );
_Static_assert( b3_inverseConcaveEdge3 == 4 * b3_inverseConcaveEdge1, "bit math" );

// Decode the four corner vertices of a height field cell into local space.
// Output order matches the index naming used throughout this file:
// corners[0] = (column, row), corners[1] = (column + 1, row),
// corners[2] = (column, row + 1), corners[3] = (column + 1, row + 1).
static inline void b3GetHeightFieldCellCorners( const b3HeightFieldData* hf, int row, int column, b3Vec3 corners[4] )
{
	B3_ASSERT( 0 <= row && row < hf->rowCount - 1 && 0 <= column && column < hf->columnCount - 1 );

	int columnCount = hf->columnCount;
	int index11 = row * columnCount + column;
	int index12 = index11 + 1;
	int index21 = ( row + 1 ) * columnCount + column;
	int index22 = index21 + 1;

	b3Fixed minHeight = hf->minHeight;
	b3Fixed heightScale = hf->heightScale;
	const uint16_t* heights = b3GetHeightFieldCompressedHeights( hf );

	b3Fixed height11 = minHeight + b3FixMul( heightScale , b3FixFromInt( heights[index11] ) );
	b3Fixed height12 = minHeight + b3FixMul( heightScale , b3FixFromInt( heights[index12] ) );
	b3Fixed height21 = minHeight + b3FixMul( heightScale , b3FixFromInt( heights[index21] ) );
	b3Fixed height22 = minHeight + b3FixMul( heightScale , b3FixFromInt( heights[index22] ) );

	b3Fixed x1 = (b3Fixed)b3FixFromInt( ( column ) );
	b3Fixed x2 = (b3Fixed)b3FixFromInt( ( column + 1 ) );
	b3Fixed z1 = (b3Fixed)b3FixFromInt( ( row ) );
	b3Fixed z2 = (b3Fixed)b3FixFromInt( ( row + 1 ) );

	b3Vec3 scale = hf->scale;
	corners[0] = b3Mul( scale, (b3Vec3){ x1, height11, z1 } );
	corners[1] = b3Mul( scale, (b3Vec3){ x2, height12, z1 } );
	corners[2] = b3Mul( scale, (b3Vec3){ x1, height21, z2 } );
	corners[3] = b3Mul( scale, (b3Vec3){ x2, height22, z2 } );
}

b3Triangle b3GetHeightFieldTriangle( const b3HeightFieldData* heightField, int triangleIndex )
{
	B3_ASSERT( 0 <= triangleIndex );
	B3_ASSERT( triangleIndex < 2 * ( heightField->columnCount - 1 ) * ( heightField->rowCount - 1 ) );

	b3Triangle triangle;
	triangle.flags = b3GetHeightFieldFlags( heightField )[triangleIndex];

	int columnCount = heightField->columnCount;
	int quadIndex = triangleIndex >> 1;
	int row = quadIndex / ( columnCount - 1 );
	int column = quadIndex - row * ( columnCount - 1 );

	int index11 = row * columnCount + column;
	int index12 = index11 + 1;
	int index21 = ( row + 1 ) * columnCount + column;
	int index22 = index21 + 1;

	int cellIndex = row * ( columnCount - 1 ) + column;

	B3_ASSERT( quadIndex == cellIndex );
	B3_ASSERT( b3GetHeightFieldMaterialIndices( heightField )[cellIndex] != B3_HEIGHT_FIELD_HOLE );
	B3_UNUSED( cellIndex );

	b3Vec3 corners[4];
	b3GetHeightFieldCellCorners( heightField, row, column, corners );

	if ( ( triangleIndex & 1 ) == 0 )
	{
		triangle.vertices[0] = corners[0];
		triangle.vertices[1] = corners[2];
		triangle.vertices[2] = corners[1];
		triangle.i1 = index11;
		triangle.i2 = index21;
		triangle.i3 = index12;
	}
	else
	{
		triangle.vertices[0] = corners[3];
		triangle.vertices[1] = corners[1];
		triangle.vertices[2] = corners[2];
		triangle.i1 = index22;
		triangle.i2 = index12;
		triangle.i3 = index21;
	}

	if ( heightField->clockwise )
	{
		B3_SWAP( triangle.vertices[1], triangle.vertices[2] );
		B3_SWAP( triangle.i2, triangle.i3 );

		// Reversing winding swaps edge1 and edge3; edge2 (the diagonal) is preserved.
		int flags = triangle.flags;
		int edge1Bits = flags & ( b3_concaveEdge1 | b3_inverseConcaveEdge1 );
		int edge3Bits = flags & ( b3_concaveEdge3 | b3_inverseConcaveEdge3 );
		flags &= ~( b3_concaveEdge1 | b3_concaveEdge3 | b3_inverseConcaveEdge1 | b3_inverseConcaveEdge3 );
		flags |= edge1Bits << 2;
		flags |= edge3Bits >> 2;
		triangle.flags = flags;
	}

	return triangle;
}

int b3GetHeightFieldMaterial( const b3HeightFieldData* heightField, int triangleIndex )
{
	B3_ASSERT( 0 <= triangleIndex );
	B3_ASSERT( triangleIndex < 2 * ( heightField->columnCount - 1 ) * ( heightField->rowCount - 1 ) );

	int cellIndex = triangleIndex >> 1;
	return b3GetHeightFieldMaterialIndices( heightField )[cellIndex];
}

b3AABB b3ComputeHeightFieldAABB( const b3HeightFieldData* shape, b3Transform transform )
{
	return b3AABB_Transform( transform, shape->aabb );
}

b3CastOutput b3RayCastHeightField( const b3HeightFieldData* heightField, const b3RayCastInput* input )
{
	b3ShapeCastInput shapeCastInput = { 0 };
	shapeCastInput.proxy = (b3ShapeProxy){ &input->origin, 1, B3_FIX( 0.0f ) };
	shapeCastInput.translation = input->translation;
	shapeCastInput.maxFraction = input->maxFraction;

	return b3ShapeCastHeightField( heightField, &shapeCastInput );
}

// todo advance cast to the grid border immediately if it starts outside the row/column range
// todo terminate the cast immediately if it leaves the row/column range
b3CastOutput b3ShapeCastHeightField( const b3HeightFieldData* heightField, const b3ShapeCastInput* input )
{
	b3AABB shapeBounds = b3MakeAABB( input->proxy.points, input->proxy.count, input->proxy.radius );
	b3Vec3 shapeTranslation = input->translation;
	b3Vec3 scale = heightField->scale;

	b3Vec3 shapeStart = b3AABB_Center( shapeBounds );
	b3Vec3 shapeDelta = b3MulSV( input->maxFraction, shapeTranslation );
	b3Vec3 shapeEnd = b3Add( shapeStart, shapeDelta );

	b3CastOutput result = { b3FixFromInt( 0 ) };

	b3Vec3 shapeExtents = b3AABB_Extents( shapeBounds );
	b3Vec3 margin = { B3_MAX_AABB_MARGIN, B3_MAX_AABB_MARGIN, B3_MAX_AABB_MARGIN };
	b3AABB combinedBounds = { b3Sub( b3Sub( heightField->aabb.lowerBound, shapeExtents ), margin ),
							  b3Add( b3Add( heightField->aabb.upperBound, shapeExtents ), margin ) };

	b3Fixed minFraction, maxFraction;
	bool intersects = b3RayCastAABB( combinedBounds, shapeStart, shapeEnd, &minFraction, &maxFraction );
	if ( intersects == false )
	{
		return result;
	}

	// These are for walking the grid, not the triangle cast.
	// The triangle cast uses the unclamped ray and fraction.
	b3Vec3 clampedStart = b3MulAdd( shapeStart, minFraction, shapeDelta );
	b3Vec3 clampedDelta = b3MulSV( maxFraction - minFraction, shapeDelta );
	b3Vec3 clampedEnd = b3Add( clampedStart, clampedDelta );

	// Preserve the un-shifted center sweep. clampedStart/clampedEnd get pushed out to the
	// leading box corner below to drive the grid DDA, but the swept-volume AABB used to
	// cull cells must stay centered on the actual shape path.
	b3Vec3 centerStart = clampedStart;
	b3Vec3 centerEnd = clampedEnd;

	// The grid traversal starts from the leading shape bounds corner
	b3Fixed signX, signZ;
	if ( shapeTranslation.x >= B3_FIX( 0.0f ) )
	{
		clampedStart.x += shapeExtents.x;
		signX = B3_FIX( 1.0f );
	}
	else
	{
		clampedStart.x -= shapeExtents.x;
		signX = -B3_FIX( 1.0f );
	}

	if ( shapeTranslation.z >= B3_FIX( 0.0f ) )
	{
		clampedStart.z += shapeExtents.z;
		signZ = B3_FIX( 1.0f );
	}
	else
	{
		clampedStart.z -= shapeExtents.z;
		signZ = -B3_FIX( 1.0f );
	}

	// Shift the end as well
	clampedEnd = b3Add( clampedStart, clampedDelta );

	// Row and column range for the shape cast
	int columnStart = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( clampedStart.x , scale.x ) ) );
	int columnEnd = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( clampedEnd.x , scale.x ) ) );
	int rowStart = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( clampedStart.z , scale.z ) ) );
	int rowEnd = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( clampedEnd.z , scale.z ) ) );

	b3Vec3 absClampedDelta = b3Abs( clampedDelta );

	// Precompute increments for row and column traversal.
	// The ray can be slightly tilted yet remain within a single row or column
	// once rasterized.
	b3Fixed deltaAlphaX;
	b3Fixed nextFractionX;
	int deltaColumn;

	if ( columnStart < columnEnd )
	{
		B3_ASSERT( absClampedDelta.x > B3_FIX( 0.0f ) );

		// Going forward on x columns
		deltaAlphaX = b3FixDiv( scale.x , absClampedDelta.x );
		nextFractionX = b3FixDiv( ( b3FixMul( scale.x , b3FixFromInt( ( columnStart + 1 ) ) ) - clampedStart.x ) , absClampedDelta.x );
		deltaColumn = 1;
	}
	else if ( columnEnd < columnStart )
	{
		B3_ASSERT( absClampedDelta.x > B3_FIX( 0.0f ) );

		// Going backwards on x columns
		deltaAlphaX = b3FixDiv( scale.x , absClampedDelta.x );
		nextFractionX = b3FixDiv( ( clampedStart.x - b3FixMul( scale.x , b3FixFromInt( columnStart ) ) ) , absClampedDelta.x );
		deltaColumn = -1;
	}
	else
	{
		// Cast stays in a single column
		deltaAlphaX = B3_FIX( 0.0f );
		nextFractionX = B3_FIXED_MAX;
		deltaColumn = 0;
	}

	b3Fixed deltaAlphaZ;
	b3Fixed nextFractionZ;
	int deltaRow;

	if ( rowStart < rowEnd )
	{
		B3_ASSERT( absClampedDelta.z > B3_FIX( 0.0f ) );

		// Going forward on z rows
		deltaAlphaZ = b3FixDiv( scale.z , absClampedDelta.z );
		nextFractionZ = b3FixDiv( ( b3FixMul( scale.z , b3FixFromInt( ( rowStart + 1 ) ) ) - clampedStart.z ) , absClampedDelta.z );
		deltaRow = 1;
	}
	else if ( rowEnd < rowStart )
	{
		B3_ASSERT( absClampedDelta.z > B3_FIX( 0.0f ) );

		// Going backwards on z rows
		deltaAlphaZ = b3FixDiv( scale.z , absClampedDelta.z );
		nextFractionZ = b3FixDiv( ( clampedStart.z - b3FixMul( scale.z , b3FixFromInt( rowStart ) ) ) , absClampedDelta.z );
		deltaRow = -1;
	}
	else
	{
		// Cast stays in a single row
		deltaAlphaZ = B3_FIX( 0.0f );
		nextFractionZ = B3_FIXED_MAX;
		deltaRow = 0;
	}

	// Column and row range for 2D projected initial shape bounds
	int boxColumnHead = columnStart;
	int boxRowHead = rowStart;

	int boxColumnTail = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( ( clampedStart.x - b3FixMul( b3FixMul( B3_FIX( 2.0f ) , signX ) , shapeExtents.x ) ) , scale.x ) ) );
	int boxRowTail = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( ( clampedStart.z - b3FixMul( b3FixMul( B3_FIX( 2.0f ) , signZ ) , shapeExtents.z ) ) , scale.z ) ) );

	b3Fixed bestFraction = input->maxFraction;

	// nextFractionX / nextFractionZ advance in units of the clamped sweep
	// [minFraction, maxFraction], but bestFraction is a fraction of the full input
	// translation. Precompute the affine map from clamped space to input space so
	// the loop termination test compares like with like — otherwise it can exit
	// early and miss a closer hit in a later cell.
	b3Fixed gridFractionScale = b3FixMul( input->maxFraction , ( maxFraction - minFraction ) );
	b3Fixed gridFractionOffset = b3FixMul( input->maxFraction , minFraction );

	int rowCount = heightField->rowCount;
	int columnCount = heightField->columnCount;
	int cellCount = ( heightField->rowCount - 1 ) * ( heightField->columnCount - 1 );
	B3_UNUSED( cellCount );

	b3ShapeCastPairInput pairInput = { 0 };
	pairInput.proxyB = input->proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input->translation;
	pairInput.canEncroach = input->canEncroach;

	b3AABB castBounds;
	castBounds.lowerBound = b3Sub( b3Min( centerStart, centerEnd ), shapeExtents );
	castBounds.upperBound = b3Add( b3Max( centerStart, centerEnd ), shapeExtents );

	b3V32 rayOrigin = b3LoadV( &shapeStart.x );
	b3V32 rayTranslation = b3LoadV( &shapeTranslation.x );

	while ( true )
	{
		int column1, column2;
		if ( boxColumnTail < boxColumnHead )
		{
			column1 = boxColumnTail;
			column2 = boxColumnHead;
		}
		else
		{
			column1 = boxColumnHead;
			column2 = boxColumnTail;
		}

		int row1, row2;
		if ( boxRowTail < boxRowHead )
		{
			row1 = boxRowTail;
			row2 = boxRowHead;
		}
		else
		{
			row1 = boxRowHead;
			row2 = boxRowTail;
		}

		for ( int row = row1; row <= row2; ++row )
		{
			if ( row < 0 || rowCount - 1 <= row )
			{
				continue;
			}

			for ( int column = column1; column <= column2; ++column )
			{
				if ( column < 0 || columnCount - 1 <= column )
				{
					continue;
				}

				int cellIndex = row * ( columnCount - 1 ) + column;
				B3_ASSERT( cellIndex < cellCount );

				uint8_t materialIndex = b3GetHeightFieldMaterialIndices( heightField )[cellIndex];
				if ( materialIndex == B3_HEIGHT_FIELD_HOLE )
				{
					continue;
				}

				b3Vec3 corners[4];
				b3GetHeightFieldCellCorners( heightField, row, column, corners );
				b3Vec3 point11 = corners[0];
				b3Vec3 point12 = corners[1];
				b3Vec3 point21 = corners[2];
				b3Vec3 point22 = corners[3];

				// I know the min/max x and z values, but not the min/max heights.
				b3AABB bounds;
				bounds.lowerBound = b3Min( b3Min( point11, point12 ), b3Min( point21, point22 ) );
				bounds.upperBound = b3Max( b3Max( point11, point12 ), b3Max( point21, point22 ) );

				if ( b3AABB_Overlaps( castBounds, bounds ) == false )
				{
					continue;
				}

				int quadIndex = row * ( columnCount - 1 ) + column;
				int triangleIndex1 = 2 * quadIndex;
				int triangleIndex2 = triangleIndex1 + 1;

				if ( input->proxy.count == 1 && input->proxy.radius == B3_FIX( 0.0f ) )
				{
					// Ray cast
					{
						b3V32 vertex1 = b3LoadV( &point11.x );
						b3V32 vertex2, vertex3;

						if ( heightField->clockwise )
						{
							vertex2 = b3LoadV( &point12.x );
							vertex3 = b3LoadV( &point21.x );
						}
						else
						{
							vertex2 = b3LoadV( &point21.x );
							vertex3 = b3LoadV( &point12.x );
						}

						b3Fixed alpha = b3IntersectRayTriangle( rayOrigin, rayTranslation, vertex1, vertex2, vertex3 );
						B3_ASSERT( b3FixFromInt( 0 ) <= alpha && alpha <= B3_FIX( 1.0f ) );

						if ( alpha < bestFraction )
						{
							b3Vec3 edge1 = b3Sub( point21, point11 );
							b3Vec3 edge2 = b3Sub( point12, point11 );
							b3Vec3 normal = heightField->clockwise ? b3Cross( edge2, edge1 ) : b3Cross( edge1, edge2 );

							result.point = b3MulAdd( shapeStart, alpha, shapeTranslation );
							result.normal = b3Normalize( normal );
							result.fraction = alpha;
							result.triangleIndex = triangleIndex1;
							result.materialIndex = materialIndex;
							result.hit = true;
							bestFraction = alpha;
						}
					}

					{
						b3V32 vertex1 = b3LoadV( &point22.x );
						b3V32 vertex2, vertex3;

						if ( heightField->clockwise )
						{
							vertex2 = b3LoadV( &point21.x );
							vertex3 = b3LoadV( &point12.x );
						}
						else
						{
							vertex2 = b3LoadV( &point12.x );
							vertex3 = b3LoadV( &point21.x );
						}

						b3Fixed alpha = b3IntersectRayTriangle( rayOrigin, rayTranslation, vertex1, vertex2, vertex3 );
						B3_ASSERT( b3FixFromInt( 0 ) <= alpha && alpha <= B3_FIX( 1.0f ) );

						if ( alpha < bestFraction )
						{
							b3Vec3 edge1 = b3Sub( point22, point21 );
							b3Vec3 edge2 = b3Sub( point12, point21 );
							b3Vec3 normal = heightField->clockwise ? b3Cross( edge2, edge1 ) : b3Cross( edge1, edge2 );

							result.point = b3MulAdd( shapeStart, alpha, shapeTranslation );
							result.normal = b3Normalize( normal );
							result.fraction = alpha;
							result.triangleIndex = triangleIndex2;
							result.materialIndex = materialIndex;
							result.hit = true;
							bestFraction = alpha;
						}
					}
				}
				else
				{
					// Shape cast
					// todo back-side culling
					{
						// Shift origin to first vertex
						b3Vec3 origin = point11;
						b3Vec3 triangleVertices[] = { b3Vec3_zero, b3Sub( point21, origin ), b3Sub( point12, origin ) };
						pairInput.proxyA = (b3ShapeProxy){ triangleVertices, 3, B3_FIX( 0.0f ) };
						pairInput.maxFraction = bestFraction;
						pairInput.transform.p = b3Neg( origin );

						b3CastOutput pairOutput = b3ShapeCast( &pairInput );

						if ( pairOutput.hit )
						{
							bestFraction = pairOutput.fraction;
							result = pairOutput;
							result.point = b3Add( result.point, origin );
							result.triangleIndex = triangleIndex1;
							result.materialIndex = materialIndex;
						}
					}

					{
						// Shift origin to first vertex
						b3Vec3 origin = point21;
						b3Vec3 triangleVertices[] = { b3Vec3_zero, b3Sub( point22, origin ), b3Sub( point12, origin ) };
						pairInput.proxyA = (b3ShapeProxy){ triangleVertices, 3, B3_FIX( 0.0f ) };
						pairInput.maxFraction = bestFraction;
						pairInput.transform.p = b3Neg( origin );

						b3CastOutput pairOutput = b3ShapeCast( &pairInput );

						if ( pairOutput.hit )
						{
							bestFraction = pairOutput.fraction;
							result = pairOutput;
							result.point = b3Add( result.point, origin );
							result.triangleIndex = triangleIndex2;
							result.materialIndex = materialIndex;
						}
					}
				}
			}
		}

		// These fractions always increase to guarantee the loop eventually exits.
		// Map them from clamped-sweep space into input-translation space before
		// comparing against bestFraction.
		b3Fixed inputFractionX = nextFractionX == B3_FIXED_MAX ? B3_FIXED_MAX : gridFractionOffset + b3FixMul( nextFractionX , gridFractionScale );
		b3Fixed inputFractionZ = nextFractionZ == B3_FIXED_MAX ? B3_FIXED_MAX : gridFractionOffset + b3FixMul( nextFractionZ , gridFractionScale );
		if ( inputFractionX > bestFraction && inputFractionZ > bestFraction )
		{
			break;
		}

		// Advance the cast to the next column or row
		if ( nextFractionX <= nextFractionZ )
		{
			if ( boxColumnHead == columnEnd )
			{
				// Hit the end already
				break;
			}

			// Advance to next column
			boxColumnHead += deltaColumn;

			// Build a single column to cast
			boxColumnTail = boxColumnHead;

			if ( shapeExtents.z == B3_FIX( 0.0f ) )
			{
				// Single row
				boxRowTail = boxRowHead;
			}
			else
			{
				// Rasterize shape row
				b3Fixed rowIntercept = clampedStart.z + b3FixMul( nextFractionX , clampedDelta.z );
				boxRowTail = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( ( rowIntercept - b3FixMul( b3FixMul( B3_FIX( 2.0f ) , signZ ) , shapeExtents.z ) ) , scale.z ) ) );
			}

			nextFractionX += deltaAlphaX;
		}
		else
		{
			if ( boxRowHead == rowEnd )
			{
				// Hit the end already
				break;
			}

			// Advance to next row
			boxRowHead += deltaRow;

			// Build a single row to cast
			boxRowTail = boxRowHead;

			if ( shapeExtents.x == B3_FIX( 0.0f ) )
			{
				// Single column
				boxColumnTail = boxColumnHead;
			}
			else
			{
				// Rasterize shape column
				b3Fixed columnIntercept = clampedStart.x + b3FixMul( nextFractionZ , clampedDelta.x );
				boxColumnTail = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( ( columnIntercept - b3FixMul( b3FixMul( B3_FIX( 2.0f ) , signX ) , shapeExtents.x ) ) , scale.x ) ) );
			}

			nextFractionZ += deltaAlphaZ;
		}
	}

	return result;
}

bool b3OverlapHeightField( const b3HeightFieldData* shape, b3Transform shapeTransform, const b3ShapeProxy* proxy )
{
	b3Vec3 buffer[B3_MAX_SHAPE_CAST_POINTS];
	b3ShapeProxy localProxy = b3MakeLocalProxy( proxy, shapeTransform, buffer );
	b3AABB aabb = b3ComputeProxyAABB( &localProxy );

	b3Vec3 scale = shape->scale;
	int minRow = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( aabb.lowerBound.z , scale.z ) ) );
	int maxRow = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( aabb.upperBound.z , scale.z ) ) );
	int minCol = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( aabb.lowerBound.x , scale.x ) ) );
	int maxCol = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( aabb.upperBound.x , scale.x ) ) );

	b3V32 boundsMin = b3LoadV( &aabb.lowerBound.x );
	b3V32 boundsMax = b3LoadV( &aabb.upperBound.x );
	b3V32 boundsCenter = b3MulV( b3_halfV, b3AddV( boundsMin, boundsMax ) );
	b3V32 boundsExtent = b3SubV( boundsMax, boundsCenter );

	b3DistanceInput input;
	input.proxyB = localProxy;
	input.transform = b3Transform_identity;
	input.useRadii = true;

	b3SimplexCache cache = { b3FixFromInt( 0 ) };

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for ( int row = minRow; row <= maxRow; ++row )
	{
		if ( row < 0 || shape->rowCount - 1 <= row )
		{
			continue;
		}

		for ( int column = minCol; column <= maxCol; ++column )
		{
			if ( column < 0 || shape->columnCount - 1 <= column )
			{
				continue;
			}

			int cellIndex = row * ( shape->columnCount - 1 ) + column;
			B3_ASSERT( cellIndex < ( shape->rowCount - 1 ) * ( shape->columnCount - 1 ) );
			uint8_t material = b3GetHeightFieldMaterialIndices( shape )[cellIndex];
			if ( material == B3_HEIGHT_FIELD_HOLE )
			{
				continue;
			}

			b3Vec3 corners[4];
			b3GetHeightFieldCellCorners( shape, row, column, corners );
			b3Vec3 point11 = corners[0];
			b3Vec3 point12 = corners[1];
			b3Vec3 point21 = corners[2];
			b3Vec3 point22 = corners[3];

			b3V32 v11 = b3LoadV( &point11.x );
			b3V32 v12 = b3LoadV( &point12.x );
			b3V32 v21 = b3LoadV( &point21.x );
			b3V32 v22 = b3LoadV( &point22.x );

			if ( b3TestBoundsTriangleOverlap( boundsCenter, boundsExtent, v11, v21, v12 ) )
			{
				b3Vec3 triangleVertices[] = { point11, point21, point12 };
				input.proxyA = (b3ShapeProxy){ triangleVertices, 3, B3_FIX( 0.0f ) };

				// reset the cache
				cache.count = 0;

				// get distance between triangle and query shape
				b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );

				b3Fixed tolerance = b3FixMul( B3_FIX( 0.1f ) , B3_LINEAR_SLOP );
				if ( output.distance < tolerance )
				{
					// overlap detected
					return true;
				}
			}

			if ( b3TestBoundsTriangleOverlap( boundsCenter, boundsExtent, v21, v22, v12 ) )
			{
				b3Vec3 triangleVertices[] = { point22, point12, point21 };
				input.proxyA = (b3ShapeProxy){ triangleVertices, 3, B3_FIX( 0.0f ) };

				// reset the cache
				cache.count = 0;

				// get distance between triangle and query shape
				b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );

				b3Fixed tolerance = b3FixMul( B3_FIX( 0.1f ) , B3_LINEAR_SLOP );
				if ( output.distance < tolerance )
				{
					// overlap detected
					return true;
				}
			}
		}
	}

	return false;
}

void b3QueryHeightField( const b3HeightFieldData* heightField, b3AABB bounds, b3MeshQueryFcn* fcn, void* context )
{
	b3Vec3 scale = heightField->scale;

	int minRow = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( bounds.lowerBound.z , scale.z ) ) );
	int maxRow = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( bounds.upperBound.z , scale.z ) ) );
	int minCol = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( bounds.lowerBound.x , scale.x ) ) );
	int maxCol = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( bounds.upperBound.x , scale.x ) ) );

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for ( int row = minRow; row <= maxRow; ++row )
	{
		if ( row < 0 || heightField->rowCount - 1 <= row )
		{
			continue;
		}

		for ( int column = minCol; column <= maxCol; ++column )
		{
			if ( column < 0 || heightField->columnCount - 1 <= column )
			{
				continue;
			}

			int cellIndex = row * ( heightField->columnCount - 1 ) + column;
			B3_ASSERT( cellIndex < ( heightField->rowCount - 1 ) * ( heightField->columnCount - 1 ) );
			uint8_t material = b3GetHeightFieldMaterialIndices( heightField )[cellIndex];
			if ( material == B3_HEIGHT_FIELD_HOLE )
			{
				continue;
			}

			b3Vec3 corners[4];
			b3GetHeightFieldCellCorners( heightField, row, column, corners );
			b3Vec3 point11 = corners[0];
			b3Vec3 point12 = corners[1];
			b3Vec3 point21 = corners[2];
			b3Vec3 point22 = corners[3];

			// I know the min/max x and z values, but not the min/max heights.
			// This could be done with no branching in SIMD.
			b3AABB cellBound;
			cellBound.lowerBound = b3Min( b3Min( point11, point12 ), b3Min( point21, point22 ) );
			cellBound.upperBound = b3Max( b3Max( point11, point12 ), b3Max( point21, point22 ) );

			if ( b3AABB_Overlaps( bounds, cellBound ) )
			{
				int quadIndex = row * ( heightField->columnCount - 1 ) + column;
				int triangleIndex = 2 * quadIndex;

				if ( heightField->clockwise )
				{
					fcn( point11, point12, point21, triangleIndex, context );
					fcn( point22, point21, point12, triangleIndex + 1, context );
				}
				else
				{
					fcn( point11, point21, point12, triangleIndex, context );
					fcn( point22, point12, point21, triangleIndex + 1, context );
				}
			}
		}
	}
}

int b3CollideMoverAndHeightField( b3PlaneResult* planes, int capacity, const b3HeightFieldData* shape, const b3Capsule* mover )
{
	b3DistanceInput distanceInput = { 0 };
	distanceInput.proxyB = (b3ShapeProxy){ &mover->center1, 2, B3_FIX( 0.0f ) };
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	b3SimplexCache cache = { b3FixFromInt( 0 ) };

	b3Fixed radius = mover->radius;
	b3V32 center1 = b3LoadV( &mover->center1.x );
	b3V32 center2 = b3LoadV( &mover->center2.x );
	b3V32 r = b3SplatV( radius );
	b3V32 boundsMin = b3SubV( b3MinV( center1, center2 ), r );
	b3V32 boundsMax = b3AddV( b3MaxV( center1, center2 ), r );
	b3V32 boundsCenter = b3MulV( b3_halfV, b3AddV( boundsMin, boundsMax ) );
	b3V32 boundsExtent = b3SubV( boundsMax, boundsCenter );

	b3Fixed localMinX = b3GetXV( boundsMin );
	b3Fixed localMinZ = b3GetZV( boundsMin );
	b3Fixed localMaxX = b3GetXV( boundsMax );
	b3Fixed localMaxZ = b3GetZV( boundsMax );

	b3Vec3 scale = shape->scale;
	int minRow = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( localMinZ , scale.z ) ) );
	int maxRow = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( localMaxZ , scale.z ) ) );
	int minCol = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( localMinX , scale.x ) ) );
	int maxCol = (int)b3FixTruncToInt( b3FixFloor( b3FixDiv( localMaxX , scale.x ) ) );

	int planeCount = 0;

	// Outer loop on rows and inner loop on columns so that triangle indices
	// increase monotonically.
	for ( int row = minRow; row <= maxRow; ++row )
	{
		if ( row < 0 || shape->rowCount - 1 <= row )
		{
			continue;
		}

		for ( int column = minCol; column <= maxCol; ++column )
		{
			if ( column < 0 || shape->columnCount - 1 <= column )
			{
				continue;
			}

			int cellIndex = row * ( shape->columnCount - 1 ) + column;
			B3_ASSERT( cellIndex < ( shape->rowCount - 1 ) * ( shape->columnCount - 1 ) );
			uint8_t material = b3GetHeightFieldMaterialIndices( shape )[cellIndex];
			if ( material == B3_HEIGHT_FIELD_HOLE )
			{
				continue;
			}

			b3Vec3 corners[4];
			b3GetHeightFieldCellCorners( shape, row, column, corners );
			b3Vec3 point11 = corners[0];
			b3Vec3 point12 = corners[1];
			b3Vec3 point21 = corners[2];
			b3Vec3 point22 = corners[3];

			b3V32 v11 = b3LoadV( &point11.x );
			b3V32 v12 = b3LoadV( &point12.x );
			b3V32 v21 = b3LoadV( &point21.x );
			b3V32 v22 = b3LoadV( &point22.x );

			if ( b3TestBoundsTriangleOverlap( boundsCenter, boundsExtent, v11, v21, v12 ) )
			{
				b3Vec3 triangleVertices[] = { point11, point21, point12 };
				distanceInput.proxyA = (b3ShapeProxy){ triangleVertices, 3, B3_FIX( 0.0f ) };

				// reset the cache
				cache.count = 0;

				// get distance between triangle and mover
				b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, NULL, 0 );

				if ( distanceOutput.distance == B3_FIX( 0.0f ) )
				{
					// todo SAT
				}
				else if ( distanceOutput.distance <= mover->radius )
				{
					b3Plane plane = { distanceOutput.normal, mover->radius - distanceOutput.distance };
					planes[planeCount] = (b3PlaneResult){ plane, distanceOutput.pointA };
					planeCount += 1;

					if ( planeCount == capacity )
					{
						return planeCount;
					}
				}
			}

			if ( b3TestBoundsTriangleOverlap( boundsCenter, boundsExtent, v21, v22, v12 ) )
			{
				b3Vec3 triangleVertices[] = { point22, point12, point21 };
				distanceInput.proxyA = (b3ShapeProxy){ triangleVertices, 3, B3_FIX( 0.0f ) };

				// reset the cache
				cache.count = 0;

				// get distance between triangle and mover
				b3DistanceOutput distanceOutput = b3ShapeDistance( &distanceInput, &cache, NULL, 0 );

				if ( distanceOutput.distance == B3_FIX( 0.0f ) )
				{
					// todo SAT
				}
				else if ( distanceOutput.distance <= mover->radius )
				{
					b3Plane plane = { distanceOutput.normal, mover->radius - distanceOutput.distance };
					planes[planeCount] = (b3PlaneResult){ plane, distanceOutput.pointA };
					planeCount += 1;

					if ( planeCount == capacity )
					{
						return planeCount;
					}
				}
			}
		}
	}

	return planeCount;
}

b3HeightFieldData* b3CreateGrid( int rowCount, int columnCount, b3Vec3 scale, bool makeHoles )
{
	int heightCount = rowCount * columnCount;
	b3Fixed* heights = (b3Fixed*)b3Alloc( heightCount * sizeof( b3Fixed ) );

	for ( int i = 0; i < rowCount; ++i )
	{
		for ( int j = 0; j < columnCount; ++j )
		{
			int k = i * columnCount + j;
			heights[k] = B3_FIX( 0.0f );
		}
	}

	int cellCount = ( rowCount - 1 ) * ( columnCount - 1 );
	uint8_t* materialIndices = (uint8_t*)b3Alloc( cellCount * sizeof( uint8_t ) );

	for ( int i = 0; i < rowCount - 1; ++i )
	{
		for ( int j = 0; j < columnCount - 1; ++j )
		{
			int k = i * ( columnCount - 1 ) + j;

			if ( makeHoles && k > 0 && k % 16 == 0 )
			{
				materialIndices[k] = B3_HEIGHT_FIELD_HOLE;
			}
			else
			{
				materialIndices[k] = 0;
			}
		}
	}

	b3HeightFieldDef data = { 0 };
	data.heights = heights;
	data.materialIndices = materialIndices;
	data.scale = scale;
	data.countX = columnCount;
	data.countZ = rowCount;
	data.globalMinimumHeight = -B3_FIX( 256.0f );
	data.globalMaximumHeight = B3_FIX( 256.0f );
	data.clockwiseWinding = false;

	b3HeightFieldData* heightField = b3CreateHeightField( &data );

	b3Free( heights, heightCount * sizeof( b3Fixed ) );
	b3Free( materialIndices, cellCount * sizeof( uint8_t ) );

	return heightField;
}

b3HeightFieldData* b3CreateWave( int rowCount, int columnCount, b3Vec3 scale, b3Fixed rowFrequency, b3Fixed columnFrequency,
							 bool makeHoles )
{
	int heightCount = rowCount * columnCount;
	b3Fixed* heights = (b3Fixed*)b3Alloc( heightCount * sizeof( b3Fixed ) );

	b3Fixed omegaZ = b3FixMul( b3FixMul( B3_FIX( 2.0f ) , B3_PI ) , rowFrequency );
	b3Fixed omegaX = b3FixMul( b3FixMul( B3_FIX( 2.0f ) , B3_PI ) , columnFrequency );

	for ( int i = 0; i < rowCount; ++i )
	{
		b3Fixed rowHeight = b3Sin( b3FixMul( omegaZ , b3FixFromInt( i ) ) );

		for ( int j = 0; j < columnCount; ++j )
		{
			int k = i * columnCount + j;
			b3Fixed columnHeight = b3Sin( b3FixMul( omegaX , b3FixFromInt( j ) ) );
			heights[k] = b3FixMul( rowHeight , columnHeight );
		}
	}

	int cellCount = ( rowCount - 1 ) * ( columnCount - 1 );
	uint8_t* materialIndices = (uint8_t*)b3Alloc( cellCount * sizeof( uint8_t ) );

	for ( int i = 0; i < rowCount - 1; ++i )
	{
		for ( int j = 0; j < columnCount - 1; ++j )
		{
			int k = i * ( columnCount - 1 ) + j;

			if ( makeHoles && k > 0 && k % 16 == 0 )
			{
				materialIndices[k] = B3_HEIGHT_FIELD_HOLE;
			}
			else
			{
				materialIndices[k] = 0;
			}
		}
	}

	b3HeightFieldDef data = { 0 };
	data.heights = heights;
	data.materialIndices = materialIndices;
	data.scale = scale;
	data.countX = columnCount;
	data.countZ = rowCount;
	data.globalMinimumHeight = -B3_FIX( 256.0f );
	data.globalMaximumHeight = B3_FIX( 256.0f );
	data.clockwiseWinding = false;

	b3HeightFieldData* heightField = b3CreateHeightField( &data );

	b3Free( heights, heightCount * sizeof( b3Fixed ) );
	b3Free( materialIndices, cellCount * sizeof( uint8_t ) );

	return heightField;
}

void b3DestroyHeightField( b3HeightFieldData* heightField )
{
	b3Free( heightField, heightField->byteCount );
}

void b3DumpHeightData( const b3HeightFieldDef* data, const char* fileName )
{
	FILE* file = NULL;

#if defined( _MSC_VER )
	errno_t e = fopen_s( &file, fileName, "w" );
	if ( e != 0 )
	{
		return;
	}
#else
	file = fopen( fileName, "w" );
	if ( file == NULL )
	{
		return;
	}
#endif

	fprintf( file, "%d %d\n", data->countX, data->countZ );
	fprintf( file, "%.9f %.9f %.9f\n", b3FixToDouble( data->scale.x ), b3FixToDouble( data->scale.y ), b3FixToDouble( data->scale.z ) );
	fprintf( file, "%.9f %.9f\n", b3FixToDouble( data->globalMinimumHeight ), b3FixToDouble( data->globalMaximumHeight ) );
	fprintf( file, "%d\n", data->clockwiseWinding );

	int heightCount = data->countX * data->countZ;
	for ( int i = 0; i < heightCount; ++i )
	{
		fprintf( file, "%.9f\n", b3FixToDouble( data->heights[i] ) );
	}

	int materialCount = ( data->countX - 1 ) * ( data->countZ - 1 );
	for ( int i = 0; i < materialCount; ++i )
	{
		fprintf( file, "%d\n", data->materialIndices[i] );
	}

	fclose( file );
}

#if defined( _MSC_VER )
#define B3_FILE_SCAN fscanf_s
#else
#define B3_FILE_SCAN fscanf
#endif

b3HeightFieldData* b3LoadHeightField( const char* fileName )
{
	FILE* file = NULL;

#if defined( _MSC_VER )
	errno_t e = fopen_s( &file, fileName, "r" );
	if ( e != 0 )
	{
		return NULL;
	}
#else
	file = fopen( fileName, "r" );
	if ( file == NULL )
	{
		return NULL;
	}
#endif

	b3HeightFieldDef data = { 0 };

	// Read dimensions
	if ( B3_FILE_SCAN( file, "%d %d", &data.countX, &data.countZ ) != 2 )
	{
		fclose( file );
		return NULL;
	}

	// Read scale. The file stores decimal text; parse into doubles and convert
	// to fixed point at the boundary.
	double scaleX, scaleY, scaleZ;
	if ( B3_FILE_SCAN( file, "%lf %lf %lf", &scaleX, &scaleY, &scaleZ ) != 3 )
	{
		fclose( file );
		return NULL;
	}
	data.scale = (b3Vec3){ b3FixFromDouble( scaleX ), b3FixFromDouble( scaleY ), b3FixFromDouble( scaleZ ) };

	// Read global height bounds
	double minHeight, maxHeight;
	if ( B3_FILE_SCAN( file, "%lf %lf", &minHeight, &maxHeight ) != 2 )
	{
		fclose( file );
		return NULL;
	}
	data.globalMinimumHeight = b3FixFromDouble( minHeight );
	data.globalMaximumHeight = b3FixFromDouble( maxHeight );

	// Read clockwise winding
	int clockwise;
	if ( B3_FILE_SCAN( file, "%d", &clockwise ) != 1 )
	{
		fclose( file );
		return NULL;
	}
	data.clockwiseWinding = clockwise != 0;

	// Allocate and read height data
	int heightCount = data.countX * data.countZ;
	data.heights = (b3Fixed*)b3Alloc( heightCount * sizeof( b3Fixed ) );

	for ( int i = 0; i < heightCount; ++i )
	{
		double height;
		if ( B3_FILE_SCAN( file, "%lf", &height ) != 1 )
		{
			b3Free( data.heights, heightCount * sizeof( b3Fixed ) );
			fclose( file );
			return NULL;
		}
		( (b3Fixed*)data.heights )[i] = b3FixFromDouble( height );
	}

	// Allocate and read material indices
	int materialCount = ( data.countX - 1 ) * ( data.countZ - 1 );
	data.materialIndices = (uint8_t*)b3Alloc( materialCount * sizeof( uint8_t ) );

	for ( int i = 0; i < materialCount; ++i )
	{
		int materialIndex;
		if ( B3_FILE_SCAN( file, "%d", &materialIndex ) != 1 )
		{
			b3Free( data.heights, heightCount * sizeof( b3Fixed ) );
			b3Free( data.materialIndices, materialCount * sizeof( uint8_t ) );
			fclose( file );
			return NULL;
		}
		data.materialIndices[i] = (uint8_t)materialIndex;
	}

	fclose( file );

	// Create height field from loaded data
	b3HeightFieldData* heightField = b3CreateHeightField( &data );

	// Clean up temporary allocations
	b3Free( data.heights, heightCount * sizeof( b3Fixed ) );
	b3Free( data.materialIndices, materialCount * sizeof( uint8_t ) );

	return heightField;
}
