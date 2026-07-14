// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/shadow.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

// Cascade fitting parameters. SHADOW_SPLIT_NEAR and the default far
// (SHADOW_SPLIT_FAR, in shadow.h) are *view-space* distances from the camera
// (positive). CSM quality at distance falls off fast, so cascades focus on
// near content.
//
// PSSM_LAMBDA blends the uniform split scheme (each cascade covers an
// equal slice of view-space distance) and the logarithmic scheme (each
// cascade covers an equal multiplicative range). 0 = pure uniform,
// 1 = pure log. 0.5 is the sweet spot at the default range. The blend is
// pushed toward log up to PSSM_LAMBDA_MAX as the range widens, otherwise a
// large far would balloon the near cascade and coarsen close-up shadows.
//
// CASTER_MARGIN_M expands the orthographic light frustum's near edge
// toward the sun so casters above the camera frustum slice (e.g. the
// building roof at y=3 when the slice itself only reaches y=1) still cast
// onto receivers inside the slice.
#define SHADOW_SPLIT_NEAR 0.1f
#define PSSM_LAMBDA 0.5f
#define PSSM_LAMBDA_MAX 0.9f
#define CASTER_MARGIN_M 50.0f

// Float 3-vector for the cascade fitting math. Frustum corners, slice
// centers, and the light frame all derive from the float camera matrices,
// whose translations are already eye-relative display meters — small
// values, so plain float math is correct end to end. The only fixed-point
// input (the sun direction, a unit vector) crosses by value at entry via
// b3FixToFloat.
typedef struct FVec3
{
	float x, y, z;
} FVec3;

static inline FVec3 FVec3Add( FVec3 a, FVec3 b )
{
	return (FVec3){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static inline FVec3 FVec3Sub( FVec3 a, FVec3 b )
{
	return (FVec3){ a.x - b.x, a.y - b.y, a.z - b.z };
}

// a + s * b
static inline FVec3 FVec3MulAdd( FVec3 a, float s, FVec3 b )
{
	return (FVec3){ a.x + s * b.x, a.y + s * b.y, a.z + s * b.z };
}

static inline float FVec3Dot( FVec3 a, FVec3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline FVec3 FVec3Cross( FVec3 a, FVec3 b )
{
	return (FVec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

static inline FVec3 FVec3Normalize( FVec3 v )
{
	const float length = sqrtf( FVec3Dot( v, v ) );
	if ( length > 0.0f )
	{
		const float inv = 1.0f / length;
		return (FVec3){ v.x * inv, v.y * inv, v.z * inv };
	}
	return v;
}

static struct
{
	b3Vec3 sunDir;										  // world-space dir TO sun, normalized (fixed point)
	float splitNear;									  // PSSM split range near (positive view-Z)
	float splitFar;										  // PSSM split range far (positive view-Z)
	float splitLambda;									  // uniform/log split blend, widens with range
	sg_image depthArray;								  // D32F, cascade layers
	sg_view cascadeAttachmentViews[SHADOW_CASCADE_COUNT]; // depth-stencil view per cascade
	sg_view sampleView;									  // full-array texture-sample view
	sg_sampler sampler;									  // comparison sampler (LESS_EQUAL, linear PCF)
	Mat4 cascadeMatrices[SHADOW_CASCADE_COUNT];			  // light-space view-proj per cascade
	float cascadeFarViewZ[SHADOW_CASCADE_COUNT];		  // far end (positive view-Z) per cascade
	float cascadeRadius[SHADOW_CASCADE_COUNT];			  // bounding-sphere radius = ortho half-extent
	bool inPass;										  // safety against unbalanced begin/end
} s_shadow;

// PSSM split scheme: blend uniform and logarithmic. Returns the *positive*
// view-space distance at the boundary of cascade `i` (i in [0..N], where
// split[0] = near and split[N] = far).
static inline float ComputeSplitDistance( int i, int cascadeCount, float nearZ, float farZ )
{
	const float fi = (float)i / (float)cascadeCount;
	const float u = nearZ + ( farZ - nearZ ) * fi;
	const float l = nearZ * powf( farZ / nearZ, fi );
	return s_shadow.splitLambda * l + ( 1.0f - s_shadow.splitLambda ) * u;
}

// Push the uniform/log blend toward log as the split range widens, so the near
// cascade stays tight when far is large. Anchored at the default range so the
// reference scenes keep the 0.5 sweet spot.
static inline float SplitLambdaForRange( float nearZ, float farZ )
{
	float decades = log10f( ( farZ / nearZ ) / ( SHADOW_SPLIT_FAR / SHADOW_SPLIT_NEAR ) );
	return b3ClampFloat( PSSM_LAMBDA + 0.4f * decades, PSSM_LAMBDA, PSSM_LAMBDA_MAX );
}

// Fill cornersOut with the 8 world-space corners of the camera frustum
// slice between view-space distances nearZ and farZ (both positive, the
// camera looks down its own -Z axis in view space).
//
// Camera basis comes straight from `viewInv` (the camera's world matrix
// produced by mat4ViewAndInverse): column 0 is right, column 1 is up,
// column 2 is back (i.e., -forward), column 3 is eye, no inversion.
// FovY and aspect are read from the projection matrix: mat4Perspective
// puts proj[1][1] = 1/tan(fovY/2) and proj[0][0] = proj[1][1] / aspect.
static void FrustumSliceCornersWorld( const Mat4* viewInv, const Mat4* proj, float nearZ, float farZ, FVec3 cornersOut[8] )
{
	const FVec3 eye = { viewInv->cw.x, viewInv->cw.y, viewInv->cw.z };
	const FVec3 right = { viewInv->cx.x, viewInv->cx.y, viewInv->cx.z };
	const FVec3 up = { viewInv->cy.x, viewInv->cy.y, viewInv->cy.z };
	const FVec3 forward = { -viewInv->cz.x, -viewInv->cz.y, -viewInv->cz.z };

	const float tanHalfFovY = ( proj->cy.y != 0.0f ) ? ( 1.0f / proj->cy.y ) : 1.0f;
	const float aspect = ( proj->cx.x != 0.0f ) ? ( proj->cy.y / proj->cx.x ) : 1.0f;

	const float distances[2] = { nearZ, farZ };
	for ( int s = 0; s < 2; ++s )
	{
		const float d = distances[s];
		const float h = d * tanHalfFovY;
		const float w = h * aspect;
		const FVec3 center = FVec3MulAdd( eye, d, forward );
		for ( int j = 0; j < 4; ++j )
		{
			const float sx = ( j & 1 ) ? +w : -w;
			const float sy = ( j & 2 ) ? +h : -h;
			FVec3 c = FVec3MulAdd( center, sx, right );
			c = FVec3MulAdd( c, sy, up );
			cornersOut[s * 4 + j] = c;
		}
	}
}

// Right-handed look-at over the file-local float vectors. Mirrors
// utility.h's MakeLookAt, but every operand here is an eye-relative float
// quantity — nothing crosses the fixed/float boundary.
static Mat4 MakeLookAtFloat( FVec3 eye, FVec3 center, FVec3 up )
{
	const FVec3 f = FVec3Normalize( FVec3Sub( center, eye ) );
	const FVec3 s = FVec3Normalize( FVec3Cross( f, up ) );
	const FVec3 u = FVec3Cross( s, f );

	Mat4 m;
	m.cx = MakeVec4( s.x, u.x, -f.x, 0.0f );
	m.cy = MakeVec4( s.y, u.y, -f.y, 0.0f );
	m.cz = MakeVec4( s.z, u.z, -f.z, 0.0f );
	m.cw = MakeVec4( -FVec3Dot( s, eye ), -FVec3Dot( u, eye ), FVec3Dot( f, eye ), 1.0f );
	return m;
}

// Fit one cascade's light-space view-projection matrix to the camera
// frustum slice [nearZ, farZ] for the configured sun direction.
//
// The slice is bounded by a *world-space sphere* (centroid + max radius
// from centroid to any slice corner). The sphere is rotation-invariant.
// Rotating the camera around its eye doesn't move it, which is the key
// property that lets the texel-snap stabilization avoid swimming edges.
//
// Light view: orthonormal frame whose -Z axis points toward the sun.
// Light proj: orthographic, sized to enclose the sphere. The near plane
// is pushed back along +sun by CASTER_MARGIN_M so casters between the
// slice and the sun are not clipped.
//
// Texel-snap: after the unsnapped lightVP is built, transform the slice
// center through it to clip space, round the resulting clip-space xy to
// the nearest texel grid step, and apply the rounding delta as a
// post-translation. This pins the cascade to a fixed grid in light
// space, small camera motions no longer cause the shadow texels to
// drift relative to the casters.
static Mat4 FitCascade( const Mat4* viewInv, const Mat4* proj, b3Vec3 dirToSun, float nearZ, float farZ, float* radiusOut )
{
	// The sun direction is a normalized fixed-point vector; unit directions
	// are small, so it crosses to float by value here.
	const FVec3 sun = { b3FixToFloat( dirToSun.x ), b3FixToFloat( dirToSun.y ), b3FixToFloat( dirToSun.z ) };

	FVec3 corners[8];
	FrustumSliceCornersWorld( viewInv, proj, nearZ, farZ, corners );

	FVec3 center = { 0.0f, 0.0f, 0.0f };
	for ( int i = 0; i < 8; ++i )
	{
		center = FVec3Add( center, corners[i] );
	}
	center = (FVec3){ center.x * ( 1.0f / 8.0f ), center.y * ( 1.0f / 8.0f ), center.z * ( 1.0f / 8.0f ) };

	float radius = 0.0f;
	for ( int i = 0; i < 8; ++i )
	{
		const FVec3 d = FVec3Sub( corners[i], center );
		const float r = sqrtf( FVec3Dot( d, d ) );
		if ( r > radius )
		{
			radius = r;
		}
	}
	if ( radius < 0.01f )
	{
		radius = 0.01f;
	}
	*radiusOut = radius;

	// Avoid a degenerate up vector when the sun is straight up/down.
	FVec3 up = { 0.0f, 1.0f, 0.0f };
	if ( fabsf( sun.y ) > 0.999f )
	{
		up = (FVec3){ 0.0f, 0.0f, 1.0f };
	}

	// Place the light eye behind the sphere along +sun. Far clip extends
	// past the sphere's back face. Near clip pulls forward by
	// CASTER_MARGIN_M to admit casters above the slice.
	const FVec3 eyeLight = FVec3MulAdd( center, radius + CASTER_MARGIN_M, sun );
	const Mat4 lightView = MakeLookAtFloat( eyeLight, center, up );
	const Mat4 lightProj = MakeOrthographic( -radius, radius, -radius, radius, 0.0f, 2.0f * radius + CASTER_MARGIN_M );
	Mat4 lightVP = MulMM4( lightProj, lightView );

	// Texel-snap stabilization. Transform the sphere center through the
	// unsnapped lightVP. Project to texel coords, round, convert the
	// delta back to clip space, and add as a post-translation.
	const Vec4 centerClip = MulMV4( lightVP, MakeVec4( center.x, center.y, center.z, 1.0f ) );
	const float halfRes = (float)SHADOW_RESOLUTION * 0.5f;
	const float texX = centerClip.x * halfRes;
	const float texY = centerClip.y * halfRes;
	const float dx = ( roundf( texX ) - texX ) / halfRes;
	const float dy = ( roundf( texY ) - texY ) / halfRes;

	Mat4 snapMat = MakeIdentity();
	snapMat.cw.x = dx;
	snapMat.cw.y = dy;
	return MulMM4( snapMat, lightVP );
}

void InitShadows( void )
{
	s_shadow.sunDir = b3Normalize( (b3Vec3){ B3_FIX( 0.5f ), B3_FIX( 0.8f ), B3_FIX( 0.4f ) } );
	s_shadow.splitNear = SHADOW_SPLIT_NEAR;
	s_shadow.splitFar = SHADOW_SPLIT_FAR;
	s_shadow.splitLambda = SplitLambdaForRange( s_shadow.splitNear, s_shadow.splitFar );

	// Sane ratios if a getter runs before the first FitShadows
	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		s_shadow.cascadeRadius[i] = 1.0f;
	}

	sg_image_desc desc = { 0 };
	desc.type = SG_IMAGETYPE_ARRAY;
	desc.usage.depth_stencil_attachment = true;
	desc.width = SHADOW_RESOLUTION;
	desc.height = SHADOW_RESOLUTION;
	desc.num_slices = SHADOW_CASCADE_COUNT;
	desc.pixel_format = SG_PIXELFORMAT_DEPTH;
	desc.sample_count = 1;
	desc.label = "shadow_depth_array";
	s_shadow.depthArray = sg_make_image( &desc );

	// Per-cascade depth attachment view (slice = cascade index).
	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		sg_view_desc vdesc = { 0 };
		vdesc.depth_stencil_attachment.image = s_shadow.depthArray;
		vdesc.depth_stencil_attachment.slice = i;
		vdesc.label = "shadow_cascade_attachment";
		s_shadow.cascadeAttachmentViews[i] = sg_make_view( &vdesc );
	}

	// Whole-array texture sample view: layer index becomes the .z arg in
	// the shader's sampler2DArrayShadow lookup. Explicitly range over all
	// cascade slices so the view binds the full array
	// (careful: sokol's "count = 0 means all remaining" default).
	sg_view_desc svdesc = { 0 };
	svdesc.texture.image = s_shadow.depthArray;
	svdesc.texture.slices.base = 0;
	svdesc.texture.slices.count = SHADOW_CASCADE_COUNT;
	svdesc.label = "shadow_sample_view";
	s_shadow.sampleView = sg_make_view( &svdesc );

	// Comparison sampler: LESS_EQUAL means the test returns 1.0 (lit) when
	// the receiver's reference depth is <= the stored depth, i.e., the
	// receiver is the closest surface to the light. Linear PCF: each
	// texture call returns a 2x2 weighted compare-result mix.
	sg_sampler_desc sdesc = { 0 };
	sdesc.min_filter = SG_FILTER_LINEAR;
	sdesc.mag_filter = SG_FILTER_LINEAR;
	sdesc.mipmap_filter = SG_FILTER_NEAREST;
	sdesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.compare = SG_COMPAREFUNC_LESS_EQUAL;
	sdesc.label = "shadow_compare_sampler";
	s_shadow.sampler = sg_make_sampler( &sdesc );

	s_shadow.inPass = false;
}

void ShutdownShadows( void )
{
	sg_destroy_sampler( s_shadow.sampler );
	sg_destroy_view( s_shadow.sampleView );
	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		sg_destroy_view( s_shadow.cascadeAttachmentViews[i] );
	}
	sg_destroy_image( s_shadow.depthArray );
}

void SetShadowSplits( float nearViewZ, float farViewZ )
{
	if ( nearViewZ <= 0.0f || farViewZ <= nearViewZ )
	{
		// Restore defaults on (0, 0) or any nonsense input.
		nearViewZ = SHADOW_SPLIT_NEAR;
		farViewZ = SHADOW_SPLIT_FAR;
	}

	s_shadow.splitNear = nearViewZ;
	s_shadow.splitFar = farViewZ;
	s_shadow.splitLambda = SplitLambdaForRange( nearViewZ, farViewZ );
}

void SetShadowSplitFar( float farViewZ )
{
	SetShadowSplits( s_shadow.splitNear, farViewZ );
}

void SetShadowSunDir( b3Vec3 dirToSun )
{
	// Nonzero guard in the fixed domain. Component compares are exact;
	// a squared length would quantize to zero for sub-resolution inputs.
	if ( dirToSun.x != 0 || dirToSun.y != 0 || dirToSun.z != 0 )
	{
		s_shadow.sunDir = b3Normalize( dirToSun );
	}
}

void FitShadows( const Mat4* viewInv, const Mat4* proj )
{
	// PSSM splits across [splitNear, splitFar]. split[0] = near,
	// split[N] = far. Cascade i covers (split[i], split[i+1]].
	float splits[SHADOW_CASCADE_COUNT + 1];
	for ( int i = 0; i <= SHADOW_CASCADE_COUNT; ++i )
	{
		splits[i] = ComputeSplitDistance( i, SHADOW_CASCADE_COUNT, s_shadow.splitNear, s_shadow.splitFar );
	}

	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		s_shadow.cascadeMatrices[i] = FitCascade( viewInv, proj, s_shadow.sunDir, splits[i], splits[i + 1], &s_shadow.cascadeRadius[i] );
		s_shadow.cascadeFarViewZ[i] = splits[i + 1];
	}
}

void BeginShadowPass( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		fprintf( stderr, "cascade index out of range: %d\n", cascade );
		return;
	}
	sg_pass pass = { 0 };
	pass.action.depth.load_action = SG_LOADACTION_CLEAR;
	pass.action.depth.clear_value = 1.0f; // standard Z far
	pass.action.depth.store_action = SG_STOREACTION_STORE;
	pass.attachments.depth_stencil = s_shadow.cascadeAttachmentViews[cascade];
	sg_begin_pass( &pass );
	s_shadow.inPass = true;
}

void EndShadowPass( void )
{
	if ( !s_shadow.inPass )
	{
		return;
	}
	sg_end_pass();
	s_shadow.inPass = false;
}

Mat4 GetCascadeMatrix( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return MakeIdentity();
	}
	return s_shadow.cascadeMatrices[cascade];
}

float GetCascadeFarViewZ( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return 0.0f;
	}
	return s_shadow.cascadeFarViewZ[cascade];
}

float GetCascadePcfScale( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return 1.0f;
	}
	// Texel world size is proportional to the cascade's ortho half-extent,
	// so the radius ratio is the texel-size ratio. The floor keeps the
	// shrunken kernel at least ~2 texels wide so bilinear smoothing still
	// gets some support in the far cascades.
	const float scale = s_shadow.cascadeRadius[0] / s_shadow.cascadeRadius[cascade];
	return b3ClampFloat( scale, 0.25f, 1.0f );
}

sg_view GetShadowSampleView( void )
{
	return s_shadow.sampleView;
}

sg_sampler GetShadowSampler( void )
{
	return s_shadow.sampler;
}

float GetShadowTexelSize( void )
{
	return 1.0f / (float)SHADOW_RESOLUTION;
}
