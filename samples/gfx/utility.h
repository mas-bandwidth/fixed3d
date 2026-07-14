// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Math utilities: the 4-wide types Fixed3D does not provide.
//
// Vec4, linear-RGBA colors and matrix columns.
// Mat4, camera-space matrices (view / proj / view-proj / shadow).
//
// Per-shape transforms use b3Transform directly. Mat4 only appears for
// camera-space matrices. These two types may later move into Fixed3D, this
// header is written to make that a clean cut-paste.
//
// Conventions match the renderer and Fixed3D:
//   * Column-major storage, multiplied as M * v.
//   * Right-handed, Y-up. The renderer drives reverse-Z by passing near/far
//     swapped into the perspective builder.

#pragma once

#include "box3d/math_functions.h"
#include "box3d/types.h"

#include <math.h>

typedef struct Vec4
{
	float x, y, z, w;
} Vec4;

// Float pair for shader vec2 UBO fields and other display-space values.
// b3Vec2 is fixed point (two int64s) and no longer layout-compatible with
// a GPU vec2, use this on the float side of the renderer.
typedef struct Vec2
{
	float x, y;
} Vec2;

// Float triple for shader vec3 UBO fields (the shaders' @ctype vec3 maps
// here). b3Vec3 is fixed point and not GPU layout-compatible.
typedef struct Vec3
{
	float x, y, z;
} Vec3;

// Column-major 4x4. cx / cy / cz are the basis columns and cw the
// translation column, matching b3Matrix3's cx / cy / cz naming.
typedef struct Mat4
{
	Vec4 cx, cy, cz, cw;
} Mat4;

static inline Vec4 MakeVec4( float x, float y, float z, float w )
{
	Vec4 v = { x, y, z, w };
	return v;
}

static inline Mat4 MakeIdentity( void )
{
	Mat4 m = {
		{ 1.0f, 0.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f, 1.0f },
	};
	return m;
}

// Matrix times column-vector.
static inline Vec4 MulMV4( Mat4 m, Vec4 v )
{
	Vec4 r;
	r.x = m.cx.x * v.x + m.cy.x * v.y + m.cz.x * v.z + m.cw.x * v.w;
	r.y = m.cx.y * v.x + m.cy.y * v.y + m.cz.y * v.z + m.cw.y * v.w;
	r.z = m.cx.z * v.x + m.cy.z * v.y + m.cz.z * v.z + m.cw.z * v.w;
	r.w = m.cx.w * v.x + m.cy.w * v.y + m.cz.w * v.z + m.cw.w * v.w;
	return r;
}

// Matrix product: applying the result is equivalent to applying b then a.
static inline Mat4 MulMM4( Mat4 a, Mat4 b )
{
	Mat4 r;
	r.cx = MulMV4( a, b.cx );
	r.cy = MulMV4( a, b.cy );
	r.cz = MulMV4( a, b.cz );
	r.cw = MulMV4( a, b.cw );
	return r;
}

static inline Mat4 Transpose4( Mat4 m )
{
	Mat4 r;
	r.cx = MakeVec4( m.cx.x, m.cy.x, m.cz.x, m.cw.x );
	r.cy = MakeVec4( m.cx.y, m.cy.y, m.cz.y, m.cw.y );
	r.cz = MakeVec4( m.cx.z, m.cy.z, m.cz.z, m.cw.z );
	r.cw = MakeVec4( m.cx.w, m.cy.w, m.cz.w, m.cw.w );
	return r;
}

// Right-handed perspective with a 0..1 clip-Z. The renderer drives reverse-Z by passing
// nearPlane / farPlane swapped, which puts clip-Z = 1 at the near plane and 0 at the far.
//
// This is the finite-far form. The infinite-far variant (set cz.z = 0,
// cw.z = nearPlane in the reverse-Z arg order) would give strictly better
// precision at distance and remove `far` from the matrix entirely, per
// Upchurch & Desbrun's roundoff analysis.
static inline Mat4 MakePerspective( float fovY, float aspect, float nearPlane, float farPlane )
{
	Mat4 m = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };

	float cotangent = 1.0f / tanf( fovY / 2.0f );
	m.cx.x = cotangent / aspect;
	m.cy.y = cotangent;
	m.cz.w = -1.0f;
	m.cz.z = farPlane / ( nearPlane - farPlane );
	m.cw.z = ( nearPlane * farPlane ) / ( nearPlane - farPlane );
	return m;
}

// Same perspective as MakePerspective, but also fills the closed-form
// inverse from the same parameters. The general inverse of a perspective
// matrix is lossy. Building it analytically here keeps the un-project path
// more accurate.
static inline void MakePerspectiveAndInverse( Mat4* outProj, Mat4* outProjInv, float fovY, float aspect, float nearPlane,
											  float farPlane )
{
	float tanHalf = tanf( fovY / 2.0f );
	float cotangent = 1.0f / tanHalf;

	// c, d are the two non-trivial slots of the forward matrix.
	float c = farPlane / ( nearPlane - farPlane );
	float d = ( nearPlane * farPlane ) / ( nearPlane - farPlane );

	Mat4 p = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };
	p.cx.x = cotangent / aspect;
	p.cy.y = cotangent;
	p.cz.z = c;
	p.cz.w = -1.0f;
	p.cw.z = d;
	*outProj = p;

	Mat4 pi = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };
	pi.cx.x = aspect * tanHalf;
	pi.cy.y = tanHalf;
	pi.cz.w = 1.0f / d;
	pi.cw.z = -1.0f;
	pi.cw.w = c / d;
	*outProjInv = pi;
}

// Right-handed orthographic projection with a 0..1 clip-Z. Used for the shadow cascade light matrices.
static inline Mat4 MakeOrthographic( float left, float right, float bottom, float top, float nearPlane, float farPlane )
{
	Mat4 m = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };
	m.cx.x = 2.0f / ( right - left );
	m.cy.y = 2.0f / ( top - bottom );
	m.cz.z = 1.0f / ( nearPlane - farPlane );
	m.cw.w = 1.0f;
	m.cw.x = ( left + right ) / ( left - right );
	m.cw.y = ( bottom + top ) / ( bottom - top );
	m.cw.z = nearPlane / ( nearPlane - farPlane );
	return m;
}

// Right-handed look-at. Inputs are eye-relative fixed point; entries cross to
// float by value (see MakeVec4FromFixed below).
static inline Mat4 MakeLookAt( b3Vec3 eye, b3Vec3 center, b3Vec3 up )
{
	b3Vec3 f = b3Normalize( b3Sub( center, eye ) );
	b3Vec3 s = b3Normalize( b3Cross( f, up ) );
	b3Vec3 u = b3Cross( s, f );

	Mat4 m;
	m.cx = MakeVec4( b3FixToFloat( s.x ), b3FixToFloat( u.x ), -b3FixToFloat( f.x ), 0.0f );
	m.cy = MakeVec4( b3FixToFloat( s.y ), b3FixToFloat( u.y ), -b3FixToFloat( f.y ), 0.0f );
	m.cz = MakeVec4( b3FixToFloat( s.z ), b3FixToFloat( u.z ), -b3FixToFloat( f.z ), 0.0f );
	m.cw = MakeVec4( -b3FixToFloat( b3Dot( s, eye ) ), -b3FixToFloat( b3Dot( u, eye ) ), b3FixToFloat( b3Dot( f, eye ) ),
					 1.0f );
	return m;
}

// Build a view matrix and its inverse together from the camera basis. `right`,
// `up` and `forward` are the world-space view-X / view-Y / view-Z directions:
// forward is +view-Z (pivot -> eye), so
// the camera looks down -forward. The basis must be orthonormal and right-
// handed (right = cross(up, forward)), both outputs assume that.
//
// viewInv is the camera's "world matrix": columns are right / up / forward /
// eye. view is its transpose-with-shifted-translation form. Neither requires
// any inversion at runtime.
// Fixed -> float VALUE conversion for the render boundary. The gfx float
// pipeline works in eye-relative display meters; b3Fixed values must cross by
// value (b3FixToFloat), never as raw int64 bits. Raw casts silently scale by
// 65536 per factor — the bug class that broke rendering after the fixed-point
// conversion. Positions must be differenced against the draw origin in fixed
// point (exact) BEFORE they cross, so the floats here only ever see small
// eye-relative values.
static inline Vec4 MakeVec4FromFixed( b3Fixed x, b3Fixed y, b3Fixed z, float w )
{
	return MakeVec4( b3FixToFloat( x ), b3FixToFloat( y ), b3FixToFloat( z ), w );
}

static inline void MakeViewAndInverse( Mat4* outView, Mat4* outViewInv, b3Vec3 right, b3Vec3 up, b3Vec3 forward, b3Vec3 eye )
{
	Mat4 v;
	v.cx = MakeVec4FromFixed( right.x, up.x, forward.x, 0.0f );
	v.cy = MakeVec4FromFixed( right.y, up.y, forward.y, 0.0f );
	v.cz = MakeVec4FromFixed( right.z, up.z, forward.z, 0.0f );
	v.cw = MakeVec4( -b3FixToFloat( b3Dot( right, eye ) ), -b3FixToFloat( b3Dot( up, eye ) ),
					 -b3FixToFloat( b3Dot( forward, eye ) ), 1.0f );
	*outView = v;

	Mat4 vi;
	vi.cx = MakeVec4FromFixed( right.x, right.y, right.z, 0.0f );
	vi.cy = MakeVec4FromFixed( up.x, up.y, up.z, 0.0f );
	vi.cz = MakeVec4FromFixed( forward.x, forward.y, forward.z, 0.0f );
	vi.cw = MakeVec4FromFixed( eye.x, eye.y, eye.z, 1.0f );
	*outViewInv = vi;
}

// Build a 4x4 model matrix from a Fixed3D rigid transform plus a (possibly
// non-uniform) scale. The transform position must already be eye-relative
// (differenced against the draw origin in fixed point); it crosses to float
// by value here.
static inline Mat4 MakeMat4FromTransform( b3Transform t, b3Vec3 scale )
{
	b3Matrix3 r = b3MakeMatrixFromQuat( t.q );
	float sx = b3FixToFloat( scale.x );
	float sy = b3FixToFloat( scale.y );
	float sz = b3FixToFloat( scale.z );
	Mat4 m;
	m.cx = MakeVec4( sx * b3FixToFloat( r.cx.x ), sx * b3FixToFloat( r.cx.y ), sx * b3FixToFloat( r.cx.z ), 0.0f );
	m.cy = MakeVec4( sy * b3FixToFloat( r.cy.x ), sy * b3FixToFloat( r.cy.y ), sy * b3FixToFloat( r.cy.z ), 0.0f );
	m.cz = MakeVec4( sz * b3FixToFloat( r.cz.x ), sz * b3FixToFloat( r.cz.y ), sz * b3FixToFloat( r.cz.z ), 0.0f );
	m.cw = MakeVec4FromFixed( t.p.x, t.p.y, t.p.z, 1.0f );
	return m;
}

static inline float SrgbToLinear( float c )
{
	return c <= 0.04045f ? c / 12.92f : powf( ( c + 0.055f ) / 1.055f, 2.4f );
}

static inline Vec4 MakeColor( b3HexColor hexColor )
{
	uint32_t v = (uint32_t)hexColor;
	float k = 1.0f / 255.0f;
	float r = (float)( ( v >> 16 ) & 0xFFu ) * k;
	float g = (float)( ( v >> 8 ) & 0xFFu ) * k;
	float b = (float)( v & 0xFFu ) * k;
	return MakeVec4( SrgbToLinear( r ), SrgbToLinear( g ), SrgbToLinear( b ), 1.0f );
}

static inline Vec4 MakeColorAlpha( b3HexColor hexColor, float alpha )
{
	uint32_t v = (uint32_t)hexColor;
	float k = 1.0f / 255.0f;
	float r = (float)( ( v >> 16 ) & 0xFFu ) * k;
	float g = (float)( ( v >> 8 ) & 0xFFu ) * k;
	float b = (float)( v & 0xFFu ) * k;
	return MakeVec4( SrgbToLinear( r ), SrgbToLinear( g ), SrgbToLinear( b ), alpha );
}
