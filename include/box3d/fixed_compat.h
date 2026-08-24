// SPDX-FileCopyrightText: 2025-2026 Erin Catto -- derived from Box3D
// SPDX-FileCopyrightText: 2026 Más Bandwidth LLC -- fixed-point conversion
// SPDX-License-Identifier: MIT
//
// Box3D's names for the scalar core of mas-bandwidth/fixed.
//
// The Q48.16 type and its arithmetic used to be defined here. They now live in the
// `fixed` library, vendored at extern/fixed and pinned in tools/revendor-fixed.sh,
// where they are called fixed_t, fixMul, FIX(x). box3d goes on calling them b3Fixed,
// b3FixMul and B3_FIX, because STAYING DROP-IN COMPATIBLE WITH BOX3D IS A DESIGN GOAL
// of this fork -- the public interface is not ours to evolve to suit a dependency.
//
// The bulk of this file is GENERATED: every name in the sections marked so was read out
// of the previous box3d/fixed.h and mapped through the rename that moved the library off
// the b3 prefix, so that set is exactly what box3d defined before -- no more, no less.
// Sections marked ADDED carry the same mapping applied by hand to parts of `fixed` that
// box3d had no name for yet, and they follow the same rule: box3d's spelling, the
// library's arithmetic, no second implementation anywhere.
//
// Forwarders rather than #defines: a macro would stop b3FixMul being a declared
// identifier, so debuggers would show fixMul and a consumer could no longer name
// anything b3FixMul. All of these were header-inline in box3d and stay header-inline,
// so linkage is unchanged; after inlining the wrapper costs nothing.
#pragma once

// ---- DOWN: hand box3d's plumbing to the library ------------------------------------
// FIX_API is the one that MATTERS, and CI is why it is here. box3d builds with
// -fvisibility=hidden. `fixed` exports fixAtan2 and fixComputeCosSin as FIX_API, which
// falls back to a bare extern "C" with no export decoration, so they compile into
// libbox3d as HIDDEN symbols. The inline forwarders below are in a public header, so a
// consumer inlines them and ends up referencing fixAtan2 directly -- and the samples
// failed to link against the shared library with "Undefined symbols: _fixAtan2".
//
// Static builds never noticed. Only samples-macos-dynamic and samples-windows-dynamic
// caught it, which is the argument for having dynamic-link jobs at all.
//
// Handing B3_API down gives those symbols box3d's export decoration, so they leave the
// shared library like every other part of box3d's math.
//
// This block could not live here before: base.h used to include fixed.h ABOVE its own
// definition of B3_API. Moving that include below the macro block is what made this
// possible.
// All of them together, not just FIX_API: the library guards its whole macro block on
// `#ifndef FIX_API`, so defining that one alone skips the definitions of FIX_INLINE,
// FIX_FORCE_INLINE, FIX_LITERAL and FIX_ZERO_INIT with it. (An each-macro-guarded
// version upstream would be better and is worth doing; handing the set down is correct
// either way, because these are box3d's compiler conventions and box3d should own them.)
#define FIX_API          B3_API
#define FIX_INLINE       B3_INLINE
#define FIX_FORCE_INLINE B3_FORCE_INLINE
#define FIX_LITERAL      B3_LITERAL
#define FIX_ZERO_INIT    B3_ZERO_INIT
#define FIX_ASSERT       B3_ASSERT
#define FIX_VALIDATE     B3_VALIDATE

#include "fixed/fixed.h"

#if defined( BOX3D_FIXED_SATURATE ) && !defined( FIX_SATURATE )
	#define FIX_SATURATE
#endif

// ---- types ------------------------------------------------------------------------
typedef fixed_t                b3Fixed;
typedef fixInt128              b3Int128;
typedef fixUInt128             b3UInt128;

// ---- macros -----------------------------------------------------------------------
#define B3_FIX                     FIX
#define B3_FIXED_EPSILON           FIX_EPSILON
#define B3_FIXED_FRACTION_BITS     FIX_FRACTION_BITS
#define B3_FIXED_HALF              FIX_HALF
#define B3_FIXED_INLINE            FIX_ALWAYS_INLINE
#define B3_FIXED_MAX               FIX_MAX
#define B3_FIXED_MIN               FIX_MIN
#define B3_FIXED_ONE               FIX_ONE
#define B3_HAS_INT128              FIX_HAS_INT128

// ---- forwarders -------------------------------------------------------------------
B3_FIXED_INLINE fixed_t b3FixAbs( fixed_t a ) { return fixAbs( a ); }
B3_FIXED_INLINE fixed_t b3FixCeil( fixed_t a ) { return fixCeil( a ); }
B3_FIXED_INLINE fixed_t b3FixClamp( fixed_t a, fixed_t lower, fixed_t upper ) { return fixClamp( a, lower, upper ); }
B3_FIXED_INLINE fixed_t b3FixDiv( fixed_t a, fixed_t b ) { return fixDiv( a, b ); }
B3_FIXED_INLINE fixed_t b3FixFloor( fixed_t a ) { return fixFloor( a ); }
B3_FIXED_INLINE int b3FixFloorToInt( fixed_t a ) { return fixFloorToInt( a ); }
B3_FIXED_INLINE fixed_t b3FixFromDouble( double x ) { return fixFromDouble( x ); }
B3_FIXED_INLINE fixed_t b3FixFromFloat( float x ) { return fixFromFloat( x ); }
B3_FIXED_INLINE fixed_t b3FixFromInt( int64_t i ) { return fixFromInt( i ); }
B3_FIXED_INLINE fixed_t b3FixMax( fixed_t a, fixed_t b ) { return fixMax( a, b ); }
B3_FIXED_INLINE fixed_t b3FixMin( fixed_t a, fixed_t b ) { return fixMin( a, b ); }
B3_FIXED_INLINE fixed_t b3FixMul( fixed_t a, fixed_t b ) { return fixMul( a, b ); }
B3_FIXED_INLINE int b3FixRoundToInt( fixed_t a ) { return fixRoundToInt( a ); }
B3_FIXED_INLINE fixed_t b3FixShiftLeft( fixed_t a, int shift ) { return fixShiftLeft( a, shift ); }
B3_FIXED_INLINE fixed_t b3FixSqrt( fixed_t a ) { return fixSqrt( a ); }
B3_FIXED_INLINE double b3FixToDouble( fixed_t a ) { return fixToDouble( a ); }
B3_FIXED_INLINE float b3FixToFloat( fixed_t a ) { return fixToFloat( a ); }
B3_FIXED_INLINE int b3FixTruncToInt( fixed_t a ) { return fixTruncToInt( a ); }
B3_FIXED_INLINE uint64_t b3ISqrt128High( uint64_t hi, uint64_t lo ) { return fixISqrt128High( hi, lo ); }
B3_FIXED_INLINE fixInt128 b3Int128Div( fixInt128 a, fixInt128 b ) { return fixInt128Div( a, b ); }
B3_FIXED_INLINE fixInt128 b3Int128ShiftLeft( fixInt128 a, int shift ) { return fixInt128ShiftLeft( a, shift ); }

// ---- ADDED: the Q2.30 packed scalar domain ----------------------------------------
// A SECOND raw fixed-point domain: 32-bit storage, 2 integer bits (sign included), 30
// fraction bits. Built for always-normalized quantities -- a quaternion component never
// leaves [-1, 1], so nearly every bit can go to fraction. box3d's use for it is
// b3Quat30 (math_functions.h), the storage form the game puts on the wire.
//
// The type is a STRUCT on purpose, upstream and here: b3Fixed is a bare int64 typedef
// and the Q48.16 and Q2.30 raws differ by 2^14, so a bare 32-bit typedef would let a
// mixup compile silently -- the same trap as assigning a double into a b3Fixed. Wrapped,
// arithmetic and cross-domain assignment refuse to compile and the converters below are
// the only way across.
//
// ROUNDING RULE (pinned upstream): dropping bits rounds to nearest via
// ( raw + ( 1 << ( drop - 1 ) ) ) >> drop, the same form as the wire's quantizers.
typedef fixed30_t              b3Fixed30;

#define B3_FIXED30_FRACTION_BITS   FIX30_FRACTION_BITS
#define B3_FIXED30_ONE             FIX30_ONE
#define B3_FIXED30_SHIFT           FIX30_SHIFT

B3_FIXED_INLINE b3Fixed30 b3Fix30FromFix( fixed_t a ) { return fix30FromFix( a ); }
B3_FIXED_INLINE fixed_t b3FixFromFix30( b3Fixed30 a ) { return fixFromFix30( a ); }
B3_FIXED_INLINE double b3Fix30ToDouble( b3Fixed30 a ) { return fix30ToDouble( a ); }
B3_FIXED_INLINE b3Fixed30 b3Fix30FromDouble( double x ) { return fix30FromDouble( x ); }

// ===================================================================================
// VECTORS, ROTATION, AND THE AGGREGATES BUILT ON THEM
// ===================================================================================
//
// Types, constants and INLINE forwarders only. Two families are deliberately absent:
//   - the exported (B3_API) symbols, declared in box3d/math_functions.h, because
//     base.h includes fixed.h before defining B3_API;
//   - the WORLD-POSITION family, which changes shape with BOX3D_LUDICROUS_MODE.
//     `fixed` exports narrow and wide under distinct names (fixPos vs fixPosWide);
//     box3d needs one name whose meaning follows its own flag, so it keeps its own.
#include "fixed/fixed_vec.h"
#include "fixed/fixed_math.h"
// b3Pos, b3WorldTransform and b3AABB take their WIDE form from here under
// BOX3D_LUDICROUS_MODE. Included unconditionally: the library exports both widths on
// every build and the header is cheap, so there is no #if guarding an #include.
#include "fixed/fixed_wide.h"

// ---- types ------------------------------------------------------------------------
typedef fixCosSin                  b3CosSin;
typedef fixInt128                  b3Int128;
typedef fixMatrix3                 b3Matrix3;
typedef fixPlane                   b3Plane;
typedef fixQuat                    b3Quat;
typedef fixTransform               b3Transform;
typedef fixUInt128                 b3UInt128;
typedef fixVec2                    b3Vec2;
typedef fixVec3                    b3Vec3;

// ---- constants --------------------------------------------------------------------
#define b3Mat3_identity                fixMat3_identity
#define b3Mat3_zero                    fixMat3_zero
#define b3Quat_identity                fixQuat_identity
#define b3Transform_identity           fixTransform_identity
#define b3Vec3_axisX                   fixVec3_axisX
#define b3Vec3_axisY                   fixVec3_axisY
#define b3Vec3_axisZ                   fixVec3_axisZ
#define b3Vec3_one                     fixVec3_one
#define b3Vec3_zero                    fixVec3_zero

// ---- value macros -----------------------------------------------------------------
#define B3_MIN_SCALE                   FIX_MIN_SCALE
#define B3_PI                          FIX_PI

// ---- inline forwarders ------------------------------------------------------------
B3_INLINE fixVec3 b3Abs( fixVec3 a ) { return fixVecAbs( a ); }
B3_INLINE fixMatrix3 b3AbsMatrix3( fixMatrix3 m ) { return fixAbsMatrix3( m ); }
B3_INLINE fixVec3 b3Add( fixVec3 a, fixVec3 b ) { return fixVecAdd( a, b ); }
B3_INLINE fixMatrix3 b3AddMM( fixMatrix3 a, fixMatrix3 b ) { return fixAddMM( a, b ); }
B3_INLINE fixVec3 b3Blend2( fixed_t s, fixVec3 a, fixed_t t, fixVec3 b ) { return fixBlend2( s, a, t, b ); }
B3_INLINE fixVec3 b3Clamp( fixVec3 a, fixVec3 lower, fixVec3 upper ) { return fixVecClamp( a, lower, upper ); }
B3_INLINE float b3ClampFloat( float a, float lower, float upper ) { return fixClampFloat( a, lower, upper ); }
B3_INLINE int b3ClampInt( int a, int lower, int upper ) { return fixClampInt( a, lower, upper ); }
B3_INLINE fixInt128 b3Cofactor128( fixed_t a, fixed_t b, fixed_t c, fixed_t d ) { return fixCofactor128( a, b, c, d ); }
B3_INLINE fixQuat b3Conjugate( fixQuat q ) { return fixConjugate( q ); }
B3_INLINE fixed_t b3Cos( fixed_t radians ) { return fixCos( radians ); }
B3_INLINE fixVec3 b3Cross( fixVec3 a, fixVec3 b ) { return fixCross( a, b ); }
B3_INLINE fixed_t b3Det( fixMatrix3 m ) { return fixDet( m ); }
B3_INLINE fixed_t b3Distance( fixVec3 a, fixVec3 b ) { return fixDistance( a, b ); }
B3_INLINE fixed_t b3DistanceSquared( fixVec3 a, fixVec3 b ) { return fixDistanceSquared( a, b ); }
B3_INLINE fixed_t b3Dot( fixVec3 a, fixVec3 b ) { return fixDot( a, b ); }
B3_INLINE fixed_t b3DotQuat( fixQuat a, fixQuat b ) { return fixDotQuat( a, b ); }
B3_INLINE fixInt128 b3DotRaw( fixVec3 a, fixVec3 b ) { return fixDotRaw( a, b ); }
B3_INLINE fixed_t b3FixFromDotRaw( fixInt128 raw ) { return fixFromDotRaw( raw ); }
B3_INLINE fixed_t b3FixLerp( fixed_t a, fixed_t b, fixed_t alpha ) { return fixLerp( a, b, alpha ); }
B3_INLINE fixVec3 b3GetAxisAngle( fixed_t* radians, fixQuat q ) { return fixGetAxisAngle( radians, q ); }
B3_INLINE fixVec3 b3GetLengthAndNormalize( fixed_t* length, fixVec3 a ) { return fixGetLengthAndNormalize( length, a ); }
B3_INLINE fixed_t b3GetQuatAngle( fixQuat q ) { return fixGetQuatAngle( q ); }
B3_INLINE fixed_t b3GetSwingAngle( fixQuat q ) { return fixGetSwingAngle( q ); }
B3_INLINE fixed_t b3GetTwistAngle( fixQuat q ) { return fixGetTwistAngle( q ); }
B3_INLINE fixQuat b3InvMulQuat( fixQuat q1, fixQuat q2 ) { return fixInvMulQuat( q1, q2 ); }
B3_INLINE fixTransform b3InvMulTransforms( fixTransform a, fixTransform b ) { return fixInvMulTransforms( a, b ); }
B3_INLINE fixVec3 b3InvRotateVector( fixQuat q, fixVec3 v ) { return fixInvRotateVector( q, v ); }
B3_INLINE fixVec3 b3InvTransformPoint( fixTransform t, fixVec3 v ) { return fixInvTransformPoint( t, v ); }
B3_INLINE fixMatrix3 b3InvertMatrix( fixMatrix3 m ) { return fixInvertMatrix( m ); }
B3_INLINE fixMatrix3 b3InvertT( fixMatrix3 m ) { return fixInvertT( m ); }
B3_INLINE fixTransform b3InvertTransform( fixTransform t ) { return fixInvertTransform( t ); }
B3_INLINE bool b3IsNormalized( fixVec3 a ) { return fixIsNormalized( a ); }
B3_INLINE bool b3IsNormalizedQuat( fixQuat q ) { return fixIsNormalizedQuat( q ); }
B3_INLINE fixed_t b3Length( fixVec3 v ) { return fixLength( v ); }
B3_INLINE fixed_t b3LengthSquared( fixVec3 a ) { return fixLengthSquared( a ); }
B3_INLINE fixVec3 b3Lerp( fixVec3 a, fixVec3 b, fixed_t alpha ) { return fixVecLerp( a, b, alpha ); }
B3_INLINE fixMatrix3 b3MakeMatrixFromQuat( fixQuat q ) { return fixMakeMatrixFromQuat( q ); }
B3_INLINE fixQuat b3MakeQuatFromAxisAngle( fixVec3 axis, fixed_t radians ) { return fixMakeQuatFromAxisAngle( axis, radians ); }
B3_INLINE fixVec3 b3Max( fixVec3 a, fixVec3 b ) { return fixVecMax( a, b ); }
B3_INLINE float b3MaxFloat( float a, float b ) { return fixMaxFloat( a, b ); }
B3_INLINE int b3MaxInt( int a, int b ) { return fixMaxInt( a, b ); }
B3_INLINE fixVec3 b3Min( fixVec3 a, fixVec3 b ) { return fixVecMin( a, b ); }
B3_INLINE float b3MinFloat( float a, float b ) { return fixMinFloat( a, b ); }
B3_INLINE int b3MinInt( int a, int b ) { return fixMinInt( a, b ); }
B3_INLINE fixVec3 b3Mul( fixVec3 a, fixVec3 b ) { return fixVecMul( a, b ); }
B3_INLINE fixVec3 b3MulAdd( fixVec3 a, fixed_t s, fixVec3 b ) { return fixMulAdd( a, s, b ); }
B3_INLINE fixMatrix3 b3MulMM( fixMatrix3 a, fixMatrix3 b ) { return fixMulMM( a, b ); }
B3_INLINE fixVec3 b3MulMV( fixMatrix3 m, fixVec3 a ) { return fixMulMV( m, a ); }
B3_INLINE fixQuat b3MulQuat( fixQuat q1, fixQuat q2 ) { return fixMulQuat( q1, q2 ); }
B3_INLINE fixMatrix3 b3MulSM( fixed_t s, fixMatrix3 a ) { return fixMulSM( s, a ); }
B3_INLINE fixVec3 b3MulSV( fixed_t s, fixVec3 a ) { return fixMulSV( s, a ); }
B3_INLINE fixVec3 b3MulSub( fixVec3 a, fixed_t s, fixVec3 b ) { return fixMulSub( a, s, b ); }
B3_INLINE fixTransform b3MulTransforms( fixTransform a, fixTransform b ) { return fixMulTransforms( a, b ); }
B3_INLINE fixQuat b3NLerp( fixQuat q1, fixQuat q2, fixed_t alpha ) { return fixNLerp( q1, q2, alpha ); }
B3_INLINE fixVec3 b3Neg( fixVec3 a ) { return fixVecNeg( a ); }
B3_INLINE fixMatrix3 b3NegateMat3( fixMatrix3 a ) { return fixNegateMat3( a ); }
B3_INLINE fixQuat b3NegateQuat( fixQuat q ) { return fixNegateQuat( q ); }
B3_INLINE fixVec3 b3Normalize( fixVec3 a ) { return fixNormalize( a ); }
B3_INLINE fixQuat b3NormalizeQuat( fixQuat q ) { return fixNormalizeQuat( q ); }
B3_INLINE fixVec3 b3Perp( fixVec3 a ) { return fixPerp( a ); }
B3_INLINE fixVec3 b3RotateVector( fixQuat q, fixVec3 v ) { return fixRotateVector( q, v ); }
B3_INLINE fixed_t b3RoundDownFloat( fixed_t x ) { return fixRoundDownFloat( x ); }
B3_INLINE fixed_t b3RoundUpFloat( fixed_t x ) { return fixRoundUpFloat( x ); }
B3_INLINE fixVec3 b3SafeScale( fixVec3 a ) { return fixSafeScale( a ); }
B3_INLINE fixVec3 b3Sign( fixVec3 a ) { return fixSign( a ); }
B3_INLINE fixed_t b3Sin( fixed_t radians ) { return fixSin( radians ); }
B3_INLINE fixVec3 b3Solve3( fixMatrix3 m, fixVec3 a ) { return fixSolve3( m, a ); }
B3_INLINE fixVec3 b3Sub( fixVec3 a, fixVec3 b ) { return fixVecSub( a, b ); }
B3_INLINE fixMatrix3 b3SubMM( fixMatrix3 a, fixMatrix3 b ) { return fixSubMM( a, b ); }
B3_INLINE fixVec3 b3TransformPoint( fixTransform t, fixVec3 v ) { return fixTransformPoint( t, v ); }
B3_INLINE fixMatrix3 b3Transpose( fixMatrix3 m ) { return fixTranspose( m ); }
B3_INLINE fixed_t b3UnwindAngle( fixed_t radians ) { return fixUnwindAngle( radians ); }

// ===================================================================================
// WORLD POSITIONS AND BOUNDING VOLUMES
// ===================================================================================
//
// These two families cross together because they are coupled: the wide b3AABB is built
// from b3Pos. Moving one while the other stayed produced "assigning to fixPosWide from
// incompatible type b3Pos" -- and only in the ludicrous configuration; the narrow build
// compiled fine either way.
//
// ONE BEHAVIOUR CHANGE, and it is the only one in the whole extraction: the library's
// wide extents differences in 128-bit before narrowing, where box3d narrowed first and
// so reported ZERO extents for any box past Q48.16 range -- at exactly the distances
// ludicrous mode exists to serve. For any box whose bounds both fit local range the two
// agree bit-for-bit, so this is invisible to every build that was already correct.
//
// SECOND THING WORTH KNOWING, inherited not introduced: the wide b3LerpPosition computes
// a + t*(b-a) rather than (1-t)*a + t*b, because the latter multiplies an absolute
// 128-bit coordinate and overflows. That is one rounding instead of two and it is
// deliberately NOT bit-identical to the narrow build -- the wide build carries its own
// goldens for it. Do not "fix" the two widths to agree.
#if defined( BOX3D_LUDICROUS_MODE )
typedef fixPosWide                 b3Pos;
typedef fixWorldTransformWide      b3WorldTransform;
typedef fixAABBWide                b3AABB;

B3_INLINE b3Pos   b3ToPos( b3Vec3 v )                                { return fixPosWideFromVec3( v ); }
B3_INLINE b3Vec3  b3ToVec3( b3Pos p )                                { return fixPosWideToVec3( p ); }
B3_INLINE b3Vec3  b3SubPos( b3Pos a, b3Pos b )                       { return fixPosWideSub( a, b ); }
B3_INLINE b3Pos   b3OffsetPos( b3Pos p, b3Vec3 d )                   { return fixPosWideOffset( p, d ); }
B3_INLINE b3Pos   b3LerpPosition( b3Pos a, b3Pos b, b3Fixed t )      { return fixLerpPositionWide( a, b, t ); }
B3_INLINE b3Pos   b3TransformWorldPoint( b3WorldTransform t, b3Vec3 p )
                                                                     { return fixTransformWorldPointWide( t, p ); }
B3_INLINE b3Vec3  b3InvTransformWorldPoint( b3WorldTransform t, b3Pos p )
                                                                     { return fixInvTransformWorldPointWide( t, p ); }
B3_INLINE b3WorldTransform b3MakeWorldTransform( b3Transform t )     { return fixMakeWorldTransformWide( t ); }
B3_INLINE b3WorldTransform b3MulWorldTransforms( b3WorldTransform A, b3Transform B )
                                                                     { return fixMulWorldTransformsWide( A, B ); }
B3_INLINE b3Transform b3InvMulWorldTransforms( b3WorldTransform A, b3WorldTransform B )
                                                                     { return fixInvMulWorldTransformsWide( A, B ); }
B3_INLINE b3Transform b3ToRelativeTransform( b3WorldTransform t, b3Pos base )
                                                                     { return fixToRelativeTransformWide( t, base ); }

B3_INLINE b3AABB  b3MakeAABB( const b3Vec3* points, int count, b3Fixed radius )
{
	// box3d builds boxes from LOCAL vertices; assembled narrow, widened once. This is the
	// gap fixMakeAABBWideAt was added upstream to close -- fixMakeAABBWide takes wide points.
	fixPosWide origin = { 0, 0, 0 };
	return fixMakeAABBWideAt( points, count, radius, origin );
}
B3_INLINE bool    b3AABB_Contains( b3AABB a, b3AABB b )              { return fixAABBWide_Contains( a, b ); }
B3_INLINE b3Fixed b3AABB_Area( b3AABB a )                            { return fixAABBWide_Area( a ); }
B3_INLINE b3Vec3  b3AABB_Center( b3AABB a )                          { return fixAABBWide_Center( a ); }
B3_INLINE b3Vec3  b3AABB_Extents( b3AABB a )                         { return fixAABBWide_Extents( a ); }
B3_INLINE b3AABB  b3AABB_Union( b3AABB a, b3AABB b )                 { return fixAABBWide_Union( a, b ); }
B3_INLINE b3AABB  b3AABB_Inflate( b3AABB a, b3Fixed e )              { return fixAABBWide_Inflate( a, e ); }
B3_INLINE bool    b3AABB_Overlaps( b3AABB a, b3AABB b )              { return fixAABBWide_Overlaps( a, b ); }
B3_INLINE b3AABB  b3AABB_Transform( b3Transform t, b3AABB a )        { return fixAABBWide_Transform( t, a ); }
B3_INLINE b3Vec3  b3ClosestPointToAABB( b3Vec3 point, b3AABB a )     { return fixClosestPointToAABBWide( point, a ); }

// box3d's local wide min/max, which were a copy of these all along.
B3_INLINE b3Int128 b3W_min128( b3Int128 a, b3Int128 b )              { return fixWideMin( a, b ); }
B3_INLINE b3Int128 b3W_max128( b3Int128 a, b3Int128 b )              { return fixWideMax( a, b ); }
#else
typedef fixVec3                    b3Pos;
typedef fixTransform               b3WorldTransform;
typedef fixAABB                    b3AABB;

B3_INLINE b3Pos   b3ToPos( b3Vec3 v )                                { return v; }
B3_INLINE b3Vec3  b3ToVec3( b3Pos p )                                { return p; }
B3_INLINE b3Vec3  b3SubPos( b3Pos a, b3Pos b )                       { return fixVecSub( a, b ); }
B3_INLINE b3Pos   b3OffsetPos( b3Pos p, b3Vec3 d )                   { return fixVecAdd( p, d ); }
B3_INLINE b3Pos   b3LerpPosition( b3Pos a, b3Pos b, b3Fixed t )      { return fixLerpPosition( a, b, t ); }
B3_INLINE b3Pos   b3TransformWorldPoint( b3WorldTransform t, b3Vec3 p )
                                                                     { return fixTransformWorldPoint( t, p ); }
B3_INLINE b3Vec3  b3InvTransformWorldPoint( b3WorldTransform t, b3Pos p )
                                                                     { return fixInvTransformWorldPoint( t, p ); }
B3_INLINE b3WorldTransform b3MakeWorldTransform( b3Transform t )     { return fixMakeWorldTransform( t ); }
B3_INLINE b3WorldTransform b3MulWorldTransforms( b3WorldTransform A, b3Transform B )
                                                                     { return fixMulWorldTransforms( A, B ); }
B3_INLINE b3Transform b3InvMulWorldTransforms( b3WorldTransform A, b3WorldTransform B )
                                                                     { return fixInvMulWorldTransforms( A, B ); }
B3_INLINE b3Transform b3ToRelativeTransform( b3WorldTransform t, b3Pos base )
                                                                     { return fixToRelativeTransform( t, base ); }

B3_INLINE b3AABB  b3MakeAABB( const b3Vec3* points, int count, b3Fixed radius )
                                                                     { return fixMakeAABB( points, count, radius ); }
B3_INLINE bool    b3AABB_Contains( b3AABB a, b3AABB b )              { return fixAABB_Contains( a, b ); }
B3_INLINE b3Fixed b3AABB_Area( b3AABB a )                            { return fixAABB_Area( a ); }
B3_INLINE b3Vec3  b3AABB_Center( b3AABB a )                          { return fixAABB_Center( a ); }
B3_INLINE b3Vec3  b3AABB_Extents( b3AABB a )                         { return fixAABB_Extents( a ); }
B3_INLINE b3AABB  b3AABB_Union( b3AABB a, b3AABB b )                 { return fixAABB_Union( a, b ); }
B3_INLINE b3AABB  b3AABB_Inflate( b3AABB a, b3Fixed e )              { return fixAABB_Inflate( a, e ); }
B3_INLINE bool    b3AABB_Overlaps( b3AABB a, b3AABB b )              { return fixAABB_Overlaps( a, b ); }
B3_INLINE b3AABB  b3AABB_Transform( b3Transform t, b3AABB a )        { return fixAABB_Transform( t, a ); }
B3_INLINE b3Vec3  b3ClosestPointToAABB( b3Vec3 point, b3AABB a )     { return fixClosestPointToAABB( point, a ); }
#endif

