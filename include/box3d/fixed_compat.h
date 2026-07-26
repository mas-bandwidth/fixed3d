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
// GENERATED. Every name below was read out of the previous box3d/fixed.h and mapped
// through the rename that moved the library off the b3 prefix, so the set is exactly
// what box3d defined before -- no more, no less.
//
// Forwarders rather than #defines: a macro would stop b3FixMul being a declared
// identifier, so debuggers would show fixMul and a consumer could no longer name
// anything b3FixMul. All of these were header-inline in box3d and stay header-inline,
// so linkage is unchanged; after inlining the wrapper costs nothing.
#pragma once

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
#define B3_ASSERT                      FIX_ASSERT
#define B3_FORCE_INLINE                FIX_FORCE_INLINE
#define B3_INLINE                      FIX_INLINE
#define B3_LITERAL                     FIX_LITERAL
#define B3_MIN_SCALE                   FIX_MIN_SCALE
#define B3_PI                          FIX_PI
#define B3_VALIDATE                    FIX_VALIDATE

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

