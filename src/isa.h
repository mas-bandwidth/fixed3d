// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Runtime ISA dispatch. One build compiles every variant the target can hold;
// detection runs once at init; each world takes an immutable kernel table at
// b3CreateWorld. Pattern: simdjson's implementation table (one source compiled
// per target, selected by name, forced by an environment variable) and Google
// Highway's foreach_target (the same kernel text re-included once per target
// under per-target flags). See CLAUDE.md "Runtime ISA dispatch".
//
// Every variant is bit-identical to the scalar variant on every input: the wide
// kernels use exact product decompositions, the same round-half-up and the same
// wrapping as the scalar helpers, so a mixed fleet stays in lockstep.

#pragma once

#include "core.h"
#include "manifold.h"
#include "solver.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum b3IsaVariant
{
	b3_isaScalar = 0,
	b3_isaAvx2,
	b3_isaAvx512,
	b3_isaAvx512Ifma,
	b3_isaNeon,
	b3_isaSme2,
	b3_isaVariantCount
} b3IsaVariant;

// Facts about the host. Recorded even where no variant uses them yet, so the
// spikes that build later variants read them from one place.
typedef struct b3IsaFacts
{
	// x86-64. The AVX facts already include the OS state check (XCR0), so a
	// true here means the instructions are usable, not only present.
	bool osAvx;	   // OSXSAVE and XCR0 bits 1, 2 (SSE and AVX state)
	bool osAvx512; // osAvx and XCR0 bits 5, 6, 7 (opmask, ZMM_Hi256, Hi16_ZMM)
	bool avx2;
	bool fma;
	bool avx512f;
	bool avx512dq;
	bool avx512bw;
	bool avx512vl;
	bool avx512cd;
	bool avx512ifma;
	bool avx512vbmi;
	bool avx512vbmi2;
	bool gfni;

	// aarch64
	bool asimd;
	bool sve2;
	bool sme;
	bool sme2;
	int smeMaxSvlBytes; // 0 when unknown or no SME
} b3IsaFacts;

// Pure parsers: register and sysctl words in, facts out. Unit tested on faked
// words (test/test_isa_dispatch.c). leaf1Ecx is CPUID.(EAX=1):ECX, leaf7Ebx and
// leaf7Ecx are CPUID.(EAX=7,ECX=0):EBX/ECX (pass 0 when the max leaf is below 7),
// xcr0 is XGETBV(0) (pass 0 when OSXSAVE is clear: XGETBV would fault).
b3IsaFacts b3IsaParseX86( uint32_t leaf1Ecx, uint32_t leaf7Ebx, uint32_t leaf7Ecx, uint64_t xcr0 );

// Linux aarch64: getauxval(AT_HWCAP) and getauxval(AT_HWCAP2)
b3IsaFacts b3IsaParseLinuxArm( uint64_t hwcap, uint64_t hwcap2 );

// macOS arm64: sysctlbyname hw.optional.arm.FEAT_SME2 / FEAT_SME / FEAT_SVE2
// (0 or 1, 0 when the name is missing) and hw.optional.arm.sme_max_svl_b
b3IsaFacts b3IsaParseMacArm( int featSme2, int featSme, int featSve2, int smeMaxSvlBytes );

// Reads the real host words and runs the parser for this target
b3IsaFacts b3IsaDetectHost( void );

// Name of a variant ("scalar", "avx2", "avx512", "avx512ifma", "neon", "sme2")
const char* b3IsaVariantName( b3IsaVariant variant );

// Variant for a name, or -1 for an unknown name
int b3IsaVariantFromName( const char* name );

// True when this build compiled the variant
bool b3IsaIsCompiled( b3IsaVariant variant );

// The first fact the variant needs that is false on this host, or NULL when the
// host supports it. Pure: depends on the facts only.
const char* b3IsaMissingFact( b3IsaVariant variant, const b3IsaFacts* facts );

// The variant detection picks with no override: the widest compiled variant
// whose facts hold. Pure.
b3IsaVariant b3IsaBestVariant( const b3IsaFacts* facts );

// Checks a forced variant name against the build and the facts. Returns the
// variant, or -1 and writes the named refusal into reason. Pure.
int b3IsaCheckForce( const char* name, const b3IsaFacts* facts, char* reason, int reasonCapacity );

// Kernel table. Dispatch is once per kernel call: a solver stage over a color
// block, or one support / SAT edge scan; never per multiply.
typedef struct b3IsaKernels
{
	b3IsaVariant variant;
	const char* name;

	// The colored mesh path takes the four-lane mesh kernels only where the
	// lanes have hardware (see contact_solver.h); both mesh paths are
	// bit-identical per lane.
	bool meshWide;

	int ( *wideContactConstraintByteCount )( void );
	int ( *meshWideContactConstraintByteCount )( void );
	int ( *meshWideManifoldConstraintByteCount )( void );

	void ( *prepareContactsConvex )( b3SolverBlock block, b3StepContext* context );
	void ( *warmStartContactsConvex )( b3SolverBlock block, b3StepContext* context );
	void ( *solveContactsConvex )( b3SolverBlock block, b3StepContext* context, bool useBias );
	void ( *applyRestitutionConvex )( b3SolverBlock block, b3StepContext* context );
	void ( *storeImpulsesConvex )( b3SolverBlock block, b3StepContext* context, int workerIndex );

	void ( *prepareContactsMeshWide )( b3SolverBlock block, b3StepContext* context );
	void ( *warmStartContactsMeshWide )( b3SolverBlock block, b3StepContext* context );
	void ( *solveContactsMeshWide )( b3SolverBlock block, b3StepContext* context, bool useBias );
	void ( *applyRestitutionMeshWide )( b3SolverBlock block, b3StepContext* context );
	void ( *storeImpulsesMeshWide )( b3SolverBlock block, b3StepContext* context, int workerIndex );

	// Body integration stages over a block of awake bodies
	void ( *integrateVelocities )( b3SolverBlock block, b3StepContext* context );
	void ( *integratePositions )( b3SolverBlock block, b3StepContext* context );

	int ( *findHullSupportVertex )( const b3HullData* hull, b3Vec3 direction );
	int ( *findHullSupportFace )( const b3HullData* hull, b3Vec3 direction );
	b3SeparatingAxis ( *queryEdgeDirections )( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA );
} b3IsaKernels;

// Runs detection and reads FIXED3D_ISA, once. Called by b3CreateWorld (world
// creation is single-threaded by the API contract), and by b3ForceIsa and
// b3GetIsaName. A FIXED3D_ISA that names a variant not compiled or not
// supported aborts with a message naming the missing fact.
void b3IsaInit( void );

// The table for a new world: called by b3CreateWorld, paired with b3IsaReleaseWorld
// in b3DestroyWorld. The table is immutable for the world's life.
const b3IsaKernels* b3IsaAcquireWorld( void );
void b3IsaReleaseWorld( void );

// The process table the narrow phase reads (support and SAT scans run outside
// any world too). It equals every live world's table: b3ForceIsa refuses while
// a world lives. Before init it is the scalar table.
const b3IsaKernels* b3IsaActive( void );

// The host facts detection recorded (valid after b3IsaInit)
const b3IsaFacts* b3IsaHostFacts( void );

// Per-variant kernels. The suffix is the variant name; the scalar ones live in
// the ordinary translation units, the others in src/isa_<variant>.c.
#define B3_ISA_DECLARE_KERNELS( suffix )                                                                                         \
	int b3GetWideContactConstraintByteCount_##suffix( void );                                                                    \
	int b3GetMeshWideContactConstraintByteCount_##suffix( void );                                                                \
	int b3GetMeshWideManifoldConstraintByteCount_##suffix( void );                                                               \
	void b3PrepareContacts_Convex_##suffix( b3SolverBlock block, b3StepContext* context );                                       \
	void b3WarmStartContacts_Convex_##suffix( b3SolverBlock block, b3StepContext* context );                                     \
	void b3SolveContacts_Convex_##suffix( b3SolverBlock block, b3StepContext* context, bool useBias );                           \
	void b3ApplyRestitution_Convex_##suffix( b3SolverBlock block, b3StepContext* context );                                      \
	void b3StoreImpulses_Convex_##suffix( b3SolverBlock block, b3StepContext* context, int workerIndex );                        \
	void b3PrepareContacts_MeshWide_##suffix( b3SolverBlock block, b3StepContext* context );                                     \
	void b3WarmStartContacts_MeshWide_##suffix( b3SolverBlock block, b3StepContext* context );                                   \
	void b3SolveContacts_MeshWide_##suffix( b3SolverBlock block, b3StepContext* context, bool useBias );                         \
	void b3ApplyRestitution_MeshWide_##suffix( b3SolverBlock block, b3StepContext* context );                                    \
	void b3StoreImpulses_MeshWide_##suffix( b3SolverBlock block, b3StepContext* context, int workerIndex );                      \
	void b3IntegrateVelocities_##suffix( b3SolverBlock block, b3StepContext* context );                                          \
	void b3IntegratePositions_##suffix( b3SolverBlock block, b3StepContext* context );                                           \
	int b3FindHullSupportVertex_##suffix( const b3HullData* hull, b3Vec3 direction );                                            \
	int b3FindHullSupportFace_##suffix( const b3HullData* hull, b3Vec3 direction );                                              \
	b3SeparatingAxis b3QueryEdgeDirections_##suffix( const b3HullData* hullA, const b3HullData* hullB, b3Transform transformBtoA )

B3_ISA_DECLARE_KERNELS( scalar );

#if defined( B3_ISA_HAVE_AVX512 )
B3_ISA_DECLARE_KERNELS( avx512 );
#endif

// Suffix for the kernels of the translation unit being compiled. The scalar
// translation units leave it unset; src/isa_<variant>.c sets it before it
// re-includes the kernel sources.
#if !defined( B3_ISA_FN )
#define B3_ISA_FN( name ) name##_scalar
#endif
