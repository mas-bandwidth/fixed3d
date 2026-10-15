// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Runtime ISA dispatch: detection, the per-variant kernel tables, and the
// FIXED3D_ISA / b3ForceIsa override. See src/isa.h and CLAUDE.md "Runtime ISA
// dispatch". This file is compiled with the project's ordinary flags: it runs
// before any variant is chosen, so it must hold no instruction the baseline
// target lacks (cpuid and xgetbv are baseline x86-64; xgetbv runs only behind
// the OSXSAVE bit).

#include "isa.h"

#include "core.h"
#include "platform.h"

#include "box3d/base.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( __linux__ ) && ( defined( __aarch64__ ) || defined( _M_ARM64 ) )
#include <sys/auxv.h>
#endif

#if defined( __APPLE__ ) && ( defined( __aarch64__ ) || defined( _M_ARM64 ) )
#include <sys/sysctl.h>
#include <sys/types.h>
#endif

#if defined( _MSC_VER ) && !defined( __clang__ ) && ( defined( _M_X64 ) || defined( _M_IX86 ) )
#include <intrin.h>
#endif

// ---- pure parsers ----------------------------------------------------------

static bool b3Bit32( uint32_t word, int bit )
{
	return ( word >> bit ) & 1u;
}

static bool b3Bit64( uint64_t word, int bit )
{
	return ( word >> bit ) & 1u;
}

b3IsaFacts b3IsaParseX86( uint32_t leaf1Ecx, uint32_t leaf7Ebx, uint32_t leaf7Ecx, uint64_t xcr0 )
{
	b3IsaFacts f = { 0 };

	// CPUID.1:ECX bit 27 OSXSAVE (the OS enabled XSAVE, so XGETBV is legal and
	// XCR0 is meaningful), bit 28 AVX, bit 12 FMA. XCR0 bit 1 SSE state, bit 2
	// AVX (YMM upper) state: without both the OS does not save YMM registers
	// across a context switch and AVX code must not run.
	bool osxsave = b3Bit32( leaf1Ecx, 27 );
	bool avx = b3Bit32( leaf1Ecx, 28 );
	f.osAvx = osxsave && avx && ( xcr0 & 0x6 ) == 0x6;

	// XCR0 bits 5, 6, 7: opmask, ZMM_Hi256, Hi16_ZMM. An OS that does not save
	// the ZMM state yields no AVX-512 even on a CPU that has it (and the 256-bit
	// AVX512VL forms still use the opmask registers, so bit 5 is needed too).
	f.osAvx512 = f.osAvx && ( xcr0 & 0xE0 ) == 0xE0;

	f.fma = f.osAvx && b3Bit32( leaf1Ecx, 12 );

	// CPUID.(7,0):EBX bit 5 AVX2, 16 AVX512F, 17 AVX512DQ, 21 AVX512_IFMA,
	// 28 AVX512CD, 30 AVX512BW, 31 AVX512VL. ECX bit 1 AVX512_VBMI,
	// 6 AVX512_VBMI2, 8 GFNI.
	f.avx2 = f.osAvx && b3Bit32( leaf7Ebx, 5 );
	f.avx512f = f.osAvx512 && b3Bit32( leaf7Ebx, 16 );
	f.avx512dq = f.avx512f && b3Bit32( leaf7Ebx, 17 );
	f.avx512ifma = f.avx512f && b3Bit32( leaf7Ebx, 21 );
	f.avx512cd = f.avx512f && b3Bit32( leaf7Ebx, 28 );
	f.avx512bw = f.avx512f && b3Bit32( leaf7Ebx, 30 );
	f.avx512vl = f.avx512f && b3Bit32( leaf7Ebx, 31 );
	f.avx512vbmi = f.avx512f && b3Bit32( leaf7Ecx, 1 );
	f.avx512vbmi2 = f.avx512f && b3Bit32( leaf7Ecx, 6 );
	// GFNI has SSE forms that need no AVX state; the raw bit is the fact
	f.gfni = b3Bit32( leaf7Ecx, 8 );
	return f;
}

b3IsaFacts b3IsaParseLinuxArm( uint64_t hwcap, uint64_t hwcap2 )
{
	// arch/arm64/include/uapi/asm/hwcap.h: HWCAP_ASIMD (1 << 1),
	// HWCAP2_SVE2 (1 << 1), HWCAP2_SME (1 << 23), HWCAP2_SME2 (1UL << 37)
	b3IsaFacts f = { 0 };
	f.asimd = b3Bit64( hwcap, 1 );
	f.sve2 = b3Bit64( hwcap2, 1 );
	f.sme = b3Bit64( hwcap2, 23 );
	f.sme2 = f.sme && b3Bit64( hwcap2, 37 );
	return f;
}

b3IsaFacts b3IsaParseMacArm( int featSme2, int featSme, int featSve2, int smeMaxSvlBytes )
{
	// Every arm64 macOS host has ASIMD
	b3IsaFacts f = { 0 };
	f.asimd = true;
	f.sve2 = featSve2 != 0;
	f.sme = featSme != 0;
	f.sme2 = f.sme && featSme2 != 0;
	f.smeMaxSvlBytes = f.sme && smeMaxSvlBytes > 0 ? smeMaxSvlBytes : 0;
	return f;
}

// ---- reading the real host -------------------------------------------------

#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( _M_IX86 )

static void b3CpuId( uint32_t leaf, uint32_t subleaf, uint32_t regs[4] )
{
#if defined( _MSC_VER ) && !defined( __clang__ )
	int r[4];
	__cpuidex( r, (int)leaf, (int)subleaf );
	for ( int i = 0; i < 4; ++i )
	{
		regs[i] = (uint32_t)r[i];
	}
#else
	uint32_t a, b, c, d;
	__asm__ __volatile__( "cpuid" : "=a"( a ), "=b"( b ), "=c"( c ), "=d"( d ) : "a"( leaf ), "c"( subleaf ) );
	regs[0] = a;
	regs[1] = b;
	regs[2] = c;
	regs[3] = d;
#endif
}

static uint64_t b3ReadXcr0( void )
{
#if defined( _MSC_VER ) && !defined( __clang__ )
	return (uint64_t)_xgetbv( 0 );
#else
	uint32_t eax, edx;
	// Volatile: XGETBV faults when OSXSAVE is clear, so it must stay behind the
	// caller's OSXSAVE check (the divq lesson in CLAUDE.md)
	__asm__ __volatile__( "xgetbv" : "=a"( eax ), "=d"( edx ) : "c"( 0 ) );
	return ( (uint64_t)edx << 32 ) | eax;
#endif
}

b3IsaFacts b3IsaDetectHost( void )
{
	uint32_t r[4];
	b3CpuId( 0, 0, r );
	uint32_t maxLeaf = r[0];

	b3CpuId( 1, 0, r );
	uint32_t leaf1Ecx = r[2];

	uint32_t leaf7Ebx = 0, leaf7Ecx = 0;
	if ( maxLeaf >= 7 )
	{
		b3CpuId( 7, 0, r );
		leaf7Ebx = r[1];
		leaf7Ecx = r[2];
	}

	uint64_t xcr0 = 0;
	if ( b3Bit32( leaf1Ecx, 27 ) )
	{
		xcr0 = b3ReadXcr0();
	}

	return b3IsaParseX86( leaf1Ecx, leaf7Ebx, leaf7Ecx, xcr0 );
}

#elif ( defined( __aarch64__ ) || defined( _M_ARM64 ) ) && defined( __linux__ )

b3IsaFacts b3IsaDetectHost( void )
{
	return b3IsaParseLinuxArm( (uint64_t)getauxval( AT_HWCAP ), (uint64_t)getauxval( AT_HWCAP2 ) );
}

#elif ( defined( __aarch64__ ) || defined( _M_ARM64 ) ) && defined( __APPLE__ )

static int b3SysctlInt( const char* name )
{
	int value = 0;
	size_t size = sizeof( value );
	if ( sysctlbyname( name, &value, &size, NULL, 0 ) != 0 )
	{
		return 0;
	}
	return value;
}

b3IsaFacts b3IsaDetectHost( void )
{
	return b3IsaParseMacArm( b3SysctlInt( "hw.optional.arm.FEAT_SME2" ), b3SysctlInt( "hw.optional.arm.FEAT_SME" ),
							 b3SysctlInt( "hw.optional.arm.FEAT_SVE2" ), b3SysctlInt( "hw.optional.arm.sme_max_svl_b" ) );
}

#elif defined( __aarch64__ ) || defined( _M_ARM64 )

b3IsaFacts b3IsaDetectHost( void )
{
	// Other aarch64 hosts (Windows on ARM): ASIMD is architectural
	b3IsaFacts f = { 0 };
	f.asimd = true;
	return f;
}

#else

b3IsaFacts b3IsaDetectHost( void )
{
	b3IsaFacts f = { 0 };
	return f;
}

#endif

// ---- variants --------------------------------------------------------------

static const char* const b3_isaNames[b3_isaVariantCount] = {
	"scalar", "avx2", "avx512", "avx512ifma", "neon", "sme2",
};

const char* b3IsaVariantName( b3IsaVariant variant )
{
	if ( variant < 0 || variant >= b3_isaVariantCount )
	{
		return "unknown";
	}
	return b3_isaNames[variant];
}

int b3IsaVariantFromName( const char* name )
{
	if ( name == NULL )
	{
		return -1;
	}
	for ( int i = 0; i < b3_isaVariantCount; ++i )
	{
		if ( strcmp( name, b3_isaNames[i] ) == 0 )
		{
			return i;
		}
	}
	return -1;
}

#define B3_ISA_TABLE( suffix, variantValue, meshWideValue )                                                                      \
	{                                                                                                                            \
		.variant = variantValue,                                                                                                 \
		.name = #suffix,                                                                                                         \
		.meshWide = meshWideValue,                                                                                               \
		.wideContactConstraintByteCount = b3GetWideContactConstraintByteCount_##suffix,                                          \
		.meshWideContactConstraintByteCount = b3GetMeshWideContactConstraintByteCount_##suffix,                                  \
		.meshWideManifoldConstraintByteCount = b3GetMeshWideManifoldConstraintByteCount_##suffix,                                \
		.prepareContactsConvex = b3PrepareContacts_Convex_##suffix,                                                              \
		.warmStartContactsConvex = b3WarmStartContacts_Convex_##suffix,                                                          \
		.solveContactsConvex = b3SolveContacts_Convex_##suffix,                                                                  \
		.applyRestitutionConvex = b3ApplyRestitution_Convex_##suffix,                                                            \
		.storeImpulsesConvex = b3StoreImpulses_Convex_##suffix,                                                                  \
		.prepareContactsMeshWide = b3PrepareContacts_MeshWide_##suffix,                                                          \
		.warmStartContactsMeshWide = b3WarmStartContacts_MeshWide_##suffix,                                                      \
		.solveContactsMeshWide = b3SolveContacts_MeshWide_##suffix,                                                              \
		.applyRestitutionMeshWide = b3ApplyRestitution_MeshWide_##suffix,                                                        \
		.storeImpulsesMeshWide = b3StoreImpulses_MeshWide_##suffix,                                                              \
		.findHullSupportVertex = b3FindHullSupportVertex_##suffix,                                                               \
		.findHullSupportFace = b3FindHullSupportFace_##suffix,                                                                   \
		.queryEdgeDirections = b3QueryEdgeDirections_##suffix,                                                                   \
	}

// The scalar variant keeps the scalar colored mesh path (no lane hardware)
static const b3IsaKernels b3_isaScalarKernels = B3_ISA_TABLE( scalar, b3_isaScalar, false );

#if defined( B3_ISA_HAVE_AVX512 )
static const b3IsaKernels b3_isaAvx512Kernels = B3_ISA_TABLE( avx512, b3_isaAvx512, true );
#endif

static const b3IsaKernels* const b3_isaTables[b3_isaVariantCount] = {
	[b3_isaScalar] = &b3_isaScalarKernels,
#if defined( B3_ISA_HAVE_AVX512 )
	[b3_isaAvx512] = &b3_isaAvx512Kernels,
#endif
};

bool b3IsaIsCompiled( b3IsaVariant variant )
{
	return 0 <= variant && variant < b3_isaVariantCount && b3_isaTables[variant] != NULL;
}

const char* b3IsaMissingFact( b3IsaVariant variant, const b3IsaFacts* facts )
{
	switch ( variant )
	{
		case b3_isaScalar:
			return NULL;
		case b3_isaAvx2:
			if ( facts->avx2 == false )
				return "avx2";
			return NULL;
		case b3_isaAvx512:
			if ( facts->avx512f == false )
				return "avx512f";
			if ( facts->avx512dq == false )
				return "avx512dq";
			if ( facts->avx512vl == false )
				return "avx512vl";
			return NULL;
		case b3_isaAvx512Ifma:
			if ( facts->avx512f == false )
				return "avx512f";
			if ( facts->avx512dq == false )
				return "avx512dq";
			if ( facts->avx512vl == false )
				return "avx512vl";
			if ( facts->avx512ifma == false )
				return "avx512ifma";
			return NULL;
		case b3_isaNeon:
			if ( facts->asimd == false )
				return "asimd";
			return NULL;
		case b3_isaSme2:
			if ( facts->sme2 == false )
				return "sme2";
			return NULL;
		default:
			return "unknown";
	}
}

b3IsaVariant b3IsaBestVariant( const b3IsaFacts* facts )
{
	// Widest first. Only compiled variants whose facts hold are candidates.
	static const b3IsaVariant order[] = { b3_isaSme2, b3_isaAvx512Ifma, b3_isaAvx512, b3_isaAvx2, b3_isaNeon };
	for ( int i = 0; i < (int)( sizeof( order ) / sizeof( order[0] ) ); ++i )
	{
		if ( b3IsaIsCompiled( order[i] ) && b3IsaMissingFact( order[i], facts ) == NULL )
		{
			return order[i];
		}
	}
	return b3_isaScalar;
}

int b3IsaCheckForce( const char* name, const b3IsaFacts* facts, char* reason, int reasonCapacity )
{
	int variant = b3IsaVariantFromName( name );
	if ( variant < 0 )
	{
		snprintf( reason, (size_t)reasonCapacity,
				  "unknown ISA variant '%s' (names: scalar, avx2, avx512, avx512ifma, neon, sme2)", name ? name : "(null)" );
		return -1;
	}

	if ( b3IsaIsCompiled( (b3IsaVariant)variant ) == false )
	{
		snprintf( reason, (size_t)reasonCapacity, "ISA variant %s is not compiled in this build", b3_isaNames[variant] );
		return -1;
	}

	const char* missing = b3IsaMissingFact( (b3IsaVariant)variant, facts );
	if ( missing != NULL )
	{
		snprintf( reason, (size_t)reasonCapacity, "ISA variant %s refused: host lacks %s", b3_isaNames[variant], missing );
		return -1;
	}

	if ( reasonCapacity > 0 )
	{
		reason[0] = 0;
	}
	return variant;
}

// ---- process state ---------------------------------------------------------
//
// No state machine (CLAUDE.md "Runtime ISA dispatch" says why no TLA+ model is
// owed): detection is a pure function of the host words, run once by
// b3IsaInit from b3CreateWorld, b3ForceIsa or b3GetIsaName, all of which the
// API contract makes single-threaded with respect to world creation. A world
// copies the table pointer at creation and never changes it; b3ForceIsa is
// refused while any world lives, so the process table the narrow phase reads
// equals every live world's table. The active variant is an atomic so a
// narrow-phase query outside any world reads a whole, valid index.

static bool b3_isaInitDone = false;
static b3IsaFacts b3_isaFacts;
static b3IsaVariant b3_isaDefault = b3_isaScalar; // detection, or FIXED3D_ISA
static int b3_isaLiveWorlds = 0;
static b3AtomicInt b3_isaActive = { b3_isaScalar };
static char b3_isaRefusal[160];

// Reads FIXED3D_ISA into buffer; false when unset or empty
static bool b3ReadIsaEnv( char* buffer, int capacity )
{
	buffer[0] = 0;
#if defined( _MSC_VER )
	// getenv is deprecated under the MSVC CRT
	char* value = NULL;
	size_t length = 0;
	if ( _dupenv_s( &value, &length, "FIXED3D_ISA" ) != 0 || value == NULL )
	{
		return false;
	}
	snprintf( buffer, (size_t)capacity, "%s", value );
	free( value );
#else
	const char* value = getenv( "FIXED3D_ISA" );
	if ( value == NULL )
	{
		return false;
	}
	snprintf( buffer, (size_t)capacity, "%s", value );
#endif
	return buffer[0] != 0;
}

void b3IsaInit( void )
{
	if ( b3_isaInitDone )
	{
		return;
	}

	b3_isaFacts = b3IsaDetectHost();
	b3IsaVariant chosen = b3IsaBestVariant( &b3_isaFacts );

	char env[64];
	if ( b3ReadIsaEnv( env, (int)sizeof( env ) ) )
	{
		char reason[160];
		int forced = b3IsaCheckForce( env, &b3_isaFacts, reason, (int)sizeof( reason ) );
		if ( forced < 0 )
		{
			// Never silently replaced: a forced variant that cannot run here
			// stops the process with the missing fact named
			fprintf( stderr, "FIXED3D_ISA=%s: %s\n", env, reason );
			b3Log( "FIXED3D_ISA=%s: %s", env, reason );
			abort();
		}
		chosen = (b3IsaVariant)forced;
	}

	b3_isaDefault = chosen;
	b3AtomicStoreInt( &b3_isaActive, (int)chosen );
	b3_isaInitDone = true;
}

const b3IsaKernels* b3IsaActive( void )
{
	return b3_isaTables[b3AtomicLoadInt( &b3_isaActive )];
}

const b3IsaFacts* b3IsaHostFacts( void )
{
	b3IsaInit();
	return &b3_isaFacts;
}

const b3IsaKernels* b3IsaAcquireWorld( void )
{
	b3IsaInit();
	b3_isaLiveWorlds += 1;
	return b3IsaActive();
}

void b3IsaReleaseWorld( void )
{
	B3_ASSERT( b3_isaLiveWorlds > 0 );
	b3_isaLiveWorlds -= 1;
}

bool b3ForceIsa( const char* name )
{
	b3IsaInit();
	b3_isaRefusal[0] = 0;

	if ( b3_isaLiveWorlds > 0 )
	{
		snprintf( b3_isaRefusal, sizeof( b3_isaRefusal ), "ISA variant %s refused: %d world(s) live, force before b3CreateWorld",
				  name ? name : "(default)", b3_isaLiveWorlds );
		return false;
	}

	if ( name == NULL )
	{
		b3AtomicStoreInt( &b3_isaActive, (int)b3_isaDefault );
		return true;
	}

	int variant = b3IsaCheckForce( name, &b3_isaFacts, b3_isaRefusal, (int)sizeof( b3_isaRefusal ) );
	if ( variant < 0 )
	{
		return false;
	}

	b3AtomicStoreInt( &b3_isaActive, variant );
	return true;
}

const char* b3GetIsaName( void )
{
	b3IsaInit();
	return b3IsaActive()->name;
}

const char* b3GetIsaRefusal( void )
{
	return b3_isaRefusal;
}
