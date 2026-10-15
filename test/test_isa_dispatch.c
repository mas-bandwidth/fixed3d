// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// Runtime ISA dispatch (src/isa.h). The parser tests feed faked cpuid, xgetbv,
// hwcap and sysctl words: no real host state, no time, no I/O. The variant
// test forces each compiled variant the host supports and runs the
// determinism goldens through it, so every variant must reproduce the scalar
// goldens bit for bit.

#include "isa.h"
#include "test_macros.h"

#include "box3d/base.h"
#include "box3d/box3d.h"

#include <stdio.h>
#include <string.h>

extern int DeterminismTest( void );

// CPUID.1:ECX bits
#define LEAF1_FMA ( 1u << 12 )
#define LEAF1_OSXSAVE ( 1u << 27 )
#define LEAF1_AVX ( 1u << 28 )

// CPUID.(7,0):EBX / ECX bits
#define LEAF7_AVX2 ( 1u << 5 )
#define LEAF7_AVX512F ( 1u << 16 )
#define LEAF7_AVX512DQ ( 1u << 17 )
#define LEAF7_AVX512IFMA ( 1u << 21 )
#define LEAF7_AVX512CD ( 1u << 28 )
#define LEAF7_AVX512BW ( 1u << 30 )
#define LEAF7_AVX512VL ( 1u << 31 )
#define LEAF7C_VBMI ( 1u << 1 )
#define LEAF7C_VBMI2 ( 1u << 6 )
#define LEAF7C_GFNI ( 1u << 8 )

// XCR0: SSE|AVX state, and the AVX-512 opmask|ZMM_Hi256|Hi16_ZMM state
#define XCR0_AVX 0x6ull
#define XCR0_AVX512 0xE0ull

static int IsaDetectParseX86( void )
{
	uint32_t leaf1 = LEAF1_FMA | LEAF1_OSXSAVE | LEAF1_AVX;
	uint32_t leaf7 = LEAF7_AVX2 | LEAF7_AVX512F | LEAF7_AVX512DQ | LEAF7_AVX512IFMA | LEAF7_AVX512CD | LEAF7_AVX512BW |
					 LEAF7_AVX512VL;
	uint32_t leaf7c = LEAF7C_VBMI | LEAF7C_VBMI2 | LEAF7C_GFNI;

	// A Zen 4 class host with an OS that saves every state
	b3IsaFacts f = b3IsaParseX86( leaf1, leaf7, leaf7c, XCR0_AVX | XCR0_AVX512 );
	ENSURE( f.osAvx && f.osAvx512 );
	ENSURE( f.avx2 && f.fma );
	ENSURE( f.avx512f && f.avx512dq && f.avx512vl && f.avx512bw && f.avx512cd );
	ENSURE( f.avx512ifma && f.avx512vbmi && f.avx512vbmi2 && f.gfni );
	ENSURE( b3IsaMissingFact( b3_isaAvx512, &f ) == NULL );
	ENSURE( b3IsaMissingFact( b3_isaAvx512Ifma, &f ) == NULL );

	// The same CPU under an OS that does not save the ZMM state: no AVX-512 at
	// all, AVX2 still usable
	f = b3IsaParseX86( leaf1, leaf7, leaf7c, XCR0_AVX );
	ENSURE( f.osAvx && f.osAvx512 == false );
	ENSURE( f.avx2 );
	ENSURE( f.avx512f == false && f.avx512dq == false && f.avx512vl == false && f.avx512ifma == false );
	ENSURE( strcmp( b3IsaMissingFact( b3_isaAvx512, &f ), "avx512f" ) == 0 );
	ENSURE( b3IsaBestVariant( &f ) == b3_isaScalar );

	// Opmask saved but not the ZMM upper halves: still no AVX-512
	f = b3IsaParseX86( leaf1, leaf7, leaf7c, XCR0_AVX | 0x20ull );
	ENSURE( f.avx512f == false );

	// OSXSAVE clear: XGETBV was not run, the caller passes 0, nothing AVX
	f = b3IsaParseX86( LEAF1_FMA | LEAF1_AVX, leaf7, leaf7c, 0 );
	ENSURE( f.osAvx == false && f.avx2 == false && f.fma == false && f.avx512f == false );

	// AVX-512F without VL (Knights Landing): the four-lane variant needs VL
	f = b3IsaParseX86( leaf1, LEAF7_AVX2 | LEAF7_AVX512F | LEAF7_AVX512CD, 0, XCR0_AVX | XCR0_AVX512 );
	ENSURE( f.avx512f && f.avx512vl == false );
	ENSURE( strcmp( b3IsaMissingFact( b3_isaAvx512, &f ), "avx512dq" ) == 0 );
	f = b3IsaParseX86( leaf1, LEAF7_AVX2 | LEAF7_AVX512F | LEAF7_AVX512DQ, 0, XCR0_AVX | XCR0_AVX512 );
	ENSURE( strcmp( b3IsaMissingFact( b3_isaAvx512, &f ), "avx512vl" ) == 0 );
	ENSURE( b3IsaBestVariant( &f ) == b3_isaScalar );

	// An AVX2-only host (Haswell class)
	f = b3IsaParseX86( leaf1, LEAF7_AVX2, 0, XCR0_AVX );
	ENSURE( f.avx2 && f.fma && f.avx512f == false );

	// Nothing at all
	f = b3IsaParseX86( 0, 0, 0, 0 );
	ENSURE( f.osAvx == false && f.avx2 == false && f.gfni == false );
	ENSURE( b3IsaBestVariant( &f ) == b3_isaScalar );

	// The best variant on a full AVX-512 host is avx512 exactly when it is compiled
	f = b3IsaParseX86( leaf1, leaf7, leaf7c, XCR0_AVX | XCR0_AVX512 );
	ENSURE( b3IsaBestVariant( &f ) == ( b3IsaIsCompiled( b3_isaAvx512 ) ? b3_isaAvx512 : b3_isaScalar ) );
	return 0;
}

static int IsaDetectParseArm( void )
{
	// Linux: HWCAP_ASIMD (1 << 1); HWCAP2_SVE2 (1 << 1), HWCAP2_SME (1 << 23),
	// HWCAP2_SME2 (1 << 37)
	b3IsaFacts f = b3IsaParseLinuxArm( 1ull << 1, ( 1ull << 1 ) | ( 1ull << 23 ) | ( 1ull << 37 ) );
	ENSURE( f.asimd && f.sve2 && f.sme && f.sme2 );
	ENSURE( f.avx2 == false && f.avx512f == false );

	f = b3IsaParseLinuxArm( 1ull << 1, 0 );
	ENSURE( f.asimd && f.sve2 == false && f.sme == false && f.sme2 == false );
	ENSURE( strcmp( b3IsaMissingFact( b3_isaSme2, &f ), "sme2" ) == 0 );

	// SME2 bit without SME is not SME2
	f = b3IsaParseLinuxArm( 1ull << 1, 1ull << 37 );
	ENSURE( f.sme2 == false );

	f = b3IsaParseLinuxArm( 0, 0 );
	ENSURE( f.asimd == false );
	ENSURE( strcmp( b3IsaMissingFact( b3_isaNeon, &f ), "asimd" ) == 0 );

	// macOS sysctl: an M4 / M5 class host (FEAT_SME2, 64-byte streaming vectors)
	f = b3IsaParseMacArm( 1, 1, 0, 64 );
	ENSURE( f.asimd && f.sme && f.sme2 && f.sve2 == false && f.smeMaxSvlBytes == 64 );
	ENSURE( b3IsaMissingFact( b3_isaSme2, &f ) == NULL );

	// An M3 class host: the names are missing, read as 0
	f = b3IsaParseMacArm( 0, 0, 0, 0 );
	ENSURE( f.asimd && f.sme == false && f.sme2 == false && f.smeMaxSvlBytes == 0 );
	return 0;
}

int IsaDetectParseTest( void )
{
	RUN_SUBTEST( IsaDetectParseX86 );
	RUN_SUBTEST( IsaDetectParseArm );
	return 0;
}

int IsaOverrideRefusesTest( void )
{
	// Names for variants not built yet are reserved and refused as not compiled
	const char* reserved[] = { "avx2", "avx512ifma", "neon", "sme2" };
	for ( int i = 0; i < ARRAY_COUNT( reserved ); ++i )
	{
		ENSURE( b3IsaIsCompiled( (b3IsaVariant)b3IsaVariantFromName( reserved[i] ) ) == false );
		ENSURE( b3ForceIsa( reserved[i] ) == false );
		ENSURE( strstr( b3GetIsaRefusal(), "not compiled" ) != NULL );
		printf( "  refused %s: %s\n", reserved[i], b3GetIsaRefusal() );
	}

	// Unknown names
	ENSURE( b3ForceIsa( "avx1024" ) == false );
	ENSURE( strstr( b3GetIsaRefusal(), "unknown ISA variant" ) != NULL );

	// A compiled variant whose facts are false: the pure check on faked facts,
	// so it runs on every host
	b3IsaFacts noZmm = b3IsaParseX86( LEAF1_FMA | LEAF1_OSXSAVE | LEAF1_AVX,
									  LEAF7_AVX2 | LEAF7_AVX512F | LEAF7_AVX512DQ | LEAF7_AVX512VL, 0, XCR0_AVX );
	char reason[160];
	int checked = b3IsaCheckForce( "avx512", &noZmm, reason, (int)sizeof( reason ) );
	ENSURE( checked == -1 );
	if ( b3IsaIsCompiled( b3_isaAvx512 ) )
	{
		ENSURE( strstr( reason, "host lacks avx512f" ) != NULL );
	}
	else
	{
		ENSURE( strstr( reason, "not compiled" ) != NULL );
	}
	printf( "  refused avx512 on a host without ZMM state: %s\n", reason );

	// And on this host, if it lacks the facts, the API refuses rather than
	// replacing the variant
	const char* missing = b3IsaMissingFact( b3_isaAvx512, b3IsaHostFacts() );
	if ( b3IsaIsCompiled( b3_isaAvx512 ) && missing != NULL )
	{
		ENSURE( b3ForceIsa( "avx512" ) == false );
		ENSURE( strstr( b3GetIsaRefusal(), missing ) != NULL );
	}

	// A refused force leaves the selection where it was
	ENSURE( b3ForceIsa( "scalar" ) );
	ENSURE( b3ForceIsa( "sme2" ) == false );
	ENSURE( strcmp( b3GetIsaName(), "scalar" ) == 0 );

	// Forcing is refused while a world lives, and the world keeps its variant
	b3WorldDef def = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &def );
	ENSURE( b3ForceIsa( "scalar" ) == false );
	ENSURE( strstr( b3GetIsaRefusal(), "world(s) live" ) != NULL );
	ENSURE( b3ForceIsa( NULL ) == false );
	b3DestroyWorld( worldId );

	// Back to the detected variant
	ENSURE( b3ForceIsa( NULL ) );
	return 0;
}

int IsaVariantsMatchScalarTest( void )
{
	const b3IsaFacts* host = b3IsaHostFacts();
	printf( "HOST avx2=%d fma=%d avx512f=%d dq=%d bw=%d vl=%d cd=%d ifma=%d vbmi=%d vbmi2=%d gfni=%d asimd=%d sve2=%d sme=%d "
			"sme2=%d svl=%d\n",
			host->avx2, host->fma, host->avx512f, host->avx512dq, host->avx512bw, host->avx512vl, host->avx512cd, host->avx512ifma,
			host->avx512vbmi, host->avx512vbmi2, host->gfni, host->asimd, host->sve2, host->sme, host->sme2, host->smeMaxSvlBytes );

	int ran = 0;
	for ( int v = 0; v < b3_isaVariantCount; ++v )
	{
		b3IsaVariant variant = (b3IsaVariant)v;
		const char* name = b3IsaVariantName( variant );
		if ( b3IsaIsCompiled( variant ) == false )
		{
			printf( "VARIANT %s SKIP not compiled\n", name );
			continue;
		}

		const char* missing = b3IsaMissingFact( variant, b3IsaHostFacts() );
		if ( missing != NULL )
		{
			ENSURE( b3ForceIsa( name ) == false );
			printf( "VARIANT %s SKIP host lacks %s\n", name, missing );
			continue;
		}

		ENSURE( b3ForceIsa( name ) );
		ENSURE( strcmp( b3GetIsaName(), name ) == 0 );

		// The determinism goldens are the scalar goldens: every scene, every
		// worker count, every state hash compared bit for bit
		int result = DeterminismTest();
		ENSURE( b3ForceIsa( NULL ) );
		if ( result != 0 )
		{
			printf( "VARIANT %s FAILED the scalar goldens\n", name );
			return 1;
		}

		printf( "VARIANT %s RAN\n", name );
		ran += 1;
	}

	// The scalar variant is always compiled and always supported
	ENSURE( ran >= 1 );
	return 0;
}
