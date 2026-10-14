// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "benchmarks.h"
#include "utils.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <assert.h>
#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef TRACY_ENABLE
#include <tracy/TracyC.h>
#endif

#define ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )
#define MAYBE_UNUSED( x ) ( (void)( x ) )

typedef void CapacityFcn( b3Capacity* capacity );
typedef void CreateFcn( b3WorldId worldId );
typedef void DestroyFcn( void );
typedef void StepFcn( b3WorldId worldId, int stepCount );

typedef struct Benchmark
{
	const char* name;
	CapacityFcn* capacityFcn;
	CreateFcn* createFcn;
	DestroyFcn* destroyFcn;
	StepFcn* stepFcn;
	int totalStepCount;
} Benchmark;

static void MinProfile( b3Profile* p1, const b3Profile* p2 )
{
	p1->step = b3FixMin( p1->step, p2->step );
	p1->pairs = b3FixMin( p1->pairs, p2->pairs );
	p1->collide = b3FixMin( p1->collide, p2->collide );
	p1->constraints = b3FixMin( p1->constraints, p2->constraints );
	p1->transforms = b3FixMin( p1->transforms, p2->transforms );
	p1->refit = b3FixMin( p1->refit, p2->refit );
	p1->sleepIslands = b3FixMin( p1->sleepIslands, p2->sleepIslands );
}

// Match a name=value option in either its short or long spelling. Returns the value or NULL.
static const char* MatchValue( const char* arg, const char* shortName, const char* longName )
{
	size_t n = strlen( shortName );
	if ( strncmp( arg, shortName, n ) == 0 )
	{
		return arg + n;
	}
	n = strlen( longName );
	if ( strncmp( arg, longName, n ) == 0 )
	{
		return arg + n;
	}
	return NULL;
}

// Distinguish a numeric benchmark index from a name. Empty string is not numeric.
static bool IsAllDigits( const char* s )
{
	if ( *s == 0 )
	{
		return false;
	}
	for ( ; *s != 0; ++s )
	{
		if ( isdigit( (unsigned char)*s ) == 0 )
		{
			return false;
		}
	}
	return true;
}

static bool EqualsIgnoreCase( const char* a, const char* b )
{
	while ( *a != 0 && *b != 0 )
	{
		if ( tolower( (unsigned char)*a ) != tolower( (unsigned char)*b ) )
		{
			return false;
		}
		++a;
		++b;
	}
	return *a == *b;
}

static int FindBenchmark( const Benchmark* benchmarks, int count, const char* name )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( EqualsIgnoreCase( benchmarks[i].name, name ) )
		{
			return i;
		}
	}
	return -1;
}

static void PrintBenchmarks( const Benchmark* benchmarks, int count )
{
	printf( "Registered benchmarks:\n" );
	for ( int i = 0; i < count; ++i )
	{
		printf( "  %2d  %s\n", i, benchmarks[i].name );
	}
}

// Fixed3D benchmark application. On Windows it is important to use affinity avoid cross CCD
// usage or efficiency cores. Also on Windows create a power plan with Processor power management
// Min/Max of 99%. This prevents boosting and makes the benchmarks more repeatable.
// Affinity [0x01 0x02 0x04 0x08 0x10 0x20 0x40 0x80]

// Run all benchmarks with 1 to 8 threads.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=8

// Run all benchmarks with 4 workers only.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4

// List the registered benchmarks.
// .\build\bin\Release\benchmark.exe -l

// Run a single benchmark by name with 4 workers.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 --benchmark=many_pyramids -r=5

// Run benchmark 3 with 4 workers and repeat 20 times. Record the step times.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 -b=3 -r=20 -s

// Run benchmark 3 with 4 workers and run once. Disable continuous collision. Record the step times.
// start /affinity 0x5555 .\build\bin\Release\benchmark.exe -t=4 -w=4 -b=3 -r=1 -nc -s

// ------------------------------------------------------------------------------------------
// Perf-harness: per-phase profiling output.
//
// The b3Profile fields are b3Fixed milliseconds (Q48.16). We accumulate them as b3Fixed
// across the whole run, then divide by the step count (integer division, still Q48.16)
// and convert to nanoseconds in one place, at output, via b3FixToDouble() * 1e6. The
// conversion happens exactly once per printed value so no rounding error accumulates.
// ------------------------------------------------------------------------------------------

#define PERF_PHASE_COUNT 17

static const char* kPerfPhaseNames[PERF_PHASE_COUNT] = {
	"pairs",			  "collide",		   "solve",
	"solverSetup",		  "constraints",	   "prepareConstraints",
	"integrateVelocities", "warmStart",		   "solveImpulses",
	"integratePositions",  "relaxImpulses",	   "applyRestitution",
	"storeImpulses",	  "transforms",		   "refit",
	"bullets",			  "sleepIslands",
};

static void AddProfile( b3Fixed* sums, const b3Profile* p )
{
	sums[0] += p->pairs;
	sums[1] += p->collide;
	sums[2] += p->solve;
	sums[3] += p->solverSetup;
	sums[4] += p->constraints;
	sums[5] += p->prepareConstraints;
	sums[6] += p->integrateVelocities;
	sums[7] += p->warmStart;
	sums[8] += p->solveImpulses;
	sums[9] += p->integratePositions;
	sums[10] += p->relaxImpulses;
	sums[11] += p->applyRestitution;
	sums[12] += p->storeImpulses;
	sums[13] += p->transforms;
	sums[14] += p->refit;
	sums[15] += p->bullets;
	sums[16] += p->sleepIslands;
	sums[17] += p->step;
}

// ------------------------------------------------------------------------------------------
// Perf-harness: --phases-json accumulation.
// ------------------------------------------------------------------------------------------

typedef struct PerfJsonEntry
{
	const char* scene;
	int workers;
	int repeats;
	double stats[PERF_PHASE_COUNT + 1][3]; // [phase][min, median, max], ns per step
} PerfJsonEntry;

static PerfJsonEntry* g_perfJson = NULL;
static int g_perfJsonCount = 0;
static int g_perfJsonCapacity = 0;

static void PerfJsonAppend( PerfJsonEntry entry )
{
	if ( g_perfJsonCount == g_perfJsonCapacity )
	{
		int newCapacity = g_perfJsonCapacity == 0 ? 8 : 2 * g_perfJsonCapacity;
		PerfJsonEntry* next = (PerfJsonEntry*)realloc( g_perfJson, (size_t)newCapacity * sizeof( PerfJsonEntry ) );
		assert( next != NULL );
		g_perfJson = next;
		g_perfJsonCapacity = newCapacity;
	}
	g_perfJson[g_perfJsonCount++] = entry;
}

static int CompareDouble( const void* a, const void* b )
{
	double da = *(const double*)a;
	double db = *(const double*)b;
	return ( da > db ) - ( da < db );
}

static void PerfJsonWrite( const char* fileName )
{
	FILE* file = fopen( fileName, "w" );
	if ( file == NULL )
	{
		printf( "Could not open %s for writing\n", fileName );
		return;
	}

	fprintf( file, "[\n" );
	for ( int i = 0; i < g_perfJsonCount; ++i )
	{
		const PerfJsonEntry* entry = g_perfJson + i;
		fprintf( file, "%s{\n", i == 0 ? "" : ",\n" );
		fprintf( file, "  \"scene\": \"%s\",\n", entry->scene );
		fprintf( file, "  \"workers\": %d,\n", entry->workers );
		fprintf( file, "  \"repeats\": %d,\n", entry->repeats );
		fprintf( file, "  \"ns_per_step\": {\n" );
		for ( int phase = 0; phase <= PERF_PHASE_COUNT; ++phase )
		{
			const char* name = phase == PERF_PHASE_COUNT ? "step" : kPerfPhaseNames[phase];
			fprintf( file, "    \"%s\": {\"min\": %.3f, \"median\": %.3f, \"max\": %.3f}%s\n", name,
					 entry->stats[phase][0], entry->stats[phase][1], entry->stats[phase][2],
					 phase == PERF_PHASE_COUNT ? "" : "," );
		}
		fprintf( file, "  }\n" );
		fprintf( file, "}" );
	}
	fprintf( file, "\n]\n" );
	fclose( file );
}

int main( int argc, char** argv )
{
#ifdef TRACY_ENABLE
	___tracy_startup_profiler();
#endif

	Benchmark benchmarks[] = {
		{ "convex_pile", NULL, CreateConvexPile, NULL, NULL, 500 },
		{ "joint_grid", NULL, CreateJointGrid, NULL, NULL, 100 },
		{ "junkyard", NULL, CreateJunkyard, NULL, StepJunkyard, 500 },
		{ "large_pyramid", NULL, CreateLargePyramid, NULL, NULL, 200 },
		{ "large_world", GetLargeWorldCapacity, CreateLargeWorld, NULL, StepLargeWorld, 500 },
		{ "many_pyramids", NULL, CreateManyPyramids, NULL, NULL, 100 },
		{ "rain", GetRainCapacity, CreateRain, DestroyRain, StepRain, 400 },
		{ "trees100", NULL, CreateTrees100, DestroyTrees, NULL, 500 },
		{ "trees50", NULL, CreateTrees50, DestroyTrees, NULL, 500 },
		{ "trees25", NULL, CreateTrees25, DestroyTrees, NULL, 500 },
		{ "washer", GetWasherCapacity, CreateWasher, NULL, NULL, 1000 },
		{ "ragdoll_pile", GetRagdollPileCapacity, CreateRagdollPile, DestroyRagdollPile, NULL, 500 },
		{ "bodies_20k", GetBodies20kCapacity, CreateBodies20k, NULL, NULL, 500 },
		//{ "smash", CreateSmash, NULL, 300 },
		//{ "spinner", CreateSpinner, StepSpinner, 1400 },
		//{ "tumbler", CreateTumbler, NULL, 750 },
	};

	int benchmarkCount = ARRAY_COUNT( benchmarks );

	int maxSteps = benchmarks[0].totalStepCount;
	for ( int i = 1; i < benchmarkCount; ++i )
	{
		maxSteps = b3MaxInt( maxSteps, benchmarks[i].totalStepCount );
	}

	b3Profile maxProfile = {
		.step = B3_FIXED_MAX,
		.pairs = B3_FIXED_MAX,
		.collide = B3_FIXED_MAX,
		.solve = B3_FIXED_MAX,
		.solverSetup = B3_FIXED_MAX,
		.constraints = B3_FIXED_MAX,
		.prepareConstraints = B3_FIXED_MAX,
		.integrateVelocities = B3_FIXED_MAX,
		.warmStart = B3_FIXED_MAX,
		.solveImpulses = B3_FIXED_MAX,
		.integratePositions = B3_FIXED_MAX,
		.relaxImpulses = B3_FIXED_MAX,
		.applyRestitution = B3_FIXED_MAX,
		.storeImpulses = B3_FIXED_MAX,
		.splitIslands = B3_FIXED_MAX,
		.transforms = B3_FIXED_MAX,
		.hitEvents = B3_FIXED_MAX,
		.refit = B3_FIXED_MAX,
		.bullets = B3_FIXED_MAX,
		.sleepIslands = B3_FIXED_MAX,
	};

	b3Profile* profiles = malloc( maxSteps * sizeof( b3Profile ) );
	for ( int i = 0; i < maxSteps; ++i )
	{
		profiles[i] = maxProfile;
	}

	int maxThreadCount = GetNumberOfCores();
	int runCount = 4;
	int singleBenchmark = -1;
	int singleWorkerCount = -1;
	b3Counters counters = { 0 };
	bool enableContinuous = true;
	bool recordStepTimes = false;
	bool hashEnabled = false;
	int hashEvery = 0;
	const char* phasesJson = NULL;

	assert( maxThreadCount <= B3_MAX_WORKERS );

	for ( int i = 1; i < argc; ++i )
	{
		const char* arg = argv[i];
		const char* value;
		if ( ( value = MatchValue( arg, "-t=", "--threads=" ) ) != NULL )
		{
			int threadCount = atoi( value );
			maxThreadCount = b3ClampInt( threadCount, 1, maxThreadCount );
		}
		else if ( ( value = MatchValue( arg, "-b=", "--benchmark=" ) ) != NULL )
		{
			// Accept an index for backward compatibility, otherwise an exact name
			if ( IsAllDigits( value ) )
			{
				singleBenchmark = b3ClampInt( atoi( value ), 0, benchmarkCount - 1 );
			}
			else
			{
				singleBenchmark = FindBenchmark( benchmarks, benchmarkCount, value );
				if ( singleBenchmark == -1 )
				{
					printf( "No benchmark named \"%s\"\n", value );
					PrintBenchmarks( benchmarks, benchmarkCount );
					exit( 1 );
				}
			}
		}
		else if ( ( value = MatchValue( arg, "-w=", "--workers=" ) ) != NULL )
		{
			singleWorkerCount = atoi( value );
		}
		else if ( ( value = MatchValue( arg, "-r=", "--repeats=" ) ) != NULL )
		{
			runCount = b3ClampInt( atoi( value ), 1, 1000 );
		}
		else if ( strncmp( arg, "--hash-every=", 13 ) == 0 )
		{
			hashEvery = atoi( arg + 13 );
			hashEnabled = true;
		}
		else if ( strcmp( arg, "--hash" ) == 0 )
		{
			hashEnabled = true;
		}
		else if ( strncmp( arg, "--phases-json=", 14 ) == 0 )
		{
			phasesJson = arg + 14;
		}
		else if ( strcmp( arg, "-nc" ) == 0 || strcmp( arg, "--no-continuous" ) == 0 )
		{
			enableContinuous = false;
			printf( "Continuous disabled\n" );
		}
		else if ( strcmp( arg, "-s" ) == 0 || strcmp( arg, "--record-steps" ) == 0 )
		{
			recordStepTimes = true;
		}
		else if ( strcmp( arg, "-l" ) == 0 || strcmp( arg, "--list" ) == 0 )
		{
			PrintBenchmarks( benchmarks, benchmarkCount );
			exit( 0 );
		}
		else if ( strcmp( arg, "-h" ) == 0 || strcmp( arg, "--help" ) == 0 )
		{
			printf( "Usage\n"
					"-t, --threads=<integer>: the maximum number of threads to use\n"
					"-b, --benchmark=<index|name>: run a single benchmark by index or name\n"
					"-w, --workers=<integer>: run a single worker count\n"
					"-r, --repeats=<integer>: number of repeats (default is 4)\n"
					"-nc, --no-continuous: disable continuous collision\n"
					"-s, --record-steps: record step times\n"
					"--hash: print a deterministic FNV-1a/64 body-state hash after the last step\n"
					"--hash-every=<k>: also print the hash every k steps\n"
					"--phases-json=<file>: write per-phase min/median/max nanoseconds per step\n"
					"-l, --list: list the registered benchmarks\n"
					"-h, --help: print this help\n" );
			exit( 0 );
		}
		else
		{
			printf( "Unknown option \"%s\", use -h for help\n", arg );
		}
	}

	if ( singleWorkerCount != -1 )
	{
		singleWorkerCount = b3ClampInt( singleWorkerCount, 1, maxThreadCount );
	}

	printf( "Starting benchmarks\n" );
	printf( "======================================\n" );

	for ( int benchmarkIndex = 0; benchmarkIndex < benchmarkCount; ++benchmarkIndex )
	{
		if ( singleBenchmark != -1 && benchmarkIndex != singleBenchmark )
		{
			continue;
		}

#ifdef NDEBUG
		int stepCount = benchmarks[benchmarkIndex].totalStepCount;
#else
		int stepCount = 10;
#endif

		Benchmark* benchmark = benchmarks + benchmarkIndex;

		bool countersAcquired = false;

		printf( "benchmark: %s, steps = %d\n", benchmarks[benchmarkIndex].name, stepCount );

		b3Fixed minTime[B3_MAX_WORKERS] = { 0 };

		for ( int threadCount = 1; threadCount <= maxThreadCount; ++threadCount )
		{
			if ( singleWorkerCount != -1 && singleWorkerCount != threadCount )
			{
				continue;
			}

			printf( "thread count: %d\n", threadCount );

			uint64_t sceneHash = 0;
			bool sceneHashComputed = false;

			// Per-run per-phase nanoseconds per step, for the --phases-json output.
			double* runNs = NULL;
			if ( phasesJson != NULL )
			{
				runNs = (double*)malloc( (size_t)runCount * ( PERF_PHASE_COUNT + 1 ) * sizeof( double ) );
			}

			for ( int runIndex = 0; runIndex < runCount; ++runIndex )
			{
				b3WorldDef worldDef = b3DefaultWorldDef();
				worldDef.enableContinuous = enableContinuous;
				worldDef.workerCount = threadCount;

				if (benchmark->capacityFcn != NULL)
				{
					benchmark->capacityFcn( &worldDef.capacity );
				}

				b3WorldId worldId = b3CreateWorld( &worldDef );

				if ( hashEnabled )
				{
					b3BenchmarkResetBodies();
				}

				benchmark->createFcn( worldId );

				b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
				int subStepCount = 4;

				// Initial step can be expensive and skew benchmark
				if ( benchmark->stepFcn != NULL )
				{
					benchmark->stepFcn( worldId, 0 );
				}

				assert( stepCount <= maxSteps );

				b3Fixed phaseSums[PERF_PHASE_COUNT + 1] = { 0 };

				b3World_Step( worldId, timeStep, subStepCount );

				b3Profile profile = b3World_GetProfile( worldId );
				MinProfile( profiles + 0, &profile );
				AddProfile( phaseSums, &profile );

				uint64_t ticks = b3GetTicks();

				for ( int stepIndex = 1; stepIndex < stepCount; ++stepIndex )
				{
					if ( benchmark->stepFcn != NULL )
					{
						benchmark->stepFcn( worldId, stepIndex );
					}

					b3World_Step( worldId, timeStep, subStepCount );

					profile = b3World_GetProfile( worldId );
					MinProfile( profiles + stepIndex, &profile );
					AddProfile( phaseSums, &profile );

					if ( hashEvery > 0 && runIndex == 0 && ( stepIndex % hashEvery ) == 0 )
					{
						uint64_t runningHash = b3BenchmarkHashBodies( worldId );
						printf( "HASH %s workers=%d step=%d %016" PRIx64 "\n", benchmark->name, threadCount, stepIndex,
								runningHash );
					}
				}

				if ( hashEnabled && runIndex == 0 )
				{
					sceneHash = b3BenchmarkHashBodies( worldId );
					sceneHashComputed = true;
				}

				b3Fixed ms = b3GetMilliseconds( ticks );
				printf( "run %d : %g (ms)\n", runIndex, b3FixToDouble( ms ) );

				if ( runIndex == 0 )
				{
					minTime[threadCount - 1] = ms;
				}
				else
				{
					minTime[threadCount - 1] = b3FixMin( minTime[threadCount - 1], ms );
				}

				if ( phasesJson != NULL )
				{
					// Convert the b3Fixed millisecond sums to nanoseconds per step exactly once,
					// here: divide by the step count (Q48.16 integer division) then scale.
					for ( int phase = 0; phase <= PERF_PHASE_COUNT; ++phase )
					{
						double nsPerStep = b3FixToDouble( phaseSums[phase] / stepCount ) * 1e6;
						runNs[runIndex * ( PERF_PHASE_COUNT + 1 ) + phase] = nsPerStep;
					}
				}

				if ( countersAcquired == false )
				{
					counters = b3World_GetCounters( worldId );
					countersAcquired = true;
				}

				if ( benchmark->destroyFcn != NULL )
				{
					benchmark->destroyFcn();
				}

				b3DestroyWorld( worldId );
			}

			if ( hashEnabled && sceneHashComputed )
			{
				printf( "HASH %s workers=%d %016" PRIx64 "\n", benchmark->name, threadCount, sceneHash );
			}

			if ( phasesJson != NULL )
			{
				PerfJsonEntry entry;
				entry.scene = benchmark->name;
				entry.workers = threadCount;
				entry.repeats = runCount;
				for ( int phase = 0; phase <= PERF_PHASE_COUNT; ++phase )
				{
					double* values = (double*)malloc( (size_t)runCount * sizeof( double ) );
					for ( int runIndex = 0; runIndex < runCount; ++runIndex )
					{
						values[runIndex] = runNs[runIndex * ( PERF_PHASE_COUNT + 1 ) + phase];
					}
					qsort( values, (size_t)runCount, sizeof( double ), CompareDouble );
					entry.stats[phase][0] = values[0];
					entry.stats[phase][2] = values[runCount - 1];
					entry.stats[phase][1] = ( runCount % 2 ) == 0
												? 0.5 * ( values[runCount / 2 - 1] + values[runCount / 2] )
												: values[runCount / 2];
					free( values );
				}
				PerfJsonAppend( entry );
				free( runNs );
			}

			if ( recordStepTimes )
			{
				char fileName[64] = { 0 };
				snprintf( fileName, 64, "%s_t%d.dat", benchmarks[benchmarkIndex].name, threadCount );
				FILE* file = fopen( fileName, "w" );
				if ( file == NULL )
				{
					continue;
				}

				for ( int stepIndex = 0; stepIndex < stepCount; ++stepIndex )
				{
					b3Profile p = profiles[stepIndex];
					fprintf( file, "%g %g %g %g %g %g %g\n", b3FixToDouble( p.step ), b3FixToDouble( p.pairs ), b3FixToDouble( p.collide ), b3FixToDouble( p.constraints ), b3FixToDouble( p.transforms ),
							 b3FixToDouble( p.refit ), b3FixToDouble( p.sleepIslands ) );
				}

				fclose( file );
			}
		}

		printf( "body %d / shape %d / contact %d / joint %d / stack %d\n\n", counters.bodyCount, counters.shapeCount,
				counters.contactCount, counters.jointCount, counters.stackUsed );

		char fileName[64] = { 0 };
		snprintf( fileName, 64, "%s.csv", benchmarks[benchmarkIndex].name );
		FILE* file = fopen( fileName, "w" );
		if ( file == NULL )
		{
			continue;
		}

		fprintf( file, "threads,ms\n" );
		for ( int threadIndex = 1; threadIndex <= maxThreadCount; ++threadIndex )
		{
			fprintf( file, "%d,%g\n", threadIndex, b3FixToDouble( minTime[threadIndex - 1] ) );
		}

		fclose( file );
	}

	if ( phasesJson != NULL )
	{
		PerfJsonWrite( phasesJson );
	}

	printf( "======================================\n" );
	printf( "All benchmarks complete!\n" );

	free( profiles );

#ifdef TRACY_ENABLE
	___tracy_shutdown_profiler();
#endif

	return 0;
}
