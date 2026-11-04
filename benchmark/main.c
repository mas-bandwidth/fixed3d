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

#define PERF_FNV_INIT UINT64_C( 14695981039346656037 )
#define PERF_FNV_PRIME UINT64_C( 1099511628211 )
#define PERF_PHASE_COUNT 18

static const char* const kPhaseNames[PERF_PHASE_COUNT] = {
	"pairs",
	"collide",
	"solve",
	"solverSetup",
	"constraints",
	"prepareConstraints",
	"integrateVelocities",
	"warmStart",
	"solveImpulses",
	"integratePositions",
	"relaxImpulses",
	"applyRestitution",
	"storeImpulses",
	"transforms",
	"refit",
	"bullets",
	"sleepIslands",
	"step",
};

typedef struct PerfSample
{
	int64_t raw[PERF_PHASE_COUNT];
} PerfSample;

typedef struct PerfRecord
{
	const char* scene;
	int workers;
	int repeats;
	int timedSteps;
	PerfSample* samples;
} PerfRecord;

static uint64_t HashValue( uint64_t hash, int64_t value )
{
	uint64_t bits = (uint64_t)value;
	for ( int i = 0; i < 8; ++i )
	{
		hash = ( hash ^ ( bits & 255 ) ) * PERF_FNV_PRIME;
		bits >>= 8;
	}
	return hash;
}

static uint64_t HashVector( uint64_t hash, b3Vec3 v )
{
	return HashValue( HashValue( HashValue( hash, v.x ), v.y ), v.z );
}

static int64_t PosComponent( b3Pos p, int axis )
{
#if defined( BOX3D_LUDICROUS_MODE )
	b3Int128 wide = axis == 0 ? p.x : ( axis == 1 ? p.y : p.z );
	return (int64_t)wide;
#else
	return axis == 0 ? p.x : ( axis == 1 ? p.y : p.z );
#endif
}

static uint64_t HashWorldState( b3WorldId worldId )
{
	uint64_t hash = PERF_FNV_INIT;
	b3Counters counters = b3World_GetCounters( worldId );
	int found = 0;
	int limit = counters.bodyCount + 65536;
	uint16_t world0 = (uint16_t)( worldId.index1 - 1 );
	for ( int index1 = 1; index1 <= limit && found < counters.bodyCount; ++index1 )
	{
		b3BodyId id = { index1, world0, 1 };
		bool valid = false;
		for ( uint16_t generation = 1; generation <= 64; ++generation )
		{
			id.generation = generation;
			if ( b3Body_IsValid( id ) )
			{
				valid = true;
				break;
			}
		}
		if ( valid == false )
		{
			continue;
		}
		found += 1;
		b3WorldTransform t = b3Body_GetTransform( id );
		hash = HashValue( hash, PosComponent( t.p, 0 ) );
		hash = HashValue( hash, PosComponent( t.p, 1 ) );
		hash = HashValue( hash, PosComponent( t.p, 2 ) );
		hash = HashVector( hash, t.q.v );
		hash = HashValue( hash, t.q.s );
		hash = HashVector( hash, b3Body_GetLinearVelocity( id ) );
		hash = HashVector( hash, b3Body_GetAngularVelocity( id ) );
	}
	if ( found != counters.bodyCount )
	{
		fprintf( stderr, "hash incomplete: found %d of %d bodies\n", found, counters.bodyCount );
	}
	return hash;
}

static void AccumProfile( PerfSample* sample, const b3Profile* profile )
{
	sample->raw[0] += profile->pairs;
	sample->raw[1] += profile->collide;
	sample->raw[2] += profile->solve;
	sample->raw[3] += profile->solverSetup;
	sample->raw[4] += profile->constraints;
	sample->raw[5] += profile->prepareConstraints;
	sample->raw[6] += profile->integrateVelocities;
	sample->raw[7] += profile->warmStart;
	sample->raw[8] += profile->solveImpulses;
	sample->raw[9] += profile->integratePositions;
	sample->raw[10] += profile->relaxImpulses;
	sample->raw[11] += profile->applyRestitution;
	sample->raw[12] += profile->storeImpulses;
	sample->raw[13] += profile->transforms;
	sample->raw[14] += profile->refit;
	sample->raw[15] += profile->bullets;
	sample->raw[16] += profile->sleepIslands;
	sample->raw[17] += profile->step;
}

static int64_t MsRawToNsPerStep( int64_t rawSum, int steps )
{
	// b3Profile times are Q48.16 milliseconds. Convert exactly once, here, at output:
	// ns = (raw / 65536) * 1000000 / steps = raw * 15625 / (1024 * steps).
	if ( steps <= 0 )
	{
		return 0;
	}
	return ( rawSum * 15625 ) / ( 1024 * (int64_t)steps );
}

static int CompareI64( const void* a, const void* b )
{
	int64_t x = *(const int64_t*)a;
	int64_t y = *(const int64_t*)b;
	return ( x > y ) - ( x < y );
}

static int64_t MedianI64( int64_t* values, int count )
{
	qsort( values, (size_t)count, sizeof( int64_t ), CompareI64 );
	if ( ( count & 1 ) == 1 )
	{
		return values[count / 2];
	}
	return values[count / 2 - 1] + ( values[count / 2] - values[count / 2 - 1] ) / 2;
}

static void WritePhasesJson( const char* path, const PerfRecord* records, int recordCount )
{
	FILE* file = fopen( path, "w" );
	if ( file == NULL )
	{
		printf( "failed to open %s\n", path );
		exit( 1 );
	}
	fprintf( file, "[\n" );
	int first = 1;
	for ( int r = 0; r < recordCount; ++r )
	{
		const PerfRecord* rec = records + r;
		for ( int phase = 0; phase < PERF_PHASE_COUNT; ++phase )
		{
			int64_t* column = malloc( (size_t)rec->repeats * sizeof( int64_t ) );
			int64_t minNs = INT64_MAX;
			int64_t maxNs = 0;
			for ( int i = 0; i < rec->repeats; ++i )
			{
				int64_t ns = MsRawToNsPerStep( rec->samples[i].raw[phase], rec->timedSteps );
				column[i] = ns;
				if ( ns < minNs )
				{
					minNs = ns;
				}
				if ( ns > maxNs )
				{
					maxNs = ns;
				}
			}
			int64_t medianNs = MedianI64( column, rec->repeats );
			free( column );
			if ( minNs == INT64_MAX )
			{
				minNs = 0;
			}
			if ( first == 0 )
			{
				fprintf( file, ",\n" );
			}
			first = 0;
			fprintf( file,
					 "{\"scene\":\"%s\",\"workers\":%d,\"repeats\":%d,\"timed_steps\":%d,\"phase\":\"%s\",\"min_ns\":%" PRId64
					 ",\"median_ns\":%" PRId64 ",\"max_ns\":%" PRId64 "}",
					 rec->scene, rec->workers, rec->repeats, rec->timedSteps, kPhaseNames[phase], minNs, medianNs, maxNs );
		}
	}
	fprintf( file, "\n]\n" );
	fclose( file );
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
		{ "ragdoll_pile", GetRagdollPileCapacity, CreateRagdollPile, DestroyRagdollPile, NULL, 100 },
		{ "bodies_20k", GetBodies20kCapacity, CreateBodies20k, NULL, NULL, 100 },
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
	bool enableHash = false;
	int hashEvery = 0;
	const char* phasesJsonPath = NULL;
	int hashFailed = 0;
	int stepCap = 0;

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
		else if ( strcmp( arg, "-nc" ) == 0 || strcmp( arg, "--no-continuous" ) == 0 )
		{
			enableContinuous = false;
			printf( "Continuous disabled\n" );
		}
		else if ( strcmp( arg, "-s" ) == 0 || strcmp( arg, "--record-steps" ) == 0 )
		{
			recordStepTimes = true;
		}
		else if ( strcmp( arg, "--hash" ) == 0 )
		{
			enableHash = true;
		}
		else if ( ( value = MatchValue( arg, "--hash-every=", "--hash-every=" ) ) != NULL )
		{
			hashEvery = atoi( value );
			if ( hashEvery < 1 )
			{
				printf( "--hash-every requires a positive integer\n" );
				exit( 1 );
			}
			enableHash = true;
		}
		else if ( ( value = MatchValue( arg, "--phases-json=", "--phases-json=" ) ) != NULL )
		{
			phasesJsonPath = value;
			if ( phasesJsonPath[0] == 0 )
			{
				printf( "--phases-json requires a file path\n" );
				exit( 1 );
			}
		}
		else if ( ( value = MatchValue( arg, "--steps=", "--steps=" ) ) != NULL )
		{
			stepCap = atoi( value );
			if ( stepCap < 1 )
			{
				printf( "--steps requires a positive integer\n" );
				exit( 1 );
			}
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
					"--hash: print HASH <scene> workers=<n> <16 hex> after the last step\n"
					"--hash-every=<k>: also print a running hash every k steps\n"
					"--phases-json=<file>: min/median/max ns per step for each profile phase\n"
					"--steps=<n>: cap the step count of every scene (default unchanged)\n"
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

	PerfRecord* records = NULL;
	int recordCount = 0;
	int recordCap = 0;

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
		if ( stepCap > 0 && stepCount > stepCap )
		{
			stepCount = stepCap;
		}

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

			PerfSample* samples = NULL;
			uint64_t* runHashes = NULL;
			int timedSteps = stepCount > 1 ? stepCount - 1 : stepCount;
			if ( phasesJsonPath != NULL )
			{
				samples = calloc( (size_t)runCount, sizeof( PerfSample ) );
			}
			if ( enableHash )
			{
				runHashes = calloc( (size_t)runCount, sizeof( uint64_t ) );
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

				benchmark->createFcn( worldId );

				b3Fixed timeStep = b3FixDiv( B3_FIX( 1.0f ) , B3_FIX( 60.0f ) );
				int subStepCount = 4;

				// Initial step can be expensive and skew benchmark
				if ( benchmark->stepFcn != NULL )
				{
					benchmark->stepFcn( worldId, 0 );
				}

				assert( stepCount <= maxSteps );

				b3World_Step( worldId, timeStep, subStepCount );

				b3Profile profile = b3World_GetProfile( worldId );
				MinProfile( profiles + 0, &profile );
				if ( samples != NULL && stepCount == 1 )
				{
					AccumProfile( samples + runIndex, &profile );
				}
				if ( enableHash && hashEvery > 0 && ( 1 % hashEvery ) == 0 )
				{
					uint64_t stepHash = HashWorldState( worldId );
					printf( "HASH %s workers=%d step=%d %016" PRIx64 "\n", benchmarks[benchmarkIndex].name, threadCount, 1, stepHash );
				}

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
					if ( samples != NULL )
					{
						AccumProfile( samples + runIndex, &profile );
					}
					if ( enableHash && hashEvery > 0 && ( ( stepIndex + 1 ) % hashEvery ) == 0 )
					{
						uint64_t stepHash = HashWorldState( worldId );
						printf( "HASH %s workers=%d step=%d %016" PRIx64 "\n", benchmarks[benchmarkIndex].name, threadCount, stepIndex + 1,
								stepHash );
					}
				}

				b3Fixed ms = b3GetMilliseconds( ticks );
				printf( "run %d : %g (ms)\n", runIndex, b3FixToDouble( ms ) );

				if ( enableHash )
				{
					runHashes[runIndex] = HashWorldState( worldId );
				}

				if ( runIndex == 0 )
				{
					minTime[threadCount - 1] = ms;
				}
				else
				{
					minTime[threadCount - 1] = b3FixMin( minTime[threadCount - 1], ms );
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

			if ( enableHash )
			{
				for ( int runIndex = 1; runIndex < runCount; ++runIndex )
				{
					if ( runHashes[runIndex] != runHashes[0] )
					{
						fprintf( stderr, "HASH MISMATCH %s workers=%d\n", benchmarks[benchmarkIndex].name, threadCount );
						hashFailed = 1;
					}
				}
				printf( "HASH %s workers=%d %016" PRIx64 "\n", benchmarks[benchmarkIndex].name, threadCount, runHashes[0] );
				fflush( stdout );
			}
			free( runHashes );

			if ( samples != NULL )
			{
				if ( recordCount == recordCap )
				{
					recordCap = recordCap == 0 ? 8 : recordCap * 2;
					records = realloc( records, (size_t)recordCap * sizeof( PerfRecord ) );
				}
				records[recordCount].scene = benchmarks[benchmarkIndex].name;
				records[recordCount].workers = threadCount;
				records[recordCount].repeats = runCount;
				records[recordCount].timedSteps = timedSteps;
				records[recordCount].samples = samples;
				recordCount += 1;
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

	printf( "======================================\n" );
	printf( "All benchmarks complete!\n" );

	if ( phasesJsonPath != NULL )
	{
		WritePhasesJson( phasesJsonPath, records, recordCount );
	}
	for ( int i = 0; i < recordCount; ++i )
	{
		free( records[i].samples );
	}
	free( records );
	free( profiles );

#ifdef TRACY_ENABLE
	___tracy_shutdown_profiler();
#endif

	return hashFailed;
}
