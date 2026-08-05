cmake_minimum_required(VERSION 3.22)
include(CMakeDependentOption)

project(box3d
	VERSION 1.4.0
	DESCRIPTION "Fixed3D: Box3D in Q48.16 fixed point"
	HOMEPAGE_URL "https://github.com/mas-bandwidth/fixed3d"
	LANGUAGES C CXX
)

# stuff to help debug cmake
# message(STATUS "cmake tool chain: ${CMAKE_TOOLCHAIN_FILE}")
# message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")
# message(STATUS "library postfix: ${CMAKE_DEBUG_POSTFIX}")
message(STATUS "CMake C compiler: ${CMAKE_C_COMPILER_ID}")
message(STATUS "CMake C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake host system processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

# static link to make leak checking easier (_crtBreakAlloc)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# clang-cl provides __int128 arithmetic but does not implicitly link the
# compiler runtime with the 128-bit division helpers (__divti3 and friends)
if (MSVC AND CMAKE_C_COMPILER_ID MATCHES "Clang")
	if (CMAKE_GENERATOR_PLATFORM MATCHES "[Aa][Rr][Mm]64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "ARM64|aarch64")
		add_link_options(clang_rt.builtins-aarch64.lib)
	else()
		add_link_options(clang_rt.builtins-x86_64.lib)
	endif()
endif()

if (MSVC OR APPLE OR UNIX)
	option(BOX3D_SANITIZE "Enable sanitizers for some builds" OFF)
	set(BOX3D_SANITIZER_TYPE "address" CACHE STRING "Type of sanitizer to use (address, thread, undefined, memory)")
	set_property(CACHE BOX3D_SANITIZER_TYPE PROPERTY STRINGS "address" "thread" "undefined" "memory")
	
	if(BOX3D_SANITIZE)
		message(STATUS "Fixed3D Sanitize: ${BOX3D_SANITIZER_TYPE}")
		# sanitizers need to apply to all compiled libraries to work well
		if(MSVC)
			# address sanitizer only in the debug build
			add_compile_options("$<$<CONFIG:Debug>:/fsanitize=address>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL:NO>")
		elseif(APPLE)
			# more sanitizers on Apple clang
			# UBSan recovers by default, no-recover makes the first report fatal so CI fails
			if(BOX3D_SANITIZER_TYPE STREQUAL "thread")
				add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
				add_link_options(-fsanitize=thread)
			elseif(BOX3D_SANITIZER_TYPE STREQUAL "undefined")
				add_compile_options(-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
				add_link_options(-fsanitize=undefined)
			else()
				# default to address sanitizer
				add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined -fno-sanitize-recover=all)
				add_link_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
			endif()
		elseif(UNIX)
			# Linux/WSL2 sanitizer support
			if(BOX3D_SANITIZER_TYPE STREQUAL "thread")
				# Disable ASLR first to avoid crash on launch
				# echo 0 | sudo tee /proc/sys/kernel/randomize_va_space
				add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -fPIE)
				add_link_options(-fsanitize=thread -pie)
			elseif(BOX3D_SANITIZER_TYPE STREQUAL "undefined")
				add_compile_options(-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
				add_link_options(-fsanitize=undefined)
			elseif(BOX3D_SANITIZER_TYPE STREQUAL "memory")
				add_compile_options(-fsanitize=memory -fno-omit-frame-pointer)
				add_link_options(-fsanitize=memory)
			else()
				# default to address sanitizer
				add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
				add_link_options(-fsanitize=address)
			endif()
		endif()
	else()
		if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND PROJECT_IS_TOP_LEVEL)
			# enable hot reloading
			add_compile_options("$<$<CONFIG:Debug>:/ZI>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL>")
		endif()
	endif()
endif()

# Deterministic math
# https://box2d.org/posts/2024/08/determinism/
if (MINGW OR APPLE OR UNIX)
	add_compile_options(-ffp-contract=off)
endif()

option(BOX3D_DISABLE_SIMD "Disable SIMD math (slower)" OFF)
option(BOX3D_AVX512 "Enable AVX-512 fixed-point SIMD in the contact solver (x86-64 with AVX512F/DQ/VL, bit-identical results)" OFF)
option(BOX3D_NEON "Enable NEON fixed-point SIMD in the narrow phase (aarch64, bit-identical results)" OFF)
option(BOX3D_COMPILE_WARNING_AS_ERROR "Compile warnings as errors" OFF)
option(BOX3D_DOUBLE_PRECISION "Enable double precision for large worlds" OFF)
# BOX3D_LUDICROUS_MODE: 128-bit world positions (Q112.16) AND 128-bit broadphase AABBs,
# so simulation and collision both work across +/-2.6e33 units — far past a light-year —
# at the same 1/65536 resolution as the origin. The interior solver stays Q48.16.
# Costs +1.6% geomean. Nobody should ever turn this on; it exists to answer exactly one
# question: how much slower is it? See the README's LUDICROUS MODE section.
option(BOX3D_LUDICROUS_MODE "Enable 128-bit positions and broadphase (you don't need this)" OFF)

# These are read by the library build, so they must precede add_subdirectory(src),
# otherwise the first configure and a re-configure produce different libraries.
# Top level only, so they stay hidden from FetchContent consumers.
if(PROJECT_IS_TOP_LEVEL)
	option(BOX3D_PROFILE "Enable profiling with Tracy" OFF)
	option(BOX3D_VALIDATE "Enable heavy validation" ON)
endif()

# Emscripten maps the SSE2 path onto wasm SIMD128. simd.h is a header included by
# non-library targets like the tests, so these flags must be directory-wide, not
# PRIVATE to the box3d library.
if (EMSCRIPTEN AND NOT BOX3D_DISABLE_SIMD)
	add_compile_options(-msimd128 -msse2)
endif()

# Emscripten pthread support. Must precede add_subdirectory(src) so the box3d library
# objects are built with the same atomics/bulk-memory features as the executables that
# link them, otherwise wasm-ld rejects --shared-memory. Top level only, so a FetchContent
# consumer keeps control of its own threading model.
if (EMSCRIPTEN AND PROJECT_IS_TOP_LEVEL)
	set(EMSCRIPTEN_PTHREADS_COMPILER_FLAGS "-pthread -s USE_PTHREADS=1")
	set(EMSCRIPTEN_PTHREADS_LINKER_FLAGS "${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS} -s ALLOW_MEMORY_GROWTH")
	string(APPEND CMAKE_C_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
	string(APPEND CMAKE_CXX_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
	string(APPEND CMAKE_EXE_LINKER_FLAGS " ${EMSCRIPTEN_PTHREADS_LINKER_FLAGS}")
endif()

# Link-time optimization: measured +1-3% on the fixed-point benchmarks (Zen 4,
# 2026-07-12). Default ON for optimized top-level builds where the toolchain
# supports it. Debug builds skip it (per-config properties below) to keep
# compile/debug cycles fast, sanitized builds skip it to keep instrumentation
# predictable, and FetchContent consumers keep control of their own IPO policy.
if(PROJECT_IS_TOP_LEVEL)
	option(BOX3D_LTO "Enable link-time optimization for optimized builds" ON)
	if(BOX3D_LTO AND NOT BOX3D_SANITIZE)
		include(CheckIPOSupported)
		check_ipo_supported(RESULT BOX3D_IPO_SUPPORTED OUTPUT BOX3D_IPO_ERROR)
		if(BOX3D_IPO_SUPPORTED)
			set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
			set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
			set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL ON)
		else()
			message(STATUS "Fixed3D LTO unsupported on this toolchain: ${BOX3D_IPO_ERROR}")
		endif()
	endif()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Needed for samples.exe to find box2d.dll
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

# Collect FetchContent downloads outside the build dir so a clean rebuild reuses them.
# Must precede the first include(FetchContent)
if(NOT FETCHCONTENT_BASE_DIR)
	set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.fetchcontent-cache"
		CACHE PATH "Shared FetchContent populate cache")
endif()

include(FetchContent)

add_subdirectory(src)

# This hides samples, test, and doxygen from apps that use box3d via FetchContent
if(PROJECT_IS_TOP_LEVEL)
	option(BOX3D_SAMPLES "Build the Fixed3D samples" ON)
	option(BOX3D_BENCHMARKS "Build the Fixed3D benchmarks" OFF)
	option(BOX3D_DOCS "Build the Fixed3D documentation" OFF)
	option(BOX3D_UNIT_TESTS "Build the Fixed3D unit tests" ON)

	# Enable this for shader development using sokol-shdc.
	# This can be a large download, so this is disabled by default.
	option(BOX3D_BUILD_SHADERS "Recompile shaders with sokol-shdc" OFF)

	if(BOX3D_UNIT_TESTS OR BOX3D_SAMPLES OR BOX3D_BENCHMARKS)
		add_subdirectory(shared)
	endif()

	# Tests need static linkage because they test internal Box3D functions
	if(NOT BUILD_SHARED_LIBS AND BOX3D_UNIT_TESTS)
		message(STATUS "Adding Fixed3D unit tests")
		add_subdirectory(test)
		set_target_properties(test PROPERTIES XCODE_GENERATE_SCHEME TRUE)
	else()
		message(STATUS "Skipping Fixed3D unit tests")
	endif()

	if(BOX3D_SAMPLES)
		add_subdirectory(samples)

		# default startup project for Visual Studio
		if(MSVC)
			set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
			set_property(TARGET samples PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
		endif()

			set_target_properties(samples PROPERTIES
				XCODE_GENERATE_SCHEME TRUE
				XCODE_SCHEME_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
	endif()

	if(BOX3D_BENCHMARKS)
		add_subdirectory(benchmark)
		set_target_properties(benchmark PROPERTIES XCODE_GENERATE_SCHEME TRUE)
	endif()

	if(BOX3D_DOCS)
		add_subdirectory(docs)
	endif()
endif()

# https://stackoverflow.com/questions/72635402/how-to-set-working-directory-in-cmake-visual-studio-2022
# launch.vs.json
# "currentDir": "${workspaceRoot}"