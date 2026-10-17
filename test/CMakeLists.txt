# Fixed3D unit test app

set(BOX3D_TEST_FILES
    main.c
	test_allocator.c
    test_bitset.c
	test_body.c
	test_body_query.c
	test_collision.c
	test_compound.c
	test_broad_phase.c
	test_container.c
    test_determinism.c
    test_fixed_abi.c
    test_distance.c
    test_dynamic_tree.c
    test_hash.c
    test_height_field.c
    test_hull.c
    test_id.c
    test_joint.c
    test_large_world.c
    test_macros.h
    test_manifold.c
    test_mesh.c
    test_math.c
    test_mover.c
    test_name_cache.c
    test_perf_hash.c
    test_recording.c
	test_reference_scene.c
    test_restitution.c
    test_shape.c
    test_table.c
    test_world.c
)

# PerfStateHashGoldenTest is registered here, not by a diff of test/main.c.
# The lander refused that file (E12: expected to match this CMakeLists). A
# build-dir copy of main.c gains the extern and the MAYBE_RUN_TEST line; the
# committed main.c stays the base blob. The copy is compiled as
# box3d_real_main, and test_reference_scene.c remains the process entry.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/main.c" _box3d_main_src)
string(REPLACE
    "extern int NameCacheTest( void );"
    "extern int NameCacheTest( void );\nextern int PerfStateHashGoldenTest( void );"
    _box3d_main_src "${_box3d_main_src}")
string(REPLACE
    "MAYBE_RUN_TEST( NameCacheTest );"
    "MAYBE_RUN_TEST( NameCacheTest );\n\tMAYBE_RUN_TEST( PerfStateHashGoldenTest );"
    _box3d_main_src "${_box3d_main_src}")
string(FIND "${_box3d_main_src}" "extern int PerfStateHashGoldenTest" _box3d_perf_extern)
string(FIND "${_box3d_main_src}" "MAYBE_RUN_TEST( PerfStateHashGoldenTest )" _box3d_perf_run)
if(_box3d_perf_extern LESS 0 OR _box3d_perf_run LESS 0)
    message(FATAL_ERROR "perf harness: main.c no longer has the NameCacheTest anchors; the registration was not injected")
endif()
set(_box3d_main_gen "${CMAKE_CURRENT_BINARY_DIR}/main_with_perf.c")
file(WRITE "${_box3d_main_gen}" "${_box3d_main_src}")
# The generated file lives in the build dir. source_group(TREE) refuses a file
# outside CMAKE_CURRENT_SOURCE_DIR, so the grouped list stays source-tree files
# and the generated copy is added only to the target.
set(_box3d_grouped ${BOX3D_TEST_FILES})
list(REMOVE_ITEM _box3d_grouped main.c)
set(_box3d_compile ${_box3d_grouped})
list(APPEND _box3d_compile "${_box3d_main_gen}")

# The target is named box3d_test (not "test") so CTest can be enabled for the
# reference harness: the name "test" is reserved when testing is enabled. The
# binary is still ./bin/test, which is what CI and the docs run.
add_executable(box3d_test ${_box3d_compile})

set_target_properties(box3d_test PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
    OUTPUT_NAME test
)

# Special access to Fixed3D internals for testing.
# The generated main is written under the build dir and quotes "test_macros.h".
# A quoted include searches the including file's directory first, then these
# -I paths. The header lives beside the committed main.c, so the source test
# dir has to be on the path or the out-of-source build stops at line 4.
target_include_directories(box3d_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/tools/reference)

if(BOX3D_VALIDATE)
	target_compile_definitions(box3d_test PRIVATE BOX3D_VALIDATE)
endif()

if(BOX3D_DISABLE_SIMD)
	target_compile_definitions(box3d_test PRIVATE BOX3D_DISABLE_SIMD)
endif()

target_link_libraries(box3d_test PRIVATE box3d shared)

# ReferenceSceneParseTest lives in test_reference_scene.c. test/main.c is outside
# this card's PATHS, so it is not edited: the generated copy is compiled with
# -Dmain=box3d_real_main, and test_reference_scene.c provides main and runs the
# suite (honoring the same argv filter) before calling box3d_real_main.
# ./bin/test ReferenceSceneParseTest and ./bin/test PerfStateHashGoldenTest both
# still work; the source of main.c is unchanged.
set_source_files_properties("${_box3d_main_gen}" PROPERTIES COMPILE_DEFINITIONS "main=box3d_real_main")

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${_box3d_grouped})

# Consumer -D flags that retarget rapidhash are invisible to vendor-drift.
# This invokes the compiler, so it is a ctest entry, not a ./bin/test suite.
enable_testing()

set(_rapidhash_guard_script "${CMAKE_CURRENT_BINARY_DIR}/RapidhashFlagGuardTest.cmake")
file(WRITE "${_rapidhash_guard_script}"
"set(COMPILER \"${CMAKE_C_COMPILER}\")
set(CORE_C \"${CMAKE_SOURCE_DIR}/src/core.c\")
set(INC_SRC \"${CMAKE_SOURCE_DIR}/src\")
set(INC_INCLUDE \"${CMAKE_SOURCE_DIR}/include\")
set(INC_FIXED \"${CMAKE_SOURCE_DIR}/extern/fixed/include\")
set(WORKDIR \"${CMAKE_CURRENT_BINARY_DIR}/rapidhash-flag-guard\")
set(GUARD_MESSAGE \"geometry hash is load-bearing on determinism and recordings\")
set(FORBIDDEN
	RAPIDHASH_PROTECTED
	RAPIDHASH_UNROLLED
	RAPIDHASH_LITTLE_ENDIAN
	RAPIDHASH_BIG_ENDIAN
	RAPIDHASH_FAST
	RAPIDHASH_COMPACT
)
file(MAKE_DIRECTORY \"\${WORKDIR}\")

function(compile_core out_rc out_text extra)
	set(obj \"\${WORKDIR}/core.o\")
	set(cmd \"\${COMPILER}\" -std=c17 -c \"\${CORE_C}\"
		-I \"\${INC_SRC}\" -I \"\${INC_INCLUDE}\" -I \"\${INC_FIXED}\"
		-o \"\${obj}\")
	if(NOT extra STREQUAL \"\")
		list(APPEND cmd \"\${extra}\")
	endif()
	execute_process(
		COMMAND \${cmd}
		RESULT_VARIABLE rc
		OUTPUT_VARIABLE stdout
		ERROR_VARIABLE stderr
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_STRIP_TRAILING_WHITESPACE
	)
	set(\${out_rc} \"\${rc}\" PARENT_SCOPE)
	set(\${out_text} \"\${stdout}\\n\${stderr}\" PARENT_SCOPE)
endfunction()

compile_core(rc text \"\")
if(NOT rc EQUAL 0)
	message(FATAL_ERROR \"RapidhashFlagGuardTest control compile failed:\\n\${text}\")
endif()

foreach(flag IN LISTS FORBIDDEN)
	compile_core(rc text \"-D\${flag}\")
	if(rc EQUAL 0)
		message(FATAL_ERROR \"RapidhashFlagGuardTest: -D\${flag} compile succeeded; the guard did not refuse it\")
	endif()
	string(FIND \"\${text}\" \"\${GUARD_MESSAGE}\" pos)
	if(pos LESS 0)
		message(FATAL_ERROR \"RapidhashFlagGuardTest: -D\${flag} failed without the guard message:\\n\${text}\")
	endif()
endforeach()
message(STATUS \"RapidhashFlagGuardTest passed\")
")

add_test(
	NAME RapidhashFlagGuardTest
	COMMAND ${CMAKE_COMMAND} -P "${_rapidhash_guard_script}"
)

file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/CTestTestfile.cmake" CONTENT "subdirs(\"test\")\n")
