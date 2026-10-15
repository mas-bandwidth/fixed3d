# Fixed3D unit test app

set(BOX3D_TEST_FILES
    main.c
	test_allocator.c
    test_bitset.c
	test_body.c
	test_body_query.c
	test_collision.c
	test_compound.c
	test_container.c
    test_determinism.c
    test_fixed_abi.c
    test_distance.c
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
    test_recording.c
	test_reference_scene.c
    test_shape.c
    test_table.c
    test_world.c
)

# The target is named box3d_test (not "test") so CTest can be enabled for the
# reference harness: the name "test" is reserved when testing is enabled. The
# binary is still ./bin/test, which is what CI and the docs run.
add_executable(box3d_test ${BOX3D_TEST_FILES})

set_target_properties(box3d_test PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
    OUTPUT_NAME test
)

# Special access to Fixed3D internals for testing
target_include_directories(box3d_test PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tools/reference)

if(BOX3D_VALIDATE)
	target_compile_definitions(box3d_test PRIVATE BOX3D_VALIDATE)
endif()

if(BOX3D_DISABLE_SIMD)
	target_compile_definitions(box3d_test PRIVATE BOX3D_DISABLE_SIMD)
endif()

target_link_libraries(box3d_test PRIVATE box3d shared)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${BOX3D_TEST_FILES})

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
