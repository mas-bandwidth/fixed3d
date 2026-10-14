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
