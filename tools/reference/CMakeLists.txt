# The float box3d reference harness. OFF by default: a default configure builds
# exactly as before. When BOX3D_REFERENCE=ON, this fetches vanilla box3d at
# BOX3D_REFERENCE_REV (ExternalProject, cached under BOX3D_REFERENCE_CACHE), builds
# the fixed and float drivers, the comparer, and registers one `ctest -L reference`
# entry per scene file found in scenes/.

option(BOX3D_REFERENCE "Build the float box3d reference harness (fetches vanilla box3d)" OFF)

# Registering tests needs CTest, and the top level does not enable it (the unit
# suite is run directly as ./bin/test). Enable it only when the harness is on.
if(BOX3D_REFERENCE)
	enable_testing()
endif()

if(NOT BOX3D_REFERENCE)
	return()
endif()

include(ExternalProject)
include(GNUInstallDirs)

# The vanilla box3d commit every scene is measured against. The pin and the cache
# key are the same sha, so switching the pin fetches and builds a fresh reference.
set(BOX3D_REFERENCE_REV "" CACHE STRING "erincatto/box3d commit to compare against (e.g. 47d7f7c)")
if(BOX3D_REFERENCE_REV STREQUAL "")
	message(FATAL_ERROR "BOX3D_REFERENCE=ON requires -DBOX3D_REFERENCE_REV=<sha> naming a vanilla box3d commit")
endif()

if(NOT DEFINED BOX3D_REFERENCE_CACHE)
	set(BOX3D_REFERENCE_CACHE "$ENV{HOME}/.cache/fixed3d-reference/${BOX3D_REFERENCE_REV}"
		CACHE PATH "Cache directory for the pinned vanilla box3d build")
endif()

set(BOX3D_REFERENCE_INSTALL "${BOX3D_REFERENCE_CACHE}/install")
set(BOX3D_REFERENCE_LIB "${BOX3D_REFERENCE_INSTALL}/${CMAKE_INSTALL_LIBDIR}/libbox3d.a")

# Fetch and build vanilla box3d once, reuse on every later configure/build.
ExternalProject_Add(box3d_reference
	GIT_REPOSITORY https://github.com/erincatto/box3d.git
	GIT_TAG ${BOX3D_REFERENCE_REV}
	PREFIX ${BOX3D_REFERENCE_CACHE}
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${BOX3D_REFERENCE_INSTALL}
		-DCMAKE_BUILD_TYPE=RelWithDebInfo
		-DBOX3D_SAMPLES=OFF
		-DBOX3D_UNIT_TESTS=OFF
		-DBOX3D_BENCHMARKS=OFF
		-DBOX3D_VALIDATE=OFF
	BUILD_BYPRODUCTS ${BOX3D_REFERENCE_LIB}
	UPDATE_COMMAND ""
)

add_library(box3d_reference_lib STATIC IMPORTED GLOBAL)
add_dependencies(box3d_reference_lib box3d_reference)
set_target_properties(box3d_reference_lib PROPERTIES
	IMPORTED_LOCATION ${BOX3D_REFERENCE_LIB}
)

set(BOX3D_REFERENCE_COMPILE_OPTS)
if(NOT MSVC)
	set(BOX3D_REFERENCE_COMPILE_OPTS -std=c17 -Wall -Wextra)
endif()

# Comparer: engine-free, only the shared parser.
add_executable(ref_compare ref_compare.c ref_scene.c)
target_include_directories(ref_compare PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(ref_compare PRIVATE ${BOX3D_REFERENCE_COMPILE_OPTS})

# Fixed driver: links the in-tree box3d.
add_executable(run_fixed run_fixed.c ref_scene.c)
target_include_directories(run_fixed PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(run_fixed PRIVATE box3d)
target_compile_options(run_fixed PRIVATE ${BOX3D_REFERENCE_COMPILE_OPTS})

# Float driver: links the fetched vanilla box3d at the pinned commit.
add_executable(run_float run_float.c ref_scene.c)
target_include_directories(run_float PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${BOX3D_REFERENCE_INSTALL}/include)
target_link_libraries(run_float PRIVATE box3d_reference_lib m)
add_dependencies(run_float box3d_reference_lib)
target_compile_options(run_float PRIVATE ${BOX3D_REFERENCE_COMPILE_OPTS})

# ref_add_scene(name): register `ctest -L reference` for one scene. The runner
# script locates the built drivers through REFERENCE_BIN_DIR, which the test
# property pins to this build directory.
function(ref_add_scene name)
	add_test(NAME reference_${name}
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/run_reference.sh
			${BOX3D_REFERENCE_REV}
			${CMAKE_CURRENT_SOURCE_DIR}/scenes/${name}.scene)
	set_tests_properties(reference_${name} PROPERTIES
		LABELS reference
		ENVIRONMENT "REFERENCE_BIN_DIR=$<TARGET_FILE_DIR:ref_compare>")
endfunction()

file(GLOB BOX3D_REFERENCE_SCENES ${CMAKE_CURRENT_SOURCE_DIR}/scenes/*.scene)
foreach(scene_file ${BOX3D_REFERENCE_SCENES})
	get_filename_component(scene_name ${scene_file} NAME_WE)
	ref_add_scene(${scene_name})
endforeach()
