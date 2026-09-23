# GoogleTest.
#
# Prefer an installed copy; fall back to fetching a pinned release. The version
# is pinned rather than tracking a branch so that a test failure is always
# attributable to our change, never to a silently updated dependency.
find_package(GTest QUIET)

if(NOT GTest_FOUND)
    message(STATUS "GoogleTest not found locally; fetching v1.15.2")
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.15.2
        GIT_SHALLOW    TRUE
    )
    # Do not let GoogleTest's own options leak into our cache.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

add_executable(geometry_tests
    test_vec3.cpp
    test_mat4.cpp
    test_ray.cpp
    test_plane.cpp
    test_aabb.cpp
    test_aabb_property.cpp
    test_triangle.cpp
    test_mesh.cpp
    test_preconditions.cpp
    test_intersect.cpp
    test_mesh_query.cpp
    test_obj_loader.cpp
    test_procedural.cpp
    test_bvh.cpp
    test_bvh_visualization.cpp
)

target_link_libraries(geometry_tests PRIVATE
    bvh::geometry
    bvh::bvh
    bvh::rendering
    bvh::scene
    GTest::gtest
    GTest::gtest_main
)

# GoogleTest's own headers do not survive our stricter warning set, and warnings
# from a dependency are noise. Our code is still compiled with the full set via
# bvh_project_options, which bvh::geometry propagates.
target_compile_options(geometry_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wno-old-style-cast>)

include(GoogleTest)
# Register each TEST() individually so a failure names the case, and so ctest
# can run them in parallel.
gtest_discover_tests(geometry_tests
    PROPERTIES LABELS "unit"
    DISCOVERY_TIMEOUT 30)
