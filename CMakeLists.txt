cmake_minimum_required(VERSION 3.20)

project(bvh_explorer
        VERSION 0.1.0
        DESCRIPTION "Interactive BVH construction, visualisation and benchmarking"
        LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Standard and global settings
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)          # -std=c++20, not -std=gnu++20

# Emit compile_commands.json for clangd / editor tooling.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release. Benchmark numbers from a Debug build are meaningless, and
# an unset CMAKE_BUILD_TYPE silently gives no optimisation at all.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
    message(STATUS "CMAKE_BUILD_TYPE not set; defaulting to Release")
endif()

# CMAKE_CXX_STANDARD_REQUIRED does NOT guarantee real C++20. For a compiler that
# only knows the pre-release draft flag, CMake silently emits -std=c++2a and
# reports "20" anyway -- a build that looks conformant but is missing most of the
# C++20 standard library. Probe for an actual C++20 library header so the gap is
# reported instead of hidden.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "${CMAKE_CXX20_STANDARD_COMPILE_OPTION}")
check_cxx_source_compiles("
    #include <concepts>
    #include <span>
    template <std::floating_point T> T twice(T v) { return v * 2; }
    int main() { return static_cast<int>(twice(0.0f)); }
" BVH_HAS_REAL_CXX20)
unset(CMAKE_REQUIRED_FLAGS)

if(NOT BVH_HAS_REAL_CXX20)
    message(WARNING
        "This toolchain does not provide the C++20 standard library "
        "(<concepts>, <span> are missing); CMake is falling back to a draft "
        "-std=c++2a. The geometry core builds, but C++20 library features are "
        "unavailable. Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}. "
        "See the Platform section of CLAUDE.md for how to fix this.")
endif()

option(BVH_BUILD_TESTS "Build the unit test suite" ON)
option(BVH_CONSERVATIVE_RAY_BOX
       "Widen the ray/AABB exit distance by (1 + 2*gamma(3)) for watertightness" ON)
option(BVH_ENABLE_SANITIZERS "Build with Address and UndefinedBehavior sanitizers" OFF)
option(BVH_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# ---------------------------------------------------------------------------
# Shared compile settings, carried as an INTERFACE target
# ---------------------------------------------------------------------------

add_library(bvh_project_options INTERFACE)

target_compile_options(bvh_project_options INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:
        -Wall -Wextra -Wpedantic
        -Wshadow                # a shadowed variable in geometry code is nearly always a bug
        -Wconversion            # silent float/double/int narrowing changes results
        -Wsign-conversion
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wdouble-promotion      # an accidental float->double promotion in a hot loop
        -Wformat=2
    >
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
)

if(BVH_WARNINGS_AS_ERRORS)
    target_compile_options(bvh_project_options INTERFACE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror>
        $<$<CXX_COMPILER_ID:MSVC>:/WX>)
endif()

# IMPORTANT: -ffast-math and friends are never enabled. The intersection
# routines depend on IEEE-754 NaN and infinity behaviour (see scalar.hpp);
# fast-math permits the compiler to assume those never occur, which turns the
# NaN-tolerant slab test into a source of wrong answers rather than a speedup.

# Kept on its own target, separate from warnings and sanitizers, because it is
# ODR-critical: intersectRay is inline, so two TUs compiled with different
# values are a silent violation. Every library links this PUBLIC, so the
# guarantee is local to each target rather than inherited from a neighbour.
add_library(bvh_config INTERFACE)
if(BVH_CONSERVATIVE_RAY_BOX)
    target_compile_definitions(bvh_config INTERFACE BVH_CONSERVATIVE_RAY_BOX=1)
else()
    target_compile_definitions(bvh_config INTERFACE BVH_CONSERVATIVE_RAY_BOX=0)
endif()

if(BVH_ENABLE_SANITIZERS)
    target_compile_options(bvh_project_options INTERFACE
        -fsanitize=address,undefined -fno-omit-frame-pointer -g)
    target_link_options(bvh_project_options INTERFACE
        -fsanitize=address,undefined)
endif()

# ---------------------------------------------------------------------------
# Geometry core
#
# Deliberately has NO dependency on Vulkan, GLFW or ImGui. The algorithmic core
# must be testable without a GPU or a window, which is what keeps the unit suite
# fast and runnable in CI.
# ---------------------------------------------------------------------------

add_library(bvh_geometry STATIC
    src/geometry/mat4.cpp
    src/geometry/mesh.cpp
    src/geometry/mesh_query.cpp
)

target_include_directories(bvh_geometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# PUBLIC so consumers get the warning settings too -- a mistake in a test is
# caught the same way as one in the library. BUILD_INTERFACE keeps the options
# out of any installed package config.
target_link_libraries(bvh_geometry PUBLIC bvh_config $<BUILD_INTERFACE:bvh_project_options>)

add_library(bvh::geometry ALIAS bvh_geometry)

# ---------------------------------------------------------------------------
# BVH
#
# Depends on the geometry core and nothing else -- no Vulkan, no windowing, no
# file I/O -- so construction and traversal stay unit-testable on their own.
# ---------------------------------------------------------------------------

add_library(bvh_bvh STATIC
    src/bvh/bvh.cpp
)

target_include_directories(bvh_bvh PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(bvh_bvh
    PUBLIC bvh_geometry bvh_config
    PRIVATE $<BUILD_INTERFACE:bvh_project_options>)

add_library(bvh::bvh ALIAS bvh_bvh)

# ---------------------------------------------------------------------------
# Scene I/O
#
# Separate from the geometry core: it touches the filesystem and untrusted
# input, so keeping it out means the core stays testable with no files on disk.
# ---------------------------------------------------------------------------

add_library(bvh_scene STATIC
    src/scene/obj_loader.cpp
    src/scene/procedural.cpp
)

target_include_directories(bvh_scene PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(bvh_scene
    PUBLIC bvh_geometry bvh_config
    PRIVATE $<BUILD_INTERFACE:bvh_project_options>)

add_library(bvh::scene ALIAS bvh_scene)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BVH_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

message(STATUS "")
message(STATUS "bvh_explorer configuration")
message(STATUS "  build type ......... ${CMAKE_BUILD_TYPE}")
message(STATUS "  compiler ........... ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
if(BVH_HAS_REAL_CXX20)
    message(STATUS "  C++ standard ....... ${CMAKE_CXX_STANDARD} (full library)")
else()
    message(STATUS "  C++ standard ....... ${CMAKE_CXX_STANDARD} (DRAFT -- no C++20 library)")
endif()
message(STATUS "  tests .............. ${BVH_BUILD_TESTS}")
message(STATUS "  sanitizers ......... ${BVH_ENABLE_SANITIZERS}")
message(STATUS "  conservative r/box . ${BVH_CONSERVATIVE_RAY_BOX}")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "  asserts ............ DISABLED (-DNDEBUG); use Debug to exercise preconditions")
else()
    message(STATUS "  asserts ............ enabled")
endif()
message(STATUS "")
