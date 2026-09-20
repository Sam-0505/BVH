# BVH Explorer

Interactive C++20/Vulkan engine for constructing, visualising, benchmarking and tuning
Bounding Volume Hierarchies for spatial queries.

Full specification: [`BVH_Explorer_Project_Plan.md`](BVH_Explorer_Project_Plan.md).

## Status

| Phase | Scope | State |
|---|---|---|
| 1 | Geometry core — Vec3/Vec4/Mat4, Ray, Plane, Triangle, AABB, Mesh | Complete, 105 tests passing |
| 2 | BVH construction and traversal | Not started |
| 3+ | Configurable parameters, Vulkan visualisation, GPU compute, benchmarking | Not started |

## Building

Requires CMake 3.20+ and a C++20 compiler. GoogleTest is fetched automatically on the
first configure if it is not already installed.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
cd build && ctest --output-on-failure
```

Options: `BVH_BUILD_TESTS` (ON), `BVH_ENABLE_SANITIZERS` (OFF), `BVH_WARNINGS_AS_ERRORS`
(OFF).

## Layout

```
include/geometry/   geometry core headers
src/geometry/       geometry core implementation
tests/              GoogleTest unit tests
benchmarks/results/ recorded measurements (see that directory's README)
docs/               subsystem documentation
```

The geometry core has no dependency on Vulkan, GLFW or ImGui, so it is testable without a
GPU. That separation is enforced deliberately — see `CLAUDE.md`.
