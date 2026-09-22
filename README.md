# BVH Explorer

Interactive C++20/Vulkan engine for constructing, visualising, benchmarking and tuning
Bounding Volume Hierarchies for spatial queries.

## Building

Requires CMake 3.20+ and a C++20 compiler. GoogleTest is fetched automatically on the
first configure if it is not already installed.

```bash
# Release -- what ships, and what benchmarks must run under
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
cd build && ctest --output-on-failure

# Debug -- required too: -DNDEBUG strips every assert, so a Release-only run
# never exercises a precondition
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug -j4
cd build-debug && ctest --output-on-failure
```

Options: `BVH_BUILD_TESTS` (ON), `BVH_ENABLE_SANITIZERS` (OFF), `BVH_WARNINGS_AS_ERRORS`
(OFF), `BVH_CONSERVATIVE_RAY_BOX` (ON).

### Toolchain caveat

The project targets C++20, but on a toolchain without a C++20 standard library CMake
quietly falls back to the draft `-std=c++2a` and still reports "20". Configure probes for
this and prints a warning when it happens. The current development machine (Apple clang
11) is in exactly that state: everything builds and all tests pass, but this is not yet a
real C++20 build. See the Platform section of `CLAUDE.md`.

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
