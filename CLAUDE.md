# BVH Explorer

Interactive C++20/Vulkan engine for constructing, visualizing, benchmarking, and tuning
Bounding Volume Hierarchies. The authoritative spec is `BVH_Explorer_Project_Plan.md` —
read it before implementing anything.

This project doubles as interview preparation. Every algorithm, tradeoff, and benchmark
must be explainable by the developer, which is why simple and defensible beats clever.

---

## Your role in the main session

You are the **Implementation Engineer**. You own the build: C++20 implementation, CMake,
geometry primitives, mesh loading, BVH construction and traversal, configurable BVH
parameters, ray and spatial queries, collision/clearance queries, Vulkan rendering and
compute, benchmarking, tests, and code organization.

Development methodology:

1. Read the project plan before implementing anything.
2. Break work into small, independently testable milestones.
3. Implement one milestone at a time.
4. Compile and run tests after every meaningful change.
5. Do not move forward when the current implementation is broken.
6. Prefer simple, understandable implementations over unnecessary abstraction.
7. Keep geometry and algorithmic logic separate from Vulkan rendering code.
8. Add tests for important mathematical and geometric operations.
9. Benchmark performance rather than making unsupported performance claims.
10. Preserve existing working functionality when adding new features.

Code quality bar: clear ownership and data flow, RAII for all resources, no unnecessary
global state, explicit error handling, sensible const-correctness, no unnecessary copies,
deliberate memory layout, no premature optimization, measurable hot paths, and comments on
non-obvious algorithms.

Implement so you can explain: why the algorithm was chosen, its complexity, memory
tradeoffs, numerical/geometric assumptions, performance bottlenecks, alternative
approaches, why Vulkan is used where it is, and the CPU vs GPU tradeoff.

---

## Project-wide rules

These bind every agent and every session.

**Never fabricate benchmark numbers.** A performance claim is valid only if it traces to a
recorded run under `benchmarks/results/`. If you have not measured it, say so — write
`UNVERIFIED` rather than an estimate. This rule is absolute and applies to documentation,
commit messages, and conversation alike.

**No external libraries for core algorithms.** Geometry math, BVH construction, traversal,
intersection, and collision queries are implemented by us. Infrastructure dependencies the
plan already sanctions are fine: GLFW, ImGui, GoogleTest, CMake. GLM is permitted only for
rendering-side convenience — the geometry core uses our own `Vec3`/`Mat4` so the math is
ours to explain. Anything beyond this list needs explicit approval.

**Layering.** `geometry/`, `bvh/`, and `collision/` must not include or depend on Vulkan,
GLFW, or ImGui headers. Rendering depends on geometry, never the reverse. This keeps the
algorithmic core unit-testable without a GPU and is the single most important structural
rule in the project.

**Don't claim done without running it.** Report build and test status from actual output.
If something fails or was skipped, say so plainly and show the output.

---

## Repository layout

```
include/{geometry,bvh,collision,rendering,scene,benchmark}/
src/{geometry,bvh,collision,rendering,scene,benchmark}/
shaders/{graphics,compute}/
tests/         GoogleTest unit + integration tests
benchmarks/    benchmark drivers
  results/     committed measurement output (see benchmarks/results/README.md)
assets/        meshes
docs/          subsystem docs + interview knowledge base
```

## Build and test

```bash
# Configure (Release is the default when CMAKE_BUILD_TYPE is unset)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -j4

# Test
cd build && ctest --output-on-failure -j4
#   or run the binary directly: ./build/tests/geometry_tests

# Benchmark: not yet implemented (Phase 9)
```

**Both configurations must pass before a milestone is done.** Release is what ships and
what benchmarks run under, but `-DNDEBUG` strips every `assert`, so a Release-only test
run never exercises a single precondition:

```bash
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug -j4
cd build-debug && ctest --output-on-failure -j4
```

The error-handling policy this validates: the `Mesh` constructor **throws**, because that
is the trust boundary where untrusted file data enters. Everywhere else a violated
precondition is a programming error and **asserts**. That policy is only real if the
asserting build is actually run.

Useful options:

| Option | Default | Effect |
|---|---|---|
| `BVH_BUILD_TESTS` | ON | Build the GoogleTest suite |
| `BVH_ENABLE_SANITIZERS` | OFF | `-fsanitize=address,undefined` |
| `BVH_WARNINGS_AS_ERRORS` | OFF | `-Werror`; the tree is currently clean under it |
| `BVH_CONSERVATIVE_RAY_BOX` | ON | Widen the ray/AABB exit distance, and the BVH's box-vs-triangle prune threshold, for watertightness. Off measures the exact test against it (Phase 9) |

GoogleTest is found via `find_package` and otherwise fetched (pinned to v1.15.2), so the
first configure needs network access.

`-ffast-math` must never be enabled. The intersection routines depend on IEEE-754 NaN and
infinity behaviour; fast-math lets the compiler assume those never occur, which silently
turns the NaN-tolerant slab test into a source of wrong answers.

## Platform

Development is on macOS (Darwin 24.6) on an **Intel** Core i5-8257U (2019 MacBook Pro,
4 cores / 8 threads, 1.4 GHz base). Two consequences that shape the whole project:

- **Integrated Iris Plus graphics.** GPU compute headroom is modest, so the Phase 8
  CPU-vs-GPU comparison will not show the speedups a discrete GPU would. Report what the
  hardware actually does; a modest or negative GPU speedup is a legitimate result and a
  better interview answer than a fabricated one.
- **4 physical cores** bounds the multithreaded BVH build comparison. Expect well under
  4x, and attribute the gap (memory bandwidth, Amdahl, scheduling) from measurement.

### Toolchain status — action needed

The installed Command Line Tools are **Apple clang 11.0.3 (2020)**, far older than the OS.
Consequences, in order of how soon they bite:

- **Now:** there is no C++20 standard library (`<concepts>`, `<span>`, `<ranges>`,
  `<numbers>` are all missing), and `-std=c++20` is not even a recognised flag. CMake
  silently falls back to the draft `-std=c++2a` and still reports "20"; the configure step
  probes for this and prints a warning. The geometry core builds and all tests pass this
  way, so Phase 1 is not blocked — but the project cannot honestly claim C++20 yet.
- **Soon:** Homebrew refuses to install anything needing compilation on this
  configuration (`Error: Your Command Line Tools are too outdated`), which already blocked
  `brew install llvm`. GLFW, the Vulkan SDK, and ImGui will hit the same wall.

Fix (needs a GUI installer and sudo, so it has to be run by hand):

```bash
# Try Software Update in System Settings first; otherwise:
sudo rm -rf /Library/Developer/CommandLineTools
sudo xcode-select --install
```

There is no native Vulkan driver — Vulkan runs through **MoltenVK** from the LunarG
Vulkan SDK. Practical consequences:

- Enable `VK_KHR_portability_enumeration` on the instance and
  `VK_KHR_portability_subset` on the device, or device creation fails.
- MoltenVK is a Vulkan-on-Metal translation layer, so some features are unavailable or
  slower than native. Verify against the portability subset before relying on a feature.
- GPU compute timings reflect Metal via translation. Note that in any benchmark writeup;
  do not present them as native Vulkan numbers.

---

## Agent roster and workflow

The main session builds. Two subagents support it — see `.claude/agents/`.

| Agent | Role | Writes |
|---|---|---|
| `geometry-reviewer` | Technical gatekeeper: correctness, then architecture, then performance, then style. Returns APPROVED or CHANGES REQUIRED. | nothing (read-only) |
| `interview-docs-engineer` | Subsystem docs and the interview knowledge base, kept in sync with the source. | `docs/` only |
| `implementation-engineer` | Same role as the main session, for isolated or parallel side tasks. | code |

After each milestone: commit, then spawn the reviewer and the docs engineer **in parallel**
— they are independent. Hand the reviewer a milestone summary (what was implemented, files
changed, design decisions, algorithms, tests run, benchmark results, known limitations,
open questions). On CHANGES REQUIRED, fix, then continue the *same* reviewer via
`SendMessage` so it re-reviews against its original findings rather than starting cold.

Evaluate review findings technically. Do not blindly accept a suggestion — explain
disagreements.

## Git

One commit per milestone, with the work building and tests passing at each commit. The
reviewer and docs engineer both read `git diff`, so milestone-sized commits directly
determine how useful their review is.
