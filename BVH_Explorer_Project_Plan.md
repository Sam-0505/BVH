# BVH Explorer — Interactive Spatial Acceleration & Geometry Engine

## Goal

Build a C++/Vulkan interactive system for constructing, visualizing, benchmarking, and tuning Bounding Volume Hierarchies (BVHs) for high-performance spatial queries.

The project should demonstrate:

- C++ systems programming
- Computational geometry
- Spatial data structures
- Collision and ray-intersection queries
- Algorithmic tradeoffs
- Performance engineering
- Vulkan rendering and GPU compute
- Interactive visualization and benchmarking

The project should feel like a small production-oriented geometry tool rather than a graphics demo.

---

## Core Architecture

```text
                    3D Mesh / Scene
                           |
                           v
                +----------------------+
                | Geometry Core (C++)  |
                | Vec3 / Mat4 / Ray    |
                | Triangle / AABB / Mesh|
                +----------+-----------+
                           |
                           v
                +----------------------+
                |    BVH Builder        |
                | Median / Centroid /   |
                | SAH splitting        |
                +----------+-----------+
                           |
             +-------------+-------------+
             |                           |
             v                           v
      Spatial Queries              Statistics
      Ray intersection            Node count
      Collision                   Tree depth
      Distance                    Leaf count
      Clearance                   Build time
             |                           |
             +-------------+-------------+
                           |
                           v
                +----------------------+
                | Vulkan Visualization |
                | Mesh / BVH / Rays    |
                | Collision / UI       |
                +----------+-----------+
                           |
                           v
                    Interactive Tool
```

---

## Technology Stack

- C++20
- Vulkan
- GLFW
- GLM or custom math implementation
- CMake
- ImGui
- Vulkan compute shaders
- GoogleTest
- OBJ mesh loading

Avoid relying on an external geometry/BVH library for the core implementation. The important algorithms should be implemented by us.

---

# Phase 1 — C++ Geometry Core

Implement the fundamental geometry types.

### Math

- `Vec3`
- `Vec4`
- `Mat4`
- dot product
- cross product
- normalization
- matrix transformations

### Geometry

- Ray
- Plane
- Triangle
- AABB
- Mesh

### Operations

- Ray transformation
- Point transformation
- AABB construction
- AABB union
- AABB surface area
- Triangle bounding box
- Mesh bounding box

### Tests

Create unit tests for:

- vector operations
- matrix transformations
- ray construction
- AABB intersection
- AABB union
- triangle bounds

---

# Phase 2 — BVH

Implement a Bounding Volume Hierarchy from scratch.

## BVH Node

Each node should contain:

```text
AABB bounds
left child
right child
primitive range / indices
```

Support:

- internal nodes
- leaf nodes
- configurable leaf size
- recursive construction
- tree traversal

---

# Phase 3 — Configurable BVH Parameters

The main feature of the application is that users can modify BVH construction parameters and immediately see the effect.

## Maximum Leaf Size

Expose:

```text
1 — 32 primitives
```

Measure how leaf size affects:

- number of nodes
- tree depth
- build time
- traversal time
- primitives tested

---

## Maximum Tree Depth

Expose:

```text
8 — 32
```

Show its effect on:

- tree depth
- node count
- leaf sizes
- query performance

---

## Split Strategy

Support:

```text
Median
Median by centroid
SAH
```

The UI should allow switching between strategies and rebuilding the BVH.

---

## SAH Bin Count

If using binned SAH:

```text
4
8
16
32
```

Measure:

- BVH build time
- query time
- node count
- primitive tests

---

# Phase 4 — BVH Visualization

The Vulkan viewer should allow users to inspect the hierarchy.

## Visualization Modes

### Mesh

Show the original 3D mesh.

### Bounding Volumes

Display AABBs at different hierarchy levels.

Allow the user to select:

```text
Depth 0
Depth 1
Depth 2
...
```

### Selected Node

When a node is selected, display:

```text
Node ID
Depth
Number of primitives
Surface area
Volume
Parent
Left child
Right child
```

### Tree View

Provide a hierarchy representation:

```text
Root
├── Node 1
│   ├── Node 3
│   └── Node 4
└── Node 2
    ├── Node 5
    └── Node 6
```

---

# Phase 5 — Interactive Ray Queries

Allow the user to generate or move rays through the scene.

Visualize:

- ray origin
- ray direction
- AABBs tested
- triangles tested
- final intersection
- hit/miss result

Display statistics:

```text
Nodes visited
AABBs tested
Triangles tested
Traversal time
Hit / miss
```

Compare against brute force:

```text
                 Brute Force      BVH

Triangles tested     N             M
Query time           T1            T2
Speedup              -             T1/T2
```

Use real measurements.

---

# Phase 6 — Collision Queries

Implement:

- AABB vs AABB collision
- mesh-level collision queries
- proximity queries
- clearance checks

Example:

```text
Object A
   |
   |  minimum clearance
   v
Object B
```

Allow users to configure the required clearance distance.

Display:

```text
Collision: YES / NO
Minimum distance
Required clearance
Constraint satisfied: YES / NO
```

---

# Phase 7 — Vulkan Rendering

Use Vulkan for:

- mesh rendering
- AABB rendering
- BVH visualization
- ray visualization
- collision visualization
- UI integration

The rendering system should support toggles such as:

```text
[ ] Show mesh
[ ] Show BVH
[ ] Show rays
[ ] Show collisions
[ ] Show selected node
```

---

# Phase 8 — GPU Compute

Use Vulkan compute shaders for highly parallel workloads.

Potential workloads:

- batch ray intersection
- batch AABB tests
- collision queries
- distance queries

Compare:

```text
CPU single-threaded
CPU multithreaded
GPU compute
```

Measure:

- total time
- throughput
- queries/second
- speedup

Do not move every algorithm to the GPU. Keep BVH construction and the core algorithmic logic understandable on the CPU initially.

---

# Phase 9 — Performance Benchmarking

Create a benchmark system.

For each configuration record:

```text
Mesh
Triangle count
Leaf size
Split strategy
SAH bins
Maximum depth

Build time
Node count
Leaf count
Tree depth
Average depth
Query time
Nodes visited
Triangles tested
Memory usage
```

Example table:

| Configuration | Build Time | Nodes | Depth | Query Time | Triangles Tested |
|---|---:|---:|---:|---:|---:|
| Leaf 2 | measured | measured | measured | measured | measured |
| Leaf 4 | measured | measured | measured | measured | measured |
| Leaf 8 | measured | measured | measured | measured | measured |
| Leaf 16 | measured | measured | measured | measured | measured |

Never fabricate benchmark values.

---

# Phase 10 — Interactive Experiments

The application should make algorithmic tradeoffs obvious.

## Experiment 1 — Leaf Size

Change:

```text
1 → 2 → 4 → 8 → 16 → 32
```

Observe:

- build time
- tree complexity
- query performance

## Experiment 2 — Split Strategy

Compare:

```text
Median
Centroid Median
SAH
```

using identical scenes and queries.

## Experiment 3 — SAH Bins

Compare:

```text
4 → 8 → 16 → 32
```

and analyze build/query tradeoffs.

## Experiment 4 — Brute Force vs BVH

Run identical rays against:

```text
Brute force
BVH
```

Measure the reduction in primitive tests.

---

# Optional Phase 11 — Path Planning

Only add this after the BVH system is polished.

Implement:

- occupancy representation
- A* pathfinding
- collision-aware path queries

Use the BVH for collision checks during path evaluation.

Demonstrate:

```text
Start
  |
  |      obstacle
  |     ███████
  |    /         |   /           +-----------------> Goal
```

---

# Optional Phase 12 — Constraint-Based Placement

Only add this after the core BVH project is complete.

Allow users to specify:

- minimum clearance
- workspace boundaries
- non-overlap constraints
- target distances

Then optimize object placement.

Example:

```text
Minimize:

    total connection distance

Subject to:

    no collisions
    minimum clearance >= threshold
    objects remain inside workspace
```

This connects the project to constraint-driven engineering layout.

---

# Suggested Repository Structure

```text
bvh-explorer/
|
├── CMakeLists.txt
├── README.md
|
├── include/
│   ├── geometry/
│   ├── bvh/
│   ├── collision/
│   ├── rendering/
│   ├── scene/
│   └── benchmark/
|
├── src/
│   ├── geometry/
│   ├── bvh/
│   ├── collision/
│   ├── rendering/
│   ├── scene/
│   └── benchmark/
|
├── shaders/
│   ├── graphics/
│   └── compute/
|
├── tests/
|
├── benchmarks/
|
├── assets/
|
└── docs/
```

---

# One-Day MVP Target

If using Codex heavily, the first milestone should be intentionally narrow.

## Must Have

- C++20 project
- CMake
- Vulkan initialization
- OBJ loading
- basic 3D viewer
- Vec3 / Ray / AABB / Triangle
- BVH construction
- BVH traversal
- ray/triangle intersection
- brute-force comparison
- BVH visualization
- configurable leaf size
- basic benchmark statistics

## Do NOT require on Day 1

- SAH
- GPU compute
- collision solver
- path planning
- constraint optimization
- sophisticated UI
- Vulkan ray tracing

Those should be extensions after the core works.

---

# Resume Positioning

Potential final project title:

**BVH Explorer — Interactive Spatial Acceleration & Geometry Engine**

Potential resume bullets after implementation:

- Built a C++20/Vulkan spatial geometry engine implementing configurable BVHs, mesh processing, ray-triangle intersection, and hierarchical spatial queries.
- Developed an interactive visualization and benchmarking system allowing users to tune leaf size and splitting strategies while analyzing tree structure, traversal cost, and primitive-test reduction.
- Implemented and benchmarked CPU/GPU spatial queries, analyzing build-time versus query-time tradeoffs across increasingly complex 3D scenes.

Only use the GPU bullet if GPU compute is actually implemented and benchmarked.

---

# Definition of Done

The project is strong enough for the target role when a reviewer can:

1. Load a 3D mesh.
2. Build a BVH.
3. Change BVH parameters.
4. Rebuild the tree.
5. See the hierarchy visually.
6. Shoot rays through the scene.
7. See which nodes/triangles are traversed.
8. Compare brute force against BVH.
9. See measured performance statistics.
10. Explain why different BVH configurations produce different performance.

The key objective is not maximum feature count.

The objective is to demonstrate:

**Geometry → Algorithms → Data Structures → Performance → Interactive Engineering Tool**
