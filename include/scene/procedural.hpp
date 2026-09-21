#pragma once

#include <cstddef>
#include <cstdint>

#include "geometry/mesh.hpp"
#include "geometry/scalar.hpp"

namespace scene {

// Meshes generated in code, for tests and benchmarks: there are no .obj assets
// here and a 12-triangle cube cannot exercise leaf size or depth. Triangle
// counts are targets -- ask the returned Mesh for the real count.

// Closed shell, vertices shared between bands. Curved and hollow, so most rays
// either miss it or cross two surfaces far apart.
geom::Mesh uvSphere(std::size_t targetTriangles, geom::Scalar radius = geom::Scalar(1));

// Flat tessellated square in the xz plane. Zero extent along y, which is the
// degenerate-axis case for any split that picks the longest axis.
geom::Mesh grid(std::size_t targetTriangles, geom::Scalar extent = geom::Scalar(1));

// Unconnected small triangles at random positions and orientations. No spatial
// coherence, so this is the adversarial case for a median split.
geom::Mesh triangleSoup(std::size_t triangleCount, std::uint32_t seed = 1u,
                        geom::Scalar extent = geom::Scalar(1),
                        geom::Scalar triangleSize = geom::Scalar(0.05));

}  // namespace scene
