#include "geometry/mesh_query.hpp"

namespace geom {

bool bruteForceClosestHit(const Mesh& mesh, const Ray& ray, MeshHit& hit, QueryStats* stats) {
    MeshHit best;
    const std::size_t count = mesh.triangleCount();

    // Shrinking tMax is the only pruning available without a spatial structure.
    // A BVH generalises it: there the same shrink skips whole subtrees.
    Scalar closest = ray.tMax;

    for (std::size_t i = 0; i < count; ++i) {
        TriangleHit th;
        if (intersectRayTriangle(mesh.triangle(i), ray.origin, ray.direction, ray.tMin, closest,
                                 th)) {
            closest = th.t;
            best.t = th.t;
            best.u = th.u;
            best.v = th.v;
            best.triangleIndex = i;
        }
    }

    if (stats != nullptr) stats->trianglesTested += count;

    if (!best.valid()) return false;
    hit = best;
    return true;
}

bool bruteForceAnyHit(const Mesh& mesh, const Ray& ray, QueryStats* stats) {
    const std::size_t count = mesh.triangleCount();
    for (std::size_t i = 0; i < count; ++i) {
        TriangleHit th;
        if (intersectRayTriangle(mesh.triangle(i), ray.origin, ray.direction, ray.tMin, ray.tMax,
                                 th)) {
            // Only what was actually examined.
            if (stats != nullptr) stats->trianglesTested += i + 1;
            return true;
        }
    }
    if (stats != nullptr) stats->trianglesTested += count;
    return false;
}

}  // namespace geom
