#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include "geometry/intersect.hpp"
#include "geometry/mesh.hpp"
#include "geometry/ray.hpp"
#include "geometry/scalar.hpp"

namespace geom {

// Result of a ray query against a mesh.
struct MeshHit {
    static constexpr std::size_t kNoTriangle = std::numeric_limits<std::size_t>::max();

    Scalar t{kInfinity};
    Scalar u{};
    Scalar v{};
    std::size_t triangleIndex{kNoTriangle};

    bool valid() const { return triangleIndex != kNoTriangle; }
};

// Shared by the reference and the accelerated path, so "the BVH tested M where
// brute force tested N" compares the same quantity. Counting is an increment in
// the inner loop; Phase 9 should price it before quoting timings.
struct QueryStats {
    std::uint64_t trianglesTested{0};
    // Popped and actually descended into. A node pushed and then skipped
    // because a closer hit arrived counts in nodesCulled instead -- their ratio
    // is what "how well did pruning work" means.
    std::uint64_t nodesExpanded{0};
    std::uint64_t nodesCulled{0};
    std::uint64_t aabbTests{0};

    void reset() { *this = QueryStats{}; }
};

// O(n) exhaustive search. This is the oracle every BVH is validated against and
// the baseline speedup is measured from, so it is deliberately the most obvious
// code that works -- no cleverness that could share a bug with what it checks.
bool bruteForceClosestHit(const Mesh& mesh, const Ray& ray, MeshHit& hit,
                          QueryStats* stats = nullptr);

// Stops at the first intersection. A separate entry point because once a BVH
// exists the two want different traversal orders: closest-hit goes near-to-far
// shrinking tMax, any-hit can stop anywhere.
bool bruteForceAnyHit(const Mesh& mesh, const Ray& ray, QueryStats* stats = nullptr);

}  // namespace geom
