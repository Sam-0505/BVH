#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "geometry/aabb.hpp"
#include "geometry/mesh.hpp"
#include "geometry/mesh_query.hpp"
#include "geometry/ray.hpp"
#include "geometry/scalar.hpp"

namespace bvh {

enum class SplitStrategy {
    ObjectMedian,    // equal primitive counts either side
    CentroidMedian,  // split plane at the midpoint of the centroid bounds
    BinnedSAH,       // 1 + (A_l / A_p) N_l + (A_r / A_p) N_r; leaf cost is N
};

const char* toString(SplitStrategy s);

// Ranges the Phase 3/4 UI exposes. isValid() is deliberately wider, so
// degenerate configurations stay reachable from tests.
inline constexpr std::uint32_t kUiMinLeafSize = 1;
inline constexpr std::uint32_t kUiMaxLeafSize = 32;
inline constexpr std::uint32_t kUiMinDepth = 8;
inline constexpr std::uint32_t kUiMaxDepth = 32;

// Bounds the fixed traversal stack and the recursive build's frame count.
inline constexpr std::uint32_t kMaxDepthLimit = 64;

struct BuildConfig {
    std::uint32_t maxLeafSize{4};
    std::uint32_t maxDepth{32};
    SplitStrategy strategy{SplitStrategy::CentroidMedian};
    std::uint32_t sahBinCount{16};

    bool isValid(std::string* error = nullptr) const;
};

// 32 bytes, so a sibling pair shares one cache line: children are always
// allocated adjacently, which is also why only the left index is stored.
// `count == 0` marks an internal node -- unambiguous because the build never
// emits an empty leaf. No parent link; Phase 4 can derive one in an O(n) pass.
struct BVHNode {
    geom::AABB bounds{};
    std::uint32_t leftOrFirst{0};
    std::uint32_t count{0};

    bool isLeaf() const { return count != 0; }

    std::uint32_t leftChild() const {
        assert(!isLeaf() && "leftChild() on a leaf node");
        return leftOrFirst;
    }
    std::uint32_t rightChild() const {
        assert(!isLeaf() && "rightChild() on a leaf node");
        return leftOrFirst + 1;
    }
    std::uint32_t firstPrimitive() const {
        assert(isLeaf() && "firstPrimitive() on an internal node");
        return leftOrFirst;
    }
};

static_assert(sizeof(BVHNode) == 32, "BVHNode must stay at 32 bytes");
static_assert(alignof(BVHNode) == 4, "BVHNode is expected to be 4-byte aligned");

struct BuildStats {
    std::size_t nodeCount{0};
    std::size_t leafCount{0};
    std::size_t internalCount{0};
    std::size_t primitiveCount{0};
    std::uint32_t maxDepth{0};
    std::size_t maxLeafSize{0};
    double meanLeafDepth{0.0};
    double meanLeafSize{0.0};
    std::size_t memoryBytes{0};
    double buildTimeMs{0.0};
};

// Binary BVH over the triangles of a Mesh. The mesh is neither owned nor
// referenced: leaves index a permutation of triangle indices and every query
// takes the mesh back, which keeps lifetimes obvious and lines the signatures
// up with the brute-force oracle.
class BVH {
public:
    BVH() = default;

    // Throws std::invalid_argument if `config` is invalid. An empty mesh builds
    // an empty tree rather than failing.
    void build(const geom::Mesh& mesh, const BuildConfig& config);

    bool empty() const { return nodes_.empty(); }
    const std::vector<BVHNode>& nodes() const { return nodes_; }
    const std::vector<std::uint32_t>& primitiveIndices() const { return primitiveIndices_; }
    const BuildConfig& config() const { return config_; }
    const BuildStats& stats() const { return stats_; }
    std::size_t primitiveCount() const { return primitiveIndices_.size(); }

    // Empty when the tree is empty.
    geom::AABB bounds() const { return nodes_.empty() ? geom::AABB{} : nodes_[0].bounds; }

    // Same contract as bruteForceClosestHit / bruteForceAnyHit.
    // Precondition: `mesh` is the mesh this tree was built from, unmodified.
    bool closestHit(const geom::Mesh& mesh, const geom::Ray& ray, geom::MeshHit& hit,
                    geom::QueryStats* stats = nullptr) const;
    bool anyHit(const geom::Mesh& mesh, const geom::Ray& ray,
                geom::QueryStats* stats = nullptr) const;

    // Every structural invariant the build promises: bounds are the EXACT
    // union of what they cover (containment alone would accept a tree of
    // all-root boxes, which answers correctly and traverses like brute force),
    // leaf ranges partition the index array, every triangle is referenced
    // once, depth is within the limit, and the recorded stats agree with the
    // tree. An oversized Binned SAH leaf must be no more expensive than its
    // best binned split. O(n), for tests and debugging.
    bool validate(const geom::Mesh& mesh, std::string* error = nullptr) const;

private:
    // Lets the tests build a deliberately corrupt tree, which is the only way
    // to check that validate() ever returns false. Defined in the test TU.
    friend struct BVHTestAccess;

    std::vector<BVHNode> nodes_;
    std::vector<std::uint32_t> primitiveIndices_;
    std::uint64_t meshRevision_{0};
    BuildConfig config_{};
    BuildStats stats_{};
};

}  // namespace bvh
