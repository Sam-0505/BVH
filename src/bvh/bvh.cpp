#include "bvh/bvh.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "geometry/intersect.hpp"
#include "geometry/triangle.hpp"

namespace bvh {

using geom::AABB;
using geom::MeshHit;
using geom::QueryStats;
using geom::Ray;
using geom::Scalar;
using geom::TriangleHit;
using geom::Vec3;

const char* toString(SplitStrategy s) {
    switch (s) {
        case SplitStrategy::ObjectMedian: return "object_median";
        case SplitStrategy::CentroidMedian: return "centroid_median";
        case SplitStrategy::BinnedSAH: return "binned_sah";
    }
    return "unknown";
}

bool BuildConfig::isValid(std::string* error) const {
    const auto fail = [error](const char* msg) {
        if (error != nullptr) *error = msg;
        return false;
    };
    // 0 would make every node unsplittable-but-not-a-leaf.
    if (maxLeafSize == 0) return fail("maxLeafSize must be at least 1");
    if (maxDepth > kMaxDepthLimit) return fail("maxDepth exceeds kMaxDepthLimit (64)");
    if (sahBinCount != 4 && sahBinCount != 8 && sahBinCount != 16 && sahBinCount != 32) {
        return fail("sahBinCount must be one of 4, 8, 16, or 32");
    }
    if (error != nullptr) error->clear();
    return true;
}

namespace {

constexpr std::uint32_t kMaxSahBins = 32;

struct SahSplit {
    int axis{0};
    std::uint32_t splitBin{0};
    Scalar cost{std::numeric_limits<Scalar>::infinity()};
    bool valid{false};
};

struct RangeInfo {
    AABB bounds;
    AABB centroidBounds;
};

std::uint32_t binIndex(const BuildConfig& config, const AABB& centroidBounds,
                       const Vec3& centroid, int axis) {
    const Scalar extent = centroidBounds.max[axis] - centroidBounds.min[axis];
    if (!(extent > Scalar(0)) || !std::isfinite(extent)) return 0;
    const Scalar offset = (centroid[axis] - centroidBounds.min[axis]) / extent;
    const Scalar scaled = offset * static_cast<Scalar>(config.sahBinCount);
    if (!std::isfinite(scaled)) return 0;
    const int bin = static_cast<int>(std::floor(scaled));
    return static_cast<std::uint32_t>(std::max(0, std::min(bin,
        static_cast<int>(config.sahBinCount) - 1)));
}

RangeInfo rangeInfo(std::uint32_t first, std::uint32_t count,
                    const std::vector<std::uint32_t>& indices,
                    const std::vector<AABB>& triBounds,
                    const std::vector<Vec3>& centroids) {
    RangeInfo info;
    for (std::uint32_t i = first; i < first + count; ++i) {
        const std::uint32_t p = indices[i];
        info.bounds.extend(triBounds[p]);
        info.centroidBounds.extend(centroids[p]);
    }
    return info;
}

SahSplit binnedSah(const BuildConfig& config, std::uint32_t first, std::uint32_t count,
                   const std::vector<std::uint32_t>& indices,
                   const std::vector<AABB>& triBounds, const std::vector<Vec3>& centroids,
                   const RangeInfo& range) {
    const Scalar parentArea = range.bounds.surfaceArea();
    if (!(parentArea > Scalar(0)) || !std::isfinite(parentArea)) return {};

    struct Bin {
        AABB bounds;
        std::uint32_t count{0};
    };
    std::array<Bin, kMaxSahBins> bins;
    SahSplit best;

    for (int axis = 0; axis < 3; ++axis) {
        const Scalar extent = range.centroidBounds.max[axis] - range.centroidBounds.min[axis];
        if (!(extent > Scalar(0)) || !std::isfinite(extent)) continue;
        for (std::uint32_t i = 0; i < config.sahBinCount; ++i) bins[i] = Bin{};
        for (std::uint32_t i = first; i < first + count; ++i) {
            const std::uint32_t p = indices[i];
            Bin& bin = bins[binIndex(config, range.centroidBounds, centroids[p], axis)];
            ++bin.count;
            bin.bounds.extend(triBounds[p]);
        }

        std::array<AABB, kMaxSahBins> leftBounds;
        std::array<AABB, kMaxSahBins> rightBounds;
        std::array<std::uint32_t, kMaxSahBins> leftCounts{};
        std::array<std::uint32_t, kMaxSahBins> rightCounts{};
        AABB runningBounds;
        std::uint32_t runningCount = 0;
        for (std::uint32_t i = 0; i < config.sahBinCount; ++i) {
            runningBounds.extend(bins[i].bounds);
            runningCount += bins[i].count;
            leftBounds[i] = runningBounds;
            leftCounts[i] = runningCount;
        }
        runningBounds = AABB{};
        runningCount = 0;
        for (std::uint32_t i = config.sahBinCount; i-- > 0;) {
            runningBounds.extend(bins[i].bounds);
            runningCount += bins[i].count;
            rightBounds[i] = runningBounds;
            rightCounts[i] = runningCount;
        }

        for (std::uint32_t split = 0; split + 1 < config.sahBinCount; ++split) {
            const std::uint32_t leftCount = leftCounts[split];
            const std::uint32_t rightCount = rightCounts[split + 1];
            if (leftCount == 0 || rightCount == 0) continue;
            const Scalar leftArea = leftBounds[split].surfaceArea();
            const Scalar rightArea = rightBounds[split + 1].surfaceArea();
            const Scalar cost = Scalar(1) +
                (leftArea / parentArea) * static_cast<Scalar>(leftCount) +
                (rightArea / parentArea) * static_cast<Scalar>(rightCount);
            // Strictly better only: axis/bin scan order makes equal costs first-wins.
            if (!std::isfinite(leftArea) || !std::isfinite(rightArea) ||
                !std::isfinite(cost) || cost >= best.cost) {
                continue;
            }
            best.axis = axis;
            best.splitBin = split;
            best.cost = cost;
            best.valid = true;
        }
    }
    return best;
}

// Build scratch. Lives only for the duration of build(): the per-triangle
// bounds and centroids cost 36 bytes per triangle but save refetching three
// vertices at every level of the recursion.
struct Builder {
    explicit Builder(const BuildConfig& c) : cfg(c) {}

    const BuildConfig& cfg;
    std::vector<AABB> triBounds;
    std::vector<Vec3> centroids;
    std::vector<std::uint32_t> indices;
    std::vector<BVHNode> nodes;

    std::uint64_t leafDepthSum{0};
    std::size_t leafCount{0};
    std::size_t largestLeaf{0};
    std::uint32_t deepest{0};

    void makeLeaf(std::uint32_t nodeIdx, std::uint32_t first, std::uint32_t count,
                  std::uint32_t depth) {
        nodes[nodeIdx].leftOrFirst = first;
        nodes[nodeIdx].count = count;
        ++leafCount;
        leafDepthSum += depth;
        largestLeaf = std::max(largestLeaf, static_cast<std::size_t>(count));
        deepest = std::max(deepest, depth);
    }

    // Reorders [first, first+count) so the lower half by centroid comes first.
    // nth_element is introselect: O(count) expected, and unlike a full sort it
    // does no work ordering within each half, which the tree does not need.
    std::uint32_t objectMedian(std::uint32_t first, std::uint32_t count, int axis) {
        const std::uint32_t mid = first + count / 2;
        const auto begin = indices.begin();
        std::nth_element(begin + first, begin + mid, begin + first + count,
                         [this, axis](std::uint32_t a, std::uint32_t b) {
                             return centroids[a][axis] < centroids[b][axis];
                         });
        return mid;
    }

    std::uint32_t centroidMedian(std::uint32_t first, std::uint32_t count, int axis,
                                 Scalar split) {
        const auto begin = indices.begin();
        const auto it = std::partition(begin + first, begin + first + count,
                                       [this, axis, split](std::uint32_t p) {
                                           return centroids[p][axis] < split;
                                       });
        return static_cast<std::uint32_t>(it - begin);
    }

    // The node slot is allocated by the caller so that siblings stay adjacent.
    void buildNode(std::uint32_t nodeIdx, std::uint32_t first, std::uint32_t count,
                   std::uint32_t depth) {
        const RangeInfo range = rangeInfo(first, count, indices, triBounds, centroids);
        nodes[nodeIdx].bounds = range.bounds;

        if (count <= cfg.maxLeafSize || count < 2 || depth >= cfg.maxDepth) {
            makeLeaf(nodeIdx, first, count, depth);
            return;
        }

        std::uint32_t mid = first + count;
        if (cfg.strategy == SplitStrategy::BinnedSAH) {
            const SahSplit split = binnedSah(cfg, first, count, indices, triBounds, centroids, range);
            if (split.valid && split.cost >= static_cast<Scalar>(count)) {
                makeLeaf(nodeIdx, first, count, depth);
                return;
            }
            if (split.valid) {
                const auto begin = indices.begin();
                const auto it = std::partition(begin + first, begin + first + count,
                                               [this, &range, split](std::uint32_t p) {
                    return binIndex(cfg, range.centroidBounds, centroids[p], split.axis) <=
                           split.splitBin;
                });
                mid = static_cast<std::uint32_t>(it - begin);
            }
        } else {
            // Centroid bounds, not geometric bounds: a few large primitives can make
            // the longest geometric axis one along which the centroids barely spread.
            const int axis = range.centroidBounds.longestAxis();
            if (cfg.strategy == SplitStrategy::CentroidMedian) {
                mid = centroidMedian(first, count, axis, range.centroidBounds.centroid()[axis]);
            }
            if (mid == first || mid == first + count) {
                mid = objectMedian(first, count, axis);
            }
        }
        // An invalid SAH candidate or an empty bin partition falls
        // back to object median, which always makes two nonempty children.
        if (mid == first || mid == first + count) {
            const int axis = range.centroidBounds.longestAxis();
            mid = objectMedian(first, count, axis);
        }

        const auto leftIdx = static_cast<std::uint32_t>(nodes.size());
        nodes.resize(nodes.size() + 2);
        nodes[nodeIdx].leftOrFirst = leftIdx;
        nodes[nodeIdx].count = 0;

        buildNode(leftIdx, first, mid - first, depth + 1);
        buildNode(leftIdx + 1, mid, first + count - mid, depth + 1);
    }
};

}  // namespace

void BVH::build(const geom::Mesh& mesh, const BuildConfig& config) {
    std::string reason;
    if (!config.isValid(&reason)) {
        throw std::invalid_argument("BVH::build: invalid configuration: " + reason);
    }

    const std::size_t n = mesh.triangleCount();
    if (n > std::numeric_limits<std::uint32_t>::max() / 2) {
        throw std::invalid_argument("BVH::build: triangle count exceeds the 32-bit node indexing");
    }

    // Both throw points are behind us, so build() either succeeds or leaves the
    // object exactly as it was.
    const auto t0 = std::chrono::steady_clock::now();

    nodes_.clear();
    primitiveIndices_.clear();
    config_ = config;
    stats_ = BuildStats{};
    meshRevision_ = mesh.revision();

    if (n > 0) {
        Builder b(config);
        b.triBounds.resize(n);
        b.centroids.resize(n);
        b.indices.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            const geom::Triangle tri = mesh.triangle(i);
            b.triBounds[i] = tri.bounds();
            // Bounds centre, not the barycentre: Phase 3 bins by position
            // within the centroid bounds, and one notion of "centre" keeps the
            // bin index and the split side from ever disagreeing.
            b.centroids[i] = b.triBounds[i].centroid();
            b.indices[i] = static_cast<std::uint32_t>(i);
        }

        b.nodes.resize(1);
        b.buildNode(0, 0, static_cast<std::uint32_t>(n), 0);
        b.nodes.shrink_to_fit();

        nodes_ = std::move(b.nodes);
        primitiveIndices_ = std::move(b.indices);

        stats_.leafCount = b.leafCount;
        stats_.maxDepth = b.deepest;
        stats_.maxLeafSize = b.largestLeaf;
        stats_.meanLeafDepth =
            static_cast<double>(b.leafDepthSum) / static_cast<double>(b.leafCount);
        stats_.meanLeafSize = static_cast<double>(n) / static_cast<double>(b.leafCount);
    }

    stats_.nodeCount = nodes_.size();
    stats_.internalCount = nodes_.size() - stats_.leafCount;
    stats_.primitiveCount = n;
    stats_.memoryBytes = nodes_.size() * sizeof(BVHNode) +
                         primitiveIndices_.size() * sizeof(std::uint32_t);

    const auto t1 = std::chrono::steady_clock::now();
    stats_.buildTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
}

namespace {

struct StackEntry {
    std::uint32_t node;
    Scalar tEnter;
};

// A node's children sit one level deeper, so descending pushes at most one
// extra entry per level: maxDepth + 1 entries, and maxDepth is capped at 64.
constexpr std::size_t kStackCapacity = kMaxDepthLimit + 2;
static_assert(kStackCapacity >= kMaxDepthLimit + 1,
              "traversal stack must hold one entry per level plus the root");

// closestHit prunes by comparing a BOX-derived entry distance against a
// TRIANGLE-derived best. Different arithmetic reaches the same surface, so the
// leaf holding the true closest hit can enter with tEnter slightly ABOVE that
// hit's t and be culled. Only what is VISITED is widened; `best` itself stays
// exact, so a farther triangle is never accepted.
//
// EMPIRICAL, not a bound. The requirement is set by triangle conditioning, not
// by an operation count, so it grows with tessellation (roughly n^0.6) and no
// gamma(n) tracks it. Measured as the worst (leafTEnter - t)/t over pole rays:
//
//        484 tris  1.4e-07      79524 tris  2.9e-06     501264 tris  1.0e-05
//       8100 tris  9.6e-07     200704 tris  5.9e-06     799236 tris  1.1e-05
//      15876 tris  1.2e-06     320356 tris  5.9e-06    1201216 tris  1.4e-05
//      40000 tris  1.9e-06
//      grid and triangleSoup: 0 at every size tried
//
// 8*gamma(16) = 7.6e-06, so this holds to ~3x10^5 triangles with 1.3x headroom
// and is first exceeded at 501264, where 2 of the 6 pole rays come back wrong.
// 32x would push that past 1.2M and, up to 200k, costs nothing over 8x -- not
// taken, because the project's meshes top out at 8x10^4 and a documented limit
// beats a constant sized to hide one. Cost of 8x over no slack, on the pole
// rays that provoke it: +5.7% nodes expanded at 200k, +1.7% at 8k.
//
// Past the limit the failure stays small: a tie at a shared vertex resolves to
// the other triangle, t differing by 1.2e-06 at 501264 rising to 3.4e-06 at
// 1201216. Never a miss, and never a farther hit ACCEPTED -- `best` is exact,
// only visits are widened. gamma() and not a literal so this tracks Scalar.
#if BVH_CONSERVATIVE_RAY_BOX
constexpr Scalar kBestPruneSlack = Scalar(8) * geom::gamma(16);
#else
constexpr Scalar kBestPruneSlack = Scalar(0);
#endif

// Sign-safe: tMin may be negative, where scaling up would narrow the window.
inline Scalar pruneBound(Scalar best) {
    return best + std::fabs(best) * kBestPruneSlack;
}

}  // namespace

bool BVH::closestHit(const geom::Mesh& mesh, const Ray& ray, MeshHit& hit,
                     QueryStats* stats) const {
    assert(mesh.triangleCount() == primitiveIndices_.size() &&
           "BVH queried against a different mesh than it was built from");
    assert(mesh.revision() == meshRevision_ &&
           "BVH queried against a mesh modified since the build");
    // Release strips both asserts, and a shorter mesh would then index past the
    // end of its own buffers. Degraded answer beats undefined behaviour. This
    // does not catch a same-size mesh or an in-place transform -- that is what
    // the revision assert above is for.
    if (mesh.triangleCount() != primitiveIndices_.size()) return false;
    if (nodes_.empty()) return false;

    const Vec3 invDir = invDirection(ray);
    Scalar best = ray.tMax;
    // The far end needs the same slack as a found hit: a triangle sitting at
    // tMax lives in a box whose entry can round past it. +inf stays +inf.
    Scalar bestPrune = pruneBound(ray.tMax);
    MeshHit result;

    // Deliberately not value-initialised: `sp` bounds the live entries, and
    // zeroing 66 slots on every ray is real work in the hottest path.
    std::array<StackEntry, kStackCapacity> stack;
    std::size_t sp = 0;

    Scalar tEnter = ray.tMin;
    if (stats != nullptr) ++stats->aabbTests;
    if (!intersectRay(nodes_[0].bounds, ray.origin, invDir, ray.tMin, bestPrune, &tEnter)) {
        return false;
    }
    stack[sp++] = StackEntry{0u, tEnter};

    while (sp > 0) {
        const StackEntry entry = stack[--sp];
        // The node was queued before a closer hit was found; its whole subtree
        // is now behind the answer. This is where the near-first order pays off.
        if (entry.tEnter > bestPrune) {
            if (stats != nullptr) ++stats->nodesCulled;
            continue;
        }

        const BVHNode& node = nodes_[entry.node];
        if (stats != nullptr) ++stats->nodesExpanded;

        if (node.isLeaf()) {
            const std::uint32_t end = node.leftOrFirst + node.count;
            for (std::uint32_t i = node.leftOrFirst; i < end; ++i) {
                const std::uint32_t p = primitiveIndices_[i];
                TriangleHit th;
                if (intersectRayTriangle(mesh.triangle(p), ray.origin, ray.direction, ray.tMin,
                                         best, th)) {
                    best = th.t;
                    bestPrune = pruneBound(best);
                    result.t = th.t;
                    result.u = th.u;
                    result.v = th.v;
                    result.triangleIndex = p;
                }
            }
            if (stats != nullptr) stats->trianglesTested += node.count;
            continue;
        }

        const std::uint32_t left = node.leftOrFirst;
        const std::uint32_t right = left + 1;
        Scalar tLeft = ray.tMin;
        Scalar tRight = ray.tMin;
        const bool hitLeft =
            intersectRay(nodes_[left].bounds, ray.origin, invDir, ray.tMin, bestPrune, &tLeft);
        const bool hitRight =
            intersectRay(nodes_[right].bounds, ray.origin, invDir, ray.tMin, bestPrune, &tRight);
        if (stats != nullptr) stats->aabbTests += 2;

        assert(sp + 2 <= kStackCapacity && "traversal stack overflow");
        if (hitLeft && hitRight) {
            // Far child first, so the near one pops next and can shrink `best`
            // before the far one is ever expanded.
            if (tLeft <= tRight) {
                stack[sp++] = StackEntry{right, tRight};
                stack[sp++] = StackEntry{left, tLeft};
            } else {
                stack[sp++] = StackEntry{left, tLeft};
                stack[sp++] = StackEntry{right, tRight};
            }
        } else if (hitLeft) {
            stack[sp++] = StackEntry{left, tLeft};
        } else if (hitRight) {
            stack[sp++] = StackEntry{right, tRight};
        }
    }

    if (!result.valid()) return false;
    hit = result;
    return true;
}

bool BVH::anyHit(const geom::Mesh& mesh, const Ray& ray, QueryStats* stats) const {
    assert(mesh.triangleCount() == primitiveIndices_.size() &&
           "BVH queried against a different mesh than it was built from");
    assert(mesh.revision() == meshRevision_ &&
           "BVH queried against a mesh modified since the build");
    if (mesh.triangleCount() != primitiveIndices_.size()) return false;
    if (nodes_.empty()) return false;

    const Vec3 invDir = invDirection(ray);
    const Scalar boxTMax = pruneBound(ray.tMax);

    // No ordering and no shrinking tMax: any intersection ends the query, so
    // the near child has no advantage.
    std::array<std::uint32_t, kStackCapacity> stack;
    std::size_t sp = 0;

    if (stats != nullptr) ++stats->aabbTests;
    if (!intersectRay(nodes_[0].bounds, ray.origin, invDir, ray.tMin, boxTMax)) return false;
    stack[sp++] = 0u;

    while (sp > 0) {
        const BVHNode& node = nodes_[stack[--sp]];
        if (stats != nullptr) ++stats->nodesExpanded;

        if (node.isLeaf()) {
            const std::uint32_t end = node.leftOrFirst + node.count;
            for (std::uint32_t i = node.leftOrFirst; i < end; ++i) {
                const std::uint32_t p = primitiveIndices_[i];
                TriangleHit th;
                if (stats != nullptr) ++stats->trianglesTested;
                if (intersectRayTriangle(mesh.triangle(p), ray.origin, ray.direction, ray.tMin,
                                         ray.tMax, th)) {
                    return true;
                }
            }
            continue;
        }

        const std::uint32_t left = node.leftOrFirst;
        assert(sp + 2 <= kStackCapacity && "traversal stack overflow");
        for (std::uint32_t c = left; c <= left + 1; ++c) {
            if (stats != nullptr) ++stats->aabbTests;
            if (intersectRay(nodes_[c].bounds, ray.origin, invDir, ray.tMin, boxTMax)) {
                stack[sp++] = c;
            }
        }
    }

    return false;
}

bool BVH::validate(const geom::Mesh& mesh, std::string* error) const {
    const auto fail = [error](const char* msg) {
        if (error != nullptr) *error = msg;
        return false;
    };
    if (error != nullptr) error->clear();

    const std::size_t n = mesh.triangleCount();
    if (primitiveIndices_.size() != n) return fail("index array size does not match the mesh");
    if (n == 0) return nodes_.empty() ? true : fail("empty mesh produced nodes");
    if (nodes_.empty()) return fail("non-empty mesh produced no nodes");
    if (stats_.nodeCount != nodes_.size()) return fail("stats.nodeCount disagrees with nodes()");

    // Keep validation's early-leaf decision bit-for-bit aligned with build().
    std::vector<AABB> triBounds;
    std::vector<Vec3> centroids;
    if (config_.strategy == SplitStrategy::BinnedSAH) {
        triBounds.resize(n);
        centroids.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            triBounds[i] = mesh.triangle(i).bounds();
            centroids[i] = triBounds[i].centroid();
        }
    }

    std::vector<char> primSeen(n, 0);
    std::vector<char> slotSeen(n, 0);

    std::vector<std::pair<std::uint32_t, std::uint32_t>> pending;
    pending.emplace_back(0u, 0u);

    std::size_t leaves = 0;
    std::size_t internals = 0;
    std::uint32_t deepest = 0;
    std::uint64_t depthSum = 0;

    while (!pending.empty()) {
        const std::uint32_t idx = pending.back().first;
        const std::uint32_t depth = pending.back().second;
        pending.pop_back();

        if (depth > config_.maxDepth) return fail("node deeper than the configured maxDepth");
        const BVHNode& node = nodes_[idx];

        if (node.isLeaf()) {
            ++leaves;
            depthSum += depth;
            deepest = std::max(deepest, depth);

            if (node.leftOrFirst > n || node.count > n - node.leftOrFirst) {
                return fail("leaf range runs past the index array");
            }
            if (node.count > config_.maxLeafSize && depth != config_.maxDepth) {
                if (config_.strategy != SplitStrategy::BinnedSAH) {
                    return fail("leaf exceeds maxLeafSize above the depth cap");
                }
                const RangeInfo range = rangeInfo(node.leftOrFirst, node.count,
                                                  primitiveIndices_, triBounds, centroids);
                const SahSplit split = binnedSah(config_, node.leftOrFirst, node.count,
                                                  primitiveIndices_, triBounds, centroids, range);
                if (!split.valid || split.cost < static_cast<Scalar>(node.count)) {
                    return fail("oversized SAH leaf has a beneficial split");
                }
            }

            const std::uint32_t end = node.leftOrFirst + node.count;
            geom::AABB leafUnion;
            for (std::uint32_t i = node.leftOrFirst; i < end; ++i) {
                if (slotSeen[i] != 0) return fail("two leaves claim the same index slot");
                slotSeen[i] = 1;
                const std::uint32_t p = primitiveIndices_[i];
                if (p >= n) return fail("primitive index out of range");
                if (primSeen[p] != 0) return fail("primitive referenced by two leaves");
                primSeen[p] = 1;
                const geom::AABB tb = mesh.triangle(p).bounds();
                if (!node.bounds.contains(tb)) {
                    return fail("leaf bounds do not contain one of its triangles");
                }
                leafUnion.extend(tb);
            }
            if (node.bounds != leafUnion) {
                return fail("leaf bounds are not the tight union of its triangles");
            }
            continue;
        }

        ++internals;
        const std::uint32_t left = node.leftOrFirst;
        // Children are appended after their parent, so a strictly increasing
        // index is also the proof that the graph is acyclic.
        if (left <= idx) return fail("child index does not exceed the parent index");
        if (static_cast<std::size_t>(left) + 1 >= nodes_.size()) {
            return fail("child index out of range");
        }
        if (!node.bounds.contains(nodes_[left].bounds) ||
            !node.bounds.contains(nodes_[left + 1].bounds)) {
            return fail("child bounds are not contained in the parent bounds");
        }
        // Containment alone would accept every internal node set to the root
        // bounds -- correct answers, traversal degraded to brute force, and
        // nothing to detect it. Exact equality is right because both sides are
        // the same fmin/fmax union, so they agree bit for bit.
        if (node.bounds != merge(nodes_[left].bounds, nodes_[left + 1].bounds)) {
            return fail("internal bounds are not the tight union of the child bounds");
        }
        pending.emplace_back(left, depth + 1);
        pending.emplace_back(left + 1, depth + 1);
    }

    if (leaves + internals != nodes_.size()) return fail("some nodes are unreachable");
    for (std::size_t i = 0; i < n; ++i) {
        if (primSeen[i] == 0) return fail("a triangle is referenced by no leaf");
        if (slotSeen[i] == 0) return fail("an index slot belongs to no leaf");
    }

    if (stats_.leafCount != leaves) return fail("stats.leafCount disagrees with the tree");
    if (stats_.internalCount != internals) return fail("stats.internalCount disagrees");
    if (stats_.maxDepth != deepest) return fail("stats.maxDepth disagrees with the tree");
    if (leaves != 0) {
        const double mean = static_cast<double>(depthSum) / static_cast<double>(leaves);
        if (std::fabs(mean - stats_.meanLeafDepth) > 1e-9) {
            return fail("stats.meanLeafDepth disagrees with the tree");
        }
    }
    return true;
}

}  // namespace bvh
