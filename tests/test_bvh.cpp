#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "bvh/bvh.hpp"
#include "geometry/mesh_query.hpp"
#include "scene/procedural.hpp"

using namespace geom;
using bvh::BuildConfig;
using bvh::BVH;
using bvh::SplitStrategy;

namespace {

constexpr SplitStrategy kStrategies[] = {SplitStrategy::ObjectMedian,
                                         SplitStrategy::CentroidMedian};

// mt19937 with a hand-rolled [0,1) mapping: std::uniform_real_distribution is
// not reproducible across standard libraries, and a comparison against brute
// force is only worth anything on a fixed ray set.
class RayGen {
public:
    explicit RayGen(std::uint32_t seed) : rng_(seed) {}

    // Top 24 bits only -- scaling all 32 rounds up to exactly 1.0 in float.
    Scalar unit() { return static_cast<Scalar>(rng_() >> 8) * (Scalar(1) / Scalar(16777216.0)); }
    Scalar signed1() { return Scalar(2) * unit() - Scalar(1); }

    Vec3 onUnitSphere() {
        const Scalar z = signed1();
        const Scalar phi = Scalar(2) * kPi * unit();
        const Scalar r = std::sqrt(std::fmax(Scalar(0), Scalar(1) - z * z));
        return Vec3(r * std::cos(phi), r * std::sin(phi), z);
    }

    // Origins on a sphere outside the mesh, aimed at a random point in a box
    // 1.2x the mesh bounds -- wide enough that a useful fraction of rays miss.
    std::vector<Ray> aroundBounds(const AABB& box, std::size_t count) {
        const Vec3 c = box.centroid();
        const Scalar radius = Scalar(0.5) * length(box.diagonal()) + Scalar(1e-3);
        std::vector<Ray> rays;
        rays.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const Vec3 origin = c + onUnitSphere() * (radius * (Scalar(1.5) + unit()));
            const Vec3 target = c + Vec3(signed1(), signed1(), signed1()) *
                                        (Scalar(0.6) * radius) * Scalar(1.2);
            rays.emplace_back(origin, target - origin);
        }
        return rays;
    }

    std::vector<Ray> insideBounds(const AABB& box, std::size_t count) {
        const Vec3 c = box.centroid();
        const Vec3 half = box.diagonal() * Scalar(0.5);
        std::vector<Ray> rays;
        rays.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const Vec3 origin = c + Vec3(mul(half, Vec3(signed1(), signed1(), signed1())));
            rays.emplace_back(origin, onUnitSphere());
        }
        return rays;
    }

private:
    std::mt19937 rng_;
};

std::vector<Ray> axisAlignedRays(const AABB& box, int samplesPerAxis) {
    const Vec3 c = box.centroid();
    const Vec3 d = box.diagonal();
    const Scalar reach = length(d) + Scalar(1);
    std::vector<Ray> rays;
    for (int axis = 0; axis < 3; ++axis) {
        for (int i = 0; i < samplesPerAxis; ++i) {
            // Deliberately includes offset 0, which puts a ray exactly on the
            // slab planes of any box centred on the mesh: the 0 * inf case.
            const Scalar f = static_cast<Scalar>(i) / static_cast<Scalar>(samplesPerAxis) -
                             Scalar(0.5);
            for (int sign = -1; sign <= 1; sign += 2) {
                Vec3 origin = c;
                Vec3 dir(Scalar(0));
                origin[axis] -= static_cast<Scalar>(sign) * reach;
                dir[axis] = static_cast<Scalar>(sign);
                origin[(axis + 1) % 3] += f * d[(axis + 1) % 3];
                origin[(axis + 2) % 3] += f * d[(axis + 2) % 3];
                rays.emplace_back(origin, dir);
            }
        }
    }
    return rays;
}

struct Agreement {
    std::size_t hits{0};
    std::uint64_t bvhTriangles{0};
    std::uint64_t bruteTriangles{0};
    std::uint64_t nodesExpanded{0};
    std::uint64_t aabbTests{0};
};

// The whole correctness argument: identical answers to the O(n) oracle.
// `t` is compared with a relative tolerance because ties between two nearly
// coincident triangles can be broken differently by the two traversal orders.
void checkAgainstBruteForce(const Mesh& mesh, const BVH& tree, const std::vector<Ray>& rays,
                            const std::string& label, Agreement* out = nullptr) {
    Agreement agg;
    for (std::size_t i = 0; i < rays.size(); ++i) {
        const Ray& r = rays[i];
        MeshHit expected;
        MeshHit actual;
        QueryStats bruteStats;
        QueryStats bvhStats;

        const bool expectHit = bruteForceClosestHit(mesh, r, expected, &bruteStats);
        const bool actualHit = tree.closestHit(mesh, r, actual, &bvhStats);

        ASSERT_EQ(expectHit, actualHit) << label << ": closestHit disagrees on ray " << i;
        if (expectHit) {
            ++agg.hits;
            EXPECT_TRUE(nearlyEqual(expected.t, actual.t, 1e-5f))
                << label << ": ray " << i << " t " << actual.t << " vs brute force " << expected.t;

            // Both paths run the same intersectRayTriangle on the same
            // Triangle, so agreeing on the triangle means agreeing bit for bit.
            // Phases 4/5/7 consume u and v; a correct distance with a garbage
            // barycentric would otherwise pass every test here.
            if (expected.triangleIndex == actual.triangleIndex) {
                EXPECT_EQ(expected.u, actual.u) << label << ": ray " << i << " u";
                EXPECT_EQ(expected.v, actual.v) << label << ": ray " << i << " v";
            } else {
#if BVH_CONSERVATIVE_RAY_BOX
                // A different triangle is legitimate only at an exact tie.
                EXPECT_EQ(expected.t, actual.t)
                    << label << ": ray " << i << " picked a different triangle ("
                    << actual.triangleIndex << " vs " << expected.triangleIndex
                    << ") at a distance that is not an exact tie";
#endif
                // With the exact slab test the BVH can cull a node holding a
                // marginally closer triangle, so only the tolerance check above
                // applies. BVHWidening.* characterises that difference.
            }
        }

        const bool expectAny = bruteForceAnyHit(mesh, r);
        const bool actualAny = tree.anyHit(mesh, r);
        ASSERT_EQ(expectAny, actualAny) << label << ": anyHit disagrees on ray " << i;
        ASSERT_EQ(expectAny, expectHit) << label << ": oracle self-inconsistent on ray " << i;

        agg.bvhTriangles += bvhStats.trianglesTested;
        agg.bruteTriangles += bruteStats.trianglesTested;
        agg.nodesExpanded += bvhStats.nodesExpanded;
        agg.aabbTests += bvhStats.aabbTests;
    }
    // A ray set that never hits anything would pass everything above vacuously.
    EXPECT_GT(agg.hits, rays.size() / 10) << label << ": ray set hits almost nothing";
    if (out != nullptr) *out = agg;
}

void expectValid(const BVH& tree, const Mesh& mesh, const std::string& label) {
    std::string error;
    EXPECT_TRUE(tree.validate(mesh, &error)) << label << ": " << error;
}

Mesh coincidentTriangles(std::size_t count) {
    std::vector<Vec3> p;
    std::vector<std::uint32_t> idx;
    for (std::size_t i = 0; i < count; ++i) {
        const auto base = static_cast<std::uint32_t>(p.size());
        p.push_back(Vec3(0.0f, 0.0f, 0.0f));
        p.push_back(Vec3(1.0f, 0.0f, 0.0f));
        p.push_back(Vec3(0.0f, 1.0f, 0.0f));
        idx.insert(idx.end(), {base, base + 1, base + 2});
    }
    return Mesh(std::move(p), std::move(idx));
}

// Half real triangles, half zero-area ones sharing their vertices. The z step
// keeps the stack roughly as deep as it is wide, so a ray set aimed at its
// bounds actually hits it.
Mesh withDegenerateTriangles(std::size_t pairs) {
    std::vector<Vec3> p;
    std::vector<std::uint32_t> idx;
    for (std::size_t i = 0; i < pairs; ++i) {
        const Scalar z = static_cast<Scalar>(i) * (2.0f / static_cast<Scalar>(pairs));
        const auto base = static_cast<std::uint32_t>(p.size());
        p.push_back(Vec3(-1.0f, -1.0f, z));
        p.push_back(Vec3(1.0f, -1.0f, z));
        p.push_back(Vec3(0.0f, 1.0f, z));
        idx.insert(idx.end(), {base, base + 1, base + 2});
        idx.insert(idx.end(), {base, base + 1, base + 1});  // zero area
    }
    return Mesh(std::move(p), std::move(idx));
}

// Centroids at x = 2^-i. A centroid-midpoint split peels exactly one triangle
// off per level, which is the only way to reach the depth cap with a median
// build -- object median always halves.
Mesh geometricallyClusteredTriangles(std::size_t count) {
    std::vector<Vec3> p;
    std::vector<std::uint32_t> idx;
    const Scalar s = 1e-4f;
    for (std::size_t i = 0; i < count; ++i) {
        const Scalar x = std::ldexp(1.0f, -static_cast<int>(i));
        const auto base = static_cast<std::uint32_t>(p.size());
        p.push_back(Vec3(x, -s, -s));
        p.push_back(Vec3(x, s, -s));
        p.push_back(Vec3(x, 0.0f, s));
        idx.insert(idx.end(), {base, base + 1, base + 2});
    }
    return Mesh(std::move(p), std::move(idx));
}

}  // namespace

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

TEST(BVHConfig, DefaultIsValid) {
    std::string error;
    EXPECT_TRUE(BuildConfig{}.isValid(&error)) << error;
    EXPECT_TRUE(error.empty());
}

TEST(BVHConfig, RejectsZeroLeafSize) {
    BuildConfig cfg;
    cfg.maxLeafSize = 0;
    std::string error;
    EXPECT_FALSE(cfg.isValid(&error));
    EXPECT_NE(error.find("maxLeafSize"), std::string::npos) << error;

    BVH tree;
    const Mesh m = scene::uvSphere(200);
    EXPECT_THROW(tree.build(m, cfg), std::invalid_argument);
}

TEST(BVHConfig, RejectsDepthBeyondTheTraversalStack) {
    BuildConfig cfg;
    cfg.maxDepth = bvh::kMaxDepthLimit + 1;
    EXPECT_FALSE(cfg.isValid());

    BVH tree;
    const Mesh m = scene::uvSphere(200);
    EXPECT_THROW(tree.build(m, cfg), std::invalid_argument);
}

TEST(BVHConfig, AcceptsTheWholeUiRange) {
    for (std::uint32_t leaf = bvh::kUiMinLeafSize; leaf <= bvh::kUiMaxLeafSize; ++leaf) {
        for (std::uint32_t depth = bvh::kUiMinDepth; depth <= bvh::kUiMaxDepth; ++depth) {
            BuildConfig cfg;
            cfg.maxLeafSize = leaf;
            cfg.maxDepth = depth;
            EXPECT_TRUE(cfg.isValid()) << "leaf " << leaf << " depth " << depth;
        }
    }
}

// ---------------------------------------------------------------------------
// Structure
// ---------------------------------------------------------------------------

TEST(BVHBuild, EmptyMeshProducesAnEmptyTree) {
    const Mesh empty;
    BVH tree;
    tree.build(empty, BuildConfig{});
    EXPECT_TRUE(tree.empty());
    EXPECT_EQ(tree.stats().nodeCount, 0u);
    EXPECT_EQ(tree.stats().memoryBytes, 0u);
    EXPECT_TRUE(tree.bounds().isEmpty());
    expectValid(tree, empty, "empty mesh");

    MeshHit hit;
    EXPECT_FALSE(tree.closestHit(empty, Ray(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f)), hit));
    EXPECT_FALSE(tree.anyHit(empty, Ray(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f))));
}

TEST(BVHBuild, SingleTriangleIsOneLeaf) {
    const Mesh m({{0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}}, {0, 1, 2});
    BuildConfig cfg;
    cfg.maxLeafSize = 1;
    BVH tree;
    tree.build(m, cfg);

    ASSERT_EQ(tree.nodes().size(), 1u);
    EXPECT_TRUE(tree.nodes()[0].isLeaf());
    EXPECT_EQ(tree.nodes()[0].count, 1u);
    EXPECT_EQ(tree.stats().maxDepth, 0u);
    expectValid(tree, m, "single triangle");

    MeshHit hit;
    ASSERT_TRUE(tree.closestHit(m, Ray(Vec3(0.5f, 0.5f, -1.0f), Vec3(0.0f, 0.0f, 1.0f)), hit));
    EXPECT_NEAR(hit.t, 1.0f, 1e-5f);
    EXPECT_EQ(hit.triangleIndex, 0u);
}

TEST(BVHBuild, TreeIsFullSoNodeCountFollowsLeafCount) {
    const Mesh m = scene::uvSphere(4000);
    for (SplitStrategy s : kStrategies) {
        for (std::uint32_t leaf : {1u, 4u, 32u}) {
            BuildConfig cfg;
            cfg.strategy = s;
            cfg.maxLeafSize = leaf;
            BVH tree;
            tree.build(m, cfg);
            const auto& st = tree.stats();
            // Every internal node has exactly two children, so nodes = 2L - 1.
            EXPECT_EQ(st.nodeCount, 2 * st.leafCount - 1);
            EXPECT_EQ(st.internalCount, st.leafCount - 1);
            EXPECT_EQ(st.primitiveCount, m.triangleCount());
            EXPECT_NEAR(st.meanLeafSize * static_cast<double>(st.leafCount),
                        static_cast<double>(m.triangleCount()), 1e-6);
            EXPECT_LE(st.meanLeafDepth, static_cast<double>(st.maxDepth));
            EXPECT_EQ(st.memoryBytes, st.nodeCount * 32 + m.triangleCount() * 4);
        }
    }
}

TEST(BVHBuild, SmallerLeavesProduceMoreNodesAndMoreDepth) {
    const Mesh m = scene::uvSphere(8000);
    std::size_t previousNodes = 0;
    for (std::uint32_t leaf : {32u, 16u, 8u, 4u, 2u, 1u}) {
        BuildConfig cfg;
        cfg.maxLeafSize = leaf;
        BVH tree;
        tree.build(m, cfg);
        EXPECT_GT(tree.stats().nodeCount, previousNodes) << "leaf " << leaf;
        previousNodes = tree.stats().nodeCount;
    }
}

TEST(BVHBuild, MaxDepthOneStopsAfterOneSplit) {
    const Mesh m = scene::uvSphere(4000);
    BuildConfig cfg;
    cfg.maxDepth = 1;
    cfg.maxLeafSize = 1;
    BVH tree;
    tree.build(m, cfg);

    EXPECT_EQ(tree.stats().nodeCount, 3u);
    EXPECT_EQ(tree.stats().leafCount, 2u);
    EXPECT_EQ(tree.stats().maxDepth, 1u);
    // The depth limit wins over maxLeafSize, so leaves overflow it by design.
    EXPECT_GT(tree.stats().maxLeafSize, cfg.maxLeafSize);
    expectValid(tree, m, "maxDepth 1");
}

TEST(BVHBuild, MaxDepthZeroIsASingleLeaf) {
    const Mesh m = scene::uvSphere(1000);
    BuildConfig cfg;
    cfg.maxDepth = 0;
    BVH tree;
    tree.build(m, cfg);
    EXPECT_EQ(tree.stats().nodeCount, 1u);
    EXPECT_EQ(tree.stats().maxLeafSize, m.triangleCount());
    expectValid(tree, m, "maxDepth 0");
}

TEST(BVHBuild, RootBoundsMatchTheMesh) {
    const Mesh m = scene::uvSphere(2000);
    BVH tree;
    tree.build(m, BuildConfig{});
    EXPECT_TRUE(nearlyEqual(tree.bounds().min, m.bounds().min, 1e-5f));
    EXPECT_TRUE(nearlyEqual(tree.bounds().max, m.bounds().max, 1e-5f));
}

TEST(BVHBuild, RebuildingReplacesThePreviousTree) {
    const Mesh sphere = scene::uvSphere(2000);
    const Mesh soup = scene::triangleSoup(500, 3);
    BVH tree;

    BuildConfig fine;
    fine.maxLeafSize = 1;
    tree.build(sphere, fine);
    const std::size_t fineNodes = tree.stats().nodeCount;
    expectValid(tree, sphere, "first build");

    BuildConfig coarse;
    coarse.maxLeafSize = 32;
    tree.build(soup, coarse);
    expectValid(tree, soup, "rebuild on a different mesh");
    EXPECT_EQ(tree.primitiveCount(), soup.triangleCount());
    EXPECT_NE(tree.stats().nodeCount, fineNodes);
    EXPECT_EQ(tree.config().maxLeafSize, 32u);
}

// ---------------------------------------------------------------------------
// Agreement with brute force
// ---------------------------------------------------------------------------

TEST(BVHQuery, MatchesBruteForceOnASphere) {
    const Mesh m = scene::uvSphere(2000);
    const std::vector<Ray> rays = RayGen(11).aroundBounds(m.bounds(), 3000);
    for (SplitStrategy s : kStrategies) {
        BuildConfig cfg;
        cfg.strategy = s;
        BVH tree;
        tree.build(m, cfg);
        expectValid(tree, m, bvh::toString(s));
        checkAgainstBruteForce(m, tree, rays, std::string("sphere/") + bvh::toString(s));
    }
}

TEST(BVHQuery, MatchesBruteForceOnAFlatGrid) {
    const Mesh m = scene::grid(2000);
    const std::vector<Ray> rays = RayGen(12).aroundBounds(m.bounds(), 1500);
    for (SplitStrategy s : kStrategies) {
        BuildConfig cfg;
        cfg.strategy = s;
        BVH tree;
        tree.build(m, cfg);
        expectValid(tree, m, bvh::toString(s));
        checkAgainstBruteForce(m, tree, rays, std::string("grid/") + bvh::toString(s));
    }
}

TEST(BVHQuery, MatchesBruteForceOnALargeTriangleSoup) {
    const Mesh m = scene::triangleSoup(12000, 5, 1.0f, 0.08f);
    ASSERT_GT(m.triangleCount(), 10000u);
    const std::vector<Ray> rays = RayGen(13).aroundBounds(m.bounds(), 600);
    for (SplitStrategy s : kStrategies) {
        BuildConfig cfg;
        cfg.strategy = s;
        BVH tree;
        tree.build(m, cfg);
        expectValid(tree, m, bvh::toString(s));
        checkAgainstBruteForce(m, tree, rays, std::string("soup12k/") + bvh::toString(s));
    }
}

TEST(BVHQuery, MatchesBruteForceOnALargeSphere) {
    const Mesh m = scene::uvSphere(16000);
    ASSERT_GT(m.triangleCount(), 10000u);
    const std::vector<Ray> rays = RayGen(14).aroundBounds(m.bounds(), 500);
    BVH tree;
    tree.build(m, BuildConfig{});
    expectValid(tree, m, "sphere16k");
    checkAgainstBruteForce(m, tree, rays, "sphere16k");
}

TEST(BVHQuery, EveryLeafSizeAndStrategyGivesTheSameAnswers) {
    const Mesh m = scene::uvSphere(3000);
    const std::vector<Ray> rays = RayGen(21).aroundBounds(m.bounds(), 400);
    for (SplitStrategy s : kStrategies) {
        for (std::uint32_t leaf : {1u, 2u, 4u, 8u, 16u, 32u}) {
            BuildConfig cfg;
            cfg.strategy = s;
            cfg.maxLeafSize = leaf;
            BVH tree;
            tree.build(m, cfg);
            const std::string label =
                std::string(bvh::toString(s)) + "/leaf" + std::to_string(leaf);
            expectValid(tree, m, label);
            EXPECT_LE(tree.stats().maxDepth, cfg.maxDepth) << label;
            checkAgainstBruteForce(m, tree, rays, label);
        }
    }
}

TEST(BVHQuery, EveryDepthLimitGivesTheSameAnswers) {
    const Mesh m = scene::uvSphere(3000);
    const std::vector<Ray> rays = RayGen(22).aroundBounds(m.bounds(), 300);
    for (SplitStrategy s : kStrategies) {
        for (std::uint32_t depth : {1u, 2u, 8u, 12u, 16u, 24u, 32u, 64u}) {
            BuildConfig cfg;
            cfg.strategy = s;
            cfg.maxDepth = depth;
            BVH tree;
            tree.build(m, cfg);
            const std::string label =
                std::string(bvh::toString(s)) + "/depth" + std::to_string(depth);
            expectValid(tree, m, label);
            EXPECT_LE(tree.stats().maxDepth, depth) << label;
            checkAgainstBruteForce(m, tree, rays, label);
        }
    }
}

TEST(BVHQuery, HandlesRaysStartingInsideTheGeometry) {
    const Mesh m = scene::uvSphere(2000);
    const std::vector<Ray> rays = RayGen(31).insideBounds(m.bounds(), 1500);
    BVH tree;
    tree.build(m, BuildConfig{});
    checkAgainstBruteForce(m, tree, rays, "inside");
}

TEST(BVHQuery, HandlesAxisAlignedRays) {
    for (const Mesh& m : {scene::uvSphere(2000), scene::grid(1000)}) {
        const std::vector<Ray> rays = axisAlignedRays(m.bounds(), 24);
        for (SplitStrategy s : kStrategies) {
            BuildConfig cfg;
            cfg.strategy = s;
            BVH tree;
            tree.build(m, cfg);
            checkAgainstBruteForce(m, tree, rays, std::string("axis-aligned/") + bvh::toString(s));
        }
    }
}

TEST(BVHQuery, RespectsTMinAndTMax) {
    const Mesh m = scene::uvSphere(2000);
    BVH tree;
    tree.build(m, BuildConfig{});

    const Ray full(Vec3(0.0f, 0.0f, -4.0f), Vec3(0.0f, 0.0f, 1.0f));
    MeshHit hit;
    ASSERT_TRUE(tree.closestHit(m, full, hit));
    const Scalar first = hit.t;

    // Cut the ray short of the near surface.
    const Ray tooShort(full.origin, full.direction, 0.0f, first * 0.5f);
    EXPECT_FALSE(tree.closestHit(m, tooShort, hit));
    EXPECT_FALSE(tree.anyHit(m, tooShort));

    // Start past the near surface: the far surface must be found instead.
    const Ray past(full.origin, full.direction, first + 1e-3f, kInfinity);
    MeshHit farHit;
    MeshHit expected;
    ASSERT_TRUE(bruteForceClosestHit(m, past, expected));
    ASSERT_TRUE(tree.closestHit(m, past, farHit));
    EXPECT_TRUE(nearlyEqual(farHit.t, expected.t, 1e-5f));
    EXPECT_GT(farHit.t, first);
}

// ---------------------------------------------------------------------------
// Degenerate input
// ---------------------------------------------------------------------------

TEST(BVHDegenerate, CoincidentTrianglesTerminateAndValidate) {
    const Mesh m = coincidentTriangles(300);
    for (SplitStrategy s : kStrategies) {
        BuildConfig cfg;
        cfg.strategy = s;
        cfg.maxLeafSize = 1;
        cfg.maxDepth = 32;
        BVH tree;
        tree.build(m, cfg);
        const std::string label = std::string("coincident/") + bvh::toString(s);
        expectValid(tree, m, label);
        // No split can separate identical centroids, so the object-median
        // fallback halves by count: depth ~= ceil(log2(300)).
        EXPECT_LE(tree.stats().maxDepth, 9u) << label;

        MeshHit hit;
        const Ray r(Vec3(0.2f, 0.2f, -1.0f), Vec3(0.0f, 0.0f, 1.0f));
        ASSERT_TRUE(tree.closestHit(m, r, hit)) << label;
        EXPECT_NEAR(hit.t, 1.0f, 1e-5f) << label;
    }
}

TEST(BVHDegenerate, CoincidentTrianglesHitTheDepthLimitGracefully) {
    // maxLeafSize 1 with 3000 identical triangles needs depth 12; cap it at 4
    // and the leaves must simply overflow rather than the build failing.
    const Mesh m = coincidentTriangles(3000);
    BuildConfig cfg;
    cfg.maxLeafSize = 1;
    cfg.maxDepth = 4;
    BVH tree;
    tree.build(m, cfg);
    EXPECT_EQ(tree.stats().maxDepth, 4u);
    EXPECT_EQ(tree.stats().leafCount, 16u);
    expectValid(tree, m, "coincident/depth4");
}

TEST(BVHDegenerate, DegenerateTrianglesDoNotBreakTheTree) {
    const Mesh m = withDegenerateTriangles(400);
    ASSERT_GT(m.countDegenerateTriangles(), 0u);
    const std::vector<Ray> rays = RayGen(41).aroundBounds(m.bounds(), 800);
    for (SplitStrategy s : kStrategies) {
        BuildConfig cfg;
        cfg.strategy = s;
        BVH tree;
        tree.build(m, cfg);
        const std::string label = std::string("degenerate/") + bvh::toString(s);
        expectValid(tree, m, label);
        checkAgainstBruteForce(m, tree, rays, label);
    }
}

TEST(BVHDegenerate, SingleTrianglePerLeafOnATinyMesh) {
    for (std::size_t n = 1; n <= 8; ++n) {
        const Mesh m = scene::triangleSoup(n, 77);
        BuildConfig cfg;
        cfg.maxLeafSize = 1;
        BVH tree;
        tree.build(m, cfg);
        expectValid(tree, m, "tiny mesh " + std::to_string(n));
        EXPECT_EQ(tree.stats().leafCount, n);
    }
}

TEST(BVHDegenerate, DepthCapHoldsUnderAWorstCaseCentroidSplit) {
    const Mesh m = geometricallyClusteredTriangles(120);
    BuildConfig cfg;
    cfg.strategy = SplitStrategy::CentroidMedian;
    cfg.maxLeafSize = 1;
    cfg.maxDepth = bvh::kMaxDepthLimit;
    BVH tree;
    tree.build(m, cfg);

    // Peeling one off per level would need depth 119; the cap stops it at 64 and
    // the deepest leaf absorbs the remainder. This also drives the traversal
    // stack to within one slot of its capacity.
    EXPECT_EQ(tree.stats().maxDepth, bvh::kMaxDepthLimit);
    EXPECT_GT(tree.stats().maxLeafSize, 1u);
    expectValid(tree, m, "geometric/centroid");

    // Object median on the same mesh stays balanced.
    cfg.strategy = SplitStrategy::ObjectMedian;
    BVH balanced;
    balanced.build(m, cfg);
    EXPECT_EQ(balanced.stats().maxDepth, 7u);
    expectValid(balanced, m, "geometric/object");

    // Rays down -x through the stack, hitting every triangle in turn.
    std::vector<Ray> rays;
    for (int i = 0; i < 64; ++i) {
        const Scalar off = static_cast<Scalar>(i) * 1e-6f - 3.2e-5f;
        rays.emplace_back(Vec3(2.0f, off, off), Vec3(-1.0f, 0.0f, 0.0f));
    }
    checkAgainstBruteForce(m, tree, rays, "geometric/centroid");
    checkAgainstBruteForce(m, balanced, rays, "geometric/object");
}

// ---------------------------------------------------------------------------
// Work reduction (counts, not timings)
// ---------------------------------------------------------------------------

TEST(BVHWork, TestsFarFewerTrianglesThanBruteForce) {
    struct Case {
        const char* name;
        Mesh mesh;
    };
    std::vector<Case> cases;
    cases.push_back({"sphere16k", scene::uvSphere(16000)});
    cases.push_back({"soup12k", scene::triangleSoup(12000, 5, 1.0f, 0.08f)});
    cases.push_back({"grid10k", scene::grid(10000)});

    for (const Case& c : cases) {
        const std::vector<Ray> rays = RayGen(101).aroundBounds(c.mesh.bounds(), 200);
        for (SplitStrategy s : kStrategies) {
            BuildConfig cfg;
            cfg.strategy = s;
            BVH tree;
            tree.build(c.mesh, cfg);

            Agreement agg;
            checkAgainstBruteForce(c.mesh, tree, rays, c.name, &agg);
            ASSERT_GT(agg.bruteTriangles, 0u);
            EXPECT_LT(agg.bvhTriangles, agg.bruteTriangles);

            std::cout << "[ work     ] " << c.name << " (" << c.mesh.triangleCount() << " tris, "
                      << bvh::toString(s) << ", leaf " << cfg.maxLeafSize << ") "
                      << rays.size() << " rays: triangles tested "
                      << agg.bvhTriangles << " vs brute force " << agg.bruteTriangles
                      << " (" << static_cast<double>(agg.bruteTriangles) /
                                    static_cast<double>(agg.bvhTriangles)
                      << "x fewer), nodes expanded " << agg.nodesExpanded << ", aabb tests "
                      << agg.aabbTests << ", tree: " << tree.stats().nodeCount << " nodes, depth "
                      << tree.stats().maxDepth << ", mean leaf depth "
                      << tree.stats().meanLeafDepth << ", " << tree.stats().memoryBytes
                      << " bytes\n";
        }
    }
}

TEST(BVHWork, SmallerLeavesTestFewerTriangles) {
    const Mesh m = scene::uvSphere(8000);
    const std::vector<Ray> rays = RayGen(102).aroundBounds(m.bounds(), 200);
    std::uint64_t previous = 0;
    for (std::uint32_t leaf : {32u, 16u, 8u, 4u, 2u, 1u}) {
        BuildConfig cfg;
        cfg.maxLeafSize = leaf;
        BVH tree;
        tree.build(m, cfg);

        QueryStats total;
        for (const Ray& r : rays) {
            MeshHit hit;
            (void)tree.closestHit(m, r, hit, &total);
        }
        if (previous != 0) {
            EXPECT_LT(total.trianglesTested, previous) << "leaf " << leaf;
        }
        previous = total.trianglesTested;
        std::cout << "[ leaf     ] leaf " << leaf << ": triangles tested "
                  << total.trianglesTested << ", nodes expanded " << total.nodesExpanded
                  << ", aabb tests " << total.aabbTests << ", nodes " << tree.stats().nodeCount
                  << ", depth " << tree.stats().maxDepth << "\n";
    }
}

TEST(BVHWork, StatsAreAccumulatedNotReset) {
    const Mesh m = scene::uvSphere(2000);
    BVH tree;
    tree.build(m, BuildConfig{});
    const Ray r(Vec3(0.0f, 0.0f, -4.0f), Vec3(0.0f, 0.0f, 1.0f));

    QueryStats once;
    MeshHit hit;
    ASSERT_TRUE(tree.closestHit(m, r, hit, &once));
    EXPECT_GT(once.trianglesTested, 0u);
    EXPECT_GT(once.nodesExpanded, 0u);
    EXPECT_GT(once.aabbTests, 0u);

    QueryStats twice = once;
    ASSERT_TRUE(tree.closestHit(m, r, hit, &twice));
    EXPECT_EQ(twice.trianglesTested, 2 * once.trianglesTested);
    EXPECT_EQ(twice.nodesExpanded, 2 * once.nodesExpanded);
    EXPECT_EQ(twice.aabbTests, 2 * once.aabbTests);
}

// ---------------------------------------------------------------------------
// Preconditions (Debug only)
// ---------------------------------------------------------------------------

#ifndef NDEBUG
TEST(BVHPreconditionDeathTest, QueryingWithTheWrongMesh) {
    const Mesh built = scene::uvSphere(500);
    const Mesh other = scene::uvSphere(800);
    ASSERT_NE(built.triangleCount(), other.triangleCount());
    BVH tree;
    tree.build(built, BuildConfig{});
    MeshHit hit;
    EXPECT_DEATH((void)tree.closestHit(other, Ray(Vec3(0.0f), Vec3(1.0f, 0.0f, 0.0f)), hit),
                 "different mesh");
}

TEST(BVHPreconditionDeathTest, LeafAccessorsOnTheWrongNodeKind) {
    bvh::BVHNode leaf;
    leaf.count = 3;
    EXPECT_DEATH((void)leaf.leftChild(), "leftChild\\(\\) on a leaf");
    bvh::BVHNode internal;
    EXPECT_DEATH((void)internal.firstPrimitive(), "firstPrimitive\\(\\) on an internal");
}
#endif

// A BVH queried with a mesh other than the one it was built from violates a
// documented precondition. Debug catches it with an assert; Release strips that,
// so the guard has to survive NDEBUG or a shorter mesh reads out of bounds.
#ifdef NDEBUG
TEST(BVHPrecondition, MismatchedMeshReturnsNoHitRatherThanReadingOutOfBounds) {
    const geom::Mesh built = scene::uvSphere(200);
    bvh::BVH tree;
    tree.build(built, bvh::BuildConfig{});

    // Far fewer triangles, so the stored indices would run past its buffers.
    const geom::Mesh other = scene::uvSphere(20);
    ASSERT_NE(other.triangleCount(), built.triangleCount());

    const geom::Ray ray(geom::Vec3(0.0f, 0.0f, -5.0f), geom::Vec3(0.0f, 0.0f, 1.0f));
    geom::MeshHit hit;
    EXPECT_FALSE(tree.closestHit(other, ray, hit));
    EXPECT_FALSE(tree.anyHit(other, ray));
}
#else
TEST(BVHPreconditionDeathTest, MismatchedMeshTripsTheAssert) {
    const geom::Mesh built = scene::uvSphere(200);
    bvh::BVH tree;
    tree.build(built, bvh::BuildConfig{});
    const geom::Mesh other = scene::uvSphere(20);

    const geom::Ray ray(geom::Vec3(0.0f, 0.0f, -5.0f), geom::Vec3(0.0f, 0.0f, 1.0f));
    geom::MeshHit hit;
    EXPECT_DEATH((void)tree.closestHit(other, ray, hit), "different mesh");
}
#endif

// --- validate() must be able to fail -----------------------------------------
//
// Every other structural assertion in this file is `expectValid(...)`. If
// validate() returned true unconditionally, all of them would still pass. These
// corrupt a known-good tree one field at a time and check the specific message.

namespace bvh {
struct BVHTestAccess {
    static std::vector<BVHNode>& nodes(BVH& t) { return t.nodes_; }
    static std::vector<std::uint32_t>& indices(BVH& t) { return t.primitiveIndices_; }
};
}  // namespace bvh

namespace {

bvh::BVH goodTree(const Mesh& m) {
    bvh::BVH t;
    BuildConfig cfg;
    cfg.maxLeafSize = 4;
    t.build(m, cfg);
    return t;
}

std::size_t firstInternal(const bvh::BVH& t) {
    for (std::size_t i = 0; i < t.nodes().size(); ++i) {
        if (!t.nodes()[i].isLeaf()) return i;
    }
    return 0;
}

std::size_t firstLeaf(const bvh::BVH& t) {
    for (std::size_t i = 0; i < t.nodes().size(); ++i) {
        if (t.nodes()[i].isLeaf()) return i;
    }
    return 0;
}

}  // namespace

TEST(BVHValidate, AcceptsAHealthyTree) {
    const Mesh m = scene::uvSphere(500);
    std::string err;
    EXPECT_TRUE(goodTree(m).validate(m, &err)) << err;
    EXPECT_TRUE(err.empty());
}

TEST(BVHValidate, RejectsAMeshOfTheWrongSize) {
    const Mesh m = scene::uvSphere(500);
    const Mesh other = scene::uvSphere(80);
    ASSERT_NE(m.triangleCount(), other.triangleCount());
    std::string err;
    EXPECT_FALSE(goodTree(m).validate(other, &err));
    EXPECT_NE(err.find("index array size"), std::string::npos) << err;
}

TEST(BVHValidate, RejectsADefaultConstructedTreeAgainstANonEmptyMesh) {
    const Mesh m = scene::uvSphere(500);
    std::string err;
    EXPECT_FALSE(bvh::BVH{}.validate(m, &err));
    // The empty index array is caught before the node check below ever runs.
    EXPECT_NE(err.find("index array size"), std::string::npos) << err;
}

// The node check itself, which needs an index array of the right length and no
// nodes -- a state build() cannot produce.
TEST(BVHValidate, RejectsANonEmptyMeshWithNoNodes) {
    const Mesh m = scene::uvSphere(500);
    bvh::BVH t = goodTree(m);
    bvh::BVHTestAccess::nodes(t).clear();
    std::string err;
    EXPECT_FALSE(t.validate(m, &err));
    EXPECT_NE(err.find("no nodes"), std::string::npos) << err;
}

TEST(BVHValidate, RejectsAParentThatNoLongerContainsItsChildren) {
    const Mesh m = scene::uvSphere(500);
    bvh::BVH t = goodTree(m);
    auto& nodes = bvh::BVHTestAccess::nodes(t);
    // Collapse the root to a point, so it contains nothing.
    nodes[0].bounds = AABB(nodes[0].bounds.centroid(), nodes[0].bounds.centroid());
    std::string err;
    EXPECT_FALSE(t.validate(m, &err));
    EXPECT_NE(err.find("contained"), std::string::npos) << err;
}

TEST(BVHValidate, RejectsInternalBoundsThatAreNotTheTightUnion) {
    // The case plain containment cannot catch: bounds that are correct but
    // loose. Traversal would degrade to brute force with no test failing.
    const Mesh m = scene::uvSphere(500);
    bvh::BVH t = goodTree(m);
    auto& nodes = bvh::BVHTestAccess::nodes(t);
    const std::size_t idx = firstInternal(t);
    ASSERT_FALSE(nodes[idx].isLeaf());
    nodes[idx].bounds.extend(nodes[idx].bounds.max + Vec3(10.0f));
    std::string err;
    EXPECT_FALSE(t.validate(m, &err));
    EXPECT_NE(err.find("tight union"), std::string::npos) << err;
}

TEST(BVHValidate, RejectsLeafBoundsThatAreNotTheTightUnion) {
    // maxDepth 0 gives a single-node tree, so the leaf is the root. Inflating a
    // leaf lower down would escape its parent and trip the containment check
    // first, which is correct but tests the wrong branch.
    const Mesh m = scene::uvSphere(500);
    bvh::BVH t;
    BuildConfig cfg;
    cfg.maxDepth = 0;
    t.build(m, cfg);
    auto& nodes = bvh::BVHTestAccess::nodes(t);
    ASSERT_EQ(nodes.size(), 1u);
    ASSERT_TRUE(nodes[0].isLeaf());

    nodes[0].bounds.extend(nodes[0].bounds.max + Vec3(10.0f));
    std::string err;
    EXPECT_FALSE(t.validate(m, &err));
    EXPECT_NE(err.find("tight union"), std::string::npos) << err;
}

TEST(BVHValidate, RejectsAChildIndexPointingBackwards) {
    const Mesh m = scene::uvSphere(500);
    bvh::BVH t = goodTree(m);
    auto& nodes = bvh::BVHTestAccess::nodes(t);
    nodes[firstInternal(t)].leftOrFirst = 0;
    std::string err;
    EXPECT_FALSE(t.validate(m, &err));
    EXPECT_NE(err.find("exceed the parent"), std::string::npos) << err;
}

TEST(BVHValidate, RejectsADuplicatedPrimitiveIndex) {
    const Mesh m = scene::uvSphere(500);
    bvh::BVH t = goodTree(m);
    auto& idx = bvh::BVHTestAccess::indices(t);
    ASSERT_GE(idx.size(), 2u);
    idx[1] = idx[0];
    std::string err;
    EXPECT_FALSE(t.validate(m, &err));
    EXPECT_NE(err.find("referenced by two leaves"), std::string::npos) << err;
}

TEST(BVHValidate, RejectsAnOversizedLeafBelowTheDepthCap) {
    // count > maxLeafSize is legal only when the depth cap cut the recursion.
    const Mesh m = scene::uvSphere(500);
    bvh::BVH t = goodTree(m);
    auto& nodes = bvh::BVHTestAccess::nodes(t);
    const std::size_t idx = firstLeaf(t);
    nodes[idx].count = t.config().maxLeafSize + 1;
    std::string err;
    EXPECT_FALSE(t.validate(m, &err));
    EXPECT_NE(err.find("maxLeafSize"), std::string::npos) << err;
}

// --- The conservative widening and the prune slack ---------------------------
//
// A ray straight down the pole of a UV sphere hits the apex vertex, which the
// whole first triangle ring shares. Each of those triangles computes a slightly
// different t for that one point. Two separate failures follow, and the two
// tests below pin one each.
//
// 1. The slab test's tExit can fall a ULP short, so a ray grazing a shared face
//    misses both children. kRayBoxWidening fixes that (geometry/aabb.hpp).
// 2. The leaf holding the closest triangle can report a box tEnter slightly
//    ABOVE that triangle's t, and traversal culls it. kBestPruneSlack fixes
//    that (bvh.cpp) -- it is the larger of the two effects, and it is the one
//    that scales with tessellation.
//
// The six rays straight through the centre are the entire failure set; the
// offset rays in axisAlignedRays never trip it, so these tests stay small
// enough to brute-force against an 80k-triangle mesh.
namespace {

std::vector<Ray> poleRays(const AABB& box) {
    const Vec3 c = box.centroid();
    const Scalar reach = length(box.diagonal()) + Scalar(1);
    std::vector<Ray> rays;
    for (int axis = 0; axis < 3; ++axis) {
        for (int sign = -1; sign <= 1; sign += 2) {
            Vec3 origin = c;
            Vec3 dir(Scalar(0));
            origin[axis] -= static_cast<Scalar>(sign) * reach;
            dir[axis] = static_cast<Scalar>(sign);
            rays.emplace_back(origin, dir);
        }
    }
    return rays;
}

}  // namespace

// 80k and not 8k: the slack a pole ray needs grows with tessellation (the
// measurement table is in bvh.cpp). At 8k the old box-sized margin already
// passed, which is exactly why this test used to be green while the bug was
// live.
TEST(BVHWidening, MatchesTheOracleOnPoleRays) {
    const Mesh m = scene::uvSphere(80000);
    const std::vector<Ray> rays = poleRays(m.bounds());
    BVH tree;
    tree.build(m, BuildConfig{});

    int disagreements = 0;
    for (std::size_t i = 0; i < rays.size(); ++i) {
        MeshHit expected;
        MeshHit actual;
        const bool e = bruteForceClosestHit(m, rays[i], expected);
        const bool a = tree.closestHit(m, rays[i], actual);
        ASSERT_EQ(e, a) << "ray " << i;
        ASSERT_TRUE(e) << "ray " << i << " should hit the sphere";
        // t and not the index: several triangles share the apex and can
        // report the same t, and picking either of those is a legal tie.
        if (expected.t != actual.t) ++disagreements;
    }

#if BVH_CONSERVATIVE_RAY_BOX
    EXPECT_EQ(disagreements, 0)
        << "traversal culled a node holding a closer triangle; kBestPruneSlack "
           "in bvh.cpp is too small for this tessellation";
#else
    // The other half of the demonstration. Measured with the exact test: 2 of
    // these 6 rays return a farther triangle. If this reaches 0, the widening
    // and the slack have stopped earning their cost and Phase 9 should say so.
    EXPECT_GT(disagreements, 0)
        << "expected the exact test to miss at least one closer triangle on a "
           "shared-vertex hit; if it no longer does, re-examine whether the "
           "conservative path is still needed";
#endif
}

// The named case the failure was first isolated on. Kept separate from the
// sweep above so a regression names one ray rather than a count: with the
// exact test this returns triangle 8063 at 0x1.bb509ap+1 where brute force
// finds 8043 at 0x1.bb508ap+1 -- farther, so a genuine miss.
TEST(BVHWidening, PoleRayRegression) {
    const Mesh m = scene::uvSphere(8000);
    const Ray ray(Vec3(Scalar(0), -0x1.1da852p+2f, Scalar(0)),
                  Vec3(Scalar(0), Scalar(1), Scalar(0)));
    BVH tree;
    tree.build(m, BuildConfig{});

    MeshHit expected;
    MeshHit actual;
    ASSERT_TRUE(bruteForceClosestHit(m, ray, expected));
    ASSERT_TRUE(tree.closestHit(m, ray, actual));

#if BVH_CONSERVATIVE_RAY_BOX
    EXPECT_EQ(actual.t, expected.t);
    EXPECT_EQ(actual.triangleIndex, expected.triangleIndex);
#else
    EXPECT_GT(actual.t, expected.t) << "the exact test is expected to miss this one";
#endif
}

// closestHit and anyHit are const and touch no mutable state -- the traversal
// stack is a local. This is what lets Phase 9 fan a benchmark across threads
// with one shared tree, so it is worth pinning rather than assuming.
TEST(BVHConcurrency, ConstQueriesAgreeAcrossThreads) {
    const Mesh m = scene::uvSphere(2000);
    BVH tree;
    tree.build(m, BuildConfig{});

    RayGen gen(20260921u);
    const std::vector<Ray> rays = gen.aroundBounds(m.bounds(), 400);

    std::vector<MeshHit> reference(rays.size());
    std::vector<char> refHit(rays.size(), 0);
    for (std::size_t i = 0; i < rays.size(); ++i) {
        refHit[i] = tree.closestHit(m, rays[i], reference[i]) ? 1 : 0;
    }

    constexpr int kThreads = 4;
    std::vector<char> ok(static_cast<std::size_t>(kThreads), 0);
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t] {
            bool good = true;
            for (std::size_t i = 0; i < rays.size(); ++i) {
                MeshHit h;
                const bool hit = tree.closestHit(m, rays[i], h);
                if (hit != (refHit[i] != 0)) good = false;
                if (hit && (h.t != reference[i].t ||
                            h.triangleIndex != reference[i].triangleIndex)) {
                    good = false;
                }
                if (tree.anyHit(m, rays[i]) != (refHit[i] != 0)) good = false;
            }
            ok[static_cast<std::size_t>(t)] = good ? 1 : 0;
        });
    }
    for (std::thread& w : workers) w.join();
    for (int t = 0; t < kThreads; ++t) {
        EXPECT_EQ(ok[static_cast<std::size_t>(t)], 1) << "thread " << t << " disagreed";
    }
}
