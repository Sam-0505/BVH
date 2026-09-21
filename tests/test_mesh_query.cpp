#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "geometry/mesh_query.hpp"

using namespace geom;

namespace {

// Three parallel quads at z = 1, 2, 3, each two triangles, all covering the
// same xy region. A ray down -z must report the nearest.
Mesh stackedQuads() {
    std::vector<Vec3> p;
    std::vector<std::uint32_t> idx;
    for (int layer = 0; layer < 3; ++layer) {
        const Scalar z = static_cast<Scalar>(layer + 1);
        const auto base = static_cast<std::uint32_t>(p.size());
        p.push_back(Vec3(-1.0f, -1.0f, z));
        p.push_back(Vec3(1.0f, -1.0f, z));
        p.push_back(Vec3(1.0f, 1.0f, z));
        p.push_back(Vec3(-1.0f, 1.0f, z));
        idx.insert(idx.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return Mesh(std::move(p), std::move(idx));
}

Mesh singleTriangle() {
    return Mesh({{0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}}, {0, 1, 2});
}

}  // namespace

TEST(BruteForce, FindsTheNearestOfSeveralCandidates) {
    const Mesh m = stackedQuads();
    // Travelling +z from behind everything: the z = 1 layer is nearest.
    const Ray r(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    MeshHit hit;
    ASSERT_TRUE(bruteForceClosestHit(m, r, hit));
    EXPECT_NEAR(hit.t, 6.0f, 1e-4f);
    EXPECT_LT(hit.triangleIndex, 2u) << "expected a triangle from the z=1 layer";
}

TEST(BruteForce, FindsTheNearestFromTheOtherDirection) {
    const Mesh m = stackedQuads();
    // Travelling -z from in front: now the z = 3 layer is nearest.
    const Ray r(Vec3(0.0f, 0.0f, 5.0f), Vec3(0.0f, 0.0f, -1.0f));
    MeshHit hit;
    ASSERT_TRUE(bruteForceClosestHit(m, r, hit));
    EXPECT_NEAR(hit.t, 2.0f, 1e-4f);
    EXPECT_GE(hit.triangleIndex, 4u) << "expected a triangle from the z=3 layer";
}

TEST(BruteForce, ReportsNoHitWhenTheRayMisses) {
    const Mesh m = stackedQuads();
    MeshHit hit;
    EXPECT_FALSE(bruteForceClosestHit(m, Ray(Vec3(10.0f, 10.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f)), hit));
    EXPECT_FALSE(hit.valid());
    // Pointing away from all the geometry.
    EXPECT_FALSE(bruteForceClosestHit(m, Ray(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, -1.0f)), hit));
}

TEST(BruteForce, EmptyMeshIsNeverHit) {
    const Mesh empty;
    MeshHit hit;
    EXPECT_FALSE(bruteForceClosestHit(empty, Ray(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f)), hit));
    EXPECT_FALSE(bruteForceAnyHit(empty, Ray(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f))));
}

TEST(BruteForce, HitCarriesUsableBarycentrics) {
    const Mesh m = singleTriangle();
    const Ray r(Vec3(0.5f, 0.5f, -1.0f), Vec3(0.0f, 0.0f, 1.0f));
    MeshHit hit;
    ASSERT_TRUE(bruteForceClosestHit(m, r, hit));
    EXPECT_EQ(hit.triangleIndex, 0u);
    // Reconstruct the point from the triangle and compare with the ray.
    const Triangle tri = m.triangle(hit.triangleIndex);
    const Vec3 fromBary = tri.v0 + tri.edge01() * hit.u + tri.edge02() * hit.v;
    EXPECT_TRUE(nearlyEqual(fromBary, r.at(hit.t), 1e-4f));
}

TEST(BruteForce, RespectsTheRayRange) {
    const Mesh m = stackedQuads();
    MeshHit hit;
    // All three layers are past tMax.
    EXPECT_FALSE(bruteForceClosestHit(m, Ray(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 0.0f, 3.0f), hit));
    // Skipping past the first two layers finds the third.
    ASSERT_TRUE(bruteForceClosestHit(m, Ray(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 7.5f, 100.0f), hit));
    EXPECT_NEAR(hit.t, 8.0f, 1e-4f);
}

TEST(BruteForce, ClosestHitTestsEveryTriangle) {
    // The baseline the BVH is measured against: O(n) with no pruning of
    // triangles, only of hits. If this ever stops testing everything, the
    // speedup numbers in Phase 9 become meaningless.
    const Mesh m = stackedQuads();
    QueryStats stats;
    MeshHit hit;
    bruteForceClosestHit(m, Ray(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f)), hit, &stats);
    EXPECT_EQ(stats.trianglesTested, m.triangleCount());
    EXPECT_EQ(stats.nodesExpanded, 0u) << "brute force visits no nodes by definition";
}

TEST(BruteForce, AnyHitStopsEarlyAndCountsOnlyWhatItTested) {
    const Mesh m = stackedQuads();
    QueryStats stats;
    ASSERT_TRUE(bruteForceAnyHit(m, Ray(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f)), &stats));
    EXPECT_GT(stats.trianglesTested, 0u);
    EXPECT_LT(stats.trianglesTested, m.triangleCount())
        << "any-hit should have stopped before examining every triangle";
}

TEST(BruteForce, AnyHitCountsEverythingOnAMiss) {
    const Mesh m = stackedQuads();
    QueryStats stats;
    ASSERT_FALSE(bruteForceAnyHit(m, Ray(Vec3(10.0f, 10.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f)), &stats));
    EXPECT_EQ(stats.trianglesTested, m.triangleCount());
}

TEST(BruteForce, AnyHitAgreesWithClosestHitOnWhetherThereIsAHit) {
    // The two entry points must never disagree about existence, only about
    // which triangle. This is the invariant a BVH must also preserve.
    const Mesh m = stackedQuads();
    std::mt19937 rng(0x0A17B3C5u);
    std::uniform_real_distribution<float> u(-3.0f, 3.0f);
    for (int i = 0; i < 5000; ++i) {
        const Ray r(Vec3(u(rng), u(rng), -5.0f), Vec3(u(rng) * 0.2f, u(rng) * 0.2f, 1.0f), 0.0f, 50.0f);
        MeshHit hit;
        const bool closest = bruteForceClosestHit(m, r, hit);
        const bool any = bruteForceAnyHit(m, r);
        ASSERT_EQ(closest, any) << "closest/any disagreement on ray " << i;
    }
}

TEST(BruteForce, ClosestHitIsTheMinimumOverAllTriangles) {
    // Independent recomputation of the answer: scan every triangle with the
    // primitive routine and take the minimum t, then compare. This is the
    // oracle-for-the-oracle -- if brute force is wrong, every BVH comparison
    // built on it is wrong too.
    const Mesh m = stackedQuads();
    std::mt19937 rng(0x5CA71234u);
    std::uniform_real_distribution<float> u(-3.0f, 3.0f);
    for (int i = 0; i < 5000; ++i) {
        const Ray r(Vec3(u(rng), u(rng), -5.0f), Vec3(u(rng) * 0.3f, u(rng) * 0.3f, 1.0f), 0.0f, 50.0f);

        Scalar expected = kInfinity;
        for (std::size_t tri = 0; tri < m.triangleCount(); ++tri) {
            TriangleHit th;
            if (intersectRayTriangle(m.triangle(tri), r, th) && th.t < expected) expected = th.t;
        }

        MeshHit hit;
        const bool got = bruteForceClosestHit(m, r, hit);
        ASSERT_EQ(got, expected != kInfinity) << "existence mismatch on ray " << i;
        if (got) ASSERT_NEAR(hit.t, expected, 1e-5f) << "distance mismatch on ray " << i;
    }
}
