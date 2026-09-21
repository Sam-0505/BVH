#include <gtest/gtest.h>

#include <cmath>

#include "scene/procedural.hpp"

using namespace geom;
using scene::grid;
using scene::triangleSoup;
using scene::uvSphere;

namespace {

// The generators round to whatever resolution their topology admits, so the
// contract is "near the target", not "exactly".
void expectNearTarget(std::size_t actual, std::size_t target) {
    EXPECT_GE(actual, target / 2) << "target " << target;
    EXPECT_LE(actual, target * 2) << "target " << target;
}

}  // namespace

TEST(Procedural, SphereHasRoughlyTheRequestedTriangleCount) {
    for (std::size_t target : {64u, 1000u, 20000u}) {
        const Mesh m = uvSphere(target);
        expectNearTarget(m.triangleCount(), target);
    }
}

TEST(Procedural, SpherePointsLieOnTheSphere) {
    const Scalar radius = 2.5f;
    const Mesh m = uvSphere(2000, radius);
    for (const Vec3& p : m.positions()) {
        EXPECT_NEAR(length(p), radius, 1e-4f);
    }
    EXPECT_TRUE(nearlyEqual(m.bounds().min, Vec3(-radius), 1e-2f));
    EXPECT_TRUE(nearlyEqual(m.bounds().max, Vec3(radius), 1e-2f));
}

TEST(Procedural, SphereIsClosedAndNonDegenerate) {
    const Mesh m = uvSphere(2000);
    EXPECT_EQ(m.countDegenerateTriangles(), 0u);
    // Every vertex is shared, so a closed shell has far fewer than 3 per triangle.
    EXPECT_LT(m.vertexCount(), m.triangleCount());
}

TEST(Procedural, GridIsFlatInY) {
    const Mesh m = grid(5000, 3.0f);
    expectNearTarget(m.triangleCount(), 5000);
    EXPECT_FLOAT_EQ(m.bounds().min.y, 0.0f);
    EXPECT_FLOAT_EQ(m.bounds().max.y, 0.0f);
    EXPECT_FLOAT_EQ(m.bounds().min.x, -3.0f);
    EXPECT_FLOAT_EQ(m.bounds().max.z, 3.0f);
    EXPECT_EQ(m.countDegenerateTriangles(), 0u);
}

TEST(Procedural, SoupHasExactlyTheRequestedTriangleCount) {
    const Mesh m = triangleSoup(1234);
    EXPECT_EQ(m.triangleCount(), 1234u);
    EXPECT_EQ(m.vertexCount(), 1234u * 3);
}

TEST(Procedural, SoupIsDeterministicForAGivenSeed) {
    const Mesh a = triangleSoup(500, 7);
    const Mesh b = triangleSoup(500, 7);
    const Mesh c = triangleSoup(500, 8);
    ASSERT_EQ(a.positions().size(), b.positions().size());
    EXPECT_TRUE(a.positions() == b.positions());
    EXPECT_FALSE(a.positions() == c.positions());
}

TEST(Procedural, AllGeneratedVerticesAreFinite) {
    for (const Mesh& m : {uvSphere(500), grid(500), triangleSoup(500)}) {
        for (const Vec3& p : m.positions()) {
            ASSERT_TRUE(isFinite(p));
        }
    }
}
