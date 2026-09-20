#include <gtest/gtest.h>

#include "geometry/plane.hpp"

using namespace geom;

TEST(Plane, FromPointNormalPassesThroughPoint) {
    const Vec3 p(1.0f, 2.0f, 3.0f);
    const Plane pl = Plane::fromPointNormal(p, Vec3(0.0f, 5.0f, 0.0f));
    // Normal is normalised by the factory.
    EXPECT_TRUE(nearlyEqual(pl.normal, Vec3(0.0f, 1.0f, 0.0f)));
    // The defining point lies on the plane.
    EXPECT_NEAR(pl.signedDistance(p), 0.0f, 1e-6f);
}

TEST(Plane, SignedDistanceSignFollowsNormal) {
    const Plane ground = Plane::fromPointNormal(Vec3(0.0f), Vec3(0.0f, 1.0f, 0.0f));
    EXPECT_NEAR(ground.signedDistance(Vec3(0.0f, 5.0f, 0.0f)), 5.0f, 1e-6f);
    EXPECT_NEAR(ground.signedDistance(Vec3(0.0f, -5.0f, 0.0f)), -5.0f, 1e-6f);
    EXPECT_NEAR(ground.signedDistance(Vec3(100.0f, 0.0f, -100.0f)), 0.0f, 1e-6f);
}

TEST(Plane, FromThreePointsIsCounterClockwise) {
    // Triangle in the XZ plane wound CCW when viewed from +Y.
    const Plane pl = Plane::fromPoints(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f), Vec3(1.0f, 0.0f, 0.0f));
    EXPECT_TRUE(nearlyEqual(pl.normal, Vec3(0.0f, 1.0f, 0.0f), 1e-5f));
    EXPECT_NEAR(pl.signedDistance(Vec3(0.5f, 0.0f, 0.25f)), 0.0f, 1e-6f);
}

TEST(Plane, CollinearPointsGiveDegeneratePlaneNotNaN) {
    const Plane pl = Plane::fromPoints(Vec3(0.0f), Vec3(1.0f, 1.0f, 1.0f), Vec3(2.0f, 2.0f, 2.0f));
    EXPECT_TRUE(pl.isDegenerate());
    EXPECT_TRUE(isFinite(pl.normal));
}

TEST(Plane, CoincidentPointsGiveDegeneratePlane) {
    const Plane pl = Plane::fromPoints(Vec3(1.0f), Vec3(1.0f), Vec3(1.0f));
    EXPECT_TRUE(pl.isDegenerate());
}
