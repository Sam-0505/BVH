#include <gtest/gtest.h>

#include "geometry/triangle.hpp"

using namespace geom;

namespace {
// Unit right triangle in the z = 0 plane, wound CCW viewed from +Z.
Triangle unitTriangle() {
    return Triangle(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f));
}
}  // namespace

TEST(Triangle, Edges) {
    const Triangle t = unitTriangle();
    EXPECT_EQ(t.edge01(), Vec3(1.0f, 0.0f, 0.0f));
    EXPECT_EQ(t.edge02(), Vec3(0.0f, 1.0f, 0.0f));
}

TEST(Triangle, NormalFollowsCounterClockwiseWinding) {
    const Triangle t = unitTriangle();
    EXPECT_TRUE(nearlyEqual(t.normal(), Vec3(0.0f, 0.0f, 1.0f)));

    // Reversing the winding flips the normal.
    const Triangle flipped(t.v0, t.v2, t.v1);
    EXPECT_TRUE(nearlyEqual(flipped.normal(), Vec3(0.0f, 0.0f, -1.0f)));
}

TEST(Triangle, UnnormalizedNormalHasMagnitudeTwiceTheArea) {
    const Triangle t = unitTriangle();
    EXPECT_NEAR(length(t.normalUnnormalized()), 2.0f * t.area(), 1e-6f);
}

TEST(Triangle, Area) {
    // Right triangle with legs 1 and 1.
    EXPECT_NEAR(unitTriangle().area(), 0.5f, 1e-6f);

    // Scaling both legs by 3 scales the area by 9.
    const Triangle bigger(Vec3(0.0f), Vec3(3.0f, 0.0f, 0.0f), Vec3(0.0f, 3.0f, 0.0f));
    EXPECT_NEAR(bigger.area(), 4.5f, 1e-6f);
}

TEST(Triangle, AreaIsIndependentOfPosition) {
    const Triangle t = unitTriangle();
    const Vec3 shift(100.0f, -50.0f, 25.0f);
    const Triangle moved(t.v0 + shift, t.v1 + shift, t.v2 + shift);
    EXPECT_NEAR(moved.area(), t.area(), 1e-5f);
}

TEST(Triangle, Centroid) {
    const Triangle t = unitTriangle();
    EXPECT_TRUE(nearlyEqual(t.centroid(), Vec3(1.0f / 3.0f, 1.0f / 3.0f, 0.0f)));

    // Centroid of a triangle with all vertices equal is that vertex.
    const Vec3 p(5.0f, 6.0f, 7.0f);
    EXPECT_TRUE(nearlyEqual(Triangle(p, p, p).centroid(), p));
}

TEST(Triangle, BoundsCoverAllVertices) {
    const Triangle t(Vec3(-1.0f, 5.0f, 2.0f), Vec3(3.0f, -2.0f, 8.0f), Vec3(0.0f, 1.0f, -4.0f));
    const AABB b = t.bounds();
    EXPECT_EQ(b.min, Vec3(-1.0f, -2.0f, -4.0f));
    EXPECT_EQ(b.max, Vec3(3.0f, 5.0f, 8.0f));

    EXPECT_TRUE(b.contains(t.v0));
    EXPECT_TRUE(b.contains(t.v1));
    EXPECT_TRUE(b.contains(t.v2));
}

TEST(Triangle, BoundsAreTight) {
    // Every face of the bound must touch a vertex -- the extremes of a triangle
    // are always at its vertices, so no slack is acceptable.
    const Triangle t = unitTriangle();
    const AABB b = t.bounds();
    EXPECT_EQ(b.min, Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(b.max, Vec3(1.0f, 1.0f, 0.0f));
    // Flat in z, so zero volume but non-zero surface area.
    EXPECT_FLOAT_EQ(b.volume(), 0.0f);
    EXPECT_FLOAT_EQ(b.surfaceArea(), 2.0f);
}

TEST(Triangle, BoundsOfDegenerateTriangleAreStillValid) {
    // A BVH must be able to carry degenerate triangles without special-casing.
    const Triangle collinear(Vec3(0.0f), Vec3(1.0f, 1.0f, 1.0f), Vec3(2.0f, 2.0f, 2.0f));
    const AABB b = collinear.bounds();
    EXPECT_FALSE(b.isEmpty());
    EXPECT_EQ(b.min, Vec3(0.0f));
    EXPECT_EQ(b.max, Vec3(2.0f));
}

TEST(Triangle, DegenerateDetectionCollinear) {
    const Triangle collinear(Vec3(0.0f), Vec3(1.0f, 1.0f, 1.0f), Vec3(2.0f, 2.0f, 2.0f));
    EXPECT_TRUE(collinear.isDegenerate());
    // Normal must be zero rather than NaN.
    EXPECT_EQ(collinear.normal(), Vec3(0.0f));
    EXPECT_TRUE(isFinite(collinear.normal()));
    EXPECT_NEAR(collinear.area(), 0.0f, 1e-6f);
}

TEST(Triangle, DegenerateDetectionDuplicateVertices) {
    const Vec3 p(1.0f, 2.0f, 3.0f);
    EXPECT_TRUE(Triangle(p, p, p).isDegenerate());
    EXPECT_TRUE(Triangle(p, p, Vec3(4.0f, 5.0f, 6.0f)).isDegenerate());
}

TEST(Triangle, NonDegenerateTriangleIsNotFlagged) {
    EXPECT_FALSE(unitTriangle().isDegenerate());
}
