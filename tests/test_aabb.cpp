#include <gtest/gtest.h>

#include <cmath>

#include "geometry/aabb.hpp"

using namespace geom;

TEST(AABB, DefaultConstructedIsEmpty) {
    const AABB b;
    EXPECT_TRUE(b.isEmpty());
    // The inverted-sentinel representation that makes extend() branch-free.
    EXPECT_EQ(b.min, Vec3(kInfinity));
    EXPECT_EQ(b.max, Vec3(-kInfinity));
    // An empty box must report zero measure, not a negative or NaN one.
    EXPECT_FLOAT_EQ(b.surfaceArea(), 0.0f);
    EXPECT_FLOAT_EQ(b.volume(), 0.0f);
}

TEST(AABB, ExtendingEmptyBoxWithPointGivesDegenerateBox) {
    AABB b;
    const Vec3 p(1.0f, 2.0f, 3.0f);
    b.extend(p);
    EXPECT_FALSE(b.isEmpty());
    EXPECT_EQ(b.min, p);
    EXPECT_EQ(b.max, p);
    EXPECT_FLOAT_EQ(b.volume(), 0.0f);
    // A point is contained in its own degenerate box.
    EXPECT_TRUE(b.contains(p));
}

TEST(AABB, ExtendGrowsToCoverAllPoints) {
    AABB b;
    b.extend(Vec3(1.0f, 2.0f, 3.0f));
    b.extend(Vec3(-4.0f, 5.0f, -6.0f));
    EXPECT_EQ(b.min, Vec3(-4.0f, 2.0f, -6.0f));
    EXPECT_EQ(b.max, Vec3(1.0f, 5.0f, 3.0f));
}

TEST(AABB, UnionOfTwoBoxes) {
    const AABB a(Vec3(0.0f), Vec3(1.0f));
    const AABB b(Vec3(2.0f), Vec3(3.0f));
    const AABB u = merge(a, b);
    EXPECT_EQ(u.min, Vec3(0.0f));
    EXPECT_EQ(u.max, Vec3(3.0f));
}

TEST(AABB, UnionWithEmptyBoxIsIdentity) {
    // Relied on by bottom-up bounds propagation in BVH construction: merging an
    // empty child must not corrupt the parent.
    const AABB a(Vec3(1.0f, 2.0f, 3.0f), Vec3(4.0f, 5.0f, 6.0f));
    const AABB empty;
    EXPECT_EQ(merge(a, empty), a);
    EXPECT_EQ(merge(empty, a), a);
    EXPECT_TRUE(merge(empty, empty).isEmpty());
}

TEST(AABB, SurfaceArea) {
    // A 1x2x3 box: 2*(1*2 + 2*3 + 3*1) = 22.
    const AABB b(Vec3(0.0f), Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_FLOAT_EQ(b.surfaceArea(), 22.0f);
    EXPECT_FLOAT_EQ(b.volume(), 6.0f);
}

TEST(AABB, SurfaceAreaOfFlatBoxIsTwiceTheFace) {
    // A zero-thickness box still has real surface area -- it is a valid
    // degenerate primitive, unlike an empty box.
    const AABB flat(Vec3(0.0f), Vec3(2.0f, 3.0f, 0.0f));
    EXPECT_FALSE(flat.isEmpty());
    EXPECT_FLOAT_EQ(flat.surfaceArea(), 12.0f);
    EXPECT_FLOAT_EQ(flat.volume(), 0.0f);
}

TEST(AABB, CentroidAndDiagonal) {
    const AABB b(Vec3(-1.0f, 0.0f, 2.0f), Vec3(3.0f, 4.0f, 8.0f));
    EXPECT_TRUE(nearlyEqual(b.centroid(), Vec3(1.0f, 2.0f, 5.0f)));
    EXPECT_EQ(b.diagonal(), Vec3(4.0f, 4.0f, 6.0f));
}

TEST(AABB, LongestAxis) {
    EXPECT_EQ(AABB(Vec3(0.0f), Vec3(5.0f, 1.0f, 2.0f)).longestAxis(), 0);
    EXPECT_EQ(AABB(Vec3(0.0f), Vec3(1.0f, 5.0f, 2.0f)).longestAxis(), 1);
    EXPECT_EQ(AABB(Vec3(0.0f), Vec3(1.0f, 2.0f, 5.0f)).longestAxis(), 2);
    // Cube: deterministic tie-break to axis 0.
    EXPECT_EQ(AABB(Vec3(0.0f), Vec3(1.0f)).longestAxis(), 0);
}

TEST(AABB, ContainsPointIsInclusiveOfBoundary) {
    const AABB b(Vec3(0.0f), Vec3(2.0f));
    EXPECT_TRUE(b.contains(Vec3(1.0f)));
    EXPECT_TRUE(b.contains(Vec3(0.0f)));   // min corner
    EXPECT_TRUE(b.contains(Vec3(2.0f)));   // max corner
    EXPECT_TRUE(b.contains(Vec3(0.0f, 1.0f, 2.0f)));  // on a face
    EXPECT_FALSE(b.contains(Vec3(-0.01f, 1.0f, 1.0f)));
    EXPECT_FALSE(b.contains(Vec3(2.01f, 1.0f, 1.0f)));
}

TEST(AABB, ContainsBox) {
    const AABB outer(Vec3(0.0f), Vec3(10.0f));
    EXPECT_TRUE(outer.contains(AABB(Vec3(1.0f), Vec3(9.0f))));
    EXPECT_TRUE(outer.contains(outer));
    EXPECT_FALSE(outer.contains(AABB(Vec3(-1.0f), Vec3(5.0f))));
    // An empty box is trivially contained in anything.
    EXPECT_TRUE(outer.contains(AABB{}));
}

TEST(AABB, IntersectsBox) {
    const AABB a(Vec3(0.0f), Vec3(2.0f));
    EXPECT_TRUE(a.intersects(AABB(Vec3(1.0f), Vec3(3.0f))));   // overlapping
    EXPECT_TRUE(a.intersects(AABB(Vec3(0.5f), Vec3(1.5f))));   // contained
    EXPECT_FALSE(a.intersects(AABB(Vec3(3.0f), Vec3(4.0f))));  // disjoint
    // Empty boxes intersect nothing.
    EXPECT_FALSE(a.intersects(AABB{}));
}

TEST(AABB, TouchingBoxesCountAsIntersecting) {
    // Deliberate convention: a false positive costs a narrow-phase test, a
    // false negative is a missed collision.
    const AABB a(Vec3(0.0f), Vec3(1.0f));
    const AABB b(Vec3(1.0f), Vec3(2.0f));
    EXPECT_TRUE(a.intersects(b));
}

TEST(AABB, IntersectionRegion) {
    const AABB a(Vec3(0.0f), Vec3(2.0f));
    const AABB b(Vec3(1.0f), Vec3(3.0f));
    const AABB i = intersection(a, b);
    EXPECT_EQ(i.min, Vec3(1.0f));
    EXPECT_EQ(i.max, Vec3(2.0f));

    // Disjoint boxes produce an empty intersection, not garbage.
    EXPECT_TRUE(intersection(AABB(Vec3(0.0f), Vec3(1.0f)), AABB(Vec3(5.0f), Vec3(6.0f))).isEmpty());
}

TEST(AABB, OffsetNormalisesPositionWithinBox) {
    const AABB b(Vec3(0.0f), Vec3(4.0f, 2.0f, 8.0f));
    EXPECT_TRUE(nearlyEqual(b.offset(Vec3(2.0f, 1.0f, 4.0f)), Vec3(0.5f)));
    EXPECT_TRUE(nearlyEqual(b.offset(b.min), Vec3(0.0f)));
    EXPECT_TRUE(nearlyEqual(b.offset(b.max), Vec3(1.0f)));
}

TEST(AABB, OffsetOnDegenerateAxisDoesNotDivideByZero) {
    // SAH binning hits this whenever all centroids share a coordinate.
    const AABB flat(Vec3(0.0f), Vec3(4.0f, 0.0f, 4.0f));
    const Vec3 o = flat.offset(Vec3(2.0f, 0.0f, 2.0f));
    EXPECT_TRUE(isFinite(o));
    EXPECT_FLOAT_EQ(o.y, 0.0f);
}

// --- Ray intersection --------------------------------------------------------

TEST(AABBRay, HitsBoxInFront) {
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    const Ray r(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    Scalar t = -1.0f;
    ASSERT_TRUE(intersectRay(b, r, &t));
    EXPECT_NEAR(t, 4.0f, 1e-4f);
}

TEST(AABBRay, MissesBoxToTheSide) {
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    const Ray r(Vec3(5.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    EXPECT_FALSE(intersectRay(b, r));
}

TEST(AABBRay, MissesBoxBehindOrigin) {
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    // Pointing away from the box.
    const Ray r(Vec3(0.0f, 0.0f, 5.0f), Vec3(0.0f, 0.0f, 1.0f));
    EXPECT_FALSE(intersectRay(b, r));
}

TEST(AABBRay, OriginInsideBoxReportsTMin) {
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    const Ray r(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f));
    Scalar t = -1.0f;
    ASSERT_TRUE(intersectRay(b, r, &t));
    // Entry distance is clamped to tMin, not a negative backward distance.
    EXPECT_FLOAT_EQ(t, 0.0f);
}

TEST(AABBRay, RespectsTMaxRange) {
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    // The box starts at t=4 but the ray is cut short at t=2.
    const Ray shortRay(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 0.0f, 2.0f);
    EXPECT_FALSE(intersectRay(b, shortRay));

    const Ray longEnough(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 0.0f, 10.0f);
    EXPECT_TRUE(intersectRay(b, longEnough));
}

TEST(AABBRay, RespectsTMinRange) {
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    // Box spans t in [4,6]; starting the range past it must miss.
    const Ray r(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f), 7.0f, 100.0f);
    EXPECT_FALSE(intersectRay(b, r));
}

TEST(AABBRay, AxisParallelRayOutsideSlabMisses) {
    // Zero direction component -> infinite reciprocal. Must not produce a NaN
    // that accidentally reports a hit.
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    const Ray r(Vec3(0.0f, 5.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    EXPECT_FALSE(intersectRay(b, r));
}

TEST(AABBRay, RayLyingExactlyOnSlabBoundaryIsHandled) {
    // The 0 * inf == NaN case: origin sits exactly on the y = 1 face plane and
    // the ray has no y component. The NaN-tolerant comparisons must treat the
    // degenerate axis as a no-op and still report the hit.
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    const Ray r(Vec3(0.0f, 1.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    Scalar t = -1.0f;
    EXPECT_TRUE(intersectRay(b, r, &t));
    EXPECT_TRUE(std::isfinite(t));
}

TEST(AABBRay, NegativeDirectionComponentsHit) {
    // Exercises the tNear/tFar swap for negative reciprocals.
    const AABB b(Vec3(-1.0f), Vec3(1.0f));
    const Ray r(Vec3(5.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f));
    Scalar t = -1.0f;
    ASSERT_TRUE(intersectRay(b, r, &t));
    EXPECT_NEAR(t, 4.0f, 1e-4f);
}

TEST(AABBRay, DiagonalRayHitsCorner) {
    const AABB b(Vec3(0.0f), Vec3(1.0f));
    const Ray r(Vec3(-1.0f), Vec3(1.0f, 1.0f, 1.0f));
    Scalar t = -1.0f;
    ASSERT_TRUE(intersectRay(b, r, &t));
    EXPECT_NEAR(t, 1.0f, 1e-4f);
}

TEST(AABBRay, FlatBoxIsStillHittable) {
    // A zero-thickness box must not be rejected by the tExit < tEnter test.
    const AABB flat(Vec3(-1.0f, -1.0f, 0.0f), Vec3(1.0f, 1.0f, 0.0f));
    const Ray r(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    Scalar t = -1.0f;
    EXPECT_TRUE(intersectRay(flat, r, &t));
    EXPECT_NEAR(t, 5.0f, 1e-4f);
}

TEST(AABBRay, EmptyBoxIsNeverHit) {
    const AABB empty;
    const Ray r(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    EXPECT_FALSE(intersectRay(empty, r));
}
