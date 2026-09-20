#include <gtest/gtest.h>

#include <cmath>

#include "geometry/mat4.hpp"
#include "geometry/ray.hpp"

using namespace geom;

TEST(Ray, DefaultConstruction) {
    const Ray r;
    EXPECT_EQ(r.origin, Vec3(0.0f));
    EXPECT_EQ(r.direction, Vec3(0.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(r.tMin, 0.0f);
    EXPECT_EQ(r.tMax, kInfinity);
}

TEST(Ray, ExplicitConstruction) {
    const Ray r(Vec3(1.0f, 2.0f, 3.0f), Vec3(0.0f, 1.0f, 0.0f), 0.5f, 100.0f);
    EXPECT_EQ(r.origin, Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(r.direction, Vec3(0.0f, 1.0f, 0.0f));
    EXPECT_FLOAT_EQ(r.tMin, 0.5f);
    EXPECT_FLOAT_EQ(r.tMax, 100.0f);
}

TEST(Ray, PointAtParameter) {
    const Ray r(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 2.0f, 0.0f));
    EXPECT_EQ(r.at(0.0f), r.origin);
    EXPECT_EQ(r.at(1.0f), Vec3(1.0f, 2.0f, 0.0f));
    EXPECT_EQ(r.at(2.5f), Vec3(1.0f, 5.0f, 0.0f));
    // Negative t is meaningful geometrically even when outside [tMin, tMax].
    EXPECT_EQ(r.at(-1.0f), Vec3(1.0f, -2.0f, 0.0f));
}

TEST(Ray, DirectionIsNotRequiredToBeNormalized) {
    // t is measured in units of |direction|. This is load-bearing: it is what
    // lets t survive a scaling transform unchanged.
    const Ray r(Vec3(0.0f), Vec3(0.0f, 0.0f, 10.0f));
    EXPECT_EQ(r.at(1.0f), Vec3(0.0f, 0.0f, 10.0f));
}

TEST(Ray, InverseDirectionOfAxisAlignedRayIsInfinite) {
    // Zero components must yield infinities, not a trapped division -- the slab
    // test is written to consume them.
    const Ray r(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f));
    const Vec3 inv = invDirection(r);
    EXPECT_TRUE(std::isinf(inv.x));
    EXPECT_TRUE(std::isinf(inv.y));
    EXPECT_FLOAT_EQ(inv.z, 1.0f);
    EXPECT_GT(inv.x, 0.0f);
}

TEST(Ray, InverseDirectionSignFollowsDirectionSign) {
    const Ray r(Vec3(0.0f), Vec3(-2.0f, 4.0f, -0.5f));
    const Vec3 inv = invDirection(r);
    EXPECT_FLOAT_EQ(inv.x, -0.5f);
    EXPECT_FLOAT_EQ(inv.y, 0.25f);
    EXPECT_FLOAT_EQ(inv.z, -2.0f);
}

TEST(Ray, TransformAppliesTranslationToOriginOnly) {
    const Mat4 t = translation(Vec3(10.0f, 0.0f, 0.0f));
    const Ray r(Vec3(1.0f, 2.0f, 3.0f), Vec3(0.0f, 0.0f, 1.0f));
    const Ray out = transformRay(t, r);

    EXPECT_TRUE(nearlyEqual(out.origin, Vec3(11.0f, 2.0f, 3.0f)));
    // A direction must not pick up translation.
    EXPECT_TRUE(nearlyEqual(out.direction, Vec3(0.0f, 0.0f, 1.0f)));
}

TEST(Ray, TransformPreservesTRange) {
    const Mat4 m = translation(Vec3(1.0f, 2.0f, 3.0f)) * scaling(Vec3(2.0f, 2.0f, 2.0f));
    const Ray r(Vec3(0.0f), Vec3(0.0f, 0.0f, 1.0f), 1.5f, 42.0f);
    const Ray out = transformRay(m, r);
    EXPECT_FLOAT_EQ(out.tMin, 1.5f);
    EXPECT_FLOAT_EQ(out.tMax, 42.0f);
}

TEST(Ray, TransformKeepsHitParameterConsistentUnderScaling) {
    // The reason transformRay does NOT renormalise the direction: a point found
    // at parameter t in one space must be the same physical point at the same t
    // in the other. If the direction were renormalised, t would silently change
    // meaning and hit distances would no longer be comparable.
    const Mat4 m = scaling(Vec3(3.0f, 3.0f, 3.0f));
    const Ray r(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 2.0f));
    const Ray out = transformRay(m, r);

    const Scalar t = 0.75f;
    // Transforming the hit point, versus evaluating the transformed ray.
    EXPECT_TRUE(nearlyEqual(transformPoint(m, r.at(t)), out.at(t), 1e-5f));
}

TEST(Ray, RoundTripThroughInverseTransform) {
    // The world -> object -> world path that object-space BVH traversal uses.
    const Mat4 m = translation(Vec3(4.0f, -2.0f, 7.0f)) *
                   rotation(Vec3(0.0f, 1.0f, 0.0f), radians(30.0f)) *
                   scaling(Vec3(2.0f, 1.0f, 0.5f));
    const Mat4 mInv = inverse(m);

    const Ray world(Vec3(1.0f, 2.0f, 3.0f), Vec3(0.3f, -0.5f, 0.8f), 0.0f, 50.0f);
    const Ray object = transformRay(mInv, world);
    const Ray back = transformRay(m, object);

    EXPECT_TRUE(nearlyEqual(back.origin, world.origin, 1e-3f));
    EXPECT_TRUE(nearlyEqual(back.direction, world.direction, 1e-3f));
}
