#include <gtest/gtest.h>

#include "geometry/vec3.hpp"
#include "geometry/vec4.hpp"

using namespace geom;

TEST(Vec3, DefaultConstructsToZero) {
    const Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3, BroadcastConstructor) {
    const Vec3 v(2.5f);
    EXPECT_EQ(v, Vec3(2.5f, 2.5f, 2.5f));
}

TEST(Vec3, IndexedAccessMatchesNamedMembers) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v[0], 1.0f);
    EXPECT_FLOAT_EQ(v[1], 2.0f);
    EXPECT_FLOAT_EQ(v[2], 3.0f);

    // Mutable indexing must alias the real members, not a copy.
    v[1] = 9.0f;
    EXPECT_FLOAT_EQ(v.y, 9.0f);
}

TEST(Vec3, Arithmetic) {
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(4.0f, 5.0f, 6.0f);

    EXPECT_EQ(a + b, Vec3(5.0f, 7.0f, 9.0f));
    EXPECT_EQ(b - a, Vec3(3.0f, 3.0f, 3.0f));
    EXPECT_EQ(a * 2.0f, Vec3(2.0f, 4.0f, 6.0f));
    EXPECT_EQ(2.0f * a, Vec3(2.0f, 4.0f, 6.0f));
    EXPECT_EQ(b / 2.0f, Vec3(2.0f, 2.5f, 3.0f));
    EXPECT_EQ(-a, Vec3(-1.0f, -2.0f, -3.0f));
    EXPECT_EQ(mul(a, b), Vec3(4.0f, 10.0f, 18.0f));
}

TEST(Vec3, CompoundAssignment) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    v += Vec3(1.0f, 1.0f, 1.0f);
    EXPECT_EQ(v, Vec3(2.0f, 3.0f, 4.0f));
    v -= Vec3(2.0f, 2.0f, 2.0f);
    EXPECT_EQ(v, Vec3(0.0f, 1.0f, 2.0f));
    v *= 3.0f;
    EXPECT_EQ(v, Vec3(0.0f, 3.0f, 6.0f));
    v /= 3.0f;
    EXPECT_EQ(v, Vec3(0.0f, 1.0f, 2.0f));
}

TEST(Vec3, DotProduct) {
    EXPECT_FLOAT_EQ(dot(Vec3(1.0f, 2.0f, 3.0f), Vec3(4.0f, -5.0f, 6.0f)), 12.0f);

    // Orthogonal vectors have zero dot product.
    EXPECT_FLOAT_EQ(dot(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f)), 0.0f);

    // dot(v,v) == |v|^2.
    const Vec3 v(3.0f, 4.0f, 12.0f);
    EXPECT_FLOAT_EQ(dot(v, v), lengthSquared(v));
}

TEST(Vec3, CrossProductIsRightHanded) {
    const Vec3 x(1.0f, 0.0f, 0.0f);
    const Vec3 y(0.0f, 1.0f, 0.0f);
    const Vec3 z(0.0f, 0.0f, 1.0f);

    // The handedness convention the whole project depends on.
    EXPECT_EQ(cross(x, y), z);
    EXPECT_EQ(cross(y, z), x);
    EXPECT_EQ(cross(z, x), y);

    // Anticommutative.
    EXPECT_EQ(cross(y, x), -z);

    // Parallel vectors produce the zero vector.
    EXPECT_EQ(cross(x, x), Vec3(0.0f));
}

TEST(Vec3, CrossProductIsOrthogonalToBothInputs) {
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(-4.0f, 5.0f, 6.0f);
    const Vec3 c = cross(a, b);
    EXPECT_NEAR(dot(c, a), 0.0f, 1e-5f);
    EXPECT_NEAR(dot(c, b), 0.0f, 1e-5f);
}

TEST(Vec3, Length) {
    EXPECT_FLOAT_EQ(length(Vec3(3.0f, 4.0f, 0.0f)), 5.0f);
    EXPECT_FLOAT_EQ(lengthSquared(Vec3(3.0f, 4.0f, 0.0f)), 25.0f);
    EXPECT_FLOAT_EQ(length(Vec3(0.0f)), 0.0f);
}

TEST(Vec3, Normalize) {
    const Vec3 n = normalize(Vec3(3.0f, 4.0f, 0.0f));
    EXPECT_NEAR(length(n), 1.0f, 1e-6f);
    EXPECT_TRUE(nearlyEqual(n, Vec3(0.6f, 0.8f, 0.0f)));
}

TEST(Vec3, NormalizeSafeHandlesZeroVector) {
    // The degenerate case that must not produce NaN: a zero-length normal from
    // a degenerate triangle.
    const Vec3 n = normalizeSafe(Vec3(0.0f));
    EXPECT_EQ(n, Vec3(0.0f));
    EXPECT_TRUE(isFinite(n));

    const Vec3 fallback(0.0f, 1.0f, 0.0f);
    EXPECT_EQ(normalizeSafe(Vec3(0.0f), fallback), fallback);

    // A vector below the epsilon threshold is also treated as degenerate.
    EXPECT_EQ(normalizeSafe(Vec3(1e-9f, 0.0f, 0.0f)), Vec3(0.0f));
}

TEST(Vec3, ComponentWiseMinMax) {
    const Vec3 a(1.0f, 5.0f, 3.0f);
    const Vec3 b(4.0f, 2.0f, 6.0f);
    EXPECT_EQ(minComponents(a, b), Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(maxComponents(a, b), Vec3(4.0f, 5.0f, 6.0f));
}

TEST(Vec3, MinMaxComponentScalars) {
    const Vec3 v(-2.0f, 7.0f, 3.0f);
    EXPECT_FLOAT_EQ(minComponent(v), -2.0f);
    EXPECT_FLOAT_EQ(maxComponent(v), 7.0f);
}

TEST(Vec3, MaxAxis) {
    EXPECT_EQ(maxAxis(Vec3(3.0f, 1.0f, 2.0f)), 0);
    EXPECT_EQ(maxAxis(Vec3(1.0f, 3.0f, 2.0f)), 1);
    EXPECT_EQ(maxAxis(Vec3(1.0f, 2.0f, 3.0f)), 2);

    // Ties must resolve deterministically to the lowest axis, so that BVH
    // split-axis choice is reproducible for symmetric bounds.
    EXPECT_EQ(maxAxis(Vec3(2.0f, 2.0f, 2.0f)), 0);
    EXPECT_EQ(maxAxis(Vec3(1.0f, 2.0f, 2.0f)), 1);
}

TEST(Vec3, Abs) {
    EXPECT_EQ(abs(Vec3(-1.0f, 2.0f, -3.0f)), Vec3(1.0f, 2.0f, 3.0f));
}

TEST(Vec4, ConstructionAndAccess) {
    const Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v[0], 1.0f);
    EXPECT_FLOAT_EQ(v[3], 4.0f);
    EXPECT_EQ(v.xyz(), Vec3(1.0f, 2.0f, 3.0f));

    // The point/direction distinction that Mat4 relies on.
    const Vec4 point(Vec3(1.0f, 2.0f, 3.0f), 1.0f);
    const Vec4 direction(Vec3(1.0f, 2.0f, 3.0f), 0.0f);
    EXPECT_FLOAT_EQ(point.w, 1.0f);
    EXPECT_FLOAT_EQ(direction.w, 0.0f);
}

TEST(Vec4, DotProduct) {
    EXPECT_FLOAT_EQ(dot(Vec4(1.0f, 2.0f, 3.0f, 4.0f), Vec4(5.0f, 6.0f, 7.0f, 8.0f)), 70.0f);
}
