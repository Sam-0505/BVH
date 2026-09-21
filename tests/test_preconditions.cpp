// The asserting half of the error policy (the throwing half is in
// test_mesh.cpp). Guarded on NDEBUG: without it these would vacuously "pass"
// in Release, which is worse than not having them. Run Debug to exercise.

#include <gtest/gtest.h>

#include <cmath>

#include "geometry/aabb.hpp"
#include "geometry/mesh.hpp"
#include "geometry/vec3.hpp"

using namespace geom;

#ifdef NDEBUG

TEST(PreconditionDeathTest, SkippedInReleaseBuild) {
    GTEST_SKIP() << "assertions are compiled out with NDEBUG; run the Debug build "
                    "to exercise preconditions";
}

#else

TEST(PreconditionDeathTest, MeshTriangleIndexOutOfRange) {
    const Mesh m({{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}}, {0, 1, 2});
    ASSERT_EQ(m.triangleCount(), 1u);
    EXPECT_DEATH((void)m.triangle(1), "triangle index out of range");
}

TEST(PreconditionDeathTest, DefaultConstructedMeshHasNoTriangleZero) {
    // The empty-mesh case: triangle(0) would index an empty vector.
    const Mesh m;
    EXPECT_DEATH((void)m.triangle(0), "triangle index out of range");
}

TEST(PreconditionDeathTest, MeshTriangleIndicesOutOfRange) {
    const Mesh m({{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}}, {0, 1, 2});
    std::uint32_t a = 0, b = 0, c = 0;
    EXPECT_DEATH(m.triangleIndices(5, a, b, c), "triangle index out of range");
}

TEST(PreconditionDeathTest, CentroidOfEmptyBox) {
    // Would silently return NaN and corrupt a SAH bin index.
    const AABB empty;
    EXPECT_DEATH((void)empty.centroid(), "centroid\\(\\) of an empty AABB");
}

TEST(PreconditionDeathTest, OffsetAgainstEmptyBox) {
    const AABB empty;
    EXPECT_DEATH((void)empty.offset(Vec3(1.0f, 2.0f, 3.0f)), "offset\\(\\) against an empty AABB");
}

TEST(PreconditionDeathTest, NormalizeZeroVector) {
    EXPECT_DEATH((void)normalize(Vec3(0.0f)), "zero-length vector");
}

TEST(PreconditionDeathTest, Vec3IndexOutOfRange) {
    const Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_DEATH((void)v[3], ".*");
}

#endif  // NDEBUG

// Release counterparts: with the assert compiled out, the operation must still
// be free of UB even though its result is meaningless.

TEST(Precondition, NormalizeZeroVectorIsNotUndefinedBehaviour) {
    // operator/ must still avoid the literal divide-by-zero that [expr.mul]/4
    // leaves undefined, with or without NDEBUG.
#ifdef NDEBUG
    const Vec3 n = normalize(Vec3(0.0f));
    // Every component is 0/0 == NaN. The point is that it is a NaN and not UB.
    EXPECT_TRUE(std::isnan(n.x));
#else
    GTEST_SKIP() << "assert fires first in a Debug build";
#endif
}

TEST(Precondition, DivideVectorByZeroMatchesIEEE) {
    const Vec3 v(1.0f, -2.0f, 0.0f);
    const Vec3 r = v / 0.0f;
    EXPECT_TRUE(std::isinf(r.x));
    EXPECT_GT(r.x, 0.0f);
    EXPECT_TRUE(std::isinf(r.y));
    EXPECT_LT(r.y, 0.0f);
    EXPECT_TRUE(std::isnan(r.z));  // 0/0
}

TEST(Precondition, NormalizeSafeIsTheSupportedPathForDegenerateInput) {
    EXPECT_EQ(normalizeSafe(Vec3(0.0f)), Vec3(0.0f));
    EXPECT_TRUE(isFinite(normalizeSafe(Vec3(0.0f))));
}
