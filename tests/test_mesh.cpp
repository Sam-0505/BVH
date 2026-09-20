#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "geometry/mesh.hpp"

using namespace geom;

namespace {

// A unit cube as 8 shared vertices and 12 triangles -- the case that motivates
// indexed storage, since each vertex is referenced by three or more triangles.
Mesh unitCube() {
    std::vector<Vec3> positions = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 1.0f},
    };
    std::vector<std::uint32_t> indices = {
        0, 2, 1, 0, 3, 2,  // -Z
        4, 5, 6, 4, 6, 7,  // +Z
        0, 1, 5, 0, 5, 4,  // -Y
        3, 7, 6, 3, 6, 2,  // +Y
        0, 4, 7, 0, 7, 3,  // -X
        1, 2, 6, 1, 6, 5,  // +X
    };
    return Mesh(std::move(positions), std::move(indices));
}

Mesh singleTriangle() {
    return Mesh({{0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}}, {0, 1, 2});
}

}  // namespace

TEST(Mesh, DefaultConstructedIsEmpty) {
    const Mesh m;
    EXPECT_TRUE(m.empty());
    EXPECT_EQ(m.vertexCount(), 0u);
    EXPECT_EQ(m.triangleCount(), 0u);
    EXPECT_TRUE(m.bounds().isEmpty());
}

TEST(Mesh, CountsAreDerivedFromBuffers) {
    const Mesh m = unitCube();
    EXPECT_EQ(m.vertexCount(), 8u);
    EXPECT_EQ(m.triangleCount(), 12u);
    EXPECT_FALSE(m.empty());
}

TEST(Mesh, TriangleAccessorResolvesIndices) {
    const Mesh m = singleTriangle();
    const Triangle t = m.triangle(0);
    EXPECT_EQ(t.v0, Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(t.v1, Vec3(2.0f, 0.0f, 0.0f));
    EXPECT_EQ(t.v2, Vec3(0.0f, 3.0f, 0.0f));
    EXPECT_NEAR(t.area(), 3.0f, 1e-6f);
}

TEST(Mesh, TriangleIndicesAccessor) {
    const Mesh m = unitCube();
    std::uint32_t a = 0, b = 0, c = 0;
    m.triangleIndices(1, a, b, c);
    EXPECT_EQ(a, 0u);
    EXPECT_EQ(b, 3u);
    EXPECT_EQ(c, 2u);
}

TEST(Mesh, BoundsCoverWholeMesh) {
    const Mesh m = unitCube();
    EXPECT_EQ(m.bounds().min, Vec3(0.0f));
    EXPECT_EQ(m.bounds().max, Vec3(1.0f));
    EXPECT_FLOAT_EQ(m.bounds().volume(), 1.0f);
}

TEST(Mesh, BoundsContainEveryTriangle) {
    const Mesh m = unitCube();
    for (std::size_t i = 0; i < m.triangleCount(); ++i) {
        EXPECT_TRUE(m.bounds().contains(m.triangle(i).bounds()))
            << "triangle " << i << " escapes the mesh bounds";
    }
}

TEST(Mesh, RejectsIndexCountNotMultipleOfThree) {
    EXPECT_THROW(Mesh({{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}}, {0, 1}), std::invalid_argument);
}

TEST(Mesh, RejectsOutOfRangeIndex) {
    // The validation that lets traversal index without bounds checks.
    EXPECT_THROW(Mesh({{0.0f, 0.0f, 0.0f}}, {0, 0, 5}), std::invalid_argument);
}

TEST(Mesh, RejectsAnyIndexIntoEmptyVertexBuffer) {
    EXPECT_THROW(Mesh({}, {0, 1, 2}), std::invalid_argument);
}

TEST(Mesh, AcceptsEmptyMesh) {
    EXPECT_NO_THROW(Mesh({}, {}));
}

TEST(Mesh, BoundsAreTightAndVertexBoundsAreConservative) {
    // A vertex no triangle references must NOT inflate bounds(): the SAH
    // normalises child area against parent area, so an inflated root would
    // shift every split decision and make benchmarks depend on file hygiene.
    std::vector<Vec3> positions = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {100.0f, 100.0f, 100.0f},  // orphan
    };
    const Mesh m(std::move(positions), {0, 1, 2});

    // Tight: covers only the referenced triangle.
    EXPECT_EQ(m.bounds().max, Vec3(1.0f, 1.0f, 0.0f));
    // Conservative: covers the whole vertex array.
    EXPECT_EQ(m.vertexBounds().max, Vec3(100.0f));
    // The conservative bound always contains the tight one.
    EXPECT_TRUE(m.vertexBounds().contains(m.bounds()));
}

TEST(Mesh, CountsDegenerateTriangles) {
    // Two triangles: one valid, one collapsed to a line.
    const Mesh m({{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {2.0f, 0.0f, 0.0f}},
                 {0, 1, 2, 0, 1, 3});
    EXPECT_EQ(m.triangleCount(), 2u);
    EXPECT_EQ(m.countDegenerateTriangles(), 1u);
}

TEST(Mesh, TransformMovesVerticesAndRefreshesBounds) {
    Mesh m = unitCube();
    m.transform(translation(Vec3(10.0f, 0.0f, 0.0f)));

    EXPECT_EQ(m.bounds().min, Vec3(10.0f, 0.0f, 0.0f));
    EXPECT_EQ(m.bounds().max, Vec3(11.0f, 1.0f, 1.0f));
    EXPECT_TRUE(nearlyEqual(m.triangle(0).v0, Vec3(10.0f, 0.0f, 0.0f)));
}

TEST(Mesh, TransformScalesBounds) {
    Mesh m = unitCube();
    m.transform(scaling(Vec3(2.0f, 4.0f, 8.0f)));
    EXPECT_TRUE(nearlyEqual(m.bounds().max, Vec3(2.0f, 4.0f, 8.0f)));
    EXPECT_NEAR(m.bounds().volume(), 64.0f, 1e-4f);
}

TEST(Mesh, TransformPreservesTopology) {
    Mesh m = unitCube();
    const std::vector<std::uint32_t> before = m.indices();
    m.transform(rotation(Vec3(0.0f, 1.0f, 0.0f), radians(45.0f)));
    EXPECT_EQ(m.indices(), before);
    EXPECT_EQ(m.triangleCount(), 12u);
}

TEST(Mesh, TransformWithNonAffineMatrixAppliesPerspectiveDivide) {
    // Every other transform test uses an affine matrix, leaving the
    // perspective-divide path in transformPoint unexercised from Mesh.
    Mesh m = singleTriangle();  // vertices (0,0,0), (2,0,0), (0,3,0)

    Mat4 projective = Mat4::identity();
    projective.m[0][3] = 1.0f;  // w_out = x_in + 1
    m.transform(projective);

    // (0,0,0) -> w=1, unchanged. (2,0,0) -> w=3, so (2/3,0,0).
    EXPECT_TRUE(nearlyEqual(m.triangle(0).v0, Vec3(0.0f, 0.0f, 0.0f), 1e-5f));
    EXPECT_TRUE(nearlyEqual(m.triangle(0).v1, Vec3(2.0f / 3.0f, 0.0f, 0.0f), 1e-5f));
    // (0,3,0) -> w=1, unchanged.
    EXPECT_TRUE(nearlyEqual(m.triangle(0).v2, Vec3(0.0f, 3.0f, 0.0f), 1e-5f));

    // Bounds must have been refreshed against the transformed positions.
    EXPECT_TRUE(nearlyEqual(m.bounds().max, Vec3(2.0f / 3.0f, 3.0f, 0.0f), 1e-5f));
}
