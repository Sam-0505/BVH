#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "geometry/intersect.hpp"

using namespace geom;

namespace {
// Unit right triangle in the z = 0 plane, CCW viewed from +Z.
Triangle unitTriangle() {
    return Triangle(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f));
}
}  // namespace

TEST(RayTriangle, HitsInteriorWithCorrectDistance) {
    const Triangle tri = unitTriangle();
    const Ray r(Vec3(0.25f, 0.25f, -3.0f), Vec3(0.0f, 0.0f, 1.0f));
    TriangleHit hit;
    ASSERT_TRUE(intersectRayTriangle(tri, r, hit));
    EXPECT_NEAR(hit.t, 3.0f, 1e-5f);
    EXPECT_NEAR(hit.u, 0.25f, 1e-5f);
    EXPECT_NEAR(hit.v, 0.25f, 1e-5f);
}

TEST(RayTriangle, BarycentricsIdentifyTheVertices) {
    const Triangle tri = unitTriangle();
    const Vec3 dir(0.0f, 0.0f, 1.0f);

    struct Case { Vec3 xy; Scalar u; Scalar v; };
    const Case cases[] = {
        {Vec3(0.0f, 0.0f, -1.0f), 0.0f, 0.0f},  // v0
        {Vec3(1.0f, 0.0f, -1.0f), 1.0f, 0.0f},  // v1
        {Vec3(0.0f, 1.0f, -1.0f), 0.0f, 1.0f},  // v2
    };
    for (const auto& c : cases) {
        TriangleHit hit;
        ASSERT_TRUE(intersectRayTriangle(tri, Ray(c.xy, dir), hit));
        EXPECT_NEAR(hit.u, c.u, 1e-5f);
        EXPECT_NEAR(hit.v, c.v, 1e-5f);
    }
}

TEST(RayTriangle, BarycentricsReconstructTheHitPoint) {
    const Triangle tri(Vec3(1.0f, 2.0f, 3.0f), Vec3(4.0f, 2.5f, 1.0f), Vec3(0.0f, 5.0f, 2.0f));
    const Ray r(Vec3(1.5f, 3.0f, 10.0f), Vec3(0.05f, -0.1f, -1.0f));
    TriangleHit hit;
    ASSERT_TRUE(intersectRayTriangle(tri, r, hit));

    // The two reconstructions must agree: along the ray, and from the triangle.
    EXPECT_TRUE(nearlyEqual(hit.position(r), hit.position(tri), 1e-4f));
    // And the barycentrics must be a valid convex combination.
    EXPECT_GE(hit.u, 0.0f);
    EXPECT_GE(hit.v, 0.0f);
    EXPECT_LE(hit.u + hit.v, 1.0f + 1e-6f);
}

TEST(RayTriangle, MissesOutsideEachEdge) {
    const Triangle tri = unitTriangle();
    const Vec3 dir(0.0f, 0.0f, 1.0f);
    // Beyond the hypotenuse, and outside each of the two legs.
    EXPECT_FALSE([&] { TriangleHit h; return intersectRayTriangle(tri, Ray(Vec3(0.8f, 0.8f, -1.0f), dir), h); }());
    EXPECT_FALSE([&] { TriangleHit h; return intersectRayTriangle(tri, Ray(Vec3(-0.1f, 0.5f, -1.0f), dir), h); }());
    EXPECT_FALSE([&] { TriangleHit h; return intersectRayTriangle(tri, Ray(Vec3(0.5f, -0.1f, -1.0f), dir), h); }());
}

TEST(RayTriangle, IsTwoSided) {
    // A spatial-query tool must find geometry regardless of facing; back-face
    // culling is a rendering concern, not an intersection one.
    const Triangle tri = unitTriangle();
    TriangleHit front, back;
    ASSERT_TRUE(intersectRayTriangle(tri, Ray(Vec3(0.25f, 0.25f, -3.0f), Vec3(0.0f, 0.0f, 1.0f)), front));
    ASSERT_TRUE(intersectRayTriangle(tri, Ray(Vec3(0.25f, 0.25f, 3.0f), Vec3(0.0f, 0.0f, -1.0f)), back));
    EXPECT_NEAR(front.t, back.t, 1e-5f);
    // Barycentrics are the same point seen from either side.
    EXPECT_NEAR(front.u, back.u, 1e-5f);
    EXPECT_NEAR(front.v, back.v, 1e-5f);
}

TEST(RayTriangle, ParallelRayMisses) {
    const Triangle tri = unitTriangle();
    // In the triangle's own plane: there is no single intersection point.
    TriangleHit hit;
    EXPECT_FALSE(intersectRayTriangle(tri, Ray(Vec3(-1.0f, 0.25f, 0.0f), Vec3(1.0f, 0.0f, 0.0f)), hit));
    // Parallel but offset from the plane.
    EXPECT_FALSE(intersectRayTriangle(tri, Ray(Vec3(-1.0f, 0.25f, 5.0f), Vec3(1.0f, 0.0f, 0.0f)), hit));
}

TEST(RayTriangle, DegenerateTriangleIsNeverHit) {
    // Collinear vertices have no area, so nothing can meaningfully hit them.
    const Triangle collinear(Vec3(0.0f), Vec3(1.0f, 1.0f, 1.0f), Vec3(2.0f, 2.0f, 2.0f));
    TriangleHit hit;
    EXPECT_FALSE(intersectRayTriangle(collinear, Ray(Vec3(2.0f, 0.0f, 0.0f), Vec3(-1.0f, 1.0f, 1.0f)), hit));

    const Vec3 p(1.0f, 2.0f, 3.0f);
    const Triangle point(p, p, p);
    EXPECT_FALSE(intersectRayTriangle(point, Ray(Vec3(0.0f), normalize(p)), hit));
}

TEST(RayTriangle, RespectsTMinAndTMax) {
    const Triangle tri = unitTriangle();
    const Vec3 o(0.25f, 0.25f, -3.0f);
    const Vec3 d(0.0f, 0.0f, 1.0f);
    TriangleHit hit;

    EXPECT_FALSE(intersectRayTriangle(tri, Ray(o, d, 0.0f, 2.0f), hit));   // cut short
    EXPECT_FALSE(intersectRayTriangle(tri, Ray(o, d, 4.0f, 100.0f), hit));  // starts past it
    EXPECT_TRUE(intersectRayTriangle(tri, Ray(o, d, 2.9f, 3.1f), hit));     // bracketing
}

TEST(RayTriangle, TIsMeasuredInUnitsOfDirectionLength) {
    // Consistent with Ray's contract: a direction of twice the length halves t.
    const Triangle tri = unitTriangle();
    TriangleHit unitDir, doubleDir;
    ASSERT_TRUE(intersectRayTriangle(tri, Ray(Vec3(0.25f, 0.25f, -4.0f), Vec3(0.0f, 0.0f, 1.0f)), unitDir));
    ASSERT_TRUE(intersectRayTriangle(tri, Ray(Vec3(0.25f, 0.25f, -4.0f), Vec3(0.0f, 0.0f, 2.0f)), doubleDir));
    EXPECT_NEAR(unitDir.t, 4.0f, 1e-5f);
    EXPECT_NEAR(doubleDir.t, 2.0f, 1e-5f);
}

TEST(RayTriangle, IsScaleInvariant) {
    // The reason there is no epsilon on the determinant: the same query at
    // three very different scales must give the same barycentric answer.
    for (const Scalar s : {1e-4f, 1.0f, 1e4f}) {
        const Triangle tri(Vec3(0.0f), Vec3(s, 0.0f, 0.0f), Vec3(0.0f, s, 0.0f));
        const Ray r(Vec3(0.25f * s, 0.25f * s, -3.0f * s), Vec3(0.0f, 0.0f, 1.0f));
        TriangleHit hit;
        ASSERT_TRUE(intersectRayTriangle(tri, r, hit)) << "missed at scale " << s;
        EXPECT_NEAR(hit.u, 0.25f, 1e-4f) << "scale " << s;
        EXPECT_NEAR(hit.v, 0.25f, 1e-4f) << "scale " << s;
        EXPECT_NEAR(hit.t, 3.0f * s, 3.0f * s * 1e-4f) << "scale " << s;
    }
}

TEST(RayTriangle, SliverTriangleDoesNotProduceSpuriousHits) {
    // A near-degenerate triangle has a tiny determinant. Without the
    // sign-folded unscaled comparisons this is where a false hit appears.
    const Triangle sliver(Vec3(0.0f), Vec3(1.0f, 0.0f, 0.0f), Vec3(0.5f, 1e-9f, 0.0f));
    TriangleHit hit;
    // Well outside the sliver in y, so it must miss.
    EXPECT_FALSE(intersectRayTriangle(sliver, Ray(Vec3(0.5f, 0.5f, -1.0f), Vec3(0.0f, 0.0f, 1.0f)), hit));
    // Through the sliver itself, which should still register.
    EXPECT_TRUE(intersectRayTriangle(sliver, Ray(Vec3(0.5f, 1e-10f, -1.0f), Vec3(0.0f, 0.0f, 1.0f)), hit));
}

// --- Randomised properties ---------------------------------------------------
//
// These aim each ray at a KNOWN point inside the triangle, chosen by sampling
// barycentric coordinates directly. That gives a ground truth to compare
// against rather than only self-consistency: the test knows what u, v and t
// ought to be before the routine runs. Scattering rays at random instead makes
// hits rare (roughly 0.5% in practice) and tests almost nothing.

namespace {

struct AimedCase {
    Triangle tri;
    Ray ray;
    Scalar expectedU;
    Scalar expectedV;
};

// Build a triangle and a ray aimed at a specific interior point. Returns false
// for configurations where the answer is legitimately ill-conditioned -- a
// near-degenerate triangle, or a ray nearly parallel to its plane -- since a
// tolerance that accommodated those would be too loose to catch real errors.
bool makeAimedCase(std::mt19937& rng, AimedCase& out) {
    std::uniform_real_distribution<float> coord(-2.0f, 2.0f);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);

    const Triangle tri(Vec3(coord(rng), coord(rng), coord(rng)),
                       Vec3(coord(rng), coord(rng), coord(rng)),
                       Vec3(coord(rng), coord(rng), coord(rng)));
    if (tri.isDegenerate(1e-3f)) return false;

    // Uniform-ish sample of the interior, folded back if it lands outside.
    Scalar u = unit(rng);
    Scalar v = unit(rng);
    if (u + v > Scalar(1)) {
        u = Scalar(1) - u;
        v = Scalar(1) - v;
    }
    // Keep away from the edges, where a hit/miss verdict is a coin flip.
    u = Scalar(0.05) + u * Scalar(0.85);
    v = Scalar(0.05) + v * Scalar(0.85);
    if (u + v > Scalar(0.95)) return false;

    const Vec3 target = tri.v0 + tri.edge01() * u + tri.edge02() * v;
    const Vec3 origin(coord(rng) * 3.0f, coord(rng) * 3.0f, coord(rng) * 3.0f);
    const Vec3 dir = target - origin;
    if (lengthSquared(dir) < 1e-6f) return false;

    // Reject grazing angles, where the intersection is genuinely ill-conditioned.
    if (std::fabs(dot(normalize(dir), tri.normal())) < 0.1f) return false;

    out.tri = tri;
    // t == 1 lands exactly on the target, so tMax of 2 comfortably contains it.
    out.ray = Ray(origin, dir, 0.0f, 2.0f);
    out.expectedU = u;
    out.expectedV = v;
    return true;
}

}  // namespace

TEST(RayTriangleProperty, RecoversTheBarycentricsItWasAimedAt) {
    std::mt19937 rng(0xB0A71234u);
    int checked = 0;
    for (int i = 0; i < 20000; ++i) {
        AimedCase c;
        if (!makeAimedCase(rng, c)) continue;

        TriangleHit hit;
        ASSERT_TRUE(intersectRayTriangle(c.tri, c.ray, hit))
            << "missed a point known to be inside the triangle, case " << i;
        ++checked;

        // The target sits at t == 1 by construction.
        ASSERT_NEAR(hit.t, 1.0f, 1e-3f) << "case " << i;
        ASSERT_NEAR(hit.u, c.expectedU, 1e-3f) << "case " << i;
        ASSERT_NEAR(hit.v, c.expectedV, 1e-3f) << "case " << i;
    }
    EXPECT_GT(checked, 2000) << "sample found too few usable cases to be meaningful";
}

TEST(RayTriangleProperty, ReportedHitPointLiesOnBothTheRayAndTheTriangle) {
    std::mt19937 rng(0xB0A79999u);
    int checked = 0;
    for (int i = 0; i < 20000; ++i) {
        AimedCase c;
        if (!makeAimedCase(rng, c)) continue;

        TriangleHit hit;
        ASSERT_TRUE(intersectRayTriangle(c.tri, c.ray, hit)) << "case " << i;
        ++checked;

        ASSERT_GE(hit.u, -1e-6f) << "case " << i;
        ASSERT_GE(hit.v, -1e-6f) << "case " << i;
        ASSERT_LE(hit.u + hit.v, 1.0f + 1e-6f) << "case " << i;

        // The two independent reconstructions of the point must agree.
        const Vec3 fromRay = hit.position(c.ray);
        const Vec3 fromTri = hit.position(c.tri);
        const Scalar scale = std::fmax(1.0f, maxComponent(abs(fromRay)));
        ASSERT_LE(length(fromRay - fromTri), 1e-3f * scale) << "case " << i;
    }
    EXPECT_GT(checked, 2000);
}

TEST(RayTriangleProperty, AHitIsIndependentOfVertexOrdering) {
    // The triangle is the same set of points however it is wound, so every
    // ordering must agree on whether there is a hit and at what distance.
    // Only the meaning of u and v changes.
    std::mt19937 rng(0xB0A75678u);
    int checked = 0;
    for (int i = 0; i < 20000; ++i) {
        AimedCase c;
        if (!makeAimedCase(rng, c)) continue;
        ++checked;

        const Triangle& base = c.tri;
        const Triangle orderings[] = {
            Triangle(base.v0, base.v1, base.v2), Triangle(base.v1, base.v2, base.v0),
            Triangle(base.v2, base.v0, base.v1), Triangle(base.v0, base.v2, base.v1),
            Triangle(base.v2, base.v1, base.v0), Triangle(base.v1, base.v0, base.v2),
        };
        for (const Triangle& permuted : orderings) {
            TriangleHit hit;
            ASSERT_TRUE(intersectRayTriangle(permuted, c.ray, hit))
                << "a vertex permutation lost the hit, case " << i;
            ASSERT_NEAR(hit.t, 1.0f, 1e-3f) << "case " << i;
        }
    }
    EXPECT_GT(checked, 2000);
}

TEST(RayTriangleProperty, RaysAimedOutsideTheTriangleMiss) {
    // The complement of the tests above: aim just beyond an edge and confirm
    // the rejection path is reached rather than a hit being invented.
    std::mt19937 rng(0xB0A7AAAAu);
    std::uniform_real_distribution<float> coord(-2.0f, 2.0f);
    int checked = 0;
    for (int i = 0; i < 20000; ++i) {
        const Triangle tri(Vec3(coord(rng), coord(rng), coord(rng)),
                           Vec3(coord(rng), coord(rng), coord(rng)),
                           Vec3(coord(rng), coord(rng), coord(rng)));
        if (tri.isDegenerate(1e-3f)) continue;

        // Barycentrics well outside the triangle: u + v comfortably above 1.
        const Scalar u = 1.2f, v = 0.4f;
        const Vec3 target = tri.v0 + tri.edge01() * u + tri.edge02() * v;
        const Vec3 origin(coord(rng) * 3.0f, coord(rng) * 3.0f, coord(rng) * 3.0f);
        const Vec3 dir = target - origin;
        if (lengthSquared(dir) < 1e-6f) continue;
        if (std::fabs(dot(normalize(dir), tri.normal())) < 0.1f) continue;
        ++checked;

        TriangleHit hit;
        ASSERT_FALSE(intersectRayTriangle(tri, Ray(origin, dir, 0.0f, 2.0f), hit))
            << "reported a hit for a point outside the triangle, case " << i;
    }
    EXPECT_GT(checked, 2000);
}
