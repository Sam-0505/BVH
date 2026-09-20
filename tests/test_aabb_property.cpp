// Randomised property tests for the ray/AABB slab test.
//
// Hand-picked cases check the situations the author thought of. These check
// invariants that must hold for EVERY input, over a large random sample, which
// is what catches the sign, swap and ordering mistakes that slab tests are
// prone to. This suite also becomes the oracle for Phase 2: BVH traversal is
// correct only if it reports the same hits as testing every primitive directly.
//
// The generator is seeded with a fixed constant so a failure is reproducible.
// A randomised test that cannot be replayed is a flaky test.

#include <gtest/gtest.h>

#include <random>

#include "geometry/aabb.hpp"

using namespace geom;

namespace {

constexpr unsigned kSeed = 0x5EED1234u;

// Reference implementation: the same slab test with NO conservative widening.
// Kept deliberately separate from the production one so the two can be
// compared; if this drifts, the comparison stops meaning anything.
bool exactSlab(const AABB& box, const Vec3& origin, const Vec3& invDir, Scalar tMin,
               Scalar tMax) {
    Scalar t0 = tMin;
    Scalar t1 = tMax;
    for (int axis = 0; axis < 3; ++axis) {
        Scalar tNear = (box.min[axis] - origin[axis]) * invDir[axis];
        Scalar tFar = (box.max[axis] - origin[axis]) * invDir[axis];
        if (invDir[axis] < Scalar(0)) {
            const Scalar tmp = tNear;
            tNear = tFar;
            tFar = tmp;
        }
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar < t1 ? tFar : t1;
        if (t1 < t0) return false;
    }
    return true;
}

class RandomGeometry {
public:
    explicit RandomGeometry(unsigned seed) : rng_(seed) {}

    Scalar coord(Scalar lo, Scalar hi) {
        return std::uniform_real_distribution<Scalar>(lo, hi)(rng_);
    }

    AABB box(Scalar extent) {
        const Vec3 a(coord(-extent, extent), coord(-extent, extent), coord(-extent, extent));
        const Vec3 b(coord(-extent, extent), coord(-extent, extent), coord(-extent, extent));
        return AABB(minComponents(a, b), maxComponents(a, b));
    }

    Vec3 point(Scalar extent) {
        return {coord(-extent, extent), coord(-extent, extent), coord(-extent, extent)};
    }

    // Direction that is sometimes exactly axis-aligned, to keep the
    // zero-component / infinite-reciprocal paths in the sample.
    Vec3 direction() {
        Vec3 d = point(Scalar(1));
        std::uniform_int_distribution<int> zeroAxis(0, 5);
        const int which = zeroAxis(rng_);
        if (which < 3) d[which] = Scalar(0);
        if (lengthSquared(d) == Scalar(0)) d = Vec3(Scalar(0), Scalar(0), Scalar(1));
        return d;
    }

private:
    std::mt19937 rng_;
};

}  // namespace

TEST(AABBRayProperty, AnySampledPointInsideTheBoxImpliesAHit) {
    // The core soundness property: the slab test must never report a miss when
    // the ray demonstrably passes through the box. Sampling can only ever prove
    // a hit, never a miss, so this is a one-directional check with no false
    // alarms -- exactly the right shape for a randomised oracle.
    RandomGeometry gen(kSeed);
    constexpr int kRays = 4000;
    constexpr int kSamples = 128;
    int hitsProven = 0;

    for (int i = 0; i < kRays; ++i) {
        const AABB box = gen.box(Scalar(5));
        const Ray ray(gen.point(Scalar(10)), gen.direction(), Scalar(0), Scalar(30));
        const bool reported = intersectRay(box, ray);

        for (int s = 0; s <= kSamples; ++s) {
            const Scalar t =
                ray.tMin + (ray.tMax - ray.tMin) * (static_cast<Scalar>(s) / kSamples);
            if (box.contains(ray.at(t))) {
                ++hitsProven;
                ASSERT_TRUE(reported)
                    << "ray " << i << " passes through the box at t=" << t
                    << " but intersectRay reported a miss";
                break;
            }
        }
    }

    // Guard against the test silently proving nothing.
    EXPECT_GT(hitsProven, kRays / 100) << "sample found too few hits to be meaningful";
}

TEST(AABBRayProperty, WideningIsConservativeAndNeverRemovesAHit) {
    // The actual contract of the gamma(3) widening: it may ADD hits (a wasted
    // primitive test) but must never REMOVE one (a crack in the geometry).
    // No test in this repository demonstrates a case where the widening
    // changes the answer at all -- see the note in aabb.hpp -- but this pins
    // the direction of the effect, which is the part correctness depends on.
    RandomGeometry gen(kSeed + 1);
    constexpr int kRays = 20000;
    int disagreements = 0;

    for (int i = 0; i < kRays; ++i) {
        const AABB box = gen.box(Scalar(5));
        const Ray ray(gen.point(Scalar(10)), gen.direction(), Scalar(0), Scalar(30));
        const Vec3 inv = invDirection(ray);

        const bool exact = exactSlab(box, ray.origin, inv, ray.tMin, ray.tMax);
        const bool widened = intersectRay(box, ray.origin, inv, ray.tMin, ray.tMax);

        if (exact) {
            ASSERT_TRUE(widened) << "widening removed a hit on ray " << i;
        }
        if (exact != widened) ++disagreements;
    }

    // Recorded, not asserted to be zero: if a future change makes the widening
    // start mattering, that is information rather than a failure.
    if (disagreements > 0) {
        GTEST_LOG_(INFO) << "widening changed the result on " << disagreements << " of "
                         << kRays << " rays";
    }
}

TEST(AABBRayProperty, AReportedHitEntersAtOrBeforeTheBoxSurface) {
    // The sampled-point test above proves soundness only (never a false miss).
    // This is the other direction: when a hit IS reported with tEnter > tMin,
    // the ray must actually reach the box at that distance. Checked against a
    // box grown by the widening tolerance, since the test is conservative by
    // construction and may report entry fractionally early.
    RandomGeometry gen(kSeed + 5);
    int checked = 0;
    for (int i = 0; i < 5000; ++i) {
        const AABB box = gen.box(Scalar(5));
        if (box.isEmpty()) continue;
        const Ray ray(gen.point(Scalar(10)), gen.direction(), Scalar(0), Scalar(30));
        Scalar tEnter = -1.0f;
        if (!intersectRay(box, ray, &tEnter)) continue;
        if (tEnter <= ray.tMin) continue;  // originated inside; nothing to check

        // Slack proportional to the box size, covering both the deliberate
        // widening and ordinary rounding in computing the entry point.
        const Scalar slack = maxComponent(box.diagonal()) * Scalar(1e-3) + Scalar(1e-4);
        const AABB grown(box.min - Vec3(slack), box.max + Vec3(slack));
        ASSERT_TRUE(grown.contains(ray.at(tEnter)))
            << "reported entry at t=" << tEnter << " is outside the box on ray " << i;
        ++checked;
    }
    EXPECT_GT(checked, 100) << "sample found too few surface entries to be meaningful";
}

TEST(AABBRayProperty, ReportedEntryDistanceLiesWithinTheRayRange) {
    RandomGeometry gen(kSeed + 2);
    for (int i = 0; i < 5000; ++i) {
        const AABB box = gen.box(Scalar(5));
        const Ray ray(gen.point(Scalar(10)), gen.direction(), Scalar(0.5), Scalar(25));
        Scalar tEnter = -1.0f;
        if (intersectRay(box, ray, &tEnter)) {
            ASSERT_GE(tEnter, ray.tMin) << "entry before tMin on ray " << i;
            ASSERT_LE(tEnter, ray.tMax) << "entry beyond tMax on ray " << i;
            ASSERT_TRUE(std::isfinite(tEnter)) << "non-finite entry on ray " << i;
        }
    }
}

TEST(AABBRayProperty, EmptyBoxIsNeverHitFromAnyDirection) {
    // Bug #1 lived in the swap branch. The hand-written test only covers a
    // positive-direction ray, so sweep every sign combination including
    // axis-aligned and negative directions.
    RandomGeometry gen(kSeed + 3);
    const AABB empty;
    for (int i = 0; i < 5000; ++i) {
        const Ray ray(gen.point(Scalar(10)), gen.direction(), Scalar(0), kInfinity);
        ASSERT_FALSE(intersectRay(empty, ray)) << "empty box reported a hit on ray " << i;
    }
}

TEST(AABBRayProperty, ARayOriginatingInsideAlwaysHits) {
    RandomGeometry gen(kSeed + 4);
    for (int i = 0; i < 5000; ++i) {
        const AABB box = gen.box(Scalar(5));
        if (box.isEmpty() || box.volume() <= Scalar(0)) continue;
        // A point strictly inside, built by interpolating between the corners.
        const Vec3 f(gen.coord(0.1f, 0.9f), gen.coord(0.1f, 0.9f), gen.coord(0.1f, 0.9f));
        const Vec3 inside = box.min + mul(box.diagonal(), f);
        ASSERT_TRUE(box.contains(inside));

        const Ray ray(inside, gen.direction(), Scalar(0), kInfinity);
        Scalar tEnter = -1.0f;
        ASSERT_TRUE(intersectRay(box, ray, &tEnter)) << "missed from inside on ray " << i;
        EXPECT_FLOAT_EQ(tEnter, ray.tMin);
    }
}
