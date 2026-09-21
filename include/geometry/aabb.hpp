#pragma once

#include <cassert>

#include "geometry/ray.hpp"
#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// Conservative widening for the ray/box exit distance; see intersectRay.
// Set project-wide via bvh_project_options -- intersectRay is inline, so two
// TUs disagreeing on this would be a silent ODR violation. The self-default is
// only so the header compiles standalone.
#ifndef BVH_CONSERVATIVE_RAY_BOX
#define BVH_CONSERVATIVE_RAY_BOX 1
#endif

#if BVH_CONSERVATIVE_RAY_BOX
inline constexpr Scalar kRayBoxWidening = Scalar(1) + Scalar(2) * gamma(3);
#else
inline constexpr Scalar kRayBoxWidening = Scalar(1);
#endif

// Axis-aligned bounding box. Chosen over tighter volumes because it is tested
// far more often than built, and union is just component-wise min/max.
//
// Default-constructed boxes are EMPTY: min = +inf, max = -inf. That inverted
// sentinel is what lets extend() work with no special case.
struct AABB {
    Vec3 min{kInfinity};
    Vec3 max{-kInfinity};

    constexpr AABB() = default;
    constexpr AABB(const Vec3& lo, const Vec3& hi) : min(lo), max(hi) {}

    // Degenerate box containing exactly one point.
    static constexpr AABB fromPoint(const Vec3& p) { return AABB(p, p); }

    // Empty (min > max somewhere) is distinct from degenerate (a point or flat
    // plane), which is a real zero-volume region that can still be hit.
    constexpr bool isEmpty() const { return min.x > max.x || min.y > max.y || min.z > max.z; }

    void extend(const Vec3& p) {
        min = minComponents(min, p);
        max = maxComponents(max, p);
    }

    void extend(const AABB& b) {
        // Merging an empty box is a branch-free no-op thanks to the sentinel.
        min = minComponents(min, b.min);
        max = maxComponents(max, b.max);
    }

    constexpr Vec3 diagonal() const { return max - min; }

    // Precondition: not empty -- an empty box gives NaN here, which SAH binning
    // would turn into a corrupt bin index rather than a crash.
    constexpr Vec3 centroid() const {
        assert(!isEmpty() && "centroid() of an empty AABB is NaN");
        // min + 0.5*(max-min), not 0.5*(min+max), to avoid overflow.
        return min + (max - min) * Scalar(0.5);
    }

    // The SAH cost term: for convex volumes, the chance a random ray hitting
    // the parent also hits the child is the ratio of their surface areas.
    // Zero for an empty box, so cost sums stay finite.
    constexpr Scalar surfaceArea() const {
        if (isEmpty()) return Scalar(0);
        const Vec3 d = max - min;
        return Scalar(2) * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    constexpr Scalar volume() const {
        if (isEmpty()) return Scalar(0);
        const Vec3 d = max - min;
        return d.x * d.y * d.z;
    }

    // Default split axis: a cheap proxy for the one that most reduces child area.
    constexpr int longestAxis() const { return maxAxis(diagonal()); }

    // Inclusive containment -- a point on the boundary counts as inside.
    constexpr bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    // An empty argument is contained in anything. Asymmetric with intersects()
    // below, but both are correct set semantics.
    constexpr bool contains(const AABB& b) const {
        if (b.isEmpty()) return true;
        return contains(b.min) && contains(b.max);
    }

    // Touching counts as intersecting: a false positive costs one narrow-phase
    // test, a false negative is a missed collision.
    constexpr bool intersects(const AABB& b) const {
        if (isEmpty() || b.isEmpty()) return false;
        return min.x <= b.max.x && max.x >= b.min.x &&
               min.y <= b.max.y && max.y >= b.min.y &&
               min.z <= b.max.z && max.z >= b.min.z;
    }

    // Position of p in [0,1] per axis, for SAH binning. Degenerate axes report
    // 0 rather than dividing by zero.
    constexpr Vec3 offset(const Vec3& p) const {
        assert(!isEmpty() && "offset() against an empty AABB is meaningless");
        Vec3 o = p - min;
        if (max.x > min.x) o.x /= (max.x - min.x);
        if (max.y > min.y) o.y /= (max.y - min.y);
        if (max.z > min.z) o.z /= (max.z - min.z);
        return o;
    }

    friend constexpr bool operator==(const AABB& a, const AABB& b) {
        return a.min == b.min && a.max == b.max;
    }
    friend constexpr bool operator!=(const AABB& a, const AABB& b) { return !(a == b); }
};

inline AABB merge(const AABB& a, const AABB& b) {
    AABB r = a;
    r.extend(b);
    return r;
}

inline AABB merge(const AABB& a, const Vec3& p) {
    AABB r = a;
    r.extend(p);
    return r;
}

// Overlap region of two boxes; empty (min > max) when they do not overlap.
inline AABB intersection(const AABB& a, const AABB& b) {
    return AABB(maxComponents(a.min, b.min), minComponents(a.max, b.max));
}

// Ray/AABB intersection by the slab method. The box is the intersection of
// three axis-aligned slabs; keep a running [tEnter, tExit] and hit iff it stays
// non-empty. No divides -- the caller supplies the reciprocal direction.
//
// tEnter receives the entry distance, clamped to tMin for a ray starting
// inside. An empty box never reports a hit.
//
// The exit widening is PBRT's error analysis (3rd ed. sec 3.9): tExit carries
// up to gamma(3) relative error, so a ray grazing a shared face can otherwise
// miss both children. The entry distance is left EXACT -- it is only ever
// compared against a triangle-derived distance, whose error budget is an order
// of magnitude larger, so the slack belongs on that comparison (see
// kBestPruneSlack in bvh.cpp) and not here. Switchable so Phase 9 can price it.
inline bool intersectRay(const AABB& box, const Vec3& origin, const Vec3& invDir, Scalar tMin,
                         Scalar tMax, Scalar* tEnter = nullptr) {
    Scalar t0 = tMin;
    Scalar t1 = tMax;

    for (int axis = 0; axis < 3; ++axis) {
        Scalar tNear = (box.min[axis] - origin[axis]) * invDir[axis];
        Scalar tFar = (box.max[axis] - origin[axis]) * invDir[axis];

        // Swap on the SIGN of the direction, not on `tNear > tFar`. The
        // published comparison form "repairs" an inverted empty box into
        // [-inf, +inf], so every empty box reports a hit. Same cost.
        if (invDir[axis] < Scalar(0)) {
            const Scalar tmp = tNear;
            tNear = tFar;
            tFar = tmp;
        }

        tFar *= kRayBoxWidening;  // exactly 1 when disabled, so this folds away

        // NaN-tolerant: an axis-parallel ray exactly on a slab plane gives
        // 0 * inf == NaN, and these comparisons then keep the existing bound.
        // Do not swap in std::max/min -- their NaN behaviour differs.
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar < t1 ? tFar : t1;

        // Strict `<` so a flat box still hits.
        if (t1 < t0) return false;
    }

    if (tEnter != nullptr) *tEnter = t0;
    return true;
}

inline bool intersectRay(const AABB& box, const Ray& r, Scalar* tEnter = nullptr) {
    return intersectRay(box, r.origin, invDirection(r), r.tMin, r.tMax, tEnter);
}

inline bool nearlyEqual(const AABB& a, const AABB& b, Scalar tol = kEpsilon) {
    return nearlyEqual(a.min, b.min, tol) && nearlyEqual(a.max, b.max, tol);
}

}  // namespace geom
