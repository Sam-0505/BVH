#pragma once

#include <cassert>

#include "geometry/ray.hpp"
#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// Conservative widening factor applied to the ray/box exit distance.
//
// Defined as a constant rather than an #if at the use site so that disabling it
// changes one value and nothing else. Set BVH_CONSERVATIVE_RAY_BOX=0 to measure
// the exact test against the conservative one; see intersectRay for why this is
// currently unproven and kept anyway.
#ifndef BVH_CONSERVATIVE_RAY_BOX
#define BVH_CONSERVATIVE_RAY_BOX 1
#endif

#if BVH_CONSERVATIVE_RAY_BOX
inline constexpr Scalar kRayBoxWidening = Scalar(1) + Scalar(2) * gamma(3);
#else
inline constexpr Scalar kRayBoxWidening = Scalar(1);
#endif

// Axis-aligned bounding box, stored as min/max corners.
//
// Why AABBs and not tighter volumes (OBBs, k-DOPs, spheres): the bounding volume
// is tested far more often than it is built, so the cost that matters is the
// per-test cost, not the tightness. An AABB/ray test is a handful of multiplies
// and comparisons with no trigonometry and no matrix; an OBB needs a transform
// into its local frame first. AABBs are also closed under the union operation in
// a way that is trivial to compute (component-wise min/max), which is what makes
// bottom-up bounds propagation in a BVH cheap. The cost is looser fit for
// diagonal geometry, which shows up as more false-positive node visits.
//
// DEFAULT-CONSTRUCTED BOXES ARE EMPTY, represented as min = +inf, max = -inf.
// This inverted sentinel is what makes `extend` work without a special case: the
// first point merged into an empty box produces exactly that point's degenerate
// box, because min(+inf, p) == p and max(-inf, p) == p. A zero-initialised box
// would instead wrongly contain the origin.
struct AABB {
    Vec3 min{kInfinity};
    Vec3 max{-kInfinity};

    constexpr AABB() = default;
    constexpr AABB(const Vec3& lo, const Vec3& hi) : min(lo), max(hi) {}

    // Degenerate box containing exactly one point.
    static constexpr AABB fromPoint(const Vec3& p) { return AABB(p, p); }

    // An empty box has min > max on at least one axis. Note this is distinct
    // from a *degenerate* box (a point or a flat plane), which is a real,
    // zero-volume region that legitimately participates in intersection tests.
    constexpr bool isEmpty() const { return min.x > max.x || min.y > max.y || min.z > max.z; }

    void extend(const Vec3& p) {
        min = minComponents(min, p);
        max = maxComponents(max, p);
    }

    void extend(const AABB& b) {
        // Merging an empty box is a no-op: its min is +inf and max is -inf, so
        // the component-wise min/max leave us unchanged. No branch needed.
        min = minComponents(min, b.min);
        max = maxComponents(max, b.max);
    }

    constexpr Vec3 diagonal() const { return max - min; }

    // Precondition: the box is not empty.
    //
    // On an empty box this is +inf + (-inf - +inf) * 0.5, which is NaN. That
    // matters because Phase 2's SAH binning takes the centroid of a primitive
    // range's bounds, and an empty range at a recursion boundary is exactly how
    // you reach here -- a NaN centroid then propagates silently into a bin
    // index and corrupts the tree instead of crashing.
    constexpr Vec3 centroid() const {
        assert(!isEmpty() && "centroid() of an empty AABB is NaN");
        // Written as min + 0.5*(max-min) rather than 0.5*(min+max) to avoid
        // overflow to inf when both corners are large and same-signed.
        return min + (max - min) * Scalar(0.5);
    }

    // Total surface area. This is the cost term in the Surface Area Heuristic:
    // for a convex volume, the probability that a uniformly distributed random
    // ray hitting the parent also hits the child is the ratio of their surface
    // areas. That relationship is why SAH minimises area-weighted primitive
    // counts rather than, say, volume-weighted ones.
    //
    // Returns 0 for an empty box so SAH cost sums stay finite.
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

    // Axis along which the box is longest. Used as the default split axis: it is
    // a cheap proxy for the axis whose split will most reduce child surface area.
    constexpr int longestAxis() const { return maxAxis(diagonal()); }

    // Inclusive containment -- a point on the boundary counts as inside.
    constexpr bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    // An empty argument is contained in anything: the empty set is a subset of
    // every set. This looks asymmetric next to intersects(), which returns
    // false for an empty box, but both follow the same set semantics --
    // (empty subset A) is true, while (empty intersect A) is empty.
    constexpr bool contains(const AABB& b) const {
        if (b.isEmpty()) return true;
        return contains(b.min) && contains(b.max);
    }

    // Touching boxes count as intersecting. Chosen deliberately: for collision
    // and clearance queries a false positive costs one extra narrow-phase test,
    // while a false negative is a missed collision.
    constexpr bool intersects(const AABB& b) const {
        if (isEmpty() || b.isEmpty()) return false;
        return min.x <= b.max.x && max.x >= b.min.x &&
               min.y <= b.max.y && max.y >= b.min.y &&
               min.z <= b.max.z && max.z >= b.min.z;
    }

    // Position of `p` within the box, normalised to [0,1] per axis. Degenerate
    // axes (min == max) report 0 rather than dividing by zero. Used by SAH
    // binning to map a centroid to a bin index.
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

// --- Ray/AABB intersection (slab method) -------------------------------------
//
// The box is the intersection of three axis-aligned slabs. For each slab compute
// the two t values where the ray crosses its planes, keeping a running
// [tEnter, tExit] interval. The ray hits iff that interval stays non-empty.
// Cost: 6 multiplies, 6 adds, and a handful of comparisons -- no divides,
// because the caller supplies the reciprocal direction.
//
// Two numerical hazards are handled explicitly:
//
//  1. AXIS-PARALLEL RAYS. A zero direction component makes invDir infinite, and
//     the products become +/-inf, which compare correctly: the slab is either
//     fully entered or fully missed. But if the origin lies EXACTLY on a slab
//     plane, the numerator is zero too and 0 * inf == NaN. Every comparison with
//     NaN is false, so the `t0 > tEnter ? t0 : tEnter` form (rather than
//     std::max) deliberately keeps the existing bound when a NaN appears. That
//     makes the degenerate axis a no-op -- conservative, and never a false miss.
//
//  2. ROUNDING AT GRAZING ANGLES. Each t is the result of a subtract and a
//     multiply, so it carries relative error up to gamma(3). Widening the exit
//     distance by (1 + 2*gamma(3)) makes the test conservative, so it may
//     report a hit fractionally outside the true box -- costing a wasted
//     primitive test -- rather than a false miss.
//
//     HONEST STATUS OF THIS WIDENING: it follows PBRT's error analysis
//     (Pharr, Jakob & Humphreys, 3rd ed., sec. 3.9), which is sound. But no
//     test in this repository demonstrates a failure it prevents, and a sweep
//     of 3 million rays aimed at shared sibling faces found no difference with
//     it removed. Its real justification arrives in Phase 2: watertightness
//     matters between a node's bound and the triangle test inside it, and
//     there is no triangle test yet. It is kept because a conservative bound
//     is the safe default, and made switchable so Phase 9 can measure what it
//     costs -- it is 3 of the 9 multiplies in this function, the innermost
//     loop of the whole system. Do not restate the PBRT rationale as a
//     measured result until there is a benchmark behind it.
//
// `tEnter` receives the entry distance when the function returns true. For a ray
// originating inside the box that is tMin, not a negative distance.
//
// An empty (default-constructed) box never reports a hit -- see the note on the
// swap below for why that needs care.
inline bool intersectRay(const AABB& box, const Vec3& origin, const Vec3& invDir, Scalar tMin,
                         Scalar tMax, Scalar* tEnter = nullptr) {
    Scalar t0 = tMin;
    Scalar t1 = tMax;

    for (int axis = 0; axis < 3; ++axis) {
        Scalar tNear = (box.min[axis] - origin[axis]) * invDir[axis];
        Scalar tFar = (box.max[axis] - origin[axis]) * invDir[axis];

        // A negative reciprocal means the ray crosses max before min.
        //
        // Swap on the SIGN OF THE DIRECTION, not on `tNear > tFar`. The
        // comparison form looks equivalent and is widely published, but it
        // silently "repairs" an inverted (empty) box into an infinite one: with
        // min = +inf and max = -inf it swaps them into [-inf, +inf], so an empty
        // box reports a hit against every ray. Testing the sign instead leaves
        // the inverted interval inverted, and the tExit < tEnter check below
        // rejects it. Same cost, one fewer way to be wrong.
        if (invDir[axis] < Scalar(0)) {
            const Scalar tmp = tNear;
            tNear = tFar;
            tFar = tmp;
        }

        // Conservative widening -- see hazard 2 above. kRayBoxWidening is
        // exactly 1 when disabled, so the multiply folds away entirely.
        tFar *= kRayBoxWidening;

        // NaN-tolerant narrowing -- see hazard 1 above. Do NOT replace with
        // std::max/std::min: their NaN behaviour is not guaranteed to match.
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar < t1 ? tFar : t1;

        // Strict `<` so that a flat box (min == max on this axis) still hits.
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
