#pragma once

#include "geometry/mat4.hpp"
#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// A parametric ray: p(t) = origin + t * direction, valid for t in [tMin, tMax].
//
// `direction` is deliberately NOT required to be unit length. Two reasons:
//   1. Normalising costs a sqrt per ray, and BVH traversal never needs it -- the
//      slab test works with any direction scale.
//   2. When a ray is transformed into an object's local space, the transform may
//      scale it. Renormalising there would silently change the meaning of t, so
//      hit distances found in local space would no longer match world space.
// The consequence is that t is measured in units of |direction|, not in world
// distance. Callers that need true distance must normalise up front and accept
// the sqrt.
//
// [tMin, tMax] is carried in the ray rather than passed alongside it so that
// traversal can shrink tMax as closer hits are found. That shrinking is the
// single most effective pruning mechanism in a BVH: once a hit at t is known,
// every subtree whose entry distance exceeds t can be skipped outright.
struct Ray {
    Vec3 origin{};
    Vec3 direction{Scalar(0), Scalar(0), Scalar(1)};
    Scalar tMin{Scalar(0)};
    Scalar tMax{kInfinity};

    constexpr Ray() = default;
    constexpr Ray(const Vec3& o, const Vec3& d, Scalar tmin = Scalar(0), Scalar tmax = kInfinity)
        : origin(o), direction(d), tMin(tmin), tMax(tmax) {}

    constexpr Vec3 at(Scalar t) const { return origin + direction * t; }
};

// Component-wise reciprocal of the direction, for the slab test.
//
// Computed once per ray and reused across every node visited, turning three
// divides per AABB into three multiplies.
//
// A zero direction component yields +/-inf by design -- the slab test is written
// to consume infinities correctly, and branching to avoid them inside traversal
// would cost more than it saves. safeReciprocal produces those infinities
// without a literal division by zero; see its comment for why that distinction
// matters.
inline Vec3 invDirection(const Ray& r) {
    return {safeReciprocal(r.direction.x), safeReciprocal(r.direction.y),
            safeReciprocal(r.direction.z)};
}

// Transform a ray into another space (typically world -> object, using the
// inverse of the object's transform).
//
// The origin transforms as a point and the direction as a vector. The direction
// is not renormalised, which is what makes t values comparable between the two
// spaces -- see the note on Ray::direction above.
inline Ray transformRay(const Mat4& xf, const Ray& r) {
    return Ray(transformPoint(xf, r.origin), transformVector(xf, r.direction), r.tMin, r.tMax);
}

}  // namespace geom
