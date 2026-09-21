#pragma once

#include "geometry/mat4.hpp"
#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// p(t) = origin + t * direction, for t in [tMin, tMax].
//
// direction is deliberately not unit length, so t is measured in units of
// |direction|. That is what lets t survive a scaling transform unchanged when a
// ray is moved into object space.
//
// tMax lives in the ray so traversal can shrink it as closer hits are found --
// the main pruning mechanism in a BVH.
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

// Computed once per ray, turning three divides per AABB into three multiplies.
// Zero components give +/-inf, which the slab test consumes correctly.
inline Vec3 invDirection(const Ray& r) {
    return {safeReciprocal(r.direction.x), safeReciprocal(r.direction.y),
            safeReciprocal(r.direction.z)};
}

// World -> object, typically. The direction is not renormalised, which keeps t
// comparable between the two spaces.
inline Ray transformRay(const Mat4& xf, const Ray& r) {
    return Ray(transformPoint(xf, r.origin), transformVector(xf, r.direction), r.tMin, r.tMax);
}

}  // namespace geom
