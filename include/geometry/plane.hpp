#pragma once

#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// Constant-normal form: dot(normal, p) + d == 0. Storing d makes the signed
// distance a dot and an add, which is the form frustum culling wants.
struct Plane {
    Vec3 normal{Scalar(0), Scalar(1), Scalar(0)};
    Scalar d{Scalar(0)};

    constexpr Plane() = default;
    constexpr Plane(const Vec3& n, Scalar dv) : normal(n), d(dv) {}

    // normalizeSafe because `n` is caller-supplied: a zero normal gives a
    // degenerate plane, not a NaN.
    static Plane fromPointNormal(const Vec3& point, const Vec3& n) {
        const Vec3 unit = normalizeSafe(n);
        return Plane(unit, -dot(unit, point));
    }

    // CCW from the side the normal faces. Collinear input gives a degenerate
    // plane rather than NaN.
    static Plane fromPoints(const Vec3& a, const Vec3& b, const Vec3& c) {
        const Vec3 n = normalizeSafe(cross(b - a, c - a));
        return Plane(n, -dot(n, a));
    }

    // Positive on the normal's side. Only a true distance for a unit normal,
    // which the factories guarantee and the raw constructor does not.
    constexpr Scalar signedDistance(const Vec3& p) const { return dot(normal, p) + d; }

    constexpr bool isDegenerate() const { return normal == Vec3(Scalar(0)); }
};

}  // namespace geom
