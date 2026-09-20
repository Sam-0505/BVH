#pragma once

#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// Plane in constant-normal form: dot(normal, p) + d == 0.
//
// Storing d rather than a point on the plane makes the signed-distance query a
// single dot product and an add, with no subtraction of a reference point. This
// is the form frustum culling wants, which is why it is chosen here even though
// the geometry core does not yet cull.
struct Plane {
    Vec3 normal{Scalar(0), Scalar(1), Scalar(0)};
    Scalar d{Scalar(0)};

    constexpr Plane() = default;
    constexpr Plane(const Vec3& n, Scalar dv) : normal(n), d(dv) {}

    // Plane through `point` with the given normal.
    //
    // Uses normalizeSafe for the same reason fromPoints does: `n` is
    // caller-supplied and a zero normal is a degenerate plane, not a crash or a
    // NaN. Check isDegenerate() if the distinction matters.
    static Plane fromPointNormal(const Vec3& point, const Vec3& n) {
        const Vec3 unit = normalizeSafe(n);
        return Plane(unit, -dot(unit, point));
    }

    // Plane through three points, wound counter-clockwise when viewed from the
    // side the normal points toward. Returns a degenerate plane (zero normal)
    // for collinear input rather than NaN.
    static Plane fromPoints(const Vec3& a, const Vec3& b, const Vec3& c) {
        const Vec3 n = normalizeSafe(cross(b - a, c - a));
        return Plane(n, -dot(n, a));
    }

    // Signed distance, positive on the side the normal points toward. Only a
    // true distance when `normal` is unit length, which the factory functions
    // guarantee but the raw constructor does not.
    constexpr Scalar signedDistance(const Vec3& p) const { return dot(normal, p) + d; }

    constexpr bool isDegenerate() const { return normal == Vec3(Scalar(0)); }
};

}  // namespace geom
