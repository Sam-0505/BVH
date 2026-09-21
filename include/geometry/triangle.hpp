#pragma once

#include <cmath>

#include "geometry/aabb.hpp"
#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// Three explicit vertices -- the unpacked form an intersection routine wants.
// Mesh stores the indexed form and materialises these on demand.
// Winding is CCW from the front, so the normal is cross(v1-v0, v2-v0).
struct Triangle {
    Vec3 v0{};
    Vec3 v1{};
    Vec3 v2{};

    constexpr Triangle() = default;
    constexpr Triangle(const Vec3& a, const Vec3& b, const Vec3& c) : v0(a), v1(b), v2(c) {}

    constexpr Vec3 edge01() const { return v1 - v0; }
    constexpr Vec3 edge02() const { return v2 - v0; }

    // Magnitude is twice the area. Unnormalised so a sign test can skip the sqrt.
    constexpr Vec3 normalUnnormalized() const { return cross(edge01(), edge02()); }

    // Zero vector for a degenerate triangle, never NaN.
    Vec3 normal() const { return normalizeSafe(normalUnnormalized()); }

    Scalar area() const { return Scalar(0.5) * length(normalUnnormalized()); }

    // Dimensionless on purpose: |cross| <= tol * L^2 for the longest edge L is
    // h/L <= tol, an aspect ratio invariant under scaling. An absolute area
    // threshold would flag healthy small triangles -- 1e-3 edges give 5e-7 area.
    bool isDegenerate(Scalar tol = kEpsilon) const {
        const Scalar longestEdgeSq =
            std::fmax(lengthSquared(edge01()),
                      std::fmax(lengthSquared(edge02()), lengthSquared(v2 - v1)));
        // All three vertices coincide: no edge, no shape, definitively degenerate.
        if (longestEdgeSq == Scalar(0)) return true;
        const Scalar limit = tol * longestEdgeSq;
        return lengthSquared(normalUnnormalized()) <= limit * limit;
    }

    // Exact -- a triangle's extremes are always at its vertices.
    AABB bounds() const {
        AABB b;
        b.extend(v0);
        b.extend(v1);
        b.extend(v2);
        return b;
    }

    // BVH construction partitions on centroids: one position per primitive to
    // sort by, where overlapping bounds give no ordering.
    constexpr Vec3 centroid() const {
        return (v0 + v1 + v2) * (Scalar(1) / Scalar(3));
    }

    friend constexpr bool operator==(const Triangle& a, const Triangle& b) {
        return a.v0 == b.v0 && a.v1 == b.v1 && a.v2 == b.v2;
    }
    friend constexpr bool operator!=(const Triangle& a, const Triangle& b) { return !(a == b); }
};

}  // namespace geom
