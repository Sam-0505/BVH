#pragma once

#include "geometry/aabb.hpp"
#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// A triangle stored as three explicit vertices.
//
// This is the "fat" representation: 36 bytes of position data with no indirection.
// The Mesh class stores indexed vertices instead (shared vertices stored once),
// and materialises a Triangle on demand. That split is intentional -- indexed
// storage is the compact form for holding a mesh, while the unpacked form is
// what an intersection routine wants, since chasing indices during traversal
// costs an extra dependent memory access per test.
//
// Winding is counter-clockwise when viewed from the front face, which puts the
// normal along cross(v1 - v0, v2 - v0) by the right-hand rule.
struct Triangle {
    Vec3 v0{};
    Vec3 v1{};
    Vec3 v2{};

    constexpr Triangle() = default;
    constexpr Triangle(const Vec3& a, const Vec3& b, const Vec3& c) : v0(a), v1(b), v2(c) {}

    constexpr Vec3 edge01() const { return v1 - v0; }
    constexpr Vec3 edge02() const { return v2 - v0; }

    // Unnormalised geometric normal. Its magnitude is twice the triangle area,
    // which is why area() below reuses it rather than computing separately.
    // Returned unnormalised so callers that only need an orientation test (a
    // sign) can skip the sqrt.
    constexpr Vec3 normalUnnormalized() const { return cross(edge01(), edge02()); }

    // Unit normal, or the zero vector for a degenerate triangle. Degenerate
    // input is real -- exported meshes routinely contain zero-area triangles
    // from welded or duplicated vertices -- so this must not return NaN.
    Vec3 normal() const { return normalizeSafe(normalUnnormalized()); }

    Scalar area() const { return Scalar(0.5) * length(normalUnnormalized()); }

    // A triangle is degenerate when its vertices are collinear (or coincident),
    // giving it zero area and no well-defined normal. Such triangles cannot be
    // hit by a ray in any meaningful sense, but they still have valid bounds and
    // a valid centroid, so a BVH can carry them without special-casing -- they
    // simply never report a hit.
    //
    // Tested on squared area against a squared tolerance to avoid a sqrt.
    bool isDegenerate(Scalar tol = kEpsilon) const {
        return lengthSquared(normalUnnormalized()) <= (Scalar(2) * tol) * (Scalar(2) * tol);
    }

    // Tight axis-aligned bounds. Exact: the extremes of a triangle are always
    // at its vertices, so no sampling or refinement is needed.
    AABB bounds() const {
        AABB b;
        b.extend(v0);
        b.extend(v1);
        b.extend(v2);
        return b;
    }

    // Centroid (barycentre). BVH construction partitions on centroids rather
    // than on bounds, because a centroid gives each primitive exactly one
    // position to sort by, whereas overlapping bounds do not induce an ordering.
    constexpr Vec3 centroid() const {
        return (v0 + v1 + v2) * (Scalar(1) / Scalar(3));
    }

    friend constexpr bool operator==(const Triangle& a, const Triangle& b) {
        return a.v0 == b.v0 && a.v1 == b.v1 && a.v2 == b.v2;
    }
    friend constexpr bool operator!=(const Triangle& a, const Triangle& b) { return !(a == b); }
};

}  // namespace geom
