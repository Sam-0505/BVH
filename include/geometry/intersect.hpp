#pragma once

#include "geometry/ray.hpp"
#include "geometry/scalar.hpp"
#include "geometry/triangle.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// (u, v) are barycentric: the hit is v0 + u*(v1-v0) + v*(v2-v0). Stored rather
// than the position because that is what interpolating attributes needs.
struct TriangleHit {
    Scalar t{};
    Scalar u{};
    Scalar v{};

    Vec3 position(const Ray& r) const { return r.at(t); }

    // From the triangle instead of the ray -- more accurate for a distant hit.
    Vec3 position(const Triangle& tri) const {
        return tri.v0 + tri.edge01() * u + tri.edge02() * v;
    }
};

// Moller-Trumbore, two-sided. Solves for barycentrics directly, so it stores no
// precomputed plane -- 16 fewer bytes per primitive, which matters in a
// memory-bound traversal. Two crosses, four dots, one divide.
//
// No epsilon on the determinant: det scales with |e1||e2||dir|, so a fixed
// cutoff means different things at different scales. Instead the rejections run
// on unscaled barycentrics with the sign of det folded in, which is exactly
// scale-invariant and divides only after every rejection. A near-degenerate
// triangle needs no special case -- its det is tiny, so any real u or v exceeds
// it. Only det == 0 is tested exactly: the ray lies in the plane, and there is
// no single intersection point to report.
inline bool intersectRayTriangle(const Triangle& tri, const Vec3& origin, const Vec3& direction,
                                 Scalar tMin, Scalar tMax, TriangleHit& hit) {
    const Vec3 e1 = tri.edge01();
    const Vec3 e2 = tri.edge02();

    const Vec3 pvec = cross(direction, e2);
    const Scalar det = dot(e1, pvec);

    // Ray parallel to the triangle's plane, or the triangle is degenerate.
    if (det == Scalar(0)) return false;

    const Vec3 tvec = origin - tri.v0;
    const Vec3 qvec = cross(tvec, e1);

    // u = uNum/det, v = vNum/det, t = tNum/det, all still unscaled.
    const Scalar uNum = dot(tvec, pvec);
    const Scalar vNum = dot(direction, qvec);
    const Scalar tNum = dot(e2, qvec);

    // A negative det reverses each inequality, so the signs are split out.
    if (det > Scalar(0)) {
        if (uNum < Scalar(0) || vNum < Scalar(0) || uNum + vNum > det) return false;
    } else {
        if (uNum > Scalar(0) || vNum > Scalar(0) || uNum + vNum < det) return false;
    }

    const Scalar invDet = Scalar(1) / det;
    const Scalar t = tNum * invDet;
    if (t < tMin || t > tMax) return false;

    hit.t = t;
    hit.u = uNum * invDet;
    hit.v = vNum * invDet;
    return true;
}

inline bool intersectRayTriangle(const Triangle& tri, const Ray& r, TriangleHit& hit) {
    return intersectRayTriangle(tri, r.origin, r.direction, r.tMin, r.tMax, hit);
}

}  // namespace geom
