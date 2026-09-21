#pragma once

#include <cassert>
#include <cmath>

#include "geometry/scalar.hpp"

namespace geom {

// operator[] uses a conditional chain rather than the usual `(&x)[i]`, which is
// UB -- three members are not an array. Clang compiles it to a cmov, and it
// costs nothing at all where the index is a constant.
struct Vec3 {
    Scalar x{};
    Scalar y{};
    Scalar z{};

    constexpr Vec3() = default;
    constexpr Vec3(Scalar xv, Scalar yv, Scalar zv) : x(xv), y(yv), z(zv) {}
    explicit constexpr Vec3(Scalar s) : x(s), y(s), z(s) {}

    constexpr Scalar& operator[](int i) {
        assert(i >= 0 && i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }
    constexpr const Scalar& operator[](int i) const {
        assert(i >= 0 && i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }

    constexpr Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(Scalar s) { x *= s; y *= s; z *= s; return *this; }
    Vec3& operator/=(Scalar s);

    // Exact comparison. Use nearlyEqual() for anything arithmetic produced.
    friend constexpr bool operator==(const Vec3& a, const Vec3& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
    friend constexpr bool operator!=(const Vec3& a, const Vec3& b) { return !(a == b); }
};

constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator-(const Vec3& v) { return {-v.x, -v.y, -v.z}; }
constexpr Vec3 operator*(const Vec3& v, Scalar s) { return {v.x * s, v.y * s, v.z * s}; }
constexpr Vec3 operator*(Scalar s, const Vec3& v) { return v * s; }
// Divide by zero is UB per [expr.mul]/4 even for float, so the zero case is
// branched out and IEEE's answer reproduced by multiplying by signed infinity.
// The normal path keeps a true divide: multiply-by-reciprocal would add a
// rounding step and overflow for denormal divisors.
inline Vec3 operator/(const Vec3& v, Scalar s) {
    if (s == Scalar(0)) {
        const Scalar inf = safeReciprocal(s);
        return {v.x * inf, v.y * inf, v.z * inf};
    }
    return {v.x / s, v.y / s, v.z / s};
}

// Out of line: it forwards to the guarded operator/ declared below the class.
inline Vec3& Vec3::operator/=(Scalar s) {
    *this = *this / s;
    return *this;
}

// Component-wise product.
constexpr Vec3 mul(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }

constexpr Scalar dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Right-handed cross product: cross(X, Y) == Z.
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

constexpr Scalar lengthSquared(const Vec3& v) { return dot(v, v); }
inline Scalar length(const Vec3& v) { return std::sqrt(lengthSquared(v)); }

// Precondition: v is non-zero. Release builds drop the assert, so operator/
// still has to be well-defined there -- it is. Use normalizeSafe() when a
// degenerate input is actually possible.
inline Vec3 normalize(const Vec3& v) {
    const Scalar len = length(v);
    assert(len > Scalar(0) && "normalize() on a zero-length vector");
    return v / len;
}

// Returns `fallback` for a vector too short to have a direction -- real mesh
// data has zero-area triangles with zero-length normals.
inline Vec3 normalizeSafe(const Vec3& v, const Vec3& fallback = Vec3(Scalar(0))) {
    const Scalar lenSq = lengthSquared(v);
    if (lenSq <= kEpsilon * kEpsilon) return fallback;
    return v / std::sqrt(lenSq);
}

inline Vec3 minComponents(const Vec3& a, const Vec3& b) {
    return {std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z)};
}
inline Vec3 maxComponents(const Vec3& a, const Vec3& b) {
    return {std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z)};
}
inline Vec3 abs(const Vec3& v) { return {std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)}; }

constexpr Scalar minComponent(const Vec3& v) {
    return v.x < v.y ? (v.x < v.z ? v.x : v.z) : (v.y < v.z ? v.y : v.z);
}
constexpr Scalar maxComponent(const Vec3& v) {
    return v.x > v.y ? (v.x > v.z ? v.x : v.z) : (v.y > v.z ? v.y : v.z);
}

// Ties go to the lowest index, so split-axis choice stays deterministic.
constexpr int maxAxis(const Vec3& v) {
    if (v.x >= v.y && v.x >= v.z) return 0;
    return v.y >= v.z ? 1 : 2;
}

inline bool nearlyEqual(const Vec3& a, const Vec3& b, Scalar tol = kEpsilon) {
    return nearlyEqual(a.x, b.x, tol) && nearlyEqual(a.y, b.y, tol) && nearlyEqual(a.z, b.z, tol);
}

inline bool isFinite(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace geom
