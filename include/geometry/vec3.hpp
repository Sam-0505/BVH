#pragma once

#include <cassert>
#include <cmath>

#include "geometry/scalar.hpp"

namespace geom {

// A 3-component vector used for both points and directions.
//
// Storage is three named scalars rather than an array, because named access
// (v.x) dominates in readability. Indexed access is still required -- a BVH
// splits along a runtime-chosen axis -- so operator[] is provided, implemented
// with a conditional chain rather than the common `(&x)[i]` trick.
//
// `(&x)[i]` is undefined behaviour: pointer arithmetic is only defined within a
// single object, and three separate members are not an array however they are
// laid out. The conditional form below is well-defined, and because a ternary of
// lvalues is itself an lvalue it still yields a real reference. Optimisers turn
// it into the same one or two instructions for a constant index, and a cmov or
// small branch for a runtime one.
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

    // Exact bitwise-value comparison. Useful for tests over exactly-representable
    // values and for detecting "unchanged"; never use it to compare results of
    // floating-point arithmetic -- use nearlyEqual() for that.
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
// Division by a zero scalar is undefined behaviour ([expr.mul]/4) even for
// floating point, where IEEE-754 would give +/-inf. The zero case is branched
// out and the IEEE answer reproduced by multiplying by the signed infinity:
// x * (+/-inf) is +/-inf with the correct sign, and 0 * inf is NaN -- exactly
// what x/0 and 0/0 produce.
//
// The non-zero path keeps a true divide rather than multiplying by a
// reciprocal everywhere. Multiply-by-reciprocal is faster but adds a second
// rounding step, and it overflows to infinity for a denormal divisor where the
// true quotient is finite. This is not a hot path, so correctness wins.
inline Vec3 operator/(const Vec3& v, Scalar s) {
    if (s == Scalar(0)) {
        const Scalar inf = safeReciprocal(s);
        return {v.x * inf, v.y * inf, v.z * inf};
    }
    return {v.x / s, v.y / s, v.z / s};
}

// Defined out of line because it forwards to the guarded operator/ above, which
// is not visible from inside the class body.
inline Vec3& Vec3::operator/=(Scalar s) {
    *this = *this / s;
    return *this;
}

// Component-wise (Hadamard) product. Distinct from dot/cross; used for scaling
// along axes and for the ray/slab test's reciprocal-direction multiply.
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

// Precondition: v is not the zero vector.
//
// A zero-length input is a programming error, caught by the assert in a debug
// build. In a release build the assert is gone, so the division must still be
// well-defined -- operator/ above guarantees that, yielding inf/NaN rather than
// undefined behaviour. Callers that cannot guarantee a non-degenerate input
// must use normalizeSafe() instead of relying on that fallback.
inline Vec3 normalize(const Vec3& v) {
    const Scalar len = length(v);
    assert(len > Scalar(0) && "normalize() on a zero-length vector");
    return v / len;
}

// Degenerate-tolerant normalize: returns `fallback` when v is too short to have a
// meaningful direction. Needed for mesh data, where degenerate triangles produce
// zero-length face normals.
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

// Index of the largest component. Ties resolve to the lowest index, which keeps
// BVH split-axis selection deterministic for symmetric bounds -- important for
// reproducible benchmarks.
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
