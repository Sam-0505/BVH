#pragma once

#include <cassert>

#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// 4-component vector, used almost exclusively as a homogeneous coordinate in
// Mat4 arithmetic. Points carry w = 1 (translation applies); directions carry
// w = 0 (translation does not). Keeping that distinction in the type is what
// makes transformPoint and transformVector separate, explicit operations rather
// than a single function with a silent convention.
struct Vec4 {
    Scalar x{};
    Scalar y{};
    Scalar z{};
    Scalar w{};

    constexpr Vec4() = default;
    constexpr Vec4(Scalar xv, Scalar yv, Scalar zv, Scalar wv) : x(xv), y(yv), z(zv), w(wv) {}
    constexpr Vec4(const Vec3& v, Scalar wv) : x(v.x), y(v.y), z(v.z), w(wv) {}

    constexpr Scalar& operator[](int i) {
        assert(i >= 0 && i < 4);
        return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w));
    }
    constexpr const Scalar& operator[](int i) const {
        assert(i >= 0 && i < 4);
        return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w));
    }

    constexpr Vec3 xyz() const { return {x, y, z}; }

    friend constexpr bool operator==(const Vec4& a, const Vec4& b) {
        return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
    }
    friend constexpr bool operator!=(const Vec4& a, const Vec4& b) { return !(a == b); }
};

constexpr Vec4 operator+(const Vec4& a, const Vec4& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
constexpr Vec4 operator-(const Vec4& a, const Vec4& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}
constexpr Vec4 operator*(const Vec4& v, Scalar s) { return {v.x * s, v.y * s, v.z * s, v.w * s}; }
constexpr Vec4 operator*(Scalar s, const Vec4& v) { return v * s; }

constexpr Scalar dot(const Vec4& a, const Vec4& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline bool nearlyEqual(const Vec4& a, const Vec4& b, Scalar tol = kEpsilon) {
    return nearlyEqual(a.x, b.x, tol) && nearlyEqual(a.y, b.y, tol) &&
           nearlyEqual(a.z, b.z, tol) && nearlyEqual(a.w, b.w, tol);
}

}  // namespace geom
