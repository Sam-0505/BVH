#pragma once

#include <cmath>
#include <limits>

namespace geom {

// float, not double: a BVH is memory-bound, and halving AABB width halves the
// bytes pulled through cache during traversal.
using Scalar = float;

// The intersection routines depend on IEEE NaN/infinity behaviour, so a
// -ffast-math build would produce wrong answers rather than slow ones.
static_assert(std::numeric_limits<Scalar>::is_iec559,
              "geometry core requires IEEE-754 floating point");

inline constexpr Scalar kInfinity = std::numeric_limits<Scalar>::infinity();
inline constexpr Scalar kPi = Scalar(3.14159265358979323846);

// General-purpose "close enough" tolerance. Intersection tests state their own.
inline constexpr Scalar kEpsilon = Scalar(1e-6);

// Higham/PBRT error bound: |computed - exact| <= gamma(n) * |exact| after n ops.
inline constexpr Scalar gamma(int n) {
    const Scalar e = std::numeric_limits<Scalar>::epsilon() * Scalar(0.5);
    return (Scalar(n) * e) / (Scalar(1) - Scalar(n) * e);
}

// 1/v, giving IEEE's +/-inf for v == 0 without the divide that [expr.mul]/4
// leaves undefined. One branch, paid once per ray rather than per node.
inline Scalar safeReciprocal(Scalar v) {
    if (v != Scalar(0)) return Scalar(1) / v;
    return std::copysign(kInfinity, v);
}

inline constexpr Scalar radians(Scalar degrees) { return degrees * (kPi / Scalar(180)); }
inline constexpr Scalar degrees(Scalar radians) { return radians * (Scalar(180) / kPi); }

// Absolute near zero, relative for large magnitudes. Tests and assertions only.
inline bool nearlyEqual(Scalar a, Scalar b, Scalar tol = kEpsilon) {
    const Scalar diff = std::fabs(a - b);
    if (diff <= tol) return true;
    return diff <= tol * std::fmax(std::fabs(a), std::fabs(b));
}

}  // namespace geom
