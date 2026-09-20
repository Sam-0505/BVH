#pragma once

#include <cmath>
#include <limits>

// Scalar type and numerical constants shared by the geometry core.
//
// Everything is templated on nothing and hard-typed to `Scalar` instead. A single
// typedef keeps the door open for a double-precision build without touching call
// sites, while keeping the common case (float) free of template noise.
//
// Why float and not double: a BVH is memory-bound, not ALU-bound. Every AABB is
// 6 scalars; halving their width halves the bytes pulled through cache during
// traversal, which is where the time actually goes. The cost is ~7 decimal digits
// of precision, which is why the intersection routines below are written to be
// robust rather than assuming exact arithmetic.

namespace geom {

using Scalar = float;

// The intersection routines rely on IEEE-754 semantics: infinities that compare
// correctly, NaNs that make every comparison false, and signed zero. A platform
// or a -ffast-math build that breaks these produces wrong results rather than
// slow ones, so the assumption is checked rather than assumed.
static_assert(std::numeric_limits<Scalar>::is_iec559,
              "geometry core requires IEEE-754 floating point");

inline constexpr Scalar kInfinity = std::numeric_limits<Scalar>::infinity();
inline constexpr Scalar kPi = Scalar(3.14159265358979323846);

// A general-purpose tolerance for "are these two quantities the same". Deliberately
// NOT used inside intersection tests -- those state their own tolerance, because a
// single global epsilon is meaningless across different magnitudes.
inline constexpr Scalar kEpsilon = Scalar(1e-6);

// gamma(n) from Higham's error analysis, as used in PBRT: a conservative bound on
// the relative error accumulated after n floating-point operations. Used to widen
// intervals just enough that a conservative test never reports a false miss.
//
//   |computed - exact| <= gamma(n) * |exact|
inline constexpr Scalar gamma(int n) {
    const Scalar e = std::numeric_limits<Scalar>::epsilon() * Scalar(0.5);
    return (Scalar(n) * e) / (Scalar(1) - Scalar(n) * e);
}

// Reciprocal that yields the IEEE-754 result for a zero input (+/-inf, with the
// sign of the zero) WITHOUT performing a division by zero.
//
// [expr.mul]/4 makes `x / 0` undefined behaviour in C++ regardless of operand
// type -- the IEEE guarantee of +/-inf is a property of the hardware, not of the
// abstract machine, and UBSan's float-divide-by-zero check flags it. Ray/AABB
// traversal genuinely wants those infinities (see intersectRay), so rather than
// suppress the check we produce the same value by construction.
//
// Cost: one predictable branch. This runs once per ray, not once per node
// visited, so it does not appear in the traversal inner loop.
inline Scalar safeReciprocal(Scalar v) {
    if (v != Scalar(0)) return Scalar(1) / v;
    // copysign propagates the sign of a negative zero, matching 1 / -0.0 == -inf.
    return std::copysign(kInfinity, v);
}

inline constexpr Scalar radians(Scalar degrees) { return degrees * (kPi / Scalar(180)); }
inline constexpr Scalar degrees(Scalar radians) { return radians * (Scalar(180) / kPi); }

// Absolute-or-relative comparison. Absolute near zero (where relative error is
// meaningless), relative for large magnitudes (where an absolute epsilon is too
// strict). Test-support and assertion use; not for hot paths.
inline bool nearlyEqual(Scalar a, Scalar b, Scalar tol = kEpsilon) {
    const Scalar diff = std::fabs(a - b);
    if (diff <= tol) return true;
    return diff <= tol * std::fmax(std::fabs(a), std::fabs(b));
}

}  // namespace geom
