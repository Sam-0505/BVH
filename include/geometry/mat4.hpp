#pragma once

#include <cassert>

#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"
#include "geometry/vec4.hpp"

namespace geom {

// COLUMN-MAJOR: m[c][r] is column c, row r. Matches GLSL/SPIR-V, so a Mat4
// memcpy's into a uniform buffer with no transpose.
//
// Column vectors: apply as M * v, and "first A, then B" composes as B * A.
struct Mat4 {
    // m[column][row]
    Scalar m[4][4]{};

    constexpr Mat4() = default;

    static constexpr Mat4 identity() {
        Mat4 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = Scalar(1);
        return r;
    }

    static constexpr Mat4 zero() { return Mat4{}; }

    // Build from columns, which is the storage order.
    static constexpr Mat4 fromColumns(const Vec4& c0, const Vec4& c1, const Vec4& c2,
                                      const Vec4& c3) {
        Mat4 r;
        for (int i = 0; i < 4; ++i) {
            r.m[0][i] = c0[i];
            r.m[1][i] = c1[i];
            r.m[2][i] = c2[i];
            r.m[3][i] = c3[i];
        }
        return r;
    }

    constexpr Vec4 column(int c) const {
        assert(c >= 0 && c < 4);
        return {m[c][0], m[c][1], m[c][2], m[c][3]};
    }
    constexpr Vec4 row(int r) const {
        assert(r >= 0 && r < 4);
        return {m[0][r], m[1][r], m[2][r], m[3][r]};
    }

    friend constexpr bool operator==(const Mat4& a, const Mat4& b) {
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                if (a.m[c][r] != b.m[c][r]) return false;
        return true;
    }
    friend constexpr bool operator!=(const Mat4& a, const Mat4& b) { return !(a == b); }
};

// --- Construction of standard transforms ------------------------------------

Mat4 translation(const Vec3& t);
Mat4 scaling(const Vec3& s);

// Right-handed, Rodrigues' formula, axis need not be unit. A zero-length axis
// gives the identity rather than NaN.
Mat4 rotation(const Vec3& axis, Scalar angle);

Mat4 rotationX(Scalar angle);
Mat4 rotationY(Scalar angle);
Mat4 rotationZ(Scalar angle);

// --- Arithmetic --------------------------------------------------------------

Mat4 operator*(const Mat4& a, const Mat4& b);
Vec4 operator*(const Mat4& a, const Vec4& v);

Mat4 transpose(const Mat4& a);
Scalar determinant(const Mat4& a);

// Gauss-Jordan with SCALED partial pivoting. Cofactor expansion would be faster
// for a fixed 4x4 but less stable, and this is not a hot path -- while the
// error lands on object-space rays, where it becomes a wrong intersection.
//
// Returns false only when elimination hits an exactly-zero pivot. A NEAR-
// singular matrix is accepted and gives a large-but-finite inverse; measured,
// that admits ~88% of degeneracies produced by arithmetic. Test determinant()
// if you need a conditioning guarantee.
//
// No magnitude tolerance, deliberately: a homogeneous transform mixes units, so
// there is no scale to compare a pivot against. The implementation records the
// three thresholds tried and what each one wrongly rejected.
bool invert(const Mat4& a, Mat4& out);

// Convenience wrapper: asserts invertibility in debug, returns identity for a
// singular matrix in release. Prefer invert() where failure is possible.
Mat4 inverse(const Mat4& a);

// --- Applying transforms -----------------------------------------------------

// POSITION: w = 1, so translation applies, with a perspective divide when w
// comes back as anything else. Three cases, not two -- w == 0 also skips the
// divide and returns raw xyz, since a point on the eye plane has no finite
// projection and infinities would be worse than a finite wrong answer.
Vec3 transformPoint(const Mat4& a, const Vec3& p);

// DIRECTION: w = 0, no translation. Length is not preserved under scaling, by
// design -- a ray direction must scale for its t values to stay meaningful.
Vec3 transformVector(const Mat4& a, const Vec3& v);

// NORMAL: inverse transpose of the upper-left 3x3. A normal is a covector, so
// under non-uniform scale it does not transform like a direction -- using
// transformVector is the classic bug that leaves normals off-perpendicular.
//
// Takes the already-inverted matrix, so a caller transforming many normals
// inverts once. The result is NOT unit length: perpendicularity is preserved,
// magnitude is not.
Vec3 transformNormalWithInverse(const Mat4& inverseTransform, const Vec3& n);

inline bool nearlyEqual(const Mat4& a, const Mat4& b, Scalar tol = kEpsilon) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            if (!nearlyEqual(a.m[c][r], b.m[c][r], tol)) return false;
    return true;
}

}  // namespace geom
