#pragma once

#include <cassert>

#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"
#include "geometry/vec4.hpp"

namespace geom {

// 4x4 matrix in COLUMN-MAJOR storage: m[c][r] is column c, row r.
//
// Column-major is chosen to match GLSL/SPIR-V, so a Mat4 can be memcpy'd into a
// uniform buffer and consumed by a Vulkan shader with no transpose. The cost is
// that the in-memory order does not match how a matrix is written on paper, so
// every access site has to be clear about which index is which -- hence m[c][r]
// everywhere rather than a flat array.
//
// Convention: column vectors, so a transform is applied as M * v, and composing
// "first A, then B" is B * A.
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

// Right-handed rotation by `angle` radians about a (not necessarily unit) axis,
// via Rodrigues' rotation formula. A zero-length axis yields the identity rather
// than NaN.
Mat4 rotation(const Vec3& axis, Scalar angle);

Mat4 rotationX(Scalar angle);
Mat4 rotationY(Scalar angle);
Mat4 rotationZ(Scalar angle);

// --- Arithmetic --------------------------------------------------------------

Mat4 operator*(const Mat4& a, const Mat4& b);
Vec4 operator*(const Mat4& a, const Vec4& v);

Mat4 transpose(const Mat4& a);
Scalar determinant(const Mat4& a);

// Inverse via Gauss-Jordan elimination with SCALED partial pivoting.
//
// Cofactor expansion is the usual choice for a fixed 4x4 and is faster, but it is
// less numerically stable and this is not a hot path -- transforms are built once
// per frame at most, not once per ray. Stability matters more here because the
// inverse is used to bring rays into object space, where error becomes a
// wrong intersection rather than a slightly wrong pixel.
//
// Returns false and leaves `out` untouched when the matrix is singular to within
// the pivot tolerance. That tolerance is applied to each pivot RELATIVE TO ITS
// OWN ROW, which is scale-invariant: an absolute threshold would reject a
// uniformly tiny but perfectly invertible matrix, and a whole-matrix threshold
// fails specifically on homogeneous transforms, where m[3][3] == 1 dwarfs a
// small linear part. See the implementation for the full argument.
bool invert(const Mat4& a, Mat4& out);

// Convenience wrapper: asserts invertibility in debug, returns identity for a
// singular matrix in release. Prefer invert() where failure is possible.
Mat4 inverse(const Mat4& a);

// --- Applying transforms -----------------------------------------------------

// Transform a POSITION: w = 1, so translation applies. Performs the perspective
// divide when the resulting w is not 1, which makes this correct for projection
// matrices as well as affine ones.
//
// Three cases, not two: w == 1 skips the divide (the affine fast path), w == 0
// also skips it and returns the raw xyz. A point that projects to w == 0 lies
// on the eye plane and has no finite projected position, so there is no correct
// answer to return; the raw direction-like vector is at least finite, where
// dividing would give infinities or NaN.
Vec3 transformPoint(const Mat4& a, const Vec3& p);

// Transform a DIRECTION: w = 0, so translation does not apply. Length is NOT
// preserved under scaling -- that is deliberate, because a ray direction must
// scale with the transform for its t values to stay meaningful.
Vec3 transformVector(const Mat4& a, const Vec3& v);

// Transform a NORMAL using the inverse transpose of the upper-left 3x3.
//
// A normal is a covector: under a non-uniform scale it does not transform like a
// direction. Scaling x by 2 halves the x component of a surface normal rather
// than doubling it, so using transformVector here is the classic bug that leaves
// normals non-perpendicular to their surface.
//
// Takes the ALREADY-INVERTED matrix so callers transforming many normals do not
// pay for a matrix inverse each time.
//
// THE RESULT IS NOT UNIT LENGTH, even for a unit input: the inverse transpose
// preserves perpendicularity, not magnitude. A unit normal through the inverse
// of a uniform 4x scale comes back with length 1/4. Normalize the result if you
// need a unit normal; shading does, a sidedness test does not.
Vec3 transformNormalWithInverse(const Mat4& inverseTransform, const Vec3& n);

inline bool nearlyEqual(const Mat4& a, const Mat4& b, Scalar tol = kEpsilon) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            if (!nearlyEqual(a.m[c][r], b.m[c][r], tol)) return false;
    return true;
}

}  // namespace geom
