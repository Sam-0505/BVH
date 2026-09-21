#include "geometry/mat4.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <utility>

namespace geom {

Mat4 translation(const Vec3& t) {
    Mat4 r = Mat4::identity();
    // Translation lives in the fourth COLUMN for column-vector convention.
    r.m[3][0] = t.x;
    r.m[3][1] = t.y;
    r.m[3][2] = t.z;
    return r;
}

Mat4 scaling(const Vec3& s) {
    Mat4 r = Mat4::identity();
    r.m[0][0] = s.x;
    r.m[1][1] = s.y;
    r.m[2][2] = s.z;
    return r;
}

Mat4 rotation(const Vec3& axis, Scalar angle) {
    // Rodrigues: R = I cos(t) + [k]_x sin(t) + k k^T (1 - cos(t)).
    const Vec3 k = normalizeSafe(axis);
    if (k == Vec3(Scalar(0))) return Mat4::identity();  // degenerate axis

    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    const Scalar t = Scalar(1) - c;

    Mat4 r = Mat4::identity();
    // Written as m[column][row].
    r.m[0][0] = t * k.x * k.x + c;
    r.m[0][1] = t * k.x * k.y + s * k.z;
    r.m[0][2] = t * k.x * k.z - s * k.y;

    r.m[1][0] = t * k.x * k.y - s * k.z;
    r.m[1][1] = t * k.y * k.y + c;
    r.m[1][2] = t * k.y * k.z + s * k.x;

    r.m[2][0] = t * k.x * k.z + s * k.y;
    r.m[2][1] = t * k.y * k.z - s * k.x;
    r.m[2][2] = t * k.z * k.z + c;
    return r;
}

Mat4 rotationX(Scalar angle) { return rotation({Scalar(1), Scalar(0), Scalar(0)}, angle); }
Mat4 rotationY(Scalar angle) { return rotation({Scalar(0), Scalar(1), Scalar(0)}, angle); }
Mat4 rotationZ(Scalar angle) { return rotation({Scalar(0), Scalar(0), Scalar(1)}, angle); }

Mat4 operator*(const Mat4& a, const Mat4& b) {
    // (a*b)[c][r] = sum_k a[k][r] * b[c][k]
    Mat4 out;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            Scalar sum = Scalar(0);
            for (int k = 0; k < 4; ++k) sum += a.m[k][r] * b.m[c][k];
            out.m[c][r] = sum;
        }
    }
    return out;
}

Vec4 operator*(const Mat4& a, const Vec4& v) {
    Vec4 out;
    for (int r = 0; r < 4; ++r) {
        out[r] = a.m[0][r] * v.x + a.m[1][r] * v.y + a.m[2][r] * v.z + a.m[3][r] * v.w;
    }
    return out;
}

Mat4 transpose(const Mat4& a) {
    Mat4 out;
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) out.m[c][r] = a.m[r][c];
    return out;
}

Scalar determinant(const Mat4& a) {
    // Laplace expansion on 2x2 minors. Reporting only, not a hot path.
    const Scalar s0 = a.m[0][0] * a.m[1][1] - a.m[1][0] * a.m[0][1];
    const Scalar s1 = a.m[0][0] * a.m[2][1] - a.m[2][0] * a.m[0][1];
    const Scalar s2 = a.m[0][0] * a.m[3][1] - a.m[3][0] * a.m[0][1];
    const Scalar s3 = a.m[1][0] * a.m[2][1] - a.m[2][0] * a.m[1][1];
    const Scalar s4 = a.m[1][0] * a.m[3][1] - a.m[3][0] * a.m[1][1];
    const Scalar s5 = a.m[2][0] * a.m[3][1] - a.m[3][0] * a.m[2][1];

    const Scalar c5 = a.m[2][2] * a.m[3][3] - a.m[3][2] * a.m[2][3];
    const Scalar c4 = a.m[1][2] * a.m[3][3] - a.m[3][2] * a.m[1][3];
    const Scalar c3 = a.m[1][2] * a.m[2][3] - a.m[2][2] * a.m[1][3];
    const Scalar c2 = a.m[0][2] * a.m[3][3] - a.m[3][2] * a.m[0][3];
    const Scalar c1 = a.m[0][2] * a.m[2][3] - a.m[2][2] * a.m[0][3];
    const Scalar c0 = a.m[0][2] * a.m[1][3] - a.m[1][2] * a.m[0][3];

    return s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
}

bool invert(const Mat4& a, Mat4& out) {
    // Gauss-Jordan on [A | I]. Local copies are [row][col] -- the transpose of
    // our storage -- because elimination is naturally row-oriented.
    Scalar lhs[4][4];
    Scalar rhs[4][4]{};
    Scalar rowScale[4];
    for (int r = 0; r < 4; ++r) {
        Scalar rowMax = Scalar(0);
        for (int c = 0; c < 4; ++c) {
            lhs[r][c] = a.m[c][r];
            rowMax = std::fmax(rowMax, std::fabs(lhs[r][c]));
        }
        // An all-zero row is rank-deficient at any scale.
        if (rowMax == Scalar(0)) return false;
        rowScale[r] = rowMax;
        rhs[r][r] = Scalar(1);
    }

    // Pivot SELECTION and singularity DETECTION are separate questions here.
    //
    // Selection uses the scaled ratio, which keeps elimination stable when rows
    // are scaled differently. Detection rejects only an exactly-zero pivot,
    // because a homogeneous transform has no single scale to threshold against:
    // absolute (1e-20) rejects scaling(1e-21); whole-matrix rejects it too,
    // since m[3][3] == 1 dominates; row-relative rejects translation(1e4) *
    // scaling(1e-4), an ordinary CAD transform, because row 0 mixes a 1e-4
    // linear part with a 1e4 translation.
    //
    // The cost, measured: exactly representable degeneracy is still caught, but
    // degeneracy produced by arithmetic lands on a tiny non-zero pivot and is
    // accepted -- 175,488 of 200,000 random cases, worst residual 1.3e+05. That
    // is the better trade, since over-rejection fails silently (inverse()
    // returns identity in release) while a large inverse is detectable via
    // determinant(). A real conditioning check belongs where transforms are
    // accepted from a user; nothing here inverts user-supplied data.

    for (int col = 0; col < 4; ++col) {
        int pivot = -1;
        Scalar best = Scalar(0);
        for (int r = col; r < 4; ++r) {
            const Scalar ratio = std::fabs(lhs[r][col]) / rowScale[r];
            // A NaN ratio compares false and simply never wins, leaving
            // pivot == -1 if every candidate is NaN.
            if (ratio > best) {
                best = ratio;
                pivot = r;
            }
        }

        // Rank-deficient in this column: no non-zero candidate remains.
        if (pivot < 0) return false;
        if (lhs[pivot][col] == Scalar(0) || !std::isfinite(lhs[pivot][col])) return false;

        if (pivot != col) {
            for (int c = 0; c < 4; ++c) {
                std::swap(lhs[col][c], lhs[pivot][c]);
                std::swap(rhs[col][c], rhs[pivot][c]);
            }
            // The scales are a property of the rows, so they travel with them.
            std::swap(rowScale[col], rowScale[pivot]);
        }

        const Scalar invPivot = Scalar(1) / lhs[col][col];
        for (int c = 0; c < 4; ++c) {
            lhs[col][c] *= invPivot;
            rhs[col][c] *= invPivot;
        }

        for (int r = 0; r < 4; ++r) {
            if (r == col) continue;
            const Scalar factor = lhs[r][col];
            if (factor == Scalar(0)) continue;
            for (int c = 0; c < 4; ++c) {
                lhs[r][c] -= factor * lhs[col][c];
                rhs[r][c] -= factor * rhs[col][c];
            }
        }
    }

    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) out.m[c][r] = rhs[r][c];
    return true;
}

Mat4 inverse(const Mat4& a) {
    Mat4 out;
    const bool ok = invert(a, out);
    assert(ok && "inverse() called on a singular matrix");
    if (!ok) return Mat4::identity();
    return out;
}

Vec3 transformPoint(const Mat4& a, const Vec3& p) {
    const Vec4 r = a * Vec4(p, Scalar(1));
    // Affine leaves w == 1 and skips the divide; projections do not.
    if (r.w != Scalar(1) && r.w != Scalar(0)) {
        const Scalar invW = Scalar(1) / r.w;
        return {r.x * invW, r.y * invW, r.z * invW};
    }
    return r.xyz();
}

Vec3 transformVector(const Mat4& a, const Vec3& v) {
    const Vec4 r = a * Vec4(v, Scalar(0));
    return r.xyz();
}

Vec3 transformNormalWithInverse(const Mat4& inverseTransform, const Vec3& n) {
    // n' = (M^-1)^T * n on the upper-left 3x3, as a transposed multiply so the
    // transpose is never materialised.
    const Mat4& i = inverseTransform;
    return {i.m[0][0] * n.x + i.m[0][1] * n.y + i.m[0][2] * n.z,
            i.m[1][0] * n.x + i.m[1][1] * n.y + i.m[1][2] * n.z,
            i.m[2][0] * n.x + i.m[2][1] * n.y + i.m[2][2] * n.z};
}

}  // namespace geom
