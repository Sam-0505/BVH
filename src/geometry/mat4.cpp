#include "geometry/mat4.hpp"

#include <cassert>
#include <cmath>
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
    // Rodrigues' rotation formula in matrix form:
    //   R = I cos(t) + [k]_x sin(t) + k k^T (1 - cos(t))
    // where k is the unit axis and [k]_x its cross-product matrix.
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
    // Laplace expansion using 2x2 minors of the top two rows paired with the
    // bottom two. Only used for reporting/validation, not in a hot path.
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
    // Gauss-Jordan on the augmented system [A | I], with partial pivoting.
    //
    // Indices below are [row][col] on local working copies -- the transpose of
    // our storage order -- because elimination is naturally row-oriented. We
    // transpose in and out rather than contort the algorithm.
    Scalar lhs[4][4];
    Scalar rhs[4][4]{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) lhs[r][c] = a.m[c][r];
        rhs[r][r] = Scalar(1);
    }

    for (int col = 0; col < 4; ++col) {
        // Partial pivoting: choose the row with the largest magnitude in this
        // column. This is what keeps the elimination stable -- dividing by a
        // near-zero pivot amplifies existing rounding error without it.
        int pivot = col;
        Scalar best = std::fabs(lhs[col][col]);
        for (int r = col + 1; r < 4; ++r) {
            const Scalar mag = std::fabs(lhs[r][col]);
            if (mag > best) {
                best = mag;
                pivot = r;
            }
        }

        // Singular to working precision. Reject rather than produce inf/NaN.
        if (best <= Scalar(1e-20)) return false;

        if (pivot != col) {
            for (int c = 0; c < 4; ++c) {
                std::swap(lhs[col][c], lhs[pivot][c]);
                std::swap(rhs[col][c], rhs[pivot][c]);
            }
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
    // Affine transforms leave w == 1 and skip the divide; projection matrices
    // do not, so handle both rather than silently producing wrong results.
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
    // n' = (M^-1)^T * n, restricted to the upper-left 3x3. Expanded here as a
    // transposed multiply so we never materialise the transpose.
    const Mat4& i = inverseTransform;
    return {i.m[0][0] * n.x + i.m[0][1] * n.y + i.m[0][2] * n.z,
            i.m[1][0] * n.x + i.m[1][1] * n.y + i.m[1][2] * n.z,
            i.m[2][0] * n.x + i.m[2][1] * n.y + i.m[2][2] * n.z};
}

}  // namespace geom
