#include <gtest/gtest.h>

#include <cmath>
#include <utility>

#include "geometry/mat4.hpp"

using namespace geom;

namespace {

// A transform with rotation, non-uniform scale and translation, so that tests
// exercise the general case rather than a conveniently symmetric one.
Mat4 makeCompositeTransform() {
    return translation(Vec3(5.0f, -3.0f, 2.0f)) *
           rotation(Vec3(1.0f, 2.0f, 3.0f), radians(37.0f)) *
           scaling(Vec3(2.0f, 3.0f, 0.5f));
}

}  // namespace

TEST(Mat4, IdentityIsMultiplicativeIdentity) {
    const Mat4 id = Mat4::identity();
    const Mat4 m = makeCompositeTransform();
    EXPECT_TRUE(nearlyEqual(id * m, m));
    EXPECT_TRUE(nearlyEqual(m * id, m));
}

TEST(Mat4, ColumnMajorStorageLayout) {
    // Pins down the storage convention the Vulkan upload path depends on:
    // translation lives in the fourth COLUMN, i.e. m[3][0..2].
    const Mat4 t = translation(Vec3(7.0f, 8.0f, 9.0f));
    EXPECT_FLOAT_EQ(t.m[3][0], 7.0f);
    EXPECT_FLOAT_EQ(t.m[3][1], 8.0f);
    EXPECT_FLOAT_EQ(t.m[3][2], 9.0f);
    EXPECT_FLOAT_EQ(t.m[3][3], 1.0f);

    // ...and not in the fourth row.
    EXPECT_FLOAT_EQ(t.m[0][3], 0.0f);
    EXPECT_FLOAT_EQ(t.m[1][3], 0.0f);
    EXPECT_FLOAT_EQ(t.m[2][3], 0.0f);
}

TEST(Mat4, ColumnAndRowAccessors) {
    const Mat4 t = translation(Vec3(7.0f, 8.0f, 9.0f));
    EXPECT_EQ(t.column(3), Vec4(7.0f, 8.0f, 9.0f, 1.0f));
    EXPECT_EQ(t.row(3), Vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

TEST(Mat4, TranslationMovesPointsButNotDirections) {
    const Mat4 t = translation(Vec3(1.0f, 2.0f, 3.0f));
    const Vec3 p(10.0f, 20.0f, 30.0f);

    // A point picks up the translation...
    EXPECT_TRUE(nearlyEqual(transformPoint(t, p), Vec3(11.0f, 22.0f, 33.0f)));
    // ...a direction does not. This is the w=1 vs w=0 distinction.
    EXPECT_TRUE(nearlyEqual(transformVector(t, p), p));
}

TEST(Mat4, Scaling) {
    const Mat4 s = scaling(Vec3(2.0f, 3.0f, 4.0f));
    EXPECT_TRUE(nearlyEqual(transformPoint(s, Vec3(1.0f, 1.0f, 1.0f)), Vec3(2.0f, 3.0f, 4.0f)));
    // Unlike translation, scaling DOES affect directions.
    EXPECT_TRUE(nearlyEqual(transformVector(s, Vec3(1.0f, 1.0f, 1.0f)), Vec3(2.0f, 3.0f, 4.0f)));
}

TEST(Mat4, AxisRotationsAreRightHanded) {
    const Scalar quarter = radians(90.0f);

    // Rotating +X by 90 degrees about Z gives +Y (right-handed).
    EXPECT_TRUE(nearlyEqual(transformVector(rotationZ(quarter), Vec3(1.0f, 0.0f, 0.0f)),
                            Vec3(0.0f, 1.0f, 0.0f), 1e-5f));
    // +Y about X gives +Z.
    EXPECT_TRUE(nearlyEqual(transformVector(rotationX(quarter), Vec3(0.0f, 1.0f, 0.0f)),
                            Vec3(0.0f, 0.0f, 1.0f), 1e-5f));
    // +Z about Y gives +X.
    EXPECT_TRUE(nearlyEqual(transformVector(rotationY(quarter), Vec3(0.0f, 0.0f, 1.0f)),
                            Vec3(1.0f, 0.0f, 0.0f), 1e-5f));
}

TEST(Mat4, RotationPreservesLength) {
    const Mat4 r = rotation(Vec3(1.0f, 2.0f, 3.0f), radians(57.0f));
    const Vec3 v(4.0f, -5.0f, 6.0f);
    EXPECT_NEAR(length(transformVector(r, v)), length(v), 1e-4f);
}

TEST(Mat4, RotationAboutDegenerateAxisIsIdentity) {
    // Must not produce NaN from normalising a zero-length axis.
    const Mat4 r = rotation(Vec3(0.0f), radians(45.0f));
    EXPECT_TRUE(nearlyEqual(r, Mat4::identity()));
}

TEST(Mat4, RotationAboutArbitraryAxisLeavesAxisFixed) {
    const Vec3 axis = normalize(Vec3(1.0f, 2.0f, 3.0f));
    const Mat4 r = rotation(axis, radians(73.0f));
    // A vector along the rotation axis is unchanged by the rotation.
    EXPECT_TRUE(nearlyEqual(transformVector(r, axis), axis, 1e-5f));
}

TEST(Mat4, MultiplicationAppliesRightmostFirst) {
    // Column-vector convention: (A * B) * v means "apply B, then A".
    const Mat4 t = translation(Vec3(10.0f, 0.0f, 0.0f));
    const Mat4 s = scaling(Vec3(2.0f, 2.0f, 2.0f));
    const Vec3 p(1.0f, 0.0f, 0.0f);

    // Scale first (1 -> 2), then translate (2 -> 12).
    EXPECT_TRUE(nearlyEqual(transformPoint(t * s, p), Vec3(12.0f, 0.0f, 0.0f)));
    // Translate first (1 -> 11), then scale (11 -> 22).
    EXPECT_TRUE(nearlyEqual(transformPoint(s * t, p), Vec3(22.0f, 0.0f, 0.0f)));
}

TEST(Mat4, Transpose) {
    const Mat4 m = makeCompositeTransform();
    const Mat4 t = transpose(m);
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) EXPECT_FLOAT_EQ(t.m[c][r], m.m[r][c]);

    // Transpose is an involution.
    EXPECT_TRUE(nearlyEqual(transpose(t), m));
}

TEST(Mat4, DeterminantOfIdentityAndScaling) {
    EXPECT_NEAR(determinant(Mat4::identity()), 1.0f, 1e-5f);
    // det of a pure scale is the product of the scale factors.
    EXPECT_NEAR(determinant(scaling(Vec3(2.0f, 3.0f, 4.0f))), 24.0f, 1e-4f);
    // A rotation preserves volume.
    EXPECT_NEAR(determinant(rotation(Vec3(1.0f, 1.0f, 1.0f), radians(30.0f))), 1.0f, 1e-4f);
}

TEST(Mat4, InverseTimesOriginalIsIdentity) {
    const Mat4 m = makeCompositeTransform();
    Mat4 inv;
    ASSERT_TRUE(invert(m, inv));

    // The strongest available check on the inverse: both orderings must give I.
    EXPECT_TRUE(nearlyEqual(m * inv, Mat4::identity(), 1e-4f));
    EXPECT_TRUE(nearlyEqual(inv * m, Mat4::identity(), 1e-4f));
}

TEST(Mat4, InverseRoundTripsPoints) {
    const Mat4 m = makeCompositeTransform();
    const Mat4 inv = inverse(m);
    const Vec3 p(3.0f, -7.0f, 11.0f);
    EXPECT_TRUE(nearlyEqual(transformPoint(inv, transformPoint(m, p)), p, 1e-3f));
}

TEST(Mat4, InvertRejectsSingularMatrix) {
    // A zero scale on one axis collapses the space; there is no inverse.
    const Mat4 singular = scaling(Vec3(1.0f, 1.0f, 0.0f));
    Mat4 out = Mat4::identity();
    EXPECT_FALSE(invert(singular, out));
    // On failure `out` must be left untouched rather than filled with NaN.
    EXPECT_TRUE(nearlyEqual(out, Mat4::identity()));

    const Mat4 zero = Mat4::zero();
    EXPECT_FALSE(invert(zero, out));
}

TEST(Mat4, NormalTransformUnderNonUniformScale) {
    // The classic bug this guards against: under non-uniform scale a normal does
    // NOT transform like a direction. Take a plane spanned by X and Z with
    // normal +Y, then scale Y by 2. The surface flattens relative to Y, so the
    // correct transformed normal still points along +Y -- but transformVector
    // would scale it by 2 in Y, which happens to keep the direction here, so use
    // a tilted normal where the two answers genuinely differ.
    const Mat4 s = scaling(Vec3(2.0f, 1.0f, 1.0f));
    const Mat4 sInv = inverse(s);

    // Surface tangent in the XY plane, and its perpendicular normal.
    const Vec3 tangent(1.0f, 1.0f, 0.0f);
    const Vec3 normalBefore(-1.0f, 1.0f, 0.0f);
    ASSERT_NEAR(dot(tangent, normalBefore), 0.0f, 1e-6f);

    const Vec3 tangentAfter = transformVector(s, tangent);
    const Vec3 normalCorrect = transformNormalWithInverse(sInv, normalBefore);
    const Vec3 normalWrong = transformVector(s, normalBefore);

    // The inverse-transpose keeps the normal perpendicular to the surface.
    EXPECT_NEAR(dot(tangentAfter, normalCorrect), 0.0f, 1e-5f);
    // Transforming it as a direction does not.
    EXPECT_GT(std::fabs(dot(normalize(tangentAfter), normalize(normalWrong))), 0.1f);
}

TEST(Mat4, TransformPointAppliesPerspectiveDivide) {
    // A matrix with a non-trivial bottom row must trigger the divide by w.
    // Bottom row becomes (0, 0, 1, 0), so w_out = z_in. Clearing m[3][3] is
    // essential: identity leaves it at 1, which would give w_out = z_in + 1.
    Mat4 m = Mat4::identity();
    m.m[2][3] = 1.0f;  // column 2, row 3
    m.m[3][3] = 0.0f;  // column 3, row 3

    const Vec3 p(4.0f, 6.0f, 2.0f);
    // w becomes 2, so the result is p/2.
    EXPECT_TRUE(nearlyEqual(transformPoint(m, p), Vec3(2.0f, 3.0f, 1.0f), 1e-5f));

    // And an affine matrix (w_out == 1) must skip the divide entirely.
    const Mat4 affine = translation(Vec3(1.0f, 1.0f, 1.0f));
    EXPECT_TRUE(nearlyEqual(transformPoint(affine, p), Vec3(5.0f, 7.0f, 3.0f), 1e-5f));
}

TEST(Mat4, MatrixVectorMultiplyMatchesManualExpansion) {
    const Mat4 m = makeCompositeTransform();
    const Vec4 v(1.0f, 2.0f, 3.0f, 1.0f);
    const Vec4 got = m * v;
    for (int r = 0; r < 4; ++r) {
        const Scalar expected =
            m.m[0][r] * v.x + m.m[1][r] * v.y + m.m[2][r] * v.z + m.m[3][r] * v.w;
        EXPECT_NEAR(got[r], expected, 1e-5f);
    }
}

TEST(Mat4, FromColumnsMatchesStorageOrder) {
    // fromColumns had no coverage at all. It is the constructor that most
    // directly encodes the column-major convention, so getting it wrong would
    // silently transpose every matrix built through it.
    const Vec4 c0(1.0f, 2.0f, 3.0f, 4.0f);
    const Vec4 c1(5.0f, 6.0f, 7.0f, 8.0f);
    const Vec4 c2(9.0f, 10.0f, 11.0f, 12.0f);
    const Vec4 c3(13.0f, 14.0f, 15.0f, 16.0f);
    const Mat4 m = Mat4::fromColumns(c0, c1, c2, c3);

    EXPECT_EQ(m.column(0), c0);
    EXPECT_EQ(m.column(1), c1);
    EXPECT_EQ(m.column(2), c2);
    EXPECT_EQ(m.column(3), c3);

    // Rows are the transpose of what was supplied.
    EXPECT_EQ(m.row(0), Vec4(1.0f, 5.0f, 9.0f, 13.0f));

    // Raw storage is m[column][row].
    EXPECT_FLOAT_EQ(m.m[0][0], 1.0f);
    EXPECT_FLOAT_EQ(m.m[0][1], 2.0f);
    EXPECT_FLOAT_EQ(m.m[1][0], 5.0f);
}

TEST(Mat4, FromColumnsCanRebuildIdentity) {
    const Mat4 id = Mat4::fromColumns(Vec4(1.0f, 0.0f, 0.0f, 0.0f), Vec4(0.0f, 1.0f, 0.0f, 0.0f),
                                      Vec4(0.0f, 0.0f, 1.0f, 0.0f), Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    EXPECT_TRUE(nearlyEqual(id, Mat4::identity()));
}

TEST(Mat4, InvertHandlesSmallObjectsFarFromTheOrigin) {
    // A millimetre-scale part 10 km from the origin: linear part 1e-4,
    // translation 1e4. An ordinary CAD/scene transform, not a pathological one.
    //
    // This is the case that a row-relative pivot tolerance rejects: row 0 is
    // (1e-4, 0, 0, 1e4), so the pivot ratio is 1e-8, below any epsilon-based
    // cutoff -- even though the matrix inverts exactly. The failure is silent,
    // because inverse() returns identity in a release build.
    const Mat4 m = translation(Vec3(1e4f, 1e4f, 1e4f)) * scaling(Vec3(1e-4f, 1e-4f, 1e-4f));
    Mat4 inv;
    ASSERT_TRUE(invert(m, inv));
    EXPECT_TRUE(nearlyEqual(m * inv, Mat4::identity(), 1e-4f));
}

TEST(Mat4, InvertHandlesWideRangesOfScaleAndTranslation) {
    // The same hazard swept across several decades. A row-scaled tolerance
    // rejected the last three outright.
    //
    // Correctness is checked against the ANALYTIC inverse rather than by
    // round-tripping a point. For M = T(t) * S(s) the inverse is exactly
    // S(1/s) * T(-t), so this tests the elimination directly, without
    // conflating it with the precision limits of the forward transform (see
    // the next test).
    const std::pair<Scalar, Scalar> cases[] = {
        {1e-2f, 1e2f}, {1e-3f, 1e3f}, {1e-4f, 1e4f}, {1e-6f, 1e6f}, {1e-3f, 1e6f},
    };
    for (const auto& [scale, trans] : cases) {
        const Mat4 m = translation(Vec3(trans, trans, trans)) * scaling(Vec3(scale, scale, scale));
        Mat4 inv;
        ASSERT_TRUE(invert(m, inv)) << "rejected scale " << scale << " at translation " << trans;

        const Mat4 analytic = scaling(Vec3(1.0f / scale, 1.0f / scale, 1.0f / scale)) *
                              translation(Vec3(-trans, -trans, -trans));
        // Relative comparison: the translation entries of the inverse reach
        // 1e12 for the widest case, where an absolute tolerance is meaningless.
        EXPECT_TRUE(nearlyEqual(inv, analytic, 1e-4f))
            << "inverse is wrong at scale " << scale << " translation " << trans;
    }
}

TEST(Mat4, RoundTripPrecisionIsLimitedByScaleToTranslationRatio) {
    // Documents a real limit of float32, not a defect in invert().
    //
    // Transforming a unit-scale point by S(s) then T(t) makes its contribution
    // s while the coordinate sits at t. Once s/t falls below the float epsilon
    // (~1.2e-7) that contribution is below one ULP at t and is destroyed by
    // the FORWARD transform. No inverse can recover it -- the information is
    // already gone.
    //
    // This is why the test above compares against the analytic inverse, and
    // why Phase 2 should keep scene geometry near the origin rather than
    // relying on large world offsets.
    const Vec3 p(0.3f, -0.7f, 0.5f);

    // Ratio 1e-4: comfortably inside float precision, round-trip is accurate.
    {
        const Mat4 m = translation(Vec3(1e2f, 1e2f, 1e2f)) * scaling(Vec3(1e-2f, 1e-2f, 1e-2f));
        const Mat4 inv = inverse(m);
        EXPECT_TRUE(nearlyEqual(transformPoint(inv, transformPoint(m, p)), p, 1e-2f));
    }

    // Ratio 1e-8: past the float epsilon, so the round-trip is lossy even
    // though the inverse itself is correct. Asserted so the limit is pinned
    // rather than discovered later in a debugging session.
    {
        const Mat4 m = translation(Vec3(1e4f, 1e4f, 1e4f)) * scaling(Vec3(1e-4f, 1e-4f, 1e-4f));
        Mat4 inv;
        ASSERT_TRUE(invert(m, inv));
        const Vec3 roundTripped = transformPoint(inv, transformPoint(m, p));
        EXPECT_FALSE(nearlyEqual(roundTripped, p, 1e-2f))
            << "float32 unexpectedly preserved a 1e-8 scale ratio; if this starts "
               "passing, Scalar may have been widened to double";
    }
}

TEST(Mat4, InvertStillRejectsGenuinelySingularMatricesAtEveryScale) {
    // Removing the magnitude tolerance must not weaken singularity detection.
    // Elimination drives each of these to an exact-zero pivot.
    Mat4 out;
    EXPECT_FALSE(invert(Mat4::zero(), out));
    EXPECT_FALSE(invert(scaling(Vec3(1.0f, 1.0f, 0.0f)), out));
    EXPECT_FALSE(invert(scaling(Vec3(1e-21f, 1e-21f, 0.0f)), out));
    EXPECT_FALSE(invert(scaling(Vec3(1e21f, 1e21f, 0.0f)), out));

    // Rank deficiency that is not a zero scale: two identical rows.
    Mat4 duplicateRows = Mat4::identity();
    for (int c = 0; c < 4; ++c) duplicateRows.m[c][1] = duplicateRows.m[c][0];
    EXPECT_FALSE(invert(duplicateRows, out));

    // ...and a row that is a linear combination of two others.
    Mat4 dependent = Mat4::identity();
    for (int c = 0; c < 4; ++c) dependent.m[c][2] = dependent.m[c][0] + dependent.m[c][1];
    EXPECT_FALSE(invert(dependent, out));
}

TEST(Mat4, InvertAcceptsUniformlyTinyButInvertibleMatrices) {
    // The singularity threshold is relative, not absolute. scaling(1e-21) has
    // every entry far below any fixed cutoff yet is perfectly invertible; an
    // absolute threshold would wrongly reject it.
    const Mat4 tiny = scaling(Vec3(1e-21f, 1e-21f, 1e-21f));
    Mat4 inv;
    EXPECT_TRUE(invert(tiny, inv));
    EXPECT_TRUE(nearlyEqual(tiny * inv, Mat4::identity(), 1e-4f));

    // ...while a genuinely rank-deficient matrix is still rejected at any scale.
    Mat4 out = Mat4::identity();
    EXPECT_FALSE(invert(scaling(Vec3(1e-21f, 1e-21f, 0.0f)), out));
    EXPECT_FALSE(invert(scaling(Vec3(1e21f, 1e21f, 0.0f)), out));
}

TEST(Mat4, NormalTransformDoesNotPreserveLength) {
    // Documented behaviour worth pinning: the inverse transpose preserves
    // perpendicularity, not magnitude. Callers that need a unit normal must
    // normalize the result themselves.
    const Mat4 s = scaling(Vec3(4.0f, 4.0f, 4.0f));
    const Vec3 n(0.0f, 1.0f, 0.0f);
    const Vec3 transformed = transformNormalWithInverse(inverse(s), n);
    EXPECT_NEAR(length(transformed), 0.25f, 1e-5f);
    // Direction is unchanged under a uniform scale.
    EXPECT_TRUE(nearlyEqual(normalize(transformed), n, 1e-5f));
}
