#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "scene/obj_loader.hpp"

using namespace geom;
using scene::loadObjFromString;
using scene::ObjLoadStats;

TEST(ObjLoader, ParsesASingleTriangle) {
    const std::string src =
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "f 1 2 3\n";
    ObjLoadStats stats;
    const Mesh m = loadObjFromString(src, "<test>", &stats);

    EXPECT_EQ(m.vertexCount(), 3u);
    EXPECT_EQ(m.triangleCount(), 1u);
    EXPECT_EQ(stats.faces, 1u);
    EXPECT_EQ(stats.triangles, 1u);
    EXPECT_EQ(stats.polygonsTriangulated, 0u);

    const Triangle t = m.triangle(0);
    EXPECT_EQ(t.v0, Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(t.v1, Vec3(1.0f, 0.0f, 0.0f));
    EXPECT_EQ(t.v2, Vec3(0.0f, 1.0f, 0.0f));
}

TEST(ObjLoader, ParsesNegativeAndDecimalCoordinates) {
    const Mesh m = loadObjFromString("v -1.5 2.25 -0.125\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    EXPECT_EQ(m.positions()[0], Vec3(-1.5f, 2.25f, -0.125f));
}

TEST(ObjLoader, FanTriangulatesAQuad) {
    const std::string src =
        "v 0 0 0\n v 1 0 0\n v 1 1 0\n v 0 1 0\n"
        "f 1 2 3 4\n";
    ObjLoadStats stats;
    const Mesh m = loadObjFromString(src, "<test>", &stats);

    EXPECT_EQ(m.triangleCount(), 2u);
    EXPECT_EQ(stats.faces, 1u);
    EXPECT_EQ(stats.polygonsTriangulated, 1u);
    // Fan from vertex 0: (0,1,2) and (0,2,3).
    EXPECT_EQ(m.indices(), (std::vector<std::uint32_t>{0, 1, 2, 0, 2, 3}));
}

TEST(ObjLoader, FanTriangulatesAPentagon) {
    const std::string src =
        "v 0 0 0\n v 1 0 0\n v 2 1 0\n v 1 2 0\n v 0 2 0\n"
        "f 1 2 3 4 5\n";
    const Mesh m = loadObjFromString(src);
    EXPECT_EQ(m.triangleCount(), 3u);  // n-gon becomes n-2 triangles
}

TEST(ObjLoader, AcceptsEveryFaceIndexForm) {
    // v, v/vt, v//vn and v/vt/vn must all resolve to the same geometry.
    const std::string forms[] = {
        "f 1 2 3\n",
        "f 1/1 2/2 3/3\n",
        "f 1//1 2//2 3//3\n",
        "f 1/1/1 2/2/2 3/3/3\n",
    };
    for (const std::string& face : forms) {
        const std::string src =
            "v 0 0 0\nv 1 0 0\nv 0 1 0\n"
            "vt 0 0\nvt 1 0\nvt 0 1\n"
            "vn 0 0 1\nvn 0 0 1\nvn 0 0 1\n" + face;
        const Mesh m = loadObjFromString(src);
        ASSERT_EQ(m.triangleCount(), 1u) << "failed on: " << face;
        EXPECT_EQ(m.indices(), (std::vector<std::uint32_t>{0, 1, 2})) << "failed on: " << face;
    }
}

TEST(ObjLoader, ResolvesNegativeIndicesRelativeToTheCurrentEnd) {
    // -1 is the most recently defined vertex.
    const Mesh m = loadObjFromString("v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n");
    EXPECT_EQ(m.indices(), (std::vector<std::uint32_t>{0, 1, 2}));
}

TEST(ObjLoader, NegativeIndicesAreRelativeToTheVertexCountAtThatLine) {
    // Two triangles, each using -1/-2/-3 but resolving differently because
    // more vertices exist by the second face.
    const std::string src =
        "v 0 0 0\nv 1 0 0\nv 0 1 0\n"
        "f -3 -2 -1\n"
        "v 5 0 0\nv 6 0 0\nv 5 1 0\n"
        "f -3 -2 -1\n";
    const Mesh m = loadObjFromString(src);
    EXPECT_EQ(m.triangleCount(), 2u);
    EXPECT_EQ(m.indices(), (std::vector<std::uint32_t>{0, 1, 2, 3, 4, 5}));
}

TEST(ObjLoader, IgnoresCommentsBlankLinesAndUnusedDirectives) {
    const std::string src =
        "# a comment\n"
        "\n"
        "mtllib scene.mtl\n"
        "o object_name\n"
        "g group_name\n"
        "s off\n"
        "usemtl material\n"
        "v 0 0 0\n v 1 0 0\n v 0 1 0\n"
        "vt 0 0\n"
        "vn 0 0 1\n"
        "   # indented comment\n"
        "f 1 2 3\n";
    ObjLoadStats stats;
    const Mesh m = loadObjFromString(src, "<test>", &stats);
    EXPECT_EQ(m.triangleCount(), 1u);
    EXPECT_GT(stats.skippedLines, 0u);
}

TEST(ObjLoader, HandlesCarriageReturnLineEndings) {
    // A file authored on Windows must not fail to parse its last coordinate.
    const Mesh m = loadObjFromString("v 0 0 0\r\nv 1 0 0\r\nv 0 1 0\r\nf 1 2 3\r\n");
    EXPECT_EQ(m.triangleCount(), 1u);
    EXPECT_EQ(m.triangle(0).v1, Vec3(1.0f, 0.0f, 0.0f));
}

TEST(ObjLoader, HandlesAFinalLineWithoutANewline) {
    const Mesh m = loadObjFromString("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3");
    EXPECT_EQ(m.triangleCount(), 1u);
}

TEST(ObjLoader, ToleratesExtraWhitespace) {
    const Mesh m = loadObjFromString("  v   0   0   0 \nv\t1\t0\t0\nv 0 1 0\n  f  1  2  3  \n");
    EXPECT_EQ(m.triangleCount(), 1u);
}

TEST(ObjLoader, ParsesTheFourthVertexComponentWithoutUsingIt) {
    const Mesh m = loadObjFromString("v 1 2 3 0.5\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    EXPECT_EQ(m.positions()[0], Vec3(1.0f, 2.0f, 3.0f));
}

TEST(ObjLoader, ComputesBoundsAndDegenerateCount) {
    const std::string src =
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 2 0 0\n"
        "f 1 2 3\n"
        "f 1 2 4\n";  // collinear along y=0 -> degenerate
    ObjLoadStats stats;
    const Mesh m = loadObjFromString(src, "<test>", &stats);
    EXPECT_EQ(m.triangleCount(), 2u);
    EXPECT_EQ(stats.degenerateTriangles, 1u);
    EXPECT_EQ(m.bounds().min, Vec3(0.0f));
    EXPECT_EQ(m.bounds().max, Vec3(2.0f, 1.0f, 0.0f));
}

// --- Error handling ----------------------------------------------------------

TEST(ObjLoader, RejectsAnOutOfRangeIndex) {
    EXPECT_THROW(loadObjFromString("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n"), std::runtime_error);
}

TEST(ObjLoader, RejectsAForwardReference) {
    // Index 4 is defined later in the file; OBJ does not permit that, and
    // accepting it would silently bind to the wrong vertex.
    const std::string src = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\nv 9 9 9\n";
    EXPECT_THROW(loadObjFromString(src), std::runtime_error);
}

TEST(ObjLoader, RejectsIndexZero) {
    EXPECT_THROW(loadObjFromString("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n"), std::runtime_error);
}

TEST(ObjLoader, RejectsANegativeIndexReachingBeforeTheStart) {
    EXPECT_THROW(loadObjFromString("v 0 0 0\nv 1 0 0\nf -5 -1 -2\n"), std::runtime_error);
}

TEST(ObjLoader, RejectsAFaceWithFewerThanThreeVertices) {
    EXPECT_THROW(loadObjFromString("v 0 0 0\nv 1 0 0\nf 1 2\n"), std::runtime_error);
}

TEST(ObjLoader, RejectsAVertexWithTooFewCoordinates) {
    EXPECT_THROW(loadObjFromString("v 0 0\n"), std::runtime_error);
}

TEST(ObjLoader, RejectsFreeFormGeometryRatherThanSilentlyDroppingIt) {
    const std::string src = "v 0 0 0\nv 1 0 0\ncurv 0.0 1.0 1 2\n";
    EXPECT_THROW(loadObjFromString(src), std::runtime_error);
}

TEST(ObjLoader, ErrorMessageNamesTheSourceAndLine) {
    try {
        loadObjFromString("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n", "bunny.obj");
        FAIL() << "expected a throw";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("bunny.obj"), std::string::npos) << msg;
        EXPECT_NE(msg.find(":4:"), std::string::npos) << "should name line 4: " << msg;
    }
}

TEST(ObjLoader, AnEmptyFileProducesAnEmptyMesh) {
    const Mesh m = loadObjFromString("");
    EXPECT_TRUE(m.empty());
    EXPECT_EQ(m.triangleCount(), 0u);
}

TEST(ObjLoader, VerticesWithNoFacesProduceOrphansNotTriangles) {
    ObjLoadStats stats;
    const Mesh m = loadObjFromString("v 0 0 0\nv 1 0 0\nv 0 1 0\n", "<test>", &stats);
    EXPECT_EQ(m.vertexCount(), 3u);
    EXPECT_EQ(m.triangleCount(), 0u);
    // bounds() covers referenced geometry, so it is empty; vertexBounds() is not.
    EXPECT_TRUE(m.bounds().isEmpty());
    EXPECT_FALSE(m.vertexBounds().isEmpty());
}

// --- The file-reading path ---------------------------------------------------
//
// Everything above exercises the parser through loadObjFromString. These cover
// loadObj itself: opening, slurping and reporting failure on a real file.

namespace {

// Writes a file next to the test binary and removes it on scope exit, so a
// failing assertion cannot leave the build directory dirty.
class TemporaryObjFile {
public:
    TemporaryObjFile(const char* name, const std::string& contents) : path_(name) {
        std::ofstream out(path_, std::ios::binary);
        out << contents;
    }
    ~TemporaryObjFile() { std::remove(path_.c_str()); }

    TemporaryObjFile(const TemporaryObjFile&) = delete;
    TemporaryObjFile& operator=(const TemporaryObjFile&) = delete;

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

}  // namespace

TEST(ObjLoaderFile, LoadsFromDisk) {
    const TemporaryObjFile file("obj_loader_test_basic.obj",
                                "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nf 1 2 3\nf 2 4 3\n");
    scene::ObjLoadStats stats;
    const Mesh m = scene::loadObj(file.path(), &stats);

    EXPECT_EQ(m.vertexCount(), 4u);
    EXPECT_EQ(m.triangleCount(), 2u);
    EXPECT_EQ(stats.faces, 2u);
    EXPECT_EQ(m.bounds().max, Vec3(1.0f, 1.0f, 0.0f));
}

TEST(ObjLoaderFile, ReportsAMissingFileByName) {
    try {
        scene::loadObj("definitely_not_a_real_file_12345.obj");
        FAIL() << "expected a throw";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("definitely_not_a_real_file_12345.obj"), std::string::npos) << msg;
    }
}

TEST(ObjLoaderFile, PropagatesParseErrorsWithTheRealFilename) {
    const TemporaryObjFile file("obj_loader_test_bad.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 7\n");
    try {
        scene::loadObj(file.path());
        FAIL() << "expected a throw";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("obj_loader_test_bad.obj"), std::string::npos) << msg;
        EXPECT_NE(msg.find(":4:"), std::string::npos) << msg;
    }
}

TEST(ObjLoaderFile, RoundTripsAMeshThroughTheFileFormat) {
    // A generated grid, written out and read back, must reproduce exactly the
    // same triangles. This is the end-to-end check that indices, 1-based
    // numbering and fan triangulation all line up.
    constexpr int kN = 8;
    std::string text;
    for (int y = 0; y <= kN; ++y) {
        for (int x = 0; x <= kN; ++x) {
            text += "v " + std::to_string(x) + " " + std::to_string(y) + " 0\n";
        }
    }
    std::size_t expectedTriangles = 0;
    for (int y = 0; y < kN; ++y) {
        for (int x = 0; x < kN; ++x) {
            // 1-based indices of the quad's corners, emitted as a quad so the
            // loader has to triangulate it.
            const int a = y * (kN + 1) + x + 1;
            const int b = a + 1;
            const int c = a + (kN + 1) + 1;
            const int d = a + (kN + 1);
            text += "f " + std::to_string(a) + " " + std::to_string(b) + " " + std::to_string(c) +
                    " " + std::to_string(d) + "\n";
            expectedTriangles += 2;
        }
    }

    const TemporaryObjFile file("obj_loader_test_grid.obj", text);
    scene::ObjLoadStats stats;
    const Mesh m = scene::loadObj(file.path(), &stats);

    EXPECT_EQ(m.vertexCount(), static_cast<std::size_t>((kN + 1) * (kN + 1)));
    EXPECT_EQ(m.triangleCount(), expectedTriangles);
    EXPECT_EQ(stats.polygonsTriangulated, static_cast<std::size_t>(kN * kN));
    EXPECT_EQ(stats.degenerateTriangles, 0u);
    EXPECT_EQ(m.bounds().min, Vec3(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(m.bounds().max, Vec3(static_cast<Scalar>(kN), static_cast<Scalar>(kN), 0.0f));

    // Every triangle must be non-degenerate and inside the grid's bounds.
    for (std::size_t i = 0; i < m.triangleCount(); ++i) {
        ASSERT_FALSE(m.triangle(i).isDegenerate()) << "triangle " << i;
        ASSERT_TRUE(m.bounds().contains(m.triangle(i).bounds())) << "triangle " << i;
    }
}

// strtof happily parses "nan" and "inf", and a mesh carrying one would bound
// geometry it does not contain. The Mesh constructor is the boundary that
// rejects it, so the loader must surface that rather than swallow it.
TEST(ObjLoader, RejectsNonFiniteCoordinates) {
    EXPECT_THROW(loadObjFromString("v nan 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"),
                 std::invalid_argument);
    EXPECT_THROW(loadObjFromString("v 0 inf 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"),
                 std::invalid_argument);
    EXPECT_THROW(loadObjFromString("v 0 0 -inf\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"),
                 std::invalid_argument);
    // Overflowing the float range reaches the same place.
    EXPECT_THROW(loadObjFromString("v 1e40 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"),
                 std::invalid_argument);
}
