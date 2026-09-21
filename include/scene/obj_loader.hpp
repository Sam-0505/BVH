#pragma once

#include <cstddef>
#include <string>
#include <string_view>

#include "geometry/mesh.hpp"

namespace scene {

// Benchmark provenance: a triangle count is only reproducible alongside how
// many came from triangulation and how many are degenerate.
struct ObjLoadStats {
    std::size_t positions{0};
    std::size_t faces{0};
    std::size_t triangles{0};
    std::size_t polygonsTriangulated{0};  // faces with more than 3 vertices
    std::size_t degenerateTriangles{0};
    std::size_t skippedLines{0};  // recognised-but-ignored directives (vt, vn, usemtl, ...)
};

// Wavefront OBJ -> indexed triangle mesh.
//
// Reads `v` positions and `f` faces in every index form (v, v/vt, v//vn,
// v/vt/vn), positive or negative indices, fan-triangulating polygons. Skips
// vt/vn/o/g/s/usemtl/mtllib -- this project wants geometry, not shading.
//
// Free-form geometry (curv, surf, deg) is REJECTED rather than skipped, since
// silently dropping it would make the triangle count wrong undetectably.
//
// Throws std::runtime_error with a line number on malformed input; parsing is
// the trust boundary, so errors are exceptions rather than asserts.
geom::Mesh loadObj(const std::string& path, ObjLoadStats* stats = nullptr);

// Same parser over a buffer; `sourceName` appears in error messages.
geom::Mesh loadObjFromString(std::string_view text, std::string_view sourceName = "<string>",
                             ObjLoadStats* stats = nullptr);

}  // namespace scene
