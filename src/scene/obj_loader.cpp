#include "scene/obj_loader.hpp"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace scene {
namespace {

using geom::Scalar;
using geom::Vec3;

[[noreturn]] void fail(std::string_view source, std::size_t line, const std::string& what) {
    std::ostringstream oss;
    oss << source << ":" << line << ": " << what;
    throw std::runtime_error(oss.str());
}

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r'; }

void skipSpaces(const char*& p, const char* end) {
    while (p < end && isSpace(*p)) ++p;
}

// strtof, not from_chars -- the floating-point overload is missing from this
// toolchain's library (see CLAUDE.md).
bool parseScalar(const char*& p, const char* end, Scalar& out) {
    skipSpaces(p, end);
    if (p >= end) return false;
    char* parseEnd = nullptr;
    errno = 0;
    const float value = std::strtof(p, &parseEnd);
    if (parseEnd == p) return false;  // no digits consumed
    p = parseEnd;
    out = static_cast<Scalar>(value);
    return true;
}

// Parse the leading integer of a face vertex reference ("12", "12/4", "12//7").
bool parseFaceIndex(const char*& p, const char* end, long& out) {
    skipSpaces(p, end);
    if (p >= end) return false;
    char* parseEnd = nullptr;
    const long value = std::strtol(p, &parseEnd, 10);
    if (parseEnd == p) return false;
    p = parseEnd;
    // Skip the texture/normal components; this loader does not use them.
    while (p < end && (*p == '/' || (*p >= '0' && *p <= '9') || *p == '-')) ++p;
    out = value;
    return true;
}

// OBJ indices are 1-based; negative values count back from the current end.
std::size_t resolveIndex(long raw, std::size_t currentCount, std::string_view source,
                         std::size_t line) {
    if (raw > 0) {
        const std::size_t idx = static_cast<std::size_t>(raw) - 1;
        if (idx >= currentCount) {
            fail(source, line,
                 "vertex index " + std::to_string(raw) + " exceeds the " +
                     std::to_string(currentCount) + " vertices defined so far");
        }
        return idx;
    }
    if (raw < 0) {
        const std::size_t back = static_cast<std::size_t>(-raw);
        if (back > currentCount) {
            fail(source, line,
                 "relative vertex index " + std::to_string(raw) + " reaches before the start of " +
                     std::to_string(currentCount) + " vertices");
        }
        return currentCount - back;
    }
    fail(source, line, "vertex index 0 is invalid; OBJ indices are 1-based");
}

bool directiveIs(const char* p, const char* end, const char* name) {
    const std::size_t n = std::char_traits<char>::length(name);
    if (static_cast<std::size_t>(end - p) < n) return false;
    for (std::size_t i = 0; i < n; ++i) {
        if (p[i] != name[i]) return false;
    }
    const char* after = p + n;
    return after == end || isSpace(*after);
}

}  // namespace

geom::Mesh loadObjFromString(std::string_view text, std::string_view sourceName,
                             ObjLoadStats* stats) {
    std::vector<Vec3> positions;
    std::vector<std::uint32_t> indices;
    ObjLoadStats local;

    // Reused across faces so an n-gon does not allocate per line.
    std::vector<std::size_t> faceVertices;

    std::size_t lineNumber = 0;
    std::size_t pos = 0;
    while (pos <= text.size()) {
        const std::size_t newline = text.find('\n', pos);
        const std::size_t lineEnd = (newline == std::string_view::npos) ? text.size() : newline;
        const char* p = text.data() + pos;
        const char* end = text.data() + lineEnd;
        ++lineNumber;

        skipSpaces(p, end);

        if (p < end && *p != '#') {
            if (directiveIs(p, end, "v")) {
                p += 1;
                Scalar x{}, y{}, z{};
                if (!parseScalar(p, end, x) || !parseScalar(p, end, y) ||
                    !parseScalar(p, end, z)) {
                    fail(sourceName, lineNumber, "vertex needs three coordinates");
                }
                positions.push_back(Vec3(x, y, z));
            } else if (directiveIs(p, end, "f")) {
                p += 1;
                faceVertices.clear();
                long raw = 0;
                while (parseFaceIndex(p, end, raw)) {
                    faceVertices.push_back(resolveIndex(raw, positions.size(), sourceName, lineNumber));
                }
                if (faceVertices.size() < 3) {
                    fail(sourceName, lineNumber,
                         "face has " + std::to_string(faceVertices.size()) +
                             " vertices; at least 3 are required");
                }
                ++local.faces;
                if (faceVertices.size() > 3) ++local.polygonsTriangulated;

                // Fan from vertex 0. Correct for convex faces, which is what
                // exporters emit; a concave n-gon would need ear clipping.
                for (std::size_t i = 1; i + 1 < faceVertices.size(); ++i) {
                    indices.push_back(static_cast<std::uint32_t>(faceVertices[0]));
                    indices.push_back(static_cast<std::uint32_t>(faceVertices[i]));
                    indices.push_back(static_cast<std::uint32_t>(faceVertices[i + 1]));
                    ++local.triangles;
                }
            } else if (directiveIs(p, end, "curv") || directiveIs(p, end, "curv2") ||
                       directiveIs(p, end, "surf") || directiveIs(p, end, "deg")) {
                fail(sourceName, lineNumber,
                     "free-form geometry is not supported; only polygonal faces are read, and "
                     "skipping this would silently drop geometry");
            } else {
                // vt, vn, vp, o, g, s, usemtl, mtllib, and anything else.
                ++local.skippedLines;
            }
        }

        if (newline == std::string_view::npos) break;
        pos = newline + 1;
    }

    local.positions = positions.size();

    // Indices were already checked against the vertex count at the line that
    // used them, which also rejects forward references.
    geom::Mesh mesh(std::move(positions), std::move(indices));
    local.degenerateTriangles = mesh.countDegenerateTriangles();

    if (stats != nullptr) *stats = local;
    return mesh;
}

geom::Mesh loadObj(const std::string& path, ObjLoadStats* stats) {
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("cannot open OBJ file: " + path);

    std::ostringstream buffer;
    buffer << file.rdbuf();
    if (file.bad()) throw std::runtime_error("error reading OBJ file: " + path);

    return loadObjFromString(buffer.str(), path, stats);
}

}  // namespace scene
