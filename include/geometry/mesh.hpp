#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/aabb.hpp"
#include "geometry/mat4.hpp"
#include "geometry/scalar.hpp"
#include "geometry/triangle.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// An indexed triangle mesh: a flat array of vertex positions plus an index
// buffer holding three vertex indices per triangle.
//
// Why indexed and not an array of Triangle:
//   - A closed mesh shares each vertex among ~6 triangles, so indexed storage is
//     roughly 12 bytes + 12 bytes of indices per triangle versus 36 bytes
//     unpacked. On a million-triangle mesh that difference decides whether the
//     vertex data fits in cache during a build.
//   - It is the layout Vulkan wants for vkCmdDrawIndexed, so the render path can
//     upload these buffers directly with no repacking.
// The cost is one level of indirection per vertex fetch, which matters during
// traversal; Phase 2 can revisit by storing unpacked triangles in leaf order.
//
// INVARIANT: indices.size() is a multiple of 3, and every index is < positions.size().
// Enforced at construction; a mesh that violates it is rejected rather than
// carried forward to crash during traversal.
class Mesh {
public:
    Mesh() = default;

    // Takes ownership of both buffers. Pass with std::move to avoid copying
    // what may be tens of megabytes.
    //
    // Throws std::invalid_argument when the index buffer is malformed. Errors
    // are reported explicitly rather than by assertion because this is the
    // boundary where untrusted file data enters the system.
    Mesh(std::vector<Vec3> positions, std::vector<std::uint32_t> indices);

    std::size_t vertexCount() const { return positions_.size(); }
    std::size_t triangleCount() const { return indices_.size() / 3; }
    bool empty() const { return indices_.empty(); }

    const std::vector<Vec3>& positions() const { return positions_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

    // The three vertex indices of triangle i.
    void triangleIndices(std::size_t i, std::uint32_t& a, std::uint32_t& b,
                         std::uint32_t& c) const {
        const std::size_t base = i * 3;
        a = indices_[base + 0];
        b = indices_[base + 1];
        c = indices_[base + 2];
    }

    // Materialise triangle i. Returned by value: 36 bytes is cheaper to copy
    // than to alias, and returning a value keeps the Mesh immutable to callers.
    Triangle triangle(std::size_t i) const {
        const std::size_t base = i * 3;
        return Triangle(positions_[indices_[base + 0]],
                        positions_[indices_[base + 1]],
                        positions_[indices_[base + 2]]);
    }

    // Bounds over all VERTICES, cached at construction.
    //
    // Note this bounds the vertex array, not the referenced triangles. For a
    // mesh with unreferenced ("orphan") vertices the result is conservative --
    // still correct as a bound, just looser. computeTriangleBounds() below gives
    // the tight answer at O(n) cost when that matters.
    const AABB& bounds() const { return bounds_; }

    // Tight bounds over only the vertices actually referenced by the index
    // buffer. O(triangleCount).
    AABB computeTriangleBounds() const;

    // Number of triangles with zero area, which cannot be hit by a ray. Reported
    // rather than silently dropped, so mesh quality is visible in benchmarks
    // instead of quietly changing the primitive count.
    std::size_t countDegenerateTriangles(Scalar tol = kEpsilon) const;

    // Apply an affine transform to every vertex in place and refresh the cached
    // bounds. Transforming the mesh rather than the query is the right choice
    // for static scene geometry: it is paid once, and it keeps BVH traversal
    // free of a per-ray matrix multiply.
    void transform(const Mat4& xf);

private:
    void recomputeBounds();

    std::vector<Vec3> positions_;
    std::vector<std::uint32_t> indices_;
    AABB bounds_;
};

}  // namespace geom
