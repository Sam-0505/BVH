#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry/aabb.hpp"
#include "geometry/mat4.hpp"
#include "geometry/scalar.hpp"
#include "geometry/triangle.hpp"
#include "geometry/vec3.hpp"

namespace geom {

// Indexed triangle mesh: positions plus three indices per triangle.
//
// Indexed rather than an array of Triangle because a closed mesh shares each
// vertex ~6 ways, and because it is the layout vkCmdDrawIndexed wants. The cost
// is an indirection per vertex fetch during traversal.
//
// INVARIANT, enforced at construction: indices.size() % 3 == 0 and every index
// is < positions.size().
class Mesh {
public:
    Mesh() = default;

    // Takes ownership -- std::move these, they may be tens of megabytes.
    // Throws std::invalid_argument on a malformed index buffer: this is the
    // trust boundary, so it reports rather than asserts.
    Mesh(std::vector<Vec3> positions, std::vector<std::uint32_t> indices);

    std::size_t vertexCount() const { return positions_.size(); }
    std::size_t triangleCount() const { return indices_.size() / 3; }
    bool empty() const { return indices_.empty(); }

    const std::vector<Vec3>& positions() const { return positions_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

    // Precondition: i < triangleCount(). The constructor validates the index
    // buffer's contents, but `i` is caller-supplied and outside that invariant.
    void triangleIndices(std::size_t i, std::uint32_t& a, std::uint32_t& b,
                         std::uint32_t& c) const {
        assert(i < triangleCount() && "triangle index out of range");
        const std::size_t base = i * 3;
        a = indices_[base + 0];
        b = indices_[base + 1];
        c = indices_[base + 2];
    }

    // By value -- 36 bytes is cheaper to copy than to alias.
    Triangle triangle(std::size_t i) const {
        assert(i < triangleCount() && "triangle index out of range");
        const std::size_t base = i * 3;
        return Triangle(positions_[indices_[base + 0]],
                        positions_[indices_[base + 1]],
                        positions_[indices_[base + 2]]);
    }

    // Tight bounds over REFERENCED vertices, cached at construction. This is
    // what a BVH build must use: SAH normalises child area against the parent,
    // so a root inflated by orphan vertices shifts every split decision.
    const AABB& bounds() const { return bounds_; }

    // The whole vertex array, orphans included. Always contains bounds().
    const AABB& vertexBounds() const { return vertexBounds_; }

    // Modification counter: 0 for a freshly constructed mesh, bumped by every
    // mutation. A BVH records it at build and asserts on it, which catches the
    // case a triangle-count check cannot -- the same mesh, moved under a tree
    // that still describes where it used to be. It is per-object, so two
    // DIFFERENT meshes of the same size both read 0 and are not distinguished.
    std::uint64_t revision() const { return revision_; }

    // Reported rather than dropped, so mesh quality shows up in benchmarks
    // instead of quietly changing the primitive count.
    std::size_t countDegenerateTriangles(Scalar tol = kEpsilon) const;

    // Transforms vertices in place and refreshes bounds. For static geometry
    // this is paid once, and keeps traversal free of a per-ray matrix multiply.
    void transform(const Mat4& xf);

private:
    void recomputeBounds();

    std::vector<Vec3> positions_;
    std::vector<std::uint32_t> indices_;
    AABB bounds_;        // referenced geometry (tight)
    AABB vertexBounds_;  // whole vertex array (conservative)
    std::uint64_t revision_{0};
};

}  // namespace geom
