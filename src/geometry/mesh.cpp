#include "geometry/mesh.hpp"

#include <stdexcept>
#include <string>

namespace geom {

Mesh::Mesh(std::vector<Vec3> positions, std::vector<std::uint32_t> indices)
    : positions_(std::move(positions)), indices_(std::move(indices)) {
    if (indices_.size() % 3 != 0) {
        throw std::invalid_argument("Mesh: index count " + std::to_string(indices_.size()) +
                                    " is not a multiple of 3");
    }
    // Validate once at the boundary so traversal can index without bounds checks.
    // O(n) here buys us an unchecked inner loop for the lifetime of the mesh.
    const std::uint32_t vertexCountU32 = static_cast<std::uint32_t>(positions_.size());
    for (std::size_t i = 0; i < indices_.size(); ++i) {
        if (indices_[i] >= vertexCountU32) {
            throw std::invalid_argument("Mesh: index " + std::to_string(indices_[i]) + " at slot " +
                                        std::to_string(i) + " exceeds vertex count " +
                                        std::to_string(positions_.size()));
        }
    }
    recomputeBounds();
}

void Mesh::recomputeBounds() {
    bounds_ = AABB{};
    for (const Vec3& p : positions_) bounds_.extend(p);
}

AABB Mesh::computeTriangleBounds() const {
    AABB b;
    for (std::uint32_t index : indices_) b.extend(positions_[index]);
    return b;
}

std::size_t Mesh::countDegenerateTriangles(Scalar tol) const {
    std::size_t count = 0;
    const std::size_t n = triangleCount();
    for (std::size_t i = 0; i < n; ++i) {
        if (triangle(i).isDegenerate(tol)) ++count;
    }
    return count;
}

void Mesh::transform(const Mat4& xf) {
    for (Vec3& p : positions_) p = transformPoint(xf, p);
    recomputeBounds();
}

}  // namespace geom
