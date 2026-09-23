#include "rendering/bvh_visualization.hpp"

#include <array>

namespace rendering {
namespace {

constexpr std::size_t kEdgesPerAabb = 12;
constexpr std::size_t kVerticesPerAabb = kEdgesPerAabb * 2;

void appendAabbWireframe(const geom::AABB& bounds, const geom::Vec3& colour,
                         std::vector<LineVertex>& vertices) {
    const std::array<geom::Vec3, 8> corners{{
        {bounds.min.x, bounds.min.y, bounds.min.z},
        {bounds.max.x, bounds.min.y, bounds.min.z},
        {bounds.max.x, bounds.max.y, bounds.min.z},
        {bounds.min.x, bounds.max.y, bounds.min.z},
        {bounds.min.x, bounds.min.y, bounds.max.z},
        {bounds.max.x, bounds.min.y, bounds.max.z},
        {bounds.max.x, bounds.max.y, bounds.max.z},
        {bounds.min.x, bounds.max.y, bounds.max.z},
    }};
    constexpr std::array<std::array<std::size_t, 2>, kEdgesPerAabb> kEdges{{
        {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
        {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
        {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}},
    }};

    for (const auto& edge : kEdges) {
        vertices.push_back(LineVertex{corners[edge[0]], colour});
        vertices.push_back(LineVertex{corners[edge[1]], colour});
    }
}

}  // namespace

BVHVisualizationModel::BVHVisualizationModel(const bvh::BVH& tree) {
    const std::vector<bvh::BVHNode>& treeNodes = tree.nodes();
    bounds_.reserve(treeNodes.size());
    metadata_.resize(treeNodes.size());

    for (std::size_t i = 0; i < treeNodes.size(); ++i) {
        const bvh::BVHNode& node = treeNodes[i];
        const NodeId nodeId = static_cast<NodeId>(i);
        NodeMetadata& metadata = metadata_[i];
        metadata.nodeId = nodeId;
        metadata.surfaceArea = node.bounds.surfaceArea();
        metadata.volume = node.bounds.volume();
        if (!node.isLeaf()) {
            metadata.leftChild = node.leftChild();
            metadata.rightChild = node.rightChild();
        }
        bounds_.push_back(node.bounds);
    }

    // Child node IDs are always greater than their parent's allocation ID.
    // This one forward pass derives parent links and depths without scanning.
    for (std::size_t i = 0; i < metadata_.size(); ++i) {
        const NodeMetadata& metadata = metadata_[i];
        if (metadata.leftChild == kInvalidNodeId) continue;

        NodeMetadata& left = metadata_[metadata.leftChild];
        NodeMetadata& right = metadata_[metadata.rightChild];
        left.parent = metadata.nodeId;
        right.parent = metadata.nodeId;
        left.depth = metadata.depth + 1;
        right.depth = metadata.depth + 1;
    }

    // Reverse allocation order is post-order for the parent/child relation,
    // so subtree primitive counts are available when an internal node is read.
    for (std::size_t i = metadata_.size(); i > 0; --i) {
        NodeMetadata& metadata = metadata_[i - 1];
        const bvh::BVHNode& node = treeNodes[i - 1];
        if (node.isLeaf()) {
            metadata.primitiveCount = node.count;
        } else {
            metadata.primitiveCount = metadata_[metadata.leftChild].primitiveCount +
                                       metadata_[metadata.rightChild].primitiveCount;
        }
    }
}

std::optional<NodeMetadata> BVHVisualizationModel::selectedNode(NodeId nodeId) const {
    if (static_cast<std::size_t>(nodeId) >= metadata_.size()) return std::nullopt;
    return metadata_[nodeId];
}

std::vector<LineVertex> BVHVisualizationModel::aabbWireframeAtDepth(
    std::uint32_t depth, const geom::Vec3& colour) const {
    std::size_t selectedCount = 0;
    for (const NodeMetadata& metadata : metadata_) {
        if (metadata.depth == depth) ++selectedCount;
    }

    std::vector<LineVertex> vertices;
    vertices.reserve(selectedCount * kVerticesPerAabb);
    for (std::size_t i = 0; i < metadata_.size(); ++i) {
        if (metadata_[i].depth == depth) {
            appendAabbWireframe(bounds_[i], colour, vertices);
        }
    }
    return vertices;
}

}  // namespace rendering
