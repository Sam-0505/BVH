#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "bvh/bvh.hpp"
#include "geometry/scalar.hpp"
#include "geometry/vec3.hpp"

namespace rendering {

using NodeId = std::uint32_t;
inline constexpr NodeId kInvalidNodeId = std::numeric_limits<NodeId>::max();

// Two consecutive vertices form one line segment. This layout maps directly
// to a Vulkan line-list vertex buffer without requiring renderer headers here.
struct LineVertex {
    geom::Vec3 position{};
    geom::Vec3 colour{};
};

struct NodeMetadata {
    NodeId nodeId{kInvalidNodeId};
    std::uint32_t depth{0};
    std::uint32_t primitiveCount{0};
    geom::Scalar surfaceArea{0};
    geom::Scalar volume{0};
    NodeId parent{kInvalidNodeId};
    NodeId leftChild{kInvalidNodeId};
    NodeId rightChild{kInvalidNodeId};
};

// Immutable snapshot of a BVH's structure and bounds at construction time.
// Rebuild this model after rebuilding the source BVH to display that new tree.
class BVHVisualizationModel {
public:
    explicit BVHVisualizationModel(const bvh::BVH& tree);

    std::size_t nodeCount() const { return metadata_.size(); }
    const std::vector<NodeMetadata>& nodes() const { return metadata_; }

    // UI-provided IDs may be stale or invalid, so failed selection is reported
    // without asserting.
    std::optional<NodeMetadata> selectedNode(NodeId nodeId) const;

    // Emits twelve segments (twenty-four vertices) for every node at `depth`,
    // in the BVH's deterministic node-vector order.
    std::vector<LineVertex> aabbWireframeAtDepth(std::uint32_t depth,
                                                 const geom::Vec3& colour) const;

private:
    std::vector<geom::AABB> bounds_;
    std::vector<NodeMetadata> metadata_;
};

}  // namespace rendering
