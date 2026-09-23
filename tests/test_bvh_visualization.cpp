#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "bvh/bvh.hpp"
#include "geometry/mesh.hpp"
#include "rendering/bvh_visualization.hpp"

namespace {

using bvh::BVH;
using bvh::BuildConfig;
using bvh::SplitStrategy;
using geom::AABB;
using geom::Mesh;
using geom::Scalar;
using geom::Vec3;
using rendering::BVHVisualizationModel;
using rendering::LineVertex;
using rendering::NodeMetadata;
using rendering::kInvalidNodeId;

Mesh fourSeparatedTriangles() {
    std::vector<Vec3> positions;
    std::vector<std::uint32_t> indices;
    positions.reserve(12);
    indices.reserve(12);
    for (std::uint32_t i = 0; i < 4; ++i) {
        const Scalar x = static_cast<Scalar>(i * 2);
        const std::uint32_t base = static_cast<std::uint32_t>(positions.size());
        positions.push_back(Vec3(x, 0.0f, 0.0f));
        positions.push_back(Vec3(x + 1.0f, 1.0f, 0.0f));
        positions.push_back(Vec3(x, 0.0f, 1.0f));
        indices.insert(indices.end(), {base, base + 1, base + 2});
    }
    return Mesh(std::move(positions), std::move(indices));
}

BVH makeFourLeafTree() {
    BVH tree;
    BuildConfig config;
    config.maxLeafSize = 1;
    config.strategy = SplitStrategy::ObjectMedian;
    tree.build(fourSeparatedTriangles(), config);
    return tree;
}

void expectSameNodes(const std::vector<bvh::BVHNode>& expected,
                     const std::vector<bvh::BVHNode>& actual) {
    ASSERT_EQ(expected.size(), actual.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(actual[i].bounds, expected[i].bounds) << i;
        EXPECT_EQ(actual[i].leftOrFirst, expected[i].leftOrFirst) << i;
        EXPECT_EQ(actual[i].count, expected[i].count) << i;
    }
}

void expectVertex(const LineVertex& vertex, const Vec3& position, const Vec3& colour) {
    EXPECT_EQ(vertex.position, position);
    EXPECT_EQ(vertex.colour, colour);
}

}  // namespace

TEST(BVHVisualization, EmptyTreeHasNoMetadataOrLines) {
    const BVH tree;
    const BVHVisualizationModel model(tree);

    EXPECT_EQ(model.nodeCount(), 0U);
    EXPECT_TRUE(model.nodes().empty());
    EXPECT_FALSE(model.selectedNode(0).has_value());
    EXPECT_TRUE(model.aabbWireframeAtDepth(0, Vec3(1.0f)).empty());
}

TEST(BVHVisualization, DepthFilteringEmitsOnlyMatchingNodesInNodeOrder) {
    const BVH tree = makeFourLeafTree();
    ASSERT_EQ(tree.nodes().size(), 7U);
    const BVHVisualizationModel model(tree);
    const Vec3 colour(0.25f, 0.5f, 0.75f);

    const std::vector<LineVertex> root = model.aabbWireframeAtDepth(0, colour);
    const std::vector<LineVertex> levelOne = model.aabbWireframeAtDepth(1, colour);
    const std::vector<LineVertex> leaves = model.aabbWireframeAtDepth(2, colour);

    ASSERT_EQ(root.size(), 24U);
    ASSERT_EQ(levelOne.size(), 48U);
    ASSERT_EQ(leaves.size(), 96U);
    EXPECT_TRUE(model.aabbWireframeAtDepth(3, colour).empty());

    // The first vertex for each box is its min corner. Node IDs 1 then 2 are
    // the stored depth-one order, not an order chosen from geometry.
    expectVertex(levelOne[0], tree.nodes()[1].bounds.min, colour);
    expectVertex(levelOne[24], tree.nodes()[2].bounds.min, colour);
}

TEST(BVHVisualization, AabbWireframeHasTwelveEdgesAndCanonicalEndpoints) {
    const BVH tree = makeFourLeafTree();
    const BVHVisualizationModel model(tree);
    const Vec3 colour(0.1f, 0.2f, 0.3f);
    const std::vector<LineVertex> vertices = model.aabbWireframeAtDepth(0, colour);

    ASSERT_EQ(vertices.size(), 24U);
    const AABB expected(Vec3(0.0f, 0.0f, 0.0f), Vec3(7.0f, 1.0f, 1.0f));
    expectVertex(vertices[0], expected.min, colour);
    expectVertex(vertices[1], Vec3(7.0f, 0.0f, 0.0f), colour);
    expectVertex(vertices[2], Vec3(7.0f, 0.0f, 0.0f), colour);
    expectVertex(vertices[3], Vec3(7.0f, 1.0f, 0.0f), colour);
    expectVertex(vertices[16], expected.min, colour);
    expectVertex(vertices[17], Vec3(0.0f, 0.0f, 1.0f), colour);
    expectVertex(vertices[22], Vec3(0.0f, 1.0f, 0.0f), colour);
    expectVertex(vertices[23], Vec3(0.0f, 1.0f, 1.0f), colour);
}

TEST(BVHVisualization, RepeatedModelsProduceDeterministicOutput) {
    const BVH tree = makeFourLeafTree();
    const BVHVisualizationModel first(tree);
    const BVHVisualizationModel second(tree);
    const Vec3 colour(0.9f, 0.8f, 0.7f);

    EXPECT_EQ(first.nodes().size(), second.nodes().size());
    for (std::size_t i = 0; i < first.nodes().size(); ++i) {
        const NodeMetadata& a = first.nodes()[i];
        const NodeMetadata& b = second.nodes()[i];
        EXPECT_EQ(a.nodeId, b.nodeId);
        EXPECT_EQ(a.depth, b.depth);
        EXPECT_EQ(a.primitiveCount, b.primitiveCount);
        EXPECT_EQ(a.surfaceArea, b.surfaceArea);
        EXPECT_EQ(a.volume, b.volume);
        EXPECT_EQ(a.parent, b.parent);
        EXPECT_EQ(a.leftChild, b.leftChild);
        EXPECT_EQ(a.rightChild, b.rightChild);
    }

    const std::vector<LineVertex> firstLines = first.aabbWireframeAtDepth(2, colour);
    const std::vector<LineVertex> secondLines = second.aabbWireframeAtDepth(2, colour);
    ASSERT_EQ(firstLines.size(), secondLines.size());
    for (std::size_t i = 0; i < firstLines.size(); ++i) {
        EXPECT_EQ(firstLines[i].position, secondLines[i].position);
        EXPECT_EQ(firstLines[i].colour, secondLines[i].colour);
    }
}

TEST(BVHVisualization, SelectedNodeReportsRootInternalAndLeafMetadata) {
    const BVH tree = makeFourLeafTree();
    const BVHVisualizationModel model(tree);

    const std::optional<NodeMetadata> root = model.selectedNode(0);
    ASSERT_TRUE(root.has_value());
    EXPECT_EQ(root->nodeId, 0U);
    EXPECT_EQ(root->depth, 0U);
    EXPECT_EQ(root->primitiveCount, 4U);
    EXPECT_EQ(root->surfaceArea, tree.nodes()[0].bounds.surfaceArea());
    EXPECT_EQ(root->volume, tree.nodes()[0].bounds.volume());
    EXPECT_EQ(root->parent, kInvalidNodeId);
    EXPECT_EQ(root->leftChild, 1U);
    EXPECT_EQ(root->rightChild, 2U);

    const std::optional<NodeMetadata> internal = model.selectedNode(1);
    ASSERT_TRUE(internal.has_value());
    EXPECT_EQ(internal->nodeId, 1U);
    EXPECT_EQ(internal->depth, 1U);
    EXPECT_EQ(internal->primitiveCount, 2U);
    EXPECT_EQ(internal->surfaceArea, tree.nodes()[1].bounds.surfaceArea());
    EXPECT_EQ(internal->volume, tree.nodes()[1].bounds.volume());
    EXPECT_EQ(internal->parent, 0U);
    EXPECT_EQ(internal->leftChild, 3U);
    EXPECT_EQ(internal->rightChild, 4U);

    const std::optional<NodeMetadata> leaf = model.selectedNode(3);
    ASSERT_TRUE(leaf.has_value());
    EXPECT_EQ(leaf->nodeId, 3U);
    EXPECT_EQ(leaf->depth, 2U);
    EXPECT_EQ(leaf->primitiveCount, 1U);
    EXPECT_EQ(leaf->surfaceArea, tree.nodes()[3].bounds.surfaceArea());
    EXPECT_EQ(leaf->volume, tree.nodes()[3].bounds.volume());
    EXPECT_EQ(leaf->parent, 1U);
    EXPECT_EQ(leaf->leftChild, kInvalidNodeId);
    EXPECT_EQ(leaf->rightChild, kInvalidNodeId);
}

TEST(BVHVisualization, InvalidSelectionAndLeafChildrenUseInvalidSentinel) {
    const BVH tree = makeFourLeafTree();
    const BVHVisualizationModel model(tree);

    EXPECT_FALSE(model.selectedNode(kInvalidNodeId).has_value());
    EXPECT_FALSE(model.selectedNode(7).has_value());
    for (std::size_t i = 0; i < tree.nodes().size(); ++i) {
        if (!tree.nodes()[i].isLeaf()) continue;
        const std::optional<NodeMetadata> metadata =
            model.selectedNode(static_cast<rendering::NodeId>(i));
        ASSERT_TRUE(metadata.has_value());
        EXPECT_EQ(metadata->leftChild, kInvalidNodeId) << i;
        EXPECT_EQ(metadata->rightChild, kInvalidNodeId) << i;
    }
}

TEST(BVHVisualization, ModelDoesNotChangeSourceNodesOrPrimitiveIndices) {
    const BVH tree = makeFourLeafTree();
    const std::vector<bvh::BVHNode> originalNodes = tree.nodes();
    const std::vector<std::uint32_t> originalIndices = tree.primitiveIndices();

    const BVHVisualizationModel model(tree);
    static_cast<void>(model.selectedNode(0));
    static_cast<void>(model.aabbWireframeAtDepth(1, Vec3(1.0f, 0.0f, 0.0f)));

    expectSameNodes(originalNodes, tree.nodes());
    EXPECT_EQ(tree.primitiveIndices(), originalIndices);
}
