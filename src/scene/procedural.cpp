#include "scene/procedural.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace scene {
namespace {

using geom::Scalar;
using geom::Vec3;

std::uint32_t isqrtRound(std::size_t v) {
    const double r = std::sqrt(static_cast<double>(v));
    const auto rounded = static_cast<std::uint32_t>(r + 0.5);
    return rounded;
}

// mt19937 is reproducible across implementations; std::uniform_real_distribution
// is not, so the mapping to [0,1) is done by hand.
//
// Only the top 24 bits are used, because that is a float's mantissa. Scaling
// the full 32 bits instead rounds 2^32-1 up to 2^32 on conversion and returns
// exactly 1.0, which is outside the range this claims to produce.
Scalar unitFloat(std::mt19937& rng) {
    return static_cast<Scalar>(rng() >> 8) * (Scalar(1) / Scalar(16777216.0));
}

Scalar signedFloat(std::mt19937& rng) { return Scalar(2) * unitFloat(rng) - Scalar(1); }

}  // namespace

geom::Mesh uvSphere(std::size_t targetTriangles, Scalar radius) {
    // 2*segments*(rings-1) triangles; segments ~ 2*(rings-1) keeps quads square.
    const std::uint32_t k = std::max(1u, isqrtRound(targetTriangles / 4));
    const std::uint32_t rings = k + 1;
    const std::uint32_t segments = std::max(3u, 2u * k);

    std::vector<Vec3> positions;
    std::vector<std::uint32_t> indices;
    positions.reserve(static_cast<std::size_t>(rings - 1) * segments + 2);

    positions.push_back(Vec3(Scalar(0), radius, Scalar(0)));
    for (std::uint32_t i = 1; i < rings; ++i) {
        const Scalar theta = geom::kPi * static_cast<Scalar>(i) / static_cast<Scalar>(rings);
        const Scalar y = radius * std::cos(theta);
        const Scalar r = radius * std::sin(theta);
        for (std::uint32_t j = 0; j < segments; ++j) {
            const Scalar phi =
                Scalar(2) * geom::kPi * static_cast<Scalar>(j) / static_cast<Scalar>(segments);
            positions.push_back(Vec3(r * std::cos(phi), y, r * std::sin(phi)));
        }
    }
    const auto southPole = static_cast<std::uint32_t>(positions.size());
    positions.push_back(Vec3(Scalar(0), -radius, Scalar(0)));

    const auto ringBase = [segments](std::uint32_t ring) { return 1u + (ring - 1u) * segments; };

    for (std::uint32_t j = 0; j < segments; ++j) {
        const std::uint32_t a = ringBase(1) + j;
        const std::uint32_t b = ringBase(1) + (j + 1) % segments;
        indices.insert(indices.end(), {0u, b, a});
    }
    for (std::uint32_t i = 1; i + 1 < rings; ++i) {
        for (std::uint32_t j = 0; j < segments; ++j) {
            const std::uint32_t jn = (j + 1) % segments;
            const std::uint32_t a = ringBase(i) + j;
            const std::uint32_t b = ringBase(i) + jn;
            const std::uint32_t c = ringBase(i + 1) + jn;
            const std::uint32_t d = ringBase(i + 1) + j;
            indices.insert(indices.end(), {a, b, c, a, c, d});
        }
    }
    for (std::uint32_t j = 0; j < segments; ++j) {
        const std::uint32_t a = ringBase(rings - 1) + j;
        const std::uint32_t b = ringBase(rings - 1) + (j + 1) % segments;
        indices.insert(indices.end(), {southPole, a, b});
    }

    return geom::Mesh(std::move(positions), std::move(indices));
}

geom::Mesh grid(std::size_t targetTriangles, Scalar extent) {
    const std::uint32_t cells = std::max(1u, isqrtRound(targetTriangles / 2));
    const std::uint32_t verts = cells + 1;
    const Scalar step = Scalar(2) * extent / static_cast<Scalar>(cells);

    std::vector<Vec3> positions;
    positions.reserve(static_cast<std::size_t>(verts) * verts);
    for (std::uint32_t iz = 0; iz < verts; ++iz) {
        for (std::uint32_t ix = 0; ix < verts; ++ix) {
            positions.push_back(Vec3(-extent + step * static_cast<Scalar>(ix), Scalar(0),
                                     -extent + step * static_cast<Scalar>(iz)));
        }
    }

    std::vector<std::uint32_t> indices;
    indices.reserve(static_cast<std::size_t>(cells) * cells * 6);
    for (std::uint32_t iz = 0; iz < cells; ++iz) {
        for (std::uint32_t ix = 0; ix < cells; ++ix) {
            const std::uint32_t a = iz * verts + ix;
            const std::uint32_t b = a + 1;
            const std::uint32_t c = a + verts;
            const std::uint32_t d = c + 1;
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    }

    return geom::Mesh(std::move(positions), std::move(indices));
}

geom::Mesh triangleSoup(std::size_t triangleCount, std::uint32_t seed, Scalar extent,
                        Scalar triangleSize) {
    std::mt19937 rng(seed);

    std::vector<Vec3> positions;
    std::vector<std::uint32_t> indices;
    positions.reserve(triangleCount * 3);
    indices.reserve(triangleCount * 3);

    for (std::size_t i = 0; i < triangleCount; ++i) {
        const Vec3 c(signedFloat(rng) * extent, signedFloat(rng) * extent,
                     signedFloat(rng) * extent);
        const auto base = static_cast<std::uint32_t>(positions.size());
        for (int v = 0; v < 3; ++v) {
            positions.push_back(c + Vec3(signedFloat(rng), signedFloat(rng), signedFloat(rng)) *
                                        triangleSize);
        }
        indices.insert(indices.end(), {base, base + 1, base + 2});
    }

    return geom::Mesh(std::move(positions), std::move(indices));
}

}  // namespace scene
