#pragma once

#include <array>
#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Triangle mesh of a unit-size rock template (vertices on the template's surface, outward unit normals).
struct TemplateMesh {
    std::vector<math::Vec3> vertices, normals;
    std::vector<int> indices;  // counter-clockwise triangles seen from outside
};

// Star-shaped rock shapes: 16 seeded radial functions r(dir) > 0 on the unit sphere, built from a few random plane
// waves (r = exp(sum a_k cos(f_k dir.d_k + p_k)), a_k ~ 1/f_k), scaled so that max r = 1 (a rock's semi-axes a, b, c
// then bound it exactly). Meshes: icosphere subdivision 4, 3, 2, 1 (2562, 642, 162, 42 vertices) with vertex = dir r(dir).
// Everything derives from the integer hash: identical on every machine.
class RockTemplates {
public:
    static constexpr int count = 16;  // [E] enough that neighbours do not look alike
    static constexpr int lods = 4;    // [E] near, mid, far, speck
    static const RockTemplates& instance();

    // Unit-size radial function (dir_local need not be normalised).
    double radius(int id, const math::Vec3& dir_local) const;
    // Outward unit normal of the unit-size surface in direction dir_local.
    math::Vec3 normal(int id, const math::Vec3& dir_local) const;
    const TemplateMesh& mesh(int id, int lod) const;

private:
    RockTemplates();

    static constexpr int waves = 8;  // [E]
    struct Wave {
        math::Vec3 direction;
        double frequency = 0.0, amplitude = 0.0, phase = 0.0;
    };
    struct Shape {
        std::array<Wave, waves> wave;
        double scale = 1.0;  // 1 / sampled maximum
    };
    double raw_radius(const Shape& shape, const math::Vec3& dir_unit) const;

    std::array<Shape, count> shapes_;
    std::array<std::array<TemplateMesh, lods>, count> meshes_;
};

}  // namespace starsim::environment
