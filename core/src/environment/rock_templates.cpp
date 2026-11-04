#include "starsim/environment/rock_templates.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <utility>

#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

namespace {

constexpr double frequency_min = 1.5, frequency_span = 3.5;  // [E] rad per unit direction: 1-3 lobes round the rock
constexpr double amplitude_gain = 0.30;                       // [E] a_k = gain / f_k: ~20 % radius scatter (1 sigma)
constexpr int sample_points = 20000;                          // Fibonacci directions that find the maximum
// The sampled maximum can miss a peak between samples (worst case ~ 0.5 f'' delta^2 ~ 5e-4); a margin keeps r <= 1.
constexpr double scale_margin = 1.005;
constexpr double normal_step = 1e-4;                          // rad, central differences for the normal
constexpr std::array<int, RockTemplates::lods> subdivisions = {4, 3, 2, 1};  // icosphere levels per LOD [E]

double unit(std::uint64_t seed, std::uint64_t a, std::uint64_t b) {
    return hash_unit(surface_hash(surface_stream::rocks << 8, 0x7E3, seed * 64 + a, b));
}

Vec3 any_perpendicular(const Vec3& d) {
    return math::normalized(cross(std::abs(d.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, d));
}

// Icosphere: subdivided icosahedron, shared vertices (watertight), vertices on the unit sphere.
struct Icosphere {
    std::vector<Vec3> vertices;
    std::vector<int> indices;
};

Icosphere make_icosphere(int subdivision) {
    const double t = 0.5 * (1.0 + std::sqrt(5.0));
    Icosphere s;
    for (const Vec3& v : {Vec3{-1, t, 0}, Vec3{1, t, 0}, Vec3{-1, -t, 0}, Vec3{1, -t, 0}, Vec3{0, -1, t}, Vec3{0, 1, t},
                          Vec3{0, -1, -t}, Vec3{0, 1, -t}, Vec3{t, 0, -1}, Vec3{t, 0, 1}, Vec3{-t, 0, -1},
                          Vec3{-t, 0, 1}}) {
        s.vertices.push_back(math::normalized(v));
    }
    s.indices = {0, 11, 5, 0, 5, 1,  0, 1,  7,  0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11, 4,  11, 10, 2,  10, 7, 6, 7, 1, 8,
                 3, 9,  4, 3, 4, 2,  3, 2,  6,  3, 6,  8,  3, 8,  9,  4, 9, 5, 2, 4,  11, 6,  2,  10, 8,  6, 7, 9, 8, 1};
    for (int level = 0; level < subdivision; ++level) {
        std::map<std::pair<int, int>, int> midpoint;
        const auto middle = [&](int a, int b) {
            const std::pair<int, int> key{std::min(a, b), std::max(a, b)};
            const auto it = midpoint.find(key);
            if (it != midpoint.end()) return it->second;
            s.vertices.push_back(math::normalized(s.vertices[static_cast<size_t>(a)] + s.vertices[static_cast<size_t>(b)]));
            return midpoint[key] = static_cast<int>(s.vertices.size()) - 1;
        };
        std::vector<int> next;
        for (size_t k = 0; k < s.indices.size(); k += 3) {
            const int a = s.indices[k], b = s.indices[k + 1], c = s.indices[k + 2];
            const int ab = middle(a, b), bc = middle(b, c), ca = middle(c, a);
            for (const int tri : {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca}) next.push_back(tri);
        }
        s.indices = std::move(next);
    }
    return s;
}

}  // namespace

double RockTemplates::raw_radius(const Shape& shape, const Vec3& d) const {
    double f = 0.0;
    for (const Wave& w : shape.wave) f += w.amplitude * std::cos(w.frequency * dot(d, w.direction) + w.phase);
    return std::exp(f);
}

RockTemplates::RockTemplates() {
    for (int id = 0; id < count; ++id) {
        Shape& shape = shapes_[static_cast<size_t>(id)];
        const auto seed = static_cast<std::uint64_t>(id);
        for (int k = 0; k < waves; ++k) {
            Wave& w = shape.wave[static_cast<size_t>(k)];
            const auto ku = static_cast<std::uint64_t>(k);
            const double z = 2.0 * unit(seed, ku * 4, 0) - 1.0, phi = constants::two_pi * unit(seed, ku * 4 + 1, 0);
            const double rho = std::sqrt(1.0 - z * z);
            w.direction = {rho * std::cos(phi), rho * std::sin(phi), z};
            w.frequency = frequency_min + frequency_span * unit(seed, ku * 4 + 2, 0);
            w.amplitude = amplitude_gain / w.frequency;
            w.phase = constants::two_pi * unit(seed, ku * 4 + 3, 0);
        }
        double max_r = 0.0;
        const double golden = constants::pi * (3.0 - std::sqrt(5.0));
        for (int n = 0; n < sample_points; ++n) {
            const double z = 1.0 - 2.0 * (n + 0.5) / sample_points, rho = std::sqrt(1.0 - z * z);
            max_r = std::max(max_r, raw_radius(shape, {rho * std::cos(golden * n), rho * std::sin(golden * n), z}));
        }
        shape.scale = 1.0 / (max_r * scale_margin);

        for (int lod = 0; lod < lods; ++lod) {
            const Icosphere ico = make_icosphere(subdivisions[static_cast<size_t>(lod)]);
            TemplateMesh& mesh = meshes_[static_cast<size_t>(id)][static_cast<size_t>(lod)];
            mesh.indices = ico.indices;
            for (const Vec3& d : ico.vertices) {
                mesh.vertices.push_back(d * radius(id, d));
                mesh.normals.push_back(normal(id, d));
            }
        }
    }
}

const RockTemplates& RockTemplates::instance() {
    static const RockTemplates templates;
    return templates;
}

double RockTemplates::radius(int id, const Vec3& dir_local) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    return shape.scale * raw_radius(shape, math::normalized(dir_local));
}

Vec3 RockTemplates::normal(int id, const Vec3& dir_local) const {
    // Surface X = r(d) d: n ~ d - grad_S r / r (tangent gradient on the sphere).
    const Vec3 d = math::normalized(dir_local);
    const Vec3 t1 = any_perpendicular(d), t2 = cross(d, t1);
    const auto slope = [&](const Vec3& t) {
        return (radius(id, d + t * normal_step) - radius(id, d - t * normal_step)) / (2.0 * normal_step);
    };
    return math::normalized(d - (t1 * slope(t1) + t2 * slope(t2)) / radius(id, d));
}

const TemplateMesh& RockTemplates::mesh(int id, int lod) const {
    return meshes_[static_cast<size_t>(id)][static_cast<size_t>(lod)];
}

}  // namespace starsim::environment
