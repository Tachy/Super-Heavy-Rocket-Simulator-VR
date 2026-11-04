#include "starsim/environment/cube_face_cell.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

namespace {

// Face axes of cube_sphere.gd (u x v = normal, outward).
constexpr std::array<Vec3, 6> normals = {Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0},
                                         Vec3{0, -1, 0}, Vec3{0, 0, 1}, Vec3{0, 0, -1}};
constexpr std::array<Vec3, 6> us = {Vec3{0, 1, 0}, Vec3{0, -1, 0}, Vec3{-1, 0, 0},
                                    Vec3{1, 0, 0}, Vec3{1, 0, 0}, Vec3{1, 0, 0}};
constexpr std::array<Vec3, 6> vs = {Vec3{0, 0, 1}, Vec3{0, 0, 1}, Vec3{0, 0, 1},
                                    Vec3{0, 0, 1}, Vec3{0, 1, 0}, Vec3{0, -1, 0}};

constexpr double quarter_pi = 0.25 * constants::pi;

std::int64_t cells_per_edge(int level) { return std::int64_t{1} << level; }

// Margin on the local cell width when collecting the cells around a point (metric change over the range, [E]).
constexpr double cell_range_margin = 1.1;

}  // namespace

const Vec3& face_normal(int face) { return normals[static_cast<size_t>(face)]; }
const Vec3& face_u(int face) { return us[static_cast<size_t>(face)]; }
const Vec3& face_v(int face) { return vs[static_cast<size_t>(face)]; }

double face_warp(double s) {
    const double m = std::abs(s);
    const double w = m == 1.0 ? 1.0 : std::tan(quarter_pi * m);
    return s < 0.0 ? -w : w;
}

Vec3 face_direction(int face, double s, double t) {
    const double a = face_warp(s), b = face_warp(t);
    return (face_normal(face) + face_u(face) * a + face_v(face) * b) / std::sqrt(1.0 + (a * a + b * b));
}

FaceCoord face_coord_on(int face, const Vec3& unit) {
    const double major = dot(unit, face_normal(face));
    return {face, std::atan(dot(unit, face_u(face)) / major) / quarter_pi,
            std::atan(dot(unit, face_v(face)) / major) / quarter_pi};
}

FaceCoord face_coord_of(const Vec3& unit) {
    const double mx = std::abs(unit.x), my = std::abs(unit.y), mz = std::abs(unit.z);
    int face = 0;
    if (my >= mx && my >= mz) {
        face = 2;
    } else if (mz >= mx && mz >= my) {
        face = 4;
    }
    if (dot(unit, face_normal(face)) < 0.0) ++face;
    return face_coord_on(face, unit);
}

FaceFrame face_frame(int face, double s, double t, double sphere_radius_m) {
    const double a = face_warp(s), b = face_warp(t);
    const double q = 1.0 + (a * a + b * b);
    const double q15 = q * std::sqrt(q);
    const Vec3& n = face_normal(face);
    const Vec3& u = face_u(face);
    const Vec3& v = face_v(face);
    // d = (n + a u + b v) / sqrt(q); dd/da = (u (1 + b^2) - a n - a b v) / q^1.5, |dd/da| = sqrt(1 + b^2) / q.
    const Vec3 dda = (u * (1.0 + b * b) - n * a - v * (a * b)) / q15;
    const Vec3 ddb = (v * (1.0 + a * a) - n * b - u * (a * b)) / q15;
    FaceFrame f;
    f.e_s = math::normalized(dda);
    f.e_t = math::normalized(ddb);
    f.m_s = sphere_radius_m * quarter_pi * (1.0 + a * a) * std::sqrt(1.0 + b * b) / q;
    f.m_t = sphere_radius_m * quarter_pi * (1.0 + b * b) * std::sqrt(1.0 + a * a) / q;
    f.scale = std::sqrt((1.0 + a * a) * (1.0 + b * b) / q15);
    return f;
}

FaceCell face_cell_of(const Vec3& unit, int level) {
    const FaceCoord c = face_coord_of(unit);
    const std::int64_t count = cells_per_edge(level);
    const auto index = [count](double s) {
        return std::clamp(static_cast<std::int64_t>(std::floor((s + 1.0) * 0.5 * static_cast<double>(count))),
                          std::int64_t{0}, count - 1);
    };
    return {c.face, level, index(c.s), index(c.t)};
}

Vec3 face_cell_point(const FaceCell& c, double s01, double t01) {
    const double count = static_cast<double>(cells_per_edge(c.level));
    return face_direction(c.face, -1.0 + 2.0 * (static_cast<double>(c.i) + s01) / count,
                          -1.0 + 2.0 * (static_cast<double>(c.j) + t01) / count);
}

std::array<FaceCell, 9> face_cell_neighbourhood(const FaceCell& c) {
    std::array<FaceCell, 9> out;
    const std::int64_t count = cells_per_edge(c.level);
    size_t k = 0;
    for (int dj = -1; dj <= 1; ++dj) {
        for (int di = -1; di <= 1; ++di) {
            const std::int64_t i = c.i + di, j = c.j + dj;
            if (i >= 0 && i < count && j >= 0 && j < count) {
                out[k++] = {c.face, c.level, i, j};
            } else {
                out[k++] = face_cell_of(face_cell_point({c.face, c.level, i, j}, 0.5, 0.5), c.level);
            }
        }
    }
    return out;
}

std::vector<FaceCell> face_cells_within(const Vec3& unit, int level, double radius_m, double sphere_radius_m) {
    std::vector<FaceCell> out;
    const std::int64_t count = cells_per_edge(level);
    const double n = static_cast<double>(count);
    for (int face = 0; face < 6; ++face) {
        // A face whose centre is 90 deg or more away holds no point near `unit` (radius_m << R).
        if (dot(unit, face_normal(face)) <= 1e-3) continue;
        const FaceCoord c = face_coord_on(face, unit);
        const double sc = std::clamp(c.s, -1.0, 1.0), tc = std::clamp(c.t, -1.0, 1.0);
        const FaceFrame f = face_frame(face, sc, tc, sphere_radius_m);
        // Width across the lines of constant s (t): metres per cell times the sine of the angle between e_s and e_t.
        const double sin_angle = std::max(norm(cross(f.e_s, f.e_t)), 0.5);
        const double cells_s = cell_range_margin * radius_m / (f.m_s * 2.0 / n * sin_angle);
        const double cells_t = cell_range_margin * radius_m / (f.m_t * 2.0 / n * sin_angle);
        const double u = (c.s + 1.0) * 0.5 * n, v = (c.t + 1.0) * 0.5 * n;
        const auto lo = [count](double x) {
            return std::clamp(static_cast<std::int64_t>(std::floor(x)), std::int64_t{0}, count);
        };
        const std::int64_t i0 = lo(u - cells_s), i1 = std::min(lo(u + cells_s), count - 1);
        const std::int64_t j0 = lo(v - cells_t), j1 = std::min(lo(v + cells_t), count - 1);
        if (i0 >= count || j0 >= count || u + cells_s < 0.0 || v + cells_t < 0.0) continue;
        for (std::int64_t j = j0; j <= j1; ++j) {
            for (std::int64_t i = i0; i <= i1; ++i) out.push_back({face, level, i, j});
        }
    }
    return out;
}

double face_cell_nominal_size_m(int level, double sphere_radius_m) {
    return sphere_radius_m * 0.5 * constants::pi / static_cast<double>(cells_per_edge(level));
}

int face_cell_level_for_size(double cell_size_m, double sphere_radius_m) {
    const double ratio = sphere_radius_m * 0.5 * constants::pi / cell_size_m;
    if (!(ratio > 1.0)) return 0;
    return std::min(static_cast<int>(std::floor(std::log2(ratio))), max_face_cell_level);
}

double face_cell_area_m2(const FaceCell& c, double sphere_radius_m) {
    const double count = static_cast<double>(cells_per_edge(c.level));
    const double ds = 2.0 / count;
    const double s = -1.0 + ds * (static_cast<double>(c.i) + 0.5);
    const double t = -1.0 + ds * (static_cast<double>(c.j) + 0.5);
    const double a = face_warp(s), b = face_warp(t);
    const double q = 1.0 + (a * a + b * b);
    const double jacobian = quarter_pi * quarter_pi * (1.0 + a * a) * (1.0 + b * b) / (q * std::sqrt(q));
    return sphere_radius_m * sphere_radius_m * jacobian * ds * ds;
}

}  // namespace starsim::environment
