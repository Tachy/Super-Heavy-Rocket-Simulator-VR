// Procedural lunar surface detail (Task 2 of the Moon procedural surface plan): craters and regolith roughness below
// the LOLA grid - determinism, measured statistics, band limits, seams, the tile blend, poles.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using environment::LunarSurfaceDetail;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double R = 1737400.0;
constexpr double site_lat = -84.79 * deg, site_lon = 29.2 * deg;  // tile centre (the shipped site's latitude)
constexpr double global_lat = -60.0 * deg, global_lon = 100.0 * deg;  // global grid only

Vec3 direction(double lat, double lon) {
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

// Tangent frame at a direction and a point x, y metres away in it.
struct Tangent {
    Vec3 c, e1, e2;
};
Tangent tangent(const Vec3& c) {
    const Vec3 e1 = math::normalized(cross(std::abs(c.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, c));
    return {c, e1, cross(c, e1)};
}
Vec3 at(const Tangent& t, double x, double y) { return math::normalized(t.c + (t.e1 * x + t.e2 * y) / R); }

double unit_hash(std::uint64_t a, std::uint64_t b, std::uint64_t c = 0) {
    return environment::hash_unit(environment::surface_hash(0x7E57, a, b, c));
}

// Flat global grid (LDEM_16 layout) and one 401 x 401 tile (20 m) centred on the site: core 6 km, blend 1 km.
environment::PolarStereographicTile site_tile() {
    environment::PolarStereographicTile t;
    t.sphere_radius_m = R;
    t.scale_m = 20.0;
    t.pole_line = t.pole_sample = 15199.5;
    const auto [line, sample] = t.position(site_lat, site_lon);
    t.first_line = static_cast<int>(std::lround(line)) - 200;
    t.first_sample = static_cast<int>(std::lround(sample)) - 200;
    t.grid = {401, 401, 0.5, std::vector<std::int16_t>(401 * 401, 0)};
    return t;
}

// Direction of a tile position (inverse polar stereographic: X = rho sin lon, Y = -rho cos lon).
Vec3 tile_direction(const environment::PolarStereographicTile& t, double line, double sample) {
    const double x = (sample + t.first_sample - t.pole_sample) * t.scale_m;
    const double y = (line + t.first_line - t.pole_line) * t.scale_m;
    const double lat = 2.0 * std::atan(std::hypot(x, y) / (2.0 * R)) - 0.5 * constants::pi;
    return direction(lat, std::atan2(x, -y));
}

std::shared_ptr<const environment::LunarTerrain> make_terrain() {
    environment::CylindricalGrid g;
    g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
    g.pixels_per_degree = 16.0;
    g.zero_line = 1439.5;
    g.zero_sample = -0.5;
    return std::make_shared<const environment::LunarTerrain>(std::move(g), std::vector{site_tile()}, R);
}

const LunarSurfaceDetail& detail() {
    static const LunarSurfaceDetail d(
        config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml"),
        make_terrain());
    return d;
}

// Guo et al. 2021's statistic: standard deviation of the plane-detrended heights in a W x W window sampled 25 x 25.
template <typename F>
double window_rms(const Tangent& t, double window_m, F&& height) {
    constexpr int n = 25;
    const double half = 0.5 * (n - 1), step = window_m / n;
    std::array<double, n * n> z{};
    double mean = 0.0, sx = 0.0, sy = 0.0, xx = 0.0;
    for (int a = 0; a < n; ++a) {
        for (int b = 0; b < n; ++b) {
            const double v = height(at(t, (a - half) * step, (b - half) * step));
            z[a * n + b] = v;
            mean += v;
            sx += v * (a - half);
            sy += v * (b - half);
            xx += (a - half) * (a - half);
        }
    }
    mean /= n * n;
    const double gx = sx / xx, gy = sy / xx;
    double var = 0.0;
    for (int a = 0; a < n; ++a) {
        for (int b = 0; b < n; ++b) {
            const double r = z[a * n + b] - mean - gx * (a - half) - gy * (b - half);
            var += r * r;
        }
    }
    return std::sqrt(var / (n * n));
}

// Median window statistic over `count` windows spread over a patch of patch_m around `centre`.
template <typename F>
double median_window_rms(const Vec3& centre, double patch_m, double window_m, int count, F&& height) {
    const Tangent base = tangent(centre);
    std::vector<double> v;
    for (int k = 0; k < count; ++k) {
        const Vec3 c = at(base, (unit_hash(1, k) - 0.5) * patch_m, (unit_hash(2, k) - 0.5) * patch_m);
        v.push_back(window_rms(tangent(c), window_m, height));
    }
    std::nth_element(v.begin(), v.begin() + count / 2, v.end());
    return v[count / 2];
}

// Largest relief difference (band limit `limit_m`, 0: full detail) over 1 mm (or `step_m`) steps at `count` points
// around the given centres (all far from cube face edges).
double step_bound(const std::vector<Vec3>& centres, double step_m, int count, double limit_m = 0.0) {
    double bound = 0.0;
    for (int k = 0; k < count; ++k) {
        const Tangent t = tangent(centres[k % centres.size()]);
        const Vec3 p = at(t, (unit_hash(3, k) - 0.5) * 200.0, (unit_hash(4, k) - 0.5) * 200.0);
        const double a = constants::two_pi * unit_hash(5, k);
        const Tangent tp = tangent(p);
        const Vec3 q = at(tp, step_m * std::cos(a), step_m * std::sin(a));
        bound = std::max(bound, std::abs(detail().relief_m(p, limit_m) - detail().relief_m(q, limit_m)));
    }
    return bound;
}

const std::vector<Vec3>& bound_centres() {
    static const std::vector<Vec3> c = {direction(site_lat, site_lon), direction(global_lat, global_lon),
                                        direction(10.0 * deg, -40.0 * deg), direction(45.0 * deg, 170.0 * deg)};
    return c;
}

}  // namespace

TEST_CASE("Face cells follow the cube-sphere mapping and find their neighbours across edges",
          "[moon_surface][environment]") {
    using environment::FaceCell;
    // The face centre of +Z is cell (2^(L-1), 2^(L-1)) at level L (s, t = 0 at the lower-left corner of that cell).
    const FaceCell c = environment::face_cell_of(Vec3{0, 0, 1}, 4);
    CHECK(c == FaceCell{4, 4, 8, 8});
    // Cell centres map back to their cell, also next to edges and corners.
    for (int k = 0; k < 2000; ++k) {
        const FaceCell cell{static_cast<int>(unit_hash(10, k) * 6), 7, static_cast<std::int64_t>(unit_hash(11, k) * 128),
                            static_cast<std::int64_t>(unit_hash(12, k) * 128)};
        CHECK(environment::face_cell_of(environment::face_cell_point(cell, 0.5, 0.5), 7) == cell);
    }
    // Edge directions are bit-identical from both faces (cube_sphere.gd's seam rule).
    CHECK(environment::face_direction(0, 1.0, 0.3) == environment::face_direction(2, -1.0, 0.3));
    // A corner cell's 3 x 3 block reaches the two other faces at the corner; every block cell touches the centre cell.
    const FaceCell corner{0, 6, 63, 63};
    const auto block = environment::face_cell_neighbourhood(corner);
    std::vector<int> faces;
    for (const auto& b : block) faces.push_back(b.face);
    std::sort(faces.begin(), faces.end());
    faces.erase(std::unique(faces.begin(), faces.end()), faces.end());
    CHECK(faces.size() == 3);
    const double size = environment::face_cell_nominal_size_m(6, R);
    const Vec3 centre = environment::face_cell_point(corner, 0.5, 0.5);
    for (const auto& b : block) CHECK(R * norm(environment::face_cell_point(b, 0.5, 0.5) - centre) < 1.6 * size);
    // face_cells_within holds the cells of every point within the radius.
    for (int k = 0; k < 500; ++k) {
        const Vec3 p = environment::face_cell_point(corner, unit_hash(13, k) * 3.0 - 1.0, unit_hash(14, k) * 3.0 - 1.0);
        const auto cells = environment::face_cells_within(p, 6, 0.45 * size, R);
        const Tangent t = tangent(p);
        for (int m = 0; m < 8; ++m) {
            const double a = constants::two_pi * m / 8.0;
            const Vec3 q = at(t, 0.44 * size * std::cos(a), 0.44 * size * std::sin(a));
            CHECK(std::find(cells.begin(), cells.end(), environment::face_cell_of(q, 6)) != cells.end());
        }
        CHECK(cells.size() <= 12);  // 2 x 2 per face, three faces at a corner
    }
    CHECK(environment::face_cell_level_for_size(environment::face_cell_nominal_size_m(12, R), R) == 12);
    CHECK(environment::face_cell_level_for_size(0.99 * environment::face_cell_nominal_size_m(12, R), R) == 12);
    CHECK(environment::face_cell_level_for_size(1.01 * environment::face_cell_nominal_size_m(12, R), R) == 11);
}

TEST_CASE("Detail is deterministic across calls and threads", "[moon_surface][environment]") {
    const auto& d = detail();
    constexpr int count = 10000;
    std::vector<Vec3> points;
    for (int k = 0; k < count; ++k) {
        if (k % 2 == 0) {
            const double z = 2.0 * unit_hash(20, k) - 1.0, phi = constants::two_pi * unit_hash(21, k);
            points.push_back({std::sqrt(1.0 - z * z) * std::cos(phi), std::sqrt(1.0 - z * z) * std::sin(phi), z});
        } else {
            points.push_back(at(tangent(direction(site_lat, site_lon)), (unit_hash(22, k) - 0.5) * 5000.0,
                                (unit_hash(23, k) - 0.5) * 5000.0));
        }
    }
    std::vector<double> full(count), coarse(count);
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < count; ++k) full[k] = d.relief_m(points[k], 0.0);
    const auto t1 = std::chrono::steady_clock::now();
    for (int k = 0; k < count; ++k) coarse[k] = d.relief_m(points[k], 1.0);
    const auto t2 = std::chrono::steady_clock::now();
    const auto us = [](auto a, auto b) { return std::chrono::duration<double, std::micro>(b - a).count() / count; };
    std::printf("relief query: full detail %.2f us, 1 m square mean %.2f us (not gated)\n", us(t0, t1), us(t1, t2));
    {
        // Inside the face blend band (2-3 face lattices; footprint means over sheared parallelograms).
        double sink = 0.0;
        const auto b0 = std::chrono::steady_clock::now();
        for (int k = 0; k < count; ++k) {
            sink += d.relief_m(environment::face_direction(k % 6, 1.0 + 0.01 * (2.0 * unit_hash(24, k) - 1.0),
                                                           2.0 * unit_hash(25, k) - 1.0),
                               k % 2 == 0 ? 0.0 : 1.0);
        }
        const auto b1 = std::chrono::steady_clock::now();
        std::printf("relief query in the face blend band (half full, half 1 m): %.2f us (not gated)\n", us(b0, b1));
        CHECK(std::isfinite(sink));
    }
    std::vector<double> full2(count), coarse2(count);
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([&, t] {
            for (int k = t; k < count; k += 4) {
                full2[k] = d.relief_m(points[k], 0.0);
                coarse2[k] = d.relief_m(points[k], 1.0);
            }
        });
    }
    for (auto& th : threads) th.join();
    int mismatches = 0;
    for (int k = 0; k < count; ++k) {
        if (!(full[k] == full2[k] && coarse[k] == coarse2[k] && std::isfinite(full[k]))) ++mismatches;
    }
    CHECK(mismatches == 0);
    // A second instance draws the same surface.
    const auto c0 = std::chrono::steady_clock::now();
    const LunarSurfaceDetail other(d.config(), make_terrain());
    std::printf("construction %.0f ms (not gated)\n",
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count());
    CHECK(other.roughness_scale_m() == d.roughness_scale_m());
    for (int k = 0; k < 200; ++k) CHECK(other.relief_m(points[k], 0.0) == full[k]);
}

TEST_CASE("Crater counts per size octave match the configured SFD", "[moon_surface][environment]") {
    const auto& d = detail();
    const auto& sfd = d.config().craters.sfd;
    // On the global grid rather than the site patch: the 1-50 m population is the same in the tile core (only craters
    // above 5 x 20 m are left to LOLA there), and the 20 km disc of the 1-2 m bin would leave the 6 km tile core.
    const Vec3 centre = direction(global_lat, global_lon);
    const std::array<std::pair<double, double>, 6> bins = {
        {{1.0, 2.0}, {2.0, 4.0}, {4.0, 8.0}, {8.0, 16.0}, {16.0, 32.0}, {32.0, 50.0}}};
    for (const auto& [lo, hi] : bins) {
        const double per_m2 = sfd.cumulative_density_per_m2(lo) - sfd.cumulative_density_per_m2(hi);
        const double radius = std::sqrt(1500.0 / per_m2 / constants::pi);  // a spherical cap's area is pi chord^2
        const double expected = per_m2 * constants::pi * radius * radius;
        const auto craters = d.craters_within(centre, radius, lo, hi);
        const double n = static_cast<double>(craters.size());
        std::printf("craters %4.0f-%4.0f m: %6.0f counted, %6.0f expected (3 sigma %4.0f)\n", lo, hi, n, expected,
                    3.0 * std::sqrt(expected));
        CHECK(expected >= 1000.0);
        CHECK(std::abs(n - expected) <= 3.0 * std::sqrt(expected));
        int fresh = 0;
        for (const auto& c : craters) {
            fresh += c.fresh ? 1 : 0;
            const double dd = c.fresh ? d.config().craters.depth_to_diameter_fresh
                                      : d.config().craters.depth_to_diameter_degraded;
            CHECK(std::abs(c.depth_m - dd * c.diameter_m) <= 1e-12 * c.diameter_m);
        }
        const double fresh_expected = (1.0 - d.config().craters.degraded_fraction) * n;
        CHECK(std::abs(fresh - fresh_expected) <= 3.0 * std::sqrt(fresh_expected));
    }
}

TEST_CASE("Roughness continues the LOLA spectrum", "[moon_surface][environment]") {
    const auto& d = detail();
    const auto& r = d.config().roughness;
    const auto roughness = [&](const Vec3& p) { return d.roughness_relief_m(p, 0.0); };
    const auto total = [&](const Vec3& p) { return d.relief_m(p, 0.0); };
    // (a) The roughness alone has the configured residual at Guo's baseline, at the site (tile core).
    const Vec3 site = direction(site_lat, site_lon);
    const double at_site = median_window_rms(site, 4000.0, r.baseline_m, 1500, roughness);
    std::printf("roughness at %.3f m: %.3f mm (configured %.3f mm)\n", r.baseline_m, 1e3 * at_site,
                1e3 * r.rms_height_at_baseline_m);
    CHECK(std::abs(at_site / r.rms_height_at_baseline_m - 1.0) <= 0.15);
    // (b) Hurst scaling of the same statistic on the global grid (band start 1.9 km).
    const Vec3 global = direction(global_lat, global_lon);
    for (const double window : {0.1, 1.0, 10.0}) {
        const double measured = median_window_rms(global, 20000.0, window, 300, roughness);
        const double law = r.rms_height_at_baseline_m * std::pow(window / r.baseline_m, r.hurst_exponent);
        std::printf("roughness in %5.1f m windows: %.4f m, law %.4f m (ratio %.3f)\n", window, measured, law,
                    measured / law);
        CHECK(std::abs(measured / law - 1.0) <= 0.15);
    }
    // (c) Craters + roughness stay below the measured total (rocks follow in Task 3). 1500 windows: the median
    // is known to ~1 %; the expected margin without rocks is ~2 %.
    const double summed = median_window_rms(site, 4000.0, r.baseline_m, 1500, total);
    std::printf("craters + roughness at %.3f m: %.3f mm (measured total %.3f mm)\n", r.baseline_m, 1e3 * summed,
                1e3 * r.rms_height_measured_total_m);
    CHECK(summed <= r.rms_height_measured_total_m);
    CHECK(summed >= at_site);
}

// The footprint of relief(L) is the L x L (s, t) parallelogram of the face lattice; across a face edge (face_blend
// band) it is the blend-weighted set of the faces' parallelograms. Reference: the full detail averaged over that.
static double footprint_mean(const Vec3& p, double limit, int samples) {
    const auto blend = environment::face_blend(p, environment::face_blend_band);
    double mean = 0.0;
    for (int f = 0; f < blend.count; ++f) {
        const auto& fc = blend.coord[f];
        const auto frame = environment::face_frame(fc.face, fc.s, fc.t, R);
        const double ds = 0.5 * limit / frame.m_s, dt = 0.5 * limit / frame.m_t;
        double face_mean = 0.0;
        for (int a = 0; a < samples; ++a) {
            for (int b = 0; b < samples; ++b) {
                const double s = fc.s + ds * (2.0 * (a + 0.5) / samples - 1.0);
                const double t = fc.t + dt * (2.0 * (b + 0.5) / samples - 1.0);
                face_mean += detail().relief_m(environment::face_direction(fc.face, s, t), 0.0);
            }
        }
        mean += blend.weight[f] * face_mean / (samples * samples);
    }
    return mean;
}

TEST_CASE("Band-limited relief is the average of the finer relief", "[moon_surface][environment]") {
    const auto& d = detail();
    const Vec3 site = direction(site_lat, site_lon);
    constexpr int points = 60, samples = 24;
    // Site (tile core, near the -Z face centre), a face corner region outside the face blend (lattice axes up to
    // ~25 deg from orthogonal) and points within the blend band across face edges and at corners.
    const auto site_point = [&](int k) {
        return at(tangent(site), (unit_hash(30, k) - 0.5) * 4000.0, (unit_hash(31, k) - 0.5) * 4000.0);
    };
    const auto corner_point = [&](int k) {
        return environment::face_direction(static_cast<int>(unit_hash(32, k) * 6), 0.9 + 0.08 * unit_hash(33, k),
                                           -0.9 - 0.08 * unit_hash(34, k));
    };
    const auto blend_point = [&](int k) {
        const double across = 1.0 + environment::face_blend_band * (2.0 * unit_hash(35, k) - 1.0);
        const double along = k % 2 == 0 ? 2.0 * unit_hash(36, k) - 1.0 : across;  // odd k: at a corner
        return environment::face_direction(static_cast<int>(unit_hash(37, k) * 6), across, along);
    };
    struct Region {
        const char* name;
        std::function<Vec3(int)> point;
    };
    for (const Region& region : {Region{"site", site_point}, Region{"face corner", corner_point},
                                 Region{"edge blend", blend_point}}) {
        for (const double limit : {10.0, 1.0, 0.1}) {
            std::vector<double> removed, diff;
            for (int k = 0; k < points; ++k) {
                const Vec3 p = region.point(k);
                const double mean = footprint_mean(p, limit, samples);
                const double coarse = d.relief_m(p, limit);
                removed.push_back(d.relief_m(p, 0.0) - coarse);
                diff.push_back(coarse - mean);
            }
            double rms_removed = 0.0, rms_diff = 0.0, max_diff = 0.0;
            for (int k = 0; k < points; ++k) {
                rms_removed += removed[k] * removed[k];
                rms_diff += diff[k] * diff[k];
                max_diff = std::max(max_diff, std::abs(diff[k]));
            }
            rms_removed = std::sqrt(rms_removed / points);
            rms_diff = std::sqrt(rms_diff / points);
            std::printf("%-11s limit %5.1f m: removed band RMS %.5f m, |coarse - footprint mean| RMS %.5f m, max %.5f m "
                        "(%.3f)\n",
                        region.name, limit, rms_removed, rms_diff, max_diff, max_diff / rms_removed);
            CHECK(max_diff <= 0.1 * rms_removed);
        }
    }
}

TEST_CASE("No seam at cube face edges and corners", "[moon_surface][environment]") {
    const auto& d = detail();
    const double bound = step_bound(bound_centres(), 1e-3, 20000);
    std::printf("full-detail 1 mm step bound: %.6f m\n", bound);
    CHECK(bound > 0.0);
    int crossings = 0;
    for (int k = 0; k < 600; ++k) {
        // A point on a random face edge, the pair 1 mm apart across it, and the pair 1 um apart. The 1 mm bound (max
        // slope seen off the edges) is loose; the 1 um pairs (bound / 1000) are the real continuity gate.
        const int face = static_cast<int>(unit_hash(40, k) * 6);
        const int side = static_cast<int>(unit_hash(41, k) * 4);
        const double along = 2.0 * unit_hash(42, k) - 1.0;
        const double s = side == 0 ? 1.0 : side == 1 ? -1.0 : along;
        const double t = side == 2 ? 1.0 : side == 3 ? -1.0 : along;
        const Vec3 e = environment::face_direction(face, s, t);
        const auto frame = environment::face_frame(face, s, t, R);
        const Vec3 across = side < 2 ? frame.e_s : frame.e_t;
        for (const double gap : {1e-3, 1e-6}) {
            const Vec3 a = math::normalized(e - across * (0.5 * gap / R));
            const Vec3 b = math::normalized(e + across * (0.5 * gap / R));
            crossings += environment::face_coord_of(a).face != environment::face_coord_of(b).face ? 1 : 0;
            CHECK(std::abs(d.relief_m(a, 0.0) - d.relief_m(b, 0.0)) <= bound * gap / 1e-3);
        }
    }
    CHECK(crossings == 1200);
    // Around the eight cube corners: pairs 1 mm apart through the corner.
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3 c = math::normalized(Vec3{corner & 1 ? 1.0 : -1.0, corner & 2 ? 1.0 : -1.0, corner & 4 ? 1.0 : -1.0});
        const Tangent t = tangent(c);
        for (int k = 0; k < 60; ++k) {
            const double a = constants::two_pi * unit_hash(43, corner * 100 + k);
            const Vec3 p = at(t, 0.5e-3 * std::cos(a), 0.5e-3 * std::sin(a));
            const Vec3 q = at(t, -0.5e-3 * std::cos(a), -0.5e-3 * std::sin(a));
            CHECK(std::abs(d.relief_m(p, 0.0) - d.relief_m(q, 0.0)) <= bound);
        }
        CHECK(std::isfinite(d.relief_m(c, 0.0)));
        CHECK(std::isfinite(d.relief_m(c, 1.0)));
    }
}

// Terrain nodes sample band-limited relief at both sides of a face edge: relief(L) must be continuous there too. Same
// structure as the full-detail case: the 1 um pairs (bound / 1000) prove continuity, the 1 mm pairs the slope.
TEST_CASE("No seam at face edges and corners for band-limited relief", "[moon_surface][environment]") {
    const auto& d = detail();
    for (const double limit : {0.1, 1.0, 10.0}) {
        const double bound = step_bound(bound_centres(), 1e-3, 4000, limit);
        double worst_edge = 0.0, worst_corner = 0.0;  // largest |difference| / allowed, 1 um pairs
        for (int k = 0; k < 300; ++k) {
            const int face = static_cast<int>(unit_hash(44, k) * 6);
            const int side = static_cast<int>(unit_hash(45, k) * 4);
            const double along = 2.0 * unit_hash(46, k) - 1.0;
            const double s = side == 0 ? 1.0 : side == 1 ? -1.0 : along;
            const double t = side == 2 ? 1.0 : side == 3 ? -1.0 : along;
            const Vec3 e = environment::face_direction(face, s, t);
            const auto frame = environment::face_frame(face, s, t, R);
            const Vec3 across = side < 2 ? frame.e_s : frame.e_t;
            for (const double gap : {1e-3, 1e-6}) {
                const Vec3 a = math::normalized(e - across * (0.5 * gap / R));
                const Vec3 b = math::normalized(e + across * (0.5 * gap / R));
                const double diff = std::abs(d.relief_m(a, limit) - d.relief_m(b, limit));
                CHECK(diff <= bound * gap / 1e-3);
                if (gap < 1e-4) worst_edge = std::max(worst_edge, diff / (bound * gap / 1e-3));
            }
        }
        for (int corner = 0; corner < 8; ++corner) {
            const Vec3 c =
                math::normalized(Vec3{corner & 1 ? 1.0 : -1.0, corner & 2 ? 1.0 : -1.0, corner & 4 ? 1.0 : -1.0});
            const Tangent t = tangent(c);
            for (int k = 0; k < 30; ++k) {
                const double a = constants::two_pi * unit_hash(47, corner * 100 + k);
                for (const double gap : {1e-3, 1e-6}) {
                    const Vec3 p = at(t, 0.5 * gap * std::cos(a), 0.5 * gap * std::sin(a));
                    const Vec3 q = at(t, -0.5 * gap * std::cos(a), -0.5 * gap * std::sin(a));
                    const double diff = std::abs(d.relief_m(p, limit) - d.relief_m(q, limit));
                    CHECK(diff <= bound * gap / 1e-3);
                    if (gap < 1e-4) worst_corner = std::max(worst_corner, diff / (bound * gap / 1e-3));
                }
            }
        }
        std::printf("limit %5.1f m: 1 mm step bound %.3e m, worst 1 um pair / allowed: edges %.3f, corners %.3f\n",
                    limit, bound, worst_edge, worst_corner);
    }
}

TEST_CASE("The procedural band follows the tile blend", "[moon_surface][environment]") {
    const auto& d = detail();
    const Vec3 site = direction(site_lat, site_lon);
    CHECK(d.band_start_m(site) == 20.0);
    CHECK(d.band_start_m(direction(global_lat, global_lon)) == d.lola().global_resolution_m());
    // Walk along +sample from the tile core (edge distance 60 px) across the 50 px blend to the global grid.
    const auto tile = site_tile();
    const double max_gradient =
        1.01 * 1.5 * (d.lola().global_resolution_m() - 20.0) / environment::LunarTerrain::blend_m;  // smoothstep slope
    constexpr double step_px = 0.0325;  // 0.65 m
    double previous = 0.0, first = 0.0, last = 0.0;
    for (int k = 0; k <= 2200; ++k) {
        const double band = d.band_start_m(tile_direction(tile, 200.0, 340.0 + k * step_px));
        if (k == 0) first = previous = band;
        CHECK(std::abs(band - previous) <= max_gradient * step_px * 20.0 + 1e-9);
        previous = band;
        last = band;
    }
    CHECK(first == 20.0);
    CHECK(last == d.lola().global_resolution_m());
    // Relief: no step larger than the 1 cm step bound measured off the blend.
    const double bound = step_bound(bound_centres(), 1e-2, 20000);
    std::printf("full-detail 1 cm step bound: %.6f m\n", bound);
    for (int k = 0; k < 400; ++k) {
        const double sample = 340.0 + k * 0.1625;
        CHECK(std::abs(d.relief_m(tile_direction(tile, 200.0, sample), 0.0) -
                       d.relief_m(tile_direction(tile, 200.0, sample + 0.0005), 0.0)) <= bound);
    }
}

TEST_CASE("Poles are regular", "[moon_surface][environment]") {
    const auto& d = detail();
    const double bound = step_bound(bound_centres(), 1e-3, 5000);
    for (const double lat : {-90.0, -89.9999, 90.0}) {
        const Vec3 p = direction(lat * deg, 0.3);
        for (const double limit : {0.0, 0.1, 1.0, 10.0}) CHECK(std::isfinite(d.relief_m(p, limit)));
        CHECK(std::isfinite(d.band_start_m(p)));
        const Tangent t = tangent(p);
        for (int k = 0; k < 40; ++k) {
            const double a = constants::two_pi * k / 40.0;
            const Vec3 q = at(t, 1e-3 * std::cos(a), 1e-3 * std::sin(a));
            CHECK(std::abs(d.relief_m(q, 0.0) - d.relief_m(p, 0.0)) <= bound);
        }
    }
    // The exact pole and the pole from latitude / longitude (cos(-pi/2) = 6e-17: 1e-10 m apart) agree.
    CHECK(std::abs(d.relief_m(Vec3{0, 0, -1}, 0.0) - d.relief_m(direction(-90.0 * deg, 0.0), 0.0)) <= 1e-9);
}

TEST_CASE("No band below the LOLA resolution is counted twice", "[moon_surface][environment]") {
    const auto& d = detail();
    // Tile core (band start 20 m): no crater above 5 x 20 m, but craters just below it.
    const Vec3 site = direction(site_lat, site_lon);
    const auto big = d.craters_within(site, 2500.0, 100.0 + 1e-9, 1e9);
    CHECK(big.empty());
    const auto below = d.craters_within(site, 2500.0, 50.0, 100.0);
    CHECK(below.size() > 50);
    for (const auto& c : d.craters_within(site, 5000.0, 50.0, 1e9)) {
        CHECK(c.diameter_m <= 5.0 * d.band_start_m(c.centre_unit));
    }
    // Global grid (1.9 km): the large craters exist up to the configured maximum.
    const Vec3 global = direction(global_lat, global_lon);
    const auto large = d.craters_within(global, 20000.0, 100.0, 1e9);
    const auto& sfd = d.config().craters.sfd;
    const double expected = (sfd.cumulative_density_per_m2(100.0) - sfd.cumulative_density_per_m2(sfd.max_diameter_m)) *
                            constants::pi * 20000.0 * 20000.0;
    std::printf("craters > 100 m within 20 km on the global grid: %zu (expected %.0f)\n", large.size(), expected);
    CHECK(std::abs(static_cast<double>(large.size()) - expected) <= 3.0 * std::sqrt(expected));
    // Roughness: a 640 m square mean keeps only octaves coarser than 40 m; in the tile core (band start 20 m) none
    // of them is generated, on the global grid they are.
    CHECK(d.roughness_relief_m(site, 640.0) == 0.0);
    CHECK(d.roughness_relief_m(global, 640.0) != 0.0);
}

// LOLA heights are area means that already contain the craters: the procedural layer must add relief, not lower the
// mean surface. Patch mean = mean of relief(L) over a grid of L x L squares tiling the patch (relief(L) is the
// footprint mean of the full detail); tolerance 4 standard errors of that mean (squares treated as independent).
TEST_CASE("The procedural relief keeps the LOLA mean height", "[moon_surface][environment]") {
    const auto& d = detail();
    const auto patch_mean = [&](const Vec3& centre, double patch_m, double limit_m, const char* name) {
        const Tangent t = tangent(centre);
        const int n = static_cast<int>(std::lround(patch_m / limit_m));
        double sum = 0.0, sum2 = 0.0;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                const double v = d.relief_m(at(t, (i + 0.5) * limit_m - 0.5 * patch_m, (j + 0.5) * limit_m - 0.5 * patch_m),
                                            limit_m);
                sum += v;
                sum2 += v * v;
            }
        }
        const double count = static_cast<double>(n) * n;
        const double mean = sum / count;
        const double se = std::sqrt(std::max(0.0, sum2 / count - mean * mean) / (count - 1.0));
        double population = 0.0;  // the crater population's mean relief (what a non-zero-mean layer would add)
        for (const auto& o : d.crater_octaves()) {
            if (o.max_diameter_m <= 5.0 * d.band_start_m(centre)) population += o.mean_relief_m;
        }
        std::printf("%s: %d x %d squares of %.0f m: mean relief %+.4f m, standard error %.4f m (crater population "
                    "mean %+.4f m)\n",
                    name, n, n, limit_m, mean, se, population);
        CHECK(std::abs(mean) <= 4.0 * se);
    };
    patch_mean(direction(site_lat, site_lon), 5000.0, 250.0, "site tile core");
    patch_mean(direction(global_lat, global_lon), 200000.0, 5000.0, "global grid");
    // Far above every generated band nothing is left (site: a 2 km square stays inside the 6 km tile core, where
    // nothing above 100 m and no roughness above 20 m is generated).
    CHECK(d.relief_m(direction(site_lat, site_lon), 2000.0) == 0.0);
    CHECK(d.relief_m(direction(global_lat, global_lon), 1e5) == 0.0);
}
