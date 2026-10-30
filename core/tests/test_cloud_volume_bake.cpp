// The cloud world volume's cell bake, CPU reference (world-volume plan Task 4): the point model mirror
// (cloud_render_field.hpp) and the per-cell bake with its tap rule (cloud_volume_bake.hpp).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <thread>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/cloud_noise.hpp"
#include "starsim/environment/cloud_optics.hpp"
#include "starsim/environment/cloud_render_field.hpp"
#include "starsim/environment/cloud_render_texels.hpp"
#include "starsim/environment/cloud_volume_bake.hpp"
#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::environment;
using Catch::Approx;

namespace {

// The former world volume's levels (its layout's CELL_H_M / CELL_V_M, SHELL_BOTTOM_M; removed 2026-10-30).
constexpr int levels = 8;
constexpr std::array<double, levels> cell_h_m = {50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0};
constexpr std::array<double, levels> cell_v_m = {25.0, 50.0, 164.0625, 164.0625, 164.0625, 164.0625, 164.0625,
                                                 164.0625};
constexpr double shell_bottom_m = -500.0;

// The broken cumulus slab of the per-cell checks: cf 0.4, 1-2.5 km, 2e-4 kg/m^3 grid mean.
constexpr double slab_bottom_m = 1000.0, slab_top_m = 2500.0, slab_water_kg_m3 = 2e-4;

const CloudNoise& noise() {
    static const CloudNoise n = generate_cloud_noise();
    return n;
}

const CloudRenderField& broken_slab() {
    static const CloudRenderField f = make_cloud_render_field(
        WeatherSnapshot::uniform_slab(19, 36, slab_bottom_m, slab_top_m, slab_water_kg_m3, 0.4));
    return f;
}

// Earth-local render axes (ITRF (x, y, z) -> (y, z, x)) of a geodetic position.
math::Vec3 earth_of(double latitude_deg, double longitude_deg, double height_m) {
    const double deg = constants::pi / 180.0;
    const math::Vec3 f = frames::geodetic_to_fixed({latitude_deg * deg, longitude_deg * deg, height_m});
    return {f.y, f.z, f.x};
}

// The point over (latitude, longitude) at the shader's model height height_m (moved along the geodetic normal).
math::Vec3 at_model_height(double latitude_deg, double longitude_deg, double height_m) {
    math::Vec3 p = earth_of(latitude_deg, longitude_deg, height_m);
    const math::Vec3 up = cloud_cell_box(p, 1.0, 1.0).edge_z_m;
    for (int i = 0; i < 3; ++i) p = p + up * (height_m - cloud_model_height_m(p));
    return p;
}

const CloudShapeFrame& frame() {
    static const CloudShapeFrame f = make_cloud_shape_frame(earth_of(26.0, -97.2, 0.0));
    return f;
}

// fn(i) for i in [0, n) on all cores.
void parallel_for(int n, const std::function<void(int)>& fn) {
    const int threads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; ++t) {
        pool.emplace_back([&, t] {
            for (int i = t; i < n; i += threads) fn(i);
        });
    }
    for (auto& th : pool) th.join();
}

// Stratified mean of the point model at a 10 m footprint over the box: one jittered point in each of n_h x n_h x n_v
// sub-boxes; extinction and in-cloud extinction.
CloudCellBake stratified_mean(const CloudRenderField& field, const CloudCellBox& box, int n_h, int n_v,
                              std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(0.0, 1.0);
    CloudCellBake sum;
    for (int k = 0; k < n_v; ++k) {
        for (int j = 0; j < n_h; ++j) {
            for (int i = 0; i < n_h; ++i) {
                const double u = (i + jitter(rng)) / n_h - 0.5, v = (j + jitter(rng)) / n_h - 0.5,
                             w = (k + jitter(rng)) / n_v - 0.5;
                const math::Vec3 p = box.centre_earth_m + box.edge_x_m * u + box.edge_y_m * v + box.edge_z_m * w;
                const CloudPointExtinction e = cloud_point_extinction_at_level(
                    field, noise(), frame(), p, cloud_shape_prefilter_level(10.0, noise().shape_size));
                sum.mean_extinction_per_m += e.extinction_per_m;
                sum.in_cloud_extinction_per_m += e.in_cloud_extinction_per_m;
            }
        }
    }
    const double n = static_cast<double>(n_h) * n_h * n_v;
    return {sum.mean_extinction_per_m / n, sum.in_cloud_extinction_per_m / n};
}

// The box of level k's cell with the horizontal centre (latitude, longitude) and the vertical index iz.
CloudCellBox level_cell(int k, double latitude_deg, double longitude_deg, int iz) {
    const double h = shell_bottom_m + (iz + 0.5) * cell_v_m[static_cast<std::size_t>(k)];
    return cloud_cell_box(at_model_height(latitude_deg, longitude_deg, h), cell_h_m[static_cast<std::size_t>(k)],
                          cell_v_m[static_cast<std::size_t>(k)]);
}

}  // namespace

TEST_CASE("The point model mirror: the slab's extinction, none above it, none under the flat base", "[weather]") {
    const double water = 3e-4;
    const CloudRenderField slab =
        make_cloud_render_field(WeatherSnapshot::uniform_slab(19, 36, slab_bottom_m, slab_top_m, water, 1.0));
    const double beta = cloud_extinction_per_m(water, 0.0);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> lat(-60.0, 60.0), lon(-180.0, 180.0);
    // Between the slab's first and last layer centres the field is flat (1125 .. 2375 m).
    std::uniform_real_distribution<double> inside(1150.0, 2350.0);
    // Every point's envelope (the grid mean the field interpolates: in-cloud x m(1)) is the slab's extinction within
    // the 8-bit water code (<= 1.97 %); the points' mean extinction too (the edge ramp thins the cloud around the
    // shape's minima even at cf 1, the edge mean factor m(1) puts that water back: kept on average, not at every
    // point).
    const double m1 = cloud_edge_mean_factor(noise(), 0.0, 255);
    const int points = 4000;
    double worst = 0.0, mean = 0.0;
    for (int i = 0; i < points; ++i) {
        const math::Vec3 p = at_model_height(lat(rng), lon(rng), inside(rng));
        const CloudPointExtinction e = cloud_point_extinction_at_level(slab, noise(), frame(), p, 0.0);
        CHECK(e.extinction_per_m == cloud_point_extinction_per_m(slab, noise(), frame(), p, 10.0));
        worst = std::max(worst, std::abs(e.in_cloud_extinction_per_m * m1 / beta - 1.0));
        mean += e.extinction_per_m / points;
        CHECK(cloud_point_extinction_per_m(slab, noise(), frame(), at_model_height(lat(rng), lon(rng), 3000.0),
                                           10.0) == 0.0);
    }
    std::printf("cf 1 slab: envelope / slab extinction within %.4f, mean point extinction / slab %.4f (m(1) %.4f)\n",
                worst, mean / beta, m1);
    CHECK(worst <= 0.02);
    CHECK(mean / beta == Approx(1.0).epsilon(0.02));

    // A snapshot built from fields (the LCL raise and the flat base): nothing under the base, cloud above it.
    const WeatherSnapshot built = WeatherSnapshot::gfs_test_column(19, 36, 2000.0, 3e-4, 70.0, 295.0, 288.0);
    const CloudRenderField field = make_cloud_render_field(built);
    const double base_m = field.base[2 * (9 * 36 + 10)];
    REQUIRE(field.base[2 * (9 * 36 + 10) + 1] > 0.0f);
    int cloudy = 0;
    for (int i = 0; i < 500; ++i) {
        const double la = lat(rng), lo = lon(rng);
        CHECK(cloud_point_extinction_per_m(field, noise(), frame(), at_model_height(la, lo, base_m - 1.0), 10.0) ==
              0.0);
        CHECK(cloud_point_extinction_per_m(field, noise(), frame(), at_model_height(la, lo, base_m - 200.0),
                                           10.0) == 0.0);
        cloudy += cloud_point_extinction_per_m(field, noise(), frame(), at_model_height(la, lo, base_m + 100.0),
                                               10.0) > 0.0;
    }
    std::printf("built column: base %.1f m, %d of 500 points 100 m above it in cloud\n", base_m, cloudy);
    CHECK(cloudy > 100);
}

namespace {

// The dense-mean check (tiered tap rule) of one field: 200 cells per level, centres in [bottom_m - 100, top_m + 100],
// against the 16^3 stratified mean of the point model at a 10 m footprint, normalised by the cell's sigma_c (the
// larger of the bake's and the dense mean's in-cloud extinction: a cell at the slab's edge). Dense tier (levels 0-2):
// every cell within 0.02 sigma_c [E: a few taps across 20-40 m edges]. Coarse tier (levels 3-7, ruling T4 Q2): rms
// within rms_bound, every cell within 0.25 sigma_c. Every cell 0 <= rho <= sigma_c.
void check_dense_mean(const char* name, const CloudRenderField& field, double bottom_m, double top_m,
                      const std::array<double, levels>& rms_bound) {
    const int cells = 200;
    for (int k = 0; k < levels; ++k) {
        std::vector<double> error(cells, 0.0);
        std::vector<int> ordered(cells, 1);
        parallel_for(cells, [&](int c) {
            std::mt19937 rng(1000u * k + c);
            std::uniform_real_distribution<double> lat(25.0, 27.0), lon(-98.0, -96.0), h(bottom_m - 100.0,
                                                                                        top_m + 100.0);
            const CloudCellBox box =
                cloud_cell_box(at_model_height(lat(rng), lon(rng), h(rng)), cell_h_m[static_cast<std::size_t>(k)],
                               cell_v_m[static_cast<std::size_t>(k)]);
            const CloudCellBake bake = bake_cloud_cell(field, noise(), frame(), box);
            const CloudCellBake dense = stratified_mean(field, box, 16, 16, 77u + c);
            const double sigma = std::max(bake.in_cloud_extinction_per_m, dense.in_cloud_extinction_per_m);
            error[static_cast<std::size_t>(c)] =
                sigma > 0.0 ? (bake.mean_extinction_per_m - dense.mean_extinction_per_m) / sigma : 0.0;
            ordered[static_cast<std::size_t>(c)] =
                bake.mean_extinction_per_m >= 0.0 && bake.mean_extinction_per_m <= bake.in_cloud_extinction_per_m;
        });
        double worst = 0.0, rms = 0.0;
        for (double e : error) {
            worst = std::max(worst, std::abs(e));
            rms += e * e;
        }
        rms = std::sqrt(rms / cells);
        const CloudBakeTaps taps =
            cloud_cell_bake_taps(cell_h_m[static_cast<std::size_t>(k)], cell_v_m[static_cast<std::size_t>(k)]);
        std::printf("%s level %d (%d x %d x %d taps, lambda %.2f): |rho - dense| / sigma_c max %.4f rms %.4f\n", name,
                    k, taps.horizontal, taps.horizontal, taps.vertical, taps.prefilter_level, worst, rms);
        CHECK(std::count(ordered.begin(), ordered.end(), 1) == cells);
        if (cell_h_m[static_cast<std::size_t>(k)] <= cloud_bake_dense_cell_max_m) {
            CHECK(worst <= 0.02);
        } else {
            CHECK(rms <= rms_bound[static_cast<std::size_t>(k)]);
            CHECK(worst <= 0.25);
        }
    }
}

// An ice slab (cirrus: ice share 1, the 2 km ice tile's 15.6 m vertical texel): the uniform slab's water moved from
// the liquid to the ice code.
WeatherSnapshot ice_slab(double bottom_m, double top_m, double ice_kg_m3, double cloud_fraction) {
    const WeatherSnapshot liquid = WeatherSnapshot::uniform_slab(19, 36, bottom_m, top_m, ice_kg_m3, cloud_fraction);
    std::vector<std::uint8_t> texels = liquid.texels();
    for (std::size_t i = 0; i < texels.size(); i += 4) {
        texels[i + 2] = texels[i + 1];
        texels[i + 1] = 0;
    }
    return WeatherSnapshot::from_parts(liquid.id(), liquid.rows(), liquid.columns(), liquid.ground_heights_m(),
                                       std::move(texels));
}

// The cirrus slab of the checks: cf 0.4, 8-10 km, 5e-5 kg/m^3 ice.
constexpr double ice_bottom_m = 8000.0, ice_top_m = 10000.0;

const CloudRenderField& cirrus_slab() {
    static const CloudRenderField f = make_cloud_render_field(ice_slab(ice_bottom_m, ice_top_m, 5e-5, 0.4));
    return f;
}

}  // namespace

TEST_CASE("A cell bake matches the point model's dense cell mean (tiered tap rule)", "[weather]") {
    // Coarse-tier rms bounds: 1.5 x the measured rms (ruling T4 Q2).
    check_dense_mean("cumulus", broken_slab(), slab_bottom_m, slab_top_m,
                     {0.0, 0.0, 0.0, 1.5 * 0.0037, 1.5 * 0.0076, 1.5 * 0.0039, 1.5 * 0.0171, 1.5 * 0.0090});
}

TEST_CASE("A cirrus cell bake matches the point model's dense cell mean", "[weather]") {
    // Ice-dominated cloud (the cirrus type alone): its shape tile is 2 km high, a 15.6 m vertical texel against the
    // bake's 5 m vertical taps. Coarse-tier rms bounds: 1.5 x the measured rms.
    double latitude = 0.0, longitude = 0.0;
    const math::Vec3 p = at_model_height(26.0, -97.0, 9000.0);
    cloud_latitude_longitude(p, latitude, longitude);
    REQUIRE(cloud_render_field_at(cirrus_slab(), latitude, longitude, 9000.0).ice_share == Approx(1.0));
    check_dense_mean("cirrus", cirrus_slab(), ice_bottom_m, ice_top_m,
                     {0.0, 0.0, 0.0, 1.5 * 0.0019, 1.5 * 0.0036, 1.5 * 0.0021, 1.5 * 0.0101, 1.5 * 0.0069});
}

TEST_CASE("Each weather cell keeps the point model's water in the bake at every level", "[weather][slow]") {
    // Ruling T4 Q1: per 0.25 deg weather cell, the sum of the level's cell bakes over the cells whose centre lies in it
    // / the dense point-model integral over the same cells (jittered strata 100 m across, 32 up: unbiased) in
    // 0.996-1.004 (the ruling's 0.996-1.000 made symmetric: the dense integral is an unbiased estimate and the bake a
    // midpoint rule, so the ratio falls either side of 1). The point model's own grid-mean conservation is
    // statistical over the shape tile (3-7 % off per weather cell) and stays as it is.
    const double lat0 = 26.0, lon0 = -97.0, box_deg = 0.25;
    const double deg = 180.0 / constants::pi;
    struct Case {
        const char* name;
        CloudRenderField field;
        double bottom_m, top_m;
    };
    const std::array<Case, 3> cases = {
        Case{"cumulus cf 0.40", broken_slab(), slab_bottom_m, slab_top_m},
        Case{"deck cf 0.95",
             make_cloud_render_field(
                 WeatherSnapshot::uniform_slab(19, 36, slab_bottom_m, slab_top_m, slab_water_kg_m3, 0.95)),
             slab_bottom_m, slab_top_m},
        Case{"cirrus cf 0.40", cirrus_slab(), ice_bottom_m, ice_top_m}};
    for (const Case& test : cases) {
        const CloudRenderField& field = test.field;
        for (int k = 2; k < levels; ++k) {
            const double h = cell_h_m[static_cast<std::size_t>(k)], v = cell_v_m[static_cast<std::size_t>(k)];
            const math::Vec3 centre = at_model_height(lat0 + box_deg / 2, lon0 + box_deg / 2, 0.0);
            const CloudCellBox axes = cloud_cell_box(centre, 1.0, 1.0);
            const int half = static_cast<int>(std::ceil(box_deg / deg * 6.4e6 / h / 2.0)) + 2;
            const int z0 = static_cast<int>(std::floor((test.bottom_m - 300.0 - shell_bottom_m) / v));
            const int z1 = static_cast<int>(std::ceil((test.top_m + 300.0 - shell_bottom_m) / v));
            const int dense_h = std::max(2, static_cast<int>(std::ceil(h / 100.0)));
            std::vector<double> baked(static_cast<std::size_t>(2 * half), 0.0), dense(baked.size(), 0.0);
            parallel_for(2 * half, [&](int jy) {
                for (int ix = 0; ix < 2 * half; ++ix) {
                    const math::Vec3 ground = centre + axes.edge_x_m * ((ix - half + 0.5) * h) +
                                              axes.edge_y_m * ((jy - half + 0.5) * h);
                    double la = 0.0, lo = 0.0;
                    cloud_latitude_longitude(ground, la, lo);
                    if (la * deg < lat0 || la * deg >= lat0 + box_deg || lo * deg < lon0 ||
                        lo * deg >= lon0 + box_deg) {
                        continue;  // assigned by centre
                    }
                    for (int iz = z0; iz < z1; ++iz) {
                        const CloudCellBox box = level_cell(k, la * deg, lo * deg, iz);
                        baked[static_cast<std::size_t>(jy)] +=
                            bake_cloud_cell(field, noise(), frame(), box).mean_extinction_per_m;
                        dense[static_cast<std::size_t>(jy)] +=
                            stratified_mean(field, box, dense_h, 32,
                                            static_cast<std::uint32_t>(k) * 2654435761u ^
                                                static_cast<std::uint32_t>((jy * 4096 + ix) * 64 + iz))
                                .mean_extinction_per_m;
                    }
                }
            });
            double sum_baked = 0.0, sum_dense = 0.0;
            for (std::size_t i = 0; i < baked.size(); ++i) {
                sum_baked += baked[i];
                sum_dense += dense[i];
            }
            const double ratio = sum_baked / sum_dense;
            std::printf("%s level %d: weather cell bake / dense point-model water %.5f\n", test.name, k, ratio);
            CHECK(ratio >= 0.996);
            CHECK(ratio <= 1.004);
        }
    }
}

TEST_CASE("A coarser cell's bake is the volume-weighted mean of the finer cells it covers", "[weather]") {
    // Ruling C20: parent rho against the volume-weighted mean of the overlapping finer cells (levels 0 -> 1: 2 x 2 x 2
    // children; 1 -> 2: 2 x 2 across, the parts of the 50 m cells inside the 164 m parent weighted by their overlap;
    // 2 -> 3 .. 6 -> 7: 2 x 2 across, same vertical cell). Dense parents (levels 1-2) within 0.02 sigma_c [E: the
    // dense tier's own per-cell accuracy - both sides are sampled bakes, so the brief's "1 %" is read in sigma_c and
    // measured 0.0135 at 0 -> 1: no pop in the fade band beyond the bake's accuracy]; coarse parents (levels 3-7)
    // within their per-cell bound (0.25 sigma_c; measured <= 0.051).
    const int parents = 100;
    for (int k = 1; k < levels; ++k) {
        const double hp = cell_h_m[static_cast<std::size_t>(k)], vp = cell_v_m[static_cast<std::size_t>(k)];
        const double vc = cell_v_m[static_cast<std::size_t>(k - 1)];
        std::vector<double> error(parents, 0.0);
        parallel_for(parents, [&](int c) {
            std::mt19937 rng(5000u * k + c);
            std::uniform_real_distribution<double> lat(25.0, 27.0), lon(-98.0, -96.0);
            std::uniform_int_distribution<int> iz_of(static_cast<int>((slab_bottom_m - 200.0 - shell_bottom_m) / vp),
                                                     static_cast<int>((slab_top_m + 200.0 - shell_bottom_m) / vp));
            const int iz = iz_of(rng);
            const CloudCellBox parent = level_cell(k, lat(rng), lon(rng), iz);
            const CloudCellBake p = bake_cloud_cell(broken_slab(), noise(), frame(), parent);
            const math::Vec3 up = parent.edge_z_m / vp;
            const double bottom = shell_bottom_m + iz * vp;
            double sum = 0.0, weight = 0.0;
            for (int child_z = static_cast<int>(std::floor((bottom - shell_bottom_m) / vc));
                 shell_bottom_m + child_z * vc < bottom + vp; ++child_z) {
                const double lo = std::max(bottom, shell_bottom_m + child_z * vc);
                const double hi = std::min(bottom + vp, shell_bottom_m + (child_z + 1) * vc);
                if (!(hi > lo)) continue;
                // The child's part inside the parent (1 -> 2: a 50 m cell overhanging the 164 m parent is clipped - the
                // layout's misalignment is not the bake's), baked with the child level's taps.
                const double dz = 0.5 * (lo + hi) - (bottom + 0.5 * vp);
                const CloudBakeTaps child_taps = cloud_cell_bake_taps(0.5 * hp, vc, noise().shape_size);
                for (int j = 0; j < 2; ++j) {
                    for (int i = 0; i < 2; ++i) {
                        const CloudCellBox child = {
                            parent.centre_earth_m + parent.edge_x_m * ((i - 0.5) * 0.5) +
                                parent.edge_y_m * ((j - 0.5) * 0.5) + up * dz,
                            parent.edge_x_m * 0.5, parent.edge_y_m * 0.5, up * (hi - lo)};
                        const CloudCellBake b = bake_cloud_cell(broken_slab(), noise(), frame(), child, child_taps);
                        sum += (hi - lo) * b.mean_extinction_per_m;
                        weight += hi - lo;
                    }
                }
            }
            const double sigma = p.in_cloud_extinction_per_m;
            error[static_cast<std::size_t>(c)] = sigma > 0.0 ? (p.mean_extinction_per_m - sum / weight) / sigma : sum;
        });
        double worst = 0.0;
        for (double e : error) worst = std::max(worst, std::abs(e));
        std::printf("level %d -> %d: |parent - weighted children| / sigma_c max %.4f\n", k - 1, k, worst);
        CHECK(worst <= (hp <= cloud_bake_dense_cell_max_m ? 0.02 : 0.25));
    }
}

TEST_CASE("A clear column bakes exactly 0", "[weather]") {
    const CloudRenderField clear =
        make_cloud_render_field(WeatherSnapshot::uniform_slab(19, 36, slab_bottom_m, slab_top_m, 0.0, 0.0));
    for (int k = 0; k < levels; ++k) {
        for (double h : {0.0, 1500.0, 9000.0}) {
            const CloudCellBox box = cloud_cell_box(at_model_height(26.3, -97.1, h),
                                                    cell_h_m[static_cast<std::size_t>(k)],
                                                    cell_v_m[static_cast<std::size_t>(k)]);
            const CloudCellBake b = bake_cloud_cell(clear, noise(), frame(), box);
            CHECK(b.mean_extinction_per_m == 0.0);
            CHECK(b.in_cloud_extinction_per_m == 0.0);
        }
        // Far above the broken slab: clear as well.
        const CloudCellBox above = cloud_cell_box(at_model_height(26.3, -97.1, 9000.0),
                                                  cell_h_m[static_cast<std::size_t>(k)],
                                                  cell_v_m[static_cast<std::size_t>(k)]);
        CHECK(bake_cloud_cell(broken_slab(), noise(), frame(), above).mean_extinction_per_m == 0.0);
    }
    // A cell up to 1 % off its level's nominal size keeps the level's taps (real grid edges, rounding).
    for (int k = 0; k < levels; ++k) {
        const CloudBakeTaps nominal =
            cloud_cell_bake_taps(cell_h_m[static_cast<std::size_t>(k)], cell_v_m[static_cast<std::size_t>(k)]);
        for (double scale : {0.996, 1.004}) {
            const CloudBakeTaps off = cloud_cell_bake_taps(cell_h_m[static_cast<std::size_t>(k)] * scale,
                                                           cell_v_m[static_cast<std::size_t>(k)] * scale);
            CHECK(off.horizontal == nominal.horizontal);
            CHECK(off.vertical == nominal.vertical);
        }
    }
    // The batched form gives the single bakes.
    std::vector<CloudCellBox> boxes;
    for (int k = 0; k < levels; ++k) {
        boxes.push_back(cloud_cell_box(at_model_height(26.2, -97.3, 1700.0), cell_h_m[static_cast<std::size_t>(k)],
                                       cell_v_m[static_cast<std::size_t>(k)]));
    }
    const std::vector<CloudCellBake> batch = bake_cloud_cells(broken_slab(), noise(), frame(), boxes);
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        const CloudCellBake one = bake_cloud_cell(broken_slab(), noise(), frame(), boxes[i]);
        CHECK(batch[i].mean_extinction_per_m == one.mean_extinction_per_m);
        CHECK(batch[i].in_cloud_extinction_per_m == one.in_cloud_extinction_per_m);
    }
}
