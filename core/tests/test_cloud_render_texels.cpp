// The GPU form of a weather snapshot (M8 weather and clouds, Task 8): linear extinction in two 8-bit channels and the
// in-cloud optical depth above each layer centre.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/cloud_optics.hpp"
#include "starsim/environment/cloud_render_texels.hpp"
#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::environment;
using Catch::Approx;

namespace {

double layer_centre_m(int layer) { return 0.5 * (weather_layer_bottom_m(layer) + weather_layer_bottom_m(layer + 1)); }

std::size_t texel_offset(int layer, int row, int column, int rows, int columns) {
    return 4 * ((static_cast<std::size_t>(layer) * rows + row) * columns + column);
}

}  // namespace

TEST_CASE("Cloud optical depth code round trip", "[weather]") {
    CHECK(encode_cloud_optical_depth(0.0) == 0);
    CHECK(decode_cloud_optical_depth(0) == 0.0);
    CHECK(decode_cloud_optical_depth(1) == Approx(cloud_depth_code_min));
    CHECK(decode_cloud_optical_depth(255) == Approx(cloud_depth_code_max));
    CHECK(encode_cloud_optical_depth(1e9) == 255);
    for (double tau : {0.02, 0.3, 1.0, 7.5, 42.0, 900.0}) {
        CHECK(decode_cloud_optical_depth(encode_cloud_optical_depth(tau)) == Approx(tau).epsilon(0.03));
    }
}

TEST_CASE("Cloud render texels carry the snapshot's extinction linearly", "[weather]") {
    // A slab with ice and liquid (uniform_slab has liquid only: build one from parts with both).
    const int rows = 5, columns = 8;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    for (int layer = 6; layer <= 9; ++layer) {
        for (int r = 0; r < rows; ++r) {
            for (int col = 0; col < columns; ++col) {
                const std::size_t i = texel_offset(layer, r, col, rows, columns);
                texels[i] = encode_weather_cloud_fraction(layer == 9 ? 0.4 : 1.0);
                texels[i + 1] = encode_weather_water(3e-4 * (1 + col));
                texels[i + 2] = encode_weather_water(2e-5);
            }
        }
    }
    const WeatherSnapshot snapshot = WeatherSnapshot::from_parts({}, rows, columns,
                                                                 std::vector<float>(rows * columns, 0.0f), texels);
    const std::vector<std::uint8_t> gpu = cloud_render_texels(snapshot);
    REQUIRE(gpu.size() == texels.size());
    for (int layer = 0; layer < weather_layer_count; ++layer) {
        for (int col = 0; col < columns; ++col) {
            const std::size_t i = texel_offset(layer, 2, col, rows, columns);
            CHECK(gpu[i] == texels[i]);  // cloud fraction unchanged
            const double expected = cloud_extinction_per_m(decode_weather_water(texels[i + 1]),
                                                           decode_weather_water(texels[i + 2]));
            const double stored = cloud_texel_extinction_per_m(gpu[i + 1], gpu[i + 2]);
            CHECK(stored == Approx(expected).margin(0.5 * cloud_texel_extinction_step_per_m()));
        }
    }
    // Linear in the two channels: the mean of two texels' codes is the code of the mean (what GPU filtering does).
    const std::size_t a = texel_offset(7, 2, 0, rows, columns), b = texel_offset(7, 2, 7, rows, columns);
    const double hi = 0.5 * (gpu[a + 1] + gpu[b + 1]), lo = 0.5 * (gpu[a + 2] + gpu[b + 2]);
    const double mean = (hi * 256.0 + lo) * cloud_texel_extinction_step_per_m();
    CHECK(mean == Approx(0.5 * (cloud_texel_extinction_per_m(gpu[a + 1], gpu[a + 2]) +
                                cloud_texel_extinction_per_m(gpu[b + 1], gpu[b + 2])))
                      .epsilon(1e-12));
}

TEST_CASE("Cloud render texels carry the in-cloud depth above each layer centre", "[weather]") {
    // Two decks in one column: broken cumulus (cloud fraction 0.5) and an overcast deck above, ground at 0.
    const int rows = 3, columns = 4;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    auto put = [&](int layer, double fraction, double liquid) {
        for (int r = 0; r < rows; ++r) {
            for (int col = 0; col < columns; ++col) {
                const std::size_t i = texel_offset(layer, r, col, rows, columns);
                texels[i] = encode_weather_cloud_fraction(fraction);
                texels[i + 1] = encode_weather_water(liquid);
            }
        }
    };
    for (int layer = 4; layer <= 7; ++layer) put(layer, 0.5, 1e-4);   // 1-2 km
    for (int layer = 18; layer <= 19; ++layer) put(layer, 1.0, 2e-5);  // 6-8 km
    put(31, 1.0, 1e-6);                                                // 19-20 km: the extension above the top centre
    const WeatherSnapshot snapshot = WeatherSnapshot::from_parts({}, rows, columns,
                                                                 std::vector<float>(rows * columns, 0.0f), texels);
    const std::vector<std::uint8_t> gpu = cloud_render_texels(snapshot);

    // Reference: in-cloud extinction (grid mean / cloud fraction) at the centres, trapezoids between them, held from
    // the top centre to 20 km.
    std::vector<double> beta(weather_layer_count);
    for (int layer = 0; layer < weather_layer_count; ++layer) {
        const std::size_t i = texel_offset(layer, 1, 1, rows, columns);
        const double fraction = decode_weather_cloud_fraction(texels[i]);
        beta[layer] = fraction > 0.0 ? cloud_extinction_per_m(decode_weather_water(texels[i + 1]), 0.0) / fraction
                                     : 0.0;
    }
    double depth = beta[31] * (weather_top_m - layer_centre_m(31));
    for (int layer = weather_layer_count - 1; layer >= 0; --layer) {
        if (layer < weather_layer_count - 1) {
            depth += 0.5 * (beta[layer] + beta[layer + 1]) * (layer_centre_m(layer + 1) - layer_centre_m(layer));
        }
        const std::uint8_t code = gpu[texel_offset(layer, 1, 1, rows, columns) + 3];
        if (depth < cloud_depth_code_min) {
            CHECK(code <= 1);
        } else {
            CHECK(decode_cloud_optical_depth(code) == Approx(depth).epsilon(0.03));
        }
    }
    // The column below the cumulus holds every deck: the profile is linear between centres, so a deck of n whole
    // layers integrates to n layer thicknesses (half ramps on both sides); the top layer: ramp + 500 m held.
    const double column = decode_cloud_optical_depth(gpu[texel_offset(0, 1, 1, rows, columns) + 3]);
    CHECK(column == Approx(1000.0 * beta[5] + 2000.0 * beta[18] + 1000.0 * beta[31]).epsilon(0.03));
}

TEST_CASE("Cloud vertical profile: flat bases, the water path kept", "[weather]") {
    // A cloud above a clear layer (fraction 0 below, 0.4 above): zero below the middle of the gap minus half the base
    // transition, the upper value above it plus half of it; the integral over the gap equals the linear interpolation's.
    for (double gap : {250.0, 625.0, 1000.0}) {
        const double hw = 0.5 * constants::cloud_base_transition_m / gap;
        CHECK(cloud_profile_value(0.0, 2e-3, 0.0, 0.4, 0.5 - hw - 1e-3, gap) == 0.0);
        CHECK(cloud_profile_value(0.0, 2e-3, 0.0, 0.4, 0.5 + hw + 1e-3, gap) == Approx(2e-3));
        auto integral = [&](double vb, double va, double fb, double fa) {
            double sum = 0.0;
            const int n = 100000;
            for (int i = 0; i < n; ++i) sum += cloud_profile_value(vb, va, fb, fa, (i + 0.5) / n, gap) / n;
            return sum;
        };
        for (const auto& c : {std::array<double, 4>{0.0, 2e-3, 0.0, 0.4}, std::array<double, 4>{1e-3, 2e-3, 0.2, 0.4},
                              std::array<double, 4>{2e-3, 0.0, 0.4, 0.0}, std::array<double, 4>{1e-3, 1e-3, 0.4, 0.4}}) {
            CHECK(integral(c[0], c[1], c[2], c[3]) == Approx(0.5 * (c[0] + c[1])).epsilon(1e-4));
        }
    }
    // Tops (thinning upward) and uniform decks: the linear interpolation.
    CHECK(cloud_profile_value(2e-3, 0.0, 0.4, 0.0, 0.3, 250.0) == Approx(0.7 * 2e-3));
    CHECK(cloud_profile_value(1e-3, 1e-3, 1.0, 1.0, 0.3, 250.0) == Approx(1e-3));
    // Never negative.
    for (double x = 0.0; x <= 1.0; x += 0.01) CHECK(cloud_profile_value(1e-3, 3e-3, 0.1, 0.5, x, 250.0) >= 0.0);
}

TEST_CASE("Cloud ice share texels", "[weather]") {
    const int rows = 2, columns = 3;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    const std::size_t liquid = texel_offset(5, 0, 0, rows, columns), ice = texel_offset(30, 1, 2, rows, columns),
                      mixed = texel_offset(12, 1, 1, rows, columns);
    texels[liquid] = texels[ice] = texels[mixed] = encode_weather_cloud_fraction(1.0);
    texels[liquid + 1] = encode_weather_water(3e-4);
    texels[ice + 2] = encode_weather_water(2e-5);
    texels[mixed + 1] = encode_weather_water(1e-4);
    texels[mixed + 2] = encode_weather_water(1e-4);
    const WeatherSnapshot snapshot = WeatherSnapshot::from_parts({}, rows, columns,
                                                                 std::vector<float>(rows * columns, 0.0f), texels);
    const std::vector<std::uint8_t> share = cloud_render_ice_texels(snapshot);
    REQUIRE(share.size() == texels.size() / 4);
    CHECK(share[liquid / 4] == 0);
    CHECK(share[ice / 4] == 255);
    // Equal water: ice extinction is rho_w r_l / (rho_ice r_i) of the liquid's.
    const double ratio = cloud_extinction_per_m(0.0, decode_weather_water(texels[mixed + 2])) /
                         (cloud_extinction_per_m(decode_weather_water(texels[mixed + 1]), 0.0) +
                          cloud_extinction_per_m(0.0, decode_weather_water(texels[mixed + 2])));
    CHECK(share[mixed / 4] == static_cast<int>(std::lround(255.0 * ratio)));
    CHECK(share[texel_offset(0, 0, 0, rows, columns) / 4] == 0);  // clear
}

TEST_CASE("Cloud column texels carry the grid-mean depth and the cover above each layer centre", "[weather]") {
    // Broken cumulus (cloud fraction 0.5) under an overcast deck, the same in every cell (ground at 0): the GPU's
    // grid-mean column above each layer centre (the sky factor's and the Sun's diffuse light's tau, Task 9) against the
    // CPU's cloud_column_optical_depth_above at the centre, within the code's half step (2.8 %).
    const int rows = 3, columns = 4;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    auto put = [&](int layer, double fraction, double liquid) {
        for (int r = 0; r < rows; ++r) {
            for (int col = 0; col < columns; ++col) {
                const std::size_t i = texel_offset(layer, r, col, rows, columns);
                texels[i] = encode_weather_cloud_fraction(fraction);
                texels[i + 1] = encode_weather_water(liquid);
            }
        }
    };
    for (int layer = 4; layer <= 7; ++layer) put(layer, 0.5, 1e-4);
    for (int layer = 18; layer <= 19; ++layer) put(layer, 1.0, 2e-5);
    put(31, 1.0, 1e-6);
    const WeatherSnapshot snapshot = WeatherSnapshot::from_parts({}, rows, columns,
                                                                 std::vector<float>(rows * columns, 0.0f), texels);
    const std::vector<std::uint8_t> column = cloud_render_column_texels(snapshot);
    REQUIRE(column.size() == texels.size() / 2);
    for (int layer = 0; layer < weather_layer_count; ++layer) {
        const CloudColumnDepths cpu = cloud_column_depths_above(snapshot, 0.3, 0.5, layer_centre_m(layer));
        const double expected = cpu.mean;
        const std::size_t texel = texel_offset(layer, 1, 2, rows, columns) / 4;
        // G: the largest cloud fraction from the top down to the centre (the column's cover).
        CHECK(decode_weather_cloud_fraction(column[2 * texel + 1]) == Approx(cpu.max_fraction).margin(0.003));
        const std::uint8_t code = column[2 * texel];
        if (expected < cloud_depth_code_min) {
            CHECK(code <= 1);
        } else {
            CHECK(decode_cloud_optical_depth(code) == Approx(expected).epsilon(0.03));
        }
    }
}

TEST_CASE("Base texels carry the flat cloud base and its layer's extinction scale", "[weather]") {
    // Ground heights 0, 1500 m (ellipsoid) with LCLs of 800 m, 600 m; saturated (0), unknown (-1), a level above the
    // boundary layer (3500 m), 250 m over -40 m ground, and the weather top (20 km, a capped level): only cells 0, 1
    // and 5 get a base, the others stay 0 (not cut).
    const int rows = 2, columns = 4;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    const std::vector<float> ground = {0.0f, 1500.0f, 20.0f, 20.0f, 20.0f, -40.0f, 0.0f, 0.0f};
    const std::vector<float> lcl = {800.0f, 600.0f, 0.0f, weather_lcl_unknown_m, 3500.0f, 250.0f, 20000.0f, 800.0f};
    // Cell 7: ice in layer 1 (250 .. 500 m), wholly under its level's layer: no base (the cut would erase that ice).
    texels[texel_offset(1, 1, 3, rows, columns) + 2] = encode_weather_water(1e-5);
    // Cell 0: liquid in the base's own layer (750 .. 1000 m) and above does not stop the base (the layer keeps it above
    // the level); its scale puts the profile's part under the base back above it.
    for (int layer = 3; layer < 8; ++layer) {
        texels[texel_offset(layer, 0, 0, rows, columns)] = encode_weather_cloud_fraction(0.6);
        texels[texel_offset(layer, 0, 0, rows, columns) + 1] = encode_weather_water(1e-4);
    }
    const WeatherSnapshot snapshot = WeatherSnapshot::from_parts({}, rows, columns, ground, texels, lcl);
    const std::vector<float> base = cloud_render_base_texels(snapshot);
    REQUIRE(base.size() == static_cast<std::size_t>(rows) * columns * 2);
    CHECK(base[0] == Approx(800.0));
    CHECK(base[2] == Approx(2100.0));  // from the ground, not the ellipsoid
    CHECK(base[10] == Approx(210.0));
    CHECK(base[1] > 1.0f);
    CHECK(base[1] == Approx(cloud_lcl_extinction_scale(snapshot, 0, 0, 800.0)));
    CHECK(base[3] == 1.0f);   // no cloud in the base's layer: nothing to put back
    CHECK(base[11] == 1.0f);
    for (int cell : {2, 3, 4, 6, 7}) CHECK(base[static_cast<std::size_t>(2 * cell + 1)] == 0.0f);
    // A trace of liquid (the smallest code) under the level's layer also keeps the cell uncut.
    texels[texel_offset(0, 0, 0, rows, columns) + 1] = 1;
    const std::vector<float> traced =
        cloud_render_base_texels(WeatherSnapshot::from_parts({}, rows, columns, ground, texels, lcl));
    CHECK(traced[1] == 0.0f);
}

TEST_CASE("A base just under its layer's top snaps to the top: no cut, no unbounded scale", "[weather]") {
    // Layer 3 is 750 .. 1000 m: bases 5 m and 19 m under its top (less than cloud_base_snap_m) are dropped, 25 m stays.
    const int rows = 2, columns = 3;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    for (int layer = 3; layer < 8; ++layer) {
        for (int col = 0; col < columns; ++col) {
            const std::size_t i = texel_offset(layer, 0, col, rows, columns);
            texels[i] = encode_weather_cloud_fraction(0.6);
            texels[i + 1] = encode_weather_water(1e-4);
        }
    }
    const std::vector<float> ground(rows * columns, 0.0f),
        lcl = {995.0f, 981.0f, 975.0f, 0.0f, 0.0f, 0.0f};
    const std::vector<float> base =
        cloud_render_base_texels(WeatherSnapshot::from_parts({}, rows, columns, ground, texels, lcl));
    CHECK(base[0] == 0.0f);
    CHECK(base[1] == 0.0f);
    CHECK(base[2] == 0.0f);
    CHECK(base[3] == 0.0f);
    CHECK(base[4] == Approx(975.0));
    CHECK(base[5] > 1.0f);
    CHECK(base[5] < 50.0f);  // measured 23.4 at 25 m under the top: the largest scale a base can now get
}

TEST_CASE("The base cut ramps the cloud in over cloud_lcl_ramp_m above the level, inside its layer", "[weather]") {
    const double base = 1200.0, ramp = constants::cloud_lcl_ramp_m;  // layer 4: 1000 .. 1250 m
    CHECK(cloud_lcl_cut_factor(base - 500.0, base) == 0.0);
    CHECK(cloud_lcl_cut_factor(base, base) == 0.0);
    CHECK(cloud_lcl_cut_factor(base + 0.5 * ramp, base) == Approx(0.5));
    CHECK(cloud_lcl_cut_factor(base + ramp, base) == 1.0);
    CHECK(cloud_lcl_cut_factor(base + 4.0 * ramp, base) == 1.0);
    CHECK(cloud_lcl_cut_factor(10.0, std::nan("")) == 1.0);  // no base: not cut
    // A base 10 m under its layer's top: the ramp ends at the top (1250 m), the layer above is not cut.
    CHECK(cloud_lcl_cut_factor(1249.0, 1240.0) == Approx(9.0 / ramp));
    CHECK(cloud_lcl_cut_factor(1250.0, 1240.0) == 1.0);
    CHECK(cloud_lcl_extinction_factor(1249.0, 1240.0, 3.0) == Approx(3.0 * 9.0 / ramp));
    CHECK(cloud_lcl_extinction_factor(1250.0, 1240.0, 3.0) == 1.0);
    CHECK(cloud_lcl_extinction_factor(1100.0, 1240.0, 3.0) == 0.0);
    CHECK(cloud_lcl_extinction_factor(1100.0, 1240.0, 0.0) == 1.0);  // scale 0: no base
    CHECK(cloud_lcl_extinction_factor(5000.0, std::nan(""), 3.0) == 1.0);
}

TEST_CASE("The base cut keeps the rendered column: the scaled profile integrates to the uncut one", "[weather]") {
    // A cloud on layers 3 .. 7 with a denser layer 4 (the profile rises through the base's layer), ice in layer 6;
    // bases low / mid / near the top of layer 3 (750 .. 1000 m), within the ramp of its top, on its bottom; a ground inside
    // the base's layer.
    const int rows = 1 + 2, columns = 2;
    std::vector<std::uint8_t> texels(static_cast<std::size_t>(weather_layer_count) * rows * columns * 4, 0);
    const std::array<double, 5> liquid = {1e-4, 4e-4, 3e-4, 3e-4, 2e-4};
    for (int layer = 3; layer < 8; ++layer) {
        const std::size_t i = texel_offset(layer, 1, 0, rows, columns);
        texels[i] = encode_weather_cloud_fraction(layer == 3 ? 0.4 : 0.7);
        texels[i + 1] = encode_weather_water(liquid[static_cast<std::size_t>(layer - 3)]);
        if (layer == 6) texels[i + 2] = encode_weather_water(2e-5);
    }
    for (const auto& [base, ground] : std::array<std::array<double, 2>, 6>{
             {{760.0, 0.0}, {880.0, 0.0}, {965.0, 0.0}, {985.0, 0.0}, {750.0, 0.0}, {900.0, 820.0}}}) {
        std::vector<float> heights(static_cast<std::size_t>(rows) * columns, static_cast<float>(ground));
        const WeatherSnapshot snapshot = WeatherSnapshot::from_parts({}, rows, columns, heights, texels);
        const double scale = cloud_lcl_extinction_scale(snapshot, 1, 0, base);
        // The renderer's profile at the column (CPU twin of the shader: the profile rule between the centres)
        // integrated over the base's layer with and without the cut: 1 cm steps.
        const auto beta = [&](int layer) {
            const std::size_t i = texel_offset(layer, 1, 0, rows, columns);
            return cloud_extinction_per_m(decode_weather_water(texels[i + 1]), decode_weather_water(texels[i + 2]));
        };
        const auto fraction = [&](int layer) {
            return decode_weather_cloud_fraction(texels[texel_offset(layer, 1, 0, rows, columns)]);
        };
        const auto profile = [&](double h) {
            const double s = weather_layer_coordinate(h) - 0.5;
            const int k = std::clamp(static_cast<int>(std::floor(s)), 0, weather_layer_count - 2);
            return cloud_profile_value(beta(k), beta(k + 1), fraction(k), fraction(k + 1), std::clamp(s - k, 0.0, 1.0),
                                       layer_centre_m(k + 1) - layer_centre_m(k));
        };
        constexpr double step_m = 0.01;
        double uncut = 0.0, cut = 0.0;
        for (double h = std::max(750.0, ground) + 0.5 * step_m; h < 1000.0; h += step_m) {
            uncut += profile(h);
            cut += profile(h) * cloud_lcl_extinction_factor(h, base, scale);
        }
        INFO("base " << base << " ground " << ground << " scale " << scale);
        CHECK(scale >= 1.0);
        CHECK(cut == Approx(uncut).epsilon(1e-5));
    }
}
