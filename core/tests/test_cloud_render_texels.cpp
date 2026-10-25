// The GPU form of a weather snapshot (M8 weather and clouds, Task 8): linear extinction in two 8-bit channels and the
// in-cloud optical depth above each layer centre.

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
