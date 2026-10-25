// The GPU form of a weather snapshot (M8 weather and clouds, Task 8); layout in cloud_render_texels.hpp.

#include "starsim/environment/cloud_render_texels.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "starsim/environment/cloud_optics.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr int channels = 4;
constexpr double extinction_code_count = 65535.0;
const double depth_code_ratio = cloud_depth_code_max / cloud_depth_code_min;

double layer_centre_m(int layer) { return 0.5 * (weather_layer_bottom_m(layer) + weather_layer_bottom_m(layer + 1)); }

}  // namespace

std::uint8_t encode_cloud_optical_depth(double optical_depth) {
    if (!(optical_depth > 0.0)) return 0;
    const double code = std::log(optical_depth / cloud_depth_code_min) / std::log(depth_code_ratio) * 254.0 + 1.0;
    if (code < 0.5) return 0;
    return static_cast<std::uint8_t>(std::min(255.0, std::round(code)));
}

double decode_cloud_optical_depth(std::uint8_t code) {
    if (code == 0) return 0.0;
    return cloud_depth_code_min * std::pow(depth_code_ratio, (code - 1) / 254.0);
}

double cloud_texel_extinction_step_per_m() {
    static const double step =
        cloud_extinction_per_m(weather_water_code_max_kg_m3, weather_water_code_max_kg_m3) / extinction_code_count;
    return step;
}

double cloud_texel_extinction_per_m(std::uint8_t high, std::uint8_t low) {
    return (256.0 * high + low) * cloud_texel_extinction_step_per_m();
}

double cloud_profile_value(double value_below, double value_above, double fraction_below, double fraction_above,
                           double x, double gap_m) {
    const double linear = (1.0 - x) * value_below + x * value_above;
    if (!(fraction_above > 0.0) || !(value_above > value_below)) return linear;
    const double b = std::pow(std::clamp(1.0 - fraction_below / fraction_above, 0.0, 1.0), 2.0);
    const double half_width = 0.5 * constants::cloud_base_transition_m / gap_m;
    const double step = std::clamp((x - 0.5) / (2.0 * half_width) + 0.5, 0.0, 1.0);
    return linear + b * (step - x) * (value_above - value_below);
}

std::vector<std::uint8_t> cloud_render_ice_texels(const WeatherSnapshot& snapshot) {
    const std::vector<std::uint8_t>& source = snapshot.texels();
    std::vector<std::uint8_t> out(source.size() / channels, 0);
    std::array<double, 256> water{};
    for (int code = 0; code < 256; ++code) water[code] = decode_weather_water(static_cast<std::uint8_t>(code));
    for (std::size_t texel = 0; texel < out.size(); ++texel) {
        const std::size_t i = texel * channels;
        const double ice = cloud_extinction_per_m(0.0, water[source[i + 2]]);
        const double total = cloud_extinction_per_m(water[source[i + 1]], 0.0) + ice;
        if (total > 0.0) out[texel] = static_cast<std::uint8_t>(std::lround(255.0 * ice / total));
    }
    return out;
}

std::vector<std::uint8_t> cloud_render_texels(const WeatherSnapshot& snapshot) {
    const std::vector<std::uint8_t>& source = snapshot.texels();
    std::vector<std::uint8_t> out(source.size(), 0);
    const std::size_t cells = static_cast<std::size_t>(snapshot.rows()) * snapshot.columns();
    // Extinction of every (liquid code, ice code) pair would be 64 k entries; the water decode is a table already.
    std::array<double, 256> water{};
    for (int code = 0; code < 256; ++code) water[code] = decode_weather_water(static_cast<std::uint8_t>(code));
    const double step = cloud_texel_extinction_step_per_m();
    // Top-down over the layers (each layer is contiguous): the running depth and the in-cloud extinction of the centre
    // above, per cell.
    std::vector<double> depth(cells, 0.0), beta_above(cells, 0.0);
    for (int layer = weather_layer_count - 1; layer >= 0; --layer) {
        const std::size_t base = static_cast<std::size_t>(layer) * cells * channels;
        const double gap_m = layer == weather_layer_count - 1 ? weather_top_m - layer_centre_m(layer)
                                                               : layer_centre_m(layer + 1) - layer_centre_m(layer);
        for (std::size_t cell = 0; cell < cells; ++cell) {
            const std::size_t i = base + cell * channels;
            const double beta = cloud_extinction_per_m(water[source[i + 1]], water[source[i + 2]]);
            const auto code = static_cast<long>(std::lround(std::min(beta / step, extinction_code_count)));
            out[i] = source[i];
            out[i + 1] = static_cast<std::uint8_t>(code >> 8);
            out[i + 2] = static_cast<std::uint8_t>(code & 0xff);
            const double fraction = decode_weather_cloud_fraction(source[i]);
            const double in_cloud = fraction > 0.0 ? beta / fraction : 0.0;
            depth[cell] += layer == weather_layer_count - 1 ? in_cloud * gap_m
                                                            : 0.5 * (in_cloud + beta_above[cell]) * gap_m;
            beta_above[cell] = in_cloud;
            out[i + 3] = encode_cloud_optical_depth(depth[cell]);
        }
    }
    return out;
}

std::vector<std::uint8_t> cloud_render_column_texels(const WeatherSnapshot& snapshot) {
    const std::vector<std::uint8_t>& source = snapshot.texels();
    const std::size_t cells = static_cast<std::size_t>(snapshot.rows()) * snapshot.columns();
    std::vector<std::uint8_t> out(source.size() / channels, 0);
    std::array<double, 256> water{};
    for (int code = 0; code < 256; ++code) water[code] = decode_weather_water(static_cast<std::uint8_t>(code));
    // Top-down: the running grid-mean depth and the grid-mean extinction of the centre above, per cell.
    std::vector<double> depth(cells, 0.0), beta_above(cells, 0.0);
    for (int layer = weather_layer_count - 1; layer >= 0; --layer) {
        const std::size_t base = static_cast<std::size_t>(layer) * cells;
        const bool top = layer == weather_layer_count - 1;
        const double gap_m =
            top ? weather_top_m - layer_centre_m(layer) : layer_centre_m(layer + 1) - layer_centre_m(layer);
        for (std::size_t cell = 0; cell < cells; ++cell) {
            const std::size_t i = (base + cell) * channels;
            const double beta = cloud_extinction_per_m(water[source[i + 1]], water[source[i + 2]]);
            depth[cell] += top ? beta * gap_m : 0.5 * (beta + beta_above[cell]) * gap_m;
            beta_above[cell] = beta;
            out[base + cell] = encode_cloud_optical_depth(depth[cell]);
        }
    }
    return out;
}

}  // namespace starsim::environment
