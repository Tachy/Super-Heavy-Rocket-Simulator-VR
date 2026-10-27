// The GPU form of a weather snapshot (M8 weather and clouds, Task 8); layout in cloud_render_texels.hpp.

#include "starsim/environment/cloud_render_texels.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "starsim/environment/cloud_optics.hpp"
#include "starsim/math/constants.hpp"
#include "parallel_rows.hpp"

namespace starsim::environment {

namespace {

constexpr int channels = 4;
constexpr double extinction_code_count = 65535.0;
const double depth_code_ratio = cloud_depth_code_max / cloud_depth_code_min;

double layer_centre_m(int layer) { return 0.5 * (weather_layer_bottom_m(layer) + weather_layer_bottom_m(layer + 1)); }

// The layer whose height range holds height_m (clamped to the 32 layers).
int layer_containing(double height_m) {
    return std::clamp(static_cast<int>(std::floor(weather_layer_coordinate(height_m))), 0, weather_layer_count - 1);
}

// The largest step of the integrals of cloud_lcl_extinction_scale: an eighth of the base's ramp.
constexpr double lcl_integration_step_m = constants::cloud_lcl_ramp_m / 8.0;

// The renderer's grid-mean extinction profile of one cell (the shader's cloud_snapshot_cell at the column centre,
// without the base cut): the vertical profile rule between the layer centres, the end values held.
class ColumnProfile {
public:
    ColumnProfile(const WeatherSnapshot& snapshot, std::size_t cell) {
        const std::size_t cells = static_cast<std::size_t>(snapshot.rows()) * snapshot.columns();
        for (int layer = 0; layer < weather_layer_count; ++layer) {
            const std::size_t i = (static_cast<std::size_t>(layer) * cells + cell) * channels;
            const auto k = static_cast<std::size_t>(layer);
            fraction_[k] = decode_weather_cloud_fraction(snapshot.texels()[i]);
            beta_[k] = cloud_extinction_per_m(decode_weather_water(snapshot.texels()[i + 1]),
                                              decode_weather_water(snapshot.texels()[i + 2]));
        }
    }
    double at(double height_m) const {
        const double s = weather_layer_coordinate(height_m) - 0.5;
        const int k = std::clamp(static_cast<int>(std::floor(s)), 0, weather_layer_count - 2);
        const double x = std::clamp(s - k, 0.0, 1.0);
        const auto lo = static_cast<std::size_t>(k), hi = lo + 1;
        return cloud_profile_value(beta_[lo], beta_[hi], fraction_[lo], fraction_[hi], x,
                                   layer_centre_m(k + 1) - layer_centre_m(k));
    }

private:
    std::array<double, weather_layer_count> beta_{}, fraction_{};
};

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

double cloud_render_top_m(const WeatherSnapshot& snapshot) {
    const std::vector<std::uint8_t>& source = snapshot.texels();
    const std::size_t cells = static_cast<std::size_t>(snapshot.rows()) * snapshot.columns();
    for (int layer = weather_layer_count - 1; layer >= 0; --layer) {
        const std::size_t base = static_cast<std::size_t>(layer) * cells * channels;
        for (std::size_t cell = 0; cell < cells; ++cell) {
            const std::size_t i = base + cell * channels;
            // Any water counts, also in a cell whose fraction code is 0: the grid-mean paths render it.
            if (source[i + 1] != 0 || source[i + 2] != 0) {
                return layer == weather_layer_count - 1 ? weather_top_m : layer_centre_m(layer + 1);
            }
        }
    }
    return 0.0;
}

std::vector<std::uint8_t> cloud_render_column_texels(const WeatherSnapshot& snapshot) {
    const std::vector<std::uint8_t>& source = snapshot.texels();
    const std::size_t cells = static_cast<std::size_t>(snapshot.rows()) * snapshot.columns();
    std::vector<std::uint8_t> out(source.size() / channels * 2, 0);
    std::array<double, 256> water{};
    for (int code = 0; code < 256; ++code) water[code] = decode_weather_water(static_cast<std::uint8_t>(code));
    // Top-down: the running grid-mean depth, the grid-mean extinction of the centre above and the largest cloud fraction
    // code so far, per cell.
    std::vector<double> depth(cells, 0.0), beta_above(cells, 0.0);
    std::vector<std::uint8_t> cover(cells, 0);
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
            cover[cell] = std::max(cover[cell], source[i]);
            out[2 * (base + cell)] = encode_cloud_optical_depth(depth[cell]);
            out[2 * (base + cell) + 1] = cover[cell];
        }
    }
    return out;
}

double cloud_lcl_cut_factor(double height_m, double base_m) {
    if (!std::isfinite(base_m) || height_m >= weather_layer_bottom_m(layer_containing(base_m) + 1)) return 1.0;
    return std::clamp((height_m - base_m) / constants::cloud_lcl_ramp_m, 0.0, 1.0);
}

double cloud_lcl_extinction_factor(double height_m, double base_m, double scale) {
    if (!std::isfinite(base_m) || !(scale > 0.0) || height_m >= weather_layer_bottom_m(layer_containing(base_m) + 1)) {
        return 1.0;
    }
    return cloud_lcl_cut_factor(height_m, base_m) * scale;
}

double cloud_lcl_extinction_scale(const WeatherSnapshot& snapshot, int row, int column, double base_m) {
    const std::size_t cell = static_cast<std::size_t>(row) * snapshot.columns() + column;
    const ColumnProfile profile(snapshot, cell);
    const int layer = layer_containing(base_m);
    const double top = weather_layer_bottom_m(layer + 1);
    const double ground = snapshot.ground_heights_m()[cell];
    const double bottom = std::max(weather_layer_bottom_m(layer), ground);
    // Simpson's rule on the pieces between the kinks (the base, the ramp's end, the layer centre and the edges of the
    // profile rule's base transitions at the layer's bottom and top): profile and cut are linear on each piece (the
    // profile rule is linear in the height inside one layer), so the rule is exact up to rounding.
    const double transition = 0.5 * constants::cloud_base_transition_m;
    std::array<double, 7> edges = {bottom, top, base_m, base_m + constants::cloud_lcl_ramp_m, layer_centre_m(layer),
                                   weather_layer_bottom_m(layer) + transition, top - transition};
    for (double& e : edges) e = std::clamp(e, bottom, top);
    std::sort(edges.begin(), edges.end());
    double whole = 0.0, kept = 0.0;
    for (std::size_t piece = 0; piece + 1 < edges.size(); ++piece) {
        const double length = edges[piece + 1] - edges[piece];
        if (!(length > 0.0)) continue;
        const int steps = 2 * std::max(1, static_cast<int>(std::ceil(length / lcl_integration_step_m)));
        const double step = length / steps;
        for (int i = 0; i <= steps; ++i) {
            const double h = edges[piece] + i * step;
            const double weight = (i == 0 || i == steps ? 1.0 : (i % 2 == 1 ? 4.0 : 2.0)) * step / 3.0;
            const double beta = profile.at(h) * weight;
            whole += beta;
            kept += beta * std::clamp((h - base_m) / constants::cloud_lcl_ramp_m, 0.0, 1.0);  // inside the layer
        }
    }
    return kept > 0.0 ? whole / kept : 1.0;
}

std::vector<float> cloud_render_base_texels(const WeatherSnapshot& snapshot) {
    const std::vector<std::uint8_t>& source = snapshot.texels();
    const int columns = snapshot.columns();
    const std::size_t cells = static_cast<std::size_t>(snapshot.rows()) * columns;
    std::vector<float> out(cells * 2, 0.0f);
    detail::for_each_row(snapshot.rows(), [&](int r) {
        for (int col = 0; col < columns; ++col) {
            const std::size_t cell = static_cast<std::size_t>(r) * columns + col;
            const float lcl = snapshot.lcl_heights_m()[cell];
            if (!(lcl > 0.0f) || !(lcl < constants::cloud_boundary_layer_top_m)) continue;
            const double base = snapshot.ground_heights_m()[cell] + static_cast<double>(lcl);
            // A base closer to its layer's top than twice the march's smallest step is the top: no cut (scale 1), the
            // sheet of at most that thickness is not resolved and its scale would diverge.
            if (weather_layer_bottom_m(layer_containing(base) + 1) - base < constants::cloud_base_snap_m) continue;
            // No base where condensate remains in a layer wholly under the base's layer (the cut would erase it).
            bool clear_below = true;
            for (int layer = 0; layer < layer_containing(base) && clear_below; ++layer) {
                const std::size_t i = (static_cast<std::size_t>(layer) * cells + cell) * channels;
                clear_below = source[i + 1] == 0 && source[i + 2] == 0;
            }
            if (!clear_below) continue;
            out[2 * cell] = static_cast<float>(base);
            out[2 * cell + 1] = static_cast<float>(cloud_lcl_extinction_scale(snapshot, r, col, base));
        }
    });
    return out;
}

double cloud_render_column_optical_depth(const WeatherSnapshot& snapshot, int row, int column) {
    const std::size_t cells = static_cast<std::size_t>(snapshot.rows()) * snapshot.columns();
    const std::size_t cell = static_cast<std::size_t>(row) * snapshot.columns() + column;
    const double ground = snapshot.ground_heights_m()[cell];
    double depth = 0.0;
    for (int layer = 0; layer < weather_layer_count; ++layer) {
        const std::size_t i = (static_cast<std::size_t>(layer) * cells + cell) * channels;
        const double thickness =
            std::max(0.0, weather_layer_bottom_m(layer + 1) - std::max(weather_layer_bottom_m(layer), ground));
        depth += cloud_extinction_per_m(decode_weather_water(snapshot.texels()[i + 1]),
                                        decode_weather_water(snapshot.texels()[i + 2])) * thickness;
    }
    return depth;
}

}  // namespace starsim::environment
