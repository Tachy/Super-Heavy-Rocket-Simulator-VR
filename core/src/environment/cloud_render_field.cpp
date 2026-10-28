#include "starsim/environment/cloud_render_field.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "starsim/environment/cloud_render_texels.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr int channels = 4;
constexpr double two_pi = 2.0 * constants::pi;
// The model space's polar stretch a / b (atmosphere.gdshaderinc ATM_POLAR_STRETCH).
constexpr double polar_stretch = 1.0 / (1.0 - constants::wgs84_flattening);
// The shader's floor of the edge mean factor (clouds.gdshaderinc: max(m, 1e-4)).
constexpr double edge_mean_floor = 1.0e-4;

double layer_centre_m(int layer) { return 0.5 * (weather_layer_bottom_m(layer) + weather_layer_bottom_m(layer + 1)); }

int wrap(int i, int n) { return ((i % n) + n) % n; }

double wrap01(double x) { return x - std::floor(x); }

// Bilinear texel weights of the field (the shader's cloud_field_uvw): column coordinate x (wraps), row coordinate y
// (clamped to the rows; the row below the last carries weight 0).
struct Bilinear {
    int c0 = 0, c1 = 0, r0 = 0, r1 = 0;
    double fx = 0.0, fy = 0.0;
};

Bilinear field_bilinear(const CloudRenderField& field, double latitude_rad, double longitude_rad) {
    const double x = longitude_rad / two_pi * field.columns;
    const double y = std::clamp((0.5 * constants::pi - latitude_rad) / (constants::pi / (field.rows - 1)), 0.0,
                                static_cast<double>(field.rows - 1));
    Bilinear b;
    const double x0 = std::floor(x), y0 = std::floor(y);
    b.fx = x - x0;
    b.fy = y - y0;
    b.c0 = wrap(static_cast<int>(x0), field.columns);
    b.c1 = wrap(b.c0 + 1, field.columns);
    b.r0 = static_cast<int>(y0);
    b.r1 = std::min(b.r0 + 1, field.rows - 1);
    return b;
}

std::size_t texel(const CloudRenderField& field, int layer, int row, int column) {
    return (static_cast<std::size_t>(layer) * field.rows + row) * field.columns + column;
}

// One layer's filtered (fraction, grid-mean extinction) at the weights.
struct LayerValue {
    double fraction = 0.0, extinction_per_m = 0.0;
};

LayerValue layer_value(const CloudRenderField& field, int layer, const Bilinear& b) {
    const double step = cloud_texel_extinction_step_per_m();
    const std::array<int, 4> rows = {b.r0, b.r0, b.r1, b.r1}, cols = {b.c0, b.c1, b.c0, b.c1};
    const std::array<double, 4> w = {(1.0 - b.fx) * (1.0 - b.fy), b.fx * (1.0 - b.fy), (1.0 - b.fx) * b.fy,
                                     b.fx * b.fy};
    LayerValue v;
    for (std::size_t i = 0; i < 4; ++i) {
        const std::size_t t = channels * texel(field, layer, rows[i], cols[i]);
        v.fraction += w[i] * field.field_rgba[t] / 255.0;
        v.extinction_per_m += w[i] * (256.0 * field.field_rgba[t + 1] + field.field_rgba[t + 2]) * step;
    }
    return v;
}

double ice_value(const CloudRenderField& field, int layer, const Bilinear& b) {
    const std::array<int, 4> rows = {b.r0, b.r0, b.r1, b.r1}, cols = {b.c0, b.c1, b.c0, b.c1};
    const std::array<double, 4> w = {(1.0 - b.fx) * (1.0 - b.fy), b.fx * (1.0 - b.fy), (1.0 - b.fx) * b.fy,
                                     b.fx * b.fy};
    double v = 0.0;
    for (std::size_t i = 0; i < 4; ++i) v += w[i] * field.ice[texel(field, layer, rows[i], cols[i])] / 255.0;
    return v;
}

// The shader's cloud_lcl_column_cut: (fraction factor, extinction factor) of one column's flat base.
std::array<double, 2> column_cut(double height_m, float base_m, float scale) {
    if (!(scale > 0.0f)) return {1.0, 1.0};
    return {cloud_lcl_cut_factor(height_m, base_m), cloud_lcl_extinction_factor(height_m, base_m, scale)};
}

// The shader's cloud_lcl_cut: the four columns' cuts mixed bilinearly (column centres at x = c, rows clamped so the
// second row exists).
std::array<double, 2> lcl_cut(const CloudRenderField& field, double latitude_rad, double longitude_rad,
                              double height_m) {
    const double x = longitude_rad / two_pi * field.columns;
    const double y = std::clamp((0.5 * constants::pi - latitude_rad) / (constants::pi / (field.rows - 1)), 0.0,
                                static_cast<double>(field.rows - 1));
    const double x0 = std::floor(x), y0 = std::min(std::floor(y), static_cast<double>(field.rows - 2));
    const double fx = x - x0, fy = y - y0;
    const int c0 = wrap(static_cast<int>(x0), field.columns), c1 = wrap(c0 + 1, field.columns);
    const int r0 = static_cast<int>(y0);
    const auto cut = [&](int r, int c) {
        const std::size_t i = 2 * (static_cast<std::size_t>(r) * field.columns + c);
        return column_cut(height_m, field.base[i], field.base[i + 1]);
    };
    const std::array<double, 2> a = cut(r0, c0), b = cut(r0, c1), c = cut(r0 + 1, c0), d = cut(r0 + 1, c1);
    std::array<double, 2> out{};
    for (std::size_t i = 0; i < 2; ++i) {
        out[i] = (1.0 - fy) * ((1.0 - fx) * a[i] + fx * b[i]) + fy * ((1.0 - fx) * c[i] + fx * d[i]);
    }
    return out;
}

// A 256-code table row pair read as the GPU's linear filter: codes along the fraction (clamped to the edge), rows
// along row_coordinate.
template <std::size_t Rows>
double table_lerp(const std::array<std::array<float, 256>, Rows>& table, double row_coordinate, double fraction) {
    const double r = std::clamp(row_coordinate, 0.0, static_cast<double>(Rows - 1));
    const double c = std::clamp(fraction * 255.0, 0.0, 255.0);
    const auto r0 = static_cast<std::size_t>(std::floor(r)), c0 = static_cast<std::size_t>(std::floor(c));
    const std::size_t r1 = std::min(r0 + 1, Rows - 1), c1 = std::min<std::size_t>(c0 + 1, 255);
    const double fr = r - static_cast<double>(r0), fc = c - static_cast<double>(c0);
    const auto row = [&](std::size_t i) { return (1.0 - fc) * table[i][c0] + fc * table[i][c1]; };
    return (1.0 - fr) * row(r0) + fr * row(r1);
}

struct TypeTile {
    double horizontal_m, vertical_m;
};

TypeTile type_tile(CloudType type) {
    switch (type) {
        case CloudType::Stratiform:
            return {cloud_shape_tile_horizontal_m * cloud_stratiform_stretch, cloud_shape_tile_vertical_m};
        case CloudType::Cirrus:
            return {cloud_shape_tile_horizontal_m, cloud_ice_tile_vertical_m};
        default:
            return {cloud_shape_tile_horizontal_m, cloud_shape_tile_vertical_m};
    }
}

// The unwrapped shape coordinates of a point for a tile (the shader's cloud_shape_coordinates_for without the origin).
math::Vec3 shape_coordinates(const math::Vec3& p, const TypeTile& tile) {
    const double k = 1.0 / tile.vertical_m - 1.0 / tile.horizontal_m;
    return p / tile.horizontal_m + math::normalized(p) * (cloud_model_height_m(p) * k);
}

math::Vec3 wrapped(const math::Vec3& q) { return {wrap01(q.x), wrap01(q.y), wrap01(q.z)}; }

}  // namespace

CloudRenderField make_cloud_render_field(const WeatherSnapshot& snapshot) {
    CloudRenderField field;
    field.rows = snapshot.rows();
    field.columns = snapshot.columns();
    field.field_rgba = cloud_render_texels(snapshot);
    field.ice = cloud_render_ice_texels(snapshot);
    field.base = cloud_render_base_texels(snapshot);
    return field;
}

CloudFieldSample cloud_render_field_at(const CloudRenderField& field, double latitude_rad, double longitude_rad,
                                       double height_m) {
    CloudFieldSample out;
    if (height_m > weather_top_m) return out;
    const Bilinear b = field_bilinear(field, latitude_rad, longitude_rad);
    const double s = weather_layer_coordinate(height_m) - 0.5;
    const int k = std::clamp(static_cast<int>(std::floor(s)), 0, weather_layer_count - 2);
    const double x = std::clamp(s - k, 0.0, 1.0);  // below centre 0: the end value; above centre 31: x = 1
    const LayerValue below = layer_value(field, k, b), above = layer_value(field, k + 1, b);
    const double gap = layer_centre_m(k + 1) - layer_centre_m(k);
    out.fraction = cloud_profile_value(below.fraction, above.fraction, below.fraction, above.fraction, x, gap);
    out.extinction_per_m =
        cloud_profile_value(below.extinction_per_m, above.extinction_per_m, below.fraction, above.fraction, x, gap);
    const std::array<double, 2> cut = lcl_cut(field, latitude_rad, longitude_rad, height_m);
    out.fraction *= cut[0];
    out.extinction_per_m *= cut[1];
    // The ice share: trilinear in the layer coordinate clamped to the centres.
    const double si = std::clamp(s, 0.0, static_cast<double>(weather_layer_count - 1));
    const int li = std::min(static_cast<int>(std::floor(si)), weather_layer_count - 1);
    const double fi = si - li;
    out.ice_share = fi > 0.0 ? (1.0 - fi) * ice_value(field, li, b) + fi * ice_value(field, li + 1, b)
                             : ice_value(field, li, b);
    const double above_next =
        k + 2 < weather_layer_count ? layer_value(field, k + 2, b).fraction : above.fraction;
    out.stratiform = cloud_stratiform_weight_between(below.fraction, above.fraction, above_next, x);
    return out;
}

double cloud_model_height_m(const math::Vec3& p) {
    return math::norm(math::Vec3{p.x, p.y * polar_stretch, p.z}) - constants::wgs84_semi_major_axis_m;
}

void cloud_latitude_longitude(const math::Vec3& p, double& latitude_rad, double& longitude_rad) {
    latitude_rad = std::atan2(polar_stretch * polar_stretch * p.y, std::hypot(p.x, p.z));
    longitude_rad = std::atan2(p.x, p.z);
}

CloudShapeFrame make_cloud_shape_frame(const math::Vec3& anchor_earth_m) {
    CloudShapeFrame frame;
    frame.anchor_earth_m = anchor_earth_m;
    frame.shape_origin_earth_m = wrapped(shape_coordinates(anchor_earth_m, type_tile(CloudType::Cumulus)));
    frame.stratiform_origin_earth_m = wrapped(shape_coordinates(anchor_earth_m, type_tile(CloudType::Stratiform)));
    frame.ice_origin_earth_m = wrapped(shape_coordinates(anchor_earth_m, type_tile(CloudType::Cirrus)));
    return frame;
}

math::Vec3 cloud_frame_shape_texel(const CloudNoise& noise, const CloudShapeFrame& frame, CloudType type,
                                   const math::Vec3& position_earth_m) {
    const TypeTile tile = type_tile(type);
    const math::Vec3& origin = type == CloudType::Stratiform ? frame.stratiform_origin_earth_m
                               : type == CloudType::Cirrus   ? frame.ice_origin_earth_m
                                                             : frame.shape_origin_earth_m;
    const math::Vec3 q = origin + (shape_coordinates(position_earth_m, tile) -
                                   shape_coordinates(frame.anchor_earth_m, tile));
    return q * static_cast<double>(noise.shape_size);
}

CloudPointExtinction cloud_point_extinction_at_level(const CloudRenderField& field, const CloudNoise& noise,
                                                     const CloudShapeFrame& frame, const math::Vec3& position_earth_m,
                                                     double prefilter_level) {
    CloudPointExtinction out;
    double latitude_rad = 0.0, longitude_rad = 0.0;
    cloud_latitude_longitude(position_earth_m, latitude_rad, longitude_rad);
    const CloudFieldSample cell =
        cloud_render_field_at(field, latitude_rad, longitude_rad, cloud_model_height_m(position_earth_m));
    if (cell.fraction < constants::cloud_min_cover || !(cell.extinction_per_m > 0.0)) return out;
    const double level = std::clamp(prefilter_level, 0.0, static_cast<double>(cloud_shape_prefilter_levels));
    const bool prefiltered = level > 0.0;
    const double row = level * cloud_shape_prefilter_rows_per_level;
    const double threshold = prefiltered ? table_lerp(noise.prefilter_threshold, row, cell.fraction)
                                         : table_lerp(noise.coverage_threshold, 0.0, cell.fraction);
    const double m = prefiltered ? table_lerp(noise.prefilter_edge_mean, row, cell.fraction)
                                 : table_lerp(noise.edge_mean_factor, 0.0, cell.fraction);
    const CloudTypeWeights w = cloud_type_weights(cell.stratiform, cell.ice_share);
    const std::array<std::pair<CloudType, double>, 3> types = {
        std::pair{CloudType::Cumulus, w.cumulus}, std::pair{CloudType::Stratiform, w.stratiform},
        std::pair{CloudType::Cirrus, w.cirrus}};
    double ramp = 0.0;
    for (const auto& [type, weight] : types) {
        if (!(weight > 0.0)) continue;
        const math::Vec3 t = cloud_frame_shape_texel(noise, frame, type, position_earth_m);
        const double s = sample_shape_prefiltered(noise, 0, t, level);
        const double n = sample_shape_prefiltered(noise, 3, t, level);
        ramp += weight * cloud_edge_ramp((s - threshold) / std::max(cloud_edge_width_value(noise, n), 1.0e-6));
    }
    out.in_cloud_extinction_per_m = cell.extinction_per_m / std::max(m, edge_mean_floor);
    out.extinction_per_m = ramp > 0.0 ? out.in_cloud_extinction_per_m * ramp : 0.0;
    return out;
}

double cloud_point_extinction_per_m(const CloudRenderField& field, const CloudNoise& noise,
                                    const CloudShapeFrame& frame, const math::Vec3& position_earth_m,
                                    double footprint_m) {
    return cloud_point_extinction_at_level(field, noise, frame, position_earth_m,
                                           cloud_shape_prefilter_level(footprint_m, noise.shape_size))
        .extinction_per_m;
}

}  // namespace starsim::environment
