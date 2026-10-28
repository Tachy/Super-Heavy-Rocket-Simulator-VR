#include "starsim/environment/cloud_volume_bake.hpp"

#include <algorithm>
#include <cmath>

#include "parallel_rows.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr double polar_stretch_squared =
    1.0 / ((1.0 - constants::wgs84_flattening) * (1.0 - constants::wgs84_flattening));

// Cells per parallel chunk of bake_cloud_cells.
constexpr int cells_per_chunk = 64;

}  // namespace

CloudCellBox cloud_cell_box(const math::Vec3& centre_earth_m, double horizontal_m, double vertical_m) {
    // The ellipsoid's normal (render axes, polar +Y) of the point's geodetic latitude.
    const math::Vec3 up =
        math::normalized(math::Vec3{centre_earth_m.x, centre_earth_m.y * polar_stretch_squared, centre_earth_m.z});
    math::Vec3 east = math::cross(math::Vec3{0.0, 1.0, 0.0}, up);
    east = math::norm(east) > 1e-12 ? math::normalized(east) : math::Vec3{1.0, 0.0, 0.0};
    const math::Vec3 north = math::normalized(math::cross(up, east));
    return {centre_earth_m, east * horizontal_m, north * horizontal_m, up * vertical_m};
}

CloudBakeTaps cloud_cell_bake_taps(double horizontal_m, double vertical_m, int shape_size) {
    CloudBakeTaps taps;
    // A cell's real edge length is off its level's nominal size by up to ~0.4 % (the AEQD's azimuthal scale, the
    // height): the counts are taken with the slack so such a cell keeps its level's taps.
    const auto count = [](double length_m, double spacing_m) {
        return static_cast<int>(std::ceil(length_m / spacing_m * (1.0 - cloud_bake_tap_count_slack)));
    };
    taps.vertical = std::max(cloud_bake_min_vertical_taps, count(vertical_m, cloud_bake_vertical_spacing_m));
    taps.horizontal = std::clamp(count(horizontal_m, cloud_bake_horizontal_spacing_m),
                                 cloud_bake_min_horizontal_taps,
                                 horizontal_m > cloud_bake_widest_cell_m ? cloud_bake_widest_cell_horizontal_taps
                                                                          : cloud_bake_max_horizontal_taps);
    const double texel_m = cloud_shape_tile_horizontal_m / shape_size;
    taps.prefilter_level = std::clamp(std::log2(horizontal_m / taps.horizontal / texel_m), 0.0,
                                      static_cast<double>(cloud_shape_prefilter_levels));
    return taps;
}

CloudCellBake bake_cloud_cell(const CloudRenderField& field, const CloudNoise& noise, const CloudShapeFrame& frame,
                              const CloudCellBox& box, const CloudBakeTaps& taps) {
    CloudCellBake out;
    const int nh = std::max(taps.horizontal, 1), nv = std::max(taps.vertical, 1);
    for (int k = 0; k < nv; ++k) {
        const double w = (k + 0.5) / nv - 0.5;
        for (int j = 0; j < nh; ++j) {
            const double v = (j + 0.5) / nh - 0.5;
            for (int i = 0; i < nh; ++i) {
                const double u = (i + 0.5) / nh - 0.5;
                const math::Vec3 p = box.centre_earth_m + box.edge_x_m * u + box.edge_y_m * v + box.edge_z_m * w;
                const CloudPointExtinction e =
                    cloud_point_extinction_at_level(field, noise, frame, p, taps.prefilter_level);
                out.mean_extinction_per_m += e.extinction_per_m;
                out.in_cloud_extinction_per_m += e.in_cloud_extinction_per_m;
            }
        }
    }
    const double n = static_cast<double>(nh) * nh * nv;
    out.mean_extinction_per_m /= n;
    out.in_cloud_extinction_per_m /= n;
    return out;
}

CloudCellBake bake_cloud_cell(const CloudRenderField& field, const CloudNoise& noise, const CloudShapeFrame& frame,
                              const CloudCellBox& box) {
    const double horizontal_m = std::max(math::norm(box.edge_x_m), math::norm(box.edge_y_m));
    return bake_cloud_cell(field, noise, frame, box,
                           cloud_cell_bake_taps(horizontal_m, math::norm(box.edge_z_m), noise.shape_size));
}

CloudCellBake bake_cloud_cell(const CloudRenderField& field, const CloudNoise& noise, const CloudShapeFrame& frame,
                              const math::Vec3& centre_earth_m, double cell_horizontal_m, double cell_vertical_m) {
    return bake_cloud_cell(field, noise, frame, cloud_cell_box(centre_earth_m, cell_horizontal_m, cell_vertical_m));
}

std::vector<CloudCellBake> bake_cloud_cells(const CloudRenderField& field, const CloudNoise& noise,
                                            const CloudShapeFrame& frame, std::span<const CloudCellBox> boxes) {
    std::vector<CloudCellBake> out(boxes.size());
    const int chunks = static_cast<int>((boxes.size() + cells_per_chunk - 1) / cells_per_chunk);
    detail::for_each_row(chunks, [&](int chunk) {
        const std::size_t first = static_cast<std::size_t>(chunk) * cells_per_chunk;
        const std::size_t last = std::min(boxes.size(), first + cells_per_chunk);
        for (std::size_t i = first; i < last; ++i) out[i] = bake_cloud_cell(field, noise, frame, boxes[i]);
    });
    return out;
}

}  // namespace starsim::environment
