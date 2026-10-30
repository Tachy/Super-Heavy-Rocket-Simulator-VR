#pragma once

// CPU mirror of the GPU's cloud point model (world-volume plan Task 4): what game/shaders/clouds.gdshaderinc
// cloud_field / cloud_extinction_in_cell / cloud_extinction_prefiltered compute at one point of ONE snapshot, without
// the detail levels (the world volume bakes the shapes; the detail is added per sample on top of it) - change both
// together. The reference of the world volume's cell bake (cloud_volume_bake.hpp), which the GPU bake is checked
// against.
//
// Positions are Earth-local render axes, metres (ITRF (x, y, z) -> (y, z, x), polar axis +Y; the shader's and the
// former world volume's axes). As the shader: the height is the model-space height |M p| - a (the WGS84 ellipsoid
// stretched to the sphere of radius a along the polar axis), the latitude geodetic from tan(phi) = (a / b)^2 y / s, the
// longitude atan2(x, z).
//
// Field (cloud_render_texels.hpp, as uploaded): RGBA8 texels filtered bilinearly in (longitude, latitude) at the texel
// centres (column c at longitude c x 360 / columns, row r at latitude 90 - r x 180 / (rows - 1) deg), the two layer
// centres around a point joined by the vertical profile rule (cloud_profile_value); the extinction is the 16-bit G/B
// number filtered linearly; the ice share (R8) trilinear in the layer coordinate; the flat base (RG32F) per column with
// nearest fetches, the four columns' cut factors mixed bilinearly; the stratiform weight from the centres k, k + 1 and
// k + 2 (cloud_stratiform_weight_between). Zero above weather_top_m.
//
// Point extinction (no detail): the coverage threshold t and edge mean factor m of the cloud fraction (tables linear in
// the fraction and in the row, as the GPU's linear filter), per cloud type present its shape value s and width noise n
// at the type's coordinates, ramp = sum_types w_type smoothstep((s - t) / w(n)), extinction = grid mean / max(m, 1e-4)
// x ramp. At prefilter level lambda > 0 the shape comes from its box-filtered mips and t, m from the prefilter tables
// (cloud_noise.hpp, "Shape prefilter"); lambda = 0 is the unfiltered shape with the state-0 tables. The pixel footprint
// form takes lambda = cloud_shape_prefilter_level(footprint) and no sub-pixel offset (the ray's offset of the GPU is a
// per-pixel jitter; its expectation is the unshifted point).
//
// Shape coordinates (the shader's cloud_shape_coordinates_for): q(p) = O + (p - anchor) / S_H + k (up(p) h(p) -
// up(anchor) h(anchor)), k = 1 / S_V - 1 / S_H, up = p / |p|, with the origin O = the anchor's own coordinates wrapped
// to [0, 1) (CloudLayer._shape_origin: the float64 values the GPU gets); texel = q x shape_size.

#include <cstdint>
#include <vector>

#include "starsim/environment/cloud_noise.hpp"
#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// One snapshot's render texels as the GPU gets them (CloudLayer's upload).
struct CloudRenderField {
    int rows = 0, columns = 0;                // the snapshot's grid; weather_layer_count layers
    std::vector<std::uint8_t> field_rgba;     // cloud_render_texels (layer-major RGBA8)
    std::vector<std::uint8_t> ice;            // cloud_render_ice_texels (R8, same layout)
    std::vector<float> base;                  // cloud_render_base_texels (RG32F, rows x columns)
};

CloudRenderField make_cloud_render_field(const WeatherSnapshot& snapshot);

// The field at a point (the shader's CloudCell without the column terms).
struct CloudFieldSample {
    double fraction = 0.0;          // cloud fraction (after the flat base's cut)
    double extinction_per_m = 0.0;  // grid-mean extinction (after the cut and its scale)
    double ice_share = 0.0;         // ice share of the extinction
    double stratiform = 0.0;        // stratiform weight (cloud_stratiform_weight_between)
};

CloudFieldSample cloud_render_field_at(const CloudRenderField& field, double latitude_rad, double longitude_rad,
                                       double height_m);

// The shader's height |M p| - a and (geodetic latitude, longitude) of an Earth-local render-axes point.
double cloud_model_height_m(const math::Vec3& position_earth_m);
void cloud_latitude_longitude(const math::Vec3& position_earth_m, double& latitude_rad, double& longitude_rad);

// The shape noise origins of the three cloud types (header): the coordinates of anchor_earth_m, wrapped.
struct CloudShapeFrame {
    math::Vec3 anchor_earth_m;             // the Earth-local point the origins belong to
    math::Vec3 shape_origin_earth_m;       // cumulus (cloud_noise_origin)
    math::Vec3 stratiform_origin_earth_m;  // stratiform (cloud_stratiform_origin)
    math::Vec3 ice_origin_earth_m;         // cirrus (cloud_ice_origin)
};

CloudShapeFrame make_cloud_shape_frame(const math::Vec3& anchor_earth_m);

// Shape texel coordinates of the type at a point (header).
math::Vec3 cloud_frame_shape_texel(const CloudNoise& noise, const CloudShapeFrame& frame, CloudType type,
                                   const math::Vec3& position_earth_m);

// The point model at an explicit prefilter level (header): the extinction and the envelope's in-cloud extinction there
// (grid mean / max(m, 1e-4): the extinction where the ramp is 1; 0 where the fraction is below cloud_min_cover or the
// grid mean is 0). extinction <= in_cloud always (the type weights sum to 1, each ramp <= 1).
struct CloudPointExtinction {
    double extinction_per_m = 0.0;
    double in_cloud_extinction_per_m = 0.0;
};

CloudPointExtinction cloud_point_extinction_at_level(const CloudRenderField& field, const CloudNoise& noise,
                                                     const CloudShapeFrame& frame, const math::Vec3& position_earth_m,
                                                     double prefilter_level);

// The point model at a pixel footprint (header: lambda = cloud_shape_prefilter_level(footprint_m)).
double cloud_point_extinction_per_m(const CloudRenderField& field, const CloudNoise& noise,
                                    const CloudShapeFrame& frame, const math::Vec3& position_earth_m,
                                    double footprint_m);

}  // namespace starsim::environment
