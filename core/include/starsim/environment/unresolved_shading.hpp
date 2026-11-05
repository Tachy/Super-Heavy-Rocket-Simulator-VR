#pragma once

// Statistics of the lunar surface relief the renderer does not draw (Task 10 of the Moon procedural surface plan):
// the reference the Moon terrain shader (game/shaders/moon_detail.gdshaderinc) mirrors.
//
// Slope budget. Hapke's mean slope theta_bar (the globe's fit, config/moon_surface.toml [hapke]) stands for ALL the
// roughness below the fit's topography; for Hapke's Gaussian slope distribution the mean square slope (gradient
// magnitude) is S^2 = pi tan^2(theta_bar) (Hapke 1984: tan theta_bar = (2/pi) <tan theta>, |grad h| Rayleigh). A
// terrain node of vertex spacing L draws part of it as geometry: its vertices are the relief's L x L box mean
// (LunarSurfaceDetail band limit) and its normals central differences over 2 L. drawn(L) is that mesh's mean square
// slope of the statistical surface: per regolith octave (value noise, lattice spacing s) the exact spectral transfer
// (box mean x central difference) of the octave's spectrum, per crater size the same filter applied to the crater
// profile in real space. The node's residual theta_bar is S_res^2 = S^2 - drawn(L). The rocks' share is taken out of
// the budget: the globe fit's pixels contain the rocks, which the renderer models explicitly (albedo share, cast
// shadows, lit faces; geometry from Task 11). In Hapke's mean-tan form (tan theta_bar = (2/pi) <|grad h|>):
// <tan>_regolith = (<tan>_globe - A pi eta) / (1 - A), A = 1 - exp(-C_all) the rocks' area share at abundance 1
// (the globe's mean), pi eta the mean |grad h| over a half-spheroid's plan area (height eta D, plan radius D / 2: the
// integral of H r / (a^2 sqrt(1 - r^2 / a^2)) over the disc is pi^2 H a / 2); S^2 = pi ((2/pi) <tan>_regolith)^2.
//
// Statistical shadows (the Sun and the earthshine, each at its own elevation; Task 11 fix round I1): the drawn mesh
// relief finer than the horizon map's grid casts no shadow in the renderer (until the near-field shadow map), so its
// lit share is Smith's shadowing function G1 of a Gaussian surface with that slope variance, drawn(L) - drawn(H) for
// the horizon grid spacing H; the unresolved rocks cast shadows behind them (Boolean model of half-spheroids).
//
// Everything is a pure function of the configuration and the generator's octave definition; thread-safe.

#include <vector>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

class LunarSurfaceDetail;
class LunarRocks;

struct UnresolvedShadingModel {
    double globe_slope_variance = 0.0;  // pi tan^2(theta_bar) of the Hapke fit (rocks included)
    double rock_area_share = 0.0;       // A = 1 - exp(-C_all) at abundance 1
    double rock_mean_tan = 0.0;         // A pi eta: the rocks' part of the globe's <|grad h|>
    double total_slope_variance = 0.0;  // the regolith's budget: the globe's without the rocks (header comment)
    // Regolith roughness: the generator's octaves (lattice spacing, mean square slope of the octave's field).
    std::vector<double> regolith_octave_spacing_m;
    std::vector<double> regolith_octave_variance;
    // Craters: mean square slope of the craters with D in [D_lo, D_hi] = coef (D_hi^q - D_lo^q) / q, q = 2 - b
    // (q = 0: coef ln(D_hi / D_lo)); the fraction a mesh draws, crater_drawn_fraction(L / D), tabulated.
    double crater_variance_coefficient = 0.0;
    double crater_exponent = 0.0;
    double crater_min_diameter_m = 0.0;
    double crater_max_diameter_m = 0.0;
    std::vector<double> crater_fraction;  // at D / L = 2^(k / 2 - crater_fraction_lowest_octaves), k = 0..
    static constexpr int crater_fraction_lowest_octaves = 3;  // [D] D / L from 1/8 (below: 0) to 128 (above: the last)
    static constexpr int crater_fraction_samples = 21;
    // drawn(L) at L = drawn_table_min_spacing_m x 2^(k / drawn_table_per_octave), linear in log2 L between, 0 beyond.
    std::vector<double> drawn_table;
    static constexpr double drawn_table_min_spacing_m = 1.0e-3;  // [D] below the finest node spacing (1 cm)
    static constexpr int drawn_table_per_octave = 4;
    static constexpr int drawn_table_samples = 97;  // 1 mm .. 16.8 km
    // Rocks (abundance factor 1; absent: rock_cover_coefficient 0).
    double rock_cover_coefficient = 0.0;  // cover of [D_lo, D_hi] = coef * (D_hi^q - D_lo^q), q = rock_cover_exponent
    double rock_cover_exponent = 0.0;     // 2 - b
    double rock_min_diameter_m = 0.0;
    double rock_max_diameter_m = 0.0;
    double rock_exposed_height_per_diameter = 0.0;  // eta: mean height of the part above the ground / D
    double rock_reflectance_ratio = 1.0;            // Hapke I/F(albedo_to_regolith x w) / I/F(w), standard geometry
    std::vector<double> rock_lit_gain;              // rock_lit_gain() at elevations k x lit_gain_step_rad, k = 0..N-1
    static constexpr int lit_gain_samples = 46;
    static constexpr double lit_gain_step_rad = 2.0 * constants::deg_to_rad;  // [D] 2 deg: 0 .. 90 deg, linear between
};

// Builds the model from the generator (octave spacings and amplitude scale), its configuration (craters, Hapke) and
// the rocks (null: no rock term). The rocks' eta is measured on generated rocks (rock_top at the base, a fixed set
// of cells per octave). Power-law size distributions only (std::invalid_argument otherwise).
UnresolvedShadingModel make_unresolved_shading_model(const LunarSurfaceDetail& detail, const LunarRocks* rocks);

// Hapke mean slope <-> mean square slope magnitude: S^2 = pi tan^2(theta_bar).
double slope_variance_from_theta_bar(double theta_bar_rad);
double theta_bar_from_slope_variance(double slope_variance);

// Share of a value-noise octave's (lattice spacing s, the generator's quintic fade) mean square slope that the mesh of
// spacing L = ratio x s draws (box mean over L, central differences over 2 L): 1 at ratio 0, 0.083 at ratio 1.
double regolith_drawn_fraction(double spacing_over_lattice);
// The same for a crater of the configured profile (fresh / degraded mix by their slope integrals), real-space mesh
// sampling over a grid of lattice offsets; L = ratio x D.
double crater_drawn_fraction(const config::CraterPopulation& craters, double spacing_over_diameter);
// The model's tabulated crater_drawn_fraction at D / L (half-octave samples, linear between; 0 below D / L = 1/8, the
// last sample above D / L = 128), as the shader reads it.
double crater_drawn_fraction_table(const UnresolvedShadingModel& m, double diameter_over_spacing);

// Mean square slope the mesh of spacing L draws of the generated surface: the regolith octaves with lattice spacing up
// to regolith_top_spacing_m and the craters up to crater_top_diameter_m (direct sum; the table below uses the
// generator's full ranges, the statistical surface everywhere).
double drawn_slope_variance_direct(const UnresolvedShadingModel& m, double spacing_m, double regolith_top_spacing_m,
                                   double crater_top_diameter_m);
// drawn(L) from the table (as the shader reads it); drawn(infinity) = 0.
double drawn_slope_variance(const UnresolvedShadingModel& m, double spacing_m);
// Residual Hapke mean slope of a node with vertex spacing L: theta_bar(S^2 - drawn(L)).
double residual_theta_bar(const UnresolvedShadingModel& m, double spacing_m);
// Slope variance of the drawn relief that casts no shadow (finer than the horizon map's grid spacing H):
// max(drawn(L) - drawn(H), 0).
double unshadowed_slope_variance(const UnresolvedShadingModel& m, double spacing_m, double horizon_spacing_m);

// Smith's (1967) shadowing function G1 = 1 / (1 + Lambda) of a Gaussian surface with mean square slope (gradient
// magnitude) slope_variance for a light at elevation with tangent tan_elevation over the mean plane (0 at elevation
// <= 0, 1 for a smooth surface). Lambda(a) = (erf(a) - 1) / 2 + exp(-a^2) / (2 a sqrt(pi)), a = tan / sqrt(variance).
double smith_lit_fraction(double tan_elevation, double slope_variance);

// Rock cover (area share, abundance 1) of diameters [d_lo, d_hi] (clipped to the law's range).
double rock_cover(const UnresolvedShadingModel& m, double d_lo_m, double d_hi_m);
// Cast shadow on the ground outside the rock per rock plan-view area, for a half-spheroid of plan radius D/2 and
// height eta D under a Sun at elevation e: (sqrt(1 + 4 eta^2 cot^2 e) - 1) / 2 (exact projection).
double rock_shadow_per_cover(double eta, double sin_elevation);
// Mean single-scattering (Lommel-Seeliger) radiance of the half-spheroid seen from the zenith, per plan-view area,
// over the flat ground's under the same Sun (elevation >= lit_gain_floor_rad: below it the floor's value).
double rock_lit_gain(double eta, double elevation_rad);
inline constexpr double lit_gain_floor_rad = 1.0 * constants::deg_to_rad;  // [E] the gain grows as 1 / sin e
// The model's tabulated gain (linear between the samples), as the shader reads it.
double rock_lit_gain_table(const UnresolvedShadingModel& m, double elevation_rad);

// Statistical rock cover (abundance 1) at the geometry threshold d (Task 11's growth band): rocks below d count fully,
// rocks in [d, 2 d] with weight 1 - log2(D / d) (the drawn rock rises out of the ground over the band, exposed height
// share log2(D / d)), rocks above 2 d not at all (drawn). Closed form of the band integral over the power law; d =
// infinity: rock_cover over the whole law.
double rock_statistical_cover(const UnresolvedShadingModel& m, double geometry_min_diameter_m);

// Light factors on the regolith's Hapke radiance of a ground fragment for the unresolved surface: rocks below the
// geometry threshold geometry_min_diameter_m (infinity: every rock statistical; growth band as in
// rock_statistical_cover) at the local abundance factor, the drawn mesh relief finer than the horizon grid (Smith), the
// Sun and the Earth (the earthshine's source: a directional light like the Sun, disc ~1.9 deg) each at its own
// macroscopic elevation e (over the terrain horizon). Per light:
// f(e) = G1(e) x [exp(-C (1 + sigma(e)) - C_g sigma'(e)) + (1 - exp(-C)) rho G(e)] / N (0 for e <= 0),
// C = abundance x rock_statistical_cover(geometry_min), C_g = abundance x rock_cover(all) - C (the drawn rocks' share),
// N = exp(-C_all) + (1 - exp(-C_all)) rho (the albedo mosaic is the mean with every rock). sigma' = sigma while the
// drawn rocks cast no shadow (geometry_shadows_statistical; until the near-field shadow map, Task 12), else 0. The
// factors hold for the ground the drawn rocks leave visible (exp(-C_g) of it): a drawn rock's own mean radiance per
// plan area is rho G1(e) G(e) / N in the model for either light (moon_rock.gdshader draws it).
struct UnresolvedLightFactors {
    double sun = 1.0;
    double earth = 1.0;
};
UnresolvedLightFactors unresolved_light_factors(const UnresolvedShadingModel& m, double abundance,
                                                double geometry_min_diameter_m, double spacing_m,
                                                double horizon_spacing_m, double sun_elevation_rad,
                                                double earth_elevation_rad,
                                                bool geometry_shadows_statistical = true);

}  // namespace starsim::environment
