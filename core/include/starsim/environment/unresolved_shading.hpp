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
// the horizon grid spacing H; the unresolved rocks cast shadows behind them (Boolean model; the shadow per cover and the
// lit gain measured on the generated angular rocks, template_rock_light_tables).
//
// Everything is a pure function of the configuration and the generator's octave definition; thread-safe.

#include <cstdint>
#include <filesystem>
#include <vector>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

class LunarSurfaceDetail;
class LunarRocks;
struct TemplateMesh;

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
    // G(e) and sigma(e) of the generated rocks (template_rock_light_tables: the angular templates, Task 11 fix C R1) at
    // elevations k x lit_gain_step_rad, k = 0..N-1.
    std::vector<double> rock_lit_gain;
    std::vector<double> rock_shadow;
    static constexpr int lit_gain_samples = 46;
    static constexpr double lit_gain_step_rad = 2.0 * constants::deg_to_rad;  // [D] 2 deg: 0 .. 90 deg, linear between
    // View-dependent rock terms (Task 12, review T11-C5 R1; template_rock_view_tables): at view elevation index kv
    // (kv x view_elevation_step_rad, floor lit_gain_floor_rad), light-view azimuth difference index ka (ka x
    // view_azimuth_step_rad) and light elevation index k (the gain grid), entry ((kv x view_azimuth_samples + ka) x
    // lit_gain_samples + k): the ground hidden from the eye outside the light's shadow per cover (X) and the drawn
    // shape's seen-radiance gain over its zenith-view value (G_v / G). Empty: nadir view only.
    std::vector<double> rock_view_excess;
    std::vector<double> rock_view_gain_ratio;
    static constexpr int view_elevation_samples = 46;
    static constexpr double view_elevation_step_rad = 2.0 * constants::deg_to_rad;  // [D] 0 .. 90 deg (X changes fast at a low eye)
    static constexpr int view_azimuth_samples = 7;
    static constexpr double view_azimuth_step_rad = 30.0 * constants::deg_to_rad;  // [D] 0 .. 180 deg
    // Hapke directional-hemispherical albedo A_h(mu0) of the regolith (the globe's parameters and theta_bar) at
    // elevations k x lit_gain_step_rad (mu0 = sin e): the terrain inter-reflection fill (Task 12).
    std::vector<double> hemispherical_albedo;
};

// Builds the model from the generator (octave spacings and amplitude scale), its configuration (craters, Hapke) and
// the rocks (null: no rock term). The rocks' eta is measured on generated rocks (rock_top at the base, a fixed set
// of cells per octave). Power-law size distributions only (std::invalid_argument otherwise).
// cache_dir non-empty: the rock view tables' file cache (template_rock_view_tables) lives there.
UnresolvedShadingModel make_unresolved_shading_model(const LunarSurfaceDetail& detail, const LunarRocks* rocks,
                                                     const std::filesystem::path& cache_dir = {});

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
// The lit share of the surface the eye sees (Task 12): Smith's height-direction-correlated masking-shadowing over the
// masking, G2(l, v) / G1(v), G2 = 1 / (1 + max(Lambda_l, Lambda_v) + lambda(phi) min(Lambda_l, Lambda_v)) with
// lambda(phi) = 4.41 phi / (4.41 phi + 1) (Heitz 2014, JCGT 3(2), eq. 103, after van Ginneken et al. 1998), phi the
// azimuth difference between the light and the eye: from the light's own direction the eye sees no shadow (shadow
// hiding), from the zenith (tan_view = infinity, Lambda_v = 0) it is G1(l).
double smith_visible_lit_fraction(double tan_light, double tan_view, double azimuth_difference_rad,
                                  double slope_variance);

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
// The model's tabulated cast shadow per cover sigma(e) (linear between the samples), as the shader reads it.
double rock_shadow_table(const UnresolvedShadingModel& m, double elevation_rad);

// One column of a rock's exposed top on a plan grid: position (rock frame, plan), height of the top above the ground
// cut and the unit surface normal there.
struct RockColumn {
    double x = 0.0, y = 0.0, height = 0.0;
    math::Vec3 normal{0.0, 0.0, 1.0};
};
// Lit gain G(e) and cast shadow per cover sigma(e) of one rock given by its exposed top columns (plan grid of spacing
// cell), averaged over rock_light_azimuths light azimuths, at the model's table elevations k x lit_gain_step_rad
// (clamped to >= lit_gain_floor_rad). G is rock_lit_gain's quantity (Lommel-Seeliger seen from the zenith, per plan
// area, over the flat ground's); sigma the ground in the cast shadow outside the rock's plan view per plan-view area:
// per grid line along the light's azimuth the convex rock's shadow interval beyond its footprint (exact for a convex
// rock up to the grid).
struct RockLightTables {
    std::vector<double> gain;
    std::vector<double> shadow;
    double mean_tan = 0.0;  // mean |grad h| of the exposed top per plan area (the rocks' share of Hapke's mean slope)
};
inline constexpr int rock_light_azimuths = 8;  // [D] the rocks' yaw is uniform; 8 azimuths: < 1 % against 32
RockLightTables rock_light_tables(const std::vector<RockColumn>& columns, double cell);
// The same averaged over the generator's rocks: the 32 angular templates (RockTemplates) at the burial law
// (clamp(mean + sd N, 0, LunarRocks::max_burial_fraction), Gauss quadrature in N), each rock's plan area equal (the
// generator scales each template to the area-equivalent D). Cached per (mean, sd); thread-safe.
RockLightTables template_rock_light_tables(double burial_mean, double burial_sd);

// View-dependent terms of one convex rock (Task 12): its exposed part (above the ground cut z = cut of the template
// mesh's convex hull) seen from view elevation e_v with the light at elevation e_l and the light-view azimuth difference
// dphi. The ground a convex body hides from a direction is the body's projection along it onto the ground (P_v; the
// light's shadow region incl. the footprint is P_l): X = (|P_v| - |P_l n P_v|) / A (the ground hidden from the eye
// outside the light's shadow, per plan area A = |P(zenith)|); the seen radiance gain is the Lommel-Seeliger radiance of
// the faces seen (projected-area weighted) over the flat ground's at (e_l, e_v), times |P_v| / A (the rock's share of
// the image per plan area). Averaged over yaws. Tables in the model's layout (rock_view_excess):
// excess = X, flux = gain x |P_v| / A, hidden = |P_v| / A - 1 (= sigma(e_v) of the polygon), flux_nadir = the zenith
// view's gain per light elevation.
struct RockViewTables {
    std::vector<double> excess;
    std::vector<double> flux;
    std::vector<double> hidden;
    std::vector<double> flux_nadir;
};
inline constexpr int rock_view_yaws = 2;  // [D] yaws per rock (the 32 templates' own orientations are random)
RockViewTables rock_view_tables(const TemplateMesh& mesh, double cut);
// The generator's mean over the 32 templates (LOD 2 meshes) and the burial law (as template_rock_light_tables); the
// excess as the template mean, the gain ratio as mean(flux) / mean(1 + hidden) over the zenith view's gain. Cached,
// thread-safe.
struct RockViewModel {
    std::vector<double> excess;
    std::vector<double> gain_ratio;
};
// cache_file non-empty (review I5 of Task 12: the tables cost ~8 s of the model build): read from that file when its
// key matches (rock_view_cache_key), else computed and written there (a failed write is ignored). The tables are
// deterministic, so the cached ones are the computed ones bit for bit.
RockViewModel template_rock_view_tables(double burial_mean, double burial_sd,
                                        const std::filesystem::path& cache_file = {});
// The file cache's key: FNV-1a over a format version, the grid sizes, the burial law and the templates' LOD-2 meshes
// (vertices and indices) the tables are built from.
std::uint64_t rock_view_cache_key(double burial_mean, double burial_sd);
// The cache file's name in a cache directory.
inline constexpr const char* rock_view_cache_name = "rock_view_tables.bin";
// The cache file's read (false: missing, another key or a broken file; out untouched) and write (a unique temporary
// renamed over the file; failures ignored).
bool read_rock_view_cache(const std::filesystem::path& file, std::uint64_t key, RockViewModel& out);
void write_rock_view_cache(const std::filesystem::path& file, std::uint64_t key, const RockViewModel& m);
// Model table lookups (trilinear; elevations clamped to [floor, 90 deg], dphi folded to [0, pi]).
double rock_view_excess_table(const UnresolvedShadingModel& m, double light_elevation_rad, double view_elevation_rad,
                              double azimuth_difference_rad);
double rock_view_gain_ratio_table(const UnresolvedShadingModel& m, double light_elevation_rad,
                                  double view_elevation_rad, double azimuth_difference_rad);

// Hapke directional-hemispherical albedo A_h(mu0) = 1 / (pi mu0) Int I/F(mu0, mu, g) mu dOmega (Gauss-Legendre in mu,
// midpoint in azimuth; the reflected over the incident flux) with mean slope theta_bar.
double hapke_hemispherical_albedo(const config::HapkeParameters& p, double mu0, double theta_bar_rad);
// The model's tabulated A_h at elevation e (mu0 = sin e; linear between the samples, the lit-gain grid).
double hemispherical_albedo_table(const UnresolvedShadingModel& m, double elevation_rad);

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

// The same seen from an eye at view elevation e_v over the mean plane (Task 12, review T11-C5 R1: an oblique eye sees
// the rocks hide the ground behind them and their lit / dark sides), with the near-field shadow map (Task 12) for the
// Sun. Per light at elevation e and light-eye azimuth difference dphi:
// f(e) = G1(e) x [exp(-C (1 + sigma(e) + X) - C_g' max(sigma(e) - sigma(e_v) + X, 0))
//                 + (1 - exp(-C (1 + sigma(e_v)))) rho G(e) R(e, e_v, dphi)] / N,
// X = rock_view_excess_table (the ground hidden from the eye outside the light's shadow, per cover; 0 from the
// zenith), R = rock_view_gain_ratio_table, sigma(e_v) the hidden ground per cover (the shadow law at the view's
// elevation). C_g' = C_g where the drawn rocks' shadows are statistical for that light, else 0 (their visible shadow,
// sigma - omega, omega = sigma(e_v) - X the overlap of the light's and the view's hidden regions). From the zenith
// (e_v = 90 deg) it is the nadir law above. sun_shadow_map: the near-field shadow map draws the Sun's shadows of the drawn
// mesh relief down to shadow_map_resolved_m (G1 of the finer drawn relief only, drawn(L) - drawn(T)) and of the drawn
// rocks from shadow_caster_factor x d (C_g' their statistical share below it); the earthshine keeps both statistical.
// [E, measured: Task 12 report] The drawn rocks cast the near-field shadow from this many times the geometry threshold
// diameter d (their own growth band [F d, 2 F d] in the shadow pass): a rock of ~1.5 px is ~1-2 shadow-map texels wide,
// its shadow unresolved, and every drawn rock in the shadow pass cost +78 ms in the stereo mock in a boulder field. The
// drawn rocks below F d keep their Sun shadows statistical (C_g' = abundance x rock_statistical_cover(F d) - C).
inline constexpr double shadow_caster_factor = 4.0;

struct UnresolvedView {
    double elevation_rad = 0.5 * constants::pi;  // the eye over the mean plane
    double sun_azimuth_difference_rad = 0.0;      // between the horizontal directions to the Sun and to the eye
    double earth_azimuth_difference_rad = 0.0;
    bool sun_shadow_map = false;
    // With the map: the smallest relief wavelength the map resolves (2 shadow-map texels, the sampling's Nyquist
    // scale; 0: all); the drawn relief finer than it keeps Smith's G1 for the Sun (drawn(L) - drawn(T)).
    double shadow_map_resolved_m = 0.0;
};
UnresolvedLightFactors unresolved_light_factors(const UnresolvedShadingModel& m, double abundance,
                                                double geometry_min_diameter_m, double spacing_m,
                                                double horizon_spacing_m, double sun_elevation_rad,
                                                double earth_elevation_rad, bool geometry_shadows_statistical,
                                                const UnresolvedView& view);

}  // namespace starsim::environment
