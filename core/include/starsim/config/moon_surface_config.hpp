#pragma once

#include <filesystem>
#include <string>

namespace starsim::config {

// Cumulative size-frequency distribution N(>D) per m^2; the form follows the source.
//   "power":       N = density_per_m2 * (D / reference_diameter_m)^-exponent
//   "exponential": N = density_per_m2 * exp(-exponent * D)   (exponent in 1/m)
// Valid over [min_diameter_m, max_diameter_m]; the formula itself is not clipped.
struct SizeFrequency {
    std::string form;
    double reference_diameter_m = 1.0, density_per_m2 = 0.0, exponent = 0.0;
    double min_diameter_m = 0.0, max_diameter_m = 0.0;
    double cumulative_density_per_m2(double diameter_m) const;
};

// Small craters below the LOLA resolution: equilibrium population, fresh or degraded per crater.
struct CraterPopulation {
    SizeFrequency sfd;
    double depth_to_diameter_fresh = 0.0, depth_to_diameter_degraded = 0.0;
    double rim_height_to_diameter = 0.0, degraded_fraction = 0.0;
};

// Self-affine regolith undulation: RMS height in a window W = rms_height_at_baseline_m * (W / baseline_m)^hurst_exponent.
// rms_height_at_baseline_m is the RESIDUAL the roughness generator draws (measured total minus the generated craters and
// rocks); rms_height_measured_total_m is the measured total of the natural surface, the target of validations of the
// summed surface (craters + rocks + roughness).
struct RegolithRoughness {
    double hurst_exponent = 0.0, rms_height_at_baseline_m = 0.0, baseline_m = 0.0;
    double rms_height_measured_total_m = 0.0;
};

// Discrete rocks: size law (shapes: RockTemplates), burial (share of the height below the surface),
// albedo relative to the regolith, local abundance field ("uniform" | "diviner") with a global scale.
struct RockPopulation {
    SizeFrequency sfd;
    double burial_fraction_mean = 0.0, burial_fraction_sd = 0.0, albedo_to_regolith = 0.0;
    std::string abundance_source;
    double abundance_scale = 1.0;
    // Area fraction of the SFD's rocks in [abundance_reference_min_diameter_m, sfd.max_diameter_m] (disc of diameter D):
    // a local map value (Diviner rock abundance) scales the population by map / abundance_reference_area_fraction.
    double abundance_reference_area_fraction = 0.0, abundance_reference_min_diameter_m = 0.0;
};

// Hapke (2012) reflectance parameters (double Henyey-Greenstein phase function, c > 0 backscattering).
struct HapkeParameters {
    double w = 0.0, b = 0.0, c = 0.0, bs0 = 0.0, hs = 0.0, bc0 = 0.0, hc = 0.0, theta_bar_rad = 0.0, porosity_k = 1.0;
};

struct MoonSurfaceConfig {
    CraterPopulation craters;
    RegolithRoughness roughness;
    RockPopulation rocks;
    HapkeParameters hapke;
};

// Reads config/moon_surface.toml. Throws ConfigError with a German message naming the missing or bad key.
MoonSurfaceConfig load_moon_surface_config(const std::filesystem::path& file);

}  // namespace starsim::config
