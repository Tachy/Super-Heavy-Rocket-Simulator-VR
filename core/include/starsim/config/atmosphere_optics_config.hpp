#pragma once

#include <filesystem>

namespace starsim::config {

// Optical inputs of the Earth atmosphere (config/atmosphere/earth_optics.toml).
struct AtmosphereOpticsConfig {
    double top_altitude_m = 0.0;                    // model top above the ellipsoid
    double ground_altitude_m = 0.0;                 // model ground (below the geoid minimum and every site)
    double aerosol_optical_depth_550nm = 0.0;       // vertical, ground (0 m) to top
    double aerosol_angstrom_exponent = 0.0;
    double aerosol_single_scattering_albedo = 0.0;  // in (0, 1]
    double aerosol_scale_height_m = 0.0;
    double mie_phase_g = 0.0;                       // in (-1, 1)
    double ozone_column_du = 0.0;
    double ground_albedo = 0.0;                     // in [0, 1]
    int scattering_orders = 0;                      // >= 1
    // Fidelity flag: the orders above scattering_orders as a geometric series (on) or truncated (Bruneton, off).
    bool multiple_scattering_tail = true;
};

// Throws ConfigError on a missing or invalid key.
AtmosphereOpticsConfig load_atmosphere_optics(const std::filesystem::path& toml_path);

}  // namespace starsim::config
