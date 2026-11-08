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
    // Aerosol phase function: w HG(g_forward) + (1 - w) HG(g_broad) (two-term Henyey-Greenstein, each normalised).
    double mie_phase_weight = 0.0;                  // w in [0, 1]
    double mie_phase_g_forward = 0.0;               // in (-1, 1)
    double mie_phase_g_broad = 0.0;                 // in (-1, 1)
    double ozone_column_du = 0.0;
    double ground_albedo = 0.0;                     // in [0, 1]
    int scattering_orders = 0;                      // >= 1
    // Fidelity flag: the orders above scattering_orders as a geometric series (on) or truncated (Bruneton, off).
    bool multiple_scattering_tail = true;
};

// The aerosol sets of earth_optics.toml ([aerosol.continental], [aerosol.maritime]); everything else is shared.
enum class AerosolType { continental, maritime };
inline constexpr AerosolType aerosol_types[] = {AerosolType::continental, AerosolType::maritime};
const char* aerosol_type_name(AerosolType type);

// The configuration with the given aerosol set. Throws ConfigError on a missing or invalid key.
AtmosphereOpticsConfig load_atmosphere_optics(const std::filesystem::path& toml_path,
                                              AerosolType aerosol = AerosolType::continental);

}  // namespace starsim::config
