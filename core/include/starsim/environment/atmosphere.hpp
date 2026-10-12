#pragma once

namespace starsim::environment {

struct AtmosphereState {
    double density_kg_m3 = 0.0;
    double pressure_pa = 0.0;
    double temperature_k = 0.0;
    double speed_of_sound_mps = 0.0;
    double mean_free_path_m = 0.0;  // infinity in vacuum
};

class Atmosphere {
public:
    virtual ~Atmosphere() = default;

    // Properties at a geometric altitude above the reference ellipsoid (WGS84).
    virtual AtmosphereState at(double altitude_m) const = 0;
};

// U.S. Standard Atmosphere 1976.
// 0-86 km: exact layer equations (geopotential altitude, molecular-scale temperature).
// 86-1000 km: log-linear interpolation of the tabulated US76 densities and temperatures; pressure from
// the ideal gas law with sea-level molar mass (pressure there only matters as ~0 for engine back-pressure).
// Above 1000 km: vacuum. Below -5 km: clamped.
class Us76Atmosphere final : public Atmosphere {
public:
    AtmosphereState at(double altitude_m) const override;
};

}  // namespace starsim::environment
