#pragma once

#include <filesystem>
#include <vector>

namespace starsim::aero {

// Equilibrium stagnation-point radiative heating of Earth entries [S: Sutton & Hartung, NASA TM-102652, 1990]: the
// tables of config/aero/radiative_heating_earth.csv, interpolated linearly in ln(q) over ln(density) (between the
// tabulated altitudes), velocity and ln(nose radius). Outside: zero below 8 km/s (radiation < 1 % of the convective
// rate there; keeps LEO entries untouched); below a density's lowest tabulated velocity ln(q) is extrapolated with the
// slope of its two lowest velocities; below the lowest density (84 km) ln(q) is extrapolated in ln(density) with the
// slope of the two lowest; nose radius and density clamped to the table's range above. Equilibrium only: the
// non-equilibrium radiation of the thin upper air is not modelled.
class RadiativeHeatingTable {
public:
    static RadiativeHeatingTable load(const std::filesystem::path& config_dir);
    double heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m) const;
    bool empty() const { return levels_.empty(); }

private:
    struct Level {  // one altitude table
        double ln_density = 0.0;
        std::vector<double> velocity_mps;          // ascending
        std::vector<double> ln_radius;             // ascending, same for all velocities
        std::vector<std::vector<double>> ln_rate;  // [velocity][radius], ln(W/m^2)
    };
    double ln_rate_at(const Level& level, double speed_mps, double ln_radius) const;
    std::vector<Level> levels_;         // ascending density
    std::vector<double> ln_densities_;  // levels_[k].ln_density
};

}  // namespace starsim::aero
