#pragma once
// Long-crested sea state of a drone ship's station: a JONSWAP spectrum (Hasselmann et al. 1973, DNV-RP-C205 sec. 3.5.5)
// discretised into regular linear (Airy) wave components with deep-water dispersion omega^2 = g k.
//
// Discretisation: omega in [0.5 omega_p, 3 omega_p] in EQUAL-WIDTH bins sampled at the bin centre (a fixed grid keeps the
// components' frequencies reproducible and independent of the sea's Hs; with 48 bins the spectral truncation is below
// 1 %); amplitude a_i = sqrt(2 S(omega_i) d_omega), all rescaled by one factor so the discrete variance sum a_i^2 / 2
// equals Hs^2 / 16 exactly. Phases are uniform in [0, 2 pi) from std::mt19937_64(seed), mapped by hand (portable: the
// standard distributions are implementation-defined). All waves travel towards the heading: the sea is deterministic.

#include <cstdint>
#include <vector>

namespace starsim::environment {

struct SeaStateConfig {
    double significant_wave_height_m = 0.0;  // Hs = 4 sqrt(m0); 0: calm
    double peak_period_s = 0.0;
    double heading_rad = 0.0;  // direction the waves travel towards, clockwise from north
    std::uint64_t seed = 1;
};

struct WaveComponent {
    double amplitude_m;
    double angular_frequency_radps;
    double wavenumber_radpm;
    double phase_rad;
};

// JONSWAP one-sided density S(omega) [m^2 s] for Hs and peak period Tp (gamma 3.3, DNV-RP-C205 normalisation).
double jonswap_spectral_density(double omega_radps, double hs_m, double tp_s);

class SeaState {
public:
    // throws std::invalid_argument on hs < 0, tp <= 0 with hs > 0, or component_count < 1
    explicit SeaState(const SeaStateConfig& config, int component_count = 48);

    const SeaStateConfig& config() const { return config_; }
    bool calm() const { return components_.empty(); }
    const std::vector<WaveComponent>& components() const { return components_; }
    double variance_m2() const;  // sum a_i^2 / 2

    // Elevation at (east, north) of a local tangent plane, t_s seconds after the sea's time origin.
    double elevation_m(double east_m, double north_m, double t_s) const;

private:
    SeaStateConfig config_;
    std::vector<WaveComponent> components_;
};

}  // namespace starsim::environment
