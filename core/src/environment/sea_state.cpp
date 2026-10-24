#include "starsim/environment/sea_state.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {
constexpr double band_low = 0.5;   // lowest omega / omega_p of the discretisation
constexpr double band_high = 3.0;  // highest omega / omega_p

// uniform [0, 2 pi) from the top 53 bits of a raw draw
double phase_from(std::uint64_t raw) {
    return constants::two_pi * static_cast<double>(raw >> 11) * (1.0 / 9007199254740992.0);
}
}  // namespace

double jonswap_spectral_density(double omega_radps, double hs_m, double tp_s) {
    if (omega_radps <= 0.0 || hs_m <= 0.0 || tp_s <= 0.0) return 0.0;
    const double wp = constants::two_pi / tp_s;
    const double gamma = constants::jonswap_gamma;
    const double ratio = wp / omega_radps;
    const double sigma = omega_radps <= wp ? constants::jonswap_sigma_low : constants::jonswap_sigma_high;
    const double r = std::exp(-0.5 * std::pow((omega_radps - wp) / (sigma * wp), 2));
    // normalising factor A_gamma = 1 - 0.287 ln(gamma) [S: DNV-RP-C205 3.5.5.4]
    const double a_gamma = 1.0 - 0.287 * std::log(gamma);
    return a_gamma * (5.0 / 16.0) * hs_m * hs_m * std::pow(wp, 4) * std::pow(omega_radps, -5) *
           std::exp(-1.25 * std::pow(ratio, 4)) * std::pow(gamma, r);
}

SeaState::SeaState(const SeaStateConfig& config, int component_count) : config_(config) {
    if (config.significant_wave_height_m < 0.0) throw std::invalid_argument("SeaState: negative wave height");
    if (config.significant_wave_height_m == 0.0) return;
    if (config.peak_period_s <= 0.0) throw std::invalid_argument("SeaState: peak period must be positive");
    if (component_count < 1) throw std::invalid_argument("SeaState: component count must be positive");

    const double hs = config.significant_wave_height_m;
    const double wp = constants::two_pi / config.peak_period_s;
    const double dw = (band_high - band_low) * wp / component_count;
    std::mt19937_64 rng(config.seed);
    double variance = 0.0;
    for (int i = 0; i < component_count; ++i) {
        const double w = (band_low * wp) + (i + 0.5) * dw;
        const double a = std::sqrt(2.0 * jonswap_spectral_density(w, hs, config.peak_period_s) * dw);
        components_.push_back({a, w, w * w / constants::standard_gravity_mps2, phase_from(rng())});
        variance += 0.5 * a * a;
    }
    const double scale = std::sqrt(hs * hs / 16.0 / variance);
    for (auto& c : components_) c.amplitude_m *= scale;
}

double SeaState::variance_m2() const {
    double v = 0.0;
    for (const auto& c : components_) v += 0.5 * c.amplitude_m * c.amplitude_m;
    return v;
}

double SeaState::elevation_m(double east_m, double north_m, double t_s) const {
    const double along = east_m * std::sin(config_.heading_rad) + north_m * std::cos(config_.heading_rad);
    double z = 0.0;
    for (const auto& c : components_)
        z += c.amplitude_m * std::cos(c.wavenumber_radpm * along - c.angular_frequency_radps * t_s + c.phase_rad);
    return z;
}

}  // namespace starsim::environment
