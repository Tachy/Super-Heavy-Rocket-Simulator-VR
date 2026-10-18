#include "starsim/aero/radiative_heating.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace starsim::aero {

namespace {

constexpr double min_speed_mps = 8000.0;  // [E] the table starts at 8 km/s (30-48 km; 9 km/s above); below it the term is 0 - small at entry heights (~2 % of the convective rate at 60 km / 6 m) but not in a dense, steep dive (48 km ~25 %), a step at 8 km/s

// Linear interpolation weight of x in [x0, x1] (clamped to [0, 1] when `clamp`).
double weight(double x, double x0, double x1, bool clamp) {
    const double w = (x - x0) / (x1 - x0);
    return clamp ? std::clamp(w, 0.0, 1.0) : w;
}

// Index i with xs[i] <= x < xs[i + 1] (first / last interval outside).
size_t interval(const std::vector<double>& xs, double x) {
    const auto it = std::upper_bound(xs.begin(), xs.end(), x);
    const size_t i = it == xs.begin() ? 0 : static_cast<size_t>(it - xs.begin()) - 1;
    return std::min(i, xs.size() - 2);
}

}  // namespace

RadiativeHeatingTable RadiativeHeatingTable::load(const std::filesystem::path& config_dir) {
    const auto path = config_dir / "aero" / "radiative_heating_earth.csv";
    std::ifstream in(path);
    if (!in) throw std::runtime_error("radiative heating table missing: " + path.string());
    // density -> velocity -> radius -> rate
    std::map<double, std::map<double, std::map<double, double>>> rows;
    std::string line;
    bool header = false;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (!header) {
            header = true;
            continue;
        }
        std::stringstream s(line);
        std::string cell;
        double v[5];
        for (double& x : v) {
            std::getline(s, cell, ',');
            x = std::stod(cell);
        }
        rows[v[1]][v[2] * 1000.0][v[3]] = v[4] * 1e6;
    }
    RadiativeHeatingTable t;
    for (const auto& [density, by_velocity] : rows) {
        Level level;
        level.ln_density = std::log(density);
        for (const auto& [velocity, by_radius] : by_velocity) {
            level.velocity_mps.push_back(velocity);
            if (level.ln_radius.empty()) {
                for (const auto& [radius, rate] : by_radius) level.ln_radius.push_back(std::log(radius));
            }
            std::vector<double> rates;
            for (const auto& [radius, rate] : by_radius) rates.push_back(std::log(rate));
            if (rates.size() != level.ln_radius.size()) throw std::runtime_error("radiative heating table: ragged radii");
            level.ln_rate.push_back(std::move(rates));
        }
        if (level.velocity_mps.size() < 2 || level.ln_radius.size() < 2) {
            throw std::runtime_error("radiative heating table: a level needs two velocities and two radii");
        }
        t.ln_densities_.push_back(level.ln_density);
        t.levels_.push_back(std::move(level));
    }
    if (t.levels_.size() < 2) throw std::runtime_error("radiative heating table: needs two density levels");
    return t;
}

double RadiativeHeatingTable::ln_rate_at(const Level& level, double speed_mps, double ln_radius) const {
    const size_t j = interval(level.ln_radius, ln_radius);
    const double wr = weight(ln_radius, level.ln_radius[j], level.ln_radius[j + 1], true);
    const auto at_velocity = [&](size_t i) {
        return level.ln_rate[i][j] + wr * (level.ln_rate[i][j + 1] - level.ln_rate[i][j]);
    };
    const size_t i = interval(level.velocity_mps, speed_mps);
    // below the lowest velocity: extrapolated with the lowest pair's slope; above the highest: clamped
    const bool above = speed_mps > level.velocity_mps.back();
    const double wv = above ? 1.0 : weight(speed_mps, level.velocity_mps[i], level.velocity_mps[i + 1], false);
    return at_velocity(i) + wv * (at_velocity(i + 1) - at_velocity(i));
}

double RadiativeHeatingTable::heat_rate_w_m2(double density_kg_m3, double speed_mps, double nose_radius_m) const {
    if (levels_.empty() || density_kg_m3 <= 0.0 || speed_mps < min_speed_mps || nose_radius_m <= 0.0) return 0.0;
    const double ln_rho = std::log(density_kg_m3);
    const size_t k = interval(ln_densities_, ln_rho);
    // above the densest level clamped; below the thinnest extrapolated (radiation falls fast with density)
    const bool denser = ln_rho > ln_densities_.back();
    const double w = denser ? 1.0 : weight(ln_rho, ln_densities_[k], ln_densities_[k + 1], false);
    const double ln_rn = std::log(nose_radius_m);
    const double a = ln_rate_at(levels_[k], speed_mps, ln_rn), b = ln_rate_at(levels_[k + 1], speed_mps, ln_rn);
    return std::exp(a + w * (b - a));
}

}  // namespace starsim::aero
