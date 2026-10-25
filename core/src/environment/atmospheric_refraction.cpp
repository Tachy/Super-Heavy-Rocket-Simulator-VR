#include "starsim/environment/atmospheric_refraction.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr double half_pi = constants::pi / 2.0;
// Simpson intervals of one ray's zenith-angle integral (even). The integrand spans a few degrees about the tangent
// point; 128 intervals over the at most ~25 deg of a limb ray (the table builds in ~1 ms). US76's lapse-rate breaks
// (tropopause etc.) make the integrand jump, so the rule converges as O(step) there: within 0.5 % of the bending
// (< 10 arcsec at the horizon), oscillating with the step count.
constexpr int simpson_intervals = 128;
constexpr int radius_newton_steps = 3;  // from r = p / sin z, n - 1 < 3e-4 off: quadratic convergence

}  // namespace

RefractionProfile::RefractionProfile(const Atmosphere& atmosphere, double top_altitude_m, double step_m)
    : top_altitude_m_(top_altitude_m), step_m_(step_m) {
    // Gladstone-Dale: n - 1 = K rho with K from standard air (Edlén's standard air has the US76 sea-level density).
    const double per_density = constants::standard_air_refractivity_550nm / constants::standard_sea_level_density_kg_m3;
    const auto count = static_cast<std::size_t>(std::ceil(top_altitude_m / step_m)) + 1;
    refractivity_.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        refractivity_[i] = per_density * atmosphere.at(static_cast<double>(i) * step_m).density_kg_m3;
    }
}

double RefractionProfile::refractivity(double altitude_m) const {
    if (altitude_m >= top_altitude_m_) return 0.0;
    const double x = std::max(altitude_m, 0.0) / step_m_;
    const auto i = std::min(static_cast<std::size_t>(x), refractivity_.size() - 2);
    const double f = x - static_cast<double>(i);
    return refractivity_[i] + f * (refractivity_[i + 1] - refractivity_[i]);
}

double RefractionProfile::refractivity_gradient_per_m(double altitude_m) const {
    if (altitude_m >= top_altitude_m_) return 0.0;
    // Central differences at the nodes, interpolated linearly: a continuous gradient for the ray integral.
    const auto node_gradient = [&](std::size_t i) {
        const std::size_t lo = i == 0 ? 0 : i - 1;
        const std::size_t hi = std::min(i + 1, refractivity_.size() - 1);
        return (refractivity_[hi] - refractivity_[lo]) / (static_cast<double>(hi - lo) * step_m_);
    };
    const double x = std::max(altitude_m, 0.0) / step_m_;
    const auto i = std::min(static_cast<std::size_t>(x), refractivity_.size() - 2);
    const double f = x - static_cast<double>(i);
    return node_gradient(i) + f * (node_gradient(i + 1) - node_gradient(i));
}

std::optional<double> refraction_bending_rad(const RefractionProfile& profile, double surface_radius_m,
                                             double observer_altitude_m, double apparent_elevation_rad) {
    const double altitude = std::max(observer_altitude_m, 0.0);
    const double top_radius = surface_radius_m + profile.top_altitude_m();
    const double z0 = half_pi - apparent_elevation_rad;
    const double p = (1.0 + profile.refractivity(altitude)) * (surface_radius_m + altitude) * std::sin(z0);
    const bool descending = z0 > half_pi;
    // The grazing ray itself (the grid's first sample) is kept: a micrometre tolerance on the invariant.
    if (descending && p < (1.0 + profile.refractivity(0.0)) * surface_radius_m - 1.0e-6) return std::nullopt;
    if (p >= top_radius) return 0.0;  // passes above the atmosphere
    const double z_end = std::asin(p / top_radius);
    double z_start = z0;
    if (altitude >= profile.top_altitude_m()) {
        if (!descending) return 0.0;
        z_start = constants::pi - z_end;  // the ray enters the atmosphere on the observer's side
    }
    // Radius on the ray at zenith angle z: n(r) r sin z = p (Newton; r(z) is single-valued in sin z on both legs).
    const auto integrand = [&](double z) {
        const double s = std::sin(z);
        double r = p / s;
        for (int k = 0; k < radius_newton_steps; ++k) {
            const double h = std::max(r - surface_radius_m, 0.0);
            const double n = 1.0 + profile.refractivity(h);
            const double g = n * r * s - p;
            const double dg = s * (n + r * profile.refractivity_gradient_per_m(h));
            r -= g / dg;
        }
        const double h = std::max(r - surface_radius_m, 0.0);
        const double rn = r * profile.refractivity_gradient_per_m(h);
        return rn / (1.0 + profile.refractivity(h) + rn);
    };
    const double step = (z_start - z_end) / simpson_intervals;
    if (step <= 0.0) return 0.0;
    double sum = integrand(z_end) + integrand(z_start);
    for (int i = 1; i < simpson_intervals; ++i) {
        sum += (i % 2 == 1 ? 4.0 : 2.0) * integrand(z_end + i * step);
    }
    return std::max(-sum * step / 3.0, 0.0);
}

double RefractionLiftTable::lift_at(double true_elevation_rad) const {
    if (lift_rad.size() < 2 || true_elevation_span_rad <= 0.0) return 0.0;
    const double x = (true_elevation_rad - true_elevation_min_rad) / true_elevation_span_rad;
    if (x > 1.0) return 0.0;
    const double u = std::sqrt(std::max(x, 0.0)) * static_cast<double>(lift_rad.size() - 1);
    const auto i = std::min(static_cast<std::size_t>(u), lift_rad.size() - 2);
    const double f = u - static_cast<double>(i);
    return lift_rad[i] + f * (lift_rad[i + 1] - lift_rad[i]);
}

RefractionLiftTable refraction_lift_table(const RefractionProfile& profile, double surface_radius_m,
                                          double observer_altitude_m, std::size_t samples) {
    const double altitude = std::max(observer_altitude_m, 0.0);
    const double observer_radius = surface_radius_m + altitude;
    const double n_observer = 1.0 + profile.refractivity(altitude);
    const double top_radius = surface_radius_m + profile.top_altitude_m();
    // Apparent elevations from the ray grazing the surface up to the zenith (observer inside the atmosphere) or to
    // the ray grazing the atmosphere's top (observer above it).
    const double low = -std::acos(std::min(
        (1.0 + profile.refractivity(0.0)) * surface_radius_m / (n_observer * observer_radius), 1.0));
    const double high = observer_radius < top_radius ? half_pi : -std::acos(top_radius / observer_radius);
    const std::size_t rays = 2 * samples;
    std::vector<double> apparent(rays);
    std::vector<double> true_elevation(rays);
    for (std::size_t i = 0; i < rays; ++i) {
        const double x = static_cast<double>(i) / static_cast<double>(rays - 1);
        apparent[i] = low + (high - low) * x * x;
        true_elevation[i] = apparent[i] - refraction_bending_rad(profile, surface_radius_m, altitude, apparent[i])
                                              .value_or(0.0);
    }
    // The upper monotone branch: below a fold the true elevation would grow again downwards (inverted image).
    std::size_t first = rays - 1;
    while (first > 0 && true_elevation[first - 1] < true_elevation[first]) --first;

    RefractionLiftTable table;
    // Far away (beyond ~330 000 km) the bent limb rays pass the nadir: elevation no longer orders them. No refraction
    // there (the Earth is a ~1 deg disc; its refracted ring of the Sun in a lunar eclipse is not modelled).
    if (true_elevation[first] < -half_pi) return table;
    table.true_elevation_min_rad = true_elevation[first];
    table.true_elevation_span_rad = true_elevation[rays - 1] - true_elevation[first];
    table.lift_rad.assign(samples, 0.0);
    if (first + 1 >= rays) return table;
    std::size_t k = first;
    for (std::size_t j = 0; j < samples; ++j) {
        const double x = static_cast<double>(j) / static_cast<double>(samples - 1);
        const double t = table.true_elevation_min_rad + table.true_elevation_span_rad * x * x;
        while (k + 2 < rays && true_elevation[k + 1] < t) ++k;
        const double f = std::clamp((t - true_elevation[k]) / (true_elevation[k + 1] - true_elevation[k]), 0.0, 1.0);
        table.lift_rad[j] = apparent[k] + f * (apparent[k + 1] - apparent[k]) - t;
    }
    return table;
}

}  // namespace starsim::environment
