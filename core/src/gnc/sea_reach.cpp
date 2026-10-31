#include "starsim/gnc/sea_reach.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

namespace {

constexpr int bisection_steps = 60;
constexpr double longest_flight_s = 1000.0;  // bracket of the search: far beyond any propellant on board

// Free fall from (height, sink), then a brake at full thrust to a stop on the ground.
double fall_time_s(const SeaReachInput& in) {
    const double g = in.gravity_mps2, brake = std::max(in.max_accel_mps2 - g, 1e-3);
    const double h = std::max(in.height_m, 0.0), v0 = std::max(in.sink_mps, 0.0);
    const double peak = std::sqrt((h + v0 * v0 / (2.0 * g)) / (0.5 / g + 0.5 / brake));
    if (peak <= v0) return v0 / brake;  // already too low to fall: braking at once
    return (peak - v0) / g + peak / brake;
}

// Lateral acceleration at flight time T: the tilt limit on the vertical thrust, within the full thrust.
double lateral_accel_mps2(const SeaReachInput& in, double time_s) {
    const double vertical = in.gravity_mps2 + (time_s > 0.0 ? std::max(in.sink_mps, 0.0) / time_s : 0.0);
    const double spare_sq = in.max_accel_mps2 * in.max_accel_mps2 - vertical * vertical;
    return std::min(vertical * std::tan(in.tilt_max_rad), std::sqrt(std::max(spare_sq, 0.0)));
}

// Distance covered in T when it starts at the seaward speed and ends at rest, accelerating then braking at `a`.
double lateral_reach_m(double speed_mps, double a, double time_s) {
    if (a <= 0.0) return speed_mps * time_s;
    const double accelerating_s = std::clamp(0.5 * (time_s - speed_mps / a), 0.0, time_s);
    const double braking_s = time_s - accelerating_s;
    return speed_mps * accelerating_s + 0.5 * a * accelerating_s * accelerating_s + 0.5 * a * braking_s * braking_s;
}

}  // namespace

SeaReachEstimate estimate_sea_reach(const SeaReachInput& in) {
    // Turning the tilt on, round and off: the time with no net lateral acceleration, 2 tilt / rate of it in all.
    const double turning_s = in.tilt_rate_radps > 0.0 ? 2.0 * in.tilt_max_rad / in.tilt_rate_radps : 0.0;
    const auto reach = [&](double t) {
        return lateral_reach_m(in.seaward_speed_mps, lateral_accel_mps2(in, t), std::max(t - turning_s, 0.0));
    };
    double t = fall_time_s(in);
    if (in.distance_m > 0.0 && reach(t) < in.distance_m) {
        double low = t, high = longest_flight_s;
        for (int i = 0; i < bisection_steps; ++i) {
            const double middle = 0.5 * (low + high);
            (reach(middle) < in.distance_m ? low : high) = middle;
        }
        t = high;
    }
    SeaReachEstimate e;
    e.time_s = t;
    e.delta_v_mps = (in.gravity_mps2 * t + std::max(in.sink_mps, 0.0)) / std::cos(in.distance_m > 0.0 ? in.tilt_max_rad : 0.0);
    e.propellant_kg = in.mass_kg * (1.0 - std::exp(-e.delta_v_mps / in.exhaust_velocity_mps));
    return e;
}

}  // namespace starsim::gnc
