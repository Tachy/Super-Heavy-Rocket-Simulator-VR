#include "starsim/gnc/upright_descent.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

UprightDescentCommand upright_descent(const Vec3& from_tower_m, const Vec3& velocity_mps, const Vec3& up, double height_m,
                                      double min_accel_mps2, double max_accel_mps2, double gravity_mps2,
                                      const UprightDescentParameters& p) {
    Vec3 away = from_tower_m - up * dot(from_tower_m, up);
    const double distance = norm(away);
    if (distance < 1e-3) {  // straight above the tower centre: any horizontal direction
        away = math::normalized(cross(up, std::abs(up.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0}));
    } else {
        away = away / distance;
    }
    const Vec3 v_h = velocity_mps - up * dot(velocity_mps, up);
    const Vec3 v_ref = distance < p.clear_radius_m ? away * p.away_speed_mps : Vec3{0.0, 0.0, 0.0};
    Vec3 a_h = (v_ref - v_h) * p.gain_per_s;

    // sink profile v^2 = v_final^2 + 2 f (a_max - g) h: it reaches the ground at the final sink; along it the reference
    // decelerates at f (a_max - g) (fed forward)
    const double sink = -dot(velocity_mps, up);
    const double excess = std::max(max_accel_mps2 - gravity_mps2, 0.0);
    const double h = std::max(height_m, 0.0);
    const double braking = p.deceleration_fraction * excess;
    const double wanted_sink = std::sqrt(p.final_sink_mps * p.final_sink_mps + 2.0 * braking * h);
    const double feedforward = h > 0.0 ? braking : 0.0;
    // the engines never run below their minimum throttle: the vertical part never drops below min_accel
    const double a_v = std::clamp(gravity_mps2 + feedforward + p.sink_gain_per_s * (sink - wanted_sink),
                                  std::min(min_accel_mps2, max_accel_mps2), max_accel_mps2);

    // tilt limit: the horizontal part never exceeds tan(tilt_max) of the vertical part (and the total stays <= max)
    const double h_max = std::min(a_v * std::tan(p.tilt_max_rad),
                                  std::sqrt(std::max(max_accel_mps2 * max_accel_mps2 - a_v * a_v, 0.0)));
    const double a_h_norm = norm(a_h);
    if (a_h_norm > h_max && a_h_norm > 0.0) a_h = a_h * (h_max / a_h_norm);
    const Vec3 a = a_h + up * a_v;
    const double a_norm = norm(a);
    if (a_norm <= 0.0) return {up, 0.0};
    return {a / a_norm, a_norm};
}

double upright_stop_propellant_kg(double mass_kg, double sink_mps, double max_accel_mps2, double gravity_mps2,
                                  double exhaust_velocity_mps, const UprightDescentParameters& p) {
    if (sink_mps <= 0.0) return 0.0;
    const double braking = p.deceleration_fraction * (max_accel_mps2 - gravity_mps2);
    if (braking <= 0.0) return mass_kg;
    const double dv = sink_mps + gravity_mps2 * sink_mps / braking;
    return mass_kg * (1.0 - std::exp(-dv / exhaust_velocity_mps));
}

}  // namespace starsim::gnc
