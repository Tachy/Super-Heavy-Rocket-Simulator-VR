#pragma once

// Helpers for return/landing scenarios: a booster with a chosen propellant load, placed over the launch site.

#include <cmath>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::test {

// Single-stage booster of `vehicle_id` with its tanks scaled to `propellant_kg` in total.
inline config::VehicleConfig booster_config(const std::string& vehicle_id, double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, vehicle_id);
    cfg.stages.resize(1);
    double total = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) total += tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= propellant_kg / total;
    return cfg;
}

// Booster state over `site`: CG at `cg_enu_m` (east, north, up from the pad centre on the ground), velocity relative
// to the ground, body x up and body +y east, then rotated by `tilt_enu` (site frame); co-rotating with the Earth.
inline vehicle::RigidBodyState state_over_site(const sim::LaunchSite& site, const frames::BodyOrientation& earth,
                                               time::Epoch epoch, const math::Vec3& cg_enu_m,
                                               const math::Vec3& velocity_enu_mps, const math::Quat& tilt_enu = {}) {
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const math::Mat3 enu_to_itrf = contact::from_columns({-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl});
    const math::Vec3 origin = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const math::Mat3 body_to_enu =
        tilt_enu.to_matrix() * contact::from_columns({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    const auto t = earth.fixed_to_gcrf(epoch);
    const math::Vec3 r_itrf = origin + enu_to_itrf * cg_enu_m;
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.position(r_itrf);
    s.velocity_gcrf_mps = t.velocity(r_itrf, enu_to_itrf * velocity_enu_mps);
    s.attitude = math::Quat::from_matrix(t.rotation * enu_to_itrf * body_to_enu);
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    return s;
}

}  // namespace starsim::test
