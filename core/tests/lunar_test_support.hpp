#pragma once

// Lunar test scenarios shared by several test files (l4): an HLS placed at the powered-descent initiation.

#include <cmath>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::test {

// The HLS with `propellant_kg` in its tanks (O/F 3.6); `dry_kg` > 0 replaces its dry mass.
inline config::VehicleConfig hls_with_propellant(double propellant_kg, double dry_kg = 0.0) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    if (dry_kg > 0.0) cfg.stages.front().dry_mass_kg = dry_kg;
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant_kg * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    return cfg;
}

// Moon-relative PDI state: periselene of a (h_point + 15 km) x 100 km polar ellipse, `range_m` of ground arc before
// the touchdown point, in the plane through the pole and the point, heading south; the HLS retrograde, ready to brake.
inline void place_hls_at_pdi(sim::FlightSession& session, const ephemeris::Ephemeris& ephemeris,
                             const frames::BodyOrientation& moon_me, const mission::TouchdownPoint& point, time::Epoch t0,
                             double propellant_kg, double dry_kg = 0.0, double range_m = 450e3) {
    using math::Vec3;
    const double R = constants::moon_mean_radius_m, gm = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const auto rot = moon_me.fixed_to_gcrf(t0).rotation;
    const Vec3 p = rot * Vec3{std::cos(point.latitude_rad) * std::cos(point.longitude_rad),
                              std::cos(point.latitude_rad) * std::sin(point.longitude_rad), std::sin(point.latitude_rad)};
    const Vec3 pole = rot * Vec3{0.0, 0.0, 1.0};
    const Vec3 n = math::normalized(cross(p, pole));  // cross(n, p): local north at p
    const double angle = range_m / (R + point.height_m);
    const Vec3 r_hat = p * std::cos(angle) + cross(n, p) * std::sin(angle);  // uprange of p: north of it
    const Vec3 along = -cross(n, r_hat);                                       // towards p: southwards
    const double rp = R + point.height_m + 15e3, ra = R + 100e3;
    const double vp = std::sqrt(gm * (2.0 / rp - 2.0 / (rp + ra)));
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
    const Vec3 r = moon.position_m + r_hat * rp, v = moon.velocity_mps + along * vp;
    const Vec3 x_body = -along;  // retrograde: ready to brake
    const auto attitude = math::Quat::from_matrix(math::Mat3::from_columns(x_body, r_hat, cross(x_body, r_hat)));
    session.place_body(hls_with_propellant(propellant_kg, dry_kg), {r, v, attitude, {}}, t0);
}

}  // namespace starsim::test
