#pragma once

// Lunar test scenarios shared by several test files (l4): an HLS placed at the powered-descent initiation.

#include <cmath>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/lunar_landing_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::test {

// Real HLS masses at the Moon (l4 mass correction, [D]: 1,720 t in LEO, TLI + LOI on the vacuum Raptors).
inline constexpr double hls_llo_propellant_kg = 453e3;           // 573 t in the LLO after the LOI
inline constexpr double hls_pdi_propellant_kg = 446e3;           // 566 t at PDI (LLO variant)
inline constexpr double hls_pdi_light_propellant_kg = 380e3;     // 500 t at PDI (NRHO variant)
inline constexpr double hls_landed_propellant_kg = 220e3;        // 340 t landed (LLO variant)
inline constexpr double hls_landed_light_propellant_kg = 140e3;  // 260 t landed (NRHO variant)

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
// `position_error_m` / `velocity_error_mps` (GCRF) are added to that state (dispersion campaigns).
inline void place_hls_at_pdi(sim::FlightSession& session, const ephemeris::Ephemeris& ephemeris,
                             const frames::BodyOrientation& moon_me, const mission::TouchdownPoint& point, time::Epoch t0,
                             double propellant_kg, double dry_kg = 0.0, double range_m = 450e3,
                             const math::Vec3& position_error_m = {}, const math::Vec3& velocity_error_mps = {}) {
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
    const Vec3 r = moon.position_m + r_hat * rp + position_error_m;
    const Vec3 v = moon.velocity_mps + along * vp + velocity_error_mps;
    const Vec3 x_body = -along;  // retrograde: ready to brake
    const auto attitude = math::Quat::from_matrix(math::Mat3::from_columns(x_body, r_hat, cross(x_body, r_hat)));
    session.place_body(hls_with_propellant(propellant_kg, dry_kg), {r, v, attitude, {}}, t0);
}

// Periselene speed of the (h_point + 15 km) x 100 km ellipse place_hls_at_pdi starts on.
inline double pdi_speed_mps(const ephemeris::Ephemeris& ephemeris, const mission::TouchdownPoint& point) {
    const double rp = constants::moon_mean_radius_m + point.height_m + 15e3;
    const double ra = constants::moon_mean_radius_m + 100e3;
    return std::sqrt(ephemeris.gm_m3ps2(ephemeris::Body::Moon) * (2.0 / rp - 2.0 / (rp + ra)));
}

// The braking range the mission planner computes for an HLS of `mass_kg` at the place_hls_at_pdi state.
inline double pdi_braking_range_m(const ephemeris::Ephemeris& ephemeris, const mission::TouchdownPoint& point,
                                  double mass_kg) {
    const vehicle::Vehicle v(hls_with_propellant(0.0), STARSIM_CONFIG_DIR);
    const double thrust_n = gnc::set_max_thrust_n(v, gnc::hls_landing_parameters().braking_sets.front());
    return mission::braking_range_m(pdi_speed_mps(ephemeris, point), mass_kg, thrust_n, {});
}

// Flies the powered descent of body 0 until its CG is `height_m` above the touchdown point (true) or the step changes.
inline bool descend_to(sim::FlightSession& session, const ephemeris::Ephemeris& ephemeris, double height_m,
                       const mission::TouchdownPoint& point) {
    const double target_r = constants::moon_mean_radius_m + point.height_m + height_m;
    for (int i = 0; i < 1200 * 10; ++i) {
        if (session.lunar_step() != sim::LunarStep::PoweredDescent) return false;
        const auto c = ephemeris.state_gcrf(ephemeris::Body::Moon, session.epoch());
        if (norm(session.body(0).state().position_gcrf_m - c.position_m) <= target_r) return true;
        session.advance_to(session.epoch() + 0.1, 100);
    }
    return false;
}

}  // namespace starsim::test
