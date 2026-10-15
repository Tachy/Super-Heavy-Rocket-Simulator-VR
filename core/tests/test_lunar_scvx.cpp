// SCvx plans a lunar approach (l4): from a high gate 2 km above and 3 km uprange of a touchdown point on the LOLA
// sphere (no terrain needed), on {sea_level_1} -> {sea_level_1} -> {landing thrusters}, to the touchdown at 1 m/s; the
// snapshot is in vacuum with the Moon's gravity; Earth snapshots unchanged.
// One Raptor flies the approach: the gate's deceleration (~2.3 m/s^2 thrust, ~1.3 MN at 580 t) lies within its range
// (0.87-2.9 MN), while three Raptors at minimum throttle (2.6 MN, 4.5 m/s^2) over-brake and SCvx ends in a poor optimum.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/gnc/descent_planner.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double site_lat = -84.7906 * deg, site_lon = 29.1957 * deg, site_height = 5322.0;  // lunar_sites.toml

struct Fixture {
    bool spice = (test::spice_session(), true);  // kernels before the members that read them
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    environment::Us76Atmosphere air;
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    vehicle::FlightEnvironment env{gravity, air, earth, &ephemeris, true, &moon};
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

gnc::LandingSite lunar_site() {
    gnc::LandingSite site;
    site.body = ephemeris::Body::Moon;
    site.latitude_rad = site_lat;
    site.longitude_rad = site_lon;
    site.height_m = site_height;
    site.pad = gnc::PadTarget{1.5, 0.0};  // feet 1.5 m below the skirt (starship_hls.toml foot_station_m)
    return site;
}

gnc::SnapshotSettings hls_settings() {
    gnc::SnapshotSettings s;
    s.phase_groups = {{{"hls/sea_level_1"}, {"hls/sea_level_1"}, {"hls/landing"}}};
    s.reserve_kg = 5000.0;
    return s;
}

// The HLS (580 t: between the variants) at the high gate: 2 km above the touchdown point, 3 km uprange to the
// south-west, moving 90 m/s towards it and sinking 35 m/s, tilted as the braking phase ends.
vehicle::FlightBody high_gate_body(Fixture& fx, const gnc::PlanFrame& plan) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    const double propellant = 460e3;
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    const auto frame = fx.moon.frame(fx.t0);
    const Vec3 dir = math::normalized(Vec3{0.0, 1.0, 1.0});  // downrange (plan y east, z north)
    const Vec3 r_plan = Vec3{2000.0, 0.0, 0.0} - dir * 3000.0;
    const Vec3 v_plan = Vec3{-35.0, 0.0, 0.0} + dir * 90.0;
    const Vec3 r_fixed = plan.origin_itrf_m + plan.plan_to_itrf * r_plan;  // "itrf" = the body-fixed frame here
    const Vec3 v_fixed = plan.plan_to_itrf * v_plan;
    // Thrust axis as the braking phase hands over: along the high gate's acceleration (0.3 m/s^2 up, 1.2 m/s^2 against
    // the motion; lunar_landing_guidance.cpp) plus the lunar gravity - tilted ~32 deg uprange.
    const double g = constants::moon_gm_m3ps2 / std::pow(constants::moon_mean_radius_m + site_height, 2);
    const Vec3 axis_plan = math::normalized(Vec3{0.3 + g, 0.0, 0.0} - dir * 1.2);
    const Vec3 side_plan = math::normalized(cross(axis_plan, dir));
    const Vec3 x_gcrf = frame.transform.rotation * plan.direction_to_itrf(axis_plan);
    const Vec3 y_gcrf = frame.transform.rotation * plan.direction_to_itrf(side_plan);
    const auto attitude = math::Quat::from_matrix(math::Mat3::from_columns(x_gcrf, y_gcrf, cross(x_gcrf, y_gcrf)));
    return vehicle::FlightBody(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR),
                               {frame.position_gcrf(r_fixed), frame.velocity_gcrf(r_fixed, v_fixed), attitude, {}});
}
}  // namespace

TEST_CASE("A lunar SCvx snapshot is in vacuum, in the Moon's gravity, with the touchdown point as origin", "[descent][lunar_scvx]") {
    Fixture fx;
    const auto site = lunar_site();
    const auto plan = gnc::plan_frame(site);
    CHECK(norm(plan.origin_itrf_m) == Catch::Approx(constants::moon_mean_radius_m + site_height).margin(1e-6));
    auto body = high_gate_body(fx, plan);
    const auto snap = gnc::take_descent_snapshot(body, fx.env, fx.t0, site, plan, hls_settings());
    const double g = constants::moon_gm_m3ps2 / std::pow(constants::moon_mean_radius_m + site_height, 2);
    CHECK(snap.model.gravity_mps2.x == Catch::Approx(-g).epsilon(1e-3));  // Earth's tide: ~1e-5 m/s^2
    CHECK(std::abs(snap.model.gravity_mps2.y) < 1e-3);
    CHECK(snap.model.density_kg_m3.front() == 0.0);
    CHECK(norm(snap.model.earth_rate_radps) == Catch::Approx(2.6617e-6).epsilon(1e-2));
    CHECK(gnc::state_position(snap.initial).x == Catch::Approx(2000.0).margin(1e-3));
    // The landing set: axial thrust (cant 15 deg) and an equivalent gimbal from the RCS torque.
    const auto& landing = snap.model.engine_sets[2];
    CHECK(landing.thrust_max_n == Catch::Approx(12.0 * 140e3 * std::cos(15.0 * deg)).epsilon(1e-6));
    CHECK(landing.gimbal_max_rad > 0.0);
    CHECK(landing.gimbal_max_rad < 1.0 * deg);
}

TEST_CASE("SCvx plans a lunar approach from the high gate to the touchdown point", "[descent][lunar_scvx]") {
    Fixture fx;
    const auto site = lunar_site();
    const auto plan = gnc::plan_frame(site);
    auto body = high_gate_body(fx, plan);
    const auto snap = gnc::take_descent_snapshot(body, fx.env, fx.t0, site, plan, hls_settings());
    gnc::ScvxSettings settings;
    // [E] HLS (Task 6 copies them into lunar_landing_guidance.cpp). 45 deg: braking 90 m/s within 3 km needs ~1.35 m/s^2
    // horizontally against 1.62 m/s^2 gravity (~40 deg); the landing thrusters are steered by the RCS only.
    settings.limits.tilt_max_rad = {45.0 * deg, 45.0 * deg, 3.0 * deg};
    settings.limits.rate_max_radps = {0.05, 0.05, 0.01};
    settings.limits.switches[0] = {0.0, 0.0, 5.0, 60.0, 0.0};
    settings.limits.switches[1] = {2.0 * deg, 0.01, 1.0, 10.0, 150.0};  // landing thrusters from >= 150 m CG height
    settings.limits.approach_time_constant_s = 0.0;                    // pad landing
    settings.kinematic_guess_s = {60.0, 10.0, 30.0};  // [E] braking ~1 min, short second phase, ~30 s on the thrusters
    settings.limits.min_mass_kg = snap.min_mass_kg;
    gnc::DescentPlanner planner;
    const auto result = planner.plan(snap.model, snap.initial, 0, snap.catch_target, settings, nullptr, 0.0);
    std::printf("[lunar_scvx] converged %d, %d iterations, phases %.1f / %.1f / %.1f s, end mass %.1f t, vc %.2e, "
                "defect %.2e\n",
                result.converged, result.iterations, result.reference.duration_s[0], result.reference.duration_s[1],
                result.reference.duration_s[2], result.reference.x.back()[gnc::descent_index::mass] / 1000.0,
                result.virtual_control, result.nonlinear_defect);
    REQUIRE(result.converged);
    const auto end = result.reference.x.back();
    CHECK(norm(gnc::state_position(end) - snap.catch_target.position_m) < 0.3);
    CHECK(gnc::state_velocity(end).x == Catch::Approx(-1.0).margin(0.05));
    // The landing thrusters take over at least 150 m above the ground (CG height).
    const int switch_node = 2 * (settings.nodes_per_phase - 1);
    CHECK(gnc::state_position(result.reference.x[static_cast<size_t>(switch_node)]).x >= 150.0 - 0.5);
}
