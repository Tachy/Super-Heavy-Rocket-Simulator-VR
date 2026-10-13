#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("The descent snapshot puts the booster into the plan frame", "[gnc][descent][snapshot]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    session.set_launch_site(site);
    const auto cfg = test::booster_config("starship_block3", 70000.0);
    const vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR),
                                   test::state_over_site(site, earth, t0, {30.0, -15.0, 1200.0}, {4.0, -2.0, -250.0}));
    const auto landing = session.return_site(body.vehicle());
    REQUIRE(landing.catch_target.has_value());
    REQUIRE(landing.sea.has_value());
    const auto frame = gnc::plan_frame(landing.latitude_rad, landing.longitude_rad, landing.height_m);
    gnc::SnapshotSettings settings;
    settings.phase_groups = {std::vector<std::string>{"booster/center", "booster/divert", "booster/middle"},
                         std::vector<std::string>{"booster/center", "booster/divert"},
                         std::vector<std::string>{"booster/center"}};
    const auto snap = gnc::take_descent_snapshot(body, session.environment(), t0, landing, frame, settings);

    const Vec3 r = gnc::state_position(snap.initial);
    const Vec3 v = gnc::state_velocity(snap.initial);
    CHECK(std::abs(r.x - 1200.0) < 1e-6);
    CHECK(std::abs(r.y - 30.0) < 1e-6);
    CHECK(std::abs(r.z + 15.0) < 1e-6);
    CHECK(std::abs(v.x + 250.0) < 1e-6);
    CHECK(std::abs(v.y - 4.0) < 1e-6);
    CHECK(std::abs(v.z + 2.0) < 1e-6);
    const auto q = gnc::state_attitude(snap.initial);  // upright, body +y east: identity
    CHECK(std::abs(std::abs(q.w) - 1.0) < 1e-9);
    CHECK(norm(gnc::state_rate(snap.initial)) < 1e-12);  // co-rotating with the Earth
    CHECK(snap.initial[gnc::descent_index::mass] == body.vehicle().mass_properties().mass_kg);

    const double lat = site.latitude_rad;
    const double omega = norm(snap.model.earth_rate_radps);
    CHECK(std::abs(omega - 7.292115e-5) < 1e-9);
    CHECK(std::abs(snap.model.earth_rate_radps.x - omega * std::sin(lat)) < 1e-9);
    CHECK(std::abs(snap.model.earth_rate_radps.z - omega * std::cos(lat)) < 1e-9);
    CHECK(snap.model.gravity_mps2.x < -9.7);
    CHECK(snap.model.gravity_mps2.x > -9.85);
    CHECK(snap.model.earth_center_m.x < -6.3e6);

    const double cg = body.vehicle().mass_properties().cg_station_m;
    const auto& tower = *site.tower;
    CHECK(std::abs(snap.catch_target.position_m.x - (tower.catch_height_m + 1.0 - (landing.catch_target->pin_station_m - cg))) < 1e-6);
    CHECK(std::abs(snap.catch_target.position_m.y) < 1e-6);
    CHECK(std::abs(snap.catch_target.position_m.z) < 1e-6);
    CHECK(snap.catch_target.velocity_mps.x == -1.0);
    // Vertical corridor from where the hull bottom is 5 m above the arm tops, half the 3 m standby clearance.
    CHECK(std::abs(snap.catch_target.corridor_top_m - (tower.catch_height_m + cg + 5.0)) < 1e-6);
    CHECK(snap.catch_target.corridor_radius_m == 1.5);
    CHECK(snap.catch_target.final_set_radius_m == 3.0);
    CHECK(snap.abort_target.goal == gnc::DescentGoal::Abort);
    CHECK(std::abs(snap.abort_target.sea_target_m.y - (site.sea->shoreline_distance_m + 150.0)) < 1e-6);  // due east
    CHECK(snap.abort_target.keep_out_radius_m > 0.5 * tower.width_m);
    // Launch mount: outer radius + hull radius + margin, active below the mount top plus the CG station and margin.
    CHECK(std::abs(snap.abort_target.mount_keep_out_radius_m - (tower.mount_outer_radius_m + 4.5 + 5.0)) < 1e-6);
    CHECK(std::abs(snap.abort_target.mount_top_cg_height_m - (tower.mount_height_m + cg + 5.0)) < 1e-6);
    CHECK(snap.min_mass_kg == body.vehicle().mass_properties().mass_kg - body.vehicle().propellant_kg() + 5000.0);
    CHECK(snap.phase0_spool == 0.0);  // engines off
    CHECK(snap.startup_s == 3.0);     // Raptor 3 start-up [E]
}
