// Deorbit burn of the ship (b4): the belly sea-level Raptor alone (`[stage.recovery] deorbit_engine_groups`), its thrust
// gimballed through the CG; the hull is steered so that this tilted thrust axis points along the burn.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>

#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double equatorial_radius_m = 6378137.0;

// The ship alone with `propellant_kg` in its main tanks.
config::VehicleConfig orbiting_ship(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    auto& ship = cfg.stages[0];
    config::fill_tanks(ship, propellant_kg);
    return cfg;
}

}  // namespace

TEST_CASE("Ship config names the belly Raptor as its deorbit engine", "[ship][burn]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    CHECK(cfg.stages[1].deorbit_engine_groups == std::vector<std::string>{"sea_level_3"});
    CHECK(cfg.stages[0].deorbit_engine_groups.empty());
}

namespace {

// A copy of the config dir with `from` removed from config/vehicles/<file> (one directory per call: ctest runs cases as
// parallel processes).
std::filesystem::path without(const std::string& file, const std::string& from) {
    namespace fs = std::filesystem;
    const fs::path src = STARSIM_CONFIG_DIR;
    const fs::path dir = fs::temp_directory_path() / ("starsim_deorbit_dispersion_" + file);
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy(src, dir, fs::copy_options::recursive);
    std::ifstream in(src / "vehicles" / file);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.erase(at, from.size());
    std::ofstream(dir / "vehicles" / file) << text;
    return dir;
}

// The same with `from` replaced by `to`, in a directory of its own (`tag`).
std::filesystem::path replaced(const std::string& tag, const std::string& file, const std::string& from, const std::string& to) {
    namespace fs = std::filesystem;
    const fs::path src = STARSIM_CONFIG_DIR;
    const fs::path dir = fs::temp_directory_path() / ("starsim_config_range_" + tag);
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy(src, dir, fs::copy_options::recursive);
    std::ifstream in(src / "vehicles" / file);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    std::ofstream(dir / "vehicles" / file) << text;
    return dir;
}

}  // namespace

// Final fix (Task 7 M15, Task 4b minor): out-of-range keys are refused with one message per key.
TEST_CASE("Range checks: the deorbit dispersion keys and the dry CG", "[deorbit][config]") {
    using Catch::Matchers::ContainsSubstring;
    CHECK_THROWS_WITH(config::load_vehicle(replaced("residual", "starship_block3.toml", "deorbit_cutoff_residual = 0.002",
                                                    "deorbit_cutoff_residual = 0.2"),
                                           "starship_block3"),
                      ContainsSubstring("deorbit_cutoff_residual must be below 0.1"));
    CHECK_THROWS_WITH(config::load_vehicle(replaced("pointing", "crew_dragon.toml", "deorbit_pointing_error_deg = 0.1",
                                                    "deorbit_pointing_error_deg = 12"),
                                           "crew_dragon"),
                      ContainsSubstring("deorbit_pointing_error_deg must be below 10"));
    CHECK_THROWS_WITH(config::load_vehicle(replaced("dry_cg", "starship_block3.toml", "dry_cg_station_m = 22.03",
                                                    "dry_cg_station_m = 99.0"),
                                           "starship_block3"),
                      ContainsSubstring("dry_cg_station_m must lie within the stage"));
}

// The deorbit burn's dispersion the planner checks is the vehicle's guided cutoff residual (fix round 4, I6): required
// for every stage returning from orbit.
TEST_CASE("The deorbit burn dispersion is the vehicle's cutoff residual", "[deorbit][config]") {
    const auto ship = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3").stages[1];
    CHECK(ship.deorbit_cutoff_residual == 0.002);  // user 2026-10-17
    CHECK(ship.deorbit_pointing_error_rad == Catch::Approx(0.1 * deg));
    const auto d = gnc::deorbit_burn_dispersion(ship);
    CHECK(d.magnitude_fraction == 0.002);
    CHECK(d.pointing_rad == Catch::Approx(0.1 * deg));
    CHECK(d.timing_s == 1.0);
    const auto capsule = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon").stages.back();
    CHECK(capsule.deorbit_cutoff_residual == 0.001);
    CHECK(capsule.deorbit_pointing_error_rad == Catch::Approx(0.1 * deg));
    CHECK(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3").stages[0].deorbit_cutoff_residual == 0.0);

    using Catch::Matchers::ContainsSubstring;
    CHECK_THROWS_WITH(config::load_vehicle(without("starship_block3.toml", "deorbit_cutoff_residual = 0.002"),
                                           "starship_block3"),
                      ContainsSubstring("Schlüssel 'deorbit_cutoff_residual' fehlt in [stage.recovery]"));
    CHECK_THROWS_WITH(config::load_vehicle(without("crew_dragon.toml", "deorbit_pointing_error_deg = 0.1"), "crew_dragon"),
                      ContainsSubstring("Schlüssel 'deorbit_pointing_error_deg' fehlt in [stage.recovery]"));
}

TEST_CASE("Deorbit burn on the belly Raptor alone", "[ship][burn][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravityConfig two_body;
    two_body.moon = two_body.sun = two_body.earth_j2 = false;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR, two_body);
    const double gm = session.gm_earth_m3ps2();
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const orbit::KeplerianElements circular{equatorial_radius_m + 200e3, 0.0, 26.5 * deg, 0.0, 0.0, 0.0};
    const auto orbit_state = orbit::state_from_elements(circular, gm);

    // nose against the velocity (retrograde), belly down
    const Vec3 x = -math::normalized(orbit_state.velocity_mps);
    const Vec3 z = -math::normalized(orbit_state.position_m);
    const Vec3 y = cross(z, x);
    math::Mat3 m;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = columns[c].x;
        m(1, c) = columns[c].y;
        m(2, c) = columns[c].z;
    }
    const vehicle::RigidBodyState state{orbit_state.position_m, orbit_state.velocity_mps, math::Quat::from_matrix(m), {}};
    const auto cfg = orbiting_ship(45000.0);
    session.place_body(cfg, state, t0);

    const double dv = 60.0;
    const double node_s = 300.0;
    const auto at_node = orbit::propagate_two_body(orbit_state, gm, node_s);
    REQUIRE(session.execute_burn({t0 + node_s, -math::normalized(at_node.velocity_mps) * dv}, {"sea_level_3"}));
    const auto& config = session.burn_configuration();
    REQUIRE(config.groups == std::vector<std::string>{"ship/sea_level_3"});  // "<stage>/<group>"
    const double tilt = std::acos(std::clamp(config.thrust_axis_body.x, -1.0, 1.0));
    CHECK(tilt > 1.0 * deg);
    CHECK(tilt < 5.0 * deg);

    std::set<std::string> fired;
    double peak_rate = 0.0, propellant_before = session.body(0).vehicle().propellant_kg();
    for (double t = 0.0; t < 900.0 && session.autopilot().state == sim::AutopilotState::Burn; t += 0.05) {
        session.advance_to(t0 + t, 100000);
        bool thrust = false;
        for (const auto& e : session.body(0).vehicle().stages().front().engines) {
            if (e.producing_thrust()) {
                fired.insert(e.group_id);
                thrust = true;
            }
        }
        if (thrust) peak_rate = std::max(peak_rate, norm(session.body(0).state().angular_velocity_body_radps));
    }
    REQUIRE(session.autopilot().state == sim::AutopilotState::Off);
    session.advance_to(session.epoch() + 5.0);
    const auto& after = session.body(0).state();
    const auto el = orbit::elements_from_state({after.position_gcrf_m, after.velocity_gcrf_mps}, gm);
    const double v_circular = std::sqrt(gm / circular.semi_major_axis_m);
    const double expected_a = 1.0 / (2.0 / circular.semi_major_axis_m - std::pow(v_circular - dv, 2) / gm);
    // out of plane: angle between the orbit normals times the speed
    const Vec3 h0 = math::normalized(cross(orbit_state.position_m, orbit_state.velocity_mps));
    const Vec3 h1 = math::normalized(cross(after.position_gcrf_m, after.velocity_gcrf_mps));
    const double out_of_plane_mps = std::acos(std::clamp(dot(h0, h1), -1.0, 1.0)) * v_circular;
    std::printf("[validation] deorbit burn 60 m/s on %zu engine group(s), thrust axis %.2f deg off the hull: semi-major "
                "axis %.2f km (expected %.2f km), out of plane %.2f m/s, peak rate %.4f rad/s, %.1f t burned\n",
                fired.size(), tilt / deg, el.semi_major_axis_m / 1000.0, expected_a / 1000.0, out_of_plane_mps,
                peak_rate, (propellant_before - session.body(0).vehicle().propellant_kg()) / 1000.0);
    CHECK(fired == std::set<std::string>{"ship/sea_level_3"});
    CHECK(std::abs(el.semi_major_axis_m - expected_a) < 1000.0);  // ~0.6 m/s: 1 % of the burn
    CHECK(out_of_plane_mps < 0.5);
    CHECK(peak_rate < 0.02);
}
