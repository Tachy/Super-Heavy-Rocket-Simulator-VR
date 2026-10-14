// Parachutes (d4 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): configuration, drag area over the
// reefing stages, riser-point force; vehicle state, body physics, opening loads; the deployment sequencer.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "starsim/aero/parachute.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/parachute_sequencer.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {

config::ParachuteConfig two_stage_chute() {
    config::ParachuteConfig p;
    p.id = "test";
    p.role = "main";
    p.count = 1;
    p.drag_area_m2 = 100.0;
    p.fill_time_s = 2.0;
    p.reefing = {{0.1, 3.0}, {0.4, 3.0}};
    p.max_load_n = 1e6;
    p.deploy_height_m = 1000.0;
    return p;
}

}  // namespace

TEST_CASE("Parachute drag area grows through its reefing stages", "[parachute]") {
    const auto p = two_stage_chute();
    CHECK(aero::parachute_drag_area_m2(p, -1.0) == 0.0);
    CHECK(aero::parachute_drag_area_m2(p, 0.0) == 0.0);
    CHECK(aero::parachute_drag_area_m2(p, 1.0) == Catch::Approx(100.0 * 0.1 * 0.25));  // t^2 fill: half time, quarter
    CHECK(aero::parachute_drag_area_m2(p, 2.0) == Catch::Approx(10.0));                // first stage reached
    CHECK(aero::parachute_drag_area_m2(p, 4.9) == Catch::Approx(10.0));                // held 3 s
    CHECK(aero::parachute_drag_area_m2(p, 6.0) == Catch::Approx(10.0 + 30.0 * 0.25));  // filling to stage 2
    CHECK(aero::parachute_drag_area_m2(p, 7.0) == Catch::Approx(40.0));
    CHECK(aero::parachute_drag_area_m2(p, 11.0) == Catch::Approx(40.0 + 60.0 * 0.25)); // full opening
    CHECK(aero::parachute_drag_area_m2(p, 12.0) == Catch::Approx(100.0));
    CHECK(aero::parachute_drag_area_m2(p, 1000.0) == Catch::Approx(100.0));
    auto unreefed = p;
    unreefed.reefing.clear();
    CHECK(aero::parachute_drag_area_m2(unreefed, 1.0) == Catch::Approx(25.0));
    CHECK(aero::parachute_drag_area_m2(unreefed, 2.0) == Catch::Approx(100.0));
}

TEST_CASE("Parachute force opposes the riser point's air velocity with q C_D S", "[parachute]") {
    const Vec3 v{-50.0, 3.0, -4.0};
    const auto f = aero::parachute_force_body_n(20.0, v, 1.2);
    const double q = 0.5 * 1.2 * dot(v, v);
    CHECK(norm(f) == Catch::Approx(q * 20.0));
    CHECK(dot(math::normalized(f), math::normalized(v)) == Catch::Approx(-1.0));
    CHECK(norm(aero::parachute_force_body_n(20.0, {}, 1.2)) == 0.0);
}

TEST_CASE("Crew Dragon carries two drogues and four mains", "[parachute][config]") {
    const auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    const auto& capsule = c.stages.back();
    REQUIRE(capsule.parachutes.size() == 2);
    const auto& drogue = capsule.parachutes[0];
    const auto& main = capsule.parachutes[1];
    CHECK(drogue.role == "drogue");
    CHECK(drogue.count == 2);
    CHECK(drogue.deploy_height_m == Catch::Approx(5500.0));
    CHECK(main.role == "main");
    CHECK(main.count == 4);
    CHECK(main.deploy_height_m == Catch::Approx(1800.0));
    CHECK(main.deploy_height_m < drogue.deploy_height_m);
    CHECK(capsule.max_splashdown_speed_mps == Catch::Approx(9.0));
    CHECK(c.stages.front().parachutes.empty());  // the trunk
}

namespace {

// A copy of the config dir with `from` replaced by `to` in crew_dragon.toml.
std::filesystem::path patched_config(const std::string& from, const std::string& to) {
    namespace fs = std::filesystem;
    const fs::path src = STARSIM_CONFIG_DIR;
    const fs::path dir = fs::temp_directory_path() / "starsim_parachute_config";
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy(src, dir, fs::copy_options::recursive);
    std::ifstream in(src / "vehicles" / "crew_dragon.toml");
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    std::ofstream(dir / "vehicles" / "crew_dragon.toml") << text;
    return dir;
}

}  // namespace

TEST_CASE("Parachute config is validated", "[parachute][config]") {
    CHECK_THROWS_AS(config::load_vehicle(patched_config("role = \"drogue\"", "role = \"ballute\""), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("reefing = [[0.35, 4.0]]", "reefing = [[1.2, 4.0]]"), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("max_splashdown_speed_mps = 9.0", ""), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("deploy_height_m = 1800.0", "deploy_height_m = 6000.0"), "crew_dragon"),
                    config::ConfigError);
}

namespace {

class CentralGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3& r, time::Epoch) const override {
        return r * (-constants::earth_gm_m3ps2 / std::pow(norm(r), 3));
    }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

// Non-rotating Earth with central gravity and the US76 atmosphere.
struct World {
    CentralGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

// The capsule alone, RCS tank at `propellant_kg` (10.0 t with 1.4 t: the parachute sizing mass).
config::VehicleConfig capsule_only(double propellant_kg = 1400.0) {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    for (auto& t : c.stages.front().tanks) t.mass_kg = propellant_kg;
    return c;
}

// Capsule `height_m` above the equator at 0 deg longitude (GCRF +x: up), falling vertically at `sink_mps`, heat shield
// down (body +x up: the identity attitude).
std::unique_ptr<vehicle::FlightBody> falling_capsule(double height_m, double sink_mps, double propellant_kg = 1400.0) {
    const Vec3 r = frames::geodetic_to_fixed({0.0, 0.0, height_m});
    vehicle::RigidBodyState s;
    s.position_gcrf_m = r;
    s.velocity_gcrf_mps = math::normalized(r) * -sink_mps;
    s.attitude = math::Quat{1.0, 0.0, 0.0, 0.0};
    auto body = std::make_unique<vehicle::FlightBody>(
        std::make_unique<vehicle::Vehicle>(capsule_only(propellant_kg), STARSIM_CONFIG_DIR), s);
    body->set_ground_height(0.0);
    return body;
}

// Steps `body` for `seconds` at 200 Hz without commands (stops once down); returns the peak aerodynamic load (g).
double fall(vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch& t, double seconds) {
    double peak = 0.0;
    for (int i = 0; i < static_cast<int>(seconds * 200.0 + 0.5) && !body.impacted() && !body.resting(); ++i) {
        body.step(env, t, 0.005, {});
        t += 0.005;
        peak = std::max(peak, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
    }
    return peak;
}

double sink_of(const vehicle::FlightBody& body) {
    return -dot(body.state().velocity_gcrf_mps, math::normalized(body.state().position_gcrf_m));
}

double height_of(const vehicle::FlightBody& body) {
    return frames::fixed_to_geodetic(body.state().position_gcrf_m).height_m;
}

}  // namespace

TEST_CASE("Vehicle opens, releases and tears parachute sets", "[parachute][vehicle]") {
    vehicle::Vehicle v(capsule_only(), STARSIM_CONFIG_DIR);
    const auto& sets = v.stages().front().parachutes;
    REQUIRE(sets.size() == 2);
    CHECK_FALSE(v.parachutes_out());
    CHECK(v.deploy_parachutes("drogue"));
    CHECK_FALSE(v.deploy_parachutes("drogue"));  // already out
    CHECK(v.parachutes_out());
    v.update_parachutes(1.0);
    CHECK(sets[0].drag_area_m2 == Catch::Approx(2 * 28.0 * 0.35));
    CHECK(sets[1].drag_area_m2 == 0.0);
    v.release_parachutes("drogue");
    CHECK(sets[0].state == vehicle::ParachuteState::Released);
    CHECK(sets[0].drag_area_m2 == 0.0);
    CHECK_FALSE(v.parachutes_out());
    CHECK(v.deploy_parachutes("main"));
    v.record_parachute_load(0, 1, 100000.0);
    CHECK(sets[1].peak_load_n == 100000.0);
    CHECK_FALSE(v.parachute_torn());
    v.record_parachute_load(0, 1, 160000.0);  // > 120 kN per canopy
    CHECK(sets[1].state == vehicle::ParachuteState::Torn);
    CHECK(v.parachute_torn());
    CHECK_FALSE(v.parachutes_out());
    CHECK(v.splashdown_recovery());
    CHECK(v.max_splashdown_speed_mps() == 9.0);
}

TEST_CASE("Parachute attachment stations follow the capsule when the trunk separates", "[parachute][vehicle]") {
    vehicle::Vehicle stack(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"), STARSIM_CONFIG_DIR);
    CHECK_FALSE(stack.splashdown_recovery());  // the trunk is the lowest stage
    const double in_stack = stack.stages().back().parachutes.front().attach_station_m;
    const double trunk_length = stack.stages().front().config.length_m;
    CHECK(in_stack == Catch::Approx(trunk_length + 3.6));
    auto capsule = stack.separate(1);
    CHECK(capsule->stages().front().parachutes.front().attach_station_m == Catch::Approx(3.6));
    CHECK(capsule->splashdown_recovery());
}

TEST_CASE("Under its drogues the capsule falls at the published 119 mph at 6,000 ft", "[parachute][body]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(4000.0, 60.0);
    body->vehicle().deploy_parachutes("drogue");
    fall(*body, w.env, t, 25.0);  // settles to its terminal speed long before 1.8 km
    while (height_of(*body) > 1800.0) fall(*body, w.env, t, 0.1);
    std::printf("[validation] drogues: %.1f m/s at %.0f m (published ~53 m/s at ~1.8 km)\n", sink_of(*body), height_of(*body));
    CHECK(sink_of(*body) == Catch::Approx(53.0).epsilon(0.03));  // [S:nasa_demo2] ~119 mph
    // hangs heat shield down: body +x within 5 deg of up
    const Vec3 x = body->state().attitude.rotate({1.0, 0.0, 0.0});
    CHECK(dot(x, math::normalized(body->state().position_gcrf_m)) > std::cos(5.0 * constants::deg_to_rad));
}

TEST_CASE("Under its mains the capsule reaches the water at the published ~16 mph", "[parachute][body]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(1000.0, 8.0);
    body->vehicle().deploy_parachutes("main");
    fall(*body, w.env, t, 30.0);  // through the reefing stages to full
    REQUIRE(height_of(*body) > 50.0);
    while (height_of(*body) > 5.0) fall(*body, w.env, t, 0.1);
    std::printf("[validation] mains: %.2f m/s at %.0f m (published 15-16 mph = 6.7-7.2 m/s)\n", sink_of(*body), height_of(*body));
    CHECK(sink_of(*body) == Catch::Approx(7.0).epsilon(0.03));  // [S:splashdown_speed] 15-16 mph
}

TEST_CASE("Reefed openings keep the crew under 3.6 g and the canopies under their limits", "[parachute][body]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(5500.0, 175.0);  // the fast end of the d3 gate dispersion
    body->vehicle().deploy_parachutes("drogue");
    double peak = fall(*body, w.env, t, 20.0);
    const auto& sets = body->vehicle().stages().front().parachutes;
    CHECK(sets[0].state == vehicle::ParachuteState::Open);
    CHECK(sets[0].peak_load_n < 0.8 * sets[0].config.max_load_n);
    while (height_of(*body) > 1800.0) fall(*body, w.env, t, 0.1);
    body->vehicle().release_parachutes("drogue");
    body->vehicle().deploy_parachutes("main");
    peak = std::max(peak, fall(*body, w.env, t, 40.0));
    CHECK(sets[1].state == vehicle::ParachuteState::Open);
    CHECK(sets[1].peak_load_n < 0.8 * sets[1].config.max_load_n);
    CHECK(peak < 3.6);
    CHECK_FALSE(body->crew_load_warning());
    std::printf("[validation] parachute openings: peak %.2f g, drogue %.0f kN, main %.0f kN per canopy\n", peak,
                sets[0].peak_load_n / 1000.0, sets[1].peak_load_n / 1000.0);
}

TEST_CASE("Mains opened at drogue speed tear", "[parachute][body]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(5500.0, 160.0);
    body->vehicle().deploy_parachutes("main");
    fall(*body, w.env, t, 5.0);
    CHECK(body->vehicle().parachute_torn());
    CHECK(body->vehicle().stages().front().parachutes[1].state == vehicle::ParachuteState::Torn);
}

namespace {

std::vector<config::ParachuteConfig> dragon_sets() {
    return config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon").stages.back().parachutes;
}

}  // namespace

TEST_CASE("Sequencer: drogues at their height, mains below, release on the water", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    CHECK(seq.watch_height_m() == Catch::Approx(5500.0 + 1000.0));
    auto a = seq.update({6000.0, 9000.0, 160.0, false});
    CHECK_FALSE(a.deploy_drogues);
    a = seq.update({5490.0, 9000.0, 160.0, false});
    CHECK(a.deploy_drogues);
    CHECK(seq.phase() == gnc::ParachutePhase::Drogues);
    a = seq.update({3000.0, 2000.0, 60.0, false});
    CHECK_FALSE(a.deploy_mains);
    a = seq.update({1795.0, 1500.0, 53.0, false});
    CHECK(a.release_drogues);
    CHECK(a.deploy_mains);
    CHECK(seq.phase() == gnc::ParachutePhase::Mains);
    a = seq.update({0.0, 30.0, 7.0, true});
    CHECK(a.release_mains);
    CHECK_FALSE(a.release_drogues);
    CHECK(seq.phase() == gnc::ParachutePhase::Down);
    CHECK(seq.drogue_height_m() == 5490.0);
    CHECK(seq.main_height_m() == 1795.0);
    a = seq.update({0.0, 0.0, 0.0, true});
    CHECK_FALSE(a.release_mains);  // once
    CHECK(std::string(gnc::to_string(gnc::ParachutePhase::Drogues)) == "Bremsschirme");
}

TEST_CASE("Sequencer: the drogues wait for the dynamic pressure; the mains still come at their height", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    CHECK_FALSE(seq.update({5400.0, 20000.0, 200.0, false}).deploy_drogues);  // > 15 kPa
    CHECK(seq.phase() == gnc::ParachutePhase::Armed);
    const auto a = seq.update({1790.0, 18000.0, 190.0, false});
    CHECK(a.deploy_mains);  // backup: the mains go at their height whatever the drogues did
    CHECK_FALSE(a.release_drogues);
    CHECK(seq.phase() == gnc::ParachutePhase::Mains);
    CHECK_FALSE(seq.drogue_height_m());
}

TEST_CASE("Sequencer: below the main height at once the mains open", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    const auto a = seq.update({1000.0, 1000.0, 40.0, false});
    CHECK(a.deploy_mains);
    CHECK_FALSE(a.deploy_drogues);
}

TEST_CASE("Sequencer: an ascending vehicle never deploys", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    for (double h = 0.0; h < 10000.0; h += 100.0) {
        const auto a = seq.update({h, 5000.0, -100.0, false});
        CHECK_FALSE(a.deploy_drogues);
        CHECK_FALSE(a.deploy_mains);
    }
    CHECK(seq.phase() == gnc::ParachutePhase::Armed);
}

TEST_CASE("Sequencer: resting on the surface before any deployment ends the sequence", "[parachute][gnc]") {
    gnc::ParachuteSequencer seq(dragon_sets());
    const auto a = seq.update({0.0, 0.0, 0.0, true});
    CHECK_FALSE(a.deploy_mains);
    CHECK(seq.phase() == gnc::ParachutePhase::Down);
}

TEST_CASE("A capsule under its mains rests on the water: gewassert", "[parachute][splashdown]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(30.0, 7.0);
    body->vehicle().deploy_parachutes("main");
    body->vehicle().update_parachutes(60.0);  // fully open
    fall(*body, w.env, t, 30.0);
    REQUIRE(body->touchdown_sink_mps());
    CHECK(*body->touchdown_sink_mps() == Catch::Approx(7.0).epsilon(0.05));
    CHECK(body->resting());
    CHECK_FALSE(body->impacted());
    sim::LaunchSite site;  // any site: a splashdown vehicle is judged by its sink rate, not by the shoreline
    CHECK(sim::landing_outcome(*body, site, {}) == sim::LandingOutcome::SplashedDown);
    CHECK(std::string(sim::to_string(sim::LandingOutcome::SplashedDown)) == "gewassert");
}

TEST_CASE("A capsule hitting the water faster than its limit is an impact", "[parachute][splashdown]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(200.0, 50.0);  // drogue speed, no mains
    fall(*body, w.env, t, 30.0);
    REQUIRE(body->touchdown_sink_mps());
    CHECK(*body->touchdown_sink_mps() > 9.0);
    CHECK(body->impacted());
    sim::LaunchSite site;
    CHECK(sim::landing_outcome(*body, site, {}) == sim::LandingOutcome::Impact);
}

TEST_CASE("A tilted capsule still splashes down", "[parachute][splashdown]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(1.0, 6.0);
    auto s = body->state();
    s.attitude = (math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 30.0 * constants::deg_to_rad) * s.attitude).normalized();
    body->set_state(s);
    fall(*body, w.env, t, 5.0);
    CHECK(body->resting());  // the ground rule (<= 10 deg tilt) does not apply on water
}

TEST_CASE("A capsule entry profile needs a capsule table", "[dragon][config]") {
    CHECK_THROWS_AS(config::load_vehicle(patched_config("capsule_table = \"aero/crew_dragon_aero.csv\"", ""), "crew_dragon"),
                    config::ConfigError);
}

TEST_CASE("Parachute states read in German; a torn set says so", "[parachute][vehicle]") {
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Stowed)) == "verstaut");
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Open)) == "offen");
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Released)) == "abgeworfen");
    CHECK(std::string(vehicle::to_string(vehicle::ParachuteState::Torn)) == "Fallschirm gerissen");
}

TEST_CASE("The air velocity at a riser point is the body's when it does not rotate", "[parachute][body]") {
    World w;
    time::Epoch t;
    auto body = falling_capsule(3000.0, 50.0);
    fall(*body, w.env, t, 0.1);
    const double station = body->vehicle().stages().front().parachutes.front().attach_station_m;
    const Vec3 v = body->point_air_velocity_body_mps(station);
    const Vec3 w_body = body->state().angular_velocity_body_radps;
    const auto mass = body->vehicle().mass_properties();
    const Vec3 arm{station - mass.cg_station_m, -mass.cg_offset_m.y, -mass.cg_offset_m.z};
    const Vec3 expected = body->air_data().air_velocity_body_mps + cross(w_body, arm);
    CHECK(norm(v - expected) < 1e-12);
    CHECK(norm(v) == Catch::Approx(50.0).epsilon(0.05));
}
