// Refuelling r3: Starship rendezvous and probe/drogue docking at the depot.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "proximity_test_support.hpp"
#include "refuelling_test_support.hpp"
#include "starsim/gnc/proximity_guidance.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/sim/station_limits.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

std::unique_ptr<vehicle::Vehicle> ship_alone(const std::string& vehicle_id) {
    return std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, vehicle_id)),
                                              config_dir);
}

}  // namespace

TEST_CASE("Tanker and HLS translate on all six axes; the Block 3 ship still has no -x", "[refuelling][rcs]") {
    for (const std::string id : {"starship_tanker", "starship_hls"}) {
        auto ship = ship_alone(id);
        ship->set_tank_mass(0, "lox_main", 7826.0);  // 10 t main propellant at O/F 3.6 (r1 translation case)
        ship->set_tank_mass(0, "ch4_main", 2174.0);
        INFO(id);
        for (const Vec3 u : {Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, -1, 0}, Vec3{0, 0, 1}, Vec3{0, 0, -1}}) {
            CHECK(ship->rcs_pure_force_capability(u).force_n > 1000.0);
        }
        CHECK(ship->rcs_pure_force_capability({-1, 0, 0}).force_n ==
              Catch::Approx(ship->rcs_pure_force_capability({1, 0, 0}).force_n).epsilon(0.01));
    }
    auto block3 = ship_alone("starship_block3");
    CHECK(block3->rcs_pure_force_capability({-1, 0, 0}).force_n == 0.0);
}

TEST_CASE("RCS supply: main-tank cold gas counts the main tanks, Dragon its RCS tank", "[refuelling][rcs]") {
    auto tanker = ship_alone("starship_tanker");
    tanker->set_tank_mass(0, "lox_main", 20000.0);
    tanker->set_tank_mass(0, "ch4_main", 6000.0);
    CHECK(tanker->rcs_supply_kg() == Catch::Approx(26000.0));  // the headers are not RCS supply (landing reserve)
    CHECK(tanker->rcs_propellant_kg() == 0.0);
    const vehicle::Vehicle dragon(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    CHECK(dragon.rcs_supply_kg() == Catch::Approx(dragon.rcs_propellant_kg()));
    CHECK(dragon.rcs_supply_kg() > 0.0);
}

namespace {

// A config dir with Block 3, the engines and propellants and one test vehicle (as test_refuelling_vehicles.cpp; own
// temp prefix - ctest runs cases in parallel).
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r3_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml", dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string reused_stage(const std::string& from, const std::string& additions) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"booster\"]\n\n[[stage]]\nfrom = \"" + from + "\"\n" + additions;
}

const std::string docking_pod = "[[stage.rcs_pod]]\nid = \"dock\"\nstation_m = 46.0\nradius_m = 4.5\n"
                                "azimuths_deg = [0.0, 180.0]\nthrust_n = 1500.0\ndirections = [\"aft\"]\n";

size_t aft_thrusters(const config::StageConfig& s) {
    return static_cast<size_t>(std::count_if(s.rcs.thrusters.begin(), s.rcs.thrusters.end(),
                                             [](const config::RcsThrusterConfig& t) { return t.direction.x < -0.99; }));
}

}  // namespace

TEST_CASE("A reused stage may add RCS pods, not other RCS keys", "[refuelling][config]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    CHECK(aft_thrusters(tanker.stages.back()) == 2);
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");

    // the booster has [stage.rcs] (its forward pod): pods may be added
    auto dir = config_with("rcs_pod_ok", reused_stage("starship_block3:booster", docking_pod));
    const auto added = config::load_vehicle(dir, "test_vehicle");
    CHECK(added.stages.back().rcs.thrusters.size() == block3.stages.front().rcs.thrusters.size() + 2);
    CHECK(aft_thrusters(added.stages.back()) == 2);
    std::filesystem::remove_all(dir);

    dir = config_with("rcs_table", reused_stage("starship_block3:booster", "[stage.rcs]\nisp_s = 70.0\n"));
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                      Catch::Matchers::ContainsSubstring("cannot be set on a reused stage"));
    std::filesystem::remove_all(dir);
}

TEST_CASE("An rcs_pod on a reused stage without RCS is an error", "[refuelling][config]") {
    const auto dir = config_with("rcs_none", reused_stage("bare:booster", docking_pod));
    std::ofstream(dir / "vehicles" / "bare.toml")
        << "[vehicle]\nid = \"bare\"\nname = \"Bare\"\npropellant = \"methalox\"\nstages_bottom_to_top = [\"booster\"]\n\n"
           "[[stage]]\nid = \"booster\"\nname = \"Bare\"\nlength_m = 10.0\ndiameter_m = 9.0\ndry_mass_kg = 1000.0\n"
           "dry_cg_station_m = 5.0\n";
    CHECK(config::load_vehicle(dir, "bare").stages.front().rcs.thrusters.empty());
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"), Catch::Matchers::ContainsSubstring("rcs_pod needs"));
    std::filesystem::remove_all(dir);
}

TEST_CASE("The depot is a target: LVLH attitude nose forward, dorsal to nadir, its approach limits",
          "[refuelling][station][config]") {
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    const auto& s = depot.stages.back();
    REQUIRE(s.body_in_lvlh_rad);
    CHECK(norm(*s.body_in_lvlh_rad) == 0.0);
    REQUIRE(s.station_proximity.keep_out_radius_m);
    CHECK(*s.station_proximity.keep_out_radius_m == 200.0);
    CHECK(s.station_proximity.approach_ellipsoid_m->x == 2000.0);
    CHECK(*s.station_proximity.waypoint0_below_m == 400.0);
    const auto iss = config::load_vehicle(config_dir, "iss");
    CHECK_FALSE(iss.stages.front().station_proximity.keep_out_radius_m);  // the ISS keeps the code defaults
}

TEST_CASE("Station limits replace only the keys the target sets", "[refuelling][station]") {
    config::StationProximityConfig s;
    s.keep_out_radius_m = 150.0;
    s.waypoint1_range_m = 180.0;
    s.waypoint0_below_m = 350.0;
    s.approach_ellipsoid_m = Vec3{1500.0, 800.0, 800.0};
    const gnc::ProximityParameters defaults;
    const auto p = sim::with_station_limits(gnc::ProximityParameters{}, s);
    CHECK(p.keep_out_radius_m == 150.0);
    CHECK(p.waypoint1_range_m == 180.0);
    CHECK(p.waypoint0_below_m == 350.0);
    CHECK(p.waypoint2_range_m == defaults.waypoint2_range_m);
    CHECK(p.flyaround_radius_m == defaults.flyaround_radius_m);
    CHECK(sim::with_station_limits(gnc::DepartureParameters{}, s).ellipsoid_semi_axes_m.x == 1500.0);
    CHECK(sim::with_station_limits(mission::PhasingParameters{}, s).waypoint0_below_m == 350.0);
    CHECK(sim::with_station_limits(gnc::ProximityParameters{}, config::StationProximityConfig{}).keep_out_radius_m ==
          defaults.keep_out_radius_m);
}

TEST_CASE("add_station takes the depot alone; its drogues face nadir, its nose the velocity", "[refuelling][station]") {
    test::spice_session();
    refuelling_test::DepotSession f;
    CHECK_THROWS_AS(f.session.add_station(config::load_vehicle(config_dir, "starship_depot"), f.depot, f.t0),
                    std::logic_error);  // the stack: a station is one stage
    auto no_target = refuelling_test::tanker_config(100.0e3);
    CHECK_THROWS_AS(f.session.add_station(no_target, f.depot, f.t0), std::logic_error);  // no [stage.station]
    f.session.add_station(refuelling_test::depot_config(f.depot_propellant_kg), f.depot, f.t0);
    CHECK_THROWS_AS(f.session.add_station(refuelling_test::depot_config(1.0), f.depot, f.t0), std::logic_error);
    f.session.advance_to(f.t0 + 1.0);
    const auto& st = f.session.body(0).state();
    const math::Mat3 to_lvlh =
        transpose(frames::lvlh_to_inertial({st.position_gcrf_m, st.velocity_gcrf_mps}).to_matrix());
    const auto port = vehicle::port_kinematics(f.session.body(0), "transfer_dock");
    CHECK((to_lvlh * column(port.axes_gcrf, 0)).z == Catch::Approx(1.0).margin(1e-6));  // drogues to nadir
    CHECK((to_lvlh * column(port.axes_gcrf, 2)).x == Catch::Approx(1.0).margin(1e-6));  // nose along +V-bar
}

namespace {

// CW model of the tanker under the depot: target port axes in LVLH x = nadir (dorsal normal), z = +V (nose),
// y = z x x = -y_lvlh. Authority along the target port axes (Task 1 tanker at ~490 t): +-x_t = the ship's +-z,
// +-y_t its +-y, +-z_t its +-x.
proximity_test::CwSim depot_cw() {
    proximity_test::CwSim s;
    s.n = std::sqrt(constants::earth_gm_m3ps2 / std::pow(constants::wgs84_semi_major_axis_m + 200.0e3, 3));
    s.port_to_lvlh = math::Mat3::from_columns({0, 0, 1}, {0, -1, 0}, {1, 0, 0});
    s.port_lever_lvlh = {2.0, 0.0, 5.0};  // [test] depot CG ~18 m up the hull, port at 20 m, 5 m off the axis
    s.chaser_lever_m = 5.0;
    const double m = 490.0e3;
    s.authority = gnc::AxisAcceleration{{11338.0 / m, 11338.0 / m, 5670.0 / m, 5670.0 / m, 3000.0 / m, 3000.0 / m}};
    s.propellant_kg = 200.0e3;
    s.kg_per_mps = m / (70.0 * constants::standard_gravity_mps2);  // cold gas Isp 70 s
    return s;
}

gnc::ProximityParameters depot_parameters() {
    gnc::ProximityParameters p;
    p.contact_plane_m = 0.80;  // probe/drogue first touch (Task 5: cone depth + stroke)
    p.envelope = gnc::probe_drogue_capture_envelope();
    return p;
}

bool saw(const gnc::ProximityGuidance& g, gnc::ProximityPhase phase) {
    for (const auto& e : g.events()) {
        if (e.phase == phase) return true;
    }
    return false;
}

}  // namespace

TEST_CASE("A nadir port is approached along the R-bar from below, no fly-around", "[refuelling][proximity]") {
    CHECK(gnc::docking_approach({0.0, 0.0, 1.0}) == gnc::DockingApproach::RBarBelow);
    CHECK(gnc::docking_approach({1.0, 0.0, 0.0}) == gnc::DockingApproach::VBar);  // ISS paths unchanged
    CHECK(gnc::docking_approach({0.0, 0.0, -1.0}) == gnc::DockingApproach::RBar);
    const auto path = gnc::proximity_path(gnc::DockingApproach::RBarBelow, gnc::ProximityParameters{});
    REQUIRE(path.size() == 3);
    CHECK(path[1].on_axis);
    CHECK(path[2].on_axis);
}

TEST_CASE("Tanker from Waypoint 0 to contact under the depot inside the probe/drogue envelope",
          "[refuelling][proximity]") {
    auto s = depot_cw();
    s.cg = {{0.0, 0.0, 400.0}, {}};  // Waypoint 0 at rest, 400 m below the depot CG
    gnc::ProximityGuidance g(gnc::DockingApproach::RBarBelow, depot_parameters());
    const double t = proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.contact());
    INFO(g.contact()->envelope.reason);
    CHECK(g.contact()->envelope.inside);
    CHECK(std::hypot(g.contact()->port.position_m.y, g.contact()->port.position_m.z) < 0.10);
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Transfer));  // straight up the axis
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Flyaround));
    CHECK_FALSE(saw(g, gnc::ProximityPhase::Abort));
    CHECK(saw(g, gnc::ProximityPhase::Approach));
    CHECK(t < 2.0 * 3600.0);
}

TEST_CASE("A retreat below the depot stops at Waypoint 2 - the -x thrusters hold the Coriolis drift",
          "[refuelling][proximity]") {
    auto s = depot_cw();
    s.cg = {{0.0, 0.0, 400.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::RBarBelow, depot_parameters());
    proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.phase() == gnc::ProximityPhase::Contact);
    g.capture_failed(s.input(), "Überlast");
    for (double t = 0.0; t < 1800.0 && g.phase() != gnc::ProximityPhase::Waypoint; t += 0.05) {
        s.step(g.update(s.input()).translation_fraction_port, 0.05);
    }
    REQUIRE(g.phase() == gnc::ProximityPhase::Waypoint);  // Waypoint 2, waiting for the go
    for (double t = 0.0; t < 600.0; t += 0.05) s.step(g.update(s.input()).translation_fraction_port, 0.05);
    const auto in = s.input();
    CHECK(in.port.position_m.x == Catch::Approx(20.0).margin(1.0));
    CHECK(std::hypot(in.port.position_m.y, in.port.position_m.z) < 1.0);  // held, no forward drift
    CHECK(norm(in.port.velocity_mps) < 0.01);
}

TEST_CASE("The probe/drogue envelope is the spec's", "[refuelling][proximity]") {
    const auto e = gnc::probe_drogue_capture_envelope();
    CHECK(e.axial_min_mps == 0.03);
    CHECK(e.axial_max_mps == 0.10);
    CHECK(e.offset_max_m == 0.10);
    CHECK(e.angle_max_rad == Catch::Approx(1.0 * constants::deg_to_rad));
    CHECK(gnc::ProximityParameters{}.envelope.axial_min_mps == 0.05);  // IDSS default unchanged
}
