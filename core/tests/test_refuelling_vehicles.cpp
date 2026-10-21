// Refuelling r1: tanker, depot and HLS vehicles, run-time tank masses, probe/drogue mating geometry.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "proximity_test_support.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/port_mating.hpp"
#include "test_support.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The top stage of a vehicle flying alone (the ship after staging).
std::unique_ptr<vehicle::Vehicle> top_stage_alone(const std::string& vehicle_id) {
    auto cfg = config::load_vehicle(config_dir, vehicle_id);
    cfg.stages.erase(cfg.stages.begin(), cfg.stages.end() - 1);
    return std::make_unique<vehicle::Vehicle>(cfg, config_dir);
}

}  // namespace

TEST_CASE("set_tank_mass sets one tank at run time, clamped to its capacity", "[refuelling][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    const double full_kg = ship->mass_properties().mass_kg;
    const double lox_capacity_kg = 1224808.7;

    ship->set_tank_mass(0, "lox_main", 100000.0);
    CHECK(ship->mass_properties().mass_kg == Catch::Approx(full_kg - (lox_capacity_kg - 100000.0)));

    ship->set_tank_mass(0, "lox_main", 5.0e6);  // more than fits
    CHECK(ship->stages()[0].propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) ==
          Catch::Approx(lox_capacity_kg));

    ship->set_tank_mass(0, "ch4_main", -1.0);
    CHECK(ship->stages()[0].propellant_kg(config::TankContents::Fuel, config::TankRole::Main) == 0.0);

    CHECK_THROWS_AS(ship->set_tank_mass(0, "no_such_tank", 1.0), std::invalid_argument);
    CHECK_THROWS_AS(ship->set_tank_mass(3, "lox_main", 1.0), std::invalid_argument);
    const double before_kg = ship->mass_properties().mass_kg;
    CHECK_THROWS_AS(ship->set_tank_mass(0, "lox_main", std::numeric_limits<double>::quiet_NaN()),
                    std::invalid_argument);
    CHECK_THROWS_AS(ship->set_tank_mass(0, "lox_main", std::numeric_limits<double>::infinity()),
                    std::invalid_argument);
    CHECK(ship->mass_properties().mass_kg == before_kg);
}

TEST_CASE("A refill does not revive a failed engine", "[refuelling][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    ship->fail_engine(0, 0);
    ship->set_tank_mass(0, "lox_main", 1.0e6);
    CHECK(ship->stages()[0].engines[0].starved);  // fail_engine: never restarts
    CHECK_FALSE(ship->stages()[0].engines[1].starved);
}

TEST_CASE("A refilled dry ship has its engines and cold-gas RCS back", "[refuelling][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    for (const char* id : {"lox_main", "ch4_main", "lox_header", "ch4_header"}) ship->set_tank_mass(0, id, 0.0);
    ship->consume_propellant(0.0);  // the main feed is dry: the engines starve
    REQUIRE(ship->stages()[0].engines.front().starved);
    CHECK(ship->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n == 0.0);  // ullage gas comes from the tanks

    ship->set_tank_mass(0, "lox_main", 300000.0);
    ship->set_tank_mass(0, "ch4_main", 80000.0);
    CHECK_FALSE(ship->stages()[0].engines.front().starved);
    CHECK(ship->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n > 0.0);
}

namespace {

// A config directory of its own (ctest runs cases in parallel): engines, propellants, Block 3 and one test vehicle.
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_refuelling_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml",
                               dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string vehicle_with_stage(const std::string& stage_body) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"ship\"]\n\n[[stage]]\n" +
           stage_body;
}

}  // namespace

TEST_CASE("A stage taken over with `from` is the source vehicle's stage", "[refuelling][config]") {
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    REQUIRE(tanker.stages.size() == block3.stages.size());
    for (size_t i = 0; i < block3.stages.size(); ++i) {
        const auto& a = block3.stages[i];
        const auto& b = tanker.stages[i];
        CAPTURE(a.id);
        CHECK(b.id == a.id);
        // the tanker ship carries its own dry mass (user 2026-10-20, r5: 209 t [E] without cargo hardware)
        CHECK(b.dry_mass_kg == (a.id == "ship" ? 209000.0 : a.dry_mass_kg));
        CHECK(b.dry_cg_station_m == a.dry_cg_station_m);
        REQUIRE(b.tanks.size() == a.tanks.size());
        for (size_t k = 0; k < a.tanks.size(); ++k) {
            CHECK(b.tanks[k].id == a.tanks[k].id);
            CHECK(b.tanks[k].mass_kg == a.tanks[k].mass_kg);
            CHECK(b.tanks[k].capacity_kg == a.tanks[k].capacity_kg);
        }
        CHECK(b.engine_groups.size() == a.engine_groups.size());
        // the ship adds its -x docking pair (refuelling r3)
        CHECK(b.rcs.thrusters.size() == a.rcs.thrusters.size() + (a.id == "ship" ? 2 : 0));
        CHECK(b.flaps.size() == a.flaps.size());
    }
    CHECK(tanker.engines.size() == block3.engines.size());
}

// User 2026-10-20 (r5): a variant may carry its own dry mass on a reused stage (the tanker); the rest stays the source's.
TEST_CASE("A reused stage takes its own dry mass", "[refuelling][config]") {
    const auto dir = config_with("from_dry_mass_ok", vehicle_with_stage("from = \"starship_block3:ship\"\ndry_mass_kg = 200000.0\n"));
    const auto v = config::load_vehicle(dir, "test_vehicle");
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    CHECK(v.stages.back().dry_mass_kg == 200000.0);
    CHECK(v.stages.back().dry_cg_station_m == block3.stages.back().dry_cg_station_m);
    CHECK(v.stages.back().tanks.size() == block3.stages.back().tanks.size());
}

TEST_CASE("Bad `from` stages are config errors", "[refuelling][config]") {
    struct Case {
        const char* name;
        const char* body;
        const char* message;
    };
    const Case cases[] = {
        {"from_override", "from = \"starship_block3:ship\"\nlength_m = 1.0\n", "cannot be set on a reused stage"},
        {"from_dry_mass", "from = \"starship_block3:ship\"\ndry_mass_kg = 0.0\n", "'dry_mass_kg' must be positive"},
        {"from_no_stage", "from = \"starship_block3:nose\"\n", "no such stage"},
        {"from_no_vehicle", "from = \"no_such_vehicle:ship\"\n", "no_such_vehicle.toml"},
        {"from_syntax", "from = \"starship_block3\"\n", "must be \"<vehicle>:<stage>\""},
        {"from_cycle", "from = \"test_vehicle:ship\"\n", "chain too long"},
    };
    for (const auto& c : cases) {
        CAPTURE(c.name);
        const auto dir = config_with(c.name, vehicle_with_stage(c.body));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"), Catch::Matchers::ContainsSubstring(c.message));
    }
    // two stages with one id: the second must not replace the first silently
    const auto dir = config_with("dup_stage", vehicle_with_stage("from = \"starship_block3:ship\"\n\n[[stage]]\n"
                                                                 "from = \"starship_block3:ship\"\n"));
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                      Catch::Matchers::ContainsSubstring("stage 'ship' defined twice"));
}

namespace {

config::DockingPortConfig dorsal_port(config::SoftCaptureMode side) {
    config::DockingPortConfig p;
    p.id = "transfer_dock";
    p.mechanism = config::DockingMechanism::ProbeDrogue;
    p.position_m = {20.0, 0.0, 5.0};
    p.axis = {0.0, 0.0, 1.0};
    p.up = {1.0, 0.0, 0.0};
    p.soft_capture = side;
    p.elements_m = {{12.0, -1.5, 5.0}, {12.0, 1.5, 5.0}, {28.0, -1.5, 5.0}, {28.0, 1.5, 5.0}};
    return p;
}

}  // namespace

TEST_CASE("Back-to-back mating puts the ships parallel, 10 m apart, rolled 180 deg", "[refuelling][docking]") {
    const auto t = vehicle::mated_stage_transform(dorsal_port(config::SoftCaptureMode::Active),
                                                  dorsal_port(config::SoftCaptureMode::Passive));
    const math::Vec3 nose = t.rotation * math::Vec3{1.0, 0.0, 0.0};
    CHECK(nose.x == Catch::Approx(1.0));  // both noses the same way
    const math::Vec3 dorsal = t.rotation * math::Vec3{0.0, 0.0, 1.0};
    CHECK(dorsal.z == Catch::Approx(-1.0));  // backs face each other
    const math::Vec3 axis_point = t.apply({20.0, 0.0, 0.0});
    CHECK(axis_point.x == Catch::Approx(20.0));
    CHECK(axis_point.y == Catch::Approx(0.0).margin(1e-12));
    CHECK(axis_point.z == Catch::Approx(10.0));  // centrelines 10 m apart
}

TEST_CASE("Ports mate only with the same mechanism and probes against drogues", "[refuelling][docking]") {
    const auto probes = dorsal_port(config::SoftCaptureMode::Active);
    const auto drogues = dorsal_port(config::SoftCaptureMode::Passive);
    CHECK_FALSE(vehicle::port_mismatch(probes, drogues).has_value());
    CHECK(vehicle::port_mismatch(probes, probes).has_value());
    CHECK(vehicle::port_mismatch(drogues, probes).has_value());

    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const auto iss = config::load_vehicle(config_dir, "iss");
    REQUIRE(dragon.stages.back().docking_ports.size() == 1);  // the capsule (stages: trunk, capsule)
    const auto& idss = dragon.stages.back().docking_ports.front();
    CHECK(idss.mechanism == config::DockingMechanism::Idss);  // existing ports default to IDSS
    CHECK(idss.elements_m.empty());
    CHECK_FALSE(vehicle::port_mismatch(idss, iss.stages.front().docking_ports.front()).has_value());
    CHECK(vehicle::port_mismatch(idss, drogues).has_value());  // a Dragon cannot dock to a depot drogue
}

TEST_CASE("A Starship near the ISS is refused a proximity flight, not thrown out", "[refuelling][proximity]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, config_dir};
    const auto archive = orbit::OemArchive::scan(proximity_test::fixture_dir, clock);
    const auto t0 = clock.from_utc({2026, 10, 16, 0, 0, 0.0});
    auto iss = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, clock, earth, t0,
                              sim::IssOrbitSource::Oem);
    auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    tanker.stages.erase(tanker.stages.begin());  // the ship alone
    session.place_in_orbit(tanker, orbit::chaser_from_lvlh(iss.state_gcrf, {{-1000.0, 0.0, 0.0}, {}}), t0);
    session.add_station(iss.vehicle, iss.state_gcrf, iss.epoch, iss.impulses);

    bool started = true;
    REQUIRE_NOTHROW(started = session.start_proximity("ida2"));
    CHECK_FALSE(started);
    CHECK_THAT(session.proximity_status(), Catch::Matchers::ContainsSubstring("Andocksysteme passen nicht"));
}

TEST_CASE("Bad probe/drogue ports and transfer connectors are config errors", "[refuelling][config]") {
    const std::string ship = "from = \"starship_block3:ship\"\n";
    const std::string port_head = "[[stage.docking_port]]\nid = \"p\"\nname = \"P\"\nposition_m = [20.0, 0.0, 5.0]\n"
                                  "axis = [0.0, 0.0, 1.0]\nup = [1.0, 0.0, 0.0]\n";
    const std::string connector_head = "[[stage.transfer_connector]]\nid = \"c\"\nposition_m = [8.0, 0.0, 5.0]\n";
    struct Case {
        const char* name;
        std::string body;
        const char* message;
    };
    const Case cases[] = {
        {"pd_no_elements", ship + port_head + "mechanism = \"probe_drogue\"\n", "at least 3 elements_m"},
        {"pd_two_elements",
         ship + port_head + "mechanism = \"probe_drogue\"\nelements_m = [[12.0, 0.0, 5.0], [28.0, 0.0, 5.0]]\n",
         "at least 3 elements_m"},
        {"idss_elements", ship + port_head + "elements_m = [[12.0, 0.0, 5.0], [28.0, 0.0, 5.0], [20.0, 1.0, 5.0]]\n",
         "belong to a probe_drogue port"},
        {"pd_mechanism", ship + port_head + "mechanism = \"magnet\"\n", "mechanism must be"},
        {"tc_no_tank", ship + connector_head + "tank = \"nope\"\n", "main propellant tank"},
        {"tc_header", ship + connector_head + "tank = \"lox_header\"\n", "main propellant tank"},
        {"dup_port", ship + port_head + port_head, "docking port id 'p' twice"},
        {"dup_connector",
         ship + connector_head + "tank = \"lox_main\"\nline_diameter_m = 0.15\ndischarge_coefficient = 0.6\n"
                                 "chilldown_kg = 0.0\n" +
             connector_head + "tank = \"ch4_main\"\nline_diameter_m = 0.15\ndischarge_coefficient = 0.6\n"
                              "chilldown_kg = 0.0\n",
         "transfer connector id 'c' twice"},
    };
    for (const auto& c : cases) {
        CAPTURE(c.name);
        const auto dir = config_with(c.name, vehicle_with_stage(c.body));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"), Catch::Matchers::ContainsSubstring(c.message));
    }
}

namespace {

const config::DockingPortConfig& transfer_dock(const config::StageConfig& s) {
    for (const auto& p : s.docking_ports) {
        if (p.id == "transfer_dock") return p;
    }
    throw std::runtime_error("no transfer_dock on stage " + s.id);
}

// Mates `chaser` (its top stage) to the depot's top stage: every probe on a drogue, every connector on the depot's
// connector of the same contents, and the hulls clear of each other.
void check_mates_with_depot(const std::string& chaser_id) {
    const auto depot = config::load_vehicle(config_dir, "starship_depot").stages.back();
    const auto chaser = config::load_vehicle(config_dir, chaser_id).stages.back();
    const auto& cp = transfer_dock(chaser);
    const auto& dp = transfer_dock(depot);
    REQUIRE_FALSE(vehicle::port_mismatch(cp, dp).has_value());
    const auto t = vehicle::mated_stage_transform(cp, dp);
    constexpr double match_m = 1e-9;
    REQUIRE(cp.elements_m.size() == dp.elements_m.size());
    std::vector<math::Vec3> probes_m;
    for (const auto& probe : cp.elements_m) probes_m.push_back(t.apply(probe));
    CHECK(proximity_test::probe_pairing_problem(probes_m, dp.elements_m, match_m) == std::nullopt);
    // a layout that does not pair one to one is caught: a drogue off by 1 m, two drogues on one point
    auto shifted = dp.elements_m;
    shifted[0].x += 1.0;
    CHECK(proximity_test::probe_pairing_problem(probes_m, shifted, match_m) == "probe without drogue");
    auto doubled = dp.elements_m;
    doubled[1] = doubled[0];
    CHECK(proximity_test::probe_pairing_problem(probes_m, doubled, match_m).has_value());
    REQUIRE(chaser.transfer_connectors.size() == 2);
    for (const auto& c : chaser.transfer_connectors) {
        CAPTURE(c.id);
        const math::Vec3 at = t.apply(c.position_m);
        bool met = false;
        for (const auto& d : depot.transfer_connectors) {
            met = met || (d.contents == c.contents && math::norm(at - d.position_m) < match_m);
        }
        CHECK(met);
    }
    const math::Vec3 chaser_axis = t.apply({0.0, 0.0, 0.0});
    const double gap_m = std::hypot(chaser_axis.y, chaser_axis.z) - 0.5 * (chaser.diameter_m + depot.diameter_m);
    constexpr double min_hull_gap_m = 0.5;  // [E] the probes' reach
    CHECK(gap_m > min_hull_gap_m);
    CHECK((t.rotation * math::Vec3{1.0, 0.0, 0.0}).x == Catch::Approx(1.0));  // noses the same way
}

}  // namespace

TEST_CASE("Tanker and HLS mate back to back with the depot", "[refuelling][docking]") {
    check_mates_with_depot("starship_tanker");
    check_mates_with_depot("starship_hls");
}

TEST_CASE("The depot is a Block 3 ship without entry hardware, 236.5 t dry", "[refuelling][vehicle]") {
    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    REQUIRE(depot.stages.size() == 2);
    CHECK(depot.stages.front().dry_mass_kg == block3.stages.front().dry_mass_kg);  // the same Super Heavy
    const auto& s = depot.stages.back();
    CHECK(s.dry_mass_kg == 236500.0);
    double propellant_kg = 0.0;
    for (const auto& t : s.tanks) propellant_kg += t.mass_kg;
    CHECK(propellant_kg == Catch::Approx(1.6e6));
    CHECK(s.flaps.empty());
    CHECK(s.heat_shield.nose_radius_m == 0.0);
    CHECK(s.catch_pins.azimuths_rad.empty());
    CHECK(s.recovery_method.empty());
    CHECK(transfer_dock(s).soft_capture == config::SoftCaptureMode::Passive);
    CHECK(transfer_dock(s).elements_m.size() == 4);
}

TEST_CASE("A tanker after its transfer translates along the docking axis on its cold-gas RCS",
          "[refuelling][vehicle][rcs]") {
    auto tanker = top_stage_alone("starship_tanker");
    constexpr double residue_kg = 10000.0;  // [E] what is left in each main tank after the transfer (r4 computes it)
    tanker->set_tank_mass(0, "lox_main", residue_kg);
    tanker->set_tank_mass(0, "ch4_main", residue_kg);
    const double mass_kg = tanker->mass_properties().mass_kg;
    const math::Vec3 axes[] = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}};
    for (const auto& u : axes) {
        const auto c = tanker->rcs_pure_force_capability(u);
        std::printf("[validation] tanker translation (%+.0f,%+.0f,%+.0f): %.0f N = %.5f m/s^2 at %.0f t\n", u.x, u.y,
                    u.z, c.force_n, c.force_n / mass_kg, mass_kg / 1000.0);
    }
    CHECK(tanker->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n > 0.0);   // dorsal: the docking axis
    CHECK(tanker->rcs_pure_force_capability({0.0, 0.0, -1.0}).force_n > 0.0);  // back off
    CHECK(tanker->rcs_pure_force_capability({0.0, 1.0, 0.0}).force_n > 0.0);
    CHECK(tanker->rcs_pure_force_capability({0.0, -1.0, 0.0}).force_n > 0.0);
    CHECK(tanker->rcs_pure_force_capability({1.0, 0.0, 0.0}).force_n > 0.0);  // settling thrust (r4)
    CHECK(tanker->rcs_pure_force_capability({-1.0, 0.0, 0.0}).force_n > 0.0);  // the docking pair (refuelling r3)
}
