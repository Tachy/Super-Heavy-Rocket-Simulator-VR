// Refuelling r4: propellant transfer between docked ships.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "refuelling_test_support.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/tanker_reserve.hpp"
#include "starsim/sim/propellant_transfer.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

const config::StageConfig& ship_stage(const std::string& id) {
    static std::map<std::string, config::VehicleConfig> cache;
    auto it = cache.find(id);
    if (it == cache.end()) it = cache.emplace(id, config::load_vehicle(config_dir, id)).first;
    return it->second.stages.back();
}

// A config dir with Block 3, the engines and propellants and one test vehicle (as test_refuelling_docking.cpp; own
// temp prefix - ctest runs cases in parallel).
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r4_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml", dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string tanker_with(const std::string& ship_additions) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"ship\"]\n\n[[stage]]\nfrom = \"starship_block3:ship\"\n" + ship_additions;
}

}  // namespace

TEST_CASE("Tanker, depot and HLS carry the transfer hardware", "[refuelling][prop_transfer][config]") {
    for (const std::string id : {"starship_tanker", "starship_depot", "starship_hls"}) {
        INFO(id);
        const auto& s = ship_stage(id);
        REQUIRE(s.propellant_transfer);
        CHECK(s.propellant_transfer->delivery_pressure_pa == 3.5e5);
        CHECK(s.propellant_transfer->receive_pressure_pa == 1.5e5);
        REQUIRE(s.transfer_connectors.size() == 2);
        for (const auto& c : s.transfer_connectors) {
            CHECK(c.line_diameter_m == 0.15);
            CHECK(c.discharge_coefficient == 0.6);
            CHECK(c.chilldown_kg == (c.contents == config::TankContents::Oxidizer ? 150.0 : 60.0));
        }
    }
    CHECK_FALSE(ship_stage("starship_block3").propellant_transfer);
}

TEST_CASE("Transfer hardware errors are reported", "[refuelling][prop_transfer][config]") {
    const std::string connector =
        "[[stage.transfer_connector]]\nid = \"lox\"\ntank = \"lox_main\"\nposition_m = [8.0, 0.0, 5.0]\n";
    SECTION("a connector without its line") {
        const auto dir = config_with("no_line", tanker_with(connector));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                          Catch::Matchers::ContainsSubstring("line_diameter_m"));
    }
    SECTION("receive pressure not below delivery") {
        const auto dir = config_with(
            "pressure", tanker_with(connector + "line_diameter_m = 0.15\ndischarge_coefficient = 0.6\n"
                                                "chilldown_kg = 150.0\n\n[stage.propellant_transfer]\n"
                                                "delivery_pressure_pa = 2.0e5\nreceive_pressure_pa = 2.0e5\n"));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                          Catch::Matchers::ContainsSubstring("receive_pressure_pa"));
    }
}

namespace {

std::unique_ptr<vehicle::Vehicle> ship_alone(const std::string& id) {
    return std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, id)), config_dir);
}

size_t tank_index(const vehicle::Vehicle& v, const std::string& id) {
    const auto& tanks = v.stages()[0].tanks;
    for (size_t i = 0; i < tanks.size(); ++i) {
        if (tanks[i].config.id == id) return i;
    }
    throw std::invalid_argument(id);
}

// 200 km, body z radial (body x perpendicular to the radius, as the along-track nose), no rotation: along x the tide
// is compressive (~1.3e-6 m/s^2 per m), the radial offsets are stretched (2x that).
vehicle::LiquidEnvironment orbit_liquid(const math::Vec3& specific_force_body_mps2) {
    const double r = constants::wgs84_semi_major_axis_m + 200.0e3;
    return {specific_force_body_mps2, {}, {0.0, 0.0, r}, math::Quat{}};
}

}  // namespace

TEST_CASE("Line flow is the orifice law of the smaller C_d A", "[refuelling][prop_transfer]") {
    const double area = 0.6 * std::numbers::pi / 4.0 * 0.15 * 0.15;  // 0.0106029 m^2
    CHECK(vehicle::line_mass_flow_kgps(area, 1200.0, 2.0e5) == Catch::Approx(232.30).epsilon(1e-4));
    CHECK(vehicle::line_mass_flow_kgps(area, 440.0, 2.0e5) == Catch::Approx(140.66).epsilon(1e-4));
    CHECK(vehicle::line_mass_flow_kgps(area, 1200.0, 0.0) == 0.0);
    CHECK(vehicle::line_mass_flow_kgps(area, 1200.0, -1.0e5) == 0.0);  // no back-flow: the valves close
    config::TransferConnectorConfig big, small;
    big.line_diameter_m = 0.20;
    big.discharge_coefficient = 0.6;
    small.line_diameter_m = 0.15;
    small.discharge_coefficient = 0.6;
    CHECK(vehicle::pair_discharge_area_m2(big, small) == Catch::Approx(area));
    CHECK(vehicle::pair_discharge_area_m2(small, big) == Catch::Approx(area));
}

TEST_CASE("Settling time: the liquid crosses the tank from rest", "[refuelling][prop_transfer]") {
    CHECK(vehicle::settling_time_s(16.9, 9.80665e-4) == Catch::Approx(185.65).epsilon(1e-3));
    CHECK(vehicle::settling_time_s(16.9, 0.0) == std::numeric_limits<double>::infinity());
}

TEST_CASE("Outlet coverage: a +x push settles the liquid aft, drag alone moves it forward",
          "[refuelling][prop_transfer]") {
    auto tanker = ship_alone("starship_tanker");
    tanker->set_tank_mass(0, "lox_main", 50.0e3);
    const size_t lox = tank_index(*tanker, "lox_main");
    const double max_tilt = 30.0 * constants::deg_to_rad;
    // settling thrust 1e-4 g along +x: apparent gravity aft, Bo ~ 1200 * 1e-3 * 4.5^2 / 0.0132 ~ 1.8e3
    const auto pushed = vehicle::outlet_check(*tanker, 0, lox, orbit_liquid({9.80665e-4, 0.0, 0.0}), max_tilt);
    CHECK(pushed.covered);
    CHECK(pushed.bond_number > 1.0e3);
    CHECK(pushed.tilt_rad < 5.0 * constants::deg_to_rad);
    // drag alone (nose along +V): the specific force points aft (-x), the liquid goes to the nose ([test] 5e-5 m/s^2,
    // a low orbit's drag - clear of the tide along the tank offset)
    const auto drag = vehicle::outlet_check(*tanker, 0, lox, orbit_liquid({-5.0e-5, 0.0, 0.0}), max_tilt);
    CHECK_FALSE(drag.covered);
    CHECK(drag.tilt_rad > 90.0 * constants::deg_to_rad);
    // the tide 10 m off the compound CG (radial: stretched, ~2.5e-5 m/s^2) pulls sideways: settled (Bo ~ 50) but away
    // from the outlet
    auto docked = orbit_liquid({});
    docked.cg_offset_body_m = {0.0, 0.0, 10.0};
    const auto tide = vehicle::outlet_check(*tanker, 0, lox, docked, max_tilt);
    CHECK(tide.bond_number > 10.0);
    CHECK_FALSE(tide.covered);
    // an empty tank has nothing at its outlet
    tanker->set_tank_mass(0, "lox_main", 0.0);
    CHECK_FALSE(vehicle::outlet_check(*tanker, 0, lox, orbit_liquid({9.80665e-4, 0.0, 0.0}), max_tilt).covered);
}

TEST_CASE("The compound CG offset moves the tide and the rotation, not the specific force",
          "[refuelling][prop_transfer][boil_off]") {
    auto liquid = orbit_liquid({1.0e-3, 0.0, 0.0});
    liquid.angular_velocity_body_radps = {0.0, 1.1e-3, 0.0};
    const math::Vec3 g0 = vehicle::apparent_gravity_body_mps2(liquid, {0.0, 0.0, 10.0});
    liquid.cg_offset_body_m = {0.0, 0.0, 4.0};
    const math::Vec3 g1 = vehicle::apparent_gravity_body_mps2(liquid, {0.0, 0.0, 6.0});
    CHECK(norm(g1 - g0) < 1e-15);  // the same point seen from the reference
}

namespace {

sim::TransferLine lox_line() { return {0.6 * std::numbers::pi / 4.0 * 0.15 * 0.15, 1200.0, 300.0}; }
sim::TransferLine ch4_line() { return {0.6 * std::numbers::pi / 4.0 * 0.15 * 0.15, 440.0, 120.0}; }

// A pure run: the tanks move as the machine says, settled when `settled(t)`.
struct Bench {
    sim::TransferTanks tanks{{156.5e3, 43.5e3}, {469.6e3, 130.4e3}, {1224.8e3, 340.2e3}};
    sim::TransferRequest request;
    std::optional<sim::PropellantTransfer> transfer;
    Bench() { request.delivering_floor_kg = {10.0e3, 2.8e3}; }
    void start() { transfer.emplace(std::array{lox_line(), ch4_line()}, request, tanks); }
    template <class Settled>
    void run(double limit_s, Settled settled, bool thrust_available = true) {
        for (double t = 0.0; t < limit_s && transfer->active(); t += request.step_s) {
            sim::TransferStepInput in;
            in.dt_s = request.step_s;
            in.tanks = tanks;
            in.settled = settled(t);
            in.thrust_available = thrust_available;
            in.settle_time_s = 185.65;
            in.pressure_difference_pa = 2.0e5;
            const auto out = transfer->step(in);
            for (size_t k = 0; k < 2; ++k) {
                tanks.delivering_kg[k] -= out.moved_kg[k] + out.vented_kg[k];
                tanks.receiving_kg[k] += out.moved_kg[k];
            }
        }
    }
};

}  // namespace

TEST_CASE("Transfer phases: settling, chill-down, flow to the floors, complete", "[refuelling][prop_transfer]") {
    Bench b;
    b.start();
    CHECK(b.transfer->state().phase == sim::TransferPhase::Settling);
    CHECK(b.transfer->active());
    b.run(180.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Settling);  // 185.65 s not yet reached
    b.run(10.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Chilldown);
    b.run(120.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Flowing);
    CHECK(b.transfer->state().chilldown_kg[0] == Catch::Approx(300.0));
    CHECK(b.transfer->state().chilldown_kg[1] == Catch::Approx(120.0));
    b.run(3600.0, [](double) { return true; });
    const auto& s = b.transfer->state();
    CHECK(s.phase == sim::TransferPhase::Complete);
    CHECK(s.status == "Transfer abgeschlossen");
    CHECK_FALSE(b.transfer->active());
    CHECK(b.tanks.delivering_kg[0] == Catch::Approx(10.0e3).margin(1e-6));
    CHECK(b.tanks.delivering_kg[1] == Catch::Approx(2.8e3).margin(1e-6));
    CHECK(s.delivered_kg[0] == Catch::Approx(156.5e3 - 300.0 - 10.0e3));
    CHECK(s.delivered_kg[1] == Catch::Approx(43.5e3 - 120.0 - 2.8e3));
    // LOX at 232.3 kg/s: 146.2 t in ~630 s of flow after 190 + 120 s
    CHECK(s.elapsed_s == Catch::Approx(190.0 + 120.0 + 630.0).margin(10.0));
}

TEST_CASE("A full receiver or a limit closes a line; nothing to move opens none", "[refuelling][prop_transfer]") {
    Bench b;
    b.tanks.receiving_kg[1] = b.tanks.receiving_capacity_kg[1] - 1000.0;  // the depot's CH4 nearly full
    b.request.limit_kg[0] = 20.0e3;
    b.start();
    b.run(3600.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Complete);
    CHECK(b.transfer->state().delivered_kg[0] == Catch::Approx(20.0e3));
    CHECK(b.tanks.receiving_kg[1] == Catch::Approx(b.tanks.receiving_capacity_kg[1]));
    sim::TransferTanks done = b.tanks;
    done.delivering_kg = {10.0e3, 2.8e3};  // both at their floors
    CHECK_FALSE(sim::PropellantTransfer::any_line_open(b.request, done));
}

TEST_CASE("An uncovered outlet pauses the transfer and a covered one resumes it", "[refuelling][prop_transfer]") {
    Bench b;
    b.start();
    b.run(400.0, [](double) { return true; });  // flowing
    REQUIRE(b.transfer->state().phase == sim::TransferPhase::Flowing);
    const double before = b.transfer->state().delivered_kg[0];
    b.run(50.0, [](double) { return false; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Paused);
    CHECK(b.transfer->state().status == "Transfer unterbrochen: Treibstoff nicht am Auslass");
    CHECK(b.transfer->state().delivered_kg[0] == before);  // nothing flows unsettled, not even in the pausing step
    b.run(10.0, [](double) { return true; });
    CHECK(b.transfer->state().phase == sim::TransferPhase::Flowing);
}

TEST_CASE("Settling that never covers the outlets pauses; no thrust pauses with its own reason",
          "[refuelling][prop_transfer]") {
    {
        Bench b;
        b.start();
        b.run(200.0, [](double) { return false; });
        CHECK(b.transfer->state().phase == sim::TransferPhase::Paused);
        CHECK(b.transfer->state().delivered_kg[0] == 0.0);
    }
    {
        Bench b;
        b.start();
        b.run(30.0, [](double) { return false; }, false);
        CHECK(b.transfer->state().phase == sim::TransferPhase::Paused);
        CHECK(b.transfer->state().status == "Transfer unterbrochen: kein Kaltgas zum Setzen");
    }
}

TEST_CASE("Stop closes the valves", "[refuelling][prop_transfer]") {
    Bench b;
    b.start();
    b.run(400.0, [](double) { return true; });
    b.transfer->stop();
    CHECK(b.transfer->state().phase == sim::TransferPhase::Stopped);
    CHECK(b.transfer->state().status == "Transfer gestoppt");
    CHECK_FALSE(b.transfer->active());
    const auto out = b.transfer->step({});
    CHECK(out.moved_kg[0] == 0.0);
    CHECK_FALSE(out.thrust);
}

namespace {

double main_kg(const vehicle::FlightBody& body) { return body.vehicle().main_propellant_kg(); }

double port_gap_m(const sim::FlightSession& s) {
    return norm(vehicle::port_kinematics(s.body(0), "transfer_dock").position_gcrf_m -
                vehicle::port_kinematics(s.body(1), "transfer_dock").position_gcrf_m);
}

}  // namespace

TEST_CASE("Docked boil-off: both ships vent through the compound, the hulls stay mated",
          "[refuelling][prop_transfer][boil_off][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    REQUIRE(f.session.docked());
    const double gap0 = port_gap_m(f.session);
    const double tanker0 = main_kg(f.session.body(0));
    const double depot0 = main_kg(f.session.body(1));
    f.session.advance_to(f.session.epoch() + 2.0 * 3600.0, 100000);
    const double tanker_loss = tanker0 - main_kg(f.session.body(0));
    const double depot_loss = depot0 - main_kg(f.session.body(1));
    INFO("tanker " << tanker_loss << " kg, depot " << depot_loss << " kg in 2 h");
    CHECK(tanker_loss > 100.0);  // bare stainless: tonnes per day
    CHECK(depot_loss > 10.0);    // r2: ~450 kg/day full -> ~37 kg in 2 h (600 t: less wall wetted)
    CHECK(depot_loss < 100.0);
    CHECK(std::abs(port_gap_m(f.session) - gap0) < 1e-5);  // r3 M2: the depot no longer vents past the compound
    CHECK(f.session.compound_mass()->mass_kg ==
          Catch::Approx(f.session.body(1).vehicle().mass_properties().mass_kg));
    CHECK(f.session.body(0).boil_off_rate().oxidizer_kgps > 0.0);  // shown for the held tanker too
}

TEST_CASE("Docked boil-off follows the fidelity flag", "[refuelling][prop_transfer][boil_off][docking]") {
    test::spice_session();
    refuelling_test::TankerAtPort f({0.0, 0.0, 0.0});
    f.session.set_boil_off(false);
    f.session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
    f.session.dock_in_place();
    const double tanker0 = main_kg(f.session.body(0));
    const double depot0 = main_kg(f.session.body(1));
    f.session.advance_to(f.session.epoch() + 3600.0, 100000);
    CHECK(main_kg(f.session.body(0)) == tanker0);
    CHECK(main_kg(f.session.body(1)) == depot0);
}

namespace {

struct DockedPair : refuelling_test::TankerAtPort {
    explicit DockedPair(const config::VehicleConfig& chaser = refuelling_test::tanker_config(200.0e3))
        : TankerAtPort({0.0, 0.0, 0.0}, {}, chaser) {
        session.set_rendezvous_target(1, "transfer_dock", "transfer_dock");
        session.dock_in_place();
    }
    double tank(size_t body, const std::string& id) const {
        for (const auto& k : session.body(body).vehicle().stages()[0].tanks) {
            if (k.config.id == id) return k.mass_kg;
        }
        return 0.0;
    }
    double all_propellant() const {
        double kg = 0.0;
        for (size_t b = 0; b < 2; ++b) {
            for (const auto& k : session.body(b).vehicle().stages()[0].tanks) kg += k.mass_kg;
        }
        return kg;
    }
    bool is_running() const {
        const auto p = session.transfer_state().phase;
        return p == sim::TransferPhase::Settling || p == sim::TransferPhase::Chilldown ||
               p == sim::TransferPhase::Flowing || p == sim::TransferPhase::Paused;
    }
    void run_to_end(double limit_s, double chunk_s) {
        for (double t = 0.0; t < limit_s && is_running(); t += chunk_s) {
            session.advance_to(session.epoch() + chunk_s, 1000000);
        }
    }
};

sim::TransferRequest tanker_request() {
    sim::TransferRequest r;
    r.delivering_floor_kg = {10.0e3, 2.8e3};  // [test] the reserve of Task 6 is ~this size
    return r;
}

}  // namespace

TEST_CASE("Tanker to depot: settle, chill, flow to the reserve; masses close, hulls stay mated",
          "[refuelling][prop_transfer][docking]") {
    test::spice_session();
    DockedPair f;
    REQUIRE(f.session.docked());
    const double lox0 = f.tank(0, "lox_main"), depot_lox0 = f.tank(1, "lox_main");
    const double total0 = f.all_propellant();
    const double gap0 = port_gap_m(f.session);
    REQUIRE(f.session.start_transfer(tanker_request()));
    CHECK(f.session.transfer_state().phase == sim::TransferPhase::Settling);
    f.run_to_end(3600.0, 60.0);
    const auto& s = f.session.transfer_state();
    INFO(s.status << ": " << s.elapsed_s << " s, LOX " << s.delivered_kg[0] << " CH4 " << s.delivered_kg[1]
                  << " gas " << s.settling_gas_kg << " chill " << s.chilldown_kg[0] + s.chilldown_kg[1]
                  << " boil-off " << s.boil_off_kg << " thrust " << s.settling_thrust_n);
    REQUIRE(s.phase == sim::TransferPhase::Complete);
    // the LOX line closed last at its floor; CH4 paid a little gas after its line closed
    CHECK(f.tank(0, "lox_main") == Catch::Approx(10.0e3).margin(50.0));
    CHECK(f.tank(0, "ch4_main") == Catch::Approx(2.8e3).margin(200.0));
    CHECK(f.tank(0, "ch4_main") > 2.5e3);
    // every kilogram is accounted for: delivered stays in the pair, the rest left it
    CHECK(total0 - f.all_propellant() ==
          Catch::Approx(s.settling_gas_kg + s.chilldown_kg[0] + s.chilldown_kg[1] + s.boil_off_kg).margin(1.0));
    CHECK(f.tank(1, "lox_main") - depot_lox0 > s.delivered_kg[0] - 50.0);  // minus the depot's boil-off
    CHECK(s.delivered_kg[0] > 140.0e3);
    CHECK(s.delivered_kg[0] < lox0 - 10.0e3);
    // ~190 s settling + 120 s chill-down + ~630 s LOX flow (232 kg/s)
    CHECK(s.elapsed_s == Catch::Approx(940.0).margin(40.0));
    // 1e-4 g on ~1,360 t: ~1.33 kN for ~940 s at Isp 70 s
    CHECK(s.settling_thrust_n == Catch::Approx(1.0e-4 * 9.80665 * f.session.compound_mass()->mass_kg).epsilon(0.02));
    CHECK(s.settling_gas_kg == Catch::Approx(1.8e3).epsilon(0.15));
    CHECK(std::abs(port_gap_m(f.session) - gap0) < 1e-5);
    CHECK(f.session.docked());
}

TEST_CASE("Transfer results do not depend on how the time is advanced", "[refuelling][prop_transfer][docking]") {
    test::spice_session();
    DockedPair a, b;
    REQUIRE(a.session.start_transfer(tanker_request()));
    REQUIRE(b.session.start_transfer(tanker_request()));
    a.run_to_end(3600.0, 1.0);
    b.run_to_end(3600.0, 1200.0);
    REQUIRE(a.session.transfer_state().phase == sim::TransferPhase::Complete);
    REQUIRE(b.session.transfer_state().phase == sim::TransferPhase::Complete);
    for (size_t k = 0; k < 2; ++k) {
        CHECK(a.session.transfer_state().delivered_kg[k] ==
              Catch::Approx(b.session.transfer_state().delivered_kg[k]).epsilon(1e-4));
    }
    CHECK(a.session.transfer_state().elapsed_s == Catch::Approx(b.session.transfer_state().elapsed_s).margin(1e-6));
}

TEST_CASE("Depot to HLS: the limits end the transfer", "[refuelling][prop_transfer][docking][hls]") {
    test::spice_session();
    DockedPair f(refuelling_test::ship_with("starship_hls", 50.0e3));
    REQUIRE(f.session.docked());
    const double hls_lox0 = f.tank(0, "lox_main");
    sim::TransferRequest r;
    r.direction = sim::TransferDirection::StationToChaser;
    r.limit_kg = {20.0e3, 5.5e3};
    REQUIRE(f.session.start_transfer(r));
    f.run_to_end(3600.0, 60.0);
    const auto& s = f.session.transfer_state();
    INFO(s.status << ": " << s.elapsed_s << " s, gas " << s.settling_gas_kg);
    REQUIRE(s.phase == sim::TransferPhase::Complete);
    CHECK(s.delivered_kg[0] == Catch::Approx(20.0e3).margin(1.0));
    CHECK(s.delivered_kg[1] == Catch::Approx(5.5e3).margin(1.0));
    // the HLS pays the settling gas from its own main tanks (it has no [stage.thermal]: no boil-off)
    CHECK(f.tank(0, "lox_main") - hls_lox0 == Catch::Approx(20.0e3 - s.settling_gas_kg * 3.6 / 4.6).epsilon(0.02));
}

TEST_CASE("Transfer refusals and stop", "[refuelling][prop_transfer][docking]") {
    test::spice_session();
    {
        refuelling_test::TankerAtPort f({20.0, 0.0, 0.0});
        CHECK_FALSE(f.session.start_transfer(tanker_request()));
        CHECK(f.session.transfer_state().status == "Transfer nicht möglich: nicht angedockt");
    }
    {
        DockedPair f(refuelling_test::tanker_config(10.0e3));  // 7.8 t LOX / 2.2 t CH4: below the floors
        CHECK_FALSE(f.session.start_transfer(tanker_request()));
        CHECK(f.session.transfer_state().status == "Transfer nicht möglich: nichts zu übertragen");
    }
    {
        DockedPair f;
        REQUIRE(f.session.start_transfer(tanker_request()));
        CHECK_FALSE(f.session.start_transfer(tanker_request()));
        CHECK(f.session.docking_status() == "Transfer läuft bereits");
        CHECK(f.session.transfer_state().phase == sim::TransferPhase::Settling);  // the running transfer is kept
        CHECK_FALSE(f.session.undock());
        CHECK(f.session.docking_status() == "Abkoppeln nicht möglich: Treibstofftransfer läuft");
        f.session.advance_to(f.session.epoch() + 400.0, 1000000);
        REQUIRE(f.session.transfer_state().phase == sim::TransferPhase::Flowing);
        f.session.stop_transfer();
        CHECK(f.session.transfer_state().phase == sim::TransferPhase::Stopped);
        const double lox = f.tank(0, "lox_main");
        f.session.advance_to(f.session.epoch() + 60.0, 1000000);
        CHECK(f.tank(0, "lox_main") > lox - 50.0);  // only boil-off, no flow, no gas
        CHECK(f.session.undock());
    }
}

TEST_CASE("Zero settling push: the liquid stays forward, the transfer pauses", "[refuelling][prop_transfer][docking]") {
    test::spice_session();
    DockedPair f;
    auto r = tanker_request();
    r.settling_acceleration_mps2 = 0.0;
    REQUIRE(f.session.start_transfer(r));
    f.session.advance_to(f.session.epoch() + 60.0, 1000000);
    CHECK(f.session.transfer_state().phase == sim::TransferPhase::Paused);
    CHECK(f.session.transfer_state().status == "Transfer unterbrochen: Treibstoff nicht am Auslass");
    CHECK(f.session.transfer_state().delivered_kg[0] == 0.0);
}

TEST_CASE("Tanker reserve: the deorbit burn by the rocket equation plus gas, boil-off and residue",
          "[refuelling][prop_transfer][mission]") {
    mission::TankerReserveParameters p;
    p.deorbit_delta_v_mps = 80.0;
    p.exhaust_velocity_mps = 3300.0;
    const auto r = mission::tanker_reserve(290.0e3, p);
    // after the burn 290 + 1.5 + 1 t; burn = 292.5 t (e^(80/3300) - 1) x 1.10 = 7.8955 t
    CHECK(r.burn_kg == Catch::Approx(7895.5).epsilon(1e-4));
    CHECK(r.total_kg == Catch::Approx(r.burn_kg + 1500.0 + 2000.0 + 1000.0));
    CHECK(r.oxidizer_kg == Catch::Approx(r.total_kg * 3.6 / 4.6));
    CHECK(r.fuel_kg == Catch::Approx(r.total_kg / 4.6));
}

TEST_CASE("Docked tanker: its reserve from the previewed return, as a transfer request",
          "[refuelling][prop_transfer][docking][deorbit]") {
    test::spice_session();
    DockedPair f;
    std::string reason;
    gnc::DeorbitParameters search;  // the depot plane meets Starbase within reach only every few days (r3)
    search.search_window_s = sim::FlightSession::tanker_reserve_search_s;
    const auto plan = f.session.ship_return_preview(search);
    INFO(plan.reason);
    REQUIRE(plan.feasible);
    const auto reserve = f.session.tanker_reserve({}, &reason);
    INFO(reason);
    REQUIRE(reserve);
    INFO("deorbit " << plan.delta_v_mps << " m/s, reserve " << reserve->total_kg << " kg (burn " << reserve->burn_kg
                    << ")");
    CHECK(plan.delta_v_mps > 30.0);
    CHECK(plan.delta_v_mps < 200.0);
    CHECK(reserve->total_kg > 8.0e3);
    CHECK(reserve->total_kg < 30.0e3);
    const auto request = f.session.tanker_transfer_request({}, &reason);
    REQUIRE(request);
    CHECK(request->direction == sim::TransferDirection::ChaserToStation);
    CHECK(request->delivering_floor_kg[0] == Catch::Approx(reserve->oxidizer_kg));
    CHECK(request->delivering_floor_kg[1] == Catch::Approx(reserve->fuel_kg));
    // not docked: no reserve
    refuelling_test::TankerAtPort free({20.0, 0.0, 0.0});
    CHECK_FALSE(free.session.tanker_reserve({}, &reason));
    CHECK(reason == "Keine Reserve: nicht angedockt");
}
