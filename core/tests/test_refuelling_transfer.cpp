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

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/propellant_transfer.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"
#include "starsim/vehicle/vehicle.hpp"

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
