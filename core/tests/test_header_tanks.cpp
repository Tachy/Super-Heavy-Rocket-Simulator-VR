#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using Catch::Approx;
using config::TankContents;
using config::TankRole;
using vehicle::FeedSource;
using vehicle::Vehicle;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

config::VehicleConfig ship_config(double propellant_kg) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], propellant_kg);
    return cfg;
}

double tank_kg(const config::StageConfig& s, TankContents contents, TankRole role) {
    double kg = 0.0;
    for (const auto& t : s.tanks) kg += t.contents == contents && t.role == role ? t.mass_kg : 0.0;
    return kg;
}

// Flow per tank role of stage 0 (oxidizer + fuel).
double flow_kgps(const Vehicle& v, TankRole role) {
    const auto flows = v.tank_mass_flows_kgps();
    double kgps = 0.0;
    for (size_t i = 0; i < v.stages()[0].tanks.size(); ++i) {
        kgps += v.stages()[0].tanks[i].config.role == role ? flows[0][i] : 0.0;
    }
    return kgps;
}

void run_first_engine(Vehicle& v) {
    auto& e = v.stages()[0].engines.front();
    e.mode = vehicle::EngineMode::Running;
    e.spool = 1.0;
    e.throttle = 1.0;
}

constexpr double ox_share = 3.6 / 4.6;  // O/F 3.6

}  // namespace

TEST_CASE("Starship's header tanks hold the landing propellant", "[header]") {
    const auto cfg = config::load_vehicle(config_dir, "starship_block3");
    const auto& ship = cfg.stages[1];
    CHECK(tank_kg(ship, TankContents::Oxidizer, TankRole::Header) == Approx(35000.0 * ox_share).margin(1.0));
    CHECK(tank_kg(ship, TankContents::Fuel, TankRole::Header) == Approx(35000.0 * (1.0 - ox_share)).margin(1.0));
    CHECK(ship.landing_propellant_kg == Approx(35000.0).margin(1.0));
    double total = 0.0;
    for (const auto& t : ship.tanks) total += t.mass_kg;
    CHECK(total == Approx(1.6e6).margin(1.0));  // the headers are part of the 1,600 t
    for (const auto& t : ship.tanks) {
        if (t.id == "lox_header") CHECK(t.diameter_m == 3.6);
        if (t.role == TankRole::Main) CHECK(t.diameter_m == 0.0);
    }
    // The booster has none: its landing propellant stays configured.
    CHECK(cfg.stages[0].landing_propellant_kg == 51000.0);
}

TEST_CASE("A landing propellant that differs from the header contents is a config error", "[header]") {
    const auto dir = std::filesystem::temp_directory_path() / "starsim_header_mismatch_config";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::ifstream in(config_dir / "vehicles" / "starship_block3.toml");
    std::stringstream text;
    text << in.rdbuf();
    std::string toml = text.str();
    const std::string anchor = "# landing_propellant_kg: the header tanks' contents";
    REQUIRE(toml.find(anchor) != std::string::npos);
    toml.replace(toml.find(anchor), anchor.size(), "landing_propellant_kg = 30000.0\n#");
    std::ofstream(dir / "vehicles" / "mismatch.toml") << toml;
    CHECK_THROWS(config::load_vehicle(dir, "mismatch"));
    std::filesystem::remove_all(dir);
}

TEST_CASE("fill_tanks fills the headers first", "[header]") {
    const auto heavy = ship_config(64000.0).stages[0];
    CHECK(tank_kg(heavy, TankContents::Oxidizer, TankRole::Header) == Approx(35000.0 * ox_share).margin(1.0));
    CHECK(tank_kg(heavy, TankContents::Oxidizer, TankRole::Main) == Approx(29000.0 * ox_share).margin(1.0));
    CHECK(tank_kg(heavy, TankContents::Fuel, TankRole::Main) == Approx(29000.0 * (1.0 - ox_share)).margin(1.0));

    const auto light = ship_config(20000.0).stages[0];  // below the header capacity: all in the headers
    CHECK(tank_kg(light, TankContents::Oxidizer, TankRole::Header) == Approx(20000.0 * ox_share).margin(1.0));
    CHECK(tank_kg(light, TankContents::Oxidizer, TankRole::Main) == 0.0);
    CHECK(tank_kg(light, TankContents::Fuel, TankRole::Main) == 0.0);

    // Per propellant (a gate replay's oxidizer and fuel).
    auto replay = ship_config(0.0).stages[0];
    config::fill_tanks(replay, 40000.0, 10000.0);
    CHECK(tank_kg(replay, TankContents::Oxidizer, TankRole::Header) == Approx(35000.0 * ox_share).margin(1.0));
    CHECK(tank_kg(replay, TankContents::Oxidizer, TankRole::Main) == Approx(40000.0 - 35000.0 * ox_share).margin(1.0));
    CHECK(tank_kg(replay, TankContents::Fuel, TankRole::Main) == Approx(10000.0 - 35000.0 * (1.0 - ox_share)).margin(1.0));

    // Without headers (the booster): every tank scaled alike, as before.
    auto booster = config::load_vehicle(config_dir, "starship_block3").stages[0];
    const double lox_full = booster.tanks[0].capacity_kg, total_full = lox_full + booster.tanks[1].capacity_kg;
    config::fill_tanks(booster, 0.1 * total_full);
    CHECK(booster.tanks[0].mass_kg == Approx(0.1 * lox_full));
}

TEST_CASE("Engines draw from the selected feed source", "[header]") {
    Vehicle ship(ship_config(64000.0), config_dir);
    run_first_engine(ship);
    CHECK(flow_kgps(ship, TankRole::Main) > 0.0);
    CHECK(flow_kgps(ship, TankRole::Header) == 0.0);
    ship.select_feed(0, FeedSource::Header);
    CHECK(ship.stages()[0].feed == FeedSource::Header);
    CHECK(flow_kgps(ship, TankRole::Main) == 0.0);
    CHECK(flow_kgps(ship, TankRole::Header) > 0.0);
    CHECK(ship.main_propellant_kg() == Approx(29000.0).margin(1.0));
    CHECK(ship.propellant_kg() == Approx(64000.0).margin(1.0));
}

TEST_CASE("A dry feed source flames out and the headers relight it", "[header]") {
    Vehicle ship(ship_config(20000.0), config_dir);  // main tanks empty
    run_first_engine(ship);
    ship.consume_propellant(0.01);
    CHECK(ship.stages()[0].engines.front().starved);
    CHECK(ship.stages()[0].engines.front().mode == vehicle::EngineMode::Off);
    ship.select_feed(0, FeedSource::Header);
    CHECK_FALSE(ship.stages()[0].engines.front().starved);
}

TEST_CASE("RCS ullage gas comes from the main tanks, from the headers once they are dry", "[header]") {
    for (double kg : {64000.0, 20000.0}) {
        Vehicle ship(ship_config(kg), config_dir);
        ship.stages()[0].rcs.front().output = 1.0;
        const bool main_left = kg > 35000.0;
        CHECK((flow_kgps(ship, TankRole::Main) > 0.0) == main_left);
        CHECK((flow_kgps(ship, TankRole::Header) > 0.0) == !main_left);
    }
}

TEST_CASE("Selecting the headers on a stage without them changes nothing", "[header]") {
    Vehicle stack(config::load_vehicle(config_dir, "starship_block3"), config_dir);
    stack.select_feed(0, FeedSource::Header);  // the booster
    CHECK(stack.stages()[0].feed == FeedSource::Main);
}

// Header-tank plan Task 3: the dry CG keeps the validated landing-mass CG. Before the headers the 35 t sat in the
// main tanks as columns on their bottoms, dry CG 28 m [D] (belly-first trim).
TEST_CASE("The dry CG keeps the landing-mass CG of the validated entries", "[header]") {
    const auto cfg = ship_config(35000.0);
    const Vehicle ship(cfg, config_dir);
    const double area = constants::pi * 0.25 * 9.0 * 9.0;
    const double lox = 35000.0 * ox_share, ch4 = 35000.0 - lox;
    const double lox_cg = 3.5 + 0.5 * lox / (cfg.propellant.oxidizer_density_kg_m3 * area);
    const double ch4_cg = 20.0 + 0.5 * ch4 / (cfg.propellant.fuel_density_kg_m3 * area);
    const double dry = cfg.stages[0].dry_mass_kg;
    const double before = (dry * 28.0 + lox * lox_cg + ch4 * ch4_cg) / (dry + 35000.0);
    CHECK(ship.mass_properties().cg_station_m == Approx(before).margin(0.01));
}
