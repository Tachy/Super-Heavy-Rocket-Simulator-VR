// Refuelling r2: propellant boil-off by a heat balance on the hull.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The top stage of a vehicle flying alone (the ship after staging).
std::unique_ptr<vehicle::Vehicle> top_stage_alone(const std::string& vehicle_id) {
    auto cfg = config::load_vehicle(config_dir, vehicle_id);
    cfg.stages.erase(cfg.stages.begin(), cfg.stages.end() - 1);
    return std::make_unique<vehicle::Vehicle>(cfg, config_dir);
}

const vehicle::Tank& tank_named(const vehicle::Vehicle& v, const std::string& id) {
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.id == id) return t;
    }
    throw std::runtime_error("no tank " + id);
}

}  // namespace

TEST_CASE("plate view factor to a sphere: nadir, horizon and the orientation average", "[boil_off][environment]") {
    const double h = (constants::earth_mean_radius_m + 200.0e3) / constants::earth_mean_radius_m;
    CHECK(environment::plate_view_factor_to_sphere(0.0, h) == Catch::Approx(1.0 / (h * h)).epsilon(1e-12));

    // Continuous where the sphere starts to sink below the plate's horizon, and zero once it is gone.
    const double full = std::acos(1.0 / h);
    CHECK(environment::plate_view_factor_to_sphere(full - 1e-9, h) ==
          Catch::Approx(environment::plate_view_factor_to_sphere(full + 1e-9, h)).margin(1e-7));
    const double gone = constants::pi - full;
    CHECK(environment::plate_view_factor_to_sphere(gone - 1e-9, h) == Catch::Approx(0.0).margin(1e-7));
    CHECK(environment::plate_view_factor_to_sphere(gone + 0.1, h) == 0.0);

    // Over all orientations a plate sees the sphere as a small sphere does: (1 - cos rho) / 2, sin rho = 1 / h.
    constexpr int n = 20000;
    double sum = 0.0;
    for (int k = 0; k < n; ++k) {
        sum += environment::plate_view_factor_to_sphere(std::acos(1.0 - 2.0 * (k + 0.5) / n), h);
    }
    CHECK(sum / n == Catch::Approx(0.5 * (1.0 - std::sqrt(1.0 - 1.0 / (h * h)))).epsilon(1e-6));
}

TEST_CASE("earth orbit radiation: sunlit noon side and umbra", "[boil_off][environment]") {
    const Vec3 position{constants::earth_mean_radius_m + 400.0e3, 0.0, 0.0};
    const Vec3 sun_gcrf{constants::astronomical_unit_m, 0.0, 0.0};

    const auto noon = environment::earth_orbit_radiation(position, sun_gcrf - position, nullptr);
    CHECK(noon.sunlit_fraction == 1.0);
    CHECK(noon.albedo_phase == Catch::Approx(1.0));
    CHECK(noon.solar_flux_w_m2 == Catch::Approx(constants::solar_constant_w_m2).epsilon(1e-4));
    CHECK(noon.nadir_direction.x == Catch::Approx(-1.0));
    CHECK(noon.distance_over_radius == Catch::Approx(position.x / constants::earth_mean_radius_m));

    const auto midnight = environment::earth_orbit_radiation(-position, sun_gcrf + position, nullptr);
    CHECK(midnight.sunlit_fraction == 0.0);
    CHECK(midnight.albedo_phase == 0.0);
}

TEST_CASE("absorbed flux of a flat element: sun, albedo and Earth infrared", "[boil_off][environment]") {
    environment::RadiationEnvironment env;
    env.sun_direction = {0.0, 1.0, 0.0};
    env.solar_flux_w_m2 = constants::solar_constant_w_m2;
    env.sunlit_fraction = 1.0;
    env.nadir_direction = {-1.0, 0.0, 0.0};
    env.distance_over_radius = 1.1;
    env.albedo_phase = 0.5;

    // Facing the Sun, edge-on to the Earth (the sphere half below its horizon).
    const double side = environment::plate_view_factor_to_sphere(constants::pi / 2.0, 1.1);
    CHECK(environment::absorbed_flux_w_m2({0.0, 1.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * (constants::solar_constant_w_m2 +
                               constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * side) +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * side));
    // Facing the Earth: no direct sun, the full view factor 1 / h^2.
    const double down = 1.0 / (1.1 * 1.1);
    CHECK(environment::absorbed_flux_w_m2({-1.0, 0.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * down +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * down));
    // In the umbra the direct sun is gone, albedo and infrared stay (the Earth below is still lit at phase 0.5).
    env.sunlit_fraction = 0.0;
    CHECK(environment::absorbed_flux_w_m2({0.0, 1.0, 0.0}, env, 0.4, 0.8) ==
          Catch::Approx(0.4 * constants::earth_bond_albedo * constants::solar_constant_w_m2 * 0.5 * side +
                        0.8 * constants::earth_outgoing_longwave_w_m2 * side));
}

TEST_CASE("methalox boils at its normal boiling points with its latent heats", "[boil_off][config]") {
    const auto ship = top_stage_alone("starship_block3");
    const auto& lox = tank_named(*ship, "lox_main");
    const auto& ch4 = tank_named(*ship, "ch4_main");
    CHECK(lox.boiling_point_k == Catch::Approx(90.19));
    CHECK(lox.latent_heat_j_kg == Catch::Approx(213.1e3));
    CHECK(ch4.boiling_point_k == Catch::Approx(111.67));
    CHECK(ch4.latent_heat_j_kg == Catch::Approx(510.8e3));
    // Storable propellants do not boil off.
    const auto dragon = top_stage_alone("crew_dragon");
    for (const auto& t : dragon->stages().back().tanks) CHECK(t.latent_heat_j_kg == 0.0);
}

TEST_CASE("stage.thermal: Block 3 ship with tiles, the tanker inherits it, the depot is insulated", "[boil_off][config]") {
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    REQUIRE(block3.stages.back().thermal.has_value());
    const auto& ship = *block3.stages.back().thermal;
    CHECK(ship.hull.absorptivity == Catch::Approx(0.40));
    CHECK(ship.hull.emissivity == Catch::Approx(0.15));
    CHECK(ship.hull.wall_resistance_m2k_w == Catch::Approx(0.01));
    REQUIRE(ship.shield.has_value());
    CHECK(ship.shield->wall_resistance_m2k_w == Catch::Approx(1.5));
    CHECK(ship.shield_half_width_rad == Catch::Approx(constants::pi / 2.0));
    CHECK_FALSE(block3.stages.front().thermal.has_value());  // the booster lives minutes: no boil-off model

    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    REQUIRE(tanker.stages.back().thermal.has_value());
    CHECK(tanker.stages.back().thermal->hull.absorptivity == Catch::Approx(0.40));

    const auto depot = config::load_vehicle(config_dir, "starship_depot");
    REQUIRE(depot.stages.back().thermal.has_value());
    CHECK(depot.stages.back().thermal->hull.wall_resistance_m2k_w == Catch::Approx(63.7));
    CHECK_FALSE(depot.stages.back().thermal->shield.has_value());
}

TEST_CASE("stage.thermal rejects optics outside 0..1", "[boil_off][config]") {
    const auto dir = std::filesystem::temp_directory_path() / "starsim_boil_off_bad_thermal";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::filesystem::copy(config_dir, dir, std::filesystem::copy_options::recursive |
                                               std::filesystem::copy_options::overwrite_existing);
    const auto path = dir / "vehicles" / "starship_depot.toml";
    std::ifstream in(path);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    const std::string good = "absorptivity = 0.30";
    REQUIRE(text.find(good) != std::string::npos);
    text.replace(text.find(good), good.size(), "absorptivity = 1.30");
    std::ofstream(path) << text;
    CHECK_THROWS_AS(config::load_vehicle(dir, "starship_depot"), config::ConfigError);
    std::filesystem::remove_all(dir);
}
