// Refuelling r2: propellant boil-off by a heat balance on the hull.

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/vehicle/boil_off.hpp"
#include "starsim/vehicle/flight_body.hpp"
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

namespace {

// Far from the Earth (no albedo, no infrared), the Sun along `sun` (unit, body frame = environment frame).
environment::RadiationEnvironment deep_space_sun(const Vec3& sun) {
    environment::RadiationEnvironment env;
    env.sun_direction = sun;
    env.solar_flux_w_m2 = constants::solar_constant_w_m2;
    env.sunlit_fraction = 1.0;
    env.nadir_direction = {-1.0, 0.0, 0.0};
    env.distance_over_radius = 1.0e6;
    env.albedo_phase = 0.0;
    return env;
}

bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

}  // namespace

TEST_CASE("strip balance: conducting wall passes the absorbed flux, an insulating one none", "[boil_off][vehicle]") {
    const double tb = 90.19, absorbed = 500.0, eps = 0.15;
    const double sigma_tb4 = eps * constants::stefan_boltzmann_w_m2k4 * std::pow(tb, 4);
    // Nearly no resistance: the skin sits at the boiling point, everything but its own tiny emission goes in.
    CHECK(vehicle::strip_heat_leak_w_m2(absorbed, {0.4, eps, 1e-6}, tb) ==
          Catch::Approx(absorbed - sigma_tb4).epsilon(1e-4));
    // Nearly perfect insulation: radiative equilibrium skin, leak (T - Tb) / R.
    const double r = 1e9;
    const double t_eq = std::pow(absorbed / (eps * constants::stefan_boltzmann_w_m2k4), 0.25);
    CHECK(vehicle::strip_heat_leak_w_m2(absorbed, {0.4, eps, r}, tb) * r + tb == Catch::Approx(t_eq).epsilon(1e-6));
    // Nothing absorbed: the wall loses heat (negative leak).
    CHECK(vehicle::strip_heat_leak_w_m2(0.0, {0.4, eps, 0.01}, tb) < 0.0);
}

TEST_CASE("cylinder broadside to the Sun absorbs alpha S D L", "[boil_off][vehicle]") {
    const config::ThermalConfig bare{{0.4, 0.15, 1e-6}, std::nullopt, 0.0};
    const vehicle::HullSection section{4.5, 10.0};
    const double tb = 90.19;
    const auto env = deep_space_sun({0.0, 1.0, 0.0});
    const double emitted = 0.15 * constants::stefan_boltzmann_w_m2k4 * std::pow(tb, 4) * constants::two_pi * 4.5 * 10.0;
    CHECK(vehicle::section_heat_w(section, bare, 1.5 * constants::pi, tb, env, {}) ==
          Catch::Approx(0.4 * constants::solar_constant_w_m2 * 9.0 * 10.0 - emitted).epsilon(5e-3));
    // End-on: the walls see no sun and lose their emission - negative.
    CHECK(vehicle::section_heat_w(section, bare, 1.5 * constants::pi, tb, deep_space_sun({1.0, 0.0, 0.0}), {}) ==
          Catch::Approx(-emitted).epsilon(1e-3));
}

TEST_CASE("boil_off_rates: hull main tanks only, heat over latent heat, clamped at zero", "[boil_off][vehicle]") {
    const auto ship = top_stage_alone("starship_block3");
    const auto sunlit = vehicle::boil_off_rates(*ship, deep_space_sun({0.0, 0.0, 1.0}), {});  // sun on the dorsal side
    REQUIRE(sunlit.size() == 2);  // lox_main, ch4_main - no headers
    for (const auto& t : sunlit) {
        const auto& tank = ship->stages()[t.stage_index].tanks[t.tank_index];
        CHECK(tank.config.role == config::TankRole::Main);
        CHECK(t.heat_w > 0.0);
        CHECK(t.mass_flow_kgps == Catch::Approx(t.heat_w / tank.latent_heat_j_kg));
    }
    // Sun along the axis: the walls lose heat - no condensation, flows clamp at 0.
    for (const auto& t : vehicle::boil_off_rates(*ship, deep_space_sun({1.0, 0.0, 0.0}), {})) {
        CHECK(t.heat_w == 0.0);
        CHECK(t.mass_flow_kgps == 0.0);
    }
    const auto total = vehicle::total_boil_off(*ship, sunlit);
    CHECK(total.oxidizer_kgps > 0.0);
    CHECK(total.fuel_kgps > 0.0);
}

// Task 5b (user 2026-10-20): the wall passes its heat into the liquid only where the liquid wets it. Where the liquid
// lies decides the Bond number: settled by a residual acceleration, or spread over the walls by surface tension.
TEST_CASE("settled liquid wets the strips below its surface", "[boil_off][vehicle]") {
    const auto aft = vehicle::settled_wetted_fractions(0.25, {-1.0, 0.0, 0.0}, 3.0);
    REQUIRE(aft.size() == static_cast<size_t>(vehicle::boil_off_strips));
    for (const double f : aft) CHECK(f == Catch::Approx(0.25).margin(0.03));  // a column on the aft end
    // Pushed to the belly (-z): the belly half wet over its length, the dorsal half dry.
    const auto side = vehicle::settled_wetted_fractions(0.5, {0.0, 0.0, -1.0}, 3.0);
    const int belly = static_cast<int>(0.75 * vehicle::boil_off_strips);  // azimuth 270 deg
    const int dorsal = static_cast<int>(0.25 * vehicle::boil_off_strips);
    CHECK(side[belly] == Catch::Approx(1.0));
    CHECK(side[dorsal] == Catch::Approx(0.0));
    for (const double f : vehicle::settled_wetted_fractions(0.0, {-1.0, 0.0, 0.0}, 3.0)) CHECK(f == 0.0);
    for (const double f : vehicle::settled_wetted_fractions(1.0, {-1.0, 0.0, 0.0}, 3.0)) CHECK(f == 1.0);
}

TEST_CASE("Bond number: surface tension wets the walls, a residual acceleration settles", "[boil_off][vehicle]") {
    CHECK(vehicle::settled_weight(0.3) == 0.0);
    CHECK(vehicle::settled_weight(1.0) == 0.0);
    CHECK(vehicle::settled_weight(std::sqrt(10.0)) == Catch::Approx(0.5));
    CHECK(vehicle::settled_weight(10.0) == 1.0);
    CHECK(vehicle::settled_weight(1.0e4) == 1.0);
}

TEST_CASE("apparent gravity on a tank: tide, specific force and rotation", "[boil_off][vehicle]") {
    const double r = constants::earth_mean_radius_m + 200.0e3;
    const double n2 = constants::earth_gm_m3ps2 / (r * r * r);
    vehicle::LiquidEnvironment liquid;
    liquid.position_gcrf_m = {r, 0.0, 0.0};
    // Tide: stretched along the radius, squeezed across it.
    const Vec3 radial = vehicle::apparent_gravity_body_mps2(liquid, {10.0, 0.0, 0.0});
    CHECK(radial.x == Catch::Approx(2.0 * n2 * 10.0));
    const Vec3 across = vehicle::apparent_gravity_body_mps2(liquid, {0.0, 10.0, 0.0});
    CHECK(across.y == Catch::Approx(-n2 * 10.0));
    // Thrust forward: the liquid goes aft. Rotation about z: outwards.
    liquid.position_gcrf_m = {1.0e15, 0.0, 0.0};  // no tide
    liquid.specific_force_body_mps2 = {2.0, 0.0, 0.0};
    CHECK(vehicle::apparent_gravity_body_mps2(liquid, {}).x == Catch::Approx(-2.0));
    liquid.specific_force_body_mps2 = {};
    liquid.angular_velocity_body_radps = {0.0, 0.0, 0.1};
    CHECK(vehicle::apparent_gravity_body_mps2(liquid, {10.0, 0.0, 0.0}).x == Catch::Approx(0.1 * 0.1 * 10.0));
}

TEST_CASE("boil_off_rates: a small settled residue boils little, in free fall the walls are wet", "[boil_off][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    ship->set_tank_mass(0, "lox_main", 0.01 * tank_named(*ship, "lox_main").config.capacity_kg);
    const auto env = deep_space_sun({0.0, 0.0, 1.0});
    auto lox_heat = [&](const vehicle::LiquidEnvironment* liquid) {
        for (const auto& t : vehicle::boil_off_rates(*ship, env, {}, liquid)) {
            if (ship->stages()[t.stage_index].tanks[t.tank_index].config.id == "lox_main") return t.heat_w;
        }
        return 0.0;
    };
    const double wetted = lox_heat(nullptr);
    vehicle::LiquidEnvironment thrust;
    thrust.position_gcrf_m = {1.0e15, 0.0, 0.0};
    thrust.specific_force_body_mps2 = {1.0, 0.0, 0.0};  // settled aft: 1 % of the tank's length wet
    CHECK(lox_heat(&thrust) > 0.0);
    CHECK(lox_heat(&thrust) < 0.1 * wetted);
    vehicle::LiquidEnvironment free_fall;
    free_fall.position_gcrf_m = {1.0e15, 0.0, 0.0};  // Bond number ~0: surface tension coats the walls
    CHECK(lox_heat(&free_fall) == Catch::Approx(wetted));
}

TEST_CASE("boil_off_rates: tiles on the belly, MLI on the depot, nothing without stage.thermal", "[boil_off][vehicle]") {
    const auto ship = top_stage_alone("starship_block3");
    auto heat = [](const vehicle::Vehicle& v, const Vec3& sun) {
        double sum = 0.0;
        for (const auto& t : vehicle::boil_off_rates(v, deep_space_sun(sun), {})) sum += t.heat_w;
        return sum;
    };
    CHECK(heat(*ship, {0.0, 0.0, -1.0}) < 0.5 * heat(*ship, {0.0, 0.0, 1.0}));  // belly (tiles) vs dorsal (bare)
    const auto depot = top_stage_alone("starship_depot");
    CHECK(heat(*depot, {0.0, 0.0, 1.0}) < 0.02 * heat(*ship, {0.0, 0.0, 1.0}));
    const auto dragon = top_stage_alone("crew_dragon");
    CHECK(vehicle::boil_off_rates(*dragon, deep_space_sun({0.0, 0.0, 1.0}), {}).empty());
}

namespace {

// Centaur LO2 sidewall in a sunlit park-orbit coast (AIAA 2007-5845): axis along the orbit normal, rolling (the strip
// sum is roll-independent), the Sun in the orbit plane at noon in mid-coast [E]; mean wall flux over the coast from
// perigee. Mirrors tools/validation/centaur_boil_off_calibration.py.
double centaur_wall_flux_w_m2(const config::ThermalSurfaceConfig& surface, double perigee_km, double apogee_km,
                              double coast_min, bool earth) {
    const double re = constants::earth_mean_radius_m, mu = constants::earth_gm_m3ps2;
    const double rp = re + perigee_km * 1e3, ra = re + apogee_km * 1e3;
    const double a = 0.5 * (rp + ra), e = (ra - rp) / (ra + rp), n = std::sqrt(mu / (a * a * a));
    auto at = [&](double t, double& r, double& nu) {
        double ecc_anomaly = n * t;
        for (int k = 0; k < 50; ++k) {
            ecc_anomaly -= (ecc_anomaly - e * std::sin(ecc_anomaly) - n * t) / (1.0 - e * std::cos(ecc_anomaly));
        }
        r = a * (1.0 - e * std::cos(ecc_anomaly));
        nu = 2.0 * std::atan2(std::sqrt(1.0 + e) * std::sin(0.5 * ecc_anomaly),
                              std::sqrt(1.0 - e) * std::cos(0.5 * ecc_anomaly));
    };
    double r_mid = 0.0, nu_mid = 0.0;
    at(0.5 * coast_min * 60.0, r_mid, nu_mid);
    const config::ThermalConfig thermal{surface, std::nullopt, 0.0};
    const vehicle::HullSection section{1.524, 1.0};  // 10 ft tank [S]; the length cancels
    // Body x (the tank axis) -> frame z (the orbit normal); body y/z -> the orbit plane.
    const math::Quat body_to_frame = math::Quat::from_axis_angle({0.0, 1.0, 0.0}, -0.5 * constants::pi);
    constexpr int samples = 60;
    double sum = 0.0;
    for (int k = 0; k < samples; ++k) {
        double r = 0.0, nu = 0.0;
        at((k + 0.5) / samples * coast_min * 60.0, r, nu);
        environment::RadiationEnvironment env;
        env.sun_direction = {std::cos(nu_mid), std::sin(nu_mid), 0.0};
        env.solar_flux_w_m2 = constants::solar_constant_w_m2;
        env.sunlit_fraction = 1.0;  // the reconstructions cover the sunlit part
        env.nadir_direction = {-std::cos(nu), -std::sin(nu), 0.0};
        env.distance_over_radius = earth ? r / re : 1.0e6;
        env.albedo_phase = earth ? std::max(0.0, std::cos(nu - nu_mid)) : 0.0;
        sum += vehicle::section_heat_w(section, thermal, 0.0, 90.19, env, body_to_frame) /
               (constants::two_pi * section.radius_m * section.length_m);
    }
    return sum / samples;
}

constexpr double btu_hr_ft2 = 3.15459;  // W/m^2 [S: NIST SP 811]

}  // namespace

TEST_CASE("Centaur LO2 sidewall flux: calibration and prediction (AIAA 2007-5845 Table 2)", "[boil_off][validation]") {
    // White-painted foam [E: paint alpha 0.25, eps 0.88]; resistance calibrated [D] on AC-206 (23 Btu/hr-ft^2).
    const config::ThermalSurfaceConfig foam{0.25, 0.88, 1.586};
    const double ac206 = centaur_wall_flux_w_m2(foam, 167.0, 1132.0, 58.0, true);
    const double ac167 = centaur_wall_flux_w_m2(foam, 185.0, 267.0, 59.0, true);
    const double av003 = centaur_wall_flux_w_m2(foam, 167.0, 4165.0, 78.0, true);
    // 3-layer MLI [E: cover alpha 0.30, eps 0.80]; calibrated [D] on the TC-11/TC-15 average (3 Btu/hr-ft^2), high
    // elliptical transfer coasts: sun only.
    const double tc = centaur_wall_flux_w_m2({0.30, 0.80, 8.22}, 0.0, 0.0, 1.0, false);
    if (verbose()) {
        std::cout << "Centaur flux W/m^2: AC-206 " << ac206 << " AC-167 " << ac167 << " AV-003 " << av003 << " TC "
                  << tc << "\n";
    }
    CHECK(ac206 == Catch::Approx(23.0 * btu_hr_ft2).epsilon(0.03));  // calibration consistency
    CHECK(tc == Catch::Approx(3.0 * btu_hr_ft2).epsilon(0.03));
    // Validation: other orbits with AC-206's wall. Tolerance 25 % [documented: TC-11 vs TC-15 differ by 2x, coast
    // geometry unknown].
    CHECK(ac167 == Catch::Approx(25.0 * btu_hr_ft2).epsilon(0.25));
    CHECK(av003 == Catch::Approx(25.0 * btu_hr_ft2).epsilon(0.25));
}

namespace {

// Circular orbit of `altitude_m`, inclination `inclination_rad`, node on +x.
math::StateVector circular_orbit(double altitude_m, double inclination_rad) {
    const double r = constants::earth_mean_radius_m + altitude_m;
    const double v = std::sqrt(constants::earth_gm_m3ps2 / r);
    return {{r, 0.0, 0.0}, {0.0, v * std::cos(inclination_rad), v * std::sin(inclination_rad)}};
}

double full_propellant_kg(const vehicle::Vehicle& v) {
    double sum = 0.0;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.contents != config::TankContents::Rcs) sum += t.config.capacity_kg;
    }
    return sum;
}

}  // namespace

TEST_CASE("orbit average: a Sun along the orbit normal gives the constant instantaneous rate", "[boil_off][vehicle]") {
    // Equatorial orbit, Sun at the north pole: no eclipse, and in LVLH the Sun and the Earth stand still.
    const auto ship = top_stage_alone("starship_tanker");
    const auto orbit = circular_orbit(300.0e3, 0.0);
    const Vec3 sun{0.0, 0.0, constants::astronomical_unit_m};
    const auto average = vehicle::orbit_average_boil_off(*ship, orbit, {}, sun);
    const auto attitude = frames::lvlh_to_inertial(orbit);
    const auto now = vehicle::total_boil_off(
        *ship, vehicle::boil_off_rates(*ship,
                                       environment::earth_orbit_radiation(orbit.position_m, sun - orbit.position_m,
                                                                          nullptr),
                                       attitude));
    CHECK(average.oxidizer_kgps == Catch::Approx(now.oxidizer_kgps).epsilon(1e-6));
    CHECK(average.fuel_kgps == Catch::Approx(now.fuel_kgps).epsilon(1e-6));
}

TEST_CASE("orbit average: eclipse lowers the rate", "[boil_off][vehicle]") {
    const auto ship = top_stage_alone("starship_tanker");
    const auto orbit = circular_orbit(300.0e3, 0.0);
    const auto in_plane = vehicle::orbit_average_boil_off(*ship, orbit, {}, {constants::astronomical_unit_m, 0.0, 0.0});
    const auto polar_sun = vehicle::orbit_average_boil_off(*ship, orbit, {}, {0.0, 0.0, constants::astronomical_unit_m});
    // In the plane the Sun is eclipsed part of the orbit and passes along the axis (end-on); from the pole it stays
    // broadside and never sets.
    CHECK(in_plane.oxidizer_kgps > 0.0);
    CHECK(in_plane.oxidizer_kgps + in_plane.fuel_kgps < polar_sun.oxidizer_kgps + polar_sun.fuel_kgps);
}

TEST_CASE("orbit average in the depot orbit: plausibility of depot and bare tanker", "[boil_off][vehicle]") {
    // 200 km, 26.5 deg (the lunar missions' start orbit), Sun in the orbit plane (the longest eclipse) and at 60 deg
    // beta; depot and tanker nose along +V, dorsal to nadir (identity in LVLH, r3's depot attitude).
    const auto orbit = circular_orbit(200.0e3, 26.5 * constants::deg_to_rad);
    const auto depot = top_stage_alone("starship_depot");
    const auto tanker = top_stage_alone("starship_tanker");
    for (const Vec3 sun_dir : {Vec3{1.0, 0.0, 0.0}, Vec3{0.5, -0.866 * std::sin(26.5 * constants::deg_to_rad),
                                                         0.866 * std::cos(26.5 * constants::deg_to_rad)}}) {
        const Vec3 sun = sun_dir * constants::astronomical_unit_m;
        const auto d = vehicle::orbit_average_boil_off(*depot, orbit, {}, sun);
        const auto t = vehicle::orbit_average_boil_off(*tanker, orbit, {}, sun);
        const double depot_kg_day = (d.oxidizer_kgps + d.fuel_kgps) * constants::seconds_per_day;
        const double tanker_kg_day = (t.oxidizer_kgps + t.fuel_kgps) * constants::seconds_per_day;
        const double depot_percent_day = 100.0 * depot_kg_day / full_propellant_kg(*depot);
        if (verbose()) {
            std::cout << "depot " << depot_kg_day << " kg/day (" << depot_percent_day << " %/day), tanker "
                      << tanker_kg_day << " kg/day\n";
        }
        // Plausibility only [documented]: conventionally insulated cryogenic tanks in orbit lose ~0.1 %/day; a large
        // depot with MLI less. A bare tanker loses orders of magnitude more.
        CHECK(depot_percent_day > 0.005);
        CHECK(depot_percent_day < 0.2);
        CHECK(tanker_kg_day > 10.0 * depot_kg_day);
    }
}

namespace {

class CentralGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3& r, time::Epoch) const override {
        const double d = norm(r);
        return r * (-constants::earth_gm_m3ps2 / (d * d * d));
    }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

// The Sun fixed in GCRF; the Moon opposite it (never in front of the Sun for a vehicle near the Earth).
class FixedSun final : public ephemeris::Ephemeris {
public:
    explicit FixedSun(const Vec3& sun_gcrf_m) : sun_(sun_gcrf_m) {}
    math::StateVector state_gcrf(ephemeris::Body target, time::Epoch) const override {
        if (target == ephemeris::Body::Sun) return {sun_, {}};
        if (target == ephemeris::Body::Moon) return {sun_ * (-3.844e8 / norm(sun_)), {}};
        return {};
    }
    double gm_m3ps2(ephemeris::Body) const override { return 0.0; }
    math::Vec3 apparent_position_gcrf(ephemeris::Body target, time::Epoch epoch,
                                      const math::StateVector&) const override {
        return state_gcrf(target, epoch).position_m;
    }

private:
    Vec3 sun_;
};

const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

struct OrbitWorld {
    explicit OrbitWorld(const Vec3& sun) : ephemeris(sun) {}
    CentralGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    FixedSun ephemeris;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, &ephemeris, false};
};

// A vehicle's top stage alone in a circular orbit, held in LVLH (identity: nose along +V, dorsal to nadir).
std::unique_ptr<vehicle::FlightBody> in_orbit(const std::string& vehicle_id, const math::StateVector& orbit) {
    auto body = std::make_unique<vehicle::FlightBody>(
        top_stage_alone(vehicle_id),
        vehicle::RigidBodyState{orbit.position_m, orbit.velocity_mps, frames::lvlh_to_inertial(orbit), {}});
    body->set_ideal_lvlh_attitude(math::Quat{});
    return body;
}

double main_propellant(const vehicle::FlightBody& body) { return body.vehicle().main_propellant_kg(); }

double all_tanks_kg(const vehicle::Vehicle& v, size_t stage_index) {
    double sum = 0.0;
    for (const auto& t : v.stages()[stage_index].tanks) sum += t.mass_kg;
    return sum;
}

}  // namespace

TEST_CASE("boil-off in flight: one orbit of coasting vents the orbit average", "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    const Vec3 sun{constants::astronomical_unit_m, 0.0, 0.0};  // in the plane: an eclipse every orbit
    OrbitWorld world(sun);
    auto ship = in_orbit("starship_tanker", orbit);
    const auto average = vehicle::orbit_average_boil_off(ship->vehicle(), orbit, {}, sun);
    const double period_s = orbit::orbital_period_s(norm(orbit.position_m), constants::earth_gm_m3ps2);
    const double before = main_propellant(*ship);
    constexpr int segments = 60;
    for (int k = 0; k < segments; ++k) ship->coast(world.env, t0 + period_s * k / segments, period_s / segments);
    const double vented = before - main_propellant(*ship);
    CHECK(vented == Catch::Approx((average.oxidizer_kgps + average.fuel_kgps) * period_s).epsilon(0.03));
    CHECK(ship->boil_off_rate().oxidizer_kgps > 0.0);
}

TEST_CASE("boil-off in flight: the flag, vehicles without stage.thermal and no ephemeris change nothing",
          "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, constants::astronomical_unit_m, 0.0});
    world.env.boil_off = false;
    auto ship = in_orbit("starship_tanker", orbit);
    const double ship_before = main_propellant(*ship);
    ship->coast(world.env, t0, 600.0);
    CHECK(main_propellant(*ship) == ship_before);

    world.env.boil_off = true;
    auto dragon = in_orbit("crew_dragon", orbit);
    const double dragon_before = all_tanks_kg(dragon->vehicle(), 0);
    dragon->coast(world.env, t0, 600.0);
    CHECK(all_tanks_kg(dragon->vehicle(), 0) == dragon_before);

    vehicle::FlightEnvironment no_ephemeris{world.gravity, world.atmosphere, world.earth, nullptr, false};
    ship->coast(no_ephemeris, t0, 600.0);
    CHECK(main_propellant(*ship) == ship_before);
}

TEST_CASE("boil-off in flight: ticks and one long coast vent the same mass", "[boil_off][flight]") {
    // Sun along the orbit normal: steady sunlight, constant rate - the cadence must not matter.
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, 0.0, constants::astronomical_unit_m});
    auto long_coast = in_orbit("starship_tanker", orbit);
    auto short_coasts = in_orbit("starship_tanker", orbit);
    auto ticks = in_orbit("starship_tanker", orbit);
    const double before = main_propellant(*long_coast);
    long_coast->coast(world.env, t0, 600.0);
    for (int k = 0; k < 60; ++k) short_coasts->coast(world.env, t0 + 10.0 * k, 10.0);
    for (int k = 0; k < 600; ++k) ticks->step(world.env, t0 + 0.1 * k, 0.1, {});  // 60 s of ticks
    const double rate = long_coast->boil_off_rate().oxidizer_kgps + long_coast->boil_off_rate().fuel_kgps;
    CHECK(before - main_propellant(*long_coast) == Catch::Approx(rate * 600.0).epsilon(1e-3));
    CHECK(before - main_propellant(*short_coasts) == Catch::Approx(before - main_propellant(*long_coast)).epsilon(5e-3));
    CHECK(before - main_propellant(*ticks) == Catch::Approx(rate * 60.0).epsilon(5e-3));
}

TEST_CASE("boil-off empties a tank to zero, never below", "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, 0.0, constants::astronomical_unit_m});
    auto ship = in_orbit("starship_tanker", orbit);
    ship->vehicle().set_tank_mass(0, "lox_main", 1.0);
    ship->coast(world.env, t0, 600.0);
    CHECK(tank_named(ship->vehicle(), "lox_main").mass_kg == 0.0);
    CHECK(std::isfinite(ship->vehicle().mass_properties().mass_kg));
}

TEST_CASE("boil-off: separation clears the stored rates, the booster does not boil", "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld world({0.0, 0.0, constants::astronomical_unit_m});
    auto cfg = config::load_vehicle(config_dir, "starship_tanker");
    auto stack = std::make_unique<vehicle::FlightBody>(
        std::make_unique<vehicle::Vehicle>(cfg, config_dir),
        vehicle::RigidBodyState{orbit.position_m, orbit.velocity_mps, frames::lvlh_to_inertial(orbit), {}});
    stack->coast(world.env, t0, 20.0);  // rates stored for stage 1 (the ship)
    auto ship = stack->separate(1);
    const double booster_before = all_tanks_kg(stack->vehicle(), 0);
    const double ship_before = main_propellant(*ship);
    stack->coast(world.env, t0 + 20.0, 20.0);
    ship->coast(world.env, t0 + 20.0, 20.0);
    CHECK(all_tanks_kg(stack->vehicle(), 0) == booster_before);
    CHECK(main_propellant(*ship) < ship_before);
}

TEST_CASE("boil-off: a hold forgets the last rates; the first coast after it vents at the new rate",
          "[boil_off][flight]") {
    // Before the hold the Sun stands along the orbit normal (broadside), after it ahead of the nose (end-on): an
    // average with the rate from before the hold would vent clearly more than a ship that never boiled.
    const auto orbit = circular_orbit(400.0e3, 0.0);
    OrbitWorld broadside({0.0, 0.0, constants::astronomical_unit_m});
    OrbitWorld end_on({0.0, constants::astronomical_unit_m, 0.0});
    auto held = in_orbit("starship_tanker", orbit);
    auto fresh = in_orbit("starship_tanker", orbit);
    held->coast(broadside.env, t0, 600.0);
    broadside.env.boil_off = false;
    fresh->coast(broadside.env, t0, 600.0);  // the same path, nothing vented and no rates stored
    for (const char* id : {"lox_main", "ch4_main"}) {
        fresh->vehicle().set_tank_mass(0, id, tank_named(held->vehicle(), id).mass_kg);
    }
    held->hold_docked();
    held->release_docked();
    const double held_before = main_propellant(*held);
    const double fresh_before = main_propellant(*fresh);
    held->coast(end_on.env, t0 + 600.0, 600.0);
    fresh->coast(end_on.env, t0 + 600.0, 600.0);
    CHECK(held_before - main_propellant(*held) ==
          Catch::Approx(fresh_before - main_propellant(*fresh)).epsilon(1e-9));
}

TEST_CASE("boil-off at time warp: 600 s coasts through the eclipse vent the orbit average", "[boil_off][flight]") {
    const auto orbit = circular_orbit(400.0e3, 0.0);
    const Vec3 sun{constants::astronomical_unit_m, 0.0, 0.0};  // in the plane: an eclipse every orbit
    OrbitWorld world(sun);
    auto ship = in_orbit("starship_tanker", orbit);
    const auto average = vehicle::orbit_average_boil_off(ship->vehicle(), orbit, {}, sun);
    const double period_s = orbit::orbital_period_s(norm(orbit.position_m), constants::earth_gm_m3ps2);
    const double before = main_propellant(*ship);
    constexpr double coast_s = 600.0;  // FlightSession::max_coast_s
    const double span_s = 5.0 * period_s;
    for (double t = 0.0; t < span_s - 1e-9; t += coast_s) ship->coast(world.env, t0 + t, std::min(coast_s, span_s - t));
    const double vented = before - main_propellant(*ship);
    CHECK(vented == Catch::Approx((average.oxidizer_kgps + average.fuel_kgps) * span_s).epsilon(0.03));
}

TEST_CASE("boil-off: not inside the air", "[boil_off][flight]") {
    OrbitWorld world({constants::astronomical_unit_m, 0.0, 0.0});
    // 50 km up, coasting: below boil_off_min_altitude_m.
    auto ship = in_orbit("starship_tanker", circular_orbit(50.0e3, 0.0));
    const double before = main_propellant(*ship);
    ship->coast(world.env, t0, 30.0);
    CHECK(main_propellant(*ship) == before);
    CHECK(ship->boil_off_rate().oxidizer_kgps == 0.0);
}

// Refuelling r5: the Moon's albedo and infrared (plan decision 9).
TEST_CASE("Moon radiation: dayside low orbit sees albedo and infrared, the night side a little infrared",
          "[boil_off][environment]") {
    const Vec3 moon_centre{3.8e8, 0.0, 0.0};         // GCRF, from the Earth
    const Vec3 sun_from_earth{-1.496e11, 0.0, 0.0};  // the Moon's Earth-facing side is lit
    const double h = constants::moon_mean_radius_m + 100.0e3;
    // over the Moon's subsolar point (on its Sun side, -x from its centre)
    const Vec3 day = moon_centre + Vec3{-h, 0.0, 0.0};
    const Vec3 moon_from_day = moon_centre - day;
    const auto env = environment::earth_orbit_radiation(day, sun_from_earth - day, &moon_from_day);
    CHECK(env.moon_distance_over_radius == Catch::Approx(h / constants::moon_mean_radius_m));
    CHECK(env.moon_albedo_phase == Catch::Approx(1.0).margin(1e-6));
    const double s = env.solar_flux_w_m2;
    CHECK(env.moon_infrared_w_m2 == Catch::Approx((1.0 - constants::moon_bond_albedo) * s).epsilon(1e-6));
    // a plate facing the Moon (the Sun and the Earth behind it): the Moon's albedo + infrared with the view factor
    const double view = environment::plate_view_factor_to_sphere(0.0, env.moon_distance_over_radius);
    const double moon_part = 0.5 * constants::moon_bond_albedo * s * view + 0.8 * env.moon_infrared_w_m2 * view;
    const Vec3 to_moon = math::normalized(moon_from_day);
    CHECK(environment::absorbed_flux_w_m2(to_moon, env, 0.5, 0.8) == Catch::Approx(moon_part).epsilon(1e-6));
    // night side: no albedo, infrared from the night temperature
    const Vec3 night = moon_centre + Vec3{h, 0.0, 0.0};
    const Vec3 moon_from_night = moon_centre - night;
    const auto dark = environment::earth_orbit_radiation(night, sun_from_earth - night, &moon_from_night);
    CHECK(dark.moon_albedo_phase == 0.0);
    CHECK(dark.moon_infrared_w_m2 ==
          Catch::Approx(constants::stefan_boltzmann_w_m2k4 * std::pow(constants::moon_night_temperature_k, 4)));
}

TEST_CASE("Moon radiation is negligible in low Earth orbit", "[boil_off][environment]") {
    const Vec3 r{constants::earth_mean_radius_m + 200.0e3, 0.0, 0.0};
    const Vec3 sun{0.0, 1.496e11, 0.0}, moon{0.0, -3.84e8, 0.0};
    const Vec3 moon_from_vehicle = moon - r;
    const auto env = environment::earth_orbit_radiation(r, sun - r, &moon_from_vehicle);
    CHECK(environment::plate_view_factor_to_sphere(0.0, env.moon_distance_over_radius) < 3e-5);
    const auto without = environment::earth_orbit_radiation(r, sun - r, nullptr);
    CHECK(without.moon_distance_over_radius == 0.0);  // no Moon given: no lunar terms
    const Vec3 n = math::normalized(moon_from_vehicle);
    CHECK(environment::absorbed_flux_w_m2(n, env, 0.3, 0.8) - environment::absorbed_flux_w_m2(n, without, 0.3, 0.8) <
          0.05);
}
