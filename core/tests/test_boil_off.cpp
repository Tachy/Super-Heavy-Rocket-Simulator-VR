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
#include "starsim/environment/thermal_radiation.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/vehicle/boil_off.hpp"
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
