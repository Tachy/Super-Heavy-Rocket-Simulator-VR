// Ship flip and catch (b3): engines 3 -> 2 -> 1, catch pins, SCvx landing plans, flip and catch in flight.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <set>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/descent_planner.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

}  // namespace

TEST_CASE("Ship has three single sea-level engine groups and catch pins", "[ship][landing]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& ship = cfg.stages.at(1);
    std::set<std::string> sea_level;
    for (const auto& g : ship.engine_groups) {
        if (g.engine_id != "raptor3") continue;
        sea_level.insert(g.id);
        CHECK(g.count == 1);
        CHECK(g.gimbal);
        CHECK(g.ring_radius_m == 1.3);
    }
    CHECK(sea_level == std::set<std::string>{"sea_level_1", "sea_level_2", "sea_level_3"});

    const vehicle::Vehicle vehicle(cfg, STARSIM_CONFIG_DIR);
    int sea_level_engines = 0;
    std::set<long> angles;
    for (const auto& e : vehicle.stages().at(1).engines) {
        if (e.group_id.rfind("ship/sea_level_", 0) != 0) continue;
        ++sea_level_engines;
        angles.insert(std::lround(std::atan2(e.gimbal_point_m.z, e.gimbal_point_m.y) / deg));
    }
    CHECK(sea_level_engines == 3);
    CHECK(angles == std::set<long>{0, 120, -120});

    REQUIRE(ship.catch_pins.azimuths_rad.size() == 2);
    CHECK(std::abs(ship.catch_pins.azimuths_rad[0] - 90.0 * deg) < 1e-9);
    CHECK(std::abs(ship.catch_pins.azimuths_rad[1] - 270.0 * deg) < 1e-9);
    CHECK(ship.catch_pins.station_m > 30.0);
    CHECK(ship.catch_pins.station_m < 38.0);  // below the forward flaps (38-46 m)
}

namespace {

// The ship after the flip: landing propellant (30 t) minus the flip (~3 t), model built like the flight will build it.
struct Ship {
    std::unique_ptr<vehicle::Vehicle> vehicle;
    environment::Us76Atmosphere atmosphere;
    gnc::DescentModel model;
};

std::unique_ptr<Ship> ship_after_flip(double propellant_kg = 27000.0) {
    auto s = std::make_unique<Ship>();
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    double full = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) full += tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= propellant_kg / full;
    s->vehicle = std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR);
    gnc::DescentModelInputs in;
    in.vehicle = s->vehicle.get();
    in.phase_groups = gnc::ship_landing_parameters().scvx.snapshot.phase_groups;
    in.atmosphere = &s->atmosphere;
    in.site_height_m = 10.0;
    const double lat = 26.0 * deg;
    in.gravity_mps2 = {-9.79, 0.0, 0.0};
    in.earth_rate_radps = math::Vec3{std::sin(lat), 0.0, std::cos(lat)} * 7.2921150e-5;
    in.earth_center_m = {-6.374e6, 0.0, 0.0};
    s->model = gnc::make_descent_model(in);
    return s;
}

}  // namespace

// After the flip (relight at the 800 m gate [D: flights 13/14 relit between 1 and 0.5 km at ~100 m/s]).
TEST_CASE("SCvx plans a ship catch from after the flip", "[ship][landing]") {
    const auto ship = ship_after_flip();
    gnc::DescentTarget target;
    target.goal = gnc::DescentGoal::Catch;
    const double pin_station = ship->vehicle->stages()[0].config.catch_pins.station_m;
    const double cg = ship->vehicle->mass_properties().cg_station_m;
    target.position_m = {96.0 - (pin_station - cg), 0.0, 0.0};  // pins 1 m above the 95 m arm tops
    target.velocity_mps = {-1.0, 0.0, 0.0};
    auto settings = gnc::ship_landing_parameters().scvx.settings;
    settings.limits.min_mass_kg = ship->vehicle->mass_properties().mass_kg - ship->vehicle->propellant_kg() + 1000.0;
    const auto x0 = gnc::make_state(ship->vehicle->mass_properties().mass_kg, {650.0, 20.0, -10.0}, {-85.0, 3.0, 0.0},
                                    math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 5.0 * deg), {});
    const auto plan = gnc::plan_descent(ship->model, x0, 0, target, settings, nullptr, 0.0);
    const auto& d = plan.reference.duration_s;
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("ship scvx: converged %d, %d iterations, vc %.2e, durations %.1f + %.1f + %.1f s, final mass %.1f t\n",
                    plan.converged, plan.iterations, plan.virtual_control, d[0], d[1], d[2],
                    gnc::state_position(plan.reference.x.back()).x * 0.0 + plan.reference.x.back()[0] / 1000.0);
        const auto& sets = ship->model.engine_sets;
        for (int p = 0; p < gnc::descent_phases; ++p) {
            std::printf("  set %d: thrust %.2f .. %.2f MN\n", p, sets[static_cast<size_t>(p)].thrust_min_n / 1e6,
                        sets[static_cast<size_t>(p)].thrust_max_n / 1e6);
        }
    }
    REQUIRE(plan.converged);
    CHECK(d[0] > 0.0);
    CHECK(d[1] > 0.0);
    CHECK(d[2] > 0.0);
    CHECK(norm(gnc::state_position(plan.reference.x.back()) - target.position_m) < 0.5);
}

