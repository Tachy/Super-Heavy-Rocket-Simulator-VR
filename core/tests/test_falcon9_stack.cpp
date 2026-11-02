// The Falcon 9 + Crew Dragon stack against the measured height (plan 2026-11-02, Task 10; docs/subsystems/falcon9.md "Stack").
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using Catch::Approx;

namespace {
constexpr double first_stage_with_interstage_m = 47.7;  // [S] Wikipedia Falcon 9
constexpr double stack_height_m = 65.6;                 // [D] 47.7 + 9.8 + 8.1, config comment of second_stage.length_m
constexpr double stack_tolerance_m = 0.5;               // the photos' +-0.2 m and the published lengths' rounding
constexpr double tank_dome_station_m = 41.2;            // [S] first stage without its interstage

vehicle::Vehicle launcher() {
    return vehicle::Vehicle(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5"), STARSIM_CONFIG_DIR);
}
}  // namespace

TEST_CASE("Falcon 9 with Dragon stands at the measured height", "[falcon9][vehicle]") {
    const vehicle::Vehicle stack(config::stack_payload(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5"),
                                                       config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon")),
                                 STARSIM_CONFIG_DIR);
    CHECK(stack.length_m() == Approx(stack_height_m).margin(stack_tolerance_m));
}

TEST_CASE("The second stage stands on the interstage top, its engine inside the interstage", "[falcon9][vehicle]") {
    const auto v = launcher();
    const auto& second = v.stages().at(1);
    CHECK(second.base_station_m == Approx(first_stage_with_interstage_m));
    const auto& mvac = second.engines.at(0);
    CHECK(mvac.gimbal_point_m.x < second.base_station_m);
    CHECK(mvac.gimbal_point_m.x > tank_dome_station_m);
    CHECK(mvac.gimbal_point_m.y == 0.0);
}

TEST_CASE("The second stage's tanks fit between its gimbal point and its top", "[falcon9][vehicle]") {
    const auto v = launcher();
    const auto& second = v.stages().at(1);
    const double gimbal_station_m = second.engines.at(0).gimbal_point_m.x - second.base_station_m;
    for (const auto& tank : second.tanks) {
        if (tank.length_m <= 0.0) continue;
        CHECK(tank.config.base_station_m > gimbal_station_m);
        CHECK(tank.config.base_station_m + tank.length_m <= second.config.length_m);
    }
}

TEST_CASE("Payload and fairing stand above the second stage, at their configured stations", "[falcon9][vehicle]") {
    constexpr double payload_kg = 16500.0;
    auto bare_config = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    auto& upper = bare_config.stages.back();
    const double payload_station_m = upper.payload_station_m.value();
    const double fairing_station_m = upper.fairing_station_m.value();
    CHECK(payload_station_m > upper.length_m);  // above the stage's top plane, not inside its RP-1
    CHECK(fairing_station_m > upper.length_m);

    const double fairing_kg = upper.fairing_mass_kg;
    upper.fairing_mass_kg = 0.0;
    const auto bare = vehicle::Vehicle(bare_config, STARSIM_CONFIG_DIR).mass_properties();

    auto loaded_config = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    loaded_config.stages.back().payload_mass_kg = payload_kg;
    const vehicle::Vehicle loaded(loaded_config, STARSIM_CONFIG_DIR);
    const auto with = loaded.mass_properties();
    const double base_m = loaded.stages().back().base_station_m;
    const double expected_cg_m = (bare.mass_kg * bare.cg_station_m + payload_kg * (base_m + payload_station_m) +
                                  fairing_kg * (base_m + fairing_station_m)) /
                                 (bare.mass_kg + payload_kg + fairing_kg);
    CHECK(with.mass_kg == Approx(bare.mass_kg + payload_kg + fairing_kg));
    CHECK(with.cg_station_m == Approx(expected_cg_m).margin(1e-9));
}

TEST_CASE("A stage without payload stations keeps its extra mass in the upper part of the stage", "[falcon9][vehicle]") {
    constexpr double payload_kg = 5000.0;
    constexpr double default_fraction = 0.75;  // [E] vehicle.cpp, every vehicle without the keys (Starship: unchanged)
    auto bare_config = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    auto& upper = bare_config.stages.back();
    upper.payload_station_m.reset();
    upper.fairing_station_m.reset();
    upper.fairing_mass_kg = 0.0;
    const auto bare = vehicle::Vehicle(bare_config, STARSIM_CONFIG_DIR).mass_properties();
    upper.payload_mass_kg = payload_kg;
    const vehicle::Vehicle loaded(bare_config, STARSIM_CONFIG_DIR);
    const double station_m = loaded.stages().back().base_station_m + default_fraction * upper.length_m;
    CHECK(loaded.mass_properties().cg_station_m ==
          Approx((bare.mass_kg * bare.cg_station_m + payload_kg * station_m) / (bare.mass_kg + payload_kg)).margin(1e-9));
}

TEST_CASE("The nozzle leaves the interstage before the second stage lights", "[falcon9][vehicle]") {
    const auto v = launcher();
    const auto& first = v.stages().at(0);
    const auto& second = v.stages().at(1);
    constexpr double drawn_nozzle_m = 3.4;  // [E] detail/falcon9_block5.toml, mvac.bell_length_m
    const double exit_below_top_m = (second.base_station_m - second.engines.at(0).gimbal_point_m.x) + drawn_nozzle_m;
    // 1 m/s of pushers alone over the ignition delay; the first stage's drag only adds to it.
    CHECK(first.config.staging->separation_dv_mps * first.config.staging->upper_ignition_delay_s > exit_below_top_m);
}
