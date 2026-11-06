// Entry corridor in flight-path angle and the highest admissible interface speed (lunar l5).

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "ship_entry_scenario.hpp"
#include "starsim/aero/radiative_heating.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_corridor.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

// The ship scenario's predictor model with the radiative table (central gravity) and the ship stage's heat shield.
std::pair<gnc::EntryPredictorModel, config::HeatShieldConfig> ship_model() {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::ShipEntryScenario scenario(test::EntryStart{});
    const auto& vehicle = scenario.body().vehicle();
    const auto& shield = vehicle.stages().front().config.heat_shield;
    gnc::EntryPredictorModel m;
    m.aero = gnc::EntryAeroTable(vehicle, schedule);
    m.mass_kg = vehicle.mass_properties().mass_kg;
    m.nose_radius_m = shield.nose_radius_m;
    m.j2 = 0.0;
    m.radiative = std::make_shared<const aero::RadiativeHeatingTable>(aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    return {m, shield};
}

// The Crew Dragon capsule alone (trunk jettisoned) at its trim, no heating predicted.
std::pair<gnc::EntryPredictorModel, config::HeatShieldConfig> capsule_model() {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    const vehicle::Vehicle capsule(c, STARSIM_CONFIG_DIR);
    gnc::EntryPredictorModel m;
    m.aero = gnc::EntryAeroTable::capsule(capsule);
    m.mass_kg = capsule.mass_properties().mass_kg;
    m.j2 = 0.0;
    return {m, capsule.stages().front().config.heat_shield};
}

void print_corridor(const char* what, double speed_mps, const gnc::EntryCorridor& c) {
    if (!std::getenv("STARSIM_VERBOSE")) return;
    std::printf("%s %.0f m/s: exists %d, steep %.2f .. shallow %.2f deg (width %.2f, middle %.2f), limiting %s\n", what,
                speed_mps, c.exists ? 1 : 0, c.steep_rad / deg, c.shallow_rad / deg, c.width_rad / deg,
                c.middle_rad / deg, c.limiting.c_str());
}

}  // namespace

// Planner plan Task 3b (dry 255 -> 177 t): at 255 t no corridor existed at 10.9 km/s; the lighter ship has one of
// 0.13 deg (-5.25 .. -5.12 deg), far below the 1 deg the planners ask for: still no direct lunar return at that speed.
TEST_CASE("The ship has no usable lunar-return corridor at 10.9 km/s within its heat limit", "[ship][guidance][corridor]") {
    auto [model, limits] = ship_model();
    gnc::CorridorQuery q;
    q.speed_mps = 10900.0;
    q.limits = limits;
    const auto c = gnc::entry_corridor(model, q);
    print_corridor("ship direct", q.speed_mps, c);
    CHECK((!c.exists || c.width_rad < 1.0 * deg));
    CHECK(c.limiting == "heat_rate");
}

TEST_CASE("The ship's corridor at the highest admissible speed is at least 1 deg wide", "[ship][guidance][corridor]") {
    auto [model, limits] = ship_model();
    gnc::CorridorQuery q;
    q.limits = limits;
    const double v = gnc::max_entry_speed(model, q, 1.0 * deg, 8000.0, 11000.0);
    if (std::getenv("STARSIM_VERBOSE")) std::printf("max entry speed %.0f m/s\n", v);
    CHECK(v > 9000.0);  // the planning estimate was ~9.6-9.7 km/s
    CHECK(v < 10500.0);
    q.speed_mps = v;
    const auto c = gnc::entry_corridor(model, q);
    print_corridor("ship direct", v, c);
    REQUIRE(c.exists);
    CHECK(c.width_rad >= 1.0 * deg - 1e-6);
    CHECK(c.middle_rad < -2.0 * deg);
}

TEST_CASE("A capsule's 11 km/s corridor is Apollo-like", "[capsule][guidance][corridor][validation]") {
    // Plausibility (spec): Apollo (L/D ~0.3) entered at -6.5 deg at 400,000 ft within a corridor ~2 deg wide; the
    // Crew Dragon model (L/D 0.2) with a 10 g load limit and no heat limit should give a corridor 1-2.5 deg wide with
    // its middle between -7.5 and -5.5 deg.
    auto [model, limits] = capsule_model();
    limits.max_heat_rate_w_m2 = 0.0;  // not judged
    limits.max_load_g = 10.0;
    gnc::CorridorQuery q;
    q.speed_mps = 11000.0;
    q.interface_height_m = 121.9e3;  // 400,000 ft
    q.gate_height_m = 5500.0;
    q.limits = limits;
    q.constraint_fraction = 1.0;
    const auto c = gnc::entry_corridor(model, q);
    print_corridor("capsule direct", q.speed_mps, c);
    REQUIRE(c.exists);
    CHECK(c.width_rad > 1.0 * deg);
    CHECK(c.width_rad < 2.5 * deg);
    CHECK(c.middle_rad > -7.5 * deg);
    CHECK(c.middle_rad < -5.5 * deg);
}

TEST_CASE("The aerocapture corridor to 400 km exists at the admissible speed", "[ship][guidance][corridor]") {
    auto [model, limits] = ship_model();
    gnc::CorridorQuery q;
    q.limits = limits;
    q.aerocapture_apoapsis_m = 400e3;
    q.speed_mps = gnc::max_entry_speed(model, q, 0.5 * deg, 8000.0, 11000.0);
    const auto c = gnc::entry_corridor(model, q);
    print_corridor("ship aerocapture", q.speed_mps, c);
    REQUIRE(c.exists);
}

// Task 9 (2026-10-16): the classical corridor judges the limits at full lift up only; the bank that exits at 400 km
// dives deeper - at 10.1 km/s it sees 1.3-2.5 MW/m2 (probe). The aerocapture speed needs a bank window: from the bank
// that exits at the apoapsis up to the deepest within the limits, at least min_bank_window wide.
TEST_CASE("The aerocapture speed leaves a bank window to 400 km within the limits", "[ship][guidance][corridor]") {
    auto [model, limits] = ship_model();
    gnc::CorridorQuery q;
    q.limits = limits;
    q.aerocapture_apoapsis_m = 400e3;
    q.min_bank_window_rad = 10.0 * deg;
    q.max_bank_rad = 150.0 * deg;
    q.speed_mps = gnc::max_entry_speed(model, q, 0.5 * deg, 8000.0, 11000.0);
    const auto c = gnc::entry_corridor(model, q);
    const auto w = gnc::widest_bank_window(model, q, c, q.max_bank_rad);
    print_corridor("ship aerocapture with window", q.speed_mps, c);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("  window at %.2f deg: %.1f .. %.1f deg\n", w.flight_path_rad / deg, w.low_rad / deg, w.high_rad / deg);
    }
    REQUIRE(w.exists);
    CHECK(w.width_rad >= 10.0 * deg);
    // the window costs speed against the corridor alone (Task 3b, 177 t: 9992 against 10273 m/s; at 255 t the window
    // speed was below 9.9 km/s)
    gnc::CorridorQuery corridor_only = q;
    corridor_only.min_bank_window_rad = 0.0;
    const double v_corridor = gnc::max_entry_speed(model, corridor_only, 0.5 * deg, 8000.0, 11000.0);
    CHECK(q.speed_mps < v_corridor);
    CHECK(q.speed_mps > 9000.0);
    // regression pins of the 177 t ship (measured 2026-11-03): 50 m/s is 0.5 % of the speed, ~1.5 % of the heat rate
    // at the limit (q ~ v^3) - a model change of that size shows here
    CHECK(q.speed_mps == Catch::Approx(9992.0).margin(50.0));
    CHECK(v_corridor == Catch::Approx(10273.0).margin(50.0));
}
