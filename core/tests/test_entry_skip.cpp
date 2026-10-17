// Entry predictor at lunar-return speeds (lunar l5): radiative heating, skip arcs carried above the atmosphere and the
// exit orbit of a skip-out.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "ship_entry_scenario.hpp"
#include "starsim/aero/radiative_heating.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_corridor.hpp"
#include "starsim/gnc/entry_predictor.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

gnc::EntryPredictorModel predictor_model(test::ShipEntryScenario& scenario, const gnc::AlphaSchedule& schedule) {
    gnc::EntryPredictorModel m;
    m.aero = gnc::EntryAeroTable(scenario.body().vehicle(), schedule);
    m.mass_kg = scenario.body().vehicle().mass_properties().mass_kg;
    m.nose_radius_m = scenario.body().vehicle().stages().front().config.heat_shield.nose_radius_m;
    m.j2 = 0.0;  // the scenario's Earth has central gravity only
    return m;
}

gnc::EntryPredictorState predictor_start(const test::ShipEntryScenario& scenario) {
    return {scenario.start_position_itrf_m(), scenario.start_velocity_itrf_mps(), scenario.initial_bank_rad()};
}

}  // namespace

TEST_CASE("Predictor reports the exit orbit of a skip-out", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    // 10.6 km/s relative eastward (~11.06 km/s inertial: just bound) at -5 deg skips out at ~74 km (probe 2026-10-16:
    // apoapsis 126-157 Mm for every bank); 10.9 km/s relative would already be hyperbolic.
    start.speed_relative_mps = 10600.0;
    start.flight_path_angle_rad = -5.0 * deg;
    test::ShipEntryScenario scenario(start);
    auto model = predictor_model(scenario, schedule);
    const auto p = gnc::predict_entry(model, predictor_start(scenario), 0.0, gnc::EntryTarget{}, 800.0);
    CHECK_FALSE(p.reached_gate);
    CHECK(p.exited);
    CHECK(p.exit_apoapsis_height_m > 1000e3);  // still a lunar-return energy minus one dip
    CHECK(p.exit_periapsis_height_m < 120e3);
}

TEST_CASE("Predictor carries a low skip arc to the second entry", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 9600.0;
    start.flight_path_angle_rad = -6.0 * deg;  // probe 2026-10-16: bank 60 deg exits to a ~710 km apoapsis (-4 deg: no dip)
    test::ShipEntryScenario scenario(start);
    auto model = predictor_model(scenario, schedule);
    // find a lift-up bank that exits low (apoapsis below the carry limit) by scanning; at least one must exist
    model.carry_skips = true;
    bool carried = false;
    for (double bank_deg = 0.0; bank_deg <= 90.0 && !carried; bank_deg += 5.0) {
        const auto p = gnc::predict_entry(model, predictor_start(scenario), bank_deg * deg, gnc::EntryTarget{}, 800.0,
                                          0.0, 1.0, 6000.0);
        if (p.skips > 0) {
            carried = true;
            CHECK(p.reached_gate);  // the carried arc came back down to the gate
        }
    }
    CHECK(carried);
}

TEST_CASE("Predictor range unwraps past the antipode", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 9150.0;
    // probe 2026-10-16: lift up at -6.5 deg carries one skip and reaches the gate ~28,200 km downrange after 4,818 s
    start.flight_path_angle_rad = -6.5 * deg;
    test::ShipEntryScenario scenario(start);
    auto model = predictor_model(scenario, schedule);
    model.carry_skips = true;
    // target 1000 km ahead on the start's heading (east along the start latitude's great circle)
    const math::Vec3 r0 = scenario.start_position_itrf_m();
    const math::Vec3 east = math::normalized(scenario.start_velocity_itrf_mps() -
                                             math::normalized(r0) * dot(scenario.start_velocity_itrf_mps(), math::normalized(r0)));
    const double angle = 1.0e6 / constants::wgs84_semi_major_axis_m;
    const gnc::EntryTarget target{(math::normalized(r0) * std::cos(angle) + east * std::sin(angle)) *
                                      constants::wgs84_semi_major_axis_m,
                                  0.0};
    const auto p = gnc::predict_entry(model, predictor_start(scenario), 0.0, target, 800.0, 0.0, 1.0, 6000.0);
    REQUIRE(p.reached_gate);
    REQUIRE(p.time_s > 3000.0);  // flown well beyond half the Earth's circumference (~20,000 km)
    // a great-circle range alone would read the end past the antipode as short of half a turn
    CHECK(p.downrange_error_m > constants::pi * constants::wgs84_semi_major_axis_m - 1.0e6);
}

TEST_CASE("Predictor heat rate includes the radiative part at lunar speeds", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    start.speed_relative_mps = 10900.0;
    start.flight_path_angle_rad = -6.0 * deg;
    test::ShipEntryScenario scenario(start);
    auto model = predictor_model(scenario, schedule);
    const auto convective = gnc::predict_entry(model, predictor_start(scenario), 0.0, gnc::EntryTarget{}, 800.0);
    model.radiative = std::make_shared<const aero::RadiativeHeatingTable>(
        aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    const auto total = gnc::predict_entry(model, predictor_start(scenario), 0.0, gnc::EntryTarget{}, 800.0);
    CHECK(total.peak_heat_rate_w_m2 > 1.3 * convective.peak_heat_rate_w_m2);
}


TEST_CASE("Guided lunar-return entry at 9.6 km/s inertial reaches the gate within the heat limit", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    // 9.15 km/s relative eastward = 9.61 km/s inertial (the planned interface speed); -5 deg lies inside the corridor
    // (probe 2026-10-16: lift up skips out to 5,300 km, 75 deg bank carries one low skip at 0.85 MW/m2 and 4 g, 90 deg
    // ends at 9 g; Task 5 measures the corridor - adjust to its middle then)
    start.speed_relative_mps = 9150.0;
    start.flight_path_angle_rad = -5.0 * deg;
    test::ShipEntryScenario scenario(start);
    const auto radiative = std::make_shared<const aero::RadiativeHeatingTable>(
        aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    // target: the end point of a 75 deg bank prediction (reachable by construction)
    auto model = predictor_model(scenario, schedule);
    model.radiative = radiative;
    model.carry_skips = true;
    const auto nominal = gnc::predict_entry(model, predictor_start(scenario), 75.0 * deg, gnc::EntryTarget{}, 800.0,
                                            0.0, 1.0, 6000.0);
    REQUIRE(nominal.reached_gate);
    const math::Vec3 end = nominal.end_position_itrf_m;
    const gnc::EntryTarget target{end * ((norm(end) - 800.0) / norm(end)), 0.0};
    auto p = gnc::EntryGuidanceParameters::lunar_return();
    p.predictor_j2 = false;
    p.final_bank_rad = -1.0;  // the constant-bank skip entry (lunar_return() profiles the bank since Task 8)
    p.initial_bank_rad = 75.0 * deg;
    p.lateral_margin_m = 0.0;  // the target is the prediction's own end point, not a track aimed beside the site
    scenario.fly_guided(schedule, target, p, radiative);
    CHECK(scenario.reached_gate());
    CHECK(scenario.gate_miss_m(target) < 2000.0);
    CHECK(scenario.body().peak_heat_rate_w_m2() < 1.2e6);
    CHECK_FALSE(scenario.body().impacted());
}

TEST_CASE("Aerocapture guidance exits near the target apoapsis", "[ship][guidance][skip]") {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::EntryStart start;
    // as the lunar-return case: at -5 deg a bank between 60 deg (apoapsis 2,471 km) and 75 deg (a low skip) exits to
    // 400 km at ~0.85 MW/m2 (probe 2026-10-16; -4.5 deg at 9.6 km/s relative exited above 15,000 km at any bank);
    // Task 5's aerocapture corridor middle replaces this
    start.speed_relative_mps = 9150.0;
    start.flight_path_angle_rad = -5.0 * deg;
    test::ShipEntryScenario scenario(start);
    const auto radiative = std::make_shared<const aero::RadiativeHeatingTable>(
        aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    auto p = gnc::EntryGuidanceParameters::aerocapture(400e3, -1.0);
    p.predictor_j2 = false;
    const auto exit = scenario.fly_aerocapture(schedule, p, radiative);
    REQUIRE(exit.exited);
    CHECK(std::abs(exit.apoapsis_height_m - 400e3) < 50e3);  // [E] band; the campaign (Task 10) widens it
    CHECK(exit.periapsis_height_m < 120e3);                  // still dips: the perigee raise follows
    CHECK(scenario.body().peak_heat_rate_w_m2() < 1.2e6);
}



TEST_CASE("Profiled lunar-return entry at the return speed reaches the gate within the limits", "[ship][guidance][skip]") {
    // l5 Task 8 finding: at 9,667 m/s relative (the lunar return's interface speed, eastward) no
    // constant bank both stays in and holds the limits; the bank profile linear in energy (lunar_return(): 40 deg at
    // the gate) has a window of initial banks. Fly the guidance at the angle with the widest window to the end point of
    // its middle's prediction.
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    const auto radiative = std::make_shared<const aero::RadiativeHeatingTable>(
        aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    auto p = gnc::EntryGuidanceParameters::lunar_return();
    p.predictor_j2 = false;
    test::EntryStart start;
    start.speed_relative_mps = 9667.0;
    start.latitude_rad = 0.0;  // geodetic = geocentric angle (the scenario measures it from the geodetic vertical)
    test::ShipEntryScenario probe(start);
    // the guidance's own predictor model (bank rates, profile, skips), as the lunar return budget takes it
    const gnc::EntryGuidance onboard(probe.body(), schedule, gnc::EntryTarget{}, p, radiative);
    const auto model = onboard.model();
    gnc::CorridorQuery q;
    q.speed_mps = start.speed_relative_mps;
    q.latitude_rad = start.latitude_rad;
    q.limits = probe.body().vehicle().stages().front().config.heat_shield;
    const auto corridor = gnc::entry_corridor(model, q);
    REQUIRE(corridor.exists);
    // the constant-bank corridor's middle has no admissible constant bank (the finding)
    const auto window = gnc::widest_bank_window(model, q, corridor, p.max_bank_rad);
    REQUIRE(window.exists);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("corridor %.2f .. %.2f deg; widest window at %.2f deg: %.1f .. %.1f deg\n", corridor.steep_rad / deg,
                    corridor.shallow_rad / deg, window.flight_path_rad / deg, window.low_rad / deg,
                    window.high_rad / deg);
    }
    CHECK(window.width_rad > 3.0 * deg);
    start.flight_path_angle_rad = window.flight_path_rad;
    start.bank_rad = window.middle_rad;
    p.initial_bank_rad = window.middle_rad;  // as the lunar return budget sets it
    test::ShipEntryScenario scenario(start);
    const auto nominal = gnc::predict_entry(model, predictor_start(scenario), window.middle_rad, gnc::EntryTarget{},
                                            800.0, 0.0, 1.0, 6000.0);
    REQUIRE(nominal.reached_gate);
    const math::Vec3 end = nominal.end_position_itrf_m;
    const gnc::EntryTarget target{end * ((norm(end) - 800.0) / norm(end)), 0.0};
    scenario.fly_guided(schedule, target, p, radiative);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("profiled entry: gate %d miss %.0f m, peak heat %.0f kW/m2\n", scenario.reached_gate(),
                    scenario.gate_miss_m(target), scenario.body().peak_heat_rate_w_m2() / 1000.0);
    }
    CHECK(scenario.reached_gate());
    CHECK(scenario.gate_miss_m(target) < 2000.0);
    CHECK(scenario.body().peak_heat_rate_w_m2() < 1.2e6);
    CHECK_FALSE(scenario.body().impacted());
}

TEST_CASE("A reversal between lift-down banks rolls through lift down", "[ship][guidance][skip]") {
    // l5 Task 8: +115 -> -115 deg rolled through 0 (lift up) during the supercircular pull-out and skipped out
    double bank = 115.0 * deg;
    double lowest_magnitude = bank;
    for (int i = 0; i < 200; ++i) {
        bank = gnc::bank_step(bank, -115.0 * deg, 1.0 * deg, true);
        lowest_magnitude = std::min(lowest_magnitude, std::abs(bank));
    }
    CHECK(std::abs(bank + 115.0 * deg) < 1e-9);
    CHECK(lowest_magnitude > 114.0 * deg);  // never towards lift up
    // small banks still reverse through 0 (the shorter way)
    double small = 50.0 * deg;
    for (int i = 0; i < 99; ++i) small = gnc::bank_step(small, -50.0 * deg, 1.0 * deg, true);
    CHECK(std::abs(small + 49.0 * deg) < 1e-9);
    // LEO / capsule entries: straight towards the command
    double straight = 115.0 * deg;
    for (int i = 0; i < 115; ++i) straight = gnc::bank_step(straight, -115.0 * deg, 1.0 * deg, false);
    CHECK(std::abs(straight) < 1e-9);
}

TEST_CASE("A reversal by about 180 deg keeps its roll direction while the command moves", "[ship][guidance][skip]") {
    // l5 Task 10: -62 -> +112 .. +129 deg - the shorter way flipped with every new command and the bank stood still
    double bank = -62.0 * deg;
    int direction = 0;
    double previous = bank;
    for (int i = 0; i < 40; ++i) {
        const double command = (i % 2 == 0 ? 112.0 : 125.0) * deg;  // either side of the opposite of the bank
        bank = gnc::bank_step_held(bank, command, 3.0 * deg, direction);
        CHECK(std::remainder(bank - previous, 2.0 * constants::pi) > 0.0);  // from lift up: on the shorter way, through 0
        previous = bank;
    }
    for (int i = 0; i < 100; ++i) bank = gnc::bank_step_held(bank, 118.0 * deg, 3.0 * deg, direction);
    CHECK(std::abs(bank - 118.0 * deg) < 1e-9);
    CHECK(direction == 0);
    // from a lift-down bank an ambiguous reversal starts through lift down (+100 -> -75 deg: up through 180, not via 0)
    bank = 100.0 * deg;
    direction = 0;
    bank = gnc::bank_step_held(bank, -75.0 * deg, 3.0 * deg, direction);
    CHECK(std::abs(bank - 103.0 * deg) < 1e-9);
    CHECK(direction == 1);
    // a command well inside the shorter way is still followed back (a reversal of the reversal)
    bank = 10.0 * deg;
    direction = 1;
    bank = gnc::bank_step_held(bank, -60.0 * deg, 3.0 * deg, direction);
    CHECK(std::abs(bank - 7.0 * deg) < 1e-9);
    CHECK(direction == -1);
}
