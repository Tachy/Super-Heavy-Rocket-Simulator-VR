// Ship entry guidance (b2): the onboard computer's aero table and point-mass predictor against the 6-DOF ship,
// the bank corrector, and the guided entry and belly flop.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <catch2/catch_test_macros.hpp>

#include "ship_entry_scenario.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_predictor.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

gnc::AlphaSchedule ship_alpha_schedule() {
    return gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
}

test::EntryLaw scheduled(const gnc::AlphaSchedule& schedule, double bank_deg) {
    return [=](const vehicle::FlightTelemetry& t, double& alpha, double& bank) {
        alpha = schedule.at(t.altitude_m < 80000.0 ? t.mach : 30.0);
        bank = bank_deg * deg;
    };
}

}  // namespace

TEST_CASE("Entry aero table reproduces the 6-DOF ship's aerodynamic acceleration", "[ship][guidance]") {
    const auto schedule = ship_alpha_schedule();
    test::ShipEntryScenario scenario(test::EntryStart{});
    const gnc::EntryAeroTable table(scenario.body().vehicle(), schedule);
    REQUIRE_FALSE(table.empty());

    const double l_over_d = table.cl_area_m2(20.0) / table.cd_area_m2(20.0);
    CHECK(l_over_d > 0.3);
    CHECK(l_over_d < 0.9);

    const auto samples = scenario.run(scheduled(schedule, 0.0), 0.6);
    int checked = 0;
    for (double mach : {25.0, 15.0, 5.0, 2.0, 0.8}) {
        // first sample below this Mach in continuum flow, with the attitude settled on the schedule
        for (const auto& s : samples) {
            const auto& t = s.telemetry;
            if (t.altitude_m > 75000.0 || t.mach > mach) continue;
            const double q_over_m = t.dynamic_pressure_pa / t.mass_kg;
            const double drag = q_over_m * table.cd_area_m2(t.mach);
            const double lift = q_over_m * table.cl_area_m2(t.mach);
            if (verbose()) {
                std::cout << "Mach " << t.mach << " h " << t.altitude_m / 1e3 << " km: drag " << s.aero_drag_mps2
                          << " / table " << drag << ", lift " << s.aero_lift_mps2 << " / table " << lift << ", aoa "
                          << t.angle_of_attack_rad / deg << " (schedule " << schedule.at(t.mach) / deg << ")\n";
            }
            CHECK(std::abs(s.aero_drag_mps2 / drag - 1.0) < 0.10);
            // Subsonic lift ~ cot(alpha) near 80 deg is very sensitive to the attitude tracking (2 deg off: 25 %);
            // the belly flop steers by tilt, not by the table's lift.
            if (mach > 1.0) CHECK(std::abs(s.aero_lift_mps2 / lift - 1.0) < 0.10);
            ++checked;
            break;
        }
    }
    CHECK(checked == 5);
}

namespace {

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

TEST_CASE("Entry predictor matches the 6-DOF ship at constant bank", "[ship][guidance]") {
    const auto schedule = ship_alpha_schedule();
    test::EntryStart start;
    start.bank_rad = 45.0 * deg;
    test::ShipEntryScenario scenario(start);
    const auto model = predictor_model(scenario, schedule);
    const auto initial = predictor_start(scenario);

    constexpr double gate_m = 1500.0;
    const auto samples = scenario.run(scheduled(schedule, 45.0), 0.0, 3000.0, gate_m);
    REQUIRE_FALSE(scenario.body().impacted());
    const math::Vec3 truth_end = scenario.position_itrf_m();
    // target = the 6-DOF end point on the ground: the prediction's downrange error and crossrange are its miss
    const gnc::EntryTarget target{truth_end * ((norm(truth_end) - gate_m) / norm(truth_end)), 0.0};
    const auto p = gnc::predict_entry(model, initial, 45.0 * deg, target, gate_m);
    const double range = gnc::ground_range_m(initial.position_itrf_m, truth_end);
    const double miss = gnc::ground_range_m(p.end_position_itrf_m, truth_end);
    if (verbose()) {
        std::cout << "6-DOF: " << scenario.time_s() << " s, range " << range / 1e3 << " km, peak heat "
                  << scenario.body().peak_heat_rate_w_m2() / 1e3 << " kW/m2; predictor: " << p.time_s << " s, miss "
                  << miss / 1e3 << " km (downrange " << p.downrange_error_m / 1e3 << ", cross " << p.crossrange_m / 1e3
                  << "), peak heat " << p.peak_heat_rate_w_m2 / 1e3 << " kW/m2, load " << p.peak_load_g << " g, q "
                  << p.peak_dynamic_pressure_pa << " Pa\n";
    }
    CHECK(p.reached_gate);
    CHECK(miss < 0.02 * range);
    CHECK(std::abs(p.time_s - scenario.time_s()) < 0.03 * scenario.time_s());
    CHECK(std::abs(p.peak_heat_rate_w_m2 / scenario.body().peak_heat_rate_w_m2() - 1.0) < 0.15);
    (void)samples;
}

TEST_CASE("Entry predictor range falls with bank and starts from the current bank", "[ship][guidance]") {
    const auto schedule = ship_alpha_schedule();
    test::ShipEntryScenario scenario(test::EntryStart{});
    const auto model = predictor_model(scenario, schedule);
    auto initial = predictor_start(scenario);
    const gnc::EntryTarget target{scenario.start_position_itrf_m(), 0.0};  // only ranges matter here

    const auto range_at = [&](double command_deg, double start_bank_deg) {
        initial.bank_rad = start_bank_deg * deg;
        const auto p = gnc::predict_entry(model, initial, command_deg * deg, target, 1500.0);
        CHECK(p.reached_gate);
        return gnc::ground_range_m(initial.position_itrf_m, p.end_position_itrf_m);
    };
    const double r0 = range_at(0.0, 0.0), r60 = range_at(60.0, 0.0), r120 = range_at(120.0, 0.0);
    const double r0_from_120 = range_at(0.0, 120.0);
    if (verbose()) {
        std::cout << "range at bank 0 / 60 / 120: " << r0 / 1e3 << " / " << r60 / 1e3 << " / " << r120 / 1e3
                  << " km; bank 0 starting from 120: " << r0_from_120 / 1e3 << " km\n";
    }
    CHECK(r0 > r60);
    CHECK(r60 > r120);
    CHECK(r0_from_120 < r0);
}
