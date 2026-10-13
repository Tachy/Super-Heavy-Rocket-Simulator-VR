// Ship entry guidance (b2): the onboard computer's aero table and point-mass predictor against the 6-DOF ship,
// the bank corrector, and the guided entry and belly flop.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <catch2/catch_test_macros.hpp>

#include "ship_entry_scenario.hpp"
#include "starsim/gnc/entry_aero_table.hpp"

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
