// Ship entry corridor: too shallow skips out, too steep overheats / overloads, nominal stays inside.

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include <catch2/catch_test_macros.hpp>

#include "ship_entry_scenario.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

test::EntryLaw constant(double alpha_deg, double bank_deg) {
    return [=](const vehicle::FlightTelemetry&, double& alpha, double& bank) {
        alpha = alpha_deg * deg;
        bank = bank_deg * deg;
    };
}

void report(const char* name, const std::vector<test::EntrySample>& samples, vehicle::FlightBody& body) {
    if (!std::getenv("STARSIM_VERBOSE")) return;
    double min_h = 1e9;
    for (const auto& s : samples) min_h = std::min(min_h, s.telemetry.altitude_m);
    std::cout << name << ": " << samples.size() << " s, min altitude " << min_h / 1e3 << " km, final "
              << samples.back().telemetry.altitude_m / 1e3 << " km / " << samples.back().telemetry.speed_relative_mps
              << " m/s, peak heat " << body.peak_heat_rate_w_m2() / 1e3 << " kW/m2, load "
              << body.heat_load_j_m2() / 1e6 << " MJ/m2, breakup '" << body.breakup_reason() << "', skip "
              << body.skipped_out() << "\n";
    for (size_t i = 0; i < samples.size(); i += 50) {
        const auto& t = samples[i].telemetry;
        std::cout << "  t " << samples[i].t_s << " h " << t.altitude_m / 1e3 << " v " << t.speed_relative_mps << " gamma "
                  << t.flight_path_angle_rad / deg << " aoa " << t.angle_of_attack_rad / deg << " drag "
                  << samples[i].drag_mps2 << " q " << t.dynamic_pressure_pa << "\n";
    }
}

}  // namespace

// Above circular speed (e.g. from an elliptical orbit) a shallow entry with the lift up dips into the atmosphere and
// is thrown back out ("Abprall"). Scanned: 7.6 km/s skips at -2.0 / -2.5 deg (dips to 74 / 67 km), stays in a long
// glide at -1.6 deg; at 7.4 km/s (LEO) lift up glides instead of skipping.
TEST_CASE("Too shallow an entry skips out of the atmosphere", "[ship][entry][corridor]") {
    test::EntryStart start;
    start.flight_path_angle_rad = -2.0 * deg;
    start.speed_relative_mps = 7600.0;
    test::ShipEntryScenario scenario(start);
    const auto samples = scenario.run(constant(40.0, 0.0), 2.0);
    report("shallow", samples, scenario.body());
    double min_h = 1e9;
    for (const auto& s : samples) min_h = std::min(min_h, s.telemetry.altitude_m);
    CHECK(min_h < 80000.0);
    CHECK(scenario.body().skipped_out());
    CHECK(samples.back().telemetry.altitude_m > 120000.0);
    CHECK_FALSE(scenario.body().impacted());
}

TEST_CASE("Too steep an entry exceeds the heat or load limit", "[ship][entry][corridor]") {
    test::EntryStart start;
    start.flight_path_angle_rad = -6.0 * deg;
    test::ShipEntryScenario scenario(start);
    const auto samples = scenario.run(constant(60.0, 180.0), 2.0);
    report("steep", samples, scenario.body());
    CHECK(scenario.body().impacted());
    const auto& reason = scenario.body().breakup_reason();
    CHECK((reason == "Hitzeschild versagt" || reason == "Strukturüberlast"));
}

TEST_CASE("A nominal entry stays inside the corridor", "[ship][entry][corridor]") {
    test::EntryStart start;
    start.flight_path_angle_rad = -1.2 * deg;
    test::ShipEntryScenario scenario(start);
    const auto samples = scenario.run(constant(60.0, 0.0), 2.0);
    report("nominal", samples, scenario.body());
    CHECK_FALSE(scenario.body().impacted());
    CHECK_FALSE(scenario.body().skipped_out());
    CHECK(samples.back().telemetry.mach < 2.0);
    CHECK(scenario.body().peak_heat_rate_w_m2() <
          0.8 * scenario.body().vehicle().stages().front().config.heat_shield.max_heat_rate_w_m2);
}
