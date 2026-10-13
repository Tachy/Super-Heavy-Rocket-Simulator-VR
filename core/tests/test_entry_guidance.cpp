// Ship entry guidance (b2): the onboard computer's aero table and point-mass predictor against the 6-DOF ship,
// the bank corrector, and the guided entry and belly flop.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <catch2/catch_test_macros.hpp>

#include "ship_entry_scenario.hpp"
#include "starsim/gnc/attitude_slew.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_guidance.hpp"
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

namespace {

// `end` moved by `distance_m` further along the great circle start -> end.
math::Vec3 shift_along(const math::Vec3& start, const math::Vec3& end, double distance_m) {
    const math::Vec3 n = math::normalized(cross(start, end));
    const double a = distance_m / constants::wgs84_semi_major_axis_m;
    return end * std::cos(a) + cross(n, end) * std::sin(a);
}

gnc::EntryGuidanceParameters test_parameters() {
    gnc::EntryGuidanceParameters p;
    p.predictor_j2 = false;  // the scenario's Earth has central gravity only
    return p;
}

// Ground point under the predicted gate for a constant bank from the scenario's start, moved downrange.
gnc::EntryTarget predicted_target(test::ShipEntryScenario& scenario, const gnc::AlphaSchedule& schedule,
                                  double bank_deg, double shift_m) {
    const auto model = predictor_model(scenario, schedule);
    const auto p = gnc::predict_entry(model, predictor_start(scenario), bank_deg * deg, gnc::EntryTarget{}, 1500.0);
    const math::Vec3 ground = p.end_position_itrf_m * ((norm(p.end_position_itrf_m) - 1500.0) / norm(p.end_position_itrf_m));
    return {shift_along(scenario.start_position_itrf_m(), ground, shift_m), 0.0};
}

}  // namespace

TEST_CASE("Entry corrector hits the target on its own model", "[ship][guidance]") {
    const auto schedule = ship_alpha_schedule();
    test::ShipEntryScenario scenario(test::EntryStart{});
    const auto start = predictor_start(scenario);

    const gnc::EntryGuidance nominal(scenario.body(), schedule, predicted_target(scenario, schedule, 55.0, 0.0),
                                     test_parameters());
    const auto s = nominal.solve(nominal.model(), start, 0.0, 1.0, 0.0);
    if (verbose()) {
        std::cout << "corrector: bank " << s.bank_rad / deg << " deg, error " << s.prediction.downrange_error_m
                  << " m, " << s.predictions << " predictions\n";
    }
    CHECK(std::abs(s.bank_rad / deg - 55.0) < 3.0);
    CHECK(std::abs(s.prediction.downrange_error_m) < 1000.0);
    CHECK_FALSE(s.constrained);

    const gnc::EntryGuidance far(scenario.body(), schedule, predicted_target(scenario, schedule, 0.0, 3.0e6),
                                 test_parameters());
    CHECK(far.solve(far.model(), start, 0.0, 1.0, 60.0 * deg).bank_rad == 0.0);
    const gnc::EntryGuidance near(scenario.body(), schedule, predicted_target(scenario, schedule, 150.0, -1.0e6),
                                  test_parameters());
    // Short of even the steepest entry: the bank would go to its limit, but beyond ~77 deg the predicted load
    // exceeds 90 % of the 6 g limit - the corridor cuts it back (range given up, not the vehicle).
    const auto short_target = near.solve(near.model(), start, 0.0, 1.0, 60.0 * deg);
    CHECK(short_target.constrained);
    CHECK(short_target.prediction.peak_load_g <= 0.9 * 6.0);
    CHECK(short_target.bank_rad > 60.0 * deg);
    CHECK(short_target.bank_rad < test_parameters().max_bank_rad);
}

TEST_CASE("Guided 6-DOF entry flies to the gate", "[ship][guidance]") {
    const auto schedule = ship_alpha_schedule();
    test::ShipEntryScenario scenario(test::EntryStart{});
    gnc::EntryGuidance guidance(scenario.body(), schedule, predicted_target(scenario, schedule, 50.0, 100000.0),
                                test_parameters());
    const auto samples = scenario.run_guided(guidance, gnc::EntryPhase::Gate);
    REQUIRE_FALSE(samples.empty());
    if (verbose()) {
        for (size_t i = 0; i < samples.size(); i += 30) {
            const auto& g = samples[i];
            std::cout << "t " << g.t_s << " h " << g.telemetry.altitude_m / 1e3 << " v " << g.telemetry.speed_relative_mps
                      << " " << gnc::to_string(g.command.phase) << " bank " << g.command.bank_rad / deg << " alpha "
                      << g.command.alpha_rad / deg << " downrange " << g.command.downrange_error_m / 1e3 << " cross "
                      << g.command.crossrange_m / 1e3 << " scales " << g.command.drag_scale << "/"
                      << g.command.lift_scale << "\n";
        }
    }
    const double miss = gnc::ground_range_m(scenario.position_itrf_m(), guidance.target().position_itrf_m);
    if (verbose()) {
        std::cout << "gate at " << samples.back().t_s << " s, miss " << miss << " m, peak heat "
                  << scenario.body().peak_heat_rate_w_m2() / 1e3 << " kW/m2\n";
    }
    CHECK(guidance.phase() == gnc::EntryPhase::Gate);
    CHECK_FALSE(scenario.body().impacted());
    CHECK(miss < 100.0);
}

TEST_CASE("Belly flop attitude faces the flow and points the nose along the lift", "[ship][guidance]") {
    const math::Vec3 up{0.0, 0.0, 1.0}, north{1.0, 0.0, 0.0};
    const math::Vec3 falling = -up * 100.0;  // vertical fall: a bank from the vertical plane is undefined
    const double belly_azimuth = 270.0 * deg, alpha = 75.0 * deg;
    const auto a = gnc::entry_attitude_with_lift(falling, north, alpha, belly_azimuth);
    const math::Vec3 z = cross(a.axis_gcrf, a.roll_reference_gcrf);
    const math::Vec3 belly = a.roll_reference_gcrf * std::cos(belly_azimuth) + z * std::sin(belly_azimuth);
    CHECK(std::abs(dot(belly, -up) - std::sin(alpha)) < 1e-12);           // belly towards the flow (from below)
    CHECK(std::abs(dot(a.axis_gcrf, north) - std::sin(alpha)) < 1e-12);  // nose towards the lift ...
    CHECK(std::abs(dot(a.axis_gcrf, -up) - std::cos(alpha)) < 1e-12);    // ... tipped down by 90 deg - alpha
    CHECK(std::abs(dot(a.axis_gcrf, {0.0, 1.0, 0.0})) < 1e-12);          // no sideslip

    // slew limiter: at most the given angle per call, exact when within it
    const math::Quat from{}, to = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 30.0 * deg);
    const math::Quat step = gnc::slew_toward(from, to, 5.0 * deg);
    CHECK(std::abs(2.0 * std::acos(step.w) - 5.0 * deg) < 1e-9);
    CHECK(gnc::slew_toward(from, to, 40.0 * deg).w == to.w);
}

TEST_CASE("Belly flop steers to the gate", "[ship][guidance]") {
    test::EntryStart start;
    start.height_m = 12000.0;
    start.speed_relative_mps = 120.0;
    start.flight_path_angle_rad = -85.0 * deg;
    start.heading_rad = 0.0;  // moving north (as after the entry: the gate ahead, slightly off the track)
    test::ShipEntryScenario scenario(start);
    const double along_m = 2000.0, across_m = 500.0;  // gate point north and east of the start's ground point
    const frames::GeodeticPosition target_geodetic{
        start.latitude_rad + along_m / constants::wgs84_semi_major_axis_m,
        across_m / (constants::wgs84_semi_major_axis_m * std::cos(start.latitude_rad)), 0.0};
    const gnc::EntryTarget target{frames::geodetic_to_fixed(target_geodetic), 0.0};
    gnc::EntryGuidance guidance(scenario.body(), ship_alpha_schedule(), target, test_parameters());
    guidance.skip_to(gnc::EntryPhase::Terminal);
    const auto samples = scenario.run_guided(guidance, gnc::EntryPhase::Gate, 600.0);
    REQUIRE_FALSE(samples.empty());

    const auto& body = scenario.body();
    const math::Vec3 up_gcrf = math::normalized(body.state().position_gcrf_m);
    const double az = body.vehicle().stages().front().config.aero.belly_azimuth_rad;
    const math::Vec3 belly = body.state().attitude.rotate({0.0, std::cos(az), std::sin(az)});
    const double belly_off_down = std::acos(std::clamp(dot(belly, -up_gcrf), -1.0, 1.0));
    const double miss = gnc::ground_range_m(scenario.position_itrf_m(), target.position_itrf_m);
    const auto& t = samples.back().telemetry;
    if (verbose()) {
        for (size_t i = 0; i < samples.size(); i += 10) {
            const auto& g = samples[i];
            std::cout << "t " << g.t_s << " h " << g.telemetry.altitude_m << " v " << g.telemetry.speed_relative_mps
                      << " gamma " << g.telemetry.flight_path_angle_rad / deg << " aoa "
                      << g.telemetry.angle_of_attack_rad / deg << " zem " << g.command.downrange_error_m << " / "
                      << g.command.crossrange_m << " alpha cmd " << g.command.alpha_rad / deg << "\n";
        }
        std::cout << "gate: " << samples.back().t_s << " s, h " << t.altitude_m << " m, miss " << miss << " m, sink "
                  << -t.vertical_speed_mps << " m/s, belly " << belly_off_down / deg << " deg off down\n";
    }
    CHECK(guidance.phase() == gnc::EntryPhase::Gate);
    CHECK_FALSE(body.impacted());
    CHECK(miss < 50.0);
    CHECK(belly_off_down < 15.0 * deg);
    CHECK(-t.vertical_speed_mps < 200.0);
}




