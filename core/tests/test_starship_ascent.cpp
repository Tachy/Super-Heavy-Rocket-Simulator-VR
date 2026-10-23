// Starship (Block 3) ascent against flight 14's webcast telemetry (ascent trajectory and staging, Task 6): the
// validation mission (config/missions/flight14_ascent.toml: its insertion orbit, payload and propellant load) flown
// through the onboard computer's plan - the kick, the catch and MECO from the booster's return are planned, nothing is
// fitted to the recording. The recording is the check.

#include <cmath>
#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "ascent_validation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
const char* const table = "flight14_2026-09-28.csv";

// [D] The table shows the stack's state 3.50 s late against the clamps' release (T+0 of the simulation): its early
// speed (8-40 m/s, drag-free) extrapolates to zero at T+3.50 s -
//   python tools/telemetry/liftoff_time.py tests/validation/telemetry/flight14_2026-09-28.csv
//   -> quadratic: speed zero at T+3.50 s, acceleration 4.79 m/s^2 at T+8.8 s
// (flight 13: 3.24 s; Falcon 9 CRS-35 / Starlink 15-25: 3.42 / 3.26 s). Compared without it the stack ran ~12 % fast
// at T+30 s (the former "open discrepancy").
constexpr double time_base_s = 3.50;
// [D] The altitude dial lags the speed dial by another 1.95 s (same command: the displayed speed integrated through the
// vertical rise matches the 0.1 km readings of T+12-32 s with a mean error of 2 m): its time base T+5.45 s. Compared on
// the speed dial's time base the altitude ran 14 % high at T+30 s.
constexpr double altitude_time_base_s = 5.45;

// Recorded MECO (display time): the last booster reading T+138.75 s (5943 km/h, 51.0 km, still accelerating) before
// the staging gap (no readings T+139-146 s): MECO at T+139.5 +- 0.75 s.
constexpr double recorded_meco_display_s = 139.5;
constexpr double recorded_meco_uncertainty_s = 0.75;
// Recorded SECO (display time): the ship's speed levels at 26,507-26,509 km/h from T+507.25 s (the tail-off ends);
// the cutoff T+506.5 +- 1 s.
constexpr double recorded_seco_display_s = 506.5;
constexpr double recorded_seco_uncertainty_s = 1.0;

// Open discrepancies (docs/subsystems.md "Ascent trajectory and staging", Task 6), per recorded sample as fractions of
// the recorded value (test::allowance: the measured gap + test::allowance_margin; a regression beyond it fails). Up to
// max-Q the model follows the flight within the readings (gap 0: T+30 s -0.9 % speed, -2.1 % altitude). From max-Q (the
// recorded q holds at 38-39 kPa from T+58 s display, peak_dynamic_pressure.py) flight 14 throttles below the
// operational throttle (gauge flow 0.78-0.90 of the rated against our 0.88: our q peaks at 38.7 kPa, under the 38.9 kPa
// limit, so our limiter never throttles) and one centre engine is out from T+76 s (32 of 33, not modelled).
constexpr test::SampleGap sample_gaps[] = {
    {30.0, 0.0, 0.0},
    {60.0, 0.017, 0.006},  // +1.7 % fast, +0.6 % high
    {90.0, 0.069, 0.074},  // +6.9 %, +7.4 %
    {120.0, 0.090, 0.112},  // +9.0 %, +11.2 %
};
// MECO: ours comes from the catch's return plan (294 t left in the booster); flight 14 staged with ~500 t by its gauge
// (not followed after staging, decision 2026-10-14) after the slower ascent above: T+129.8 against T+136.0 s (-4.6 %),
// 6559 against 5943 km/h (+10.4 %), 52.5 against 51.0 km (+3.0 %), path 30.3 against 27.8 deg.
constexpr double meco_time_allowance = 0.06;
constexpr double meco_speed_allowance = 0.12;
constexpr double meco_altitude_allowance = 0.05;
constexpr double meco_gamma_allowance_deg = 4.0;
// SECO: flight 14 flew its last ~67 s on 5 of 6 engines (RVac out at T+438 s), staged ~600 km/h slower and inserted
// ~105 m/s lower (periapsis -44 km, ours 165 km): ours cuts off T+447 against T+503 s (-11.1 %).
constexpr double seco_time_allowance = 0.13;

const test::FlownAscent& flight14() {
    static const test::FlownAscent flown = [] {
        test::spice_session();
        const spice::SpiceTimeConverter time;
        return test::fly_validation_ascent("flight14_ascent", time.from_utc({2026, 9, 28, 12, 48, 59.0}));  // [S:flights.toml]
    }();
    return flown;
}

}  // namespace

TEST_CASE("Starship ascent to flight 14's insertion orbit against its webcast telemetry",
          "[gnc][validation][ascent][slow]") {
    const auto& a = flight14();
    REQUIRE(a.plan.feasible);
    REQUIRE(a.seco_s > 0.0);
    std::printf("[validation] flight 14: kick %.2f deg, MECO T+%.2f s with %.1f t in the booster, SECO T+%.1f s, "
                "%.1f x %.1f km, i %.2f deg, %.1f t left in the ship\n",
                a.plan.kick_angle_rad / deg, a.meco.t_s, a.booster_meco_propellant_t, a.seco_s, a.periapsis_km,
                a.apoapsis_km, a.inclination_deg, a.upper_propellant_at_seco_t);
    // the insertion orbit flown
    CHECK(std::abs(a.periapsis_km - 165.0) < 5.0);
    CHECK(std::abs(a.apoapsis_km - 276.0) < 5.0);
    CHECK(std::abs(a.inclination_deg - 30.5) < 0.1);

    // every recorded sample of the booster phase (engine-outs not modelled: flight 14's 33rd engine is out from T+76 s)
    for (const auto& gap : sample_gaps) {
        const auto speed = test::compare_speed(table, a, gap.t_s, time_base_s, test::allowance(gap.speed));
        const auto altitude =
            test::compare_altitude(table, a, gap.t_s, altitude_time_base_s, test::allowance(gap.altitude));
        test::print("flight 14", speed);
        test::print("flight 14", altitude);
        CHECK(speed.within());
        CHECK(altitude.within());
    }

    // MECO: the recorded state at the recorded MECO against the simulated state at the simulated MECO. The last
    // readings before the staging gap: speed at MECO; the altitude dial shows the state `lag` earlier, compared with the
    // simulation that much before its MECO (the path angle from its rate and the speed of that instant).
    // Reading tolerance at MECO: rate x 2 x uncertainty - the last reading lies at the lower end of the MECO's interval
    // (T+139.5 +- 0.75 s), so the stack may still accelerate over the whole interval after it (CRS-35 reads at the
    // centre of its MECO interval: rate x 1 x uncertainty).
    const double last_reading_s = recorded_meco_display_s - recorded_meco_uncertainty_s;
    const double lag = altitude_time_base_s - time_base_s;
    const auto r = test::recorded_stack(table, last_reading_s);
    const auto r_lag = test::recorded_stack(table, last_reading_s - lag);
    const auto s_lag = a.at(a.meco.t_s - lag);
    const double recorded_gamma_deg = std::asin(r.altitude_rate_km_s * 3600.0 / r_lag.speed_kmh) / deg;
    const double gamma_reading_deg =
        test::altitude_resolution_km / 2.0 * 3600.0 / r_lag.speed_kmh / deg;  // the rate over the +-2 s line
    const test::Comparison meco[] = {
        {"MECO time [s]", recorded_meco_display_s - time_base_s, a.meco.t_s,
         recorded_meco_uncertainty_s + test::time_base_uncertainty_s, meco_time_allowance},
        {"MECO speed [km/h]", r.speed_kmh, a.meco.speed_kmh,
         test::speed_resolution_kmh + std::abs(r.speed_rate_kmh_s) * 2.0 * recorded_meco_uncertainty_s,
         meco_speed_allowance},
        {"MECO altitude [km]", r.altitude_km, s_lag.altitude_km,
         test::altitude_resolution_km + std::abs(r.altitude_rate_km_s) * 2.0 * recorded_meco_uncertainty_s,
         meco_altitude_allowance},
        {"MECO path angle [deg]", recorded_gamma_deg, s_lag.gamma_deg, gamma_reading_deg + meco_gamma_allowance_deg, 0.0},
        {"SECO time [s]", recorded_seco_display_s - time_base_s, a.seco_s,
         recorded_seco_uncertainty_s + test::time_base_uncertainty_s, seco_time_allowance},
    };
    for (const auto& c : meco) {
        test::print("flight 14", c);
        CHECK(c.within());
    }
}
