// Plan time of the lunar mission starts (l6 Task 1): wall seconds per start_mission, printed. Hidden: run by name.
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "lunar_return_test_support.hpp"
#include "starsim/sim/mission_start.hpp"

using namespace starsim;

namespace {
double timed_start(const char* mission_id) {
    test::LunarReturnSession s;
    const auto t0 = std::chrono::steady_clock::now();
    const auto start = sim::start_mission(s.session, STARSIM_CONFIG_DIR, mission_id, {}, s.time);
    const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[plan_timing] %-26s %7.1f s  planned %d  %s\n", mission_id, wall_s, start.return_planned ? 1 : 0,
                start.refusal.c_str());
    if (const auto* plan = s.session.lunar_return_plan()) {
        std::printf("[plan_timing]   tei %.3f m/s  braking %.3f m/s  interface %.3f s  speed %.3f m/s  angle %.9f rad\n",
                    plan->tei_delta_v_mps, plan->braking_delta_v_mps,
                    plan->arrival.interface_epoch.tdb_seconds_past_j2000(), plan->arrival.speed_mps,
                    plan->arrival.flight_path_rad);
    }
    std::fflush(stdout);
    return wall_s;
}
}  // namespace

namespace {
// The plans of the scan before l6 Task 1 (the probe above, Ledger Task 1).
struct ReferencePlan {
    const char* mission_id;
    double tei_mps, braking_mps, interface_tdb_s, interface_speed_mps, interface_angle_rad;
    double time_limit_s;
};
constexpr double kPlanTimeLimitS = 10.0;      // user 2026-10-18
constexpr double kNrhoPlanTimeLimitS = 20.0;  // [E] two burns, 8 departure options per perilune
constexpr ReferencePlan kLlo{"ship_lunar_return_llo", 883.185, 1032.647, 848009633.349, 9695.247, -0.084746628,
                             kPlanTimeLimitS};
constexpr ReferencePlan kNrho{"ship_lunar_return_nrho", 575.758, 1007.453, 848277848.768, 9695.014, -0.084750394,
                              kNrhoPlanTimeLimitS};
constexpr ReferencePlan kAircatch{"ship_lunar_aircatch_llo", 864.312, 927.338, 848016057.211, 9790.176, -0.085253983,
                                  kPlanTimeLimitS};

// The fast planning (interpolated ephemeris, coarse Lambert bracket) finds the same plans (l6 Task 1, user 2026-10-18:
// target < 10 s). TEI within 0.5 m/s, braking within
// 1 m/s, interface epoch within 60 s, interface speed within 2 m/s and flight-path angle within 0.02 deg [E] (well
// inside the campaign's closed-loop residuals +-0.2 % / 0.1 deg).
void check_plan(const ReferencePlan& ref) {
    test::LunarReturnSession s;
    const auto t0 = std::chrono::steady_clock::now();
    const auto start = sim::start_mission(s.session, STARSIM_CONFIG_DIR, ref.mission_id, {}, s.time);
    const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    INFO(ref.mission_id << ": " << wall_s << " s");
    REQUIRE(start.return_planned);
    const auto* plan = s.session.lunar_return_plan();
    REQUIRE(plan != nullptr);
    CHECK(std::abs(plan->tei_delta_v_mps - ref.tei_mps) < 0.5);
    CHECK(std::abs(plan->braking_delta_v_mps - ref.braking_mps) < 1.0);
    CHECK(std::abs(plan->arrival.interface_epoch.tdb_seconds_past_j2000() - ref.interface_tdb_s) < 60.0);
    CHECK(std::abs(plan->arrival.speed_mps - ref.interface_speed_mps) < 2.0);
    CHECK(std::abs(plan->arrival.flight_path_rad - ref.interface_angle_rad) < 0.02 * constants::deg_to_rad);
    CHECK(wall_s < ref.time_limit_s);
}
}  // namespace

TEST_CASE("fast planning keeps the LLO return plan", "[lunar_return][plan_speed]") { check_plan(kLlo); }

TEST_CASE("fast planning keeps the NRHO return plan", "[lunar_return][plan_speed]") { check_plan(kNrho); }

TEST_CASE("fast planning keeps the aerocapture return plan", "[lunar_return][plan_speed]") { check_plan(kAircatch); }

TEST_CASE("plan time of the lunar mission starts", "[.plan_timing]") {
    for (const char* id : {"ship_lunar_return_llo", "ship_lunar_return_nrho", "ship_lunar_aircatch_llo",
                           "hls_lunar_llo"}) {
        timed_start(id);
    }
}
