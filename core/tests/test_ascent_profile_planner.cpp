// Pre-launch ascent planner (ascent trajectory and staging, Task 3): the first-stage pitch kick chosen by trial flights
// for the most upper-stage propellant at SECO, with the booster's return planned at MECO.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/ascent_profile_planner.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

time::Epoch liftoff() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
}

sim::FlightSession& like_session() {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    static sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    return session;
}

// Starship (Block 3) from Starbase to 200 km / 26.5 deg (mission starship_leo), the booster caught at the tower.
// `extra_payload_kg`: a test that asserts trial flights gets a memo key of its own (the memo lives as long as the
// process: in a single-process run another test may already have planned the plain request).
sim::AscentProfileRequest starship_request(double extra_payload_kg = 0.0) {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    sim::AscentProfileRequest r;
    r.vehicle_id = m.vehicle_id;
    r.site_id = m.site_id;
    r.heading_rad = m.heading_rad;
    r.liftoff = liftoff();
    r.target = {m.periapsis_height_m, m.apoapsis_height_m, m.inclination_rad, m.northerly};
    r.payload_kg = m.payload_kg + extra_payload_kg;
    r.fairing = m.fairing;
    return r;
}

sim::AscentProfile timed_plan(const sim::AscentProfileRequest& request) {
    const auto start = std::chrono::steady_clock::now();
    const auto plan = sim::plan_ascent_profile(like_session(), request);
    const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::printf("[ascent_planner] kick %.2f deg, upper-stage propellant at SECO %.1f t, %d trial flights, %.1f s%s%s\n",
                plan.kick_angle_rad / deg, plan.upper_propellant_at_seco_kg / 1e3, plan.trial_flights, wall_s,
                plan.feasible ? "" : " - infeasible: ", plan.reason.c_str());
    return plan;
}

}  // namespace

TEST_CASE("The planner's kick is the optimum of a kick sweep", "[sim][ascent_planner][slow]") {
    const auto request = starship_request(100.0);  // own memo key
    struct Point {
        double kick_deg, propellant_kg;
    };
    std::vector<Point> sweep;
    for (double k = 6.0; k <= 22.0 + 1e-9; k += 1.0) {
        const double p = sim::fly_ascent_trial(like_session(), request, k * deg);
        std::printf("[ascent_planner] sweep kick %4.1f deg: %.1f t\n", k, p / 1e3);
        sweep.push_back({k, p});
    }
    const auto best = std::max_element(sweep.begin(), sweep.end(), [](const Point& a, const Point& b) {
        return std::isnan(a.propellant_kg) || (!std::isnan(b.propellant_kg) && a.propellant_kg < b.propellant_kg);
    });
    REQUIRE(!std::isnan(best->propellant_kg));
    REQUIRE(best != sweep.begin());
    REQUIRE(best + 1 != sweep.end());
    // The sweep's optimum: the best 1-deg point refined at 0.2 deg (+-1 deg around it).
    Point fine = *best;
    for (double k = best->kick_deg - 0.8; k <= best->kick_deg + 0.8 + 1e-9; k += 0.2) {
        if (std::abs(k - best->kick_deg) < 1e-9) continue;
        const double p = sim::fly_ascent_trial(like_session(), request, k * deg);
        std::printf("[ascent_planner] fine kick %4.1f deg: %.3f t\n", k, p / 1e3);
        if (!std::isnan(p) && p > fine.propellant_kg) fine = {k, p};
    }

    const auto plan = timed_plan(request);
    REQUIRE(plan.feasible);
    CHECK(plan.trial_flights > 0);
    CHECK(std::abs(plan.kick_angle_rad / deg - fine.kick_deg) <= 0.2 + 1e-9);
    for (const auto& p : sweep) {
        if (!std::isnan(p.propellant_kg)) CHECK(plan.upper_propellant_at_seco_kg >= p.propellant_kg - 1000.0);
    }
}

TEST_CASE("The optimised arc has no kink at staging", "[sim][ascent_planner]") {
    const auto request = starship_request();
    const auto plan = timed_plan(request);
    REQUIRE(plan.feasible);

    // the planned kick flown as a mission
    auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    mission.kick_angle_rad = plan.kick_angle_rad;
    auto& session = like_session();
    sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, request.liftoff);
    std::optional<double> meco_fpa_rad, separation_s;
    double worst_below_rad = -1e9;  // flight-path angle - pitch, worst in the first 60 s after the separation
    for (double t = 0.5; t < 600.0; t += 0.5) {
        session.advance_to(request.liftoff + t, 1000);
        const auto state = session.autopilot().state;
        const auto tel = session.telemetry(session.controlled_index());
        if (!meco_fpa_rad && session.mission_time_s() > 10.0 && state != sim::AutopilotState::Booster) {
            meco_fpa_rad = tel.flight_path_angle_rad;  // the first tick after MECO (state Staging)
        }
        if (!separation_s && state == sim::AutopilotState::UpperStage && session.body_count() > 1) {
            separation_s = session.mission_time_s();
        }
        if (separation_s && session.mission_time_s() <= *separation_s + 60.0) {
            const auto& s = session.body(session.controlled_index()).state();
            const auto axis = s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0});
            const double pitch_rad = std::asin(dot(axis, math::normalized(s.position_gcrf_m)));
            worst_below_rad = std::max(worst_below_rad, tel.flight_path_angle_rad - pitch_rad);
        }
        if (separation_s && session.mission_time_s() > *separation_s + 60.0) break;
    }
    REQUIRE(meco_fpa_rad);
    REQUIRE(separation_s);
    std::printf("[ascent_planner] MECO flight-path angle %.1f deg; upper stage at most %.1f deg below it in the first 60 s\n",
                *meco_fpa_rad / deg, worst_below_rad / deg);
    CHECK(*meco_fpa_rad > 10.0 * deg);  // [E]
    CHECK(*meco_fpa_rad < 40.0 * deg);  // [E]
    CHECK(worst_below_rad <= 15.0 * deg);  // [E] no kink (before: 50-70 deg below the velocity at a 64 deg MECO)
}

TEST_CASE("A repeated request is answered from the memo", "[sim][ascent_planner]") {
    auto request = starship_request(200.0);  // own memo key
    const auto first = timed_plan(request);
    REQUIRE(first.feasible);
    REQUIRE(first.trial_flights > 0);
    request.liftoff = request.liftoff + 600.0;  // a later start of the same launch
    const auto second = timed_plan(request);
    CHECK(second.trial_flights == 0);
    CHECK(second.feasible);
    CHECK(second.kick_angle_rad == first.kick_angle_rad);
    CHECK(second.upper_propellant_at_seco_kg == first.upper_propellant_at_seco_kg);
}
