// The ship's return autopilot from the LLO (l5): TEI flown on the vacuum Raptors, the coast back to the Earth, the
// braking burn, the direct entry and the catch at Starbase; a second plan is refused while flying.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_test_macros.hpp>

#include "lunar_return_test_support.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/earth_return.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/lunar_return_mission.hpp"
#include "starsim/sim/mission_start.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

// Starts the LLO direct return through start_mission; REQUIREs the plan.
void start(test::LunarReturnSession& t) {
    const auto s = sim::start_mission(t.session, STARSIM_CONFIG_DIR, "ship_lunar_return_llo", {}, t.time);
    INFO(s.refusal);
    REQUIRE(s.return_planned);
    REQUIRE(t.session.lunar_step() == sim::LunarStep::ReturnDeparture);
    REQUIRE(t.session.lunar_return_plan() != nullptr);
}

// Advances until `done` or `max_s` passed (calls of `step_s`).
template <class Done>
void run(test::LunarReturnSession& t, double max_s, double step_s, Done done) {
    const auto end = t.session.epoch() + max_s;
    while (!done() && end - t.session.epoch() > 0.0) {
        t.session.advance_to(std::min(t.session.epoch() + step_s, end), 4000000);
    }
}

const sim::LunarBurnRecord* burn_of(const sim::FlightSession& s, sim::LunarStep step) {
    for (const auto& b : s.lunar_burns()) {
        if (b.step == step) return &b;
    }
    return nullptr;
}

double geodetic_height_m(const test::LunarReturnSession& t, const math::Vec3& position_gcrf_m, time::Epoch epoch) {
    const math::Vec3 r = transpose(t.earth.fixed_to_gcrf(epoch).rotation) * position_gcrf_m;
    return frames::fixed_to_geodetic(r).height_m;
}

}  // namespace

TEST_CASE("Lunar return: TEI is flown and the coast reaches the braking height", "[lunar][return][session][slow]") {
    test::LunarReturnSession t;
    start(t);
    const double planned_tei_mps = norm(t.session.lunar_return_plan()->tei.delta_v_gcrf_mps);
    run(t, 2.0 * 86400.0, 3600.0, [&] { return t.session.lunar_step() == sim::LunarStep::ReturnCoast; });
    INFO(t.session.lunar_status());
    REQUIRE(t.session.lunar_step() == sim::LunarStep::ReturnCoast);
    const auto* tei = burn_of(t.session, sim::LunarStep::Tei);
    REQUIRE(tei != nullptr);
    if (verbose()) {
        std::printf("[lunar return] TEI planned %.1f m/s, flown %.1f m/s\n", planned_tei_mps,
                    norm(tei->node.delta_v_gcrf_mps));
    }
    CHECK(std::abs(norm(tei->node.delta_v_gcrf_mps) - planned_tei_mps) < 0.02 * planned_tei_mps);
    // Earth-bound once out of the Moon's sphere of influence (~66,000 km)
    run(t, 1.5 * 86400.0, 3600.0, [&] {
        const auto& s = t.session.body(t.session.controlled_index()).state();
        const auto rel = mission::moon_relative(t.ephemeris, {s.position_gcrf_m, s.velocity_gcrf_mps}, t.session.epoch());
        return norm(rel.position_m) > 100e6;
    });
    const auto& s = t.session.body(t.session.controlled_index()).state();
    const auto e = orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps},
                                              t.ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    REQUIRE(e.eccentricity < 1.0);
    // falling towards the Earth from beyond the Moon's distance (~406,000 km at the epoch; the flight measured 505,700
    // km osculating apogee) with a low perigee (the Moon still bends the osculating conic by thousands of km)
    CHECK(e.semi_major_axis_m * (1.0 + e.eccentricity) < 600e6);
    CHECK(e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m < 10000e3);
    CHECK(t.session.autopilot().state == sim::AutopilotState::Lunar);
}

TEST_CASE("Lunar return refuses a second plan while flying", "[lunar][return][session][slow]") {
    test::LunarReturnSession t;
    start(t);
    run(t, 2.0 * 86400.0, 3600.0, [&] { return t.session.lunar_step() == sim::LunarStep::ReturnCoast; });
    REQUIRE(t.session.lunar_step() == sim::LunarStep::ReturnCoast);
    const auto tei_epoch = t.session.lunar_return_plan()->tei.epoch;
    sim::LunarReturnSetup setup;
    CHECK_FALSE(t.session.plan_lunar_return(setup));
    CHECK(t.session.lunar_step() == sim::LunarStep::ReturnCoast);
    REQUIRE(t.session.lunar_return_plan() != nullptr);
    CHECK(t.session.lunar_return_plan()->tei.epoch - tei_epoch == 0.0);
}

// Task 8b (2026-10-16): the plan aims at one bank sign's track (~960 km beside the arrival plane) - caught, 1,055 kW/m2,
// 4.2 g, 7.9 t left.
TEST_CASE("Lunar return direct: the ship is caught at Starbase", "[lunar][return][session][slow]") {
    test::LunarReturnSession t;
    start(t);
    const auto& plan = *t.session.lunar_return_plan();
    const auto& body = t.session.body(t.session.controlled_index());
    const double braking_height_m = 2000e3;  // the mission's [return] braking_height_km
    // 1-day calls of at most 200 ticks: a coast stride is one tick, so the coast moves in large steps while the ticked
    // flight (burns, entry) is sampled every second.
    double peak_radiative = 0.0, peak_load_g = 0.0, entry_speed = 0.0, entry_angle = 0.0;
    bool entered = false;
    const auto start_epoch = t.session.epoch();
    while (!body.caught() && !body.impacted() && t.session.epoch() - start_epoch < 8.0 * 86400.0) {
        t.session.advance_to(t.session.epoch() + 86400.0, 200);
        if (t.session.autopilot().state == sim::AutopilotState::Return) {
            static bool snapshot = false;  // the hand-over state for the fast entry test (entry scenario fixture)
            if (!snapshot && std::getenv("STARSIM_VERBOSE")) {
                snapshot = true;
                const auto& s = body.state();
                std::printf("[lunar return] hand-over: epoch_tdb %.17g propellant %.17g r %.17g %.17g %.17g v %.17g %.17g %.17g "
                            "q %.17g %.17g %.17g %.17g\n",
                            t.session.epoch().tdb_seconds_past_j2000(), body.vehicle().propellant_kg(),
                            s.position_gcrf_m.x, s.position_gcrf_m.y, s.position_gcrf_m.z, s.velocity_gcrf_mps.x,
                            s.velocity_gcrf_mps.y, s.velocity_gcrf_mps.z, s.attitude.w, s.attitude.x, s.attitude.y,
                            s.attitude.z);
            }
            const auto tel = t.session.telemetry(t.session.controlled_index());
            if (!entered && tel.altitude_m < 120e3) {
                entered = true;
                entry_speed = tel.speed_relative_mps;
                entry_angle = tel.flight_path_angle_rad;
            }
            peak_radiative = std::max(peak_radiative, body.radiative_heat_rate_w_m2());
            peak_load_g = std::max(peak_load_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        }
    }
    INFO(t.session.lunar_status());
    INFO(body.breakup_reason());
    const auto* braking = burn_of(t.session, sim::LunarStep::Braking);
    std::printf("[lunar return] TEI %.1f m/s (plan %.1f), braking %.1f m/s (plan %.1f), entry %.0f m/s %.2f deg, peak heat "
                "%.0f kW/m2 (radiative %.0f), peak load %.2f g, %s, %.1f t left\n",
                burn_of(t.session, sim::LunarStep::Tei) ? norm(burn_of(t.session, sim::LunarStep::Tei)->node.delta_v_gcrf_mps) : 0.0,
                norm(plan.tei.delta_v_gcrf_mps), braking ? norm(braking->node.delta_v_gcrf_mps) : 0.0,
                plan.braking_delta_v_mps, entry_speed, entry_angle / deg, body.peak_heat_rate_w_m2() / 1000.0,
                peak_radiative / 1000.0, peak_load_g, body.caught() ? "caught" : body.impacted() ? "impacted" : "open",
                body.vehicle().propellant_kg() / 1000.0);
    if (const auto* es = t.session.entry_status(t.session.controlled_index())) {
        const auto tel = t.session.telemetry(t.session.controlled_index());
        std::printf("[lunar return]   entry phase %s, downrange %.1f km, crossrange %.1f km, height %.0f m, %.0f m/s\n",
                    gnc::to_string(es->phase), es->downrange_error_m / 1000.0, es->crossrange_m / 1000.0,
                    tel.altitude_m, tel.speed_relative_mps);
    }
    for (const auto& b : t.session.lunar_burns()) {
        std::printf("[lunar return]   %s %.2f m/s at %.0f km\n", sim::to_string(b.step), norm(b.node.delta_v_gcrf_mps),
                    geodetic_height_m(t, b.centre_gcrf_m, b.node.epoch) / 1000.0);
    }
    REQUIRE(braking != nullptr);
    CHECK(std::abs(geodetic_height_m(t, braking->centre_gcrf_m, braking->node.epoch) - braking_height_m) < 50e3);
    CHECK(body.breakup_reason().find("Hitzeschild") == std::string::npos);
    CHECK(body.peak_heat_rate_w_m2() < 1.2e6);
    CHECK(body.vehicle().propellant_kg() >= 0.0);
    CHECK(body.caught());
}
