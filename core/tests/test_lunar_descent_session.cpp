// Descent autopilot from the LLO (l4): one time-warp call from the LLO to the landing (steps not skipped), pilot
// override in the braking phase, refusals.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"
#include "lunar_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;


struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    sim::LunarSite site;

    Fixture() {
        const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
        session.set_moon_terrain(terrain);
        site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
        session.set_lunar_landing_site(site);
    }
    // Circular polar 100 km LLO whose plane contains the site at t0 + align_s (as test_lunar_descent.cpp).
    void place_in_llo(double propellant_kg, double align_s, double dry_kg = 0.0) {
        const auto t = t0 + align_s;
        const auto rot = moon_me.fixed_to_gcrf(t).rotation;
        const Vec3 pole = rot * Vec3{0.0, 0.0, 1.0};
        const double lat = site.config.latitude_rad, lon = site.config.longitude_rad;
        const Vec3 s = rot * Vec3{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
        const Vec3 n = math::normalized(cross(pole, s));
        const double r = constants::moon_mean_radius_m + 100e3, gm = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
        const Vec3 x = math::normalized(cross(n, pole)), y = cross(n, x);
        const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
        session.place_in_orbit(test::hls_with_propellant(propellant_kg, dry_kg),
                               {moon.position_m + x * r, moon.velocity_mps + y * std::sqrt(gm / r)}, t0);
    }
};
}  // namespace

TEST_CASE("From the LLO the HLS waits for the site, trims, lowers its orbit and lands", "[sim][lunar_descent_session][terrain][data][slow]") {
    Fixture fx;
    fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    sim::LunarLandingSetup setup;
    setup.surface_stay_s = 3600.0;
    REQUIRE(fx.session.plan_lunar_landing(setup));
    CHECK_FALSE(fx.session.plan_lunar_landing(setup));   // a descent under way keeps its plan
    const auto* plan = fx.session.lunar_descent_plan();
    REQUIRE(plan);
    // Review focus 1: one warp call over the wait, the trim, the DOI and the PDI.
    fx.session.advance_to(plan->pdi_epoch + 1200.0, 50'000'000);
    std::vector<sim::LunarStep> flown;
    for (const auto& b : fx.session.lunar_burns()) flown.push_back(b.step);
    CHECK(std::find(flown.begin(), flown.end(), sim::LunarStep::Doi) != flown.end());
    if (plan->plane_trim) CHECK(std::find(flown.begin(), flown.end(), sim::LunarStep::PlaneTrim) != flown.end());
    const auto& td = fx.session.lunar_touchdown();
    REQUIRE(td);
    std::printf("[lunar_descent_session] %s, offset %.2f m, sink %.2f m/s, wait %.2f d, %.1f t -> %.1f t landed, status %s\n",
                sim::to_string(td->outcome), td->offset_m, td->sink_mps, (plan->pdi_epoch - fx.t0) / 86400.0,
                m0 / 1000.0, fx.session.body(0).vehicle().mass_properties().mass_kg / 1000.0,
                fx.session.lunar_status().c_str());
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);
    CHECK(fx.session.lunar_step() == sim::LunarStep::SurfaceStay);
    fx.session.advance_to(fx.session.epoch() + 2.0 * 3600.0, 50'000'000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::Landed);   // stay over (no ascent requested)
    CHECK(fx.session.body(0).resting());
}

namespace {

// Flies a planned descent from 30 min before the PDI one session step at a time, holding the HLS's main-tank liquid
// settled (or shaking it loose) before every step through test access to the flown vehicle (the coast between the
// steps moves it). Returns when the braking Raptors were lit - they give thrust, or an unsettled start lost them -
// and whether the settling push ran before.
struct PdiRun {
    std::optional<time::Epoch> lit;
    bool pushed = false;
};
PdiRun fly_pdi(Fixture& fx, double settled, double after_pdi_s) {
    const auto pdi = fx.session.lunar_descent_plan()->pdi_epoch;
    fx.session.advance_to(pdi - 1800.0, 50'000'000);
    auto& stage = const_cast<vehicle::FlightBody&>(fx.session.body(0)).vehicle().stages()[0];
    PdiRun run;
    while (fx.session.epoch() < pdi + after_pdi_s && !run.lit) {
        stage.settled = settled;
        fx.session.advance_to(pdi + after_pdi_s, 1);
        run.pushed = run.pushed || fx.session.settling();
        if (fx.session.body(0).vehicle().any_engine_producing_thrust() || !fx.session.burn_abort_reason().empty()) {
            run.lit = fx.session.epoch();
        }
    }
    return run;
}

}  // namespace

TEST_CASE("PDI: settled propellant needs no push and lights on time",
          "[sim][lunar_descent_session][ignition_settling][terrain][data][slow]") {
    // f3 final review M10: the push ran even when settled and moved the ignition one lead early
    Fixture fx;
    fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
    sim::LunarLandingSetup setup;
    REQUIRE(fx.session.plan_lunar_landing(setup));
    const auto pdi = fx.session.lunar_descent_plan()->pdi_epoch;
    const auto run = fly_pdi(fx, 1.0, 120.0);
    INFO(fx.session.lunar_status() << " / " << sim::to_string(fx.session.lunar_step()) << ", abort: "
         << fx.session.burn_abort_reason());
    REQUIRE(run.lit);
    CHECK_FALSE(run.pushed);
    CHECK(*run.lit - pdi > -10.0);  // [E] the braking Raptors' start-up before the PDI, not a whole lead
    CHECK(*run.lit - pdi < 5.0);
}

TEST_CASE("PDI: unsettled propellant holds the ignition while the push settles it",
          "[sim][lunar_descent_session][ignition_settling][terrain][data][slow]") {
    // f3 final review M9: the PDI lit after the push without the hold the burns have
    Fixture fx;
    fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
    sim::LunarLandingSetup setup;
    REQUIRE(fx.session.plan_lunar_landing(setup));
    const auto pdi = fx.session.lunar_descent_plan()->pdi_epoch;
    const double lead = vehicle::settling_push(fx.session.body(0).vehicle(), 0).lead_s;  // the push's lead
    REQUIRE(lead > 10.0);
    const auto run = fly_pdi(fx, 0.0, sim::FlightSession::settling_hold_factor * lead + 300.0);
    INFO(fx.session.lunar_status() << " / " << sim::to_string(fx.session.lunar_step()) << ", abort: "
         << fx.session.burn_abort_reason());
    REQUIRE(run.lit);
    CHECK(run.pushed);
    CHECK(*run.lit - pdi > 0.5 * lead);  // held past the planned ignition (bounded: it lights in the end)
}

TEST_CASE("A surface stay of 0 h with an ascent lifts off at once", "[sim][lunar_descent_session][terrain][data][slow]") {
    // Final review l4 I1: stay_hours = 0 is a valid mission value; with an ascent requested the HLS must not stay Landed.
    Fixture fx;
    fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
    sim::LunarLandingSetup setup;
    setup.surface_stay_s = 0.0;
    setup.ascent = gnc::LunarAscentParameters{};
    REQUIRE(fx.session.plan_lunar_landing(setup));
    const auto* plan = fx.session.lunar_descent_plan();
    REQUIRE(plan);
    fx.session.advance_to(plan->pdi_epoch + 1200.0, 50'000'000);
    REQUIRE(fx.session.lunar_touchdown());
    REQUIRE(fx.session.lunar_touchdown()->outcome == sim::LegTouchdown::Landed);
    fx.session.advance_to(fx.session.epoch() + 60.0, 50'000);
    std::printf("[lunar_descent_session] stay 0 h: step %d, %s\n", static_cast<int>(fx.session.lunar_step()),
                fx.session.lunar_status().c_str());
    CHECK(fx.session.lunar_step() != sim::LunarStep::Landed);
    CHECK_FALSE(fx.session.body(0).resting());   // lifted off
}

TEST_CASE("An ascent refused at the stay's end ends the mission sequence", "[sim][lunar_descent_session][terrain][data][slow]") {
    // Review l4 M3: the refusal left the sequence in leg Surface without its reason prefixed.
    Fixture fx;
    fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
    sim::LunarMissionSequence sequence;
    sequence.landing.surface_stay_s = 36.0;
    sequence.landing.ascent = gnc::LunarAscentParameters{};
    sequence.landing.ascent->ascent_groups = {"no_such_group"};   // refused: "keine Aufstiegstriebwerke"
    REQUIRE(fx.session.plan_lunar_landing(sequence.landing));
    fx.session.set_lunar_mission(sequence);
    const auto* plan = fx.session.lunar_descent_plan();
    REQUIRE(plan);
    fx.session.advance_to(plan->pdi_epoch + 1200.0, 50'000'000);
    REQUIRE(fx.session.lunar_touchdown());
    REQUIRE(fx.session.lunar_touchdown()->outcome == sim::LegTouchdown::Landed);
    fx.session.advance_to(fx.session.epoch() + 60.0, 50'000);
    std::printf("[lunar_descent_session] refused ascent: step %d, %s\n", static_cast<int>(fx.session.lunar_step()),
                fx.session.lunar_status().c_str());
    CHECK(fx.session.lunar_mission_ended());
    CHECK(fx.session.lunar_status().rfind("Mission beendet: Kein Aufstieg", 0) == 0);
    CHECK(fx.session.body(0).resting());
}

TEST_CASE("The pilot takes over in the braking phase: the descent guidance stops for good", "[sim][lunar_descent_session][terrain][data][slow]") {
    Fixture fx;
    fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
    REQUIRE(fx.session.plan_lunar_landing({}));
    const auto pdi = fx.session.lunar_descent_plan()->pdi_epoch;
    fx.session.advance_to(pdi + 60.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::PoweredDescent);
    fx.session.disengage_autopilot();
    sim::PilotInput input;
    input.throttle = 0.0;
    fx.session.set_input(input);
    fx.session.cutoff();
    fx.session.advance_to(fx.session.epoch() + 10.0, 5000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::None);
    CHECK(fx.session.autopilot().state == sim::AutopilotState::Off);
    CHECK(fx.session.lunar_landing_guidance() == nullptr);
    CHECK_FALSE(fx.session.body(0).vehicle().any_engine_producing_thrust());
}

TEST_CASE("The pilot takes over in the approach phase: the descent guidance stops for good", "[sim][lunar_descent_session][terrain][data][slow]") {
    // Review l4 M5: the takeover outside the braking phase.
    Fixture fx;
    fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
    REQUIRE(fx.session.plan_lunar_landing({}));
    const auto pdi = fx.session.lunar_descent_plan()->pdi_epoch;
    fx.session.advance_to(pdi + 1.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::PoweredDescent);
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent &&
                    fx.session.lunar_landing_guidance()->phase() != gnc::LunarDescentPhase::Approach;
         ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
    }
    REQUIRE(fx.session.lunar_landing_guidance());
    REQUIRE(fx.session.lunar_landing_guidance()->phase() == gnc::LunarDescentPhase::Approach);
    fx.session.disengage_autopilot();
    sim::PilotInput input;
    input.throttle = 0.0;
    fx.session.set_input(input);
    fx.session.cutoff();
    fx.session.advance_to(fx.session.epoch() + 10.0, 5000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::None);
    CHECK(fx.session.autopilot().state == sim::AutopilotState::Off);
    CHECK(fx.session.lunar_landing_guidance() == nullptr);
    CHECK_FALSE(fx.session.body(0).vehicle().any_engine_producing_thrust());
}

TEST_CASE("Descent planning refusals","[sim][lunar_descent_session][terrain][data]") {
    Fixture fx;
    SECTION("not in a lunar orbit") {
        fx.session.place_in_orbit("starship_hls", orbit::state_from_elements({6578137.0, 0.0, 0.5, 0.0, 0.0, 0.0},
                                                                             fx.ephemeris.gm_m3ps2(ephemeris::Body::Earth)),
                                  fx.t0);
        CHECK_FALSE(fx.session.plan_lunar_landing({}));
        CHECK(fx.session.lunar_status().find("Mondumlaufbahn") != std::string::npos);
    }
    SECTION("propellant short for descent and ascent (204 t dry, half tanks)") {
        fx.place_in_llo(400e3, 0.5 * 86400.0, 204e3);
        CHECK_FALSE(fx.session.plan_lunar_landing({}));
        CHECK(fx.session.lunar_status().find("Treibstoff") != std::string::npos);
    }
    SECTION("no admissible touchdown point (review focus 4)") {
        fx.place_in_llo(test::hls_llo_propellant_kg, 0.5 * 86400.0);
        sim::LunarLandingSetup setup;
        setup.descent.slope_limit_rad = 0.01 * deg;
        CHECK_FALSE(fx.session.plan_lunar_landing(setup));
        CHECK(fx.session.lunar_status().find("Landepunkt") != std::string::npos);
        CHECK(fx.session.lunar_step() == sim::LunarStep::None);
        CHECK(fx.session.lunar_burns().empty());
    }
}

TEST_CASE("A forced engine set never falls back to the RCS", "[sim][lunar_descent_session][burn_engines]") {
    Fixture fx;
    fx.place_in_llo(450e3, 0.5 * 86400.0);  // ~570 t: the HLS in the LLO after a full-tank outbound flight
    const auto& s = fx.session.body(0).state();
    const math::Vec3 prograde = math::normalized(
        s.velocity_gcrf_mps - fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.t0).velocity_mps);
    // 1 m/s is below a Raptor's start/shutdown impulse even at minimum throttle: refused, not flown on cold gas
    CHECK_FALSE(fx.session.execute_burn({fx.t0 + 600.0, prograde * 1.0}, {"sea_level_1"}));
    // 17 m/s (a plane trim) fits the single Raptor at minimum throttle
    REQUIRE(fx.session.execute_burn({fx.t0 + 600.0, prograde * 17.0}, {"sea_level_1"}));
    CHECK_FALSE(fx.session.burn_configuration().rcs);
    REQUIRE(fx.session.burn_configuration().groups.size() == 1);
    CHECK(fx.session.burn_configuration().groups.front() == "hls/sea_level_1");
}

TEST_CASE("At ~570 t the plane trim and the DOI fly on the single sea-level Raptor, not on cold gas",
          "[sim][lunar_descent_session][burn_engines][terrain][data]") {
    Fixture fx;
    fx.place_in_llo(450e3, 0.5 * 86400.0);
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(fx.session.plan_lunar_landing({}));
    const auto* plan = fx.session.lunar_descent_plan();
    REQUIRE(plan);
    fx.session.advance_to(plan->doi.epoch + 600.0, 50'000'000);
    REQUIRE_FALSE(fx.session.lunar_burns().empty());
    CHECK(fx.session.lunar_step() == sim::LunarStep::DescentCoast);
    const double used = m0 - fx.session.body(0).vehicle().mass_properties().mass_kg;
    std::printf("[burn_engines] trim %.2f m/s + DOI %.2f m/s at %.0f t: %.2f t used\n",
                plan->plane_trim ? norm(plan->plane_trim->delta_v_gcrf_mps) : 0.0, norm(plan->doi.delta_v_gcrf_mps),
                m0 / 1000.0, used / 1000.0);
    // rocket equation at Isp 350 s (plus a day of Sun pointing): ~3-6 t; cold gas (~66 s) took ~30 t
    CHECK(used < 8e3);
}
