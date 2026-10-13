// Falcon 9 return to the landing zone (milestone c3): entry burn, landing on SCvx plans onto the legs at LZ-40.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const spice::SpiceEphemeris& de440() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

time::Epoch start() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 10, 40, 0.0});
}

sim::LaunchSite lz40() {
    return sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40"));
}

gnc::BoosterReturnParameters falcon9_parameters() {
    return gnc::booster_return_parameters(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5").stages.front());
}

}  // namespace

TEST_CASE("A Falcon 9 MECO plan keeps the entry burn's propellant", "[gnc][falcon9][rtls]") {
    // The stack after ~2 min of flight east of the Cape, coasting north-east at ~1.9 km/s.
    const auto t0 = start();
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const vehicle::Vehicle stack(cfg, STARSIM_CONFIG_DIR);
    const auto site = lz40();
    const gnc::LandingSite landing{site.latitude_rad, site.longitude_rad, site.height_m, {}, {}, {}};
    const auto tr = earth().fixed_to_gcrf(t0);
    const Vec3 r_fixed = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad + 0.6 * constants::deg_to_rad, 60e3});
    const Vec3 up = math::normalized(r_fixed);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    const math::StateVector state{tr.position(r_fixed), tr.velocity(r_fixed, east * 1500.0 + up * 1000.0)};
    auto with_entry = falcon9_parameters();
    auto without = with_entry;
    without.entry_altitude_m = 0.0;
    const double landing_kg = 11300.0;
    const double a = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, landing, with_entry, landing_kg);
    const double b = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, landing, without, landing_kg);
    // Entry burn by the rocket equation on the stage (dry 25.6 t + 11.3 t) at the centre engine's vacuum exhaust
    // velocity (311 s): ~1.6 t (the CRS-35 gauge: 3.5 -> 3.1 % of 395.7 t), more after the boostback's leverage.
    std::printf("[falcon9] MECO propellant with entry burn %.1f t, without %.1f t\n", a / 1e3, b / 1e3);
    CHECK(a - b > 1500.0);
    CHECK(a - b < 3000.0);
}

TEST_CASE("A falling Falcon 9 stage flies its entry burn on the centre engine", "[gnc][falcon9][rtls]") {
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(lz40());
    const auto cfg = test::booster_config("falcon9_block5", 14000.0);
    // 70 km above the zone, coming back from the east at 300 m/s, falling at 1100 m/s, engines first.
    const Vec3 velocity{-300.0, 0.0, -1100.0};
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, {20e3, 0.0, 70e3}, velocity),
                                    t0, falcon9_parameters(), gnc::ReturnPhase::Coast);
    bool burned = false, ended = false;
    double ignition_altitude_m = 0.0;
    std::vector<std::string> groups;
    for (double t = 0.0; t < 60.0 && !ended;) {
        t += sim::FlightSession::tick_s;
        session.advance_to(t0 + t, 10);
        const auto* status = session.return_status(0);
        REQUIRE(status != nullptr);
        if (status->phase == gnc::ReturnPhase::EntryBurn && !burned) {
            burned = true;
            groups = status->run_groups;
            const auto tr = earth().fixed_to_gcrf(session.epoch());
            ignition_altitude_m = frames::fixed_to_geodetic(transpose(tr.rotation) * session.body(0).state().position_gcrf_m).height_m -
                                  session.launch_site().height_m;
        }
        if (burned && status->phase != gnc::ReturnPhase::EntryBurn) ended = true;
    }
    const auto* guidance = session.return_guidance(0);
    std::printf("[falcon9] entry burn from %.1f km, %.0f m/s burned\n", ignition_altitude_m / 1e3, guidance->entry_delta_v_mps());
    REQUIRE(burned);
    REQUIRE(ended);
    CHECK(groups == std::vector<std::string>{"first_stage/center"});
    CHECK(ignition_altitude_m < 56.6e3);
    CHECK(ignition_altitude_m > 55.0e3);
    CHECK(guidance->entry_delta_v_mps() >= 125.0);
    CHECK(guidance->entry_delta_v_mps() < 130.0);
    CHECK(guidance->phase() == gnc::ReturnPhase::Coast);
}

namespace {

// End of a landing on legs: first foot contact (CG offset from the zone centre, sink, lateral speed, site frame), the
// leg judgement and the session's outcome.
struct LegRun {
    bool contact = false;
    double contact_offset_m = 0.0;
    double contact_sink_mps = 0.0;
    double contact_lateral_mps = 0.0;
    double legs_at_contact = 0.0;  // deployment fraction
    double offset_m = 0.0;         // where it ended
    double propellant_kg = 0.0;    // left at the end
    sim::LegTouchdown touchdown = sim::LegTouchdown::None;
    sim::LandingOutcome outcome = sim::LandingOutcome::Flying;
};

LegRun fly_to_touchdown(sim::FlightSession& session, time::Epoch t0, double max_s) {
    LegRun run;
    const auto& stage = session.body(0);
    const auto zone = session.launch_site();
    auto enu = [&](math::Vec3* velocity) {
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const auto& s = stage.state();
        if (velocity) {
            const Vec3 v_fixed = transpose(tr.rotation) * (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
            const Vec3 origin = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, zone.height_m});
            *velocity = sim::site_enu_m(zone, origin + v_fixed);
        }
        return sim::site_enu_m(zone, transpose(tr.rotation) * s.position_gcrf_m);
    };
    for (double t = sim::FlightSession::tick_s; t <= max_s; t += sim::FlightSession::tick_s) {
        session.advance_to(t0 + t, 10);
        if (!run.contact && session.leg_landing().in_contact(stage)) {
            Vec3 v;
            const Vec3 p = enu(&v);
            run.contact = true;
            run.contact_offset_m = std::hypot(p.x, p.y);
            run.contact_sink_mps = -v.z;
            run.contact_lateral_mps = std::hypot(v.x, v.y);
            run.legs_at_contact = stage.vehicle().leg_deployment();
        }
        if (session.leg_touchdown(0) != sim::LegTouchdown::None || stage.impacted()) break;
    }
    const Vec3 p = enu(nullptr);
    run.offset_m = std::hypot(p.x, p.y);
    run.propellant_kg = stage.vehicle().propellant_kg();
    run.touchdown = session.leg_touchdown(0);
    run.outcome = session.landing_outcome(0);
    return run;
}

constexpr double ignition_sink_mps = 300.0;      // [D] CRS-35: ~300 m/s at landing-burn ignition (~3.2 km)
constexpr double ignition_propellant_kg = 11300.0;  // [D] gauge 2.8-2.9 % at ignition

}  // namespace

TEST_CASE("Falcon 9 lands on LZ-40 from the landing-burn ignition state", "[gnc][falcon9][rtls]") {
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(lz40());
    session.set_landing_zone(lz40());
    const auto parameters = falcon9_parameters();
    const auto cfg = test::booster_config("falcon9_block5", ignition_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, ignition_sink_mps, parameters);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, {15.0, -10.0, h},
                                                          {2.0, -1.0, -ignition_sink_mps}),
                                    t0, parameters, gnc::ReturnPhase::LandingBurn);
    const auto run = fly_to_touchdown(session, t0, 60.0);
    std::printf("[falcon9] ignition at %.0f m: %s / %s, contact offset %.2f m, sink %.2f m/s, lateral %.2f m/s, legs %.2f, "
                "ended %.2f m off, %.0f kg left\n", h, sim::to_string(run.touchdown), sim::to_string(run.outcome),
                run.contact_offset_m, run.contact_sink_mps, run.contact_lateral_mps, run.legs_at_contact, run.offset_m,
                run.propellant_kg);
    CHECK(h > 2500.0);
    CHECK(h < 4000.0);
    REQUIRE(run.contact);
    CHECK(run.legs_at_contact == 1.0);
    CHECK(run.contact_offset_m < 1.0);
    CHECK(run.contact_sink_mps < 2.0);
    CHECK(run.touchdown == sim::LegTouchdown::Landed);
    CHECK(run.outcome == sim::LandingOutcome::Landed);
}

namespace {

struct RtlsDispersion {
    Vec3 offset_m;               // lateral, site frame
    Vec3 velocity_mps;           // added to the nominal fall
    double mass_factor = 1.0;    // propellant at ignition
    math::Quat tilt;             // attitude error
    double aero_factor = 1.0;    // truth hull aerodynamics vs the onboard models
};

// Fixed seeds; spec c3: +-10 m, +-3 m/s per axis, +-5 % propellant, +-3 deg about a random horizontal axis, truth
// aerodynamics +-10 %.
RtlsDispersion rtls_dispersion(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    RtlsDispersion d;
    d.offset_m = {10.0 * unit(rng), 10.0 * unit(rng), 0.0};
    d.velocity_mps = {3.0 * unit(rng), 3.0 * unit(rng), 3.0 * unit(rng)};
    d.mass_factor = 1.0 + 0.05 * unit(rng);
    const double azimuth = constants::pi * unit(rng);
    d.tilt = math::Quat::from_axis_angle({std::cos(azimuth), std::sin(azimuth), 0.0}, 3.0 * constants::deg_to_rad * unit(rng));
    d.aero_factor = 1.0 + 0.1 * unit(rng);
    return d;
}

LegRun fly_rtls_dispersed(const RtlsDispersion& d) {
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(lz40());
    session.set_landing_zone(lz40());
    const auto parameters = falcon9_parameters();
    const auto nominal = test::booster_config("falcon9_block5", ignition_propellant_kg);
    const double h = test::ignition_height_m(session, nominal, earth(), t0, ignition_sink_mps, parameters);
    const auto cfg = test::booster_config("falcon9_block5", ignition_propellant_kg * d.mass_factor);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, d.offset_m + Vec3{0.0, 0.0, h},
                                                          d.velocity_mps + Vec3{0.0, 0.0, -ignition_sink_mps}, d.tilt),
                                    t0, parameters, gnc::ReturnPhase::LandingBurn);
    session.set_aero_dispersion(0, d.aero_factor);
    return fly_to_touchdown(session, t0, 60.0);
}

}  // namespace

TEST_CASE("Falcon 9 RTLS landing campaign: dispersions at ignition", "[gnc][falcon9][rtls][validation]") {
    std::vector<RtlsDispersion> cases{RtlsDispersion{}};
    for (unsigned seed = 1; seed <= 20; ++seed) cases.push_back(rtls_dispersion(seed));
    int landed = 0;
    double worst_offset = 0.0, worst_sink = 0.0, least_propellant = 1e9;
    for (size_t i = 0; i < cases.size(); ++i) {
        const auto run = fly_rtls_dispersed(cases[i]);
        CAPTURE(i);
        std::printf("case %2zu: %s, contact %.2f m, sink %.2f m/s, lateral %.2f m/s, ended %.2f m off, %.0f kg left "
                    "(aero x%.2f)\n", i, sim::to_string(run.outcome), run.contact_offset_m, run.contact_sink_mps,
                    run.contact_lateral_mps, run.offset_m, run.propellant_kg, cases[i].aero_factor);
        std::fflush(stdout);
        CHECK(run.outcome == sim::LandingOutcome::Landed);
        CHECK(run.offset_m < 5.0);  // spec success: < 5 m from the centre
        if (run.outcome == sim::LandingOutcome::Landed) ++landed;
        worst_offset = std::max(worst_offset, run.contact_offset_m);
        worst_sink = std::max(worst_sink, run.contact_sink_mps);
        least_propellant = std::min(least_propellant, run.propellant_kg);
    }
    std::printf("[validation] Falcon 9 RTLS: landed %d of %zu, contact offset <= %.2f m, sink <= %.2f m/s, >= %.0f kg left\n",
                landed, cases.size(), worst_offset, worst_sink, least_propellant);
}
