// Falcon 9 landing on the drone ship (milestone c4): the deck as a landing zone, the drone-ship return (no boostback,
// three-engine entry burn, one-engine landing burn) and the Starlink-class mission from SLC-40.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <random>
#include <cstdio>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

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

// The drone ship ~600 km north-east of the Cape, long axis north (scenarios).
sim::LaunchSite ship() {
    return sim::drone_ship_site(config::load_drone_ship(STARSIM_CONFIG_DIR, "asds_atlantic"), 32.0 * deg, -76.0 * deg,
                                -35.0, 0.0);
}

// Falcon 9 first stage (dry + 1 t), legs out, its feet `feet_height_m` above the deck at `east_m` from the deck centre,
// sinking at `sink_mps`.
void drop_on_ship(sim::FlightSession& session, double east_m, double feet_height_m, double sink_mps) {
    const auto cfg = test::booster_config("falcon9_block5", 1000.0);
    const auto t0 = start();
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double foot = cfg.stages.front().landing_legs.foot_station_m;
    session.place_body(cfg, test::state_over_site(session.launch_site(), earth(), t0, {east_m, 0.0, feet_height_m + cg - foot},
                                                  {0.0, 0.0, -sink_mps}),
                       t0);
    session.set_leg_deployment(0, 1.0);
}

sim::LegTouchdown run_drop(sim::FlightSession& session) {
    const auto t0 = session.epoch();
    while (session.leg_touchdown(0) == sim::LegTouchdown::None && session.epoch() - t0 < 20.0) {
        session.advance_to(session.epoch() + 0.1);
    }
    return session.leg_touchdown(0);
}

}  // namespace

TEST_CASE("The drone ship's deck loads and becomes a landing zone", "[config][falcon9][asds]") {
    const auto c = config::load_drone_ship(STARSIM_CONFIG_DIR, "asds_atlantic");
    CHECK(c.name == "Drohnenschiff");
    CHECK(c.deck_length_m == 91.0);
    CHECK(c.deck_width_m == 52.0);
    CHECK(c.freeboard_m == 3.0);
    const auto site = ship();
    REQUIRE(site.deck);
    CHECK(site.height_m == -32.0);  // sea -35 m + 3 m freeboard
    CHECK(site.deck->half_length_m == 45.5);
    CHECK(site.deck->half_width_m == 26.0);
    REQUIRE(site.sea);
    CHECK(std::abs(site.sea->direction_rad - 90.0 * deg) < 1e-12);  // beside the long edge (east)
    CHECK(site.sea->shoreline_distance_m == 26.0);
    CHECK_THROWS(config::load_drone_ship(STARSIM_CONFIG_DIR, "nowhere"));
}

TEST_CASE("A Falcon 9 stage returning to a drone ship aims at its deck", "[gnc][falcon9][asds]") {
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const vehicle::Vehicle stage(test::booster_config("falcon9_block5", 12100.0), STARSIM_CONFIG_DIR);
    const auto target = session.return_site(stage);
    CHECK(target.height_m == -32.0);
    REQUIRE(target.pad);
    CHECK(target.pad->radius_m == 26.0);
    REQUIRE(target.sea);
    CHECK(target.sea->shoreline_distance_m == 26.0);
}

TEST_CASE("A stage set down on the deck lands, one beside it splashes down", "[legs][falcon9][asds]") {
    {
        sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
        drop_on_ship(session, 0.0, 0.5, 2.0);
        CHECK(run_drop(session) == sim::LegTouchdown::Landed);
        CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
    }
    {
        // Feet (9 m from the axis) 10 m beyond the western long edge (26 m from the centre line): water on every side of
        // the ship, not only beside the edge the abort aims at.
        sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
        drop_on_ship(session, -45.0, 0.5, 2.0);
        CHECK(run_drop(session) == sim::LegTouchdown::Splashdown);
        CHECK(session.landing_outcome(0) == sim::LandingOutcome::Splashdown);
    }
}

namespace {

gnc::BoosterReturnParameters asds_parameters() {
    const auto stage = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5").stages.front();
    return gnc::booster_return_parameters(stage.id, stage.return_profile_for(true));
}

constexpr double asds_ignition_sink_mps = 270.0;       // [D] Starlink 15-25: ~270 m/s at ~3.0 km (T+476 s)
constexpr double asds_ignition_propellant_kg = 12100.0;  // [D] gauge 2.8-2.9 % of 425 t at ignition

}  // namespace

TEST_CASE("A Falcon 9 drone-ship MECO plan has no boostback", "[gnc][falcon9][asds]") {
    // The stack at ~60 km, ~2.2 km/s, climbing north-east of the Cape: the plan is the run-down, the entry burn and
    // the landing propellant, wherever the ship waits.
    const auto t0 = start();
    const vehicle::Vehicle stack(config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5"), STARSIM_CONFIG_DIR);
    const auto tr = earth().fixed_to_gcrf(t0);
    const Vec3 r_fixed = frames::geodetic_to_fixed({29.5 * deg, -79.5 * deg, 60e3});
    const Vec3 up = math::normalized(r_fixed);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, up));
    const math::StateVector state{tr.position(r_fixed), tr.velocity(r_fixed, east * 1800.0 + up * 900.0)};
    const auto p = asds_parameters();
    const auto near = ship();
    const gnc::LandingSite a{near.latitude_rad, near.longitude_rad, near.height_m, {}, {}, {}};
    const gnc::LandingSite b{near.latitude_rad + 2.0 * deg, near.longitude_rad + 3.0 * deg, near.height_m, {}, {}, {}};
    const double at_a = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, a, p, 12100.0);
    const double at_b = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, b, p, 12100.0);
    // Entry burn by the rocket equation: (25.6 + 12.1) t x exp(880 / (311 x 9.807)) - 25.6 t = 24.7 t; nine engines'
    // run-down (0.15 s each at full flow) ~0.4 t.
    std::printf("[falcon9] drone-ship MECO propellant %.1f t\n", at_a / 1e3);
    CHECK(at_a == at_b);
    CHECK(at_a > 24.0e3);
    CHECK(at_a < 26.0e3);
}

namespace {

// A stage falling engines first towards the ship (75 km up, 1.9 km/s horizontally north, 1.0 km/s down) from
// `start_enu_m` in the ship's frame, coasting into its entry burn with the burn's lean limited to `max_lean_rad`.
struct EntryRun {
    bool burned = false;
    bool finite = true;  // commanded axis finite on every tick
    std::vector<std::string> groups;
    double ignition_altitude_m = 0.0;
    double delta_v_mps = 0.0;
    double used_kg = 0.0;
    Vec3 predicted_enu_m;          // just before the burn: impact with the burn ahead (finite), ship frame
    Vec3 predicted_impulse_enu_m;  // the same with the burn as an impulse at its altitude
    Vec3 after_enu_m;              // predicted 5 s after the burn (drag only)
};

EntryRun fly_entry_burn(const Vec3& start_enu_m, double max_lean_rad) {
    EntryRun run;
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const auto cfg = test::booster_config("falcon9_block5", 24100.0);  // [D] gauge 6.1 % at the entry burn
    const Vec3 velocity{0.0, 1900.0, -1000.0};
    const Vec3 axis = math::normalized(-velocity);
    const auto tilt = math::Quat::from_axis_angle(math::normalized(cross(Vec3{0.0, 0.0, 1.0}, axis)),
                                                  std::acos(std::clamp(axis.z, -1.0, 1.0)));
    auto parameters = asds_parameters();
    parameters.entry_max_lean_rad = max_lean_rad;
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, start_enu_m, velocity, tilt),
                                    t0, parameters, gnc::ReturnPhase::Coast);
    const double ground = session.landing_zone()->height_m;
    auto in_ship_frame = [&](const gnc::BoosterReturnGuidance::Impact& impact) {
        REQUIRE(impact.valid);
        return sim::site_enu_m(session.launch_site(), impact.position_itrf_m);
    };
    double propellant_before_kg = 0.0, end_s = 0.0;
    bool ended = false;
    for (double t = 0.0; t < 60.0 && !ended;) {
        if (!run.burned) {
            const auto& body = session.body(0);
            auto burn = gnc::BoosterReturnGuidance::entry_burn_ahead(body.vehicle(), parameters);
            run.predicted_enu_m = in_ship_frame(
                gnc::BoosterReturnGuidance::predict_impact(body, session.environment(), session.epoch(), ground, nullptr, &burn));
            burn.thrust_n = 0.0;
            run.predicted_impulse_enu_m = in_ship_frame(
                gnc::BoosterReturnGuidance::predict_impact(body, session.environment(), session.epoch(), ground, nullptr, &burn));
        }
        t += sim::FlightSession::tick_s;
        session.advance_to(t0 + t, 10);
        const auto* status = session.return_status(0);
        REQUIRE(status != nullptr);
        run.finite = run.finite && std::isfinite(status->axis_gcrf.x) && std::isfinite(status->axis_gcrf.y) &&
                     std::isfinite(status->axis_gcrf.z);
        if (status->phase == gnc::ReturnPhase::EntryBurn && !run.burned) {
            run.burned = true;
            run.groups = status->run_groups;
            propellant_before_kg = session.body(0).vehicle().propellant_kg();
            const auto tr = earth().fixed_to_gcrf(session.epoch());
            run.ignition_altitude_m =
                frames::fixed_to_geodetic(transpose(tr.rotation) * session.body(0).state().position_gcrf_m).height_m - ground;
        }
        if (run.burned && status->phase != gnc::ReturnPhase::EntryBurn) {
            ended = true;
            end_s = t;
        }
    }
    REQUIRE(ended);
    run.delta_v_mps = session.return_guidance(0)->entry_delta_v_mps();
    run.used_kg = propellant_before_kg - session.body(0).vehicle().propellant_kg();
    session.advance_to(t0 + end_s + 5.0, 2000);
    run.after_enu_m = in_ship_frame(
        gnc::BoosterReturnGuidance::predict_impact(session.body(0), session.environment(), session.epoch(), ground));
    return run;
}

double horizontal_m(const Vec3& v) { return std::hypot(v.x, v.y); }

}  // namespace

TEST_CASE("A falling Falcon 9 stage flies its drone-ship entry burn on three engines", "[gnc][falcon9][asds]") {
    const auto run = fly_entry_burn({0.0, -40e3, 75e3}, 0.0);  // straight retrograde: the prediction's own test
    const Vec3 error = run.after_enu_m - run.predicted_enu_m;
    const Vec3 impulse_error = run.after_enu_m - run.predicted_impulse_enu_m;
    std::printf("[falcon9] drone-ship entry burn from %.1f km, %.0f m/s burned, %.1f t used; impact predicted before the burn "
                "%.0f m off the flown one (as an impulse: %.0f m)\n", run.ignition_altitude_m / 1e3, run.delta_v_mps,
                run.used_kg / 1e3, horizontal_m(error), horizontal_m(impulse_error));
    REQUIRE(run.burned);
    CHECK(run.groups == std::vector<std::string>{"first_stage/center", "first_stage/pair"});
    CHECK(run.finite);  // the commanded axis, also on the burn's first ticks
    CHECK(run.ignition_altitude_m < 66.0e3);
    CHECK(run.ignition_altitude_m > 64.0e3);
    CHECK(run.delta_v_mps >= 880.0);
    CHECK(run.delta_v_mps < 900.0);
    // [D] gauge 6.1 -> 2.85 % of 425 t = 13.8 t; the rocket equation on 880 m/s gives 12.6 t from 24.7 t.
    CHECK(std::abs(run.used_kg - 13200.0) < 1000.0);
    CHECK(horizontal_m(error) < 300.0);
}

TEST_CASE("The drone-ship entry burn steers the impact onto the deck", "[gnc][falcon9][asds]") {
    // Starting 3 km east of a path that leads to the ship: the burn's lean removes most of the miss.
    const auto probe = fly_entry_burn({0.0, -40e3, 75e3}, 0.0);
    const Vec3 on_path{-probe.after_enu_m.x, -40e3 - probe.after_enu_m.y, 75e3};
    const auto run = fly_entry_burn(on_path + Vec3{3000.0, 0.0, 0.0}, asds_parameters().entry_max_lean_rad);
    std::printf("[falcon9] drone-ship entry burn from 3 km off the path: %.0f m off after the burn\n",
                horizontal_m(run.after_enu_m));
    CHECK(run.finite);
    CHECK(horizontal_m(run.after_enu_m) < 500.0);
}

TEST_CASE("Falcon 9 lands on the drone ship from the landing-burn ignition state", "[gnc][falcon9][asds]") {
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const auto parameters = asds_parameters();
    const auto cfg = test::booster_config("falcon9_block5", asds_ignition_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, asds_ignition_sink_mps, parameters);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, {15.0, -10.0, h},
                                                          {2.0, -1.0, -asds_ignition_sink_mps}),
                                    t0, parameters, gnc::ReturnPhase::LandingBurn);
    const auto run = test::fly_to_touchdown(session, earth(), t0, 60.0);
    std::printf("[falcon9] drone ship: ignition at %.0f m: %s / %s, contact offset %.2f m, sink %.2f m/s, lateral %.2f "
                "m/s, legs %.2f, ended %.2f m off, %.0f kg left\n", h, sim::to_string(run.touchdown),
                sim::to_string(run.outcome), run.contact_offset_m, run.contact_sink_mps, run.contact_lateral_mps,
                run.legs_at_contact, run.offset_m, run.propellant_kg);
    CHECK(h > 2500.0);  // [D] Starlink 15-25: ~3.0 km
    CHECK(h < 3500.0);
    REQUIRE(run.contact);
    CHECK(run.legs_at_contact == 1.0);
    CHECK(run.contact_offset_m < 1.0);
    CHECK(run.contact_sink_mps < 2.0);
    CHECK(run.outcome == sim::LandingOutcome::Landed);
}

TEST_CASE("A Falcon 9 stage that cannot reach the drone ship ditches beside it", "[gnc][falcon9][asds]") {
    // Lit 900 m off the deck towards the abort side (across the deck), falling straight: one engine diverts a few hundred
    // metres, the deck is out of reach; the abort plans for the water beside the ship (spec c4).
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const auto parameters = asds_parameters();
    const auto cfg = test::booster_config("falcon9_block5", asds_ignition_propellant_kg);
    const double h = test::ignition_height_m(session, cfg, earth(), t0, asds_ignition_sink_mps, parameters);
    session.place_returning_booster(cfg,
                                    test::state_over_site(session.launch_site(), earth(), t0, {900.0, 0.0, h},
                                                          {0.0, 0.0, -asds_ignition_sink_mps}),
                                    t0, parameters, gnc::ReturnPhase::LandingBurn);
    const auto run = test::fly_to_touchdown(session, earth(), t0, 60.0);
    const auto* guidance = session.return_guidance(0);
    std::printf("[falcon9] drone ship abort: %s / %s, ended %.0f m from the deck centre, abort %d\n",
                sim::to_string(run.touchdown), sim::to_string(run.outcome), run.offset_m,
                guidance && guidance->scvx() ? guidance->scvx()->aborting() : -1);
    REQUIRE(guidance->scvx() != nullptr);
    CHECK(guidance->scvx()->aborting());
    CHECK(run.outcome == sim::LandingOutcome::Splashdown);
}

TEST_CASE("The Falcon 9 drone-ship mission loads", "[config][falcon9][asds]") {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    CHECK(m.vehicle_id == "falcon9_block5");
    CHECK(m.site_id == "ccsfs_slc40");
    CHECK(m.landing_zone_id.empty());
    REQUIRE(m.drone_ship);
    CHECK(m.drone_ship->id == "asds_atlantic");
    CHECK(m.payload_kg > 15000.0);
    CHECK(m.payload_kg < 17500.0);
    CHECK(m.fairing);
    CHECK(std::abs(m.inclination_rad / deg - 53.0) < 1e-9);
    // The ship waits ~600 km down the ground track (spec), north-east of the Cape.
    const auto pad = config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id);
    const double distance_m = constants::wgs84_semi_major_axis_m * std::acos(std::sin(pad.latitude_rad) * std::sin(m.drone_ship->latitude_rad) +
                                                               std::cos(pad.latitude_rad) * std::cos(m.drone_ship->latitude_rad) *
                                                                   std::cos(m.drone_ship->longitude_rad - pad.longitude_rad));
    std::printf("[falcon9] drone ship %.0f km from the pad\n", distance_m / 1e3);
    CHECK(distance_m > 450e3);
    CHECK(distance_m < 750e3);
}

TEST_CASE("Starting the drone-ship mission puts the ship out as the landing zone", "[sim][falcon9][asds]") {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, start());
    REQUIRE(session.landing_zone());
    REQUIRE(session.landing_zone()->deck);
    CHECK(session.landing_zone()->latitude_rad == m.drone_ship->latitude_rad);
    CHECK(session.landing_zone()->height_m == m.drone_ship->sea_height_m + 3.0);
    CHECK(session.autopilot().state != sim::AutopilotState::Off);
    CHECK(session.returns_after_staging(session.body(0).vehicle()));
    CHECK(session.return_parameters(session.body(0).vehicle().stages().front().config).boostback == false);
    CHECK(session.body(0).vehicle().stages().back().config.payload_mass_kg == m.payload_kg);
}

TEST_CASE("A plain launch after a mission does not inherit its autopilot", "[sim][falcon9][asds]") {
    // Review finding: start_mission engages the autopilot; a later free launch on the same session must start idle.
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    const auto t0 = start();
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    session.launch_from_pad("starship_block3",
                            sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")),
                            0.0, t0 + 10.0);
    CHECK(session.autopilot().state == sim::AutopilotState::Off);
    CHECK(session.ascent_guidance() == nullptr);
    session.advance_to(t0 + 20.0);
    CHECK(session.autopilot().state == sim::AutopilotState::Off);
    CHECK(session.body(0).clamped());
    CHECK(session.telemetry(0).engines_running == 0);
}

TEST_CASE("Falcon 9 drone-ship mission: nominal impact point of the first stage", "[.][falcon9][asds]") {
    // Diagnostic: where the stage would come down from MECO (drag, the entry burn as an impulse, no steering) - the
    // drone ship's position in config/missions/falcon9_asds.toml [D].
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    for (double t = 0.0; t < 300.0 && session.body_count() < 2; t += 0.05) session.advance_to(t0 + t, 100);
    REQUIRE(session.body_count() == 2);
    const double meco_s = session.mission_time_s();
    {
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const auto& b = session.body(0).state();
        const Vec3 v = b.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), b.position_gcrf_m);
        const double gamma = std::asin(dot(math::normalized(v), math::normalized(b.position_gcrf_m)));
        const double h = frames::fixed_to_geodetic(transpose(tr.rotation) * b.position_gcrf_m).height_m;
        std::printf("[falcon9] MECO T+%.1f s: %.1f km, %.0f km/h, flight path %.1f deg (Starlink 15-25: T+149 s, 60 km, "
                    "~8000 km/h)\n", meco_s, h / 1e3, norm(v) * 3.6, gamma / deg);
    }
    // After the engines' tail-off and the separation (~20 m/s, ~10 km at the impact); the coast's RCS trim moves the
    // impact point only ~0.1 km in these seconds.
    session.advance_to(session.epoch() + 10.0, 4000);
    const auto& stage = session.body(0);
    const auto p = session.return_parameters(stage.vehicle().stages().front().config);
    const auto burn = gnc::BoosterReturnGuidance::entry_burn_ahead(stage.vehicle(), p);
    const double ground = session.landing_zone()->height_m;
    const auto impact = gnc::BoosterReturnGuidance::predict_impact(stage, session.environment(), session.epoch(), ground,
                                                                   nullptr, &burn);
    REQUIRE(impact.valid);
    const auto geo = frames::fixed_to_geodetic(impact.position_itrf_m);
    const auto tr = earth().fixed_to_gcrf(session.epoch());
    const auto meco = frames::fixed_to_geodetic(transpose(tr.rotation) * stage.state().position_gcrf_m);
    std::printf("[falcon9] MECO T+%.1f s; at T+%.1f s %.1f km with %.1f t; impact after %.0f s at latitude %.4f deg, "
                "longitude %.4f deg\n", meco_s, session.mission_time_s(), meco.height_m / 1e3, stage.vehicle().propellant_kg() / 1e3, impact.time_s,
                geo.latitude_rad / deg, geo.longitude_rad / deg);
}

TEST_CASE("Falcon 9 drone-ship mission: liftoff to orbit and the first stage landed on the deck", "[gnc][falcon9][asds][validation]") {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    const auto ship_site = *session.landing_zone();
    auto height_speed = [&](const vehicle::FlightBody& b) {
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const auto& s = b.state();
        const double h = frames::fixed_to_geodetic(transpose(tr.rotation) * s.position_gcrf_m).height_m - ship_site.height_m;
        return std::pair{h, norm(s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m))};
    };
    double meco_s = -1.0, meco_h = 0.0, meco_v = 0.0, meco_kg = 0.0, entry_s = -1.0, entry_h = 0.0, entry_v = 0.0,
           entry_kg = 0.0, entry_pointing_deg = 0.0, entry_end_s = -1.0, entry_end_v = 0.0, landing_s = -1.0,
           landing_h = 0.0, landing_v = 0.0, landing_kg = 0.0;
    gnc::ReturnPhase last_phase = gnc::ReturnPhase::Coast;
    for (double t = 0.0; t < 900.0; t += 0.05) {
        session.advance_to(t0 + t, 100);
        if (session.body_count() < 2) continue;
        const auto& stage = session.body(0);
        if (meco_s < 0.0) {
            meco_s = session.mission_time_s();
            std::tie(meco_h, meco_v) = height_speed(stage);
            meco_kg = stage.vehicle().propellant_kg();
        }
        const auto* status = session.return_status(0);
        if (!status) break;
        if (std::getenv("STARSIM_VERBOSE") && std::fmod(session.mission_time_s(), 5.0) < 0.05) {
            const auto [h, v] = height_speed(stage);
            const Vec3 axis_now = stage.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
            std::printf("    T+%.0f s %s %.1f km %.0f m/s RCS %.0f kg miss %.0f m, axis error %.1f deg, rate %.3f rad/s\n",
                        session.mission_time_s(), gnc::to_string(status->phase), h / 1e3, v,
                        stage.vehicle().rcs_propellant_kg(), status->impact_miss_m,
                        std::acos(std::clamp(dot(axis_now, status->axis_gcrf), -1.0, 1.0)) / deg,
                        norm(stage.state().angular_velocity_body_radps));        }
        if (status->phase != last_phase) {
            const auto [h, v] = height_speed(stage);
            if (std::getenv("STARSIM_VERBOSE")) {
                std::printf("  T+%.1f s %s: %.1f km, %.0f m/s, %.1f t, RCS %.0f kg, miss %.0f m\n", session.mission_time_s(),
                            gnc::to_string(status->phase), h / 1e3, v, stage.vehicle().propellant_kg() / 1e3,
                            stage.vehicle().rcs_propellant_kg(), status->impact_miss_m);
            }
            if (status->phase == gnc::ReturnPhase::EntryBurn && entry_s < 0.0) {
                entry_s = session.mission_time_s();
                entry_h = h;
                entry_v = v;
                entry_kg = stage.vehicle().propellant_kg();
                const Vec3 axis = stage.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
                entry_pointing_deg = std::acos(std::clamp(dot(axis, status->axis_gcrf), -1.0, 1.0)) / deg;
            }
            if (last_phase == gnc::ReturnPhase::EntryBurn && entry_end_s < 0.0) {
                entry_end_s = session.mission_time_s();
                entry_end_v = v;
            }
            if (status->phase == gnc::ReturnPhase::LandingBurn && landing_s < 0.0) {
                landing_s = session.mission_time_s();
                std::tie(landing_h, landing_v) = std::pair{h, v};
                landing_kg = stage.vehicle().propellant_kg();
            }
            last_phase = status->phase;
        }
        if (session.leg_touchdown(0) != sim::LegTouchdown::None || stage.impacted()) break;
    }
    const auto& stage = session.body(0);
    const Vec3 p = sim::site_enu_m(ship_site, transpose(earth().fixed_to_gcrf(session.epoch()).rotation) * stage.state().position_gcrf_m);
    const auto outcome = session.landing_outcome(0);
    const double touchdown_s = session.mission_time_s();
    const double left_kg = stage.vehicle().propellant_kg();
    // The second stage flies on to its orbit.
    while (session.autopilot().state == sim::AutopilotState::UpperStage && session.mission_time_s() < 1200.0) {
        session.advance_to(session.epoch() + 1.0, 400);
    }
    const auto& upper = session.body(session.controlled_index());
    const auto orbit = orbit::elements_from_state({upper.state().position_gcrf_m, upper.state().velocity_gcrf_mps},
                                                session.gm_earth_m3ps2());
    const double periapsis_km = (orbit.semi_major_axis_m * (1.0 - orbit.eccentricity) - constants::wgs84_semi_major_axis_m) / 1e3;
    const double apoapsis_km = (orbit.semi_major_axis_m * (1.0 + orbit.eccentricity) - constants::wgs84_semi_major_axis_m) / 1e3;
    std::printf("[validation] Falcon 9 ASDS: %.1f t to %.0f x %.0f km, i = %.2f deg (%s at T+%.0f s, %.1f t left in stage 2)\n", m.payload_kg / 1e3,
                periapsis_km, apoapsis_km, orbit.inclination_rad / deg, sim::to_string(session.autopilot().state),
                session.mission_time_s(), upper.vehicle().propellant_kg() / 1e3);
    CHECK(session.autopilot().state == sim::AutopilotState::Orbit);
    CHECK(std::abs(periapsis_km - 200.0) < 5.0);
    CHECK(std::abs(apoapsis_km - 280.0) < 5.0);
    std::printf("[validation] Falcon 9 ASDS: MECO T+%.0f s at %.1f km, %.0f km/h, %.1f t (Starlink 15-25: T+149 s, 60 km, "
                "~8000 km/h, gauge ~47 t); entry burn T+%.0f s at %.1f km, %.0f km/h, %.1f t, %.1f deg off the burn axis "
                "(T+364 s, 67.4 km, ~7900 km/h, 25.9 t) to T+%.0f s, %.0f km/h (T+386 s, ~5250 km/h); landing burn T+%.0f "
                "s at %.2f km, %.0f m/s, %.1f t (T+476 s, ~3.0 km, ~270 m/s, 11.9 t); %s at T+%.0f s (T+499 s), %.2f m from "
                "the deck centre, %.1f t left\n",
                meco_s, meco_h / 1e3, meco_v * 3.6, meco_kg / 1e3, entry_s, entry_h / 1e3, entry_v * 3.6, entry_kg / 1e3,
                entry_pointing_deg, entry_end_s, entry_end_v * 3.6, landing_s, landing_h / 1e3, landing_v,
                landing_kg / 1e3, sim::to_string(outcome), touchdown_s, std::hypot(p.x, p.y), left_kg / 1e3);
    CHECK(meco_s > 0.0);
    CHECK(entry_s > 0.0);
    CHECK(entry_pointing_deg < 15.0);  // turned engines first on the RCS before the burn
    CHECK(landing_s > 0.0);
    CHECK(outcome == sim::LandingOutcome::Landed);
    CHECK(std::hypot(p.x, p.y) < 5.0);
    // The landing-burn ignition state lies within the telemetry's spread (spec): ~3.0 km, ~270 m/s, 11.9 t.
    CHECK(std::abs(landing_kg - 12100.0) < 1000.0);
    CHECK(landing_h > 2500.0);
    CHECK(landing_h < 4000.0);
    CHECK(std::abs(landing_v - 270.0) < 40.0);
    // Timeline [D] of Starlink 15-25: entry burn T+364 s, landing burn T+476 s.
    CHECK(std::abs(entry_s - 364.0) < 25.0);
    CHECK(std::abs(landing_s - 476.0) < 25.0);
}

namespace {

// The drone ship where the mission puts it.
sim::LaunchSite ship_of_mission() {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    const auto& d = *m.drone_ship;
    return sim::drone_ship_site(config::load_drone_ship(STARSIM_CONFIG_DIR, d.id), d.latitude_rad, d.longitude_rad,
                                d.sea_height_m, d.heading_rad);
}

// The nominal drone-ship mission, flown once: the first stage 10 s after MECO (tail-off and separation done) and at
// its landing-burn ignition. Both campaigns start from these states.
struct NominalReturn {
    vehicle::RigidBodyState after_meco;
    time::Epoch after_meco_epoch;
    double after_meco_propellant_kg = 0.0;
    vehicle::RigidBodyState ignition;
    time::Epoch ignition_epoch;
    double ignition_propellant_kg = 0.0;
};

const NominalReturn& nominal_return() {
    static const NominalReturn nominal = [] {
        NominalReturn n;
        const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
        const auto t0 = start();
        sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
        sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
        double meco_s = -1.0;
        for (double t = 0.0; t < 900.0; t += 0.05) {
            session.advance_to(t0 + t, 100);
            if (session.body_count() < 2) continue;
            if (meco_s < 0.0) meco_s = session.mission_time_s();
            const auto& stage = session.body(0);
            if (n.after_meco_propellant_kg == 0.0 && session.mission_time_s() >= meco_s + 10.0) {
                n.after_meco = stage.state();
                n.after_meco_epoch = session.epoch();
                n.after_meco_propellant_kg = stage.vehicle().propellant_kg();
            }
            const auto* status = session.return_status(0);
            if (status && status->phase == gnc::ReturnPhase::LandingBurn) {
                n.ignition = stage.state();
                n.ignition_epoch = session.epoch();
                n.ignition_propellant_kg = stage.vehicle().propellant_kg();
                break;
            }
        }
        REQUIRE(n.after_meco_propellant_kg > 0.0);
        REQUIRE(n.ignition_propellant_kg > 0.0);
        return n;
    }();
    return nominal;
}

struct AsdsDispersion {
    Vec3 position_m;           // GCRF
    Vec3 velocity_mps;         // GCRF
    double mass_factor = 1.0;  // propellant
    double tilt_rad = 0.0;     // attitude error about a random axis
    Vec3 tilt_axis{1.0, 0.0, 0.0};
    double aero_factor = 1.0;  // truth hull aerodynamics vs the onboard models
};

// Fixed seeds. `position_m`, `velocity_mps`: per axis; propellant +-5 %, truth aerodynamics +-10 %.
AsdsDispersion asds_dispersion(unsigned seed, double position_m, double velocity_mps, double tilt_rad) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    AsdsDispersion d;
    d.position_m = {position_m * unit(rng), position_m * unit(rng), position_m * unit(rng)};
    d.velocity_mps = {velocity_mps * unit(rng), velocity_mps * unit(rng), velocity_mps * unit(rng)};
    d.mass_factor = 1.0 + 0.05 * unit(rng);
    d.tilt_rad = tilt_rad * unit(rng);
    d.tilt_axis = math::normalized(Vec3{unit(rng), unit(rng), unit(rng)});
    d.aero_factor = 1.0 + 0.1 * unit(rng);
    return d;
}

test::LegRun fly_asds_dispersed(const vehicle::RigidBodyState& nominal, time::Epoch epoch, double propellant_kg,
                                gnc::ReturnPhase phase, const AsdsDispersion& d, double max_s) {
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(ship_of_mission());
    session.set_landing_zone(ship_of_mission());
    auto state = nominal;
    state.position_gcrf_m += d.position_m;
    state.velocity_gcrf_mps += d.velocity_mps;
    state.attitude = math::Quat::from_axis_angle(d.tilt_axis, d.tilt_rad) * state.attitude;
    session.place_returning_booster(test::booster_config("falcon9_block5", propellant_kg * d.mass_factor), state, epoch,
                                    asds_parameters(), phase);
    session.set_aero_dispersion(0, d.aero_factor);
    return test::fly_to_touchdown(session, earth(), epoch, max_s);
}

// `allowed_misses`: cases that may miss the spec's success (landed < 5 m from the deck centre) if they still end safely:
// landed elsewhere on the deck, or aborted to the sea. Any other outcome fails.
void report_campaign(const char* name, const std::vector<AsdsDispersion>& cases,
                     const std::function<test::LegRun(const AsdsDispersion&)>& fly, int allowed_misses = 0) {
    int landed = 0, missed = 0;
    double worst_offset = 0.0, worst_sink = 0.0, least_propellant = 1e9;
    for (size_t i = 0; i < cases.size(); ++i) {
        const auto run = fly(cases[i]);
        CAPTURE(i);
        std::printf("%s case %2zu: %s, contact %.2f m, sink %.2f m/s, lateral %.2f m/s, ended %.2f m off, %.0f kg left "
                    "(aero x%.2f)\n", name, i, sim::to_string(run.outcome), run.contact_offset_m, run.contact_sink_mps,
                    run.contact_lateral_mps, run.offset_m, run.propellant_kg, cases[i].aero_factor);
        std::fflush(stdout);
        const bool success = run.outcome == sim::LandingOutcome::Landed && run.offset_m < 5.0;
        const bool safe = run.outcome == sim::LandingOutcome::Landed || run.outcome == sim::LandingOutcome::Splashdown;
        if (!success && safe && missed < allowed_misses) {
            ++missed;
        } else {
            CHECK(run.outcome == sim::LandingOutcome::Landed);
            CHECK(run.offset_m < 5.0);  // spec success: < 5 m from the deck centre
        }
        if (run.outcome == sim::LandingOutcome::Landed) ++landed;
        worst_offset = std::max(worst_offset, run.contact_offset_m);
        worst_sink = std::max(worst_sink, run.contact_sink_mps);
        least_propellant = std::min(least_propellant, run.propellant_kg);
    }
    std::printf("[validation] Falcon 9 ASDS %s: landed %d of %zu, contact offset <= %.2f m, sink <= %.2f m/s, >= %.0f kg "
                "left\n", name, landed, cases.size(), worst_offset, worst_sink, least_propellant);
}

}  // namespace

TEST_CASE("Falcon 9 ASDS landing campaign: dispersions at ignition", "[gnc][falcon9][asds][validation]") {
    // As for RTLS (spec c4): +-10 m, +-3 m/s per axis, +-5 % propellant, +-3 deg, truth aero +-10 %, around the
    // nominal mission's slanted ignition state (~0.8 km short of the deck, ~90 m/s across).
    const auto& n = nominal_return();
    std::vector<AsdsDispersion> cases{AsdsDispersion{}};
    for (unsigned seed = 1; seed <= 20; ++seed) cases.push_back(asds_dispersion(seed, 10.0, 3.0, 3.0 * deg));
    report_campaign("ignition", cases, [&](const AsdsDispersion& d) {
        return fly_asds_dispersed(n.ignition, n.ignition_epoch, n.ignition_propellant_kg, gnc::ReturnPhase::LandingBurn, d,
                                  60.0);
    });
}

namespace {

// Spec c4: the stage 10 s after MECO with +-200 m, +-5 m/s per axis (~2.5 km of impact spread), +-5 % propellant,
// truth aero +-10 %: the coast, the entry burn and the fins bring it onto the deck. Since d1 (individual thrusters) the
// RCS cannot translate the stage in the coast (its pods sit at one station): the entry burn removes the miss by its
// length and lean, which leaves the stage faster or lighter at the landing burn - two cases per half may miss the 5 m
// if it ends safely (on the deck or aborted to the sea; user 2026-10-14: correct over the burn). Two halves (ctest runs them in
// parallel; each case flies ~340 s).
void meco_campaign(const char* name, unsigned first_seed, unsigned last_seed, bool with_nominal) {
    const auto& n = nominal_return();
    std::vector<AsdsDispersion> cases;
    if (with_nominal) cases.push_back(AsdsDispersion{});
    for (unsigned seed = first_seed; seed <= last_seed; ++seed) cases.push_back(asds_dispersion(seed, 200.0, 5.0, 0.0));
    report_campaign(
        name, cases,
        [&](const AsdsDispersion& d) {
            return fly_asds_dispersed(n.after_meco, n.after_meco_epoch, n.after_meco_propellant_kg, gnc::ReturnPhase::Coast,
                                      d, 600.0);
        },
        2);
}

}  // namespace

TEST_CASE("Falcon 9 ASDS landing campaign: dispersed MECO states (1 of 2)", "[gnc][falcon9][asds][validation]") {
    meco_campaign("MECO 1", 101, 110, true);
}

TEST_CASE("Falcon 9 ASDS landing campaign: dispersed MECO states (2 of 2)", "[gnc][falcon9][asds][validation]") {
    meco_campaign("MECO 2", 111, 120, false);
}

TEST_CASE("Falcon 9 ASDS single MECO case", "[.][asds-single]") {
    // Diagnostics: STARSIM_ASDS_SEED=<seed of the MECO campaign> (0: nominal); prints the return's phases.
    const auto& n = nominal_return();
    const char* seed_text = std::getenv("STARSIM_ASDS_SEED");
    const unsigned seed = seed_text ? static_cast<unsigned>(std::atoi(seed_text)) : 0u;
    const auto d = seed ? asds_dispersion(seed, 200.0, 5.0, 0.0) : AsdsDispersion{};
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    const auto site = ship_of_mission();
    session.set_launch_site(site);
    session.set_landing_zone(site);
    auto state = n.after_meco;
    state.position_gcrf_m += d.position_m;
    state.velocity_gcrf_mps += d.velocity_mps;
    session.place_returning_booster(test::booster_config("falcon9_block5", n.after_meco_propellant_kg * d.mass_factor), state,
                                    n.after_meco_epoch, asds_parameters(), gnc::ReturnPhase::Coast);
    session.set_aero_dispersion(0, d.aero_factor);
    gnc::ReturnPhase last = gnc::ReturnPhase::Coast;
    for (double t = 0.0; t < 600.0; t += 0.05) {
        session.advance_to(n.after_meco_epoch + t, 100);
        const auto* status = session.return_status(0);
        const auto& b = session.body(0);
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const Vec3 p = sim::site_enu_m(site, transpose(tr.rotation) * b.state().position_gcrf_m);
        const Vec3 v_fixed = transpose(tr.rotation) *
                             (b.state().velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), b.state().position_gcrf_m));
        const Vec3 origin = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
        const Vec3 v = sim::site_enu_m(site, origin + v_fixed);
        if (status->phase != last || std::fmod(t, 10.0) < 0.05) {
            const Vec3 axis = b.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
            std::printf("  t %.1f %s: enu %.0f,%.0f,%.0f v %.1f,%.1f,%.1f miss %.0f m, %.1f t, RCS %.0f kg, axis error %.1f deg\n",
                        t, gnc::to_string(status->phase), p.x, p.y, p.z, v.x, v.y, v.z, status->impact_miss_m,
                        b.vehicle().propellant_kg() / 1e3, b.vehicle().rcs_propellant_kg(),
                        std::acos(std::clamp(dot(axis, status->axis_gcrf), -1.0, 1.0)) / deg);
            last = status->phase;
        }
        if (session.leg_touchdown(0) != sim::LegTouchdown::None || b.impacted()) break;
    }
    std::printf("[asds] seed %u: %s\n", seed, sim::to_string(session.landing_outcome(0)));
}
