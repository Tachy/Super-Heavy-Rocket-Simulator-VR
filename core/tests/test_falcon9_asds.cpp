// Falcon 9 landing on the drone ship (milestone c4): the deck as a landing zone, the drone-ship return (no boostback,
// three-engine entry burn, one-engine landing burn) and the Starlink-class mission from SLC-40.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
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
    const vehicle::Vehicle stage(test::booster_config("falcon9_block5", 11300.0), STARSIM_CONFIG_DIR);
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
constexpr double asds_ignition_propellant_kg = 11300.0;  // [D] gauge 2.8 % at ignition

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
    const double at_a = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, a, p, 11300.0);
    const double at_b = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, state, earth(), t0, b, p, 11300.0);
    // Entry burn by the rocket equation: (25.6 + 11.3) t x exp(880 / (311 x 9.807)) - 25.6 t = 23.6 t; nine engines'
    // run-down (0.15 s each at full flow) ~0.4 t.
    std::printf("[falcon9] drone-ship MECO propellant %.1f t\n", at_a / 1e3);
    CHECK(at_a == at_b);
    CHECK(at_a > 23.0e3);
    CHECK(at_a < 25.0e3);
}

TEST_CASE("A falling Falcon 9 stage flies its drone-ship entry burn on three engines", "[gnc][falcon9][asds]") {
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const auto cfg = test::booster_config("falcon9_block5", 24100.0);  // [D] gauge 6.1 % at the entry burn
    // 75 km up, 40 km short of the ship, flying towards it at 1.9 km/s horizontally, falling at 1.0 km/s, engines first.
    const Vec3 velocity{0.0, 1900.0, -1000.0};
    const Vec3 axis = math::normalized(-velocity);
    const auto tilt = math::Quat::from_axis_angle(math::normalized(cross(Vec3{0.0, 0.0, 1.0}, axis)),
                                                  std::acos(std::clamp(axis.z, -1.0, 1.0)));
    session.place_returning_booster(cfg, test::state_over_site(session.launch_site(), earth(), t0, {0.0, -40e3, 75e3}, velocity, tilt),
                                    t0, asds_parameters(), gnc::ReturnPhase::Coast);
    bool burned = false, ended = false, finite = true;
    double ignition_altitude_m = 0.0, propellant_before_kg = 0.0;
    std::vector<std::string> groups;
    for (double t = 0.0; t < 60.0 && !ended;) {
        t += sim::FlightSession::tick_s;
        session.advance_to(t0 + t, 10);
        const auto* status = session.return_status(0);
        REQUIRE(status != nullptr);
        finite = finite && std::isfinite(status->axis_gcrf.x) && std::isfinite(status->axis_gcrf.y) &&
                 std::isfinite(status->axis_gcrf.z);
        if (status->phase == gnc::ReturnPhase::EntryBurn && !burned) {
            burned = true;
            groups = status->run_groups;
            propellant_before_kg = session.body(0).vehicle().propellant_kg();
            const auto tr = earth().fixed_to_gcrf(session.epoch());
            ignition_altitude_m = frames::fixed_to_geodetic(transpose(tr.rotation) * session.body(0).state().position_gcrf_m).height_m -
                                  session.launch_site().height_m;
        }
        if (burned && status->phase != gnc::ReturnPhase::EntryBurn) ended = true;
    }
    const auto* guidance = session.return_guidance(0);
    const double used_kg = propellant_before_kg - session.body(0).vehicle().propellant_kg();
    std::printf("[falcon9] drone-ship entry burn from %.1f km, %.0f m/s burned, %.1f t used\n", ignition_altitude_m / 1e3,
                guidance->entry_delta_v_mps(), used_kg / 1e3);
    REQUIRE(burned);
    REQUIRE(ended);
    CHECK(groups == std::vector<std::string>{"first_stage/center", "first_stage/pair"});
    CHECK(finite);  // the commanded axis, also on the burn's first ticks
    CHECK(ignition_altitude_m < 66.0e3);
    CHECK(ignition_altitude_m > 64.0e3);
    CHECK(guidance->entry_delta_v_mps() >= 880.0);
    CHECK(guidance->entry_delta_v_mps() < 900.0);
    CHECK(std::abs(used_kg - 12800.0) < 1000.0);  // [D] 24.1 -> 11.3 t
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

TEST_CASE("The Falcon 9 drone-ship mission loads", "[config][falcon9][asds]") {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    CHECK(m.vehicle_id == "falcon9_block5");
    CHECK(m.site_id == "ccsfs_slc40");
    CHECK(m.landing_zone_id.empty());
    REQUIRE(m.drone_ship);
    CHECK(m.drone_ship->id == "asds_atlantic");
    CHECK(m.payload_kg > 16000.0);
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

TEST_CASE("Falcon 9 drone-ship mission: nominal impact point of the first stage", "[.][falcon9][asds]") {
    // Diagnostic: where the stage would come down from MECO (drag, the entry burn as an impulse, no steering) - the
    // drone ship's position in config/missions/falcon9_asds.toml [D].
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_asds");
    const auto t0 = start();
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    for (double t = 0.0; t < 300.0 && session.body_count() < 2; t += 0.05) session.advance_to(t0 + t, 100);
    REQUIRE(session.body_count() == 2);
    const auto& stage = session.body(0);
    const auto p = session.return_parameters(stage.vehicle().stages().front().config);
    const gnc::BoosterReturnGuidance::PendingBurn burn{p.entry_altitude_m, p.entry_delta_v_mps};
    const double ground = session.landing_zone()->height_m;
    const auto impact = gnc::BoosterReturnGuidance::predict_impact(stage, session.environment(), session.epoch(), ground,
                                                                   nullptr, &burn);
    REQUIRE(impact.valid);
    const auto geo = frames::fixed_to_geodetic(impact.position_itrf_m);
    const auto tr = earth().fixed_to_gcrf(session.epoch());
    const auto meco = frames::fixed_to_geodetic(transpose(tr.rotation) * stage.state().position_gcrf_m);
    std::printf("[falcon9] MECO T+%.1f s at %.1f km with %.1f t; impact after %.0f s at latitude %.4f deg, longitude %.4f "
                "deg\n", session.mission_time_s(), meco.height_m / 1e3, stage.vehicle().propellant_kg() / 1e3, impact.time_s,
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
            std::printf("    T+%.0f s %s %.1f km %.0f m/s miss %.0f m\n", session.mission_time_s(), gnc::to_string(status->phase),
                        h / 1e3, v, status->impact_miss_m);
        }
        if (status->phase != last_phase) {
            const auto [h, v] = height_speed(stage);
            if (std::getenv("STARSIM_VERBOSE")) {
                std::printf("  T+%.1f s %s: %.1f km, %.0f m/s, %.1f t, miss %.0f m\n", session.mission_time_s(),
                            gnc::to_string(status->phase), h / 1e3, v, stage.vehicle().propellant_kg() / 1e3, status->impact_miss_m);
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
    std::printf("[validation] Falcon 9 ASDS: MECO T+%.0f s at %.1f km, %.0f km/h, %.1f t (Starlink 15-25: T+149 s, 60 km, "
                "~8000 km/h, gauge ~43 t); entry burn T+%.0f s at %.1f km, %.0f km/h, %.1f t, %.1f deg off the burn axis "
                "(T+364 s, 67.4 km, ~7900 km/h, 24.1 t) to T+%.0f s, %.0f km/h (T+386 s, ~5250 km/h); landing burn T+%.0f "
                "s at %.2f km, %.0f m/s, %.1f t (T+476 s, ~3.0 km, ~270 m/s, 11.1 t); %s at T+%.0f s (T+499 s), %.2f m from "
                "the deck centre, %.1f t left\n",
                meco_s, meco_h / 1e3, meco_v * 3.6, meco_kg / 1e3, entry_s, entry_h / 1e3, entry_v * 3.6, entry_kg / 1e3,
                entry_pointing_deg, entry_end_s, entry_end_v * 3.6, landing_s, landing_h / 1e3, landing_v,
                landing_kg / 1e3, sim::to_string(session.landing_outcome(0)), session.mission_time_s(), std::hypot(p.x, p.y),
                stage.vehicle().propellant_kg() / 1e3);
    CHECK(meco_s > 0.0);
    CHECK(entry_s > 0.0);
    CHECK(entry_pointing_deg < 15.0);  // turned engines first on the RCS before the burn
    CHECK(landing_s > 0.0);
    CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
    CHECK(std::hypot(p.x, p.y) < 5.0);
    // The landing-burn ignition state lies within the telemetry's spread (spec): ~3.0 km, ~270 m/s, 11.1 t.
    CHECK(std::abs(landing_kg - 11300.0) < 1000.0);
    CHECK(landing_h > 2500.0);
    CHECK(landing_h < 4000.0);
    CHECK(std::abs(landing_v - 270.0) < 40.0);
    // Timeline [D] of Starlink 15-25: entry burn T+364 s, landing burn T+476 s.
    CHECK(std::abs(entry_s - 364.0) < 25.0);
    CHECK(std::abs(landing_s - 476.0) < 25.0);
}
