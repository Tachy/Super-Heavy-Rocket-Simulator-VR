// Staging sequence (ascent trajectory and staging, Task 1): cold staging with its delays (Falcon 9), hot staging with
// the ship lit while joined (Starship), the Falcon 9 booster's RCS flip and a repeated manual stage command.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/mission_timeline.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"
#include "webcast_report.hpp"

using namespace starsim;
using sim::MissionEventKind;

namespace {

const spice::SpiceEphemeris& staging_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& staging_earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

// The first recorded event of `kind` (of body `serial`; 0: any body).
const sim::MissionEvent* recorded(const sim::MissionTimeline& t, MissionEventKind kind, uint32_t serial = 0) {
    for (const auto& e : t.recorded()) {
        if (e.kind == kind && (serial == 0 || e.body_serial == serial)) return &e;
    }
    return nullptr;
}

size_t count(const sim::MissionTimeline& t, MissionEventKind kind) {
    size_t n = 0;
    for (const auto& e : t.recorded()) n += e.kind == kind ? 1 : 0;
    return n;
}

// Along-track speed of body `a` relative to body `b`, along b's velocity.
double along_track_mps(const sim::FlightSession& s, size_t a, size_t b) {
    const auto& va = s.body(a).state().velocity_gcrf_mps;
    const auto& vb = s.body(b).state().velocity_gcrf_mps;
    return dot(va - vb, math::normalized(vb));
}

// Flies the mission until the booster phase has ended (MECO commanded), then tick by tick until the separation (two
// bodies). Returns false if the flight never separated.
// `saw_staging` (optional): the autopilot was in the state Staging on the way (the MECO of the Booster state).
bool fly_to_separation(sim::FlightSession& session, bool* saw_staging = nullptr) {
    const double tick = sim::FlightSession::tick_s;
    const auto t0 = session.epoch();
    while (session.epoch() - t0 < 400.0 && session.body_count() < 2) {
        const bool booster = session.autopilot().state == sim::AutopilotState::Booster;
        session.advance_to(session.epoch() + (booster ? 0.1 : 1.5 * tick), 1000);
        if (saw_staging && session.autopilot().state == sim::AutopilotState::Staging) *saw_staging = true;
    }
    return session.body_count() >= 2;
}

}  // namespace

TEST_CASE("Cold staging: MECO, separation after the delay, upper ignition after its delay", "[staging][ascent_planner]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 13, 10, 40, 0.0});
    const auto staging = *config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5").stages.front().staging;
    REQUIRE(staging.kind == config::StagingConfig::Kind::Cold);
    sim::FlightSession session(staging_ephemeris(), staging_earth(), STARSIM_CONFIG_DIR);
    sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_rtls", t0, time);
    bool saw_staging = false;
    REQUIRE(fly_to_separation(session, &saw_staging));
    CHECK(saw_staging);  // the autopilot's MECO puts it in Staging (the placed stack of the re-engage case starts there)
    // right after the separation: the push sends the booster back, no tail-off acts on it alone
    const double relative_mps = along_track_mps(session, 0, session.controlled_index());
    session.advance_to(session.epoch() + staging.upper_ignition_delay_s + 2.0, 100000);

    const auto& t = session.timeline();
    const auto* meco = recorded(t, MissionEventKind::Meco);
    const auto* separation = recorded(t, MissionEventKind::StageSeparation);
    REQUIRE(meco);
    REQUIRE(separation);
    const auto* ignition = recorded(t, MissionEventKind::Ignition, session.body_serial(session.controlled_index()));
    REQUIRE(ignition);
    std::printf("[staging] Falcon 9: separation %.3f s after MECO, upper ignition %.3f s after it; booster %+.2f m/s "
                "along track\n",
                separation->epoch - meco->epoch, ignition->epoch - separation->epoch, relative_mps);
    const double tick = sim::FlightSession::tick_s;
    CHECK(separation->epoch - meco->epoch == Catch::Approx(staging.separation_delay_s).margin(tick));
    CHECK(ignition->epoch - separation->epoch == Catch::Approx(staging.upper_ignition_delay_s).margin(tick));
    CHECK(relative_mps <= 0.5);
    CHECK(count(t, MissionEventKind::Ignition) == 1);  // the observer does not record the upper ignition again
    CHECK(session.autopilot().state == sim::AutopilotState::UpperStage);
}

TEST_CASE("Hot staging: the ship lights while joined and separates at thrust; the booster falls back", "[staging][ascent_planner]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::FlightSession session(staging_ephemeris(), staging_earth(), STARSIM_CONFIG_DIR);
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);
    bool saw_staging = false;
    REQUIRE(fly_to_separation(session, &saw_staging));
    CHECK(saw_staging);  // the autopilot's MECO puts it in Staging (the placed stack of the re-engage case starts there)
    const size_t ship = session.controlled_index();
    REQUIRE(ship == 1);
    // the separation tick: every ship engine at thrust
    for (const auto& stage : session.body(ship).vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (!e.auxiliary()) CHECK(e.mode == vehicle::EngineMode::Running);
        }
    }
    const auto& t = session.timeline();
    const auto* meco = recorded(t, MissionEventKind::Meco);
    const auto* separation = recorded(t, MissionEventKind::StageSeparation);
    const auto* ignition = recorded(t, MissionEventKind::Ignition, session.body_serial(ship));
    REQUIRE(meco);
    REQUIRE(separation);
    REQUIRE(ignition);
    CHECK(ignition->epoch == meco->epoch);  // the ship lights in the MECO's tick, joined
    CHECK(ignition->body_name == "Starship Block 3");
    // separated as soon as the ship's engines are at thrust: after their start-up, at most one tick later
    double startup_s = 0.0;
    for (const auto& e : session.body(ship).vehicle().stages().front().engines) {
        if (!e.auxiliary()) startup_s = std::max(startup_s, e.performance->config().startup_s);
    }
    REQUIRE(startup_s > 0.0);
    CHECK(separation->epoch - meco->epoch > 0.0);
    CHECK(separation->epoch - meco->epoch <= startup_s + sim::FlightSession::tick_s + 1e-9);

    std::vector<double> relative_mps{along_track_mps(session, 0, ship)};
    for (int i = 1; i <= 5; ++i) {
        session.advance_to(separation->epoch + static_cast<double>(i), 100000);
        relative_mps.push_back(along_track_mps(session, 0, ship));
    }
    std::printf("[staging] Starship: separation %.3f s after MECO; booster along track", separation->epoch - meco->epoch);
    for (const double v : relative_mps) std::printf(" %+.2f", v);
    std::printf(" m/s\n");
    CHECK(relative_mps.front() <= 0.5);
    for (size_t i = 1; i < relative_mps.size(); ++i) CHECK(relative_mps[i] < relative_mps[i - 1]);
    // the booster keeps its centre engines into the flip
    CHECK(session.telemetry(0).engines_running == 3);
}

namespace {

struct BoosterFlip {
    bool lit_in_flip = false;
    double boostback_s = -1.0;  // after the separation
    double n2_used_kg = 0.0;
};

BoosterFlip fly_booster_flip() {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 13, 10, 40, 0.0});
    sim::FlightSession session(staging_ephemeris(), staging_earth(), STARSIM_CONFIG_DIR);
    sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_rtls", t0, time);
    REQUIRE(fly_to_separation(session));
    const auto separated = session.epoch();
    const double rcs_at_separation_kg = session.body(0).vehicle().rcs_supply_kg();
    std::printf("[staging] Falcon 9 booster at separation: pitch inertia %.3g kg m2, RCS pitch torque %.3g Nm, N2 %.0f kg\n",
                session.body(0).vehicle().mass_properties().inertia_kg_m2.y,
                session.body(0).vehicle().max_rcs_torque_nm().y, rcs_at_separation_kg);
    bool lit_in_flip = false;
    std::optional<double> boostback_s;
    while (session.epoch() - separated < 60.0 && !boostback_s) {
        const auto* status = session.return_status(0);
        REQUIRE(status);
        if (status->phase == gnc::ReturnPhase::Boostback) {
            boostback_s = session.epoch() - separated;
            break;
        }
        lit_in_flip = lit_in_flip || session.body(0).vehicle().any_engine_producing_thrust();
        session.advance_to(session.epoch() + 1.5 * sim::FlightSession::tick_s, 10);
    }
    REQUIRE(boostback_s);
    BoosterFlip flip;
    flip.lit_in_flip = lit_in_flip;
    flip.boostback_s = *boostback_s;
    flip.n2_used_kg = rcs_at_separation_kg - session.body(0).vehicle().rcs_supply_kg();
    std::printf("[staging] Falcon 9 booster: boostback %.1f s after the separation, %.1f kg N2 in the flip\n", flip.boostback_s,
                flip.n2_used_kg);
    return flip;
}

}  // namespace

TEST_CASE("The Falcon 9 booster flips on RCS with its engines off", "[staging][ascent_planner]") {
    // Physical gate only (user 2026-11-03): the timing against the SpaceX timelines is the nightly report below.
    const auto flip = fly_booster_flip();
    CHECK_FALSE(flip.lit_in_flip);
    CHECK(flip.boostback_s > 0.0);  // the flip ends in the boostback
}

TEST_CASE("The Falcon 9 booster flip: boostback time against the SpaceX timelines (report)", "[nightly][webcast][staging][ascent_planner]") {
    const auto flip = fly_booster_flip();
    // separation -> boostback ignition: 13 / 14 / 14 s on CRS-35, Bandwagon-3 and Crew-13 (SpaceX mission timelines,
    // config/vehicles/falcon9_block5.toml [sources]); mean 13.7 s. The band is the former gate's +-3 s; the tighter
    // +-0.6 s of the timelines' spread is in the falcon9_rtls report.
    test::webcast_report("staging_falcon9_flip",
                         {{"separation -> boostback [s]", flip.boostback_s, 13.7, 3.0,
                           "SpaceX timelines CRS-35 / Bandwagon-3 / Crew-13 mean",
                           "flip_duration_s, boostback_loss_fraction, RCS thrust and N2 supply, stage inertia"}});
}

TEST_CASE("A manual stage command during the sequence separates once", "[staging]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(staging_ephemeris(), staging_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const double deg = constants::deg_to_rad;
    session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, t0);
    session.ignite();
    session.release_clamps();
    session.advance_to(t0 + 10.0);
    session.stage();
    session.stage();
    CHECK(session.body_count() == 1);  // joined while the ship's engines start
    session.advance_to(t0 + 10.5);
    session.stage();
    session.advance_to(t0 + 20.0);
    session.stage();  // the ship alone: a single stage, nothing to separate
    CHECK(session.body_count() == 2);
    CHECK(count(session.timeline(), MissionEventKind::StageSeparation) == 1);
}

// The sequence is started on a placed stack: launched from the pad, 10 s under thrust, then the stage command (the
// manual command of the case above) - no ascent plan, no flight to the real MECO (formerly both missions were flown
// from the pad to their MECO: 124 s). engage_ascent_autopilot resets the autopilot and takes the state from the
// sequence under way, so the state before the engagement does not matter.
TEST_CASE("Re-engaging the ascent autopilot during a staging sequence keeps it in Staging", "[staging][ascent_planner]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const double tick = sim::FlightSession::tick_s;
    const double deg = constants::deg_to_rad;
    gnc::AscentTarget target{200e3, 200e3, 26.5 * deg, true};
    gnc::PitchProgram program;
    program.kick_angle_rad = 14.0 * deg;
    struct Stack {
        const char* vehicle_id;
        const char* site_id;
        double heading_deg;
    };
    // hot: joined; cold: the separated upper stage
    for (const Stack stack : {Stack{"starship_block3", "starbase_pad2", 90.0}, Stack{"falcon9_block5", "ccsfs_slc40", 45.0}}) {
        sim::FlightSession session(staging_ephemeris(), staging_earth(), STARSIM_CONFIG_DIR);
        const auto t0 = time.from_utc({2026, 10, 13, 10, 40, 0.0});
        session.launch_from_pad(stack.vehicle_id,
                                sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, stack.site_id)),
                                stack.heading_deg * deg, t0);
        session.set_booster_return(false);  // the booster's fate is not the subject
        session.ignite();
        session.release_clamps();
        session.advance_to(t0 + 10.0);
        session.stage();
        REQUIRE(session.staging());
        const bool hot = session.staging()->config.kind == config::StagingConfig::Kind::Hot;
        if (!hot) {  // cold: on to the separation, the upper stage not yet lit
            for (int i = 0; i < 2000 && !session.staging()->separation_epoch; ++i) {
                session.advance_to(session.epoch() + 1.5 * tick, 10);
            }
            REQUIRE(session.staging());
            REQUIRE(session.staging()->separation_epoch);
            REQUIRE_FALSE(session.staging()->upper_ignition_epoch);
        }
        INFO(stack.vehicle_id);
        session.engage_ascent_autopilot(target, program);
        CHECK(session.autopilot().state == sim::AutopilotState::Staging);  // not Booster on the joined stack
        // the sequence still ends in the upper stage's flight
        for (int i = 0; i < 2000 && session.staging(); ++i) session.advance_to(session.epoch() + 1.5 * tick, 10);
        CHECK_FALSE(session.staging());
        CHECK(session.autopilot().state == sim::AutopilotState::UpperStage);
        CHECK(session.body_count() == 2);
    }
}
