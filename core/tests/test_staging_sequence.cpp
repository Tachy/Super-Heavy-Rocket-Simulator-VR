// Staging sequence (ascent trajectory and staging, Task 1): cold staging with its delays (Falcon 9), hot staging with
// the ship lit while joined (Starship), the Falcon 9 booster's RCS flip and a repeated manual stage command.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/mission_timeline.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

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
bool fly_to_separation(sim::FlightSession& session) {
    const double tick = sim::FlightSession::tick_s;
    const auto t0 = session.epoch();
    while (session.epoch() - t0 < 400.0 && session.body_count() < 2) {
        const bool booster = session.autopilot().state == sim::AutopilotState::Booster;
        session.advance_to(session.epoch() + (booster ? 0.1 : 1.5 * tick), 1000);
    }
    return session.body_count() >= 2;
}

}  // namespace

TEST_CASE("Cold staging: MECO, separation after the delay, upper ignition after its delay", "[staging]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 13, 10, 40, 0.0});
    const auto staging = *config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5").stages.front().staging;
    REQUIRE(staging.kind == config::StagingConfig::Kind::Cold);
    sim::FlightSession session(staging_ephemeris(), staging_earth(), STARSIM_CONFIG_DIR);
    sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_rtls", t0, time);
    REQUIRE(fly_to_separation(session));
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

TEST_CASE("Hot staging: the ship lights while joined and separates at thrust; the booster falls back", "[staging]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::FlightSession session(staging_ephemeris(), staging_earth(), STARSIM_CONFIG_DIR);
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);
    REQUIRE(fly_to_separation(session));
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
    CHECK(separation->epoch - meco->epoch > 0.0);

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

TEST_CASE("The Falcon 9 booster flips on RCS with its engines off", "[staging]") {
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
    std::printf("[staging] Falcon 9 booster: boostback %.1f s after the separation, %.1f kg N2 in the flip\n", *boostback_s,
                rcs_at_separation_kg - session.body(0).vehicle().rcs_supply_kg());
    CHECK_FALSE(lit_in_flip);
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
