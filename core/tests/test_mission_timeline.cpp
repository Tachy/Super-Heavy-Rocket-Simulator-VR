#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "lunar_return_test_support.hpp"
#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/capsule_return_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/iss_mission.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/sim/lunar_return_mission.hpp"
#include "starsim/sim/mission_event_observer.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/mission_timeline.hpp"
#include "starsim/sim/refuelling_mission.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("Event kinds have snake_case ids", "[timeline]") {
    CHECK(std::string(sim::id_of(sim::MissionEventKind::MaxQ)) == "max_q");
    CHECK(std::string(sim::id_of(sim::MissionEventKind::StageSeparation)) == "stage_separation");
    CHECK(std::string(sim::id_of(sim::MissionEventKind::Liftoff)) == "liftoff");
}

TEST_CASE("Body serials are unique and survive reindexing", "[timeline]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);

    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const double deg = constants::deg_to_rad;
    session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, t0);
    REQUIRE(session.body_count() == 1);
    const uint32_t before = session.body_serial(0);
    CHECK(before >= 1);

    session.ignite();
    session.release_clamps();
    session.advance_to(t0 + 10.0);
    session.stage();
    session.advance_to(t0 + 15.0);  // hot staging: separated once the ship's engines are at thrust
    REQUIRE(session.body_count() == 2);

    const uint32_t a = session.body_serial(0);
    const uint32_t b = session.body_serial(1);
    CHECK(a >= 1);
    CHECK(b >= 1);
    CHECK(a != b);
    CHECK((a == before || b == before));
}

namespace {
using sim::MissionEvent;
using sim::MissionEventKind;
const time::Epoch kBase = time::Epoch::from_tdb_seconds_past_j2000(1.0e8);
MissionEvent event(MissionEventKind kind, double offset_s, bool predicted = false) {
    return MissionEvent{kind, kBase + offset_s, 0, "", predicted};
}
}  // namespace

TEST_CASE("T-0 of a pad mission is the scheduled liftoff, then the real one", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(true);
    timeline.set_scheduled_liftoff(kBase + 100.0);
    REQUIRE(timeline.t0().has_value());
    CHECK(*timeline.t0() == kBase + 100.0);
    CHECK(*timeline.mission_clock_s(kBase) == -100.0);

    timeline.record(event(MissionEventKind::Liftoff, 101.5));
    CHECK(*timeline.t0() == kBase + 101.5);
}

TEST_CASE("A pad mission past its T-0 without a liftoff holds the clock at T-0", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(true);
    timeline.set_scheduled_liftoff(kBase + 100.0);
    CHECK(*timeline.mission_clock_s(kBase + 40.0) == -60.0);
    CHECK(*timeline.mission_clock_s(kBase + 100.0) == 0.0);
    CHECK(*timeline.mission_clock_s(kBase + 160.0) == 0.0);  // a hold on the pad: no T+ before the liftoff
    timeline.record(event(MissionEventKind::Liftoff, 170.0));
    CHECK(*timeline.mission_clock_s(kBase + 175.0) == 5.0);  // then from the real liftoff
}

TEST_CASE("An engine start before liftoff is no T-0", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(true);
    timeline.record(event(MissionEventKind::EngineStart, 0.0));
    CHECK_FALSE(timeline.t0().has_value());
}

TEST_CASE("T-0 of an orbit mission is its first action", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(false);
    timeline.set_predicted({event(MissionEventKind::Ignition, 3600.0, true)});
    REQUIRE(timeline.t0().has_value());
    CHECK(*timeline.t0() == kBase + 3600.0);

    timeline.record(event(MissionEventKind::Undocking, 10.0));
    CHECK(*timeline.t0() == kBase + 10.0);
}

TEST_CASE("No action, no T-0", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(false);
    CHECK_FALSE(timeline.t0().has_value());
    CHECK_FALSE(timeline.mission_clock_s(kBase).has_value());
}

TEST_CASE("Recorded events stay sorted; predicted are replaced", "[timeline]") {
    sim::MissionTimeline timeline;
    timeline.reset(true);
    timeline.record(event(MissionEventKind::Meco, 160.0));
    timeline.record(event(MissionEventKind::Liftoff, 0.0, true));  // predicted flag is forced off
    timeline.record(event(MissionEventKind::MaxQ, 70.0));
    REQUIRE(timeline.recorded().size() == 3);
    CHECK(timeline.recorded()[0].kind == MissionEventKind::Liftoff);
    CHECK(timeline.recorded()[1].kind == MissionEventKind::MaxQ);
    CHECK(timeline.recorded()[2].kind == MissionEventKind::Meco);
    CHECK_FALSE(timeline.recorded()[0].predicted);

    timeline.set_predicted({event(MissionEventKind::Seco, 500.0), event(MissionEventKind::Landed, 900.0)});
    timeline.set_predicted({event(MissionEventKind::Splashdown, 1200.0)});
    REQUIRE(timeline.predicted().size() == 1);
    CHECK(timeline.predicted()[0].kind == MissionEventKind::Splashdown);
    CHECK(timeline.predicted()[0].predicted);
    CHECK(std::string(sim::to_string(MissionEventKind::Meco)) == "MECO");
}

namespace {
const spice::SpiceEphemeris& timeline_ephemeris() {
    static const spice::SpiceEphemeris e;
    return e;
}
const spice::SpiceOrientation& timeline_earth() {
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

// The kinds of `events` (optionally of one body) contain `expected` in this order (other events between them allowed).
bool contains_in_order(const std::vector<MissionEvent>& events, const std::vector<MissionEventKind>& expected,
                       uint32_t serial = 0) {
    size_t k = 0;
    for (const auto& e : events) {
        if (k < expected.size() && e.kind == expected[k] && (serial == 0 || e.body_serial == serial)) ++k;
    }
    return k == expected.size();
}

const MissionEvent* find_event(const sim::MissionTimeline& t, MissionEventKind kind) {
    for (const auto& e : t.recorded()) {
        if (e.kind == kind) return &e;
    }
    return nullptr;
}

// The predicted event of this kind (of the body `serial`; 0: any body).
const MissionEvent* predicted_of(const sim::MissionTimeline& t, MissionEventKind kind, uint32_t serial = 0) {
    for (const auto& e : t.predicted()) {
        if (e.kind == kind && (serial == 0 || e.body_serial == serial)) return &e;
    }
    return nullptr;
}

// The recorded event of this kind of the body `serial` (the first after `after`, if given).
const MissionEvent* recorded_of(const sim::MissionTimeline& t, MissionEventKind kind, uint32_t serial,
                                std::optional<time::Epoch> after = std::nullopt) {
    for (const auto& e : t.recorded()) {
        if (e.kind == kind && e.body_serial == serial && (!after || e.epoch > *after)) return &e;
    }
    return nullptr;
}

// A prediction as sampled: the epochs of the sample and of the predicted event.
struct PredictionTrack {
    std::vector<std::pair<time::Epoch, time::Epoch>> samples;
    std::optional<time::Epoch> first_seen, first, last_seen, last;
    void sample(const MissionEvent* e, time::Epoch now) {
        if (!e) return;
        samples.emplace_back(now, e->epoch);
        if (!first) {
            first_seen = now;
            first = e->epoch;
        }
        last_seen = now;
        last = e->epoch;
    }
    // The largest error |predicted - actual| of the samples taken at most `lead_s` before `actual` (0: none).
    double max_error_within_s(time::Epoch actual, double lead_s) const {
        double worst = 0.0;
        for (const auto& [now, predicted] : samples) {
            if (actual - now <= lead_s) worst = std::max(worst, std::abs(predicted - actual));
        }
        return worst;
    }
};

// A catch of the body `serial` is on the timeline, predicted or recorded.
bool caught_shown(const sim::MissionTimeline& t, uint32_t serial) {
    return predicted_of(t, MissionEventKind::Caught, serial) || recorded_of(t, MissionEventKind::Caught, serial);
}

void print_prediction(const char* label, const PredictionTrack& p, const MissionEvent* recorded) {
    if (!std::getenv("STARSIM_VERBOSE") || !p.first || !recorded) return;
    std::printf("  [prediction] %-12s first seen %.1f s before: error %+.2f s; last seen %.1f s before: error %+.2f s\n",
                label, recorded->epoch - *p.first_seen, *p.first - recorded->epoch, recorded->epoch - *p.last_seen,
                *p.last - recorded->epoch);
    if (std::getenv("STARSIM_DEBUG_PREDICTIONS")) {  // every sample
        for (const auto& [now, predicted] : p.samples) {
            std::printf("  [sample] %-12s %.1f s before: error %+.2f s\n", label, recorded->epoch - now, predicted - recorded->epoch);
        }
    }
    for (const double lead : {120.0, 60.0, 30.0, 10.0}) {
        std::printf("  [prediction] %-12s max error within %3.0f s before: %.2f s\n", label, lead,
                    p.max_error_within_s(recorded->epoch, lead));
    }
}

void print_events(const sim::MissionTimeline& t) {
    if (!std::getenv("STARSIM_VERBOSE")) return;
    for (const auto& e : t.recorded()) {
        std::printf("  [timeline] %.2f %-16s body %u %s\n", e.epoch.tdb_seconds_past_j2000(), sim::to_string(e.kind),
                    e.body_serial, e.body_name.c_str());
    }
}
}  // namespace

TEST_CASE("A Starship ascent records liftoff, Max Q, MECO, separation and SECO", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);

    double q_peak_pa = 0.0;  // the stack's q while climbing, sampled by the test
    time::Epoch q_peak_epoch = t0;
    std::optional<time::Epoch> seco_command;  // the tick whose autopilot commanded the upper stage's cutoff
    const auto& t = session.timeline();
    // onboard predictions (Task 4b): MECO and separation at T+30 s, SECO from the upper stage's closed loop
    std::optional<time::Epoch> meco_at_30, separation_at_30;
    PredictionTrack meco_prediction, seco_prediction;
    while (session.epoch() - t0 < 900.0 && !find_event(t, MissionEventKind::Seco)) {
        if (const auto* lifted = find_event(t, MissionEventKind::Liftoff);
            lifted && !meco_at_30 && session.epoch() - lifted->epoch >= 30.0) {
            if (const auto* p = predicted_of(t, MissionEventKind::Meco)) meco_at_30 = p->epoch;
            if (const auto* p = predicted_of(t, MissionEventKind::StageSeparation)) separation_at_30 = p->epoch;
        }
        meco_prediction.sample(predicted_of(t, MissionEventKind::Meco), session.epoch());
        seco_prediction.sample(predicted_of(t, MissionEventKind::Seco), session.epoch());
        const bool upper = session.autopilot().state == sim::AutopilotState::UpperStage;
        const auto& g = session.autopilot().guidance;  // single ticks at the cutoff
        const bool near = upper && g.phase == gnc::AscentPhase::ClosedLoop && g.time_to_go_s < 1.0;
        const auto before = session.epoch();
        session.advance_to(before + (near ? 1.5 * sim::FlightSession::tick_s : 0.1), 100);  // near: one tick
        if (upper && session.autopilot().state != sim::AutopilotState::UpperStage) seco_command = before;
        if (session.body_count() == 0) break;
        const auto tm = session.telemetry(0);
        if (tm.vertical_speed_mps > 0.0 && tm.dynamic_pressure_pa > q_peak_pa) {
            q_peak_pa = tm.dynamic_pressure_pa;
            q_peak_epoch = session.epoch();
        }
    }
    print_events(t);
    using K = MissionEventKind;
    CHECK(contains_in_order(t.recorded(), {K::EngineStart, K::Liftoff, K::MaxQ, K::Meco, K::StageSeparation, K::Seco}));
    const auto* liftoff = find_event(t, K::Liftoff);
    const auto* max_q = find_event(t, K::MaxQ);
    const auto* meco = find_event(t, K::Meco);
    const auto* seco = find_event(t, K::Seco);
    REQUIRE(liftoff);
    REQUIRE(max_q);
    REQUIRE(meco);
    REQUIRE(seco);
    CHECK(max_q->epoch - liftoff->epoch > 50.0);
    CHECK(max_q->epoch - liftoff->epoch < 80.0);
    CHECK(q_peak_pa >= 10e3);
    CHECK(std::abs(max_q->epoch - q_peak_epoch) <= 0.2);  // the observer's 0.1 s sampling
    CHECK(meco->body_serial != seco->body_serial);
    // the part that acts: the stack at the pad, the booster at MECO and separation, the ship at SECO
    CHECK(liftoff->body_name == "Heavy Launcher Mk3");
    CHECK(meco->body_name == "Super Heavy Block 3");
    CHECK(find_event(t, K::StageSeparation)->body_name == "Super Heavy Block 3");
    CHECK(seco->body_name == "Starship Block 3");
    REQUIRE(seco_command);
    CHECK(std::abs(seco->epoch - *seco_command) <= sim::FlightSession::tick_s);  // the command, not the tail-off end
    CHECK(find_event(t, K::EntryInterface) == nullptr);  // the 200 km orbit: no entry
    CHECK(*t.t0() == liftoff->epoch);

    // Onboard predictions: the booster's cutoff criterion run forward, PEG's time to go.
    const auto* separation = find_event(t, K::StageSeparation);
    print_prediction("MECO", meco_prediction, meco);
    print_prediction("SECO", seco_prediction, seco);
    if (std::getenv("STARSIM_VERBOSE") && meco_at_30) {
        std::printf("  [prediction] MECO at T+30 s: error %+.2f s\n", *meco_at_30 - meco->epoch);
    }
    REQUIRE(meco_at_30);
    REQUIRE(separation_at_30);
    // Measured 2026-10-22 (flow along the throttle schedule, the cutoff propellant of the moment): +6.97 s at T+30 s,
    // +6.1..+8.3 s over the booster phase (+8.2 s in the max-Q bucket) - late by the return's cutoff propellant, which
    // grows with the downrange speed; converging: +1.46 s 5.7 s before. (At the flow now alone: +0.87 s at T+30 s by
    // cancellation, up to 12 s off in the bucket.)
    CHECK(std::abs(*meco_at_30 - meco->epoch) <= 8.0);
    CHECK(std::abs(*separation_at_30 - separation->epoch) <= 8.0);  // hot staging: the ship's start-up after the MECO
    CHECK(meco_prediction.max_error_within_s(meco->epoch, 1e9) <= 9.0);
    CHECK(meco_prediction.max_error_within_s(meco->epoch, 3.0) <= 1.0);
    // PEG: -1.94 s at the first closed-loop tick, within 0.29 s over the last 120 s, +0.24 s at the end.
    REQUIRE(seco_prediction.first);
    CHECK(*seco_prediction.first_seen - separation->epoch <= 1.0);  // as soon as the upper stage's closed loop runs
    CHECK(std::abs(*seco_prediction.first - seco->epoch) <= 3.0);
    CHECK(seco_prediction.max_error_within_s(seco->epoch, 120.0) <= 0.5);
    CHECK(predicted_of(t, K::Meco) == nullptr);  // recorded: no longer predicted
    CHECK(predicted_of(t, K::Seco) == nullptr);
}

TEST_CASE("Booster return records boostback, landing burn and the catch", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);  // booster return on, tower catch

    const auto& t = session.timeline();
    std::optional<uint32_t> booster;
    PredictionTrack cutoff_prediction, landing_burn_prediction, catch_prediction;  // onboard predictions (Task 4b)
    int after_contact = 0, without_catch = 0;  // samples after the first pin contact; of these without a Caught
    while (session.epoch() - t0 < 900.0 && !find_event(t, MissionEventKind::Caught)) {
        session.advance_to(session.epoch() + 1.0, 1000);
        if (!booster) {
            if (const auto* meco = find_event(t, MissionEventKind::Meco)) booster = meco->body_serial;
        }
        if (booster) {
            cutoff_prediction.sample(predicted_of(t, MissionEventKind::Cutoff, *booster), session.epoch());
            landing_burn_prediction.sample(predicted_of(t, MissionEventKind::LandingBurn, *booster), session.epoch());
            catch_prediction.sample(predicted_of(t, MissionEventKind::Caught, *booster), session.epoch());
            for (size_t i = 0; i < session.body_count(); ++i) {  // from the pins' first touch a catch dot is shown
                if (session.body_serial(i) == *booster && session.catch_contact_epoch(i)) {
                    ++after_contact;
                    if (!caught_shown(t, *booster)) ++without_catch;
                }
            }
        }
        if (booster) {
            bool down = false;
            for (size_t i = 0; i < session.body_count(); ++i) {
                if (session.body_serial(i) == *booster) down = session.landing_outcome(i) != sim::LandingOutcome::Flying;
            }
            if (down) {
                session.advance_to(session.epoch() + 1.0, 1000);  // one more observation
                break;
            }
        }
    }
    print_events(t);
    REQUIRE(booster);
    using K = MissionEventKind;
    CHECK(contains_in_order(t.recorded(), {K::Boostback, K::LandingBurn, K::Caught}, *booster));

    // Onboard predictions of the return guidance: the boostback's cutoff, the landing burn's ignition, the catch.
    const auto* boostback = recorded_of(t, K::Boostback, *booster);
    REQUIRE(boostback);
    const auto* cutoff = recorded_of(t, K::Cutoff, *booster, boostback->epoch);
    const auto* landing_burn = recorded_of(t, K::LandingBurn, *booster);
    const auto* caught = recorded_of(t, K::Caught, *booster);
    REQUIRE(cutoff);
    REQUIRE(landing_burn);
    REQUIRE(caught);
    print_prediction("cutoff", cutoff_prediction, cutoff);
    print_prediction("landing burn", landing_burn_prediction, landing_burn);
    print_prediction("catch", catch_prediction, caught);
    REQUIRE(cutoff_prediction.first);
    REQUIRE(landing_burn_prediction.first);
    REQUIRE(catch_prediction.first);
    // Measured 2026-10-22: cutoff -1.57 s at the boostback's start, 0.16 s within the last 10 s; landing burn -5.4 s at
    // the apex, within 4.8 s on the way down, 1.4 s within the last 30 s; catch (recorded at the pins' first touch):
    // -16 s at the apex (ignition plus the one-dimensional burn duration), then the SCvx plan's end, 4.3 s within the
    // last 30 s, 0.9 s within the last 10 s. With the hot-staging sequence: landing burn -13.3 s, catch -23.5 s at the
    // apex (its first sample, where the fall rate is near 0: the height over the sink is ill-conditioned there and moves
    // with the apex state), unchanged from 120 s on (4.9 / 4.6 s).
    CHECK(*cutoff_prediction.first_seen - boostback->epoch <= 1.0);  // from the boostback's first second
    CHECK(cutoff_prediction.max_error_within_s(cutoff->epoch, 1e9) <= 2.0);
    CHECK(landing_burn_prediction.max_error_within_s(landing_burn->epoch, 1e9) <= 15.0);
    CHECK(landing_burn_prediction.max_error_within_s(landing_burn->epoch, 120.0) <= 6.0);
    CHECK(landing_burn_prediction.max_error_within_s(landing_burn->epoch, 30.0) <= 2.0);
    CHECK(catch_prediction.max_error_within_s(caught->epoch, 1e9) <= 25.0);
    CHECK(catch_prediction.max_error_within_s(caught->epoch, 120.0) <= 6.0);
    CHECK(catch_prediction.max_error_within_s(caught->epoch, 30.0) <= 5.0);
    CHECK(catch_prediction.max_error_within_s(caught->epoch, 10.0) <= 1.5);
    CHECK(after_contact > 0);
    CHECK(without_catch == 0);
    CHECK(predicted_of(t, K::LandingBurn, *booster) == nullptr);
    CHECK(predicted_of(t, K::Caught, *booster) == nullptr);
}

TEST_CASE("An undocking mission starts its clock at the undocking", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto st = sim::start_mission(session, STARSIM_CONFIG_DIR, "crew_dragon_iss_departure", {}, time);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    const auto& t = session.timeline();
    CHECK(t.recorded().empty());  // a scenario that starts docked: no Docking event
    const auto start = session.epoch();
    while (session.epoch() - start < 3600.0 && !find_event(t, MissionEventKind::Undocking)) {
        session.advance_to(session.epoch() + 10.0, 10000000);
    }
    print_events(t);
    const auto* undocking = find_event(t, MissionEventKind::Undocking);
    REQUIRE(undocking);
    REQUIRE(t.t0().has_value());
    CHECK(*t.t0() == undocking->epoch);
}

TEST_CASE("start_mission resets the timeline; campaign launches do not", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);
    session.advance_to(t0 + 15.0, 100000);
    const auto& t = session.timeline();
    REQUIRE(find_event(t, MissionEventKind::Liftoff));
    const auto first_liftoff = find_event(t, MissionEventKind::Liftoff)->epoch;

    const auto t1 = t0 + 3600.0;
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t1, time);
    CHECK(t.recorded().empty());
    session.advance_to(t1 + 15.0, 100000);
    REQUIRE(find_event(t, MissionEventKind::Liftoff));
    CHECK(find_event(t, MissionEventKind::Liftoff)->epoch != first_liftoff);
    const size_t events = t.recorded().size();

    // a campaign driver places its next vehicle without a mission start: the log goes on
    const double deg = constants::deg_to_rad;
    session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, session.epoch());
    CHECK(t.recorded().size() == events);
    session.ignite();
    session.advance_to(session.epoch() + 5.0, 100000);
    session.release_clamps();
    session.advance_to(session.epoch() + 1.0, 100000);
    CHECK(std::count_if(t.recorded().begin(), t.recorded().end(),
                        [](const MissionEvent& e) { return e.kind == MissionEventKind::Liftoff; }) == 2);
}

TEST_CASE("A refused start_mission keeps the running flight's log", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    // a config copy whose ISS launch finds no window: a 1 h search, a mission limit just past the insertion
    const auto dir = std::filesystem::temp_directory_path() / "starsim_timeline_refused_start";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::filesystem::copy(STARSIM_CONFIG_DIR, dir / "config", std::filesystem::copy_options::recursive);
    const auto mission_file = dir / "config" / "missions" / "falcon9_crew_dragon_iss.toml";
    std::stringstream text;
    text << std::ifstream(mission_file).rdbuf();
    std::string toml = text.str();
    const auto replace = [&](const std::string& from, const std::string& to) {
        const auto at = toml.find(from);
        REQUIRE(at != std::string::npos);
        toml.replace(at, from.size(), to);
    };
    replace("search_days = 3.0", "search_days = 0.04");
    replace("mission_limit_h = 28.0", "mission_limit_h = 0.5");
    std::ofstream(mission_file) << toml;

    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", t0, time);
    session.advance_to(t0 + 15.0, 100000);
    const auto& t = session.timeline();
    REQUIRE(find_event(t, MissionEventKind::Liftoff));
    const size_t events = t.recorded().size();

    const auto refused = sim::start_mission(session, dir / "config", "falcon9_crew_dragon_iss", session.epoch(), time);
    INFO(refused.refusal);
    CHECK_FALSE(refused.refusal.empty());
    CHECK(t.recorded().size() == events);
    CHECK(find_event(t, MissionEventKind::Liftoff));
    std::filesystem::remove_all(dir);
}

namespace {
constexpr double deg = constants::deg_to_rad;

// The ship alone; `propellant_kg` < 0: its landing propellant (as the entry tests).
config::VehicleConfig ship_alone(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    auto& ship = cfg.stages[0];
    config::fill_tanks(ship, propellant_kg < 0.0 ? ship.landing_propellant_kg : propellant_kg);
    for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    return cfg;
}

// A belly-first ship `offset_enu_m` from the Starbase pad (height in z) with `velocity_enu_mps`.
vehicle::RigidBodyState belly_state(const sim::FlightSession& session, const config::VehicleConfig& cfg, time::Epoch t0,
                                    const math::Vec3& offset_enu_m, const math::Vec3& velocity_enu_mps, double alpha_deg) {
    auto state = test::state_over_site(session.launch_site(), timeline_earth(), t0, offset_enu_m, velocity_enu_mps);
    const auto t = timeline_earth().fixed_to_gcrf(t0);
    const math::Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), alpha_deg * deg, 0.0,
                                                    cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    return state;
}

void set_starbase(sim::FlightSession& session) {
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
}

size_t count_of(const sim::MissionTimeline& t, MissionEventKind kind) {
    return static_cast<size_t>(std::count_if(t.recorded().begin(), t.recorded().end(),
                                             [&](const MissionEvent& e) { return e.kind == kind; }));
}
}  // namespace

// The hand-over to the entry guidance of test_lunar_return_entry.cpp (mission ship_lunar_return_llo, printed
// 2026-10-17): a direct lunar-return entry at 9,695 m/s that lofts at entry q.
constexpr double handover_epoch_tdb_s = 848008959.11018324;
constexpr double handover_propellant_kg = 42958.659955708943;
const math::Vec3 handover_position_gcrf_m{7086860.4675348997, 2531187.6710331743, 2929169.4508524211};
const math::Vec3 handover_velocity_gcrf_mps{-6563.1519895914944, 5826.099272684959, 1092.0146363209715};
const math::Quat handover_attitude{-0.55696594189169446, -0.50181461927849202, 0.23702032888641714,
                                   0.6178935111697279};
constexpr double handover_initial_bank_rad = 113.25 * deg;

TEST_CASE("A lunar-return entry's loft records no Max Q", "[timeline]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    t.session.set_boil_off(m.boil_off);
    t.session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    config::fill_tanks(ship.stages.front(), handover_propellant_kg);
    auto p = gnc::EntryGuidanceParameters::lunar_return();
    p.initial_bank_rad = handover_initial_bank_rad;
    const auto epoch = time::Epoch::from_tdb_seconds_past_j2000(handover_epoch_tdb_s);
    t.session.place_entering_ship(ship, {handover_position_gcrf_m, handover_velocity_gcrf_mps, handover_attitude, {}},
                                  epoch, p);
    // Fly until the ascent's Max Q rule would have fired on the pull-up: climbing at >= 10 kPa, then q 10 % below.
    double q_climbing_pa = 0.0;
    bool would_fire = false;
    double altitude_m = 1e9;
    while (!would_fire && !t.session.body(0).impacted() && t.session.epoch() - epoch < 3600.0 && altitude_m > 30e3) {
        t.session.advance_to(t.session.epoch() + 1.0, 400);
        const auto tm = t.session.telemetry(0);
        altitude_m = tm.altitude_m;
        if (tm.vertical_speed_mps > 0.0) q_climbing_pa = std::max(q_climbing_pa, tm.dynamic_pressure_pa);
        would_fire = q_climbing_pa >= sim::MissionEventObserver::max_q_min_pa &&
                     tm.dynamic_pressure_pa < sim::MissionEventObserver::max_q_drop_fraction * q_climbing_pa;
    }
    print_events(t.session.timeline());
    if (std::getenv("STARSIM_VERBOSE")) std::printf("  [loft] climbing q up to %.0f Pa\n", q_climbing_pa);
    CHECK(would_fire);
    CHECK(count_of(t.session.timeline(), MissionEventKind::MaxQ) == 0);
    CHECK(count_of(t.session.timeline(), MissionEventKind::EntryInterface) == 1);
}

TEST_CASE("A ship's flip and catch records one landing burn", "[timeline]") {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    set_starbase(session);
    const auto cfg = ship_alone(-1.0);  // its landing propellant
    // 28 km west of the pad, 26 km up, Mach ~2.6, 8 deg down (test_flight_session_entry: caught in the arms)
    const double speed = 780.0, gamma = -8.0 * deg;
    session.place_entering_ship(
        cfg, belly_state(session, cfg, t0, {-28000.0, 0.0, 26000.0}, {speed * std::cos(gamma), 0.0, speed * std::sin(gamma)},
                         65.0),
        t0);
    const auto& t = session.timeline();
    PredictionTrack landing_burn_prediction, catch_prediction;  // onboard predictions (Task 4b)
    int after_contact = 0, without_catch = 0;
    while (session.epoch() - t0 < 600.0 && session.landing_outcome(0) == sim::LandingOutcome::Flying) {
        session.advance_to(session.epoch() + 1.0, 1000);
        landing_burn_prediction.sample(predicted_of(t, MissionEventKind::LandingBurn), session.epoch());
        catch_prediction.sample(predicted_of(t, MissionEventKind::Caught), session.epoch());
        if (session.catch_contact_epoch(0)) {  // from the pins' first touch a catch dot is shown
            ++after_contact;
            if (!caught_shown(t, session.body_serial(0))) ++without_catch;
        }
    }
    session.advance_to(session.epoch() + 1.0, 1000);  // one more observation
    print_events(t);
    CHECK(count_of(t, MissionEventKind::LandingBurn) == 1);
    CHECK(count_of(t, MissionEventKind::Caught) == 1);
    CHECK(after_contact > 0);
    CHECK(without_catch == 0);

    // Onboard predictions: the gate (flip on the landing burn) from the entry guidance, the catch from the burn's plan.
    const auto* landing_burn = find_event(t, MissionEventKind::LandingBurn);
    const auto* caught = find_event(t, MissionEventKind::Caught);
    REQUIRE(landing_burn);
    REQUIRE(caught);
    print_prediction("landing burn", landing_burn_prediction, landing_burn);
    print_prediction("catch", catch_prediction, caught);
    REQUIRE(landing_burn_prediction.first);
    REQUIRE(catch_prediction.first);
    CHECK(*landing_burn_prediction.first_seen - t0 <= 1.0);  // from the entry guidance's first cycle
    // Measured 2026-10-22: the gate -0.83 s 179 s before, 0.07 s within the last 30 s; the catch (the plan's end vs the
    // pins' first touch on the arms): -2.9 s 18 s before, -1.3 s at the end.
    CHECK(landing_burn_prediction.max_error_within_s(landing_burn->epoch, 1e9) <= 1.0);
    CHECK(catch_prediction.max_error_within_s(caught->epoch, 1e9) <= 4.0);
    CHECK(t.predicted().empty());
}

TEST_CASE("Falcon 9 records MECO at the command, once, without the tail-off's cutoff", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    // The same deterministic flight twice: 0.5 s steps find the half second of the autopilot's MECO command (it ends the
    // booster phase: MECO, the staging sequence), the second flight steps through it tick by tick.
    std::optional<time::Epoch> window;
    {
        sim::FlightSession probe(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
        sim::start_mission(probe, STARSIM_CONFIG_DIR, "falcon9_rtls", t0, time);
        while (!window && probe.epoch() - t0 < 600.0) {
            const bool booster = probe.autopilot().state == sim::AutopilotState::Booster;
            const auto before = probe.epoch();
            probe.advance_to(before + 0.5, 1000);
            if (booster && probe.autopilot().state != sim::AutopilotState::Booster) window = before;
        }
    }
    REQUIRE(window);
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_rtls", t0, time);
    const auto& t = session.timeline();
    std::optional<time::Epoch> meco_command;  // the tick whose autopilot ended the booster phase
    double thrust_after_command_n = 0.0;      // the booster's thrust right after that tick: still tailing off
    std::optional<time::Epoch> meco_at_30;    // the onboard prediction at T+30 s (Task 4b)
    while (session.epoch() - t0 < 600.0 && (!meco_command || session.epoch() - *meco_command < 60.0)) {
        if (const auto* lifted = find_event(t, MissionEventKind::Liftoff);
            lifted && !meco_at_30 && session.epoch() - lifted->epoch >= 30.0) {
            if (const auto* p = predicted_of(t, MissionEventKind::Meco)) meco_at_30 = p->epoch;
        }
        const bool booster = session.autopilot().state == sim::AutopilotState::Booster;
        const auto before = session.epoch();
        const bool near = booster && before >= *window - 0.01;
        // one tick: 1.5 ticks ahead (a target exactly one tick ahead may round below it and run none)
        const double to_window_s = *window - before + 0.001;
        const double step = near ? 1.5 * sim::FlightSession::tick_s : to_window_s > 0.0 ? std::min(0.5, to_window_s) : 0.5;
        session.advance_to(before + step, 1000);
        if (booster && session.autopilot().state != sim::AutopilotState::Booster) {
            meco_command = before;
            thrust_after_command_n = session.telemetry(0).thrust_n;
        }
    }
    print_events(t);
    const auto* meco = find_event(t, MissionEventKind::Meco);
    const auto* separation = find_event(t, MissionEventKind::StageSeparation);
    REQUIRE(meco);
    REQUIRE(separation);
    REQUIRE(meco_command);
    CHECK(std::abs(meco->epoch - *meco_command) <= sim::FlightSession::tick_s);
    CHECK(meco->epoch <= separation->epoch);
    CHECK(meco < separation);  // listed first
    CHECK(thrust_after_command_n > 0.0);
    CHECK(meco->body_name == "Falcon 9 Block 5 first stage");
    if (std::getenv("STARSIM_VERBOSE") && meco_at_30) {
        std::printf("  [prediction] MECO at T+30 s: error %+.2f s\n", *meco_at_30 - meco->epoch);
    }
    REQUIRE(meco_at_30);
    // measured 2026-10-22: +7.35 s (the return's cutoff propellant grows; +9.8 s at most, +9 s in the bucket); +8.44 s
    // with the staging sequence (the 54.5 s MECO -> boostback of the RCS flip and boostback_loss_fraction 0.19)
    CHECK(std::abs(*meco_at_30 - meco->epoch) <= 9.0);
    const uint32_t booster = meco->body_serial;
    size_t mecos = 0;
    std::optional<time::Epoch> next_start;  // the booster's next burn
    for (const auto& e : t.recorded()) {
        if (e.body_serial != booster) continue;
        if (e.kind == MissionEventKind::Meco) ++mecos;
        const bool start = e.kind == MissionEventKind::Boostback || e.kind == MissionEventKind::Ignition ||
                           e.kind == MissionEventKind::LandingBurn;
        if (start && e.epoch > meco->epoch && !next_start) next_start = e.epoch;
    }
    CHECK(mecos == 1);
    for (const auto& e : t.recorded()) {  // the tail-off after the MECO is no event of its own
        if (e.body_serial == booster && e.kind == MissionEventKind::Cutoff && e.epoch > meco->epoch) {
            CHECK((next_start && e.epoch > *next_start));
        }
    }
}

TEST_CASE("Engines that fail without a command record a cutoff", "[timeline]") {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const double r = constants::wgs84_semi_major_axis_m + 300e3;
    const double v = std::sqrt(constants::earth_gm_m3ps2 / r);
    session.place_in_orbit("starship_block3", {{r, 0.0, 0.0}, {0.0, v, 0.0}}, t0, 1);  // the ship alone
    session.ignite();
    session.advance_to(t0 + 10.0, 100000);
    REQUIRE(session.telemetry(0).engines_running > 0);
    const auto& engines = session.body(0).vehicle().stages().front().engines;
    for (size_t k = 0; k < engines.size(); ++k) session.fail_engine(0, 0, k);
    session.advance_to(t0 + 20.0, 100000);
    print_events(session.timeline());
    CHECK(session.engines_commanded(0));  // nobody commanded the shutdown
    CHECK(session.telemetry(0).engines_running == 0);
    CHECK(count_of(session.timeline(), MissionEventKind::Cutoff) == 1);
}

namespace {
const MissionEvent* find_predicted(const sim::MissionTimeline& t, MissionEventKind kind) {
    for (const auto& e : t.predicted()) {
        if (e.kind == kind) return &e;
    }
    return nullptr;
}
}  // namespace

TEST_CASE("A scheduled liftoff waits on the pad and lifts off at T-0", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto e = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", e, time);  // on the pad, ascent autopilot engaged
    const auto t0 = e + 6.0 * 3600.0;
    session.schedule_liftoff(t0);
    const auto& t = session.timeline();
    REQUIRE(t.t0());
    CHECK(*t.t0() == t0);
    CHECK(*t.mission_clock_s(session.epoch()) == -6.0 * 3600.0);
    CHECK(session.liftoff_ignition_lead_s() == 3.5);  // Raptor 3 start-up 3.0 s + 0.5 s hold margin
    const double pad_altitude_m = session.telemetry(0).altitude_m;

    // the hold coasts: one call reaches its target with few ticks
    const auto hold_end = t0 - session.liftoff_ignition_lead_s() - sim::FlightSession::burn_preparation_s - 10.0;
    CHECK(session.advance_to(hold_end, 4000) == hold_end);
    CHECK(session.body(0).clamped());
    CHECK(std::abs(session.telemetry(0).altitude_m - pad_altitude_m) < 0.1);
    const auto* predicted = find_predicted(t, MissionEventKind::Liftoff);
    REQUIRE(predicted);
    CHECK(predicted->epoch == t0);
    CHECK(find_event(t, MissionEventKind::EngineStart) == nullptr);

    session.advance_to(t0 - 10.0, 1000000);
    CHECK(std::abs(session.telemetry(0).altitude_m - pad_altitude_m) < 0.1);
    session.advance_to(t0 + 20.0, 1000000);
    print_events(t);
    const auto* start = find_event(t, MissionEventKind::EngineStart);
    const auto* liftoff = find_event(t, MissionEventKind::Liftoff);
    REQUIRE(start);
    REQUIRE(liftoff);
    CHECK(liftoff->epoch - t0 >= 0.0);
    CHECK(liftoff->epoch - t0 <= 1.0);
    CHECK(start->epoch - t0 >= -3.5 - 0.1);
    CHECK(start->epoch - t0 <= 0.0);
    CHECK(session.telemetry(session.controlled_index()).altitude_m - pad_altitude_m > 10.0);
}

TEST_CASE("An ascent waiting on the pad counts down from the requested epoch", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto requested = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::StartOptions options;
    options.wait_on_pad = true;
    options.countdown_s = 600.0;  // starship_leo has no [mission] countdown_s
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", requested, time, options);
    CHECK(session.epoch() == requested);
    REQUIRE(session.timeline().t0());
    CHECK(*session.timeline().t0() == requested + 600.0);
    CHECK(*session.timeline().mission_clock_s(session.epoch()) == -600.0);
    CHECK(config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo").countdown_s == std::nullopt);
}

TEST_CASE("A countdown shorter than the engine start moves T-0, the clock never runs back", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto requested = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::StartOptions options;
    options.wait_on_pad = true;  // countdown_s 0
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", requested, time, options);
    const auto& t = session.timeline();
    REQUIRE(t.t0());
    CHECK(*t.t0() - requested == session.liftoff_ignition_lead_s());
    double last = *t.mission_clock_s(session.epoch());
    bool monotonic = true;
    while (session.epoch() - requested < 10.0) {
        session.advance_to(session.epoch() + 0.05, 1000);
        const double clock = *t.mission_clock_s(session.epoch());
        monotonic = monotonic && clock >= last;
        last = clock;
    }
    CHECK(monotonic);
    const auto* liftoff = find_event(t, MissionEventKind::Liftoff);
    REQUIRE(liftoff);
    CHECK(std::abs(liftoff->epoch - *t.t0()) <= sim::FlightSession::tick_s);
}

TEST_CASE("A window liftoff closer than the engine start places the vehicle earlier", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    sim::FlightSession probe(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto window = sim::start_mission(probe, STARSIM_CONFIG_DIR, "falcon9_crew_dragon_iss", {}, time);
    REQUIRE(window.launch_window);
    const auto liftoff = window.launch_window->liftoff;
    sim::StartOptions options;
    options.wait_on_pad = true;
    const auto st =
        sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_crew_dragon_iss", liftoff - 1.0, time, options);
    REQUIRE(st.return_planned);
    REQUIRE(session.timeline().t0());
    CHECK(*session.timeline().t0() == liftoff);  // the window is kept
    CHECK(liftoff - session.epoch() == session.liftoff_ignition_lead_s());
    CHECK(st.epoch == session.epoch());
}

TEST_CASE("An ISS launch waits only where the station source of its liftoff covers", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    sim::StartOptions options;
    options.wait_on_pad = true;
    const auto requested = time.from_utc({2026, 9, 1, 0, 0, 0.0});  // long before the archived OEM
    const auto st = sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_crew_dragon_iss", requested, time, options);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    REQUIRE(st.launch_window);
    const auto archive = sim::iss_archive(STARSIM_CONFIG_DIR, time);
    CHECK(session.epoch() - requested >= 0.0);
    if (const auto* oem = archive.for_epoch(st.launch_window->liftoff)) {  // the OEM: the wait starts at its coverage
        CHECK(session.epoch() == oem->header().usable_start);
    }
    CHECK(archive.for_epoch(session.epoch()) == archive.for_epoch(st.launch_window->liftoff));  // same file (or none)
    CHECK(session.epoch() - st.launch_window->liftoff < 0.0);
}

TEST_CASE("Disengaging on the pad drops the scheduled liftoff", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto requested = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::StartOptions options;
    options.wait_on_pad = true;
    options.countdown_s = 3600.0;
    sim::start_mission(session, STARSIM_CONFIG_DIR, "starship_leo", requested, time, options);
    session.advance_to(requested + 60.0, 4000);
    REQUIRE(find_predicted(session.timeline(), MissionEventKind::Liftoff));
    session.disengage_autopilot();
    session.advance_to(requested + 120.0, 40000);
    CHECK(find_predicted(session.timeline(), MissionEventKind::Liftoff) == nullptr);
    CHECK_FALSE(session.timeline().t0().has_value());
    CHECK(session.body(0).clamped());
}

TEST_CASE("An ISS launch can wait on the pad for its window", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto m = config::load_iss_launch_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon_iss");
    const auto requested = time.from_utc(m.earliest_utc);
    sim::StartOptions options;
    options.wait_on_pad = true;
    const auto st = sim::start_mission(session, STARSIM_CONFIG_DIR, "falcon9_crew_dragon_iss", requested, time, options);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    REQUIRE(st.launch_window);
    CHECK(session.epoch() == requested);
    REQUIRE(session.timeline().t0());
    CHECK(*session.timeline().t0() == st.launch_window->liftoff);
    CHECK(*session.timeline().mission_clock_s(session.epoch()) < 0.0);
    REQUIRE(session.station_index());
    CHECK(session.body(0).clamped());
}

namespace {
// The ship of ship_return_starbase on its start orbit (as test_ship_return_mission.cpp places it).
void place_ship_return(sim::FlightSession& session, const config::ShipReturnMission& m, time::Epoch t0) {
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
    while (!cfg.stages.empty() && cfg.stages.front().id != m.stage_id) cfg.stages.erase(cfg.stages.begin());
    REQUIRE(!cfg.stages.empty());
    config::fill_tanks(cfg.stages[0], m.propellant_kg);
    const double re = constants::wgs84_semi_major_axis_m;
    const double rp = re + m.periapsis_height_m, ra = re + m.apoapsis_height_m;
    const orbit::KeplerianElements elements{0.5 * (rp + ra), (ra - rp) / (ra + rp), m.inclination_rad, m.raan_rad, 0.0,
                                            m.argument_of_latitude_rad};
    const auto s = orbit::state_from_elements(elements, session.gm_earth_m3ps2());
    const math::Vec3 x = math::normalized(s.velocity_mps);
    const math::Vec3 z = math::normalized(s.position_m);
    const math::Vec3 y = cross(z, x);
    math::Mat3 r;
    const math::Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        r(0, c) = columns[c].x;
        r(1, c) = columns[c].y;
        r(2, c) = columns[c].z;
    }
    session.place_body(cfg, {s.position_m, s.velocity_mps, math::Quat::from_matrix(r), {}}, t0);
}
}  // namespace

TEST_CASE("Predicted events follow the plan", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
    place_ship_return(session, m, time.from_utc(m.epoch_utc));
    auto p = sim::ship_deorbit_parameters(m.deorbit);
    p.search_window_s = m.deorbit.search_window_s;
    REQUIRE(session.plan_ship_return(p));
    const auto first = *session.ship_return_plan();
    REQUIRE(first.node.epoch - session.epoch() > sim::FlightSession::deorbit_replan_age_s);
    session.advance_to(session.epoch() + 60.0, 100000);
    const auto& t = session.timeline();
    const auto check_plan = [&](const gnc::DeorbitPlan& plan) {
        const auto* ignition = find_predicted(t, MissionEventKind::Ignition);
        const auto* entry = find_predicted(t, MissionEventKind::EntryInterface);
        REQUIRE(ignition);
        REQUIRE(entry);
        // the ignition comes half the burn before the impulsive node
        CHECK(plan.node.epoch - ignition->epoch >= 0.0);
        CHECK(plan.node.epoch - ignition->epoch < 120.0);
        CHECK(std::abs(entry->epoch - plan.entry_interface_epoch) < 1e-6);
        CHECK(std::count_if(t.predicted().begin(), t.predicted().end(), [](const MissionEvent& e) {
                  return e.kind == MissionEventKind::Ignition;
              }) == 1);  // the pending burn stands for the plan's
    };
    check_plan(first);
    REQUIRE(t.t0());  // an orbit mission: T- until the deorbit burn
    CHECK(*t.mission_clock_s(session.epoch()) < 0.0);

    // the far plan is planned again deorbit_replan_lead_s before its burn: the predicted events follow it
    const auto replanned = first.node.epoch - sim::FlightSession::deorbit_replan_lead_s + 600.0;
    while (session.epoch() < replanned) session.advance_to(session.epoch() + 1800.0, 400000);
    REQUIRE(session.ship_return_plan());
    const auto second = *session.ship_return_plan();
    CHECK(second.entry_interface_epoch != first.entry_interface_epoch);
    check_plan(second);

    // while the deorbit burn is lit the entry interface stays predicted (the burn is the plan's node)
    while (!session.autopilot().burn.ignite && session.autopilot().state == sim::AutopilotState::Burn) {
        session.advance_to(session.epoch() + 10.0, 400000);
    }
    session.advance_to(session.epoch() + 2.0, 1000);
    REQUIRE(session.autopilot().state == sim::AutopilotState::Burn);
    REQUIRE(session.autopilot().burn.ignite);
    const auto* entry = find_predicted(t, MissionEventKind::EntryInterface);
    REQUIRE(entry);
    CHECK(std::abs(entry->epoch - second.entry_interface_epoch) < 1e-6);
    CHECK(find_predicted(t, MissionEventKind::Ignition) == nullptr);  // lit: no ignition ahead
}

TEST_CASE("A planned burn on cold gas records its ignition and cutoff", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const double r = constants::wgs84_semi_major_axis_m + 400e3;
    const math::StateVector orbit{{r, 0.0, 0.0}, {0.0, std::sqrt(session.gm_earth_m3ps2() / r), 0.0}};
    session.place_in_orbit("crew_dragon", orbit, t0);  // no main engines: the burn flies on the Dracos
    session.reset_timeline();
    const auto serial = session.body_serial(session.controlled_index());
    const auto& s = session.body(session.controlled_index()).state();
    const gnc::BurnNode node{t0 + 400.0, 0.5 * math::normalized(s.velocity_gcrf_mps)};
    REQUIRE(session.execute_burn(node));
    session.advance_to(t0 + 1.0, 1000);
    const auto* predicted = find_predicted(session.timeline(), MissionEventKind::Ignition);
    REQUIRE(predicted);
    const auto predicted_ignition = predicted->epoch;
    CHECK(predicted->body_name == "Crew Dragon capsule");  // the thrusters' part, not the trunk below it
    while (session.autopilot().state == sim::AutopilotState::Burn && session.epoch() - t0 < 1800.0) {
        session.advance_to(session.epoch() + 10.0, 100000);
    }
    const auto& t = session.timeline();
    print_events(t);
    REQUIRE(session.autopilot().state != sim::AutopilotState::Burn);
    CHECK(contains_in_order(t.recorded(), {MissionEventKind::Ignition, MissionEventKind::Cutoff}, serial));
    CHECK(count_of(t, MissionEventKind::Ignition) == 1);
    CHECK(count_of(t, MissionEventKind::Cutoff) == 1);
    const auto* ignition = find_event(t, MissionEventKind::Ignition);
    REQUIRE(ignition);
    CHECK(std::abs(ignition->epoch - predicted_ignition) < 1.0);
    CHECK(ignition->body_name == "Crew Dragon capsule");
    CHECK(node.epoch - ignition->epoch >= 0.0);
    CHECK(find_predicted(t, MissionEventKind::Ignition) == nullptr);
}

TEST_CASE("A capsule return predicts the capsule's entry interface, not the trunk's", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    sim::place_capsule_return_mission(session, m, config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id),
                                      STARSIM_CONFIG_DIR, time.from_utc(m.epoch_utc));
    REQUIRE(session.plan_capsule_return(sim::capsule_return_parameters(m)));
    const auto& t = session.timeline();
    const auto stack = session.body_serial(session.controlled_index());
    const auto start = session.epoch();
    session.advance_to(session.epoch() + 1.0, 100000);
    const auto* entry = predicted_of(t, MissionEventKind::EntryInterface);
    REQUIRE(entry);
    const auto predicted_serial = entry->body_serial;
    CHECK(predicted_serial != stack);  // the stack's serial stays with the trunk at the jettison
    CHECK(entry->body_name == "Crew Dragon capsule");
    // to the jettison: big steps to the burn, then on
    while (session.body_count() < 2 && session.epoch() - start < 3.0 * 86400.0) {
        double step = 10.0;
        if (session.autopilot().state == sim::AutopilotState::Burn && session.autopilot().burn.time_to_ignition_s > 900.0) {
            step = session.autopilot().burn.time_to_ignition_s - 600.0;
        }
        session.advance_to(session.epoch() + step, 1000000);
    }
    REQUIRE(session.body_count() == 2);
    CHECK(session.body_serial(session.controlled_index()) == predicted_serial);  // the capsule
    const auto* after = predicted_of(t, MissionEventKind::EntryInterface);
    REQUIRE(after);
    CHECK(after->body_serial == predicted_serial);
}

TEST_CASE("A cold-gas burn stopped by a disengage records its cutoff", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const double r = constants::wgs84_semi_major_axis_m + 400e3;
    const math::StateVector orbit{{r, 0.0, 0.0}, {0.0, std::sqrt(session.gm_earth_m3ps2() / r), 0.0}};
    session.place_in_orbit("crew_dragon", orbit, t0);
    session.reset_timeline();
    const auto& s = session.body(session.controlled_index()).state();
    REQUIRE(session.execute_burn({t0 + 400.0, 5.0 * math::normalized(s.velocity_gcrf_mps)}));  // ~40 s on the Dracos
    const auto& t = session.timeline();
    while (!find_event(t, MissionEventKind::Ignition) && session.epoch() - t0 < 900.0) {
        session.advance_to(session.epoch() + 1.0, 1000);
    }
    REQUIRE(find_event(t, MissionEventKind::Ignition));
    REQUIRE(session.autopilot().state == sim::AutopilotState::Burn);
    session.disengage_autopilot();
    CHECK(count_of(t, MissionEventKind::Cutoff) == 1);
    session.advance_to(session.epoch() + 10.0, 10000);
    CHECK(count_of(t, MissionEventKind::Cutoff) == 1);
}

TEST_CASE("The predicted undocking is the separation the mission records", "[timeline]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const auto st = sim::start_mission(session, STARSIM_CONFIG_DIR, "crew_dragon_iss_departure", {}, time);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    REQUIRE(session.iss_mission());
    const auto command = *session.iss_mission()->undock_epoch;
    session.advance_to(session.epoch() + 1.0, 1000);
    const auto& t = session.timeline();
    const auto* predicted = find_predicted(t, MissionEventKind::Undocking);
    REQUIRE(predicted);
    const auto predicted_separation = predicted->epoch;
    // the hooks open first
    CHECK(std::abs(predicted_separation - command - contact::DockingMechanismParameters{}.hooks_open_s) < 1e-3);
    while (session.epoch() - st.epoch < 3600.0 && !find_event(t, MissionEventKind::Undocking)) {
        session.advance_to(session.epoch() + 10.0, 10000000);
        if (session.docking_phase() == contact::DockingPhase::HooksOpening) {  // in progress: still the separation
            const auto* p = find_predicted(t, MissionEventKind::Undocking);
            REQUIRE(p);
            CHECK(std::abs(p->epoch - predicted_separation) < 0.1);
        }
    }
    const auto* undocking = find_event(t, MissionEventKind::Undocking);
    REQUIRE(undocking);
    CHECK(std::abs(undocking->epoch - predicted_separation) < 0.1);
    CHECK(find_predicted(t, MissionEventKind::Undocking) == nullptr);
}

TEST_CASE("Mission event probe", "[.][probe][timeline_probe]") {  // STARSIM_PROBE_MISSION, STARSIM_PROBE_S: prints the log
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const char* mission = std::getenv("STARSIM_PROBE_MISSION");
    const char* duration = std::getenv("STARSIM_PROBE_S");
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const auto st = sim::start_mission(session, STARSIM_CONFIG_DIR, mission ? mission : "starship_leo", t0, time);
    const auto start = session.epoch();
    const double limit_s = duration ? std::atof(duration) : 600.0;
    const char* step = std::getenv("STARSIM_PROBE_STEP_S");  // also samples the predicted events at this step
    const double step_s = step ? std::atof(step) : 60.0;
    std::map<std::pair<MissionEventKind, uint32_t>, PredictionTrack> predictions;
    while (session.epoch() - start < limit_s && session.body_count() > 0) {
        session.advance_to(session.epoch() + step_s, 10000000);
        for (const auto& e : session.timeline().predicted()) predictions[{e.kind, e.body_serial}].sample(&e, session.epoch());
    }
    for (const auto& e : session.timeline().recorded()) {
        std::printf("  [probe] T%+10.1f %-16s body %u %s\n", e.epoch - start, sim::to_string(e.kind), e.body_serial,
                    e.body_name.c_str());
        if (const auto it = predictions.find({e.kind, e.body_serial}); it != predictions.end()) {
            print_prediction(sim::to_string(e.kind), it->second, recorded_of(session.timeline(), e.kind, e.body_serial));
        }
    }
    std::printf("  [probe] flown %.0f s, leg %s: %s; %zu bodies\n", session.epoch() - start,
                sim::to_string(session.iss_mission_leg()), session.iss_mission_status().c_str(), session.body_count());
    (void)st;
}

// Cost of the predicted events per FlightSession::advance_to (final review, item 10): every advance_to call rebuilds the
// predicted list once; this times one extra predicted_mission_events call per step against the advance_to calls.
// STARSIM_PROBE_MISSION (starship_leo), STARSIM_PROBE_S (400), STARSIM_PROBE_STEP_S (1/60: a game frame).
TEST_CASE("Predicted events cost probe", "[.][probe][timeline_probe]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    const char* mission = std::getenv("STARSIM_PROBE_MISSION");
    const char* duration = std::getenv("STARSIM_PROBE_S");
    const char* step = std::getenv("STARSIM_PROBE_STEP_S");
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_mission(session, STARSIM_CONFIG_DIR, mission ? mission : "starship_leo", t0, time);
    const auto start = session.epoch();
    const double limit_s = duration ? std::atof(duration) : 400.0;
    const double step_s = step ? std::atof(step) : 1.0 / 60.0;
    using clock = std::chrono::steady_clock;
    double advance_ms = 0.0, predict_ms = 0.0;
    size_t calls = 0, events = 0;
    while (session.epoch() - start < limit_s && session.body_count() > 0) {
        const auto a = clock::now();
        session.advance_to(session.epoch() + step_s, 10000000);
        const auto b = clock::now();
        events += sim::predicted_mission_events(session).size();
        const auto c = clock::now();
        advance_ms += std::chrono::duration<double, std::milli>(b - a).count();
        predict_ms += std::chrono::duration<double, std::milli>(c - b).count();
        ++calls;
    }
    std::printf("  [probe] %zu advance_to calls: %.1f ms in advance_to, one predicted list %.4f ms (%.1f events) = "
                "%.2f %% of advance_to without it\n",
                calls, advance_ms, predict_ms / calls, static_cast<double>(events) / calls,
                100.0 * predict_ms / (advance_ms - predict_ms));
}

// The ship return from orbit (ship_return_starbase) to the catch - with STARSIM_PROBE_CAPSULE the Crew Dragon return to
// the splashdown -, sampling the predicted events every second near the events: prints the onboard predictions' errors
// against the recorded events (Task 4b measurements; STARSIM_VERBOSE=1).
TEST_CASE("Return prediction probe", "[.][probe][timeline_probe]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(timeline_ephemeris(), timeline_earth(), STARSIM_CONFIG_DIR);
    if (std::getenv("STARSIM_PROBE_CAPSULE")) {
        const auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
        sim::place_capsule_return_mission(session, m, config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id),
                                          STARSIM_CONFIG_DIR, time.from_utc(m.epoch_utc));
        REQUIRE(session.plan_capsule_return(sim::capsule_return_parameters(m)));
    } else {
        const auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
        place_ship_return(session, m, time.from_utc(m.epoch_utc));
        auto p = sim::ship_deorbit_parameters(m.deorbit);
        p.search_window_s = m.deorbit.search_window_s;
        REQUIRE(session.plan_ship_return(p));
    }
    const auto& t = session.timeline();
    std::map<std::pair<MissionEventKind, uint32_t>, PredictionTrack> predictions;
    const auto start = session.epoch();
    const auto flying = [&] {
        return session.body_count() > 0 &&
               session.landing_outcome(session.controlled_index()) == sim::LandingOutcome::Flying;
    };
    while (session.epoch() - start < 3.0 * 86400.0 && flying()) {
        const bool far = session.autopilot().state == sim::AutopilotState::Burn && !session.autopilot().burn.ignite &&
                         session.autopilot().burn.time_to_ignition_s > 120.0;
        session.advance_to(session.epoch() + (far ? 600.0 : 1.0), 10000000);
        for (const auto& e : t.predicted()) predictions[{e.kind, e.body_serial}].sample(&e, session.epoch());
    }
    session.advance_to(session.epoch() + 1.0, 1000);
    for (const auto& e : t.recorded()) {
        std::printf("  [probe] T%+10.1f %-16s body %u %s\n", e.epoch - start, sim::to_string(e.kind), e.body_serial,
                    e.body_name.c_str());
        if (const auto it = predictions.find({e.kind, e.body_serial}); it != predictions.end()) {
            print_prediction(sim::to_string(e.kind), it->second, &e);
        }
    }
}
