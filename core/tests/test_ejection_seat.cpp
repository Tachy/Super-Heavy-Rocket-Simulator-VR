// Crew escape (docs/superpowers/specs/2026-10-22-crew-escape-design.md), Task 2: the seat envelope and the seat
// sequencer (ACES II-class modes).

#include <algorithm>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/crew_escape_config.hpp"
#include "starsim/gnc/ejection_seat.hpp"

using namespace starsim;
using gnc::SeatMode;
using gnc::SeatPhase;

namespace {

const config::CrewEscapeConfig& crew() {
    static const auto c = config::load_crew_escape(STARSIM_CONFIG_DIR, "starship_crew");
    return c;
}

double q_of_eas(double eas_mps) { return 0.5 * 1.225 * eas_mps * eas_mps; }

}  // namespace

TEST_CASE("Seat envelope: belly flop inside, hypersonic and high outside", "[crew_escape][gnc]") {
    CHECK(gnc::equivalent_airspeed_mps(q_of_eas(100.0)) == Catch::Approx(100.0));
    CHECK(gnc::inside_seat_envelope(crew(), 3000.0, 5000.0));
    CHECK(gnc::inside_seat_envelope(crew(), 0.0, 0.0));  // zero-zero
    CHECK_FALSE(gnc::inside_seat_envelope(crew(), 20000.0, 1000.0));
    CHECK_FALSE(gnc::inside_seat_envelope(crew(), 5000.0, 60000.0));
}

TEST_CASE("Seat modes by speed and height", "[crew_escape][gnc]") {
    CHECK(gnc::seat_mode(crew(), 1000.0, q_of_eas(90.0)) == SeatMode::Mode1);
    CHECK(gnc::seat_mode(crew(), 1000.0, q_of_eas(200.0)) == SeatMode::Mode2);
    CHECK(gnc::seat_mode(crew(), 8000.0, q_of_eas(90.0)) == SeatMode::Mode3);
}

namespace {

struct Run {
    int drogues = 0, separations = 0, mains = 0;
    double drogue_t = -1.0, separation_t = -1.0, main_t = -1.0;
    double catapult_thrust = 0.0, rocket_thrust = 0.0, late_thrust = 0.0;
};

// Steps a sequencer at 200 Hz for `seconds` at a fixed height (or one falling at `sink_mps`).
Run run(SeatMode mode, double height_m, double seconds, double sink_mps = 0.0) {
    gnc::SeatSequencer s(crew(), mode);
    Run r;
    for (int i = 0; i <= static_cast<int>(seconds * 200.0); ++i) {
        const double t = i * 0.005;
        const auto a = s.update({t, height_m - sink_mps * t, q_of_eas(90.0), false});
        if (a.deploy_drogue && r.drogues++ == 0) r.drogue_t = t;
        if (a.separate && r.separations++ == 0) r.separation_t = t;
        if (a.deploy_main && r.mains++ == 0) r.main_t = t;
        if (i == 10) r.catapult_thrust = a.rail_thrust_n;  // 0.05 s
        if (i == 70) r.rocket_thrust = a.rail_thrust_n;    // 0.35 s
        if (i == 200) r.late_thrust = a.rail_thrust_n;     // 1.0 s
    }
    return r;
}

}  // namespace

TEST_CASE("Mode 1 sequence", "[crew_escape][gnc]") {
    const auto r = run(SeatMode::Mode1, 1000.0, 5.0);
    CHECK(r.catapult_thrust > 0.0);
    CHECK(r.rocket_thrust > 0.0);
    CHECK(r.late_thrust == 0.0);
    CHECK(r.mains == 1);
    // the main goes with the seat-man separation, at the mode's delay but not before the rocket burned out (the rocket
    // carries the seat with its occupant; nothing pushes the occupant alone)
    CHECK(r.main_t == Catch::Approx(std::max(crew().mode1_main_delay_s, crew().rocket.back().time_s)).margin(0.006));
    CHECK(r.drogues == 0);
    CHECK(r.separations == 1);
}

TEST_CASE("No rail thrust after the separation", "[crew_escape][gnc]") {
    gnc::SeatSequencer s(crew(), SeatMode::Mode2);
    bool separated = false;
    for (int i = 0; i <= 400; ++i) {
        const auto a = s.update({i * 0.005, 1000.0, q_of_eas(200.0), false});
        if (separated) CHECK(a.rail_thrust_n == 0.0);
        if (a.separate) separated = true;
    }
    CHECK(separated);
}

TEST_CASE("Mode 2 sequence", "[crew_escape][gnc]") {
    const auto r = run(SeatMode::Mode2, 1000.0, 5.0);
    CHECK(r.drogues == 1);
    CHECK(r.drogue_t == Catch::Approx(crew().mode2_drogue_delay_s).margin(0.006));
    CHECK(r.separations == 1);
    CHECK(r.separation_t == Catch::Approx(crew().mode2_separation_delay_s).margin(0.006));
    CHECK(r.mains == 1);
    CHECK(r.main_t == r.separation_t);
}

TEST_CASE("Mode 3 holds the drogue until the low-mode height", "[crew_escape][gnc]") {
    const auto high = run(SeatMode::Mode3, 8000.0, 5.0);  // stays high
    CHECK(high.drogues == 1);
    CHECK(high.separations == 0);
    const auto falling = run(SeatMode::Mode3, 4700.0, 10.0, 50.0);  // below 4,572 m after ~2.6 s
    CHECK(falling.separations == 1);
    CHECK(falling.separation_t > 2.5);
    CHECK(falling.separation_t < 2.7);
}

TEST_CASE("The seat sequence ends on the surface", "[crew_escape][gnc]") {
    gnc::SeatSequencer s(crew(), SeatMode::Mode1);
    s.update({0.0, 100.0, 0.0, false});
    s.update({3.0, 0.0, 0.0, false});
    CHECK(s.phase() == SeatPhase::Main);
    s.update({40.0, 0.0, 0.0, true});
    CHECK(s.phase() == SeatPhase::Down);
    CHECK(std::string(gnc::to_string(SeatPhase::Down)) == "gelandet");
}
