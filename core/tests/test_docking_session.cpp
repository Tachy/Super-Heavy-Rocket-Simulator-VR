// ISS i5: docking in the flight session - capture, merge, docked coast, failed capture, collision, refusals, clearing.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "docking_test_support.hpp"
#include "starsim/sim/flight_session.hpp"

using namespace starsim;
using math::Vec3;
using contact::DockingPhase;

TEST_CASE("Docking session: a Dragon at rest in the rotating port frame stays at rest, no contact", "[iss][docking]") {
    // 0.5 m outside the petal mesh plane (0.30 + 2 x 0.20), zero velocity relative to the turning station port
    auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.40 + 0.5, 0.0, 0.0, 0.0);
    d.session.advance_to(d.t0 + 10.0, 100000);
    CHECK(d.session.docking_phase() == DockingPhase::Ready);
    const auto rel = d.session.relative_port_state();
    REQUIRE(rel);
    // the frame turns with the station (1.1e-3 rad/s); without an attitude hold the session's rate damping stops the
    // Dragon's orbital-rate turn, which moves its port (4 m lever) by ~4 mm/s - no contact force comes of it
    CHECK(norm(rel->velocity_mps) < 0.01);
    CHECK(std::abs(rel->position_m.x - 1.2) < 0.05);
    CHECK(norm(d.session.docking_wrench_balance_n()) == 0.0);
    CHECK_FALSE(d.session.collision_impulse_ns());
}

TEST_CASE("Docking session: IDA-2 at 0.075 m/s captures, merges and docks", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.40 + 0.1, 0.03, -0.075, 0.0);
    const double ms = d.station().vehicle().mass_properties().mass_kg;
    const double mc = d.dragon().vehicle().mass_properties().mass_kg;
    REQUIRE(d.fly_until(DockingPhase::SoftCapture, 60.0));
    CHECK(norm(d.session.docking_wrench_balance_n()) < 1e-9);
    CHECK(d.session.docking_busy());
    REQUIRE(d.fly_until(DockingPhase::Docked, 1200.0));
    CHECK(d.session.docked());
    CHECK(d.dragon().docked());
    const double mc_docked = d.dragon().vehicle().mass_properties().mass_kg;  // RCS used during the approach
    CHECK(mc_docked < mc);
    const auto& merge = d.session.docking_merge_record();
    CHECK(norm(merge.momentum_after - merge.momentum_before) < 1e-9 * norm(merge.momentum_before));
    CHECK(d.station().vehicle().mass_properties().mass_kg == Catch::Approx(ms + mc_docked));
    REQUIRE(d.session.compound_mass());
    CHECK(d.session.compound_mass()->mass_kg == Catch::Approx(ms + mc_docked));
    const auto rel = d.session.relative_port_state();
    REQUIRE(rel);
    CHECK(norm(rel->position_m) < 1e-6);  // held at the mated pose
    const auto loads = d.session.docking_loads();
    REQUIRE(loads);
    CHECK(loads->compression_n < 6500.0);
    const auto& ev = *d.session.docking_events();
    const double minutes = (ev.back().t_s - ev.front().t_s) / 60.0;
    CHECK(minutes > 8.0);
    CHECK(minutes < 13.0);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[docking] session IDA-2: first contact -> docked %.1f min, peak compression %.0f N, shear %.0f N\n",
                    minutes, loads->compression_n, loads->shear_n);
    }
}

TEST_CASE("Docking session: docked time warp keeps the Dragon on the port", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida3");
    REQUIRE(d.session.docked());
    const auto start = d.session.epoch();
    d.session.advance_to(start + 6.0 * 3600.0, 100000);
    CHECK(d.session.epoch() - start == Catch::Approx(6.0 * 3600.0).margin(1.0));  // coasted, not 100000 ticks
    const auto rel = d.session.relative_port_state();
    REQUIRE(rel);
    CHECK(norm(rel->position_m) < 0.001);
    CHECK(norm(rel->velocity_mps) < 1e-4);
}

TEST_CASE("Docking session: commands refused while docked", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida2");
    auto& s = d.session;
    REQUIRE(s.docked());
    CHECK_FALSE(s.start_proximity("ida2"));
    CHECK(s.proximity_status() == "Anflug nicht möglich: angedockt");
    CHECK_FALSE(s.plan_rendezvous({}));
    CHECK(s.rendezvous_status() == "Rendezvous nicht möglich: angedockt");
    CHECK_FALSE(s.plan_capsule_return());
    CHECK(s.docking_status() == "Rückkehr nicht möglich: angedockt");
    s.stage();
    CHECK(s.body_count() == 2);
    s.ignite();
    s.set_translation_command({1, 0, 0});
    s.advance_to(s.epoch() + 5.0, 10000);
    CHECK(norm(s.relative_port_state()->position_m) < 1e-6);
}

TEST_CASE("Docking session: placing new bodies clears the docked pair", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida2");
    REQUIRE(d.session.docked());
    const auto& st = d.station().state();
    const math::StateVector away{st.position_gcrf_m + Vec3{0.0, 0.0, 1.0e5}, st.velocity_gcrf_mps};
    d.session.place_in_orbit(config::load_vehicle(proximity_test::config_dir, "crew_dragon"), away, d.session.epoch());
    CHECK_FALSE(d.session.docked());
    CHECK_FALSE(d.session.docking_phase());
    CHECK_FALSE(d.session.station_index());
    CHECK_FALSE(d.session.compound_mass());
}

TEST_CASE("Docking session: a hull collision ends with Kollision and the impulse", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::beside_modules(1.0, 0.1);
    d.session.advance_to(d.t0 + 30.0, 100000);
    REQUIRE(d.session.collision_impulse_ns());
    CHECK(*d.session.collision_impulse_ns() > 0.0);
    CHECK(d.session.docking_status() == "Kollision");
}

TEST_CASE("Docking session: a too fast contact overloads the mechanism and rebounds", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.40 + 0.05, 0.0, -0.25, 0.0);
    REQUIRE(d.fly_until(DockingPhase::Failed, 60.0));
    CHECK(d.session.docking_status() == "Fangen fehlgeschlagen: Überlast");
    d.session.advance_to(d.session.epoch() + 10.0, 100000);
    CHECK(d.session.relative_port_state()->velocity_mps.x > 0.0);  // moving out again
    CHECK_FALSE(d.session.docked());
}

TEST_CASE("Docking session: a time warp does not coast through the station", "[iss][docking]") {
    // 5 m off the module surface at 0.05 m/s: one advance_to over 10 min must still see the hull contact
    auto d = docking_test::DragonAtPort::beside_modules(5.0, 0.05);
    d.session.advance_to(d.t0 + 600.0, 1000000);
    REQUIRE(d.session.collision_impulse_ns());
    CHECK(d.session.docking_status() == "Kollision");
}

TEST_CASE("Undocking: hooks open, springs separate at 0.085 m/s, momentum conserved, departure completes",
          "[iss][docking]") {
    auto d = docking_test::DragonAtPort::docked("ida2");
    auto& s = d.session;
    REQUIRE(s.docked());
    REQUIRE(s.undock());
    CHECK_FALSE(s.undock());
    CHECK(s.docking_status() == "Abkoppeln nicht möglich: Abkoppeln läuft");
    REQUIRE(d.fly_until(DockingPhase::Separated, 400.0));
    const auto& split = s.docking_separation_record();
    CHECK(split.relative_speed_mps == Catch::Approx(0.085).margin(0.003));
    CHECK(norm(split.momentum_after - split.momentum_before) < 1e-9 * norm(split.momentum_before));
    CHECK_FALSE(d.dragon().docked());
    CHECK_FALSE(s.compound_mass());
    CHECK(s.autopilot().state == sim::AutopilotState::Departure);
    const double kg0 = d.dragon().vehicle().rcs_propellant_kg();
    double t = std::round(s.epoch() - d.t0);
    const double t_separated = t;
    while (s.departure_phase() != gnc::DeparturePhase::Complete && t < 3.0 * 3600.0) {
        t += 10.0;
        s.advance_to(d.t0 + t, 1000000);
    }
    REQUIRE(s.departure_phase() == gnc::DeparturePhase::Complete);
    CHECK(s.autopilot().state == sim::AutopilotState::Off);
    CHECK_FALSE(s.collision_impulse_ns());
    const double kg = kg0 - d.dragon().vehicle().rcs_propellant_kg();
    CHECK(kg < 60.0);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[docking] undock IDA-2: separation %.4f m/s, departure complete %.1f min after the springs, %.1f kg\n",
                    split.relative_speed_mps, (t - t_separated) / 60.0, kg);
    }
}

TEST_CASE("Undocking: refused unless hard docked", "[iss][docking]") {
    auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.40 + 0.5, 0.0, 0.0, 0.0);
    CHECK_FALSE(d.session.undock());
    CHECK(d.session.docking_status() == "Abkoppeln nicht möglich: nicht fest angedockt");
}
