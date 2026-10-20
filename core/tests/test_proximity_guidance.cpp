// ISS i4: proximity guidance state machine on a CW point mass (fast; the session campaigns are in Task 6).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "proximity_test_support.hpp"

using namespace starsim;
using math::Vec3;
using gnc::ProximityPhase;
using proximity_test::CwSim;

namespace {
bool saw(const gnc::ProximityGuidance& g, ProximityPhase phase) {
    for (const auto& e : g.events()) if (e.phase == phase) return true;
    return false;
}
CwSim at_waypoint0(CwSim s) {
    s.cg = {{-5.0, 0.0, 397.0}, {0.01, 0.0, -0.01}};  // i3's Waypoint 0 arrival: a few metres, ~0.02 m/s
    return s;
}
}  // namespace

TEST_CASE("IDA-2: Waypoint 0 to contact inside the envelope", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida2());
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, {});
    const double kg0 = s.propellant_kg;
    const double t = proximity_test::run(g, s, 3.0 * 3600.0);
    REQUIRE(g.contact());
    std::printf("[validation] CW IDA-2: contact after %.1f min, %.1f kg, axial %.3f m/s, offset %.3f m, %s\n", t / 60.0,
                kg0 - s.propellant_kg, -g.contact()->port.velocity_mps.x,
                std::hypot(g.contact()->port.position_m.y, g.contact()->port.position_m.z),
                g.contact()->envelope.reason.c_str());
    CHECK(g.contact()->envelope.inside);
    CHECK(-g.contact()->port.velocity_mps.x < 0.1);
    CHECK_FALSE(saw(g, ProximityPhase::Hold));
    CHECK_FALSE(saw(g, ProximityPhase::Retreat));
    CHECK_FALSE(saw(g, ProximityPhase::Abort));
    CHECK(saw(g, ProximityPhase::Approach));
    CHECK(saw(g, ProximityPhase::FinalApproach));
    CHECK(t < 90.0 * 60.0);
}

TEST_CASE("IDA-3: fly-around to the zenith axis, contact inside the envelope", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida3());
    gnc::ProximityGuidance g(gnc::DockingApproach::RBar, {});
    const double kg0 = s.propellant_kg;
    const double t = proximity_test::run(g, s, 4.0 * 3600.0);
    REQUIRE(g.contact());
    std::printf("[validation] CW IDA-3: contact after %.1f min, %.1f kg, axial %.3f m/s, offset %.3f m, %s\n", t / 60.0,
                kg0 - s.propellant_kg, -g.contact()->port.velocity_mps.x,
                std::hypot(g.contact()->port.position_m.y, g.contact()->port.position_m.z),
                g.contact()->envelope.reason.c_str());
    CHECK(g.contact()->envelope.inside);
    CHECK(saw(g, ProximityPhase::Flyaround));
    CHECK_FALSE(saw(g, ProximityPhase::Abort));
}

TEST_CASE("Never inside the keep-out sphere outside the corridor", "[iss][proximity]") {
    for (auto s0 : {CwSim::ida2(), CwSim::ida3()}) {
        auto s = at_waypoint0(s0);
        gnc::ProximityGuidance g(s0.port_lever_lvlh.z < 0 ? gnc::DockingApproach::RBar : gnc::DockingApproach::VBar, {});
        for (double t = 0.0; t < 4.0 * 3600.0 && !g.contact(); t += 0.05) {
            const auto in = s.input();
            const auto out = g.update(in);
            // the contact step itself reaches the port plane (x <= 0), which the corridor counts as behind it
            if (!g.contact() && norm(in.cg_lvlh.position_m) < 200.0) {
                CHECK(gnc::corridor_status(in.port.position_m, norm(in.cg_lvlh.position_m), {}) ==
                      gnc::CorridorStatus::Inside);
            }
            s.step(out.translation_fraction_port, 0.05);
        }
        CHECK(g.contact());
    }
}

TEST_CASE("Corridor violation holds and resumes by itself", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;  // fly from Waypoint 2
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 5.0, 0.0}, {}};  // 20 m out, 5 m lateral: 14 deg > 10 deg, < 20 deg
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 3600.0);
    CHECK(saw(g, ProximityPhase::Hold));
    REQUIRE(g.contact());
    CHECK(g.contact()->envelope.inside);
}

TEST_CASE("A large corridor deviation retreats and waits for the go", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 15.0, 7.0, 0.0}, {}};  // 25 deg off the axis
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 1800.0);
    CHECK(saw(g, ProximityPhase::Retreat));
    CHECK(g.phase() == ProximityPhase::Waypoint);
    CHECK_FALSE(g.contact());  // waiting at Waypoint 2
    REQUIRE(g.resume());
    proximity_test::run(g, s, 1800.0);
    REQUIRE(g.contact());
    CHECK(g.contact()->envelope.inside);
}

TEST_CASE("Commanded hold, resume, abort and refusals", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    CHECK_FALSE(g.resume());  // nothing held
    proximity_test::run(g, s, 400.0);  // Waypoint 2 hold (300 s), then into the final approach
    REQUIRE(g.phase() == ProximityPhase::FinalApproach);
    REQUIRE(g.hold());
    CHECK_FALSE(g.hold());
    const Vec3 held = s.input().port.position_m;
    proximity_test::run(g, s, 300.0);
    CHECK(norm(s.input().port.position_m - held) < 0.5);
    CHECK(norm(s.input().port.velocity_mps) < 0.01);
    REQUIRE(g.resume());
    proximity_test::run(g, s, 30.0);
    REQUIRE(g.abort());
    CHECK_FALSE(g.abort());
    CHECK_FALSE(g.retreat());
    proximity_test::run(g, s, 600.0);
    CHECK(g.phase() == ProximityPhase::Abort);
    CHECK(g.status().find("Ausweichmanöver ausgeführt") != std::string::npos);
    // the breakout leaves the sphere and stays out for a day (CW)
    double closest_after_exit = 1e9;
    bool out = false;
    for (double t = 0.0; t < 86400.0; t += 10.0) {
        s.step({}, 10.0);
        const double r = norm(s.cg.position_m);
        out = out || r > 210.0;
        if (out) closest_after_exit = std::min(closest_after_exit, r);
    }
    CHECK(out);
    CHECK(closest_after_exit > 200.0);
}

TEST_CASE("Retreat is refused at Waypoint 0", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida2());
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, {});
    g.update(s.input());
    CHECK_FALSE(g.retreat());
    CHECK(g.phase() == ProximityPhase::Waypoint);
}

TEST_CASE("Propellant below the deorbit reserve aborts", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    s.propellant_kg = p.deorbit_reserve_kg + 0.5;
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 1200.0);
    CHECK(saw(g, ProximityPhase::Abort));
    CHECK_FALSE(g.contact());
}

// Final review i4 (fix pass): retreat, resume and refusals.

TEST_CASE("A commanded retreat on the axis just outside the sphere flies back axially", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida2());
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, {});
    bool reached = false;
    for (double t = 0.0; t < 3.0 * 3600.0; t += 0.05) {
        const auto in = s.input();
        const double r = norm(in.cg_lvlh.position_m);
        if (g.phase() == ProximityPhase::Approach && r > 200.0 && r < 210.0) {
            reached = true;
            break;
        }
        s.step(g.update(in).translation_fraction_port, 0.05);
    }
    REQUIRE(reached);
    REQUIRE(g.retreat());
    g.update(s.input());
    CHECK(g.phase() == ProximityPhase::Retreat);
    proximity_test::run(g, s, 1200.0);
    CHECK_FALSE(saw(g, ProximityPhase::Abort));
    CHECK(g.phase() == ProximityPhase::Waypoint);  // back at Waypoint 1, waiting for the go
}

TEST_CASE("A start just outside the keep-out sphere plans a transfer", "[iss][proximity]") {
    auto s = CwSim::ida2();
    s.cg = {{0.0, 0.0, 205.0}, {}};  // below the station, inside the 10 m transfer clearance
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, {});
    g.update(s.input());
    CHECK(g.phase() == ProximityPhase::Transfer);
}

TEST_CASE("A large deviation at an on-axis waypoint retreats once, not every tick", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 25.0, 10.0, 0.0}, {}};  // 22 deg off the axis beyond Waypoint 2
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 60.0);
    CHECK(saw(g, ProximityPhase::Retreat));
    CHECK(g.events().size() < 10);
}

TEST_CASE("Resume after an abort is checked against the live state", "[iss][proximity]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    g.update(s.input());
    REQUIRE(g.abort());
    g.update(s.input());
    REQUIRE(g.phase() == ProximityPhase::Abort);
    CHECK_FALSE(g.resume());  // breakout still being flown
    CHECK(g.refusal().find("Ausweichmanöver") != std::string::npos);
    proximity_test::run(g, s, 600.0);
    REQUIRE(g.status().find("ausgeführt") != std::string::npos);
    // drifted far away without guidance updates (the session coasts after a breakout)
    s.cg = {{-6000.0, 0.0, 0.0}, {}};
    g.observe(s.input());
    CHECK_FALSE(g.resume());
    CHECK(g.refusal().find("zu weit") != std::string::npos);
    // below the deorbit reserve: no resume (it would abort and fire a new breakout at once)
    s.cg = {{-1000.0, 0.0, 0.0}, {}};
    s.propellant_kg = p.deorbit_reserve_kg - 1.0;
    g.observe(s.input());
    CHECK_FALSE(g.resume());
    CHECK(g.refusal().find("Treibstoffreserve") != std::string::npos);
    s.propellant_kg = 2000.0;
    g.observe(s.input());
    CHECK(g.resume());
}

TEST_CASE("Refused commands give a German reason and leave the phase", "[iss][proximity]") {
    auto s = at_waypoint0(CwSim::ida2());
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, {});
    CHECK_FALSE(g.hold());
    CHECK(g.refusal().find("nicht gestartet") != std::string::npos);
    g.update(s.input());
    const std::string status = g.status();
    CHECK_FALSE(g.retreat());
    CHECK(g.refusal().find("Rückzug nicht möglich") != std::string::npos);
    CHECK_FALSE(g.resume());
    CHECK(g.refusal().find("Wiederaufnahme nicht möglich") != std::string::npos);
    REQUIRE(g.abort());
    CHECK_FALSE(g.abort());
    CHECK(g.refusal().find("Befehl") != std::string::npos);  // the abort is still pending
    g.update(s.input());
    CHECK_FALSE(g.abort());
    CHECK(g.refusal().find("Abbruch läuft") != std::string::npos);
    CHECK_FALSE(g.hold());
    CHECK(g.phase() == ProximityPhase::Abort);
    CHECK(status.find("Wegpunkt 0") != std::string::npos);
}

TEST_CASE("A failed capture retreats to Waypoint 2 and waits for resume", "[iss][proximity][docking]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    p.contact_plane_m = 0.7;  // the petal mesh plane of the docking interface (i5)
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 1800.0);
    REQUIRE(g.phase() == ProximityPhase::Contact);
    s.cg.velocity_mps = s.port_to_lvlh * Vec3{0.03, 0.0, 0.0};  // the ring bounced it back out
    g.capture_failed(s.input(), "abgeprallt");
    CHECK(g.phase() == ProximityPhase::Retreat);
    CHECK(g.status() == "Rückzug: Fangen fehlgeschlagen (abgeprallt)");
    proximity_test::run(g, s, 900.0);
    CHECK(g.phase() == ProximityPhase::Waypoint);
    CHECK(g.path()[g.waypoint()].name == "Wegpunkt 2");
    CHECK(g.waiting_after_capture_failure());  // review r5: an autonomous mission may resume this one
    REQUIRE(g.resume());
    CHECK_FALSE(g.waiting_after_capture_failure());
    proximity_test::run(g, s, 1800.0);
    CHECK(g.phase() == ProximityPhase::Contact);
    REQUIRE(g.contact());
    CHECK(g.contact()->envelope.inside);
}

TEST_CASE("A commanded retreat waits for resume, not as a failed capture", "[iss][proximity][refuelling]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    proximity_test::run(g, s, 60.0);
    REQUIRE(g.retreat());
    proximity_test::run(g, s, 3600.0);
    CHECK(g.waiting_for_go());
    CHECK_FALSE(g.waiting_after_capture_failure());  // the pilot's retreat stands (review r5)
}

TEST_CASE("A collision ends the guidance passively", "[iss][proximity][docking]") {
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    auto s = CwSim::ida2();
    s.cg = {{35.0 + 4.0 + 20.0, 0.0, 0.0}, {}};
    gnc::ProximityGuidance g(gnc::DockingApproach::VBar, p);
    g.update(s.input());
    g.collided(s.epoch);
    CHECK(g.phase() == ProximityPhase::Contact);
    CHECK(g.status() == "Kollision");
    CHECK(norm(g.update(s.input()).translation_fraction_port) == 0.0);
    CHECK_FALSE(g.resume());
}
