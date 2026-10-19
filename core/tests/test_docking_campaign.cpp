// ISS i5 validation campaigns: the full chain from Waypoint 0 to hard capture at both ports, the IDSS envelope grid in
// the session, the failure cases (overload, offset, angle) and the retreat / resume after a failed capture, a docked
// day with a station reboost, and undock -> departure -> capsule return to splashdown.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "docking_test_support.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/sim/capsule_return_mission.hpp"
#include "starsim/sim/flight_session.hpp"

using namespace starsim;
using math::Vec3;
using contact::DockingPhase;

namespace {
constexpr double deg = constants::deg_to_rad;
bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

// The i4 start: i3's measured Waypoint 0 arrival, the session's proximity flight to the port, then the docking.
struct ChainResult {
    bool docked = false;
    double flight_min = 0.0, dock_min = 0.0, kg = 0.0;
    std::optional<gnc::ProximityContact> contact;
    contact::LoadPeaks loads;
};
ChainResult full_chain(const std::string& port) {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}));
    auto& s = d.session;
    REQUIRE(s.start_proximity(port));
    const double kg0 = s.body(0).vehicle().rcs_propellant_kg();
    ChainResult r;
    double t = 0.0, contact_t = -1.0;
    for (; t < 4.0 * 3600.0 && !s.docked(); t += 10.0) {
        s.advance_to(d.t0 + t + 10.0, 1000000);
        if (contact_t < 0.0 && s.proximity_contact()) {
            contact_t = t + 10.0;
            r.contact = s.proximity_contact();
        }
        if (s.collision_impulse_ns()) break;
    }
    r.docked = s.docked();
    r.flight_min = contact_t / 60.0;
    r.dock_min = (t - contact_t) / 60.0;
    r.kg = kg0 - s.body(0).vehicle().rcs_propellant_kg();
    if (s.docking_loads()) r.loads = *s.docking_loads();
    if (r.docked) {  // one docked hour: the pair stays rigid
        s.advance_to(s.epoch() + 3600.0, 1000000);
        CHECK(norm(s.relative_port_state()->position_m) < 0.001);
    }
    return r;
}

// One session envelope case: the Dragon 5 mm before its first touch with the given initial contact conditions.
struct Icc {
    double axial_mps, offset_m, misalignment_rad, roll_rad, lateral_mps, rate_radps;
};
docking_test::DragonAtPort at_first_touch(const std::string& port, const Icc& c) {
    const contact::DockingInterfaceParameters ip;
    const double first_gap =
        std::max(0.0, (c.offset_m - ip.petal_clearance_m) / std::tan(ip.petal_lean_rad)) + 0.005 +
        ip.ring_extension_m * (1.0 - std::cos(c.misalignment_rad));  // the tilted ring sits a little lower
    // misalignment: pitch about the chaser's port y (body y); roll: about its docking axis (body x)
    const math::Quat attitude = math::Quat::from_axis_angle({0.0, 1.0, 0.0}, c.misalignment_rad) *
                                math::Quat::from_axis_angle({1.0, 0.0, 0.0}, c.roll_rad);
    auto d = docking_test::DragonAtPort::place(port, ip.ring_extension_m + first_gap, c.offset_m, -c.axial_mps,
                                               -c.lateral_mps, attitude);
    if (c.rate_radps > 0.0) {  // a pitch rate on top of the station's turn
        const auto& st = d.dragon().state();
        d.session.apply_angular_rate(0, st.angular_velocity_body_radps + Vec3{0.0, c.rate_radps, 0.0});
    }
    return d;
}
}  // namespace

TEST_CASE("Docking campaign: IDA-2 from Waypoint 0 to hard capture", "[iss][docking][proximity][slow]") {
    const auto r = full_chain("ida2");
    REQUIRE(r.contact);
    CHECK(r.contact->envelope.inside);
    CHECK(r.docked);
    CHECK(r.dock_min > 8.0);
    CHECK(r.dock_min < 13.0);
    CHECK(r.loads.compression_n < 6500.0);
    CHECK(r.loads.shear_n < 3200.0);
    if (verbose()) {
        std::printf("[docking] chain IDA-2: contact after %.1f min, docked %.1f min later, %.1f kg, peak comp %.0f N "
                    "shear %.0f N bending %.0f N m\n",
                    r.flight_min, r.dock_min, r.kg, r.loads.compression_n, r.loads.shear_n, r.loads.bending_nm);
    }
}

TEST_CASE("Docking campaign: IDA-3 from Waypoint 0 to hard capture", "[iss][docking][proximity][slow]") {
    const auto r = full_chain("ida3");
    REQUIRE(r.contact);
    CHECK(r.contact->envelope.inside);
    CHECK(r.docked);
    CHECK(r.loads.compression_n < 6500.0);
    CHECK(r.loads.shear_n < 3200.0);
    if (verbose()) {
        std::printf("[docking] chain IDA-3: contact after %.1f min, docked %.1f min later, %.1f kg, peak comp %.0f N "
                    "shear %.0f N bending %.0f N m\n",
                    r.flight_min, r.dock_min, r.kg, r.loads.compression_n, r.loads.shear_n, r.loads.bending_nm);
    }
}

TEST_CASE("Docking campaign: the IDSS envelope grid captures and docks at both ports", "[iss][docking][slow]") {
    // 2^(6-2) fractional factorial: A axial, B offset, C misalignment, D roll; E = ABC lateral, F = BCD pitch rate
    for (const std::string port : {"ida2", "ida3"}) {
        for (int k = 0; k < 16; ++k) {
            const int a = k & 1, b = (k >> 1) & 1, c = (k >> 2) & 1, dd = (k >> 3) & 1;
            const int e = a ^ b ^ c, f = b ^ c ^ dd;
            const Icc icc{a ? 0.10 : 0.05, b ? 0.10 : 0.0, c ? 4.0 * deg : 0.0, dd ? 4.0 * deg : 0.0, e ? 0.04 : 0.0,
                          f ? 0.2 * deg : 0.0};
            auto d = at_first_touch(port, icc);
            const bool docked = d.fly_until(DockingPhase::Docked, 1200.0);
            const auto loads = *d.session.docking_loads();
            INFO(port << " case " << k << ": " << d.session.docking_status());
            CHECK(docked);
            CHECK(loads.compression_n < 6500.0);
            CHECK(loads.shear_n < 3200.0);
            CHECK(loads.bending_nm < 2800.0);
            CHECK(loads.torsion_nm < 1500.0);
            if (verbose()) {
                std::printf("[docking] grid %s %2d: %.2f m/s %.2f m %.0f/%.0f deg %.2f m/s %.1f deg/s -> %s, comp %.0f N "
                            "shear %.0f N bend %.0f N m tors %.0f N m\n",
                            port.c_str(), k, icc.axial_mps, icc.offset_m, icc.misalignment_rad / deg, icc.roll_rad / deg,
                            icc.lateral_mps, icc.rate_radps / deg, docked ? "docked" : "NOT docked", loads.compression_n,
                            loads.shear_n, loads.bending_nm, loads.torsion_nm);
            }
        }
    }
}

TEST_CASE("Docking campaign: failures outside the envelope - offset and angle", "[iss][docking]") {
    {
        auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.40 + 0.05, 0.30, -0.075, 0.0);
        REQUIRE(d.fly_until(DockingPhase::Failed, 60.0));
        CHECK(d.session.docking_status() == "Fangen fehlgeschlagen: Versatz zu groß");
    }
    {
        const math::Quat tilt = math::Quat::from_axis_angle({0.0, 1.0, 0.0}, 10.0 * deg);
        auto d = docking_test::DragonAtPort::place("ida2", 0.30 + 0.05, 0.0, -0.075, 0.0, tilt);
        REQUIRE(d.fly_until(DockingPhase::Failed, 60.0));
        CHECK(d.session.docking_status() == "Fangen fehlgeschlagen: Winkel zu groß");
    }
}

TEST_CASE("Docking campaign: an overloading contact retreats to Waypoint 2, resume docks", "[iss][docking][proximity]") {
    // the final approach from Waypoint 2; at the guidance's "Kontakt" (the petal mesh plane, autopilot off, free
    // drift) a push makes it 0.28 m/s. (A push before it is braked by the "Halt: zu schnell" monitor.)
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::at_port("ida2", {20.0, 0.0, 0.0}));
    auto& s = d.session;
    gnc::ProximityParameters p;
    p.start_waypoint = 2;
    REQUIRE(s.start_proximity("ida2", p));
    REQUIRE(docking_test::DragonAtPort::wait_for(
        s, d.t0, [&] { return s.proximity_phase() == gnc::ProximityPhase::Contact; }, 1800.0, 0.1));
    const auto port = vehicle::port_kinematics(s.body(1), "ida2");
    s.apply_velocity_error(0, column(port.axes_gcrf, 0) * (-0.20));
    REQUIRE(docking_test::DragonAtPort::wait_for(s, d.t0, [&] { return s.docking_phase() == DockingPhase::Failed; }, 60.0));
    CHECK(s.docking_status() == "Fangen fehlgeschlagen: Überlast");
    REQUIRE(docking_test::DragonAtPort::wait_for(
        s, d.t0, [&] { return s.proximity_phase() == gnc::ProximityPhase::Waypoint; }, 1800.0));
    CHECK(s.proximity_status().find("Wegpunkt 2") != std::string::npos);
    CHECK(s.docking_phase() == DockingPhase::Ready);  // re-armed
    REQUIRE(s.proximity_resume());
    REQUIRE(docking_test::DragonAtPort::wait_for(s, d.t0, [&] { return s.docked(); }, 3600.0));
    CHECK_FALSE(s.collision_impulse_ns());
}

TEST_CASE("Docking campaign: a docked day with a station reboost keeps the pair rigid", "[iss][docking]") {
    // docked after ~10 min; a 0.3 m/s reboost 2 h after the start acts on the compound
    auto d = docking_test::DragonAtPort::docked("ida2", {{2.0 * 3600.0, 0.3}});
    auto& s = d.session;
    REQUIRE(s.docked());
    s.advance_to(d.t0 + 2.0 * 3600.0 - 1.0, 1000000);
    const double v_before = norm(d.dragon().state().velocity_gcrf_mps);
    s.advance_to(d.t0 + 2.0 * 3600.0 + 1.0, 1000000);
    CHECK(norm(d.dragon().state().velocity_gcrf_mps) - v_before == Catch::Approx(0.3).margin(0.01));  // moves along
    s.advance_to(d.t0 + 24.0 * 3600.0, 1000000);
    CHECK(norm(s.relative_port_state()->position_m) < 0.001);
    CHECK(norm(s.relative_port_state()->velocity_mps) < 1e-4);
}

TEST_CASE("Docking campaign: undock, departure and capsule return to splashdown", "[iss][docking][slow]") {
    auto d = docking_test::DragonAtPort::docked("ida2");
    auto& s = d.session;
    REQUIRE(s.undock());
    REQUIRE(docking_test::DragonAtPort::wait_for(
        s, d.t0, [&] { return s.departure_phase() == gnc::DeparturePhase::Complete; }, 3.0 * 3600.0, 10.0));
    const double departure_kg = 2562.0 - s.body(0).vehicle().rcs_propellant_kg();
    const auto mission = config::load_capsule_return_mission(proximity_test::config_dir, "crew_dragon_return");
    s.set_splashdown_zone(config::load_splashdown_zone(proximity_test::config_dir, mission.zone_id));
    // the real undocking is timed to a return opportunity (i6); here the onboard computer searches a week of passes
    auto parameters = sim::capsule_return_parameters(mission);
    parameters.search_window_s = 7.0 * 86400.0;
    const bool planned = s.plan_capsule_return(parameters);
    INFO((s.ship_return_plan() ? s.ship_return_plan()->reason : std::string("kein Plan")) << " / " << s.docking_status());
    REQUIRE(planned);
    const double planned_h = (s.ship_return_plan()->node.epoch - s.epoch()) / 3600.0;
    const time::Epoch start = s.epoch();
    while (s.capsule_return_step() != sim::CapsuleReturnStep::SplashedDown &&
           s.capsule_return_step() != sim::CapsuleReturnStep::Lost && s.epoch() - start < 8.0 * 86400.0) {
        double step = 60.0;
        if (s.autopilot().state == sim::AutopilotState::Burn && s.autopilot().burn.time_to_ignition_s > 900.0) {
            step = s.autopilot().burn.time_to_ignition_s - 600.0;
        }
        s.advance_to(s.epoch() + step, 10000000);
    }
    CHECK(s.capsule_return_step() == sim::CapsuleReturnStep::SplashedDown);
    const double rcs_left = s.body(s.controlled_index()).vehicle().rcs_propellant_kg();
    CHECK(rcs_left > 0.0);
    CHECK(departure_kg < 30.0);
    if (verbose()) {
        std::printf("[docking] undock -> splashdown: departure %.1f kg, deorbit burn %.1f h after the departure, RCS "
                    "left %.0f kg\n",
                    departure_kg, planned_h, rcs_left);
    }
}
