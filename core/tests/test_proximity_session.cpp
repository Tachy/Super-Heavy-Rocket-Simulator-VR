// ISS i4: proximity operations in the flight session - start_proximity, its refusals, the docking attitude and the
// clearing paths, and the validation campaigns (both ports from Waypoint 0, dispersed final approaches, abort, hold,
// a station reboost, time warp at a hold).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/sim/flight_session.hpp"
#include "proximity_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

// Runs the session in 10 s calls (from its current epoch) until contact, a completed breakout, `stop`, or `limit_s`
// seconds after t0; returns the seconds after t0. Every call checks the corridor: inside the keep-out sphere the chaser
// port must not be in the retreat zone. (The 10 s calls only sample; the guidance runs every tick, and the contact
// record is the exact crossing tick.)
double fly(proximity_test::DragonNearIss& d, double limit_s, bool check_corridor = true,
           const std::function<bool()>& stop = {}) {
    auto& s = d.session;
    const gnc::ProximityParameters p;
    double t = std::round(s.epoch() - d.t0);
    for (; t < limit_s; t += 10.0) {
        s.advance_to(d.t0 + t + 10.0, 1000000);  // t0 + elapsed, not epoch() + 10: keeps the calls deterministic
        if (s.proximity_contact()) break;
        const auto phase = s.proximity_phase();
        if (!phase || (*phase == gnc::ProximityPhase::Abort &&
                       s.proximity_status().find("ausgeführt") != std::string::npos)) {
            break;
        }
        if (check_corridor && *phase != gnc::ProximityPhase::Abort) {
            const auto rel = *s.relative_port_state();
            const double cg_range = norm(s.body(0).state().position_gcrf_m - s.body(1).state().position_gcrf_m);
            if (cg_range < p.keep_out_radius_m) {
                CHECK(gnc::corridor_status(rel.position_m, cg_range, p) != gnc::CorridorStatus::Retreat);
            }
        }
        if (stop && stop()) break;
    }
    return t + 10.0;
}

void print_events(const proximity_test::DragonNearIss& d) {
    for (const auto& e : *d.session.proximity_events()) {
        std::printf("[validation]   %6.1f min  %-10s %s\n", (e.epoch - d.t0) / 60.0, gnc::to_string(e.phase),
                    e.status.c_str());
    }
}

bool has_event(const proximity_test::DragonNearIss& d, gnc::ProximityPhase phase) {
    const auto& events = *d.session.proximity_events();
    return std::any_of(events.begin(), events.end(), [&](const auto& e) { return e.phase == phase; });
}

bool has_status(const proximity_test::DragonNearIss& d, const std::string& text) {
    const auto& events = *d.session.proximity_events();
    return std::any_of(events.begin(), events.end(),
                       [&](const auto& e) { return e.status.find(text) != std::string::npos; });
}

void print_contact(const char* label, const gnc::ProximityContact& c, double kg) {
    const auto& v = c.port.velocity_mps;
    std::printf("[validation] %s contact: axial %.3f m/s, lateral %.4f m/s, offset %.3f m, angle %.2f deg, rate %.3f "
                "deg/s, %.1f kg, %s\n",
                label, -v.x, std::hypot(v.y, v.z), std::hypot(c.port.position_m.y, c.port.position_m.z),
                c.port.misalignment_rad / deg, norm(c.port.angular_rate_radps) / deg, kg,
                c.envelope.inside ? "inside" : c.envelope.reason.c_str());
}

// The full approach from i3's measured Waypoint 0 arrival (LVLH (-5.0, 0.0, 396.9) m, 0.017 m/s).
void full_approach(const char* port, bool flyaround) {
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}));
    REQUIRE(d.session.start_proximity(port));
    const double kg0 = d.session.body(0).vehicle().rcs_propellant_kg();
    const double t = fly(d, 3.0 * 3600.0);
    std::printf("[validation] proximity to %s from Waypoint 0: %.1f min\n", port, t / 60.0);
    print_events(d);
    const auto c = d.session.proximity_contact();
    REQUIRE(c);
    const double kg = kg0 - d.session.body(0).vehicle().rcs_propellant_kg();
    print_contact(port, *c, kg);
    CHECK(c->envelope.inside);
    CHECK(-c->port.velocity_mps.x < 0.1);  // M9 criterion
    CHECK(kg < 150.0);                     // [E] bound, i3 reserve 900 kg incl. proximity
    CHECK(has_event(d, gnc::ProximityPhase::Flyaround) == flyaround);
    CHECK_FALSE(has_event(d, gnc::ProximityPhase::Abort));
    CHECK(d.session.autopilot().state == sim::AutopilotState::Off);  // contact ends the autopilot
}

gnc::ProximityParameters at_waypoint2(const std::string& port, double hold_s) {
    gnc::ProximityParameters p;
    p.waypoint2_hold_s = hold_s;
    const auto approach = port == "ida2" ? gnc::DockingApproach::VBar : gnc::DockingApproach::RBar;
    p.start_waypoint = gnc::proximity_path(approach, p).size() - 1;  // Waypoint 2 (IDA-3: after the fly-around points)
    return p;
}
}  // namespace

TEST_CASE("start_proximity refusals", "[iss][proximity][session]") {
    test::spice_session();
    {  // no station
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}), /*with_station=*/false);
        CHECK_FALSE(d.session.start_proximity("ida2"));
        CHECK(d.session.proximity_status().find("keine Station") != std::string::npos);
    }
    {  // too far: 8 km behind
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-8000, 0, 0}));
        CHECK_FALSE(d.session.start_proximity("ida2"));
        CHECK(d.session.proximity_status().find("zu weit") != std::string::npos);
    }
    {  // unknown port
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}));
        CHECK_FALSE(d.session.start_proximity("pma1"));
        CHECK_FALSE(d.session.proximity_active());
        CHECK(d.session.proximity_status().find("pma1") != std::string::npos);
    }
    {  // inside the sphere without an on-axis start
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 150}));
        CHECK_FALSE(d.session.start_proximity("ida2"));
        CHECK(d.session.proximity_status().find("Sicherheitszone") != std::string::npos);
    }
    {  // commands without a flight
        proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0, 0, 400}));
        CHECK_FALSE(d.session.proximity_hold());
        CHECK_FALSE(d.session.proximity_resume());
        CHECK_FALSE(d.session.proximity_abort());
        CHECK_FALSE(d.session.proximity_retreat());
        CHECK(d.session.proximity_status() == "Kein Anflug aktiv");
        CHECK_FALSE(d.session.proximity_phase());
        CHECK(d.session.proximity_events() == nullptr);
    }
}

TEST_CASE("start_proximity at Waypoint 0: docking attitude, cone open, ticking", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}), true, /*cone_open=*/false);
    auto& s = d.session;
    REQUIRE(s.start_proximity("ida2"));
    CHECK(s.body(0).vehicle().nose_cone_open());
    CHECK(s.autopilot().state == sim::AutopilotState::Proximity);
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
    s.advance_to(d.t0 + 240.0, 1000000);  // inside the 300 s hold
    const auto rel = *s.relative_port_state();
    CHECK(rel.misalignment_rad < 1.0 * deg);  // turned into the docking attitude
    CHECK(std::abs(rel.clocking_rad) < 1.0 * deg);
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
    CHECK(s.epoch() - d.t0 > 239.0);  // ticked through, not refused
}

TEST_CASE("Placing a body clears the proximity flight", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}));
    REQUIRE(d.session.start_proximity("ida2"));
    d.session.place_in_orbit(config::load_vehicle(proximity_test::config_dir, "crew_dragon"),
                             {d.session.body(0).state().position_gcrf_m, d.session.body(0).state().velocity_gcrf_mps},
                             d.session.epoch());
    CHECK_FALSE(d.session.proximity_active());
    CHECK(norm(d.session.translation_command()) == 0.0);
    CHECK(d.session.autopilot().state == sim::AutopilotState::Off);
}

TEST_CASE("Disengaging the autopilot ends the proximity flight", "[iss][proximity][session]") {
    test::spice_session();
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5, 0, 397}));
    REQUIRE(d.session.start_proximity("ida2"));
    d.session.disengage_autopilot();
    CHECK_FALSE(d.session.proximity_active());
    CHECK(norm(d.session.translation_command()) == 0.0);
}

TEST_CASE("Proximity to IDA-2 from Waypoint 0", "[iss][proximity][session][validation]") {
    test::spice_session();
    full_approach("ida2", false);
}

TEST_CASE("Proximity to IDA-3 with the fly-around", "[iss][proximity][session][validation]") {
    test::spice_session();
    full_approach("ida3", true);
}

TEST_CASE("Final approach campaign with dispersed Waypoint 2 states", "[iss][proximity][session][validation]") {
    test::spice_session();
    // [E] dispersions: lateral offset, velocity in the port frame (x > 0: receding), attitude about body y
    struct Dispersion {
        Vec3 offset_m, velocity_mps;
        double attitude_rad;
    };
    const Dispersion dispersions[] = {
        {{0.0, 0.5, 0.0}, {0.02, 0.01, 0.0}, 2.0 * deg},
        {{0.0, 0.0, -0.5}, {-0.02, 0.0, -0.01}, 2.0 * deg},
        {{0.0, 0.5, 0.0}, {-0.02, -0.01, 0.0}, -2.0 * deg},
        {{0.0, 0.0, -0.5}, {0.02, 0.0, 0.01}, -2.0 * deg},
    };
    std::printf("[validation] final approach campaign (Waypoint 2 hold 60 s)\n");
    for (const std::string port : {"ida2", "ida3"}) {
        int k = 0;
        for (const auto& dispersion : dispersions) {
            ++k;
            CAPTURE(port, k);
            const gnc::ProximityParameters p = at_waypoint2(port, 60.0);
            proximity_test::DragonNearIss d(proximity_test::at_port(
                port, Vec3{p.waypoint2_range_m, 0.0, 0.0} + dispersion.offset_m, dispersion.velocity_mps,
                math::Quat::from_axis_angle({0.0, 1.0, 0.0}, dispersion.attitude_rad)));
            REQUIRE(d.session.start_proximity(port, p));
            const double kg0 = d.session.body(0).vehicle().rcs_propellant_kg();
            const double t = fly(d, 1800.0);
            const auto c = d.session.proximity_contact();
            REQUIRE(c);
            char label[48];
            std::snprintf(label, sizeof label, "%s case %d (%.0f s)", port.c_str(), k, t);
            print_contact(label, *c, kg0 - d.session.body(0).vehicle().rcs_propellant_kg());
            CHECK(c->envelope.inside);
            CHECK(-c->port.velocity_mps.x < 0.1);
        }
    }
}

TEST_CASE("From the i3 Waypoint 0 state with the braking attitude", "[iss][proximity][session][validation]") {
    test::spice_session();
    // place_in_orbit points the nose along the velocity: 180 deg from the IDA-2 docking attitude, as the braking burn
    // (flown along +x against the closing velocity) leaves it; drifting 0.02 m/s
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({0.0, 0.0, 400.0}, {0.02, 0.0, 0.0}));
    auto& s = d.session;
    REQUIRE(s.start_proximity("ida2"));
    REQUIRE(s.relative_port_state()->misalignment_rad > 170.0 * deg);
    const gnc::ProximityParameters p;
    const auto cg_lvlh = [&] {
        const auto& iss = s.body(1).state();
        const auto& dragon = s.body(0).state();
        return orbit::lvlh_relative_state({iss.position_gcrf_m, iss.velocity_gcrf_mps},
                                          {dragon.position_gcrf_m, dragon.velocity_gcrf_mps});
    };
    double aligned_s = -1.0, worst_offset_m = 0.0;
    for (double t = 10.0; t <= p.waypoint0_hold_s - 10.0; t += 10.0) {
        s.advance_to(d.t0 + t, 1000000);
        if (aligned_s < 0.0 && s.relative_port_state()->misalignment_rad < 2.0 * deg) aligned_s = t;
        worst_offset_m = std::max(worst_offset_m, norm(cg_lvlh().position_m - Vec3{0.0, 0.0, p.waypoint0_below_m}));
        CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
    }
    std::printf("[validation] braking attitude at Waypoint 0: aligned (< 2 deg) after %.0f s, worst offset %.2f m\n",
                aligned_s, worst_offset_m);
    CHECK(aligned_s > 0.0);
    CHECK(worst_offset_m < 10.0);
    const double kg0 = s.body(0).vehicle().rcs_propellant_kg();
    fly(d, 3.0 * 3600.0);
    const auto c = s.proximity_contact();
    REQUIRE(c);
    print_contact("ida2 from the braking attitude", *c, kg0 - s.body(0).vehicle().rcs_propellant_kg());
    CHECK(c->envelope.inside);
}

TEST_CASE("A station reboost during the transfer is absorbed", "[iss][proximity][session][validation]") {
    test::spice_session();
    const gnc::ProximityParameters p;
    const spice::SpiceTimeConverter clock;
    const time::Epoch t0 = clock.from_utc({2026, 10, 16, 0, 0, 0.0});  // DragonNearIss::t0
    const time::Epoch boost = t0 + p.waypoint0_hold_s + 600.0;         // 600 s into the Waypoint 0 -> 1 transfer
    proximity_test::DragonNearIss d(proximity_test::lvlh_from_cg({-5.0, 0.0, 396.9}, {0.012, 0.0, -0.012}), true, true,
                                    {sim::StationImpulse{boost, 0.3, "Testanhebung"}});
    REQUIRE(d.session.start_proximity("ida2"));
    for (const auto& w : d.session.proximity_warnings()) std::printf("[validation] %s\n", w.c_str());
    CHECK(d.session.proximity_warnings().size() == 1);
    const double kg0 = d.session.body(0).vehicle().rcs_propellant_kg();
    fly(d, 3.0 * 3600.0);
    std::printf("[validation] station reboost 0.3 m/s during the transfer:\n");
    print_events(d);
    CHECK(has_status(d, "Wegpunkt 1"));
    CHECK_FALSE(has_event(d, gnc::ProximityPhase::Abort));
    const auto c = d.session.proximity_contact();
    REQUIRE(c);
    print_contact("ida2 with a reboost", *c, kg0 - d.session.body(0).vehicle().rcs_propellant_kg());
    CHECK(c->envelope.inside);
}

TEST_CASE("Time warp at a waypoint hold keeps ticking", "[iss][proximity][session]") {
    test::spice_session();
    gnc::ProximityParameters p;
    p.start_waypoint = 1;  // IDA-2: Waypoint 1, 220 m out on the axis
    proximity_test::DragonNearIss d(proximity_test::at_port("ida2", {p.waypoint1_range_m, 0.0, 0.0}));
    auto& s = d.session;
    REQUIRE(s.start_proximity("ida2", p));
    s.advance_to(d.t0 + 290.0, 1000000);  // one call over most of the 300 s hold
    CHECK(s.epoch() - d.t0 > 289.0);
    const auto rel = *s.relative_port_state();
    std::printf("[validation] Waypoint 1 hold after 290 s in one call: offset %.2f m\n",
                norm(rel.position_m - Vec3{p.waypoint1_range_m, 0.0, 0.0}));
    CHECK(norm(rel.position_m - Vec3{p.waypoint1_range_m, 0.0, 0.0}) < 2.0);
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Waypoint);
}

TEST_CASE("Abort from the final approach: breakout, passive drift outside the sphere",
          "[iss][proximity][session][validation]") {
    test::spice_session();
    const gnc::ProximityParameters p = at_waypoint2("ida2", 30.0);
    proximity_test::DragonNearIss d(proximity_test::at_port("ida2", {p.waypoint2_range_m, 0.0, 0.0}));
    auto& s = d.session;
    REQUIRE(s.start_proximity("ida2", p));
    fly(d, 1800.0, true, [&] { return s.relative_port_state()->position_m.x < 10.0; });
    REQUIRE(s.relative_port_state()->position_m.x < 10.0);
    REQUIRE_FALSE(s.proximity_contact());
    const auto before = *s.relative_port_state();
    REQUIRE(s.proximity_abort());
    const double t_abort = std::round(s.epoch() - d.t0);
    fly(d, t_abort + 3600.0, false);
    REQUIRE(s.proximity_status().find("Ausweichmanöver ausgeführt") != std::string::npos);
    const Vec3 dv = s.relative_port_state()->velocity_mps - before.velocity_mps;
    std::printf("[validation] abort at x %.1f m: breakout done after %.0f s, dv %.3f m/s (port frame %.3f %.3f %.3f), "
                "%s\n",
                before.position_m.x, s.epoch() - d.t0 - t_abort, norm(dv), dv.x, dv.y, dv.z,
                s.proximity_status().c_str());
    CHECK(s.proximity_status().find("nicht passiv sicher") == std::string::npos);
    // passive drift for 24 h (the session coasts: no attitude hold after the breakout)
    const auto cg_range = [&] { return norm(s.body(0).state().position_gcrf_m - s.body(1).state().position_gcrf_m); };
    bool left = cg_range() > p.keep_out_radius_m;
    double closest = std::numeric_limits<double>::infinity();
    const time::Epoch drift_start = s.epoch();
    for (double t = 600.0; t <= 86400.0; t += 600.0) {
        s.advance_to(drift_start + t, 1000000);
        const double r = cg_range();
        if (left) closest = std::min(closest, r);
        left = left || r > p.keep_out_radius_m;
    }
    std::printf("[validation] passive drift 24 h after the breakout: closest CG range %.0f m (sampled every 10 min)\n",
                closest);
    CHECK(left);
    CHECK(closest > p.keep_out_radius_m);
}

TEST_CASE("Hold and resume in the final approach", "[iss][proximity][session]") {
    test::spice_session();
    const gnc::ProximityParameters p = at_waypoint2("ida2", 30.0);
    proximity_test::DragonNearIss d(proximity_test::at_port("ida2", {p.waypoint2_range_m, 0.0, 0.0}));
    auto& s = d.session;
    REQUIRE(s.start_proximity("ida2", p));
    fly(d, 1800.0, true, [&] { return s.relative_port_state()->position_m.x < 12.0; });
    REQUIRE(s.relative_port_state()->position_m.x < 12.0);
    REQUIRE(s.proximity_hold());
    const Vec3 at_hold = s.relative_port_state()->position_m;
    const double t_hold = std::round(s.epoch() - d.t0);
    double worst_drift = 0.0;
    for (double t = 10.0; t <= 120.0; t += 10.0) {
        s.advance_to(d.t0 + t_hold + t, 1000000);
        worst_drift = std::max(worst_drift, norm(s.relative_port_state()->position_m - at_hold));
    }
    const auto rel = *s.relative_port_state();
    std::printf("[validation] hold at x %.2f m for 120 s: drift %.3f m, speed %.4f m/s, phase %s\n", at_hold.x,
                worst_drift, norm(rel.velocity_mps), gnc::to_string(*s.proximity_phase()));
    CHECK(*s.proximity_phase() == gnc::ProximityPhase::Hold);
    CHECK(norm(rel.velocity_mps) < 0.01);
    CHECK(worst_drift < 0.5);
    REQUIRE(s.proximity_resume());
    fly(d, t_hold + 1800.0);
    const auto c = s.proximity_contact();
    REQUIRE(c);
    print_contact("ida2 after hold + resume", *c, 0.0);
    CHECK(c->envelope.inside);
}
