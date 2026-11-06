// ISS i5: IDSS docking mechanism - capture inside the envelope, failures outside, timeline, loads, undocking.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "report_check.hpp"
#include "starsim/contact/docking_mechanism.hpp"
#include "starsim/math/constants.hpp"
#include "webcast_report.hpp"

using namespace starsim;
using math::Mat3;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
const Mat3 mated = Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});

// Chaser rigid body reduced to its port point (lever folded into the rotational inertia): translational state of the
// port and a small-angle attitude about the passive y / z / x axes. Fixed passive port. dt 5 ms as the session.
struct PortModel {
    double mass_kg = 12000.0;
    double inertia_kg_m2 = 60000.0;  // about the port point [IDSS-15T class + m l^2 with the lever folded in]
    Vec3 p, v, angle, rate;          // angle: small rotation vector of the chaser axes from mated
    contact::DockingPose pose() const {
        // small-angle rotation applied to the mated axes
        const Mat3 r = Mat3::from_columns({1, angle.z, -angle.y}, {-angle.z, 1, angle.x}, {angle.y, -angle.x, 1});
        return {p, v, r * mated, rate};
    }
    void step(const contact::DockingWrench& w, double dt) {
        v += w.force_port_n / mass_kg * dt;
        const Vec3 lever = w.point_port_m - p;
        rate += (w.torque_port_nm + cross(lever, w.force_port_n)) / inertia_kg_m2 * dt;
        p += v * dt;
        angle += rate * dt;
    }
};

struct Run {
    contact::DockingMechanism m;
    PortModel c;
    double t = 0.0;
    void fly(double seconds) {
        const double dt = 0.005;
        for (double end = t + seconds; t < end; t += dt) c.step(m.update(t, c.pose(), c.mass_kg, c.inertia_kg_m2, dt), dt);
    }
};

// The IDSS initial contact conditions hold at the first physical touch (IDSS IDD E 3.3.1.1): the chaser starts 5 mm
// before it - on the petal cone for an offset beyond the meshed clearance, on the ring face otherwise; offsets beyond
// the capture radius start 5 cm before the petal tips.
Run approach(double axial_mps, double offset_m, double angle_rad = 0.0, double lateral_mps = 0.0, double lean_rad = -1.0) {
    contact::DockingMechanismParameters p;
    if (lean_rad > 0.0) p.interface.petal_lean_rad = lean_rad;
    Run r{contact::DockingMechanism(p), {}, 0.0};
    const auto& ip = p.interface;
    const double tips = 2.0 * ip.petal_height_m;
    const double capture = ip.petal_clearance_m + tips * std::tan(ip.petal_lean_rad);
    const double first_gap = offset_m > capture ? tips + 0.05
                                                : std::max(0.0, (offset_m - ip.petal_clearance_m) / std::tan(ip.petal_lean_rad)) +
                                                      0.005;
    r.c.p = {ip.ring_extension_m + first_gap, offset_m, 0.0};
    r.c.v = {-axial_mps, lateral_mps, 0.0};
    r.c.angle = {0.0, angle_rad, 0.0};
    return r;
}
}  // namespace

TEST_CASE("Docking mechanism: nominal contact captures, retracts and hard-captures in ~11 min", "[iss][docking]") {
    auto r = approach(0.075, 0.03);
    r.fly(20.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::SoftCapture);
    r.fly(900.0);
    CHECK(r.m.phase() == contact::DockingPhase::Docked);
    CHECK(r.m.hard_captured());
    const auto& ev = r.m.events();
    const double soft = ev.front().t_s;  // first contact ~ soft capture
    const double hard = ev.back().t_s;
    CHECK(hard - soft > 9.0 * 60.0);
    CHECK(hard - soft < 13.0 * 60.0);  // Demo-2: 11 min
    CHECK(norm(r.c.p) < 0.005);
    CHECK(r.m.peaks().compression_n < 6500.0);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[docking] nominal: first contact -> hard capture %.1f min, peak compression %.0f N, shear %.0f N\n",
                    (hard - soft) / 60.0, r.m.peaks().compression_n, r.m.peaks().shear_n);
    }
}

TEST_CASE("Docking mechanism: every corner of the IDSS envelope captures within the load limits", "[iss][docking]") {
    for (double axial : {0.05, 0.10}) {
        for (double offset : {0.0, 0.10}) {
            for (double angle : {0.0, 4.0 * deg}) {
                for (double lateral : {0.0, -0.04}) {
                    auto r = approach(axial, offset, angle, lateral);
                    r.fly(60.0);
                    INFO("axial " << axial << " offset " << offset << " angle " << angle << " lateral " << lateral << " -> "
                         << report::id_of(r.m.failure().code));
                    CHECK(r.m.phase() == contact::DockingPhase::SoftCapture);
                    CHECK(r.m.peaks().compression_n < 6500.0);
                    CHECK(r.m.peaks().shear_n < 3200.0);
                    if (std::getenv("STARSIM_VERBOSE")) {
                        std::printf("[docking] corner %.2f m/s %.2f m %.1f deg %.2f m/s: compression %.0f N shear %.0f N "
                                    "bending %.0f N m\n",
                                    axial, offset, angle / deg, lateral, r.m.peaks().compression_n, r.m.peaks().shear_n,
                                    r.m.peaks().bending_nm);
                    }
                }
            }
        }
    }
}

TEST_CASE("Docking mechanism: too fast overloads, the latches release, the chaser rebounds", "[iss][docking]") {
    auto r = approach(0.25, 0.0);
    r.fly(60.0);
    CHECK(r.m.phase() == contact::DockingPhase::Failed);
    CHECK(r.m.failure().code == report::Code::docking_fail_overload);
    CHECK(r.c.v.x > 0.0);  // moving out again
}

TEST_CASE("Docking mechanism: an offset beyond the petals' capture radius bounces off the tips", "[iss][docking]") {
    auto r = approach(0.075, 0.45);
    r.fly(60.0);
    CHECK(r.m.phase() == contact::DockingPhase::Failed);
    CHECK(r.m.failure().code == report::Code::docking_fail_offset);
    CHECK(r.c.v.x > 0.0);
}

TEST_CASE("Docking mechanism: inside the capture radius a large offset is pushed back out by the petal cone", "[iss][docking]") {
    // 0.30 m is inside the 0.41 m, but the chaser is not stopped by the platform spring: measured (scan below) it
    // reaches a peak deflection of only about 0.13 m (about 17 J of the 34 J), the 45 deg cone turns its axial speed
    // into lateral speed, it crosses the axis, strikes the opposite side of the cone and is reflected out (a V, not
    // an energy barrier).
    auto r = approach(0.075, 0.30);
    r.fly(60.0);
    CHECK(r.m.phase() == contact::DockingPhase::Failed);
    CHECK(r.m.failure().code == report::Code::docking_fail_bounced);
    CHECK(r.c.v.x > 0.0);
}

TEST_CASE("Docking mechanism: IDSS envelope corner with outward lateral speed (report)",
          "[nightly][webcast][iss][docking]") {
    // KNOWN STATE (docs/known-issues.md): 0.05 m/s axial, 0.10 m offset, +0.04 m/s lateral AWAY from the axis is inside the
    // IDSS initial-condition envelope (lateral speed is limited by magnitude), yet the model does not capture it, at
    // either lean (measured: both "abgeprallt"). Pre-existing at 30 deg, not caused by the 45 deg lean. Not tuned: a
    // report row until the undamped platform [E] is revisited (user 2026-11-03: the pin became a report; it no longer
    // fails when the state changes).
    std::vector<test::WebcastRow> rows;
    for (double lean_deg : {30.0, 45.0}) {
        auto r = approach(0.05, 0.10, 0.0, 0.04, lean_deg * deg);
        r.fly(60.0);
        INFO("lean " << lean_deg << " -> " << report::id_of(r.m.failure().code));
        const bool captured = r.m.phase() != contact::DockingPhase::Failed;
        rows.push_back({"captured at 0.05 m/s axial, 0.10 m offset, +0.04 m/s lateral, petal lean " + std::to_string(static_cast<int>(lean_deg)) +
                            " deg (1 = captured; failure: '" + std::string(report::id_of(r.m.failure().code)) + "')",
                        captured ? 1.0 : 0.0, 1.0, 0.0, "IDSS initial-condition envelope", "ring platform damping [E], petal lean, petal geometry"});
    }
    test::webcast_report("docking_idss_envelope_corner", rows);
}

TEST_CASE("Docking mechanism: rearm after a failure, then a good approach captures", "[iss][docking]") {
    auto r = approach(0.075, 0.30);
    r.fly(60.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::Failed);
    r.m.rearm();
    CHECK(r.m.phase() == contact::DockingPhase::Ready);
    const contact::DockingMechanismParameters p;
    r.c.p = {p.interface.ring_extension_m + 2.0 * p.interface.petal_height_m + 0.05, 0.02, 0.0};
    r.c.v = {-0.075, 0.0, 0.0};
    r.c.rate = {};
    r.fly(30.0);
    CHECK(r.m.phase() == contact::DockingPhase::SoftCapture);
}

TEST_CASE("Docking mechanism: undock opens the hooks, then the separation is due once", "[iss][docking]") {
    auto r = approach(0.075, 0.0);
    r.fly(900.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::Docked);
    CHECK_FALSE(r.m.separation_due());
    REQUIRE(r.m.undock(r.t));
    CHECK_FALSE(r.m.undock(r.t));  // second command refused
    r.fly(contact::DockingMechanismParameters{}.hooks_open_s + 1.0);
    CHECK(r.m.phase() == contact::DockingPhase::Separated);
    CHECK(r.m.separation_due());
    CHECK_FALSE(r.m.separation_due());
}

TEST_CASE("Docking mechanism: undock refused before hard capture", "[iss][docking]") {
    auto r = approach(0.075, 0.0);
    r.fly(20.0);
    REQUIRE(r.m.phase() == contact::DockingPhase::SoftCapture);
    CHECK_FALSE(r.m.undock(r.t));
    CHECK(test::refused(r.m.refusal(), report::Code::docking_undock_not_possible, report::Code::docking_cause_not_hard_docked));
}

TEST_CASE("Docking mechanism: separation speed from the IDSS spring energy", "[iss][docking]") {
    const contact::DockingMechanismParameters p;
    // Dragon 12 t on the 460 t ISS: v_rel = sqrt(2 E / mu)
    const double mu = 12000.0 * 460320.0 / (12000.0 + 460320.0);
    CHECK(contact::separation_relative_speed_mps(p, 12000.0, 460320.0) == Catch::Approx(std::sqrt(2.0 * 43.35 / mu)));
    CHECK(contact::separation_relative_speed_mps(p, 12000.0, 460320.0) == Catch::Approx(0.0862).margin(0.002));
}

TEST_CASE("Docking mechanism: start_docked enters Docked from Ready only", "[docking]") {
    contact::DockingMechanism m;
    REQUIRE(m.start_docked(5.0));
    CHECK(m.phase() == contact::DockingPhase::Docked);
    CHECK(m.hard_captured());
    CHECK(m.ring_extension_m() == 0.0);  // retracted, as after a capture
    CHECK_FALSE(m.start_docked(6.0));
    CHECK_FALSE(m.refusal().ok());
    CHECK(m.undock(10.0));  // the docked state undocks as after a capture
    CHECK(m.phase() == contact::DockingPhase::HooksOpening);
}

// Measurements for docs/subsystems/iss.md (STARSIM_VERBOSE prints them): hidden, run with "[.scan]".
namespace {
bool captures(double axial, double offset, double lean, double lateral = 0.0) {
    auto r = approach(axial, offset, 0.0, lateral, lean);
    r.fly(60.0);
    return r.m.phase() == contact::DockingPhase::SoftCapture;
}
}  // namespace

TEST_CASE("Docking mechanism scan: largest captured offset by speed and lean, bounce mechanism, latch state",
          "[.scan]") {
    for (double lean_deg : {30.0, 45.0}) {
        for (double axial : {0.05, 0.075, 0.10}) {
            double best = 0.0;
            for (double o = 0.02; o < 0.44; o += 0.005) {
                if (captures(axial, o, lean_deg * deg)) best = o;
            }
            std::printf("[scan] lean %.0f axial %.3f largest captured offset (0.005 grid) %.3f m\n", lean_deg, axial, best);
        }
    }
    for (double lean_deg : {30.0, 45.0}) {
        auto r = approach(0.075, 0.30, 0.0, 0.0, lean_deg * deg);
        double min_x = 1e9, max_abs_y = 0.0, y_at_cross = 0.0, vx_exit = 0.0, x0 = r.c.p.x;
        bool crossed = false;
        double peak_defl = 0.0, peak_defl_energy_j = 0.0;
        for (int i = 0; i < 12000; ++i) {
            const double y_before = r.c.p.y;
            r.fly(0.005);
            min_x = std::min(min_x, r.c.p.x);
            {   // platform deflection = offset - clearance(gap) while the ring is in the cone
                const contact::DockingInterfaceParameters ip;
                const double gap = r.c.p.x - ip.ring_extension_m;
                const double defl = std::hypot(r.c.p.y, r.c.p.z) - (ip.petal_clearance_m + std::max(gap, 0.0) * std::tan(lean_deg * deg));
                if (defl > peak_defl) { peak_defl = defl; peak_defl_energy_j = 0.5 * ip.ring_lateral_stiffness_n_per_m * defl * defl; }
            }
            max_abs_y = std::max(max_abs_y, std::fabs(r.c.p.y));
            if (!crossed && y_before * r.c.p.y < 0.0) { crossed = true; y_at_cross = r.c.p.x; }
        }
        vx_exit = r.c.v.x;
        std::printf("[scan] 0.30 m @0.075 lean %.0f: %s, min x %.3f (start %.3f, travel %.3f), max |y| %.3f, crossed axis %d at x %.3f, "
                    "exit vx %+.3f, peak deflection %.3f m (spring work %.1f J), chaser KE at start %.1f J\n", lean_deg,
                    report::id_of(r.m.failure().code).data(), min_x, x0, x0 - min_x, max_abs_y, crossed, y_at_cross, vx_exit, peak_defl,
                    peak_defl_energy_j, 0.5 * 12000.0 * 0.075 * 0.075);
    }
    for (double lean_deg : {30.0, 45.0}) {
        for (double lat : {-0.04, 0.04}) {
            auto r = approach(0.05, 0.10, 0.0, lat, lean_deg * deg);
            Vec3 pl, vl;
            bool latched = false;
            for (int i = 0; i < 12000; ++i) {
                r.fly(0.005);
                if (!latched && r.m.phase() == contact::DockingPhase::SoftCapture) { latched = true; pl = r.c.p; vl = r.c.v; }
            }
            std::printf("[scan] corner 0.05 m/s 0.10 m lateral %+.2f lean %.0f: %s, latch y %.3f vy %+.4f, shear %.0f N\n", lat,
                        lean_deg, latched ? "captured" : report::id_of(r.m.failure().code).data(), pl.y, vl.y, r.m.peaks().shear_n);
        }
    }
    for (double lean_deg : {30.0, 45.0}) {
        for (double off : {0.10}) {
            auto r = approach(0.075, off, 0.0, 0.0, lean_deg * deg);
            Vec3 pl, vl;
            bool latched = false;
            for (int i = 0; i < 12000; ++i) {
                r.fly(0.005);
                if (!latched && r.m.phase() == contact::DockingPhase::SoftCapture) { latched = true; pl = r.c.p; vl = r.c.v; }
            }
            std::printf("[scan] latch 0.075 m/s offset %.2f lean %.0f: y %.3f vy %+.4f, shear %.0f N\n", off, lean_deg, pl.y, vl.y,
                        r.m.peaks().shear_n);
        }
    }
    SUCCEED();
}
