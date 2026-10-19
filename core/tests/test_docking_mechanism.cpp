// ISS i5: IDSS docking mechanism - capture inside the envelope, failures outside, timeline, loads, undocking.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/contact/docking_mechanism.hpp"
#include "starsim/math/constants.hpp"

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
Run approach(double axial_mps, double offset_m, double angle_rad = 0.0, double lateral_mps = 0.0) {
    Run r;
    const contact::DockingMechanismParameters p;
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
                         << r.m.failure());
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
    CHECK(r.m.failure() == "Überlast");
    CHECK(r.c.v.x > 0.0);  // moving out again
}

TEST_CASE("Docking mechanism: an offset beyond the petals' capture radius bounces off the tips", "[iss][docking]") {
    auto r = approach(0.075, 0.30);
    r.fly(60.0);
    CHECK(r.m.phase() == contact::DockingPhase::Failed);
    CHECK(r.m.failure() == "Versatz zu groß");
    CHECK(r.c.v.x > 0.0);
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
    CHECK(r.m.refusal() == "Abkoppeln nicht möglich: nicht fest angedockt");
}

TEST_CASE("Docking mechanism: separation speed from the IDSS spring energy", "[iss][docking]") {
    const contact::DockingMechanismParameters p;
    // Dragon 12 t on the 460 t ISS: v_rel = sqrt(2 E / mu)
    const double mu = 12000.0 * 460320.0 / (12000.0 + 460320.0);
    CHECK(contact::separation_relative_speed_mps(p, 12000.0, 460320.0) == Catch::Approx(std::sqrt(2.0 * 43.35 / mu)));
    CHECK(contact::separation_relative_speed_mps(p, 12000.0, 460320.0) == Catch::Approx(0.0862).margin(0.002));
}
