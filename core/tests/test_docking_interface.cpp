// ISS i5: pre-capture contact of the IDSS soft-capture interface (ring faces, petal cone, latch condition).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

#include "starsim/contact/docking_interface.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Mat3;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
// Chaser port axes mated with the passive port: x_c = -x_t, y_c = -y_t, z_c = z_t.
const Mat3 mated_axes = Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});

contact::DockingPose pose(Vec3 position, Vec3 velocity = {}, Mat3 axes = mated_axes) {
    return {position, velocity, axes, {}};
}
}  // namespace

TEST_CASE("Docking interface: no contact while the petals do not overlap", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const auto c = contact::docking_interface_contact(pose({e + 2.0 * p.petal_height_m + 0.01, 0.05, 0.0}), e, p);
    CHECK_FALSE(c.any_contact);
    CHECK(norm(c.force_port_n) == 0.0);
}

TEST_CASE("Docking interface: aligned ring faces touch at the ring extension", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    // 1 cm into the passive ring face, closing at 0.075 m/s
    const auto c = contact::docking_interface_contact(pose({e - 0.01, 0, 0}, {-0.075, 0, 0}), e, p);
    CHECK(c.ring_contact);
    CHECK_FALSE(c.petal_contact);
    // pushes the chaser out of the port: k d + c v
    CHECK(c.force_port_n.x == Catch::Approx(p.ring_stiffness_n_per_m * 0.01 + p.ring_damping_ns_per_m * 0.075));
    CHECK(std::abs(c.force_port_n.y) < 1e-9);
    CHECK(c.latch_ready);  // full ring contact, centred, aligned
}

TEST_CASE("Docking interface: the penalty contact never pulls", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const auto c = contact::docking_interface_contact(pose({e - 0.001, 0, 0}, {0.5, 0, 0}), e, p);  // separating fast
    CHECK(c.force_port_n.x >= 0.0);
}

TEST_CASE("Docking interface: the petal cone centres a lateral offset inside the capture radius", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    // halfway into the petal overlap with 0.20 m offset along +y (clearance there 0.01 + 0.20 tan 30 deg = 0.125 m,
    // capture radius 0.241 m): the cone pushes towards -y and out (+x)
    const double gap = p.petal_height_m;  // ring faces apart by one petal height
    const auto c = contact::docking_interface_contact(pose({e + gap, 0.20, 0}), e, p);
    REQUIRE(c.petal_contact);
    CHECK(c.force_port_n.y < 0.0);
    CHECK(c.force_port_n.x > 0.0);
    CHECK_FALSE(c.ring_contact);
    CHECK_FALSE(c.latch_ready);
    CHECK(c.capture_radius_m == Catch::Approx(p.petal_clearance_m + 2.0 * p.petal_height_m * std::tan(p.petal_lean_rad)));
}

TEST_CASE("Docking interface: beyond the capture radius the petal tips strike the opposing petals", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const double tips = e + 2.0 * p.petal_height_m;
    const auto c = contact::docking_interface_contact(pose({tips - 0.01, 0.40, 0}, {-0.075, 0, 0}), e, p);
    REQUIRE(c.tip_strike);
    CHECK(c.force_port_n.x > 0.0);
    CHECK(std::abs(c.force_port_n.y) < 1e-9);  // no centring on the tips
    CHECK_FALSE(c.latch_ready);
}

TEST_CASE("Docking interface: latch tolerances - offset, tilt beyond the ring's range, roll", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    const double e = p.ring_extension_m;
    const auto touching = Vec3{e - 0.002, 0, 0};
    CHECK(contact::docking_interface_contact(pose(touching), e, p).latch_ready);
    CHECK_FALSE(contact::docking_interface_contact(pose(touching + Vec3{0, p.latch_offset_max_m + 0.005, 0}), e, p)
                    .latch_ready);
    const auto tilt = [&](double angle) {  // chaser pitched about the passive y axis
        const double c = std::cos(angle), s = std::sin(angle);
        const Mat3 r = Mat3::from_columns({c, 0, -s}, {0, 1, 0}, {s, 0, c});
        return r * mated_axes;
    };
    // a tilted chaser with its ring face centre on the passive axis, 2 mm into the passive ring face
    const auto centred = [&](const Mat3& axes) { return Vec3{-0.002, 0, 0} - e * column(axes, 0); };
    const Mat3 inside = tilt(p.ring_tilt_max_rad - 0.5 * deg), beyond = tilt(p.ring_tilt_max_rad + 0.5 * deg);
    CHECK(contact::docking_interface_contact(pose(centred(inside), {}, inside), e, p).latch_ready);
    CHECK_FALSE(contact::docking_interface_contact(pose(centred(beyond), {}, beyond), e, p).latch_ready);
    const auto roll = [&](double angle) {  // about the passive x axis
        const double c = std::cos(angle), s = std::sin(angle);
        return Mat3::from_columns({1, 0, 0}, {0, c, s}, {0, -s, c}) * mated_axes;
    };
    CHECK_FALSE(
        contact::docking_interface_contact(pose(touching, {}, roll(p.latch_roll_max_rad + 0.5 * deg)), e, p).latch_ready);
}

TEST_CASE("Docking interface: the IDSS outline and the dynamic load limit are the IDD values", "[iss][docking]") {
    const contact::DockingInterfaceParameters p;
    CHECK(p.petal_outline_radius_m == 0.600);           // 1200 mm diameter at the SCS mating plane
    CHECK(p.petal_count == 3);
    // a 0.10 m/s ring-face impact (Dragon on the ISS, reduced mass ~12 t) peaks below the dynamic compression limit
    // (planning estimate with these values: 3,981 N; 0.25 m/s: 9,954 N)
    const double mu = 12000.0, dt = 1e-4;
    double x = p.ring_extension_m + 0.001, v = -0.10, peak = 0.0;
    for (int i = 0; i < 200000 && !(x > p.ring_extension_m + 0.001 && v > 0.0); ++i) {
        const auto c = contact::docking_interface_contact(pose({x, 0, 0}, {v, 0, 0}), p.ring_extension_m, p);
        peak = std::max(peak, c.compression_n);
        v += c.force_port_n.x / mu * dt;
        x += v * dt;
    }
    CHECK(peak > 3000.0);
    CHECK(peak < 6500.0);
    CHECK(v > 0.0);  // rebounds without a latch (restitution ~0.46)
}
