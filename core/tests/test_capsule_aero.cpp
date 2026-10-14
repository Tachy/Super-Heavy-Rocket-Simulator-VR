// Blunt capsule aerodynamics (d3 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): the table from
// tools/aero/gen_capsule_aero.py (modified Newtonian), force and moment conventions, offset-CG trim.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/aero/capsule_aero.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {

aero::CapsuleAero dragon_aero() {
    return aero::CapsuleAero(
        aero::load_capsule_aero_table(std::filesystem::path(STARSIM_CONFIG_DIR) / "aero/crew_dragon_aero.csv"), 4.0, 0.0);
}

Vec3 air_velocity(double speed, double theta_rad) {  // rho = +y
    return {-speed * std::cos(theta_rad), speed * std::sin(theta_rad), 0.0};
}

// Generator output (python tools/aero/gen_capsule_aero.py --trim --cg-station-m 1.25): total radial offset for L/D 0.20.
constexpr double generator_offset_m = 0.0747;

}  // namespace

TEST_CASE("Capsule aero: shield-first flow pushes along +x with the tabulated axial force", "[aero][capsule]") {
    const auto a = dragon_aero();
    const double rho = 1e-4, sos = 300.0, v = 7000.0;
    const auto r = a.compute(air_velocity(v, 0.0), rho, sos);
    const double q = 0.5 * rho * v * v, s = constants::pi * 4.0;
    CHECK(r.force_body_n.x / (q * s) > 1.3);
    CHECK(r.force_body_n.x / (q * s) < 1.7);
    CHECK(std::abs(r.force_body_n.y) < 1e-6 * r.force_body_n.x);
    CHECK(norm(r.moment_body_nm) < 1e-6 * q * s * 4.0);
    CHECK(r.dynamic_pressure_pa == Catch::Approx(q));
}

TEST_CASE("Capsule aero: force and moment follow the radial direction of the flow", "[aero][capsule]") {
    const auto a = dragon_aero();
    const double th = 20.0 * constants::deg_to_rad;
    const auto ry = a.compute(air_velocity(7000.0, th), 1e-4, 300.0);
    const Vec3 uz{-7000.0 * std::cos(th), 0.0, 7000.0 * std::sin(th)};  // rho = +z
    const auto rz = a.compute(uz, 1e-4, 300.0);
    CHECK(ry.force_body_n.x == Catch::Approx(rz.force_body_n.x));
    CHECK(ry.force_body_n.y == Catch::Approx(rz.force_body_n.z));
    CHECK(ry.moment_body_nm.z == Catch::Approx(-rz.moment_body_nm.y));  // x cross y = z, x cross z = -y
    CHECK(ry.force_body_n.y < 0.0);  // the normal force pushes away from the windward side
    CHECK(ry.angle_of_attack_rad == Catch::Approx(th));
}

TEST_CASE("Capsule aero: offset CG trims at L/D ~0.2 and is statically stable", "[aero][capsule]") {
    const auto a = dragon_aero();
    const Vec3 cg{1.25, generator_offset_m, 0.0};
    const auto t = a.trim(20.0, cg);
    REQUIRE(t.found);
    CHECK(t.lift_to_drag == Catch::Approx(0.20).margin(0.01));
    CHECK(t.theta_rad > 10.0 * constants::deg_to_rad);
    CHECK(t.theta_rad < 30.0 * constants::deg_to_rad);
    CHECK(a.pitch_moment_about(20.0, t.theta_rad + 0.02, cg) < a.pitch_moment_about(20.0, t.theta_rad - 0.02, cg));
    // the trim's force has no moment about the CG in the full 3-D sense either
    const auto r = a.compute(t.air_velocity_body_unit * 7000.0, 1e-4, 300.0);
    const Vec3 moment_cg = r.moment_body_nm - cross(cg, r.force_body_n);
    CHECK(norm(moment_cg) < 1e-3 * norm(r.force_body_n) * 4.0);
    // lift and drag split of that force
    const double drag = -dot(r.force_body_n, t.air_velocity_body_unit);
    const Vec3 lift = r.force_body_n + t.air_velocity_body_unit * drag;
    CHECK(norm(lift) / drag == Catch::Approx(t.lift_to_drag).epsilon(1e-6));
    CHECK(dot(math::normalized(lift), t.lift_body_unit) == Catch::Approx(1.0));
}

TEST_CASE("Capsule aero: subsonic drag of a blunt capsule", "[aero][capsule]") {
    const auto c = dragon_aero().coefficients(0.5, 0.0);
    CHECK(c.ca > 0.85);  // [E] capsules ~0.9-1.2 subsonic
    CHECK(c.ca < 1.3);
}
