// ISS i2 (docs/superpowers/specs/2026-10-19-iss-docking-design.md): Crew Dragon translation on its 16 Dracos -
// authority per axis at zero torque, coupling, propellant per m/s, and the Dragon docking port.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
// [E] user decision 2026-10-19 ("Layout behalten"): the [E] Draco layout reaches ~0.046 m/s^2 laterally at 12.5 t
// with the cone open (LP bound); 0.04 m/s^2 is 1 m/s in 25 s - ample for cm/s approach corrections.
constexpr double min_translation_accel_mps2 = 0.04;
const Vec3 axes[] = {{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}};
}  // namespace

TEST_CASE("Draco translation authority with the nose cone open", "[iss][rendezvous][rcs][validation]") {
    vehicle::Vehicle v(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    v.set_nose_cone_open(true);
    const double mass = v.mass_properties().mass_kg;
    const auto& a = v.rcs_allocator();
    const Vec3 couple = a.max_torque_nm();
    const double smallest_couple = std::min({couple.x, couple.y, couple.z});
    const double ideal_kg_per_mps = mass / (v.stages().back().config.rcs.isp_s * constants::standard_gravity_mps2);
    for (const Vec3& u : axes) {
        CAPTURE(u.x, u.y, u.z);
        std::vector<double> duty;
        const double force = a.pure_force_capability_n(u, &duty);
        const Vec3 f = a.force_n(duty), torque = a.torque_nm(duty);
        const auto capability = v.rcs_pure_force_capability(u);
        REQUIRE(capability.force_n == Catch::Approx(force));
        const double kg_per_mps = capability.mass_flow_kgps / (capability.force_n / mass);
        std::printf("[validation] Draco translation (%+.0f,%+.0f,%+.0f): %.1f N = %.4f m/s^2, torque %.2f N m, "
                    "%.2f kg per m/s (ideal %.2f); coupled capability %.1f N\n",
                    u.x, u.y, u.z, force, force / mass, norm(torque), kg_per_mps, ideal_kg_per_mps,
                    a.force_capability_n(u));
        CHECK(force / mass >= min_translation_accel_mps2);
        CHECK(norm(torque) < 0.02 * smallest_couple);   // translation without a turn: < 2 % of the weakest couple
        CHECK(norm(f - u * force) < 0.02 * force);      // no cross-axis push
        CHECK(kg_per_mps >= ideal_kg_per_mps * 0.999);  // physics bound: m / (Isp g0)
        // [E] opposing thrusters cost: measured 2026-10-19 axial 1.0-1.07x, lateral 3.8x (16.2 kg per m/s: the axial
        // pairs that cancel the side pods' couple fire too)
        CHECK(kg_per_mps < 4.5 * ideal_kg_per_mps);
    }
}

TEST_CASE("Draco lateral translation needs the open nose cone", "[iss][rendezvous][rcs]") {
    vehicle::Vehicle v(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    REQUIRE_FALSE(v.nose_cone_open());
    const auto& a = v.rcs_allocator();
    for (const Vec3& u : {Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, -1.0}, Vec3{-1.0, 0.0, 0.0}}) {
        CAPTURE(u.x, u.y, u.z);
        CHECK(a.pure_force_capability_n(u) < 1.0);  // LP bound 0 N: the nose Dracos cancel the side pods' couple
    }
}

TEST_CASE("A pure translation command fires a zero-torque set", "[iss][rendezvous][rcs]") {
    vehicle::Vehicle v(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    v.set_nose_cone_open(true);
    vehicle::VehicleCommand c;
    c.rcs_force_fraction = {0.0, 1.0, 0.0};
    c.rcs_force_pure = true;
    const double pure = v.rcs_pure_force_capability({0.0, 1.0, 0.0}).force_n;
    constexpr double dt_s = 0.005;
    constexpr int ticks = 400;  // 2 s of pulses
    Vec3 force, torque;
    for (int k = 0; k < ticks; ++k) {
        v.update_rcs(dt_s, c);
        force += v.rcs_force_n() / ticks;
        torque += v.rcs_torque_nm() / ticks;
    }
    CHECK(force.y == Catch::Approx(pure).epsilon(0.05));  // [E] on/off pulses with a 10 ms minimum on-time
    CHECK(std::abs(force.x) < 0.05 * pure);              // no push along the docking axis
    CHECK(norm(torque) < 0.05 * pure * 1.0);             // ~ 1 m lever: no turn either
}

TEST_CASE("Crew Dragon has its docking port under the nose cone", "[iss][rendezvous][config]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    const auto& capsule = cfg.stages.back();
    REQUIRE(capsule.id == "capsule");
    REQUIRE(capsule.docking_ports.size() == 1);
    const auto& port = capsule.docking_ports.front();
    CHECK(port.id == "idss");
    CHECK(port.behind_nose_cone);
    CHECK(port.axis.x == Catch::Approx(1.0));
    CHECK(port.position_m.x == Catch::Approx(4.0));
    for (const auto& p : config::load_vehicle(config_dir, "iss").stages.front().docking_ports) {
        CHECK_FALSE(p.behind_nose_cone);
    }
}
