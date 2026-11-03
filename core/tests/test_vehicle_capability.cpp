// Vehicle capability derivations (planner-to-vehicle-parameters plan, Task 2): hand values against the config.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/vehicle_capability.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

const config::StageConfig& stage_named(const config::VehicleConfig& v, const std::string& id) {
    for (const auto& s : v.stages) {
        if (s.id == id) return s;
    }
    throw std::runtime_error("no stage " + id);
}

}  // namespace

TEST_CASE("Stage pitch inertia: the vehicle's cylinder, or the configured moments", "[gnc][capability]") {
    config::StageConfig s;
    s.length_m = 71.0;
    s.diameter_m = 9.0;
    s.dry_mass_kg = 200000.0;
    // Thin-walled cylinder of the whole mass: m (r^2 / 2 + l^2 / 12) = 5e5 (10.125 + 420.0833) kg m^2.
    CHECK(gnc::stage_pitch_inertia_kg_m2(s, 300000.0) == Approx(5.0e5 * (0.5 * 4.5 * 4.5 + 71.0 * 71.0 / 12.0)));
    CHECK(gnc::stage_pitch_inertia_kg_m2(s, 300000.0) == Approx(2.151042e8).epsilon(1e-6));
    s.inertia_kg_m2 = math::Vec3{1.0e6, 2.0e7, 2.1e7};
    CHECK(gnc::stage_pitch_inertia_kg_m2(s, 0.0) == Approx(2.0e7));
}

TEST_CASE("Engine set: the 13 centre Raptors of Super Heavy from the engine config", "[gnc][capability]") {
    const auto v = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& booster = stage_named(v, "booster");
    const auto& raptor = v.engine("raptor3");
    REQUIRE(raptor.throttle_min == 0.30);
    // F = t (F_ref + p_ref A_e) - p A_e (config/README.md), A_e from the 1.30 m exit.
    const double a_exit = constants::pi * 0.65 * 0.65;
    REQUIRE(raptor.nozzle_exit_area_m2 == Approx(a_exit).epsilon(1e-9));
    const double f_vac = raptor.thrust_ref_n + raptor.reference_pressure_pa * a_exit;
    const std::vector<std::string> centre{"center", "divert", "middle"};
    const double cg = 30.0;

    const auto vac = gnc::engine_set_thrust(v, booster, centre, 0.0, cg);
    CHECK(vac.min_n == Approx(13.0 * 0.30 * f_vac));
    CHECK(vac.min_n == Approx(11.2335e6).epsilon(1e-4));
    CHECK(vac.max_n == Approx(13.0 * f_vac));
    CHECK(vac.gimbal_rad == Approx(15.0 * constants::pi / 180.0));
    CHECK(vac.gimbal_rate_radps == Approx(20.0 * constants::pi / 180.0));
    CHECK(vac.lever_m == Approx(cg));  // thrust at the stage base
    CHECK(vac.gimballed_fraction == Approx(1.0));

    const auto sl = gnc::engine_set_thrust(v, booster, {"booster/center", "booster/divert", "booster/middle"}, 101325.0, cg);
    CHECK(sl.min_n == Approx(13.0 * (0.30 * f_vac - 101325.0 * a_exit)));
    CHECK(sl.min_n == Approx(9.4851e6).epsilon(1e-4));
    CHECK(sl.max_n == Approx(13.0 * raptor.thrust_ref_n));

    // The fixed outer ring adds thrust but no gimbal torque.
    const auto all = gnc::engine_set_thrust(v, booster, {"center", "divert", "middle", "outer"}, 0.0, cg);
    CHECK(all.max_n == Approx(33.0 * f_vac));
    CHECK(all.gimballed_fraction == Approx(13.0 / 33.0));

    // Gimbal pitch acceleration: F(t) sin(gimbal) lever / I; t = 0.3 is the set's minimum.
    const double inertia = 1.0e8;
    CHECK(gnc::max_pitch_acceleration_radps2(vac, 0.30, inertia) ==
          Approx(vac.min_n * std::sin(vac.gimbal_rad) * cg / inertia));
    CHECK(gnc::max_pitch_acceleration_radps2(vac, 1.0, inertia) ==
          Approx(vac.max_n * std::sin(vac.gimbal_rad) * cg / inertia));
    CHECK(gnc::max_pitch_acceleration_radps2(all, 1.0, inertia) ==
          Approx(13.0 * f_vac * std::sin(vac.gimbal_rad) * cg / inertia));

    CHECK_THROWS_AS(gnc::engine_set_thrust(v, booster, {"nope"}, 0.0, cg), std::invalid_argument);
}

TEST_CASE("Flip duration: bang-bang with a rate cap", "[gnc][capability]") {
    const double pi = constants::pi;
    CHECK(gnc::flip_duration_s(pi, 0.2, 0.0) == Approx(2.0 * std::sqrt(pi / 0.2)));
    // Cap 0.1 rad/s: 0.5 s to reach it each way (0.025 rad each), the rest coasts: pi / 0.1 + 0.1 / 0.2.
    CHECK(gnc::flip_duration_s(pi, 0.2, 0.1) == Approx(pi / 0.1 + 0.1 / 0.2));
    CHECK(gnc::flip_duration_s(pi, 0.2, 0.1) == Approx(31.9159).epsilon(1e-5));
    // A cap above the triangle's peak rate sqrt(angle accel) changes nothing.
    CHECK(gnc::flip_duration_s(pi, 0.2, 10.0) == Approx(2.0 * std::sqrt(pi / 0.2)));
}

TEST_CASE("Falcon 9 RCS turn against its config comment's stage inertia", "[gnc][capability][falcon9]") {
    const auto v = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& s1 = stage_named(v, "first_stage");
    // falcon9_block5.toml [[stage.rcs.pod]]: two thrusters of 6.5 kN turn the stage of 1.12e7 kg m^2 on a 30.4 m
    // lever: 3.95e5 N m, 0.0353 rad/s^2 = 2.0 deg/s^2 (the comment's 6.5 kN "flies 14.7 s").
    const double lever = 30.4;
    const double cg = 41.9 - lever;
    const double a = gnc::rcs_pitch_acceleration_radps2(s1, 1.12e7, cg);
    CHECK(a == Approx(2.0 * 6500.0 * lever / 1.12e7));
    CHECK(a == Approx(0.035286).epsilon(1e-4));
}
