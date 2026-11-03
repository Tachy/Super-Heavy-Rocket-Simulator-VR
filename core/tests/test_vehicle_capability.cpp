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
#include "starsim/vehicle/stage_inertia.hpp"
#include "starsim/vehicle/vehicle.hpp"

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

TEST_CASE("Stage pitch inertia: dry cylinder and settled tank columns about the common CG", "[gnc][capability]") {
    config::VehicleConfig v;
    v.propellant.oxidizer_density_kg_m3 = 1000.0;
    v.propellant.fuel_density_kg_m3 = 500.0;
    config::StageConfig s;
    s.id = "test";
    s.length_m = 71.0;
    s.diameter_m = 9.0;
    s.dry_mass_kg = 200000.0;
    s.dry_cg_station_m = 30.0;
    // Dry only: the thin-walled cylinder about its own CG, m (r^2 / 2 + l^2 / 12) = 2e5 (10.125 + 420.0833) kg m^2.
    CHECK(gnc::stage_pitch_inertia_kg_m2(v, s, 0.0) == Approx(2.0e5 * (0.5 * 4.5 * 4.5 + 71.0 * 71.0 / 12.0)));
    CHECK(gnc::stage_pitch_inertia_kg_m2(v, s, 0.0) == Approx(8.604167e7).epsilon(1e-6));

    // Two main tanks (capacity 2:1), the load split by capacity and each a column on its bottom:
    // LOX 200 t at 1000 kg/m^3 on station 0: h = 200 / (pi 4.5^2) = 3.1438 m; CH4 100 t at 500 kg/m^3 on 40 m: same h.
    config::TankConfig lox, fuel;
    lox.contents = config::TankContents::Oxidizer;
    lox.capacity_kg = 2.0e6;
    lox.base_station_m = 0.0;
    fuel.contents = config::TankContents::Fuel;
    fuel.capacity_kg = 1.0e6;
    fuel.base_station_m = 40.0;
    s.tanks = {lox, fuel};
    const double area = constants::pi * 4.5 * 4.5;
    const double h = 2.0e5 / (1000.0 * area);
    const double cg = (2.0e5 * 30.0 + 2.0e5 * 0.5 * h + 1.0e5 * (40.0 + 0.5 * h)) / 5.0e5;
    const double column = 0.25 * 4.5 * 4.5 + h * h / 12.0;
    const double expected = 2.0e5 * (0.5 * 4.5 * 4.5 + 71.0 * 71.0 / 12.0) + 2.0e5 * (30.0 - cg) * (30.0 - cg) +
                            2.0e5 * (column + (0.5 * h - cg) * (0.5 * h - cg)) +
                            1.0e5 * (column + (40.0 + 0.5 * h - cg) * (40.0 + 0.5 * h - cg));
    const auto mp = vehicle::settled_stage_mass_properties(v, s, 3.0e5);
    CHECK(mp.mass_kg == Approx(5.0e5));
    CHECK(mp.cg_station_m == Approx(cg));
    CHECK(mp.pitch_inertia_kg_m2 == Approx(expected));
    CHECK(gnc::stage_pitch_inertia_kg_m2(v, s, 3.0e5) == Approx(expected));

    // Configured dry moments replace the cylinder.
    s.tanks.clear();
    s.inertia_kg_m2 = math::Vec3{1.0e6, 2.0e7, 2.1e7};
    CHECK(gnc::stage_pitch_inertia_kg_m2(v, s, 0.0) == Approx(2.0e7));
    CHECK_THROWS_AS(gnc::stage_pitch_inertia_kg_m2(v, s, 1.0), std::invalid_argument);  // propellant but no main tank
}

TEST_CASE("Falcon 9 first stage at separation: derived inertia against the flight dynamics and the config comment",
          "[gnc][capability][falcon9]") {
    const auto full = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& s1 = stage_named(full, "first_stage");
    // The comment's flip is the RTLS one (Crew-13 frames); our RTLS mission separates with 52.1 t
    // (docs/subsystems/ascent-staging.md, test_falcon9_rtls). For information, the drone-ship MECO load
    // (reserve_propellant_kg, 46.75 t) gives 1.0646e7 kg m^2.
    const double reserve_kg = 52100.0;
    const auto mp = vehicle::settled_stage_mass_properties(full, s1, reserve_kg);
    CHECK(vehicle::settled_stage_mass_properties(full, s1, s1.reserve_propellant_kg).pitch_inertia_kg_m2 ==
          Approx(1.0646e7).epsilon(1e-3));

    // The flight dynamics on the same stage alone at the same load, propellant settled aft: must agree exactly.
    auto alone = full;
    alone.stages = {s1};
    double main_capacity_kg = 0.0;
    for (const auto& t : alone.stages.front().tanks) {
        if (t.role == config::TankRole::Main && t.contents != config::TankContents::Rcs) main_capacity_kg += t.capacity_kg;
    }
    for (auto& t : alone.stages.front().tanks) {
        if (t.role == config::TankRole::Main && t.contents != config::TankContents::Rcs) {
            t.mass_kg = reserve_kg * t.capacity_kg / main_capacity_kg;
        }
    }
    const vehicle::Vehicle stage(alone, STARSIM_CONFIG_DIR);
    const auto dyn = stage.mass_properties();
    CHECK(mp.mass_kg == Approx(dyn.mass_kg).epsilon(1e-12));
    CHECK(mp.cg_station_m == Approx(dyn.cg_station_m).epsilon(1e-12));
    CHECK(mp.pitch_inertia_kg_m2 == Approx(dyn.inertia_kg_m2.y).epsilon(1e-12));
    CHECK(gnc::stage_pitch_inertia_kg_m2(full, s1, reserve_kg) == Approx(dyn.inertia_kg_m2.y).epsilon(1e-12));

    // The config comment (falcon9_block5.toml [[stage.rcs.pod]], [D] from the Crew-13 frames): 1.12e7 kg m^2 on a 30.4 m
    // lever (CG 41.9 - 30.4 = 11.5 m). Derived at 52.1 t: 1.123e7 kg m^2, CG 11.46 m (lever 30.44 m). Tolerance 1 % on
    // the inertia and 0.1 m on the CG: the comment's figures are rounded to three digits; the separation load itself
    // moves with the ascent (1.1e3 kg m^2 per kg of propellant here).
    INFO("derived I = " << mp.pitch_inertia_kg_m2 << " kg m^2, CG = " << mp.cg_station_m << " m, mass = " << mp.mass_kg);
    CHECK(mp.pitch_inertia_kg_m2 == Approx(1.12e7).epsilon(0.01));
    CHECK(mp.cg_station_m == Approx(41.9 - 30.4).margin(0.1));
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
    CHECK_THROWS_AS(gnc::engine_set_thrust(v, booster, {"center", "booster/center"}, 0.0, cg), std::invalid_argument);
}

TEST_CASE("Engine set of mixed types: the gimballed thrust per throttle end", "[gnc][capability]") {
    // Ship: 3 Raptor SL gimballed + 3 RVac fixed. At sea level the RVac's larger exit loses more (- p A_e), so the
    // gimballed share differs between the throttle ends; the torque follows the gimballed engines' own thrust.
    const auto v = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& ship = stage_named(v, "ship");
    const double cg = 20.0, inertia = 1.0e8, p = 101325.0;
    const auto all = gnc::engine_set_thrust(v, ship, {"sea_level_1", "sea_level_2", "sea_level_3", "vacuum"}, p, cg);
    const auto sl = gnc::engine_set_thrust(v, ship, {"sea_level_1", "sea_level_2", "sea_level_3"}, p, cg);
    REQUIRE(sl.throttle_min == all.throttle_min);
    REQUIRE(sl.throttle_max == all.throttle_max);
    CHECK(all.gimballed_min_n == Approx(sl.min_n));
    CHECK(all.gimballed_max_n == Approx(sl.max_n));
    CHECK(all.gimballed_min_n / all.min_n != Approx(all.gimballed_max_n / all.max_n));
    for (const double t : {all.throttle_min, 0.5 * (all.throttle_min + all.throttle_max), all.throttle_max}) {
        CHECK(gnc::max_pitch_acceleration_radps2(all, t, inertia) == Approx(gnc::max_pitch_acceleration_radps2(sl, t, inertia)));
    }
}

TEST_CASE("Flip duration: bang-bang with a rate cap", "[gnc][capability]") {
    const double pi = constants::pi;
    CHECK(gnc::flip_duration_s(pi, 0.2, 0.0) == Approx(2.0 * std::sqrt(pi / 0.2)));
    // Cap 0.1 rad/s: 0.5 s to reach it each way (0.025 rad each), the rest coasts: pi / 0.1 + 0.1 / 0.2.
    CHECK(gnc::flip_duration_s(pi, 0.2, 0.1) == Approx(pi / 0.1 + 0.1 / 0.2));
    CHECK(gnc::flip_duration_s(pi, 0.2, 0.1) == Approx(31.9159).epsilon(1e-5));
    // A cap above the triangle's peak rate sqrt(angle accel) changes nothing.
    CHECK(gnc::flip_duration_s(pi, 0.2, 10.0) == Approx(2.0 * std::sqrt(pi / 0.2)));
    CHECK_THROWS_AS(gnc::flip_duration_s(pi, 0.0, 0.1), std::invalid_argument);
    CHECK_THROWS_AS(gnc::flip_duration_s(pi, -0.1, 0.0), std::invalid_argument);
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
