#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "starsim/aero/slender_body_aero.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/propulsion/engine_performance.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
}

TEST_CASE("Vehicle configuration loads with all referenced engines", "[config]") {
    const auto v = config::load_vehicle(config_dir, "starship_block3");
    REQUIRE(v.stages.size() == 2);
    CHECK(v.stages[0].id == "booster");
    CHECK(v.stages[1].id == "ship");
    CHECK(v.stages[0].engine_groups.size() == 3);
    CHECK(v.engines.size() == 2);
    CHECK(v.engine("raptor3").gimbal_max_rad == Catch::Approx(15.0 * constants::deg_to_rad));
    CHECK(v.propellant.mixture_ratio == 3.6);
    CHECK(v.stages[1].aero.nose_length_m == 17.0);
}

TEST_CASE("All shipped vehicle configurations load", "[config]") {
    const auto id = GENERATE("starship_block1", "starship_block2", "starship_block3", "falcon9_block5");
    const auto v = config::load_vehicle(config_dir, id);
    CHECK_FALSE(v.stages.empty());
    for (const auto& stage : v.stages) {
        if (!stage.aero.ascent_drag_table.empty()) {
            CHECK_NOTHROW(aero::AxialDragCurve(config::load_csv_table(config_dir / stage.aero.ascent_drag_table)));
        }
    }
}

TEST_CASE("Configuration errors name the file", "[config]") {
    CHECK_THROWS_AS(config::load_vehicle(config_dir, "does_not_exist"), config::ConfigError);
}

TEST_CASE("US76 atmosphere matches the published tables", "[environment][validation]") {
    // Geometric altitude, temperature K, pressure Pa, density kg/m^3 (US76, NOAA-S/T 76-1562, Table I).
    struct Row { double z, t, p, rho; };
    const auto row = GENERATE(Row{0, 288.150, 101325.0, 1.2250}, Row{5000, 255.676, 54048.0, 0.73643},
                              Row{10000, 223.252, 26500.0, 0.41351}, Row{20000, 216.650, 5529.3, 0.088910},
                              Row{30000, 226.509, 1197.0, 0.018410}, Row{50000, 270.650, 79.779, 1.0269e-3},
                              Row{70000, 219.585, 5.2209, 8.2829e-5}, Row{80000, 198.639, 1.0524, 1.8458e-5});
    const environment::Us76Atmosphere atmosphere;
    const auto s = atmosphere.at(row.z);
    INFO("altitude " << row.z);
    CHECK(s.temperature_k == Catch::Approx(row.t).epsilon(2e-4));
    CHECK(s.pressure_pa == Catch::Approx(row.p).epsilon(2e-3));
    CHECK(s.density_kg_m3 == Catch::Approx(row.rho).epsilon(2e-3));
}

TEST_CASE("US76 atmosphere is continuous at 86 km and vanishes in space", "[environment]") {
    const environment::Us76Atmosphere atmosphere;
    const double below = atmosphere.at(85999.0).density_kg_m3;
    const double above = atmosphere.at(86001.0).density_kg_m3;
    CHECK(above == Catch::Approx(below).epsilon(0.01));
    CHECK(atmosphere.at(400e3).density_kg_m3 == Catch::Approx(2.803e-12).epsilon(1e-6));
    CHECK(atmosphere.at(2000e3).density_kg_m3 == 0.0);
    CHECK(atmosphere.at(0.0).speed_of_sound_mps == Catch::Approx(340.294).epsilon(1e-5));
}

TEST_CASE("Engine performance reproduces sea-level and vacuum data", "[propulsion]") {
    const auto v = config::load_vehicle(config_dir, "starship_block3");
    const propulsion::EnginePerformance raptor(v.engine("raptor3"));
    CHECK(raptor.thrust_n(1.0, constants::standard_pressure_pa) == Catch::Approx(2.4517e6));
    CHECK(raptor.specific_impulse_s(1.0, constants::standard_pressure_pa) == Catch::Approx(330.0));
    CHECK(raptor.specific_impulse_s(1.0, 0.0) == Catch::Approx(350.0).epsilon(1e-3));
    CHECK(raptor.thrust_n(0.4, 0.0) == Catch::Approx(0.4 * raptor.thrust_n(1.0, 0.0)));

    const propulsion::EnginePerformance rvac(v.engine("raptor_vacuum3"));
    CHECK(rvac.specific_impulse_s(1.0, 0.0) == Catch::Approx(380.0));
    CHECK_FALSE(rvac.can_operate(constants::standard_pressure_pa));
    CHECK(rvac.can_operate(100.0));
}

TEST_CASE("Slender body aerodynamics", "[aero]") {
    const auto table = config::load_csv_table(config_dir / "aero/starship_stack_ascent.csv");
    const aero::AxialDragCurve drag(table);
    CHECK(drag.cd(0.0, true) == Catch::Approx(0.30));
    CHECK(drag.cd(1.125, true) == Catch::Approx(0.60));  // midway between Mach 1.05 and 1.2
    CHECK(drag.cd(99.0, false) == Catch::Approx(0.29));

    const aero::SlenderBodyAero body({124.4, 9.0, 17.0}, drag);
    // Nose-first at zero angle of attack: pure drag along -x.
    const auto straight = body.compute({300.0, 0.0, 0.0}, 0.4, 300.0, true);
    CHECK(straight.mach == Catch::Approx(1.0));
    CHECK(straight.force_body_n.x < 0.0);
    CHECK(straight.force_body_n.y == 0.0);
    CHECK(straight.dynamic_pressure_pa == Catch::Approx(0.5 * 0.4 * 300.0 * 300.0));

    // Sideslip to +y: normal force pushes towards -y, centre of pressure ahead of mid-length.
    const auto slip = body.compute({300.0, 300.0 * std::tan(0.05), 0.0}, 0.4, 300.0, true);
    CHECK(slip.angle_of_attack_rad == Catch::Approx(0.05));
    CHECK(slip.force_body_n.y < 0.0);
    CHECK(slip.normal_force_station_m > 0.5 * 124.4);
}
