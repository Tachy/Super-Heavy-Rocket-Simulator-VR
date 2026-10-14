// Crew Dragon (d2 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): vehicle data, payload stacking,
// thruster availability, orbital attitude modes, Draco translation burns.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

}  // namespace

TEST_CASE("Crew Dragon loads as capsule and trunk", "[dragon][config]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    REQUIRE(cfg.stages.size() == 2);
    CHECK(cfg.stages[0].id == "trunk");
    CHECK(cfg.stages[1].id == "capsule");
    const auto& capsule = cfg.stages[1];
    CHECK(capsule.rcs.thrusters.size() == 16);
    int behind_cone = 0;
    for (const auto& t : capsule.rcs.thrusters) {
        CHECK(t.thrust_n == Catch::Approx(400.0));
        behind_cone += t.behind_nose_cone ? 1 : 0;
    }
    CHECK(behind_cone == 4);
    CHECK(capsule.nose_cone);
    REQUIRE(capsule.engine_groups.size() == 1);
    CHECK(capsule.engine_groups[0].count == 8);
    CHECK(capsule.engine_groups[0].abort_only);
    // 12.5 t at launch [S], 2,562 kg MMH/NTO [S] in the RCS tank (out of the propellant gauge).
    const vehicle::Vehicle v(cfg, config_dir);
    CHECK(v.mass_properties().mass_kg == Catch::Approx(12500.0));
    CHECK(v.rcs_propellant_kg() == Catch::Approx(2562.0));
    CHECK(v.propellant_kg() == 0.0);
}

TEST_CASE("A payload vehicle stacks on top of its launcher", "[dragon][config]") {
    const auto f9 = config::load_vehicle(config_dir, "falcon9_block5");
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const auto stack = config::stack_payload(f9, dragon);
    REQUIRE(stack.stages.size() == 4);
    CHECK(stack.stages[2].id == "trunk");
    CHECK(stack.stages[3].id == "capsule");
    CHECK_FALSE(stack.stages[1].payload);
    CHECK(stack.stages[2].payload);
    CHECK(stack.stages[3].payload);
    CHECK(stack.stages[1].payload_mass_kg == 0.0);  // the payload is the stages now, not a mass
    CHECK(stack.engines.contains("superdraco"));
    CHECK(stack.engines.contains("merlin1d"));
    auto bare = f9;
    bare.stages.back().payload_mass_kg = 0.0;
    const vehicle::Vehicle launcher(bare, config_dir), stacked(stack, config_dir);
    CHECK(stacked.mass_properties().mass_kg == Catch::Approx(launcher.mass_properties().mass_kg + 12500.0));
    // Stage ids must stay unique (engine groups are addressed "<stage>/<group>").
    CHECK_THROWS_AS(config::stack_payload(f9, f9), config::ConfigError);
}

