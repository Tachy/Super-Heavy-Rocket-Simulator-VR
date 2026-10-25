// Crew escape (docs/superpowers/archive/specs/2026-10-22-crew-escape-design.md), Task 3: can a crewed ship still try the catch?

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/catch_feasibility.hpp"
#include "starsim/gnc/ship_landing.hpp"

using namespace starsim;
using Catch::Matchers::ContainsSubstring;

namespace {

vehicle::Vehicle ship_with(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], propellant_kg);
    return vehicle::Vehicle(cfg, STARSIM_CONFIG_DIR);
}

const std::vector<std::string>& flip_groups() {
    static const auto groups = gnc::ship_landing_parameters().flip_groups;
    return groups;
}

}  // namespace

TEST_CASE("A full landing load with all engines can try the catch", "[crew_escape][gnc]") {
    const auto f = gnc::catch_feasibility(ship_with(35000.0), flip_groups(), 25000.0);
    CHECK(f.feasible);
    CHECK(f.reason.empty());
}

TEST_CASE("A failed flip engine gives the catch up", "[crew_escape][gnc]") {
    auto ship = ship_with(35000.0);
    auto& engines = ship.stages().front().engines;
    for (auto& e : engines) {
        if (e.group_id == "ship/sea_level_2") e.failed = true;
    }
    const auto f = gnc::catch_feasibility(ship, flip_groups(), 25000.0);
    CHECK_FALSE(f.feasible);
    CHECK_THAT(f.reason, ContainsSubstring("Triebwerk"));
}

TEST_CASE("Too little propellant gives the catch up", "[crew_escape][gnc]") {
    const auto f = gnc::catch_feasibility(ship_with(15000.0), flip_groups(), 25000.0);
    CHECK_FALSE(f.feasible);
    CHECK_THAT(f.reason, ContainsSubstring("Treibstoff"));
}
