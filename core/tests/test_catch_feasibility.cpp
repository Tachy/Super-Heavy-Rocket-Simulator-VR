// Crew escape (docs/superpowers/archive/specs/2026-10-22-crew-escape-design.md), Task 3: can a crewed ship still try the catch?

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>

#include "starsim/config/crew_escape_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/catch_feasibility.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/constants.hpp"

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
    static const auto groups = gnc::ship_flip_groups(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3").stages.at(1));
    return groups;
}

}  // namespace

TEST_CASE("A full landing load with all engines can try the catch", "[crew_escape][gnc]") {
    const auto f = gnc::catch_feasibility(ship_with(35000.0), flip_groups(), 25000.0);
    CHECK(f.feasible);
    CHECK(f.reason.ok());
}

TEST_CASE("A failed flip engine gives the catch up", "[crew_escape][gnc]") {
    auto ship = ship_with(35000.0);
    auto& engines = ship.stages().front().engines;
    for (auto& e : engines) {
        if (e.group_id == "ship/sea_level_2") e.failed = true;
    }
    const auto f = gnc::catch_feasibility(ship, flip_groups(), 25000.0);
    CHECK_FALSE(f.feasible);
    CHECK(f.reason.code == report::Code::catch_engine_failed);
    REQUIRE(f.reason.params.size() == 1);
    CHECK(f.reason.params.front().text == "ship/sea_level_2");
}

TEST_CASE("Too little propellant gives the catch up", "[crew_escape][gnc]") {
    const auto f = gnc::catch_feasibility(ship_with(15000.0), flip_groups(), 25000.0);
    CHECK_FALSE(f.feasible);
    CHECK(f.reason.code == report::Code::catch_propellant_short);
    CHECK(f.reason.params.front().value_si < 25000.0);  // usable below the floor
    CHECK(f.reason.params.back().value_si == 25000.0);
}

// Planner plan Task 4 fix round 2: the floor is the ship's own catch need (the former fixed 30 t was the 290 t ship's).
TEST_CASE("The catch propellant need follows the ship", "[crew_escape][gnc][catch][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& ship = cfg.stages.at(1);
    const double g = constants::standard_gravity_mps2;
    const double drop_m = 700.0, sink_mps = 100.0, catch_sink_mps = 1.0;
    const auto n = gnc::ship_catch_propellant_need(cfg, ship, drop_m, sink_mps, catch_sink_mps);
    const auto p = gnc::ship_landing_parameters(cfg, ship);
    const auto masses = gnc::ship_landing_masses(cfg, ship);
    const double mass = n.end_mass_kg + n.propellant_kg;
    const double a = p.gate_flip.brake_accel_mps2 * masses.heavy_kg / mass;
    std::printf("catch need: %.0f kg (stop %.1f m/s, flip %.1f m/s, ve %.0f m/s, end %.0f kg, a %.2f m/s^2)\n",
                n.propellant_kg, n.stop_dv_mps, n.flip_dv_mps, n.exhaust_velocity_mps, n.end_mass_kg, a);
    CHECK(n.end_mass_kg == Catch::Approx(masses.light_kg));
    CHECK(n.exhaust_velocity_mps == Catch::Approx(330.0 * g));  // Raptor sea level [config]
    CHECK(n.flip_dv_mps == Catch::Approx(gnc::ship_flip_push(p.gate_flip).speed_mps * masses.heavy_kg / mass));
    CHECK(n.propellant_kg == Catch::Approx(n.end_mass_kg * std::expm1((n.stop_dv_mps + n.flip_dv_mps) / n.exhaust_velocity_mps)));

    // Hand case of the stop: fall, then brake at a - g from the latest ignition to the catch sink at the catch, flown
    // by small steps (ignition height by bisection); its velocity change a t_burn is the stop's dv.
    const auto end_sink = [&](double ignition_drop_m, double* burn_s) {
        double v = std::sqrt(sink_mps * sink_mps + 2.0 * g * ignition_drop_m), h = drop_m - ignition_drop_m, t = 0.0;
        const double dt = 1e-4;
        while (h > 0.0 && v > catch_sink_mps) {
            h -= v * dt;
            v -= (a - g) * dt;
            t += dt;
        }
        *burn_s = t;
        return h > 0.0 ? -h : v;  // < catch sink with height left: ignited too early
    };
    double lo = 0.0, hi = drop_m, burn_s = 0.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (end_sink(mid, &burn_s) <= catch_sink_mps ? lo : hi) = mid;
    }
    end_sink(lo, &burn_s);
    CHECK(a * burn_s == Catch::Approx(n.stop_dv_mps).epsilon(2e-3));
    // No thrust that holds the weight: no catch.
    auto weak = cfg;
    for (auto& [id, e] : weak.engines) e.thrust_ref_n *= 0.01;
    CHECK(std::isinf(gnc::ship_catch_propellant_need(weak, weak.stages.at(1), drop_m, sink_mps, catch_sink_mps).propellant_kg));
}

TEST_CASE("The crewed ship's catch floor gives up no catchable load", "[crew_escape][gnc][catch][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& ship = cfg.stages.at(1);
    const double arm_top_m = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2").tower->catch_height_m;
    const double margin = config::load_crew_escape(STARSIM_CONFIG_DIR, "starship_crew").catch_propellant_margin;
    const double floor_kg = gnc::ship_catch_propellant_floor_kg(cfg, ship, 800.0, 100.0, arm_top_m, margin);
    const auto masses = gnc::ship_landing_masses(cfg, ship);
    const double drop_m = 800.0 - (arm_top_m - (ship.catch_pins.station_m - masses.heavy_cg_station_m));
    const auto p = gnc::ship_landing_parameters(cfg, ship);
    const auto n = gnc::ship_catch_propellant_need(cfg, ship, drop_m, 100.0, p.scvx.snapshot.catch_sink_mps);
    std::printf("catch floor: %.0f kg (need %.0f kg over %.0f m)\n", floor_kg, n.propellant_kg, drop_m);
    CHECK(floor_kg == Catch::Approx(n.propellant_kg * (1.0 + margin)));
    // Hidden "Ship catch: lowest catchable gate load": 18 t of propellant caught (dry 177 t ship); usable above the
    // unusable header propellant
    CHECK(floor_kg < 18000.0 - ship.unusable_propellant.header_kg);
}
