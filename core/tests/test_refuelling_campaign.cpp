// Refuelling r6: campaign calculator, its setup from config, the measured-numbers file.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_reference.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
}

TEST_CASE("lunar_mission_legs: the LLO mission's legs in flight order", "[refuelling][campaign][lunar]") {
    const auto m = config::load_lunar_landing_mission(config_dir, "hls_lunar_llo_depot");
    mission::OutboundPlan plan;
    plan.tli.delta_v_gcrf_mps = {3100.0, 0.0, 0.0};
    plan.insertion.delta_v_gcrf_mps = {0.0, 900.0, 0.0};
    const auto legs = sim::lunar_mission_legs(m, plan, 375.0, 330.0);
    REQUIRE(legs.size() == 5);
    CHECK(legs[0].name == "TLI");
    CHECK(legs[0].delta_v_mps == Catch::Approx(3100.0));
    CHECK(legs[1].name == "LOI");
    CHECK(legs[1].delta_v_mps == Catch::Approx(900.0));
    CHECK(legs[3].isp_s == Catch::Approx(330.0));  // the approach on the landing set
    CHECK(legs[4].name == "Aufstieg");
    CHECK(legs[2].delta_v_mps + legs[3].delta_v_mps == Catch::Approx(m.descent_delta_v_mps));
}

TEST_CASE("The measured-numbers file loads and matches its vehicles", "[refuelling][campaign][config]") {
    const auto r = config::load_refuelling_reference(config_dir);
    CHECK(r.depot.mission == "depot_launch");
    CHECK(r.depot.initial_propellant_kg == Catch::Approx(55.5e3));
    CHECK(r.tanker.mission == "tanker_flight");
    CHECK(r.tanker.delivered_oxidizer_kg + r.tanker.delivered_fuel_kg == Catch::Approx(67.5e3));
    CHECK(r.tanker.dry_mass_sensitivity == Catch::Approx(0.99));
    CHECK(r.hls.mission == "hls_refuel");
    CHECK(r.hls.main_at_docking_kg == Catch::Approx(184.5e3));
    // the reference tanker is the configured one
    const auto tanker = config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker"));
    CHECK(tanker.stages.back().dry_mass_kg == Catch::Approx(r.tanker.dry_mass_kg));
}

