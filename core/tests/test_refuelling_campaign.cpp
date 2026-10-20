// Refuelling r6: campaign calculator, its setup from config, the measured-numbers file.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/mission_config.hpp"
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

