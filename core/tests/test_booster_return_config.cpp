// Return parameters of a first stage from its [stage.recovery] (milestone c3): Falcon 9 brings its own engine sets,
// entry burn and landing-burn profile; Super Heavy keeps the guidance's defaults.

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/booster_return.hpp"

using namespace starsim;
using Groups = std::vector<std::string>;

TEST_CASE("Falcon 9 return parameters come from its recovery config", "[gnc][falcon9]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto p = gnc::booster_return_parameters(cfg.stages.front());
    CHECK(p.boostback_groups == Groups{"first_stage/center", "first_stage/pair"});
    CHECK(p.entry_groups == Groups{"first_stage/center"});
    CHECK(p.landing_ignition_groups == Groups{"first_stage/center"});
    CHECK(p.landing_divert_groups == Groups{"first_stage/center", "first_stage/pair"});
    CHECK(p.landing_groups == Groups{"first_stage/center"});
    CHECK(p.flight_time_s == 260.0);
    CHECK(p.boostback_loss_fraction == 0.19);
    CHECK(p.flip_groups.empty());  // the flip on the nitrogen thrusters, engines off
    CHECK(p.flip_duration_s == 50.0);
    CHECK(p.entry_altitude_m == 56600.0);
    CHECK(p.entry_delta_v_mps == 125.0);
    CHECK(p.entry_throttle == 0.4);
    CHECK(p.scvx.ignition_throttle == 0.7);
    CHECK(p.scvx.settings.limits.switches[0].min_sink_mps == 230.0);
    CHECK(p.scvx.settings.limits.switches[1].max_sink_mps == 190.0);
    CHECK(p.scvx.divert_speed_mps == 250.0);
    CHECK(p.scvx.switch_speed_mps == 165.0);
}

TEST_CASE("Super Heavy keeps the default return parameters", "[gnc][booster]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto p = gnc::booster_return_parameters(cfg.stages.front());
    const gnc::BoosterReturnParameters defaults;
    CHECK(p.boostback_groups == defaults.boostback_groups);
    CHECK(p.landing_ignition_groups == defaults.landing_ignition_groups);
    CHECK(p.landing_divert_groups == defaults.landing_divert_groups);
    CHECK(p.landing_groups == defaults.landing_groups);
    CHECK(p.flip_groups == defaults.flip_groups);  // the centre engines into the flip
    CHECK(p.flight_time_s == defaults.flight_time_s);
    CHECK(p.entry_altitude_m == 0.0);
    CHECK(p.scvx.divert_speed_mps == defaults.scvx.divert_speed_mps);
    CHECK(p.scvx.settings.limits.switches[1].max_tilt_rad == defaults.scvx.settings.limits.switches[1].max_tilt_rad);
}

TEST_CASE("Falcon 9 drone-ship returns fly their own profile", "[gnc][falcon9][asds]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& stage = cfg.stages.front();
    REQUIRE(stage.drone_ship_return);
    const auto& profile = stage.return_profile_for(true);
    CHECK(&profile == &*stage.drone_ship_return);
    CHECK(&stage.return_profile_for(false) == &stage.return_profile);
    const auto p = gnc::booster_return_parameters(stage.id, profile);
    CHECK_FALSE(p.boostback);
    CHECK(p.entry_groups == Groups{"first_stage/center", "first_stage/pair"});
    CHECK(p.entry_altitude_m == 66000.0);
    CHECK(p.entry_delta_v_mps == 880.0);
    CHECK(p.landing_ignition_groups == Groups{"first_stage/center"});
    CHECK(p.landing_divert_groups == Groups{"first_stage/center"});
    CHECK(p.landing_groups == Groups{"first_stage/center"});
    CHECK(std::abs(p.scvx.settings.limits.tilt_max_rad[2] - 20.0 * 3.14159265358979 / 180.0) < 1e-12);
    // Keys the drone-ship table leaves out come from the RTLS profile.
    CHECK(p.boostback_groups == Groups{"first_stage/center", "first_stage/pair"});
    // The RTLS profile is untouched.
    const auto rtls = gnc::booster_return_parameters(stage);
    CHECK(rtls.boostback);
    CHECK(rtls.entry_groups == Groups{"first_stage/center"});
    CHECK(rtls.entry_delta_v_mps == 125.0);
}

TEST_CASE("Super Heavy has no drone-ship profile", "[gnc][booster]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    CHECK_FALSE(cfg.stages.front().drone_ship_return);
    CHECK(&cfg.stages.front().return_profile_for(true) == &cfg.stages.front().return_profile);
    CHECK(gnc::booster_return_parameters(cfg.stages.front()).boostback);
}

TEST_CASE("Falcon 9 drone-ship landing plans take their own gimbal allowance, RTLS and Super Heavy keep the default",
          "[gnc][falcon9][asds][rtls]") {
    const auto f9 = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& stage = f9.stages.front();
    const double default_fraction = gnc::DescentLimits{}.gimbal_fraction;
    CHECK(stage.drone_ship_return->landing_gimbal_fraction == 0.45);
    CHECK(gnc::booster_return_parameters(stage.id, stage.return_profile_for(true)).scvx.settings.limits.gimbal_fraction == 0.45);
    CHECK(gnc::booster_return_parameters(stage).scvx.settings.limits.gimbal_fraction == default_fraction);
    const auto sh = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    CHECK(gnc::booster_return_parameters(sh.stages.front()).scvx.settings.limits.gimbal_fraction == default_fraction);
}
