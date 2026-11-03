// Return parameters of a first stage from its config (milestone c3, planner plan Task 5): the return profile
// ([stage.recovery], [stage.recovery.drone_ship]), the grid fins' steering limit, the legs' / catch pins' touchdown
// limits, and the values derived from the vehicle (catch approach time, the flip's turn authority and duration).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/gnc/vehicle_capability.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/propulsion/engine_performance.hpp"
#include "starsim/propulsion/rcs_allocator.hpp"
#include "starsim/vehicle/stage_inertia.hpp"

using namespace starsim;
using Catch::Approx;
using Groups = std::vector<std::string>;

namespace {

constexpr double deg = constants::deg_to_rad;

// Throttle range time of one engine type: (max - min) / rate.
double range_time_s(const config::EngineConfig& e) { return (e.throttle_max - e.throttle_min) / e.throttle_rate_per_s; }

}  // namespace

TEST_CASE("Falcon 9 return parameters come from its recovery config", "[gnc][falcon9][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto p = gnc::booster_return_parameters(cfg, cfg.stages.front());
    CHECK(p.boostback_groups == Groups{"first_stage/center", "first_stage/pair"});
    CHECK(p.entry_groups == Groups{"first_stage/center"});
    CHECK(p.landing_ignition_groups == Groups{"first_stage/center"});
    CHECK(p.landing_divert_groups == Groups{"first_stage/center", "first_stage/pair"});
    CHECK(p.landing_groups == Groups{"first_stage/center"});
    CHECK(p.flight_time_s == 260.0);
    CHECK(p.boostback_loss_fraction == 0.157);
    CHECK(p.flip_groups.empty());  // the flip on the nitrogen thrusters, engines off
    CHECK(p.flip_hold_s == 4.0);
    CHECK(p.entry_altitude_m == 56600.0);
    CHECK(p.entry_delta_v_mps == 125.0);
    CHECK(p.entry_throttle == 0.4);
    CHECK(p.scvx.ignition_throttle == 0.7);
    CHECK(p.scvx.settings.limits.switches[0].min_sink_mps == 230.0);
    CHECK(p.scvx.settings.limits.switches[1].max_sink_mps == 190.0);
    CHECK(p.scvx.settings.limits.switches[1].max_tilt_rad == 0.0);  // no switch attitude limits set
    CHECK(p.scvx.divert_speed_mps == 250.0);
    CHECK(p.scvx.switch_speed_mps == 165.0);
    // K: grid fins and legs
    CHECK(p.aero_steering_max_alpha_rad == Approx(8.0 * deg).epsilon(1e-12));
    CHECK(p.touchdown_tilt_rad == Approx(4.0 * deg).epsilon(1e-12));
    CHECK(p.scvx.settings.limits.terminal_tilt_rad == p.touchdown_tilt_rad);
    CHECK(p.leg_deploy_margin_s == 5.0);
    // D: the approach time is the Merlin's throttle range, (1 - 0.4) / 0.5 = 1.2 s (a pad landing flies none)
    CHECK(p.approach_time_constant_s == Approx(range_time_s(cfg.engine("merlin1d"))).epsilon(1e-12));
    CHECK(p.approach_time_constant_s == Approx(1.2).epsilon(1e-12));
}

TEST_CASE("Super Heavy return parameters come from its config", "[gnc][booster][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& stage = cfg.stages.front();
    const auto p = gnc::booster_return_parameters(cfg, stage);
    // K: the profile
    CHECK(p.boostback);
    CHECK(p.boostback_groups == Groups{"booster/center", "booster/divert", "booster/middle"});
    CHECK(p.landing_ignition_groups == Groups{"booster/center", "booster/divert", "booster/middle"});
    CHECK(p.landing_divert_groups == Groups{"booster/center", "booster/divert"});
    CHECK(p.landing_groups == Groups{"booster/center"});
    CHECK(p.flip_groups == p.landing_groups);  // the centre engines into the flip
    CHECK(p.flight_time_s == 230.0);
    CHECK(p.boostback_loss_fraction == 0.086);
    CHECK(p.entry_altitude_m == 0.0);
    CHECK(p.scvx.ignition_throttle == 0.35);
    CHECK(p.scvx.divert_throttle == 0.4);
    CHECK(p.scvx.final_throttle == 0.4);
    const auto& sw = p.scvx.settings.limits.switches;
    CHECK(sw[0].min_sink_mps == 180.0);
    CHECK(sw[0].max_sink_mps == 220.0);
    CHECK(sw[0].max_tilt_rad == 0.0);
    CHECK(sw[1].min_sink_mps == 15.0);
    CHECK(sw[1].max_sink_mps == 30.0);
    CHECK(sw[1].max_tilt_rad == Approx(3.0 * deg).epsilon(1e-12));
    CHECK(sw[1].max_rate_radps == 0.03);
    CHECK(p.scvx.divert_speed_mps == 200.0);  // the burn prediction switches in the windows' middle
    CHECK(p.scvx.switch_speed_mps == 22.5);
    CHECK(p.aero_steering_max_alpha_rad == Approx(8.0 * deg).epsilon(1e-12));
    CHECK(p.touchdown_tilt_rad == Approx(4.0 * deg).epsilon(1e-12));  // the catch pins' key
    CHECK(p.scvx.settings.limits.terminal_tilt_rad == p.touchdown_tilt_rad);

    // D: catch approach, the Raptor's throttle range (1 - 0.3) / 0.5 = 1.4 s, in the explicit law and the SCvx plans
    const auto& raptor = cfg.engine("raptor3");
    CHECK(p.approach_time_constant_s == Approx(range_time_s(raptor)).epsilon(1e-12));
    CHECK(p.approach_time_constant_s == Approx(1.4).epsilon(1e-12));
    CHECK(p.scvx.settings.limits.approach_time_constant_s == p.approach_time_constant_s);

    // D: the flip's turn authority by hand - three centre Raptors at minimum throttle in vacuum, full gimbal, about the
    // CG of the stage alone at the flown separation load (293 t, our full return)
    const double propellant_kg = 293.0e3;
    const auto mass = vehicle::settled_stage_mass_properties(cfg, stage, propellant_kg);
    const propulsion::EnginePerformance engine(raptor);
    const config::EngineGroupConfig* centre = nullptr;
    for (const auto& g : stage.engine_groups) {
        if (g.id == "center") centre = &g;
    }
    REQUIRE(centre);
    const double lever_m = mass.cg_station_m - centre->station_m;
    const double alpha = 3.0 * engine.thrust_n(raptor.throttle_min, 0.0) * std::sin(raptor.gimbal_max_rad) * lever_m /
                         mass.pitch_inertia_kg_m2;
    CHECK(gnc::booster_flip_acceleration_radps2(cfg, stage, p, propellant_kg) == Approx(alpha).epsilon(1e-12));
    // Rest-to-rest bang-bang through 153 deg (the flown turn at the separation), lit within the 15 deg tolerance:
    // T = 2 sqrt(theta / alpha), the last 15 deg take sqrt(2 tol / alpha).
    const double theta = 153.0 * deg, tol = p.flip_tolerance_rad;
    const double flip_s = 2.0 * std::sqrt(theta / alpha) - std::sqrt(2.0 * tol / alpha);
    CHECK(gnc::booster_flip_s(p, alpha, theta) == Approx(flip_s).epsilon(1e-12));
    std::printf("[flip] Super Heavy: I %.4g kg m2, CG %.2f m, lever %.2f m, alpha %.4f rad/s2, 153 deg -> %.2f s "
                "(flown 10.43 s, the former fixed 10 s)\n",
                mass.pitch_inertia_kg_m2, mass.cg_station_m, lever_m, alpha, flip_s);
    // physical band: the flown flip (10.4 s, attitude controller) - the bang-bang turn at full authority is not slower
    CHECK(flip_s > 5.0);
    CHECK(flip_s < 11.0);
}

TEST_CASE("Falcon 9's flip time follows its RCS and the free turn", "[gnc][falcon9][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& stage = cfg.stages.front();
    const auto p = gnc::booster_return_parameters(cfg, stage);
    // by hand: two 6.5 kN thrusters on the pods' lever about the CG of the stage alone at the flown separation load
    const double propellant_kg = 52.0e3;
    const auto mass = vehicle::settled_stage_mass_properties(cfg, stage, propellant_kg);
    double pod_station_m = 0.0, pod_thrust_n = 0.0;
    for (const auto& t : stage.rcs.thrusters) {
        pod_station_m = t.position_m.x;
        pod_thrust_n = t.thrust_n;
    }
    // the allocator's pure torque about a transverse axis: one tangential thruster per pod, opposite pods (a couple)
    // plus the inward ones on the lever - taken from the allocator itself, checked against the 2-thruster lever bound
    const double alpha = gnc::booster_flip_acceleration_radps2(cfg, stage, p, propellant_kg);
    const double lever_bound = 2.0 * pod_thrust_n * (pod_station_m - mass.cg_station_m) / mass.pitch_inertia_kg_m2;
    CHECK(alpha == Approx(gnc::rcs_pitch_acceleration_radps2(stage, mass.pitch_inertia_kg_m2, mass.cg_station_m)).epsilon(1e-12));
    CHECK(alpha > 0.5 * lever_bound);
    CHECK(alpha < 2.5 * lever_bound);
    // free turn to the ignition angle, capped at flip_max_rate, after the hold: hold + w / a + (theta - w^2 / 2a) / w
    const double w = p.flip_max_rate_radps, theta = p.flip_ignition_off_velocity_rad;
    REQUIRE(w * w / (2.0 * alpha) < theta);
    const double flip_s = p.flip_hold_s + w / alpha + (theta - w * w / (2.0 * alpha)) / w;
    CHECK(gnc::booster_flip_s(p, alpha, 0.0) == Approx(flip_s).epsilon(1e-12));
    std::printf("[flip] Falcon 9: I %.4g kg m2, CG %.2f m, alpha %.4f rad/s2 (%.2f deg/s2), flip %.2f s (timelines "
                "13.7 +-0.6 s, flown 14.79 s, the former fitted 14.8 s)\n",
                mass.pitch_inertia_kg_m2, mass.cg_station_m, alpha, alpha / deg, flip_s);
    // physical band: the measured turn 1.6-1.9 deg/s2 (Crew-13 frames, falcon9.md); the timelines 13.7 +-0.6 s, below
    // the 17 s timeout
    CHECK(alpha / deg > 1.4);
    CHECK(alpha / deg < 2.4);
    CHECK(flip_s > 12.5);
    CHECK(flip_s < 17.0);
}

TEST_CASE("Turn time helpers: free turn and rest-to-rest within a tolerance", "[gnc][config]") {
    const double a = 0.2, w = 0.1, pi = constants::pi;
    // free turn: below the cap a pure ramp, above it ramp + coast
    CHECK(gnc::free_turn_time_s(0.01, a, w) == Approx(std::sqrt(2.0 * 0.01 / a)).epsilon(1e-12));
    CHECK(gnc::free_turn_time_s(pi, a, w) == Approx(w / a + (pi - 0.5 * w * w / a) / w).epsilon(1e-12));
    CHECK(gnc::free_turn_time_s(pi, a, 0.0) == Approx(std::sqrt(2.0 * pi / a)).epsilon(1e-12));
    // rest-to-rest within a tolerance: the triangle's last tol takes sqrt(2 tol / a); capped: the coast ends earlier
    CHECK(gnc::turn_time_within_s(pi, 0.0, a, 0.0) == Approx(gnc::flip_duration_s(pi, a, 0.0)).epsilon(1e-12));
    CHECK(gnc::turn_time_within_s(pi, 0.3, a, 0.0) ==
          Approx(gnc::flip_duration_s(pi, a, 0.0) - std::sqrt(2.0 * 0.3 / a)).epsilon(1e-12));
    CHECK(gnc::turn_time_within_s(pi, 2.0, a, 0.0) == Approx(std::sqrt(2.0 * (pi - 2.0) / a)).epsilon(1e-12));
    CHECK(gnc::turn_time_within_s(pi, 1.0, a, w) == Approx(w / a + (pi - 1.0 - 0.5 * w * w / a) / w).epsilon(1e-12));
    CHECK(gnc::turn_time_within_s(pi, 4.0, a, w) == 0.0);
    CHECK_THROWS_AS(gnc::free_turn_time_s(1.0, 0.0, w), std::invalid_argument);
}

TEST_CASE("Falcon 9 drone-ship returns fly their own profile", "[gnc][falcon9][asds][config]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& stage = cfg.stages.front();
    REQUIRE(stage.drone_ship_return);
    const auto& profile = stage.return_profile_for(true);
    CHECK(&profile == &*stage.drone_ship_return);
    CHECK(&stage.return_profile_for(false) == &stage.return_profile);
    const auto p = gnc::booster_return_parameters(cfg, stage, true);
    CHECK_FALSE(p.boostback);
    CHECK(p.entry_groups == Groups{"first_stage/center", "first_stage/pair"});
    CHECK(p.entry_altitude_m == 66000.0);
    CHECK(p.entry_delta_v_mps == 880.0);
    CHECK(p.landing_ignition_groups == Groups{"first_stage/center"});
    CHECK(p.landing_divert_groups == Groups{"first_stage/center"});
    CHECK(p.landing_groups == Groups{"first_stage/center"});
    CHECK(std::abs(p.scvx.settings.limits.tilt_max_rad[2] - 20.0 * deg) < 1e-12);
    // Keys the drone-ship table leaves out come from the RTLS profile (Review Focus 3) ...
    const auto rtls = gnc::booster_return_parameters(cfg, stage);
    CHECK(p.boostback_groups == Groups{"first_stage/center", "first_stage/pair"});
    CHECK(p.flight_time_s == rtls.flight_time_s);
    CHECK(p.boostback_loss_fraction == rtls.boostback_loss_fraction);
    CHECK(p.flip_ignition_off_velocity_rad == rtls.flip_ignition_off_velocity_rad);
    CHECK(p.flip_max_rate_radps == rtls.flip_max_rate_radps);
    CHECK(p.scvx.settings.limits.gimbal_fraction == rtls.scvx.settings.limits.gimbal_fraction);
    // ... and the stage's own keys and derived values are the same on both
    CHECK(p.aero_steering_max_alpha_rad == rtls.aero_steering_max_alpha_rad);
    CHECK(p.touchdown_tilt_rad == rtls.touchdown_tilt_rad);
    CHECK(p.leg_deploy_margin_s == rtls.leg_deploy_margin_s);
    CHECK(p.approach_time_constant_s == rtls.approach_time_constant_s);
    // The RTLS profile is untouched.
    CHECK(rtls.boostback);
    CHECK(rtls.entry_groups == Groups{"first_stage/center"});
    CHECK(rtls.entry_delta_v_mps == 125.0);
}

TEST_CASE("Super Heavy has no drone-ship profile", "[gnc][booster]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    CHECK_FALSE(cfg.stages.front().drone_ship_return);
    CHECK(&cfg.stages.front().return_profile_for(true) == &cfg.stages.front().return_profile);
    CHECK(gnc::booster_return_parameters(cfg, cfg.stages.front(), true).boostback);
}

TEST_CASE("Reused Super Heavy stages inherit the return keys", "[gnc][booster][config]") {
    const auto block3 = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto own = gnc::booster_return_parameters(block3, block3.stages.front());
    for (const char* id : {"starship_tanker", "starship_hls_launch", "starship_depot"}) {
        CAPTURE(id);
        const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, id);
        if (cfg.stages.front().id != "booster") continue;  // the depot flies alone (no booster)
        const auto p = gnc::booster_return_parameters(cfg, cfg.stages.front());
        CHECK(p.flight_time_s == own.flight_time_s);
        CHECK(p.boostback_loss_fraction == own.boostback_loss_fraction);
        CHECK(p.scvx.ignition_throttle == own.scvx.ignition_throttle);
        CHECK(p.scvx.settings.limits.switches[1].max_tilt_rad == own.scvx.settings.limits.switches[1].max_tilt_rad);
        CHECK(p.aero_steering_max_alpha_rad == own.aero_steering_max_alpha_rad);
        CHECK(p.touchdown_tilt_rad == own.touchdown_tilt_rad);
        CHECK(p.approach_time_constant_s == own.approach_time_constant_s);
    }
}

TEST_CASE("A returning booster without its return keys is refused", "[gnc][booster][falcon9][config]") {
    namespace fs = std::filesystem;
    const fs::path source(STARSIM_CONFIG_DIR);
    struct Case {
        const char* vehicle;
        const char* key;
    };
    for (const auto& c : {Case{"starship_block3", "return_flight_time_s"}, Case{"starship_block3", "landing_phase_throttles"},
                          Case{"starship_block3", "max_return_alpha_deg"}, Case{"starship_block3", "max_catch_tilt_deg"},
                          Case{"falcon9_block5", "boostback_loss_fraction"}, Case{"falcon9_block5", "deploy_margin_s"},
                          Case{"falcon9_block5", "max_touchdown_tilt_deg"}}) {
        const std::string key = c.key;
        CAPTURE(c.vehicle, key);
        const auto dir = fs::temp_directory_path() / ("starsim_booster_return_keys_" + std::string(c.vehicle) + "_" + key);
        fs::remove_all(dir);
        fs::create_directories(dir / "vehicles");
        fs::copy(source / "engines", dir / "engines", fs::copy_options::recursive);
        fs::copy_file(source / "propellants.toml", dir / "propellants.toml");
        const std::string file = std::string(c.vehicle) + ".toml";
        std::ifstream in(source / "vehicles" / file);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const auto at = text.find("\n" + key + " =");  // the first stage's (it comes first in the file)
        REQUIRE(at != std::string::npos);
        text.insert(at + 1, "# ");
        std::ofstream(dir / "vehicles" / file) << text;
        try {
            config::load_vehicle(dir, c.vehicle);
            FAIL("loaded without " << key);
        } catch (const config::ConfigError& e) {
            const std::string what = e.what();
            CHECK(what.find("Rückkehr") != std::string::npos);  // German, naming the key and the file
            CHECK(what.find(key) != std::string::npos);
            CHECK(what.find(file) != std::string::npos);
        }
        fs::remove_all(dir);
    }
    // vehicles without a returning first stage load as before
    for (const char* id : {"crew_dragon", "parachutist", "starship_hls", "iss"}) {
        CHECK_NOTHROW(config::load_vehicle(STARSIM_CONFIG_DIR, id));
    }
}

TEST_CASE("Only Falcon 9's fresh landing plans take their own trust-region weight", "[gnc][falcon9][asds][rtls][booster][ship]") {
    const auto f9 = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& stage = f9.stages.front();
    const double weight = stage.return_profile.landing_fresh_trust_region_weight;
    CHECK(weight > gnc::DescentWeights{}.trust_region);
    // RTLS and drone ship alike; the weight of the re-plans stays the default
    const auto rtls = gnc::booster_return_parameters(f9, stage).scvx.settings;
    const auto deck = gnc::booster_return_parameters(f9, stage, true).scvx.settings;
    CHECK(rtls.fresh_trust_region_weight == weight);
    CHECK(deck.fresh_trust_region_weight == weight);
    CHECK(deck.weights.trust_region == gnc::DescentWeights{}.trust_region);
    const auto sh = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    CHECK(gnc::booster_return_parameters(sh, sh.stages.front()).scvx.settings.fresh_trust_region_weight == 0.0);
    CHECK(gnc::ship_landing_parameters(sh, sh.stages.at(1)).scvx.settings.fresh_trust_region_weight == 0.0);
    // Likewise the tilt penalty on fresh plans
    const double penalty = stage.return_profile.landing_fresh_attitude_violation_weight;
    CHECK(penalty > gnc::DescentWeights{}.attitude_violation);
    CHECK(rtls.fresh_attitude_violation_weight == penalty);
    CHECK(deck.fresh_attitude_violation_weight == penalty);
    CHECK(deck.weights.attitude_violation == gnc::DescentWeights{}.attitude_violation);
    CHECK(gnc::booster_return_parameters(sh, sh.stages.front()).scvx.settings.fresh_attitude_violation_weight == 0.0);
    CHECK(gnc::ship_landing_parameters(sh, sh.stages.at(1)).scvx.settings.fresh_attitude_violation_weight == 0.0);
}

TEST_CASE("Falcon 9 landing plans take their own gimbal allowance, Super Heavy keeps the default",
          "[gnc][falcon9][asds][rtls]") {
    const auto f9 = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    const auto& stage = f9.stages.front();
    CHECK(stage.return_profile.landing_gimbal_fraction == 0.45);
    // one allowance for the one engine: the drone-ship profile inherits it from the RTLS profile
    CHECK(gnc::booster_return_parameters(f9, stage, true).scvx.settings.limits.gimbal_fraction == 0.45);
    CHECK(gnc::booster_return_parameters(f9, stage).scvx.settings.limits.gimbal_fraction == 0.45);
    const auto sh = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    CHECK(gnc::booster_return_parameters(sh, sh.stages.front()).scvx.settings.limits.gimbal_fraction ==
          gnc::DescentLimits{}.gimbal_fraction);
}
