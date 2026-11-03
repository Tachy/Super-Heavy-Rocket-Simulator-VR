// The HLS (lunar l2): masses, landing thrusters sized to the lunar weight, fixed legs.

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/gnc/lunar_landing_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

// Lunar surface gravity GM / R^2 = 1.624 m/s^2.
constexpr double lunar_g_mps2 = constants::moon_gm_m3ps2 / (constants::moon_mean_radius_m * constants::moon_mean_radius_m);

}  // namespace

TEST_CASE("The HLS has the Block 3 tanks, 120 t dry and no entry hardware", "[vehicle][hls]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    REQUIRE(cfg.stages.size() == 1);
    const auto& s = cfg.stages.front();
    CHECK(s.dry_mass_kg == 120000.0);
    double propellant = 0.0;
    for (const auto& t : s.tanks) propellant += t.mass_kg;
    CHECK(propellant == Catch::Approx(1.6e6));
    CHECK(s.flaps.empty());
    CHECK(s.heat_shield.nose_radius_m == 0.0);
    CHECK(s.catch_pins.azimuths_rad.empty());
    CHECK(s.landing_legs.fixed);
    CHECK(s.landing_legs.azimuths_rad.size() == 6);
}

TEST_CASE("The Moon GM constant matches the DE440 kernel", "[vehicle][hls]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    CHECK(constants::moon_gm_m3ps2 == Catch::Approx(ephemeris.gm_m3ps2(ephemeris::Body::Moon)).epsilon(1e-12));
}

TEST_CASE("HLS landing thrusters take over from one Raptor and hover the real landing masses", "[vehicle][hls]") {
    const vehicle::Vehicle v(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls"), STARSIM_CONFIG_DIR);
    double max_axial = 0.0, min_axial = 0.0;
    int n = 0;
    for (const auto& e : v.stages().front().engines) {
        if (!e.landing_only) continue;
        ++n;
        const auto& c = e.performance->config();
        max_axial += e.performance->thrust_n(1.0, 0.0) * e.thrust_direction().x;
        min_axial += e.performance->thrust_n(c.throttle_min, 0.0) * e.thrust_direction().x;
        CHECK(e.gimbal_point_m.x > 30.0);  // high on the hull
    }
    std::printf("[hls] landing thrust %.0f-%.0f kN axial: hovers %.0f-%.0f t\n", min_axial / 1e3, max_axial / 1e3,
                min_axial / lunar_g_mps2 / 1e3, max_axial / lunar_g_mps2 / 1e3);
    // l4 mass correction: landing 260-340 t (not ~660 t); the braking drops from one sea-level Raptor (minimum 874 kN)
    // onto them, so their maximum must exceed that minimum / set_drop_fraction (0.9).
    CHECK(n == 12);
    double raptor_min = 0.0;
    for (const auto& e : v.stages().front().engines) {
        if (e.group_id == "hls/sea_level_1") raptor_min = e.performance->thrust_n(e.performance->config().throttle_min, 0.0);
    }
    REQUIRE(raptor_min > 0.0);
    CHECK(max_axial > raptor_min / 0.9);
    CHECK(min_axial / lunar_g_mps2 < 0.95 * 260e3);  // hovers the lightest landing, -5 %
    CHECK(max_axial / lunar_g_mps2 > 1.5 * 340e3);   // lifts the heaviest with 1.5 lunar weights
}

TEST_CASE("Fixed legs are deployed from the start and stay deployed", "[vehicle][hls][legs]") {
    vehicle::Vehicle v(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls"), STARSIM_CONFIG_DIR);
    CHECK(v.has_landing_legs());
    CHECK(v.leg_deployment() == 1.0);
    v.update_landing_legs(1.0);
    CHECK(v.leg_deployment() == 1.0);
}

TEST_CASE("An upper stage with fixed legs keeps them out after separation", "[vehicle][hls][legs]") {
    auto stack = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto hls = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    stack.stages[1] = hls.stages[0];  // Super Heavy + HLS
    stack.engines.insert(hls.engines.begin(), hls.engines.end());
    vehicle::Vehicle v(stack, STARSIM_CONFIG_DIR);
    CHECK(v.leg_deployment() == 0.0);  // the booster's front stage has no legs
    const auto upper = v.separate(1);
    REQUIRE(upper->has_landing_legs());
    CHECK(upper->leg_deployment() == 1.0);
}

TEST_CASE("The truth thrust factor scales the force, not the performance the guidance reads", "[vehicle][hls][dispersion]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_pad2")));
    const auto start = test::state_over_site(session.launch_site(), earth, t0, {0.0, 0.0, 2.0e6}, {});  // vacuum
    const auto body = [&](double factor) {
        auto v = std::make_unique<vehicle::Vehicle>(config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls"),
                                                    STARSIM_CONFIG_DIR);
        v->set_thrust_factor(factor);
        return vehicle::FlightBody(std::move(v), start);
    };
    vehicle::FlightBody coast = body(1.0), nominal = body(1.0), dispersed = body(1.02);
    vehicle::VehicleCommand burn;
    burn.groups = {{"hls/vacuum", true, 1.0}};
    for (int i = 0; i < 1000; ++i) {  // 5 s: spool-up and full thrust
        const auto epoch = t0 + i * 0.005;
        coast.step(session.environment(), epoch, 0.005, {});
        nominal.step(session.environment(), epoch, 0.005, burn);
        dispersed.step(session.environment(), epoch, 0.005, burn);
    }
    const double dv_nominal = math::norm(nominal.state().velocity_gcrf_mps - coast.state().velocity_gcrf_mps);
    const double dv_dispersed = math::norm(dispersed.state().velocity_gcrf_mps - coast.state().velocity_gcrf_mps);
    std::printf("[dispersion] dv %.4f vs %.4f m/s\n", dv_nominal, dv_dispersed);
    REQUIRE(dv_nominal > 10.0);
    CHECK(dv_dispersed / dv_nominal == Catch::Approx(1.02).margin(0.001));
    // Mass flow scales too (same Isp); the guidance's performance query stays nominal.
    const double m0 = coast.vehicle().mass_properties().mass_kg;
    const double used_nominal = m0 - nominal.vehicle().mass_properties().mass_kg;
    const double used_dispersed = m0 - dispersed.vehicle().mass_properties().mass_kg;
    CHECK(used_dispersed / used_nominal == Catch::Approx(1.02).margin(1e-6));
    const auto& engines = dispersed.vehicle().stages().front().engines;
    const auto& nominal_engines = nominal.vehicle().stages().front().engines;
    for (size_t i = 0; i < engines.size(); ++i) {
        CHECK(engines[i].performance->thrust_n(1.0, 0.0) == nominal_engines[i].performance->thrust_n(1.0, 0.0));
    }
}

// Planner plan 2026-11-03 Task 6: the approach's lateral effort and the ascent's liftoff heights and abort margin are
// vehicle keys ([stage.lunar_landing], [stage.lunar_ascent], [E] in starship_hls.toml); the approach's phase times are
// no longer a fixed guess (the planner's ballistic guess from the landing set's T/W and the hand-over state).
TEST_CASE("HLS landing and ascent parameters follow config", "[vehicle][hls][config]") {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    auto& s = cfg.stages.front();
    REQUIRE(s.lunar_landing);
    REQUIRE(s.lunar_ascent);
    CHECK(s.lunar_landing->lateral_effort_fraction == 0.05);
    CHECK(s.lunar_ascent->raptor_start_height_m == 30.0);
    CHECK(s.lunar_ascent->vertical_rise_height_m == 150.0);
    CHECK(s.lunar_ascent->abort_delta_v_margin == 0.15);
    // a changed key reaches the guidance parameters; the mission's values on the base stay
    s.lunar_landing->lateral_effort_fraction = 0.07;
    s.lunar_ascent->raptor_start_height_m = 40.0;
    s.lunar_ascent->vertical_rise_height_m = 200.0;
    s.lunar_ascent->abort_delta_v_margin = 0.2;
    CHECK(gnc::lunar_landing_parameters(s).approach.lateral_effort_fraction == 0.07);
    gnc::LunarAscentParameters mission;
    mission.insertion_altitude_above_site_m = 12.0e3;
    const auto ascent = gnc::lunar_ascent_parameters(s, mission);
    CHECK(ascent.raptor_start_height_m == 40.0);
    CHECK(ascent.vertical_rise_height_m == 200.0);
    CHECK(ascent.abort_delta_v_margin == 0.2);
    CHECK(ascent.insertion_altitude_above_site_m == 12.0e3);
    // the refuelled launch variant reuses the HLS stage with its keys
    const auto launch = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls_launch");
    CHECK(launch.stages.back().lunar_landing.has_value());
    CHECK(launch.stages.back().lunar_ascent.has_value());
    // a stage that is no lunar lander has none: the guidance refuses it instead of using a default
    const auto block3 = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    for (const auto& stage : block3.stages) {
        CHECK_FALSE(stage.lunar_landing.has_value());
        CHECK_THROWS_AS(gnc::lunar_landing_parameters(stage), std::runtime_error);
        CHECK_THROWS_AS(gnc::lunar_ascent_parameters(stage), std::runtime_error);
    }
}

TEST_CASE("A lunar lander without its landing or ascent keys is refused", "[vehicle][hls][config]") {
    namespace fs = std::filesystem;
    const fs::path source(STARSIM_CONFIG_DIR);
    for (const std::string key : {"lateral_effort_fraction", "raptor_start_height_m", "abort_delta_v_margin"}) {
        const auto dir = fs::temp_directory_path() / ("starsim_lunar_lander_keys_" + key);
        fs::remove_all(dir);
        fs::create_directories(dir / "vehicles");
        fs::copy(source / "engines", dir / "engines", fs::copy_options::recursive);
        fs::copy_file(source / "propellants.toml", dir / "propellants.toml");
        std::ifstream in(source / "vehicles" / "starship_hls.toml");
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const auto at = text.find("\n" + key + " =");
        REQUIRE(at != std::string::npos);
        text.insert(at + 1, "# ");
        std::ofstream(dir / "vehicles" / "starship_hls.toml") << text;
        try {
            config::load_vehicle(dir, "starship_hls");
            FAIL("loaded without " << key);
        } catch (const config::ConfigError& e) {
            const std::string what = e.what();
            CHECK(what.find("Mondlandung") != std::string::npos);  // German, naming the key and the file
            CHECK(what.find(key) != std::string::npos);
            CHECK(what.find("starship_hls.toml") != std::string::npos);
        }
        fs::remove_all(dir);
    }
}
