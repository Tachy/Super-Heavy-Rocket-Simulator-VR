// The HLS (lunar l2): masses, landing thrusters sized to the lunar weight, fixed legs.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
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
