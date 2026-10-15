// The HLS (lunar l2): masses, landing thrusters sized to the lunar weight, fixed legs.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double lunar_g_mps2 = 4.9028e12 / (1737.4e3 * 1737.4e3);  // GM [S: DE440] / R^2: 1.624 m/s^2

// Landing mass [E]: dry + the ascent propellant to a 100 km LLO (1.9 km/s x 1.1) at Raptor vacuum Isp 350 s.
double landing_mass_kg() { return 120.0e3 * std::exp(1.1 * 1900.0 / (350.0 * constants::standard_gravity_mps2)); }

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

TEST_CASE("HLS landing thrusters lift ~1.5 lunar weights and can hover", "[vehicle][hls]") {
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
    CHECK(n == 12);
    const double weight = landing_mass_kg() * lunar_g_mps2;
    std::printf("[hls] landing mass %.1f t, lunar weight %.0f kN, landing thrust %.0f-%.0f kN axial\n",
                landing_mass_kg() / 1e3, weight / 1e3, min_axial / 1e3, max_axial / 1e3);
    CHECK(max_axial / weight == Catch::Approx(1.5).margin(0.1));
    CHECK(min_axial < weight);  // can hover
    // A single Raptor at minimum throttle exceeds the landing weight: the reason for the landing thrusters
    int raptors = 0;
    for (const auto& e : v.stages().front().engines) {
        if (e.group_id != "hls/sea_level_1") continue;
        ++raptors;
        CHECK(e.performance->thrust_n(e.performance->config().throttle_min, 0.0) > weight);
    }
    CHECK(raptors == 1);
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
