// Lunar descent planning (l4): the LLO's first pass of the site within 20 km, the plane trim, the DOI to 15 km above
// the site and the PDI 450 km uprange; touchdown point selection on the LOLA tile; the budget.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double site_lat = -84.7906 * deg, site_lon = 29.1957 * deg;

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    std::shared_ptr<const environment::LunarTerrain> terrain =
        sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);   // as test_hls_landing.cpp loads it

    // Circular polar 100 km LLO (Earth-centred GCRF state) whose plane contains the site at t0 + `align_s`.
    math::StateVector llo(double align_s) {
        const auto t = t0 + align_s;
        const auto rot = moon_me.fixed_to_gcrf(t).rotation;
        const Vec3 pole = rot * Vec3{0.0, 0.0, 1.0};
        const Vec3 site = rot * Vec3{std::cos(site_lat) * std::cos(site_lon), std::cos(site_lat) * std::sin(site_lon),
                                     std::sin(site_lat)};
        const Vec3 n = math::normalized(cross(pole, site));             // polar plane through the site
        const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
        const double r = constants::moon_mean_radius_m + 100e3;
        const Vec3 x = math::normalized(cross(n, pole));                 // in the plane, on the equator
        const Vec3 y = cross(n, x);
        const double speed = std::sqrt(gm / r);
        // at t0 on the equator (the orbit's plane drifts slowly against the rotating Moon: aligned at t0 + align_s)
        const math::StateVector moon_relative{x * r, y * speed};
        const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
        return {moon.position_m + moon_relative.position_m, moon.velocity_mps + moon_relative.velocity_mps};
    }
};
}  // namespace

TEST_CASE("The braking range grows with the PDI mass and stays within its limits", "[mission][lunar_descent_plan]") {
    const mission::BrakingRangeParameters p;
    const double t = 8.94e6;  // three RVacs
    const double r500 = mission::braking_range_m(1690.0, 500e3, t, p), r566 = mission::braking_range_m(1690.0, 566e3, t, p);
    std::printf("[lunar_descent] braking range %.1f km at 500 t, %.1f km at 566 t\n", r500 / 1000.0, r566 / 1000.0);
    CHECK(r566 > r500);
    CHECK(r500 >= p.min_range_m);
    CHECK(mission::braking_range_m(1690.0, 5e6, t, p) == p.max_range_m);
    CHECK(mission::braking_range_m(1690.0, 1e3, t, p) == p.min_range_m);
}

TEST_CASE("The budget closes or names the leg where the propellant runs out", "[mission][lunar_descent_plan]") {
    const auto ok = mission::lunar_budget(566e3, 120e3, 10e3, {{"Abstieg", 2000.0, 350.0}, {"Aufstieg", 1900.0, 350.0}});
    CHECK(ok.closes);
    CHECK(ok.final_mass_kg == Catch::Approx(566e3 * std::exp(-3900.0 / (350.0 * 9.80665))).epsilon(1e-12));
    const auto short_ = mission::lunar_budget(400e3, 204e3, 10e3, {{"Abstieg", 2000.0, 350.0}, {"Aufstieg", 1900.0, 350.0}});
    CHECK_FALSE(short_.closes);
    CHECK(short_.reason.find("Aufstieg") != std::string::npos);
    CHECK(short_.margin_kg < 0.0);
}

TEST_CASE("The required mass closes the budget with zero margin", "[mission][lunar_descent_plan]") {
    const std::vector<mission::BudgetLeg> legs{{"Abstieg", 1000.0, 350.0}, {"Aufstieg", 500.0, 380.0}};
    const double g0 = constants::standard_gravity_mps2;
    const double expected = 110e3 * std::exp(500.0 / (380.0 * g0)) * std::exp(1000.0 / (350.0 * g0));
    const double m = mission::lunar_budget_required_mass(100e3, 10e3, legs);
    CHECK(m == Catch::Approx(expected).epsilon(1e-12));
    const auto b = mission::lunar_budget(m, 100e3, 10e3, legs);
    CHECK(b.closes);
    CHECK(b.margin_kg == Catch::Approx(0.0).margin(1e-6));
    CHECK(mission::lunar_budget_required_mass(100e3, 10e3, {}) == Catch::Approx(110e3));
}

TEST_CASE("The flattest admissible touchdown point near the site is chosen", "[mission][lunar_descent_plan][terrain][data]") {
    Fixture fx;
    mission::LunarDescentParameters p;
    const auto point = mission::select_touchdown_point(*fx.terrain, site_lat, site_lon, 0.0, 12.0, p);
    REQUIRE(point);
    CHECK(point->slope_rad <= p.slope_limit_rad);
    CHECK(point->offset_m <= p.touchdown_search_radius_m);
    std::printf("[lunar_descent] touchdown %.1f m from the site, slope %.2f deg, height %.1f m\n", point->offset_m,
                point->slope_rad / deg, point->height_m);
    // Review focus 4: no admissible point.
    p.slope_limit_rad = 0.01 * deg;
    CHECK_FALSE(mission::select_touchdown_point(*fx.terrain, site_lat, site_lon, 0.0, 12.0, p));
}

TEST_CASE("Descent planning waits for the site's pass, trims the plane and lowers the periselene", "[mission][lunar_descent_plan][terrain][data]") {
    Fixture fx;
    mission::LunarDescentPlanner planner(fx.gravity, fx.ephemeris, fx.moon_me);
    mission::LunarDescentParameters p;
    const auto plan = planner.plan(fx.llo(0.5 * 86400.0), fx.t0, *fx.terrain, site_lat, site_lon, 12.0, p);
    std::printf("[lunar_descent] %s: pass in %.2f d, crossrange %.1f -> %.1f m, trim %.2f m/s, DOI %.2f m/s, "
                "periselene %.2f km, PDI range %.1f km\n",
                plan.feasible ? "feasible" : plan.reason.c_str(), (plan.site_pass_epoch - fx.t0) / 86400.0,
                plan.crossrange_before_trim_m, plan.crossrange_m,
                plan.plane_trim ? norm(plan.plane_trim->delta_v_gcrf_mps) : 0.0, norm(plan.doi.delta_v_gcrf_mps),
                plan.periselene_altitude_m / 1000.0, plan.pdi_range_m / 1000.0);
    REQUIRE(plan.feasible);
    CHECK(plan.site_pass_epoch - fx.t0 < 1.0 * 86400.0);
    CHECK(std::abs(plan.crossrange_m) < p.trim_tolerance_m);
    CHECK(plan.periselene_altitude_m == Catch::Approx(plan.touchdown.height_m + 15e3).margin(p.doi_tolerance_m));
    CHECK(plan.pdi_range_m == Catch::Approx(450e3).margin(5e3));
    CHECK(norm(plan.doi.delta_v_gcrf_mps) == Catch::Approx(20.0).margin(6.0));   // spec: ~20 m/s (Apollo profile)
    CHECK(plan.doi.epoch < plan.pdi_epoch);
    const auto first_burn = plan.plane_trim ? plan.plane_trim->epoch : plan.doi.epoch;
    if (plan.plane_trim) CHECK(plan.plane_trim->epoch < plan.doi.epoch);
    CHECK(first_burn > fx.t0 + p.min_lead_s);
}

TEST_CASE("Descent planning refuses when no pass comes within reach", "[mission][lunar_descent_plan][terrain][data]") {
    Fixture fx;
    mission::LunarDescentPlanner planner(fx.gravity, fx.ephemeris, fx.moon_me);
    mission::LunarDescentParameters p;
    p.search_s = 0.2 * 86400.0;            // the plane is aligned only after 5 days
    const auto plan = planner.plan(fx.llo(5.0 * 86400.0), fx.t0, *fx.terrain, site_lat, site_lon, 12.0, p);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.find("Überflug") != std::string::npos);
}
