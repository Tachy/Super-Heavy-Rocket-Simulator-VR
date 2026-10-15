// Powered descent from PDI (l4): an HLS on a 15 km x 100 km polar ellipse, 450 km uprange of the touchdown point,
// brakes on the Apollo law, flies the SCvx approach and lands on the LOLA terrain within 1 m at <= 1 m/s.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/lunar_landing_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "lunar_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    std::shared_ptr<const environment::LunarTerrain> terrain;
    mission::TouchdownPoint point;
    double azimuth = 0.0;

    Fixture() {
        terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);  // as test_hls_landing.cpp
        session.set_moon_terrain(terrain);
        const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
        session.set_lunar_landing_site(site);
        azimuth = 180.0 * deg;  // a polar pass heading south over the site
        point = *mission::select_touchdown_point(*terrain, site.config.latitude_rad, site.config.longitude_rad, azimuth,
                                                 12.0, mission::LunarDescentParameters{});
    }

    void place_at_pdi(double propellant_kg, double range_m = 450e3) {
        test::place_hls_at_pdi(session, ephemeris, moon_me, point, t0, propellant_kg, 0.0, range_m);
    }
};
}  // namespace

TEST_CASE("The HLS brakes from PDI and lands on the LOLA terrain within 1 m",
          "[sim][lunar_descent_flight][terrain][data][slow]") {
    Fixture fx;
    fx.place_at_pdi(1065e3);  // ~1,185 t in the LLO after the DOI (LLO variant)
    const double m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(fx.session.begin_powered_descent(fx.point, fx.azimuth));
    double braking_s = -1.0;
    // 0.1 s steps: epoch() + tick_s alone rounds below one tick and advance_to then does nothing
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
        const auto* g = fx.session.lunar_landing_guidance();
        if (braking_s < 0.0 && g && g->phase() != gnc::LunarDescentPhase::Braking) braking_s = fx.session.epoch() - fx.t0;
    }
    fx.session.advance_to(fx.session.epoch() + 5.0, 2000);
    const auto& td = fx.session.lunar_touchdown();
    REQUIRE(td);
    const double m1 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    const double used_t = (m0 - m1) / 1000.0;
    const double dv = 350.0 * constants::standard_gravity_mps2 * std::log(m0 / m1);
    std::printf("[lunar_descent] %s: offset %.2f m, sink %.2f m/s, lateral %.2f m/s, braking %.0f s, total %.0f s, "
                "%.0f t used (%.0f m/s at 350 s), %s\n",
                sim::to_string(td->outcome), td->offset_m, td->sink_mps, td->lateral_mps, braking_s,
                fx.session.epoch() - fx.t0, used_t, dv, fx.session.lunar_status().c_str());
    CHECK(fx.session.lunar_step() == sim::LunarStep::Landed);
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);  // spec
    CHECK(td->sink_mps <= 1.0 + 0.1);
    CHECK(braking_s > 240.0);  // spec ~6-7 min with Apollo-like targets; Raptors are stronger - recorded
    CHECK(dv == Catch::Approx(2000.0).margin(250.0));  // spec ~2.0 km/s (incl. the approach)
}

TEST_CASE("Powered descent is refused without a lunar landing site", "[sim][lunar_descent_flight]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    session.place_in_orbit("starship_hls", orbit::state_from_elements({6578137.0, 0.0, 0.5, 0.0, 0.0, 0.0},
                                                                       ephemeris.gm_m3ps2(ephemeris::Body::Earth)),
                           time::Epoch::from_tdb_seconds_past_j2000(8.4e8));
    CHECK_FALSE(session.begin_powered_descent({}, 0.0));
    CHECK(session.lunar_status().find("Landestelle") != std::string::npos);
}
