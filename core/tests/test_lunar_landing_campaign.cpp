// Lunar landing campaign (spec l4): from PDI with state +-1 km / +-5 m/s per axis, thrust +-2 %, propellant +-5 % and
// touchdown points on the real terrain (the admissible point nearest to a random spot within 1 km of the site): 21 / 21
// land within 1 m at <= 1 m/s. Aborts from 3 / 1 / 0.3 km reach orbit. Single cases: "[.lunar_case]" with
// STARSIM_LUNAR_CASE=<seed>.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "lunar_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double approach_azimuth_rad = 180.0 * deg;  // polar pass heading south over the site
constexpr double footprint_radius_m = 12.0;          // HLS legs (l2)

struct Dispersion {
    Vec3 position_m;               // added to the PDI state (GCRF), each axis +-1 km
    Vec3 velocity_mps;             // +-5 m/s per axis
    double thrust_factor = 1.0;    // +-2 %
    double mass_factor = 1.0;      // propellant +-5 %
    double east_m = 0.0, north_m = 0.0;  // touchdown target spot (within 1 km of the site)
};

Dispersion dispersion(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    Dispersion d;
    d.position_m = {1000.0 * unit(rng), 1000.0 * unit(rng), 1000.0 * unit(rng)};
    d.velocity_mps = {5.0 * unit(rng), 5.0 * unit(rng), 5.0 * unit(rng)};
    d.thrust_factor = 1.0 + 0.02 * unit(rng);
    d.mass_factor = 1.0 + 0.05 * unit(rng);
    const double r = 1000.0 * std::sqrt(0.5 * (unit(rng) + 1.0)), a = constants::pi * unit(rng);
    d.east_m = r * std::cos(a);
    d.north_m = r * std::sin(a);
    return d;
}

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    std::shared_ptr<const environment::LunarTerrain> terrain;
    sim::LunarSite site;
    Fixture() {
        terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
        session.set_moon_terrain(terrain);
        site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
        session.set_lunar_landing_site(site);
    }
    // The admissible touchdown point nearest to the spot (east, north) metres from the site centre.
    mission::TouchdownPoint point_near(double east_m, double north_m) const {
        const double r = constants::moon_mean_radius_m;
        const double lat = site.config.latitude_rad + north_m / r;
        const double lon = site.config.longitude_rad + east_m / (r * std::cos(site.config.latitude_rad));
        const auto p = mission::select_touchdown_point(*terrain, lat, lon, approach_azimuth_rad, footprint_radius_m,
                                                       mission::LunarDescentParameters{});
        REQUIRE(p);
        return *p;
    }
    double mass_kg() const { return session.body(0).vehicle().mass_properties().mass_kg; }
    double altitude_m() const {
        const auto c = ephemeris.state_gcrf(ephemeris::Body::Moon, session.epoch());
        return norm(session.body(0).state().position_gcrf_m - c.position_m) - constants::moon_mean_radius_m;
    }
};

struct Run {
    sim::LegTouchdown outcome = sim::LegTouchdown::None;
    double offset_m = 0.0, sink_mps = 0.0, m0_t = 0.0, used_t = 0.0, slope_deg = 0.0;
    std::string status;
};

// PDI at the real mass (566 t, propellant x mass factor) with the braking range the planner computes for that mass,
// the dispersion on the state and the engines, the touchdown point near the spot; flies to the leg judgement.
Run fly(const Dispersion& d) {
    Fixture fx;
    const auto point = fx.point_near(d.east_m, d.north_m);
    const double propellant = test::hls_pdi_propellant_kg * d.mass_factor;
    const double range = test::pdi_braking_range_m(fx.ephemeris, point, 120e3 + propellant);
    test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, propellant, 0.0, range, d.position_m,
                           d.velocity_mps);
    fx.session.set_thrust_dispersion(0, d.thrust_factor);
    Run r;
    r.m0_t = fx.mass_kg() / 1000.0;
    r.slope_deg = point.slope_rad / deg;
    REQUIRE(fx.session.begin_powered_descent(point, approach_azimuth_rad));
    // 0.1 s steps (as test_lunar_powered_descent.cpp)
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
    }
    fx.session.advance_to(fx.session.epoch() + 5.0, 2000);
    r.status = fx.session.lunar_status();
    r.used_t = r.m0_t - fx.mass_kg() / 1000.0;
    if (const auto& td = fx.session.lunar_touchdown()) {
        r.outcome = td->outcome;
        r.offset_m = td->offset_m;
        r.sink_mps = td->sink_mps;
    }
    return r;
}

int fly_seeds(unsigned first, unsigned last) {
    int landed = 0;
    for (unsigned seed = first; seed <= last; ++seed) {
        const auto d = dispersion(seed);
        const auto run = fly(d);
        std::printf("[lunar_campaign] case %2u: thrust %.3f, propellant %.3f, spot %.0f/%.0f m, slope %.1f deg: %s, offset %.2f m, "
                    "sink %.2f m/s, %.1f t at PDI, %.1f t used, %s\n",
                    seed, d.thrust_factor, d.mass_factor, d.east_m, d.north_m, run.slope_deg, sim::to_string(run.outcome),
                    run.offset_m, run.sink_mps, run.m0_t, run.used_t, run.status.c_str());
        std::fflush(stdout);
        if (run.outcome == sim::LegTouchdown::Landed && run.offset_m < 1.0 && run.sink_mps <= 1.1) ++landed;
    }
    std::printf("[lunar_campaign] seeds %u-%u: landed %d of %u\n", first, last, landed, last - first + 1);
    return landed;
}
}  // namespace

// Split into three cases so ctest runs them in parallel; together 21 / 21 (spec).
TEST_CASE("Lunar landing campaign: dispersed descents 1-7 land within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    CHECK(fly_seeds(1, 7) == 7);
}

TEST_CASE("Lunar landing campaign: dispersed descents 8-14 land within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    CHECK(fly_seeds(8, 14) == 7);
}

TEST_CASE("Lunar landing campaign: dispersed descents 15-21 land within 1 m",
          "[sim][lunar_campaign][validation][terrain][data][slow]") {
    CHECK(fly_seeds(15, 21) == 7);
}

TEST_CASE("Lunar landing campaign: a single case", "[.lunar_case]") {
    const char* seed = std::getenv("STARSIM_LUNAR_CASE");
    const unsigned s = seed ? static_cast<unsigned>(std::atoi(seed)) : 1u;
    fly_seeds(s, s);
}

TEST_CASE("Aborts from 3, 1 and 0.3 km reach orbit", "[sim][lunar_campaign][abort][validation][terrain][data][slow]") {
    for (const double h : {3000.0, 1000.0, 300.0}) {
        Fixture fx;
        const auto point = fx.point_near(0.0, 0.0);
        const double range = test::pdi_braking_range_m(fx.ephemeris, point, 120e3 + test::hls_pdi_propellant_kg);
        test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, test::hls_pdi_propellant_kg, 0.0,
                               range);
        REQUIRE(fx.session.begin_powered_descent(point, approach_azimuth_rad));
        REQUIRE(test::descend_to(fx.session, fx.ephemeris, h, point));
        const double m0 = fx.mass_kg();
        REQUIRE(fx.session.abort_landing(0));
        fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
        std::printf("[lunar_campaign] abort from %.1f km at %.1f t: %.1f t used, altitude %.1f km, %s\n", h / 1000.0,
                    m0 / 1000.0, (m0 - fx.mass_kg()) / 1000.0, fx.altitude_m() / 1000.0,
                    fx.session.lunar_status().c_str());
        std::fflush(stdout);
        CAPTURE(h);
        CHECK(fx.session.lunar_step() == sim::LunarStep::InLlo);
        CHECK(fx.altitude_m() > 90e3);
    }
}
