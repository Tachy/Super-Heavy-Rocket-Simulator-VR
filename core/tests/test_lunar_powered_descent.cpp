// Powered descent from PDI (l4): an HLS on a 15 km x 100 km polar ellipse, the braking range before the touchdown
// point, brakes on the Apollo law, flies the SCvx approach and lands on the LOLA terrain within 1 m at <= 1 m/s - at
// the real PDI masses (l4 mass correction: 500-566 t).

#include <algorithm>
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
#include "starsim/vehicle/vehicle.hpp"
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

    void place_at_pdi(double propellant_kg, double range_m) {
        test::place_hls_at_pdi(session, ephemeris, moon_me, point, t0, propellant_kg, 0.0, range_m);
    }
    // Periselene speed of the (h_point + 15 km) x 100 km ellipse place_hls_at_pdi starts on.
    double pdi_speed_mps() const {
        const double rp = constants::moon_mean_radius_m + point.height_m + 15e3;
        const double ra = constants::moon_mean_radius_m + 100e3;
        return std::sqrt(ephemeris.gm_m3ps2(ephemeris::Body::Moon) * (2.0 / rp - 2.0 / (rp + ra)));
    }
    double vacuum_set_max_thrust_n() const {
        const vehicle::Vehicle v(test::hls_with_propellant(0.0), STARSIM_CONFIG_DIR);
        return gnc::set_max_thrust_n(v, gnc::hls_landing_parameters().braking_sets.front());
    }
    // Placed at PDI with the braking range the mission planner computes for this mass.
    void place_at_pdi(double propellant_kg) {
        const double m = 120e3 + propellant_kg;
        const mission::BrakingRangeParameters p;
        const double range = mission::braking_range_m(pdi_speed_mps(), m, vacuum_set_max_thrust_n(), p);
        std::printf("[lunar_descent] PDI %.0f t, %.1f m/s, range %.1f km (fraction %.2f, factor %.2f)\n", m / 1000.0,
                    pdi_speed_mps(), range / 1000.0, p.thrust_fraction, p.range_factor);
        place_at_pdi(propellant_kg, range);
    }
};

struct DescentResult {
    sim::LegTouchdown outcome = sim::LegTouchdown::Impact;
    double offset_m = 0.0, sink_mps = 0.0, lateral_mps = 0.0;
    double m0 = 0.0, m_drop = 0.0, m1 = 0.0;  // at PDI, at the first set drop (off the vacuum Raptors), landed
    double braking_s = 0.0, seconds = 0.0;
};

// Flies the descent placed in `fx` (optionally failing a running vacuum Raptor `fail_after_s` after PDI).
DescentResult fly_from_pdi(Fixture& fx, double fail_after_s = -1.0) {
    DescentResult r;
    r.m0 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    REQUIRE(fx.session.begin_powered_descent(fx.point, fx.azimuth));
    r.braking_s = -1.0;
    bool failed = fail_after_s < 0.0;
    // 0.1 s steps: epoch() + tick_s alone rounds below one tick and advance_to then does nothing
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
        const auto* g = fx.session.lunar_landing_guidance();
        const double mass = fx.session.body(0).vehicle().mass_properties().mass_kg;
        if (r.m_drop == 0.0 && g && g->braking_groups() != gnc::hls_landing_parameters().braking_sets.front()) {
            r.m_drop = mass;
        }
        if (r.braking_s < 0.0 && g && g->phase() != gnc::LunarDescentPhase::Braking) {
            r.braking_s = fx.session.epoch() - fx.t0;
        }
        if (!failed && fx.session.epoch() - fx.t0 >= fail_after_s) {
            const auto& engines = fx.session.body(0).vehicle().stages().front().engines;
            size_t rvac = engines.size();
            for (size_t k = 0; k < engines.size() && rvac == engines.size(); ++k) {
                if (engines[k].group_id == "hls/vacuum" && engines[k].producing_thrust()) rvac = k;
            }
            REQUIRE(rvac < engines.size());
            fx.session.fail_engine(0, 0, rvac);  // as test_lunar_ascent.cpp's engine-out case
            failed = true;
        }
    }
    fx.session.advance_to(fx.session.epoch() + 5.0, 2000);
    const auto& td = fx.session.lunar_touchdown();
    REQUIRE(td);
    r.outcome = td->outcome;
    r.offset_m = td->offset_m;
    r.sink_mps = td->sink_mps;
    r.lateral_mps = td->lateral_mps;
    r.m1 = fx.session.body(0).vehicle().mass_properties().mass_kg;
    r.seconds = fx.session.epoch() - fx.t0;
    return r;
}

// Descent delta-v: braking on the vacuum Raptors (380 s) to the first set drop, the rest at 350 s (sea_level_1; the
// landing thrusters' 340 s is within the rounding).
double descent_delta_v_mps(const DescentResult& r) {
    const double g0 = constants::standard_gravity_mps2;
    const double m_drop = r.m_drop > 0.0 ? r.m_drop : r.m1;
    return 380.0 * g0 * std::log(r.m0 / m_drop) + 350.0 * g0 * std::log(m_drop / r.m1);
}

void report(const char* label, const Fixture& fx, const DescentResult& r) {
    std::printf("[lunar_descent] %s: %s, offset %.2f m, sink %.2f m/s, lateral %.2f m/s, braking %.0f s, total %.0f s, "
                "%.1f -> %.1f (drop) -> %.1f t, %.0f m/s (%.0f m/s at 350 s), %s\n",
                label, sim::to_string(r.outcome), r.offset_m, r.sink_mps, r.lateral_mps, r.braking_s, r.seconds,
                r.m0 / 1000.0, r.m_drop / 1000.0, r.m1 / 1000.0, descent_delta_v_mps(r),
                350.0 * constants::standard_gravity_mps2 * std::log(r.m0 / r.m1), fx.session.lunar_status().c_str());
}
}  // namespace

TEST_CASE("The HLS brakes from PDI at the real mass and lands within 1 m",
          "[sim][lunar_descent_flight][terrain][data][slow]") {
    for (const double propellant : {test::hls_pdi_propellant_kg, test::hls_pdi_light_propellant_kg}) {
        Fixture fx;
        fx.place_at_pdi(propellant);
        const auto r = fly_from_pdi(fx);
        char label[32];
        std::snprintf(label, sizeof(label), "%.0f t at PDI", r.m0 / 1000.0);
        report(label, fx, r);
        CAPTURE(propellant);
        CHECK(fx.session.lunar_step() == sim::LunarStep::Landed);
        CHECK(r.outcome == sim::LegTouchdown::Landed);
        CHECK(r.offset_m < 1.0);  // spec
        CHECK(r.sink_mps <= 1.0 + 0.1);
        CHECK(descent_delta_v_mps(r) < 2400.0);  // spec ~2.0 km/s incl. the approach; the crash runs burnt ~3.5 km/s
    }
}

TEST_CASE("A vacuum-Raptor failure early in the braking phase at the real mass: the sea-level Raptors land the HLS",
          "[sim][lunar_descent_flight][terrain][data][slow]") {
    // Review focus 3 of the mass correction (untested since l4 Task 6): one RVac fails 30 s after PDI.
    Fixture fx;
    fx.place_at_pdi(test::hls_pdi_propellant_kg);
    const auto r = fly_from_pdi(fx, 30.0);
    report("engine out 30 s after PDI", fx, r);
    CHECK(r.outcome == sim::LegTouchdown::Landed);
    CHECK(r.offset_m < 1.0);
    CHECK(descent_delta_v_mps(r) < 2600.0);
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
