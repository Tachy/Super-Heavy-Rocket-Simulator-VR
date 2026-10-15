// Lunar ascent (l4): a landed HLS lifts off on the landing thrusters, ascends on PEG into 15 x 100 km and circularises
// into the 100 km LLO (< 1 km); a surface stay under time warp; a vacuum-Raptor engine-out during PEG; abort to orbit
// from 1 km, its refusal without propellant, and the automatic abort when the touchdown point is out of reach.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
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

config::VehicleConfig hls(double propellant_kg, double dry_kg = 0.0) {
    return test::hls_with_propellant(propellant_kg, dry_kg);
}

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 6, 0, 0, 0.0});
    std::shared_ptr<const environment::LunarTerrain> terrain;
    sim::LunarSite site;
    Fixture() {
        terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
        session.set_moon_terrain(terrain);
        site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
        session.set_lunar_landing_site(site);
    }
    const environment::LunarTerrain* session_terrain() const { return terrain.get(); }
    // Places the HLS upright over the site-frame point (east, north) with the foot over the highest ground
    // `clearance_m` above it, moving at `velocity_enu_mps` relative to the Moon (the l2 helper of test_hls_landing.cpp).
    void place_over_site(const config::VehicleConfig& cfg, double clearance_m, const Vec3& velocity_enu_mps) {
        const auto& legs = session.leg_landing();
        const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
        const double cg = probe.mass_properties().cg_station_m;
        const auto& lc = cfg.stages.front().landing_legs;
        double cg_height = -1e9;
        for (double a : lc.azimuths_rad) {
            const double fe = -lc.footprint_radius_m * std::sin(a), fn = lc.footprint_radius_m * std::cos(a);
            cg_height = std::max(cg_height, legs.surface().below(fe, fn).height_m + (cg - lc.foot_station_m));
        }
        const Vec3 cg_site{0.0, 0.0, cg_height + clearance_m};
        const auto frame = moon.frame(t0);
        const Vec3 fixed = legs.origin_fixed_m() + legs.site_to_fixed() * cg_site;
        const Vec3 r = frame.position_gcrf(fixed);
        const Vec3 v = frame.velocity_gcrf(fixed, legs.site_to_fixed() * velocity_enu_mps);
        const math::Mat3 site_to_gcrf = frame.transform.rotation * legs.site_to_fixed();
        const auto attitude =
            math::Quat::from_matrix(site_to_gcrf * contact::from_columns({0, 0, 1}, {0, 1, 0}, {-1, 0, 0}));
        session.place_body(cfg, {r, v, attitude, {}}, t0);
    }
    // Sets the HLS down at the site centre 0.3 m above the ground and lets the legs judge it "gelandet".
    void land(const config::VehicleConfig& cfg) {
        place_over_site(cfg, 0.3, {});
        for (int i = 0; i < 20 * 200 && session.leg_touchdown(0) != sim::LegTouchdown::Landed; ++i) {
            session.advance_to(session.epoch() + 0.1, 100);
        }
        REQUIRE(session.leg_touchdown(0) == sim::LegTouchdown::Landed);
    }
    double altitude_m() const {
        const auto& s = session.body(0).state();
        const auto c = ephemeris.state_gcrf(ephemeris::Body::Moon, session.epoch());
        return norm(s.position_gcrf_m - c.position_m) - constants::moon_mean_radius_m;
    }
    double mass_kg() const { return session.body(0).vehicle().mass_properties().mass_kg; }
    // Low and high altitude over two revolutions of the LLO.
    std::pair<double, double> llo_apsides() {
        double low = 1e12, high = -1e12;
        for (int i = 0; i < 2 * 355; ++i) {
            session.advance_to(session.epoch() + 20.0, 100000);
            low = std::min(low, altitude_m());
            high = std::max(high, altitude_m());
        }
        return {low, high};
    }
};

// Flies the powered descent from PDI until the CG is `height_m` above the touchdown point (or the step changes).
bool descend_to(Fixture& fx, double height_m, const mission::TouchdownPoint& point) {
    const double target_r = constants::moon_mean_radius_m + point.height_m + height_m;
    for (int i = 0; i < 1200 * 10; ++i) {
        if (fx.session.lunar_step() != sim::LunarStep::PoweredDescent) return false;
        const auto c = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.session.epoch());
        if (norm(fx.session.body(0).state().position_gcrf_m - c.position_m) <= target_r) return true;
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
    }
    return false;
}

mission::TouchdownPoint touchdown_point(const Fixture& fx) {
    return *mission::select_touchdown_point(*fx.session_terrain(), fx.site.config.latitude_rad,
                                            fx.site.config.longitude_rad, 180.0 * deg, 12.0, {});
}
}  // namespace

TEST_CASE("A released landed body flies again and the leg judgement starts over", "[sim][lunar_ascent][terrain][data]") {
    Fixture fx;
    fx.land(hls(542e3));
    REQUIRE(fx.session.body(0).resting());
    REQUIRE(fx.session.plan_lunar_ascent({}, 0.0));
    fx.session.advance_to(fx.session.epoch() + 3.0, 2000);
    CHECK_FALSE(fx.session.body(0).resting());
    CHECK(fx.session.leg_touchdown(0) == sim::LegTouchdown::None);
    CHECK(fx.session.lunar_step() == sim::LunarStep::Ascent);
}

TEST_CASE("A lunar ascent is refused when the vehicle is not landed or lacks the propellant",
          "[sim][lunar_ascent][terrain][data]") {
    Fixture fx;
    fx.place_over_site(hls(542e3), 50.0, {});  // hovering 50 m up, not landed
    CHECK_FALSE(fx.session.plan_lunar_ascent({}, 0.0));
    CHECK(fx.session.lunar_status().find("nicht gelandet") != std::string::npos);
    Fixture dry;
    dry.land(hls(60e3));  // ~180 t: ~1.2 km/s, far short of the ~1.9 km/s to orbit
    CHECK_FALSE(dry.session.plan_lunar_ascent({}, 0.0));
    CHECK(dry.session.lunar_status().find("Aufstieg") != std::string::npos);
    CHECK(dry.session.lunar_step() != sim::LunarStep::Ascent);
}

TEST_CASE("After a 3-day stay under time warp the HLS ascends into the 100 km LLO",
          "[sim][lunar_ascent][terrain][data][slow]") {
    Fixture fx;
    fx.land(hls(542e3));  // the LLO variant's landing mass ~662 t
    const auto fixed0 = fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m);
    REQUIRE(fx.session.plan_lunar_ascent({}, 3.0 * 86400.0));
    // Review focus 5: one warp call over the stay - still landed, no drift on the Moon.
    fx.session.advance_to(fx.session.epoch() + 3.0 * 86400.0 - 60.0, 50'000'000);
    CHECK(fx.session.body(0).resting());
    CHECK(fx.session.leg_touchdown(0) == sim::LegTouchdown::Landed);
    CHECK(norm(fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m) - fixed0) < 0.01);
    const double m0 = fx.mass_kg();
    fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
    const double m1 = fx.mass_kg();
    const auto [low, high] = fx.llo_apsides();
    const double dv = 380.0 * constants::standard_gravity_mps2 * std::log(m0 / m1);
    std::printf("[lunar_ascent] LLO %.2f x %.2f km, %.1f t used (%.0f m/s at 380 s), %s\n", low / 1000.0, high / 1000.0,
                (m0 - m1) / 1000.0, dv, fx.session.lunar_status().c_str());
    CHECK(low > 99e3);  // spec: the LLO within 1 km
    CHECK(high < 101e3);
    CHECK(dv == Catch::Approx(1900.0).margin(200.0));  // spec ~1.9 km/s
}

TEST_CASE("A vacuum-Raptor failure 30 s into PEG: the sea-level Raptors finish the ascent into the LLO",
          "[sim][lunar_ascent][engine_out][terrain][data][slow]") {
    Fixture fx;
    fx.land(hls(542e3));
    const double m0 = fx.mass_kg();
    REQUIRE(fx.session.plan_lunar_ascent({}, 0.0));
    double peg_start = -1.0;
    for (int i = 0; i < 600 * 10 && fx.session.lunar_step() == sim::LunarStep::Ascent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
        const auto* g = fx.session.lunar_ascent_guidance();
        if (peg_start < 0.0 && g && g->phase() == gnc::LunarAscentPhase::Peg) peg_start = fx.session.epoch() - fx.t0;
        if (peg_start >= 0.0 && fx.session.epoch() - fx.t0 >= peg_start + 30.0) break;
    }
    REQUIRE(peg_start >= 0.0);
    const auto& engines = fx.session.body(0).vehicle().stages().front().engines;
    size_t rvac = engines.size();
    for (size_t i = 0; i < engines.size(); ++i) {
        if (engines[i].group_id == "hls/vacuum" && engines[i].producing_thrust()) rvac = std::min(rvac, i);
    }
    REQUIRE(rvac < engines.size());
    fx.session.fail_engine(0, 0, rvac);
    fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
    REQUIRE(fx.session.lunar_step() == sim::LunarStep::InLlo);
    const double m1 = fx.mass_kg();
    const auto [low, high] = fx.llo_apsides();
    std::printf("[lunar_ascent] engine out 30 s into PEG: LLO %.2f x %.2f km, %.1f t used, %s\n", low / 1000.0,
                high / 1000.0, (m0 - m1) / 1000.0, fx.session.lunar_status().c_str());
    CHECK(low > 99e3);
    CHECK(high < 101e3);
}

TEST_CASE("An abort 1 km above the touchdown point reaches orbit", "[sim][lunar_ascent][abort][terrain][data][slow]") {
    Fixture fx;
    const auto point = touchdown_point(fx);
    test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, 1065e3);
    REQUIRE(fx.session.begin_powered_descent(point, 180.0 * deg));
    REQUIRE(descend_to(fx, 1000.0, point));
    const double m0 = fx.mass_kg();
    REQUIRE(fx.session.abort_landing(0));
    CHECK(fx.session.lunar_step() == sim::LunarStep::AbortAscent);
    fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::InLlo);
    std::printf("[lunar_ascent] abort from 1 km: %.1f t used, altitude %.1f km, %s\n", (m0 - fx.mass_kg()) / 1000.0,
                fx.altitude_m() / 1000.0, fx.session.lunar_status().c_str());
    CHECK(fx.altitude_m() > 90e3);
}

TEST_CASE("An abort without the propellant for orbit is refused and the landing continues",
          "[sim][lunar_ascent][abort][terrain][data][slow]") {
    Fixture fx;
    const auto point = touchdown_point(fx);
    // Review focus 3: a heavy dry mass leaves ~20 t of propellant at 1 km - enough to land, far too little for orbit.
    test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, 330e3, 420e3);
    REQUIRE(fx.session.begin_powered_descent(point, 180.0 * deg));
    REQUIRE(descend_to(fx, 1000.0, point));
    CHECK_FALSE(fx.session.abort_landing(0));
    CHECK(fx.session.lunar_status().find("Abbruch nicht möglich") != std::string::npos);
    CHECK(fx.session.lunar_step() == sim::LunarStep::PoweredDescent);
}

TEST_CASE("A touchdown point out of reach ends in the automatic abort to orbit",
          "[sim][lunar_ascent][abort][terrain][data][slow]") {
    Fixture fx;
    const auto point = touchdown_point(fx);
    // PDI only 60 km before the point at orbital speed (the braking phase needs ~450 km): the approach cannot be
    // planned. (A point 30 km off the track is still reachable on the Raptors.)
    test::place_hls_at_pdi(fx.session, fx.ephemeris, fx.moon_me, point, fx.t0, 1065e3, 0.0, 60e3);
    REQUIRE(fx.session.begin_powered_descent(point, 180.0 * deg));
    bool aborted = false;
    for (int i = 0; i < 1200 * 10 && fx.session.lunar_step() == sim::LunarStep::PoweredDescent; ++i) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100);
    }
    aborted = fx.session.lunar_step() == sim::LunarStep::AbortAscent;
    std::printf("[lunar_ascent] PDI 60 km short: step %s, %s\n", sim::to_string(fx.session.lunar_step()),
                fx.session.lunar_status().c_str());
    REQUIRE(aborted);
    fx.session.advance_to(fx.session.epoch() + 3.0 * 3600.0, 50'000'000);
    CHECK(fx.session.lunar_step() == sim::LunarStep::InLlo);
}
