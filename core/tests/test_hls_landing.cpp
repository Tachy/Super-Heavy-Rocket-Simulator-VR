// The HLS on the Moon (lunar l2): standing on its legs at a lunar site, judged by the leg simulation; away from the
// site by its own ground contact; drop tests on slopes and on the LOLA terrain.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/leg_landing_sim.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double site_lat = -84.7906 * deg, site_lon = 29.1957 * deg;  // config/lunar_sites.toml

struct Fixture {
    bool spice = (test::spice_session(), true);  // kernels before the members that read them
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

// The HLS with `propellant_kg` in its tanks (split at O/F 3.6), engines off.
config::VehicleConfig hls(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = propellant_kg * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    return cfg;
}

// Places the HLS upright over the site-frame point (east, north) with the foot over the highest ground `clearance_m`
// above it, moving at `velocity_enu_mps` relative to the Moon. Site frame and surface of the session's leg simulation.
void place_over_site(Fixture& fx, const config::VehicleConfig& cfg, double clearance_m, const Vec3& velocity_enu_mps,
                     double east_m = 0.0, double north_m = 0.0) {
    const auto& legs = fx.session.leg_landing();
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const auto& lc = cfg.stages.front().landing_legs;
    double cg_height = -1e9;
    for (double a : lc.azimuths_rad) {
        // feet at body (y, z) = r (cos a, sin a) (make_leg_geometry); body y = north, body z = west
        const double fe = east_m - lc.footprint_radius_m * std::sin(a), fn = north_m + lc.footprint_radius_m * std::cos(a);
        cg_height = std::max(cg_height, legs.surface().below(fe, fn).height_m + (cg - lc.foot_station_m));
    }
    const Vec3 cg_site{east_m, north_m, cg_height + clearance_m};
    const auto frame = fx.moon.frame(fx.t0);
    const Vec3 fixed = legs.origin_fixed_m() + legs.site_to_fixed() * cg_site;
    const Vec3 r = frame.position_gcrf(fixed);
    const Vec3 v = frame.velocity_gcrf(fixed, legs.site_to_fixed() * velocity_enu_mps);
    // body x up, body y north, body z west (right-handed)
    const math::Mat3 site_to_gcrf = frame.transform.rotation * legs.site_to_fixed();
    const auto attitude = math::Quat::from_matrix(site_to_gcrf * contact::from_columns({0, 0, 1}, {0, 1, 0}, {-1, 0, 0}));
    fx.session.place_body(cfg, {r, v, attitude, {}}, fx.t0);
}

sim::LegSite lunar_site_on_sphere() { return {ephemeris::Body::Moon, site_lat, site_lon, 0.0, 10000.0}; }

// Runs a placed drop until the legs' judgement (or 60 s).
sim::LegTouchdown run_drop(Fixture& fx) {
    const auto t0 = fx.session.epoch();
    while (fx.session.leg_touchdown(0) == sim::LegTouchdown::None && fx.session.epoch() - t0 < 60.0) {
        fx.session.advance_to(fx.session.epoch() + 0.1, 100000);
    }
    return fx.session.leg_touchdown(0);
}

// Drop on a plane of `slope_deg` rising towards `uphill_azimuth_deg`: the foot over the highest ground 0.5 m up,
// 1 m/s down (spec l4: touchdown <= 1 m/s), with `propellant_kg` in the tanks.
sim::LegTouchdown drop_on_slope(double slope_deg, double uphill_azimuth_deg, double propellant_kg = 100.0e3) {
    Fixture fx;
    contact::LandingSurface surface;
    surface.field = std::make_shared<contact::PlaneField>(slope_deg * deg, uphill_azimuth_deg * deg);
    fx.session.set_landing_surface(lunar_site_on_sphere(), surface);
    place_over_site(fx, hls(propellant_kg), 0.5, {0.0, 0.0, -1.0});
    const auto result = run_drop(fx);
    double stroke = 0.0;
    for (double c : fx.session.leg_crushed_m(0)) stroke = std::max(stroke, c);
    std::printf("[hls] %3.0f t propellant, slope %4.1f deg uphill %5.1f deg: %-10s crushed %.3f m\n",
                propellant_kg / 1e3, slope_deg, uphill_azimuth_deg, sim::to_string(result), stroke);
    return result;
}

// Static tip-over slope across the weakest edge (between two of six legs): atan(r cos 30 deg / CG height above the feet).
double tip_over_slope_deg(const config::VehicleConfig& cfg) {
    const vehicle::Vehicle v(cfg, STARSIM_CONFIG_DIR);
    const auto& lc = cfg.stages.front().landing_legs;
    const double h = v.mass_properties().cg_station_m - lc.foot_station_m;
    return std::atan(lc.footprint_radius_m * std::cos(30.0 * deg) / h) / deg;
}

}  // namespace

TEST_CASE("An HLS set down gently at a lunar site lands and follows the Moon", "[hls][legs][lunar]") {
    Fixture fx;
    fx.session.set_landing_surface(lunar_site_on_sphere(), contact::LandingSurface{});
    place_over_site(fx, hls(100.0e3), 0.3, {0.0, 0.0, -0.5});
    fx.session.advance_to(fx.t0 + 30.0, 100000);
    REQUIRE(fx.session.leg_touchdown(0) == sim::LegTouchdown::Landed);
    const auto fixed0 = fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m);
    fx.session.advance_to(fx.session.epoch() + 3600.0, 100000);
    const auto fixed1 = fx.moon.frame(fx.session.epoch()).to_fixed(fx.session.body(0).state().position_gcrf_m);
    CHECK(norm(fixed1 - fixed0) < 1e-3);
    CHECK(fx.session.body(0).resting());
    CHECK(fx.session.telemetry(0).reference_body == ephemeris::Body::Moon);
}

TEST_CASE("An HLS touching the Moon far from the landing site is judged by its own ground contact", "[hls][legs][lunar]") {
    Fixture fx;
    fx.session.set_landing_surface(lunar_site_on_sphere(), contact::LandingSurface{});
    place_over_site(fx, hls(100.0e3), 50.0, {0.0, 0.0, -20.0}, 50000.0, 0.0);  // 50 km east, falling
    fx.session.advance_to(fx.t0 + 60.0, 100000);
    CHECK(fx.session.body(0).impacted());  // not through the ground
    CHECK(std::abs(fx.session.telemetry(0).altitude_m) < 30.0);
    CHECK(fx.session.leg_touchdown(0) == sim::LegTouchdown::None);
}

TEST_CASE("An HLS without a lunar landing site rests on the Moon by its own ground contact", "[hls][legs][lunar]") {
    Fixture fx;  // the session's leg simulation sits on the Earth
    const auto frame = fx.moon.frame(fx.t0);
    const Vec3 up = frames::geodetic_to_fixed({site_lat, site_lon, 0.0}, fx.moon.shape());
    const auto cfg = hls(100.0e3);
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const Vec3 fixed = up + math::normalized(up) * (cg + 1.0);
    const Vec3 r = frame.position_gcrf(fixed);
    const Vec3 v = frame.velocity_gcrf(fixed, -math::normalized(up) * 1.0);
    const Vec3 x = frame.transform.rotation * math::normalized(up);
    const Vec3 y = math::normalized(cross(x, Vec3{0.0, 0.0, 1.0}));
    const auto attitude = math::Quat::from_matrix(contact::from_columns(x, y, cross(x, y)));
    fx.session.place_body(cfg, {r, v, attitude, {}}, fx.t0);
    fx.session.advance_to(fx.t0 + 30.0, 100000);
    CHECK(!fx.session.body(0).impacted());
    CHECK(fx.session.body(0).resting());
    CHECK(std::abs(fx.session.telemetry(0).altitude_m - cg) < 2.0);  // CG over the sphere: standing on its skirt
}

// Slopes up to NASA's Artemis landing-slope limit (~8-10 deg) land; the leg sizing is in starship_hls.toml.
TEST_CASE("HLS drop tests on 0, 5 and 8 deg slopes land; 25 deg tips over", "[hls][legs][lunar]") {
    const double tip = tip_over_slope_deg(hls(100.0e3));
    std::printf("[hls] static tip-over slope across an edge: %.1f deg\n", tip);
    REQUIRE(tip > 25.0);  // the leg sizing (user 2026-10-15: 12 m footprint)
    for (double slope : {0.0, 5.0, 8.0}) {
        CHECK(drop_on_slope(slope, 0.0) == sim::LegTouchdown::Landed);   // uphill towards a leg
        CHECK(drop_on_slope(slope, 30.0) == sim::LegTouchdown::Landed);  // uphill between two legs (weakest)
    }
    CHECK(drop_on_slope(25.0, 30.0) == sim::LegTouchdown::TippedOver);
}

TEST_CASE("HLS drop tests at the l4 landing masses (503 t and 662 t)", "[hls][legs][lunar]") {
    // l4: the HLS lands with 380-540 t of propellant (NRHO / LLO variant); the legs sized in l2 at 221 t must still
    // land 1 m/s + 0.5 m on 0 / 5 / 8 deg and tip over at 25 deg.
    for (const double propellant_kg : {383e3, 542e3}) {
        CAPTURE(propellant_kg);
        for (double slope : {0.0, 5.0, 8.0}) {
            CAPTURE(slope);
            CHECK(drop_on_slope(slope, 0.0, propellant_kg) == sim::LegTouchdown::Landed);
            CHECK(drop_on_slope(slope, 30.0, propellant_kg) == sim::LegTouchdown::Landed);
        }
        CHECK(drop_on_slope(25.0, 30.0, propellant_kg) == sim::LegTouchdown::TippedOver);
    }
}

TEST_CASE("An HLS lands on the LOLA terrain at the flattest spot near the site", "[hls][legs][lunar][terrain][data]") {
    Fixture fx;
    const auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
    fx.session.set_moon_terrain(terrain);
    const auto site = sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain);
    fx.session.set_lunar_landing_site(site);
    // Flattest spot of a 50 m lattice within 500 m: the smallest largest tilt of the terrain normal under the footprint
    // (centre and the six feet) - l4's touchdown-point choice will do this properly.
    const auto& surface = fx.session.leg_landing().surface();
    const auto cfg = hls(100.0e3);
    const double r = cfg.stages.front().landing_legs.footprint_radius_m;
    double best = 1e9, be = 0.0, bn = 0.0;
    for (int i = -10; i <= 10; ++i) {
        for (int j = -10; j <= 10; ++j) {
            double worst = 0.0;
            for (int k = -1; k < 6; ++k) {
                const double a = k * 60.0 * deg;
                const double e = 50.0 * i - (k < 0 ? 0.0 : r * std::sin(a));
                const double n = 50.0 * j + (k < 0 ? 0.0 : r * std::cos(a));
                worst = std::max(worst, std::acos(std::min(1.0, surface.below(e, n).normal.z)));
            }
            if (worst < best) {
                best = worst;
                be = 50.0 * i;
                bn = 50.0 * j;
            }
        }
    }
    const double at_site = std::acos(std::min(1.0, surface.below(0.0, 0.0).normal.z)) / deg;
    std::printf("[hls] site %.1f m above the sphere, slope there %.2f deg; flattest spot (%.0f, %.0f) m, %.2f deg\n",
                site.height_m, at_site, be, bn, best / deg);
    place_over_site(fx, cfg, 0.5, {0.0, 0.0, -1.0}, be, bn);
    CHECK(run_drop(fx) == sim::LegTouchdown::Landed);
    // The feet stand on the terrain: spring compression of millimetres, no crush at 1 m/s
    for (double c : fx.session.leg_crushed_m(0)) CHECK(c < 0.01);
    CHECK(fx.session.body(0).resting());
}

TEST_CASE("Replacing the Moon terrain drops a lunar landing site built on the old one", "[hls][legs][lunar][terrain][data]") {
    Fixture fx;
    auto terrain = sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
    const std::weak_ptr<const environment::LunarTerrain> watch = terrain;
    fx.session.set_moon_terrain(terrain);
    fx.session.set_lunar_landing_site(
        sim::lunar_site(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"), *terrain));
    terrain.reset();
    // The landing surface keeps the terrain it was built on alive (no dangling pointer) ...
    CHECK(!watch.expired());
    CHECK(fx.session.leg_landing().surface().below(0.0, 0.0).height_m == Catch::Approx(0.0).margin(1e-6));
    // ... and a new terrain (here: none) does not leave a lunar site on the old one: the site must be set again.
    fx.session.set_moon_terrain(nullptr);
    CHECK(fx.session.leg_landing().site().body == ephemeris::Body::Earth);
    CHECK(watch.expired());
}
