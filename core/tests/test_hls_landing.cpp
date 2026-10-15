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
