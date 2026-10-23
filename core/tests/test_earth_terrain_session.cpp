// Earth terrain in the flight session (M8 Plan A, Task 4): ground contact, leg landings and the crew escape's water on
// a synthetic EarthTerrain (no data/); without terrain the flat model is unchanged.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "return_scenario.hpp"
#include "starsim/config/crew_escape_config.hpp"
#include "starsim/config/geoid_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/environment/earth_terrain.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/crew_escape.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/leg_landing_sim.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using environment::EarthTerrain;
using environment::GeographicGrid;
using environment::Plateau;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double a = constants::wgs84_semi_major_axis_m;
constexpr double geoid_n = -25.0;  // synthetic undulation everywhere
constexpr double landing_propellant_kg = 1000.0;

struct Env {
    Env() { test::spice_session(); }
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
};

Env& env() {
    static Env e;
    return e;
}

time::Epoch t0() { return env().time.from_utc({2026, 10, 13, 12, 0, 0.0}); }

sim::LaunchSite site(const std::string& id) {
    return sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, id));
}

// A global 3 x 4 grid (90 deg nodes) of one ellipsoidal height.
GeographicGrid constant_global(double value_m, double dn_scale_m) {
    GeographicGrid g;
    g.grid = {3, 4, dn_scale_m, std::vector<std::int16_t>(12, static_cast<std::int16_t>(std::lround(value_m / dn_scale_m)))};
    g.north_latitude_deg = 90.0;
    g.west_longitude_deg = -180.0;
    g.step_latitude_deg = 90.0;
    g.step_longitude_deg = 90.0;
    g.global_longitude = true;
    return g;
}

environment::Geoid constant_geoid(double n_m) { return environment::Geoid(constant_global(n_m, 0.01)); }

// n x n nodes, step_deg apart, centred on (lat, lon), all `value_m`.
GeographicGrid box(double lat_deg, double lon_deg, int n, double step_deg, double value_m, double dn_scale_m) {
    GeographicGrid g;
    g.grid = {n, n, dn_scale_m, std::vector<std::int16_t>(static_cast<size_t>(n) * n,
                                                          static_cast<std::int16_t>(std::lround(value_m / dn_scale_m)))};
    g.north_latitude_deg = lat_deg + 0.5 * (n - 1) * step_deg;
    g.west_longitude_deg = lon_deg - 0.5 * (n - 1) * step_deg;
    g.step_latitude_deg = step_deg;
    g.step_longitude_deg = step_deg;
    return g;
}

double lon_deg_of_metres_east(double metres, double lat_rad) { return metres / (a * deg * std::cos(lat_rad)); }

Plateau plateau_at(const sim::LaunchSite& s, double height_m, double radius_m, double apron_m) {
    return {s.id, s.latitude_rad, s.longitude_rad, height_m, radius_m, apron_m};
}

// Falcon 9 first stage (dry + 1 t), upright, its bottom point (legs stowed) or its deployed feet `clear_m` above the
// site-plane point `east_m` east of the site, sinking at `sink_mps`.
void drop_falcon(sim::FlightSession& session, double clear_m, double sink_mps, double east_m = 0.0, bool feet = false) {
    const auto cfg = test::booster_config("falcon9_block5", landing_propellant_kg);
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double base = feet ? cfg.stages.front().landing_legs.foot_station_m : 0.0;
    session.place_body(cfg, test::state_over_site(session.launch_site(), env().earth, t0(), {east_m, 0.0, clear_m + cg - base},
                                                  {0.0, 0.0, -sink_mps}),
                       t0());
}

// Geodetic position of a body's bottom point (station 0).
frames::GeodeticPosition bottom_geodetic(const sim::FlightSession& session, size_t index) {
    const auto& b = session.body(index);
    const double cg = b.vehicle().mass_properties().cg_station_m;
    const auto t = env().earth.fixed_to_gcrf(session.epoch());
    const Vec3 bottom = b.state().position_gcrf_m - b.state().attitude.rotate(Vec3{cg, 0.0, 0.0});
    return frames::fixed_to_geodetic(transpose(t.rotation) * bottom);
}

frames::GeodeticPosition body_geodetic(const sim::FlightSession& session, size_t index) {
    const auto t = env().earth.fixed_to_gcrf(session.epoch());
    return frames::fixed_to_geodetic(transpose(t.rotation) * session.body(index).state().position_gcrf_m);
}

double tilt_from_vertical_deg(const sim::FlightSession& session, size_t index) {
    const auto g = body_geodetic(session, index);
    const auto t = env().earth.fixed_to_gcrf(session.epoch());
    const Vec3 up = t.rotation * Vec3{std::cos(g.latitude_rad) * std::cos(g.longitude_rad),
                                      std::cos(g.latitude_rad) * std::sin(g.longitude_rad), std::sin(g.latitude_rad)};
    const Vec3 axis = session.body(index).state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
    return std::acos(std::clamp(dot(axis, up), -1.0, 1.0)) / deg;
}

// --- crew escape helpers (as test_crew_escape_session.cpp) ---

const config::CrewEscapeConfig& crew() {
    static const auto c = config::load_crew_escape(STARSIM_CONFIG_DIR, "starship_crew");
    return c;
}

config::VehicleConfig ship_with(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    auto& ship = cfg.stages[0];
    config::fill_tanks(ship, propellant_kg);
    for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    return cfg;
}

vehicle::RigidBodyState belly_state(const sim::FlightSession& session, const config::VehicleConfig& cfg,
                                    const Vec3& offset_enu_m, const Vec3& velocity_enu_mps) {
    auto state = test::state_over_site(session.launch_site(), env().earth, t0(), offset_enu_m, velocity_enu_mps);
    const auto t = env().earth.fixed_to_gcrf(t0());
    const Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), 80.0 * deg, 0.0,
                                                    cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    return state;
}

bool all_down(const sim::FlightSession& session) {
    const auto* s = session.crew_escape_state();
    if (!s || s->members.empty()) return false;
    for (const auto& m : s->members) {
        if (m.outcome == sim::CrewOutcome::Flying) return false;
        if (m.body_index && !m.touchdown_enu_m) return false;
    }
    return session.body(0).impacted() || session.body(0).resting();
}

}  // namespace

TEST_CASE("vehicle rests on the plateau height", "[session][earth_terrain]") {
    const auto pad = site("ccsfs_lz40");  // no tower
    const double plateau_m = pad.height_m + 7.0;  // differs from the flat site height on purpose
    auto terrain = std::make_shared<const EarthTerrain>(constant_geoid(geoid_n), constant_global(5.0, 1.0),
                                                        std::vector<GeographicGrid>{}, std::vector<GeographicGrid>{},
                                                        std::vector<Plateau>{plateau_at(pad, plateau_m, 150.0, 50.0)});
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    session.set_earth_terrain(terrain);
    REQUIRE(session.earth_terrain() == terrain.get());
    session.set_launch_site(pad);
    drop_falcon(session, 7.1, 1.0, 30.0);  // 30 m east of the centre, 0.1 m above the plateau
    session.advance_to(session.epoch() + 5.0);
    REQUIRE(session.body(0).resting());
    CHECK_FALSE(session.body(0).impacted());
    CHECK(bottom_geodetic(session, 0).height_m == Catch::Approx(plateau_m).margin(1e-3));
    const double cg = session.body(0).vehicle().mass_properties().cg_station_m;
    CHECK(session.telemetry(0).altitude_terrain_m == Catch::Approx(cg).margin(0.01));
}

TEST_CASE("impact on terrain away from the site", "[session][earth_terrain]") {
    const auto pad = site("ccsfs_lz40");  // no tower
    const double hill_m = 300.0;
    const double hill_east_m = 20000.0;
    const double hill_lat_deg = pad.latitude_rad / deg;
    const double hill_lon_deg = pad.longitude_rad / deg + lon_deg_of_metres_east(hill_east_m, pad.latitude_rad);
    // a 2 km hill (21 x 21 nodes of 0.001 deg) at 300 m, 20 km east of the pad; land at 5 m elsewhere
    auto terrain = std::make_shared<const EarthTerrain>(
        constant_geoid(geoid_n), constant_global(5.0, 1.0), std::vector<GeographicGrid>{},
        std::vector<GeographicGrid>{box(hill_lat_deg, hill_lon_deg, 21, 0.001, hill_m, 0.1)},
        std::vector<Plateau>{plateau_at(pad, pad.height_m, 150.0, 50.0)});
    REQUIRE(terrain->height_ellipsoid_m(hill_lat_deg * deg, hill_lon_deg * deg) == Catch::Approx(hill_m));
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    session.set_earth_terrain(terrain);
    session.set_launch_site(pad);
    drop_falcon(session, 600.0, 50.0, hill_east_m);  // ~570 m above the ellipsoid there (the Earth curves away)
    session.advance_to(session.epoch() + 30.0);
    REQUIRE(session.body(0).impacted());
    const auto bottom = bottom_geodetic(session, 0);
    CAPTURE(bottom.height_m);
    CHECK(bottom.height_m > hill_m - 2.0);  // struck the hill (one tick of penetration), not the pad height
    CHECK(bottom.height_m < hill_m + 1.0);
}

TEST_CASE("crew escape touchdown over water from terrain", "[session][earth_terrain][crew_escape]") {
    const auto starbase = site("starbase_olp1");
    // sea everywhere (orthometric -10 m: water at the geoid) except a small pad plateau
    auto terrain = std::make_shared<const EarthTerrain>(constant_geoid(geoid_n), constant_global(geoid_n - 10.0, 1.0),
                                                        std::vector<GeographicGrid>{}, std::vector<GeographicGrid>{},
                                                        std::vector<Plateau>{plateau_at(starbase, starbase.height_m, 20.0, 10.0)});
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    session.set_earth_terrain(terrain);
    session.set_launch_site(starbase);
    const Vec3 to_tower{std::sin(315.0 * deg), std::cos(315.0 * deg), 0.0};
    const auto cfg = ship_with(20000.0);  // below min_catch_propellant_kg: the give-up fires at once
    session.place_entering_ship(cfg, belly_state(session, cfg, to_tower * -300.0 + Vec3{0, 0, 1200}, {0.0, 0.0, -95.0}),
                                t0());
    session.set_crew_escape(crew());
    const auto start = session.epoch();
    while (session.epoch() - start < 600.0 && !all_down(session)) session.advance_to(session.epoch() + 0.5, 1000000);
    const auto& s = *session.crew_escape_state();
    REQUIRE(s.fired);
    int water = 0;
    for (const auto& m : s.members) {
        REQUIRE(m.body_index);
        REQUIRE(m.touchdown_enu_m);
        const auto g = body_geodetic(session, *m.body_index);
        CAPTURE(g.latitude_rad, g.longitude_rad, m.touchdown_enu_m->x, m.touchdown_enu_m->y);
        CHECK(m.touchdown_over_water == terrain->water(g.latitude_rad, g.longitude_rad));
        water += m.touchdown_over_water ? 1 : 0;
    }
    CHECK(water > 0);
    REQUIRE(s.ship_impact_enu_m);
    const auto ship = body_geodetic(session, session.controlled_index());
    CHECK(s.ship_impact_over_water == terrain->water(ship.latitude_rad, ship.longitude_rad));
}

TEST_CASE("leg landing on a sloped terrain field", "[session][earth_terrain][legs]") {
    const auto lz = site("ccsfs_lz40");
    // 1.5 deg rising towards the east. From 2 deg on the stage creeps downhill at ~0.054 m/s on its feet (the leg
    // contact's regularised friction; a PlaneField the same) - above the 0.05 m/s rest speed, never judged "gelandet".
    const double slope_deg = 1.5;
    const double slope = slope_deg * deg;
    // 101 x 101 nodes of 0.0002 deg (~20 m, 2 km square) centred on the pad: h = pad + tan(slope) x metres east
    const int n = 101;
    const double step_deg = 0.0002, dn_scale = 0.01;
    const double step_east_m = a * step_deg * deg * std::cos(lz.latitude_rad);
    GeographicGrid slope_grid = box(lz.latitude_rad / deg, lz.longitude_rad / deg, n, step_deg, 0.0, dn_scale);
    for (int l = 0; l < n; ++l) {
        for (int s = 0; s < n; ++s) {
            const double h = lz.height_m + std::tan(slope) * (s - n / 2) * step_east_m;
            slope_grid.grid.dn[static_cast<size_t>(l) * n + s] = static_cast<std::int16_t>(std::lround(h / dn_scale));
        }
    }
    auto terrain = std::make_shared<const EarthTerrain>(constant_geoid(geoid_n), constant_global(lz.height_m, 0.01),
                                                        std::vector<GeographicGrid>{},
                                                        std::vector<GeographicGrid>{std::move(slope_grid)},
                                                        std::vector<Plateau>{});
    const auto cfg = test::booster_config("falcon9_block5", landing_propellant_kg);
    const double radius = cfg.stages.front().landing_legs.footprint_radius_m;
    // the same drop on the terrain (the session's own leg site) and on the reference plane of the same slope
    auto land = [&](bool on_terrain, double& tilt_deg) {
        sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
        if (on_terrain) session.set_earth_terrain(terrain);
        session.set_launch_site(lz);
        if (on_terrain) {
            REQUIRE(session.leg_landing().surface().field != nullptr);
        } else {
            contact::LandingSurface plane;
            plane.field = std::make_shared<contact::PlaneField>(slope, 90.0 * deg);
            session.set_landing_surface(lz.latitude_rad, lz.longitude_rad, lz.height_m, plane);
        }
        drop_falcon(session, radius * std::tan(slope) + 0.3, 1.0, 0.0, true);  // feet clear of the uphill ground
        session.set_leg_deployment(0, 1.0);
        const auto start = session.epoch();
        while (session.leg_touchdown(0) == sim::LegTouchdown::None && session.epoch() - start < 20.0) {
            session.advance_to(session.epoch() + 0.1);
        }
        tilt_deg = tilt_from_vertical_deg(session, 0);
        std::printf("[earth terrain] F9 on a %.1f deg %s: %s, tilt %.3f deg\n", slope_deg, on_terrain ? "terrain" : "plane",
                    sim::to_string(session.leg_touchdown(0)), tilt_deg);
        CHECK(session.body(0).resting());
        return session.leg_touchdown(0);
    };
    double terrain_tilt_deg = 0.0, plane_tilt_deg = 0.0;
    CHECK(land(true, terrain_tilt_deg) == sim::LegTouchdown::Landed);
    CHECK(land(false, plane_tilt_deg) == sim::LegTouchdown::Landed);
    // leans with the slope (less by the crushed uphill core; flat model: upright), as on the plane
    CHECK(terrain_tilt_deg > 0.5);
    CHECK(terrain_tilt_deg == Catch::Approx(plane_tilt_deg).margin(0.02));
}

TEST_CASE("flat model unchanged without terrain", "[session][earth_terrain]") {
    const auto lz = site("ccsfs_lz40");
    auto terrain = std::make_shared<const EarthTerrain>(constant_geoid(geoid_n), constant_global(30.0, 1.0),
                                                        std::vector<GeographicGrid>{}, std::vector<GeographicGrid>{},
                                                        std::vector<Plateau>{});
    sim::FlightSession plain(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    sim::FlightSession toggled(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    CHECK(plain.earth_terrain() == nullptr);
    toggled.set_earth_terrain(terrain);
    toggled.set_earth_terrain(nullptr);
    CHECK(toggled.earth_terrain() == nullptr);
    CHECK(toggled.earth_terrain_shared() == nullptr);
    for (auto* s : {&plain, &toggled}) {
        s->set_launch_site(lz);
        drop_falcon(*s, 0.1, 1.0);
        s->advance_to(s->epoch() + 3.0);
        REQUIRE(s->body(0).resting());
        CHECK(s->body(0).ground_height_m() == lz.height_m);
        CHECK(bottom_geodetic(*s, 0).height_m == Catch::Approx(lz.height_m).margin(1e-6));
        CHECK(s->telemetry(0).altitude_terrain_m == s->telemetry(0).altitude_m);
        CHECK(s->leg_landing().surface().field == nullptr);
        CHECK(s->leg_landing().surface().sea.has_value() == lz.sea.has_value());
    }
    const auto& p = plain.body(0).state();
    const auto& q = toggled.body(0).state();
    CHECK(p.position_gcrf_m.x == q.position_gcrf_m.x);
    CHECK(p.position_gcrf_m.y == q.position_gcrf_m.y);
    CHECK(p.position_gcrf_m.z == q.position_gcrf_m.z);
    CHECK(p.velocity_gcrf_mps.x == q.velocity_gcrf_mps.x);
    CHECK(p.velocity_gcrf_mps.y == q.velocity_gcrf_mps.y);
    CHECK(p.velocity_gcrf_mps.z == q.velocity_gcrf_mps.z);
    CHECK(p.attitude.w == q.attitude.w);
    CHECK(p.attitude.x == q.attitude.x);
    CHECK(p.attitude.y == q.attitude.y);
    CHECK(p.attitude.z == q.attitude.z);
}

TEST_CASE("sea height uses the coarse geoid without terrain", "[session][earth_terrain]") {
    const auto starbase = site("starbase_olp1");
    const auto coarse = config::load_coarse_geoid(STARSIM_CONFIG_DIR);
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    const double n = coarse.undulation_m(starbase.latitude_rad, starbase.longitude_rad);
    CHECK(n < -20.0);  // EGM2008 at Starbase: about -27 m
    CHECK(session.sea_height_m(starbase.latitude_rad, starbase.longitude_rad) == n);
    CHECK(session.geoid().undulation_m(starbase.latitude_rad, starbase.longitude_rad) == n);
    // with terrain: its fine geoid
    session.set_earth_terrain(std::make_shared<const EarthTerrain>(constant_geoid(geoid_n), constant_global(5.0, 1.0),
                                                                   std::vector<GeographicGrid>{},
                                                                   std::vector<GeographicGrid>{}, std::vector<Plateau>{}));
    CHECK(session.sea_height_m(starbase.latitude_rad, starbase.longitude_rad) == Catch::Approx(geoid_n));
    CHECK(session.geoid().undulation_m(starbase.latitude_rad, starbase.longitude_rad) == n);  // the coarse grid stays
}
