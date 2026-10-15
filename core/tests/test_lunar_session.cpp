// FlightSession near the Moon (lunar l1): ticks near the lunar ground, LVLH about the Moon.

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

struct Fixture {
    bool spice = (test::spice_session(), true);  // kernels before the members that read them
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

config::VehicleConfig ship_config() {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    return cfg;
}

// A flat global grid at `h` m (DN = 2h) with LDEM_16's layout.
environment::CylindricalGrid flat_global(double h) {
    environment::CylindricalGrid g;
    g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, static_cast<std::int16_t>(2.0 * h))};
    g.pixels_per_degree = 16.0;
    g.zero_line = 1439.5;
    g.zero_sample = -0.5;
    return g;
}

// Upright (body +x along the local vertical) at rest `height_m` above a selenographic point.
vehicle::RigidBodyState upright_above_moon(const environment::ReferenceBody& moon, time::Epoch t, double lat,
                                           double lon, double height_m) {
    const auto frame = moon.frame(t);
    const Vec3 fixed = frames::geodetic_to_fixed({lat, lon, height_m}, moon.shape());
    const Vec3 up = frame.transform.rotation * math::normalized(fixed);
    const Vec3 y = math::normalized(cross(up, Vec3{0.0, 0.0, 1.0}));
    const Vec3 z = cross(up, y);
    math::Mat3 axes;
    const Vec3 columns[3] = {up, y, z};
    for (int c = 0; c < 3; ++c) {
        axes(0, c) = columns[c].x;
        axes(1, c) = columns[c].y;
        axes(2, c) = columns[c].z;
    }
    vehicle::RigidBodyState s;
    s.position_gcrf_m = frame.position_gcrf(fixed);
    s.velocity_gcrf_mps = frame.velocity_gcrf(fixed, {});
    s.attitude = math::Quat::from_matrix(axes);
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(frame.rotation_gcrf_radps);
    return s;
}

}  // namespace

TEST_CASE("A session ticks a body falling towards the lunar ground instead of coasting through it", "[sim][lunar]") {
    Fixture fx;
    // At rest relative to the surface 3 km up (below the 5 km tick height): free fall, ~98 m/s at the ground.
    const auto frame = fx.moon.frame(fx.t0);
    const Vec3 fixed = frames::geodetic_to_fixed({20.0 * deg, 40.0 * deg, 3000.0}, fx.moon.shape());
    vehicle::RigidBodyState s{frame.position_gcrf(fixed), frame.velocity_gcrf(fixed, {}), {}, {}};
    fx.session.place_body(ship_config(), s, fx.t0);
    CHECK(fx.session.reference_body(0) == ephemeris::Body::Moon);
    fx.session.advance_to(fx.t0 + 120.0, 100000);
    CHECK(fx.session.body(0).impacted());
    // The wreck stays where it struck, on the moving Moon (not 100 km under the ground, not left behind in space).
    CHECK(std::abs(fx.session.telemetry(0).altitude_m) < 50.0);
}

TEST_CASE("LVLH hold in lunar orbit is about the Moon", "[sim][lunar]") {
    Fixture fx;
    const auto rel = orbit::state_from_elements({constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0},
                                                fx.moon.gm_m3ps2());
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    const math::StateVector state{rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps};
    vehicle::RigidBodyState s{state.position_m, state.velocity_mps, gnc::lvlh_attitude_gcrf(rel), {}};
    fx.session.place_body(ship_config(), s, fx.t0);
    fx.session.set_attitude_mode(gnc::lvlh_hold());
    fx.session.advance_to(fx.t0 + 1200.0, 1000000);
    const auto& b = fx.session.body(0).state();
    const auto now = fx.moon.centre_gcrf(fx.session.epoch());
    const Vec3 v_moon = math::normalized(b.velocity_gcrf_mps - now.velocity_mps);
    const Vec3 nose = b.attitude.rotate(Vec3{1.0, 0.0, 0.0});
    // LVLH identity: nose along the Moon-relative velocity (an Earth-relative LVLH would be tens of degrees off).
    CHECK(std::acos(std::clamp(dot(nose, v_moon), -1.0, 1.0)) < 2.0 * deg);
    CHECK(fx.session.telemetry(0).reference_body == ephemeris::Body::Moon);
}

TEST_CASE("A session without a Moon orientation keeps the Earth as the only reference body", "[sim][lunar]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
    vehicle::RigidBodyState s{moon.position_m + Vec3{2.0e6, 0.0, 0.0}, moon.velocity_mps, {}, {}};
    session.place_body(ship_config(), s, t0);
    CHECK(session.reference_body(0) == ephemeris::Body::Earth);
    CHECK(session.environment().moon == nullptr);
}

TEST_CASE("Lunar terrain is refused once a body rests on the Moon", "[sim][lunar]") {
    Fixture fx;
    const auto cfg = ship_config();
    const double cg = vehicle::Vehicle(cfg, STARSIM_CONFIG_DIR).mass_properties().cg_station_m;
    fx.session.place_body(cfg, upright_above_moon(fx.moon, fx.t0, 20.0 * deg, 40.0 * deg, cg + 0.5), fx.t0);
    fx.session.advance_to(fx.t0 + 30.0, 100000);
    REQUIRE(fx.session.body(0).resting());
    auto terrain = std::make_shared<const environment::LunarTerrain>(
        flat_global(0.0), std::vector<environment::PolarStereographicTile>{}, constants::moon_mean_radius_m);
    CHECK_THROWS_AS(fx.session.set_moon_terrain(terrain), std::runtime_error);
}
