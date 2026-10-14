// FlightSession near the Moon (lunar l1): ticks near the lunar ground, LVLH about the Moon.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
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
