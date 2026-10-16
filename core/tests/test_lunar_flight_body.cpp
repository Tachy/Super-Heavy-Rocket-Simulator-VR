// FlightBody near the Moon (lunar l1): the reference body decides ground, air and telemetry; the state stays GCRF.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;
using vehicle::FlightBody;
using vehicle::RigidBodyState;

namespace {

constexpr double deg = constants::deg_to_rad;
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

struct LunarFixture {
    bool spice = (test::spice_session(), true);  // kernels before the members that read them
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    environment::Us76Atmosphere atmosphere;
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, &ephemeris, false, &moon};
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

std::unique_ptr<FlightBody> ship(const RigidBodyState& state) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    return std::make_unique<FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
}

// Upright (body +x along the local vertical) at a selenographic point, `sink_mps` downwards relative to the surface.
RigidBodyState above_moon(const environment::ReferenceBody& moon, time::Epoch t, double lat, double lon,
                          double height_m, double sink_mps) {
    const auto frame = moon.frame(t);
    const Vec3 fixed = frames::geodetic_to_fixed({lat, lon, height_m}, moon.shape());
    const Vec3 up_fixed = math::normalized(fixed);
    const Vec3 up = frame.transform.rotation * up_fixed;
    const Vec3 y = math::normalized(cross(up, Vec3{0.0, 0.0, 1.0}));
    const Vec3 z = cross(up, y);
    math::Mat3 axes;
    for (int i = 0; i < 3; ++i) {
        axes(i, 0) = i == 0 ? up.x : i == 1 ? up.y : up.z;
        axes(i, 1) = i == 0 ? y.x : i == 1 ? y.y : y.z;
        axes(i, 2) = i == 0 ? z.x : i == 1 ? z.y : z.z;
    }
    RigidBodyState s;
    s.position_gcrf_m = frame.position_gcrf(fixed);
    s.velocity_gcrf_mps = frame.velocity_gcrf(fixed, -sink_mps * up_fixed);
    s.attitude = math::Quat::from_matrix(axes);
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(frame.rotation_gcrf_radps);
    return s;
}

// 100 km circular polar orbit about the Moon (Moon-centred elements in GCRF axes; inclination to GCRF z).
RigidBodyState lunar_orbit(const environment::ReferenceBody& moon, time::Epoch t) {
    const auto rel = orbit::state_from_elements(
        {constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0}, moon.gm_m3ps2());
    const auto centre = moon.centre_gcrf(t);
    return {rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps, {}, {}};
}

}  // namespace

TEST_CASE("A ship touching down gently on the Moon rests and follows the Moon for a day", "[vehicle][lunar]") {
    LunarFixture fx;
    // The state is the CG's: put the bottom 0.5 m above the ground.
    const double cg = ship(lunar_orbit(fx.moon, fx.t0))->vehicle().mass_properties().cg_station_m;
    auto body = ship(above_moon(fx.moon, fx.t0, -85.0 * deg, 30.0 * deg, cg + 0.5, 1.0));
    time::Epoch t = fx.t0;
    for (int i = 0; i < 2000 && !body->resting(); ++i, t = t + 0.005) {
        body->step(fx.env, t, 0.005, {});
    }
    REQUIRE(body->resting());
    CHECK_FALSE(body->impacted());
    REQUIRE(body->touchdown_sink_mps().has_value());
    CHECK(*body->touchdown_sink_mps() < 2.0);
    const auto tm0 = body->telemetry(fx.env, t);
    CHECK(tm0.reference_body == ephemeris::Body::Moon);
    CHECK(tm0.latitude_rad == Catch::Approx(-85.0 * deg).margin(1e-6));
    CHECK(tm0.longitude_rad == Catch::Approx(30.0 * deg).margin(1e-6));
    const Vec3 fixed0 = fx.moon.frame(t).to_fixed(body->state().position_gcrf_m);
    body->coast(fx.env, t, 86400.0);
    t = t + 86400.0;
    CHECK(body->resting());
    CHECK(body->reference_body(fx.env, t) == ephemeris::Body::Moon);
    CHECK(norm(fx.moon.frame(t).to_fixed(body->state().position_gcrf_m) - fixed0) < 1e-3);
    CHECK(body->telemetry(fx.env, t).speed_relative_mps < 1e-6);
}

TEST_CASE("A ship falling hard onto the Moon impacts", "[vehicle][lunar]") {
    LunarFixture fx;
    const double cg = ship(lunar_orbit(fx.moon, fx.t0))->vehicle().mass_properties().cg_station_m;
    auto body = ship(above_moon(fx.moon, fx.t0, 10.0 * deg, -20.0 * deg, cg + 2.0, 20.0));
    time::Epoch t = fx.t0;
    for (int i = 0; i < 1000 && !body->impacted(); ++i, t = t + 0.005) body->step(fx.env, t, 0.005, {});
    CHECK(body->impacted());
}

TEST_CASE("Telemetry near the Moon is relative to the Moon, also before the first step", "[vehicle][lunar]") {
    LunarFixture fx;
    auto body = ship(lunar_orbit(fx.moon, fx.t0));
    const auto tm = body->telemetry(fx.env, fx.t0);  // no step yet
    CHECK(tm.reference_body == ephemeris::Body::Moon);
    CHECK(tm.altitude_m == Catch::Approx(100e3).margin(1.0));
    CHECK(tm.periapsis_altitude_m == Catch::Approx(100e3).margin(1.0));
    CHECK(tm.apoapsis_altitude_m == Catch::Approx(100e3).margin(1.0));
    CHECK(tm.speed_inertial_mps == Catch::Approx(norm(body->state().velocity_gcrf_mps)));  // GCRF, unchanged meaning
    CHECK(tm.speed_relative_mps == Catch::Approx(1633.0).margin(10.0));  // relative to the rotating Moon
    CHECK(tm.dynamic_pressure_pa == 0.0);  // vacuum
    CHECK(std::abs(tm.vertical_speed_mps) < 0.5);
}

TEST_CASE("Telemetry in Earth orbit reports orbit elements about the Earth", "[vehicle][lunar]") {
    LunarFixture fx;
    const auto rel = orbit::state_from_elements({6378137.0 + 200e3, 0.0, 51.6 * deg, 0.0, 0.0, 0.0},
                                                fx.ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    auto body = ship({rel.position_m, rel.velocity_mps, {}, {}});
    const auto tm = body->telemetry(fx.env, fx.t0);
    CHECK(tm.reference_body == ephemeris::Body::Earth);
    CHECK(tm.periapsis_altitude_m == Catch::Approx(200e3).margin(1.0));
    CHECK(tm.apoapsis_altitude_m == Catch::Approx(200e3).margin(1.0));
    // Inclination to the true equator of date (ITRF z): within the ~0.2 deg of precession-nutation of GCRF's.
    CHECK(tm.inclination_rad == Catch::Approx(51.6 * deg).margin(0.3 * deg));
}

TEST_CASE("The reference body switches with hysteresis at 60,000 / 100,000 km from the Moon", "[vehicle][lunar]") {
    LunarFixture fx;
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    const Vec3 out = math::normalized(-centre.position_m);  // from the Moon towards the Earth
    auto at = [&](double distance_m) {
        return RigidBodyState{centre.position_m + distance_m * out, centre.velocity_mps, {}, {}};
    };
    auto body = ship(at(101.0e6));
    body->coast(fx.env, fx.t0, 1.5);  // records the Earth
    body->set_state(at(80.0e6));
    CHECK(body->reference_body(fx.env, fx.t0 + 1.5) == ephemeris::Body::Earth);  // came from the Earth: band keeps it
    body->set_state(at(59.0e6));
    body->coast(fx.env, fx.t0, 1.5);  // a coast (or step) records the choice
    CHECK(body->reference_body(fx.env, fx.t0 + 1.5) == ephemeris::Body::Moon);
    body->set_state(at(90.0e6));
    CHECK(body->reference_body(fx.env, fx.t0 + 1.5) == ephemeris::Body::Moon);  // inside the hysteresis band
    body->set_state(at(101.0e6));
    CHECK(body->reference_body(fx.env, fx.t0 + 1.5) == ephemeris::Body::Earth);
    // Without a Moon in the environment everything is relative to the Earth (all callers before l1).
    vehicle::FlightEnvironment earth_only{fx.gravity, fx.atmosphere, fx.earth, &fx.ephemeris, false};
    body->set_state(at(10.0e6));
    CHECK(body->reference_body(earth_only, fx.t0) == ephemeris::Body::Earth);
}

TEST_CASE("Separated parts keep the reference body", "[vehicle][lunar]") {
    LunarFixture fx;
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    auto stack = std::make_unique<FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir),
                                              lunar_orbit(fx.moon, fx.t0));
    stack->coast(fx.env, fx.t0, 2.0);  // records the Moon
    // Inside the hysteresis band (80,000 km): only the recorded choice keeps the Moon.
    const auto centre = fx.moon.centre_gcrf(fx.t0 + 2.0);
    stack->set_state({centre.position_m + Vec3{80.0e6, 0.0, 0.0}, centre.velocity_mps, {}, {}});
    auto upper = stack->separate(1, 0.5);
    CHECK(stack->reference_body(fx.env, fx.t0 + 2.0) == ephemeris::Body::Moon);
    CHECK(upper->reference_body(fx.env, fx.t0 + 2.0) == ephemeris::Body::Moon);
}

TEST_CASE("A body placed inside the hysteresis band reports the Moon before its first step", "[vehicle][lunar]") {
    LunarFixture fx;
    // An NRHO near its apolune (~70,000 km): undecided bodies take the Moon within the release distance.
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    auto body = ship({centre.position_m + Vec3{0.0, 0.0, -70.0e6}, centre.velocity_mps, {}, {}});
    CHECK(body->reference_body(fx.env, fx.t0) == ephemeris::Body::Moon);
    CHECK(body->telemetry(fx.env, fx.t0).reference_body == ephemeris::Body::Moon);
}

TEST_CASE("No skip-out is judged at the Moon", "[vehicle][lunar]") {
    LunarFixture fx;
    // Through 80 km and back above 100 km above the Moon: Earth's atmosphere events must not fire.
    auto body = ship(above_moon(fx.moon, fx.t0, 0.0, 0.0, 70e3, -400.0));  // climbing 400 m/s from 70 km
    time::Epoch t = fx.t0;
    for (int i = 0; i < 30000; ++i, t = t + 0.005) body->step(fx.env, t, 0.005, {});  // 150 s: ~112 km
    CHECK(body->telemetry(fx.env, t).altitude_m > 100e3);
    CHECK_FALSE(body->skipped_out());
}

TEST_CASE("Ground clearance is measured over the reference body", "[vehicle][lunar]") {
    LunarFixture fx;
    auto body = ship(above_moon(fx.moon, fx.t0, 45.0 * deg, 45.0 * deg, 3000.0, 12.0));
    const auto c = body->ground_clearance(fx.env, fx.t0);
    CHECK(c.height_m == Catch::Approx(3000.0).margin(0.01));  // of the CG (placed 3 km up); lunar ground height 0
    CHECK(c.sink_mps == Catch::Approx(12.0).margin(0.01));
}

TEST_CASE("A body with fixed legs and no leg site keeps its own ground contact on the Moon", "[vehicle][lunar][legs]") {
    LunarFixture fx;
    auto hls = [&](const RigidBodyState& s) {
        return std::make_unique<FlightBody>(
            std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, "starship_hls"), config_dir), s);
    };
    const double cg = hls(lunar_orbit(fx.moon, fx.t0))->vehicle().mass_properties().cg_station_m;
    auto body = hls(above_moon(fx.moon, fx.t0, -85.0 * deg, 30.0 * deg, cg + 0.5, 1.0));
    REQUIRE(body->vehicle().leg_deployment() == 1.0);
    time::Epoch t = fx.t0;
    for (int i = 0; i < 2000 && !body->resting() && !body->impacted(); ++i, t = t + 0.005) {
        body->step(fx.env, t, 0.005, {});
    }
    CHECK((body->resting() || body->impacted()));  // not fallen through the ground
}

TEST_CASE("The truth air density factor scales the Earth's air density and free path", "[vehicle][lunar]") {
    LunarFixture fx;
    const auto plain = environment::ReferenceBody::earth(fx.earth, fx.atmosphere, constants::earth_gm_m3ps2);
    auto dense = plain;
    dense.set_air_density_factor(1.15);
    const auto a = plain.air(60e3), b = dense.air(60e3);
    CHECK(b.density_kg_m3 == Catch::Approx(1.15 * a.density_kg_m3));
    CHECK(b.mean_free_path_m == Catch::Approx(a.mean_free_path_m / 1.15));
    CHECK(b.pressure_pa == a.pressure_pa);  // engine back-pressure is not dispersed

    vehicle::FlightEnvironment env = fx.env;
    env.air_density_factor = 1.15;
    CHECK(vehicle::reference_body_of(env, ephemeris::Body::Earth).air(60e3).density_kg_m3 ==
          Catch::Approx(b.density_kg_m3));
    CHECK(vehicle::reference_body_of(fx.env, ephemeris::Body::Earth).air(60e3).density_kg_m3 ==
          Catch::Approx(a.density_kg_m3));
}
