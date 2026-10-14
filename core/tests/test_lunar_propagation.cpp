// Lunar l1 validation: vehicle states stay Earth-centred GCRF near the Moon. A 100 km LLO and an NRHO-like orbit
// (3,000 x 70,000 km about the Moon) propagated one day by FlightBody::coast (Earth-centred, full model) agree with
// a Moon-centred propagation of the same forces (point masses Earth, Moon, Sun; no Earth J2, no SRP) within 1 m [E];
// 200 Hz ticks (RK4, as in a landing) over 20 min within 5 cm [E].
// The same forces in Moon-centred coordinates: the craft's Earth-centred acceleration minus the DE440 Moon's
// acceleration relative to the Earth. A Moon-centred PointMassGravity is a different model: it accelerates its frame
// as point-mass Earth and Sun would pull the Moon, while DE440's Moon also feels the planets, Earth's J2 and tides
// (a few 1e-9 m/s^2) - that alone moves the NRHO-like orbit ~4 m in a day (printed for information).

#include <cmath>
#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/integrators/integrate.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
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
    environment::PointMassGravity earth_centred{ephemeris, earth, {ephemeris::Body::Earth, true, true, true, false}};
    environment::PointMassGravity moon_centred{ephemeris, earth, {ephemeris::Body::Moon, true, true, true, false}};
    environment::Us76Atmosphere atmosphere;
    environment::ReferenceBody moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    vehicle::FlightEnvironment env{earth_centred, atmosphere, earth, &ephemeris, false, &moon};
    time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
};

std::unique_ptr<vehicle::FlightBody> ship(const vehicle::RigidBodyState& state) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    return std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR), state);
}

// DE440 Moon's acceleration relative to the Earth (central difference of its velocity; error ~1e-11 m/s^2).
Vec3 moon_acceleration_mps2(const Fixture& fx, time::Epoch t) {
    constexpr double h = 5.0;
    return (fx.ephemeris.state_gcrf(ephemeris::Body::Moon, t + h).velocity_mps -
            fx.ephemeris.state_gcrf(ephemeris::Body::Moon, t - h).velocity_mps) /
           (2.0 * h);
}

// Moon-centred reference: DOP853 at tight tolerances on the Moon-relative state. `point_mass_frame`: the Moon-centred
// point-mass model instead (information only, see the header).
math::StateVector moon_centred_reference(const Fixture& fx, const math::StateVector& rel, double duration_s,
                                         bool point_mass_frame = false) {
    const integrators::OdeFunction<6> f = [&](double t, const math::OdeState6& y) {
        const Vec3 r{y[0], y[1], y[2]};
        const time::Epoch epoch = fx.t0 + t;
        const Vec3 a = point_mass_frame
                           ? fx.moon_centred.acceleration_mps2(r, epoch)
                           : fx.earth_centred.acceleration_mps2(
                                 r + fx.ephemeris.state_gcrf(ephemeris::Body::Moon, epoch).position_m, epoch) -
                                 moon_acceleration_mps2(fx, epoch);
        return math::OdeState6{{y[3], y[4], y[5], a.x, a.y, a.z}};
    };
    integrators::Dop853Options<6> options;
    options.relative_tolerance = 1e-13;
    options.absolute_tolerance = math::OdeState6{{1e-6, 1e-6, 1e-6, 1e-9, 1e-9, 1e-9}};
    return math::unpack(integrators::integrate<6>(f, 0.0, math::pack(rel), duration_s, options).y);
}

double coast_error_m(Fixture& fx, const orbit::KeplerianElements& elements, double duration_s) {
    const auto rel = orbit::state_from_elements(elements, fx.moon.gm_m3ps2());
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    auto body = ship({rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps, {}, {}});
    body->coast(fx.env, fx.t0, duration_s);
    const auto end_centre = fx.moon.centre_gcrf(fx.t0 + duration_s);
    const auto reference = moon_centred_reference(fx, rel, duration_s);
    const auto point_mass = moon_centred_reference(fx, rel, duration_s, true);
    std::printf("[validation]   (Moon-centred point-mass model, other frame acceleration: %.3f m)\n",
                norm(point_mass.position_m - reference.position_m));
    return norm(body->state().position_gcrf_m - end_centre.position_m - reference.position_m);
}

}  // namespace

TEST_CASE("A low lunar orbit propagated Earth-centred matches the Moon-centred reference", "[lunar][validation]") {
    Fixture fx;
    const double error = coast_error_m(fx, {constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0},
                                       86400.0);
    std::printf("[validation] LLO 100 km, 1 day: Earth-centred vs Moon-centred %.3f m\n", error);
    CHECK(error < 1.0);
}

TEST_CASE("An NRHO-like orbit propagated Earth-centred matches the Moon-centred reference", "[lunar][validation]") {
    Fixture fx;
    const double rp = constants::moon_mean_radius_m + 3000e3, ra = 70000e3;
    const double error =
        coast_error_m(fx, {0.5 * (rp + ra), (ra - rp) / (ra + rp), 90.0 * deg, 0.0, 90.0 * deg, 0.0}, 86400.0);
    std::printf("[validation] NRHO-like 3,000 x 70,000 km, 1 day: Earth-centred vs Moon-centred %.3f m\n", error);
    CHECK(error < 1.0);
}

TEST_CASE("200 Hz ticks in a low lunar orbit match the Moon-centred reference", "[lunar][validation]") {
    Fixture fx;
    const auto rel = orbit::state_from_elements({constants::moon_mean_radius_m + 100e3, 0.0, 90.0 * deg, 0.0, 0.0, 0.0},
                                                fx.moon.gm_m3ps2());
    const auto centre = fx.moon.centre_gcrf(fx.t0);
    auto body = ship({rel.position_m + centre.position_m, rel.velocity_mps + centre.velocity_mps, {}, {}});
    const double duration = 1200.0;
    const int ticks = static_cast<int>(std::lround(duration / 0.005));
    for (int i = 0; i < ticks; ++i) body->step(fx.env, fx.t0 + i * 0.005, 0.005, {});
    const auto reference = moon_centred_reference(fx, rel, duration);
    const double error = norm(body->state().position_gcrf_m - fx.moon.centre_gcrf(fx.t0 + duration).position_m -
                              reference.position_m);
    std::printf("[validation] LLO 100 km, 20 min of 200 Hz ticks: Earth-centred vs Moon-centred %.2e m\n", error);
    CHECK(error < 0.05);
}
