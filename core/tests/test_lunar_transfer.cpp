// Earth-Moon mission: TLI targeting in the full force model, finite TLI burn, mid-course correction,
// lunar orbit insertion. Plan criterion (M6): after the correction the predicted periselene altitude is
// within 1 km of the target.

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double earth_radius_m = 6378137.0;

struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};  // same model as the flight session
    mission::LunarTransferTargeter targeter{gravity, ephemeris, moon};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    time::Epoch arrival = t0 + 4.0 * 86400.0;
    // Launch window: parking-orbit plane through the Moon's direction at arrival.
    math::StateVector parking = orbit::state_from_elements(
        {earth_radius_m + 200e3, 0.0, 28.5 * deg, targeter.parking_orbit_raan(arrival, 28.5 * deg).value(), 0.0, 0.0},
        ephemeris.gm_m3ps2(ephemeris::Body::Earth));
};

void wait_for_burn(sim::FlightSession& session, double max_s) {
    const auto start = session.epoch();
    while (session.autopilot().state == sim::AutopilotState::Burn && session.epoch() - start < max_s) {
        session.advance_to(session.epoch() + 60.0, 100000);
    }
}

}  // namespace

TEST_CASE("TLI targeting converges in the full force model", "[mission][validation]") {
    test::spice_session();
    Fixture fx;
    mission::LunarArrivalTarget target;
    target.periselene_altitude_m = 100e3;
    target.inclination_rad = 90.0 * deg;
    target.periselene_epoch = fx.arrival;

    const auto burn_epoch = fx.targeter.best_departure(fx.parking, fx.t0, fx.t0 + 1800.0, target.periselene_epoch);
    const auto solution = fx.targeter.solve(fx.parking, fx.t0, burn_epoch, target);
    std::printf("[validation] TLI targeting: %d iterations, dv %.1f m/s, periselene %.3f km, i %.4f deg, "
                "arrival %+.1f s\n", solution.iterations, norm(solution.burn.delta_v_gcrf_mps),
                solution.arrival.periselene_altitude_m / 1000.0, solution.arrival.inclination_rad / deg,
                solution.arrival.periselene_epoch - target.periselene_epoch);
    REQUIRE(solution.converged);
    CHECK(norm(solution.burn.delta_v_gcrf_mps) == Catch::Approx(3150.0).margin(150.0));  // typical TLI
}

TEST_CASE("Earth-Moon mission: finite TLI, mid-course correction, lunar orbit insertion",
          "[mission][validation][lunar]") {
    test::spice_session();
    Fixture fx;
    mission::LunarArrivalTarget target;
    target.periselene_altitude_m = 100e3;
    target.inclination_rad = 90.0 * deg;
    target.periselene_epoch = fx.arrival;

    // Refuelled ship (full tanks) in the parking orbit.
    sim::FlightSession session(fx.ephemeris, fx.earth, STARSIM_CONFIG_DIR);
    session.place_in_orbit("starship_block3", fx.parking, fx.t0, 1);

    const auto burn_epoch = fx.targeter.best_departure(fx.parking, fx.t0, fx.t0 + 1800.0, target.periselene_epoch);
    const auto tli = fx.targeter.solve(fx.parking, fx.t0, burn_epoch, target);
    REQUIRE(tli.converged);
    REQUIRE(session.execute_burn(tli.burn));
    wait_for_burn(session, 7200.0);
    REQUIRE(session.autopilot().state == sim::AutopilotState::Off);

    auto state_now = [&] {
        const auto& s = session.body(0).state();
        return math::StateVector{s.position_gcrf_m, s.velocity_gcrf_mps};
    };
    const auto after_tli = fx.targeter.evaluate(state_now(), session.epoch(), session.epoch(), {});
    std::printf("[validation] after finite TLI (%.0f m/s planned): periselene %.1f km (target 100), arrival %+.0f s\n",
                norm(tli.burn.delta_v_gcrf_mps), after_tli.periselene_altitude_m / 1000.0,
                after_tli.periselene_epoch - target.periselene_epoch);

    // Mid-course correction 12 h after TLI.
    session.advance_to(burn_epoch + 12.0 * 3600.0, 100000);
    const auto mcc = fx.targeter.solve(state_now(), session.epoch(), session.epoch() + 600.0, target, math::Vec3{});
    REQUIRE(mcc.converged);
    REQUIRE(session.execute_burn(mcc.burn));
    wait_for_burn(session, 7200.0);
    const auto after_mcc1 = fx.targeter.evaluate(state_now(), session.epoch(), session.epoch(), {});
    std::printf("[validation] MCC-1 %.2f m/s (%s): predicted periselene %.3f km, i %.3f deg\n",
                norm(mcc.burn.delta_v_gcrf_mps), session.burn_configuration().rcs ? "RCS" : "Raptor",
                after_mcc1.periselene_altitude_m / 1000.0, after_mcc1.inclination_rad / deg);

    // MCC-2 one day before arrival: small enough for RCS translation.
    session.advance_to(after_mcc1.periselene_epoch - 86400.0, 100000);
    const auto mcc2 = fx.targeter.solve(state_now(), session.epoch(), session.epoch() + 600.0, target, math::Vec3{});
    REQUIRE(mcc2.converged);
    REQUIRE(session.execute_burn(mcc2.burn));
    wait_for_burn(session, 7200.0);
    const auto after_mcc = fx.targeter.evaluate(state_now(), session.epoch(), session.epoch(), {});
    std::printf("[validation] MCC-2 %.3f m/s (%s): predicted periselene %.3f km, i %.4f deg\n",
                norm(mcc2.burn.delta_v_gcrf_mps), session.burn_configuration().rcs ? "RCS" : "Raptor",
                after_mcc.periselene_altitude_m / 1000.0, after_mcc.inclination_rad / deg);
    CHECK(std::abs(after_mcc.periselene_altitude_m - target.periselene_altitude_m) < 1000.0);

    // Lunar orbit insertion at periselene.
    const auto loi = mission::LunarTransferTargeter::orbit_insertion(after_mcc, fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon));
    REQUIRE(session.execute_burn(loi));
    wait_for_burn(session, 6.0 * 86400.0);
    REQUIRE(session.autopilot().state == sim::AutopilotState::Off);

    // Flown lunar orbit: altitude range over two hours (one revolution is ~1.97 h at 100 km).
    double low = 1e12, high = -1e12;
    for (int i = 0; i < 240; ++i) {
        session.advance_to(session.epoch() + 30.0, 100000);
        const auto moon = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, session.epoch()).position_m;
        const double altitude = norm(session.body(0).state().position_gcrf_m - moon) - constants::moon_mean_radius_m;
        low = std::min(low, altitude);
        high = std::max(high, altitude);
    }
    std::printf("[validation] LOI %.0f m/s: lunar orbit %.1f x %.1f km, %.0f t propellant left\n",
                norm(loi.delta_v_gcrf_mps), low / 1000.0, high / 1000.0,
                session.body(0).vehicle().propellant_kg() / 1000.0);
    CHECK(low > 80e3);
    CHECK(high < 130e3);
}
