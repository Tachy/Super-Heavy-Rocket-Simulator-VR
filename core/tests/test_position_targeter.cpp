// Fixed-time position targeting in the full force model: a known manoeuvre is recovered from the point it reaches.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/environment/gravity.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("Position targeter recovers a known manoeuvre in the full force model", "[mission][targeter]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const mission::PositionTargeter targeter{gravity};
    const auto t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Earth);
    const auto parking = orbit::state_from_elements({6578137.0, 0.0, 0.497, 1.2, 0.0, 0.0}, gm);

    // Truth: a 3.1 km/s TLI-like burn at t0 + 600 s, the point reached 20 h later; first guess ~110 m/s off (the
    // callers start from a two-body or patched-conic solution).
    const auto burn_epoch = t0 + 600.0;
    const auto at_burn = targeter.coast(parking, t0, burn_epoch);
    const math::Vec3 truth_dv = 3100.0 * normalized(at_burn.velocity_mps) + math::Vec3{0.0, 0.0, 50.0};
    math::StateVector kicked = at_burn;
    kicked.velocity_mps += truth_dv;
    const auto arrival_epoch = burn_epoch + 20.0 * 3600.0;
    const auto arrival = targeter.coast(kicked, burn_epoch, arrival_epoch);

    const auto solution = targeter.solve(parking, t0, burn_epoch, {arrival.position_m, arrival_epoch},
                                         3000.0 * normalized(at_burn.velocity_mps), 0.01);
    REQUIRE(solution.converged);
    CHECK(solution.miss_m < 0.01);
    CHECK(norm(solution.burn.delta_v_gcrf_mps - truth_dv) < 1e-4);
    CHECK(solution.burn.epoch == burn_epoch);
}

TEST_CASE("Position targeter reports an unreachable point instead of diverging", "[mission][targeter]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    const mission::PositionTargeter targeter{gravity};
    const auto t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const auto parking = orbit::state_from_elements({6578137.0, 0.0, 0.497, 1.2, 0.0, 0.0},
                                                    ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    // 1e10 m away in 60 s: no finite solution within the iteration budget.
    const auto solution = targeter.solve(parking, t0, t0, {{1e10, 0.0, 0.0}, t0 + 60.0}, {}, 10.0, 8);
    CHECK_FALSE(solution.converged);
    CHECK(std::isfinite(solution.miss_m));
}
